/******************************************************************************

  Copyright (c) 2019, Aquantia Corporation
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

   3. Neither the name of the Aquantia Corporation nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

******************************************************************************/

#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <signal.h>
#include <errno.h>

#include <ifaddrs.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/mman.h>
#include <sys/user.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdint.h>
#include <semaphore.h>
#include <pthread.h>
#include <limits.h>
#include <net/if.h>
#include <dirent.h>

#include <linux/version.h>
#include <linux/types.h>
#include <linux/ethtool.h>
#include <linux/sockios.h>

#include "list.h"
#include "atl_private.h"
#include "atl_llh.h"
#include "../kmod/aq_common.h"
#include "../kmod/aq_tsn.h"

#define TSN_CLASS_A_PPS 8000
#define TSN_CLASS_B_PPS 4000

#define HW_ATL_MPI_FW_VERSION	0x18

static int atl_readreg(device_t *dev, u_int32_t reg, u_int32_t *data);
static int atl_writereg(device_t *dev, u_int32_t reg, u_int32_t data);
static avb_log_func ext_avb_log_fnc = NULL;
static const char *avb_log_tag = "AQDEV";

u_int32_t log_level = LOG_LVL_INFO;

void atl_init_avb_log(avb_log_func log_func, const char *tag)
{
	ext_avb_log_fnc = log_func;
	avb_log_tag = tag;
}

void __logprint(u_int32_t lvl, const char *path, int line, const char *msg, ...)
{
 	va_list args;
	va_start(args, msg);
	if( ext_avb_log_fnc ) {
		ext_avb_log_fnc(0, avb_log_tag, "AQ", "ETHERNET", path, line, msg, args);
	} else {
		vprintf(msg, args);
		printf("\n");
	}
	va_end(args);
}

void __dump(u_int32_t lvl, const char *path, int line, const char *name, u_int8_t *buf, u_int32_t len)
{
	u32 i;
	__logprint(lvl, path, line, "%s. Buf 0x%p, length 0x%x", name, buf, len);
	for( i = 0; i * 16 + 15 < len; i++, buf += 16 ) {
		__logprint(lvl, path, line, "%03x : %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x",
			i * 16, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], 
			buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
	}
	if( i*16 < len ) {
		const int MAX_DUMP_LINE_SIZE = 54;
		char buf[MAX_DUMP_LINE_SIZE];
		int offset = 0;
		offset += snprintf(&buf[offset], MAX_DUMP_LINE_SIZE - offset, "%03x :", i*16);
		len -= i*16;
		for( i = 0; i < len; i++ ) {
			if( i == 8 )
				offset += snprintf(&buf[offset], MAX_DUMP_LINE_SIZE - offset, " ");
			offset += snprintf(&buf[offset], MAX_DUMP_LINE_SIZE - offset, " %02x", buf[i]);
		}
		buf[offset] = '\0';
		__logprint(lvl, path, line, "%s", buf);
	}
	__logprint(lvl, path, line, "");
}

int atl_getpagesize()
{
	return 4096*256;//getpagesize();
}
/*********************************************************************
 *  PCI Device ID Table
 *
 *  Used by probe to select devices to load on
 *  Last field stores an index into e1000_strings
 *  Last entry must be all 0s
 *
 *  { Vendor ID, Device ID, SubVendor ID, SubDevice ID, String Index }
 *********************************************************************/
static atl_vendor_info_t atl_vendor_info_array[] = {
	{ PCI_VENDOR_ID_AQUANTIA,		AQ_DEVICE_ID_D107,		AQ_HWREV_2,		&hw_atl_avb_ops_b0, },
	{ PCI_VENDOR_ID_AQUANTIA,		AQ_DEVICE_ID_AQC107,	AQ_HWREV_2,		&hw_atl_avb_ops_b0, },
	{ PCI_VENDOR_ID_AQUANTIA,		AQ_DEVICE_ID_AQC107S,	AQ_HWREV_2,		&hw_atl_avb_ops_b0, },
	{ PCI_VENDOR_ID_AQUANTIA,		AQ_DEVICE_ID_AQC107,	AQ_HWREV_2,		&hw_atl_avb_ops_b0, },
	{ PCI_VENDOR_ID_AQUANTIA,		AQ_DEVICE_ID_0001,		AQ_HWREV_ANY,	&hw_atl2_avb_ops, },
	{ PCI_VENDOR_ID_AQUANTIA,		AQ_DEVICE_ID_AQC113,	AQ_HWREV_ANY,	&hw_atl2_avb_ops, },
	{ PCI_VENDOR_ID_AQUANTIA,		AQ_DEVICE_ID_AQC107,	AQ_HWREV_ANY,	&hw_atl_avb_ops_b0, },
};

/* Atantic driver name */
static const char *atl_driver_name="atl_tsn"; 
static char *pcie_addr_string = "0000:00:00.0"; //Temp address

LIST_HEAD(adapter_list);
static unsigned dev_idx = 0;

static uint32_t getDeviceCount()
{
	unsigned i = 0;
	device_t *dev;
	list_for_each_entry(dev, &adapter_list, list)
		i++;
	return i;
}

/*********************************************************************
 *  Function prototypes
 *********************************************************************/
//static int atl_read_mac_addr(struct atl_adapter] *hw);
static int atl_allocate_pci_resources(struct atl_adapter *adapter);
static void atl_free_pci_resources(struct atl_adapter *adapter);
//static void atl_reset(struct atl_adapter *adapter);
static int  atl_share_state(struct atl_adapter *adapter);

struct atl_share_mem {
	volatile u32 adapter_ref_cntr;
	volatile u32 tx_ring_ref_cntr[MAX_TX_RING_COUNT];
	volatile pid_t tx_ring_occupation_pid[MAX_TX_RING_COUNT];
	volatile u32 rx_ring_ref_cntr[MAX_RX_RING_COUNT];
	volatile pid_t rx_ring_occupation_pid[MAX_RX_RING_COUNT];
 	volatile u_int32_t class_a_bytes_per_second;
	volatile u_int32_t class_b_bytes_per_second;
};
#define ATL_SHARED_MEM (sizeof(pthread_mutex_t) + sizeof(struct atl_share_mem))

static int aq_pci_probe_get_hw_by_id(device_t *dev,
				     const struct aq_avb_hw_ops **ops)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(atl_vendor_info_array); i++) {
		if (atl_vendor_info_array[i].device_id == dev->device &&
		    atl_vendor_info_array[i].vendor_id == dev->vendor &&
		    (atl_vendor_info_array[i].revision == AQ_HWREV_ANY ||
		     atl_vendor_info_array[i].revision == dev->revision)) {
			if( ops ) {
				*ops = atl_vendor_info_array[i].hw_ops;
			}
			break;
		}
	}

	if (i == ARRAY_SIZE(atl_vendor_info_array))
		return -EINVAL;

	return 0;
}

#define MAX_ATTR_LEN 256
static char *read_attr(int dirfd, const char *attr)
{
	static char buf[MAX_ATTR_LEN];
	int len = 0;

	int fd = openat(dirfd, attr, O_RDONLY);
	if (fd == -1)
		goto out;
	len = read(fd, buf, sizeof(buf) - 1);
	close(fd);

out:
	buf[len] = '\0';
	return buf;
}

static unsigned long read_ulong_attr(int dirfd, const char *attr)
{
	return strtoul(read_attr(dirfd, attr), NULL, 0);
}

static int init_attrs(device_t *dev)
{
	int dirfd = dev->class_fd;
	char buf[PATH_MAX];

	//Get driver class name
	int len = readlinkat(dirfd, "device/driver", buf, sizeof(buf) - 1);
	if (len == -1) {
		if( ENOENT != errno ) {
			logprint(LOG_LVL_DEBUG, "Can't access PCI device driver link: %s", strerror(errno));
		}
		return -1;
	}
	buf[len] = '\0';
	char *driver_class = strrchr(buf, '/');
	dev->driver_class = strdup(dev->ifname);
	if (!dev->driver_class) {
		logprint(LOG_LVL_ERROR, "Can't alloc memory: %s", strerror(errno));
		return -1;
	}

	//Get PCIe address and IDs
	len = readlinkat(dirfd, "device", buf, sizeof(buf) - 1);
	if (len == -1) {
		logprint(LOG_LVL_DEBUG, "Can't access PCI dev link: %s", strerror(errno));
		return -1;
	}

	buf[len] = '\0';
	char *bdf  = strrchr(buf, '/');
	
	unsigned domain = 0, bus = 0, pdev = 0, func = 0;
	sscanf(bdf, "/%x:%x:%x.%x", &domain, &bus, &pdev, &func);
	dev->domain = domain;
	dev->bus = bus;
	dev->dev = pdev;
	dev->func = func;

	dev->vendor = read_ulong_attr(dirfd, "device/vendor");
	dev->device = read_ulong_attr(dirfd, "device/device");
	dev->subvend = read_ulong_attr(dirfd, "device/subsystem_vendor");
	dev->subdev = read_ulong_attr(dirfd, "device/subsystem_device");

	return 0;
}

static int get_dev_info_from_sysfs(int cldir_fd, char *ifa_name) {
	int ret = -1;
	int class_fd = openat(cldir_fd, ifa_name, O_RDONLY | O_DIRECTORY);

	if (class_fd == -1) {
		logprint(LOG_LVL_DEBUG, "Can't open device dir %s: %s", ifa_name, strerror(errno));
		goto err_cldir;
	}
	logprint(LOG_LVL_DEBUG, "Check device %s at index %d", ifa_name, dev_idx);

	device_t *dev = calloc(1, sizeof(*dev)); //changed so c++ doesn't complain
	if (!dev) {
		logprint(LOG_LVL_ERROR, "Can't alloc DEVICE_INFO: %s", strerror(errno));
		goto err_alloc;
	}

	dev->class_fd = class_fd;
	dev->ifname = strdup(ifa_name);
	if (!dev->ifname) {
		logprint(LOG_LVL_ERROR, "Can't alloc memory: %s", strerror(errno));
		goto err_attrs;
	}

	if (init_attrs(dev)) {
		if( ENOENT != errno ) {
			logprint(LOG_LVL_DEBUG, "Can't read device %s attrs", strerror(errno));
		}
		goto err_attrs;
	}

	if( aq_pci_probe_get_hw_by_id(dev, NULL) ) {
		logprint(LOG_LVL_DEBUG, "Device doesn't support avb.");
		goto err_attrs;
	}

	logprint(LOG_LVL_DEBUG, "Found device %s at index %d", ifa_name, dev_idx);
	dev->index = dev_idx++;
	list_add(&dev->list, &adapter_list);

	return 0;

err_attrs:
	free(dev);
err_alloc:
	close(class_fd);
err_cldir:
	return ret;
}

static char *class_dir = "/sys/class/net";
static int scan_ifaces()
{
    struct ifaddrs *ifaddr, *ifa;

	int cldir_fd = open(class_dir, O_RDONLY | O_DIRECTORY);
	if (cldir_fd == -1) {
		logprint(LOG_LVL_CRITICAL, "Can't open aqdiag class dir: %s", strerror(errno));
		return -1;
	}

    if (getifaddrs(&ifaddr) == -1) {
        logprint(LOG_LVL_ERROR, "Can't enumerate network interfaces.");
        return EXIT_FAILURE;
    }

    /* Walk through linked list, maintaining head pointer so we
        can free list later */

    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
		get_dev_info_from_sysfs(cldir_fd, ifa->ifa_name);
    }

    freeifaddrs(ifaddr);
    return EXIT_SUCCESS;
}

static int cleanup_iface(device_t *dev)
{
	if (dev->private_data)
	{
		atl_detach(dev);
	}
	logprint(LOG_LVL_DEBUG, "Remove device from list: %d - %s", dev->index, dev->ifname);
	free(dev->ifname);
	free(dev->driver_class);
	close(dev->class_fd);
	free(dev);
}

int atl_probe(device_t *pdev)
{
	uint32_t i, res = -ENXIO;
	device_t *local_dev, *temp_dev;
	logprint(LOG_LVL_VERBOSE, "Find device: %s %04x-%02x.%02x.%d", 
			pdev->ifname ? pdev->ifname : "NO_IFNAME", 
			pdev->domain, pdev->bus, pdev->dev, pdev->func);
	scan_ifaces();
	if (!dev_idx || pdev == NULL)
		return -EINVAL;

	list_for_each_entry_safe(local_dev, temp_dev, &adapter_list, list)
	{
		list_del(&local_dev->list);
		logprint(LOG_LVL_DEBUG, "Check device: %s %04x-%02x.%02x.%d", 
			local_dev->ifname, local_dev->domain, 
			local_dev->bus, local_dev->dev, local_dev->func);

		if( !!res &&
			((pdev->ifname != NULL && pdev->ifname[0] != '\0' && !strncmp(pdev->ifname, local_dev->ifname, IFNAMSIZ)) ||
			 (  pdev->domain == local_dev->domain && 
			 	pdev->bus == local_dev->bus &&
			 	pdev->dev == local_dev->dev &&
				pdev->func == local_dev->func  )) )
		{
			if( pdev->ifname ) free(pdev->ifname);
			*pdev = *local_dev;
			res = 0;
			logprint(LOG_LVL_VERBOSE, "Match device: %s", pdev->ifname);
		}
		else
		{
			cleanup_iface(local_dev);
		}
	}
	return res;
}

int check_access(struct atl_adapter *adapter, int idx, u_int32_t addr)
{
	if (!adapter) {
		logprint(LOG_LVL_CRITICAL, "Device not opened");
		return -1;
	}

	if (idx < -1 || idx > 5)
		return -1;
	struct mmap_res *bar = idx == -1 ? adapter->default_bar : &adapter->bar[idx];

	if (!bar->vaddr) {
		logprint(LOG_LVL_ERROR, "Default BAR not mapped");
		return -1;
	}

	if (addr > bar->size) {
		logprint(LOG_LVL_ERROR, "Address 0x%08x out of range (max 0x%08x)",
		       addr, bar->size);
		return -1;
	}

	return 0;
}

int wr(struct atl_adapter *adapter, u32 addr, u32 val)
{
	if (check_access(adapter, 0, addr) == -1)
		return -1;

	*(volatile u32 *)(((uint8_t *)adapter->bar[0].vaddr) + addr) = val;
	return 0;
}

int rr(struct atl_adapter *adapter, u32 addr, u32 *val)
{
	if (check_access(adapter, 0, addr) == -1 || !val)
		return -1;

	*val = *(volatile u32 *)(((uint8_t *)adapter->bar[0].vaddr) + addr);
	return 0;
}

int wr64(struct atl_adapter *adapter, u32 addr, u64 val)
{
	if (check_access(adapter, 0, addr) == -1)
		return -1;

	*(volatile u64 *)(((uint8_t *)adapter->bar[0].vaddr) + addr) = val;
	return 0;
}

int rr64(struct atl_adapter *adapter, u32 addr, u64 *val)
{
	if (check_access(adapter, 0, addr) == -1 || !val)
		return -1;

	*val = *(volatile u64 *)(((uint8_t *)adapter->bar[0].vaddr) + addr);
	return 0;
}

int atl_attach(device_t *pdev)
{
	struct atl_adapter *adapter;
	struct ifreq ifr;
	//struct aq_tsn_init tsn_init;
	struct atl_link_state link = {0};
	int error = 0;
	bool locked = false;
	u_int32_t hw_rev, spec_rev;
	
	if (pdev == NULL)
		return -EINVAL;

	adapter = (struct atl_adapter *)pdev->private_data;

	if (adapter != NULL) {
		logprint(LOG_LVL_DEBUG, "Adapter is already attached.");
		return -EBUSY;
	}

	/* allocate an adapter */
	pdev->private_data = malloc(sizeof(struct atl_adapter));
	if (pdev->private_data == NULL)
		return -ENOMEM;

	memset(pdev->private_data, 0, sizeof(struct atl_adapter));

	adapter = (struct atl_adapter *)pdev->private_data;
	adapter->class_fd = pdev->class_fd;
	adapter->ifname = pdev->ifname;
	adapter->driver_class = pdev->driver_class;

	INIT_LIST_HEAD(&adapter->memregs);

	adapter->sd = socket(PF_PACKET, SOCK_RAW, 0);
	
	if (adapter->sd < 0) {
		logprint(LOG_LVL_ERROR, "Cannot open socket: %s", strerror(errno));
		error = -ENXIO;
		goto err_prebind;
	}

	if (setsockopt(adapter->sd, SOL_SOCKET, SO_BINDTODEVICE, adapter->ifname, \
					strnlen(pdev->ifname, IFNAMSIZ)) != 0) {
		logprint(LOG_LVL_ERROR, "Cannot bind to device: %s", strerror(errno));
		error = -errno;
		goto err_bind;
	}
	
	if (atl_share_state(adapter) != 0) {
		logprint(LOG_LVL_ERROR, "Cannot create lock: %s", strerror(errno));
		error = -errno;
		goto err_bind;
	}

	adapter->active = 1;

	if (atl_lock(pdev) != 0) {
		logprint(LOG_LVL_ERROR, "Cannot lock device: %s", strerror(errno));
		error = -errno;
		goto err_bind;
	}

	locked = true;

	/*
	 * dev_path should look something "0000:01:00.0"
	 */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_name, pdev->ifname, sizeof(ifr.ifr_name)-1);
	//ifr.ifr_data = &tsn_init;
	if (ioctl(adapter->sd, SIOCINITTSN, &ifr) < 0) {
		logprint(LOG_LVL_ERROR, "Cannot init device: %s", strerror(errno));
		error = -ENXIO;
		goto err_bind;
	}

	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_name, pdev->ifname, sizeof(ifr.ifr_name)-1);
	ifr.ifr_data = (void *)&link;
	/* get current link speed */
	error = ioctl(adapter->sd, SIOCLINKCMD, &ifr);
	if (error < 0){
		logprint(LOG_LVL_ERROR, "Cannot read link state: %s", strerror(errno));
		goto err_pci;
	}
	adapter->link_speed = link.speed;
	adapter->hw_offsets = link.hw_offsets;
	/* Setup PCI resources */
	error = atl_allocate_pci_resources(adapter);
	if (error) {
		logprint(LOG_LVL_ERROR, "Cannot allocate pci resources: %s", strerror(errno));
		goto err_pci;
	}

	error = aq_pci_probe_get_hw_by_id(pdev, &adapter->aq_hw_avb_ops);
	if (error) {
		logprint(LOG_LVL_ERROR, "Cannot get hw by id: %s", strerror(errno));
		goto err_pci;
	}

	error = atl_readreg(pdev, HW_ATL2_GLB_MIF_ID_ADR, &hw_rev);
	if (error) {
		logprint(LOG_LVL_ERROR, "Cannot read hw rev: %s", strerror(errno));
		goto err_pci;
	}

	if( hw_rev & 0x107 == 0x102 ) {
		error = atl_readreg(pdev, HW_ATL_MPI_FW_VERSION, &spec_rev);
		if (error) {
			logprint(LOG_LVL_ERROR, "Cannot read hw rev: %s", strerror(errno));
			goto err_pci;
		}
	} else {
		error = atl_readreg(pdev, HW_ATL2_GLB_FW_ID_ADR, &spec_rev);
		if (error) {
			logprint(LOG_LVL_ERROR, "Cannot read hw rev: %s", strerror(errno));
			goto err_pci;
		}
	}

	adapter->aq_shmem->adapter_ref_cntr++;
	logprint(LOG_LVL_INFO, "Attach to Aquantia %s device. FW ver: %x.", 
			hw_rev & 0x3 == 0x3 ? "AQC113" : "AQC107", spec_rev);
	logprint(LOG_LVL_INFO, "Attached process count %d.",
			adapter->aq_shmem->adapter_ref_cntr);
	/*
	 * Copy the permanent MAC address out of the EEPROM
	 */
	/*
	if (atl_read_mac_addr(&adapter->hw) < 0) {
		error = -EIO;
		goto err_late;
	}
	*/
	adapter->a1 = hw_rev & 0x3 == 0x2; //adapter->aq_hw_avb_ops == &hw_atl_avb_ops_b0;
	adapter->a2 = hw_rev & 0x3 == 0x3; //adapter->aq_hw_avb_ops == &hw_atl2_avb_ops;

	adapter->tx_rings = NULL;
	adapter->rx_rings = NULL;

	if (atl_unlock(pdev) != 0) {
		logprint(LOG_LVL_ERROR, "Cannot unlock device: %s", strerror(errno));
		error = -errno;
		goto err_gen;
	}

	return 0;

//err_late:
err_pci:
	atl_free_pci_resources(adapter);
err_bind:
	if (locked)
		(void) atl_unlock(pdev);
	if (adapter && adapter->memlock) {
		(void) munmap(adapter->memlock, ATL_SHARED_MEM);
		adapter->memlock = NULL;
		adapter->aq_shmem = NULL;
	}
	close(adapter->sd);
err_prebind:
	pdev->private_data = NULL;
	free(adapter);

err_gen:
	return error;
}

static int atl_ring_tx_attach(struct tx_ring *txring, u32 flags)
{
	return 0;
}

static int atl_ring_rx_attach(struct rx_ring *rxring, u32 flags)
{
	return 0;
}

static int atl_ring_tx_detach(struct tx_ring *txring)
{
	return 0;
}

static int atl_ring_rx_detach(struct rx_ring *rxring)
{
	return 0;
}

int atl_attach_tx(device_t *pdev)
{
	int error, i;
	struct atl_adapter *adapter;
	struct tx_ring *txring;
	u32 flags = 0;
	bool flex_filters = false;

	if (pdev == NULL)
		return -EINVAL;

	adapter = (struct atl_adapter *)pdev->private_data;

	if (adapter == NULL)
		return -EINVAL;

	if (atl_lock(pdev) != 0)
		return -errno;

	flags = !adapter->a2 ? TX_RING_FLAG_WRITEBACK_EOP : 0;
	if( adapter->a2 ) {
		u_int32_t data = 0;
		flags |= TX_RING_FLAG_EG_TS;
		atl_readreg(pdev, HW_ATL2_RX_FLR_CONTROL2_ADR, &data);
		flex_filters = !!(data & HW_ATL2_RX_FLR_NEW_RPF_EN_MSK);
	}

	for( i = (adapter->a2 ? MAX_TX_RING_COUNT : 1) - 1; i >= 0 ; i-- ) {
		txring = malloc(sizeof(struct tx_ring));
		if (txring == NULL)
			goto release;

		memset(txring, 0, sizeof(struct tx_ring));
		txring->adapter = adapter;

		/* Allocate and Setup Queues */
		if( flex_filters )
			txring->index = i == 0 ? ATL_AVB_FLEX_RING : ATL_AVB_FLEX_RING2;
		else
			txring->index = i == 0 ? ATL_AVB_RING : ATL2_AVB_RING2;
		if( adapter->aq_shmem->tx_ring_ref_cntr[i] > 0 ) {
			error = adapter->aq_hw_avb_ops->hw_avb_ring_tx_attach(txring, flags);
		} else {
			error = adapter->aq_hw_avb_ops->hw_avb_ring_tx_init(txring, flags);
		}
		if (error)
			goto release;
		adapter->aq_shmem->tx_ring_ref_cntr[i]++;
		txring->next_ring = adapter->tx_rings;
		logprint(LOG_LVL_DEBUG, "Add tx ring %p, prev %p. Ref counter %d.",
				txring, adapter->tx_rings, adapter->aq_shmem->tx_ring_ref_cntr[i]);
		adapter->tx_rings = txring;
	}

	if (atl_unlock(pdev) != 0)
		free(txring);

	return 0;
release:
	if ( txring )
		free(txring);

	if (atl_unlock(pdev) != 0)
		return -errno;

	return error;
}

int atl_attach_rx(device_t *pdev)
{
	int error, i;
	struct atl_adapter *adapter;
	struct rx_ring *rxring;
	u32 flags = 0;
	bool flex_filters = false;

	if (pdev == NULL)
		return -EINVAL;

	adapter = (struct atl_adapter *)pdev->private_data;

	if (adapter == NULL)
		return -EINVAL;

	if (atl_lock(pdev) != 0)
		return -errno;

	if( adapter->a2 ) {
		u_int32_t data = 0;
		flags |= RX_RING_FLAG_ING_TS;
		atl_readreg(pdev, HW_ATL2_RX_FLR_CONTROL2_ADR, &data);
		flex_filters = !!(data & HW_ATL2_RX_FLR_NEW_RPF_EN_MSK);
	}

	for( i = (adapter->a2 ? MAX_RX_RING_COUNT : 1) - 1; i >= 0 ; i-- ) {
		rxring = malloc(sizeof(struct rx_ring));
		if (rxring == NULL)
			goto release;
		
		memset(rxring, 0, sizeof(struct rx_ring));
		rxring->adapter = adapter;
	
		if( flex_filters )
			rxring->index = i == 0 ? ATL_AVB_FLEX_RING : ATL_AVB_FLEX_RING2;
		else
			rxring->index = i == 0 ? ATL_AVB_RING : ATL2_AVB_RING2;
		/* Allocate and Setup Queues */
		if( adapter->aq_shmem->rx_ring_ref_cntr[i] > 0 ) 
			error = adapter->aq_hw_avb_ops->hw_avb_ring_rx_attach(rxring, flags);
		else
			error = adapter->aq_hw_avb_ops->hw_avb_ring_rx_init(rxring, flags);
		if (error)
			goto release;

		adapter->aq_shmem->rx_ring_ref_cntr[i]++;
		rxring->packet_per_desc = (struct atl_packet **)malloc(rxring->size*sizeof(struct atl_packet *));
		if (rxring->packet_per_desc == NULL)
			goto release;

		memset(rxring->packet_per_desc, 0, rxring->size*sizeof(struct atl_packet *));
			
		rxring->next_ring = adapter->rx_rings;
		
		logprint(LOG_LVL_DEBUG, "Add rx ring %p, prev %p. Ref counter %d.",
				rxring, adapter->rx_rings, adapter->aq_shmem->rx_ring_ref_cntr[i]);
		adapter->rx_rings = rxring;
	}

	if (atl_unlock(pdev) != 0)
		free(rxring);

	return 0;
release:
	if ( rxring ) {
		if (rxring->packet_per_desc )
			free(rxring->packet_per_desc);
		free(rxring);
	}

	if (atl_unlock(pdev) != 0)
		return -errno;

	return error;
}

int atl_stop_tx(device_t *dev, unsigned int queue_index, struct atl_packet **cleaned_packets)
{
	int ring_index = 0;
	struct atl_adapter *adapter;
	struct tx_ring *txr;
	u32 head, tail;
	struct atl_packet *pkt;

	if (dev == NULL)
		return -EINVAL;

	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return -ENXIO;

	txr = adapter->tx_rings;
	while( queue_index-- && txr ){
		ring_index++;
		txr = txr->next_ring;
	}
	if (!txr)
		return -EINVAL;

	if (atl_lock(dev) != 0)
		return errno;
	
	head = txr->shead;
	tail = txr->tail;
	logprint(LOG_LVL_VERBOSE, "Stop TX: %d %d", head, tail);
	while( head != tail ) {
		logprint(LOG_LVL_DEBUG, "Dump 1 TX Desc %d.", head );
		dump(LOG_LVL_DEBUG, "", (uint8_t *)&(txr->desc[head]), sizeof(union atl_rx_desc_u));
		head++;
		if( head >= txr->size ) head -= txr->size;
	}

	if( adapter->aq_shmem->tx_ring_occupation_pid[ring_index] == getpid() )
		adapter->aq_hw_avb_ops->hw_avb_ring_tx_reset(txr);

	*cleaned_packets = txr->sent_packet;
	pkt = *cleaned_packets;
	while( pkt ) {
		struct tx_desc_ptr *d_ptr = (struct tx_desc_ptr *)&pkt->private;
		logprint(LOG_LVL_DEBUG, "Dump 2 TX Desc %d.", d_ptr->last );
		dump(LOG_LVL_DEBUG, "", (uint8_t *)&(txr->desc[d_ptr->last]), sizeof(union atl_rx_desc_u));
		pkt = pkt->next;
	}

	txr->sent_packet = NULL;
	txr->last_packet = NULL;
	if( adapter->aq_shmem->tx_ring_occupation_pid[ring_index] == getpid() )
		adapter->aq_shmem->tx_ring_occupation_pid[ring_index] = 0;

	atl_unlock(dev);
	return 0;	
}

int atl_stop_rx(device_t *dev, unsigned int queue_index, struct atl_packet **cleaned_packets)
{
	int ring_index = 0;
	struct atl_adapter *adapter;
	struct rx_ring *rxr;
	struct atl_packet *cur_desc_pkt;
	u32 cur, tail;

	if (dev == NULL)
		return -EINVAL;

	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return -ENXIO;

	rxr = adapter->rx_rings;
	while( queue_index-- && rxr ){
		rxr = rxr->next_ring;
		ring_index++;
	}
	if (!rxr)
		return -EINVAL;

	if (atl_lock(dev) != 0)
		return errno;
	
	cur = rxr->shead;
	tail = rxr->tail;
	if( adapter->aq_shmem->rx_ring_occupation_pid[ring_index] == getpid() )
		adapter->aq_hw_avb_ops->hw_avb_ring_rx_reset(rxr);

	while( cur != tail ) {
		cur_desc_pkt = rxr->packet_per_desc[cur++];
		cur_desc_pkt->next = *cleaned_packets;
		*cleaned_packets = cur_desc_pkt;
		if( cur_desc_pkt == cur_desc_pkt->next ) {
			logprint(LOG_LVL_ERROR, "Error on stop! cur packet %p", cur_desc_pkt);
		}
		if( cur >= rxr->size ) {
			cur -= rxr->size;
		}
		rxr->empty++;
	}

	if( adapter->aq_shmem->rx_ring_occupation_pid[ring_index] == getpid() )
		adapter->aq_shmem->rx_ring_occupation_pid[ring_index] = 0;

	atl_unlock(dev);
	return 0;	
}

int atl_detach(device_t *dev)
{
	struct atl_adapter *adapter;
	struct ifreq ifr;
	int ring_idx = 0;

	if (dev == NULL)
		return -EINVAL;
	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return -ENXIO;

	if (atl_lock(dev) != 0)
		goto err_nolock;

	adapter->active = 0;

	//atl_reset(adapter);

	atl_unlock(dev);

	while (adapter->tx_rings){
		struct tx_ring *txring = adapter->tx_rings;
		adapter->tx_rings = txring->next_ring;

		logprint(LOG_LVL_DEBUG, "Disable tx ring %p, next ring %p. Ref count %d.", 
				txring, adapter->tx_rings, adapter->aq_shmem->tx_ring_ref_cntr[ring_idx]);
		if( (--adapter->aq_shmem->tx_ring_ref_cntr[ring_idx++]) > 0 )
			adapter->aq_hw_avb_ops->hw_avb_ring_tx_detach(txring);
		else 
			adapter->aq_hw_avb_ops->hw_avb_ring_tx_disable(txring);
		free(txring);
	}
	ring_idx = 0;
	while (adapter->rx_rings){
		struct rx_ring *rxring = adapter->rx_rings;
		adapter->rx_rings = rxring->next_ring;

		logprint(LOG_LVL_DEBUG, "Disable rx ring %p, next ring %p. Ref count %d. ", 
				rxring, adapter->rx_rings, adapter->aq_shmem->rx_ring_ref_cntr[ring_idx]);
		
		if( (--adapter->aq_shmem->rx_ring_ref_cntr[ring_idx++]) > 0 )
			adapter->aq_hw_avb_ops->hw_avb_ring_rx_detach(rxring);
		else 
			adapter->aq_hw_avb_ops->hw_avb_ring_rx_disable(rxring);

		if (rxring->packet_per_desc){
			u32 i;
			for(i = 0; i < rxring->size; i++) {
				if( rxring->packet_per_desc[i] ) {
					rxring->packet_per_desc[i] = NULL;
				}
			}
			free(rxring->packet_per_desc);
		}
		free(rxring);
	}

	atl_free_pci_resources(adapter);

	adapter->aq_shmem->adapter_ref_cntr--;
	logprint(LOG_LVL_DEBUG, "Dettach. Process count attached to device %s: %d", 
			adapter->ifname, adapter->aq_shmem->adapter_ref_cntr);

	strncpy(ifr.ifr_name, adapter->ifname, sizeof(ifr.ifr_name)-1);
	if (ioctl(adapter->sd, SIOCRELEASETSN, &ifr) < 0) {
		logprint(LOG_LVL_ERROR, "Cannot release device: %s", strerror(errno));
	}
err_nolock:
	if (adapter->memlock) {
		/*
		 * Do not unmap the shared memory region holding the pthread mutex.
		 *
		 * (void) munmap(adapter->memlock, sizeof(pthread_mutex_t));
		 *
		 * The pthread mutex is configured as a robust type mutex so that
		 * it can automatically be unlocked on process termination if needed.
		 * In order to complete the cleanup, the memory region holding the
		 * mutex instance must be accessible until that cleanup timing.
		 * Therefore we should not unmap the memory region here, otherwise
		 * the cleanup may fail.
		 *
		 * The mapped regions will automatically be unmapped at the end of
		 * the process termination.
		 */
		adapter->memlock = NULL;
		adapter->aq_shmem = NULL;
	}

	close(adapter->sd);

	free(dev->ifname);
	free(dev->driver_class);
	close(dev->class_fd);

	dev->private_data = NULL;
	free(adapter);
	logprint(LOG_LVL_INFO, "Detach complete");
	return 0;
}

int atl_init(device_t *dev)
{
	struct atl_adapter *adapter;

	if (dev == NULL)
		return -EINVAL;
	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return -ENXIO;

	if (atl_lock(dev) != 0)
		return errno;

	//atl_reset(adapter);

	if (atl_unlock(dev) != 0)
		return errno;

	return 0;
}

static int map_bar(struct atl_adapter *adapter, int dir_fd, char *name)
{
	int ret = -1;
	int bar_fd = openat(dir_fd, name, O_RDONLY | O_DIRECTORY);
	if (bar_fd == -1)
		return -1;

	int index = read_ulong_attr(bar_fd, "index");
	struct mmap_res *bar = &adapter->bar[index];
	bar->paddr = read_ulong_attr(bar_fd, "addr");
	bar->size = read_ulong_attr(bar_fd, "len");
	bar->idx = index;

	char res_name[PATH_MAX];
	snprintf(res_name, sizeof(res_name), "device/resource%d", index);
	bar->fd = openat(adapter->class_fd, res_name, O_RDWR);
	if (bar->fd == -1) {
		logprint(LOG_LVL_ERROR, "Can't open bar %d: %s", index, strerror(errno));
		goto out;
	}
	bar->vaddr = mmap(NULL, bar->size, PROT_READ|PROT_WRITE, MAP_SHARED,
			  bar->fd, 0);
	if (bar->vaddr == MAP_FAILED) {
		bar->vaddr = NULL;
		logprint(LOG_LVL_ERROR, "Can't mmap bar 0: %s", strerror(errno));
		goto out;
	}
	ret = 0;

out:
	close(bar_fd);
	return ret;
}

static int barfilter(const struct dirent *dir)
{
	return !strncmp(dir->d_name, "membar", 6);
}

static void unmap_bars(struct atl_adapter *adapter)
{
	int i;
	for (i = 0; i < 6; i++) {
		struct mmap_res *bar = &adapter->bar[i];
		if (bar->vaddr)
			munmap(bar->vaddr, bar->size);
		if (bar->fd)
			close(bar->fd);
		memset(bar, 0, sizeof(*bar));
	}
}

static int map_bars(struct atl_adapter *adapter)
{
	struct dirent **bars;
	char dirname[PATH_MAX];
	int bardir_fd = openat(adapter->class_fd, "device/bars", O_RDONLY | O_DIRECTORY);
	if (bardir_fd == -1) {
		logprint(LOG_LVL_ERROR, "Can't open BAR directory: %s", strerror(errno));
		return -1;
	}

	snprintf(dirname, sizeof(dirname), "%s/%s/device/bars", class_dir, adapter->ifname);
	int num = scandir(dirname, &bars, &barfilter, &alphasort);

	int i, ret = 0;
	for (i = 0; i < num; i++) {
		if (!ret)  {// Don't map subsequent BARs after first failure
			ret = map_bar(adapter, bardir_fd, bars[i]->d_name);
		}
		free(bars[i]);
	}
	free(bars);
	close(bardir_fd);

	if (ret){
		unmap_bars(adapter);
		return -1;
	}

	for (i = 0; i < num; i++)
		if (adapter->bar[i].vaddr) {
			adapter->default_bar = &adapter->bar[i];
			break;
		}

	return 0;
}

static int atl_allocate_pci_resources(struct atl_adapter *adapter)
{
	int fd = openat(adapter->class_fd, "device/mem", O_RDONLY | O_DIRECTORY);
	if (fd == -1) {
		logprint(LOG_LVL_ERROR, "Can't open memreg sysfs directory: %s",
		       strerror(errno));
		goto err_mem_fd;
	}
	adapter->mem_fd = fd;

	if (map_bars(adapter) == -1)
		goto err_bars;

	//adapter->opened = 1;
	return 0;

err_bars:
	close(adapter->mem_fd);
err_mem_fd:
	close(fd);
	return -1;
}

static int free_memreg(struct atl_adapter *adapter, struct mmap_res *memreg)
{
	struct ifreq ifr;
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_name, adapter->ifname, sizeof(ifr.ifr_name)-1);
	ifr.ifr_metric = memreg->idx;

	munmap(memreg->vaddr, memreg->size);
	close(memreg->fd);
	ioctl(adapter->sd, SIOCFREEDMABUF, &ifr);
	free(memreg);

	return 0;
}

static void free_memregs(struct atl_adapter *adapter)
{
	struct mmap_res *memreg, *tmp;
	list_for_each_entry_safe(memreg, tmp, &adapter->memregs, list) {
		list_del(&memreg->list);
		free_memreg(adapter, memreg);
	}
}

static void atl_free_pci_resources(struct atl_adapter *adapter)
{
	logprint(LOG_LVL_DEBUG, "atl_free_pci_resources");
	free_memregs(adapter);
	unmap_bars(adapter);
	close(adapter->mem_fd);
   
	return;
}

int atl_dma_remap_buffer(struct atl_adapter *adapter, struct atl_dma_alloc *dma) 
{
	int dir_fd, mem_idx = 0;
	int error = 0;
	char buf[PATH_MAX];
	struct mmap_res *memreg;

	if (dma == NULL)
		return -EINVAL;

	logprint(LOG_LVL_DEBUG, "Find mem: paddr %lx, size %x", dma->dma_paddr, dma->mmap_size);
	memreg = calloc(1, sizeof(*memreg));
	if (!memreg) {
		logprint(LOG_LVL_ERROR, "Can't alloc memory descriptor: %s",
		       strerror(errno));
		error = errno;
		goto err_memreg;
	}
	INIT_LIST_HEAD(&memreg->list);

	while( true ) {
		snprintf(buf, sizeof(buf), "%d", mem_idx);
		dir_fd = openat(adapter->mem_fd, buf, O_RDONLY | O_DIRECTORY); //so c++ doesn't complain
		if (dir_fd == -1) {
			logprint(LOG_LVL_ERROR, "Can't open memreg's sysfs dir: %s",
		    	   strerror(errno));
			logprint(LOG_LVL_DEBUG, "Last idx %d", mem_idx);
			error = errno;
			goto err_open_dir;
		}
		memreg->idx = mem_idx;
		memreg->paddr = read_ulong_attr(dir_fd, "paddr");
		memreg->size = read_ulong_attr(dir_fd, "real_size");
		memreg->fd = openat(dir_fd, "mmap", O_RDWR);
		logprint(LOG_LVL_DEBUG, "Idx %d, paddr %lx, size %x", mem_idx, memreg->paddr, memreg->size);
		if (memreg->fd == -1) {
			logprint(LOG_LVL_ERROR, "Can't open memreg with id 0x%x: %s", mem_idx,
		    	   strerror(errno));
			goto err_open_mem;
		}
		if( dma->dma_paddr == memreg->paddr &&
			dma->mmap_size == memreg->size ) {
			break;
		}
		//memreg->idx = 0;
		//memreg->paddr = 0;
		//memreg->size = 0;
		close(memreg->fd);
		close(dir_fd);
		mem_idx++;
	}
	memreg->vaddr = mmap(NULL, memreg->size, PROT_READ|PROT_WRITE,
			     MAP_SHARED, memreg->fd, 0);
	if (memreg->vaddr == MAP_FAILED) {
		logprint(LOG_LVL_ERROR, "Can't mmap memreg: %s",
		       strerror(errno));
		error = errno;
		goto err_mmap;
	}

	logprint(LOG_LVL_DEBUG, "Done! vaddr %p, memreg %p", memreg->vaddr, memreg);
	list_add(&memreg->list, &adapter->memregs);
	close(dir_fd);

	dma->dma_paddr = memreg->paddr;
	dma->dma_vaddr = memreg->vaddr;
	dma->mmap_size = memreg->size;
	return 0;

err_mmap:
	close(memreg->fd);
err_open_mem:
	close(dir_fd);
err_open_dir:
	free(memreg);
err_memreg:
	return error;
}

int atl_dma_detach_buffer(struct atl_adapter *adapter, struct atl_dma_alloc *dma) 
{
	struct mmap_res *memreg, *tmp;

	logprint(LOG_LVL_DEBUG, "Find memreg: paddr %lx, vaddr %p, size %x", 
			dma->dma_paddr, dma->dma_vaddr, dma->mmap_size);
	list_for_each_entry_safe(memreg, tmp, &adapter->memregs, list) {
		logprint(LOG_LVL_DEBUG, "Memreg %p, paddr %lx, vaddr %p, size %x", memreg, memreg->paddr, memreg->vaddr, memreg->size);
		if( dma->dma_paddr == memreg->paddr &&
			dma->dma_vaddr == memreg->vaddr &&
			dma->mmap_size == memreg->size ) {
			list_del(&memreg->list);
			break;
		}
		memreg = NULL;
	}

	if( memreg == NULL )
		goto err;
	
	logprint(LOG_LVL_DEBUG, "Done! memreg %p", memreg);
	free(memreg);
	dma->dma_paddr = 0;
	dma->dma_vaddr = NULL;
	dma->mmap_size = 0;

err:
	return 0;
}

/*
 * Manage DMA'able memory.
 */
int atl_dma_alloc_buffer(struct atl_adapter *adapter, struct atl_dma_alloc *dma)
{
	int error = 0;
	struct ifreq ifr;
	struct aq_alloc_mem req;

	req.size = dma->mmap_size;
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_name, adapter->ifname, sizeof(ifr.ifr_name)-1);
	ifr.ifr_data = (void *)&req;

	if (dma == NULL)
		return -EINVAL;

	error = ioctl(adapter->sd, SIOCALLOCDMABUF, &ifr);
	if (error < 0) {
		error = errno;
		if(error != -EINVAL)
			error = -ENOMEM;
		goto err;
	}

	struct mmap_res *memreg = calloc(1, sizeof(*memreg));
	if (!memreg) {
		logprint(LOG_LVL_ERROR, "Can't alloc memory descriptor: %s",
		       strerror(errno));
		error = errno;
		goto err_memreg;
	}
	INIT_LIST_HEAD(&memreg->list);

	char buf[PATH_MAX];
	snprintf(buf, sizeof(buf), "%d", req.index);
	int dir_fd;
	dir_fd = openat(adapter->mem_fd, buf, O_RDONLY | O_DIRECTORY); //so c++ doesn't complain
	if (dir_fd == -1) {
		logprint(LOG_LVL_ERROR, "Can't open memreg's sysfs dir: %s",
		       strerror(errno));
		error = errno;
		goto err_open_dir;
	}

	memreg->idx = req.index;
	memreg->paddr = read_ulong_attr(dir_fd, "paddr");
	memreg->size = read_ulong_attr(dir_fd, "real_size");
	memreg->fd = openat(dir_fd, "mmap", O_RDWR);
	if (memreg->fd == -1) {
		logprint(LOG_LVL_ERROR, "Can't open memreg with id 0x%x: %s", req.index,
		       strerror(errno));
		goto err_open_mem;
	}

	memreg->vaddr = mmap(NULL, memreg->size, PROT_READ|PROT_WRITE,
			     MAP_SHARED, memreg->fd, 0);
	if (memreg->vaddr == MAP_FAILED) {
		logprint(LOG_LVL_ERROR, "Can't mmap memreg: %s",
		       strerror(errno));
		error = errno;
		goto err_mmap;
	}

	list_add(&memreg->list, &adapter->memregs);
	close(dir_fd);

	dma->dma_paddr = memreg->paddr;
	dma->dma_vaddr = memreg->vaddr;
	dma->mmap_size = memreg->size;
	return 0;

err_mmap:
	close(memreg->fd);
err_open_mem:
	close(dir_fd);
err_open_dir:
	free(memreg);
err_memreg:
	ifr.ifr_metric = req.index;
	ioctl(adapter->sd, SIOCFREEDMABUF, &ifr);
err:
	return error;
}

int atl_dma_malloc_page(device_t *dev, struct atl_dma_alloc *dma)
{
	struct atl_adapter *adapter;
	int error = 0;
	if (dev == NULL)
		return -EINVAL;

	if (dma == NULL)
		return -EINVAL;

	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return -ENXIO;

	if (atl_lock(dev) != 0) {
		error = errno;
		goto err;
	}

	if( dma )
		dma->mmap_size = atl_getpagesize();
	error = atl_dma_alloc_buffer(adapter, dma);
	if (atl_unlock(dev) != 0) {
		error = errno;
	}
err:
	return error;
}

void atl_dma_free_buffer(struct atl_adapter *adapter, struct atl_dma_alloc *dma)
{
	struct ifreq ifr;
	struct mmap_res *memreg, *tmp;

	list_for_each_entry_safe(memreg, tmp, &adapter->memregs, list) {
		if( dma->dma_paddr == memreg->paddr &&
			dma->dma_vaddr == memreg->vaddr &&
			dma->mmap_size == memreg->size ) {
			list_del(&memreg->list);
			break;
		}
		memreg = NULL;
	}

	if( memreg == NULL )
		goto err;
	
	free_memreg(adapter, memreg);
	dma->dma_paddr = 0;
	dma->dma_vaddr = NULL;
	dma->mmap_size = 0;

err:
	return;
}

void atl_dma_free_page(device_t *dev, struct atl_dma_alloc *dma)
{
	struct atl_adapter *adapter;

	if (dev == NULL)
		return;
	if (dma == NULL)
		return;

	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return;

	if (atl_lock(dev) != 0)
		goto err;

	atl_dma_free_buffer(adapter, dma);
	if (atl_unlock(dev) != 0)
		goto err;
err:
	return;
}

static int atl_writereg(device_t *dev, u_int32_t reg, u_int32_t data)
{
	struct atl_adapter *adapter;

	if (dev == NULL)
		return -EINVAL;

	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return -ENXIO;

	if (reg > 0x9000) //TODO better solution
		return -EINVAL;

	((volatile u32 *)(adapter->bar[0].vaddr))[reg>>2] = data;
	return 0;
}

static int atl_readreg(device_t *dev, u_int32_t reg, u_int32_t *data)
{
	struct atl_adapter *adapter;

	if (dev == NULL)
		return -EINVAL;

	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return -ENXIO;

	if (data == NULL)
		return -EINVAL;

	if (reg > 0x9000) //TODO better solution
		return -EINVAL;

	*data = ((volatile u32 *)(adapter->bar[0].vaddr))[reg>>2];
	return 0;
}

int atl_lock(device_t *dev)
{
	struct atl_adapter *adapter;
	int error = -1;

	if (dev == NULL)
		return -ENODEV;

	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return -ENXIO;

	if (adapter->active != 1)	// detach in progress
		return -ENXIO;

	if (!adapter->memlock)
		return -ENXIO;

	error = pthread_mutex_lock(adapter->memlock);
	switch (error) {
		case 0:
			break;
		case EOWNERDEAD:
			// some process terminated without unlocking the mutex
			if (pthread_mutex_consistent(adapter->memlock) != 0)
				return -errno;
			break;
		default:
			return -errno;
			break;
	}

	if (adapter->active != 1) {
		(void) pthread_mutex_unlock(adapter->memlock);
		return -ENXIO;
	}

	return 0;
}

int atl_unlock(device_t *dev)
{
	struct atl_adapter *adapter;

	if (dev == NULL)
		return -ENODEV;

	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return -ENXIO;

	if (!adapter->memlock)
		return -ENXIO;

	return pthread_mutex_unlock(adapter->memlock);
}

union atl_tx_desc_u *get_empty_tx_desc(struct tx_ring *txr)
{
	union atl_tx_desc_u *txd = NULL;
	if (txr->empty > 1) {
		txd = &txr->desc[txr->tail++];
		if( txr->tail >= txr->size ) txr->tail = 0;
		txr->empty--;
	}
	return txd;
}

/* Context Descriptor setup for VLAN or CSUM */
static void atl_tx_ctx_setup(struct tx_ring *txr, struct atl_packet *packet)
{
	struct atl_adapter *adapter = txr->adapter;
	union atl_tx_desc_u *txd;
	int ct_idx = 0;

	if( !packet->vlan )
		return;
		
	txd = get_empty_tx_desc(txr);

	/* Now copy bits into descriptor */
	txd->ctx.vlan_tag = packet->vlan;
	txd->ctx.mss_len = 0;
	txd->ctx.ct_idx = ct_idx;
	packet->ct_idx = ct_idx;
}

int atl_get_xmit_space(device_t *dev, unsigned int queue_index)
{
	struct atl_adapter *adapter;
	struct tx_ring *txr;
	if (dev == NULL)
		return -EINVAL;

	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return -ENXIO;

	txr = adapter->tx_rings;
	while( queue_index-- && txr ){
		txr = txr->next_ring;
	}
	if (!txr)
		return -EINVAL;

	return txr->empty;
}

static int atl_soft_shaper(struct atl_adapter *adapter, bool class_a, struct atl_packet *packet)
{
	u32 cur_credit = class_a ? adapter->class_a_cur_credit : adapter->class_b_cur_credit;
	u32 threshold = class_a ? adapter->class_a_threshold : (s32)adapter->class_b_threshold;
	u64 last_lt = class_a ? adapter->class_a_last_lt : adapter->class_b_last_lt, cur_lt = packet->attime;
	u32 window_size_in_ns = 1000000000 / (class_a ? TSN_CLASS_A_PPS : TSN_CLASS_B_PPS);
	u32 max_per_window = window_size_in_ns * adapter->credit_for_one_ns;

	if( !cur_lt ) {
		adapter->aq_hw_avb_ops->hw_latch_clock(adapter);
		adapter->aq_hw_avb_ops->hw_get_clock(adapter, &cur_lt);
	}

	if( last_lt ) {
		if( cur_credit < threshold ) {
			logprint(LOG_LVL_VERBOSE,
				"Send more data over class %c than it required for time-window. Cur credit %d.", 
				class_a ? 'A' : 'B', cur_credit);
			//Reschedule the packet to next time window as shaper will do
			cur_lt = last_lt + window_size_in_ns;
		}
		if( cur_lt > last_lt ) {
			cur_credit += (cur_lt - last_lt) * adapter->credit_for_one_ns;
			if( cur_credit > max_per_window ) {
				cur_credit = max_per_window;
			}
		} else if( cur_lt < last_lt ) {
			cur_lt = last_lt;
		}
	}

	if( cur_credit < packet->len * adapter->credit_for_one_byte ) {
		logprint(LOG_LVL_WARNING, 
			"Too much data over class %c", class_a ? 'A' : 'B');
		cur_credit = 0;
	}
	else {
		cur_credit -= packet->len * adapter->credit_for_one_byte;
	}

	if( class_a ) {
		adapter->class_a_cur_credit = cur_credit;
		adapter->class_a_last_lt = cur_lt;
	} else {
		adapter->class_b_cur_credit = cur_credit;
		adapter->class_b_last_lt = cur_lt;
	}

	if( packet->attime != cur_lt ) {
		packet->attime = cur_lt;
	}
	return 0;
}

/*********************************************************************
 *
 *  This routine maps a single buffer to an TX descriptor.
 *  returns ENOSPC if we run low on tx descriptors and the app needs to
 *  cleanup descriptors.
 *
 *  this is a simplified routine which doesn't do LSO, checksum offloads,
 *  multiple fragments, etc. The provided buffers are assumed to have
 *  been previously mapped with the provided dma_malloc_page routines.
 *
 **********************************************************************/
int atl_xmit(device_t *dev, unsigned int queue_index, struct atl_packet **packet)
{
	struct atl_adapter *adapter;
	struct tx_ring *txr;
	struct atl_packet *send_packet = NULL;
	int error = 0, ring_index = 0;
	u32 cur_class_bandwith = 0;
	uint32_t egress_offset;
	bool class_a = queue_index == 0;

	if (dev == NULL)
		return -EINVAL;

	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return -ENXIO;

	//txr = &adapter->tx_rings[queue_index];
	egress_offset = adapter->hw_offsets & 0xffff;

	cur_class_bandwith = class_a ? adapter->class_a_bytes_per_second : adapter->class_b_bytes_per_second;
	txr = adapter->tx_rings;
	while( queue_index-- && txr ){
		txr = txr->next_ring;
		ring_index++;
	}

	if (!txr)
		return -EINVAL;

	if (packet == NULL || *packet == NULL)
		return -EINVAL;

	if (atl_lock(dev) != 0)
		return errno;

	if ( adapter->aq_shmem->tx_ring_occupation_pid[ring_index] &&
		 adapter->aq_shmem->tx_ring_occupation_pid[ring_index] != getpid() ){
		error = -EBUSY;
		goto unlock;
	}

	if ( !adapter->aq_shmem->tx_ring_occupation_pid[ring_index] ) {
		adapter->aq_hw_avb_ops->hw_avb_ring_tx_reset(txr);
	}

	do {
		if (txr->empty <= 2) {
			error = ENOSPC;
			break;
		}
		send_packet = *packet;
		*packet = send_packet->next;
		send_packet->next = NULL; /* used for cleanup */

		if( cur_class_bandwith ) {
			atl_soft_shaper(adapter, class_a, send_packet);
		}

		if( send_packet->attime > egress_offset )
			send_packet->attime -= egress_offset;

		adapter->aq_hw_avb_ops->hw_ring_tx_prepare_xmit(txr, send_packet);

		if( txr->sent_packet == NULL ){
			txr->sent_packet = send_packet;
			adapter->aq_shmem->tx_ring_occupation_pid[ring_index] = getpid();
		}

		if( txr->last_packet ) {
			txr->last_packet->next = send_packet;
		}
		txr->last_packet = send_packet;

		logprint(LOG_LVL_DEBUG, "Sent %p, last %p, cur %p", txr->sent_packet, txr->last_packet, send_packet );
	} while( *packet );
unlock:
	if (atl_unlock(dev) != 0)
		return errno;

	return error;
}

/**********************************************************************
 *
 *  Examine each tx_buffer in the used queue. If the hardware is done
 *  processing the packet then return the linked list of associated resources.
 *
 **********************************************************************/
int atl_clean(device_t *dev, struct atl_packet **cleaned_packets)
{
	int count = 0, ring_index = 0;
	struct atl_adapter *adapter;
	struct tx_ring *txr;
	struct atl_packet *cur_pkt, *last = NULL;
	uint32_t egress_offset;
	//u32 ts_idx;

	if (dev == NULL)
		return -EINVAL;

	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return -ENXIO;

	if (cleaned_packets == NULL)
		return -EINVAL;

	egress_offset = adapter->hw_offsets & 0xffff;
	*cleaned_packets = NULL; /* nothing reclaimed yet */

	if (atl_lock(dev) != 0)
		return errno;

	txr = adapter->tx_rings;
	while( txr ) {
		if ( adapter->aq_shmem->tx_ring_occupation_pid[ring_index] == getpid() ){
			last = NULL;
			cur_pkt = txr->sent_packet;

			while (cur_pkt) {
				if( !adapter->aq_hw_avb_ops->hw_ring_tx_desc_complete(txr, cur_pkt) ){
					break;
				}
				if( *cleaned_packets == NULL ){
					*cleaned_packets = cur_pkt;
				}

				if( cur_pkt->flags & ATL_PKT_FLAG_HW_TS_VLD ) {
					cur_pkt->hwtime += egress_offset;
				}
				count++;

				last = cur_pkt;
				cur_pkt = cur_pkt->next;
			}

			if( last ) {
				last->next = NULL;
				txr->sent_packet = cur_pkt;

				if( cur_pkt == NULL ) {
					txr->last_packet = NULL;
				}
			}
			if( txr->sent_packet == NULL )
				adapter->aq_shmem->tx_ring_occupation_pid[ring_index] = 0;
		}
		ring_index++;
		txr = txr->next_ring;
	}
	atl_unlock(dev);
	return count;
}

/*
 *  Refresh mbuf buffers for RX descriptor rings
 *   - now keeps its own state so discards due to resource
 *     exhaustion are unnecessary, if an mbuf cannot be obtained
 *     it just returns, keeping its placeholder, thus it can simply
 *     be recalled to try again.
 *
 */
int atl_refresh_buffers(device_t *dev, u_int32_t idx,
			 struct atl_packet **rxbuf_packets, u_int32_t *num_bufs)
{
	int error = 0;
	struct atl_packet *cur_pkt;
	struct atl_adapter *adapter;
	struct rx_ring *rxr;
	u_int32_t refresh_bufs, ring_index = 0;
	bool refreshed = FALSE;
	union atl_rx_desc_u *d_ptr;

	if (dev == NULL)
		return -EINVAL;

	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return -EINVAL;

	//if (adapter->active != 1)	// detach in progress
	//	return -ENXIO;

	if (rxbuf_packets == NULL)
		return -EINVAL;

	if (idx > 1)
		return -EINVAL;

	rxr = adapter->rx_rings;
	while( rxr != NULL && idx-- > 0 ) {
		rxr = rxr->next_ring;
		ring_index++;
	}

	if (rxr == NULL)
		return -EINVAL;
	//if (sem_trywait(&rxr->lock) != 0)
	//	return errno; /* EAGAIN */
	if (atl_lock(dev) != 0)
		return errno;

	//logprint(LOG_LVL_DEBUG, "RX ring pid: %x. Cur pid %x.", adapter->aq_shmem->rx_ring_occupation_pid[ring_index], getpid() );
	if ( adapter->aq_shmem->rx_ring_occupation_pid[ring_index] && 
		adapter->aq_shmem->rx_ring_occupation_pid[ring_index] != getpid() ){
		error = -EBUSY;
		goto unlock;
	}

	if ( !adapter->aq_shmem->rx_ring_occupation_pid[ring_index] ) {
		adapter->aq_hw_avb_ops->hw_avb_ring_rx_reset(rxr);
	}

	refresh_bufs = *num_bufs;
	adapter->aq_hw_avb_ops->hw_ring_rx_refill(rxr, rxbuf_packets, num_bufs);
	logprint(LOG_LVL_DEBUG, "Refresh buffers, ring_index %d, bufs %d, num_bufs %d", ring_index, refresh_bufs, *num_bufs);

	if( *num_bufs != refresh_bufs )
		adapter->aq_shmem->rx_ring_occupation_pid[ring_index] = getpid();
unlock:
	atl_unlock(dev);

	return error;
}

/**********************************************************************
 *
 *  Examine each rx_buffer in the used queue. If the hardware is done
 *  processing the packet then return the linked list of associated resources.
 *
 **********************************************************************/
int atl_receive(device_t *dev, unsigned int queue_index,
				struct atl_packet **received_packets, u_int32_t *count)
{
	int ring_index = 0, error = 0;
	struct atl_adapter *adapter;
	struct rx_ring *rxr;
	bool eop = FALSE;
	u_int32_t staterr = 0;
	u_int32_t desc    = 0;
	u_int32_t max_pkt = 0;
	uint32_t ingress_offset;

	if (dev == NULL)
		return -EINVAL;

	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return -ENXIO;

	//if (adapter->active != 1)	// detach in progress
	//	return -ENXIO;
	ingress_offset = (adapter->hw_offsets >> 16) & 0xffff;
	rxr = adapter->rx_rings;
	while( rxr != NULL && queue_index-- > 0 ) {
		rxr = rxr->next_ring;
		ring_index++;
	}
	if (!rxr)
		return -EINVAL;

	if (count == NULL)
		return -EINVAL;

	if (received_packets == NULL)
		return -EINVAL;

	*received_packets = NULL; /* nothing reclaimed yet */

	if (atl_lock(dev) != 0)
		return errno;

	if( rxr->size == rxr->empty ) {
		error = -EINVAL;
		goto unlock;
	}

	if ( adapter->aq_shmem->rx_ring_occupation_pid[ring_index] &&
		 adapter->aq_shmem->rx_ring_occupation_pid[ring_index] != getpid() ){
		error = -EBUSY;
		goto unlock;
	}

	adapter->aq_hw_avb_ops->hw_ring_rx_ts_receive(rxr, received_packets, count);

	if( rxr->size == rxr->empty )
		adapter->aq_shmem->rx_ring_occupation_pid[ring_index] = 0;
unlock:
	if (atl_unlock(dev) != 0)
		return errno;

	if (*received_packets == NULL) {
		/* nothing reclaimed yet */
		error = EAGAIN;
	} else {
		struct atl_packet *tmp_packet = *received_packets;
		while( tmp_packet ) {
			if( tmp_packet->flags & ATL_PKT_FLAG_HW_TS_VLD ) {
				tmp_packet->hwtime -= ingress_offset;
			}
			tmp_packet = tmp_packet->next;
		}
	}

	return error;
}

#define MAX_ITER 32
#define MIN_WALLCLOCK_TSC_WINDOW 80 /* cycles */
#define MIN_SYSCLOCK_WINDOW 72 /* ns */

static inline void rdtscpll(uint64_t *val)
{
	uint32_t high, low;

	__asm__ __volatile__("lfence;"
						  "rdtsc;"
						  : "=d"(high), "=a"(low)
						  :
						  : "memory");
	*val = high;
	*val = (*val << 32) | low;
}

static inline void __sync(void)
{
	__asm__ __volatile__("mfence;"
						  :
						  :
						  : "memory");
}

int atl_get_wallclock(device_t *dev, u_int64_t *curtime, u_int64_t *rdtsc)
{
	u_int64_t t0 = 0, t1 = -1;
	u_int32_t duration = -1;
	struct atl_adapter *adapter;
	int error = 0;
	int iter = 0;

	if (dev == NULL)
		return -EINVAL;

	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return -ENXIO;

	if (atl_lock(dev) != 0)
		return errno;

	/* sample the timestamp bracketed by the RDTSC */
	for (iter = 0; iter < MAX_ITER && t1 - t0 > MIN_WALLCLOCK_TSC_WINDOW;
	     ++iter) {
		rdtscpll(&t0);
		adapter->aq_hw_avb_ops->hw_latch_clock(adapter);
		rdtscpll(&t1);

		if (t1 - t0 < duration) {
			duration = t1 - t0;

			if (curtime)
				adapter->aq_hw_avb_ops->hw_get_clock(adapter, curtime);

			if (rdtsc)
				/* average */
				*rdtsc = (t1 - t0) / 2 + t0;
		}
	}

	if (atl_unlock(dev) != 0) {
		error = errno;
		goto err;
	}

	/* Return the window size * -1 */
	return -duration;

err:
	return error;
}

/* Atlantic has no AVB credit scheduler. Just check the parameter is valid or not */
int atl_set_class_bandwidth(device_t *dev, u_int32_t class_a_bytes_per_second,
		u_int32_t class_b_bytes_per_second)
{
	struct atl_link_state link = {0};
	u_int32_t linkrate;
	struct atl_adapter *adapter;
	int err;
	float class_a_percent, class_b_percent;
	int error = 0;
	struct ifreq ifr;
	u32 max_bytes_per_second = 0;

	if (dev == NULL)
		return -EINVAL;
	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return -ENXIO;

	if (adapter->a1 && class_b_bytes_per_second > 0 )
		return -ENXIO; //No class B support for AQC107

	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_name, dev->ifname, sizeof(ifr.ifr_name)-1);
	ifr.ifr_data = (void *)&link;

	/* get current link speed */
	err = ioctl(adapter->sd, SIOCLINKCMD, &ifr);
	if (err)
		return -ENXIO;

	if (link.speed == ATL_LINK_DOWN)
		return -EINVAL;

	if (link.speed > ATL_LINK_10G)
		return -EINVAL;

	if (atl_lock(dev) != 0)
		return errno;

	adapter->link_speed = link.speed;
	adapter->hw_offsets = link.hw_offsets;
	
	if( adapter->aq_shmem->class_a_bytes_per_second >= adapter->class_a_bytes_per_second ) {
		adapter->aq_shmem->class_a_bytes_per_second -= adapter->class_a_bytes_per_second;
	} else {
		error = -EINVAL;
		goto unlock;
	}
	if( adapter->aq_shmem->class_b_bytes_per_second >= adapter->class_b_bytes_per_second ) {
		adapter->aq_shmem->class_b_bytes_per_second -= adapter->class_b_bytes_per_second;
	} else {
		error = -EINVAL;
		goto unlock;
	}

	adapter->class_a_last_lt = 0;
	adapter->class_b_last_lt = 0;

	if ((class_a_bytes_per_second + class_b_bytes_per_second) == 0) {
		logprint(LOG_LVL_VERBOSE, "Disable traffic shaper. Rest: Class A : %d, Class B : %d",
			adapter->aq_shmem->class_a_bytes_per_second,
			adapter->aq_shmem->class_b_bytes_per_second);
		goto unlock;
	}

	/*
	 * class_a and class_b are the packets-per-(respective)observation
	 * interval (125 usec for class A, 250 usec for class B)
	 * these parameters are also used when establishing the MSRP
	 * talker advertise attribute (as well as the tpktsize)
	 *
	 * note that class_a and class_b are independent of the media
	 * rate. For our idle slope calculation, we need to scale the
	 * (tpktsz + (media overhead)) * rate -> percentage of media rate.
	 */

	if (link.speed == ATL_LINK_100M) {
		/* bytes-per-sec @ 100Mbps */ 
		adapter->credit_for_one_ns = 1;
		adapter->credit_for_one_byte = 80;
		max_bytes_per_second = 12500000;
	} else if (link.speed == ATL_LINK_1G) {
		/* bytes-per-sec @ 1Gbps */
		adapter->credit_for_one_ns = 1;
		adapter->credit_for_one_byte = 8;
		max_bytes_per_second = 125000000;
	} else if (link.speed == ATL_LINK_2_5G) {
		/* bytes-per-sec @ 1Gbps */
		adapter->credit_for_one_ns = 5;
		adapter->credit_for_one_byte = 16;
		max_bytes_per_second = 1250000000/4;
	} else if (link.speed == ATL_LINK_5G) {
		/* bytes-per-sec @ 1Gbps */
		adapter->credit_for_one_ns = 5;
		adapter->credit_for_one_byte = 8;
		max_bytes_per_second = 1250000000/2;
	} else if (link.speed == ATL_LINK_10G) {
		/* bytes-per-sec @ 1Gbps */
		adapter->credit_for_one_ns = 5;
		adapter->credit_for_one_byte = 4;
		max_bytes_per_second = 1250000000;
	}

	if (//(class_a_bytes_per_second + class_b_bytes_per_second) > (max_bytes_per_second/4)*3 ||  // Class A + Class B > 75%
		 class_a_bytes_per_second > (max_bytes_per_second/4)*3 || //Wrong input value
		 class_b_bytes_per_second > (max_bytes_per_second/4)*3 || //Wrong input value
		(adapter->aq_shmem->class_a_bytes_per_second + class_a_bytes_per_second + 
		 adapter->aq_shmem->class_b_bytes_per_second + class_b_bytes_per_second) > (max_bytes_per_second/4)*3 // Class A + Class B > 75%
	) {
		error = -EINVAL;
		logprint(LOG_LVL_ERROR, "Too big amount of TSN data: Class A %d Bps, Class B %d Bps, Link MAX Bps %d",
			adapter->aq_shmem->class_a_bytes_per_second + class_a_bytes_per_second,
			adapter->aq_shmem->class_b_bytes_per_second + class_b_bytes_per_second,
			max_bytes_per_second);
		goto unlock;
	}

	// Add credits for first time-window
	adapter->class_a_cur_credit = (1000000000 / TSN_CLASS_A_PPS) * adapter->credit_for_one_ns;
	adapter->class_b_cur_credit = (1000000000 / TSN_CLASS_B_PPS) * adapter->credit_for_one_ns; 

	//
	adapter->class_a_threshold = adapter->class_a_cur_credit - \
		((class_a_bytes_per_second + TSN_CLASS_A_PPS-1)/TSN_CLASS_A_PPS)*adapter->credit_for_one_byte;
	adapter->class_b_threshold = adapter->class_b_cur_credit - \
		((class_b_bytes_per_second + TSN_CLASS_B_PPS-1)/TSN_CLASS_B_PPS)*adapter->credit_for_one_byte;

	logprint(LOG_LVL_VERBOSE, "Set traffic shaper values (cur/thr). Class A : %d/%d, Class B : %d/%d",
		adapter->class_a_cur_credit, adapter->class_a_threshold,
		adapter->class_b_cur_credit, adapter->class_b_threshold);

	adapter->class_a_bytes_per_second = class_a_bytes_per_second;
	adapter->class_b_bytes_per_second = class_b_bytes_per_second;

	adapter->aq_shmem->class_a_bytes_per_second += class_a_bytes_per_second;
	adapter->aq_shmem->class_b_bytes_per_second += class_b_bytes_per_second;
	logprint(LOG_LVL_VERBOSE, "Set traffic shaper. Class A : %d/%d, Class B : %d/%d",
		adapter->class_a_bytes_per_second, adapter->aq_shmem->class_a_bytes_per_second,
		adapter->class_b_bytes_per_second, adapter->aq_shmem->class_b_bytes_per_second);

unlock:
	if (atl_unlock(dev) != 0)
		error = errno;

	return error;
}

/*
int atl_get_mac_addr(device_t *dev, u_int8_t mac_addr[ETH_ADDR_LEN])
{
	struct atl_adapter *adapter;

	if (dev == NULL)
		return -EINVAL;
	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return -ENXIO;

	memcpy(mac_addr, adapter->mac.addr, ETH_ADDR_LEN);
return 0;
}
*/
int atl_setup_filter(device_t *dev, unsigned int queue_index,
			unsigned int vlan_id, unsigned int vlan_pcp,
			unsigned int eth_type, unsigned int udp_dest_port, 
			unsigned int ipv4, const unsigned char dst_mac[6], 
			unsigned int *filter_id)
{
	int error = 0;
	struct atl_adapter *adapter;
	struct rx_ring *rxr;
	struct ifreq ifr;
	struct ethtool_rxnfc nfccmd;
	struct ethtool_rx_flow_spec *fsp =
		(struct ethtool_rx_flow_spec *)&nfccmd.fs;
	if (dev == NULL)
		return -EINVAL;

	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return -ENXIO;

	if (queue_index > 1)
		return -EINVAL;

	rxr = adapter->rx_rings;
	while( rxr != NULL && queue_index-- > 0 ) {
		rxr = rxr->next_ring;
	}
	if (!rxr)
		return -EINVAL;

	if( eth_type >= 0x10000 && udp_dest_port >= 0x10000 && !dst_mac ){
	    logprint(LOG_LVL_WARNING, "No filter selected");
		return -EINVAL;
	}

	if( vlan_id != -1 && vlan_id >= 0x1000 || 
		vlan_pcp != -1 && vlan_pcp > 7 ) {
	    logprint(LOG_LVL_WARNING, "Wrong VLAN filter parameters");
		return -EINVAL;
	}

	if ( //vlan_id == 0 || vlan_id >= 0x1000 || vlan_pcp > 7 ||
		 //(eth_type >= 0x10000 && udp_dest_port >= 0x10000) || 
		 (eth_type <  0x10000 && udp_dest_port <  0x10000)  ) {
	    logprint(LOG_LVL_WARNING, "Ethertype filter cannot be mixed with UDP filter");
		return -EINVAL;
	}

	if( dst_mac && udp_dest_port < 0x10000  ) {
	    logprint(LOG_LVL_WARNING, "Destination MAC Ethertype filter cannot be mixed with UDP filter");
		return -EINVAL;
	}

	if (atl_lock(dev) != 0)
		return errno;

	memset(fsp, 0, sizeof(*fsp));

	fsp->location = RX_CLS_LOC_ANY;
	fsp->flow_type = 0;
	fsp->ring_cookie = rxr->index;
	if( vlan_id < 0x1000 || vlan_pcp < 8 ){
		fsp->h_ext.vlan_tci = htobe16(vlan_id | (vlan_pcp << 13));
		fsp->m_ext.vlan_tci = htobe16((vlan_id > 0 ? 0xfff : 0) | ((vlan_pcp > 0 ? 0x7 : 0) << 13));
		fsp->h_ext.vlan_etype = 0x8100;
		fsp->flow_type |= FLOW_EXT;
	}

	if( eth_type < 0x10000 || dst_mac ){
		fsp->flow_type |= ETHER_FLOW;
		if( eth_type < 0x10000 ) {
			fsp->h_u.ether_spec.h_proto = htobe16(eth_type);
			fsp->m_u.ether_spec.h_proto = 0xffff;
		}
		if( dst_mac ) {
			memcpy(fsp->h_u.ether_spec.h_dest, dst_mac, ETH_ALEN);
			memset(fsp->m_u.ether_spec.h_dest, 0xff, ETH_ALEN);
		}
	} else {
		if( ipv4 ) {
			fsp->flow_type |= UDP_V4_FLOW;
			fsp->h_u.udp_ip4_spec.pdst = htobe16(udp_dest_port);
			fsp->m_u.udp_ip4_spec.pdst = 0xffff;
		} else {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 6, 0)
			return -ENXIO;
#else
			fsp->flow_type |= UDP_V6_FLOW;
			fsp->h_u.udp_ip6_spec.pdst = htobe16(udp_dest_port);
			fsp->m_u.udp_ip6_spec.pdst = 0xffff;
#endif
		}
	}
	dump(LOG_LVL_DEBUG, "New filter: ", (uint8_t *)fsp, sizeof(*fsp));
	(void)strncpy(ifr.ifr_name, dev->ifname, sizeof(ifr.ifr_name)-1);
    nfccmd.cmd = ETHTOOL_SRXCLSRLINS;
    ifr.ifr_data = (void*) &nfccmd;
	if (ioctl(adapter->sd, SIOCETHTOOL, &ifr) < 0) {
		logprint(LOG_LVL_ERROR, "Cannot apply filter configuration: %s", strerror(errno));
		error = -ENXIO;
	}
	else {
		if ( filter_id )
			*filter_id = fsp->location;
	}

	if (atl_unlock(dev) != 0)
		error = errno;

	return error;
}

int atl_clear_filter(device_t *dev, unsigned int filter_id)
{
	int error = 0;
	struct atl_adapter *adapter;
	struct rx_ring *rxr;
	struct ifreq ifr;
	struct ethtool_rxnfc nfccmd;
	struct ethtool_rx_flow_spec *fsp =
		(struct ethtool_rx_flow_spec *)&nfccmd.fs;
	if (dev == NULL)
		return -EINVAL;

	adapter = (struct atl_adapter *)dev->private_data;
	if (adapter == NULL)
		return -ENXIO;

	if (atl_lock(dev) != 0)
		return errno;

	memset(fsp, 0, sizeof(*fsp));

	fsp->location = filter_id;
	(void)strncpy(ifr.ifr_name, dev->ifname, sizeof(ifr.ifr_name)-1);
    nfccmd.cmd = ETHTOOL_SRXCLSRLDEL;
    ifr.ifr_data = (void*) &nfccmd;
	if (ioctl(adapter->sd, SIOCETHTOOL, &ifr) < 0) {
		logprint(LOG_LVL_ERROR, "Cannot remove filter configuration: %s", strerror(errno));
		error -ENXIO;
	}

	if (atl_unlock(dev) != 0)
		error = errno;

	return error;
}

static int atl_share_state(struct atl_adapter *adapter)
{
	int error = -1;
	int fd = -1;
	bool locked = false;
	struct flock fl;
	struct stat stat;
	mode_t fmode = S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP;
	uint8_t *shmem = NULL;
	bool attr_allocated = false;
	pthread_mutexattr_t attr;
	char atl_sem_name[PATH_MAX];

	if (!adapter) {
		errno = EINVAL;
		goto err;
	}

	if (adapter->memlock) {	// already created
		errno = EINVAL;
		goto err;
	}

	/*
	 * inter-process syncronization
	 *
	 * Use a posix mutex for inter-process syncronization
	 *
	 * atl lib used a posix named semaphore to protect concurrent accesses
	 * from multiple processes. But since the posix semaphore cannot be
	 * automatically released on process termination, if some process holding
	 * the semaphore terminates without releasing it, other processes cannot
	 * acquire the semaphore afterward. This could potentially cause a denial
	 * of service condition.
	 */
	snprintf(atl_sem_name, sizeof(atl_sem_name), "/atl_sem_%s", adapter->ifname);
	fd = shm_open(atl_sem_name, O_RDWR|O_CREAT|O_CLOEXEC, fmode);
	if (fd < 0 )
		goto err;

	(void) fchmod(fd, fmode); // just to make sure fmode is applied

	// shared memory holding the mutex instance
	shmem = (uint8_t *) mmap(NULL, ATL_SHARED_MEM,
							PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
	if (!shmem)
		goto err;
	
	adapter->memlock = (pthread_mutex_t*)shmem;
	adapter->aq_shmem = (struct atl_share_mem *)(shmem + sizeof(pthread_mutex_t));
	/*
	 * Exclusive access lock
	 *
	 * At this moment the posix mutex in the shared memory might not be
	 * available yet and it needs initialization. We need to protect the
	 * initialization code otherwise multiple processes could concurrently
	 * do initialization. Create an exclusive access section by applying
	 * the file-lock against the shared memory file.
	 *
	 * By the way the file-lock itself could safely be used for inter-process
	 * synchronization because it also automatically gets unlocked on process
	 * termination. But it is slower than the posix-mutex. So we should use
	 * the posix-mutex once its initialization done.
	 */
	fl.l_type = F_WRLCK;
	fl.l_whence = SEEK_SET;
	fl.l_start = 0;
	fl.l_len = 1;
	fl.l_pid = getpid();

	if (fcntl(fd, F_SETLKW, &fl) != 0)
		goto err;

	locked = true;

	if (fstat(fd, &stat) != 0)
		goto err;

	if (stat.st_size == 0) { // file is empty, do initialization
		/*
		 * file-size becomes non-zero and given that when other processes
		 * attach lib atl we can skip the initialization code for the mutex
		 * and other objects.
		 */
		if (ftruncate(fd, sizeof(pthread_mutex_t)) != 0)
			goto err;

		if (pthread_mutexattr_init(&attr) != 0)
			goto err;

		attr_allocated = true;

		// to be used for both inter-process and inter-thread synchronization
		if (pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED) != 0)
			goto err;

		// to avoid dead lock due to a dead process holding the semaphore
		if (pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST) != 0)
			goto err;

		if (pthread_mutex_init(adapter->memlock, &attr) != 0)
			goto err;
		memset(adapter->aq_shmem, 0, sizeof(*adapter->aq_shmem));

		logprint(LOG_LVL_VERBOSE, "Create new lock and shared state");
	}
	error = 0;
err:
	// no actual effect but to avoid a warning from a static code analyzer
	if (attr_allocated)
		(void) pthread_mutexattr_destroy(&attr);

	if (error != 0) {
		error = -errno;
		if (adapter && adapter->memlock) {
			(void) munmap(adapter->memlock, ATL_SHARED_MEM);
			adapter->memlock = NULL;
			adapter->aq_shmem = NULL;
		}
	}

	if (fd >= 0) {
		if (locked) {
			fl.l_type = F_UNLCK;
			(void) fcntl(fd, F_SETLK, &fl);
		}
		(void) close(fd);
	}

	return error;
}
//EOF
