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

#ifndef _ATL_H_DEFINED_
#define _ATL_H_DEFINED_

#include <stdarg.h>
#include <sys/types.h>

struct resource {
	u_int64_t paddr;
	u_int32_t mmap_size;
};

#define MAX_DEVICE_COUNT_IN_SYSTEM 4

#define ATL_AVB_RING_SIZE   2048
#define ATL_AVB_BUFFER_SIZE 2048

#ifndef _LINUX_LIST_H
struct list_head {
	struct list_head *next, *prev;
};
#endif
typedef struct _device_t {
	void *private_data;
	int class_fd;
	char *ifname;
	char *driver_class;
	u_int16_t vendor;
	u_int16_t device;
	u_int16_t subvend;
	u_int16_t subdev;
	u_int8_t revision;
	u_int16_t domain;
	u_int8_t bus;
	u_int8_t dev;
	u_int8_t func;

	unsigned index;
	struct list_head list;
} device_t;


struct atl_packet {
	struct resource map;	/* bus_dma map for packet */
	unsigned int offset;	/* offset into physical page */
	void *vaddr;
	u_int32_t len;
	u_int32_t flags;
	u_int64_t attime;	/* launchtime */
	u_int64_t hwtime;	/* when hardware sent/received packet */
	u_int16_t vlan;     /* vlan tag insertion */
	u_int16_t ct_idx;   /* context index for few vlans */
	u_int64_t private;  /* TX: first and last descriptor index 
						   RX: next buffer index for multi-segment packets */
	void *extra; 		/* Some additional pointer */
	struct atl_packet *next;	/* used in the clean routine */
};

#define ATL_PKT_FLAG_HW_TS_VLD 1

struct atl_dma_alloc {
	u_int64_t dma_paddr;
	void *dma_vaddr;
	unsigned int mmap_size;
};

typedef void (*avb_log_func)(
	int level,
	const char *tag,
	const char *company,
	const char *component,
	const char *path,
	int line,
	const char *fmt,
	va_list vargs);

void atl_init_avb_log(avb_log_func log_func, const char *tag);

int atl_probe(device_t *dev);
int atl_attach(device_t *pdev);
int atl_attach_rx(device_t *pdev);
int atl_attach_tx(device_t *pdev);
int atl_detach(device_t *dev);
int atl_init(device_t *dev);
int atl_dma_malloc_page(device_t *dev, struct atl_dma_alloc *page);
void atl_dma_free_page(device_t *dev, struct atl_dma_alloc *page);

int atl_xmit(device_t *dev, unsigned int queue_index,
	     struct atl_packet **packet);
int atl_refresh_buffers(device_t *dev, u_int32_t idx,
			 struct atl_packet **rxbuf_packets,
			 u_int32_t *num_bufs);
int atl_receive(device_t *dev, unsigned int queue_index, 
	     struct atl_packet **received_packets, u_int32_t *count);
int atl_get_xmit_space(device_t *dev, unsigned int queue_index);
int atl_clean(device_t *dev, struct atl_packet **cleaned_packets);
int atl_stop_tx(device_t *dev, unsigned int queue_index, struct atl_packet **cleaned_packets);
int atl_stop_rx(device_t *dev, unsigned int queue_index, struct atl_packet **cleaned_packets);

int atl_get_wallclock(device_t *dev, u_int64_t *curtime, u_int64_t *rdtsc);

int atl_set_class_bandwidth(device_t *dev, u_int32_t class_a_bytes_per_second,
			     u_int32_t class_b_bytes_per_second);
int atl_setup_filter(device_t *dev, unsigned int queue_index,
			unsigned int vlan_id, unsigned int vlan_pcp,
			unsigned int eth_type, unsigned int udp_dest_port, 
			unsigned int ipv4, const unsigned char dst_mac[6], 
			unsigned int *filter_id);
int atl_clear_filter(device_t *dev, unsigned int filter_id);
int atl_lock(device_t *dev);
int atl_unlock(device_t *dev);

int atl_getpagesize();

#include "log.h"

#endif /* _ATL_H_DEFINED_ */
