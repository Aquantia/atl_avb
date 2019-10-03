/*
 * aQuantia Corporation Network Driver
 * Copyright (C) 2014-2019 aQuantia Corporation. All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

/* File hw_atl2_utils_fw2x.c: Definition of firmware 2.x functions for
 * Atlantic hardware abstraction layer.
 */

#include "../aq_hw.h"
#include "../aq_hw_utils.h"
#include "../aq_pci_func.h"
#include "../aq_ring.h"
#include "../aq_vec.h"
#include "../aq_nic.h"
#include "hw_atl2_utils.h"
#include "hw_atl2_llh.h"
#include "hw_atl2_internal.h"

#define AQ_A2_FW_READ_TRY_MAX 1000

#define  hw_atl2_shared_buffer_write(HW, ITEM, VARIABLE) \
	hw_atl2_mif_shared_buf_write(HW,\
		(offsetof(struct fw_interface_in, ITEM) / sizeof(u32)),\
		(u32 *)&VARIABLE, sizeof(VARIABLE) / sizeof(u32))

#define  hw_atl2_shared_buffer_get(HW, ITEM, VARIABLE) \
	hw_atl2_mif_shared_buf_get(HW, \
		(offsetof(struct fw_interface_in, ITEM) / sizeof(u32)),\
		(u32 *)&VARIABLE, \
		sizeof(VARIABLE) / sizeof(u32))

#define  hw_atl2_shared_buffer_read(HW, ITEM, VARIABLE) \
	hw_atl2_mif_shared_buf_read(HW, \
		(offsetof(struct fw_interface_out, ITEM) / sizeof(u32)),\
		(u32 *)&VARIABLE, \
		sizeof(VARIABLE) / sizeof(u32))

#define  hw_atl2_shared_buffer_read_item_fn_decl(ITEM) \
static int hw_atl2_mif_shared_buf_read_item_##ITEM(struct aq_hw_s *self,\
						   void *data)\
{\
	hw_atl2_mif_shared_buf_read(self, \
		(offsetof(struct fw_interface_out, ITEM) / sizeof(u32)),\
		(u32 *)data,\
		sizeof(((struct fw_interface_out *)0)->ITEM) / sizeof(u32));\
	return 0;\
}

#define  hw_atl2_shared_buffer_read_item_fn_name(ITEM) \
	hw_atl2_mif_shared_buf_read_item_##ITEM

static int hw_atl2_shared_buffer_read_safe(struct aq_hw_s *self,
		     int (*read_item_fn)(struct aq_hw_s *self, void *data),
		     void *data)
{
	struct transaction_counter_s tid1, tid2;
	int cnt = 0;

	do {
		do {
			hw_atl2_shared_buffer_read(self, transactoin_id, tid1);
			cnt++;
			if (cnt > AQ_A2_FW_READ_TRY_MAX)
				return -ETIME;
			if (tid1.transaction_cnt_a != tid1.transaction_cnt_b)
				udelay(1);
		} while (tid1.transaction_cnt_a != tid1.transaction_cnt_b);

		read_item_fn(self, data);

		hw_atl2_shared_buffer_read(self, transactoin_id, tid2);

		cnt++;
		if (cnt > AQ_A2_FW_READ_TRY_MAX)
			return -ETIME;
	} while (tid2.transaction_cnt_a != tid2.transaction_cnt_b ||
		 tid1.transaction_cnt_a != tid2.transaction_cnt_a);

	return 0;
}

static int aq_a2_fw_init(struct aq_hw_s *self)
{
	struct link_control_s link_control;
	int err = 0;
	u32 mcp_read_status;

	BUILD_BUG_ON_MSG(sizeof(struct link_options_s) != 0x4,
			 "linkOptions invalid size");
	BUILD_BUG_ON_MSG(sizeof(struct thermal_shutdown_s) != 0x4,
			 "thermalShutdown invalid size");
	BUILD_BUG_ON_MSG(sizeof(struct sleep_proxy_s) != 0x958,
			 "sleepProxy invalid size");
	BUILD_BUG_ON_MSG(sizeof(struct pause_quanta_s) != 0x18,
			 "pauseQuanta invalid size");
	BUILD_BUG_ON_MSG(sizeof(struct cable_diag_control_s) != 0x4,
			 "cableDiagControl invalid size");

	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_in, mtu) != 0,
			 "mtu invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_in, mac_address) != 0x8,
			 "macAddress invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_in,
				  link_control) != 0x10,
			 "linkControl invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_in,
				  link_options) != 0x18,
			 "linkOptions invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_in,
				  thermal_shutdown) != 0x20,
			 "thermalShutdown invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_in, sleep_proxy) != 0x28,
			 "sleepProxy invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_in,
				  pause_quanta) != 0x984,
			 "pauseQuanta invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_in,
				  cable_diag_control) != 0xA44,
			 "cableDiagControl invalid offset");

	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_out, version) != 0x04,
			 "version invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_out, link_status) != 0x14,
			 "linkStatus invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_out,
				  wol_status) != 0x18,
			 "wolStatus invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_out,
				  mac_health_monitor) != 0x610,
			 "macHealthMonitor invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_out,
				  phy_health_monitor) != 0x620,
			 "phyHealthMonitor invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_out,
				  cable_diag_status) != 0x630,
			 "cableDiagStatus invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_out,
				  device_link_caps) != 0x648,
			 "deviceLinkCaps invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_out,
				 sleep_proxy_caps) != 0x650,
			 "sleepProxyCaps invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_out,
				  lkp_link_caps) != 0x660,
			 "lkpLinkCaps invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_out, core_dump) != 0x668,
			 "coreDump invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_out,
				  mem_box_status) != 0x700,
			 "memBoxStatus invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_out,
				  phy_fw_load_status) != 0x70C,
			 "phyFwLoadStatus invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_out, stats) != 0x718,
			 "stats invalid offset");
	BUILD_BUG_ON_MSG(offsetof(struct fw_interface_out, trace) != 0xC00,
			 "trace invalid offset");

	hw_atl2_shared_buffer_get(self, link_control, link_control);
	link_control.mode = AQ_HOST_MODE_ACTIVE;

	hw_atl2_shared_buffer_write(self, link_control, link_control);
	hw_atl2_mif_host_finished_write_set(self, 1U);

	readx_poll_timeout_atomic(hw_atl2_mif_mcp_finished_read_get,
		self,
		mcp_read_status,
		mcp_read_status == 0,
		1U,
		1000U);

	return err;
}

static int aq_a2_fw_deinit(struct aq_hw_s *self)
{
	struct link_control_s link_control;
	int err = 0;
	u32 mcp_read_status;

	hw_atl2_shared_buffer_get(self, link_control, link_control);
	link_control.mode = AQ_HOST_MODE_SHUTDOWN;

	hw_atl2_shared_buffer_write(self, link_control, link_control);
	hw_atl2_mif_host_finished_write_set(self, 1U);

	readx_poll_timeout_atomic(hw_atl2_mif_mcp_finished_read_get,
		self,
		mcp_read_status,
		mcp_read_status == 0,
		1U,
		1000U);
	return err;
}


static void a2_link_speed_mask2fw(u32 speed,
				  struct link_options_s *link_options)
{
	link_options->rate_10G = !!(speed & AQ_NIC_RATE_10G);
	link_options->rate_5G = !!(speed & AQ_NIC_RATE_5G);
	link_options->rate_N5G = !!(speed & AQ_NIC_RATE_5GSR);
	link_options->rate_2P5G = !!(speed & AQ_NIC_RATE_2GS);
	link_options->rate_1G = !!(speed & AQ_NIC_RATE_1G);
	link_options->rate_100M = !!(speed & AQ_NIC_RATE_100M);
}

static u32 a2_fw_dev_to_eee_mask(struct device_link_caps_s *device_link_caps)
{
	u32 rate = 0;

	if (device_link_caps->eee_10G)
		rate |= AQ_NIC_RATE_EEE_10G;
	if (device_link_caps->eee_5G)
		rate |= AQ_NIC_RATE_EEE_5G;
	if (device_link_caps->eee_2P5G)
		rate |= AQ_NIC_RATE_EEE_2GS;
	if (device_link_caps->eee_1G)
		rate |= AQ_NIC_RATE_EEE_1G;

	return rate;
}

static u32 a2_fw_lkp_to_mask(struct lkp_link_caps_s *lkp_link_caps)
{
	u32 rate = 0;

	if (lkp_link_caps->rate_10G)
		rate |= AQ_NIC_RATE_10G;
	if (lkp_link_caps->rate_5G)
		rate |= AQ_NIC_RATE_5G;
	if (lkp_link_caps->rate_N5G)
		rate |= AQ_NIC_RATE_5GSR;
	if (lkp_link_caps->rate_2P5G)
		rate |= AQ_NIC_RATE_2GS;
	if (lkp_link_caps->rate_1G)
		rate |= AQ_NIC_RATE_1G;
	if (lkp_link_caps->rate_100M)
		rate |= AQ_NIC_RATE_100M;

	if (lkp_link_caps->eee_10G)
		rate |= AQ_NIC_RATE_EEE_10G;
	if (lkp_link_caps->eee_5G)
		rate |= AQ_NIC_RATE_EEE_5G;
	if (lkp_link_caps->eee_2P5G)
		rate |= AQ_NIC_RATE_EEE_2GS;
	if (lkp_link_caps->eee_1G)
		rate |= AQ_NIC_RATE_EEE_1G;

	return rate;
}

static int aq_a2_fw_set_link_speed(struct aq_hw_s *self, u32 speed)
{
	struct link_options_s link_options;
	int err = 0;
	u32 mcp_read_status;

	hw_atl2_shared_buffer_get(self, link_options, link_options);
	link_options.link_up = 1U;

	a2_link_speed_mask2fw(speed, &link_options);

	hw_atl2_shared_buffer_write(self, link_options, link_options);
	hw_atl2_mif_host_finished_write_set(self, 1U);
	readx_poll_timeout_atomic(hw_atl2_mif_mcp_finished_read_get,
		self,
		mcp_read_status,
		mcp_read_status == 0,
		1U,
		1000U);

	return err;
}

static void aq_a2_fw_set_mpi_flow_control(struct aq_hw_s *self,
					 struct link_options_s *link_options)
{
	u32 flow_control = self->aq_nic_cfg->fc.req;

	link_options->pause_rx = !!(flow_control & AQ_NIC_FC_RX);
	link_options->pause_tx = !!(flow_control & AQ_NIC_FC_TX);
}

static void aq_a2_fw_upd_eee_rate_bits(struct aq_hw_s *self,
				      struct link_options_s *link_options,
				      u32 eee_speeds)
{
	link_options->eee_10G =  !!(eee_speeds & AQ_NIC_RATE_EEE_10G);
	link_options->eee_5G = !!(eee_speeds & AQ_NIC_RATE_EEE_5G);
	link_options->eee_2P5G = !!(eee_speeds & AQ_NIC_RATE_EEE_2GS);
	link_options->eee_1G = !!(eee_speeds & AQ_NIC_RATE_EEE_1G);
}

static int aq_a2_fw_set_state(struct aq_hw_s *self,
			     enum hal_atl_utils_fw_state_e state)
{
	struct link_options_s link_options;
	int err = 0;
	u32 mcp_read_status;

	hw_atl2_shared_buffer_get(self, link_options, link_options);

	switch (state) {
	case MPI_INIT:
		link_options.link_up = 1U;
		aq_a2_fw_upd_eee_rate_bits(self, &link_options,
					   self->aq_nic_cfg->eee_speeds);
		aq_a2_fw_set_mpi_flow_control(self, &link_options);
		break;
	case MPI_DEINIT:
		link_options.link_up = 0U;
		break;
	case MPI_RESET:
	case MPI_POWER:
		/* No actions */
		break;
	}

	hw_atl2_shared_buffer_write(self, link_options, link_options);
	hw_atl2_mif_host_finished_write_set(self, 1U);

	readx_poll_timeout_atomic(hw_atl2_mif_mcp_finished_read_get,
		self,
		mcp_read_status,
		mcp_read_status == 0,
		1U,
		1000U);

	return err;
}

static int aq_a2_fw_update_link_status(struct aq_hw_s *self)
{
	struct link_status_s link_status;
	struct lkp_link_caps_s lkp_link_caps;

	hw_atl2_shared_buffer_read(self, link_status, link_status);

	switch (link_status.link_rate) {
	case AQ_A2_FW_LINK_RATE_10G:
		self->aq_link_status.mbps = 10000;
		break;
	case AQ_A2_FW_LINK_RATE_5G:
		self->aq_link_status.mbps = 5000;
		break;
	case AQ_A2_FW_LINK_RATE_2G5:
		self->aq_link_status.mbps = 2500;
		break;
	case AQ_A2_FW_LINK_RATE_1G:
		self->aq_link_status.mbps = 1000;
		break;
	case AQ_A2_FW_LINK_RATE_100M:
		self->aq_link_status.mbps = 100;
		break;
	case AQ_A2_FW_LINK_RATE_10M:
		self->aq_link_status.mbps = 10;
		break;
	default:
		self->aq_link_status.mbps = 0;
	}

	hw_atl2_shared_buffer_read(self, lkp_link_caps, lkp_link_caps);
	/*
	self->aq_link_status.lp_link_speed_msk =
				 a2_fw_lkp_to_mask(&lkp_link_caps);
	self->aq_link_status.lp_flow_control =
				((lkp_link_caps.pause_rx) ? AQ_NIC_FC_RX : 0) |
				((lkp_link_caps.pause_tx) ? AQ_NIC_FC_TX : 0);
	*/
	return 0;
}

int aq_a2_fw_get_mac_permanent(struct aq_hw_s *self, u8 *mac)
{
	struct mac_address_s mac_address;
	int err = 0;
	u32 h = 0U;
	u32 l = 0U;

	hw_atl2_shared_buffer_get(self, mac_address, mac_address);

	ether_addr_copy(mac, (u8 *)mac_address.mac_address);

	if ((mac[0] & 0x01U) || ((mac[0] | mac[1] | mac[2]) == 0x00U) ||
	    (mac[0] == mac[3] && mac[3] == mac[4] && mac[1] == mac[5] &&
		 mac[0] == 0xde && mac[1] == 0xc0 && mac[2] == 0xad) ) {
		unsigned int rnd = 0;

		get_random_bytes(&rnd, sizeof(unsigned int));

		l = 0xE3000000U
			| (0xFFFFU & rnd)
			| (0x00 << 16);
		h = 0x8001300EU;

		mac[5] = (u8)(0xFFU & l);
		l >>= 8;
		mac[4] = (u8)(0xFFU & l);
		l >>= 8;
		mac[3] = (u8)(0xFFU & l);
		l >>= 8;
		mac[2] = (u8)(0xFFU & l);
		mac[1] = (u8)(0xFFU & h);
		h >>= 8;
		mac[0] = (u8)(0xFFU & h);
	}
	return err;
}

hw_atl2_shared_buffer_read_item_fn_decl(stats);

static struct hw_atl2_priv atl2_priv;
static int aq_a2_fw_update_stats(struct aq_hw_s *self)
{
	struct hw_atl2_priv *priv = &atl2_priv;
	struct statistics_s stats;

	hw_atl2_shared_buffer_read_safe(self,
		hw_atl2_shared_buffer_read_item_fn_name(stats),
		&stats);

#define AQ_SDELTA(_N_, _F_) (self->curr_stats._N_ += \
			stats.msm._F_ - priv->last_stats.msm._F_)

	if (self->aq_link_status.mbps) {
		AQ_SDELTA(uprc, rx_unicast_frames);
		AQ_SDELTA(mprc, rx_multicast_frames);
		AQ_SDELTA(bprc, rx_broadcast_frames);
		AQ_SDELTA(erpr, rx_dropped_frames);

		AQ_SDELTA(uptc, tx_unicast_frames);
		AQ_SDELTA(mptc, tx_multicast_frames);
		AQ_SDELTA(bptc, tx_broadcast_frames);
		AQ_SDELTA(erpt, tx_errors);

		AQ_SDELTA(ubrc, rx_unicast_octets);
		AQ_SDELTA(ubtc, tx_unicast_octets);
		AQ_SDELTA(mbrc, rx_multicast_octets);
		AQ_SDELTA(mbtc, tx_multicast_octets);
		AQ_SDELTA(bbrc, rx_broadcast_octets);
		AQ_SDELTA(bbtc, tx_broadcast_octets);
	}
#undef AQ_SDELTA
	self->curr_stats.dma_pkt_rc =
		hw_atl2_stats_rx_dma_good_pkt_counterlsw_get(self) +
		((u64)hw_atl2_stats_rx_dma_good_pkt_countermsw_get(self) << 32);
	self->curr_stats.dma_pkt_tc =
		hw_atl2_stats_tx_dma_good_pkt_counterlsw_get(self) +
		((u64)hw_atl2_stats_tx_dma_good_pkt_countermsw_get(self) << 32);
	self->curr_stats.dma_oct_rc =
		hw_atl2_stats_rx_dma_good_octet_counterlsw_get(self) +
		((u64)hw_atl2_stats_rx_dma_good_octet_countermsw_get(self) << 32);
	self->curr_stats.dma_oct_tc =
		hw_atl2_stats_tx_dma_good_octet_counterlsw_get(self) +
		((u64)hw_atl2_stats_tx_dma_good_octet_countermsw_get(self) << 32);
	self->curr_stats.dpc = hw_atl2_rpb_rx_dma_drop_pkt_cnt_get(self);

	memcpy(&priv->last_stats, &stats, sizeof(stats));

	return 0;
}

hw_atl2_shared_buffer_read_item_fn_decl(phy_health_monitor);

static int aq_a2_fw_get_temp(struct aq_hw_s *self, int *temp)
{
	struct phy_health_monitor_s phy_health_monitor;

	hw_atl2_shared_buffer_read_safe(self,
		hw_atl2_shared_buffer_read_item_fn_name(phy_health_monitor),
		&phy_health_monitor);

	*temp = phy_health_monitor.phy_temperature  * 100 / 256;
	return 0;
}

hw_atl2_shared_buffer_read_item_fn_decl(cable_diag_status);

static int aq_a2_fw_get_cable_len(struct aq_hw_s *self, int *cable_len)
{
	struct cable_diag_control_s cable_diag_control;
	struct cable_diag_status_s cable_diag_status;
	u32 mcp_read_status;
	int err = 0;

	hw_atl2_shared_buffer_get(self, cable_diag_control, cable_diag_control);
	cable_diag_control.toggle ^= 1U;

	hw_atl2_shared_buffer_write(self, cable_diag_control,
				    cable_diag_control);
	hw_atl2_mif_host_finished_write_set(self, 1U);
	readx_poll_timeout_atomic(hw_atl2_mif_mcp_finished_read_get,
		self,
		mcp_read_status,
		mcp_read_status == 0,
		1U,
		1000U);

	do {
		unsigned int AQ_HW_WAIT_FOR_i;
		for (AQ_HW_WAIT_FOR_i = 1000U; AQ_HW_WAIT_FOR_i; --AQ_HW_WAIT_FOR_i) {
			err = hw_atl2_shared_buffer_read_safe(self,
					hw_atl2_shared_buffer_read_item_fn_name(cable_diag_status),
					&cable_diag_status);
			if( (cable_diag_status.state == 1U) || err ) {
				break;
			}
			udelay(1U);
		}
		if (!AQ_HW_WAIT_FOR_i) {
			err = -ETIME; 
		}
	} while (0);

	if (err)
		goto exit;

	*cable_len = cable_diag_status.lane_data[0].dist;
exit:
	return err;
}
/*
static int aq_a2_fw_set_wol_params(struct aq_hw_s *self, u8 *mac)
{
	struct wake_on_lan_s wake_on_lan;
	struct mac_address_s mac_address;
	int err = 0;
	u32 mcp_read_status;

	hw_atl2_shared_buffer_get(self, sleep_proxy, wake_on_lan);

	wake_on_lan.wake_on_magic_packet = 1U;
	memcpy(mac_address.mac_address, mac, ETH_ALEN);

	hw_atl2_shared_buffer_write(self, mac_address, mac_address);
	hw_atl2_shared_buffer_write(self, sleep_proxy, wake_on_lan);
	hw_atl2_mif_host_finished_write_set(self, 1U);
	readx_poll_timeout_atomic(hw_atl2_mif_mcp_finished_read_get,
		self,
		mcp_read_status,
		mcp_read_status == 0,
		1U,
		1000U);


	return err;
}

static int aq_a2_fw_set_power(struct aq_hw_s *self, unsigned int power_state,
			     u8 *mac)
{
	int err = 0;

	if (self->aq_nic_cfg->wol & AQ_NIC_WOL_ENABLED)
		err = aq_a2_fw_set_wol_params(self, mac);

	return err;
}
*/

static int aq_a2_fw_set_eee_rate(struct aq_hw_s *self, u32 speed)
{
	struct link_options_s link_options;
	int err = 0;
	u32 mcp_read_status;

	hw_atl2_shared_buffer_get(self, link_options, link_options);

	aq_a2_fw_upd_eee_rate_bits(self, &link_options, speed);


	hw_atl2_shared_buffer_write(self, link_options, link_options);
	hw_atl2_mif_host_finished_write_set(self, 1U);
	readx_poll_timeout_atomic(hw_atl2_mif_mcp_finished_read_get,
		self,
		mcp_read_status,
		mcp_read_status == 0,
		1U,
		1000U);

	return err;
}

static int aq_a2_fw_get_eee_rate(struct aq_hw_s *self, u32 *rate,
				u32 *supported_rates)
{
	struct device_link_caps_s device_link_caps;
	struct lkp_link_caps_s lkp_link_caps;
	//uint32_t lpi;
	int err = 0;


	hw_atl2_shared_buffer_read(self, device_link_caps, device_link_caps);
	hw_atl2_shared_buffer_read(self, lkp_link_caps, lkp_link_caps);

	*supported_rates = a2_fw_dev_to_eee_mask(&device_link_caps);
	*rate = a2_fw_lkp_to_mask(&lkp_link_caps);
/*
	err = hw_atl_msm_read_lpi_timer(self, &lpi);
	if (err)
		return err;
*/
/* clock is 3.2 ns*/
	//#define HW_ATL_CLOCK_TO_US(clk)  ((clk) * 32 / 10000)
	//*lpi_timer = HW_ATL_CLOCK_TO_US(lpi);

	return err;
}

static int aq_a2_fw_renegotiate(struct aq_hw_s *self)
{
	struct link_options_s link_options;
	int err = 0;
	u32 mcp_read_status;

	hw_atl2_shared_buffer_get(self, link_options, link_options);
	link_options.link_renegotiate = 1U;

	hw_atl2_shared_buffer_write(self, link_options, link_options);
	hw_atl2_mif_host_finished_write_set(self, 1U);
	readx_poll_timeout_atomic(hw_atl2_mif_mcp_finished_read_get,
		self,
		mcp_read_status,
		mcp_read_status == 0,
		1U,
		1000U);

	return err;
}

static int aq_a2_fw_set_flow_control(struct aq_hw_s *self)
{
	struct link_options_s link_options;
	int err = 0;
	u32 mcp_read_status;

	hw_atl2_shared_buffer_get(self, link_options, link_options);

	aq_a2_fw_set_mpi_flow_control(self, &link_options);

	hw_atl2_shared_buffer_write(self, link_options, link_options);
	hw_atl2_mif_host_finished_write_set(self, 1U);
	readx_poll_timeout_atomic(hw_atl2_mif_mcp_finished_read_get,
		self,
		mcp_read_status,
		mcp_read_status == 0,
		1U,
		1000U);

	return err;
}

static u32 aq_a2_fw_get_flow_control(struct aq_hw_s *self, u32 *fcmode)
{
	struct link_status_s link_status;

	hw_atl2_shared_buffer_read(self, link_status, link_status);

	*fcmode = ((link_status.pause_rx) ? AQ_NIC_FC_RX : 0) |
		  ((link_status.pause_tx) ? AQ_NIC_FC_TX : 0);
	return 0;
}

static int aq_a2_fw_set_phyloopback(struct aq_hw_s *self, u32 mode, bool enable)
{
	struct link_options_s link_options;
	int err = 0;
	u32 mcp_read_status;

	hw_atl2_shared_buffer_get(self, link_options, link_options);

	switch (mode) {
	case AQ_HW_LOOPBACK_PHYINT_SYS:
		link_options.internal_loopback = enable;
		break;
	case AQ_HW_LOOPBACK_PHYEXT_SYS:
		link_options.external_loopback = enable;
		break;
	default:
		return -EINVAL;
	}

	hw_atl2_shared_buffer_write(self, link_options, link_options);
	hw_atl2_mif_host_finished_write_set(self, 1U);
	readx_poll_timeout_atomic(hw_atl2_mif_mcp_finished_read_get,
		self,
		mcp_read_status,
		mcp_read_status == 0,
		1U,
		1000U);

	return err;
}

const struct aq_fw_ops aq_a2_fw_ops = {
	.init               = aq_a2_fw_init,
	.deinit             = aq_a2_fw_deinit,
	.reset              = NULL,
	.renegotiate        = aq_a2_fw_renegotiate,
	.get_mac_permanent  = aq_a2_fw_get_mac_permanent,
	.set_link_speed     = aq_a2_fw_set_link_speed,
	.set_state          = aq_a2_fw_set_state,
	.update_link_status = aq_a2_fw_update_link_status,
	.update_stats       = aq_a2_fw_update_stats,
	//.set_power          = aq_a2_fw_set_power,
	.get_phy_temp       = aq_a2_fw_get_temp,
	.get_cable_len      = aq_a2_fw_get_cable_len,
	.set_eee_rate       = aq_a2_fw_set_eee_rate,
	.get_eee_rate       = aq_a2_fw_get_eee_rate,
	.set_flow_control   = aq_a2_fw_set_flow_control,
	.get_flow_control   = aq_a2_fw_get_flow_control,
	.send_fw_request    = NULL,
	.enable_ptp         = NULL,
	.led_control        = NULL,
	.set_phyloopback    = aq_a2_fw_set_phyloopback,
};
