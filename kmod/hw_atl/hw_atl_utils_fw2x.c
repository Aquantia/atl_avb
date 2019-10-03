// SPDX-License-Identifier: GPL-2.0-only
/*
 * aQuantia Corporation Network Driver
 * Copyright (C) 2014-2019 aQuantia Corporation. All rights reserved
 */

/* File hw_atl_utils_fw2x.c: Definition of firmware 2.x functions for
 * Atlantic hardware abstraction layer.
 */

#include "../aq_hw.h"
#include "../aq_hw_utils.h"
#include "../aq_pci_func.h"
#include "../aq_ring.h"
#include "../aq_vec.h"
#include "../aq_nic.h"
#include "hw_atl_utils.h"
#include "hw_atl_llh.h"

#define HW_ATL_FW2X_MPI_LED_ADDR         0x31c
#define HW_ATL_FW2X_MPI_RPC_ADDR         0x334

#define HW_ATL_FW2X_MPI_MBOX_ADDR        0x360
#define HW_ATL_FW2X_MPI_EFUSE_ADDR       0x364
#define HW_ATL_FW2X_MPI_CONTROL_ADDR     0x368
#define HW_ATL_FW2X_MPI_CONTROL2_ADDR    0x36C
#define HW_ATL_FW2X_MPI_STATE_ADDR       0x370
#define HW_ATL_FW2X_MPI_STATE2_ADDR      0x374
#define HW_ATL_FW2X_MPI_CONTROL3_ADDR    0x374

#define HW_ATL_FW3X_EXT_CONTROL_ADDR	 0x378
#define HW_ATL_FW3X_EXT_STATE_ADDR	 0x37c

#define HW_ATL_FW3X_PTP_ADJ_LSW_ADDR	 0x50a0
#define HW_ATL_FW3X_PTP_ADJ_MSW_ADDR	 0x50a4


#define HW_ATL_FW2X_CAP_SLEEP_PROXY      BIT(CAPS_HI_SLEEP_PROXY)
#define HW_ATL_FW2X_CAP_WOL              BIT(CAPS_HI_WOL)

#define HW_ATL_FW2X_CTRL_WAKE_ON_LINK     BIT(CTRL_WAKE_ON_LINK)
#define HW_ATL_FW2X_CTRL_SLEEP_PROXY      BIT(CTRL_SLEEP_PROXY)
#define HW_ATL_FW2X_CTRL_WOL              BIT(CTRL_WOL)
#define HW_ATL_FW2X_CTRL_LINK_DROP        BIT(CTRL_LINK_DROP)
#define HW_ATL_FW2X_CTRL_TEMPERATURE      BIT(CTRL_TEMPERATURE)
#define HW_ATL_FW2X_CTRL_STATISTICS       BIT(CTRL_STATISTICS)
#define HW_ATL_FW2X_CTRL_PAUSE            BIT(CTRL_PAUSE)
#define HW_ATL_FW2X_CTRL_ASYMMETRIC_PAUSE BIT(CTRL_ASYMMETRIC_PAUSE)
#define HW_ATL_FW2X_CTRL_INT_LOOPBACK     BIT(CTRL_INT_LOOPBACK)
#define HW_ATL_FW2X_CTRL_EXT_LOOPBACK     BIT(CTRL_EXT_LOOPBACK)
#define HW_ATL_FW2X_CTRL_FORCE_RECONNECT  BIT(CTRL_FORCE_RECONNECT)
#define HW_ATL_FW2X_CTRL_DOWNSHIFT        BIT(CTRL_DOWNSHIFT)

#define HW_ATL_FW2X_CAP_EEE_1G_MASK      BIT(CAPS_HI_1000BASET_FD_EEE)
#define HW_ATL_FW2X_CAP_EEE_2G5_MASK     BIT(CAPS_HI_2P5GBASET_FD_EEE)
#define HW_ATL_FW2X_CAP_EEE_5G_MASK      BIT(CAPS_HI_5GBASET_FD_EEE)
#define HW_ATL_FW2X_CAP_EEE_10G_MASK     BIT(CAPS_HI_10GBASET_FD_EEE)
#define HW_ATL_FW2X_CAP_PAUSE            BIT(CAPS_HI_PAUSE)
#define HW_ATL_FW2X_CAP_ASYM_PAUSE       BIT(CAPS_HI_ASYMMETRIC_PAUSE)

#define HAL_ATLANTIC_WOL_FILTERS_COUNT   8
#define HAL_ATLANTIC_UTILS_FW2X_MSG_WOL  0x0E

#define HW_ATL_FW_VER_LED                0x03010026U
#define HW_ATL_FW_VER_MEDIA_CONTROL      0x0301005aU

struct __packed fw2x_msg_wol_pattern {
	u8 mask[16];
	u32 crc;
};

struct __packed fw2x_msg_wol {
	u32 msg_id;
	u8 hw_addr[ETH_ALEN];
	u8 magic_packet_enabled;
	u8 filter_count;
	struct fw2x_msg_wol_pattern filter[HAL_ATLANTIC_WOL_FILTERS_COUNT];
	u8 link_up_enabled;
	u8 link_down_enabled;
	u16 reserved;
	u32 link_up_timeout;
	u32 link_down_timeout;
};

static int aq_fw2x_set_link_speed(struct aq_hw_s *self, u32 speed);
static int aq_fw2x_set_state(struct aq_hw_s *self,
			     enum hal_atl_utils_fw_state_e state);

static u32 aq_fw2x_mbox_get(struct aq_hw_s *self);
static u32 aq_fw2x_rpc_get(struct aq_hw_s *self);
static int aq_fw2x_settings_get(struct aq_hw_s *self, u32 *addr);
static u32 aq_fw2x_state2_get(struct aq_hw_s *self);

static int aq_fw2x_init(struct aq_hw_s *self)
{
	int err = 0;

	/* check 10 times by 1ms */
	err = readx_poll_timeout_atomic(aq_fw2x_mbox_get,
					self, self->mbox_addr,
					self->mbox_addr != 0U,
					1000U, 10000U);

	err = readx_poll_timeout_atomic(aq_fw2x_rpc_get,
					self, self->rpc_addr,
					self->rpc_addr != 0U,
					1000U, 100000U);

	err = aq_fw2x_settings_get(self, &self->settings_addr);

	return err;
}

static int aq_fw2x_deinit(struct aq_hw_s *self)
{
	int err = aq_fw2x_set_link_speed(self, 0);

	if (!err)
		err = aq_fw2x_set_state(self, MPI_DEINIT);

	return err;
}

static enum hw_atl_fw2x_rate link_speed_mask_2fw2x_ratemask(u32 speed)
{
	enum hw_atl_fw2x_rate rate = 0;

	if (speed & AQ_NIC_RATE_10G)
		rate |= FW2X_RATE_10G;

	if (speed & AQ_NIC_RATE_5G)
		rate |= FW2X_RATE_5G;

	if (speed & AQ_NIC_RATE_5GSR)
		rate |= FW2X_RATE_5G;

	if (speed & AQ_NIC_RATE_2GS)
		rate |= FW2X_RATE_2G5;

	if (speed & AQ_NIC_RATE_1G)
		rate |= FW2X_RATE_1G;

	if (speed & AQ_NIC_RATE_100M)
		rate |= FW2X_RATE_100M;

	return rate;
}

static u32 fw2x_to_eee_mask(u32 speed)
{
	u32 rate = 0;

	if (speed & HW_ATL_FW2X_CAP_EEE_10G_MASK)
		rate |= AQ_NIC_RATE_EEE_10G;
	if (speed & HW_ATL_FW2X_CAP_EEE_5G_MASK)
		rate |= AQ_NIC_RATE_EEE_5G;
	if (speed & HW_ATL_FW2X_CAP_EEE_2G5_MASK)
		rate |= AQ_NIC_RATE_EEE_2GS;
	if (speed & HW_ATL_FW2X_CAP_EEE_1G_MASK)
		rate |= AQ_NIC_RATE_EEE_1G;

	return rate;
}

static u32 eee_mask_to_fw2x(u32 speed)
{
	u32 rate = 0;

	if (speed & AQ_NIC_RATE_EEE_10G)
		rate |= HW_ATL_FW2X_CAP_EEE_10G_MASK;
	if (speed & AQ_NIC_RATE_EEE_5G)
		rate |= HW_ATL_FW2X_CAP_EEE_5G_MASK;
	if (speed & AQ_NIC_RATE_EEE_2GS)
		rate |= HW_ATL_FW2X_CAP_EEE_2G5_MASK;
	if (speed & AQ_NIC_RATE_EEE_1G)
		rate |= HW_ATL_FW2X_CAP_EEE_1G_MASK;

	return rate;
}

static int aq_fw2x_set_link_speed(struct aq_hw_s *self, u32 speed)
{
	u32 val = link_speed_mask_2fw2x_ratemask(speed);

	aq_hw_write_reg(self, HW_ATL_FW2X_MPI_CONTROL_ADDR, val);

	return 0;
}

static void aq_fw2x_upd_flow_control_bits(struct aq_hw_s *self,
					  u32 *mpi_state, u32 fc)
{
	*mpi_state &= ~(HW_ATL_FW2X_CTRL_PAUSE |
			HW_ATL_FW2X_CTRL_ASYMMETRIC_PAUSE);

	switch (fc) {
	/* There is not explicit mode of RX only pause frames,
	 * thus, we join this mode with FC full.
	 * FC full is either Rx, either Tx, or both.
	 */
	case AQ_NIC_FC_FULL:
	case AQ_NIC_FC_RX:
		*mpi_state |= HW_ATL_FW2X_CTRL_PAUSE |
			      HW_ATL_FW2X_CTRL_ASYMMETRIC_PAUSE;
		break;
	case AQ_NIC_FC_TX:
		*mpi_state |= HW_ATL_FW2X_CTRL_ASYMMETRIC_PAUSE;
		break;
	}
}

static void aq_fw2x_upd_eee_rate_bits(struct aq_hw_s *self,
				      u32 *mpi_opts,
				      u32 eee_speeds)
{
	*mpi_opts &= ~(HW_ATL_FW2X_CAP_EEE_1G_MASK |
		       HW_ATL_FW2X_CAP_EEE_2G5_MASK |
		       HW_ATL_FW2X_CAP_EEE_5G_MASK |
		       HW_ATL_FW2X_CAP_EEE_10G_MASK);

	*mpi_opts |= eee_mask_to_fw2x(eee_speeds);
}

static int aq_fw2x_set_state(struct aq_hw_s *self,
			     enum hal_atl_utils_fw_state_e state)
{
	u32 mpi_state = aq_hw_read_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR);

	switch (state) {
	case MPI_INIT:
		mpi_state &= ~HW_ATL_FW2X_CTRL_LINK_DROP;
		aq_fw2x_upd_eee_rate_bits(self, &mpi_state,
					  self->aq_nic_cfg->eee_speeds);
		aq_fw2x_upd_flow_control_bits(self, &mpi_state,
					      self->aq_nic_cfg->fc.req);
		break;
	case MPI_DEINIT:
		mpi_state |= HW_ATL_FW2X_CTRL_LINK_DROP;
		break;
	case MPI_RESET:
	case MPI_POWER:
		/* No actions */
		break;
	}
	aq_hw_write_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR, mpi_state);
	return 0;
}

static int aq_fw2x_update_link_status(struct aq_hw_s *self)
{
	u32 mpi_state = aq_hw_read_reg(self, HW_ATL_FW2X_MPI_STATE_ADDR);
	u32 speed = mpi_state & (FW2X_RATE_100M | FW2X_RATE_1G |
				 FW2X_RATE_2G5 | FW2X_RATE_5G | FW2X_RATE_10G);
	struct aq_hw_link_status_s *link_status = &self->aq_link_status;

	if (speed) {
		if (speed & FW2X_RATE_10G)
			link_status->mbps = 10000;
		else if (speed & FW2X_RATE_5G)
			link_status->mbps = 5000;
		else if (speed & FW2X_RATE_2G5)
			link_status->mbps = 2500;
		else if (speed & FW2X_RATE_1G)
			link_status->mbps = 1000;
		else if (speed & FW2X_RATE_100M)
			link_status->mbps = 100;
		else
			link_status->mbps = 10000;
	} else {
		link_status->mbps = 0;
	}

	return 0;
}

static int aq_fw2x_get_mac_permanent(struct aq_hw_s *self, u8 *mac)
{
	int err = 0;
	u32 h = 0U;
	u32 l = 0U;
	u32 mac_addr[2] = { 0 };
	u32 efuse_addr = aq_hw_read_reg(self, HW_ATL_FW2X_MPI_EFUSE_ADDR);

	if (efuse_addr != 0) {
		err = hw_atl_utils_fw_downld_dwords(self, efuse_addr +
						    (40U * 4U), mac_addr,
						    ARRAY_SIZE(mac_addr));
		if (err)
			return err;
		mac_addr[0] = __swab32(mac_addr[0]);
		mac_addr[1] = __swab32(mac_addr[1]);
	}

	ether_addr_copy(mac, (u8 *)mac_addr);

	if ((mac[0] & 0x01U) || ((mac[0] | mac[1] | mac[2]) == 0x00U)) {
		unsigned int rnd = 0;

		get_random_bytes(&rnd, sizeof(unsigned int));

		l = 0xE3000000U | (0xFFFFU & rnd) | (0x00 << 16);
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

static int aq_fw2x_update_stats(struct aq_hw_s *self)
{
	u32 mpi_opts = aq_hw_read_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR);
	u32 orig_stats_val = mpi_opts & HW_ATL_FW2X_CTRL_STATISTICS;
	u32 stats_val;
	int err = 0;

	/* Toggle statistics bit for FW to update */
	mpi_opts = mpi_opts ^ HW_ATL_FW2X_CTRL_STATISTICS;
	aq_hw_write_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR, mpi_opts);

	/* Wait FW to report back */
	err = readx_poll_timeout_atomic(aq_fw2x_state2_get,
					self, stats_val,
					orig_stats_val != (stats_val &
					BIT(CAPS_HI_STATISTICS)),
					1U, 10000U);
	if (err)
		return err;

	return hw_atl_utils_update_stats(self);
}

static int aq_fw2x_get_phy_temp(struct aq_hw_s *self, int *temp)
{
	u32 mpi_opts = aq_hw_read_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR);
	u32 temp_val = mpi_opts & HW_ATL_FW2X_CTRL_TEMPERATURE;
	u32 temp_res;
	int err = 0;
	u32 offset;
	u32 val;

	offset = self->mbox_addr + offsetof(struct hw_atl_utils_mbox,
					    info.phy_temperature);

	/* Toggle statistics bit for FW to 0x36C.18 (CTRL_TEMPERATURE) */
	mpi_opts = mpi_opts ^ HW_ATL_FW2X_CTRL_TEMPERATURE;
	aq_hw_write_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR, mpi_opts);

	/* Wait FW to report back */
	err = readx_poll_timeout_atomic(aq_fw2x_state2_get, self, val,
					temp_val !=
					(val & HW_ATL_FW2X_CTRL_TEMPERATURE),
					1U, 10000U);
	err = hw_atl_utils_fw_downld_dwords(self, offset, &temp_res, 1);

	if (err)
		return err;

	/* Convert PHY temperature from 1/256 degree Celsius
	 * to 1/1000 degree Celsius.
	 */
	*temp = (temp_res & 0xFFFF)  * 1000 / 256;

	return 0;
}

static int aq_fw2x_get_cable_len(struct aq_hw_s *self, int *cable_len)
{
	u32 cable_len_res;
	int err = 0;
	u32 offset;

	/* To correct read dword, we begin to read with phy_temperature
	 * (aligned 4)
	 */
	offset = self->mbox_addr + offsetof(struct hw_atl_utils_mbox,
					    info.phy_temperature);

	err = hw_atl_utils_fw_downld_dwords(self, offset, &cable_len_res, 1);

	if (err)
		return err;

	/* Skip the first two bytes, which respond to phy_temperature,
	 * take only the first byte, because cable len is one byte in size
	 */
	*cable_len = (cable_len_res >> 16) & 0xFF;
	return 0;
}

static int aq_fw2x_set_wol(struct aq_hw_s *self, u8 *mac)
{
	struct hw_atl_utils_fw_rpc *rpc = NULL;
	struct offload_info *info = NULL;
	u32 wol_bits = 0;
	u32 rpc_size;
	int err = 0;
	u32 val;

	if (self->aq_nic_cfg->wol & WAKE_PHY) {
		aq_hw_write_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR,
				HW_ATL_FW2X_CTRL_LINK_DROP);
		readx_poll_timeout_atomic(aq_fw2x_state2_get, self, val,
					  (val &
					   HW_ATL_FW2X_CTRL_LINK_DROP) != 0,
					  1000, 100000);
		wol_bits |= HW_ATL_FW2X_CTRL_WAKE_ON_LINK;
	}

	if (self->aq_nic_cfg->wol & WAKE_MAGIC) {
		wol_bits |= HW_ATL_FW2X_CTRL_SLEEP_PROXY |
			    HW_ATL_FW2X_CTRL_WOL;

		err = hw_atl_utils_fw_rpc_wait(self, &rpc);
		if (err < 0)
			goto err_exit;

		rpc_size = sizeof(*info) +
			   offsetof(struct hw_atl_utils_fw_rpc, fw2x_offloads);
		memset(rpc, 0, rpc_size);
		info = &rpc->fw2x_offloads;
		memcpy(info->mac_addr, mac, ETH_ALEN);
		info->len = sizeof(*info);

		err = hw_atl_utils_fw_rpc_call(self, rpc_size);
		if (err < 0)
			goto err_exit;
	}

	aq_hw_write_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR, wol_bits);

err_exit:
	return err;
}

static int aq_fw2x_set_power(struct aq_hw_s *self, unsigned int power_state,
			     u8 *mac)
{
	int err = 0;

	if (self->aq_nic_cfg->wol)
		err = aq_fw2x_set_wol(self, mac);

	return err;
}

static int aq_fw2x_send_fw_request(struct aq_hw_s *self,
				   const struct hw_fw_request_iface *fw_req,
				   size_t size)
{
	u32 ctrl2, orig_ctrl2;
	u32 dword_cnt;
	int err = 0;
	u32 val;

	/* Write data to drvIface Mailbox */
	dword_cnt = size / sizeof(u32);
	if (size % sizeof(u32))
		dword_cnt++;
	err = hw_atl_write_fwcfg_dwords(self, (void *)fw_req, dword_cnt);
	if (err < 0)
		goto err_exit;

	/* Toggle statistics bit for FW to update */
	ctrl2 = aq_hw_read_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR);
	orig_ctrl2 = ctrl2 & BIT(CAPS_HI_FW_REQUEST);
	ctrl2 = ctrl2 ^ BIT(CAPS_HI_FW_REQUEST);
	aq_hw_write_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR, ctrl2);

	/* Wait FW to report back */
	err = readx_poll_timeout_atomic(aq_fw2x_state2_get, self, val,
					orig_ctrl2 != (val &
						       BIT(CAPS_HI_FW_REQUEST)),
					1U, 10000U);

err_exit:
	return err;
}

static void aq_fw3x_enable_ptp(struct aq_hw_s *self, int enable)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 0)) ||\
    (RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(7, 2))
	u32 ptp_opts = aq_hw_read_reg(self, HW_ATL_FW3X_EXT_STATE_ADDR);
	u32 all_ptp_features = BIT(CAPS_EX_PHY_PTP_EN) |
						   BIT(CAPS_EX_PTP_GPIO_EN);

	if (enable)
		ptp_opts |= all_ptp_features;
	else
		ptp_opts &= ~all_ptp_features;

	aq_hw_write_reg(self, HW_ATL_FW3X_EXT_CONTROL_ADDR, ptp_opts);
#endif
}

static void aq_fw3x_adjust_ptp(struct aq_hw_s *self, uint64_t adj)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 0)) ||\
    (RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(7, 2))
	aq_hw_write_reg(self, HW_ATL_FW3X_PTP_ADJ_LSW_ADDR, (adj >>  0) & 0xffffffff);
	aq_hw_write_reg(self, HW_ATL_FW3X_PTP_ADJ_MSW_ADDR, (adj >> 32) & 0xffffffff);
#endif
}

#ifdef TSN_SUPPORT
static void aq_fw3x_enable_tsn(struct aq_hw_s *self, int enable)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 0)) ||\
    (RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(7, 2))
	u32 tpo2_opts = aq_hw_read_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR);
	u32 tsn_opts = aq_hw_read_reg(self, HW_ATL_FW3X_EXT_STATE_ADDR);
	if (enable) {
		tpo2_opts |= IS_CHIP_FEATURE(TPO2) ? BIT(CTRL_TPO2_EN) : 0;
		tsn_opts |= BIT(CAPS_EX_SCHED_DMA_EN);
	} else {
		tpo2_opts &= ~BIT(CTRL_TPO2_EN);
		tsn_opts &= ~BIT(CAPS_EX_SCHED_DMA_EN);
	}
	//printk("Enable TSN in FW: 368 %x 378 %x\n", tpo2_opts, tsn_opts);
	aq_hw_write_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR, tpo2_opts);
	aq_hw_write_reg(self, HW_ATL_FW3X_EXT_CONTROL_ADDR, tsn_opts);
#endif
}
#endif

static int aq_fw2x_led_control(struct aq_hw_s *self, u32 mode)
{
	if (HW_ATL_FW_VER_LED > self->fw_ver_actual)
		return -EOPNOTSUPP;
	aq_hw_write_reg(self, HW_ATL_FW2X_MPI_LED_ADDR, mode);
	return 0;
}

static int aq_fw2x_set_eee_rate(struct aq_hw_s *self, u32 speed)
{
	u32 mpi_opts = aq_hw_read_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR);

	aq_fw2x_upd_eee_rate_bits(self, &mpi_opts, speed);

	aq_hw_write_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR, mpi_opts);

	return 0;
}

static int aq_fw2x_get_eee_rate(struct aq_hw_s *self, u32 *rate,
				u32 *supported_rates)
{
	u32 mpi_state;
	u32 caps_hi;
	int err = 0;
	u32 offset;

	offset = self->mbox_addr + offsetof(struct hw_atl_utils_mbox,
					    info.caps_hi);

	err = hw_atl_utils_fw_downld_dwords(self, offset, &caps_hi, 1);

	if (err)
		return err;

	*supported_rates = fw2x_to_eee_mask(caps_hi);

	mpi_state = aq_fw2x_state2_get(self);
	*rate = fw2x_to_eee_mask(mpi_state);

	return err;
}

static int aq_fw2x_renegotiate(struct aq_hw_s *self)
{
	u32 mpi_opts = aq_hw_read_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR);

	mpi_opts |= HW_ATL_FW2X_CTRL_FORCE_RECONNECT;

	aq_hw_write_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR, mpi_opts);

	return 0;
}

static int aq_fw2x_set_flow_control(struct aq_hw_s *self)
{
	u32 mpi_state = aq_hw_read_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR);

	aq_fw2x_upd_flow_control_bits(self, &mpi_state,
				      self->aq_nic_cfg->fc.req);

	aq_hw_write_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR, mpi_state);

	return 0;
}

static u32 aq_fw2x_get_flow_control(struct aq_hw_s *self, u32 *fcmode)
{
	u32 mpi_state = aq_fw2x_state2_get(self);
	*fcmode = 0;

	if (mpi_state & HW_ATL_FW2X_CAP_PAUSE)
		*fcmode |= AQ_NIC_FC_RX;

	if (mpi_state & HW_ATL_FW2X_CAP_ASYM_PAUSE)
		*fcmode |= AQ_NIC_FC_TX;

	return 0;
}

static int aq_fw2x_set_phyloopback(struct aq_hw_s *self, u32 mode, bool enable)
{
	u32 mpi_opts;

	switch (mode) {
	case AQ_HW_LOOPBACK_PHYINT_SYS:
		mpi_opts = aq_hw_read_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR);
		if (enable)
			mpi_opts |= HW_ATL_FW2X_CTRL_INT_LOOPBACK;
		else
			mpi_opts &= ~HW_ATL_FW2X_CTRL_INT_LOOPBACK;
		aq_hw_write_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR, mpi_opts);
		break;
	case AQ_HW_LOOPBACK_PHYEXT_SYS:
		mpi_opts = aq_hw_read_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR);
		if (enable)
			mpi_opts |= HW_ATL_FW2X_CTRL_EXT_LOOPBACK;
		else
			mpi_opts &= ~HW_ATL_FW2X_CTRL_EXT_LOOPBACK;
		aq_hw_write_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR, mpi_opts);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static u32 aq_fw2x_mbox_get(struct aq_hw_s *self)
{
	return aq_hw_read_reg(self, HW_ATL_FW2X_MPI_MBOX_ADDR);
}

static u32 aq_fw2x_rpc_get(struct aq_hw_s *self)
{
	return aq_hw_read_reg(self, HW_ATL_FW2X_MPI_RPC_ADDR);
}

static int aq_fw2x_settings_get(struct aq_hw_s *self, u32 *addr)
{
	int err = 0;
	u32 offset;

	offset = self->mbox_addr + offsetof(struct hw_atl_utils_mbox,
					    info.setting_address);

	err = hw_atl_utils_fw_downld_dwords(self, offset, addr, 1);

	return err;
}

static u32 aq_fw2x_state2_get(struct aq_hw_s *self)
{
	return aq_hw_read_reg(self, HW_ATL_FW2X_MPI_STATE2_ADDR);
}

static void aq_fw2x_set_downshift(struct aq_hw_s *self, bool enable)
{
	u32 mpi_opts;

	mpi_opts = aq_hw_read_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR);
	if (enable)
		mpi_opts |= HW_ATL_FW2X_CTRL_DOWNSHIFT;
	else
		mpi_opts &= ~HW_ATL_FW2X_CTRL_DOWNSHIFT;
	aq_hw_write_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR, mpi_opts);
}

static int aq_fw2x_set_media_detect(struct aq_hw_s *self, bool on)
{
	int err = 0;
	u32 enable;
	u32 offset;

	if (self->fw_ver_actual < HW_ATL_FW_VER_MEDIA_CONTROL)
		return -EOPNOTSUPP;

	offset = offsetof(struct hw_atl_utils_settings, media_detect);
	enable = on;

	err = hw_atl_write_fwsettings_dwords(self, offset, &enable, 1);

	if (err)
		return err;

	return 0;
}

static int aq_fw2x_run_tdr_diag(struct aq_hw_s *self)
{
	int err = 0;
	u32 caps_hi;
	u32 offset;
	u32 val;

	offset = self->mbox_addr + offsetof(struct hw_atl_utils_mbox,
					    info.caps_hi);

	/* Check if capability is available */
	err = hw_atl_utils_fw_downld_dwords(self, offset, &caps_hi, 1);
	if (err)
		return err;

	if (!(caps_hi & BIT(CAPS_HI_CABLE_DIAG)))
		return -EOPNOTSUPP;

	/* Trigger cable diag mode */
	val = aq_hw_read_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR) ^
			BIT(CTRL_CABLE_DIAG);
	aq_hw_write_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR, val);

	return err;
}

static int aq_fw2x_get_diag_data(struct aq_hw_s *self, struct aq_diag_s *diag)
{
	u32 caps_ex;
	int err = 0;
	u32 offset;
	u32 val;

	if (!diag)
		return -EINVAL;

	memset(diag, 0, sizeof(*diag));

	/* Check if FW reported back ready TDR diag */
	val = aq_hw_read_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR) &
		BIT(CTRL_CABLE_DIAG);
	if (val != (aq_fw2x_state2_get(self) & BIT(CTRL_CABLE_DIAG)))
		return -EBUSY;

	offset = self->mbox_addr + offsetof(struct hw_atl_utils_mbox,
					    info.caps_ex);

	/* Check if capability is available */
	err = hw_atl_utils_fw_downld_dwords(self, offset, &caps_ex, 1);
	if (err)
		return err;

	/** Reads SNR margins
	 */
	if (caps_ex & BIT(CAPS_EX_SNR_OPERATING_MARGIN)) {
		offset = self->mbox_addr + offsetof(struct hw_atl_utils_mbox,
						    info.snr_margin);

		err = hw_atl_utils_fw_downld_dwords(self, offset,
						    (u32 *)&diag->snr_margin,
						    sizeof(diag->snr_margin) /
						    sizeof(u32));
	}

	offset = self->mbox_addr + offsetof(struct hw_atl_utils_mbox,
					    info.cable_diag_data);

	/** Reads Cable diag data
	 */
	err = hw_atl_utils_fw_downld_dwords(self, offset,
					    (u32 *)&diag->cable_diag,
					    sizeof(diag->cable_diag) /
					    sizeof(u32));
	if (err)
		return err;

	(void)aq_fw2x_get_cable_len(self, &diag->cable_len);

	(void)aq_fw2x_get_phy_temp(self, &diag->phy_temp);

	diag->phy_temp = diag->phy_temp / 1000;

	return err;
}

int aq_fw2x_restart_aneg(struct aq_hw_s *self)
{
	u32 mpi_opts;

	mpi_opts = aq_hw_read_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR);
	mpi_opts |= HW_ATL_FW2X_CTRL_FORCE_RECONNECT;
	aq_hw_write_reg(self, HW_ATL_FW2X_MPI_CONTROL2_ADDR, mpi_opts);

	return 0;
}

const struct aq_fw_ops aq_fw_2x_ops = {
	.init               = aq_fw2x_init,
	.deinit             = aq_fw2x_deinit,
	.reset              = NULL,
	.renegotiate        = aq_fw2x_renegotiate,
	.get_mac_permanent  = aq_fw2x_get_mac_permanent,
	.set_link_speed     = aq_fw2x_set_link_speed,
	.set_state          = aq_fw2x_set_state,
	.update_link_status = aq_fw2x_update_link_status,
	.update_stats       = aq_fw2x_update_stats,
	.set_power          = aq_fw2x_set_power,
	.get_phy_temp       = aq_fw2x_get_phy_temp,
	.get_cable_len      = aq_fw2x_get_cable_len,
	.set_eee_rate       = aq_fw2x_set_eee_rate,
	.get_eee_rate       = aq_fw2x_get_eee_rate,
	.set_flow_control   = aq_fw2x_set_flow_control,
	.get_flow_control   = aq_fw2x_get_flow_control,
	.send_fw_request    = aq_fw2x_send_fw_request,
	.enable_ptp         = aq_fw3x_enable_ptp,
#ifdef TSN_SUPPORT
	.enable_tsn         = aq_fw3x_enable_tsn,
#endif
	.led_control        = aq_fw2x_led_control,
	.set_phyloopback    = aq_fw2x_set_phyloopback,
	.set_downshift      = aq_fw2x_set_downshift,
	.set_media_detect   = aq_fw2x_set_media_detect,
	.run_tdr_diag       = aq_fw2x_run_tdr_diag,
	.get_diag_data      = aq_fw2x_get_diag_data,
	.adjust_ptp			= aq_fw3x_adjust_ptp,
	.restart_aneg       = aq_fw2x_restart_aneg,
};