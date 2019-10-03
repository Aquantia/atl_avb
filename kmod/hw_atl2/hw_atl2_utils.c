/*
 * aQuantia Corporation Network Driver
 * Copyright (C) 2014-2019 aQuantia Corporation. All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

/* File hw_atl2_utils.c: Definition of common functions for Atlantic hardware
 * abstraction layer.
 */

#include "../aq_nic.h"
#include "../aq_hw_utils.h"
#include "hw_atl2_utils.h"
#include "hw_atl2_internal.h"
#include "hw_atl2_llh.h"
#include "hw_atl2_llh_internal.h"

#include <linux/random.h>

#define A2_FORCE_FLASHLESS       0

#define HW_ATL2_MIF_CMD          0x0200U
#define HW_ATL2_MIF_ADDR         0x0208U
#define HW_ATL2_MIF_VAL          0x020CU

#define HW_ATL2_FW_SM_RAM        0x2U
#define HW_ATL2_FW_VERSION       0x18
#define HW_ATL2_FW_VER 0x23000000U
#define NO_HW_ATL2_FW_VER 0x00000U


#define HW_ATL2_MPI_EFUSE_ADDR       0x364
#define HW_ATL2_MPI_MBOX_ADDR        0x360
#define HW_ATL2_MPI_RPC_ADDR         0x334

#define HW_ATL2_MPI_CONTROL_ADDR     0x368
#define HW_ATL2_MPI_CONTROL2_ADDR    0x36C
#define HW_ATL2_MPI_CONTROL3_ADDR    0x374

#define HW_ATL2_MPI_STATE_ADDR       0x370
#define HW_ATL2_MPI_STATE2_ADDR      0x374

#define HW_ATL2_EXT_CONTROL_ADDR	 0x378
#define HW_ATL2_EXT_STATE_ADDR		 0x37c

#define HW_ATL2_CAP_SLEEP_PROXY      BIT(CAPS_HI_SLEEP_PROXY)
#define HW_ATL2_CAP_WOL              BIT(CAPS_HI_WOL)

#define HW_ATL2_CAP_EEE_1G_MASK      BIT(CAPS_HI_1000BASET_FD_EEE)
#define HW_ATL2_CAP_EEE_2G5_MASK     BIT(CAPS_HI_2P5GBASET_FD_EEE)
#define HW_ATL2_CAP_EEE_5G_MASK      BIT(CAPS_HI_5GBASET_FD_EEE)
#define HW_ATL2_CAP_EEE_10G_MASK     BIT(CAPS_HI_10GBASET_FD_EEE)

#define HAL_ATLANTIC_WOL_FILTERS_COUNT   8
#define HAL_ATLANTIC_UTILS_MSG_WOL  0x0E

#define AQ_A2_BOOT_STARTED         BIT(0x18)
#define AQ_A2_CRASH_INIT           BIT(0x1B)
#define AQ_A2_BOOT_CODE_FAILED     BIT(0x1C)
#define AQ_A2_FW_INIT_FAILED       BIT(0x1D)
#define AQ_A2_FW_INIT_COMP_SUCCESS BIT(0x1F)
#define AQ_A2_FW_BOOT_FAILED_MASK (AQ_A2_CRASH_INIT | \
				   AQ_A2_BOOT_CODE_FAILED | \
				   AQ_A2_FW_INIT_FAILED)
#define AQ_A2_FW_BOOT_COMPLETE_MASK (AQ_A2_FW_BOOT_FAILED_MASK | \
				     AQ_A2_FW_INIT_COMP_SUCCESS)

#define AQ_A2_FW_BOOT_REQ_REBOOT        BIT(0x0)
#define AQ_A2_FW_BOOT_REQ_HOST_BOOT     BIT(0x8)
#define AQ_A2_FW_BOOT_REQ_MAC_FAST_BOOT BIT(0xA)
#define AQ_A2_FW_BOOT_REQ_PHY_FAST_BOOT BIT(0xB)

static int hw_atl2_utils_ver_match(u32 ver_expected, u32 ver_actual);


int hw_atl2_utils_initfw(struct aq_hw_s *self, const struct aq_fw_ops **fw_ops)
{
	int err = 0;
	
	err = hw_atl2_utils_soft_reset(self);
	if (err)
		return err;

	self->fw_ver_actual = hw_atl2_utils_get_fw_version(self);

	if (hw_atl2_utils_ver_match(NO_HW_ATL2_FW_VER,
				   self->fw_ver_actual) == 0) {
		*fw_ops = &aq_a2_fw_ops;
	} else {
		aq_pr_err("Bad FW version detected: %x, but continue\n",
			  self->fw_ver_actual);
		*fw_ops = &aq_a2_fw_ops;
	}
	aq_pr_trace("Detect ATL2FW %x\n", self->fw_ver_actual);
	self->aq_fw_ops = *fw_ops;
	err = self->aq_fw_ops->init(self);
	return err;
}

static bool hw_atl2_mcp_boot_complete(struct aq_hw_s *self)
{
	u32 rbl_status;

	rbl_status = hw_atl2_mif_mcp_boot_reg_get(self);
	if (rbl_status & AQ_A2_FW_BOOT_COMPLETE_MASK)
		return true;

	/* Host boot requested */
	if (hw_atl2_mif_host_req_int_get(self) & 0x1)
		return true;

	return false;
}

int hw_atl2_utils_soft_reset(struct aq_hw_s *self)
{
	bool rbl_complete = false;
	u32 rbl_status = 0;
	u32 rbl_request;
	int i, err = 0;

	if( hw_atl2_mif_mcp_boot_ver_get(self) ) {
		err = readx_poll_timeout_atomic(hw_atl2_mif_mcp_boot_reg_get,
					self,
					rbl_status,
					(rbl_status & AQ_A2_BOOT_STARTED),
					10, 500000);
		if (err)
			aq_pr_trace("Boot code probably hanged, reboot anyway");

		hw_atl2_mif_host_req_int_clr(self, 0x01);
		rbl_request = AQ_A2_FW_BOOT_REQ_REBOOT;
	#ifdef AQ_CFG_FAST_START
		rbl_request |= AQ_A2_FW_BOOT_REQ_MAC_FAST_BOOT;
	#endif
		hw_atl2_mif_mcp_boot_reg_set(self, rbl_request);

		/* Wait for RBL boot */
		err = readx_poll_timeout_atomic(hw_atl2_mif_mcp_boot_reg_get,
					self,
					rbl_status,
					(rbl_status & AQ_A2_BOOT_STARTED),
					10, 500000);
		if (err) {
			aq_pr_err("Boot code hanged");
			goto err_exit;
		}


		err = readx_poll_timeout_atomic(hw_atl2_mcp_boot_complete,
					self,
					rbl_complete,
					rbl_complete,
					10, 1000000);

		if (err) {
			aq_pr_err("FW Restart timed out");
			goto err_exit;
		}

		rbl_status = hw_atl2_mif_mcp_boot_reg_get(self);

		if (rbl_status & AQ_A2_FW_BOOT_FAILED_MASK) {
			err = -EIO;
			aq_pr_err("FW Restart failed");
			goto err_exit;
		}

		if (hw_atl2_mif_host_req_int_get(self) & 0x1) {
			err = -EIO;
			aq_pr_err("No FW detected. Dynamic FW load not implemented");
			goto err_exit;
		}

		if (self->aq_fw_ops) {
			err = self->aq_fw_ops->init(self);
			if (err) {
				aq_pr_err("FW Init failed");
				goto err_exit;
			}
		}
		AQ_HW_SLEEP(1000);
	} else {
		aq_pr_trace("Non-RBL new reset sequence!\n");
		hw_atl2_com_ful_reset_tgl(self);
		AQ_HW_SLEEP(50);
		for(i = 1000; ((hw_atl2_reg_glb_mif_id_get(self) & 0xf0f) != 0x103u) && i; i--)
			udelay(100);

		if( !i ) {
			err = -ETIME;
			return err;
		}
	}
err_exit:
	return err;
}

static int hw_atl2_utils_ver_match(u32 ver_expected, u32 ver_actual)
{
	int err = 0;
	const u32 dw_major_mask = 0xff000000U;
	const u32 dw_minor_mask = 0x00ffffffU;

	err = (dw_major_mask & (ver_expected ^ ver_actual)) ? -EOPNOTSUPP : 0;
	if (err < 0)
		goto err_exit;
	err = ((dw_minor_mask & ver_expected) > (dw_minor_mask & ver_actual)) ?
		-EOPNOTSUPP : 0;
err_exit:
	return err;
}

static const u32 hw_atl2_utils_hw_mac_regs[] = {
	0x00005580U, 0x00005590U, 0x000055B0U, 0x000055B4U,
	0x000055C0U, 0x00005B00U, 0x00005B04U, 0x00005B08U,
	0x00005B0CU, 0x00005B10U, 0x00005B14U, 0x00005B18U,
	0x00005B1CU, 0x00005B20U, 0x00005B24U, 0x00005B28U,
	0x00005B2CU, 0x00005B30U, 0x00005B34U, 0x00005B38U,
	0x00005B3CU, 0x00005B40U, 0x00005B44U, 0x00005B48U,
	0x00005B4CU, 0x00005B50U, 0x00005B54U, 0x00005B58U,
	0x00005B5CU, 0x00005B60U, 0x00005B64U, 0x00005B68U,
	0x00005B6CU, 0x00005B70U, 0x00005B74U, 0x00005B78U,
	0x00005B7CU, 0x00007C00U, 0x00007C04U, 0x00007C08U,
	0x00007C0CU, 0x00007C10U, 0x00007C14U, 0x00007C18U,
	0x00007C1CU, 0x00007C20U, 0x00007C40U, 0x00007C44U,
	0x00007C48U, 0x00007C4CU, 0x00007C50U, 0x00007C54U,
	0x00007C58U, 0x00007C5CU, 0x00007C60U, 0x00007C80U,
	0x00007C84U, 0x00007C88U, 0x00007C8CU, 0x00007C90U,
	0x00007C94U, 0x00007C98U, 0x00007C9CU, 0x00007CA0U,
	0x00007CC0U, 0x00007CC4U, 0x00007CC8U, 0x00007CCCU,
	0x00007CD0U, 0x00007CD4U, 0x00007CD8U, 0x00007CDCU,
	0x00007CE0U, 0x00000300U, 0x00000304U, 0x00000308U,
	0x0000030cU, 0x00000310U, 0x00000314U, 0x00000318U,
	0x0000031cU, 0x00000360U, 0x00000364U, 0x00000368U,
	0x0000036cU, 0x00000370U, 0x00000374U, 0x00006900U,
};

int hw_atl2_utils_hw_get_regs(struct aq_hw_s *self,
			     const struct aq_hw_caps_s *aq_hw_caps,
			     u32 *regs_buff)
{
	unsigned int i = 0U;

	for (i = 0; i < aq_hw_caps->mac_regs_count; i++)
		regs_buff[i] = aq_hw_read_reg(self,
					      hw_atl2_utils_hw_mac_regs[i]);
	return 0;
}

u32 hw_atl2_utils_get_fw_version(struct aq_hw_s *self)
{
	return aq_hw_read_reg(self, 0x13008);
}
//EOF
