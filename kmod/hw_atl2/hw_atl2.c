/*
 * aQuantia Corporation Network Driver
 * Copyright (C) 2014-2019 aQuantia Corporation. All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

/* File hw_atl2.c: Definition of Atlantic hardware specific functions. */

#include "../aq_hw.h"
#include "../aq_hw_utils.h"
#include "../aq_ring.h"
#include "../aq_nic.h"
#include "../aq_filters.h"
#include "../aq_ptp.h"
#include "../aq_trace.h"
#include "hw_atl2.h"
#include "hw_atl2_utils.h"
#include "hw_atl2_llh.h"
#include "hw_atl2_internal.h"
#include "hw_atl2_llh_internal.h"

static int hw_atl2_rx_tc_mode_get(struct aq_hw_s *self, u32 *tc_mode);
static int hw_atl2_act_rslvr_table_set(struct aq_hw_s *self, u8 location,
				      u32 tag, u32 mask, u32 action);
static void hw_atl2_enable_ptp(struct aq_hw_s *self, unsigned int param, int enable);

#define DEFAULT_BOARD_BASIC_CAPABILITIES \
	.is_64_dma = true,		  \
	.op64bit = true,          \
	.msix_irqs = 4U,		  \
	.irq_mask = ~0U,		  \
	.vecs = HW_ATL2_RSS_MAX,	  \
	.tcs = HW_ATL2_TC_MAX,	  \
	.rxd_alignment = 1U,		  \
	.rxd_size = HW_ATL2_RXD_SIZE,   \
	.rxds_max = HW_ATL2_MAX_RXD,    \
	.rxds_min = HW_ATL2_MIN_RXD,    \
	.txd_alignment = 1U,		  \
	.txd_size = HW_ATL2_TXD_SIZE,   \
	.txds_max = HW_ATL2_MAX_TXD,    \
	.txds_min = HW_ATL2_MIN_TXD,    \
	.txhwb_alignment = 4096U,	  \
	.tx_rings = HW_ATL2_TX_RINGS,   \
	.rx_rings = HW_ATL2_RX_RINGS,   \
	.hw_features = NETIF_F_HW_CSUM |  \
			NETIF_F_RXCSUM |  \
			NETIF_F_RXHASH |  \
			NETIF_F_SG |      \
			NETIF_F_TSO |     \
			NETIF_F_LRO |     \
			NETIF_F_NTUPLE |  \
			NETIF_F_HW_VLAN_CTAG_FILTER | \
			NETIF_F_HW_VLAN_CTAG_RX |     \
			NETIF_F_HW_VLAN_CTAG_TX |     \
			NETIF_F_GSO_UDP_L4      |     \
			NETIF_F_GSO_PARTIAL,          \
	.hw_priv_flags = IFF_UNICAST_FLT, \
	.flow_control = true,		  \
	.mtu = HW_ATL2_MTU_JUMBO,	  \
	.mac_regs_count = 88,		  \
	.hw_alive_check_addr = 0x1CU

const struct aq_hw_caps_s hw_atl2_caps_aqc113 = {
	DEFAULT_BOARD_BASIC_CAPABILITIES,
	.media_type = AQ_HW_MEDIA_TYPE_TP,
	.link_speed_msk = AQ_NIC_RATE_10G|
			  AQ_NIC_RATE_5G|
			  AQ_NIC_RATE_2GS|
			  AQ_NIC_RATE_1G|
			  AQ_NIC_RATE_100M,
};

static u32 clk_select = -1;

static int hw_atl2_hw_reset(struct aq_hw_s *self)
{
	int err = 0;
	err = hw_atl2_utils_soft_reset(self);
	if (err)
		return err;

	if( clk_select != -1 ) {
		hw_atl2_enable_ptp(self, clk_select,
			aq_utils_obj_test(&self->flags, AQ_HW_PTP_AVAILABLE) ? 1 : 0);
	}
	self->aq_fw_ops->set_state(self, MPI_RESET);

	err = aq_hw_err_from_flags(self);

	return err;
}

static int hw_atl2_set_fc(struct aq_hw_s *self, u32 fc, u32 tc)
{
	hw_atl2_rpb_rx_xoff_en_per_tc_set(self, !!(fc & AQ_NIC_FC_RX), tc);
	return 0;
}

static int hw_atl2_hw_qos_set(struct aq_hw_s *self)
{
	u32 q;
	u32 tc = 0U, tc_cnt = 0U;
	u32 buff_size = 0U;
	unsigned int i_priority = 0U;

	/* TPS Descriptor rate init */
	hw_atl2_tps_tx_pkt_shed_desc_rate_curr_time_res_set(self, 0x0U);
	hw_atl2_tps_tx_pkt_shed_desc_rate_lim_set(self, 0xA);

	/* TPS VM init */
	hw_atl2_tps_tx_pkt_shed_desc_vm_arb_mode_set(self, 0U);

	/* TPS TC credits init */
	hw_atl2_tps_tx_pkt_shed_desc_tc_arb_mode_set(self, 0U);
	hw_atl2_tps_tx_pkt_shed_data_arb_mode_set(self, 0U);

	tc = 0;
	
	/* TX Packet Scheduler Data TC0 */
	hw_atl2_tps_tx_pkt_shed_tc_data_max_credit_set(self, 0xFFF, tc);
	hw_atl2_tps_tx_pkt_shed_tc_data_weight_set(self, 0x64, tc);
	hw_atl2_tps_tx_pkt_shed_desc_tc_max_credit_set(self, 0x50, tc);
	hw_atl2_tps_tx_pkt_shed_desc_tc_weight_set(self, 0x1E, tc);

	/* Tx buf size TC0 */
	buff_size = HW_ATL2_TXBUF_MAX - 
#ifdef TSN_SUPPORT
				HW_ATL2_AVB1_TXBUF_SIZE - 
				HW_ATL2_AVB2_TXBUF_SIZE - 
#endif
				HW_ATL2_PTP_TXBUF_SIZE;

	hw_atl2_tpb_tx_pkt_buff_size_per_tc_set(self, buff_size, tc);
	hw_atl2_tpb_tx_buff_hi_threshold_per_tc_set(self,
						(buff_size * (1024 / 32U) * 66U) / 100U, tc);
	hw_atl2_tpb_tx_buff_lo_threshold_per_tc_set(self,
						(buff_size * (1024 / 32U) * 50U) / 100U, tc);
	
	/* Init TC2 for PTP_TX */
	tc = 2;
	hw_atl2_tpb_tx_packet_scheduler_highest_priority_tc_set(self, tc);
	hw_atl2_tpb_tx_pkt_buff_size_per_tc_set(self, HW_ATL2_PTP_TXBUF_SIZE, tc);
	hw_atl2_tpb_tx_flex_map_en_set(self, aq_new_filters_enabled);
#ifdef TSN_SUPPORT
	/* Init TC1 for AVB1_TX */
	tc = 1;
	hw_atl2_tpb_tx_packet_scheduler_avb_high_priority0_tc_set(self, tc);
	hw_atl2_tpb_tx_pkt_buff_size_per_tc_set(self, HW_ATL2_AVB1_TXBUF_SIZE, tc);
	hw_atl2_tpb_set_high_priority(self, tc);

	/* Init TC3 for AVB2_TX */
	tc = 3;
	hw_atl2_tpb_tx_packet_scheduler_avb_high_priority1_tc_set(self, tc);
	hw_atl2_tpb_tx_pkt_buff_size_per_tc_set(self, HW_ATL2_AVB2_TXBUF_SIZE, tc);
	hw_atl2_tpb_set_high_priority(self, tc);

	hw_atl2_tpb_tx_enable_avb_tcs_set(self, 0);
#endif

	/* QoS Rx buf size per TC */
	tc = 0;
	buff_size = HW_ATL2_RXBUF_MAX - 
#ifdef TSN_SUPPORT
				HW_ATL2_AVB1_RXBUF_SIZE - 
				HW_ATL2_AVB2_RXBUF_SIZE - 
#endif
				HW_ATL2_PTP_RXBUF_SIZE;

	hw_atl2_rpb_rx_pkt_buff_size_per_tc_set(self, buff_size, tc);
	hw_atl2_rpb_rx_buff_hi_threshold_per_tc_set(self,
						(buff_size * (1024U / 32U) * 66U) / 100U, tc);
	hw_atl2_rpb_rx_buff_lo_threshold_per_tc_set(self,
						(buff_size * (1024U / 32U) * 50U) / 100U, tc);

	hw_atl2_set_fc(self, self->aq_nic_cfg->fc.req, tc);

	hw_atl2_rx_tc_mode_get(self, &tc_cnt);

	//ATL2 Apply ring to TC mapping
	for(q = 0; q <= (!aq_new_filters_enabled ? hw_atl2_caps_aqc113.rx_rings : 32); q++) {
		if( !aq_new_filters_enabled && 
			(q >> (tc_cnt ? 2 : 3)) != tc ) {
				//hw_atl2_caps_aqc113.rx_rings = q;
				break; //Cannot use legacy mapping in such way
			}
		hw_atl2_rx_q_map_to_tc(self, q, tc);
		hw_atl2_tpb_tx_flex_map_set(self, q, tc);
	}
	
	/* Init TC2 for PTP_RX */
	tc = 2;
	hw_atl2_rpb_rx_pkt_buff_size_per_tc_set(self, HW_ATL2_PTP_RXBUF_SIZE, tc);

	/* Init Rx Q to TC mapping */
	hw_atl2_rx_q_map_to_tc(self, tc_cnt ? AQ_CFG_PTP_4TC_RING_IDX : AQ_CFG_PTP_8TC_RING_IDX, tc);
	hw_atl2_tpb_tx_flex_map_set(self, tc_cnt ? AQ_CFG_PTP_4TC_RING_IDX : AQ_CFG_PTP_8TC_RING_IDX, tc);

#ifdef TSN_SUPPORT
	/* Init TC1 for AVB1_RX */
	tc = 1;
	hw_atl2_rpb_rx_pkt_buff_size_per_tc_set(self, HW_ATL2_AVB1_RXBUF_SIZE, tc);

	/* Init Rx Q to TC mapping */
	hw_atl2_rx_q_map_to_tc(self, aq_new_filters_enabled ? AQ_A2_CFG_AVB1_FTC_RING_IDX :
			tc_cnt ? AQ_A2_CFG_AVB1_4TC_RING_IDX : AQ_A2_CFG_AVB1_8TC_RING_IDX, tc);
	hw_atl2_tpb_tx_flex_map_set(self, aq_new_filters_enabled ? AQ_A2_CFG_AVB1_FTC_RING_IDX :
			tc_cnt ? AQ_A2_CFG_AVB1_4TC_RING_IDX : AQ_A2_CFG_AVB1_8TC_RING_IDX, tc);
	/* Init TC3 for AVB2_RX */
	tc = 3;
	hw_atl2_rpb_rx_pkt_buff_size_per_tc_set(self, HW_ATL2_AVB2_RXBUF_SIZE, tc);

	/* Init Rx Q to TC mapping */
	hw_atl2_rx_q_map_to_tc(self, aq_new_filters_enabled ? AQ_A2_CFG_AVB2_FTC_RING_IDX :
			tc_cnt ? AQ_A2_CFG_AVB2_4TC_RING_IDX : AQ_A2_CFG_AVB2_8TC_RING_IDX, tc);
	hw_atl2_tpb_tx_flex_map_set(self, aq_new_filters_enabled ? AQ_A2_CFG_AVB2_FTC_RING_IDX :
			tc_cnt ? AQ_A2_CFG_AVB2_4TC_RING_IDX : AQ_A2_CFG_AVB2_8TC_RING_IDX, tc);
#endif

	/* QoS 802.1p priority -> TC mapping */
	for (i_priority = 8U; i_priority--;)
		hw_atl2_rpf_rpb_user_priority_tc_map_set(self, i_priority, 0U);

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_rss_hash_set(struct aq_hw_s *self,
				     struct aq_rss_parameters *rss_params)
{
	struct aq_nic_cfg_s *cfg = self->aq_nic_cfg;
	int err = 0;
	unsigned int i = 0U;
	unsigned int addr = 0U;

	for (i = 10, addr = 0U; i--; ++addr) {
		u32 val, key_data = cfg->is_rss ?
			__swab32(rss_params->hash_secret_key[i]) : 0U;
		hw_atl2_rpf_rss_key_wr_data_set(self, key_data);
		hw_atl2_rpf_rss_key_addr_set(self, addr);
		hw_atl2_rpf_rss_key_wr_en_set(self, 1U);
		
		readx_poll_timeout_atomic(hw_atl2_rpf_rss_key_wr_en_get, self,
					val, val == 0U, 1000U, 10U);
		if (err < 0)
			goto err_exit;
	}

	err = aq_hw_err_from_flags(self);

err_exit:
	return err;
}

static int hw_atl2_hw_rss_set(struct aq_hw_s *self,
				struct aq_rss_parameters *rss_params)
{
	u8 *indirection_table =	rss_params->indirection_table;
	u32 val, i = 0U;
	u32 num_rss_queues = max(1U, self->aq_nic_cfg->num_rss_queues);
	int err = 0;
	u16 bitary[(HW_ATL2_RSS_REDIRECTION_MAX *
					HW_ATL2_RSS_REDIRECTION_BITS / 16U)];

	memset(bitary, 0, sizeof(bitary));

	for (i = HW_ATL2_RSS_REDIRECTION_MAX; i--;) {
		(*(u32 *)(bitary + ((i * 3U) / 16U))) |=
			((indirection_table[i] % num_rss_queues) <<
			((i * 3U) & 0xFU));
		hw_atl2_rpf_rss_table1_queue_tc0_set(self, (indirection_table[i] % num_rss_queues), i);
	}

	for (i = ARRAY_SIZE(bitary); i--;) {
		hw_atl2_rpf_rss_redir_tbl_wr_data_set(self, bitary[i]);
		hw_atl2_rpf_rss_redir_tbl_addr_set(self, i);
		hw_atl2_rpf_rss_redir_wr_en_set(self, 1U);
		readx_poll_timeout_atomic(hw_atl2_rpf_rss_redir_wr_en_get, self,
					val, val == 0U, 1000U, 10U);
		if (err < 0)
			goto err_exit;
	}

	err = aq_hw_err_from_flags(self);

err_exit:
	return err;
}

static int hw_atl2_hw_offload_set(struct aq_hw_s *self,
				    struct aq_nic_cfg_s *aq_nic_cfg)
{
	unsigned int i;

	/* TX checksums offloads*/
	hw_atl2_tpo_ipv4header_crc_offload_en_set(self, 1);
	hw_atl2_tpo_tcp_udp_crc_offload_en_set(self, 1);

	/* RX checksums offloads*/
	hw_atl2_rpo_ipv4header_crc_offload_en_set(self,
				 !!(aq_nic_cfg->features & NETIF_F_RXCSUM));
	hw_atl2_rpo_tcp_udp_crc_offload_en_set(self,
				 !!(aq_nic_cfg->features & NETIF_F_RXCSUM));

	/* LSO offloads*/
	hw_atl2_tdm_large_send_offload_en_set(self, 0xFFFFFFFFU);

	/* Outer VLAN tag offload */
	hw_atl2_rpo_outer_vlan_tag_mode_set(self, 1U);

/* LRO offloads */
	{
		unsigned int val = (8U < HW_ATL2_LRO_RXD_MAX) ? 0x3U :
			((4U < HW_ATL2_LRO_RXD_MAX) ? 0x2U :
			((2U < HW_ATL2_LRO_RXD_MAX) ? 0x1U : 0x0));

		for (i = 0; i < HW_ATL2_RINGS_MAX; i++)
			hw_atl2_rpo_lro_max_num_of_descriptors_set(self, val, i);

		hw_atl2_rpo_lro_time_base_divider_set(self, 0x61AU);
		hw_atl2_rpo_lro_inactive_interval_set(self, 0);
		/* the LRO timebase divider is 5 uS (0x61a),
		 * which is multiplied by 50(0x32)
		 * to get a maximum coalescing interval of 250 uS,
		 * which is the default value
		 */
		hw_atl2_rpo_lro_max_coalescing_interval_set(self, 50);

		hw_atl2_rpo_lro_qsessions_lim_set(self, 1U);

		hw_atl2_rpo_lro_total_desc_lim_set(self, 2U);

		hw_atl2_rpo_lro_patch_optimization_en_set(self, 0U);

		hw_atl2_rpo_lro_min_pay_of_first_pkt_set(self, 10U);

		hw_atl2_rpo_lro_pkt_lim_set(self, 1U);

		hw_atl2_rpo_lro_en_set(self,
				      aq_nic_cfg->is_lro ? 0xFFFFFFFFU : 0U);
	}
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_init_tx_path(struct aq_hw_s *self)
{
	/* Tx TC/RSS number config */
	hw_atl2_rpb_tps_tx_tc_mode_set(self, 1U);

	hw_atl2_thm_lso_tcp_flag_of_first_pkt_set(self, 0x0FF6U);
	hw_atl2_thm_lso_tcp_flag_of_middle_pkt_set(self, 0x0FF6U);
	hw_atl2_thm_lso_tcp_flag_of_last_pkt_set(self, 0x0F7FU);

	/* Tx interrupts */
	hw_atl2_tdm_tx_desc_wr_wb_irq_en_set(self, 1U);

	/* misc */
	hw_atl2_tdm_tx_dca_en_set(self, 0U);
	hw_atl2_tdm_tx_dca_mode_set(self, 0U);

	hw_atl2_tpb_tx_path_scp_ins_en_set(self, 1U);
	hw_atl2_tpb_tx_buf_clk_gate_en_set(self, 0U);

	return aq_hw_err_from_flags(self);
}

/** Initialise new rx filters
 * L2 promisc OFF
 * VLAN promisc OFF
 *
 * VLAN
 * MAC
 * ALLMULTI
 * UT
 * VLAN promisc ON
 * L2 promisc ON
 */
static void hw_atl2_hw_init_new_rx_filters(struct aq_hw_s *self)
{
	hw_atl2_rpf_act_rslvr_section_en_set(self, 0xFFFF);
	hw_atl2_rpfl2_uc_flr_tag_set(self,
				     HW_ATL2_RPF_TAG_BASE_UC,
				     HW_ATL2_MAC_UC);
	hw_atl2_rpfl2_bc_flr_tag_set(self, HW_ATL2_RPF_TAG_BASE_UC);

	hw_atl2_act_rslvr_table_set(self,
		HW_ATL2_RPF_L2_PROMISC_OFF_INDEX,
		0,
		HW_ATL2_RPF_TAG_UC_MASK | HW_ATL2_RPF_TAG_ALLMC_MASK,
		HW_ATL2_ACTION_DROP);

	hw_atl2_act_rslvr_table_set(self,
		HW_ATL2_RPF_VLAN_PROMISC_OFF_INDEX,
		0,
		HW_ATL2_RPF_TAG_VLAN_MASK | HW_ATL2_RPF_TAG_UNTAG_MASK,
		HW_ATL2_ACTION_DROP);


	hw_atl2_act_rslvr_table_set(self,
		HW_ATL2_RPF_VLAN_INDEX,
		HW_ATL2_RPF_TAG_BASE_VLAN,
		HW_ATL2_RPF_TAG_VLAN_MASK,
		HW_ATL2_ACTION_ASSIGN_TC(0));

	hw_atl2_act_rslvr_table_set(self,
		HW_ATL2_RPF_MAC_INDEX,
		HW_ATL2_RPF_TAG_BASE_UC,
		HW_ATL2_RPF_TAG_UC_MASK,
		HW_ATL2_ACTION_ASSIGN_TC(0));

	hw_atl2_act_rslvr_table_set(self,
		HW_ATL2_RPF_ALLMC_INDEX,
		HW_ATL2_RPF_TAG_BASE_ALLMC,
		HW_ATL2_RPF_TAG_ALLMC_MASK,
		HW_ATL2_ACTION_ASSIGN_TC(0));

	hw_atl2_act_rslvr_table_set(self,
		HW_ATL2_RPF_UNTAG_INDEX,
		HW_ATL2_RPF_TAG_UNTAG_MASK,
		HW_ATL2_RPF_TAG_UNTAG_MASK,
		HW_ATL2_ACTION_ASSIGN_TC(0));

	hw_atl2_act_rslvr_table_set(self,
		HW_ATL2_RPF_VLAN_PROMISC_ON_INDEX,
		0,
		HW_ATL2_RPF_TAG_VLAN_MASK,
		HW_ATL2_ACTION_DISABLE);

	hw_atl2_act_rslvr_table_set(self,
		HW_ATL2_RPF_L2_PROMISC_ON_INDEX,
		0,
		HW_ATL2_RPF_TAG_UC_MASK,
		HW_ATL2_ACTION_DISABLE);
}

static void hw_atl2_hw_new_rx_filter_vlan_promisc(struct aq_hw_s *self,
						  bool promisc)
{
	u16 on_action = promisc ?
		     HW_ATL2_ACTION_ASSIGN_TC(0) :
		     HW_ATL2_ACTION_DISABLE;
	u16 off_action = (!promisc &&
			  !hw_atl2_rpfl2promiscuous_mode_en_get(self)) ?
				HW_ATL2_ACTION_DROP:
				HW_ATL2_ACTION_DISABLE;

	hw_atl2_act_rslvr_table_set(self,
		HW_ATL2_RPF_VLAN_PROMISC_ON_INDEX,
		0,
		HW_ATL2_RPF_TAG_VLAN_MASK,
		on_action);

	hw_atl2_act_rslvr_table_set(self,
		HW_ATL2_RPF_VLAN_PROMISC_OFF_INDEX,
		0,
		HW_ATL2_RPF_TAG_VLAN_MASK | HW_ATL2_RPF_TAG_UNTAG_MASK,
		off_action);
}

static void hw_atl2_hw_new_rx_filter_promisc(struct aq_hw_s *self, bool promisc)
{
	u16 on_action = promisc ?
			HW_ATL2_ACTION_ASSIGN_TC(0) :
			HW_ATL2_ACTION_DISABLE;
	u16 off_action = promisc ?
			 HW_ATL2_ACTION_DISABLE :
			 HW_ATL2_ACTION_DROP;

	hw_atl2_act_rslvr_table_set(self,
		HW_ATL2_RPF_L2_PROMISC_OFF_INDEX,
		0,
		HW_ATL2_RPF_TAG_UC_MASK | HW_ATL2_RPF_TAG_ALLMC_MASK,
		off_action);

	hw_atl2_act_rslvr_table_set(self,
		HW_ATL2_RPF_L2_PROMISC_ON_INDEX,
		0,
		HW_ATL2_RPF_TAG_UC_MASK,
		on_action);

	/* turn VLAN promisc mode too */
	hw_atl2_hw_new_rx_filter_vlan_promisc(self,
			promisc | hw_atl2_rpf_vlan_prom_mode_en_get(self));
}

static int hw_atl2_hw_init_rx_path(struct aq_hw_s *self)
{
	struct aq_nic_cfg_s *cfg = self->aq_nic_cfg;
	unsigned int control_reg_val = 0U;
	int i;

	/* Rx TC/RSS number config */
	hw_atl2_rpb_rpf_rx_traf_class_mode_set(self, 1U);

	/* Rx flow control */
	hw_atl2_rpb_rx_flow_ctl_mode_set(self, 1U);

	hw_atl2_reg_rx_flr_rss_hash_type_set(self, 0x1FFU);
	/* RSS Ring selection */
	hw_atl2_reg_rx_flr_rss_control1set(self, cfg->is_rss ?
					0xB3333333U : 0x00000000U);

	/* Multicast filters */
	for (i = HW_ATL2_MAC_MAX; i--;) {
		hw_atl2_rpfl2_uc_flr_en_set(self, (i == 0U) ? 1U : 0U, i);
		hw_atl2_rpfl2unicast_flr_act_set(self, 1U, i);
	}

	hw_atl2_reg_rx_flr_mcst_flr_msk_set(self, 0x00000000U);
	hw_atl2_reg_rx_flr_mcst_flr_set(self, 0x00010FFFU, 0U);

	/* Vlan filters */
	hw_atl2_rpf_vlan_outer_etht_set(self, 0x88A8U);
	hw_atl2_rpf_vlan_inner_etht_set(self, 0x8100U);

	hw_atl2_rpf_vlan_prom_mode_en_set(self, 1);

	/* Always accept untagged packets */
	hw_atl2_rpf_vlan_accept_untagged_packets_set(self, 1U);
	hw_atl2_rpf_vlan_untagged_act_set(self, 1U);

	if (aq_new_filters_enabled)
		hw_atl2_hw_init_new_rx_filters(self);

	/* Rx Interrupts */
	hw_atl2_rdm_rx_desc_wr_wb_irq_en_set(self, 1U);

	control_reg_val =
			   (3 << 17) | /* Filter logic version 3 */
			   BIT(19); /* RPF lpbk and mac counters by EOP */

	aq_hw_write_reg(self, 0x00005040U, control_reg_val);	
	hw_atl2_rpfl2broadcast_flr_act_set(self, 1U);
	hw_atl2_rpfl2broadcast_count_threshold_set(self, 0xFFFFU & (~0U / 256U));

	hw_atl2_rdm_rx_dca_en_set(self, 0U);
	hw_atl2_rdm_rx_dca_mode_set(self, 0U);

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_act_rslvr_table_set(struct aq_hw_s *self, u8 location,
				      u32 tag, u32 mask, u32 action)
{
	int err = 0;
	u32 val;
	//printk("hw_atl2_act_rslvr_table_set: location %d, tag %x, mask %x, action %x\n",
	//		location, tag, mask, action);
	readx_poll_timeout_atomic(hw_atl2_sem_act_rslvr_tbl_get, self,
					val, val == 1U,  1U, 10000U);
	if (!err)
		hw_atl2_rpf_act_rslvr_record_set(self, location, tag, mask,
						 action);
	hw_atl2_reg_glb_cpu_sem_set(self, 1, HW_ATL2_FW_SM_ACT_RSLVR);

	return err;
}

static int hw_atl2_hw_mac_addr_set(struct aq_hw_s *self, u8 *mac_addr)
{
	int err = 0;
	unsigned int h = 0U;
	unsigned int l = 0U;

	if (!mac_addr) {
		err = -EINVAL;
		goto err_exit;
	}
	h = (mac_addr[0] << 8) | (mac_addr[1]);
	l = (mac_addr[2] << 24) | (mac_addr[3] << 16) |
		(mac_addr[4] << 8) | mac_addr[5];

	hw_atl2_rpfl2_uc_flr_en_set(self, 0U, HW_ATL2_MAC_UC);
	hw_atl2_rpfl2unicast_dest_addresslsw_set(self, l, HW_ATL2_MAC_UC);
	hw_atl2_rpfl2unicast_dest_addressmsw_set(self, h, HW_ATL2_MAC_UC);
	hw_atl2_rpfl2_uc_flr_en_set(self, 1U, HW_ATL2_MAC_UC);

	err = aq_hw_err_from_flags(self);

err_exit:
	return err;
}

static int hw_atl2_hw_init(struct aq_hw_s *self, u8 *mac_addr)
{
	static u32 aq_hw_atl2_igcr_table_[4][2] = {
		{ 0x20000000U, 0x20000000U }, /* AQ_IRQ_INVALID */
		{ 0x20000080U, 0x20000080U }, /* AQ_IRQ_LEGACY */
		{ 0x20000021U, 0x20000025U }, /* AQ_IRQ_MSI */
		{ 0x20000022U, 0x20000026U }  /* AQ_IRQ_MSIX */
	};

	int err = 0;
	u32 val;

	struct aq_nic_cfg_s *aq_nic_cfg = self->aq_nic_cfg;

	hw_atl2_init_launchtime(self);

	hw_atl2_hw_init_tx_path(self);
	hw_atl2_hw_init_rx_path(self);

	hw_atl2_hw_mac_addr_set(self, mac_addr);

	self->aq_fw_ops->set_link_speed(self, aq_nic_cfg->link_speed_msk);
	self->aq_fw_ops->set_state(self, MPI_INIT);

	hw_atl2_hw_qos_set(self);
	hw_atl2_hw_rss_set(self, &aq_nic_cfg->aq_rss);
	hw_atl2_hw_rss_hash_set(self, &aq_nic_cfg->aq_rss);

	/* Force limit MRRS on RDM/TDM to 2K */
	val = aq_hw_read_reg(self, HW_ATL2_PCI_REG_CONTROL6_ADR);
	aq_hw_write_reg(self, HW_ATL2_PCI_REG_CONTROL6_ADR,
			(val & ~0x707) | 0x404);

	/* TX DMA total request limit. A2 hardware is not capable to
	 * handle more than (8K-MRRS) incoming DMA data.
	 * Value 24 in 256byte units
	 */
	aq_hw_write_reg(self, HW_ATL2_TX_DMA_TOTAL_REQ_LIMIT_ADR, 24);

	if (aq_new_filters_enabled)
		hw_atl2_rpf_new_enable_set(self, 1);

	/* Reset link status and read out initial hardware counters */
	self->aq_link_status.mbps = 0;
	self->aq_fw_ops->update_stats(self);

	err = aq_hw_err_from_flags(self);
	if (err < 0)
		goto err_exit;

	/* Interrupts */
	hw_atl2_reg_irq_glb_ctl_set(self,
				   aq_hw_atl2_igcr_table_[aq_nic_cfg->irq_type]
						 [(aq_nic_cfg->vecs > 1U) ?
						 1 : 0]);

	hw_atl2_itr_irq_auto_masklsw_set(self, aq_nic_cfg->aq_hw_caps->irq_mask);

	/* Enable link interrupt */
	if (aq_nic_cfg->link_irq_vec)
		hw_atl2_reg_gen_irq_map_set(self, BIT(7) | aq_nic_cfg->link_irq_vec, 3U);

	/* Interrupts */
	hw_atl2_reg_gen_irq_map_set(self,
				    ((HW_ATL2_ERR_INT << 0x18) | (1U << 0x1F)) |
				    ((HW_ATL2_ERR_INT << 0x10) | (1U << 0x17)), 0U);

	hw_atl2_hw_offload_set(self, aq_nic_cfg);

err_exit:
	return err;
}

static int hw_atl2_hw_ring_tx_start(struct aq_hw_s *self,
				      struct aq_ring_s *ring)
{
	hw_atl2_tdm_tx_desc_en_set(self, 1, ring->idx);
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_ring_rx_start(struct aq_hw_s *self,
				      struct aq_ring_s *ring)
{
	hw_atl2_rdm_rx_desc_en_set(self, 1, ring->idx);
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_start(struct aq_hw_s *self)
{
	hw_atl2_tpb_tx_buff_en_set(self, 1);
	hw_atl2_rpb_rx_buff_en_set(self, 1);
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_tx_ring_tail_update(struct aq_hw_s *self,
					    struct aq_ring_s *ring)
{
	hw_atl2_reg_tx_dma_desc_tail_ptr_set(self, ring->sw_tail, ring->idx);
	return 0;
}

static int hw_atl2_hw_ring_tx_xmit(struct aq_hw_s *self,
				     struct aq_ring_s *ring,
				     unsigned int frags)
{
	struct aq_ring_buff_s *buff = NULL;
	struct hw_atl2_txd_s *txd = NULL;

	unsigned int buff_pa_len = 0U;
	unsigned int pkt_len = 0U;
	unsigned int frag_count = 0U;
	bool is_vlan = false;
	bool is_gso = false;

	buff = &ring->buff_ring[ring->sw_tail];
	pkt_len = (buff->is_eop && buff->is_sop) ? buff->len : buff->len_pkt;

	for (frag_count = 0; frag_count < frags; frag_count++) {
		txd = (struct hw_atl2_txd_s *)&ring->dx_ring[ring->sw_tail *
						HW_ATL2_TXD_SIZE];
		txd->ctl = 0;
		txd->ctl2 = 0;
		txd->buf_addr = 0;

		buff = &ring->buff_ring[ring->sw_tail];

		if (buff->is_gso_tcp || buff->is_gso_udp) {
			if (buff->is_gso_tcp)
				txd->ctl |= HW_ATL2_TXD_CTL_CMD_TCP;
			txd->ctl |= HW_ATL2_TXD_CTL_DESC_TYPE_TXC;
			txd->ctl |= (buff->len_l3 << 31) |
						(buff->len_l2 << 24);
			txd->ctl2 |= (buff->mss << 16);
			is_gso = true;

			pkt_len -= (buff->len_l4 +
				    buff->len_l3 +
				    buff->len_l2);
			is_gso = true;

			if (buff->is_ipv6)
				txd->ctl |= HW_ATL2_TXD_CTL_CMD_IPV6;
			txd->ctl2 |= (buff->len_l4 << 8) |
				     (buff->len_l3 >> 1);
		}
		if (buff->is_vlan) {
			txd->ctl |= HW_ATL2_TXD_CTL_DESC_TYPE_TXC;
			txd->ctl |= buff->vlan_tx_tag << 4;
			is_vlan = true;
		}
		if (!buff->is_gso_tcp && !buff->is_gso_udp && !buff->is_vlan) {
			buff_pa_len = buff->len;

			txd->buf_addr = buff->pa;
			txd->ctl |= (HW_ATL2_TXD_CTL_BLEN &
						((u32)buff_pa_len << 4));
			txd->ctl |= HW_ATL2_TXD_CTL_DESC_TYPE_TXD;
			/* PAY_LEN */
			txd->ctl2 |= HW_ATL2_TXD_CTL2_LEN & (pkt_len << 14);

			if (is_gso || is_vlan) {
				/* enable tx context */
				txd->ctl2 |= HW_ATL2_TXD_CTL2_CTX_EN;
			}
			if (is_gso)
				txd->ctl |= HW_ATL2_TXD_CTL_CMD_LSO;

			/* Tx checksum offloads */
			if (buff->is_ip_cso)
				txd->ctl |= HW_ATL2_TXD_CTL_CMD_IPCSO;

			if (buff->is_udp_cso || buff->is_tcp_cso)
				txd->ctl |= HW_ATL2_TXD_CTL_CMD_TUCSO;

			if (is_vlan)
				txd->ctl |= HW_ATL2_TXD_CTL_CMD_VLAN;

			if (unlikely(buff->is_eop)) {
				txd->ctl |= HW_ATL2_TXD_CTL_EOP;
				txd->ctl |= HW_ATL2_TXD_CTL_CMD_WB;
				is_gso = false;
				is_vlan = false;
				if (unlikely(buff->request_ts)) {
					txd->ctl |= HW_ATL2_TXD_CTL_TS_EN;
					txd->ctl |= buff->clk_sel ? HW_ATL2_TXD_CTL_TS_PTP : 0;
					//aq_pr_trace("Request timestamp for desc %x. ctl %x, ctl2 %x\n", ring->sw_tail, txd->ctl, txd->ctl2);
				}
			}
		}
		/*trace_aq_tx_descriptor_a2(ring->idx, ring->sw_tail, (void *)txd);*/
		ring->sw_tail = aq_ring_next_dx(ring, ring->sw_tail);
	}

	wmb();

	hw_atl2_hw_tx_ring_tail_update(self, ring);
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_tx_ptp_ring_init(struct aq_hw_s *self,
				     struct aq_ring_s *aq_ring)
{
	hw_atl2_tdm_tx_desc_wb_ts_req_set(self, true, aq_ring->idx);
	hw_atl2_tdm_tx_desc_ts_req_set(self, true, aq_ring->idx);
	hw_atl2_tdm_tx_desc_lt_en_set(self, true, aq_ring->idx);
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_rx_ptp_ring_init(struct aq_hw_s *self,
				     struct aq_ring_s *aq_ring)
{
	hw_atl2_rdm_rx_desc_ts_req_set(self, clk_select, aq_ring->idx);
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_ring_rx_init(struct aq_hw_s *self,
				     struct aq_ring_s *aq_ring,
				     struct aq_ring_param_s *aq_ring_param)
{
	u32 dma_desc_addr_lsw = (u32)aq_ring->dx_ring_pa;
	u32 dma_desc_addr_msw = (u32)(((u64)aq_ring->dx_ring_pa) >> 32);
	u32 vlan_rx_stripping = self->aq_nic_cfg->is_vlan_rx_strip;

	hw_atl2_rdm_rx_desc_en_set(self, false, aq_ring->idx);

	hw_atl2_rdm_rx_desc_head_splitting_set(self, 0U, aq_ring->idx);

	hw_atl2_reg_rx_dma_desc_base_addresslswset(self, dma_desc_addr_lsw,
						  aq_ring->idx);

	hw_atl2_reg_rx_dma_desc_base_addressmswset(self,
						  dma_desc_addr_msw, aq_ring->idx);

	hw_atl2_rdm_rx_desc_len_set(self, aq_ring->size / 8U, aq_ring->idx);

	hw_atl2_rdm_rx_desc_data_buff_size_set(self,
					      AQ_CFG_RX_FRAME_MAX / 1024U,
				       aq_ring->idx);

	hw_atl2_rdm_rx_desc_head_buff_size_set(self, 0U, aq_ring->idx);
	hw_atl2_rdm_rx_desc_head_splitting_set(self, 0U, aq_ring->idx);
	hw_atl2_rpo_rx_desc_vlan_stripping_set(self, !!vlan_rx_stripping,
					       aq_ring->idx);

	/* Rx ring set mode */

	/* Mapping interrupt vector */
	hw_atl2_itr_irq_map_rx_set(self, aq_ring_param->vec_idx, aq_ring->idx);
	hw_atl2_itr_irq_map_en_rx_set(self, true, aq_ring->idx);

	hw_atl2_rdm_cpu_id_set(self, aq_ring_param->cpu, aq_ring->idx);
	hw_atl2_rdm_rx_desc_dca_en_set(self, 0U, aq_ring->idx);
	hw_atl2_rdm_rx_head_dca_en_set(self, 0U, aq_ring->idx);
	hw_atl2_rdm_rx_pld_dca_en_set(self, 0U, aq_ring->idx);

	if( aq_ptp_ring(aq_ring) )
		hw_atl2_hw_rx_ptp_ring_init(self, aq_ring);

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_ring_tx_init(struct aq_hw_s *self,
				     struct aq_ring_s *aq_ring,
				     struct aq_ring_param_s *aq_ring_param)
{
	u32 dma_desc_lsw_addr = (u32)aq_ring->dx_ring_pa;
	u32 dma_desc_msw_addr = (u32)(((u64)aq_ring->dx_ring_pa) >> 32);

	hw_atl2_reg_tx_dma_desc_base_addresslswset(self, dma_desc_lsw_addr,
						  aq_ring->idx);

	hw_atl2_reg_tx_dma_desc_base_addressmswset(self, dma_desc_msw_addr,
						  aq_ring->idx);

	hw_atl2_tdm_tx_desc_len_set(self, aq_ring->size / 8U, aq_ring->idx);

	hw_atl2_hw_tx_ring_tail_update(self, aq_ring);

	/* Set Tx threshold */
	hw_atl2_tdm_tx_desc_wr_wb_threshold_set(self, 0U, aq_ring->idx);

	/* Mapping interrupt vector */
	hw_atl2_itr_irq_map_tx_set(self, aq_ring_param->vec_idx, aq_ring->idx);
	hw_atl2_itr_irq_map_en_tx_set(self, true, aq_ring->idx);

	hw_atl2_tdm_cpu_id_set(self, aq_ring_param->cpu, aq_ring->idx);
	hw_atl2_tdm_tx_desc_dca_en_set(self, 0U, aq_ring->idx);

	if( aq_ptp_ring(aq_ring) )
		hw_atl2_hw_tx_ptp_ring_init(self, aq_ring);

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_ring_rx_fill(struct aq_hw_s *self,
				     struct aq_ring_s *ring,
				     unsigned int sw_tail_old)
{
	for (; sw_tail_old != ring->sw_tail;
		sw_tail_old = aq_ring_next_dx(ring, sw_tail_old)) {
		struct hw_atl2_rxd_s *rxd =
			(struct hw_atl2_rxd_s *)&ring->dx_ring[sw_tail_old *
							HW_ATL2_RXD_SIZE];

		struct aq_ring_buff_s *buff = &ring->buff_ring[sw_tail_old];

		rxd->buf_addr = buff->pa;
		rxd->hdr_addr = 0U;
	}

	wmb();

	hw_atl2_reg_rx_dma_desc_tail_ptr_set(self, sw_tail_old, ring->idx);

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_ring_tx_head_update(struct aq_hw_s *self,
					    struct aq_ring_s *ring)
{
	int err = 0;
	unsigned int hw_head_ = hw_atl2_tdm_tx_desc_head_ptr_get(self, ring->idx);

	if (aq_utils_obj_test(&self->flags, AQ_HW_FLAG_ERR_UNPLUG)) {
		err = -ENXIO;
		goto err_exit;
	}
	ring->hw_head = hw_head_;
	err = aq_hw_err_from_flags(self);

err_exit:
	return err;
}

static int hw_atl2_hw_ring_rx_receive(struct aq_hw_s *self,
					struct aq_ring_s *ring)
{
	for (; ring->hw_head != ring->sw_tail;
		ring->hw_head = aq_ring_next_dx(ring, ring->hw_head)) {
		struct aq_ring_buff_s *buff = NULL;
		struct hw_atl2_rxd_wb_s *rxd_wb = (struct hw_atl2_rxd_wb_s *)
			&ring->dx_ring[ring->hw_head * HW_ATL2_RXD_SIZE];

		unsigned int is_rx_check_sum_enabled = 0U;
		unsigned int pkt_type = 0U;
		u8 rx_stat = 0U;

		if (!(rxd_wb->status & 0x1U)) { /* RxD is not done */
			break;
		}

		/*trace_aq_rx_descr_a2(ring->idx, ring->hw_head, (u64*)rxd_wb);*/

		buff = &ring->buff_ring[ring->hw_head];

		buff->flags = 0U;
		buff->is_hash_l4 = 0U;

		rx_stat = (0x0000003CU & rxd_wb->status) >> 2;

		is_rx_check_sum_enabled = (rxd_wb->type >> 19) & 0x3U;

		pkt_type = (rxd_wb->type & HW_ATL2_RXD_WB_STAT_PKTTYPE) >>
			   HW_ATL2_RXD_WB_STAT_PKTTYPE_SHIFT;

		if (is_rx_check_sum_enabled & BIT(0) &&
		    (0x0U == (pkt_type & 0x3U)))
			buff->is_ip_cso = !(rx_stat & BIT(1));

		if (is_rx_check_sum_enabled & BIT(1)) {
			if (0x4U == (pkt_type & 0x1CU))
				buff->is_udp_cso = !(rx_stat & BIT(2));
			else if (0x0U == (pkt_type & 0x1CU))
				buff->is_tcp_cso = !(rx_stat & BIT(2));
		}
		buff->is_cso_err = !!(rx_stat & 0x6);
		/* Checksum offload workaround for small packets */
		if (unlikely(rxd_wb->pkt_len <= 60)) {
			buff->is_ip_cso = 0U;
			buff->is_cso_err = 0U;
		}

		if (self->aq_nic_cfg->is_vlan_rx_strip &&
			((pkt_type & HW_ATL2_RXD_WB_PKTTYPE_VLAN) ||
			 (pkt_type & HW_ATL2_RXD_WB_PKTTYPE_VLAN_DOUBLE))) {
			buff->is_vlan = 1;
			buff->vlan_rx_tag = le16_to_cpu(rxd_wb->vlan);;
		}

		if ((rx_stat & BIT(0)) || rxd_wb->type & 0x1000U) {
			/* MAC error or DMA error */
			buff->is_error = 1U;
		} else {
			if (self->aq_nic_cfg->is_rss) {
				/* last 4 byte */
				u16 rss_type = rxd_wb->type & 0xFU;

				if (rss_type && rss_type < 0x8U) {
					buff->is_hash_l4 = (rss_type == 0x4 ||
					rss_type == 0x5);
					buff->rss_hash = rxd_wb->rss_hash;
				}
			}

			if (HW_ATL2_RXD_WB_STAT2_EOP & rxd_wb->status) {
				buff->len = rxd_wb->pkt_len %
					AQ_CFG_RX_FRAME_MAX;
				buff->len = buff->len ?
					buff->len : AQ_CFG_RX_FRAME_MAX;
				buff->next = 0U;
				buff->is_eop = 1U;
			} else {
				buff->len = rxd_wb->pkt_len > AQ_CFG_RX_FRAME_MAX ?
					AQ_CFG_RX_FRAME_MAX : rxd_wb->pkt_len;

				if (HW_ATL2_RXD_WB_STAT2_RSCCNT &
					rxd_wb->status) {
					/* LRO */
					buff->next = rxd_wb->next_desc_ptr;
					++ring->stats.rx.lro_packets;
				} else {
					/* jumbo */
					buff->next =
						aq_ring_next_dx(ring,
								ring->hw_head);
					++ring->stats.rx.jumbo_packets;
				}
			}
		}
	}

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_irq_enable(struct aq_hw_s *self, u64 mask)
{
	hw_atl2_itr_irq_msk_setlsw_set(self, LODWORD(mask));
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_irq_disable(struct aq_hw_s *self, u64 mask)
{
	hw_atl2_itr_irq_msk_clearlsw_set(self, LODWORD(mask));
	hw_atl2_itr_irq_status_clearlsw_set(self, LODWORD(mask));

	atomic_inc(&self->dpc);
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_irq_read(struct aq_hw_s *self, u64 *mask)
{
	*mask = hw_atl2_itr_irq_statuslsw_get(self);
	return aq_hw_err_from_flags(self);
}

#define IS_FILTER_ENABLED(_F_) ((packet_filter & (_F_)) ? 1U : 0U)

static int hw_atl2_hw_packet_filter_set(struct aq_hw_s *self,
					  unsigned int packet_filter)
{
	unsigned int i = 0U;
	struct aq_nic_cfg_s *cfg = self->aq_nic_cfg;

	hw_atl2_rpfl2promiscuous_mode_en_set(self,
					     IS_FILTER_ENABLED(IFF_PROMISC));
	if (aq_new_filters_enabled) {
		hw_atl2_hw_new_rx_filter_promisc(self,
						IS_FILTER_ENABLED(IFF_PROMISC));
	}

	hw_atl2_rpf_vlan_prom_mode_en_set(self,
				     IS_FILTER_ENABLED(IFF_PROMISC) ||
				     cfg->is_vlan_force_promisc);

	hw_atl2_rpfl2multicast_flr_en_set(self,
					 IS_FILTER_ENABLED(IFF_ALLMULTI) &&
					 IS_FILTER_ENABLED(IFF_MULTICAST), 0);

	hw_atl2_rpfl2_accept_all_mc_packets_set(self,
					 IS_FILTER_ENABLED(IFF_ALLMULTI) &&
					 IS_FILTER_ENABLED(IFF_MULTICAST));

	hw_atl2_rpfl2broadcast_en_set(self, IS_FILTER_ENABLED(IFF_BROADCAST));

	for (i = HW_ATL2_MAC_MIN; i < HW_ATL2_MAC_MAX; ++i)
		hw_atl2_rpfl2_uc_flr_en_set(self,
				   (cfg->is_mc_list_enabled &&
				    (i <= cfg->mc_list_count)) ?
				    1U : 0U, i);

	return aq_hw_err_from_flags(self);
}

#undef IS_FILTER_ENABLED

static int hw_atl2_hw_multicast_list_set(struct aq_hw_s *self,
					   u8 ar_mac
					   [AQ_HW_MULTICAST_ADDRESS_MAX]
					   [ETH_ALEN],
					   u32 count)
{
	int err = 0;

	if (count > (HW_ATL2_MAC_MAX - HW_ATL2_MAC_MIN)) {
		err = -EBADRQC;
		goto err_exit;
	}
	for (self->aq_nic_cfg->mc_list_count = 0U;
			self->aq_nic_cfg->mc_list_count < count;
			++self->aq_nic_cfg->mc_list_count) {
		u32 i = self->aq_nic_cfg->mc_list_count;
		u32 h = (ar_mac[i][0] << 8) | (ar_mac[i][1]);
		u32 l = (ar_mac[i][2] << 24) | (ar_mac[i][3] << 16) |
					(ar_mac[i][4] << 8) | ar_mac[i][5];

		hw_atl2_rpfl2_uc_flr_en_set(self, 0U, HW_ATL2_MAC_MIN + i);

		hw_atl2_rpfl2unicast_dest_addresslsw_set(self,
							l, HW_ATL2_MAC_MIN + i);

		hw_atl2_rpfl2unicast_dest_addressmsw_set(self,
							h, HW_ATL2_MAC_MIN + i);
		if (aq_new_filters_enabled)
			hw_atl2_rpfl2_uc_flr_tag_set(self,
						     self->aq_nic_cfg->special_uc_tag[i] + HW_ATL2_RPF_TAG_BASE_UC,
						     HW_ATL2_MAC_MIN + i);

		hw_atl2_rpfl2_uc_flr_en_set(self,
					   (self->aq_nic_cfg->is_mc_list_enabled),
					   HW_ATL2_MAC_MIN + i);
	}

	err = aq_hw_err_from_flags(self);

err_exit:
	return err;
}

static int hw_atl2_hw_interrupt_moderation_set(struct aq_hw_s *self)
{
	unsigned int i = 0U;
	u32 itr_tx = 2U;
	u32 itr_rx = 2U;

	switch (self->aq_nic_cfg->itr) {
	case  AQ_CFG_INTERRUPT_MODERATION_ON:
	case  AQ_CFG_INTERRUPT_MODERATION_AUTO:
		hw_atl2_tdm_tx_desc_wr_wb_irq_en_set(self, 0U);
		hw_atl2_tdm_tdm_intr_moder_en_set(self, 1U);
		hw_atl2_rdm_rx_desc_wr_wb_irq_en_set(self, 0U);
		hw_atl2_rdm_rdm_intr_moder_en_set(self, 1U);

		if (self->aq_nic_cfg->itr == AQ_CFG_INTERRUPT_MODERATION_ON) {
			/* HW timers are in 2us units */
			int tx_max_timer = self->aq_nic_cfg->tx_itr / 2;
			int tx_min_timer = tx_max_timer / 2;

			int rx_max_timer = self->aq_nic_cfg->rx_itr / 2;
			int rx_min_timer = rx_max_timer / 2;

			tx_max_timer = min(HW_ATL2_INTR_MODER_MAX, tx_max_timer);
			tx_min_timer = min(HW_ATL2_INTR_MODER_MIN, tx_min_timer);
			rx_max_timer = min(HW_ATL2_INTR_MODER_MAX, rx_max_timer);
			rx_min_timer = min(HW_ATL2_INTR_MODER_MIN, rx_min_timer);

			itr_tx |= tx_min_timer << 0x8U;
			itr_tx |= tx_max_timer << 0x10U;
			itr_rx |= rx_min_timer << 0x8U;
			itr_rx |= rx_max_timer << 0x10U;
		} else {
			static unsigned int hw_atl2_timers_table_tx_[][2] = {
				{0xfU, 0xffU}, /* 10Gbit */
				{0xfU, 0x1ffU}, /* 5Gbit */
				{0xfU, 0x1ffU}, /* 5Gbit 5GS */
				{0xfU, 0x1ffU}, /* 2.5Gbit */
				{0xfU, 0x1ffU}, /* 1Gbit */
				{0xfU, 0x1ffU}, /* 100Mbit */
			};

			static unsigned int hw_atl2_timers_table_rx_[][2] = {
				{0x6U, 0x38U},/* 10Gbit */
				{0xCU, 0x70U},/* 5Gbit */
				{0xCU, 0x70U},/* 5Gbit 5GS */
				{0x18U, 0xE0U},/* 2.5Gbit */
				{0x30U, 0x80U},/* 1Gbit */
				{0x4U, 0x50U},/* 100Mbit */
			};

			unsigned int speed_index = 0;
					/*hw_atl2_utils_mbps_2_speed_index(
						self->aq_link_status.mbps);*/

			/* Update user visible ITR settings */
			self->aq_nic_cfg->tx_itr = hw_atl2_timers_table_tx_
							[speed_index][1] * 2;
			self->aq_nic_cfg->rx_itr = hw_atl2_timers_table_rx_
							[speed_index][1] * 2;

			itr_tx |= hw_atl2_timers_table_tx_
						[speed_index][0] << 0x8U;
			itr_tx |= hw_atl2_timers_table_tx_
						[speed_index][1] << 0x10U;

			itr_rx |= hw_atl2_timers_table_rx_
						[speed_index][0] << 0x8U;
			itr_rx |= hw_atl2_timers_table_rx_
						[speed_index][1] << 0x10U;
		}
		break;
	case AQ_CFG_INTERRUPT_MODERATION_OFF:
		hw_atl2_tdm_tx_desc_wr_wb_irq_en_set(self, 1U);
		hw_atl2_tdm_tdm_intr_moder_en_set(self, 0U);
		hw_atl2_rdm_rx_desc_wr_wb_irq_en_set(self, 1U);
		hw_atl2_rdm_rdm_intr_moder_en_set(self, 0U);
		itr_tx = 0U;
		itr_rx = 0U;
		break;
	}

	for (i = HW_ATL2_RINGS_MAX; i--;) {
		hw_atl2_reg_tx_intr_moder_ctrl_set(self, itr_tx, i);
		hw_atl2_reg_rx_intr_moder_ctrl_set(self, itr_rx, i);
	}

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_stop(struct aq_hw_s *self)
{
	int err;
	u32 val;

	hw_atl2_hw_irq_disable(self, HW_ATL2_INT_MASK);

	/* Invalidate Descriptor Cache to prevent writing to the cached
	 * descriptors and to the data pointer of those descriptors
	 */
	hw_atl2_rdm_rx_dma_desc_cache_init_tgl(self);

	err = aq_hw_err_from_flags(self);

	if (err)
		goto err_exit;

	readx_poll_timeout_atomic(hw_atl2_rdm_rx_dma_desc_cache_init_done_get,
				  self, val, val == 1, 1000U, 10000U);

err_exit:
	return err;
}

static int hw_atl2_hw_ring_tx_stop(struct aq_hw_s *self,
				     struct aq_ring_s *ring)
{
	hw_atl2_tdm_tx_desc_en_set(self, 0U, ring->idx);
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_ring_rx_stop(struct aq_hw_s *self,
				     struct aq_ring_s *ring)
{
	hw_atl2_rdm_rx_desc_en_set(self, 0U, ring->idx);
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_set_loopback(struct aq_hw_s *self, u32 mode, bool enable)
{
	switch (mode) {
	case AQ_HW_LOOPBACK_DMA_SYS:
		hw_atl2_tpb_tx_dma_sys_lbk_en_set(self, enable);
		hw_atl2_rpb_dma_sys_lbk_set(self, enable);
		break;
	case AQ_HW_LOOPBACK_PKT_SYS:
		hw_atl2_tpo_tx_pkt_sys_lbk_en_set(self, enable);
		hw_atl2_rpf_tpo_to_rpf_sys_lbk_set(self, enable);
		break;
	case AQ_HW_LOOPBACK_DMA_NET:
		hw_atl2_tpb_tx_dma_net_lbk_en_set(self, enable);
		hw_atl2_rpb_dma_net_lbk_set(self, enable);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int hw_atl2_tx_tc_mode_get(struct aq_hw_s *self, u32 *tc_mode)
{
	*tc_mode = hw_atl2_rpb_tps_tx_tc_mode_get(self);
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_rx_tc_mode_get(struct aq_hw_s *self, u32 *tc_mode)
{
	*tc_mode = hw_atl2_rpb_rpf_rx_traf_class_mode_get(self);
	return aq_hw_err_from_flags(self);
}

static struct aq_stats_s *hw_atl2_utils_get_hw_stats(struct aq_hw_s *self)
{
	return &self->curr_stats;
}

static u32 aq_atl2_clk_freq(struct aq_hw_s *self)
{
	return AQ2_HW_PTP_COUNTER_HZ;
}

static void hw_atl2_enable_ptp(struct aq_hw_s *self, unsigned int param, int enable)
{
	s64 freq, base_ns, divisor;
	u32 ns = 0, fns = 0;
	clk_select = param;

	// Enable TC priority
	hw_atl2_tpb_tx_packet_scheduler_highest_priority_tc_enable_set(self, enable);
	freq = aq_atl2_clk_freq(self);
	base_ns = (1000000000ll * 1000000000ll) / freq;
	ns = base_ns / 1000000000ll;
	if( base_ns != ns * 1000000000ll ){
		divisor = 1000000000000000000ll / (base_ns - ns * 1000000000ll);
		fns = 0x100000000ll * 1000000000ll / divisor;
	}
	//enable tsg counter
	hw_atl2_tsg_clock_en(self, clk_select, enable);
	//hw_atl2_tsg_clock_reset(self, clk_select);
	if( enable ){
		hw_atl2_tsg_clock_increment_set(self, clk_select, ns, fns);
	}
}

static void aq_get_ptp_ts(struct aq_hw_s *self, u64 *stamp)
{
	if( stamp ) *stamp = hw_atl2_tsg_clock_read(self, clk_select);
	//aq_pr_trace("Sys TS: %llu\n", *stamp);
}

static u64 hw_atl2_hw_ring_tx_ptp_get_ts(struct aq_ring_s *ring)
{
	struct hw_atl2_txts_s *txts = (struct hw_atl2_txts_s *)&ring->dx_ring[ring->sw_head *
						HW_ATL2_TXD_SIZE];
	if( txts->ctrl & (1<<3) && txts->ctrl & (1<<20) ) { //DD + TS_VALIS
		//aq_pr_trace("Egress TS: %llu\n", txts->ts);
		return txts->ts;
	}
	return 0;
}

static u16 hw_atl2_hw_rx_extract_ts(u8 *p, unsigned int len, u64 *timestamp)
{
	unsigned int offset = HW_ATL2_RX_TS_SIZE;
	u8 *ptr;
	//u64 ns;

	if (len <= offset || !timestamp)
		return 0;

	ptr = p + (len - offset);
	memcpy(timestamp, ptr, sizeof(*timestamp));

	//*timestamp = be64_to_cpu(ns);
	//aq_pr_trace("Ingress TS: %llu\n", *timestamp);
	return HW_ATL2_RX_TS_SIZE;
}

static int hw_atl2_adj_sys_clock(struct aq_hw_s *self, s64 delta)
{
	if( delta >= 0 ) {
		hw_atl2_tsg_clock_add(self, clk_select, (u64)delta, 0);
	}
	else {
		hw_atl2_tsg_clock_sub(self, clk_select, (u64)(-delta), 0);
	}
	return 0;
}

static int hw_atl2_adj_clock_freq(struct aq_hw_s *self, s32 ppb)
{
	s64 adj = ppb, freq = aq_atl2_clk_freq(self);
	s64 divisor = 0, base_ns = ((adj + 1000000000ll) * 1000000000ll) / freq; //7999935952
	u32 nsi_frac = 0, nsi = base_ns / 1000000000ll; //7
	if( base_ns != nsi * 1000000000ll ){
		divisor = 1000000000000000000ll / (base_ns - nsi * 1000000000ll); // 1000064052 
		nsi_frac = 0x100000000ll * 1000000000ll / divisor; //4294692212
	}
	//aq_pr_trace("PTP Adj Freq %lld, ppb %lld. divisor %lld. nsi %d fnsi %x\n", 
	//			freq, adj, divisor, nsi, nsi_frac);
	hw_atl2_tsg_clock_increment_set(self, clk_select, nsi, nsi_frac);
	return 0;
}

static u32 hw_atl2_hw_get_clk_sel(struct aq_hw_s *self)
{
	return clk_select == ATL_TSG_CLOCK_SEL_1 ? 0 : 1;
}

static u32 ext_int_flags = 0;
static int hw_atl2_ptp_ext_interrupt_en(struct aq_hw_s *self, int on, u32 flags)
{
	struct aq_nic_cfg_s *aq_nic_cfg = self->aq_nic_cfg;
	if( (!ext_int_flags && on) ||
		(!(ext_int_flags ^ flags) && !on) ) {
		hw_atl2_tsg_ext_isr_to_host_set(self, on);
		hw_atl2_reg_gen_irq_map_set(self,
						(BIT(0x7) |   aq_nic_cfg->link_irq_vec) |
				   (on ? BIT(0xF) | ((aq_nic_cfg->link_irq_vec + 2) << 0x8) : 0), 3U); //MIF2
	}

	if( on ) {
		ext_int_flags |= flags;
	} else {
		ext_int_flags &= ~flags;
	}
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_ptp_gpio_in_en(struct aq_hw_s *self, u32 index, u32 channel, int on)
{
	hw_atl2_tsg_gpio_isr_to_host_set(self, on, clk_select);
	hw_atl2_tsg_gpio_clear_status(self, clk_select);
	hw_atl2_tsg_gpio_input_set(self, on, index, clk_select);
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_gpio_pulse(struct aq_hw_s *self, u32 index, u32 clk_sel, u64 start, u32 period, u32 hightime)
{
	hw_atl2_tsg_ptp_gpio_gen_pulse(self, clk_sel, start, period, hightime);
	return 0;
}

static int hw_atl2_hw_filter_chain_build(struct aq_hw_s *self, struct aq_rx_filter *data, bool add)
{
	struct ethtool_rx_flow_spec *fsp = &data->aq_fsp;
	u8 location = (fsp->location & AQ_RX_FILTER_CHAIN_MASK) - AQ_RX_FIRST_LOC_CHAIN;
	u32 req_tag = 0;
	u32 mask = 0;
	u32 queue = fsp->ring_cookie & 0x1f;
	u32 action = 0;

	if( data->type & aq_rx_filter_type_vlan ) {
		u32 vlan_loc = (fsp->location & AQ_RX_FILTER_CHAIN_VLAN_LOC_MASK) >> AQ_RX_FILTER_CHAIN_VLAN_LOC_SHIFT;
		req_tag |= (vlan_loc + 2) << HW_ATL2_RPF_TAG_VLAN_OFFSET;
		mask |= HW_ATL2_RPF_TAG_VLAN_MASK;
	}

	if( data->type & aq_rx_filter_type_ethertype ) {
		u32 et_loc = (fsp->location & AQ_RX_FILTER_CHAIN_L2ET_LOC_MASK) >> AQ_RX_FILTER_CHAIN_L2ET_LOC_SHIFT;
		u32 dst_loc = (fsp->location & AQ_RX_FILTER_CHAIN_L2_DST_MASK) >> AQ_RX_FILTER_CHAIN_L2_DST_SHIFT;

		if( et_loc ) {
			req_tag |= et_loc << HW_ATL2_RPF_TAG_ET_OFFSET;
			mask |= HW_ATL2_RPF_TAG_ET_MASK;
		}

		if( dst_loc ) {
			req_tag |= dst_loc << HW_ATL2_RPF_TAG_UC_OFFSET;
			mask |= HW_ATL2_RPF_TAG_UC_MASK;
		}
	}

	if( data->type & aq_rx_filter_type_l3 ) {
		u32 l3_loc = (fsp->location & AQ_RX_FILTER_CHAIN_L3_LOC_MASK) >> AQ_RX_FILTER_CHAIN_L3_LOC_SHIFT;
		bool is_ipv6 =  ((fsp->flow_type & ~FLOW_EXT) == TCP_V6_FLOW) || 
						((fsp->flow_type & ~FLOW_EXT) == UDP_V6_FLOW) || 
						((fsp->flow_type & ~FLOW_EXT) == SCTP_V6_FLOW) || 
						((fsp->flow_type & ~FLOW_EXT) == IPV6_USER_FLOW) || 
						((fsp->flow_type & ~FLOW_EXT) == IPV6_FLOW);
		if (is_ipv6) {
			req_tag |= (l3_loc + 1) <<
				   HW_ATL2_RPF_TAG_L3_V6_OFFSET;
			mask |= HW_ATL2_RPF_TAG_L3_V6_MASK;
		} else {
			req_tag |= (l3_loc + 1) <<
				   HW_ATL2_RPF_TAG_L3_V4_OFFSET;
			mask |= HW_ATL2_RPF_TAG_L3_V4_MASK;
		}
	}

	if( data->type & aq_rx_filter_type_l3l4 ) {
		u32 l4_loc = (fsp->location & AQ_RX_FILTER_CHAIN_L3L4_LOC_MASK) >> AQ_RX_FILTER_CHAIN_L3L4_LOC_SHIFT;
		req_tag |= (l4_loc + 1) << HW_ATL2_RPF_TAG_L4_OFFSET;
		mask |= HW_ATL2_RPF_TAG_L4_MASK;
	}

	if( fsp->m_ext.vlan_tci & VLAN_PRIO_MASK ) {
		req_tag |= (be16_to_cpu(fsp->h_ext.vlan_tci)
			       & VLAN_PRIO_MASK) << (HW_ATL2_RPF_TAG_PCP_OFFSET - VLAN_PRIO_SHIFT);
		mask |= HW_ATL2_RPF_TAG_PCP_MASK;
	}

	//printk("%s filter chain. Origianl type %x, Loc %d, tag %x, mask %x. queue %x\n", add ? "Add" : "Del", data->type, location, req_tag, mask, queue);
	action = HW_ATL2_ACTION_ASSIGN_QUEUE(queue);

	hw_atl2_act_rslvr_table_set(self,
		HW_ATL2_RPF_CHAIN_INDEX + location,
		req_tag,
		mask,
		action);

	return 0;
}

static void hw_atl2_hw_filter_flex_fill_byte_mask(struct aq_hw_s *self, u32 mask[128], unsigned long byteen[4], u32 size, u32 filter)
{
	u32 i, j, bytemask = 0;
	hw_atl2_rpf_flex_byte_mask_len_set(self, (size + 3) >> 2);
	hw_atl2_rpf_flex_bytepat_wr_en_set(self, 0);
	for( i = 0; i < (size + sizeof(u32) - 1)/sizeof(u32); i++) {
		u32 value = mask[i];
		for( j = 0; j < sizeof(u32); j++ ) {
			value = value & (~(test_bit(i*4 + j, byteen) ? 0 : (0xff << (j*8))));
			bytemask |= test_bit(i*4 + j, byteen) ? (1 << ((i & 0x7)*4 + j)) : 0;
		}
		//printk("Flex mask: %02d DWORD %08x, mask %08x", i, value, bytemask);
		if( (i & 0x7) == 0x7 || 
		    (i + 1 >= (size + sizeof(u32) - 1)/sizeof(u32)) ) {
			hw_atl2_rpf_flex_mask_word_set(self, bytemask, filter, i >> 3);
			bytemask = 0;
		}
		hw_atl2_rpf_flex_bytepat_data_set(self, ntohl(value));
		hw_atl2_rpf_flex_bytepat_addr_set(self, i);
		hw_atl2_rpf_flex_bytepat_sel_set(self, filter);
		hw_atl2_rpf_flex_bytepat_wr_en_set(self, 1);
		hw_atl2_rpf_flex_bytepat_wr_en_set(self, 0);
	}
}

static int hw_atl2_hw_filter_flex_set(struct aq_hw_s *self, struct aq_rx_filter_flex *data)
{	
	if( data->cmd & HW_ATL_RX_ENABLE_FLTR_FLEX ) {
		if( data->cmd & HW_ATL_RX_ENABLE_QUEUE_FLEX ) {
			hw_atl2_rpf_flex_rxqen_set(self, !!(data->cmd & HW_ATL_RX_ENABLE_FLTR_FLEX), data->location);
			hw_atl2_rpf_flex_rxqf_set(self, 
				(data->cmd & HW_ATL2_RPF_FLEX_RXQH_MSK) >> HW_ATL2_RPF_FLEX_RXQH_SHIFT, 
				 data->location);
		}
		hw_atl2_rpf_flex_act_set(self, 
				(data->cmd & HW_ATL2_RPF_FLEX_ACTH_MSK) >> HW_ATL2_RPF_FLEX_ACTH_SHIFT, 
				 data->location);

		hw_atl2_rpf_flex_byte_a_loc_set(self, data->offset_a, data->location);
		hw_atl2_rpf_flex_byte_b_loc_set(self, data->offset_b, data->location);

		hw_atl2_rpf_flex_byte_a_msk_set(self, data->mask_a, data->location);
		hw_atl2_rpf_flex_byte_b_msk_set(self, data->mask_b, data->location);

		hw_atl2_rpf_flex_byte_a_pat_set(self, data->biten_a, data->location);
		hw_atl2_rpf_flex_byte_b_pat_set(self, data->biten_b, data->location);
		hw_atl2_hw_filter_flex_fill_byte_mask(self, data->mask, data->byteen, data->filter_size, data->location);
	}
	hw_atl2_rpf_flex_en_set(self, !!(data->cmd & HW_ATL_RX_ENABLE_FLTR_FLEX), data->location);

	//printk("%s Flex filter. Place %d.", data->cmd ? "Add" : "Del", data->location);
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_fl3_clear(struct aq_hw_s *self,
				    struct aq_rx_filter_l3 *data)
{
	u8 location = data->location;
	u8 queue = 0xFF;

	if( data->is_ipv6 ) {
		hw_atl2_rpf_l3_v6_cmd_set(self, 0, location);
		hw_atl2_rpf_l3_v6_tag_set(self, 0, location);
	} else {
		hw_atl2_rpf_l3_v4_cmd_set(self, 0, location);
		hw_atl2_rpf_l3_v4_tag_set(self, 0, location);
	}

	queue = (data->cmd  >> HW_ATL2_RPF_L3_L4_RXQF_SHIFT) & 0x7f;
	if( queue != 0x7F ) {
		hw_atl2_act_rslvr_table_set(self,
			HW_ATL2_RPF_L3L4_USER_INDEX + data->location,
			0,
			0,
			HW_ATL2_ACTION_DISABLE);
	}
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_fl3l4_clear(struct aq_hw_s *self,
				    struct aq_rx_filter_l3l4 *data)
{
	u8 location = data->location;
	if (!data->is_ipv6) {
		hw_atl2_rpfl3l4_cmd_clear(self, location);
		hw_atl2_rpf_l4_spd_set(self, 0U, location);
		hw_atl2_rpf_l4_dpd_set(self, 0U, location);
		hw_atl2_rpfl3l4_ipv4_src_addr_clear(self, location);
		hw_atl2_rpfl3l4_ipv4_dest_addr_clear(self, location);
	} else {
		int i;

		for (i = 0; i < HW_ATL_RX_CNT_REG_ADDR_IPV6; ++i) {
			hw_atl2_rpfl3l4_cmd_clear(self, location + i);
			hw_atl2_rpf_l4_spd_set(self, 0U, location + i);
			hw_atl2_rpf_l4_dpd_set(self, 0U, location + i);
		}
		hw_atl2_rpfl3l4_ipv6_src_addr_clear(self, location);
		hw_atl2_rpfl3l4_ipv6_dest_addr_clear(self, location);
	}

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_fl3_set(struct aq_hw_s *self,
				  		struct aq_rx_filter_l3 *data)
{
	struct hw_atl2_l3_filter l3 = {0};
	u8 location = data->location;
	u8 queue = 0xFF;
	u32 req_tag = 0;
	u32 mask = 0;
	u16 action = 0;

	hw_atl2_hw_fl3_clear(self, data);

	if (data->cmd & HW_ATL_RX_ENABLE_CMP_DEST_ADDR_L3)
		l3.cmd_l3 |= HW_ATL2_RPF_L3_CMD_DA_EN;
	if (data->cmd & HW_ATL_RX_ENABLE_CMP_SRC_ADDR_L3)
		l3.cmd_l3 |= HW_ATL2_RPF_L3_CMD_SA_EN;
	if (l3.cmd_l3)
		l3.cmd_l3 |= HW_ATL2_RPF_L3_CMD_EN;

	if( data->cmd & HW_ATL_RX_ENABLE_CMP_PROT_L4 ) {
		l3.cmd_l3 |= HW_ATL2_RPF_L3_CMD_PROTO_EN;
		l3.cmd_l3 |= ((data->cmd & HW_ATL_RX_TCP ) ? 0x06U : //TCP
					  (data->cmd & HW_ATL_RX_UDP ) ? 0x11U : //UDP
					 								0xFFu) << 8;  //UNKNOWN
	}

	if (data->cmd & HW_ATL_RX_ENABLE_L3_IPV6) {
		l3.ipv6 = 1;
		memcpy(l3.srcip, data->ip_src, sizeof(l3.srcip));
		memcpy(l3.dstip, data->ip_dst, sizeof(l3.dstip));
		hw_atl2_rpf_l3_v6_dest_addr_set(self,
						location,
						data->ip_dst);
		hw_atl2_rpf_l3_v6_src_addr_set(self,
							location,
							data->ip_src);
		hw_atl2_rpf_l3_v6_cmd_set(self, l3.cmd_l3, location);
		hw_atl2_rpf_l3_v6_tag_set(self, location + 1, location);
	} else {
		l3.srcip[0] =  data->ip_src[0];
		l3.dstip[0] =  data->ip_dst[0];
		hw_atl2_rpf_l3_v4_dest_addr_set(self,
						location,
						data->ip_dst[0]);
		hw_atl2_rpf_l3_v4_src_addr_set(self,
							location,
							data->ip_src[0]);
		hw_atl2_rpf_l3_v6_v4_select_set(self, 1);
		hw_atl2_rpf_l3_v4_cmd_set(self, l3.cmd_l3, location);
		hw_atl2_rpf_l3_v4_tag_set(self, location + 1, location);
	}

	if (l3.ipv6) {
		req_tag |= (location + 1) <<
				HW_ATL2_RPF_TAG_L3_V6_OFFSET;
		mask |= HW_ATL2_RPF_TAG_L3_V6_MASK;
	} else {
		req_tag |= (location + 1) <<
				HW_ATL2_RPF_TAG_L3_V4_OFFSET;
		mask |= HW_ATL2_RPF_TAG_L3_V4_MASK;
	}

	queue = (data->cmd  >> HW_ATL2_RPF_L3_L4_RXQF_SHIFT) & 0x7f;
	if( queue != 0x7F ) {
		if (data->cmd & (HW_ATL_RX_HOST << HW_ATL2_RPF_L3_L4_ACTF_SHIFT))
			action = HW_ATL2_ACTION_ASSIGN_QUEUE(queue);
		else if (data->cmd)
			action = HW_ATL2_ACTION_DROP;
		else
			action = HW_ATL2_ACTION_DISABLE;
		hw_atl2_act_rslvr_table_set(self,
			HW_ATL2_RPF_L3L4_USER_INDEX + data->location,
			req_tag,
			mask,
			action);
	}
	return 0;
}

static int hw_atl2_hw_fl4_set(struct aq_hw_s *self,
				  struct aq_rx_filter_l3l4 *data)
{
	struct hw_atl2_l4_filter l4 = {0};
	u8 location = data->location;
	u8 queue = 0xFF;
	u32 req_tag = 0;
	u32 mask = 0;
	u16 action = 0;

	if (data->cmd & HW_ATL_RX_ENABLE_CMP_DEST_PORT_L4)
		l4.cmd_l4 |= HW_ATL2_RPF_L4_CMD_DP_EN;
	if (data->cmd & HW_ATL_RX_ENABLE_CMP_SRC_PORT_L4)
		l4.cmd_l4 |= HW_ATL2_RPF_L4_CMD_SP_EN;
	if (l4.cmd_l4)
		l4.cmd_l4 |= HW_ATL2_RPF_L4_CMD_EN;
	l4.sport = data->p_src;
	l4.dport = data->p_dst;

	hw_atl2_rpf_l4_tag_set(self, location + 1, location);
	hw_atl2_rpf_l4_cmd_set(self, l4.cmd_l4, location);

	req_tag |= (location + 1) << HW_ATL2_RPF_TAG_L4_OFFSET;
	mask |= HW_ATL2_RPF_TAG_L4_MASK;

	queue = (data->cmd  >> HW_ATL2_RPF_L3_L4_RXQF_SHIFT) & 0x7f;
	if( queue != 0x7F ) {
		if (data->cmd & (HW_ATL_RX_HOST << HW_ATL2_RPF_L3_L4_ACTF_SHIFT))
			action = HW_ATL2_ACTION_ASSIGN_QUEUE(queue);
		else if (data->cmd)
			action = HW_ATL2_ACTION_DROP;
		else
			action = HW_ATL2_ACTION_DISABLE;

		hw_atl2_act_rslvr_table_set(self,
			HW_ATL2_RPF_L3L4_USER_INDEX + data->location,
			req_tag,
			mask,
			action);
	}
	return 0;
}

static int hw_atl2_hw_fl3l4_set(struct aq_hw_s *self,
				  struct aq_rx_filter_l3l4 *data)
{
	u8 location = data->location;

	hw_atl2_hw_fl3l4_clear(self, data);

	hw_atl2_rpf_l4_dpd_set(self, data->p_dst, location);
	hw_atl2_rpf_l4_spd_set(self, data->p_src, location);

	if (aq_new_filters_enabled) {
		hw_atl2_hw_fl4_set(self, data);
	} else {
		if (data->cmd) {
			if (!data->is_ipv6) {
				hw_atl2_rpfl3l4_ipv4_dest_addr_set(self,
								location,
								data->ip_dst[0]);
				hw_atl2_rpfl3l4_ipv4_src_addr_set(self,
								location,
								data->ip_src[0]);
			} else {
				hw_atl2_rpfl3l4_ipv6_dest_addr_set(self,
								location,
								data->ip_dst);
				hw_atl2_rpfl3l4_ipv6_src_addr_set(self,
								location,
								data->ip_src);
			}
		}
		hw_atl2_rpfl3l4_cmd_set(self, location, data->cmd);
		hw_atl2_rpf_l3_l4_enf_set(self, 1, location);
	}

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_fl2_set(struct aq_hw_s *self,
				struct aq_rx_filter_l2 *data)
{
	u32 req_tag = 0;
	u32 mask = 0;
	u16 action = 0;
	u8 et_loc = data->location & 0xf;
	u8 dst_loc = (data->location & 0xf0) >> 4;

	if( data->en_flag & HW_ATL_RX_L2_ET_EN ) {
		req_tag = (et_loc + 1) << HW_ATL2_RPF_TAG_ET_OFFSET;
		mask = HW_ATL2_RPF_TAG_ET_MASK;
		hw_atl2_rpf_etht_flr_en_set(self, 1U, et_loc);
		hw_atl2_rpf_etht_flr_set(self, data->ethertype, et_loc);
		hw_atl2_rpf_etht_user_priority_en_set(self,
							!!(data->en_flag & HW_ATL_RX_L2_UP_EN),
							et_loc);
		if( data->en_flag & HW_ATL_RX_L2_UP_EN ) {
			hw_atl2_rpf_etht_user_priority_set(self,
							data->user_priority,
							et_loc);
			req_tag |= data->user_priority << HW_ATL2_RPF_TAG_PCP_OFFSET;
			mask |= HW_ATL2_RPF_TAG_PCP_MASK;
		}

		if (data->queue == 0xFF ) {
			hw_atl2_rpf_etht_flr_act_set(self, 0U, et_loc);
			hw_atl2_rpf_etht_rx_queue_en_set(self, 0U, et_loc);
		} else {
			hw_atl2_rpf_etht_flr_act_set(self, 1U, et_loc);
			hw_atl2_rpf_etht_rx_queue_en_set(self, 1U, et_loc);
			if( data->queue != 0x7F ) {
				hw_atl2_rpf_etht_rx_queue_set(self, data->queue, et_loc);
			}
		}
	}

	if( data->en_flag & HW_ATL_RX_L2_DST_EN ) {
		req_tag |= (dst_loc) << HW_ATL2_RPF_TAG_UC_OFFSET;
		mask |= HW_ATL2_RPF_TAG_UC_MASK;
		//printk("Change uc/mc filter to tag %x!\n", dst_loc);
		self->aq_nic_cfg->special_uc_tag[dst_loc - HW_ATL2_MAC_MIN] = dst_loc - HW_ATL2_RPF_TAG_BASE_UC;
		hw_atl2_rpfl2_uc_flr_tag_set(self,
							dst_loc,
							dst_loc);
	}

	if (data->queue == 0xFF ) {
		action = HW_ATL2_ACTION_DROP;
	} else if( data->queue != 0x7F ) {
		action = HW_ATL2_ACTION_ASSIGN_QUEUE(data->queue);
	}

	if (aq_new_filters_enabled ) { 
		if( data->en_flag & HW_ATL_RX_L2_ET_EN ) {
			hw_atl2_rpf_etht_flr_tag_set(self,
							et_loc + 1,
							et_loc);
		}

		if( data->queue != 0x7F ) {
			hw_atl2_act_rslvr_table_set(self,
				data->en_flag & HW_ATL_RX_L2_ET_EN ? 
					HW_ATL2_RPF_ET_PCP_USER_INDEX + et_loc : 
					HW_ATL2_RPF_DST_FIRST_INDEX + et_loc,
				req_tag,
				mask,
				action);
		}
	}

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_fl2_clear(struct aq_hw_s *self,
				  struct aq_rx_filter_l2 *data)
{
	u8 et_loc = data->location & 0xf;
	u8 dst_loc = (data->location & 0xf0) >> 4;

	hw_atl2_rpf_etht_flr_en_set(self, 0U, et_loc);
	hw_atl2_rpf_etht_flr_set(self, 0U, et_loc);
	hw_atl2_rpf_etht_user_priority_en_set(self, 0U, et_loc);
	if( data->en_flag & HW_ATL_RX_L2_DST_EN ) {
		//printk("Change uc/mc filter from tag %x to 1!\n", dst_loc);
		self->aq_nic_cfg->special_uc_tag[dst_loc - HW_ATL2_MAC_MIN] = 0;
		hw_atl2_rpfl2_uc_flr_tag_set(self,
							HW_ATL2_RPF_TAG_BASE_UC,
							dst_loc);
	}

	if (aq_new_filters_enabled)
		hw_atl2_act_rslvr_table_set(self,
			data->en_flag & HW_ATL_RX_L2_ET_EN ? 
					HW_ATL2_RPF_ET_PCP_USER_INDEX + et_loc : 
					HW_ATL2_RPF_DST_FIRST_INDEX + et_loc,
			0,
			0,
			HW_ATL2_ACTION_DISABLE);

	return aq_hw_err_from_flags(self);
}

/**
 * @brief Set VLAN filter table
 * @details Configure VLAN filter table to accept (and assign the queue) traffic
 *  for the particular vlan ids.
 * Note: use this function under vlan promisc mode not to lost the traffic
 *
 * @param aq_hw_s
 * @param aq_rx_filter_vlan VLAN filter configuration
 * @return 0 - OK, <0 - error
 */
static int hw_atl2_hw_vlan_set(struct aq_hw_s *self,
				  struct aq_rx_filter_vlan *aq_vlans)
{
	int i;

	for (i = 0; i < AQ_RX_QUEUE_NOT_ASSIGNED; i++) {
		hw_atl2_rpf_vlan_flr_en_set(self, 0U, i);
		hw_atl2_rpf_vlan_rxq_en_flr_set(self, 0U, i);
		hw_atl2_act_rslvr_table_set(self,
			HW_ATL2_RPF_VLAN_USER_INDEX + i,
			0,
			0,
			HW_ATL2_ACTION_DISABLE);
		if (aq_vlans[i].enable) {
			hw_atl2_rpf_vlan_id_flr_set(self,
						   aq_vlans[i].vlan_id,
						   i);
			hw_atl2_rpf_vlan_flr_act_set(self, 1U, i);
			hw_atl2_rpf_vlan_flr_en_set(self, 1U, i);

			if (aq_vlans[i].queue != 0xFF) {
				hw_atl2_rpf_vlan_rxq_flr_set(self,
							    aq_vlans[i].queue,
							    i);
				hw_atl2_rpf_vlan_rxq_en_flr_set(self, 1U, i);

				hw_atl2_rpf_vlan_flr_tag_set(self, i + 2, i);
				if( aq_vlans[i].queue != 0x7F ) {
				hw_atl2_act_rslvr_table_set(self,
					HW_ATL2_RPF_VLAN_USER_INDEX + i,
					(i + 2) << HW_ATL2_RPF_TAG_VLAN_OFFSET,
					HW_ATL2_RPF_TAG_VLAN_MASK,
					HW_ATL2_ACTION_ASSIGN_QUEUE(aq_vlans[i].queue));
				}
			} else {
				hw_atl2_rpf_vlan_flr_tag_set(self, 1, i);
			}
		}
	}

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_vlan_ctrl(struct aq_hw_s *self, bool enable)
{
	/* set promisc in case of disabing the vlan filter */
	hw_atl2_rpf_vlan_prom_mode_en_set(self, !enable);
	if (aq_new_filters_enabled)
		hw_atl2_hw_new_rx_filter_vlan_promisc(self, !enable);

	return aq_hw_err_from_flags(self);
}

static u64 hw_atl2_ptp_gpio_get_event(struct aq_hw_s *self, u32 channel, u32 *event_count)
{
	u64 event_ts;
	hw_atl2_tsg_gpio_clear_status(self, channel);
	hw_atl2_tsg_gpio_input_envent_info_get(self, channel, event_count, &event_ts);
	return aq_hw_err_from_flags(self) ? (u64)-1 : event_ts;
}

void hw_atl2_apply_link_speed(struct aq_hw_s *self, unsigned int mbps)
{
	unsigned int adj =  mbps == 10000 ? 4 : //TODO
						mbps == 5000  ? 4 : //TODO
						mbps == 2500  ? 0x34 : //TODO
						mbps == 1000  ? 0x24 : 
						mbps == 100   ? 4 + 4 : 4;
	hw_atl2_correct_launchtime(self, mbps, adj);
}

#ifdef TSN_SUPPORT
static int hw_atl2_tsn_enable(struct aq_hw_s *self, int enable)
{
	hw_atl2_tpb_tx_high_prio_avb_packet_length_comparison_enable_set(self, enable);
	hw_atl2_tpb_tx_enable_tx_packet_scheduler_avb_high_priority1_tc_set(self, enable);
	hw_atl2_tpb_tx_enable_tx_packet_scheduler_avb_high_priority0_tc_set(self, enable);
	return aq_hw_err_from_flags(self);
}
#endif

#ifdef PTM_SUPPORT
static int hw_atl2_ptm_enable(struct aq_hw_s *self, u32 timeout, uint32_t endianess_fix)
{
	//if( timeout > 0 ) {
	//	hw_atl2_pci_cfg_ptm_long_timer_set(self, timeout);
	//}
	hw_atl2_ptm_isr_to_host_set(self, timeout > 0);
	hw_atl2_pci_cfg_ptm_en(self, timeout > 0);
	if( timeout > 0 ) {
		hw_atl2_pci_cfg_ptm_enianess_fix(self, endianess_fix);
	}
	//hw_atl2_phi_ptm_en(self, timeout > 0);
	//hw_atl2_request_to_auto_update_clock_set(self, timeout > 0);
	hw_atl2_ptm_update_counter_value_set(self, (timeout*156250)); //ms => 6.4 ns
	hw_atl2_timer_enable_to_update_ptm_clock_set(self, timeout > 0);
	hw_atl2_manual_request_to_update_clock_set(self, timeout > 0);
	hw_atl2_ptm_clear_status(self);

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_ptm_get_info(struct aq_hw_s *self, struct ptm_data *info)
{
	if( !info )
		return -EINVAL;
	
	memset(info, 0, sizeof(struct ptm_data));
	if( !hw_atl2_ptm_context_valid_get(self) )
		return -ENXIO; //Invalid context

	hw_atl2_pci_cfg_ptm_req_t1p_get(self, &info->t1p);
	hw_atl2_pci_cfg_ptm_req_t1_get(self, &info->t1);
	hw_atl2_pci_cfg_ptm_req_t4p_get(self, &info->t4p);
	hw_atl2_pci_cfg_ptm_req_t4_get(self, &info->t4);
	hw_atl2_pci_cfg_ptm_req_pdelay_get(self, &info->t32);
	info->pdelay = (info->t4p - info->t1p) - info->t32;
	info->gen_id = hw_atl2_ptm_context_id_get(self);
	info->seq_id = hw_atl2_ptm_sequence_id_get(self);
	info->ptm_local_clock = hw_atl2_local_clock_value_get(self);
	info->correction = hw_atl2_local_clock_correction_value_get(self);
	info->free_run_clock = clk_select == ATL_TSG_CLOCK_SEL_1 ?
							hw_atl2_ptm_tsg_local_clock_get(self) : 
							hw_atl2_ptp_tsg_local_clock_get(self);
	hw_atl2_ptm_clear_status(self);
	return aq_hw_err_from_flags(self);
}
#endif
const struct aq_hw_ops hw_atl2_ops = {
	.hw_set_mac_address   = hw_atl2_hw_mac_addr_set,
	.hw_init              = hw_atl2_hw_init,
	.hw_reset             = hw_atl2_hw_reset,
	.hw_start             = hw_atl2_hw_start,
	.hw_ring_tx_start     = hw_atl2_hw_ring_tx_start,
	.hw_ring_tx_stop      = hw_atl2_hw_ring_tx_stop,
	.hw_ring_rx_start     = hw_atl2_hw_ring_rx_start,
	.hw_ring_rx_stop      = hw_atl2_hw_ring_rx_stop,
	.hw_stop              = hw_atl2_hw_stop,

	.hw_ring_tx_xmit         = hw_atl2_hw_ring_tx_xmit,
	.hw_ring_tx_head_update  = hw_atl2_hw_ring_tx_head_update,

	.hw_ring_rx_receive      = hw_atl2_hw_ring_rx_receive,
	.hw_ring_rx_fill         = hw_atl2_hw_ring_rx_fill,

	.hw_irq_enable           = hw_atl2_hw_irq_enable,
	.hw_irq_disable          = hw_atl2_hw_irq_disable,
	.hw_irq_read             = hw_atl2_hw_irq_read,

	.hw_ring_rx_init             = hw_atl2_hw_ring_rx_init,
	.hw_ring_tx_init             = hw_atl2_hw_ring_tx_init,
	.hw_packet_filter_set        = hw_atl2_hw_packet_filter_set,
	.hw_filter_l2_set            = hw_atl2_hw_fl2_set,
	.hw_filter_l2_clear          = hw_atl2_hw_fl2_clear,
	.hw_filter_l3_set            = hw_atl2_hw_fl3_set,
	.hw_filter_l3l4_set          = hw_atl2_hw_fl3l4_set,
	.hw_filter_l3l4_clear        = hw_atl2_hw_fl3l4_clear,
	.hw_filter_vlan_set          = hw_atl2_hw_vlan_set,
	.hw_filter_vlan_ctrl         = hw_atl2_hw_vlan_ctrl,
	.hw_multicast_list_set       = hw_atl2_hw_multicast_list_set,
	.hw_filter_chain_build       = hw_atl2_hw_filter_chain_build,
	.hw_filter_flex_set          = hw_atl2_hw_filter_flex_set,
	.hw_interrupt_moderation_set = hw_atl2_hw_interrupt_moderation_set,
	.hw_rss_set                  = hw_atl2_hw_rss_set,
	.hw_rss_hash_set             = hw_atl2_hw_rss_hash_set,
	.hw_get_regs                 = hw_atl2_utils_hw_get_regs,
	.hw_get_hw_stats             = hw_atl2_utils_get_hw_stats,
	.hw_get_fw_version           = hw_atl2_utils_get_fw_version,
	.hw_set_offload              = hw_atl2_hw_offload_set,
	.hw_set_loopback             = hw_atl2_set_loopback,
	.hw_set_fc                   = hw_atl2_set_fc,

	.hw_tx_tc_mode_get       = hw_atl2_tx_tc_mode_get,
	.hw_rx_tc_mode_get       = hw_atl2_rx_tc_mode_get,

	.hw_ring_hwts_rx_fill        = NULL,
	.hw_ring_hwts_rx_receive     = NULL,

	.hw_get_ptp_ts           = aq_get_ptp_ts,
	.hw_adj_clock_freq       = hw_atl2_adj_clock_freq,
	.hw_adj_sys_clock        = hw_atl2_adj_sys_clock,
	.hw_gpio_pulse           = hw_atl2_gpio_pulse,
	.hw_ext_interrupr_en     = hw_atl2_ptp_ext_interrupt_en,
	.hw_extts_gpio_enable    = hw_atl2_ptp_gpio_in_en,
	.hw_ptp_gpio_get_event   = hw_atl2_ptp_gpio_get_event,

	.enable_ptp              = hw_atl2_enable_ptp,
	.hw_ring_tx_ptp_get_ts   = hw_atl2_hw_ring_tx_ptp_get_ts,
	.rx_extract_ts           = hw_atl2_hw_rx_extract_ts,
	.hw_get_clk_sel          = hw_atl2_hw_get_clk_sel,
	.extract_hwts            = NULL,
	.apply_link_speed        = hw_atl2_apply_link_speed,
	.hw_get_chip_info        = hw_atl_utils_get_chip_info,

#ifdef PTM_SUPPORT
	.hw_ptm_enable           = hw_atl2_ptm_enable, //TODO FW call instead
	.hw_ptm_get_info         = hw_atl2_ptm_get_info,
#endif
#ifdef TSN_SUPPORT
	.hw_tsn_enable           = hw_atl2_tsn_enable,
#endif
};
