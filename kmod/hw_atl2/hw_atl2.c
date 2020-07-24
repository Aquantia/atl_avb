// SPDX-License-Identifier: GPL-2.0-only
/* Atlantic Network Driver
 * Copyright (C) 2020 Marvell International Ltd.
 */

/* File hw_atl2.c: Definition of Atlantic hardware specific functions. */

#include "aq_hw.h"
#include "aq_hw_utils.h"
#include "aq_ring.h"
#include "aq_nic.h"
#include "aq_filters.h"
#include "aq_ptp.h"
#include "aq_trace.h"
#include "hw_atl/hw_atl_b0.h"
#include "hw_atl/hw_atl_llh.h"
#include "hw_atl/hw_atl_llh_internal.h"
#include "hw_atl2.h"
#include "hw_atl2_utils.h"
#include "hw_atl2_llh.h"
#include "hw_atl2_internal.h"
#include "hw_atl2_llh_internal.h"

static int hw_atl2_act_rslvr_table_set(struct aq_hw_s *self, u8 location,
				       u32 tag, u32 mask, u32 action);

static void hw_atl2_enable_ptp(struct aq_hw_s *self,
			       unsigned int param, int enable);
static int hw_atl2_hw_tx_ptp_ring_init(struct aq_hw_s *self,
				     struct aq_ring_s *aq_ring);
static int hw_atl2_hw_rx_ptp_ring_init(struct aq_hw_s *self,
				     struct aq_ring_s *aq_ring);
static void aq_get_ptp_ts(struct aq_hw_s *self, u64 *stamp);
static int hw_atl2_adj_clock_freq(struct aq_hw_s *self, s32 ppb);
static int hw_atl2_hw_fl4_set(struct aq_hw_s *self,
				  struct aq_rx_filter_l3l4 *data);

#define DEFAULT_BOARD_BASIC_CAPABILITIES \
	.is_64_dma = true,		  \
	.op64bit = true,		  \
	.msix_irqs = 8U,		  \
	.irq_mask = ~0U,		  \
	.vecs = HW_ATL2_RSS_MAX,	  \
	.tcs_max = HW_ATL2_TC_MAX,	  \
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
			NETIF_F_TSO6 |    \
			NETIF_F_LRO |     \
			NETIF_F_NTUPLE |  \
			NETIF_F_HW_VLAN_CTAG_FILTER | \
			NETIF_F_HW_VLAN_CTAG_RX |     \
			NETIF_F_HW_VLAN_CTAG_TX |     \
			NETIF_F_GSO_UDP_L4      |     \
			NETIF_F_GSO_PARTIAL     |     \
			NETIF_F_HW_TC,                \
	.hw_priv_flags = IFF_UNICAST_FLT, \
	.flow_control = true,		  \
	.mtu = HW_ATL2_MTU_JUMBO,	  \
	.mac_regs_count = 72,		  \
	.hw_alive_check_addr = 0x1CU,     \
	.priv_data_len = sizeof(struct hw_atl2_priv)

const struct aq_hw_caps_s hw_atl2_caps_aqc113 = {
	DEFAULT_BOARD_BASIC_CAPABILITIES,
	.media_type = AQ_HW_MEDIA_TYPE_TP,
	.link_speed_msk = AQ_NIC_RATE_10G |
			  AQ_NIC_RATE_5G  |
			  AQ_NIC_RATE_2G5 |
			  AQ_NIC_RATE_1G  |
			  AQ_NIC_RATE_1G_HALF   |
			  AQ_NIC_RATE_100M      |
			  AQ_NIC_RATE_100M_HALF |
			  AQ_NIC_RATE_10M       |
			  AQ_NIC_RATE_10M_HALF,
};

static u32 hw_atl2_sem_act_rslvr_get(struct aq_hw_s *self)
{
	return hw_atl_reg_glb_cpu_sem_get(self, HW_ATL2_FW_SM_ACT_RSLVR);
}

static int hw_atl2_hw_reset(struct aq_hw_s *self)
{
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
	int err;
	int i;

	memset(priv, 0, sizeof(*priv));
	err = hw_atl2_utils_soft_reset(self);
	if (err)
		return err;

	for( i = 0; i < HW_ATL2_MAC_MAX - HW_ATL2_MAC_MIN; i++ ){
		priv->special_uc_tag[i] = HW_ATL2_RPF_TAG_BASE_UC;
	}

	if (self->clk_select != -1)
		hw_atl2_enable_ptp(self,
			self->clk_select,
			aq_utils_obj_test(&self->flags, AQ_HW_PTP_AVAILABLE) ?
				1 : 0);

	self->aq_fw_ops->set_state(self, MPI_RESET);

	err = aq_hw_err_from_flags(self);

	return err;
}

static int hw_atl2_tc_ptp_set(struct aq_hw_s *self, u32 tc, u32 q)
{
	/* Init TC2 for PTP_TX */
	hw_atl_tpb_tx_pkt_buff_size_per_tc_set(self, HW_ATL2_PTP_TXBUF_SIZE,
					       tc);

	/* Init TC2 for PTP_RX */
	hw_atl_rpb_rx_pkt_buff_size_per_tc_set(self, HW_ATL2_PTP_RXBUF_SIZE,
					       tc);

	/* No flow control for PTP */
	hw_atl_rpb_rx_xoff_en_per_tc_set(self, 0U, tc);

	hw_atl2_tpb_tps_highest_priority_tc_set(self, tc);

	/* Init Rx Q to TC mapping */
	hw_atl2_rx_q_map_to_tc(self, q, tc);
	hw_atl2_tpb_tx_flex_map_set(self, q, tc);
	return aq_hw_err_from_flags(self);
}

#ifdef TSN_SUPPORT
static int hw_atl2_tc_avb0_set(struct aq_hw_s *self, u32 tc, u32 q)
{
	/* Init TC for AVB0_TX */
	hw_atl2_tpb_tx_packet_scheduler_avb_high_priority0_tc_set(self, tc);
	hw_atl_tpb_tx_pkt_buff_size_per_tc_set(self, HW_ATL2_AVB0_TXBUF_SIZE, tc);
	hw_atl2_tpb_set_high_priority(self, tc);

	/* Init TC for AVB0_RX */
	hw_atl_rpb_rx_pkt_buff_size_per_tc_set(self, HW_ATL2_AVB0_RXBUF_SIZE, tc);

	/* Init Rx Q to TC mapping */
	hw_atl2_rx_q_map_to_tc(self, q, tc);
	hw_atl2_tpb_tx_flex_map_set(self, q, tc);
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_tc_avb1_set(struct aq_hw_s *self, u32 tc, u32 q)
{
	/* Init TC for AVB1_TX */
	hw_atl2_tpb_tx_packet_scheduler_avb_high_priority1_tc_set(self, tc);
	hw_atl_tpb_tx_pkt_buff_size_per_tc_set(self, HW_ATL2_AVB1_TXBUF_SIZE, tc);
	hw_atl2_tpb_set_high_priority(self, tc);

	/* Init TC for AVB1_RX */
	hw_atl_rpb_rx_pkt_buff_size_per_tc_set(self, q, tc);

	/* Init Rx Q to TC mapping */
	hw_atl2_rx_q_map_to_tc(self, q, tc);
	hw_atl2_tpb_tx_flex_map_set(self, q, tc);
	return aq_hw_err_from_flags(self);
}
#endif

int hw_atl2_hw_queue_to_tc_map_set(struct aq_hw_s *self)
{
	struct aq_nic_cfg_s *cfg = self->aq_nic_cfg;
	unsigned int tcs, q_per_tc;
	unsigned int tc, q;
	hw_atl2_tpb_tx_tc_q_rand_map_en_set(self, 1U);

	switch (cfg->tc_mode) {
	case AQ_TC_MODE_8TCS:
		tcs = 8;
		q_per_tc = 4;
		break;
	case AQ_TC_MODE_4TCS:
		tcs = 4;
		q_per_tc = 8;
		break;
	default:
		return -EINVAL;
	}

	for (tc = 0; tc != tcs; tc++) {
		unsigned int tc_q_offset = tc * q_per_tc;

		for (q = tc_q_offset; q != tc_q_offset + q_per_tc; q++) {
			hw_atl2_rx_q_map_to_tc(self, q, tc);
			hw_atl2_tpb_tx_flex_map_set(self, q, tc);
		}
	}

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_qos_set(struct aq_hw_s *self)
{
	struct aq_nic_cfg_s *cfg = self->aq_nic_cfg;
	u32 tx_buff_size = HW_ATL2_TXBUF_MAX;
	u32 rx_buff_size = HW_ATL2_RXBUF_MAX;
	unsigned int prio = 0U;
	u32 tc = 0U;

	if (cfg->is_ptp) {
		tx_buff_size -= HW_ATL2_PTP_TXBUF_SIZE;
		rx_buff_size -= HW_ATL2_PTP_RXBUF_SIZE;
	}

#ifdef TSN_SUPPORT
	tx_buff_size -= HW_ATL2_AVB0_TXBUF_SIZE + HW_ATL2_AVB1_TXBUF_SIZE;
	rx_buff_size -= HW_ATL2_AVB0_RXBUF_SIZE + HW_ATL2_AVB1_RXBUF_SIZE;
#endif

	/* TPS Descriptor rate init */
	hw_atl_tps_tx_pkt_shed_desc_rate_curr_time_res_set(self, 0x0U);
	hw_atl_tps_tx_pkt_shed_desc_rate_lim_set(self, 0xA);

	/* TPS VM init */
	hw_atl_tps_tx_pkt_shed_desc_vm_arb_mode_set(self, 0U);

	tx_buff_size /= cfg->tcs;
	rx_buff_size /= cfg->tcs;
	for (tc = 0; tc < cfg->tcs; tc++) {
		u32 threshold = 0U;

		/* Tx buf size TC0 */
		hw_atl_tpb_tx_pkt_buff_size_per_tc_set(self, tx_buff_size, tc);

		threshold = (tx_buff_size * (1024 / 32U) * 66U) / 100U;
		hw_atl_tpb_tx_buff_hi_threshold_per_tc_set(self, threshold, tc);

		threshold = (tx_buff_size * (1024 / 32U) * 50U) / 100U;
		hw_atl_tpb_tx_buff_lo_threshold_per_tc_set(self, threshold, tc);

		/* QoS Rx buf size per TC */
		hw_atl_rpb_rx_pkt_buff_size_per_tc_set(self, rx_buff_size, tc);

		threshold = (rx_buff_size * (1024U / 32U) * 66U) / 100U;
		hw_atl_rpb_rx_buff_hi_threshold_per_tc_set(self, threshold, tc);

		threshold = (rx_buff_size * (1024U / 32U) * 50U) / 100U;
		hw_atl_rpb_rx_buff_lo_threshold_per_tc_set(self, threshold, tc);

		hw_atl_b0_set_fc(self, self->aq_nic_cfg->fc.req, tc);
	}

	if (cfg->is_ptp)
		hw_atl2_tc_ptp_set(self, AQ_HW_PTP_TC, AQ_HW_PTP_4TC_RING_IDX);

#ifdef TSN_SUPPORT
	hw_atl2_tc_avb0_set(self, AQ_HW_AVB_CLASS_0_TC, HW_ATL2_AVB0_FTC_RING_IDX);
	hw_atl2_tc_avb1_set(self, AQ_HW_AVB_CLASS_1_TC, HW_ATL2_AVB1_FTC_RING_IDX);
#endif

	/* QoS 802.1p priority -> TC mapping */
	for (prio = 0; prio < 8; ++prio)
		hw_atl_rpf_rpb_user_priority_tc_map_set(self, prio,
							cfg->prio_tc_map[prio]);

	/* ATL2 Apply ring to TC mapping */
	hw_atl2_hw_queue_to_tc_map_set(self);

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_rss_set(struct aq_hw_s *self,
			      struct aq_rss_parameters *rss_params)
{
	u8 *indirection_table = rss_params->indirection_table;
	const u32 num_tcs = aq_hw_num_tcs(self);
	u32 rpf_redir2_enable;
	u32 queue;
	u32 tc;
	u32 i;

	rpf_redir2_enable = num_tcs > 4 ? 1 : 0;

	hw_atl2_rpf_redirection_table2_select_set(self, rpf_redir2_enable);

	for (i = HW_ATL2_RSS_REDIRECTION_MAX; i--;) {
		for (tc = 0; tc != num_tcs; tc++) {
			queue = tc * aq_hw_q_per_tc(self) +
				indirection_table[i];
			hw_atl2_new_rpf_rss_redir_set(self, tc, i, queue);
		}
	}

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_init_tx_tc_rate_limit(struct aq_hw_s *self)
{
	static const u32 max_weight = BIT(HW_ATL2_TPS_DATA_TCTWEIGHT_WIDTH) - 1;
	/* Scale factor is based on the number of bits in fractional portion */
	static const u32 scale = BIT(HW_ATL_TPS_DESC_RATE_Y_WIDTH);
	static const u32 frac_msk = HW_ATL_TPS_DESC_RATE_Y_MSK >>
				    HW_ATL_TPS_DESC_RATE_Y_SHIFT;
	const u32 link_speed = self->aq_link_status.mbps;
	struct aq_nic_cfg_s *nic_cfg = self->aq_nic_cfg;
	unsigned long num_min_rated_tcs = 0;
	u32 tc_weight[AQ_CFG_TCS_MAX];
	u32 fixed_max_credit_4b;
	u32 fixed_max_credit;
	u8 min_rate_msk = 0;
	u32 sum_weight = 0;
	int tc;

	/* By default max_credit is based upon MTU (in unit of 64b) */
	fixed_max_credit = nic_cfg->aq_hw_caps->mtu / 64;
	/* in unit of 4b */
	fixed_max_credit_4b = nic_cfg->aq_hw_caps->mtu / 4;

	if (link_speed) {
		min_rate_msk = nic_cfg->tc_min_rate_msk &
			       (BIT(nic_cfg->tcs) - 1);
		num_min_rated_tcs = hweight8(min_rate_msk);
	}

	/* First, calculate weights where min_rate is specified */
	if (num_min_rated_tcs) {
		for (tc = 0; tc != nic_cfg->tcs; tc++) {
			if (!nic_cfg->tc_min_rate[tc]) {
				tc_weight[tc] = 0;
				continue;
			}

			tc_weight[tc] = (-1L + link_speed +
					 nic_cfg->tc_min_rate[tc] *
					 max_weight) /
					link_speed;
			tc_weight[tc] = min(tc_weight[tc], max_weight);
			sum_weight += tc_weight[tc];
		}
	}

	/* WSP, if min_rate is set for at least one TC.
	 * RR otherwise.
	 */
	hw_atl2_tps_tx_pkt_shed_data_arb_mode_set(self, min_rate_msk ? 1U : 0U);
	/* Data TC Arbiter takes precedence over Descriptor TC Arbiter,
	 * leave Descriptor TC Arbiter as RR.
	 */
	hw_atl_tps_tx_pkt_shed_desc_tc_arb_mode_set(self, 0U);

	hw_atl_tps_tx_desc_rate_mode_set(self, nic_cfg->is_qos ? 1U : 0U);

	for (tc = 0; tc != nic_cfg->tcs; tc++) {
		const u32 en = (nic_cfg->tc_max_rate[tc] != 0) ? 1U : 0U;
		const u32 desc = AQ_NIC_CFG_TCVEC2RING(nic_cfg, tc, 0);
		u32 weight, max_credit;

		hw_atl_tps_tx_pkt_shed_desc_tc_max_credit_set(self, tc,
							      fixed_max_credit);
		hw_atl_tps_tx_pkt_shed_desc_tc_weight_set(self, tc, 0x1E);

		if (num_min_rated_tcs) {
			weight = tc_weight[tc];

			if (!weight && sum_weight < max_weight)
				weight = (max_weight - sum_weight) /
					 (nic_cfg->tcs - num_min_rated_tcs);
			else if (!weight)
				weight = 0x640;

			max_credit = max(2 * weight, fixed_max_credit_4b);
		} else {
			weight = 0x640;
			max_credit = 0xFFF0;
		}

		hw_atl2_tps_tx_pkt_shed_tc_data_weight_set(self, tc, weight);
		hw_atl2_tps_tx_pkt_shed_tc_data_max_credit_set(self, tc,
							       max_credit);

		hw_atl_tps_tx_desc_rate_en_set(self, desc, en);

		if (en) {
			/* Nominal rate is always 10G */
			const u32 rate = 10000U * scale /
					 nic_cfg->tc_max_rate[tc];
			const u32 rate_int = rate >>
					     HW_ATL_TPS_DESC_RATE_Y_WIDTH;
			const u32 rate_frac = rate & frac_msk;

			hw_atl_tps_tx_desc_rate_x_set(self, desc, rate_int);
			hw_atl_tps_tx_desc_rate_y_set(self, desc, rate_frac);
		} else {
			/* A value of 1 indicates the queue is not
			 * rate controlled.
			 */
			hw_atl_tps_tx_desc_rate_x_set(self, desc, 1U);
			hw_atl_tps_tx_desc_rate_y_set(self, desc, 0U);
		}
	}
	for (tc = nic_cfg->tcs; tc != AQ_CFG_TCS_MAX; tc++) {
		const u32 desc = AQ_NIC_CFG_TCVEC2RING(nic_cfg, tc, 0);

		hw_atl_tps_tx_desc_rate_en_set(self, desc, 0U);
		hw_atl_tps_tx_desc_rate_x_set(self, desc, 1U);
		hw_atl_tps_tx_desc_rate_y_set(self, desc, 0U);
	}

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_init_tx_path(struct aq_hw_s *self)
{
	struct aq_nic_cfg_s *nic_cfg = self->aq_nic_cfg;

	/* Tx TC/RSS number config */
	hw_atl_tpb_tps_tx_tc_mode_set(self, nic_cfg->tc_mode);

	hw_atl_thm_lso_tcp_flag_of_first_pkt_set(self, 0x0FF6U);
	hw_atl_thm_lso_tcp_flag_of_middle_pkt_set(self, 0x0FF6U);
	hw_atl_thm_lso_tcp_flag_of_last_pkt_set(self, 0x0F7FU);

	/* Tx interrupts */
	hw_atl_tdm_tx_desc_wr_wb_irq_en_set(self, 1U);

	/* misc */
	hw_atl_tdm_tx_dca_en_set(self, 0U);
	hw_atl_tdm_tx_dca_mode_set(self, 0U);

	hw_atl_tpb_tx_path_scp_ins_en_set(self, 1U);

	hw_atl2_tpb_tx_buf_clk_gate_en_set(self, 0U);

	if (hw_atl2_phi_ext_tag_get(self)) {
		hw_atl2_tdm_tx_data_read_req_limit_set(self, 0x7F);
		hw_atl2_tdm_tx_desc_read_req_limit_set(self, 0x0F);
	}

	return aq_hw_err_from_flags(self);
}

/** Initialise new rx filters
 * L2 promisc OFF
 * VLAN promisc OFF
 *
 * User prioryty to TC
 */
static void hw_atl2_hw_init_new_rx_filters(struct aq_hw_s *self)
{
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
	u8 *prio_tc_map = self->aq_nic_cfg->prio_tc_map;
	u32 art_first_sec, art_last_sec;
	u32 art_mask = 0;
	u16 action;
	u8 index;
	int i;

	/* Action Resolver Table (ART) is used by RPF to decide which action
	 * to take with a packet based upon input tag and tag mask, where:
	 *  - input tag is a combination of 3-bit VLan Prio (PTP) and
	 *    29-bit concatenation of all tags from filter block;
	 *  - tag mask is a mask used for matching against input tag.
	 * The input_tag is compared with the all the Requested_tags in the
	 * Record table to find a match. Action field of the selected matched
	 * REC entry is used for further processing. If multiple entries match,
	 * the lowest REC entry, Action field will be selected.
	 */
	art_last_sec = priv->art_base_index / 8 + priv->art_count / 8;
	art_first_sec = priv->art_base_index / 8;
	art_mask = (BIT(art_last_sec) - 1) - (BIT(art_first_sec) - 1);
	hw_atl2_rpf_act_rslvr_section_en_set(self,
			hw_atl2_rpf_act_rslvr_section_en_get(self) | art_mask);
	hw_atl2_rpf_l3_v6_v4_select_set(self, 1);
	hw_atl2_rpfl2_uc_flr_tag_set(self, HW_ATL2_RPF_TAG_BASE_UC,
				     priv->l2_filters_base_index);
	hw_atl2_rpfl2_bc_flr_tag_set(self, HW_ATL2_RPF_TAG_BC);

	/* FW reserves the beginning of ART, thus all driver entries must
	 * start from the offset specified in FW caps.
	 */
	index = priv->art_base_index + HW_ATL2_RPF_L2_PROMISC_OFF_INDEX;
	hw_atl2_act_rslvr_table_set(self, index, 0,
				    HW_ATL2_RPF_TAG_UC_MASK |
				    HW_ATL2_RPF_TAG_ALLMC_MASK,
				    HW_ATL2_ACTION_DROP);

	index = priv->art_base_index + HW_ATL2_RPF_VLAN_PROMISC_OFF_INDEX;
	hw_atl2_act_rslvr_table_set(self, index, 0,
				    HW_ATL2_RPF_TAG_VLAN_MASK |
				    HW_ATL2_RPF_TAG_UNTAG_MASK,
				    HW_ATL2_ACTION_DROP);

	/* Configure ART to map given VLan Prio (PCP) to the TC index for
	 * RSS redirection table.
	 */
	for (i = 0; i < AQ_CFG_TCS_MAX; i++) {
		action = HW_ATL2_ACTION_ASSIGN_TC(prio_tc_map[i]);

		index = priv->art_base_index + HW_ATL2_RPF_PCP_TO_TC_INDEX + i;
		hw_atl2_act_rslvr_table_set(self, index,
					    i << HW_ATL2_RPF_TAG_PCP_OFFSET,
					    HW_ATL2_RPF_TAG_PCP_MASK, action);
	}
}

static void hw_atl2_hw_new_rx_filter_vlan_promisc(struct aq_hw_s *self,
						  bool promisc)
{
	u16 off_action = (!promisc &&
			  !hw_atl_rpfl2promiscuous_mode_en_get(self)) ?
				HW_ATL2_ACTION_DROP : HW_ATL2_ACTION_DISABLE;
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
	u8 index;

	index = priv->art_base_index + HW_ATL2_RPF_VLAN_PROMISC_OFF_INDEX;
	hw_atl2_act_rslvr_table_set(self, index, 0,
				    HW_ATL2_RPF_TAG_VLAN_MASK |
				    HW_ATL2_RPF_TAG_UNTAG_MASK, off_action);
}

static void hw_atl2_hw_new_rx_filter_promisc(struct aq_hw_s *self, bool promisc)
{
	u16 off_action = promisc ? HW_ATL2_ACTION_DISABLE : HW_ATL2_ACTION_DROP;
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
	bool vlan_promisc_enable;
	u8 index;

	index = priv->art_base_index + HW_ATL2_RPF_L2_PROMISC_OFF_INDEX;
	hw_atl2_act_rslvr_table_set(self, index, 0,
				    HW_ATL2_RPF_TAG_UC_MASK |
				    HW_ATL2_RPF_TAG_ALLMC_MASK,
				    off_action);

	/* turn VLAN promisc mode too */
	vlan_promisc_enable = hw_atl_rpf_vlan_prom_mode_en_get(self);
	hw_atl2_hw_new_rx_filter_vlan_promisc(self, promisc |
					      vlan_promisc_enable);
}

static int hw_atl2_act_rslvr_table_set(struct aq_hw_s *self, u8 location,
				       u32 tag, u32 mask, u32 action)
{
	u32 val;
	int err;
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
	/*printk("ART: location %03d [%03d:%03d], tag %08x/%08x, action %x.\n", location,
		priv->art_base_index, priv->art_base_index + priv->art_count, tag, mask, action);*/
	if( location < priv->art_base_index ||
		location >= priv->art_base_index + priv->art_count )
		return -EINVAL;

	err = readx_poll_timeout_atomic(hw_atl2_sem_act_rslvr_get,
					self, val, val == 1,
					1, 10000U);
	if (err)
		return err;

	hw_atl2_rpf_act_rslvr_record_set(self, location, tag, mask,
					 action);

	hw_atl_reg_glb_cpu_sem_set(self, 1, HW_ATL2_FW_SM_ACT_RSLVR);

	return err;
}

static int hw_atl2_hw_init_rx_path(struct aq_hw_s *self)
{
	struct aq_nic_cfg_s *cfg = self->aq_nic_cfg;
	u32 control_reg_val;

	/* Rx TC/RSS number config */
	hw_atl_rpb_rpf_rx_traf_class_mode_set(self, cfg->tc_mode);

	/* Rx flow control */
	hw_atl_rpb_rx_flow_ctl_mode_set(self, 1U);

	hw_atl2_rpf_rss_hash_type_set(self, HW_ATL2_RPF_RSS_HASH_TYPE_ALL);

	/* RSS Ring selection */
	hw_atl_b0_hw_init_rx_rss_ctrl1(self);

	/* Multicast filters */
	hw_atl_reg_rx_flr_mcst_flr_msk_set(self, 0x00000000U);
	hw_atl_reg_rx_flr_mcst_flr_set(self, HW_ATL_MCAST_FLT_ANY_TO_HOST, 0U);

	/* Vlan filters */
	hw_atl_rpf_vlan_outer_etht_set(self, ETH_P_8021AD);
	hw_atl_rpf_vlan_inner_etht_set(self, ETH_P_8021Q);

	hw_atl_rpf_vlan_prom_mode_en_set(self, 1);

	/* Always accept untagged packets */
	hw_atl_rpf_vlan_accept_untagged_packets_set(self, 1U);
	hw_atl_rpf_vlan_untagged_act_set(self, 1U);

	hw_atl2_hw_init_new_rx_filters(self);

	/* Rx Interrupts */
	hw_atl_rdm_rx_desc_wr_wb_irq_en_set(self, 1U);

	control_reg_val =
			   (3 << 17) | /* Filter logic version 3 */
			   BIT(19); /* RPF lpbk and mac counters by EOP */

	aq_hw_write_reg(self, 0x00005040U, control_reg_val);	
	hw_atl_rpfl2broadcast_flr_act_set(self,
				hw_atl_rpfl2broadcast_flr_act_get(self) | 1);
	hw_atl_rpfl2broadcast_count_threshold_set(self, 0xFFFFU & (~0U / 256U));

	hw_atl_rdm_rx_dca_en_set(self, 0U);
	hw_atl_rdm_rx_dca_mode_set(self, 0U);

	/* RDM extended tag */
	if (hw_atl2_phi_ext_tag_get(self))
		aq_hw_write_reg(self, HW_ATL2_RDM_RX_DESC_RD_REQ_LIMIT_ADR,
				0x10);

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_mac_addr_set(struct aq_hw_s *self, u8 *mac_addr)
{

	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
	u32 location = priv->l2_filters_base_index;
	unsigned int h = 0U;
	unsigned int l = 0U;
	int err = 0;

	if (!mac_addr) {
		err = -EINVAL;
		goto err_exit;
	}
	h = (mac_addr[0] << 8) | (mac_addr[1]);
	l = (mac_addr[2] << 24) | (mac_addr[3] << 16) |
		(mac_addr[4] << 8) | mac_addr[5];

	hw_atl_rpfl2_uc_flr_en_set(self, 0U, location);
	hw_atl_rpfl2unicast_dest_addresslsw_set(self, l, location);
	hw_atl_rpfl2unicast_dest_addressmsw_set(self, h, location);
	hw_atl_rpfl2unicast_flr_act_set(self, 1U, location);
	hw_atl2_rpfl2_uc_flr_tag_set(self, HW_ATL2_RPF_TAG_BASE_UC, location);
	hw_atl_rpfl2_uc_flr_en_set(self, 1U, location);

	err = aq_hw_err_from_flags(self);

err_exit:
	return err;
}

static int hw_atl2_hw_init(struct aq_hw_s *self, u8 *mac_addr)
{
	static u32 aq_hw_atl2_igcr_table_[4][2] = {
		[AQ_HW_IRQ_INVALID] = { 0x20000000U, 0x20000000U },
		[AQ_HW_IRQ_LEGACY]  = { 0x20000080U, 0x20000080U },
		[AQ_HW_IRQ_MSI]     = { 0x20000021U, 0x20000025U },
		[AQ_HW_IRQ_MSIX]    = { 0x20000022U, 0x20000026U },
	};
	struct aq_nic_cfg_s *aq_nic_cfg = self->aq_nic_cfg;
	u32 val;
	int err;

	// TODO: PCI perf optimization tryouts:

	// 8 queues mode
	//aq_hw_write_reg(self, 0xC9C, 3);

	/* PCIe extended bit support */
	if (aq_enable_wa & AQ_WA_FORCE_PCIE_EXTTAG) {
		hw_atl2_phi_ext_tag_set(self, 1);
		aq_pr_err("Enabled: AQ_WA_FORCE_PCIE_EXTTAG 0x%x\n",
			  (int)AQ_WA_FORCE_PCIE_EXTTAG);
	}

	hw_atl2_init_launchtime(self);

	hw_atl2_hw_init_tx_path(self);
	hw_atl2_hw_init_rx_path(self);

	hw_atl2_hw_mac_addr_set(self, mac_addr);

	self->aq_fw_ops->set_link_speed(self, aq_nic_cfg->link_speed_msk);
	self->aq_fw_ops->set_state(self, MPI_INIT);

	hw_atl2_hw_qos_set(self);
	hw_atl2_hw_rss_set(self, &aq_nic_cfg->aq_rss);
	hw_atl_b0_hw_rss_hash_set(self, &aq_nic_cfg->aq_rss);

	if (aq_enable_wa & AQ_WA_MRRS_LIMIT) {
		/* Force limit MRRS on RDM/TDM to 2K */
		val = aq_hw_read_reg(self, HW_ATL_PCI_REG_CONTROL6_ADR);
		aq_hw_write_reg(self, HW_ATL_PCI_REG_CONTROL6_ADR,
				(val & ~0x707) | 0x404);
	}

	if (aq_enable_wa & AQ_WA_DMA_REQ_LIMIT) {
		/* TX DMA total request limit. A2 hardware is not capable to
		 * handle more than (8K-MRRS) incoming DMA data.
		 * Value 24 in 256byte units
		 */
		aq_hw_write_reg(self, HW_ATL_TX_DMA_TOTAL_REQ_LIMIT_ADR, 24);
	}

	/* Continue service with the subsequent packet, if TX DMA could
	 * accommodate it within the target byte count set.
	 */
	if (!(aq_enable_wa & AQ_WA_EARLY_PKT_BRK))
		hw_atl_tdm_tx_early_pkt_brk_set(self, 0);

	hw_atl2_rpf_new_enable_set(self, 1);

	/* Reset link status and read out initial hardware counters */
	self->aq_link_status.mbps = 0;
	self->aq_fw_ops->update_stats(self);

	err = aq_hw_err_from_flags(self);
	if (err < 0)
		goto err_exit;

	/* Interrupts */
	hw_atl_reg_irq_glb_ctl_set(self,
		aq_hw_atl2_igcr_table_[aq_nic_cfg->irq_type]
			[(aq_nic_cfg->num_irq_vecs > 0) ?
				1 : 0]);

	hw_atl_itr_irq_auto_masklsw_set(self, aq_nic_cfg->aq_hw_caps->irq_mask);

	/* Interrupts */
	hw_atl_reg_gen_irq_map_set(self,
				   ((HW_ATL2_ERR_INT << 0x18) |
				    (1U << 0x1F)) |
				   ((HW_ATL2_ERR_INT << 0x10) |
				    (1U << 0x17)), 0U);

	hw_atl_b0_hw_offload_set(self, aq_nic_cfg);

err_exit:
	return err;
}

static int hw_atl2_hw_ring_rx_init(struct aq_hw_s *self,
				   struct aq_ring_s *aq_ring,
				   struct aq_ring_param_s *aq_ring_param)
{
	int res = hw_atl_b0_hw_ring_rx_init(self, aq_ring, aq_ring_param);

	if (aq_ptp_ring(aq_ring))
		hw_atl2_hw_rx_ptp_ring_init(self, aq_ring);

	return res;
}

static int hw_atl2_hw_ring_tx_init(struct aq_hw_s *self,
				   struct aq_ring_s *aq_ring,
				   struct aq_ring_param_s *aq_ring_param)
{
	int res = hw_atl_b0_hw_ring_tx_init(self, aq_ring, aq_ring_param);

	if (aq_ptp_ring(aq_ring))
		hw_atl2_hw_tx_ptp_ring_init(self, aq_ring);

	return res;
}

#define IS_FILTER_ENABLED(_F_) ((packet_filter & (_F_)) ? 1U : 0U)

static int hw_atl2_hw_packet_filter_set(struct aq_hw_s *self,
					unsigned int packet_filter)
{
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
	struct aq_nic_cfg_s *cfg = self->aq_nic_cfg;
	unsigned int i, location;
	u32 vlan_promisc;
	u32 l2_promisc;

	l2_promisc = IS_FILTER_ENABLED(IFF_PROMISC) ||
		     !!(cfg->priv_flags & BIT(AQ_HW_LOOPBACK_DMA_NET));
	vlan_promisc = l2_promisc || cfg->is_vlan_force_promisc;

	hw_atl_rpfl2promiscuous_mode_en_set(self, l2_promisc);

	hw_atl_rpf_vlan_prom_mode_en_set(self, vlan_promisc);

	hw_atl_rpfl2broadcast_en_set(self, IS_FILTER_ENABLED(IFF_BROADCAST));

	hw_atl_rpfl2multicast_flr_en_set(self,
					 IS_FILTER_ENABLED(IFF_ALLMULTI) &&
					 IS_FILTER_ENABLED(IFF_MULTICAST), 0);

	hw_atl_rpfl2_accept_all_mc_packets_set(self,
					      IS_FILTER_ENABLED(IFF_ALLMULTI) &&
					      IS_FILTER_ENABLED(IFF_MULTICAST));

	hw_atl2_hw_new_rx_filter_promisc(self, IS_FILTER_ENABLED(IFF_PROMISC));
	hw_atl2_utils_set_filter_policy(self, IS_FILTER_ENABLED(IFF_PROMISC),
					IS_FILTER_ENABLED(IFF_ALLMULTI) &&
					IS_FILTER_ENABLED(IFF_MULTICAST));

	for (i = 0, location = priv->l2_filters_base_index + 1;
	     location < priv->l2_filters_base_index + priv->l2_filter_count;
	     location++, i++)
		hw_atl_rpfl2_uc_flr_en_set(self,
					   (cfg->is_mc_list_enabled &&
					    (i < cfg->mc_list_count)) ?
					    1U : 0U, location);

	return aq_hw_err_from_flags(self);
}

#undef IS_FILTER_ENABLED

static int hw_atl2_hw_multicast_list_set(struct aq_hw_s *self,
					 u8 ar_mac
					 [AQ_HW_MULTICAST_ADDRESS_MAX]
					 [ETH_ALEN],
					 u32 count)
{
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
	u32 base_location = priv->l2_filters_base_index + 1;
	struct aq_nic_cfg_s *cfg = self->aq_nic_cfg;
	int err = 0;

	if (count > (HW_ATL2_MAC_MAX - HW_ATL2_MAC_MIN)) {
		err = -EBADRQC;
		goto err_exit;
	}
	for (cfg->mc_list_count = 0U;
			cfg->mc_list_count < count;
			++cfg->mc_list_count) {
		u32 i = cfg->mc_list_count;
		u32 h = (ar_mac[i][0] << 8) | (ar_mac[i][1]);
		u32 l = (ar_mac[i][2] << 24) | (ar_mac[i][3] << 16) |
					(ar_mac[i][4] << 8) | ar_mac[i][5];

		hw_atl_rpfl2_uc_flr_en_set(self, 0U, base_location + i);

		hw_atl_rpfl2unicast_dest_addresslsw_set(self, l,
							base_location + i);

		hw_atl_rpfl2unicast_dest_addressmsw_set(self, h,
							base_location + i);
		hw_atl_rpfl2unicast_flr_act_set(self, 1U, base_location + i);

		hw_atl2_rpfl2_uc_flr_tag_set(self, priv->special_uc_tag[i],
					     base_location + i);

		hw_atl_rpfl2_uc_flr_en_set(self,
					   (cfg->is_mc_list_enabled),
					   base_location + i);
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
		hw_atl_tdm_tx_desc_wr_wb_irq_en_set(self, 0U);
		hw_atl_tdm_tdm_intr_moder_en_set(self, 1U);
		hw_atl_rdm_rx_desc_wr_wb_irq_en_set(self, 0U);
		hw_atl_rdm_rdm_intr_moder_en_set(self, 1U);

		if (self->aq_nic_cfg->itr == AQ_CFG_INTERRUPT_MODERATION_ON) {
			/* HW timers are in 2us units */
			int tx_max_timer = self->aq_nic_cfg->tx_itr / 2;
			int tx_min_timer = tx_max_timer / 2;

			int rx_max_timer = self->aq_nic_cfg->rx_itr / 2;
			int rx_min_timer = rx_max_timer / 2;

			tx_max_timer = min(HW_ATL2_INTR_MODER_MAX,
					   tx_max_timer);
			tx_min_timer = min(HW_ATL2_INTR_MODER_MIN,
					   tx_min_timer);
			rx_max_timer = min(HW_ATL2_INTR_MODER_MAX,
					   rx_max_timer);
			rx_min_timer = min(HW_ATL2_INTR_MODER_MIN,
					   rx_min_timer);

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
			unsigned int mbps = self->aq_link_status.mbps;
			unsigned int speed_index;

			speed_index = hw_atl_utils_mbps_2_speed_index(mbps);

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
		hw_atl_tdm_tx_desc_wr_wb_irq_en_set(self, 1U);
		hw_atl_tdm_tdm_intr_moder_en_set(self, 0U);
		hw_atl_rdm_rx_desc_wr_wb_irq_en_set(self, 1U);
		hw_atl_rdm_rdm_intr_moder_en_set(self, 0U);
		itr_tx = 0U;
		itr_rx = 0U;
		break;
	}

	for (i = HW_ATL2_RINGS_MAX; i--;) {
		hw_atl2_reg_tx_intr_moder_ctrl_set(self, itr_tx, i);
		hw_atl_reg_rx_intr_moder_ctrl_set(self, itr_rx, i);
	}

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_stop(struct aq_hw_s *self)
{
	int err;
	u32 val;

	hw_atl_b0_hw_irq_disable(self, HW_ATL2_INT_MASK);

	if (aq_enable_wa & AQ_WA_RDM_CACHE_CLEAR) {
		/* Invalidate Descriptor Cache to prevent writing to the cached
		 * descriptors and to the data pointer of those descriptors
		 */
		hw_atl_rdm_rx_dma_desc_cache_init_tgl(self);
		err = readx_poll_timeout_atomic(
				hw_atl_rdm_rx_dma_desc_cache_init_done_get,
				self, val, val == 1, 1000U, 10000U);
		WARN_ON(err);
	}
	return 0;
}

static struct aq_stats_s *hw_atl2_utils_get_hw_stats(struct aq_hw_s *self)
{
	return &self->curr_stats;
}

static u32 hw_atl2_tsg_int_clk_freq(struct aq_hw_s *self)
{
	return AQ2_HW_PTP_COUNTER_HZ;
}

static u32 hw_atl2_312p5_clk_freq(struct aq_hw_s *self)
{
	if (ATL_HW_IS_CHIP_FEATURE(self, FPGA))
		return AQ2_HW_312P5_COUNTER_HZ / 4;

	return AQ2_HW_312P5_COUNTER_HZ;
}

static void hw_atl2_enable_ptp(struct aq_hw_s *self,
			       unsigned int param, int enable)
{
	self->clk_select = param;

	/* enable tsg counter */
	hw_atl2_tsg_clock_reset(self, self->clk_select);
	hw_atl2_tsg_clock_en(self, !self->clk_select, enable);
	hw_atl2_tsg_clock_en(self, self->clk_select, enable);

	if (enable)
		hw_atl2_adj_clock_freq(self, 0);

	hw_atl2_tpb_tps_highest_priority_tc_enable_set(self, enable);

	if (ATL_HW_IS_CHIP_FEATURE(self, REVISION_B0))
		hw_atl2_prim_ts_clk_sel_set(self, self->clk_select);
}

static void aq_get_ptp_ts(struct aq_hw_s *self, u64 *stamp)
{
	if (stamp)
		*stamp = hw_atl2_tsg_clock_read(self, self->clk_select);
	//aq_pr_trace("Sys TS: %llu\n", *stamp);
}

static u64 hw_atl2_hw_ring_tx_ptp_get_ts(struct aq_ring_s *ring)
{
	struct hw_atl2_txts_s *txts;
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)ring->aq_nic->aq_hw->priv;
	bool ptp_ts_maybe_missed = priv->ptp_check_cntr == 1; //W/A for ANTIGUAA0-366
	
	if (priv->ptp_check_cntr > 0)priv->ptp_check_cntr--;
	txts = (struct hw_atl2_txts_s *)&ring->dx_ring[ring->sw_head *
						HW_ATL2_TXD_SIZE];
	//aq_pr_trace("Egress TS: %llu, valid %lu, dd %lu, ts read attempt %d\n", 
	//	txts->ts, txts->ctrl & BIT(3), txts->ctrl & BIT(20), priv->ptp_check_cntr);
	/* DD + TS_VALID */
	if (txts->ctrl & BIT(3) && txts->ctrl & BIT(20) ) {
		priv->ptp_check_cntr = 0;
		return txts->ts;
	}

	if (ptp_ts_maybe_missed) //W/A for ANTIGUAA0-366
		return 1LLU;
	return 0;
}

static u16 hw_atl2_hw_rx_extract_ts(struct aq_hw_s *self, u8 *p,
				    unsigned int len, u64 *timestamp)
{
	unsigned int offset = HW_ATL2_RX_TS_SIZE;
	//u8 pkt_type = p[14];
	u8 *ptr;
	//u64 ns;

	if (len <= offset || !timestamp)
		return 0;

	ptr = p + (len - offset);
	memcpy(timestamp, ptr, sizeof(*timestamp));
	//if( (pkt_type & 0xf)  == 0xb )
	//        aq_pr_trace("PTP Msg %x, seq %03d, Ingress TS: %llu\n", pkt_type, (p[44]<<8) + p[45], *timestamp);
	//*timestamp = be64_to_cpu(ns);
	//aq_pr_trace("Ingress TS: %llu\n", *timestamp);
	return HW_ATL2_RX_TS_SIZE;
}

static int hw_atl2_adj_sys_clock(struct aq_hw_s *self, s64 delta)
{
	if (delta >= 0)
		hw_atl2_tsg_clock_add(self, self->clk_select, (u64)delta, 0);
	else
		hw_atl2_tsg_clock_sub(self, self->clk_select, (u64)(-delta), 0);

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_adj_msm_freq(struct aq_hw_s *self, s32 ppb)
{
	u64 divisor = 0, base_ns;
	u32 nsi_frac = 0, nsi;
	u32 nsi_rem;
	u32 adj = ppb;
	u32 freq = hw_atl2_312p5_clk_freq(self);

	base_ns = div_u64(mul_u32_u32(adj + NSEC_PER_SEC, NSEC_PER_SEC), freq);
	nsi = (u32)div_u64_rem(base_ns, NSEC_PER_SEC, &nsi_rem);
	if (nsi_rem != 0) {
		divisor = div_u64(mul_u32_u32(NSEC_PER_SEC, NSEC_PER_SEC),
				nsi_rem);
		nsi_frac = (u32)div64_u64(AQ_FRAC_PER_NS * NSEC_PER_SEC,
					divisor);
	}

	//printk("Detected freq %d, nsi %x, nsi_frac %x\n", freq, nsi, nsi_frac);
	hw_atl2_fifo312p5_fns_inc_val_set(self, nsi_frac);
	hw_atl2_fifo312p5_corr_period_set(self, 0);
	hw_atl2_fifo312p5_ns_inc_set(self, nsi);
	hw_atl2_fifo312p5_fns_corr_set(self, 0);
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_adj_clock_freq(struct aq_hw_s *self, s32 ppb)
{
	u32 freq = hw_atl2_tsg_int_clk_freq(self);
	u64 divisor = 0, base_ns;
	u32 nsi_frac = 0, nsi;
	u32 adj = ppb;
	u32 nsi_rem;

	base_ns = div_u64(mul_u32_u32(adj + NSEC_PER_SEC, NSEC_PER_SEC), freq);
	nsi = (u32)div_u64_rem(base_ns, NSEC_PER_SEC, &nsi_rem);
	if (nsi_rem != 0) {
		divisor = div_u64(mul_u32_u32(NSEC_PER_SEC, NSEC_PER_SEC),
				  nsi_rem);
		nsi_frac = (u32)div64_u64(AQ_FRAC_PER_NS * NSEC_PER_SEC,
					  divisor);
	}

	hw_atl2_tsg_clock_increment_set(self, self->clk_select, nsi, nsi_frac);

	if (ATL_HW_IS_CHIP_FEATURE(self, REVISION_B0)) {
		struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
		priv->ppb = ppb;
		hw_atl2_adj_msm_freq(self, ppb);
	}
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_tx_ptp_ring_init(struct aq_hw_s *self,
				       struct aq_ring_s *aq_ring)
{
	hw_atl2_tdm_tx_desc_timestamp_writeback_en_set(self, true,
						       aq_ring->idx);
	hw_atl2_tdm_tx_desc_timestamp_en_set(self, true, aq_ring->idx);
	hw_atl2_tdm_tx_desc_avb_en_set(self, true, aq_ring->idx);

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_rx_ptp_ring_init(struct aq_hw_s *self,
				       struct aq_ring_s *aq_ring)
{
	hw_atl2_rpf_rx_desc_timestamp_req_set(self,
			self->clk_select == ATL_TSG_CLOCK_SEL_1 ? 2 : 1,
			aq_ring->idx);
	return aq_hw_err_from_flags(self);
}

static u32 hw_atl2_hw_get_clk_sel(struct aq_hw_s *self)
{
	return self->clk_select;
}

static int hw_atl2_ptp_ext_interrupt_en(struct aq_hw_s *self,
					int on, u32 flags)
{
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
	struct aq_nic_cfg_s *aq_nic_cfg = self->aq_nic_cfg;

	if( (!priv->ext_int_flags && on) ||
		(!(priv->ext_int_flags ^ flags) && !on) ) {
		hw_atl2_tsg_ext_isr_to_host_set(self, on);
		hw_atl_reg_gen_irq_map_set(self,
			(BIT(0x7) |
			aq_nic_cfg->link_irq_vec) |
			(on ? BIT(0xF) |
			((aq_nic_cfg->link_irq_vec + 2) << 0x8) : 0),
			3U); //MIF2
	}

	if( on ) {
		priv->ext_int_flags |= flags;
	} else {
		priv->ext_int_flags &= ~flags;
	}
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_ptp_gpio_in_en(struct aq_hw_s *self, u32 index,
				  u32 channel, int on)
{
	hw_atl2_tsg_gpio_isr_to_host_set(self, on, channel);
	hw_atl2_tsg_gpio_clear_status(self, channel);
	hw_atl2_tsg_gpio_input_set(self, on, index, channel);

	if (ATL_HW_IS_CHIP_FEATURE(self, FPGA)) {
		hw_atl2_fpga_tsg_gpio_input_set(self, channel);
	} else {
		hw_atl2_tsg0_ext_gpio_ts_input_select_set(self,
			(channel == ATL_TSG_CLOCK_SEL_0) && on ? index : 0);
		hw_atl2_tsg1_ext_gpio_ts_input_select_set(self,
			(channel == ATL_TSG_CLOCK_SEL_1) && on ? index : 0);
	}

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_gpio_pulse(struct aq_hw_s *self, u32 index, u32 clk_sel,
			      u64 start, u32 period, u32 hightime)
{
	if (ATL_HW_IS_CHIP_FEATURE(self, FPGA)) {
		hw_atl2_fpga_tsg_ptp_gpio_gen_pulse(self, clk_sel, start > 0);
	} else if (index == 1 || index == 3) { //Hardware limitation
		hw_atl2_gpio_special_mode_set(self,
			start == 0 ? HW_ATL2_GPIO_PIN_SPEC_MODE_GPIO :
				(clk_sel == ATL_TSG_CLOCK_SEL_0 ?
				HW_ATL2_GPIO_PIN_SPEC_MODE_TSG0_EVENT_OUTPUT :
				HW_ATL2_GPIO_PIN_SPEC_MODE_TSG1_EVENT_OUTPUT),
			index);
	}

	hw_atl2_tsg_ptp_gpio_gen_pulse(self, clk_sel, start, period, hightime);
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_filter_chain_build(struct aq_hw_s *self, struct aq_rx_filter *data, bool add)
{
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
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
		priv->art_base_index + HW_ATL2_RPF_CHAIN_INDEX + location,
		req_tag,
		mask,
		action);

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_filter_flex_set(struct aq_hw_s *self, 
	struct aq_rx_filter_flex *data)
{	
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
	//u32 tag = priv->flex_filter_available == 
	//		(HW_ATL2_FLEX0_AVAILABLE_BIT | HW_ATL2_FLEX1_AVAILABLE_BIT) ? 
	//			(data->location + 1) : 0x1;
	u32 queue = (data->cmd & HW_ATL_RPF_FLEX_RXQH_MSK) >> HW_ATL_RPF_FLEX_RXQH_SHIFT;
		
	if( !(data->location & priv->flex_filter_available) ) {
		return -EINVAL;
	}
	if( data->cmd & HW_ATL_RX_ENABLE_FLTR_FLEX ) {
		if( data->cmd & HW_ATL_RX_ENABLE_QUEUE_FLEX ) {
			hw_atl_rpf_flex_rxqen_set(self, !!(data->cmd & HW_ATL_RX_ENABLE_FLTR_FLEX), data->location);
			hw_atl_rpf_flex_rxqf_set(self, queue, data->location);
		}
		hw_atl_rpf_flex_act_set(self, 
				(data->cmd & HW_ATL_RPF_FLEX_ACTH_MSK) >> HW_ATL_RPF_FLEX_ACTH_SHIFT, 
				 data->location);

		hw_atl_rpf_flex_byte_a_loc_set(self, data->offset_a, data->location);
		hw_atl_rpf_flex_byte_b_loc_set(self, data->offset_b, data->location);

		hw_atl_rpf_flex_byte_a_msk_set(self, data->mask_a, data->location);
		hw_atl_rpf_flex_byte_b_msk_set(self, data->mask_b, data->location);

		hw_atl_rpf_flex_byte_a_pat_set(self, data->biten_a, data->location);
		hw_atl_rpf_flex_byte_b_pat_set(self, data->biten_b, data->location);
		hw_atl_hw_filter_flex_fill_byte_mask(self, data->mask, data->byteen, 
			data->filter_size, data->location);
	}
	hw_atl_rpf_flex_en_set(self, !!(data->cmd & HW_ATL_RX_ENABLE_FLTR_FLEX), data->location);
	/*
	if( queue != CHAINING_QUEUE ) {
		if (data->cmd & (HW_ATL_RX_HOST << HW_ATL_RX_BOFFSET_ACTION_FL3F4))
			action = HW_ATL2_ACTION_ASSIGN_QUEUE(queue);
		else if (data->cmd)
			action = HW_ATL2_ACTION_DROP;
		else
			action = HW_ATL2_ACTION_DISABLE;
		hw_atl2_act_rslvr_table_set(self,
			priv->art_base_index + HW_ATL2_RPF_L3L4_USER_INDEX + data->location,
			req_tag,
			mask,
			action);
	}
	*/
	//printk("%s Flex filter. Place %d.", data->cmd ? "Add" : "Del", data->location);
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_fl3l4_clear(struct aq_hw_s *self,
				  struct aq_rx_filter_l3l4 *data)
{
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
	u8 l4_f_idx = priv->l4_filter_base_index + data->location;
	if( data->location >= priv->l4_filter_count )
		return -EINVAL;

	if (!data->is_ipv6) {

		hw_atl_rpfl3l4_cmd_clear(self, l4_f_idx);
		hw_atl_rpf_l4_spd_set(self, 0U, l4_f_idx);
		hw_atl_rpf_l4_dpd_set(self, 0U, l4_f_idx);
		hw_atl_rpfl3l4_ipv4_src_addr_clear(self, l4_f_idx);
		hw_atl_rpfl3l4_ipv4_dest_addr_clear(self, l4_f_idx);
	} else {
		int i;

		for (i = 0; i < HW_ATL_RX_CNT_REG_ADDR_IPV6; ++i) {
			hw_atl_rpfl3l4_cmd_clear(self, l4_f_idx + i);
			hw_atl_rpf_l4_spd_set(self, 0U, l4_f_idx + i);
			hw_atl_rpf_l4_dpd_set(self, 0U, l4_f_idx + i);
		}
		hw_atl_rpfl3l4_ipv6_src_addr_clear(self, l4_f_idx);
		hw_atl_rpfl3l4_ipv6_dest_addr_clear(self, l4_f_idx);
	}

	return aq_hw_err_from_flags(self);
}

static u64 hw_atl2_ptp_gpio_get_event(struct aq_hw_s *self, u32 channel,
				      u32 *event_count)
{
	u64 event_ts;
	hw_atl2_tsg_gpio_clear_status(self, channel);
	hw_atl2_tsg_gpio_input_event_info_get(self, channel, event_count,
					      &event_ts);
	return aq_hw_err_from_flags(self) ? (u64)-1 : event_ts;
}

static int hw_atl2_hw_fl3l4_set(struct aq_hw_s *self,
				       struct aq_rx_filter_l3l4 *data)
{
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
	u8 l4_f_idx = priv->l4_filter_base_index + data->location;

	if (data->is_ipv6 && !(data->location == 4 || data->location == 0))
		return -EINVAL;

	if( data->location >= priv->l4_filter_count )
		return -EINVAL;

	hw_atl2_hw_fl3l4_clear(self, data);

	hw_atl_rpf_l4_dpd_set(self, data->p_dst, l4_f_idx);
	hw_atl_rpf_l4_spd_set(self, data->p_src, l4_f_idx);

	hw_atl2_hw_fl4_set(self, data);

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_fl3_clear(struct aq_hw_s *self,
				    struct aq_rx_filter_l3 *data)
{
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
	u8 queue = 0xFF;
	u8 l3_f_idx;

	if( data->is_ipv6 ) {
		l3_f_idx = priv->l3_v6_filter_base_index + data->location;
		if( data->location >= priv->l3_v6_filter_count )
			return -EINVAL;
		hw_atl2_rpf_l3_v6_cmd_set(self, 0, l3_f_idx);
		hw_atl2_rpf_l3_v6_tag_set(self, 0, l3_f_idx);
	} else {
		if( data->location >= priv->l3_v6_filter_count )
			return -EINVAL;
		l3_f_idx = priv->l3_v4_filter_base_index + data->location;
		hw_atl_rpfl3l4_cmd_set(self, 0, l3_f_idx);
		hw_atl2_rpf_l3_v4_cmd_set(self, 0, l3_f_idx);
		hw_atl2_rpf_l3_v4_tag_set(self, 0, l3_f_idx);
	}

	queue = (data->cmd  >> HW_ATL_RX_BOFFSET_QUEUE_FL3L4) & 0x7f;
	if( queue != CHAINING_QUEUE ) {
		hw_atl2_act_rslvr_table_set(self,
			priv->art_base_index + HW_ATL2_RPF_L3L4_USER_INDEX + data->location,
			0,
			0,
			HW_ATL2_ACTION_DISABLE);
	}
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_fl3_set(struct aq_hw_s *self,
				  		struct aq_rx_filter_l3 *data)
{
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
	struct hw_atl2_l3_filter l3 = {0};
	u8 location = data->location;
	u8 queue = 0xFF;
	u32 req_tag = 0;
	u32 mask = 0;
	u16 action = 0;
	u8 l3_f_idx;
	int err;

	err = hw_atl2_hw_fl3_clear(self, data);
	if( err < 0 )
		return err;

	if (data->cmd & HW_ATL_RX_ENABLE_L3_IPV6) {
		if (data->cmd & HW_ATL_RX_ENABLE_CMP_DEST_ADDR_L3)
			l3.cmd_l3 |= HW_ATL2_RPF_L3_V6_CMD_DA_EN;
		if (data->cmd & HW_ATL_RX_ENABLE_CMP_SRC_ADDR_L3)
			l3.cmd_l3 |= HW_ATL2_RPF_L3_V6_CMD_SA_EN;
		if (l3.cmd_l3)
			l3.cmd_l3 |= HW_ATL2_RPF_L3_V6_CMD_EN;
		if( data->cmd & HW_ATL_RX_ENABLE_CMP_PROT_L4 ) {
			l3.cmd_l3 |= HW_ATL2_RPF_L3_V6_CMD_PROTO_EN;
			l3.cmd_l3 |= ((data->cmd & HW_ATL_RX_TCP ) ? 0x06U : //TCP
						(data->cmd & HW_ATL_RX_UDP ) ? 0x11U : //UDP
														0xFFu) << 24;  //UNKNOWN
		}

		l3_f_idx = priv->l3_v6_filter_base_index + data->location;
		l3.ipv6 = 1;
		memcpy(l3.srcip, data->ip_src, sizeof(l3.srcip));	
		memcpy(l3.dstip, data->ip_dst, sizeof(l3.dstip));
		hw_atl2_rpf_l3_v6_dest_addr_set(self,
						l3_f_idx,
						data->ip_dst);
		hw_atl2_rpf_l3_v6_src_addr_set(self,
							l3_f_idx,
							data->ip_src);
		hw_atl2_rpf_l3_v6_v4_select_set(self, 0);
		hw_atl2_rpf_l3_v6_cmd_set(self, l3.cmd_l3, l3_f_idx);
		hw_atl2_rpf_l3_v6_tag_set(self, location + 1, l3_f_idx);
	} else {
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

		l3_f_idx = priv->l3_v4_filter_base_index + data->location;
		l3.srcip[0] =  data->ip_src[0];
		l3.dstip[0] =  data->ip_dst[0];
		hw_atl2_rpf_l3_v4_dest_addr_set(self,
						l3_f_idx,
						data->ip_dst[0]);
		hw_atl2_rpf_l3_v4_src_addr_set(self,
							l3_f_idx,
							data->ip_src[0]);
		hw_atl2_rpf_l3_v6_v4_select_set(self, 1);
		hw_atl2_rpf_l3_v4_cmd_set(self, l3.cmd_l3, l3_f_idx);
		hw_atl2_rpf_l3_v4_tag_set(self, location + 1, l3_f_idx);
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

	queue = (data->cmd  >> HW_ATL_RX_BOFFSET_QUEUE_FL3L4) & 0x7f;
	if( queue != CHAINING_QUEUE ) {
		if (data->cmd & (HW_ATL_RX_HOST << HW_ATL_RX_BOFFSET_ACTION_FL3F4))
			action = HW_ATL2_ACTION_ASSIGN_QUEUE(queue);
		else if (data->cmd)
			action = HW_ATL2_ACTION_DROP;
		else
			action = HW_ATL2_ACTION_DISABLE;
		hw_atl2_act_rslvr_table_set(self,
			priv->art_base_index + HW_ATL2_RPF_L3L4_USER_INDEX + data->location,
			req_tag,
			mask,
			action);
	}
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_fl4_set(struct aq_hw_s *self,
				  struct aq_rx_filter_l3l4 *data)
{
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
	struct hw_atl2_l4_filter l4 = {0};
	u8 location = data->location;
	u8 l4_f_idx = priv->l4_filter_base_index + data->location;
	u8 queue = 0xFF;
	u32 req_tag = 0;
	u32 mask = 0;
	u16 action = 0;

	if( data->location >= priv->l4_filter_count )
		return -EINVAL;

	if (data->cmd & HW_ATL_RX_ENABLE_CMP_DEST_PORT_L4)
		l4.cmd_l4 |= HW_ATL2_RPF_L4_CMD_DP_EN;
	if (data->cmd & HW_ATL_RX_ENABLE_CMP_SRC_PORT_L4)
		l4.cmd_l4 |= HW_ATL2_RPF_L4_CMD_SP_EN;
	if (l4.cmd_l4)
		l4.cmd_l4 |= HW_ATL2_RPF_L4_CMD_EN;
	l4.sport = data->p_src;
	l4.dport = data->p_dst;

	hw_atl2_rpf_l4_tag_set(self, location + 1, l4_f_idx);
	hw_atl2_rpf_l4_cmd_set(self, l4.cmd_l4, l4_f_idx);

	req_tag |= (location + 1) << HW_ATL2_RPF_TAG_L4_OFFSET;
	mask |= HW_ATL2_RPF_TAG_L4_MASK;

	queue = (data->cmd  >> HW_ATL_RX_BOFFSET_QUEUE_FL3L4) & 0x7f;
	if( queue != CHAINING_QUEUE ) {
		if (data->cmd & (HW_ATL_RX_HOST << HW_ATL_RX_BOFFSET_ACTION_FL3F4))
			action = HW_ATL2_ACTION_ASSIGN_QUEUE(queue);
		else if (data->cmd)
			action = HW_ATL2_ACTION_DROP;
		else
			action = HW_ATL2_ACTION_DISABLE;

		hw_atl2_act_rslvr_table_set(self,
			priv->art_base_index + HW_ATL2_RPF_L3L4_USER_INDEX + data->location,
			req_tag,
			mask,
			action);
	}
	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_fl2_set(struct aq_hw_s *self,
			      struct aq_rx_filter_l2 *data)
{
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
	u32 mask = HW_ATL2_RPF_TAG_ET_MASK;
	u32 req_tag = 0;
	u16 action = 0;
	u8 et_loc =  (data->location & 0xf);
	u8 et_f_idx = priv->etype_filter_base_index + et_loc;
	u8 dst_loc = (data->location & 0xf0) >> 4;

	if( data->en_flag & HW_ATL_RX_L2_ET_EN ) {
		req_tag = (et_loc + 1) << HW_ATL2_RPF_TAG_ET_OFFSET;
		mask = HW_ATL2_RPF_TAG_ET_MASK;
		hw_atl_rpf_etht_flr_en_set(self, 1U, et_f_idx);
		hw_atl_rpf_etht_flr_set(self, data->ethertype, et_f_idx);
		hw_atl_rpf_etht_user_priority_en_set(self,
							!!(data->en_flag & HW_ATL_RX_L2_UP_EN),
							et_f_idx);
		if( data->en_flag & HW_ATL_RX_L2_UP_EN ) {
			hw_atl_rpf_etht_user_priority_set(self,
							data->user_priority,
							et_f_idx);
			req_tag |= data->user_priority << HW_ATL2_RPF_TAG_PCP_OFFSET;
			mask |= HW_ATL2_RPF_TAG_PCP_MASK;
		}

		if (data->queue == 0xFF ) {
			hw_atl_rpf_etht_flr_act_set(self, 0U, et_f_idx);
			hw_atl_rpf_etht_rx_queue_en_set(self, 0U, et_f_idx);
		} else {
			hw_atl_rpf_etht_flr_act_set(self, 1U, et_f_idx);
			hw_atl_rpf_etht_rx_queue_en_set(self, 1U, et_f_idx);
			if( data->queue != CHAINING_QUEUE ) {
				hw_atl_rpf_etht_rx_queue_set(self, data->queue, et_f_idx);
			}
		}
	}

	if( data->en_flag & HW_ATL_RX_L2_DST_EN ) {
		req_tag |= (dst_loc) << HW_ATL2_RPF_TAG_UC_OFFSET;
		mask |= HW_ATL2_RPF_TAG_UC_MASK;
		//printk("Change uc/mc filter to tag %x!\n", dst_loc);
		priv->special_uc_tag[dst_loc - HW_ATL2_MAC_MIN] = dst_loc;
		hw_atl2_rpfl2_uc_flr_tag_set(self,
							dst_loc,
							priv->l2_filters_base_index + dst_loc);
	}

	if (data->queue == 0xFF ) {
		action = HW_ATL2_ACTION_DROP;
	} else if( data->queue != CHAINING_QUEUE ) {
		action = HW_ATL2_ACTION_ASSIGN_QUEUE(data->queue);
	}

	if( data->en_flag & HW_ATL_RX_L2_ET_EN ) {
		hw_atl2_rpf_etht_flr_tag_set(self,
						et_loc + 1,
						et_f_idx);
	}

	if( data->queue != CHAINING_QUEUE ) {
		hw_atl2_act_rslvr_table_set(self,
			data->en_flag & HW_ATL_RX_L2_ET_EN ? 
				priv->art_base_index + HW_ATL2_RPF_ET_PCP_USER_INDEX + et_loc : 
				priv->art_base_index + HW_ATL2_RPF_DST_FIRST_INDEX + et_loc,
			req_tag,
			mask,
			action);
	}

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_fl2_clear(struct aq_hw_s *self,
				struct aq_rx_filter_l2 *data)
{
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
	u8 et_loc = data->location & 0xf;
	u8 et_f_idx = priv->etype_filter_base_index + et_loc;
	u8 dst_loc = (data->location & 0xf0) >> 4;
	u8 index;

	hw_atl_rpf_etht_flr_en_set(self, 0U, et_f_idx);
	hw_atl_rpf_etht_flr_set(self, 0U, et_f_idx);
	hw_atl_rpf_etht_user_priority_en_set(self, 0U, et_f_idx);
	if( data->en_flag & HW_ATL_RX_L2_DST_EN ) {
		//printk("Change uc/mc filter from tag %x to 1!\n", dst_loc);
		priv->special_uc_tag[dst_loc - HW_ATL2_MAC_MIN] = HW_ATL2_RPF_TAG_BASE_UC;
		hw_atl2_rpfl2_uc_flr_tag_set(self,
			HW_ATL2_RPF_TAG_BASE_UC,
			priv->l2_filters_base_index + dst_loc);
	}

	//u32 tag;

	index = data->en_flag & HW_ATL_RX_L2_ET_EN ? 
				priv->art_base_index + HW_ATL2_RPF_ET_PCP_USER_INDEX + et_loc : 
				priv->art_base_index + HW_ATL2_RPF_DST_FIRST_INDEX + et_loc;
	hw_atl2_act_rslvr_table_set(self, index, 0, 0,
					HW_ATL2_ACTION_DISABLE);
	//tag = hw_atl2_rpf_etht_flr_tag_get(self, location);
	//hw_atl2_filter_tag_put(priv->etype_policy, tag);

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
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
	u16 location;
	u16 queue;
	u8 index;
	int i;

	hw_atl_rpf_vlan_prom_mode_en_set(self, 1U);

	for (i = 0; i < priv->vlan_filter_count; i++) {
		queue = HW_ATL2_ACTION_ASSIGN_QUEUE(aq_vlans[i].queue);
		location = priv->vlan_filter_base_index + i;

		hw_atl_rpf_vlan_flr_en_set(self, 0U, location);
		hw_atl_rpf_vlan_rxq_en_flr_set(self, 0U, location);
		index = priv->art_base_index + HW_ATL2_RPF_VLAN_USER_INDEX + i;
		hw_atl2_act_rslvr_table_set(self, index, 0, 0,
					    HW_ATL2_ACTION_DISABLE);
		if (aq_vlans[i].enable) {
			hw_atl_rpf_vlan_id_flr_set(self,
						   aq_vlans[i].vlan_id,
						   location);
			hw_atl_rpf_vlan_flr_act_set(self, 1U, location);
			hw_atl_rpf_vlan_flr_en_set(self, 1U, location);

			if (aq_vlans[i].queue != 0xFF) {
				hw_atl_rpf_vlan_rxq_flr_set(self,
							    aq_vlans[i].queue,
							    location);
				hw_atl_rpf_vlan_rxq_en_flr_set(self, 1U,
							       location);

				hw_atl2_rpf_vlan_flr_tag_set(self, i + 2,
							     location);
				if( aq_vlans[i].queue != CHAINING_QUEUE ) {
					hw_atl2_act_rslvr_table_set(self, index,
						(i + 2) << HW_ATL2_RPF_TAG_VLAN_OFFSET,
						HW_ATL2_RPF_TAG_VLAN_MASK, queue);
				}
			} else {
				hw_atl2_rpf_vlan_flr_tag_set(self, 1, location);
			}
		}
	}

	return aq_hw_err_from_flags(self);
}

static int hw_atl2_hw_vlan_ctrl(struct aq_hw_s *self, bool enable)
{
	/* set promisc in case of disabing the vlan filter */
	hw_atl_rpf_vlan_prom_mode_en_set(self, !enable);
	hw_atl2_hw_new_rx_filter_vlan_promisc(self, !enable);

	return aq_hw_err_from_flags(self);
}

void hw_atl2_apply_link_speed(struct aq_hw_s *self, unsigned int mbps)
{
	struct hw_atl2_priv *priv = (struct hw_atl2_priv *)self->priv;
#ifdef TSN_SUPPORT
	unsigned int adj =  mbps == 10000 ? 0x4 : //TODO
						mbps == 5000  ? 0x4 : //TODO
						mbps == 2500  ? 0x4 : //TODO
						mbps == 1000  ? 0x6 : 
						mbps == 100   ? 0xF : 4;
	hw_atl2_correct_launchtime(self, mbps, adj);
#endif
	priv->ptp_check_cntr = 0; // w/a if first PTP timestamp is missed = 32;
	if (ATL_HW_IS_CHIP_FEATURE(self, REVISION_B0)) {
		hw_atl2_adj_msm_freq(self, priv->ppb);
	}
}

#ifdef TSN_SUPPORT
static int hw_atl2_tsn_enable(struct aq_hw_s *self, int enable)
{
	printk("AVB enable\n");
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
		hw_atl2_pci_cfg_ptm_endianess_fix(self, endianess_fix);
	}
	//hw_atl2_phi_ptm_en(self, timeout > 0);
	//hw_atl2_request_to_auto_update_clock_set(self, timeout > 0);
	hw_atl2_ptm_update_counter_value_set(self, (timeout*156250)); //ms => 6.4 ns
	hw_atl2_pci_cfg_ptm_long_timer_set(self, timeout - 1);
	//hw_atl2_timer_enable_to_update_ptm_clock_set(self, timeout > 0);
	hw_atl2_request_to_auto_update_clock_set(self, timeout > 0);
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
	info->free_run_clock = self->clk_select == ATL_TSG_CLOCK_SEL_1 ?
							hw_atl2_ptm_tsg_local_clock_get(self) : 
							hw_atl2_ptp_tsg_local_clock_get(self);

	//printk("Valid? Seq Id %d\n", info->seq_id);
	//hw_atl2_ptm_clear_status(self);
	return aq_hw_err_from_flags(self);
}
#endif

const struct aq_hw_ops hw_atl2_ops = {
	.hw_soft_reset        = hw_atl2_utils_soft_reset,
	.hw_prepare           = hw_atl2_utils_initfw,
	.hw_set_mac_address   = hw_atl2_hw_mac_addr_set,
	.hw_init              = hw_atl2_hw_init,
	.hw_reset             = hw_atl2_hw_reset,
	.hw_start             = hw_atl_b0_hw_start,
	.hw_ring_tx_start     = hw_atl_b0_hw_ring_tx_start,
	.hw_ring_tx_stop      = hw_atl_b0_hw_ring_tx_stop,
	.hw_ring_rx_start     = hw_atl_b0_hw_ring_rx_start,
	.hw_ring_rx_stop      = hw_atl_b0_hw_ring_rx_stop,
	.hw_stop              = hw_atl2_hw_stop,

	.hw_ring_tx_xmit         = hw_atl_b0_hw_ring_tx_xmit,
	.hw_ring_tx_head_update  = hw_atl_b0_hw_ring_tx_head_update,

	.hw_ring_rx_receive      = hw_atl_b0_hw_ring_rx_receive,
	.hw_ring_rx_fill         = hw_atl_b0_hw_ring_rx_fill,

	.hw_irq_enable           = hw_atl_b0_hw_irq_enable,
	.hw_irq_disable          = hw_atl_b0_hw_irq_disable,
	.hw_irq_read             = hw_atl_b0_hw_irq_read,

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
	.hw_rss_hash_set             = hw_atl_b0_hw_rss_hash_set,
	.hw_tc_rate_limit_set        = hw_atl2_hw_init_tx_tc_rate_limit,
	.hw_get_regs                 = hw_atl2_utils_hw_get_regs,
	.hw_get_hw_stats             = hw_atl2_utils_get_hw_stats,
	.hw_get_fw_version           = hw_atl2_utils_get_fw_version,
	.hw_set_offload              = hw_atl_b0_hw_offload_set,
	.hw_set_loopback             = hw_atl_b0_set_loopback,
	.hw_set_fc                   = hw_atl_b0_set_fc,

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
	.hw_tx_ptp_ring_init     = hw_atl2_hw_tx_ptp_ring_init,
	.hw_rx_ptp_ring_init     = hw_atl2_hw_rx_ptp_ring_init,
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
