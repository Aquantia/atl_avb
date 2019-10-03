/*
 * aQuantia Corporation Network Driver
 * Copyright (C) 2014-2019 aQuantia Corporation. All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

/* File hw_atl2_llh.h: Declarations of bitfield and register access functions for
 * Atlantic registers.
 */

#ifndef HW_ATL2_LLH_H
#define HW_ATL2_LLH_H

#include <linux/types.h>

struct aq_hw_s;

/* global */

/* reset */
void hw_atl2_dmc_logic_reset_set(struct aq_hw_s *aq_hw, uint32_t dmcLogicReset);
uint32_t hw_atl2_dmc_logic_reset_get(struct aq_hw_s *aq_hw);
void hw_atl2_phi_logic_reset_set(struct aq_hw_s *aq_hw, uint32_t phiLogicReset);
uint32_t hw_atl2_phi_logic_reset_get(struct aq_hw_s *aq_hw);
void hw_atl2_phi_register_reset_set(struct aq_hw_s *aq_hw, uint32_t phiRegisterReset);
uint32_t hw_atl2_phi_register_reset_get(struct aq_hw_s *aq_hw);
void hw_atl2_mif_mips_combo_reset(struct aq_hw_s *aq_hw);
void hw_atl2_mips_reset_pulse_set(struct aq_hw_s *aq_hw, uint32_t mipsResetPulse);
uint32_t hw_atl2_mips_reset_pulse_get(struct aq_hw_s *aq_hw);
void hw_atl2_mif_ahb_reset_set(struct aq_hw_s *aq_hw, uint32_t ahbReset);
uint32_t hw_atl2_mif_ahb_reset_get(struct aq_hw_s *aq_hw);
void hw_atl2_mif_aux_reset_set(struct aq_hw_s *aq_hw, uint32_t auxReset);
uint32_t hw_atl2_mif_aux_reset_get(struct aq_hw_s *aq_hw);
void hw_atl2_mif_logic_reset_set(struct aq_hw_s *aq_hw, uint32_t mifLogicReset);
uint32_t hw_atl2_mif_logic_reset_get(struct aq_hw_s *aq_hw);
void hw_atl2_mif_register_reset_set(struct aq_hw_s *aq_hw, uint32_t mifRegisterReset);
uint32_t hw_atl2_mif_register_reset_get(struct aq_hw_s *aq_hw);
void hw_atl2_mpi_mac_phy_path_reset_set(struct aq_hw_s *aq_hw, uint32_t macPhyPathReset);
uint32_t hw_atl2_mpi_mac_phy_path_reset_get(struct aq_hw_s *aq_hw);
void hw_atl2_mpi_mac_phy_register_reset_set(struct aq_hw_s *aq_hw, uint32_t macPhyRegisterReset);
uint32_t hw_atl2_mpi_mac_phy_register_reset_get(struct aq_hw_s *aq_hw);
void hw_atl2_tx_logic_reset_set(struct aq_hw_s *aq_hw, uint32_t txLogicReset);
uint32_t hw_atl2_tx_logic_reset_get(struct aq_hw_s *aq_hw);
void hw_atl2_tx_register_reset_set(struct aq_hw_s *aq_hw, uint32_t txRegisterReset);
uint32_t hw_atl2_tx_register_reset_get(struct aq_hw_s *aq_hw);
void hw_atl2_rx_logic_reset_set(struct aq_hw_s *aq_hw, uint32_t rxLogicReset);
uint32_t hw_atl2_rx_logic_reset_get(struct aq_hw_s *aq_hw);
void hw_atl2_rx_register_reset_set(struct aq_hw_s *aq_hw, uint32_t rxRegisterReset);
uint32_t hw_atl2_rx_register_reset_get(struct aq_hw_s *aq_hw);

void hw_atl2_itr_reset_interrupt_set(struct aq_hw_s *aq_hw, uint32_t resetInterrupt);
uint32_t hw_atl2_itr_reset_interrupt_get(struct aq_hw_s *aq_hw);
void hw_atl2_itr_interrupt_register_reset_disable_set(struct aq_hw_s *aq_hw, uint32_t interruptRegisterResetDisable);
uint32_t hw_atl2_itr_interrupt_register_reset_disable_get(struct aq_hw_s *aq_hw);


/* set global microprocessor semaphore */
void hw_atl2_reg_glb_cpu_sem_set(struct aq_hw_s *aq_hw,	u32 glb_cpu_sem,
				u32 semaphore);

/* get global microprocessor semaphore */
u32 hw_atl2_reg_glb_cpu_sem_get(struct aq_hw_s *aq_hw, u32 semaphore);

/* set global register reset disable */
void hw_atl2_glb_glb_reg_res_dis_set(struct aq_hw_s *aq_hw, u32 glb_reg_res_dis);

/* set soft reset */
void hw_atl2_glb_soft_res_set(struct aq_hw_s *aq_hw, u32 soft_res);

/* get soft reset */
u32 hw_atl2_glb_soft_res_get(struct aq_hw_s *aq_hw);

/* stats */

u32 hw_atl2_rpb_rx_dma_drop_pkt_cnt_get(struct aq_hw_s *aq_hw);

/* get rx dma good octet counter lsw */
u32 hw_atl2_stats_rx_dma_good_octet_counterlsw_get(struct aq_hw_s *aq_hw);

/* get rx dma good packet counter lsw */
u32 hw_atl2_stats_rx_dma_good_pkt_counterlsw_get(struct aq_hw_s *aq_hw);

/* get tx dma good octet counter lsw */
u32 hw_atl2_stats_tx_dma_good_octet_counterlsw_get(struct aq_hw_s *aq_hw);

/* get tx dma good packet counter lsw */
u32 hw_atl2_stats_tx_dma_good_pkt_counterlsw_get(struct aq_hw_s *aq_hw);

/* get rx dma good octet counter msw */
u32 hw_atl2_stats_rx_dma_good_octet_countermsw_get(struct aq_hw_s *aq_hw);

/* get rx dma good packet counter msw */
u32 hw_atl2_stats_rx_dma_good_pkt_countermsw_get(struct aq_hw_s *aq_hw);

/* get tx dma good octet counter msw */
u32 hw_atl2_stats_tx_dma_good_octet_countermsw_get(struct aq_hw_s *aq_hw);

/* get tx dma good packet counter msw */
u32 hw_atl2_stats_tx_dma_good_pkt_countermsw_get(struct aq_hw_s *aq_hw);

/* get msm rx errors counter register */
u32 hw_atl2_reg_mac_msm_rx_errs_cnt_get(struct aq_hw_s *aq_hw);

/* get msm rx unicast frames counter register */
u32 hw_atl2_reg_mac_msm_rx_ucst_frm_cnt_get(struct aq_hw_s *aq_hw);

/* get msm rx multicast frames counter register */
u32 hw_atl2_reg_mac_msm_rx_mcst_frm_cnt_get(struct aq_hw_s *aq_hw);

/* get msm rx broadcast frames counter register */
u32 hw_atl2_reg_mac_msm_rx_bcst_frm_cnt_get(struct aq_hw_s *aq_hw);

/* get msm rx broadcast octets counter register 1 */
u32 hw_atl2_reg_mac_msm_rx_bcst_octets_counter1get(struct aq_hw_s *aq_hw);

/* get msm rx unicast octets counter register 0 */
u32 hw_atl2_reg_mac_msm_rx_ucst_octets_counter0get(struct aq_hw_s *aq_hw);

/* get rx dma statistics counter 7 */
u32 hw_atl2_reg_rx_dma_stat_counter7get(struct aq_hw_s *aq_hw);

/* get msm tx errors counter register */
u32 hw_atl2_reg_mac_msm_tx_errs_cnt_get(struct aq_hw_s *aq_hw);

/* get msm tx unicast frames counter register */
u32 hw_atl2_reg_mac_msm_tx_ucst_frm_cnt_get(struct aq_hw_s *aq_hw);

/* get msm tx multicast frames counter register */
u32 hw_atl2_reg_mac_msm_tx_mcst_frm_cnt_get(struct aq_hw_s *aq_hw);

/* get msm tx broadcast frames counter register */
u32 hw_atl2_reg_mac_msm_tx_bcst_frm_cnt_get(struct aq_hw_s *aq_hw);

/* get msm tx multicast octets counter register 1 */
u32 hw_atl2_reg_mac_msm_tx_mcst_octets_counter1get(struct aq_hw_s *aq_hw);

/* get msm tx broadcast octets counter register 1 */
u32 hw_atl2_reg_mac_msm_tx_bcst_octets_counter1get(struct aq_hw_s *aq_hw);

/* get msm tx unicast octets counter register 0 */
u32 hw_atl2_reg_mac_msm_tx_ucst_octets_counter0get(struct aq_hw_s *aq_hw);

/* get global mif identification */
u32 hw_atl2_reg_glb_mif_id_get(struct aq_hw_s *aq_hw);

/* interrupt */

/* set interrupt auto mask lsw */
void hw_atl2_itr_irq_auto_masklsw_set(struct aq_hw_s *aq_hw,
				     u32 irq_auto_masklsw);

/* set interrupt mapping enable rx */
void hw_atl2_itr_irq_map_en_rx_set(struct aq_hw_s *aq_hw, u32 irq_map_en_rx,
				  u32 rx);

/* set interrupt mapping enable tx */
void hw_atl2_itr_irq_map_en_tx_set(struct aq_hw_s *aq_hw, u32 irq_map_en_tx,
				  u32 tx);

/* set interrupt mapping rx */
void hw_atl2_itr_irq_map_rx_set(struct aq_hw_s *aq_hw, u32 irq_map_rx, u32 rx);

/* set interrupt mapping tx */
void hw_atl2_itr_irq_map_tx_set(struct aq_hw_s *aq_hw, u32 irq_map_tx, u32 tx);

/* set interrupt mask clear lsw */
void hw_atl2_itr_irq_msk_clearlsw_set(struct aq_hw_s *aq_hw,
				     u32 irq_msk_clearlsw);

/* set interrupt mask set lsw */
void hw_atl2_itr_irq_msk_setlsw_set(struct aq_hw_s *aq_hw, u32 irq_msk_setlsw);

/* set interrupt register reset disable */
void hw_atl2_itr_irq_reg_res_dis_set(struct aq_hw_s *aq_hw, u32 irq_reg_res_dis);

/* set interrupt status clear lsw */
void hw_atl2_itr_irq_status_clearlsw_set(struct aq_hw_s *aq_hw,
					u32 irq_status_clearlsw);

/* get interrupt status lsw */
u32 hw_atl2_itr_irq_statuslsw_get(struct aq_hw_s *aq_hw);

/* get reset interrupt */
u32 hw_atl2_itr_res_irq_get(struct aq_hw_s *aq_hw);

/* set reset interrupt */
void hw_atl2_itr_res_irq_set(struct aq_hw_s *aq_hw, u32 res_irq);

/* rdm */

/* set cpu id */
void hw_atl2_rdm_cpu_id_set(struct aq_hw_s *aq_hw, u32 cpuid, u32 dca);

/* set rx dca enable */
void hw_atl2_rdm_rx_dca_en_set(struct aq_hw_s *aq_hw, u32 rx_dca_en);

/* set rx dca mode */
void hw_atl2_rdm_rx_dca_mode_set(struct aq_hw_s *aq_hw, u32 rx_dca_mode);

/* set rx descriptor data buffer size */
void hw_atl2_rdm_rx_desc_data_buff_size_set(struct aq_hw_s *aq_hw,
					   u32 rx_desc_data_buff_size,
				    u32 descriptor);

/* set rx descriptor dca enable */
void hw_atl2_rdm_rx_desc_dca_en_set(struct aq_hw_s *aq_hw, u32 rx_desc_dca_en,
				   u32 dca);

/* set rx descriptor enable */
void hw_atl2_rdm_rx_desc_en_set(struct aq_hw_s *aq_hw, u32 rx_desc_en,
			       u32 descriptor);

/* set rx descriptor header splitting */
void hw_atl2_rdm_rx_desc_head_splitting_set(struct aq_hw_s *aq_hw,
					   u32 rx_desc_head_splitting,
				    u32 descriptor);

/* get rx descriptor head pointer */
u32 hw_atl2_rdm_rx_desc_head_ptr_get(struct aq_hw_s *aq_hw, u32 descriptor);

/* set rx descriptor length */
void hw_atl2_rdm_rx_desc_len_set(struct aq_hw_s *aq_hw, u32 rx_desc_len,
				u32 descriptor);

/* set rx descriptor write-back interrupt enable */
void hw_atl2_rdm_rx_desc_wr_wb_irq_en_set(struct aq_hw_s *aq_hw,
					 u32 rx_desc_wr_wb_irq_en);

/* set rx header dca enable */
void hw_atl2_rdm_rx_head_dca_en_set(struct aq_hw_s *aq_hw, u32 rx_head_dca_en,
				   u32 dca);

/* set rx payload dca enable */
void hw_atl2_rdm_rx_pld_dca_en_set(struct aq_hw_s *aq_hw, u32 rx_pld_dca_en,
				  u32 dca);

/* set rx descriptor header buffer size */
void hw_atl2_rdm_rx_desc_head_buff_size_set(struct aq_hw_s *aq_hw,
					   u32 rx_desc_head_buff_size,
					   u32 descriptor);

/* set rx descriptor reset */
void hw_atl2_rdm_rx_desc_res_set(struct aq_hw_s *aq_hw, u32 rx_desc_res,
				u32 descriptor);

/* Set RDM Interrupt Moderation Enable */
void hw_atl2_rdm_rdm_intr_moder_en_set(struct aq_hw_s *aq_hw,
				      u32 rdm_intr_moder_en);

/* reg */

/* set general interrupt mapping register */
void hw_atl2_reg_gen_irq_map_set(struct aq_hw_s *aq_hw, u32 gen_intr_map,
				u32 regidx);

/* get general interrupt status register */
u32 hw_atl2_reg_gen_irq_status_get(struct aq_hw_s *aq_hw);

/* set interrupt global control register */
void hw_atl2_reg_irq_glb_ctl_set(struct aq_hw_s *aq_hw, u32 intr_glb_ctl);

/* set interrupt throttle register */
void hw_atl2_reg_irq_thr_set(struct aq_hw_s *aq_hw, u32 intr_thr, u32 throttle);

/* set rx dma descriptor base address lsw */
void hw_atl2_reg_rx_dma_desc_base_addresslswset(struct aq_hw_s *aq_hw,
					       u32 rx_dma_desc_base_addrlsw,
					u32 descriptor);

/* set rx dma descriptor base address msw */
void hw_atl2_reg_rx_dma_desc_base_addressmswset(struct aq_hw_s *aq_hw,
					       u32 rx_dma_desc_base_addrmsw,
					u32 descriptor);

/* get rx dma descriptor status register */
u32 hw_atl2_reg_rx_dma_desc_status_get(struct aq_hw_s *aq_hw, u32 descriptor);

/* set rx dma descriptor tail pointer register */
void hw_atl2_reg_rx_dma_desc_tail_ptr_set(struct aq_hw_s *aq_hw,
					 u32 rx_dma_desc_tail_ptr,
				  u32 descriptor);

/* set rx filter multicast filter mask register */
void hw_atl2_reg_rx_flr_mcst_flr_msk_set(struct aq_hw_s *aq_hw,
					u32 rx_flr_mcst_flr_msk);

/* set rx filter multicast filter register */
void hw_atl2_reg_rx_flr_mcst_flr_set(struct aq_hw_s *aq_hw, u32 rx_flr_mcst_flr,
				    u32 filter);

/* set rx filter rss control register 1 */
void hw_atl2_reg_rx_flr_rss_control1set(struct aq_hw_s *aq_hw,
				       u32 rx_flr_rss_control1);

/* set rx filter rss hash type */
void hw_atl2_reg_rx_flr_rss_hash_type_set(struct aq_hw_s *aq_hw,
				       u32 rx_flr_rss_hash_type);

/* Set RX Filter Control Register 2 */
void hw_atl2_reg_rx_flr_control2_set(struct aq_hw_s *aq_hw, u32 rx_flr_control2);

/* Set RX Interrupt Moderation Control Register */
void hw_atl2_reg_rx_intr_moder_ctrl_set(struct aq_hw_s *aq_hw,
				       u32 rx_intr_moderation_ctl,
				u32 queue);

/* set tx dma debug control */
void hw_atl2_reg_tx_dma_debug_ctl_set(struct aq_hw_s *aq_hw,
				     u32 tx_dma_debug_ctl);

/* set tx dma descriptor base address lsw */
void hw_atl2_reg_tx_dma_desc_base_addresslswset(struct aq_hw_s *aq_hw,
					       u32 tx_dma_desc_base_addrlsw,
					u32 descriptor);

/* set tx dma descriptor base address msw */
void hw_atl2_reg_tx_dma_desc_base_addressmswset(struct aq_hw_s *aq_hw,
					       u32 tx_dma_desc_base_addrmsw,
					u32 descriptor);

/* set tx dma descriptor tail pointer register */
void hw_atl2_reg_tx_dma_desc_tail_ptr_set(struct aq_hw_s *aq_hw,
					 u32 tx_dma_desc_tail_ptr,
					 u32 descriptor);

/* Set TX Interrupt Moderation Control Register */
void hw_atl2_reg_tx_intr_moder_ctrl_set(struct aq_hw_s *aq_hw,
				       u32 tx_intr_moderation_ctl,
				       u32 queue);

/* set global microprocessor scratch pad */
void hw_atl2_reg_glb_cpu_scratch_scp_set(struct aq_hw_s *aq_hw,
					u32 glb_cpu_scratch_scp,
					u32 scratch_scp);

/* rpb */

/* set dma system loopback */
void hw_atl2_rpb_dma_sys_lbk_set(struct aq_hw_s *aq_hw, u32 dma_sys_lbk);

/* set dma network loopback */
void hw_atl2_rpb_dma_net_lbk_set(struct aq_hw_s *aq_hw, u32 dma_net_lbk);

/* set rx traffic class mode */
void hw_atl2_rpb_rpf_rx_traf_class_mode_set(struct aq_hw_s *aq_hw,
					   u32 rx_traf_class_mode);

/* get rx traffic class mode */
u32 hw_atl2_rpb_rpf_rx_traf_class_mode_get(struct aq_hw_s *aq_hw);

/* set rx buffer enable */
void hw_atl2_rpb_rx_buff_en_set(struct aq_hw_s *aq_hw, u32 rx_buff_en);

/* set rx buffer high threshold (per tc) */
void hw_atl2_rpb_rx_buff_hi_threshold_per_tc_set(struct aq_hw_s *aq_hw,
						u32 rx_buff_hi_threshold_per_tc,
						u32 buffer);

/* set rx buffer low threshold (per tc) */
void hw_atl2_rpb_rx_buff_lo_threshold_per_tc_set(struct aq_hw_s *aq_hw,
						u32 rx_buff_lo_threshold_per_tc,
					 u32 buffer);

/* set rx flow control mode */
void hw_atl2_rpb_rx_flow_ctl_mode_set(struct aq_hw_s *aq_hw, u32 rx_flow_ctl_mode);

/* set rx packet buffer size (per tc) */
void hw_atl2_rpb_rx_pkt_buff_size_per_tc_set(struct aq_hw_s *aq_hw,
					    u32 rx_pkt_buff_size_per_tc,
					    u32 buffer);

/* set rdm rx dma descriptor cache init */
void hw_atl2_rdm_rx_dma_desc_cache_init_tgl(struct aq_hw_s *aq_hw);
u32 hw_atl2_rdm_rx_dma_desc_cache_init_done_get(struct aq_hw_s *aq_hw);

/* set rx xoff enable (per tc) */
void hw_atl2_rpb_rx_xoff_en_per_tc_set(struct aq_hw_s *aq_hw, u32 rx_xoff_en_per_tc,
				      u32 buffer);

/* rpf */

/* set new RPF enable */
void hw_atl2_rpf_new_enable_set(struct aq_hw_s *aq_hw, u32 enable);

/* set l2 broadcast count threshold */
void hw_atl2_rpfl2broadcast_count_threshold_set(struct aq_hw_s *aq_hw,
					       u32 l2broadcast_count_threshold);

/* set l2 broadcast enable */
void hw_atl2_rpfl2broadcast_en_set(struct aq_hw_s *aq_hw, u32 l2broadcast_en);

/* set l2 broadcast filter action */
void hw_atl2_rpfl2broadcast_flr_act_set(struct aq_hw_s *aq_hw,
				       u32 l2broadcast_flr_act);

/* set l2 multicast filter enable */
void hw_atl2_rpfl2multicast_flr_en_set(struct aq_hw_s *aq_hw,
				      u32 l2multicast_flr_en,
				      u32 filter);

/* set l2 promiscuous mode enable */
void hw_atl2_rpfl2promiscuous_mode_en_set(struct aq_hw_s *aq_hw,
					 u32 l2promiscuous_mode_en);
/* get l2 promiscuous mode enable */
u32 hw_atl2_rpfl2promiscuous_mode_en_get(struct aq_hw_s *aq_hw);

/* set l2 unicast filter action */
void hw_atl2_rpfl2unicast_flr_act_set(struct aq_hw_s *aq_hw,
				     u32 l2unicast_flr_act,
				     u32 filter);

/* set l2 unicast filter enable */
void hw_atl2_rpfl2_uc_flr_en_set(struct aq_hw_s *aq_hw, u32 l2unicast_flr_en,
				u32 filter);

/* set l2 unicast destination address lsw */
void hw_atl2_rpfl2unicast_dest_addresslsw_set(struct aq_hw_s *aq_hw,
					     u32 l2unicast_dest_addresslsw,
				      u32 filter);

/* set l2 unicast destination address msw */
void hw_atl2_rpfl2unicast_dest_addressmsw_set(struct aq_hw_s *aq_hw,
					     u32 l2unicast_dest_addressmsw,
				      u32 filter);

/* set l2 unicast filter tag */
void hw_atl2_rpfl2_uc_flr_tag_set(struct aq_hw_s *aq_hw, u32 tag, u32 filter);

/* set l2 broadcast filter tag */
void hw_atl2_rpfl2_bc_flr_tag_set(struct aq_hw_s *aq_hw, u32 tag);


/* Set L2 Accept all Multicast packets */
void hw_atl2_rpfl2_accept_all_mc_packets_set(struct aq_hw_s *aq_hw,
					    u32 l2_accept_all_mc_packets);

/* set user-priority tc mapping */
void hw_atl2_rpf_rpb_user_priority_tc_map_set(struct aq_hw_s *aq_hw,
					     u32 user_priority_tc_map, u32 tc);

/* set rss key address */
void hw_atl2_rpf_rss_key_addr_set(struct aq_hw_s *aq_hw, u32 rss_key_addr);

/* set rss key write data */
void hw_atl2_rpf_rss_key_wr_data_set(struct aq_hw_s *aq_hw, u32 rss_key_wr_data);

/* get rss key write enable */
u32 hw_atl2_rpf_rss_key_wr_en_get(struct aq_hw_s *aq_hw);

/* set rss key write enable */
void hw_atl2_rpf_rss_key_wr_en_set(struct aq_hw_s *aq_hw, u32 rss_key_wr_en);

/* set rss redirection table address */
void hw_atl2_rpf_rss_redir_tbl_addr_set(struct aq_hw_s *aq_hw,
				       u32 rss_redir_tbl_addr);

/* set rss redirection table write data */
void hw_atl2_rpf_rss_redir_tbl_wr_data_set(struct aq_hw_s *aq_hw,
					  u32 rss_redir_tbl_wr_data);

/* get rss redirection write enable */
u32 hw_atl2_rpf_rss_redir_wr_en_get(struct aq_hw_s *aq_hw);

/* set rss redirection write enable */
void hw_atl2_rpf_rss_redir_wr_en_set(struct aq_hw_s *aq_hw, u32 rss_redir_wr_en);

/* set tpo to rpf system loopback */
void hw_atl2_rpf_tpo_to_rpf_sys_lbk_set(struct aq_hw_s *aq_hw,
				       u32 tpo_to_rpf_sys_lbk);

/* set vlan inner ethertype */
void hw_atl2_rpf_vlan_inner_etht_set(struct aq_hw_s *aq_hw, u32 vlan_inner_etht);

/* set vlan outer ethertype */
void hw_atl2_rpf_vlan_outer_etht_set(struct aq_hw_s *aq_hw, u32 vlan_outer_etht);

/* set vlan promiscuous mode enable */
void hw_atl2_rpf_vlan_prom_mode_en_set(struct aq_hw_s *aq_hw,
				      u32 vlan_prom_mode_en);
/* get vlan promiscuous mode enable */
u32 hw_atl2_rpf_vlan_prom_mode_en_get(struct aq_hw_s *aq_hw);

/* Set VLAN untagged action */
void hw_atl2_rpf_vlan_untagged_act_set(struct aq_hw_s *aq_hw,
				      u32 vlan_untagged_act);

/* Set VLAN accept untagged packets */
void hw_atl2_rpf_vlan_accept_untagged_packets_set(struct aq_hw_s *aq_hw,
						 u32 vlan_acc_untagged_packets);

/* Set VLAN filter enable */
void hw_atl2_rpf_vlan_flr_en_set(struct aq_hw_s *aq_hw, u32 vlan_flr_en,
				u32 filter);

/* Set VLAN Filter Action */
void hw_atl2_rpf_vlan_flr_act_set(struct aq_hw_s *aq_hw, u32 vlan_filter_act,
				 u32 filter);

/* Set VLAN ID Filter */
void hw_atl2_rpf_vlan_id_flr_set(struct aq_hw_s *aq_hw, u32 vlan_id_flr,
				u32 filter);

/* Set VLAN RX queue assignment enable */
void hw_atl2_rpf_vlan_rxq_en_flr_set(struct aq_hw_s *aq_hw, u32 vlan_rxq_en,
				u32 filter);

/* Set VLAN RX queue */
void hw_atl2_rpf_vlan_rxq_flr_set(struct aq_hw_s *aq_hw, u32 vlan_rxq,
				u32 filter);

/* Set VLAN filter tag */
void hw_atl2_rpf_vlan_flr_tag_set(struct aq_hw_s *aq_hw, u32 tag, u32 filter);

/* set ethertype filter enable */
void hw_atl2_rpf_etht_flr_en_set(struct aq_hw_s *aq_hw, u32 etht_flr_en,
				u32 filter);

/* set  ethertype user-priority enable */
void hw_atl2_rpf_etht_user_priority_en_set(struct aq_hw_s *aq_hw,
					  u32 etht_user_priority_en,
					  u32 filter);

/* set  ethertype rx queue enable */
void hw_atl2_rpf_etht_rx_queue_en_set(struct aq_hw_s *aq_hw,
				     u32 etht_rx_queue_en,
				     u32 filter);

/* set ethertype rx queue */
void hw_atl2_rpf_etht_rx_queue_set(struct aq_hw_s *aq_hw, u32 etht_rx_queue,
				  u32 filter);

/* set ethertype user-priority */
void hw_atl2_rpf_etht_user_priority_set(struct aq_hw_s *aq_hw,
				       u32 etht_user_priority,
				       u32 filter);

/* set ethertype management queue */
void hw_atl2_rpf_etht_mgt_queue_set(struct aq_hw_s *aq_hw, u32 etht_mgt_queue,
				   u32 filter);

/* set ethertype filter action */
void hw_atl2_rpf_etht_flr_act_set(struct aq_hw_s *aq_hw, u32 etht_flr_act,
				 u32 filter);

/* set ethertype filter */
void hw_atl2_rpf_etht_flr_set(struct aq_hw_s *aq_hw, u32 etht_flr, u32 filter);

/* set ethertype filter tag */
void hw_atl2_rpf_etht_flr_tag_set(struct aq_hw_s *aq_hw, u32 tag, u32 filter);

/* set L3/L4 filter enable */
void hw_atl2_rpf_l3_l4_enf_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L3 IPv6 enable */
void hw_atl2_rpf_l3_v6_enf_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L3 source address enable */
void hw_atl2_rpf_l3_saf_en_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L3 destination address enable */
void hw_atl2_rpf_l3_daf_en_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L4 source port enable */
void hw_atl2_rpf_l4_spf_en_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L4 destination port enable */
void hw_atl2_rpf_l4_dpf_en_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L4 protocol enable */
void hw_atl2_rpf_l4_protf_en_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L3 ARP filter enable */
void hw_atl2_rpf_l3_arpf_en_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L3/L4 rx queue enable */
void hw_atl2_rpf_l3_l4_rxqf_en_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L3/L4 management queue */
void hw_atl2_rpf_l3_l4_mng_rxqf_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L3/L4 filter action */
void hw_atl2_rpf_l3_l4_actf_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L3/L4 rx queue */
void hw_atl2_rpf_l3_l4_rxqf_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L3 v4 dest address */
void hw_atl2_rpf_l3_v4_dest_addr_set(struct aq_hw_s *aq_hw, u32 filter, u32 val);

/* set L3 v4 src address */
void hw_atl2_rpf_l3_v4_src_addr_set(struct aq_hw_s *aq_hw, u32 filter, u32 val);

/* set L3 v4 cmd */
void hw_atl2_rpf_l3_v4_cmd_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L3 v6 dest address */
void hw_atl2_rpf_l3_v6_dest_addr_set(struct aq_hw_s *aq_hw, u8 location,
				      u32 *ipv6_dst);

/* set L3 v6 src address */
void hw_atl2_rpf_l3_v6_src_addr_set(struct aq_hw_s *aq_hw, u8 location,
				      u32 *ipv6_src);

/* set L3 v6 cmd */
void hw_atl2_rpf_l3_v6_cmd_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L3 v6 v4 select */
void hw_atl2_rpf_l3_v6_v4_select_set(struct aq_hw_s *aq_hw, u32 val);

/* set L3 v4 tag */
void hw_atl2_rpf_l3_v4_tag_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L3 v6 tag */
void hw_atl2_rpf_l3_v6_tag_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L4 cmd */
void hw_atl2_rpf_l4_cmd_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L4 tag */
void hw_atl2_rpf_l4_tag_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L4 protocol value */
void hw_atl2_rpf_l4_protf_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L4 source port */
void hw_atl2_rpf_l4_spd_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

/* set L4 destination port */
void hw_atl2_rpf_l4_dpd_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);


void hw_atl2_rpf_flex_en_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

void hw_atl2_rpf_flex_rxqen_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

void hw_atl2_rpf_flex_tag_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

void hw_atl2_rpf_flex_rxqf_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

void hw_atl2_rpf_flex_mngrxq_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

void hw_atl2_rpf_flex_act_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

void hw_atl2_rpf_flex_byte_a_loc_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

void hw_atl2_rpf_flex_byte_b_loc_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

void hw_atl2_rpf_flex_byte_a_msk_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

void hw_atl2_rpf_flex_byte_b_msk_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

void hw_atl2_rpf_flex_byte_a_pat_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

void hw_atl2_rpf_flex_byte_b_pat_set(struct aq_hw_s *aq_hw, u32 val, u32 filter);

void hw_atl2_rpf_flex_mask_word_set(struct aq_hw_s *aq_hw, u32 val, u32 filter, u32 word);

void hw_atl2_rpf_flex_bytepat_sel_set(struct aq_hw_s *aq_hw, u32 val);

void hw_atl2_rpf_flex_bytepat_wr_en_set(struct aq_hw_s *aq_hw, u32 val);

void hw_atl2_rpf_flex_bytepat_addr_set(struct aq_hw_s *aq_hw, u32 val);

void hw_atl2_rpf_flex_bytepat_data_set(struct aq_hw_s *aq_hw, u32 val);

void hw_atl2_rpf_flex_byte_mask_len_set(struct aq_hw_s *aq_hw, u32 val);

/* rpo */

/* set ipv4 header checksum offload enable */
void hw_atl2_rpo_ipv4header_crc_offload_en_set(struct aq_hw_s *aq_hw,
					      u32 ipv4header_crc_offload_en);

/* set rx descriptor vlan stripping */
void hw_atl2_rpo_rx_desc_vlan_stripping_set(struct aq_hw_s *aq_hw,
					   u32 rx_desc_vlan_stripping,
					   u32 descriptor);

void hw_atl2_rpo_outer_vlan_tag_mode_set(struct aq_hw_s *aq_hw,
					uint32_t outervlantagmode);

uint32_t hw_atl2_rpo_outer_vlan_tag_mode_get(struct aq_hw_s *aq_hw);

/* set tcp/udp checksum offload enable */
void hw_atl2_rpo_tcp_udp_crc_offload_en_set(struct aq_hw_s *aq_hw,
					   u32 tcp_udp_crc_offload_en);

/* Set LRO Patch Optimization Enable. */
void hw_atl2_rpo_lro_patch_optimization_en_set(struct aq_hw_s *aq_hw,
					      u32 lro_patch_optimization_en);

/* Set Large Receive Offload Enable */
void hw_atl2_rpo_lro_en_set(struct aq_hw_s *aq_hw, u32 lro_en);

/* Set LRO Q Sessions Limit */
void hw_atl2_rpo_lro_qsessions_lim_set(struct aq_hw_s *aq_hw,
				      u32 lro_qsessions_lim);

/* Set LRO Total Descriptor Limit */
void hw_atl2_rpo_lro_total_desc_lim_set(struct aq_hw_s *aq_hw,
				       u32 lro_total_desc_lim);

/* Set LRO Min Payload of First Packet */
void hw_atl2_rpo_lro_min_pay_of_first_pkt_set(struct aq_hw_s *aq_hw,
					     u32 lro_min_pld_of_first_pkt);

/* Set LRO Packet Limit */
void hw_atl2_rpo_lro_pkt_lim_set(struct aq_hw_s *aq_hw, u32 lro_packet_lim);

/* Set LRO Max Number of Descriptors */
void hw_atl2_rpo_lro_max_num_of_descriptors_set(struct aq_hw_s *aq_hw,
					       u32 lro_max_desc_num, u32 lro);

/* Set LRO Time Base Divider */
void hw_atl2_rpo_lro_time_base_divider_set(struct aq_hw_s *aq_hw,
					  u32 lro_time_base_divider);

/*Set LRO Inactive Interval */
void hw_atl2_rpo_lro_inactive_interval_set(struct aq_hw_s *aq_hw,
					  u32 lro_inactive_interval);

/*Set LRO Max Coalescing Interval */
void hw_atl2_rpo_lro_max_coalescing_interval_set(struct aq_hw_s *aq_hw,
						u32 lro_max_coal_interval);

/* rx */

/* set rx register reset disable */
void hw_atl2_rx_rx_reg_res_dis_set(struct aq_hw_s *aq_hw, u32 rx_reg_res_dis);

/* mpi */
void hw_atl2_mpi_reg_res_dis_set(struct aq_hw_s *aq_hw, u32 rx_reg_res_dis);
/* tdm */

/* set cpu id */
void hw_atl2_tdm_cpu_id_set(struct aq_hw_s *aq_hw, u32 cpuid, u32 dca);

/* set large send offload enable */
void hw_atl2_tdm_large_send_offload_en_set(struct aq_hw_s *aq_hw,
					  u32 large_send_offload_en);

/* set tx descriptor enable */
void hw_atl2_tdm_tx_desc_en_set(struct aq_hw_s *aq_hw, u32 tx_desc_en,
			       u32 descriptor);

/* set tx dca enable */
void hw_atl2_tdm_tx_dca_en_set(struct aq_hw_s *aq_hw, u32 tx_dca_en);

/* set tx dca mode */
void hw_atl2_tdm_tx_dca_mode_set(struct aq_hw_s *aq_hw, u32 tx_dca_mode);

/* set tx descriptor dca enable */
void hw_atl2_tdm_tx_desc_dca_en_set(struct aq_hw_s *aq_hw, u32 tx_desc_dca_en,
				   u32 dca);

/* get tx descriptor head pointer */
u32 hw_atl2_tdm_tx_desc_head_ptr_get(struct aq_hw_s *aq_hw, u32 descriptor);

/* set tx descriptor length */
void hw_atl2_tdm_tx_desc_len_set(struct aq_hw_s *aq_hw, u32 tx_desc_len,
				u32 descriptor);

/* set tx descriptor write-back interrupt enable */
void hw_atl2_tdm_tx_desc_wr_wb_irq_en_set(struct aq_hw_s *aq_hw,
					 u32 tx_desc_wr_wb_irq_en);

/* set tx descriptor write-back threshold */
void hw_atl2_tdm_tx_desc_wr_wb_threshold_set(struct aq_hw_s *aq_hw,
					    u32 tx_desc_wr_wb_threshold,
				     u32 descriptor);

/* Set TDM Interrupt Moderation Enable */
void hw_atl2_tdm_tdm_intr_moder_en_set(struct aq_hw_s *aq_hw,
				      u32 tdm_irq_moderation_en);
/* thm */

/* set lso tcp flag of first packet */
void hw_atl2_thm_lso_tcp_flag_of_first_pkt_set(struct aq_hw_s *aq_hw,
					      u32 lso_tcp_flag_of_first_pkt);

/* set lso tcp flag of last packet */
void hw_atl2_thm_lso_tcp_flag_of_last_pkt_set(struct aq_hw_s *aq_hw,
					     u32 lso_tcp_flag_of_last_pkt);

/* set lso tcp flag of middle packet */
void hw_atl2_thm_lso_tcp_flag_of_middle_pkt_set(struct aq_hw_s *aq_hw,
					       u32 lso_tcp_flag_of_middle_pkt);

/* tpb */

/* set TX Traffic Class Mode */
void hw_atl2_rpb_tps_tx_tc_mode_set(struct aq_hw_s *aq_hw, u32 tx_traf_class_mode);

/* get TX Traffic Class Mode */
u32 hw_atl2_rpb_tps_tx_tc_mode_get(struct aq_hw_s *aq_hw);

/* set tx buffer enable */
void hw_atl2_tpb_tx_buff_en_set(struct aq_hw_s *aq_hw, u32 tx_buff_en);

void hw_atl2_tpb_tx_flex_map_en_set(struct aq_hw_s *aq_hw, u32 en);
void hw_atl2_tpb_tx_flex_map_set(struct aq_hw_s *aq_hw, u32 q, u32 tc);


/* set tx buffer high threshold (per tc) */
void hw_atl2_tpb_tx_buff_hi_threshold_per_tc_set(struct aq_hw_s *aq_hw,
						u32 tx_buff_hi_threshold_per_tc,
					 u32 buffer);

/* set tx buffer low threshold (per tc) */
void hw_atl2_tpb_tx_buff_lo_threshold_per_tc_set(struct aq_hw_s *aq_hw,
						u32 tx_buff_lo_threshold_per_tc,
					 u32 buffer);

/* set tx dma system loopback enable */
void hw_atl2_tpb_tx_dma_sys_lbk_en_set(struct aq_hw_s *aq_hw, u32 tx_dma_sys_lbk_en);

/* set tx dma network loopback enable */
void hw_atl2_tpb_tx_dma_net_lbk_en_set(struct aq_hw_s *aq_hw, u32 tx_dma_net_lbk_en);

/* set tx packet buffer size (per tc) */
void hw_atl2_tpb_tx_pkt_buff_size_per_tc_set(struct aq_hw_s *aq_hw,
					    u32 tx_pkt_buff_size_per_tc, u32 buffer);

void hw_atl2_tpb_set_high_priority(struct aq_hw_s *aq_hw, u32 buffer);
/* set tx path pad insert enable */
void hw_atl2_tpb_tx_path_scp_ins_en_set(struct aq_hw_s *aq_hw, u32 tx_path_scp_ins_en);

void hw_atl2_tpb_tx_packet_scheduler_highest_priority_tc_enable_set(struct aq_hw_s *aq_hw, uint32_t txPacketSchedulerHighestPriorityTcEnable);
uint32_t hw_atl2_tpb_tx_packet_scheduler_highest_priority_tc_enable_get(struct aq_hw_s *aq_hw);

void hw_atl2_tpb_tx_packet_scheduler_highest_priority_tc_set(struct aq_hw_s *aq_hw, uint32_t txPacketSchedulerHighestPriorityTc);
uint32_t hw_atl2_tpb_tx_packet_scheduler_highest_priority_tc_get(struct aq_hw_s *aq_hw);

void hw_atl2_tpb_tx_high_prio_avb_packet_length_comparison_enable_set(struct aq_hw_s *aq_hw, uint32_t highPrioAvbPacketLengthComparisonEnable);
uint32_t hw_atl2_tpb_tx_high_prio_avb_packet_length_comparison_enable_get(struct aq_hw_s *aq_hw);

void hw_atl2_tpb_tx_enable_tx_packet_scheduler_avb_high_priority1_tc_set(struct aq_hw_s *aq_hw, uint32_t enableTxPacketSchedulerAvbHighPriority1Tc);
uint32_t hw_atl2_tpb_tx_enable_tx_packet_scheduler_avb_high_priority1_tc_get(struct aq_hw_s *aq_hw);

void hw_atl2_tpb_tx_packet_scheduler_avb_high_priority1_tc_set(struct aq_hw_s *aq_hw, uint32_t txPacketSchedulerAvbHighPriority1Tc);
uint32_t hw_atl2_tpb_tx_packet_scheduler_avb_high_priority1_tc_get(struct aq_hw_s *aq_hw);

void hw_atl2_tpb_tx_enable_tx_packet_scheduler_avb_high_priority0_tc_set(struct aq_hw_s *aq_hw, uint32_t enableTxPacketSchedulerAvbHighPriority0Tc);
uint32_t hw_atl2_tpb_tx_enable_tx_packet_scheduler_avb_high_priority0_tc_get(struct aq_hw_s *aq_hw);

void hw_atl2_tpb_tx_packet_scheduler_avb_high_priority0_tc_set(struct aq_hw_s *aq_hw, uint32_t txPacketSchedulerAvbHighPriority0Tc);
uint32_t hw_atl2_tpb_tx_packet_scheduler_avb_high_priority0_tc_get(struct aq_hw_s *aq_hw);

void hw_atl2_tpb_tx_enable_avb_tcs_set(struct aq_hw_s *aq_hw, uint32_t enableAvbTcs);
uint32_t hw_atl2_tpb_tx_enable_avb_tcs_get(struct aq_hw_s *aq_hw);

/* set tx buffer clock gate enable */
void hw_atl2_tpb_tx_buf_clk_gate_en_set(struct aq_hw_s *aq_hw, u32 clk_gate_en);

/* tpo */

/* set ipv4 header checksum offload enable */
void hw_atl2_tpo_ipv4header_crc_offload_en_set(struct aq_hw_s *aq_hw,
					      u32 ipv4header_crc_offload_en);

/* set tcp/udp checksum offload enable */
void hw_atl2_tpo_tcp_udp_crc_offload_en_set(struct aq_hw_s *aq_hw,
					   u32 tcp_udp_crc_offload_en);

/* set tx pkt system loopback enable */
void hw_atl2_tpo_tx_pkt_sys_lbk_en_set(struct aq_hw_s *aq_hw,
				      u32 tx_pkt_sys_lbk_en);

/* tps */

/* set tx packet scheduler data arbitration mode */
void hw_atl2_tps_tx_pkt_shed_data_arb_mode_set(struct aq_hw_s *aq_hw,
					      u32 tx_pkt_shed_data_arb_mode);

/* set tx packet scheduler descriptor rate current time reset */
void hw_atl2_tps_tx_pkt_shed_desc_rate_curr_time_res_set(struct aq_hw_s *aq_hw,
							u32 curr_time_res);

/* set tx packet scheduler descriptor rate limit */
void hw_atl2_tps_tx_pkt_shed_desc_rate_lim_set(struct aq_hw_s *aq_hw,
					      u32 tx_pkt_shed_desc_rate_lim);

/* set tx packet scheduler descriptor tc arbitration mode */
void hw_atl2_tps_tx_pkt_shed_desc_tc_arb_mode_set(struct aq_hw_s *aq_hw,
						 u32 arb_mode);

/* set tx packet scheduler descriptor tc max credit */
void hw_atl2_tps_tx_pkt_shed_desc_tc_max_credit_set(struct aq_hw_s *aq_hw,
						   u32 max_credit,
					    u32 tc);

/* set tx packet scheduler descriptor tc weight */
void hw_atl2_tps_tx_pkt_shed_desc_tc_weight_set(struct aq_hw_s *aq_hw,
					       u32 tx_pkt_shed_desc_tc_weight,
					u32 tc);

/* set tx packet scheduler descriptor vm arbitration mode */
void hw_atl2_tps_tx_pkt_shed_desc_vm_arb_mode_set(struct aq_hw_s *aq_hw,
						 u32 arb_mode);

/* set tx packet scheduler tc data max credit */
void hw_atl2_tps_tx_pkt_shed_tc_data_max_credit_set(struct aq_hw_s *aq_hw,
						   u32 max_credit,
					    u32 tc);

/* set tx packet scheduler tc data weight */
void hw_atl2_tps_tx_pkt_shed_tc_data_weight_set(struct aq_hw_s *aq_hw,
					       u32 tx_pkt_shed_tc_data_weight,
					u32 tc);

/* tx */

/* set tx register reset disable */
void hw_atl2_tx_tx_reg_res_dis_set(struct aq_hw_s *aq_hw, u32 tx_reg_res_dis);

/* msm */

/* get register access status */
u32 hw_atl2_msm_reg_access_status_get(struct aq_hw_s *aq_hw);

/* set  register address for indirect address */
void hw_atl2_msm_reg_addr_for_indirect_addr_set(struct aq_hw_s *aq_hw,
					       u32 reg_addr_for_indirect_addr);

/* set register read strobe */
void hw_atl2_msm_reg_rd_strobe_set(struct aq_hw_s *aq_hw, u32 reg_rd_strobe);

/* get  register read data */
u32 hw_atl2_msm_reg_rd_data_get(struct aq_hw_s *aq_hw);

/* set  register write data */
void hw_atl2_msm_reg_wr_data_set(struct aq_hw_s *aq_hw, u32 reg_wr_data);

/* set register write strobe */
void hw_atl2_msm_reg_wr_strobe_set(struct aq_hw_s *aq_hw, u32 reg_wr_strobe);

/* pci */

/* set pci register reset disable */
void hw_atl2_pci_pci_reg_res_dis_set(struct aq_hw_s *aq_hw, u32 pci_reg_res_dis);

/* tsg */

void hw_atl2_tsg_clock_en(struct aq_hw_s *aq_hw, u32 clock_sel, u32 clock_enable);

void hw_atl2_tsg_clock_reset(struct aq_hw_s *aq_hw, u32 clock_sel);
u64 hw_atl2_tsg_clock_read(struct aq_hw_s *aq_hw, u32 clock_sel);
void hw_atl2_tsg_clock_set(struct aq_hw_s *aq_hw, u32 clock_sel, u64 ns, u32 fns);
void hw_atl2_tsg_clock_add(struct aq_hw_s *aq_hw, u32 clock_sel, u64 ns, u32 fns);
void hw_atl2_tsg_clock_sub(struct aq_hw_s *aq_hw, u32 clock_sel, u64 ns, u32 fns);

void hw_atl2_tsg_clock_increment_set(struct aq_hw_s *aq_hw, u32 clock_sel, u32 ns, u32 fns);
void hw_atl2_tsg_gpio_input_set(struct aq_hw_s *aq_hw, int on, u32 pin, u32 clock_sel);
void hw_atl2_tsg_gpio_isr_to_host_set(struct aq_hw_s *aq_hw, int on, u32 clock_sel);
void hw_atl2_tsg_ext_isr_to_host_set(struct aq_hw_s *aq_hw, int on);
void hw_atl2_tsg_gpio_clear_status(struct aq_hw_s *aq_hw, u32 clock_sel);
void hw_atl2_tsg_gpio_input_envent_info_get(struct aq_hw_s *aq_hw, u32 clock_sel, u32 *event_count, u64 *event_ts);
void hw_atl2_tsg_ptp_gpio_gen_pulse(struct aq_hw_s *aq_hw, u32 clk_sel, u64 ts, u32 period, u32 hightime);

void hw_atl2_phi_ptm_en(struct aq_hw_s *aq_hw, int on);
void hw_atl2_ptm_isr_to_host_set(struct aq_hw_s *aq_hw, int on);
void hw_atl2_ptm_clear_status(struct aq_hw_s *aq_hw);

void hw_atl2_rx_q_map_to_tc(struct aq_hw_s *aq_hw, u32 q, u32 tc);

/* set uP Force Interrupt */
void hw_atl2_rdm_rx_desc_ts_req_set(struct aq_hw_s *aq_hw, u32 clk_sel,
			       u32 descriptor);
void hw_atl2_tdm_tx_desc_wb_ts_req_set(struct aq_hw_s *aq_hw, bool en,
			       u32 descriptor);
void hw_atl2_tdm_tx_desc_ts_req_set(struct aq_hw_s *aq_hw, bool en,
			       u32 descriptor);
void hw_atl2_tdm_tx_desc_lt_en_set(struct aq_hw_s *aq_hw, bool en,
			       u32 descriptor);
void hw_atl2_init_launchtime(struct aq_hw_s *aq_hw);
void hw_atl2_correct_launchtime(struct aq_hw_s *aq_hw, unsigned int mbps, unsigned int offset);

/* clear ipv4 filter destination address */
void hw_atl2_rpfl3l4_ipv4_dest_addr_clear(struct aq_hw_s *aq_hw, u8 location);

/* clear ipv4 filter source address */
void hw_atl2_rpfl3l4_ipv4_src_addr_clear(struct aq_hw_s *aq_hw, u8 location);

/* clear command for filter l3-l4 */
void hw_atl2_rpfl3l4_cmd_clear(struct aq_hw_s *aq_hw, u8 location);

/* clear ipv6 filter destination address */
void hw_atl2_rpfl3l4_ipv6_dest_addr_clear(struct aq_hw_s *aq_hw, u8 location);

/* clear ipv6 filter source address */
void hw_atl2_rpfl3l4_ipv6_src_addr_clear(struct aq_hw_s *aq_hw, u8 location);

/* set ipv4 filter destination address */
void hw_atl2_rpfl3l4_ipv4_dest_addr_set(struct aq_hw_s *aq_hw, u8 location,
				       u32 ipv4_dest);

/* set ipv4 filter source address */
void hw_atl2_rpfl3l4_ipv4_src_addr_set(struct aq_hw_s *aq_hw, u8 location,
				      u32 ipv4_src);

/* set command for filter l3-l4 */
void hw_atl2_rpfl3l4_cmd_set(struct aq_hw_s *aq_hw, u8 location, u32 cmd);

/* set ipv6 filter source address */
void hw_atl2_rpfl3l4_ipv6_src_addr_set(struct aq_hw_s *aq_hw, u8 location,
				      u32 *ipv6_src);

/* set ipv6 filter destination address */
void hw_atl2_rpfl3l4_ipv6_dest_addr_set(struct aq_hw_s *aq_hw, u8 location,
				       u32 *ipv6_dest);

/* set action resolver record */
void hw_atl2_rpf_act_rslvr_record_set(struct aq_hw_s *aq_hw, u8 location,
				      u32 tag, u32 mask, u32 action);
/* get action resolver tag record */
void hw_atl2_rpf_act_rslvr_record_get(struct aq_hw_s *aq_hw, u8 location,
				      u32 *tag, u32 *mask, u32 *action);

/* set enable action resolver section */
void hw_atl2_rpf_act_rslvr_section_en_set(struct aq_hw_s *aq_hw, u32 sections);

void hw_atl2_rpf_rss_table1_queue_tc0_set(struct aq_hw_s *aq_hw, u32 rssTable1QueueTc3, u32 repeat);
u32 hw_atl2_rpf_rss_table1_queue_tc0_get(struct aq_hw_s *aq_hw, u32 repeat);
void hw_atl2_rpf_rss_table1_queue_tc1_set(struct aq_hw_s *aq_hw, u32 rssTable1QueueTc3, u32 repeat);
u32 hw_atl2_rpf_rss_table1_queue_tc1_get(struct aq_hw_s *aq_hw, u32 repeat);
void hw_atl2_rpf_rss_table1_queue_tc2_set(struct aq_hw_s *aq_hw, u32 rssTable1QueueTc3, u32 repeat);
u32 hw_atl2_rpf_rss_table1_queue_tc2_get(struct aq_hw_s *aq_hw, u32 repeat);
void hw_atl2_rpf_rss_table1_queue_tc3_set(struct aq_hw_s *aq_hw, u32 rssTable1QueueTc3, u32 repeat);
u32 hw_atl2_rpf_rss_table1_queue_tc3_get(struct aq_hw_s *aq_hw, u32 repeat);
void hw_atl2_rpf_rss_table1_queue_tc4_set(struct aq_hw_s *aq_hw, u32 rssTable1QueueTc3, u32 repeat);
u32 hw_atl2_rpf_rss_table1_queue_tc4_get(struct aq_hw_s *aq_hw, u32 repeat);
void hw_atl2_rpf_rss_table1_queue_tc5_set(struct aq_hw_s *aq_hw, u32 rssTable1QueueTc3, u32 repeat);
u32 hw_atl2_rpf_rss_table1_queue_tc5_get(struct aq_hw_s *aq_hw, u32 repeat);
void hw_atl2_rpf_rss_table1_queue_tc6_set(struct aq_hw_s *aq_hw, u32 rssTable1QueueTc3, u32 repeat);
u32 hw_atl2_rpf_rss_table1_queue_tc6_get(struct aq_hw_s *aq_hw, u32 repeat);
void hw_atl2_rpf_rss_table1_queue_tc7_set(struct aq_hw_s *aq_hw, u32 rssTable1QueueTc3, u32 repeat);
u32 hw_atl2_rpf_rss_table1_queue_tc7_get(struct aq_hw_s *aq_hw, u32 repeat);

/* get data from firmware shared input buffer */
void hw_atl2_mif_shared_buf_get(struct aq_hw_s *aq_hw, int offset,
				  u32 *data, int len);

/* set data into firmware shared input buffer */
void hw_atl2_mif_shared_buf_write(struct aq_hw_s *aq_hw, int offset,
				  u32 *data, int len);

/* get data from firmware shared output buffer */
void hw_atl2_mif_shared_buf_read(struct aq_hw_s *aq_hw, int offset,
				  u32 *data, int len);

/* set host finished write shared buffer indication */
void hw_atl2_mif_host_finished_write_set(struct aq_hw_s *aq_hw, u32 finish);

/* get mcp finished read shared buffer indication */
u32 hw_atl2_mif_mcp_finished_read_get(struct aq_hw_s *aq_hw);

/* get mcp boot register */
u32 hw_atl2_mif_mcp_boot_reg_get(struct aq_hw_s *aq_hw);

/* set mcp boot register */
void hw_atl2_mif_mcp_boot_reg_set(struct aq_hw_s *aq_hw, u32 val);

/* get host interrupt request */
u32 hw_atl2_mif_host_req_int_get(struct aq_hw_s *aq_hw);

/* clear host interrupt request */
void hw_atl2_mif_host_req_int_clr(struct aq_hw_s *aq_hw, u32 val);

/* get rbl version */
u32 hw_atl2_mif_mcp_boot_ver_get(struct aq_hw_s *aq_hw);

void hw_atl2_com_ful_reset_tgl(struct aq_hw_s *aq_hw);

/* PTM */
void hw_atl2_pci_cfg_ptm_long_timer_set(struct aq_hw_s *aq_hw, u32 timer);
void hw_atl2_pci_cfg_ptm_en(struct aq_hw_s *aq_hw, bool en);
void hw_atl2_pci_cfg_ptm_enianess_fix(struct aq_hw_s *aq_hw, uint32_t fix);
void hw_atl2_pci_cfg_ptm_manual_trig(struct aq_hw_s *aq_hw);
void hw_atl2_pci_cfg_ptm_req_local_get(struct aq_hw_s *aq_hw, u64 *ts);
void hw_atl2_pci_cfg_ptm_req_t1_get(struct aq_hw_s *aq_hw, u64 *ts);
void hw_atl2_pci_cfg_ptm_req_t1p_get(struct aq_hw_s *aq_hw, u64 *ts);
void hw_atl2_pci_cfg_ptm_req_t4_get(struct aq_hw_s *aq_hw, u64 *ts);
void hw_atl2_pci_cfg_ptm_req_t4p_get(struct aq_hw_s *aq_hw, u64 *ts);
void hw_atl2_pci_cfg_ptm_req_master_get(struct aq_hw_s *aq_hw, u64 *ts);
void hw_atl2_pci_cfg_ptm_req_pdelay_get(struct aq_hw_s *aq_hw, u32 *pdelay);
void hw_atl2_pci_cfg_ptm_req_master1_get(struct aq_hw_s *aq_hw, u64 *ts);
void hw_atl2_pci_cfg_ptm_req_clock_corr_get(struct aq_hw_s *aq_hw, u64 *ts);
void hw_atl2_pci_cfg_ptm_req_tx_lat_get(struct aq_hw_s *aq_hw, u32 *tx_lat);
void hw_atl2_pci_cfg_ptm_req_rx_lat_get(struct aq_hw_s *aq_hw, u32 *rx_lat);

/* Set Timer enable to update PTM clock */
void hw_atl2_timer_enable_to_update_ptm_clock_set(struct aq_hw_s *aq_hw, uint32_t timerEnableToUpdatePtmClock);

/* Get Timer enable to update PTM clock */
uint32_t hw_atl2_timer_enable_to_update_ptm_clock_get(struct aq_hw_s *aq_hw);

/* Set Request to auto update clock */
void hw_atl2_request_to_auto_update_clock_set(struct aq_hw_s *aq_hw, uint32_t requestToAutoUpdateClock);

/* Get Request to auto update clock */
uint32_t hw_atl2_request_to_auto_update_clock_get(struct aq_hw_s *aq_hw);

/* Set Manual Request to update clock */
void hw_atl2_manual_request_to_update_clock_set(struct aq_hw_s *aq_hw, uint32_t manualRequestToUpdateClock);

/* Get Manual Request to update clock */
uint32_t hw_atl2_manual_request_to_update_clock_get(struct aq_hw_s *aq_hw);

/* Get Local clock updated */
uint32_t hw_atl2_local_clock_updated_get(struct aq_hw_s *aq_hw);

/* Get PTM Responder ready */
uint32_t hw_atl2_ptm_responder_ready_get(struct aq_hw_s *aq_hw);

/* Get PTM context valid */
uint32_t hw_atl2_ptm_context_valid_get(struct aq_hw_s *aq_hw);

/* Get Local Clock value LSB */
uint64_t hw_atl2_local_clock_value_get(struct aq_hw_s *aq_hw);

/* Get Local Clock correction value */
uint64_t hw_atl2_local_clock_correction_value_get(struct aq_hw_s *aq_hw);

/* Set PTM Update Counter Value */
void hw_atl2_ptm_update_counter_value_set(struct aq_hw_s *aq_hw, uint32_t ptmUpdateCounterValue);

/* Get PTM Update Counter Value */
uint32_t hw_atl2_ptm_update_counter_value_get(struct aq_hw_s *aq_hw);

/* Get PTM Context ID */
uint32_t hw_atl2_ptm_context_id_get(struct aq_hw_s *aq_hw);

/* Get PTM Sequence ID */
uint32_t hw_atl2_ptm_sequence_id_get(struct aq_hw_s *aq_hw);

/* Get PTM TSG local clock Lower */
uint64_t hw_atl2_ptm_tsg_local_clock_get(struct aq_hw_s *aq_hw);

/* Set PTP TSG local clock Lower */
void hw_atl2_ptp_tsg_local_clock_lower_set(struct aq_hw_s *aq_hw, uint32_t ptpTsgLocalClockLower);

/* Set PTP TSG local clock upper */
void hw_atl2_ptp_tsg_local_clock_upper_set(struct aq_hw_s *aq_hw, uint32_t ptpTsgLocalClockUpper);

/* Get PTP TSG local clock Lower */
uint64_t hw_atl2_ptp_tsg_local_clock_get(struct aq_hw_s *aq_hw);

u32 hw_atl2_sem_act_rslvr_tbl_get(struct aq_hw_s *self);

#endif /* HW_ATL2_LLH_H */
