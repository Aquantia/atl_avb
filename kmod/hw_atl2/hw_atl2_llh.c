/*
 * aQuantia Corporation Network Driver
 * Copyright (C) 2014-2019 aQuantia Corporation. All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

/* File hw_atl2_llh.c: Definitions of bitfield and register access functions for
 * Atlantic registers.
 */

#include "hw_atl2_llh.h"
#include "hw_atl2_llh_internal.h"
#include "../aq_hw_utils.h"

/* reset */
void hw_atl2_dmc_logic_reset_set(struct aq_hw_s *aq_hw, uint32_t dmcLogicReset)
{
    aq_hw_write_reg_bit(aq_hw,  HW_ATL2_LOGIC_RESET_ADR,  HW_ATL2_LOGIC_RESET_MSK,  HW_ATL2_LOGIC_RESET_SHIFT, dmcLogicReset);
}

uint32_t hw_atl2_dmc_logic_reset_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw,  HW_ATL2_LOGIC_RESET_ADR,  HW_ATL2_LOGIC_RESET_MSK,  HW_ATL2_LOGIC_RESET_SHIFT);
}


void hw_atl2_phi_logic_reset_set(struct aq_hw_s *aq_hw, uint32_t phiLogicReset)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_PCI_LOGIC_RESET_ADR, HW_ATL2_PCI_LOGIC_RESET_MSK, HW_ATL2_PCI_LOGIC_RESET_SHIFT, phiLogicReset);
}

uint32_t hw_atl2_phi_logic_reset_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_PCI_LOGIC_RESET_ADR, HW_ATL2_PCI_LOGIC_RESET_MSK, HW_ATL2_PCI_LOGIC_RESET_SHIFT);
}

void hw_atl2_phi_register_reset_set(struct aq_hw_s *aq_hw, uint32_t phiRegisterReset)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_PCI_REGISTER_RESET_ADR, HW_ATL2_PCI_REGISTER_RESET_MSK, HW_ATL2_PCI_REGISTER_RESET_SHIFT, phiRegisterReset);
}

uint32_t hw_atl2_phi_register_reset_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_PCI_REGISTER_RESET_ADR, HW_ATL2_PCI_REGISTER_RESET_MSK, HW_ATL2_PCI_REGISTER_RESET_SHIFT);
}

void hw_atl2_mif_mips_combo_reset(struct aq_hw_s *aq_hw)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_MIF_REGISTER_RESET_ADR, HW_ATL2_MIPS_RESET_PULSE_MSK |
														   HW_ATL2_MIF_AHB_RESET_MSK |
														   HW_ATL2_MIF_AUX_RESET_MSK |
														   HW_ATL2_MIF_LOGIC_RESET_MSK | 
														   HW_ATL2_MIF_REGISTER_RESET_MSK);
}

void hw_atl2_mips_reset_pulse_set(struct aq_hw_s *aq_hw, uint32_t mipsResetPulse)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_MIPS_RESET_PULSE_ADR, HW_ATL2_MIPS_RESET_PULSE_MSK, HW_ATL2_MIPS_RESET_PULSE_SHIFT, mipsResetPulse);
}

uint32_t hw_atl2_mips_reset_pulse_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_MIPS_RESET_PULSE_ADR, HW_ATL2_MIPS_RESET_PULSE_MSK, HW_ATL2_MIPS_RESET_PULSE_SHIFT);
}


void hw_atl2_mif_ahb_reset_set(struct aq_hw_s *aq_hw, uint32_t ahbReset)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_MIF_AHB_RESET_ADR, HW_ATL2_MIF_AHB_RESET_MSK, HW_ATL2_MIF_AHB_RESET_SHIFT, ahbReset);
}

uint32_t hw_atl2_mif_ahb_reset_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_MIF_AHB_RESET_ADR, HW_ATL2_MIF_AHB_RESET_MSK, HW_ATL2_MIF_AHB_RESET_SHIFT);
}


void hw_atl2_mif_aux_reset_set(struct aq_hw_s *aq_hw, uint32_t auxReset)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_MIF_AUX_RESET_ADR, HW_ATL2_MIF_AUX_RESET_MSK, HW_ATL2_MIF_AUX_RESET_SHIFT, auxReset);
}

uint32_t hw_atl2_mif_aux_reset_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_MIF_AUX_RESET_ADR, HW_ATL2_MIF_AUX_RESET_MSK, HW_ATL2_MIF_AUX_RESET_SHIFT);
}


void hw_atl2_mif_logic_reset_set(struct aq_hw_s *aq_hw, uint32_t mifLogicReset)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_MIF_LOGIC_RESET_ADR, HW_ATL2_MIF_LOGIC_RESET_MSK, HW_ATL2_MIF_LOGIC_RESET_SHIFT, mifLogicReset);
}

uint32_t hw_atl2_mif_logic_reset_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_MIF_LOGIC_RESET_ADR, HW_ATL2_MIF_LOGIC_RESET_MSK, HW_ATL2_MIF_LOGIC_RESET_SHIFT);
}


void hw_atl2_mif_register_reset_set(struct aq_hw_s *aq_hw, uint32_t mifRegisterReset)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_MIF_REGISTER_RESET_ADR, HW_ATL2_MIF_REGISTER_RESET_MSK, HW_ATL2_MIF_REGISTER_RESET_SHIFT, mifRegisterReset);
}

uint32_t hw_atl2_mif_register_reset_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_MIF_REGISTER_RESET_ADR, HW_ATL2_MIF_REGISTER_RESET_MSK, HW_ATL2_MIF_REGISTER_RESET_SHIFT);
}


void hw_atl2_mpi_mac_phy_path_reset_set(struct aq_hw_s *aq_hw, uint32_t macPhyPathReset)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_MPI_LOGIC_RESET_ADR, HW_ATL2_MPI_LOGIC_RESET_MSK, HW_ATL2_MPI_LOGIC_RESET_SHIFT, macPhyPathReset);
}

uint32_t hw_atl2_mpi_mac_phy_path_reset_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_MPI_LOGIC_RESET_ADR, HW_ATL2_MPI_LOGIC_RESET_MSK, HW_ATL2_MPI_LOGIC_RESET_SHIFT);
}


void hw_atl2_mpi_mac_phy_register_reset_set(struct aq_hw_s *aq_hw, uint32_t macPhyRegisterReset)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_MPI_REGISTER_RESET_ADR, HW_ATL2_MPI_REGISTER_RESET_MSK, HW_ATL2_MPI_REGISTER_RESET_SHIFT, macPhyRegisterReset);
}

uint32_t hw_atl2_mpi_mac_phy_register_reset_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_MPI_REGISTER_RESET_ADR, HW_ATL2_MPI_REGISTER_RESET_MSK, HW_ATL2_MPI_REGISTER_RESET_SHIFT);
}

void hw_atl2_tx_logic_reset_set(struct aq_hw_s *aq_hw, uint32_t txLogicReset)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_TX_LOGIC_RESET_ADR, HW_ATL2_TX_LOGIC_RESET_MSK, HW_ATL2_TX_LOGIC_RESET_SHIFT, txLogicReset);
}

uint32_t hw_atl2_tx_logic_reset_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_TX_LOGIC_RESET_ADR, HW_ATL2_TX_LOGIC_RESET_MSK, HW_ATL2_TX_LOGIC_RESET_SHIFT);
}


void hw_atl2_tx_register_reset_set(struct aq_hw_s *aq_hw, uint32_t txRegisterReset)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_TX_REGISTER_RESET_ADR, HW_ATL2_TX_REGISTER_RESET_MSK, HW_ATL2_TX_REGISTER_RESET_SHIFT, txRegisterReset);
}

uint32_t hw_atl2_tx_register_reset_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_TX_REGISTER_RESET_ADR, HW_ATL2_TX_REGISTER_RESET_MSK, HW_ATL2_TX_REGISTER_RESET_SHIFT);
}

void hw_atl2_rx_logic_reset_set(struct aq_hw_s *aq_hw, uint32_t rxLogicReset)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_RX_LOGIC_RESET_ADR, HW_ATL2_RX_LOGIC_RESET_MSK, HW_ATL2_RX_LOGIC_RESET_SHIFT, rxLogicReset);
}

uint32_t hw_atl2_rx_logic_reset_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_RX_LOGIC_RESET_ADR, HW_ATL2_RX_LOGIC_RESET_MSK, HW_ATL2_RX_LOGIC_RESET_SHIFT);
}

void hw_atl2_rx_register_reset_set(struct aq_hw_s *aq_hw, uint32_t rxRegisterReset)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_RX_REGISTER_RESET_ADR, HW_ATL2_RX_REGISTER_RESET_MSK, HW_ATL2_RX_REGISTER_RESET_SHIFT, rxRegisterReset);
}

uint32_t hw_atl2_rx_register_reset_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_RX_REGISTER_RESET_ADR, HW_ATL2_RX_REGISTER_RESET_MSK, HW_ATL2_RX_REGISTER_RESET_SHIFT);
}

void hw_atl2_itr_reset_interrupt_set(struct aq_hw_s *aq_hw, uint32_t resetInterrupt)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_ITR_RESET_ADR, HW_ATL2_ITR_RESET_MSK, HW_ATL2_ITR_RESET_SHIFT, resetInterrupt);
}

uint32_t hw_atl2_itr_reset_interrupt_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_ITR_RESET_ADR, HW_ATL2_ITR_RESET_MSK, HW_ATL2_ITR_RESET_SHIFT);
}


void hw_atl2_itr_interrupt_register_reset_disable_set(struct aq_hw_s *aq_hw, uint32_t interruptRegisterResetDisable)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_ITR_REG_RESET_DSBL_ADR, HW_ATL2_ITR_REG_RESET_DSBL_MSK, HW_ATL2_ITR_REG_RESET_DSBL_SHIFT, interruptRegisterResetDisable);
}

uint32_t hw_atl2_itr_interrupt_register_reset_disable_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_ITR_REG_RESET_DSBL_ADR, HW_ATL2_ITR_REG_RESET_DSBL_MSK, HW_ATL2_ITR_REG_RESET_DSBL_SHIFT);
}


/* global */
void hw_atl2_reg_glb_cpu_sem_set(struct aq_hw_s *aq_hw, u32 glb_cpu_sem,
				u32 semaphore)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_GLB_CPU_SEM_ADR(semaphore), glb_cpu_sem);
}

u32 hw_atl2_reg_glb_cpu_sem_get(struct aq_hw_s *aq_hw, u32 semaphore)
{
	return aq_hw_read_reg(aq_hw, HW_ATL2_GLB_CPU_SEM_ADR(semaphore));
}

void hw_atl2_glb_glb_reg_res_dis_set(struct aq_hw_s *aq_hw, u32 glb_reg_res_dis)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_GLB_REG_RES_DIS_ADR,
			    HW_ATL2_GLB_REG_RES_DIS_MSK,
			    HW_ATL2_GLB_REG_RES_DIS_SHIFT,
			    glb_reg_res_dis);
}

void hw_atl2_glb_soft_res_set(struct aq_hw_s *aq_hw, u32 soft_res)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_GLB_SOFT_RES_ADR,
			    HW_ATL2_GLB_SOFT_RES_MSK,
			    HW_ATL2_GLB_SOFT_RES_SHIFT, soft_res);
}

u32 hw_atl2_glb_soft_res_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg_bit(aq_hw, HW_ATL2_GLB_SOFT_RES_ADR,
				  HW_ATL2_GLB_SOFT_RES_MSK,
				  HW_ATL2_GLB_SOFT_RES_SHIFT);
}

u32 hw_atl2_reg_glb_mif_id_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg(aq_hw, HW_ATL2_GLB_MIF_ID_ADR);
}

/* stats */
u32 hw_atl2_rpb_rx_dma_drop_pkt_cnt_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg(aq_hw, HW_ATL2_RPB_RX_DMA_DROP_PKT_CNT_ADR);
}

u32 hw_atl2_stats_rx_dma_good_octet_counterlsw_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg(aq_hw, HW_ATL2_STATS_RX_DMA_GOOD_OCTET_COUNTERLSW);
}

u32 hw_atl2_stats_rx_dma_good_pkt_counterlsw_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg(aq_hw, HW_ATL2_STATS_RX_DMA_GOOD_PKT_COUNTERLSW);
}

u32 hw_atl2_stats_tx_dma_good_octet_counterlsw_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg(aq_hw, HW_ATL2_STATS_TX_DMA_GOOD_OCTET_COUNTERLSW);
}

u32 hw_atl2_stats_tx_dma_good_pkt_counterlsw_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg(aq_hw, HW_ATL2_STATS_TX_DMA_GOOD_PKT_COUNTERLSW);
}

u32 hw_atl2_stats_rx_dma_good_octet_countermsw_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg(aq_hw, HW_ATL2_STATS_RX_DMA_GOOD_OCTET_COUNTERMSW);
}

u32 hw_atl2_stats_rx_dma_good_pkt_countermsw_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg(aq_hw, HW_ATL2_STATS_RX_DMA_GOOD_PKT_COUNTERMSW);
}

u32 hw_atl2_stats_tx_dma_good_octet_countermsw_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg(aq_hw, HW_ATL2_STATS_TX_DMA_GOOD_OCTET_COUNTERMSW);
}

u32 hw_atl2_stats_tx_dma_good_pkt_countermsw_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg(aq_hw, HW_ATL2_STATS_TX_DMA_GOOD_PKT_COUNTERMSW);
}

/* interrupt */
void hw_atl2_itr_irq_auto_masklsw_set(struct aq_hw_s *aq_hw,
				     u32 irq_auto_masklsw)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_ITR_IAMRLSW_ADR, irq_auto_masklsw);
}

void hw_atl2_itr_irq_map_en_rx_set(struct aq_hw_s *aq_hw, u32 irq_map_en_rx,
				  u32 rx)
{
/* register address for bitfield imr_rx{r}_en */
	static u32 itr_imr_rxren_adr[32] = {
			0x00002100U, 0x00002100U, 0x00002104U, 0x00002104U,
			0x00002108U, 0x00002108U, 0x0000210CU, 0x0000210CU,
			0x00002110U, 0x00002110U, 0x00002114U, 0x00002114U,
			0x00002118U, 0x00002118U, 0x0000211CU, 0x0000211CU,
			0x00002120U, 0x00002120U, 0x00002124U, 0x00002124U,
			0x00002128U, 0x00002128U, 0x0000212CU, 0x0000212CU,
			0x00002130U, 0x00002130U, 0x00002134U, 0x00002134U,
			0x00002138U, 0x00002138U, 0x0000213CU, 0x0000213CU
		};

/* bitmask for bitfield imr_rx{r}_en */
	static u32 itr_imr_rxren_msk[32] = {
			0x00008000U, 0x00000080U, 0x00008000U, 0x00000080U,
			0x00008000U, 0x00000080U, 0x00008000U, 0x00000080U,
			0x00008000U, 0x00000080U, 0x00008000U, 0x00000080U,
			0x00008000U, 0x00000080U, 0x00008000U, 0x00000080U,
			0x00008000U, 0x00000080U, 0x00008000U, 0x00000080U,
			0x00008000U, 0x00000080U, 0x00008000U, 0x00000080U,
			0x00008000U, 0x00000080U, 0x00008000U, 0x00000080U,
			0x00008000U, 0x00000080U, 0x00008000U, 0x00000080U
		};

/* lower bit position of bitfield imr_rx{r}_en */
	static u32 itr_imr_rxren_shift[32] = {
			15U, 7U, 15U, 7U, 15U, 7U, 15U, 7U,
			15U, 7U, 15U, 7U, 15U, 7U, 15U, 7U,
			15U, 7U, 15U, 7U, 15U, 7U, 15U, 7U,
			15U, 7U, 15U, 7U, 15U, 7U, 15U, 7U
		};

	aq_hw_write_reg_bit(aq_hw, itr_imr_rxren_adr[rx],
			    itr_imr_rxren_msk[rx],
			    itr_imr_rxren_shift[rx],
			    irq_map_en_rx);
}

void hw_atl2_itr_irq_map_en_tx_set(struct aq_hw_s *aq_hw, u32 irq_map_en_tx,
				  u32 tx)
{
/* register address for bitfield imr_tx{t}_en */
	static u32 itr_imr_txten_adr[32] = {
			0x00002100U, 0x00002100U, 0x00002104U, 0x00002104U,
			0x00002108U, 0x00002108U, 0x0000210CU, 0x0000210CU,
			0x00002110U, 0x00002110U, 0x00002114U, 0x00002114U,
			0x00002118U, 0x00002118U, 0x0000211CU, 0x0000211CU,
			0x00002120U, 0x00002120U, 0x00002124U, 0x00002124U,
			0x00002128U, 0x00002128U, 0x0000212CU, 0x0000212CU,
			0x00002130U, 0x00002130U, 0x00002134U, 0x00002134U,
			0x00002138U, 0x00002138U, 0x0000213CU, 0x0000213CU
		};

/* bitmask for bitfield imr_tx{t}_en */
	static u32 itr_imr_txten_msk[32] = {
			0x80000000U, 0x00800000U, 0x80000000U, 0x00800000U,
			0x80000000U, 0x00800000U, 0x80000000U, 0x00800000U,
			0x80000000U, 0x00800000U, 0x80000000U, 0x00800000U,
			0x80000000U, 0x00800000U, 0x80000000U, 0x00800000U,
			0x80000000U, 0x00800000U, 0x80000000U, 0x00800000U,
			0x80000000U, 0x00800000U, 0x80000000U, 0x00800000U,
			0x80000000U, 0x00800000U, 0x80000000U, 0x00800000U,
			0x80000000U, 0x00800000U, 0x80000000U, 0x00800000U
		};

/* lower bit position of bitfield imr_tx{t}_en */
	static u32 itr_imr_txten_shift[32] = {
			31U, 23U, 31U, 23U, 31U, 23U, 31U, 23U,
			31U, 23U, 31U, 23U, 31U, 23U, 31U, 23U,
			31U, 23U, 31U, 23U, 31U, 23U, 31U, 23U,
			31U, 23U, 31U, 23U, 31U, 23U, 31U, 23U
		};

	aq_hw_write_reg_bit(aq_hw, itr_imr_txten_adr[tx],
			    itr_imr_txten_msk[tx],
			    itr_imr_txten_shift[tx],
			    irq_map_en_tx);
}

void hw_atl2_itr_irq_map_rx_set(struct aq_hw_s *aq_hw, u32 irq_map_rx, u32 rx)
{
/* register address for bitfield imr_rx{r}[4:0] */
	static u32 itr_imr_rxr_adr[32] = {
			0x00002100U, 0x00002100U, 0x00002104U, 0x00002104U,
			0x00002108U, 0x00002108U, 0x0000210CU, 0x0000210CU,
			0x00002110U, 0x00002110U, 0x00002114U, 0x00002114U,
			0x00002118U, 0x00002118U, 0x0000211CU, 0x0000211CU,
			0x00002120U, 0x00002120U, 0x00002124U, 0x00002124U,
			0x00002128U, 0x00002128U, 0x0000212CU, 0x0000212CU,
			0x00002130U, 0x00002130U, 0x00002134U, 0x00002134U,
			0x00002138U, 0x00002138U, 0x0000213CU, 0x0000213CU
		};

/* bitmask for bitfield imr_rx{r}[4:0] */
	static u32 itr_imr_rxr_msk[32] = {
			0x00001f00U, 0x0000001FU, 0x00001F00U, 0x0000001FU,
			0x00001f00U, 0x0000001FU, 0x00001F00U, 0x0000001FU,
			0x00001f00U, 0x0000001FU, 0x00001F00U, 0x0000001FU,
			0x00001f00U, 0x0000001FU, 0x00001F00U, 0x0000001FU,
			0x00001f00U, 0x0000001FU, 0x00001F00U, 0x0000001FU,
			0x00001f00U, 0x0000001FU, 0x00001F00U, 0x0000001FU,
			0x00001f00U, 0x0000001FU, 0x00001F00U, 0x0000001FU,
			0x00001f00U, 0x0000001FU, 0x00001F00U, 0x0000001FU
		};

/* lower bit position of bitfield imr_rx{r}[4:0] */
	static u32 itr_imr_rxr_shift[32] = {
			8U, 0U, 8U, 0U, 8U, 0U, 8U, 0U,
			8U, 0U, 8U, 0U, 8U, 0U, 8U, 0U,
			8U, 0U, 8U, 0U, 8U, 0U, 8U, 0U,
			8U, 0U, 8U, 0U, 8U, 0U, 8U, 0U
		};

	aq_hw_write_reg_bit(aq_hw, itr_imr_rxr_adr[rx],
			    itr_imr_rxr_msk[rx],
			    itr_imr_rxr_shift[rx],
			    irq_map_rx);
}

void hw_atl2_itr_irq_map_tx_set(struct aq_hw_s *aq_hw, u32 irq_map_tx, u32 tx)
{
/* register address for bitfield imr_tx{t}[4:0] */
	static u32 itr_imr_txt_adr[32] = {
			0x00002100U, 0x00002100U, 0x00002104U, 0x00002104U,
			0x00002108U, 0x00002108U, 0x0000210CU, 0x0000210CU,
			0x00002110U, 0x00002110U, 0x00002114U, 0x00002114U,
			0x00002118U, 0x00002118U, 0x0000211CU, 0x0000211CU,
			0x00002120U, 0x00002120U, 0x00002124U, 0x00002124U,
			0x00002128U, 0x00002128U, 0x0000212CU, 0x0000212CU,
			0x00002130U, 0x00002130U, 0x00002134U, 0x00002134U,
			0x00002138U, 0x00002138U, 0x0000213CU, 0x0000213CU
		};

/* bitmask for bitfield imr_tx{t}[4:0] */
	static u32 itr_imr_txt_msk[32] = {
			0x1f000000U, 0x001F0000U, 0x1F000000U, 0x001F0000U,
			0x1f000000U, 0x001F0000U, 0x1F000000U, 0x001F0000U,
			0x1f000000U, 0x001F0000U, 0x1F000000U, 0x001F0000U,
			0x1f000000U, 0x001F0000U, 0x1F000000U, 0x001F0000U,
			0x1f000000U, 0x001F0000U, 0x1F000000U, 0x001F0000U,
			0x1f000000U, 0x001F0000U, 0x1F000000U, 0x001F0000U,
			0x1f000000U, 0x001F0000U, 0x1F000000U, 0x001F0000U,
			0x1f000000U, 0x001F0000U, 0x1F000000U, 0x001F0000U
		};

/* lower bit position of bitfield imr_tx{t}[4:0] */
	static u32 itr_imr_txt_shift[32] = {
			24U, 16U, 24U, 16U, 24U, 16U, 24U, 16U,
			24U, 16U, 24U, 16U, 24U, 16U, 24U, 16U,
			24U, 16U, 24U, 16U, 24U, 16U, 24U, 16U,
			24U, 16U, 24U, 16U, 24U, 16U, 24U, 16U
		};

	aq_hw_write_reg_bit(aq_hw, itr_imr_txt_adr[tx],
			    itr_imr_txt_msk[tx],
			    itr_imr_txt_shift[tx],
			    irq_map_tx);
}

void hw_atl2_itr_irq_msk_clearlsw_set(struct aq_hw_s *aq_hw,
				     u32 irq_msk_clearlsw)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_ITR_IMCRLSW_ADR, irq_msk_clearlsw);
}

void hw_atl2_itr_irq_msk_setlsw_set(struct aq_hw_s *aq_hw, u32 irq_msk_setlsw)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_ITR_IMSRLSW_ADR, irq_msk_setlsw);
}

void hw_atl2_itr_irq_reg_res_dis_set(struct aq_hw_s *aq_hw, u32 irq_reg_res_dis)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_ITR_REG_RES_DSBL_ADR,
			    HW_ATL2_ITR_REG_RES_DSBL_MSK,
			    HW_ATL2_ITR_REG_RES_DSBL_SHIFT, irq_reg_res_dis);
}

void hw_atl2_itr_irq_status_clearlsw_set(struct aq_hw_s *aq_hw,
					u32 irq_status_clearlsw)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_ITR_ISCRLSW_ADR, irq_status_clearlsw);
}

u32 hw_atl2_itr_irq_statuslsw_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg(aq_hw, HW_ATL2_ITR_ISRLSW_ADR);
}

u32 hw_atl2_itr_res_irq_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg_bit(aq_hw, HW_ATL2_ITR_RES_ADR, HW_ATL2_ITR_RES_MSK,
				  HW_ATL2_ITR_RES_SHIFT);
}

void hw_atl2_itr_res_irq_set(struct aq_hw_s *aq_hw, u32 res_irq)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_ITR_RES_ADR, HW_ATL2_ITR_RES_MSK,
			    HW_ATL2_ITR_RES_SHIFT, res_irq);
}

/* rdm */
void hw_atl2_rdm_cpu_id_set(struct aq_hw_s *aq_hw, u32 cpuid, u32 dca)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RDM_DCADCPUID_ADR(dca),
			    HW_ATL2_RDM_DCADCPUID_MSK,
			    HW_ATL2_RDM_DCADCPUID_SHIFT, cpuid);
}

void hw_atl2_rdm_rx_dca_en_set(struct aq_hw_s *aq_hw, u32 rx_dca_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RDM_DCA_EN_ADR, HW_ATL2_RDM_DCA_EN_MSK,
			    HW_ATL2_RDM_DCA_EN_SHIFT, rx_dca_en);
}

void hw_atl2_rdm_rx_dca_mode_set(struct aq_hw_s *aq_hw, u32 rx_dca_mode)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RDM_DCA_MODE_ADR,
			    HW_ATL2_RDM_DCA_MODE_MSK,
			    HW_ATL2_RDM_DCA_MODE_SHIFT, rx_dca_mode);
}

void hw_atl2_rdm_rx_desc_data_buff_size_set(struct aq_hw_s *aq_hw,
					   u32 rx_desc_data_buff_size,
					   u32 descriptor)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RDM_DESCDDATA_SIZE_ADR(descriptor),
			    HW_ATL2_RDM_DESCDDATA_SIZE_MSK,
			    HW_ATL2_RDM_DESCDDATA_SIZE_SHIFT,
			    rx_desc_data_buff_size);
}

void hw_atl2_rdm_rx_desc_dca_en_set(struct aq_hw_s *aq_hw, u32 rx_desc_dca_en,
				   u32 dca)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RDM_DCADDESC_EN_ADR(dca),
			    HW_ATL2_RDM_DCADDESC_EN_MSK,
			    HW_ATL2_RDM_DCADDESC_EN_SHIFT,
			    rx_desc_dca_en);
}

void hw_atl2_rdm_rx_desc_en_set(struct aq_hw_s *aq_hw, u32 rx_desc_en,
			       u32 descriptor)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RDM_DESCDEN_ADR(descriptor),
			    HW_ATL2_RDM_DESCDEN_MSK,
			    HW_ATL2_RDM_DESCDEN_SHIFT,
			    rx_desc_en);
}

void hw_atl2_rdm_rx_desc_head_buff_size_set(struct aq_hw_s *aq_hw,
					   u32 rx_desc_head_buff_size,
					   u32 descriptor)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RDM_DESCDHDR_SIZE_ADR(descriptor),
			    HW_ATL2_RDM_DESCDHDR_SIZE_MSK,
			    HW_ATL2_RDM_DESCDHDR_SIZE_SHIFT,
			    rx_desc_head_buff_size);
}

void hw_atl2_rdm_rx_desc_head_splitting_set(struct aq_hw_s *aq_hw,
					   u32 rx_desc_head_splitting,
					   u32 descriptor)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RDM_DESCDHDR_SPLIT_ADR(descriptor),
			    HW_ATL2_RDM_DESCDHDR_SPLIT_MSK,
			    HW_ATL2_RDM_DESCDHDR_SPLIT_SHIFT,
			    rx_desc_head_splitting);
}

u32 hw_atl2_rdm_rx_desc_head_ptr_get(struct aq_hw_s *aq_hw, u32 descriptor)
{
	return aq_hw_read_reg_bit(aq_hw, HW_ATL2_RDM_DESCDHD_ADR(descriptor),
				  HW_ATL2_RDM_DESCDHD_MSK,
				  HW_ATL2_RDM_DESCDHD_SHIFT);
}

void hw_atl2_rdm_rx_desc_len_set(struct aq_hw_s *aq_hw, u32 rx_desc_len,
				u32 descriptor)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RDM_DESCDLEN_ADR(descriptor),
			    HW_ATL2_RDM_DESCDLEN_MSK, HW_ATL2_RDM_DESCDLEN_SHIFT,
			    rx_desc_len);
}

void hw_atl2_rdm_rx_desc_res_set(struct aq_hw_s *aq_hw, u32 rx_desc_res,
				u32 descriptor)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RDM_DESCDRESET_ADR(descriptor),
			    HW_ATL2_RDM_DESCDRESET_MSK,
			    HW_ATL2_RDM_DESCDRESET_SHIFT,
			    rx_desc_res);
}

void hw_atl2_rdm_rx_desc_wr_wb_irq_en_set(struct aq_hw_s *aq_hw,
					 u32 rx_desc_wr_wb_irq_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RDM_INT_DESC_WRB_EN_ADR,
			    HW_ATL2_RDM_INT_DESC_WRB_EN_MSK,
			    HW_ATL2_RDM_INT_DESC_WRB_EN_SHIFT,
			    rx_desc_wr_wb_irq_en);
}

void hw_atl2_rdm_rx_head_dca_en_set(struct aq_hw_s *aq_hw, u32 rx_head_dca_en,
				   u32 dca)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RDM_DCADHDR_EN_ADR(dca),
			    HW_ATL2_RDM_DCADHDR_EN_MSK,
			    HW_ATL2_RDM_DCADHDR_EN_SHIFT,
			    rx_head_dca_en);
}

void hw_atl2_rdm_rx_pld_dca_en_set(struct aq_hw_s *aq_hw, u32 rx_pld_dca_en,
				  u32 dca)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RDM_DCADPAY_EN_ADR(dca),
			    HW_ATL2_RDM_DCADPAY_EN_MSK,
			    HW_ATL2_RDM_DCADPAY_EN_SHIFT,
			    rx_pld_dca_en);
}

void hw_atl2_rdm_rdm_intr_moder_en_set(struct aq_hw_s *aq_hw,
				      u32 rdm_intr_moder_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RDM_INT_RIM_EN_ADR,
			    HW_ATL2_RDM_INT_RIM_EN_MSK,
			    HW_ATL2_RDM_INT_RIM_EN_SHIFT,
			    rdm_intr_moder_en);
}

/* reg */
void hw_atl2_reg_gen_irq_map_set(struct aq_hw_s *aq_hw, u32 gen_intr_map,
				u32 regidx)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_GEN_INTR_MAP_ADR(regidx), gen_intr_map);
}

u32 hw_atl2_reg_gen_irq_status_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg(aq_hw, HW_ATL2_GEN_INTR_STAT_ADR);
}

void hw_atl2_reg_irq_glb_ctl_set(struct aq_hw_s *aq_hw, u32 intr_glb_ctl)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_INTR_GLB_CTL_ADR, intr_glb_ctl);
}

void hw_atl2_reg_irq_thr_set(struct aq_hw_s *aq_hw, u32 intr_thr, u32 throttle)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_INTR_THR_ADR(throttle), intr_thr);
}

void hw_atl2_reg_rx_dma_desc_base_addresslswset(struct aq_hw_s *aq_hw,
					       u32 rx_dma_desc_base_addrlsw,
					       u32 descriptor)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_RX_DMA_DESC_BASE_ADDRLSW_ADR(descriptor),
			rx_dma_desc_base_addrlsw);
}

void hw_atl2_reg_rx_dma_desc_base_addressmswset(struct aq_hw_s *aq_hw,
					       u32 rx_dma_desc_base_addrmsw,
					       u32 descriptor)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_RX_DMA_DESC_BASE_ADDRMSW_ADR(descriptor),
			rx_dma_desc_base_addrmsw);
}

u32 hw_atl2_reg_rx_dma_desc_status_get(struct aq_hw_s *aq_hw, u32 descriptor)
{
	return aq_hw_read_reg(aq_hw, HW_ATL2_RX_DMA_DESC_STAT_ADR(descriptor));
}

void hw_atl2_reg_rx_dma_desc_tail_ptr_set(struct aq_hw_s *aq_hw,
					 u32 rx_dma_desc_tail_ptr,
					 u32 descriptor)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_RX_DMA_DESC_TAIL_PTR_ADR(descriptor),
			rx_dma_desc_tail_ptr);
}

void hw_atl2_reg_rx_flr_mcst_flr_msk_set(struct aq_hw_s *aq_hw,
					u32 rx_flr_mcst_flr_msk)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_RX_FLR_MCST_FLR_MSK_ADR,
			rx_flr_mcst_flr_msk);
}

void hw_atl2_reg_rx_flr_mcst_flr_set(struct aq_hw_s *aq_hw, u32 rx_flr_mcst_flr,
				    u32 filter)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_RX_FLR_MCST_FLR_ADR(filter),
			rx_flr_mcst_flr);
}

void hw_atl2_reg_rx_flr_rss_control1set(struct aq_hw_s *aq_hw,
				       u32 rx_flr_rss_control1)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_RX_FLR_RSS_CONTROL1_ADR,
			rx_flr_rss_control1);
}

void hw_atl2_reg_rx_flr_rss_hash_type_set(struct aq_hw_s *aq_hw,
				       u32 rx_flr_rss_hash_type)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RX_FLR_RSS_HASH_TYPE_ADR,
			    HW_ATL2_RX_FLR_RSS_HASH_TYPE_MSK,
			    HW_ATL2_RX_FLR_RSS_HASH_TYPE_SHIFT,
			    rx_flr_rss_hash_type);
}

void hw_atl2_reg_rx_flr_control2_set(struct aq_hw_s *aq_hw,
				    u32 rx_filter_control2)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_RX_FLR_CONTROL2_ADR, rx_filter_control2);
}

void hw_atl2_reg_rx_intr_moder_ctrl_set(struct aq_hw_s *aq_hw,
				       u32 rx_intr_moderation_ctl,
				       u32 queue)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_RX_INTR_MODERATION_CTL_ADR(queue),
			rx_intr_moderation_ctl);
}

void hw_atl2_reg_tx_dma_debug_ctl_set(struct aq_hw_s *aq_hw,
				     u32 tx_dma_debug_ctl)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_TX_DMA_DEBUG_CTL_ADR, tx_dma_debug_ctl);
}

void hw_atl2_reg_tx_dma_desc_base_addresslswset(struct aq_hw_s *aq_hw,
					       u32 tx_dma_desc_base_addrlsw,
					       u32 descriptor)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_TX_DMA_DESC_BASE_ADDRLSW_ADR(descriptor),
			tx_dma_desc_base_addrlsw);
}

void hw_atl2_reg_tx_dma_desc_base_addressmswset(struct aq_hw_s *aq_hw,
					       u32 tx_dma_desc_base_addrmsw,
					       u32 descriptor)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_TX_DMA_DESC_BASE_ADDRMSW_ADR(descriptor),
			tx_dma_desc_base_addrmsw);
}

void hw_atl2_reg_tx_dma_desc_tail_ptr_set(struct aq_hw_s *aq_hw,
					 u32 tx_dma_desc_tail_ptr,
					 u32 descriptor)
{
	wmb();

	aq_hw_write_reg(aq_hw, HW_ATL2_TX_DMA_DESC_TAIL_PTR_ADR(descriptor),
			tx_dma_desc_tail_ptr);
}

void hw_atl2_reg_tx_intr_moder_ctrl_set(struct aq_hw_s *aq_hw,
				       u32 tx_intr_moderation_ctl,
				       u32 queue)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_TX_INTR_MODERATION_CTL_ADR(queue),
			tx_intr_moderation_ctl);
}

/* RPB: rx packet buffer */
void hw_atl2_rpb_dma_sys_lbk_set(struct aq_hw_s *aq_hw, u32 dma_sys_lbk)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPB_DMA_SYS_LBK_ADR,
			    HW_ATL2_RPB_DMA_SYS_LBK_MSK,
			    HW_ATL2_RPB_DMA_SYS_LBK_SHIFT, dma_sys_lbk);
}

void hw_atl2_rpb_dma_net_lbk_set(struct aq_hw_s *aq_hw, u32 dma_net_lbk)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPB_DMA_NET_LBK_ADR,
			    HW_ATL2_RPB_DMA_NET_LBK_MSK,
			    HW_ATL2_RPB_DMA_NET_LBK_SHIFT, dma_net_lbk);
}
void hw_atl2_rpb_rpf_rx_traf_class_mode_set(struct aq_hw_s *aq_hw,
					   u32 rx_traf_class_mode)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPB_RPF_RX_TC_MODE_ADR,
			    HW_ATL2_RPB_RPF_RX_TC_MODE_MSK,
			    HW_ATL2_RPB_RPF_RX_TC_MODE_SHIFT,
			    rx_traf_class_mode);
}

u32 hw_atl2_rpb_rpf_rx_traf_class_mode_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg_bit(aq_hw, HW_ATL2_RPB_RPF_RX_TC_MODE_ADR,
			HW_ATL2_RPB_RPF_RX_TC_MODE_MSK,
			HW_ATL2_RPB_RPF_RX_TC_MODE_SHIFT);
}

void hw_atl2_rpb_rx_buff_en_set(struct aq_hw_s *aq_hw, u32 rx_buff_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPB_RX_BUF_EN_ADR,
			    HW_ATL2_RPB_RX_BUF_EN_MSK,
			    HW_ATL2_RPB_RX_BUF_EN_SHIFT, rx_buff_en);
}

void hw_atl2_rpb_rx_buff_hi_threshold_per_tc_set(struct aq_hw_s *aq_hw,
						u32 rx_buff_hi_threshold_per_tc,
						u32 buffer)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPB_RXBHI_THRESH_ADR(buffer),
			    HW_ATL2_RPB_RXBHI_THRESH_MSK,
			    HW_ATL2_RPB_RXBHI_THRESH_SHIFT,
			    rx_buff_hi_threshold_per_tc);
}

void hw_atl2_rpb_rx_buff_lo_threshold_per_tc_set(struct aq_hw_s *aq_hw,
						u32 rx_buff_lo_threshold_per_tc,
						u32 buffer)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPB_RXBLO_THRESH_ADR(buffer),
			    HW_ATL2_RPB_RXBLO_THRESH_MSK,
			    HW_ATL2_RPB_RXBLO_THRESH_SHIFT,
			    rx_buff_lo_threshold_per_tc);
}

void hw_atl2_rpb_rx_flow_ctl_mode_set(struct aq_hw_s *aq_hw, u32 rx_flow_ctl_mode)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPB_RX_FC_MODE_ADR,
			    HW_ATL2_RPB_RX_FC_MODE_MSK,
			    HW_ATL2_RPB_RX_FC_MODE_SHIFT, rx_flow_ctl_mode);
}

u32 hw_atl2_rdm_rx_dma_desc_cache_init_done_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg_bit(aq_hw, HW_ATL2_RDM_RX_DMA_DESC_CACHE_INIT_ADR,
				 HW_ATL2_RDM_RX_DMA_DESC_CACHE_INIT_MSK,
				 HW_ATL2_RDM_RX_DMA_DESC_CACHE_INIT_SHIFT);
}

void hw_atl2_rdm_rx_dma_desc_cache_init_tgl(struct aq_hw_s *aq_hw)
{
	u32 val = hw_atl2_rdm_rx_dma_desc_cache_init_done_get(aq_hw);

 	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RDM_RX_DMA_DESC_CACHE_INIT_ADR,
 			    HW_ATL2_RDM_RX_DMA_DESC_CACHE_INIT_MSK,
 			    HW_ATL2_RDM_RX_DMA_DESC_CACHE_INIT_SHIFT,
			    val ^ 1);
}

void hw_atl2_rpb_rx_pkt_buff_size_per_tc_set(struct aq_hw_s *aq_hw,
					    u32 rx_pkt_buff_size_per_tc, u32 buffer)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPB_RXBBUF_SIZE_ADR(buffer),
			    HW_ATL2_RPB_RXBBUF_SIZE_MSK,
			    HW_ATL2_RPB_RXBBUF_SIZE_SHIFT,
			    rx_pkt_buff_size_per_tc);
}

void hw_atl2_rpb_rx_xoff_en_per_tc_set(struct aq_hw_s *aq_hw, u32 rx_xoff_en_per_tc,
				      u32 buffer)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPB_RXBXOFF_EN_ADR(buffer),
			    HW_ATL2_RPB_RXBXOFF_EN_MSK,
			    HW_ATL2_RPB_RXBXOFF_EN_SHIFT,
			    rx_xoff_en_per_tc);
}

/* rpf */

void hw_atl2_rpf_new_enable_set(struct aq_hw_s *aq_hw, u32 enable)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_NEW_EN_ADR,
			    HW_ATL2_RPF_NEW_EN_MSK,
			    HW_ATL2_RPF_NEW_EN_SHIFT,
			    enable);
}

void hw_atl2_rpfl2broadcast_count_threshold_set(struct aq_hw_s *aq_hw,
					       u32 l2broadcast_count_threshold)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPFL2BC_THRESH_ADR,
			    HW_ATL2_RPFL2BC_THRESH_MSK,
			    HW_ATL2_RPFL2BC_THRESH_SHIFT,
			    l2broadcast_count_threshold);
}

void hw_atl2_rpfl2broadcast_en_set(struct aq_hw_s *aq_hw, u32 l2broadcast_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPFL2BC_EN_ADR, HW_ATL2_RPFL2BC_EN_MSK,
			    HW_ATL2_RPFL2BC_EN_SHIFT, l2broadcast_en);
}

void hw_atl2_rpfl2broadcast_flr_act_set(struct aq_hw_s *aq_hw,
				       u32 l2broadcast_flr_act)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPFL2BC_ACT_ADR,
			    HW_ATL2_RPFL2BC_ACT_MSK,
			    HW_ATL2_RPFL2BC_ACT_SHIFT, l2broadcast_flr_act);
}

void hw_atl2_rpfl2multicast_flr_en_set(struct aq_hw_s *aq_hw,
				      u32 l2multicast_flr_en,
				      u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPFL2MC_ENF_ADR(filter),
			    HW_ATL2_RPFL2MC_ENF_MSK,
			    HW_ATL2_RPFL2MC_ENF_SHIFT, l2multicast_flr_en);
}

void hw_atl2_rpfl2promiscuous_mode_en_set(struct aq_hw_s *aq_hw,
					 u32 l2promiscuous_mode_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPFL2PROMIS_MODE_ADR,
			    HW_ATL2_RPFL2PROMIS_MODE_MSK,
			    HW_ATL2_RPFL2PROMIS_MODE_SHIFT,
			    l2promiscuous_mode_en);
}

u32  hw_atl2_rpfl2promiscuous_mode_en_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg_bit(aq_hw, HW_ATL2_RPFL2PROMIS_MODE_ADR,
				  HW_ATL2_RPFL2PROMIS_MODE_MSK,
				  HW_ATL2_RPFL2PROMIS_MODE_SHIFT);
}

void hw_atl2_rpfl2unicast_flr_act_set(struct aq_hw_s *aq_hw,
				     u32 l2unicast_flr_act,
				     u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPFL2UC_ACTF_ADR(filter),
			    HW_ATL2_RPFL2UC_ACTF_MSK, HW_ATL2_RPFL2UC_ACTF_SHIFT,
			    l2unicast_flr_act);
}

void hw_atl2_rpfl2_uc_flr_en_set(struct aq_hw_s *aq_hw, u32 l2unicast_flr_en,
				u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPFL2UC_ENF_ADR(filter),
			    HW_ATL2_RPFL2UC_ENF_MSK,
			    HW_ATL2_RPFL2UC_ENF_SHIFT, l2unicast_flr_en);
}

void hw_atl2_rpfl2unicast_dest_addresslsw_set(struct aq_hw_s *aq_hw,
					     u32 l2unicast_dest_addresslsw,
					     u32 filter)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_RPFL2UC_DAFLSW_ADR(filter),
			l2unicast_dest_addresslsw);
}

void hw_atl2_rpfl2unicast_dest_addressmsw_set(struct aq_hw_s *aq_hw,
					     u32 l2unicast_dest_addressmsw,
					     u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPFL2UC_DAFMSW_ADR(filter),
			    HW_ATL2_RPFL2UC_DAFMSW_MSK,
			    HW_ATL2_RPFL2UC_DAFMSW_SHIFT,
			    l2unicast_dest_addressmsw);
}

void hw_atl2_rpfl2_uc_flr_tag_set(struct aq_hw_s *aq_hw, u32 tag, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPFL2UC_TAG_ADR(filter),
			    HW_ATL2_RPFL2UC_TAG_MSK,
			    HW_ATL2_RPFL2UC_TAG_SHIFT,
			    tag);
}

void hw_atl2_rpfl2_bc_flr_tag_set(struct aq_hw_s *aq_hw, u32 tag)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L2_BC_TAG_ADR,
			    HW_ATL2_RPF_L2_BC_TAG_MSK,
			    HW_ATL2_RPF_L2_BC_TAG_SHIFT,
			    tag);
}

void hw_atl2_rpfl2_accept_all_mc_packets_set(struct aq_hw_s *aq_hw,
					    u32 l2_accept_all_mc_packets)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPFL2MC_ACCEPT_ALL_ADR,
			    HW_ATL2_RPFL2MC_ACCEPT_ALL_MSK,
			    HW_ATL2_RPFL2MC_ACCEPT_ALL_SHIFT,
			    l2_accept_all_mc_packets);
}

void hw_atl2_rpf_rpb_user_priority_tc_map_set(struct aq_hw_s *aq_hw,
					     u32 user_priority_tc_map, u32 tc)
{
/* register address for bitfield rx_tc_up{t}[2:0] */
	static u32 rpf_rpb_rx_tc_upt_adr[8] = {
			0x000054c4U, 0x000054C4U, 0x000054C4U, 0x000054C4U,
			0x000054c4U, 0x000054C4U, 0x000054C4U, 0x000054C4U
		};

/* bitmask for bitfield rx_tc_up{t}[2:0] */
	static u32 rpf_rpb_rx_tc_upt_msk[8] = {
			0x00000007U, 0x00000070U, 0x00000700U, 0x00007000U,
			0x00070000U, 0x00700000U, 0x07000000U, 0x70000000U
		};

/* lower bit position of bitfield rx_tc_up{t}[2:0] */
	static u32 rpf_rpb_rx_tc_upt_shft[8] = {
			0U, 4U, 8U, 12U, 16U, 20U, 24U, 28U
		};

	aq_hw_write_reg_bit(aq_hw, rpf_rpb_rx_tc_upt_adr[tc],
			    rpf_rpb_rx_tc_upt_msk[tc],
			    rpf_rpb_rx_tc_upt_shft[tc],
			    user_priority_tc_map);
}

void hw_atl2_rpf_rss_key_addr_set(struct aq_hw_s *aq_hw, u32 rss_key_addr)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_RSS_KEY_ADDR_ADR,
			    HW_ATL2_RPF_RSS_KEY_ADDR_MSK,
			    HW_ATL2_RPF_RSS_KEY_ADDR_SHIFT,
			    rss_key_addr);
}

void hw_atl2_rpf_rss_key_wr_data_set(struct aq_hw_s *aq_hw, u32 rss_key_wr_data)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_RPF_RSS_KEY_WR_DATA_ADR,
			rss_key_wr_data);
}

u32 hw_atl2_rpf_rss_key_wr_en_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg_bit(aq_hw, HW_ATL2_RPF_RSS_KEY_WR_ENI_ADR,
				  HW_ATL2_RPF_RSS_KEY_WR_ENI_MSK,
				  HW_ATL2_RPF_RSS_KEY_WR_ENI_SHIFT);
}

void hw_atl2_rpf_rss_key_wr_en_set(struct aq_hw_s *aq_hw, u32 rss_key_wr_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_RSS_KEY_WR_ENI_ADR,
			    HW_ATL2_RPF_RSS_KEY_WR_ENI_MSK,
			    HW_ATL2_RPF_RSS_KEY_WR_ENI_SHIFT,
			    rss_key_wr_en);
}

void hw_atl2_rpf_rss_redir_tbl_addr_set(struct aq_hw_s *aq_hw,
				       u32 rss_redir_tbl_addr)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_RSS_REDIR_ADDR_ADR,
			    HW_ATL2_RPF_RSS_REDIR_ADDR_MSK,
			    HW_ATL2_RPF_RSS_REDIR_ADDR_SHIFT,
			    rss_redir_tbl_addr);
}

void hw_atl2_rpf_rss_redir_tbl_wr_data_set(struct aq_hw_s *aq_hw,
					  u32 rss_redir_tbl_wr_data)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_RSS_REDIR_WR_DATA_ADR,
			    HW_ATL2_RPF_RSS_REDIR_WR_DATA_MSK,
			    HW_ATL2_RPF_RSS_REDIR_WR_DATA_SHIFT,
			    rss_redir_tbl_wr_data);
}

u32 hw_atl2_rpf_rss_redir_wr_en_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg_bit(aq_hw, HW_ATL2_RPF_RSS_REDIR_WR_ENI_ADR,
				  HW_ATL2_RPF_RSS_REDIR_WR_ENI_MSK,
				  HW_ATL2_RPF_RSS_REDIR_WR_ENI_SHIFT);
}

void hw_atl2_rpf_rss_redir_wr_en_set(struct aq_hw_s *aq_hw, u32 rss_redir_wr_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_RSS_REDIR_WR_ENI_ADR,
			    HW_ATL2_RPF_RSS_REDIR_WR_ENI_MSK,
			    HW_ATL2_RPF_RSS_REDIR_WR_ENI_SHIFT, rss_redir_wr_en);
}

void hw_atl2_rpf_tpo_to_rpf_sys_lbk_set(struct aq_hw_s *aq_hw,
				       u32 tpo_to_rpf_sys_lbk)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_TPO_RPF_SYS_LBK_ADR,
			    HW_ATL2_RPF_TPO_RPF_SYS_LBK_MSK,
			    HW_ATL2_RPF_TPO_RPF_SYS_LBK_SHIFT,
			    tpo_to_rpf_sys_lbk);
}

void hw_atl2_rpf_vlan_inner_etht_set(struct aq_hw_s *aq_hw, u32 vlan_inner_etht)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_VL_INNER_TPID_ADR,
			    HW_ATL2_RPF_VL_INNER_TPID_MSK,
			    HW_ATL2_RPF_VL_INNER_TPID_SHIFT,
			    vlan_inner_etht);
}

void hw_atl2_rpf_vlan_outer_etht_set(struct aq_hw_s *aq_hw, u32 vlan_outer_etht)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_VL_OUTER_TPID_ADR,
			    HW_ATL2_RPF_VL_OUTER_TPID_MSK,
			    HW_ATL2_RPF_VL_OUTER_TPID_SHIFT,
			    vlan_outer_etht);
}

void hw_atl2_rpf_vlan_prom_mode_en_set(struct aq_hw_s *aq_hw,
				      u32 vlan_prom_mode_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_VL_PROMIS_MODE_ADR,
			    HW_ATL2_RPF_VL_PROMIS_MODE_MSK,
			    HW_ATL2_RPF_VL_PROMIS_MODE_SHIFT,
			    vlan_prom_mode_en);
}

u32 hw_atl2_rpf_vlan_prom_mode_en_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg_bit(aq_hw, HW_ATL2_RPF_VL_PROMIS_MODE_ADR,
				   HW_ATL2_RPF_VL_PROMIS_MODE_MSK,
				   HW_ATL2_RPF_VL_PROMIS_MODE_SHIFT);
}

void hw_atl2_rpf_vlan_accept_untagged_packets_set(struct aq_hw_s *aq_hw,
						 u32 vlan_acc_untagged_packets)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_VL_ACCEPT_UNTAGGED_MODE_ADR,
			    HW_ATL2_RPF_VL_ACCEPT_UNTAGGED_MODE_MSK,
			    HW_ATL2_RPF_VL_ACCEPT_UNTAGGED_MODE_SHIFT,
			    vlan_acc_untagged_packets);
}

void hw_atl2_rpf_vlan_untagged_act_set(struct aq_hw_s *aq_hw,
				      u32 vlan_untagged_act)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_VL_UNTAGGED_ACT_ADR,
			    HW_ATL2_RPF_VL_UNTAGGED_ACT_MSK,
			    HW_ATL2_RPF_VL_UNTAGGED_ACT_SHIFT,
			    vlan_untagged_act);
}

void hw_atl2_rpf_vlan_flr_en_set(struct aq_hw_s *aq_hw, u32 vlan_flr_en,
				u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_VL_EN_F_ADR(filter),
			    HW_ATL2_RPF_VL_EN_F_MSK,
			    HW_ATL2_RPF_VL_EN_F_SHIFT,
			    vlan_flr_en);
}

void hw_atl2_rpf_vlan_flr_act_set(struct aq_hw_s *aq_hw, u32 vlan_flr_act,
				 u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_VL_ACT_F_ADR(filter),
			    HW_ATL2_RPF_VL_ACT_F_MSK,
			    HW_ATL2_RPF_VL_ACT_F_SHIFT,
			    vlan_flr_act);
}

void hw_atl2_rpf_vlan_id_flr_set(struct aq_hw_s *aq_hw, u32 vlan_id_flr,
				u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_VL_ID_F_ADR(filter),
			    HW_ATL2_RPF_VL_ID_F_MSK,
			    HW_ATL2_RPF_VL_ID_F_SHIFT,
			    vlan_id_flr);
}

void hw_atl2_rpf_vlan_rxq_en_flr_set(struct aq_hw_s *aq_hw, u32 vlan_rxq_en,
				u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_VL_RXQ_EN_F_ADR(filter),
			    HW_ATL2_RPF_VL_RXQ_EN_F_MSK,
			    HW_ATL2_RPF_VL_RXQ_EN_F_SHIFT,
			    vlan_rxq_en);
}

void hw_atl2_rpf_vlan_rxq_flr_set(struct aq_hw_s *aq_hw, u32 vlan_rxq,
				u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_VL_RXQ_F_ADR(filter),
			    HW_ATL2_RPF_VL_RXQ_F_MSK,
			    HW_ATL2_RPF_VL_RXQ_F_SHIFT,
			    vlan_rxq);
};

void hw_atl2_rpf_vlan_flr_tag_set(struct aq_hw_s *aq_hw, u32 tag, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_VL_TAG_ADR(filter),
			    HW_ATL2_RPF_VL_TAG_MSK,
			    HW_ATL2_RPF_VL_TAG_SHIFT,
			    tag);
};

void hw_atl2_rpf_etht_flr_en_set(struct aq_hw_s *aq_hw, u32 etht_flr_en,
				u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_ET_ENF_ADR(filter),
			    HW_ATL2_RPF_ET_ENF_MSK,
			    HW_ATL2_RPF_ET_ENF_SHIFT, etht_flr_en);
}

void hw_atl2_rpf_etht_user_priority_en_set(struct aq_hw_s *aq_hw,
					  u32 etht_user_priority_en, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_ET_UPFEN_ADR(filter),
			    HW_ATL2_RPF_ET_UPFEN_MSK, HW_ATL2_RPF_ET_UPFEN_SHIFT,
			    etht_user_priority_en);
}

void hw_atl2_rpf_etht_rx_queue_en_set(struct aq_hw_s *aq_hw,
				     u32 etht_rx_queue_en,
				     u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_ET_RXQFEN_ADR(filter),
			    HW_ATL2_RPF_ET_RXQFEN_MSK,
			    HW_ATL2_RPF_ET_RXQFEN_SHIFT,
			    etht_rx_queue_en);
}

void hw_atl2_rpf_etht_user_priority_set(struct aq_hw_s *aq_hw,
				       u32 etht_user_priority,
				       u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_ET_UPF_ADR(filter),
			    HW_ATL2_RPF_ET_UPF_MSK,
			    HW_ATL2_RPF_ET_UPF_SHIFT, etht_user_priority);
}

void hw_atl2_rpf_etht_rx_queue_set(struct aq_hw_s *aq_hw, u32 etht_rx_queue,
				  u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_ET_RXQF_ADR(filter),
			    HW_ATL2_RPF_ET_RXQF_MSK,
			    HW_ATL2_RPF_ET_RXQF_SHIFT, etht_rx_queue);
}

void hw_atl2_rpf_etht_mgt_queue_set(struct aq_hw_s *aq_hw, u32 etht_mgt_queue,
				   u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_ET_MNG_RXQF_ADR(filter),
			    HW_ATL2_RPF_ET_MNG_RXQF_MSK,
			    HW_ATL2_RPF_ET_MNG_RXQF_SHIFT,
			    etht_mgt_queue);
}

void hw_atl2_rpf_etht_flr_act_set(struct aq_hw_s *aq_hw, u32 etht_flr_act,
				 u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_ET_ACTF_ADR(filter),
			    HW_ATL2_RPF_ET_ACTF_MSK,
			    HW_ATL2_RPF_ET_ACTF_SHIFT, etht_flr_act);
}

void hw_atl2_rpf_etht_flr_set(struct aq_hw_s *aq_hw, u32 etht_flr, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_ET_VALF_ADR(filter),
			    HW_ATL2_RPF_ET_VALF_MSK,
			    HW_ATL2_RPF_ET_VALF_SHIFT, etht_flr);
}

void hw_atl2_rpf_etht_flr_tag_set(struct aq_hw_s *aq_hw, u32 tag, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_ET_TAG_ADR(filter),
			    HW_ATL2_RPF_ET_TAG_MSK,
			    HW_ATL2_RPF_ET_TAG_SHIFT, tag);
}

void hw_atl2_rpf_l3_l4_enf_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L3_L4_ENF_ADR(filter),
			HW_ATL2_RPF_L3_L4_ENF_MSK,
			HW_ATL2_RPF_L3_L4_ENF_SHIFT, val);
}

void hw_atl2_rpf_l3_v6_enf_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L3_V6_ENF_ADR(filter),
			HW_ATL2_RPF_L3_V6_ENF_MSK,
			HW_ATL2_RPF_L3_V6_ENF_SHIFT, val);
}

void hw_atl2_rpf_l3_saf_en_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L3_SAF_EN_ADR(filter),
			HW_ATL2_RPF_L3_SAF_EN_MSK,
			HW_ATL2_RPF_L3_SAF_EN_SHIFT, val);
}

void hw_atl2_rpf_l3_daf_en_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L3_DAF_EN_ADR(filter),
			HW_ATL2_RPF_L3_DAF_EN_MSK,
			HW_ATL2_RPF_L3_DAF_EN_SHIFT, val);
}

void hw_atl2_rpf_l4_spf_en_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L4_SPF_EN_ADR(filter),
			HW_ATL2_RPF_L4_SPF_EN_MSK,
			HW_ATL2_RPF_L4_SPF_EN_SHIFT, val);
}

void hw_atl2_rpf_l4_dpf_en_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L4_DPF_EN_ADR(filter),
			HW_ATL2_RPF_L4_DPF_EN_MSK,
			HW_ATL2_RPF_L4_DPF_EN_SHIFT, val);
}

void hw_atl2_rpf_l4_protf_en_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L4_PROTF_EN_ADR(filter),
			HW_ATL2_RPF_L4_PROTF_EN_MSK,
			HW_ATL2_RPF_L4_PROTF_EN_SHIFT, val);
}

void hw_atl2_rpf_l3_arpf_en_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L3_ARPF_EN_ADR(filter),
			HW_ATL2_RPF_L3_ARPF_EN_MSK,
			HW_ATL2_RPF_L3_ARPF_EN_SHIFT, val);
}

void hw_atl2_rpf_l3_l4_rxqf_en_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L3_L4_RXQF_EN_ADR(filter),
			HW_ATL2_RPF_L3_L4_RXQF_EN_MSK,
			HW_ATL2_RPF_L3_L4_RXQF_EN_SHIFT, val);
}

void hw_atl2_rpf_l3_l4_mng_rxqf_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L3_L4_MNG_RXQF_ADR(filter),
			HW_ATL2_RPF_L3_L4_MNG_RXQF_MSK,
			HW_ATL2_RPF_L3_L4_MNG_RXQF_SHIFT, val);
}

void hw_atl2_rpf_l3_l4_actf_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L3_L4_ACTF_ADR(filter),
			HW_ATL2_RPF_L3_L4_ACTF_MSK,
			HW_ATL2_RPF_L3_L4_ACTF_SHIFT, val);
}

void hw_atl2_rpf_l3_l4_rxqf_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L3_L4_RXQF_ADR(filter),
			HW_ATL2_RPF_L3_L4_RXQF_MSK,
			HW_ATL2_RPF_L3_L4_RXQF_SHIFT, val);
}

void hw_atl2_rpf_l3_v4_dest_addr_set(struct aq_hw_s *aq_hw, u32 filter, u32 val)
{
	u32 dword = filter % 4;
	u32 addr_set = 6 + ((filter < 4)?(0):(1));
	aq_hw_write_reg(aq_hw,
			HW_ATL2_RPF_L3_DA_DW_ADR(addr_set, dword),
			val);

}

void hw_atl2_rpf_l3_v4_src_addr_set(struct aq_hw_s *aq_hw, u32 filter, u32 val)
{
	u32 dword = filter % 4;
	u32 addr_set = 6 + ((filter < 4)?(0):(1));
	aq_hw_write_reg(aq_hw,
			HW_ATL2_RPF_L3_SA_DW_ADR(addr_set, dword),
			val);
}

void hw_atl2_rpf_l3_v4_cmd_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L3_V4_CMD_ADR(filter),
			HW_ATL2_RPF_L3_V4_CMD_MSK,
			HW_ATL2_RPF_L3_V4_CMD_SHIFT, val);
}

void hw_atl2_rpf_l3_v6_dest_addr_set(struct aq_hw_s *aq_hw, u8 location,
				      u32 *ipv6_dst)
{
	int i;

	for (i = 0; i < 4; ++i)
		aq_hw_write_reg(aq_hw,
				HW_ATL2_RPF_L3_DA_DW_ADR(location, 3 - i),
				ipv6_dst[i]);
}

void hw_atl2_rpf_l3_v6_src_addr_set(struct aq_hw_s *aq_hw, u8 location,
				      u32 *ipv6_src)
{
	int i;

	for (i = 0; i < 4; ++i)
		aq_hw_write_reg(aq_hw,
				HW_ATL2_RPF_L3_SA_DW_ADR(location, 3 - i),
				ipv6_src[i]);
}

void hw_atl2_rpf_l3_v6_cmd_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L3_V6_CMD_ADR(filter),
			HW_ATL2_RPF_L3_V6_CMD_MSK,
			HW_ATL2_RPF_L3_V6_CMD_SHIFT, val);
}

void hw_atl2_rpf_l3_v6_v4_select_set(struct aq_hw_s *aq_hw, u32 val)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L3_V6_V4_SELECT_ADR,
			HW_ATL2_RPF_L3_V6_V4_SELECT_MSK,
			HW_ATL2_RPF_L3_V6_V4_SELECT_SHIFT, val);
}

void hw_atl2_rpf_l3_v4_tag_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L3_V4_TAG_ADR(filter),
			HW_ATL2_RPF_L3_V4_TAG_MSK,
			HW_ATL2_RPF_L3_V4_TAG_SHIFT, val);
}

void hw_atl2_rpf_l3_v6_tag_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L3_V6_TAG_ADR(filter),
			HW_ATL2_RPF_L3_V6_TAG_MSK,
			HW_ATL2_RPF_L3_V6_TAG_SHIFT, val);
}

void hw_atl2_rpf_l4_tag_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L4_TAG_ADR(filter),
			HW_ATL2_RPF_L4_TAG_MSK,
			HW_ATL2_RPF_L4_TAG_SHIFT, val);
}
void hw_atl2_rpf_l4_cmd_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L4_CMD_ADR(filter),
			HW_ATL2_RPF_L4_CMD_MSK,
			HW_ATL2_RPF_L4_CMD_SHIFT, val);
}

void hw_atl2_rpf_l4_protf_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L4_PROTF_ADR(filter),
			HW_ATL2_RPF_L4_PROTF_MSK,
			HW_ATL2_RPF_L4_PROTF_SHIFT, val);
}

void hw_atl2_rpf_l4_spd_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L4_SPD_ADR(filter),
			HW_ATL2_RPF_L4_SPD_MSK,
			HW_ATL2_RPF_L4_SPD_SHIFT, val);
}

void hw_atl2_rpf_l4_dpd_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_L4_DPD_ADR(filter),
			HW_ATL2_RPF_L4_DPD_MSK,
			HW_ATL2_RPF_L4_DPD_SHIFT, val);
}

void hw_atl2_rpf_flex_en_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_FLEX_ENH_ADR(filter),
			HW_ATL2_RPF_FLEX_ENH_MSK,
			HW_ATL2_RPF_FLEX_ENH_SHIFT, val);
}

void hw_atl2_rpf_flex_rxqen_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_FLEX_RXEN_ADR(filter),
			HW_ATL2_RPF_FLEX_RXEN_MSK,
			HW_ATL2_RPF_FLEX_RXEN_SHIFT, val);
}

void hw_atl2_rpf_flex_tag_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_FLEX_TAG_ADR(filter),
			HW_ATL2_RPF_FLEX_TAG_MSK,
			HW_ATL2_RPF_FLEX_TAG_SHIFT, val);
}

void hw_atl2_rpf_flex_rxqf_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_FLEX_RXQH_ADR(filter),
			HW_ATL2_RPF_FLEX_RXQH_MSK,
			HW_ATL2_RPF_FLEX_RXQH_SHIFT, val);
}

void hw_atl2_rpf_flex_mngrxq_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_FLEX_MNGRXQH_ADR(filter),
			HW_ATL2_RPF_FLEX_MNGRXQH_MSK,
			HW_ATL2_RPF_FLEX_MNGRXQH_SHIFT, val);
}

void hw_atl2_rpf_flex_act_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_FLEX_ACTH_ADR(filter),
			HW_ATL2_RPF_FLEX_ACTH_MSK,
			HW_ATL2_RPF_FLEX_ACTH_SHIFT, val);
}

void hw_atl2_rpf_flex_byte_a_loc_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_FLEX_BYTEALOCH_ADR(filter),
			HW_ATL2_RPF_FLEX_BYTEALOCH_MSK,
			HW_ATL2_RPF_FLEX_BYTEALOCH_SHIFT, val);
}

void hw_atl2_rpf_flex_byte_b_loc_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_FLEX_BYTEBLOCH_ADR(filter),
			HW_ATL2_RPF_FLEX_BYTEBLOCH_MSK,
			HW_ATL2_RPF_FLEX_BYTEBLOCH_SHIFT, val);
}

void hw_atl2_rpf_flex_byte_a_msk_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_FLEX_BYTEAMSKH_ADR(filter),
			HW_ATL2_RPF_FLEX_BYTEAMSKH_MSK,
			HW_ATL2_RPF_FLEX_BYTEAMSKH_SHIFT, val);
}
void hw_atl2_rpf_flex_byte_b_msk_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_FLEX_BYTEBMSKH_ADR(filter),
			HW_ATL2_RPF_FLEX_BYTEBMSKH_MSK,
			HW_ATL2_RPF_FLEX_BYTEBMSKH_SHIFT, val);
}

void hw_atl2_rpf_flex_byte_a_pat_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_FLEX_BYTEAPATH_ADR(filter),
			HW_ATL2_RPF_FLEX_BYTEAPATH_MSK,
			HW_ATL2_RPF_FLEX_BYTEAPATH_SHIFT, val);
}

void hw_atl2_rpf_flex_byte_b_pat_set(struct aq_hw_s *aq_hw, u32 val, u32 filter)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_FLEX_BYTEBPATH_ADR(filter),
			HW_ATL2_RPF_FLEX_BYTEBPATH_MSK,
			HW_ATL2_RPF_FLEX_BYTEBPATH_SHIFT, val);
}

void hw_atl2_rpf_flex_mask_word_set(struct aq_hw_s *aq_hw, u32 val, u32 filter, u32 word)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_RPF_FLEX_BYTEMSKHWORDW_ADR(filter) + word * 4, val);
}

void hw_atl2_rpf_flex_bytepat_sel_set(struct aq_hw_s *aq_hw, u32 val)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_FLEX_BYTEPATSEL_ADR,
			HW_ATL2_RPF_FLEX_BYTEPATSEL_MSK,
			HW_ATL2_RPF_FLEX_BYTEPATSEL_SHIFT, val);
}

void hw_atl2_rpf_flex_bytepat_wr_en_set(struct aq_hw_s *aq_hw, u32 val)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_FLEX_BYTEPATWRENI_ADR,
			HW_ATL2_RPF_FLEX_BYTEPATWRENI_MSK,
			HW_ATL2_RPF_FLEX_BYTEPATWRENI_SHIFT, val);
}

void hw_atl2_rpf_flex_bytepat_addr_set(struct aq_hw_s *aq_hw, u32 val)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_FLEX_BYTEPATADDR_ADR,
			HW_ATL2_RPF_FLEX_BYTEPATADDR_MSK,
			HW_ATL2_RPF_FLEX_BYTEPATADDR_SHIFT, val);
}

void hw_atl2_rpf_flex_bytepat_data_set(struct aq_hw_s *aq_hw, u32 val)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_RPF_FLEX_BYTEPATWRDATA_ADR, val);
}

void hw_atl2_rpf_flex_byte_mask_len_set(struct aq_hw_s *aq_hw, u32 val)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_FLEX_FILTERLEN_ADR,
			HW_ATL2_RPF_FLEX_FILTERLEN_MSK,
			HW_ATL2_RPF_FLEX_FILTERLEN_SHIFT, val);
}

void hw_atl2_rpf_rss_table1_queue_tc3_set(struct aq_hw_s *aq_hw, u32 rssTable1QueueTc3, u32 repeat)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_RSS1RED4_ADR(repeat), HW_ATL2_RPF_RSS1RED4_MSK, HW_ATL2_RPF_RSS1RED4_SHIFT, rssTable1QueueTc3);
}

u32 hw_atl2_rpf_rss_table1_queue_tc3_get(struct aq_hw_s *aq_hw, u32 repeat)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_RPF_RSS1RED4_ADR(repeat), HW_ATL2_RPF_RSS1RED4_MSK, HW_ATL2_RPF_RSS1RED4_SHIFT);
}


void hw_atl2_rpf_rss_table1_queue_tc2_set(struct aq_hw_s *aq_hw, u32 rssTable1QueueTc2, u32 repeat)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_RSS1RED3_ADR(repeat), HW_ATL2_RPF_RSS1RED3_MSK, HW_ATL2_RPF_RSS1RED3_SHIFT, rssTable1QueueTc2);
}

u32 hw_atl2_rpf_rss_table1_queue_tc2_get(struct aq_hw_s *aq_hw, u32 repeat)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_RPF_RSS1RED3_ADR(repeat), HW_ATL2_RPF_RSS1RED3_MSK, HW_ATL2_RPF_RSS1RED3_SHIFT);
}


void hw_atl2_rpf_rss_table1_queue_tc1_set(struct aq_hw_s *aq_hw, u32 rssTable1QueueTc1, u32 repeat)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_RSS1RED2_ADR(repeat), HW_ATL2_RPF_RSS1RED2_MSK, HW_ATL2_RPF_RSS1RED2_SHIFT, rssTable1QueueTc1);
}

u32 hw_atl2_rpf_rss_table1_queue_tc1_get(struct aq_hw_s *aq_hw, u32 repeat)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_RPF_RSS1RED2_ADR(repeat), HW_ATL2_RPF_RSS1RED2_MSK, HW_ATL2_RPF_RSS1RED2_SHIFT);
}


void hw_atl2_rpf_rss_table1_queue_tc0_set(struct aq_hw_s *aq_hw, u32 rssTable1QueueTc0, u32 repeat)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_RSS1RED1_ADR(repeat), HW_ATL2_RPF_RSS1RED1_MSK, HW_ATL2_RPF_RSS1RED1_SHIFT, rssTable1QueueTc0);
}

u32 hw_atl2_rpf_rss_table1_queue_tc0_get(struct aq_hw_s *aq_hw, u32 repeat)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_RPF_RSS1RED1_ADR(repeat), HW_ATL2_RPF_RSS1RED1_MSK, HW_ATL2_RPF_RSS1RED1_SHIFT);
}



void hw_atl2_rpf_rss_table2_queue_tc7_set(struct aq_hw_s *aq_hw, u32 rssTable2QueueTc7, u32 repeat)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_RSS2RED4_ADR(repeat), HW_ATL2_RPF_RSS2RED4_MSK, HW_ATL2_RPF_RSS2RED4_SHIFT, rssTable2QueueTc7);
}

u32 hw_atl2_rpf_rss_table2_queue_tc7_get(struct aq_hw_s *aq_hw, u32 repeat)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_RPF_RSS2RED4_ADR(repeat), HW_ATL2_RPF_RSS2RED4_MSK, HW_ATL2_RPF_RSS2RED4_SHIFT);
}


void hw_atl2_rpf_rss_table2_queue_tc6_set(struct aq_hw_s *aq_hw, u32 rssTable2QueueTc6, u32 repeat)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_RSS2RED3_ADR(repeat), HW_ATL2_RPF_RSS2RED3_MSK, HW_ATL2_RPF_RSS2RED3_SHIFT, rssTable2QueueTc6);
}

u32 hw_atl2_rpf_rss_table2_queue_tc6_get(struct aq_hw_s *aq_hw, u32 repeat)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_RPF_RSS2RED3_ADR(repeat), HW_ATL2_RPF_RSS2RED3_MSK, HW_ATL2_RPF_RSS2RED3_SHIFT);
}


void hw_atl2_rpf_rss_table2_queue_tc5_set(struct aq_hw_s *aq_hw, u32 rssTable2QueueTc5, u32 repeat)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_RSS2RED2_ADR(repeat), HW_ATL2_RPF_RSS2RED2_MSK, HW_ATL2_RPF_RSS2RED2_SHIFT, rssTable2QueueTc5);
}

u32 hw_atl2_rpf_rss_table2_queue_tc5_get(struct aq_hw_s *aq_hw, u32 repeat)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_RPF_RSS2RED2_ADR(repeat), HW_ATL2_RPF_RSS2RED2_MSK, HW_ATL2_RPF_RSS2RED2_SHIFT);
}


void hw_atl2_rpf_rss_table2_queue_tc4_set(struct aq_hw_s *aq_hw, u32 rssTable2QueueTc4, u32 repeat)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_RSS2RED1_ADR(repeat), HW_ATL2_RPF_RSS2RED1_MSK, HW_ATL2_RPF_RSS2RED1_SHIFT, rssTable2QueueTc4);
}

u32 hw_atl2_rpf_rss_table2_queue_tc4_get(struct aq_hw_s *aq_hw, u32 repeat)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_RPF_RSS2RED1_ADR(repeat), HW_ATL2_RPF_RSS2RED1_MSK, HW_ATL2_RPF_RSS2RED1_SHIFT);
}

/* RPO: rx packet offload */
void hw_atl2_rpo_ipv4header_crc_offload_en_set(struct aq_hw_s *aq_hw,
					      u32 ipv4header_crc_offload_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPO_IPV4CHK_EN_ADR,
			    HW_ATL2_RPO_IPV4CHK_EN_MSK,
			    HW_ATL2_RPO_IPV4CHK_EN_SHIFT,
			    ipv4header_crc_offload_en);
}

void hw_atl2_rpo_rx_desc_vlan_stripping_set(struct aq_hw_s *aq_hw,
					   u32 rx_desc_vlan_stripping,
					   u32 descriptor)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPO_DESCDVL_STRIP_ADR(descriptor),
			    HW_ATL2_RPO_DESCDVL_STRIP_MSK,
			    HW_ATL2_RPO_DESCDVL_STRIP_SHIFT,
			    rx_desc_vlan_stripping);
}

void hw_atl2_rpo_outer_vlan_tag_mode_set(struct aq_hw_s *aq_hw, u32 outervlantagmode)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPO_OUTER_VL_INS_MODE_ADR,
			    HW_ATL2_RPO_OUTER_VL_INS_MODE_MSK,
			    HW_ATL2_RPO_OUTER_VL_INS_MODE_SHIFT,
			    outervlantagmode);
}

u32 hw_atl2_rpo_outer_vlan_tag_mode_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg_bit(aq_hw, HW_ATL2_RPO_OUTER_VL_INS_MODE_ADR,
				  HW_ATL2_RPO_OUTER_VL_INS_MODE_MSK,
				  HW_ATL2_RPO_OUTER_VL_INS_MODE_SHIFT);
}

void hw_atl2_rpo_tcp_udp_crc_offload_en_set(struct aq_hw_s *aq_hw,
					   u32 tcp_udp_crc_offload_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPOL4CHK_EN_ADR,
			    HW_ATL2_RPOL4CHK_EN_MSK,
			    HW_ATL2_RPOL4CHK_EN_SHIFT, tcp_udp_crc_offload_en);
}

void hw_atl2_rpo_lro_en_set(struct aq_hw_s *aq_hw, u32 lro_en)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_RPO_LRO_EN_ADR, lro_en);
}

void hw_atl2_rpo_lro_patch_optimization_en_set(struct aq_hw_s *aq_hw,
					      u32 lro_patch_optimization_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPO_LRO_PTOPT_EN_ADR,
			    HW_ATL2_RPO_LRO_PTOPT_EN_MSK,
			    HW_ATL2_RPO_LRO_PTOPT_EN_SHIFT,
			    lro_patch_optimization_en);
}

void hw_atl2_rpo_lro_qsessions_lim_set(struct aq_hw_s *aq_hw,
				      u32 lro_qsessions_lim)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPO_LRO_QSES_LMT_ADR,
			    HW_ATL2_RPO_LRO_QSES_LMT_MSK,
			    HW_ATL2_RPO_LRO_QSES_LMT_SHIFT,
			    lro_qsessions_lim);
}

void hw_atl2_rpo_lro_total_desc_lim_set(struct aq_hw_s *aq_hw,
				       u32 lro_total_desc_lim)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPO_LRO_TOT_DSC_LMT_ADR,
			    HW_ATL2_RPO_LRO_TOT_DSC_LMT_MSK,
			    HW_ATL2_RPO_LRO_TOT_DSC_LMT_SHIFT,
			    lro_total_desc_lim);
}

void hw_atl2_rpo_lro_min_pay_of_first_pkt_set(struct aq_hw_s *aq_hw,
					     u32 lro_min_pld_of_first_pkt)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPO_LRO_PKT_MIN_ADR,
			    HW_ATL2_RPO_LRO_PKT_MIN_MSK,
			    HW_ATL2_RPO_LRO_PKT_MIN_SHIFT,
			    lro_min_pld_of_first_pkt);
}

void hw_atl2_rpo_lro_pkt_lim_set(struct aq_hw_s *aq_hw, u32 lro_pkt_lim)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_RPO_LRO_RSC_MAX_ADR, lro_pkt_lim);
}

void hw_atl2_rpo_lro_max_num_of_descriptors_set(struct aq_hw_s *aq_hw,
					       u32 lro_max_number_of_descriptors,
					       u32 lro)
{
/* Register address for bitfield lro{L}_des_max[1:0] */
	static u32 rpo_lro_ldes_max_adr[32] = {
			0x000055A0U, 0x000055A0U, 0x000055A0U, 0x000055A0U,
			0x000055A0U, 0x000055A0U, 0x000055A0U, 0x000055A0U,
			0x000055A4U, 0x000055A4U, 0x000055A4U, 0x000055A4U,
			0x000055A4U, 0x000055A4U, 0x000055A4U, 0x000055A4U,
			0x000055A8U, 0x000055A8U, 0x000055A8U, 0x000055A8U,
			0x000055A8U, 0x000055A8U, 0x000055A8U, 0x000055A8U,
			0x000055ACU, 0x000055ACU, 0x000055ACU, 0x000055ACU,
			0x000055ACU, 0x000055ACU, 0x000055ACU, 0x000055ACU
		};

/* Bitmask for bitfield lro{L}_des_max[1:0] */
	static u32 rpo_lro_ldes_max_msk[32] = {
			0x00000003U, 0x00000030U, 0x00000300U, 0x00003000U,
			0x00030000U, 0x00300000U, 0x03000000U, 0x30000000U,
			0x00000003U, 0x00000030U, 0x00000300U, 0x00003000U,
			0x00030000U, 0x00300000U, 0x03000000U, 0x30000000U,
			0x00000003U, 0x00000030U, 0x00000300U, 0x00003000U,
			0x00030000U, 0x00300000U, 0x03000000U, 0x30000000U,
			0x00000003U, 0x00000030U, 0x00000300U, 0x00003000U,
			0x00030000U, 0x00300000U, 0x03000000U, 0x30000000U
		};

/* Lower bit position of bitfield lro{L}_des_max[1:0] */
	static u32 rpo_lro_ldes_max_shift[32] = {
			0U, 4U, 8U, 12U, 16U, 20U, 24U, 28U,
			0U, 4U, 8U, 12U, 16U, 20U, 24U, 28U,
			0U, 4U, 8U, 12U, 16U, 20U, 24U, 28U,
			0U, 4U, 8U, 12U, 16U, 20U, 24U, 28U
		};

	aq_hw_write_reg_bit(aq_hw, rpo_lro_ldes_max_adr[lro],
			    rpo_lro_ldes_max_msk[lro],
			    rpo_lro_ldes_max_shift[lro],
			    lro_max_number_of_descriptors);
}

void hw_atl2_rpo_lro_time_base_divider_set(struct aq_hw_s *aq_hw,
					  u32 lro_time_base_divider)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPO_LRO_TB_DIV_ADR,
			    HW_ATL2_RPO_LRO_TB_DIV_MSK,
			    HW_ATL2_RPO_LRO_TB_DIV_SHIFT,
			    lro_time_base_divider);
}

void hw_atl2_rpo_lro_inactive_interval_set(struct aq_hw_s *aq_hw,
					  u32 lro_inactive_interval)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPO_LRO_INA_IVAL_ADR,
			    HW_ATL2_RPO_LRO_INA_IVAL_MSK,
			    HW_ATL2_RPO_LRO_INA_IVAL_SHIFT,
			    lro_inactive_interval);
}

void hw_atl2_rpo_lro_max_coalescing_interval_set(struct aq_hw_s *aq_hw,
						u32 lro_max_coal_interval)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPO_LRO_MAX_IVAL_ADR,
			    HW_ATL2_RPO_LRO_MAX_IVAL_MSK,
			    HW_ATL2_RPO_LRO_MAX_IVAL_SHIFT,
			    lro_max_coal_interval);
}

/* rx */
void hw_atl2_rx_rx_reg_res_dis_set(struct aq_hw_s *aq_hw, u32 rx_reg_res_dis)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RX_REG_RES_DSBL_ADR,
			    HW_ATL2_RX_REG_RES_DSBL_MSK,
			    HW_ATL2_RX_REG_RES_DSBL_SHIFT,
			    rx_reg_res_dis);
}

/* MPI */
void hw_atl2_mpi_reg_res_dis_set(struct aq_hw_s *aq_hw, u32 rx_reg_res_dis)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_MPI_CTRL_REG_1_ADR,
			 	HW_ATL2_MPI_REG_RES_DSBL_MSK,
			 	HW_ATL2_MPI_REG_RES_DSBL_SHIFT,
			 	rx_reg_res_dis);
}

/* tdm */
void hw_atl2_tdm_cpu_id_set(struct aq_hw_s *aq_hw, u32 cpuid, u32 dca)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TDM_DCADCPUID_ADR(dca),
			    HW_ATL2_TDM_DCADCPUID_MSK,
			    HW_ATL2_TDM_DCADCPUID_SHIFT, cpuid);
}

void hw_atl2_tdm_large_send_offload_en_set(struct aq_hw_s *aq_hw,
					  u32 large_send_offload_en)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_TDM_LSO_EN_ADR, large_send_offload_en);
}

void hw_atl2_tdm_tx_dca_en_set(struct aq_hw_s *aq_hw, u32 tx_dca_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TDM_DCA_EN_ADR, HW_ATL2_TDM_DCA_EN_MSK,
			    HW_ATL2_TDM_DCA_EN_SHIFT, tx_dca_en);
}

void hw_atl2_tdm_tx_dca_mode_set(struct aq_hw_s *aq_hw, u32 tx_dca_mode)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TDM_DCA_MODE_ADR,
			    HW_ATL2_TDM_DCA_MODE_MSK,
			    HW_ATL2_TDM_DCA_MODE_SHIFT, tx_dca_mode);
}

void hw_atl2_tdm_tx_desc_dca_en_set(struct aq_hw_s *aq_hw, u32 tx_desc_dca_en,
				   u32 dca)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TDM_DCADDESC_EN_ADR(dca),
			    HW_ATL2_TDM_DCADDESC_EN_MSK,
			    HW_ATL2_TDM_DCADDESC_EN_SHIFT,
			    tx_desc_dca_en);
}

void hw_atl2_tdm_tx_desc_en_set(struct aq_hw_s *aq_hw, u32 tx_desc_en,
			       u32 descriptor)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TDM_DESCDEN_ADR(descriptor),
			    HW_ATL2_TDM_DESCDEN_MSK,
			    HW_ATL2_TDM_DESCDEN_SHIFT,
			    tx_desc_en);
}

u32 hw_atl2_tdm_tx_desc_head_ptr_get(struct aq_hw_s *aq_hw, u32 descriptor)
{
	return aq_hw_read_reg_bit(aq_hw, HW_ATL2_TDM_DESCDHD_ADR(descriptor),
				  HW_ATL2_TDM_DESCDHD_MSK,
				  HW_ATL2_TDM_DESCDHD_SHIFT);
}

void hw_atl2_tdm_tx_desc_len_set(struct aq_hw_s *aq_hw, u32 tx_desc_len,
				u32 descriptor)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TDM_DESCDLEN_ADR(descriptor),
			    HW_ATL2_TDM_DESCDLEN_MSK,
			    HW_ATL2_TDM_DESCDLEN_SHIFT,
			    tx_desc_len);
}

void hw_atl2_tdm_tx_desc_wr_wb_irq_en_set(struct aq_hw_s *aq_hw,
					 u32 tx_desc_wr_wb_irq_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TDM_INT_DESC_WRB_EN_ADR,
			    HW_ATL2_TDM_INT_DESC_WRB_EN_MSK,
			    HW_ATL2_TDM_INT_DESC_WRB_EN_SHIFT,
			    tx_desc_wr_wb_irq_en);
}

void hw_atl2_tdm_tx_desc_wr_wb_threshold_set(struct aq_hw_s *aq_hw,
					    u32 tx_desc_wr_wb_threshold,
					    u32 descriptor)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TDM_DESCDWRB_THRESH_ADR(descriptor),
			    HW_ATL2_TDM_DESCDWRB_THRESH_MSK,
			    HW_ATL2_TDM_DESCDWRB_THRESH_SHIFT,
			    tx_desc_wr_wb_threshold);
}

void hw_atl2_tdm_tdm_intr_moder_en_set(struct aq_hw_s *aq_hw,
				      u32 tdm_irq_moderation_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TDM_INT_MOD_EN_ADR,
			    HW_ATL2_TDM_INT_MOD_EN_MSK,
			    HW_ATL2_TDM_INT_MOD_EN_SHIFT,
			    tdm_irq_moderation_en);
}

/* thm */
void hw_atl2_thm_lso_tcp_flag_of_first_pkt_set(struct aq_hw_s *aq_hw,
					      u32 lso_tcp_flag_of_first_pkt)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_THM_LSO_TCP_FLAG_FIRST_ADR,
			    HW_ATL2_THM_LSO_TCP_FLAG_FIRST_MSK,
			    HW_ATL2_THM_LSO_TCP_FLAG_FIRST_SHIFT,
			    lso_tcp_flag_of_first_pkt);
}

void hw_atl2_thm_lso_tcp_flag_of_last_pkt_set(struct aq_hw_s *aq_hw,
					     u32 lso_tcp_flag_of_last_pkt)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_THM_LSO_TCP_FLAG_LAST_ADR,
			    HW_ATL2_THM_LSO_TCP_FLAG_LAST_MSK,
			    HW_ATL2_THM_LSO_TCP_FLAG_LAST_SHIFT,
			    lso_tcp_flag_of_last_pkt);
}

void hw_atl2_thm_lso_tcp_flag_of_middle_pkt_set(struct aq_hw_s *aq_hw,
					       u32 lso_tcp_flag_of_middle_pkt)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_THM_LSO_TCP_FLAG_MID_ADR,
			    HW_ATL2_THM_LSO_TCP_FLAG_MID_MSK,
			    HW_ATL2_THM_LSO_TCP_FLAG_MID_SHIFT,
			    lso_tcp_flag_of_middle_pkt);
}

/* TPB: tx packet buffer */
void hw_atl2_tpb_tx_buff_en_set(struct aq_hw_s *aq_hw, u32 tx_buff_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPB_TX_BUF_EN_ADR,
			    HW_ATL2_TPB_TX_BUF_EN_MSK,
			    HW_ATL2_TPB_TX_BUF_EN_SHIFT, tx_buff_en);
}

void hw_atl2_tpb_tx_flex_map_en_set(struct aq_hw_s *aq_hw, u32 en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPB_TX_FLEX_MODE_ADR,
			    HW_ATL2_TPB_TX_FLEX_MODE_MSK,
			    HW_ATL2_TPB_TX_FLEX_MODE_SHIFT, en);
}

void hw_atl2_tpb_tx_flex_map_set(struct aq_hw_s *aq_hw, u32 q, u32 tc)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPB_FLEX_MAP_ADR(q),
			    HW_ATL2_TPB_FLEX_MAP_MSK(q),
			    HW_ATL2_TPB_FLEX_MAP_SHIFT(q), tc);
}

u32 hw_atl2_rpb_tps_tx_tc_mode_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg_bit(aq_hw, HW_ATL2_TPB_TX_TC_MODE_ADDR,
			HW_ATL2_TPB_TX_TC_MODE_MSK,
			HW_ATL2_TPB_TX_TC_MODE_SHIFT);
}

void hw_atl2_rpb_tps_tx_tc_mode_set(struct aq_hw_s *aq_hw, u32 tx_traf_class_mode)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPB_TX_TC_MODE_ADDR,
			HW_ATL2_TPB_TX_TC_MODE_MSK,
			HW_ATL2_TPB_TX_TC_MODE_SHIFT,
			tx_traf_class_mode);
}

void hw_atl2_tpb_tx_buff_hi_threshold_per_tc_set(struct aq_hw_s *aq_hw,
						u32 tx_buff_hi_threshold_per_tc,
					 u32 buffer)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPB_TXBHI_THRESH_ADR(buffer),
			    HW_ATL2_TPB_TXBHI_THRESH_MSK,
			    HW_ATL2_TPB_TXBHI_THRESH_SHIFT,
			    tx_buff_hi_threshold_per_tc);
}

void hw_atl2_tpb_tx_buff_lo_threshold_per_tc_set(struct aq_hw_s *aq_hw,
						u32 tx_buff_lo_threshold_per_tc,
					 u32 buffer)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPB_TXBLO_THRESH_ADR(buffer),
			    HW_ATL2_TPB_TXBLO_THRESH_MSK,
			    HW_ATL2_TPB_TXBLO_THRESH_SHIFT,
			    tx_buff_lo_threshold_per_tc);
}

void hw_atl2_tpb_tx_dma_sys_lbk_en_set(struct aq_hw_s *aq_hw, u32 tx_dma_sys_lbk_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPB_DMA_SYS_LBK_ADR,
			    HW_ATL2_TPB_DMA_SYS_LBK_MSK,
			    HW_ATL2_TPB_DMA_SYS_LBK_SHIFT,
			    tx_dma_sys_lbk_en);
}

void hw_atl2_tpb_tx_dma_net_lbk_en_set(struct aq_hw_s *aq_hw, u32 tx_dma_net_lbk_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPB_DMA_NET_LBK_ADR,
			    HW_ATL2_TPB_DMA_NET_LBK_MSK,
			    HW_ATL2_TPB_DMA_NET_LBK_SHIFT,
			    tx_dma_net_lbk_en);
}

void hw_atl2_tpb_tx_pkt_buff_size_per_tc_set(struct aq_hw_s *aq_hw,
					    u32 tx_pkt_buff_size_per_tc, u32 buffer)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPB_TXBBUF_SIZE_ADR(buffer),
			    HW_ATL2_TPB_TXBBUF_SIZE_MSK,
			    HW_ATL2_TPB_TXBBUF_SIZE_SHIFT,
			    tx_pkt_buff_size_per_tc);
}

void hw_atl2_tpb_set_high_priority(struct aq_hw_s *aq_hw, u32 buffer)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPS_TC_PRIO_CTRL_ADR,
			    HW_ATL2_TPS_TC_PRIO_CTRL_MSK(buffer),
			    HW_ATL2_TPS_TC_PRIO_CTRL_SHIFT(buffer),
			    1);
}

void hw_atl2_tpb_tx_path_scp_ins_en_set(struct aq_hw_s *aq_hw, u32 tx_path_scp_ins_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPB_TX_SCP_INS_EN_ADR,
			    HW_ATL2_TPB_TX_SCP_INS_EN_MSK,
			    HW_ATL2_TPB_TX_SCP_INS_EN_SHIFT,
			    tx_path_scp_ins_en);
}

void hw_atl2_tpb_tx_buf_clk_gate_en_set(struct aq_hw_s *aq_hw, u32 clk_gate_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPB_TX_BUF_CLK_GATE_EN_ADR,
			    HW_ATL2_TPB_TX_BUF_CLK_GATE_EN_MSK,
			    HW_ATL2_TPB_TX_BUF_CLK_GATE_EN_SHIFT,
			    clk_gate_en);
}

void hw_atl2_tpb_tx_packet_scheduler_highest_priority_tc_enable_set(struct aq_hw_s *aq_hw, uint32_t txPacketSchedulerHighestPriorityTcEnable)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPB_HIGHEST_PRIO_TC_EN_ADR, HW_ATL2_TPB_HIGHEST_PRIO_TC_EN_MSK, HW_ATL2_TPB_HIGHEST_PRIO_TC_EN_SHIFT, txPacketSchedulerHighestPriorityTcEnable);
}

uint32_t hw_atl2_tpb_tx_packet_scheduler_highest_priority_tc_enable_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_TPB_HIGHEST_PRIO_TC_EN_ADR, HW_ATL2_TPB_HIGHEST_PRIO_TC_EN_MSK, HW_ATL2_TPB_HIGHEST_PRIO_TC_EN_SHIFT);
}

void hw_atl2_tpb_tx_packet_scheduler_highest_priority_tc_set(struct aq_hw_s *aq_hw, uint32_t txPacketSchedulerHighestPriorityTc)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPB_HIGHEST_PRIO_TC_ADR, HW_ATL2_TPB_HIGHEST_PRIO_TC_MSK, HW_ATL2_TPB_HIGHEST_PRIO_TC_SHIFT, txPacketSchedulerHighestPriorityTc);
}

uint32_t hw_atl2_tpb_tx_packet_scheduler_highest_priority_tc_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_TPB_HIGHEST_PRIO_TC_ADR, HW_ATL2_TPB_HIGHEST_PRIO_TC_MSK, HW_ATL2_TPB_HIGHEST_PRIO_TC_SHIFT);
}

void hw_atl2_tpb_tx_high_prio_avb_packet_length_comparison_enable_set(struct aq_hw_s *aq_hw, uint32_t highPrioAvbPacketLengthComparisonEnable)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPB_AVB_PKT_LEN_COMP_EN_ADR, HW_ATL2_TPB_AVB_PKT_LEN_COMP_EN_MSK, HW_ATL2_TPB_AVB_PKT_LEN_COMP_EN_SHIFT, highPrioAvbPacketLengthComparisonEnable);
}

uint32_t hw_atl2_tpb_tx_high_prio_avb_packet_length_comparison_enable_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_TPB_AVB_PKT_LEN_COMP_EN_ADR, HW_ATL2_TPB_AVB_PKT_LEN_COMP_EN_MSK, HW_ATL2_TPB_AVB_PKT_LEN_COMP_EN_SHIFT);
}

void hw_atl2_tpb_tx_enable_tx_packet_scheduler_avb_high_priority1_tc_set(struct aq_hw_s *aq_hw, uint32_t enableTxPacketSchedulerAvbHighPriority1Tc)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPB_HIGH_PRIO_AVB_EN1_ADR, HW_ATL2_TPB_HIGH_PRIO_AVB_EN1_MSK, HW_ATL2_TPB_HIGH_PRIO_AVB_EN1_SHIFT, enableTxPacketSchedulerAvbHighPriority1Tc);
}

uint32_t hw_atl2_tpb_tx_enable_tx_packet_scheduler_avb_high_priority1_tc_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_TPB_HIGH_PRIO_AVB_EN1_ADR, HW_ATL2_TPB_HIGH_PRIO_AVB_EN1_MSK, HW_ATL2_TPB_HIGH_PRIO_AVB_EN1_SHIFT);
}

void hw_atl2_tpb_tx_packet_scheduler_avb_high_priority1_tc_set(struct aq_hw_s *aq_hw, uint32_t txPacketSchedulerAvbHighPriority1Tc)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPB_HIGH_PRIO_AVB_TC1_ADR, HW_ATL2_TPB_HIGH_PRIO_AVB_TC1_MSK, HW_ATL2_TPB_HIGH_PRIO_AVB_TC1_SHIFT, txPacketSchedulerAvbHighPriority1Tc);
}

uint32_t hw_atl2_tpb_tx_packet_scheduler_avb_high_priority1_tc_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_TPB_HIGH_PRIO_AVB_TC1_ADR, HW_ATL2_TPB_HIGH_PRIO_AVB_TC1_MSK, HW_ATL2_TPB_HIGH_PRIO_AVB_TC1_SHIFT);
}

void hw_atl2_tpb_tx_enable_tx_packet_scheduler_avb_high_priority0_tc_set(struct aq_hw_s *aq_hw, uint32_t enableTxPacketSchedulerAvbHighPriority0Tc)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPB_HIGH_PRIO_AVB_EN0_ADR, HW_ATL2_TPB_HIGH_PRIO_AVB_EN0_MSK, HW_ATL2_TPB_HIGH_PRIO_AVB_EN0_SHIFT, enableTxPacketSchedulerAvbHighPriority0Tc);
}

uint32_t hw_atl2_tpb_tx_enable_tx_packet_scheduler_avb_high_priority0_tc_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_TPB_HIGH_PRIO_AVB_EN0_ADR, HW_ATL2_TPB_HIGH_PRIO_AVB_EN0_MSK, HW_ATL2_TPB_HIGH_PRIO_AVB_EN0_SHIFT);
}

void hw_atl2_tpb_tx_packet_scheduler_avb_high_priority0_tc_set(struct aq_hw_s *aq_hw, uint32_t txPacketSchedulerAvbHighPriority0Tc)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPB_HIGH_PRIO_AVB_TC0_ADR, HW_ATL2_TPB_HIGH_PRIO_AVB_TC0_MSK, HW_ATL2_TPB_HIGH_PRIO_AVB_TC0_SHIFT, txPacketSchedulerAvbHighPriority0Tc);
}

uint32_t hw_atl2_tpb_tx_packet_scheduler_avb_high_priority0_tc_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_TPB_HIGH_PRIO_AVB_TC0_ADR, HW_ATL2_TPB_HIGH_PRIO_AVB_TC0_MSK, HW_ATL2_TPB_HIGH_PRIO_AVB_TC0_SHIFT);
}

void hw_atl2_tpb_tx_enable_avb_tcs_set(struct aq_hw_s *aq_hw, uint32_t enableAvbTcs)
{
    aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPB_AVB_SCHDL_EN_ADR, HW_ATL2_TPB_AVB_SCHDL_EN_MSK, HW_ATL2_TPB_AVB_SCHDL_EN_SHIFT, enableAvbTcs);
}

uint32_t hw_atl2_tpb_tx_enable_avb_tcs_get(struct aq_hw_s *aq_hw)
{
    return aq_hw_read_reg_bit(aq_hw, HW_ATL2_TPB_AVB_SCHDL_EN_ADR, HW_ATL2_TPB_AVB_SCHDL_EN_MSK, HW_ATL2_TPB_AVB_SCHDL_EN_SHIFT);
}


/* TPO: tx packet offload */
void hw_atl2_tpo_ipv4header_crc_offload_en_set(struct aq_hw_s *aq_hw,
					      u32 ipv4header_crc_offload_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPO_IPV4CHK_EN_ADR,
			    HW_ATL2_TPO_IPV4CHK_EN_MSK,
			    HW_ATL2_TPO_IPV4CHK_EN_SHIFT,
			    ipv4header_crc_offload_en);
}

void hw_atl2_tpo_tcp_udp_crc_offload_en_set(struct aq_hw_s *aq_hw,
					   u32 tcp_udp_crc_offload_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPOL4CHK_EN_ADR,
			    HW_ATL2_TPOL4CHK_EN_MSK,
			    HW_ATL2_TPOL4CHK_EN_SHIFT,
			    tcp_udp_crc_offload_en);
}

void hw_atl2_tpo_tx_pkt_sys_lbk_en_set(struct aq_hw_s *aq_hw,
				      u32 tx_pkt_sys_lbk_en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPO_PKT_SYS_LBK_ADR,
			    HW_ATL2_TPO_PKT_SYS_LBK_MSK,
			    HW_ATL2_TPO_PKT_SYS_LBK_SHIFT,
			    tx_pkt_sys_lbk_en);
}

/* TPS: tx packet scheduler */
void hw_atl2_tps_tx_pkt_shed_data_arb_mode_set(struct aq_hw_s *aq_hw,
					      u32 tx_pkt_shed_data_arb_mode)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPS_DATA_TC_ARB_MODE_ADR,
			    HW_ATL2_TPS_DATA_TC_ARB_MODE_MSK,
			    HW_ATL2_TPS_DATA_TC_ARB_MODE_SHIFT,
			    tx_pkt_shed_data_arb_mode);
}

void hw_atl2_tps_tx_pkt_shed_desc_rate_curr_time_res_set(struct aq_hw_s *aq_hw,
							u32 curr_time_res)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPS_DESC_RATE_TA_RST_ADR,
			    HW_ATL2_TPS_DESC_RATE_TA_RST_MSK,
			    HW_ATL2_TPS_DESC_RATE_TA_RST_SHIFT,
			    curr_time_res);
}

void hw_atl2_tps_tx_pkt_shed_desc_rate_lim_set(struct aq_hw_s *aq_hw,
					      u32 tx_pkt_shed_desc_rate_lim)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPS_DESC_RATE_LIM_ADR,
			    HW_ATL2_TPS_DESC_RATE_LIM_MSK,
			    HW_ATL2_TPS_DESC_RATE_LIM_SHIFT,
			    tx_pkt_shed_desc_rate_lim);
}

void hw_atl2_tps_tx_pkt_shed_desc_tc_arb_mode_set(struct aq_hw_s *aq_hw,
						 u32 arb_mode)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPS_DESC_TC_ARB_MODE_ADR,
			    HW_ATL2_TPS_DESC_TC_ARB_MODE_MSK,
			    HW_ATL2_TPS_DESC_TC_ARB_MODE_SHIFT,
			    arb_mode);
}

void hw_atl2_tps_tx_pkt_shed_desc_tc_max_credit_set(struct aq_hw_s *aq_hw,
						   u32 max_credit,
						   u32 tc)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPS_DESC_TCTCREDIT_MAX_ADR(tc),
			    HW_ATL2_TPS_DESC_TCTCREDIT_MAX_MSK,
			    HW_ATL2_TPS_DESC_TCTCREDIT_MAX_SHIFT,
			    max_credit);
}

void hw_atl2_tps_tx_pkt_shed_desc_tc_weight_set(struct aq_hw_s *aq_hw,
					       u32 tx_pkt_shed_desc_tc_weight,
					       u32 tc)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPS_DESC_TCTWEIGHT_ADR(tc),
			    HW_ATL2_TPS_DESC_TCTWEIGHT_MSK,
			    HW_ATL2_TPS_DESC_TCTWEIGHT_SHIFT,
			    tx_pkt_shed_desc_tc_weight);
}

void hw_atl2_tps_tx_pkt_shed_desc_vm_arb_mode_set(struct aq_hw_s *aq_hw,
						 u32 arb_mode)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPS_DESC_VM_ARB_MODE_ADR,
			    HW_ATL2_TPS_DESC_VM_ARB_MODE_MSK,
			    HW_ATL2_TPS_DESC_VM_ARB_MODE_SHIFT,
			    arb_mode);
}

void hw_atl2_tps_tx_pkt_shed_tc_data_max_credit_set(struct aq_hw_s *aq_hw,
						   u32 max_credit,
						   u32 tc)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPS_DATA_TCTCREDIT_MAX_ADR(tc),
			    HW_ATL2_TPS_DATA_TCTCREDIT_MAX_MSK,
			    HW_ATL2_TPS_DATA_TCTCREDIT_MAX_SHIFT,
			    max_credit);
}

void hw_atl2_tps_tx_pkt_shed_tc_data_weight_set(struct aq_hw_s *aq_hw,
					       u32 tx_pkt_shed_tc_data_weight,
					       u32 tc)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TPS_DATA_TCTWEIGHT_ADR(tc),
			    HW_ATL2_TPS_DATA_TCTWEIGHT_MSK,
			    HW_ATL2_TPS_DATA_TCTWEIGHT_SHIFT,
			    tx_pkt_shed_tc_data_weight);
}

/* tx */
void hw_atl2_tx_tx_reg_res_dis_set(struct aq_hw_s *aq_hw, u32 tx_reg_res_dis)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TX_REG_RES_DSBL_ADR,
			    HW_ATL2_TX_REG_RES_DSBL_MSK,
			    HW_ATL2_TX_REG_RES_DSBL_SHIFT, tx_reg_res_dis);
}

/* msm */
u32 hw_atl2_msm_reg_access_status_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg_bit(aq_hw, HW_ATL2_MSM_REG_ACCESS_BUSY_ADR,
				  HW_ATL2_MSM_REG_ACCESS_BUSY_MSK,
				  HW_ATL2_MSM_REG_ACCESS_BUSY_SHIFT);
}

void hw_atl2_msm_reg_addr_for_indirect_addr_set(struct aq_hw_s *aq_hw,
					       u32 reg_addr_for_indirect_addr)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_MSM_REG_ADDR_ADR,
			    HW_ATL2_MSM_REG_ADDR_MSK,
			    HW_ATL2_MSM_REG_ADDR_SHIFT,
			    reg_addr_for_indirect_addr);
}

void hw_atl2_msm_reg_rd_strobe_set(struct aq_hw_s *aq_hw, u32 reg_rd_strobe)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_MSM_REG_RD_STROBE_ADR,
			    HW_ATL2_MSM_REG_RD_STROBE_MSK,
			    HW_ATL2_MSM_REG_RD_STROBE_SHIFT,
			    reg_rd_strobe);
}

u32 hw_atl2_msm_reg_rd_data_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg(aq_hw, HW_ATL2_MSM_REG_RD_DATA_ADR);
}

void hw_atl2_msm_reg_wr_data_set(struct aq_hw_s *aq_hw, u32 reg_wr_data)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_MSM_REG_WR_DATA_ADR, reg_wr_data);
}

void hw_atl2_msm_reg_wr_strobe_set(struct aq_hw_s *aq_hw, u32 reg_wr_strobe)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_MSM_REG_WR_STROBE_ADR,
			    HW_ATL2_MSM_REG_WR_STROBE_MSK,
			    HW_ATL2_MSM_REG_WR_STROBE_SHIFT,
			    reg_wr_strobe);
}

/* pci */
void hw_atl2_pci_pci_reg_res_dis_set(struct aq_hw_s *aq_hw, u32 pci_reg_res_dis)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_PCI_REG_RES_DSBL_ADR,
			    HW_ATL2_PCI_REG_RES_DSBL_MSK,
			    HW_ATL2_PCI_REG_RES_DSBL_SHIFT,
			    pci_reg_res_dis);
}

void hw_atl2_reg_glb_cpu_scratch_scp_set(struct aq_hw_s *aq_hw,
					u32 glb_cpu_scratch_scp,
					u32 scratch_scp)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_GLB_CPU_SCRATCH_SCP_ADR(scratch_scp),
			glb_cpu_scratch_scp);
}

/* tsg */
inline static void _hw_atl2_clock_modif_value_set(struct aq_hw_s* aq_hw, u32 clock_sel, u64 ns)
{
	aq_hw_write_reg64(aq_hw, HW_ATL2_TSG_REG_ADR(clock_sel, CLOCK_MODIF_VAL_LSW), ns);
}

void hw_atl2_tsg_clock_en(struct aq_hw_s *aq_hw, u32 clock_sel, u32 clock_enable)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TSG_REG_ADR(clock_sel, CLOCK_CFG), 
		HW_ATL2_TSG_CLOCK_EN_MSK, HW_ATL2_TSG_CLOCK_EN_SHIFT, clock_enable);
}

void hw_atl2_tsg_clock_reset(struct aq_hw_s *aq_hw, u32 clock_sel)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TSG_REG_ADR(clock_sel, CLOCK_CFG), 
		HW_ATL2_TSG_SYNC_RESET_MSK, HW_ATL2_TSG_SYNC_RESET_SHIFT, 1);
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TSG_REG_ADR(clock_sel, CLOCK_CFG), 
		HW_ATL2_TSG_SYNC_RESET_MSK, HW_ATL2_TSG_SYNC_RESET_SHIFT, 0);
}

u64 hw_atl2_tsg_clock_read(struct aq_hw_s *aq_hw, u32 clock_sel)
{
	return aq_hw_read_reg64(aq_hw, HW_ATL2_TSG_REG_ADR(clock_sel, READ_CUR_NS_LSW));
}

void hw_atl2_tsg_clock_set(struct aq_hw_s *aq_hw, u32 clock_sel, u64 ns, u32 fns)
{
	_hw_atl2_clock_modif_value_set(aq_hw, clock_sel, ns);
	aq_hw_write_reg(aq_hw, HW_ATL2_TSG_REG_ADR(clock_sel, CLOCK_MODIF_CTRL), 
		HW_ATL2_TSG_SET_COUNTER_MSK);
}

void hw_atl2_tsg_clock_add(struct aq_hw_s *aq_hw, u32 clock_sel, u64 ns, u32 fns)
{
	_hw_atl2_clock_modif_value_set(aq_hw, clock_sel, ns);
	aq_hw_write_reg(aq_hw, HW_ATL2_TSG_REG_ADR(clock_sel, CLOCK_MODIF_CTRL), 
		HW_ATL2_TSG_ADD_COUNTER_MSK);
}

void hw_atl2_tsg_clock_sub(struct aq_hw_s *aq_hw, u32 clock_sel, u64 ns, u32 fns)
{
	_hw_atl2_clock_modif_value_set(aq_hw, clock_sel, ns);
	aq_hw_write_reg(aq_hw, HW_ATL2_TSG_REG_ADR(clock_sel, CLOCK_MODIF_CTRL), 
		HW_ATL2_TSG_SUBSTRACT_COUNTER_MSK);
}

void hw_atl2_tsg_clock_increment_set(struct aq_hw_s *aq_hw, u32 clock_sel, u32 ns, u32 fns)
{
	u32 nsfns = (ns & 0xff) | (fns & 0xffffff00);
	aq_hw_write_reg(aq_hw, HW_ATL2_TSG_REG_ADR(clock_sel, CLOCK_INC_CFG),  nsfns);
	aq_hw_write_reg(aq_hw, HW_ATL2_TSG_REG_ADR(clock_sel, CLOCK_MODIF_CTRL), 
		HW_ATL2_TSG_LOAD_INC_CFG_MSK);
}

void hw_atl2_tsg_gpio_input_set(struct aq_hw_s *aq_hw, int on, u32 pin, u32 clock_sel)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TSG_REG_ADR(clock_sel, GPIO_CFG), 
		HW_ATL2_TSG_GPIO_IN_MODE_MSK, HW_ATL2_TSG_GPIO_IN_MONITOR_EN_SHIFT, 
		!!on ? HW_ATL2_TSG_GPIO_IN_MONITOR_EN_MSK | (HW_ATL2_TSG_GPIO_IN_MODE_POSEDGE << HW_ATL2_TSG_GPIO_IN_MODE_SHIFT): 0);

	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TSG_SPARE_WRITE_REG_ADR, 
		HW_ATL2_TSG_SPARE_FPGA_GPIO_CTRL_MSK, HW_ATL2_TSG_SPARE_FPGA_GPIO_CTRL_SHIFT,
		clock_sel == ATL_TSG_CLOCK_SEL_1 ? HW_ATL2_TSG_SPARE_FPGA_PTM_GPIO_TS_I : HW_ATL2_TSG_SPARE_FPGA_PTP_GPIO_TS_I);
}

void hw_atl2_tsg_ext_isr_to_host_set(struct aq_hw_s *aq_hw, int on)
{
	printk("aq_hw %p, on %d\n", aq_hw, on);
	aq_hw_write_reg_bit(aq_hw, A2_GLOBAL_CONTROL_2_ADR, 
		A2_MIF_INTERRUPT_2_TO_ITR_MSK, A2_MIF_INTERRUPT_TO_ITR_SHIFT + 2,
		!!on);
	aq_hw_write_reg_bit(aq_hw, A2_GLOBAL_CONTROL_2_ADR, 
		A2_EN_INTERRUPT_MIF2_TO_ITR_MSK, A2_EN_INTERRUPT_TO_ITR_SHIFT + 2,
		!!on);
}

void hw_atl2_tsg_gpio_isr_to_host_set(struct aq_hw_s *aq_hw, int on, u32 clock_sel)
{
	aq_hw_write_reg_bit(aq_hw, A2_GLOBAL_HIGH_PRIO_INTERRUPT_1_MASK_ADR, 
		clock_sel == ATL_TSG_CLOCK_SEL_1 ? A2_TSG_PTM_GPIO_INTERRUPT_MSK : A2_TSG_PTP_GPIO_INTERRUPT_MSK, 
		clock_sel == ATL_TSG_CLOCK_SEL_1 ? A2_TSG_PTM_GPIO_INTERRUPT_SHIFT : A2_TSG_PTP_GPIO_INTERRUPT_SHIFT,
		!!on);
}

void hw_atl2_ptm_isr_to_host_set(struct aq_hw_s *aq_hw, int on)
{
	aq_hw_write_reg_bit(aq_hw, A2_PCIE_HIGH_PRIO_INTERRUPT_0_MASK_ADR, 
		A2_PCIE_PTM_CLOCK_UPDATE_ALARM_MSK, 
		A2_PCIE_PTM_CLOCK_UPDATE_ALARM_SHIFT,
		!!on);
}

void hw_atl2_phi_ptm_en(struct aq_hw_s *aq_hw, int on)
{
	//Stub if something required for special platforms	
}

void hw_atl2_ptm_clear_status(struct aq_hw_s *aq_hw)
{
	/*u32 hw_alarm0 = */aq_hw_read_reg(aq_hw, A2_PCIE_INTERNAL_ALARMS_0_ADR);
}

void hw_atl2_tsg_gpio_clear_status(struct aq_hw_s *aq_hw, u32 clock_sel)
{
	/*u32 hw_alarm1 = */aq_hw_read_reg(aq_hw, A2_GLOBAL_INTERNAL_ALARMS_1_ADR);
}

void hw_atl2_tsg_gpio_input_envent_info_get(struct aq_hw_s *aq_hw, u32 clock_sel, u32 *event_count, u64 *event_ts)
{
	if( event_count )
		*event_count = aq_hw_read_reg(aq_hw, HW_ATL2_TSG_REG_ADR(clock_sel, EXT_CLK_COUNT));
	if( event_ts )
		*event_ts = aq_hw_read_reg64(aq_hw, HW_ATL2_TSG_REG_ADR(clock_sel, GPIO_EVENT_TS_LSW));
}

void hw_atl2_tsg_ptp_gpio_gen_pulse(struct aq_hw_s *aq_hw, u32 clk_sel, u64 ts, u32 period, u32 hightime)
{
	if( ts != 0 ) {
		aq_hw_write_reg64(aq_hw, HW_ATL2_TSG_REG_ADR(clk_sel, GPIO_EVENT_GEN_TS_LSW), ts);

		aq_hw_write_reg64(aq_hw, HW_ATL2_TSG_REG_ADR(clk_sel, GPIO_EVENT_HIGH_TIME_LSW), hightime);
		aq_hw_write_reg64(aq_hw, HW_ATL2_TSG_REG_ADR(clk_sel, GPIO_EVENT_LOW_TIME_LSW), (period - hightime));

		aq_hw_write_reg_bit(aq_hw, HW_ATL2_TSG_SPARE_WRITE_REG_ADR, 
			HW_ATL2_TSG_SPARE_FPGA_GPIO_CTRL_MSK, HW_ATL2_TSG_SPARE_FPGA_GPIO_CTRL_SHIFT,
			clk_sel == ATL_TSG_CLOCK_SEL_1 ? HW_ATL2_TSG_SPARE_FPGA_PTM_CLK_EVNT_O : HW_ATL2_TSG_SPARE_FPGA_PTP_CLK_EVNT_O);
	}

	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TSG_REG_ADR(clk_sel, GPIO_EVENT_GEN_CFG),
		HW_ATL2_TSG_GPIO_EVENT_MODE_MSK | HW_ATL2_TSG_GPIO_OUTPUT_EN_MSK | HW_ATL2_TSG_GPIO_GEN_OUTPUT_EN_MSK, 
		HW_ATL2_TSG_GPIO_OUTPUT_EN_SHIFT,
			(!ts  ? 0 : 
			(HW_ATL2_TSG_GPIO_EVENT_MODE_SET_ON_TIME << (HW_ATL2_TSG_GPIO_EVENT_MODE_SHIFT - HW_ATL2_TSG_GPIO_OUTPUT_EN_SHIFT)) | 
			(HW_ATL2_TSG_GPIO_GEN_OUTPUT_EN_MSK) |
			(HW_ATL2_TSG_GPIO_OUTPUT_EN_MSK))
			);
}

/* tdm, rdm and etc */
void hw_atl2_rx_q_map_to_tc(struct aq_hw_s *aq_hw, u32 q, u32 tc)
{
	u32 reg_addr = HW_ATL2_RX_Q_TO_TC_MAP_ADR(q);
	aq_hw_write_reg_bit(aq_hw, reg_addr, HW_ATL2_RX_Q_TO_TC_MAP_MSK(q), 
						HW_ATL2_RX_Q_TO_TC_MAP_SHIFT(q), tc);
}

void hw_atl2_rdm_rx_desc_ts_req_set(struct aq_hw_s *aq_hw, u32 clk_sel,
			       u32 descriptor)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RDM_DESCDEN_ADR(descriptor),
			    HW_ATL2_RDM_DESCTSREQ_MSK,
			    HW_ATL2_RDM_DESCTSREQ_SHIFT,
			    clk_sel == ATL_TSG_CLOCK_SEL_1 ? 0x2 : 0x1);
}

void hw_atl2_tdm_tx_desc_wb_ts_req_set(struct aq_hw_s *aq_hw, bool en,
			       u32 descriptor)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TDM_DESCDEN_ADR(descriptor),
			    HW_ATL2_TDM_DESCDTSWBEN_MSK,
			    HW_ATL2_TDM_DESCDTSWBEN_SHIFT,
			    en);
}

void hw_atl2_tdm_tx_desc_ts_req_set(struct aq_hw_s *aq_hw, bool en,
			       u32 descriptor)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TDM_DESCDEN_ADR(descriptor),
			    HW_ATL2_TDM_DESCDTSEN_MSK,
			    HW_ATL2_TDM_DESCDTSEN_SHIFT,
			    en);
}

void hw_atl2_tdm_tx_desc_lt_en_set(struct aq_hw_s *aq_hw, bool en,
			       u32 descriptor)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_TDM_DESCDEN_ADR(descriptor),
			    HW_ATL2_TDM_DESCDLTEN_MSK,
			    HW_ATL2_TDM_DESCDLTEN_SHIFT,
			    en);
}

void hw_atl2_init_launchtime(struct aq_hw_s *aq_hw)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_LT_CTRL_ADR, HW_ATL2_LT_CTRL_CLK_RATIO_MSK,
		HW_ATL2_LT_CTRL_CLK_RATIO_SHIFT, HW_ATL2_LT_CTRL_CLK_RATIO_QUATER_SPEED); //TODO FIX for ASIC
}

void hw_atl2_correct_launchtime(struct aq_hw_s *aq_hw, unsigned int mbps, unsigned int offset)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_LT_CTRL_ADR, HW_ATL2_LT_CTRL_LINK_SPEED_MSK,
		HW_ATL2_LT_CTRL_LINK_SPEED_SHIFT, mbps == 10000 ? 1 : 
										  mbps == 5000  ? 2 : 
										  mbps == 2500  ? 3 : 
										  mbps == 1000  ? 4 : 
										  mbps == 100   ? 5 : 
										  mbps == 10 	? 6 : 7);

	aq_hw_write_reg_bit(aq_hw, HW_ATL2_LT_CTRL2_ADR, HW_ATL2_LT_CTRL2_TIME_ADJ_MSK,
		HW_ATL2_LT_CTRL2_TIME_ADJ_SHIFT, offset);

	aq_hw_write_reg_bit(aq_hw, HW_ATL2_LT_CTRL3_ADR, HW_ATL2_LT_CTRL3_BG_TRF_ADJ_MSK,
		HW_ATL2_LT_CTRL3_BG_TRF_ADJ_SHIFT, 0x64); //TODO some calculations
}

void hw_atl2_rpfl3l4_ipv4_dest_addr_clear(struct aq_hw_s *aq_hw, u8 location)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_RX_GET_ADDR_DESTA_FL3L4(location), 0U);
}

void hw_atl2_rpfl3l4_ipv4_src_addr_clear(struct aq_hw_s *aq_hw, u8 location)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_RX_GET_ADDR_SRCA_FL3L4(location), 0U);
}

void hw_atl2_rpfl3l4_cmd_clear(struct aq_hw_s *aq_hw, u8 location)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_RX_GET_ADDR_CTRL_FL3L4(location), 0U);
}

void hw_atl2_rpfl3l4_ipv6_dest_addr_clear(struct aq_hw_s *aq_hw, u8 location)
{
	int i;

	for (i = 0; i < 4; ++i)
		aq_hw_write_reg(aq_hw,
				HW_ATL2_RX_GET_ADDR_DESTA_FL3L4(location + i),
				0U);
}

void hw_atl2_rpfl3l4_ipv6_src_addr_clear(struct aq_hw_s *aq_hw, u8 location)
{
	int i;

	for (i = 0; i < 4; ++i)
		aq_hw_write_reg(aq_hw,
				HW_ATL2_RX_GET_ADDR_SRCA_FL3L4(location + i),
				0U);
}

void hw_atl2_rpfl3l4_ipv4_dest_addr_set(struct aq_hw_s *aq_hw, u8 location,
				       u32 ipv4_dest)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_RX_GET_ADDR_DESTA_FL3L4(location),
			ipv4_dest);
}

void hw_atl2_rpfl3l4_ipv4_src_addr_set(struct aq_hw_s *aq_hw, u8 location,
				      u32 ipv4_src)
{
	aq_hw_write_reg(aq_hw,
			HW_ATL2_RX_GET_ADDR_SRCA_FL3L4(location),
			ipv4_src);
}

void hw_atl2_rpfl3l4_cmd_set(struct aq_hw_s *aq_hw, u8 location, u32 cmd)
{
	aq_hw_write_reg(aq_hw, HW_ATL2_RX_GET_ADDR_CTRL_FL3L4(location), cmd);
}

void hw_atl2_rpfl3l4_ipv6_src_addr_set(struct aq_hw_s *aq_hw, u8 location,
				      u32 *ipv6_src)
{
	int i;

	for (i = 0; i < 4; ++i)
		aq_hw_write_reg(aq_hw,
				HW_ATL2_RX_GET_ADDR_SRCA_FL3L4(location + i),
				ipv6_src[i]);
}

void hw_atl2_rpfl3l4_ipv6_dest_addr_set(struct aq_hw_s *aq_hw, u8 location,
				       u32 *ipv6_dest)
{
	int i;

	for (i = 0; i < 4; ++i)
		aq_hw_write_reg(aq_hw,
				HW_ATL2_RX_GET_ADDR_DESTA_FL3L4(location + i),
				ipv6_dest[i]);
}

/* set action resolver record */
void hw_atl2_rpf_act_rslvr_record_set(struct aq_hw_s *aq_hw, u8 location,
				      u32 tag, u32 mask, u32 action)
{
	aq_hw_write_reg(aq_hw,
			HW_ATL2_RPF_ACT_RSLVR_REQ_TAG_ADR(location),
			tag);
	aq_hw_write_reg(aq_hw,
			HW_ATL2_RPF_ACT_RSLVR_TAG_MASK_ADR(location),
			mask);
	aq_hw_write_reg(aq_hw,
			HW_ATL2_RPF_ACT_RSLVR_ACTN_ADR(location),
			action);
}

/* get action resolver tag record */
void hw_atl2_rpf_act_rslvr_record_get(struct aq_hw_s *aq_hw, u8 location,
				      u32 *tag, u32 *mask, u32 *action)
{
	*tag = aq_hw_read_reg(aq_hw,
			      HW_ATL2_RPF_ACT_RSLVR_REQ_TAG_ADR(location));
	*mask = aq_hw_read_reg(aq_hw,
			       HW_ATL2_RPF_ACT_RSLVR_TAG_MASK_ADR(location));
	*action = aq_hw_read_reg(aq_hw,
				 HW_ATL2_RPF_ACT_RSLVR_ACTN_ADR(location));
}

void hw_atl2_rpf_act_rslvr_section_en_set(struct aq_hw_s *aq_hw, u32 sections)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_RPF_REC_TAB_EN_ADR,
			HW_ATL2_RPF_REC_TAB_EN_MSK,
			HW_ATL2_RPF_REC_TAB_EN_SHIFT,
			sections);
}

void hw_atl2_mif_shared_buf_get(struct aq_hw_s *aq_hw, int offset,
				  u32 *data, int len)
{
	int i;
	int j = 0;

	for (i = offset; i < offset + len; i++, j++)
		data[j] = aq_hw_read_reg(aq_hw,
					 HW_ATL2_MIF_SHARED_BUFFER_IN_ADR(i));
}

void hw_atl2_mif_shared_buf_write(struct aq_hw_s *aq_hw, int offset,
				  u32 *data, int len)
{
	int i;
	int j = 0;

	for (i = offset; i < offset + len; i++, j++)
		aq_hw_write_reg(aq_hw, HW_ATL2_MIF_SHARED_BUFFER_IN_ADR(i),
				data[j]);
}

void hw_atl2_mif_shared_buf_read(struct aq_hw_s *aq_hw, int offset,
				  u32 *data, int len)
{
	int i;
	int j = 0;

	for (i = offset; i < offset + len; i++, j++)
		data[j] = aq_hw_read_reg(aq_hw,
					 HW_ATL2_MIF_SHARED_BUFFER_OUT_ADR(i));
}

void hw_atl2_mif_host_finished_write_set(struct aq_hw_s *aq_hw, u32 finish)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_MIF_HOST_FINISHED_WRITE_ADR,
			    HW_ATL2_MIF_HOST_FINISHED_WRITE_MSK,
			    HW_ATL2_MIF_HOST_FINISHED_WRITE_SHIFT,
			    finish);
}

u32 hw_atl2_mif_mcp_finished_read_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg_bit(aq_hw, HW_ATL2_MIF_MCP_FINISHED_READ_ADR,
				  HW_ATL2_MIF_MCP_FINISHED_READ_MSK,
				  HW_ATL2_MIF_MCP_FINISHED_READ_SHIFT);
}

u32 hw_atl2_mif_mcp_boot_reg_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg(aq_hw, HW_ATL2_MIF_BOOT_REG_ADR);
}

void hw_atl2_mif_mcp_boot_reg_set(struct aq_hw_s *aq_hw, u32 val)
{
	return aq_hw_write_reg(aq_hw, HW_ATL2_MIF_BOOT_REG_ADR, val);
}

u32 hw_atl2_mif_host_req_int_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg(aq_hw, HW_ATL2_MCP_HOST_REQ_INT_ADR);
}

void hw_atl2_mif_host_req_int_clr(struct aq_hw_s *aq_hw, u32 val)
{
	return aq_hw_write_reg(aq_hw, HW_ATL2_MCP_HOST_REQ_INT_CLR_ADR,
			       val);
}

u32 hw_atl2_mif_mcp_boot_ver_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg(aq_hw, HW_ATL2_MIF_BOOT_VER_ADR);
}

void hw_atl2_com_ful_reset_tgl(struct aq_hw_s *aq_hw)
{
	//aq_hw_write_reg_bit(aq_hw, HW_ATL2_COMMON_GENERAL_CTRL_REG_ADR,
	//		HW_ATL2_COM_FULL_RESET_MSK,
	//		HW_ATL2_COM_FULL_RESET_SHIFT,
	//		1);
	aq_hw_write_reg(aq_hw, HW_ATL2_COMMON_GENERAL_CTRL_REG_ADR, HW_ATL2_COM_FULL_RESET_MSK);
	AQ_HW_SLEEP(10);
	aq_hw_write_reg(aq_hw, HW_ATL2_COMMON_GENERAL_CTRL_REG_ADR, 0);
	//aq_hw_write_reg_bit(aq_hw, HW_ATL2_COMMON_GENERAL_CTRL_REG_ADR,
	//		HW_ATL2_COM_FULL_RESET_MSK,
	//		HW_ATL2_COM_FULL_RESET_SHIFT,
	//		0); 
}

/* PCI Config space */
void hw_atl2_pci_cfg_ptm_long_timer_set(struct aq_hw_s *aq_hw, u32 timer)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_PCI_PTM_REQ_CTRL_ADR,
			HW_ATL2_PCI_PTM_REQ_LONG_TIMER_MSK,
			HW_ATL2_PCI_PTM_REQ_LONG_TIMER_SHIFT,
			timer);
}

void hw_atl2_pci_cfg_ptm_en(struct aq_hw_s *aq_hw, bool en)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_PCI_PTM_CONTROL_ADR,
			HW_ATL2_PCI_PTM_CTRL_PTM_EN_MSK,
			HW_ATL2_PCI_PTM_CTRL_PTM_EN_SHIFT,
			en);
}

void hw_atl2_pci_cfg_ptm_manual_trig(struct aq_hw_s *aq_hw)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_PCI_PTM_REQ_CTRL_ADR,
			HW_ATL2_PCI_PTM_REQ_START_UPDATE_MSK,
			HW_ATL2_PCI_PTM_REQ_START_UPDATE_SHIFT,
			1);
}

void hw_atl2_pci_cfg_ptm_enianess_fix(struct aq_hw_s *aq_hw, uint32_t fix)
{
	aq_hw_write_reg_bit(aq_hw, HW_ATL2_PCI_PTM_REQ_CTRL_ADR,
			HW_ATL2_PCI_PTM_REQ_PDEL_BYTE_REV_MSK,
			HW_ATL2_PCI_PTM_REQ_PDEL_BYTE_REV_SHIFT,
			fix);
}

void hw_atl2_pci_cfg_ptm_req_local_get(struct aq_hw_s *aq_hw, u64 *ts)
{
	if( ts )
		*ts = aq_hw_read_reg64(aq_hw, HW_ATL2_PCI_PTM_REQ_LOCAL_LSW_ADR);
}

void hw_atl2_pci_cfg_ptm_req_t1_get(struct aq_hw_s *aq_hw, u64 *ts)
{
	if( ts )
		*ts = aq_hw_read_reg64(aq_hw, HW_ATL2_PCI_PTM_REQ_T1_LSW_ADR);
}

void hw_atl2_pci_cfg_ptm_req_t1p_get(struct aq_hw_s *aq_hw, u64 *ts)
{
	if( ts )
		*ts = aq_hw_read_reg64(aq_hw, HW_ATL2_PCI_PTM_REQ_T1P_LSW_ADR);
}

void hw_atl2_pci_cfg_ptm_req_t4_get(struct aq_hw_s *aq_hw, u64 *ts)
{
	if( ts )
		*ts = aq_hw_read_reg64(aq_hw, HW_ATL2_PCI_PTM_REQ_T4_LSW_ADR);
}

void hw_atl2_pci_cfg_ptm_req_t4p_get(struct aq_hw_s *aq_hw, u64 *ts)
{
	if( ts )
		*ts = aq_hw_read_reg64(aq_hw, HW_ATL2_PCI_PTM_REQ_T4P_LSW_ADR);
}

void hw_atl2_pci_cfg_ptm_req_master_get(struct aq_hw_s *aq_hw, u64 *ts)
{
	if( ts )
		*ts = aq_hw_read_reg64(aq_hw, HW_ATL2_PCI_PTM_REQ_MASTER_LSW_ADR);
}

void hw_atl2_pci_cfg_ptm_req_pdelay_get(struct aq_hw_s *aq_hw, u32 *pdelay)
{
	if( pdelay )
		*pdelay = aq_hw_read_reg(aq_hw, HW_ATL2_PCI_PTM_REQ_PROP_DELAY_ADR);
}

void hw_atl2_pci_cfg_ptm_req_master1_get(struct aq_hw_s *aq_hw, u64 *ts)
{
	if( ts )
		*ts = aq_hw_read_reg64(aq_hw, HW_ATL2_PCI_PTM_REQ_MASTERT1_LSW_ADR);
}

void hw_atl2_pci_cfg_ptm_req_clock_corr_get(struct aq_hw_s *aq_hw, u64 *ts)
{
	if( ts )
		*ts = aq_hw_read_reg64(aq_hw, HW_ATL2_PCI_PTM_REQ_CLOCK_CORR_LSW_ADR);
}

void hw_atl2_pci_cfg_ptm_req_tx_lat_get(struct aq_hw_s *aq_hw, u32 *tx_lat)
{
	if( tx_lat )
		*tx_lat = aq_hw_read_reg(aq_hw, HW_ATL2_PCI_PTM_REQ_TX_LATENCY_ADR);
}

void hw_atl2_pci_cfg_ptm_req_rx_lat_get(struct aq_hw_s *aq_hw, u32 *rx_lat)
{
	if( rx_lat )
		*rx_lat = aq_hw_read_reg(aq_hw, HW_ATL2_PCI_PTM_REQ_RX_LATENCY_ADR);
}

/* PHI */
void hw_atl2_timer_enable_to_update_ptm_clock_set(struct aq_hw_s *aq_hw, u32 timerEnableToUpdatePtmClock)
{
 	aq_hw_write_reg_bit(aq_hw, 
		HW_ATL2_HW_ATL2_PTM_UPDATETIMERENABLE_ADR, 
		HW_ATL2_HW_ATL2_PTM_UPDATETIMERENABLE_MSK, 
		HW_ATL2_HW_ATL2_PTM_UPDATETIMERENABLE_SHIFT, 
		timerEnableToUpdatePtmClock);
}

u32 hw_atl2_timer_enable_to_update_ptm_clock_get(struct aq_hw_s *aq_hw)
{
 	return aq_hw_read_reg_bit(aq_hw,
		HW_ATL2_HW_ATL2_PTM_UPDATETIMERENABLE_ADR, 
		HW_ATL2_HW_ATL2_PTM_UPDATETIMERENABLE_MSK, 
		HW_ATL2_HW_ATL2_PTM_UPDATETIMERENABLE_SHIFT);
}


void hw_atl2_request_to_auto_update_clock_set(struct aq_hw_s *aq_hw, u32 requestToAutoUpdateClock)
{
 	aq_hw_write_reg_bit(aq_hw, 
		HW_ATL2_HW_ATL2_PTM_AUTOUPDATESIGNAL_ADR, 
		HW_ATL2_HW_ATL2_PTM_AUTOUPDATESIGNAL_MSK, 
		HW_ATL2_HW_ATL2_PTM_AUTOUPDATESIGNAL_SHIFT, 
		requestToAutoUpdateClock);
}

u32 hw_atl2_request_to_auto_update_clock_get(struct aq_hw_s *aq_hw)
{
 	return aq_hw_read_reg_bit(aq_hw,
		HW_ATL2_HW_ATL2_PTM_AUTOUPDATESIGNAL_ADR, 
		HW_ATL2_HW_ATL2_PTM_AUTOUPDATESIGNAL_MSK, 
		HW_ATL2_HW_ATL2_PTM_AUTOUPDATESIGNAL_SHIFT);
}


void hw_atl2_manual_request_to_update_clock_set(struct aq_hw_s *aq_hw, u32 manualRequestToUpdateClock)
{
 	aq_hw_write_reg_bit(aq_hw, 
		HW_ATL2_HW_ATL2_PTM_MANUALUPDATEPULSE_ADR, 
		HW_ATL2_HW_ATL2_PTM_MANUALUPDATEPULSE_MSK, 
		HW_ATL2_HW_ATL2_PTM_MANUALUPDATEPULSE_SHIFT, 
		manualRequestToUpdateClock);
}

u32 hw_atl2_manual_request_to_update_clock_get(struct aq_hw_s *aq_hw)
{
 	return aq_hw_read_reg_bit(aq_hw,
		HW_ATL2_HW_ATL2_PTM_MANUALUPDATEPULSE_ADR, 
		HW_ATL2_HW_ATL2_PTM_MANUALUPDATEPULSE_MSK, 
		HW_ATL2_HW_ATL2_PTM_MANUALUPDATEPULSE_SHIFT);
}


u32 hw_atl2_local_clock_updated_get(struct aq_hw_s *aq_hw)
{
 	return aq_hw_read_reg_bit(aq_hw,
		HW_ATL2_PTMCLOCKUPDATED_ADR,
		HW_ATL2_PTMCLOCKUPDATED_MSK,
		HW_ATL2_PTMCLOCKUPDATED_SHIFT);
}


u32 hw_atl2_ptm_responder_ready_get(struct aq_hw_s *aq_hw)
{
 	return aq_hw_read_reg_bit(aq_hw,
		HW_ATL2_PTMRESPONDERRDYTOVALIDATE_ADR, 
		HW_ATL2_PTMRESPONDERRDYTOVALIDATE_MSK, 
		HW_ATL2_PTMRESPONDERRDYTOVALIDATE_SHIFT);
}


u32 hw_atl2_ptm_context_valid_get(struct aq_hw_s *aq_hw)
{
 	return aq_hw_read_reg_bit(aq_hw,
		HW_ATL2_PTMCONTEXTVALID_ADR, 
		HW_ATL2_PTMCONTEXTVALID_MSK, 
		HW_ATL2_PTMCONTEXTVALID_SHIFT);
}

uint64_t hw_atl2_local_clock_value_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg64(aq_hw, HW_ATL2_PTM_LOCALCLOCKLSB_ADR);
}

uint64_t hw_atl2_local_clock_correction_value_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg64(aq_hw, HW_ATL2_PTM_CLOCKCORRECTIONLSB_ADR);
}

void hw_atl2_ptm_update_counter_value_set(struct aq_hw_s *aq_hw, u32 ptmUpdateCounterValue)
{
 	aq_hw_write_reg(aq_hw, HW_ATL2_PTM_UPDATECTR_ADR, ptmUpdateCounterValue);
}

u32 hw_atl2_ptm_update_counter_value_get(struct aq_hw_s *aq_hw)
{
 	return aq_hw_read_reg(aq_hw, HW_ATL2_PTM_UPDATECTR_ADR);
}

u32 hw_atl2_ptm_context_id_get(struct aq_hw_s *aq_hw)
{
 	return aq_hw_read_reg_bit(aq_hw,
		HW_ATL2_PTM_CTXTID_ADR, 
		HW_ATL2_PTM_CTXTID_MSK, 
		HW_ATL2_PTM_CTXTID_SHIFT);
}


u32 hw_atl2_ptm_sequence_id_get(struct aq_hw_s *aq_hw)
{
 	return aq_hw_read_reg_bit(aq_hw,
		HW_ATL2_PTM_SEQID_ADR,
		HW_ATL2_PTM_SEQID_MSK, 
		HW_ATL2_PTM_SEQID_SHIFT);
}

uint64_t hw_atl2_ptm_tsg_local_clock_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg64(aq_hw, HW_ATL2_PTMTSGLOCALCLOCKLOWER_ADR);
}

void hw_atl2_ptp_tsg_local_clock_lower_set(struct aq_hw_s *aq_hw, u32 ptpTsgLocalClockLower)
{
 	aq_hw_write_reg(aq_hw, HW_ATL2_PTPTSGLOCALCLOCKLOW_ADR, ptpTsgLocalClockLower);
}

void hw_atl2_ptp_tsg_local_clock_upper_set(struct aq_hw_s *aq_hw, u32 ptpTsgLocalClockUpper)
{
 	aq_hw_write_reg(aq_hw, HW_ATL2_PTPTSGLOCALCLOCKUPPER_ADR, ptpTsgLocalClockUpper);
}

uint64_t hw_atl2_ptp_tsg_local_clock_get(struct aq_hw_s *aq_hw)
{
	return aq_hw_read_reg64(aq_hw, HW_ATL2_PTPTSGLOCALCLOCKLOW_ADR);
}

u32 hw_atl2_sem_act_rslvr_tbl_get(struct aq_hw_s *self)
{
	return hw_atl2_reg_glb_cpu_sem_get(self, HW_ATL2_FW_SM_ACT_RSLVR);
}

//EOF
