/*
 * aQuantia Corporation Network Driver
 * Copyright (C) 2014-2019 aQuantia Corporation. All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

/* File hw_atl2_llh_internal.h: Preprocessor definitions
 * for Atlantic registers.
 */

#ifndef HW_ATL2_LLH_INTERNAL_H
#define HW_ATL2_LLH_INTERNAL_H

/* Resets */
/*! @name PCIE dmc_logic_reset Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "dmc_logic_reset".
*
*   Type: R/WSC
*
*   Notes: Resets the DMC Logic
This bit is self-clearing.
*
*   PORT="pif_dmc_logic_reset_i"
@{ */
/*! \brief Register address for bitfield dmc_logic_reset */
#define HW_ATL2_LOGIC_RESET_ADR 0x00001004
/*! \brief Bitmask for bitfield dmc_logic_reset */
#define  HW_ATL2_LOGIC_RESET_MSK 0x00080000
/*! \brief Inverted bitmask for bitfield dmc_logic_reset */
#define  HW_ATL2_LOGIC_RESET_MSKN 0xFFF7FFFF
/*! \brief Lower bit position of bitfield dmc_logic_reset */
#define  HW_ATL2_LOGIC_RESET_SHIFT 19
/*! \brief Width of bitfield dmc_logic_reset */
#define  HW_ATL2_LOGIC_RESET_WIDTH 1
/*! \brief Default value of bitfield dmc_logic_reset */
#define  HW_ATL2_LOGIC_RESET_DEFAULT 0x0
/*@}*/

/*! @name PCIE pci_logic_reset Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pci_logic_reset".
*
*   Type: R/WSC
*
*   Notes: Resets the PHI and Controller Logic
This bit is self-clearing.
*
*   PORT="pif_phi_logic_reset_i"
@{ */
/*! \brief Register address for bitfield pci_logic_reset */
#define HW_ATL2_PCI_LOGIC_RESET_ADR 0x00001004
/*! \brief Bitmask for bitfield pci_logic_reset */
#define HW_ATL2_PCI_LOGIC_RESET_MSK 0x00040000
/*! \brief Inverted bitmask for bitfield pci_logic_reset */
#define HW_ATL2_PCI_LOGIC_RESET_MSKN 0xFFFBFFFF
/*! \brief Lower bit position of bitfield pci_logic_reset */
#define HW_ATL2_PCI_LOGIC_RESET_SHIFT 18
/*! \brief Width of bitfield pci_logic_reset */
#define HW_ATL2_PCI_LOGIC_RESET_WIDTH 1
/*! \brief Default value of bitfield pci_logic_reset */
#define HW_ATL2_PCI_LOGIC_RESET_DEFAULT 0x0
/*@}*/

/*! @name PCIE pci_register_reset Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pci_register_reset".
*
*   Type: R/WSC
*
*   Notes: Resets the PHI Registers.


This bit is self-clearing.
*
*   PORT="pif_phi_reg_reset_i"
@{ */
/*! \brief Register address for bitfield pci_register_reset */
#define HW_ATL2_PCI_REGISTER_RESET_ADR 0x00001004
/*! \brief Bitmask for bitfield pci_register_reset */
#define HW_ATL2_PCI_REGISTER_RESET_MSK 0x00020000
/*! \brief Inverted bitmask for bitfield pci_register_reset */
#define HW_ATL2_PCI_REGISTER_RESET_MSKN 0xFFFDFFFF
/*! \brief Lower bit position of bitfield pci_register_reset */
#define HW_ATL2_PCI_REGISTER_RESET_SHIFT 17
/*! \brief Width of bitfield pci_register_reset */
#define HW_ATL2_PCI_REGISTER_RESET_WIDTH 1
/*! \brief Default value of bitfield pci_register_reset */
#define HW_ATL2_PCI_REGISTER_RESET_DEFAULT 0x0
/*@}*/

/*! @name MIF MIPS Reset Pulse Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "MIPS Reset Pulse".
*
*   Type: R/WSC
*
*   Notes: Resets the MIPS core. 


This bit is self-clearing.
*
*   PORT="pif_mips_reset_pulse_i"
@{ */
/*! \brief Register address for bitfield MIPS Reset Pulse */
#define HW_ATL2_MIPS_RESET_PULSE_ADR 0x00000400
/*! \brief Bitmask for bitfield MIPS Reset Pulse */
#define HW_ATL2_MIPS_RESET_PULSE_MSK 0x00000010
/*! \brief Inverted bitmask for bitfield MIPS Reset Pulse */
#define HW_ATL2_MIPS_RESET_PULSE_MSKN 0xFFFFFFEF
/*! \brief Lower bit position of bitfield MIPS Reset Pulse */
#define HW_ATL2_MIPS_RESET_PULSE_SHIFT 4
/*! \brief Width of bitfield MIPS Reset Pulse */
#define HW_ATL2_MIPS_RESET_PULSE_WIDTH 1
/*! \brief Default value of bitfield MIPS Reset Pulse */
#define HW_ATL2_MIPS_RESET_PULSE_DEFAULT 0x0
/*@}*/

/*! @name MIF AHB Reset Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "AHB Reset".
*
*   Type: R/WSC
*
*   Notes: Resets the AHB Bus
This will trigger HRESETn and resets the AHB arbitration module

This bit is self-clearing.
*
*   PORT="pif_mif_ahb_reset_i"
@{ */
/*! \brief Register address for bitfield AHB Reset */
#define HW_ATL2_MIF_AHB_RESET_ADR 0x00000400
/*! \brief Bitmask for bitfield AHB Reset */
#define HW_ATL2_MIF_AHB_RESET_MSK 0x00000008
/*! \brief Inverted bitmask for bitfield AHB Reset */
#define HW_ATL2_MIF_AHB_RESET_MSKN 0xFFFFFFF7
/*! \brief Lower bit position of bitfield AHB Reset */
#define HW_ATL2_MIF_AHB_RESET_SHIFT 3
/*! \brief Width of bitfield AHB Reset */
#define HW_ATL2_MIF_AHB_RESET_WIDTH 1
/*! \brief Default value of bitfield AHB Reset */
#define HW_ATL2_MIF_AHB_RESET_DEFAULT 0x0
/*@}*/

/*! @name MIF AUX Reset Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "AUX Reset".
*
*   Type: R/WSC
*
*   Notes: Resets the AUX Arbitration Logic
This includes resetting MIPS and will rerun DRAM Init, if disabled.
Security Register won't be affected

This bit is self-clearing.
*
*   PORT="pif_mif_aux_reset_i"
@{ */
/*! \brief Register address for bitfield AUX Reset */
#define HW_ATL2_MIF_AUX_RESET_ADR 0x00000400
/*! \brief Bitmask for bitfield AUX Reset */
#define HW_ATL2_MIF_AUX_RESET_MSK 0x00000004
/*! \brief Inverted bitmask for bitfield AUX Reset */
#define HW_ATL2_MIF_AUX_RESET_MSKN 0xFFFFFFFB
/*! \brief Lower bit position of bitfield AUX Reset */
#define HW_ATL2_MIF_AUX_RESET_SHIFT 2
/*! \brief Width of bitfield AUX Reset */
#define HW_ATL2_MIF_AUX_RESET_WIDTH 1
/*! \brief Default value of bitfield AUX Reset */
#define HW_ATL2_MIF_AUX_RESET_DEFAULT 0x0
/*@}*/

/*! @name MIF MIF Logic Reset Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "MIF Logic Reset".
*
*   Type: R/WSC
*
*   Notes: Resets the MIF Logic
This includes resetting MIPS and will rerun DRAM Init, if enabled
Security Register won't be affected

This bit is self-clearing.
*
*   PORT="pif_mif_logic_reset_i"
@{ */
/*! \brief Register address for bitfield MIF Logic Reset */
#define HW_ATL2_MIF_LOGIC_RESET_ADR 0x00000400
/*! \brief Bitmask for bitfield MIF Logic Reset */
#define HW_ATL2_MIF_LOGIC_RESET_MSK 0x00000002
/*! \brief Inverted bitmask for bitfield MIF Logic Reset */
#define HW_ATL2_MIF_LOGIC_RESET_MSKN 0xFFFFFFFD
/*! \brief Lower bit position of bitfield MIF Logic Reset */
#define HW_ATL2_MIF_LOGIC_RESET_SHIFT 1
/*! \brief Width of bitfield MIF Logic Reset */
#define HW_ATL2_MIF_LOGIC_RESET_WIDTH 1
/*! \brief Default value of bitfield MIF Logic Reset */
#define HW_ATL2_MIF_LOGIC_RESET_DEFAULT 0x0
/*@}*/

/*! @name MIF MIF Register Reset Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "MIF Register Reset".
*
*   Type: R/WSC
*
*   Notes: Resets the MIF Registers.
Won't affect Security registers and NO_SOFT_RESET registers.

This bit is self-clearing.
*
*   PORT="pif_mif_reg_reset_i"
@{ */
/*! \brief Register address for bitfield MIF Register Reset */
#define HW_ATL2_MIF_REGISTER_RESET_ADR 0x00000400
/*! \brief Bitmask for bitfield MIF Register Reset */
#define HW_ATL2_MIF_REGISTER_RESET_MSK 0x00000001
/*! \brief Inverted bitmask for bitfield MIF Register Reset */
#define HW_ATL2_MIF_REGISTER_RESET_MSKN 0xFFFFFFFE
/*! \brief Lower bit position of bitfield MIF Register Reset */
#define HW_ATL2_MIF_REGISTER_RESET_SHIFT 0
/*! \brief Width of bitfield MIF Register Reset */
#define HW_ATL2_MIF_REGISTER_RESET_WIDTH 1
/*! \brief Default value of bitfield MIF Register Reset */
#define HW_ATL2_MIF_REGISTER_RESET_DEFAULT 0x0
/*@}*/

/*! @name MAC_PHY MPI Logic reset Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "MPI Logic reset".
*
*   Type: R/W
*
*   Notes: Setting this bit to reset all the MAC-PHY logic.
MAC-PHY is hold in Reset by default and needs to be disabled to use MAC-PHY path.
*
*   PORT="pif_mpi_logic_reset_i"
@{ */
/*! \brief Register address for bitfield MPI Logic reset */
#define HW_ATL2_MPI_LOGIC_RESET_ADR 0x00004004
/*! \brief Bitmask for bitfield MPI Logic reset */
#define HW_ATL2_MPI_LOGIC_RESET_MSK 0x00000002
/*! \brief Inverted bitmask for bitfield MPI Logic reset */
#define HW_ATL2_MPI_LOGIC_RESET_MSKN 0xFFFFFFFD
/*! \brief Lower bit position of bitfield MPI Logic reset */
#define HW_ATL2_MPI_LOGIC_RESET_SHIFT 1
/*! \brief Width of bitfield MPI Logic reset */
#define HW_ATL2_MPI_LOGIC_RESET_WIDTH 1
/*! \brief Default value of bitfield MPI Logic reset */
#define HW_ATL2_MPI_LOGIC_RESET_DEFAULT 0x0
/*@}*/

/*! @name MAC_PHY MPI Register reset Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "MPI Register reset".
*
*   Type: R/WSC
*
*   Notes: Resets the MAC-PHY Registers.

This bit is self-clearing.
*
*   PORT="pif_mpi_reg_reset_i"
@{ */
/*! \brief Register address for bitfield MPI Register reset */
#define HW_ATL2_MPI_REGISTER_RESET_ADR 0x00004004
/*! \brief Bitmask for bitfield MPI Register reset */
#define HW_ATL2_MPI_REGISTER_RESET_MSK 0x00000001
/*! \brief Inverted bitmask for bitfield MPI Register reset */
#define HW_ATL2_MPI_REGISTER_RESET_MSKN 0xFFFFFFFE
/*! \brief Lower bit position of bitfield MPI Register reset */
#define HW_ATL2_MPI_REGISTER_RESET_SHIFT 0
/*! \brief Width of bitfield MPI Register reset */
#define HW_ATL2_MPI_REGISTER_RESET_WIDTH 1
/*! \brief Default value of bitfield MPI Register reset */
#define HW_ATL2_MPI_REGISTER_RESET_DEFAULT 0x0
/*@}*/

/*! @name TX TX Logic Reset Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "TX Logic Reset".
*
*   Type: R/W
*
*   Notes: Setting this bit to reset all the Tx DMA and datapath logic.
TX is hold in Reset by default and needs to be disabled to use TX path.
*
*   PORT="pif_tx_logic_reset_i"
@{ */
/*! \brief Register address for bitfield TX Logic Reset */
#define HW_ATL2_TX_LOGIC_RESET_ADR 0x00007008
/*! \brief Bitmask for bitfield TX Logic Reset */
#define HW_ATL2_TX_LOGIC_RESET_MSK 0x00000002
/*! \brief Inverted bitmask for bitfield TX Logic Reset */
#define HW_ATL2_TX_LOGIC_RESET_MSKN 0xFFFFFFFD
/*! \brief Lower bit position of bitfield TX Logic Reset */
#define HW_ATL2_TX_LOGIC_RESET_SHIFT 1
/*! \brief Width of bitfield TX Logic Reset */
#define HW_ATL2_TX_LOGIC_RESET_WIDTH 1
/*! \brief Default value of bitfield TX Logic Reset */
#define HW_ATL2_TX_LOGIC_RESET_DEFAULT 0x0
/*@}*/

/*! @name TX TX Register Reset Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "TX Register Reset".
*
*   Type: R/WSC
*
*   Notes: Resets the TX Registers.

This bit is self-clearing.
*
*   PORT="pif_tx_reg_reset_i"
@{ */
/*! \brief Register address for bitfield TX Register Reset */
#define HW_ATL2_TX_REGISTER_RESET_ADR 0x00007008
/*! \brief Bitmask for bitfield TX Register Reset */
#define HW_ATL2_TX_REGISTER_RESET_MSK 0x00000001
/*! \brief Inverted bitmask for bitfield TX Register Reset */
#define HW_ATL2_TX_REGISTER_RESET_MSKN 0xFFFFFFFE
/*! \brief Lower bit position of bitfield TX Register Reset */
#define HW_ATL2_TX_REGISTER_RESET_SHIFT 0
/*! \brief Width of bitfield TX Register Reset */
#define HW_ATL2_TX_REGISTER_RESET_WIDTH 1
/*! \brief Default value of bitfield TX Register Reset */
#define HW_ATL2_TX_REGISTER_RESET_DEFAULT 0x0
/*@}*/

/*! @name RX RX Logic Reset Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "RX Logic Reset".
*
*   Type: R/W
*
*   Notes: Setting this bit to reset all the Rx DMA and datapath logic.
RX is hold in Reset by default and needs to be disabled to use RX path.
*
*   PORT="pif_rx_logic_reset_i"
@{ */
/*! \brief Register address for bitfield RX Logic Reset */
#define HW_ATL2_RX_LOGIC_RESET_ADR 0x00005008
/*! \brief Bitmask for bitfield RX Logic Reset */
#define HW_ATL2_RX_LOGIC_RESET_MSK 0x00000002
/*! \brief Inverted bitmask for bitfield RX Logic Reset */
#define HW_ATL2_RX_LOGIC_RESET_MSKN 0xFFFFFFFD
/*! \brief Lower bit position of bitfield RX Logic Reset */
#define HW_ATL2_RX_LOGIC_RESET_SHIFT 1
/*! \brief Width of bitfield RX Logic Reset */
#define HW_ATL2_RX_LOGIC_RESET_WIDTH 1
/*! \brief Default value of bitfield RX Logic Reset */
#define HW_ATL2_RX_LOGIC_RESET_DEFAULT 0x0
/*@}*/

/*! @name RX RX Register Reset Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "RX Register Reset".
*
*   Type: R/WSC
*
*   Notes: Resets the RX Registers.

This bit is self-clearing.
*
*   PORT="pif_rx_reg_reset_i"
@{ */
/*! \brief Register address for bitfield RX Register Reset */
#define HW_ATL2_RX_REGISTER_RESET_ADR 0x00005008
/*! \brief Bitmask for bitfield RX Register Reset */
#define HW_ATL2_RX_REGISTER_RESET_MSK 0x00000001
/*! \brief Inverted bitmask for bitfield RX Register Reset */
#define HW_ATL2_RX_REGISTER_RESET_MSKN 0xFFFFFFFE
/*! \brief Lower bit position of bitfield RX Register Reset */
#define HW_ATL2_RX_REGISTER_RESET_SHIFT 0
/*! \brief Width of bitfield RX Register Reset */
#define HW_ATL2_RX_REGISTER_RESET_WIDTH 1
/*! \brief Default value of bitfield RX Register Reset */
#define HW_ATL2_RX_REGISTER_RESET_DEFAULT 0x0
/*@}*/

/*! @name INTR itr_reset Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "itr_reset".
*
*   Type: R/W
*
*   Notes: Write a 1 to reset all the interrupt logics, this register is self-clearing by hardware.
*
*   PORT="pif_itr_reset_i"
@{ */
/*! \brief Register address for bitfield itr_reset */
#define HW_ATL2_ITR_RESET_ADR 0x00002300
/*! \brief Bitmask for bitfield itr_reset */
#define HW_ATL2_ITR_RESET_MSK 0x80000000
/*! \brief Inverted bitmask for bitfield itr_reset */
#define HW_ATL2_ITR_RESET_MSKN 0x7FFFFFFF
/*! \brief Lower bit position of bitfield itr_reset */
#define HW_ATL2_ITR_RESET_SHIFT 31
/*! \brief Width of bitfield itr_reset */
#define HW_ATL2_ITR_RESET_WIDTH 1
/*! \brief Default value of bitfield itr_reset */
#define HW_ATL2_ITR_RESET_DEFAULT 0x0
/*@}*/

/*! @name INTR itr_reg_reset_dsbl Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "itr_reg_reset_dsbl".
*
*   Type: R/W
*
*   Notes: Setting this bit prevents an Interrupt S/W reset from resetting the Interrupt registers
*
*   PORT="pif_itr_reg_reset_dsbl_i"
@{ */
/*! \brief Register address for bitfield itr_reg_reset_dsbl */
#define HW_ATL2_ITR_REG_RESET_DSBL_ADR 0x00002300
/*! \brief Bitmask for bitfield itr_reg_reset_dsbl */
#define HW_ATL2_ITR_REG_RESET_DSBL_MSK 0x20000000
/*! \brief Inverted bitmask for bitfield itr_reg_reset_dsbl */
#define HW_ATL2_ITR_REG_RESET_DSBL_MSKN 0xDFFFFFFF
/*! \brief Lower bit position of bitfield itr_reg_reset_dsbl */
#define HW_ATL2_ITR_REG_RESET_DSBL_SHIFT 29
/*! \brief Width of bitfield itr_reg_reset_dsbl */
#define HW_ATL2_ITR_REG_RESET_DSBL_WIDTH 1
/*! \brief Default value of bitfield itr_reg_reset_dsbl */
#define HW_ATL2_ITR_REG_RESET_DSBL_DEFAULT 0x1
/*@}*/


/* global microprocessor semaphore  definitions
 * base address: 0x000003a0
 * parameter: semaphore {s} | stride size 0x4 | range [0, 15]
 */
#define HW_ATL2_GLB_CPU_SEM_ADR(semaphore)  (0x000003a0u + (semaphore) * 0x4)
/* register address for bitfield rx dma good octet counter lsw [1f:0] */
#define HW_ATL2_STATS_RX_DMA_GOOD_OCTET_COUNTERLSW 0x00006808
/* register address for bitfield rx dma good packet counter lsw [1f:0] */
#define HW_ATL2_STATS_RX_DMA_GOOD_PKT_COUNTERLSW 0x00006800
/* register address for bitfield tx dma good octet counter lsw [1f:0] */
#define HW_ATL2_STATS_TX_DMA_GOOD_OCTET_COUNTERLSW 0x00008808
/* register address for bitfield tx dma good packet counter lsw [1f:0] */
#define HW_ATL2_STATS_TX_DMA_GOOD_PKT_COUNTERLSW 0x00008800

/* register address for bitfield rx dma good octet counter msw [3f:20] */
#define HW_ATL2_STATS_RX_DMA_GOOD_OCTET_COUNTERMSW 0x0000680c
/* register address for bitfield rx dma good packet counter msw [3f:20] */
#define HW_ATL2_STATS_RX_DMA_GOOD_PKT_COUNTERMSW 0x00006804
/* register address for bitfield tx dma good octet counter msw [3f:20] */
#define HW_ATL2_STATS_TX_DMA_GOOD_OCTET_COUNTERMSW 0x0000880c
/* register address for bitfield tx dma good packet counter msw [3f:20] */
#define HW_ATL2_STATS_TX_DMA_GOOD_PKT_COUNTERMSW 0x00008804

/* preprocessor definitions for msm rx errors counter register */
#define HW_ATL2_MAC_MSM_RX_ERRS_CNT_ADR 0x00000120u

/* preprocessor definitions for msm rx unicast frames counter register */
#define HW_ATL2_MAC_MSM_RX_UCST_FRM_CNT_ADR 0x000000e0u

/* preprocessor definitions for msm rx multicast frames counter register */
#define HW_ATL2_MAC_MSM_RX_MCST_FRM_CNT_ADR 0x000000e8u

/* preprocessor definitions for msm rx broadcast frames counter register */
#define HW_ATL2_MAC_MSM_RX_BCST_FRM_CNT_ADR 0x000000f0u

/* preprocessor definitions for msm rx broadcast octets counter register 1 */
#define HW_ATL2_MAC_MSM_RX_BCST_OCTETS_COUNTER1_ADR 0x000001b0u

/* preprocessor definitions for msm rx broadcast octets counter register 2 */
#define HW_ATL2_MAC_MSM_RX_BCST_OCTETS_COUNTER2_ADR 0x000001b4u

/* preprocessor definitions for msm rx unicast octets counter register 0 */
#define HW_ATL2_MAC_MSM_RX_UCST_OCTETS_COUNTER0_ADR 0x000001b8u

/* preprocessor definitions for rx dma statistics counter 7 */
#define HW_ATL2_RX_DMA_STAT_COUNTER7_ADR 0x00006818u

/* preprocessor definitions for msm tx unicast frames counter register */
#define HW_ATL2_MAC_MSM_TX_UCST_FRM_CNT_ADR 0x00000108u

/* preprocessor definitions for msm tx multicast frames counter register */
#define HW_ATL2_MAC_MSM_TX_MCST_FRM_CNT_ADR 0x00000110u

/* preprocessor definitions for global mif identification */
#define HW_ATL2_GLB_MIF_ID_ADR 0x0000001cu

/* preprocessor definitions for global mif identification */
#define HW_ATL2_GLB_FW_ID_ADR 0x00000018u

/* register address for bitfield iamr_lsw[1f:0] */
#define HW_ATL2_ITR_IAMRLSW_ADR 0x00002090
/* register address for bitfield rx dma drop packet counter [1f:0] */
#define HW_ATL2_RPB_RX_DMA_DROP_PKT_CNT_ADR 0x00006818

/* register address for bitfield imcr_lsw[1f:0] */
#define HW_ATL2_ITR_IMCRLSW_ADR 0x00002070
/* register address for bitfield imsr_lsw[1f:0] */
#define HW_ATL2_ITR_IMSRLSW_ADR 0x00002060
/* register address for bitfield itr_reg_res_dsbl */
#define HW_ATL2_ITR_REG_RES_DSBL_ADR 0x00002300
/* bitmask for bitfield itr_reg_res_dsbl */
#define HW_ATL2_ITR_REG_RES_DSBL_MSK 0x20000000
/* lower bit position of bitfield itr_reg_res_dsbl */
#define HW_ATL2_ITR_REG_RES_DSBL_SHIFT 29
/* register address for bitfield iscr_lsw[1f:0] */
#define HW_ATL2_ITR_ISCRLSW_ADR 0x00002050
/* register address for bitfield isr_lsw[1f:0] */
#define HW_ATL2_ITR_ISRLSW_ADR 0x00002000
/* register address for bitfield itr_reset */
#define HW_ATL2_ITR_RES_ADR 0x00002300
/* bitmask for bitfield itr_reset */
#define HW_ATL2_ITR_RES_MSK 0x80000000
/* lower bit position of bitfield itr_reset */
#define HW_ATL2_ITR_RES_SHIFT 31
/* register address for bitfield dca{d}_cpuid[7:0] */
#define HW_ATL2_RDM_DCADCPUID_ADR(dca) (0x00006100 + (dca) * 0x4)
/* bitmask for bitfield dca{d}_cpuid[7:0] */
#define HW_ATL2_RDM_DCADCPUID_MSK 0x000000ff
/* lower bit position of bitfield dca{d}_cpuid[7:0] */
#define HW_ATL2_RDM_DCADCPUID_SHIFT 0
/* register address for bitfield dca_en */
#define HW_ATL2_RDM_DCA_EN_ADR 0x00006180

/* rx dca_en bitfield definitions
 * preprocessor definitions for the bitfield "dca_en".
 * port="pif_rdm_dca_en_i"
 */

/* register address for bitfield dca_en */
#define HW_ATL2_RDM_DCA_EN_ADR 0x00006180
/* bitmask for bitfield dca_en */
#define HW_ATL2_RDM_DCA_EN_MSK 0x80000000
/* inverted bitmask for bitfield dca_en */
#define HW_ATL2_RDM_DCA_EN_MSKN 0x7fffffff
/* lower bit position of bitfield dca_en */
#define HW_ATL2_RDM_DCA_EN_SHIFT 31
/* width of bitfield dca_en */
#define HW_ATL2_RDM_DCA_EN_WIDTH 1
/* default value of bitfield dca_en */
#define HW_ATL2_RDM_DCA_EN_DEFAULT 0x1

/* rx dca_mode[3:0] bitfield definitions
 * preprocessor definitions for the bitfield "dca_mode[3:0]".
 * port="pif_rdm_dca_mode_i[3:0]"
 */

/* register address for bitfield dca_mode[3:0] */
#define HW_ATL2_RDM_DCA_MODE_ADR 0x00006180
/* bitmask for bitfield dca_mode[3:0] */
#define HW_ATL2_RDM_DCA_MODE_MSK 0x0000000f
/* inverted bitmask for bitfield dca_mode[3:0] */
#define HW_ATL2_RDM_DCA_MODE_MSKN 0xfffffff0
/* lower bit position of bitfield dca_mode[3:0] */
#define HW_ATL2_RDM_DCA_MODE_SHIFT 0
/* width of bitfield dca_mode[3:0] */
#define HW_ATL2_RDM_DCA_MODE_WIDTH 4
/* default value of bitfield dca_mode[3:0] */
#define HW_ATL2_RDM_DCA_MODE_DEFAULT 0x0

/* rx desc{d}_data_size[4:0] bitfield definitions
 * preprocessor definitions for the bitfield "desc{d}_data_size[4:0]".
 * parameter: descriptor {d} | stride size 0x20 | range [0, 31]
 * port="pif_rdm_desc0_data_size_i[4:0]"
 */

/* register address for bitfield desc{d}_data_size[4:0] */
#define HW_ATL2_RDM_DESCDDATA_SIZE_ADR(descriptor) \
	(0x00005b18 + (descriptor) * 0x20)
/* bitmask for bitfield desc{d}_data_size[4:0] */
#define HW_ATL2_RDM_DESCDDATA_SIZE_MSK 0x0000001f
/* inverted bitmask for bitfield desc{d}_data_size[4:0] */
#define HW_ATL2_RDM_DESCDDATA_SIZE_MSKN 0xffffffe0
/* lower bit position of bitfield desc{d}_data_size[4:0] */
#define HW_ATL2_RDM_DESCDDATA_SIZE_SHIFT 0
/* width of bitfield desc{d}_data_size[4:0] */
#define HW_ATL2_RDM_DESCDDATA_SIZE_WIDTH 5
/* default value of bitfield desc{d}_data_size[4:0] */
#define HW_ATL2_RDM_DESCDDATA_SIZE_DEFAULT 0x0

/* rx dca{d}_desc_en bitfield definitions
 * preprocessor definitions for the bitfield "dca{d}_desc_en".
 * parameter: dca {d} | stride size 0x4 | range [0, 31]
 * port="pif_rdm_dca_desc_en_i[0]"
 */

/* register address for bitfield dca{d}_desc_en */
#define HW_ATL2_RDM_DCADDESC_EN_ADR(dca) (0x00006100 + (dca) * 0x4)
/* bitmask for bitfield dca{d}_desc_en */
#define HW_ATL2_RDM_DCADDESC_EN_MSK 0x80000000
/* inverted bitmask for bitfield dca{d}_desc_en */
#define HW_ATL2_RDM_DCADDESC_EN_MSKN 0x7fffffff
/* lower bit position of bitfield dca{d}_desc_en */
#define HW_ATL2_RDM_DCADDESC_EN_SHIFT 31
/* width of bitfield dca{d}_desc_en */
#define HW_ATL2_RDM_DCADDESC_EN_WIDTH 1
/* default value of bitfield dca{d}_desc_en */
#define HW_ATL2_RDM_DCADDESC_EN_DEFAULT 0x0

/* rx desc{d}_en bitfield definitions
 * preprocessor definitions for the bitfield "desc{d}_en".
 * parameter: descriptor {d} | stride size 0x20 | range [0, 31]
 * port="pif_rdm_desc_en_i[0]"
 */

/* register address for bitfield desc{d}_en */
#define HW_ATL2_RDM_DESCDEN_ADR(descriptor) (0x00005b08 + (descriptor) * 0x20)
/* bitmask for bitfield desc{d}_en */
#define HW_ATL2_RDM_DESCDEN_MSK 0x80000000
/* inverted bitmask for bitfield desc{d}_en */
#define HW_ATL2_RDM_DESCDEN_MSKN 0x7fffffff
/* lower bit position of bitfield desc{d}_en */
#define HW_ATL2_RDM_DESCDEN_SHIFT 31
/* width of bitfield desc{d}_en */
#define HW_ATL2_RDM_DESCDEN_WIDTH 1
/* default value of bitfield desc{d}_en */
#define HW_ATL2_RDM_DESCDEN_DEFAULT 0x0

/* rx desc{d}_hdr_size[4:0] bitfield definitions
 * preprocessor definitions for the bitfield "desc{d}_hdr_size[4:0]".
 * parameter: descriptor {d} | stride size 0x20 | range [0, 31]
 * port="pif_rdm_desc0_hdr_size_i[4:0]"
 */

/* register address for bitfield desc{d}_hdr_size[4:0] */
#define HW_ATL2_RDM_DESCDHDR_SIZE_ADR(descriptor) \
	(0x00005b18 + (descriptor) * 0x20)
/* bitmask for bitfield desc{d}_hdr_size[4:0] */
#define HW_ATL2_RDM_DESCDHDR_SIZE_MSK 0x00001f00
/* inverted bitmask for bitfield desc{d}_hdr_size[4:0] */
#define HW_ATL2_RDM_DESCDHDR_SIZE_MSKN 0xffffe0ff
/* lower bit position of bitfield desc{d}_hdr_size[4:0] */
#define HW_ATL2_RDM_DESCDHDR_SIZE_SHIFT 8
/* width of bitfield desc{d}_hdr_size[4:0] */
#define HW_ATL2_RDM_DESCDHDR_SIZE_WIDTH 5
/* default value of bitfield desc{d}_hdr_size[4:0] */
#define HW_ATL2_RDM_DESCDHDR_SIZE_DEFAULT 0x0

/* rx desc{d}_hdr_split bitfield definitions
 * preprocessor definitions for the bitfield "desc{d}_hdr_split".
 * parameter: descriptor {d} | stride size 0x20 | range [0, 31]
 * port="pif_rdm_desc_hdr_split_i[0]"
 */

/* register address for bitfield desc{d}_hdr_split */
#define HW_ATL2_RDM_DESCDHDR_SPLIT_ADR(descriptor) \
	(0x00005b08 + (descriptor) * 0x20)
/* bitmask for bitfield desc{d}_hdr_split */
#define HW_ATL2_RDM_DESCDHDR_SPLIT_MSK 0x10000000
/* inverted bitmask for bitfield desc{d}_hdr_split */
#define HW_ATL2_RDM_DESCDHDR_SPLIT_MSKN 0xefffffff
/* lower bit position of bitfield desc{d}_hdr_split */
#define HW_ATL2_RDM_DESCDHDR_SPLIT_SHIFT 28
/* width of bitfield desc{d}_hdr_split */
#define HW_ATL2_RDM_DESCDHDR_SPLIT_WIDTH 1
/* default value of bitfield desc{d}_hdr_split */
#define HW_ATL2_RDM_DESCDHDR_SPLIT_DEFAULT 0x0

/* rx desc{d}_hd[c:0] bitfield definitions
 * preprocessor definitions for the bitfield "desc{d}_hd[c:0]".
 * parameter: descriptor {d} | stride size 0x20 | range [0, 31]
 * port="rdm_pif_desc0_hd_o[12:0]"
 */

/* register address for bitfield desc{d}_hd[c:0] */
#define HW_ATL2_RDM_DESCDHD_ADR(descriptor) (0x00005b0c + (descriptor) * 0x20)
/* bitmask for bitfield desc{d}_hd[c:0] */
#define HW_ATL2_RDM_DESCDHD_MSK 0x00001fff
/* inverted bitmask for bitfield desc{d}_hd[c:0] */
#define HW_ATL2_RDM_DESCDHD_MSKN 0xffffe000
/* lower bit position of bitfield desc{d}_hd[c:0] */
#define HW_ATL2_RDM_DESCDHD_SHIFT 0
/* width of bitfield desc{d}_hd[c:0] */
#define HW_ATL2_RDM_DESCDHD_WIDTH 13

/* rx desc{d}_len[9:0] bitfield definitions
 * preprocessor definitions for the bitfield "desc{d}_len[9:0]".
 * parameter: descriptor {d} | stride size 0x20 | range [0, 31]
 * port="pif_rdm_desc0_len_i[9:0]"
 */

/* register address for bitfield desc{d}_len[9:0] */
#define HW_ATL2_RDM_DESCDLEN_ADR(descriptor) (0x00005b08 + (descriptor) * 0x20)
/* bitmask for bitfield desc{d}_len[9:0] */
#define HW_ATL2_RDM_DESCDLEN_MSK 0x00001ff8
/* inverted bitmask for bitfield desc{d}_len[9:0] */
#define HW_ATL2_RDM_DESCDLEN_MSKN 0xffffe007
/* lower bit position of bitfield desc{d}_len[9:0] */
#define HW_ATL2_RDM_DESCDLEN_SHIFT 3
/* width of bitfield desc{d}_len[9:0] */
#define HW_ATL2_RDM_DESCDLEN_WIDTH 10
/* default value of bitfield desc{d}_len[9:0] */
#define HW_ATL2_RDM_DESCDLEN_DEFAULT 0x0

/* rx desc{d}_reset bitfield definitions
 * preprocessor definitions for the bitfield "desc{d}_reset".
 * parameter: descriptor {d} | stride size 0x20 | range [0, 31]
 * port="pif_rdm_q_pf_res_i[0]"
 */

/* bitmask for bitfield desc{d}_ */
#define HW_ATL2_RDM_DESCTSREQ_MSK 0x00030000
/* lower bit position of bitfield desc{d}_tsreq[1:0] */
#define HW_ATL2_RDM_DESCTSREQ_SHIFT 16


/* register address for bitfield desc{d}_reset */
#define HW_ATL2_RDM_DESCDRESET_ADR(descriptor) (0x00005b08 + (descriptor) * 0x20)
/* bitmask for bitfield desc{d}_reset */
#define HW_ATL2_RDM_DESCDRESET_MSK 0x02000000
/* inverted bitmask for bitfield desc{d}_reset */
#define HW_ATL2_RDM_DESCDRESET_MSKN 0xfdffffff
/* lower bit position of bitfield desc{d}_reset */
#define HW_ATL2_RDM_DESCDRESET_SHIFT 25
/* width of bitfield desc{d}_reset */
#define HW_ATL2_RDM_DESCDRESET_WIDTH 1
/* default value of bitfield desc{d}_reset */
#define HW_ATL2_RDM_DESCDRESET_DEFAULT 0x0

/* rdm_desc_init_i bitfield definitions
 * preprocessor definitions for the bitfield rdm_desc_init_i.
 * port="pif_rdm_desc_init_i"
 */

/* register address for bitfield rdm_desc_init_i */
#define HW_ATL2_RDM_RX_DMA_DESC_CACHE_INIT_ADR 0x00005a00
/* bitmask for bitfield rdm_desc_init_i */
#define HW_ATL2_RDM_RX_DMA_DESC_CACHE_INIT_MSK 0xffffffff
/* inverted bitmask for bitfield rdm_desc_init_i */
#define HW_ATL2_RDM_RX_DMA_DESC_CACHE_INIT_MSKN 0x00000000
/* lower bit position of bitfield  rdm_desc_init_i */
#define HW_ATL2_RDM_RX_DMA_DESC_CACHE_INIT_SHIFT 0
/* width of bitfield rdm_desc_init_i */
#define HW_ATL2_RDM_RX_DMA_DESC_CACHE_INIT_WIDTH 32
/* default value of bitfield rdm_desc_init_i */
#define HW_ATL2_RDM_RX_DMA_DESC_CACHE_INIT_DEFAULT 0x0

/* rx int_desc_wrb_en bitfield definitions
 * preprocessor definitions for the bitfield "int_desc_wrb_en".
 * port="pif_rdm_int_desc_wrb_en_i"
 */

/* register address for bitfield int_desc_wrb_en */
#define HW_ATL2_RDM_INT_DESC_WRB_EN_ADR 0x00005a30
/* bitmask for bitfield int_desc_wrb_en */
#define HW_ATL2_RDM_INT_DESC_WRB_EN_MSK 0x00000004
/* inverted bitmask for bitfield int_desc_wrb_en */
#define HW_ATL2_RDM_INT_DESC_WRB_EN_MSKN 0xfffffffb
/* lower bit position of bitfield int_desc_wrb_en */
#define HW_ATL2_RDM_INT_DESC_WRB_EN_SHIFT 2
/* width of bitfield int_desc_wrb_en */
#define HW_ATL2_RDM_INT_DESC_WRB_EN_WIDTH 1
/* default value of bitfield int_desc_wrb_en */
#define HW_ATL2_RDM_INT_DESC_WRB_EN_DEFAULT 0x0

/* rx dca{d}_hdr_en bitfield definitions
 * preprocessor definitions for the bitfield "dca{d}_hdr_en".
 * parameter: dca {d} | stride size 0x4 | range [0, 31]
 * port="pif_rdm_dca_hdr_en_i[0]"
 */

/* register address for bitfield dca{d}_hdr_en */
#define HW_ATL2_RDM_DCADHDR_EN_ADR(dca) (0x00006100 + (dca) * 0x4)
/* bitmask for bitfield dca{d}_hdr_en */
#define HW_ATL2_RDM_DCADHDR_EN_MSK 0x40000000
/* inverted bitmask for bitfield dca{d}_hdr_en */
#define HW_ATL2_RDM_DCADHDR_EN_MSKN 0xbfffffff
/* lower bit position of bitfield dca{d}_hdr_en */
#define HW_ATL2_RDM_DCADHDR_EN_SHIFT 30
/* width of bitfield dca{d}_hdr_en */
#define HW_ATL2_RDM_DCADHDR_EN_WIDTH 1
/* default value of bitfield dca{d}_hdr_en */
#define HW_ATL2_RDM_DCADHDR_EN_DEFAULT 0x0

/* rx dca{d}_pay_en bitfield definitions
 * preprocessor definitions for the bitfield "dca{d}_pay_en".
 * parameter: dca {d} | stride size 0x4 | range [0, 31]
 * port="pif_rdm_dca_pay_en_i[0]"
 */

/* register address for bitfield dca{d}_pay_en */
#define HW_ATL2_RDM_DCADPAY_EN_ADR(dca) (0x00006100 + (dca) * 0x4)
/* bitmask for bitfield dca{d}_pay_en */
#define HW_ATL2_RDM_DCADPAY_EN_MSK 0x20000000
/* inverted bitmask for bitfield dca{d}_pay_en */
#define HW_ATL2_RDM_DCADPAY_EN_MSKN 0xdfffffff
/* lower bit position of bitfield dca{d}_pay_en */
#define HW_ATL2_RDM_DCADPAY_EN_SHIFT 29
/* width of bitfield dca{d}_pay_en */
#define HW_ATL2_RDM_DCADPAY_EN_WIDTH 1
/* default value of bitfield dca{d}_pay_en */
#define HW_ATL2_RDM_DCADPAY_EN_DEFAULT 0x0

/* RX rdm_int_rim_en Bitfield Definitions
 * Preprocessor definitions for the bitfield "rdm_int_rim_en".
 * PORT="pif_rdm_int_rim_en_i"
 */

/* Register address for bitfield rdm_int_rim_en */
#define HW_ATL2_RDM_INT_RIM_EN_ADR 0x00005A30
/* Bitmask for bitfield rdm_int_rim_en */
#define HW_ATL2_RDM_INT_RIM_EN_MSK 0x00000008
/* Inverted bitmask for bitfield rdm_int_rim_en */
#define HW_ATL2_RDM_INT_RIM_EN_MSKN 0xFFFFFFF7
/* Lower bit position of bitfield rdm_int_rim_en */
#define HW_ATL2_RDM_INT_RIM_EN_SHIFT 3
/* Width of bitfield rdm_int_rim_en */
#define HW_ATL2_RDM_INT_RIM_EN_WIDTH 1
/* Default value of bitfield rdm_int_rim_en */
#define HW_ATL2_RDM_INT_RIM_EN_DEFAULT 0x0

/* general interrupt mapping register definitions
 * preprocessor definitions for general interrupt mapping register
 * base address: 0x00002180
 * parameter: regidx {f} | stride size 0x4 | range [0, 3]
 */
#define HW_ATL2_GEN_INTR_MAP_ADR(regidx) (0x00002180u + (regidx) * 0x4)

/* general interrupt status register definitions
 * preprocessor definitions for general interrupt status register
 * address: 0x000021A0
 */

#define HW_ATL2_GEN_INTR_STAT_ADR 0x000021A4U

/* interrupt global control register  definitions
 * preprocessor definitions for interrupt global control register
 * address: 0x00002300
 */
#define HW_ATL2_INTR_GLB_CTL_ADR 0x00002300u

/* interrupt throttle register definitions
 * preprocessor definitions for interrupt throttle register
 * base address: 0x00002800
 * parameter: throttle {t} | stride size 0x4 | range [0, 31]
 */
#define HW_ATL2_INTR_THR_ADR(throttle) (0x00002800u + (throttle) * 0x4)

/* rx dma descriptor base address lsw definitions
 * preprocessor definitions for rx dma descriptor base address lsw
 * base address: 0x00005b00
 * parameter: descriptor {d} | stride size 0x20 | range [0, 31]
 */
#define HW_ATL2_RX_DMA_DESC_BASE_ADDRLSW_ADR(descriptor) \
(0x00005b00u + (descriptor) * 0x20)

/* rx dma descriptor base address msw definitions
 * preprocessor definitions for rx dma descriptor base address msw
 * base address: 0x00005b04
 * parameter: descriptor {d} | stride size 0x20 | range [0, 31]
 */
#define HW_ATL2_RX_DMA_DESC_BASE_ADDRMSW_ADR(descriptor) \
(0x00005b04u + (descriptor) * 0x20)

/* rx dma descriptor status register definitions
 * preprocessor definitions for rx dma descriptor status register
 * base address: 0x00005b14
 * parameter: descriptor {d} | stride size 0x20 | range [0, 31]
 */
#define HW_ATL2_RX_DMA_DESC_STAT_ADR(descriptor) \
	(0x00005b14u + (descriptor) * 0x20)

/* rx dma descriptor tail pointer register definitions
 * preprocessor definitions for rx dma descriptor tail pointer register
 * base address: 0x00005b10
 * parameter: descriptor {d} | stride size 0x20 | range [0, 31]
 */
#define HW_ATL2_RX_DMA_DESC_TAIL_PTR_ADR(descriptor) \
	(0x00005b10u + (descriptor) * 0x20)

/* rx interrupt moderation control register definitions
 * Preprocessor definitions for RX Interrupt Moderation Control Register
 * Base Address: 0x00005A40
 * Parameter: RIM {R} | stride size 0x4 | range [0, 31]
 */
#define HW_ATL2_RX_INTR_MODERATION_CTL_ADR(rim) (0x00005A40u + (rim) * 0x4)

/* rx filter multicast filter mask register definitions
 * preprocessor definitions for rx filter multicast filter mask register
 * address: 0x00005270
 */
#define HW_ATL2_RX_FLR_MCST_FLR_MSK_ADR 0x00005270u

/* rx filter multicast filter register definitions
 * preprocessor definitions for rx filter multicast filter register
 * base address: 0x00005250
 * parameter: filter {f} | stride size 0x4 | range [0, 7]
 */
#define HW_ATL2_RX_FLR_MCST_FLR_ADR(filter) (0x00005250u + (filter) * 0x4)

/* RX Filter RSS Control Register 1 Definitions
 * Preprocessor definitions for RX Filter RSS Control Register 1
 * Address: 0x000054C0
 */
#define HW_ATL2_RX_FLR_RSS_CONTROL1_ADR 0x000054C0u

/* RX Filter RSS HASH Control Register Definitions
 * Preprocessor definitions for RX Filter RSS HASH Control Register
 * Address: 0x000054C8
 * port=" pif_rpf_rss_hash_type_i [8:0]"
 */
#define HW_ATL2_RX_FLR_RSS_HASH_TYPE_ADR 0x000054C8u
#define HW_ATL2_RX_FLR_RSS_HASH_TYPE_MSK 0x000001FF
#define HW_ATL2_RX_FLR_RSS_HASH_TYPE_SHIFT 0
#define HW_ATL2_RX_FLR_RSS_HASH_TYPE_WIDTH 9

/* RX Filter Control Register 2 Definitions
 * Preprocessor definitions for RX Filter Control Register 2
 * Address: 0x00005104
 */
#define HW_ATL2_RX_FLR_CONTROL2_ADR 0x00005104u
#define HW_ATL2_RX_FLR_NEW_RPF_EN_MSK 0x00000800
#define HW_ATL2_RX_FLR_NEW_RPF_EN_SHIFT 0xB

/* tx tx dma debug control [1f:0] bitfield definitions
 * preprocessor definitions for the bitfield "tx dma debug control [1f:0]".
 * port="pif_tdm_debug_cntl_i[31:0]"
 */

/* register address for bitfield tx dma debug control [1f:0] */
#define HW_ATL2_TDM_TX_DMA_DEBUG_CTL_ADR 0x00008920
/* bitmask for bitfield tx dma debug control [1f:0] */
#define HW_ATL2_TDM_TX_DMA_DEBUG_CTL_MSK 0xffffffff
/* inverted bitmask for bitfield tx dma debug control [1f:0] */
#define HW_ATL2_TDM_TX_DMA_DEBUG_CTL_MSKN 0x00000000
/* lower bit position of bitfield tx dma debug control [1f:0] */
#define HW_ATL2_TDM_TX_DMA_DEBUG_CTL_SHIFT 0
/* width of bitfield tx dma debug control [1f:0] */
#define HW_ATL2_TDM_TX_DMA_DEBUG_CTL_WIDTH 32
/* default value of bitfield tx dma debug control [1f:0] */
#define HW_ATL2_TDM_TX_DMA_DEBUG_CTL_DEFAULT 0x0

/* tx dma descriptor base address lsw definitions
 * preprocessor definitions for tx dma descriptor base address lsw
 * base address: 0x00007c00
 * parameter: descriptor {d} | stride size 0x40 | range [0, 31]
 */
#define HW_ATL2_TX_DMA_DESC_BASE_ADDRLSW_ADR(descriptor) \
	(0x00007c00u + (descriptor) * 0x40)

/* tx dma descriptor tail pointer register definitions
 * preprocessor definitions for tx dma descriptor tail pointer register
 * base address: 0x00007c10
 *  parameter: descriptor {d} | stride size 0x40 | range [0, 31]
 */
#define HW_ATL2_TX_DMA_DESC_TAIL_PTR_ADR(descriptor) \
	(0x00007c10u + (descriptor) * 0x40)

/* rx dma_sys_loopback bitfield definitions
 * preprocessor definitions for the bitfield "dma_sys_loopback".
 * port="pif_rpb_dma_sys_lbk_i"
 */

/* register address for bitfield dma_sys_loopback */
#define HW_ATL2_RPB_DMA_SYS_LBK_ADR 0x00005000
/* bitmask for bitfield dma_sys_loopback */
#define HW_ATL2_RPB_DMA_SYS_LBK_MSK 0x00000040
/* inverted bitmask for bitfield dma_sys_loopback */
#define HW_ATL2_RPB_DMA_SYS_LBK_MSKN 0xffffffbf
/* lower bit position of bitfield dma_sys_loopback */
#define HW_ATL2_RPB_DMA_SYS_LBK_SHIFT 6
/* width of bitfield dma_sys_loopback */
#define HW_ATL2_RPB_DMA_SYS_LBK_WIDTH 1
/* default value of bitfield dma_sys_loopback */
#define HW_ATL2_RPB_DMA_SYS_LBK_DEFAULT 0x0

/* rx dma_net_loopback bitfield definitions
 * preprocessor definitions for the bitfield "dma_net_loopback".
 * port="pif_rpb_dma_net_lbk_i"
 */

/* register address for bitfield dma_net_loopback */
#define HW_ATL2_RPB_DMA_NET_LBK_ADR 0x00005000
/* bitmask for bitfield dma_net_loopback */
#define HW_ATL2_RPB_DMA_NET_LBK_MSK 0x00000010
/* inverted bitmask for bitfield dma_net_loopback */
#define HW_ATL2_RPB_DMA_NET_LBK_MSKN 0xffffffef
/* lower bit position of bitfield dma_net_loopback */
#define HW_ATL2_RPB_DMA_NET_LBK_SHIFT 4
/* width of bitfield dma_net_loopback */
#define HW_ATL2_RPB_DMA_NET_LBK_WIDTH 1
/* default value of bitfield dma_net_loopback */
#define HW_ATL2_RPB_DMA_NET_LBK_DEFAULT 0x0

/* rx rx_tc_mode bitfield definitions
 * preprocessor definitions for the bitfield "rx_tc_mode".
 * port="pif_rpb_rx_tc_mode_i,pif_rpf_rx_tc_mode_i"
 */

/* register address for bitfield rx_tc_mode */
#define HW_ATL2_RPB_RPF_RX_TC_MODE_ADR 0x00005700
/* bitmask for bitfield rx_tc_mode */
#define HW_ATL2_RPB_RPF_RX_TC_MODE_MSK 0x00000100
/* inverted bitmask for bitfield rx_tc_mode */
#define HW_ATL2_RPB_RPF_RX_TC_MODE_MSKN 0xfffffeff
/* lower bit position of bitfield rx_tc_mode */
#define HW_ATL2_RPB_RPF_RX_TC_MODE_SHIFT 8
/* width of bitfield rx_tc_mode */
#define HW_ATL2_RPB_RPF_RX_TC_MODE_WIDTH 1
/* default value of bitfield rx_tc_mode */
#define HW_ATL2_RPB_RPF_RX_TC_MODE_DEFAULT 0x0

/* rx rx_buf_en bitfield definitions
 * preprocessor definitions for the bitfield "rx_buf_en".
 * port="pif_rpb_rx_buf_en_i"
 */

/* register address for bitfield rx_buf_en */
#define HW_ATL2_RPB_RX_BUF_EN_ADR 0x00005700
/* bitmask for bitfield rx_buf_en */
#define HW_ATL2_RPB_RX_BUF_EN_MSK 0x00000001
/* inverted bitmask for bitfield rx_buf_en */
#define HW_ATL2_RPB_RX_BUF_EN_MSKN 0xfffffffe
/* lower bit position of bitfield rx_buf_en */
#define HW_ATL2_RPB_RX_BUF_EN_SHIFT 0
/* width of bitfield rx_buf_en */
#define HW_ATL2_RPB_RX_BUF_EN_WIDTH 1
/* default value of bitfield rx_buf_en */
#define HW_ATL2_RPB_RX_BUF_EN_DEFAULT 0x0

/* rx rx{b}_hi_thresh[d:0] bitfield definitions
 * preprocessor definitions for the bitfield "rx{b}_hi_thresh[d:0]".
 * parameter: buffer {b} | stride size 0x10 | range [0, 7]
 * port="pif_rpb_rx0_hi_thresh_i[13:0]"
 */

/* register address for bitfield rx{b}_hi_thresh[d:0] */
#define HW_ATL2_RPB_RXBHI_THRESH_ADR(buffer) (0x00005714 + (buffer) * 0x10)
/* bitmask for bitfield rx{b}_hi_thresh[d:0] */
#define HW_ATL2_RPB_RXBHI_THRESH_MSK 0x3fff0000
/* inverted bitmask for bitfield rx{b}_hi_thresh[d:0] */
#define HW_ATL2_RPB_RXBHI_THRESH_MSKN 0xc000ffff
/* lower bit position of bitfield rx{b}_hi_thresh[d:0] */
#define HW_ATL2_RPB_RXBHI_THRESH_SHIFT 16
/* width of bitfield rx{b}_hi_thresh[d:0] */
#define HW_ATL2_RPB_RXBHI_THRESH_WIDTH 14
/* default value of bitfield rx{b}_hi_thresh[d:0] */
#define HW_ATL2_RPB_RXBHI_THRESH_DEFAULT 0x0

/* rx rx{b}_lo_thresh[d:0] bitfield definitions
 * preprocessor definitions for the bitfield "rx{b}_lo_thresh[d:0]".
 * parameter: buffer {b} | stride size 0x10 | range [0, 7]
 * port="pif_rpb_rx0_lo_thresh_i[13:0]"
 */

/* register address for bitfield rx{b}_lo_thresh[d:0] */
#define HW_ATL2_RPB_RXBLO_THRESH_ADR(buffer) (0x00005714 + (buffer) * 0x10)
/* bitmask for bitfield rx{b}_lo_thresh[d:0] */
#define HW_ATL2_RPB_RXBLO_THRESH_MSK 0x00003fff
/* inverted bitmask for bitfield rx{b}_lo_thresh[d:0] */
#define HW_ATL2_RPB_RXBLO_THRESH_MSKN 0xffffc000
/* lower bit position of bitfield rx{b}_lo_thresh[d:0] */
#define HW_ATL2_RPB_RXBLO_THRESH_SHIFT 0
/* width of bitfield rx{b}_lo_thresh[d:0] */
#define HW_ATL2_RPB_RXBLO_THRESH_WIDTH 14
/* default value of bitfield rx{b}_lo_thresh[d:0] */
#define HW_ATL2_RPB_RXBLO_THRESH_DEFAULT 0x0

/* rx rx_fc_mode[1:0] bitfield definitions
 * preprocessor definitions for the bitfield "rx_fc_mode[1:0]".
 * port="pif_rpb_rx_fc_mode_i[1:0]"
 */

/* register address for bitfield rx_fc_mode[1:0] */
#define HW_ATL2_RPB_RX_FC_MODE_ADR 0x00005700
/* bitmask for bitfield rx_fc_mode[1:0] */
#define HW_ATL2_RPB_RX_FC_MODE_MSK 0x00000030
/* inverted bitmask for bitfield rx_fc_mode[1:0] */
#define HW_ATL2_RPB_RX_FC_MODE_MSKN 0xffffffcf
/* lower bit position of bitfield rx_fc_mode[1:0] */
#define HW_ATL2_RPB_RX_FC_MODE_SHIFT 4
/* width of bitfield rx_fc_mode[1:0] */
#define HW_ATL2_RPB_RX_FC_MODE_WIDTH 2
/* default value of bitfield rx_fc_mode[1:0] */
#define HW_ATL2_RPB_RX_FC_MODE_DEFAULT 0x0

/* rx rx{b}_buf_size[8:0] bitfield definitions
 * preprocessor definitions for the bitfield "rx{b}_buf_size[8:0]".
 * parameter: buffer {b} | stride size 0x10 | range [0, 7]
 * port="pif_rpb_rx0_buf_size_i[8:0]"
 */

/* register address for bitfield rx{b}_buf_size[8:0] */
#define HW_ATL2_RPB_RXBBUF_SIZE_ADR(buffer) (0x00005710 + (buffer) * 0x10)
/* bitmask for bitfield rx{b}_buf_size[8:0] */
#define HW_ATL2_RPB_RXBBUF_SIZE_MSK 0x000001ff
/* inverted bitmask for bitfield rx{b}_buf_size[8:0] */
#define HW_ATL2_RPB_RXBBUF_SIZE_MSKN 0xfffffe00
/* lower bit position of bitfield rx{b}_buf_size[8:0] */
#define HW_ATL2_RPB_RXBBUF_SIZE_SHIFT 0
/* width of bitfield rx{b}_buf_size[8:0] */
#define HW_ATL2_RPB_RXBBUF_SIZE_WIDTH 9
/* default value of bitfield rx{b}_buf_size[8:0] */
#define HW_ATL2_RPB_RXBBUF_SIZE_DEFAULT 0x0

/* rx rx{b}_xoff_en bitfield definitions
 * preprocessor definitions for the bitfield "rx{b}_xoff_en".
 * parameter: buffer {b} | stride size 0x10 | range [0, 7]
 * port="pif_rpb_rx_xoff_en_i[0]"
 */

/* register address for bitfield rx{b}_xoff_en */
#define HW_ATL2_RPB_RXBXOFF_EN_ADR(buffer) (0x00005714 + (buffer) * 0x10)
/* bitmask for bitfield rx{b}_xoff_en */
#define HW_ATL2_RPB_RXBXOFF_EN_MSK 0x80000000
/* inverted bitmask for bitfield rx{b}_xoff_en */
#define HW_ATL2_RPB_RXBXOFF_EN_MSKN 0x7fffffff
/* lower bit position of bitfield rx{b}_xoff_en */
#define HW_ATL2_RPB_RXBXOFF_EN_SHIFT 31
/* width of bitfield rx{b}_xoff_en */
#define HW_ATL2_RPB_RXBXOFF_EN_WIDTH 1
/* default value of bitfield rx{b}_xoff_en */
#define HW_ATL2_RPB_RXBXOFF_EN_DEFAULT 0x0

/* rx l2_bc_thresh[f:0] bitfield definitions
 * preprocessor definitions for the bitfield "l2_bc_thresh[f:0]".
 * port="pif_rpf_l2_bc_thresh_i[15:0]"
 */

/* register address for bitfield l2_bc_thresh[f:0] */
#define HW_ATL2_RPFL2BC_THRESH_ADR 0x00005100
/* bitmask for bitfield l2_bc_thresh[f:0] */
#define HW_ATL2_RPFL2BC_THRESH_MSK 0xffff0000
/* inverted bitmask for bitfield l2_bc_thresh[f:0] */
#define HW_ATL2_RPFL2BC_THRESH_MSKN 0x0000ffff
/* lower bit position of bitfield l2_bc_thresh[f:0] */
#define HW_ATL2_RPFL2BC_THRESH_SHIFT 16
/* width of bitfield l2_bc_thresh[f:0] */
#define HW_ATL2_RPFL2BC_THRESH_WIDTH 16
/* default value of bitfield l2_bc_thresh[f:0] */
#define HW_ATL2_RPFL2BC_THRESH_DEFAULT 0x0

/* rx l2_bc_en bitfield definitions
 * preprocessor definitions for the bitfield "l2_bc_en".
 * port="pif_rpf_l2_bc_en_i"
 */

/* register address for bitfield l2_bc_en */
#define HW_ATL2_RPFL2BC_EN_ADR 0x00005100
/* bitmask for bitfield l2_bc_en */
#define HW_ATL2_RPFL2BC_EN_MSK 0x00000001
/* inverted bitmask for bitfield l2_bc_en */
#define HW_ATL2_RPFL2BC_EN_MSKN 0xfffffffe
/* lower bit position of bitfield l2_bc_en */
#define HW_ATL2_RPFL2BC_EN_SHIFT 0
/* width of bitfield l2_bc_en */
#define HW_ATL2_RPFL2BC_EN_WIDTH 1
/* default value of bitfield l2_bc_en */
#define HW_ATL2_RPFL2BC_EN_DEFAULT 0x0

/* rx l2_bc_act[2:0] bitfield definitions
 * preprocessor definitions for the bitfield "l2_bc_act[2:0]".
 * port="pif_rpf_l2_bc_act_i[2:0]"
 */

/* register address for bitfield l2_bc_act[2:0] */
#define HW_ATL2_RPFL2BC_ACT_ADR 0x00005100
/* bitmask for bitfield l2_bc_act[2:0] */
#define HW_ATL2_RPFL2BC_ACT_MSK 0x00007000
/* inverted bitmask for bitfield l2_bc_act[2:0] */
#define HW_ATL2_RPFL2BC_ACT_MSKN 0xffff8fff
/* lower bit position of bitfield l2_bc_act[2:0] */
#define HW_ATL2_RPFL2BC_ACT_SHIFT 12
/* width of bitfield l2_bc_act[2:0] */
#define HW_ATL2_RPFL2BC_ACT_WIDTH 3
/* default value of bitfield l2_bc_act[2:0] */
#define HW_ATL2_RPFL2BC_ACT_DEFAULT 0x0

/* rx rpf_new_rpf_en bitfield definitions
 * preprocessor definitions for the bitfield "rpf_new_rpf_en_i".
 * port="pif_rpf_new_rpf_en_i
 */

/* register address for bitfield rpf_new_rpf_en */
#define HW_ATL2_RPF_NEW_EN_ADR 0x00005104
/* bitmask for bitfield rpf_new_rpf_en */
#define HW_ATL2_RPF_NEW_EN_MSK 0x00000800
/* inverted bitmask for bitfield rpf_new_rpf_en */
#define HW_ATL2_RPF_NEW_EN_MSKN 0xfffff7ff
/* lower bit position of bitfield rpf_new_rpf_en */
#define HW_ATL2_RPF_NEW_EN_SHIFT 11
/* width of bitfield rpf_new_rpf_en */
#define HW_ATL2_RPF_NEW_EN_WIDTH 1
/* default value of bitfield rpf_new_rpf_en */
#define HW_ATL2_RPF_NEW_EN_DEFAULT 0x0

/* rx l2_mc_en{f} bitfield definitions
 * preprocessor definitions for the bitfield "l2_mc_en{f}".
 * parameter: filter {f} | stride size 0x4 | range [0, 7]
 * port="pif_rpf_l2_mc_en_i[0]"
 */

/* register address for bitfield l2_mc_en{f} */
#define HW_ATL2_RPFL2MC_ENF_ADR(filter) (0x00005250 + (filter) * 0x4)
/* bitmask for bitfield l2_mc_en{f} */
#define HW_ATL2_RPFL2MC_ENF_MSK 0x80000000
/* inverted bitmask for bitfield l2_mc_en{f} */
#define HW_ATL2_RPFL2MC_ENF_MSKN 0x7fffffff
/* lower bit position of bitfield l2_mc_en{f} */
#define HW_ATL2_RPFL2MC_ENF_SHIFT 31
/* width of bitfield l2_mc_en{f} */
#define HW_ATL2_RPFL2MC_ENF_WIDTH 1
/* default value of bitfield l2_mc_en{f} */
#define HW_ATL2_RPFL2MC_ENF_DEFAULT 0x0

/* rx l2_promis_mode bitfield definitions
 * preprocessor definitions for the bitfield "l2_promis_mode".
 * port="pif_rpf_l2_promis_mode_i"
 */

/* register address for bitfield l2_promis_mode */
#define HW_ATL2_RPFL2PROMIS_MODE_ADR 0x00005100
/* bitmask for bitfield l2_promis_mode */
#define HW_ATL2_RPFL2PROMIS_MODE_MSK 0x00000008
/* inverted bitmask for bitfield l2_promis_mode */
#define HW_ATL2_RPFL2PROMIS_MODE_MSKN 0xfffffff7
/* lower bit position of bitfield l2_promis_mode */
#define HW_ATL2_RPFL2PROMIS_MODE_SHIFT 3
/* width of bitfield l2_promis_mode */
#define HW_ATL2_RPFL2PROMIS_MODE_WIDTH 1
/* default value of bitfield l2_promis_mode */
#define HW_ATL2_RPFL2PROMIS_MODE_DEFAULT 0x0

/* rx l2_uc_act{f}[2:0] bitfield definitions
 * preprocessor definitions for the bitfield "l2_uc_act{f}[2:0]".
 * parameter: filter {f} | stride size 0x8 | range [0, 37]
 * port="pif_rpf_l2_uc_act0_i[2:0]"
 */

/* register address for bitfield l2_uc_act{f}[2:0] */
#define HW_ATL2_RPFL2UC_ACTF_ADR(filter) (0x00005114 + (filter) * 0x8)
/* bitmask for bitfield l2_uc_act{f}[2:0] */
#define HW_ATL2_RPFL2UC_ACTF_MSK 0x00070000
/* inverted bitmask for bitfield l2_uc_act{f}[2:0] */
#define HW_ATL2_RPFL2UC_ACTF_MSKN 0xfff8ffff
/* lower bit position of bitfield l2_uc_act{f}[2:0] */
#define HW_ATL2_RPFL2UC_ACTF_SHIFT 16
/* width of bitfield l2_uc_act{f}[2:0] */
#define HW_ATL2_RPFL2UC_ACTF_WIDTH 3
/* default value of bitfield l2_uc_act{f}[2:0] */
#define HW_ATL2_RPFL2UC_ACTF_DEFAULT 0x0

/* rx l2_uc_req_tag0{f}[5:0] bitfield definitions
 * preprocessor definitions for the bitfield "l2_uc_req_tag0{f}[7:0]".
 * parameter: filter {f} | stride size 0x8 | range [0, 37]
 * port="pif_rpf_l2_uc_req_tag0[5:0]"
 */

/* register address for bitfield l2_uc_req_tag0{f}[2:0] */
#define HW_ATL2_RPFL2UC_TAG_ADR(filter) (0x00005114 + (filter) * 0x8)
/* bitmask for bitfield l2_uc_req_tag0{f}[2:0] */
#define HW_ATL2_RPFL2UC_TAG_MSK 0x0FC00000
/* inverted bitmask for bitfield l2_uc_req_tag0{f}[2:0] */
#define HW_ATL2_RPFL2UC_TAG_MSKN 0xF03FFFFF
/* lower bit position of bitfield l2_uc_req_tag0{f}[2:0] */
#define HW_ATL2_RPFL2UC_TAG_SHIFT 22
/* width of bitfield l2_uc_req_tag0{f}[2:0] */
#define HW_ATL2_RPFL2UC_TAG_WIDTH 6
/* default value of bitfield l2_uc_req_tag0{f}[2:0] */
#define HW_ATL2_RPFL2UC_TAG_DEFAULT 0x0

/* rx l2_uc_en{f} bitfield definitions
 * preprocessor definitions for the bitfield "l2_uc_en{f}".
 * parameter: filter {f} | stride size 0x8 | range [0, 37]
 * port="pif_rpf_l2_uc_en_i[0]"
 */

/* register address for bitfield l2_uc_en{f} */
#define HW_ATL2_RPFL2UC_ENF_ADR(filter) (0x00005114 + (filter) * 0x8)
/* bitmask for bitfield l2_uc_en{f} */
#define HW_ATL2_RPFL2UC_ENF_MSK 0x80000000
/* inverted bitmask for bitfield l2_uc_en{f} */
#define HW_ATL2_RPFL2UC_ENF_MSKN 0x7fffffff
/* lower bit position of bitfield l2_uc_en{f} */
#define HW_ATL2_RPFL2UC_ENF_SHIFT 31
/* width of bitfield l2_uc_en{f} */
#define HW_ATL2_RPFL2UC_ENF_WIDTH 1
/* default value of bitfield l2_uc_en{f} */
#define HW_ATL2_RPFL2UC_ENF_DEFAULT 0x0

/* register address for bitfield l2_uc_da{f}_lsw[1f:0] */
#define HW_ATL2_RPFL2UC_DAFLSW_ADR(filter) (0x00005110 + (filter) * 0x8)
/* register address for bitfield l2_uc_da{f}_msw[f:0] */
#define HW_ATL2_RPFL2UC_DAFMSW_ADR(filter) (0x00005114 + (filter) * 0x8)
/* bitmask for bitfield l2_uc_da{f}_msw[f:0] */
#define HW_ATL2_RPFL2UC_DAFMSW_MSK 0x0000ffff
/* lower bit position of bitfield l2_uc_da{f}_msw[f:0] */
#define HW_ATL2_RPFL2UC_DAFMSW_SHIFT 0

/* rx l2_mc_accept_all bitfield definitions
 * Preprocessor definitions for the bitfield "l2_mc_accept_all".
 * PORT="pif_rpf_l2_mc_all_accept_i"
 */

/* Register address for bitfield l2_mc_accept_all */
#define HW_ATL2_RPFL2MC_ACCEPT_ALL_ADR 0x00005270
/* Bitmask for bitfield l2_mc_accept_all */
#define HW_ATL2_RPFL2MC_ACCEPT_ALL_MSK 0x00004000
/* Inverted bitmask for bitfield l2_mc_accept_all */
#define HW_ATL2_RPFL2MC_ACCEPT_ALL_MSKN 0xFFFFBFFF
/* Lower bit position of bitfield l2_mc_accept_all */
#define HW_ATL2_RPFL2MC_ACCEPT_ALL_SHIFT 14
/* Width of bitfield l2_mc_accept_all */
#define HW_ATL2_RPFL2MC_ACCEPT_ALL_WIDTH 1
/* Default value of bitfield l2_mc_accept_all */
#define HW_ATL2_RPFL2MC_ACCEPT_ALL_DEFAULT 0x0

/* rpf_l2_bc_req_tag[5:0] bitfield definitions
 * preprocessor definitions for the bitfield "rpf_l2_bc_req_tag[5:0]".
 * port="pifrpf_l2_bc_req_tag_i[5:0]"
 */

/* register address for bitfield rpf_l2_bc_req_tag */
#define HW_ATL2_RPF_L2_BC_TAG_ADR 0x000050F0
/* bitmask for bitfield rpf_l2_bc_req_tag */
#define HW_ATL2_RPF_L2_BC_TAG_MSK 0x0000003F
/* inverted bitmask for bitfield rpf_l2_bc_req_tag */
#define HW_ATL2_RPF_L2_BC_TAG_MSKN 0xffffffc0
/* lower bit position of bitfield rpf_l2_bc_req_tag */
#define HW_ATL2_RPF_L2_BC_TAG_SHIFT 0
/* width of bitfield rpf_l2_bc_req_tag */
#define HW_ATL2_RPF_L2_BC_TAG_WIDTH 6
/* default value of bitfield rpf_l2_bc_req_tag */
#define HW_ATL2_RPF_L2_BC_TAG_DEFAULT 0x0

/* width of bitfield rx_tc_up{t}[2:0] */
#define HW_ATL2_RPF_RPB_RX_TC_UPT_WIDTH 3
/* default value of bitfield rx_tc_up{t}[2:0] */
#define HW_ATL2_RPF_RPB_RX_TC_UPT_DEFAULT 0x0

/* rx rss_key_addr[4:0] bitfield definitions
 * preprocessor definitions for the bitfield "rss_key_addr[4:0]".
 * port="pif_rpf_rss_key_addr_i[4:0]"
 */

/* register address for bitfield rss_key_addr[4:0] */
#define HW_ATL2_RPF_RSS_KEY_ADDR_ADR 0x000054d0
/* bitmask for bitfield rss_key_addr[4:0] */
#define HW_ATL2_RPF_RSS_KEY_ADDR_MSK 0x0000001f
/* inverted bitmask for bitfield rss_key_addr[4:0] */
#define HW_ATL2_RPF_RSS_KEY_ADDR_MSKN 0xffffffe0
/* lower bit position of bitfield rss_key_addr[4:0] */
#define HW_ATL2_RPF_RSS_KEY_ADDR_SHIFT 0
/* width of bitfield rss_key_addr[4:0] */
#define HW_ATL2_RPF_RSS_KEY_ADDR_WIDTH 5
/* default value of bitfield rss_key_addr[4:0] */
#define HW_ATL2_RPF_RSS_KEY_ADDR_DEFAULT 0x0

/* rx rss_key_wr_data[1f:0] bitfield definitions
 * preprocessor definitions for the bitfield "rss_key_wr_data[1f:0]".
 * port="pif_rpf_rss_key_wr_data_i[31:0]"
 */

/* register address for bitfield rss_key_wr_data[1f:0] */
#define HW_ATL2_RPF_RSS_KEY_WR_DATA_ADR 0x000054d4
/* bitmask for bitfield rss_key_wr_data[1f:0] */
#define HW_ATL2_RPF_RSS_KEY_WR_DATA_MSK 0xffffffff
/* inverted bitmask for bitfield rss_key_wr_data[1f:0] */
#define HW_ATL2_RPF_RSS_KEY_WR_DATA_MSKN 0x00000000
/* lower bit position of bitfield rss_key_wr_data[1f:0] */
#define HW_ATL2_RPF_RSS_KEY_WR_DATA_SHIFT 0
/* width of bitfield rss_key_wr_data[1f:0] */
#define HW_ATL2_RPF_RSS_KEY_WR_DATA_WIDTH 32
/* default value of bitfield rss_key_wr_data[1f:0] */
#define HW_ATL2_RPF_RSS_KEY_WR_DATA_DEFAULT 0x0

/* rx rss_key_wr_en_i bitfield definitions
 * preprocessor definitions for the bitfield "rss_key_wr_en_i".
 * port="pif_rpf_rss_key_wr_en_i"
 */

/* register address for bitfield rss_key_wr_en_i */
#define HW_ATL2_RPF_RSS_KEY_WR_ENI_ADR 0x000054d0
/* bitmask for bitfield rss_key_wr_en_i */
#define HW_ATL2_RPF_RSS_KEY_WR_ENI_MSK 0x00000020
/* inverted bitmask for bitfield rss_key_wr_en_i */
#define HW_ATL2_RPF_RSS_KEY_WR_ENI_MSKN 0xffffffdf
/* lower bit position of bitfield rss_key_wr_en_i */
#define HW_ATL2_RPF_RSS_KEY_WR_ENI_SHIFT 5
/* width of bitfield rss_key_wr_en_i */
#define HW_ATL2_RPF_RSS_KEY_WR_ENI_WIDTH 1
/* default value of bitfield rss_key_wr_en_i */
#define HW_ATL2_RPF_RSS_KEY_WR_ENI_DEFAULT 0x0

/* rx rss_redir_addr[3:0] bitfield definitions
 * preprocessor definitions for the bitfield "rss_redir_addr[3:0]".
 * port="pif_rpf_rss_redir_addr_i[3:0]"
 */

/* register address for bitfield rss_redir_addr[3:0] */
#define HW_ATL2_RPF_RSS_REDIR_ADDR_ADR 0x000054e0
/* bitmask for bitfield rss_redir_addr[3:0] */
#define HW_ATL2_RPF_RSS_REDIR_ADDR_MSK 0x0000000f
/* inverted bitmask for bitfield rss_redir_addr[3:0] */
#define HW_ATL2_RPF_RSS_REDIR_ADDR_MSKN 0xfffffff0
/* lower bit position of bitfield rss_redir_addr[3:0] */
#define HW_ATL2_RPF_RSS_REDIR_ADDR_SHIFT 0
/* width of bitfield rss_redir_addr[3:0] */
#define HW_ATL2_RPF_RSS_REDIR_ADDR_WIDTH 4
/* default value of bitfield rss_redir_addr[3:0] */
#define HW_ATL2_RPF_RSS_REDIR_ADDR_DEFAULT 0x0

/* rx rss_redir_wr_data[f:0] bitfield definitions
 * preprocessor definitions for the bitfield "rss_redir_wr_data[f:0]".
 * port="pif_rpf_rss_redir_wr_data_i[15:0]"
 */

/* register address for bitfield rss_redir_wr_data[f:0] */
#define HW_ATL2_RPF_RSS_REDIR_WR_DATA_ADR 0x000054e4
/* bitmask for bitfield rss_redir_wr_data[f:0] */
#define HW_ATL2_RPF_RSS_REDIR_WR_DATA_MSK 0x0000ffff
/* inverted bitmask for bitfield rss_redir_wr_data[f:0] */
#define HW_ATL2_RPF_RSS_REDIR_WR_DATA_MSKN 0xffff0000
/* lower bit position of bitfield rss_redir_wr_data[f:0] */
#define HW_ATL2_RPF_RSS_REDIR_WR_DATA_SHIFT 0
/* width of bitfield rss_redir_wr_data[f:0] */
#define HW_ATL2_RPF_RSS_REDIR_WR_DATA_WIDTH 16
/* default value of bitfield rss_redir_wr_data[f:0] */
#define HW_ATL2_RPF_RSS_REDIR_WR_DATA_DEFAULT 0x0

/* rx rss_redir_wr_en_i bitfield definitions
 * preprocessor definitions for the bitfield "rss_redir_wr_en_i".
 * port="pif_rpf_rss_redir_wr_en_i"
 */

/* register address for bitfield rss_redir_wr_en_i */
#define HW_ATL2_RPF_RSS_REDIR_WR_ENI_ADR 0x000054e0
/* bitmask for bitfield rss_redir_wr_en_i */
#define HW_ATL2_RPF_RSS_REDIR_WR_ENI_MSK 0x00000010
/* inverted bitmask for bitfield rss_redir_wr_en_i */
#define HW_ATL2_RPF_RSS_REDIR_WR_ENI_MSKN 0xffffffef
/* lower bit position of bitfield rss_redir_wr_en_i */
#define HW_ATL2_RPF_RSS_REDIR_WR_ENI_SHIFT 4
/* width of bitfield rss_redir_wr_en_i */
#define HW_ATL2_RPF_RSS_REDIR_WR_ENI_WIDTH 1
/* default value of bitfield rss_redir_wr_en_i */
#define HW_ATL2_RPF_RSS_REDIR_WR_ENI_DEFAULT 0x0

/* rx tpo_rpf_sys_loopback bitfield definitions
 * preprocessor definitions for the bitfield "tpo_rpf_sys_loopback".
 * port="pif_rpf_tpo_pkt_sys_lbk_i"
 */

/* register address for bitfield tpo_rpf_sys_loopback */
#define HW_ATL2_RPF_TPO_RPF_SYS_LBK_ADR 0x00005000
/* bitmask for bitfield tpo_rpf_sys_loopback */
#define HW_ATL2_RPF_TPO_RPF_SYS_LBK_MSK 0x00000100
/* inverted bitmask for bitfield tpo_rpf_sys_loopback */
#define HW_ATL2_RPF_TPO_RPF_SYS_LBK_MSKN 0xfffffeff
/* lower bit position of bitfield tpo_rpf_sys_loopback */
#define HW_ATL2_RPF_TPO_RPF_SYS_LBK_SHIFT 8
/* width of bitfield tpo_rpf_sys_loopback */
#define HW_ATL2_RPF_TPO_RPF_SYS_LBK_WIDTH 1
/* default value of bitfield tpo_rpf_sys_loopback */
#define HW_ATL2_RPF_TPO_RPF_SYS_LBK_DEFAULT 0x0

/* rx vl_inner_tpid[f:0] bitfield definitions
 * preprocessor definitions for the bitfield "vl_inner_tpid[f:0]".
 * port="pif_rpf_vl_inner_tpid_i[15:0]"
 */

/* register address for bitfield vl_inner_tpid[f:0] */
#define HW_ATL2_RPF_VL_INNER_TPID_ADR 0x00005284
/* bitmask for bitfield vl_inner_tpid[f:0] */
#define HW_ATL2_RPF_VL_INNER_TPID_MSK 0x0000ffff
/* inverted bitmask for bitfield vl_inner_tpid[f:0] */
#define HW_ATL2_RPF_VL_INNER_TPID_MSKN 0xffff0000
/* lower bit position of bitfield vl_inner_tpid[f:0] */
#define HW_ATL2_RPF_VL_INNER_TPID_SHIFT 0
/* width of bitfield vl_inner_tpid[f:0] */
#define HW_ATL2_RPF_VL_INNER_TPID_WIDTH 16
/* default value of bitfield vl_inner_tpid[f:0] */
#define HW_ATL2_RPF_VL_INNER_TPID_DEFAULT 0x8100

/* rx vl_outer_tpid[f:0] bitfield definitions
 * preprocessor definitions for the bitfield "vl_outer_tpid[f:0]".
 * port="pif_rpf_vl_outer_tpid_i[15:0]"
 */

/* register address for bitfield vl_outer_tpid[f:0] */
#define HW_ATL2_RPF_VL_OUTER_TPID_ADR 0x00005284
/* bitmask for bitfield vl_outer_tpid[f:0] */
#define HW_ATL2_RPF_VL_OUTER_TPID_MSK 0xffff0000
/* inverted bitmask for bitfield vl_outer_tpid[f:0] */
#define HW_ATL2_RPF_VL_OUTER_TPID_MSKN 0x0000ffff
/* lower bit position of bitfield vl_outer_tpid[f:0] */
#define HW_ATL2_RPF_VL_OUTER_TPID_SHIFT 16
/* width of bitfield vl_outer_tpid[f:0] */
#define HW_ATL2_RPF_VL_OUTER_TPID_WIDTH 16
/* default value of bitfield vl_outer_tpid[f:0] */
#define HW_ATL2_RPF_VL_OUTER_TPID_DEFAULT 0x88a8

/* rx vl_promis_mode bitfield definitions
 * preprocessor definitions for the bitfield "vl_promis_mode".
 * port="pif_rpf_vl_promis_mode_i"
 */

/* register address for bitfield vl_promis_mode */
#define HW_ATL2_RPF_VL_PROMIS_MODE_ADR 0x00005280
/* bitmask for bitfield vl_promis_mode */
#define HW_ATL2_RPF_VL_PROMIS_MODE_MSK 0x00000002
/* inverted bitmask for bitfield vl_promis_mode */
#define HW_ATL2_RPF_VL_PROMIS_MODE_MSKN 0xfffffffd
/* lower bit position of bitfield vl_promis_mode */
#define HW_ATL2_RPF_VL_PROMIS_MODE_SHIFT 1
/* width of bitfield vl_promis_mode */
#define HW_ATL2_RPF_VL_PROMIS_MODE_WIDTH 1
/* default value of bitfield vl_promis_mode */
#define HW_ATL2_RPF_VL_PROMIS_MODE_DEFAULT 0x0

/* RX vl_accept_untagged_mode Bitfield Definitions
 * Preprocessor definitions for the bitfield "vl_accept_untagged_mode".
 * PORT="pif_rpf_vl_accept_untagged_i"
 */

/* Register address for bitfield vl_accept_untagged_mode */
#define HW_ATL2_RPF_VL_ACCEPT_UNTAGGED_MODE_ADR 0x00005280
/* Bitmask for bitfield vl_accept_untagged_mode */
#define HW_ATL2_RPF_VL_ACCEPT_UNTAGGED_MODE_MSK 0x00000004
/* Inverted bitmask for bitfield vl_accept_untagged_mode */
#define HW_ATL2_RPF_VL_ACCEPT_UNTAGGED_MODE_MSKN 0xFFFFFFFB
/* Lower bit position of bitfield vl_accept_untagged_mode */
#define HW_ATL2_RPF_VL_ACCEPT_UNTAGGED_MODE_SHIFT 2
/* Width of bitfield vl_accept_untagged_mode */
#define HW_ATL2_RPF_VL_ACCEPT_UNTAGGED_MODE_WIDTH 1
/* Default value of bitfield vl_accept_untagged_mode */
#define HW_ATL2_RPF_VL_ACCEPT_UNTAGGED_MODE_DEFAULT 0x0

/* rX vl_untagged_act[2:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "vl_untagged_act[2:0]".
 * PORT="pif_rpf_vl_untagged_act_i[2:0]"
 */

/* Register address for bitfield vl_untagged_act[2:0] */
#define HW_ATL2_RPF_VL_UNTAGGED_ACT_ADR 0x00005280
/* Bitmask for bitfield vl_untagged_act[2:0] */
#define HW_ATL2_RPF_VL_UNTAGGED_ACT_MSK 0x00000038
/* Inverted bitmask for bitfield vl_untagged_act[2:0] */
#define HW_ATL2_RPF_VL_UNTAGGED_ACT_MSKN 0xFFFFFFC7
/* Lower bit position of bitfield vl_untagged_act[2:0] */
#define HW_ATL2_RPF_VL_UNTAGGED_ACT_SHIFT 3
/* Width of bitfield vl_untagged_act[2:0] */
#define HW_ATL2_RPF_VL_UNTAGGED_ACT_WIDTH 3
/* Default value of bitfield vl_untagged_act[2:0] */
#define HW_ATL2_RPF_VL_UNTAGGED_ACT_DEFAULT 0x0

/* RX vl_en{F} Bitfield Definitions
 * Preprocessor definitions for the bitfield "vl_en{F}".
 * Parameter: filter {F} | stride size 0x4 | range [0, 15]
 * PORT="pif_rpf_vl_en_i[0]"
 */

/* Register address for bitfield vl_en{F} */
#define HW_ATL2_RPF_VL_EN_F_ADR(filter) (0x00005290 + (filter) * 0x4)
/* Bitmask for bitfield vl_en{F} */
#define HW_ATL2_RPF_VL_EN_F_MSK 0x80000000
/* Inverted bitmask for bitfield vl_en{F} */
#define HW_ATL2_RPF_VL_EN_F_MSKN 0x7FFFFFFF
/* Lower bit position of bitfield vl_en{F} */
#define HW_ATL2_RPF_VL_EN_F_SHIFT 31
/* Width of bitfield vl_en{F} */
#define HW_ATL2_RPF_VL_EN_F_WIDTH 1
/* Default value of bitfield vl_en{F} */
#define HW_ATL2_RPF_VL_EN_F_DEFAULT 0x0

/* RX vl_act{F}[2:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "vl_act{F}[2:0]".
 * Parameter: filter {F} | stride size 0x4 | range [0, 15]
 * PORT="pif_rpf_vl_act0_i[2:0]"
 */

/* Register address for bitfield vl_act{F}[2:0] */
#define HW_ATL2_RPF_VL_ACT_F_ADR(filter) (0x00005290 + (filter) * 0x4)
/* Bitmask for bitfield vl_act{F}[2:0] */
#define HW_ATL2_RPF_VL_ACT_F_MSK 0x00070000
/* Inverted bitmask for bitfield vl_act{F}[2:0] */
#define HW_ATL2_RPF_VL_ACT_F_MSKN 0xFFF8FFFF
/* Lower bit position of bitfield vl_act{F}[2:0] */
#define HW_ATL2_RPF_VL_ACT_F_SHIFT 16
/* Width of bitfield vl_act{F}[2:0] */
#define HW_ATL2_RPF_VL_ACT_F_WIDTH 3
/* Default value of bitfield vl_act{F}[2:0] */
#define HW_ATL2_RPF_VL_ACT_F_DEFAULT 0x0

/* rx vlan_req_tag0{f}[3:0] bitfield definitions
 * preprocessor definitions for the bitfield "vlan_req_tag0{f}[3:0]".
 * parameter: filter {f} | stride size 0x4 | range [0, 15]
 * port="pif_rpf_vlan_req_tag0[3:0]"
 */

/* register address for bitfield vlan_req_tag0{f}[3:0] */
#define HW_ATL2_RPF_VL_TAG_ADR(filter) (0x00005290 + (filter) * 0x4)
/* bitmask for bitfield vlan_req_tag0{f}[3:0] */
#define HW_ATL2_RPF_VL_TAG_MSK 0x0000F000
/* inverted bitmask for bitfield vlan_req_tag0{f}[3:0] */
#define HW_ATL2_RPF_VL_TAG_MSKN 0xFFFF0FFF
/* lower bit position of bitfield vlan_req_tag0{f}[3:0] */
#define HW_ATL2_RPF_VL_TAG_SHIFT 12
/* width of bitfield vlan_req_tag0{f}[3:0] */
#define HW_ATL2_RPF_VL_TAG_WIDTH 4
/* default value of bitfield vlan_req_tag0{f}[3:0] */
#define HW_ATL2_RPF_VL_TAG_DEFAULT 0x0

/* RX vl_id{F}[B:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "vl_id{F}[B:0]".
 * Parameter: filter {F} | stride size 0x4 | range [0, 15]
 * PORT="pif_rpf_vl_id0_i[11:0]"
 */

/* Register address for bitfield vl_id{F}[B:0] */
#define HW_ATL2_RPF_VL_ID_F_ADR(filter) (0x00005290 + (filter) * 0x4)
/* Bitmask for bitfield vl_id{F}[B:0] */
#define HW_ATL2_RPF_VL_ID_F_MSK 0x00000FFF
/* Inverted bitmask for bitfield vl_id{F}[B:0] */
#define HW_ATL2_RPF_VL_ID_F_MSKN 0xFFFFF000
/* Lower bit position of bitfield vl_id{F}[B:0] */
#define HW_ATL2_RPF_VL_ID_F_SHIFT 0
/* Width of bitfield vl_id{F}[B:0] */
#define HW_ATL2_RPF_VL_ID_F_WIDTH 12
/* Default value of bitfield vl_id{F}[B:0] */
#define HW_ATL2_RPF_VL_ID_F_DEFAULT 0x0

/* RX vl_rxq_en{F} Bitfield Definitions
 * Preprocessor definitions for the bitfield "vl_rxq{F}".
 * Parameter: filter {F} | stride size 0x4 | range [0, 15]
 * PORT="pif_rpf_vl_rxq_en_i"
 */

/* Register address for bitfield vl_rxq_en{F} */
#define HW_ATL2_RPF_VL_RXQ_EN_F_ADR(filter) (0x00005290 + (filter) * 0x4)
/* Bitmask for bitfield vl_rxq_en{F} */
#define HW_ATL2_RPF_VL_RXQ_EN_F_MSK 0x10000000
/* Inverted bitmask for bitfield vl_rxq_en{F}[ */
#define HW_ATL2_RPF_VL_RXQ_EN_F_MSKN 0xEFFFFFFF
/* Lower bit position of bitfield vl_rxq_en{F} */
#define HW_ATL2_RPF_VL_RXQ_EN_F_SHIFT 28
/* Width of bitfield vl_rxq_en{F} */
#define HW_ATL2_RPF_VL_RXQ_EN_F_WIDTH 1
/* Default value of bitfield vl_rxq_en{F} */
#define HW_ATL2_RPF_VL_RXQ_EN_F_DEFAULT 0x0

/* RX vl_rxq{F}[4:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "vl_rxq{F}[4:0]".
 * Parameter: filter {F} | stride size 0x4 | range [0, 15]
 * PORT="pif_rpf_vl_rxq0_i[4:0]"
 */

/* Register address for bitfield vl_rxq{F}[4:0] */
#define HW_ATL2_RPF_VL_RXQ_F_ADR(filter) (0x00005290 + (filter) * 0x4)
/* Bitmask for bitfield vl_rxq{F}[4:0] */
#define HW_ATL2_RPF_VL_RXQ_F_MSK 0x01F00000
/* Inverted bitmask for bitfield vl_rxq{F}[4:0] */
#define HW_ATL2_RPF_VL_RXQ_F_MSKN 0xFE0FFFFF
/* Lower bit position of bitfield vl_rxq{F}[4:0] */
#define HW_ATL2_RPF_VL_RXQ_F_SHIFT 20
/* Width of bitfield vl_rxw{F}[4:0] */
#define HW_ATL2_RPF_VL_RXQ_F_WIDTH 5
/* Default value of bitfield vl_rxq{F}[4:0] */
#define HW_ATL2_RPF_VL_RXQ_F_DEFAULT 0x0

/* rx et_en{f} bitfield definitions
 * preprocessor definitions for the bitfield "et_en{f}".
 * parameter: filter {f} | stride size 0x4 | range [0, 15]
 * port="pif_rpf_et_en_i[0]"
 */

/* register address for bitfield et_en{f} */
#define HW_ATL2_RPF_ET_ENF_ADR(filter) (0x00005300 + (filter) * 0x4)
/* bitmask for bitfield et_en{f} */
#define HW_ATL2_RPF_ET_ENF_MSK 0x80000000
/* inverted bitmask for bitfield et_en{f} */
#define HW_ATL2_RPF_ET_ENF_MSKN 0x7fffffff
/* lower bit position of bitfield et_en{f} */
#define HW_ATL2_RPF_ET_ENF_SHIFT 31
/* width of bitfield et_en{f} */
#define HW_ATL2_RPF_ET_ENF_WIDTH 1
/* default value of bitfield et_en{f} */
#define HW_ATL2_RPF_ET_ENF_DEFAULT 0x0

/* rx et_up{f}_en bitfield definitions
 * preprocessor definitions for the bitfield "et_up{f}_en".
 * parameter: filter {f} | stride size 0x4 | range [0, 15]
 * port="pif_rpf_et_up_en_i[0]"
 */

/* register address for bitfield et_up{f}_en */
#define HW_ATL2_RPF_ET_UPFEN_ADR(filter) (0x00005300 + (filter) * 0x4)
/* bitmask for bitfield et_up{f}_en */
#define HW_ATL2_RPF_ET_UPFEN_MSK 0x40000000
/* inverted bitmask for bitfield et_up{f}_en */
#define HW_ATL2_RPF_ET_UPFEN_MSKN 0xbfffffff
/* lower bit position of bitfield et_up{f}_en */
#define HW_ATL2_RPF_ET_UPFEN_SHIFT 30
/* width of bitfield et_up{f}_en */
#define HW_ATL2_RPF_ET_UPFEN_WIDTH 1
/* default value of bitfield et_up{f}_en */
#define HW_ATL2_RPF_ET_UPFEN_DEFAULT 0x0

/* rx et_rxq{f}_en bitfield definitions
 * preprocessor definitions for the bitfield "et_rxq{f}_en".
 * parameter: filter {f} | stride size 0x4 | range [0, 15]
 * port="pif_rpf_et_rxq_en_i[0]"
 */

/* register address for bitfield et_rxq{f}_en */
#define HW_ATL2_RPF_ET_RXQFEN_ADR(filter) (0x00005300 + (filter) * 0x4)
/* bitmask for bitfield et_rxq{f}_en */
#define HW_ATL2_RPF_ET_RXQFEN_MSK 0x20000000
/* inverted bitmask for bitfield et_rxq{f}_en */
#define HW_ATL2_RPF_ET_RXQFEN_MSKN 0xdfffffff
/* lower bit position of bitfield et_rxq{f}_en */
#define HW_ATL2_RPF_ET_RXQFEN_SHIFT 29
/* width of bitfield et_rxq{f}_en */
#define HW_ATL2_RPF_ET_RXQFEN_WIDTH 1
/* default value of bitfield et_rxq{f}_en */
#define HW_ATL2_RPF_ET_RXQFEN_DEFAULT 0x0

/* rx et_up{f}[2:0] bitfield definitions
 * preprocessor definitions for the bitfield "et_up{f}[2:0]".
 * parameter: filter {f} | stride size 0x4 | range [0, 15]
 * port="pif_rpf_et_up0_i[2:0]"
 */

/* register address for bitfield et_up{f}[2:0] */
#define HW_ATL2_RPF_ET_UPF_ADR(filter) (0x00005300 + (filter) * 0x4)
/* bitmask for bitfield et_up{f}[2:0] */
#define HW_ATL2_RPF_ET_UPF_MSK 0x1c000000
/* inverted bitmask for bitfield et_up{f}[2:0] */
#define HW_ATL2_RPF_ET_UPF_MSKN 0xe3ffffff
/* lower bit position of bitfield et_up{f}[2:0] */
#define HW_ATL2_RPF_ET_UPF_SHIFT 26
/* width of bitfield et_up{f}[2:0] */
#define HW_ATL2_RPF_ET_UPF_WIDTH 3
/* default value of bitfield et_up{f}[2:0] */
#define HW_ATL2_RPF_ET_UPF_DEFAULT 0x0

/* rx et_rxq{f}[4:0] bitfield definitions
 * preprocessor definitions for the bitfield "et_rxq{f}[4:0]".
 * parameter: filter {f} | stride size 0x4 | range [0, 15]
 * port="pif_rpf_et_rxq0_i[4:0]"
 */

/* register address for bitfield et_rxq{f}[4:0] */
#define HW_ATL2_RPF_ET_RXQF_ADR(filter) (0x00005300 + (filter) * 0x4)
/* bitmask for bitfield et_rxq{f}[4:0] */
#define HW_ATL2_RPF_ET_RXQF_MSK 0x01f00000
/* inverted bitmask for bitfield et_rxq{f}[4:0] */
#define HW_ATL2_RPF_ET_RXQF_MSKN 0xfe0fffff
/* lower bit position of bitfield et_rxq{f}[4:0] */
#define HW_ATL2_RPF_ET_RXQF_SHIFT 20
/* width of bitfield et_rxq{f}[4:0] */
#define HW_ATL2_RPF_ET_RXQF_WIDTH 5
/* default value of bitfield et_rxq{f}[4:0] */
#define HW_ATL2_RPF_ET_RXQF_DEFAULT 0x0

/* rx et_mng_rxq{f} bitfield definitions
 * preprocessor definitions for the bitfield "et_mng_rxq{f}".
 * parameter: filter {f} | stride size 0x4 | range [0, 15]
 * port="pif_rpf_et_mng_rxq_i[0]"
 */

/* register address for bitfield et_mng_rxq{f} */
#define HW_ATL2_RPF_ET_MNG_RXQF_ADR(filter) (0x00005300 + (filter) * 0x4)
/* bitmask for bitfield et_mng_rxq{f} */
#define HW_ATL2_RPF_ET_MNG_RXQF_MSK 0x00080000
/* inverted bitmask for bitfield et_mng_rxq{f} */
#define HW_ATL2_RPF_ET_MNG_RXQF_MSKN 0xfff7ffff
/* lower bit position of bitfield et_mng_rxq{f} */
#define HW_ATL2_RPF_ET_MNG_RXQF_SHIFT 19
/* width of bitfield et_mng_rxq{f} */
#define HW_ATL2_RPF_ET_MNG_RXQF_WIDTH 1
/* default value of bitfield et_mng_rxq{f} */
#define HW_ATL2_RPF_ET_MNG_RXQF_DEFAULT 0x0

/* rx et_act{f}[2:0] bitfield definitions
 * preprocessor definitions for the bitfield "et_act{f}[2:0]".
 * parameter: filter {f} | stride size 0x4 | range [0, 15]
 * port="pif_rpf_et_act0_i[2:0]"
 */

/* register address for bitfield et_act{f}[2:0] */
#define HW_ATL2_RPF_ET_ACTF_ADR(filter) (0x00005300 + (filter) * 0x4)
/* bitmask for bitfield et_act{f}[2:0] */
#define HW_ATL2_RPF_ET_ACTF_MSK 0x00070000
/* inverted bitmask for bitfield et_act{f}[2:0] */
#define HW_ATL2_RPF_ET_ACTF_MSKN 0xfff8ffff
/* lower bit position of bitfield et_act{f}[2:0] */
#define HW_ATL2_RPF_ET_ACTF_SHIFT 16
/* width of bitfield et_act{f}[2:0] */
#define HW_ATL2_RPF_ET_ACTF_WIDTH 3
/* default value of bitfield et_act{f}[2:0] */
#define HW_ATL2_RPF_ET_ACTF_DEFAULT 0x0

/* rx et_val{f}[f:0] bitfield definitions
 * preprocessor definitions for the bitfield "et_val{f}[f:0]".
 * parameter: filter {f} | stride size 0x4 | range [0, 15]
 * port="pif_rpf_et_val0_i[15:0]"
 */

/* register address for bitfield et_val{f}[f:0] */
#define HW_ATL2_RPF_ET_VALF_ADR(filter) (0x00005300 + (filter) * 0x4)
/* bitmask for bitfield et_val{f}[f:0] */
#define HW_ATL2_RPF_ET_VALF_MSK 0x0000ffff
/* inverted bitmask for bitfield et_val{f}[f:0] */
#define HW_ATL2_RPF_ET_VALF_MSKN 0xffff0000
/* lower bit position of bitfield et_val{f}[f:0] */
#define HW_ATL2_RPF_ET_VALF_SHIFT 0
/* width of bitfield et_val{f}[f:0] */
#define HW_ATL2_RPF_ET_VALF_WIDTH 16
/* default value of bitfield et_val{f}[f:0] */
#define HW_ATL2_RPF_ET_VALF_DEFAULT 0x0

/* rx etype_req_tag0{f}[2:0] bitfield definitions
 * preprocessor definitions for the bitfield "etype_req_tag0{f}[2:0]".
 * parameter: filter {f} | stride size 0x4 | range [0, 15]
 * port="pif_rpf_etype_req_tag0[2:0]"
 */

/* register address for bitfield etype_req_tag0{f}[2:0] */
#define HW_ATL2_RPF_ET_TAG_ADR(filter) (0x00005340 + (filter) * 0x4)
/* bitmask for bitfield etype_req_tag0{f}[2:0] */
#define HW_ATL2_RPF_ET_TAG_MSK 0x00000007
/* inverted bitmask for bitfield etype_req_tag0{f}[2:0] */
#define HW_ATL2_RPF_ET_TAG_MSKN 0xFFFFFFF8
/* lower bit position of bitfield etype_req_tag0{f}[2:0] */
#define HW_ATL2_RPF_ET_TAG_SHIFT 0
/* width of bitfield etype_req_tag0{f}[2:0] */
#define HW_ATL2_RPF_ET_TAG_WIDTH 3
/* default value of bitfield etype_req_tag0{f}[2:0] */
#define HW_ATL2_RPF_ET_TAG_DEFAULT 0x0

/* RX l3_l4_en{F} Bitfield Definitions
 * Preprocessor definitions for the bitfield "l3_l4_en{F}".
 * Parameter: filter {F} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l3_l4_en_i[0]"
 */

/* Register address for bitfield l3_l4_en{F} */
#define HW_ATL2_RPF_L3_L4_ENF_ADR(filter) (0x00005380u + (filter) * 0x4)
/* Bitmask for bitfield l3_l4_en{F} */
#define HW_ATL2_RPF_L3_L4_ENF_MSK 0x80000000u
/* Inverted bitmask for bitfield l3_l4_en{F} */
#define HW_ATL2_RPF_L3_L4_ENF_MSKN 0x7FFFFFFFu
/* Lower bit position of bitfield l3_l4_en{F} */
#define HW_ATL2_RPF_L3_L4_ENF_SHIFT 31
/* Width of bitfield l3_l4_en{F} */
#define HW_ATL2_RPF_L3_L4_ENF_WIDTH 1
/* Default value of bitfield l3_l4_en{F} */
#define HW_ATL2_RPF_L3_L4_ENF_DEFAULT 0x0

/* RX l3_v6_en{F} Bitfield Definitions
 * Preprocessor definitions for the bitfield "l3_v6_en{F}".
 * Parameter: filter {F} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l3_v6_en_i[0]"
 */
/* Register address for bitfield l3_v6_en{F} */
#define HW_ATL2_RPF_L3_V6_ENF_ADR(filter) (0x00005380u + (filter) * 0x4)
/* Bitmask for bitfield l3_v6_en{F} */
#define HW_ATL2_RPF_L3_V6_ENF_MSK 0x40000000u
/* Inverted bitmask for bitfield l3_v6_en{F} */
#define HW_ATL2_RPF_L3_V6_ENF_MSKN 0xBFFFFFFFu
/* Lower bit position of bitfield l3_v6_en{F} */
#define HW_ATL2_RPF_L3_V6_ENF_SHIFT 30
/* Width of bitfield l3_v6_en{F} */
#define HW_ATL2_RPF_L3_V6_ENF_WIDTH 1
/* Default value of bitfield l3_v6_en{F} */
#define HW_ATL2_RPF_L3_V6_ENF_DEFAULT 0x0

/* RX l3_sa{F}_en Bitfield Definitions
 * Preprocessor definitions for the bitfield "l3_sa{F}_en".
 * Parameter: filter {F} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l3_sa_en_i[0]"
 */

/* Register address for bitfield l3_sa{F}_en */
#define HW_ATL2_RPF_L3_SAF_EN_ADR(filter) (0x00005380u + (filter) * 0x4)
/* Bitmask for bitfield l3_sa{F}_en */
#define HW_ATL2_RPF_L3_SAF_EN_MSK 0x20000000u
/* Inverted bitmask for bitfield l3_sa{F}_en */
#define HW_ATL2_RPF_L3_SAF_EN_MSKN 0xDFFFFFFFu
/* Lower bit position of bitfield l3_sa{F}_en */
#define HW_ATL2_RPF_L3_SAF_EN_SHIFT 29
/* Width of bitfield l3_sa{F}_en */
#define HW_ATL2_RPF_L3_SAF_EN_WIDTH 1
/* Default value of bitfield l3_sa{F}_en */
#define HW_ATL2_RPF_L3_SAF_EN_DEFAULT 0x0

/* RX l3_da{F}_en Bitfield Definitions
 * Preprocessor definitions for the bitfield "l3_da{F}_en".
 * Parameter: filter {F} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l3_da_en_i[0]"
 */

/* Register address for bitfield l3_da{F}_en */
#define HW_ATL2_RPF_L3_DAF_EN_ADR(filter) (0x00005380u + (filter) * 0x4)
/* Bitmask for bitfield l3_da{F}_en */
#define HW_ATL2_RPF_L3_DAF_EN_MSK 0x10000000u
/* Inverted bitmask for bitfield l3_da{F}_en */
#define HW_ATL2_RPF_L3_DAF_EN_MSKN 0xEFFFFFFFu
/* Lower bit position of bitfield l3_da{F}_en */
#define HW_ATL2_RPF_L3_DAF_EN_SHIFT 28
/* Width of bitfield l3_da{F}_en */
#define HW_ATL2_RPF_L3_DAF_EN_WIDTH 1
/* Default value of bitfield l3_da{F}_en */
#define HW_ATL2_RPF_L3_DAF_EN_DEFAULT 0x0

/* RX l4_sp{F}_en Bitfield Definitions
 * Preprocessor definitions for the bitfield "l4_sp{F}_en".
 * Parameter: filter {F} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l4_sp_en_i[0]"
 */

/* Register address for bitfield l4_sp{F}_en */
#define HW_ATL2_RPF_L4_SPF_EN_ADR(filter) (0x00005380u + (filter) * 0x4)
/* Bitmask for bitfield l4_sp{F}_en */
#define HW_ATL2_RPF_L4_SPF_EN_MSK 0x08000000u
/* Inverted bitmask for bitfield l4_sp{F}_en */
#define HW_ATL2_RPF_L4_SPF_EN_MSKN 0xF7FFFFFFu
/* Lower bit position of bitfield l4_sp{F}_en */
#define HW_ATL2_RPF_L4_SPF_EN_SHIFT 27
/* Width of bitfield l4_sp{F}_en */
#define HW_ATL2_RPF_L4_SPF_EN_WIDTH 1
/* Default value of bitfield l4_sp{F}_en */
#define HW_ATL2_RPF_L4_SPF_EN_DEFAULT 0x0

/* RX l4_dp{F}_en Bitfield Definitions
 * Preprocessor definitions for the bitfield "l4_dp{F}_en".
 * Parameter: filter {F} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l4_dp_en_i[0]"
 */

/* Register address for bitfield l4_dp{F}_en */
#define HW_ATL2_RPF_L4_DPF_EN_ADR(filter) (0x00005380u + (filter) * 0x4)
/* Bitmask for bitfield l4_dp{F}_en */
#define HW_ATL2_RPF_L4_DPF_EN_MSK 0x04000000u
/* Inverted bitmask for bitfield l4_dp{F}_en */
#define HW_ATL2_RPF_L4_DPF_EN_MSKN 0xFBFFFFFFu
/* Lower bit position of bitfield l4_dp{F}_en */
#define HW_ATL2_RPF_L4_DPF_EN_SHIFT 26
/* Width of bitfield l4_dp{F}_en */
#define HW_ATL2_RPF_L4_DPF_EN_WIDTH 1
/* Default value of bitfield l4_dp{F}_en */
#define HW_ATL2_RPF_L4_DPF_EN_DEFAULT 0x0

/* RX l4_prot{F}_en Bitfield Definitions
 * Preprocessor definitions for the bitfield "l4_prot{F}_en".
 * Parameter: filter {F} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l4_prot_en_i[0]"
 */

/* Register address for bitfield l4_prot{F}_en */
#define HW_ATL2_RPF_L4_PROTF_EN_ADR(filter) (0x00005380u + (filter) * 0x4)
/* Bitmask for bitfield l4_prot{F}_en */
#define HW_ATL2_RPF_L4_PROTF_EN_MSK 0x02000000u
/* Inverted bitmask for bitfield l4_prot{F}_en */
#define HW_ATL2_RPF_L4_PROTF_EN_MSKN 0xFDFFFFFFu
/* Lower bit position of bitfield l4_prot{F}_en */
#define HW_ATL2_RPF_L4_PROTF_EN_SHIFT 25
/* Width of bitfield l4_prot{F}_en */
#define HW_ATL2_RPF_L4_PROTF_EN_WIDTH 1
/* Default value of bitfield l4_prot{F}_en */
#define HW_ATL2_RPF_L4_PROTF_EN_DEFAULT 0x0

/* RX l3_arp{F}_en Bitfield Definitions
 * Preprocessor definitions for the bitfield "l3_arp{F}_en".
 * Parameter: filter {F} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l3_arp_en_i[0]"
 */

/* Register address for bitfield l3_arp{F}_en */
#define HW_ATL2_RPF_L3_ARPF_EN_ADR(filter) (0x00005380u + (filter) * 0x4)
/* Bitmask for bitfield l3_arp{F}_en */
#define HW_ATL2_RPF_L3_ARPF_EN_MSK 0x01000000u
/* Inverted bitmask for bitfield l3_arp{F}_en */
#define HW_ATL2_RPF_L3_ARPF_EN_MSKN 0xFEFFFFFFu
/* Lower bit position of bitfield l3_arp{F}_en */
#define HW_ATL2_RPF_L3_ARPF_EN_SHIFT 24
/* Width of bitfield l3_arp{F}_en */
#define HW_ATL2_RPF_L3_ARPF_EN_WIDTH 1
/* Default value of bitfield l3_arp{F}_en */
#define HW_ATL2_RPF_L3_ARPF_EN_DEFAULT 0x0

/* RX l3_l4_rxq{F}_en Bitfield Definitions
 * Preprocessor definitions for the bitfield "l3_l4_rxq{F}_en".
 * Parameter: filter {F} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l3_l4_rxq_en_i[0]"
 */

/* Register address for bitfield l3_l4_RXq{F}_en */
#define HW_ATL2_RPF_L3_L4_RXQF_EN_ADR(filter) (0x00005380u + (filter) * 0x4)
/* Bitmask for bitfield l3_l4_RXq{F}_en */
#define HW_ATL2_RPF_L3_L4_RXQF_EN_MSK 0x00800000u
/* Inverted bitmask for bitfield l3_l4_RXq{F}_en */
#define HW_ATL2_RPF_L3_L4_RXQF_EN_MSKN 0xFF7FFFFFu
/* Lower bit position of bitfield l3_l4_RXq{F}_en */
#define HW_ATL2_RPF_L3_L4_RXQF_EN_SHIFT 23
/* Width of bitfield l3_l4_RXq{F}_en */
#define HW_ATL2_RPF_L3_L4_RXQF_EN_WIDTH 1
/* Default value of bitfield l3_l4_RXq{F}_en */
#define HW_ATL2_RPF_L3_L4_RXQF_EN_DEFAULT 0x0

/* RX l3_l4_mng_RXq{F} Bitfield Definitions
 * Preprocessor definitions for the bitfield "l3_l4_mng_RXq{F}".
 * Parameter: filter {F} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l3_l4_mng_rxq_i[0]"
 */

/* Register address for bitfield l3_l4_mng_rxq{F} */
#define HW_ATL2_RPF_L3_L4_MNG_RXQF_ADR(filter) (0x00005380u + (filter) * 0x4)
/* Bitmask for bitfield l3_l4_mng_rxq{F} */
#define HW_ATL2_RPF_L3_L4_MNG_RXQF_MSK 0x00400000u
/* Inverted bitmask for bitfield l3_l4_mng_rxq{F} */
#define HW_ATL2_RPF_L3_L4_MNG_RXQF_MSKN 0xFFBFFFFFu
/* Lower bit position of bitfield l3_l4_mng_rxq{F} */
#define HW_ATL2_RPF_L3_L4_MNG_RXQF_SHIFT 22
/* Width of bitfield l3_l4_mng_rxq{F} */
#define HW_ATL2_RPF_L3_L4_MNG_RXQF_WIDTH 1
/* Default value of bitfield l3_l4_mng_rxq{F} */
#define HW_ATL2_RPF_L3_L4_MNG_RXQF_DEFAULT 0x0

/* RX l3_l4_act{F}[2:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "l3_l4_act{F}[2:0]".
 * Parameter: filter {F} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l3_l4_act0_i[2:0]"
 */

/* Register address for bitfield l3_l4_act{F}[2:0] */
#define HW_ATL2_RPF_L3_L4_ACTF_ADR(filter) (0x00005380u + (filter) * 0x4)
/* Bitmask for bitfield l3_l4_act{F}[2:0] */
#define HW_ATL2_RPF_L3_L4_ACTF_MSK 0x00070000u
/* Inverted bitmask for bitfield l3_l4_act{F}[2:0] */
#define HW_ATL2_RPF_L3_L4_ACTF_MSKN 0xFFF8FFFFu
/* Lower bit position of bitfield l3_l4_act{F}[2:0] */
#define HW_ATL2_RPF_L3_L4_ACTF_SHIFT 16
/* Width of bitfield l3_l4_act{F}[2:0] */
#define HW_ATL2_RPF_L3_L4_ACTF_WIDTH 3
/* Default value of bitfield l3_l4_act{F}[2:0] */
#define HW_ATL2_RPF_L3_L4_ACTF_DEFAULT 0x0

/* RX l3_l4_rxq{F}[4:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "l3_l4_rxq{F}[4:0]".
 * Parameter: filter {F} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l3_l4_rxq0_i[4:0]"
 */

/* Register address for bitfield l3_l4_rxq{F}[4:0] */
#define HW_ATL2_RPF_L3_L4_RXQF_ADR(filter) (0x00005380u + (filter) * 0x4)
/* Bitmask for bitfield l3_l4_rxq{F}[4:0] */
#define HW_ATL2_RPF_L3_L4_RXQF_MSK 0x00001F00u
/* Inverted bitmask for bitfield l3_l4_rxq{F}[4:0] */
#define HW_ATL2_RPF_L3_L4_RXQF_MSKN 0xFFFFE0FFu
/* Lower bit position of bitfield l3_l4_rxq{F}[4:0] */
#define HW_ATL2_RPF_L3_L4_RXQF_SHIFT 8
/* Width of bitfield l3_l4_rxq{F}[4:0] */
#define HW_ATL2_RPF_L3_L4_RXQF_WIDTH 5
/* Default value of bitfield l3_l4_rxq{F}[4:0] */
#define HW_ATL2_RPF_L3_L4_RXQF_DEFAULT 0x0

/*! @name RX pif_rpf_rss1_red4 Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pif_rpf_rss1_red4".
*
*   Parameter: repeat {R} | stride size 0x4 | range [0, 63]
*
*   Type: R/W
*
*   Notes: RSS redirection table RX queue for TC 3
*
*   PORT="pif_rpf_rss_red1_data_i[19:15]"
@{ */
/*! \brief Register address for bitfield pif_rpf_rss1_red4 */
#define HW_ATL2_RPF_RSS1RED4_ADR(repeat) (0x00006200 + (repeat) * 0x4)
/*! \brief Bitmask for bitfield pif_rpf_rss1_red4 */
#define HW_ATL2_RPF_RSS1RED4_MSK 0x001F8000
/*! \brief Inverted bitmask for bitfield pif_rpf_rss1_red4 */
#define HW_ATL2_RPF_RSS1RED4_MSKN 0xFFE07FFF
/*! \brief Lower bit position of bitfield pif_rpf_rss1_red4 */
#define HW_ATL2_RPF_RSS1RED4_SHIFT 15
/*! \brief Width of bitfield pif_rpf_rss1_red4 */
#define HW_ATL2_RPF_RSS1RED4_WIDTH 6
/*! \brief Default value of bitfield pif_rpf_rss1_red4 */
#define HW_ATL2_RPF_RSS1RED4_DEFAULT 0x0
/*@}*/

/*! @name RX pif_rpf_rss1_red3 Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pif_rpf_rss1_red3".
*
*   Parameter: repeat {R} | stride size 0x4 | range [0, 63]
*
*   Type: R/W
*
*   Notes: RSS redirection table RX queue for TC 2
*
*   PORT="pif_rpf_rss_red1_data_i[14:10]"
@{ */
/*! \brief Register address for bitfield pif_rpf_rss1_red3 */
#define HW_ATL2_RPF_RSS1RED3_ADR(repeat) (0x00006200 + (repeat) * 0x4)
/*! \brief Bitmask for bitfield pif_rpf_rss1_red3 */
#define HW_ATL2_RPF_RSS1RED3_MSK 0x00007C00
/*! \brief Inverted bitmask for bitfield pif_rpf_rss1_red3 */
#define HW_ATL2_RPF_RSS1RED3_MSKN 0xFFFF83FF
/*! \brief Lower bit position of bitfield pif_rpf_rss1_red3 */
#define HW_ATL2_RPF_RSS1RED3_SHIFT 10
/*! \brief Width of bitfield pif_rpf_rss1_red3 */
#define HW_ATL2_RPF_RSS1RED3_WIDTH 5
/*! \brief Default value of bitfield pif_rpf_rss1_red3 */
#define HW_ATL2_RPF_RSS1RED3_DEFAULT 0x0
/*@}*/

/*! @name RX pif_rpf_rss1_red2 Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pif_rpf_rss1_red2".
*
*   Parameter: repeat {R} | stride size 0x4 | range [0, 63]
*
*   Type: R/W
*
*   Notes: RSS redirection table RX queue for TC 1
*
*   PORT="pif_rpf_rss_red1_data_i[9:5]"
@{ */
/*! \brief Register address for bitfield pif_rpf_rss1_red2 */
#define HW_ATL2_RPF_RSS1RED2_ADR(repeat) (0x00006200 + (repeat) * 0x4)
/*! \brief Bitmask for bitfield pif_rpf_rss1_red2 */
#define HW_ATL2_RPF_RSS1RED2_MSK 0x000003E0
/*! \brief Inverted bitmask for bitfield pif_rpf_rss1_red2 */
#define HW_ATL2_RPF_RSS1RED2_MSKN 0xFFFFFC1F
/*! \brief Lower bit position of bitfield pif_rpf_rss1_red2 */
#define HW_ATL2_RPF_RSS1RED2_SHIFT 5
/*! \brief Width of bitfield pif_rpf_rss1_red2 */
#define HW_ATL2_RPF_RSS1RED2_WIDTH 5
/*! \brief Default value of bitfield pif_rpf_rss1_red2 */
#define HW_ATL2_RPF_RSS1RED2_DEFAULT 0x0
/*@}*/

/*! @name RX pif_rpf_rss1_red1 Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pif_rpf_rss1_red1".
*
*   Parameter: repeat {R} | stride size 0x4 | range [0, 63]
*
*   Type: R/W
*
*   Notes: RSS redirection table RX queue for TC 0
*
*   PORT="pif_rpf_rss_red1_data_i[4:0]"
@{ */
/*! \brief Register address for bitfield pif_rpf_rss1_red1 */
#define HW_ATL2_RPF_RSS1RED1_ADR(repeat) (0x00006200 + (repeat) * 0x4)
/*! \brief Bitmask for bitfield pif_rpf_rss1_red1 */
#define HW_ATL2_RPF_RSS1RED1_MSK 0x0000001F
/*! \brief Inverted bitmask for bitfield pif_rpf_rss1_red1 */
#define HW_ATL2_RPF_RSS1RED1_MSKN 0xFFFFFFE0
/*! \brief Lower bit position of bitfield pif_rpf_rss1_red1 */
#define HW_ATL2_RPF_RSS1RED1_SHIFT 0
/*! \brief Width of bitfield pif_rpf_rss1_red1 */
#define HW_ATL2_RPF_RSS1RED1_WIDTH 5
/*! \brief Default value of bitfield pif_rpf_rss1_red1 */
#define HW_ATL2_RPF_RSS1RED1_DEFAULT 0x0
/*@}*/


/*! @name RX pif_rpf_rss2_red4 Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pif_rpf_rss2_red4".
*
*   Parameter: repeat {R} | stride size 0x4 | range [0, 63]
*
*   Type: R/W
*
*   Notes: RSS redirection table RX queue for TC 7
*
*   PORT="pif_rpf_rss_red1_data_i[19:15]"
@{ */
/*! \brief Register address for bitfield pif_rpf_rss2_red4 */
#define HW_ATL2_RPF_RSS2RED4_ADR(repeat) (0x00006300 + (repeat) * 0x4)
/*! \brief Bitmask for bitfield pif_rpf_rss2_red4 */
#define HW_ATL2_RPF_RSS2RED4_MSK 0x001F8000
/*! \brief Inverted bitmask for bitfield pif_rpf_rss2_red4 */
#define HW_ATL2_RPF_RSS2RED4_MSKN 0xFFE07FFF
/*! \brief Lower bit position of bitfield pif_rpf_rss2_red4 */
#define HW_ATL2_RPF_RSS2RED4_SHIFT 15
/*! \brief Width of bitfield pif_rpf_rss2_red4 */
#define HW_ATL2_RPF_RSS2RED4_WIDTH 6
/*! \brief Default value of bitfield pif_rpf_rss2_red4 */
#define HW_ATL2_RPF_RSS2RED4_DEFAULT 0x0
/*@}*/

/*! @name RX pif_rpf_rss2_red3 Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pif_rpf_rss2_red3".
*
*   Parameter: repeat {R} | stride size 0x4 | range [0, 63]
*
*   Type: R/W
*
*   Notes: RSS redirection table RX queue for TC 6
*
*   PORT="pif_rpf_rss_red1_data_i[14:10]"
@{ */
/*! \brief Register address for bitfield pif_rpf_rss2_red3 */
#define HW_ATL2_RPF_RSS2RED3_ADR(repeat) (0x00006300 + (repeat) * 0x4)
/*! \brief Bitmask for bitfield pif_rpf_rss2_red3 */
#define HW_ATL2_RPF_RSS2RED3_MSK 0x00007C00
/*! \brief Inverted bitmask for bitfield pif_rpf_rss2_red3 */
#define HW_ATL2_RPF_RSS2RED3_MSKN 0xFFFF83FF
/*! \brief Lower bit position of bitfield pif_rpf_rss2_red3 */
#define HW_ATL2_RPF_RSS2RED3_SHIFT 10
/*! \brief Width of bitfield pif_rpf_rss2_red3 */
#define HW_ATL2_RPF_RSS2RED3_WIDTH 5
/*! \brief Default value of bitfield pif_rpf_rss2_red3 */
#define HW_ATL2_RPF_RSS2RED3_DEFAULT 0x0
/*@}*/

/*! @name RX pif_rpf_rss2_red2 Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pif_rpf_rss2_red2".
*
*   Parameter: repeat {R} | stride size 0x4 | range [0, 63]
*
*   Type: R/W
*
*   Notes: RSS redirection table RX queue for TC 5
*
*   PORT="pif_rpf_rss_red1_data_i[9:5]"
@{ */
/*! \brief Register address for bitfield pif_rpf_rss2_red2 */
#define HW_ATL2_RPF_RSS2RED2_ADR(repeat) (0x00006300 + (repeat) * 0x4)
/*! \brief Bitmask for bitfield pif_rpf_rss2_red2 */
#define HW_ATL2_RPF_RSS2RED2_MSK 0x000003E0
/*! \brief Inverted bitmask for bitfield pif_rpf_rss2_red2 */
#define HW_ATL2_RPF_RSS2RED2_MSKN 0xFFFFFC1F
/*! \brief Lower bit position of bitfield pif_rpf_rss2_red2 */
#define HW_ATL2_RPF_RSS2RED2_SHIFT 5
/*! \brief Width of bitfield pif_rpf_rss2_red2 */
#define HW_ATL2_RPF_RSS2RED2_WIDTH 5
/*! \brief Default value of bitfield pif_rpf_rss2_red2 */
#define HW_ATL2_RPF_RSS2RED2_DEFAULT 0x0
/*@}*/

/*! @name RX pif_rpf_rss2_red1 Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pif_rpf_rss2_red1".
*
*   Parameter: repeat {R} | stride size 0x4 | range [0, 63]
*
*   Type: R/W
*
*   Notes: RSS redirection table RX queue for TC 4
*
*   PORT="pif_rpf_rss_red1_data_i[4:0]"
@{ */
/*! \brief Register address for bitfield pif_rpf_rss2_red1 */
#define HW_ATL2_RPF_RSS2RED1_ADR(repeat) (0x00006300 + (repeat) * 0x4)
/*! \brief Bitmask for bitfield pif_rpf_rss2_red1 */
#define HW_ATL2_RPF_RSS2RED1_MSK 0x0000001F
/*! \brief Inverted bitmask for bitfield pif_rpf_rss2_red1 */
#define HW_ATL2_RPF_RSS2RED1_MSKN 0xFFFFFFE0
/*! \brief Lower bit position of bitfield pif_rpf_rss2_red1 */
#define HW_ATL2_RPF_RSS2RED1_SHIFT 0
/*! \brief Width of bitfield pif_rpf_rss2_red1 */
#define HW_ATL2_RPF_RSS2RED1_WIDTH 5
/*! \brief Default value of bitfield pif_rpf_rss2_red1 */
#define HW_ATL2_RPF_RSS2RED1_DEFAULT 0x0
/*@}*/

/* RX rpf_l3_v6_sa{F}_dw{D}[1F:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "rpf_l3_v6_sa{F}_dw{D}[1F:0]".
 * Parameter: filter {F} | stride size 0x10 | range [0, 7]
 * Parameter: dword {D} | stride size 0x4 | range [0, 3]
 * PORT="pif_rpf_l3_v6_sa{F}_dw0[1F:0]"
 */

/* Register address for bitfield rpf_l3_v6_sa{F}_dw{D}[1F:0] */
#define HW_ATL2_RPF_L3_SA_DW_ADR(filter, dword) \
	(0x00006400u + (filter) * 0x10 + (dword) * 0x4)

/* RX rpf_l3_v6_da{F}_dw{D}[1F:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "rpf_l3_v6_da{F}_dw{D}[1F:0]".
 * Parameter: filter {F} | stride size 0x10 | range [0, 7]
 * Parameter: dword {D} | stride size 0x4 | range [0, 3]
 * PORT="pif_rpf_l3_v6_da{F}_dw{D}[1F:0]"
 */

/* Register address for bitfield rpf_l3_v6_da{F}_dw{D}[1F:0] */
#define HW_ATL2_RPF_L3_DA_DW_ADR(filter, dword) \
	(0x00006480u + (filter) * 0x10 + (dword) * 0x4)

/* RX rpf_l3_v4_cmd{F}[F:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "rpf_l3_v4_cmd{F}[F:0]".
 * Parameter: filter {F} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l3_v4_cmd{F}[F:0]"
 */

/* Register address for bitfield rpf_l3_v4_cmd{F}[F:0] */
#define HW_ATL2_RPF_L3_V4_CMD_ADR(filter) (0x00006500u + (filter) * 0x4)
/* Bitmask for bitfield rpf_l3_v4_cmd{F}[F:0] */
#define HW_ATL2_RPF_L3_V4_CMD_MSK 0x0000FFFFu
/* Inverted bitmask for bitfield rpf_l3_v4_cmd{F}[F:0] */
#define HW_ATL2_RPF_L3_V4_CMD_MSKN 0xFFFF0000u
/* Lower bit position of bitfield rpf_l3_v4_cmd{F}[F:0] */
#define HW_ATL2_RPF_L3_V4_CMD_SHIFT 0
/* Width of bitfield rpf_l3_v4_cmd{F}[F:0] */
#define HW_ATL2_RPF_L3_V4_CMD_WIDTH 16
/* Default value of bitfield rpf_l3_v4_cmd{F}[F:0] */
#define HW_ATL2_RPF_L3_V4_CMD_DEFAULT 0x0

/* RX rpf_l3_v6_cmd{F}[F:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "rpf_l3_v6_cmd{F}[F:0]".
 * Parameter: filter {F} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l3_v6_cmd{F}[F:0]"
 */

/* Register address for bitfield rpf_l3_v6_cmd{F}[F:0] */
#define HW_ATL2_RPF_L3_V6_CMD_ADR(filter) (0x00006500u + (filter) * 0x4)
/* Bitmask for bitfield rpf_l3_v6_cmd{F}[F:0] */
#define HW_ATL2_RPF_L3_V6_CMD_MSK 0xFF7F0000u
/* Inverted bitmask for bitfield rpf_l3_v6_cmd{F}[F:0] */
#define HW_ATL2_RPF_L3_V6_CMD_MSKN 0x0080FFFFu
/* Lower bit position of bitfield rpf_l3_v6_cmd{F}[F:0] */
#define HW_ATL2_RPF_L3_V6_CMD_SHIFT 16
/* Width of bitfield rpf_l3_v6_cmd{F}[F:0] */
#define HW_ATL2_RPF_L3_V6_CMD_WIDTH 16
/* Default value of bitfield rpf_l3_v6_cmd{F}[F:0] */
#define HW_ATL2_RPF_L3_V6_CMD_DEFAULT 0x0

/* RX rpf_l3_v6_v4_select Bitfield Definitions
 * Preprocessor definitions for the bitfield "rpf_l3_v6_v4_select".
 * PORT="pif_rpf_l3_v6_v4_select"
 */

/* Register address for bitfield rpf_l3_v6_cmd{F}[F:0] */
#define HW_ATL2_RPF_L3_V6_V4_SELECT_ADR 0x00006500u
/* Bitmask for bitfield pif_rpf_l3_v6_v4_select*/
#define HW_ATL2_RPF_L3_V6_V4_SELECT_MSK 0x00800000u
/* Inverted bitmask for bitfield pif_rpf_l3_v6_v4_select */
#define HW_ATL2_RPF_L3_V6_V4_SELECT_MSKN 0xFF7FFFFFu
/* Lower bit position of bitfield pif_rpf_l3_v6_v4_select */
#define HW_ATL2_RPF_L3_V6_V4_SELECT_SHIFT 23
/* Width of bitfield pif_rpf_l3_v6_v4_select */
#define HW_ATL2_RPF_L3_V6_V4_SELECT_WIDTH 1
/* Default value of bitfield pif_rpf_l3_v6_v4_select*/
#define HW_ATL2_RPF_L3_V6_V4_SELECT_DEFAULT 0x0

/* RX rpf_l3_v4_req_tag{F}[2:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "rpf_l3_v4_req_tag{F}[2:0]".
 * Parameter: filter {F} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l3_v4_req_tag0[2:0]"
 */

/* Register address for bitfield rpf_l3_v4_req_tag{F}[2:0] */
#define HW_ATL2_RPF_L3_V4_TAG_ADR(filter) (0x00006500u + (filter) * 0x4)
/* Bitmask for bitfield rpf_l3_v4_req_tag{F}[2:0] */
#define HW_ATL2_RPF_L3_V4_TAG_MSK 0x00000070u
/* Inverted bitmask for bitfield rpf_l3_v4_req_tag{F}[2:0] */
#define HW_ATL2_RPF_L3_V4_TAG_MSKN 0xFFFFFF8Fu
/* Lower bit position of bitfield rpf_l3_v4_req_tag{F}[2:0] */
#define HW_ATL2_RPF_L3_V4_TAG_SHIFT 4
/* Width of bitfield rpf_l3_v4_req_tag{F}[2:0] */
#define HW_ATL2_RPF_L3_V4_TAG_WIDTH 3
/* Default value of bitfield rpf_l3_v4_req_tag{F}[2:0] */
#define HW_ATL2_RPF_L3_V4_TAG_DEFAULT 0x0

/* RX rpf_l3_v6_req_tag{F}[2:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "rpf_l3_v6_req_tag{F}[2:0]".
 * Parameter: filter {F} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l3_v6_req_tag0[2:0]"
 */

/* Register address for bitfield rpf_l3_v6_req_tag{F}[2:0] */
#define HW_ATL2_RPF_L3_V6_TAG_ADR(filter) (0x00006500u + (filter) * 0x4)
/* Bitmask for bitfield rpf_l3_v6_req_tag{F}[2:0] */
#define HW_ATL2_RPF_L3_V6_TAG_MSK 0x00700000
/* Inverted bitmask for bitfield rpf_l3_v6_req_tag{F}[2:0] */
#define HW_ATL2_RPF_L3_V6_TAG_MSKN 0xFF8FFFFFu
/* Lower bit position of bitfield rpf_l3_v6_req_tag{F}[2:0] */
#define HW_ATL2_RPF_L3_V6_TAG_SHIFT 20
/* Width of bitfield rpf_l3_v6_req_tag{F}[2:0] */
#define HW_ATL2_RPF_L3_V6_TAG_WIDTH 3
/* Default value of bitfield rpf_l3_v6_req_tag{F}[2:0] */
#define HW_ATL2_RPF_L3_V6_TAG_DEFAULT 0x0

/* RX rpf_l4_cmd{F}[2:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "rpf_l4_cmd{F}[2:0]".
 * Parameter: filter {F} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l4_cmd{F}[2:0]"
 */

/* Register address for bitfield rpf_l4_cmd{F}[2:0] */
#define HW_ATL2_RPF_L4_CMD_ADR(filter) (0x00006520u + (filter) * 0x4)
/* Bitmask for bitfield rpf_l4_cmd{F}[2:0] */
#define HW_ATL2_RPF_L4_CMD_MSK 0x00000007u
/* Inverted bitmask for bitfield rpf_l4_cmd{F}[2:0] */
#define HW_ATL2_RPF_L4_CMD_MSKN 0xFFFFFFF8u
/* Lower bit position of bitfield rpf_l4_cmd{F}[2:0] */
#define HW_ATL2_RPF_L4_CMD_SHIFT 0
/* Width of bitfield rpf_l4_cmd{F}[2:0]*/
#define HW_ATL2_RPF_L4_CMD_WIDTH 3
/* Default value of bitfield rpf_l4_cmd{F}[2:0] */
#define HW_ATL2_RPF_L4_CMD_DEFAULT 0x0

/* RX rpf_l4_req_tag{F}[2:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "rpf_l4_tag{F}[2:0]".
 * Parameter: filter {F} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l4_tag{F}[2:0]"
 */

/* Register address for bitfield rpf_l4_tag{F}[2:0] */
#define HW_ATL2_RPF_L4_TAG_ADR(filter) (0x00006520u + (filter) * 0x4)
/* Bitmask for bitfield rpf_l4_tag{F}[2:0] */
#define HW_ATL2_RPF_L4_TAG_MSK 0x00000070u
/* Inverted bitmask for bitfield rpf_l4_tag{F}[2:0] */
#define HW_ATL2_RPF_L4_TAG_MSKN 0xFFFFFF8Fu
/* Lower bit position of bitfield rpf_l4_tag{F}[2:0] */
#define HW_ATL2_RPF_L4_TAG_SHIFT 4
/* Width of bitfield rpf_l4_tag{F}[2:0]*/
#define HW_ATL2_RPF_L4_TAG_WIDTH 3
/* Default value of bitfield rpf_l4_tag{F}[2:0] */
#define HW_ATL2_RPF_L4_TAG_DEFAULT 0x0

/* RX l4_prot{F}[2:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "l4_prot{F}[2:0]".
 * Parameter: filter {F} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l4_prot0_i[2:0]"
 */

/* Register address for bitfield l4_prot{F}[2:0] */
#define HW_ATL2_RPF_L4_PROTF_ADR(filter) (0x00005380u + (filter) * 0x4)
/* Bitmask for bitfield l4_prot{F}[2:0] */
#define HW_ATL2_RPF_L4_PROTF_MSK 0x00000007u
/* Inverted bitmask for bitfield l4_prot{F}[2:0] */
#define HW_ATL2_RPF_L4_PROTF_MSKN 0xFFFFFFF8u
/* Lower bit position of bitfield l4_prot{F}[2:0] */
#define HW_ATL2_RPF_L4_PROTF_SHIFT 0
/* Width of bitfield l4_prot{F}[2:0] */
#define HW_ATL2_RPF_L4_PROTF_WIDTH 3
/* Default value of bitfield l4_prot{F}[2:0] */
#define HW_ATL2_RPF_L4_PROTF_DEFAULT 0x0

/* RX l4_sp{D}[F:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "l4_sp{D}[F:0]".
 * Parameter: srcport {D} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l4_sp0_i[15:0]"
 */

/* Register address for bitfield l4_sp{D}[F:0] */
#define HW_ATL2_RPF_L4_SPD_ADR(srcport) (0x00005400u + (srcport) * 0x4)
/* Bitmask for bitfield l4_sp{D}[F:0] */
#define HW_ATL2_RPF_L4_SPD_MSK 0x0000FFFFu
/* Inverted bitmask for bitfield l4_sp{D}[F:0] */
#define HW_ATL2_RPF_L4_SPD_MSKN 0xFFFF0000u
/* Lower bit position of bitfield l4_sp{D}[F:0] */
#define HW_ATL2_RPF_L4_SPD_SHIFT 0
/* Width of bitfield l4_sp{D}[F:0] */
#define HW_ATL2_RPF_L4_SPD_WIDTH 16
/* Default value of bitfield l4_sp{D}[F:0] */
#define HW_ATL2_RPF_L4_SPD_DEFAULT 0x0

/* RX l4_dp{D}[F:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "l4_dp{D}[F:0]".
 * Parameter: destport {D} | stride size 0x4 | range [0, 7]
 * PORT="pif_rpf_l4_dp0_i[15:0]"
 */

/* Register address for bitfield l4_dp{D}[F:0] */
#define HW_ATL2_RPF_L4_DPD_ADR(destport) (0x00005420u + (destport) * 0x4)
/* Bitmask for bitfield l4_dp{D}[F:0] */
#define HW_ATL2_RPF_L4_DPD_MSK 0x0000FFFFu
/* Inverted bitmask for bitfield l4_dp{D}[F:0] */
#define HW_ATL2_RPF_L4_DPD_MSKN 0xFFFF0000u
/* Lower bit position of bitfield l4_dp{D}[F:0] */
#define HW_ATL2_RPF_L4_DPD_SHIFT 0
/* Width of bitfield l4_dp{D}[F:0] */
#define HW_ATL2_RPF_L4_DPD_WIDTH 16
/* Default value of bitfield l4_dp{D}[F:0] */
#define HW_ATL2_RPF_L4_DPD_DEFAULT 0x0


/*! \brief Register address for bitfield flex_en{H} */
#define HW_ATL2_RPF_FLEX_ENH_ADR(header) (0x00005460 + (header) * 0x20)
/*! \brief Bitmask for bitfield flex_en{H} */
#define HW_ATL2_RPF_FLEX_ENH_MSK 0x80000000
/*! \brief Inverted bitmask for bitfield flex_en{H} */
#define HW_ATL2_RPF_FLEX_ENH_MSKN 0x7FFFFFFF
/*! \brief Lower bit position of bitfield flex_en{H} */
#define HW_ATL2_RPF_FLEX_ENH_SHIFT 31
/*! \brief Width of bitfield flex_en{H} */
#define HW_ATL2_RPF_FLEX_ENH_WIDTH 1
/*! \brief Default value of bitfield flex_en{H} */
#define HW_ATL2_RPF_FLEX_ENH_DEFAULT 0x0

/*! \brief Register address for bitfield flex_rxqen{H} */
#define HW_ATL2_RPF_FLEX_RXEN_ADR(header) (0x00005460 + (header) * 0x20)
/*! \brief Bitmask for bitfield flex_rxqen{H} */
#define HW_ATL2_RPF_FLEX_RXEN_MSK 0x40000000
/*! \brief Inverted bitmask for bitfield flex_rxqen{H} */
#define HW_ATL2_RPF_FLEX_RXEN_MSKN 0xbFFFFFFF
/*! \brief Lower bit position of bitfield flex_rxqen{H} */
#define HW_ATL2_RPF_FLEX_RXEN_SHIFT 30
/*! \brief Width of bitfield flex_rxqen{H} */
#define HW_ATL2_RPF_FLEX_RXEN_WIDTH 1
/*! \brief Default value of bitfield flex_rxqen{H} */
#define HW_ATL2_RPF_FLEX_RXEN_DEFAULT 0x0

/*! \brief Register address for bitfield flex_tag0[1:0] */
#define HW_ATL2_RPF_FLEX_TAG_ADR(header) (0x00005460 + (header) * 0x20)
/*! \brief Bitmask for bitfield flex_tag0[1:0] */
#define HW_ATL2_RPF_FLEX_TAG_MSK 0x06000000
/*! \brief Inverted bitmask for bitfield flex_tag0[1:0] */
#define HW_ATL2_RPF_FLEX_TAG_MSKN 0xF9FFFFFF
/*! \brief Lower bit position of bitfield flex_tag0[1:0] */
#define HW_ATL2_RPF_FLEX_TAG_SHIFT 25
/*! \brief Width of bitfield flex_tag0[1:0] */
#define HW_ATL2_RPF_FLEX_TAG_WIDTH 2
/*! \brief Default value of bitfield flex_tag0[1:0] */
#define HW_ATL2_RPF_FLEX_TAG_DEFAULT 0x0

/*! \brief Register address for bitfield flex_rxq{H}[4:0] */
#define HW_ATL2_RPF_FLEX_RXQH_ADR(header) (0x00005460 + (header) * 0x20)
/*! \brief Bitmask for bitfield flex_rxq{H}[4:0] */
#define HW_ATL2_RPF_FLEX_RXQH_MSK 0x01F00000
/*! \brief Inverted bitmask for bitfield flex_rxq{H}[4:0] */
#define HW_ATL2_RPF_FLEX_RXQH_MSKN 0xFE0FFFFF
/*! \brief Lower bit position of bitfield flex_rxq{H}[4:0] */
#define HW_ATL2_RPF_FLEX_RXQH_SHIFT 20
/*! \brief Width of bitfield flex_rxq{H}[4:0] */
#define HW_ATL2_RPF_FLEX_RXQH_WIDTH 5
/*! \brief Default value of bitfield flex_rxq{H}[4:0] */
#define HW_ATL2_RPF_FLEX_RXQH_DEFAULT 0x0

/*! \brief Register address for bitfield flex_mng_rxq{H} */
#define HW_ATL2_RPF_FLEX_MNGRXQH_ADR(header) (0x00005460 + (header) * 0x20)
/*! \brief Bitmask for bitfield flex_mng_rxq{H} */
#define HW_ATL2_RPF_FLEX_MNGRXQH_MSK 0x00080000
/*! \brief Inverted bitmask for bitfield flex_mng_rxq{H} */
#define HW_ATL2_RPF_FLEX_MNGRXQH_MSKN 0xFFF7FFFF
/*! \brief Lower bit position of bitfield flex_mng_rxq{H} */
#define HW_ATL2_RPF_FLEX_MNGRXQH_SHIFT 19
/*! \brief Width of bitfield flex_mng_rxq{H} */
#define HW_ATL2_RPF_FLEX_MNGRXQH_WIDTH 1
/*! \brief Default value of bitfield flex_mng_rxq{H} */
#define HW_ATL2_RPF_FLEX_MNGRXQH_DEFAULT 0x0

/*! \brief Register address for bitfield flex_act{H}[2:0] */
#define HW_ATL2_RPF_FLEX_ACTH_ADR(header) (0x00005460 + (header) * 0x20)
/*! \brief Bitmask for bitfield flex_act{H}[2:0] */
#define HW_ATL2_RPF_FLEX_ACTH_MSK 0x00070000
/*! \brief Inverted bitmask for bitfield flex_act{H}[2:0] */
#define HW_ATL2_RPF_FLEX_ACTH_MSKN 0xFFF8FFFF
/*! \brief Lower bit position of bitfield flex_act{H}[2:0] */
#define HW_ATL2_RPF_FLEX_ACTH_SHIFT 16
/*! \brief Width of bitfield flex_act{H}[2:0] */
#define HW_ATL2_RPF_FLEX_ACTH_WIDTH 3
/*! \brief Default value of bitfield flex_act{H}[2:0] */
#define HW_ATL2_RPF_FLEX_ACTH_DEFAULT 0x0

/*! \brief Register address for bitfield flex_byte_a_loc{H}[6:0] */
#define HW_ATL2_RPF_FLEX_BYTEALOCH_ADR(header) (0x00005460 + (header) * 0x20)
/*! \brief Bitmask for bitfield flex_byte_a_loc{H}[6:0] */
#define HW_ATL2_RPF_FLEX_BYTEALOCH_MSK 0x00007F00
/*! \brief Inverted bitmask for bitfield flex_byte_a_loc{H}[6:0] */
#define HW_ATL2_RPF_FLEX_BYTEALOCH_MSKN 0xFFFF80FF
/*! \brief Lower bit position of bitfield flex_byte_a_loc{H}[6:0] */
#define HW_ATL2_RPF_FLEX_BYTEALOCH_SHIFT 8
/*! \brief Width of bitfield flex_byte_a_loc{H}[6:0] */
#define HW_ATL2_RPF_FLEX_BYTEALOCH_WIDTH 7
/*! \brief Default value of bitfield flex_byte_a_loc{H}[6:0] */
#define HW_ATL2_RPF_FLEX_BYTEALOCH_DEFAULT 0x0

/*! \brief Register address for bitfield flex_byte_b_loc{H}[6:0] */
#define HW_ATL2_RPF_FLEX_BYTEBLOCH_ADR(header) (0x00005460 + (header) * 0x20)
/*! \brief Bitmask for bitfield flex_byte_b_loc{H}[6:0] */
#define HW_ATL2_RPF_FLEX_BYTEBLOCH_MSK 0x0000007F
/*! \brief Inverted bitmask for bitfield flex_byte_b_loc{H}[6:0] */
#define HW_ATL2_RPF_FLEX_BYTEBLOCH_MSKN 0xFFFFFF80
/*! \brief Lower bit position of bitfield flex_byte_b_loc{H}[6:0] */
#define HW_ATL2_RPF_FLEX_BYTEBLOCH_SHIFT 0
/*! \brief Width of bitfield flex_byte_b_loc{H}[6:0] */
#define HW_ATL2_RPF_FLEX_BYTEBLOCH_WIDTH 7
/*! \brief Default value of bitfield flex_byte_b_loc{H}[6:0] */
#define HW_ATL2_RPF_FLEX_BYTEBLOCH_DEFAULT 0x0

/*! \brief Register address for bitfield flex_byte_a_msk{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEAMSKH_ADR(header) (0x00005464 + (header) * 0x20)
/*! \brief Bitmask for bitfield flex_byte_a_msk{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEAMSKH_MSK 0xFF000000
/*! \brief Inverted bitmask for bitfield flex_byte_a_msk{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEAMSKH_MSKN 0x00FFFFFF
/*! \brief Lower bit position of bitfield flex_byte_a_msk{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEAMSKH_SHIFT 24
/*! \brief Width of bitfield flex_byte_a_msk{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEAMSKH_WIDTH 8
/*! \brief Default value of bitfield flex_byte_a_msk{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEAMSKH_DEFAULT 0x0

/*! \brief Register address for bitfield flex_byte_b_msk{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEBMSKH_ADR(header) (0x00005464 + (header) * 0x20)
/*! \brief Bitmask for bitfield flex_byte_b_msk{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEBMSKH_MSK 0x00FF0000
/*! \brief Inverted bitmask for bitfield flex_byte_b_msk{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEBMSKH_MSKN 0xFF00FFFF
/*! \brief Lower bit position of bitfield flex_byte_b_msk{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEBMSKH_SHIFT 16
/*! \brief Width of bitfield flex_byte_b_msk{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEBMSKH_WIDTH 8
/*! \brief Default value of bitfield flex_byte_b_msk{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEBMSKH_DEFAULT 0x0

/*! \brief Register address for bitfield flex_byte_a_pat{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEAPATH_ADR(header) (0x00005464 + (header) * 0x20)
/*! \brief Bitmask for bitfield flex_byte_a_pat{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEAPATH_MSK 0x0000FF00
/*! \brief Inverted bitmask for bitfield flex_byte_a_pat{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEAPATH_MSKN 0xFFFF00FF
/*! \brief Lower bit position of bitfield flex_byte_a_pat{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEAPATH_SHIFT 8
/*! \brief Width of bitfield flex_byte_a_pat{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEAPATH_WIDTH 8
/*! \brief Default value of bitfield flex_byte_a_pat{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEAPATH_DEFAULT 0x0

/*! \brief Register address for bitfield flex_byte_b_pat{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEBPATH_ADR(header) (0x00005464 + (header) * 0x20)
/*! \brief Bitmask for bitfield flex_byte_b_pat{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEBPATH_MSK 0x000000FF
/*! \brief Inverted bitmask for bitfield flex_byte_b_pat{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEBPATH_MSKN 0xFFFFFF00
/*! \brief Lower bit position of bitfield flex_byte_b_pat{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEBPATH_SHIFT 0
/*! \brief Width of bitfield flex_byte_b_pat{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEBPATH_WIDTH 8
/*! \brief Default value of bitfield flex_byte_b_pat{H}[7:0] */
#define HW_ATL2_RPF_FLEX_BYTEBPATH_DEFAULT 0x0

/*! \brief Register address for bitfield flex_byte_msk{H}_word{W}[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORDW_ADR(header) (0x00005468 + (header) * 0x20)
/*! \brief Bitmask for bitfield flex_byte_msk{H}_word{W}[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORDW_MSK 0xFFFFFFFF
/*! \brief Inverted bitmask for bitfield flex_byte_msk{H}_word{W}[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORDW_MSKN 0x00000000
/*! \brief Lower bit position of bitfield flex_byte_msk{H}_word{W}[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORDW_SHIFT 0
/*! \brief Width of bitfield flex_byte_msk{H}_word{W}[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORDW_WIDTH 32
/*! \brief Default value of bitfield flex_byte_msk{H}_word{W}[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORDW_DEFAULT 0x0

/*! \brief Register address for bitfield flex_byte_msk{H}_word1[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORD1_ADR(header) (0x0000546C + (header) * 0x20)
/*! \brief Bitmask for bitfield flex_byte_msk{H}_word1[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORD1_MSK 0xFFFFFFFF
/*! \brief Inverted bitmask for bitfield flex_byte_msk{H}_word1[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORD1_MSKN 0x00000000
/*! \brief Lower bit position of bitfield flex_byte_msk{H}_word1[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORD1_SHIFT 0
/*! \brief Width of bitfield flex_byte_msk{H}_word1[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORD1_WIDTH 32
/*! \brief Default value of bitfield flex_byte_msk{H}_word1[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORD1_DEFAULT 0x0

/*! \brief Register address for bitfield flex_byte_msk{H}_word2[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORD2_ADR(header) (0x00005470 + (header) * 0x20)
/*! \brief Bitmask for bitfield flex_byte_msk{H}_word2[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORD2_MSK 0xFFFFFFFF
/*! \brief Inverted bitmask for bitfield flex_byte_msk{H}_word2[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORD2_MSKN 0x00000000
/*! \brief Lower bit position of bitfield flex_byte_msk{H}_word2[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORD2_SHIFT 0
/*! \brief Width of bitfield flex_byte_msk{H}_word2[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORD2_WIDTH 32
/*! \brief Default value of bitfield flex_byte_msk{H}_word2[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORD2_DEFAULT 0x0

/*! \brief Register address for bitfield flex_byte_msk{H}_word3[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORD3_ADR(header) (0x00005474 + (header) * 0x20)
/*! \brief Bitmask for bitfield flex_byte_msk{H}_word3[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORD3_MSK 0xFFFFFFFF
/*! \brief Inverted bitmask for bitfield flex_byte_msk{H}_word3[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORD3_MSKN 0x00000000
/*! \brief Lower bit position of bitfield flex_byte_msk{H}_word3[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORD3_SHIFT 0
/*! \brief Width of bitfield flex_byte_msk{H}_word3[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORD3_WIDTH 32
/*! \brief Default value of bitfield flex_byte_msk{H}_word3[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEMSKHWORD3_DEFAULT 0x0

/*! \brief Register address for bitfield flex_byte_pat_sel */
#define HW_ATL2_RPF_FLEX_BYTEPATSEL_ADR 0x000054A0
/*! \brief Bitmask for bitfield flex_byte_pat_sel */
#define HW_ATL2_RPF_FLEX_BYTEPATSEL_MSK 0x00000100
/*! \brief Inverted bitmask for bitfield flex_byte_pat_sel */
#define HW_ATL2_RPF_FLEX_BYTEPATSEL_MSKN 0xFFFFFEFF
/*! \brief Lower bit position of bitfield flex_byte_pat_sel */
#define HW_ATL2_RPF_FLEX_BYTEPATSEL_SHIFT 8
/*! \brief Width of bitfield flex_byte_pat_sel */
#define HW_ATL2_RPF_FLEX_BYTEPATSEL_WIDTH 1
/*! \brief Default value of bitfield flex_byte_pat_sel */
#define HW_ATL2_RPF_FLEX_BYTEPATSEL_DEFAULT 0x0
/*@}*/

/*! \brief Register address for bitfield flex_byte_pat_wr_en_i */
#define HW_ATL2_RPF_FLEX_BYTEPATWRENI_ADR 0x000054A0
/*! \brief Bitmask for bitfield flex_byte_pat_wr_en_i */
#define HW_ATL2_RPF_FLEX_BYTEPATWRENI_MSK 0x00000040
/*! \brief Inverted bitmask for bitfield flex_byte_pat_wr_en_i */
#define HW_ATL2_RPF_FLEX_BYTEPATWRENI_MSKN 0xFFFFFFBF
/*! \brief Lower bit position of bitfield flex_byte_pat_wr_en_i */
#define HW_ATL2_RPF_FLEX_BYTEPATWRENI_SHIFT 6
/*! \brief Width of bitfield flex_byte_pat_wr_en_i */
#define HW_ATL2_RPF_FLEX_BYTEPATWRENI_WIDTH 1
/*! \brief Default value of bitfield flex_byte_pat_wr_en_i */
#define HW_ATL2_RPF_FLEX_BYTEPATWRENI_DEFAULT 0x0

/*! \brief Register address for bitfield flex_byte_pat_addr[5:0] */
#define HW_ATL2_RPF_FLEX_BYTEPATADDR_ADR 0x000054A0
/*! \brief Bitmask for bitfield flex_byte_pat_addr[5:0] */
#define HW_ATL2_RPF_FLEX_BYTEPATADDR_MSK 0x0000003F
/*! \brief Inverted bitmask for bitfield flex_byte_pat_addr[5:0] */
#define HW_ATL2_RPF_FLEX_BYTEPATADDR_MSKN 0xFFFFFFC0
/*! \brief Lower bit position of bitfield flex_byte_pat_addr[5:0] */
#define HW_ATL2_RPF_FLEX_BYTEPATADDR_SHIFT 0
/*! \brief Width of bitfield flex_byte_pat_addr[5:0] */
#define HW_ATL2_RPF_FLEX_BYTEPATADDR_WIDTH 6
/*! \brief Default value of bitfield flex_byte_pat_addr[5:0] */
#define HW_ATL2_RPF_FLEX_BYTEPATADDR_DEFAULT 0x0

/*! \brief Register address for bitfield flex_byte_pat_wr_data[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEPATWRDATA_ADR 0x000054A4
/*! \brief Bitmask for bitfield flex_byte_pat_wr_data[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEPATWRDATA_MSK 0xFFFFFFFF
/*! \brief Inverted bitmask for bitfield flex_byte_pat_wr_data[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEPATWRDATA_MSKN 0x00000000
/*! \brief Lower bit position of bitfield flex_byte_pat_wr_data[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEPATWRDATA_SHIFT 0
/*! \brief Width of bitfield flex_byte_pat_wr_data[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEPATWRDATA_WIDTH 32
/*! \brief Default value of bitfield flex_byte_pat_wr_data[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEPATWRDATA_DEFAULT 0x0

/*! \brief Register address for bitfield flex_byte_pat_rd_data[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEPATRDDATA_ADR 0x000054A8
/*! \brief Bitmask for bitfield flex_byte_pat_rd_data[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEPATRDDATA_MSK 0xFFFFFFFF
/*! \brief Inverted bitmask for bitfield flex_byte_pat_rd_data[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEPATRDDATA_MSKN 0x00000000
/*! \brief Lower bit position of bitfield flex_byte_pat_rd_data[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEPATRDDATA_SHIFT 0
/*! \brief Width of bitfield flex_byte_pat_rd_data[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEPATRDDATA_WIDTH 32
/*! \brief Default value of bitfield flex_byte_pat_rd_data[1F:0] */
#define HW_ATL2_RPF_FLEX_BYTEPATRDDATA_DEFAULT 0x0

/*! \brief Register address for bitfield flex_filter_len[5:0] */
#define HW_ATL2_RPF_FLEX_FILTERLEN_ADR 0x000054AC
/*! \brief Bitmask for bitfield flex_filter_len[5:0] */
#define HW_ATL2_RPF_FLEX_FILTERLEN_MSK 0x0000003F
/*! \brief Inverted bitmask for bitfield flex_filter_len[5:0] */
#define HW_ATL2_RPF_FLEX_FILTERLEN_MSKN 0xFFFFFFC0
/*! \brief Lower bit position of bitfield flex_filter_len[5:0] */
#define HW_ATL2_RPF_FLEX_FILTERLEN_SHIFT 0
/*! \brief Width of bitfield flex_filter_len[5:0] */
#define HW_ATL2_RPF_FLEX_FILTERLEN_WIDTH 6
/*! \brief Default value of bitfield flex_filter_len[5:0] */
#define HW_ATL2_RPF_FLEX_FILTERLEN_DEFAULT 0x20

/* rx ipv4_chk_en bitfield definitions
 * preprocessor definitions for the bitfield "ipv4_chk_en".
 * port="pif_rpo_ipv4_chk_en_i"
 */

/* register address for bitfield ipv4_chk_en */
#define HW_ATL2_RPO_IPV4CHK_EN_ADR 0x00005580
/* bitmask for bitfield ipv4_chk_en */
#define HW_ATL2_RPO_IPV4CHK_EN_MSK 0x00000002
/* inverted bitmask for bitfield ipv4_chk_en */
#define HW_ATL2_RPO_IPV4CHK_EN_MSKN 0xfffffffd
/* lower bit position of bitfield ipv4_chk_en */
#define HW_ATL2_RPO_IPV4CHK_EN_SHIFT 1
/* width of bitfield ipv4_chk_en */
#define HW_ATL2_RPO_IPV4CHK_EN_WIDTH 1
/* default value of bitfield ipv4_chk_en */
#define HW_ATL2_RPO_IPV4CHK_EN_DEFAULT 0x0

/* rx desc{d}_vl_strip bitfield definitions
 * preprocessor definitions for the bitfield "desc{d}_vl_strip".
 * parameter: descriptor {d} | stride size 0x20 | range [0, 31]
 * port="pif_rpo_desc_vl_strip_i[0]"
 */

/* register address for bitfield desc{d}_vl_strip */
#define HW_ATL2_RPO_DESCDVL_STRIP_ADR(descriptor) \
	(0x00005b08 + (descriptor) * 0x20)
/* bitmask for bitfield desc{d}_vl_strip */
#define HW_ATL2_RPO_DESCDVL_STRIP_MSK 0x20000000
/* inverted bitmask for bitfield desc{d}_vl_strip */
#define HW_ATL2_RPO_DESCDVL_STRIP_MSKN 0xdfffffff
/* lower bit position of bitfield desc{d}_vl_strip */
#define HW_ATL2_RPO_DESCDVL_STRIP_SHIFT 29
/* width of bitfield desc{d}_vl_strip */
#define HW_ATL2_RPO_DESCDVL_STRIP_WIDTH 1
/* default value of bitfield desc{d}_vl_strip */
#define HW_ATL2_RPO_DESCDVL_STRIP_DEFAULT 0x0

/* rx l4_chk_en bitfield definitions
 * preprocessor definitions for the bitfield "l4_chk_en".
 * port="pif_rpo_l4_chk_en_i"
 */

/* register address for bitfield l4_chk_en */
#define HW_ATL2_RPOL4CHK_EN_ADR 0x00005580
/* bitmask for bitfield l4_chk_en */
#define HW_ATL2_RPOL4CHK_EN_MSK 0x00000001
/* inverted bitmask for bitfield l4_chk_en */
#define HW_ATL2_RPOL4CHK_EN_MSKN 0xfffffffe
/* lower bit position of bitfield l4_chk_en */
#define HW_ATL2_RPOL4CHK_EN_SHIFT 0
/* width of bitfield l4_chk_en */
#define HW_ATL2_RPOL4CHK_EN_WIDTH 1
/* default value of bitfield l4_chk_en */
#define HW_ATL2_RPOL4CHK_EN_DEFAULT 0x0

/* RX outer_vl_ins_mode Bitfield Definitions
 *   Preprocessor definitions for the bitfield "outer_vl_ins_mode".
 *   PORT="pif_rpo_outer_vl_mode_i"
 */

/* Register address for bitfield outer_vl_ins_mode */
#define HW_ATL2_RPO_OUTER_VL_INS_MODE_ADR 0x00005580
/* Bitmask for bitfield outer_vl_ins_mode */
#define HW_ATL2_RPO_OUTER_VL_INS_MODE_MSK 0x00000004
/* Inverted bitmask for bitfield outer_vl_ins_mode */
#define HW_ATL2_RPO_OUTER_VL_INS_MODE_MSKN 0xFFFFFFFB
/* Lower bit position of bitfield outer_vl_ins_mode */
#define HW_ATL2_RPO_OUTER_VL_INS_MODE_SHIFT 2
/* Width of bitfield outer_vl_ins_mode */
#define HW_ATL2_RPO_OUTER_VL_INS_MODE_WIDTH 1
/* Default value of bitfield outer_vl_ins_mode */
#define HW_ATL2_RPO_OUTER_VL_INS_MODE_DEFAULT 0x0

/* rx reg_res_dsbl bitfield definitions
 * preprocessor definitions for the bitfield "reg_res_dsbl".
 * port="pif_rx_reg_res_dsbl_i"
 */

/* register address for bitfield reg_res_dsbl */
#define HW_ATL2_RX_REG_RES_DSBL_ADR 0x00005000
/* bitmask for bitfield reg_res_dsbl */
#define HW_ATL2_RX_REG_RES_DSBL_MSK 0x20000000
/* inverted bitmask for bitfield reg_res_dsbl */
#define HW_ATL2_RX_REG_RES_DSBL_MSKN 0xdfffffff
/* lower bit position of bitfield reg_res_dsbl */
#define HW_ATL2_RX_REG_RES_DSBL_SHIFT 29
/* width of bitfield reg_res_dsbl */
#define HW_ATL2_RX_REG_RES_DSBL_WIDTH 1
/* default value of bitfield reg_res_dsbl */
#define HW_ATL2_RX_REG_RES_DSBL_DEFAULT 0x1

/* tx dca{d}_cpuid[7:0] bitfield definitions
 * preprocessor definitions for the bitfield "dca{d}_cpuid[7:0]".
 * parameter: dca {d} | stride size 0x4 | range [0, 31]
 * port="pif_tdm_dca0_cpuid_i[7:0]"
 */

/* register address for bitfield dca{d}_cpuid[7:0] */
#define HW_ATL2_TDM_DCADCPUID_ADR(dca) (0x00008400 + (dca) * 0x4)
/* bitmask for bitfield dca{d}_cpuid[7:0] */
#define HW_ATL2_TDM_DCADCPUID_MSK 0x000000ff
/* inverted bitmask for bitfield dca{d}_cpuid[7:0] */
#define HW_ATL2_TDM_DCADCPUID_MSKN 0xffffff00
/* lower bit position of bitfield dca{d}_cpuid[7:0] */
#define HW_ATL2_TDM_DCADCPUID_SHIFT 0
/* width of bitfield dca{d}_cpuid[7:0] */
#define HW_ATL2_TDM_DCADCPUID_WIDTH 8
/* default value of bitfield dca{d}_cpuid[7:0] */
#define HW_ATL2_TDM_DCADCPUID_DEFAULT 0x0

/* tx lso_en[1f:0] bitfield definitions
 * preprocessor definitions for the bitfield "lso_en[1f:0]".
 * port="pif_tdm_lso_en_i[31:0]"
 */

/* register address for bitfield lso_en[1f:0] */
#define HW_ATL2_TDM_LSO_EN_ADR 0x00007810
/* bitmask for bitfield lso_en[1f:0] */
#define HW_ATL2_TDM_LSO_EN_MSK 0xffffffff
/* inverted bitmask for bitfield lso_en[1f:0] */
#define HW_ATL2_TDM_LSO_EN_MSKN 0x00000000
/* lower bit position of bitfield lso_en[1f:0] */
#define HW_ATL2_TDM_LSO_EN_SHIFT 0
/* width of bitfield lso_en[1f:0] */
#define HW_ATL2_TDM_LSO_EN_WIDTH 32
/* default value of bitfield lso_en[1f:0] */
#define HW_ATL2_TDM_LSO_EN_DEFAULT 0x0

/* tx dca_en bitfield definitions
 * preprocessor definitions for the bitfield "dca_en".
 * port="pif_tdm_dca_en_i"
 */

/* register address for bitfield dca_en */
#define HW_ATL2_TDM_DCA_EN_ADR 0x00008480
/* bitmask for bitfield dca_en */
#define HW_ATL2_TDM_DCA_EN_MSK 0x80000000
/* inverted bitmask for bitfield dca_en */
#define HW_ATL2_TDM_DCA_EN_MSKN 0x7fffffff
/* lower bit position of bitfield dca_en */
#define HW_ATL2_TDM_DCA_EN_SHIFT 31
/* width of bitfield dca_en */
#define HW_ATL2_TDM_DCA_EN_WIDTH 1
/* default value of bitfield dca_en */
#define HW_ATL2_TDM_DCA_EN_DEFAULT 0x1

/* tx dca_mode[3:0] bitfield definitions
 * preprocessor definitions for the bitfield "dca_mode[3:0]".
 * port="pif_tdm_dca_mode_i[3:0]"
 */

/* register address for bitfield dca_mode[3:0] */
#define HW_ATL2_TDM_DCA_MODE_ADR 0x00008480
/* bitmask for bitfield dca_mode[3:0] */
#define HW_ATL2_TDM_DCA_MODE_MSK 0x0000000f
/* inverted bitmask for bitfield dca_mode[3:0] */
#define HW_ATL2_TDM_DCA_MODE_MSKN 0xfffffff0
/* lower bit position of bitfield dca_mode[3:0] */
#define HW_ATL2_TDM_DCA_MODE_SHIFT 0
/* width of bitfield dca_mode[3:0] */
#define HW_ATL2_TDM_DCA_MODE_WIDTH 4
/* default value of bitfield dca_mode[3:0] */
#define HW_ATL2_TDM_DCA_MODE_DEFAULT 0x0

/* tx dca{d}_desc_en bitfield definitions
 * preprocessor definitions for the bitfield "dca{d}_desc_en".
 * parameter: dca {d} | stride size 0x4 | range [0, 31]
 * port="pif_tdm_dca_desc_en_i[0]"
 */

/* register address for bitfield dca{d}_desc_en */
#define HW_ATL2_TDM_DCADDESC_EN_ADR(dca) (0x00008400 + (dca) * 0x4)
/* bitmask for bitfield dca{d}_desc_en */
#define HW_ATL2_TDM_DCADDESC_EN_MSK 0x80000000
/* inverted bitmask for bitfield dca{d}_desc_en */
#define HW_ATL2_TDM_DCADDESC_EN_MSKN 0x7fffffff
/* lower bit position of bitfield dca{d}_desc_en */
#define HW_ATL2_TDM_DCADDESC_EN_SHIFT 31
/* width of bitfield dca{d}_desc_en */
#define HW_ATL2_TDM_DCADDESC_EN_WIDTH 1
/* default value of bitfield dca{d}_desc_en */
#define HW_ATL2_TDM_DCADDESC_EN_DEFAULT 0x0

/* tx desc{d}_en bitfield definitions
 * preprocessor definitions for the bitfield "desc{d}_en".
 * parameter: descriptor {d} | stride size 0x40 | range [0, 31]
 * port="pif_tdm_desc_en_i[0]"
 */

/* register address for bitfield desc{d}_en */
#define HW_ATL2_TDM_DESCDEN_ADR(descriptor) (0x00007c08 + (descriptor) * 0x40)
/* bitmask for bitfield desc{d}_en */
#define HW_ATL2_TDM_DESCDEN_MSK 0x80000000
/* inverted bitmask for bitfield desc{d}_en */
#define HW_ATL2_TDM_DESCDEN_MSKN 0x7fffffff
/* lower bit position of bitfield desc{d}_en */
#define HW_ATL2_TDM_DESCDEN_SHIFT 31
/* width of bitfield desc{d}_en */
#define HW_ATL2_TDM_DESCDEN_WIDTH 1
/* default value of bitfield desc{d}_en */
#define HW_ATL2_TDM_DESCDEN_DEFAULT 0x0

/* tx desc{d}_hd[c:0] bitfield definitions
 * preprocessor definitions for the bitfield "desc{d}_hd[c:0]".
 * parameter: descriptor {d} | stride size 0x40 | range [0, 31]
 * port="tdm_pif_desc0_hd_o[12:0]"
 */

/* register address for bitfield desc{d}_hd[c:0] */
#define HW_ATL2_TDM_DESCDHD_ADR(descriptor) (0x00007c0c + (descriptor) * 0x40)
/* bitmask for bitfield desc{d}_hd[c:0] */
#define HW_ATL2_TDM_DESCDHD_MSK 0x00001fff
/* inverted bitmask for bitfield desc{d}_hd[c:0] */
#define HW_ATL2_TDM_DESCDHD_MSKN 0xffffe000
/* lower bit position of bitfield desc{d}_hd[c:0] */
#define HW_ATL2_TDM_DESCDHD_SHIFT 0
/* width of bitfield desc{d}_hd[c:0] */
#define HW_ATL2_TDM_DESCDHD_WIDTH 13

/* tx desc{d}_len[9:0] bitfield definitions
 * preprocessor definitions for the bitfield "desc{d}_len[9:0]".
 * parameter: descriptor {d} | stride size 0x40 | range [0, 31]
 * port="pif_tdm_desc0_len_i[9:0]"
 */

/* register address for bitfield desc{d}_len[9:0] */
#define HW_ATL2_TDM_DESCDLEN_ADR(descriptor) (0x00007c08 + (descriptor) * 0x40)
/* bitmask for bitfield desc{d}_len[9:0] */
#define HW_ATL2_TDM_DESCDLEN_MSK 0x00001ff8
/* inverted bitmask for bitfield desc{d}_len[9:0] */
#define HW_ATL2_TDM_DESCDLEN_MSKN 0xffffe007
/* lower bit position of bitfield desc{d}_len[9:0] */
#define HW_ATL2_TDM_DESCDLEN_SHIFT 3
/* width of bitfield desc{d}_len[9:0] */
#define HW_ATL2_TDM_DESCDLEN_WIDTH 10
/* default value of bitfield desc{d}_len[9:0] */
#define HW_ATL2_TDM_DESCDLEN_DEFAULT 0x0


/* bitmask for bitfield desc{d}_tswben */
#define HW_ATL2_TDM_DESCDTSWBEN_MSK 0x00040000
/* lower bit position of bitfield desc{d}_tswben */
#define HW_ATL2_TDM_DESCDTSWBEN_SHIFT 18

/* bitmask for bitfield desc{d}_tsen */
#define HW_ATL2_TDM_DESCDTSEN_MSK 0x00020000
/* lower bit position of bitfield desc{d}_tsen */
#define HW_ATL2_TDM_DESCDTSEN_SHIFT 17

/* bitmask for bitfield desc{d}_lten */
#define HW_ATL2_TDM_DESCDLTEN_MSK 0x00010000
/* lower bit position of bitfield desc{d}_lten */
#define HW_ATL2_TDM_DESCDLTEN_SHIFT 16

/* tx int_desc_wrb_en bitfield definitions
 * preprocessor definitions for the bitfield "int_desc_wrb_en".
 * port="pif_tdm_int_desc_wrb_en_i"
 */

/* register address for bitfield int_desc_wrb_en */
#define HW_ATL2_TDM_INT_DESC_WRB_EN_ADR 0x00007b40
/* bitmask for bitfield int_desc_wrb_en */
#define HW_ATL2_TDM_INT_DESC_WRB_EN_MSK 0x00000002
/* inverted bitmask for bitfield int_desc_wrb_en */
#define HW_ATL2_TDM_INT_DESC_WRB_EN_MSKN 0xfffffffd
/* lower bit position of bitfield int_desc_wrb_en */
#define HW_ATL2_TDM_INT_DESC_WRB_EN_SHIFT 1
/* width of bitfield int_desc_wrb_en */
#define HW_ATL2_TDM_INT_DESC_WRB_EN_WIDTH 1
/* default value of bitfield int_desc_wrb_en */
#define HW_ATL2_TDM_INT_DESC_WRB_EN_DEFAULT 0x0

/* tx desc{d}_wrb_thresh[6:0] bitfield definitions
 * preprocessor definitions for the bitfield "desc{d}_wrb_thresh[6:0]".
 * parameter: descriptor {d} | stride size 0x40 | range [0, 31]
 * port="pif_tdm_desc0_wrb_thresh_i[6:0]"
 */

/* register address for bitfield desc{d}_wrb_thresh[6:0] */
#define HW_ATL2_TDM_DESCDWRB_THRESH_ADR(descriptor) \
	(0x00007c18 + (descriptor) * 0x40)
/* bitmask for bitfield desc{d}_wrb_thresh[6:0] */
#define HW_ATL2_TDM_DESCDWRB_THRESH_MSK 0x00007f00
/* inverted bitmask for bitfield desc{d}_wrb_thresh[6:0] */
#define HW_ATL2_TDM_DESCDWRB_THRESH_MSKN 0xffff80ff
/* lower bit position of bitfield desc{d}_wrb_thresh[6:0] */
#define HW_ATL2_TDM_DESCDWRB_THRESH_SHIFT 8
/* width of bitfield desc{d}_wrb_thresh[6:0] */
#define HW_ATL2_TDM_DESCDWRB_THRESH_WIDTH 7
/* default value of bitfield desc{d}_wrb_thresh[6:0] */
#define HW_ATL2_TDM_DESCDWRB_THRESH_DEFAULT 0x0

/* tx lso_tcp_flag_first[b:0] bitfield definitions
 * preprocessor definitions for the bitfield "lso_tcp_flag_first[b:0]".
 * port="pif_thm_lso_tcp_flag_first_i[11:0]"
 */

/* register address for bitfield lso_tcp_flag_first[b:0] */
#define HW_ATL2_THM_LSO_TCP_FLAG_FIRST_ADR 0x00007820
/* bitmask for bitfield lso_tcp_flag_first[b:0] */
#define HW_ATL2_THM_LSO_TCP_FLAG_FIRST_MSK 0x00000fff
/* inverted bitmask for bitfield lso_tcp_flag_first[b:0] */
#define HW_ATL2_THM_LSO_TCP_FLAG_FIRST_MSKN 0xfffff000
/* lower bit position of bitfield lso_tcp_flag_first[b:0] */
#define HW_ATL2_THM_LSO_TCP_FLAG_FIRST_SHIFT 0
/* width of bitfield lso_tcp_flag_first[b:0] */
#define HW_ATL2_THM_LSO_TCP_FLAG_FIRST_WIDTH 12
/* default value of bitfield lso_tcp_flag_first[b:0] */
#define HW_ATL2_THM_LSO_TCP_FLAG_FIRST_DEFAULT 0x0

/* tx lso_tcp_flag_last[b:0] bitfield definitions
 * preprocessor definitions for the bitfield "lso_tcp_flag_last[b:0]".
 * port="pif_thm_lso_tcp_flag_last_i[11:0]"
 */

/* register address for bitfield lso_tcp_flag_last[b:0] */
#define HW_ATL2_THM_LSO_TCP_FLAG_LAST_ADR 0x00007824
/* bitmask for bitfield lso_tcp_flag_last[b:0] */
#define HW_ATL2_THM_LSO_TCP_FLAG_LAST_MSK 0x00000fff
/* inverted bitmask for bitfield lso_tcp_flag_last[b:0] */
#define HW_ATL2_THM_LSO_TCP_FLAG_LAST_MSKN 0xfffff000
/* lower bit position of bitfield lso_tcp_flag_last[b:0] */
#define HW_ATL2_THM_LSO_TCP_FLAG_LAST_SHIFT 0
/* width of bitfield lso_tcp_flag_last[b:0] */
#define HW_ATL2_THM_LSO_TCP_FLAG_LAST_WIDTH 12
/* default value of bitfield lso_tcp_flag_last[b:0] */
#define HW_ATL2_THM_LSO_TCP_FLAG_LAST_DEFAULT 0x0

/* tx lso_tcp_flag_mid[b:0] bitfield definitions
 * preprocessor definitions for the bitfield "lso_tcp_flag_mid[b:0]".
 * port="pif_thm_lso_tcp_flag_mid_i[11:0]"
 */

/* Register address for bitfield lro_rsc_max[1F:0] */
#define HW_ATL2_RPO_LRO_RSC_MAX_ADR 0x00005598
/* Bitmask for bitfield lro_rsc_max[1F:0] */
#define HW_ATL2_RPO_LRO_RSC_MAX_MSK 0xFFFFFFFF
/* Inverted bitmask for bitfield lro_rsc_max[1F:0] */
#define HW_ATL2_RPO_LRO_RSC_MAX_MSKN 0x00000000
/* Lower bit position of bitfield lro_rsc_max[1F:0] */
#define HW_ATL2_RPO_LRO_RSC_MAX_SHIFT 0
/* Width of bitfield lro_rsc_max[1F:0] */
#define HW_ATL2_RPO_LRO_RSC_MAX_WIDTH 32
/* Default value of bitfield lro_rsc_max[1F:0] */
#define HW_ATL2_RPO_LRO_RSC_MAX_DEFAULT 0x0

/* RX lro_en[1F:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "lro_en[1F:0]".
 * PORT="pif_rpo_lro_en_i[31:0]"
 */

/* Register address for bitfield lro_en[1F:0] */
#define HW_ATL2_RPO_LRO_EN_ADR 0x00005590
/* Bitmask for bitfield lro_en[1F:0] */
#define HW_ATL2_RPO_LRO_EN_MSK 0xFFFFFFFF
/* Inverted bitmask for bitfield lro_en[1F:0] */
#define HW_ATL2_RPO_LRO_EN_MSKN 0x00000000
/* Lower bit position of bitfield lro_en[1F:0] */
#define HW_ATL2_RPO_LRO_EN_SHIFT 0
/* Width of bitfield lro_en[1F:0] */
#define HW_ATL2_RPO_LRO_EN_WIDTH 32
/* Default value of bitfield lro_en[1F:0] */
#define HW_ATL2_RPO_LRO_EN_DEFAULT 0x0

/* RX lro_ptopt_en Bitfield Definitions
 * Preprocessor definitions for the bitfield "lro_ptopt_en".
 * PORT="pif_rpo_lro_ptopt_en_i"
 */

/* Register address for bitfield lro_ptopt_en */
#define HW_ATL2_RPO_LRO_PTOPT_EN_ADR 0x00005594
/* Bitmask for bitfield lro_ptopt_en */
#define HW_ATL2_RPO_LRO_PTOPT_EN_MSK 0x00008000
/* Inverted bitmask for bitfield lro_ptopt_en */
#define HW_ATL2_RPO_LRO_PTOPT_EN_MSKN 0xFFFF7FFF
/* Lower bit position of bitfield lro_ptopt_en */
#define HW_ATL2_RPO_LRO_PTOPT_EN_SHIFT 15
/* Width of bitfield lro_ptopt_en */
#define HW_ATL2_RPO_LRO_PTOPT_EN_WIDTH 1
/* Default value of bitfield lro_ptopt_en */
#define HW_ATL2_RPO_LRO_PTOPT_EN_DEFALT 0x1

/* RX lro_q_ses_lmt Bitfield Definitions
 * Preprocessor definitions for the bitfield "lro_q_ses_lmt".
 * PORT="pif_rpo_lro_q_ses_lmt_i[1:0]"
 */

/* Register address for bitfield lro_q_ses_lmt */
#define HW_ATL2_RPO_LRO_QSES_LMT_ADR 0x00005594
/* Bitmask for bitfield lro_q_ses_lmt */
#define HW_ATL2_RPO_LRO_QSES_LMT_MSK 0x00003000
/* Inverted bitmask for bitfield lro_q_ses_lmt */
#define HW_ATL2_RPO_LRO_QSES_LMT_MSKN 0xFFFFCFFF
/* Lower bit position of bitfield lro_q_ses_lmt */
#define HW_ATL2_RPO_LRO_QSES_LMT_SHIFT 12
/* Width of bitfield lro_q_ses_lmt */
#define HW_ATL2_RPO_LRO_QSES_LMT_WIDTH 2
/* Default value of bitfield lro_q_ses_lmt */
#define HW_ATL2_RPO_LRO_QSES_LMT_DEFAULT 0x1

/* RX lro_tot_dsc_lmt[1:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "lro_tot_dsc_lmt[1:0]".
 * PORT="pif_rpo_lro_tot_dsc_lmt_i[1:0]"
 */

/* Register address for bitfield lro_tot_dsc_lmt[1:0] */
#define HW_ATL2_RPO_LRO_TOT_DSC_LMT_ADR 0x00005594
/* Bitmask for bitfield lro_tot_dsc_lmt[1:0] */
#define HW_ATL2_RPO_LRO_TOT_DSC_LMT_MSK 0x00000060
/* Inverted bitmask for bitfield lro_tot_dsc_lmt[1:0] */
#define HW_ATL2_RPO_LRO_TOT_DSC_LMT_MSKN 0xFFFFFF9F
/* Lower bit position of bitfield lro_tot_dsc_lmt[1:0] */
#define HW_ATL2_RPO_LRO_TOT_DSC_LMT_SHIFT 5
/* Width of bitfield lro_tot_dsc_lmt[1:0] */
#define HW_ATL2_RPO_LRO_TOT_DSC_LMT_WIDTH 2
/* Default value of bitfield lro_tot_dsc_lmt[1:0] */
#define HW_ATL2_RPO_LRO_TOT_DSC_LMT_DEFALT 0x1

/* RX lro_pkt_min[4:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "lro_pkt_min[4:0]".
 * PORT="pif_rpo_lro_pkt_min_i[4:0]"
 */

/* Register address for bitfield lro_pkt_min[4:0] */
#define HW_ATL2_RPO_LRO_PKT_MIN_ADR 0x00005594
/* Bitmask for bitfield lro_pkt_min[4:0] */
#define HW_ATL2_RPO_LRO_PKT_MIN_MSK 0x0000001F
/* Inverted bitmask for bitfield lro_pkt_min[4:0] */
#define HW_ATL2_RPO_LRO_PKT_MIN_MSKN 0xFFFFFFE0
/* Lower bit position of bitfield lro_pkt_min[4:0] */
#define HW_ATL2_RPO_LRO_PKT_MIN_SHIFT 0
/* Width of bitfield lro_pkt_min[4:0] */
#define HW_ATL2_RPO_LRO_PKT_MIN_WIDTH 5
/* Default value of bitfield lro_pkt_min[4:0] */
#define HW_ATL2_RPO_LRO_PKT_MIN_DEFAULT 0x8

/* Width of bitfield lro{L}_des_max[1:0] */
#define HW_ATL2_RPO_LRO_LDES_MAX_WIDTH 2
/* Default value of bitfield lro{L}_des_max[1:0] */
#define HW_ATL2_RPO_LRO_LDES_MAX_DEFAULT 0x0

/* RX lro_tb_div[11:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "lro_tb_div[11:0]".
 * PORT="pif_rpo_lro_tb_div_i[11:0]"
 */

/* Register address for bitfield lro_tb_div[11:0] */
#define HW_ATL2_RPO_LRO_TB_DIV_ADR 0x00005620
/* Bitmask for bitfield lro_tb_div[11:0] */
#define HW_ATL2_RPO_LRO_TB_DIV_MSK 0xFFF00000
/* Inverted bitmask for bitfield lro_tb_div[11:0] */
#define HW_ATL2_RPO_LRO_TB_DIV_MSKN 0x000FFFFF
/* Lower bit position of bitfield lro_tb_div[11:0] */
#define HW_ATL2_RPO_LRO_TB_DIV_SHIFT 20
/* Width of bitfield lro_tb_div[11:0] */
#define HW_ATL2_RPO_LRO_TB_DIV_WIDTH 12
/* Default value of bitfield lro_tb_div[11:0] */
#define HW_ATL2_RPO_LRO_TB_DIV_DEFAULT 0xC35

/* RX lro_ina_ival[9:0] Bitfield Definitions
 *   Preprocessor definitions for the bitfield "lro_ina_ival[9:0]".
 *   PORT="pif_rpo_lro_ina_ival_i[9:0]"
 */

/* Register address for bitfield lro_ina_ival[9:0] */
#define HW_ATL2_RPO_LRO_INA_IVAL_ADR 0x00005620
/* Bitmask for bitfield lro_ina_ival[9:0] */
#define HW_ATL2_RPO_LRO_INA_IVAL_MSK 0x000FFC00
/* Inverted bitmask for bitfield lro_ina_ival[9:0] */
#define HW_ATL2_RPO_LRO_INA_IVAL_MSKN 0xFFF003FF
/* Lower bit position of bitfield lro_ina_ival[9:0] */
#define HW_ATL2_RPO_LRO_INA_IVAL_SHIFT 10
/* Width of bitfield lro_ina_ival[9:0] */
#define HW_ATL2_RPO_LRO_INA_IVAL_WIDTH 10
/* Default value of bitfield lro_ina_ival[9:0] */
#define HW_ATL2_RPO_LRO_INA_IVAL_DEFAULT 0xA

/* RX lro_max_ival[9:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "lro_max_ival[9:0]".
 * PORT="pif_rpo_lro_max_ival_i[9:0]"
 */

/* Register address for bitfield lro_max_ival[9:0] */
#define HW_ATL2_RPO_LRO_MAX_IVAL_ADR 0x00005620
/* Bitmask for bitfield lro_max_ival[9:0] */
#define HW_ATL2_RPO_LRO_MAX_IVAL_MSK 0x000003FF
/* Inverted bitmask for bitfield lro_max_ival[9:0] */
#define HW_ATL2_RPO_LRO_MAX_IVAL_MSKN 0xFFFFFC00
/* Lower bit position of bitfield lro_max_ival[9:0] */
#define HW_ATL2_RPO_LRO_MAX_IVAL_SHIFT 0
/* Width of bitfield lro_max_ival[9:0] */
#define HW_ATL2_RPO_LRO_MAX_IVAL_WIDTH 10
/* Default value of bitfield lro_max_ival[9:0] */
#define HW_ATL2_RPO_LRO_MAX_IVAL_DEFAULT 0x19

/* TX dca{D}_cpuid[7:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "dca{D}_cpuid[7:0]".
 * Parameter: DCA {D} | stride size 0x4 | range [0, 31]
 * PORT="pif_tdm_dca0_cpuid_i[7:0]"
 */

/* Register address for bitfield dca{D}_cpuid[7:0] */
#define HW_ATL2_TDM_DCA_DCPUID_ADR(dca) (0x00008400 + (dca) * 0x4)
/* Bitmask for bitfield dca{D}_cpuid[7:0] */
#define HW_ATL2_TDM_DCA_DCPUID_MSK 0x000000FF
/* Inverted bitmask for bitfield dca{D}_cpuid[7:0] */
#define HW_ATL2_TDM_DCA_DCPUID_MSKN 0xFFFFFF00
/* Lower bit position of bitfield dca{D}_cpuid[7:0] */
#define HW_ATL2_TDM_DCA_DCPUID_SHIFT 0
/* Width of bitfield dca{D}_cpuid[7:0] */
#define HW_ATL2_TDM_DCA_DCPUID_WIDTH 8
/* Default value of bitfield dca{D}_cpuid[7:0] */
#define HW_ATL2_TDM_DCA_DCPUID_DEFAULT 0x0

/* TX dca{D}_desc_en Bitfield Definitions
 * Preprocessor definitions for the bitfield "dca{D}_desc_en".
 * Parameter: DCA {D} | stride size 0x4 | range [0, 31]
 * PORT="pif_tdm_dca_desc_en_i[0]"
 */

/* Register address for bitfield dca{D}_desc_en */
#define HW_ATL2_TDM_DCA_DDESC_EN_ADR(dca) (0x00008400 + (dca) * 0x4)
/* Bitmask for bitfield dca{D}_desc_en */
#define HW_ATL2_TDM_DCA_DDESC_EN_MSK 0x80000000
/* Inverted bitmask for bitfield dca{D}_desc_en */
#define HW_ATL2_TDM_DCA_DDESC_EN_MSKN 0x7FFFFFFF
/* Lower bit position of bitfield dca{D}_desc_en */
#define HW_ATL2_TDM_DCA_DDESC_EN_SHIFT 31
/* Width of bitfield dca{D}_desc_en */
#define HW_ATL2_TDM_DCA_DDESC_EN_WIDTH 1
/* Default value of bitfield dca{D}_desc_en */
#define HW_ATL2_TDM_DCA_DDESC_EN_DEFAULT 0x0

/* TX desc{D}_en Bitfield Definitions
 * Preprocessor definitions for the bitfield "desc{D}_en".
 * Parameter: descriptor {D} | stride size 0x40 | range [0, 31]
 * PORT="pif_tdm_desc_en_i[0]"
 */

/* Register address for bitfield desc{D}_en */
#define HW_ATL2_TDM_DESC_DEN_ADR(descriptor) (0x00007C08 + (descriptor) * 0x40)
/* Bitmask for bitfield desc{D}_en */
#define HW_ATL2_TDM_DESC_DEN_MSK 0x80000000
/* Inverted bitmask for bitfield desc{D}_en */
#define HW_ATL2_TDM_DESC_DEN_MSKN 0x7FFFFFFF
/* Lower bit position of bitfield desc{D}_en */
#define HW_ATL2_TDM_DESC_DEN_SHIFT 31
/* Width of bitfield desc{D}_en */
#define HW_ATL2_TDM_DESC_DEN_WIDTH 1
/* Default value of bitfield desc{D}_en */
#define HW_ATL2_TDM_DESC_DEN_DEFAULT 0x0

/* TX desc{D}_hd[C:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "desc{D}_hd[C:0]".
 * Parameter: descriptor {D} | stride size 0x40 | range [0, 31]
 * PORT="tdm_pif_desc0_hd_o[12:0]"
 */

/* Register address for bitfield desc{D}_hd[C:0] */
#define HW_ATL2_TDM_DESC_DHD_ADR(descriptor) (0x00007C0C + (descriptor) * 0x40)
/* Bitmask for bitfield desc{D}_hd[C:0] */
#define HW_ATL2_TDM_DESC_DHD_MSK 0x00001FFF
/* Inverted bitmask for bitfield desc{D}_hd[C:0] */
#define HW_ATL2_TDM_DESC_DHD_MSKN 0xFFFFE000
/* Lower bit position of bitfield desc{D}_hd[C:0] */
#define HW_ATL2_TDM_DESC_DHD_SHIFT 0
/* Width of bitfield desc{D}_hd[C:0] */
#define HW_ATL2_TDM_DESC_DHD_WIDTH 13

/* TX desc{D}_len[9:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "desc{D}_len[9:0]".
 * Parameter: descriptor {D} | stride size 0x40 | range [0, 31]
 * PORT="pif_tdm_desc0_len_i[9:0]"
 */

/* Register address for bitfield desc{D}_len[9:0] */
#define HW_ATL2_TDM_DESC_DLEN_ADR(descriptor) (0x00007C08 + (descriptor) * 0x40)
/* Bitmask for bitfield desc{D}_len[9:0] */
#define HW_ATL2_TDM_DESC_DLEN_MSK 0x00001FF8
/* Inverted bitmask for bitfield desc{D}_len[9:0] */
#define HW_ATL2_TDM_DESC_DLEN_MSKN 0xFFFFE007
/* Lower bit position of bitfield desc{D}_len[9:0] */
#define HW_ATL2_TDM_DESC_DLEN_SHIFT 3
/* Width of bitfield desc{D}_len[9:0] */
#define HW_ATL2_TDM_DESC_DLEN_WIDTH 10
/* Default value of bitfield desc{D}_len[9:0] */
#define HW_ATL2_TDM_DESC_DLEN_DEFAULT 0x0

/* TX desc{D}_wrb_thresh[6:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "desc{D}_wrb_thresh[6:0]".
 * Parameter: descriptor {D} | stride size 0x40 | range [0, 31]
 * PORT="pif_tdm_desc0_wrb_thresh_i[6:0]"
 */

/* Register address for bitfield desc{D}_wrb_thresh[6:0] */
#define HW_ATL2_TDM_DESC_DWRB_THRESH_ADR(descriptor) \
	(0x00007C18 + (descriptor) * 0x40)
/* Bitmask for bitfield desc{D}_wrb_thresh[6:0] */
#define HW_ATL2_TDM_DESC_DWRB_THRESH_MSK 0x00007F00
/* Inverted bitmask for bitfield desc{D}_wrb_thresh[6:0] */
#define HW_ATL2_TDM_DESC_DWRB_THRESH_MSKN 0xFFFF80FF
/* Lower bit position of bitfield desc{D}_wrb_thresh[6:0] */
#define HW_ATL2_TDM_DESC_DWRB_THRESH_SHIFT 8
/* Width of bitfield desc{D}_wrb_thresh[6:0] */
#define HW_ATL2_TDM_DESC_DWRB_THRESH_WIDTH 7
/* Default value of bitfield desc{D}_wrb_thresh[6:0] */
#define HW_ATL2_TDM_DESC_DWRB_THRESH_DEFAULT 0x0

/* TX tdm_int_mod_en Bitfield Definitions
 * Preprocessor definitions for the bitfield "tdm_int_mod_en".
 * PORT="pif_tdm_int_mod_en_i"
 */

/* Register address for bitfield tdm_int_mod_en */
#define HW_ATL2_TDM_INT_MOD_EN_ADR 0x00007B40
/* Bitmask for bitfield tdm_int_mod_en */
#define HW_ATL2_TDM_INT_MOD_EN_MSK 0x00000010
/* Inverted bitmask for bitfield tdm_int_mod_en */
#define HW_ATL2_TDM_INT_MOD_EN_MSKN 0xFFFFFFEF
/* Lower bit position of bitfield tdm_int_mod_en */
#define HW_ATL2_TDM_INT_MOD_EN_SHIFT 4
/* Width of bitfield tdm_int_mod_en */
#define HW_ATL2_TDM_INT_MOD_EN_WIDTH 1
/* Default value of bitfield tdm_int_mod_en */
#define HW_ATL2_TDM_INT_MOD_EN_DEFAULT 0x0

/* TX lso_tcp_flag_mid[B:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "lso_tcp_flag_mid[B:0]".
 * PORT="pif_thm_lso_tcp_flag_mid_i[11:0]"
 */
/* register address for bitfield lso_tcp_flag_mid[b:0] */
#define HW_ATL2_THM_LSO_TCP_FLAG_MID_ADR 0x00007820
/* bitmask for bitfield lso_tcp_flag_mid[b:0] */
#define HW_ATL2_THM_LSO_TCP_FLAG_MID_MSK 0x0fff0000
/* inverted bitmask for bitfield lso_tcp_flag_mid[b:0] */
#define HW_ATL2_THM_LSO_TCP_FLAG_MID_MSKN 0xf000ffff
/* lower bit position of bitfield lso_tcp_flag_mid[b:0] */
#define HW_ATL2_THM_LSO_TCP_FLAG_MID_SHIFT 16
/* width of bitfield lso_tcp_flag_mid[b:0] */
#define HW_ATL2_THM_LSO_TCP_FLAG_MID_WIDTH 12
/* default value of bitfield lso_tcp_flag_mid[b:0] */
#define HW_ATL2_THM_LSO_TCP_FLAG_MID_DEFAULT 0x0

/* tx tx_buf_en bitfield definitions
 * preprocessor definitions for the bitfield "tx_buf_en".
 * port="pif_tpb_tx_buf_en_i"
 */

/* register address for bitfield tx_buf_en */
#define HW_ATL2_TPB_TX_BUF_EN_ADR 0x00007900
/* bitmask for bitfield tx_buf_en */
#define HW_ATL2_TPB_TX_BUF_EN_MSK 0x00000001
/* inverted bitmask for bitfield tx_buf_en */
#define HW_ATL2_TPB_TX_BUF_EN_MSKN 0xfffffffe
/* lower bit position of bitfield tx_buf_en */
#define HW_ATL2_TPB_TX_BUF_EN_SHIFT 0
/* width of bitfield tx_buf_en */
#define HW_ATL2_TPB_TX_BUF_EN_WIDTH 1
/* default value of bitfield tx_buf_en */
#define HW_ATL2_TPB_TX_BUF_EN_DEFAULT 0x0

/* register address for bitfield tx_tc_mode */
#define HW_ATL2_TPB_TX_TC_MODE_ADDR 0x00007900
/* bitmask for bitfield tx_tc_mode */
#define HW_ATL2_TPB_TX_TC_MODE_MSK 0x00000100
/* inverted bitmask for bitfield tx_tc_mode */
#define HW_ATL2_TPB_TX_TC_MODE_MSKN 0xFFFFFEFF
/* lower bit position of bitfield tx_tc_mode */
#define HW_ATL2_TPB_TX_TC_MODE_SHIFT 8
/* width of bitfield tx_tc_mode */
#define HW_ATL2_TPB_TX_TC_MODE_WIDTH 1
/* default value of bitfield tx_tc_mode */
#define HW_ATL2_TPB_TX_TC_MODE_DEFAULT 0x0

/* register address for bitfield tx_tc_q_rand_map_en */
#define HW_ATL2_TPB_TX_FLEX_MODE_ADR 0x00007900
/* bitmask for bitfield tx_tc_q_rand_map_en */
#define HW_ATL2_TPB_TX_FLEX_MODE_MSK 0x00000020
/* inverted bitmask for bitfield tx_tc_q_rand_map_en */
#define HW_ATL2_TPB_TX_FLEX_MODE_MSKN 0xffffffdf
/* lower bit position of bitfield tx_tc_q_rand_map_en */
#define HW_ATL2_TPB_TX_FLEX_MODE_SHIFT 9
/* width of bitfield tx_tc_q_rand_map_en */
#define HW_ATL2_TPB_TX_FLEX_MODE_WIDTH 1
/* default value of bitfield tx_tc_q_rand_map_en */
#define HW_ATL2_TPB_TX_FLEX_MODE_DEFAULT 0x0

/* tx tx{b}_hi_thresh[c:0] bitfield definitions
 * preprocessor definitions for the bitfield "tx{b}_hi_thresh[c:0]".
 * parameter: buffer {b} | stride size 0x10 | range [0, 7]
 * port="pif_tpb_tx0_hi_thresh_i[12:0]"
 */

/* register address for bitfield tx{b}_hi_thresh[c:0] */
#define HW_ATL2_TPB_TXBHI_THRESH_ADR(buffer) (0x00007914 + (buffer) * 0x10)
/* bitmask for bitfield tx{b}_hi_thresh[c:0] */
#define HW_ATL2_TPB_TXBHI_THRESH_MSK 0x1fff0000
/* inverted bitmask for bitfield tx{b}_hi_thresh[c:0] */
#define HW_ATL2_TPB_TXBHI_THRESH_MSKN 0xe000ffff
/* lower bit position of bitfield tx{b}_hi_thresh[c:0] */
#define HW_ATL2_TPB_TXBHI_THRESH_SHIFT 16
/* width of bitfield tx{b}_hi_thresh[c:0] */
#define HW_ATL2_TPB_TXBHI_THRESH_WIDTH 13
/* default value of bitfield tx{b}_hi_thresh[c:0] */
#define HW_ATL2_TPB_TXBHI_THRESH_DEFAULT 0x0

/* tx tx{b}_lo_thresh[c:0] bitfield definitions
 * preprocessor definitions for the bitfield "tx{b}_lo_thresh[c:0]".
 * parameter: buffer {b} | stride size 0x10 | range [0, 7]
 * port="pif_tpb_tx0_lo_thresh_i[12:0]"
 */

/* register address for bitfield tx{b}_lo_thresh[c:0] */
#define HW_ATL2_TPB_TXBLO_THRESH_ADR(buffer) (0x00007914 + (buffer) * 0x10)
/* bitmask for bitfield tx{b}_lo_thresh[c:0] */
#define HW_ATL2_TPB_TXBLO_THRESH_MSK 0x00001fff
/* inverted bitmask for bitfield tx{b}_lo_thresh[c:0] */
#define HW_ATL2_TPB_TXBLO_THRESH_MSKN 0xffffe000
/* lower bit position of bitfield tx{b}_lo_thresh[c:0] */
#define HW_ATL2_TPB_TXBLO_THRESH_SHIFT 0
/* width of bitfield tx{b}_lo_thresh[c:0] */
#define HW_ATL2_TPB_TXBLO_THRESH_WIDTH 13
/* default value of bitfield tx{b}_lo_thresh[c:0] */
#define HW_ATL2_TPB_TXBLO_THRESH_DEFAULT 0x0

/* register address for bitfield pif_tpb_tx_q_tc_map[2:0] */
#define HW_ATL2_TPB_FLEX_MAP_ADR(q) (0x0000799c + (q >> 2) * 0x4)
/* bitmask for bitfield tx{b}_lo_thresh[2:0] */
#define HW_ATL2_TPB_FLEX_MAP_MSK(q) (0x01f << ((q & 3)*8))
/* inverted bitmask for bitfield tx{b}_lo_thresh[2:0] */
#define HW_ATL2_TPB_FLEX_MAP_MSKN (~HW_ATL2_TPB_FLEX_MAP_MSK(q))
/* lower bit position of bitfield tx{b}_lo_thresh[2c:0] */
#define HW_ATL2_TPB_FLEX_MAP_SHIFT(q) ((q & 3)*8)
/* width of bitfield tx{b}_lo_thresh[2:0] */
#define HW_ATL2_TPB_FLEX_MAP_WIDTH 5
/* default value of bitfield tx{b}_lo_thresh[2:0] */
#define HW_ATL2_TPB_FLEX_MAP_DEFAULT 0x0


/* tx dma_sys_loopback bitfield definitions
 * preprocessor definitions for the bitfield "dma_sys_loopback".
 * port="pif_tpb_dma_sys_lbk_i"
 */

/* register address for bitfield dma_sys_loopback */
#define HW_ATL2_TPB_DMA_SYS_LBK_ADR 0x00007000
/* bitmask for bitfield dma_sys_loopback */
#define HW_ATL2_TPB_DMA_SYS_LBK_MSK 0x00000040
/* inverted bitmask for bitfield dma_sys_loopback */
#define HW_ATL2_TPB_DMA_SYS_LBK_MSKN 0xffffffbf
/* lower bit position of bitfield dma_sys_loopback */
#define HW_ATL2_TPB_DMA_SYS_LBK_SHIFT 6
/* width of bitfield dma_sys_loopback */
#define HW_ATL2_TPB_DMA_SYS_LBK_WIDTH 1
/* default value of bitfield dma_sys_loopback */
#define HW_ATL2_TPB_DMA_SYS_LBK_DEFAULT 0x0

/* tx dma_net_loopback bitfield definitions
 * preprocessor definitions for the bitfield "dma_net_loopback".
 * port="pif_tpb_dma_net_lbk_i"
 */

/* register address for bitfield dma_net_loopback */
#define HW_ATL2_TPB_DMA_NET_LBK_ADR 0x00007000
/* bitmask for bitfield dma_net_loopback */
#define HW_ATL2_TPB_DMA_NET_LBK_MSK 0x00000010
/* inverted bitmask for bitfield dma_net_loopback */
#define HW_ATL2_TPB_DMA_NET_LBK_MSKN 0xffffffef
/* lower bit position of bitfield dma_net_loopback */
#define HW_ATL2_TPB_DMA_NET_LBK_SHIFT 4
/* width of bitfield dma_net_loopback */
#define HW_ATL2_TPB_DMA_NET_LBK_WIDTH 1
/* default value of bitfield dma_net_loopback */
#define HW_ATL2_TPB_DMA_NET_LBK_DEFAULT 0x0

/* tx tx{b}_buf_size[7:0] bitfield definitions
 * preprocessor definitions for the bitfield "tx{b}_buf_size[7:0]".
 * parameter: buffer {b} | stride size 0x10 | range [0, 7]
 * port="pif_tpb_tx0_buf_size_i[7:0]"
 */

/* register address for bitfield tx{b}_buf_size[7:0] */
#define HW_ATL2_TPB_TXBBUF_SIZE_ADR(buffer) (0x00007910 + (buffer) * 0x10)
/* bitmask for bitfield tx{b}_buf_size[7:0] */
#define HW_ATL2_TPB_TXBBUF_SIZE_MSK 0x000000ff
/* inverted bitmask for bitfield tx{b}_buf_size[7:0] */
#define HW_ATL2_TPB_TXBBUF_SIZE_MSKN 0xffffff00
/* lower bit position of bitfield tx{b}_buf_size[7:0] */
#define HW_ATL2_TPB_TXBBUF_SIZE_SHIFT 0
/* width of bitfield tx{b}_buf_size[7:0] */
#define HW_ATL2_TPB_TXBBUF_SIZE_WIDTH 8
/* default value of bitfield tx{b}_buf_size[7:0] */
#define HW_ATL2_TPB_TXBBUF_SIZE_DEFAULT 0x0

/* tx tx_scp_ins_en bitfield definitions
 * preprocessor definitions for the bitfield "tx_scp_ins_en".
 * port="pif_tpb_scp_ins_en_i"
 */

/* register address for bitfield tx_scp_ins_en */
#define HW_ATL2_TPB_TX_SCP_INS_EN_ADR 0x00007900
/* bitmask for bitfield tx_scp_ins_en */
#define HW_ATL2_TPB_TX_SCP_INS_EN_MSK 0x00000004
/* inverted bitmask for bitfield tx_scp_ins_en */
#define HW_ATL2_TPB_TX_SCP_INS_EN_MSKN 0xfffffffb
/* lower bit position of bitfield tx_scp_ins_en */
#define HW_ATL2_TPB_TX_SCP_INS_EN_SHIFT 2
/* width of bitfield tx_scp_ins_en */
#define HW_ATL2_TPB_TX_SCP_INS_EN_WIDTH 1
/* default value of bitfield tx_scp_ins_en */
#define HW_ATL2_TPB_TX_SCP_INS_EN_DEFAULT 0x0

/* tx tx_buffer_clk_gate_en bitfield definitions
 * preprocessor definitions for the bitfield "tx_buffer_clk_gate_en".
 * port="pif_tpb_tx_buffer_clk_gate_en_i"
 */

/* register address for bitfield tx_buffer_clk_gate_en */
#define HW_ATL2_TPB_TX_BUF_CLK_GATE_EN_ADR 0x00007900
/* bitmask for bitfield tx_buffer_clk_gate_en */
#define HW_ATL2_TPB_TX_BUF_CLK_GATE_EN_MSK 0x00000020
/* inverted bitmask for bitfield tx_buffer_clk_gate_en */
#define HW_ATL2_TPB_TX_BUF_CLK_GATE_EN_MSKN 0xffffffdf
/* lower bit position of bitfield tx_buffer_clk_gate_en */
#define HW_ATL2_TPB_TX_BUF_CLK_GATE_EN_SHIFT 5
/* width of bitfield tx_buffer_clk_gate_en */
#define HW_ATL2_TPB_TX_BUF_CLK_GATE_EN_WIDTH 1
/* default value of bitfield tx_buffer_clk_gate_en */
#define HW_ATL2_TPB_TX_BUF_CLK_GATE_EN_DEFAULT 0x0

/* tx ipv4_chk_en bitfield definitions
 * preprocessor definitions for the bitfield "ipv4_chk_en".
 * port="pif_tpo_ipv4_chk_en_i"
 */

/* register address for bitfield ipv4_chk_en */
#define HW_ATL2_TPO_IPV4CHK_EN_ADR 0x00007800
/* bitmask for bitfield ipv4_chk_en */
#define HW_ATL2_TPO_IPV4CHK_EN_MSK 0x00000002
/* inverted bitmask for bitfield ipv4_chk_en */
#define HW_ATL2_TPO_IPV4CHK_EN_MSKN 0xfffffffd
/* lower bit position of bitfield ipv4_chk_en */
#define HW_ATL2_TPO_IPV4CHK_EN_SHIFT 1
/* width of bitfield ipv4_chk_en */
#define HW_ATL2_TPO_IPV4CHK_EN_WIDTH 1
/* default value of bitfield ipv4_chk_en */
#define HW_ATL2_TPO_IPV4CHK_EN_DEFAULT 0x0

/* tx l4_chk_en bitfield definitions
 * preprocessor definitions for the bitfield "l4_chk_en".
 * port="pif_tpo_l4_chk_en_i"
 */

/* register address for bitfield l4_chk_en */
#define HW_ATL2_TPOL4CHK_EN_ADR 0x00007800
/* bitmask for bitfield l4_chk_en */
#define HW_ATL2_TPOL4CHK_EN_MSK 0x00000001
/* inverted bitmask for bitfield l4_chk_en */
#define HW_ATL2_TPOL4CHK_EN_MSKN 0xfffffffe
/* lower bit position of bitfield l4_chk_en */
#define HW_ATL2_TPOL4CHK_EN_SHIFT 0
/* width of bitfield l4_chk_en */
#define HW_ATL2_TPOL4CHK_EN_WIDTH 1
/* default value of bitfield l4_chk_en */
#define HW_ATL2_TPOL4CHK_EN_DEFAULT 0x0

/* tx pkt_sys_loopback bitfield definitions
 * preprocessor definitions for the bitfield "pkt_sys_loopback".
 * port="pif_tpo_pkt_sys_lbk_i"
 */

/* register address for bitfield pkt_sys_loopback */
#define HW_ATL2_TPO_PKT_SYS_LBK_ADR 0x00007000
/* bitmask for bitfield pkt_sys_loopback */
#define HW_ATL2_TPO_PKT_SYS_LBK_MSK 0x00000080
/* inverted bitmask for bitfield pkt_sys_loopback */
#define HW_ATL2_TPO_PKT_SYS_LBK_MSKN 0xffffff7f
/* lower bit position of bitfield pkt_sys_loopback */
#define HW_ATL2_TPO_PKT_SYS_LBK_SHIFT 7
/* width of bitfield pkt_sys_loopback */
#define HW_ATL2_TPO_PKT_SYS_LBK_WIDTH 1
/* default value of bitfield pkt_sys_loopback */
#define HW_ATL2_TPO_PKT_SYS_LBK_DEFAULT 0x0

/* tx data_tc_arb_mode bitfield definitions
 * preprocessor definitions for the bitfield "data_tc_arb_mode".
 * port="pif_tps_data_tc_arb_mode_i"
 */

/* register address for bitfield data_tc_arb_mode */
#define HW_ATL2_TPS_DATA_TC_ARB_MODE_ADR 0x00007100
/* bitmask for bitfield data_tc_arb_mode */
#define HW_ATL2_TPS_DATA_TC_ARB_MODE_MSK 0x00000001
/* inverted bitmask for bitfield data_tc_arb_mode */
#define HW_ATL2_TPS_DATA_TC_ARB_MODE_MSKN 0xfffffffe
/* lower bit position of bitfield data_tc_arb_mode */
#define HW_ATL2_TPS_DATA_TC_ARB_MODE_SHIFT 0
/* width of bitfield data_tc_arb_mode */
#define HW_ATL2_TPS_DATA_TC_ARB_MODE_WIDTH 1
/* default value of bitfield data_tc_arb_mode */
#define HW_ATL2_TPS_DATA_TC_ARB_MODE_DEFAULT 0x0

/* register address for bitfield tps_tc_pri_ctl */
#define HW_ATL2_TPS_TC_PRIO_CTRL_ADR 0x0000710C
/* bitmask for bitfield data_tc_arb_mode */
#define HW_ATL2_TPS_TC_PRIO_CTRL_MSK(tc) (1 << (tc))
/* inverted bitmask for bitfield tps_tc_pri_ctl */
#define HW_ATL2_TPS_TC_PRIO_CTRL_MSKN(tc) (~(1 << (tc)))
/* lower bit position of bitfield tps_tc_pri_ctl */
#define HW_ATL2_TPS_TC_PRIO_CTRL_SHIFT(tc) (tc)
/* width of bitfield tps_tc_pri_ctl */
#define HW_ATL2_TPS_TC_PRIO_CTRL_WIDTH 1
/* default value of bitfield tps_tc_pri_ctl */
#define HW_ATL2_TPS_TC_PRIO_CTRL_DEFAULT 0x0

/*! @name TX pif_tpb_highest_prio_tc_en Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pif_tpb_highest_prio_tc_en".
*
*   Type: R/W
*
*   Notes: Enable highest priority TC
If set, the configured highest priority TC will be scheduled with highest priority.
*
*   PORT="pif_tpb_highest_prio_tc_en_i"
@{ */
/*! \brief Register address for bitfield pif_tpb_highest_prio_tc_en */
#define HW_ATL2_TPB_HIGHEST_PRIO_TC_EN_ADR 0x00007180
/*! \brief Bitmask for bitfield pif_tpb_highest_prio_tc_en */
#define HW_ATL2_TPB_HIGHEST_PRIO_TC_EN_MSK 0x00000100
/*! \brief Inverted bitmask for bitfield pif_tpb_highest_prio_tc_en */
#define HW_ATL2_TPB_HIGHEST_PRIO_TC_EN_MSKN 0xFFFFFEFF
/*! \brief Lower bit position of bitfield pif_tpb_highest_prio_tc_en */
#define HW_ATL2_TPB_HIGHEST_PRIO_TC_EN_SHIFT 8
/*! \brief Width of bitfield pif_tpb_highest_prio_tc_en */
#define HW_ATL2_TPB_HIGHEST_PRIO_TC_EN_WIDTH 1
/*! \brief Default value of bitfield pif_tpb_highest_prio_tc_en */
#define HW_ATL2_TPB_HIGHEST_PRIO_TC_EN_DEFAULT 0x0
/*@}*/

/*! @name TX pif_tpb_highest_prio_tc Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pif_tpb_highest_prio_tc".
*
*   Type: R/W
*
*   Notes: Configure a single TC in TPB that has highest priority. If enabled, this TC will be scheduled with higher priority than any AVB or mng-inj traffic.
*
*   PORT="pif_tpb_highest_prio_tc_i[2:0]"
@{ */
/*! \brief Register address for bitfield pif_tpb_highest_prio_tc */
#define HW_ATL2_TPB_HIGHEST_PRIO_TC_ADR 0x00007180
/*! \brief Bitmask for bitfield pif_tpb_highest_prio_tc */
#define HW_ATL2_TPB_HIGHEST_PRIO_TC_MSK 0x00000007
/*! \brief Inverted bitmask for bitfield pif_tpb_highest_prio_tc */
#define HW_ATL2_TPB_HIGHEST_PRIO_TC_MSKN 0xFFFFFFF8
/*! \brief Lower bit position of bitfield pif_tpb_highest_prio_tc */
#define HW_ATL2_TPB_HIGHEST_PRIO_TC_SHIFT 0
/*! \brief Width of bitfield pif_tpb_highest_prio_tc */
#define HW_ATL2_TPB_HIGHEST_PRIO_TC_WIDTH 3
/*! \brief Default value of bitfield pif_tpb_highest_prio_tc */
#define HW_ATL2_TPB_HIGHEST_PRIO_TC_DEFAULT 0x0
/*@}*/


/*! @name TX pif_tpb_avb_pkt_len_comp_en Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pif_tpb_avb_pkt_len_comp_en".
*
*   Type: R/W
*
*   Notes: If set, lower priority AVB packets will be hold back to not delay launch time of High Priority level AVB Packets.
AVB level 0 > AVB level 1 > normal AVB packets
Recommended to use in combination with High Priority AVB TCs.
Won't have any effect if no High Priority TC is configured.
*
*   PORT="pif_tpb_avb_pkt_len_comp_en_i"
@{ */
/*! \brief Register address for bitfield pif_tpb_avb_pkt_len_comp_en */
#define HW_ATL2_TPB_AVB_PKT_LEN_COMP_EN_ADR 0x00007184
/*! \brief Bitmask for bitfield pif_tpb_avb_pkt_len_comp_en */
#define HW_ATL2_TPB_AVB_PKT_LEN_COMP_EN_MSK 0x00010000
/*! \brief Inverted bitmask for bitfield pif_tpb_avb_pkt_len_comp_en */
#define HW_ATL2_TPB_AVB_PKT_LEN_COMP_EN_MSKN 0xFFFEFFFF
/*! \brief Lower bit position of bitfield pif_tpb_avb_pkt_len_comp_en */
#define HW_ATL2_TPB_AVB_PKT_LEN_COMP_EN_SHIFT 16
/*! \brief Width of bitfield pif_tpb_avb_pkt_len_comp_en */
#define HW_ATL2_TPB_AVB_PKT_LEN_COMP_EN_WIDTH 1
/*! \brief Default value of bitfield pif_tpb_avb_pkt_len_comp_en */
#define HW_ATL2_TPB_AVB_PKT_LEN_COMP_EN_DEFAULT 0x0
/*@}*/

/*! @name TX pif_tpb_high_prio_avb_en1 Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pif_tpb_high_prio_avb_en1".
*
*   Type: R/W
*
*   Notes: Enable High Priority AVB TC with priority level 1.
Must be set to use High Priority 1 AVB TC
*
*   PORT="pif_tpb_high_prio_avb_en_i[1]"
@{ */
/*! \brief Register address for bitfield pif_tpb_high_prio_avb_en1 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_EN1_ADR 0x00007184
/*! \brief Bitmask for bitfield pif_tpb_high_prio_avb_en1 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_EN1_MSK 0x00001000
/*! \brief Inverted bitmask for bitfield pif_tpb_high_prio_avb_en1 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_EN1_MSKN 0xFFFFEFFF
/*! \brief Lower bit position of bitfield pif_tpb_high_prio_avb_en1 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_EN1_SHIFT 12
/*! \brief Width of bitfield pif_tpb_high_prio_avb_en1 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_EN1_WIDTH 1
/*! \brief Default value of bitfield pif_tpb_high_prio_avb_en1 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_EN1_DEFAULT 0x0
/*@}*/

/*! @name TX pif_tpb_high_prio_avb_tc1 Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pif_tpb_high_prio_avb_tc1".
*
*   Type: R/W
*
*   Notes: Configure High Priority AVB TC with priority level 1.
AVB packets of this TC will be prioritized over all other AVB TCs - except priority 0 AVB.
*
*   PORT="pif_tpb_high_prio_avb_tc1_i[2:0]"
@{ */
/*! \brief Register address for bitfield pif_tpb_high_prio_avb_tc1 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_TC1_ADR 0x00007184
/*! \brief Bitmask for bitfield pif_tpb_high_prio_avb_tc1 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_TC1_MSK 0x00000700
/*! \brief Inverted bitmask for bitfield pif_tpb_high_prio_avb_tc1 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_TC1_MSKN 0xFFFFF8FF
/*! \brief Lower bit position of bitfield pif_tpb_high_prio_avb_tc1 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_TC1_SHIFT 8
/*! \brief Width of bitfield pif_tpb_high_prio_avb_tc1 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_TC1_WIDTH 3
/*! \brief Default value of bitfield pif_tpb_high_prio_avb_tc1 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_TC1_DEFAULT 0x3
/*@}*/

/*! @name TX pif_tpb_high_prio_avb_en0 Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pif_tpb_high_prio_avb_en0".
*
*   Type: R/W
*
*   Notes: Enable High Priority AVB TC with priority level 0.
Must be set to use High Priority 0 AVB TC
*
*   PORT="pif_tpb_high_prio_avb_en_i[0]"
@{ */
/*! \brief Register address for bitfield pif_tpb_high_prio_avb_en0 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_EN0_ADR 0x00007184
/*! \brief Bitmask for bitfield pif_tpb_high_prio_avb_en0 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_EN0_MSK 0x00000010
/*! \brief Inverted bitmask for bitfield pif_tpb_high_prio_avb_en0 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_EN0_MSKN 0xFFFFFFEF
/*! \brief Lower bit position of bitfield pif_tpb_high_prio_avb_en0 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_EN0_SHIFT 4
/*! \brief Width of bitfield pif_tpb_high_prio_avb_en0 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_EN0_WIDTH 1
/*! \brief Default value of bitfield pif_tpb_high_prio_avb_en0 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_EN0_DEFAULT 0x0
/*@}*/

/*! @name TX pif_tpb_high_prio_avb_tc0 Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pif_tpb_high_prio_avb_tc0".
*
*   Type: R/W
*
*   Notes: Configure High Priority AVB TC with priority level 0.
AVB packets of this TC will be prioritized over all other AVB TCs.
*
*   PORT="pif_tpb_high_prio_avb_tc0_i[2:0]"
@{ */
/*! \brief Register address for bitfield pif_tpb_high_prio_avb_tc0 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_TC0_ADR 0x00007184
/*! \brief Bitmask for bitfield pif_tpb_high_prio_avb_tc0 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_TC0_MSK 0x00000007
/*! \brief Inverted bitmask for bitfield pif_tpb_high_prio_avb_tc0 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_TC0_MSKN 0xFFFFFFF8
/*! \brief Lower bit position of bitfield pif_tpb_high_prio_avb_tc0 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_TC0_SHIFT 0
/*! \brief Width of bitfield pif_tpb_high_prio_avb_tc0 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_TC0_WIDTH 3
/*! \brief Default value of bitfield pif_tpb_high_prio_avb_tc0 */
#define HW_ATL2_TPB_HIGH_PRIO_AVB_TC0_DEFAULT 0x1
/*@}*/


/*! @name TX pif_tpb_avb_schdl_en Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pif_tpb_avb_schdl_en".
*
*   Type: R/W
*
*   Notes: Enable AVB and launchtime based scheduling per TC in TPB.
If disabled, TC with launchtime packets will be flushed (scheduled according to configured algorithm)
*
*   PORT="pif_tpb_avb_schdl_en_i[7:0]"
@{ */
/*! \brief Register address for bitfield pif_tpb_avb_schdl_en */
#define HW_ATL2_TPB_AVB_SCHDL_EN_ADR 0x00007188
/*! \brief Bitmask for bitfield pif_tpb_avb_schdl_en */
#define HW_ATL2_TPB_AVB_SCHDL_EN_MSK 0x000000FF
/*! \brief Inverted bitmask for bitfield pif_tpb_avb_schdl_en */
#define HW_ATL2_TPB_AVB_SCHDL_EN_MSKN 0xFFFFFF00
/*! \brief Lower bit position of bitfield pif_tpb_avb_schdl_en */
#define HW_ATL2_TPB_AVB_SCHDL_EN_SHIFT 0
/*! \brief Width of bitfield pif_tpb_avb_schdl_en */
#define HW_ATL2_TPB_AVB_SCHDL_EN_WIDTH 8
/*! \brief Default value of bitfield pif_tpb_avb_schdl_en */
#define HW_ATL2_TPB_AVB_SCHDL_EN_DEFAULT 0xFF
/*@}*/



/* tx desc_rate_ta_rst bitfield definitions
 * preprocessor definitions for the bitfield "desc_rate_ta_rst".
 * port="pif_tps_desc_rate_ta_rst_i"
 */

/* register address for bitfield desc_rate_ta_rst */
#define HW_ATL2_TPS_DESC_RATE_TA_RST_ADR 0x00007310
/* bitmask for bitfield desc_rate_ta_rst */
#define HW_ATL2_TPS_DESC_RATE_TA_RST_MSK 0x80000000
/* inverted bitmask for bitfield desc_rate_ta_rst */
#define HW_ATL2_TPS_DESC_RATE_TA_RST_MSKN 0x7fffffff
/* lower bit position of bitfield desc_rate_ta_rst */
#define HW_ATL2_TPS_DESC_RATE_TA_RST_SHIFT 31
/* width of bitfield desc_rate_ta_rst */
#define HW_ATL2_TPS_DESC_RATE_TA_RST_WIDTH 1
/* default value of bitfield desc_rate_ta_rst */
#define HW_ATL2_TPS_DESC_RATE_TA_RST_DEFAULT 0x0

/* tx desc_rate_limit[a:0] bitfield definitions
 * preprocessor definitions for the bitfield "desc_rate_limit[a:0]".
 * port="pif_tps_desc_rate_lim_i[10:0]"
 */

/* register address for bitfield desc_rate_limit[a:0] */
#define HW_ATL2_TPS_DESC_RATE_LIM_ADR 0x00007310
/* bitmask for bitfield desc_rate_limit[a:0] */
#define HW_ATL2_TPS_DESC_RATE_LIM_MSK 0x000007ff
/* inverted bitmask for bitfield desc_rate_limit[a:0] */
#define HW_ATL2_TPS_DESC_RATE_LIM_MSKN 0xfffff800
/* lower bit position of bitfield desc_rate_limit[a:0] */
#define HW_ATL2_TPS_DESC_RATE_LIM_SHIFT 0
/* width of bitfield desc_rate_limit[a:0] */
#define HW_ATL2_TPS_DESC_RATE_LIM_WIDTH 11
/* default value of bitfield desc_rate_limit[a:0] */
#define HW_ATL2_TPS_DESC_RATE_LIM_DEFAULT 0x0

/* tx desc_tc_arb_mode[1:0] bitfield definitions
 * preprocessor definitions for the bitfield "desc_tc_arb_mode[1:0]".
 * port="pif_tps_desc_tc_arb_mode_i[1:0]"
 */

/* register address for bitfield desc_tc_arb_mode[1:0] */
#define HW_ATL2_TPS_DESC_TC_ARB_MODE_ADR 0x00007200
/* bitmask for bitfield desc_tc_arb_mode[1:0] */
#define HW_ATL2_TPS_DESC_TC_ARB_MODE_MSK 0x00000003
/* inverted bitmask for bitfield desc_tc_arb_mode[1:0] */
#define HW_ATL2_TPS_DESC_TC_ARB_MODE_MSKN 0xfffffffc
/* lower bit position of bitfield desc_tc_arb_mode[1:0] */
#define HW_ATL2_TPS_DESC_TC_ARB_MODE_SHIFT 0
/* width of bitfield desc_tc_arb_mode[1:0] */
#define HW_ATL2_TPS_DESC_TC_ARB_MODE_WIDTH 2
/* default value of bitfield desc_tc_arb_mode[1:0] */
#define HW_ATL2_TPS_DESC_TC_ARB_MODE_DEFAULT 0x0

/* tx desc_tc{t}_credit_max[b:0] bitfield definitions
 * preprocessor definitions for the bitfield "desc_tc{t}_credit_max[b:0]".
 * parameter: tc {t} | stride size 0x4 | range [0, 7]
 * port="pif_tps_desc_tc0_credit_max_i[11:0]"
 */

/* register address for bitfield desc_tc{t}_credit_max[b:0] */
#define HW_ATL2_TPS_DESC_TCTCREDIT_MAX_ADR(tc) (0x00007210 + (tc) * 0x4)
/* bitmask for bitfield desc_tc{t}_credit_max[b:0] */
#define HW_ATL2_TPS_DESC_TCTCREDIT_MAX_MSK 0x0fff0000
/* inverted bitmask for bitfield desc_tc{t}_credit_max[b:0] */
#define HW_ATL2_TPS_DESC_TCTCREDIT_MAX_MSKN 0xf000ffff
/* lower bit position of bitfield desc_tc{t}_credit_max[b:0] */
#define HW_ATL2_TPS_DESC_TCTCREDIT_MAX_SHIFT 16
/* width of bitfield desc_tc{t}_credit_max[b:0] */
#define HW_ATL2_TPS_DESC_TCTCREDIT_MAX_WIDTH 12
/* default value of bitfield desc_tc{t}_credit_max[b:0] */
#define HW_ATL2_TPS_DESC_TCTCREDIT_MAX_DEFAULT 0x0

/* tx desc_tc{t}_weight[8:0] bitfield definitions
 * preprocessor definitions for the bitfield "desc_tc{t}_weight[8:0]".
 * parameter: tc {t} | stride size 0x4 | range [0, 7]
 * port="pif_tps_desc_tc0_weight_i[8:0]"
 */

/* register address for bitfield desc_tc{t}_weight[8:0] */
#define HW_ATL2_TPS_DESC_TCTWEIGHT_ADR(tc) (0x00007210 + (tc) * 0x4)
/* bitmask for bitfield desc_tc{t}_weight[8:0] */
#define HW_ATL2_TPS_DESC_TCTWEIGHT_MSK 0x000001ff
/* inverted bitmask for bitfield desc_tc{t}_weight[8:0] */
#define HW_ATL2_TPS_DESC_TCTWEIGHT_MSKN 0xfffffe00
/* lower bit position of bitfield desc_tc{t}_weight[8:0] */
#define HW_ATL2_TPS_DESC_TCTWEIGHT_SHIFT 0
/* width of bitfield desc_tc{t}_weight[8:0] */
#define HW_ATL2_TPS_DESC_TCTWEIGHT_WIDTH 9
/* default value of bitfield desc_tc{t}_weight[8:0] */
#define HW_ATL2_TPS_DESC_TCTWEIGHT_DEFAULT 0x0

/* tx desc_vm_arb_mode bitfield definitions
 * preprocessor definitions for the bitfield "desc_vm_arb_mode".
 * port="pif_tps_desc_vm_arb_mode_i"
 */

/* register address for bitfield desc_vm_arb_mode */
#define HW_ATL2_TPS_DESC_VM_ARB_MODE_ADR 0x00007300
/* bitmask for bitfield desc_vm_arb_mode */
#define HW_ATL2_TPS_DESC_VM_ARB_MODE_MSK 0x00000001
/* inverted bitmask for bitfield desc_vm_arb_mode */
#define HW_ATL2_TPS_DESC_VM_ARB_MODE_MSKN 0xfffffffe
/* lower bit position of bitfield desc_vm_arb_mode */
#define HW_ATL2_TPS_DESC_VM_ARB_MODE_SHIFT 0
/* width of bitfield desc_vm_arb_mode */
#define HW_ATL2_TPS_DESC_VM_ARB_MODE_WIDTH 1
/* default value of bitfield desc_vm_arb_mode */
#define HW_ATL2_TPS_DESC_VM_ARB_MODE_DEFAULT 0x0

/* tx data_tc{t}_credit_max[b:0] bitfield definitions
 * preprocessor definitions for the bitfield "data_tc{t}_credit_max[b:0]".
 * parameter: tc {t} | stride size 0x4 | range [0, 7]
 * port="pif_tps_data_tc0_credit_max_i[11:0]"
 */

/* register address for bitfield data_tc{t}_credit_max[b:0] */
#define HW_ATL2_TPS_DATA_TCTCREDIT_MAX_ADR(tc) (0x00007110 + (tc) * 0x4)
/* bitmask for bitfield data_tc{t}_credit_max[b:0] */
#define HW_ATL2_TPS_DATA_TCTCREDIT_MAX_MSK 0x0fff0000
/* inverted bitmask for bitfield data_tc{t}_credit_max[b:0] */
#define HW_ATL2_TPS_DATA_TCTCREDIT_MAX_MSKN 0xf000ffff
/* lower bit position of bitfield data_tc{t}_credit_max[b:0] */
#define HW_ATL2_TPS_DATA_TCTCREDIT_MAX_SHIFT 16
/* width of bitfield data_tc{t}_credit_max[b:0] */
#define HW_ATL2_TPS_DATA_TCTCREDIT_MAX_WIDTH 12
/* default value of bitfield data_tc{t}_credit_max[b:0] */
#define HW_ATL2_TPS_DATA_TCTCREDIT_MAX_DEFAULT 0x0

/* tx data_tc{t}_weight[8:0] bitfield definitions
 * preprocessor definitions for the bitfield "data_tc{t}_weight[8:0]".
 * parameter: tc {t} | stride size 0x4 | range [0, 7]
 * port="pif_tps_data_tc0_weight_i[8:0]"
 */

/* register address for bitfield data_tc{t}_weight[8:0] */
#define HW_ATL2_TPS_DATA_TCTWEIGHT_ADR(tc) (0x00007110 + (tc) * 0x4)
/* bitmask for bitfield data_tc{t}_weight[8:0] */
#define HW_ATL2_TPS_DATA_TCTWEIGHT_MSK 0x000001ff
/* inverted bitmask for bitfield data_tc{t}_weight[8:0] */
#define HW_ATL2_TPS_DATA_TCTWEIGHT_MSKN 0xfffffe00
/* lower bit position of bitfield data_tc{t}_weight[8:0] */
#define HW_ATL2_TPS_DATA_TCTWEIGHT_SHIFT 0
/* width of bitfield data_tc{t}_weight[8:0] */
#define HW_ATL2_TPS_DATA_TCTWEIGHT_WIDTH 9
/* default value of bitfield data_tc{t}_weight[8:0] */
#define HW_ATL2_TPS_DATA_TCTWEIGHT_DEFAULT 0x0

/* tx reg_res_dsbl bitfield definitions
 * preprocessor definitions for the bitfield "reg_res_dsbl".
 * port="pif_tx_reg_res_dsbl_i"
 */

/* register address for bitfield reg_res_dsbl */
#define HW_ATL2_TX_REG_RES_DSBL_ADR 0x00007000
/* bitmask for bitfield reg_res_dsbl */
#define HW_ATL2_TX_REG_RES_DSBL_MSK 0x20000000
/* inverted bitmask for bitfield reg_res_dsbl */
#define HW_ATL2_TX_REG_RES_DSBL_MSKN 0xdfffffff
/* lower bit position of bitfield reg_res_dsbl */
#define HW_ATL2_TX_REG_RES_DSBL_SHIFT 29
/* width of bitfield reg_res_dsbl */
#define HW_ATL2_TX_REG_RES_DSBL_WIDTH 1
/* default value of bitfield reg_res_dsbl */
#define HW_ATL2_TX_REG_RES_DSBL_DEFAULT 0x1

/* mac_phy register access busy bitfield definitions
 * preprocessor definitions for the bitfield "register access busy".
 * port="msm_pif_reg_busy_o"
 */

/* register address for bitfield reg_res_dsbl */
#define HW_ATL2_MPI_CTRL_REG_1_ADR 0x00004000
/* bitmask for bitfield reg_res_dsbl */
#define HW_ATL2_MPI_REG_RESET_MSK 0x80000000
/* lower bit position of bitfield reg_res_dsbl */
#define HW_ATL2_MPI_REG_RESET_SHIFT 31
/* bitmask for bitfield reg_res_dsbl */
#define HW_ATL2_MPI_REG_RES_DSBL_MSK 0x20000000
/* lower bit position of bitfield reg_res_dsbl */
#define HW_ATL2_MPI_REG_RES_DSBL_SHIFT 29

/* register address for bitfield register access busy */
#define HW_ATL2_MSM_REG_ACCESS_BUSY_ADR 0x00004400
/* bitmask for bitfield register access busy */
#define HW_ATL2_MSM_REG_ACCESS_BUSY_MSK 0x00001000
/* inverted bitmask for bitfield register access busy */
#define HW_ATL2_MSM_REG_ACCESS_BUSY_MSKN 0xffffefff
/* lower bit position of bitfield register access busy */
#define HW_ATL2_MSM_REG_ACCESS_BUSY_SHIFT 12
/* width of bitfield register access busy */
#define HW_ATL2_MSM_REG_ACCESS_BUSY_WIDTH 1

/* mac_phy msm register address[7:0] bitfield definitions
 * preprocessor definitions for the bitfield "msm register address[7:0]".
 * port="pif_msm_reg_addr_i[7:0]"
 */

/* register address for bitfield msm register address[7:0] */
#define HW_ATL2_MSM_REG_ADDR_ADR 0x00004400
/* bitmask for bitfield msm register address[7:0] */
#define HW_ATL2_MSM_REG_ADDR_MSK 0x000000ff
/* inverted bitmask for bitfield msm register address[7:0] */
#define HW_ATL2_MSM_REG_ADDR_MSKN 0xffffff00
/* lower bit position of bitfield msm register address[7:0] */
#define HW_ATL2_MSM_REG_ADDR_SHIFT 0
/* width of bitfield msm register address[7:0] */
#define HW_ATL2_MSM_REG_ADDR_WIDTH 8
/* default value of bitfield msm register address[7:0] */
#define HW_ATL2_MSM_REG_ADDR_DEFAULT 0x0

/* mac_phy register read strobe bitfield definitions
 * preprocessor definitions for the bitfield "register read strobe".
 * port="pif_msm_reg_rden_i"
 */

/* register address for bitfield register read strobe */
#define HW_ATL2_MSM_REG_RD_STROBE_ADR 0x00004400
/* bitmask for bitfield register read strobe */
#define HW_ATL2_MSM_REG_RD_STROBE_MSK 0x00000200
/* inverted bitmask for bitfield register read strobe */
#define HW_ATL2_MSM_REG_RD_STROBE_MSKN 0xfffffdff
/* lower bit position of bitfield register read strobe */
#define HW_ATL2_MSM_REG_RD_STROBE_SHIFT 9
/* width of bitfield register read strobe */
#define HW_ATL2_MSM_REG_RD_STROBE_WIDTH 1
/* default value of bitfield register read strobe */
#define HW_ATL2_MSM_REG_RD_STROBE_DEFAULT 0x0

/* mac_phy msm register read data[31:0] bitfield definitions
 * preprocessor definitions for the bitfield "msm register read data[31:0]".
 * port="msm_pif_reg_rd_data_o[31:0]"
 */

/* register address for bitfield msm register read data[31:0] */
#define HW_ATL2_MSM_REG_RD_DATA_ADR 0x00004408
/* bitmask for bitfield msm register read data[31:0] */
#define HW_ATL2_MSM_REG_RD_DATA_MSK 0xffffffff
/* inverted bitmask for bitfield msm register read data[31:0] */
#define HW_ATL2_MSM_REG_RD_DATA_MSKN 0x00000000
/* lower bit position of bitfield msm register read data[31:0] */
#define HW_ATL2_MSM_REG_RD_DATA_SHIFT 0
/* width of bitfield msm register read data[31:0] */
#define HW_ATL2_MSM_REG_RD_DATA_WIDTH 32

/* mac_phy msm register write data[31:0] bitfield definitions
 * preprocessor definitions for the bitfield "msm register write data[31:0]".
 * port="pif_msm_reg_wr_data_i[31:0]"
 */

/* register address for bitfield msm register write data[31:0] */
#define HW_ATL2_MSM_REG_WR_DATA_ADR 0x00004404
/* bitmask for bitfield msm register write data[31:0] */
#define HW_ATL2_MSM_REG_WR_DATA_MSK 0xffffffff
/* inverted bitmask for bitfield msm register write data[31:0] */
#define HW_ATL2_MSM_REG_WR_DATA_MSKN 0x00000000
/* lower bit position of bitfield msm register write data[31:0] */
#define HW_ATL2_MSM_REG_WR_DATA_SHIFT 0
/* width of bitfield msm register write data[31:0] */
#define HW_ATL2_MSM_REG_WR_DATA_WIDTH 32
/* default value of bitfield msm register write data[31:0] */
#define HW_ATL2_MSM_REG_WR_DATA_DEFAULT 0x0

/* mac_phy register write strobe bitfield definitions
 * preprocessor definitions for the bitfield "register write strobe".
 * port="pif_msm_reg_wren_i"
 */

/* register address for bitfield register write strobe */
#define HW_ATL2_MSM_REG_WR_STROBE_ADR 0x00004400
/* bitmask for bitfield register write strobe */
#define HW_ATL2_MSM_REG_WR_STROBE_MSK 0x00000100
/* inverted bitmask for bitfield register write strobe */
#define HW_ATL2_MSM_REG_WR_STROBE_MSKN 0xfffffeff
/* lower bit position of bitfield register write strobe */
#define HW_ATL2_MSM_REG_WR_STROBE_SHIFT 8
/* width of bitfield register write strobe */
#define HW_ATL2_MSM_REG_WR_STROBE_WIDTH 1
/* default value of bitfield register write strobe */
#define HW_ATL2_MSM_REG_WR_STROBE_DEFAULT 0x0

/* register address for bitfield PTP Digital Clock Read Enable */
#define HW_ATL2_PCS_PTP_CLOCK_READ_ENABLE_ADR 0x00004628
/* bitmask for bitfield PTP Digital Clock Read Enable */
#define HW_ATL2_PCS_PTP_CLOCK_READ_ENABLE_MSK 0x00000010
/* inverted bitmask for bitfield PTP Digital Clock Read Enable */
#define HW_ATL2_PCS_PTP_CLOCK_READ_ENABLE_MSKN 0xFFFFFFEF
/* lower bit position of bitfield PTP Digital Clock Read Enable */
#define HW_ATL2_PCS_PTP_CLOCK_READ_ENABLE_SHIFT 4
/* width of bitfield PTP Digital Clock Read Enable */
#define HW_ATL2_PCS_PTP_CLOCK_READ_ENABLE_WIDTH 1
/* default value of bitfield PTP Digital Clock Read Enable */
#define HW_ATL2_PCS_PTP_CLOCK_READ_ENABLE_DEFAULT 0x0

/* register address for ptp counter reading */
#define HW_ATL2_PCS_PTP_TS_VAL_ADDR(index) (0x00004900 + (index) * 0x4)

/* mif soft reset bitfield definitions
 * preprocessor definitions for the bitfield "soft reset".
 * port="pif_glb_res_i"
 */

/* register address for bitfield soft reset */
#define HW_ATL2_GLB_SOFT_RES_ADR 0x00000000
/* bitmask for bitfield soft reset */
#define HW_ATL2_GLB_SOFT_RES_MSK 0x00008000
/* inverted bitmask for bitfield soft reset */
#define HW_ATL2_GLB_SOFT_RES_MSKN 0xffff7fff
/* lower bit position of bitfield soft reset */
#define HW_ATL2_GLB_SOFT_RES_SHIFT 15
/* width of bitfield soft reset */
#define HW_ATL2_GLB_SOFT_RES_WIDTH 1
/* default value of bitfield soft reset */
#define HW_ATL2_GLB_SOFT_RES_DEFAULT 0x0

/* mif register reset disable bitfield definitions
 * preprocessor definitions for the bitfield "register reset disable".
 * port="pif_glb_reg_res_dsbl_i"
 */

/* register address for bitfield register reset disable */
#define HW_ATL2_GLB_REG_RES_DIS_ADR 0x00000000
/* bitmask for bitfield register reset disable */
#define HW_ATL2_GLB_REG_RES_DIS_MSK 0x00004000
/* inverted bitmask for bitfield register reset disable */
#define HW_ATL2_GLB_REG_RES_DIS_MSKN 0xffffbfff
/* lower bit position of bitfield register reset disable */
#define HW_ATL2_GLB_REG_RES_DIS_SHIFT 14
/* width of bitfield register reset disable */
#define HW_ATL2_GLB_REG_RES_DIS_WIDTH 1
/* default value of bitfield register reset disable */
#define HW_ATL2_GLB_REG_RES_DIS_DEFAULT 0x1

/* tx dma debug control definitions */
#define HW_ATL2_TX_DMA_DEBUG_CTL_ADR 0x00008920u

/* tx dma descriptor base address msw definitions */
#define HW_ATL2_TX_DMA_DESC_BASE_ADDRMSW_ADR(descriptor) \
			(0x00007c04u + (descriptor) * 0x40)

/* tx dma total request limit */
#define HW_ATL2_TX_DMA_TOTAL_REQ_LIMIT_ADR 0x00007b20u

/* tx interrupt moderation control register definitions
 * Preprocessor definitions for TX Interrupt Moderation Control Register
 * Base Address: 0x00008980
 * Parameter: queue {Q} | stride size 0x4 | range [0, 31]
 */

#define HW_ATL2_TX_INTR_MODERATION_CTL_ADR(queue) (0x00007c28u + (queue) * 0x40)

/* pcie reg_res_dsbl bitfield definitions
 * preprocessor definitions for the bitfield "reg_res_dsbl".
 * port="pif_pci_reg_res_dsbl_i"
 */

/* register address for bitfield reg_res_dsbl */
#define HW_ATL2_PCI_REG_RES_DSBL_ADR 0x00001000
/* bitmask for bitfield reg_res_dsbl */
#define HW_ATL2_PCI_REG_RES_DSBL_MSK 0x20000000
/* inverted bitmask for bitfield reg_res_dsbl */
#define HW_ATL2_PCI_REG_RES_DSBL_MSKN 0xdfffffff
/* lower bit position of bitfield reg_res_dsbl */
#define HW_ATL2_PCI_REG_RES_DSBL_SHIFT 29
/* width of bitfield reg_res_dsbl */
#define HW_ATL2_PCI_REG_RES_DSBL_WIDTH 1
/* default value of bitfield reg_res_dsbl */
#define HW_ATL2_PCI_REG_RES_DSBL_DEFAULT 0x1

/* PCI core control register */
#define HW_ATL2_PCI_REG_CONTROL6_ADR 0x1014u

/* global microprocessor scratch pad definitions */
#define HW_ATL2_GLB_CPU_SCRATCH_SCP_ADR(scratch_scp) \
	(0x00000300u + (scratch_scp) * 0x4)

/* register address for bitfield uP Force Interrupt */
#define A2_GLOBAL_CONTROL_2_ADR 0x00000404
/* bitmask for bitfield MIF Interrupt to ITR */
#define A2_MIF_INTERRUPT_TO_ITR_MSK 0x000003c0
#define A2_MIF_INTERRUPT_0_TO_ITR_MSK 0x00000040
#define A2_MIF_INTERRUPT_1_TO_ITR_MSK 0x00000080
#define A2_MIF_INTERRUPT_2_TO_ITR_MSK 0x00000100
#define A2_MIF_INTERRUPT_3_TO_ITR_MSK 0x00000200
/* lower bit position of bitfield MIF Interrupt to ITR */
#define A2_MIF_INTERRUPT_TO_ITR_SHIFT 6
/* width of bitfield MIF Interrupt to ITR */
#define A2_MIF_INTERRUPT_TO_ITR_WIDTH 4
/* default value of bitfield MIF Interrupt to ITR */
#define A2_MIF_INTERRUPT_TO_ITR_DEFAULT 0x3
/* bitmask for bitfield Enable MIF Interrupt to ITR */
#define A2_EN_INTERRUPT_TO_ITR_MSK 0x00003c00
#define A2_EN_INTERRUPT_MIF0_TO_ITR_MSK 0x00000400
#define A2_EN_INTERRUPT_MIF1_TO_ITR_MSK 0x00000800
#define A2_EN_INTERRUPT_MIF2_TO_ITR_MSK 0x00001000
#define A2_EN_INTERRUPT_MIF3_TO_ITR_MSK 0x00002000
/* lower bit position of bitfield Enable MIF Interrupt to ITR */
#define A2_EN_INTERRUPT_TO_ITR_SHIFT 0xA
/* width of bitfield Enable MIF Interrupt to ITR */
#define A2_EN_INTERRUPT_TO_ITR_WIDTH 4
/* default value of bitfield Enable MIF Interrupt to ITR */
#define A2_EN_INTERRUPT_TO_ITR_DEFAULT 0x0


/* register address for bitfield uP High Priority Interrupt */
#define A2_GLOBAL_ALARMS_1_ADR 0x00000904
#define A2_GLOBAL_INTERNAL_ALARMS_1_ADR 0x00000924
#define A2_GLOBAL_LASI_1_MASK_ADR 0x00000944
#define A2_GLOBAL_HIGH_PRIO_INTERRUPT_1_MASK_ADR 0x00000964
#define A2_GLOBAL_LOW_PRIO_INTERRUPT_1_MASK_ADR 0x00000984
/* bitmask for bitfield TSG PTM GPIO interrupt */
#define A2_TSG_PTM_GPIO_INTERRUPT_MSK 0x00000200
/* lower bit position of bitfield TSG PTM GPIO interrupt */
#define A2_TSG_PTM_GPIO_INTERRUPT_SHIFT 9
/* bitmask for bitfield TSG PTP GPIO interrupt */
#define A2_TSG_PTP_GPIO_INTERRUPT_MSK 0x00000020
/* lower bit position of bitfield TSG PTP GPIO interrupt */
#define A2_TSG_PTP_GPIO_INTERRUPT_SHIFT 5


/* register address for bitfield uP High Priority Interrupt */
#define A2_PCIE_ALARMS_0_ADR 0x00001F00
#define A2_PCIE_INTERNAL_ALARMS_0_ADR 0x00001F20
#define A2_PCIE_LASI_0_MASK_ADR 0x00001F40
#define A2_PCIE_HIGH_PRIO_INTERRUPT_0_MASK_ADR 0x00001F60
#define A2_PCIE_LOW_PRIO_INTERRUPT_0_MASK_ADR 0x00001F80
/* bitmask for bitfield TSG PTM GPIO interrupt */
#define A2_PCIE_PTM_CLOCK_UPDATE_ALARM_MSK 0x00000040
/* lower bit position of bitfield TSG PTM GPIO interrupt */
#define A2_PCIE_PTM_CLOCK_UPDATE_ALARM_SHIFT 6


#define HW_ATL2_RX_CTRL_ADDR_BEGIN_FL3L4   0x00005380
#define HW_ATL2_RX_SRCA_ADDR_BEGIN_FL3L4   0x000053B0
#define HW_ATL2_RX_DESTA_ADDR_BEGIN_FL3L4  0x000053D0

#define HW_ATL2_RX_GET_ADDR_CTRL_FL3L4(location)  \
	(HW_ATL2_RX_CTRL_ADDR_BEGIN_FL3L4 + ((location) * 0x4))
#define HW_ATL2_RX_GET_ADDR_SRCA_FL3L4(location)  \
	(HW_ATL2_RX_SRCA_ADDR_BEGIN_FL3L4 + ((location) * 0x4))
#define HW_ATL2_RX_GET_ADDR_DESTA_FL3L4(location) \
	(HW_ATL2_RX_DESTA_ADDR_BEGIN_FL3L4 + ((location) * 0x4))

/* TSG registers */
#define HW_ATL2_TSG_REG_ADR(clk, reg_name) \
		(clk == ATL_TSG_CLOCK_SEL_0 ? HW_ATL2_CLK0_##reg_name##_ADR : HW_ATL2_CLK1_##reg_name##_ADR)\

#define HW_ATL2_CLK0_CLOCK_CFG_ADR 0x00000CA0u
#define HW_ATL2_CLK1_CLOCK_CFG_ADR 0x00000D50u
#define HW_ATL2_TSG_SYNC_RESET_MSK 0x00000001
#define HW_ATL2_TSG_SYNC_RESET_SHIFT 0x00000000
#define HW_ATL2_TSG_CLOCK_EN_MSK 0x00000002
#define HW_ATL2_TSG_CLOCK_EN_SHIFT 0x00000001
#define HW_ATL2_TSG_CLOCK_MUX_SELECT_MSK 0x0000000C
#define HW_ATL2_TSG_CLOCK_MUX_SELECT_SHIFT 0x00000002
#define HW_ATL2_TSG_CLOCK_MUX_INTERNAL 0x00000000
#define HW_ATL2_TSG_CLOCK_MUX_REFERENCE 0x00000001
#define HW_ATL2_TSG_CLOCK_MUX_GPIO 0x00000002
#define HW_ATL2_TSG_CLOCK_MUX_1588 0x00000003
 
#define HW_ATL2_CLK0_CLOCK_MODIF_CTRL_ADR 0x00000CA4u
#define HW_ATL2_CLK1_CLOCK_MODIF_CTRL_ADR 0x00000D54u
#define HW_ATL2_TSG_SET_COUNTER_MSK 0x00000001
#define HW_ATL2_TSG_SUBSTRACT_COUNTER_MSK 0x00000002
#define HW_ATL2_TSG_ADD_COUNTER_MSK 0x00000004
#define HW_ATL2_TSG_LOAD_INC_CFG_MSK 0x00000008
#define HW_ATL2_TSG_SET_PERIODIC_CORRECTION_MSK 0x00000010

#define HW_ATL2_CLK0_CLOCK_MODIF_VAL_LSW_ADR 0x00000CA8u
#define HW_ATL2_CLK1_CLOCK_MODIF_VAL_LSW_ADR 0x00000D58u

#define HW_ATL2_CLK0_CLOCK_MODIF_VAL_MSW_ADR 0x00000Cacu
#define HW_ATL2_CLK1_CLOCK_MODIF_VAL_MSW_ADR 0x00000D5cu

#define HW_ATL2_CLK0_CLOCK_INC_CFG_ADR 0x00000CB0u
#define HW_ATL2_CLK1_CLOCK_INC_CFG_ADR 0x00000D60u
#define HW_ATL2_TSG_CLOCK_INC_CFG_NS_SHIFT 0x00000000
#define HW_ATL2_TSG_CLOCK_INC_CFG_NS_MSK 0x000000FF
#define HW_ATL2_TSG_CLOCK_INC_CFG_FNS_SHIFT 0x00000018
#define HW_ATL2_TSG_CLOCK_INC_CFG_FNS_MSK 0xFFFFFF00

#define HW_ATL2_CLK0_PERIODIC_CORRECTION_ADR 0x00000CB4u
#define HW_ATL2_CLK1_PERIODIC_CORRECTION_ADR 0x00000D64u
#define HW_ATL2_TSG_PERIODIC_CORRECTION_PERIOD_SHIFT 0x00000000
#define HW_ATL2_TSG_PERIODIC_CORRECTION_PERIOD_MSK 0x000000FF
#define HW_ATL2_TSG_CLOCK_PERIODIC_CORRECTION_FNS_SHIFT 0x00000018
#define HW_ATL2_TSG_CLOCK_PERIODIC_CORRECTION_FNS_MSK 0xFFFFFF00

#define HW_ATL2_CLK0_READ_CUR_NS_LSW_ADR 0x00000CB8u
#define HW_ATL2_CLK1_READ_CUR_NS_LSW_ADR 0x00000D68u

#define HW_ATL2_CLK0_READ_CUR_NS_MSW_ADR 0x00000CBCu
#define HW_ATL2_CLK1_READ_CUR_NS_MSW_ADR 0x00000D6cu

#define HW_ATL2_CLK0_READ_TIME_CFG_ADR 0x00000CC0u
#define HW_ATL2_CLK1_READ_TIME_CFG_ADR 0x00000D70u
#define HW_ATL2_TSG_READ_CUR_TIME_MSK 0x00000001

#define HW_ATL2_CLK0_GPIO_CFG_ADR 0x00000CC4u
#define HW_ATL2_CLK1_GPIO_CFG_ADR 0x00000D74u
#define HW_ATL2_TSG_GPIO_IN_MONITOR_EN_SHIFT 0x00000000
#define HW_ATL2_TSG_GPIO_IN_MONITOR_EN_MSK 0x00000001
#define HW_ATL2_TSG_GPIO_IN_MODE_SHIFT 0x00000001
#define HW_ATL2_TSG_GPIO_IN_MODE_MSK 0x00000006
#define HW_ATL2_TSG_GPIO_IN_MODE_POSEDGE 0x00000000
#define HW_ATL2_TSG_GPIO_IN_MODE_NEGEDGE 0x00000002
#define HW_ATL2_TSG_GPIO_IN_MODE_TOGGLE 0x00000004

#define HW_ATL2_CLK0_EXT_CLK_CFG_ADR 0x00000CC8u
#define HW_ATL2_CLK1_EXT_CLK_CFG_ADR 0x00000D78u
#define HW_ATL2_TSG_EXT_CLK_MONITOR_EN_SHIFT 0x00000000
#define HW_ATL2_TSG_EXT_CLK_MONITOR_EN_MSK 0x00000001
#define HW_ATL2_TSG_EXT_CLK_MONITOR_PERIOD_SHIFT 0x00000001
#define HW_ATL2_TSG_EXT_CLK_MONITOR_PERIOD_MSK 0x00FFFFFE

#define HW_ATL2_CLK0_EXT_CLK_COUNT_ADR 0x00000CCCu
#define HW_ATL2_CLK1_EXT_CLK_COUNT_ADR 0x00000D7Cu

#define HW_ATL2_CLK0_GPIO_EVENT_TS_LSW_ADR 0x00000CD0u
#define HW_ATL2_CLK1_GPIO_EVENT_TS_LSW_ADR 0x00000D80u

#define HW_ATL2_CLK0_GPIO_EVENT_TS_MSW_ADR 0x00000CD4u
#define HW_ATL2_CLK1_GPIO_EVENT_TS_MSW_ADR 0x00000D84u

#define HW_ATL2_CLK0_EXT_CLK_TS_LSW_ADR 0x00000CD8u
#define HW_ATL2_CLK1_EXT_CLK_TS_LSW_ADR 0x00000D88u

#define HW_ATL2_CLK0_EXT_CLK_TS_MSW_ADR 0x00000CDCu
#define HW_ATL2_CLK1_EXT_CLK_TS_MSW_ADR 0x00000D8Cu

#define HW_ATL2_CLK0_GPIO_EVENT_GEN_TS_LSW_ADR 0x00000CE0u
#define HW_ATL2_CLK1_GPIO_EVENT_GEN_TS_LSW_ADR 0x00000D90u

#define HW_ATL2_CLK0_GPIO_EVENT_GEN_TS_MSW_ADR 0x00000CE4u
#define HW_ATL2_CLK1_GPIO_EVENT_GEN_TS_MSW_ADR 0x00000D94u

#define HW_ATL2_CLK0_GPIO_EVENT_GEN_CFG_ADR 0x00000CE8u
#define HW_ATL2_CLK1_GPIO_EVENT_GEN_CFG_ADR 0x00000D98u
#define HW_ATL2_TSG_GPIO_OUTPUT_EN_SHIFT 0x00000000
#define HW_ATL2_TSG_GPIO_OUTPUT_EN_MSK 0x00000001
#define HW_ATL2_TSG_GPIO_EVENT_MODE_SHIFT 0x00000001
#define HW_ATL2_TSG_GPIO_EVENT_MODE_MSK 0x00000006
#define HW_ATL2_TSG_GPIO_EVENT_MODE_CLEAR 0x00000000
#define HW_ATL2_TSG_GPIO_EVENT_MODE_SET 0x00000001
#define HW_ATL2_TSG_GPIO_EVENT_MODE_CLEAR_ON_TIME 0x00000002
#define HW_ATL2_TSG_GPIO_EVENT_MODE_SET_ON_TIME 0x00000003
#define HW_ATL2_TSG_GPIO_GEN_OUTPUT_EN_SHIFT 0x00000003
#define HW_ATL2_TSG_GPIO_GEN_OUTPUT_EN_MSK 0x00000008
#define HW_ATL2_TSG_GPIO_EVENT_STATUS_SHIFT 0x00000004
#define HW_ATL2_TSG_GPIO_EVENT_STATUS_MSK 0x00000010
#define HW_ATL2_TSG_GPIO_CLK_OUTPUT_EN_SHIFT 0x00000005
#define HW_ATL2_TSG_GPIO_CLK_OUTPUT_EN_MSK 0x00000020
#define HW_ATL2_TSG_GPIO_OUTPUT_BIT_POS_SHIFT 0x00000006
#define HW_ATL2_TSG_GPIO_OUTPUT_BIT_POS_MSK 0x00000fc0

#define HW_ATL2_CLK0_GPIO_EVENT_HIGH_TIME_LSW_ADR 0x00000CF0u
#define HW_ATL2_CLK1_GPIO_EVENT_HIGH_TIME_LSW_ADR 0x00000DA0u

#define HW_ATL2_CLK0_GPIO_EVENT_HIGH_TIME_MSW_ADR 0x00000CF4u
#define HW_ATL2_CLK1_GPIO_EVENT_HIGH_TIME_MSW_ADR 0x00000DA4u

#define HW_ATL2_CLK0_GPIO_EVENT_LOW_TIME_LSW_ADR 0x00000CF8u
#define HW_ATL2_CLK1_GPIO_EVENT_LOW_TIME_LSW_ADR 0x00000DA8u

#define HW_ATL2_CLK0_GPIO_EVENT_LOW_TIME_MSW_ADR 0x00000CFCu
#define HW_ATL2_CLK1_GPIO_EVENT_LOW_TIME_MSW_ADR 0x00000DACu

#define HW_ATL2_TSG_SPARE_READ_REG_ADR 0x00000D00u
#define HW_ATL2_TSG_SPARE_WRITE_REG_ADR 0x00000D04u
#define HW_ATL2_TSG_SPARE_FPGA_GPIO_CTRL_SHIFT 0x00000000u
#define HW_ATL2_TSG_SPARE_FPGA_GPIO_CTRL_MSK 0x00000FFFu
#define HW_ATL2_TSG_SPARE_FPGA_PTP_GPIO_EVNT_O 0x00000000u
#define HW_ATL2_TSG_SPARE_FPGA_PTP_CLK_EVNT_O 0x00000000u
#define HW_ATL2_TSG_SPARE_FPGA_PTP_GPIO_TS_I 0x00000001u
#define HW_ATL2_TSG_SPARE_FPGA_PTP_EXT_CLK_TS_I 0x00000002u
#define HW_ATL2_TSG_SPARE_FPGA_PTM_GPIO_TS_I 0x00000004u
#define HW_ATL2_TSG_SPARE_FPGA_PTM_EXT_CLK_TS_I 0x00000008u
#define HW_ATL2_TSG_SPARE_FPGA_PTM_CLK_EVNT_O 0x00000010u
#define HW_ATL2_TSG_SPARE_FPGA_PTM_EXT_CLK_O 0x00000020u

/* PCI core control register */
#define HW_ATL2_PCI_REG_CONTROL6_ADR 0x1014u

/* Launch time control register */
#define HW_ATL2_LT_CTRL_ADR 0x00007a1c

#define HW_ATL2_LT_CTRL_AVB_LEN_CMP_TRSHLD_MSK 0xFFFF0000
#define HW_ATL2_LT_CTRL_AVB_LEN_CMP_TRSHLD_SHIFT 16

#define HW_ATL2_LT_CTRL_CLK_RATIO_MSK 0x0000FF00
#define HW_ATL2_LT_CTRL_CLK_RATIO_SHIFT 8
#define HW_ATL2_LT_CTRL_CLK_RATIO_QUATER_SPEED 4
#define HW_ATL2_LT_CTRL_CLK_RATIO_HALF_SPEED 2
#define HW_ATL2_LT_CTRL_CLK_RATIO_FULL_SPEED 1

#define HW_ATL2_LT_CTRL_25G_MODE_SUPPORT_MSK 0x00000008
#define HW_ATL2_LT_CTRL_25G_MODE_SUPPORT_SHIFT 3

#define HW_ATL2_LT_CTRL_LINK_SPEED_MSK 0x00000007
#define HW_ATL2_LT_CTRL_LINK_SPEED_SHIFT 0

#define HW_ATL2_LT_CTRL2_ADR 0x00007A20
#define HW_ATL2_LT_CTRL2_TIME_ADJ_MSK 0x000000ff
#define HW_ATL2_LT_CTRL2_TIME_ADJ_SHIFT 0

#define HW_ATL2_LT_CTRL3_ADR 0x00007A24
#define HW_ATL2_LT_CTRL3_BG_TRF_ADJ_MSK 0x000000ff
#define HW_ATL2_LT_CTRL3_BG_TRF_ADJ_SHIFT 0


/*  ahb_mem_addr{f}[31:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "ahb_mem_addr{f}[31:0]".
 * Parameter: filter {f} | stride size 0x10 | range [0, 127]
 * PORT="ahb_mem_addr{f}[31:0]"
 */

/* Register address for bitfield ahb_mem_addr{f}[31:0] */
#define HW_ATL2_RPF_ACT_RSLVR_REQ_TAG_ADR(filter) (0x00014000u + (filter) * 0x10)
/* Bitmask for bitfield ahb_mem_addr{f}[31:0] */
#define HW_ATL2_RPF_ACT_RSLVR_REQ_TAG_MSK 0xFFFFFFFFu
/* Inverted bitmask for bitfield ahb_mem_addr{f}[31:0] */
#define HW_ATL2_RPF_ACT_RSLVR_REQ_TAG_MSKN 0x00000000u
/* Lower bit position of bitfield ahb_mem_addr{f}[31:0] */
#define HW_ATL2_RPF_ACT_RSLVR_REQ_TAG_SHIFT 0
/* Width of bitfield ahb_mem_addr{f}[31:0] */
#define HW_ATL2_RPF_ACT_RSLVR_REQ_TAG_WIDTH 31
/* Default value of bitfield ahb_mem_addr{f}[31:0] */
#define HW_ATL2_RPF_ACT_RSLVR_REQ_TAG_DEFAULT 0x0

/* Register address for bitfield ahb_mem_addr{f}[31:0] */
#define HW_ATL2_RPF_ACT_RSLVR_TAG_MASK_ADR(filter) (0x00014004u + (filter) * 0x10)
/* Bitmask for bitfield ahb_mem_addr{f}[31:0] */
#define HW_ATL2_RPF_ACT_RSLVR_TAG_MASK_MSK 0xFFFFFFFFu
/* Inverted bitmask for bitfield ahb_mem_addr{f}[31:0] */
#define HW_ATL2_RPF_ACT_RSLVR_TAG_MASK_MSKN 0x00000000u
/* Lower bit position of bitfield ahb_mem_addr{f}[31:0] */
#define HW_ATL2_RPF_ACT_RSLVR_TAG_MASK_SHIFT 0
/* Width of bitfield ahb_mem_addr{f}[31:0] */
#define HW_ATL2_RPF_ACT_RSLVR_TAG_MASK_WIDTH 31
/* Default value of bitfield ahb_mem_addr{f}[31:0] */
#define HW_ATL2_RPF_ACT_RSLVR_TAG_MASK_DEFAULT 0x0

/* Register address for bitfield ahb_mem_addr{f}[31:0] */
#define HW_ATL2_RPF_ACT_RSLVR_ACTN_ADR(filter) (0x00014008u + (filter) * 0x10)
/* Bitmask for bitfield ahb_mem_addr{f}[31:0] */
#define HW_ATL2_RPF_ACT_RSLVR_ACTN_MSK 0x000007FFu
/* Inverted bitmask for bitfield ahb_mem_addr{f}[31:0] */
#define HW_ATL2_RPF_ACT_RSLVR_ACTN_MSKN 0xFFFFF800u
/* Lower bit position of bitfield ahb_mem_addr{f}[31:0] */
#define HW_ATL2_RPF_ACT_RSLVR_ACTN_SHIFT 0
/* Width of bitfield ahb_mem_addr{f}[31:0] */
#define HW_ATL2_RPF_ACT_RSLVR_ACTN_WIDTH 10
/* Default value of bitfield ahb_mem_addr{f}[31:0] */
#define HW_ATL2_RPF_ACT_RSLVR_ACTN_DEFAULT 0x0

/* Register address to map Q to TC */
#define HW_ATL2_RX_Q_TO_TC_MAP_ADR(q) (0x00005900u + ((q >> 1) & (~0x3)))
/* Width of bitfield */
#define HW_ATL2_RX_Q_TO_TC_MAP_WIDTH 4
/* Lower bit position of bitfield */
#define HW_ATL2_RX_Q_TO_TC_MAP_SHIFT(q) ((q & 0x7) * HW_ATL2_RX_Q_TO_TC_MAP_WIDTH)
/* Bitmask for bitfield */
#define HW_ATL2_RX_Q_TO_TC_MAP_MSK(q) (0x00000007u << HW_ATL2_RX_Q_TO_TC_MAP_SHIFT(q))
/* Inverted bitmask for bitfield */
#define HW_ATL2_RX_Q_TO_TC_MAP_MSKN(q) (~HW_ATL2_RX_Q_TO_TC_MAP_MSK(q))


/* rpf_rec_tab_en[15:0] Bitfield Definitions
 * Preprocessor definitions for the bitfield "rpf_rec_tab_en[15:0]".
 * PORT="pif_rpf_rec_tab_en[15:0]"
 */
/* Register address for bitfield rpf_rec_tab_en[15:0] */
#define HW_ATL2_RPF_REC_TAB_EN_ADR 0x00006ff0u
/* Bitmask for bitfield rpf_rec_tab_en[15:0] */
#define HW_ATL2_RPF_REC_TAB_EN_MSK 0x0000FFFFu
/* Inverted bitmask for bitfield rpf_rec_tab_en[15:0] */
#define HW_ATL2_RPF_REC_TAB_EN_MSKN 0xFFFF0000u
/* Lower bit position of bitfield rpf_rec_tab_en[15:0] */
#define HW_ATL2_RPF_REC_TAB_EN_SHIFT 0
/* Width of bitfield rpf_rec_tab_en[15:0] */
#define HW_ATL2_RPF_REC_TAB_EN_WIDTH 16
/* Default value of bitfield rpf_rec_tab_en[15:0] */
#define HW_ATL2_RPF_REC_TAB_EN_DEFAULT 0x0

/* Register address for firmware shared input buffer */
#define HW_ATL2_MIF_SHARED_BUFFER_IN_ADR(dword) (0x00012000U + (dword) * 0x4U)
/* Register address for firmware shared output buffer */
#define HW_ATL2_MIF_SHARED_BUFFER_OUT_ADR(dword) (0x00013000U + (dword) * 0x4U)

/* pif_host_finished_buf_wr_i Bitfield Definitions
 * Preprocessor definitions for the bitfield "pif_host_finished_buf_wr_i".
 * PORT="pif_host_finished_buf_wr_i"
 */
/* Register address for bitfield rpif_host_finished_buf_wr_i */
#define HW_ATL2_MIF_HOST_FINISHED_WRITE_ADR 0x00000e00u
/* Bitmask for bitfield pif_host_finished_buf_wr_i */
#define HW_ATL2_MIF_HOST_FINISHED_WRITE_MSK 0x00000001u
/* Inverted bitmask for bitfield pif_host_finished_buf_wr_i */
#define HW_ATL2_MIF_HOST_FINISHED_WRITE_MSKN 0xFFFFFFFEu
/* Lower bit position of bitfield pif_host_finished_buf_wr_i */
#define HW_ATL2_MIF_HOST_FINISHED_WRITE_SHIFT 0
/* Width of bitfield pif_host_finished_buf_wr_i */
#define HW_ATL2_MIF_HOST_FINISHED_WRITE_WIDTH 1
/* Default value of bitfield pif_host_finished_buf_wr_i */
#define HW_ATL2_MIF_HOST_FINISHED_WRITE_DEFAULT 0x0


/* pif_mcp_finished_buf_rd_i Bitfield Definitions
 * Preprocessor definitions for the bitfield "pif_mcp_finished_buf_rd_i".
 * PORT="pif_mcp_finished_buf_rd_i"
 */
/* Register address for bitfield pif_mcp_finished_buf_rd_i */
#define HW_ATL2_MIF_MCP_FINISHED_READ_ADR 0x00000e04u
/* Bitmask for bitfield pif_mcp_finished_buf_rd_i */
#define HW_ATL2_MIF_MCP_FINISHED_READ_MSK 0x00000001u
/* Inverted bitmask for bitfield pif_mcp_finished_buf_rd_i */
#define HW_ATL2_MIF_MCP_FINISHED_READ_MSKN 0xFFFFFFFEu
/* Lower bit position of bitfield pif_mcp_finished_buf_rd_i */
#define HW_ATL2_MIF_MCP_FINISHED_READ_SHIFT 0
/* Width of bitfield pif_mcp_finished_buf_rd_i */
#define HW_ATL2_MIF_MCP_FINISHED_READ_WIDTH 1
/* Default value of bitfield pif_mcp_finished_buf_rd_i */
#define HW_ATL2_MIF_MCP_FINISHED_READ_DEFAULT 0x0

/* Register address for bitfield pif_full_reset_reg_i */
#define HW_ATL2_COMMON_GENERAL_CTRL_REG_ADR 0x00003000u
/* Bitmask for bitfield pif_full_reset_reg_i */
#define HW_ATL2_COM_FULL_RESET_MSK 0x00000001u
/* Inverted bitmask for bitfield pif_full_reset_reg_i */
#define HW_ATL2_COM_FULL_RESET_MSKN 0xFFFFFFFEu
/* Lower bit position of bitfield pif_full_reset_reg_i */
#define HW_ATL2_COM_FULL_RESET_SHIFT 0
/* Width of bitfield pif_full_reset_reg_i */
#define HW_ATL2_COM_FULL_RESET_WIDTH 1
/* Default value of bitfield pif_full_reset_reg_i */
#define HW_ATL2_COM_FULL_RESET_DEFAULT 0x0

#define HW_ATL2_MIF_BOOT_REG_ADR 0x00003040u

#define HW_ATL2_MIF_BOOT_VER_ADR 0x000003E0u

#define HW_ATL2_MCP_HOST_REQ_INT_ADR 0x00000F00u
#define HW_ATL2_MCP_HOST_REQ_INT_SET_ADR 0x00000F04u
#define HW_ATL2_MCP_HOST_REQ_INT_CLR_ADR 0x00000F08u


/* Register address for PCI config */
#define HW_ATL2_PCI_CFG_ADR(offset) (0x00016000u + offset)

#define HW_ATL2_PCI_CFG_PTM_CAP_OFF (0x270u)

#define HW_ATL2_PCI_PTM_CAP_ADR 	HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_CAP_OFF + 0x4u)
#define HW_ATL2_PCI_PTM_CAP_CLK_GRAN_MSK 0x0000FF00u
#define HW_ATL2_PCI_PTM_CAP_CLK_GRAN_SHIFT 8
#define HW_ATL2_PCI_PTM_CAP_ROOT_MSK 0x00000004u
#define HW_ATL2_PCI_PTM_CAP_ROOT_SHIFT 2
#define HW_ATL2_PCI_PTM_CAP_RES_MSK 0x00000002u
#define HW_ATL2_PCI_PTM_CAP_RES_SHIFT 1
#define HW_ATL2_PCI_PTM_CAP_REQ_MSK 0x00000001u
#define HW_ATL2_PCI_PTM_CAP_REQ_SHIFT 0

#define HW_ATL2_PCI_PTM_CONTROL_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_CAP_OFF + 0x8u)
#define HW_ATL2_PCI_PTM_CTRL_EFF_GRAN_MSK 0x0000FF00u
#define HW_ATL2_PCI_PTM_CTRL_EFF_GRAN_SHIFT 8
#define HW_ATL2_PCI_PTM_CTRL_ROOT_SEL_MSK 0x00000002u
#define HW_ATL2_PCI_PTM_CTRL_ROOT_SEL_SHIFT 1
#define HW_ATL2_PCI_PTM_CTRL_PTM_EN_MSK 0x00000001u
#define HW_ATL2_PCI_PTM_CTRL_PTM_EN_SHIFT 0

#define HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF (0x27Cu)

#define HW_ATL2_PCI_PTM_REQ_HDR_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 4u)
#define HW_ATL2_PCI_PTM_REQ_VSEC_LEN_MSK 0xFFF00000u
#define HW_ATL2_PCI_PTM_REQ_VSEC_LEN_SHIFT 20
#define HW_ATL2_PCI_PTM_REQ_VSEC_REV_MSK 0x000F0000u
#define HW_ATL2_PCI_PTM_REQ_VSEC_REV_SHIFT 16
#define HW_ATL2_PCI_PTM_REQ_VSEC_ID_MSK 0x0000FFFFu
#define HW_ATL2_PCI_PTM_REQ_VSEC_ID_SHIFT 0

#define HW_ATL2_PCI_PTM_REQ_CTRL_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 8u)
#define HW_ATL2_PCI_PTM_REQ_PDEL_BYTE_REV_MSK 0x00010000u
#define HW_ATL2_PCI_PTM_REQ_PDEL_BYTE_REV_SHIFT 16
#define HW_ATL2_PCI_PTM_REQ_LONG_TIMER_MSK 0x0000FF00u
#define HW_ATL2_PCI_PTM_REQ_LONG_TIMER_SHIFT 8
#define HW_ATL2_PCI_PTM_REQ_START_UPDATE_MSK 0x00000002u
#define HW_ATL2_PCI_PTM_REQ_START_UPDATE_SHIFT 1
#define HW_ATL2_PCI_PTM_REQ_AUTO_UPDATE_EN_MSK 0x00000001u
#define HW_ATL2_PCI_PTM_REQ_AUTO_UPDATE_EN_SHIFT 0

#define HW_ATL2_PCI_PTM_REQ_STATUS_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0xCu)
#define HW_ATL2_PCI_PTM_REQ_STATUS_MANUAL_UPD_ALLOWED_MSK 0x00000002u
#define HW_ATL2_PCI_PTM_REQ_STATUS_MANUAL_UPD_ALLOWED_SHIFT 1
#define HW_ATL2_PCI_PTM_REQ_CONTEXT_VALID_MSK 0x00000001u
#define HW_ATL2_PCI_PTM_REQ_CONTEXT_VALID_SHIFT 0

#define HW_ATL2_PCI_PTM_REQ_LOCAL_LSW_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x10u)
#define HW_ATL2_PCI_PTM_REQ_LOCAL_MSW_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x14u)

#define HW_ATL2_PCI_PTM_REQ_T1_LSW_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x18u)
#define HW_ATL2_PCI_PTM_REQ_T1_MSW_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x1Cu)
#define HW_ATL2_PCI_PTM_REQ_T1P_LSW_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x20u)
#define HW_ATL2_PCI_PTM_REQ_T1P_MSW_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x24u)

#define HW_ATL2_PCI_PTM_REQ_T4_LSW_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x28u)
#define HW_ATL2_PCI_PTM_REQ_T4_MSW_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x2Cu)
#define HW_ATL2_PCI_PTM_REQ_T4P_LSW_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x30u)
#define HW_ATL2_PCI_PTM_REQ_T4P_MSW_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x34u)

#define HW_ATL2_PCI_PTM_REQ_MASTER_LSW_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x38u)
#define HW_ATL2_PCI_PTM_REQ_MASTER_MSW_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x3Cu)

#define HW_ATL2_PCI_PTM_REQ_PROP_DELAY_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x40u)

#define HW_ATL2_PCI_PTM_REQ_MASTERT1_LSW_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x44u)
#define HW_ATL2_PCI_PTM_REQ_MASTERT1_MSW_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x48u)

#define HW_ATL2_PCI_PTM_REQ_TX_LATENCY_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x4Cu)
#define HW_ATL2_PCI_PTM_REQ_RX_LATENCY_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x50u)

#define HW_ATL2_PCI_PTM_REQ_CLOCK_CORR_LSW_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x54u)
#define HW_ATL2_PCI_PTM_REQ_CLOCK_CORR_MSW_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x58u)

#define HW_ATL2_PCI_PTM_REQ_NOM_CLOCK_T_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x5Cu)
#define HW_ATL2_PCI_PTM_REQ_NOM_CLOCK_T_INT_MSK 0x00FF00000u
#define HW_ATL2_PCI_PTM_REQ_NOM_CLOCK_T_INT_SHIFT 16
#define HW_ATL2_PCI_PTM_REQ_NOM_CLOCK_T_FRAC_MSK 0x0000FFFFu
#define HW_ATL2_PCI_PTM_REQ_NOM_CLOCK_T_FRAC_SHIFT 0

#define HW_ATL2_PCI_PTM_REQ_SCALED_CLOCK_T_ADR HW_ATL2_PCI_CFG_ADR(HW_ATL2_PCI_CFG_PTM_REQ_CAP_OFF + 0x60u)
#define HW_ATL2_PCI_PTM_REQ_SCALED_CLOCK_T_EN_MSK 0x80000000u
#define HW_ATL2_PCI_PTM_REQ_SCALED_CLOCK_T_EN_SHIFT 31
#define HW_ATL2_PCI_PTM_REQ_SCALED_CLOCK_T_INT_MSK 0x00FF00000u
#define HW_ATL2_PCI_PTM_REQ_SCALED_CLOCK_T_INT_SHIFT 16
#define HW_ATL2_PCI_PTM_REQ_SCALED_CLOCK_T_FRAC_MSK 0x0000FFFFu
#define HW_ATL2_PCI_PTM_REQ_SCALED_CLOCK_T_FRAC_SHIFT 0

/*! @name PCIE pif_phi_ptm_update_timer_enable Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pif_phi_ptm_update_timer_enable".
*
*   Type: R/W
*
*   Notes: Enable programmable timer to update PTM clock. Refer PTM timer register for programming the timer value.
*
*   PORT="pif_phi_ptm_update_timer_enable_i"
@{ */
/*! \brief Register address for bitfield pif_phi_ptm_update_timer_enable */
#define HW_ATL2_HW_ATL2_PTM_UPDATETIMERENABLE_ADR 0x00001530u
/*! \brief Bitmask for bitfield pif_phi_ptm_update_timer_enable */
#define HW_ATL2_HW_ATL2_PTM_UPDATETIMERENABLE_MSK 0x00000400u
/*! \brief Inverted bitmask for bitfield pif_phi_ptm_update_timer_enable */
#define HW_ATL2_HW_ATL2_PTM_UPDATETIMERENABLE_MSKN 0xFFFFFBFFu
/*! \brief Lower bit position of bitfield pif_phi_ptm_update_timer_enable */
#define HW_ATL2_HW_ATL2_PTM_UPDATETIMERENABLE_SHIFT 10
/*! \brief Width of bitfield pif_phi_ptm_update_timer_enable */
#define HW_ATL2_HW_ATL2_PTM_UPDATETIMERENABLE_WIDTH 1
/*! \brief Default value of bitfield pif_phi_ptm_update_timer_enable */
#define HW_ATL2_HW_ATL2_PTM_UPDATETIMERENABLE_DEFAULT 0x0u
/*@}*/

/*! @name PCIE pif_phi_ptm_auto_update_signal Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pif_phi_ptm_auto_update_signal".
*
*   Type: R/W
*
*   Notes: update the PTM Requester Context
and Clock automatically every 10ms
*
*   PORT="pif_phi_ptm_auto_update_signal_i"
@{ */
/*! \brief Register address for bitfield pif_phi_ptm_auto_update_signal */
#define HW_ATL2_HW_ATL2_PTM_AUTOUPDATESIGNAL_ADR 0x00001530u
/*! \brief Bitmask for bitfield pif_phi_ptm_auto_update_signal */
#define HW_ATL2_HW_ATL2_PTM_AUTOUPDATESIGNAL_MSK 0x00000200u
/*! \brief Inverted bitmask for bitfield pif_phi_ptm_auto_update_signal */
#define HW_ATL2_HW_ATL2_PTM_AUTOUPDATESIGNAL_MSKN 0xFFFFFDFFu
/*! \brief Lower bit position of bitfield pif_phi_ptm_auto_update_signal */
#define HW_ATL2_HW_ATL2_PTM_AUTOUPDATESIGNAL_SHIFT 9
/*! \brief Width of bitfield pif_phi_ptm_auto_update_signal */
#define HW_ATL2_HW_ATL2_PTM_AUTOUPDATESIGNAL_WIDTH 1
/*! \brief Default value of bitfield pif_phi_ptm_auto_update_signal */
#define HW_ATL2_HW_ATL2_PTM_AUTOUPDATESIGNAL_DEFAULT 0x0u
/*@}*/

/*! @name PCIE pif_phi_ptm_manual_update_pulse Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "pif_phi_ptm_manual_update_pulse".
*
*   Type: R/WSC
*
*   Notes: update the PTM Requester Context
and Clock now
*
*   PORT="pif_phi_ptm_manual_update_pulse_i"
@{ */
/*! \brief Register address for bitfield pif_phi_ptm_manual_update_pulse */
#define HW_ATL2_HW_ATL2_PTM_MANUALUPDATEPULSE_ADR 0x00001530u
/*! \brief Bitmask for bitfield pif_phi_ptm_manual_update_pulse */
#define HW_ATL2_HW_ATL2_PTM_MANUALUPDATEPULSE_MSK 0x00000100u
/*! \brief Inverted bitmask for bitfield pif_phi_ptm_manual_update_pulse */
#define HW_ATL2_HW_ATL2_PTM_MANUALUPDATEPULSE_MSKN 0xFFFFFEFFu
/*! \brief Lower bit position of bitfield pif_phi_ptm_manual_update_pulse */
#define HW_ATL2_HW_ATL2_PTM_MANUALUPDATEPULSE_SHIFT 8
/*! \brief Width of bitfield pif_phi_ptm_manual_update_pulse */
#define HW_ATL2_HW_ATL2_PTM_MANUALUPDATEPULSE_WIDTH 1
/*! \brief Default value of bitfield pif_phi_ptm_manual_update_pulse */
#define HW_ATL2_HW_ATL2_PTM_MANUALUPDATEPULSE_DEFAULT 0x0u
/*@}*/

/*! @name PCIE phi_pif_ptm_clock_updated Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "phi_pif_ptm_clock_updated".
*
*   Type: RO
*
*   Notes: Indicates that the controller has updated the Local CLock
*
*   PORT="phi_pif_ptm_clock_updated_o"
@{ */
/*! \brief Register address for bitfield phi_pif_ptm_clock_updated */
#define HW_ATL2_PTMCLOCKUPDATED_ADR 0x00001530u
/*! \brief Bitmask for bitfield phi_pif_ptm_clock_updated */
#define HW_ATL2_PTMCLOCKUPDATED_MSK 0x00000004u
/*! \brief Inverted bitmask for bitfield phi_pif_ptm_clock_updated */
#define HW_ATL2_PTMCLOCKUPDATED_MSKN 0xFFFFFFFBu
/*! \brief Lower bit position of bitfield phi_pif_ptm_clock_updated */
#define HW_ATL2_PTMCLOCKUPDATED_SHIFT 2
/*! \brief Width of bitfield phi_pif_ptm_clock_updated */
#define HW_ATL2_PTMCLOCKUPDATED_WIDTH 1
/*! \brief Default value of bitfield phi_pif_ptm_clock_updated */
#define HW_ATL2_PTMCLOCKUPDATED_DEFAULT 0x0u
/*@}*/

/*! @name PCIE phi_pif_ptm_responder_rdy_to_validate Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "phi_pif_ptm_responder_rdy_to_validate".
*
*   Type: RO
*
*   Notes: PTM Responder Ready to Validate.
*
*   PORT="phi_pif_ptm_responder_rdy_to_validate_o"
@{ */
/*! \brief Register address for bitfield phi_pif_ptm_responder_rdy_to_validate */
#define HW_ATL2_PTMRESPONDERRDYTOVALIDATE_ADR 0x00001530u
/*! \brief Bitmask for bitfield phi_pif_ptm_responder_rdy_to_validate */
#define HW_ATL2_PTMRESPONDERRDYTOVALIDATE_MSK 0x00000002u
/*! \brief Inverted bitmask for bitfield phi_pif_ptm_responder_rdy_to_validate */
#define HW_ATL2_PTMRESPONDERRDYTOVALIDATE_MSKN 0xFFFFFFFDu
/*! \brief Lower bit position of bitfield phi_pif_ptm_responder_rdy_to_validate */
#define HW_ATL2_PTMRESPONDERRDYTOVALIDATE_SHIFT 1
/*! \brief Width of bitfield phi_pif_ptm_responder_rdy_to_validate */
#define HW_ATL2_PTMRESPONDERRDYTOVALIDATE_WIDTH 1
/*! \brief Default value of bitfield phi_pif_ptm_responder_rdy_to_validate */
#define HW_ATL2_PTMRESPONDERRDYTOVALIDATE_DEFAULT 0x0u
/*@}*/

/*! @name PCIE phi_pif_ptm_context_valid Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "phi_pif_ptm_context_valid".
*
*   Type: RO
*
*   Notes: Context Valid.
*
*   PORT="phi_pif_ptm_context_valid_o"
@{ */
/*! \brief Register address for bitfield phi_pif_ptm_context_valid */
#define HW_ATL2_PTMCONTEXTVALID_ADR 0x00001530u
/*! \brief Bitmask for bitfield phi_pif_ptm_context_valid */
#define HW_ATL2_PTMCONTEXTVALID_MSK 0x00000001u
/*! \brief Inverted bitmask for bitfield phi_pif_ptm_context_valid */
#define HW_ATL2_PTMCONTEXTVALID_MSKN 0xFFFFFFFEu
/*! \brief Lower bit position of bitfield phi_pif_ptm_context_valid */
#define HW_ATL2_PTMCONTEXTVALID_SHIFT 0
/*! \brief Width of bitfield phi_pif_ptm_context_valid */
#define HW_ATL2_PTMCONTEXTVALID_WIDTH 1
/*! \brief Default value of bitfield phi_pif_ptm_context_valid */
#define HW_ATL2_PTMCONTEXTVALID_DEFAULT 0x0u
/*@}*/


/*! @name PCIE ptm_local_clock_lsb[1F:0] Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "ptm_local_clock_lsb[1F:0]".
*
*   Type: RO
*
*   Notes: Local Clock value.
*
*   PORT="phi_pif_ptm_local_clock_o[31:0]"
@{ */
/*! \brief Register address for bitfield ptm_local_clock_lsb[1F:0] */
#define HW_ATL2_PTM_LOCALCLOCKLSB_ADR 0x00001540u
/*! \brief Bitmask for bitfield ptm_local_clock_lsb[1F:0] */
#define HW_ATL2_PTM_LOCALCLOCKLSB_MSK 0xFFFFFFFFu
/*! \brief Inverted bitmask for bitfield ptm_local_clock_lsb[1F:0] */
#define HW_ATL2_PTM_LOCALCLOCKLSB_MSKN 0x00000000u
/*! \brief Lower bit position of bitfield ptm_local_clock_lsb[1F:0] */
#define HW_ATL2_PTM_LOCALCLOCKLSB_SHIFT 0
/*! \brief Width of bitfield ptm_local_clock_lsb[1F:0] */
#define HW_ATL2_PTM_LOCALCLOCKLSB_WIDTH 32
/*! \brief Default value of bitfield ptm_local_clock_lsb[1F:0] */
#define HW_ATL2_PTM_LOCALCLOCKLSB_DEFAULT 0x0u
/*@}*/


/*! @name PCIE ptm_local_clock_msb[1F:0] Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "ptm_local_clock_msb[1F:0]".
*
*   Type: RO
*
*   Notes: Local Clock value.
*
*   PORT="phi_pif_ptm_local_clock_o[63:32]"
@{ */
/*! \brief Register address for bitfield ptm_local_clock_msb[1F:0] */
#define HW_ATL2_PTM_LOCALCLOCKMSB_ADR 0x00001544u
/*! \brief Bitmask for bitfield ptm_local_clock_msb[1F:0] */
#define HW_ATL2_PTM_LOCALCLOCKMSB_MSK 0xFFFFFFFFu
/*! \brief Inverted bitmask for bitfield ptm_local_clock_msb[1F:0] */
#define HW_ATL2_PTM_LOCALCLOCKMSB_MSKN 0x00000000u
/*! \brief Lower bit position of bitfield ptm_local_clock_msb[1F:0] */
#define HW_ATL2_PTM_LOCALCLOCKMSB_SHIFT 0
/*! \brief Width of bitfield ptm_local_clock_msb[1F:0] */
#define HW_ATL2_PTM_LOCALCLOCKMSB_WIDTH 32
/*! \brief Default value of bitfield ptm_local_clock_msb[1F:0] */
#define HW_ATL2_PTM_LOCALCLOCKMSB_DEFAULT 0x0u
/*@}*/


/*! @name PCIE ptm_clock_correction_lsb[31:0] Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "ptm_clock_correction_lsb[31:0]".
*
*   Type: RO
*
*   Notes: Amount by which Local Clock has been corrected.
*
*   PORT="phi_pif_ptm_clock_correction_o[31:0]"
@{ */
/*! \brief Register address for bitfield ptm_clock_correction_lsb[31:0] */
#define HW_ATL2_PTM_CLOCKCORRECTIONLSB_ADR 0x00001548u
/*! \brief Bitmask for bitfield ptm_clock_correction_lsb[31:0] */
#define HW_ATL2_PTM_CLOCKCORRECTIONLSB_MSK 0xFFFFFFFFu
/*! \brief Inverted bitmask for bitfield ptm_clock_correction_lsb[31:0] */
#define HW_ATL2_PTM_CLOCKCORRECTIONLSB_MSKN 0x00000000u
/*! \brief Lower bit position of bitfield ptm_clock_correction_lsb[31:0] */
#define HW_ATL2_PTM_CLOCKCORRECTIONLSB_SHIFT 0
/*! \brief Width of bitfield ptm_clock_correction_lsb[31:0] */
#define HW_ATL2_PTM_CLOCKCORRECTIONLSB_WIDTH 32
/*! \brief Default value of bitfield ptm_clock_correction_lsb[31:0] */
#define HW_ATL2_PTM_CLOCKCORRECTIONLSB_DEFAULT 0x0u
/*@}*/


/*! @name PCIE ptm_clock_correction_msb[31:0] Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "ptm_clock_correction_msb[31:0]".
*
*   Type: RO
*
*   Notes: Amount by which Local Clock has been corrected.
*
*   PORT="phi_pif_ptm_clock_correction_o[63:32]"
@{ */
/*! \brief Register address for bitfield ptm_clock_correction_msb[31:0] */
#define HW_ATL2_PTM_CLOCKCORRECTIONMSB_ADR 0x0000154Cu
/*! \brief Bitmask for bitfield ptm_clock_correction_msb[31:0] */
#define HW_ATL2_PTM_CLOCKCORRECTIONMSB_MSK 0xFFFFFFFFu
/*! \brief Inverted bitmask for bitfield ptm_clock_correction_msb[31:0] */
#define HW_ATL2_PTM_CLOCKCORRECTIONMSB_MSKN 0x00000000u
/*! \brief Lower bit position of bitfield ptm_clock_correction_msb[31:0] */
#define HW_ATL2_PTM_CLOCKCORRECTIONMSB_SHIFT 0
/*! \brief Width of bitfield ptm_clock_correction_msb[31:0] */
#define HW_ATL2_PTM_CLOCKCORRECTIONMSB_WIDTH 32
/*! \brief Default value of bitfield ptm_clock_correction_msb[31:0] */
#define HW_ATL2_PTM_CLOCKCORRECTIONMSB_DEFAULT 0x0u
/*@}*/


/*! @name PCIE ptm_update_ctr[1F:0] Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "ptm_update_ctr[1F:0]".
*
*   Type: R/W
*
*   Notes: This Timer should be used when a periodic update of PTM clock is triggered based on the clock count value loaded to this register. The clock count value to be loaded is the total number of clocks to be waited before a PTM clock update is triggered. Clock is counted using a 156.25MHz clock.
*
*   PORT="pif_phi_ptm_update_ctr_i[31:0]"
@{ */
/*! \brief Register address for bitfield ptm_update_ctr[1F:0] */
#define HW_ATL2_PTM_UPDATECTR_ADR 0x00001550u
/*! \brief Bitmask for bitfield ptm_update_ctr[1F:0] */
#define HW_ATL2_PTM_UPDATECTR_MSK 0xFFFFFFFFu
/*! \brief Inverted bitmask for bitfield ptm_update_ctr[1F:0] */
#define HW_ATL2_PTM_UPDATECTR_MSKN 0x00000000u
/*! \brief Lower bit position of bitfield ptm_update_ctr[1F:0] */
#define HW_ATL2_PTM_UPDATECTR_SHIFT 0
/*! \brief Width of bitfield ptm_update_ctr[1F:0] */
#define HW_ATL2_PTM_UPDATECTR_WIDTH 32
/*! \brief Default value of bitfield ptm_update_ctr[1F:0] */
#define HW_ATL2_PTM_UPDATECTR_DEFAULT 0x0u
/*@}*/


/*! @name PCIE ptm_ctxt_id[7:0] Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "ptm_ctxt_id[7:0]".
*
*   Type: RO
*
*   Notes: This field indicates the PTM Context ID which is incremented every time a context is invalidated by the Synopsys Core through the de-assertion of ptm_context_valid signal. 
*
*   PORT="phi_pif_ptm_ctxt_id_o[7:0]"
@{ */
/*! \brief Register address for bitfield ptm_ctxt_id[7:0] */
#define HW_ATL2_PTM_CTXTID_ADR 0x00001554u
/*! \brief Bitmask for bitfield ptm_ctxt_id[7:0] */
#define HW_ATL2_PTM_CTXTID_MSK 0x0000FF00u
/*! \brief Inverted bitmask for bitfield ptm_ctxt_id[7:0] */
#define HW_ATL2_PTM_CTXTID_MSKN 0xFFFF00FFu
/*! \brief Lower bit position of bitfield ptm_ctxt_id[7:0] */
#define HW_ATL2_PTM_CTXTID_SHIFT 8
/*! \brief Width of bitfield ptm_ctxt_id[7:0] */
#define HW_ATL2_PTM_CTXTID_WIDTH 8
/*! \brief Default value of bitfield ptm_ctxt_id[7:0] */
#define HW_ATL2_PTM_CTXTID_DEFAULT 0x0u
/*@}*/

/*! @name PCIE ptm_seq_id[7:0] Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "ptm_seq_id[7:0]".
*
*   Type: RO
*
*   Notes: This field indicates the PTM Sequence ID which is incremented for every PTM Clock update pulse received by the Synopsys Core which is an indication of successful PTM clock update.
*
*   PORT="phi_pif_ptm_seq_id_o[7:0]"
@{ */
/*! \brief Register address for bitfield ptm_seq_id[7:0] */
#define HW_ATL2_PTM_SEQID_ADR 0x00001554u
/*! \brief Bitmask for bitfield ptm_seq_id[7:0] */
#define HW_ATL2_PTM_SEQID_MSK 0x000000FFu
/*! \brief Inverted bitmask for bitfield ptm_seq_id[7:0] */
#define HW_ATL2_PTM_SEQID_MSKN 0xFFFFFF00u
/*! \brief Lower bit position of bitfield ptm_seq_id[7:0] */
#define HW_ATL2_PTM_SEQID_SHIFT 0
/*! \brief Width of bitfield ptm_seq_id[7:0] */
#define HW_ATL2_PTM_SEQID_WIDTH 8
/*! \brief Default value of bitfield ptm_seq_id[7:0] */
#define HW_ATL2_PTM_SEQID_DEFAULT 0x0u
/*@}*/

/*! @name PCIE phi_pif_ptm_tsg_local_clock_lower Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "phi_pif_ptm_tsg_local_clock_lower".
*
*   Type: RO
*
*   Notes: Specifies PTM Clock Lower Value latched when PTM clock is synced from the host. This value should be read for calculating the drift.
*
*   PORT="phi_pif_ptm_tsg_local_clock_o[31:0]"
@{ */
/*! \brief Register address for bitfield phi_pif_ptm_tsg_local_clock_lower */
#define HW_ATL2_PTMTSGLOCALCLOCKLOWER_ADR 0x00001580u
/*! \brief Bitmask for bitfield phi_pif_ptm_tsg_local_clock_lower */
#define HW_ATL2_PTMTSGLOCALCLOCKLOWER_MSK 0xFFFFFFFFu
/*! \brief Inverted bitmask for bitfield phi_pif_ptm_tsg_local_clock_lower */
#define HW_ATL2_PTMTSGLOCALCLOCKLOWER_MSKN 0x00000000u
/*! \brief Lower bit position of bitfield phi_pif_ptm_tsg_local_clock_lower */
#define HW_ATL2_PTMTSGLOCALCLOCKLOWER_SHIFT 0
/*! \brief Width of bitfield phi_pif_ptm_tsg_local_clock_lower */
#define HW_ATL2_PTMTSGLOCALCLOCKLOWER_WIDTH 32
/*! \brief Default value of bitfield phi_pif_ptm_tsg_local_clock_lower */
#define HW_ATL2_PTMTSGLOCALCLOCKLOWER_DEFAULT 0x0u
/*@}*/


/*! @name PCIE phi_pif_ptm_tsg_local_clock_upper Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "phi_pif_ptm_tsg_local_clock_upper".
*
*   Type: RO
*
*   Notes: Specifies PTM Clock Upper Value latched when PTM clock is synced from the host. This value should be read for calculating the drift.
*
*   PORT="phi_pif_ptm_tsg_local_clock_o[63:32]"
@{ */
/*! \brief Register address for bitfield phi_pif_ptm_tsg_local_clock_upper */
#define HW_ATL2_PTMTSGLOCALCLOCKUPPER_ADR 0x00001584u
/*! \brief Bitmask for bitfield phi_pif_ptm_tsg_local_clock_upper */
#define HW_ATL2_PTMTSGLOCALCLOCKUPPER_MSK 0xFFFFFFFFu
/*! \brief Inverted bitmask for bitfield phi_pif_ptm_tsg_local_clock_upper */
#define HW_ATL2_PTMTSGLOCALCLOCKUPPER_MSKN 0x00000000u
/*! \brief Lower bit position of bitfield phi_pif_ptm_tsg_local_clock_upper */
#define HW_ATL2_PTMTSGLOCALCLOCKUPPER_SHIFT 0
/*! \brief Width of bitfield phi_pif_ptm_tsg_local_clock_upper */
#define HW_ATL2_PTMTSGLOCALCLOCKUPPER_WIDTH 32
/*! \brief Default value of bitfield phi_pif_ptm_tsg_local_clock_upper */
#define HW_ATL2_PTMTSGLOCALCLOCKUPPER_DEFAULT 0x0u
/*@}*/


/*! @name PCIE phi_pif_ptp_tsg_local_clock_low Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "phi_pif_ptp_tsg_local_clock_low".
*
*   Type: R/W
*
*   Notes: Specifies PTP Clock Lower Value latched when PTM clock is synced from the host. This value should be read for calculating the drift.
*
*   PORT="phi_pif_ptp_tsg_local_clock_o[31:0]"
@{ */
/*! \brief Register address for bitfield phi_pif_ptp_tsg_local_clock_low */
#define HW_ATL2_PTPTSGLOCALCLOCKLOW_ADR 0x00001588u
/*! \brief Bitmask for bitfield phi_pif_ptp_tsg_local_clock_low */
#define HW_ATL2_PTPTSGLOCALCLOCKLOW_MSK 0xFFFFFFFFu
/*! \brief Inverted bitmask for bitfield phi_pif_ptp_tsg_local_clock_low */
#define HW_ATL2_PTPTSGLOCALCLOCKLOW_MSKN 0x00000000u
/*! \brief Lower bit position of bitfield phi_pif_ptp_tsg_local_clock_low */
#define HW_ATL2_PTPTSGLOCALCLOCKLOW_SHIFT 0
/*! \brief Width of bitfield phi_pif_ptp_tsg_local_clock_low */
#define HW_ATL2_PTPTSGLOCALCLOCKLOW_WIDTH 32
/*! \brief Default value of bitfield phi_pif_ptp_tsg_local_clock_low */
#define HW_ATL2_PTPTSGLOCALCLOCKLOW_DEFAULT 0x0u
/*@}*/


/*! @name PCIE phi_pif_ptp_tsg_local_clock_upper Bitfield Definitions
*
*   Preprocessor definitions for the bitfield "phi_pif_ptp_tsg_local_clock_upper".
*
*   Type: R/W
*
*   Notes: Specifies PTP Clock Upper Value latched when PTM clock is synced from the host. This value should be read for calculating the drift.
*
*   PORT="phi_pif_ptp_tsg_local_clock_o[63:32]"
@{ */
/*! \brief Register address for bitfield phi_pif_ptp_tsg_local_clock_upper */
#define HW_ATL2_PTPTSGLOCALCLOCKUPPER_ADR 0x0000158Cu
/*! \brief Bitmask for bitfield phi_pif_ptp_tsg_local_clock_upper */
#define HW_ATL2_PTPTSGLOCALCLOCKUPPER_MSK 0xFFFFFFFFu
/*! \brief Inverted bitmask for bitfield phi_pif_ptp_tsg_local_clock_upper */
#define HW_ATL2_PTPTSGLOCALCLOCKUPPER_MSKN 0x00000000u
/*! \brief Lower bit position of bitfield phi_pif_ptp_tsg_local_clock_upper */
#define HW_ATL2_PTPTSGLOCALCLOCKUPPER_SHIFT 0
/*! \brief Width of bitfield phi_pif_ptp_tsg_local_clock_upper */
#define HW_ATL2_PTPTSGLOCALCLOCKUPPER_WIDTH 32
/*! \brief Default value of bitfield phi_pif_ptp_tsg_local_clock_upper */
#define HW_ATL2_PTPTSGLOCALCLOCKUPPER_DEFAULT 0x0u
/*@}*/

#define HW_ATL2_FW_SM_ACT_RSLVR  0x3U

#endif /* HW_ATL2_LLH_INTERNAL_H */
