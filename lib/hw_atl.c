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

#include <time.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/mman.h>
#include <sys/user.h>
#include <sys/stat.h>
#include <stdint.h>
#include <semaphore.h>
#include <pthread.h>

#include "list.h"
#include "atl_private.h"

#include "atl_llh.h"

#define ATL_AVBTS_RING        24

CREATE_RING(tx, TX);
CREATE_RING(rx, RX);
ATTACH_RING(tx, TX);
ATTACH_RING(rx, RX);

inline static u32 macBitRead( void *context, u32 address, u32 mask, u32 shift )
{
    u32 regValue = 0xffffffff;
	if (rr((struct atl_adapter *)context, address, &regValue) )
		return 0xffffffff;
    return (regValue & mask) >> shift;
}

inline static void macBitWrite( void *context, u32 address, u32 mask, u32 shift, u32 data )
{
    u32 regValue = 0xffffffff;
	if (rr((struct atl_adapter *)context, address, &regValue) )
		return;
	regValue &= ~mask;
	regValue |= (data << shift) & mask;
	wr((struct atl_adapter *)context, address, regValue);
}

static void tdmTxDescriptorCacheInitializationSet(void *context, u32 txDescriptorCacheInitialization)
{
    macBitWrite(context, HW_ATL_TDM_DESC_CACHE_INIT_ADR, 
            HW_ATL_TDM_DESC_CACHE_MSK, 
            HW_ATL_TDM_DESC_CACHE_SHIFT, 
            txDescriptorCacheInitialization);
}

static void rdmRxDescriptorCacheInitializationSet(void *context, u32 rxDescriptorCacheInitialization)
{
    macBitWrite(context, HW_ATL_RDM_DESC_INIT_ADR, 
            HW_ATL_RDM_DESC_INIT_MSK, 
            HW_ATL_RDM_DESC_INIT_SHIFT, 
            rxDescriptorCacheInitialization);
}

void ptpPtpDigitalClockReadEnableSet(void *context, u32 ptpDigitalClockReadEnable)
{
	macBitWrite(context, HW_ATL_PCS_PTP_CLOCK_READ_ENABLE_ADR,
			HW_ATL_PCS_PTP_CLOCK_READ_ENABLE_MSK,
			HW_ATL_PCS_PTP_CLOCK_READ_ENABLE_SHIFT,
			ptpDigitalClockReadEnable);
}

static void hw_atl_b0_tx_avb_ring_hpwb_en(struct tx_ring *aq_ring)
{
	u64 hpwb_paddr;
	aq_ring->size--;
	aq_ring->empty--;
	hpwb_paddr = aq_ring->dma_paddr + aq_ring->size * sizeof(union atl_tx_desc_u);
	aq_ring->ctrl->hpwb_addr_lsw = (u32)((hpwb_paddr >> 0) & 0xffffffff);
	aq_ring->ctrl->hpwb_addr_msw = (u32)((hpwb_paddr >> 32) & 0xffffffff);
	aq_ring->ctrl->ctrl_len |= BIT(RING_CTRL_HEAD_WB_BIT);
	aq_ring->flags |= TX_RING_FLAG_HEAD_WB;
}

static int hw_atl_b0_latch_clock(struct atl_adapter *adapter)
{
	ptpPtpDigitalClockReadEnableSet(adapter, 1);
	return 0;	
}

static int hw_atl_b0_get_clock(struct atl_adapter *adapter, u64 *ts)
{
	u32 lns = 0, hns = 0, ls = 0, hs = 0;
	u32 adjl = 0, adjh = 0;
	ptpPtpDigitalClockReadEnableSet(adapter, 0);
	rr(adapter, HW_ATL_PCS_PTP_TS_VAL_ADDR(0), &ls);
	rr(adapter, HW_ATL_PCS_PTP_TS_VAL_ADDR(1), &hs);
	rr(adapter, HW_ATL_PCS_PTP_TS_VAL_ADDR(3), &lns);
	rr(adapter, HW_ATL_PCS_PTP_TS_VAL_ADDR(4), &hns);
	rr(adapter, HW_ATL_GLB_CPU_SCRPAD2_SCP_ADR(0), &adjl);
	rr(adapter, HW_ATL_GLB_CPU_SCRPAD2_SCP_ADR(1), &adjh);
	if( ts ) *ts = ((((u64)hs)<<16) + (u64)ls)*1000000000 + (((u64)hns)<<16) + (u64)lns + ((u64)adjh << 32) + adjl;
	return 0;
}

static int hw_atl_b0_avb_ring_tx_init(struct tx_ring *aq_ring, u32 flags)
{
	struct atl_dma_alloc *dma;
	int error, ring_size;
	struct atl_adapter *adapter = aq_ring->adapter;
	struct tx_ring *private_aq_ringing = NULL;
	// Allocate place for AVB TS packets
	dma = malloc(sizeof(*dma));
	if (dma == NULL) {
		goto release;
	}
	memset(dma, 0, sizeof(*dma));
	dma->mmap_size = ATL_AVB_RING_SIZE * sizeof(struct atl_fw_ts_packet);
	error = atl_dma_alloc_buffer(adapter, dma);
	if (error) {
		goto release;
	}

	// Allocate special ring for AVB TS
	private_aq_ringing = malloc(sizeof(struct tx_ring));
	if (private_aq_ringing == NULL)
		goto release;

	memset(private_aq_ringing, 0, sizeof(struct tx_ring));
	private_aq_ringing->adapter = adapter;
	private_aq_ringing->private_ring_data = dma;

	/* Allocate and Setup tx_ring */
	ring_size = ATL_AVB_RING_SIZE + (flags & TX_RING_FLAG_HEAD_WB ? 1 : 0);
	error = create_tx_ring(private_aq_ringing, ring_size, ATL_AVBTS_RING);
	if (error)
		goto release;

	if( flags & TX_RING_FLAG_HEAD_WB ) {
		hw_atl_b0_tx_avb_ring_hpwb_en(private_aq_ringing);
	}
	private_aq_ringing->flags |= flags & TX_RING_FLAG_WRITEBACK_EOP;

	aq_ring->private_ring_data = private_aq_ringing;
    error = create_tx_ring(aq_ring, ring_size, aq_ring->index);
	if (error) {
		goto release;
	}

	if( flags & TX_RING_FLAG_HEAD_WB ) {
		hw_atl_b0_tx_avb_ring_hpwb_en(aq_ring);
	}
	aq_ring->flags |= flags & TX_RING_FLAG_WRITEBACK_EOP;

	return 0;
release:
	if ( private_aq_ringing ){
		free(private_aq_ringing);
	}	

    if( dma ){
		if( dma->dma_vaddr ){
	        atl_dma_free_buffer(adapter, dma);
		};
		free(dma);
    }

	return error;
}

static int hw_atl_b0_avb_ring_rx_init(struct rx_ring *aq_ring, u32 flags)
{
    int error = create_rx_ring(aq_ring, ATL_AVB_RING_SIZE, aq_ring->index);
	if (!error) {
		aq_ring->flags = flags; //RX_RING_FLAG_SPLIT_HEADER
		aq_ring->ctrl->sizes = ATL_AVB_BUFFER_SIZE >> 10;
	}
    
    return error;
}

static void tx_ring_predisable_check(struct tx_ring *aq_ring){
	if( aq_ring->shead != aq_ring->tail ){
		int i = 0;
		while( (aq_ring->ctrl->head & 0xffff) != aq_ring->ctrl->tail ||
				aq_ring->ctrl->tail != aq_ring->tail ) {
			if( i++ > 10000u ) {
				logprint(LOG_LVL_WARNING, "Ring %d. HW head %x and tail %x mismatched. Waiting. Or driver must be reloaded!", 
						aq_ring->index, aq_ring->ctrl->head, aq_ring->ctrl->tail);
				i = 0;
			}
		}	
	}
}

static int hw_atl_b0_avb_ring_tx_disable(struct tx_ring *aq_ring)
{
	struct atl_adapter *adapter = aq_ring->adapter;
	if ( aq_ring->private_ring_data ) {
		struct tx_ring *ts_ring = (struct tx_ring *)aq_ring->private_ring_data;
		tx_ring_predisable_check(ts_ring);
		ts_ring->ctrl->ctrl_len &= ~BIT(RING_CTRL_ENABLE_BIT);// Disable ring
		if( ts_ring->private_ring_data ){
			free(ts_ring->private_ring_data);
		}
		free(ts_ring);
		aq_ring->private_ring_data = NULL;
	}

	tx_ring_predisable_check(aq_ring);
    aq_ring->ctrl->ctrl_len &= ~BIT(RING_CTRL_ENABLE_BIT); // Disable ring
	return 0;
}

static int hw_atl_b0_avb_ring_rx_disable(struct rx_ring *aq_ring)
{
    aq_ring->ctrl->ctrl_len &= ~BIT(RING_CTRL_ENABLE_BIT); // Disable ring
	return 0;
}

static int hw_atl_b0_avb_ring_tx_reset(struct tx_ring *aq_ring)
{
	if ( aq_ring->private_ring_data ) {
		struct tx_ring *ts_ring = (struct tx_ring *)aq_ring->private_ring_data;
		tx_ring_predisable_check(ts_ring);
		ts_ring->ctrl->ctrl_len &= ~BIT(RING_CTRL_ENABLE_BIT);// Disable ring
		ts_ring->ctrl->ctrl_len |= BIT(RING_CTRL_RESET_BIT);// Reset ring
	}
	tx_ring_predisable_check(aq_ring);
    aq_ring->ctrl->ctrl_len &= ~BIT(RING_CTRL_ENABLE_BIT); // Disable ring
    aq_ring->ctrl->ctrl_len |= BIT(RING_CTRL_RESET_BIT); // Reset ring

	//Clear prefetched descriptors
	//tdmTxDescriptorCacheInitializationSet(aq_ring->adapter, 1);
	//tdmTxDescriptorCacheInitializationSet(aq_ring->adapter, 0);
    aq_ring->tail = aq_ring->shead = aq_ring->hhead = aq_ring->ctrl->tail = aq_ring->ctrl->head & 0xffff;
    aq_ring->ctrl->ctrl_len &= ~BIT(RING_CTRL_RESET_BIT); // Reset ring
    aq_ring->ctrl->ctrl_len |= BIT(RING_CTRL_ENABLE_BIT); // Enable ring
	if ( aq_ring->private_ring_data ) {
		struct tx_ring *ts_ring = (struct tx_ring *)aq_ring->private_ring_data;
	    ts_ring->tail = ts_ring->shead = ts_ring->hhead = ts_ring->ctrl->tail = ts_ring->ctrl->head & 0xffff;
		ts_ring->ctrl->ctrl_len &= ~BIT(RING_CTRL_RESET_BIT);// Reset ring

		ts_ring->ctrl->ctrl_len |= BIT(RING_CTRL_ENABLE_BIT);// Enable ring
	}
	return 0;
}

static int hw_atl_b0_avb_ring_rx_reset(struct rx_ring *aq_ring)
{
    aq_ring->ctrl->ctrl_len &= ~BIT(RING_CTRL_ENABLE_BIT); // Disable ring
    aq_ring->ctrl->ctrl_len |= BIT(RING_CTRL_RESET_BIT); // Reset ring
	//Clear prefetched descriptors
	//rdmRxDescriptorCacheInitializationSet(aq_ring->adapter, 1);
	//rdmRxDescriptorCacheInitializationSet(aq_ring->adapter, 0);

    aq_ring->tail = aq_ring->shead = aq_ring->hhead = aq_ring->ctrl->tail = aq_ring->ctrl->head & 0xffff;
    aq_ring->ctrl->ctrl_len &= ~BIT(RING_CTRL_RESET_BIT); // Reset ring
    aq_ring->ctrl->ctrl_len |= BIT(RING_CTRL_ENABLE_BIT); // Enable ring
	return 0;
}

static int hw_atl_b0_ring_tx_attach(struct tx_ring *aq_ring, u32 flags)
{
	struct atl_dma_alloc *dma;
	int error, ring_size;
	struct atl_adapter *adapter = aq_ring->adapter;
	struct tx_ring *private_aq_ringing = NULL;
	// Allocate place for AVB TS packets
	dma = malloc(sizeof(*dma));
	if (dma == NULL) {
		goto release;
	}
	memset(dma, 0, sizeof(*dma));
	dma->mmap_size = ATL_AVB_RING_SIZE * sizeof(struct atl_fw_ts_packet);
	error = atl_dma_alloc_buffer(adapter, dma);
	if (error) {
		goto release;
	}

	// Allocate special ring for AVB TS
	private_aq_ringing = malloc(sizeof(struct tx_ring));
	if (private_aq_ringing == NULL)
		goto release;

	memset(private_aq_ringing, 0, sizeof(struct tx_ring));
	private_aq_ringing->adapter = adapter;
	private_aq_ringing->private_ring_data = dma;

	/* Allocate and Setup tx_ring */
	ring_size = ATL_AVB_RING_SIZE + (flags & TX_RING_FLAG_HEAD_WB ? 1 : 0);
	error = attach_tx_ring(private_aq_ringing, ring_size, ATL_AVBTS_RING);
	if (error)
		goto release;

	if( flags & TX_RING_FLAG_HEAD_WB ) {
		hw_atl_b0_tx_avb_ring_hpwb_en(private_aq_ringing);
	}
	private_aq_ringing->flags |= flags & TX_RING_FLAG_WRITEBACK_EOP;

	aq_ring->private_ring_data = private_aq_ringing;
    error = attach_tx_ring(aq_ring, ring_size, aq_ring->index);
	if (error) {
		goto release;
	}

	if( flags & TX_RING_FLAG_HEAD_WB ) {
		hw_atl_b0_tx_avb_ring_hpwb_en(aq_ring);
	}
	aq_ring->flags |= flags & TX_RING_FLAG_WRITEBACK_EOP;

	return 0;
release:
	if ( private_aq_ringing ){
		free(private_aq_ringing);
	}	

    if( dma ){
		if( dma->dma_vaddr ){
	        atl_dma_free_buffer(adapter, dma);
		};
		free(dma);
    }

	return error;
}

static int hw_atl_b0_ring_rx_attach(struct rx_ring *aq_ring, u32 flags)
{
    int error = attach_rx_ring(aq_ring, ATL_AVB_RING_SIZE, aq_ring->index);
	if (!error) {
		aq_ring->flags = flags; //RX_RING_FLAG_SPLIT_HEADER
	}
    
    return error;
}

static int hw_atl_b0_ring_tx_detach(struct tx_ring *aq_ring)
{
	struct atl_adapter *adapter = aq_ring->adapter;
	struct atl_dma_alloc dma = {0};
	if ( aq_ring->private_ring_data ) {
		struct tx_ring *ts_ring = (struct tx_ring *)aq_ring->private_ring_data;
		if( ts_ring->private_ring_data ){
			free(ts_ring->private_ring_data);
		}
		dma.dma_paddr = ts_ring->dma_paddr;
		dma.dma_vaddr = ts_ring->desc;
		dma.mmap_size = (ATL_AVB_RING_SIZE + (ts_ring->flags & TX_RING_FLAG_HEAD_WB ? 1 : 0)) * sizeof(union atl_tx_desc_u);

		atl_dma_detach_buffer(adapter, &dma);
		free(ts_ring);
		aq_ring->private_ring_data = NULL;
	}
	dma.dma_paddr = aq_ring->dma_paddr;
	dma.dma_vaddr = aq_ring->desc;
	dma.mmap_size = (ATL_AVB_RING_SIZE + (aq_ring->flags & TX_RING_FLAG_HEAD_WB ? 1 : 0)) * sizeof(union atl_tx_desc_u);;

	atl_dma_detach_buffer(adapter, &dma);
	aq_ring->private_ring_data = NULL;
	return 0;
}

static int hw_atl_b0_ring_rx_detach(struct rx_ring *aq_ring)
{
	struct atl_adapter *adapter = aq_ring->adapter;
	struct atl_dma_alloc dma = {0};
	dma.dma_paddr = aq_ring->dma_paddr;
	dma.dma_vaddr = aq_ring->desc;
	dma.mmap_size = (ATL_AVB_RING_SIZE + (aq_ring->flags & TX_RING_FLAG_HEAD_WB ? 1 : 0)) * sizeof(union atl_rx_desc_u);

	atl_dma_detach_buffer(adapter, &dma);
	return 0;
}

static union atl_tx_desc_u *get_empty_tx_desc(struct tx_ring *aq_ring)
{
	union atl_tx_desc_u *txd = NULL;
	if (aq_ring->empty > 1) {
		txd = &aq_ring->desc[aq_ring->tail++];
		if( aq_ring->tail >= aq_ring->size ) aq_ring->tail = 0;
		aq_ring->empty--;
	} else {
		logprint(LOG_LVL_ERROR, "No more empty descs. tail %x", aq_ring->tail);
	}
	return txd;
}

/* Context Descriptor setup for VLAN or CSUM */
static void hw_atl_b0_tx_ctx_setup(struct tx_ring *aq_ring, struct atl_packet *packet)
{
	struct atl_adapter *adapter = aq_ring->adapter;
	union atl_tx_desc_u *txd;
	int ct_idx = 0;

	if( !packet->vlan )
		return;
		
	txd = get_empty_tx_desc(aq_ring);

	/* Now copy bits into descriptor */
	txd->ctx.vlan_tag = packet->vlan;
	txd->ctx.mss_len = 0;
	txd->ctx.ct_idx = ct_idx;
	packet->ct_idx = ct_idx;
}

static int hw_atl_b0_avb_ring_tx_prepare_xmit(struct tx_ring *aq_ring, struct atl_packet *packet)
{
	union atl_tx_desc_u *txd;
	struct tx_desc_ptr *d_ptr = (struct tx_desc_ptr *)&packet->private;
	
	d_ptr->first = aq_ring->tail;
	/*
	 * Set up the context descriptor to specify
	 * VLAN insertion
	 */
	hw_atl_b0_tx_ctx_setup(aq_ring, packet);
	
	d_ptr->last = aq_ring->tail;
	txd = get_empty_tx_desc(aq_ring);
	memset(txd, 0, sizeof(*txd));

	txd->dsc.addr = htole64(packet->map.paddr + packet->offset);
	txd->dsc.pay_len = htole32(packet->len);
	/* we assume every packet is contiguous one packet - one buffer*/
	txd->dsc.buf_len = htole32(packet->len);
	txd->dsc.desc_type = TX_PACKET_DESC_TYPE;
	txd->dsc.eop = 1;
	txd->dsc.tx_cmd = TX_CMD_MAC_FCS | (aq_ring->flags & TX_RING_FLAG_WRITEBACK_EOP ? TX_CMD_WB_EN : 0);
	if( packet->vlan ){
		txd->dsc.ct_en = 1;
		txd->dsc.ct_idx = packet->ct_idx;
		txd->dsc.tx_cmd |= TX_CMD_VLAN_INSERT;
	}
	dump(LOG_LVL_DEBUG, "AVB TX Desc", (uint8_t *)txd, sizeof(*txd));

	if ( aq_ring->private_ring_data ) {
		u8 dst[6] = {0xde, 0xfe, 0xc3, 0x00, 0x00, 0x00};
		struct tx_ring *ts_ring = (struct tx_ring *)aq_ring->private_ring_data;
		u32 tsdi = ts_ring->tail;
		struct atl_dma_alloc *ts_pkt_dma = (struct atl_dma_alloc *)ts_ring->private_ring_data;
		struct atl_fw_ts_packet *ts_pkt = (struct atl_fw_ts_packet *)(((uint8_t *)ts_pkt_dma->dma_vaddr) + tsdi*sizeof(struct atl_fw_ts_packet));
		union atl_tx_desc_u *tsd;

		d_ptr->pr_first = d_ptr->pr_last = ts_ring->tail;
		tsd = get_empty_tx_desc(ts_ring);
		memset(tsd, 0, sizeof(*tsd));
		tsd->dsc.addr = htole64(ts_pkt_dma->dma_paddr + tsdi*sizeof(struct atl_fw_ts_packet));
		tsd->dsc.pay_len = htole32(sizeof(struct atl_fw_ts_packet));
		tsd->dsc.buf_len = htole32(sizeof(struct atl_fw_ts_packet));
		tsd->dsc.desc_type = TX_PACKET_DESC_TYPE;
		tsd->dsc.eop = 1;
		tsd->dsc.tx_cmd = TX_CMD_MAC_FCS | (aq_ring->flags & TX_RING_FLAG_WRITEBACK_EOP ? TX_CMD_WB_EN : 0);
		memcpy(ts_pkt->dst, dst, sizeof(dst));
		ts_pkt->ring = aq_ring->index;
		ts_pkt->ts_cnt = 1;
		ts_pkt->ts_link[0].ts = packet->attime; 
		ts_pkt->ts_link[0].last_desc = d_ptr->last;
		ts_pkt->ts_link[0].pkt_len = packet->len;

		dump(LOG_LVL_DEBUG2, "AVBTS TX Packet", (uint8_t *)ts_pkt, sizeof(*ts_pkt));
		dump(LOG_LVL_DEBUG, "AVBTS TX Desc", (uint8_t *)tsd, sizeof(*tsd));
		ts_ring->ctrl->tail = ts_ring->tail;
		//logprint(LOG_LVL_DEBUG2, "ts_ring tail %x ctrl tail %x ctrl head %x", ts_ring->tail, ts_ring->ctrl->tail, ts_ring->ctrl->head & 0xffff);
	}
	else {
		aq_ring->ctrl->tail = aq_ring->tail;
	}
	logprint(LOG_LVL_DEBUG2, "tx b0 xmit aq_ring tail %x shead %x hhead %x ctrl tail %x ctrl head %x", aq_ring->tail, aq_ring->shead, aq_ring->hhead, aq_ring->ctrl->tail, aq_ring->ctrl->head & 0xffff);
}

static bool atl_b0_tx_desc_complete(struct tx_ring *aq_ring, u16 sop_idx, u16 eop_idx)
{
	u32 hhead, shead = aq_ring->shead, ring_size = aq_ring->size;
	u32 chk_desc = eop_idx + 1 + (eop_idx < shead ? ring_size : 0);
	u32 tail = aq_ring->tail + (aq_ring->tail < shead ? ring_size : 0);

	if( aq_ring->flags & TX_RING_FLAG_HEAD_WB ) { 
		hhead = ((u32 *)(((uint8_t *)aq_ring->desc) + ring_size * sizeof(union atl_tx_desc_u)))[0];
	} else {
		union atl_tx_desc_u *ts_desc, *eop_desc;
		eop_desc = &aq_ring->desc[eop_idx];
		hhead = eop_desc->wb.dd ? chk_desc : shead;
	}
	if( hhead < shead ) hhead += ring_size;
	if( hhead > tail || chk_desc < shead || chk_desc > tail ) {
		logprint(LOG_LVL_DEBUG2, "Ring %d. Flags %x. Wrong value of head %x/%x, tail %x and packet descriptor index %x", 
				aq_ring->index, aq_ring->flags, shead, hhead, aq_ring->tail, eop_idx);
	}
	if( chk_desc <= hhead ) {
		aq_ring->shead = chk_desc - (chk_desc >= ring_size ? ring_size : 0);
		aq_ring->empty += eop_idx + (eop_idx < sop_idx ? aq_ring->size : 0) - sop_idx + 1;
		logprint(LOG_LVL_DEBUG2, "DMA Complete! Desc %x. Empty %x", eop_idx, aq_ring->empty);
	}

	aq_ring->hhead = hhead - (hhead >= ring_size ? ring_size : 0);
	return chk_desc <= hhead;
}

static bool hw_atl_b0_tx_desc_complete(struct tx_ring *aq_ring, struct atl_packet *pkt)
{
	struct tx_desc_ptr *d_ptr = (struct tx_desc_ptr *)&pkt->private;

	if ( aq_ring->private_ring_data && d_ptr->pr_first == d_ptr->pr_last ) {
		struct tx_ring *ts_ring = (struct tx_ring *)aq_ring->private_ring_data;
		if( atl_b0_tx_desc_complete(ts_ring, d_ptr->pr_first, d_ptr->pr_last) ){ 
			d_ptr->pr_last++;
		}
	}
	return atl_b0_tx_desc_complete(aq_ring, d_ptr->first, d_ptr->last);
}

static int hw_atl_b0_avb_ring_rx_refill(struct rx_ring *aq_ring, struct atl_packet **rxbuf_packets, int *num_bufs)
{
	struct atl_packet *cur_pkt;
	struct atl_adapter *adapter;
	int bufs_used;
	bool refreshed = FALSE;
	union atl_rx_desc_u *d_ptr;
	int available = *num_bufs;
	cur_pkt = *rxbuf_packets;

	bufs_used = 0;
	while( cur_pkt && bufs_used < available && aq_ring->empty > 1 ) {
		if (((aq_ring->tail < (aq_ring->size - 1)) ? aq_ring->tail + 1 : 0) == aq_ring->shead) {
			logprint(LOG_LVL_WARNING, "Wrong empty count: empty %x, tail %x, shead %x", aq_ring->empty, aq_ring->tail, aq_ring->shead);
			break;
		}

		aq_ring->packet_per_desc[aq_ring->tail] = cur_pkt;
		d_ptr = &aq_ring->desc[aq_ring->tail++];
		if( aq_ring->tail >= aq_ring->size ) aq_ring->tail = 0;
		aq_ring->empty--;

		d_ptr->dsc.data_addr = (htole64(cur_pkt->map.paddr + cur_pkt->offset))>>1;
		d_ptr->dsc.dd = 0;
		d_ptr->dsc.header_addr = 0;
		d_ptr->dsc.nse_a0 = 0;

		dump(LOG_LVL_DEBUG, "AVB RX Desc", (uint8_t *)d_ptr, sizeof(*d_ptr));
		*rxbuf_packets = cur_pkt->next;
		cur_pkt->next = NULL;
		cur_pkt->private = 0;
		cur_pkt->flags = 0;

		refreshed = TRUE; /* I feel wefreshed :) */

		bufs_used++;
		cur_pkt = *rxbuf_packets;
	}

	if (refreshed) /* update tail */
		aq_ring->ctrl->tail = aq_ring->tail;

	logprint(LOG_LVL_DEBUG2, "rx refill aq_ring tail %x ctrl tail %x ctrl head %x", aq_ring->tail, aq_ring->ctrl->tail, aq_ring->ctrl->head & 0xffff);

	if( *num_bufs < bufs_used ){
		return -1;
	}
	*num_bufs -= bufs_used;
	return 0;
}

static int hw_atl_b0_avb_ring_rx_receive(struct rx_ring *aq_ring, 
		struct atl_packet **received_packets, u_int32_t *count)
{
	int desc_in_pkt = 0;
	u_int32_t max_pkt = 0;
	struct atl_packet *cur_pkt = NULL;
	struct atl_packet *cur_desc_pkt = NULL;
	struct atl_packet *prev_pkt = NULL;
	union atl_rx_desc_u *d_ptr;

	max_pkt = *count;
	*count  = 0;

	/* Main clean loop - receive packets until no more
	 * received_packets[]
	 */
	while( *count < max_pkt ) {
		d_ptr = &(aq_ring->desc[aq_ring->shead]);
		if ( !d_ptr->wb.dd )
			break;

		dump(LOG_LVL_DEBUG, "AVB WB Desc", (uint8_t *)d_ptr, sizeof(*d_ptr));
		cur_desc_pkt = aq_ring->packet_per_desc[aq_ring->shead];
		desc_in_pkt++;
		if ( (d_ptr->wb.rx_stat & 0x1) != 0 || d_ptr->wb.rdm_err ) {
			++aq_ring->rx_discarded;
			//printf ("discard error packet");
			while( cur_pkt ) {
				cur_pkt->next = cur_desc_pkt;
				cur_desc_pkt = cur_pkt;
				cur_pkt = (struct atl_packet *)cur_pkt->private;
			}
			aq_ring->empty += desc_in_pkt;
			aq_ring->shead += desc_in_pkt;
			if (aq_ring->shead >= aq_ring->size) aq_ring->shead -= aq_ring->size;

			hw_atl_b0_avb_ring_rx_refill(aq_ring, &cur_desc_pkt, &desc_in_pkt);
			desc_in_pkt = 0;
			continue;
		} else {
			/*
			* Free the frame (all segments) if we're at EOP and
			* it's an error.
			*
			* The datasheet states that EOP + status is only valid
			* for the final segment in a multi-segment frame.
			*/
			/*
				* add new packet to list of received packets
				* to return
				*/
			if( cur_pkt == NULL ){
				cur_pkt = cur_desc_pkt;
			}
			else {
				struct atl_packet *tmp_pkt = cur_pkt;
				while( tmp_pkt->private ) {
					tmp_pkt = (struct atl_packet *)tmp_pkt->private;
				}
				tmp_pkt->private = (uintptr_t)cur_desc_pkt;
			}

			if ( !d_ptr->wb.eop ) {
				cur_desc_pkt->len = (aq_ring->ctrl->sizes & 0xffff) << 10;
				continue;
			}
			cur_desc_pkt->len = d_ptr->wb.pkt_len;

			if( aq_ring->flags & RX_RING_FLAG_ING_TS ) {
				cur_pkt->flags = d_ptr->wb.ts_vld ? ATL_PKT_FLAG_HW_TS_VLD : 0;
			}
		}

		if (*received_packets == NULL)
			*received_packets = cur_pkt;
		if (prev_pkt)
			prev_pkt->next = cur_pkt;

		prev_pkt = cur_pkt;
		(*count)++;
		++aq_ring->rx_packets;
	
		/* Advance our pointers to the next descriptor. */
		aq_ring->empty += desc_in_pkt;
		aq_ring->shead += desc_in_pkt;
		if (aq_ring->shead >= aq_ring->size) aq_ring->shead -= aq_ring->size;
		desc_in_pkt = 0;
		cur_pkt = NULL;
	}
	return 0;
}

const struct aq_avb_hw_ops hw_atl_avb_ops_b0 = {
	.hw_avb_ring_tx_init = hw_atl_b0_avb_ring_tx_init,
	.hw_avb_ring_rx_init = hw_atl_b0_avb_ring_rx_init,

	.hw_avb_ring_tx_disable = hw_atl_b0_avb_ring_tx_disable,
	.hw_avb_ring_rx_disable = hw_atl_b0_avb_ring_rx_disable,

	.hw_avb_ring_tx_attach = hw_atl_b0_ring_tx_attach,
	.hw_avb_ring_rx_attach = hw_atl_b0_ring_rx_attach,
	.hw_avb_ring_tx_detach = hw_atl_b0_ring_tx_detach,
	.hw_avb_ring_rx_detach = hw_atl_b0_ring_rx_detach,

	.hw_avb_ring_tx_reset = hw_atl_b0_avb_ring_tx_reset,
	.hw_avb_ring_rx_reset = hw_atl_b0_avb_ring_rx_reset,

    .hw_ring_tx_prepare_xmit = hw_atl_b0_avb_ring_tx_prepare_xmit,
	.hw_ring_tx_desc_complete = hw_atl_b0_tx_desc_complete,
	
	.hw_ring_rx_refill = hw_atl_b0_avb_ring_rx_refill,
    .hw_ring_rx_ts_receive = hw_atl_b0_avb_ring_rx_receive,

	.hw_latch_clock = hw_atl_b0_latch_clock,
	.hw_get_clock = hw_atl_b0_get_clock,
};

#define ATL_TSG_CLOCK_SEL_0 0U
#define ATL_TSG_CLOCK_SEL_1 1U

static u32 clk_sel = ATL_TSG_CLOCK_SEL_1;
static u64 last_ts = (u64)-1;

static u32 hw_atl_a2_get_tc_assigned_to_q(struct tx_ring *aq_ring)
{
	u32 reg = 0;
	u32 ring_tc = -1;
	rr(aq_ring->adapter, HW_ATL2_TPB_FLEX_MAP_ADR(aq_ring->index), &reg);
	ring_tc = (reg & HW_ATL2_TPB_FLEX_MAP_MSK(aq_ring->index)) >> HW_ATL2_TPB_FLEX_MAP_SHIFT(aq_ring->index);
	return ring_tc;
}

static int hw_atl2_avb_ring_tx_init(struct tx_ring *aq_ring, u32 flags)
{
	int error, ring_size;
	u32 reg = 0;
	u32 ring_tc = hw_atl_a2_get_tc_assigned_to_q(aq_ring);
	if( flags & TX_RING_FLAG_EG_TS ) {
		flags &= ~TX_RING_FLAG_HEAD_WB;
	}
	ring_size = ATL_AVB_RING_SIZE + (flags & TX_RING_FLAG_HEAD_WB ? 1 : 0);
    error = create_tx_ring(aq_ring, ring_size, (int)aq_ring->index);
	if( !error ) {
		aq_ring->ctrl->prefetch = 125000; //TODO
		aq_ring->ctrl->ctrl_len |= BIT(A2_TX_RING_AVB_EN);
		aq_ring->ctrl->ctrl_len |= BIT(A2_TX_RING_AVB_NO_LT);
		aq_ring->flags |= flags & TX_RING_FLAG_WRITEBACK_EOP;
		if( flags & TX_RING_FLAG_EG_TS ) {
			aq_ring->ctrl->ctrl_len |= BIT(A2_TX_RING_EG_TS_WB_EN);
			aq_ring->flags |= TX_RING_FLAG_EG_TS;
		}
		if( flags & TX_RING_FLAG_HEAD_WB ) {
			hw_atl_b0_tx_avb_ring_hpwb_en(aq_ring);
		}
	}

	// Enable AVB support for ring
	rr(aq_ring->adapter, HW_ATL2_TPB_AVB_SCHDL_EN_ADR, &reg);
	reg |= 1 << ring_tc;
	wr(aq_ring->adapter, HW_ATL2_TPB_AVB_SCHDL_EN_ADR, reg);
	printf("Enable AVB for ring %d: %x", aq_ring->index, reg);
	return error;
}

static int hw_atl2_avb_ring_rx_init(struct rx_ring *aq_ring, u32 flags)
{
	int err = hw_atl_b0_avb_ring_rx_init(aq_ring, flags);
	if( !err ) {
		if( flags & RX_RING_FLAG_ING_TS ) {
			aq_ring->ctrl->ctrl_len &= ~(3U<<16);
			aq_ring->ctrl->ctrl_len |= (clk_sel == ATL_TSG_CLOCK_SEL_1 ? 2U : 1U) << 16;
			aq_ring->flags |= RX_RING_FLAG_ING_TS;
		}
	}
	return err;
}

static int hw_atl2_ring_tx_attach(struct tx_ring *aq_ring, u32 flags)
{
	int error, ring_size;
	struct atl_adapter *adapter = aq_ring->adapter;

	// Allocate place for AVB TS packets
	ring_size = ATL_AVB_RING_SIZE + (flags & TX_RING_FLAG_HEAD_WB ? 1 : 0);
    error = attach_tx_ring(aq_ring, ring_size, aq_ring->index);
	if (error) {
		goto release;
	}

	if( flags & TX_RING_FLAG_HEAD_WB ) {
		hw_atl_b0_tx_avb_ring_hpwb_en(aq_ring);
	}
	aq_ring->flags |= flags & TX_RING_FLAG_WRITEBACK_EOP;

	if( flags & TX_RING_FLAG_EG_TS ) {
		aq_ring->flags |= TX_RING_FLAG_EG_TS;
	}

	return 0;
release:
	return error;
}

static int hw_atl2_ring_tx_detach(struct tx_ring *aq_ring)
{
	struct atl_adapter *adapter = aq_ring->adapter;
	struct atl_dma_alloc dma = {0};
	dma.dma_paddr = aq_ring->dma_paddr;
	dma.dma_vaddr = aq_ring->desc;
	dma.mmap_size = (ATL_AVB_RING_SIZE + (aq_ring->flags & TX_RING_FLAG_HEAD_WB ? 1 : 0)) * sizeof(union atl_tx_desc_u);

	atl_dma_detach_buffer(adapter, &dma);
	return 0;
}

static int hw_atl_a2_avb_ring_tx_reset(struct tx_ring *aq_ring)
{
	int res;
	u32 reg = 0;
	u32 ring_tc = hw_atl_a2_get_tc_assigned_to_q(aq_ring);

	rr(aq_ring->adapter, HW_ATL2_TPB_AVB_SCHDL_EN_ADR, &reg);
	reg &= ~(1 << ring_tc);
	wr(aq_ring->adapter, HW_ATL2_TPB_AVB_SCHDL_EN_ADR, reg);
	//printf("Disable AVB for ring %d r: %x", aq_ring->index, reg);
	res = hw_atl_b0_avb_ring_tx_reset(aq_ring);

	reg |= 1 << ring_tc;
	wr(aq_ring->adapter, HW_ATL2_TPB_AVB_SCHDL_EN_ADR, reg);
	printf("Reset AVB for ring %d r: %x", aq_ring->index, reg);

	return res;
}

static int hw_atl_a2_avb_ring_tx_disable(struct tx_ring *aq_ring)
{
	struct atl_adapter *adapter = aq_ring->adapter;
	u32 reg = 0;
	u32 ring_tc = hw_atl_a2_get_tc_assigned_to_q(aq_ring);

	rr(aq_ring->adapter, HW_ATL2_TPB_FLEX_MAP_ADR(aq_ring->index), &reg);
	ring_tc = (reg & HW_ATL2_TPB_FLEX_MAP_MSK(aq_ring->index)) >> HW_ATL2_TPB_FLEX_MAP_SHIFT(aq_ring->index);

	// Disable AVB support for ring
	rr(aq_ring->adapter, HW_ATL2_TPB_AVB_SCHDL_EN_ADR, &reg);
	reg &= ~(1 << ring_tc);
	wr(aq_ring->adapter, HW_ATL2_TPB_AVB_SCHDL_EN_ADR, reg);
	printf("Disable AVB for ring %d: %x", aq_ring->index, reg);

	return hw_atl_b0_avb_ring_tx_disable(aq_ring);
}


static int hw_atl2_avb_ring_tx_prepare_xmit(struct tx_ring *aq_ring, struct atl_packet *packet)
{
	union atl_tx_desc_u *txd;
	struct tx_desc_ptr *d_ptr = (struct tx_desc_ptr *)&packet->private;
	d_ptr->first = aq_ring->tail;
	hw_atl_b0_tx_ctx_setup(aq_ring, packet);
	d_ptr->last = aq_ring->tail;
	txd = get_empty_tx_desc(aq_ring);
	memset(txd, 0, sizeof(*txd));
	logprint(LOG_LVL_DEBUG, "Insert packet %p, attime %lu, last_ts %lu", packet, packet->attime, last_ts);
 	if( packet->attime != last_ts ){
		txd->dsc.desc_type = TX_TS_DESC_TYPE;
		txd->ltd.lt_vld = 1;
		txd->ltd.clk_sel = clk_sel == ATL_TSG_CLOCK_SEL_0;
		txd->ltd.lt = htole64(packet->attime);
		last_ts = packet->attime;		
		dump(LOG_LVL_DEBUG, "AVB LT TX Desc", (uint8_t *)txd, sizeof(*txd));
		d_ptr->last = aq_ring->tail;
		txd = get_empty_tx_desc(aq_ring);
		memset(txd, 0, sizeof(*txd));
	}
	txd->dsc.addr = htole64(packet->map.paddr + packet->offset);
	txd->dsc.pay_len = htole32(packet->len);
	/* we assume every packet is contiguous one packet - one buffer*/
	txd->dsc.buf_len = htole32(packet->len);
	txd->dsc.desc_type = TX_PACKET_DESC_TYPE;
	txd->dsc.eop = 1;
	txd->dsc.dd = 0;
	txd->dsc.ts_vld = 0;
	txd->dsc.tx_cmd = TX_CMD_MAC_FCS | 
					(aq_ring->flags & TX_RING_FLAG_WRITEBACK_EOP ? TX_CMD_WB_EN : 0);
	if( aq_ring->flags & TX_RING_FLAG_EG_TS ) {
		txd->dsc.clk_sel = clk_sel == ATL_TSG_CLOCK_SEL_0;
		txd->dsc.ts_en = 1;
	}
	if( packet->vlan ){
		txd->dsc.ct_en = 1;
		txd->dsc.ct_idx = packet->ct_idx;
		txd->dsc.tx_cmd |= TX_CMD_VLAN_INSERT;
	}
	dump(LOG_LVL_DEBUG, "AVB Data TX Desc", (uint8_t *)txd, sizeof(*txd));
	aq_ring->ctrl->tail = aq_ring->tail;
	logprint(LOG_LVL_DEBUG2, "tx a2 xmit aq_ring tail %x ctrl tail %x ctrl head %x empty %x", aq_ring->tail, aq_ring->ctrl->tail, aq_ring->ctrl->head & 0xffff, aq_ring->empty);
	return 0;
}

static bool hw_atl2_tx_desc_complete(struct tx_ring *aq_ring, struct atl_packet *pkt)
{
	bool res = true;
	struct tx_desc_ptr *d_ptr = (struct tx_desc_ptr *)&pkt->private;

	if( d_ptr->first < aq_ring->size ) {
		res = atl_b0_tx_desc_complete(aq_ring, d_ptr->first, d_ptr->last);
		if( res ) {
			d_ptr->first = aq_ring->size;
		}
	}
	if( res && aq_ring->flags & TX_RING_FLAG_EG_TS ) {
		union atl_tx_desc_u *eop_desc;
		eop_desc = &aq_ring->desc[d_ptr->last];
		res = eop_desc->wb.ts_vld;
		if( res ) {
			pkt->hwtime = eop_desc->wb.ts;
			pkt->flags |= ATL_PKT_FLAG_HW_TS_VLD;
		}
	}
	return res;
}

static int hw_atl2_avb_ring_rx_ts_receive(struct rx_ring *aq_ring, 
		struct atl_packet **received_packets, u_int32_t *count) 
{
	struct atl_packet *cur_pkt = NULL;
	int err = hw_atl_b0_avb_ring_rx_receive(aq_ring, received_packets, count);
	if( err || *count == 0 || !(aq_ring->flags & RX_RING_FLAG_ING_TS) ) {
		return err;
	}

	cur_pkt = *received_packets;
	while( cur_pkt ) {
		if( cur_pkt->flags & ATL_PKT_FLAG_HW_TS_VLD ) {
			u64 ts = 0;
			u32 offset = 0;
			struct atl_packet *tmp_pkt = cur_pkt, *prev_pkt = NULL;
			while( tmp_pkt->private ) {
				prev_pkt = tmp_pkt;
				tmp_pkt = (struct atl_packet *)tmp_pkt->private;
			}
			if( tmp_pkt->len >= sizeof(u64) ){
				offset = tmp_pkt->len - sizeof(u64);
			}
			ts = ((u64 *)((uint8_t *)tmp_pkt->vaddr + offset))[0];
			if( tmp_pkt->len < sizeof(u64) ) {
				if( prev_pkt ) {
					int refresh = 1;
					offset = prev_pkt->len - tmp_pkt->len;
					ts |= (((u64 *)((uint8_t *)prev_pkt->vaddr + offset))[0]) << (tmp_pkt->len*8);

					prev_pkt->private = 0;
					hw_atl_b0_avb_ring_rx_refill(aq_ring, &tmp_pkt, &refresh);
				} else {
					logprint(LOG_LVL_WARNING, "Wrong packet. Length less than timestamp length.");
				}
			}
			else {
				tmp_pkt->len -= sizeof(u64);
			}
			cur_pkt->hwtime = ts;
		}
		cur_pkt = cur_pkt->next;
	}
}

static int hw_atl2_latch_clock(struct atl_adapter *adapter)
{
	return 0;
}

static int hw_atl2_get_clock(struct atl_adapter *adapter, u64 *ts)
{
	rr64(adapter, HW_ATL2_TSG_REG_ADR(clk_sel, READ_CUR_NS_LSW), ts);
	return 0;
}

const struct aq_avb_hw_ops hw_atl2_avb_ops = {
	.hw_avb_ring_tx_init = hw_atl2_avb_ring_tx_init,
	.hw_avb_ring_rx_init = hw_atl2_avb_ring_rx_init,

	.hw_avb_ring_tx_attach = hw_atl2_ring_tx_attach,
	.hw_avb_ring_rx_attach = hw_atl_b0_ring_rx_attach,
	.hw_avb_ring_tx_detach = hw_atl2_ring_tx_detach,
	.hw_avb_ring_rx_detach = hw_atl_b0_ring_rx_detach,

	.hw_avb_ring_tx_disable = hw_atl_a2_avb_ring_tx_disable,
	.hw_avb_ring_rx_disable = hw_atl_b0_avb_ring_rx_disable,

	.hw_avb_ring_tx_reset = hw_atl_a2_avb_ring_tx_reset,
	.hw_avb_ring_rx_reset = hw_atl_b0_avb_ring_rx_reset,

    .hw_ring_tx_prepare_xmit = hw_atl2_avb_ring_tx_prepare_xmit,
	.hw_ring_tx_desc_complete = hw_atl2_tx_desc_complete,

	.hw_ring_rx_refill = hw_atl_b0_avb_ring_rx_refill,
    .hw_ring_rx_ts_receive = hw_atl2_avb_ring_rx_ts_receive,

	.hw_latch_clock = hw_atl2_latch_clock,
	.hw_get_clock = hw_atl2_get_clock,
};
//EOF
