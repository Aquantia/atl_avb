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

#ifndef _ATL_PRIVATE_H_DEFINED_
#define _ATL_PRIVATE_H_DEFINED_

#include "atl.h"

#ifndef _OSDEP_H_
#define _OSDEP_H_

/* generic boolean compatibility */
#undef TRUE
#undef FALSE
#define TRUE true
#define FALSE false
#define _Bool char
#ifndef bool
#define bool _Bool
#define true 1
#define false 0
#endif

typedef u_int64_t	u64;
typedef u_int32_t	u32;
typedef u_int16_t	u16;
typedef u_int8_t		u8;
typedef int64_t		s64;
typedef int32_t		s32;
typedef int16_t		s16;
typedef int8_t		s8;

#define __le16		u16
#define __le32		u32
#define __le64		u64
#endif  /* _OSDEP_H_ */

#ifdef ARRAY_SIZE
#undef ARRAY_SIZE
#endif
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

/*
 * Micellaneous constants
 */
#define ATL_VENDOR_ID			0x1d6a

#define ETHER_ALIGN                     2
#define ETH_ZLEN		60
#define ETH_ADDR_LEN		6

/* Precision Time Sync (IEEE 1588) defines */
#define ETHERTYPE_IEEE1588	0x88F7
#define PICOSECS_PER_TICK	20833
#define TSYNC_PORT	    	319 /* UDP port for the protocol */

#define ATL_AVB_RING           8
#define ATL2_AVB_RING2         24
#define ATL_AVB_FLEX_RING      17
#define ATL2_AVB_RING2         24
#define ATL_AVB_FLEX_RING2     18
#define MAX_TX_RING_COUNT      2
#define MAX_RX_RING_COUNT      2

struct atl_adapter;
struct tx_ring;
struct rx_ring;

struct mmap_res {
	void *vaddr;
	u64 paddr;
	u32 size;
	int fd;
	int idx;
	struct list_head list;
};

/*
 * vendor_info_array
 *
 * This array contains the list of Subvendor/Subdevice IDs on which the driver
 * should load.
 *
 */
typedef struct _atl_vendor_info_t {
	unsigned int vendor_id;
	unsigned int device_id;
	//unsigned int subvendor_id;
	//unsigned int subdevice_id;
	unsigned int revision;
	const struct aq_avb_hw_ops *hw_ops;
} atl_vendor_info_t;

#pragma pack(push,1)
struct atl_tx_ring_ctrl {
	/*volatile u64 paddr;
	volatile u32 len : 13;
	volatile u32 reserved0 : 12;
	volatile u32 reset : 1;
	volatile u32 reserved1 : 2;
	volatile u32 en_hpwb : 1;
	volatile u32 reserved2 : 2;
	volatile u32 en : 1;
	volatile u32 current : 16;
	volatile u32 head : 16;*/
	volatile u32 paddr_lsw;
	volatile u32 paddr_msw;
	volatile u32 ctrl_len;
	volatile u32 head;
	volatile u32 tail;
	volatile u32 stat;
	volatile u32 thresholds;
	volatile u32 hpwb_addr_lsw;
	volatile u32 hpwb_addr_msw;
	volatile u32 prefetch;
	volatile u32 itr_moderation;
	volatile u32 reserved3;
	u64 reserved4[2];
};

struct atl_rx_ring_ctrl {
	/*volatile u64 paddr;
	volatile u32 len : 13;
	volatile u32 reserved : 12;
	volatile u32 reset : 1;
	volatile u32 reserved1: 5;
	volatile u32 en : 1; */
	volatile u32 paddr_lsw;
	volatile u32 paddr_msw;
	volatile u32 ctrl_len;
	volatile u32 head;
	volatile u32 tail;
	volatile u32 stat;
	volatile u32 sizes;
	volatile u32 thresholds;
};

#ifndef BIT
#define BIT(n) (1 << (n))
#endif
#define RING_CTRL_ENABLE_BIT 31
#define RING_CTRL_HEAD_WB_BIT 28
#define RING_CTRL_RESET_BIT 25

#define A2_TX_RING_EG_TS_WB_EN 18
#define A2_TX_RING_AVB_NO_LT 17
#define A2_TX_RING_AVB_EN 16
enum {
	TX_PACKET_DESC_TYPE = 1,
	TX_CONTEXT_DESC_TYPE = 2,
	TX_TS_DESC_TYPE = 3,
};

enum {
	TX_CMD_VLAN_INSERT = 0x01,
	TX_CMD_MAC_FCS = 0x02,
	TX_CMD_WB_EN = 0x20,
};

struct atl_tx_cont_desc_s {
	u64 reserved : 40;
	u64 tun_len : 8;
	u64 out_len : 8;
	u64 desc_type : 3;
	u64 ct_idx : 1;
	u64 vlan_tag : 16;
	u64 ct_cmd : 4;
	u64 l2_len : 7;
	u64 l3_len : 9;
	u64 l4_len : 8;
	u64 mss_len : 16;
};

struct atl2_tx_lt_desc_s {
	u64 lt;
	u64 desc_type : 3;
	u64 reserved : 1;
	u64 lt_vld : 1;
	u64 clk_sel : 1;
	u64 reserved1 : 58;
};

struct atl_tx_desc_s {
	u64 addr;
	u64 desc_type : 3;
	u64 ts_vld : 1;
	u64 buf_len : 16;
	u64 dd : 1;
	u64 eop : 1;
	u64 tx_cmd : 8;
	u64 ts_en : 1;
	u64 clk_sel : 1;
	u64 reserved1 : 12;
	u64 ct_idx : 1;
	u64 ct_en : 1;
	u64 pay_len : 18;
};

struct atl_tx_wb_desc_s {
	u64 ts;
	u64 reserved1 : 3;
	u64 ts_vld : 1;
	u64 reserved2 : 16;
	u64 dd : 1;
	u64 reserved3 : 43;
};

union atl_tx_desc_u {
	struct atl_tx_cont_desc_s ctx;
	struct atl_tx_desc_s dsc;
	struct atl_tx_wb_desc_s wb;
	struct atl2_tx_lt_desc_s ltd;
	u64 qw[2];
};

struct atl_rx_desc_s {
	u64 nse_a0 : 1;
	u64 data_addr : 63;
	u64 dd : 1;
	u64 header_addr : 63;
};

struct atl_rx_wb_desc_s {
	u64 rss_type : 4;
	u64 pkt_type : 8;
	u64 rdm_err : 1;
	u64 ts_vld : 1;
	u64 reserved : 5;
	u64 rx_cntl : 2;
	u64 sph : 1;
	u64 hdr_len : 10;
	u64 rss_hash : 32;
	u64 dd : 1;
	u64 eop : 1;
	u64 rx_stat : 4;
	u64 rx_estat : 6;
	u64 rsc_cnt : 4;
	u64 pkt_len : 16;
	u64 next_desc : 16;
	u64 vlan_tag : 16;
};

union atl_rx_desc_u {
	struct atl_rx_desc_s dsc;
	struct atl_rx_wb_desc_s wb;
	u64 qw[2];
};

struct aq_avb_hw_ops {
	int (*hw_avb_ring_tx_init)(struct tx_ring *aq_ring, u32 flags);
	int (*hw_avb_ring_rx_init)(struct rx_ring *aq_ring, u32 flags);
	int (*hw_avb_ring_tx_reset)(struct tx_ring *aq_ring);
	int (*hw_avb_ring_rx_reset)(struct rx_ring *aq_ring);
	int (*hw_avb_ring_tx_disable)(struct tx_ring *aq_ring);
	int (*hw_avb_ring_rx_disable)(struct rx_ring *aq_ring);

	int (*hw_avb_ring_tx_attach)(struct tx_ring *aq_ring, u32 flags);
	int (*hw_avb_ring_rx_attach)(struct rx_ring *aq_ring, u32 flags);
	int (*hw_avb_ring_tx_detach)(struct tx_ring *aq_ring);
	int (*hw_avb_ring_rx_detach)(struct rx_ring *aq_ring);

	int (*hw_ring_tx_prepare_xmit)(struct tx_ring *aq_ring, struct atl_packet *packet);
	bool (*hw_ring_tx_desc_complete)(struct tx_ring *aq_ring, struct atl_packet *pkt);

	int (*hw_ring_rx_refill)(struct rx_ring *aq_ring, struct atl_packet **rxbuf_packets, int *num_bufs);
	int (*hw_ring_rx_ts_receive)(struct rx_ring *aq_ring, struct atl_packet **received_packets, u_int32_t *count);

	int (*hw_latch_clock)(struct atl_adapter *adapter);
	int (*hw_get_clock)(struct atl_adapter *adapter, u64 *ts);

/*
	int (*hw_ring_rx_fill)(struct aq_hw_s *self, struct aq_ring_s *aq_ring,
			       unsigned int sw_tail_old);

	int (*hw_ring_tx_start)(struct aq_hw_s *self,
				struct aq_ring_s *aq_ring);

	int (*hw_ring_tx_stop)(struct aq_hw_s *self,
			       struct aq_ring_s *aq_ring);

	int (*hw_ring_rx_start)(struct aq_hw_s *self,
				struct aq_ring_s *aq_ring);

	int (*hw_ring_rx_stop)(struct aq_hw_s *self,
			       struct aq_ring_s *aq_ring);
*/
};

struct tx_desc_ptr {
	u16 first;	
	u16 last;
	u16 pr_first;	
	u16 pr_last;
};

/*
 * Transmit ring: one per queue
 */
struct tx_ring {
	struct atl_adapter *adapter;
	u32 index;

	u_int64_t dma_paddr;

	union atl_tx_desc_u *desc;
	volatile struct atl_tx_ring_ctrl *ctrl;

	u32 size; // In descriptors
	u32 empty; // In descriptors
	u32 hhead;
	u32 shead;
	u32 tail;

	u32 flags;

	void *private_ring_data; //AVBTS ring for B0

	struct atl_packet *sent_packet;
	struct atl_packet *last_packet;

	int queue_status;
	struct tx_ring *next_ring;
};

#define TX_RING_FLAG_HEAD_WB 1
#define TX_RING_FLAG_EG_TS 2
#define TX_RING_FLAG_WRITEBACK_EOP 4
#define RX_RING_FLAG_SPLIT_HDR 1
#define RX_RING_FLAG_ING_TS 2

/*
 * Receive ring: one per queue
 */
struct rx_ring {
	struct atl_adapter *adapter;
	u32 index;

	u_int64_t dma_paddr;

	union atl_rx_desc_u *desc;
	volatile struct atl_rx_ring_ctrl *ctrl;

	u32 size; // In descriptors
	u32 empty; // In descriptors
	u32 hhead;
	u32 shead;
	u32 tail;

	u32 flags;

	/* Soft stats */
	u64 rx_split_packets;
	u64 rx_discarded;
	u64 rx_packets;
	u64 rx_bytes;

	struct atl_packet **packet_per_desc;

	struct rx_ring *next_ring;
	sem_t lock;
};

#define RX_RING_FLAG_SPLIT_HEADER 1
struct atl_share_mem;
struct atl_adapter {
	int sd; /* socket to atl */

	int class_fd;
	int mem_fd;

	char *ifname;
	char *driver_class;

	struct atl_share_mem *aq_shmem;
	pthread_mutex_t *memlock;

	struct mmap_res bar[6];
	struct mmap_res *default_bar;

	struct list_head memregs;

	const struct aq_avb_hw_ops *aq_hw_avb_ops;

	struct tx_ring *tx_rings;
	struct rx_ring *rx_rings;

	u32 link_speed;
	u32 hw_offsets;

	u32 class_a_bytes_per_second;
	u32 class_b_bytes_per_second;

	u32 credit_for_one_ns;
	u32 credit_for_one_byte;

	s32 class_a_cur_credit;
	s32 class_b_cur_credit;

	u32 class_a_threshold;
	u32 class_b_threshold;

	u64 class_a_last_lt;
	u64 class_b_last_lt;

	int active;
	bool a1;
	bool a2;
};

struct atl_fw_ts_link {
	u64 ts;
	u32 last_desc;
	u32 pkt_len;
};

struct atl_fw_ts_packet {
	u8 dst[6];
	u8 ring;
	u8 ts_cnt;
	u64 reserved;
	struct atl_fw_ts_link ts_link[1];
};
#pragma pack(pop)

int check_access(struct atl_adapter *adapter, int idx, u_int32_t addr);
int atl_dma_remap_buffer(struct atl_adapter *adapter, struct atl_dma_alloc *dma);
int atl_dma_alloc_buffer(struct atl_adapter *adapter, struct atl_dma_alloc *dma);
int atl_dma_detach_buffer(struct atl_adapter *adapter, struct atl_dma_alloc *dma);
void atl_dma_free_buffer(struct atl_adapter *adapter, struct atl_dma_alloc *dma);

extern const struct aq_avb_hw_ops hw_atl_avb_ops_b0;
extern const struct aq_avb_hw_ops hw_atl2_avb_ops;

#define ATL_GET_ADDR(adapter, addr)\
	(!check_access(adapter, 0, addr) ? (((uint8_t *)adapter->bar[0].vaddr) + addr) : 0)

int wr(struct atl_adapter *adapter, u32 addr, u32 val);
int rr(struct atl_adapter *adapter, u32 addr, u32 *val);
int wr64(struct atl_adapter *adapter, u32 addr, u64 val);
int rr64(struct atl_adapter *adapter, u32 addr, u64 *val);

#define ATL_TX_RING_BASE_ADDRESS(n_txring) (0x7c00 + n_txring*sizeof(struct atl_tx_ring_ctrl))
#define ATL_RX_RING_BASE_ADDRESS(n_rxring) (0x5b00 + n_rxring*sizeof(struct atl_rx_ring_ctrl))
#define INIT_RING_ACCESS(dir, DIR, adapter, n_ring) \
	((volatile struct atl_##dir##_ring_ctrl *)ATL_GET_ADDR(adapter, (ATL_##DIR##_RING_BASE_ADDRESS(n_ring))))

#ifndef min
#define min(a,b) ((a)<=(b)?(a):(b))
#endif

#define CREATE_RING(dir, DIR) \
static int create_##dir##_ring(struct dir##_ring *aq_ring, int ring_size, int n)\
{\
	struct atl_adapter *adapter = aq_ring->adapter;\
	struct atl_dma_alloc dma = {.mmap_size = ring_size * sizeof(union atl_##dir##_desc_u)};\
\
	int error = atl_dma_alloc_buffer(adapter, &dma);\
	if (error) {\
		goto release;\
	}\
    aq_ring->ctrl = INIT_RING_ACCESS(dir, DIR, adapter, n);\
    if (!aq_ring->ctrl) {\
        goto release;\
    }\
	aq_ring->index = n; \
    aq_ring->dma_paddr = dma.dma_paddr; \
    aq_ring->ctrl->paddr_lsw = (u32)((dma.dma_paddr >> 0) & 0xffffffff);\
    aq_ring->ctrl->paddr_msw = (u32)((dma.dma_paddr >> 32) & 0xffffffff);\
\
    aq_ring->desc = (union atl_##dir##_desc_u *)dma.dma_vaddr;\
\
    aq_ring->size = min(ring_size, dma.mmap_size / sizeof(union atl_tx_desc_u)) & (~0x7);\
    aq_ring->empty = aq_ring->size;\
    /*aq_ring->shead = 0;*/\
    /*aq_ring->ctrl->head = aq_ring->hhead = 0;*/\
	/*aq_ring->ctrl->tail = aq_ring->tail = 0;*/\
	aq_ring->tail = aq_ring->shead = aq_ring->hhead = aq_ring->ctrl->tail = aq_ring->ctrl->head & 0xffff;\
\
    /*? memset(aq_ring->desc, 0, aq_ring->size*sizeof(union atl_tx_desc_u));*/ \
	aq_ring->flags = 0;\
    aq_ring->ctrl->ctrl_len = BIT(RING_CTRL_ENABLE_BIT) | aq_ring->size; /* Enable ring */\
	/*aq_ring->ctrl->en = 1; *//* Enable ring */ \
    return 0; \
release: \
    if( dma.dma_vaddr ){ \
        atl_dma_free_buffer(adapter, &dma); \
    } \
    memset(aq_ring, 0, sizeof(*aq_ring));\
    return error;\
}

#define ATTACH_RING(dir, DIR) \
static int attach_##dir##_ring(struct dir##_ring *aq_ring, int ring_size, int n)\
{\
	struct atl_adapter *adapter = aq_ring->adapter;\
	struct atl_dma_alloc dma = {.mmap_size = ring_size * sizeof(union atl_##dir##_desc_u)};\
	int error = 0;\
    aq_ring->ctrl = INIT_RING_ACCESS(dir, DIR, adapter, n);\
    if (!aq_ring->ctrl) {\
        goto release;\
    }\
    dma.dma_paddr = ((u64)aq_ring->ctrl->paddr_msw << 32) | aq_ring->ctrl->paddr_lsw;\
	error = atl_dma_remap_buffer(adapter, &dma);\
	if (error) {\
		goto release;\
	}\
	aq_ring->index = n; \
    aq_ring->dma_paddr = dma.dma_paddr; \
\
    aq_ring->desc = (union atl_##dir##_desc_u *)dma.dma_vaddr;\
\
    aq_ring->size = min(ring_size, dma.mmap_size / sizeof(union atl_tx_desc_u)) & (~0x7);\
    aq_ring->empty = aq_ring->size;\
	aq_ring->shead = aq_ring->hhead = aq_ring->ctrl->head & 0xffff;\
	aq_ring->tail = aq_ring->ctrl->tail;\
	aq_ring->flags = 0;\
    /*aq_ring->ctrl->ctrl_len = BIT(RING_CTRL_ENABLE_BIT) | aq_ring->size; / * Enable ring */\
    return 0; \
release: \
    memset(aq_ring, 0, sizeof(*aq_ring));\
    return error;\
}

/*END*/
#endif /* _ATL_H_DEFINED_ */
