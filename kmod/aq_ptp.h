/* SPDX-License-Identifier: GPL-2.0-only */
/* Atlantic Network Driver
 *
 * Copyright (C) 2014-2019 aQuantia Corporation
 * Copyright (C) 2019-2020 Marvell International Ltd.
 */

/* File aq_ptp.h: Declaration of PTP functions.
 */
#ifndef AQ_PTP_H
#define AQ_PTP_H

#include <linux/net_tstamp.h>
#include <linux/version.h>

#include "aq_compat.h"

#define AQ_PTP_SYNC_CFG (SIOCDEVPRIVATE + 1)
#define SIOCENPTM       (SIOCDEVPRIVATE + 8)		/* Enable PTM */
#define SIOCGETPTMINFO  (SIOCDEVPRIVATE + 9)		/* Get PTM info */

enum aq_sync_cntr_action {
	aq_sync_cntr_nop = 0, /* no action */
	aq_sync_cntr_set, /* set new counter value */
	aq_sync_cntr_add, /* add value to counter value */
	aq_sync_cntr_sub, /* subtract value from counter value */
};

struct aq_ptp_ext_gpio_event {
	uint64_t time_ns; /* new/adjusted PTP clock value in ns*/
	enum aq_sync_cntr_action action;
	uint16_t sync_pulse_ms;
	uint8_t clock_sync_en; /* Enabling sync clock */
	uint8_t gpio_index;
} __packed;

enum aq_ptp_state {
	AQ_PTP_NO_LINK = 0,
	AQ_PTP_FIRST_INIT = 1,
	AQ_PTP_LINK_UP = 2,
};

static inline unsigned int aq_ptp_ring_idx(const enum aq_tc_mode tc_mode)
{
	return tc_mode == AQ_TC_MODE_8TCS ? AQ_HW_PTP_8TC_RING_IDX :
										AQ_HW_PTP_4TC_RING_IDX;
}

#if IS_REACHABLE(CONFIG_PTP_1588_CLOCK)

/* Common functions */
int aq_ptp_init(struct aq_nic_s *aq_nic, unsigned int idx_ptp_vec,
		unsigned int idx_ext_vec);

void aq_ptp_unregister(struct aq_nic_s *aq_nic);
void aq_ptp_free(struct aq_nic_s *aq_nic);

int aq_ptp_irq_alloc(struct aq_nic_s *aq_nic);
void aq_ptp_irq_free(struct aq_nic_s *aq_nic);

int aq_ptp_ring_alloc(struct aq_nic_s *aq_nic);
void aq_ptp_ring_free(struct aq_nic_s *aq_nic);

int aq_ptp_ring_init(struct aq_nic_s *aq_nic);
int aq_ptp_ring_start(struct aq_nic_s *aq_nic);
void aq_ptp_ring_stop(struct aq_nic_s *aq_nic);
void aq_ptp_ring_deinit(struct aq_nic_s *aq_nic);

void aq_ptp_service_task(struct aq_nic_s *aq_nic);

void aq_ptp_tm_offset_set(struct aq_nic_s *aq_nic, unsigned int mbps);
void aq_ptp_offset_get(struct aq_ptp_s *aq_ptp,
		       unsigned int mbps, int *egress, int *ingress);

void aq_ptp_clock_init(struct aq_nic_s *aq_nic, enum aq_ptp_state state);

/* Traffic processing functions */
int aq_ptp_xmit(struct aq_nic_s *aq_nic, struct sk_buff *skb);
void aq_ptp_tx_hwtstamp(struct aq_nic_s *aq_nic, u64 timestamp);

/* Must be to check available of PTP before call */
void aq_ptp_hwtstamp_config_get(struct aq_ptp_s *aq_ptp,
				struct hwtstamp_config *config);
int aq_ptp_hwtstamp_config_set(struct aq_ptp_s *aq_ptp,
			       struct hwtstamp_config *config);

/* Return either ring is belong to PTP or not*/
bool aq_ptp_ring(struct aq_ring_s *ring);

u16 aq_ptp_extract_ts(struct aq_nic_s *aq_nic, struct sk_buff *skb, u8 *p,
		      unsigned int len);

struct ptp_clock *aq_ptp_get_ptp_clock(struct aq_ptp_s *aq_ptp);

#ifdef PTM_SUPPORT
int aq_ptp_ptm_enable(struct aq_ptp_s *aq_ptp, struct ifreq *ifr);
int aq_ptp_read_ptm_info(struct aq_ptp_s *aq_ptp, struct ifreq *ifr);
#endif
int aq_ptp_link_change(struct aq_nic_s *aq_nic);

int aq_ptp_configure_ext_gpio(struct net_device *ndev,
			      struct aq_ptp_ext_gpio_event *gpio_event);


/* PTP ring statistics */
int aq_ptp_get_ring_cnt(struct aq_nic_s *aq_nic);
u64 *aq_ptp_get_stats(struct aq_nic_s *aq_nic, u64 *data);

#else

struct aq_ptp_s {
};

/* Common functions */
static inline int aq_ptp_init(struct aq_nic_s *aq_nic, unsigned int idx_ptp_vec,
		unsigned int idx_ext_vec)
{
	return 0;
}

static inline void aq_ptp_unregister(struct aq_nic_s *aq_nic) {}

static inline void aq_ptp_free(struct aq_nic_s *aq_nic)
{
}

static inline int aq_ptp_irq_alloc(struct aq_nic_s *aq_nic)
{
	return 0;
}

static inline void aq_ptp_irq_free(struct aq_nic_s *aq_nic)
{
}

static inline int aq_ptp_ring_alloc(struct aq_nic_s *aq_nic)
{
	return 0;
}

static inline void aq_ptp_ring_free(struct aq_nic_s *aq_nic) {}

static inline int aq_ptp_ring_init(struct aq_nic_s *aq_nic)
{
	return 0;
}

static inline int aq_ptp_ring_start(struct aq_nic_s *aq_nic)
{
	return 0;
}

static inline void aq_ptp_ring_stop(struct aq_nic_s *aq_nic) {}
static inline void aq_ptp_ring_deinit(struct aq_nic_s *aq_nic) {}
static inline void aq_ptp_service_task(struct aq_nic_s *aq_nic) {}
static inline void aq_ptp_tm_offset_set(struct aq_nic_s *aq_nic,
					unsigned int mbps) {}
static inline void aq_ptp_offset_get(struct aq_ptp_s *aq_ptp,
				     unsigned int mbps,
				     int *egress, int *ingress) {}
static inline void aq_ptp_clock_init(struct aq_nic_s *aq_nic,
				     enum aq_ptp_state state) {}
static inline int aq_ptp_xmit(struct aq_nic_s *aq_nic, struct sk_buff *skb)
{
	return -EOPNOTSUPP;
}

static inline void aq_ptp_tx_hwtstamp(struct aq_nic_s *aq_nic, u64 timestamp) {}
static inline void aq_ptp_hwtstamp_config_get(struct aq_ptp_s *aq_ptp,
					      struct hwtstamp_config *config) {}
static inline int aq_ptp_hwtstamp_config_set(struct aq_ptp_s *aq_ptp,
					     struct hwtstamp_config *config)
{
	return 0;
}

static inline bool aq_ptp_ring(struct aq_ring_s *ring)
{
	return false;
}

static inline u16 aq_ptp_extract_ts(struct aq_nic_s *aq_nic,
				    struct sk_buff *skb, u8 *p,
				    unsigned int len)
{
	return 0;
}

static inline struct ptp_clock *aq_ptp_get_ptp_clock(struct aq_ptp_s *aq_ptp)
{
	return NULL;
}

static inline int aq_ptp_link_change(struct aq_nic_s *aq_nic)
{
	return 0;
}
#endif

#endif /* AQ_PTP_H */