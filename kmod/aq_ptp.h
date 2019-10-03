/* SPDX-License-Identifier: GPL-2.0-only */
/* Aquantia Corporation Network Driver
 * Copyright (C) 2014-2019 Aquantia Corporation. All rights reserved
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

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 0)) ||\
    (RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(7, 2))

/* Common functions */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0)
int aq_ptp_init(struct aq_nic_s *aq_nic, unsigned int idx_vec, 
        unsigned int ptp_vec, unsigned int gpio_vec);
#else
int aq_ptp_init(struct aq_nic_s *aq_nic, unsigned int idx_vec);
#endif

enum aq_ptp_state {
    AQ_PTP_NO_LINK = 0,
    AQ_PTP_FIRST_INIT = 1,
    AQ_PTP_LINK_UP = 2,
};

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
void aq_ptp_offset_get(unsigned int mbps, int *egress, int *ingress);

void aq_ptp_clock_init(struct aq_nic_s *aq_nic, enum aq_ptp_state state);

/* Traffic processing functions */
int aq_ptp_xmit(struct aq_nic_s *aq_nic, struct sk_buff *skb);
void aq_ptp_tx_hwtstamp(struct aq_nic_s *aq_nic, u64 timestamp);

/* Must be to check available of PTP before call */
void aq_ptp_hwtstamp_config_get(struct aq_ptp_s *self,
				struct hwtstamp_config *config);
int aq_ptp_hwtstamp_config_set(struct aq_ptp_s *self,
			       struct hwtstamp_config *config);

/* Return either ring is belong to PTP or not*/
bool aq_ptp_ring(struct aq_ring_s *ring);

u16 aq_ptp_extract_ts(struct aq_nic_s *aq_nic, struct sk_buff *skb, u8 *p,
		      unsigned int len);

struct ptp_clock *aq_ptp_get_ptp_clock(struct aq_ptp_s *self);

#ifdef PTM_SUPPORT
int aq_ptp_ptm_enable(struct aq_ptp_s *self, struct ifreq *ifr);
int aq_ptp_read_ptm_info(struct aq_ptp_s *self, struct ifreq *ifr);
#endif
int aq_ptp_link_change(struct aq_nic_s *aq_nic);

int aq_ptp_configure_ext_gpio(struct net_device *ndev,
				 struct aq_ptp_ext_gpio_event *gpio_event);
#else
/* Return either ring is belong to PTP or not*/
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
#endif

#endif /* AQ_PTP_H */