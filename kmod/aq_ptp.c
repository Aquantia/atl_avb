// SPDX-License-Identifier: GPL-2.0-only
/* Aquantia Corporation Network Driver
 * Copyright (C) 2014-2019 Aquantia Corporation. All rights reserved
 */

/* File aq_ptp.c:
 * Definition of functions for Linux PTP support.
 */

#include <linux/version.h>
#include <linux/moduleparam.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/ptp_classify.h>
#include <linux/interrupt.h>
#include <linux/clocksource.h>

#include "aq_nic.h"
#include "aq_ptp.h"
#include "aq_ring.h"
#include "aq_phy.h"
#include "aq_ethtool.h"
#include "aq_filters.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 0)) ||\
    (RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(7, 2))

#define AQ_PTP_TX_TIMEOUT        (HZ *  10)

static unsigned int aq_ptp_offset_forced = 0;
module_param(aq_ptp_offset_forced, uint, 0644);
MODULE_PARM_DESC(aq_ptp_offset_forced, "Force to use the driver parameters");

static unsigned int aq_ptp_offset_100 = 0;
module_param(aq_ptp_offset_100, uint, 0644);
MODULE_PARM_DESC(aq_ptp_offset_100, "PTP offset for 100M");

static unsigned int aq_ptp_offset_1000 = 0;
module_param(aq_ptp_offset_1000, uint, 0644);
MODULE_PARM_DESC(aq_ptp_offset_1000, "PTP offset for 1G");

static unsigned int aq_ptp_offset_2500 = 0;
module_param(aq_ptp_offset_2500, uint, 0644);
MODULE_PARM_DESC(aq_ptp_offset_2500, "PTP offset for 2,5G");

static unsigned int aq_ptp_offset_5000 = 0;
module_param(aq_ptp_offset_5000, uint, 0644);
MODULE_PARM_DESC(aq_ptp_offset_5000, "PTP offset for 5G");

static unsigned int aq_ptp_offset_10000 = 0;
module_param(aq_ptp_offset_10000, uint, 0644);
MODULE_PARM_DESC(aq_ptp_offset_10000, "PTP offset for 10G");

#define POLL_SYNC_TIMER_MS 15

/* Coefficients of PID. Multiplier and divider are used for distinguish
 * more accuracy while calculating PID parts
 */
#define PTP_MULT_COEF_P  15LL
#define PTP_MULT_COEF_I   5LL
#define PTP_MULT_COEF_D   1LL
#define PTP_DIV_COEF     10LL
#define PTP_DIV_RATIO   100LL

#define ACCURACY  20

static unsigned int aq_ptp_gpio_hightime = 100000;
module_param_named(aq_ptp_gpio_hightime, aq_ptp_gpio_hightime, uint, 0644);
MODULE_PARM_DESC(aq_ptp_gpio_hightime, "PTP GPIO high time");

static unsigned int aq_ptm_synopsys_fix = 1;
module_param_named(aq_ptm_synopsys_fix, aq_ptm_synopsys_fix, uint, 0644);
MODULE_PARM_DESC(aq_ptm_synopsys_fix, "PTM pdelay endianess fix");

enum ptp_extts_action {
	ptp_extts_disabled = 0,
	ptp_extts_user,
	ptp_extts_timesync,
	ptp_extts_freqsync
};

enum ptp_speed_offsets {
	ptp_offset_idx_10 = 0,
	ptp_offset_idx_100,
	ptp_offset_idx_1000,
	ptp_offset_idx_2500,
	ptp_offset_idx_5000,
	ptp_offset_idx_10000,
};

struct ptp_skb_ring {
	struct sk_buff **buff;
	spinlock_t lock;
	unsigned int size;
	volatile unsigned int head;
	volatile unsigned int tail;
};

struct ptp_tx_timeout {
	spinlock_t lock;
	bool active;
	unsigned long tx_start;
};

#define PTM_MAX_EVENTS 128
struct ptm_event_queue {
	struct ptm_data buf[PTM_MAX_EVENTS];
	int head;
	int tail;
	spinlock_t lock;
};

struct aq_ptp_pid {
	bool first_diff;
	uint64_t last_sync1588_ts;
	u64 ext_sync_period;

	/*PID related values*/
	s64 delta[3];
	s64 adjust[2];

	/*Describes ratio of current period to 1s*/
	s64 multiplier;
	s64 divider;
	/*end of PID related values*/
};

struct aq_ptp_s {
	struct aq_nic_s *aq_nic;

	struct hwtstamp_config hwtstamp_config;

	spinlock_t ptp_lock;
	spinlock_t ptp_ring_lock;
	struct ptp_clock *ptp_clock;
	struct ptp_clock_info ptp_info;

	atomic_t offset_egress;
	atomic_t offset_ingress;

	struct aq_ring_param_s ptp_ring_param;

	struct ptp_tx_timeout ptp_tx_timeout;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0)
	unsigned int ptp_vector;
	unsigned int gpio_vector;
#endif
	unsigned int idx_ptp_vector;
	unsigned int idx_gpio_vector;
	struct napi_struct napi;

	struct aq_ring_s ptp_tx;
	struct aq_ring_s ptp_rx;
	struct aq_ring_s hwts_rx;

	struct ptp_skb_ring skb_ring;

	u32 ptp_clock_sel; //TSG clock selection: 0 - PTP, 1 - PTM

	bool a1_ptp;
	bool a2_ptp;

	u32 udp_loc[4];
	u32 et_loc;

#ifdef PTM_SUPPORT
	bool ptm_en;
	struct ptm_event_queue ptmq;
	wait_queue_head_t ptm_wq;
	u64 last_hw_ts;
#endif
	struct delayed_work poll_sync;
	u32 poll_timeout_ms;

	u64 sync_time_value;

	struct aq_ptp_pid pid;
};

struct ptp_tm_offset {
	u64 sync_time_value;
	u64 ext_sync_period;

	unsigned int mbps;
	int egress;
	int ingress;
};

static struct ptp_tm_offset ptp_offset[6];
static struct ptp_clock_info aq_ptp_clock;

static inline int ptm_queue_cnt(struct ptm_event_queue *q)
{
	int cnt = q->tail - q->head;
	return cnt < 0 ? PTM_MAX_EVENTS + cnt : cnt;
}

static inline int ptm_queue_free(struct ptm_event_queue *q)
{
	return PTM_MAX_EVENTS - ptm_queue_cnt(q) - 1;
}

static void enqueue_ptm_event(struct ptm_event_queue *queue,
				       		  struct ptm_data *src)
{
	struct ptm_data *dst;
	unsigned long flags;

	spin_lock_irqsave(&queue->lock, flags);

	dst = &queue->buf[queue->tail];
	*dst = *src;

	if (!ptm_queue_free(queue))
		queue->head = (queue->head + 1) % PTM_MAX_EVENTS;

	queue->tail = (queue->tail + 1) % PTM_MAX_EVENTS;

	spin_unlock_irqrestore(&queue->lock, flags);
}

static inline int aq_ptp_tm_offset_egress_get(struct aq_ptp_s *self)
{
	return atomic_read(&self->offset_egress);
}

static inline int aq_ptp_tm_offset_ingress_get(struct aq_ptp_s *self)
{
	return atomic_read(&self->offset_ingress);
}

void aq_ptp_offset_get(unsigned int mbps, int *egress, int *ingress)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(ptp_offset); i++) {
		if (mbps == ptp_offset[i].mbps) {
			*egress = ptp_offset[i].egress;
			*ingress = ptp_offset[i].ingress;
			break;
		}
	}
}


void aq_ptp_tm_offset_set(struct aq_nic_s *aq_nic, unsigned int mbps)
{
	struct aq_ptp_s *self = aq_nic->aq_ptp;
	int i, egress, ingress;

	if (!self)
		return;

	egress = 0;
	ingress = 0;

	for (i = 0; i < ARRAY_SIZE(ptp_offset); i++) {
		if (mbps == ptp_offset[i].mbps) {
			egress = ptp_offset[i].egress;
			ingress = ptp_offset[i].ingress;
			break;
		}
	}

	atomic_set(&self->offset_egress, egress);
	atomic_set(&self->offset_ingress, ingress);
}

static int __aq_ptp_skb_put(struct ptp_skb_ring *ring, struct sk_buff *skb)
{
	unsigned int next_head = (ring->head + 1) % ring->size;

	if (next_head == ring->tail)
		return -1;

	ring->buff[ring->head] = skb_get(skb);
	ring->head = next_head;

	return 0;
}

static int aq_ptp_skb_put(struct ptp_skb_ring *ring, struct sk_buff *skb)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ring->lock, flags);
	ret = __aq_ptp_skb_put(ring, skb);
	spin_unlock_irqrestore(&ring->lock, flags);

	return ret;
}

static struct sk_buff *__aq_ptp_skb_get(struct ptp_skb_ring *ring)
{
	struct sk_buff *skb;

	if (ring->tail == ring->head)
		return NULL;

	skb = ring->buff[ring->tail];
	ring->tail = (ring->tail + 1) % ring->size;

	return skb;
}

static struct sk_buff *aq_ptp_skb_get(struct ptp_skb_ring *ring)
{
	unsigned long flags;
	struct sk_buff *skb;

	spin_lock_irqsave(&ring->lock, flags);
	skb = __aq_ptp_skb_get(ring);
	spin_unlock_irqrestore(&ring->lock, flags);

	return skb;
}

static unsigned int aq_ptp_skb_buf_len(struct ptp_skb_ring *ring)
{
	unsigned long flags;
	unsigned int len;

	spin_lock_irqsave(&ring->lock, flags);
	len = (ring->head >= ring->tail) ?
	ring->head - ring->tail :
	ring->size - ring->tail + ring->head;
	spin_unlock_irqrestore(&ring->lock, flags);

	return len;
}

static int aq_ptp_skb_ring_init(struct ptp_skb_ring *ring, unsigned int size)
{
	struct sk_buff **buff = kmalloc(sizeof(*buff) * size, GFP_KERNEL);

	if (!buff)
		return -ENOMEM;

	spin_lock_init(&ring->lock);

	ring->buff = buff;
	ring->size = size;
	ring->head = 0;
	ring->tail = 0;

	return 0;
}

static void aq_ptp_skb_ring_clean(struct ptp_skb_ring *ring)
{
	struct sk_buff *skb;

	while ((skb = aq_ptp_skb_get(ring)) != NULL)
		dev_kfree_skb_any(skb);
}

static void aq_ptp_skb_ring_release(struct ptp_skb_ring *ring)
{
	if (ring->buff) {
		aq_ptp_skb_ring_clean(ring);
		kfree(ring->buff);
		ring->buff = NULL;
	}
}

static void aq_ptp_tx_timeout_init(struct ptp_tx_timeout *timeout)
{
	spin_lock_init(&timeout->lock);
	timeout->active = false;
}

static void aq_ptp_tx_timeout_start(struct aq_ptp_s *self)
{
	struct ptp_tx_timeout *timeout = &self->ptp_tx_timeout;
	unsigned long flags;

	spin_lock_irqsave(&timeout->lock, flags);
	timeout->active = true;
	timeout->tx_start = jiffies;
	spin_unlock_irqrestore(&timeout->lock, flags);
}

static void aq_ptp_tx_timeout_update(struct aq_ptp_s *self)
{
	if (!aq_ptp_skb_buf_len(&self->skb_ring)) {
		struct ptp_tx_timeout *timeout = &self->ptp_tx_timeout;
		unsigned long flags;

		spin_lock_irqsave(&timeout->lock, flags);
		timeout->active = false;
		spin_unlock_irqrestore(&timeout->lock, flags);
	}
}

static void aq_ptp_tx_timeout_check(struct aq_ptp_s *self)
{
	struct ptp_tx_timeout *timeout = &self->ptp_tx_timeout;
	unsigned long flags;
	bool timeout_flag;

	timeout_flag = false;

	spin_lock_irqsave(&timeout->lock, flags);
	if (timeout->active) {
		timeout_flag = time_is_before_jiffies(timeout->tx_start +
						      AQ_PTP_TX_TIMEOUT);
		/* reset active flag if timeout detected */
		if (timeout_flag)
			timeout->active = false;
	}
	spin_unlock_irqrestore(&timeout->lock, flags);

	if (timeout_flag) {
		aq_ptp_skb_ring_clean(&self->skb_ring);
		netdev_err(self->aq_nic->ndev,
			   "PTP Timeout. Clearing Tx Timestamp SKBs\n");
	}
}

/* aq_ptp_adjfreq
 * @ptp: the ptp clock structure
 * @ppb: parts per billion adjustment from base
 *
 * adjust the frequency of the ptp cycle counter by the
 * indicated ppb from the base frequency.
 */
static int aq_ptp_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	struct aq_ptp_s *self = container_of(ptp, struct aq_ptp_s, ptp_info);
	struct aq_nic_s *aq_nic = self->aq_nic;

	mutex_lock(&aq_nic->fwreq_mutex);
	aq_nic->aq_hw_ops->hw_adj_clock_freq(aq_nic->aq_hw, ppb);
	mutex_unlock(&aq_nic->fwreq_mutex);

	return 0;
}

/* aq_ptp_adjtime
 * @ptp: the ptp clock structure
 * @delta: offset to adjust the cycle counter by
 *
 * adjust the timer by resetting the timecounter structure.
 */
static int aq_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct aq_ptp_s *self = container_of(ptp, struct aq_ptp_s, ptp_info);
	struct aq_nic_s *aq_nic = self->aq_nic;
	unsigned long flags;

	spin_lock_irqsave(&self->ptp_lock, flags);
	aq_nic->aq_hw_ops->hw_adj_sys_clock(aq_nic->aq_hw, delta);
	spin_unlock_irqrestore(&self->ptp_lock, flags);

	return 0;
}

/* aq_ptp_gettime
 * @ptp: the ptp clock structure
 * @ts: timespec structure to hold the current time value
 *
 * read the timecounter and return the correct value on ns,
 * after converting it into a struct timespec.
 */
static int aq_ptp_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	struct aq_ptp_s *self = container_of(ptp, struct aq_ptp_s, ptp_info);
	struct aq_nic_s *aq_nic = self->aq_nic;
	unsigned long flags;
	u64 ns;

	spin_lock_irqsave(&self->ptp_lock, flags);
	aq_nic->aq_hw_ops->hw_get_ptp_ts(aq_nic->aq_hw, &ns);
	spin_unlock_irqrestore(&self->ptp_lock, flags);

	*ts = ns_to_timespec64(ns);

	return 0;
}

/* aq_ptp_settime
 * @ptp: the ptp clock structure
 * @ts: the timespec containing the new time for the cycle counter
 *
 * reset the timecounter to use a new base value instead of the kernel
 * wall timer value.
 */
static int aq_ptp_settime(struct ptp_clock_info *ptp,
			  const struct timespec64 *ts)
{
	struct aq_ptp_s *self = container_of(ptp, struct aq_ptp_s, ptp_info);
	struct aq_nic_s *aq_nic = self->aq_nic;
	unsigned long flags;
	u64 ns = timespec64_to_ns(ts);
	u64 now;

	spin_lock_irqsave(&self->ptp_lock, flags);
	aq_nic->aq_hw_ops->hw_get_ptp_ts(aq_nic->aq_hw, &now);
	aq_nic->aq_hw_ops->hw_adj_sys_clock(aq_nic->aq_hw, (s64)ns - (s64)now);

	spin_unlock_irqrestore(&self->ptp_lock, flags);

	return 0;
}

static void aq_ptp_convert_to_hwtstamp(struct aq_ptp_s *self,
				       struct skb_shared_hwtstamps *hwtstamp,
				       u64 timestamp)
{
	memset(hwtstamp, 0, sizeof(*hwtstamp));
	hwtstamp->hwtstamp = ns_to_ktime(timestamp);
}

//PTM
#ifdef PTM_SUPPORT
int aq_ptp_ptm_enable(struct aq_ptp_s *self, struct ifreq *ifr)
{
	struct aq_nic_s *aq_nic;
	int err = 0;
	u32 timeout = 0; //timeout in ms == -1 - disable

	if (!self || !self->a2_ptp)
		return -EINVAL; //No support

	if( ifr ) {
		if (copy_from_user(&timeout, ifr->ifr_data, sizeof(u32)))
			return -EFAULT;
	}
	aq_nic = self->aq_nic;

	if( timeout ) {
		self->ptm_en = 1U;
		self->last_hw_ts = 0;
	}
	//TODO FW call instead
	aq_nic->aq_hw_ops->hw_ext_interrupr_en(aq_nic->aq_hw, !!timeout, AQ_HW_PTP_EXT_INT_PTM);
	aq_nic->aq_hw_ops->hw_ptm_enable(aq_nic->aq_hw, timeout, aq_ptm_synopsys_fix);
	if( !timeout )
		self->ptm_en = 0U;
	netdev_info(aq_nic->ndev, "PTM %sabled. timeout %d\n", self->ptm_en ? "en" : "dis", timeout);
	return err;
}

int aq_ptp_read_ptm_info(struct aq_ptp_s *self, struct ifreq *ifr)
{
	struct ptm_event_queue *queue;
	struct ptm_data ptm_event;
	unsigned long flags;
	int result = 0;

	if (!self || !self->a2_ptp || !self->ptm_en)
		return -EINVAL; //No support

	queue = &self->ptmq;
	//if (mutex_lock_interruptible(&ptp->ptmq_mux))
	//	return -ERESTARTSYS;

	if( wait_event_interruptible(self->ptm_wq,
	    !self->ptm_en || ptm_queue_cnt(queue)) ) {
		//mutex_unlock(&ptp->ptmq_mux);
		return -ERESTARTSYS;
	}

	if (!self->ptm_en) {
		//mutex_unlock(&ptp->ptmq_mux);
		return -ENODEV;
	}

	spin_lock_irqsave(&queue->lock, flags);
/*	
	qcnt = queue_cnt(queue);

	if (cnt > qcnt)
		cnt = qcnt;

	for (i = 0; i < cnt; i++) {
		event[i] = queue->buf[queue->head];
		queue->head = (queue->head + 1) % PTM_MAX_EVENTS;
	}
*/
	ptm_event = queue->buf[queue->head];
	queue->head = (queue->head + 1) % PTM_MAX_EVENTS;
	spin_unlock_irqrestore(&queue->lock, flags);

	//mutex_unlock(&ptp->tsevq_mux);
	if (copy_to_user(ifr->ifr_data, &ptm_event, sizeof(ptm_event)))
		result = -EFAULT;

	return result;
}

static void aq_ptp_ptm_get_info(struct aq_nic_s *aq_nic, struct ptm_data *data) //timeout == -1 - disable
{
	struct aq_ptp_s *self = aq_nic->aq_ptp;
	if (!self)
		return;

	memset(data, 0, sizeof(struct ptm_data));
	if(self->a2_ptp)
		aq_nic->aq_hw_ops->hw_ptm_get_info(aq_nic->aq_hw, data);
}
#endif //PTM_SUPPORT

static bool aq_ptp_event_ts_updated(struct aq_ptp_s *self, u32 clk_sel, u64 prev_ts, u64 *new_ts, u32 *cnt)
{
	struct aq_nic_s *aq_nic = self->aq_nic;
	uint64_t event_ts2;
	uint64_t event_ts;

	event_ts = aq_nic->aq_hw_ops->hw_ptp_gpio_get_event(aq_nic->aq_hw, clk_sel, cnt);
	if (event_ts != prev_ts) {
		event_ts2 = aq_nic->aq_hw_ops->hw_ptp_gpio_get_event(aq_nic->aq_hw, clk_sel, cnt);
		if (event_ts != event_ts2) {
			event_ts = event_ts2;
			event_ts2 = aq_nic->aq_hw_ops->hw_ptp_gpio_get_event(aq_nic->aq_hw, clk_sel, cnt);
			if (event_ts != event_ts2) {
				netdev_err(aq_nic->ndev,
					   "%s: Unable to get correct GPIO TS",
					   __func__);
				event_ts = 0;
			}
		}

		*new_ts = event_ts;
		return true;
	}
	return false;
}

bool aq_ptp_ts_valid(struct aq_ptp_pid *aq_pid, u64 diff)
{
	/* check we get valid TS, let's use simple check: if difference of
	 * ts_diff and expected period more than half of expected period it
	 * means we've got invalid TS
	 */
	return abs((int64_t)diff - aq_pid->ext_sync_period) <
	       aq_pid->ext_sync_period / 3;
}

static void aq_ptp_pid_reset(struct aq_ptp_pid *aq_pid)
{
	memset(aq_pid->delta, 0, sizeof(aq_pid->delta));
	memset(aq_pid->adjust, 0, sizeof(aq_pid->adjust));
	aq_pid->first_diff = true;
}

static int aq_ptp_pid(struct aq_ptp_s *self, u64 ts_diff)
{
	s64 p, integral, diff;
	struct aq_ptp_pid *aq_pid = &self->pid;
	if (aq_pid->first_diff) {
		aq_pid->first_diff = false;
		return 0;
	}

	if (!aq_ptp_ts_valid(aq_pid, ts_diff)) {
		netdev_err(self->aq_nic->ndev,
			"Invalid TS got, reset synchronization"
			" algorithm: TS diff: %llu,"
			" expected: about %llu",
			ts_diff, aq_pid->ext_sync_period);
		aq_ptp_pid_reset(aq_pid);
		aq_ptp_adjfreq(&self->ptp_info, 0);
		return 0;
	}
	aq_pid->delta[0] += ts_diff;
	aq_pid->delta[0] -= aq_pid->ext_sync_period;

	p = PTP_MULT_COEF_P * aq_pid->multiplier *
		(aq_pid->delta[0] - aq_pid->delta[1]);
	integral = PTP_MULT_COEF_I * aq_pid->multiplier * aq_pid->delta[1];
	diff = PTP_MULT_COEF_D * aq_pid->multiplier *
		   (aq_pid->delta[0] - 2 * aq_pid->delta[1] + aq_pid->delta[2]);

	netdev_dbg(self->aq_nic->ndev,
		   "p = %lld, integral = %lld, diff = %lld",
		   p / PTP_DIV_COEF / aq_pid->divider,
		   integral / PTP_DIV_COEF / aq_pid->divider,
		   diff / PTP_DIV_COEF / aq_pid->divider);
	aq_pid->adjust[0] = (p + integral + diff) / PTP_DIV_COEF /
			    aq_pid->divider + aq_pid->adjust[1];

	aq_pid->adjust[1] = aq_pid->adjust[0];
	aq_pid->delta[2] = aq_pid->delta[1];
	aq_pid->delta[1] = aq_pid->delta[0];
	netdev_dbg(self->aq_nic->ndev, "delta = %lld, adjust = %lld",
		   aq_pid->delta[0], aq_pid->adjust[0]);

	/* Apply adjust in case if current delta more than 20 or
	* changing of delta more than 20 (speed of delta
	* changing)
	*/
	if (abs(aq_pid->delta[0]) > ACCURACY ||
		abs(aq_pid->delta[1] - aq_pid->delta[2]) > ACCURACY)
		aq_ptp_adjfreq(&self->ptp_info,
				-aq_pid->adjust[0]);

	return 0;
}

/* Check whether sync1588 pin was triggered, and set stored new PTP time */
static int aq_ptp_check_ext_gpio_event(struct aq_ptp_s *self)
{
	struct aq_nic_s *aq_nic = self->aq_nic;
	int repeat_event = 0;
	int n_pin;
	for( n_pin = 0; n_pin < aq_ptp_clock.n_pins; n_pin++ ) {
		if( aq_ptp_clock.pin_config[n_pin].func == PTP_PF_EXTTS ){
			u32 cnt = 0;
			u64 prev_ts = ((uint64_t *)aq_ptp_clock.pin_config[n_pin].rsv)[0];
			enum ptp_extts_action action = aq_ptp_clock.pin_config[n_pin].rsv[2];
			u64 ts = prev_ts;
			
			repeat_event = 1;
			/* Sync1588 pin was triggered */
			if (aq_ptp_event_ts_updated(self, self->ptp_clock_sel, prev_ts, &ts, &cnt)) {
				u64 ts_diff = ts - prev_ts;
				netdev_dbg(aq_nic->ndev, "%s: pin %d with act %x triggered TS: %llu, prev TS %llu, diff %llu",
					__func__, n_pin, action, ts, prev_ts, ts_diff);
				switch(action) {
				case ptp_extts_timesync: 
					{
						unsigned long flags;
						spin_lock_irqsave(&self->ptp_lock, flags);
						aq_nic->aq_hw_ops->hw_set_sys_clock(aq_nic->aq_hw,
										self->sync_time_value,
										ts);
						spin_unlock_irqrestore(&self->ptp_lock, flags);
						action = ptp_extts_disabled;
						repeat_event = 0;
					}
					break;
				case ptp_extts_user: 
					{
						struct ptp_clock_event ptp_event;
						ptp_event.type = PTP_CLOCK_EXTTS;
						ptp_event.index = self->a2_ptp ? n_pin : self->ptp_info.n_pins - 1;
						ptp_event.timestamp = ts;
						ptp_clock_event(self->ptp_clock, &ptp_event);
					};
					break;
				case ptp_extts_freqsync:
					if( aq_ptp_pid(self, ts_diff) ) {
						repeat_event = 0;
					}
					break;
				default:
					break;
				}
				((uint64_t *)aq_ptp_clock.pin_config[n_pin].rsv)[0]	= ts;
				aq_ptp_clock.pin_config[n_pin].rsv[2] = action;
			}
		}
	}

	return repeat_event;
}

/* PTP external GPIO nanoseconds count */
void aq_ptp_poll_sync_work_cb(struct work_struct *w)
{
	struct delayed_work *dw = to_delayed_work(w);
	struct aq_ptp_s *self = container_of(dw, struct aq_ptp_s, poll_sync);

	if( aq_ptp_check_ext_gpio_event(self) ) {
		unsigned long timeout = msecs_to_jiffies(self->poll_timeout_ms);
		schedule_delayed_work(&self->poll_sync, timeout);
	}

}


static int aq_ptp_hw_pin_conf(struct aq_nic_s *aq_nic, u32 pin_index, u64 start,
			      u64 period)
{
	struct aq_ptp_s *self = aq_nic->aq_ptp;
	if (period)
		netdev_info(aq_nic->ndev,
			"Enable GPIO %d pulsing, start time %llu, period %u\n",
			pin_index, start, (u32)period);
	else
		netdev_info(aq_nic->ndev,
			"Disable GPIO %d pulsing, start time %llu, period %u\n",
			pin_index, start, (u32)period);

	/* Notify hardware of request to being sending pulses.
	 * If period is ZERO then pulsen is disabled.
	 */
	mutex_lock(&aq_nic->fwreq_mutex);
	aq_nic->aq_hw_ops->hw_gpio_pulse(aq_nic->aq_hw, pin_index,
					self->ptp_clock_sel, start, (u32)period, aq_ptp_gpio_hightime);
	mutex_unlock(&aq_nic->fwreq_mutex);

	return 0;
}

static int aq_ptp_perout_pin_configure(struct ptp_clock_info *ptp,
				       struct ptp_clock_request *rq, int on)
{
	struct aq_ptp_s *self = container_of(ptp, struct aq_ptp_s, ptp_info);
	struct ptp_clock_time *t = &rq->perout.period;
	struct ptp_clock_time *s = &rq->perout.start;
	struct aq_nic_s *aq_nic = self->aq_nic;
	u64 start, period;
	u32 n_pin = rq->perout.index;

	/* verify the request channel is there */
	if (n_pin >= ptp->n_per_out)
		return -EINVAL;

	if ( aq_ptp_clock.pin_config[n_pin].func != PTP_PF_PEROUT )
		return -EINVAL;

	/* we cannot support periods greater
	 * than 4 seconds due to reg limit
	 */
	if (t->sec > 4 || t->sec < 0)
		return -ERANGE;

	/* convert to unsigned 64b ns,
	 * verify we can put it in a 32b register
	 */
	period = on ? t->sec * 1000000000LL + t->nsec : 0;

	/* verify the value is in range supported by hardware */
	if (period > U32_MAX)
		return -ERANGE;
	/* convert to unsigned 64b ns */
	/* TODO convert to AQ time */
	start = on ? s->sec * 1000000000LL + s->nsec : 0;

	aq_ptp_hw_pin_conf(aq_nic, aq_ptp_clock.pin_config[n_pin].rsv[3], start, period);

	return 0;
}

static int aq_ptp_pps_pin_configure(struct ptp_clock_info *ptp, int on)
{
	struct aq_ptp_s *self = container_of(ptp, struct aq_ptp_s, ptp_info);
	struct aq_nic_s *aq_nic = self->aq_nic;
	u64 start, period;
	u64 rest = 0;

	u32 n_pin;

	for( n_pin = 0; n_pin < ptp->n_per_out; n_pin++ ) {
		if( aq_ptp_clock.pin_config[n_pin].func == PTP_PF_PEROUT )
			break;
	}
	/* verify the request channel is there */
	if (n_pin >= ptp->n_per_out)
		return -EINVAL;
	
	aq_nic->aq_hw_ops->hw_get_ptp_ts(aq_nic->aq_hw, &start);
	rest = start % 1000000000LL;
	period = on ? 1000000000LL : 0; /* PPS - pulse per second */
	start = on ? start - rest + 1000000000LL *
		(rest > 990000000LL ? 2 : 1) : 0;

	aq_ptp_hw_pin_conf(aq_nic, aq_ptp_clock.pin_config[n_pin].rsv[3], start, period);

	return 0;
}

static int aq_ptp_extts_pin_configure(struct ptp_clock_info *ptp,
			u32 n_pin, enum ptp_extts_action action)
{
	struct aq_ptp_s *self = container_of(ptp, struct aq_ptp_s, ptp_info);
	struct aq_nic_s *aq_nic = self->aq_nic;
	u32 on = action != ptp_extts_disabled;

	if( !aq_ptp_clock.n_ext_ts || n_pin >= (aq_ptp_clock.n_ext_ts + aq_ptp_clock.n_per_out) )
		return -EINVAL;

	if( self->a1_ptp ) {
		n_pin = aq_ptp_clock.n_pins - 1; // TODO more clear selection
	}

	if ( aq_ptp_clock.pin_config[n_pin].func != PTP_PF_EXTTS )
		return -EINVAL;

	if( self->a1_ptp ) {
		cancel_delayed_work_sync(&self->poll_sync);
	} else {
		if( aq_nic->aq_hw_ops->hw_ext_interrupr_en ) {
			aq_nic->aq_hw_ops->hw_ext_interrupr_en(aq_nic->aq_hw, on, AQ_HW_PTP_EXT_INT_GPIO0 << n_pin);
		}
	}

	aq_ptp_clock.pin_config[n_pin].rsv[2] = action;

	((uint64_t *)aq_ptp_clock.pin_config[n_pin].rsv)[0] = \
				aq_nic->aq_hw_ops->hw_ptp_gpio_get_event(aq_nic->aq_hw, self->ptp_clock_sel, NULL);

	netdev_info(aq_nic->ndev, "GPIO %d input event: %s.\n", n_pin,
		action == ptp_extts_disabled ? "disabled" :
		action == ptp_extts_user ? "requested" :
		action == ptp_extts_timesync ? "time sync" :
		action == ptp_extts_freqsync ? "freq sync" : "unknown");

	aq_nic->aq_hw_ops->hw_extts_gpio_enable(aq_nic->aq_hw, 
					aq_ptp_clock.pin_config[n_pin].rsv[3], self->ptp_clock_sel, on);

	if( self->a1_ptp && on && aq_nic->aq_hw->aq_link_status.mbps ) {
		if( action != ptp_extts_freqsync ) {
			self->poll_timeout_ms = POLL_SYNC_TIMER_MS;
		}
		//Here shoiuld be request interupt from firmware
		schedule_delayed_work(&self->poll_sync,
					msecs_to_jiffies(self->poll_timeout_ms));
	}
	return 0;
}

/* aq_ptp_gpio_feature_enable
 * @ptp: the ptp clock structure
 * @rq: the requested feature to change
 * @on: whether to enable or disable the feature
 */
static int aq_ptp_gpio_feature_enable(struct ptp_clock_info *ptp,
				      struct ptp_clock_request *rq, int on)
{
	switch (rq->type) {
	case PTP_CLK_REQ_EXTTS:
		return aq_ptp_extts_pin_configure(ptp, rq->extts.index, on ? ptp_extts_user : ptp_extts_disabled);
	case PTP_CLK_REQ_PEROUT:
		return aq_ptp_perout_pin_configure(ptp, rq, on);
	case PTP_CLK_REQ_PPS:
		return aq_ptp_pps_pin_configure(ptp, on);
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

/* Store new PTP time and wait until gpio pin triggered */
int aq_ptp_configure_ext_gpio(struct net_device *ndev,
			  struct aq_ptp_ext_gpio_event *ext_gpio_evnt)
{
	struct aq_nic_s *aq_nic = NULL;
	struct aq_ptp_s *self = aq_nic->aq_ptp;
	enum ptp_extts_action action = ptp_extts_disabled;
	int err = 0;
	u32 n_pin = ext_gpio_evnt->gpio_index;

	if (!ndev)
		return -EINVAL;

	if (ndev->ethtool_ops != &aq_ethtool_ops)
		return -EINVAL;

	aq_nic = netdev_priv(ndev);
	if (!aq_nic)
		return -EINVAL;

	if (!self) {
		err = -ENOTSUPP;
		goto err_exit;
	}

	if( !aq_ptp_clock.n_ext_ts || n_pin >= (aq_ptp_clock.n_ext_ts + aq_ptp_clock.n_per_out) ){
		netdev_info(aq_nic->ndev,
			"Not supported, selected EXT TS pin was not advertised");
		err = -ENOTSUPP;
		goto err_exit;
	}

	if( self->a1_ptp ) {
		//Not required if FW supports
		cancel_delayed_work_sync(&self->poll_sync);
		self->poll_timeout_ms = POLL_SYNC_TIMER_MS;
	}

	switch (ext_gpio_evnt->action) {
	case aq_sync_cntr_set:
		netdev_info(aq_nic->ndev, "Enable sync time on event:%llu",
			    ext_gpio_evnt->time_ns);
		action = ptp_extts_timesync;
		self->sync_time_value = ext_gpio_evnt->time_ns;
		break;
	case 0:
		break;
	default:
		err = -ENOTSUPP;
		goto err_exit;
	}

	if (ext_gpio_evnt->clock_sync_en) {
		if (ext_gpio_evnt->sync_pulse_ms < 50 ||
		    ext_gpio_evnt->sync_pulse_ms > MSEC_PER_SEC) {
			netdev_err(aq_nic->ndev,
				   "Sync pulse ms should not be equal less"
				   " than 50ms or higher than 1s");
			err = -EINVAL;
			goto err_exit;
		}

		netdev_info(aq_nic->ndev,
			    "Enable sync clock with ext signal with period: %u",
			    ext_gpio_evnt->sync_pulse_ms);
		action = ptp_extts_freqsync;
		self->pid.ext_sync_period = (uint64_t)ext_gpio_evnt->sync_pulse_ms *
					  NSEC_PER_MSEC;

		self->pid.multiplier = NSEC_PER_SEC * PTP_DIV_RATIO /
				     self->pid.ext_sync_period;
		self->pid.divider = PTP_DIV_RATIO;

		/* If we need only clock sync poll TS at least
		 * 3 times per period
		 */
		self->poll_timeout_ms = self->pid.ext_sync_period /
						NSEC_PER_MSEC / 3;
	}
	
	if( action == ptp_extts_disabled) {
		netdev_info(aq_nic->ndev, "Disable sync time/freq on event");
		aq_ptp_pid_reset(&self->pid);
	}

	aq_ptp_extts_pin_configure(&self->ptp_info, ext_gpio_evnt->gpio_index, action);

err_exit:
	return err;
}

/* aq_ptp_verify
 * @ptp: the ptp clock structure
 * @pin: index of the pin in question
 * @func: the desired function to use
 * @chan: the function channel index to use
 */
static int aq_ptp_verify(struct ptp_clock_info *ptp, unsigned int n_pin,
			 enum ptp_pin_function func, unsigned int chan)
{
	struct aq_ptp_s *self = container_of(ptp, struct aq_ptp_s, ptp_info);
	/* verify the requested pin is there */
	if (n_pin >= ptp->n_pins)
		return -EINVAL;

	/* we want to keep the functions locked as well */
	switch(func) {
		case PTP_PF_NONE:
		case PTP_PF_EXTTS:
			if( self->a1_ptp && n_pin < ptp->n_per_out ) {
				return -EINVAL; //A1: Only main GPIO may be PTP out, A2 all GPIO are bi-dir
			}
			break;
		case PTP_PF_PEROUT:
			if( n_pin >= ptp->n_per_out ) {
				return -EINVAL; //A1: Only main GPIO may be PTP out, A2 all GPIO are bi-dir
			}
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

/* aq_ptp_tx_hwtstamp - utility function which checks for TX time stamp
 * @adapter: the private adapter struct
 *
 * if the timestamp is valid, we convert it into the timecounter ns
 * value, then store that result into the shhwtstamps structure which
 * is passed up the network stack
 */
void aq_ptp_tx_hwtstamp(struct aq_nic_s *aq_nic, u64 timestamp)
{
	struct aq_ptp_s *self = aq_nic->aq_ptp;
	struct sk_buff *skb = aq_ptp_skb_get(&self->skb_ring);
	struct skb_shared_hwtstamps hwtstamp;

	if (!skb) {
		netdev_err(aq_nic->ndev, "have timestamp but tx_queus empty\n");
		return;
	}

	timestamp += aq_ptp_tm_offset_egress_get(self);
	aq_ptp_convert_to_hwtstamp(self, &hwtstamp, timestamp);
	skb_tstamp_tx(skb, &hwtstamp);
	dev_kfree_skb_any(skb);

	aq_ptp_tx_timeout_update(self);
}

static int aq_ptp_dpath_enable(struct aq_ptp_s *self, unsigned int enable, u16 rx_queue)
{
	int err = 0, i = 0;
	struct ethtool_rxnfc cmd = { 0 };
	struct aq_nic_s *aq_nic = self->aq_nic;
	struct ethtool_rx_flow_spec *fsp =
		(struct ethtool_rx_flow_spec *)&cmd.fs;

	netdev_info(aq_nic->ndev, "%sable ptp filters: %x.\n", enable ? "En" : "Dis", enable);
	if( enable ) {
		if( enable & (AQ_HW_PTP_L4_ENABLE ) ){
			if( self->a1_ptp || !aq_new_filters_enabled ) {
				fsp->ring_cookie = rx_queue;
				fsp->location = RX_CLS_LOC_ANY;
				fsp->flow_type = UDP_V4_FLOW;
				fsp->h_u.udp_ip4_spec.pdst = cpu_to_be16(PTP_EVENT_MESSAGE_PORT);
				fsp->m_u.udp_ip4_spec.pdst = cpu_to_be16(0xffff);
				err = aq_add_rxnfc_rule(aq_nic, &cmd);
				if( !err ) {
					self->udp_loc[i++] = fsp->location;
					netdev_info(aq_nic->ndev, "Set UDPv4 filter complete. Location: %x\n", fsp->location);
				}
				fsp->location = RX_CLS_LOC_ANY;
				fsp->flow_type = UDP_V6_FLOW;
				fsp->h_u.udp_ip6_spec.pdst = cpu_to_be16(PTP_EVENT_MESSAGE_PORT);
				fsp->m_u.udp_ip6_spec.pdst = cpu_to_be16(0xffff);
				err = aq_add_rxnfc_rule(aq_nic, &cmd);
				if( !err ) {
					self->udp_loc[i++] = fsp->location;
					netdev_info(aq_nic->ndev, "Set UDPv6 filter complete. Location: %x\n", fsp->location);
				}
			} else {
				fsp->ring_cookie = rx_queue;
				fsp->location = RX_CLS_LOC_ANY;
				fsp->flow_type = UDP_V4_FLOW;
				fsp->h_u.udp_ip4_spec.psrc = 0;
				fsp->m_u.udp_ip4_spec.psrc = 0;
				fsp->h_u.udp_ip4_spec.pdst = cpu_to_be16(PTP_EVENT_MESSAGE_PORT);
				fsp->m_u.udp_ip4_spec.pdst = cpu_to_be16(0xffff);
				fsp->h_u.udp_ip4_spec.ip4dst = cpu_to_be32(PTP_IPV4_MC_ADDR1);
				fsp->m_u.udp_ip4_spec.ip4dst = cpu_to_be32(0xffffffff);
				err = aq_add_rxnfc_rule(aq_nic, &cmd);
				if( !err ) {
					self->udp_loc[i++] = fsp->location;
					netdev_info(aq_nic->ndev, "Set UDPv4 filter complete. Location: %x\n", fsp->location);
				}
				memset(fsp, 0, sizeof(*fsp));
				fsp->ring_cookie = rx_queue;
				fsp->location = RX_CLS_LOC_ANY;
				fsp->flow_type = UDP_V6_FLOW;
				fsp->h_u.udp_ip6_spec.psrc = 0;
				fsp->m_u.udp_ip6_spec.psrc = 0;
				fsp->h_u.udp_ip6_spec.pdst = cpu_to_be16(PTP_EVENT_MESSAGE_PORT);
				fsp->m_u.udp_ip6_spec.pdst = cpu_to_be16(0xffff);
				fsp->h_u.udp_ip6_spec.ip6dst[0] = cpu_to_be32(PTP_IPV6_MC_ADDR20 << 16);
				fsp->m_u.udp_ip6_spec.ip6dst[0] = cpu_to_be32(0xffff0000);
				fsp->h_u.udp_ip6_spec.ip6dst[3] = cpu_to_be32(PTP_IPV6_MC_ADDR24);
				fsp->m_u.udp_ip6_spec.ip6dst[3] = cpu_to_be32(0x0000ffff);
				err = aq_add_rxnfc_rule(aq_nic, &cmd);
				if( !err ) {
					self->udp_loc[i++] = fsp->location;
					netdev_info(aq_nic->ndev, "Set UDPv6 filter complete. Location: %x\n", fsp->location);
				}
				memset(fsp, 0, sizeof(*fsp));
				fsp->ring_cookie = rx_queue;
				fsp->location = RX_CLS_LOC_ANY;
				fsp->flow_type = UDP_V6_FLOW;
				fsp->h_u.udp_ip6_spec.psrc = 0;
				fsp->m_u.udp_ip6_spec.psrc = 0;
				fsp->h_u.udp_ip6_spec.pdst = cpu_to_be16(PTP_EVENT_MESSAGE_PORT);
				fsp->m_u.udp_ip6_spec.pdst = cpu_to_be16(0xffff);
				fsp->h_u.udp_ip6_spec.ip6dst[0] = cpu_to_be32(PTP_IPV6_MC_ADDR10 << 16);
				fsp->m_u.udp_ip6_spec.ip6dst[0] = cpu_to_be32(0xffff0000);
				fsp->h_u.udp_ip6_spec.ip6dst[3] = cpu_to_be32(PTP_IPV6_MC_ADDR14);
				fsp->m_u.udp_ip6_spec.ip6dst[3] = cpu_to_be32(0x0000ffff);
				err = aq_add_rxnfc_rule(aq_nic, &cmd);
				if( !err ) {
					self->udp_loc[i++] = fsp->location;
					netdev_info(aq_nic->ndev, "Set UDPv6 filter complete. Location: %x\n", fsp->location);
				}
				memset(fsp, 0, sizeof(*fsp));
				fsp->ring_cookie = rx_queue;
				fsp->location = RX_CLS_LOC_ANY;
				fsp->flow_type = UDP_V4_FLOW;
				fsp->h_u.udp_ip4_spec.psrc = 0;
				fsp->m_u.udp_ip4_spec.psrc = 0;
				fsp->h_u.udp_ip4_spec.pdst = cpu_to_be16(PTP_EVENT_MESSAGE_PORT);
				fsp->m_u.udp_ip4_spec.pdst = cpu_to_be16(0xffff);
				fsp->h_u.udp_ip4_spec.ip4dst = cpu_to_be32(PTP_IPV4_MC_ADDR2);
				fsp->m_u.udp_ip4_spec.ip4dst = cpu_to_be32(0xffffffff);
				err = aq_add_rxnfc_rule(aq_nic, &cmd);
				if( !err ) {
					self->udp_loc[i++] = fsp->location;
					netdev_info(aq_nic->ndev, "Set UDPv4 filter complete. Location: %x\n", fsp->location);
				}
			}
		}
		if( !err && (enable & AQ_HW_PTP_L2_ENABLE) ){
			memset(fsp, 0, sizeof(*fsp));
			fsp->ring_cookie = rx_queue;
			fsp->location = RX_CLS_LOC_ANY;
			fsp->flow_type = ETHER_FLOW;
			fsp->h_u.ether_spec.h_proto = cpu_to_be16(ETH_P_1588);
			fsp->m_u.ether_spec.h_proto = 0xffff;
			netdev_info(aq_nic->ndev, "Set L2 filter\n");
			err = aq_add_rxnfc_rule(aq_nic, &cmd);
			if( !err ) {
				self->et_loc = fsp->location;
				netdev_info(aq_nic->ndev, "Set L2 filter complete. Location: %d\n", fsp->location);
			}
		}
	}
	else {
		for( i = 0; i < 4; i++ ) {
			if( self->udp_loc[i] > 0 && self->udp_loc[i] < RX_CLS_LOC_ANY ){
				fsp->location = self->udp_loc[i];
				err = aq_del_rxnfc_rule(aq_nic, &cmd);
			}
		}
		if( !err && self->et_loc > 0 && self->et_loc < RX_CLS_LOC_ANY ){
			fsp->location = self->et_loc;
			err = aq_del_rxnfc_rule(aq_nic, &cmd);
		}
	}
	return err;	
}

/* aq_ptp_rx_hwtstamp - utility function which checks for RX time stamp
 * @adapter: pointer to adapter struct
 * @skb: particular skb to send timestamp with
 *
 * if the timestamp is valid, we convert it into the timecounter ns
 * value, then store that result into the shhwtstamps structure which
 * is passed up the network stack
 */
static void aq_ptp_rx_hwtstamp(struct aq_ptp_s *self, struct sk_buff *skb,
			       u64 timestamp)
{
	timestamp -= aq_ptp_tm_offset_ingress_get(self);
	aq_ptp_convert_to_hwtstamp(self, skb_hwtstamps(skb), timestamp);
}

void aq_ptp_hwtstamp_config_get(struct aq_ptp_s *self,
				struct hwtstamp_config *config)
{
	*config = self->hwtstamp_config;
}

static unsigned int parse_rx_filters(enum hwtstamp_rx_filters rx_filter)
{
	unsigned int ptp_en_flags = AQ_HW_PTP_DISABLE;
	switch (rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		break;
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
		ptp_en_flags = AQ_HW_PTP_L2_ENABLE;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
		ptp_en_flags = AQ_HW_PTP_L4_ENABLE;
		break;
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		ptp_en_flags = AQ_HW_PTP_L4_ENABLE | AQ_HW_PTP_L2_ENABLE;
		break;
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
	case HWTSTAMP_FILTER_ALL:
		/* fall through */
	default:
		return ERANGE;
	}
	return ptp_en_flags;
}

int aq_ptp_hwtstamp_config_set(struct aq_ptp_s *self,
			       struct hwtstamp_config *config)
{
	struct aq_nic_s *aq_nic = self->aq_nic;
	int err = 0;
	unsigned int ptp_en_flags = parse_rx_filters(config->rx_filter);
	if( ptp_en_flags == ERANGE ) {
		config->rx_filter = HWTSTAMP_FILTER_NONE;
		return -ERANGE;
	}

	if( self->hwtstamp_config.rx_filter != config->rx_filter ) {
		err = aq_ptp_dpath_enable(self, ptp_en_flags, self->ptp_rx.idx);
	}
	if (/*(config->tx_type == HWTSTAMP_TX_ON) ||
		(config->rx_filter == HWTSTAMP_FILTER_PTP_V2_EVENT)*/
		ptp_en_flags != AQ_HW_PTP_DISABLE ) {
		aq_utils_obj_set(&aq_nic->flags, AQ_NIC_PTP_DPATH_UP);
	} else {
		aq_utils_obj_clear(&aq_nic->flags, AQ_NIC_PTP_DPATH_UP);
	}

	if (err)
		return -EREMOTEIO;

	self->hwtstamp_config = *config;

	return 0;
}

bool aq_ptp_ring(struct aq_ring_s *ring)
{
	struct aq_ptp_s *self = NULL;
	if( !ring || !ring->aq_nic )
		return false;
	self = ring->aq_nic->aq_ptp;
	if (!self)
		return false;

	return (&self->ptp_tx == ring) ||
	       (&self->ptp_rx == ring) || 
		   (self->a1_ptp && &self->hwts_rx == ring);
}

u16 aq_ptp_extract_ts(struct aq_nic_s *aq_nic, struct sk_buff *skb, u8 *p,
		      unsigned int len)
{
	struct aq_ptp_s *self = aq_nic->aq_ptp;
	u64 timestamp = 0;
	u16 ret = aq_nic->aq_hw_ops->rx_extract_ts(p, len, &timestamp);

	if (ret > 0)
		aq_ptp_rx_hwtstamp(self, skb, timestamp);

	return ret;
}

static int aq_ptp_poll(struct napi_struct *napi, int budget)
{
	struct aq_ptp_s *self = container_of(napi, struct aq_ptp_s, napi);
	struct aq_nic_s *aq_nic = self->aq_nic;
	const struct aq_hw_ops *hw_ops = aq_nic->aq_hw_ops;
	bool was_cleaned = false;
	int work_done = 0;
	int err;

	/* Processing PTP TX traffic */
	err = hw_ops->hw_ring_tx_head_update(aq_nic->aq_hw,
							&self->ptp_tx);
	if (err < 0)
		goto err_exit;

	if (self->ptp_tx.sw_head != self->ptp_tx.hw_head) {
		aq_ring_tx_clean(&self->ptp_tx);

		was_cleaned = true;
	}

	/* Processing HW_TIMESTAMP RX traffic */
	if( self->a1_ptp )
	{
		err = hw_ops->hw_ring_hwts_rx_receive(aq_nic->aq_hw,
						      &self->hwts_rx);
		if (err < 0)
			goto err_exit;

		if (self->hwts_rx.sw_head != self->hwts_rx.hw_head) {
			aq_ring_hwts_rx_clean(&self->hwts_rx, aq_nic);

			err = hw_ops->hw_ring_hwts_rx_fill(aq_nic->aq_hw,
							   &self->hwts_rx);

		was_cleaned = true;
		}
	}

	/* Processing PTP RX traffic */
	err = hw_ops->hw_ring_rx_receive(aq_nic->aq_hw, &self->ptp_rx);
	if (err < 0)
		goto err_exit;

	if (self->ptp_rx.sw_head != self->ptp_rx.hw_head) {
		unsigned int sw_tail_old;

		err = aq_ring_rx_clean(&self->ptp_rx, napi, &work_done, budget);
		if (err < 0)
			goto err_exit;

		sw_tail_old = self->ptp_rx.sw_tail;
		err = aq_ring_rx_fill(&self->ptp_rx);
		if (err < 0)
			goto err_exit;

		err = hw_ops->hw_ring_rx_fill(aq_nic->aq_hw, 
							 &self->ptp_rx,
							 sw_tail_old);
		if (err < 0)
			goto err_exit;
	}

	if (was_cleaned)
		work_done = budget;

	if (work_done < budget) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 19, 0)
		napi_complete_done(napi, work_done);
#else
		napi_complete(napi);
#endif
		hw_ops->hw_irq_enable(aq_nic->aq_hw,
					1 << self->ptp_ring_param.vec_idx);
	}

err_exit:
	return work_done;
}

static irqreturn_t aq_ext_ptp_isr(int irq, void *private)
{
	struct aq_ptp_s *self = private;
	const struct aq_hw_ops *hw_ops;
	int err = 0;
	hw_ops = self->aq_nic->aq_hw_ops;

	if (!self) {
		err = -EINVAL;
		goto err_exit;
	}

	aq_ptp_check_ext_gpio_event(self);

#ifdef PTM_SUPPORT
	if( self->ptm_en ) {
		struct ptm_data info;
		aq_ptp_ptm_get_info(self->aq_nic, &info);
		if( info.free_run_clock != self->last_hw_ts ) {
			self->last_hw_ts = info.free_run_clock;
			enqueue_ptm_event(&self->ptmq, &info);
			wake_up_interruptible(&self->ptm_wq);
		}
	}
#endif
	hw_ops->hw_irq_enable(self->aq_nic->aq_hw,
				1 << self->idx_gpio_vector);

err_exit:
	return err >= 0 ? IRQ_HANDLED : IRQ_NONE;
}

static irqreturn_t aq_ptp_isr(int irq, void *private)
{
	struct aq_ptp_s *self = private;
	int err = 0;

	if (!self) {
		err = -EINVAL;
		goto err_exit;
	}
	napi_schedule(&self->napi);

err_exit:
	return err >= 0 ? IRQ_HANDLED : IRQ_NONE;
}

int aq_ptp_xmit(struct aq_nic_s *aq_nic, struct sk_buff *skb)
{
	struct aq_ptp_s *self = aq_nic->aq_ptp;
	struct aq_ring_s *ring = &self->ptp_tx;
	unsigned long irq_flags;
	int err = NETDEV_TX_OK;
	unsigned int frags;

	if (skb->len <= 0) {
		dev_kfree_skb_any(skb);
		goto err_exit;
	}

	frags = skb_shinfo(skb)->nr_frags + 1;
	/* Frags cannot be bigger 16KB
	 * because PTP usually works
	 * without Jumbo even in a background
	 */
	if (frags > AQ_CFG_SKB_FRAGS_MAX || frags > aq_ring_avail_dx(ring)) {
		/* Drop packet because it doesn't make sence to delay it */
		dev_kfree_skb_any(skb);
		goto err_exit;
	}

	err = aq_ptp_skb_put(&self->skb_ring, skb);
	if (err) {
		netdev_err(aq_nic->ndev, "SKB Ring is overflow (%u)!\n",
			   ring->size);
		return NETDEV_TX_BUSY;
	}
	skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
	aq_ptp_tx_timeout_start(self);
	skb_tx_timestamp(skb);

	spin_lock_irqsave(&aq_nic->aq_ptp->ptp_ring_lock, irq_flags);
	frags = aq_nic_map_skb(aq_nic, skb, ring);

	if (likely(frags)) {
		err = aq_nic->aq_hw_ops->hw_ring_tx_xmit(aq_nic->aq_hw,
						       ring, frags);
		if (err >= 0) {
			++ring->stats.tx.packets;
			ring->stats.tx.bytes += skb->len;
		}
	} else {
		err = NETDEV_TX_BUSY;
	}
	spin_unlock_irqrestore(&aq_nic->aq_ptp->ptp_ring_lock, irq_flags);

err_exit:
	return err;
}

void aq_ptp_service_task(struct aq_nic_s *aq_nic)
{
	struct aq_ptp_s *self = aq_nic->aq_ptp;

	if (!self)
		return;

	aq_ptp_tx_timeout_check(self);
}

int aq_ptp_irq_alloc(struct aq_nic_s *aq_nic)
{
	struct pci_dev *pdev = aq_nic->pdev;
	struct aq_ptp_s *self = aq_nic->aq_ptp;
	int err = 0;

	if (!self)
		return 0;

	if (pdev->msix_enabled || pdev->msi_enabled) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0)
		err = request_irq(self->ptp_vector, aq_ptp_isr, 0,
				aq_nic->ndev->name, self);
		if( !err
#ifndef PTM_SUPPORT
				&& aq_ptp_clock.n_pins > 0 
#endif
				) {
			err = request_irq(self->gpio_vector, aq_ext_ptp_isr, 0,
					aq_nic->ndev->name, self);
		}
#else
		err = request_irq(pci_irq_vector(pdev, self->idx_ptp_vector),
				  aq_ptp_isr, 0, aq_nic->ndev->name, self);
		if( !err 
#ifndef PTM_SUPPORT
				&& aq_ptp_clock.n_pins > 0
#endif
		 ) {
			err = request_irq(pci_irq_vector(pdev, self->idx_gpio_vector),
				  aq_ext_ptp_isr, 0, aq_nic->ndev->name, self);
		}
#endif
	} else {
		err = -EINVAL;
		goto err_exit;
	}

err_exit:
	return err;
}

void aq_ptp_irq_free(struct aq_nic_s *aq_nic)
{
	struct aq_ptp_s *self = aq_nic->aq_ptp;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
	struct pci_dev *pdev = aq_nic->pdev;
#endif

	if (!self)
		return;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
	free_irq(pci_irq_vector(pdev, self->idx_ptp_vector), self);
	if( aq_ptp_clock.n_pins > 0 ) {
		free_irq(pci_irq_vector(pdev, self->idx_gpio_vector), self);
	}
#else
	free_irq(self->ptp_vector, self);
	if( aq_ptp_clock.n_pins > 0 ) {
		free_irq(self->gpio_vector, self);
	}
#endif
}

int aq_ptp_ring_init(struct aq_nic_s *aq_nic)
{
	struct aq_ptp_s *self = aq_nic->aq_ptp;
	int err = 0;

	if (!self)
		return 0;

	err = aq_ring_init(&self->ptp_tx);
	if (err < 0)
		goto err_exit;
	err = aq_nic->aq_hw_ops->hw_ring_tx_init(aq_nic->aq_hw,
						 &self->ptp_tx,
						 &self->ptp_ring_param);
	if (err < 0)
		goto err_exit;

	err = aq_ring_init(&self->ptp_rx);
	if (err < 0)
		goto err_exit;
	err = aq_nic->aq_hw_ops->hw_ring_rx_init(aq_nic->aq_hw,
						 &self->ptp_rx,
						 &self->ptp_ring_param);
	if (err < 0)
		goto err_exit;

	err = aq_ring_rx_fill(&self->ptp_rx);
	if (err < 0)
		goto err_exit;
	err = aq_nic->aq_hw_ops->hw_ring_rx_fill(aq_nic->aq_hw,
						 &self->ptp_rx,
						 0U);
	if (err < 0)
		goto err_exit;

	if( self->a1_ptp ) {
	err = aq_ring_init(&self->hwts_rx);
		if (err < 0)
			goto err_exit;
		err = aq_nic->aq_hw_ops->hw_ring_rx_init(aq_nic->aq_hw,
							 &self->hwts_rx,
							 &self->ptp_ring_param);
		if (err < 0)
			goto err_exit;
		err = aq_nic->aq_hw_ops->hw_ring_hwts_rx_fill(aq_nic->aq_hw,
							      &self->hwts_rx);
		if (err < 0)
			goto err_exit;
	}
err_exit:
	return err;
}

int aq_ptp_ring_start(struct aq_nic_s *aq_nic)
{
	struct aq_ptp_s *self = aq_nic->aq_ptp;
	int err = 0;

	if (!self)
		return 0;

	err = aq_nic->aq_hw_ops->hw_ring_tx_start(aq_nic->aq_hw, &self->ptp_tx);
	if (err < 0)
		goto err_exit;

	err = aq_nic->aq_hw_ops->hw_ring_rx_start(aq_nic->aq_hw, &self->ptp_rx);
	if (err < 0)
		goto err_exit;

	if( self->a1_ptp ) {
		err = aq_nic->aq_hw_ops->hw_ring_rx_start(aq_nic->aq_hw,
							  &self->hwts_rx);
		if (err < 0)
			goto err_exit;
	}

	napi_enable(&self->napi);

err_exit:
	return err;
}

void aq_ptp_ring_stop(struct aq_nic_s *aq_nic)
{
	struct aq_ptp_s *self = aq_nic->aq_ptp;

	if (!self)
		return;

	aq_nic->aq_hw_ops->hw_ring_tx_stop(aq_nic->aq_hw, &self->ptp_tx);
	aq_nic->aq_hw_ops->hw_ring_rx_stop(aq_nic->aq_hw, &self->ptp_rx);

	if( self->a1_ptp ) {
		aq_nic->aq_hw_ops->hw_ring_rx_stop(aq_nic->aq_hw,
						   &self->hwts_rx);
	}

	napi_disable(&self->napi);
}

void aq_ptp_ring_deinit(struct aq_nic_s *aq_nic)
{
	struct aq_ptp_s *self = aq_nic->aq_ptp;

	if (!self || !self->ptp_tx.aq_nic || !self->ptp_rx.aq_nic)
		return;

	aq_ring_tx_clean(&self->ptp_tx);
	aq_ring_rx_deinit(&self->ptp_rx);
}

int aq_ptp_ring_alloc(struct aq_nic_s *aq_nic)
{
	struct aq_ptp_s *self = aq_nic->aq_ptp;
	unsigned int tx_ring_idx, rx_ring_idx;
	struct aq_ring_s *hwts = 0;
	u32 tx_tc_mode, rx_tc_mode;
	struct aq_ring_s *ring;
	int err;

	if (!self)
		return 0;

	/* Index must to be 8 (8 TCs) or 16 (4 TCs).
	 * It depends from Traffic Class mode.
	 */
	aq_nic->aq_hw_ops->hw_tx_tc_mode_get(aq_nic->aq_hw, &tx_tc_mode);
	if (tx_tc_mode == 0)
		tx_ring_idx = AQ_CFG_PTP_8TC_RING_IDX;
	else
		tx_ring_idx = AQ_CFG_PTP_4TC_RING_IDX;

	ring = aq_ring_tx_alloc(&self->ptp_tx, aq_nic,
				tx_ring_idx, &aq_nic->aq_nic_cfg);
	if (!ring) {
		err = -ENOMEM;
		goto err_exit_1;
	}

	aq_nic->aq_hw_ops->hw_rx_tc_mode_get(aq_nic->aq_hw, &rx_tc_mode);
	if (rx_tc_mode == 0)
		rx_ring_idx = AQ_CFG_PTP_8TC_RING_IDX;
	else
		rx_ring_idx = AQ_CFG_PTP_4TC_RING_IDX;

	ring = aq_ring_rx_alloc(&self->ptp_rx, aq_nic,
				rx_ring_idx, &aq_nic->aq_nic_cfg);
	if (!ring) {
		err = -ENOMEM;
		goto err_exit_2;
	}

	if( self->a1_ptp ) {
		hwts = aq_ring_hwts_rx_alloc(&self->hwts_rx, aq_nic,
					     AQ_CFG_PTP_HWST_RING_IDX,
					     aq_nic->aq_nic_cfg.rxds,
					     aq_nic->aq_nic_cfg.aq_hw_caps->rxd_size);
		if (!hwts) {
			err = -ENOMEM;
			goto err_exit_3;
		}
	}

	err = aq_ptp_skb_ring_init(&self->skb_ring, aq_nic->aq_nic_cfg.rxds);
	if (err != 0) {
		err = -ENOMEM;
		goto err_exit_4;
	}

	self->ptp_ring_param.vec_idx = self->idx_ptp_vector;
	self->ptp_ring_param.cpu = self->ptp_ring_param.vec_idx +
			aq_nic_get_cfg(aq_nic)->aq_rss.base_cpu_number;
	cpumask_set_cpu(self->ptp_ring_param.cpu,
			&self->ptp_ring_param.affinity_mask);

	return 0;

err_exit_4:
	if( self->a1_ptp ) {
		aq_ring_free(&self->hwts_rx);
	}
err_exit_3:
	aq_ring_free(&self->ptp_rx);
err_exit_2:
	aq_ring_free(&self->ptp_tx);
err_exit_1:
	return err;
}

void aq_ptp_ring_free(struct aq_nic_s *aq_nic)
{
	struct aq_ptp_s *self = aq_nic->aq_ptp;

	if (!self)
		return;

	aq_ring_free(&self->ptp_tx);
	aq_ring_free(&self->ptp_rx);

	if( self->a1_ptp ) {
		aq_ring_free(&self->hwts_rx);
	}

	aq_ptp_skb_ring_release(&self->skb_ring);
}

static struct ptp_pin_desc aq_ptp_pd[MAX_PTP_GPIO_COUNT] = {
	{
		.name = "AQ_GPIO0",
		.index = 0,
		.func = PTP_PF_NONE,
		.chan = 0,
	},
	{
		.name = "AQ_GPIO1",
		.index = 1,
		.func = PTP_PF_NONE,
		.chan = 0,
	},
	{
		.name = "AQ_GPIO2",
		.index = 2,
		.func = PTP_PF_NONE,
		.chan = 0,
	},
	{
		.name = "AQ_GPIO3",
		.index = 3,
		.func = PTP_PF_NONE,
		.chan = 0,
	}
};

static struct ptp_clock_info aq_ptp_clock = {
	.owner		= THIS_MODULE,
	.name		= "atlantic ptp",
	.max_adj	= 999999999,
	.n_ext_ts	= 0,
	.pps		= 0,
	.adjfreq	= aq_ptp_adjfreq,
	.adjtime	= aq_ptp_adjtime,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
	.gettime64	= aq_ptp_gettime,
	.settime64	= aq_ptp_settime,
#else
	.gettime	= aq_ptp_gettime,
	.settime	= aq_ptp_settime,
#endif
	/* enable periodic outputs */
	.n_per_out     = 0,
	.enable        = aq_ptp_gpio_feature_enable,
	/* enable clock pins */
	.n_pins        = 0,
	.verify        = aq_ptp_verify,
	.pin_config    = aq_ptp_pd,
};

#define ptp_offset_init(__idx, __mbps, __egress, __ingress)   do { \
		ptp_offset[__idx].mbps = (__mbps); \
		ptp_offset[__idx].egress = (__egress); \
		ptp_offset[__idx].ingress = (__ingress); } \
		while (0)

static void aq_ptp_offset_init_from_fw(const struct hw_aq_ptp_offset *offsets)
{
	int i;

	/* Load offsets for PTP */
	for (i = 0; i < ARRAY_SIZE(ptp_offset); i++) {
		switch (i) {
		/* 100M */
		case ptp_offset_idx_100:
			ptp_offset_init(i, 100,
					offsets->egress_100,
					offsets->ingress_100);
			break;
		/* 1G */
		case ptp_offset_idx_1000:
			ptp_offset_init(i, 1000,
					offsets->egress_1000,
					offsets->ingress_1000);
			break;
		/* 2.5G */
		case ptp_offset_idx_2500:
			ptp_offset_init(i, 2500,
					offsets->egress_2500,
					offsets->ingress_2500);
			break;
		/* 5G */
		case ptp_offset_idx_5000:
			ptp_offset_init(i, 5000,
					offsets->egress_5000,
					offsets->ingress_5000);
			break;
		/* 10G */
		case ptp_offset_idx_10000:
			ptp_offset_init(i, 10000,
					offsets->egress_10000,
					offsets->ingress_10000);
			break;
		}
	}
}

static void aq_ptp_offset_init_from_params(int force)
{
	if (force || aq_ptp_offset_100)
		ptp_offset_init(ptp_offset_idx_100, 100,
				(aq_ptp_offset_100 >> 16) & 0xffff,
				aq_ptp_offset_100 & 0xffff);
	if (force || aq_ptp_offset_1000)
		ptp_offset_init(ptp_offset_idx_1000, 1000,
				(aq_ptp_offset_1000 >> 16) & 0xffff,
				aq_ptp_offset_1000 & 0xffff);
	if (force || aq_ptp_offset_2500)
		ptp_offset_init(ptp_offset_idx_2500, 2500,
				(aq_ptp_offset_2500 >> 16) & 0xffff,
				aq_ptp_offset_2500 & 0xffff);
	if (force || aq_ptp_offset_5000)
		ptp_offset_init(ptp_offset_idx_5000, 5000,
				(aq_ptp_offset_5000 >> 16) & 0xffff,
				aq_ptp_offset_5000 & 0xffff);
	if (force || aq_ptp_offset_10000)
		ptp_offset_init(ptp_offset_idx_10000, 10000,
				(aq_ptp_offset_10000 >> 16) & 0xffff,
				aq_ptp_offset_10000 & 0xffff);
}

static void aq_ptp_offset_init(const struct hw_aq_ptp_offset *offsets)
{
	memset(ptp_offset, 0, sizeof(ptp_offset));

	if (aq_ptp_offset_forced) {
		aq_ptp_offset_init_from_params(1);
	} else {
		aq_ptp_offset_init_from_fw(offsets);
		aq_ptp_offset_init_from_params(0);
	}
}

static void aq_ptp_gpio_init(struct aq_ptp_s *self, 
	enum gpio_pin_function gpio_pin[MAX_PTP_GPIO_COUNT], 
	int sync1588_input)
{
	u32 ncount = 0;
	u32 i;

	for (i = 0; i < MAX_PTP_GPIO_COUNT; i++) {
		if (gpio_pin[i] == (GPIO_PIN_FUNCTION_PTP0 + ncount)) {
			char *name = aq_ptp_clock.pin_config[ncount].name;
			u32 offset = 7;
			name[offset] = '0' + i;
			strncat(name, self->a2_ptp ? "_INOUT" : "_OUT", 64 - offset);
			
			aq_ptp_clock.pin_config[ncount].index = ncount;
			aq_ptp_clock.pin_config[ncount].chan = 0;
			aq_ptp_clock.pin_config[ncount].rsv[3] = i;
			aq_ptp_clock.pin_config[ncount++].func = PTP_PF_NONE;
		}
	}
	
	aq_ptp_clock.n_per_out = ncount;

	if (self->aq_nic->aq_hw_ops->hw_extts_gpio_enable && 
		self->aq_nic->aq_hw_ops->hw_ptp_gpio_get_event) {

		//AQC107 has single input SYNC1588
		if( sync1588_input && ncount < MAX_PTP_GPIO_COUNT ) {
				char *name = aq_ptp_clock.pin_config[ncount].name;
				u32 offset = 3;
				name[offset] = '\0';
				strncat(name, "SYNC1588_IN", 64 - offset);
				aq_ptp_clock.pin_config[ncount].index = ncount;
				aq_ptp_clock.pin_config[ncount].func = PTP_PF_NONE;
				aq_ptp_clock.pin_config[ncount].rsv[3] = i;
				aq_ptp_clock.pin_config[ncount++].chan = 0;
		}

		//AQC113 has two indepent counters doesn't make sence to use more than 2 channels
		aq_ptp_clock.n_ext_ts = self->a2_ptp ? (ncount >= 2 ? 2 : ncount) : !!sync1588_input; 
	}
	aq_ptp_clock.n_pins = ncount;
}

void aq_ptp_clock_init(struct aq_nic_s *aq_nic, enum aq_ptp_state state)
{
	struct aq_ptp_s *self = aq_nic->aq_ptp;

	if( !self )
		return;

	if( self->a1_ptp || state == AQ_PTP_FIRST_INIT ) {
		struct timespec64 ts;

		ktime_get_real_ts64(&ts);
		aq_ptp_settime(&self->ptp_info, &ts);
	}

	if( self->a1_ptp ) {
		if (state == AQ_PTP_LINK_UP) {
			u32 n_pin = aq_ptp_clock.n_pins - 1;
			if( n_pin < MAX_PTP_GPIO_COUNT &&
				aq_ptp_clock.pin_config[n_pin].func == PTP_PF_EXTTS ) {
				aq_ptp_pid_reset(&self->pid);
				aq_ptp_extts_pin_configure(&self->ptp_info, n_pin, aq_ptp_clock.pin_config[n_pin].rsv[2]);
			}
		} else {
			cancel_delayed_work_sync(&self->poll_sync);
			aq_ptp_pid_reset(&self->pid);
		}
	}
	
	if( !self->a1_ptp && state != AQ_PTP_FIRST_INIT ) {
		unsigned int ptp_en_flags = 
		parse_rx_filters(state == AQ_PTP_LINK_UP ? self->hwtstamp_config.rx_filter : AQ_HW_PTP_DISABLE);
		aq_ptp_dpath_enable(self, ptp_en_flags, self->ptp_rx.idx);
	}
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0)
int aq_ptp_init(struct aq_nic_s *aq_nic, unsigned int idx_vec, 
        unsigned int ptp_vec, unsigned int gpio_vec)
#else
int aq_ptp_init(struct aq_nic_s *aq_nic, unsigned int idx_vec)
#endif
{
	struct hw_atl_utils_mbox mbox;
	struct ptp_clock *clock;
	struct aq_ptp_s *self;
	int err = 0;
	bool a1_ptp = !!(aq_nic->aq_fw_ops->enable_ptp);
	bool a2_ptp = !!(aq_nic->aq_hw_ops->enable_ptp);

	if (!a1_ptp && !a2_ptp) {
		aq_nic->aq_ptp = NULL;
		return 0;
	}

	if( a1_ptp )
	{
		hw_atl_utils_mpi_read_stats(aq_nic->aq_hw, &mbox);

		if (!(mbox.info.caps_ex & BIT(CAPS_EX_PHY_PTP_EN))) {
			aq_nic->aq_ptp = NULL;
			return 0;
		}
	} else {
		//TODO A2 GPIO config read
		mbox.info.gpio_pin[0] = GPIO_PIN_FUNCTION_PTP0;
		mbox.info.gpio_pin[1] = GPIO_PIN_FUNCTION_NC;
		mbox.info.gpio_pin[2] = GPIO_PIN_FUNCTION_NC;
		//mbox.info.gpio_pin[3] = GPIO_PIN_FUNCTION_NC;

		memset(&mbox.info.ptp_offset, 0, sizeof(mbox.info.ptp_offset));
		mbox.info.ptp_offset.ingress_100 = 0x8c0;
		mbox.info.ptp_offset.egress_100 = 0x8c0;
		mbox.info.ptp_offset.ingress_1000 = 0x380;
		mbox.info.ptp_offset.egress_1000 = 0x380;
	}

	aq_ptp_offset_init(&mbox.info.ptp_offset);

	self = kzalloc(sizeof(*self), GFP_KERNEL);
	if (!self) {
		err = -ENOMEM;
		goto err_exit;
	}

	self->aq_nic = aq_nic;
	self->a1_ptp = a1_ptp;
	self->a2_ptp = a2_ptp;

	aq_ptp_gpio_init(self, mbox.info.gpio_pin, a1_ptp ? mbox.info.caps_ex & BIT(CAPS_EX_PHY_CTRL_TS_PIN) : 0);
	spin_lock_init(&self->ptp_lock);
	spin_lock_init(&self->ptp_ring_lock);


#ifdef PTM_SUPPORT
	self->ptm_en = false;
	spin_lock_init(&self->ptmq.lock);
	init_waitqueue_head(&self->ptm_wq);
#endif

	self->ptp_info = aq_ptp_clock;
	clock = ptp_clock_register(&self->ptp_info, &aq_nic->ndev->dev);
	if (!clock) {
		netdev_err(aq_nic->ndev, "ptp_clock_register failed\n");
		err = 0;
		goto err_exit;
	}
	self->ptp_clock = clock;
	aq_ptp_tx_timeout_init(&self->ptp_tx_timeout);

	atomic_set(&self->offset_egress, 0);
	atomic_set(&self->offset_ingress, 0);

	netif_napi_add(aq_nic_get_ndev(aq_nic), &self->napi,
		       aq_ptp_poll, AQ_CFG_NAPI_WEIGHT);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0)
	self->ptp_vector = ptp_vec;
	self->gpio_vector = gpio_vec;
#endif
	self->idx_ptp_vector = idx_vec;
	self->idx_gpio_vector = idx_vec + 1;

	aq_nic->aq_ptp = self;

	/* enable ptp counter */
	self->ptp_clock_sel = ATL_TSG_CLOCK_SEL_1; //mbox.info.caps_ex.;
	aq_utils_obj_set(&aq_nic->aq_hw->flags, AQ_HW_PTP_AVAILABLE);
	if(a1_ptp) {
		mutex_lock(&aq_nic->fwreq_mutex);
		aq_nic->aq_fw_ops->enable_ptp(aq_nic->aq_hw, 1);
		mutex_unlock(&aq_nic->fwreq_mutex);
	}
	if(a2_ptp)
		aq_nic->aq_hw_ops->enable_ptp(aq_nic->aq_hw, self->ptp_clock_sel, 1);

	INIT_DELAYED_WORK(&self->poll_sync, &aq_ptp_poll_sync_work_cb);

	aq_ptp_clock_init(aq_nic, AQ_PTP_FIRST_INIT);
	netdev_info(aq_nic->ndev, "Enable PTP Support. %d GPIO(s)\n", aq_ptp_clock.n_pins);

	return 0;

err_exit:
	kfree(self);
	aq_nic->aq_ptp = NULL;
	return err;
}

/* aq_ptp_stop - close the PTP device
 * @adapter: pointer to adapter struct
 *
 * completely destroy the PTP device, should only be called when the device is
 * being fully closed.
 */
void aq_ptp_unregister(struct aq_nic_s *aq_nic)
{
	struct aq_ptp_s *self = aq_nic->aq_ptp;

	if (!self)
		return;

	ptp_clock_unregister(self->ptp_clock);
#ifdef PTM_SUPPORT
	self->ptm_en = false;
#endif
}

void aq_ptp_free(struct aq_nic_s *aq_nic)
{
	struct aq_ptp_s *self = aq_nic->aq_ptp;

	if (!self)
		return;

	/* disable ptp */
	if(self->a1_ptp){
		cancel_delayed_work_sync(&self->poll_sync);
		mutex_lock(&aq_nic->fwreq_mutex);
		aq_nic->aq_fw_ops->enable_ptp(aq_nic->aq_hw, 0);
		mutex_unlock(&aq_nic->fwreq_mutex);
	}
	if(self->a2_ptp)
		aq_nic->aq_hw_ops->enable_ptp(aq_nic->aq_hw, self->ptp_clock_sel, 0);

	netif_napi_del(&self->napi);
	kfree(self);
	aq_utils_obj_clear(&aq_nic->aq_hw->flags, AQ_HW_PTP_AVAILABLE);
	aq_nic->aq_ptp = NULL;
}

struct ptp_clock *aq_ptp_get_ptp_clock(struct aq_ptp_s *self)
{
	return self->ptp_clock;
}
EXPORT_SYMBOL_GPL(aq_ptp_configure_ext_gpio);
#endif
