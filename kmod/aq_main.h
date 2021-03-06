/* SPDX-License-Identifier: GPL-2.0-only */
/* Atlantic Network Driver
 *
 * Copyright (C) 2014-2019 aQuantia Corporation
 * Copyright (C) 2019-2020 Marvell International Ltd.
 */

/* File aq_main.h: Main file for aQuantia Linux driver. */

#ifndef AQ_MAIN_H
#define AQ_MAIN_H

#include "aq_common.h"
#include "aq_nic.h"

extern const char aq_ndev_driver_name[];

void aq_ndev_schedule_work(struct work_struct *work);
struct net_device *aq_ndev_alloc(void);

#endif /* AQ_MAIN_H */
