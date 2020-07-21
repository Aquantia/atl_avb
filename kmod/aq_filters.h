/* SPDX-License-Identifier: GPL-2.0-only */
/* Atlantic Network Driver
 *
 * Copyright (C) 2014-2019 aQuantia Corporation
 * Copyright (C) 2019-2020 Marvell International Ltd.
 */

/* File aq_filters.h: RX filters related functions. */

#ifndef AQ_FILTERS_H
#define AQ_FILTERS_H

#include "aq_nic.h"
#include "aq_compat.h"

enum aq_rx_filter_type {
	aq_rx_filter_type_vlan = 0x0010,
	aq_rx_filter_type_vlan_pcp = 0x0020,
	aq_rx_filter_type_ethertype = 0x0040,
	aq_rx_filter_type_l3l4 = 0x0080,
	aq_rx_filter_type_ipv4 = 0x0100,
	aq_rx_filter_type_ipv6 = 0x0200,
	aq_rx_filter_type_l3 = 0x0300,
	aq_rx_filter_type_flex = 0x1000,
	aq_rx_filter_type_l4flex = 0x2000,
	aq_rx_filter_type_chain = 0x10000,
};

#define CHAINING_QUEUE 	0x7F

struct aq_rx_filter {
	struct hlist_node aq_node;
	u32 type;
	struct ethtool_rx_flow_spec aq_fsp;
};

u16 aq_get_rxnfc_count_all_rules(struct aq_nic_s *aq_nic);
struct aq_hw_rx_fltrs_s *aq_get_hw_rx_fltrs(struct aq_nic_s *aq_nic);
int aq_add_rxnfc_rule(struct aq_nic_s *aq_nic, const struct ethtool_rxnfc *cmd);
int aq_del_rxnfc_rule(struct aq_nic_s *aq_nic, const struct ethtool_rxnfc *cmd);
int aq_get_rxnfc_rule(struct aq_nic_s *aq_nic, struct ethtool_rxnfc *cmd);
int aq_get_rxnfc_all_rules(struct aq_nic_s *aq_nic, struct ethtool_rxnfc *cmd,
			   u32 *rule_locs);
int aq_del_fvlan_by_vlan(struct aq_nic_s *aq_nic, u16 vlan_id);
int aq_clear_rxnfc_all_rules(struct aq_nic_s *aq_nic);
int aq_filters_vlans_update(struct aq_nic_s *aq_nic);
int aq_filters_vlans_off(struct aq_nic_s *aq_nic);
int aq_filters_vlans_on(struct aq_nic_s *aq_nic);
int aq_apply_all_rule(struct aq_nic_s *aq_nic);

#endif /* AQ_FILTERS_H */
