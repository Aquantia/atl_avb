// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (C) 2014-2019 aQuantia Corporation. */

/* File aq_filters.c: RX filters related functions. */

#include "aq_filters.h"

static bool __must_check
aq_rule_is_approve(struct ethtool_rx_flow_spec *fsp)
{
	if (fsp->flow_type & FLOW_MAC_EXT)
		return false;

	switch (fsp->flow_type & ~FLOW_EXT) {
	case ETHER_FLOW:
	case TCP_V4_FLOW:
	case UDP_V4_FLOW:
	case SCTP_V4_FLOW:
	case TCP_V6_FLOW:
	case UDP_V6_FLOW:
	case SCTP_V6_FLOW:
	case IPV4_FLOW:
	case IPV6_FLOW:
		return true;
	case IP_USER_FLOW:
		switch (fsp->h_u.usr_ip4_spec.proto) {
		case IPPROTO_TCP:
		case IPPROTO_UDP:
		case IPPROTO_SCTP:
		case IPPROTO_IP:
			return true;
		default:
			return false;
			}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0)
	case IPV6_USER_FLOW:
		switch (fsp->h_u.usr_ip6_spec.l4_proto) {
		case IPPROTO_TCP:
		case IPPROTO_UDP:
		case IPPROTO_SCTP:
		case IPPROTO_IP:
			return true;
		default:
			return false;
			}
#endif
	default:
		return false;
	}

	return false;
}

static bool __must_check
aq_match_filter(struct ethtool_rx_flow_spec *fsp1,
		struct ethtool_rx_flow_spec *fsp2)
{
	if (fsp1->flow_type != fsp2->flow_type ||
	    memcmp(&fsp1->h_u, &fsp2->h_u, sizeof(fsp2->h_u)) ||
	    memcmp(&fsp1->h_ext, &fsp2->h_ext, sizeof(fsp2->h_ext)) ||
	    memcmp(&fsp1->m_u, &fsp2->m_u, sizeof(fsp2->m_u)) ||
	    memcmp(&fsp1->m_ext, &fsp2->m_ext, sizeof(fsp2->m_ext)))
		return false;

	return true;
}

static u32 get_empty_location(struct aq_nic_s *aq_nic, u32 aq_first, u32 aq_last, u32 fsp_loc, int filter_type)
{
	struct aq_hw_rx_fltrs_s *rx_fltrs = aq_get_hw_rx_fltrs(aq_nic);
	struct aq_rx_filter *rule = NULL;
	struct hlist_node *aq_node2;
	u32 res, chain = 0, rule_loc = 0xff;
	bool rescan = false;
	//printk("Find location: first %x, last %x, request %x, filter_type %x\n",
	//		aq_first, aq_last, fsp_loc, filter_type);
	hlist_for_each_entry_safe(rule, aq_node2,
				  &rx_fltrs->filter_list, aq_node) {
		rule_loc = rule->aq_fsp.location & AQ_RX_FILTER_CHAIN_MASK;
		if( !(filter_type & rule->type) || 
			rule->aq_fsp.location & AQ_RX_FILTER_CHAIN_CURRENT_MASK ){ 
			continue;
		}
		if (rule_loc >= AQ_RX_FIRST_LOC_CHAIN &&
			rule_loc < AQ_RX_LAST_LOC_CHAIN) {
			chain++;
		}
		else if ( rule_loc == aq_first ) {
			aq_first++;
		}
		else if ( rule_loc == aq_last - 1 ) {
			aq_last++;
		}	
		else if ( rule_loc > aq_first &&
				  rule_loc < aq_last - 1 ) {
			rescan = true;
		}
	}

	res = fsp_loc < RX_CLS_LOC_LAST ? fsp_loc :
		  fsp_loc == RX_CLS_LOC_LAST ? aq_last - 1 : aq_first + chain;

	while( rescan )
	{
		bool bondaries_changed = false;
		rescan = false;

		hlist_for_each_entry_safe(rule, aq_node2,
				  &rx_fltrs->filter_list, aq_node) {
			rule_loc = rule->aq_fsp.location & AQ_RX_FILTER_CHAIN_MASK;
			if ( rule_loc == aq_first ) {
				aq_first++;
				bondaries_changed = true;
			}
			else if ( rule_loc == aq_last - 1 ) {
				bondaries_changed = true;
				aq_last++;
			}
			else if ( rule_loc > aq_first &&
					  rule_loc < aq_last - 1 ) {
				rescan = true;
			}
		}

		if( bondaries_changed ) {
			res = fsp_loc < RX_CLS_LOC_LAST ? fsp_loc :
				fsp_loc == RX_CLS_LOC_LAST ? aq_last - 1 : aq_first + chain;
			continue;
		}

		rescan = false;
		hlist_for_each_entry_safe(rule, aq_node2,
				  &rx_fltrs->filter_list, aq_node) {
			rule_loc = rule->aq_fsp.location & AQ_RX_FILTER_CHAIN_MASK;
			if ( rule_loc > aq_first &&
				 rule_loc < aq_last - 1 ) {
				if ( rule_loc <= aq_first + chain ) {
					bondaries_changed = true;
					chain -= rule_loc - aq_first;
					aq_first = rule_loc;
				}
				else if( rule_loc == res ) {
					rescan = true;
				}
			}
		}
		if( bondaries_changed ) {
			res = fsp_loc < RX_CLS_LOC_LAST ? fsp_loc :
				fsp_loc == RX_CLS_LOC_LAST ? aq_last - 1 : aq_first + chain;
			continue;
		}
		if( rescan ) {
			res = fsp_loc < RX_CLS_LOC_LAST ? fsp_loc : 
					res + (fsp_loc == RX_CLS_LOC_LAST ? -1 : 1);
		}
	}
	//printk("Res %x, first %x, chain %x, last %x\n",
	//		res, aq_first, chain, aq_last);
	return res >= aq_first + chain && res <= aq_last ? res : RX_CLS_LOC_ANY;
}

static bool __must_check
aq_rule_already_exists(struct aq_nic_s *aq_nic,
		       struct ethtool_rx_flow_spec *fsp)
{
	struct aq_rx_filter *rule;
	struct hlist_node *aq_node2;
	struct aq_hw_rx_fltrs_s *rx_fltrs = aq_get_hw_rx_fltrs(aq_nic);

	hlist_for_each_entry_safe(rule, aq_node2,
				  &rx_fltrs->filter_list, aq_node) {
		if (rule->aq_fsp.location != fsp->location)
			continue;
		if (aq_match_filter(&rule->aq_fsp, fsp)) {
			netdev_err(aq_nic->ndev,
				     "ethtool: This filter is already set\n");
			return true;
		}
	}

	return false;
}

static int aq_check_approve_ipv4(struct aq_nic_s *aq_nic,
				  struct aq_hw_rx_fltrs_s *rx_fltrs,
				  struct ethtool_rx_flow_spec *fsp)
{
	u32 fsp_loc = fsp->location;
	if( fsp_loc > AQ_RX_MAX_RXNFC_LOC && fsp_loc < RX_CLS_LOC_LAST ) {
		fsp_loc = (fsp_loc & AQ_RX_FILTER_CHAIN_L3_LOC_MASK) >> AQ_RX_FILTER_CHAIN_L3_LOC_SHIFT;
	}
	if( !(fsp_loc >= AQ_RX_FIRST_LOC_FIPV4 &&
		  fsp_loc < AQ_RX_LAST_LOC_FIPV4) && 
			fsp_loc < RX_CLS_LOC_LAST ) {
		netdev_err(aq_nic->ndev,
				"ethtool: The specified location for ipv4 is wrong");
		return -EINVAL;
		}
	else if( fsp_loc >= RX_CLS_LOC_LAST &&
				get_empty_location(aq_nic, 
							AQ_RX_FIRST_LOC_FIPV4,
							AQ_RX_LAST_LOC_FIPV4,
							fsp_loc, aq_rx_filter_type_l3) == RX_CLS_LOC_ANY ) {
		netdev_err(aq_nic->ndev,
				"ethtool: No more free ipv4 filters");
		return -EINVAL;
	}
	return 0;
}

static int aq_check_approve_ipv6(struct aq_nic_s *aq_nic,
				  struct aq_hw_rx_fltrs_s *rx_fltrs,
				  struct ethtool_rx_flow_spec *fsp)
{
	u32 fsp_loc = fsp->location;
	if( fsp_loc > AQ_RX_MAX_RXNFC_LOC && fsp_loc < RX_CLS_LOC_LAST ) {
		fsp_loc = (fsp_loc & AQ_RX_FILTER_CHAIN_L3_LOC_MASK) >> AQ_RX_FILTER_CHAIN_L3_LOC_SHIFT;
	}
	if( !(fsp_loc >= AQ_RX_FIRST_LOC_FIPV6 &&
		  fsp_loc < AQ_RX_LAST_LOC_FIPV6) && 
			fsp_loc < RX_CLS_LOC_LAST ) {
		netdev_err(aq_nic->ndev,
				"ethtool: The specified location for ipv6 is wrong");
		return -EINVAL;
		}
	else if( fsp_loc >= RX_CLS_LOC_LAST &&
				get_empty_location(aq_nic, 
							AQ_RX_FIRST_LOC_FIPV6,
							AQ_RX_LAST_LOC_FIPV6,
							fsp_loc, aq_rx_filter_type_l3) == RX_CLS_LOC_ANY ) {
		netdev_err(aq_nic->ndev,
				"ethtool: No more free ipv6 filters");
		return -EINVAL;
	}
	return 0;
}

static int aq_check_approve_l4(struct aq_nic_s *aq_nic,
				  struct aq_hw_rx_fltrs_s *rx_fltrs,
				  struct ethtool_rx_flow_spec *fsp)
{
	u32 fsp_loc = fsp->location;
	if( fsp_loc > AQ_RX_MAX_RXNFC_LOC && fsp_loc < RX_CLS_LOC_LAST ) {
		fsp_loc = (fsp_loc & AQ_RX_FILTER_CHAIN_L3L4_LOC_MASK) >> AQ_RX_FILTER_CHAIN_L3L4_LOC_SHIFT;
	}
	if( !(fsp_loc >= AQ_RX_FIRST_LOC_FL3L4 &&
		  fsp_loc < AQ_RX_LAST_LOC_FL3L4) && 
			fsp_loc < RX_CLS_LOC_LAST ) {
		netdev_err(aq_nic->ndev,
				"ethtool: The specified location for ipv6 is wrong");
		return -EINVAL;
		}
	else if( fsp_loc >= RX_CLS_LOC_LAST &&
				get_empty_location(aq_nic, 
							AQ_RX_FIRST_LOC_FL3L4,
							AQ_RX_LAST_LOC_FL3L4,
							fsp_loc, aq_rx_filter_type_l3l4) == RX_CLS_LOC_ANY ) {
		netdev_err(aq_nic->ndev,
				"ethtool: No more free ipv6 filters");
		return -EINVAL;
	}
	return 0;
}

static int aq_check_approve_fl3l4(struct aq_nic_s *aq_nic,
				  struct aq_hw_rx_fltrs_s *rx_fltrs,
				  struct ethtool_rx_flow_spec *fsp,
				  bool is_ipv6)
{
	u32 fsp_loc = fsp->location;
	if( fsp_loc > AQ_RX_MAX_RXNFC_LOC && fsp_loc < RX_CLS_LOC_LAST ) {
		fsp_loc = (fsp_loc & AQ_RX_FILTER_CHAIN_L3L4_LOC_MASK) >> AQ_RX_FILTER_CHAIN_L3L4_LOC_SHIFT;
	}
	/*
	if (is_ipv6 && rx_fltrs->fl3l4.active_ipv4) {
		netdev_err(aq_nic->ndev,
			     "ethtool: mixing ipv4 and ipv6 is not allowd");
		return -EINVAL;
	} else if (!is_ipv6 && rx_fltrs->fl3l4.active_ipv6) {
		netdev_err(aq_nic->ndev,
			     "ethtool: mixing ipv4 and ipv6 is not allowd");
		return -EINVAL;
	} else  */
	if (is_ipv6) {
		u32 first_loc = AQ_RX_FIRST_LOC_FL3L4 + (rx_fltrs->fl3l4.active_ipv4 & 0x0f ? 4 : 0);
		u32 last_loc = AQ_RX_LAST_LOC_FL3L4 - 3 - (rx_fltrs->fl3l4.active_ipv4 & 0xf0 ? 4 : 0);

		if( first_loc > last_loc ) {
			netdev_err(aq_nic->ndev,
			     "ethtool: All filters are busy by IPv4. No place for IPv6");
			return -EINVAL;
		}

		//A1 or A2 legacy
		if(  fsp_loc != first_loc &&
		     fsp_loc != last_loc - 1 && 
		     fsp_loc < RX_CLS_LOC_LAST ) {
			netdev_err(aq_nic->ndev,
			     "ethtool: The available location for ipv6 is %d or %d",
			     first_loc, last_loc - 1);
			return -EINVAL;
			}
		else if( fsp_loc >= RX_CLS_LOC_LAST &&
				 get_empty_location(aq_nic, 
		  					 first_loc,
							 first_loc + 1,
							 fsp_loc, aq_rx_filter_type_l3l4) == RX_CLS_LOC_ANY &&
				 get_empty_location(aq_nic, 
		  					 last_loc - 1,
							 last_loc,
							 fsp_loc, aq_rx_filter_type_l3l4) == RX_CLS_LOC_ANY ) {
			netdev_err(aq_nic->ndev,
			     "ethtool: No more free ipv6 filters");
			return -EINVAL;
		}
	} else {
		u32 first_loc = AQ_RX_FIRST_LOC_FL3L4 + (rx_fltrs->fl3l4.active_ipv6 & 0x1 ? 4 : 0);
		u32 last_loc = AQ_RX_LAST_LOC_FL3L4 - (rx_fltrs->fl3l4.active_ipv6 & 0x2 ? 4 : 0);
		if( first_loc > last_loc ) {
			netdev_err(aq_nic->ndev,
			     "ethtool: All filters are busy by IPv6. No place for IPv4");
			return -EINVAL;
		}
		if( (fsp_loc < first_loc ||
		     fsp_loc > last_loc ) && 
		     fsp_loc < RX_CLS_LOC_LAST ) {
			netdev_err(aq_nic->ndev,
			     "ethtool: no location for IPv4 filter");
			return -EINVAL;
			}
		else if( fsp_loc >= RX_CLS_LOC_LAST &&
				 get_empty_location(aq_nic, 
		  					 first_loc,
							 last_loc,
							 fsp_loc, aq_rx_filter_type_l3l4) == RX_CLS_LOC_ANY ) {
			netdev_err(aq_nic->ndev,
					"ethtool: no location for IPv4 filter");
			return -EINVAL;
		}
	}
	return 0;
}

static int __must_check
aq_check_approve_fl2(struct aq_nic_s *aq_nic,
		     struct aq_hw_rx_fltrs_s *rx_fltrs,
		     struct ethtool_rx_flow_spec *fsp)
{
	u32 fsp_loc = fsp->location;
	if( fsp_loc > AQ_RX_MAX_RXNFC_LOC && fsp_loc < RX_CLS_LOC_LAST ) {
		fsp_loc = ((fsp_loc & AQ_RX_FILTER_CHAIN_L2ET_LOC_MASK) >> AQ_RX_FILTER_CHAIN_L2ET_LOC_SHIFT) - 1;
	}
	if( (fsp_loc < AQ_RX_FIRST_LOC_FETHERT
	   || fsp_loc > AQ_RX_LAST_LOC_FETHERT) 
	   && fsp_loc < RX_CLS_LOC_LAST) {
		netdev_err(aq_nic->ndev,
			     "ethtool: location must be in range [%d, %d]",
			     AQ_RX_FIRST_LOC_FETHERT,
			     AQ_RX_LAST_LOC_FETHERT);
		return -EINVAL;
	}
	else if (fsp_loc >= RX_CLS_LOC_LAST &&
		  get_empty_location(aq_nic, 
		  					 AQ_RX_FIRST_LOC_FETHERT,
							 AQ_RX_LAST_LOC_FETHERT,
							 fsp_loc,
							 aq_rx_filter_type_ethertype) == RX_CLS_LOC_ANY) {
		netdev_err(aq_nic->ndev,
			     "ethtool: no location for ethertype filter");
		return -EINVAL;
	}
	return 0;
}

static int __must_check
aq_check_approve_dst_fl2(struct aq_nic_s *aq_nic,
		     struct aq_hw_rx_fltrs_s *rx_fltrs,
		     struct ethtool_rx_flow_spec *fsp)
{
	u32 i;
	u32 fsp_loc = fsp->location;
	if( fsp_loc > AQ_RX_MAX_RXNFC_LOC && fsp_loc < RX_CLS_LOC_LAST ) {
		fsp_loc = ((fsp_loc & AQ_RX_FILTER_CHAIN_L2_DST_MASK) >> AQ_RX_FILTER_CHAIN_L2_DST_SHIFT) - 1;
	}
	if( (fsp_loc < AQ_RX_FIRST_LOC_L2DST
	   || fsp_loc > AQ_RX_LAST_LOC_L2DST) 
	   && fsp_loc < RX_CLS_LOC_LAST) {
		netdev_err(aq_nic->ndev,
			     "ethtool: L2 DST location must be in range [%d, %d]",
			     AQ_RX_FIRST_LOC_L2DST,
			     AQ_RX_LAST_LOC_L2DST);
		return -EINVAL;
	}
	else if (fsp_loc >= RX_CLS_LOC_LAST &&
		  get_empty_location(aq_nic, 
		  					 AQ_RX_FIRST_LOC_L2DST,
							 AQ_RX_LAST_LOC_L2DST,
							 fsp_loc,
							 aq_rx_filter_type_ethertype) == RX_CLS_LOC_ANY) {
		netdev_err(aq_nic->ndev,
			     "ethtool: no location for L2 DST filter");
		return -EINVAL;
	}

	for (i = 0U; i < aq_nic->mc_list.count; i++) {
		if( ((uint32_t *)fsp->m_u.ether_spec.h_dest)[0] == ((uint32_t *)aq_nic->mc_list.ar)[0] &&
			((uint16_t *)fsp->m_u.ether_spec.h_dest)[2] == ((uint16_t *)aq_nic->mc_list.ar)[2] )
		{
			break;
		}			
	}
	
	if( i > aq_nic->mc_list.count ) {
		netdev_err(aq_nic->ndev,
			     "ethtool: driver instance doesn't link with dest L2 address.");
		return -EINVAL;
	}
	return 0;
}

static int __must_check
aq_check_approve_fvlan_pcp(struct aq_nic_s *aq_nic,
		       struct aq_hw_rx_fltrs_s *rx_fltrs,
		       struct ethtool_rx_flow_spec *fsp)
{
	/*
	if (((be16_to_cpu(fsp->m_ext.vlan_tci) & VLAN_PRIO_MASK) == VLAN_PRIO_MASK) &&
	     (fsp->m_u.ether_spec.h_proto == 0U)) {
		netdev_err(aq_nic->ndev,
			     "ethtool: proto (ether_type) parameter must be specfied"
			     );
		return -EINVAL;
	}
	*/
	return 0;
}

static int __must_check
aq_check_approve_fvlan(struct aq_nic_s *aq_nic,
		       struct aq_hw_rx_fltrs_s *rx_fltrs,
		       struct ethtool_rx_flow_spec *fsp)
{
	u32 fsp_loc = fsp->location;
	if( fsp_loc > AQ_RX_MAX_RXNFC_LOC && fsp_loc < RX_CLS_LOC_LAST ) {
		fsp_loc = (fsp_loc & AQ_RX_FILTER_CHAIN_VLAN_LOC_MASK) >> AQ_RX_FILTER_CHAIN_VLAN_LOC_SHIFT;
	}
	if( (fsp_loc < AQ_RX_FIRST_LOC_FVLANID
	   || fsp_loc > AQ_RX_LAST_LOC_FVLANID)
	   && fsp_loc < RX_CLS_LOC_LAST ) {
		netdev_err(aq_nic->ndev,
			     "ethtool: location must be in range [%d, %d]",
			     AQ_RX_FIRST_LOC_FVLANID,
			     AQ_RX_LAST_LOC_FVLANID);
		return -EINVAL;
	}
	else if( fsp_loc >= RX_CLS_LOC_LAST &&
		  get_empty_location(aq_nic, 
		  					 AQ_RX_FIRST_LOC_FVLANID,
							 AQ_RX_LAST_LOC_FVLANID,
							 fsp_loc,
							 aq_rx_filter_type_vlan) == RX_CLS_LOC_ANY ) {
		netdev_err(aq_nic->ndev,
			     "ethtool: no location for VLAN filter");
		return -EINVAL;
	}
	if (!test_bit(be16_to_cpu(fsp->h_ext.vlan_tci) & VLAN_VID_MASK, aq_nic->active_vlans)) {
		netdev_err(aq_nic->ndev,
			     "ethtool: unknown vlan-id specified.");
		return -EINVAL;
	}
	return 0;
}

static int __must_check
aq_check_filter(struct aq_nic_s *aq_nic,
		struct ethtool_rx_flow_spec *fsp)
{
	bool a2_new_filter = aq_nic->aq_hw_ops->hw_filter_chain_build && aq_new_filters_enabled;
	bool is_ipv6 = false;
	int err = 0, filter_type = 0;
	struct aq_hw_rx_fltrs_s *rx_fltrs = aq_get_hw_rx_fltrs(aq_nic);

	if (fsp->flow_type & FLOW_EXT) {
		if ((be16_to_cpu(fsp->m_ext.vlan_tci) & VLAN_VID_MASK) == VLAN_VID_MASK) {
			err = aq_check_approve_fvlan(aq_nic, rx_fltrs, fsp);
			filter_type |= aq_rx_filter_type_vlan;
		}
		if (!err && (be16_to_cpu(fsp->m_ext.vlan_tci) & VLAN_PRIO_MASK) == VLAN_PRIO_MASK) {
			err = aq_check_approve_fvlan_pcp(aq_nic, rx_fltrs, fsp);
			filter_type |= aq_rx_filter_type_vlan_pcp;
		} 
		if (!err && (be16_to_cpu(fsp->m_ext.vlan_tci) & (~(VLAN_PRIO_MASK | VLAN_VID_MASK)))) {
			netdev_err(aq_nic->ndev,
				     "ethtool: invalid vlan mask 0x%x specified",
				     be16_to_cpu(fsp->m_ext.vlan_tci));
			err = -EINVAL;
		}
	}

	if(!err) {
		switch (fsp->flow_type & ~FLOW_EXT) {
		case ETHER_FLOW:
			if( fsp->m_u.ether_spec.h_proto ) {
				err = aq_check_approve_fl2(aq_nic, rx_fltrs, fsp);
			}
			if( ((uint32_t *)fsp->m_u.ether_spec.h_dest)[0] || 
				((uint16_t *)fsp->m_u.ether_spec.h_dest)[2] ) {
				err = aq_check_approve_dst_fl2(aq_nic, rx_fltrs, fsp);
			}
			filter_type |= aq_rx_filter_type_ethertype;
			break;
		case TCP_V4_FLOW:
		case UDP_V4_FLOW:
		case SCTP_V4_FLOW:
		case IP_USER_FLOW:
			if( a2_new_filter ) {
				filter_type |= (fsp->flow_type & ~FLOW_EXT) == SCTP_V4_FLOW ? 
								aq_rx_filter_type_l4flex : aq_rx_filter_type_l3l4;
				err = aq_check_approve_l4(aq_nic, rx_fltrs, fsp);
				if( err )
					break;
			}
		case IPV4_FLOW:
			is_ipv6 = false;
			if( !a2_new_filter ) {
				filter_type |= aq_rx_filter_type_l3l4;
				err = aq_check_approve_fl3l4(aq_nic, rx_fltrs, fsp, is_ipv6);
			} else {
				if( fsp->m_u.tcp_ip4_spec.ip4src ||
					fsp->m_u.tcp_ip4_spec.ip4dst ) {
					filter_type |= aq_rx_filter_type_ipv4;
					err = aq_check_approve_ipv4(aq_nic, rx_fltrs, fsp);
				}
			}
			break;
		case TCP_V6_FLOW:
		case UDP_V6_FLOW:
		case SCTP_V6_FLOW:
		case IPV6_USER_FLOW:
			if( a2_new_filter ) {
				filter_type |= (fsp->flow_type & ~FLOW_EXT) == SCTP_V6_FLOW ? 
								aq_rx_filter_type_l4flex : aq_rx_filter_type_l3l4;
				err = aq_check_approve_l4(aq_nic, rx_fltrs, fsp);
				if( err )
					break;
			}
		case IPV6_FLOW:
			is_ipv6 = true;
			if( !a2_new_filter ) {
				filter_type |= aq_rx_filter_type_l3l4;
				err = aq_check_approve_fl3l4(aq_nic, rx_fltrs, fsp, is_ipv6);
			} else {
				if( fsp->m_u.tcp_ip6_spec.ip6src[0] || 
				    fsp->m_u.tcp_ip6_spec.ip6src[3] ||
					fsp->m_u.tcp_ip6_spec.ip6dst[0] ||
					fsp->m_u.tcp_ip6_spec.ip6dst[3] ) {
					filter_type |= aq_rx_filter_type_ipv6;
					err = aq_check_approve_ipv6(aq_nic, rx_fltrs, fsp);
				}
			}
			break;
		default:
			netdev_err(aq_nic->ndev,
				     "ethtool: unknown flow-type specified");
			err = -EINVAL;
		}
	}

	if( !err && fsp->location >= RX_CLS_LOC_LAST ) {
		u32 f_first = 0, f_last = AQ_RX_MAX_RXNFC_LOC, new_location;
		if (hweight_long(filter_type) == 1) {
			switch(filter_type) {
				case aq_rx_filter_type_vlan:
					f_first = AQ_RX_FIRST_LOC_FVLANID;
					f_last = AQ_RX_LAST_LOC_FVLANID;
					break;
				case aq_rx_filter_type_ethertype:
					if( fsp->m_u.ether_spec.h_proto ) {
						f_first = AQ_RX_FIRST_LOC_FETHERT;
						f_last = AQ_RX_LAST_LOC_FETHERT;
					} else {
						f_first = AQ_RX_FIRST_LOC_L2DST;
						f_last = AQ_RX_LAST_LOC_L2DST;
					}
					break;
				case aq_rx_filter_type_ipv4:
				case aq_rx_filter_type_ipv6:
					f_first = is_ipv6 ? AQ_RX_FIRST_LOC_FIPV6 : AQ_RX_FIRST_LOC_FIPV4;
					f_last = is_ipv6 ? AQ_RX_LAST_LOC_FIPV6 : AQ_RX_LAST_LOC_FIPV4;
					break;
				case aq_rx_filter_type_l3l4:
					f_first = AQ_RX_FIRST_LOC_FL3L4 + 
							(is_ipv6 ? (rx_fltrs->fl3l4.active_ipv4 & 0x0f ? 4 : 0) : 
									   (rx_fltrs->fl3l4.active_ipv6 & 0x01 ? 4 : 0));
					f_last = AQ_RX_LAST_LOC_FL3L4 - 
							(is_ipv6 ? (rx_fltrs->fl3l4.active_ipv4 & 0xf0 ? 7 : 3) : 
									   (rx_fltrs->fl3l4.active_ipv6 & 0x02 ? 4 : 0));
					break;
			}
		}
		else if ( hweight_long(filter_type) > 1 ) {
			f_first = a2_new_filter ? AQ_RX_FIRST_LOC_CHAIN : AQ_RX_FIRST_LOC_FLEX;
			f_last = a2_new_filter ? AQ_RX_LAST_LOC_CHAIN : AQ_RX_LAST_LOC_FLEX;
			filter_type = a2_new_filter ? aq_rx_filter_type_chain : aq_rx_filter_type_flex;
		}
		new_location = get_empty_location(aq_nic, f_first, f_last, fsp->location, filter_type);
		if( new_location == RX_CLS_LOC_ANY ) {
			netdev_err(aq_nic->ndev,
					"ethtool: no place for flex/chain filters");
			err = -EINVAL;
		} else {
			//aq_nic_print(aq_nic, info, drv, "Add filter type %x, requested location %x, new location %x\n", 
			//								filter_type, fsp->location, new_location);
			fsp->location = new_location;
		}
	}

	return err;
}

static bool __must_check
aq_rule_is_not_support(struct aq_nic_s *aq_nic,
		       struct ethtool_rx_flow_spec *fsp)
{
	bool rule_is_not_support = false;
	u32 l3l4_flow_type = fsp->flow_type;
	if (!(aq_nic->ndev->features & NETIF_F_NTUPLE)) {
		netdev_err(aq_nic->ndev,
			     "ethtool: Please, to enable the RX flow control:\n"
			     "ethtool -K %s ntuple on\n", aq_nic->ndev->name);
		rule_is_not_support = true;
	} else if ((fsp->h_ext.vlan_tci || fsp->h_ext.vlan_etype) &&
		   !(aq_nic->ndev->features & NETIF_F_HW_VLAN_CTAG_FILTER)) {
		netdev_err(aq_nic->ndev,
			     "ethtool: Please, to enable the RX vlan filter:\n"
			     "ethtool -K %s rx-vlan-filter on\n",
			     aq_nic->ndev->name);
		rule_is_not_support = true;
	} else if (!aq_rule_is_approve(fsp)) {
		netdev_err(aq_nic->ndev,
			     "ethtool: The specified flow type is not supported\n");
		rule_is_not_support = true;
	}
	else if ((l3l4_flow_type & ~FLOW_EXT) != ETHER_FLOW) {
		bool is_ipv6 =  (l3l4_flow_type == TCP_V6_FLOW) || 
						(l3l4_flow_type == UDP_V6_FLOW) || 
						(l3l4_flow_type == SCTP_V6_FLOW) || 
						(l3l4_flow_type == IPV6_USER_FLOW) || 
						(l3l4_flow_type == IPV6_FLOW);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0)
		if ((fsp->h_u.tcp_ip4_spec.tos && !is_ipv6) ||
		     (fsp->h_u.tcp_ip6_spec.tclass && is_ipv6)) {
#else
		if (fsp->h_u.tcp_ip4_spec.tos) {
#endif
		netdev_err(aq_nic->ndev,
			     "ethtool: The specified tos/tclass is not supported\n");
		rule_is_not_support = true;
		}
	}

	return rule_is_not_support;
}

static bool __must_check
aq_rule_is_not_correct(struct aq_nic_s *aq_nic,
		       struct ethtool_rx_flow_spec *fsp)
{
	bool rule_is_not_correct = false;

	if (!aq_nic) {
		rule_is_not_correct = true;
	} else if (fsp->flow_type & FLOW_MAC_EXT) {
		netdev_err(aq_nic->ndev,
			     "ethtool: MAC_EXT is not supported");
		rule_is_not_correct = true;
	} else if (aq_check_filter(aq_nic, fsp)) {
		rule_is_not_correct = true;
	} else if (fsp->ring_cookie != RX_CLS_FLOW_DISC) {
#ifndef TSN_SUPPORT
		u32 rx_tc_mode;
		aq_nic->aq_hw_ops->hw_rx_tc_mode_get(aq_nic->aq_hw, &rx_tc_mode);
		if (fsp->ring_cookie >= HW_ATL_RX_MAX_QUEUE &&
		    ((rx_tc_mode == 1 && fsp->ring_cookie != AQ_CFG_PTP_4TC_RING_IDX) ||
			 (rx_tc_mode == 0 && fsp->ring_cookie != AQ_CFG_PTP_8TC_RING_IDX)) ) {
			netdev_err(aq_nic->ndev,
				     "ethtool: The specified action is invalid.\n"
				     "Maximum allowable value action is %u.\n",
				     HW_ATL_RX_MAX_QUEUE - 1);
			rule_is_not_correct = true;
		}
#endif
	}

	return rule_is_not_correct;
}

static int __must_check
aq_check_rule(struct aq_nic_s *aq_nic,
	      struct ethtool_rx_flow_spec *fsp)
{
	int err = 0;

	if (aq_rule_is_not_correct(aq_nic, fsp))
		err = -EINVAL;
	else if (aq_rule_is_not_support(aq_nic, fsp))
		err = -EOPNOTSUPP;
	else if (aq_rule_already_exists(aq_nic, fsp))
		err = -EEXIST;

	return err;
}

static void aq_set_data_fl2(struct aq_nic_s *aq_nic,
			   struct aq_rx_filter *aq_rx_fltr,
			   struct aq_rx_filter_l2 *data, bool add)
{
	struct ethtool_rx_flow_spec *fsp = &aq_rx_fltr->aq_fsp;
	u32 location = fsp->location & AQ_RX_FILTER_CHAIN_MASK;

	memset(data, 0, sizeof(*data));

	if( fsp->m_u.ether_spec.h_proto ) {
		if( add ) {
			data->location = (location <= AQ_RX_MAX_RXNFC_LOC ? 
							  location : get_empty_location(aq_nic, 
											AQ_RX_FIRST_LOC_FETHERT, 
											AQ_RX_LAST_LOC_FETHERT, 
											RX_CLS_LOC_ANY,
											aq_rx_filter_type_ethertype)) - AQ_RX_FIRST_LOC_FETHERT;
			//printk("Locations fsp %x, data %x\n", location, data->location);
		} else {
			data->location = location <= AQ_RX_MAX_RXNFC_LOC ? 
							 location - AQ_RX_FIRST_LOC_FETHERT : 
						  (((fsp->location & AQ_RX_FILTER_CHAIN_L2ET_LOC_MASK) >> AQ_RX_FILTER_CHAIN_L2ET_LOC_SHIFT) - 1);
		}
	} else {
		if( add ) {
			data->location = (location <= AQ_RX_MAX_RXNFC_LOC ? 
							  location : get_empty_location(aq_nic, 
											AQ_RX_FIRST_LOC_L2DST, 
											AQ_RX_LAST_LOC_L2DST, 
											RX_CLS_LOC_ANY,
											aq_rx_filter_type_ethertype) ) - AQ_RX_FIRST_LOC_L2DST;
		} else {
			data->location = location <= AQ_RX_MAX_RXNFC_LOC ? 
							 location - AQ_RX_FIRST_LOC_L2DST : 
							(((fsp->location & AQ_RX_FILTER_CHAIN_L2_DST_MASK) >> AQ_RX_FILTER_CHAIN_L2_DST_SHIFT) - 1); 
		}
	}
	
	if (fsp->ring_cookie != RX_CLS_FLOW_DISC)
		data->queue = location <= AQ_RX_MAX_RXNFC_LOC ? fsp->ring_cookie : 0x7F;
	else
		data->queue = -1;

	data->en_flag = 0;
	if( fsp->m_u.ether_spec.h_proto ) {
		
		data->ethertype = be16_to_cpu(fsp->h_u.ether_spec.h_proto);
		data->en_flag |= HW_ATL_RX_L2_ET_EN | 
			(be16_to_cpu(fsp->m_ext.vlan_tci) == VLAN_PRIO_MASK ? HW_ATL_RX_L2_UP_EN : 0);
		data->user_priority = (be16_to_cpu(fsp->h_ext.vlan_tci)
					& VLAN_PRIO_MASK) >> VLAN_PRIO_SHIFT;

		if( add && location > AQ_RX_MAX_RXNFC_LOC ) {
			fsp->location &= ~AQ_RX_FILTER_CHAIN_L2ET_LOC_MASK;
			fsp->location |= (data->location + 1) << AQ_RX_FILTER_CHAIN_L2ET_LOC_SHIFT;
		}
	}

	if( aq_new_filters_enabled && 
		( ((uint32_t *)fsp->m_u.ether_spec.h_dest)[0] ||
		  ((uint16_t *)fsp->m_u.ether_spec.h_dest)[2] ) 
	  ) {
		uint32_t i;
		for (i = 0U;
				i < aq_nic->mc_list.count;
				i++) {
			if( ((uint32_t *)fsp->h_u.ether_spec.h_dest)[0] == ((uint32_t *)aq_nic->mc_list.ar[i])[0] &&
		  		((uint16_t *)fsp->h_u.ether_spec.h_dest)[2] == ((uint16_t *)aq_nic->mc_list.ar[i])[2] )
			{
				break;
			}			
		}

		if( i < aq_nic->mc_list.count ) 
		{
			if(	location > AQ_RX_MAX_RXNFC_LOC ) {
				fsp->location &= ~AQ_RX_FILTER_CHAIN_L2_DST_MASK;
				fsp->location |= (i+1) << AQ_RX_FILTER_CHAIN_L2_DST_SHIFT;
			}
			data->location |= (i+1) << 4;
			data->en_flag |= HW_ATL_RX_L2_DST_EN;
		}
	}

	if (netif_msg_link(aq_nic))
		netdev_dbg(aq_nic->ndev,
			"etherfilter[%d] = {add:%d, ethertype:%x, en_flags:%d, user_priority:%d}",
			 data->location,
			 add,
			 data->ethertype,
			 data->en_flag,
			 data->user_priority);
}

static int aq_add_del_fether(struct aq_nic_s *aq_nic,
			     struct aq_rx_filter *aq_rx_fltr, bool add)
{
	struct aq_rx_filter_l2 data;
	struct aq_hw_s *aq_hw = aq_nic->aq_hw;
	const struct aq_hw_ops *aq_hw_ops = aq_nic->aq_hw_ops;

	aq_set_data_fl2(aq_nic, aq_rx_fltr, &data, add);

	if (unlikely(!aq_hw_ops->hw_filter_l2_set))
		return -EOPNOTSUPP;
	if (unlikely(!aq_hw_ops->hw_filter_l2_clear))
		return -EOPNOTSUPP;

	if (add)
		return aq_hw_ops->hw_filter_l2_set(aq_hw, &data);
	else
		return aq_hw_ops->hw_filter_l2_clear(aq_hw, &data);
}

static int aq_add_del_vlan_pcp(struct aq_nic_s *aq_nic,
			    struct aq_rx_filter *aq_rx_fltr, bool add)
{
	//const struct aq_hw_ops *aq_hw_ops = aq_nic->aq_hw_ops;
	//TODO something
	return 0;
}

static void aq_fvlan_print(struct aq_nic_s *aq_nic,
			   const struct aq_rx_filter_vlan *aq_vlans)
{
	int i;

	if (netif_msg_link(aq_nic)) {
		for (i = 0; i < AQ_RX_QUEUE_NOT_ASSIGNED; ++i) {
			netdev_dbg(aq_nic->ndev,
				"vlans[%d] = {en:%d, vlan:%d, queue:%d}", i,
				 aq_vlans[i].enable,
				 aq_vlans[i].vlan_id,
				 aq_vlans[i].queue);
		}
	}
}

static void aq_fvlan_rebuild(struct aq_nic_s *aq_nic,
		     unsigned long *active_vlans,
		     struct aq_rx_filter_vlan *aq_vlans)
{
	bool vlan_busy = false;
	int vlan = -1;
	int i, j;

	for (i = 0; i < AQ_RX_QUEUE_NOT_ASSIGNED; ++i) {
		if ((!aq_vlans[i].enable)
		    || (aq_vlans[i].queue == AQ_RX_QUEUE_NOT_ASSIGNED)) {
			do {
				vlan = find_next_bit(active_vlans,
						     VLAN_N_VID,
						     vlan + 1);
				if (vlan == VLAN_N_VID) {
					aq_vlans[i].enable = 0U;
					aq_vlans[i].queue = AQ_RX_QUEUE_NOT_ASSIGNED;
					aq_vlans[i].vlan_id = 0;
					continue;
				}

				vlan_busy = false;
				for (j = 0; j < AQ_RX_QUEUE_NOT_ASSIGNED; ++j) {
					if (aq_vlans[j].enable
					   && (aq_vlans[j].queue 
					       != AQ_RX_QUEUE_NOT_ASSIGNED)
					   && (aq_vlans[j].vlan_id == vlan)) {
						vlan_busy = true;
						break;
					}
				}
				if (!vlan_busy) {
					aq_vlans[i].enable = 1U;
					aq_vlans[i].queue = AQ_RX_QUEUE_NOT_ASSIGNED;
					aq_vlans[i].vlan_id = vlan;
				}
			} while (vlan_busy && vlan != VLAN_N_VID);
		}
	}

	aq_fvlan_print(aq_nic, aq_vlans);
}

static int aq_set_data_fvlan(struct aq_nic_s *aq_nic,
			     struct aq_rx_filter *aq_rx_fltr,
			     struct aq_rx_filter_vlan *aq_vlans, bool add)
{
	struct ethtool_rx_flow_spec *fsp = &aq_rx_fltr->aq_fsp;
	u32 fsp_location = fsp->location & AQ_RX_FILTER_CHAIN_MASK;
	int location = AQ_RX_QUEUE_NOT_ASSIGNED;
	int i;

	if( add ) {
		location = (fsp_location <= AQ_RX_MAX_RXNFC_LOC ? 
					fsp_location : 
					get_empty_location(aq_nic, 
									AQ_RX_FIRST_LOC_FVLANID, 
									AQ_RX_LAST_LOC_FVLANID, 
									RX_CLS_LOC_ANY,
									aq_rx_filter_type_vlan) ) - AQ_RX_FIRST_LOC_FVLANID;
	} else {
		location = fsp_location <= AQ_RX_MAX_RXNFC_LOC ? 
				   fsp_location - AQ_RX_FIRST_LOC_FVLANID : 
				  (fsp->location & AQ_RX_FILTER_CHAIN_VLAN_LOC_MASK) >> AQ_RX_FILTER_CHAIN_VLAN_LOC_SHIFT;
	}

	memset(&aq_vlans[location], 0, sizeof(aq_vlans[location]));

	if (!add)
		return 0;

	if( fsp_location > AQ_RX_MAX_RXNFC_LOC ) {
		fsp->location &= ~AQ_RX_FILTER_CHAIN_VLAN_LOC_MASK;
		fsp->location |= location << AQ_RX_FILTER_CHAIN_VLAN_LOC_SHIFT;
	}

	/* remove vlan if it was in table without queue assignment */
	for (i = 0; i < AQ_RX_QUEUE_NOT_ASSIGNED; ++i) {
		if (aq_vlans[i].vlan_id ==
		   (be16_to_cpu(fsp->h_ext.vlan_tci) & VLAN_VID_MASK)) {
			aq_vlans[i].enable = false;
		}
	}

	aq_vlans[location].location = location;
	aq_vlans[location].vlan_id = be16_to_cpu(fsp->h_ext.vlan_tci)
				     & VLAN_VID_MASK;
	aq_vlans[location].queue = fsp_location <= AQ_RX_MAX_RXNFC_LOC ? fsp->ring_cookie & 0x1FU : 0x7F;
	aq_vlans[location].enable = 1U;
	return 0;
}

static int aq_add_del_fvlan(struct aq_nic_s *aq_nic,
			    struct aq_rx_filter *aq_rx_fltr, bool add)
{
	const struct aq_hw_ops *aq_hw_ops = aq_nic->aq_hw_ops;

	if (unlikely(!aq_hw_ops->hw_filter_vlan_set))
		return -EOPNOTSUPP;

	aq_set_data_fvlan(aq_nic,
			  aq_rx_fltr,
			  aq_nic->aq_hw_rx_fltrs.fl2.aq_vlans,
			  add);

	return aq_filters_vlans_update(aq_nic);
}

static int aq_set_data_fl3(struct aq_nic_s *aq_nic,
			     struct aq_rx_filter *aq_rx_fltr,
			     struct aq_rx_filter_l3 *data, bool ipv6, bool add)
{
	struct ethtool_rx_flow_spec *fsp = &aq_rx_fltr->aq_fsp;
	u32 fsp_location = fsp->location & AQ_RX_FILTER_CHAIN_MASK;
	memset(data, 0, sizeof(*data));
	data->is_ipv6 = ipv6;
	if( add ) {
		data->location = fsp_location <= AQ_RX_MAX_RXNFC_LOC ? 
						 fsp_location :
						 get_empty_location(aq_nic,
								ipv6 ? AQ_RX_FIRST_LOC_FIPV6 : AQ_RX_FIRST_LOC_FIPV4, 
								ipv6 ? AQ_RX_LAST_LOC_FIPV6 : AQ_RX_LAST_LOC_FIPV4, 
								RX_CLS_LOC_ANY,
								ipv6 ? aq_rx_filter_type_ipv6 : aq_rx_filter_type_ipv4) - 
							   (ipv6 ? AQ_RX_FIRST_LOC_FIPV6 : AQ_RX_FIRST_LOC_FIPV4);
	} else {
		data->location = fsp_location <= AQ_RX_MAX_RXNFC_LOC ? 
						 fsp_location - (ipv6 ? AQ_RX_FIRST_LOC_FIPV6 : AQ_RX_FIRST_LOC_FIPV4) :
						(fsp->location & AQ_RX_FILTER_CHAIN_L3_LOC_MASK) >> AQ_RX_FILTER_CHAIN_L3_LOC_SHIFT;
	}

	if (!add) {
		return 0;
	}

	if( fsp_location > AQ_RX_MAX_RXNFC_LOC ) {
		fsp->location &= ~AQ_RX_FILTER_CHAIN_L3_LOC_MASK;
		fsp->location |= data->location << AQ_RX_FILTER_CHAIN_L3_LOC_SHIFT;
	}

	if (!ipv6) {
		data->ip_src[0] =
			ntohl(fsp->h_u.tcp_ip4_spec.ip4src);
		data->ip_dst[0] =
			ntohl(fsp->h_u.tcp_ip4_spec.ip4dst);
	} 
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0)
	 else {
		int i;

		for (i = 0; i < HW_ATL_RX_CNT_REG_ADDR_IPV6; ++i) {
			data->ip_dst[i] =
				ntohl(fsp->h_u.tcp_ip6_spec.ip6dst[i]) & ntohl(fsp->m_u.tcp_ip6_spec.ip6dst[i]);
			data->ip_src[i] =
				ntohl(fsp->h_u.tcp_ip6_spec.ip6src[i]) & ntohl(fsp->m_u.tcp_ip6_spec.ip6src[i]);
		}
		data->cmd |= HW_ATL_RX_ENABLE_L3_IPV6;
	}
#endif
	switch( fsp->flow_type & ~FLOW_EXT ){
	case TCP_V4_FLOW:
	case TCP_V6_FLOW:
	data->cmd |= (HW_ATL_RX_TCP | HW_ATL_RX_ENABLE_CMP_PROT_L4);
	break;
	case UDP_V4_FLOW:
	case UDP_V6_FLOW:
	data->cmd |= (HW_ATL_RX_UDP | HW_ATL_RX_ENABLE_CMP_PROT_L4);
	break;
	}
	
	if (data->ip_src[0])
		data->cmd |= HW_ATL_RX_ENABLE_CMP_SRC_ADDR_L3;
	if (data->ip_dst[0])
		data->cmd |= HW_ATL_RX_ENABLE_CMP_DEST_ADDR_L3;
	if (fsp->ring_cookie != RX_CLS_FLOW_DISC ) {
		data->cmd |= HW_ATL_RX_HOST << HW_ATL_RX_BOFFSET_ACTION_FL3F4;
		data->cmd |= (fsp_location < AQ_RX_FIRST_LOC_CHAIN ? fsp->ring_cookie : 0x7f) << HW_ATL_RX_BOFFSET_QUEUE_FL3L4;
		data->cmd |= HW_ATL_RX_ENABLE_QUEUE_L3L4;
	} else {
		data->cmd |= HW_ATL_RX_DISCARD << HW_ATL_RX_BOFFSET_ACTION_FL3F4;
	}
	return 0;
}

static int aq_add_del_fl3(struct aq_nic_s *aq_nic,
			    struct aq_rx_filter *aq_rx_fltr, bool ipv6, bool add)
{
	struct aq_rx_filter_l3 data;
	struct aq_hw_s *aq_hw = aq_nic->aq_hw;
	const struct aq_hw_ops *aq_hw_ops = aq_nic->aq_hw_ops;

	if (unlikely(!aq_hw_ops->hw_filter_l3_set))
		return -EOPNOTSUPP;

	if (unlikely(aq_set_data_fl3(aq_nic, aq_rx_fltr, &data, ipv6, add)))
		return -EINVAL;

	return aq_hw_ops->hw_filter_l3_set(aq_hw, &data);
}

static u32 aq_exsiting_l3l4_filter_location(struct aq_nic_s *aq_nic, bool is_ipv6, 
											 struct ethtool_rx_flow_spec *fsp)
{
	struct aq_hw_rx_fltrs_s *rx_fltrs = aq_get_hw_rx_fltrs(aq_nic);
	struct aq_rx_filter *rule = NULL;
	struct hlist_node *aq_node2;
	u32 fsp_location = fsp->location & AQ_RX_FILTER_CHAIN_MASK;

	hlist_for_each_entry_safe(rule, aq_node2,
				  &rx_fltrs->filter_list, aq_node) {
		u32 rule_loc = rule->aq_fsp.location;
		bool match = false;
		if( (rule_loc & AQ_RX_FILTER_CHAIN_MASK) == fsp_location ) {
			continue;
		}
		switch( (rule->aq_fsp.flow_type & ~FLOW_EXT) ) {
		case TCP_V6_FLOW:
		case UDP_V6_FLOW:
		case SCTP_V6_FLOW:
			if( is_ipv6 ) {
				match = (rule->aq_fsp.h_u.tcp_ip6_spec.psrc == fsp->h_u.tcp_ip6_spec.psrc) &&
						(rule->aq_fsp.m_u.tcp_ip6_spec.psrc == fsp->m_u.tcp_ip6_spec.psrc) && 
						(rule->aq_fsp.h_u.tcp_ip6_spec.pdst == fsp->h_u.tcp_ip6_spec.pdst) &&
						(rule->aq_fsp.m_u.tcp_ip6_spec.pdst == fsp->m_u.tcp_ip6_spec.pdst);
			} else {
				match = (rule->aq_fsp.h_u.tcp_ip6_spec.psrc == fsp->h_u.tcp_ip4_spec.psrc) &&
						(rule->aq_fsp.m_u.tcp_ip6_spec.psrc == fsp->m_u.tcp_ip4_spec.psrc) && 
						(rule->aq_fsp.h_u.tcp_ip6_spec.pdst == fsp->h_u.tcp_ip4_spec.pdst) &&
						(rule->aq_fsp.m_u.tcp_ip6_spec.pdst == fsp->m_u.tcp_ip4_spec.pdst);
			}
			break;
		case TCP_V4_FLOW:
		case UDP_V4_FLOW:
		case SCTP_V4_FLOW:
			if( is_ipv6 ) {
				match = (rule->aq_fsp.h_u.udp_ip4_spec.psrc == fsp->h_u.tcp_ip6_spec.psrc) &&
						(rule->aq_fsp.m_u.udp_ip4_spec.psrc == fsp->m_u.tcp_ip6_spec.psrc) && 
						(rule->aq_fsp.h_u.udp_ip4_spec.pdst == fsp->h_u.tcp_ip6_spec.pdst) &&
						(rule->aq_fsp.m_u.udp_ip4_spec.pdst == fsp->m_u.tcp_ip6_spec.pdst);
			} else {
				match = (rule->aq_fsp.h_u.tcp_ip4_spec.psrc == fsp->h_u.tcp_ip4_spec.psrc) &&
						(rule->aq_fsp.m_u.tcp_ip4_spec.psrc == fsp->m_u.tcp_ip4_spec.psrc) && 
						(rule->aq_fsp.h_u.tcp_ip4_spec.pdst == fsp->h_u.tcp_ip4_spec.pdst) &&
						(rule->aq_fsp.m_u.tcp_ip4_spec.pdst == fsp->m_u.tcp_ip4_spec.pdst);
			}
			break;
		}
		if (match) {
			return (rule_loc & AQ_RX_FILTER_CHAIN_MASK) > AQ_RX_MAX_RXNFC_LOC ? 
				   ((rule_loc >> AQ_RX_FILTER_CHAIN_L3L4_LOC_SHIFT) & 0x7) :
				    (rule_loc & AQ_RX_FILTER_CHAIN_MASK);
		}
	}

	return fsp_location;
}

static int aq_set_data_fl3l4(struct aq_nic_s *aq_nic,
			     struct aq_rx_filter *aq_rx_fltr,
			     struct aq_rx_filter_l3l4 *data, bool is_ipv6, bool add)
{
	struct aq_hw_rx_fltrs_s *rx_fltrs = aq_get_hw_rx_fltrs(aq_nic);
	struct ethtool_rx_flow_spec *fsp = &aq_rx_fltr->aq_fsp;
	u32 fsp_location = fsp->location & AQ_RX_FILTER_CHAIN_MASK;
	u32 l3l4_flow_type = fsp->flow_type & ~FLOW_EXT;
	bool a2_new_filter = aq_nic->aq_hw_ops->hw_filter_chain_build && aq_new_filters_enabled;
	bool l3_only = l3l4_flow_type == IPV4_FLOW || l3l4_flow_type == IPV6_FLOW;

	memset(data, 0, sizeof(*data));

	data->is_ipv6 = is_ipv6;

	if( add )
		if( a2_new_filter ) {
			u32 location = aq_exsiting_l3l4_filter_location(aq_nic, is_ipv6, fsp);
			data->location = location <= AQ_RX_MAX_RXNFC_LOC ? 
							 location :
							 get_empty_location(aq_nic,
								AQ_RX_FIRST_LOC_FL3L4, 
								AQ_RX_LAST_LOC_FL3L4, 
								RX_CLS_LOC_ANY,
								aq_rx_filter_type_l3l4) - AQ_RX_FIRST_LOC_FL3L4;
		} else {
			u32 first_loc = AQ_RX_FIRST_LOC_FL3L4 + 
							(is_ipv6 ? (rx_fltrs->fl3l4.active_ipv4 & 0x0f ? 4 : 0) : 
									   (rx_fltrs->fl3l4.active_ipv6 & 0x01 ? 4 : 0));
			u32 last_loc = AQ_RX_LAST_LOC_FL3L4 - 
							(is_ipv6 ? (rx_fltrs->fl3l4.active_ipv4 & 0xf0 ? 7 : 3) : 
									   (rx_fltrs->fl3l4.active_ipv6 & 0x02 ? 4 : 0));
			data->location = (fsp_location <= AQ_RX_MAX_RXNFC_LOC ? 
						fsp_location :
						get_empty_location(aq_nic,
								first_loc, 
								last_loc, 
								RX_CLS_LOC_ANY,
								aq_rx_filter_type_l3l4) ) - AQ_RX_FIRST_LOC_FL3L4;
		}
	else {
		data->location = fsp_location <= AQ_RX_MAX_RXNFC_LOC ? 
						fsp_location -AQ_RX_FIRST_LOC_FL3L4 :
						(fsp->location & AQ_RX_FILTER_CHAIN_L3L4_LOC_MASK) >> AQ_RX_FILTER_CHAIN_L3L4_LOC_SHIFT;
	}

	if (!add) {
		if (!data->is_ipv6)
			rx_fltrs->fl3l4.active_ipv4 &= ~BIT(data->location);
		else
			rx_fltrs->fl3l4.active_ipv6 &= ~BIT((data->location)/4);
		return 0;
	}

	if( fsp_location > AQ_RX_MAX_RXNFC_LOC ) {
		fsp->location &= ~AQ_RX_FILTER_CHAIN_L3L4_LOC_MASK;
		fsp->location |= data->location << AQ_RX_FILTER_CHAIN_L3L4_LOC_SHIFT;
	}

	if( !l3_only ) {
		data->cmd |= HW_ATL_RX_ENABLE_FLTR_L3L4;
		switch (l3l4_flow_type) {
		case TCP_V4_FLOW:
		case TCP_V6_FLOW:
			data->cmd |= HW_ATL_RX_ENABLE_CMP_PROT_L4;
			break;
		case UDP_V4_FLOW:
		case UDP_V6_FLOW:
			data->cmd |= HW_ATL_RX_UDP;
			data->cmd |= HW_ATL_RX_ENABLE_CMP_PROT_L4;
			break;
		case SCTP_V4_FLOW:
		case SCTP_V6_FLOW:
			data->cmd |= HW_ATL_RX_SCTP;
			data->cmd |= HW_ATL_RX_ENABLE_CMP_PROT_L4;
			break;
		default:
			break;
		}
	}

	if( !a2_new_filter ) {
		if (!data->is_ipv6) {
			data->ip_src[0] =
				ntohl(fsp->h_u.tcp_ip4_spec.ip4src);
			data->ip_dst[0] =
				ntohl(fsp->h_u.tcp_ip4_spec.ip4dst);
			rx_fltrs->fl3l4.active_ipv4 |= BIT(data->location);
		} 
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0)
		else {
			int i;

			rx_fltrs->fl3l4.active_ipv6 |= BIT((data->location)/4);
			for (i = 0; i < HW_ATL_RX_CNT_REG_ADDR_IPV6; ++i) {
				data->ip_dst[i] =
					ntohl(fsp->h_u.tcp_ip6_spec.ip6dst[i]);
				data->ip_src[i] =
					ntohl(fsp->h_u.tcp_ip6_spec.ip6src[i]);
			}
			data->cmd |= HW_ATL_RX_ENABLE_L3_IPV6;
		}
#endif
	}
	if (l3l4_flow_type == TCP_V4_FLOW || 
	    l3l4_flow_type == UDP_V4_FLOW ||
	    l3l4_flow_type == SCTP_V4_FLOW) {
		data->p_dst = ntohs(fsp->h_u.tcp_ip4_spec.pdst);
		data->p_src = ntohs(fsp->h_u.tcp_ip4_spec.psrc);
	}
	//printk("fsp->flow_type %x, data->p_dst %x\n", l3l4_flow_type, data->p_dst);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0)
	if (l3l4_flow_type == TCP_V6_FLOW || 
	    l3l4_flow_type == UDP_V6_FLOW ||
	    l3l4_flow_type == SCTP_V6_FLOW) {
    	data->p_dst = ntohs(fsp->h_u.tcp_ip6_spec.pdst);
    	data->p_src = ntohs(fsp->h_u.tcp_ip6_spec.psrc);
	}
#endif
	if (data->ip_src[0])
		data->cmd |= HW_ATL_RX_ENABLE_CMP_SRC_ADDR_L3;
	if (data->ip_dst[0])
		data->cmd |= HW_ATL_RX_ENABLE_CMP_DEST_ADDR_L3;
	if (data->p_dst)
		data->cmd |= HW_ATL_RX_ENABLE_CMP_DEST_PORT_L4;
	if (data->p_src)
		data->cmd |= HW_ATL_RX_ENABLE_CMP_SRC_PORT_L4;
	if (fsp->ring_cookie != RX_CLS_FLOW_DISC ) {
		data->cmd |= HW_ATL_RX_HOST << HW_ATL_RX_BOFFSET_ACTION_FL3F4;
		data->cmd |= (fsp_location <= AQ_RX_MAX_RXNFC_LOC ? fsp->ring_cookie : 0x7f) << HW_ATL_RX_BOFFSET_QUEUE_FL3L4;
		data->cmd |= HW_ATL_RX_ENABLE_QUEUE_L3L4;
	} else {
		data->cmd |= HW_ATL_RX_DISCARD << HW_ATL_RX_BOFFSET_ACTION_FL3F4;
	}
	return 0;
}

static int aq_add_del_fl3l4(struct aq_nic_s *aq_nic,
			    struct aq_rx_filter *aq_rx_fltr, bool ipv6, bool add)
{
	struct aq_rx_filter_l3l4 data;
	struct aq_hw_s *aq_hw = aq_nic->aq_hw;
	const struct aq_hw_ops *aq_hw_ops = aq_nic->aq_hw_ops;

	if (unlikely(!aq_hw_ops->hw_filter_l3l4_set))
		return -EOPNOTSUPP;

	if (unlikely(aq_set_data_fl3l4(aq_nic, aq_rx_fltr, &data, ipv6, add)))
		return -EINVAL;

	return aq_hw_ops->hw_filter_l3l4_set(aq_hw, &data);
}

static inline void flex_insert_byte(u8 *flex_mask, unsigned long byteen[4], u32 offset, u8 val)
{
	flex_mask[offset] = val;
	set_bit(offset++, byteen);
}

static int aq_set_data_flex(struct aq_nic_s *aq_nic,
			     struct aq_rx_filter *aq_rx_fltr,
			     struct aq_rx_filter_flex *data, bool add)
{
	struct ethtool_rx_flow_spec *fsp = &aq_rx_fltr->aq_fsp;
	u32 fsp_location = fsp->location & AQ_RX_FILTER_CHAIN_MASK;
	u32 offset = 0;
	bool vlan; //Read reg
	//bool dvlan = false;
	u8 *flex_mask = (u8 *)&data->mask;

	memset(data, 0, sizeof(*data));
	if( add ) {
		data->location = (fsp_location <= AQ_RX_MAX_RXNFC_LOC ?
						  fsp_location : 
						  get_empty_location(aq_nic, 
								AQ_RX_FIRST_LOC_FLEX, 
								AQ_RX_LAST_LOC_FLEX, 
								RX_CLS_LOC_ANY,
								aq_rx_filter_type_flex)) - AQ_RX_FIRST_LOC_FLEX;
	} else {
		data->location =  fsp_location <= AQ_RX_MAX_RXNFC_LOC ?
						  fsp_location - AQ_RX_FIRST_LOC_FLEX : 
					    ((fsp->location & AQ_RX_FILTER_CHAIN_FLEX_LOC_MASK) >> AQ_RX_FILTER_CHAIN_FLEX_LOC_SHIFT);
	}
	if (!add) {
		return 0;
	}
	//printk("fsp->location %d, data->location %d\n", fsp->location, data->location );
	if( fsp_location > AQ_RX_MAX_RXNFC_LOC ) {
		fsp->location &= ~AQ_RX_FILTER_CHAIN_FLEX_LOC_MASK;
		fsp->location |= data->location << AQ_RX_FILTER_CHAIN_FLEX_LOC_SHIFT;
	}

	data->cmd |= HW_ATL_RX_ENABLE_FLTR_FLEX;
	if (fsp->ring_cookie != RX_CLS_FLOW_DISC ) {
		data->cmd |= HW_ATL_RX_HOST << HW_ATL_RX_BOFFSET_ACTION_FLEX;
		data->cmd |= (fsp_location <= AQ_RX_MAX_RXNFC_LOC ? 
			fsp->ring_cookie : 0x7f) << HW_ATL_RX_BOFFSET_QUEUE_FLEX;
		data->cmd |= HW_ATL_RX_ENABLE_QUEUE_FLEX;
	} else {
		data->cmd |= HW_ATL_RX_DISCARD << HW_ATL_RX_BOFFSET_ACTION_FLEX;
	}

	// Flex VLAN
	if( aq_rx_fltr->type & (aq_rx_filter_type_vlan_pcp | aq_rx_filter_type_vlan) ) {
		u16 vlan_value = be16_to_cpu(aq_rx_fltr->aq_fsp.h_ext.vlan_tci);
		u16 vlan_mask = be16_to_cpu(aq_rx_fltr->aq_fsp.m_ext.vlan_tci);
		u16 vlan_et = be16_to_cpu(aq_rx_fltr->aq_fsp.h_ext.vlan_etype);
		u16 vlan_et_mask = be16_to_cpu(aq_rx_fltr->aq_fsp.m_ext.vlan_etype);
		//Single VLAN
		vlan = true;
		offset = 12; // After SRC MAC address + VLAN ET

		if( vlan_et_mask & 0xffff ) {
			flex_insert_byte(flex_mask, data->byteen, offset++, vlan_et >> 8);
			flex_insert_byte(flex_mask, data->byteen, offset++, vlan_et & 0xff);
		} else {
			offset += 2;
		}

		if( (vlan_mask & 0xff00) && ((vlan_mask & 0xff00) != 0xff00) ) {
			data->offset_a = offset;
			data->cmd |= HW_ATL_RX_BYTE_A_APPLY;
			data->mask_a = vlan_value >> 8;
			data->biten_a = vlan_mask >> 8;
		} else if( vlan_mask & 0xff00 ) {
			flex_insert_byte(flex_mask, data->byteen, offset++, vlan_value >> 8);
		}
		offset++;
		if( (vlan_mask & 0xff) == 0xff ) {
			flex_insert_byte(flex_mask, data->byteen, offset++, vlan_value & 0xff);
		} /*else if( vlan_mask & 0xff ){
			if( !(data->cmd & HW_ATL_RX_BYTE_A_APPLY) ) {
				data->offset_a = offset; 
				data->cmd |= HW_ATL_RX_BYTE_A_APPLY;
				data->mask_a = vlan_value & 0xff;
				data->biten_a = vlan_mask & 0xff;
			} else {
				data->offset_b = offset; 
				data->cmd |= HW_ATL_RX_BYTE_B_APPLY;
				data->mask_b = vlan_value & 0xff;
				data->biten_b = vlan_mask & 0xff;
			}
		}*/
	}

	// Flex Ethertype
	if( aq_rx_fltr->type & aq_rx_filter_type_ethertype ) {
		u16 et = be16_to_cpu(fsp->h_u.ether_spec.h_proto);
		flex_insert_byte(flex_mask, data->byteen, offset++, et >> 8);
		flex_insert_byte(flex_mask, data->byteen, offset++, et & 0xff);
	}
	// Flex l3/l4
	if( aq_rx_fltr->type & aq_rx_filter_type_l3l4 ) {
		offset = 14 + (vlan ? 4 : 0);
		//TODO
	}

	//Dump 
	data->filter_size = offset;
	/*
	printk("Flex filter: cmd %x, Byte A: offset %d, mask %x, pattern %x, Byte B: offset %d, mask %x, pattern %x\n",
			data->cmd, data->offset_a, data->mask_a, data->biten_a, data->offset_b, data->mask_b, data->biten_b);
	printk("Flex filter: Size %d, ByteEn: %08lx%08lx%08lx%08lx\n", data->filter_size,
			 data->byteen[3], data->byteen[2], data->byteen[1], data->byteen[0]);
	for( offset = 0; offset < data->filter_size; offset += 16 ) {
		u8 *buf = flex_mask + offset;
		printk("Flex filter: %02x: %02x %02x %02x %02x %02x %02x %02x %02x  %02x %02x %02x %02x %02x %02x %02x %02x\n",
				offset, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
				buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);
	}*/
	return 0;
}

static int aq_add_del_complex_filter(struct aq_nic_s *aq_nic,
			struct aq_rx_filter *aq_rx_fltr, bool add)
{
	struct aq_rx_filter_flex data;
	struct aq_hw_s *aq_hw = aq_nic->aq_hw;
	const struct aq_hw_ops *aq_hw_ops = aq_nic->aq_hw_ops;
	int err;
	if( aq_nic->aq_hw_ops->hw_filter_chain_build && aq_new_filters_enabled ) {
		err = aq_hw_ops->hw_filter_chain_build(aq_hw, aq_rx_fltr, add);
	}
	else {
		// Create flex filter
		if (unlikely(aq_set_data_flex(aq_nic, aq_rx_fltr, &data, add)))
			return -EINVAL;

		if (unlikely(!aq_hw_ops->hw_filter_l3l4_set))
			return -EOPNOTSUPP;
		
		err = aq_hw_ops->hw_filter_flex_set(aq_hw, &data);
	}
	return err;
}

static int aq_add_del_rule(struct aq_nic_s *aq_nic,
			   struct aq_rx_filter *aq_rx_fltr, bool add)
{
	int err = -EINVAL;
	const struct ethtool_rx_flow_spec *fsp = &aq_rx_fltr->aq_fsp;
	u32 fsp_loc = fsp->location & AQ_RX_FILTER_CHAIN_MASK;
	bool a2_new_filter = aq_nic->aq_hw_ops->hw_filter_chain_build && aq_new_filters_enabled;
	bool is_flex = fsp_loc >= AQ_RX_FIRST_LOC_FLEX && 
				   fsp_loc < AQ_RX_LAST_LOC_FLEX;
	bool tcp_udp = false;

	aq_rx_fltr->type = 0;
	if (fsp->flow_type & FLOW_EXT) {
		if ((be16_to_cpu(fsp->m_ext.vlan_tci) & VLAN_PRIO_MASK)
			== VLAN_PRIO_MASK ) {
			aq_rx_fltr->type |= aq_rx_filter_type_vlan_pcp;
			if( !is_flex ) {
				err = aq_add_del_vlan_pcp(aq_nic, aq_rx_fltr, add);
				if( err )
					return err;
			}
		}
		if ((be16_to_cpu(fsp->m_ext.vlan_tci) & VLAN_VID_MASK)
		    == VLAN_VID_MASK ) {
			aq_rx_fltr->type |= aq_rx_filter_type_vlan;
			if( !is_flex ) {
				err = aq_add_del_fvlan(aq_nic, aq_rx_fltr, add);
				if( err )
					return err;
			}
		}
	}
	switch (fsp->flow_type & ~FLOW_EXT) {
	case ETHER_FLOW:
		aq_rx_fltr->type |= aq_rx_filter_type_ethertype;
		if( !is_flex ) {
			err = aq_add_del_fether(aq_nic, aq_rx_fltr, add);
			if( err ) {
				return err;
			}
		}
		break;
	case TCP_V4_FLOW:
	case UDP_V4_FLOW:
	case IP_USER_FLOW:
		if( a2_new_filter ) {
			aq_rx_fltr->type |= aq_rx_filter_type_l3l4;
			err = aq_add_del_fl3l4(aq_nic, aq_rx_fltr, false, add);
			if( err ) {
				return err;
			tcp_udp = true;
			}
		}
	case SCTP_V4_FLOW:
		if( a2_new_filter && !tcp_udp ) {
			/*
			//TODO add SCTP
			err = aq_add_del_fl3l4(aq_nic, aq_rx_fltr, add);
			if( err ) {
				return err;
			}
			*/
		}
	case IPV4_FLOW:
		if( a2_new_filter ) {
			if( fsp->m_u.tcp_ip4_spec.ip4src ||
			    fsp->m_u.tcp_ip4_spec.ip4dst ) {
				aq_rx_fltr->type |= aq_rx_filter_type_ipv4;
				err = aq_add_del_fl3(aq_nic, aq_rx_fltr, false, add);
			}
		} else {
			aq_rx_fltr->type |= aq_rx_filter_type_l3l4;
			if( !is_flex  )
				err = aq_add_del_fl3l4(aq_nic, aq_rx_fltr, false, add);
		}
		if( err ) {
			return err;
		}
		break;
	case TCP_V6_FLOW:
	case UDP_V6_FLOW:
	case IPV6_USER_FLOW:
		if( a2_new_filter ) {
			aq_rx_fltr->type |= aq_rx_filter_type_l3l4;
			err = aq_add_del_fl3l4(aq_nic, aq_rx_fltr, true, add);
			if( err ) {
				return err;
			tcp_udp = true;
			}
		}
	case SCTP_V6_FLOW:
		if( a2_new_filter && !tcp_udp ) {
			/*
			//TODO add SCTP
			err = aq_add_del_fl3l4(aq_nic, aq_rx_fltr, add);
			if( err ) {
				return err;
			}
			*/
		}
	case IPV6_FLOW:
		if( a2_new_filter ) {
			if(fsp->m_u.tcp_ip6_spec.ip6src[0] ||
			   fsp->m_u.tcp_ip6_spec.ip6src[3] || 
			   fsp->m_u.tcp_ip6_spec.ip6dst[0] ||
			   fsp->m_u.tcp_ip6_spec.ip6dst[3]) {
				aq_rx_fltr->type |= aq_rx_filter_type_ipv6;
				err = aq_add_del_fl3(aq_nic, aq_rx_fltr, true, add);
			}
		} else {
			aq_rx_fltr->type |= aq_rx_filter_type_l3l4;
			if( !is_flex  )
				err = aq_add_del_fl3l4(aq_nic, aq_rx_fltr, true, add);
		}
		if( err )
			return err;
		break;
	default:
		err = -EINVAL;
	}

	if( hweight_long(aq_rx_fltr->type ) > 1 ) {
		aq_rx_fltr->type |= a2_new_filter ? aq_rx_filter_type_chain : aq_rx_filter_type_flex;
		err = aq_add_del_complex_filter(aq_nic, aq_rx_fltr, add);
	}
	//aq_nic_print(aq_nic, info, drv, "%s filter, type %x. Location: %x\n", 
	//						add ? "Add" : "Del", aq_rx_fltr->type, fsp->location);

	return err;
}

static int aq_remove_rule(struct aq_nic_s *aq_nic, struct aq_rx_filter *rule)
{
	int err = -EINVAL;
	struct aq_hw_rx_fltrs_s *rx_fltrs = aq_get_hw_rx_fltrs(aq_nic);

	if (rule) {
		err = aq_add_del_rule(aq_nic, rule, false);
		hlist_del(&rule->aq_node);
		kfree(rule);
		--rx_fltrs->active_filters;
	}
	return err;
}

int aq_del_fvlan_by_vlan(struct aq_nic_s *aq_nic, u16 vlan_id)
{
	struct aq_hw_rx_fltrs_s *rx_fltrs = aq_get_hw_rx_fltrs(aq_nic);
	struct aq_rx_filter *rule = NULL;
	struct hlist_node *aq_node2;

	hlist_for_each_entry_safe(rule, aq_node2,
				  &rx_fltrs->filter_list, aq_node) {
		if ((be16_to_cpu(rule->aq_fsp.h_ext.vlan_tci) & VLAN_PRIO_MASK) == vlan_id)
			break;
	}
	if (rule && be16_to_cpu(rule->aq_fsp.h_ext.vlan_tci) == vlan_id) {
		return aq_remove_rule(aq_nic, rule);
	}

	return -ENOENT;
}

static int aq_apply_rule(struct aq_nic_s *aq_nic,
			   struct aq_rx_filter *aq_rx_fltr, bool add)
{
	int err = -EINVAL;
	if( aq_rx_fltr->type & aq_rx_filter_type_vlan_pcp ) {
		err = aq_add_del_fvlan(aq_nic, aq_rx_fltr, add);
	}
	if( aq_rx_fltr->type & aq_rx_filter_type_vlan ) {
		err = aq_add_del_fvlan(aq_nic, aq_rx_fltr, add);
	}
	if( aq_rx_fltr->type & aq_rx_filter_type_flex ) {
		err = aq_add_del_complex_filter(aq_nic, aq_rx_fltr, add);
	} else if( aq_rx_fltr->type & aq_rx_filter_type_ethertype ) {
		err = aq_add_del_fether(aq_nic, aq_rx_fltr, add);
	} else if( aq_rx_fltr->type & aq_rx_filter_type_l3l4 ) {
		bool is_ipv6;
		switch( aq_rx_fltr->aq_fsp.flow_type & ~FLOW_EXT) {
		case TCP_V4_FLOW:
		case UDP_V4_FLOW:
		case SCTP_V4_FLOW:
		case IP_USER_FLOW:
		case IPV4_FLOW:
			is_ipv6 = false;
			break;
		case TCP_V6_FLOW:
		case UDP_V6_FLOW:
		case SCTP_V6_FLOW:
		case IPV6_FLOW:
			is_ipv6 = true;
			break;
		default:
			return -EINVAL;
		}
		err = aq_add_del_fl3l4(aq_nic, aq_rx_fltr, is_ipv6, add);
	}
	return err;

}

static int aq_update_table_filters(struct aq_nic_s *aq_nic,
				   struct aq_rx_filter *aq_rx_fltr, u16 index,
				   struct ethtool_rxnfc *cmd)
{
	int err = -EINVAL;
	u32 rule_loc = 0xff;
	struct hlist_node *aq_node2;
	struct aq_rx_filter *rule = NULL, *parent = NULL;
	struct aq_hw_rx_fltrs_s *rx_fltrs = aq_get_hw_rx_fltrs(aq_nic);

	hlist_for_each_entry_safe(rule, aq_node2,
				  &rx_fltrs->filter_list, aq_node) {
		rule_loc = rule->aq_fsp.location & AQ_RX_FILTER_CHAIN_MASK;
		if (rule_loc >= index ) {
			break;
		}
		parent = rule;
		rule = NULL;
	}

	if (rule && (rule_loc == index)) {
		err = aq_remove_rule(aq_nic, rule);
	}
	if (unlikely(!aq_rx_fltr))
		return err;

	INIT_HLIST_NODE(&aq_rx_fltr->aq_node);

	if (parent)
		hlist_add_behind(&aq_rx_fltr->aq_node, &parent->aq_node);
	else
		hlist_add_head(&aq_rx_fltr->aq_node, &rx_fltrs->filter_list);

	++rx_fltrs->active_filters;

	return 0;
}

u16 aq_get_rxnfc_count_all_rules(struct aq_nic_s *aq_nic)
{
	struct aq_hw_rx_fltrs_s *rx_fltrs = aq_get_hw_rx_fltrs(aq_nic);

	return rx_fltrs->active_filters;
}

struct aq_hw_rx_fltrs_s *aq_get_hw_rx_fltrs(struct aq_nic_s *aq_nic)
{
	return &aq_nic->aq_hw_rx_fltrs;
}

int aq_add_rxnfc_rule(struct aq_nic_s *aq_nic, const struct ethtool_rxnfc *cmd)
{
	int err = 0;
	struct aq_rx_filter *aq_rx_fltr;
	struct ethtool_rx_flow_spec *fsp =
		(struct ethtool_rx_flow_spec *)&cmd->fs;
	struct aq_hw_rx_fltrs_s *rx_fltrs = aq_get_hw_rx_fltrs(aq_nic);

	err = aq_check_rule(aq_nic, fsp);
	if (err)
		goto err_exit;

	aq_rx_fltr = kzalloc(sizeof(*aq_rx_fltr), GFP_KERNEL);
	if (unlikely(!aq_rx_fltr)) {
		err = -ENOMEM;
		goto err_exit;
	}

	memcpy(&aq_rx_fltr->aq_fsp, fsp, sizeof(*fsp));

	err = aq_update_table_filters(aq_nic, aq_rx_fltr, fsp->location & AQ_RX_FILTER_CHAIN_MASK, NULL);
	if (unlikely(err))
		goto err_free;
	aq_rx_fltr->aq_fsp.location |= AQ_RX_FILTER_CHAIN_CURRENT_MASK;
	err = aq_add_del_rule(aq_nic, aq_rx_fltr, true);
	aq_rx_fltr->aq_fsp.location &= ~AQ_RX_FILTER_CHAIN_CURRENT_MASK;
	if (unlikely(err)) {
		hlist_del(&aq_rx_fltr->aq_node);
		--rx_fltrs->active_filters;
		goto err_free;
	}
	fsp->location = aq_rx_fltr->aq_fsp.location;
	return 0;

err_free:
	kfree(aq_rx_fltr);
err_exit:
	return err;
}

int aq_del_rxnfc_rule(struct aq_nic_s *aq_nic, const struct ethtool_rxnfc *cmd)
{
	int err = -EINVAL;
	struct hlist_node *aq_node2;
	struct aq_rx_filter *rule = NULL;
	struct aq_hw_rx_fltrs_s *rx_fltrs = aq_get_hw_rx_fltrs(aq_nic);

	hlist_for_each_entry_safe(rule, aq_node2,
				  &rx_fltrs->filter_list, aq_node) {
		if (rule->aq_fsp.location == cmd->fs.location)
			break;
		rule = NULL;
	}

	err = aq_remove_rule(aq_nic, rule);
	return err;
}

int aq_get_rxnfc_rule(struct aq_nic_s *aq_nic, struct ethtool_rxnfc *cmd)
{
	struct hlist_node *aq_node2;
	struct aq_rx_filter *rule = NULL;
	struct ethtool_rx_flow_spec *fsp =
			(struct ethtool_rx_flow_spec *)&cmd->fs;
	struct aq_hw_rx_fltrs_s *rx_fltrs = aq_get_hw_rx_fltrs(aq_nic);

	hlist_for_each_entry_safe(rule, aq_node2,
				  &rx_fltrs->filter_list, aq_node) {
		if ((fsp->location & AQ_RX_FILTER_CHAIN_MASK) <= (rule->aq_fsp.location & AQ_RX_FILTER_CHAIN_MASK))
			break;
		rule = NULL;
	}
	if (unlikely(!rule))
		return -EINVAL;

	memcpy(fsp, &rule->aq_fsp, sizeof(*fsp));

	return 0;
}

int aq_get_rxnfc_all_rules(struct aq_nic_s *aq_nic, struct ethtool_rxnfc *cmd,
			    u32 *rule_locs)
{
	int count = 0;
	struct aq_rx_filter *rule;
	struct hlist_node *aq_node2;
	struct aq_hw_rx_fltrs_s *rx_fltrs = aq_get_hw_rx_fltrs(aq_nic);

	cmd->data = aq_get_rxnfc_count_all_rules(aq_nic);

	hlist_for_each_entry_safe(rule, aq_node2,
				  &rx_fltrs->filter_list, aq_node) {
		if (unlikely(count == cmd->rule_cnt))
			return -EMSGSIZE;

		rule_locs[count++] = rule->aq_fsp.location;
	}

	cmd->rule_cnt = count;

	return 0;
}

int aq_clear_rxnfc_all_rules(struct aq_nic_s *aq_nic)
{
	int err = 0;
	struct aq_rx_filter *rule;
	struct hlist_node *aq_node2;
	struct aq_hw_rx_fltrs_s *rx_fltrs = aq_get_hw_rx_fltrs(aq_nic);

	hlist_for_each_entry_safe(rule, aq_node2,
				  &rx_fltrs->filter_list, aq_node) {
		err = aq_remove_rule(aq_nic, rule);
		if (err)
			break;
	}

	return err;
}


int aq_apply_all_rule(struct aq_nic_s *aq_nic)
{
	int err = 0;
	struct aq_rx_filter *rule;
	struct hlist_node *aq_node2;
	struct aq_hw_rx_fltrs_s *rx_fltrs = aq_get_hw_rx_fltrs(aq_nic);

	hlist_for_each_entry_safe(rule, aq_node2,
				  &rx_fltrs->filter_list, aq_node) {
		err = aq_apply_rule(aq_nic, rule, true);
		if (err)
			goto err_exit;
	}

err_exit:
	return err;
}

int aq_filters_vlans_update(struct aq_nic_s *aq_nic)
{
	const struct aq_hw_ops *aq_hw_ops = aq_nic->aq_hw_ops;
	struct aq_hw_s *aq_hw = aq_nic->aq_hw;
	int hweight = 0;
	int err = 0;
	int i;

	if (unlikely(!aq_hw_ops->hw_filter_vlan_set))
		return -EOPNOTSUPP;
	if (unlikely(!aq_hw_ops->hw_filter_vlan_ctrl))
		return -EOPNOTSUPP;

	aq_fvlan_rebuild(aq_nic, aq_nic->active_vlans,
			 aq_nic->aq_hw_rx_fltrs.fl2.aq_vlans);

	for (i = 0; i < BITS_TO_LONGS(VLAN_N_VID); i++)
		hweight += hweight_long(aq_nic->active_vlans[i]);

	err = aq_hw_ops->hw_filter_vlan_ctrl(aq_hw, false);
	if (err)
		return err;
	err = aq_hw_ops->hw_filter_vlan_set(aq_hw,
					    aq_nic->aq_hw_rx_fltrs.fl2.aq_vlans
					   );
	if (err)
		return err;
	if (hweight <= AQ_RX_QUEUE_NOT_ASSIGNED && hweight > 0)
		err = aq_hw_ops->hw_filter_vlan_ctrl(aq_hw, true);
	/* otherwise left in promiscue mode */

	return err;
}

int aq_filters_vlans_on(struct aq_nic_s *aq_nic)
{
	int err = 0;

	memset(aq_nic->active_vlans, 0, sizeof(aq_nic->active_vlans));

	err = aq_filters_vlans_update(aq_nic);

	return err;
}

int aq_filters_vlans_off(struct aq_nic_s *aq_nic)
{
	struct aq_hw_rx_fltrs_s *rx_fltrs = aq_get_hw_rx_fltrs(aq_nic);
	const struct aq_hw_ops *aq_hw_ops = aq_nic->aq_hw_ops;
	struct aq_hw_s *aq_hw = aq_nic->aq_hw;
	struct hlist_node *aq_node2;
	struct aq_rx_filter *rule;
	int err = 0;

	hlist_for_each_entry_safe(rule, aq_node2,
				  &rx_fltrs->filter_list, aq_node) {
		if (rule->type == aq_rx_filter_type_vlan) {
			hlist_del(&rule->aq_node);
			kfree(rule);
			--rx_fltrs->active_filters;
		}
	}

	memset(aq_nic->active_vlans, 0, sizeof(aq_nic->active_vlans));
	memset(aq_nic->aq_hw_rx_fltrs.fl2.aq_vlans,
	       0,
	       sizeof(aq_nic->aq_hw_rx_fltrs.fl2.aq_vlans));

	if (unlikely(!aq_hw_ops->hw_filter_vlan_ctrl))
		return -EOPNOTSUPP;

	err = aq_filters_vlans_update(aq_nic);
	if (err)
		return err;

	return aq_hw_ops->hw_filter_vlan_ctrl(aq_hw, false);
}
