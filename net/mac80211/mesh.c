/*
 * Copyright (c) 2008, 2009 open80211s Ltd.
 * Authors:    Luis Carlos Cobo <luisca@cozybit.com>
 * 	       Javier Cardona <javier@cozybit.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <asm/unaligned.h>
#include "ieee80211_i.h"
#include "mesh.h"

#define MESHCONF_CAPAB_ACCEPT_PLINKS 0x01
#define MESHCONF_CAPAB_FORWARDING    0x08

#define TMR_RUNNING_HK	0
#define TMR_RUNNING_MP	1
#define TMR_RUNNING_MPR	2

int mesh_allocated;
static struct kmem_cache *rm_cache;

#ifdef CONFIG_MAC80211_MESH
bool mesh_action_is_path_sel(struct ieee80211_mgmt *mgmt)
{
	return (mgmt->u.action.u.mesh_action.action_code ==
			WLAN_MESH_ACTION_HWMP_PATH_SELECTION);
}
#else
bool mesh_action_is_path_sel(struct ieee80211_mgmt *mgmt)
{ return false; }
#endif

void ieee80211s_init(void)
{
	mesh_pathtbl_init();
	mesh_allocated = 1;
	rm_cache = kmem_cache_create("mesh_rmc", sizeof(struct rmc_entry),
				     0, 0, NULL);
}

void ieee80211s_stop(void)
{
	mesh_pathtbl_unregister();
	kmem_cache_destroy(rm_cache);
}

static void ieee80211_mesh_housekeeping_timer(unsigned long data)
{
	struct ieee80211_sub_if_data *sdata = (void *) data;
	struct ieee80211_local *local = sdata->local;
	struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;

	set_bit(MESH_WORK_HOUSEKEEPING, &ifmsh->wrkq_flags);

	if (local->quiescing) {
		set_bit(TMR_RUNNING_HK, &ifmsh->timers_running);
		return;
	}

	ieee80211_queue_work(&local->hw, &sdata->work);
}

/**
 * mesh_matches_local - check if the config of a mesh point matches ours
 *
 * @ie: information elements of a management frame from the mesh peer
 * @sdata: local mesh subif
 *
 * This function checks if the mesh configuration of a mesh point matches the
 * local mesh configuration, i.e. if both nodes belong to the same mesh network.
 */
bool mesh_matches_local(struct ieee802_11_elems *ie, struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;
	struct ieee80211_local *local = sdata->local;

	/*
	 * As support for each feature is added, check for matching
	 * - On mesh config capabilities
	 *   - Power Save Support En
	 *   - Sync support enabled
	 *   - Sync support active
	 *   - Sync support required from peer
	 *   - MDA enabled
	 * - Power management control on fc
	 */
	if (!(ifmsh->mesh_id_len == ie->mesh_id_len &&
	     memcmp(ifmsh->mesh_id, ie->mesh_id, ie->mesh_id_len) == 0 &&
	     (ifmsh->mesh_pp_id == ie->mesh_config->meshconf_psel) &&
	     (ifmsh->mesh_pm_id == ie->mesh_config->meshconf_pmetric) &&
	     (ifmsh->mesh_cc_id == ie->mesh_config->meshconf_congest) &&
	     (ifmsh->mesh_sp_id == ie->mesh_config->meshconf_synch) &&
	     (ifmsh->mesh_auth_id == ie->mesh_config->meshconf_auth)))
		goto mismatch;

	/* disallow peering with mismatched channel types for now */
	if (ie->ht_info_elem &&
	    (local->_oper_channel_type !=
	     ieee80211_ht_info_to_channel_type(ie->ht_info_elem)))
		goto mismatch;

	return true;
mismatch:
	return false;
}

/**
 * mesh_peer_accepts_plinks - check if an mp is willing to establish peer links
 *
 * @ie: information elements of a management frame from the mesh peer
 */
bool mesh_peer_accepts_plinks(struct ieee802_11_elems *ie)
{
	return (ie->mesh_config->meshconf_cap &
	    MESHCONF_CAPAB_ACCEPT_PLINKS) != 0;
}

/**
 * mesh_accept_plinks_update: update accepting_plink in local mesh beacons
 *
 * @sdata: mesh interface in which mesh beacons are going to be updated
 */
void mesh_accept_plinks_update(struct ieee80211_sub_if_data *sdata)
{
	bool free_plinks;

	/* In case mesh_plink_free_count > 0 and mesh_plinktbl_capacity == 0,
	 * the mesh interface might be able to establish plinks with peers that
	 * are already on the table but are not on PLINK_ESTAB state. However,
	 * in general the mesh interface is not accepting peer link requests
	 * from new peers, and that must be reflected in the beacon
	 */
	free_plinks = mesh_plink_availables(sdata);

	if (free_plinks != sdata->u.mesh.accepting_plinks)
		ieee80211_mesh_housekeeping_timer((unsigned long) sdata);
}

int mesh_rmc_init(struct ieee80211_sub_if_data *sdata)
{
	int i;

	sdata->u.mesh.rmc = kmalloc(sizeof(struct mesh_rmc), GFP_KERNEL);
	if (!sdata->u.mesh.rmc)
		return -ENOMEM;
	sdata->u.mesh.rmc->idx_mask = RMC_BUCKETS - 1;
	for (i = 0; i < RMC_BUCKETS; i++)
		INIT_LIST_HEAD(&sdata->u.mesh.rmc->bucket[i].list);
	return 0;
}

void mesh_rmc_free(struct ieee80211_sub_if_data *sdata)
{
	struct mesh_rmc *rmc = sdata->u.mesh.rmc;
	struct rmc_entry *p, *n;
	int i;

	if (!sdata->u.mesh.rmc)
		return;

	for (i = 0; i < RMC_BUCKETS; i++)
		list_for_each_entry_safe(p, n, &rmc->bucket[i].list, list) {
			list_del(&p->list);
			kmem_cache_free(rm_cache, p);
		}

	kfree(rmc);
	sdata->u.mesh.rmc = NULL;
}

/**
 * mesh_rmc_check - Check frame in recent multicast cache and add if absent.
 *
 * @sa:		source address
 * @mesh_hdr:	mesh_header
 *
 * Returns: 0 if the frame is not in the cache, nonzero otherwise.
 *
 * Checks using the source address and the mesh sequence number if we have
 * received this frame lately. If the frame is not in the cache, it is added to
 * it.
 */
int mesh_rmc_check(u8 *sa, struct ieee80211s_hdr *mesh_hdr,
		   struct ieee80211_sub_if_data *sdata)
{
	struct mesh_rmc *rmc = sdata->u.mesh.rmc;
	u32 seqnum = 0;
	int entries = 0;
	u8 idx;
	struct rmc_entry *p, *n;

	/* Don't care about endianness since only match matters */
	memcpy(&seqnum, &mesh_hdr->seqnum, sizeof(mesh_hdr->seqnum));
	idx = le32_to_cpu(mesh_hdr->seqnum) & rmc->idx_mask;
	list_for_each_entry_safe(p, n, &rmc->bucket[idx].list, list) {
		++entries;
		if (time_after(jiffies, p->exp_time) ||
				(entries == RMC_QUEUE_MAX_LEN)) {
			list_del(&p->list);
			kmem_cache_free(rm_cache, p);
			--entries;
		} else if ((seqnum == p->seqnum) &&
			   (compare_ether_addr(sa, p->sa) == 0))
			return -1;
	}

	p = kmem_cache_alloc(rm_cache, GFP_ATOMIC);
	if (!p)
		return 0;

	p->seqnum = seqnum;
	p->exp_time = jiffies + RMC_TIMEOUT;
	memcpy(p->sa, sa, ETH_ALEN);
	list_add(&p->list, &rmc->bucket[idx].list);
	return 0;
}

int
mesh_add_meshconf_ie(struct sk_buff *skb, struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;
	u8 *pos, neighbors;
	u8 meshconf_len = sizeof(struct ieee80211_meshconf_ie);

	if (skb_tailroom(skb) < 2 + meshconf_len)
		return -ENOMEM;

	pos = skb_put(skb, 2 + meshconf_len);
	*pos++ = WLAN_EID_MESH_CONFIG;
	*pos++ = meshconf_len;

	/* Active path selection protocol ID */
	*pos++ = ifmsh->mesh_pp_id;
	/* Active path selection metric ID   */
	*pos++ = ifmsh->mesh_pm_id;
	/* Congestion control mode identifier */
	*pos++ = ifmsh->mesh_cc_id;
	/* Synchronization protocol identifier */
	*pos++ = ifmsh->mesh_sp_id;
	/* Authentication Protocol identifier */
	*pos++ = ifmsh->mesh_auth_id;
	/* Mesh Formation Info - number of neighbors */
	neighbors = atomic_read(&ifmsh->mshstats.estab_plinks);
	/* Number of neighbor mesh STAs or 15 whichever is smaller */
	neighbors = (neighbors > 15) ? 15 : neighbors;
	*pos++ = neighbors << 1;
	/* Mesh capability */
	ifmsh->accepting_plinks = mesh_plink_availables(sdata);
	*pos = MESHCONF_CAPAB_FORWARDING;
	*pos++ |= ifmsh->accepting_plinks ?
	    MESHCONF_CAPAB_ACCEPT_PLINKS : 0x00;
	*pos++ = 0x00;

	return 0;
}

int
mesh_add_meshid_ie(struct sk_buff *skb, struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;
	u8 *pos;

	if (skb_tailroom(skb) < 2 + ifmsh->mesh_id_len)
		return -ENOMEM;

	pos = skb_put(skb, 2 + ifmsh->mesh_id_len);
	*pos++ = WLAN_EID_MESH_ID;
	*pos++ = ifmsh->mesh_id_len;
	if (ifmsh->mesh_id_len)
		memcpy(pos, ifmsh->mesh_id, ifmsh->mesh_id_len);

	return 0;
}

int
mesh_add_vendor_ies(struct sk_buff *skb, struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;
	u8 offset, len;
	const u8 *data;

	if (!ifmsh->ie || !ifmsh->ie_len)
		return 0;

	/* fast-forward to vendor IEs */
	offset = ieee80211_ie_split_vendor(ifmsh->ie, ifmsh->ie_len, 0);

	if (offset) {
		len = ifmsh->ie_len - offset;
		data = ifmsh->ie + offset;
		if (skb_tailroom(skb) < len)
			return -ENOMEM;
		memcpy(skb_put(skb, len), data, len);
	}

	return 0;
}

int
mesh_add_rsn_ie(struct sk_buff *skb, struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;
	u8 len = 0;
	const u8 *data;

	if (!ifmsh->ie || !ifmsh->ie_len)
		return 0;

	/* find RSN IE */
	data = ifmsh->ie;
	while (data < ifmsh->ie + ifmsh->ie_len) {
		if (*data == WLAN_EID_RSN) {
			len = data[1] + 2;
			break;
		}
		data++;
	}

	if (len) {
		if (skb_tailroom(skb) < len)
			return -ENOMEM;
		memcpy(skb_put(skb, len), data, len);
	}

	return 0;
}

int mesh_add_ds_params_ie(struct sk_buff *skb,
			  struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_local *local = sdata->local;
	struct ieee80211_supported_band *sband;
	u8 *pos;

	if (skb_tailroom(skb) < 3)
		return -ENOMEM;

	sband = local->hw.wiphy->bands[local->hw.conf.channel->band];
	if (sband->band == IEEE80211_BAND_2GHZ) {
		pos = skb_put(skb, 2 + 1);
		*pos++ = WLAN_EID_DS_PARAMS;
		*pos++ = 1;
		*pos++ = ieee80211_frequency_to_channel(local->hw.conf.channel->center_freq);
	}

	return 0;
}

int mesh_add_ht_cap_ie(struct sk_buff *skb,
		       struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_local *local = sdata->local;
	struct ieee80211_supported_band *sband;
	u8 *pos;

	sband = local->hw.wiphy->bands[local->oper_channel->band];
	if (!sband->ht_cap.ht_supported ||
	    local->_oper_channel_type == NL80211_CHAN_NO_HT)
		return 0;

	if (skb_tailroom(skb) < 2 + sizeof(struct ieee80211_ht_cap))
		return -ENOMEM;

	pos = skb_put(skb, 2 + sizeof(struct ieee80211_ht_cap));
	ieee80211_ie_build_ht_cap(pos, &sband->ht_cap, sband->ht_cap.cap);

	return 0;
}

int mesh_add_ht_info_ie(struct sk_buff *skb,
			struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_local *local = sdata->local;
	struct ieee80211_channel *channel = local->oper_channel;
	enum nl80211_channel_type channel_type = local->_oper_channel_type;
	struct ieee80211_supported_band *sband =
				local->hw.wiphy->bands[channel->band];
	struct ieee80211_sta_ht_cap *ht_cap = &sband->ht_cap;
	u8 *pos;

	if (!ht_cap->ht_supported || channel_type == NL80211_CHAN_NO_HT)
		return 0;

	if (skb_tailroom(skb) < 2 + sizeof(struct ieee80211_ht_info))
		return -ENOMEM;

	pos = skb_put(skb, 2 + sizeof(struct ieee80211_ht_info));
	ieee80211_ie_build_ht_info(pos, ht_cap, channel, channel_type);

	return 0;
}
static void ieee80211_mesh_path_timer(unsigned long data)
{
	struct ieee80211_sub_if_data *sdata =
		(struct ieee80211_sub_if_data *) data;
	struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;
	struct ieee80211_local *local = sdata->local;

	if (local->quiescing) {
		set_bit(TMR_RUNNING_MP, &ifmsh->timers_running);
		return;
	}

	ieee80211_queue_work(&local->hw, &sdata->work);
}

static void ieee80211_mesh_path_root_timer(unsigned long data)
{
	struct ieee80211_sub_if_data *sdata =
		(struct ieee80211_sub_if_data *) data;
	struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;
	struct ieee80211_local *local = sdata->local;

	set_bit(MESH_WORK_ROOT, &ifmsh->wrkq_flags);

	if (local->quiescing) {
		set_bit(TMR_RUNNING_MPR, &ifmsh->timers_running);
		return;
	}

	ieee80211_queue_work(&local->hw, &sdata->work);
}

void ieee80211_mesh_root_setup(struct ieee80211_if_mesh *ifmsh)
{
	if (ifmsh->mshcfg.dot11MeshHWMPRootMode)
		set_bit(MESH_WORK_ROOT, &ifmsh->wrkq_flags);
	else {
		clear_bit(MESH_WORK_ROOT, &ifmsh->wrkq_flags);
		/* stop running timer */
		del_timer_sync(&ifmsh->mesh_path_root_timer);
	}
}

/**
 * ieee80211_fill_mesh_addresses - fill addresses of a locally originated mesh frame
 * @hdr:    	802.11 frame header
 * @fc:		frame control field
 * @meshda:	destination address in the mesh
 * @meshsa:	source address address in the mesh.  Same as TA, as frame is
 *              locally originated.
 *
 * Return the length of the 802.11 (does not include a mesh control header)
 */
int ieee80211_fill_mesh_addresses(struct ieee80211_hdr *hdr, __le16 *fc,
				  const u8 *meshda, const u8 *meshsa)
{
	if (is_multicast_ether_addr(meshda)) {
		*fc |= cpu_to_le16(IEEE80211_FCTL_FROMDS);
		/* DA TA SA */
		memcpy(hdr->addr1, meshda, ETH_ALEN);
		memcpy(hdr->addr2, meshsa, ETH_ALEN);
		memcpy(hdr->addr3, meshsa, ETH_ALEN);
		return 24;
	} else {
		*fc |= cpu_to_le16(IEEE80211_FCTL_FROMDS | IEEE80211_FCTL_TODS);
		/* RA TA DA SA */
		memset(hdr->addr1, 0, ETH_ALEN);   /* RA is resolved later */
		memcpy(hdr->addr2, meshsa, ETH_ALEN);
		memcpy(hdr->addr3, meshda, ETH_ALEN);
		memcpy(hdr->addr4, meshsa, ETH_ALEN);
		return 30;
	}
}

/**
 * ieee80211_new_mesh_header - create a new mesh header
 * @meshhdr:    uninitialized mesh header
 * @sdata:	mesh interface to be used
 * @addr4or5:   1st address in the ae header, which may correspond to address 4
 *              (if addr6 is NULL) or address 5 (if addr6 is present). It may
 *              be NULL.
 * @addr6:	2nd address in the ae header, which corresponds to addr6 of the
 *              mesh frame
 *
 * Return the header length.
 */
int ieee80211_new_mesh_header(struct ieee80211s_hdr *meshhdr,
		struct ieee80211_sub_if_data *sdata, char *addr4or5,
		char *addr6)
{
	int aelen = 0;
	BUG_ON(!addr4or5 && addr6);
	memset(meshhdr, 0, sizeof(*meshhdr));
	meshhdr->ttl = sdata->u.mesh.mshcfg.dot11MeshTTL;
	put_unaligned(cpu_to_le32(sdata->u.mesh.mesh_seqnum), &meshhdr->seqnum);
	sdata->u.mesh.mesh_seqnum++;
	if (addr4or5 && !addr6) {
		meshhdr->flags |= MESH_FLAGS_AE_A4;
		aelen += ETH_ALEN;
		memcpy(meshhdr->eaddr1, addr4or5, ETH_ALEN);
	} else if (addr4or5 && addr6) {
		meshhdr->flags |= MESH_FLAGS_AE_A5_A6;
		aelen += 2 * ETH_ALEN;
		memcpy(meshhdr->eaddr1, addr4or5, ETH_ALEN);
		memcpy(meshhdr->eaddr2, addr6, ETH_ALEN);
	}
	return 6 + aelen;
}

static void ieee80211_mesh_housekeeping(struct ieee80211_sub_if_data *sdata,
			   struct ieee80211_if_mesh *ifmsh)
{
	bool free_plinks;

#ifdef CONFIG_MAC80211_VERBOSE_DEBUG
	printk(KERN_DEBUG "%s: running mesh housekeeping\n",
	       sdata->name);
#endif

	ieee80211_sta_expire(sdata, IEEE80211_MESH_PEER_INACTIVITY_LIMIT);
	mesh_path_expire(sdata);

	free_plinks = mesh_plink_availables(sdata);
	if (free_plinks != sdata->u.mesh.accepting_plinks)
		ieee80211_bss_info_change_notify(sdata, BSS_CHANGED_BEACON);

	mod_timer(&ifmsh->housekeeping_timer,
		  round_jiffies(jiffies + IEEE80211_MESH_HOUSEKEEPING_INTERVAL));
}

static void ieee80211_mesh_rootpath(struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;

	mesh_path_tx_root_frame(sdata);
	mod_timer(&ifmsh->mesh_path_root_timer,
		  round_jiffies(TU_TO_EXP_TIME(
				  ifmsh->mshcfg.dot11MeshHWMPRannInterval)));
}

#ifdef CONFIG_PM
void ieee80211_mesh_quiesce(struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;

	/* use atomic bitops in case all timers fire at the same time */

	if (del_timer_sync(&ifmsh->housekeeping_timer))
		set_bit(TMR_RUNNING_HK, &ifmsh->timers_running);
	if (del_timer_sync(&ifmsh->mesh_path_timer))
		set_bit(TMR_RUNNING_MP, &ifmsh->timers_running);
	if (del_timer_sync(&ifmsh->mesh_path_root_timer))
		set_bit(TMR_RUNNING_MPR, &ifmsh->timers_running);
}

void ieee80211_mesh_restart(struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;

	if (test_and_clear_bit(TMR_RUNNING_HK, &ifmsh->timers_running))
		add_timer(&ifmsh->housekeeping_timer);
	if (test_and_clear_bit(TMR_RUNNING_MP, &ifmsh->timers_running))
		add_timer(&ifmsh->mesh_path_timer);
	if (test_and_clear_bit(TMR_RUNNING_MPR, &ifmsh->timers_running))
		add_timer(&ifmsh->mesh_path_root_timer);
	ieee80211_mesh_root_setup(ifmsh);
}
#endif

void ieee80211_start_mesh(struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;
	struct ieee80211_local *local = sdata->local;

	local->fif_other_bss++;
	/* mesh ifaces must set allmulti to forward mcast traffic */
	atomic_inc(&local->iff_allmultis);
	ieee80211_configure_filter(local);

	ifmsh->mesh_cc_id = 0;	/* Disabled */
	ifmsh->mesh_sp_id = 0;	/* Neighbor Offset */
	ifmsh->mesh_auth_id = 0;	/* Disabled */
	set_bit(MESH_WORK_HOUSEKEEPING, &ifmsh->wrkq_flags);
	ieee80211_mesh_root_setup(ifmsh);
	ieee80211_queue_work(&local->hw, &sdata->work);
	sdata->vif.bss_conf.beacon_int = MESH_DEFAULT_BEACON_INTERVAL;
	ieee80211_bss_info_change_notify(sdata, BSS_CHANGED_BEACON |
						BSS_CHANGED_BEACON_ENABLED |
						BSS_CHANGED_BEACON_INT);
}

void ieee80211_stop_mesh(struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_local *local = sdata->local;
	struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;

	ifmsh->mesh_id_len = 0;
	ieee80211_bss_info_change_notify(sdata, BSS_CHANGED_BEACON_ENABLED);
	sta_info_flush(local, NULL);

	del_timer_sync(&sdata->u.mesh.housekeeping_timer);
	del_timer_sync(&sdata->u.mesh.mesh_path_root_timer);
	/*
	 * If the timer fired while we waited for it, it will have
	 * requeued the work. Now the work will be running again
	 * but will not rearm the timer again because it checks
	 * whether the interface is running, which, at this point,
	 * it no longer is.
	 */
	cancel_work_sync(&sdata->work);

	local->fif_other_bss--;
	atomic_dec(&local->iff_allmultis);
	ieee80211_configure_filter(local);
}

static void ieee80211_mesh_rx_bcn_presp(struct ieee80211_sub_if_data *sdata,
					u16 stype,
					struct ieee80211_mgmt *mgmt,
					size_t len,
					struct ieee80211_rx_status *rx_status)
{
	struct ieee80211_local *local = sdata->local;
	struct ieee802_11_elems elems;
	struct ieee80211_channel *channel;
	u32 supp_rates = 0;
	size_t baselen;
	int freq;
	enum ieee80211_band band = rx_status->band;

	/* ignore ProbeResp to foreign address */
	if (stype == IEEE80211_STYPE_PROBE_RESP &&
	    compare_ether_addr(mgmt->da, sdata->vif.addr))
		return;

	baselen = (u8 *) mgmt->u.probe_resp.variable - (u8 *) mgmt;
	if (baselen > len)
		return;

	ieee802_11_parse_elems(mgmt->u.probe_resp.variable, len - baselen,
			       &elems);

	/* ignore beacons from secure mesh peers if our security is off */
	if (elems.rsn_len && sdata->u.mesh.security == IEEE80211_MESH_SEC_NONE)
		return;

	if (elems.ds_params && elems.ds_params_len == 1)
		freq = ieee80211_channel_to_frequency(elems.ds_params[0], band);
	else
		freq = rx_status->freq;

	channel = ieee80211_get_channel(local->hw.wiphy, freq);

	if (!channel || channel->flags & IEEE80211_CHAN_DISABLED)
		return;

	if (elems.mesh_id && elems.mesh_config &&
	    mesh_matches_local(&elems, sdata)) {
		supp_rates = ieee80211_sta_get_rates(local, &elems, band);
		mesh_neighbour_update(mgmt->sa, supp_rates, sdata, &elems);
	}
}

static void ieee80211_mesh_rx_mgmt_action(struct ieee80211_sub_if_data *sdata,
					  struct ieee80211_mgmt *mgmt,
					  size_t len,
					  struct ieee80211_rx_status *rx_status)
{
	switch (mgmt->u.action.category) {
	case WLAN_CATEGORY_SELF_PROTECTED:
		switch (mgmt->u.action.u.self_prot.action_code) {
		case WLAN_SP_MESH_PEERING_OPEN:
		case WLAN_SP_MESH_PEERING_CLOSE:
		case WLAN_SP_MESH_PEERING_CONFIRM:
			mesh_rx_plink_frame(sdata, mgmt, len, rx_status);
			break;
		}
		break;
	case WLAN_CATEGORY_MESH_ACTION:
		if (mesh_action_is_path_sel(mgmt))
			mesh_rx_path_sel_frame(sdata, mgmt, len);
		break;
	}
}

void ieee80211_mesh_rx_queued_mgmt(struct ieee80211_sub_if_data *sdata,
				   struct sk_buff *skb)
{
	struct ieee80211_rx_status *rx_status;
	struct ieee80211_mgmt *mgmt;
	u16 stype;

	rx_status = IEEE80211_SKB_RXCB(skb);
	mgmt = (struct ieee80211_mgmt *) skb->data;
	stype = le16_to_cpu(mgmt->frame_control) & IEEE80211_FCTL_STYPE;

	switch (stype) {
	case IEEE80211_STYPE_PROBE_RESP:
	case IEEE80211_STYPE_BEACON:
		ieee80211_mesh_rx_bcn_presp(sdata, stype, mgmt, skb->len,
					    rx_status);
		break;
	case IEEE80211_STYPE_ACTION:
		ieee80211_mesh_rx_mgmt_action(sdata, mgmt, skb->len, rx_status);
		break;
	}
}

void ieee80211_mesh_work(struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;

	if (ifmsh->preq_queue_len &&
	    time_after(jiffies,
		       ifmsh->last_preq + msecs_to_jiffies(ifmsh->mshcfg.dot11MeshHWMPpreqMinInterval)))
		mesh_path_start_discovery(sdata);

	if (test_and_clear_bit(MESH_WORK_GROW_MPATH_TABLE, &ifmsh->wrkq_flags))
		mesh_mpath_table_grow();

	if (test_and_clear_bit(MESH_WORK_GROW_MPP_TABLE, &ifmsh->wrkq_flags))
		mesh_mpp_table_grow();

	if (test_and_clear_bit(MESH_WORK_HOUSEKEEPING, &ifmsh->wrkq_flags))
		ieee80211_mesh_housekeeping(sdata, ifmsh);

	if (test_and_clear_bit(MESH_WORK_ROOT, &ifmsh->wrkq_flags))
		ieee80211_mesh_rootpath(sdata);
}

void ieee80211_mesh_notify_scan_completed(struct ieee80211_local *local)
{
	struct ieee80211_sub_if_data *sdata;

	rcu_read_lock();
	list_for_each_entry_rcu(sdata, &local->interfaces, list)
		if (ieee80211_vif_is_mesh(&sdata->vif))
			ieee80211_queue_work(&local->hw, &sdata->work);
	rcu_read_unlock();
}

void ieee80211_mesh_init_sdata(struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;

	setup_timer(&ifmsh->housekeeping_timer,
		    ieee80211_mesh_housekeeping_timer,
		    (unsigned long) sdata);

	ifmsh->accepting_plinks = true;
	ifmsh->preq_id = 0;
	ifmsh->sn = 0;
	ifmsh->num_gates = 0;
	atomic_set(&ifmsh->mpaths, 0);
	mesh_rmc_init(sdata);
	ifmsh->last_preq = jiffies;
	ifmsh->next_perr = jiffies;
	/* Allocate all mesh structures when creating the first mesh interface. */
	if (!mesh_allocated)
		ieee80211s_init();
	setup_timer(&ifmsh->mesh_path_timer,
		    ieee80211_mesh_path_timer,
		    (unsigned long) sdata);
	setup_timer(&ifmsh->mesh_path_root_timer,
		    ieee80211_mesh_path_root_timer,
		    (unsigned long) sdata);
	INIT_LIST_HEAD(&ifmsh->preq_queue.list);
	spin_lock_init(&ifmsh->mesh_preq_queue_lock);
}
