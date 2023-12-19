/*
 * ChromeOS backport definitions
 * Copyright (C) 2015-2017 Intel Deutschland GmbH
 * Copyright (C) 2018-2023 Intel Corporation
 */
#include <linux/if_ether.h>
#include <net/cfg80211.h>
#include <linux/errqueue.h>
#include <generated/utsrelease.h>
/* ipv6_addr_is_multicast moved - include old header */
#include <net/addrconf.h>
#include <net/ieee80211_radiotap.h>
#include <crypto/hash.h>
#include <net/dsfield.h>

/* make sure we include iw_handler.h to get wireless_nlevent_flush() */
#include <net/iw_handler.h>

/* common backward compat code */

#include "version.h"

/* Dummy RHEL macros */
#define RHEL_RELEASE_CODE 0
#define RHEL_RELEASE_VERSION(a,b) 1

/* backport artifacts */
#define netdev_tstats(dev)	dev->tstats
#define netdev_assign_tstats(dev, e)	dev->tstats = (e);

static inline unsigned int
csa_n_counter_offsets_beacon(struct cfg80211_csa_settings *s)
{
	return s->n_counter_offsets_beacon;
}

static inline unsigned int
csa_n_counter_offsets_presp(struct cfg80211_csa_settings *s)
{
	return s->n_counter_offsets_presp;
}

static inline const u16 *
csa_counter_offsets_beacon(struct cfg80211_csa_settings *s)
{
	return s->counter_offsets_beacon;
}

static inline const u16 *
csa_counter_offsets_presp(struct cfg80211_csa_settings *s)
{
	return s->counter_offsets_presp;
}

#if CFG80211_VERSION <= KERNEL_VERSION(6,8,0)
bool
ieee80211_uhb_power_type_valid(struct ieee80211_mgmt *mgmt, size_t len,
			       struct ieee80211_channel *channel);

#define IEEE80211_CHAN_NO_UHB_VLP_CLIENT BIT(21)
#define IEEE80211_CHAN_NO_UHB_AFC_CLIENT BIT(22)

#define NL80211_RRF_NO_UHB_VLP_CLIENT BIT(22)
#define NL80211_RRF_NO_UHB_AFC_CLIENT BIT(23)
#endif

#if CFG80211_VERSION < KERNEL_VERSION(5,18,0)
#define IEEE80211_CHAN_NO_HE 0
#define IEEE80211_CHAN_NO_EHT 0

#define NL80211_RRF_NO_HE 0

#define NL80211_CHAN_WIDTH_320 13

#define IEEE80211_EHT_PPE_THRES_MAX_LEN	32

struct ieee80211_eht_mcs_nss_supp {
	union {
		struct ieee80211_eht_mcs_nss_supp_20mhz_only only_20mhz;
		struct {
			struct ieee80211_eht_mcs_nss_supp_bw _80;
			struct ieee80211_eht_mcs_nss_supp_bw _160;
			struct ieee80211_eht_mcs_nss_supp_bw _320;
		} __packed bw;
	} __packed;
} __packed;

struct ieee80211_sta_eht_cap {
	bool has_eht;
	struct ieee80211_eht_cap_elem_fixed eht_cap_elem;
	struct ieee80211_eht_mcs_nss_supp eht_mcs_nss_supp;
	u8 eht_ppe_thres[IEEE80211_EHT_PPE_THRES_MAX_LEN];
};

static inline const struct ieee80211_sta_eht_cap *
ieee80211_get_eht_iftype_cap(const struct ieee80211_supported_band *sband,
			     enum nl80211_iftype iftype)
{
	return NULL;
}

#define cfg_eht_cap_has_eht(obj) (false && (obj))
#define cfg_eht_cap_set_has_eht(obj, val) do { (void)obj; (void)val; } while (0)
#define cfg_eht_cap(obj) ((struct ieee80211_sta_eht_cap *)((obj) ? NULL : NULL))

/* mbssid was added in 5.18.0, so it's safe to return 0 prior to that version */
#define ieee80211_get_mbssid_beacon_len(...) 0
#define ieee80211_beacon_get_ap_ema_list(...) 0
#else
#define cfg_eht_cap_has_eht(obj) (obj)->eht_cap.has_eht
#define cfg_eht_cap_set_has_eht(obj, val) (obj)->eht_cap.has_eht = val
#define cfg_eht_cap(obj) (&(obj)->eht_cap)
#endif

/* backport wiphy_ext_feature_set/_isset
 *
 * To do so, define our own versions thereof that check for a negative
 * feature index and in that case ignore it entirely. That allows us to
 * define the ones that the cfg80211 version doesn't support to -1.
 */
static inline void iwl7000_wiphy_ext_feature_set(struct wiphy *wiphy, int ftidx)
{
	if (ftidx < 0)
		return;
	wiphy_ext_feature_set(wiphy, ftidx);
}

static inline bool iwl7000_wiphy_ext_feature_isset(struct wiphy *wiphy,
						   int ftidx)
{
	if (ftidx < 0)
		return false;
	return wiphy_ext_feature_isset(wiphy, ftidx);
}
#define wiphy_ext_feature_set iwl7000_wiphy_ext_feature_set
#define wiphy_ext_feature_isset iwl7000_wiphy_ext_feature_isset

struct backport_sinfo {
	u32 filled;
	u32 connected_time;
	u32 inactive_time;
	u64 rx_bytes;
	u64 tx_bytes;
	u16 llid;
	u16 plid;
	u8 plink_state;
	s8 signal;
	s8 signal_avg;

	u8 chains;
	s8 chain_signal[IEEE80211_MAX_CHAINS];
	s8 chain_signal_avg[IEEE80211_MAX_CHAINS];

	struct rate_info txrate;
	struct rate_info rxrate;
	u32 rx_packets;
	u32 tx_packets;
	u32 tx_retries;
	u32 tx_failed;
	u32 rx_dropped_misc;
	struct sta_bss_parameters bss_param;
	struct nl80211_sta_flag_update sta_flags;

	int generation;

	const u8 *assoc_req_ies;
	size_t assoc_req_ies_len;

	u32 beacon_loss_count;
	s64 t_offset;
	enum nl80211_mesh_power_mode local_pm;
	enum nl80211_mesh_power_mode peer_pm;
	enum nl80211_mesh_power_mode nonpeer_pm;

	u32 expected_throughput;

	u64 tx_duration;
	u64 rx_duration;
	u64 rx_beacon;
	u8 rx_beacon_signal_avg;
#if CFG80211_VERSION < KERNEL_VERSION(4,18,0)
	/*
	 * With < 4.18 we use an array here, like before, so we don't
	 * need to alloc/free it
	 */
	struct cfg80211_tid_stats pertid[IEEE80211_NUM_TIDS + 1];
#else
	struct cfg80211_tid_stats *pertid;
#endif
	s8 ack_signal;
	s8 avg_ack_signal;

	u16 airtime_weight;

	u32 rx_mpdu_count;
	u32 fcs_err_count;

	u32 airtime_link_metric;

	u64 assoc_at;
};

/* these are constants in nl80211.h, so it's
 * harmless to define them unconditionally
 */
#define NL80211_STA_INFO_RX_DROP_MISC		28
#define NL80211_STA_INFO_BEACON_RX		29
#define NL80211_STA_INFO_BEACON_SIGNAL_AVG	30
#define NL80211_STA_INFO_TID_STATS		31
#define NL80211_TID_STATS_RX_MSDU		1
#define NL80211_TID_STATS_TX_MSDU		2
#define NL80211_TID_STATS_TX_MSDU_RETRIES	3
#define NL80211_TID_STATS_TX_MSDU_FAILED	4

static inline void iwl7000_convert_sinfo(struct backport_sinfo *bpsinfo,
					 struct station_info *sinfo)
{
	memset(sinfo, 0, sizeof(*sinfo));
#define COPY(x)	sinfo->x = bpsinfo->x
#define MCPY(x)	memcpy(&sinfo->x, &bpsinfo->x, sizeof(sinfo->x))
	COPY(connected_time);
	COPY(inactive_time);
	COPY(rx_bytes);
	COPY(tx_bytes);
	COPY(llid);
	COPY(plid);
	COPY(plink_state);
	COPY(signal);
	COPY(signal_avg);
	COPY(chains);
	MCPY(chain_signal);
	MCPY(chain_signal_avg);
	COPY(txrate);
	COPY(rxrate);
	COPY(rx_packets);
	COPY(tx_packets);
	COPY(tx_retries);
	COPY(tx_failed);
	COPY(rx_dropped_misc);
	COPY(bss_param);
	COPY(sta_flags);
	COPY(generation);
	COPY(assoc_req_ies);
	COPY(assoc_req_ies_len);
	COPY(beacon_loss_count);
	COPY(t_offset);
	COPY(local_pm);
	COPY(peer_pm);
	COPY(nonpeer_pm);
	COPY(expected_throughput);
#if CFG80211_VERSION >= KERNEL_VERSION(4,18,0)
	COPY(ack_signal);
	COPY(avg_ack_signal);
	COPY(rx_duration);
#if CFG80211_VERSION >= KERNEL_VERSION(4,20,0)
	COPY(rx_mpdu_count);
	COPY(fcs_err_count);
#endif
#endif
#if CFG80211_VERSION >= KERNEL_VERSION(5,1,0)
	COPY(tx_duration);
	COPY(airtime_weight);
#endif
#if CFG80211_VERSION >= KERNEL_VERSION(5,2,0)
	COPY(airtime_link_metric);
#endif
	COPY(rx_beacon);
	COPY(rx_beacon_signal_avg);
	MCPY(pertid);
	COPY(filled);
#undef COPY
}
typedef struct station_info cfg_station_info_t;
#define station_info backport_sinfo

static inline void
backport_cfg80211_new_sta(struct net_device *dev, const u8 *mac_addr,
			  struct station_info *sinfo, gfp_t gfp)
{
	cfg_station_info_t cfg_info;

	iwl7000_convert_sinfo(sinfo, &cfg_info);
	cfg80211_new_sta(dev, mac_addr, &cfg_info, gfp);
}
#define cfg80211_new_sta backport_cfg80211_new_sta

static inline void
backport_cfg80211_del_sta_sinfo(struct net_device *dev, const u8 *mac_addr,
				struct station_info *sinfo, gfp_t gfp)
{
	cfg_station_info_t cfg_info;

	iwl7000_convert_sinfo(sinfo, &cfg_info);
	cfg80211_del_sta_sinfo(dev, mac_addr, &cfg_info, gfp);
}
#define cfg80211_del_sta_sinfo backport_cfg80211_del_sta_sinfo

typedef struct survey_info cfg_survey_info_t;

static inline void iwl7000_convert_survey_info(struct survey_info *survey,
					       cfg_survey_info_t *cfg)
{
	memcpy(cfg, survey, sizeof(*cfg));
}


#if CFG80211_VERSION < KERNEL_VERSION(4,20,0)
#define beacon_ftm_len(beacon, m) 0
#else
#define beacon_ftm_len(beacon, m) ((beacon)->m)
#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,1,0)
#define ASSOC_REQ_DISABLE_EHT BIT(5)
#define NL80211_EXT_FEATURE_POWERED_ADDR_CHANGE -1
#endif /* CFG80211_VERSION < KERNEL_VERSION(6,1,0) */

#if CFG80211_VERSION < KERNEL_VERSION(4,19,0)
#define IEEE80211_HE_PPE_THRES_MAX_LEN		25
#define RATE_INFO_FLAGS_HE_MCS BIT(4)

/**
 * enum nl80211_he_gi - HE guard interval
 * @NL80211_RATE_INFO_HE_GI_0_8: 0.8 usec
 * @NL80211_RATE_INFO_HE_GI_1_6: 1.6 usec
 * @NL80211_RATE_INFO_HE_GI_3_2: 3.2 usec
 */
enum nl80211_he_gi {
	NL80211_RATE_INFO_HE_GI_0_8,
	NL80211_RATE_INFO_HE_GI_1_6,
	NL80211_RATE_INFO_HE_GI_3_2,
};

/**
 * @enum nl80211_he_ru_alloc - HE RU allocation values
 * @NL80211_RATE_INFO_HE_RU_ALLOC_26: 26-tone RU allocation
 * @NL80211_RATE_INFO_HE_RU_ALLOC_52: 52-tone RU allocation
 * @NL80211_RATE_INFO_HE_RU_ALLOC_106: 106-tone RU allocation
 * @NL80211_RATE_INFO_HE_RU_ALLOC_242: 242-tone RU allocation
 * @NL80211_RATE_INFO_HE_RU_ALLOC_484: 484-tone RU allocation
 * @NL80211_RATE_INFO_HE_RU_ALLOC_996: 996-tone RU allocation
 * @NL80211_RATE_INFO_HE_RU_ALLOC_2x996: 2x996-tone RU allocation
 */
enum nl80211_he_ru_alloc {
	NL80211_RATE_INFO_HE_RU_ALLOC_26,
	NL80211_RATE_INFO_HE_RU_ALLOC_52,
	NL80211_RATE_INFO_HE_RU_ALLOC_106,
	NL80211_RATE_INFO_HE_RU_ALLOC_242,
	NL80211_RATE_INFO_HE_RU_ALLOC_484,
	NL80211_RATE_INFO_HE_RU_ALLOC_996,
	NL80211_RATE_INFO_HE_RU_ALLOC_2x996,
};

#define RATE_INFO_BW_HE_RU	(RATE_INFO_BW_160 + 1)

/**
 * struct ieee80211_sta_he_cap - STA's HE capabilities
 *
 * This structure describes most essential parameters needed
 * to describe 802.11ax HE capabilities for a STA.
 *
 * @has_he: true iff HE data is valid.
 * @he_cap_elem: Fixed portion of the HE capabilities element.
 * @he_mcs_nss_supp: The supported NSS/MCS combinations.
 * @ppe_thres: Holds the PPE Thresholds data.
 */
struct ieee80211_sta_he_cap {
	bool has_he;
	struct ieee80211_he_cap_elem he_cap_elem;
	struct ieee80211_he_mcs_nss_supp he_mcs_nss_supp;
	u8 ppe_thres[IEEE80211_HE_PPE_THRES_MAX_LEN];
};

/**
 * struct ieee80211_sband_iftype_data
 *
 * This structure encapsulates sband data that is relevant for the interface
 * types defined in %types
 *
 * @types_mask: interface types (bits)
 * @he_cap: holds the HE capabilities
 */
struct ieee80211_sband_iftype_data {
	u16 types_mask;
	struct ieee80211_sta_he_cap he_cap;
};

static inline u16
ieee80211_sband_get_num_iftypes_data(struct ieee80211_supported_band *sband)
{
	return 0;
}

static inline struct ieee80211_sband_iftype_data *
ieee80211_sband_get_iftypes_data(struct ieee80211_supported_band *sband)
{
	return NULL;
}

static inline struct ieee80211_sband_iftype_data *
ieee80211_sband_get_iftypes_data_entry(struct ieee80211_supported_band *sband,
				       u16 i)
{
	WARN_ONCE(1,
		  "Tried to use unsupported sband iftype data\n");
	return NULL;
}

static inline const struct ieee80211_sband_iftype_data *
ieee80211_get_sband_iftype_data(const struct ieee80211_supported_band *sband,
				u8 iftype)
{
	return NULL;
}
#else  /* CFG80211_VERSION < KERNEL_VERSION(4,19,0) */
static inline u16
ieee80211_sband_get_num_iftypes_data(struct ieee80211_supported_band *sband)
{
	return sband->n_iftype_data;
}

static inline const struct ieee80211_sband_iftype_data *
ieee80211_sband_get_iftypes_data(struct ieee80211_supported_band *sband)
{
	return sband->iftype_data;
}

static inline const struct ieee80211_sband_iftype_data *
ieee80211_sband_get_iftypes_data_entry(struct ieee80211_supported_band *sband,
				       u16 i)
{
	return &sband->iftype_data[i];
}
#endif /* CFG80211_VERSION < KERNEL_VERSION(4,19,0) */

#if CFG80211_VERSION < KERNEL_VERSION(5,8,0)
/**
 * ieee80211_get_he_6ghz_sta_cap - return HE 6GHZ capabilities for an sband's
 * STA
 * @sband: the sband to search for the STA on
 *
 * Return: the 6GHz capabilities
 */
static inline __le16
ieee80211_get_he_6ghz_sta_cap(const struct ieee80211_supported_band *sband)
{
	return 0;
}
#endif /* < 5.8.0 */

#if CFG80211_VERSION < KERNEL_VERSION(5,4,0)
#define RATE_INFO_FLAGS_DMG	BIT(3)
#define RATE_INFO_FLAGS_EDMG	BIT(5)
/* yes, it really has that number upstream */
#define NL80211_STA_INFO_ASSOC_AT_BOOTTIME 42

struct ieee80211_he_obss_pd {
	bool enable;
	u8 min_offset;
	u8 max_offset;
};

static inline const struct ieee80211_sta_he_cap *
ieee80211_get_he_iftype_cap(const struct ieee80211_supported_band *sband,
			    u8 iftype)
{
	const struct ieee80211_sband_iftype_data *data =
		ieee80211_get_sband_iftype_data(sband, iftype);

	if (data && data->he_cap.has_he)
		return &data->he_cap;

	return NULL;
}

static inline bool regulatory_pre_cac_allowed(struct wiphy *wiphy)
{
	return false;
}

static inline void
cfg80211_tx_mgmt_expired(struct wireless_dev *wdev, u64 cookie,
			 struct ieee80211_channel *chan, gfp_t gfp)
{
}

#define cfg80211_iftype_allowed iwl7000_cfg80211_iftype_allowed
static inline bool
cfg80211_iftype_allowed(struct wiphy *wiphy, enum nl80211_iftype iftype,
			bool is_4addr, u8 check_swif)

{
	bool is_vlan = iftype == NL80211_IFTYPE_AP_VLAN;

	switch (check_swif) {
	case 0:
		if (is_vlan && is_4addr)
			return wiphy->flags & WIPHY_FLAG_4ADDR_AP;
		return wiphy->interface_modes & BIT(iftype);
	case 1:
		if (!(wiphy->software_iftypes & BIT(iftype)) && is_vlan)
			return wiphy->flags & WIPHY_FLAG_4ADDR_AP;
		return wiphy->software_iftypes & BIT(iftype);
	default:
		break;
	}

	return false;
}

#define cfg80211_tx_mlme_mgmt(netdev, buf, len, reconnect) cfg80211_tx_mlme_mgmt(netdev, buf, len)
#endif /* < 5.4.0 */

#if LINUX_VERSION_IS_LESS(5,5,0)
#define kcov_remote_start_common(id) {}
#define kcov_remote_stop() {}
#endif /* LINUX_VERSION_IS_LESS(5,5,0) */

#if CFG80211_VERSION < KERNEL_VERSION(5,5,0)
#define NL80211_EXT_FEATURE_AQL -1

#define IEEE80211_DEFAULT_AQL_TXQ_LIMIT_L	5000
#define IEEE80211_DEFAULT_AQL_TXQ_LIMIT_H	12000
#define IEEE80211_AQL_THRESHOLD			24000
#endif

#define netdev_set_priv_destructor(_dev, _destructor) \
	(_dev)->needs_free_netdev = true; \
	(_dev)->priv_destructor = (_destructor);
#define netdev_set_def_destructor(_dev) \
	(_dev)->needs_free_netdev = true;

#if LINUX_VERSION_IS_LESS(6,1,0)
static inline u16 get_random_u16(void)
{
	return get_random_int() & 0xffff;
}
#endif

#define rb_root_node(root) (root)->rb_root.rb_node

#if LINUX_VERSION_IS_LESS(4,20,0)
static inline struct sk_buff *__skb_peek(const struct sk_buff_head *list_)
{
	return list_->next;
}
#endif

#if LINUX_VERSION_IS_LESS(4,15,0)
#define NL80211_EXT_FEATURE_FILS_MAX_CHANNEL_TIME -1
#define NL80211_EXT_FEATURE_ACCEPT_BCAST_PROBE_RESP -1
#define NL80211_EXT_FEATURE_OCE_PROBE_REQ_HIGH_TX_RATE -1
#define NL80211_EXT_FEATURE_OCE_PROBE_REQ_DEFERRAL_SUPPRESSION -1
#define NL80211_SCAN_FLAG_FILS_MAX_CHANNEL_TIME BIT(4)
#define NL80211_SCAN_FLAG_ACCEPT_BCAST_PROBE_RESP BIT(5)
#define NL80211_SCAN_FLAG_OCE_PROBE_REQ_HIGH_TX_RATE BIT(6)
#define NL80211_SCAN_FLAG_OCE_PROBE_REQ_DEFERRAL_SUPPRESSION BIT(7)
#endif

#if CFG80211_VERSION < KERNEL_VERSION(4,17,0)
struct ieee80211_wmm_ac {
	u16 cw_min;
	u16 cw_max;
	u16 cot;
	u8 aifsn;
};

struct ieee80211_wmm_rule {
	struct ieee80211_wmm_ac client[IEEE80211_NUM_ACS];
	struct ieee80211_wmm_ac ap[IEEE80211_NUM_ACS];
};

static inline int
reg_query_regdb_wmm(char *alpha2, int freq, u32 *ptr,
		    struct ieee80211_wmm_rule *rule)
{
	pr_debug_once(KERN_DEBUG
		      "iwl7000: ETSI WMM data not implemented yet!\n");
	return -ENODATA;
}
#endif /* < 4.17.0 */

#if CFG80211_VERSION >= KERNEL_VERSION(5,7,0)
#define cfg_he_oper(params) params->he_oper
#else
#define cfg_he_oper(params) ((struct ieee80211_he_operation *)NULL)
#endif /* >= 5.7 */

#if CFG80211_VERSION >= KERNEL_VERSION(4,20,0)
#define cfg_he_cap(params) params->he_cap
#else
#define cfg_he_cap(params) ((struct ieee80211_he_cap_elem *)NULL)

/* Layer 2 Update frame (802.2 Type 1 LLC XID Update response) */
struct iapp_layer2_update {
	u8 da[ETH_ALEN];	/* broadcast */
	u8 sa[ETH_ALEN];	/* STA addr */
	__be16 len;		/* 6 */
	u8 dsap;		/* 0 */
	u8 ssap;		/* 0 */
	u8 control;
	u8 xid_info[3];
} __packed;

static inline
void cfg80211_send_layer2_update(struct net_device *dev, const u8 *addr)
{
	struct iapp_layer2_update *msg;
	struct sk_buff *skb;

	/* Send Level 2 Update Frame to update forwarding tables in layer 2
	 * bridge devices */

	skb = dev_alloc_skb(sizeof(*msg));
	if (!skb)
		return;
	msg = skb_put(skb, sizeof(*msg));

	/* 802.2 Type 1 Logical Link Control (LLC) Exchange Identifier (XID)
	 * Update response frame; IEEE Std 802.2-1998, 5.4.1.2.1 */

	eth_broadcast_addr(msg->da);
	ether_addr_copy(msg->sa, addr);
	msg->len = htons(6);
	msg->dsap = 0;
	msg->ssap = 0x01;	/* NULL LSAP, CR Bit: Response */
	msg->control = 0xaf;	/* XID response lsb.1111F101.
				 * F=0 (no poll command; unsolicited frame) */
	msg->xid_info[0] = 0x81;	/* XID format identifier */
	msg->xid_info[1] = 1;	/* LLC types/classes: Type 1 LLC */
	msg->xid_info[2] = 0;	/* XID sender's receive window size (RW) */

	skb->dev = dev;
	skb->protocol = eth_type_trans(skb, dev);
	memset(skb->cb, 0, sizeof(skb->cb));
	netif_rx_ni(skb);
}

#define NL80211_EXT_FEATURE_CAN_REPLACE_PTK0 -1
#endif /* >= 4.20 */

#if CFG80211_VERSION < KERNEL_VERSION(4,19,0)
#define NL80211_EXT_FEATURE_SCAN_RANDOM_SN		-1
#define NL80211_EXT_FEATURE_SCAN_MIN_PREQ_CONTENT	-1
#endif

#if CFG80211_VERSION < KERNEL_VERSION(4,20,0)
#define NL80211_EXT_FEATURE_ENABLE_FTM_RESPONDER	-1
#endif

#if CFG80211_VERSION < KERNEL_VERSION(4,17,0)
#define NL80211_EXT_FEATURE_CONTROL_PORT_OVER_NL80211	-1

/* define it here so we can set the values in mac80211... */
struct sta_opmode_info {
	u32 changed;
	enum nl80211_smps_mode smps_mode;
	enum nl80211_chan_width bw;
	u8 rx_nss;
};

#define STA_OPMODE_MAX_BW_CHANGED	0
#define STA_OPMODE_SMPS_MODE_CHANGED	0
#define STA_OPMODE_N_SS_CHANGED		0

/* ...but make the user an empty function, since we don't have it in cfg80211 */
#define cfg80211_sta_opmode_change_notify(...)  do { } while (0)

/*
 * we should never call this function since we force
 * cfg_control_port_over_nl80211 to be 0.
 */
#define cfg80211_rx_control_port(...) do { } while (0)

#define cfg_control_port_over_nl80211(params) 0
#else
#if CFG80211_VERSION >= KERNEL_VERSION(4,17,0) && \
	CFG80211_VERSION < KERNEL_VERSION(4,18,0)
static inline bool
iwl7000_cfg80211_rx_control_port(struct net_device *dev, struct sk_buff *skb,
				 bool unencrypted, int link_id)
{
	struct ethhdr *ehdr;

	/*
	 * Try to linearize the skb, because in 4.17
	 * cfg80211_rx_control_port() is broken and needs it to be
	 * linear.  If it fails, too bad, we fail too.
	 */
	if (skb_linearize(skb))
		return false;

	ehdr = eth_hdr(skb);

	return cfg80211_rx_control_port(dev, skb->data, skb->len,
				ehdr->h_source,
				be16_to_cpu(skb->protocol), unencrypted);
}
#define cfg80211_rx_control_port iwl7000_cfg80211_rx_control_port
#endif
#define cfg_control_port_over_nl80211(params) (params)->control_port_over_nl80211
#endif

#if CFG80211_VERSION < KERNEL_VERSION(4,18,0)
#define NL80211_EXT_FEATURE_TXQS -1

/*
 * This function just allocates tidstats and returns 0 if it
 * succeeded.  Since pre-4.18 tidstats is pre-allocated as part of
 * sinfo, we can simply return 0 because it's already allocated.
 */
#define cfg80211_sinfo_alloc_tid_stats(...) 0

#define WIPHY_PARAM_TXQ_LIMIT		0
#define WIPHY_PARAM_TXQ_MEMORY_LIMIT	0
#define WIPHY_PARAM_TXQ_QUANTUM		0

#else
static inline int
backport_cfg80211_sinfo_alloc_tid_stats(struct station_info *sinfo, gfp_t gfp)
{
	int ret;
	cfg_station_info_t cfg_info = {};

	ret = cfg80211_sinfo_alloc_tid_stats(&cfg_info, gfp);
	if (ret)
		return ret;

	sinfo->pertid = cfg_info.pertid;

	return 0;
}
#define cfg80211_sinfo_alloc_tid_stats backport_cfg80211_sinfo_alloc_tid_stats
#endif

#if CFG80211_VERSION < KERNEL_VERSION(4,19,0)
#define NL80211_SCAN_FLAG_RANDOM_SN		0
#define NL80211_SCAN_FLAG_MIN_PREQ_CONTENT	0
#endif /* CFG80211_VERSION < KERNEL_VERSION(4,19,0) */

#if CFG80211_VERSION < KERNEL_VERSION(4,20,0)
enum nl80211_ftm_responder_stats {
	__NL80211_FTM_STATS_INVALID,
	NL80211_FTM_STATS_SUCCESS_NUM,
	NL80211_FTM_STATS_PARTIAL_NUM,
	NL80211_FTM_STATS_FAILED_NUM,
	NL80211_FTM_STATS_ASAP_NUM,
	NL80211_FTM_STATS_NON_ASAP_NUM,
	NL80211_FTM_STATS_TOTAL_DURATION_MSEC,
	NL80211_FTM_STATS_UNKNOWN_TRIGGERS_NUM,
	NL80211_FTM_STATS_RESCHEDULE_REQUESTS_NUM,
	NL80211_FTM_STATS_OUT_OF_WINDOW_TRIGGERS_NUM,
	NL80211_FTM_STATS_PAD,

	/* keep last */
	__NL80211_FTM_STATS_AFTER_LAST,
	NL80211_FTM_STATS_MAX = __NL80211_FTM_STATS_AFTER_LAST - 1
};

struct cfg80211_ftm_responder_stats {
	u32 filled;
	u32 success_num;
	u32 partial_num;
	u32 failed_num;
	u32 asap_num;
	u32 non_asap_num;
	u64 total_duration_ms;
	u32 unknown_triggers_num;
	u32 reschedule_requests_num;
	u32 out_of_window_triggers_num;
};
#endif /* CFG80211_VERSION < KERNEL_VERSION(4,20,0) */

#ifndef ETH_P_PREAUTH
#define ETH_P_PREAUTH  0x88C7	/* 802.11 Preauthentication */
#endif

#if CFG80211_VERSION < KERNEL_VERSION(4,21,0)
enum nl80211_preamble {
	NL80211_PREAMBLE_LEGACY,
	NL80211_PREAMBLE_HT,
	NL80211_PREAMBLE_VHT,
	NL80211_PREAMBLE_DMG,
};

enum nl80211_peer_measurement_status {
	NL80211_PMSR_STATUS_SUCCESS,
	NL80211_PMSR_STATUS_REFUSED,
	NL80211_PMSR_STATUS_TIMEOUT,
	NL80211_PMSR_STATUS_FAILURE,
};

enum nl80211_peer_measurement_type {
	NL80211_PMSR_TYPE_INVALID,

	NL80211_PMSR_TYPE_FTM,

	NUM_NL80211_PMSR_TYPES,
	NL80211_PMSR_TYPE_MAX = NUM_NL80211_PMSR_TYPES - 1
};

enum nl80211_peer_measurement_ftm_failure_reasons {
	NL80211_PMSR_FTM_FAILURE_UNSPECIFIED,
	NL80211_PMSR_FTM_FAILURE_NO_RESPONSE,
	NL80211_PMSR_FTM_FAILURE_REJECTED,
	NL80211_PMSR_FTM_FAILURE_WRONG_CHANNEL,
	NL80211_PMSR_FTM_FAILURE_PEER_NOT_CAPABLE,
	NL80211_PMSR_FTM_FAILURE_INVALID_TIMESTAMP,
	NL80211_PMSR_FTM_FAILURE_PEER_BUSY,
	NL80211_PMSR_FTM_FAILURE_BAD_CHANGED_PARAMS,
};

struct cfg80211_pmsr_ftm_result {
	const u8 *lci;
	const u8 *civicloc;
	unsigned int lci_len;
	unsigned int civicloc_len;
	enum nl80211_peer_measurement_ftm_failure_reasons failure_reason;
	u32 num_ftmr_attempts, num_ftmr_successes;
	s16 burst_index;
	u8 busy_retry_time;
	u8 num_bursts_exp;
	u8 burst_duration;
	u8 ftms_per_burst;
	s32 rssi_avg;
	s32 rssi_spread;
	struct rate_info tx_rate, rx_rate;
	s64 rtt_avg;
	s64 rtt_variance;
	s64 rtt_spread;
	s64 dist_avg;
	s64 dist_variance;
	s64 dist_spread;

	u16 num_ftmr_attempts_valid:1,
	    num_ftmr_successes_valid:1,
	    rssi_avg_valid:1,
	    rssi_spread_valid:1,
	    tx_rate_valid:1,
	    rx_rate_valid:1,
	    rtt_avg_valid:1,
	    rtt_variance_valid:1,
	    rtt_spread_valid:1,
	    dist_avg_valid:1,
	    dist_variance_valid:1,
	    dist_spread_valid:1;
};

struct cfg80211_pmsr_result {
	u64 host_time, ap_tsf;
	enum nl80211_peer_measurement_status status;

	u8 addr[ETH_ALEN];

	u8 final:1,
	   ap_tsf_valid:1;

	enum nl80211_peer_measurement_type type;

	union {
		struct cfg80211_pmsr_ftm_result ftm;
	};
};

struct cfg80211_pmsr_ftm_request_peer {
	enum nl80211_preamble preamble;
	u16 burst_period;
	u8 requested:1,
	   asap:1,
	   request_lci:1,
	   request_civicloc:1;
	u8 num_bursts_exp;
	u8 burst_duration;
	u8 ftms_per_burst;
	u8 ftmr_retries;
};

struct cfg80211_pmsr_request_peer {
	u8 addr[ETH_ALEN];
	struct cfg80211_chan_def chandef;
	u8 report_ap_tsf:1;
	struct cfg80211_pmsr_ftm_request_peer ftm;
};

struct cfg80211_pmsr_request {
	u64 cookie;
	void *drv_data;
	u32 n_peers;
	u32 nl_portid;

	u32 timeout;

	u8 mac_addr[ETH_ALEN] __aligned(2);
	u8 mac_addr_mask[ETH_ALEN] __aligned(2);

	struct list_head list;

	struct cfg80211_pmsr_request_peer peers[];
};

static inline void cfg80211_pmsr_report(struct wireless_dev *wdev,
					struct cfg80211_pmsr_request *req,
					struct cfg80211_pmsr_result *result,
					gfp_t gfp)
{
	/* nothing */
}

static inline void cfg80211_pmsr_complete(struct wireless_dev *wdev,
					  struct cfg80211_pmsr_request *req,
					  gfp_t gfp)
{
	kfree(req);
}

#endif /* CFG80211_VERSION < KERNEL_VERSION(4,21,0) */

#if CFG80211_VERSION < KERNEL_VERSION(5,1,0)
static inline int cfg80211_vendor_cmd_get_sender(struct wiphy *wiphy)
{
	/* cfg80211 doesn't really let us backport this */
	return 0;
}

static inline struct sk_buff *
cfg80211_vendor_event_alloc_ucast(struct wiphy *wiphy,
				  struct wireless_dev *wdev,
				  unsigned int portid, int approxlen,
				  int event_idx, gfp_t gfp)
{
	/*
	 * We might be able to fake backporting this, but not the
	 * associated changes to __cfg80211_send_event_skb(), at
	 * least not without duplicating all that code.
	 * And in any case, we cannot backport the get_sender()
	 * function above properly, so we might as well ignore
	 * this all.
	 */
	return NULL;
}

static inline const struct element *
cfg80211_find_elem(u8 eid, const u8 *ies, int len)
{
	return (void *)cfg80211_find_ie(eid, ies, len);
}

static inline const struct element *
cfg80211_find_ext_elem(u8 ext_eid, const u8 *ies, int len)
{
	return (void *)cfg80211_find_ext_ie(ext_eid, ies, len);
}

static inline const struct element *
ieee80211_bss_get_elem(struct cfg80211_bss *bss, u8 id)
{
	const struct cfg80211_bss_ies *ies;

	ies = rcu_dereference(bss->ies);
	if (!ies)
		return NULL;

	return cfg80211_find_elem(id, ies->data, ies->len);
}

#define IEEE80211_DEFAULT_AIRTIME_WEIGHT       256

#endif /* CFG80211_VERSION < KERNEL_VERSION(5,1,0) */

#if CFG80211_VERSION < KERNEL_VERSION(5,2,0)
#define NL80211_EXT_FEATURE_EXT_KEY_ID -1
#define NL80211_EXT_FEATURE_AIRTIME_FAIRNESS -1
#endif /* CFG80211_VERSION < KERNEL_VERSION(5,2,0) */

#if CFG80211_VERSION < KERNEL_VERSION(5,3,0)
static inline void cfg80211_bss_iter(struct wiphy *wiphy,
				     struct cfg80211_chan_def *chandef,
				     void (*iter)(struct wiphy *wiphy,
						  struct cfg80211_bss *bss,
						  void *data),
				     void *iter_data)
{
	/*
	 * It might be possible to backport this function, but that would
	 * require duplicating large portions of data structure/code, so
	 * leave it empty for now.
	 */
}
#define NL80211_EXT_FEATURE_SAE_OFFLOAD -1
#endif /* CFG80211_VERSION < KERNEL_VERSION(5,3,0) */

#if CFG80211_VERSION < KERNEL_VERSION(5,4,0)
#define NL80211_BAND_6GHZ 3
#endif

#if CFG80211_VERSION < KERNEL_VERSION(5,7,0)
#define ieee80211_preamble_he() 0
#define ftm_non_trigger_based(peer)	0
#define ftm_trigger_based(peer)	0
#else
#define ftm_non_trigger_based(peer)	((peer)->ftm.non_trigger_based)
#define ftm_trigger_based(peer)	((peer)->ftm.trigger_based)
#define ieee80211_preamble_he() BIT(NL80211_PREAMBLE_HE)
#endif

#if CFG80211_VERSION < KERNEL_VERSION(5,13,0)
#define ftm_lmr_feedback(peer)		0
#else
#define ftm_lmr_feedback(peer)		((peer)->ftm.lmr_feedback)
#endif

#if CFG80211_VERSION < KERNEL_VERSION(5,14,0)
#define ftm_bss_color(peer)		0
#else
#define ftm_bss_color(peer)		((peer)->ftm.bss_color)
#endif

#if CFG80211_VERSION < KERNEL_VERSION(5,6,0)
int ieee80211_get_vht_max_nss(struct ieee80211_vht_cap *cap,
			      enum ieee80211_vht_chanwidth bw,
			      int mcs, bool ext_nss_bw_capable,
			      unsigned int max_vht_nss);
#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,5,0)
ssize_t cfg80211_defragment_element(const struct element *elem, const u8 *ies,
				    size_t ieslen, u8 *data, size_t data_len,
				    u8 frag_id);
#endif

#if CFG80211_VERSION < KERNEL_VERSION(5,8,0)
#define NL80211_EXT_FEATURE_BEACON_PROTECTION_CLIENT -1
#endif

#if CFG80211_VERSION < KERNEL_VERSION(5,7,0)
#define NL80211_EXT_FEATURE_BEACON_PROTECTION -1
#define NL80211_EXT_FEATURE_PROTECTED_TWT -1
#endif

static inline size_t cfg80211_rekey_get_kek_len(struct cfg80211_gtk_rekey_data *data)
{
#if CFG80211_VERSION < KERNEL_VERSION(5,8,0)
	return NL80211_KEK_LEN;
#else
	return data->kek_len;
#endif
}

static inline size_t cfg80211_rekey_get_kck_len(struct cfg80211_gtk_rekey_data *data)
{
#if CFG80211_VERSION < KERNEL_VERSION(5,8,0)
	return NL80211_KCK_LEN;
#else
	return data->kck_len;
#endif
}

static inline size_t cfg80211_rekey_akm(struct cfg80211_gtk_rekey_data *data)
{
#if CFG80211_VERSION < KERNEL_VERSION(5,8,0)
	/* we dont really use this */
	return 0;
#else
	return data->akm;
#endif
}

#if CFG80211_VERSION < KERNEL_VERSION(5,4,0)
/**
 * struct cfg80211_he_bss_color - AP settings for BSS coloring
 *
 * @color: the current color.
 * @disabled: is the feature disabled.
 * @partial: define the AID equation.
 */
struct cfg80211_he_bss_color {
	u8 color;
	bool disabled;
	bool partial;
};

/**
 * enum nl80211_tid_config - TID config state
 * @NL80211_TID_CONFIG_ENABLE: Enable config for the TID
 * @NL80211_TID_CONFIG_DISABLE: Disable config for the TID
 */
enum nl80211_tid_config {
	NL80211_TID_CONFIG_ENABLE,
	NL80211_TID_CONFIG_DISABLE,
};
/**
 * struct cfg80211_tid_cfg - TID specific configuration
 * @config_override: Flag to notify driver to reset TID configuration
 *	of the peer.
 * @tids: bitmap of TIDs to modify
 * @mask: bitmap of attributes indicating which parameter changed,
 *	similar to &nl80211_tid_config_supp.
 * @noack: noack configuration value for the TID
 * @retry_long: retry count value
 * @retry_short: retry count value
 * @ampdu: Enable/Disable aggregation
 * @rtscts: Enable/Disable RTS/CTS
 */
struct cfg80211_tid_cfg {
	bool config_override;
	u8 tids;
	u32 mask;
	enum nl80211_tid_config noack;
	u8 retry_long, retry_short;
	enum nl80211_tid_config ampdu;
	enum nl80211_tid_config rtscts;
};

/**
 * struct cfg80211_tid_config - TID configuration
 * @peer: Station's MAC address
 * @n_tid_conf: Number of TID specific configurations to be applied
 * @tid_conf: Configuration change info
 */
struct cfg80211_tid_config {
	const u8 *peer;
	u32 n_tid_conf;
	struct cfg80211_tid_cfg tid_conf[];
};
#endif

#if CFG80211_VERSION < KERNEL_VERSION(5,7,0)
#define NL80211_EXT_FEATURE_CONTROL_PORT_NO_PREAUTH -1
#define NL80211_EXT_FEATURE_DEL_IBSS_STA -1

static inline bool
cfg80211_crypto_control_port_no_preauth(struct cfg80211_crypto_settings *crypto)
{
	return false;
}

static inline unsigned long
cfg80211_wiphy_tx_queue_len(struct wiphy *wiphy)
{
	return 0;
}
#else /* < 5.7 */
static inline bool
cfg80211_crypto_control_port_no_preauth(struct cfg80211_crypto_settings *crypto)
{
	return crypto->control_port_no_preauth;
}

static inline unsigned long
cfg80211_wiphy_tx_queue_len(struct wiphy *wiphy)
{
	return wiphy->tx_queue_len;
}
#endif /* < 5.7 */

int ieee80211_tx_control_port(struct wiphy *wiphy, struct net_device *dev,
			      const u8 *buf, size_t len,
			      const u8 *dest, __be16 proto, bool unencrypted,
			      int link_id, u64 *cookie);

#if CFG80211_VERSION < KERNEL_VERSION(5,8,0)
#define NL80211_EXT_FEATURE_CONTROL_PORT_OVER_NL80211_TX_STATUS -1
#define NL80211_EXT_FEATURE_SCAN_FREQ_KHZ -1

static inline int
cfg80211_chan_freq_offset(struct ieee80211_channel *chan)
{
	return 0;
}

static inline void
cfg80211_chandef_freq1_offset_set(struct cfg80211_chan_def *chandef, u16 e)
{
}

static inline u16
cfg80211_chandef_freq1_offset(struct cfg80211_chan_def *chandef)
{
	return 0;
}

static inline void
cfg80211_control_port_tx_status(struct wireless_dev *wdev, u64 cookie,
				const u8 *buf, size_t len, bool ack,
				gfp_t gfp)
{
}

static inline __le16
ieee80211_get_he_6ghz_capa(const struct ieee80211_supported_band *sband,
			   enum nl80211_iftype iftype)
{
	return 0;
}
void ieee80211_mgmt_frame_register(struct wiphy *wiphy,
				   struct wireless_dev *wdev,
				   u16 frame_type, bool reg);
static inline __le16
cfg80211_iftd_he_6ghz_capa(const struct ieee80211_sband_iftype_data *iftd)
{
	return 0;
}

static inline void
cfg80211_iftd_set_he_6ghz_capa(struct ieee80211_sband_iftype_data *iftd,
			       __le16 capa)
{
}

static inline int
bp_ieee80211_tx_control_port(struct wiphy *wiphy, struct net_device *dev,
			     const u8 *buf, size_t len,
			     const u8 *dest, __be16 proto, bool unencrypted)
{
	return ieee80211_tx_control_port(wiphy, dev, buf, len, dest, proto,
					 unencrypted, -1, NULL);
}
#else /* < 5.8 */
static inline int
cfg80211_chan_freq_offset(struct ieee80211_channel *chan)
{
	return chan->freq_offset;
}

static inline void
cfg80211_chandef_freq1_offset_set(struct cfg80211_chan_def *chandef, u16 e)
{
	chandef->freq1_offset = e;
}

static inline u16
cfg80211_chandef_freq1_offset(struct cfg80211_chan_def *chandef)
{
	return chandef->freq1_offset;
}

static inline __le16
cfg80211_iftd_he_6ghz_capa(const struct ieee80211_sband_iftype_data *iftd)
{
	return iftd->he_6ghz_capa.capa;
}

static inline void
cfg80211_iftd_set_he_6ghz_capa(struct ieee80211_sband_iftype_data *iftd,
			       __le16 capa)
{
	iftd->he_6ghz_capa.capa = capa;
}
#endif /* < 5.8 */

#if LINUX_VERSION_IS_GEQ(4,20,0)
#include <hdrs/linux/compiler_attributes.h>
#else
# define __counted_by(member)
#endif

#ifndef __has_attribute
# define __has_attribute(x) __GCC4_has_attribute_##x
#endif

#ifndef __GCC4_has_attribute___fallthrough__
# define __GCC4_has_attribute___fallthrough__         0
#endif /* __GCC4_has_attribute___fallthrough__ */

#ifndef fallthrough
/*
 * Add the pseudo keyword 'fallthrough' so case statement blocks
 * must end with any of these keywords:
 *   break;
 *   fallthrough;
 *   goto <label>;
 *   return [expression];
 *
 *  gcc: https://gcc.gnu.org/onlinedocs/gcc/Statement-Attributes.html#Statement-Attributes
 */
#if __has_attribute(__fallthrough__)
# define fallthrough                    __attribute__((__fallthrough__))
#else
# define fallthrough                    do {} while (0)  /* fallthrough */
#endif
#endif /* fallthrough */

#if CFG80211_VERSION < KERNEL_VERSION(5,10,0)
#define WIPHY_FLAG_SPLIT_SCAN_6GHZ 0
#define NL80211_SCAN_FLAG_COLOCATED_6GHZ 0
#endif /* < 5.10 */

#if CFG80211_VERSION < KERNEL_VERSION(5,11,0)
static inline void
LINUX_BACKPORT(cfg80211_ch_switch_started_notify)(struct net_device *dev,
						  struct cfg80211_chan_def *chandef,
						  unsigned int link_id, u8 count,
						  bool quiet, u16 punct_bitmap)
{
	cfg80211_ch_switch_started_notify(dev, chandef, count);
}
#define cfg80211_ch_switch_started_notify LINUX_BACKPORT(cfg80211_ch_switch_started_notify)

#elif CFG80211_VERSION < KERNEL_VERSION(6,1,0)
static inline void
LINUX_BACKPORT(cfg80211_ch_switch_started_notify)(struct net_device *dev,
						  struct cfg80211_chan_def *chandef,
						  unsigned int link_id, u8 count,
						  bool quiet, u16 punct_bitmap)
{
	cfg80211_ch_switch_started_notify(dev, chandef, count, quiet);
}
#define cfg80211_ch_switch_started_notify LINUX_BACKPORT(cfg80211_ch_switch_started_notify)
#endif /* < 6.1.0 */

#ifndef ETH_TLEN
#define ETH_TLEN	2		/* Octets in ethernet type field */
#endif

#if CFG80211_VERSION < KERNEL_VERSION(5,4,0)
static inline bool cfg80211_channel_is_psc(struct ieee80211_channel *chan)
{
	return false;
}
#elif CFG80211_VERSION < KERNEL_VERSION(5,8,0)
static inline bool cfg80211_channel_is_psc(struct ieee80211_channel *chan)
{
	if (chan->band != NL80211_BAND_6GHZ)
		return false;

	return ieee80211_frequency_to_channel(chan->center_freq) % 16 == 5;
}
#endif /* < 5.8.0 */

#if LINUX_VERSION_IS_LESS(5,9,0)

#define kfree_sensitive(p) kzfree(p)

#include <linux/thermal.h>
#ifdef CONFIG_THERMAL
static inline int thermal_zone_device_enable(struct thermal_zone_device *tz)
{ return 0; }
#else /* CONFIG_THERMAL */
static inline int thermal_zone_device_enable(struct thermal_zone_device *tz)
{ return -ENODEV; }
#endif /* CONFIG_THERMAL */

#endif /* < 5.9.0 */

#if LINUX_VERSION_IS_LESS(5,4,0)
static inline void
tasklet_setup(struct tasklet_struct *t,
	      void (*callback)(struct tasklet_struct *))
{
	void (*cb)(unsigned long data) = (void *)callback;

	tasklet_init(t, cb, (unsigned long)t);
}

#define from_tasklet(var, callback_tasklet, tasklet_fieldname) \
	container_of(callback_tasklet, typeof(*var), tasklet_fieldname)
#endif /* < 5.4.0 */

#if CFG80211_VERSION < KERNEL_VERSION(5,9,0)
#define NL80211_BAND_S1GHZ 4
#define NL80211_CHAN_WIDTH_1 8
#define NL80211_CHAN_WIDTH_2 9
#define NL80211_CHAN_WIDTH_4 10
#define NL80211_CHAN_WIDTH_8 11
#define NL80211_CHAN_WIDTH_16 12
#endif /* CFG80211_VERSION < 5.9.0 */

#if LINUX_VERSION_IS_LESS(4,19,0)
static inline void netif_receive_skb_list(struct sk_buff_head *head)
{
	struct sk_buff *skb, *next;

	skb_queue_walk_safe(head, skb, next) {
		__skb_unlink(skb, head);
		netif_receive_skb(skb);
	}
}

static inline u8 cfg80211_he_gi(struct rate_info *ri)
{
	return 0;
}

#else /* < 4.19.0 */

static inline u8 cfg80211_he_gi(struct rate_info *ri)
{
	return ri->he_gi;
}

#endif /* < 4.19.0 */

#if CFG80211_VERSION < KERNEL_VERSION(5,10,0)
static inline enum nl80211_chan_width
ieee80211_s1g_channel_width(const struct ieee80211_channel *chan)
{
	return NL80211_CHAN_WIDTH_20_NOHT;
}

#define NL80211_BSS_CHAN_WIDTH_1	3
#define NL80211_BSS_CHAN_WIDTH_2	4

#endif /* CFG80211_VERSION < KERNEL_VERSION(5,10,0) */

#if LINUX_VERSION_IS_LESS(5,10,0)
/**
 *      dev_fetch_sw_netstats - get per-cpu network device statistics
 *      @s: place to store stats
 *      @netstats: per-cpu network stats to read from
 *
 *      Read per-cpu network statistics and populate the related fields in @s.
 */
static inline
void dev_fetch_sw_netstats(struct rtnl_link_stats64 *s,
                           const struct pcpu_sw_netstats __percpu *netstats)
{
        int cpu;

        for_each_possible_cpu(cpu) {
                const struct pcpu_sw_netstats *stats;
                struct pcpu_sw_netstats tmp;
                unsigned int start;

                stats = per_cpu_ptr(netstats, cpu);
                do {
                        start = u64_stats_fetch_begin_irq(&stats->syncp);
                        tmp.rx_packets = stats->rx_packets;
                        tmp.rx_bytes   = stats->rx_bytes;
                        tmp.tx_packets = stats->tx_packets;
                        tmp.tx_bytes   = stats->tx_bytes;
                } while (u64_stats_fetch_retry_irq(&stats->syncp, start));

                s->rx_packets += tmp.rx_packets;
                s->rx_bytes   += tmp.rx_bytes;
                s->tx_packets += tmp.tx_packets;
                s->tx_bytes   += tmp.tx_bytes;
        }
}

static inline void dev_sw_netstats_rx_add(struct net_device *dev, unsigned int len)
{
	struct pcpu_sw_netstats *tstats = this_cpu_ptr(dev->tstats);

	u64_stats_update_begin(&tstats->syncp);
	tstats->rx_bytes += len;
	tstats->rx_packets++;
	u64_stats_update_end(&tstats->syncp);
}

#define bp_ieee80211_set_unsol_bcast_probe_resp(sdata, params, link, link_conf) 0
#define bp_unsol_bcast_probe_resp_interval(params) 0

#else /* < 5.10 */

#define bp_ieee80211_set_unsol_bcast_probe_resp(sdata, params, link, link_conf) \
	ieee80211_set_unsol_bcast_probe_resp(sdata, params, link, link_conf)
#define bp_unsol_bcast_probe_resp_interval(params) \
	(params->unsol_bcast_probe_resp.interval)

#endif /* < 5.10 */

#if CFG80211_VERSION < KERNEL_VERSION(5,10,0) &&     \
	(CFG80211_VERSION < KERNEL_VERSION(5,4,0) || \
	 CFG80211_VERSION >= KERNEL_VERSION(5,5,0))
enum nl80211_sar_type {
	NL80211_SAR_TYPE_NONE,
};

struct cfg80211_sar_sub_specs {
	s32 power;
	u32 freq_range_index;
};

/**
 * struct cfg80211_sar_specs - sar limit specs
 * @type: it's set with power in 0.25dbm or other types
 * @num_sub_specs: number of sar sub specs
 * @sub_specs: memory to hold the sar sub specs
 */
struct cfg80211_sar_specs {
	enum nl80211_sar_type type;
	u32 num_sub_specs;
	struct cfg80211_sar_sub_specs sub_specs[];
};
#endif /* < 5.4.0 */

#if CFG80211_VERSION < KERNEL_VERSION(5,14,0)
#define NL80211_EXT_FEATURE_PROT_RANGE_NEGO_AND_MEASURE -1

static inline bool cfg80211_any_usable_channels(struct wiphy *wiphy,
						unsigned long sband_mask,
						u32 prohibited_flags)
{
	int idx;

	prohibited_flags |= IEEE80211_CHAN_DISABLED;

	for_each_set_bit(idx, &sband_mask, NUM_NL80211_BANDS) {
		struct ieee80211_supported_band *sband = wiphy->bands[idx];
		int chanidx;

		if (!sband)
			continue;

		for (chanidx = 0; chanidx < sband->n_channels; chanidx++) {
			struct ieee80211_channel *chan;

			chan = &sband->channels[chanidx];

			if (chan->flags & prohibited_flags)
				continue;

			return true;
		}
	}

	return false;
}
#endif /* < 5.13.0 */

#if LINUX_VERSION_IS_LESS(5,10,0)
static inline u64 skb_get_kcov_handle(struct sk_buff *skb)
{
	return 0;
}
#endif

#if LINUX_VERSION_IS_LESS(5,11,00)
#ifndef CONFIG_LOCKDEP
/* upstream since 5.11 in this exact same way - calls compile away */
int lockdep_is_held(const void *);
#endif

static inline void dev_sw_netstats_tx_add(struct net_device *dev,
					  unsigned int packets,
					  unsigned int len)
{
	struct pcpu_sw_netstats *tstats = this_cpu_ptr(dev->tstats);

	u64_stats_update_begin(&tstats->syncp);
	tstats->tx_bytes += len;
	tstats->tx_packets += packets;
	u64_stats_update_end(&tstats->syncp);
}
#endif

#if CFG80211_VERSION < KERNEL_VERSION(5,12,0)
#define wiphy_dereference(w, r) rtnl_dereference(r)
#define regulatory_set_wiphy_regd_sync(w, r) regulatory_set_wiphy_regd_sync_rtnl(w, r)
#define lockdep_assert_wiphy(w) ASSERT_RTNL()
#define cfg80211_register_netdevice(n) register_netdevice(n)
#define cfg80211_unregister_netdevice(n) unregister_netdevice(n)
#define cfg80211_sched_scan_stopped_locked(w, r) cfg80211_sched_scan_stopped_rtnl(w, r)
#define ASSOC_REQ_DISABLE_HE BIT(4)
static inline void __iwl7000_cfg80211_unregister_wdev(struct wireless_dev *wdev)
{
	if (wdev->netdev)
		unregister_netdevice(wdev->netdev);
	else
		cfg80211_unregister_wdev(wdev);
}
#define cfg80211_unregister_wdev __iwl7000_cfg80211_unregister_wdev
#define lockdep_is_wiphy_held(wiphy) 0
static inline bool wdev_registered(struct wireless_dev *wdev)
{
	return true;
}
#else
#define lockdep_is_wiphy_held(wiphy) lockdep_is_held(&(wiphy)->mtx)
static inline bool wdev_registered(struct wireless_dev *wdev)
{
	return wdev->registered;
}
#endif /* < 5.12 */

#if CFG80211_VERSION < KERNEL_VERSION(5,17,0)
static inline void
cfg80211_assoc_comeback(struct net_device *netdev,
			struct cfg80211_bss *bss, u32 timeout)
{
}
#endif /* CFG80211_VERSION < KERNEL_VERSION(5,17,0) */

#if CFG80211_VERSION < KERNEL_VERSION(5,18,0)
#define ieee80211_data_to_8023_exthdr iwl7000_ieee80211_data_to_8023_exthdr
int ieee80211_data_to_8023_exthdr(struct sk_buff *skb, struct ethhdr *ehdr,
				  const u8 *addr, enum nl80211_iftype iftype,
				  u8 data_offset, bool is_amsdu);

#define ieee80211_data_to_8023 iwl7000_ieee80211_data_to_8023
static inline int ieee80211_data_to_8023(struct sk_buff *skb, const u8 *addr,
					 enum nl80211_iftype iftype)
{
	return ieee80211_data_to_8023_exthdr(skb, NULL, addr, iftype, 0, false);
}

enum nl80211_eht_gi {
	NL80211_RATE_INFO_EHT_GI_0_8,
	NL80211_RATE_INFO_EHT_GI_1_6,
	NL80211_RATE_INFO_EHT_GI_3_2,
};

#define RATE_INFO_BW_320 (RATE_INFO_BW_HE_RU + 1)
#define NL80211_RRF_NO_320MHZ 0
#endif /* CFG80211_VERSION < KERNEL_VERSION(5,18,0) */

#if LINUX_VERSION_IS_LESS(4,19,0)
/**
 * eth_hw_addr_set - Assign Ethernet address to a net_device
 * @dev: pointer to net_device structure
 * @addr: address to assign
 *
 * Assign given address to the net_device, addr_assign_type is not changed.
 */
static inline void eth_hw_addr_set(struct net_device *dev, const u8 *addr)
{
	ether_addr_copy(dev->dev_addr, addr);
}
#endif /* LINUX_VERSION_IS_LESS(5,15,0) */

#if CFG80211_VERSION < KERNEL_VERSION(5,16,0)
#define NL80211_BAND_LC	5
#endif

#if LINUX_VERSION_IS_LESS(5,16,0)
#define skb_ext_reset LINUX_BACKPORT(skb_get_dsfield)
static inline int skb_get_dsfield(struct sk_buff *skb)
{
	switch (skb_protocol(skb, true)) {
	case cpu_to_be16(ETH_P_IP):
		if (!pskb_network_may_pull(skb, sizeof(struct iphdr)))
			break;
		return ipv4_get_dsfield(ip_hdr(skb));

	case cpu_to_be16(ETH_P_IPV6):
		if (!pskb_network_may_pull(skb, sizeof(struct ipv6hdr)))
			break;
		return ipv6_get_dsfield(ipv6_hdr(skb));
	}

	return -1;
}
#endif

#ifndef NET_DEVICE_PATH_STACK_MAX
enum net_device_path_type {
	DEV_PATH_ETHERNET = 0,
	DEV_PATH_VLAN,
	DEV_PATH_BRIDGE,
	DEV_PATH_PPPOE,
	DEV_PATH_DSA,
};

struct net_device_path {
	enum net_device_path_type	type;
	const struct net_device		*dev;
	union {
		struct {
			u16		id;
			__be16		proto;
			u8		h_dest[ETH_ALEN];
		} encap;
		struct {
			enum {
				DEV_PATH_BR_VLAN_KEEP,
				DEV_PATH_BR_VLAN_TAG,
				DEV_PATH_BR_VLAN_UNTAG,
				DEV_PATH_BR_VLAN_UNTAG_HW,
			}		vlan_mode;
			u16		vlan_id;
			__be16		vlan_proto;
		} bridge;
		struct {
			int port;
			u16 proto;
		} dsa;
	};
};

#define NET_DEVICE_PATH_STACK_MAX	5
#define NET_DEVICE_PATH_VLAN_MAX	2

struct net_device_path_stack {
	int			num_paths;
	struct net_device_path	path[NET_DEVICE_PATH_STACK_MAX];
};

struct net_device_path_ctx {
	const struct net_device *dev;
	const u8		*daddr;

	int			num_vlans;
	struct {
		u16		id;
		__be16		proto;
	} vlan[NET_DEVICE_PATH_VLAN_MAX];
};
#endif /* NET_DEVICE_PATH_STACK_MAX */

#ifndef memset_after
#define memset_after(obj, v, member)					\
({									\
	u8 *__ptr = (u8 *)(obj);					\
	typeof(v) __val = (v);						\
	memset(__ptr + offsetofend(typeof(*(obj)), member), __val,	\
	       sizeof(*(obj)) - offsetofend(typeof(*(obj)), member));	\
})
#endif

#ifndef memset_startat
#define memset_startat(obj, v, member)					\
({									\
	u8 *__ptr = (u8 *)(obj);					\
	typeof(v) __val = (v);						\
	memset(__ptr + offsetof(typeof(*(obj)), member), __val,		\
	       sizeof(*(obj)) - offsetof(typeof(*(obj)), member));	\
})
#endif

#if CFG80211_VERSION < KERNEL_VERSION(5,19,0)
struct cfg80211_rx_info {
	int freq;
	int sig_dbm;
	bool have_link_id;
	u8 link_id;
	const u8 *buf;
	size_t len;
	u32 flags;
	u64 rx_tstamp;
	u64 ack_tstamp;
};

static inline bool cfg80211_rx_mgmt_ext(struct wireless_dev *wdev,
					struct cfg80211_rx_info *info)
{
	return cfg80211_rx_mgmt(wdev, KHZ_TO_MHZ(info->freq), info->sig_dbm,
				info->buf, info->len, info->flags);
}

/**
 * struct cfg80211_tx_status - TX status for management frame information
 *
 * @cookie: Cookie returned by cfg80211_ops::mgmt_tx()
 * @tx_tstamp: hardware TX timestamp in nanoseconds
 * @ack_tstamp: hardware ack RX timestamp in nanoseconds
 * @buf: Management frame (header + body)
 * @len: length of the frame data
 * @ack: Whether frame was acknowledged
 */
struct cfg80211_tx_status {
	u64 cookie;
	u64 tx_tstamp;
	u64 ack_tstamp;
	const u8 *buf;
	size_t len;
	bool ack;
};

static inline
void cfg80211_mgmt_tx_status_ext(struct wireless_dev *wdev,
				 struct cfg80211_tx_status *status, gfp_t gfp)
{
	cfg80211_mgmt_tx_status(wdev, status->cookie, status->buf, status->len,
				status->ack, gfp);
}
#endif /* CFG80211_VERSION < KERNEL_VERSION(5,19,0) */

#if CFG80211_VERSION < KERNEL_VERSION(6,4,0)
struct cfg80211_set_hw_timestamp {
	const u8 *macaddr;
	bool enable;
};
#define set_hw_timestamp_max_peers(hw, val)	do { } while (0)
#else
#define set_hw_timestamp_max_peers(hw, val)	(hw)->wiphy->hw_timestamp_max_peers = val
#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,0,0)
#define cfg80211_ch_switch_notify(dev, chandef, link_id, punct_bitmap) cfg80211_ch_switch_notify(dev, chandef)
#endif

#ifdef CONFIG_THERMAL
#if LINUX_VERSION_IS_GEQ(5,10,0) && LINUX_VERSION_IS_LESS(6,0,0)
#include <linux/thermal.h>
struct thermal_trip {
	int temperature;
	int hysteresis;
	enum thermal_trip_type type;
};
#endif
#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,0,0)
static inline enum ieee80211_rate_flags
ieee80211_chanwidth_rate_flags(enum nl80211_chan_width width)
{
	switch (width) {
	case NL80211_CHAN_WIDTH_5:
		return IEEE80211_RATE_SUPPORTS_5MHZ;
	case NL80211_CHAN_WIDTH_10:
		return IEEE80211_RATE_SUPPORTS_10MHZ;
	default:
		break;
	}
	return 0;
}

#define link_sta_params_link_id(params)	-1
#define link_sta_params_link_mac(params)	NULL
#define WIPHY_FLAG_SUPPORTS_MLO 0
#define cfg80211_disassoc_ap_addr(req)	((req)->bss->bssid)

struct iwl7000_cfg80211_rx_assoc_resp {
	struct cfg80211_bss *bss;
	const u8 *buf;
	size_t len;
	const u8 *req_ies;
	size_t req_ies_len;
	int uapsd_queues;
	const u8 *ap_mld_addr;
	struct {
		u8 addr[ETH_ALEN];
		struct cfg80211_bss *bss;
		u16 status;
	} links[IEEE80211_MLD_MAX_NUM_LINKS];
};

static inline void
iwl7000_cfg80211_rx_assoc_resp(struct net_device *dev,
			       struct iwl7000_cfg80211_rx_assoc_resp *data)
{
	WARN_ON(data->ap_mld_addr);
	if (WARN_ON(!data->links[0].bss))
		return;

	cfg80211_rx_assoc_resp(dev, data->links[0].bss, data->buf, data->len,
			       data->uapsd_queues
#if CFG80211_VERSION >= KERNEL_VERSION(5,1,0)
			       , data->req_ies, data->req_ies_len
#endif
			      );
}

#define cfg80211_rx_assoc_resp iwl7000_cfg80211_rx_assoc_resp

struct cfg80211_assoc_failure {
	const u8 *ap_mld_addr;
	struct cfg80211_bss *bss[IEEE80211_MLD_MAX_NUM_LINKS];
	bool timeout;
};

static inline void cfg80211_assoc_failure(struct net_device *dev,
					  struct cfg80211_assoc_failure *data)
{
	int i;

	WARN_ON(!data->bss[0]);
	WARN_ON(data->ap_mld_addr);

	for (i = 1; i < ARRAY_SIZE(data->bss); i++)
		WARN_ON(data->bss[i]);

	if (data->timeout)
		cfg80211_assoc_timeout(dev, data->bss[0]);
	else
		cfg80211_abandon_assoc(dev, data->bss[0]);
}

#if CFG80211_VERSION >= KERNEL_VERSION(5,8,0)
static inline int
bp_ieee80211_tx_control_port(struct wiphy *wiphy, struct net_device *dev,
			     const u8 *buf, size_t len,
			     const u8 *dest, __be16 proto, bool unencrypted,
			     u64 *cookie)
{
	return ieee80211_tx_control_port(wiphy, dev, buf, len, dest, proto,
					 unencrypted, -1, cookie);
}
#endif /* >= 5.8 */

#define cfg80211_req_ap_mld_addr(req)		NULL

static inline const struct wiphy_iftype_ext_capab *
cfg80211_get_iftype_ext_capa(struct wiphy *wiphy, enum nl80211_iftype type)
{
	int i;

	for (i = 0; i < wiphy->num_iftype_ext_capab; i++) {
		if (wiphy->iftype_ext_capab[i].iftype == type)
			return &wiphy->iftype_ext_capab[i];
	}

	return NULL;
}
#define cfg80211_ext_capa_eml_capabilities(ift_ext_capa)	0
#define cfg80211_ext_capa_set_eml_capabilities(ift_ext_capa, v)	do {} while (0)
#define cfg80211_ext_capa_mld_capa_and_ops(ift_ext_capa)	0
#define cfg80211_mgmt_tx_params_link_id(params)			-1
#define cfg80211_mgmt_tx_params_link_id_mask(params)		0
#define link_sta_params_mld_mac(params)		NULL
#define cfg80211_beacon_data_link_id(params)	0
#define cfg80211_req_link_bss(req, link)	NULL
#define cfg80211_req_link_id(req)		-1
#define cfg80211_req_link_elems_len(req, link)	0

#ifdef CONFIG_THERMAL
#include <linux/thermal.h>
struct thermal_zone_device *
thermal_zone_device_register_with_trips(const char *type,
					struct thermal_trip *trips,
					int num_trips, int mask, void *devdata,
					struct thermal_zone_device_ops *ops,
					struct thermal_zone_params *tzp, int passive_delay,
					int polling_delay);
#endif /* CONFIG_THERMAL */

#else /* CFG80211 < 6.0 */
#define link_sta_params_link_id(params) ((params)->link_sta_params.link_id)
#define link_sta_params_link_mac(params) ((params)->link_sta_params.link_mac)
#define cfg80211_disassoc_ap_addr(req)	((req)->ap_addr)
#define cfg80211_req_ap_mld_addr(req)		((req)->ap_mld_addr)
#define cfg80211_ext_capa_eml_capabilities(ift_ext_capa)	(ift_ext_capa)->eml_capabilities
#define cfg80211_ext_capa_set_eml_capabilities(ift_ext_capa, v)	(ift_ext_capa)->eml_capabilities = (v)
#define cfg80211_ext_capa_mld_capa_and_ops(ift_ext_capa)	(ift_ext_capa)->mld_capa_and_ops
#define cfg80211_mgmt_tx_params_link_id(params)	((params)->link_id)
#define cfg80211_mgmt_tx_params_link_id_mask(params) BIT((params)->link_id)
#define link_sta_params_mld_mac(params)		(params->link_sta_params.mld_mac)
#define cfg80211_beacon_data_link_id(params)	(params->link_id)
#define cfg80211_req_link_bss(req, link)	((req)->links[link].bss)
#define cfg80211_req_link_id(req)		((req)->link_id)
#define cfg80211_req_link_elems_len(req, link)	((req)->links[link].elems_len)
#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,4,0) && \
    !defined(cfg80211_rx_control_port)
static inline bool
iwl7000_cfg80211_rx_control_port(struct net_device *dev, struct sk_buff *skb,
				 bool unencrypted, int link_id)
{
	return cfg80211_rx_control_port(dev, skb, unencrypted);
}

#define cfg80211_rx_control_port iwl7000_cfg80211_rx_control_port
#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,1,0)
#define cfg80211_txq_params_link_id(params)			0
#define cfg80211_bss_params_link_id(params)			-1
#else
#define cfg80211_txq_params_link_id(params)			(params)->link_id
#define cfg80211_bss_params_link_id(params)			((params)->link_id)
#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,1,0)
static inline void backport_netif_napi_add(struct net_device *dev,
					   struct napi_struct *napi,
					   int (*poll)(struct napi_struct *, int))
{
	netif_napi_add(dev, napi, poll, NAPI_POLL_WEIGHT);
}
#define netif_napi_add LINUX_BACKPORT(netif_napi_add)
#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,8,0)
#define cfg80211_scan_request_tsf_report_link_id(req)             -1
#define cfg80211_scan_request_set_tsf_report_link_id(req, ink_id) do {} while (0)
#define cfg80211_scan_request_check_tsf_report_link_id(req, mask) false
#else
#define cfg80211_scan_request_tsf_report_link_id(req)	  (req)->tsf_report_link_id
#define cfg80211_scan_request_set_tsf_report_link_id(req, v) (req)->tsf_report_link_id = (v)
#define cfg80211_scan_request_check_tsf_report_link_id(req, mask) ((mask) & BIT((req)->tsf_report_link_id))
#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,5,0)
#define cfg80211_req_link_disabled(req, link)	0
#define NL80211_RRF_NO_EHT 0
static inline void
cfg80211_links_removed(struct net_device *dev, u16 removed_links)
{
}
#else
#define cfg80211_req_link_disabled(req, link)	((req)->links[link].disabled)
#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,4,0)
#ifdef CONFIG_THERMAL
#include <linux/thermal.h>
void *thermal_zone_device_priv(struct thermal_zone_device *tzd);
#endif
#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,1,0)
bool cfg80211_valid_disable_subchannel_bitmap(u16 *bitmap,
					      struct cfg80211_chan_def *chandef);
#define ieee80211_amsdu_to_8023s(skb, list, addr, type, headroom, check_sa, check_da, mesh) \
	ieee80211_amsdu_to_8023s(skb, list, addr, type, headroom, check_sa, check_da)
#endif /* < 6.1  */

#if LINUX_VERSION_IS_LESS(6,4,0)
#define ieee80211_is_valid_amsdu LINUX_BACKPORT(ieee80211_is_valid_amsdu)
static inline bool ieee80211_is_valid_amsdu(struct sk_buff *skb, u8 mesh_hdr)
{
	return mesh_hdr == 0;
}

static inline void
LINUX_BACKPORT(kfree_skb_reason)(struct sk_buff *skb, u32 reason)
{
#if LINUX_VERSION_IS_LESS(5,17,0)
	dev_kfree_skb(skb);
#else
	kfree_skb_reason(skb, SKB_DROP_REASON_NOT_SPECIFIED);
#endif
}
#define kfree_skb_reason LINUX_BACKPORT(kfree_skb_reason)
#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,7,0)
void ieee80211_fragment_element(struct sk_buff *skb, u8 *len_pos, u8 frag_id);

static inline void
_ieee80211_set_sband_iftype_data(struct ieee80211_supported_band *sband,
				 const struct ieee80211_sband_iftype_data *iftd,
				 u16 n_iftd)
{
#if CFG80211_VERSION >= KERNEL_VERSION(4,19,0)
	sband->iftype_data = iftd;
	sband->n_iftype_data = n_iftd;
#endif
}

#if CFG80211_VERSION < KERNEL_VERSION(6,1,0)
struct wiphy_work;
typedef void (*wiphy_work_func_t)(struct wiphy *, struct wiphy_work *);

struct wiphy_work {
	struct list_head entry;
	wiphy_work_func_t func;
};

static inline void wiphy_work_init(struct wiphy_work *work,
				   wiphy_work_func_t func)
{
	INIT_LIST_HEAD(&work->entry);
	work->func = func;
}

struct wiphy_delayed_work {
	struct wiphy_work work;
	struct wiphy *wiphy;
	struct timer_list timer;
};
#endif

void wiphy_delayed_work_timer(struct timer_list *t);

#define wiphy_delayed_work_init LINUX_BACKPORT(wiphy_delayed_work_init)
static inline void wiphy_delayed_work_init(struct wiphy_delayed_work *dwork,
					   wiphy_work_func_t func)
{
	timer_setup(&dwork->timer, wiphy_delayed_work_timer, 0);
	wiphy_work_init(&dwork->work, func);
}

void wiphy_work_queue(struct wiphy *wiphy, struct wiphy_work *work);
void wiphy_work_cancel(struct wiphy *wiphy, struct wiphy_work *work);

void wiphy_delayed_work_queue(struct wiphy *wiphy,
			      struct wiphy_delayed_work *dwork,
			      unsigned long delay);
void wiphy_delayed_work_cancel(struct wiphy *wiphy,
			       struct wiphy_delayed_work *dwork);

void wiphy_work_flush(struct wiphy *wiphy, struct wiphy_work *work);
void wiphy_delayed_work_flush(struct wiphy *wiphy,
			      struct wiphy_delayed_work *work);

#if CFG80211_VERSION < KERNEL_VERSION(4,19,0)
#define for_each_sband_iftype_data(sband, i, iftd)	\
	for (; 0 ;)
#else
#define for_each_sband_iftype_data(sband, i, iftd)	\
	for (i = 0, iftd = &(sband)->iftype_data[i];	\
	     i < (sband)->n_iftype_data;		\
	     i++, iftd = &(sband)->iftype_data[i])
#endif /* CFG80211_VERSION < KERNEL_VERSION(4,19,0) */

/* older cfg80211 requires wdev to be locked */
#define sdata_lock_old_cfg80211(sdata) mutex_lock(&(sdata)->wdev.mtx)
#define sdata_unlock_old_cfg80211(sdata) mutex_unlock(&(sdata)->wdev.mtx)
#define WRAP_LOCKED(sym) wdev_locked_ ## sym

static inline void
WRAP_LOCKED(cfg80211_links_removed)(struct net_device *dev, u16 removed_links)
{
	mutex_lock(&dev->ieee80211_ptr->mtx);
	cfg80211_links_removed(dev, removed_links);
	mutex_unlock(&dev->ieee80211_ptr->mtx);
}
#define cfg80211_links_removed WRAP_LOCKED(cfg80211_links_removed)
#else
#define sdata_lock_old_cfg80211(sdata) do {} while (0)
#define sdata_unlock_old_cfg80211(sdata) do {} while (0)
#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,5,0)

#include <linux/leds.h>

static inline void backport_led_trigger_blink_oneshot(struct led_trigger *trigger,
						      unsigned long delay_on,
						      unsigned long delay_off,
						      int invert)
{
	led_trigger_blink_oneshot(trigger, &delay_on, &delay_off, invert);
}
#define led_trigger_blink_oneshot LINUX_BACKPORT(led_trigger_blink_oneshot)

static inline void backport_led_trigger_blink(struct led_trigger *trigger,
					      unsigned long delay_on,
					      unsigned long delay_off)
{
	led_trigger_blink(trigger, &delay_on, &delay_off);
}
#define led_trigger_blink LINUX_BACKPORT(led_trigger_blink)
#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,8,0)
static inline void cfg80211_schedule_channels_check(struct wireless_dev *wdev)
{
}
#define NL80211_EXT_FEATURE_DFS_CONCURRENT -1
#define NL80211_RRF_DFS_CONCURRENT 0

struct cfg80211_ttlm_params {
	u16 dlink[8];
	u16 ulink[8];
};

#if CFG80211_VERSION < KERNEL_VERSION(6, 8, 0)
#define NL80211_EXT_FEATURE_SPP_AMSDU_SUPPORT -1
#define NL80211_EXT_FEATURE_SECURE_LTF -1
#define ASSOC_REQ_SPP_AMSDU BIT(7)
#define NL80211_STA_FLAG_SPP_AMSDU 8
#endif

#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,7,0)
static inline u32
iwl7000_ieee80211_mandatory_rates(struct ieee80211_supported_band *sband)
{
	return ieee80211_mandatory_rates(sband, NL80211_BSS_CHAN_WIDTH_20);
}
#define ieee80211_mandatory_rates iwl7000_ieee80211_mandatory_rates
#endif

#if LINUX_VERSION_IS_LESS(6,7,0)
/**
 *	napi_schedule - schedule NAPI poll
 *	@n: NAPI context
 *
 * Schedule NAPI poll routine to be called if it is not already
 * running.
 * Return true if we schedule a NAPI or false if not.
 * Refer to napi_schedule_prep() for additional reason on why
 * a NAPI might not be scheduled.
 */
static inline bool LINUX_BACKPORT(napi_schedule)(struct napi_struct *n)
{
	if (napi_schedule_prep(n)) {
		__napi_schedule(n);
		return true;
	}

	return false;
}
#define napi_schedule LINUX_BACKPORT(napi_schedule)

#ifdef CONFIG_CFG80211_DEBUGFS
/**
 * wiphy_locked_debugfs_read - do a locked read in debugfs
 * @wiphy: the wiphy to use
 * @file: the file being read
 * @buf: the buffer to fill and then read from
 * @bufsize: size of the buffer
 * @userbuf: the user buffer to copy to
 * @count: read count
 * @ppos: read position
 * @handler: the read handler to call (under wiphy lock)
 * @data: additional data to pass to the read handler
 */
static inline
ssize_t wiphy_locked_debugfs_read(struct wiphy *wiphy, struct file *file,
				  char *buf, size_t bufsize,
				  char __user *userbuf, size_t count,
				  loff_t *ppos,
				  ssize_t (*handler)(struct wiphy *wiphy,
						     struct file *file,
						     char *buf,
						     size_t bufsize,
						     void *data),
				  void *data)
{
	ssize_t ret = -EINVAL;

#if CFG80211_VERSION >= KERNEL_VERSION(5,12,0)
	wiphy_lock(wiphy);
#else
	rtnl_lock();
#endif
	ret = handler(wiphy, file, buf, bufsize, data);
#if CFG80211_VERSION >= KERNEL_VERSION(5,12,0)
	wiphy_unlock(wiphy);
#else
	rtnl_unlock();
#endif

	if (ret >= 0)
		ret = simple_read_from_buffer(userbuf, count, ppos, buf, ret);

	return ret;
}

/**
 * wiphy_locked_debugfs_write - do a locked write in debugfs
 * @wiphy: the wiphy to use
 * @file: the file being written to
 * @buf: the buffer to copy the user data to
 * @bufsize: size of the buffer
 * @userbuf: the user buffer to copy from
 * @count: read count
 * @handler: the write handler to call (under wiphy lock)
 * @data: additional data to pass to the write handler
 */
static inline
ssize_t wiphy_locked_debugfs_write(struct wiphy *wiphy, struct file *file,
				   char *buf, size_t bufsize,
				   const char __user *userbuf, size_t count,
				   ssize_t (*handler)(struct wiphy *wiphy,
						      struct file *file,
						      char *buf,
						      size_t count,
						      void *data),
				   void *data)
{
	ssize_t ret;

	if (count >= sizeof(buf))
		return -E2BIG;

	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;
	buf[count] = '\0';

#if CFG80211_VERSION >= KERNEL_VERSION(5,12,0)
	wiphy_lock(wiphy);
#else
	rtnl_lock();
#endif
	ret = handler(wiphy, file, buf, bufsize, data);
#if CFG80211_VERSION >= KERNEL_VERSION(5,12,0)
	wiphy_unlock(wiphy);
#else
	rtnl_unlock();
#endif

	return ret;
}
#endif
#endif /* < 6.7.0 */

#if CFG80211_VERSION < KERNEL_VERSION(6,8,0)
int cfg80211_chandef_primary_freq(const struct cfg80211_chan_def *chandef,
				  enum nl80211_chan_width primary_width);
#endif /* cfg < 6.8 */
