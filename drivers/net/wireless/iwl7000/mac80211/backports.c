/*
 * Copyright(c) 2015 - 2017 Intel Deutschland GmbH
 * Copyright (C) 2018, 2020, 2022-2023 Intel Corporation
 *
 * Backport functionality introduced in Linux 4.4.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "mac80211-exp.h"

#include <linux/export.h>
#include <linux/if_vlan.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <net/ip.h>
#include <asm/unaligned.h>
#include <linux/device.h>

#if CFG80211_VERSION < KERNEL_VERSION(5,18,0)
static unsigned int __ieee80211_get_mesh_hdrlen(u8 flags)
{
	int ae = flags & MESH_FLAGS_AE;
	/* 802.11-2012, 8.2.4.7.3 */
	switch (ae) {
	default:
	case 0:
		return 6;
	case MESH_FLAGS_AE_A4:
		return 12;
	case MESH_FLAGS_AE_A5_A6:
		return 18;
	}
}

int ieee80211_data_to_8023_exthdr(struct sk_buff *skb, struct ethhdr *ehdr,
				  const u8 *addr, enum nl80211_iftype iftype,
				  u8 data_offset, bool is_amsdu)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;
	struct {
		u8 hdr[ETH_ALEN] __aligned(2);
		__be16 proto;
	} payload;
	struct ethhdr tmp;
	u16 hdrlen;
	u8 mesh_flags = 0;

	if (unlikely(!ieee80211_is_data_present(hdr->frame_control)))
		return -1;

	hdrlen = ieee80211_hdrlen(hdr->frame_control) + data_offset;
	if (skb->len < hdrlen + 8)
		return -1;

	/* convert IEEE 802.11 header + possible LLC headers into Ethernet
	 * header
	 * IEEE 802.11 address fields:
	 * ToDS FromDS Addr1 Addr2 Addr3 Addr4
	 *   0     0   DA    SA    BSSID n/a
	 *   0     1   DA    BSSID SA    n/a
	 *   1     0   BSSID SA    DA    n/a
	 *   1     1   RA    TA    DA    SA
	 */
	memcpy(tmp.h_dest, ieee80211_get_DA(hdr), ETH_ALEN);
	memcpy(tmp.h_source, ieee80211_get_SA(hdr), ETH_ALEN);

	if (iftype == NL80211_IFTYPE_MESH_POINT)
		skb_copy_bits(skb, hdrlen, &mesh_flags, 1);

	mesh_flags &= MESH_FLAGS_AE;

	switch (hdr->frame_control &
		cpu_to_le16(IEEE80211_FCTL_TODS | IEEE80211_FCTL_FROMDS)) {
	case cpu_to_le16(IEEE80211_FCTL_TODS):
		if (unlikely(iftype != NL80211_IFTYPE_AP &&
			     iftype != NL80211_IFTYPE_AP_VLAN &&
			     iftype != NL80211_IFTYPE_P2P_GO))
			return -1;
		break;
	case cpu_to_le16(IEEE80211_FCTL_TODS | IEEE80211_FCTL_FROMDS):
		if (unlikely(iftype != NL80211_IFTYPE_MESH_POINT &&
			     iftype != NL80211_IFTYPE_AP_VLAN &&
			     iftype != NL80211_IFTYPE_STATION))
			return -1;
		if (iftype == NL80211_IFTYPE_MESH_POINT) {
			if (mesh_flags == MESH_FLAGS_AE_A4)
				return -1;
			if (mesh_flags == MESH_FLAGS_AE_A5_A6) {
				skb_copy_bits(skb, hdrlen +
					offsetof(struct ieee80211s_hdr, eaddr1),
					tmp.h_dest, 2 * ETH_ALEN);
			}
			hdrlen += __ieee80211_get_mesh_hdrlen(mesh_flags);
		}
		break;
	case cpu_to_le16(IEEE80211_FCTL_FROMDS):
		if ((iftype != NL80211_IFTYPE_STATION &&
		     iftype != NL80211_IFTYPE_P2P_CLIENT &&
		     iftype != NL80211_IFTYPE_MESH_POINT) ||
		    (is_multicast_ether_addr(tmp.h_dest) &&
		     ether_addr_equal(tmp.h_source, addr)))
			return -1;
		if (iftype == NL80211_IFTYPE_MESH_POINT) {
			if (mesh_flags == MESH_FLAGS_AE_A5_A6)
				return -1;
			if (mesh_flags == MESH_FLAGS_AE_A4)
				skb_copy_bits(skb, hdrlen +
					offsetof(struct ieee80211s_hdr, eaddr1),
					tmp.h_source, ETH_ALEN);
			hdrlen += __ieee80211_get_mesh_hdrlen(mesh_flags);
		}
		break;
	case cpu_to_le16(0):
		if (iftype != NL80211_IFTYPE_ADHOC &&
		    iftype != NL80211_IFTYPE_STATION &&
		    iftype != NL80211_IFTYPE_OCB)
				return -1;
		break;
	}

	skb_copy_bits(skb, hdrlen, &payload, sizeof(payload));
	tmp.h_proto = payload.proto;

	if (likely((!is_amsdu && ether_addr_equal(payload.hdr, rfc1042_header) &&
		    tmp.h_proto != htons(ETH_P_AARP) &&
		    tmp.h_proto != htons(ETH_P_IPX)) ||
		   ether_addr_equal(payload.hdr, bridge_tunnel_header))) {
		/* remove RFC1042 or Bridge-Tunnel encapsulation and
		 * replace EtherType */
		hdrlen += ETH_ALEN + 2;
		skb_postpull_rcsum(skb, &payload, ETH_ALEN + 2);
	} else {
		tmp.h_proto = htons(skb->len - hdrlen);
	}

	pskb_pull(skb, hdrlen);

	if (!ehdr)
		ehdr = skb_push(skb, sizeof(struct ethhdr));
	memcpy(ehdr, &tmp, sizeof(tmp));

	return 0;
}
EXPORT_SYMBOL(/* don't auto-generate a rename */
	ieee80211_data_to_8023_exthdr);
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(5,18,0) */

#if CFG80211_VERSION < KERNEL_VERSION(5,8,0)
#include "mac80211/ieee80211_i.h"
#include "mac80211/driver-ops.h"

void ieee80211_mgmt_frame_register(struct wiphy *wiphy,
				   struct wireless_dev *wdev,
				   u16 frame_type, bool reg)
{
        struct ieee80211_local *local = wiphy_priv(wiphy);
        struct ieee80211_sub_if_data *sdata = IEEE80211_WDEV_TO_SUB_IF(wdev);

	switch (frame_type) {
	case IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_PROBE_REQ:
		if (reg) {
			local->probe_req_reg = true;
			sdata->vif.probe_req_reg = true;
		} else {
			if (local->probe_req_reg)
				local->probe_req_reg = false;

			if (sdata->vif.probe_req_reg)
				sdata->vif.probe_req_reg = false;
		}

		if (!local->open_count)
			break;

		if (ieee80211_sdata_running(sdata)) {
			if (sdata->vif.probe_req_reg == 1)
				drv_config_iface_filter(local, sdata,
							FIF_PROBE_REQ,
							FIF_PROBE_REQ);
			else if (sdata->vif.probe_req_reg == 0)
				drv_config_iface_filter(local, sdata, 0,
							FIF_PROBE_REQ);
		}

                ieee80211_configure_filter(local);
		break;
	default:
		break;
	}
}
#endif /* < 5.8 */

#ifdef CONFIG_THERMAL
#if CFG80211_VERSION < KERNEL_VERSION(6,0,0)
struct thermal_zone_device *
thermal_zone_device_register_with_trips(const char *type,
					struct thermal_trip *trips,
					int num_trips, int mask, void *devdata,
					struct thermal_zone_device_ops *ops,
					struct thermal_zone_params *tzp, int passive_delay,
					int polling_delay)
{
	return thermal_zone_device_register(type, num_trips, mask, devdata, ops, tzp,
					    passive_delay, polling_delay);
}
EXPORT_SYMBOL_GPL(thermal_zone_device_register_with_trips);
#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,4,0)
void *thermal_zone_device_priv(struct thermal_zone_device *tzd)
{
	return tzd->devdata;
}
EXPORT_SYMBOL_GPL(thermal_zone_device_priv);
#endif /* < 6.4 */
#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,1,0)
struct ieee80211_per_bw_puncturing_values {
	u8 len;
	const u16 *valid_values;
};

static const u16 puncturing_values_80mhz[] = {
	0x8, 0x4, 0x2, 0x1
};

static const u16 puncturing_values_160mhz[] = {
	 0x80, 0x40, 0x20, 0x10, 0x8, 0x4, 0x2, 0x1, 0xc0, 0x30, 0xc, 0x3
};

static const u16 puncturing_values_320mhz[] = {
	0xc000, 0x3000, 0xc00, 0x300, 0xc0, 0x30, 0xc, 0x3, 0xf000, 0xf00,
	0xf0, 0xf, 0xfc00, 0xf300, 0xf0c0, 0xf030, 0xf00c, 0xf003, 0xc00f,
	0x300f, 0xc0f, 0x30f, 0xcf, 0x3f
};

#define IEEE80211_PER_BW_VALID_PUNCTURING_VALUES(_bw) \
	{ \
		.len = ARRAY_SIZE(puncturing_values_ ## _bw ## mhz), \
		.valid_values = puncturing_values_ ## _bw ## mhz \
	}

static const struct ieee80211_per_bw_puncturing_values per_bw_puncturing[] = {
	IEEE80211_PER_BW_VALID_PUNCTURING_VALUES(80),
	IEEE80211_PER_BW_VALID_PUNCTURING_VALUES(160),
	IEEE80211_PER_BW_VALID_PUNCTURING_VALUES(320)
};

static bool
ieee80211_valid_disable_subchannel_bitmap(u16 *bitmap, enum nl80211_chan_width bw)
{
	u32 idx, i;

	switch((int)bw) {
	case NL80211_CHAN_WIDTH_80:
		idx = 0;
		break;
	case NL80211_CHAN_WIDTH_160:
		idx = 1;
		break;
	case NL80211_CHAN_WIDTH_320:
		idx = 2;
		break;
	default:
		*bitmap = 0;
		break;
	}

	if (!*bitmap)
		return true;

	for (i = 0; i < per_bw_puncturing[idx].len; i++)
		if (per_bw_puncturing[idx].valid_values[i] == *bitmap)
			return true;

	return false;
}

bool cfg80211_valid_disable_subchannel_bitmap(u16 *bitmap,
					      struct cfg80211_chan_def *chandef)
{
	return ieee80211_valid_disable_subchannel_bitmap(bitmap, chandef->width);
}

#endif /* < 6.3  */

#if CFG80211_VERSION < KERNEL_VERSION(6,7,0)
#include "mac80211/ieee80211_i.h"

static void cfg80211_wiphy_work(struct work_struct *work)
{
	struct ieee80211_local *local;
	struct wiphy_work *wk;

	local = container_of(work, struct ieee80211_local, wiphy_work);

#if CFG80211_VERSION < KERNEL_VERSION(5,12,0)
	rtnl_lock();
#else
	wiphy_lock(local->hw.wiphy);
#endif
	if (local->suspended)
		goto out;

	spin_lock_irq(&local->wiphy_work_lock);
	wk = list_first_entry_or_null(&local->wiphy_work_list,
				      struct wiphy_work, entry);
	if (wk) {
		list_del_init(&wk->entry);
		if (!list_empty(&local->wiphy_work_list))
			schedule_work(work);
		spin_unlock_irq(&local->wiphy_work_lock);

		wk->func(local->hw.wiphy, wk);
	} else {
		spin_unlock_irq(&local->wiphy_work_lock);
	}
out:
#if CFG80211_VERSION < KERNEL_VERSION(5,12,0)
	rtnl_unlock();
#else
	wiphy_unlock(local->hw.wiphy);
#endif
}

void wiphy_work_setup(struct ieee80211_local *local)
{
	INIT_WORK(&local->wiphy_work, cfg80211_wiphy_work);
	INIT_LIST_HEAD(&local->wiphy_work_list);
	spin_lock_init(&local->wiphy_work_lock);
}

void wiphy_work_flush(struct wiphy *wiphy, struct wiphy_work *end)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct ieee80211_local *local = hw_to_local(hw);
	struct wiphy_work *wk;
	unsigned long flags;
	int runaway_limit = 100;

	lockdep_assert_wiphy(wiphy);

	spin_lock_irqsave(&local->wiphy_work_lock, flags);
	while (!list_empty(&local->wiphy_work_list)) {
		struct wiphy_work *wk;

		wk = list_first_entry(&local->wiphy_work_list,
				      struct wiphy_work, entry);
		list_del_init(&wk->entry);
		spin_unlock_irqrestore(&local->wiphy_work_lock, flags);

		wk->func(local->hw.wiphy, wk);

		spin_lock_irqsave(&local->wiphy_work_lock, flags);

		if (wk == end)
			break;

		if (WARN_ON(--runaway_limit == 0))
			INIT_LIST_HEAD(&local->wiphy_work_list);
	}
	spin_unlock_irqrestore(&local->wiphy_work_lock, flags);
}
EXPORT_SYMBOL(wiphy_work_flush);

void wiphy_delayed_work_flush(struct wiphy *wiphy,
			      struct wiphy_delayed_work *dwork)
{
	del_timer_sync(&dwork->timer);
	wiphy_work_flush(wiphy, &dwork->work);
}
EXPORT_SYMBOL(wiphy_delayed_work_flush);

void wiphy_work_teardown(struct ieee80211_local *local)
{
#if CFG80211_VERSION < KERNEL_VERSION(5,12,0)
	rtnl_lock();
#else
	wiphy_lock(local->hw.wiphy);
#endif

	wiphy_work_flush(local->hw.wiphy, NULL);

#if CFG80211_VERSION < KERNEL_VERSION(5,12,0)
	rtnl_unlock();
#else
	wiphy_unlock(local->hw.wiphy);
#endif

	cancel_work_sync(&local->wiphy_work);
}

void wiphy_work_queue(struct wiphy *wiphy, struct wiphy_work *work)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct ieee80211_local *local = hw_to_local(hw);
	unsigned long flags;

	spin_lock_irqsave(&local->wiphy_work_lock, flags);
	if (list_empty(&work->entry))
		list_add_tail(&work->entry, &local->wiphy_work_list);
	spin_unlock_irqrestore(&local->wiphy_work_lock, flags);

	schedule_work(&local->wiphy_work);
}
EXPORT_SYMBOL_GPL(wiphy_work_queue);

void wiphy_work_cancel(struct wiphy *wiphy, struct wiphy_work *work)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct ieee80211_local *local = hw_to_local(hw);
	unsigned long flags;

#if CFG80211_VERSION < KERNEL_VERSION(5,12,0)
	ASSERT_RTNL();
#else
	lockdep_assert_held(&wiphy->mtx);
#endif

	spin_lock_irqsave(&local->wiphy_work_lock, flags);
	if (!list_empty(&work->entry))
		list_del_init(&work->entry);
	spin_unlock_irqrestore(&local->wiphy_work_lock, flags);
}
EXPORT_SYMBOL_GPL(wiphy_work_cancel);

void wiphy_delayed_work_timer(struct timer_list *t)
{
	struct wiphy_delayed_work *dwork = from_timer(dwork, t, timer);

	wiphy_work_queue(dwork->wiphy, &dwork->work);
}
EXPORT_SYMBOL(wiphy_delayed_work_timer);

void wiphy_delayed_work_queue(struct wiphy *wiphy,
			      struct wiphy_delayed_work *dwork,
			      unsigned long delay)
{
	if (!delay) {
		wiphy_work_queue(wiphy, &dwork->work);
		return;
	}

	dwork->wiphy = wiphy;
	mod_timer(&dwork->timer, jiffies + delay);
}
EXPORT_SYMBOL_GPL(wiphy_delayed_work_queue);

void wiphy_delayed_work_cancel(struct wiphy *wiphy,
			       struct wiphy_delayed_work *dwork)
{
	lockdep_assert_held(&wiphy->mtx);

	del_timer_sync(&dwork->timer);
	wiphy_work_cancel(wiphy, &dwork->work);
}
EXPORT_SYMBOL_GPL(wiphy_delayed_work_cancel);
#endif /* CFG80211_VERSION < KERNEL_VERSION(6,5,0) */

#if CFG80211_VERSION < KERNEL_VERSION(6,8,0)
static int nl80211_chan_width_to_mhz(enum nl80211_chan_width chan_width)
{
	int mhz;

	switch((int)chan_width) {
	case NL80211_CHAN_WIDTH_1:
		mhz = 1;
		break;
	case NL80211_CHAN_WIDTH_2:
		mhz = 2;
		break;
	case NL80211_CHAN_WIDTH_4:
		mhz = 4;
		break;
	case NL80211_CHAN_WIDTH_8:
		mhz = 8;
		break;
	case NL80211_CHAN_WIDTH_16:
		mhz = 16;
		break;
	case NL80211_CHAN_WIDTH_5:
		mhz = 5;
		break;
	case NL80211_CHAN_WIDTH_10:
		mhz = 10;
		break;
	case NL80211_CHAN_WIDTH_20:
	case NL80211_CHAN_WIDTH_20_NOHT:
		mhz = 20;
		break;
	case NL80211_CHAN_WIDTH_40:
		mhz = 40;
		break;
	case NL80211_CHAN_WIDTH_80P80:
	case NL80211_CHAN_WIDTH_80:
		mhz = 80;
		break;
	case NL80211_CHAN_WIDTH_160:
		mhz = 160;
		break;
	case NL80211_CHAN_WIDTH_320:
		mhz = 320;
		break;
	default:
		WARN_ON_ONCE(1);
		return -1;
	}
	return mhz;
}

static int cfg80211_chandef_get_width(const struct cfg80211_chan_def *c)
{
	return nl80211_chan_width_to_mhz(c->width);
}

int cfg80211_chandef_primary_freq(const struct cfg80211_chan_def *c,
				  enum nl80211_chan_width primary_chan_width)
{
	int pri_width = nl80211_chan_width_to_mhz(primary_chan_width);
	int width = cfg80211_chandef_get_width(c);
	u32 control = c->chan->center_freq;
	u32 center = c->center_freq1;

	if (WARN_ON_ONCE(pri_width < 0 || width < 0))
		return -1;

	/* not intended to be called this way, can't determine */
	if (WARN_ON_ONCE(pri_width > width))
		return -1;

	while (width > pri_width) {
		if (control > center)
			center += width / 4;
		else
			center -= width / 4;
		width /= 2;
	}

	return center;
}
#endif /* cfg < 6.8 */
