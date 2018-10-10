// SPDX-License-Identifier: GPL-2.0
/* drivers/android/staging/virt_wifi.c
 *
 * A fake implementation of cfg80211_ops that can be tacked on to an ethernet
 * net_device to make it appear as a wireless connection.
 *
 * Copyright (C) 2018 Google, Inc.
 *
 * Author: schuffelen@google.com
 */

#include <net/cfg80211.h>
#include <net/rtnetlink.h>
#include <linux/etherdevice.h>
#include <linux/module.h>

struct virt_wifi_priv {
	bool being_deleted;
	bool is_connected;
	bool netdev_is_up;
	struct net_device *netdev;
	struct cfg80211_scan_request *scan_request;
	u8 connect_requested_bss[ETH_ALEN];
	struct delayed_work scan_result;
	struct delayed_work connect;
	struct delayed_work disconnect;
	u16 disconnect_reason;
	u32 tx_packets;
	u32 tx_failed;
};

static struct ieee80211_channel channel_2ghz = {
	.band = NL80211_BAND_2GHZ,
	.center_freq = 2432,
	.hw_value = 2432,
	.max_power = 20,
};

static struct ieee80211_rate bitrates_2ghz[] = {
	{ .bitrate = 10 },
	{ .bitrate = 20 },
	{ .bitrate = 55 },
	{ .bitrate = 60 },
	{ .bitrate = 110 },
	{ .bitrate = 120 },
	{ .bitrate = 240 },
};

static struct ieee80211_supported_band band_2ghz = {
	.channels = &channel_2ghz,
	.bitrates = bitrates_2ghz,
	.band = NL80211_BAND_2GHZ,
	.n_channels = 1,
	.n_bitrates = ARRAY_SIZE(bitrates_2ghz),
	.ht_cap = {
		.ht_supported = true,
		.cap = IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
		       IEEE80211_HT_CAP_GRN_FLD |
		       IEEE80211_HT_CAP_SGI_20 |
		       IEEE80211_HT_CAP_SGI_40 |
		       IEEE80211_HT_CAP_DSSSCCK40,
		.ampdu_factor = 0x3,
		.ampdu_density = 0x6,
		.mcs = {
			.rx_mask = {0xff, 0xff},
			.tx_params = IEEE80211_HT_MCS_TX_DEFINED,
		},
	},
};

static struct ieee80211_channel channel_5ghz = {
	.band = NL80211_BAND_5GHZ,
	.center_freq = 5240,
	.hw_value = 5240,
	.max_power = 20,
};

static struct ieee80211_rate bitrates_5ghz[] = {
	{ .bitrate = 60 },
	{ .bitrate = 120 },
	{ .bitrate = 240 },
};

static struct ieee80211_supported_band band_5ghz = {
	.channels = &channel_5ghz,
	.bitrates = bitrates_5ghz,
	.band = NL80211_BAND_5GHZ,
	.n_channels = 1,
	.n_bitrates = ARRAY_SIZE(bitrates_5ghz),
	.ht_cap = {
		.ht_supported = true,
		.cap = IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
		       IEEE80211_HT_CAP_GRN_FLD |
		       IEEE80211_HT_CAP_SGI_20 |
		       IEEE80211_HT_CAP_SGI_40 |
		       IEEE80211_HT_CAP_DSSSCCK40,
		.ampdu_factor = 0x3,
		.ampdu_density = 0x6,
		.mcs = {
			.rx_mask = {0xff, 0xff},
			.tx_params = IEEE80211_HT_MCS_TX_DEFINED,
		},
	},
	.vht_cap = {
		.vht_supported = true,
		.cap = IEEE80211_VHT_CAP_MAX_MPDU_LENGTH_11454 |
		       IEEE80211_VHT_CAP_SUPP_CHAN_WIDTH_160_80PLUS80MHZ |
		       IEEE80211_VHT_CAP_RXLDPC |
		       IEEE80211_VHT_CAP_SHORT_GI_80 |
		       IEEE80211_VHT_CAP_SHORT_GI_160 |
		       IEEE80211_VHT_CAP_TXSTBC |
		       IEEE80211_VHT_CAP_RXSTBC_1 |
		       IEEE80211_VHT_CAP_RXSTBC_2 |
		       IEEE80211_VHT_CAP_RXSTBC_3 |
		       IEEE80211_VHT_CAP_RXSTBC_4 |
		       IEEE80211_VHT_CAP_MAX_A_MPDU_LENGTH_EXPONENT_MASK,
		.vht_mcs = {
			.rx_mcs_map = cpu_to_le16(
				IEEE80211_VHT_MCS_SUPPORT_0_9 << 0 |
				IEEE80211_VHT_MCS_SUPPORT_0_9 << 2 |
				IEEE80211_VHT_MCS_SUPPORT_0_9 << 4 |
				IEEE80211_VHT_MCS_SUPPORT_0_9 << 6 |
				IEEE80211_VHT_MCS_SUPPORT_0_9 << 8 |
				IEEE80211_VHT_MCS_SUPPORT_0_9 << 10 |
				IEEE80211_VHT_MCS_SUPPORT_0_9 << 12 |
				IEEE80211_VHT_MCS_SUPPORT_0_9 << 14),
			.tx_mcs_map = cpu_to_le16(
				IEEE80211_VHT_MCS_SUPPORT_0_9 << 0 |
				IEEE80211_VHT_MCS_SUPPORT_0_9 << 2 |
				IEEE80211_VHT_MCS_SUPPORT_0_9 << 4 |
				IEEE80211_VHT_MCS_SUPPORT_0_9 << 6 |
				IEEE80211_VHT_MCS_SUPPORT_0_9 << 8 |
				IEEE80211_VHT_MCS_SUPPORT_0_9 << 10 |
				IEEE80211_VHT_MCS_SUPPORT_0_9 << 12 |
				IEEE80211_VHT_MCS_SUPPORT_0_9 << 14),
		}
	},
};

/** Assigned at module init. Guaranteed locally-administered and unicast. */
static u8 fake_router_bssid[ETH_ALEN] __ro_after_init = {};

static int virt_wifi_scan(struct wiphy *wiphy,
			  struct cfg80211_scan_request *request)
{
	struct virt_wifi_priv *priv = wiphy_priv(wiphy);

	wiphy_debug(wiphy, "scan\n");

	if (priv->scan_request || priv->being_deleted)
		return -EBUSY;

	priv->scan_request = request;
	schedule_delayed_work(&priv->scan_result, HZ * 2);

	return 0;
}

static void virt_wifi_scan_result(struct work_struct *work)
{
	struct {
		u8 tag;
		u8 len;
		u8 ssid[11];
	} __packed ssid = {
		.tag = WLAN_EID_SSID, .len = 11, .ssid = "AndroidWifi",
	};
	struct cfg80211_bss *informed_bss;
	struct virt_wifi_priv *priv =
		container_of(work, struct virt_wifi_priv,
			     scan_result.work);
	struct wiphy *wiphy = priv_to_wiphy(priv);
	struct cfg80211_scan_info scan_info;

	informed_bss = cfg80211_inform_bss(wiphy, &channel_5ghz,
					   CFG80211_BSS_FTYPE_PRESP,
					   fake_router_bssid,
					   ktime_get_boot_ns(),
					   WLAN_CAPABILITY_ESS, 0,
					   (void *)&ssid, sizeof(ssid),
					   DBM_TO_MBM(60), GFP_KERNEL);
	cfg80211_put_bss(wiphy, informed_bss);

	rtnl_lock();
	if (priv->scan_request) {
		cfg80211_scan_done(priv->scan_request, &scan_info);
		priv->scan_request = NULL;
	}
	rtnl_unlock();
}

static int virt_wifi_connect(struct wiphy *wiphy,
			     struct net_device *netdev,
			     struct cfg80211_connect_params *sme)
{
	struct virt_wifi_priv *priv = wiphy_priv(wiphy);
	bool could_schedule;

	if (priv->being_deleted)
		return -EBUSY;

	could_schedule = schedule_delayed_work(&priv->connect, HZ * 2);
	if (!could_schedule)
		return -EBUSY;

	if (sme->bssid)
		ether_addr_copy(priv->connect_requested_bss, sme->bssid);
	else
		eth_zero_addr(priv->connect_requested_bss);

	wiphy_debug(wiphy, "connect\n");

	return 0;
}

static void virt_wifi_connect_complete(struct work_struct *work)
{
	struct virt_wifi_priv *priv =
		container_of(work, struct virt_wifi_priv, connect.work);
	u8 *requested_bss = priv->connect_requested_bss;
	bool has_addr = !is_zero_ether_addr(requested_bss);
	bool right_addr = ether_addr_equal(requested_bss, fake_router_bssid);
	u16 status = WLAN_STATUS_SUCCESS;

	rtnl_lock();
	if (!priv->netdev_is_up || (has_addr && !right_addr))
		status = WLAN_STATUS_UNSPECIFIED_FAILURE;
	else
		priv->is_connected = true;

	cfg80211_connect_result(priv->netdev, requested_bss, NULL, 0, NULL, 0,
				status, GFP_KERNEL);
	rtnl_unlock();
	netif_carrier_on(priv->netdev);
}

static int virt_wifi_disconnect(struct wiphy *wiphy, struct net_device *netdev,
				u16 reason_code)
{
	struct virt_wifi_priv *priv = wiphy_priv(wiphy);
	bool could_schedule;

	if (priv->being_deleted)
		return -EBUSY;

	wiphy_debug(wiphy, "disconnect\n");
	could_schedule = schedule_delayed_work(&priv->disconnect, HZ * 2);
	if (!could_schedule)
		return -EBUSY;
	priv->disconnect_reason = reason_code;
	return 0;
}

static void virt_wifi_disconnect_complete(struct work_struct *work)
{
	struct virt_wifi_priv *priv =
		container_of(work, struct virt_wifi_priv, disconnect.work);

	cfg80211_disconnected(priv->netdev, priv->disconnect_reason, NULL, 0,
			      true, GFP_KERNEL);
	priv->is_connected = false;
	netif_carrier_off(priv->netdev);
}

static int virt_wifi_get_station(
		struct wiphy *wiphy,
		struct net_device *dev,
		const u8 *mac,
		struct station_info *sinfo)
{
	struct virt_wifi_priv *priv = wiphy_priv(wiphy);

	wiphy_debug(wiphy, "get_station\n");

	if (!ether_addr_equal(mac, fake_router_bssid))
		return -ENOENT;

	sinfo->filled = BIT_ULL(NL80211_STA_INFO_TX_PACKETS) |
		BIT_ULL(NL80211_STA_INFO_TX_FAILED) |
		BIT_ULL(NL80211_STA_INFO_SIGNAL) |
		BIT_ULL(NL80211_STA_INFO_TX_BITRATE);
	sinfo->tx_packets = priv->tx_packets;
	sinfo->tx_failed = priv->tx_failed;
	sinfo->signal = -60;
	sinfo->txrate = (struct rate_info) {
		.legacy = 10, /* units are 100kbit/s */
	};
	return 0;
}

static int virt_wifi_dump_station(
		struct wiphy *wiphy,
		struct net_device *dev,
		int idx,
		u8 *mac,
		struct station_info *sinfo)
{
	struct virt_wifi_priv *priv = wiphy_priv(wiphy);

	wiphy_debug(wiphy, "dump_station\n");

	if (idx != 0 || !priv->is_connected)
		return -ENOENT;

	ether_addr_copy(mac, fake_router_bssid);
	return virt_wifi_get_station(wiphy, dev, fake_router_bssid, sinfo);
}

static const struct cfg80211_ops virt_wifi_cfg80211_ops = {
	.scan = virt_wifi_scan,

	.connect = virt_wifi_connect,
	.disconnect = virt_wifi_disconnect,

	.get_station = virt_wifi_get_station,
	.dump_station = virt_wifi_dump_station,
};

static struct wireless_dev *virt_wireless_dev(struct device *device,
					      struct net_device *netdev)
{
	struct wireless_dev *wdev;
	struct wiphy *wiphy;
	struct virt_wifi_priv *priv;

	wdev = kzalloc(sizeof(*wdev), GFP_KERNEL);

	if (!wdev)
		return ERR_PTR(-ENOMEM);

	wdev->iftype = NL80211_IFTYPE_STATION;
	wiphy = wiphy_new(&virt_wifi_cfg80211_ops, sizeof(*priv));

	if (!wiphy) {
		kfree(wdev);
		return ERR_PTR(-ENOMEM);
	}

	wdev->wiphy = wiphy;

	wiphy->max_scan_ssids = 4;
	wiphy->max_scan_ie_len = 1000;
	wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;

	wiphy->bands[NL80211_BAND_2GHZ] = &band_2ghz;
	wiphy->bands[NL80211_BAND_5GHZ] = &band_5ghz;
	wiphy->bands[NL80211_BAND_60GHZ] = NULL;

	/* Don't worry about frequency regulations. */
	wiphy->regulatory_flags = REGULATORY_WIPHY_SELF_MANAGED;
	wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION);
	set_wiphy_dev(wiphy, device);

	priv = wiphy_priv(wiphy);
	priv->netdev_is_up = false;
	priv->being_deleted = false;
	priv->is_connected = false;
	priv->scan_request = NULL;
	priv->netdev = netdev;
	INIT_DELAYED_WORK(&priv->scan_result, virt_wifi_scan_result);
	INIT_DELAYED_WORK(&priv->connect, virt_wifi_connect_complete);
	INIT_DELAYED_WORK(&priv->disconnect, virt_wifi_disconnect_complete);
	return wdev;
}

struct virt_wifi_netdev_priv {
	struct net_device *lowerdev;
	struct net_device *upperdev;
	struct work_struct register_wiphy_work;
};

static netdev_tx_t virt_wifi_start_xmit(struct sk_buff *skb,
					struct net_device *dev)
{
	struct virt_wifi_netdev_priv *priv = netdev_priv(dev);
	struct virt_wifi_priv *w_priv = wiphy_priv(dev->ieee80211_ptr->wiphy);

	w_priv->tx_packets++;
	if (!w_priv->is_connected) {
		w_priv->tx_failed++;
		return NET_XMIT_DROP;
	}

	skb->dev = priv->lowerdev;
	return dev_queue_xmit(skb);
}

/* Called with rtnl lock held. */
static int virt_wifi_net_device_open(struct net_device *dev)
{
	struct virt_wifi_priv *w_priv;

	if (!dev->ieee80211_ptr)
		return 0;
	w_priv = wiphy_priv(dev->ieee80211_ptr->wiphy);

	w_priv->netdev_is_up = true;
	return 0;
}

/* Called with rtnl lock held. */
static int virt_wifi_net_device_stop(struct net_device *dev)
{
	struct virt_wifi_priv *w_priv;
	struct cfg80211_scan_info scan_info = { .aborted = true };

	if (!dev->ieee80211_ptr)
		return 0;
	w_priv = wiphy_priv(dev->ieee80211_ptr->wiphy);
	w_priv->netdev_is_up = false;

	if (w_priv->scan_request) {
		cfg80211_scan_done(w_priv->scan_request, &scan_info);
		w_priv->scan_request = NULL;
	}

	return 0;
}

static const struct net_device_ops virt_wifi_ops = {
	.ndo_start_xmit = virt_wifi_start_xmit,
	.ndo_open = virt_wifi_net_device_open,
	.ndo_stop = virt_wifi_net_device_stop,
};

static void free_netdev_and_wiphy(struct net_device *dev)
{
	struct virt_wifi_netdev_priv *priv = netdev_priv(dev);
	struct virt_wifi_priv *w_priv;

	flush_work(&priv->register_wiphy_work);
	if (dev->ieee80211_ptr && !IS_ERR(dev->ieee80211_ptr)) {
		w_priv = wiphy_priv(dev->ieee80211_ptr->wiphy);
		w_priv->being_deleted = true;
		flush_delayed_work(&w_priv->scan_result);
		flush_delayed_work(&w_priv->connect);
		flush_delayed_work(&w_priv->disconnect);

		if (dev->ieee80211_ptr->wiphy->registered)
			wiphy_unregister(dev->ieee80211_ptr->wiphy);
		wiphy_free(dev->ieee80211_ptr->wiphy);
		kfree(dev->ieee80211_ptr);
	}
	free_netdev(dev);
}

static void virt_wifi_setup(struct net_device *dev)
{
	ether_setup(dev);
	dev->netdev_ops = &virt_wifi_ops;
	dev->priv_destructor = free_netdev_and_wiphy;
}

/* Called under rcu_read_lock() from netif_receive_skb */
static rx_handler_result_t virt_wifi_rx_handler(struct sk_buff **pskb)
{
	struct sk_buff *skb = *pskb;
	struct virt_wifi_netdev_priv *priv =
		rcu_dereference(skb->dev->rx_handler_data);
	struct virt_wifi_priv *w_priv =
		wiphy_priv(priv->upperdev->ieee80211_ptr->wiphy);

	if (!w_priv->is_connected)
		return RX_HANDLER_PASS;

	/* GFP_ATOMIC because this is a packet interrupt handler. */
	skb = skb_share_check(skb, GFP_ATOMIC);
	if (!skb) {
		dev_err(&priv->upperdev->dev, "can't skb_share_check\n");
		return RX_HANDLER_CONSUMED;
	}

	*pskb = skb;
	skb->dev = priv->upperdev;
	skb->pkt_type = PACKET_HOST;
	return RX_HANDLER_ANOTHER;
}

static void virt_wifi_register_wiphy(struct work_struct *work)
{
	struct virt_wifi_netdev_priv *priv =
		container_of(work, struct virt_wifi_netdev_priv,
			     register_wiphy_work);
	struct wireless_dev *wdev = priv->upperdev->ieee80211_ptr;
	int err;

	err = wiphy_register(wdev->wiphy);
	if (err < 0) {
		dev_err(&priv->upperdev->dev, "can't wiphy_register (%d)\n",
			err);

		/* Roll back the net_device, it's not going to do wifi. */
		rtnl_lock();
		err = rtnl_delete_link(priv->upperdev);
		rtnl_unlock();

		/* rtnl_delete_link should only throw errors if it's not a
		 * netlink device, but we know here it is already a virt_wifi
		 * device.
		 */
		WARN_ONCE(err, "rtnl_delete_link failed on a virt_wifi device");
	}
}

/* Called with rtnl lock held. */
static int virt_wifi_newlink(struct net *src_net, struct net_device *dev,
			     struct nlattr *tb[], struct nlattr *data[],
			     struct netlink_ext_ack *extack)
{
	struct virt_wifi_netdev_priv *priv = netdev_priv(dev);
	int err;

	if (!tb[IFLA_LINK])
		return -EINVAL;

	netif_carrier_off(dev);

	INIT_WORK(&priv->register_wiphy_work, virt_wifi_register_wiphy);
	priv->upperdev = dev;
	priv->lowerdev = __dev_get_by_index(src_net,
					    nla_get_u32(tb[IFLA_LINK]));

	if (!priv->lowerdev)
		return -ENODEV;
	if (!tb[IFLA_MTU])
		dev->mtu = priv->lowerdev->mtu;
	else if (dev->mtu > priv->lowerdev->mtu)
		return -EINVAL;

	err = netdev_rx_handler_register(priv->lowerdev, virt_wifi_rx_handler,
					 priv);
	if (err) {
		dev_err(&priv->lowerdev->dev,
			"can't netdev_rx_handler_register: %d\n", err);
		return err;
	}

	eth_hw_addr_inherit(dev, priv->lowerdev);
	netif_stacked_transfer_operstate(priv->lowerdev, dev);

	SET_NETDEV_DEV(dev, &priv->lowerdev->dev);
	dev->ieee80211_ptr = virt_wireless_dev(&priv->lowerdev->dev, dev);

	if (IS_ERR(dev->ieee80211_ptr)) {
		dev_err(&priv->lowerdev->dev, "can't init wireless: %ld\n",
			PTR_ERR(dev->ieee80211_ptr));
		err = PTR_ERR(dev->ieee80211_ptr);
		goto remove_handler;
	}

	err = register_netdevice(dev);
	if (err) {
		dev_err(&priv->lowerdev->dev, "can't register_netdevice: %d\n",
			err);
		goto free_wiphy;
	}

	err = netdev_upper_dev_link(priv->lowerdev, dev);
	if (err) {
		dev_err(&priv->lowerdev->dev, "can't netdev_upper_dev_link: %d\n",
			err);
		goto unregister_netdev;
	}

	/* The newlink callback is invoked while holding the rtnl lock, but
	 * register_wiphy wants to claim the rtnl lock itself.
	 */
	schedule_work(&priv->register_wiphy_work);

	return 0;
unregister_netdev:
	unregister_netdevice(dev);
free_wiphy: /* Safe whether or not netdev destructor runs. */
	wiphy_free(dev->ieee80211_ptr->wiphy);
	kfree(dev->ieee80211_ptr);
	dev->ieee80211_ptr = NULL;
remove_handler:
	netdev_rx_handler_unregister(priv->lowerdev);

	return err;
}

/* Called with rtnl lock held. */
static void virt_wifi_dellink(struct net_device *dev,
			      struct list_head *head)
{
	struct virt_wifi_netdev_priv *priv = netdev_priv(dev);

	netdev_rx_handler_unregister(priv->lowerdev);
	netdev_upper_dev_unlink(priv->lowerdev, dev);

	unregister_netdevice_queue(dev, head);

	/* Deleting the wiphy is handled in the netdev destructor. */
}

static struct rtnl_link_ops virt_wifi_link_ops = {
	.kind		= "virt_wifi",
	.setup		= virt_wifi_setup,
	.newlink	= virt_wifi_newlink,
	.dellink	= virt_wifi_dellink,
	.priv_size	= sizeof(struct virt_wifi_netdev_priv),
};

static int __init virt_wifi_init_module(void)
{
	/* Guaranteed to be locallly-administered and not multicast. */
	eth_random_addr(fake_router_bssid);
	return rtnl_link_register(&virt_wifi_link_ops);
}

static void __exit virt_wifi_cleanup_module(void)
{
	rtnl_link_unregister(&virt_wifi_link_ops);
}

module_init(virt_wifi_init_module);
module_exit(virt_wifi_cleanup_module);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Cody Schuffelen <schuffelen@google.com>");
MODULE_DESCRIPTION("Driver for a wireless wrapper of ethernet devices");
MODULE_ALIAS_RTNL_LINK("virt_wifi");
