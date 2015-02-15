/*
 * Marvell Wireless LAN device driver: CFG80211
 *
 * Copyright (C) 2011, Marvell International Ltd.
 *
 * This software file (the "File") is distributed by Marvell International
 * Ltd. under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

#include "cfg80211.h"
#include "main.h"

/*
 * This function maps the nl802.11 channel type into driver channel type.
 *
 * The mapping is as follows -
 *      NL80211_CHAN_NO_HT     -> IEEE80211_HT_PARAM_CHA_SEC_NONE
 *      NL80211_CHAN_HT20      -> IEEE80211_HT_PARAM_CHA_SEC_NONE
 *      NL80211_CHAN_HT40PLUS  -> IEEE80211_HT_PARAM_CHA_SEC_ABOVE
 *      NL80211_CHAN_HT40MINUS -> IEEE80211_HT_PARAM_CHA_SEC_BELOW
 *      Others                 -> IEEE80211_HT_PARAM_CHA_SEC_NONE
 */
static u8
mwifiex_cfg80211_channel_type_to_sec_chan_offset(enum nl80211_channel_type
						 channel_type)
{
	switch (channel_type) {
	case NL80211_CHAN_NO_HT:
	case NL80211_CHAN_HT20:
		return IEEE80211_HT_PARAM_CHA_SEC_NONE;
	case NL80211_CHAN_HT40PLUS:
		return IEEE80211_HT_PARAM_CHA_SEC_ABOVE;
	case NL80211_CHAN_HT40MINUS:
		return IEEE80211_HT_PARAM_CHA_SEC_BELOW;
	default:
		return IEEE80211_HT_PARAM_CHA_SEC_NONE;
	}
}

/*
 * This function checks whether WEP is set.
 */
static int
mwifiex_is_alg_wep(u32 cipher)
{
	switch (cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
	case WLAN_CIPHER_SUITE_WEP104:
		return 1;
	default:
		break;
	}

	return 0;
}

/*
 * This function retrieves the private structure from kernel wiphy structure.
 */
static void *mwifiex_cfg80211_get_priv(struct wiphy *wiphy)
{
	return (void *) (*(unsigned long *) wiphy_priv(wiphy));
}

/*
 * CFG802.11 operation handler to delete a network key.
 */
static int
mwifiex_cfg80211_del_key(struct wiphy *wiphy, struct net_device *netdev,
			 u8 key_index, bool pairwise, const u8 *mac_addr)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(netdev);

	if (mwifiex_set_encode(priv, NULL, 0, key_index, 1)) {
		wiphy_err(wiphy, "deleting the crypto keys\n");
		return -EFAULT;
	}

	wiphy_dbg(wiphy, "info: crypto keys deleted\n");
	return 0;
}

/*
 * CFG802.11 operation handler to set Tx power.
 */
static int
mwifiex_cfg80211_set_tx_power(struct wiphy *wiphy,
			      enum nl80211_tx_power_setting type,
			      int mbm)
{
	struct mwifiex_private *priv = mwifiex_cfg80211_get_priv(wiphy);
	struct mwifiex_power_cfg power_cfg;
	int dbm = MBM_TO_DBM(mbm);

	if (type == NL80211_TX_POWER_FIXED) {
		power_cfg.is_power_auto = 0;
		power_cfg.power_level = dbm;
	} else {
		power_cfg.is_power_auto = 1;
	}

	return mwifiex_set_tx_power(priv, &power_cfg);
}

/*
 * CFG802.11 operation handler to set Power Save option.
 *
 * The timeout value, if provided, is currently ignored.
 */
static int
mwifiex_cfg80211_set_power_mgmt(struct wiphy *wiphy,
				struct net_device *dev,
				bool enabled, int timeout)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(dev);
	u32 ps_mode;

	if (timeout)
		wiphy_dbg(wiphy,
			  "info: ignore timeout value for IEEE Power Save\n");

	ps_mode = enabled;

	return mwifiex_drv_set_power(priv, &ps_mode);
}

/*
 * CFG802.11 operation handler to set the default network key.
 */
static int
mwifiex_cfg80211_set_default_key(struct wiphy *wiphy, struct net_device *netdev,
				 u8 key_index, bool unicast,
				 bool multicast)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(netdev);

	/* Return if WEP key not configured */
	if (!priv->sec_info.wep_enabled)
		return 0;

	if (mwifiex_set_encode(priv, NULL, 0, key_index, 0)) {
		wiphy_err(wiphy, "set default Tx key index\n");
		return -EFAULT;
	}

	return 0;
}

/*
 * CFG802.11 operation handler to add a network key.
 */
static int
mwifiex_cfg80211_add_key(struct wiphy *wiphy, struct net_device *netdev,
			 u8 key_index, bool pairwise, const u8 *mac_addr,
			 struct key_params *params)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(netdev);

	if (mwifiex_set_encode(priv, params->key, params->key_len,
			       key_index, 0)) {
		wiphy_err(wiphy, "crypto keys added\n");
		return -EFAULT;
	}

	return 0;
}

/*
 * This function sends domain information to the firmware.
 *
 * The following information are passed to the firmware -
 *      - Country codes
 *      - Sub bands (first channel, number of channels, maximum Tx power)
 */
static int mwifiex_send_domain_info_cmd_fw(struct wiphy *wiphy)
{
	u8 no_of_triplet = 0;
	struct ieee80211_country_ie_triplet *t;
	u8 no_of_parsed_chan = 0;
	u8 first_chan = 0, next_chan = 0, max_pwr = 0;
	u8 i, flag = 0;
	enum ieee80211_band band;
	struct ieee80211_supported_band *sband;
	struct ieee80211_channel *ch;
	struct mwifiex_private *priv = mwifiex_cfg80211_get_priv(wiphy);
	struct mwifiex_adapter *adapter = priv->adapter;
	struct mwifiex_802_11d_domain_reg *domain_info = &adapter->domain_reg;

	/* Set country code */
	domain_info->country_code[0] = priv->country_code[0];
	domain_info->country_code[1] = priv->country_code[1];
	domain_info->country_code[2] = ' ';

	band = mwifiex_band_to_radio_type(adapter->config_bands);
	if (!wiphy->bands[band]) {
		wiphy_err(wiphy, "11D: setting domain info in FW\n");
		return -1;
	}

	sband = wiphy->bands[band];

	for (i = 0; i < sband->n_channels ; i++) {
		ch = &sband->channels[i];
		if (ch->flags & IEEE80211_CHAN_DISABLED)
			continue;

		if (!flag) {
			flag = 1;
			first_chan = (u32) ch->hw_value;
			next_chan = first_chan;
			max_pwr = ch->max_power;
			no_of_parsed_chan = 1;
			continue;
		}

		if (ch->hw_value == next_chan + 1 &&
		    ch->max_power == max_pwr) {
			next_chan++;
			no_of_parsed_chan++;
		} else {
			t = &domain_info->triplet[no_of_triplet];
			t->chans.first_channel = first_chan;
			t->chans.num_channels = no_of_parsed_chan;
			t->chans.max_power = max_pwr;
			no_of_triplet++;
			first_chan = (u32) ch->hw_value;
			next_chan = first_chan;
			max_pwr = ch->max_power;
			no_of_parsed_chan = 1;
		}
	}

	if (flag) {
		t = &domain_info->triplet[no_of_triplet];
		t->chans.first_channel = first_chan;
		t->chans.num_channels = no_of_parsed_chan;
		t->chans.max_power = max_pwr;
		no_of_triplet++;
	}

	domain_info->no_of_triplet = no_of_triplet;

	if (mwifiex_send_cmd_async(priv, HostCmd_CMD_802_11D_DOMAIN_INFO,
				   HostCmd_ACT_GEN_SET, 0, NULL)) {
		wiphy_err(wiphy, "11D: setting domain info in FW\n");
		return -1;
	}

	return 0;
}

/*
 * CFG802.11 regulatory domain callback function.
 *
 * This function is called when the regulatory domain is changed due to the
 * following reasons -
 *      - Set by driver
 *      - Set by system core
 *      - Set by user
 *      - Set bt Country IE
 */
static int mwifiex_reg_notifier(struct wiphy *wiphy,
				struct regulatory_request *request)
{
	struct mwifiex_private *priv = mwifiex_cfg80211_get_priv(wiphy);

	wiphy_dbg(wiphy, "info: cfg80211 regulatory domain callback for domain"
			" %c%c\n", request->alpha2[0], request->alpha2[1]);

	memcpy(priv->country_code, request->alpha2, sizeof(request->alpha2));

	switch (request->initiator) {
	case NL80211_REGDOM_SET_BY_DRIVER:
	case NL80211_REGDOM_SET_BY_CORE:
	case NL80211_REGDOM_SET_BY_USER:
		break;
		/* Todo: apply driver specific changes in channel flags based
		   on the request initiator if necessary. */
	case NL80211_REGDOM_SET_BY_COUNTRY_IE:
		break;
	}
	mwifiex_send_domain_info_cmd_fw(wiphy);

	return 0;
}

/*
 * This function sets the RF channel.
 *
 * This function creates multiple IOCTL requests, populates them accordingly
 * and issues them to set the band/channel and frequency.
 */
static int
mwifiex_set_rf_channel(struct mwifiex_private *priv,
		       struct ieee80211_channel *chan,
		       enum nl80211_channel_type channel_type)
{
	struct mwifiex_chan_freq_power cfp;
	u32 config_bands = 0;
	struct wiphy *wiphy = priv->wdev->wiphy;
	struct mwifiex_adapter *adapter = priv->adapter;

	if (chan) {
		/* Set appropriate bands */
		if (chan->band == IEEE80211_BAND_2GHZ) {
			if (channel_type == NL80211_CHAN_NO_HT)
				if (priv->adapter->config_bands == BAND_B ||
				    priv->adapter->config_bands == BAND_G)
					config_bands =
						priv->adapter->config_bands;
				else
					config_bands = BAND_B | BAND_G;
			else
				config_bands = BAND_B | BAND_G | BAND_GN;
		} else {
			if (channel_type == NL80211_CHAN_NO_HT)
				config_bands = BAND_A;
			else
				config_bands = BAND_AN | BAND_A;
		}

		if (!((config_bands | adapter->fw_bands) &
						~adapter->fw_bands)) {
			adapter->config_bands = config_bands;
			if (priv->bss_mode == NL80211_IFTYPE_ADHOC) {
				adapter->adhoc_start_band = config_bands;
				if ((config_bands & BAND_GN) ||
				    (config_bands & BAND_AN))
					adapter->adhoc_11n_enabled = true;
				else
					adapter->adhoc_11n_enabled = false;
			}
		}
		adapter->sec_chan_offset =
			mwifiex_cfg80211_channel_type_to_sec_chan_offset
			(channel_type);
		adapter->channel_type = channel_type;

		mwifiex_send_domain_info_cmd_fw(wiphy);
	}

	wiphy_dbg(wiphy, "info: setting band %d, chan offset %d, mode %d\n",
		  config_bands, adapter->sec_chan_offset, priv->bss_mode);
	if (!chan)
		return 0;

	memset(&cfp, 0, sizeof(cfp));
	cfp.freq = chan->center_freq;
	cfp.channel = ieee80211_frequency_to_channel(chan->center_freq);

	if (mwifiex_bss_set_channel(priv, &cfp))
		return -EFAULT;

	return mwifiex_drv_change_adhoc_chan(priv, cfp.channel);
}

/*
 * CFG802.11 operation handler to set channel.
 *
 * This function can only be used when station is not connected.
 */
static int
mwifiex_cfg80211_set_channel(struct wiphy *wiphy, struct net_device *dev,
			     struct ieee80211_channel *chan,
			     enum nl80211_channel_type channel_type)
{
	struct mwifiex_private *priv;

	if (dev)
		priv = mwifiex_netdev_get_priv(dev);
	else
		priv = mwifiex_cfg80211_get_priv(wiphy);

	if (priv->media_connected) {
		wiphy_err(wiphy, "This setting is valid only when station "
				"is not connected\n");
		return -EINVAL;
	}

	return mwifiex_set_rf_channel(priv, chan, channel_type);
}

/*
 * This function sets the fragmentation threshold.
 *
 * The fragmentation threshold value must lie between MWIFIEX_FRAG_MIN_VALUE
 * and MWIFIEX_FRAG_MAX_VALUE.
 */
static int
mwifiex_set_frag(struct mwifiex_private *priv, u32 frag_thr)
{
	int ret;

	if (frag_thr < MWIFIEX_FRAG_MIN_VALUE ||
	    frag_thr > MWIFIEX_FRAG_MAX_VALUE)
		return -EINVAL;

	/* Send request to firmware */
	ret = mwifiex_send_cmd_sync(priv, HostCmd_CMD_802_11_SNMP_MIB,
				    HostCmd_ACT_GEN_SET, FRAG_THRESH_I,
				    &frag_thr);

	return ret;
}

/*
 * This function sets the RTS threshold.

 * The rts value must lie between MWIFIEX_RTS_MIN_VALUE
 * and MWIFIEX_RTS_MAX_VALUE.
 */
static int
mwifiex_set_rts(struct mwifiex_private *priv, u32 rts_thr)
{
	if (rts_thr < MWIFIEX_RTS_MIN_VALUE || rts_thr > MWIFIEX_RTS_MAX_VALUE)
		rts_thr = MWIFIEX_RTS_MAX_VALUE;

	return mwifiex_send_cmd_sync(priv, HostCmd_CMD_802_11_SNMP_MIB,
				    HostCmd_ACT_GEN_SET, RTS_THRESH_I,
				    &rts_thr);
}

/*
 * CFG802.11 operation handler to set wiphy parameters.
 *
 * This function can be used to set the RTS threshold and the
 * Fragmentation threshold of the driver.
 */
static int
mwifiex_cfg80211_set_wiphy_params(struct wiphy *wiphy, u32 changed)
{
	struct mwifiex_private *priv = mwifiex_cfg80211_get_priv(wiphy);
	int ret = 0;

	if (changed & WIPHY_PARAM_RTS_THRESHOLD) {
		ret = mwifiex_set_rts(priv, wiphy->rts_threshold);
		if (ret)
			return ret;
	}

	if (changed & WIPHY_PARAM_FRAG_THRESHOLD)
		ret = mwifiex_set_frag(priv, wiphy->frag_threshold);

	return ret;
}

/*
 * CFG802.11 operation handler to change interface type.
 */
static int
mwifiex_cfg80211_change_virtual_intf(struct wiphy *wiphy,
				     struct net_device *dev,
				     enum nl80211_iftype type, u32 *flags,
				     struct vif_params *params)
{
	int ret;
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(dev);

	if (priv->bss_mode == type) {
		wiphy_warn(wiphy, "already set to required type\n");
		return 0;
	}

	priv->bss_mode = type;

	switch (type) {
	case NL80211_IFTYPE_ADHOC:
		dev->ieee80211_ptr->iftype = NL80211_IFTYPE_ADHOC;
		wiphy_dbg(wiphy, "info: setting interface type to adhoc\n");
		break;
	case NL80211_IFTYPE_STATION:
		dev->ieee80211_ptr->iftype = NL80211_IFTYPE_STATION;
		wiphy_dbg(wiphy, "info: setting interface type to managed\n");
		break;
	case NL80211_IFTYPE_UNSPECIFIED:
		dev->ieee80211_ptr->iftype = NL80211_IFTYPE_STATION;
		wiphy_dbg(wiphy, "info: setting interface type to auto\n");
		return 0;
	default:
		wiphy_err(wiphy, "unknown interface type: %d\n", type);
		return -EINVAL;
	}

	mwifiex_deauthenticate(priv, NULL);

	priv->sec_info.authentication_mode = NL80211_AUTHTYPE_OPEN_SYSTEM;

	ret = mwifiex_send_cmd_sync(priv, HostCmd_CMD_SET_BSS_MODE,
				    HostCmd_ACT_GEN_SET, 0, NULL);

	return ret;
}

/*
 * This function dumps the station information on a buffer.
 *
 * The following information are shown -
 *      - Total bytes transmitted
 *      - Total bytes received
 *      - Total packets transmitted
 *      - Total packets received
 *      - Signal quality level
 *      - Transmission rate
 */
static int
mwifiex_dump_station_info(struct mwifiex_private *priv,
			  struct station_info *sinfo)
{
	struct mwifiex_ds_get_signal signal;
	struct mwifiex_rate_cfg rate;
	int ret = 0;

	sinfo->filled = STATION_INFO_RX_BYTES | STATION_INFO_TX_BYTES |
		STATION_INFO_RX_PACKETS |
		STATION_INFO_TX_PACKETS
		| STATION_INFO_SIGNAL | STATION_INFO_TX_BITRATE;

	/* Get signal information from the firmware */
	memset(&signal, 0, sizeof(struct mwifiex_ds_get_signal));
	if (mwifiex_get_signal_info(priv, &signal)) {
		dev_err(priv->adapter->dev, "getting signal information\n");
		ret = -EFAULT;
	}

	if (mwifiex_drv_get_data_rate(priv, &rate)) {
		dev_err(priv->adapter->dev, "getting data rate\n");
		ret = -EFAULT;
	}

	/* Get DTIM period information from firmware */
	mwifiex_send_cmd_sync(priv, HostCmd_CMD_802_11_SNMP_MIB,
			      HostCmd_ACT_GEN_GET, DTIM_PERIOD_I,
			      &priv->dtim_period);

	/*
	 * Bit 0 in tx_htinfo indicates that current Tx rate is 11n rate. Valid
	 * MCS index values for us are 0 to 7.
	 */
	if ((priv->tx_htinfo & BIT(0)) && (priv->tx_rate < 8)) {
		sinfo->txrate.mcs = priv->tx_rate;
		sinfo->txrate.flags |= RATE_INFO_FLAGS_MCS;
		/* 40MHz rate */
		if (priv->tx_htinfo & BIT(1))
			sinfo->txrate.flags |= RATE_INFO_FLAGS_40_MHZ_WIDTH;
		/* SGI enabled */
		if (priv->tx_htinfo & BIT(2))
			sinfo->txrate.flags |= RATE_INFO_FLAGS_SHORT_GI;
	}

	sinfo->rx_bytes = priv->stats.rx_bytes;
	sinfo->tx_bytes = priv->stats.tx_bytes;
	sinfo->rx_packets = priv->stats.rx_packets;
	sinfo->tx_packets = priv->stats.tx_packets;
	sinfo->signal = priv->qual_level;
	/* bit rate is in 500 kb/s units. Convert it to 100kb/s units */
	sinfo->txrate.legacy = rate.rate * 5;

	if (priv->bss_mode == NL80211_IFTYPE_STATION) {
		sinfo->filled |= STATION_INFO_BSS_PARAM;
		sinfo->bss_param.flags = 0;
		if (priv->curr_bss_params.bss_descriptor.cap_info_bitmap &
						WLAN_CAPABILITY_SHORT_PREAMBLE)
			sinfo->bss_param.flags |=
					BSS_PARAM_FLAGS_SHORT_PREAMBLE;
		if (priv->curr_bss_params.bss_descriptor.cap_info_bitmap &
						WLAN_CAPABILITY_SHORT_SLOT_TIME)
			sinfo->bss_param.flags |=
					BSS_PARAM_FLAGS_SHORT_SLOT_TIME;
		sinfo->bss_param.dtim_period = priv->dtim_period;
		sinfo->bss_param.beacon_interval =
			priv->curr_bss_params.bss_descriptor.beacon_period;
	}

	return ret;
}

/*
 * CFG802.11 operation handler to get station information.
 *
 * This function only works in connected mode, and dumps the
 * requested station information, if available.
 */
static int
mwifiex_cfg80211_get_station(struct wiphy *wiphy, struct net_device *dev,
			     u8 *mac, struct station_info *sinfo)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(dev);

	if (!priv->media_connected)
		return -ENOENT;
	if (memcmp(mac, priv->cfg_bssid, ETH_ALEN))
		return -ENOENT;

	return mwifiex_dump_station_info(priv, sinfo);
}

/* Supported rates to be advertised to the cfg80211 */

static struct ieee80211_rate mwifiex_rates[] = {
	{.bitrate = 10, .hw_value = 2, },
	{.bitrate = 20, .hw_value = 4, },
	{.bitrate = 55, .hw_value = 11, },
	{.bitrate = 110, .hw_value = 22, },
	{.bitrate = 60, .hw_value = 12, },
	{.bitrate = 90, .hw_value = 18, },
	{.bitrate = 120, .hw_value = 24, },
	{.bitrate = 180, .hw_value = 36, },
	{.bitrate = 240, .hw_value = 48, },
	{.bitrate = 360, .hw_value = 72, },
	{.bitrate = 480, .hw_value = 96, },
	{.bitrate = 540, .hw_value = 108, },
};

/* Channel definitions to be advertised to cfg80211 */

static struct ieee80211_channel mwifiex_channels_2ghz[] = {
	{.center_freq = 2412, .hw_value = 1, },
	{.center_freq = 2417, .hw_value = 2, },
	{.center_freq = 2422, .hw_value = 3, },
	{.center_freq = 2427, .hw_value = 4, },
	{.center_freq = 2432, .hw_value = 5, },
	{.center_freq = 2437, .hw_value = 6, },
	{.center_freq = 2442, .hw_value = 7, },
	{.center_freq = 2447, .hw_value = 8, },
	{.center_freq = 2452, .hw_value = 9, },
	{.center_freq = 2457, .hw_value = 10, },
	{.center_freq = 2462, .hw_value = 11, },
	{.center_freq = 2467, .hw_value = 12, },
	{.center_freq = 2472, .hw_value = 13, },
	{.center_freq = 2484, .hw_value = 14, },
};

static struct ieee80211_supported_band mwifiex_band_2ghz = {
	.channels = mwifiex_channels_2ghz,
	.n_channels = ARRAY_SIZE(mwifiex_channels_2ghz),
	.bitrates = mwifiex_rates,
	.n_bitrates = ARRAY_SIZE(mwifiex_rates),
};

static struct ieee80211_channel mwifiex_channels_5ghz[] = {
	{.center_freq = 5040, .hw_value = 8, },
	{.center_freq = 5060, .hw_value = 12, },
	{.center_freq = 5080, .hw_value = 16, },
	{.center_freq = 5170, .hw_value = 34, },
	{.center_freq = 5190, .hw_value = 38, },
	{.center_freq = 5210, .hw_value = 42, },
	{.center_freq = 5230, .hw_value = 46, },
	{.center_freq = 5180, .hw_value = 36, },
	{.center_freq = 5200, .hw_value = 40, },
	{.center_freq = 5220, .hw_value = 44, },
	{.center_freq = 5240, .hw_value = 48, },
	{.center_freq = 5260, .hw_value = 52, },
	{.center_freq = 5280, .hw_value = 56, },
	{.center_freq = 5300, .hw_value = 60, },
	{.center_freq = 5320, .hw_value = 64, },
	{.center_freq = 5500, .hw_value = 100, },
	{.center_freq = 5520, .hw_value = 104, },
	{.center_freq = 5540, .hw_value = 108, },
	{.center_freq = 5560, .hw_value = 112, },
	{.center_freq = 5580, .hw_value = 116, },
	{.center_freq = 5600, .hw_value = 120, },
	{.center_freq = 5620, .hw_value = 124, },
	{.center_freq = 5640, .hw_value = 128, },
	{.center_freq = 5660, .hw_value = 132, },
	{.center_freq = 5680, .hw_value = 136, },
	{.center_freq = 5700, .hw_value = 140, },
	{.center_freq = 5745, .hw_value = 149, },
	{.center_freq = 5765, .hw_value = 153, },
	{.center_freq = 5785, .hw_value = 157, },
	{.center_freq = 5805, .hw_value = 161, },
	{.center_freq = 5825, .hw_value = 165, },
};

static struct ieee80211_supported_band mwifiex_band_5ghz = {
	.channels = mwifiex_channels_5ghz,
	.n_channels = ARRAY_SIZE(mwifiex_channels_5ghz),
	.bitrates = mwifiex_rates + 4,
	.n_bitrates = ARRAY_SIZE(mwifiex_rates) - 4,
};


/* Supported crypto cipher suits to be advertised to cfg80211 */

static const u32 mwifiex_cipher_suites[] = {
	WLAN_CIPHER_SUITE_WEP40,
	WLAN_CIPHER_SUITE_WEP104,
	WLAN_CIPHER_SUITE_TKIP,
	WLAN_CIPHER_SUITE_CCMP,
};

/*
 * CFG802.11 operation handler for setting bit rates.
 *
 * Function selects legacy bang B/G/BG from corresponding bitrates selection.
 * Currently only 2.4GHz band is supported.
 */
static int mwifiex_cfg80211_set_bitrate_mask(struct wiphy *wiphy,
				struct net_device *dev,
				const u8 *peer,
				const struct cfg80211_bitrate_mask *mask)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(dev);
	int index = 0, mode = 0, i;
	struct mwifiex_adapter *adapter = priv->adapter;

	/* Currently only 2.4GHz is supported */
	for (i = 0; i < mwifiex_band_2ghz.n_bitrates; i++) {
		/*
		 * Rates below 6 Mbps in the table are CCK rates; 802.11b
		 * and from 6 they are OFDM; 802.11G
		 */
		if (mwifiex_rates[i].bitrate == 60) {
			index = 1 << i;
			break;
		}
	}

	if (mask->control[IEEE80211_BAND_2GHZ].legacy < index) {
		mode = BAND_B;
	} else {
		mode = BAND_G;
		if (mask->control[IEEE80211_BAND_2GHZ].legacy % index)
			mode |=  BAND_B;
	}

	if (!((mode | adapter->fw_bands) & ~adapter->fw_bands)) {
		adapter->config_bands = mode;
		if (priv->bss_mode == NL80211_IFTYPE_ADHOC) {
			adapter->adhoc_start_band = mode;
			adapter->adhoc_11n_enabled = false;
		}
	}
	adapter->sec_chan_offset = IEEE80211_HT_PARAM_CHA_SEC_NONE;
	adapter->channel_type = NL80211_CHAN_NO_HT;

	wiphy_debug(wiphy, "info: device configured in 802.11%s%s mode\n",
		    (mode & BAND_B) ? "b" : "", (mode & BAND_G) ? "g" : "");

	return 0;
}

/*
 * CFG802.11 operation handler for disconnection request.
 *
 * This function does not work when there is already a disconnection
 * procedure going on.
 */
static int
mwifiex_cfg80211_disconnect(struct wiphy *wiphy, struct net_device *dev,
			    u16 reason_code)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(dev);

	if (mwifiex_deauthenticate(priv, NULL))
		return -EFAULT;

	wiphy_dbg(wiphy, "info: successfully disconnected from %pM:"
		" reason code %d\n", priv->cfg_bssid, reason_code);

	memset(priv->cfg_bssid, 0, ETH_ALEN);

	return 0;
}

/*
 * This function informs the CFG802.11 subsystem of a new IBSS.
 *
 * The following information are sent to the CFG802.11 subsystem
 * to register the new IBSS. If we do not register the new IBSS,
 * a kernel panic will result.
 *      - SSID
 *      - SSID length
 *      - BSSID
 *      - Channel
 */
static int mwifiex_cfg80211_inform_ibss_bss(struct mwifiex_private *priv)
{
	struct ieee80211_channel *chan;
	struct mwifiex_bss_info bss_info;
	struct cfg80211_bss *bss;
	int ie_len;
	u8 ie_buf[IEEE80211_MAX_SSID_LEN + sizeof(struct ieee_types_header)];
	enum ieee80211_band band;

	if (mwifiex_get_bss_info(priv, &bss_info))
		return -1;

	ie_buf[0] = WLAN_EID_SSID;
	ie_buf[1] = bss_info.ssid.ssid_len;

	memcpy(&ie_buf[sizeof(struct ieee_types_header)],
	       &bss_info.ssid.ssid, bss_info.ssid.ssid_len);
	ie_len = ie_buf[1] + sizeof(struct ieee_types_header);

	band = mwifiex_band_to_radio_type(priv->curr_bss_params.band);
	chan = __ieee80211_get_channel(priv->wdev->wiphy,
			ieee80211_channel_to_frequency(bss_info.bss_chan,
						       band));

	bss = cfg80211_inform_bss(priv->wdev->wiphy, chan,
				  bss_info.bssid, 0, WLAN_CAPABILITY_IBSS,
				  0, ie_buf, ie_len, 0, GFP_KERNEL);
	cfg80211_put_bss(bss);
	memcpy(priv->cfg_bssid, bss_info.bssid, ETH_ALEN);

	return 0;
}

/*
 * This function connects with a BSS.
 *
 * This function handles both Infra and Ad-Hoc modes. It also performs
 * validity checking on the provided parameters, disconnects from the
 * current BSS (if any), sets up the association/scan parameters,
 * including security settings, and performs specific SSID scan before
 * trying to connect.
 *
 * For Infra mode, the function returns failure if the specified SSID
 * is not found in scan table. However, for Ad-Hoc mode, it can create
 * the IBSS if it does not exist. On successful completion in either case,
 * the function notifies the CFG802.11 subsystem of the new BSS connection.
 */
static int
mwifiex_cfg80211_assoc(struct mwifiex_private *priv, size_t ssid_len, u8 *ssid,
		       u8 *bssid, int mode, struct ieee80211_channel *channel,
		       struct cfg80211_connect_params *sme, bool privacy)
{
	struct cfg80211_ssid req_ssid;
	int ret, auth_type = 0;
	struct cfg80211_bss *bss = NULL;
	u8 is_scanning_required = 0;

	memset(&req_ssid, 0, sizeof(struct cfg80211_ssid));

	req_ssid.ssid_len = ssid_len;
	if (ssid_len > IEEE80211_MAX_SSID_LEN) {
		dev_err(priv->adapter->dev, "invalid SSID - aborting\n");
		return -EINVAL;
	}

	memcpy(req_ssid.ssid, ssid, ssid_len);
	if (!req_ssid.ssid_len || req_ssid.ssid[0] < 0x20) {
		dev_err(priv->adapter->dev, "invalid SSID - aborting\n");
		return -EINVAL;
	}

	/* disconnect before try to associate */
	mwifiex_deauthenticate(priv, NULL);

	if (channel)
		ret = mwifiex_set_rf_channel(priv, channel,
						priv->adapter->channel_type);

	/* As this is new association, clear locally stored
	 * keys and security related flags */
	priv->sec_info.wpa_enabled = false;
	priv->sec_info.wpa2_enabled = false;
	priv->wep_key_curr_index = 0;
	priv->sec_info.encryption_mode = 0;
	priv->sec_info.is_authtype_auto = 0;
	ret = mwifiex_set_encode(priv, NULL, 0, 0, 1);

	if (mode == NL80211_IFTYPE_ADHOC) {
		/* "privacy" is set only for ad-hoc mode */
		if (privacy) {
			/*
			 * Keep WLAN_CIPHER_SUITE_WEP104 for now so that
			 * the firmware can find a matching network from the
			 * scan. The cfg80211 does not give us the encryption
			 * mode at this stage so just setting it to WEP here.
			 */
			priv->sec_info.encryption_mode =
					WLAN_CIPHER_SUITE_WEP104;
			priv->sec_info.authentication_mode =
					NL80211_AUTHTYPE_OPEN_SYSTEM;
		}

		goto done;
	}

	/* Now handle infra mode. "sme" is valid for infra mode only */
	if (sme->auth_type == NL80211_AUTHTYPE_AUTOMATIC) {
		auth_type = NL80211_AUTHTYPE_OPEN_SYSTEM;
		priv->sec_info.is_authtype_auto = 1;
	} else {
		auth_type = sme->auth_type;
	}

	if (sme->crypto.n_ciphers_pairwise) {
		priv->sec_info.encryption_mode =
						sme->crypto.ciphers_pairwise[0];
		priv->sec_info.authentication_mode = auth_type;
	}

	if (sme->crypto.cipher_group) {
		priv->sec_info.encryption_mode = sme->crypto.cipher_group;
		priv->sec_info.authentication_mode = auth_type;
	}
	if (sme->ie)
		ret = mwifiex_set_gen_ie(priv, sme->ie, sme->ie_len);

	if (sme->key) {
		if (mwifiex_is_alg_wep(priv->sec_info.encryption_mode)) {
			dev_dbg(priv->adapter->dev,
				"info: setting wep encryption"
				" with key len %d\n", sme->key_len);
			priv->wep_key_curr_index = sme->key_idx;
			ret = mwifiex_set_encode(priv, sme->key, sme->key_len,
							sme->key_idx, 0);
		}
	}
done:
	/*
	 * Scan entries are valid for some time (15 sec). So we can save one
	 * active scan time if we just try cfg80211_get_bss first. If it fails
	 * then request scan and cfg80211_get_bss() again for final output.
	 */
	while (1) {
		if (is_scanning_required) {
			/* Do specific SSID scanning */
			if (mwifiex_request_scan(priv, &req_ssid)) {
				dev_err(priv->adapter->dev, "scan error\n");
				return -EFAULT;
			}
		}

		/* Find the BSS we want using available scan results */
		if (mode == NL80211_IFTYPE_ADHOC)
			bss = cfg80211_get_bss(priv->wdev->wiphy, channel,
					       bssid, ssid, ssid_len,
					       WLAN_CAPABILITY_IBSS,
					       WLAN_CAPABILITY_IBSS);
		else
			bss = cfg80211_get_bss(priv->wdev->wiphy, channel,
					       bssid, ssid, ssid_len,
					       WLAN_CAPABILITY_ESS,
					       WLAN_CAPABILITY_ESS);

		if (!bss) {
			if (is_scanning_required) {
				dev_warn(priv->adapter->dev,
					 "assoc: requested bss not found in scan results\n");
				break;
			}
			is_scanning_required = 1;
		} else {
			dev_dbg(priv->adapter->dev,
				"info: trying to associate to '%s' bssid %pM\n",
				(char *) req_ssid.ssid, bss->bssid);
			memcpy(&priv->cfg_bssid, bss->bssid, ETH_ALEN);
			break;
		}
	}

	if (mwifiex_bss_start(priv, bss, &req_ssid))
		return -EFAULT;

	if (mode == NL80211_IFTYPE_ADHOC) {
		/* Inform the BSS information to kernel, otherwise
		 * kernel will give a panic after successful assoc */
		if (mwifiex_cfg80211_inform_ibss_bss(priv))
			return -EFAULT;
	}

	return ret;
}

/*
 * CFG802.11 operation handler for association request.
 *
 * This function does not work when the current mode is set to Ad-Hoc, or
 * when there is already an association procedure going on. The given BSS
 * information is used to associate.
 */
static int
mwifiex_cfg80211_connect(struct wiphy *wiphy, struct net_device *dev,
			 struct cfg80211_connect_params *sme)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(dev);
	int ret = 0;

	if (priv->bss_mode == NL80211_IFTYPE_ADHOC) {
		wiphy_err(wiphy, "received infra assoc request "
				"when station is in ibss mode\n");
		goto done;
	}

	wiphy_dbg(wiphy, "info: Trying to associate to %s and bssid %pM\n",
		  (char *) sme->ssid, sme->bssid);

	ret = mwifiex_cfg80211_assoc(priv, sme->ssid_len, sme->ssid, sme->bssid,
				     priv->bss_mode, sme->channel, sme, 0);
done:
	if (!ret) {
		cfg80211_connect_result(priv->netdev, priv->cfg_bssid, NULL, 0,
					NULL, 0, WLAN_STATUS_SUCCESS,
					GFP_KERNEL);
		dev_dbg(priv->adapter->dev,
			"info: associated to bssid %pM successfully\n",
			priv->cfg_bssid);
	} else {
		dev_dbg(priv->adapter->dev,
			"info: association to bssid %pM failed\n",
			priv->cfg_bssid);
		memset(priv->cfg_bssid, 0, ETH_ALEN);
	}

	return ret;
}

/*
 * CFG802.11 operation handler to join an IBSS.
 *
 * This function does not work in any mode other than Ad-Hoc, or if
 * a join operation is already in progress.
 */
static int
mwifiex_cfg80211_join_ibss(struct wiphy *wiphy, struct net_device *dev,
			   struct cfg80211_ibss_params *params)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(dev);
	int ret = 0;

	if (priv->bss_mode != NL80211_IFTYPE_ADHOC) {
		wiphy_err(wiphy, "request to join ibss received "
				"when station is not in ibss mode\n");
		goto done;
	}

	wiphy_dbg(wiphy, "info: trying to join to %s and bssid %pM\n",
		  (char *) params->ssid, params->bssid);

	ret = mwifiex_cfg80211_assoc(priv, params->ssid_len, params->ssid,
				     params->bssid, priv->bss_mode,
				     params->channel, NULL, params->privacy);
done:
	if (!ret) {
		cfg80211_ibss_joined(priv->netdev, priv->cfg_bssid, GFP_KERNEL);
		dev_dbg(priv->adapter->dev,
			"info: joined/created adhoc network with bssid"
			" %pM successfully\n", priv->cfg_bssid);
	} else {
		dev_dbg(priv->adapter->dev,
			"info: failed creating/joining adhoc network\n");
	}

	return ret;
}

/*
 * CFG802.11 operation handler to leave an IBSS.
 *
 * This function does not work if a leave operation is
 * already in progress.
 */
static int
mwifiex_cfg80211_leave_ibss(struct wiphy *wiphy, struct net_device *dev)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(dev);

	wiphy_dbg(wiphy, "info: disconnecting from essid %pM\n",
		  priv->cfg_bssid);
	if (mwifiex_deauthenticate(priv, NULL))
		return -EFAULT;

	memset(priv->cfg_bssid, 0, ETH_ALEN);

	return 0;
}

/*
 * CFG802.11 operation handler for scan request.
 *
 * This function issues a scan request to the firmware based upon
 * the user specified scan configuration. On successfull completion,
 * it also informs the results.
 */
static int
mwifiex_cfg80211_scan(struct wiphy *wiphy, struct net_device *dev,
		      struct cfg80211_scan_request *request)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(dev);
	int i;
	struct ieee80211_channel *chan;

	wiphy_dbg(wiphy, "info: received scan request on %s\n", dev->name);

	priv->scan_request = request;

	priv->user_scan_cfg = kzalloc(sizeof(struct mwifiex_user_scan_cfg),
				      GFP_KERNEL);
	if (!priv->user_scan_cfg) {
		dev_err(priv->adapter->dev, "failed to alloc scan_req\n");
		return -ENOMEM;
	}

	priv->user_scan_cfg->num_ssids = request->n_ssids;
	priv->user_scan_cfg->ssid_list = request->ssids;

	for (i = 0; i < request->n_channels; i++) {
		chan = request->channels[i];
		priv->user_scan_cfg->chan_list[i].chan_number = chan->hw_value;
		priv->user_scan_cfg->chan_list[i].radio_type = chan->band;

		if (chan->flags & IEEE80211_CHAN_PASSIVE_SCAN)
			priv->user_scan_cfg->chan_list[i].scan_type =
						MWIFIEX_SCAN_TYPE_PASSIVE;
		else
			priv->user_scan_cfg->chan_list[i].scan_type =
						MWIFIEX_SCAN_TYPE_ACTIVE;

		priv->user_scan_cfg->chan_list[i].scan_time = 0;
	}
	if (mwifiex_set_user_scan_ioctl(priv, priv->user_scan_cfg))
		return -EFAULT;

	return 0;
}

/*
 * This function sets up the CFG802.11 specific HT capability fields
 * with default values.
 *
 * The following default values are set -
 *      - HT Supported = True
 *      - Maximum AMPDU length factor = IEEE80211_HT_MAX_AMPDU_64K
 *      - Minimum AMPDU spacing = IEEE80211_HT_MPDU_DENSITY_NONE
 *      - HT Capabilities supported by firmware
 *      - MCS information, Rx mask = 0xff
 *      - MCD information, Tx parameters = IEEE80211_HT_MCS_TX_DEFINED (0x01)
 */
static void
mwifiex_setup_ht_caps(struct ieee80211_sta_ht_cap *ht_info,
		      struct mwifiex_private *priv)
{
	int rx_mcs_supp;
	struct ieee80211_mcs_info mcs_set;
	u8 *mcs = (u8 *)&mcs_set;
	struct mwifiex_adapter *adapter = priv->adapter;

	ht_info->ht_supported = true;
	ht_info->ampdu_factor = IEEE80211_HT_MAX_AMPDU_64K;
	ht_info->ampdu_density = IEEE80211_HT_MPDU_DENSITY_NONE;

	memset(&ht_info->mcs, 0, sizeof(ht_info->mcs));

	/* Fill HT capability information */
	if (ISSUPP_CHANWIDTH40(adapter->hw_dot_11n_dev_cap))
		ht_info->cap |= IEEE80211_HT_CAP_SUP_WIDTH_20_40;
	else
		ht_info->cap &= ~IEEE80211_HT_CAP_SUP_WIDTH_20_40;

	if (ISSUPP_SHORTGI20(adapter->hw_dot_11n_dev_cap))
		ht_info->cap |= IEEE80211_HT_CAP_SGI_20;
	else
		ht_info->cap &= ~IEEE80211_HT_CAP_SGI_20;

	if (ISSUPP_SHORTGI40(adapter->hw_dot_11n_dev_cap))
		ht_info->cap |= IEEE80211_HT_CAP_SGI_40;
	else
		ht_info->cap &= ~IEEE80211_HT_CAP_SGI_40;

	if (ISSUPP_RXSTBC(adapter->hw_dot_11n_dev_cap))
		ht_info->cap |= 1 << IEEE80211_HT_CAP_RX_STBC_SHIFT;
	else
		ht_info->cap &= ~(3 << IEEE80211_HT_CAP_RX_STBC_SHIFT);

	if (ISSUPP_TXSTBC(adapter->hw_dot_11n_dev_cap))
		ht_info->cap |= IEEE80211_HT_CAP_TX_STBC;
	else
		ht_info->cap &= ~IEEE80211_HT_CAP_TX_STBC;

	ht_info->cap &= ~IEEE80211_HT_CAP_MAX_AMSDU;
	ht_info->cap |= IEEE80211_HT_CAP_SM_PS;

	rx_mcs_supp = GET_RXMCSSUPP(adapter->hw_dev_mcs_support);
	/* Set MCS for 1x1 */
	memset(mcs, 0xff, rx_mcs_supp);
	/* Clear all the other values */
	memset(&mcs[rx_mcs_supp], 0,
	       sizeof(struct ieee80211_mcs_info) - rx_mcs_supp);
	if (priv->bss_mode == NL80211_IFTYPE_STATION ||
	    ISSUPP_CHANWIDTH40(adapter->hw_dot_11n_dev_cap))
		/* Set MCS32 for infra mode or ad-hoc mode with 40MHz support */
		SETHT_MCS32(mcs_set.rx_mask);

	memcpy((u8 *) &ht_info->mcs, mcs, sizeof(struct ieee80211_mcs_info));

	ht_info->mcs.tx_params = IEEE80211_HT_MCS_TX_DEFINED;
}

/*
 *  create a new virtual interface with the given name
 */
struct net_device *mwifiex_add_virtual_intf(struct wiphy *wiphy,
					    char *name,
					    enum nl80211_iftype type,
					    u32 *flags,
					    struct vif_params *params)
{
	struct mwifiex_private *priv = mwifiex_cfg80211_get_priv(wiphy);
	struct mwifiex_adapter *adapter;
	struct net_device *dev;
	void *mdev_priv;

	if (!priv)
		return NULL;

	adapter = priv->adapter;
	if (!adapter)
		return NULL;

	switch (type) {
	case NL80211_IFTYPE_UNSPECIFIED:
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_ADHOC:
		if (priv->bss_mode) {
			wiphy_err(wiphy, "cannot create multiple"
					" station/adhoc interfaces\n");
			return NULL;
		}

		if (type == NL80211_IFTYPE_UNSPECIFIED)
			priv->bss_mode = NL80211_IFTYPE_STATION;
		else
			priv->bss_mode = type;

		priv->bss_type = MWIFIEX_BSS_TYPE_STA;
		priv->frame_type = MWIFIEX_DATA_FRAME_TYPE_ETH_II;
		priv->bss_priority = 0;
		priv->bss_role = MWIFIEX_BSS_ROLE_STA;
		priv->bss_num = 0;

		break;
	default:
		wiphy_err(wiphy, "type not supported\n");
		return NULL;
	}

	dev = alloc_netdev_mq(sizeof(struct mwifiex_private *), name,
			      ether_setup, 1);
	if (!dev) {
		wiphy_err(wiphy, "no memory available for netdevice\n");
		goto error;
	}

	dev_net_set(dev, wiphy_net(wiphy));
	dev->ieee80211_ptr = priv->wdev;
	dev->ieee80211_ptr->iftype = priv->bss_mode;
	memcpy(dev->dev_addr, wiphy->perm_addr, ETH_ALEN);
	memcpy(dev->perm_addr, wiphy->perm_addr, ETH_ALEN);
	SET_NETDEV_DEV(dev, wiphy_dev(wiphy));

	dev->flags |= IFF_BROADCAST | IFF_MULTICAST;
	dev->watchdog_timeo = MWIFIEX_DEFAULT_WATCHDOG_TIMEOUT;
	dev->hard_header_len += MWIFIEX_MIN_DATA_HEADER_LEN;

	mdev_priv = netdev_priv(dev);
	*((unsigned long *) mdev_priv) = (unsigned long) priv;

	priv->netdev = dev;
	mwifiex_init_priv_params(priv, dev);

	SET_NETDEV_DEV(dev, adapter->dev);

	/* Register network device */
	if (register_netdevice(dev)) {
		wiphy_err(wiphy, "cannot register virtual network device\n");
		goto error;
	}

	sema_init(&priv->async_sem, 1);
	priv->scan_pending_on_block = false;

	dev_dbg(adapter->dev, "info: %s: Marvell 802.11 Adapter\n", dev->name);

#ifdef CONFIG_DEBUG_FS
	mwifiex_dev_debugfs_init(priv);
#endif
	return dev;
error:
	if (dev && (dev->reg_state == NETREG_UNREGISTERED))
		free_netdev(dev);
	priv->bss_mode = NL80211_IFTYPE_UNSPECIFIED;

	return NULL;
}
EXPORT_SYMBOL_GPL(mwifiex_add_virtual_intf);

/*
 * del_virtual_intf: remove the virtual interface determined by dev
 */
int mwifiex_del_virtual_intf(struct wiphy *wiphy, struct net_device *dev)
{
	struct mwifiex_private *priv = mwifiex_netdev_get_priv(dev);

#ifdef CONFIG_DEBUG_FS
	mwifiex_dev_debugfs_remove(priv);
#endif

	if (!netif_queue_stopped(priv->netdev))
		netif_stop_queue(priv->netdev);

	if (netif_carrier_ok(priv->netdev))
		netif_carrier_off(priv->netdev);

	if (dev->reg_state == NETREG_REGISTERED)
		unregister_netdevice(dev);

	if (dev->reg_state == NETREG_UNREGISTERED)
		free_netdev(dev);

	/* Clear the priv in adapter */
	priv->netdev = NULL;

	priv->media_connected = false;

	priv->bss_mode = NL80211_IFTYPE_UNSPECIFIED;

	return 0;
}
EXPORT_SYMBOL_GPL(mwifiex_del_virtual_intf);

/* station cfg80211 operations */
static struct cfg80211_ops mwifiex_cfg80211_ops = {
	.add_virtual_intf = mwifiex_add_virtual_intf,
	.del_virtual_intf = mwifiex_del_virtual_intf,
	.change_virtual_intf = mwifiex_cfg80211_change_virtual_intf,
	.scan = mwifiex_cfg80211_scan,
	.connect = mwifiex_cfg80211_connect,
	.disconnect = mwifiex_cfg80211_disconnect,
	.get_station = mwifiex_cfg80211_get_station,
	.set_wiphy_params = mwifiex_cfg80211_set_wiphy_params,
	.set_channel = mwifiex_cfg80211_set_channel,
	.join_ibss = mwifiex_cfg80211_join_ibss,
	.leave_ibss = mwifiex_cfg80211_leave_ibss,
	.add_key = mwifiex_cfg80211_add_key,
	.del_key = mwifiex_cfg80211_del_key,
	.set_default_key = mwifiex_cfg80211_set_default_key,
	.set_power_mgmt = mwifiex_cfg80211_set_power_mgmt,
	.set_tx_power = mwifiex_cfg80211_set_tx_power,
	.set_bitrate_mask = mwifiex_cfg80211_set_bitrate_mask,
};

/*
 * This function registers the device with CFG802.11 subsystem.
 *
 * The function creates the wireless device/wiphy, populates it with
 * default parameters and handler function pointers, and finally
 * registers the device.
 */
int mwifiex_register_cfg80211(struct mwifiex_private *priv)
{
	int ret;
	void *wdev_priv;
	struct wireless_dev *wdev;
	struct ieee80211_sta_ht_cap *ht_info;

	wdev = kzalloc(sizeof(struct wireless_dev), GFP_KERNEL);
	if (!wdev) {
		dev_err(priv->adapter->dev, "%s: allocating wireless device\n",
			__func__);
		return -ENOMEM;
	}
	wdev->wiphy =
		wiphy_new(&mwifiex_cfg80211_ops,
			  sizeof(struct mwifiex_private *));
	if (!wdev->wiphy) {
		kfree(wdev);
		return -ENOMEM;
	}
	wdev->iftype = NL80211_IFTYPE_STATION;
	wdev->wiphy->max_scan_ssids = 10;
	wdev->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION) |
				       BIT(NL80211_IFTYPE_ADHOC);

	wdev->wiphy->bands[IEEE80211_BAND_2GHZ] = &mwifiex_band_2ghz;
	ht_info = &wdev->wiphy->bands[IEEE80211_BAND_2GHZ]->ht_cap;
	mwifiex_setup_ht_caps(ht_info, priv);

	if (priv->adapter->config_bands & BAND_A) {
		wdev->wiphy->bands[IEEE80211_BAND_5GHZ] = &mwifiex_band_5ghz;
		ht_info = &wdev->wiphy->bands[IEEE80211_BAND_5GHZ]->ht_cap;
		mwifiex_setup_ht_caps(ht_info, priv);
	} else {
		wdev->wiphy->bands[IEEE80211_BAND_5GHZ] = NULL;
	}

	/* Initialize cipher suits */
	wdev->wiphy->cipher_suites = mwifiex_cipher_suites;
	wdev->wiphy->n_cipher_suites = ARRAY_SIZE(mwifiex_cipher_suites);

	memcpy(wdev->wiphy->perm_addr, priv->curr_addr, ETH_ALEN);
	wdev->wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;

	/* Reserve space for bss band information */
	wdev->wiphy->bss_priv_size = sizeof(u8);

	wdev->wiphy->reg_notifier = mwifiex_reg_notifier;

	/* Set struct mwifiex_private pointer in wiphy_priv */
	wdev_priv = wiphy_priv(wdev->wiphy);

	*(unsigned long *) wdev_priv = (unsigned long) priv;

	set_wiphy_dev(wdev->wiphy, (struct device *) priv->adapter->dev);

	ret = wiphy_register(wdev->wiphy);
	if (ret < 0) {
		dev_err(priv->adapter->dev, "%s: registering cfg80211 device\n",
			__func__);
		wiphy_free(wdev->wiphy);
		kfree(wdev);
		return ret;
	} else {
		dev_dbg(priv->adapter->dev,
			"info: successfully registered wiphy device\n");
	}

	priv->wdev = wdev;

	return ret;
}
