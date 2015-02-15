/******************************************************************************
 *
 * Copyright(c) 2003 - 2012 Intel Corporation. All rights reserved.
 *
 * Portions of this file are derived from the ipw3945 project, as well
 * as portions of the ieee80211 subsystem header files.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 * The full GNU General Public License is included in this distribution in the
 * file called LICENSE.
 *
 * Contact Information:
 *  Intel Linux Wireless <ilw@linux.intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 *****************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>

#include <net/mac80211.h>

#include <asm/div64.h>

#include "iwl-eeprom.h"
#include "iwl-dev.h"
#include "iwl-core.h"
#include "iwl-io.h"
#include "iwl-agn-calib.h"
#include "iwl-agn.h"
#include "iwl-shared.h"
#include "iwl-trans.h"
#include "iwl-op-mode.h"

/*****************************************************************************
 *
 * mac80211 entry point functions
 *
 *****************************************************************************/

static const struct ieee80211_iface_limit iwlagn_sta_ap_limits[] = {
	{
		.max = 1,
		.types = BIT(NL80211_IFTYPE_STATION),
	},
	{
		.max = 1,
		.types = BIT(NL80211_IFTYPE_AP),
	},
};

static const struct ieee80211_iface_limit iwlagn_2sta_limits[] = {
	{
		.max = 2,
		.types = BIT(NL80211_IFTYPE_STATION),
	},
};

static const struct ieee80211_iface_limit iwlagn_p2p_sta_go_limits[] = {
	{
		.max = 1,
		.types = BIT(NL80211_IFTYPE_STATION),
	},
	{
		.max = 1,
		.types = BIT(NL80211_IFTYPE_P2P_GO) |
			 BIT(NL80211_IFTYPE_AP),
	},
};

static const struct ieee80211_iface_limit iwlagn_p2p_2sta_limits[] = {
	{
		.max = 2,
		.types = BIT(NL80211_IFTYPE_STATION),
	},
	{
		.max = 1,
		.types = BIT(NL80211_IFTYPE_P2P_CLIENT),
	},
};

static const struct ieee80211_iface_combination
iwlagn_iface_combinations_dualmode[] = {
	{ .num_different_channels = 1,
	  .max_interfaces = 2,
	  .beacon_int_infra_match = true,
	  .limits = iwlagn_sta_ap_limits,
	  .n_limits = ARRAY_SIZE(iwlagn_sta_ap_limits),
	},
	{ .num_different_channels = 1,
	  .max_interfaces = 2,
	  .limits = iwlagn_2sta_limits,
	  .n_limits = ARRAY_SIZE(iwlagn_2sta_limits),
	},
};

static const struct ieee80211_iface_combination
iwlagn_iface_combinations_p2p[] = {
	{ .num_different_channels = 1,
	  .max_interfaces = 2,
	  .beacon_int_infra_match = true,
	  .limits = iwlagn_p2p_sta_go_limits,
	  .n_limits = ARRAY_SIZE(iwlagn_p2p_sta_go_limits),
	},
	{ .num_different_channels = 1,
	  .max_interfaces = 2,
	  .limits = iwlagn_p2p_2sta_limits,
	  .n_limits = ARRAY_SIZE(iwlagn_p2p_2sta_limits),
	},
};

/*
 * Not a mac80211 entry point function, but it fits in with all the
 * other mac80211 functions grouped here.
 */
int iwlagn_mac_setup_register(struct iwl_priv *priv,
			      const struct iwl_ucode_capabilities *capa)
{
	int ret;
	struct ieee80211_hw *hw = priv->hw;
	struct iwl_rxon_context *ctx;

	hw->rate_control_algorithm = "iwl-agn-rs";

	/* Tell mac80211 our characteristics */
	hw->flags = IEEE80211_HW_SIGNAL_DBM |
		    IEEE80211_HW_AMPDU_AGGREGATION |
		    IEEE80211_HW_NEED_DTIM_PERIOD |
		    IEEE80211_HW_SPECTRUM_MGMT |
		    IEEE80211_HW_REPORTS_TX_ACK_STATUS;

	/*
	 * Including the following line will crash some AP's.  This
	 * workaround removes the stimulus which causes the crash until
	 * the AP software can be fixed.
	hw->max_tx_aggregation_subframes = LINK_QUAL_AGG_FRAME_LIMIT_DEF;
	 */

	hw->flags |= IEEE80211_HW_SUPPORTS_PS |
		     IEEE80211_HW_SUPPORTS_DYNAMIC_PS;

	if (hw_params(priv).sku & EEPROM_SKU_CAP_11N_ENABLE)
		hw->flags |= IEEE80211_HW_SUPPORTS_DYNAMIC_SMPS |
			     IEEE80211_HW_SUPPORTS_STATIC_SMPS;

#ifndef CONFIG_IWLWIFI_EXPERIMENTAL_MFP
	/* enable 11w if the uCode advertise */
	if (capa->flags & IWL_UCODE_TLV_FLAGS_MFP)
#endif /* !CONFIG_IWLWIFI_EXPERIMENTAL_MFP */
		hw->flags |= IEEE80211_HW_MFP_CAPABLE;

	hw->sta_data_size = sizeof(struct iwl_station_priv);
	hw->vif_data_size = sizeof(struct iwl_vif_priv);

	for_each_context(priv, ctx) {
		hw->wiphy->interface_modes |= ctx->interface_modes;
		hw->wiphy->interface_modes |= ctx->exclusive_interface_modes;
	}

	BUILD_BUG_ON(NUM_IWL_RXON_CTX != 2);

	if (hw->wiphy->interface_modes & BIT(NL80211_IFTYPE_P2P_CLIENT)) {
		hw->wiphy->iface_combinations = iwlagn_iface_combinations_p2p;
		hw->wiphy->n_iface_combinations =
			ARRAY_SIZE(iwlagn_iface_combinations_p2p);
	} else if (hw->wiphy->interface_modes & BIT(NL80211_IFTYPE_AP)) {
		hw->wiphy->iface_combinations =
			iwlagn_iface_combinations_dualmode;
		hw->wiphy->n_iface_combinations =
			ARRAY_SIZE(iwlagn_iface_combinations_dualmode);
	}

	hw->wiphy->max_remain_on_channel_duration = 1000;

	hw->wiphy->flags |= WIPHY_FLAG_CUSTOM_REGULATORY |
			    WIPHY_FLAG_DISABLE_BEACON_HINTS |
			    WIPHY_FLAG_IBSS_RSN;

	if (priv->fw->img[IWL_UCODE_WOWLAN].sec[0].len &&
	    trans(priv)->ops->wowlan_suspend &&
	    device_can_wakeup(trans(priv)->dev)) {
		hw->wiphy->wowlan.flags = WIPHY_WOWLAN_MAGIC_PKT |
					  WIPHY_WOWLAN_DISCONNECT |
					  WIPHY_WOWLAN_EAP_IDENTITY_REQ |
					  WIPHY_WOWLAN_RFKILL_RELEASE;
		if (!iwlagn_mod_params.sw_crypto)
			hw->wiphy->wowlan.flags |=
				WIPHY_WOWLAN_SUPPORTS_GTK_REKEY |
				WIPHY_WOWLAN_GTK_REKEY_FAILURE;

		hw->wiphy->wowlan.n_patterns = IWLAGN_WOWLAN_MAX_PATTERNS;
		hw->wiphy->wowlan.pattern_min_len =
					IWLAGN_WOWLAN_MIN_PATTERN_LEN;
		hw->wiphy->wowlan.pattern_max_len =
					IWLAGN_WOWLAN_MAX_PATTERN_LEN;
	}

	if (iwlagn_mod_params.power_save)
		hw->wiphy->flags |= WIPHY_FLAG_PS_ON_BY_DEFAULT;
	else
		hw->wiphy->flags &= ~WIPHY_FLAG_PS_ON_BY_DEFAULT;

	hw->wiphy->max_scan_ssids = PROBE_OPTION_MAX;
	/* we create the 802.11 header and a zero-length SSID element */
	hw->wiphy->max_scan_ie_len = capa->max_probe_length - 24 - 2;

	/* Default value; 4 EDCA QOS priorities */
	hw->queues = 4;

	hw->max_listen_interval = IWL_CONN_MAX_LISTEN_INTERVAL;

	if (priv->bands[IEEE80211_BAND_2GHZ].n_channels)
		priv->hw->wiphy->bands[IEEE80211_BAND_2GHZ] =
			&priv->bands[IEEE80211_BAND_2GHZ];
	if (priv->bands[IEEE80211_BAND_5GHZ].n_channels)
		priv->hw->wiphy->bands[IEEE80211_BAND_5GHZ] =
			&priv->bands[IEEE80211_BAND_5GHZ];

	hw->wiphy->hw_version = trans(priv)->hw_id;

	iwl_leds_init(priv);

	ret = ieee80211_register_hw(priv->hw);
	if (ret) {
		IWL_ERR(priv, "Failed to register hw (error %d)\n", ret);
		return ret;
	}
	priv->mac80211_registered = 1;

	return 0;
}

void iwlagn_mac_unregister(struct iwl_priv *priv)
{
	if (!priv->mac80211_registered)
		return;
	iwl_leds_exit(priv);
	ieee80211_unregister_hw(priv->hw);
	priv->mac80211_registered = 0;
}

static int __iwl_up(struct iwl_priv *priv)
{
	struct iwl_rxon_context *ctx;
	int ret;

	lockdep_assert_held(&priv->mutex);

	if (test_bit(STATUS_EXIT_PENDING, &priv->status)) {
		IWL_WARN(priv, "Exit pending; will not bring the NIC up\n");
		return -EIO;
	}

	for_each_context(priv, ctx) {
		ret = iwlagn_alloc_bcast_station(priv, ctx);
		if (ret) {
			iwl_dealloc_bcast_stations(priv);
			return ret;
		}
	}

	ret = iwl_run_init_ucode(priv);
	if (ret) {
		IWL_ERR(priv, "Failed to run INIT ucode: %d\n", ret);
		goto error;
	}

	ret = iwl_load_ucode_wait_alive(priv, IWL_UCODE_REGULAR);
	if (ret) {
		IWL_ERR(priv, "Failed to start RT ucode: %d\n", ret);
		goto error;
	}

	ret = iwl_alive_start(priv);
	if (ret)
		goto error;
	return 0;

 error:
	set_bit(STATUS_EXIT_PENDING, &priv->status);
	iwl_down(priv);
	clear_bit(STATUS_EXIT_PENDING, &priv->status);

	IWL_ERR(priv, "Unable to initialize device.\n");
	return ret;
}

static int iwlagn_mac_start(struct ieee80211_hw *hw)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);
	int ret;

	IWL_DEBUG_MAC80211(priv, "enter\n");

	/* we should be verifying the device is ready to be opened */
	mutex_lock(&priv->mutex);
	ret = __iwl_up(priv);
	mutex_unlock(&priv->mutex);
	if (ret)
		return ret;

	IWL_DEBUG_INFO(priv, "Start UP work done.\n");

	/* Now we should be done, and the READY bit should be set. */
	if (WARN_ON(!test_bit(STATUS_READY, &priv->status)))
		ret = -EIO;

	iwlagn_led_enable(priv);

	priv->is_open = 1;
	IWL_DEBUG_MAC80211(priv, "leave\n");
	return 0;
}

static void iwlagn_mac_stop(struct ieee80211_hw *hw)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);

	IWL_DEBUG_MAC80211(priv, "enter\n");

	if (!priv->is_open)
		return;

	priv->is_open = 0;

	mutex_lock(&priv->mutex);
	iwl_down(priv);
	mutex_unlock(&priv->mutex);

	iwl_cancel_deferred_work(priv);

	flush_workqueue(priv->workqueue);

	/* User space software may expect getting rfkill changes
	 * even if interface is down, trans->down will leave the RF
	 * kill interrupt enabled
	 */
	iwl_trans_stop_hw(trans(priv));

	IWL_DEBUG_MAC80211(priv, "leave\n");
}

static void iwlagn_mac_set_rekey_data(struct ieee80211_hw *hw,
				      struct ieee80211_vif *vif,
				      struct cfg80211_gtk_rekey_data *data)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);

	if (iwlagn_mod_params.sw_crypto)
		return;

	IWL_DEBUG_MAC80211(priv, "enter\n");
	mutex_lock(&priv->mutex);

	if (priv->contexts[IWL_RXON_CTX_BSS].vif != vif)
		goto out;

	memcpy(priv->kek, data->kek, NL80211_KEK_LEN);
	memcpy(priv->kck, data->kck, NL80211_KCK_LEN);
	priv->replay_ctr =
		cpu_to_le64(be64_to_cpup((__be64 *)&data->replay_ctr));
	priv->have_rekey_data = true;

 out:
	mutex_unlock(&priv->mutex);
	IWL_DEBUG_MAC80211(priv, "leave\n");
}

#ifdef CONFIG_PM_SLEEP

static int iwlagn_mac_suspend(struct ieee80211_hw *hw,
			      struct cfg80211_wowlan *wowlan)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);
	struct iwl_rxon_context *ctx = &priv->contexts[IWL_RXON_CTX_BSS];
	int ret;

	if (WARN_ON(!wowlan))
		return -EINVAL;

	IWL_DEBUG_MAC80211(priv, "enter\n");
	mutex_lock(&priv->mutex);

	/* Don't attempt WoWLAN when not associated, tear down instead. */
	if (!ctx->vif || ctx->vif->type != NL80211_IFTYPE_STATION ||
	    !iwl_is_associated_ctx(ctx)) {
		ret = 1;
		goto out;
	}

	ret = iwlagn_suspend(priv, wowlan);
	if (ret)
		goto error;

	device_set_wakeup_enable(trans(priv)->dev, true);

	iwl_trans_wowlan_suspend(trans(priv));

	goto out;

 error:
	priv->wowlan = false;
	iwlagn_prepare_restart(priv);
	ieee80211_restart_hw(priv->hw);
 out:
	mutex_unlock(&priv->mutex);
	IWL_DEBUG_MAC80211(priv, "leave\n");

	return ret;
}

static int iwlagn_mac_resume(struct ieee80211_hw *hw)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);
	struct iwl_rxon_context *ctx = &priv->contexts[IWL_RXON_CTX_BSS];
	struct ieee80211_vif *vif;
	unsigned long flags;
	u32 base, status = 0xffffffff;
	int ret = -EIO;
	const struct fw_img *img;

	IWL_DEBUG_MAC80211(priv, "enter\n");
	mutex_lock(&priv->mutex);

	iwl_write32(trans(priv), CSR_UCODE_DRV_GP1_CLR,
			  CSR_UCODE_DRV_GP1_BIT_D3_CFG_COMPLETE);

	base = priv->shrd->device_pointers.error_event_table;
	if (iwlagn_hw_valid_rtc_data_addr(base)) {
		spin_lock_irqsave(&trans(priv)->reg_lock, flags);
		ret = iwl_grab_nic_access_silent(trans(priv));
		if (likely(ret == 0)) {
			iwl_write32(trans(priv), HBUS_TARG_MEM_RADDR, base);
			status = iwl_read32(trans(priv), HBUS_TARG_MEM_RDAT);
			iwl_release_nic_access(trans(priv));
		}
		spin_unlock_irqrestore(&trans(priv)->reg_lock, flags);

#ifdef CONFIG_IWLWIFI_DEBUGFS
		if (ret == 0) {
			img = &(priv->fw->img[IWL_UCODE_WOWLAN]);
			if (!priv->wowlan_sram) {
				priv->wowlan_sram =
				   kzalloc(img->sec[IWL_UCODE_SECTION_DATA].len,
						GFP_KERNEL);
			}

			if (priv->wowlan_sram)
				_iwl_read_targ_mem_words(
				      trans(priv), 0x800000,
				      priv->wowlan_sram,
				      img->sec[IWL_UCODE_SECTION_DATA].len / 4);
		}
#endif
	}

	/* we'll clear ctx->vif during iwlagn_prepare_restart() */
	vif = ctx->vif;

	priv->wowlan = false;

	device_set_wakeup_enable(trans(priv)->dev, false);

	iwlagn_prepare_restart(priv);

	memset((void *)&ctx->active, 0, sizeof(ctx->active));
	iwl_connection_init_rx_config(priv, ctx);
	iwlagn_set_rxon_chain(priv, ctx);

	mutex_unlock(&priv->mutex);
	IWL_DEBUG_MAC80211(priv, "leave\n");

	ieee80211_resume_disconnect(vif);

	return 1;
}

#endif

static void iwlagn_mac_tx(struct ieee80211_hw *hw, struct sk_buff *skb)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);

	IWL_DEBUG_TX(priv, "dev->xmit(%d bytes) at rate 0x%02x\n", skb->len,
		     ieee80211_get_tx_rate(hw, IEEE80211_SKB_CB(skb))->bitrate);

	if (iwlagn_tx_skb(priv, skb))
		dev_kfree_skb_any(skb);
}

static void iwlagn_mac_update_tkip_key(struct ieee80211_hw *hw,
				       struct ieee80211_vif *vif,
				       struct ieee80211_key_conf *keyconf,
				       struct ieee80211_sta *sta,
				       u32 iv32, u16 *phase1key)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);

	iwl_update_tkip_key(priv, vif, keyconf, sta, iv32, phase1key);
}

static int iwlagn_mac_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
			      struct ieee80211_vif *vif,
			      struct ieee80211_sta *sta,
			      struct ieee80211_key_conf *key)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);
	struct iwl_vif_priv *vif_priv = (void *)vif->drv_priv;
	struct iwl_rxon_context *ctx = vif_priv->ctx;
	int ret;
	bool is_default_wep_key = false;

	IWL_DEBUG_MAC80211(priv, "enter\n");

	if (iwlagn_mod_params.sw_crypto) {
		IWL_DEBUG_MAC80211(priv, "leave - hwcrypto disabled\n");
		return -EOPNOTSUPP;
	}

	switch (key->cipher) {
	case WLAN_CIPHER_SUITE_TKIP:
		key->flags |= IEEE80211_KEY_FLAG_GENERATE_MMIC;
		/* fall through */
	case WLAN_CIPHER_SUITE_CCMP:
		key->flags |= IEEE80211_KEY_FLAG_GENERATE_IV;
		break;
	default:
		break;
	}

	/*
	 * We could program these keys into the hardware as well, but we
	 * don't expect much multicast traffic in IBSS and having keys
	 * for more stations is probably more useful.
	 *
	 * Mark key TX-only and return 0.
	 */
	if (vif->type == NL80211_IFTYPE_ADHOC &&
	    !(key->flags & IEEE80211_KEY_FLAG_PAIRWISE)) {
		key->hw_key_idx = WEP_INVALID_OFFSET;
		return 0;
	}

	/* If they key was TX-only, accept deletion */
	if (cmd == DISABLE_KEY && key->hw_key_idx == WEP_INVALID_OFFSET)
		return 0;

	mutex_lock(&priv->mutex);
	iwl_scan_cancel_timeout(priv, 100);

	BUILD_BUG_ON(WEP_INVALID_OFFSET == IWLAGN_HW_KEY_DEFAULT);

	/*
	 * If we are getting WEP group key and we didn't receive any key mapping
	 * so far, we are in legacy wep mode (group key only), otherwise we are
	 * in 1X mode.
	 * In legacy wep mode, we use another host command to the uCode.
	 */
	if ((key->cipher == WLAN_CIPHER_SUITE_WEP40 ||
	     key->cipher == WLAN_CIPHER_SUITE_WEP104) && !sta) {
		if (cmd == SET_KEY)
			is_default_wep_key = !ctx->key_mapping_keys;
		else
			is_default_wep_key =
				key->hw_key_idx == IWLAGN_HW_KEY_DEFAULT;
	}


	switch (cmd) {
	case SET_KEY:
		if (is_default_wep_key) {
			ret = iwl_set_default_wep_key(priv, vif_priv->ctx, key);
			break;
		}
		ret = iwl_set_dynamic_key(priv, vif_priv->ctx, key, sta);
		if (ret) {
			/*
			 * can't add key for RX, but we don't need it
			 * in the device for TX so still return 0
			 */
			ret = 0;
			key->hw_key_idx = WEP_INVALID_OFFSET;
		}

		IWL_DEBUG_MAC80211(priv, "enable hwcrypto key\n");
		break;
	case DISABLE_KEY:
		if (is_default_wep_key)
			ret = iwl_remove_default_wep_key(priv, ctx, key);
		else
			ret = iwl_remove_dynamic_key(priv, ctx, key, sta);

		IWL_DEBUG_MAC80211(priv, "disable hwcrypto key\n");
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&priv->mutex);
	IWL_DEBUG_MAC80211(priv, "leave\n");

	return ret;
}

static int iwlagn_mac_ampdu_action(struct ieee80211_hw *hw,
				   struct ieee80211_vif *vif,
				   enum ieee80211_ampdu_mlme_action action,
				   struct ieee80211_sta *sta, u16 tid, u16 *ssn,
				   u8 buf_size)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);
	int ret = -EINVAL;
	struct iwl_station_priv *sta_priv = (void *) sta->drv_priv;

	IWL_DEBUG_HT(priv, "A-MPDU action on addr %pM tid %d\n",
		     sta->addr, tid);

	if (!(hw_params(priv).sku & EEPROM_SKU_CAP_11N_ENABLE))
		return -EACCES;

	IWL_DEBUG_MAC80211(priv, "enter\n");
	mutex_lock(&priv->mutex);

	switch (action) {
	case IEEE80211_AMPDU_RX_START:
		if (iwlagn_mod_params.disable_11n & IWL_DISABLE_HT_RXAGG)
			break;
		IWL_DEBUG_HT(priv, "start Rx\n");
		ret = iwl_sta_rx_agg_start(priv, sta, tid, *ssn);
		break;
	case IEEE80211_AMPDU_RX_STOP:
		IWL_DEBUG_HT(priv, "stop Rx\n");
		ret = iwl_sta_rx_agg_stop(priv, sta, tid);
		break;
	case IEEE80211_AMPDU_TX_START:
		if (iwlagn_mod_params.disable_11n & IWL_DISABLE_HT_TXAGG)
			break;
		IWL_DEBUG_HT(priv, "start Tx\n");
		ret = iwlagn_tx_agg_start(priv, vif, sta, tid, ssn);
		break;
	case IEEE80211_AMPDU_TX_STOP:
		IWL_DEBUG_HT(priv, "stop Tx\n");
		ret = iwlagn_tx_agg_stop(priv, vif, sta, tid);
		if ((ret == 0) && (priv->agg_tids_count > 0)) {
			priv->agg_tids_count--;
			IWL_DEBUG_HT(priv, "priv->agg_tids_count = %u\n",
				     priv->agg_tids_count);
		}
		if (!priv->agg_tids_count &&
		    hw_params(priv).use_rts_for_aggregation) {
			/*
			 * switch off RTS/CTS if it was previously enabled
			 */
			sta_priv->lq_sta.lq.general_params.flags &=
				~LINK_QUAL_FLAGS_SET_STA_TLC_RTS_MSK;
			iwl_send_lq_cmd(priv, iwl_rxon_ctx_from_vif(vif),
					&sta_priv->lq_sta.lq, CMD_ASYNC, false);
		}
		break;
	case IEEE80211_AMPDU_TX_OPERATIONAL:
		ret = iwlagn_tx_agg_oper(priv, vif, sta, tid, buf_size);
		break;
	}
	mutex_unlock(&priv->mutex);
	IWL_DEBUG_MAC80211(priv, "leave\n");
	return ret;
}

static int iwlagn_mac_sta_add(struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif,
			      struct ieee80211_sta *sta)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);
	struct iwl_station_priv *sta_priv = (void *)sta->drv_priv;
	struct iwl_vif_priv *vif_priv = (void *)vif->drv_priv;
	bool is_ap = vif->type == NL80211_IFTYPE_STATION;
	int ret;
	u8 sta_id;

	IWL_DEBUG_INFO(priv, "proceeding to add station %pM\n",
			sta->addr);
	sta_priv->sta_id = IWL_INVALID_STATION;

	atomic_set(&sta_priv->pending_frames, 0);
	if (vif->type == NL80211_IFTYPE_AP)
		sta_priv->client = true;

	ret = iwl_add_station_common(priv, vif_priv->ctx, sta->addr,
				     is_ap, sta, &sta_id);
	if (ret) {
		IWL_ERR(priv, "Unable to add station %pM (%d)\n",
			sta->addr, ret);
		/* Should we return success if return code is EEXIST ? */
		return ret;
	}

	sta_priv->sta_id = sta_id;

	return 0;
}

static int iwlagn_mac_sta_remove(struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif,
				 struct ieee80211_sta *sta)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);
	struct iwl_station_priv *sta_priv = (void *)sta->drv_priv;
	int ret;

	IWL_DEBUG_INFO(priv, "proceeding to remove station %pM\n", sta->addr);

	if (vif->type == NL80211_IFTYPE_STATION) {
		/*
		 * Station will be removed from device when the RXON
		 * is set to unassociated -- just deactivate it here
		 * to avoid re-programming it.
		 */
		ret = 0;
		iwl_deactivate_station(priv, sta_priv->sta_id, sta->addr);
	} else {
		ret = iwl_remove_station(priv, sta_priv->sta_id, sta->addr);
		if (ret)
			IWL_DEBUG_QUIET_RFKILL(priv,
				"Error removing station %pM\n", sta->addr);
	}
	return ret;
}

static int iwlagn_mac_sta_state(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif,
				struct ieee80211_sta *sta,
				enum ieee80211_sta_state old_state,
				enum ieee80211_sta_state new_state)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);
	struct iwl_vif_priv *vif_priv = (void *)vif->drv_priv;
	enum {
		NONE, ADD, REMOVE, HT_RATE_INIT, ADD_RATE_INIT,
	} op = NONE;
	int ret;

	IWL_DEBUG_MAC80211(priv, "station %pM state change %d->%d\n",
			   sta->addr, old_state, new_state);

	mutex_lock(&priv->mutex);
	if (vif->type == NL80211_IFTYPE_STATION) {
		if (old_state == IEEE80211_STA_NOTEXIST &&
		    new_state == IEEE80211_STA_NONE)
			op = ADD;
		else if (old_state == IEEE80211_STA_NONE &&
			 new_state == IEEE80211_STA_NOTEXIST)
			op = REMOVE;
		else if (old_state == IEEE80211_STA_AUTH &&
			 new_state == IEEE80211_STA_ASSOC)
			op = HT_RATE_INIT;
	} else {
		if (old_state == IEEE80211_STA_AUTH &&
		    new_state == IEEE80211_STA_ASSOC)
			op = ADD_RATE_INIT;
		else if (old_state == IEEE80211_STA_ASSOC &&
			 new_state == IEEE80211_STA_AUTH)
			op = REMOVE;
	}

	switch (op) {
	case ADD:
		ret = iwlagn_mac_sta_add(hw, vif, sta);
		break;
	case REMOVE:
		ret = iwlagn_mac_sta_remove(hw, vif, sta);
		break;
	case ADD_RATE_INIT:
		ret = iwlagn_mac_sta_add(hw, vif, sta);
		if (ret)
			break;
		/* Initialize rate scaling */
		IWL_DEBUG_INFO(priv,
			       "Initializing rate scaling for station %pM\n",
			       sta->addr);
		iwl_rs_rate_init(priv, sta, iwl_sta_id(sta));
		ret = 0;
		break;
	case HT_RATE_INIT:
		/* Initialize rate scaling */
		ret = iwl_sta_update_ht(priv, vif_priv->ctx, sta);
		if (ret)
			break;
		IWL_DEBUG_INFO(priv,
			       "Initializing rate scaling for station %pM\n",
			       sta->addr);
		iwl_rs_rate_init(priv, sta, iwl_sta_id(sta));
		ret = 0;
		break;
	default:
		ret = 0;
		break;
	}

	/*
	 * mac80211 might WARN if we fail, but due the way we
	 * (badly) handle hard rfkill, we might fail here
	 */
	if (iwl_is_rfkill(priv))
		ret = 0;

	mutex_unlock(&priv->mutex);
	IWL_DEBUG_MAC80211(priv, "leave\n");

	return ret;
}

static void iwlagn_mac_channel_switch(struct ieee80211_hw *hw,
				struct ieee80211_channel_switch *ch_switch)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);
	const struct iwl_channel_info *ch_info;
	struct ieee80211_conf *conf = &hw->conf;
	struct ieee80211_channel *channel = ch_switch->channel;
	struct iwl_ht_config *ht_conf = &priv->current_ht_config;
	/*
	 * MULTI-FIXME
	 * When we add support for multiple interfaces, we need to
	 * revisit this. The channel switch command in the device
	 * only affects the BSS context, but what does that really
	 * mean? And what if we get a CSA on the second interface?
	 * This needs a lot of work.
	 */
	struct iwl_rxon_context *ctx = &priv->contexts[IWL_RXON_CTX_BSS];
	u16 ch;

	IWL_DEBUG_MAC80211(priv, "enter\n");

	mutex_lock(&priv->mutex);

	if (iwl_is_rfkill(priv))
		goto out;

	if (test_bit(STATUS_EXIT_PENDING, &priv->status) ||
	    test_bit(STATUS_SCANNING, &priv->status) ||
	    test_bit(STATUS_CHANNEL_SWITCH_PENDING, &priv->status))
		goto out;

	if (!iwl_is_associated_ctx(ctx))
		goto out;

	if (!cfg(priv)->lib->set_channel_switch)
		goto out;

	ch = channel->hw_value;
	if (le16_to_cpu(ctx->active.channel) == ch)
		goto out;

	ch_info = iwl_get_channel_info(priv, channel->band, ch);
	if (!is_channel_valid(ch_info)) {
		IWL_DEBUG_MAC80211(priv, "invalid channel\n");
		goto out;
	}

	priv->current_ht_config.smps = conf->smps_mode;

	/* Configure HT40 channels */
	ctx->ht.enabled = conf_is_ht(conf);
	if (ctx->ht.enabled)
		iwlagn_config_ht40(conf, ctx);
	else
		ctx->ht.is_40mhz = false;

	if ((le16_to_cpu(ctx->staging.channel) != ch))
		ctx->staging.flags = 0;

	iwl_set_rxon_channel(priv, channel, ctx);
	iwl_set_rxon_ht(priv, ht_conf);
	iwl_set_flags_for_band(priv, ctx, channel->band, ctx->vif);

	iwl_set_rate(priv);
	/*
	 * at this point, staging_rxon has the
	 * configuration for channel switch
	 */
	set_bit(STATUS_CHANNEL_SWITCH_PENDING, &priv->status);
	priv->switch_channel = cpu_to_le16(ch);
	if (cfg(priv)->lib->set_channel_switch(priv, ch_switch)) {
		clear_bit(STATUS_CHANNEL_SWITCH_PENDING, &priv->status);
		priv->switch_channel = 0;
		ieee80211_chswitch_done(ctx->vif, false);
	}

out:
	mutex_unlock(&priv->mutex);
	IWL_DEBUG_MAC80211(priv, "leave\n");
}

static void iwlagn_configure_filter(struct ieee80211_hw *hw,
				    unsigned int changed_flags,
				    unsigned int *total_flags,
				    u64 multicast)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);
	__le32 filter_or = 0, filter_nand = 0;
	struct iwl_rxon_context *ctx;

#define CHK(test, flag)	do { \
	if (*total_flags & (test))		\
		filter_or |= (flag);		\
	else					\
		filter_nand |= (flag);		\
	} while (0)

	IWL_DEBUG_MAC80211(priv, "Enter: changed: 0x%x, total: 0x%x\n",
			changed_flags, *total_flags);

	CHK(FIF_OTHER_BSS | FIF_PROMISC_IN_BSS, RXON_FILTER_PROMISC_MSK);
	/* Setting _just_ RXON_FILTER_CTL2HOST_MSK causes FH errors */
	CHK(FIF_CONTROL, RXON_FILTER_CTL2HOST_MSK | RXON_FILTER_PROMISC_MSK);
	CHK(FIF_BCN_PRBRESP_PROMISC, RXON_FILTER_BCON_AWARE_MSK);

#undef CHK

	mutex_lock(&priv->mutex);

	for_each_context(priv, ctx) {
		ctx->staging.filter_flags &= ~filter_nand;
		ctx->staging.filter_flags |= filter_or;

		/*
		 * Not committing directly because hardware can perform a scan,
		 * but we'll eventually commit the filter flags change anyway.
		 */
	}

	mutex_unlock(&priv->mutex);

	/*
	 * Receiving all multicast frames is always enabled by the
	 * default flags setup in iwl_connection_init_rx_config()
	 * since we currently do not support programming multicast
	 * filters into the device.
	 */
	*total_flags &= FIF_OTHER_BSS | FIF_ALLMULTI | FIF_PROMISC_IN_BSS |
			FIF_BCN_PRBRESP_PROMISC | FIF_CONTROL;
}

static void iwlagn_mac_flush(struct ieee80211_hw *hw, bool drop)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);

	mutex_lock(&priv->mutex);
	IWL_DEBUG_MAC80211(priv, "enter\n");

	if (test_bit(STATUS_EXIT_PENDING, &priv->status)) {
		IWL_DEBUG_TX(priv, "Aborting flush due to device shutdown\n");
		goto done;
	}
	if (iwl_is_rfkill(priv)) {
		IWL_DEBUG_TX(priv, "Aborting flush due to RF Kill\n");
		goto done;
	}

	/*
	 * mac80211 will not push any more frames for transmit
	 * until the flush is completed
	 */
	if (drop) {
		IWL_DEBUG_MAC80211(priv, "send flush command\n");
		if (iwlagn_txfifo_flush(priv, IWL_DROP_ALL)) {
			IWL_ERR(priv, "flush request fail\n");
			goto done;
		}
	}
	IWL_DEBUG_MAC80211(priv, "wait transmit/flush all frames\n");
	iwl_trans_wait_tx_queue_empty(trans(priv));
done:
	mutex_unlock(&priv->mutex);
	IWL_DEBUG_MAC80211(priv, "leave\n");
}

static int iwlagn_mac_remain_on_channel(struct ieee80211_hw *hw,
				     struct ieee80211_channel *channel,
				     enum nl80211_channel_type channel_type,
				     int duration)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);
	struct iwl_rxon_context *ctx = &priv->contexts[IWL_RXON_CTX_PAN];
	int err = 0;

	if (!(priv->shrd->valid_contexts & BIT(IWL_RXON_CTX_PAN)))
		return -EOPNOTSUPP;

	if (!(ctx->interface_modes & BIT(NL80211_IFTYPE_P2P_CLIENT)))
		return -EOPNOTSUPP;

	IWL_DEBUG_MAC80211(priv, "enter\n");
	mutex_lock(&priv->mutex);

	if (test_bit(STATUS_SCAN_HW, &priv->status)) {
		err = -EBUSY;
		goto out;
	}

	priv->hw_roc_channel = channel;
	priv->hw_roc_chantype = channel_type;
	/* convert from ms to TU */
	priv->hw_roc_duration = DIV_ROUND_UP(1000 * duration, 1024);
	priv->hw_roc_start_notified = false;
	cancel_delayed_work(&priv->hw_roc_disable_work);

	if (!ctx->is_active) {
		static const struct iwl_qos_info default_qos_data = {
			.def_qos_parm = {
				.ac[0] = {
					.cw_min = cpu_to_le16(3),
					.cw_max = cpu_to_le16(7),
					.aifsn = 2,
					.edca_txop = cpu_to_le16(1504),
				},
				.ac[1] = {
					.cw_min = cpu_to_le16(7),
					.cw_max = cpu_to_le16(15),
					.aifsn = 2,
					.edca_txop = cpu_to_le16(3008),
				},
				.ac[2] = {
					.cw_min = cpu_to_le16(15),
					.cw_max = cpu_to_le16(1023),
					.aifsn = 3,
				},
				.ac[3] = {
					.cw_min = cpu_to_le16(15),
					.cw_max = cpu_to_le16(1023),
					.aifsn = 7,
				},
			},
		};

		ctx->is_active = true;
		ctx->qos_data = default_qos_data;
		ctx->staging.dev_type = RXON_DEV_TYPE_P2P;
		memcpy(ctx->staging.node_addr,
		       priv->contexts[IWL_RXON_CTX_BSS].staging.node_addr,
		       ETH_ALEN);
		memcpy(ctx->staging.bssid_addr,
		       priv->contexts[IWL_RXON_CTX_BSS].staging.node_addr,
		       ETH_ALEN);
		err = iwlagn_commit_rxon(priv, ctx);
		if (err)
			goto out;
		ctx->staging.filter_flags |= RXON_FILTER_ASSOC_MSK |
					     RXON_FILTER_PROMISC_MSK |
					     RXON_FILTER_CTL2HOST_MSK;

		err = iwlagn_commit_rxon(priv, ctx);
		if (err) {
			iwlagn_disable_roc(priv);
			goto out;
		}
		priv->hw_roc_setup = true;
	}

	err = iwl_scan_initiate(priv, ctx->vif, IWL_SCAN_ROC, channel->band);
	if (err)
		iwlagn_disable_roc(priv);

 out:
	mutex_unlock(&priv->mutex);
	IWL_DEBUG_MAC80211(priv, "leave\n");

	return err;
}

static int iwlagn_mac_cancel_remain_on_channel(struct ieee80211_hw *hw)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);

	if (!(priv->shrd->valid_contexts & BIT(IWL_RXON_CTX_PAN)))
		return -EOPNOTSUPP;

	IWL_DEBUG_MAC80211(priv, "enter\n");
	mutex_lock(&priv->mutex);
	iwl_scan_cancel_timeout(priv, priv->hw_roc_duration);
	iwlagn_disable_roc(priv);
	mutex_unlock(&priv->mutex);
	IWL_DEBUG_MAC80211(priv, "leave\n");

	return 0;
}

static void iwlagn_mac_rssi_callback(struct ieee80211_hw *hw,
			   enum ieee80211_rssi_event rssi_event)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);

	IWL_DEBUG_MAC80211(priv, "enter\n");
	mutex_lock(&priv->mutex);

	if (cfg(priv)->bt_params &&
			cfg(priv)->bt_params->advanced_bt_coexist) {
		if (rssi_event == RSSI_EVENT_LOW)
			priv->bt_enable_pspoll = true;
		else if (rssi_event == RSSI_EVENT_HIGH)
			priv->bt_enable_pspoll = false;

		iwlagn_send_advance_bt_config(priv);
	} else {
		IWL_DEBUG_MAC80211(priv, "Advanced BT coex disabled,"
				"ignoring RSSI callback\n");
	}

	mutex_unlock(&priv->mutex);
	IWL_DEBUG_MAC80211(priv, "leave\n");
}

static int iwlagn_mac_set_tim(struct ieee80211_hw *hw,
			   struct ieee80211_sta *sta, bool set)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);

	queue_work(priv->workqueue, &priv->beacon_update);

	return 0;
}

static int iwlagn_mac_conf_tx(struct ieee80211_hw *hw,
		    struct ieee80211_vif *vif, u16 queue,
		    const struct ieee80211_tx_queue_params *params)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);
	struct iwl_vif_priv *vif_priv = (void *)vif->drv_priv;
	struct iwl_rxon_context *ctx = vif_priv->ctx;
	int q;

	if (WARN_ON(!ctx))
		return -EINVAL;

	IWL_DEBUG_MAC80211(priv, "enter\n");

	if (!iwl_is_ready_rf(priv)) {
		IWL_DEBUG_MAC80211(priv, "leave - RF not ready\n");
		return -EIO;
	}

	if (queue >= AC_NUM) {
		IWL_DEBUG_MAC80211(priv, "leave - queue >= AC_NUM %d\n", queue);
		return 0;
	}

	q = AC_NUM - 1 - queue;

	mutex_lock(&priv->mutex);

	ctx->qos_data.def_qos_parm.ac[q].cw_min =
		cpu_to_le16(params->cw_min);
	ctx->qos_data.def_qos_parm.ac[q].cw_max =
		cpu_to_le16(params->cw_max);
	ctx->qos_data.def_qos_parm.ac[q].aifsn = params->aifs;
	ctx->qos_data.def_qos_parm.ac[q].edca_txop =
			cpu_to_le16((params->txop * 32));

	ctx->qos_data.def_qos_parm.ac[q].reserved1 = 0;

	mutex_unlock(&priv->mutex);

	IWL_DEBUG_MAC80211(priv, "leave\n");
	return 0;
}

static int iwlagn_mac_tx_last_beacon(struct ieee80211_hw *hw)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);

	return priv->ibss_manager == IWL_IBSS_MANAGER;
}

static int iwl_set_mode(struct iwl_priv *priv, struct iwl_rxon_context *ctx)
{
	iwl_connection_init_rx_config(priv, ctx);

	iwlagn_set_rxon_chain(priv, ctx);

	return iwlagn_commit_rxon(priv, ctx);
}

static int iwl_setup_interface(struct iwl_priv *priv,
			       struct iwl_rxon_context *ctx)
{
	struct ieee80211_vif *vif = ctx->vif;
	int err;

	lockdep_assert_held(&priv->mutex);

	/*
	 * This variable will be correct only when there's just
	 * a single context, but all code using it is for hardware
	 * that supports only one context.
	 */
	priv->iw_mode = vif->type;

	ctx->is_active = true;

	err = iwl_set_mode(priv, ctx);
	if (err) {
		if (!ctx->always_active)
			ctx->is_active = false;
		return err;
	}

	if (cfg(priv)->bt_params && cfg(priv)->bt_params->advanced_bt_coexist &&
	    vif->type == NL80211_IFTYPE_ADHOC) {
		/*
		 * pretend to have high BT traffic as long as we
		 * are operating in IBSS mode, as this will cause
		 * the rate scaling etc. to behave as intended.
		 */
		priv->bt_traffic_load = IWL_BT_COEX_TRAFFIC_LOAD_HIGH;
	}

	return 0;
}

static int iwlagn_mac_add_interface(struct ieee80211_hw *hw,
			     struct ieee80211_vif *vif)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);
	struct iwl_vif_priv *vif_priv = (void *)vif->drv_priv;
	struct iwl_rxon_context *tmp, *ctx = NULL;
	int err;
	enum nl80211_iftype viftype = ieee80211_vif_type_p2p(vif);
	bool reset = false;

	IWL_DEBUG_MAC80211(priv, "enter: type %d, addr %pM\n",
			   viftype, vif->addr);

	cancel_delayed_work_sync(&priv->hw_roc_disable_work);

	mutex_lock(&priv->mutex);

	iwlagn_disable_roc(priv);

	if (!iwl_is_ready_rf(priv)) {
		IWL_WARN(priv, "Try to add interface when device not ready\n");
		err = -EINVAL;
		goto out;
	}

	for_each_context(priv, tmp) {
		u32 possible_modes =
			tmp->interface_modes | tmp->exclusive_interface_modes;

		if (tmp->vif) {
			/* On reset we need to add the same interface again */
			if (tmp->vif == vif) {
				reset = true;
				ctx = tmp;
				break;
			}

			/* check if this busy context is exclusive */
			if (tmp->exclusive_interface_modes &
						BIT(tmp->vif->type)) {
				err = -EINVAL;
				goto out;
			}
			continue;
		}

		if (!(possible_modes & BIT(viftype)))
			continue;

		/* have maybe usable context w/o interface */
		ctx = tmp;
		break;
	}

	if (!ctx) {
		err = -EOPNOTSUPP;
		goto out;
	}

	vif_priv->ctx = ctx;
	ctx->vif = vif;

	err = iwl_setup_interface(priv, ctx);
	if (!err || reset)
		goto out;

	ctx->vif = NULL;
	priv->iw_mode = NL80211_IFTYPE_STATION;
 out:
	mutex_unlock(&priv->mutex);

	IWL_DEBUG_MAC80211(priv, "leave\n");
	return err;
}

static void iwl_teardown_interface(struct iwl_priv *priv,
				   struct ieee80211_vif *vif,
				   bool mode_change)
{
	struct iwl_rxon_context *ctx = iwl_rxon_ctx_from_vif(vif);

	lockdep_assert_held(&priv->mutex);

	if (priv->scan_vif == vif) {
		iwl_scan_cancel_timeout(priv, 200);
		iwl_force_scan_end(priv);
	}

	if (!mode_change) {
		iwl_set_mode(priv, ctx);
		if (!ctx->always_active)
			ctx->is_active = false;
	}

	/*
	 * When removing the IBSS interface, overwrite the
	 * BT traffic load with the stored one from the last
	 * notification, if any. If this is a device that
	 * doesn't implement this, this has no effect since
	 * both values are the same and zero.
	 */
	if (vif->type == NL80211_IFTYPE_ADHOC)
		priv->bt_traffic_load = priv->last_bt_traffic_load;
}

static void iwlagn_mac_remove_interface(struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);
	struct iwl_rxon_context *ctx = iwl_rxon_ctx_from_vif(vif);

	IWL_DEBUG_MAC80211(priv, "enter\n");

	mutex_lock(&priv->mutex);

	if (WARN_ON(ctx->vif != vif)) {
		struct iwl_rxon_context *tmp;
		IWL_ERR(priv, "ctx->vif = %p, vif = %p\n", ctx->vif, vif);
		for_each_context(priv, tmp)
			IWL_ERR(priv, "\tID = %d:\tctx = %p\tctx->vif = %p\n",
				tmp->ctxid, tmp, tmp->vif);
	}
	ctx->vif = NULL;

	iwl_teardown_interface(priv, vif, false);

	mutex_unlock(&priv->mutex);

	IWL_DEBUG_MAC80211(priv, "leave\n");

}

static int iwlagn_mac_change_interface(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif,
				enum nl80211_iftype newtype, bool newp2p)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);
	struct iwl_rxon_context *ctx = iwl_rxon_ctx_from_vif(vif);
	struct iwl_rxon_context *bss_ctx = &priv->contexts[IWL_RXON_CTX_BSS];
	struct iwl_rxon_context *tmp;
	enum nl80211_iftype newviftype = newtype;
	u32 interface_modes;
	int err;

	IWL_DEBUG_MAC80211(priv, "enter\n");

	newtype = ieee80211_iftype_p2p(newtype, newp2p);

	mutex_lock(&priv->mutex);

	if (!ctx->vif || !iwl_is_ready_rf(priv)) {
		/*
		 * Huh? But wait ... this can maybe happen when
		 * we're in the middle of a firmware restart!
		 */
		err = -EBUSY;
		goto out;
	}

	interface_modes = ctx->interface_modes | ctx->exclusive_interface_modes;

	if (!(interface_modes & BIT(newtype))) {
		err = -EBUSY;
		goto out;
	}

	/*
	 * Refuse a change that should be done by moving from the PAN
	 * context to the BSS context instead, if the BSS context is
	 * available and can support the new interface type.
	 */
	if (ctx->ctxid == IWL_RXON_CTX_PAN && !bss_ctx->vif &&
	    (bss_ctx->interface_modes & BIT(newtype) ||
	     bss_ctx->exclusive_interface_modes & BIT(newtype))) {
		BUILD_BUG_ON(NUM_IWL_RXON_CTX != 2);
		err = -EBUSY;
		goto out;
	}

	if (ctx->exclusive_interface_modes & BIT(newtype)) {
		for_each_context(priv, tmp) {
			if (ctx == tmp)
				continue;

			if (!tmp->vif)
				continue;

			/*
			 * The current mode switch would be exclusive, but
			 * another context is active ... refuse the switch.
			 */
			err = -EBUSY;
			goto out;
		}
	}

	/* success */
	iwl_teardown_interface(priv, vif, true);
	vif->type = newviftype;
	vif->p2p = newp2p;
	err = iwl_setup_interface(priv, ctx);
	WARN_ON(err);
	/*
	 * We've switched internally, but submitting to the
	 * device may have failed for some reason. Mask this
	 * error, because otherwise mac80211 will not switch
	 * (and set the interface type back) and we'll be
	 * out of sync with it.
	 */
	err = 0;

 out:
	mutex_unlock(&priv->mutex);
	IWL_DEBUG_MAC80211(priv, "leave\n");

	return err;
}

static int iwlagn_mac_hw_scan(struct ieee80211_hw *hw,
		    struct ieee80211_vif *vif,
		    struct cfg80211_scan_request *req)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);
	int ret;

	IWL_DEBUG_MAC80211(priv, "enter\n");

	if (req->n_channels == 0)
		return -EINVAL;

	mutex_lock(&priv->mutex);

	/*
	 * If an internal scan is in progress, just set
	 * up the scan_request as per above.
	 */
	if (priv->scan_type != IWL_SCAN_NORMAL) {
		IWL_DEBUG_SCAN(priv,
			       "SCAN request during internal scan - defer\n");
		priv->scan_request = req;
		priv->scan_vif = vif;
		ret = 0;
	} else {
		priv->scan_request = req;
		priv->scan_vif = vif;
		/*
		 * mac80211 will only ask for one band at a time
		 * so using channels[0] here is ok
		 */
		ret = iwl_scan_initiate(priv, vif, IWL_SCAN_NORMAL,
					req->channels[0]->band);
		if (ret) {
			priv->scan_request = NULL;
			priv->scan_vif = NULL;
		}
	}

	IWL_DEBUG_MAC80211(priv, "leave\n");

	mutex_unlock(&priv->mutex);

	return ret;
}

static void iwl_sta_modify_ps_wake(struct iwl_priv *priv, int sta_id)
{
	struct iwl_addsta_cmd cmd = {
		.mode = STA_CONTROL_MODIFY_MSK,
		.station_flags_msk = STA_FLG_PWR_SAVE_MSK,
		.sta.sta_id = sta_id,
	};

	iwl_send_add_sta(priv, &cmd, CMD_ASYNC);
}

static void iwlagn_mac_sta_notify(struct ieee80211_hw *hw,
			   struct ieee80211_vif *vif,
			   enum sta_notify_cmd cmd,
			   struct ieee80211_sta *sta)
{
	struct iwl_priv *priv = IWL_MAC80211_GET_DVM(hw);
	struct iwl_station_priv *sta_priv = (void *)sta->drv_priv;
	int sta_id;

	IWL_DEBUG_MAC80211(priv, "enter\n");

	switch (cmd) {
	case STA_NOTIFY_SLEEP:
		WARN_ON(!sta_priv->client);
		sta_priv->asleep = true;
		if (atomic_read(&sta_priv->pending_frames) > 0)
			ieee80211_sta_block_awake(hw, sta, true);
		break;
	case STA_NOTIFY_AWAKE:
		WARN_ON(!sta_priv->client);
		if (!sta_priv->asleep)
			break;
		sta_priv->asleep = false;
		sta_id = iwl_sta_id(sta);
		if (sta_id != IWL_INVALID_STATION)
			iwl_sta_modify_ps_wake(priv, sta_id);
		break;
	default:
		break;
	}
	IWL_DEBUG_MAC80211(priv, "leave\n");
}

struct ieee80211_ops iwlagn_hw_ops = {
	.tx = iwlagn_mac_tx,
	.start = iwlagn_mac_start,
	.stop = iwlagn_mac_stop,
#ifdef CONFIG_PM_SLEEP
	.suspend = iwlagn_mac_suspend,
	.resume = iwlagn_mac_resume,
#endif
	.add_interface = iwlagn_mac_add_interface,
	.remove_interface = iwlagn_mac_remove_interface,
	.change_interface = iwlagn_mac_change_interface,
	.config = iwlagn_mac_config,
	.configure_filter = iwlagn_configure_filter,
	.set_key = iwlagn_mac_set_key,
	.update_tkip_key = iwlagn_mac_update_tkip_key,
	.set_rekey_data = iwlagn_mac_set_rekey_data,
	.conf_tx = iwlagn_mac_conf_tx,
	.bss_info_changed = iwlagn_bss_info_changed,
	.ampdu_action = iwlagn_mac_ampdu_action,
	.hw_scan = iwlagn_mac_hw_scan,
	.sta_notify = iwlagn_mac_sta_notify,
	.sta_state = iwlagn_mac_sta_state,
	.channel_switch = iwlagn_mac_channel_switch,
	.flush = iwlagn_mac_flush,
	.tx_last_beacon = iwlagn_mac_tx_last_beacon,
	.remain_on_channel = iwlagn_mac_remain_on_channel,
	.cancel_remain_on_channel = iwlagn_mac_cancel_remain_on_channel,
	.rssi_callback = iwlagn_mac_rssi_callback,
	CFG80211_TESTMODE_CMD(iwlagn_mac_testmode_cmd)
	CFG80211_TESTMODE_DUMP(iwlagn_mac_testmode_dump)
	.set_tim = iwlagn_mac_set_tim,
};

/* This function both allocates and initializes hw and priv. */
struct ieee80211_hw *iwl_alloc_all(void)
{
	struct iwl_priv *priv;
	struct iwl_op_mode *op_mode;
	/* mac80211 allocates memory for this device instance, including
	 *   space for this driver's private structure */
	struct ieee80211_hw *hw;

	hw = ieee80211_alloc_hw(sizeof(struct iwl_priv) +
				sizeof(struct iwl_op_mode), &iwlagn_hw_ops);
	if (!hw)
		goto out;

	op_mode = hw->priv;
	priv = IWL_OP_MODE_GET_DVM(op_mode);
	priv->hw = hw;

out:
	return hw;
}
