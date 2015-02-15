/******************************************************************************
 *
 * Copyright(c) 2008 - 2012 Intel Corporation. All rights reserved.
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
#include <linux/delay.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <net/mac80211.h>
#include <linux/etherdevice.h>
#include <asm/unaligned.h>
#include <linux/stringify.h>

#include "iwl-eeprom.h"
#include "iwl-dev.h"
#include "iwl-core.h"
#include "iwl-io.h"
#include "iwl-agn.h"
#include "iwl-agn-hw.h"
#include "iwl-trans.h"
#include "iwl-shared.h"
#include "iwl-cfg.h"

/* Highest firmware API version supported */
#define IWL6000_UCODE_API_MAX 6
#define IWL6050_UCODE_API_MAX 5
#define IWL6000G2_UCODE_API_MAX 6

/* Oldest version we won't warn about */
#define IWL6000_UCODE_API_OK 4
#define IWL6000G2_UCODE_API_OK 5
#define IWL6050_UCODE_API_OK 5
#define IWL6000G2B_UCODE_API_OK 6

/* Lowest firmware API version supported */
#define IWL6000_UCODE_API_MIN 4
#define IWL6050_UCODE_API_MIN 4
#define IWL6000G2_UCODE_API_MIN 4

#define IWL6000_FW_PRE "iwlwifi-6000-"
#define IWL6000_MODULE_FIRMWARE(api) IWL6000_FW_PRE __stringify(api) ".ucode"

#define IWL6050_FW_PRE "iwlwifi-6050-"
#define IWL6050_MODULE_FIRMWARE(api) IWL6050_FW_PRE __stringify(api) ".ucode"

#define IWL6005_FW_PRE "iwlwifi-6000g2a-"
#define IWL6005_MODULE_FIRMWARE(api) IWL6005_FW_PRE __stringify(api) ".ucode"

#define IWL6030_FW_PRE "iwlwifi-6000g2b-"
#define IWL6030_MODULE_FIRMWARE(api) IWL6030_FW_PRE __stringify(api) ".ucode"

static void iwl6000_set_ct_threshold(struct iwl_priv *priv)
{
	/* want Celsius */
	hw_params(priv).ct_kill_threshold = CT_KILL_THRESHOLD;
	hw_params(priv).ct_kill_exit_threshold = CT_KILL_EXIT_THRESHOLD;
}

static void iwl6050_additional_nic_config(struct iwl_priv *priv)
{
	/* Indicate calibration version to uCode. */
	if (iwl_eeprom_calib_version(priv->shrd) >= 6)
		iwl_set_bit(trans(priv), CSR_GP_DRIVER_REG,
				CSR_GP_DRIVER_REG_BIT_CALIB_VERSION6);
}

static void iwl6150_additional_nic_config(struct iwl_priv *priv)
{
	/* Indicate calibration version to uCode. */
	if (iwl_eeprom_calib_version(priv->shrd) >= 6)
		iwl_set_bit(trans(priv), CSR_GP_DRIVER_REG,
				CSR_GP_DRIVER_REG_BIT_CALIB_VERSION6);
	iwl_set_bit(trans(priv), CSR_GP_DRIVER_REG,
		    CSR_GP_DRIVER_REG_BIT_6050_1x2);
}

static void iwl6000i_additional_nic_config(struct iwl_priv *priv)
{
	/* 2x2 IPA phy type */
	iwl_write32(trans(priv), CSR_GP_DRIVER_REG,
		     CSR_GP_DRIVER_REG_BIT_RADIO_SKU_2x2_IPA);
}

/* NIC configuration for 6000 series */
static void iwl6000_nic_config(struct iwl_priv *priv)
{
	iwl_rf_config(priv);

	/* do additional nic configuration if needed */
	if (cfg(priv)->additional_nic_config)
		cfg(priv)->additional_nic_config(priv);
}

static const struct iwl_sensitivity_ranges iwl6000_sensitivity = {
	.min_nrg_cck = 110,
	.auto_corr_min_ofdm = 80,
	.auto_corr_min_ofdm_mrc = 128,
	.auto_corr_min_ofdm_x1 = 105,
	.auto_corr_min_ofdm_mrc_x1 = 192,

	.auto_corr_max_ofdm = 145,
	.auto_corr_max_ofdm_mrc = 232,
	.auto_corr_max_ofdm_x1 = 110,
	.auto_corr_max_ofdm_mrc_x1 = 232,

	.auto_corr_min_cck = 125,
	.auto_corr_max_cck = 175,
	.auto_corr_min_cck_mrc = 160,
	.auto_corr_max_cck_mrc = 310,
	.nrg_th_cck = 110,
	.nrg_th_ofdm = 110,

	.barker_corr_th_min = 190,
	.barker_corr_th_min_mrc = 336,
	.nrg_th_cca = 62,
};

static void iwl6000_hw_set_hw_params(struct iwl_priv *priv)
{
	hw_params(priv).ht40_channel =  BIT(IEEE80211_BAND_2GHZ) |
					BIT(IEEE80211_BAND_5GHZ);

	hw_params(priv).tx_chains_num =
		num_of_ant(hw_params(priv).valid_tx_ant);
	if (cfg(priv)->rx_with_siso_diversity)
		hw_params(priv).rx_chains_num = 1;
	else
		hw_params(priv).rx_chains_num =
			num_of_ant(hw_params(priv).valid_rx_ant);

	iwl6000_set_ct_threshold(priv);

	/* Set initial sensitivity parameters */
	hw_params(priv).sens = &iwl6000_sensitivity;

}

static int iwl6000_hw_channel_switch(struct iwl_priv *priv,
				     struct ieee80211_channel_switch *ch_switch)
{
	/*
	 * MULTI-FIXME
	 * See iwlagn_mac_channel_switch.
	 */
	struct iwl_rxon_context *ctx = &priv->contexts[IWL_RXON_CTX_BSS];
	struct iwl6000_channel_switch_cmd cmd;
	const struct iwl_channel_info *ch_info;
	u32 switch_time_in_usec, ucode_switch_time;
	u16 ch;
	u32 tsf_low;
	u8 switch_count;
	u16 beacon_interval = le16_to_cpu(ctx->timing.beacon_interval);
	struct ieee80211_vif *vif = ctx->vif;
	struct iwl_host_cmd hcmd = {
		.id = REPLY_CHANNEL_SWITCH,
		.len = { sizeof(cmd), },
		.flags = CMD_SYNC,
		.data = { &cmd, },
	};

	cmd.band = priv->band == IEEE80211_BAND_2GHZ;
	ch = ch_switch->channel->hw_value;
	IWL_DEBUG_11H(priv, "channel switch from %u to %u\n",
		      ctx->active.channel, ch);
	cmd.channel = cpu_to_le16(ch);
	cmd.rxon_flags = ctx->staging.flags;
	cmd.rxon_filter_flags = ctx->staging.filter_flags;
	switch_count = ch_switch->count;
	tsf_low = ch_switch->timestamp & 0x0ffffffff;
	/*
	 * calculate the ucode channel switch time
	 * adding TSF as one of the factor for when to switch
	 */
	if ((priv->ucode_beacon_time > tsf_low) && beacon_interval) {
		if (switch_count > ((priv->ucode_beacon_time - tsf_low) /
		    beacon_interval)) {
			switch_count -= (priv->ucode_beacon_time -
				tsf_low) / beacon_interval;
		} else
			switch_count = 0;
	}
	if (switch_count <= 1)
		cmd.switch_time = cpu_to_le32(priv->ucode_beacon_time);
	else {
		switch_time_in_usec =
			vif->bss_conf.beacon_int * switch_count * TIME_UNIT;
		ucode_switch_time = iwl_usecs_to_beacons(priv,
							 switch_time_in_usec,
							 beacon_interval);
		cmd.switch_time = iwl_add_beacon_time(priv,
						      priv->ucode_beacon_time,
						      ucode_switch_time,
						      beacon_interval);
	}
	IWL_DEBUG_11H(priv, "uCode time for the switch is 0x%x\n",
		      cmd.switch_time);
	ch_info = iwl_get_channel_info(priv, priv->band, ch);
	if (ch_info)
		cmd.expect_beacon = is_channel_radar(ch_info);
	else {
		IWL_ERR(priv, "invalid channel switch from %u to %u\n",
			ctx->active.channel, ch);
		return -EFAULT;
	}

	return iwl_dvm_send_cmd(priv, &hcmd);
}

static struct iwl_lib_ops iwl6000_lib = {
	.set_hw_params = iwl6000_hw_set_hw_params,
	.set_channel_switch = iwl6000_hw_channel_switch,
	.nic_config = iwl6000_nic_config,
	.eeprom_ops = {
		.regulatory_bands = {
			EEPROM_REG_BAND_1_CHANNELS,
			EEPROM_REG_BAND_2_CHANNELS,
			EEPROM_REG_BAND_3_CHANNELS,
			EEPROM_REG_BAND_4_CHANNELS,
			EEPROM_REG_BAND_5_CHANNELS,
			EEPROM_6000_REG_BAND_24_HT40_CHANNELS,
			EEPROM_REG_BAND_52_HT40_CHANNELS
		},
		.enhanced_txpower = true,
	},
	.temperature = iwlagn_temperature,
};

static struct iwl_lib_ops iwl6030_lib = {
	.set_hw_params = iwl6000_hw_set_hw_params,
	.set_channel_switch = iwl6000_hw_channel_switch,
	.nic_config = iwl6000_nic_config,
	.eeprom_ops = {
		.regulatory_bands = {
			EEPROM_REG_BAND_1_CHANNELS,
			EEPROM_REG_BAND_2_CHANNELS,
			EEPROM_REG_BAND_3_CHANNELS,
			EEPROM_REG_BAND_4_CHANNELS,
			EEPROM_REG_BAND_5_CHANNELS,
			EEPROM_6000_REG_BAND_24_HT40_CHANNELS,
			EEPROM_REG_BAND_52_HT40_CHANNELS
		},
		.enhanced_txpower = true,
	},
	.temperature = iwlagn_temperature,
};

static const struct iwl_base_params iwl6000_base_params = {
	.eeprom_size = OTP_LOW_IMAGE_SIZE,
	.num_of_queues = IWLAGN_NUM_QUEUES,
	.num_of_ampdu_queues = IWLAGN_NUM_AMPDU_QUEUES,
	.pll_cfg_val = 0,
	.max_ll_items = OTP_MAX_LL_ITEMS_6x00,
	.shadow_ram_support = true,
	.led_compensation = 51,
	.adv_thermal_throttle = true,
	.support_ct_kill_exit = true,
	.plcp_delta_threshold = IWL_MAX_PLCP_ERR_THRESHOLD_DEF,
	.chain_noise_scale = 1000,
	.wd_timeout = IWL_DEF_WD_TIMEOUT,
	.max_event_log_size = 512,
	.shadow_reg_enable = true,
};

static const struct iwl_base_params iwl6050_base_params = {
	.eeprom_size = OTP_LOW_IMAGE_SIZE,
	.num_of_queues = IWLAGN_NUM_QUEUES,
	.num_of_ampdu_queues = IWLAGN_NUM_AMPDU_QUEUES,
	.pll_cfg_val = 0,
	.max_ll_items = OTP_MAX_LL_ITEMS_6x50,
	.shadow_ram_support = true,
	.led_compensation = 51,
	.adv_thermal_throttle = true,
	.support_ct_kill_exit = true,
	.plcp_delta_threshold = IWL_MAX_PLCP_ERR_THRESHOLD_DEF,
	.chain_noise_scale = 1500,
	.wd_timeout = IWL_DEF_WD_TIMEOUT,
	.max_event_log_size = 1024,
	.shadow_reg_enable = true,
};

static const struct iwl_base_params iwl6000_g2_base_params = {
	.eeprom_size = OTP_LOW_IMAGE_SIZE,
	.num_of_queues = IWLAGN_NUM_QUEUES,
	.num_of_ampdu_queues = IWLAGN_NUM_AMPDU_QUEUES,
	.pll_cfg_val = 0,
	.max_ll_items = OTP_MAX_LL_ITEMS_6x00,
	.shadow_ram_support = true,
	.led_compensation = 57,
	.adv_thermal_throttle = true,
	.support_ct_kill_exit = true,
	.plcp_delta_threshold = IWL_MAX_PLCP_ERR_THRESHOLD_DEF,
	.chain_noise_scale = 1000,
	.wd_timeout = IWL_LONG_WD_TIMEOUT,
	.max_event_log_size = 512,
	.shadow_reg_enable = true,
};

static const struct iwl_ht_params iwl6000_ht_params = {
	.ht_greenfield_support = true,
	.use_rts_for_aggregation = true, /* use rts/cts protection */
};

static const struct iwl_bt_params iwl6000_bt_params = {
	/* Due to bluetooth, we transmit 2.4 GHz probes only on antenna A */
	.advanced_bt_coexist = true,
	.agg_time_limit = BT_AGG_THRESHOLD_DEF,
	.bt_init_traffic_load = IWL_BT_COEX_TRAFFIC_LOAD_NONE,
	.bt_prio_boost = IWLAGN_BT_PRIO_BOOST_DEFAULT,
	.bt_sco_disable = true,
};

#define IWL_DEVICE_6005						\
	.fw_name_pre = IWL6005_FW_PRE,				\
	.ucode_api_max = IWL6000G2_UCODE_API_MAX,		\
	.ucode_api_ok = IWL6000G2_UCODE_API_OK,			\
	.ucode_api_min = IWL6000G2_UCODE_API_MIN,		\
	.max_inst_size = IWL60_RTC_INST_SIZE,			\
	.max_data_size = IWL60_RTC_DATA_SIZE,			\
	.eeprom_ver = EEPROM_6005_EEPROM_VERSION,		\
	.eeprom_calib_ver = EEPROM_6005_TX_POWER_VERSION,	\
	.lib = &iwl6000_lib,					\
	.base_params = &iwl6000_g2_base_params,			\
	.need_temp_offset_calib = true,				\
	.led_mode = IWL_LED_RF_STATE

const struct iwl_cfg iwl6005_2agn_cfg = {
	.name = "Intel(R) Centrino(R) Advanced-N 6205 AGN",
	IWL_DEVICE_6005,
	.ht_params = &iwl6000_ht_params,
};

const struct iwl_cfg iwl6005_2abg_cfg = {
	.name = "Intel(R) Centrino(R) Advanced-N 6205 ABG",
	IWL_DEVICE_6005,
};

const struct iwl_cfg iwl6005_2bg_cfg = {
	.name = "Intel(R) Centrino(R) Advanced-N 6205 BG",
	IWL_DEVICE_6005,
};

const struct iwl_cfg iwl6005_2agn_sff_cfg = {
	.name = "Intel(R) Centrino(R) Advanced-N 6205S AGN",
	IWL_DEVICE_6005,
	.ht_params = &iwl6000_ht_params,
};

const struct iwl_cfg iwl6005_2agn_d_cfg = {
	.name = "Intel(R) Centrino(R) Advanced-N 6205D AGN",
	IWL_DEVICE_6005,
	.ht_params = &iwl6000_ht_params,
};

const struct iwl_cfg iwl6005_2agn_mow1_cfg = {
	.name = "Intel(R) Centrino(R) Advanced-N 6206 AGN",
	IWL_DEVICE_6005,
	.ht_params = &iwl6000_ht_params,
};

const struct iwl_cfg iwl6005_2agn_mow2_cfg = {
	.name = "Intel(R) Centrino(R) Advanced-N 6207 AGN",
	IWL_DEVICE_6005,
	.ht_params = &iwl6000_ht_params,
};

#define IWL_DEVICE_6030						\
	.fw_name_pre = IWL6030_FW_PRE,				\
	.ucode_api_max = IWL6000G2_UCODE_API_MAX,		\
	.ucode_api_ok = IWL6000G2B_UCODE_API_OK,		\
	.ucode_api_min = IWL6000G2_UCODE_API_MIN,		\
	.max_inst_size = IWL60_RTC_INST_SIZE,			\
	.max_data_size = IWL60_RTC_DATA_SIZE,			\
	.eeprom_ver = EEPROM_6030_EEPROM_VERSION,		\
	.eeprom_calib_ver = EEPROM_6030_TX_POWER_VERSION,	\
	.lib = &iwl6030_lib,					\
	.base_params = &iwl6000_g2_base_params,			\
	.bt_params = &iwl6000_bt_params,			\
	.need_temp_offset_calib = true,				\
	.led_mode = IWL_LED_RF_STATE,				\
	.adv_pm = true						\

const struct iwl_cfg iwl6030_2agn_cfg = {
	.name = "Intel(R) Centrino(R) Advanced-N 6230 AGN",
	IWL_DEVICE_6030,
	.ht_params = &iwl6000_ht_params,
};

const struct iwl_cfg iwl6030_2abg_cfg = {
	.name = "Intel(R) Centrino(R) Advanced-N 6230 ABG",
	IWL_DEVICE_6030,
};

const struct iwl_cfg iwl6030_2bgn_cfg = {
	.name = "Intel(R) Centrino(R) Advanced-N 6230 BGN",
	IWL_DEVICE_6030,
	.ht_params = &iwl6000_ht_params,
};

const struct iwl_cfg iwl6030_2bg_cfg = {
	.name = "Intel(R) Centrino(R) Advanced-N 6230 BG",
	IWL_DEVICE_6030,
};

const struct iwl_cfg iwl6035_2agn_cfg = {
	.name = "Intel(R) Centrino(R) Advanced-N 6235 AGN",
	IWL_DEVICE_6030,
	.ht_params = &iwl6000_ht_params,
};

const struct iwl_cfg iwl1030_bgn_cfg = {
	.name = "Intel(R) Centrino(R) Wireless-N 1030 BGN",
	IWL_DEVICE_6030,
	.ht_params = &iwl6000_ht_params,
};

const struct iwl_cfg iwl1030_bg_cfg = {
	.name = "Intel(R) Centrino(R) Wireless-N 1030 BG",
	IWL_DEVICE_6030,
};

const struct iwl_cfg iwl130_bgn_cfg = {
	.name = "Intel(R) Centrino(R) Wireless-N 130 BGN",
	IWL_DEVICE_6030,
	.ht_params = &iwl6000_ht_params,
	.rx_with_siso_diversity = true,
};

const struct iwl_cfg iwl130_bg_cfg = {
	.name = "Intel(R) Centrino(R) Wireless-N 130 BG",
	IWL_DEVICE_6030,
	.rx_with_siso_diversity = true,
};

/*
 * "i": Internal configuration, use internal Power Amplifier
 */
#define IWL_DEVICE_6000i					\
	.fw_name_pre = IWL6000_FW_PRE,				\
	.ucode_api_max = IWL6000_UCODE_API_MAX,			\
	.ucode_api_ok = IWL6000_UCODE_API_OK,			\
	.ucode_api_min = IWL6000_UCODE_API_MIN,			\
	.max_inst_size = IWL60_RTC_INST_SIZE,			\
	.max_data_size = IWL60_RTC_DATA_SIZE,			\
	.valid_tx_ant = ANT_BC,		/* .cfg overwrite */	\
	.valid_rx_ant = ANT_BC,		/* .cfg overwrite */	\
	.eeprom_ver = EEPROM_6000_EEPROM_VERSION,		\
	.eeprom_calib_ver = EEPROM_6000_TX_POWER_VERSION,	\
	.lib = &iwl6000_lib,					\
	.additional_nic_config = iwl6000i_additional_nic_config,\
	.base_params = &iwl6000_base_params,			\
	.led_mode = IWL_LED_BLINK

const struct iwl_cfg iwl6000i_2agn_cfg = {
	.name = "Intel(R) Centrino(R) Advanced-N 6200 AGN",
	IWL_DEVICE_6000i,
	.ht_params = &iwl6000_ht_params,
};

const struct iwl_cfg iwl6000i_2abg_cfg = {
	.name = "Intel(R) Centrino(R) Advanced-N 6200 ABG",
	IWL_DEVICE_6000i,
};

const struct iwl_cfg iwl6000i_2bg_cfg = {
	.name = "Intel(R) Centrino(R) Advanced-N 6200 BG",
	IWL_DEVICE_6000i,
};

#define IWL_DEVICE_6050						\
	.fw_name_pre = IWL6050_FW_PRE,				\
	.ucode_api_max = IWL6050_UCODE_API_MAX,			\
	.ucode_api_min = IWL6050_UCODE_API_MIN,			\
	.max_inst_size = IWL60_RTC_INST_SIZE,			\
	.max_data_size = IWL60_RTC_DATA_SIZE,			\
	.valid_tx_ant = ANT_AB,		/* .cfg overwrite */	\
	.valid_rx_ant = ANT_AB,		/* .cfg overwrite */	\
	.lib = &iwl6000_lib,					\
	.additional_nic_config = iwl6050_additional_nic_config,	\
	.eeprom_ver = EEPROM_6050_EEPROM_VERSION,		\
	.eeprom_calib_ver = EEPROM_6050_TX_POWER_VERSION,	\
	.base_params = &iwl6050_base_params,			\
	.led_mode = IWL_LED_BLINK,				\
	.internal_wimax_coex = true

const struct iwl_cfg iwl6050_2agn_cfg = {
	.name = "Intel(R) Centrino(R) Advanced-N + WiMAX 6250 AGN",
	IWL_DEVICE_6050,
	.ht_params = &iwl6000_ht_params,
};

const struct iwl_cfg iwl6050_2abg_cfg = {
	.name = "Intel(R) Centrino(R) Advanced-N + WiMAX 6250 ABG",
	IWL_DEVICE_6050,
};

#define IWL_DEVICE_6150						\
	.fw_name_pre = IWL6050_FW_PRE,				\
	.ucode_api_max = IWL6050_UCODE_API_MAX,			\
	.ucode_api_min = IWL6050_UCODE_API_MIN,			\
	.max_inst_size = IWL60_RTC_INST_SIZE,			\
	.max_data_size = IWL60_RTC_DATA_SIZE,			\
	.lib = &iwl6000_lib,					\
	.additional_nic_config = iwl6150_additional_nic_config,	\
	.eeprom_ver = EEPROM_6150_EEPROM_VERSION,		\
	.eeprom_calib_ver = EEPROM_6150_TX_POWER_VERSION,	\
	.base_params = &iwl6050_base_params,			\
	.led_mode = IWL_LED_BLINK,				\
	.internal_wimax_coex = true

const struct iwl_cfg iwl6150_bgn_cfg = {
	.name = "Intel(R) Centrino(R) Wireless-N + WiMAX 6150 BGN",
	IWL_DEVICE_6150,
	.ht_params = &iwl6000_ht_params,
};

const struct iwl_cfg iwl6150_bg_cfg = {
	.name = "Intel(R) Centrino(R) Wireless-N + WiMAX 6150 BG",
	IWL_DEVICE_6150,
};

const struct iwl_cfg iwl6000_3agn_cfg = {
	.name = "Intel(R) Centrino(R) Ultimate-N 6300 AGN",
	.fw_name_pre = IWL6000_FW_PRE,
	.ucode_api_max = IWL6000_UCODE_API_MAX,
	.ucode_api_ok = IWL6000_UCODE_API_OK,
	.ucode_api_min = IWL6000_UCODE_API_MIN,
	.max_inst_size = IWL60_RTC_INST_SIZE,
	.max_data_size = IWL60_RTC_DATA_SIZE,
	.eeprom_ver = EEPROM_6000_EEPROM_VERSION,
	.eeprom_calib_ver = EEPROM_6000_TX_POWER_VERSION,
	.lib = &iwl6000_lib,
	.base_params = &iwl6000_base_params,
	.ht_params = &iwl6000_ht_params,
	.led_mode = IWL_LED_BLINK,
};

MODULE_FIRMWARE(IWL6000_MODULE_FIRMWARE(IWL6000_UCODE_API_OK));
MODULE_FIRMWARE(IWL6050_MODULE_FIRMWARE(IWL6050_UCODE_API_OK));
MODULE_FIRMWARE(IWL6005_MODULE_FIRMWARE(IWL6000G2_UCODE_API_OK));
MODULE_FIRMWARE(IWL6030_MODULE_FIRMWARE(IWL6000G2B_UCODE_API_OK));
