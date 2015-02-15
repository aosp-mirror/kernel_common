/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * HTC: 8x26 machine driver which defines board-specific data
 * Copy from sound/soc/msm/msm8226.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/io.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/jack.h>
#include <sound/q6afe-v2.h>
#include <asm/mach-types.h>
#include <mach/socinfo.h>
#include <mach/subsystem_notif.h>
#include "../../../sound/soc/msm/qdsp6v2/msm-pcm-routing-v2.h"
#include <sound/q6core.h>
#include "../../../sound/soc/codecs/wcd9xxx-common.h"
#include "../../../sound/soc/codecs/wcd9306.h"

#include <mach/htc_acoustic_alsa.h>

#undef pr_info
#undef pr_err
#define pr_info(fmt, ...) pr_aud_info(fmt, ##__VA_ARGS__)
#define pr_err(fmt, ...) pr_aud_err(fmt, ##__VA_ARGS__)

#define HTC_FEATURES 1

struct request_gpio {
	int gpio_no;
	char *gpio_name;
};

static struct spk_config {
	int init;
	struct request_gpio gpio[2];
} htc_spk_config = {

	.init = 0,
	.gpio = {
			{ .gpio_name = "spk-l-gpio-en",},
			{ .gpio_name = "spk-r-gpio-en",},
	},
};

static struct rcv_config {
	int init;
	struct request_gpio gpio[2];
} htc_rcv_config = {

	.init = 0,
	.gpio = {
			{ .gpio_name = "rcv-gpio-sel",},
			{ .gpio_name = "rcv-gpio-en",},
	},
};

static struct mi2s_config {
	int init;
	struct request_gpio gpio[4];
	unsigned int afe_port_id;
} htc_mi2s_config = {

	.init = 0,
	.gpio = {
			{ .gpio_name = "mi2s-gpio-sck",},
			{ .gpio_name = "mi2s-gpio-ws",},
			{ .gpio_name = "mi2s-gpio-sda0",},
			{ .gpio_name = "mi2s-gpio-sda1",},
	},
	.afe_port_id = 0,
};

struct mi2s_clk {
	struct clk *core_clk;
	struct clk *osr_clk;
	struct clk *bit_clk;
	atomic_t mi2s_rsc_ref;
};
static struct mi2s_clk quat_mi2s_clk;

#define GPIO_WCD_INTR2 68
#define HTC_HS_AMP  0x1
#define HTC_RCV_AMP 0x2
#define HTC_SPK_AMP 0x4
static int htc_hw_component_mask = 0;
static int htc_digital_mic_en = 0;
static int htc_24b_audio_en = 0;
static int htc_amp_mask = 0;
static int hs_amp_on = 0;
static int rcv_amp_on = 0;
static int spk_amp_on = 0;
static struct mutex htc_amp_mutex;

static atomic_t q6_effect_mode = ATOMIC_INIT(-1);
extern unsigned int system_rev;

static int msm8226_get_hw_component(void)
{
	return htc_hw_component_mask;
}

static int msm8226_enable_digital_mic(void)
{
	return htc_digital_mic_en;
}

int msm8226_enable_24b_audio(void)
{
	return htc_24b_audio_en;
}

void msm8226_set_q6_effect_mode(int mode)
{
	pr_info("%s: mode %d\n", __func__, mode);
	atomic_set(&q6_effect_mode, mode);
}

int msm8226_get_q6_effect_mode(void)
{
	int mode = atomic_read(&q6_effect_mode);
	pr_info("%s: mode %d\n", __func__, mode);
	return mode;
}

static struct acoustic_ops acoustic = {
	.enable_digital_mic = msm8226_enable_digital_mic,
	.get_hw_component = msm8226_get_hw_component,
	.set_q6_effect = msm8226_set_q6_effect_mode,
	.get_q6_effect = msm8226_get_q6_effect_mode,
	.enable_24b_audio = msm8226_enable_24b_audio
};

#define DRV_NAME "msm8226-asoc-tapan"

#define MSM_SLIM_0_RX_MAX_CHANNELS		2
#define MSM_SLIM_0_TX_MAX_CHANNELS		4

#define BTSCO_RATE_8KHZ 8000
#define BTSCO_RATE_16KHZ 16000

#define EXT_CLASS_D_EN_DELAY 13000
#define EXT_CLASS_D_DIS_DELAY 3000
#define EXT_CLASS_D_DELAY_DELTA 2000

#define WCD9XXX_MBHC_DEF_BUTTONS 8
#define WCD9XXX_MBHC_DEF_RLOADS 5
#define TAPAN_EXT_CLK_RATE 9600000

#define NUM_OF_AUXPCM_GPIOS 4

#define LO_1_SPK_AMP   0x1
#define LO_2_SPK_AMP   0x2

#define ADSP_STATE_READY_TIMEOUT_MS 3000

static void *adsp_state_notifier;

static int msm8226_auxpcm_rate = 8000;
static atomic_t auxpcm_rsc_ref;
static const char *const auxpcm_rate_text[] = {"rate_8000", "rate_16000"};
static const struct soc_enum msm8226_auxpcm_enum[] = {
		SOC_ENUM_SINGLE_EXT(2, auxpcm_rate_text),
};

#define LPAIF_OFFSET 0xFE000000
#define LPAIF_PRI_MODE_MUXSEL (LPAIF_OFFSET + 0x2B000)
#define LPAIF_SEC_MODE_MUXSEL (LPAIF_OFFSET + 0x2C000)
#define LPAIF_TER_MODE_MUXSEL (LPAIF_OFFSET + 0x2D000)
#define LPAIF_QUAD_MODE_MUXSEL (LPAIF_OFFSET + 0x2E000)

#define I2S_PCM_SEL 1
#define I2S_PCM_SEL_OFFSET 1

void *def_tapan_mbhc_cal(void);
static int msm_snd_enable_codec_ext_clk(struct snd_soc_codec *codec, int enable,
					bool dapm);

static struct wcd9xxx_mbhc_config mbhc_cfg = {
	.read_fw_bin = false,
	.calibration = NULL,
	.micbias = MBHC_MICBIAS2,
	.mclk_cb_fn = msm_snd_enable_codec_ext_clk,
	.mclk_rate = TAPAN_EXT_CLK_RATE,
	.gpio = 0,
	.gpio_irq = 0,
	.gpio_level_insert = 0,
	.detect_extn_cable = true,
	.micbias_enable_flags = 1 << MBHC_MICBIAS_ENABLE_THRESHOLD_HEADSET,
	.insert_detect = true,
	.swap_gnd_mic = NULL,
	.cs_enable_flags = (1 << MBHC_CS_ENABLE_POLLING |
			    1 << MBHC_CS_ENABLE_INSERTION |
			    1 << MBHC_CS_ENABLE_REMOVAL),
};

struct msm_auxpcm_gpio {
	unsigned gpio_no;
	const char *gpio_name;
};

struct msm_auxpcm_ctrl {
	struct msm_auxpcm_gpio *pin_data;
	u32 cnt;
};

struct msm8226_asoc_mach_data {
	int mclk_gpio;
	u32 mclk_freq;
	struct msm_auxpcm_ctrl *auxpcm_ctrl;
	u32 us_euro_gpio;
};

#define GPIO_NAME_INDEX 0
#define DT_PARSE_INDEX  1

static char *msm_auxpcm_gpio_name[][2] = {
	{"PRIM_AUXPCM_CLK",       "qcom,prim-auxpcm-gpio-clk"},
	{"PRIM_AUXPCM_SYNC",      "qcom,prim-auxpcm-gpio-sync"},
	{"PRIM_AUXPCM_DIN",       "qcom,prim-auxpcm-gpio-din"},
	{"PRIM_AUXPCM_DOUT",      "qcom,prim-auxpcm-gpio-dout"},
};

void *lpaif_pri_muxsel_virt_addr;

enum {
	SLIM_1_RX_1 = 145, 
	SLIM_1_TX_1 = 146, 
	SLIM_2_RX_1 = 147, 
	SLIM_3_RX_1 = 148, 
	SLIM_3_RX_2 = 149, 
	SLIM_4_TX_1 = 150, 
};

static int msm8226_ext_spk_pamp;
static int msm_slim_0_rx_ch = 1;
static int msm_slim_0_tx_ch = 1;

static int msm_btsco_rate = BTSCO_RATE_8KHZ;
static int msm_btsco_ch = 1;

static struct mutex cdc_mclk_mutex;
static struct clk *codec_clk;
static int clk_users;
#if !HTC_FEATURES
static int ext_spk_amp_gpio = -1;
#endif
static int vdd_spkr_gpio = -1;
static int top_SPK_muted = 0;
static int msm_proxy_rx_ch = 2;
static int slim0_rx_bit_format = SNDRV_PCM_FORMAT_S16_LE;

static inline int param_is_mask(int p)
{
	return ((p >= SNDRV_PCM_HW_PARAM_FIRST_MASK) &&
			(p <= SNDRV_PCM_HW_PARAM_LAST_MASK));
}

static inline struct snd_mask *param_to_mask(struct snd_pcm_hw_params *p, int n)
{
	return &(p->masks[n - SNDRV_PCM_HW_PARAM_FIRST_MASK]);
}

static void param_set_mask(struct snd_pcm_hw_params *p, int n, unsigned bit)
{
	if (bit >= SNDRV_MASK_MAX)
		return;
	if (param_is_mask(n)) {
		struct snd_mask *m = param_to_mask(p, n);
		m->bits[0] = 0;
		m->bits[1] = 0;
		m->bits[bit >> 5] |= (1 << (bit & 31));
	}
}

static int msm_snd_enable_codec_ext_clk(struct snd_soc_codec *codec, int enable,
					bool dapm)
{
	int ret = 0;
	pr_debug("%s: enable = %d clk_users = %d\n",
		__func__, enable, clk_users);

	mutex_lock(&cdc_mclk_mutex);
	if (enable) {
		if (!codec_clk) {
			dev_err(codec->dev, "%s: did not get Taiko MCLK\n",
					__func__);
			ret = -EINVAL;
			goto exit;
		}

		clk_users++;
		if (clk_users != 1)
			goto exit;
		if (codec_clk) {
			clk_prepare_enable(codec_clk);
			tapan_mclk_enable(codec, 1, dapm);
		} else {
			pr_err("%s: Error setting Tapan MCLK\n", __func__);
			clk_users--;
			ret = -EINVAL;
			goto exit;
		}
	} else {
		if (clk_users > 0) {
			clk_users--;
			if (clk_users == 0) {
				tapan_mclk_enable(codec, 0, dapm);
				clk_disable_unprepare(codec_clk);
			}
		} else {
			pr_err("%s: Error releasing Tapan MCLK\n", __func__);
			ret = -EINVAL;
			goto exit;
		}
	}
exit:
	mutex_unlock(&cdc_mclk_mutex);
	return ret;
}

static int msm8226_mclk_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	pr_debug("%s: event = %d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		return msm_snd_enable_codec_ext_clk(w->codec, 1, true);
	case SND_SOC_DAPM_POST_PMD:
		return msm_snd_enable_codec_ext_clk(w->codec, 0, true);
	}

	return 0;
}

#if !HTC_FEATURES
static void msm8226_ext_spk_power_amp_enable(u32 enable)
{
	if (enable) {
		gpio_direction_output(ext_spk_amp_gpio, enable);
		
		usleep_range(EXT_CLASS_D_EN_DELAY,
			EXT_CLASS_D_EN_DELAY + EXT_CLASS_D_DELAY_DELTA);
	} else {
		gpio_direction_output(ext_spk_amp_gpio, enable);
		
		usleep_range(EXT_CLASS_D_DIS_DELAY,
			EXT_CLASS_D_DIS_DELAY + EXT_CLASS_D_DELAY_DELTA);
	}

	pr_debug("%s: %s external speaker PAs.\n", __func__,
		enable ? "Enable" : "Disable");
}
#endif

static void htc_spk_amp_ctl(int enable)
{
	int i, value = (enable)?1:0;

	if(!htc_spk_config.init)
		return;

	for(i = 0; i < ARRAY_SIZE(htc_spk_config.gpio); i++) {

		pr_info("%s: gpio = %d, gpio name = %s value %d\n", __func__,
			htc_spk_config.gpio[i].gpio_no, htc_spk_config.gpio[i].gpio_name, value);
		if (top_SPK_muted && i == 0) {
			pr_info("top_SPK_muted = 1,do nothing for top speaker");
		} else {
			gpio_set_value(htc_spk_config.gpio[i].gpio_no, value);
		}

	}

}

static int mute_left_SPK_set(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	pr_info("%s: gpio = %d, spk_amp_on = %d value %ld\n", __func__,
		htc_spk_config.gpio[1].gpio_no, spk_amp_on,
		ucontrol->value.integer.value[0]);

	if (ucontrol->value.integer.value[0]) {
		pr_info("%s: value is %ld to mute left SPK\n", __func__ ,
			ucontrol->value.integer.value[0]);
		if (spk_amp_on == 1) {
			gpio_set_value(htc_spk_config.gpio[0].gpio_no, 0);
		}
		top_SPK_muted = 1;
	} else if (ucontrol->value.integer.value[0] == 0) {
		pr_info("%s: value is %ld to unmute left SPK\n", __func__ ,
			ucontrol->value.integer.value[0]);
		if (spk_amp_on == 1) {
			gpio_set_value(htc_spk_config.gpio[0].gpio_no, 1);
		} else {
			gpio_set_value(htc_spk_config.gpio[0].gpio_no, 0);
                }
		top_SPK_muted = 0;
	}

	return 0;

}

static void htc_rcv_amp_ctl(int enable)
{
	int i, value = (enable)?1:0;

	if(!htc_rcv_config.init)
		return;

	for(i = 0; i < ARRAY_SIZE(htc_rcv_config.gpio); i++) {

		pr_info("%s: gpio = %d, gpio name = %s value %d\n", __func__,
		htc_rcv_config.gpio[i].gpio_no, htc_rcv_config.gpio[i].gpio_name,value);

		gpio_set_value(htc_rcv_config.gpio[i].gpio_no, value);
	}

}

static void htc_amp_control(int amp_mask, int lineout_mask)
{
	if((lineout_mask & LO_1_SPK_AMP)) {
		if((lineout_mask & LO_2_SPK_AMP)) {
			if((amp_mask & HTC_HS_AMP) && hs_amp_on == 0) {
				pr_info("headphone amp on\n");
				hs_amp_on = 1;
				htc_acoustic_hs_amp_ctrl(1, 0);
			} else if (!(amp_mask & HTC_HS_AMP) && hs_amp_on == 1) {

				pr_info("headphone amp off\n");
				hs_amp_on = 0;
				htc_acoustic_hs_amp_ctrl(0, 0);
			}
		} else if(hs_amp_on == 1) {
			pr_info("headphone amp off\n");
			hs_amp_on = 0;
			htc_acoustic_hs_amp_ctrl(0, 0);
		}

		if((amp_mask & HTC_SPK_AMP) && spk_amp_on == 0) {
			pr_info("speaker amp on\n");
			htc_spk_amp_ctl(1);
			spk_amp_on = 1;
		} else if(!(amp_mask & HTC_SPK_AMP) && spk_amp_on == 1) {
			pr_info("speaker amp off\n");
			htc_spk_amp_ctl(0);
			spk_amp_on = 0;
		}

		if((amp_mask & HTC_RCV_AMP) && rcv_amp_on == 0) {
			pr_info("receiver amp on\n");
			htc_rcv_amp_ctl(1);
			rcv_amp_on = 1;
		} else if(!(amp_mask & HTC_RCV_AMP) && rcv_amp_on == 1) {
			pr_info("receiver amp off\n");
			htc_rcv_amp_ctl(0);
			rcv_amp_on = 0;
		}

	} else {
		if(hs_amp_on == 1) {
			pr_info("headphone amp off\n");
			hs_amp_on = 0;
			htc_acoustic_hs_amp_ctrl(0, 0);
		}

		if(spk_amp_on == 1) {
			pr_info("speaker amp off\n");
			htc_spk_amp_ctl(0);
			spk_amp_on = 0;
		}

		if(rcv_amp_on == 1) {
			pr_info("receiver amp off\n");
			htc_rcv_amp_ctl(0);
			rcv_amp_on = 0;
		}

	}

}

#if HTC_FEATURES
static void msm8226_ext_spk_power_amp_on(u32 spk)
{
	if (spk & (LO_1_SPK_AMP | LO_2_SPK_AMP)) {
		pr_debug("%s:Enable left and right speakers case spk = 0x%x\n",
			__func__, spk);

		msm8226_ext_spk_pamp |= spk;

		mutex_lock(&htc_amp_mutex);
		msm8226_ext_spk_pamp |= spk;
		htc_amp_control(htc_amp_mask,msm8226_ext_spk_pamp);
		mutex_unlock(&htc_amp_mutex);
	} else  {
		pr_err("%s: Invalid external speaker ampl. spk = 0x%x\n",
			__func__, spk);
	}
}

static void msm8226_ext_spk_power_amp_off(u32 spk)
{
	if (spk & (LO_1_SPK_AMP | LO_2_SPK_AMP)) {
		pr_debug("%s Disable left and right speakers case spk = 0x%08x",
			__func__, spk);

		if(!(msm8226_ext_spk_pamp & spk))
			return;

		mutex_lock(&htc_amp_mutex);
		msm8226_ext_spk_pamp &= ~spk;
		htc_amp_control(htc_amp_mask,msm8226_ext_spk_pamp);
		mutex_unlock(&htc_amp_mutex);
	} else  {
		pr_err("%s: ERROR : Invalid Ext Spk Ampl. spk = 0x%08x\n",
			__func__, spk);
		return;
	}
}

#else
static void msm8226_ext_spk_power_amp_on(u32 spk)
{
	if (gpio_is_valid(ext_spk_amp_gpio)) {
		if (spk & (LO_1_SPK_AMP | LO_2_SPK_AMP)) {
			pr_debug("%s:Enable left and right speakers case spk = 0x%x\n",
				__func__, spk);

			msm8226_ext_spk_pamp |= spk;

			if ((msm8226_ext_spk_pamp & LO_1_SPK_AMP) &&
				(msm8226_ext_spk_pamp & LO_2_SPK_AMP))
				if (ext_spk_amp_gpio >= 0) {
					pr_debug("%s  enable power", __func__);
					msm8226_ext_spk_power_amp_enable(1);
				}
		} else  {
			pr_err("%s: Invalid external speaker ampl. spk = 0x%x\n",
				__func__, spk);
		}
	}
}

static void msm8226_ext_spk_power_amp_off(u32 spk)
{
	if (gpio_is_valid(ext_spk_amp_gpio)) {
		if (spk & (LO_1_SPK_AMP | LO_2_SPK_AMP)) {
			pr_debug("%s Disable left and right speakers case spk = 0x%08x",
				__func__, spk);

			msm8226_ext_spk_pamp &= ~spk;

			if (!msm8226_ext_spk_pamp) {
				if (ext_spk_amp_gpio >= 0) {
					pr_debug("%s  disable power", __func__);
					msm8226_ext_spk_power_amp_enable(0);
				}
				msm8226_ext_spk_pamp = 0;
			}
		 } else  {
			pr_err("%s: ERROR : Invalid Ext Spk Ampl. spk = 0x%08x\n",
				__func__, spk);
		}
	}
}
#endif

static int msm8226_ext_spkramp_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	pr_debug("%s()\n", __func__);

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		if (!strncmp(w->name, "Lineout_1 amp", 14))
			msm8226_ext_spk_power_amp_on(LO_1_SPK_AMP);
		else if (!strncmp(w->name, "Lineout_2 amp", 14))
			msm8226_ext_spk_power_amp_on(LO_2_SPK_AMP);
		else {
			pr_err("%s() Invalid Speaker Widget = %s\n",
				__func__, w->name);
			return -EINVAL;
		}
	} else {
		if (!strncmp(w->name, "Lineout_1 amp", 14))
			msm8226_ext_spk_power_amp_off(LO_1_SPK_AMP);
		else if (!strncmp(w->name, "Lineout_2 amp", 14))
			msm8226_ext_spk_power_amp_off(LO_2_SPK_AMP);
		else {
			pr_err("%s() Invalid Speaker Widget = %s\n",
				__func__, w->name);
			return -EINVAL;
		}
	}

	return 0;
}

static int msm8226_vdd_spkr_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	pr_debug("%s: event = %d\n", __func__, event);

	return 0;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (vdd_spkr_gpio >= 0) {
			gpio_direction_output(vdd_spkr_gpio, 1);
			pr_debug("%s: Enabled 5V external supply for speaker\n",
					__func__);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (vdd_spkr_gpio >= 0) {
			gpio_direction_output(vdd_spkr_gpio, 0);
			pr_debug("%s: Disabled 5V external supply for speaker\n",
					__func__);
		}
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_widget msm8226_dapm_widgets[] = {

	SND_SOC_DAPM_SUPPLY("MCLK",  SND_SOC_NOPM, 0, 0,
	msm8226_mclk_event, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIC("Handset Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("ANCRight Headset Mic", NULL),
	SND_SOC_DAPM_MIC("ANCLeft Headset Mic", NULL),

	SND_SOC_DAPM_MIC("Digital Mic1", NULL),
	SND_SOC_DAPM_MIC("Digital Mic2", NULL),
	SND_SOC_DAPM_MIC("Digital Mic3", NULL),
	SND_SOC_DAPM_MIC("Digital Mic4", NULL),
	SND_SOC_DAPM_MIC("Digital Mic5", NULL),
	SND_SOC_DAPM_MIC("Digital Mic6", NULL),

	SND_SOC_DAPM_MIC("Analog Mic3", NULL),
	SND_SOC_DAPM_MIC("Analog Mic4", NULL),

	SND_SOC_DAPM_SPK("Lineout_1 amp", msm8226_ext_spkramp_event),
	SND_SOC_DAPM_SPK("Lineout_2 amp", msm8226_ext_spkramp_event),

	SND_SOC_DAPM_SUPPLY("EXT_VDD_SPKR",  SND_SOC_NOPM, 0, 0,
	msm8226_vdd_spkr_event, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

static const char *const slim0_rx_ch_text[] = {"One", "Two"};
static const char *const slim0_tx_ch_text[] = {"One", "Two", "Three", "Four"};
static const char *const proxy_rx_ch_text[] = {"One", "Two", "Three", "Four",
	"Five", "Six", "Seven", "Eight"};

static const struct soc_enum msm_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, slim0_rx_ch_text),
	SOC_ENUM_SINGLE_EXT(4, slim0_tx_ch_text),
};

static const char *const btsco_rate_text[] = {"8000", "16000"};
static const struct soc_enum msm_btsco_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, btsco_rate_text),
};

static int msm_slim_0_rx_ch_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: msm_slim_0_rx_ch  = %d\n", __func__,
		 msm_slim_0_rx_ch);
	ucontrol->value.integer.value[0] = msm_slim_0_rx_ch - 1;
	return 0;
}

static int msm_slim_0_rx_ch_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	msm_slim_0_rx_ch = ucontrol->value.integer.value[0] + 1;

	pr_debug("%s: msm_slim_0_rx_ch = %d\n", __func__,
		 msm_slim_0_rx_ch);
	return 1;
}

static int msm_slim_0_tx_ch_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: msm_slim_0_tx_ch  = %d\n", __func__,
		 msm_slim_0_tx_ch);
	ucontrol->value.integer.value[0] = msm_slim_0_tx_ch - 1;
	return 0;
}

static int msm_slim_0_tx_ch_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	msm_slim_0_tx_ch = ucontrol->value.integer.value[0] + 1;

	pr_debug("%s: msm_slim_0_tx_ch = %d\n", __func__, msm_slim_0_tx_ch);
	return 1;
}

static int msm_btsco_rate_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: msm_btsco_rate  = %d", __func__, msm_btsco_rate);
	ucontrol->value.integer.value[0] = msm_btsco_rate;
	return 0;
}

static int msm_btsco_rate_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 8000:
		msm_btsco_rate = BTSCO_RATE_8KHZ;
		break;
	case 16000:
		msm_btsco_rate = BTSCO_RATE_16KHZ;
		break;
	default:
		msm_btsco_rate = BTSCO_RATE_8KHZ;
		break;
	}

	pr_debug("%s: msm_btsco_rate = %d\n", __func__, msm_btsco_rate);
	return 0;
}

static int msm_btsco_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
					struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);

	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);

	rate->min = rate->max = msm_btsco_rate;
	channels->min = channels->max = msm_btsco_ch;

	return 0;
}

static int msm8226_auxpcm_rate_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = msm8226_auxpcm_rate;
	return 0;
}

static int msm8226_auxpcm_rate_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 0:
		msm8226_auxpcm_rate = 8000;
		break;
	case 1:
		msm8226_auxpcm_rate = 16000;
		break;
	default:
		msm8226_auxpcm_rate = 8000;
		break;
	}
	return 0;
}

static int msm_proxy_rx_ch_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: msm_proxy_rx_ch = %d\n", __func__,
						msm_proxy_rx_ch);
	ucontrol->value.integer.value[0] = msm_proxy_rx_ch - 1;
	return 0;
}

static int msm_proxy_rx_ch_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	msm_proxy_rx_ch = ucontrol->value.integer.value[0] + 1;
	pr_debug("%s: msm_proxy_rx_ch = %d\n", __func__,
						msm_proxy_rx_ch);
	return 1;
}

static int slim0_rx_bit_format_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	switch (slim0_rx_bit_format) {
	case SNDRV_PCM_FORMAT_S24_LE:
		ucontrol->value.integer.value[0] = 1;
		break;

	case SNDRV_PCM_FORMAT_S16_LE:
	default:
		ucontrol->value.integer.value[0] = 0;
		break;
	}

	pr_debug("%s: slim0_rx_bit_format = %d, ucontrol value = %ld\n",
			__func__, slim0_rx_bit_format,
			ucontrol->value.integer.value[0]);

	return 0;
}

static int slim0_rx_bit_format_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 1:
		slim0_rx_bit_format = SNDRV_PCM_FORMAT_S24_LE;
		break;
	case 0:
	default:
		slim0_rx_bit_format = SNDRV_PCM_FORMAT_S16_LE;
		break;
	}
	return 0;
}


static int msm_auxpcm_be_params_fixup(struct snd_soc_pcm_runtime *rtd,
					struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate =
	    hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);

	struct snd_interval *channels =
	    hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);

	rate->min = rate->max = msm8226_auxpcm_rate;
	channels->min = channels->max = 1;

	return 0;
}

static int msm_proxy_rx_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
					struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);

	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("%s: msm_proxy_rx_ch =%d\n", __func__, msm_proxy_rx_ch);

	if (channels->max < 2)
		channels->min = channels->max = 2;
	channels->min = channels->max = msm_proxy_rx_ch;
	rate->min = rate->max = 48000;
	return 0;
}

static int msm_proxy_tx_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
					struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);

	rate->min = rate->max = 48000;
	return 0;
}

static int msm_aux_pcm_get_gpios(struct msm_auxpcm_ctrl *auxpcm_ctrl)
{
	struct msm_auxpcm_gpio *pin_data = NULL;
	int ret = 0;
	int i;
	int j;

	pin_data = auxpcm_ctrl->pin_data;
	if (!pin_data) {
		pr_err("%s: Invalid control data for AUXPCM\n", __func__);
		ret = -EINVAL;
		goto err;
	}
	for (i = 0; i < auxpcm_ctrl->cnt; i++, pin_data++) {
		ret = gpio_request(pin_data->gpio_no,
				pin_data->gpio_name);
		pr_debug("%s: gpio = %d, gpio name = %s\n"
			"ret = %d\n", __func__,
			pin_data->gpio_no,
			pin_data->gpio_name,
			ret);
		if (ret) {
			pr_err("%s: Failed to request gpio %d\n",
				__func__, pin_data->gpio_no);
			
			if (i > 0) {
				for (j = i; j >= 0; j--)
					gpio_free(pin_data->gpio_no);
			}
			goto err;
		}
	}
err:
	return ret;
}

static int msm_aux_pcm_free_gpios(struct msm_auxpcm_ctrl *auxpcm_ctrl)
{
	struct msm_auxpcm_gpio *pin_data = NULL;
	int i;
	int ret = 0;

	if (auxpcm_ctrl == NULL || auxpcm_ctrl->pin_data == NULL) {
		pr_err("%s: Invalid control data for AUXPCM\n", __func__);
		ret = -EINVAL;
		goto err;
	}

	pin_data = auxpcm_ctrl->pin_data;
	for (i = 0; i < auxpcm_ctrl->cnt; i++, pin_data++) {
		gpio_free(pin_data->gpio_no);
		pr_debug("%s: gpio = %d, gpio_name = %s\n",
			__func__, pin_data->gpio_no,
			pin_data->gpio_name);
	}
err:
	return ret;
}

static int msm_auxpcm_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct msm8226_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	struct msm_auxpcm_ctrl *auxpcm_ctrl = NULL;
	int ret = 0;
	return 0;

	pr_debug("%s(): substream = %s, auxpcm_rsc_ref counter = %d\n",
		__func__, substream->name, atomic_read(&auxpcm_rsc_ref));

	auxpcm_ctrl = pdata->auxpcm_ctrl;

	if (auxpcm_ctrl == NULL || auxpcm_ctrl->pin_data == NULL ||
		lpaif_pri_muxsel_virt_addr == NULL) {
		pr_err("%s: Invalid control data for AUXPCM\n", __func__);
		ret = -EINVAL;
		goto err;
	}
	if (atomic_inc_return(&auxpcm_rsc_ref) == 1) {
		iowrite32(I2S_PCM_SEL << I2S_PCM_SEL_OFFSET,
				lpaif_pri_muxsel_virt_addr);
		ret = msm_aux_pcm_get_gpios(auxpcm_ctrl);
	}
	if (ret < 0) {
		pr_err("%s: Aux PCM GPIO request failed\n", __func__);
		return -EINVAL;
	}
err:
	return ret;
}

static void msm_auxpcm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct msm8226_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	struct msm_auxpcm_ctrl *auxpcm_ctrl = NULL;
	return;

	pr_debug("%s(): substream = %s, auxpcm_rsc_ref counter = %d\n",
		__func__, substream->name, atomic_read(&auxpcm_rsc_ref));

	auxpcm_ctrl = pdata->auxpcm_ctrl;

	if (atomic_dec_return(&auxpcm_rsc_ref) == 0)
		msm_aux_pcm_free_gpios(auxpcm_ctrl);
}

static struct snd_soc_ops msm_auxpcm_be_ops = {
	.startup = msm_auxpcm_startup,
	.shutdown = msm_auxpcm_shutdown,
};

static int msm_slim_0_rx_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
					    struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
		SNDRV_PCM_HW_PARAM_RATE);

	struct snd_interval *channels =
		hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("%s()\n", __func__);
	if(htc_acoustic_query_feature(HTC_AUD_24BIT)) {
		param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT,
			SNDRV_PCM_FORMAT_S24_LE);
	}
	rate->min = rate->max = 48000;
	channels->min = channels->max = msm_slim_0_rx_ch;

	return 0;
}

static int msm_slim_0_tx_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
					    struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
	SNDRV_PCM_HW_PARAM_RATE);

	struct snd_interval *channels = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("%s()\n", __func__);
	rate->min = rate->max = 48000;
	channels->min = channels->max = msm_slim_0_tx_ch;

	return 0;
}

static int msm_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);

	pr_debug("%s()\n", __func__);
	rate->min = rate->max = 48000;

	return 0;
}

static int msm_be_fm_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);

	struct snd_interval *channels = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("%s()\n", __func__);
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	return 0;
}

static int msm_be_mi2s_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);

	struct snd_interval *channels =
	    hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);

	if(htc_acoustic_query_feature(HTC_AUD_24BIT)) {
		param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT,
			SNDRV_PCM_FORMAT_S24_LE);
	}

	channels->min = channels->max = 2;
	pr_debug("%s()\n", __func__);
	rate->min = rate->max = 48000;

	return 0;
}

static const struct soc_enum msm_snd_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, slim0_rx_ch_text),
	SOC_ENUM_SINGLE_EXT(4, slim0_tx_ch_text),
	SOC_ENUM_SINGLE_EXT(8, proxy_rx_ch_text),
};


static const char *const mute_left_SPK_text[] = {"disable", "enable"};
static const struct soc_enum mute_left_SPK_enum[] = {
		SOC_ENUM_SINGLE_EXT(2, mute_left_SPK_text),
};


static const struct snd_kcontrol_new msm_snd_controls[] = {
	SOC_ENUM_EXT("SLIM_0_RX Channels", msm_snd_enum[0],
		     msm_slim_0_rx_ch_get, msm_slim_0_rx_ch_put),
	SOC_ENUM_EXT("SLIM_0_TX Channels", msm_snd_enum[1],
		     msm_slim_0_tx_ch_get, msm_slim_0_tx_ch_put),
	SOC_ENUM_EXT("AUX PCM SampleRate", msm8226_auxpcm_enum[0],
			msm8226_auxpcm_rate_get, msm8226_auxpcm_rate_put),
	SOC_ENUM_EXT("Internal BTSCO SampleRate", msm_btsco_enum[0],
		     msm_btsco_rate_get, msm_btsco_rate_put),
	SOC_ENUM_EXT("PROXY_RX Channels", msm_snd_enum[2],
			msm_proxy_rx_ch_get, msm_proxy_rx_ch_put),
	SOC_ENUM_EXT("SLIM_0_RX Format", msm_snd_enum[3],
			slim0_rx_bit_format_get, slim0_rx_bit_format_put),
	SOC_ENUM_EXT("Mute_Left_SPK", mute_left_SPK_enum[0],
			NULL, mute_left_SPK_set),

};

static int msm_afe_set_config(struct snd_soc_codec *codec)
{
	int rc;
	void *config_data;

	pr_debug("%s: enter\n", __func__);
	config_data = tapan_get_afe_config(codec, AFE_CDC_REGISTERS_CONFIG);
	rc = afe_set_config(AFE_CDC_REGISTERS_CONFIG, config_data, 0);
	if (rc) {
		pr_err("%s: Failed to set codec registers config %d\n",
			__func__, rc);
		return rc;
	}
	config_data = tapan_get_afe_config(codec, AFE_SLIMBUS_SLAVE_CONFIG);
	rc = afe_set_config(AFE_SLIMBUS_SLAVE_CONFIG, config_data, 0);
	if (rc) {
		pr_err("%s: Failed to set slimbus slave config %d\n", __func__,
			rc);
		return rc;
	}
	config_data = tapan_get_afe_config(codec, AFE_AANC_VERSION);
	rc = afe_set_config(AFE_AANC_VERSION, config_data, 0);
	if (rc) {
		pr_err("%s: Failed to set AANC version %d\n", __func__,
			rc);
		return rc;
	}
	return 0;
}

static void msm_afe_clear_config(void)
{
	afe_clear_config(AFE_CDC_REGISTERS_CONFIG);
	afe_clear_config(AFE_SLIMBUS_SLAVE_CONFIG);
	afe_clear_config(AFE_AANC_VERSION);
}

static int  msm8226_adsp_state_callback(struct notifier_block *nb,
		unsigned long value, void *priv)
{
	if (value == SUBSYS_BEFORE_SHUTDOWN) {
		pr_debug("%s: ADSP is about to shutdown. Clearing AFE config\n",
			 __func__);
		msm_afe_clear_config();
	} else if (value == SUBSYS_AFTER_POWERUP) {
		pr_debug("%s: ADSP is up\n", __func__);
	}

	return NOTIFY_OK;
}

static struct notifier_block adsp_state_notifier_block = {
	.notifier_call = msm8226_adsp_state_callback,
	.priority = -INT_MAX,
};

static int msm8226_tapan_codec_up(struct snd_soc_codec *codec)
{
	int err;
	unsigned long timeout;
	int adsp_ready = 0;

	pr_debug("%s\n", __func__);
	timeout = jiffies +
		msecs_to_jiffies(ADSP_STATE_READY_TIMEOUT_MS);

	do {
		if (!q6core_is_adsp_ready()) {
			pr_err("%s: ADSP Audio isn't ready\n", __func__);
		} else {
			pr_debug("%s: ADSP Audio is ready\n", __func__);
			adsp_ready = 1;
			break;
		}
	} while (time_after(timeout, jiffies));

	if (!adsp_ready) {
		pr_err("%s: timed out waiting for ADSP Audio\n", __func__);
		return -ETIMEDOUT;
	}

	err = msm_afe_set_config(codec);
	if (err)
		pr_err("%s: Failed to set AFE config. err %d\n",
			__func__, err);
	return err;
}

static int msm8226_tapan_event_cb(struct snd_soc_codec *codec,
	enum wcd9xxx_codec_event codec_event)
{
	switch (codec_event) {
	case WCD9XXX_CODEC_EVENT_CODEC_UP:
		return msm8226_tapan_codec_up(codec);
	default:
		pr_err("%s: UnSupported codec event %d\n",
			__func__, codec_event);
		return -EINVAL;
	}
}

static int msm_htc_amp_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int msm_spk_amp_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	mutex_lock(&htc_amp_mutex);

	if (ucontrol->value.integer.value[0])
		htc_amp_mask |= HTC_SPK_AMP;
	else
		htc_amp_mask &= ~HTC_SPK_AMP;

	htc_amp_control(htc_amp_mask,msm8226_ext_spk_pamp);
	mutex_unlock(&htc_amp_mutex);
	return 1;
}

static int msm_rcv_amp_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	mutex_lock(&htc_amp_mutex);

	if (ucontrol->value.integer.value[0])
		htc_amp_mask |= HTC_RCV_AMP;
	else
		htc_amp_mask &= ~HTC_RCV_AMP;

	htc_amp_control(htc_amp_mask,msm8226_ext_spk_pamp);
	mutex_unlock(&htc_amp_mutex);
	return 1;
}

static int msm_hs_amp_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	mutex_lock(&htc_amp_mutex);

	if (ucontrol->value.integer.value[0])
		htc_amp_mask |= HTC_HS_AMP;
	else
		htc_amp_mask &= ~HTC_HS_AMP;

	htc_amp_control(htc_amp_mask,msm8226_ext_spk_pamp);

	mutex_unlock(&htc_amp_mutex);
	return 1;
}

static int htc_mi2s_amp_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *k, int event)
{
	pr_info("%s() %s\n", __func__,w->name);

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		pr_info("%s:mi2s amp on\n",__func__);
		htc_acoustic_spk_amp_ctrl(SPK_AMP_RIGHT,1, 0);
		htc_acoustic_spk_amp_ctrl(SPK_AMP_LEFT,1, 0);
	} else {
		pr_info("%s:mi2s amp off\n",__func__);
		htc_acoustic_spk_amp_ctrl(SPK_AMP_RIGHT,0, 0);
		htc_acoustic_spk_amp_ctrl(SPK_AMP_LEFT,0, 0);
	}

	return 0;

}

static const struct snd_kcontrol_new htc_amp_siwth_control[] = {
	SOC_SINGLE_EXT("SPK AMP EN Switch", SND_SOC_NOPM,
	0, 1, 0, msm_htc_amp_get,msm_spk_amp_put),

	SOC_SINGLE_EXT("RCV AMP EN Switch", SND_SOC_NOPM,
	0, 1, 0, msm_htc_amp_get,msm_rcv_amp_put),

	SOC_SINGLE_EXT("HS AMP EN Switch", SND_SOC_NOPM,
	0, 1, 0, msm_htc_amp_get,msm_hs_amp_put),
};

static const struct snd_soc_dapm_widget htc_mi2s_widget[] = {

	SND_SOC_DAPM_AIF_IN_E("HTC VIRTUAL MI2S", "htc-virtual-mi2s-if", 0, SND_SOC_NOPM,
				0, 0, htc_mi2s_amp_event,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SPK("HTC_MI2S_OUT", NULL),
};

static const struct snd_soc_dapm_route htc_mi2s_virtual_route[] = {
	
	{"HTC_MI2S_OUT", NULL, "HTC VIRTUAL MI2S"},
};

static int msm_htc_mi2s_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	pr_info("%s: ++\n",__func__);
	snd_soc_dapm_new_controls(dapm, htc_mi2s_widget,
				  ARRAY_SIZE(htc_mi2s_widget));

	snd_soc_dapm_add_routes(dapm, htc_mi2s_virtual_route,
		ARRAY_SIZE(htc_mi2s_virtual_route));
	pr_info("%s: --\n",__func__);

	return 0;
}

static int msm_audrx_init(struct snd_soc_pcm_runtime *rtd)
{
	int err;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	unsigned int rx_ch[TAPAN_RX_MAX] = {144, 145, 146, 147, 148};
	unsigned int tx_ch[TAPAN_TX_MAX]  = {128, 129, 130, 131, 132};


	pr_info("%s(), dev_name%s\n", __func__, dev_name(cpu_dai->dev));

	rtd->pmdown_time = 0;

	err = snd_soc_add_codec_controls(codec, msm_snd_controls,
					 ARRAY_SIZE(msm_snd_controls));
	if (err < 0)
		return err;

	err = snd_soc_add_codec_controls(codec, htc_amp_siwth_control,
					 ARRAY_SIZE(htc_amp_siwth_control));
	if (err < 0)
		return err;

	snd_soc_dapm_new_controls(dapm, msm8226_dapm_widgets,
				ARRAY_SIZE(msm8226_dapm_widgets));

	snd_soc_dapm_enable_pin(dapm, "Lineout_1 amp");
	snd_soc_dapm_enable_pin(dapm, "Lineout_2 amp");

	snd_soc_dapm_sync(dapm);

	codec_clk = clk_get(cpu_dai->dev, "osr_clk");
	if (codec_clk < 0)
		pr_err("%s() Failed to get clock for %s\n",
			   __func__, dev_name(cpu_dai->dev));

	snd_soc_dai_set_channel_map(codec_dai, ARRAY_SIZE(tx_ch),
				    tx_ch, ARRAY_SIZE(rx_ch), rx_ch);

	err = msm_afe_set_config(codec);
	if (err) {
		pr_err("%s: Failed to set AFE config %d\n",
			__func__, err);
		return err;
	}

	
	mbhc_cfg.calibration = def_tapan_mbhc_cal();
	if (mbhc_cfg.calibration) {
		err = tapan_hs_detect(codec, &mbhc_cfg);
	} else {
		err = -ENOMEM;
		goto out;
	}

	adsp_state_notifier =
		subsys_notif_register_notifier("adsp",
			&adsp_state_notifier_block);

	if (!adsp_state_notifier) {
		pr_err("%s: Failed to register adsp state notifier\n",
			__func__);
		err = -EFAULT;
		goto out;
	}

	tapan_event_register(msm8226_tapan_event_cb, rtd->codec);
	return 0;

out:
	return err;
}

static int msm_snd_startup(struct snd_pcm_substream *substream)
{
	pr_debug("%s(): substream = %s  stream = %d\n", __func__,
		 substream->name, substream->stream);
	return 0;
}

void *def_tapan_mbhc_cal(void)
{
	void *tapan_cal;
	struct wcd9xxx_mbhc_btn_detect_cfg *btn_cfg;
	u16 *btn_low, *btn_high;
	u8 *n_ready, *n_cic, *gain;

	tapan_cal = kzalloc(WCD9XXX_MBHC_CAL_SIZE(WCD9XXX_MBHC_DEF_BUTTONS,
						WCD9XXX_MBHC_DEF_RLOADS),
			    GFP_KERNEL);
	if (!tapan_cal) {
		pr_err("%s: out of memory\n", __func__);
		return NULL;
	}

#define S(X, Y) ((WCD9XXX_MBHC_CAL_GENERAL_PTR(tapan_cal)->X) = (Y))
	S(t_ldoh, 100);
	S(t_bg_fast_settle, 100);
	S(t_shutdown_plug_rem, 255);
	S(mbhc_nsa, 2);
	S(mbhc_navg, 128);
#undef S
#define S(X, Y) ((WCD9XXX_MBHC_CAL_PLUG_DET_PTR(tapan_cal)->X) = (Y))
	S(mic_current, TAPAN_PID_MIC_5_UA);
	S(hph_current, TAPAN_PID_MIC_5_UA);
	S(t_mic_pid, 100);
	S(t_ins_complete, 250);
	S(t_ins_retry, 200);
#undef S
#define S(X, Y) ((WCD9XXX_MBHC_CAL_PLUG_TYPE_PTR(tapan_cal)->X) = (Y))
	S(v_no_mic, 30);
	S(v_hs_max, 2450);
#undef S
#define S(X, Y) ((WCD9XXX_MBHC_CAL_BTN_DET_PTR(tapan_cal)->X) = (Y))
	S(c[0], 62);
	S(c[1], 124);
	S(nc, 1);
	S(n_meas, 5);
	S(mbhc_nsc, 10);
	S(n_btn_meas, 1);
	S(n_btn_con, 2);
	S(num_btn, WCD9XXX_MBHC_DEF_BUTTONS);
	S(v_btn_press_delta_sta, 100);
	S(v_btn_press_delta_cic, 50);
#undef S
	btn_cfg = WCD9XXX_MBHC_CAL_BTN_DET_PTR(tapan_cal);
	btn_low = wcd9xxx_mbhc_cal_btn_det_mp(btn_cfg, MBHC_BTN_DET_V_BTN_LOW);
	btn_high = wcd9xxx_mbhc_cal_btn_det_mp(btn_cfg,
					       MBHC_BTN_DET_V_BTN_HIGH);
	btn_low[0] = -50;
	btn_high[0] = 20;
	btn_low[1] = 21;
	btn_high[1] = 61;
	btn_low[2] = 62;
	btn_high[2] = 104;
	btn_low[3] = 105;
	btn_high[3] = 148;
	btn_low[4] = 149;
	btn_high[4] = 189;
	btn_low[5] = 190;
	btn_high[5] = 228;
	btn_low[6] = 229;
	btn_high[6] = 269;
	btn_low[7] = 270;
	btn_high[7] = 500;
	n_ready = wcd9xxx_mbhc_cal_btn_det_mp(btn_cfg, MBHC_BTN_DET_N_READY);
	n_ready[0] = 80;
	n_ready[1] = 12;
	n_cic = wcd9xxx_mbhc_cal_btn_det_mp(btn_cfg, MBHC_BTN_DET_N_CIC);
	n_cic[0] = 60;
	n_cic[1] = 47;
	gain = wcd9xxx_mbhc_cal_btn_det_mp(btn_cfg, MBHC_BTN_DET_GAIN);
	gain[0] = 11;
	gain[1] = 14;

	return tapan_cal;
}

static int msm_snd_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret = 0;
	unsigned int rx_ch[SLIM_MAX_RX_PORTS], tx_ch[SLIM_MAX_TX_PORTS];
	unsigned int rx_ch_cnt = 0, tx_ch_cnt = 0;
	unsigned int user_set_tx_ch = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		pr_debug("%s: rx_0_ch=%d\n", __func__, msm_slim_0_rx_ch);
		ret = snd_soc_dai_get_channel_map(codec_dai,
					&tx_ch_cnt, tx_ch, &rx_ch_cnt , rx_ch);
		if (ret < 0) {
			pr_err("%s: failed to get codec chan map\n", __func__);
			goto end;
		}

		ret = snd_soc_dai_set_channel_map(cpu_dai, 0, 0,
						  msm_slim_0_rx_ch, rx_ch);
		if (ret < 0) {
			pr_err("%s: failed to set cpu chan map\n", __func__);
			goto end;
		}
	} else {

		pr_debug("%s: %s_tx_dai_id_%d_ch=%d\n", __func__,
			 codec_dai->name, codec_dai->id, user_set_tx_ch);

		ret = snd_soc_dai_get_channel_map(codec_dai,
					 &tx_ch_cnt, tx_ch, &rx_ch_cnt , rx_ch);
		if (ret < 0) {
			pr_err("%s: failed to get codec chan map\n", __func__);
			goto end;
		}
		
		if (codec_dai->id == 1)
			user_set_tx_ch = msm_slim_0_tx_ch;
		
		else if (codec_dai->id == 3)
			user_set_tx_ch = params_channels(params);
		else
			user_set_tx_ch = tx_ch_cnt;

		pr_debug("%s: msm_slim_0_tx_ch(%d)user_set_tx_ch(%d)tx_ch_cnt(%d)\n",
			 __func__, msm_slim_0_tx_ch, user_set_tx_ch, tx_ch_cnt);

		ret = snd_soc_dai_set_channel_map(cpu_dai,
						  user_set_tx_ch, tx_ch, 0 , 0);
		if (ret < 0) {
			pr_err("%s: failed to set cpu chan map\n", __func__);
			goto end;
		}
	}
end:
	return ret;
}

static void msm_snd_shutdown(struct snd_pcm_substream *substream)
{
	pr_debug("%s(): substream = %s stream = %d\n", __func__,
		 substream->name, substream->stream);
}

static struct snd_soc_ops msm8226_be_ops = {
	.startup = msm_snd_startup,
	.hw_params = msm_snd_hw_params,
	.shutdown = msm_snd_shutdown,
};

static int msm8226_quat_mi2s_free_gpios(void)
{
	int	i;
	for (i = 0; i < ARRAY_SIZE(htc_mi2s_config.gpio); i++)
		gpio_free(htc_mi2s_config.gpio[i].gpio_no);
	return 0;
}

static struct afe_clk_cfg lpass_mi2s_enable[2] = {
	{
		AFE_API_VERSION_I2S_CONFIG,
		Q6AFE_LPASS_IBIT_CLK_1_P536_MHZ,
		Q6AFE_LPASS_OSR_CLK_12_P288_MHZ,
		Q6AFE_LPASS_CLK_SRC_INTERNAL,
		Q6AFE_LPASS_CLK_ROOT_DEFAULT,
		Q6AFE_LPASS_MODE_BOTH_VALID,
		0,
	},
	{
		AFE_API_VERSION_I2S_CONFIG,
		Q6AFE_LPASS_IBIT_CLK_3_P072_MHZ,
		Q6AFE_LPASS_OSR_CLK_12_P288_MHZ,
		Q6AFE_LPASS_CLK_SRC_INTERNAL,
		Q6AFE_LPASS_CLK_ROOT_DEFAULT,
		Q6AFE_LPASS_MODE_BOTH_VALID,
		0,
	},
};
static struct afe_clk_cfg lpass_mi2s_disable = {
	AFE_API_VERSION_I2S_CONFIG,
	0,
	0,
	Q6AFE_LPASS_CLK_SRC_INTERNAL,
	Q6AFE_LPASS_CLK_ROOT_DEFAULT,
	Q6AFE_LPASS_MODE_BOTH_VALID,
	0,
};


static void msm8226_mi2s_shutdown(struct snd_pcm_substream *substream)
{
	int ret =0;

	if(!htc_mi2s_config.init) {
		pr_err("%s: mi2s config is not initial\n",__func__);
		return;
	}

	if (atomic_dec_return(&quat_mi2s_clk.mi2s_rsc_ref) == 0) {
		pr_info("%s: free mi2s resources\n", __func__);

		ret = afe_set_lpass_clock(htc_mi2s_config.afe_port_id, &lpass_mi2s_disable);
		if (ret < 0) {
			pr_err("%s: afe_set_lpass_clock failed\n", __func__);
		}

		msm8226_quat_mi2s_free_gpios();
	}
}

static int msm8226_configure_quat_mi2s_gpio(void)
{
	int	rtn;
	int	i;
	for (i = 0; i < ARRAY_SIZE(htc_mi2s_config.gpio); i++) {

		rtn = gpio_request(htc_mi2s_config.gpio[i].gpio_no,
				htc_mi2s_config.gpio[i].gpio_name);
		pr_info("%s: gpio = %d, gpio name = %s, rtn = %d\n", __func__,
		htc_mi2s_config.gpio[i].gpio_no, htc_mi2s_config.gpio[i].gpio_name, rtn);
		gpio_set_value(htc_mi2s_config.gpio[i].gpio_no, 1);

		if (rtn) {
			pr_err("%s: Failed to request gpio %d\n",
				   __func__,
				   htc_mi2s_config.gpio[i].gpio_no);
			while( i >= 0) {
				gpio_free(htc_mi2s_config.gpio[i].gpio_no);
				i--;
			}
			break;
		}
	}

	return rtn;
}
static int msm8226_mi2s_startup(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct afe_clk_cfg *lpass_mi2s_clk = NULL;

	pr_info("%s: dai name %s %p\n", __func__, cpu_dai->name, cpu_dai->dev);
	if(!htc_mi2s_config.init) {
		pr_err("%s: mi2s config is not initial\n",__func__);
		return -EINVAL;
	}

	if (atomic_inc_return(&quat_mi2s_clk.mi2s_rsc_ref) == 1) {
		pr_info("%s: acquire mi2s resources\n", __func__);
		msm8226_configure_quat_mi2s_gpio();


		if(htc_acoustic_query_feature(HTC_AUD_24BIT))
			lpass_mi2s_clk = &lpass_mi2s_enable[1];
		else
			lpass_mi2s_clk = &lpass_mi2s_enable[0];

		ret = afe_set_lpass_clock(htc_mi2s_config.afe_port_id, lpass_mi2s_clk);
		if (ret < 0) {
			int cnt = atomic_dec_return(&quat_mi2s_clk.mi2s_rsc_ref);
			pr_err("%s: afe_set_lpass_clock failed, cur_mi2s_ref %d\n", __func__,cnt);
		return ret;
		}

		ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_CBS_CFS);
		if (ret < 0)
			dev_err(cpu_dai->dev, "set format for CPU dai"
				" failed\n");

		ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_CBS_CFS);
		if (ret < 0)
			dev_err(codec_dai->dev, "set format for codec dai"
				 " failed\n");

		ret  = 0;

	}
	return ret;
}

static struct snd_soc_ops msm8226_mi2s_be_ops = {
	.startup = msm8226_mi2s_startup,
	.shutdown = msm8226_mi2s_shutdown
};

static struct snd_soc_dai_link msm8226_common_dai[] = {
	
	{
		.name = "MSM8226 Media1",
		.stream_name = "MultiMedia1",
		.cpu_dai_name	= "MultiMedia1",
		.platform_name  = "msm-pcm-dsp.0",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA1
	},
	{
		.name = "MSM8226 Media2",
		.stream_name = "MultiMedia2",
		.cpu_dai_name   = "MultiMedia2",
		.platform_name  = "msm-pcm-dsp.0",
		.dynamic = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA2,
	},
	{
		.name = "Circuit-Switch Voice",
		.stream_name = "CS-Voice",
		.cpu_dai_name   = "CS-VOICE",
		.platform_name  = "msm-pcm-voice",
		.dynamic = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_CS_VOICE,
	},
	{
		.name = "MSM VoIP",
		.stream_name = "VoIP",
		.cpu_dai_name	= "VoIP",
		.platform_name  = "msm-voip-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_VOIP,
	},
	{
		.name = "MSM8226 LPA",
		.stream_name = "LPA",
		.cpu_dai_name	= "MultiMedia3",
		.platform_name  = "msm-pcm-lpa",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA3,
	},
	
	{
		.name = "SLIMBUS_0 Hostless",
		.stream_name = "SLIMBUS_0 Hostless",
		.cpu_dai_name = "SLIMBUS0_HOSTLESS",
		.platform_name = "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, 
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	{
		.name = "INT_FM Hostless",
		.stream_name = "INT_FM Hostless",
		.cpu_dai_name	= "INT_FM_HOSTLESS",
		.platform_name  = "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	{
		.name = "MSM AFE-PCM RX",
		.stream_name = "AFE-PROXY RX",
		.cpu_dai_name = "msm-dai-q6-dev.241",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.platform_name  = "msm-pcm-afe",
		.ignore_suspend = 1,
		
		.ignore_pmdown_time = 1,
	},
	{
		.name = "MSM AFE-PCM TX",
		.stream_name = "AFE-PROXY TX",
		.cpu_dai_name = "msm-dai-q6-dev.240",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.platform_name  = "msm-pcm-afe",
		.ignore_suspend = 1,
	},
	{
		.name = "MSM8226 Compr",
		.stream_name = "COMPR",
		.cpu_dai_name	= "MultiMedia4",
		.platform_name  = "msm-compress-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			 SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		 
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA4,
	},
	{
		.name = "AUXPCM Hostless",
		.stream_name = "AUXPCM Hostless",
		.cpu_dai_name   = "AUXPCM_HOSTLESS",
		.platform_name  = "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	{
		.name = "SLIMBUS_1 Hostless",
		.stream_name = "SLIMBUS_1 Hostless",
		.cpu_dai_name = "SLIMBUS1_HOSTLESS",
		.platform_name = "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, 
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	{
		.name = "SLIMBUS_3 Hostless",
		.stream_name = "SLIMBUS_3 Hostless",
		.cpu_dai_name = "SLIMBUS3_HOSTLESS",
		.platform_name = "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, 
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	{
		.name = "SLIMBUS_4 Hostless",
		.stream_name = "SLIMBUS_4 Hostless",
		.cpu_dai_name = "SLIMBUS4_HOSTLESS",
		.platform_name = "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, 
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	{
		.name = "Voice2",
		.stream_name = "Voice2",
		.cpu_dai_name   = "Voice2",
		.platform_name  = "msm-pcm-voice",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	{
		.name = "MSM8226 LowLatency",
		.stream_name = "MultiMedia5",
		.cpu_dai_name   = "MultiMedia5",
		.platform_name  = "msm-pcm-dsp.1",
		.dynamic = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
				SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA5,
	},
	{
		.name = "MSM8226 Media9",
		.stream_name = "MultiMedia9",
		.cpu_dai_name   = "MultiMedia9",
		.platform_name  = "msm-pcm-dsp.0",
		.dynamic = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.ignore_suspend = 1,
		
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA9,
	},
	{
		.name = "VoLTE",
		.stream_name = "VoLTE",
		.cpu_dai_name   = "VoLTE",
		.platform_name  = "msm-pcm-voice",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.be_id = MSM_FRONTEND_DAI_VOLTE,
	},
	{
		.name = "QCHAT",
		.stream_name = "QCHAT",
		.cpu_dai_name   = "QCHAT",
		.platform_name  = "msm-pcm-voice",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.be_id = MSM_FRONTEND_DAI_QCHAT,
	},
	
	{
		.name = "Listen Audio Service",
		.stream_name = "Listen Audio Service",
		.cpu_dai_name = "LSM1",
		.platform_name = "msm-lsm-client",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			    SND_SOC_DPCM_TRIGGER_POST },
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.be_id = MSM_FRONTEND_DAI_LSM1,
	},
	
	{
		.name = "msm8226 Compr2",
		.stream_name = "COMPR2",
		.cpu_dai_name	= "MultiMedia6",
		.platform_name  = "msm-compress-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			 SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		 
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA6,
	},
	{
		.name = "msm8226 Compr3",
		.stream_name = "COMPR3",
		.cpu_dai_name	= "MultiMedia7",
		.platform_name  = "msm-compress-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			 SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		 
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA7,
	},
	{
		.name = "msm8226 Compr4",
		.stream_name = "COMPR4",
		.cpu_dai_name	= "MultiMedia8",
		.platform_name  = "msm-compress-dsp",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			 SND_SOC_DPCM_TRIGGER_POST},
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		 
		.be_id = MSM_FRONTEND_DAI_MULTIMEDIA8,
	},
	{
		.name = "Compress Stub",
		.stream_name = "Compress Stub",
		.cpu_dai_name	= "MM_STUB",
		.platform_name  = "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, 
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	
	{
		.name = LPASS_BE_INT_BT_SCO_RX,
		.stream_name = "Internal BT-SCO Playback",
		.cpu_dai_name = "msm-dai-q6-dev.12288",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name	= "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_INT_BT_SCO_RX,
		.be_hw_params_fixup = msm_btsco_be_hw_params_fixup,
		
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_INT_BT_SCO_TX,
		.stream_name = "Internal BT-SCO Capture",
		.cpu_dai_name = "msm-dai-q6-dev.12289",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name	= "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_INT_BT_SCO_TX,
		.be_hw_params_fixup = msm_btsco_be_hw_params_fixup,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_INT_FM_RX,
		.stream_name = "Internal FM Playback",
		.cpu_dai_name = "msm-dai-q6-dev.12292",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_INT_FM_RX,
		.be_hw_params_fixup = msm_be_fm_hw_params_fixup,
		
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_INT_FM_TX,
		.stream_name = "Internal FM Capture",
		.cpu_dai_name = "msm-dai-q6-dev.12293",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_INT_FM_TX,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ignore_suspend = 1,
	},
	
	{
		.name = LPASS_BE_AFE_PCM_RX,
		.stream_name = "AFE Playback",
		.cpu_dai_name = "msm-dai-q6-dev.224",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_AFE_PCM_RX,
		.be_hw_params_fixup = msm_proxy_rx_be_hw_params_fixup,
		
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_AFE_PCM_TX,
		.stream_name = "AFE Capture",
		.cpu_dai_name = "msm-dai-q6-dev.225",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_AFE_PCM_TX,
		.be_hw_params_fixup = msm_proxy_tx_be_hw_params_fixup,
		.ignore_suspend = 1,
	},
	
	{
		.name = "HDMI_RX_HOSTLESS",
		.stream_name = "HDMI_RX_HOSTLESS",
		.cpu_dai_name = "HDMI_HOSTLESS",
		.platform_name = "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	
	{
		.name = LPASS_BE_AUXPCM_RX,
		.stream_name = "AUX PCM Playback",
		.cpu_dai_name = "msm-dai-q6-auxpcm.1",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_AUXPCM_RX,
		.be_hw_params_fixup = msm_auxpcm_be_params_fixup,
		.ops = &msm_auxpcm_be_ops,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1
		
	},
	{
		.name = LPASS_BE_AUXPCM_TX,
		.stream_name = "AUX PCM Capture",
		.cpu_dai_name = "msm-dai-q6-auxpcm.1",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_AUXPCM_TX,
		.be_hw_params_fixup = msm_auxpcm_be_params_fixup,
		.ops = &msm_auxpcm_be_ops,
		.ignore_suspend = 1
	},
	
	{
		.name = LPASS_BE_INCALL_RECORD_TX,
		.stream_name = "Voice Uplink Capture",
		.cpu_dai_name = "msm-dai-q6-dev.32772",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_INCALL_RECORD_TX,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ignore_suspend = 1,
	},
	
	{
		.name = LPASS_BE_INCALL_RECORD_RX,
		.stream_name = "Voice Downlink Capture",
		.cpu_dai_name = "msm-dai-q6-dev.32771",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_INCALL_RECORD_RX,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ignore_suspend = 1,
	},
	
	{
		.name = LPASS_BE_VOICE_PLAYBACK_TX,
		.stream_name = "Voice Farend Playback",
		.cpu_dai_name = "msm-dai-q6-dev.32773",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_VOICE_PLAYBACK_TX,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ignore_suspend = 1,
	},
	
	{
		.name = LPASS_BE_VOICE2_PLAYBACK_TX,
		.stream_name = "Voice2 Farend Playback",
		.cpu_dai_name = "msm-dai-q6-dev.32770",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_VOICE2_PLAYBACK_TX,
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		.ignore_suspend = 1,
	},
        
        {
                .name = "VoWLAN",
                .stream_name = "VoWLAN",
                .cpu_dai_name   = "VoWLAN",
                .platform_name  = "msm-pcm-voice",
                .dynamic = 1,
                .trigger = {SND_SOC_DPCM_TRIGGER_POST,
                            SND_SOC_DPCM_TRIGGER_POST},
                .no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
                .ignore_suspend = 1,
                .ignore_pmdown_time = 1,
                .codec_dai_name = "snd-soc-dummy-dai",
                .codec_name = "snd-soc-dummy",
                .be_id = MSM_FRONTEND_DAI_VOWLAN,
        },
};

static struct snd_soc_dai_link msm8226_9306_dai[] = {
	
	{
		.name = LPASS_BE_SLIMBUS_0_RX,
		.stream_name = "Slimbus Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16384",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tapan_codec",
		.codec_dai_name	= "tapan_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_0_RX,
		.init = &msm_audrx_init,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		.ops = &msm8226_be_ops,
		.ignore_pmdown_time = 1, 
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_0_TX,
		.stream_name = "Slimbus Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16385",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tapan_codec",
		.codec_dai_name	= "tapan_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_0_TX,
		.be_hw_params_fixup = msm_slim_0_tx_be_hw_params_fixup,
		.ops = &msm8226_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_1_RX,
		.stream_name = "Slimbus1 Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16386",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tapan_codec",
		.codec_dai_name	= "tapan_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_1_RX,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		.ops = &msm8226_be_ops,
		
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_1_TX,
		.stream_name = "Slimbus1 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16387",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tapan_codec",
		.codec_dai_name	= "tapan_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_1_TX,
		.be_hw_params_fixup = msm_slim_0_tx_be_hw_params_fixup,
		.ops = &msm8226_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_3_RX,
		.stream_name = "Slimbus3 Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16390",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tapan_codec",
		.codec_dai_name	= "tapan_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_3_RX,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		.ops = &msm8226_be_ops,
		
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_3_TX,
		.stream_name = "Slimbus3 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16391",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tapan_codec",
		.codec_dai_name	= "tapan_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_3_TX,
		.be_hw_params_fixup = msm_slim_0_tx_be_hw_params_fixup,
		.ops = &msm8226_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_4_RX,
		.stream_name = "Slimbus4 Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16392",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tapan_codec",
		.codec_dai_name	= "tapan_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_4_RX,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		.ops = &msm8226_be_ops,
		
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_4_TX,
		.stream_name = "Slimbus4 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16393",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tapan_codec",
		.codec_dai_name	= "tapan_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_4_TX,
		.be_hw_params_fixup = msm_slim_0_tx_be_hw_params_fixup,
		.ops = &msm8226_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_5_TX,
		.stream_name = "Slimbus5 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16395",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tapan_codec",
		.codec_dai_name = "tapan_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_5_TX,
		.be_hw_params_fixup = msm_slim_0_tx_be_hw_params_fixup,
		.ops = &msm8226_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_TERT_MI2S_RX,
		.stream_name = "Tertiary MI2S Playback",
		.cpu_dai_name = "msm-dai-q6-mi2s.2",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name = "msm_htc_mi2s_codec",
		.no_pcm = 1,
		.init = &msm_htc_mi2s_init,
		.be_id = MSM_BACKEND_DAI_TERTIARY_MI2S_RX,
		.be_hw_params_fixup = msm_be_mi2s_hw_params_fixup,
		.ops = &msm8226_mi2s_be_ops,
	},
	{
		.name = LPASS_BE_TERT_MI2S_TX,
		.stream_name = "Tertiary MI2S Capture",
		.cpu_dai_name = "msm-dai-q6-mi2s.2",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_TERTIARY_MI2S_TX,
		.be_hw_params_fixup = msm_be_mi2s_hw_params_fixup,
		.ops = &msm8226_mi2s_be_ops,
	},
	{
		.name = "MI2S TX Hostless Capture",
		.stream_name = "MI2S TX Hostless Capture",
		.cpu_dai_name	= "MI2S_TX_HOSTLESS",
		.platform_name  = "msm-pcm-hostless",
		.dynamic = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1, 
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
};

static struct snd_soc_dai_link msm8226_9302_dai[] = {
	
	{
		.name = LPASS_BE_SLIMBUS_0_RX,
		.stream_name = "Slimbus Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16384",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tapan_codec",
		.codec_dai_name	= "tapan9302_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_0_RX,
		.init = &msm_audrx_init,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		.ops = &msm8226_be_ops,
		.ignore_pmdown_time = 1, 
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_0_TX,
		.stream_name = "Slimbus Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16385",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tapan_codec",
		.codec_dai_name	= "tapan9302_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_0_TX,
		.be_hw_params_fixup = msm_slim_0_tx_be_hw_params_fixup,
		.ops = &msm8226_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_1_RX,
		.stream_name = "Slimbus1 Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16386",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tapan_codec",
		.codec_dai_name	= "tapan9302_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_1_RX,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		.ops = &msm8226_be_ops,
		
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_1_TX,
		.stream_name = "Slimbus1 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16387",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tapan_codec",
		.codec_dai_name	= "tapan9302_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_1_TX,
		.be_hw_params_fixup = msm_slim_0_tx_be_hw_params_fixup,
		.ops = &msm8226_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_3_RX,
		.stream_name = "Slimbus3 Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16390",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tapan_codec",
		.codec_dai_name	= "tapan9302_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_3_RX,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		.ops = &msm8226_be_ops,
		
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_3_TX,
		.stream_name = "Slimbus3 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16391",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tapan_codec",
		.codec_dai_name	= "tapan9302_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_3_TX,
		.be_hw_params_fixup = msm_slim_0_tx_be_hw_params_fixup,
		.ops = &msm8226_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_4_RX,
		.stream_name = "Slimbus4 Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16392",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tapan_codec",
		.codec_dai_name	= "tapan9302_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_4_RX,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		.ops = &msm8226_be_ops,
		
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_4_TX,
		.stream_name = "Slimbus4 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16393",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tapan_codec",
		.codec_dai_name	= "tapan9302_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_4_TX,
		.be_hw_params_fixup = msm_slim_0_tx_be_hw_params_fixup,
		.ops = &msm8226_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_5_TX,
		.stream_name = "Slimbus5 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16395",
		.platform_name = "msm-pcm-routing",
		.codec_name = "tapan_codec",
		.codec_dai_name = "tapan9302_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_5_TX,
		.be_hw_params_fixup = msm_slim_0_tx_be_hw_params_fixup,
		.ops = &msm8226_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_TERT_MI2S_RX,
		.stream_name = "Tertiary MI2S Playback",
		.cpu_dai_name = "msm-dai-q6-mi2s.2",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name = "msm_htc_mi2s_codec",
		.no_pcm = 1,
		.init = &msm_htc_mi2s_init,
		.be_id = MSM_BACKEND_DAI_TERTIARY_MI2S_RX,
		.be_hw_params_fixup = msm_be_mi2s_hw_params_fixup,
		.ops = &msm8226_mi2s_be_ops,
	},
	{
		.name = LPASS_BE_TERT_MI2S_TX,
		.stream_name = "Tertiary MI2S Capture",
		.cpu_dai_name = "msm-dai-q6-mi2s.2",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_TERTIARY_MI2S_TX,
		.be_hw_params_fixup = msm_be_mi2s_hw_params_fixup,
		.ops = &msm8226_mi2s_be_ops,
	},
};

static struct snd_soc_dai_link msm8226_9306_dai_links[
				ARRAY_SIZE(msm8226_common_dai) +
				ARRAY_SIZE(msm8226_9306_dai)];

static struct snd_soc_dai_link msm8226_9302_dai_links[
				ARRAY_SIZE(msm8226_common_dai) +
				ARRAY_SIZE(msm8226_9302_dai)];

struct snd_soc_card snd_soc_card_msm8226 = {
	.name		= "msm8226-tapan-snd-card",
	.dai_link	= msm8226_9306_dai_links,
	.num_links	= ARRAY_SIZE(msm8226_9306_dai_links),
};

struct snd_soc_card snd_soc_card_9302_msm8226 = {
	.name		= "msm8226-tapan9302-snd-card",
	.dai_link	= msm8226_9302_dai_links,
	.num_links	= ARRAY_SIZE(msm8226_9302_dai_links),
};

static int msm8226_dtparse_auxpcm(struct platform_device *pdev,
				struct msm_auxpcm_ctrl **auxpcm_ctrl,
				char *msm_auxpcm_gpio_name[][2])
{
	int ret = 0;
	int i = 0;
	struct msm_auxpcm_gpio *pin_data = NULL;
	struct msm_auxpcm_ctrl *ctrl;
	unsigned int gpio_no[NUM_OF_AUXPCM_GPIOS];
	enum of_gpio_flags flags = OF_GPIO_ACTIVE_LOW;
	int auxpcm_cnt = 0;

	return 0;

	pin_data = devm_kzalloc(&pdev->dev, (ARRAY_SIZE(gpio_no) *
				sizeof(struct msm_auxpcm_gpio)),
				GFP_KERNEL);
	if (!pin_data) {
		dev_err(&pdev->dev, "No memory for gpio\n");
		ret = -ENOMEM;
		goto err;
	}

	for (i = 0; i < ARRAY_SIZE(gpio_no); i++) {
		gpio_no[i] = of_get_named_gpio_flags(pdev->dev.of_node,
				msm_auxpcm_gpio_name[i][DT_PARSE_INDEX],
				0, &flags);

		if (gpio_no[i] > 0) {
			pin_data[i].gpio_name =
			     msm_auxpcm_gpio_name[auxpcm_cnt][GPIO_NAME_INDEX];
			pin_data[i].gpio_no = gpio_no[i];
			dev_dbg(&pdev->dev, "%s:GPIO gpio[%s] =\n"
				"0x%x\n", __func__,
				pin_data[i].gpio_name,
				pin_data[i].gpio_no);
			auxpcm_cnt++;
		} else {
			dev_err(&pdev->dev, "%s:Invalid AUXPCM GPIO[%s]= %x\n",
				 __func__,
				msm_auxpcm_gpio_name[i][GPIO_NAME_INDEX],
				gpio_no[i]);
			ret = -ENODEV;
			goto err;
		}
	}

	ctrl = devm_kzalloc(&pdev->dev,
				sizeof(struct msm_auxpcm_ctrl), GFP_KERNEL);
	if (!ctrl) {
		dev_err(&pdev->dev, "No memory for gpio\n");
		ret = -ENOMEM;
		goto err;
	}

	ctrl->pin_data = pin_data;
	ctrl->cnt = auxpcm_cnt;
	*auxpcm_ctrl = ctrl;
	return ret;

err:
	if (pin_data)
		devm_kfree(&pdev->dev, pin_data);
	return ret;
}

static int msm8226_prepare_codec_mclk(struct snd_soc_card *card)
{
	struct msm8226_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	int ret;
	if (pdata->mclk_gpio) {
		ret = gpio_request(pdata->mclk_gpio, "TAPAN_CODEC_PMIC_MCLK");
		if (ret) {
			dev_err(card->dev,
				"%s: Failed to request tapan mclk gpio %d\n",
				__func__, pdata->mclk_gpio);
			return ret;
		}
	}
	return 0;
}

static bool msm8226_swap_gnd_mic(struct snd_soc_codec *codec)
{
	struct snd_soc_card *card = codec->card;
	struct msm8226_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	int value = gpio_get_value_cansleep(pdata->us_euro_gpio);

	pr_debug("%s: swap select switch %d to %d\n", __func__, value, !value);
	gpio_direction_output(pdata->us_euro_gpio, !value);

	return true;
}

static int msm8226_setup_hs_jack(struct platform_device *pdev,
		struct msm8226_asoc_mach_data *pdata)
{
	int rc;

	pdata->us_euro_gpio = of_get_named_gpio(pdev->dev.of_node,
				"qcom,cdc-us-euro-gpios", 0);
	if (pdata->us_euro_gpio < 0) {
		dev_dbg(&pdev->dev,
			"property %s in node %s not found %d\n",
			"qcom,cdc-us-euro-gpios", pdev->dev.of_node->full_name,
			pdata->us_euro_gpio);
	} else {
		rc = gpio_request(pdata->us_euro_gpio,
						  "TAPAN_CODEC_US_EURO_GPIO");
		if (rc) {
			dev_err(&pdev->dev,
				"%s: Failed to request tapan us-euro gpio %d\n",
				__func__, pdata->us_euro_gpio);
		} else {
			mbhc_cfg.swap_gnd_mic = msm8226_swap_gnd_mic;
		}
	}
	return 0;
}

static int msm8226_dtparse_mi2s(struct platform_device *pdev,
				struct mi2s_config *pconfig)
{
	int i, ret;

	if(!pconfig || !pdev)
		return 0;

	for(i = 0; i < ARRAY_SIZE(pconfig->gpio); i++) {

		if(!pconfig->gpio[i].gpio_name) {
			pr_err("%s: %d mi2s gpio name is null\n",__func__,i);
			return 0;
		}

		pconfig->gpio[i].gpio_no = of_get_named_gpio(pdev->dev.of_node, \
						pconfig->gpio[i].gpio_name, 0);

		if(pconfig->gpio[i].gpio_no < 0) {

			pr_err("%s: mi2s get gpio %s fail\n",__func__,pconfig->gpio[i].gpio_name);
			return 0;
		} else
			pr_info("%s: mi2s gpio %s no %d\n",__func__,pconfig->gpio[i].gpio_name,pconfig->gpio[i].gpio_no);
	}

	ret = of_property_read_u32(pdev->dev.of_node,
			"mi2s-afe-portid", &pconfig->afe_port_id);
	if (ret) {
		pr_err("%s: mi2s get afe port id fail\n",__func__);
		return 0;
	} else
		pr_info("%s: mi2s afe port id 0x%x\n",__func__,pconfig->afe_port_id);

	pconfig->init = 1;
	return 0;
}

static int msm8226_dtparse_spk(struct platform_device *pdev,
				struct spk_config *pconfig)
{
	int i, ret;

	if(!pconfig || !pdev)
		return 0;

	for(i = 0; i < ARRAY_SIZE(pconfig->gpio); i++) {

		if(!pconfig->gpio[i].gpio_name) {
			pr_err("%s: %d spk gpio name is null\n",__func__,i);
			return 0;
		}

		pconfig->gpio[i].gpio_no = of_get_named_gpio(pdev->dev.of_node, \
						pconfig->gpio[i].gpio_name, 0);

		if(pconfig->gpio[i].gpio_no < 0) {

			pr_err("%s: spk get gpio %s fail\n",__func__,pconfig->gpio[i].gpio_name);
			return 0;
		} else
			pr_info("%s: spk gpio %s no %d\n",__func__,pconfig->gpio[i].gpio_name,pconfig->gpio[i].gpio_no);

		ret = gpio_request(pconfig->gpio[i].gpio_no, pconfig->gpio[i].gpio_name);
		if (ret) {
			pr_err(	"%s: Failed to request gpio %d error %d\n",
				__func__, pconfig->gpio[i].gpio_no, ret);

			for(--i; i >= 0; i--)
				gpio_free(pconfig->gpio[i].gpio_no);

			return ret;
		}

		gpio_direction_output(pconfig->gpio[i].gpio_no, 0);
	}

	pconfig->init = 1;
	return 0;
}

static int msm8226_dtparse_rcv(struct platform_device *pdev,
				struct rcv_config *pconfig)
{
	int i, ret;

	if(!pconfig || !pdev)
		return 0;

	for(i = 0; i < ARRAY_SIZE(pconfig->gpio); i++) {

		if(!pconfig->gpio[i].gpio_name) {
			pr_err("%s: %d rcv gpio name is null\n",__func__,i);
			return 0;
		}

		pconfig->gpio[i].gpio_no = of_get_named_gpio(pdev->dev.of_node, \
						pconfig->gpio[i].gpio_name, 0);

		if(pconfig->gpio[i].gpio_no < 0) {

			pr_err("%s: rcv get gpio %s fail\n",__func__,pconfig->gpio[i].gpio_name);
			return 0;
		} else
			pr_info("%s: rcv gpio %s no %d\n",__func__,pconfig->gpio[i].gpio_name,pconfig->gpio[i].gpio_no);

		ret = gpio_request(pconfig->gpio[i].gpio_no, pconfig->gpio[i].gpio_name);
		if (ret) {
			pr_err(	"%s: Failed to request gpio %d error %d\n",
				__func__, pconfig->gpio[i].gpio_no, ret);

			for(--i; i >= 0; i--)
				gpio_free(pconfig->gpio[i].gpio_no);

			return ret;
		}

		gpio_direction_output(pconfig->gpio[i].gpio_no, 0);
	}

	pconfig->init = 1;
	return 0;
}

static struct snd_soc_card *populate_snd_card_dailinks(struct device *dev)
{

	struct snd_soc_card *card;

	if (of_property_read_bool(dev->of_node,
					"qcom,tapan-codec-9302")) {
		card = &snd_soc_card_9302_msm8226;

		memcpy(msm8226_9302_dai_links, msm8226_common_dai,
				sizeof(msm8226_common_dai));
		memcpy(msm8226_9302_dai_links + ARRAY_SIZE(msm8226_common_dai),
			msm8226_9302_dai, sizeof(msm8226_9302_dai));

	} else {

		card = &snd_soc_card_msm8226;

		memcpy(msm8226_9306_dai_links, msm8226_common_dai,
				sizeof(msm8226_common_dai));
		memcpy(msm8226_9306_dai_links + ARRAY_SIZE(msm8226_common_dai),
			msm8226_9306_dai, sizeof(msm8226_9306_dai));
	}

	return card;
}

static __devinit int msm8226_asoc_machine_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct msm8226_asoc_mach_data *pdata;
	int ret;
	
	int i = 0, j = 0;
	int num_hw = 0;

	pr_info("%s: ++\n",__func__);

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No platform supplied from device tree\n");
		return -EINVAL;
	}

	pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct msm8226_asoc_mach_data), GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev, "Can't allocate msm8226_asoc_mach_data\n");
		ret = -ENOMEM;
		goto err1;
	}

	card = populate_snd_card_dailinks(&pdev->dev);

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, pdata);

	ret = snd_soc_of_parse_card_name(card, "qcom,model");
	if (ret)
		goto err;

	ret = snd_soc_of_parse_audio_routing(card,
			"qcom,audio-routing");
	if (ret)
		goto err;

	ret = of_property_read_u32(pdev->dev.of_node,
			"qcom,tapan-mclk-clk-freq", &pdata->mclk_freq);
	if (ret) {
		dev_err(&pdev->dev, "Looking up %s property in node %s failed",
			"qcom,tapan-mclk-clk-freq",
			pdev->dev.of_node->full_name);
		goto err;
	}

	if (pdata->mclk_freq != 9600000) {
		dev_err(&pdev->dev, "unsupported tapan mclk freq %u\n",
			pdata->mclk_freq);
		ret = -EINVAL;
		goto err;
	}

	pdata->mclk_gpio = of_get_named_gpio(pdev->dev.of_node,
				"qcom,cdc-mclk-gpios", 0);
	pr_info("%s: pdata->mclk_gpio %d\n",__func__,pdata->mclk_gpio);
	if (pdata->mclk_gpio < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed %d\n",
			"qcom, cdc-mclk-gpios", pdev->dev.of_node->full_name,
			pdata->mclk_gpio);
		ret = -ENODEV;
		goto err;
	}

	ret = msm8226_prepare_codec_mclk(card);
	if (ret)
		goto err1;

	mutex_init(&cdc_mclk_mutex);

	mbhc_cfg.gpio_level_insert = of_property_read_bool(pdev->dev.of_node,
					"qcom,headset-jack-type-NC");

	ret = snd_soc_register_card(card);
	if (ret == -EPROBE_DEFER) {
		dev_info(&pdev->dev, "snd_soc_register_card dferal (%d)\n", ret);
		goto err;
	} else if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err;
	}

	mutex_init(&htc_amp_mutex);

	num_hw = of_property_count_strings(pdev->dev.of_node, "htc,aud-hw-component");
	for (i = 0; i < num_hw; i++) {
		const char *str = NULL;
		extern struct hw_component HTC_AUD_HW_LIST[AUD_HW_NUM];
		ret = of_property_read_string_index(pdev->dev.of_node, "htc,aud-hw-component",
			i, &str);
		if (ret) {
			dev_err(&pdev->dev,
				"htc,aud-hw-component index %d could not be read: %d\n", j, ret);
			continue;
		}
		for (j = 0; j < ARRAY_SIZE(HTC_AUD_HW_LIST); j++) {
			if (!strncmp(str, HTC_AUD_HW_LIST[j].name, sizeof(HTC_AUD_HW_LIST[j].name)) &&
                strlen(str) == strlen(HTC_AUD_HW_LIST[j].name)) {
				htc_hw_component_mask |= HTC_AUD_HW_LIST[j].id;
				dev_info(&pdev->dev, "Found HW: %s, htc_hw_component_mask 0x%X.\n",
					HTC_AUD_HW_LIST[j].name, htc_hw_component_mask);
			}
		}
	}

	ret = of_property_read_u32(pdev->dev.of_node,
			"htc,aud-digital-mic-en", &htc_digital_mic_en);
	if (ret) {
		dev_err(&pdev->dev, "htc,aud-digital-mic-en get fail\n");
	} else {
		dev_info(&pdev->dev, "htc,aud-digital-mic-en: %d.\n", htc_digital_mic_en);
	}

	ret = of_property_read_u32(pdev->dev.of_node,
			"htc,aud-24b-en", &htc_24b_audio_en);
	if (ret) {
		dev_err(&pdev->dev, "htc,aud-24b-en get fail\n");
	} else {
		dev_info(&pdev->dev, "htc,aud-24b-en: %d.\n", htc_24b_audio_en);
	}

	msm8226_dtparse_mi2s(pdev, &htc_mi2s_config);
	msm8226_dtparse_spk(pdev, &htc_spk_config);
	msm8226_dtparse_rcv(pdev, &htc_rcv_config);

	
	ret = msm8226_dtparse_auxpcm(pdev, &pdata->auxpcm_ctrl,
					msm_auxpcm_gpio_name);
	if (ret) {
		dev_err(&pdev->dev,
		"%s: Auxpcm pin data parse failed\n", __func__);
		goto err;
	}

#if !HTC_FEATURES
	vdd_spkr_gpio = of_get_named_gpio(pdev->dev.of_node,
				"qcom,cdc-vdd-spkr-gpios", 0);
	if (vdd_spkr_gpio < 0) {
		dev_dbg(&pdev->dev,
			"Looking up %s property in node %s failed %d\n",
			"qcom, cdc-vdd-spkr-gpios",
			pdev->dev.of_node->full_name, vdd_spkr_gpio);
	} else {
		ret = gpio_request(vdd_spkr_gpio, "TAPAN_CODEC_VDD_SPKR");
		if (ret) {
			
			dev_err(card->dev,
					"%s: Failed to request tapan vdd spkr gpio %d\n",
					__func__, vdd_spkr_gpio);
			goto err;
		}
	}

	ext_spk_amp_gpio = of_get_named_gpio(pdev->dev.of_node,
			"qcom,cdc-lineout-spkr-gpios", 0);
	if (ext_spk_amp_gpio < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed %d\n",
			"qcom, cdc-lineout-spkr-gpios",
			pdev->dev.of_node->full_name, ext_spk_amp_gpio);
	} else {
		ret = gpio_request(ext_spk_amp_gpio,
				"TAPAN_CODEC_LINEOUT_SPKR");
		if (ret) {
			
			dev_err(card->dev,
				"%s: Failed to request tapan amp spkr gpio %d\n",
				__func__, ext_spk_amp_gpio);
			goto err_vdd_spkr;
		}
	}
#endif

	msm8226_setup_hs_jack(pdev, pdata);

#if !HTC_FEATURES
	ret = of_property_read_string(pdev->dev.of_node,
			"qcom,prim-auxpcm-gpio-set", &auxpcm_pri_gpio_set);
	if (ret) {
		dev_err(&pdev->dev, "Looking up %s property in node %s failed",
			"qcom,prim-auxpcm-gpio-set",
			pdev->dev.of_node->full_name);
		goto err_lineout_spkr;
	}
	if (!strcmp(auxpcm_pri_gpio_set, "prim-gpio-prim")) {
		lpaif_pri_muxsel_virt_addr = ioremap(LPAIF_PRI_MODE_MUXSEL, 4);
	} else if (!strcmp(auxpcm_pri_gpio_set, "prim-gpio-tert")) {
		lpaif_pri_muxsel_virt_addr = ioremap(LPAIF_TER_MODE_MUXSEL, 4);
	} else {
		dev_err(&pdev->dev, "Invalid value %s for AUXPCM GPIO set\n",
			auxpcm_pri_gpio_set);
		ret = -EINVAL;
		goto err_lineout_spkr;
	}
	if (lpaif_pri_muxsel_virt_addr == NULL) {
		pr_err("%s Pri muxsel virt addr is null\n", __func__);
		ret = -EINVAL;
		goto err_lineout_spkr;
	}
#endif
	pr_info("%s: --\n",__func__);

	return 0;

#if !HTC_FEATURES
err_lineout_spkr:
	if (ext_spk_amp_gpio >= 0) {
		gpio_free(ext_spk_amp_gpio);
		ext_spk_amp_gpio = -1;
	}

err_vdd_spkr:
	if (vdd_spkr_gpio >= 0) {
		gpio_free(vdd_spkr_gpio);
		vdd_spkr_gpio = -1;
	}
#endif

err:
	if (pdata->mclk_gpio > 0) {
		dev_dbg(&pdev->dev, "%s free gpio %d\n",
			__func__, pdata->mclk_gpio);
		gpio_free(pdata->mclk_gpio);
		pdata->mclk_gpio = 0;
	}
err1:
	devm_kfree(&pdev->dev, pdata);
	return ret;
}

static int __devexit msm8226_asoc_machine_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct msm8226_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);

	gpio_free(pdata->mclk_gpio);
#if !HTC_FEATURES 
	if (vdd_spkr_gpio >= 0)
		gpio_free(vdd_spkr_gpio);
	if (ext_spk_amp_gpio >= 0)
		gpio_free(ext_spk_amp_gpio);
	if (pdata->us_euro_gpio > 0)
		gpio_free(pdata->us_euro_gpio);

	vdd_spkr_gpio = -1;
	ext_spk_amp_gpio = -1;
#endif
	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id msm8226_asoc_machine_of_match[]  = {
	{ .compatible = "qcom,msm8226-audio-tapan", },
	{},
};

static struct platform_driver msm8226_asoc_machine_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = msm8226_asoc_machine_of_match,
	},
	.probe = msm8226_asoc_machine_probe,
	.remove = __devexit_p(msm8226_asoc_machine_remove),
};

static int __init msm8226_audio_init(void)
{
	int ret = 0;

	htc_acoustic_register_ops(&acoustic);
	platform_driver_register(&msm8226_asoc_machine_driver);

	return ret;

}
late_initcall(msm8226_audio_init);

static void __exit msm8226_audio_exit(void)
{
	pr_info("%s", __func__);
	platform_driver_unregister(&msm8226_asoc_machine_driver);
}
module_exit(msm8226_audio_exit);

MODULE_DESCRIPTION("ALSA SoC msm");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, msm8226_asoc_machine_of_match);
