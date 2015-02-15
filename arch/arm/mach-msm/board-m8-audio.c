/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * HTC: elite machine driver which defines board-specific data
 * Copy from sound/soc/msm/msm8960.c
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


#include <mach/htc_acoustic_alsa.h>
#include <linux/module.h>


#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/qpnp/clkdiv.h>
#include <linux/regulator/consumer.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/jack.h>
#include <sound/q6afe-v2.h>
#include <asm/mach-types.h>
#include <mach/subsystem_notif.h>
#include <sound/q6core.h>
#include <mach/socinfo.h>
#include <sound/pcm_params.h>
#include "../../../sound/soc/msm/qdsp6v2/msm-pcm-routing-v2.h"
#include "../../../sound/soc/codecs/wcd9320.h"
#include <linux/io.h>
#include <mach/devices_cmdline.h>

#undef pr_info
#undef pr_err
#define pr_info(fmt, ...) pr_aud_info(fmt, ##__VA_ARGS__)
#define pr_err(fmt, ...) pr_aud_err(fmt, ##__VA_ARGS__)

#define DRV_NAME "msm8974-asoc-taiko"

#define MSM8974_SPK_ON 1
#define MSM8974_SPK_OFF 0

#define MSM_SLIM_0_RX_MAX_CHANNELS		2
#define MSM_SLIM_0_TX_MAX_CHANNELS		4

#define BTSCO_RATE_8KHZ 8000
#define BTSCO_RATE_16KHZ 16000

static int slim0_rx_bit_format = SNDRV_PCM_FORMAT_S16_LE;
static int hdmi_rx_bit_format = SNDRV_PCM_FORMAT_S16_LE;

#define SAMPLING_RATE_48KHZ 48000
#define SAMPLING_RATE_96KHZ 96000
#define SAMPLING_RATE_192KHZ 192000

static int msm8974_auxpcm_rate = 8000;
#define LO_1_SPK_AMP	0x1
#define LO_3_SPK_AMP	0x2
#define LO_2_SPK_AMP	0x4
#define LO_4_SPK_AMP	0x8

#define LPAIF_OFFSET 0xFE000000
#define LPAIF_PRI_MODE_MUXSEL (LPAIF_OFFSET + 0x2B000)
#define LPAIF_SEC_MODE_MUXSEL (LPAIF_OFFSET + 0x2C000)
#define LPAIF_TER_MODE_MUXSEL (LPAIF_OFFSET + 0x2D000)
#define LPAIF_QUAD_MODE_MUXSEL (LPAIF_OFFSET + 0x2E000)

#define I2S_PCM_SEL 1
#define I2S_PCM_SEL_OFFSET 1


#define WCD9XXX_MBHC_DEF_BUTTONS 8
#define WCD9XXX_MBHC_DEF_RLOADS 5
#define TAIKO_EXT_CLK_RATE 9600000

#define EXT_CLASS_D_EN_DELAY 13000
#define EXT_CLASS_D_DIS_DELAY 3000
#define EXT_CLASS_D_DELAY_DELTA 2000

#define NUM_OF_AUXPCM_GPIOS 4
static void *adsp_state_notifier;

#define ADSP_STATE_READY_TIMEOUT_MS 3000

static char *MID_LIST[][2] = {
	{"0P6B100", "MET#UL-EMEA"},
	{"0P6B110", "MET#UL-ASIA"},
	{"0P6B120", "MET#UL-ATT"},
	{"0P6B130", "MET#UL-TMUS"},
	{"0P6B150", "MET#UL-TW"},
	{"0P6B200", "MET#WL-VZW"},
	{"0P6B300", "MET#ULJ-SBM"},
	{"0P6B700", "MET#WHL-SPCS"},
};

#define HTC_HS_AMP  0x1
#define HTC_RCV_AMP 0x2
static int htc_amp_mask = 0;
static int hs_amp_on = 0;
static int rcv_amp_on = 0;
static int wcd_gpio_direct = 1;
static struct mutex htc_amp_mutex;

static atomic_t q6_effect_mode = ATOMIC_INIT(-1);
static atomic_t dev_use_dmic = ATOMIC_INIT(0);
extern unsigned int system_rev;

static int m8_get_hw_component(void)
{
	return (HTC_AUDIO_RT5501 | HTC_AUDIO_TFA9887 | HTC_AUDIO_TFA9887L);
}

char* m8_get_mid(void)
{
	char* mid = board_mid();
	int i;
	pr_info("mid=%s", mid);
	for (i = 0; i < ARRAY_SIZE(MID_LIST); i++)
	{
	    if(!strncmp(mid, MID_LIST[i][0],7)){
	        pr_info("match mid %s at %d", mid, i);
	        break;
	    }
	}
	if (i == ARRAY_SIZE(MID_LIST)){
		pr_info("cannot match SKU table, use Generic setting.");
		return "Generic";
	}
	else
		return MID_LIST[i][1];
}

static int m8_enable_digital_mic(void)
{
	return atomic_read(&dev_use_dmic);
}

int m8_enable_24b_audio(void)
{
	return 1;
}

void m8_set_q6_effect_mode(int mode)
{
	pr_info("%s: mode %d\n", __func__, mode);
	atomic_set(&q6_effect_mode, mode);
}

int m8_get_q6_effect_mode(void)
{
	int mode = atomic_read(&q6_effect_mode);
	pr_info("%s: mode %d\n", __func__, mode);
	return mode;
}

static struct acoustic_ops acoustic = {
	.get_mid = m8_get_mid,
	.enable_digital_mic = m8_enable_digital_mic,
	.get_hw_component = m8_get_hw_component,
	.set_q6_effect = m8_set_q6_effect_mode,
	.get_q6_effect = m8_get_q6_effect_mode,
	.enable_24b_audio = m8_enable_24b_audio
};


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

static const char *const auxpcm_rate_text[] = {"rate_8000", "rate_16000"};
static const struct soc_enum msm8974_auxpcm_enum[] = {
		SOC_ENUM_SINGLE_EXT(2, auxpcm_rate_text),
};

#define MI2S_LPASS_CLK_ENABLE

#ifdef MI2S_LPASS_CLK_ENABLE
#include <sound/q6afe-v2.h>
#endif

struct request_gpio {
	int gpio_no;
	char *gpio_name;
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

static struct wcd_intr_config {
	int init;
	struct request_gpio gpio[1];
} htc_wcd_config = {

	.init = 0,
	.gpio = {
			{ .gpio_name = "wcd-intr-gpio",},
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

static int msm_snd_enable_codec_ext_clk(struct snd_soc_codec *codec, int enable,
					bool dapm);
#if 0
static struct wcd9xxx_mbhc_config mbhc_cfg = {
	.read_fw_bin = false,
	.calibration = NULL,
	.micbias = MBHC_MICBIAS2,
	.mclk_cb_fn = msm_snd_enable_codec_ext_clk,
	.mclk_rate = TAIKO_EXT_CLK_RATE,
	.gpio = 0,
	.gpio_irq = 0,
	.gpio_level_insert = 1,
	.detect_extn_cable = true,
	.insert_detect = true,
	.swap_gnd_mic = NULL,
};
#endif
struct msm_auxpcm_gpio {
	unsigned gpio_no;
	const char *gpio_name;
};

struct msm_auxpcm_ctrl {
	struct msm_auxpcm_gpio *pin_data;
	u32 cnt;
};

struct msm8974_asoc_mach_data {
	int mclk_gpio;
	u32 mclk_freq;
	int us_euro_gpio;
	struct msm_auxpcm_ctrl *pri_auxpcm_ctrl;
	struct msm_auxpcm_ctrl *sec_auxpcm_ctrl;
};

#define GPIO_NAME_INDEX 0
#define DT_PARSE_INDEX  1

static char *msm_prim_auxpcm_gpio_name[][2] = {
	{"PRIM_AUXPCM_CLK",       "qcom,prim-auxpcm-gpio-clk"},
	{"PRIM_AUXPCM_SYNC",      "qcom,prim-auxpcm-gpio-sync"},
	{"PRIM_AUXPCM_DIN",       "qcom,prim-auxpcm-gpio-din"},
	{"PRIM_AUXPCM_DOUT",      "qcom,prim-auxpcm-gpio-dout"},
};

static char *msm_sec_auxpcm_gpio_name[][2] = {
	{"SEC_AUXPCM_CLK",       "qcom,sec-auxpcm-gpio-clk"},
	{"SEC_AUXPCM_SYNC",      "qcom,sec-auxpcm-gpio-sync"},
	{"SEC_AUXPCM_DIN",       "qcom,sec-auxpcm-gpio-din"},
	{"SEC_AUXPCM_DOUT",      "qcom,sec-auxpcm-gpio-dout"},
};

void *lpaif_pri_muxsel_virt_addr;
void *lpaif_sec_muxsel_virt_addr;

struct msm8974_liquid_dock_dev {
	int dock_plug_gpio;
	int dock_plug_irq;
	int dock_plug_det;
	struct snd_soc_dapm_context *dapm;
	struct work_struct irq_work;
};

static struct msm8974_liquid_dock_dev *msm8974_liquid_dock_dev;

enum {
	SLIM_1_RX_1 = 145, 
	SLIM_1_TX_1 = 146, 
	SLIM_2_RX_1 = 147, 
	SLIM_3_RX_1 = 148, 
	SLIM_3_RX_2 = 149, 
	SLIM_4_TX_1 = 150, 
};

static struct platform_device *spdev;
static struct regulator *ext_spk_amp_regulator;
static int ext_spk_amp_gpio = -1;
static int msm8974_spk_control = 1;
static int msm8974_ext_spk_pamp;
static int msm_slim_0_rx_ch = 1;
static int msm_slim_0_tx_ch = 1;

static int msm_btsco_rate = BTSCO_RATE_8KHZ;
static int msm_btsco_ch = 1;
static int msm_hdmi_rx_ch = 2;
static int slim0_rx_sample_rate = SAMPLING_RATE_48KHZ;
static int msm_proxy_rx_ch = 2;
static int hdmi_rx_sample_rate = SAMPLING_RATE_48KHZ;

static struct mutex cdc_mclk_mutex;
static struct clk *codec_clk;
static int clk_users;
static atomic_t prim_auxpcm_rsc_ref;
static atomic_t sec_auxpcm_rsc_ref;


static int msm8974_liquid_ext_spk_power_amp_init(void)
{
	int ret = 0;
	return 0; 
	ext_spk_amp_gpio = of_get_named_gpio(spdev->dev.of_node,
		"qcom,ext-spk-amp-gpio", 0);
	if (ext_spk_amp_gpio >= 0) {
		ret = gpio_request(ext_spk_amp_gpio, "ext_spk_amp_gpio");
		if (ret) {
			pr_err("%s: gpio_request failed for ext_spk_amp_gpio.\n",
				__func__);
			return -EINVAL;
		}
		gpio_direction_output(ext_spk_amp_gpio, 0);

		if (ext_spk_amp_regulator == NULL) {
			ext_spk_amp_regulator = regulator_get(&spdev->dev,
									"qcom,ext-spk-amp");

			if (IS_ERR(ext_spk_amp_regulator)) {
				pr_err("%s: Cannot get regulator %s.\n",
					__func__, "qcom,ext-spk-amp");

				gpio_free(ext_spk_amp_gpio);
				return PTR_ERR(ext_spk_amp_regulator);
			}
		}
	}

	return 0;
}

static void msm8974_liquid_ext_spk_power_amp_enable(u32 on)
{
	if (on) {
		regulator_enable(ext_spk_amp_regulator);
		gpio_direction_output(ext_spk_amp_gpio, on);
		
		usleep_range(EXT_CLASS_D_EN_DELAY,
			     EXT_CLASS_D_EN_DELAY + EXT_CLASS_D_DELAY_DELTA);
	} else {
		gpio_direction_output(ext_spk_amp_gpio, on);
		regulator_disable(ext_spk_amp_regulator);
		
		usleep_range(EXT_CLASS_D_DIS_DELAY,
			     EXT_CLASS_D_DIS_DELAY + EXT_CLASS_D_DELAY_DELTA);
	}

	pr_debug("%s: %s external speaker PAs.\n", __func__,
			on ? "Enable" : "Disable");
}

static void msm8974_liquid_docking_irq_work(struct work_struct *work)
{
	struct msm8974_liquid_dock_dev *dock_dev =
		container_of(work,
					 struct msm8974_liquid_dock_dev,
					 irq_work);

	struct snd_soc_dapm_context *dapm = dock_dev->dapm;


	mutex_lock(&dapm->codec->mutex);
	dock_dev->dock_plug_det =
		gpio_get_value(dock_dev->dock_plug_gpio);


	if (0 == dock_dev->dock_plug_det) {
		if ((msm8974_ext_spk_pamp & LO_1_SPK_AMP) &&
			(msm8974_ext_spk_pamp & LO_3_SPK_AMP) &&
			(msm8974_ext_spk_pamp & LO_2_SPK_AMP) &&
			(msm8974_ext_spk_pamp & LO_4_SPK_AMP))
			msm8974_liquid_ext_spk_power_amp_enable(1);
	} else {
		if ((msm8974_ext_spk_pamp & LO_1_SPK_AMP) &&
			(msm8974_ext_spk_pamp & LO_3_SPK_AMP) &&
			(msm8974_ext_spk_pamp & LO_2_SPK_AMP) &&
			(msm8974_ext_spk_pamp & LO_4_SPK_AMP))
			msm8974_liquid_ext_spk_power_amp_enable(0);
	}

	mutex_unlock(&dapm->codec->mutex);

}


static irqreturn_t msm8974_liquid_docking_irq_handler(int irq, void *dev)
{
	struct msm8974_liquid_dock_dev *dock_dev = dev;

	
	schedule_work(&dock_dev->irq_work);

	return IRQ_HANDLED;
}

static int msm8974_liquid_init_docking(struct snd_soc_dapm_context *dapm)
{
	int ret = 0;
	int dock_plug_gpio = 0;

	
	u32 dock_plug_irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

	dock_plug_gpio = of_get_named_gpio(spdev->dev.of_node,
					   "qcom,dock-plug-det-irq", 0);

	if (dock_plug_gpio >= 0) {

		msm8974_liquid_dock_dev =
		 kzalloc(sizeof(*msm8974_liquid_dock_dev), GFP_KERNEL);

		if (!msm8974_liquid_dock_dev) {
			pr_err("msm8974_liquid_dock_dev alloc fail.\n");
			return -ENOMEM;
		}

		msm8974_liquid_dock_dev->dock_plug_gpio = dock_plug_gpio;

		ret = gpio_request(msm8974_liquid_dock_dev->dock_plug_gpio,
					   "dock-plug-det-irq");
		if (ret) {
			pr_err("%s:failed request msm8974_liquid_dock_plug_gpio.\n",
				__func__);
			return -EINVAL;
		}

		msm8974_liquid_dock_dev->dock_plug_det =
			gpio_get_value(msm8974_liquid_dock_dev->dock_plug_gpio);

		msm8974_liquid_dock_dev->dock_plug_irq =
			gpio_to_irq(msm8974_liquid_dock_dev->dock_plug_gpio);

		msm8974_liquid_dock_dev->dapm = dapm;

		ret = request_irq(msm8974_liquid_dock_dev->dock_plug_irq,
				  msm8974_liquid_docking_irq_handler,
				  dock_plug_irq_flags,
				  "liquid_dock_plug_irq",
				  msm8974_liquid_dock_dev);

		INIT_WORK(
			&msm8974_liquid_dock_dev->irq_work,
			msm8974_liquid_docking_irq_work);
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
	if((lineout_mask & LO_1_SPK_AMP) && (lineout_mask & LO_3_SPK_AMP)) {
		if((lineout_mask & LO_2_SPK_AMP) && (lineout_mask & LO_4_SPK_AMP)) {
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

		if(rcv_amp_on == 1) {
			pr_info("receiver amp off\n");
			htc_rcv_amp_ctl(0);
			rcv_amp_on = 0;
		}

	}

}


static void msm8974_ext_spk_power_amp_on(u32 spk)
{
	if (spk & (LO_1_SPK_AMP |
		   LO_3_SPK_AMP |
		   LO_2_SPK_AMP |
		   LO_4_SPK_AMP)) {

		pr_debug("%s() External Left/Right Speakers already turned on. spk = 0x%08x\n",
						__func__, spk);

#if 0
		if ((msm8974_ext_spk_pamp & LO_1_SPK_AMP) &&
			(msm8974_ext_spk_pamp & LO_3_SPK_AMP) &&
			(msm8974_ext_spk_pamp & LO_2_SPK_AMP) &&
			(msm8974_ext_spk_pamp & LO_4_SPK_AMP)) {

			if (ext_spk_amp_gpio >= 0 &&
				msm8974_liquid_dock_dev != NULL &&
				msm8974_liquid_dock_dev->dock_plug_det == 0)
				msm8974_liquid_ext_spk_power_amp_enable(1);
		}
#endif
		mutex_lock(&htc_amp_mutex);
		msm8974_ext_spk_pamp |= spk;
		htc_amp_control(htc_amp_mask,msm8974_ext_spk_pamp);
		mutex_unlock(&htc_amp_mutex);

	} else  {

		pr_err("%s: ERROR : Invalid External Speaker Ampl. spk = 0x%08x\n",
			__func__, spk);
		return;
	}
}

static void msm8974_ext_spk_power_amp_off(u32 spk)
{
	if (spk & (LO_1_SPK_AMP |
		   LO_3_SPK_AMP |
		   LO_2_SPK_AMP |
		   LO_4_SPK_AMP)) {

		pr_debug("%s Left and right speakers case spk = 0x%08x",
				  __func__, spk);
#if 0
		if (!msm8974_ext_spk_pamp) {
			if (ext_spk_amp_gpio >= 0 &&
				msm8974_liquid_dock_dev != NULL &&
				msm8974_liquid_dock_dev->dock_plug_det == 0)
				msm8974_liquid_ext_spk_power_amp_enable(0);
			msm8974_ext_spk_pamp = 0;
		}
#endif
		if(!msm8974_ext_spk_pamp)
			return;

		mutex_lock(&htc_amp_mutex);
		msm8974_ext_spk_pamp &= ~spk;
		htc_amp_control(htc_amp_mask,msm8974_ext_spk_pamp);
		mutex_unlock(&htc_amp_mutex);
	} else  {

		pr_err("%s: ERROR : Invalid Ext Spk Ampl. spk = 0x%08x\n",
			__func__, spk);
		return;
	}
}

static void msm8974_ext_control(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	mutex_lock(&dapm->codec->mutex);

	pr_debug("%s: msm8974_spk_control = %d", __func__, msm8974_spk_control);
	if (msm8974_spk_control == MSM8974_SPK_ON) {
		snd_soc_dapm_enable_pin(dapm, "Lineout_1 amp");
		snd_soc_dapm_enable_pin(dapm, "Lineout_3 amp");
		snd_soc_dapm_enable_pin(dapm, "Lineout_2 amp");
		snd_soc_dapm_enable_pin(dapm, "Lineout_4 amp");
	} else {
		snd_soc_dapm_disable_pin(dapm, "Lineout_1 amp");
		snd_soc_dapm_disable_pin(dapm, "Lineout_3 amp");
		snd_soc_dapm_disable_pin(dapm, "Lineout_2 amp");
		snd_soc_dapm_disable_pin(dapm, "Lineout_4 amp");
	}

	snd_soc_dapm_sync(dapm);
	mutex_unlock(&dapm->codec->mutex);
}

static int msm8974_get_spk(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: msm8974_spk_control = %d", __func__, msm8974_spk_control);
	ucontrol->value.integer.value[0] = msm8974_spk_control;
	return 0;
}

static int msm8974_set_spk(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	pr_debug("%s()\n", __func__);
	if (msm8974_spk_control == ucontrol->value.integer.value[0])
		return 0;

	msm8974_spk_control = ucontrol->value.integer.value[0];
	msm8974_ext_control(codec);
	return 1;
}


static int msm_ext_spkramp_event(struct snd_soc_dapm_widget *w,
			     struct snd_kcontrol *k, int event)
{
	pr_info("%s() %s\n", __func__,w->name);

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		if (!strncmp(w->name, "Lineout_1 amp", 14))
			msm8974_ext_spk_power_amp_on(LO_1_SPK_AMP);
		else if (!strncmp(w->name, "Lineout_3 amp", 14))
			msm8974_ext_spk_power_amp_on(LO_3_SPK_AMP);
		else if (!strncmp(w->name, "Lineout_2 amp", 14))
			msm8974_ext_spk_power_amp_on(LO_2_SPK_AMP);
		else if  (!strncmp(w->name, "Lineout_4 amp", 14))
			msm8974_ext_spk_power_amp_on(LO_4_SPK_AMP);
		else {
			pr_err("%s() Invalid Speaker Widget = %s\n",
					__func__, w->name);
			return -EINVAL;
		}
	} else {
		if (!strncmp(w->name, "Lineout_1 amp", 14))
			msm8974_ext_spk_power_amp_off(LO_1_SPK_AMP);
		else if (!strncmp(w->name, "Lineout_3 amp", 14))
			msm8974_ext_spk_power_amp_off(LO_3_SPK_AMP);
		else if (!strncmp(w->name, "Lineout_2 amp", 14))
			msm8974_ext_spk_power_amp_off(LO_2_SPK_AMP);
		else if  (!strncmp(w->name, "Lineout_4 amp", 14))
			msm8974_ext_spk_power_amp_off(LO_4_SPK_AMP);
		else {
			pr_err("%s() Invalid Speaker Widget = %s\n",
					__func__, w->name);
			return -EINVAL;
		}
	}

	return 0;

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
			taiko_mclk_enable(codec, 1, dapm);
		} else {
			pr_err("%s: Error setting Taiko MCLK\n", __func__);
			clk_users--;
			goto exit;
		}
	} else {
		if (clk_users > 0) {
			clk_users--;
			if (clk_users == 0) {
				taiko_mclk_enable(codec, 0, dapm);
				clk_disable_unprepare(codec_clk);
			}
		} else {
			pr_err("%s: Error releasing Taiko MCLK\n", __func__);
			ret = -EINVAL;
			goto exit;
		}
	}
exit:
	mutex_unlock(&cdc_mclk_mutex);
	return ret;
}

static int msm8974_mclk_event(struct snd_soc_dapm_widget *w,
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

static const struct snd_soc_dapm_widget msm8974_dapm_widgets[] = {

	SND_SOC_DAPM_SUPPLY("MCLK",  SND_SOC_NOPM, 0, 0,
	msm8974_mclk_event, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SPK("Lineout_1 amp", msm_ext_spkramp_event),
	SND_SOC_DAPM_SPK("Lineout_3 amp", msm_ext_spkramp_event),

	SND_SOC_DAPM_SPK("Lineout_2 amp", msm_ext_spkramp_event),
	SND_SOC_DAPM_SPK("Lineout_4 amp", msm_ext_spkramp_event),

	SND_SOC_DAPM_MIC("Handset Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("ANCRight Headset Mic", NULL),
	SND_SOC_DAPM_MIC("ANCLeft Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Analog Mic4", NULL),
	SND_SOC_DAPM_MIC("Analog Mic6", NULL),
	SND_SOC_DAPM_MIC("Analog Mic7", NULL),

	SND_SOC_DAPM_MIC("Digital Mic1", NULL),
	SND_SOC_DAPM_MIC("Digital Mic2", NULL),
	SND_SOC_DAPM_MIC("Digital Mic3", NULL),
	SND_SOC_DAPM_MIC("Digital Mic4", NULL),
	SND_SOC_DAPM_MIC("Digital Mic5", NULL),
	SND_SOC_DAPM_MIC("Digital Mic6", NULL),
};

static const char *const spk_function[] = {"Off", "On"};
static const char *const slim0_rx_ch_text[] = {"One", "Two"};
static const char *const slim0_tx_ch_text[] = {"One", "Two", "Three", "Four",
						"Five"};
static char const *hdmi_rx_ch_text[] = {"Two", "Three", "Four", "Five",
					"Six", "Seven", "Eight"};
static char const *rx_bit_format_text[] = {"S16_LE", "S24_LE"};
static char const *slim0_rx_sample_rate_text[] = {"KHZ_48", "KHZ_96",
					"KHZ_192"};
static const char *const proxy_rx_ch_text[] = {"One", "Two", "Three", "Four",
	"Five",	"Six", "Seven", "Eight"};
static char const *hdmi_rx_sample_rate_text[] = {"KHZ_48", "KHZ_96",
					"KHZ_192"};
static const char *const btsco_rate_text[] = {"8000", "16000"};
static const struct soc_enum msm_btsco_enum[] = {
	SOC_ENUM_SINGLE_EXT(2, btsco_rate_text),
};

static int slim0_rx_sample_rate_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int sample_rate_val = 0;

	switch (slim0_rx_sample_rate) {
	case SAMPLING_RATE_192KHZ:
		sample_rate_val = 2;
		break;

	case SAMPLING_RATE_96KHZ:
		sample_rate_val = 1;
		break;

	case SAMPLING_RATE_48KHZ:
	default:
		sample_rate_val = 0;
		break;
	}

	ucontrol->value.integer.value[0] = sample_rate_val;
	pr_debug("%s: slim0_rx_sample_rate = %d\n", __func__,
				slim0_rx_sample_rate);

	return 0;
}

static int slim0_rx_sample_rate_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: ucontrol value = %ld\n", __func__,
			ucontrol->value.integer.value[0]);

	switch (ucontrol->value.integer.value[0]) {
	case 2:
		slim0_rx_sample_rate = SAMPLING_RATE_192KHZ;
		break;
	case 1:
		slim0_rx_sample_rate = SAMPLING_RATE_96KHZ;
		break;
	case 0:
	default:
		slim0_rx_sample_rate = SAMPLING_RATE_48KHZ;
	}

	pr_debug("%s: slim0_rx_sample_rate = %d\n", __func__,
			slim0_rx_sample_rate);

	return 0;
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
	case 0:
		msm_btsco_rate = BTSCO_RATE_8KHZ;
		break;
	case 1:
		msm_btsco_rate = BTSCO_RATE_16KHZ;
		break;
	default:
		msm_btsco_rate = BTSCO_RATE_8KHZ;
		break;
	}
	pr_debug("%s: msm_btsco_rate = %d\n", __func__, msm_btsco_rate);
	return 0;
}

static int hdmi_rx_bit_format_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	switch (hdmi_rx_bit_format) {
	case SNDRV_PCM_FORMAT_S24_LE:
		ucontrol->value.integer.value[0] = 1;
		break;

	case SNDRV_PCM_FORMAT_S16_LE:
	default:
		ucontrol->value.integer.value[0] = 0;
		break;
	}

	pr_debug("%s: hdmi_rx_bit_format = %d, ucontrol value = %ld\n",
			 __func__, hdmi_rx_bit_format,
			ucontrol->value.integer.value[0]);

	return 0;
}

static int hdmi_rx_bit_format_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 1:
		hdmi_rx_bit_format = SNDRV_PCM_FORMAT_S24_LE;
		break;
	case 0:
	default:
		hdmi_rx_bit_format = SNDRV_PCM_FORMAT_S16_LE;
		break;
	}
	pr_debug("%s: hdmi_rx_bit_format = %d, ucontrol value = %ld\n",
			 __func__, hdmi_rx_bit_format,
			ucontrol->value.integer.value[0]);
	return 0;
}

static int msm_hdmi_rx_ch_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: msm_hdmi_rx_ch  = %d\n", __func__,
			msm_hdmi_rx_ch);
	ucontrol->value.integer.value[0] = msm_hdmi_rx_ch - 2;

	return 0;
}

static int msm_hdmi_rx_ch_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	msm_hdmi_rx_ch = ucontrol->value.integer.value[0] + 2;
	if (msm_hdmi_rx_ch > 8) {
		pr_err("%s: channels exceeded 8.Limiting to max channels-8\n",
			__func__);
		msm_hdmi_rx_ch = 8;
	}
	pr_debug("%s: msm_hdmi_rx_ch = %d\n", __func__, msm_hdmi_rx_ch);

	return 1;
}

static int hdmi_rx_sample_rate_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int sample_rate_val = 0;

	switch (hdmi_rx_sample_rate) {
	case SAMPLING_RATE_192KHZ:
		sample_rate_val = 2;
		break;

	case SAMPLING_RATE_96KHZ:
		sample_rate_val = 1;
		break;

	case SAMPLING_RATE_48KHZ:
	default:
		sample_rate_val = 0;
		break;
	}

	ucontrol->value.integer.value[0] = sample_rate_val;
	pr_debug("%s: hdmi_rx_sample_rate = %d\n", __func__,
				hdmi_rx_sample_rate);

	return 0;
}

static int hdmi_rx_sample_rate_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	pr_debug("%s: ucontrol value = %ld\n", __func__,
			ucontrol->value.integer.value[0]);

	switch (ucontrol->value.integer.value[0]) {
	case 2:
		hdmi_rx_sample_rate = SAMPLING_RATE_192KHZ;
		break;
	case 1:
		hdmi_rx_sample_rate = SAMPLING_RATE_96KHZ;
		break;
	case 0:
	default:
		hdmi_rx_sample_rate = SAMPLING_RATE_48KHZ;
	}

	pr_debug("%s: hdmi_rx_sample_rate = %d\n", __func__,
			hdmi_rx_sample_rate);

	return 0;
}

static const struct snd_kcontrol_new int_btsco_rate_mixer_controls[] = {
	SOC_ENUM_EXT("Internal BTSCO SampleRate", msm_btsco_enum[0],
		     msm_btsco_rate_get, msm_btsco_rate_put),
};

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

static int msm8974_auxpcm_rate_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = msm8974_auxpcm_rate;
	return 0;
}

static int msm8974_auxpcm_rate_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	switch (ucontrol->value.integer.value[0]) {
	case 0:
		msm8974_auxpcm_rate = 8000;
		break;
	case 1:
		msm8974_auxpcm_rate = 16000;
		break;
	default:
		msm8974_auxpcm_rate = 8000;
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

static int msm_auxpcm_be_params_fixup(struct snd_soc_pcm_runtime *rtd,
					struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate =
	    hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);

	struct snd_interval *channels =
	    hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);

	rate->min = rate->max = msm8974_auxpcm_rate;
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

static int msm8974_hdmi_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
					struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);

	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("%s channels->min %u channels->max %u ()\n", __func__,
			channels->min, channels->max);

	if(htc_acoustic_query_feature(HTC_AUD_24BIT)) {
		hdmi_rx_bit_format = SNDRV_PCM_FORMAT_S24_LE;
	}

	param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT,
				hdmi_rx_bit_format);
	if (channels->max < 2)
		channels->min = channels->max = 2;
	rate->min = rate->max = hdmi_rx_sample_rate;
	channels->min = channels->max = msm_hdmi_rx_ch;

	return 0;
}

static int msm_aux_pcm_get_gpios(struct msm_auxpcm_ctrl *auxpcm_ctrl)
{
	struct msm_auxpcm_gpio *pin_data = NULL;
	int ret = 0;
	int i;
	int j;

	pin_data = auxpcm_ctrl->pin_data;
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
			
			for (j = i; j >= 0; j--)
				gpio_free(pin_data->gpio_no);
			return ret;
		}
	}
	return 0;
}

static int msm_aux_pcm_free_gpios(struct msm_auxpcm_ctrl *auxpcm_ctrl)
{
	struct msm_auxpcm_gpio *pin_data = NULL;
	int i;
	int ret = 0;

	if (auxpcm_ctrl == NULL || auxpcm_ctrl->pin_data == NULL) {
		pr_err("%s: Ctrl pointers are NULL\n", __func__);
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

static int msm_prim_auxpcm_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct msm8974_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	struct msm_auxpcm_ctrl *auxpcm_ctrl = NULL;
	int ret = 0;
	return 0;

	pr_debug("%s(): substream = %s, prim_auxpcm_rsc_ref counter = %d\n",
		__func__, substream->name, atomic_read(&prim_auxpcm_rsc_ref));

	auxpcm_ctrl = pdata->pri_auxpcm_ctrl;

	if (auxpcm_ctrl == NULL || auxpcm_ctrl->pin_data == NULL) {
		pr_err("%s: Ctrl pointers are NULL\n", __func__);
		ret = -EINVAL;
		goto err;
	}
	if (atomic_inc_return(&prim_auxpcm_rsc_ref) == 1) {
		if (lpaif_pri_muxsel_virt_addr != NULL)
			iowrite32(I2S_PCM_SEL << I2S_PCM_SEL_OFFSET,
				lpaif_pri_muxsel_virt_addr);
		else
			pr_err("%s lpaif_pri_muxsel_virt_addr is NULL\n",
				 __func__);
		ret = msm_aux_pcm_get_gpios(auxpcm_ctrl);
	}
	if (ret < 0) {
		pr_err("%s: Aux PCM GPIO request failed\n", __func__);
		return -EINVAL;
	}
err:
	return ret;
}

static void msm_prim_auxpcm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct msm8974_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	struct msm_auxpcm_ctrl *auxpcm_ctrl = NULL;
	return;

	pr_debug("%s(): substream = %s, prim_auxpcm_rsc_ref counter = %d\n",
		__func__, substream->name, atomic_read(&prim_auxpcm_rsc_ref));

	auxpcm_ctrl = pdata->pri_auxpcm_ctrl;

	if (atomic_dec_return(&prim_auxpcm_rsc_ref) == 0)
		msm_aux_pcm_free_gpios(auxpcm_ctrl);
}
static struct snd_soc_ops msm_pri_auxpcm_be_ops = {
	.startup = msm_prim_auxpcm_startup,
	.shutdown = msm_prim_auxpcm_shutdown,
};

static int msm_sec_auxpcm_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct msm8974_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	struct msm_auxpcm_ctrl *auxpcm_ctrl = NULL;
	int ret = 0;

	pr_debug("%s(): substream = %s, sec_auxpcm_rsc_ref counter = %d\n",
		__func__, substream->name, atomic_read(&sec_auxpcm_rsc_ref));
	return 0;
	auxpcm_ctrl = pdata->sec_auxpcm_ctrl;

	if (auxpcm_ctrl == NULL || auxpcm_ctrl->pin_data == NULL) {
		pr_err("%s: Ctrl pointers are NULL\n", __func__);
		ret = -EINVAL;
		goto err;
	}
	if (atomic_inc_return(&sec_auxpcm_rsc_ref) == 1) {
		if (lpaif_sec_muxsel_virt_addr != NULL)
			iowrite32(I2S_PCM_SEL << I2S_PCM_SEL_OFFSET,
				lpaif_sec_muxsel_virt_addr);
		else
			pr_err("%s lpaif_sec_muxsel_virt_addr is NULL\n",
				 __func__);
		ret = msm_aux_pcm_get_gpios(auxpcm_ctrl);
	}
	if (ret < 0) {
		pr_err("%s: Aux PCM GPIO request failed\n", __func__);
		return -EINVAL;
	}
err:
	return ret;
}

static void msm_sec_auxpcm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct msm8974_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	struct msm_auxpcm_ctrl *auxpcm_ctrl = NULL;

	return;
	pr_debug("%s(): substream = %s, sec_auxpcm_rsc_ref counter = %d\n",
		__func__, substream->name, atomic_read(&sec_auxpcm_rsc_ref));

	auxpcm_ctrl = pdata->sec_auxpcm_ctrl;

	if (atomic_dec_return(&sec_auxpcm_rsc_ref) == 0)
		msm_aux_pcm_free_gpios(auxpcm_ctrl);
}

static struct snd_soc_ops msm_sec_auxpcm_be_ops = {
	.startup = msm_sec_auxpcm_startup,
	.shutdown = msm_sec_auxpcm_shutdown,
};

static int msm_slim_0_rx_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
					    struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
	SNDRV_PCM_HW_PARAM_RATE);

	struct snd_interval *channels =
	    hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("%s()\n", __func__);
	param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT,
				   slim0_rx_bit_format);
	if(htc_acoustic_query_feature(HTC_AUD_24BIT)) {
		param_set_mask(params, SNDRV_PCM_HW_PARAM_FORMAT,
			SNDRV_PCM_FORMAT_S24_LE);
	}
	rate->min = rate->max = slim0_rx_sample_rate;
	channels->min = channels->max = msm_slim_0_rx_ch;

	 pr_debug("%s: format = %d, rate = %d, channels = %d\n",
			  __func__, params_format(params), params_rate(params),
			  msm_slim_0_rx_ch);

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

static int msm_slim_4_tx_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
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

static int msm_slim_5_tx_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
					    struct snd_pcm_hw_params *params)
{
	int rc;
	void *config;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_interval *rate =
	    hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels =
	    hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("%s enter\n", __func__);
	rate->min = rate->max = 16000;
	channels->min = channels->max = 1;

	config = taiko_get_afe_config(codec, AFE_SLIMBUS_SLAVE_PORT_CONFIG);
	rc = afe_set_config(AFE_SLIMBUS_SLAVE_PORT_CONFIG, config,
			    SLIMBUS_5_TX);
	if (rc) {
		pr_err("%s: Failed to set slimbus slave port config %d\n",
		       __func__, rc);
		return rc;
	}

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
	SOC_ENUM_SINGLE_EXT(2, spk_function),
	SOC_ENUM_SINGLE_EXT(2, slim0_rx_ch_text),
	SOC_ENUM_SINGLE_EXT(5, slim0_tx_ch_text),
	SOC_ENUM_SINGLE_EXT(7, hdmi_rx_ch_text),
	SOC_ENUM_SINGLE_EXT(2, rx_bit_format_text),
	SOC_ENUM_SINGLE_EXT(3, slim0_rx_sample_rate_text),
	SOC_ENUM_SINGLE_EXT(8, proxy_rx_ch_text),
	SOC_ENUM_SINGLE_EXT(3, hdmi_rx_sample_rate_text),
};

static const struct snd_kcontrol_new msm_snd_controls[] = {
	SOC_ENUM_EXT("Speaker Function", msm_snd_enum[0], msm8974_get_spk,
			msm8974_set_spk),
	SOC_ENUM_EXT("SLIM_0_RX Channels", msm_snd_enum[1],
			msm_slim_0_rx_ch_get, msm_slim_0_rx_ch_put),
	SOC_ENUM_EXT("SLIM_0_TX Channels", msm_snd_enum[2],
			msm_slim_0_tx_ch_get, msm_slim_0_tx_ch_put),
	SOC_ENUM_EXT("AUX PCM SampleRate", msm8974_auxpcm_enum[0],
			msm8974_auxpcm_rate_get, msm8974_auxpcm_rate_put),
	SOC_ENUM_EXT("HDMI_RX Channels", msm_snd_enum[3],
			msm_hdmi_rx_ch_get, msm_hdmi_rx_ch_put),
	SOC_ENUM_EXT("SLIM_0_RX Format", msm_snd_enum[4],
			slim0_rx_bit_format_get, slim0_rx_bit_format_put),
	SOC_ENUM_EXT("SLIM_0_RX SampleRate", msm_snd_enum[5],
			slim0_rx_sample_rate_get, slim0_rx_sample_rate_put),
	SOC_ENUM_EXT("HDMI_RX Bit Format", msm_snd_enum[4],
			hdmi_rx_bit_format_get, hdmi_rx_bit_format_put),
	SOC_ENUM_EXT("PROXY_RX Channels", msm_snd_enum[6],
			msm_proxy_rx_ch_get, msm_proxy_rx_ch_put),
	SOC_ENUM_EXT("HDMI_RX SampleRate", msm_snd_enum[7],
			hdmi_rx_sample_rate_get, hdmi_rx_sample_rate_put),
};


static int msm_afe_set_config(struct snd_soc_codec *codec)
{
	int rc;
	void *config_data;

	pr_debug("%s: enter\n", __func__);
	config_data = taiko_get_afe_config(codec, AFE_CDC_REGISTERS_CONFIG);
	rc = afe_set_config(AFE_CDC_REGISTERS_CONFIG, config_data, 0);
	if (rc) {
		pr_err("%s: Failed to set codec registers config %d\n",
		       __func__, rc);
		return rc;
	}

	config_data = taiko_get_afe_config(codec, AFE_SLIMBUS_SLAVE_CONFIG);
	rc = afe_set_config(AFE_SLIMBUS_SLAVE_CONFIG, config_data, 0);
	if (rc) {
		pr_err("%s: Failed to set slimbus slave config %d\n", __func__,
		       rc);
		return rc;
	}

	return 0;
}

static void msm_afe_clear_config(void)
{
	afe_clear_config(AFE_CDC_REGISTERS_CONFIG);
	afe_clear_config(AFE_SLIMBUS_SLAVE_CONFIG);
}

static int  msm8974_adsp_state_callback(struct notifier_block *nb,
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
	.notifier_call = msm8974_adsp_state_callback,
	.priority = -INT_MAX,
};

static int msm8974_taiko_codec_up(struct snd_soc_codec *codec)
{
	int err;
	unsigned long timeout;
	int adsp_ready = 0;

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

static int msm8974_taiko_event_cb(struct snd_soc_codec *codec,
		enum wcd9xxx_codec_event codec_event)
{
	switch (codec_event) {
	case WCD9XXX_CODEC_EVENT_CODEC_UP:
		return msm8974_taiko_codec_up(codec);
		break;
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

static int msm_rcv_amp_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	mutex_lock(&htc_amp_mutex);

	if (ucontrol->value.integer.value[0])
		htc_amp_mask |= HTC_RCV_AMP;
	else
		htc_amp_mask &= ~HTC_RCV_AMP;

	htc_amp_control(htc_amp_mask,msm8974_ext_spk_pamp);
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

	htc_amp_control(htc_amp_mask,msm8974_ext_spk_pamp);

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
	void *config_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	unsigned int rx_ch[TAIKO_RX_MAX] = {144, 145, 146, 147, 148, 149, 150,
					    151, 152, 153, 154, 155, 156};
	unsigned int tx_ch[TAIKO_TX_MAX]  = {128, 129, 130, 131, 132, 133,
					     134, 135, 136, 137, 138, 139,
					     140, 141, 142, 143};


	pr_info("%s(), dev_name%s\n", __func__, dev_name(cpu_dai->dev));

	rtd->pmdown_time = 0;

	err = snd_soc_add_codec_controls(codec, msm_snd_controls,
					 ARRAY_SIZE(msm_snd_controls));
	if (err < 0)
		return err;

	err = snd_soc_add_codec_controls(codec, int_btsco_rate_mixer_controls,
					 ARRAY_SIZE(int_btsco_rate_mixer_controls));
	if (err < 0)
		return err;

	err = snd_soc_add_codec_controls(codec, htc_amp_siwth_control,
					 ARRAY_SIZE(htc_amp_siwth_control));
	if (err < 0)
		return err;

	err = msm8974_liquid_ext_spk_power_amp_init();
	if (err) {
		pr_err("%s: LiQUID 8974 CLASS_D PAs init failed (%d)\n",
			__func__, err);
		return err;
	}

	err = msm8974_liquid_init_docking(dapm);
	if (err) {
		pr_err("%s: LiQUID 8974 init Docking stat IRQ failed (%d)\n",
			   __func__, err);
		return err;
	}

	snd_soc_dapm_new_controls(dapm, msm8974_dapm_widgets,
				ARRAY_SIZE(msm8974_dapm_widgets));

	snd_soc_dapm_enable_pin(dapm, "Lineout_1 amp");
	snd_soc_dapm_enable_pin(dapm, "Lineout_3 amp");
	snd_soc_dapm_enable_pin(dapm, "Lineout_2 amp");
	snd_soc_dapm_enable_pin(dapm, "Lineout_4 amp");


	snd_soc_dapm_sync(dapm);

	codec_clk = clk_get(cpu_dai->dev, "osr_clk");

	snd_soc_dai_set_channel_map(codec_dai, ARRAY_SIZE(tx_ch),
				    tx_ch, ARRAY_SIZE(rx_ch), rx_ch);


	err = msm_afe_set_config(codec);
	if (err) {
		pr_err("%s: Failed to set AFE config %d\n", __func__, err);
		goto out;
	}

	config_data = taiko_get_afe_config(codec, AFE_AANC_VERSION);
	err = afe_set_config(AFE_AANC_VERSION, config_data, 0);
	if (err) {
		pr_err("%s: Failed to set aanc version %d\n",
			__func__, err);
		goto out;
	}
	config_data = taiko_get_afe_config(codec,
				AFE_CDC_CLIP_REGISTERS_CONFIG);
	if (config_data) {
		err = afe_set_config(AFE_CDC_CLIP_REGISTERS_CONFIG,
					config_data, 0);
		if (err) {
			pr_err("%s: Failed to set clip registers %d\n",
				__func__, err);
			return err;
		}
	}
	config_data = taiko_get_afe_config(codec, AFE_CLIP_BANK_SEL);
	if (config_data) {
		err = afe_set_config(AFE_CLIP_BANK_SEL, config_data, 0);
		if (err) {
			pr_err("%s: Failed to set AFE bank selection %d\n",
				__func__, err);
			return err;
		}
	}

#if 0
	
	mbhc_cfg.calibration = def_taiko_mbhc_cal();
	if (mbhc_cfg.calibration) {
		err = taiko_hs_detect(codec, &mbhc_cfg);
		if (err)
			goto out;
	} else {
		err = -ENOMEM;
		goto out;
	}
#endif
	
	if(htc_wcd_config.init) {
		gpio_tlmm_config(GPIO_CFG(htc_wcd_config.gpio[0].gpio_no, \
			0,wcd_gpio_direct,GPIO_CFG_PULL_DOWN,GPIO_CFG_2MA),GPIO_CFG_ENABLE);
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

	taiko_event_register(msm8974_taiko_event_cb, rtd->codec);
	return 0;
out:
	clk_put(codec_clk);
	return err;
}

static int msm8974_snd_startup(struct snd_pcm_substream *substream)
{
	pr_debug("%s(): substream = %s  stream = %d\n", __func__,
		 substream->name, substream->stream);
	return 0;
}
#if 0
void *def_taiko_mbhc_cal(void)
{
	void *taiko_cal;
	struct wcd9xxx_mbhc_btn_detect_cfg *btn_cfg;
	u16 *btn_low, *btn_high;
	u8 *n_ready, *n_cic, *gain;

	taiko_cal = kzalloc(WCD9XXX_MBHC_CAL_SIZE(WCD9XXX_MBHC_DEF_BUTTONS,
						WCD9XXX_MBHC_DEF_RLOADS),
			    GFP_KERNEL);
	if (!taiko_cal) {
		pr_err("%s: out of memory\n", __func__);
		return NULL;
	}

#define S(X, Y) ((WCD9XXX_MBHC_CAL_GENERAL_PTR(taiko_cal)->X) = (Y))
	S(t_ldoh, 100);
	S(t_bg_fast_settle, 100);
	S(t_shutdown_plug_rem, 255);
	S(mbhc_nsa, 4);
	S(mbhc_navg, 4);
#undef S
#define S(X, Y) ((WCD9XXX_MBHC_CAL_PLUG_DET_PTR(taiko_cal)->X) = (Y))
	S(mic_current, TAIKO_PID_MIC_5_UA);
	S(hph_current, TAIKO_PID_MIC_5_UA);
	S(t_mic_pid, 100);
	S(t_ins_complete, 250);
	S(t_ins_retry, 200);
#undef S
#define S(X, Y) ((WCD9XXX_MBHC_CAL_PLUG_TYPE_PTR(taiko_cal)->X) = (Y))
	S(v_no_mic, 30);
	S(v_hs_max, 2400);
#undef S
#define S(X, Y) ((WCD9XXX_MBHC_CAL_BTN_DET_PTR(taiko_cal)->X) = (Y))
	S(c[0], 62);
	S(c[1], 124);
	S(nc, 1);
	S(n_meas, 3);
	S(mbhc_nsc, 11);
	S(n_btn_meas, 1);
	S(n_btn_con, 2);
	S(num_btn, WCD9XXX_MBHC_DEF_BUTTONS);
	S(v_btn_press_delta_sta, 100);
	S(v_btn_press_delta_cic, 50);
#undef S
	btn_cfg = WCD9XXX_MBHC_CAL_BTN_DET_PTR(taiko_cal);
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
	n_ready[1] = 68;
	n_cic = wcd9xxx_mbhc_cal_btn_det_mp(btn_cfg, MBHC_BTN_DET_N_CIC);
	n_cic[0] = 60;
	n_cic[1] = 47;
	gain = wcd9xxx_mbhc_cal_btn_det_mp(btn_cfg, MBHC_BTN_DET_GAIN);
	gain[0] = 11;
	gain[1] = 9;

	return taiko_cal;
}
#endif
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

static void msm8974_snd_shudown(struct snd_pcm_substream *substream)
{
	pr_debug("%s(): substream = %s stream = %d\n", __func__,
		 substream->name, substream->stream);

}

static struct snd_soc_ops msm8974_be_ops = {
	.startup = msm8974_snd_startup,
	.hw_params = msm_snd_hw_params,
	.shutdown = msm8974_snd_shudown,
};


static int msm8974_quat_mi2s_free_gpios(void)
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


static void msm8974_mi2s_shutdown(struct snd_pcm_substream *substream)
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

		msm8974_quat_mi2s_free_gpios();
	}
}

static int msm8974_configure_quat_mi2s_gpio(void)
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
static int msm8974_mi2s_startup(struct snd_pcm_substream *substream)
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
		msm8974_configure_quat_mi2s_gpio();

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

static struct snd_soc_ops msm8974_mi2s_be_ops = {
	.startup = msm8974_mi2s_startup,
	.shutdown = msm8974_mi2s_shutdown
};


static int msm8974_slimbus_2_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret = 0;
	unsigned int rx_ch[SLIM_MAX_RX_PORTS], tx_ch[SLIM_MAX_TX_PORTS];
	unsigned int rx_ch_cnt = 0, tx_ch_cnt = 0;
	unsigned int num_tx_ch = 0;
	unsigned int num_rx_ch = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {

		num_rx_ch =  params_channels(params);

		pr_debug("%s: %s rx_dai_id = %d  num_ch = %d\n", __func__,
			codec_dai->name, codec_dai->id, num_rx_ch);

		ret = snd_soc_dai_get_channel_map(codec_dai,
				&tx_ch_cnt, tx_ch, &rx_ch_cnt , rx_ch);
		if (ret < 0) {
			pr_err("%s: failed to get codec chan map\n", __func__);
			goto end;
		}

		ret = snd_soc_dai_set_channel_map(cpu_dai, 0, 0,
				num_rx_ch, rx_ch);
		if (ret < 0) {
			pr_err("%s: failed to set cpu chan map\n", __func__);
			goto end;
		}
	} else {

		num_tx_ch =  params_channels(params);

		pr_debug("%s: %s  tx_dai_id = %d  num_ch = %d\n", __func__,
			codec_dai->name, codec_dai->id, num_tx_ch);

		ret = snd_soc_dai_get_channel_map(codec_dai,
				&tx_ch_cnt, tx_ch, &rx_ch_cnt , rx_ch);
		if (ret < 0) {
			pr_err("%s: failed to get codec chan map\n", __func__);
			goto end;
		}

		ret = snd_soc_dai_set_channel_map(cpu_dai,
				num_tx_ch, tx_ch, 0 , 0);
		if (ret < 0) {
			pr_err("%s: failed to set cpu chan map\n", __func__);
			goto end;
		}
	}
end:
	return ret;
}


static struct snd_soc_ops msm8974_slimbus_2_be_ops = {
	.startup = msm8974_snd_startup,
	.hw_params = msm8974_slimbus_2_hw_params,
	.shutdown = msm8974_snd_shudown,
};

static struct snd_soc_dai_link msm8974_common_dai_links[] = {
	
	{
		.name = "MSM8974 Media1",
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
		.name = "MSM8974 Media2",
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
		.name = "MSM8974 LPA",
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
		.name = "MSM8974 Compr",
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
		.name = "Voice2",
		.stream_name = "Voice2",
		.cpu_dai_name   = "Voice2",
		.platform_name  = "msm-pcm-voice",
		.dynamic = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		
		.ignore_pmdown_time = 1,
		.be_id = MSM_FRONTEND_DAI_VOICE2,
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
		.name = "MSM8974 LowLatency",
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
		.name = "Listen Audio Service",
		.stream_name = "Listen Audio Service",
		.cpu_dai_name = "LSM1",
		.platform_name = "msm-lsm-client",
		.dynamic = 1,
		.trigger = { SND_SOC_DPCM_TRIGGER_POST,
			     SND_SOC_DPCM_TRIGGER_POST },
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.be_id = MSM_FRONTEND_DAI_LSM1,
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
		.be_hw_params_fixup = msm_be_hw_params_fixup,
		
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
		.ops = &msm_pri_auxpcm_be_ops,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		
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
		.ops = &msm_pri_auxpcm_be_ops,
		.ignore_suspend = 1,
	},
	
	{
		.name = LPASS_BE_SLIMBUS_0_RX,
		.stream_name = "Slimbus Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16384",
		.platform_name = "msm-pcm-routing",
		.codec_name = "taiko_codec",
		.codec_dai_name	= "taiko_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_0_RX,
		.init = &msm_audrx_init,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		.ops = &msm8974_be_ops,
		.ignore_pmdown_time = 1, 
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_0_TX,
		.stream_name = "Slimbus Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16385",
		.platform_name = "msm-pcm-routing",
		.codec_name = "taiko_codec",
		.codec_dai_name	= "taiko_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_0_TX,
		.be_hw_params_fixup = msm_slim_0_tx_be_hw_params_fixup,
		.ops = &msm8974_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_1_RX,
		.stream_name = "Slimbus1 Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16386",
		.platform_name = "msm-pcm-routing",
		.codec_name = "taiko_codec",
		.codec_dai_name	= "taiko_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_1_RX,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		.ops = &msm8974_be_ops,
		
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_1_TX,
		.stream_name = "Slimbus1 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16387",
		.platform_name = "msm-pcm-routing",
		.codec_name = "taiko_codec",
		.codec_dai_name	= "taiko_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_1_TX,
		.be_hw_params_fixup = msm_slim_0_tx_be_hw_params_fixup,
		.ops = &msm8974_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_3_RX,
		.stream_name = "Slimbus3 Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16390",
		.platform_name = "msm-pcm-routing",
		.codec_name = "taiko_codec",
		.codec_dai_name	= "taiko_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_3_RX,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		.ops = &msm8974_be_ops,
		
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_3_TX,
		.stream_name = "Slimbus3 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16391",
		.platform_name = "msm-pcm-routing",
		.codec_name = "taiko_codec",
		.codec_dai_name	= "taiko_tx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_3_TX,
		.be_hw_params_fixup = msm_slim_0_tx_be_hw_params_fixup,
		.ops = &msm8974_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_4_RX,
		.stream_name = "Slimbus4 Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16392",
		.platform_name = "msm-pcm-routing",
		.codec_name = "taiko_codec",
		.codec_dai_name	= "taiko_rx1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_4_RX,
		.be_hw_params_fixup = msm_slim_0_rx_be_hw_params_fixup,
		.ops = &msm8974_be_ops,
		
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		.name = LPASS_BE_SLIMBUS_4_TX,
		.stream_name = "Slimbus4 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16393",
		.platform_name = "msm-pcm-hostless",
		.codec_name = "taiko_codec",
		.codec_dai_name	= "taiko_vifeedback",
		.be_id = MSM_BACKEND_DAI_SLIMBUS_4_TX,
		.be_hw_params_fixup = msm_slim_4_tx_be_hw_params_fixup,
		.ops = &msm8974_be_ops,
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ignore_suspend = 1,
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
		.name = LPASS_BE_SLIMBUS_5_TX,
		.stream_name = "Slimbus5 Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16395",
		.platform_name = "msm-pcm-routing",
		.codec_name = "taiko_codec",
		.codec_dai_name = "taiko_mad1",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SLIMBUS_5_TX,
		.be_hw_params_fixup = msm_slim_5_tx_be_hw_params_fixup,
		.ops = &msm8974_be_ops,
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
		.name = "SLIMBUS_2 Hostless Playback",
		.stream_name = "SLIMBUS_2 Hostless Playback",
		.cpu_dai_name = "msm-dai-q6-dev.16388",
		.platform_name = "msm-pcm-hostless",
		.codec_name = "taiko_codec",
		.codec_dai_name = "taiko_rx2",
		.ignore_suspend = 1,
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ops = &msm8974_slimbus_2_be_ops,
	},
	
	{
		.name = "SLIMBUS_2 Hostless Capture",
		.stream_name = "SLIMBUS_2 Hostless Capture",
		.cpu_dai_name = "msm-dai-q6-dev.16389",
		.platform_name = "msm-pcm-hostless",
		.codec_name = "taiko_codec",
		.codec_dai_name = "taiko_tx2",
		.ignore_suspend = 1,
		.no_host_mode = SND_SOC_DAI_LINK_NO_HOST,
		.ops = &msm8974_slimbus_2_be_ops,
	},
	{
		.name = LPASS_BE_QUAT_MI2S_RX,
		.stream_name = "Quaternary MI2S Playback",
		.cpu_dai_name = "msm-dai-q6-mi2s.3",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name = "msm_htc_mi2s_codec",
		.no_pcm = 1,
		.init = &msm_htc_mi2s_init,
		.be_id = MSM_BACKEND_DAI_QUATERNARY_MI2S_RX,
		.be_hw_params_fixup = msm_be_mi2s_hw_params_fixup,
		.ops = &msm8974_mi2s_be_ops,
	},
	{
		.name = LPASS_BE_QUAT_MI2S_TX,
		.stream_name = "Quaternary MI2S Capture",
		.cpu_dai_name = "msm-dai-q6-mi2s.3",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_QUATERNARY_MI2S_TX,
		.be_hw_params_fixup = msm_be_mi2s_hw_params_fixup,
		.ops = &msm8974_mi2s_be_ops,
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
		.name = "MSM8974 Compr2",
		.stream_name = "COMPR2",
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
		.name = LPASS_BE_SEC_AUXPCM_RX,
		.stream_name = "Sec AUX PCM Playback",
		.cpu_dai_name = "msm-dai-q6-auxpcm.2",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-rx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SEC_AUXPCM_RX,
		.be_hw_params_fixup = msm_auxpcm_be_params_fixup,
		.ops = &msm_sec_auxpcm_be_ops,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		
	},
	{
		.name = LPASS_BE_SEC_AUXPCM_TX,
		.stream_name = "Sec AUX PCM Capture",
		.cpu_dai_name = "msm-dai-q6-auxpcm.2",
		.platform_name = "msm-pcm-routing",
		.codec_name = "msm-stub-codec.1",
		.codec_dai_name = "msm-stub-tx",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_SEC_AUXPCM_TX,
		.be_hw_params_fixup = msm_auxpcm_be_params_fixup,
		.ops = &msm_sec_auxpcm_be_ops,
		.ignore_suspend = 1,
	},
	{
		.name = "MSM8974 Compr3",
		.stream_name = "COMPR3",
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
		.name = "MSM8974 Compr4",
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

static struct snd_soc_dai_link msm8974_hdmi_dai_link[] = {
	{
		.name = LPASS_BE_HDMI,
		.stream_name = "HDMI Playback",
		.cpu_dai_name = "msm-dai-q6-hdmi.8",
		.platform_name = "msm-pcm-routing",
		.codec_name     = "msm-hdmi-audio-codec-rx",
		.codec_dai_name = "msm_hdmi_audio_codec_rx_dai",
		.no_pcm = 1,
		.be_id = MSM_BACKEND_DAI_HDMI_RX,
		.be_hw_params_fixup = msm8974_hdmi_be_hw_params_fixup,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
};

static struct snd_soc_dai_link msm8974_dai_links[
					 ARRAY_SIZE(msm8974_common_dai_links) +
					 ARRAY_SIZE(msm8974_hdmi_dai_link)];

struct snd_soc_card snd_soc_card_msm8974 = {
	.name		= "msm8974-taiko-snd-card",
};

static int msm8974_dtparse_auxpcm(struct platform_device *pdev,
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

static int msm8974_prepare_codec_mclk(struct snd_soc_card *card)
{
	struct msm8974_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	int ret;
	if (pdata->mclk_gpio) {
		ret = gpio_request(pdata->mclk_gpio, "TAIKO_CODEC_PMIC_MCLK");
		if (ret) {
			dev_err(card->dev,
				"%s: Failed to request taiko mclk gpio %d ret %d\n",
				__func__, pdata->mclk_gpio,ret);
			return ret;
		}
	}

	return 0;
}

static int msm8974_prepare_us_euro(struct snd_soc_card *card)
{
	struct msm8974_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);
	int ret;
	return 0; 
	if (pdata->us_euro_gpio) {
		dev_dbg(card->dev, "%s : us_euro gpio request %d", __func__,
			pdata->us_euro_gpio);
		ret = gpio_request(pdata->us_euro_gpio, "TAIKO_CODEC_US_EURO");
		if (ret) {
			dev_err(card->dev,
				"%s: Failed to request taiko US/EURO gpio %d error %d\n",
				__func__, pdata->us_euro_gpio, ret);
			return ret;
		}
	}

	return 0;
}

static int msm8974_dtparse_mi2s(struct platform_device *pdev,
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

		if(!gpio_is_valid(pconfig->gpio[i].gpio_no)) {

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

static int msm8974_dtparse_rcv(struct platform_device *pdev,
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

		if(!gpio_is_valid(pconfig->gpio[i].gpio_no)) {

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

static int msm8974_dtparse_wcd(struct platform_device *pdev,
				struct wcd_intr_config *pconfig)
{
	int i,ret;

	if(!pconfig || !pdev)
		return 0;

	for(i = 0; i < ARRAY_SIZE(pconfig->gpio); i++) {

		if(!pconfig->gpio[i].gpio_name) {
			pr_err("%s: %d rcv gpio name is null\n",__func__,i);
			return 0;
		}

		pconfig->gpio[i].gpio_no = of_get_named_gpio(pdev->dev.of_node, \
						pconfig->gpio[i].gpio_name, 0);

		if(!gpio_is_valid(pconfig->gpio[i].gpio_no)) {

			pr_err("%s: wcd get gpio %s fail\n",__func__,pconfig->gpio[i].gpio_name);
			return 0;
		} else
			pr_info("%s: wcd gpio %s no %d\n",__func__,pconfig->gpio[i].gpio_name,pconfig->gpio[i].gpio_no);

	}

	ret = of_property_read_u32(pdev->dev.of_node, "wcd-intr-gpio-dir",
		&wcd_gpio_direct);
	if (ret) {
		pr_err("Error: wcd-intr-gpio-dir not found\n");
	}
	if (wcd_gpio_direct)
		wcd_gpio_direct = GPIO_CFG_OUTPUT;
	else
		wcd_gpio_direct = GPIO_CFG_INPUT;

	pconfig->init = 1;
	return 0;
}


static __devinit int msm8974_asoc_machine_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_card_msm8974;
	struct msm8974_asoc_mach_data *pdata;
	int ret;
	

	pr_info("%s: ++\n",__func__);

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No platform supplied from device tree\n");
		return -EINVAL;
	}

	pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct msm8974_asoc_mach_data), GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev, "Can't allocate msm8974_asoc_mach_data\n");
		return -ENOMEM;
	}

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
			"qcom,taiko-mclk-clk-freq", &pdata->mclk_freq);
	if (ret) {
		dev_err(&pdev->dev, "Looking up %s property in node %s failed",
			"qcom,taiko-mclk-clk-freq",
			pdev->dev.of_node->full_name);
		goto err;
	}

	if (pdata->mclk_freq != 9600000) {
		dev_err(&pdev->dev, "unsupported taiko mclk freq %u\n",
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

	ret = msm8974_prepare_codec_mclk(card);
	if (ret)
		goto err;

	if (of_property_read_bool(pdev->dev.of_node, "qcom,hdmi-audio-rx")) {
		dev_info(&pdev->dev, "%s(): hdmi audio support present\n",
				__func__);

		memcpy(msm8974_dai_links, msm8974_common_dai_links,
			sizeof(msm8974_common_dai_links));

		memcpy(msm8974_dai_links + ARRAY_SIZE(msm8974_common_dai_links),
			msm8974_hdmi_dai_link, sizeof(msm8974_hdmi_dai_link));

		card->dai_link	= msm8974_dai_links;
		card->num_links	= ARRAY_SIZE(msm8974_dai_links);
	} else {
		dev_info(&pdev->dev, "%s(): No hdmi audio support\n", __func__);

		card->dai_link	= msm8974_common_dai_links;
		card->num_links	= ARRAY_SIZE(msm8974_common_dai_links);
	}
	mutex_init(&cdc_mclk_mutex);
	atomic_set(&prim_auxpcm_rsc_ref, 0);
	atomic_set(&sec_auxpcm_rsc_ref, 0);
	spdev = pdev;
	ext_spk_amp_regulator = NULL;
	msm8974_liquid_dock_dev = NULL;

	ret = snd_soc_register_card(card);
	if (ret == -EPROBE_DEFER)
		goto err;
	else if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err;
	}

	mutex_init(&htc_amp_mutex);

	msm8974_dtparse_mi2s(pdev, &htc_mi2s_config);
	msm8974_dtparse_rcv(pdev, &htc_rcv_config);
	msm8974_dtparse_wcd(pdev,&htc_wcd_config);

	if (of_property_read_bool(pdev->dev.of_node, "htc,audio_use_dmic")) {
		atomic_set(&dev_use_dmic,1);
	}

	
	ret = msm8974_dtparse_auxpcm(pdev, &pdata->pri_auxpcm_ctrl,
					msm_prim_auxpcm_gpio_name);
	if (ret) {
		dev_err(&pdev->dev,
		"%s: Primary Auxpcm pin data parse failed\n", __func__);
		goto err;
	}

	
	ret = msm8974_dtparse_auxpcm(pdev, &pdata->sec_auxpcm_ctrl,
					msm_sec_auxpcm_gpio_name);
	if (ret) {
		dev_err(&pdev->dev,
		"%s: Secondary Auxpcm pin data parse failed\n", __func__);
		goto err;
	}

#if 0 
	pdata->us_euro_gpio = of_get_named_gpio(pdev->dev.of_node,
				"qcom,us-euro-gpios", 0);
	if (pdata->us_euro_gpio < 0) {
		dev_err(&pdev->dev, "Looking up %s property in node %s failed",
			"qcom,us-euro-gpios",
			pdev->dev.of_node->full_name);
	} else {
		dev_dbg(&pdev->dev, "%s detected %d",
			"qcom,us-euro-gpios", pdata->us_euro_gpio);
	}
#endif
	ret = msm8974_prepare_us_euro(card);
	if (ret)
		dev_err(&pdev->dev, "msm8974_prepare_us_euro failed (%d)\n",
			ret);

#if 0 
	ret = of_property_read_string(pdev->dev.of_node,
			"qcom,prim-auxpcm-gpio-set", &auxpcm_pri_gpio_set);
	if (ret) {
		dev_err(&pdev->dev, "Looking up %s property in node %s failed",
			"qcom,prim-auxpcm-gpio-set",
			pdev->dev.of_node->full_name);
		goto err;
	}
	if (!strcmp(auxpcm_pri_gpio_set, "prim-gpio-prim")) {
		lpaif_pri_muxsel_virt_addr = ioremap(LPAIF_PRI_MODE_MUXSEL, 4);
	} else if (!strcmp(auxpcm_pri_gpio_set, "prim-gpio-tert")) {
		lpaif_pri_muxsel_virt_addr = ioremap(LPAIF_TER_MODE_MUXSEL, 4);
	} else {
		dev_err(&pdev->dev, "Invalid value %s for AUXPCM GPIO set\n",
			auxpcm_pri_gpio_set);
		ret = -EINVAL;
		goto err;
	}
	if (lpaif_pri_muxsel_virt_addr == NULL) {
		pr_err("%s Pri muxsel virt addr is null\n", __func__);
		ret = -EINVAL;
		goto err;
	}
	lpaif_sec_muxsel_virt_addr = ioremap(LPAIF_SEC_MODE_MUXSEL, 4);
	if (lpaif_sec_muxsel_virt_addr == NULL) {
		pr_err("%s Sec muxsel virt addr is null\n", __func__);
		ret = -EINVAL;
		goto err;
	}
#endif
	pr_info("%s: --\n",__func__);
	return 0;
err:
	if (pdata->mclk_gpio > 0) {
		dev_dbg(&pdev->dev, "%s free gpio %d\n",
			__func__, pdata->mclk_gpio);
		gpio_free(pdata->mclk_gpio);
		pdata->mclk_gpio = 0;
	}
	if (pdata->us_euro_gpio > 0) {
		dev_dbg(&pdev->dev, "%s free us_euro gpio %d\n",
			__func__, pdata->us_euro_gpio);
		gpio_free(pdata->us_euro_gpio);
		pdata->us_euro_gpio = 0;
	}
	mutex_destroy(&cdc_mclk_mutex);
	devm_kfree(&pdev->dev, pdata);
	return ret;
}

static int __devexit msm8974_asoc_machine_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct msm8974_asoc_mach_data *pdata = snd_soc_card_get_drvdata(card);

	if (ext_spk_amp_regulator)
		regulator_put(ext_spk_amp_regulator);

	gpio_free(pdata->mclk_gpio);
#if 0 
	gpio_free(pdata->us_euro_gpio);

	if (ext_spk_amp_gpio >= 0)
		gpio_free(ext_spk_amp_gpio);
#endif
	if (msm8974_liquid_dock_dev != NULL) {
		if (msm8974_liquid_dock_dev->dock_plug_gpio)
			gpio_free(msm8974_liquid_dock_dev->dock_plug_gpio);

		if (msm8974_liquid_dock_dev->dock_plug_irq)
			free_irq(msm8974_liquid_dock_dev->dock_plug_irq,
				 msm8974_liquid_dock_dev);

		kfree(msm8974_liquid_dock_dev);
		msm8974_liquid_dock_dev = NULL;
	}

	iounmap(lpaif_pri_muxsel_virt_addr);
	iounmap(lpaif_sec_muxsel_virt_addr);
	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id msm8974_asoc_machine_of_match[]  = {
	{ .compatible = "qcom,msm8974-audio-taiko", },
	{},
};

static struct platform_driver msm8974_asoc_machine_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = msm8974_asoc_machine_of_match,
	},
	.probe = msm8974_asoc_machine_probe,
	.remove = __devexit_p(msm8974_asoc_machine_remove),
};

static int __init m8_audio_init(void)
{
        int ret = 0;

	htc_acoustic_register_ops(&acoustic);
	platform_driver_register(&msm8974_asoc_machine_driver);

	return ret;

}
late_initcall(m8_audio_init);

static void __exit m8_audio_exit(void)
{
	pr_info("%s", __func__);
	platform_driver_unregister(&msm8974_asoc_machine_driver);
}
module_exit(m8_audio_exit);

MODULE_DESCRIPTION("ALSA SoC msm");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, msm8974_asoc_machine_of_match);
