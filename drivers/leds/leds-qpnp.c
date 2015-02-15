
/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/spmi.h>
#include <linux/qpnp/pwm.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>

#define WLED_MOD_EN_REG(base, n)	(base + 0x60 + n*0x10)
#define WLED_IDAC_DLY_REG(base, n)	(WLED_MOD_EN_REG(base, n) + 0x01)
#define WLED_FULL_SCALE_REG(base, n)	(WLED_IDAC_DLY_REG(base, n) + 0x01)
#define WLED_MOD_SRC_SEL_REG(base, n)	(WLED_FULL_SCALE_REG(base, n) + 0x01)

#define WLED_OVP_INT_STATUS(base)		(base + 0x10)
#define WLED_BRIGHTNESS_CNTL_LSB(base, n)	(base + 0x40 + 2*n)
#define WLED_BRIGHTNESS_CNTL_MSB(base, n)	(base + 0x41 + 2*n)
#define WLED_MOD_CTRL_REG(base)			(base + 0x46)
#define WLED_SYNC_REG(base)			(base + 0x47)
#define WLED_FDBCK_CTRL_REG(base)		(base + 0x48)
#define WLED_SWITCHING_FREQ_REG(base)		(base + 0x4C)
#define WLED_OVP_CFG_REG(base)			(base + 0x4D)
#define WLED_BOOST_LIMIT_REG(base)		(base + 0x4E)
#define WLED_CURR_SINK_REG(base)		(base + 0x4F)
#define WLED_HIGH_POLE_CAP_REG(base)		(base + 0x58)
#define WLED_CURR_SINK_MASK		0xE0
#define WLED_CURR_SINK_SHFT		0x05
#define WLED_DISABLE_ALL_SINKS		0x00
#define WLED_DISABLE_1_2_SINKS		0x80
#define WLED_SWITCH_FREQ_MASK		0x0F
#define WLED_OVP_VAL_MASK		0x03
#define WLED_OVP_INT_MASK		0x02
#define WLED_OVP_VAL_BIT_SHFT		0x00
#define WLED_BOOST_LIMIT_MASK		0x07
#define WLED_BOOST_LIMIT_BIT_SHFT	0x00
#define WLED_BOOST_ON			0x80
#define WLED_BOOST_OFF			0x00
#define WLED_EN_MASK			0x80
#define WLED_NO_MASK			0x00
#define WLED_CP_SELECT_MAX		0x03
#define WLED_CP_SELECT_MASK		0x02
#define WLED_USE_EXT_GEN_MOD_SRC	0x01
#define WLED_CTL_DLY_STEP		200
#define WLED_CTL_DLY_MAX		1400
#define WLED_MAX_CURR			25
#define WLED_NO_CURRENT			0x00
#define WLED_OVP_DELAY			1000
#define WLED_OVP_DELAY_INT		200
#define WLED_OVP_DELAY_LOOP		100
#define WLED_MSB_MASK			0x0F
#define WLED_MAX_CURR_MASK		0x1F
#define WLED_OP_FDBCK_MASK		0x07
#define WLED_OP_FDBCK_BIT_SHFT		0x00
#define WLED_OP_FDBCK_DEFAULT		0x00

#define WLED_SET_ILIM_CODE		0x01

#define WLED_MAX_LEVEL			4095
#define WLED_8_BIT_MASK			0xFF
#define WLED_4_BIT_MASK			0x0F
#define WLED_8_BIT_SHFT			0x08
#define WLED_MAX_DUTY_CYCLE		0xFFF

#define WLED_SYNC_VAL			0x07
#define WLED_SYNC_RESET_VAL		0x00

#define PMIC_VER_8026			0x04
#define PMIC_VER_8941			0x01
#define PMIC_VERSION_REG		0x0105

#define WLED_DEFAULT_STRINGS		0x01
#define WLED_THREE_STRINGS		0x03
#define WLED_MAX_TRIES			5
#define WLED_DEFAULT_OVP_VAL		0x02
#define WLED_BOOST_LIM_DEFAULT		0x03
#define WLED_CP_SEL_DEFAULT		0x00
#define WLED_CTRL_DLY_DEFAULT		0x00
#define WLED_SWITCH_FREQ_DEFAULT	0x0B

#define FLASH_SAFETY_TIMER(base)	(base + 0x40)
#define FLASH_MAX_CURR(base)		(base + 0x41)
#define FLASH_LED_0_CURR(base)		(base + 0x42)
#define FLASH_LED_1_CURR(base)		(base + 0x43)
#define FLASH_CLAMP_CURR(base)		(base + 0x44)
#define FLASH_LED_TMR_CTRL(base)	(base + 0x48)
#define FLASH_HEADROOM(base)		(base + 0x4A)
#define FLASH_STARTUP_DELAY(base)	(base + 0x4B)
#define FLASH_MASK_ENABLE(base)		(base + 0x4C)
#define FLASH_VREG_OK_FORCE(base)	(base + 0x4F)
#define FLASH_ENABLE_CONTROL(base)	(base + 0x46)
#define FLASH_LED_STROBE_CTRL(base)	(base + 0x47)
#define FLASH_WATCHDOG_TMR(base)	(base + 0x49)
#define FLASH_FAULT_DETECT(base)	(base + 0x51)
#define FLASH_PERIPHERAL_SUBTYPE(base)	(base + 0x05)
#define FLASH_CURRENT_RAMP(base)	(base + 0x54)

#define FLASH_MAX_LEVEL			0x4F
#define TORCH_MAX_LEVEL			0x0F
#define	FLASH_NO_MASK			0x00

#define FLASH_MASK_1			0x20
#define FLASH_MASK_REG_MASK		0xE0
#define FLASH_HEADROOM_MASK		0x03
#define FLASH_SAFETY_TIMER_MASK		0x7F
#define FLASH_CURRENT_MASK		0xFF
#define FLASH_MAX_CURRENT_MASK		0x7F
#define FLASH_TMR_MASK			0x03
#define FLASH_TMR_WATCHDOG		0x03
#define FLASH_TMR_SAFETY		0x00
#define FLASH_FAULT_DETECT_MASK		0X80
#define FLASH_HW_VREG_OK		0x40
#define FLASH_SW_VREG_OK                0x80
#define FLASH_VREG_MASK			0xC0
#define FLASH_STARTUP_DLY_MASK		0x02
#define FLASH_CURRENT_RAMP_MASK		0xBF

#define FLASH_ENABLE_ALL		0xE0
#define FLASH_ENABLE_MODULE		0x80
#define FLASH_ENABLE_MODULE_MASK	0x80
#define FLASH_DISABLE_ALL		0x00
#define FLASH_ENABLE_MASK		0xE0
#define FLASH_ENABLE_LED_0		0xC0
#define FLASH_ENABLE_LED_1		0xA0
#define FLASH_INIT_MASK			0xE0
#define	FLASH_SELFCHECK_ENABLE		0x80
#define FLASH_WATCHDOG_MASK		0x1F
#define FLASH_RAMP_STEP_27US		0xBF

#define FLASH_HW_SW_STROBE_SEL_MASK	0x04
#define FLASH_STROBE_MASK		0xC7
#define FLASH_LED_0_OUTPUT		0x80
#define FLASH_LED_1_OUTPUT		0x40
#define FLASH_TORCH_OUTPUT		0xC0

#define FLASH_CURRENT_PRGM_MIN		1
#define FLASH_CURRENT_PRGM_SHIFT	1
#define FLASH_CURRENT_MAX		0x4F
#define FLASH_CURRENT_TORCH		0x07

#define FLASH_DURATION_200ms		0x13
#define TORCH_DURATION_12s		0x0A
#define FLASH_CLAMP_200mA		0x0F

#define FLASH_SUBTYPE_DUAL		0x01
#define FLASH_SUBTYPE_SINGLE		0x02

#define FLASH_RAMP_UP_DELAY_US		1000
#define FLASH_RAMP_DN_DELAY_US		2160

#define LED_TRIGGER_DEFAULT		"none"

#define RGB_LED_SRC_SEL(base)		(base + 0x45)
#define RGB_LED_EN_CTL(base)		(base + 0x46)
#define RGB_LED_ATC_CTL(base)		(base + 0x47)

#define RGB_MAX_LEVEL			LED_FULL
#define RGB_LED_ENABLE_RED		0x80
#define RGB_LED_ENABLE_GREEN		0x40
#define RGB_LED_ENABLE_BLUE		0x20
#define RGB_LED_SOURCE_VPH_PWR		0x01
#define RGB_LED_ENABLE_MASK		0xE0
#define RGB_LED_SRC_MASK		0x03
#define QPNP_LED_PWM_FLAGS	(PM_PWM_LUT_LOOP | PM_PWM_LUT_RAMP_UP)
#define QPNP_LUT_RAMP_STEP_DEFAULT	255
#define	PWM_LUT_MAX_SIZE		63
#define	PWM_GPLED_LUT_MAX_SIZE		31
#define RGB_LED_DISABLE			0x00

#define MPP_MAX_LEVEL			LED_FULL
#define LED_MPP_MODE_CTRL(base)		(base + 0x40)
#define LED_MPP_VIN_CTRL(base)		(base + 0x41)
#define LED_MPP_EN_CTRL(base)		(base + 0x46)
#define LED_MPP_SINK_CTRL(base)		(base + 0x4C)

#define LED_MPP_CURRENT_MIN		5
#define LED_MPP_CURRENT_MAX		40
#define LED_MPP_VIN_CTRL_DEFAULT	0
#define LED_MPP_CURRENT_PER_SETTING	5
#define LED_MPP_SOURCE_SEL_DEFAULT	LED_MPP_MODE_ENABLE

#define LED_MPP_SINK_MASK		0x07
#define LED_MPP_MODE_MASK		0x7F
#define LED_MPP_VIN_MASK		0x03
#define LED_MPP_EN_MASK			0x80
#define LED_MPP_SRC_MASK		0x0F
#define LED_MPP_MODE_CTRL_MASK		0x70

#define LED_MPP_MODE_SINK		(0x06 << 4)
#define LED_MPP_MODE_ENABLE		0x01
#define LED_MPP_MODE_OUTPUT		0x10
#define LED_MPP_MODE_DISABLE		0x00
#define LED_MPP_EN_ENABLE		0x80
#define LED_MPP_EN_DISABLE		0x00

#define MPP_SOURCE_DTEST1		0x08

#define GPIO_MAX_LEVEL			LED_FULL
#define LED_GPIO_MODE_CTRL(base)	(base + 0x40)
#define LED_GPIO_VIN_CTRL(base)		(base + 0x41)
#define LED_GPIO_EN_CTRL(base)		(base + 0x46)

#define LED_GPIO_VIN_CTRL_DEFAULT	0
#define LED_GPIO_SOURCE_SEL_DEFAULT	LED_GPIO_MODE_ENABLE

#define LED_GPIO_MODE_MASK		0x3F
#define LED_GPIO_VIN_MASK		0x0F
#define LED_GPIO_EN_MASK		0x80
#define LED_GPIO_SRC_MASK		0x0F
#define LED_GPIO_MODE_CTRL_MASK		0x30

#define LED_GPIO_MODE_ENABLE	0x01
#define LED_GPIO_MODE_DISABLE	0x00
#define LED_GPIO_MODE_OUTPUT		0x10
#define LED_GPIO_EN_ENABLE		0x80
#define LED_GPIO_EN_DISABLE		0x00

#define KPDBL_MAX_LEVEL			LED_FULL
#define KPDBL_ROW_SRC_SEL(base)		(base + 0x40)
#define KPDBL_ENABLE(base)		(base + 0x46)
#define KPDBL_ROW_SRC(base)		(base + 0xE5)

#define KPDBL_ROW_SRC_SEL_VAL_MASK	0x0F
#define KPDBL_ROW_SCAN_EN_MASK		0x80
#define KPDBL_ROW_SCAN_VAL_MASK		0x0F
#define KPDBL_ROW_SCAN_EN_SHIFT		7
#define KPDBL_MODULE_EN			0x80
#define KPDBL_MODULE_DIS		0x00
#define KPDBL_MODULE_EN_MASK		0x80

#define LED_DBG(fmt, ...) \
		({ if (0) printk(KERN_DEBUG "[LED]" fmt, ##__VA_ARGS__); })
#define LED_INFO(fmt, ...) \
		printk(KERN_ERR "[LED]" fmt, ##__VA_ARGS__)
#define LED_ERR(fmt, ...) \
		printk(KERN_ERR "[LED][ERR]" fmt, ##__VA_ARGS__)
static int lut_on [] = {0, 0, 15, 30, 45, 60, 75, 90, 100};
static int lut_off [] = {100, 100, 90, 75, 60, 45, 30, 10, 0};
struct wake_lock pmic_led_wake_lock;
enum qpnp_leds {
	QPNP_ID_WLED = 0,
	QPNP_ID_FLASH1_LED0,
	QPNP_ID_FLASH1_LED1,
	QPNP_ID_RGB_RED,
	QPNP_ID_RGB_GREEN,
	QPNP_ID_RGB_BLUE,
	QPNP_ID_LED_MPP,
	QPNP_ID_KPDBL,
	QPNP_ID_LED_GPIO,
	QPNP_ID_MAX,
};

enum wled_current_boost_limit {
	WLED_CURR_LIMIT_105mA,
	WLED_CURR_LIMIT_385mA,
	WLED_CURR_LIMIT_525mA,
	WLED_CURR_LIMIT_805mA,
	WLED_CURR_LIMIT_980mA,
	WLED_CURR_LIMIT_1260mA,
	WLED_CURR_LIMIT_1400mA,
	WLED_CURR_LIMIT_1680mA,
};

enum wled_ovp_threshold {
	WLED_OVP_35V,
	WLED_OVP_32V,
	WLED_OVP_29V,
	WLED_OVP_27V,
};

enum flash_headroom {
	HEADROOM_250mV = 0,
	HEADROOM_300mV,
	HEADROOM_400mV,
	HEADROOM_500mV,
};

enum flash_startup_dly {
	DELAY_10us = 0,
	DELAY_32us,
	DELAY_64us,
	DELAY_128us,
};

enum rgb_mode {
	RGB_MODE_PWM = 0,
	RGB_MODE_LPG,
	RGB_MANUAL_MODE,
};
enum led_mode {
	PWM_MODE = 0,
	LPG_MODE,
	MANUAL_MODE,
};

enum led_status {
	OFF = 0,
	ON,
	BLINK,
};

enum pm8xxx_blink_type {
	BLINK_STOP = -1,
	BLINK_UNCHANGE = 0,
	BLINK_64MS_PER_2SEC,
	BLINK_64MS_ON_310MS_PER_2SEC,
	BLINK_64MS_ON_2SEC_PER_2SEC,
	BLINK_1SEC_PER_2SEC,
};
static u8 wled_debug_regs[] = {
	
	0x40, 0x41, 0x42, 0x43, 0x44, 0x45,
	
	0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
	
	0x60, 0x61, 0x62, 0x63, 0x66,
	
	0x70, 0x71, 0x72, 0x73, 0x76,
	
	0x80, 0x81, 0x82, 0x83, 0x86,
};

static u8 flash_debug_regs[] = {
	0x40, 0x41, 0x42, 0x43, 0x44, 0x48, 0x49, 0x4b, 0x4c,
	0x4f, 0x46, 0x47,
};

static u8 rgb_pwm_debug_regs[] = {
	0x45, 0x46, 0x47,
};

static u8 mpp_debug_regs[] = {
	0x40, 0x41, 0x42, 0x45, 0x46, 0x4c,
};

static u8 kpdbl_debug_regs[] = {
	0x40, 0x46, 0xb1, 0xb3, 0xb4, 0xe5,
};

static u8 gpio_debug_regs[] = {
	0x40, 0x41, 0x42, 0x45, 0x46,
};

struct pwm_config_data {
	struct lut_params	lut_params;
	struct pwm_device	*pwm_dev;
	int			pwm_channel;
	u32			pwm_period_us;
	u32                     pwm_duty_us;
	struct pwm_duty_cycles	*duty_cycles;
	int	*old_duty_pcts;
	u8	mode;
	u8	default_mode;
	bool	pwm_enabled;
	bool use_blink;
	bool blinking;
	int 	pwm_coefficient;
};

struct wled_config_data {
	u8	num_strings;
	u8	num_physical_strings;
	u8	ovp_val;
	u8	boost_curr_lim;
	u8	cp_select;
	u8	ctrl_delay_us;
	u8	switch_freq;
	u8	op_fdbck;
	u8	pmic_version;
	bool	dig_mod_gen_en;
	bool	cs_out_en;
};

struct mpp_config_data {
	struct pwm_config_data	*pwm_cfg;
	u8	current_setting;
	u8	source_sel;
	u8	mode_ctrl;
	u8	vin_ctrl;
	u8	min_brightness;
	u8 pwm_mode;
	u32	max_uV;
	u32	min_uV;
	struct regulator *mpp_reg;
	bool	enable;
	u8 blink_mode;
};

/**
 *  flash_config_data - flash configuration data
 *  @current_prgm - current to be programmed, scaled by max level
 *  @clamp_curr - clamp current to use
 *  @headroom - headroom value to use
 *  @duration - duration of the flash
 *  @enable_module - enable address for particular flash
 *  @trigger_flash - trigger flash
 *  @startup_dly - startup delay for flash
 *  @strobe_type - select between sw and hw strobe
 *  @peripheral_subtype - module peripheral subtype
 *  @current_addr - address to write for current
 *  @second_addr - address of secondary flash to be written
 *  @safety_timer - enable safety timer or watchdog timer
 *  @torch_enable - enable flash LED torch mode
 *  @flash_reg_get - flash regulator attached or not
 *  @flash_on - flash status, on or off
 *  @torch_on - torch status, on or off
 *  @vreg_ok - specifies strobe type, sw or hw
 *  @no_smbb_support - specifies if smbb boost is not required and there is a
    single regulator for both flash and torch
 *  @flash_boost_reg - boost regulator for flash
 *  @torch_boost_reg - boost regulator for torch
 *  @flash_wa_reg - flash regulator for wa
 */
struct flash_config_data {
	u8	current_prgm;
	u8	clamp_curr;
	u8	headroom;
	u8	duration;
	u8	enable_module;
	u8	trigger_flash;
	u8	startup_dly;
	u8	strobe_type;
	u8	peripheral_subtype;
	u16	current_addr;
	u16	second_addr;
	bool	safety_timer;
	bool	torch_enable;
	bool	flash_reg_get;
	bool	flash_on;
	bool	torch_on;
	bool	vreg_ok;
	bool    no_smbb_support;
	struct regulator *flash_boost_reg;
	struct regulator *torch_boost_reg;
	struct regulator *flash_wa_reg;
};

struct kpdbl_config_data {
	struct pwm_config_data	*pwm_cfg;
	u32	row_id;
	bool	row_src_vbst;
	bool	row_src_en;
	bool	always_on;
	struct pwm_duty_cycles  *duty_cycles;
	struct lut_params	lut_params;
};

struct rgb_config_data {
	struct pwm_config_data	*pwm_cfg;
	u8	enable;
};

struct gpio_config_data {
	u8	source_sel;
	u8	mode_ctrl;
	u8	vin_ctrl;
	bool	enable;
};

struct qpnp_led_data {
	struct led_classdev	cdev;
	struct spmi_device	*spmi_dev;
	struct delayed_work	dwork;
	struct workqueue_struct *workqueue;
	struct work_struct	work;
	int			id;
	u16			base;
	u8			reg;
	u8			num_leds;
	struct mutex		lock;
	struct wled_config_data *wled_cfg;
	struct flash_config_data	*flash_cfg;
	struct kpdbl_config_data	*kpdbl_cfg;
	struct rgb_config_data	*rgb_cfg;
	struct mpp_config_data	*mpp_cfg;
	struct gpio_config_data	*gpio_cfg;
	int			max_current;
	bool			default_on;
	bool                    in_order_command_processing;
	struct delayed_work	blink_delayed_work;
	struct delayed_work	gpled_blink_delayed_work;
	struct delayed_work 	fade_delayed_work;
	int			turn_off_delay_ms;
	
	struct work_struct 		led_off_work;
	int status;
	int mode;
	struct work_struct 		led_blink_work;
};
static struct workqueue_struct *g_led_work_queue;
static struct workqueue_struct *g_gpled_work_queue;
static struct workqueue_struct *g_led_on_work_queue;

static int current_blink = 0;
static int num_kpbl_leds_on;
static DEFINE_MUTEX(flash_lock);

static int
qpnp_led_masked_write(struct qpnp_led_data *led, u16 addr, u8 mask, u8 val)
{
	int rc;
	u8 reg;

	rc = spmi_ext_register_readl(led->spmi_dev->ctrl, led->spmi_dev->sid,
		addr, &reg, 1);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Unable to read from addr=%x, rc(%d)\n", addr, rc);
	}

	reg &= ~mask;
	reg |= val;

	rc = spmi_ext_register_writel(led->spmi_dev->ctrl, led->spmi_dev->sid,
		addr, &reg, 1);
	if (rc)
		dev_err(&led->spmi_dev->dev,
			"Unable to write to addr=%x, rc(%d)\n", addr, rc);
	return rc;
}

static void qpnp_dump_regs(struct qpnp_led_data *led, u8 regs[], u8 array_size)
{
	int i;
	u8 val;

	pr_debug("===== %s LED register dump start =====\n", led->cdev.name);
	for (i = 0; i < array_size; i++) {
		spmi_ext_register_readl(led->spmi_dev->ctrl,
					led->spmi_dev->sid,
					led->base + regs[i],
					&val, sizeof(val));
		pr_debug("%s: 0x%x = 0x%x\n", led->cdev.name,
					led->base + regs[i], val);
	}
	pr_debug("===== %s LED register dump end =====\n", led->cdev.name);
}

static int qpnp_wled_sync(struct qpnp_led_data *led)
{
	int rc;
	u8 val;

	
	val = WLED_SYNC_VAL;
	rc = spmi_ext_register_writel(led->spmi_dev->ctrl, led->spmi_dev->sid,
		WLED_SYNC_REG(led->base), &val, 1);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
				"WLED set sync reg failed(%d)\n", rc);
		return rc;
	}

	val = WLED_SYNC_RESET_VAL;
	rc = spmi_ext_register_writel(led->spmi_dev->ctrl, led->spmi_dev->sid,
		WLED_SYNC_REG(led->base), &val, 1);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
				"WLED reset sync reg failed(%d)\n", rc);
		return rc;
	}
	return 0;
}

static int qpnp_wled_set(struct qpnp_led_data *led)
{
	int rc, duty, level, tries = 0;
	u8 val, i, num_wled_strings, sink_val, ilim_val, ovp_val;

	num_wled_strings = led->wled_cfg->num_strings;

	level = led->cdev.brightness;

	if (level > WLED_MAX_LEVEL)
		level = WLED_MAX_LEVEL;
	if (level == 0) {
		for (i = 0; i < num_wled_strings; i++) {
			rc = qpnp_led_masked_write(led,
				WLED_FULL_SCALE_REG(led->base, i),
				WLED_MAX_CURR_MASK, WLED_NO_CURRENT);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Write max current failure (%d)\n",
					rc);
				return rc;
			}
		}

		rc = qpnp_wled_sync(led);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"WLED sync failed(%d)\n", rc);
			return rc;
		}

		rc = spmi_ext_register_readl(led->spmi_dev->ctrl,
			led->spmi_dev->sid, WLED_CURR_SINK_REG(led->base),
			&sink_val, 1);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"WLED read sink reg failed(%d)\n", rc);
			return rc;
		}

		if (led->wled_cfg->pmic_version == PMIC_VER_8026) {
			val = WLED_DISABLE_ALL_SINKS;
			rc = spmi_ext_register_writel(led->spmi_dev->ctrl,
				led->spmi_dev->sid,
				WLED_CURR_SINK_REG(led->base), &val, 1);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"WLED write sink reg failed(%d)\n", rc);
				return rc;
			}

			usleep(WLED_OVP_DELAY);
		} else if (led->wled_cfg->pmic_version == PMIC_VER_8941) {
			if (led->wled_cfg->num_physical_strings <=
					WLED_THREE_STRINGS) {
				val = WLED_DISABLE_1_2_SINKS;
				rc = spmi_ext_register_writel(
					led->spmi_dev->ctrl,
					led->spmi_dev->sid,
					WLED_CURR_SINK_REG(led->base), &val, 1);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
						"WLED write sink reg failed");
					return rc;
				}

				rc = spmi_ext_register_readl(
					led->spmi_dev->ctrl,
					led->spmi_dev->sid,
					WLED_BOOST_LIMIT_REG(led->base),
					&ilim_val, 1);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
						"Unable to read boost reg");
				}
				val = WLED_SET_ILIM_CODE;
				rc = spmi_ext_register_writel(
					led->spmi_dev->ctrl,
					led->spmi_dev->sid,
					WLED_BOOST_LIMIT_REG(led->base),
					&val, 1);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
						"WLED write sink reg failed");
					return rc;
				}
				usleep(WLED_OVP_DELAY);
			} else {
				val = WLED_DISABLE_ALL_SINKS;
				rc = spmi_ext_register_writel(
					led->spmi_dev->ctrl,
					led->spmi_dev->sid,
					WLED_CURR_SINK_REG(led->base), &val, 1);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
						"WLED write sink reg failed");
					return rc;
				}

				msleep(WLED_OVP_DELAY_INT);
				while (tries < WLED_MAX_TRIES) {
					rc = spmi_ext_register_readl(
						led->spmi_dev->ctrl,
						led->spmi_dev->sid,
						WLED_OVP_INT_STATUS(led->base),
						&ovp_val, 1);
					if (rc) {
						dev_err(&led->spmi_dev->dev,
						"Unable to read boost reg");
					}

					if (ovp_val & WLED_OVP_INT_MASK)
						break;

					msleep(WLED_OVP_DELAY_LOOP);
					tries++;
				}
				usleep(WLED_OVP_DELAY);
			}
		}

		val = WLED_BOOST_OFF;
		rc = spmi_ext_register_writel(led->spmi_dev->ctrl,
			led->spmi_dev->sid, WLED_MOD_CTRL_REG(led->base),
			&val, 1);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"WLED write ctrl reg failed(%d)\n", rc);
			return rc;
		}

		for (i = 0; i < num_wled_strings; i++) {
			rc = qpnp_led_masked_write(led,
				WLED_FULL_SCALE_REG(led->base, i),
				WLED_MAX_CURR_MASK, (u8)led->max_current);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Write max current failure (%d)\n",
					rc);
				return rc;
			}
		}

		rc = qpnp_wled_sync(led);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"WLED sync failed(%d)\n", rc);
			return rc;
		}

		if (led->wled_cfg->pmic_version == PMIC_VER_8941) {
			if (led->wled_cfg->num_physical_strings <=
					WLED_THREE_STRINGS) {
				rc = spmi_ext_register_writel(
					led->spmi_dev->ctrl,
					led->spmi_dev->sid,
					WLED_BOOST_LIMIT_REG(led->base),
					&ilim_val, 1);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
						"WLED write sink reg failed");
					return rc;
				}
			} else {
				
				rc = spmi_ext_register_writel(
					led->spmi_dev->ctrl,
					led->spmi_dev->sid,
					WLED_OVP_CFG_REG(led->base),
					&led->wled_cfg->ovp_val, 1);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
						"WLED write sink reg failed");
					return rc;
				}
			}
		}

		
		rc = spmi_ext_register_writel(led->spmi_dev->ctrl,
			led->spmi_dev->sid, WLED_CURR_SINK_REG(led->base),
			&sink_val, 1);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"WLED write sink reg failed(%d)\n", rc);
			return rc;
		}

	} else {
		val = WLED_BOOST_ON;
		rc = spmi_ext_register_writel(led->spmi_dev->ctrl,
			led->spmi_dev->sid, WLED_MOD_CTRL_REG(led->base),
			&val, 1);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"WLED write ctrl reg failed(%d)\n", rc);
			return rc;
		}
	}

	duty = (WLED_MAX_DUTY_CYCLE * level) / WLED_MAX_LEVEL;

	
	for (i = 0; i < num_wled_strings; i++) {
		rc = qpnp_led_masked_write(led,
			WLED_BRIGHTNESS_CNTL_MSB(led->base, i), WLED_MSB_MASK,
			(duty >> WLED_8_BIT_SHFT) & WLED_4_BIT_MASK);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"WLED set brightness MSB failed(%d)\n", rc);
			return rc;
		}
		val = duty & WLED_8_BIT_MASK;
		rc = spmi_ext_register_writel(led->spmi_dev->ctrl,
			led->spmi_dev->sid,
			WLED_BRIGHTNESS_CNTL_LSB(led->base, i), &val, 1);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"WLED set brightness LSB failed(%d)\n", rc);
			return rc;
		}
	}

	rc = qpnp_wled_sync(led);
	if (rc) {
		dev_err(&led->spmi_dev->dev, "WLED sync failed(%d)\n", rc);
		return rc;
	}
	return 0;
}

static int qpnp_mpp_set(struct qpnp_led_data *led, int blink_mode)
{
	int rc;
	u8 val;
	int duty_us, duty_ns, period_us, pwm_period;
	mutex_lock(&led->lock);
	LED_INFO("%s, name:%s, brightness = %d, blink = %d\n", __func__, led->cdev.name, led->cdev.brightness, blink_mode);
	if (led->cdev.brightness) {
		if (led->mpp_cfg->mpp_reg && !led->mpp_cfg->enable) {
			rc = regulator_set_voltage(led->mpp_cfg->mpp_reg,
					led->mpp_cfg->min_uV,
					led->mpp_cfg->max_uV);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Regulator voltage set failed rc=%d\n",
									rc);
				return rc;
			}

			rc = regulator_enable(led->mpp_cfg->mpp_reg);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Regulator enable failed(%d)\n", rc);
				goto err_reg_enable;
			}
		}

		led->mpp_cfg->enable = true;

		if (led->cdev.brightness < led->mpp_cfg->min_brightness) {
			dev_warn(&led->spmi_dev->dev,
				"brightness is less than supported..." \
				"set to minimum supported\n");
			led->cdev.brightness = led->mpp_cfg->min_brightness;
		}

		if (led->mpp_cfg->pwm_mode != MANUAL_MODE) {
			if (!led->mpp_cfg->pwm_cfg->blinking) {
				led->mpp_cfg->pwm_cfg->mode =
					led->mpp_cfg->pwm_cfg->default_mode;
				led->mpp_cfg->pwm_mode =
					led->mpp_cfg->pwm_cfg->default_mode;
			}
		}
		if (led->mpp_cfg->pwm_mode == PWM_MODE) {
			
			period_us = led->mpp_cfg->pwm_cfg->pwm_period_us;
			if (period_us > INT_MAX / NSEC_PER_USEC) {
				duty_us = (period_us * led->cdev.brightness) /
					LED_FULL;
				rc = pwm_config_us(
					led->mpp_cfg->pwm_cfg->pwm_dev,
					duty_us,
					period_us);
			} else {
				duty_ns = ((period_us * NSEC_PER_USEC) /
					LED_FULL) * led->cdev.brightness;
				rc = pwm_config(
					led->mpp_cfg->pwm_cfg->pwm_dev,
					duty_ns,
					period_us * NSEC_PER_USEC);
			}
			if (rc < 0) {
				dev_err(&led->spmi_dev->dev, "Failed to " \
					"configure pwm for new values\n");
				goto err_mpp_reg_write;
			}
		}
		if (blink_mode) {
			switch (blink_mode) {
				case BLINK_64MS_PER_2SEC:
					pwm_disable(led->mpp_cfg->pwm_cfg->pwm_dev);
					duty_us = 64000;
					pwm_period = 2000000;
					rc = pwm_config_us(led->mpp_cfg->pwm_cfg->pwm_dev,
							duty_us,
							pwm_period);
					if (rc < 0) {
						dev_err(&led->spmi_dev->dev, "Failed to configure pwm for new values\n");
						mutex_unlock(&led->lock);
						return rc;
					}
					break;
				case BLINK_64MS_ON_310MS_PER_2SEC:
					cancel_delayed_work_sync(&led->blink_delayed_work);
					pwm_disable(led->mpp_cfg->pwm_cfg->pwm_dev);
					rc = qpnp_led_masked_write(led,
							LED_MPP_MODE_CTRL(led->base),
							LED_MPP_MODE_MASK,
							LED_MPP_MODE_DISABLE);
					if (rc) {
						dev_err(&led->spmi_dev->dev, "Failed to write led mode reg\n");
						mutex_unlock(&led->lock);
						return rc;
					}
					rc = qpnp_led_masked_write(led,
							LED_MPP_EN_CTRL(led->base),
							LED_MPP_EN_MASK,
							LED_MPP_EN_DISABLE);
					if (rc) {
						dev_err(&led->spmi_dev->dev, "Failed to write led enable reg\n");
						mutex_unlock(&led->lock);
						return rc;
					}
					led->mpp_cfg->pwm_cfg->pwm_duty_us = 64000;
					led->mpp_cfg->pwm_cfg->pwm_period_us = 2000000;
					queue_delayed_work(g_led_work_queue, &led->blink_delayed_work,
							msecs_to_jiffies(310));
					break;
				case BLINK_64MS_ON_2SEC_PER_2SEC:
					cancel_delayed_work_sync(&led->blink_delayed_work);
					pwm_disable(led->mpp_cfg->pwm_cfg->pwm_dev);
					rc = qpnp_led_masked_write(led,
							LED_MPP_MODE_CTRL(led->base),
							LED_MPP_MODE_MASK,
							LED_MPP_MODE_DISABLE);
					if (rc) {
						dev_err(&led->spmi_dev->dev, "Failed to write led mode reg\n");
						mutex_unlock(&led->lock);
						return rc;
					}
					rc = qpnp_led_masked_write(led,
							LED_MPP_EN_CTRL(led->base),
							LED_MPP_EN_MASK,
							LED_MPP_EN_DISABLE);
					if (rc) {
						dev_err(&led->spmi_dev->dev, "Failed to write led enable reg\n");
						mutex_unlock(&led->lock);
						return rc;
					}
					led->mpp_cfg->pwm_cfg->pwm_duty_us = 64000;
					led->mpp_cfg->pwm_cfg->pwm_period_us = 2000000;
					queue_delayed_work(g_led_work_queue, &led->blink_delayed_work,
							msecs_to_jiffies(1000));
					break;
				case BLINK_1SEC_PER_2SEC:
					pwm_disable(led->mpp_cfg->pwm_cfg->pwm_dev);
					duty_us = 1000000;
					pwm_period = 2000000;
					rc = pwm_config_us(led->mpp_cfg->pwm_cfg->pwm_dev,
							duty_us,
							pwm_period);
					if (rc < 0) {
						dev_err(&led->spmi_dev->dev, "Failed to configure pwm for new values\n");
						mutex_unlock(&led->lock);
						return rc;
					}
					break;
			}
		} else {
			pwm_disable(led->mpp_cfg->pwm_cfg->pwm_dev);
			duty_us = 640*led->mpp_cfg->pwm_cfg->pwm_coefficient/255;
			pwm_period = 640;
			rc = pwm_config_us(led->mpp_cfg->pwm_cfg->pwm_dev,
					duty_us,
					pwm_period);
			if (rc < 0) {
				dev_err(&led->spmi_dev->dev, "Failed to configure pwm for new values\n");
				mutex_unlock(&led->lock);
				return rc;
			}
		}
		if (blink_mode != BLINK_64MS_ON_310MS_PER_2SEC &&
				blink_mode != BLINK_64MS_ON_2SEC_PER_2SEC) {
			if (led->mpp_cfg->pwm_mode != MANUAL_MODE) {
				pwm_enable(led->mpp_cfg->pwm_cfg->pwm_dev);
				mdelay(10);
			}

		if (led->mpp_cfg->pwm_mode != MANUAL_MODE)
			pwm_enable(led->mpp_cfg->pwm_cfg->pwm_dev);
		else {
			if (led->cdev.brightness < LED_MPP_CURRENT_MIN)
				led->cdev.brightness = LED_MPP_CURRENT_MIN;
			else {
				led->cdev.brightness /= LED_MPP_CURRENT_MIN;
				led->cdev.brightness *= LED_MPP_CURRENT_MIN;
			}

			val = (led->cdev.brightness / LED_MPP_CURRENT_MIN) - 1;

			rc = qpnp_led_masked_write(led,
					LED_MPP_SINK_CTRL(led->base),
					LED_MPP_SINK_MASK, val);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Failed to write sink control reg\n");
				goto err_mpp_reg_write;
			}
		}

		val = (led->mpp_cfg->source_sel & LED_MPP_SRC_MASK) |
			(led->mpp_cfg->mode_ctrl & LED_MPP_MODE_CTRL_MASK);

		rc = qpnp_led_masked_write(led,
			LED_MPP_MODE_CTRL(led->base), LED_MPP_MODE_MASK,
			val);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
					"Failed to write led mode reg\n");
			goto err_mpp_reg_write;
		}

		rc = qpnp_led_masked_write(led,
				LED_MPP_EN_CTRL(led->base), LED_MPP_EN_MASK,
				LED_MPP_EN_ENABLE);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
					"Failed to write led enable " \
					"reg\n");
			goto err_mpp_reg_write;
		}
			
			if (blink_mode == 5) {
				LED_INFO("MPP LED breath\n");
			}
	} else {
		if (led->mpp_cfg->pwm_mode != MANUAL_MODE) {
			led->mpp_cfg->pwm_cfg->mode =
				led->mpp_cfg->pwm_cfg->default_mode;
			led->mpp_cfg->pwm_mode =
				led->mpp_cfg->pwm_cfg->default_mode;
			pwm_disable(led->mpp_cfg->pwm_cfg->pwm_dev);
		}
		rc = qpnp_led_masked_write(led,
					LED_MPP_MODE_CTRL(led->base),
					LED_MPP_MODE_MASK,
					LED_MPP_MODE_DISABLE);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
					"Failed to write led mode reg\n");
			goto err_mpp_reg_write;
		}

		rc = qpnp_led_masked_write(led,
					LED_MPP_EN_CTRL(led->base),
					LED_MPP_EN_MASK,
					LED_MPP_EN_DISABLE);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
					"Failed to write led enable reg\n");
			goto err_mpp_reg_write;
		}

		if (led->mpp_cfg->mpp_reg && led->mpp_cfg->enable) {
			rc = regulator_disable(led->mpp_cfg->mpp_reg);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"MPP regulator disable failed(%d)\n",
					rc);
				return rc;
			}

			rc = regulator_set_voltage(led->mpp_cfg->mpp_reg,
						0, led->mpp_cfg->max_uV);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"MPP regulator voltage set failed(%d)\n",
					rc);
				return rc;
			}
		}

		led->mpp_cfg->enable = false;
	}
	}
	if (led->mpp_cfg->pwm_mode != MANUAL_MODE)
		led->mpp_cfg->pwm_cfg->blinking = false;
	qpnp_dump_regs(led, mpp_debug_regs, ARRAY_SIZE(mpp_debug_regs));
		mutex_unlock(&led->lock);

	return 0;

err_mpp_reg_write:
	if (led->mpp_cfg->mpp_reg)
		regulator_disable(led->mpp_cfg->mpp_reg);
err_reg_enable:
	if (led->mpp_cfg->mpp_reg)
		regulator_set_voltage(led->mpp_cfg->mpp_reg, 0,
							led->mpp_cfg->max_uV);
	led->mpp_cfg->enable = false;
		mutex_unlock(&led->lock);

	return rc;
}

static int qpnp_gpio_set(struct qpnp_led_data *led)
{
	int rc, val;

	if (led->cdev.brightness) {
		val = (led->gpio_cfg->source_sel & LED_GPIO_SRC_MASK) |
			(led->gpio_cfg->mode_ctrl & LED_GPIO_MODE_CTRL_MASK);

		rc = qpnp_led_masked_write(led,
			 LED_GPIO_MODE_CTRL(led->base),
			 LED_GPIO_MODE_MASK,
			 val);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
					"Failed to write led mode reg\n");
			goto err_gpio_reg_write;
		}

		rc = qpnp_led_masked_write(led,
			 LED_GPIO_EN_CTRL(led->base),
			 LED_GPIO_EN_MASK,
			 LED_GPIO_EN_ENABLE);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
					"Failed to write led enable reg\n");
			goto err_gpio_reg_write;
		}

		led->gpio_cfg->enable = true;
	} else {
		rc = qpnp_led_masked_write(led,
				LED_GPIO_MODE_CTRL(led->base),
				LED_GPIO_MODE_MASK,
				LED_GPIO_MODE_DISABLE);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
					"Failed to write led mode reg\n");
			goto err_gpio_reg_write;
		}

		rc = qpnp_led_masked_write(led,
				LED_GPIO_EN_CTRL(led->base),
				LED_GPIO_EN_MASK,
				LED_GPIO_EN_DISABLE);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
					"Failed to write led enable reg\n");
			goto err_gpio_reg_write;
		}

		led->gpio_cfg->enable = false;
	}

	qpnp_dump_regs(led, gpio_debug_regs, ARRAY_SIZE(gpio_debug_regs));

	return 0;

err_gpio_reg_write:
	led->gpio_cfg->enable = false;

	return rc;
}

static int qpnp_flash_regulator_operate(struct qpnp_led_data *led, bool on)
{
	int rc, i;
	struct qpnp_led_data *led_array;
	bool regulator_on = false;

	led_array = dev_get_drvdata(&led->spmi_dev->dev);
	if (!led_array) {
		dev_err(&led->spmi_dev->dev,
				"Unable to get LED array\n");
		return -EINVAL;
	}

	for (i = 0; i < led->num_leds; i++)
		regulator_on |= led_array[i].flash_cfg->flash_on;

	if (!on)
		goto regulator_turn_off;

	if (!regulator_on && !led->flash_cfg->flash_on) {
		for (i = 0; i < led->num_leds; i++) {
			if (led_array[i].flash_cfg->flash_reg_get) {
				rc = regulator_enable(
					led_array[i].flash_cfg->flash_wa_reg);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
						"Flash_wa regulator enable failed(%d)\n",
								rc);
					return rc;
				}

				rc = regulator_enable(
					led_array[i].flash_cfg->\
					flash_boost_reg);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
						"Regulator enable failed(%d)\n",
									rc);
					return rc;
				}
				led->flash_cfg->flash_on = true;
			}
			break;
		}
	}

	return 0;

regulator_turn_off:
	if (regulator_on && led->flash_cfg->flash_on) {
		for (i = 0; i < led->num_leds; i++) {
			if (led_array[i].flash_cfg->flash_reg_get) {
				rc = qpnp_led_masked_write(led,
					FLASH_ENABLE_CONTROL(led->base),
					FLASH_ENABLE_MASK,
					FLASH_DISABLE_ALL);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
						"Enable reg write failed(%d)\n",
						rc);
				}

				rc = regulator_disable(led_array[i].flash_cfg->\
							flash_boost_reg);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
						"Regulator disable failed(%d)\n",
									rc);
					return rc;
				}
				rc = regulator_disable(
					led_array[i].flash_cfg->flash_wa_reg);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
						"Flash_wa regulator disable failed(%d)\n",
								rc);
					return rc;
				}
				led->flash_cfg->flash_on = false;
			}
			break;
		}
	}

	return 0;
}

static int qpnp_torch_regulator_operate(struct qpnp_led_data *led, bool on)
{
	int rc;

	if (!on)
		goto regulator_turn_off;

	if (!led->flash_cfg->torch_on) {
		rc = regulator_enable(led->flash_cfg->torch_boost_reg);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"Regulator enable failed(%d)\n", rc);
				return rc;
		}
		led->flash_cfg->torch_on = true;
	}
	return 0;

regulator_turn_off:
	if (led->flash_cfg->torch_on) {
		rc = qpnp_led_masked_write(led,	FLASH_ENABLE_CONTROL(led->base),
				FLASH_ENABLE_MODULE_MASK, FLASH_DISABLE_ALL);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"Enable reg write failed(%d)\n", rc);
		}

		rc = regulator_disable(led->flash_cfg->torch_boost_reg);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"Regulator disable failed(%d)\n", rc);
			return rc;
		}
		led->flash_cfg->torch_on = false;
	}
	return 0;
}

static int qpnp_flash_set(struct qpnp_led_data *led)
{
	int rc, error;
	int val = led->cdev.brightness;

	if (led->flash_cfg->torch_enable)
		led->flash_cfg->current_prgm =
			(val * TORCH_MAX_LEVEL / led->max_current);
	else
		led->flash_cfg->current_prgm =
			(val * FLASH_MAX_LEVEL / led->max_current);

	
	if (val > 0) {
		if (led->flash_cfg->torch_enable) {
			if (led->flash_cfg->peripheral_subtype ==
							FLASH_SUBTYPE_DUAL) {
				if (!led->flash_cfg->no_smbb_support)
					rc = qpnp_torch_regulator_operate(led,
									true);
				else
					rc = qpnp_flash_regulator_operate(led,
									true);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
					"Torch regulator operate failed(%d)\n",
					rc);
					return rc;
				}
			} else if (led->flash_cfg->peripheral_subtype ==
							FLASH_SUBTYPE_SINGLE) {
				rc = qpnp_flash_regulator_operate(led, true);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
					"Flash regulator operate failed(%d)\n",
					rc);
					goto error_flash_set;
				}
			}

			qpnp_led_masked_write(led, FLASH_MAX_CURR(led->base),
				FLASH_CURRENT_MASK,
				TORCH_MAX_LEVEL);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Max current reg write failed(%d)\n",
					rc);
				goto error_reg_write;
			}

			qpnp_led_masked_write(led,
				FLASH_LED_TMR_CTRL(led->base),
				FLASH_TMR_MASK,
				FLASH_TMR_WATCHDOG);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Timer control reg write failed(%d)\n",
					rc);
				goto error_reg_write;
			}

			rc = qpnp_led_masked_write(led,
				led->flash_cfg->current_addr,
				FLASH_CURRENT_MASK,
				led->flash_cfg->current_prgm);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Current reg write failed(%d)\n", rc);
				goto error_reg_write;
			}

			rc = qpnp_led_masked_write(led,
				led->flash_cfg->second_addr,
				FLASH_CURRENT_MASK,
				led->flash_cfg->current_prgm);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"2nd Current reg write failed(%d)\n",
					rc);
				goto error_reg_write;
			}

			qpnp_led_masked_write(led,
				FLASH_WATCHDOG_TMR(led->base),
				FLASH_WATCHDOG_MASK,
				led->flash_cfg->duration);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Max current reg write failed(%d)\n",
					rc);
				goto error_reg_write;
			}

			rc = qpnp_led_masked_write(led,
				FLASH_ENABLE_CONTROL(led->base),
				FLASH_ENABLE_MASK,
				led->flash_cfg->enable_module);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Enable reg write failed(%d)\n",
					rc);
				goto error_reg_write;
			}

			if (!led->flash_cfg->strobe_type)
				led->flash_cfg->trigger_flash &=
						~FLASH_HW_SW_STROBE_SEL_MASK;
			else
				led->flash_cfg->trigger_flash |=
						FLASH_HW_SW_STROBE_SEL_MASK;

			rc = qpnp_led_masked_write(led,
				FLASH_LED_STROBE_CTRL(led->base),
				led->flash_cfg->trigger_flash,
				led->flash_cfg->trigger_flash);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"LED %d strobe reg write failed(%d)\n",
					led->id, rc);
				goto error_reg_write;
			}
		} else {
			rc = qpnp_flash_regulator_operate(led, true);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Flash regulator operate failed(%d)\n",
					rc);
				goto error_flash_set;
			}

			qpnp_led_masked_write(led,
				FLASH_LED_TMR_CTRL(led->base),
				FLASH_TMR_MASK,
				FLASH_TMR_SAFETY);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Timer control reg write failed(%d)\n",
					rc);
				goto error_reg_write;
			}

			
			rc = qpnp_led_masked_write(led,
				FLASH_SAFETY_TIMER(led->base),
				FLASH_SAFETY_TIMER_MASK,
				led->flash_cfg->duration);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Safety timer reg write failed(%d)\n",
					rc);
				goto error_flash_set;
			}

			
			rc = qpnp_led_masked_write(led,
				FLASH_MAX_CURR(led->base), FLASH_CURRENT_MASK,
				FLASH_MAX_LEVEL);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Max current reg write failed(%d)\n",
					rc);
				goto error_flash_set;
			}

			
			rc = qpnp_led_masked_write(led,
				FLASH_CLAMP_CURR(led->base),
				FLASH_CURRENT_MASK,
				led->flash_cfg->clamp_curr);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Clamp current reg write failed(%d)\n",
					rc);
				goto error_flash_set;
			}

			rc = qpnp_led_masked_write(led,
				led->flash_cfg->current_addr,
				FLASH_CURRENT_MASK,
				led->flash_cfg->current_prgm);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Current reg write failed(%d)\n", rc);
				goto error_flash_set;
			}

			rc = qpnp_led_masked_write(led,
				FLASH_ENABLE_CONTROL(led->base),
				led->flash_cfg->enable_module,
				led->flash_cfg->enable_module);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Enable reg write failed(%d)\n", rc);
				goto error_flash_set;
			}

			usleep(FLASH_RAMP_UP_DELAY_US);

			if (!led->flash_cfg->strobe_type)
				led->flash_cfg->trigger_flash &=
						~FLASH_HW_SW_STROBE_SEL_MASK;
			else
				led->flash_cfg->trigger_flash |=
						FLASH_HW_SW_STROBE_SEL_MASK;

			rc = qpnp_led_masked_write(led,
				FLASH_LED_STROBE_CTRL(led->base),
				led->flash_cfg->trigger_flash,
				led->flash_cfg->trigger_flash);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
				"LED %d strobe reg write failed(%d)\n",
				led->id, rc);
				goto error_flash_set;
			}
		}
	} else {
		rc = qpnp_led_masked_write(led,
			FLASH_LED_STROBE_CTRL(led->base),
			led->flash_cfg->trigger_flash,
			FLASH_DISABLE_ALL);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"LED %d flash write failed(%d)\n", led->id, rc);
			if (led->flash_cfg->torch_enable)
				goto error_torch_set;
			else
				goto error_flash_set;
		}

		if (led->flash_cfg->torch_enable) {
			if (led->flash_cfg->peripheral_subtype ==
							FLASH_SUBTYPE_DUAL) {
				if (!led->flash_cfg->no_smbb_support)
					rc = qpnp_torch_regulator_operate(led,
									false);
				else
					rc = qpnp_flash_regulator_operate(led,
									false);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
						"Torch regulator operate failed(%d)\n",
						rc);
					return rc;
				}
			} else if (led->flash_cfg->peripheral_subtype ==
							FLASH_SUBTYPE_SINGLE) {
				rc = qpnp_flash_regulator_operate(led, false);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
						"Flash regulator operate failed(%d)\n",
						rc);
					return rc;
				}
			}
		} else {
			usleep(FLASH_RAMP_DN_DELAY_US);

			rc = qpnp_led_masked_write(led,
				FLASH_ENABLE_CONTROL(led->base),
				led->flash_cfg->enable_module &
				~FLASH_ENABLE_MODULE_MASK,
				FLASH_DISABLE_ALL);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Enable reg write failed(%d)\n", rc);
				if (led->flash_cfg->torch_enable)
					goto error_torch_set;
				else
					goto error_flash_set;
			}

			rc = qpnp_flash_regulator_operate(led, false);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Flash regulator operate failed(%d)\n",
					rc);
				return rc;
			}
		}
	}

	qpnp_dump_regs(led, flash_debug_regs, ARRAY_SIZE(flash_debug_regs));

	return 0;

error_reg_write:
	if (led->flash_cfg->peripheral_subtype == FLASH_SUBTYPE_SINGLE)
		goto error_flash_set;

error_torch_set:
	if (!led->flash_cfg->no_smbb_support)
		error = qpnp_torch_regulator_operate(led, false);
	else
		error = qpnp_flash_regulator_operate(led, false);
	if (error) {
		dev_err(&led->spmi_dev->dev,
			"Torch regulator operate failed(%d)\n", rc);
		return error;
	}
	return rc;

error_flash_set:
	error = qpnp_flash_regulator_operate(led, false);
	if (error) {
		dev_err(&led->spmi_dev->dev,
			"Flash regulator operate failed(%d)\n", rc);
		return error;
	}
	return rc;
}

static int qpnp_kpdbl_set(struct qpnp_led_data *led)
{
	int rc;
	int duty_us, duty_ns, period_us;

	if (led->cdev.brightness) {
		if (!led->kpdbl_cfg->pwm_cfg->blinking)
			led->kpdbl_cfg->pwm_cfg->mode =
				led->kpdbl_cfg->pwm_cfg->default_mode;
		if (!num_kpbl_leds_on) {
			rc = qpnp_led_masked_write(led, KPDBL_ENABLE(led->base),
					KPDBL_MODULE_EN_MASK, KPDBL_MODULE_EN);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Enable reg write failed(%d)\n", rc);
				return rc;
			}
		}

		if (led->kpdbl_cfg->pwm_cfg->mode == PWM_MODE) {
			period_us = led->kpdbl_cfg->pwm_cfg->pwm_period_us;
			if (period_us > INT_MAX / NSEC_PER_USEC) {
				duty_us = (period_us * led->cdev.brightness) /
					KPDBL_MAX_LEVEL;
				rc = pwm_config_us(
					led->kpdbl_cfg->pwm_cfg->pwm_dev,
					duty_us,
					period_us);
			} else {
				duty_ns = ((period_us * NSEC_PER_USEC) /
					KPDBL_MAX_LEVEL) * led->cdev.brightness;
				rc = pwm_config(
					led->kpdbl_cfg->pwm_cfg->pwm_dev,
					duty_ns,
					period_us * NSEC_PER_USEC);
			}
			if (rc < 0) {
				dev_err(&led->spmi_dev->dev, "pwm config failed\n");
				return rc;
			}
		}

		rc = pwm_enable(led->kpdbl_cfg->pwm_cfg->pwm_dev);
		if (rc < 0) {
			dev_err(&led->spmi_dev->dev, "pwm enable failed\n");
			return rc;
		}

		num_kpbl_leds_on++;

	} else {
		led->kpdbl_cfg->pwm_cfg->mode =
			led->kpdbl_cfg->pwm_cfg->default_mode;

		if (led->kpdbl_cfg->always_on) {
			rc = pwm_config_us(led->kpdbl_cfg->pwm_cfg->pwm_dev, 0,
				led->kpdbl_cfg->pwm_cfg->pwm_period_us);
			if (rc < 0) {
				dev_err(&led->spmi_dev->dev,
						"pwm config failed\n");
				return rc;
			}

			rc = pwm_enable(led->kpdbl_cfg->pwm_cfg->pwm_dev);
			if (rc < 0) {
				dev_err(&led->spmi_dev->dev, "pwm enable failed\n");
				return rc;
			}
		} else
			pwm_disable(led->kpdbl_cfg->pwm_cfg->pwm_dev);

		if (num_kpbl_leds_on > 0)
			num_kpbl_leds_on--;

		if (!num_kpbl_leds_on) {
			rc = qpnp_led_masked_write(led, KPDBL_ENABLE(led->base),
					KPDBL_MODULE_EN_MASK, KPDBL_MODULE_DIS);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Failed to write led enable reg\n");
				return rc;
			}
		}
	}

	led->kpdbl_cfg->pwm_cfg->blinking = false;

	qpnp_dump_regs(led, kpdbl_debug_regs, ARRAY_SIZE(kpdbl_debug_regs));

	return 0;
}

static void led_fade_do_work(struct work_struct *work)
{
        struct qpnp_led_data *led;
        int rc;

        led = container_of(work, struct qpnp_led_data, fade_delayed_work.work);

        pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
        rc = qpnp_led_masked_write(led,RGB_LED_EN_CTL(led->base), led->rgb_cfg->enable, RGB_LED_DISABLE);
        if (rc) {
                dev_err(&led->spmi_dev->dev, "Failed to write led enable reg\n");
        }
}
static void led_blink_do_work(struct work_struct *work)
{
	struct qpnp_led_data *led;
	int rc,val;
	led = container_of(work, struct qpnp_led_data, blink_delayed_work.work);
	switch(led->id) {
		case QPNP_ID_LED_MPP:
			rc = pwm_config_us(led->mpp_cfg->pwm_cfg->pwm_dev, led->mpp_cfg->pwm_cfg->pwm_duty_us, led->mpp_cfg->pwm_cfg->pwm_period_us);
			rc = pwm_enable(led->mpp_cfg->pwm_cfg->pwm_dev);
			
			mdelay(10);
			rc = pwm_enable(led->mpp_cfg->pwm_cfg->pwm_dev);
			val = (led->mpp_cfg->source_sel & LED_MPP_SRC_MASK) |
				(led->mpp_cfg->mode_ctrl & LED_MPP_MODE_CTRL_MASK);

			rc = qpnp_led_masked_write(led,
				LED_MPP_MODE_CTRL(led->base), LED_MPP_MODE_MASK,
				val);
			rc = qpnp_led_masked_write(led,
					LED_MPP_EN_CTRL(led->base), LED_MPP_EN_MASK,
					LED_MPP_EN_ENABLE);
		break;
		case QPNP_ID_RGB_RED:
		case QPNP_ID_RGB_GREEN:
		case QPNP_ID_RGB_BLUE:
			rc = pwm_config_us(led->rgb_cfg->pwm_cfg->pwm_dev, led->rgb_cfg->pwm_cfg->pwm_duty_us, led->rgb_cfg->pwm_cfg->pwm_period_us);
			rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
			
			mdelay(10);
			rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
			led->status = BLINK;
		break;
		default:
		break;
	}
}
static int qpnp_rgb_set(struct qpnp_led_data *led)
{
	int rc;
	LED_INFO("%s, name:%s, brightness = %d status: %d\n", __func__, led->cdev.name, led->cdev.brightness, led->status);

	if (led->rgb_cfg->pwm_cfg->mode == RGB_MODE_LPG) {
		cancel_delayed_work(&led->fade_delayed_work);
	}
	if (led->cdev.brightness) {
		if (led->status != ON) {
			if (led->rgb_cfg->pwm_cfg->mode == RGB_MODE_PWM) {
				rc = pwm_config_us(led->rgb_cfg->pwm_cfg->pwm_dev, 640 * led->rgb_cfg->pwm_cfg->pwm_coefficient / 255, 640);
				if (rc < 0) {
					dev_err(&led->spmi_dev->dev, "Failed to " \
						"configure pwm for new values\n");
					return rc;
				}

			} else if (led->rgb_cfg->pwm_cfg->mode == RGB_MODE_LPG) {
							rc = pwm_lut_config(led->rgb_cfg->pwm_cfg->pwm_dev,
					PM_PWM_PERIOD_MIN,
									lut_on,
									led->rgb_cfg->pwm_cfg->lut_params);
			}

			rc = qpnp_led_masked_write(led,	RGB_LED_EN_CTL(led->base),
				led->rgb_cfg->enable, led->rgb_cfg->enable);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
					"Failed to write led enable reg\n");
				return rc;
			}
			rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
			
			mdelay(10);
			rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
			led->status = ON;
			if (led->cdev.brightness == 9) {
				msleep(60);
				led->rgb_cfg->pwm_cfg->mode = led->rgb_cfg->pwm_cfg->default_mode;
				pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
				rc = qpnp_led_masked_write(led, RGB_LED_EN_CTL(led->base), led->rgb_cfg->enable, RGB_LED_DISABLE);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
							"Failed to write led enable reg\n");
					return rc;
				}
				led->status = OFF;
			}
			if (led->cdev.brightness == 10) {
				msleep(490);
				led->rgb_cfg->pwm_cfg->mode = led->rgb_cfg->pwm_cfg->default_mode;
				pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
				rc = qpnp_led_masked_write(led, RGB_LED_EN_CTL(led->base), led->rgb_cfg->enable, RGB_LED_DISABLE);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
							"Failed to write led enable reg\n");
					return rc;
				}
				led->status = OFF;
			}

		}
	} else {
                if (led->rgb_cfg->pwm_cfg->mode == RGB_MODE_LPG) {
			wake_lock_timeout(&pmic_led_wake_lock, HZ*2);
                        rc = pwm_lut_config(led->rgb_cfg->pwm_cfg->pwm_dev,
				PM_PWM_PERIOD_MIN,
				lut_off,
                                led->rgb_cfg->pwm_cfg->lut_params);
                        if (rc < 0) {
                                dev_err(&led->spmi_dev->dev, "Failed to " \
                                        "configure pwm LUT\n");
                                return rc;
		}
		rc = qpnp_led_masked_write(led,
			RGB_LED_EN_CTL(led->base),
			led->rgb_cfg->enable, led->rgb_cfg->enable);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"Failed to write led enable reg\n");
			return rc;
		}
		rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
			queue_delayed_work(g_led_work_queue, &led->fade_delayed_work,
				msecs_to_jiffies(led->rgb_cfg->pwm_cfg->lut_params.ramp_step_ms * led->rgb_cfg->pwm_cfg->lut_params.idx_len));
	} else {
		led->rgb_cfg->pwm_cfg->mode =
			led->rgb_cfg->pwm_cfg->default_mode;
		pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
		rc = qpnp_led_masked_write(led,
			RGB_LED_EN_CTL(led->base),
			led->rgb_cfg->enable, RGB_LED_DISABLE);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"Failed to write led enable reg\n");
			return rc;
			}
		}
		led->status = OFF;
	}

	led->rgb_cfg->pwm_cfg->blinking = false;
	qpnp_dump_regs(led, rgb_pwm_debug_regs, ARRAY_SIZE(rgb_pwm_debug_regs));

	return 0;
}

static void qpnp_led_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	struct qpnp_led_data *led;
	bool ret;
	int retry_count = 0;
	led = container_of(led_cdev, struct qpnp_led_data, cdev);
	if (value < LED_OFF) {
		dev_err(&led->spmi_dev->dev, "Invalid brightness value\n");
		return;
	}

	if (value > led->cdev.max_brightness)
		value = led->cdev.max_brightness;

	led->cdev.brightness = value;
	do {
		ret = queue_work(g_led_on_work_queue, &led->work);
		if (!ret) {
			retry_count++;
			msleep(1);
			LED_INFO("%s qpnp_led_set work already on queue, requeue!! ret:%d retry:%d\n", __func__, ret, retry_count);
		}
	} while ((!ret) && (retry_count < 3));
}

static void __qpnp_led_work(struct qpnp_led_data *led,
				enum led_brightness value)
{
	int rc;

	if (led->id == QPNP_ID_FLASH1_LED0 || led->id == QPNP_ID_FLASH1_LED1)
		mutex_lock(&flash_lock);
	else
		mutex_lock(&led->lock);

	switch (led->id) {
	case QPNP_ID_WLED:
		rc = qpnp_wled_set(led);
		if (rc < 0)
			dev_err(&led->spmi_dev->dev,
				"WLED set brightness failed (%d)\n", rc);
		break;
	case QPNP_ID_FLASH1_LED0:
	case QPNP_ID_FLASH1_LED1:
		rc = qpnp_flash_set(led);
		if (rc < 0)
			dev_err(&led->spmi_dev->dev,
				"FLASH set brightness failed (%d)\n", rc);
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		rc = qpnp_rgb_set(led);
		if (rc < 0)
			dev_err(&led->spmi_dev->dev,
				"RGB set brightness failed (%d)\n", rc);
		break;
	case QPNP_ID_LED_MPP:
		rc = qpnp_mpp_set(led, 0);
		if (rc < 0)
			dev_err(&led->spmi_dev->dev,
					"MPP set brightness failed (%d)\n", rc);
		break;
	case QPNP_ID_LED_GPIO:
		rc = qpnp_gpio_set(led);
		if (rc < 0)
			dev_err(&led->spmi_dev->dev,
					"GPIO set brightness failed (%d)\n",
					rc);
		break;
	case QPNP_ID_KPDBL:
		rc = qpnp_kpdbl_set(led);
		if (rc < 0)
			dev_err(&led->spmi_dev->dev,
				"KPDBL set brightness failed (%d)\n", rc);
		break;
	default:
		dev_err(&led->spmi_dev->dev, "Invalid LED(%d)\n", led->id);
		break;
	}
	if (led->id == QPNP_ID_FLASH1_LED0 || led->id == QPNP_ID_FLASH1_LED1)
		mutex_unlock(&flash_lock);
	else
		mutex_unlock(&led->lock);

}

static void qpnp_led_work(struct work_struct *work)
{
	struct qpnp_led_data *led = container_of(work,
					struct qpnp_led_data, work);

	__qpnp_led_work(led, led->cdev.brightness);

	return;
}

static int qpnp_led_set_max_brightness(struct qpnp_led_data *led)
{
	switch (led->id) {
	case QPNP_ID_WLED:
		led->cdev.max_brightness = WLED_MAX_LEVEL;
		break;
	case QPNP_ID_FLASH1_LED0:
	case QPNP_ID_FLASH1_LED1:
		led->cdev.max_brightness = led->max_current;
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		led->cdev.max_brightness = RGB_MAX_LEVEL;
		break;
	case QPNP_ID_LED_MPP:
		if (led->mpp_cfg->pwm_mode == MANUAL_MODE)
			led->cdev.max_brightness = led->max_current;
		else
			led->cdev.max_brightness = MPP_MAX_LEVEL;
		break;
	case QPNP_ID_LED_GPIO:
			led->cdev.max_brightness = led->max_current;
		break;
	case QPNP_ID_KPDBL:
		led->cdev.max_brightness = KPDBL_MAX_LEVEL;
		break;
	default:
		dev_err(&led->spmi_dev->dev, "Invalid LED(%d)\n", led->id);
		return -EINVAL;
	}

	return 0;
}

static enum led_brightness qpnp_led_get(struct led_classdev *led_cdev)
{
	struct qpnp_led_data *led;

	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	return led->cdev.brightness;
}

static void qpnp_led_turn_off_delayed(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct qpnp_led_data *led
		= container_of(dwork, struct qpnp_led_data, dwork);

	led->cdev.brightness = LED_OFF;
	qpnp_led_set(&led->cdev, led->cdev.brightness);
}

static void qpnp_led_turn_off(struct qpnp_led_data *led)
{
	INIT_DELAYED_WORK(&led->dwork, qpnp_led_turn_off_delayed);
	schedule_delayed_work(&led->dwork,
		msecs_to_jiffies(led->turn_off_delay_ms));
}

static int qpnp_wled_init(struct qpnp_led_data *led)
{
	int rc, i;
	u8 num_wled_strings, val = 0;

	num_wled_strings = led->wled_cfg->num_strings;

	
	if (led->wled_cfg->ovp_val > WLED_OVP_27V) {
		dev_err(&led->spmi_dev->dev, "Invalid ovp value\n");
		return -EINVAL;
	}

	if (led->wled_cfg->boost_curr_lim > WLED_CURR_LIMIT_1680mA) {
		dev_err(&led->spmi_dev->dev, "Invalid boost current limit\n");
		return -EINVAL;
	}

	if (led->wled_cfg->cp_select > WLED_CP_SELECT_MAX) {
		dev_err(&led->spmi_dev->dev, "Invalid pole capacitance\n");
		return -EINVAL;
	}

	if ((led->max_current > WLED_MAX_CURR)) {
		dev_err(&led->spmi_dev->dev, "Invalid max current\n");
		return -EINVAL;
	}

	if ((led->wled_cfg->ctrl_delay_us % WLED_CTL_DLY_STEP) ||
		(led->wled_cfg->ctrl_delay_us > WLED_CTL_DLY_MAX)) {
		dev_err(&led->spmi_dev->dev, "Invalid control delay\n");
		return -EINVAL;
	}

	
	rc = qpnp_led_masked_write(led, WLED_OVP_CFG_REG(led->base),
		WLED_OVP_VAL_MASK,
		(led->wled_cfg->ovp_val << WLED_OVP_VAL_BIT_SHFT));
	if (rc) {
		dev_err(&led->spmi_dev->dev,
				"WLED OVP reg write failed(%d)\n", rc);
		return rc;
	}

	
	rc = qpnp_led_masked_write(led, WLED_BOOST_LIMIT_REG(led->base),
		WLED_BOOST_LIMIT_MASK, led->wled_cfg->boost_curr_lim);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
				"WLED boost limit reg write failed(%d)\n", rc);
		return rc;
	}

	
	rc = qpnp_led_masked_write(led, WLED_FDBCK_CTRL_REG(led->base),
		WLED_OP_FDBCK_MASK,
		(led->wled_cfg->op_fdbck << WLED_OP_FDBCK_BIT_SHFT));
	if (rc) {
		dev_err(&led->spmi_dev->dev,
				"WLED fdbck ctrl reg write failed(%d)\n", rc);
		return rc;
	}

	
	rc = qpnp_led_masked_write(led,
		WLED_SWITCHING_FREQ_REG(led->base),
		WLED_SWITCH_FREQ_MASK, led->wled_cfg->switch_freq);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"WLED switch freq reg write failed(%d)\n", rc);
		return rc;
	}

	
	if (led->wled_cfg->cs_out_en) {
		for (i = 0; i < led->wled_cfg->num_strings; i++)
			val |= 1 << i;
		rc = qpnp_led_masked_write(led, WLED_CURR_SINK_REG(led->base),
			WLED_CURR_SINK_MASK, (val << WLED_CURR_SINK_SHFT));
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"WLED curr sink reg write failed(%d)\n", rc);
			return rc;
		}
	}

	
	rc = qpnp_led_masked_write(led, WLED_HIGH_POLE_CAP_REG(led->base),
		WLED_CP_SELECT_MASK, led->wled_cfg->cp_select);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
				"WLED pole cap reg write failed(%d)\n", rc);
		return rc;
	}

	
	for (i = 0; i < num_wled_strings; i++) {
		rc = qpnp_led_masked_write(led, WLED_MOD_EN_REG(led->base, i),
			WLED_NO_MASK, WLED_EN_MASK);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"WLED mod enable reg write failed(%d)\n", rc);
			return rc;
		}

		if (led->wled_cfg->dig_mod_gen_en) {
			rc = qpnp_led_masked_write(led,
				WLED_MOD_SRC_SEL_REG(led->base, i),
				WLED_NO_MASK, WLED_USE_EXT_GEN_MOD_SRC);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
				"WLED dig mod en reg write failed(%d)\n", rc);
			}
		}

		rc = qpnp_led_masked_write(led,
			WLED_FULL_SCALE_REG(led->base, i), WLED_MAX_CURR_MASK,
			(u8)led->max_current);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"WLED max current reg write failed(%d)\n", rc);
			return rc;
		}

	}

	
	rc = qpnp_led_masked_write(led, WLED_MOD_CTRL_REG(led->base),
		WLED_8_BIT_MASK, WLED_BOOST_OFF);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"WLED write ctrl reg failed(%d)\n", rc);
		return rc;
	}

	
	qpnp_dump_regs(led, wled_debug_regs, ARRAY_SIZE(wled_debug_regs));

	return 0;
}

static ssize_t led_mode_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	unsigned long state;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &state);
	if (ret)
		return ret;

	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	
	if (state == 1)
		led->flash_cfg->torch_enable = true;
	else
		led->flash_cfg->torch_enable = false;

	return count;
}

static ssize_t led_strobe_type_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	unsigned long state;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &state);
	if (ret)
		return ret;

	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	
	if (state == 1)
		led->flash_cfg->strobe_type = 1;
	else
		led->flash_cfg->strobe_type = 0;

	return count;
}

static int qpnp_pwm_init(struct pwm_config_data *pwm_cfg,
					struct spmi_device *spmi_dev,
					const char *name)
{
	int rc, start_idx, idx_len;

	if (pwm_cfg->pwm_channel != -1) {
		pwm_cfg->pwm_dev =
			pwm_request(pwm_cfg->pwm_channel, name);

		if (IS_ERR_OR_NULL(pwm_cfg->pwm_dev)) {
			dev_err(&spmi_dev->dev,
				"could not acquire PWM Channel %d, " \
				"error %ld\n",
				pwm_cfg->pwm_channel,
				PTR_ERR(pwm_cfg->pwm_dev));
			pwm_cfg->pwm_dev = NULL;
			return -ENODEV;
		}

			if (pwm_cfg->mode == LPG_MODE || pwm_cfg->use_blink) {
			start_idx =
			pwm_cfg->duty_cycles->start_idx;
			idx_len =
			pwm_cfg->duty_cycles->num_duty_pcts;

			if (idx_len >= PWM_LUT_MAX_SIZE &&
					start_idx) {
				dev_err(&spmi_dev->dev,
					"Wrong LUT size or index\n");
				return -EINVAL;
			}
			if ((start_idx + idx_len) >
					PWM_LUT_MAX_SIZE) {
				dev_err(&spmi_dev->dev,
					"Exceed LUT limit\n");
				return -EINVAL;
			}
			rc = pwm_lut_config(pwm_cfg->pwm_dev,
				pwm_cfg->pwm_period_us,
				pwm_cfg->duty_cycles->duty_pcts,
				pwm_cfg->lut_params);
			if (rc < 0) {
				dev_err(&spmi_dev->dev, "Failed to " \
					"configure pwm LUT\n");
				return rc;
			}
		}
	} else {
		dev_err(&spmi_dev->dev,
			"Invalid PWM channel\n");
		return -EINVAL;
	}

	return 0;
}

static ssize_t pwm_us_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	u32 pwm_us;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret;
	u32 previous_pwm_us;
	struct pwm_config_data *pwm_cfg;

	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	ret = kstrtou32(buf, 10, &pwm_us);
	if (ret)
		return ret;

	switch (led->id) {
	case QPNP_ID_LED_MPP:
		pwm_cfg = led->mpp_cfg->pwm_cfg;
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		pwm_cfg = led->rgb_cfg->pwm_cfg;
		break;
	default:
		dev_err(&led->spmi_dev->dev,
			"Invalid LED id type for pwm_us\n");
		return -EINVAL;
	}

	if (pwm_cfg->mode == LPG_MODE)
		pwm_cfg->blinking = true;

	previous_pwm_us = pwm_cfg->pwm_period_us;

	pwm_cfg->pwm_period_us = pwm_us;
	pwm_free(pwm_cfg->pwm_dev);
	ret = qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
	if (ret) {
		pwm_cfg->pwm_period_us = previous_pwm_us;
		pwm_free(pwm_cfg->pwm_dev);
		qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
		qpnp_led_set(&led->cdev, led->cdev.brightness);
		dev_err(&led->spmi_dev->dev,
			"Failed to initialize pwm with new pwm_us value\n");
		return ret;
	}
	qpnp_led_set(&led->cdev, led->cdev.brightness);
	return count;
}

static ssize_t pause_lo_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	u32 pause_lo;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret;
	u32 previous_pause_lo;
	struct pwm_config_data *pwm_cfg;

	ret = kstrtou32(buf, 10, &pause_lo);
	if (ret)
		return ret;
	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	switch (led->id) {
	case QPNP_ID_LED_MPP:
		pwm_cfg = led->mpp_cfg->pwm_cfg;
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		pwm_cfg = led->rgb_cfg->pwm_cfg;
		break;
	default:
		dev_err(&led->spmi_dev->dev,
			"Invalid LED id type for pause lo\n");
		return -EINVAL;
	}

	if (pwm_cfg->mode == LPG_MODE)
		pwm_cfg->blinking = true;

	previous_pause_lo = pwm_cfg->lut_params.lut_pause_lo;

	pwm_free(pwm_cfg->pwm_dev);
	pwm_cfg->lut_params.lut_pause_lo = pause_lo;
	ret = qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
	if (ret) {
		pwm_cfg->lut_params.lut_pause_lo = previous_pause_lo;
		pwm_free(pwm_cfg->pwm_dev);
		qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
		qpnp_led_set(&led->cdev, led->cdev.brightness);
		dev_err(&led->spmi_dev->dev,
			"Failed to initialize pwm with new pause lo value\n");
		return ret;
	}
	qpnp_led_set(&led->cdev, led->cdev.brightness);
	return count;
}

static ssize_t pause_hi_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	u32 pause_hi;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret;
	u32 previous_pause_hi;
	struct pwm_config_data *pwm_cfg;

	ret = kstrtou32(buf, 10, &pause_hi);
	if (ret)
		return ret;
	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	switch (led->id) {
	case QPNP_ID_LED_MPP:
		pwm_cfg = led->mpp_cfg->pwm_cfg;
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		pwm_cfg = led->rgb_cfg->pwm_cfg;
		break;
	default:
		dev_err(&led->spmi_dev->dev,
			"Invalid LED id type for pause hi\n");
		return -EINVAL;
	}

	if (pwm_cfg->mode == LPG_MODE)
		pwm_cfg->blinking = true;

	previous_pause_hi = pwm_cfg->lut_params.lut_pause_hi;

	pwm_free(pwm_cfg->pwm_dev);
	pwm_cfg->lut_params.lut_pause_hi = pause_hi;
	ret = qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
	if (ret) {
		pwm_cfg->lut_params.lut_pause_hi = previous_pause_hi;
		pwm_free(pwm_cfg->pwm_dev);
		qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
		qpnp_led_set(&led->cdev, led->cdev.brightness);
		dev_err(&led->spmi_dev->dev,
			"Failed to initialize pwm with new pause hi value\n");
		return ret;
	}
	qpnp_led_set(&led->cdev, led->cdev.brightness);
	return count;
}

static ssize_t start_idx_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	u32 start_idx;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret;
	u32 previous_start_idx;
	struct pwm_config_data *pwm_cfg;

	ret = kstrtou32(buf, 10, &start_idx);
	if (ret)
		return ret;
	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	switch (led->id) {
	case QPNP_ID_LED_MPP:
		pwm_cfg = led->mpp_cfg->pwm_cfg;
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		pwm_cfg = led->rgb_cfg->pwm_cfg;
		break;
	default:
		dev_err(&led->spmi_dev->dev,
			"Invalid LED id type for start idx\n");
		return -EINVAL;
	}

	if (pwm_cfg->mode == LPG_MODE)
		pwm_cfg->blinking = true;

	previous_start_idx = pwm_cfg->duty_cycles->start_idx;
	pwm_cfg->duty_cycles->start_idx = start_idx;
	pwm_cfg->lut_params.start_idx = pwm_cfg->duty_cycles->start_idx;
	pwm_free(pwm_cfg->pwm_dev);
	ret = qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
	if (ret) {
		pwm_cfg->duty_cycles->start_idx = previous_start_idx;
		pwm_cfg->lut_params.start_idx = pwm_cfg->duty_cycles->start_idx;
		pwm_free(pwm_cfg->pwm_dev);
		qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
		qpnp_led_set(&led->cdev, led->cdev.brightness);
		dev_err(&led->spmi_dev->dev,
			"Failed to initialize pwm with new start idx value\n");
		return ret;
	}
	qpnp_led_set(&led->cdev, led->cdev.brightness);
	return count;
}

static ssize_t ramp_step_ms_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	u32 ramp_step_ms;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret;
	u32 previous_ramp_step_ms;
	struct pwm_config_data *pwm_cfg;

	ret = kstrtou32(buf, 10, &ramp_step_ms);
	if (ret)
		return ret;
	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	switch (led->id) {
	case QPNP_ID_LED_MPP:
		pwm_cfg = led->mpp_cfg->pwm_cfg;
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		pwm_cfg = led->rgb_cfg->pwm_cfg;
		break;
	default:
		dev_err(&led->spmi_dev->dev,
			"Invalid LED id type for ramp step\n");
		return -EINVAL;
	}

	if (pwm_cfg->mode == LPG_MODE)
		pwm_cfg->blinking = true;

	previous_ramp_step_ms = pwm_cfg->lut_params.ramp_step_ms;

	pwm_free(pwm_cfg->pwm_dev);
	pwm_cfg->lut_params.ramp_step_ms = ramp_step_ms;
	ret = qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
	if (ret) {
		pwm_cfg->lut_params.ramp_step_ms = previous_ramp_step_ms;
		pwm_free(pwm_cfg->pwm_dev);
		qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
		qpnp_led_set(&led->cdev, led->cdev.brightness);
		dev_err(&led->spmi_dev->dev,
			"Failed to initialize pwm with new ramp step value\n");
		return ret;
	}
	qpnp_led_set(&led->cdev, led->cdev.brightness);
	return count;
}

static ssize_t lut_flags_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	u32 lut_flags;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret;
	u32 previous_lut_flags;
	struct pwm_config_data *pwm_cfg;

	ret = kstrtou32(buf, 10, &lut_flags);
	if (ret)
		return ret;
	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	switch (led->id) {
	case QPNP_ID_LED_MPP:
		pwm_cfg = led->mpp_cfg->pwm_cfg;
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		pwm_cfg = led->rgb_cfg->pwm_cfg;
		break;
	default:
		dev_err(&led->spmi_dev->dev,
			"Invalid LED id type for lut flags\n");
		return -EINVAL;
	}

	if (pwm_cfg->mode == LPG_MODE)
		pwm_cfg->blinking = true;

	previous_lut_flags = pwm_cfg->lut_params.flags;

	pwm_free(pwm_cfg->pwm_dev);
	pwm_cfg->lut_params.flags = lut_flags;
	ret = qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
	if (ret) {
		pwm_cfg->lut_params.flags = previous_lut_flags;
		pwm_free(pwm_cfg->pwm_dev);
		qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
		qpnp_led_set(&led->cdev, led->cdev.brightness);
		dev_err(&led->spmi_dev->dev,
			"Failed to initialize pwm with new lut flags value\n");
		return ret;
	}
	qpnp_led_set(&led->cdev, led->cdev.brightness);
	return count;
}

static ssize_t duty_pcts_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	int num_duty_pcts = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	char *buffer;
	ssize_t ret;
	int i = 0;
	int max_duty_pcts;
	struct pwm_config_data *pwm_cfg;
	u32 previous_num_duty_pcts;
	int value;
	int *previous_duty_pcts;

	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	switch (led->id) {
	case QPNP_ID_LED_MPP:
		pwm_cfg = led->mpp_cfg->pwm_cfg;
		max_duty_pcts = PWM_LUT_MAX_SIZE;
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		pwm_cfg = led->rgb_cfg->pwm_cfg;
		max_duty_pcts = PWM_LUT_MAX_SIZE;
		break;
	default:
		dev_err(&led->spmi_dev->dev,
			"Invalid LED id type for duty pcts\n");
		return -EINVAL;
	}

	if (pwm_cfg->mode == LPG_MODE)
		pwm_cfg->blinking = true;

	buffer = (char *)buf;

	for (i = 0; i < max_duty_pcts; i++) {
		if (buffer == NULL)
			break;
		ret = sscanf((const char *)buffer, "%u,%s", &value, buffer);
		pwm_cfg->old_duty_pcts[i] = value;
		num_duty_pcts++;
		if (ret <= 1)
			break;
	}

	if (num_duty_pcts >= max_duty_pcts) {
		dev_err(&led->spmi_dev->dev,
			"Number of duty pcts given exceeds max (%d)\n",
			max_duty_pcts);
		return -EINVAL;
	}

	previous_num_duty_pcts = pwm_cfg->duty_cycles->num_duty_pcts;
	previous_duty_pcts = pwm_cfg->duty_cycles->duty_pcts;

	pwm_cfg->duty_cycles->num_duty_pcts = num_duty_pcts;
	pwm_cfg->duty_cycles->duty_pcts = pwm_cfg->old_duty_pcts;
	pwm_cfg->old_duty_pcts = previous_duty_pcts;
	pwm_cfg->lut_params.idx_len = pwm_cfg->duty_cycles->num_duty_pcts;

	pwm_free(pwm_cfg->pwm_dev);
	ret = qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
	if (ret)
		goto restore;

	qpnp_led_set(&led->cdev, led->cdev.brightness);
	return count;

restore:
	dev_err(&led->spmi_dev->dev,
		"Failed to initialize pwm with new duty pcts value\n");
	pwm_cfg->duty_cycles->num_duty_pcts = previous_num_duty_pcts;
	pwm_cfg->old_duty_pcts = pwm_cfg->duty_cycles->duty_pcts;
	pwm_cfg->duty_cycles->duty_pcts = previous_duty_pcts;
	pwm_cfg->lut_params.idx_len = pwm_cfg->duty_cycles->num_duty_pcts;
	pwm_free(pwm_cfg->pwm_dev);
	qpnp_pwm_init(pwm_cfg, led->spmi_dev, led->cdev.name);
	qpnp_led_set(&led->cdev, led->cdev.brightness);
	return ret;
}

static void mpp_blink(struct qpnp_led_data *led,
					struct pwm_config_data *pwm_cfg)
{
	if (led->cdev.brightness) {
		pwm_cfg->blinking = true;
		if (led->id == QPNP_ID_LED_MPP)
			led->mpp_cfg->pwm_mode = LPG_MODE;
		pwm_cfg->mode = LPG_MODE;
	} else {
		pwm_cfg->blinking = false;
		pwm_cfg->mode = pwm_cfg->default_mode;
		if (led->id == QPNP_ID_LED_MPP)
			led->mpp_cfg->pwm_mode = pwm_cfg->default_mode;
	}
	
	
	
	qpnp_led_set(&led->cdev, led->cdev.brightness);
}

static ssize_t blink_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	unsigned long blinking;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;

	ret = kstrtoul(buf, 10, &blinking);
	if (ret)
		return ret;
	led = container_of(led_cdev, struct qpnp_led_data, cdev);
	led->cdev.brightness = blinking ? led->cdev.max_brightness : 0;

	switch (led->id) {
	case QPNP_ID_LED_MPP:
			mpp_blink(led, led->mpp_cfg->pwm_cfg);
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
			mpp_blink(led, led->rgb_cfg->pwm_cfg);
		break;
	default:
		dev_err(&led->spmi_dev->dev, "Invalid LED id type for blink\n");
		return -EINVAL;
	}
	return count;
}

static DEVICE_ATTR(led_mode, 0664, NULL, led_mode_store);
static DEVICE_ATTR(strobe, 0664, NULL, led_strobe_type_store);
static DEVICE_ATTR(pwm_us, 0664, NULL, pwm_us_store);
static DEVICE_ATTR(pause_lo, 0664, NULL, pause_lo_store);
static DEVICE_ATTR(pause_hi, 0664, NULL, pause_hi_store);
static DEVICE_ATTR(start_idx, 0664, NULL, start_idx_store);
static DEVICE_ATTR(ramp_step_ms, 0664, NULL, ramp_step_ms_store);
static DEVICE_ATTR(lut_flags, 0664, NULL, lut_flags_store);
static DEVICE_ATTR(duty_pcts, 0664, NULL, duty_pcts_store);
static DEVICE_ATTR(qcom_blink, 0664, NULL, blink_store);

static struct attribute *led_attrs[] = {
	&dev_attr_led_mode.attr,
	&dev_attr_strobe.attr,
	NULL
};

static const struct attribute_group led_attr_group = {
	.attrs = led_attrs,
};

static struct attribute *pwm_attrs[] = {
	&dev_attr_pwm_us.attr,
	NULL
};

static struct attribute *lpg_attrs[] = {
	&dev_attr_pause_lo.attr,
	&dev_attr_pause_hi.attr,
	&dev_attr_start_idx.attr,
	&dev_attr_ramp_step_ms.attr,
	&dev_attr_lut_flags.attr,
	&dev_attr_duty_pcts.attr,
	NULL
};

static struct attribute *blink_attrs[] = {
	&dev_attr_qcom_blink.attr,
	NULL
};

static const struct attribute_group pwm_attr_group = {
	.attrs = pwm_attrs,
};

static const struct attribute_group lpg_attr_group = {
	.attrs = lpg_attrs,
};

static const struct attribute_group blink_attr_group = {
	.attrs = blink_attrs,
};

static int qpnp_flash_init(struct qpnp_led_data *led)
{
	int rc;

	led->flash_cfg->flash_on = false;

	rc = qpnp_led_masked_write(led,
		FLASH_LED_STROBE_CTRL(led->base),
		FLASH_STROBE_MASK, FLASH_DISABLE_ALL);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"LED %d flash write failed(%d)\n", led->id, rc);
		return rc;
	}

	
	rc = qpnp_led_masked_write(led, FLASH_ENABLE_CONTROL(led->base),
		FLASH_ENABLE_MASK, FLASH_DISABLE_ALL);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Enable reg write failed(%d)\n", rc);
		return rc;
	}

	if (led->flash_cfg->torch_enable)
		return 0;

	
	rc = qpnp_led_masked_write(led, FLASH_HEADROOM(led->base),
		FLASH_HEADROOM_MASK, led->flash_cfg->headroom);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Headroom reg write failed(%d)\n", rc);
		return rc;
	}

	
	rc = qpnp_led_masked_write(led,
		FLASH_STARTUP_DELAY(led->base), FLASH_STARTUP_DLY_MASK,
		led->flash_cfg->startup_dly);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Startup delay reg write failed(%d)\n", rc);
		return rc;
	}

	
	if (led->flash_cfg->safety_timer) {
		rc = qpnp_led_masked_write(led,
			FLASH_LED_TMR_CTRL(led->base),
			FLASH_TMR_MASK, FLASH_TMR_SAFETY);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"LED timer ctrl reg write failed(%d)\n",
				rc);
			return rc;
		}
	}

	
	if (led->flash_cfg->vreg_ok)
		rc = qpnp_led_masked_write(led,	FLASH_VREG_OK_FORCE(led->base),
			FLASH_VREG_MASK, FLASH_SW_VREG_OK);
	else
		rc = qpnp_led_masked_write(led, FLASH_VREG_OK_FORCE(led->base),
			FLASH_VREG_MASK, FLASH_HW_VREG_OK);

	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Vreg OK reg write failed(%d)\n", rc);
		return rc;
	}

	
	rc = qpnp_led_masked_write(led, FLASH_FAULT_DETECT(led->base),
		FLASH_FAULT_DETECT_MASK, FLASH_SELFCHECK_ENABLE);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Fault detect reg write failed(%d)\n", rc);
		return rc;
	}

	
	rc = qpnp_led_masked_write(led, FLASH_MASK_ENABLE(led->base),
		FLASH_MASK_REG_MASK, FLASH_MASK_1);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Mask enable reg write failed(%d)\n", rc);
		return rc;
	}

	
	rc = qpnp_led_masked_write(led, FLASH_CURRENT_RAMP(led->base),
		FLASH_CURRENT_RAMP_MASK, FLASH_RAMP_STEP_27US);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Current ramp reg write failed(%d)\n", rc);
		return rc;
	}

	led->flash_cfg->strobe_type = 0;

	
	qpnp_dump_regs(led, flash_debug_regs, ARRAY_SIZE(flash_debug_regs));

	return 0;
}

static int qpnp_kpdbl_init(struct qpnp_led_data *led)
{
	int rc;
	u8 val;

	
	rc = spmi_ext_register_readl(led->spmi_dev->ctrl, led->spmi_dev->sid,
				KPDBL_ROW_SRC_SEL(led->base), &val, 1);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Unable to read from addr=%x, rc(%d)\n",
			KPDBL_ROW_SRC_SEL(led->base), rc);
		return rc;
	}

	if (led->kpdbl_cfg->row_src_vbst)
		val |= 1 << led->kpdbl_cfg->row_id;
	else
		val &= ~(1 << led->kpdbl_cfg->row_id);

	rc = spmi_ext_register_writel(led->spmi_dev->ctrl, led->spmi_dev->sid,
				KPDBL_ROW_SRC_SEL(led->base), &val, 1);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Unable to read from addr=%x, rc(%d)\n",
			KPDBL_ROW_SRC_SEL(led->base), rc);
		return rc;
	}

	
	rc = spmi_ext_register_readl(led->spmi_dev->ctrl, led->spmi_dev->sid,
				KPDBL_ROW_SRC(led->base), &val, 1);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Unable to read from addr=%x, rc(%d)\n",
			KPDBL_ROW_SRC(led->base), rc);
		return rc;
	}

	if (led->kpdbl_cfg->row_src_en)
		val |= KPDBL_ROW_SCAN_EN_MASK | (1 << led->kpdbl_cfg->row_id);
	else
		val &= ~(1 << led->kpdbl_cfg->row_id);

	rc = spmi_ext_register_writel(led->spmi_dev->ctrl, led->spmi_dev->sid,
		KPDBL_ROW_SRC(led->base), &val, 1);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Unable to write to addr=%x, rc(%d)\n",
			KPDBL_ROW_SRC(led->base), rc);
		return rc;
	}

	
	rc = qpnp_led_masked_write(led, KPDBL_ENABLE(led->base),
		KPDBL_MODULE_EN_MASK, KPDBL_MODULE_EN);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Enable module write failed(%d)\n", rc);
		return rc;
	}

	rc = qpnp_pwm_init(led->kpdbl_cfg->pwm_cfg, led->spmi_dev,
				led->cdev.name);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Failed to initialize pwm\n");
		return rc;
	}

	
	qpnp_dump_regs(led, kpdbl_debug_regs, ARRAY_SIZE(kpdbl_debug_regs));

	return 0;
}

static int qpnp_rgb_init(struct qpnp_led_data *led)
{
	int rc;

	rc = qpnp_led_masked_write(led, RGB_LED_SRC_SEL(led->base),
		RGB_LED_SRC_MASK, RGB_LED_SOURCE_VPH_PWR);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Failed to write led source select register\n");
		return rc;
	}

	rc = qpnp_pwm_init(led->rgb_cfg->pwm_cfg, led->spmi_dev,
				led->cdev.name);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Failed to initialize pwm\n");
		return rc;
	}
	
	rc = qpnp_led_masked_write(led, RGB_LED_ATC_CTL(led->base),
		led->rgb_cfg->enable, led->rgb_cfg->enable);

	return 0;
}

static int qpnp_mpp_init(struct qpnp_led_data *led)
{
	int rc;
	u8 val;


	if (led->max_current < LED_MPP_CURRENT_MIN ||
		led->max_current > LED_MPP_CURRENT_MAX) {
		dev_err(&led->spmi_dev->dev,
			"max current for mpp is not valid\n");
		return -EINVAL;
	}

	val = (led->mpp_cfg->current_setting / LED_MPP_CURRENT_PER_SETTING) - 1;

	if (val < 0)
		val = 0;

	rc = qpnp_led_masked_write(led, LED_MPP_VIN_CTRL(led->base),
		LED_MPP_VIN_MASK, led->mpp_cfg->vin_ctrl);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Failed to write led vin control reg\n");
		return rc;
	}

	rc = qpnp_led_masked_write(led, LED_MPP_SINK_CTRL(led->base),
		LED_MPP_SINK_MASK, val);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Failed to write sink control reg\n");
		return rc;
	}

	if (led->mpp_cfg->pwm_mode != MANUAL_MODE) {
		rc = qpnp_pwm_init(led->mpp_cfg->pwm_cfg, led->spmi_dev,
					led->cdev.name);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"Failed to initialize pwm\n");
			return rc;
		}
	}

	return 0;
}

static int qpnp_gpio_init(struct qpnp_led_data *led)
{
	int rc;

	rc = qpnp_led_masked_write(led, LED_GPIO_VIN_CTRL(led->base),
		LED_GPIO_VIN_MASK, led->gpio_cfg->vin_ctrl);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Failed to write led vin control reg\n");
		return rc;
	}

	return 0;
}

static int qpnp_led_initialize(struct qpnp_led_data *led)
{
	int rc = 0;

	switch (led->id) {
	case QPNP_ID_WLED:
		rc = qpnp_wled_init(led);
		if (rc)
			dev_err(&led->spmi_dev->dev,
				"WLED initialize failed(%d)\n", rc);
		break;
	case QPNP_ID_FLASH1_LED0:
	case QPNP_ID_FLASH1_LED1:
		rc = qpnp_flash_init(led);
		if (rc)
			dev_err(&led->spmi_dev->dev,
				"FLASH initialize failed(%d)\n", rc);
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		rc = qpnp_rgb_init(led);
		if (rc)
			dev_err(&led->spmi_dev->dev,
				"RGB initialize failed(%d)\n", rc);
		break;
	case QPNP_ID_LED_MPP:
		rc = qpnp_mpp_init(led);
		if (rc)
			dev_err(&led->spmi_dev->dev,
				"MPP initialize failed(%d)\n", rc);
		break;
	case QPNP_ID_LED_GPIO:
		rc = qpnp_gpio_init(led);
		if (rc)
			dev_err(&led->spmi_dev->dev,
				"GPIO initialize failed(%d)\n", rc);
		break;
	case QPNP_ID_KPDBL:
		rc = qpnp_kpdbl_init(led);
		if (rc)
			dev_err(&led->spmi_dev->dev,
				"KPDBL initialize failed(%d)\n", rc);
		break;
	default:
		dev_err(&led->spmi_dev->dev, "Invalid LED(%d)\n", led->id);
		return -EINVAL;
	}

	return rc;
}

static int qpnp_get_common_configs(struct qpnp_led_data *led,
				struct device_node *node)
{
	int rc;
	u32 val;
	const char *temp_string;

	led->cdev.default_trigger = LED_TRIGGER_DEFAULT;
	rc = of_property_read_string(node, "linux,default-trigger",
		&temp_string);
	if (!rc)
		led->cdev.default_trigger = temp_string;
	else if (rc != -EINVAL)
		return rc;

	led->default_on = false;
	rc = of_property_read_string(node, "qcom,default-state",
		&temp_string);
	if (!rc) {
		if (strncmp(temp_string, "on", sizeof("on")) == 0)
			led->default_on = true;
	} else if (rc != -EINVAL)
		return rc;

	led->turn_off_delay_ms = 0;
	rc = of_property_read_u32(node, "qcom,turn-off-delay-ms", &val);
	if (!rc)
		led->turn_off_delay_ms = val;
	else if (rc != -EINVAL)
		return rc;

	return 0;
}

static int qpnp_get_config_wled(struct qpnp_led_data *led,
				struct device_node *node)
{
	u32 val;
	int rc;

	led->wled_cfg = devm_kzalloc(&led->spmi_dev->dev,
				sizeof(struct wled_config_data), GFP_KERNEL);
	if (!led->wled_cfg) {
		dev_err(&led->spmi_dev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	rc = spmi_ext_register_readl(led->spmi_dev->ctrl, led->spmi_dev->sid,
		PMIC_VERSION_REG, &led->wled_cfg->pmic_version, 1);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Unable to read pmic ver, rc(%d)\n", rc);
	}

	led->wled_cfg->num_strings = WLED_DEFAULT_STRINGS;
	rc = of_property_read_u32(node, "qcom,num-strings", &val);
	if (!rc)
		led->wled_cfg->num_strings = (u8) val;
	else if (rc != -EINVAL)
		return rc;

	led->wled_cfg->num_physical_strings = led->wled_cfg->num_strings;
	rc = of_property_read_u32(node, "qcom,num-physical-strings", &val);
	if (!rc)
		led->wled_cfg->num_physical_strings = (u8) val;
	else if (rc != -EINVAL)
		return rc;

	led->wled_cfg->ovp_val = WLED_DEFAULT_OVP_VAL;
	rc = of_property_read_u32(node, "qcom,ovp-val", &val);
	if (!rc)
		led->wled_cfg->ovp_val = (u8) val;
	else if (rc != -EINVAL)
		return rc;

	led->wled_cfg->boost_curr_lim = WLED_BOOST_LIM_DEFAULT;
	rc = of_property_read_u32(node, "qcom,boost-curr-lim", &val);
	if (!rc)
		led->wled_cfg->boost_curr_lim = (u8) val;
	else if (rc != -EINVAL)
		return rc;

	led->wled_cfg->cp_select = WLED_CP_SEL_DEFAULT;
	rc = of_property_read_u32(node, "qcom,cp-sel", &val);
	if (!rc)
		led->wled_cfg->cp_select = (u8) val;
	else if (rc != -EINVAL)
		return rc;

	led->wled_cfg->ctrl_delay_us = WLED_CTRL_DLY_DEFAULT;
	rc = of_property_read_u32(node, "qcom,ctrl-delay-us", &val);
	if (!rc)
		led->wled_cfg->ctrl_delay_us = (u8) val;
	else if (rc != -EINVAL)
		return rc;

	led->wled_cfg->op_fdbck = WLED_OP_FDBCK_DEFAULT;
	rc = of_property_read_u32(node, "qcom,op-fdbck", &val);
	if (!rc)
		led->wled_cfg->op_fdbck = (u8) val;
	else if (rc != -EINVAL)
		return rc;

	led->wled_cfg->switch_freq = WLED_SWITCH_FREQ_DEFAULT;
	rc = of_property_read_u32(node, "qcom,switch-freq", &val);
	if (!rc)
		led->wled_cfg->switch_freq = (u8) val;
	else if (rc != -EINVAL)
		return rc;

	led->wled_cfg->dig_mod_gen_en =
		of_property_read_bool(node, "qcom,dig-mod-gen-en");

	led->wled_cfg->cs_out_en =
		of_property_read_bool(node, "qcom,cs-out-en");

	return 0;
}

static int qpnp_get_config_flash(struct qpnp_led_data *led,
				struct device_node *node, bool *reg_set)
{
	int rc;
	u32 val;

	led->flash_cfg = devm_kzalloc(&led->spmi_dev->dev,
				sizeof(struct flash_config_data), GFP_KERNEL);
	if (!led->flash_cfg) {
		dev_err(&led->spmi_dev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	rc = spmi_ext_register_readl(led->spmi_dev->ctrl, led->spmi_dev->sid,
			FLASH_PERIPHERAL_SUBTYPE(led->base),
			&led->flash_cfg->peripheral_subtype, 1);
	if (rc) {
		dev_err(&led->spmi_dev->dev,
			"Unable to read from addr=%x, rc(%d)\n",
			FLASH_PERIPHERAL_SUBTYPE(led->base), rc);
	}

	led->flash_cfg->torch_enable =
		of_property_read_bool(node, "qcom,torch-enable");

	led->flash_cfg->no_smbb_support =
		of_property_read_bool(node, "qcom,no-smbb-support");

	if (of_find_property(of_get_parent(node), "flash-wa-supply",
					NULL) && (!*reg_set)) {
		led->flash_cfg->flash_wa_reg =
			devm_regulator_get(&led->spmi_dev->dev,
					"flash-wa");
		if (IS_ERR_OR_NULL(led->flash_cfg->flash_wa_reg)) {
			rc = PTR_ERR(led->flash_cfg->flash_wa_reg);
			if (rc != EPROBE_DEFER) {
				dev_err(&led->spmi_dev->dev,
						"Falsh wa regulator get failed(%d)\n",
						rc);
			}
		}
	}

	if (led->id == QPNP_ID_FLASH1_LED0) {
		led->flash_cfg->enable_module = FLASH_ENABLE_LED_0;
		led->flash_cfg->current_addr = FLASH_LED_0_CURR(led->base);
		led->flash_cfg->trigger_flash = FLASH_LED_0_OUTPUT;
		if (!*reg_set) {
			led->flash_cfg->flash_boost_reg =
				regulator_get(&led->spmi_dev->dev,
							"flash-boost");
			if (IS_ERR(led->flash_cfg->flash_boost_reg)) {
				rc = PTR_ERR(led->flash_cfg->flash_boost_reg);
				dev_err(&led->spmi_dev->dev,
					"Regulator get failed(%d)\n", rc);
				goto error_get_flash_reg;
			}
			led->flash_cfg->flash_reg_get = true;
			*reg_set = true;
		} else
			led->flash_cfg->flash_reg_get = false;

		if (led->flash_cfg->torch_enable) {
			led->flash_cfg->second_addr =
						FLASH_LED_1_CURR(led->base);
		}
	} else if (led->id == QPNP_ID_FLASH1_LED1) {
		led->flash_cfg->enable_module = FLASH_ENABLE_LED_1;
		led->flash_cfg->current_addr = FLASH_LED_1_CURR(led->base);
		led->flash_cfg->trigger_flash = FLASH_LED_1_OUTPUT;
		if (!*reg_set) {
			led->flash_cfg->flash_boost_reg =
					regulator_get(&led->spmi_dev->dev,
								"flash-boost");
			if (IS_ERR(led->flash_cfg->flash_boost_reg)) {
				rc = PTR_ERR(led->flash_cfg->flash_boost_reg);
				dev_err(&led->spmi_dev->dev,
					"Regulator get failed(%d)\n", rc);
				goto error_get_flash_reg;
			}
			led->flash_cfg->flash_reg_get = true;
			*reg_set = true;
		} else
			led->flash_cfg->flash_reg_get = false;

		if (led->flash_cfg->torch_enable) {
			led->flash_cfg->second_addr =
						FLASH_LED_0_CURR(led->base);
		}
	} else {
		dev_err(&led->spmi_dev->dev, "Unknown flash LED name given\n");
		return -EINVAL;
	}

	if (led->flash_cfg->torch_enable) {
		if (of_find_property(of_get_parent(node), "torch-boost-supply",
									NULL)) {
			if (!led->flash_cfg->no_smbb_support) {
				led->flash_cfg->torch_boost_reg =
					regulator_get(&led->spmi_dev->dev,
								"torch-boost");
				if (IS_ERR(led->flash_cfg->torch_boost_reg)) {
					rc = PTR_ERR(led->flash_cfg->
							torch_boost_reg);
					dev_err(&led->spmi_dev->dev,
					"Torch regulator get failed(%d)\n", rc);
					goto error_get_torch_reg;
				}
			}
			led->flash_cfg->enable_module = FLASH_ENABLE_MODULE;
		} else
			led->flash_cfg->enable_module = FLASH_ENABLE_ALL;
		led->flash_cfg->trigger_flash = FLASH_TORCH_OUTPUT;

		rc = of_property_read_u32(node, "qcom,duration", &val);
		if (!rc)
			led->flash_cfg->duration = ((u8) val) - 2;
		else if (rc == -EINVAL)
			led->flash_cfg->duration = TORCH_DURATION_12s;
		else {
			if (led->flash_cfg->peripheral_subtype ==
							FLASH_SUBTYPE_SINGLE)
				goto error_get_flash_reg;
			else if (led->flash_cfg->peripheral_subtype ==
							FLASH_SUBTYPE_DUAL)
				goto error_get_torch_reg;
		}

		rc = of_property_read_u32(node, "qcom,current", &val);
		if (!rc)
			led->flash_cfg->current_prgm = (val *
				TORCH_MAX_LEVEL / led->max_current);
		else {
			if (led->flash_cfg->peripheral_subtype ==
							FLASH_SUBTYPE_SINGLE)
				goto error_get_flash_reg;
			else if (led->flash_cfg->peripheral_subtype ==
							FLASH_SUBTYPE_DUAL)
				goto error_get_torch_reg;
			goto error_get_torch_reg;
		}

		return 0;
	} else {
		rc = of_property_read_u32(node, "qcom,duration", &val);
		if (!rc)
			led->flash_cfg->duration = (u8)((val - 10) / 10);
		else if (rc == -EINVAL)
			led->flash_cfg->duration = FLASH_DURATION_200ms;
		else
			goto error_get_flash_reg;

		rc = of_property_read_u32(node, "qcom,current", &val);
		if (!rc)
			led->flash_cfg->current_prgm = (val *
				FLASH_MAX_LEVEL / led->max_current);
		else
			goto error_get_flash_reg;
	}

	rc = of_property_read_u32(node, "qcom,headroom", &val);
	if (!rc)
		led->flash_cfg->headroom = (u8) val;
	else if (rc == -EINVAL)
		led->flash_cfg->headroom = HEADROOM_500mV;
	else
		goto error_get_flash_reg;

	rc = of_property_read_u32(node, "qcom,clamp-curr", &val);
	if (!rc)
		led->flash_cfg->clamp_curr = (val *
				FLASH_MAX_LEVEL / led->max_current);
	else if (rc == -EINVAL)
		led->flash_cfg->clamp_curr = FLASH_CLAMP_200mA;
	else
		goto error_get_flash_reg;

	rc = of_property_read_u32(node, "qcom,startup-dly", &val);
	if (!rc)
		led->flash_cfg->startup_dly = (u8) val;
	else if (rc == -EINVAL)
		led->flash_cfg->startup_dly = DELAY_128us;
	else
		goto error_get_flash_reg;

	led->flash_cfg->safety_timer =
		of_property_read_bool(node, "qcom,safety-timer");

	led->flash_cfg->vreg_ok =
		of_property_read_bool(node, "qcom,sw_vreg_ok");

	return 0;

error_get_torch_reg:
	if (led->flash_cfg->no_smbb_support)
		regulator_put(led->flash_cfg->flash_boost_reg);
	else
		regulator_put(led->flash_cfg->torch_boost_reg);

error_get_flash_reg:
	regulator_put(led->flash_cfg->flash_boost_reg);
	return rc;

}

static int qpnp_get_config_pwm(struct pwm_config_data *pwm_cfg,
				struct spmi_device *spmi_dev,
				struct device_node *node)
{
	struct property *prop;
	int rc, i;
	u32 val;
	u8 *temp_cfg;

	rc = of_property_read_u32(node, "qcom,pwm-channel", &val);
	if (!rc)
		pwm_cfg->pwm_channel = val;
	else
		return rc;

	pwm_cfg->pwm_coefficient = 255;
	rc = of_property_read_u32(node, "qcom,pwm_coefficient", &val);
	if (!rc)
		pwm_cfg->pwm_coefficient = val;
	else if (rc != -EINVAL)
		return rc;

	if (pwm_cfg->mode != MANUAL_MODE) {
		rc = of_property_read_u32(node, "qcom,pwm-us", &val);
		if (!rc)
			pwm_cfg->pwm_period_us = val;
		else
			return rc;
	}

	pwm_cfg->use_blink =
		of_property_read_bool(node, "qcom,use-blink");

	if (pwm_cfg->mode == LPG_MODE || pwm_cfg->use_blink) {
		pwm_cfg->duty_cycles =
			devm_kzalloc(&spmi_dev->dev,
			sizeof(struct pwm_duty_cycles), GFP_KERNEL);
		if (!pwm_cfg->duty_cycles) {
			dev_err(&spmi_dev->dev,
				"Unable to allocate memory\n");
			rc = -ENOMEM;
			goto bad_lpg_params;
		}

		prop = of_find_property(node, "qcom,duty-pcts",
			&pwm_cfg->duty_cycles->num_duty_pcts);
		if (!prop) {
			dev_err(&spmi_dev->dev, "Looking up property " \
				"node qcom,duty-pcts failed\n");
			rc =  -ENODEV;
			goto bad_lpg_params;
		} else if (!pwm_cfg->duty_cycles->num_duty_pcts) {
			dev_err(&spmi_dev->dev, "Invalid length of " \
				"duty pcts\n");
			rc =  -EINVAL;
			goto bad_lpg_params;
		}

		pwm_cfg->duty_cycles->duty_pcts =
			devm_kzalloc(&spmi_dev->dev,
			sizeof(int) * PWM_LUT_MAX_SIZE,
			GFP_KERNEL);
		if (!pwm_cfg->duty_cycles->duty_pcts) {
			dev_err(&spmi_dev->dev,
				"Unable to allocate memory\n");
			rc = -ENOMEM;
			goto bad_lpg_params;
		}

		pwm_cfg->old_duty_pcts =
			devm_kzalloc(&spmi_dev->dev,
			sizeof(int) * PWM_LUT_MAX_SIZE,
			GFP_KERNEL);
		if (!pwm_cfg->old_duty_pcts) {
			dev_err(&spmi_dev->dev,
				"Unable to allocate memory\n");
			rc = -ENOMEM;
			goto bad_lpg_params;
		}

		temp_cfg = devm_kzalloc(&spmi_dev->dev,
				pwm_cfg->duty_cycles->num_duty_pcts *
				sizeof(u8), GFP_KERNEL);
		if (!temp_cfg) {
			dev_err(&spmi_dev->dev, "Failed to allocate " \
				"memory for duty pcts\n");
			rc = -ENOMEM;
			goto bad_lpg_params;
		}

		memcpy(temp_cfg, prop->value,
			pwm_cfg->duty_cycles->num_duty_pcts);

		for (i = 0; i < pwm_cfg->duty_cycles->num_duty_pcts; i++)
			pwm_cfg->duty_cycles->duty_pcts[i] =
				(int) temp_cfg[i];

		rc = of_property_read_u32(node, "qcom,start-idx", &val);
		if (!rc) {
			pwm_cfg->lut_params.start_idx = val;
			pwm_cfg->duty_cycles->start_idx = val;
		} else
			goto bad_lpg_params;

		pwm_cfg->lut_params.lut_pause_hi = 0;
		rc = of_property_read_u32(node, "qcom,pause-hi", &val);
		if (!rc)
			pwm_cfg->lut_params.lut_pause_hi = val;
		else if (rc != -EINVAL)
			goto bad_lpg_params;

		pwm_cfg->lut_params.lut_pause_lo = 0;
		rc = of_property_read_u32(node, "qcom,pause-lo", &val);
		if (!rc)
			pwm_cfg->lut_params.lut_pause_lo = val;
		else if (rc != -EINVAL)
			goto bad_lpg_params;

		pwm_cfg->lut_params.ramp_step_ms =
				QPNP_LUT_RAMP_STEP_DEFAULT;
		rc = of_property_read_u32(node, "qcom,ramp-step-ms", &val);
		if (!rc)
			pwm_cfg->lut_params.ramp_step_ms = val;
		else if (rc != -EINVAL)
			goto bad_lpg_params;

		pwm_cfg->lut_params.flags = QPNP_LED_PWM_FLAGS;
		rc = of_property_read_u32(node, "qcom,lut-flags", &val);
		if (!rc)
			pwm_cfg->lut_params.flags = (u8) val;
		else if (rc != -EINVAL)
			goto bad_lpg_params;

		pwm_cfg->lut_params.idx_len =
			pwm_cfg->duty_cycles->num_duty_pcts;
	}
	return 0;

bad_lpg_params:
	pwm_cfg->use_blink = false;
	if (pwm_cfg->mode == PWM_MODE) {
		dev_err(&spmi_dev->dev, "LPG parameters not set for" \
			" blink mode, defaulting to PWM mode\n");
		return 0;
	}
	return rc;
};

static int qpnp_led_get_mode(const char *mode)
{
	if (strncmp(mode, "manual", strlen(mode)) == 0)
		return MANUAL_MODE;
	else if (strncmp(mode, "pwm", strlen(mode)) == 0)
		return PWM_MODE;
	else if (strncmp(mode, "lpg", strlen(mode)) == 0)
		return LPG_MODE;
	else
		return -EINVAL;
};

static int qpnp_get_config_kpdbl(struct qpnp_led_data *led,
				struct device_node *node)
{
	int rc;
	u32 val;
	u8 led_mode;
	const char *mode;

	led->kpdbl_cfg = devm_kzalloc(&led->spmi_dev->dev,
				sizeof(struct kpdbl_config_data), GFP_KERNEL);
	if (!led->kpdbl_cfg) {
		dev_err(&led->spmi_dev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	rc = of_property_read_string(node, "qcom,mode", &mode);
	if (!rc) {
		led_mode = qpnp_led_get_mode(mode);
		if ((led_mode == MANUAL_MODE) || (led_mode == -EINVAL)) {
			dev_err(&led->spmi_dev->dev, "Selected mode not " \
				"supported for kpdbl.\n");
			return -EINVAL;
		}
		led->kpdbl_cfg->pwm_cfg = devm_kzalloc(&led->spmi_dev->dev,
					sizeof(struct pwm_config_data),
					GFP_KERNEL);
		if (!led->kpdbl_cfg->pwm_cfg) {
			dev_err(&led->spmi_dev->dev,
				"Unable to allocate memory\n");
			return -ENOMEM;
		}
		led->kpdbl_cfg->pwm_cfg->mode = led_mode;
		led->kpdbl_cfg->pwm_cfg->default_mode = led_mode;
	} else
		return rc;

	rc = qpnp_get_config_pwm(led->kpdbl_cfg->pwm_cfg, led->spmi_dev,  node);
	if (rc < 0)
		return rc;

	rc = of_property_read_u32(node, "qcom,row-id", &val);
	if (!rc)
		led->kpdbl_cfg->row_id = val;
	else
		return rc;

	led->kpdbl_cfg->row_src_vbst =
			of_property_read_bool(node, "qcom,row-src-vbst");

	led->kpdbl_cfg->row_src_en =
			of_property_read_bool(node, "qcom,row-src-en");

	led->kpdbl_cfg->always_on =
			of_property_read_bool(node, "qcom,always-on");

	return 0;
}

static int qpnp_get_config_rgb(struct qpnp_led_data *led,
				struct device_node *node)
{
	int rc;
	u8 led_mode;
	const char *mode;

	led->rgb_cfg = devm_kzalloc(&led->spmi_dev->dev,
				sizeof(struct rgb_config_data), GFP_KERNEL);
	if (!led->rgb_cfg) {
		dev_err(&led->spmi_dev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (led->id == QPNP_ID_RGB_RED)
		led->rgb_cfg->enable = RGB_LED_ENABLE_RED;
	else if (led->id == QPNP_ID_RGB_GREEN)
		led->rgb_cfg->enable = RGB_LED_ENABLE_GREEN;
	else if (led->id == QPNP_ID_RGB_BLUE)
		led->rgb_cfg->enable = RGB_LED_ENABLE_BLUE;
	else
		return -EINVAL;

	rc = of_property_read_string(node, "qcom,mode", &mode);
	if (!rc) {
		led_mode = qpnp_led_get_mode(mode);
		if ((led_mode == MANUAL_MODE) || (led_mode == -EINVAL)) {
			dev_err(&led->spmi_dev->dev, "Selected mode not " \
				"supported for rgb.\n");
			return -EINVAL;
		}
		led->rgb_cfg->pwm_cfg = devm_kzalloc(&led->spmi_dev->dev,
					sizeof(struct pwm_config_data),
					GFP_KERNEL);
		if (!led->rgb_cfg->pwm_cfg) {
			dev_err(&led->spmi_dev->dev,
				"Unable to allocate memory\n");
			return -ENOMEM;
		}
		led->rgb_cfg->pwm_cfg->mode = led_mode;
		led->rgb_cfg->pwm_cfg->default_mode = led_mode;
	} else
		return rc;

	rc = qpnp_get_config_pwm(led->rgb_cfg->pwm_cfg, led->spmi_dev, node);
	if (rc < 0)
		return rc;
	
	if (led->rgb_cfg->pwm_cfg->mode == LPG_MODE) {
		wake_lock_init(&pmic_led_wake_lock, WAKE_LOCK_SUSPEND, "qpnp_led");
		INIT_DELAYED_WORK(&led->fade_delayed_work, led_fade_do_work);
		led->rgb_cfg->pwm_cfg->lut_params.flags = QPNP_LED_PWM_FLAGS;
		led->rgb_cfg->pwm_cfg->duty_cycles->start_idx = 0;
		led->rgb_cfg->pwm_cfg->lut_params.start_idx = 0;
		led->rgb_cfg->pwm_cfg->lut_params.idx_len = 9;
	}
	

	return 0;
}

static int qpnp_get_config_mpp(struct qpnp_led_data *led,
		struct device_node *node)
{
	int rc;
	u32 val;
	u8 led_mode;
	const char *mode;

	led->mpp_cfg = devm_kzalloc(&led->spmi_dev->dev,
			sizeof(struct mpp_config_data), GFP_KERNEL);
	if (!led->mpp_cfg) {
		dev_err(&led->spmi_dev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (of_find_property(of_get_parent(node), "mpp-power-supply", NULL)) {
		led->mpp_cfg->mpp_reg =
				regulator_get(&led->spmi_dev->dev,
							"mpp-power");
		if (IS_ERR(led->mpp_cfg->mpp_reg)) {
			rc = PTR_ERR(led->mpp_cfg->mpp_reg);
			dev_err(&led->spmi_dev->dev,
				"MPP regulator get failed(%d)\n", rc);
			return rc;
		}
	}

	if (led->mpp_cfg->mpp_reg) {
		rc = of_property_read_u32(of_get_parent(node),
					"qcom,mpp-power-max-voltage", &val);
		if (!rc)
			led->mpp_cfg->max_uV = val;
		else
			goto err_config_mpp;

		rc = of_property_read_u32(of_get_parent(node),
					"qcom,mpp-power-min-voltage", &val);
		if (!rc)
			led->mpp_cfg->min_uV = val;
		else
			goto err_config_mpp;

	} else {
		rc = of_property_read_u32(of_get_parent(node),
					"qcom,mpp-power-max-voltage", &val);
		if (!rc)
			dev_warn(&led->spmi_dev->dev,
						"No regulator specified\n");

		rc = of_property_read_u32(of_get_parent(node),
					"qcom,mpp-power-min-voltage", &val);
		if (!rc)
			dev_warn(&led->spmi_dev->dev,
						"No regulator specified\n");
	}

	led->mpp_cfg->current_setting = LED_MPP_CURRENT_MIN;
	rc = of_property_read_u32(node, "qcom,current-setting", &val);
	if (!rc) {
		if (led->mpp_cfg->current_setting < LED_MPP_CURRENT_MIN)
			led->mpp_cfg->current_setting = LED_MPP_CURRENT_MIN;
		else if (led->mpp_cfg->current_setting > LED_MPP_CURRENT_MAX)
			led->mpp_cfg->current_setting = LED_MPP_CURRENT_MAX;
		else
			led->mpp_cfg->current_setting = (u8) val;
	} else if (rc != -EINVAL)
		goto err_config_mpp;

	led->mpp_cfg->source_sel = LED_MPP_SOURCE_SEL_DEFAULT;
	rc = of_property_read_u32(node, "qcom,source-sel", &val);
	if (!rc)
		led->mpp_cfg->source_sel = (u8) val;
	else if (rc != -EINVAL)
		goto err_config_mpp;

	led->mpp_cfg->mode_ctrl = LED_MPP_MODE_SINK;
	rc = of_property_read_u32(node, "qcom,mode-ctrl", &val);
	if (!rc)
		led->mpp_cfg->mode_ctrl = (u8) val;
	else if (rc != -EINVAL)
		goto err_config_mpp;

	led->mpp_cfg->vin_ctrl = LED_MPP_VIN_CTRL_DEFAULT;
	rc = of_property_read_u32(node, "qcom,vin-ctrl", &val);
	if (!rc)
		led->mpp_cfg->vin_ctrl = (u8) val;
	else if (rc != -EINVAL)
		goto err_config_mpp;

	led->mpp_cfg->min_brightness = 0;
	rc = of_property_read_u32(node, "qcom,min-brightness", &val);
	if (!rc)
		led->mpp_cfg->min_brightness = (u8) val;
	else if (rc != -EINVAL)
		goto err_config_mpp;

	rc = of_property_read_string(node, "qcom,mode", &mode);
	if (!rc) {
		led_mode = qpnp_led_get_mode(mode);
		led->mpp_cfg->pwm_mode = led_mode;
		if (led_mode == MANUAL_MODE)
			return MANUAL_MODE;
		else if (led_mode == -EINVAL) {
			dev_err(&led->spmi_dev->dev, "Selected mode not " \
				"supported for mpp.\n");
			rc = -EINVAL;
			goto err_config_mpp;
		}
		led->mpp_cfg->pwm_cfg = devm_kzalloc(&led->spmi_dev->dev,
					sizeof(struct pwm_config_data),
					GFP_KERNEL);
		if (!led->mpp_cfg->pwm_cfg) {
			dev_err(&led->spmi_dev->dev,
				"Unable to allocate memory\n");
			rc = -ENOMEM;
			goto err_config_mpp;
		}
		led->mpp_cfg->pwm_cfg->mode = led_mode;
		led->mpp_cfg->pwm_cfg->default_mode = led_mode;
	} else
		return rc;

	rc = qpnp_get_config_pwm(led->mpp_cfg->pwm_cfg, led->spmi_dev, node);
	if (rc < 0)
		goto err_config_mpp;

	return 0;

err_config_mpp:
	if (led->mpp_cfg->mpp_reg)
		regulator_put(led->mpp_cfg->mpp_reg);
	return rc;
}
static int lpg_blink(struct led_classdev *led_cdev, int val)
{
	struct qpnp_led_data *led;
	int rc;

	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	LED_INFO("%s: bank %d blink %d status %d\n", __func__, led->rgb_cfg->pwm_cfg->pwm_channel, val, led->status);

	switch (val) {
	case BLINK_STOP:
		pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
		rc = qpnp_led_masked_write(led,
				RGB_LED_EN_CTL(led->base),
				led->rgb_cfg->enable, RGB_LED_DISABLE);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
					"Failed to write led enable reg\n");
			return rc;
		}
		led->status = OFF;
		break;
	case BLINK_UNCHANGE:
		if (led->cdev.brightness) {
			if (led->status == BLINK) {
				if (led->rgb_cfg->pwm_cfg->mode == RGB_MODE_PWM) {
					rc = pwm_config_us(led->rgb_cfg->pwm_cfg->pwm_dev, 640 * led->rgb_cfg->pwm_cfg->pwm_coefficient / 255, 640);
					if (rc < 0) {
					dev_err(&led->spmi_dev->dev, "Failed to " \
					"configure pwm for new values\n");
					return rc;
					}
				}
				rc = qpnp_led_masked_write(led,
				RGB_LED_EN_CTL(led->base),
				led->rgb_cfg->enable, led->rgb_cfg->enable);
				if (rc) {
					dev_err(&led->spmi_dev->dev,
					"Failed to write led enable reg\n");
					return rc;
				}
				rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
				
				mdelay(10);
				rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
				led->status = ON;
			}
		} else {
			pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
			rc = qpnp_led_masked_write(led,
			RGB_LED_EN_CTL(led->base),
			led->rgb_cfg->enable, RGB_LED_DISABLE);
			if (rc) {
				dev_err(&led->spmi_dev->dev,
				"Failed to write led enable reg\n");
				return rc;
			}
			led->status = OFF;
		}
		break;
	case BLINK_64MS_PER_2SEC:
		pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
		if (led->rgb_cfg->pwm_cfg->mode == RGB_MODE_PWM) {
			rc = pwm_config_us(led->rgb_cfg->pwm_cfg->pwm_dev, 64000, 2000000);
			if (rc < 0) {
				dev_err(&led->spmi_dev->dev, "Failed to " \
					"configure pwm for new values\n");
				return rc;
			}
		}
		rc = qpnp_led_masked_write(led,
			RGB_LED_EN_CTL(led->base),
			led->rgb_cfg->enable, led->rgb_cfg->enable);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"Failed to write led enable reg\n");
			return rc;
		}
		rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
		
		mdelay(10);
		rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
		led->status = BLINK;
		break;
	case BLINK_64MS_ON_310MS_PER_2SEC:
		cancel_delayed_work_sync(&led->blink_delayed_work);
		pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
		led->rgb_cfg->pwm_cfg->pwm_duty_us = 64000;
		led->rgb_cfg->pwm_cfg->pwm_period_us = 2000000;
		queue_delayed_work(g_led_work_queue, &led->blink_delayed_work,
					msecs_to_jiffies(310));
		break;
	case BLINK_64MS_ON_2SEC_PER_2SEC:
		cancel_delayed_work_sync(&led->blink_delayed_work);
		pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
		led->rgb_cfg->pwm_cfg->pwm_duty_us = 64000;
		led->rgb_cfg->pwm_cfg->pwm_period_us = 2000000;
		queue_delayed_work(g_led_work_queue, &led->blink_delayed_work,
				   msecs_to_jiffies(1000));
		break;
	case BLINK_1SEC_PER_2SEC:
		pwm_disable(led->rgb_cfg->pwm_cfg->pwm_dev);
		if (led->rgb_cfg->pwm_cfg->mode == RGB_MODE_PWM) {
				rc = pwm_config_us(led->rgb_cfg->pwm_cfg->pwm_dev, 1000000, 2000000);
				if (rc < 0) {
						dev_err(&led->spmi_dev->dev, "Failed to " \
								"configure pwm for new values\n");
						return rc;
				}
		}
		rc = qpnp_led_masked_write(led,
				RGB_LED_EN_CTL(led->base),
				led->rgb_cfg->enable, led->rgb_cfg->enable);
		if (rc) {
				dev_err(&led->spmi_dev->dev,
						"Failed to write led enable reg\n");
				return rc;
		}
		rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
		
		mdelay(10);
		rc = pwm_enable(led->rgb_cfg->pwm_cfg->pwm_dev);
		led->status = BLINK;
		break;
	default:
		LED_ERR("%s: bank %d did not support blink type %d\n", __func__, led->rgb_cfg->pwm_cfg->pwm_channel, val);
		return -EINVAL;
	}
	return 0;
}

static void led_blink_work_func(struct work_struct *work)
{
	struct qpnp_led_data *ldata;
	int rc;
	LED_INFO("%s +++\n", __func__);
	ldata = container_of(work, struct qpnp_led_data, led_blink_work);
	if (ldata->id == QPNP_ID_LED_MPP) {
		rc = qpnp_mpp_set(ldata, ldata->mpp_cfg->blink_mode);
		if (rc < 0)
			dev_err(&ldata->spmi_dev->dev, "MPP set brightness failed (%d)\n", rc);
	} else {
		lpg_blink(&ldata->cdev, ldata->mode);
	}

	LED_INFO("%s ---\n", __func__);
}

static void led_off_work_func(struct work_struct *work)
{
	struct qpnp_led_data *ldata;

	ldata = container_of(work, struct qpnp_led_data, led_off_work);
	LED_INFO("%s: bank %d\n", __func__, ldata->id);
	qpnp_led_turn_off(ldata);
}

static int qpnp_get_config_gpio(struct qpnp_led_data *led,
		struct device_node *node)
{
	int rc;
	u32 val;

	led->gpio_cfg = devm_kzalloc(&led->spmi_dev->dev,
			sizeof(struct gpio_config_data), GFP_KERNEL);
	if (!led->gpio_cfg) {
		dev_err(&led->spmi_dev->dev, "Unable to allocate memory gpio struct\n");
		return -ENOMEM;
	}

	led->gpio_cfg->source_sel = LED_GPIO_SOURCE_SEL_DEFAULT;
	rc = of_property_read_u32(node, "qcom,source-sel", &val);
	if (!rc)
		led->gpio_cfg->source_sel = (u8) val;
	else if (rc != -EINVAL)
		goto err_config_gpio;

	led->gpio_cfg->mode_ctrl = LED_GPIO_MODE_OUTPUT;
	rc = of_property_read_u32(node, "qcom,mode-ctrl", &val);
	if (!rc)
		led->gpio_cfg->mode_ctrl = (u8) val;
	else if (rc != -EINVAL)
		goto err_config_gpio;

	led->gpio_cfg->vin_ctrl = LED_GPIO_VIN_CTRL_DEFAULT;
	rc = of_property_read_u32(node, "qcom,vin-ctrl", &val);
	if (!rc)
		led->gpio_cfg->vin_ctrl = (u8) val;
	else if (rc != -EINVAL)
		goto err_config_gpio;

	return 0;

err_config_gpio:
	return rc;
}

static ssize_t led_off_timer_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	struct led_classdev *led_cdev;
	int min, sec;
	uint16_t off_timer;

	min = -1;
	sec = -1;
	sscanf(buf, "%d %d", &min, &sec);

	if (min < 0 || min > 255)
		return -EINVAL;
	if (sec < 0 || sec > 255)
		return -EINVAL;

	led_cdev = (struct led_classdev *) dev_get_drvdata(dev);
	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	LED_INFO("Setting %s off_timer to %d min %d sec \n", led_cdev->name, min, sec);
	off_timer = min * 60 + sec;

	return count;
}
static DEVICE_ATTR(off_timer, 0644, NULL, led_off_timer_store);


static ssize_t pm8xxx_led_blink_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf)
{
	return sprintf(buf, "%d\n", current_blink);
}

static ssize_t pm8xxx_led_blink_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	struct led_classdev *led_cdev;
	int val;
	bool ret;
	int retry_count = 0;

	val = -1;
	sscanf(buf, "%u", &val);
	if (val < -1 || val > 255)
			return -EINVAL;
	led_cdev = (struct led_classdev *) dev_get_drvdata(dev);
	led = container_of(led_cdev, struct qpnp_led_data, cdev);
	led->mode = val;
	current_blink = val;
	LED_INFO("%s: blink: %d\n", __func__, val);
	switch(led->id) {
		case QPNP_ID_LED_MPP:
			led->mpp_cfg->blink_mode = val;
			if (led->cdev.brightness) {
				led->mpp_cfg->pwm_cfg->blinking = true;
				if (led->id == QPNP_ID_LED_MPP)
					led->mpp_cfg->pwm_mode = LPG_MODE;
				led->mpp_cfg->pwm_cfg->mode = LPG_MODE;
			} else {
				led->mpp_cfg->pwm_cfg->blinking = false;
				led->mpp_cfg->pwm_cfg->mode = led->mpp_cfg->pwm_cfg->default_mode;
				if (led->id == QPNP_ID_LED_MPP)
					led->mpp_cfg->pwm_mode = led->mpp_cfg->pwm_cfg->default_mode;
			}
		case QPNP_ID_RGB_RED:
		case QPNP_ID_RGB_GREEN:
		case QPNP_ID_RGB_BLUE:
			do {
				ret = queue_work(g_led_on_work_queue, &led->led_blink_work);
				if (!ret) {
					retry_count++;
					msleep(1);
					LED_INFO("%s led_blink_work already on queue, requeue!! ret:%d retry:%d\n", __func__, ret, retry_count);
				}
			} while ((!ret) && (retry_count < 3));
			break;
		case QPNP_ID_KPDBL:
			break;
		default:
			return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR(blink, 0644, pm8xxx_led_blink_show, pm8xxx_led_blink_store);

static ssize_t led_pwm_coefficient_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct qpnp_led_data *led;
	struct led_classdev *led_cdev;

	led_cdev = (struct led_classdev *) dev_get_drvdata(dev);
	led = container_of(led_cdev, struct qpnp_led_data, cdev);
	switch(led->id) {
		case QPNP_ID_LED_MPP:
			return sprintf(buf, "%d\n", led->mpp_cfg->pwm_cfg->pwm_coefficient);
				break;
			case QPNP_ID_RGB_RED:
			case QPNP_ID_RGB_GREEN:
			case QPNP_ID_RGB_BLUE:
				return sprintf(buf, "%d\n", led->rgb_cfg->pwm_cfg->pwm_coefficient);
				break;
				default:
			return -EINVAL;
	}
}

static ssize_t led_pwm_coefficient_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int pwm_coefficient1 = 0;
	struct qpnp_led_data *led;
	struct led_classdev *led_cdev;
	int val;

	val = -1;
	sscanf(buf, "%u", &val);
	if (val < -1 || val > 255)
			return -EINVAL;
	led_cdev = (struct led_classdev *) dev_get_drvdata(dev);
	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	sscanf(buf, "%d", &pwm_coefficient1);
	if ((pwm_coefficient1 < 0) || (pwm_coefficient1 > 255)) {
		LED_INFO("%s: pwm_coefficient = %d, out of range.\n",
			__func__, pwm_coefficient1);
		return -EINVAL;
	}

	LED_INFO("%s: pwm_coefficient %d\n", __func__, pwm_coefficient1);
	switch(led->id) {
	case QPNP_ID_LED_MPP:
		led->mpp_cfg->pwm_cfg->pwm_coefficient = pwm_coefficient1;
		break;
	case QPNP_ID_RGB_RED:
	case QPNP_ID_RGB_GREEN:
	case QPNP_ID_RGB_BLUE:
		led->rgb_cfg->pwm_cfg->pwm_coefficient = pwm_coefficient1;
		break;
	default:
		return -EINVAL;
		}
	return count;
}
static DEVICE_ATTR(pwm_coefficient, 0644, led_pwm_coefficient_show, led_pwm_coefficient_store);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void qpnp_led_early_suspend(struct early_suspend *handler)
{
}

static void qpnp_led_late_resume(struct early_suspend *handler)
{
}
#endif
static int qpnp_leds_probe(struct spmi_device *spmi)
{
	struct qpnp_led_data *led, *led_array;
	struct resource *led_resource;
	struct device_node *node, *temp;
	int rc, i, num_leds = 0, parsed_leds = 0;
	const char *led_label;
	bool regulator_probe = false;
	LED_INFO("led driver probe");
	node = spmi->dev.of_node;
	if (node == NULL)
		return -ENODEV;

	temp = NULL;
	while ((temp = of_get_next_child(node, temp)))
		num_leds++;

	if (!num_leds)
		return -ECHILD;

	led_array = devm_kzalloc(&spmi->dev,
		(sizeof(struct qpnp_led_data) * num_leds), GFP_KERNEL);
	if (!led_array) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	for_each_child_of_node(node, temp) {
		led = &led_array[parsed_leds];
		led->num_leds = num_leds;
		led->spmi_dev = spmi;

		led_resource = spmi_get_resource(spmi, NULL, IORESOURCE_MEM, 0);
		if (!led_resource) {
			dev_err(&spmi->dev, "Unable to get LED base address\n");
			rc = -ENXIO;
			goto fail_id_check;
		}
		led->base = led_resource->start;

		rc = of_property_read_string(temp, "label", &led_label);
		if (rc < 0) {
			dev_err(&led->spmi_dev->dev,
				"Failure reading label, rc = %d\n", rc);
			goto fail_id_check;
		}

		rc = of_property_read_string(temp, "linux,name",
			&led->cdev.name);
		if (rc < 0) {
			dev_err(&led->spmi_dev->dev,
				"Failure reading led name, rc = %d\n", rc);
			goto fail_id_check;
		}

		rc = of_property_read_u32(temp, "qcom,max-current",
			&led->max_current);
		if (rc < 0) {
			dev_err(&led->spmi_dev->dev,
				"Failure reading max_current, rc =  %d\n", rc);
			goto fail_id_check;
		}

		rc = of_property_read_u32(temp, "qcom,id", &led->id);
		if (rc < 0) {
			dev_err(&led->spmi_dev->dev,
				"Failure reading led id, rc =  %d\n", rc);
			goto fail_id_check;
		}

		rc = qpnp_get_common_configs(led, temp);
		if (rc) {
			dev_err(&led->spmi_dev->dev,
				"Failure reading common led configuration," \
				" rc = %d\n", rc);
			goto fail_id_check;
		}

		led->cdev.brightness_set    = qpnp_led_set;
		led->cdev.brightness_get    = qpnp_led_get;

		if (strncmp(led_label, "wled", sizeof("wled")) == 0) {
			rc = qpnp_get_config_wled(led, temp);
			if (rc < 0) {
				dev_err(&led->spmi_dev->dev,
					"Unable to read wled config data\n");
				goto fail_id_check;
			}
		} else if (strncmp(led_label, "flash", sizeof("flash"))
				== 0) {
			if (!of_find_property(node, "flash-boost-supply", NULL))
				regulator_probe = true;
			rc = qpnp_get_config_flash(led, temp, &regulator_probe);
			if (rc < 0) {
				dev_err(&led->spmi_dev->dev,
					"Unable to read flash config data\n");
				goto fail_id_check;
			}
		} else if (strncmp(led_label, "rgb", sizeof("rgb")) == 0) {
			rc = qpnp_get_config_rgb(led, temp);
			if (rc < 0) {
				dev_err(&led->spmi_dev->dev,
					"Unable to read rgb config data\n");
				goto fail_id_check;
			}
		} else if (strncmp(led_label, "mpp", sizeof("mpp")) == 0) {
			rc = qpnp_get_config_mpp(led, temp);
			if (rc < 0) {
				dev_err(&led->spmi_dev->dev,
						"Unable to read mpp config data\n");
				goto fail_id_check;
			}
		} else if (strcmp(led_label, "gpio") == 0) {
			rc = qpnp_get_config_gpio(led, temp);
			if (rc < 0) {
				dev_err(&led->spmi_dev->dev,
						"Unable to read gpio config data\n");
				goto fail_id_check;
			}
		} else if (strncmp(led_label, "kpdbl", sizeof("kpdbl")) == 0) {
			num_kpbl_leds_on = 0;
			rc = qpnp_get_config_kpdbl(led, temp);
			if (rc < 0) {
				dev_err(&led->spmi_dev->dev,
					"Unable to read kpdbl config data\n");
				goto fail_id_check;
			}
		} else {
			dev_err(&led->spmi_dev->dev, "No LED matching label\n");
			rc = -EINVAL;
			goto fail_id_check;
		}

		if (led->id != QPNP_ID_FLASH1_LED0 &&
					led->id != QPNP_ID_FLASH1_LED1)
			mutex_init(&led->lock);
		INIT_WORK(&led->work, qpnp_led_work);
		INIT_WORK(&led->led_blink_work, led_blink_work_func);
		g_led_work_queue = create_workqueue("qpnp-led");
		if (g_led_work_queue == NULL) {
			LED_ERR("failed to create workqueue\n");
			goto err_create_work_queue;
		}
		g_gpled_work_queue = create_workqueue("qpnp-gpled");
		if (g_gpled_work_queue == NULL) {
			LED_ERR("failed to create workqueue\n");
			goto err_create_work_queue;
		}
		g_led_on_work_queue = create_workqueue("pm8xxx-led-on");
		if (g_led_on_work_queue == NULL) {
			LED_ERR("failed to create workqueue\n");
			goto err_create_work_queue;
		}

		led->in_order_command_processing = of_property_read_bool
				(temp, "qcom,in-order-command-processing");

		if (led->in_order_command_processing) {
			led->workqueue = alloc_ordered_workqueue
							("led_workqueue", 0);
			if (!led->workqueue) {
				rc = -ENOMEM;
				goto fail_id_check;
			}
		}

		INIT_WORK(&led->work, qpnp_led_work);

		rc =  qpnp_led_initialize(led);
		if (rc < 0)
			goto fail_id_check;

		rc = qpnp_led_set_max_brightness(led);
		if (rc < 0)
			goto fail_id_check;

		rc = led_classdev_register(&spmi->dev, &led->cdev);
		if (rc) {
			dev_err(&spmi->dev, "unable to register led %d,rc=%d\n",
						 led->id, rc);
			goto fail_id_check;
		}

		if (led->id == QPNP_ID_FLASH1_LED0 ||
			led->id == QPNP_ID_FLASH1_LED1) {
			rc = sysfs_create_group(&led->cdev.dev->kobj,
							&led_attr_group);
			if (rc)
				goto fail_id_check;

		}

		if (led->id == QPNP_ID_LED_MPP) {
			if (!led->mpp_cfg->pwm_cfg)
				break;
			if (led->mpp_cfg->pwm_cfg->mode == PWM_MODE) {
				rc = sysfs_create_group(&led->cdev.dev->kobj,
					&pwm_attr_group);
				if (rc)
					goto fail_id_check;
			}
			if (led->mpp_cfg->pwm_cfg->use_blink) {
				
				rc = sysfs_create_group(&led->cdev.dev->kobj,
					&blink_attr_group);
				if (rc)
					goto fail_id_check;

				rc = sysfs_create_group(&led->cdev.dev->kobj,
					&lpg_attr_group);
				if (rc)
					goto fail_id_check;
			} else if (led->mpp_cfg->pwm_cfg->mode == LPG_MODE) {
				rc = sysfs_create_group(&led->cdev.dev->kobj,
					&lpg_attr_group);
				if (rc)
					goto fail_id_check;
			}
		} else if ((led->id == QPNP_ID_RGB_RED) ||
			(led->id == QPNP_ID_RGB_GREEN) ||
			(led->id == QPNP_ID_RGB_BLUE)) {
			if (led->rgb_cfg->pwm_cfg->mode == PWM_MODE) {
				rc = sysfs_create_group(&led->cdev.dev->kobj,
					&pwm_attr_group);
				if (rc)
					goto fail_id_check;
			}
			if (led->rgb_cfg->pwm_cfg->use_blink) {
				rc = sysfs_create_group(&led->cdev.dev->kobj,
					&blink_attr_group);
				if (rc)
					goto fail_id_check;

				rc = sysfs_create_group(&led->cdev.dev->kobj,
					&lpg_attr_group);
				if (rc)
					goto fail_id_check;
			} else if (led->rgb_cfg->pwm_cfg->mode == LPG_MODE) {
				rc = sysfs_create_group(&led->cdev.dev->kobj,
					&lpg_attr_group);
				if (rc)
					goto fail_id_check;
			}
		}

		
		if (led->default_on) {
			led->cdev.brightness = led->cdev.max_brightness;
			__qpnp_led_work(led, led->cdev.brightness);
			if (led->turn_off_delay_ms > 0)
				qpnp_led_turn_off(led);
		} else
			led->cdev.brightness = LED_OFF;
		if (strncmp(led_label, "rgb", sizeof("rgb")) == 0) {
			rc = device_create_file(led->cdev.dev, &dev_attr_blink);
			rc = device_create_file(led->cdev.dev, &dev_attr_pwm_coefficient);
			if (rc < 0) {
				LED_ERR("%s: Failed to create %s attr blink\n", __func__,  led->cdev.name);
			}
			if (led->rgb_cfg->pwm_cfg->mode == RGB_MODE_PWM) {
				rc = device_create_file(led->cdev.dev, &dev_attr_off_timer);
				if (rc < 0) {
					LED_ERR("%s: Failed to create %s attr off_timer\n", __func__,  led->cdev.name);
				}
				
				INIT_WORK(&led->led_off_work, led_off_work_func); 
			}
			INIT_DELAYED_WORK(&led->blink_delayed_work, led_blink_do_work);
		}
		if (strncmp(led_label, "mpp", sizeof("mpp")) == 0) {
			rc = device_create_file(led->cdev.dev, &dev_attr_blink);
			rc = device_create_file(led->cdev.dev, &dev_attr_pwm_coefficient);
			if (rc < 0) {
				LED_ERR("%s: Failed to create %s attr blink\n", __func__,  led->cdev.name);
			}
			if (led->mpp_cfg->pwm_cfg->mode == PWM_MODE) {
				rc = device_create_file(led->cdev.dev, &dev_attr_off_timer);
				if (rc < 0) {
					LED_ERR("%s: Failed to create %s attr off_timer\n", __func__,  led->cdev.name);
				}
				
				INIT_WORK(&led->led_off_work, led_off_work_func); 
			}
			INIT_DELAYED_WORK(&led->blink_delayed_work, led_blink_do_work);
		}

		if (strncmp(led_label, "kpdbl", sizeof("kpdbl")) == 0) {
			rc = device_create_file(led->cdev.dev, &dev_attr_blink);
			if (rc < 0) {
				LED_ERR("%s: Failed to create %s attr blink\n", __func__,  led->cdev.name);
			}
		}
		parsed_leds++;
	}
	dev_set_drvdata(&spmi->dev, led_array);
	LED_INFO("led driver probe --");
	return 0;
err_create_work_queue:
fail_id_check:
	for (i = 0; i < parsed_leds; i++) {
		if (led_array[i].id != QPNP_ID_FLASH1_LED0 &&
				led_array[i].id != QPNP_ID_FLASH1_LED1)
			mutex_destroy(&led_array[i].lock);
		if (led_array[i].in_order_command_processing)
			destroy_workqueue(led_array[i].workqueue);
		led_classdev_unregister(&led_array[i].cdev);
	}

	return rc;
}

static int qpnp_leds_remove(struct spmi_device *spmi)
{
	struct qpnp_led_data *led_array  = dev_get_drvdata(&spmi->dev);
	int i, parsed_leds = led_array->num_leds;

	for (i = 0; i < parsed_leds; i++) {
		cancel_work_sync(&led_array[i].work);
		if (led_array[i].id != QPNP_ID_FLASH1_LED0 &&
				led_array[i].id != QPNP_ID_FLASH1_LED1)
			mutex_destroy(&led_array[i].lock);

		if (led_array[i].in_order_command_processing)
			destroy_workqueue(led_array[i].workqueue);
		led_classdev_unregister(&led_array[i].cdev);
		switch (led_array[i].id) {
		case QPNP_ID_WLED:
			break;
		case QPNP_ID_FLASH1_LED0:
		case QPNP_ID_FLASH1_LED1:
			if (led_array[i].flash_cfg->flash_reg_get)
				regulator_put(led_array[i].flash_cfg-> \
							flash_boost_reg);
			if (led_array[i].flash_cfg->torch_enable)
				if (!led_array[i].flash_cfg->no_smbb_support)
					regulator_put(led_array[i].
					flash_cfg->torch_boost_reg);
			sysfs_remove_group(&led_array[i].cdev.dev->kobj,
							&led_attr_group);
			break;
		case QPNP_ID_RGB_RED:
		case QPNP_ID_RGB_GREEN:
		case QPNP_ID_RGB_BLUE:
			if (led_array[i].rgb_cfg->pwm_cfg->mode == PWM_MODE)
				sysfs_remove_group(&led_array[i].cdev.dev->\
					kobj, &pwm_attr_group);
			if (led_array[i].rgb_cfg->pwm_cfg->use_blink) {
				sysfs_remove_group(&led_array[i].cdev.dev->\
					kobj, &blink_attr_group);
				sysfs_remove_group(&led_array[i].cdev.dev->\
					kobj, &lpg_attr_group);
			} else if (led_array[i].rgb_cfg->pwm_cfg->mode\
					== LPG_MODE)
				sysfs_remove_group(&led_array[i].cdev.dev->\
					kobj, &lpg_attr_group);
			break;
		case QPNP_ID_LED_MPP:
			if (!led_array[i].mpp_cfg->pwm_cfg)
				break;
			if (led_array[i].mpp_cfg->pwm_cfg->mode == PWM_MODE)
				sysfs_remove_group(&led_array[i].cdev.dev->\
					kobj, &pwm_attr_group);
			if (led_array[i].mpp_cfg->pwm_cfg->use_blink) {
				sysfs_remove_group(&led_array[i].cdev.dev->\
					kobj, &blink_attr_group);
				sysfs_remove_group(&led_array[i].cdev.dev->\
					kobj, &lpg_attr_group);
			} else if (led_array[i].mpp_cfg->pwm_cfg->mode\
					== LPG_MODE)
				sysfs_remove_group(&led_array[i].cdev.dev->\
					kobj, &lpg_attr_group);
			if (led_array[i].mpp_cfg->mpp_reg)
				regulator_put(led_array[i].mpp_cfg->mpp_reg);
			break;
		default:
			dev_err(&led_array[i].spmi_dev->dev,
					"Invalid LED(%d)\n",
					led_array[i].id);
			return -EINVAL;
		}
	}

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id spmi_match_table[] = {
	{ .compatible = "qcom,leds-qpnp",},
	{ },
};
#else
#define spmi_match_table NULL
#endif

static struct spmi_driver qpnp_leds_driver = {
	.driver		= {
		.name	= "qcom,leds-qpnp",
		.of_match_table = spmi_match_table,
	},
	.probe		= qpnp_leds_probe,
	.remove		= qpnp_leds_remove,
};

static int __init qpnp_led_init(void)
{
	return spmi_driver_register(&qpnp_leds_driver);
}
module_init(qpnp_led_init);

static void __exit qpnp_led_exit(void)
{
	spmi_driver_unregister(&qpnp_leds_driver);
}
module_exit(qpnp_led_exit);

MODULE_DESCRIPTION("QPNP LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("leds:leds-qpnp");

