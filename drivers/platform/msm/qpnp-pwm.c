/* Copyright (c) 2012, The Linux Foundation. All rights reserved.

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

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/spmi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/radix-tree.h>
#include <linux/qpnp/pwm.h>
#include <linux/leds.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/earlysuspend.h>
#include <linux/android_alarm.h>
#include <linux/string.h>

#define QPNP_LPG_DRIVER_NAME	"qcom,qpnp-pwm"
#define QPNP_LPG_CHANNEL_BASE	"qpnp-lpg-channel-base"
#define QPNP_LPG_LUT_BASE	"qpnp-lpg-lut-base"
#define QPNP_PWM_MODE_ONLY_SUB_TYPE	0x0B

#define QPNP_RAMP_DIRECTION_SHIFT	4
#define QPNP_RAMP_DIRECTION_MASK	0x10
#define QPNP_PATTERN_REPEAT_SHIFT	3
#define QPNP_PATTERN_REPEAT_MASK	0x08
#define QPNP_RAMP_TOGGLE_SHIFT		2
#define QPNP_RAMP_TOGGLE_MASK		0x04
#define QPNP_EN_PAUSE_HI_SHIFT		1
#define QPNP_EN_PAUSE_HI_MASK		0x02
#define QPNP_EN_PAUSE_LO_MASK		0x01

#define QPNP_PWM_SIZE_SHIFT_SUB_TYPE		2
#define QPNP_PWM_SIZE_MASK_SUB_TYPE		0x4
#define QPNP_PWM_FREQ_CLK_SELECT_MASK_SUB_TYPE	0x03
#define QPNP_PWM_SIZE_9_BIT_SUB_TYPE		0x01

#define QPNP_SET_PWM_CLK_SUB_TYPE(val, clk, pwm_size) \
do { \
	val = (clk + 1) & QPNP_PWM_FREQ_CLK_SELECT_MASK_SUB_TYPE; \
	val |= (((pwm_size > 6 ? QPNP_PWM_SIZE_9_BIT_SUB_TYPE : 0) << \
		QPNP_PWM_SIZE_SHIFT_SUB_TYPE) & QPNP_PWM_SIZE_MASK_SUB_TYPE); \
} while (0)

#define QPNP_GET_PWM_SIZE_SUB_TYPE(reg) ((reg & QPNP_PWM_SIZE_MASK_SUB_TYPE) \
				>> QPNP_PWM_SIZE_SHIFT_SUB_TYPE)

#define QPNP_PWM_SIZE_SHIFT			4
#define QPNP_PWM_SIZE_MASK			0x30
#define QPNP_PWM_FREQ_CLK_SELECT_MASK		0x03
#define QPNP_MIN_PWM_BIT_SIZE		6
#define QPNP_MAX_PWM_BIT_SIZE		9
#define QPNP_SET_PWM_CLK(val, clk, pwm_size) \
do { \
	val = (clk + 1) & QPNP_PWM_FREQ_CLK_SELECT_MASK; \
	val |= (((pwm_size - QPNP_MIN_PWM_BIT_SIZE) << \
		QPNP_PWM_SIZE_SHIFT) & QPNP_PWM_SIZE_MASK); \
} while (0)

#define QPNP_GET_PWM_SIZE(reg) ((reg & QPNP_PWM_SIZE_MASK) \
				>> QPNP_PWM_SIZE_SHIFT)

#define QPNP_PWM_FREQ_PRE_DIVIDE_SHIFT		5
#define QPNP_PWM_FREQ_PRE_DIVIDE_MASK		0x60
#define QPNP_PWM_FREQ_EXP_MASK			0x07

#define QPNP_SET_PWM_FREQ_PREDIV(val, pre_div, pre_div_exp) \
do { \
	val = (pre_div << QPNP_PWM_FREQ_PRE_DIVIDE_SHIFT) & \
				QPNP_PWM_FREQ_PRE_DIVIDE_MASK;	\
	val |= (pre_div_exp & QPNP_PWM_FREQ_EXP_MASK);	\
} while (0)

#define QPNP_EN_GLITCH_REMOVAL_SHIFT		5
#define QPNP_EN_GLITCH_REMOVAL_MASK		0x20
#define QPNP_EN_FULL_SCALE_SHIFT		3
#define QPNP_EN_FULL_SCALE_MASK			0x08
#define QPNP_EN_PHASE_STAGGER_SHIFT		2
#define QPNP_EN_PHASE_STAGGER_MASK		0x04
#define QPNP_PHASE_STAGGER_MASK			0x03

#define QPNP_PWM_VALUE_LSB_MASK			0xFF

#define QPNP_PWM_VALUE_MSB_SHIFT		8
#define QPNP_PWM_VALUE_MSB_MASK			0x01

#define QPNP_EN_PWM_HIGH_SHIFT			7
#define QPNP_EN_PWM_HIGH_MASK			0x80
#define QPNP_EN_PWM_LO_SHIFT			6
#define QPNP_EN_PWM_LO_MASK			0x40
#define QPNP_EN_PWM_OUTPUT_SHIFT		5
#define QPNP_EN_PWM_OUTPUT_MASK			0x20
#define QPNP_PWM_SRC_SELECT_SHIFT		2
#define QPNP_PWM_SRC_SELECT_MASK		0x04
#define QPNP_PWM_EN_RAMP_GEN_SHIFT		1
#define QPNP_PWM_EN_RAMP_GEN_MASK		0x02

#define QPNP_ENABLE_PWM(value) \
	(value |= (1 << QPNP_EN_PWM_OUTPUT_SHIFT) & QPNP_EN_PWM_OUTPUT_MASK)

#define QPNP_DISABLE_PWM(value)  (value &= ~QPNP_EN_PWM_OUTPUT_MASK)

#define QPNP_PWM_SYNC_VALUE			0x01
#define QPNP_PWM_SYNC_MASK			0x01

#define QPNP_RAMP_START_MASK			0x01
#define QPNP_RAMP_CONTROL_SHIFT			8

#define QPNP_ENABLE_LUT_V0(value) (value |= QPNP_RAMP_START_MASK)
#define QPNP_DISABLE_LUT_V0(value) (value &= ~QPNP_RAMP_START_MASK)
#define QPNP_ENABLE_LUT_V1(value, id) \
do { \
	(id < 8) ? (value |= BIT(id)) : \
	(value |= (BIT(id) >> QPNP_RAMP_CONTROL_SHIFT)); \
} while (0)

#define QPNP_RAMP_STEP_DURATION_LSB_MASK	0xFF

#define QPNP_RAMP_STEP_DURATION_MSB_SHIFT	8
#define QPNP_RAMP_STEP_DURATION_MSB_MASK	0x01

#define QPNP_PWM_1KHZ				1024
#define QPNP_GET_RAMP_STEP_DURATION(ramp_time_ms) \
		((ramp_time_ms * QPNP_PWM_1KHZ) / 1000)

#define QPNP_PAUSE_HI_MULTIPLIER_LSB_MASK	0xFF

#define QPNP_PAUSE_HI_MULTIPLIER_MSB_SHIFT	8
#define QPNP_PAUSE_HI_MULTIPLIER_MSB_MASK	0x1F

#define QPNP_PAUSE_LO_MULTIPLIER_LSB_MASK	0xFF

#define QPNP_PAUSE_LO_MULTIPLIER_MSB_SHIFT	8
#define QPNP_PAUSE_LO_MULTIPLIER_MSB_MASK	0x1F

#define QPNP_HI_INDEX_MASK			0x3F

#define QPNP_LO_INDEX_MASK			0x3F

#define NUM_CLOCKS				3
#define QPNP_PWM_M_MAX				7
#define NSEC_1024HZ	(NSEC_PER_SEC / 1024)
#define NSEC_32768HZ	(NSEC_PER_SEC / 32768)
#define NSEC_19P2MHZ	(NSEC_PER_SEC / 19200000)

#define NUM_LPG_PRE_DIVIDE	4

#define PRE_DIVIDE_1		1
#define PRE_DIVIDE_3		3
#define PRE_DIVIDE_5		5
#define PRE_DIVIDE_6		6

#define SPMI_LPG_REG_BASE_OFFSET	0x40
#define SPMI_LPG_REVISION2_OFFSET	0x1
#define SPMI_LPG_REV1_RAMP_CONTROL_OFFSET	0x86
#define SPMI_LPG_SUB_TYPE_OFFSET	0x5
#define SPMI_LPG_PWM_SYNC		0x7
#define SPMI_LPG_REG_ADDR(b, n)	(b + SPMI_LPG_REG_BASE_OFFSET + (n))
#define SPMI_MAX_BUF_LEN	8

#define QPNP_GPLED_LPG_CHANNEL_RANGE_START 8
#define QPNP_GPLED_LPG_CHANNEL_RANGE_END 11
#define qpnp_check_gpled_lpg_channel(id) \
	(id >= QPNP_GPLED_LPG_CHANNEL_RANGE_START && \
	id <= QPNP_GPLED_LPG_CHANNEL_RANGE_END)
#define QPNP_PWM_LUT_NOT_SUPPORTED	0x1

#define LED_INFO(fmt, ...) \
		printk(KERN_INFO "[LED]" fmt, ##__VA_ARGS__)
#define LED_ERR(fmt, ...) \
		printk(KERN_ERR "[LED][ERR]" fmt, ##__VA_ARGS__)

#define LPG_CHAN2_LPG_PATTERN_CONFIG 0xb240
#define LPG_CHAN2_LPG_PWM_SIZE_CLK 0xb241
#define LPG_LUT_RAMP_CONTROL 0xb0c8
#define QPNP_PWM_SIZE_6_BIT		6
#define QPNP_PWM_SIZE_7_BIT		7
#define QPNP_PWM_SIZE_8_BIT		8
#define QPNP_PWM_SIZE_9_BIT		9

enum time_level {
	LVL_NSEC,
	LVL_USEC,
};

enum qpnp_lpg_revision {
	QPNP_LPG_REVISION_0 = 0x0,
	QPNP_LPG_REVISION_1 = 0x1,
};

enum qpnp_lut_state {
	QPNP_LUT_ENABLE = 0x0,
	QPNP_LUT_DISABLE = 0x1,
};

enum qpnp_pwm_state {
	QPNP_PWM_ENABLE = 0x0,
	QPNP_PWM_DISABLE = 0x1,
};

enum qpnp_lpg_registers_list {
	QPNP_LPG_PATTERN_CONFIG,
	QPNP_LPG_PWM_SIZE_CLK,
	QPNP_LPG_PWM_FREQ_PREDIV_CLK,
	QPNP_LPG_PWM_TYPE_CONFIG,
	QPNP_PWM_VALUE_LSB,
	QPNP_PWM_VALUE_MSB,
	QPNP_ENABLE_CONTROL,
	QPNP_RAMP_CONTROL,
	QPNP_RAMP_STEP_DURATION_LSB = QPNP_RAMP_CONTROL + 9,
	QPNP_RAMP_STEP_DURATION_MSB,
	QPNP_PAUSE_HI_MULTIPLIER_LSB,
	QPNP_PAUSE_HI_MULTIPLIER_MSB,
	QPNP_PAUSE_LO_MULTIPLIER_LSB,
	QPNP_PAUSE_LO_MULTIPLIER_MSB,
	QPNP_HI_INDEX,
	QPNP_LO_INDEX,
	QPNP_TOTAL_LPG_SPMI_REGISTERS
};

#define QPNP_SET_PAUSE_CNT(to_pause_cnt, from_pause, ramp_ms) \
	(to_pause_cnt = (from_pause / (ramp_ms ? ramp_ms : 1)) + 1)


static unsigned int pt_t[NUM_LPG_PRE_DIVIDE][NUM_CLOCKS] = {
	{	PRE_DIVIDE_1 * NSEC_1024HZ,
		PRE_DIVIDE_1 * NSEC_32768HZ,
		PRE_DIVIDE_1 * NSEC_19P2MHZ,
	},
	{	PRE_DIVIDE_3 * NSEC_1024HZ,
		PRE_DIVIDE_3 * NSEC_32768HZ,
		PRE_DIVIDE_3 * NSEC_19P2MHZ,
	},
	{	PRE_DIVIDE_5 * NSEC_1024HZ,
		PRE_DIVIDE_5 * NSEC_32768HZ,
		PRE_DIVIDE_5 * NSEC_19P2MHZ,
	},
	{	PRE_DIVIDE_6 * NSEC_1024HZ,
		PRE_DIVIDE_6 * NSEC_32768HZ,
		PRE_DIVIDE_6 * NSEC_19P2MHZ,
	},
};

static RADIX_TREE(lpg_dev_tree, GFP_KERNEL);

struct qpnp_lut_config {
	u8	*duty_pct_list;
	int	list_len;
	int	lo_index;
	int	hi_index;
	int	lut_pause_hi_cnt;
	int	lut_pause_lo_cnt;
	int	ramp_step_ms;
	bool	ramp_direction;
	bool	pattern_repeat;
	bool	ramp_toggle;
	bool	enable_pause_hi;
	bool	enable_pause_lo;
};

struct qpnp_lpg_config {
	struct qpnp_lut_config	lut_config;
	u16			base_addr;
	u16			lut_base_addr;
	u16			lut_size;
};

struct qpnp_pwm_config {
	int				channel_id;
	bool				in_use;
	const char			*lable;
	int				pwm_value;
	int				pwm_period;	
	int				pwm_duty;	
	struct pwm_period_config	period;
	int				force_pwm_size;
};

struct pwm_device {
	struct qpnp_lpg_chip	*chip;
	struct qpnp_pwm_config	pwm_config;
};

struct qpnp_lpg_chip {
	struct	spmi_device	*spmi_dev;
	struct	pwm_device	pwm_dev;
	spinlock_t		lpg_lock;
	struct	qpnp_lpg_config	lpg_config;
	u8	qpnp_lpg_registers[QPNP_TOTAL_LPG_SPMI_REGISTERS];
	enum qpnp_lpg_revision	revision;
	u8			sub_type;
	u32			flags;
};

struct qpnp_led_data {
	struct led_classdev	cdev;
	struct spmi_device	*spmi_dev;
	struct qpnp_lpg_chip	*chip;
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
	int			max_current;
	bool			default_on;
	u8			default_state;
	struct delayed_work	blink_delayed_work;
	struct delayed_work	gpled_blink_delayed_work;
	struct delayed_work 	fade_delayed_work;
	struct delayed_work	dwork;
	int			turn_off_delay_ms;
	struct delayed_work reflesh_timer;
	struct early_suspend    flt_early_suspend;
	int                     torch_mode;
	struct alarm            led_alarm;
	struct work_struct 		led_off_work;
	int			status;
};
static struct qpnp_led_data *led_for_key;
static int flag_hold_virtual_key = 0;
static int blink_for_key = 0;
static int dutys_array[8];
static int dutys_size;
static int lut_coefficient = 100;
static inline void qpnp_set_pattern_config(u8 *val,
			struct qpnp_lut_config *lut_config)
{
	*val = lut_config->enable_pause_lo & QPNP_EN_PAUSE_LO_MASK;
	*val |= (lut_config->enable_pause_hi << QPNP_EN_PAUSE_HI_SHIFT) &
						QPNP_EN_PAUSE_HI_MASK;
	*val |= (lut_config->ramp_toggle << QPNP_RAMP_TOGGLE_SHIFT) &
						QPNP_RAMP_TOGGLE_MASK;
	*val |= (lut_config->pattern_repeat << QPNP_PATTERN_REPEAT_SHIFT) &
						QPNP_PATTERN_REPEAT_MASK;
	*val |= (lut_config->ramp_direction << QPNP_RAMP_DIRECTION_SHIFT) &
						QPNP_RAMP_DIRECTION_MASK;
}

static inline void qpnp_set_pwm_type_config(u8 *val, bool glitch,
			bool full_scale, bool en_phase, bool phase)
{
	*val = phase;
	*val |= (en_phase << QPNP_EN_PHASE_STAGGER_SHIFT) &
				QPNP_EN_PHASE_STAGGER_MASK;
	*val |= (full_scale << QPNP_EN_FULL_SCALE_SHIFT) &
				QPNP_EN_FULL_SCALE_MASK;
	*val |= (glitch << QPNP_EN_GLITCH_REMOVAL_SHIFT) &
				QPNP_EN_GLITCH_REMOVAL_MASK;
}

static int qpnp_set_control(bool pwm_hi, bool pwm_lo, bool pwm_out,
					bool pwm_src, bool ramp_gen)
{
	return (ramp_gen << QPNP_PWM_EN_RAMP_GEN_SHIFT)
		| (pwm_src << QPNP_PWM_SRC_SELECT_SHIFT)
		| (pwm_out << QPNP_EN_PWM_OUTPUT_SHIFT)
		| (pwm_lo << QPNP_EN_PWM_LO_SHIFT)
		| (pwm_hi << QPNP_EN_PWM_HIGH_SHIFT);
}

#define QPNP_ENABLE_LUT_CONTROL		qpnp_set_control(0, 0, 0, 0, 1)
#define QPNP_ENABLE_PWM_CONTROL		qpnp_set_control(0, 0, 0, 1, 0)
#define QPNP_ENABLE_PWM_MODE		qpnp_set_control(1, 1, 1, 1, 0)
#define QPNP_ENABLE_PWM_MODE_GPLED_CHANNEL	qpnp_set_control(1, 1, 1, 1, 1)
#define QPNP_ENABLE_LPG_MODE		qpnp_set_control(1, 1, 1, 0, 1)
#define QPNP_DISABLE_PWM_MODE		qpnp_set_control(0, 0, 0, 1, 0)
#define QPNP_DISABLE_LPG_MODE		qpnp_set_control(0, 0, 0, 0, 1)
#define QPNP_IS_PWM_CONFIG_SELECTED(val) (val & QPNP_PWM_SRC_SELECT_MASK)

#define QPNP_ENABLE_PWM_MODE_ONLY_SUB_TYPE			0x80
#define QPNP_DISABLE_PWM_MODE_ONLY_SUB_TYPE			0x0
#define QPNP_PWM_MODE_ONLY_ENABLE_DISABLE_MASK_SUB_TYPE	0x80

static inline void qpnp_convert_to_lut_flags(int *flags,
				struct qpnp_lut_config *l_config)
{
	*flags = ((l_config->ramp_direction ? PM_PWM_LUT_RAMP_UP : 0) |
		(l_config->pattern_repeat ? PM_PWM_LUT_LOOP : 0)|
		(l_config->ramp_toggle ? PM_PWM_LUT_REVERSE : 0) |
		(l_config->enable_pause_hi ? PM_PWM_LUT_PAUSE_HI_EN : 0) |
		(l_config->enable_pause_lo ? PM_PWM_LUT_PAUSE_LO_EN : 0));
}

static inline void qpnp_set_lut_params(struct lut_params *l_params,
		struct qpnp_lut_config *l_config, int s_idx, int size)
{
	l_params->start_idx = s_idx;
	l_params->idx_len = size;
	l_params->lut_pause_hi = l_config->lut_pause_hi_cnt;
	l_params->lut_pause_lo = l_config->lut_pause_lo_cnt;
	l_params->ramp_step_ms = l_config->ramp_step_ms;
	qpnp_convert_to_lut_flags(&l_params->flags, l_config);
}

static void qpnp_lpg_save(u8 *u8p, u8 mask, u8 val)
{
	*u8p &= ~mask;
	*u8p |= val & mask;
}

static int qpnp_lpg_save_and_write(u8 value, u8 mask, u8 *reg, u16 addr,
				u16 size, struct qpnp_lpg_chip *chip)
{
	qpnp_lpg_save(reg, mask, value);

	return spmi_ext_register_writel(chip->spmi_dev->ctrl,
			chip->spmi_dev->sid, addr, reg, size);
}

static void qpnp_lpg_calc_period(enum time_level tm_lvl,
				unsigned int period_value,
				struct pwm_device *pwm)
{
	int		n, m, clk, div;
	int		best_m, best_div, best_clk;
	unsigned int	last_err, cur_err, min_err;
	unsigned int	tmp_p, period_n;
	int		id = pwm->pwm_config.channel_id;
	int		force_pwm_size = pwm->pwm_config.force_pwm_size;
	struct qpnp_lpg_chip *chip = pwm->chip;
	struct pwm_period_config *period = &pwm->pwm_config.period;

	
	if (qpnp_check_gpled_lpg_channel(id))
		n = 7;
	else
		n = 6;

	if (tm_lvl == LVL_USEC) {
		if (period_value < ((unsigned)(-1) / NSEC_PER_USEC)) {
			period_n = (period_value * NSEC_PER_USEC) >> n;
		} else {
			if (qpnp_check_gpled_lpg_channel(id))
				n = 8;
			else
				n = 9;
			period_n = (period_value >> n) * NSEC_PER_USEC;
		}
	} else {
		period_n = period_value >> n;
	}

	if (force_pwm_size != 0) {
		if (n < force_pwm_size)
			period_n = period_n >> (force_pwm_size - n);
		else
			period_n = period_n << (n - force_pwm_size);
		n = force_pwm_size;
		pr_info("LPG channel '%d' pwm size is forced to=%d\n", id, n);
	}

	min_err = last_err = (unsigned)(-1);
	best_m = 0;
	best_clk = 0;
	best_div = 0;
	for (clk = 0; clk < NUM_CLOCKS; clk++) {
		for (div = 0; div < NUM_LPG_PRE_DIVIDE; div++) {
			
			
			tmp_p = pt_t[div][clk];
			for (m = 0; m <= QPNP_PWM_M_MAX; m++) {
				if (period_n > tmp_p)
					cur_err = period_n - tmp_p;
				else
					cur_err = tmp_p - period_n;

				if (cur_err < min_err) {
					min_err = cur_err;
					best_m = m;
					best_clk = clk;
					best_div = div;
				}

				if (m && cur_err > last_err)
					
					break;

				last_err = cur_err;
				tmp_p <<= 1;
			}
		}
	}

	
	if (!force_pwm_size) {
		if (qpnp_check_gpled_lpg_channel(id)) {
			if (n == 7 && best_m >= 1) {
				n += 1;
				best_m -= 1;
			}
		} else if (n == 6) {
			if (best_m >= 3) {
				n += 3;
				best_m -= 3;
			} else if (best_m >= 1 &&
				chip->sub_type != QPNP_PWM_MODE_ONLY_SUB_TYPE) {
				n += 1;
				best_m -= 1;
			}
		}
	}
	period->pwm_size = n;
	period->clk = best_clk;
	period->pre_div = best_div;
	period->pre_div_exp = best_m;
}

static void qpnp_lpg_calc_pwm_value(struct pwm_device *pwm,
				      unsigned int period_value,
				      unsigned int duty_value)
{
	unsigned int		max_pwm_value, tmp;
	struct qpnp_pwm_config	*pwm_config = &pwm->pwm_config;

	
	tmp = 1 << (sizeof(tmp) * 8 - pwm_config->period.pwm_size);
	if (duty_value < tmp) {
		tmp = duty_value << pwm_config->period.pwm_size;
		pwm_config->pwm_value = tmp / period_value;
	} else {
		tmp = period_value >> pwm_config->period.pwm_size;
		pwm_config->pwm_value = duty_value / tmp;
	}
	max_pwm_value = (1 << pwm_config->period.pwm_size) - 1;
	if (pwm_config->pwm_value > max_pwm_value)
		pwm_config->pwm_value = max_pwm_value;
}

static int qpnp_lpg_change_table(struct pwm_device *pwm,
					int duty_pct[], int raw_value)
{
	unsigned int		pwm_value, max_pwm_value;
	struct qpnp_lpg_chip	*chip = pwm->chip;
	struct qpnp_lut_config	*lut = &chip->lpg_config.lut_config;
	int			i, pwm_size, rc = 0;
	int			burst_size = SPMI_MAX_BUF_LEN;
	int			list_len = lut->list_len << 1;
	int			offset = (lut->lo_index << 1) - 2;

	pwm_size = QPNP_GET_PWM_SIZE(
			chip->qpnp_lpg_registers[QPNP_LPG_PWM_SIZE_CLK]) +
				QPNP_MIN_PWM_BIT_SIZE;

	max_pwm_value = (1 << pwm_size) - 1;

	if (unlikely(lut->list_len != (lut->hi_index - lut->lo_index + 1))) {
		pr_err("LUT internal Data structure corruption detected\n");
		pr_err("LUT list size: %d\n", lut->list_len);
		pr_err("However, index size is: %d\n",
				(lut->hi_index - lut->lo_index + 1));
		return -EINVAL;
	}

	for (i = 0; i < lut->list_len; i++) {
		if (raw_value)
			pwm_value = duty_pct[i];
		else
			pwm_value = (duty_pct[i] << pwm_size) / 100;

		if (pwm_value > max_pwm_value)
			pwm_value = max_pwm_value;

		if (qpnp_check_gpled_lpg_channel(pwm->pwm_config.channel_id)) {
			lut->duty_pct_list[i] = pwm_value;
		} else {
			lut->duty_pct_list[i*2] = pwm_value;
			lut->duty_pct_list[(i*2)+1] = (pwm_value >>
			 QPNP_PWM_VALUE_MSB_SHIFT) & QPNP_PWM_VALUE_MSB_MASK;
		}
	}

	if (qpnp_check_gpled_lpg_channel(pwm->pwm_config.channel_id))
		offset = lut->lo_index;

	
	for (i = 0; i < list_len; i += burst_size) {
		if (i + burst_size >= list_len)
			burst_size = list_len - i;
		rc = spmi_ext_register_writel(chip->spmi_dev->ctrl,
			chip->spmi_dev->sid,
			chip->lpg_config.lut_base_addr + offset + i,
			lut->duty_pct_list + i, burst_size);
	}

	return rc;
}

static void qpnp_lpg_save_period(struct pwm_device *pwm)
{
	u8 mask, val;
	struct qpnp_lpg_chip	*chip = pwm->chip;
	struct qpnp_pwm_config	*pwm_config = &pwm->pwm_config;

	if (chip->sub_type == QPNP_PWM_MODE_ONLY_SUB_TYPE) {
		QPNP_SET_PWM_CLK_SUB_TYPE(val, pwm_config->period.clk,
				pwm_config->period.pwm_size);
		mask = QPNP_PWM_SIZE_MASK_SUB_TYPE |
				QPNP_PWM_FREQ_CLK_SELECT_MASK_SUB_TYPE;
	} else {
		QPNP_SET_PWM_CLK(val, pwm_config->period.clk,
				pwm_config->period.pwm_size);
		mask = QPNP_PWM_SIZE_MASK | QPNP_PWM_FREQ_CLK_SELECT_MASK;
	}

	qpnp_lpg_save(&chip->qpnp_lpg_registers[QPNP_LPG_PWM_SIZE_CLK],
							mask, val);

	QPNP_SET_PWM_FREQ_PREDIV(val, pwm_config->period.pre_div,
					pwm_config->period.pre_div_exp);

	mask = QPNP_PWM_FREQ_PRE_DIVIDE_MASK | QPNP_PWM_FREQ_EXP_MASK;

	qpnp_lpg_save(&chip->qpnp_lpg_registers[QPNP_LPG_PWM_FREQ_PREDIV_CLK],
								mask, val);
}

static int qpnp_lpg_save_pwm_value(struct pwm_device *pwm)
{
	unsigned int		max_pwm_value;
	int			pwm_size;
	u8			mask, value;
	struct qpnp_lpg_chip	*chip = pwm->chip;
	struct qpnp_pwm_config	*pwm_config = &pwm->pwm_config;
	struct qpnp_lpg_config	*lpg_config = &chip->lpg_config;
	int rc;
	pr_err("led workaround \n");

	if (chip->sub_type == QPNP_PWM_MODE_ONLY_SUB_TYPE)
		pwm_size = QPNP_GET_PWM_SIZE_SUB_TYPE(
			chip->qpnp_lpg_registers[QPNP_LPG_PWM_SIZE_CLK]) ?
				QPNP_MAX_PWM_BIT_SIZE : QPNP_MIN_PWM_BIT_SIZE;
	else
		pwm_size = QPNP_GET_PWM_SIZE(
			chip->qpnp_lpg_registers[QPNP_LPG_PWM_SIZE_CLK]) +
				QPNP_MIN_PWM_BIT_SIZE;

	max_pwm_value = (1 << pwm_size) - 1;

	if (pwm_config->pwm_value > max_pwm_value)
		pwm_config->pwm_value = max_pwm_value;

	value = pwm_config->pwm_value;
	mask = QPNP_PWM_VALUE_LSB_MASK;

	rc = qpnp_lpg_save_and_write(value, mask,
			&pwm->chip->qpnp_lpg_registers[QPNP_PWM_VALUE_LSB],
			SPMI_LPG_REG_ADDR(lpg_config->base_addr,
			QPNP_PWM_VALUE_LSB), 1, chip);
	if (rc)
		return rc;

	value = (pwm_config->pwm_value >> QPNP_PWM_VALUE_MSB_SHIFT) &
					QPNP_PWM_VALUE_MSB_MASK;

	mask = QPNP_PWM_VALUE_MSB_MASK;

	rc = qpnp_lpg_save_and_write(value, mask,
			&pwm->chip->qpnp_lpg_registers[QPNP_PWM_VALUE_MSB],
			SPMI_LPG_REG_ADDR(lpg_config->base_addr,
			QPNP_PWM_VALUE_MSB), 1, chip);
	if (rc)
		return rc;

	if (chip->sub_type == QPNP_PWM_MODE_ONLY_SUB_TYPE) {
		value = QPNP_PWM_SYNC_VALUE & QPNP_PWM_SYNC_MASK;
		rc = spmi_ext_register_writel(chip->spmi_dev->ctrl,
			chip->spmi_dev->sid,
			SPMI_LPG_REG_ADDR(lpg_config->base_addr,
			SPMI_LPG_PWM_SYNC), &value, 1);
	}

	return rc;
}

static int qpnp_lpg_configure_pattern(struct pwm_device *pwm)
{
	struct qpnp_lpg_config	*lpg_config = &pwm->chip->lpg_config;
	struct qpnp_lut_config	*lut_config = &lpg_config->lut_config;
	struct qpnp_lpg_chip	*chip = pwm->chip;
	u8			value, mask;

	qpnp_set_pattern_config(&value, lut_config);

	mask = QPNP_RAMP_DIRECTION_MASK | QPNP_PATTERN_REPEAT_MASK |
			QPNP_RAMP_TOGGLE_MASK | QPNP_EN_PAUSE_HI_MASK |
			QPNP_EN_PAUSE_LO_MASK;

	return qpnp_lpg_save_and_write(value, mask,
		&pwm->chip->qpnp_lpg_registers[QPNP_LPG_PATTERN_CONFIG],
		SPMI_LPG_REG_ADDR(lpg_config->base_addr,
		QPNP_LPG_PATTERN_CONFIG), 1, chip);
}

static int qpnp_lpg_configure_pwm(struct pwm_device *pwm)
{
	struct qpnp_lpg_config	*lpg_config = &pwm->chip->lpg_config;
	struct qpnp_lpg_chip	*chip = pwm->chip;
	int			rc;
	u8			value, mask;

	rc = spmi_ext_register_writel(chip->spmi_dev->ctrl, chip->spmi_dev->sid,
		SPMI_LPG_REG_ADDR(lpg_config->base_addr, QPNP_LPG_PWM_SIZE_CLK),
		&chip->qpnp_lpg_registers[QPNP_LPG_PWM_SIZE_CLK], 1);

	if (rc)
		return rc;

	rc = spmi_ext_register_writel(chip->spmi_dev->ctrl, chip->spmi_dev->sid,
		SPMI_LPG_REG_ADDR(lpg_config->base_addr,
		QPNP_LPG_PWM_FREQ_PREDIV_CLK),
		&chip->qpnp_lpg_registers[QPNP_LPG_PWM_FREQ_PREDIV_CLK], 1);
	if (rc)
		return rc;

	qpnp_set_pwm_type_config(&value, 1, 0, 0, 0);

	mask = QPNP_EN_GLITCH_REMOVAL_MASK | QPNP_EN_FULL_SCALE_MASK |
			QPNP_EN_PHASE_STAGGER_MASK | QPNP_PHASE_STAGGER_MASK;

	return qpnp_lpg_save_and_write(value, mask,
		&pwm->chip->qpnp_lpg_registers[QPNP_LPG_PWM_TYPE_CONFIG],
		SPMI_LPG_REG_ADDR(lpg_config->base_addr,
		QPNP_LPG_PWM_TYPE_CONFIG), 1, chip);
}

static int qpnp_configure_pwm_control(struct pwm_device *pwm)
{
	struct qpnp_lpg_config	*lpg_config = &pwm->chip->lpg_config;
	struct qpnp_lpg_chip	*chip = pwm->chip;
	u8			value, mask;

	if (chip->sub_type == QPNP_PWM_MODE_ONLY_SUB_TYPE)
		return 0;

	value = QPNP_ENABLE_PWM_CONTROL;

	mask = QPNP_EN_PWM_HIGH_MASK | QPNP_EN_PWM_LO_MASK |
		QPNP_EN_PWM_OUTPUT_MASK | QPNP_PWM_SRC_SELECT_MASK |
					QPNP_PWM_EN_RAMP_GEN_MASK;

	return qpnp_lpg_save_and_write(value, mask,
		&pwm->chip->qpnp_lpg_registers[QPNP_ENABLE_CONTROL],
		SPMI_LPG_REG_ADDR(lpg_config->base_addr,
		QPNP_ENABLE_CONTROL), 1, chip);

}

static int qpnp_configure_lpg_control(struct pwm_device *pwm)
{
	struct qpnp_lpg_config	*lpg_config = &pwm->chip->lpg_config;
	struct qpnp_lpg_chip	*chip = pwm->chip;
	u8			value, mask;

	value = QPNP_ENABLE_LUT_CONTROL;

	mask = QPNP_EN_PWM_HIGH_MASK | QPNP_EN_PWM_LO_MASK |
		QPNP_EN_PWM_OUTPUT_MASK | QPNP_PWM_SRC_SELECT_MASK |
				QPNP_PWM_EN_RAMP_GEN_MASK;

	return qpnp_lpg_save_and_write(value, mask,
		&pwm->chip->qpnp_lpg_registers[QPNP_ENABLE_CONTROL],
		SPMI_LPG_REG_ADDR(lpg_config->base_addr,
		QPNP_ENABLE_CONTROL), 1, chip);

}

static int qpnp_lpg_configure_ramp_step_duration(struct pwm_device *pwm)
{
	struct qpnp_lpg_config	*lpg_config = &pwm->chip->lpg_config;
	struct qpnp_lut_config	lut_config = lpg_config->lut_config;
	struct qpnp_lpg_chip	*chip = pwm->chip;
	int			rc, value;
	u8			val, mask;

	value = QPNP_GET_RAMP_STEP_DURATION(lut_config.ramp_step_ms);
	val = value & QPNP_RAMP_STEP_DURATION_LSB_MASK;
	mask = QPNP_RAMP_STEP_DURATION_LSB_MASK;

	rc = qpnp_lpg_save_and_write(val, mask,
		&pwm->chip->qpnp_lpg_registers[QPNP_RAMP_STEP_DURATION_LSB],
		SPMI_LPG_REG_ADDR(lpg_config->base_addr,
		QPNP_RAMP_STEP_DURATION_LSB), 1, chip);
	if (rc)
		return rc;

	val = (value >> QPNP_RAMP_STEP_DURATION_MSB_SHIFT) &
				QPNP_RAMP_STEP_DURATION_MSB_MASK;

	mask = QPNP_RAMP_STEP_DURATION_MSB_MASK;

	return qpnp_lpg_save_and_write(val, mask,
		&pwm->chip->qpnp_lpg_registers[QPNP_RAMP_STEP_DURATION_MSB],
		SPMI_LPG_REG_ADDR(lpg_config->base_addr,
		QPNP_RAMP_STEP_DURATION_MSB), 1, chip);
}

static int qpnp_lpg_configure_pause(struct pwm_device *pwm)
{
	struct qpnp_lpg_config	*lpg_config = &pwm->chip->lpg_config;
	struct qpnp_lut_config	lut_config = lpg_config->lut_config;
	struct qpnp_lpg_chip	*chip = pwm->chip;
	u8			value, mask;
	int			rc = 0;

	if (lut_config.enable_pause_hi) {
		value = lut_config.lut_pause_hi_cnt;
		mask = QPNP_PAUSE_HI_MULTIPLIER_LSB_MASK;

		rc = qpnp_lpg_save_and_write(value, mask,
		&pwm->chip->qpnp_lpg_registers[QPNP_PAUSE_HI_MULTIPLIER_LSB],
		SPMI_LPG_REG_ADDR(lpg_config->base_addr,
		QPNP_PAUSE_HI_MULTIPLIER_LSB), 1, chip);
		if (rc)
			return rc;

		value = (lut_config.lut_pause_hi_cnt >>
			QPNP_PAUSE_HI_MULTIPLIER_MSB_SHIFT) &
					QPNP_PAUSE_HI_MULTIPLIER_MSB_MASK;

		mask = QPNP_PAUSE_HI_MULTIPLIER_MSB_MASK;

		rc = qpnp_lpg_save_and_write(value, mask,
		&pwm->chip->qpnp_lpg_registers[QPNP_PAUSE_HI_MULTIPLIER_MSB],
		SPMI_LPG_REG_ADDR(lpg_config->base_addr,
		QPNP_PAUSE_HI_MULTIPLIER_MSB), 1, chip);
	} else {
		value = 0;
		mask = QPNP_PAUSE_HI_MULTIPLIER_LSB_MASK;

		rc = qpnp_lpg_save_and_write(value, mask,
		&pwm->chip->qpnp_lpg_registers[QPNP_PAUSE_HI_MULTIPLIER_LSB],
		SPMI_LPG_REG_ADDR(lpg_config->base_addr,
		QPNP_PAUSE_HI_MULTIPLIER_LSB), 1, chip);
		if (rc)
			return rc;

		mask = QPNP_PAUSE_HI_MULTIPLIER_MSB_MASK;

		rc = qpnp_lpg_save_and_write(value, mask,
		&pwm->chip->qpnp_lpg_registers[QPNP_PAUSE_HI_MULTIPLIER_MSB],
		SPMI_LPG_REG_ADDR(lpg_config->base_addr,
		QPNP_PAUSE_HI_MULTIPLIER_MSB), 1, chip);
		if (rc)
			return rc;

	}

	if (lut_config.enable_pause_lo) {
		value = lut_config.lut_pause_lo_cnt;
		mask = QPNP_PAUSE_LO_MULTIPLIER_LSB_MASK;

		rc = qpnp_lpg_save_and_write(value, mask,
		&pwm->chip->qpnp_lpg_registers[QPNP_PAUSE_LO_MULTIPLIER_LSB],
		SPMI_LPG_REG_ADDR(lpg_config->base_addr,
		QPNP_PAUSE_LO_MULTIPLIER_LSB), 1, chip);
		if (rc)
			return rc;

		value = (lut_config.lut_pause_lo_cnt >>
				QPNP_PAUSE_LO_MULTIPLIER_MSB_SHIFT) &
					QPNP_PAUSE_LO_MULTIPLIER_MSB_MASK;

		mask = QPNP_PAUSE_LO_MULTIPLIER_MSB_MASK;

		rc = qpnp_lpg_save_and_write(value, mask,
		&pwm->chip->qpnp_lpg_registers[QPNP_PAUSE_LO_MULTIPLIER_MSB],
		SPMI_LPG_REG_ADDR(lpg_config->base_addr,
		QPNP_PAUSE_LO_MULTIPLIER_MSB), 1, chip);
	} else {
		value = 0;
		mask = QPNP_PAUSE_LO_MULTIPLIER_LSB_MASK;

		rc = qpnp_lpg_save_and_write(value, mask,
		&pwm->chip->qpnp_lpg_registers[QPNP_PAUSE_LO_MULTIPLIER_LSB],
		SPMI_LPG_REG_ADDR(lpg_config->base_addr,
		QPNP_PAUSE_LO_MULTIPLIER_LSB), 1, chip);
		if (rc)
			return rc;

		mask = QPNP_PAUSE_LO_MULTIPLIER_MSB_MASK;

		rc = qpnp_lpg_save_and_write(value, mask,
		&pwm->chip->qpnp_lpg_registers[QPNP_PAUSE_LO_MULTIPLIER_MSB],
		SPMI_LPG_REG_ADDR(lpg_config->base_addr,
		QPNP_PAUSE_LO_MULTIPLIER_MSB), 1, chip);
		return rc;
	}

	return rc;
}

static int qpnp_lpg_configure_index(struct pwm_device *pwm)
{
	struct qpnp_lpg_config	*lpg_config = &pwm->chip->lpg_config;
	struct qpnp_lut_config	lut_config = lpg_config->lut_config;
	struct qpnp_lpg_chip	*chip = pwm->chip;
	u8			value, mask;
	int			rc = 0;

	value = lut_config.hi_index;
	mask = QPNP_HI_INDEX_MASK;

	rc = qpnp_lpg_save_and_write(value, mask,
		&pwm->chip->qpnp_lpg_registers[QPNP_HI_INDEX],
		SPMI_LPG_REG_ADDR(lpg_config->base_addr,
		QPNP_HI_INDEX), 1, chip);
	if (rc)
		return rc;

	value = lut_config.lo_index;
	mask = QPNP_LO_INDEX_MASK;

	rc = qpnp_lpg_save_and_write(value, mask,
		&pwm->chip->qpnp_lpg_registers[QPNP_LO_INDEX],
		SPMI_LPG_REG_ADDR(lpg_config->base_addr,
		QPNP_LO_INDEX), 1, chip);

	return rc;
}

static int qpnp_lpg_change_lut(struct pwm_device *pwm)
{
	int	rc;

	rc = qpnp_lpg_configure_pattern(pwm);
	if (rc) {
		pr_err("Failed to configure LUT pattern");
		return rc;
	}
	rc = qpnp_lpg_configure_pwm(pwm);
	if (rc) {
		pr_err("Failed to configure LUT pattern");
		return rc;
	}
	rc = qpnp_configure_lpg_control(pwm);
	if (rc) {
		pr_err("Failed to configure pause registers");
		return rc;
	}
	rc = qpnp_lpg_configure_ramp_step_duration(pwm);
	if (rc) {
		pr_err("Failed to configure duty time");
		return rc;
	}
	rc = qpnp_lpg_configure_pause(pwm);
	if (rc) {
		pr_err("Failed to configure pause registers");
		return rc;
	}
	rc = qpnp_lpg_configure_index(pwm);
	if (rc) {
		pr_err("Failed to configure index registers");
		return rc;
	}
	return rc;
}

static int qpnp_lpg_configure_lut_state(struct pwm_device *pwm,
				enum qpnp_lut_state state)
{
	struct qpnp_lpg_config	*lpg_config = &pwm->chip->lpg_config;
	struct qpnp_lpg_chip	*chip = pwm->chip;
	u8			value1, value2, mask1, mask2;
	u8			*reg1, *reg2;
	u16			addr, addr1;
	int			rc;

	value1 = pwm->chip->qpnp_lpg_registers[QPNP_RAMP_CONTROL];
	reg1 = &pwm->chip->qpnp_lpg_registers[QPNP_RAMP_CONTROL];
	reg2 = &pwm->chip->qpnp_lpg_registers[QPNP_ENABLE_CONTROL];
	mask2 = QPNP_EN_PWM_HIGH_MASK | QPNP_EN_PWM_LO_MASK |
		QPNP_EN_PWM_OUTPUT_MASK | QPNP_PWM_SRC_SELECT_MASK |
					QPNP_PWM_EN_RAMP_GEN_MASK;

	switch (chip->revision) {
	case QPNP_LPG_REVISION_0:
		if (state == QPNP_LUT_ENABLE) {
			QPNP_ENABLE_LUT_V0(value1);
			value2 = QPNP_ENABLE_LPG_MODE;
		} else {
			QPNP_DISABLE_LUT_V0(value1);
			value2 = QPNP_DISABLE_LPG_MODE;
		}
		mask1 = QPNP_RAMP_START_MASK;
		addr1 = SPMI_LPG_REG_ADDR(lpg_config->base_addr,
					QPNP_RAMP_CONTROL);
		break;
	case QPNP_LPG_REVISION_1:
		if (state == QPNP_LUT_ENABLE) {
			QPNP_ENABLE_LUT_V1(value1, pwm->pwm_config.channel_id);
			value2 = QPNP_ENABLE_LPG_MODE;
		} else {
			value2 = QPNP_DISABLE_LPG_MODE;
		}
		mask1 = value1;
		addr1 = lpg_config->lut_base_addr +
			SPMI_LPG_REV1_RAMP_CONTROL_OFFSET;
		break;
	default:
		pr_err("Invalid LPG revision\n");
		return -EINVAL;
	}

	addr = SPMI_LPG_REG_ADDR(lpg_config->base_addr,
				QPNP_ENABLE_CONTROL);

	rc = qpnp_lpg_save_and_write(value2, mask2, reg2,
					addr, 1, chip);
	if (rc)
		return rc;

	if (state == QPNP_LUT_ENABLE || chip->revision == QPNP_LPG_REVISION_0)
		rc = qpnp_lpg_save_and_write(value1, mask1, reg1,
					addr1, 1, chip);
	return rc;
}

static inline int qpnp_enable_pwm_mode(struct qpnp_pwm_config *pwm_conf)
{
	if (qpnp_check_gpled_lpg_channel(pwm_conf->channel_id))
		return QPNP_ENABLE_PWM_MODE_GPLED_CHANNEL;
	return QPNP_ENABLE_PWM_MODE;
}

static int qpnp_lpg_configure_pwm_state(struct pwm_device *pwm,
					enum qpnp_pwm_state state)
{
	struct qpnp_lpg_config	*lpg_config = &pwm->chip->lpg_config;
	struct qpnp_lpg_chip	*chip = pwm->chip;
	u8			value, mask;
	int			rc;

	if (chip->sub_type == QPNP_PWM_MODE_ONLY_SUB_TYPE) {
		if (state == QPNP_PWM_ENABLE)
			value = QPNP_ENABLE_PWM_MODE_ONLY_SUB_TYPE;
		else
			value = QPNP_DISABLE_PWM_MODE_ONLY_SUB_TYPE;

		mask = QPNP_PWM_MODE_ONLY_ENABLE_DISABLE_MASK_SUB_TYPE;
	} else {
		if (state == QPNP_PWM_ENABLE)
			value = qpnp_enable_pwm_mode(&pwm->pwm_config);
		else
			value = QPNP_DISABLE_PWM_MODE;

		mask = QPNP_EN_PWM_HIGH_MASK | QPNP_EN_PWM_LO_MASK |
			QPNP_EN_PWM_OUTPUT_MASK | QPNP_PWM_SRC_SELECT_MASK |
				QPNP_PWM_EN_RAMP_GEN_MASK;
	}


	rc = qpnp_lpg_save_and_write(value, mask,
		&pwm->chip->qpnp_lpg_registers[QPNP_ENABLE_CONTROL],
		SPMI_LPG_REG_ADDR(lpg_config->base_addr,
		QPNP_ENABLE_CONTROL), 1, chip);
	if (rc)
		goto out;

	if (state == QPNP_PWM_ENABLE)
		return qpnp_lpg_save_pwm_value(pwm);

out:
	return rc;
}

static int _pwm_config(struct pwm_device *pwm,
				enum time_level tm_lvl,
				int duty_value, int period_value)
{
	struct qpnp_pwm_config		*pwm_config;
	struct qpnp_lpg_chip		*chip;
	struct pwm_period_config	*period;
	int period_us, duty_us;
	int	rc;

	chip = pwm->chip;
	pwm_config = &pwm->pwm_config;
	period = &pwm_config->period;

	if (tm_lvl == LVL_USEC) {
		period_us = period_value;
		duty_us = duty_value;
	} else {
		period_us = period_value / NSEC_PER_USEC;
		duty_us = duty_value / NSEC_PER_USEC;
	}

	if (pwm_config->pwm_period != period_us) {
		qpnp_lpg_calc_period(tm_lvl, period_value, pwm);
		qpnp_lpg_save_period(pwm);
		pwm_config->pwm_period = period_us;
	}

	pwm_config->pwm_duty = duty_us;
	qpnp_lpg_calc_pwm_value(pwm, period_value, duty_value);
	rc = qpnp_lpg_save_pwm_value(pwm);

	if (rc) {
		pr_err("Could not update PWM value for channel %d rc=%d\n",
						pwm_config->channel_id, rc);
		return rc;
	}

	rc = qpnp_lpg_configure_pwm(pwm);
	if (rc) {
		pr_err("Could not configure PWM clock for\n");
		pr_err("channel %d rc=%d\n", pwm_config->channel_id, rc);
		return rc;
	}

	rc = qpnp_configure_pwm_control(pwm);
	if (rc) {
		pr_err("Could not update PWM control for");
		pr_err("channel %d rc=%d\n", pwm_config->channel_id, rc);
		return rc;
	}

	pr_debug("duty/period=%u/%u %s: pwm_value=%d (of %d)\n",
		 (unsigned)duty_us, (unsigned)period_us,
		 (tm_lvl == LVL_USEC) ? "usec" : "nsec",
		 pwm_config->pwm_value, 1 << period->pwm_size);

	return 0;
}

static int _pwm_lut_config(struct pwm_device *pwm, int period_us,
			int duty_pct[], struct lut_params lut_params)
{
	struct qpnp_lpg_config		*lpg_config;
	struct qpnp_lut_config		*lut_config;
	struct pwm_period_config	*period;
	struct qpnp_pwm_config		*pwm_config;
	int				start_idx = lut_params.start_idx;
	int				len = lut_params.idx_len;
	int				flags = lut_params.flags;
	int				raw_lut, ramp_step_ms;
	int				rc = 0;

	pwm_config = &pwm->pwm_config;
	lpg_config = &pwm->chip->lpg_config;
	lut_config = &lpg_config->lut_config;

	period = &pwm_config->period;

	if (pwm_config->pwm_period != period_us) {
		qpnp_lpg_calc_period(LVL_USEC, period_us, pwm);
		qpnp_lpg_save_period(pwm);
		pwm_config->pwm_period = period_us;
	}

	if (flags & PM_PWM_LUT_NO_TABLE)
		goto after_table_write;

	raw_lut = 0;
	if (flags & PM_PWM_LUT_USE_RAW_VALUE)
		raw_lut = 1;

	lut_config->list_len = len;
	lut_config->lo_index = start_idx + 1;
	lut_config->hi_index = start_idx + len;

	rc = qpnp_lpg_change_table(pwm, duty_pct, raw_lut);
	if (rc) {
		pr_err("qpnp_lpg_change_table: rc=%d\n", rc);
		return -EINVAL;
	}

after_table_write:
	ramp_step_ms = lut_params.ramp_step_ms;

	if (ramp_step_ms > PM_PWM_LUT_RAMP_STEP_TIME_MAX)
		ramp_step_ms = PM_PWM_LUT_RAMP_STEP_TIME_MAX;

	QPNP_SET_PAUSE_CNT(lut_config->lut_pause_lo_cnt,
			lut_params.lut_pause_lo, ramp_step_ms);
	if (lut_config->lut_pause_lo_cnt > PM_PWM_MAX_PAUSE_CNT)
		lut_config->lut_pause_lo_cnt = PM_PWM_MAX_PAUSE_CNT;

	QPNP_SET_PAUSE_CNT(lut_config->lut_pause_hi_cnt,
			lut_params.lut_pause_hi, ramp_step_ms);
	if (lut_config->lut_pause_hi_cnt > PM_PWM_MAX_PAUSE_CNT)
			lut_config->lut_pause_hi_cnt = PM_PWM_MAX_PAUSE_CNT;

	lut_config->ramp_step_ms = ramp_step_ms;

	lut_config->ramp_direction  = !!(flags & PM_PWM_LUT_RAMP_UP);
	lut_config->pattern_repeat  = !!(flags & PM_PWM_LUT_LOOP);
	lut_config->ramp_toggle	    = !!(flags & PM_PWM_LUT_REVERSE);
	lut_config->enable_pause_hi = !!(flags & PM_PWM_LUT_PAUSE_HI_EN);
	lut_config->enable_pause_lo = !!(flags & PM_PWM_LUT_PAUSE_LO_EN);

	rc = qpnp_lpg_change_lut(pwm);

	return rc;
}

static int _pwm_enable(struct pwm_device *pwm)
{
	int rc = 0;
	struct qpnp_lpg_chip *chip;
	unsigned long flags;

	chip = pwm->chip;

	spin_lock_irqsave(&pwm->chip->lpg_lock, flags);

	if (QPNP_IS_PWM_CONFIG_SELECTED(
		chip->qpnp_lpg_registers[QPNP_ENABLE_CONTROL]) ||
			chip->flags & QPNP_PWM_LUT_NOT_SUPPORTED) {
		rc = qpnp_lpg_configure_pwm_state(pwm, QPNP_PWM_ENABLE);
	} else if (!(chip->flags & QPNP_PWM_LUT_NOT_SUPPORTED)) {
			rc = qpnp_lpg_configure_lut_state(pwm, QPNP_LUT_ENABLE);
	}

	spin_unlock_irqrestore(&pwm->chip->lpg_lock, flags);

	if (rc)
		pr_err("Failed to enable PWM channel: %d\n",
				pwm->pwm_config.channel_id);

	return rc;
}

struct pwm_device *pwm_request(int pwm_id, const char *lable)
{
	struct qpnp_lpg_chip	*chip;
	struct pwm_device	*pwm;
	unsigned long		flags;

	chip = radix_tree_lookup(&lpg_dev_tree, pwm_id);

	if (!chip) {
		pr_err("Could not find PWM Device for the\n");
		pr_err("input pwm channel %d\n", pwm_id);
		return ERR_PTR(-EINVAL);
	}

	spin_lock_irqsave(&chip->lpg_lock, flags);

	pwm = &chip->pwm_dev;

	if (pwm->pwm_config.in_use) {
		pr_err("PWM device associated with the");
		pr_err("input pwm id: %d is in use by %s",
			pwm_id, pwm->pwm_config.lable);
		pwm = ERR_PTR(-EBUSY);
	} else {
		pwm->pwm_config.in_use = 1;
		pwm->pwm_config.lable  = lable;
	}

	spin_unlock_irqrestore(&chip->lpg_lock, flags);

	return pwm;
}
EXPORT_SYMBOL_GPL(pwm_request);

void pwm_free(struct pwm_device *pwm)
{
	struct qpnp_pwm_config	*pwm_config;
	unsigned long		flags;

	if (pwm == NULL || IS_ERR(pwm) || pwm->chip == NULL) {
		pr_err("Invalid pwm handle or no pwm_chip\n");
		return;
	}

	spin_lock_irqsave(&pwm->chip->lpg_lock, flags);

	pwm_config = &pwm->pwm_config;

	if (pwm_config->in_use) {
		qpnp_lpg_configure_pwm_state(pwm, QPNP_PWM_DISABLE);
		if (!(pwm->chip->flags & QPNP_PWM_LUT_NOT_SUPPORTED))
			qpnp_lpg_configure_lut_state(pwm, QPNP_LUT_DISABLE);
		pwm_config->in_use = 0;
		pwm_config->lable = NULL;
	}

	spin_unlock_irqrestore(&pwm->chip->lpg_lock, flags);
}
EXPORT_SYMBOL_GPL(pwm_free);

int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	int rc;
	unsigned long flags;

	if (pwm == NULL || IS_ERR(pwm) || duty_ns > period_ns ||
		(unsigned)period_ns < PM_PWM_PERIOD_MIN * NSEC_PER_USEC) {
		pr_err("Invalid pwm handle or parameters\n");
		return -EINVAL;
	}

	if (!pwm->pwm_config.in_use)
		return -EINVAL;

	spin_lock_irqsave(&pwm->chip->lpg_lock, flags);
	rc = _pwm_config(pwm, LVL_NSEC, duty_ns, period_ns);
	spin_unlock_irqrestore(&pwm->chip->lpg_lock, flags);

	if (rc)
		pr_err("Failed to configure PWM mode\n");

	return rc;
}
EXPORT_SYMBOL_GPL(pwm_config);

int pwm_config_us(struct pwm_device *pwm, int duty_us, int period_us)
{
	int rc;
	unsigned long flags;

	if (pwm == NULL || IS_ERR(pwm) ||
		duty_us > period_us ||
		(unsigned)period_us > PM_PWM_PERIOD_MAX ||
		(unsigned)period_us < PM_PWM_PERIOD_MIN) {
		pr_err("Invalid pwm handle or parameters\n");
		return -EINVAL;
	}

	if (!pwm->pwm_config.in_use)
		return -EINVAL;

	spin_lock_irqsave(&pwm->chip->lpg_lock, flags);
	rc = _pwm_config(pwm, LVL_USEC, duty_us, period_us);
	spin_unlock_irqrestore(&pwm->chip->lpg_lock, flags);

	if (rc)
		pr_err("Failed to configure PWM mode\n");

	return rc;
}
EXPORT_SYMBOL_GPL(pwm_config_us);

int pwm_enable(struct pwm_device *pwm)
{
	struct qpnp_pwm_config	*p_config;

	if (pwm == NULL || IS_ERR(pwm) || pwm->chip == NULL) {
		pr_err("Invalid pwm handle or no pwm_chip\n");
		return -EINVAL;
	}

	p_config = &pwm->pwm_config;

	if (!p_config->in_use) {
		pr_err("channel_id: %d: stale handle?\n", p_config->channel_id);
		return -EINVAL;
	}

	return _pwm_enable(pwm);
}
EXPORT_SYMBOL_GPL(pwm_enable);

void pwm_disable(struct pwm_device *pwm)
{
	struct qpnp_pwm_config	*pwm_config;
	struct qpnp_lpg_chip	*chip;
	unsigned long		flags;
	int rc = 0;

	if (pwm == NULL || IS_ERR(pwm) || pwm->chip == NULL) {
		pr_err("Invalid pwm handle or no pwm_chip\n");
		return;
	}

	spin_lock_irqsave(&pwm->chip->lpg_lock, flags);

	chip = pwm->chip;
	pwm_config = &pwm->pwm_config;

	if (pwm_config->in_use) {
		if (QPNP_IS_PWM_CONFIG_SELECTED(
			chip->qpnp_lpg_registers[QPNP_ENABLE_CONTROL]) ||
				chip->flags & QPNP_PWM_LUT_NOT_SUPPORTED) {
			rc = qpnp_lpg_configure_pwm_state(pwm,
						QPNP_PWM_DISABLE);
		} else if (!(chip->flags & QPNP_PWM_LUT_NOT_SUPPORTED)) {
				rc = qpnp_lpg_configure_lut_state(pwm,
							QPNP_LUT_DISABLE);
		}
	}

	spin_unlock_irqrestore(&pwm->chip->lpg_lock, flags);

	if (rc)
		pr_err("Failed to disable PWM channel: %d\n",
					pwm_config->channel_id);
}
EXPORT_SYMBOL_GPL(pwm_disable);

int pwm_change_mode(struct pwm_device *pwm, enum pm_pwm_mode mode)
{
	int rc;
	unsigned long flags;

	if (pwm == NULL || IS_ERR(pwm) || pwm->chip == NULL) {
		pr_err("Invalid pwm handle or no pwm_chip\n");
		return -EINVAL;
	}

	if (mode < PM_PWM_MODE_PWM || mode > PM_PWM_MODE_LPG) {
		pr_err("Invalid mode value\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&pwm->chip->lpg_lock, flags);

	if (mode)
		rc = qpnp_configure_lpg_control(pwm);
	else
		rc = qpnp_configure_pwm_control(pwm);

	spin_unlock_irqrestore(&pwm->chip->lpg_lock, flags);

	if (rc)
		pr_err("Failed to change the mode\n");
	return rc;
}
EXPORT_SYMBOL_GPL(pwm_change_mode);

int pwm_config_period(struct pwm_device *pwm,
			     struct pwm_period_config *period)
{
	struct qpnp_pwm_config	*pwm_config;
	struct qpnp_lpg_config	*lpg_config;
	struct qpnp_lpg_chip	*chip;
	unsigned long		flags;
	int			rc = 0;

	if (pwm == NULL || IS_ERR(pwm) || period == NULL)
		return -EINVAL;
	if (pwm->chip == NULL)
		return -ENODEV;

	spin_lock_irqsave(&pwm->chip->lpg_lock, flags);

	chip = pwm->chip;
	pwm_config = &pwm->pwm_config;
	lpg_config = &chip->lpg_config;

	if (!pwm_config->in_use) {
		rc = -EINVAL;
		goto out_unlock;
	}

	pwm_config->period.pwm_size = period->pwm_size;
	pwm_config->period.clk = period->clk;
	pwm_config->period.pre_div = period->pre_div;
	pwm_config->period.pre_div_exp = period->pre_div_exp;

	qpnp_lpg_save_period(pwm);

	rc = spmi_ext_register_writel(chip->spmi_dev->ctrl, chip->spmi_dev->sid,
			SPMI_LPG_REG_ADDR(lpg_config->base_addr,
			QPNP_LPG_PWM_SIZE_CLK),
			&chip->qpnp_lpg_registers[QPNP_LPG_PWM_SIZE_CLK], 1);

	if (rc) {
		pr_err("Write failed: QPNP_LPG_PWM_SIZE_CLK register, rc: %d\n",
									rc);
		goto out_unlock;
	}

	rc = spmi_ext_register_writel(chip->spmi_dev->ctrl, chip->spmi_dev->sid,
			SPMI_LPG_REG_ADDR(lpg_config->base_addr,
			QPNP_LPG_PWM_FREQ_PREDIV_CLK),
		&chip->qpnp_lpg_registers[QPNP_LPG_PWM_FREQ_PREDIV_CLK], 1);
	if (rc) {
		pr_err("Failed to write to QPNP_LPG_PWM_FREQ_PREDIV_CLK\n");
		pr_err("register, rc = %d\n", rc);
	}

out_unlock:
	spin_unlock_irqrestore(&pwm->chip->lpg_lock, flags);
	return rc;
}
EXPORT_SYMBOL(pwm_config_period);

int pwm_config_pwm_value(struct pwm_device *pwm, int pwm_value)
{
	struct qpnp_lpg_config	*lpg_config;
	struct qpnp_pwm_config	*pwm_config;
	unsigned long		flags;
	int			rc = 0;

	if (pwm == NULL || IS_ERR(pwm)) {
		pr_err("Invalid parameter passed\n");
		return -EINVAL;
	}

	if (pwm->chip == NULL) {
		pr_err("Invalid device handle\n");
		return -ENODEV;
	}

	lpg_config = &pwm->chip->lpg_config;
	pwm_config = &pwm->pwm_config;

	spin_lock_irqsave(&pwm->chip->lpg_lock, flags);

	if (!pwm_config->in_use || !pwm_config->pwm_period) {
		rc = -EINVAL;
		pr_err("PWM channel isn't in use or period value missing\n");
		goto out_unlock;
	}

	if (pwm_config->pwm_value == pwm_value)
		goto out_unlock;

	pwm_config->pwm_value = pwm_value;

	rc = qpnp_lpg_save_pwm_value(pwm);

	if (rc)
		pr_err("Could not update PWM value for channel %d rc=%d\n",
						pwm_config->channel_id, rc);

out_unlock:
	spin_unlock_irqrestore(&pwm->chip->lpg_lock, flags);
	return rc;
}
EXPORT_SYMBOL_GPL(pwm_config_pwm_value);

int pwm_lut_config(struct pwm_device *pwm, int period_us,
		int duty_pct[], struct lut_params lut_params)
{
	unsigned long flags;
	int rc = 0;

	if (pwm == NULL || IS_ERR(pwm) || !lut_params.idx_len) {
		pr_err("Invalid pwm handle or idx_len=0\n");
		return -EINVAL;
	}

	if (pwm->chip == NULL)
		return -ENODEV;

	if (pwm->chip->flags & QPNP_PWM_LUT_NOT_SUPPORTED) {
		pr_err("LUT mode isn't supported\n");
		return -EINVAL;
	}

	if (!pwm->pwm_config.in_use) {
		pr_err("channel_id: %d: stale handle?\n",
				pwm->pwm_config.channel_id);
		return -EINVAL;
	}

	if (duty_pct == NULL && !(lut_params.flags & PM_PWM_LUT_NO_TABLE)) {
		pr_err("Invalid duty_pct with flag\n");
		return -EINVAL;
	}

	if ((lut_params.start_idx + lut_params.idx_len) >
				pwm->chip->lpg_config.lut_size) {
		pr_err("Exceed LUT limit\n");
		return -EINVAL;
	}

	if ((unsigned)period_us > PM_PWM_PERIOD_MAX ||
		(unsigned)period_us < PM_PWM_PERIOD_MIN) {
		pr_err("Period out of range\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&pwm->chip->lpg_lock, flags);

	rc = _pwm_lut_config(pwm, period_us, duty_pct, lut_params);

	spin_unlock_irqrestore(&pwm->chip->lpg_lock, flags);

	if (rc)
		pr_err("Failed to configure LUT\n");

	return rc;
}
EXPORT_SYMBOL_GPL(pwm_lut_config);

static int qpnp_parse_pwm_dt_config(struct device_node *of_pwm_node,
		struct device_node *of_parent, struct qpnp_lpg_chip *chip)
{
	int rc, period;
	struct pwm_device *pwm_dev = &chip->pwm_dev;

	rc = of_property_read_u32(of_parent, "qcom,period", (u32 *)&period);
	if (rc) {
		pr_err("node is missing PWM Period prop");
		return rc;
	}

	rc = of_property_read_u32(of_pwm_node, "qcom,duty",
				&pwm_dev->pwm_config.pwm_duty);
	if (rc) {
		pr_err("node is missing PWM Duty prop");
		return rc;
	}

	rc = _pwm_config(pwm_dev, LVL_USEC,
					pwm_dev->pwm_config.pwm_duty, period);

	return rc;
}

#define qpnp_check_optional_dt_bindings(func)	\
do {					\
	rc = func;			\
	if (rc && rc != -EINVAL)	\
		goto out;		\
	rc = 0;				\
} while (0);

static int qpnp_parse_lpg_dt_config(struct device_node *of_lpg_node,
		struct device_node *of_parent, struct qpnp_lpg_chip *chip)
{
	int rc, period, list_size, start_idx, *duty_pct_list, i;
	struct pwm_device *pwm_dev = &chip->pwm_dev;
	struct qpnp_lpg_config	*lpg_config = &chip->lpg_config;
	struct qpnp_lut_config	*lut_config = &lpg_config->lut_config;
	struct lut_params	lut_params;

	rc = of_property_read_u32(of_parent, "qcom,period", &period);
	if (rc) {
		pr_err("node is missing PWM Period prop");
		return rc;
	}

	if (!of_get_property(of_lpg_node, "qcom,duty-percents", &list_size)) {
		pr_err("node is missing duty-pct list");
		return rc;
	}

	rc = of_property_read_u32(of_lpg_node, "cell-index", &start_idx);
	if (rc) {
		pr_err("Missing start index");
		return rc;
	}

	list_size /= sizeof(u32);

	if (list_size + start_idx > lpg_config->lut_size) {
		pr_err("duty pct list size overflows\n");
		return -EINVAL;
	}

	duty_pct_list = kzalloc(sizeof(u32) * list_size, GFP_KERNEL);

	if (!duty_pct_list) {
		pr_err("kzalloc failed on duty_pct_list\n");
		return -ENOMEM;
	}

	qpnp_check_optional_dt_bindings(of_property_read_u32(of_lpg_node,
				"qcom,lut_coefficient",
				&lut_coefficient));
	rc = of_property_read_u32_array(of_lpg_node, "qcom,duty-percents",
						duty_pct_list, list_size);
	if (rc) {
		pr_err("invalid or missing property:\n");
		pr_err("qcom,duty-pcts-list\n");
		kfree(duty_pct_list);
		return rc;
	}
	for (i = 0;i < list_size;i++){
		dutys_array[i] = *(duty_pct_list + i);
		*(duty_pct_list + i) = *(duty_pct_list + i) * lut_coefficient / 100;
	}
	dutys_size = list_size;

	
	qpnp_check_optional_dt_bindings(of_property_read_u32(of_lpg_node,
		"qcom,ramp-step-duration", &lut_config->ramp_step_ms));
	qpnp_check_optional_dt_bindings(of_property_read_u32(of_lpg_node,
		"qcom,lpg-lut-pause-hi", &lut_config->lut_pause_hi_cnt));
	qpnp_check_optional_dt_bindings(of_property_read_u32(of_lpg_node,
		"qcom,lpg-lut-pause-lo", &lut_config->lut_pause_lo_cnt));
	qpnp_check_optional_dt_bindings(of_property_read_u32(of_lpg_node,
				"qcom,lpg-lut-ramp-direction",
				(u32 *)&lut_config->ramp_direction));
	qpnp_check_optional_dt_bindings(of_property_read_u32(of_lpg_node,
				"qcom,lpg-lut-pattern-repeat",
				(u32 *)&lut_config->pattern_repeat));
	qpnp_check_optional_dt_bindings(of_property_read_u32(of_lpg_node,
				"qcom,lpg-lut-ramp-toggle",
				(u32 *)&lut_config->ramp_toggle));
	qpnp_check_optional_dt_bindings(of_property_read_u32(of_lpg_node,
				"qcom,lpg-lut-enable-pause-hi",
				(u32 *)&lut_config->enable_pause_hi));
	qpnp_check_optional_dt_bindings(of_property_read_u32(of_lpg_node,
				"qcom,lpg-lut-enable-pause-lo",
				(u32 *)&lut_config->enable_pause_lo));

	qpnp_set_lut_params(&lut_params, lut_config, start_idx, list_size);

	_pwm_lut_config(pwm_dev, period, duty_pct_list, lut_params);

out:
	kfree(duty_pct_list);
	return rc;
}

void qpnp_led_set_for_key(int brightness_key)
{
	int rc, i, dutys[8];
	struct pwm_device *pwm_dev = &led_for_key->chip->pwm_dev;
	u8 val, mask;
	LED_INFO("%s id: %d value %d status: %d\n", __func__, led_for_key->id, brightness_key, led_for_key->status);

	mutex_lock(&led_for_key->lock);
	if(brightness_key) {
		dutys[0] = 0;
		for (i = 1;i < dutys_size; i++) {
			dutys[i] = 100;
		}
		qpnp_lpg_change_table(pwm_dev, dutys, 0);
		val = 0x1B;
		mask = 0xff;
		rc = qpnp_lpg_save_and_write(val, mask,
			&led_for_key->chip->qpnp_lpg_registers[QPNP_LPG_PATTERN_CONFIG],
			LPG_CHAN2_LPG_PATTERN_CONFIG,
			1, led_for_key->chip);
		val = 0x33;
		mask = 0xff;
		rc = qpnp_lpg_save_and_write(val, mask,
			&led_for_key->chip->qpnp_lpg_registers[QPNP_LPG_PATTERN_CONFIG],
			LPG_CHAN2_LPG_PWM_SIZE_CLK,
			1, led_for_key->chip);
		val = 0x2;
		mask = 0xff;
		rc = qpnp_lpg_save_and_write(val, mask,
			&led_for_key->chip->qpnp_lpg_registers[QPNP_LPG_PATTERN_CONFIG],
			LPG_LUT_RAMP_CONTROL,
			1, led_for_key->chip);
		flag_hold_virtual_key = 1;
	} else if (!brightness_key) {
		val = 0x14;
		mask = 0xff;
		rc = qpnp_lpg_save_and_write(val, mask,
			&led_for_key->chip->qpnp_lpg_registers[QPNP_LPG_PATTERN_CONFIG],
			LPG_CHAN2_LPG_PATTERN_CONFIG,
			1, led_for_key->chip);
		val = 0x33;
		mask = 0xff;
		rc = qpnp_lpg_save_and_write(val, mask,
			&led_for_key->chip->qpnp_lpg_registers[QPNP_LPG_PATTERN_CONFIG],
			LPG_CHAN2_LPG_PWM_SIZE_CLK,
			1, led_for_key->chip);
		val = 0x2;
		mask = 0xff;
		rc = qpnp_lpg_save_and_write(val, mask,
			&led_for_key->chip->qpnp_lpg_registers[QPNP_LPG_PATTERN_CONFIG],
			LPG_LUT_RAMP_CONTROL,
			1, led_for_key->chip);
		led_for_key->status = brightness_key;
		flag_hold_virtual_key = 0;
		blink_for_key = 1;
	}
	mutex_unlock(&led_for_key->lock);
}

static void qpnp_led_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	int rc, i, dutys[8];
	struct qpnp_led_data *led;
	struct pwm_device *pwm_dev = &led_for_key->chip->pwm_dev;
	u8 val, mask;

	if (flag_hold_virtual_key == 1) {
		LED_INFO("Button-backlight controlled by key\n");
		return;
	}
	if (blink_for_key) {
		LED_INFO("Reset the LUT\n");
		for (i = 0;i<dutys_size;i++) {
			dutys[i] = dutys_array[i] * lut_coefficient / 100;
		}
		qpnp_lpg_change_table(pwm_dev, dutys, 0);
		blink_for_key = 0;
	}

	led = container_of(led_cdev, struct qpnp_led_data, cdev);
	LED_INFO("%s id: %d value %d status: %d\n", __func__, led->id, value, led->status);
	mutex_lock(&led->lock);
	if(value && !led->status) {
		val = 0x10;
		mask = 0xff;
		rc = qpnp_lpg_save_and_write(val, mask,
			&led->chip->qpnp_lpg_registers[QPNP_LPG_PATTERN_CONFIG],
			LPG_CHAN2_LPG_PATTERN_CONFIG,
			1, led->chip);
		val = 0x33;
		mask = 0xff;
		rc = qpnp_lpg_save_and_write(val, mask,
			&led->chip->qpnp_lpg_registers[QPNP_LPG_PATTERN_CONFIG],
			LPG_CHAN2_LPG_PWM_SIZE_CLK,
			1, led->chip);
		val = 0x2;
		mask = 0xff;
		rc = qpnp_lpg_save_and_write(val, mask,
			&led->chip->qpnp_lpg_registers[QPNP_LPG_PATTERN_CONFIG],
			LPG_LUT_RAMP_CONTROL,
			1, led->chip);
		led->status = value;
	} else if (!value && led->status) {
		val = 0x0;
		mask = 0xff;
		rc = qpnp_lpg_save_and_write(val, mask,
			&led->chip->qpnp_lpg_registers[QPNP_LPG_PATTERN_CONFIG],
			LPG_CHAN2_LPG_PATTERN_CONFIG,
			1, led->chip);
		val = 0x33;
		mask = 0xff;
		rc = qpnp_lpg_save_and_write(val, mask,
			&led->chip->qpnp_lpg_registers[QPNP_LPG_PATTERN_CONFIG],
			LPG_CHAN2_LPG_PWM_SIZE_CLK,
			1, led->chip);
		val = 0x2;
		mask = 0xff;
		rc = qpnp_lpg_save_and_write(val, mask,
			&led->chip->qpnp_lpg_registers[QPNP_LPG_PATTERN_CONFIG],
			LPG_LUT_RAMP_CONTROL,
			1, led->chip);
		led->status = value;
	}
	mutex_unlock(&led->lock);

}

static enum led_brightness qpnp_led_get(struct led_classdev *led_cdev)
{
	struct qpnp_led_data *led;

	led = container_of(led_cdev, struct qpnp_led_data, cdev);

	return led->status;
}

static ssize_t lut_coefficient_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf)
{
      return sprintf(buf, "%d\n", lut_coefficient);
}

static ssize_t lut_coefficient_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct qpnp_led_data *led;
	struct led_classdev *led_cdev;
	int val, i;
	struct pwm_device *pwm_dev;
	int dutys[8];

	val = -1;
	sscanf(buf, "%u", &val);
	LED_INFO("lut_coefficient %d\n", val);
	lut_coefficient = val;
	if (val < 0 || val > 100)
		return -EINVAL;
	led_cdev = (struct led_classdev *) dev_get_drvdata(dev);
	led = container_of(led_cdev, struct qpnp_led_data, cdev);
	pwm_dev = &led->chip->pwm_dev;

	for (i = 0;i<dutys_size;i++) {
		dutys[i] = dutys_array[i] * val / 100;
		LED_INFO("dutys %d\n", dutys[i]);
	}
	qpnp_lpg_change_table(pwm_dev, dutys, 0);
	return count;
}
static DEVICE_ATTR(lut_coefficient, 0644, lut_coefficient_show, lut_coefficient_store);

static int qpnp_button_backlight_config(struct spmi_device *spmi, struct qpnp_lpg_chip *chip)
{
	struct qpnp_led_data *led, *led_array;
	struct resource *led_resource;
	struct device_node *node, *temp;
	int rc = 0, num_leds = 0, parsed_leds = 0;

	node = spmi->dev.of_node;

	if (node == NULL)
		return -ENODEV;

	temp = NULL;
	while ((temp = of_get_next_child(node, temp)))
		num_leds++;
	led_array = devm_kzalloc(&spmi->dev,
		(sizeof(struct qpnp_led_data) * num_leds), GFP_KERNEL);
	if (!led_array) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}
	for_each_child_of_node(node, temp) {
		led = &led_array[parsed_leds];
		LED_INFO("%s id: %d\n", __func__, led->id);
		led->num_leds = 1;
		led->spmi_dev = spmi;
		led->chip = chip;
		led->status = 0;

		led_resource = spmi_get_resource(spmi, NULL, IORESOURCE_MEM, 0);
		if (!led_resource) {
			dev_err(&spmi->dev, "Unable to get LED base address\n");
			kfree(led_array);
			return -ENXIO;
		}
		led->base = led_resource->start;

		rc = of_property_read_string(temp, "linux,name",
			&led->cdev.name);

		rc = of_property_read_u32(temp, "qcom,channel-id",
				&led->id);

		led->cdev.brightness_set    = qpnp_led_set;
		led->cdev.brightness_get    = qpnp_led_get;
		mutex_init(&led->lock);

		rc = led_classdev_register(&spmi->dev, &led->cdev);

		dev_set_drvdata(&spmi->dev, led_array);

		if (!strncmp(led->cdev.name, "button-backlight", 16)) {
			rc = device_create_file(led->cdev.dev, &dev_attr_lut_coefficient);
			LED_INFO("Init button-backlight key control\n");
			led_for_key = led;
			qpnp_led_set_for_key(0);
		}

		parsed_leds++;
	}
	return 0;
}

static int qpnp_parse_dt_config(struct spmi_device *spmi,
					struct qpnp_lpg_chip *chip)
{
	int			rc, enable, lut_entry_size;
	const char		*lable;
	struct resource		*res;
	struct device_node	*node;
	int found_pwm_subnode = 0;
	int found_lpg_subnode = 0;
	struct device_node	*of_node = spmi->dev.of_node;
	struct pwm_device	*pwm_dev = &chip->pwm_dev;
	struct qpnp_lpg_config	*lpg_config = &chip->lpg_config;
	struct qpnp_lut_config	*lut_config = &lpg_config->lut_config;
	int			force_pwm_size = 0;

	rc = of_property_read_u32(of_node, "qcom,channel-id",
				&pwm_dev->pwm_config.channel_id);
	if (rc) {
		dev_err(&spmi->dev, "%s: node is missing LPG channel id\n",
								__func__);
		goto out;
	}

	rc = of_property_read_u32(of_node, "qcom,force-pwm-size",
				&force_pwm_size);
	if (qpnp_check_gpled_lpg_channel(pwm_dev->pwm_config.channel_id)) {
		if (!(force_pwm_size == QPNP_PWM_SIZE_7_BIT ||
				force_pwm_size == QPNP_PWM_SIZE_8_BIT))
			force_pwm_size = 0;
	} else if (chip->sub_type == QPNP_PWM_MODE_ONLY_SUB_TYPE) {
		if (!(force_pwm_size == QPNP_PWM_SIZE_6_BIT ||
				force_pwm_size == QPNP_PWM_SIZE_9_BIT))
			force_pwm_size = 0;
	} else if (!(force_pwm_size == QPNP_PWM_SIZE_6_BIT ||
				force_pwm_size == QPNP_PWM_SIZE_7_BIT ||
				force_pwm_size == QPNP_PWM_SIZE_9_BIT))
			force_pwm_size = 0;

	pwm_dev->pwm_config.force_pwm_size = force_pwm_size;
	res = spmi_get_resource_byname(spmi, NULL, IORESOURCE_MEM,
					QPNP_LPG_CHANNEL_BASE);
	if (!res) {
		dev_err(&spmi->dev, "%s: node is missing base address\n",
			__func__);
		return -EINVAL;
	}

	lpg_config->base_addr = res->start;

	res = spmi_get_resource_byname(spmi, NULL, IORESOURCE_MEM,
						QPNP_LPG_LUT_BASE);
	if (!res) {
		chip->flags |= QPNP_PWM_LUT_NOT_SUPPORTED;
	} else {
		lpg_config->lut_base_addr = res->start;
		lpg_config->lut_size = resource_size(res) >> 1;
		lut_entry_size = sizeof(u16);

		if (qpnp_check_gpled_lpg_channel(
				pwm_dev->pwm_config.channel_id)) {
			lpg_config->lut_size = resource_size(res);
			lut_entry_size = sizeof(u8);
		}

		lut_config->duty_pct_list = kzalloc(lpg_config->lut_size *
					lut_entry_size, GFP_KERNEL);
		if (!lut_config->duty_pct_list) {
			pr_err("can not allocate duty pct list\n");
			return -ENOMEM;
		}
	}

	for_each_child_of_node(of_node, node) {
		rc = of_property_read_string(node, "label", &lable);
		if (rc) {
			dev_err(&spmi->dev, "%s: Missing lable property\n",
								__func__);
			goto out;
		}
		if (!strncmp(lable, "pwm", 3)) {
			rc = qpnp_parse_pwm_dt_config(node, of_node, chip);
			if (rc)
				goto out;
			found_pwm_subnode = 1;
		} else if (!strncmp(lable, "lpg", 3) &&
				!(chip->flags & QPNP_PWM_LUT_NOT_SUPPORTED)) {
			qpnp_parse_lpg_dt_config(node, of_node, chip);
			qpnp_button_backlight_config(spmi, chip);
			if (rc)
				goto out;
			found_lpg_subnode = 1;
		} else {
			dev_err(&spmi->dev, "%s: Invalid value for lable prop",
								__func__);
		}
	}

	rc = of_property_read_u32(of_node, "qcom,mode-select", &enable);
	if (rc)
		goto read_opt_props;

	if ((enable == PM_PWM_MODE_PWM && found_pwm_subnode == 0) ||
		(enable == PM_PWM_MODE_LPG && found_lpg_subnode == 0)) {
		dev_err(&spmi->dev, "%s: Invalid mode select\n", __func__);
		rc = -EINVAL;
		goto out;
	}

	pwm_change_mode(pwm_dev, enable);
	_pwm_enable(pwm_dev);

read_opt_props:
	
	of_property_read_string(node, "qcom,channel-owner",
				&pwm_dev->pwm_config.lable);

	return 0;

out:
	kfree(lut_config->duty_pct_list);
	return rc;
}

static int __devinit qpnp_pwm_probe(struct spmi_device *spmi)
{
	struct qpnp_lpg_chip	*chip;
	int			rc, id;

	chip = kzalloc(sizeof *chip, GFP_KERNEL);
	if (chip == NULL) {
		pr_err("kzalloc() failed.\n");
		return -ENOMEM;
	}

	spin_lock_init(&chip->lpg_lock);

	chip->spmi_dev = spmi;
	chip->pwm_dev.chip = chip;
	dev_set_drvdata(&spmi->dev, chip);

	rc = qpnp_parse_dt_config(spmi, chip);

	if (rc)
		goto failed_config;

	id = chip->pwm_dev.pwm_config.channel_id;

	spmi_ext_register_readl(chip->spmi_dev->ctrl,
		chip->spmi_dev->sid,
		chip->lpg_config.base_addr + SPMI_LPG_REVISION2_OFFSET,
		(u8 *) &chip->revision, 1);

	if (chip->revision < QPNP_LPG_REVISION_0 ||
		chip->revision > QPNP_LPG_REVISION_1) {
		pr_err("Unknown LPG revision detected, rev:%d\n",
						chip->revision);
		rc = -EINVAL;
		goto failed_insert;
	}

	spmi_ext_register_readl(chip->spmi_dev->ctrl,
		chip->spmi_dev->sid,
		chip->lpg_config.base_addr + SPMI_LPG_SUB_TYPE_OFFSET,
		&chip->sub_type, 1);

	rc = radix_tree_insert(&lpg_dev_tree, id, chip);

	if (rc) {
		dev_err(&spmi->dev, "%s: Failed to register LPG Channel %d\n",
								__func__, id);
		goto failed_insert;
	}

	return 0;

failed_insert:
	kfree(chip->lpg_config.lut_config.duty_pct_list);
failed_config:
	dev_set_drvdata(&spmi->dev, NULL);
	kfree(chip);
	return rc;
}

static int __devexit qpnp_pwm_remove(struct spmi_device *spmi)
{
	struct qpnp_lpg_chip *chip;
	struct qpnp_lpg_config *lpg_config;

	chip = dev_get_drvdata(&spmi->dev);

	dev_set_drvdata(&spmi->dev, NULL);

	if (chip) {
		lpg_config = &chip->lpg_config;
		kfree(lpg_config->lut_config.duty_pct_list);
		kfree(chip);
	}

	return 0;
}

static struct of_device_id spmi_match_table[] = {
	{ .compatible = QPNP_LPG_DRIVER_NAME, },
	{}
};

static const struct spmi_device_id qpnp_lpg_id[] = {
	{ QPNP_LPG_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spmi, qpnp_lpg_id);

static struct spmi_driver qpnp_lpg_driver = {
	.driver		= {
		.name	= QPNP_LPG_DRIVER_NAME,
		.of_match_table = spmi_match_table,
		.owner = THIS_MODULE,
	},
	.probe		= qpnp_pwm_probe,
	.remove		= __devexit_p(qpnp_pwm_remove),
	.id_table	= qpnp_lpg_id,
};

int __init qpnp_lpg_init(void)
{
	return spmi_driver_register(&qpnp_lpg_driver);
}

static void __exit qpnp_lpg_exit(void)
{
	spmi_driver_unregister(&qpnp_lpg_driver);
}

MODULE_DESCRIPTION("QPNP PMIC LPG driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" QPNP_LPG_DRIVER_NAME);

subsys_initcall(qpnp_lpg_init);
module_exit(qpnp_lpg_exit);
