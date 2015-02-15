/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt)	"[BATT][BMS] %s: " fmt, __func__

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/power_supply.h>
#include <linux/spmi.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/qpnp/qpnp-charger.h>
#include <linux/qpnp/qpnp-bms.h>
#include <linux/qpnp/power-on.h>
#include <linux/of_batterydata.h>
#include <mach/devices_dtb.h>
#include <mach/htc_gauge.h>
#include <mach/htc_footprint.h>
#include <linux/interrupt.h>
#ifdef CONFIG_HTC_BATT_8960
#include "mach/htc_battery_cell.h"
#endif

#ifdef pr_debug
#undef pr_debug
#endif
#define pr_debug(fmt, args...) do { \
		if (flag_enable_bms_charger_log) \
			printk(KERN_INFO pr_fmt(fmt), ## args); \
	} while (0)

#define REVISION1			0x0
#define REVISION2			0x1
#define BMS1_STATUS1			0x8
#define BMS1_INT_RT_STS		0x10
#define BMS1_MODE_CTL			0X40
#define BMS1_CC_DATA_CTL		0x42
#define BMS1_CC_CLEAR_CTL		0x43
#define BMS1_TOL_CTL			0X44
#define BMS1_OCV_USE_LOW_LIMIT_THR0	0x48
#define BMS1_OCV_USE_LOW_LIMIT_THR1	0x49
#define BMS1_OCV_USE_HIGH_LIMIT_THR0	0x4A
#define BMS1_OCV_USE_HIGH_LIMIT_THR1	0x4B
#define BMS1_OCV_USE_LIMIT_CTL		0x4C
#define BMS1_S1_DELAY_CTL		0x5A
#define BMS1_OCV_THR0			0x50
#define BMS1_S2_SAMP_AVG_CTL		0x61
#define BMS1_SW_CC_THR0			0xA0
#define BMS1_S3_VSENSE_THR_CTL		0x77
#define BMS1_OCV_FOR_R_DATA0		0x80
#define BMS1_VSENSE_FOR_R_DATA0		0x82
#define BMS1_CC_DATA0			0x8A
#define BMS1_SW_CC_DATA0		0xA8
#define BMS1_OCV_FOR_SOC_DATA0		0x90
#define BMS1_VSENSE_PON_DATA0		0x94
#define BMS1_VSENSE_AVG_DATA0		0x98
#define BMS1_VBAT_AVG_DATA0		0x9E
#define SOC_STORAGE_REG			0xB0
#define IAVG_STORAGE_REG		0xB1
#define BMS_FCC_COUNT			0xB2
#define BMS_FCC_BASE_REG		0xB3 
#define BMS_CHGCYL_BASE_REG		0xB8 
#define CHARGE_INCREASE_STORAGE		0xBD
#define CHARGE_CYCLE_STORAGE_LSB	0xBE 

#define IADC1_BMS_REVISION2		0x01
#define IADC1_BMS_ADC_CH_SEL_CTL	0x48
#define IADC1_BMS_ADC_INT_RSNSN_CTL	0x49
#define IADC1_BMS_FAST_AVG_EN		0x5B

#define IGNORE_SOC_TEMP_DECIDEG		50
#define IAVG_STEP_SIZE_MA		10
#define IAVG_INVALID			0xFF
#define SOC_INVALID			0x7E

#define IAVG_SAMPLES 16

#define MAX_FCC_CYCLES				5
#define DELTA_FCC_PERCENT                       5
#define VALID_FCC_CHGCYL_RANGE                  50
#define CHGCYL_RESOLUTION			6
#define FCC_DEFAULT_TEMP			250

#define BMS_STORE_MAGIC_NUM		0xDDAACC00
#define BMS_STORE_MAGIC_OFFSET		1056	
#define BMS_STORE_SOC_OFFSET		1060	
#define BMS_STORE_OCV_OFFSET		1064	
#define BMS_STORE_CC_OFFSET		1068	
#define BMS_STORE_CURRTIME_OFFSET	1072	

#define QPNP_BMS_DEV_NAME "qcom,qpnp-bms"

#define FIRST_SW_EST_OCV_THR_MS	(21600000)	
#define DEFAULT_SW_EST_OCV_THR_MS (79200000)
#define DISABLE_SW_OCV_LEVEL_THRESHOLD	30

enum {
	SHDW_CC,
	CC
};

enum {
	NORESET,
	RESET
};

struct soc_params {
	int		fcc_uah;
	int		cc_uah;
	int		rbatt_mohm;
	int		iavg_ua;
	int		uuc_uah;
	int		ocv_charge_uah;
	int		delta_time_s;
};

struct raw_soc_params {
	uint16_t	last_good_ocv_raw;
	int64_t		cc;
	int64_t		shdw_cc;
	int		last_good_ocv_uv;
};

struct fcc_sample {
	int fcc_new;
	int chargecycles;
};

struct bms_irq {
	unsigned int	irq;
	unsigned long	disabled;
};

struct bms_wakeup_source {
	struct wakeup_source	source;
	unsigned long		disabled;
};

struct qpnp_bms_chip {
	struct device			*dev;
	struct power_supply		bms_psy;
	bool				bms_psy_registered;
	struct power_supply		*batt_psy;
	struct spmi_device		*spmi;
	wait_queue_head_t		bms_wait_queue;
	u16				base;
	u16				iadc_base;
	u16				batt_pres_addr;
	u16				soc_storage_addr;

	u8				revision1;
	u8				revision2;

	u8				iadc_bms_revision1;
	u8				iadc_bms_revision2;

	int				battery_present;
	int				battery_status;
	bool				batfet_closed;
	bool				new_battery;
	bool				done_charging;
	bool				last_soc_invalid;
	
	int				enable_sw_ocv_in_eoc;
	int				store_batt_data_soc_thre;
	int 				batt_stored_magic_num;
	int 				batt_stored_soc;
	int 				batt_stored_ocv_uv;
	int 				batt_stored_cc_uah;
	int				cc_backup_uah;
	int				ocv_backup_uv;
	unsigned int			batt_stored_update_time;
	int				r_sense_uohm;
	unsigned int			v_cutoff_uv;
	unsigned int			shutdown_vol_criteria;
	int				max_voltage_uv;
	int				r_conn_mohm;
	int				shutdown_soc_valid_limit;
	int				adjust_soc_low_threshold;
	int				chg_term_ua;
	enum battery_type		batt_type;
	unsigned int			fcc_mah;
	struct single_row_lut		*fcc_temp_lut;
	struct single_row_lut		*fcc_sf_lut;
	struct pc_temp_ocv_lut		*pc_temp_ocv_lut;
	struct sf_lut			*pc_sf_lut;
	struct sf_lut			*rbatt_sf_lut;
	struct sf_lut			*rbatt_est_ocv_lut;
	int				default_rbatt_mohm;
	int				rbatt_capacitive_mohm;
	int				rbatt_mohm;

	struct delayed_work		calculate_soc_delayed_work;
	struct work_struct		recalc_work;
	struct work_struct		batfet_open_work;

	struct mutex			bms_output_lock;
	struct mutex			last_ocv_uv_mutex;
	struct mutex			vbat_monitor_mutex;
	struct mutex			soc_invalidation_mutex;
	struct mutex			last_soc_mutex;
	struct mutex			status_lock;

	bool				use_external_rsense;
	bool				use_ocv_thresholds;

	bool				ignore_shutdown_soc;
	bool				shutdown_soc_invalid;
	int				shutdown_soc;
	int				shutdown_iavg_ma;

	struct wake_lock		low_voltage_wake_lock;
	int				low_voltage_threshold;
	int				low_soc_calc_threshold;
	int				low_soc_calculate_soc_ms;
	int				low_voltage_calculate_soc_ms;
	int				calculate_soc_ms;
	struct bms_wakeup_source	soc_wake_source;
	struct wake_lock		cv_wake_lock;

	uint16_t			ocv_reading_at_100;
	uint16_t			prev_last_good_ocv_raw;
	int				insertion_ocv_uv;
	int				last_ocv_uv;
	int				charging_adjusted_ocv;
	int				last_ocv_temp;
	int				last_cc_uah;
	unsigned long			last_soc_change_sec;
	unsigned long			tm_sec;
	unsigned long			report_tm_sec;
	bool				first_time_calc_soc;
	bool				first_time_calc_uuc;
	int64_t				software_cc_uah;
	int64_t				software_shdw_cc_uah;

	int				iavg_samples_ma[IAVG_SAMPLES];
	int				iavg_index;
	int				iavg_num_samples;
	struct timespec			t_soc_queried;
	int				last_soc;
	int				last_soc_est;
	int				last_soc_unbound;
	bool				was_charging_at_sleep;
	int				charge_start_tm_sec;
	int				catch_up_time_sec;
	struct single_row_lut		*adjusted_fcc_temp_lut;

	struct qpnp_adc_tm_btm_param	vbat_monitor_params;
	struct qpnp_adc_tm_btm_param	die_temp_monitor_params;
	int				temperature_margin;
	unsigned int			vadc_v0625;
	unsigned int			vadc_v1250;
	unsigned int			criteria_sw_est_ocv;
	unsigned int 			rconn_mohm_sw_est_ocv;
	int				system_load_count;
	int				prev_uuc_iavg_ma;
	int				prev_pc_unusable;
	int				ibat_at_cv_ua;
	int				soc_at_cv;
	int				prev_chg_soc;
	int				calculated_soc;
	int				prev_voltage_based_soc;
	bool				use_voltage_soc;
	bool				in_cv_range;

	int				prev_batt_terminal_uv;
	int				high_ocv_correction_limit_uv;
	int				low_ocv_correction_limit_uv;
	int				flat_ocv_threshold_uv;
	int				hold_soc_est;

	int				ocv_high_threshold_uv;
	int				ocv_low_threshold_uv;
	unsigned long			last_recalc_time;

	struct fcc_sample		*fcc_learning_samples;
	u8				fcc_sample_count;
	int				enable_fcc_learning;
	int				min_fcc_learning_soc;
	int				min_fcc_ocv_pc;
	int				min_fcc_learning_samples;
	int				start_soc;
	int				end_soc;
	int				start_pc;
	int				start_cc_uah;
	int				start_real_soc;
	int				end_cc_uah;
	uint16_t			fcc_new_mah;
	int				fcc_new_batt_temp;
	uint16_t			charge_cycles;
	u8				charge_increase;
	int				fcc_resolution;
	bool				battery_removed;
	struct bms_irq			cc_thr_irq;
	struct bms_irq			ocv_for_r_irq;
	struct bms_irq			good_ocv_irq;
	struct bms_irq			charge_begin_irq;
	struct bms_irq			sw_cc_thr_irq;
	struct bms_irq			ocv_thr_irq;
	struct bms_irq			vsense_avg_irq;
	struct bms_irq			vsense_for_r_irq;
	struct qpnp_vadc_chip		*vadc_dev;
	struct qpnp_iadc_chip		*iadc_dev;
	struct qpnp_adc_tm_chip		*adc_tm_dev;
	bool			batt_full_fake_ocv;
	int				enable_batt_full_fake_ocv;
	int				qb_mode_cc_criteria_uAh;
	int				soc_overrate_criteria_for_sw_ocv;
};

struct pm8941_bms_debug {
	int fcc_uah;
	int uuc_uah;
	int rc_uah;
	int ruc_uah;
	int cc_uah;
	int ori_cc_uah;
	int raw_soc;
	int adjusted_soc;
	int ori_soc_change;
	int rbatt;
	int rbatt_sf;
	int uuc_iavg_ma;
	int uuc_rbatt_mohm;
	int ori_uuc_uah;
	int unusable_uv;
	int pc_unusable;
	int rc_pc;
	int shutdown_soc;
	int soc_rbatt;
	int pon_est_ocv;
	int ibat_for_est_ocv;
	int vbat_for_est_ocv;
	int rbat_for_est_ocv;
	int time_last_change_s;
	int last_ocv_raw_uv;
};
static struct pm8941_bms_debug bms_dbg;

struct pm8941_battery_data_store {
	int           store_soc;
	int           store_ocv_uv;
	int           store_cc_uah;
	unsigned long store_currtime_ms;
};
static struct pm8941_battery_data_store store_emmc;

static struct of_device_id qpnp_bms_match_table[] = {
	{ .compatible = QPNP_BMS_DEV_NAME },
	{}
};

static char *qpnp_bms_supplicants[] = {
	"battery"
};

static enum power_supply_property msm_bms_power_props[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_RESISTANCE,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CHARGE_COUNTER_SHADOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
};

struct htc_bms_timer {
	unsigned long batt_system_jiffies;
	unsigned long batt_suspend_ms;
	unsigned long no_ocv_update_period_ms;
};
static struct htc_bms_timer htc_batt_bms_timer;

static int ocv_update_stop_active_mask = OCV_UPDATE_STOP_BIT_CABLE_IN |
											OCV_UPDATE_STOP_BIT_ATTR_FILE |
											OCV_UPDATE_STOP_BIT_BOOT_UP;
static int ocv_update_stop_reason;
static int sw_ocv_estimated_stop_active_mask = OCV_UPDATE_STOP_BIT_CABLE_OUT;
static int sw_ocv_estimated_stop_reason;
static int is_ocv_update_start = 0;
static int is_do_sw_ocv_in_eoc = 0;
struct mutex ocv_update_lock;
static int batt_level = 0;
static bool qb_mode_enter = false;
static int qb_mode_cc_accumulation_uah, qb_mode_prev_cc;
static int qb_mode_ocv_start = 0, qb_mode_cc_start = 0, qb_mode_over_criteria_count = 0;
static unsigned long qb_mode_time_accumulation = 0;

static void disable_ocv_update_with_reason(bool disable, int reason);
static int discard_backup_fcc_data(struct qpnp_bms_chip *chip);
static void backup_charge_cycle(struct qpnp_bms_chip *chip);
static int64_t read_battery_id(struct qpnp_bms_chip *chip);
static int get_rbatt(struct qpnp_bms_chip *chip,
					int soc_rbatt_mohm, int batt_temp);
int emmc_misc_write(int val, int offset);
static int calculate_cc(struct qpnp_bms_chip *chip, int64_t cc, int cc_type, int clear_cc);
int pm8941_bms_get_percent_charge(struct qpnp_bms_chip *chip);

static bool bms_reset;
static struct qpnp_bms_chip *the_chip;
static bool flag_enable_bms_charger_log;
#define BATT_LOG_BUF_LEN (512)
static char batt_log_buf[BATT_LOG_BUF_LEN];
static unsigned long allow_ocv_time = 0;
static unsigned long allow_sw_ocv_est_time = 0;
static int new_boot_soc = 0;
static int consistent_flag = false;;
static int store_soc_ui = -1;
static int64_t pre_cc_uah = 0;
static bool gbFCC_Start = false;
static bool gbFCC_Update = false;
static int store_start_soc = 0;
static int store_start_cc = 0;
static int store_start_pc = 0;
static int store_start_real_soc = 0;
static int Last_OCV_Raw = 0;
static int Count_FCC_Checking_cycle = 0;
static int chgcyl_checking_setting = 0;
static int chgcyl_checking_table[6][2] = {
				{500,85},
				{400,87},
				{300,89},
				{200,91},
				{100,93},
				{0,95}
				};

static int qpnp_read_wrapper(struct qpnp_bms_chip *chip, u8 *val,
			u16 base, int count)
{
	int rc;
	struct spmi_device *spmi = chip->spmi;

	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, base, val, count);
	if (rc) {
		pr_err("SPMI read failed rc=%d\n", rc);
		return rc;
	}
	return 0;
}

static int qpnp_write_wrapper(struct qpnp_bms_chip *chip, u8 *val,
			u16 base, int count)
{
	int rc;
	struct spmi_device *spmi = chip->spmi;

	rc = spmi_ext_register_writel(spmi->ctrl, spmi->sid, base, val, count);
	if (rc) {
		pr_err("SPMI write failed rc=%d\n", rc);
		return rc;
	}
	return 0;
}

static int qpnp_masked_write_base(struct qpnp_bms_chip *chip, u16 addr,
							u8 mask, u8 val)
{
	int rc;
	u8 reg;

	rc = qpnp_read_wrapper(chip, &reg, addr, 1);
	if (rc) {
		pr_err("read failed addr = %03X, rc = %d\n", addr, rc);
		return rc;
	}
	reg &= ~mask;
	reg |= val & mask;
	rc = qpnp_write_wrapper(chip, &reg, addr, 1);
	if (rc) {
		pr_err("write failed addr = %03X, val = %02x, mask = %02x, reg = %02x, rc = %d\n",
					addr, val, mask, reg, rc);
		return rc;
	}
	return 0;
}

static int qpnp_masked_write_iadc(struct qpnp_bms_chip *chip, u16 addr,
							u8 mask, u8 val)
{
	return qpnp_masked_write_base(chip, chip->iadc_base + addr, mask, val);
}

static int qpnp_masked_write(struct qpnp_bms_chip *chip, u16 addr,
							u8 mask, u8 val)
{
	return qpnp_masked_write_base(chip, chip->base + addr, mask, val);
}

static void bms_stay_awake(struct bms_wakeup_source *source)
{
	if (__test_and_clear_bit(0, &source->disabled)) {
		__pm_stay_awake(&source->source);
		pr_debug("enabled source %s\n", source->source.name);
	}
}

static void bms_relax(struct bms_wakeup_source *source)
{
	if (!__test_and_set_bit(0, &source->disabled)) {
		__pm_relax(&source->source);
		pr_debug("disabled source %s\n", source->source.name);
	}
}

static void enable_bms_irq(struct bms_irq *irq)
{
	if (__test_and_clear_bit(0, &irq->disabled)) {
		enable_irq(irq->irq);
		pr_debug("enabled irq %d\n", irq->irq);
	}
}

static void disable_bms_irq(struct bms_irq *irq)
{
	if (!__test_and_set_bit(0, &irq->disabled)) {
		disable_irq(irq->irq);
		pr_debug("disabled irq %d\n", irq->irq);
	}
}

#define HOLD_OREG_DATA		BIT(0)
static int lock_output_data(struct qpnp_bms_chip *chip)
{
	int rc;

	rc = qpnp_masked_write(chip, BMS1_CC_DATA_CTL,
				HOLD_OREG_DATA, HOLD_OREG_DATA);
	if (rc) {
		pr_err("couldnt lock bms output rc = %d\n", rc);
		return rc;
	}
	return 0;
}

static int unlock_output_data(struct qpnp_bms_chip *chip)
{
	int rc;

	rc = qpnp_masked_write(chip, BMS1_CC_DATA_CTL, HOLD_OREG_DATA, 0);
	if (rc) {
		pr_err("fail to unlock BMS_CONTROL rc = %d\n", rc);
		return rc;
	}
	return 0;
}

#define V_PER_BIT_MUL_FACTOR	97656
#define V_PER_BIT_DIV_FACTOR	1000
#define VADC_INTRINSIC_OFFSET	0x6000

static int vadc_reading_to_uv(int reading)
{
	if (reading <= VADC_INTRINSIC_OFFSET)
		return 0;

	return (reading - VADC_INTRINSIC_OFFSET)
			* V_PER_BIT_MUL_FACTOR / V_PER_BIT_DIV_FACTOR;
}

#define VADC_CALIB_UV		625000
#define VBATT_MUL_FACTOR	3
static int adjust_vbatt_reading(struct qpnp_bms_chip *chip, int reading_uv)
{
	s64 numerator, denominator;

	if (reading_uv == 0)
		return 0;

	
	if (chip->vadc_v0625 == 0 || chip->vadc_v1250 == 0) {
		pr_debug("No cal yet return %d\n",
				VBATT_MUL_FACTOR * reading_uv);
		return VBATT_MUL_FACTOR * reading_uv;
	}

	numerator = ((s64)reading_uv - chip->vadc_v0625) * VADC_CALIB_UV;
	denominator =  (s64)chip->vadc_v1250 - chip->vadc_v0625;
	if (denominator == 0)
		return reading_uv * VBATT_MUL_FACTOR;
	return (VADC_CALIB_UV + div_s64(numerator, denominator))
						* VBATT_MUL_FACTOR;
}

static int convert_vbatt_uv_to_raw(struct qpnp_bms_chip *chip,
					int unadjusted_vbatt)
{
	int scaled_vbatt = unadjusted_vbatt / VBATT_MUL_FACTOR;

	if (scaled_vbatt <= 0)
		return VADC_INTRINSIC_OFFSET;
	return ((scaled_vbatt * V_PER_BIT_DIV_FACTOR) / V_PER_BIT_MUL_FACTOR)
						+ VADC_INTRINSIC_OFFSET;
}

static inline int convert_vbatt_raw_to_uv(struct qpnp_bms_chip *chip,
					uint16_t reading, bool is_pon_ocv)
{
	int64_t uv;
	int rc;

	uv = vadc_reading_to_uv(reading);
	pr_debug("%u raw converted into %lld uv\n", reading, uv);
	uv = adjust_vbatt_reading(chip, uv);
	pr_debug("adjusted into %lld uv\n", uv);
	rc = qpnp_vbat_sns_comp_result(chip->vadc_dev, &uv, is_pon_ocv);
	if (rc)
		pr_debug("could not compensate vbatt\n");
	pr_debug("compensated into %lld uv\n", uv);
	return uv;
}

#define CC_READING_RESOLUTION_N	542535
#define CC_READING_RESOLUTION_D	100000
static s64 cc_reading_to_uv(s64 reading)
{
	return div_s64(reading * CC_READING_RESOLUTION_N,
					CC_READING_RESOLUTION_D);
}

#define QPNP_ADC_GAIN_IDEAL				3291LL
static s64 cc_adjust_for_gain(s64 uv, uint16_t gain)
{
	s64 result_uv;

	pr_debug("adjusting_uv = %lld\n", uv);
	if (gain == 0) {
		pr_debug("gain is %d, not adjusting\n", gain);
		return uv;
	}
	pr_debug("adjusting by factor: %lld/%hu = %lld%%\n",
			QPNP_ADC_GAIN_IDEAL, gain,
			div_s64(QPNP_ADC_GAIN_IDEAL * 100LL, (s64)gain));

	result_uv = div_s64(uv * QPNP_ADC_GAIN_IDEAL, (s64)gain);
	pr_debug("result_uv = %lld\n", result_uv);
	return result_uv;
}

static s64 cc_reverse_adjust_for_gain(struct qpnp_bms_chip *chip, s64 uv)
{
	struct qpnp_iadc_calib calibration;
	int gain;
	s64 result_uv;

	qpnp_iadc_get_gain_and_offset(chip->iadc_dev, &calibration);
	gain = (int)calibration.gain_raw - (int)calibration.offset_raw;

	pr_debug("reverse adjusting_uv = %lld\n", uv);
	if (gain == 0) {
		pr_debug("gain is %d, not adjusting\n", gain);
		return uv;
	}
	pr_debug("adjusting by factor: %hu/%lld = %lld%%\n",
			gain, QPNP_ADC_GAIN_IDEAL,
			div64_s64((s64)gain * 100LL,
				(s64)QPNP_ADC_GAIN_IDEAL));

	result_uv = div64_s64(uv * (s64)gain, QPNP_ADC_GAIN_IDEAL);
	pr_debug("result_uv = %lld\n", result_uv);
	return result_uv;
}

static int convert_vsense_to_uv(struct qpnp_bms_chip *chip,
					int16_t reading)
{
	struct qpnp_iadc_calib calibration;

	qpnp_iadc_get_gain_and_offset(chip->iadc_dev, &calibration);
	return cc_adjust_for_gain(cc_reading_to_uv(reading),
			calibration.gain_raw - calibration.offset_raw);
}

static int read_vsense_avg(struct qpnp_bms_chip *chip, int *result_uv)
{
	int rc;
	int16_t reading;

	rc = qpnp_read_wrapper(chip, (u8 *)&reading,
			chip->base + BMS1_VSENSE_AVG_DATA0, 2);

	if (rc) {
		pr_err("fail to read VSENSE_AVG rc = %d\n", rc);
		return rc;
	}

	*result_uv = convert_vsense_to_uv(chip, reading);
	return 0;
}

static int get_battery_current(struct qpnp_bms_chip *chip, int *result_ua)
{
	int rc, vsense_uv = 0;
	int64_t temp_current;

	if (chip->r_sense_uohm == 0) {
		pr_err("r_sense is zero\n");
		return -EINVAL;
	}

	mutex_lock(&chip->bms_output_lock);
	lock_output_data(chip);
	read_vsense_avg(chip, &vsense_uv);
	unlock_output_data(chip);
	mutex_unlock(&chip->bms_output_lock);

	pr_debug("vsense_uv=%duV\n", vsense_uv);
	
	temp_current = div_s64((vsense_uv * 1000000LL),
				(int)chip->r_sense_uohm);

	rc = qpnp_iadc_comp_result(chip->iadc_dev, &temp_current);
	if (rc)
		pr_debug("error compensation failed: %d\n", rc);

	*result_ua = temp_current;
	pr_debug("err compensated ibat=%duA\n", *result_ua);
	return 0;
}

static int get_battery_voltage(struct qpnp_bms_chip *chip, int *result_uv)
{
	int rc;
	struct qpnp_vadc_result adc_result;

	rc = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &adc_result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					VBAT_SNS, rc);
		return rc;
	}
	pr_debug("mvolts phy = %lld meas = 0x%llx\n", adc_result.physical,
						adc_result.measurement);
	*result_uv = (int)adc_result.physical;
	return 0;
}

#define CC_36_BIT_MASK 0xFFFFFFFFFLL
static uint64_t convert_s64_to_s36(int64_t raw64)
{
	return (uint64_t) raw64 & CC_36_BIT_MASK;
}

#define SIGN_EXTEND_36_TO_64_MASK (-1LL ^ CC_36_BIT_MASK)
static int64_t convert_s36_to_s64(uint64_t raw36)
{
	raw36 = raw36 & CC_36_BIT_MASK;
	
	return (raw36 >> 35) == 0LL ?
		raw36 : (SIGN_EXTEND_36_TO_64_MASK | raw36);
}

static int read_cc_raw(struct qpnp_bms_chip *chip, int64_t *reading,
							int cc_type)
{
	int64_t raw_reading;
	int rc;

	if (cc_type == SHDW_CC)
		rc = qpnp_read_wrapper(chip, (u8 *)&raw_reading,
				chip->base + BMS1_SW_CC_DATA0, 5);
	else
		rc = qpnp_read_wrapper(chip, (u8 *)&raw_reading,
				chip->base + BMS1_CC_DATA0, 5);
	if (rc) {
		pr_err("Error reading cc: rc = %d\n", rc);
		return -ENXIO;
	}

	*reading = convert_s36_to_s64(raw_reading);

	return 0;
}

static int calib_vadc(struct qpnp_bms_chip *chip)
{
	int rc, raw_0625, raw_1250;
	struct qpnp_vadc_result result;

	rc = qpnp_vadc_read(chip->vadc_dev, REF_625MV, &result);
	if (rc) {
		pr_debug("vadc read failed with rc = %d\n", rc);
		return rc;
	}
	raw_0625 = result.adc_code;

	rc = qpnp_vadc_read(chip->vadc_dev, REF_125V, &result);
	if (rc) {
		pr_debug("vadc read failed with rc = %d\n", rc);
		return rc;
	}
	raw_1250 = result.adc_code;
	chip->vadc_v0625 = vadc_reading_to_uv(raw_0625);
	chip->vadc_v1250 = vadc_reading_to_uv(raw_1250);
	pr_debug("vadc calib: 0625 = %d raw (%d uv), 1250 = %d raw (%d uv)\n",
			raw_0625, chip->vadc_v0625,
			raw_1250, chip->vadc_v1250);
	return 0;
}

static void convert_and_store_ocv(struct qpnp_bms_chip *chip,
				struct raw_soc_params *raw,
				int batt_temp, bool is_pon_ocv)
{
	int rc;

	pr_info("prev_last_good_ocv_raw = %d, last_good_ocv_raw = %d\n",
			chip->prev_last_good_ocv_raw,
			raw->last_good_ocv_raw);
	rc = calib_vadc(chip);
	if (rc)
		pr_err("Vadc reference voltage read failed, rc = %d\n", rc);
	chip->prev_last_good_ocv_raw = raw->last_good_ocv_raw;
	raw->last_good_ocv_uv = convert_vbatt_raw_to_uv(chip,
					raw->last_good_ocv_raw, is_pon_ocv);
	chip->last_ocv_uv = raw->last_good_ocv_uv;
	chip->last_ocv_temp = batt_temp;
	chip->software_cc_uah = 0;
	pr_debug("last_good_ocv_uv = %d\n", raw->last_good_ocv_uv);
}

#define CLEAR_CC			BIT(7)
#define CLEAR_SHDW_CC			BIT(6)
static void reset_cc(struct qpnp_bms_chip *chip, u8 flags)
{
	int rc;

	pr_info("resetting cc manually with flags %hhu\n", flags);
	mutex_lock(&chip->bms_output_lock);
	rc = qpnp_masked_write(chip, BMS1_CC_CLEAR_CTL,
				flags,
				flags);
	if (rc)
		pr_err("cc reset failed: %d\n", rc);

	
	udelay(100);

	rc = qpnp_masked_write(chip, BMS1_CC_CLEAR_CTL,
				flags, 0);
	if (rc)
		pr_err("cc reenable failed: %d\n", rc);
	mutex_unlock(&chip->bms_output_lock);
}

static int get_battery_status(struct qpnp_bms_chip *chip)
{
#if !(defined(CONFIG_HTC_BATT_8960))
	union power_supply_propval ret = {0,};

	if (chip->batt_psy == NULL)
		chip->batt_psy = power_supply_get_by_name("battery");
	if (chip->batt_psy) {
		
		chip->batt_psy->get_property(chip->batt_psy,
					POWER_SUPPLY_PROP_STATUS, &ret);
		return ret.intval;
	}

	
	pr_debug("battery power supply is not registered\n");
	return POWER_SUPPLY_STATUS_UNKNOWN;

#else
	return pm8941_get_battery_status();
#endif
}

static bool is_battery_charging(struct qpnp_bms_chip *chip)
{
	return get_battery_status(chip) == POWER_SUPPLY_STATUS_CHARGING;
}

static bool is_battery_full(struct qpnp_bms_chip *chip)
{
	return get_battery_status(chip) == POWER_SUPPLY_STATUS_FULL;
}

static bool is_battery_present(struct qpnp_bms_chip *chip)
{
#if !(defined(CONFIG_HTC_BATT_8960))
	union power_supply_propval ret = {0,};

	if (chip->batt_psy == NULL)
		chip->batt_psy = power_supply_get_by_name("battery");
	if (chip->batt_psy) {
		
		chip->batt_psy->get_property(chip->batt_psy,
					POWER_SUPPLY_PROP_PRESENT, &ret);
		return ret.intval;
	}

	
	pr_debug("battery power supply is not registered\n");
	return false;
#else
	return pm8941_get_batt_present();
#endif
}

static int get_battery_insertion_ocv_uv(struct qpnp_bms_chip *chip)
{
	union power_supply_propval ret = {0,};
	int rc, vbat;

	if (chip->batt_psy == NULL)
		chip->batt_psy = power_supply_get_by_name("battery");
	if (chip->batt_psy) {
		
		rc = chip->batt_psy->get_property(chip->batt_psy,
				POWER_SUPPLY_PROP_VOLTAGE_OCV, &ret);
		if (rc) {
			pr_debug("Battery psy does not have voltage ocv\n");
			rc = get_battery_voltage(chip, &vbat);
			if (rc)
				return -EINVAL;
			return vbat;
		}
		return ret.intval;
	}

	pr_debug("battery power supply is not registered\n");
	return -EINVAL;
}

static bool is_batfet_closed(struct qpnp_bms_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->batt_psy == NULL)
		chip->batt_psy = power_supply_get_by_name("battery");
	if (chip->batt_psy) {
		
		chip->batt_psy->get_property(chip->batt_psy,
					POWER_SUPPLY_PROP_ONLINE, &ret);
		return !!ret.intval;
	}

	
	pr_debug("battery power supply is not registered\n");
	return true;
}

static int get_simultaneous_batt_v_and_i(struct qpnp_bms_chip *chip,
					int *ibat_ua, int *vbat_uv)
{
	struct qpnp_iadc_result i_result;
	struct qpnp_vadc_result v_result;
	enum qpnp_iadc_channels iadc_channel;
	int rc;

	iadc_channel = chip->use_external_rsense ?
				EXTERNAL_RSENSE : INTERNAL_RSENSE;
	if (is_battery_full(chip)) {
		rc = get_battery_current(chip, ibat_ua);
		if (rc) {
			pr_err("bms current read failed with rc: %d\n", rc);
			return rc;
		}
		rc = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &v_result);
		if (rc) {
			pr_err("vadc read failed with rc: %d\n", rc);
			return rc;
		}
		*vbat_uv = (int)v_result.physical;
	} else {
		rc = qpnp_iadc_vadc_sync_read(chip->iadc_dev,
					iadc_channel, &i_result,
					VBAT_SNS, &v_result);
		if (rc) {
			pr_err("adc sync read failed with rc: %d\n", rc);
			return rc;
		}
		*ibat_ua = -1 * (int)i_result.result_ua;
		*vbat_uv = (int)v_result.physical;
	}

	return 0;
}

static int estimate_ocv(struct qpnp_bms_chip *chip, int batt_temp)
{
	int ibat_ua, vbat_uv, ocv_est_uv;
	int rc;
	int rbatt_mohm;

	rbatt_mohm = get_rbatt(chip, bms_dbg.soc_rbatt, batt_temp);
	rc = get_simultaneous_batt_v_and_i(chip, &ibat_ua, &vbat_uv);
	if (rc) {
		pr_err("simultaneous failed rc = %d\n", rc);
		return rc;
	}

	ocv_est_uv = vbat_uv + (ibat_ua * rbatt_mohm) / 1000;
	bms_dbg.ibat_for_est_ocv = ibat_ua;
	bms_dbg.vbat_for_est_ocv = vbat_uv;
	bms_dbg.rbat_for_est_ocv = rbatt_mohm;
	pr_info("estimated pon ocv = %d, vbat_uv = %d, ibat_ua = %d, "
			"rbatt_mohm = %d\n",
			ocv_est_uv, vbat_uv, ibat_ua, rbatt_mohm);

	return ocv_est_uv;
}

static void reset_for_new_battery(struct qpnp_bms_chip *chip, int batt_temp)
{
	chip->last_ocv_uv = estimate_ocv(chip, batt_temp);
	mutex_lock(&chip->last_soc_mutex);
	chip->last_soc = -EINVAL;
	chip->last_soc_invalid = true;
	mutex_unlock(&chip->last_soc_mutex);
	chip->soc_at_cv = -EINVAL;
	chip->shutdown_soc_invalid = true;
	chip->shutdown_soc = 0;
	chip->shutdown_iavg_ma = 0;
	chip->prev_pc_unusable = -EINVAL;
	reset_cc(chip, CLEAR_CC | CLEAR_SHDW_CC);
	chip->software_cc_uah = 0;
	chip->software_shdw_cc_uah = 0;
	chip->last_cc_uah = INT_MIN;
	chip->last_ocv_temp = batt_temp;
	chip->prev_batt_terminal_uv = 0;
	if (chip->enable_fcc_learning) {
		chip->adjusted_fcc_temp_lut = NULL;
		chip->fcc_new_mah = -EINVAL;
		
		chip->charge_increase = 0;
		chip->charge_cycles = 0;
		backup_charge_cycle(chip);
		
		discard_backup_fcc_data(chip);
		memset(chip->fcc_learning_samples, 0,
			chip->min_fcc_learning_samples *
				sizeof(struct fcc_sample));
	}
}

int pm8941_bms_batt_full_fake_ocv(void)
{
	if (!the_chip) {
		pr_info("called before initialization\n");
		return -EINVAL;
	}

	pr_info("ui_soc=%d, raw_soc=%d\n", store_soc_ui, bms_dbg.raw_soc);
	if (the_chip->enable_batt_full_fake_ocv && store_soc_ui == 100
		&& bms_dbg.raw_soc >= 97)
		the_chip->batt_full_fake_ocv = true;

	return 0;
}

#define OCV_RAW_UNINITIALIZED	0xFFFF
#define MIN_OCV_UV		2000000
static int read_soc_params_raw(struct qpnp_bms_chip *chip,
				struct raw_soc_params *raw,
				int batt_temp)
{
	int warm_reset;
	int rc;

	mutex_lock(&chip->bms_output_lock);

	lock_output_data(chip);

	rc = qpnp_read_wrapper(chip, (u8 *)&raw->last_good_ocv_raw,
			chip->base + BMS1_OCV_FOR_SOC_DATA0, 2);
	if (rc) {
		pr_err("Error reading ocv: rc = %d\n", rc);
		rc = -ENXIO;
		goto error_handle;
	}

	rc = read_cc_raw(chip, &raw->cc, CC);
	if (rc) {
		pr_err("Failed to read raw cc data, rc = %d\n", rc);
		goto error_handle;
	}
	rc = read_cc_raw(chip, &raw->shdw_cc, SHDW_CC);
	if (rc) {
		pr_err("Failed to read raw shdw_cc data, rc = %d\n", rc);
		goto error_handle;
	}

	unlock_output_data(chip);
	mutex_unlock(&chip->bms_output_lock);

	if (chip->prev_last_good_ocv_raw == OCV_RAW_UNINITIALIZED) {
		convert_and_store_ocv(chip, raw, batt_temp, true);
		warm_reset = qpnp_pon_is_warm_reset();
		pr_info("PON_OCV_UV=%d, cc=%llx, warm_reset=%d, ocv_backup=%d, "
				"cc_backup=%d\n",
			chip->last_ocv_uv, raw->cc, warm_reset, chip->ocv_backup_uv,
			chip->cc_backup_uah);

		bms_dbg.last_ocv_raw_uv = raw->last_good_ocv_uv;

#if 0 
		if (read_backup_ocv_uv() > 0
				&& warm_reset > 0) {
			chip->cc_backup_uah = read_backup_cc_uah();
			raw->last_good_ocv_uv = chip->last_ocv_uv = read_backup_ocv_uv();
		} else {
			write_backup_ocv_uv(0);
			write_backup_cc_uah(0);
		}
#endif
		
		if (chip->ocv_backup_uv) {
			if (chip->ocv_reading_at_100 > 0)
				chip->last_ocv_uv = chip->max_voltage_uv;
			else
				raw->last_good_ocv_uv = chip->last_ocv_uv = chip->ocv_backup_uv;
		}

		pr_info("last_good_ocv_raw=0x%x,last_good_ocv_uv/ori=%duV/%duV,"
				"ocv_reading_at_100=%x,cc_backup=%d,ocv_backup=%d,last_ocv_uv=%d\n",
				raw->last_good_ocv_raw, raw->last_good_ocv_uv,
				bms_dbg.last_ocv_raw_uv, chip->ocv_reading_at_100,
				chip->cc_backup_uah, chip->ocv_backup_uv, chip->last_ocv_uv);
		#if 0 
		if (raw->last_good_ocv_uv < MIN_OCV_UV
				|| warm_reset > 0) {
			pr_debug("OCV is stale or bad, estimating new OCV.\n");
			chip->last_ocv_uv = estimate_ocv(chip, batt_temp);
			bms_dbg.pon_est_ocv = chip->last_ocv_uv;
			raw->last_good_ocv_uv = chip->last_ocv_uv;
			reset_cc(chip, CLEAR_CC | CLEAR_SHDW_CC);
			pr_info("New PON_OCV_UV = %d, cc = %llx\n",
					chip->last_ocv_uv, raw->cc);
		}
		#endif
	} else if (chip->new_battery) {
		
		reset_for_new_battery(chip, batt_temp);
		raw->cc = 0;
		raw->shdw_cc = 0;
		raw->last_good_ocv_uv = chip->last_ocv_uv;
		chip->new_battery = false;
	} else if (chip->done_charging) {
		chip->done_charging = false;
		
		chip->ocv_reading_at_100 = raw->last_good_ocv_raw;
		chip->last_ocv_uv = chip->max_voltage_uv;
		raw->last_good_ocv_uv = chip->max_voltage_uv;
		raw->cc = 0;
		raw->shdw_cc = 0;
		reset_cc(chip, CLEAR_CC | CLEAR_SHDW_CC);
		chip->last_ocv_temp = batt_temp;
		chip->software_cc_uah = 0;
		chip->software_shdw_cc_uah = 0;
		chip->last_cc_uah = INT_MIN;
		pr_info("EOC Battery full ocv_reading = 0x%x\n",
				chip->ocv_reading_at_100);
	} else if (chip->prev_last_good_ocv_raw != raw->last_good_ocv_raw) {
		convert_and_store_ocv(chip, raw, batt_temp, false);
		
		htc_batt_bms_timer.no_ocv_update_period_ms = 0;
		if(chip->criteria_sw_est_ocv == FIRST_SW_EST_OCV_THR_MS) {
			rc = of_property_read_u32(chip->spmi->dev.of_node,
				"qcom,criteria-sw-est-ocv",
				&chip->criteria_sw_est_ocv);
			if (rc) {
				pr_err("err:%d, criteria-sw-est-ocv missing in dt, set default value\n", rc);
				chip->criteria_sw_est_ocv = DEFAULT_SW_EST_OCV_THR_MS;
			}
		}
		
		chip->last_cc_uah = INT_MIN;

		store_emmc.store_ocv_uv = bms_dbg.last_ocv_raw_uv = raw->last_good_ocv_uv;

		
		chip->ocv_backup_uv = 0;
		chip->cc_backup_uah = 0;
#if 0 
		
		write_backup_cc_uah(0);
		write_backup_ocv_uv(0);
#endif
		
		chip->batt_full_fake_ocv = false;
		store_start_soc = 0;
		store_start_cc = 0;
		store_start_pc = 0;
		store_start_real_soc = 0;
		pr_info("Reset the Store CC, since SW OCV update.\n");
	} else if (chip->batt_full_fake_ocv) {
		chip->batt_full_fake_ocv = false;
		
		chip->cc_backup_uah = bms_dbg.ori_cc_uah = calculate_cc(chip, raw->cc, CC, NORESET);
		
		chip->ocv_reading_at_100 = raw->last_good_ocv_raw;
		raw->last_good_ocv_uv = chip->last_ocv_uv = chip->max_voltage_uv;
		pr_info("Fake full ocv_reading_at_100=0x%x, last_ocv_uv=%d, cc_backup_uah=%d\n",
				chip->ocv_reading_at_100, chip->last_ocv_uv, chip->cc_backup_uah);
		
		store_emmc.store_ocv_uv = chip->last_ocv_uv;
		store_emmc.store_cc_uah = 0; 
#if 0 
		
		write_backup_cc_uah(chip->cc_backup_uah);
		write_backup_ocv_uv(chip->last_ocv_uv);
#endif
	} else {
		store_emmc.store_ocv_uv = raw->last_good_ocv_uv = chip->last_ocv_uv;
	}

	
	if (chip->ocv_reading_at_100 != raw->last_good_ocv_raw)
		chip->ocv_reading_at_100 = OCV_RAW_UNINITIALIZED;

	pr_debug("last_good_ocv_raw= 0x%x, last_good_ocv_uv= %duV\n",
			raw->last_good_ocv_raw, raw->last_good_ocv_uv);

	pr_debug("cc_raw= 0x%llx\n", raw->cc);
	return 0;

	error_handle:
		mutex_unlock(&chip->bms_output_lock);
		return rc;
}

static int calculate_pc(struct qpnp_bms_chip *chip, int ocv_uv,
							int batt_temp)
{
	int pc;

	pc = interpolate_pc(chip->pc_temp_ocv_lut,
			batt_temp, ocv_uv / 1000);
	pr_debug("pc = %u %% for ocv = %d uv batt_temp = %d\n",pc, ocv_uv, batt_temp);
	
	return pc;
}

static int calculate_fcc(struct qpnp_bms_chip *chip, int batt_temp)
{
	int fcc_uah;

	if (chip->adjusted_fcc_temp_lut == NULL) {
		
		fcc_uah = interpolate_fcc(chip->fcc_temp_lut,
						batt_temp) * 1000;
		pr_debug("fcc = %d uAh\n", fcc_uah);
		return fcc_uah;
	} else {
		return 1000 * interpolate_fcc(chip->adjusted_fcc_temp_lut,
				batt_temp);
	}
}

static int calculate_ocv_charge(struct qpnp_bms_chip *chip,
						struct raw_soc_params *raw,
						int fcc_uah)
{
	int  ocv_uv, pc;

	ocv_uv = raw->last_good_ocv_uv;
	pc = calculate_pc(chip, ocv_uv, chip->last_ocv_temp);
	bms_dbg.rc_pc = pc;
	pr_debug("ocv_uv = %d pc = %d\n", ocv_uv, pc);
	return (fcc_uah * pc) / 100;
}

#define CC_READING_TICKS	56
#define SLEEP_CLK_HZ		32764
#define SECONDS_PER_HOUR	3600

static s64 cc_uv_to_pvh(s64 cc_uv)
{
	return div_s64(cc_uv * CC_READING_TICKS * 100000,
			SLEEP_CLK_HZ * SECONDS_PER_HOUR) * 10;
}

static int calculate_cc(struct qpnp_bms_chip *chip, int64_t cc,
					int cc_type, int clear_cc)
{
	struct qpnp_iadc_calib calibration;
	struct qpnp_vadc_result result;
	int64_t cc_voltage_uv, cc_pvh, cc_uah, *software_counter;
	int rc;

	software_counter = cc_type == SHDW_CC ?
			&chip->software_shdw_cc_uah : &chip->software_cc_uah;
	rc = qpnp_vadc_read(chip->vadc_dev, DIE_TEMP, &result);
	if (rc) {
		pr_err("could not read pmic die temperature: %d\n", rc);
		return *software_counter;
	}

	qpnp_iadc_get_gain_and_offset(chip->iadc_dev, &calibration);
	pr_debug("%scc = %lld, die_temp = %lld\n",cc_type == SHDW_CC ? "shdw_" : "", cc, result.physical);
	cc_voltage_uv = cc_reading_to_uv(cc);
	cc_voltage_uv = cc_adjust_for_gain(cc_voltage_uv,
					calibration.gain_raw
					- calibration.offset_raw);
	cc_pvh = cc_uv_to_pvh(cc_voltage_uv);
	cc_uah = div_s64(cc_pvh, chip->r_sense_uohm);
	rc = qpnp_iadc_comp_result(chip->iadc_dev, &cc_uah);
	if (rc)
		pr_debug("error compensation failed: %d\n", rc);
	if (clear_cc == RESET) {
		pr_info("software_%scc = %lld, added cc_uah = %lld\n",
				cc_type == SHDW_CC ? "sw_" : "",
				*software_counter, cc_uah);
		*software_counter += cc_uah;
		reset_cc(chip, cc_type == SHDW_CC ? CLEAR_SHDW_CC : CLEAR_CC);
		return (int)*software_counter;
	} else {
		pr_debug("software_%scc = %lld, cc_uah = %lld(%lld), total = %lld\n",
				cc_type == SHDW_CC ? "shdw_" : "",
				*software_counter, cc_uah,pre_cc_uah,
				*software_counter + cc_uah);
		if(gbFCC_Start){
			if(cc_type != SHDW_CC){
				if(pre_cc_uah == 0){
					pre_cc_uah = cc_uah;
				}else{
					if(cc_uah < pre_cc_uah)
						pre_cc_uah = cc_uah;
				}
			}
		}
		return *software_counter + cc_uah;
	}
}

static int get_rbatt(struct qpnp_bms_chip *chip,
					int soc_rbatt_mohm, int batt_temp)
{
	int rbatt_mohm, scalefactor;

	rbatt_mohm = chip->default_rbatt_mohm;
	if (chip->rbatt_sf_lut == NULL)  {
		pr_debug("RBATT = %d\n", rbatt_mohm);
		return rbatt_mohm;
	}
	
	
	if (soc_rbatt_mohm > 100)
		scalefactor = interpolate_scalingfactor(chip->rbatt_sf_lut,
							batt_temp, 100);
	else
	scalefactor = interpolate_scalingfactor(chip->rbatt_sf_lut,
						batt_temp, soc_rbatt_mohm);
	bms_dbg.rbatt_sf = scalefactor;

	rbatt_mohm = (rbatt_mohm * scalefactor) / 100;

	rbatt_mohm += chip->r_conn_mohm;
	rbatt_mohm += chip->rbatt_capacitive_mohm;

	return rbatt_mohm;
}

#define IAVG_MINIMAL_TIME	2
static void calculate_iavg(struct qpnp_bms_chip *chip, int cc_uah,
				int *iavg_ua, int delta_time_s)
{
	int delta_cc_uah = 0;

	if (delta_time_s < IAVG_MINIMAL_TIME
			|| chip->last_cc_uah == INT_MIN) {
		get_battery_current(chip, iavg_ua);
		goto out;
	}

	delta_cc_uah = cc_uah - chip->last_cc_uah;

	*iavg_ua = div_s64((s64)delta_cc_uah * 3600, delta_time_s);

out:
	pr_debug("delta_cc = %d iavg_ua = %d, cc_uah = %d, last_cc_uah=%d\n", delta_cc_uah, (int)*iavg_ua, cc_uah, chip->last_cc_uah);

	
	chip->last_cc_uah = cc_uah;
}

#if !(defined(CONFIG_HTC_BATT_8960))
static int calculate_termination_uuc(struct qpnp_bms_chip *chip,
					struct soc_params *params,
					int batt_temp, int uuc_iavg_ma,
					int *ret_pc_unusable)
{
	int unusable_uv, pc_unusable, uuc_uah;
	int i = 0;
	int ocv_mv;
	int rbatt_mohm;
	int delta_uv;
	int prev_delta_uv = 0;
	int prev_rbatt_mohm = 0;
	int uuc_rbatt_mohm;

	for (i = 0; i <= 100; i++) {
		ocv_mv = interpolate_ocv(chip->pc_temp_ocv_lut,
				batt_temp, i);
		rbatt_mohm = get_rbatt(chip, i, batt_temp);
		unusable_uv = (rbatt_mohm * uuc_iavg_ma)
							+ (chip->v_cutoff_uv);
		delta_uv = ocv_mv * 1000 - unusable_uv;

		if (delta_uv > 0)
			break;

		prev_delta_uv = delta_uv;
		prev_rbatt_mohm = rbatt_mohm;
	}

	uuc_rbatt_mohm = linear_interpolate(rbatt_mohm, delta_uv,
					prev_rbatt_mohm, prev_delta_uv,
					0);

	unusable_uv = (uuc_rbatt_mohm * uuc_iavg_ma) + (chip->v_cutoff_uv);
	bms_dbg.uuc_rbatt_mohm = uuc_rbatt_mohm;
	bms_dbg.unusable_uv = unusable_uv;

	pc_unusable = calculate_pc(chip, unusable_uv, batt_temp);
	uuc_uah = (params->fcc_uah * pc_unusable) / 100;
	pr_debug("For uuc_iavg_ma = %d, unusable_rbatt = %d unusable_uv = %d unusable_pc = %d rbatt_pc = %d uuc = %d\n",
					uuc_iavg_ma,
					uuc_rbatt_mohm, unusable_uv,
					pc_unusable, i, uuc_uah);
	*ret_pc_unusable = pc_unusable;
	return uuc_uah;
}

#define TIME_PER_PERCENT_UUC			60
static int adjust_uuc(struct qpnp_bms_chip *chip,
			struct soc_params *params,
			int new_pc_unusable,
			int new_uuc_uah,
			int batt_temp)
{
	int new_unusable_mv, new_iavg_ma;
	int max_percent_change;

	max_percent_change = max(params->delta_time_s
				/ TIME_PER_PERCENT_UUC, 1);

	if (chip->prev_pc_unusable == -EINVAL
		|| abs(chip->prev_pc_unusable - new_pc_unusable)
			<= max_percent_change) {
		chip->prev_pc_unusable = new_pc_unusable;
		return new_uuc_uah;
	}

	
	if (new_pc_unusable > chip->prev_pc_unusable)
		chip->prev_pc_unusable += max_percent_change;
	else
		chip->prev_pc_unusable -= max_percent_change;

	new_uuc_uah = (params->fcc_uah * chip->prev_pc_unusable) / 100;

	
	new_unusable_mv = interpolate_ocv(chip->pc_temp_ocv_lut,
			batt_temp, chip->prev_pc_unusable);
	if (new_unusable_mv < chip->v_cutoff_uv/1000)
		new_unusable_mv = chip->v_cutoff_uv/1000;

	new_iavg_ma = (new_unusable_mv * 1000 - chip->v_cutoff_uv)
						/ params->rbatt_mohm;
	if (new_iavg_ma == 0)
		new_iavg_ma = 1;
	chip->prev_uuc_iavg_ma = new_iavg_ma;
	pr_debug("Restricting UUC to %d (%d%%) unusable_mv = %d iavg_ma = %d\n",
					new_uuc_uah, chip->prev_pc_unusable,
					new_unusable_mv, new_iavg_ma);

	return new_uuc_uah;
}
#endif

#define MIN_IAVG_MA 250
#if !(defined(CONFIG_HTC_BATT_8960))
static int calculate_unusable_charge_uah(struct qpnp_bms_chip *chip,
					struct soc_params *params,
					int batt_temp)
{
	int uuc_uah_iavg;
	int i;
	int uuc_iavg_ma = params->iavg_ua / 1000;
	int pc_unusable;

	if (chip->first_time_calc_uuc && chip->shutdown_iavg_ma != 0) {
		pr_debug("Using shutdown_iavg_ma = %d in all samples\n",
				chip->shutdown_iavg_ma);
		for (i = 0; i < IAVG_SAMPLES; i++)
			chip->iavg_samples_ma[i] = chip->shutdown_iavg_ma;

		chip->iavg_index = 0;
		chip->iavg_num_samples = IAVG_SAMPLES;
	}

	if (params->delta_time_s >= IAVG_MINIMAL_TIME) {
		if (uuc_iavg_ma < MIN_IAVG_MA)
			uuc_iavg_ma = MIN_IAVG_MA;
		chip->iavg_samples_ma[chip->iavg_index] = uuc_iavg_ma;
		chip->iavg_index = (chip->iavg_index + 1) % IAVG_SAMPLES;
		chip->iavg_num_samples++;
		if (chip->iavg_num_samples >= IAVG_SAMPLES)
			chip->iavg_num_samples = IAVG_SAMPLES;
	}

	
	uuc_iavg_ma = 0;
	if (chip->iavg_num_samples != 0) {
		for (i = 0; i < chip->iavg_num_samples; i++) {
			pr_debug("iavg_samples_ma[%d] = %d\n", i,
					chip->iavg_samples_ma[i]);
			uuc_iavg_ma += chip->iavg_samples_ma[i];
		}

		uuc_iavg_ma = DIV_ROUND_CLOSEST(uuc_iavg_ma,
						chip->iavg_num_samples);
	}

	if (bms_reset)
		return (params->fcc_uah * 3) / 100;

	uuc_uah_iavg = calculate_termination_uuc(chip, params, batt_temp,
						uuc_iavg_ma, &pc_unusable);
	bms_dbg.pc_unusable = pc_unusable;
	bms_dbg.uuc_iavg_ma = uuc_iavg_ma;
	bms_dbg.ori_uuc_uah = uuc_uah_iavg;
	pr_debug("uuc_iavg_ma = %d uuc with iavg = %d\n",
						uuc_iavg_ma, uuc_uah_iavg);

	chip->prev_uuc_iavg_ma = uuc_iavg_ma;
	
	uuc_uah_iavg = adjust_uuc(chip, params, pc_unusable,
					uuc_uah_iavg, batt_temp);
	return uuc_uah_iavg;
}
#else
#define IBAT_FOR_UC_MA	2000
static int calculate_unusable_charge_uah(struct qpnp_bms_chip *chip,
					struct soc_params *params,
					int batt_temp)
{
	int voltage_unusable_uv, pc_unusable;

	
	voltage_unusable_uv = (params->rbatt_mohm * IBAT_FOR_UC_MA)
						+ chip->v_cutoff_uv;
	pc_unusable = calculate_pc(chip, voltage_unusable_uv,
						batt_temp);
	pr_debug("rbatt = %umilliOhms unusable_v =%d unusable_pc = %d\n",
			params->rbatt_mohm, voltage_unusable_uv, pc_unusable);
	bms_dbg.unusable_uv = voltage_unusable_uv;
	bms_dbg.pc_unusable = pc_unusable;
	return (params->fcc_uah* pc_unusable) / 100;
}
#endif

static s64 find_ocv_charge_for_soc(struct qpnp_bms_chip *chip,
				struct soc_params *params, int soc)
{
	return div_s64((s64)soc * (params->fcc_uah - params->uuc_uah),
			100) + params->cc_uah + params->uuc_uah;
}

static int find_pc_for_soc(struct qpnp_bms_chip *chip,
			struct soc_params *params, int soc)
{
	int ocv_charge_uah = find_ocv_charge_for_soc(chip, params, soc);
	int pc;

	pc = DIV_ROUND_CLOSEST((int)ocv_charge_uah * 100, params->fcc_uah);
	pc = clamp(pc, 0, 100);
	pr_debug("soc = %d, fcc = %d uuc = %d rc = %d pc = %d\n",
			soc, params->fcc_uah, params->uuc_uah,
			ocv_charge_uah, pc);
	return pc;
}

#define SIGN(x) ((x) < 0 ? -1 : 1)
#define UV_PER_SPIN 50000
static int find_ocv_for_pc(struct qpnp_bms_chip *chip, int batt_temp, int pc)
{
	int new_pc;
	int ocv_mv;
	int delta_mv = 5;
	int max_spin_count;
	int count = 0;
	int sign, new_sign;

	ocv_mv = interpolate_ocv(chip->pc_temp_ocv_lut, batt_temp, pc);

	new_pc = interpolate_pc(chip->pc_temp_ocv_lut, batt_temp, ocv_mv);
	pr_debug("test revlookup pc = %d for ocv = %d\n", new_pc, ocv_mv);
	max_spin_count = 1 + (chip->max_voltage_uv - chip->v_cutoff_uv)
						/ UV_PER_SPIN;
	sign = SIGN(pc - new_pc);

	while (abs(new_pc - pc) != 0 && count < max_spin_count) {
		new_sign = SIGN(pc - new_pc);
		if (new_sign != sign) {
			if (delta_mv > 1)
				delta_mv = 1;
			else
				break;
		}
		sign = new_sign;

		ocv_mv = ocv_mv + delta_mv * sign;
		new_pc = interpolate_pc(chip->pc_temp_ocv_lut,
				batt_temp, ocv_mv);
		pr_debug("test revlookup pc = %d for ocv = %d\n",
			new_pc, ocv_mv);
		count++;
	}

	return ocv_mv * 1000;
}

static int get_rbatt_for_estimate_ocv(struct sf_lut *rbatt_lut, int temp)
{
	int x, y, rows, cols;

	rows = rbatt_lut->rows;
	cols = rbatt_lut->cols;
	for (x= 0; x < rows; x++) {
		for (y= 0; y < cols; y++) {
			if (temp < rbatt_lut->row_entries[y])
				return rbatt_lut->sf[x][y];
		}
	}
	return rbatt_lut->sf[rows-1][cols-1];
}

static int estimate_sw_ocv(struct qpnp_bms_chip *chip, int ibatt_ua, int vbat_uv)
{
	int ocv_est_uv, batt_temp, rc;
	int rbatt_mohm, rbatt_for_estimated_ocv;
	struct qpnp_vadc_result result;

	if (!chip) {
		pr_info("[EST]called before initialization\n");
		return -EINVAL;
	}
	
	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM,
								&result);
	if (rc) {
		pr_info("[EST]error reading adc channel = %d, rc = %d\n",
					LR_MUX1_BATT_THERM, rc);
		return rc;
	}
	pr_debug("batt_temp phy = %lld, meas = 0x%llx\n", result.physical, result.measurement);
	batt_temp = (int)result.physical;

	
	if (batt_temp <= 0) {
		pr_info("[EST]batt_temp=%d, return!\n", batt_temp);
		return 0;
	}
	
	if (chip->rbatt_est_ocv_lut == NULL)  {
		pr_info("[EST]rbatt_est_ocv_lut is NULL\n");
		return 0;
	}
	rbatt_for_estimated_ocv = get_rbatt_for_estimate_ocv(chip->rbatt_est_ocv_lut,
		batt_temp/10);
	rbatt_mohm = rbatt_for_estimated_ocv + chip->rconn_mohm_sw_est_ocv;

	ocv_est_uv = vbat_uv + (ibatt_ua * rbatt_mohm) / 1000;
	pr_info("[EST]estimated ocv=%d, rbatt=%d, rconn=%d, ibatt_ua=%d, vbat_uv=%d, "
			"ori last_ocv_uv=%d, no_hw_ocv_ms=%lu\n",
			ocv_est_uv, rbatt_for_estimated_ocv, chip->rconn_mohm_sw_est_ocv,
			ibatt_ua, vbat_uv, chip->last_ocv_uv,
			htc_batt_bms_timer.no_ocv_update_period_ms);
	return ocv_est_uv;
}

static int pm8941_bms_estimate_ocv(void)
{
	int	rc;
	int	estimated_ocv_uv = 0;
	int ibatt_ua, vbat_uv;
	int soc_by_sw_ocv, prev_raw_soc, prev_last_ocv_uv, prev_cc_backup_uah;
	struct raw_soc_params raw;

	if (!the_chip) {
		pr_info("[EST] called before initialization\n");
		return -EINVAL;
	}

	
	rc = get_simultaneous_batt_v_and_i(the_chip,
							&ibatt_ua, &vbat_uv);
	if (rc) {
		pr_err("[EST]simultaneous failed rc = %d\n", rc);
		return rc;
	}

	
	if (ibatt_ua > 60000 && !is_do_sw_ocv_in_eoc) {
		pr_info("[EST]ibatt_ua=%d uA exceed 60mA, "
			       "no_hw_ocv_ms=%lu, return!\n", ibatt_ua,
				htc_batt_bms_timer.no_ocv_update_period_ms);
		return 0;
	}

	mutex_lock(&the_chip->bms_output_lock);
	lock_output_data(the_chip);

	rc = qpnp_read_wrapper(the_chip, (u8 *)&raw.last_good_ocv_raw,
			the_chip->base + BMS1_OCV_FOR_SOC_DATA0, 2);
	if (rc) {
		pr_info("[EST]error reading ocv: rc = %d, return!\n", rc);
		return 0;
	}

	rc = read_cc_raw(the_chip, &raw.cc, CC);
	if (rc) {
		pr_info("[EST]failed to read raw cc data, rc = %d\n", rc);
		return 0;
	}
	rc = read_cc_raw(the_chip, &raw.shdw_cc, SHDW_CC);
	if (rc) {
		pr_info("[EST]failed to read raw shdw_cc data, rc = %d\n", rc);
		return 0;
	}

	unlock_output_data(the_chip);
	mutex_unlock(&the_chip->bms_output_lock);

	if (the_chip->prev_last_good_ocv_raw != raw.last_good_ocv_raw) {
		pr_info("[EST]ocv is updated by hw, pre_ocv_raw=%x, last_ocv_raw=%x, "
				"no_hw_ocv_ms=%lu\n",
				the_chip->prev_last_good_ocv_raw, raw.last_good_ocv_raw,
				htc_batt_bms_timer.no_ocv_update_period_ms);
		return 0;
	}

	estimated_ocv_uv = estimate_sw_ocv(the_chip, ibatt_ua, vbat_uv);

	
	
	if(the_chip->soc_overrate_criteria_for_sw_ocv) {
		
		prev_last_ocv_uv = the_chip->last_ocv_uv;
		prev_cc_backup_uah = the_chip->cc_backup_uah;
		prev_raw_soc = bms_dbg.raw_soc;

		
		the_chip->last_ocv_uv = estimated_ocv_uv;
		the_chip->cc_backup_uah = calculate_cc(the_chip, raw.cc, CC, NORESET);
		soc_by_sw_ocv = pm8941_bms_get_percent_charge(the_chip);

		if(soc_by_sw_ocv - prev_raw_soc >= the_chip->soc_overrate_criteria_for_sw_ocv) {
			pr_info("[EST]New SoC overrate %d%% after SW OCV, skip it! "
				"raw_soc/ori=%d/%d, last_ocv_uv/ori=%d/%d, cc_backup_uah/ori=%d/%d\n",
				the_chip->soc_overrate_criteria_for_sw_ocv,
				soc_by_sw_ocv,
				prev_raw_soc,
				the_chip->last_ocv_uv,
				prev_last_ocv_uv,
				the_chip->cc_backup_uah,
				prev_cc_backup_uah);
			
			mutex_lock(&the_chip->bms_output_lock);
			the_chip->last_ocv_uv = prev_last_ocv_uv;
			the_chip->cc_backup_uah = prev_cc_backup_uah;
			mutex_unlock(&the_chip->bms_output_lock);

			return 0;
		}
	}
	
	if (estimated_ocv_uv > 0) {
		is_do_sw_ocv_in_eoc = 0;
		
		store_emmc.store_ocv_uv = the_chip->last_ocv_uv = estimated_ocv_uv;
		the_chip->cc_backup_uah = bms_dbg.ori_cc_uah = calculate_cc(the_chip, raw.cc, CC, NORESET);
		store_emmc.store_cc_uah = 0; 
#if 0 
		
		write_backup_cc_uah(the_chip->cc_backup_uah);
		write_backup_ocv_uv(the_chip->last_ocv_uv);
#endif
		
		htc_batt_bms_timer.no_ocv_update_period_ms = 0;
		
		if(the_chip->criteria_sw_est_ocv == FIRST_SW_EST_OCV_THR_MS) {
			rc = of_property_read_u32(the_chip->spmi->dev.of_node,
				"qcom,criteria-sw-est-ocv",
				&the_chip->criteria_sw_est_ocv);
			if (rc) {
				pr_err("err:%d, criteria-sw-est-ocv missing in dt, set default value\n", rc);
				the_chip->criteria_sw_est_ocv = DEFAULT_SW_EST_OCV_THR_MS;
			}
		}

		pr_info("[EST]last_ocv=%d, ori_cc_uah=%d, backup_cc=%d, "
			"no_hw_ocv_ms=%ld, criteria_sw_est_ocv=%d\n",
			the_chip->last_ocv_uv, bms_dbg.ori_cc_uah, the_chip->cc_backup_uah,
			htc_batt_bms_timer.no_ocv_update_period_ms,
			the_chip->criteria_sw_est_ocv);
	}
	return estimated_ocv_uv;
}

static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}

static int get_prop_bms_batt_resistance(struct qpnp_bms_chip *chip)
{
	return chip->rbatt_mohm * 1000;
}

static int get_prop_bms_current_now(struct qpnp_bms_chip *chip)
{
	int rc, result_ua;

	rc = get_battery_current(chip, &result_ua);
	if (rc) {
		pr_err("failed to get current: %d\n", rc);
		return rc;
	}
	return result_ua;
}

static int get_prop_bms_charge_counter(struct qpnp_bms_chip *chip)
{
	int64_t cc_raw;

	mutex_lock(&chip->bms_output_lock);
	lock_output_data(chip);
	read_cc_raw(chip, &cc_raw, false);
	unlock_output_data(chip);
	mutex_unlock(&chip->bms_output_lock);

	return calculate_cc(chip, cc_raw, CC, NORESET);
}

static int get_prop_bms_charge_counter_shadow(struct qpnp_bms_chip *chip)
{
	int64_t cc_raw;

	mutex_lock(&chip->bms_output_lock);
	lock_output_data(chip);
	read_cc_raw(chip, &cc_raw, true);
	unlock_output_data(chip);
	mutex_unlock(&chip->bms_output_lock);

	return calculate_cc(chip, cc_raw, SHDW_CC, NORESET);
}

static int get_prop_bms_charge_full_design(struct qpnp_bms_chip *chip)
{
	return chip->fcc_mah * 1000;
}

static int get_prop_bms_charge_full(struct qpnp_bms_chip *chip)
{
	int rc;
	struct qpnp_vadc_result result;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &result);
	if (rc) {
		pr_err("Unable to read battery temperature\n");
		return rc;
	}

	return calculate_fcc(chip, (int)result.physical);
}

static int calculate_delta_time(unsigned long *time_stamp, int *delta_time_s)
{
	unsigned long now_tm_sec = 0;

	
	*delta_time_s = 0;

	if (get_current_time(&now_tm_sec)) {
		pr_err("RTC read failed, delta_s = %d\n", *delta_time_s);
		return 0;
	}

	*delta_time_s = (now_tm_sec - *time_stamp);
	pr_debug("time_stamp = %ld, now_tm_sec = %ld, delta_s = %d\n",
		*time_stamp, now_tm_sec, *delta_time_s);
	
	*time_stamp = now_tm_sec;
	return 0;
}

static void calculate_soc_params(struct qpnp_bms_chip *chip,
						struct raw_soc_params *raw,
						struct soc_params *params,
						int batt_temp)
{
	int soc_rbatt, shdw_cc_uah;

	calculate_delta_time(&chip->tm_sec, &params->delta_time_s);
	pr_debug("tm_sec = %ld, delta_s = %d\n",
		chip->tm_sec, params->delta_time_s);
	params->fcc_uah = calculate_fcc(chip, batt_temp);
	pr_debug("FCC = %uuAh batt_temp = %d\n", params->fcc_uah, batt_temp);

	
	params->ocv_charge_uah = calculate_ocv_charge(
						chip, raw,
						params->fcc_uah);
	pr_debug("ocv_charge_uah = %uuAh\n", params->ocv_charge_uah);

	
	params->cc_uah = bms_dbg.ori_cc_uah = calculate_cc(chip, raw->cc, CC, NORESET);
	store_emmc.store_cc_uah = params->cc_uah -= chip->cc_backup_uah;
	pr_debug("cc_uah = %d. after subtracting %d cc_uah = %d\n",
					bms_dbg.ori_cc_uah, chip->cc_backup_uah,
					params->cc_uah);
	shdw_cc_uah = calculate_cc(chip, raw->shdw_cc, SHDW_CC, NORESET);
	pr_debug("cc_uah = %duAh raw->cc = %llx, shdw_cc_uah = %duAh raw->shdw_cc = %llx\n",
			params->cc_uah, raw->cc,
			shdw_cc_uah, raw->shdw_cc);

	soc_rbatt = ((params->ocv_charge_uah - params->cc_uah) * 100)
							/ params->fcc_uah;
	bms_dbg.soc_rbatt = soc_rbatt;
	if (soc_rbatt < 0)
		soc_rbatt = 0;
	params->rbatt_mohm = get_rbatt(chip, soc_rbatt, batt_temp);
	bms_dbg.rbatt = params->rbatt_mohm;
	pr_debug("rbatt_mohm = %d\n", params->rbatt_mohm);

	if (params->rbatt_mohm != chip->rbatt_mohm) {
		chip->rbatt_mohm = params->rbatt_mohm;
		if (chip->bms_psy_registered)
			power_supply_changed(&chip->bms_psy);
	}

	calculate_iavg(chip, params->cc_uah, &params->iavg_ua,
						params->delta_time_s);

	params->uuc_uah = calculate_unusable_charge_uah(chip, params,
							batt_temp);
	pr_debug("UUC = %uuAh\n", params->uuc_uah);
}

static int bound_soc(int soc)
{
	soc = max(0, soc);
	soc = min(100, soc);
	return soc;
}

#define IBAT_TOL_MASK		0x0F
#define OCV_TOL_MASK		0xF0
#define IBAT_TOL_DEFAULT	0x03
#define IBAT_TOL_NOCHG		0x0F
#define OCV_TOL_DEFAULT		0x20
#define OCV_TOL_NO_OCV		0x00
static int stop_ocv_updates(struct qpnp_bms_chip *chip)
{
	pr_debug("stopping ocv updates\n");
	return qpnp_masked_write(chip, BMS1_TOL_CTL,
			OCV_TOL_MASK, OCV_TOL_NO_OCV);
}

static int reset_bms_for_test(struct qpnp_bms_chip *chip)
{
	int ibat_ua = 0, vbat_uv = 0, rc;
	int ocv_est_uv;

	if (!chip) {
		pr_err("BMS driver has not been initialized yet!\n");
		return -EINVAL;
	}

	rc = get_simultaneous_batt_v_and_i(chip, &ibat_ua, &vbat_uv);

	ocv_est_uv = vbat_uv + (ibat_ua * chip->r_conn_mohm) / 1000;
	pr_info("forcing ocv to be %d due to bms reset mode\n", ocv_est_uv);
	chip->last_ocv_uv = ocv_est_uv;
	mutex_lock(&chip->last_soc_mutex);
	chip->last_soc = -EINVAL;
	chip->last_soc_invalid = true;
	mutex_unlock(&chip->last_soc_mutex);
	reset_cc(chip, CLEAR_CC | CLEAR_SHDW_CC);
	chip->software_cc_uah = 0;
	chip->software_shdw_cc_uah = 0;
	chip->last_cc_uah = INT_MIN;
	stop_ocv_updates(chip);

	pr_debug("bms reset to ocv = %duv vbat_ua = %d ibat_ua = %d\n",
			chip->last_ocv_uv, vbat_uv, ibat_ua);

	return rc;
}

static int bms_reset_set(const char *val, const struct kernel_param *kp)
{
	int rc;

	rc = param_set_bool(val, kp);
	if (rc) {
		pr_err("Unable to set bms_reset: %d\n", rc);
		return rc;
	}

	if (*(bool *)kp->arg) {
#if !(defined(CONFIG_HTC_BATT_8960))
		struct power_supply *bms_psy = power_supply_get_by_name("bms");
		struct qpnp_bms_chip *chip = container_of(bms_psy,
					struct qpnp_bms_chip, bms_psy);

		rc = reset_bms_for_test(chip);
#else
		rc = reset_bms_for_test(the_chip);
#endif
		if (rc) {
			pr_err("Unable to modify bms_reset: %d\n", rc);
			return rc;
		}
	}
	return 0;
}

static struct kernel_param_ops bms_reset_ops = {
	.set = bms_reset_set,
	.get = param_get_bool,
};

module_param_cb(bms_reset, &bms_reset_ops, &bms_reset, 0644);

#define SOC_STORAGE_MASK	0xFE
static void backup_soc_and_iavg(struct qpnp_bms_chip *chip, int batt_temp,
				int soc)
{
	u8 temp;
	int rc;
	int iavg_ma = chip->prev_uuc_iavg_ma;

	if (iavg_ma > MIN_IAVG_MA)
		temp = (iavg_ma - MIN_IAVG_MA) / IAVG_STEP_SIZE_MA;
	else
		temp = 0;

	rc = qpnp_write_wrapper(chip, &temp, chip->base + IAVG_STORAGE_REG, 1);

	
	if (batt_temp > IGNORE_SOC_TEMP_DECIDEG)
		qpnp_masked_write_base(chip, chip->soc_storage_addr,
				SOC_STORAGE_MASK, (soc + 1) << 1);
}

static int scale_soc_while_chg(struct qpnp_bms_chip *chip, int chg_time_sec,
				int catch_up_sec, int new_soc, int prev_soc)
{
	int scaled_soc;
	int numerator;

	pr_debug("cts = %d catch_up_sec = %d\n", chg_time_sec, catch_up_sec);
	if (catch_up_sec == 0)
		return new_soc;

	if (chg_time_sec > catch_up_sec)
		return new_soc;

	numerator = (catch_up_sec - chg_time_sec) * prev_soc
			+ chg_time_sec * new_soc;
	scaled_soc = numerator / catch_up_sec;

	pr_debug("cts = %d new_soc = %d prev_soc = %d scaled_soc = %d\n",
			chg_time_sec, new_soc, prev_soc, scaled_soc);

	return scaled_soc;
}

static int bms_fake_battery = -EINVAL;
module_param(bms_fake_battery, int, 0644);

static int report_voltage_based_soc(struct qpnp_bms_chip *chip)
{
	pr_debug("Reported voltage based soc = %d\n",
			chip->prev_voltage_based_soc);
	return chip->prev_voltage_based_soc;
}

#define SOC_CATCHUP_SEC_MAX		600
#define SOC_CATCHUP_SEC_PER_PERCENT	60
#define MAX_CATCHUP_SOC	(SOC_CATCHUP_SEC_MAX / SOC_CATCHUP_SEC_PER_PERCENT)
#define SOC_CHANGE_PER_SEC		5
#define REPORT_SOC_WAIT_MS		10000
static int report_cc_based_soc(struct qpnp_bms_chip *chip)
{
	int soc, soc_change;
	int time_since_last_change_sec, charge_time_sec = 0;
	unsigned long last_change_sec;
	struct timespec now;
	struct qpnp_vadc_result result;
	int batt_temp;
	int rc;
	bool charging, charging_since_last_report;

	rc = wait_event_interruptible_timeout(chip->bms_wait_queue,
			chip->calculated_soc != -EINVAL,
			round_jiffies_relative(msecs_to_jiffies
			(REPORT_SOC_WAIT_MS)));

	if (rc == 0 && chip->calculated_soc == -EINVAL) {
		pr_debug("calculate soc timed out\n");
	} else if (rc == -ERESTARTSYS) {
		pr_err("Wait for SoC interrupted.\n");
		return rc;
	}

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &result);

	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					LR_MUX1_BATT_THERM, rc);
		return rc;
	}
	pr_debug("batt_temp phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);
	batt_temp = (int)result.physical;

	mutex_lock(&chip->last_soc_mutex);
	soc = chip->calculated_soc;

	last_change_sec = chip->last_soc_change_sec;
	calculate_delta_time(&last_change_sec, &time_since_last_change_sec);
	bms_dbg.time_last_change_s = time_since_last_change_sec;
	charging = is_battery_charging(chip);
	charging_since_last_report = charging || (chip->last_soc_unbound
			&& chip->was_charging_at_sleep);
	if (charging) {
		if (chip->charge_start_tm_sec == 0) {
			if (abs(soc - chip->last_soc) < MAX_CATCHUP_SOC)
				chip->catch_up_time_sec =
				(soc - chip->last_soc)
					* SOC_CATCHUP_SEC_PER_PERCENT;
			else
				chip->catch_up_time_sec = SOC_CATCHUP_SEC_MAX;

			if (chip->catch_up_time_sec < 0)
				chip->catch_up_time_sec = 0;
			chip->charge_start_tm_sec = last_change_sec;
		}

		charge_time_sec = min(SOC_CATCHUP_SEC_MAX, (int)last_change_sec
				- chip->charge_start_tm_sec);

		
		if (chip->last_soc == soc)
			chip->catch_up_time_sec = 0;
	}

	if (chip->last_soc != -EINVAL) {
		if (chip->last_soc < soc && !charging_since_last_report)
			soc = chip->last_soc;
		else if (chip->last_soc < soc && soc != 100)
			soc = scale_soc_while_chg(chip, charge_time_sec,
					chip->catch_up_time_sec,
					soc, chip->last_soc);

		soc_change = min((int)abs(chip->last_soc - soc),
			time_since_last_change_sec / SOC_CHANGE_PER_SEC);
		bms_dbg.ori_soc_change = soc_change;
		if (chip->last_soc_unbound) {
			chip->last_soc_unbound = false;
		} else {
			soc_change = min(1, soc_change);
		}

		if (soc < chip->last_soc && soc != 0)
			soc = chip->last_soc - soc_change;
		if (soc > chip->last_soc && soc != 100)
			soc = chip->last_soc + soc_change;
	}

	if (chip->last_soc != soc && !chip->last_soc_unbound)
		chip->last_soc_change_sec = last_change_sec;

	pr_info("last_soc = %d, calculated_soc = %d, soc = %d, time since last change = %d,"
			"ori_soc_change = %d, soc_change = %d\n",
			chip->last_soc, chip->calculated_soc,
			soc, time_since_last_change_sec, bms_dbg.ori_soc_change, soc_change);

	chip->last_soc = bound_soc(soc);
	backup_soc_and_iavg(chip, batt_temp, chip->last_soc);
	pr_debug("Reported SOC = %d\n", chip->last_soc);
	chip->t_soc_queried = now;
	mutex_unlock(&chip->last_soc_mutex);

	return soc;
}

static int report_state_of_charge(struct qpnp_bms_chip *chip)
{
	if (bms_fake_battery != -EINVAL) {
		pr_debug("Returning Fake SOC = %d%%\n", bms_fake_battery);
		return bms_fake_battery;
	} else if (chip->use_voltage_soc)
		return report_voltage_based_soc(chip);
	else
		return report_cc_based_soc(chip);
}

#if !(defined(CONFIG_HTC_BATT_8960))
#define VDD_MAX_ERR			5000
#define VDD_STEP_SIZE			10000
#define MAX_COUNT_BEFORE_RESET_TO_CC	3
static int charging_adjustments(struct qpnp_bms_chip *chip,
				struct soc_params *params, int soc,
				int vbat_uv, int ibat_ua, int batt_temp)
{
	int chg_soc, soc_ibat, batt_terminal_uv, weight_ibat, weight_cc;

	batt_terminal_uv = vbat_uv + (ibat_ua * chip->r_conn_mohm) / 1000;

	if (chip->soc_at_cv == -EINVAL) {
		if (batt_terminal_uv >= chip->max_voltage_uv - VDD_MAX_ERR) {
			chip->soc_at_cv = soc;
			chip->prev_chg_soc = soc;
			chip->ibat_at_cv_ua = params->iavg_ua;
			pr_debug("CC_TO_CV ibat_ua = %d CHG SOC %d\n",
					ibat_ua, soc);
		} else {
			
			pr_debug("CC CHG SOC %d\n", soc);
		}

		chip->prev_batt_terminal_uv = batt_terminal_uv;
		chip->system_load_count = 0;
		return soc;
	} else if (ibat_ua > 0 && batt_terminal_uv
			< chip->max_voltage_uv - (VDD_MAX_ERR * 2)) {
		if (chip->system_load_count > MAX_COUNT_BEFORE_RESET_TO_CC) {
			chip->soc_at_cv = -EINVAL;
			pr_debug("Vbat below CV threshold, resetting CC_TO_CV\n");
			chip->system_load_count = 0;
		} else {
			chip->system_load_count += 1;
			pr_debug("Vbat below CV threshold, count: %d\n",
					chip->system_load_count);
		}
		return soc;
	} else if (ibat_ua > 0) {
		pr_debug("NOT CHARGING SOC %d\n", soc);
		chip->system_load_count = 0;
		chip->prev_chg_soc = soc;
		return soc;
	}

	chip->system_load_count = 0;

	if (batt_terminal_uv <= chip->prev_batt_terminal_uv - VDD_STEP_SIZE) {
		pr_debug("batt_terminal_uv %d < (max = %d - 10000); CC CHG SOC %d\n",
			batt_terminal_uv, chip->prev_batt_terminal_uv,
			chip->prev_chg_soc);
		chip->prev_batt_terminal_uv = batt_terminal_uv;
		return chip->prev_chg_soc;
	}

	soc_ibat = bound_soc(linear_interpolate(chip->soc_at_cv,
					chip->ibat_at_cv_ua,
					100, -1 * chip->chg_term_ua,
					params->iavg_ua));
	weight_ibat = bound_soc(linear_interpolate(1, chip->soc_at_cv,
					100, 100, chip->prev_chg_soc));
	weight_cc = 100 - weight_ibat;
	chg_soc = bound_soc(DIV_ROUND_CLOSEST(soc_ibat * weight_ibat
			+ weight_cc * soc, 100));

	pr_debug("weight_ibat = %d, weight_cc = %d, soc_ibat = %d, soc_cc = %d\n",
			weight_ibat, weight_cc, soc_ibat, soc);

	
	if (chg_soc > chip->prev_chg_soc) {
		chip->prev_chg_soc = chg_soc;

		chip->charging_adjusted_ocv = find_ocv_for_pc(chip, batt_temp,
				find_pc_for_soc(chip, params, chg_soc));
		pr_debug("CC CHG ADJ OCV = %d CHG SOC %d\n",
				chip->charging_adjusted_ocv,
				chip->prev_chg_soc);
	}

	pr_debug("Reporting CHG SOC %d\n", chip->prev_chg_soc);
	chip->prev_batt_terminal_uv = batt_terminal_uv;
	return chip->prev_chg_soc;
}

static void very_low_voltage_check(struct qpnp_bms_chip *chip, int vbat_uv)
{
	if (vbat_uv <= chip->low_voltage_threshold
			&& !wake_lock_active(&chip->low_voltage_wake_lock)) {
		pr_debug("voltage = %d low holding wakelock\n", vbat_uv);
		wake_lock(&chip->low_voltage_wake_lock);
	} else if (vbat_uv > chip->low_voltage_threshold
			&& wake_lock_active(&chip->low_voltage_wake_lock)) {
		pr_debug("voltage = %d releasing wakelock\n", vbat_uv);
		wake_unlock(&chip->low_voltage_wake_lock);
	}
}

static void cv_voltage_check(struct qpnp_bms_chip *chip, int vbat_uv)
{
	if (wake_lock_active(&chip->cv_wake_lock)) {
		if (chip->soc_at_cv != -EINVAL) {
			pr_debug("hit CV, releasing cv wakelock\n");
			wake_unlock(&chip->cv_wake_lock);
		} else if (!is_battery_charging(chip)) {
			pr_debug("charging stopped, releasing cv wakelock\n");
			wake_unlock(&chip->cv_wake_lock);
		}
	} else if (vbat_uv > chip->max_voltage_uv - VBATT_ERROR_MARGIN
			&& chip->soc_at_cv == -EINVAL
			&& is_battery_charging(chip)
			&& !wake_lock_active(&chip->cv_wake_lock)) {
		pr_debug("voltage = %d holding cv wakelock\n", vbat_uv);
		wake_lock(&chip->cv_wake_lock);
	}
}

#define NO_ADJUST_HIGH_SOC_THRESHOLD	90
static int adjust_soc(struct qpnp_bms_chip *chip, struct soc_params *params,
							int soc, int batt_temp)
{
	int ibat_ua = 0, vbat_uv = 0;
	int ocv_est_uv = 0, soc_est = 0, pc_est = 0, pc = 0;
	int delta_ocv_uv = 0;
	int n = 0;
	int rc_new_uah = 0;
	int pc_new = 0;
	int soc_new = 0;
	int slope = 0;
	int rc = 0;
	int delta_ocv_uv_limit = 0;
	int correction_limit_uv = 0;

	rc = get_simultaneous_batt_v_and_i(chip, &ibat_ua, &vbat_uv);
	if (rc < 0) {
		pr_err("simultaneous vbat ibat failed err = %d\n", rc);
		goto out;
	}

	very_low_voltage_check(chip, vbat_uv);
	cv_voltage_check(chip, vbat_uv);

	delta_ocv_uv_limit = DIV_ROUND_CLOSEST(ibat_ua, 1000);

	ocv_est_uv = vbat_uv + (ibat_ua * params->rbatt_mohm)/1000;

	pc_est = calculate_pc(chip, ocv_est_uv, batt_temp);
	soc_est = div_s64((s64)params->fcc_uah * pc_est - params->uuc_uah*100,
				(s64)params->fcc_uah - params->uuc_uah);
	soc_est = bound_soc(soc_est);

	
	if (bms_reset) {
		pr_debug("bms reset mode, SOC adjustment skipped\n");
		goto out;
	}

	if (is_battery_charging(chip)) {
		soc = charging_adjustments(chip, params, soc, vbat_uv, ibat_ua,
				batt_temp);
		
		if (chip->soc_at_cv != -EINVAL || ibat_ua < 0)
			goto out;
	}

	if (soc_est == soc
		|| soc_est > chip->adjust_soc_low_threshold
		|| soc >= NO_ADJUST_HIGH_SOC_THRESHOLD)
		goto out;

	if (chip->last_soc_est == -EINVAL)
		chip->last_soc_est = soc;

	n = min(200, max(1 , soc + soc_est + chip->last_soc_est));
	chip->last_soc_est = soc_est;

	pc = calculate_pc(chip, chip->last_ocv_uv, chip->last_ocv_temp);
	if (pc > 0) {
		pc_new = calculate_pc(chip,
				chip->last_ocv_uv - (++slope * 1000),
				chip->last_ocv_temp);
		while (pc_new == pc) {
			
			slope = slope + 10;
			pc_new = calculate_pc(chip,
				chip->last_ocv_uv - (slope * 1000),
				chip->last_ocv_temp);
		}
	} else {
		pc = 1;
		pc_new = 0;
		slope = 1;
	}

	delta_ocv_uv = div_s64((soc - soc_est) * (s64)slope * 1000,
							n * (pc - pc_new));

	if (abs(delta_ocv_uv) > delta_ocv_uv_limit) {
		pr_debug("limiting delta ocv %d limit = %d\n", delta_ocv_uv,
				delta_ocv_uv_limit);

		if (delta_ocv_uv > 0)
			delta_ocv_uv = delta_ocv_uv_limit;
		else
			delta_ocv_uv = -1 * delta_ocv_uv_limit;
		pr_debug("new delta ocv = %d\n", delta_ocv_uv);
	}

#if !(defined(CONFIG_HTC_BATT_8960))
	if (wake_lock_active(&chip->low_voltage_wake_lock))
		goto skip_limits;
#endif

	if (chip->last_ocv_uv > chip->flat_ocv_threshold_uv)
		correction_limit_uv = chip->high_ocv_correction_limit_uv;
	else
		correction_limit_uv = chip->low_ocv_correction_limit_uv;

	if (abs(delta_ocv_uv) > correction_limit_uv) {
		pr_debug("limiting delta ocv %d limit = %d\n",
			delta_ocv_uv, correction_limit_uv);
		if (delta_ocv_uv > 0)
			delta_ocv_uv = correction_limit_uv;
		else
			delta_ocv_uv = -correction_limit_uv;
		pr_debug("new delta ocv = %d\n", delta_ocv_uv);
	}

skip_limits:

	chip->last_ocv_uv -= delta_ocv_uv;

	if (chip->last_ocv_uv >= chip->max_voltage_uv)
		chip->last_ocv_uv = chip->max_voltage_uv;

	
	pc_new = calculate_pc(chip, chip->last_ocv_uv, chip->last_ocv_temp);
	rc_new_uah = (params->fcc_uah * pc_new) / 100;
	soc_new = (rc_new_uah - params->cc_uah - params->uuc_uah)*100
					/ (params->fcc_uah - params->uuc_uah);
	soc_new = bound_soc(soc_new);

	if (soc_new == 0 && soc_est >= chip->hold_soc_est)
		soc_new = 1;

	soc = soc_new;

out:
	pr_info("ibat_ua=%d,vbat_uv=%d,ocv_est_uv=%d,pc_est=%d,"
			"soc_est=%d,n=%d,delta_ocv_uv=%d,last_ocv_uv=%d,"
			"pc_new=%d,soc_new=%d,rbatt=%d,slope=%d\n",
		ibat_ua, vbat_uv, ocv_est_uv, pc_est,
		soc_est, n, delta_ocv_uv, chip->last_ocv_uv,
		pc_new, soc_new, params->rbatt_mohm, slope);

	return soc;
}
#endif 
static int clamp_soc_based_on_voltage(struct qpnp_bms_chip *chip, int soc)
{
	int rc, vbat_uv, batt_temp;

	rc = get_battery_voltage(chip, &vbat_uv);
	if (rc < 0) {
		pr_err("adc vbat failed err = %d\n", rc);
		return soc;
	}

	rc = pm8941_get_batt_temperature(&batt_temp);
	if (rc) {
		pr_err("get temperature failed err = %d\n", rc);
		return soc;
	}

	if (soc == 0)
		pr_info("batt_vol = %d, batt_temp = %d\n", vbat_uv, batt_temp);

	if (chip->shutdown_vol_criteria && soc == 0 &&
			vbat_uv > chip->shutdown_vol_criteria && batt_temp > 0) {
		pr_debug("clamping soc to 1, temp = %d, vbat (%d) > cutoff (%d)\n",
						vbat_uv, batt_temp, chip->shutdown_vol_criteria);
		return 1;
	} else {
		pr_debug("not clamping, using soc = %d, vbat = %d and cutoff = %d\n",
				soc, vbat_uv, chip->shutdown_vol_criteria);
		return soc;
	}
}

static int64_t convert_cc_uah_to_raw(struct qpnp_bms_chip *chip, int64_t cc_uah)
{
	int64_t cc_uv, cc_pvh, cc_raw;

	cc_pvh = cc_uah * chip->r_sense_uohm;
	cc_uv = div_s64(cc_pvh * SLEEP_CLK_HZ * SECONDS_PER_HOUR,
				CC_READING_TICKS * 1000000LL);
	cc_raw = div_s64(cc_uv * CC_READING_RESOLUTION_D,
			CC_READING_RESOLUTION_N);
	return cc_raw;
}

#define CC_STEP_INCREMENT_UAH	1500
#define OCV_STEP_INCREMENT	0x10
static void configure_soc_wakeup(struct qpnp_bms_chip *chip,
				struct soc_params *params,
				int batt_temp, int target_soc)
{
	int target_ocv_uv;
	int64_t target_cc_uah, cc_raw_64, current_shdw_cc_raw_64;
	int64_t current_shdw_cc_uah, iadc_comp_factor;
	uint64_t cc_raw, current_shdw_cc_raw;
	int16_t ocv_raw, current_ocv_raw;

	current_shdw_cc_raw = 0;
	mutex_lock(&chip->bms_output_lock);
	lock_output_data(chip);
	qpnp_read_wrapper(chip, (u8 *)&current_ocv_raw,
			chip->base + BMS1_OCV_FOR_SOC_DATA0, 2);
	unlock_output_data(chip);
	mutex_unlock(&chip->bms_output_lock);
	current_shdw_cc_uah = get_prop_bms_charge_counter_shadow(chip);
	current_shdw_cc_raw_64 = convert_cc_uah_to_raw(chip,
			current_shdw_cc_uah);

	target_cc_uah = (100 - target_soc)
		* (params->fcc_uah - params->uuc_uah)
		/ 100 - current_shdw_cc_uah;
	if (target_cc_uah < 0) {
		target_cc_uah = CC_STEP_INCREMENT_UAH;
	}
	iadc_comp_factor = 100000;
	qpnp_iadc_comp_result(chip->iadc_dev, &iadc_comp_factor);
	target_cc_uah = div64_s64(target_cc_uah * 100000, iadc_comp_factor);
	target_cc_uah = cc_reverse_adjust_for_gain(chip, target_cc_uah);
	cc_raw_64 = convert_cc_uah_to_raw(chip, target_cc_uah);
	cc_raw = convert_s64_to_s36(cc_raw_64);

	target_ocv_uv = find_ocv_for_pc(chip, batt_temp,
				find_pc_for_soc(chip, params, target_soc));
	ocv_raw = convert_vbatt_uv_to_raw(chip, target_ocv_uv);

	if (current_ocv_raw != chip->ocv_reading_at_100
			&& current_ocv_raw < ocv_raw)
		ocv_raw = current_ocv_raw - OCV_STEP_INCREMENT;

	qpnp_write_wrapper(chip, (u8 *)&cc_raw,
			chip->base + BMS1_SW_CC_THR0, 5);
	qpnp_write_wrapper(chip, (u8 *)&ocv_raw,
			chip->base + BMS1_OCV_THR0, 2);

	pr_debug("current sw_cc_raw = 0x%llx, current ocv = 0x%hx\n",
			current_shdw_cc_raw, (uint16_t)current_ocv_raw);
	pr_debug("target_cc_uah = %lld, raw64 = 0x%llx, raw 36 = 0x%llx, ocv_raw = 0x%hx\n",
			target_cc_uah,
			(uint64_t)cc_raw_64, cc_raw,
			(uint16_t)ocv_raw);
}

static int calculate_raw_soc(struct qpnp_bms_chip *chip,
					struct raw_soc_params *raw,
					struct soc_params *params,
					int batt_temp)
{
	int soc, remaining_usable_charge_uah;

	
	remaining_usable_charge_uah = params->ocv_charge_uah
					- params->cc_uah
					- params->uuc_uah;
	pr_debug("RUC = %duAh\n", remaining_usable_charge_uah);

	soc = DIV_ROUND_CLOSEST((remaining_usable_charge_uah * 100),
				(params->fcc_uah - params->uuc_uah));

	bms_dbg.fcc_uah = params->fcc_uah;
	bms_dbg.uuc_uah = params->uuc_uah;
	bms_dbg.rc_uah = params->ocv_charge_uah;
	bms_dbg.ruc_uah = remaining_usable_charge_uah;
	bms_dbg.cc_uah = params->cc_uah;
	bms_dbg.raw_soc = soc;

	if (chip->first_time_calc_soc && soc < 0) {
		pr_info("soc is %d, adjusting pon ocv to make it 0\n", soc);
		chip->last_ocv_uv = find_ocv_for_pc(chip, batt_temp,
				find_pc_for_soc(chip, params, 0));
		params->ocv_charge_uah = find_ocv_charge_for_soc(chip,
				params, 0);

		remaining_usable_charge_uah = params->ocv_charge_uah
					- params->cc_uah
					- params->uuc_uah;

		soc = DIV_ROUND_CLOSEST((remaining_usable_charge_uah * 100),
					(params->fcc_uah
						- params->uuc_uah));
		pr_info("DONE for O soc is %d, pon ocv adjusted to %duV, "
				"ocv_charge_uah=%d\n",
				soc, chip->last_ocv_uv, params->ocv_charge_uah);
	}

	if (soc > 100)
		soc = 100;

	if (soc < 0) {
		pr_debug("bad rem_usb_chg = %d rem_chg %d, cc_uah %d, unusb_chg %d\n",
				remaining_usable_charge_uah,
				params->ocv_charge_uah,
				params->cc_uah, params->uuc_uah);

		pr_debug("for bad rem_usb_chg last_ocv_uv = %d batt_temp = %d fcc = %d soc =%d\n",
				chip->last_ocv_uv, batt_temp,
				params->fcc_uah, soc);
		soc = 0;
	}

	return soc;
}

#define SLEEP_RECALC_INTERVAL	3
static int calculate_state_of_charge(struct qpnp_bms_chip *chip,
					struct raw_soc_params *raw,
					int batt_temp)
{
	struct soc_params params;
	int soc, previous_soc, shutdown_soc, new_calculated_soc;
	int remaining_usable_charge_uah;

	calculate_soc_params(chip, raw, &params, batt_temp);
	if (!is_battery_present(chip)) {
		pr_debug("battery gone, reporting 100\n");
		new_calculated_soc = 100;
		goto done_calculating;
	}

	if (params.fcc_uah - params.uuc_uah <= 0) {
		pr_debug("FCC = %duAh, UUC = %duAh forcing soc = 0\n",
						params.fcc_uah,
						params.uuc_uah);
		new_calculated_soc = 0;
		goto done_calculating;
	}

	soc = calculate_raw_soc(chip, raw, &params, batt_temp);

	mutex_lock(&chip->soc_invalidation_mutex);
	shutdown_soc = chip->shutdown_soc;

	if (chip->first_time_calc_soc && soc != shutdown_soc
			&& !chip->shutdown_soc_invalid) {
		pr_info("soc = %d before forcing shutdown_soc = %d\n",
							soc, shutdown_soc);
		chip->last_ocv_uv = find_ocv_for_pc(chip, batt_temp,
				find_pc_for_soc(chip, &params, shutdown_soc));
		params.ocv_charge_uah = find_ocv_charge_for_soc(chip,
				&params, shutdown_soc);

		remaining_usable_charge_uah = params.ocv_charge_uah
					- params.cc_uah
					- params.uuc_uah;

		soc = DIV_ROUND_CLOSEST((remaining_usable_charge_uah * 100),
					(params.fcc_uah
						- params.uuc_uah));

		pr_info("DONE for shutdown_soc = %d soc is %d, adjusted ocv to %duV\n",
				shutdown_soc, soc, chip->last_ocv_uv);
	}
	mutex_unlock(&chip->soc_invalidation_mutex);

	pr_debug("SOC before adjustment = %d\n", soc);
#if !(defined(CONFIG_HTC_BATT_8960))
	new_calculated_soc = adjust_soc(chip, &params, soc, batt_temp);
#else
	new_calculated_soc = soc;
#endif
	bms_dbg.adjusted_soc = new_calculated_soc;

	
	new_calculated_soc = clamp_soc_based_on_voltage(chip,
					new_calculated_soc);

	if (is_battery_full(chip))
		configure_soc_wakeup(chip, &params,
				batt_temp, bound_soc(new_calculated_soc - 1));
done_calculating:
	mutex_lock(&chip->last_soc_mutex);
	previous_soc = chip->calculated_soc;
	chip->calculated_soc = new_calculated_soc;

	pr_debug("CC based calculated SOC = %d\n", chip->calculated_soc);
	if (chip->last_soc_invalid) {
		chip->last_soc_invalid = false;
		chip->last_soc = -EINVAL;
	}
	if (params.delta_time_s * 1000 >
			chip->calculate_soc_ms * SLEEP_RECALC_INTERVAL
			&& !chip->first_time_calc_soc) {
		chip->last_soc_unbound = true;
		chip->last_soc_change_sec = chip->last_recalc_time;
		pr_debug("last_soc unbound because elapsed time = %d\n",
				params.delta_time_s);
	}
	mutex_unlock(&chip->last_soc_mutex);
	wake_up_interruptible(&chip->bms_wait_queue);

	if (new_calculated_soc != previous_soc && chip->bms_psy_registered) {
		power_supply_changed(&chip->bms_psy);
		pr_debug("power supply changed\n");
	} else {
		report_state_of_charge(chip);
	}
	pr_info("FCC=%d,UC=%d,RC=%d,CC_uAh/ori=%d/%d,RUC=%d,SOC=%d,raw_soc=%d,"
		       "start_pc=%d,end_pc=%d,OCV_uV/ori=%d/%d,OCV_raw=%x,"
		       "rbatt=%d,rbatt_sf=%d,batt_temp=%d,soc_rbatt=%d,"
		       "ori_uuc_uah=%d,uuc_rbatt=%d,uuc_iavg_ma=%d,"
		       "unusable_uv=%d,pc_unusable=%d,rc_pc=%d,sh_soc=%d,"
		       "t_last_change_s=%d,adj_soc=%d,cal_soc=%d,cc_raw=%lld,"
		       "shdw_cc_raw=%lld,ocv_at_100=%x,"
		       "cc_backup=%d,ocv_backup=%d,consistent_flag=%d,is_ocv_update_start=%d,"
		       "no_hw_ocv_ms=%ld\n",
			bms_dbg.fcc_uah, bms_dbg.uuc_uah, bms_dbg.rc_uah, bms_dbg.cc_uah,bms_dbg.ori_cc_uah,
			bms_dbg.ruc_uah, soc, bms_dbg.raw_soc,
			chip->start_soc, chip->end_soc,
			raw->last_good_ocv_uv, bms_dbg.last_ocv_raw_uv, raw->last_good_ocv_raw,
			bms_dbg.rbatt, bms_dbg.rbatt_sf, batt_temp,
			bms_dbg.soc_rbatt, bms_dbg.ori_uuc_uah,
			bms_dbg.uuc_rbatt_mohm, bms_dbg.uuc_iavg_ma,
			bms_dbg.unusable_uv, bms_dbg.pc_unusable, bms_dbg.rc_pc,
			bms_dbg.shutdown_soc, bms_dbg.time_last_change_s, bms_dbg.adjusted_soc,
			chip->calculated_soc, raw->cc, raw->shdw_cc, chip->ocv_reading_at_100,
			chip->cc_backup_uah, chip->ocv_backup_uv, consistent_flag, is_ocv_update_start,
			htc_batt_bms_timer.no_ocv_update_period_ms);

	if(Last_OCV_Raw == 0){
		Last_OCV_Raw = raw->last_good_ocv_raw;
	}
	else if(raw->last_good_ocv_raw != Last_OCV_Raw)
	{
		pr_info("HW OCV update: %d -> %d.\n",Last_OCV_Raw, raw->last_good_ocv_raw);
		Last_OCV_Raw = raw->last_good_ocv_raw;
		if(soc < 100){
			store_start_soc = 0;
			store_start_cc = 0;
			store_start_pc = 0;
			store_start_real_soc = 0;
			pr_info("Reset the Store CC.\n");
		}
	}
	get_current_time(&chip->last_recalc_time);
	chip->first_time_calc_soc = 0;
	chip->first_time_calc_uuc = 0;
	return chip->calculated_soc;
}

static int calculate_soc_from_voltage(struct qpnp_bms_chip *chip)
{
	int voltage_range_uv, voltage_remaining_uv, voltage_based_soc;
	int rc, vbat_uv;

	rc = get_battery_voltage(chip, &vbat_uv);
	if (rc < 0) {
		pr_err("adc vbat failed err = %d\n", rc);
		return rc;
	}
	voltage_range_uv = chip->max_voltage_uv - chip->v_cutoff_uv;
	voltage_remaining_uv = vbat_uv - chip->v_cutoff_uv;
	voltage_based_soc = voltage_remaining_uv * 100 / voltage_range_uv;

	voltage_based_soc = clamp(voltage_based_soc, 0, 100);

	if (chip->prev_voltage_based_soc != voltage_based_soc
				&& chip->bms_psy_registered) {
		power_supply_changed(&chip->bms_psy);
		pr_debug("power supply changed\n");
	}
	chip->prev_voltage_based_soc = voltage_based_soc;

	pr_debug("vbat used = %duv\n", vbat_uv);
	pr_debug("Calculated voltage based soc = %d\n", voltage_based_soc);
	return voltage_based_soc;
}

static int recalculate_raw_soc(struct qpnp_bms_chip *chip)
{
	int batt_temp, rc, soc;
	struct qpnp_vadc_result result;
	struct raw_soc_params raw;
	struct soc_params params;

	bms_stay_awake(&chip->soc_wake_source);
	if (chip->use_voltage_soc) {
		soc = calculate_soc_from_voltage(chip);
	} else {
		if (!chip->batfet_closed)
			qpnp_iadc_calibrate_for_trim(chip->iadc_dev, true);
		rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM,
								&result);
		if (rc) {
			pr_err("error reading vadc LR_MUX1_BATT_THERM = %d, rc = %d\n",
						LR_MUX1_BATT_THERM, rc);
			soc = chip->calculated_soc;
		} else {
			pr_debug("batt_temp phy = %lld meas = 0x%llx\n",
							result.physical,
							result.measurement);
			batt_temp = (int)result.physical;

			mutex_lock(&chip->last_ocv_uv_mutex);
			read_soc_params_raw(chip, &raw, batt_temp);
			calculate_soc_params(chip, &raw, &params, batt_temp);
			if (!is_battery_present(chip)) {
				pr_debug("battery gone\n");
				soc = 0;
			} else if (params.fcc_uah - params.uuc_uah <= 0) {
				pr_debug("FCC = %duAh, UUC = %duAh forcing soc = 0\n",
							params.fcc_uah,
							params.uuc_uah);
				soc = 0;
			} else {
				soc = calculate_raw_soc(chip, &raw,
							&params, batt_temp);
			}
			mutex_unlock(&chip->last_ocv_uv_mutex);
		}
	}
	bms_relax(&chip->soc_wake_source);
	return soc;
}

static int recalculate_soc(struct qpnp_bms_chip *chip)
{
	int batt_temp, rc, soc;
	struct qpnp_vadc_result result;
	struct raw_soc_params raw;

	bms_stay_awake(&chip->soc_wake_source);
	mutex_lock(&chip->vbat_monitor_mutex);
	if (chip->vbat_monitor_params.state_request !=
			ADC_TM_HIGH_LOW_THR_DISABLE)
		qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
					&chip->vbat_monitor_params);
	mutex_unlock(&chip->vbat_monitor_mutex);
	if (chip->use_voltage_soc) {
		soc = calculate_soc_from_voltage(chip);
	} else {
		if (!chip->batfet_closed)
			qpnp_iadc_calibrate_for_trim(chip->iadc_dev, true);
		rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM,
								&result);
		if (rc) {
			pr_err("error reading vadc LR_MUX1_BATT_THERM = %d, rc = %d\n",
						LR_MUX1_BATT_THERM, rc);
			soc = chip->calculated_soc;
		} else {
			pr_debug("batt_temp phy = %lld meas = 0x%llx\n",
							result.physical,
							result.measurement);
			batt_temp = (int)result.physical;

			mutex_lock(&chip->last_ocv_uv_mutex);
			read_soc_params_raw(chip, &raw, batt_temp);
			soc = calculate_state_of_charge(chip, &raw, batt_temp);
			mutex_unlock(&chip->last_ocv_uv_mutex);
		}
	}
	bms_relax(&chip->soc_wake_source);
	return soc;
}

static void recalculate_work(struct work_struct *work)
{
	struct qpnp_bms_chip *chip = container_of(work,
				struct qpnp_bms_chip,
				recalc_work);

	recalculate_soc(chip);
}

static int get_calculation_delay_ms(struct qpnp_bms_chip *chip)
{
#if !(defined(CONFIG_HTC_BATT_8960))
	if (wake_lock_active(&chip->low_voltage_wake_lock))
		return chip->low_voltage_calculate_soc_ms;
	else if (chip->calculated_soc < chip->low_soc_calc_threshold)
		return chip->low_soc_calculate_soc_ms;
	else
#endif
		return chip->calculate_soc_ms;
}

static void calculate_soc_work(struct work_struct *work)
{
	struct qpnp_bms_chip *chip = container_of(work,
				struct qpnp_bms_chip,
				calculate_soc_delayed_work.work);

	recalculate_soc(chip);
	schedule_delayed_work(&chip->calculate_soc_delayed_work,
		round_jiffies_relative(msecs_to_jiffies
		(get_calculation_delay_ms(chip))));
}

#define VBATT_ERROR_MARGIN	20000
#if !(defined(CONFIG_HTC_BATT_8960))
static void configure_vbat_monitor_low(struct qpnp_bms_chip *chip)
{
	mutex_lock(&chip->vbat_monitor_mutex);
	if (chip->vbat_monitor_params.state_request
			== ADC_TM_HIGH_LOW_THR_ENABLE) {
		pr_debug("battery entered cutoff range\n");
#if !(defined(CONFIG_HTC_BATT_8960))
		if (!wake_lock_active(&chip->low_voltage_wake_lock)) {
			pr_debug("voltage low, holding wakelock\n");
			wake_lock(&chip->low_voltage_wake_lock);
			cancel_delayed_work_sync(
					&chip->calculate_soc_delayed_work);
			schedule_delayed_work(
					&chip->calculate_soc_delayed_work, 0);
		}
#endif
		chip->vbat_monitor_params.state_request =
					ADC_TM_HIGH_THR_ENABLE;
		chip->vbat_monitor_params.high_thr =
			(chip->low_voltage_threshold + VBATT_ERROR_MARGIN);
		pr_debug("set low thr to %d and high to %d\n",
				chip->vbat_monitor_params.low_thr,
				chip->vbat_monitor_params.high_thr);
		chip->vbat_monitor_params.low_thr = 0;
	} else if (chip->vbat_monitor_params.state_request
			== ADC_TM_LOW_THR_ENABLE) {
#if !(defined(CONFIG_HTC_BATT_8960))
		pr_debug("battery entered normal range\n");
		if (wake_lock_active(&chip->cv_wake_lock)) {
			wake_unlock(&chip->cv_wake_lock);
			pr_debug("releasing cv wake lock\n");
		}
#endif
		chip->in_cv_range = false;
		chip->vbat_monitor_params.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		chip->vbat_monitor_params.high_thr = chip->max_voltage_uv
				- VBATT_ERROR_MARGIN;
		chip->vbat_monitor_params.low_thr =
				chip->low_voltage_threshold;
		pr_debug("set low thr to %d and high to %d\n",
				chip->vbat_monitor_params.low_thr,
				chip->vbat_monitor_params.high_thr);
	}
	qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
					&chip->vbat_monitor_params);
	mutex_unlock(&chip->vbat_monitor_mutex);
}

#define CV_LOW_THRESHOLD_HYST_UV 100000
static void configure_vbat_monitor_high(struct qpnp_bms_chip *chip)
{
	mutex_lock(&chip->vbat_monitor_mutex);
	if (chip->vbat_monitor_params.state_request
			== ADC_TM_HIGH_LOW_THR_ENABLE) {
		pr_debug("battery entered vddmax range\n");
		chip->in_cv_range = true;
#if !(defined(CONFIG_HTC_BATT_8960))
		if (!wake_lock_active(&chip->cv_wake_lock)) {
			wake_lock(&chip->cv_wake_lock);
			pr_debug("holding cv wake lock\n");
		}
#endif
		schedule_work(&chip->recalc_work);
		chip->vbat_monitor_params.state_request =
					ADC_TM_LOW_THR_ENABLE;
		chip->vbat_monitor_params.low_thr =
			(chip->max_voltage_uv - CV_LOW_THRESHOLD_HYST_UV);
		chip->vbat_monitor_params.high_thr = chip->max_voltage_uv * 2;
		pr_debug("set low thr to %d and high to %d\n",
				chip->vbat_monitor_params.low_thr,
				chip->vbat_monitor_params.high_thr);
	} else if (chip->vbat_monitor_params.state_request
			== ADC_TM_HIGH_THR_ENABLE) {
#if !(defined(CONFIG_HTC_BATT_8960))
		pr_debug("battery entered normal range\n");
		if (wake_lock_active(&chip->low_voltage_wake_lock)) {
			pr_debug("voltage high, releasing wakelock\n");
			wake_unlock(&chip->low_voltage_wake_lock);
		}
#endif
		chip->vbat_monitor_params.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		chip->vbat_monitor_params.high_thr =
			chip->max_voltage_uv - VBATT_ERROR_MARGIN;
		chip->vbat_monitor_params.low_thr =
				chip->low_voltage_threshold;
		pr_debug("set low thr to %d and high to %d\n",
				chip->vbat_monitor_params.low_thr,
				chip->vbat_monitor_params.high_thr);
	}
	qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
					&chip->vbat_monitor_params);
	mutex_unlock(&chip->vbat_monitor_mutex);
}

static void btm_notify_vbat(enum qpnp_tm_state state, void *ctx)
{
	struct qpnp_bms_chip *chip = ctx;
	int vbat_uv;
	struct qpnp_vadc_result result;
	int rc;

	rc = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &result);
	pr_debug("vbat = %lld, raw = 0x%x\n", result.physical, result.adc_code);

	get_battery_voltage(chip, &vbat_uv);
	pr_debug("vbat is at %d, state is at %d\n", vbat_uv, state);

	if (state == ADC_TM_LOW_STATE) {
		pr_debug("low voltage btm notification triggered\n");
		if (vbat_uv - VBATT_ERROR_MARGIN
				< chip->vbat_monitor_params.low_thr) {
			configure_vbat_monitor_low(chip);
		} else {
			pr_debug("faulty btm trigger, discarding\n");
			qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
					&chip->vbat_monitor_params);
		}
	} else if (state == ADC_TM_HIGH_STATE) {
		pr_debug("high voltage btm notification triggered\n");
		if (vbat_uv + VBATT_ERROR_MARGIN
				> chip->vbat_monitor_params.high_thr) {
			configure_vbat_monitor_high(chip);
		} else {
			pr_debug("faulty btm trigger, discarding\n");
			qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
					&chip->vbat_monitor_params);
		}
	} else {
		pr_debug("unknown voltage notification state: %d\n", state);
	}
	if (chip->bms_psy_registered)
		power_supply_changed(&chip->bms_psy);
}
#endif

static int reset_vbat_monitoring(struct qpnp_bms_chip *chip)
{
	int rc;

	chip->vbat_monitor_params.state_request = ADC_TM_HIGH_LOW_THR_DISABLE;

	rc = qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
						&chip->vbat_monitor_params);
	if (rc) {
		pr_err("tm disable failed: %d\n", rc);
		return rc;
	}
#if !(defined(CONFIG_HTC_BATT_8960))
	if (wake_lock_active(&chip->low_voltage_wake_lock)) {
		pr_debug("battery removed, releasing wakelock\n");
		wake_unlock(&chip->low_voltage_wake_lock);
	}
#endif
	if (chip->in_cv_range) {
		pr_debug("battery removed, removing in_cv_range state\n");
		chip->in_cv_range = false;
	}
	return 0;
}

#if !(defined(CONFIG_HTC_BATT_8960))
static int setup_vbat_monitoring(struct qpnp_bms_chip *chip)
{
	int rc;

	chip->vbat_monitor_params.low_thr = chip->low_voltage_threshold;
	chip->vbat_monitor_params.high_thr = chip->max_voltage_uv
							- VBATT_ERROR_MARGIN;
	chip->vbat_monitor_params.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
	chip->vbat_monitor_params.channel = VBAT_SNS;
	chip->vbat_monitor_params.btm_ctx = (void *)chip;
	chip->vbat_monitor_params.timer_interval = ADC_MEAS1_INTERVAL_1S;
	chip->vbat_monitor_params.threshold_notification = &btm_notify_vbat;
	pr_debug("set low thr to %d and high to %d\n",
			chip->vbat_monitor_params.low_thr,
			chip->vbat_monitor_params.high_thr);

	if (!is_battery_present(chip)) {
		pr_debug("no battery inserted, do not enable vbat monitoring\n");
		chip->vbat_monitor_params.state_request =
			ADC_TM_HIGH_LOW_THR_DISABLE;
	} else {
		rc = qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
						&chip->vbat_monitor_params);
		if (rc) {
			pr_err("tm setup failed: %d\n", rc);
		return rc;
		}
	}

	pr_debug("setup complete\n");
	return 0;
}
#endif

static void readjust_fcc_table(struct qpnp_bms_chip *chip)
{
	struct single_row_lut *temp, *old;
	int i, fcc, ratio;

	if (!chip->enable_fcc_learning)
		return;

	if (!chip->fcc_temp_lut) {
		pr_err("The static fcc lut table is NULL\n");
		return;
	}

	temp = devm_kzalloc(chip->dev, sizeof(struct single_row_lut),
			GFP_KERNEL);
	if (!temp) {
		pr_err("Cannot allocate memory for adjusted fcc table\n");
		return;
	}

	fcc = interpolate_fcc(chip->fcc_temp_lut, chip->fcc_new_batt_temp);

	temp->cols = chip->fcc_temp_lut->cols;
	for (i = 0; i < chip->fcc_temp_lut->cols; i++) {
		pr_info("old temp_lut: [%d], x=%d, y=%d\n",i, chip->fcc_temp_lut->x[i], chip->fcc_temp_lut->y[i]);
		temp->x[i] = chip->fcc_temp_lut->x[i];
		ratio = div_u64(chip->fcc_temp_lut->y[i] * 1000, fcc);
		temp->y[i] =  (ratio * chip->fcc_new_mah);
		temp->y[i] /= 1000;
		pr_info("new temp_lut: [%d], x=%d, y=%d\n",i, temp->x[i], temp->y[i]);
	}

	old = chip->adjusted_fcc_temp_lut;
	chip->adjusted_fcc_temp_lut = temp;
	devm_kfree(chip->dev, old);
}

static int read_fcc_data_from_backup(struct qpnp_bms_chip *chip)
{
	int rc, i;
	u8 fcc = 0, chgcyl = 0;

	for (i = 0; i < chip->min_fcc_learning_samples; i++) {
		rc = qpnp_read_wrapper(chip, &fcc,
			chip->base + BMS_FCC_BASE_REG + i, 1);
		rc |= qpnp_read_wrapper(chip, &chgcyl,
			chip->base + BMS_CHGCYL_BASE_REG + i, 1);
		if (rc) {
			pr_err("Unable to read FCC data\n");
			return rc;
		}
		if (fcc == 0 || (fcc == 0xFF && chgcyl == 0xFF)) {
			pr_info("No fcc data sample!!!\n");
			
			chip->fcc_learning_samples[i].fcc_new = 0;
			chip->fcc_learning_samples[i].chargecycles = 0;
		} else {
			
			chip->fcc_sample_count++;
			chip->fcc_learning_samples[i].fcc_new =
						fcc * chip->fcc_resolution;
			chip->fcc_learning_samples[i].chargecycles =
						chgcyl * CHGCYL_RESOLUTION;
			pr_info("backup data: sample[%d], fcc_new=%d, chargecycles=%d\n",i, chip->fcc_learning_samples[i].fcc_new, chip->fcc_learning_samples[i].chargecycles);
		}
	}

	return 0;
}

static int discard_backup_fcc_data(struct qpnp_bms_chip *chip)
{
	int rc = 0, i;
	u8 temp_u8 = 0;

	chip->fcc_sample_count = 0;
	for (i = 0; i < chip->min_fcc_learning_samples; i++) {
		rc = qpnp_write_wrapper(chip, &temp_u8,
			chip->base + BMS_FCC_BASE_REG + i, 1);
		rc |= qpnp_write_wrapper(chip, &temp_u8,
			chip->base + BMS_CHGCYL_BASE_REG + i, 1);
		if (rc) {
			pr_err("Unable to clear FCC data\n");
			return rc;
		}
	}

	return 0;
}

static void
average_fcc_samples_and_readjust_fcc_table(struct qpnp_bms_chip *chip)
{
	int i, temp_fcc_avg = 0, temp_fcc_delta = 0, new_fcc_avg = 0, chgcyle_check_fcc = 0, index;
	struct fcc_sample *ft;

	for (i = 0; i < chip->min_fcc_learning_samples; i++){
		temp_fcc_avg += chip->fcc_learning_samples[i].fcc_new;
		pr_info("sample[%d], fcc_new=%d, chargecycles=%d\n",i, chip->fcc_learning_samples[i].fcc_new, chip->fcc_learning_samples[i].chargecycles);
	}

	temp_fcc_avg /= chip->min_fcc_learning_samples;
	temp_fcc_delta = div_u64(temp_fcc_avg * DELTA_FCC_PERCENT, 100);

	pr_info("measure sample: fcc_avg=%d, fcc_delta=%d\n", temp_fcc_avg, temp_fcc_delta);

	
	for (i = 0; i < chip->min_fcc_learning_samples; i++) {
		ft = &chip->fcc_learning_samples[i];
		if (abs(ft->fcc_new - temp_fcc_avg) > temp_fcc_delta){
			new_fcc_avg += temp_fcc_avg;
			pr_info("FCC measure (%d): total fcc_mah=%d, use avg fcc=%d\n",
					i, new_fcc_avg, temp_fcc_avg);
		}else{
			new_fcc_avg += ft->fcc_new;
			pr_info("FCC measure (%d): total fcc_mah=%d, use sample fcc=%d\n",
					i, new_fcc_avg, ft->fcc_new);
		}
	}
	new_fcc_avg /= chip->min_fcc_learning_samples;
	
	if(chgcyl_checking_setting){
		for(index = 0; index < 6; index++)
		{
			if(chip->charge_cycles >= chgcyl_checking_table[index][0]){
				chgcyle_check_fcc = chip->fcc_mah * chgcyl_checking_table[index][1] / 100;
				break;
			}
		}

		if((chgcyle_check_fcc > new_fcc_avg)&&(index < 6)){
			Count_FCC_Checking_cycle++;
			pr_info("Ignore the FCC measure on the charge cycle checking(%d): new_fcc:%d < limit_fcc:%d \n", chip->charge_cycles,new_fcc_avg,chgcyle_check_fcc);
			pr_info("Count = %d\n", Count_FCC_Checking_cycle);
			if(Count_FCC_Checking_cycle > 3)
			{
				Count_FCC_Checking_cycle = 0;
				chip->charge_cycles += 100;
				pr_info("Discard the backup and change the charging cycle to %d.\n", chip->charge_cycles);
				discard_backup_fcc_data(chip);
			}
			return;
		}
	}

	chip->fcc_new_mah = new_fcc_avg;
	chip->fcc_new_batt_temp = FCC_DEFAULT_TEMP;
	pr_info("FCC update: New fcc_mah=%d, fcc_batt_temp=%d, samples=%d\n",
				new_fcc_avg, FCC_DEFAULT_TEMP, chip->min_fcc_learning_samples);
	readjust_fcc_table(chip);
}

static void backup_charge_cycle(struct qpnp_bms_chip *chip)
{
	int rc = 0;

	if (chip->charge_increase >= 0) {
		rc = qpnp_write_wrapper(chip, &chip->charge_increase,
			chip->base + CHARGE_INCREASE_STORAGE, 1);
		if (rc)
			pr_err("Unable to backup charge_increase\n");
	}

	if (chip->charge_cycles >= 0) {
		rc = qpnp_write_wrapper(chip, (u8 *)&chip->charge_cycles,
				chip->base + CHARGE_CYCLE_STORAGE_LSB, 2);
		if (rc)
			pr_err("Unable to backup charge_cycles\n");
	}
}

static bool chargecycles_in_range(struct qpnp_bms_chip *chip)
{
	int i, min_cycle, max_cycle, valid_range;

	
	max_cycle = min_cycle = chip->fcc_learning_samples[0].chargecycles;
	for (i = 1; i < chip->min_fcc_learning_samples; i++) {
		if (min_cycle > chip->fcc_learning_samples[i].chargecycles)
			min_cycle = chip->fcc_learning_samples[i].chargecycles;
		if (max_cycle < chip->fcc_learning_samples[i].chargecycles)
			max_cycle = chip->fcc_learning_samples[i].chargecycles;
		pr_info("learning cycle. [%d]:%d\n", i ,chip->fcc_learning_samples[i].chargecycles);
	}

	
	valid_range = DIV_ROUND_UP(VALID_FCC_CHGCYL_RANGE,
					CHGCYL_RESOLUTION) * CHGCYL_RESOLUTION;
	if (abs(max_cycle - min_cycle) > valid_range){
		pr_info("Invalid range: %d[%d-%d]\n",valid_range, max_cycle, min_cycle);
		return false;
	}
	pr_info("Valid range: %d\n",valid_range);
	return true;
}

static int read_chgcycle_data_from_backup(struct qpnp_bms_chip *chip)
{
	int rc;
	uint16_t temp_u16 = 0;
	u8 temp_u8 = 0;

	rc = qpnp_read_wrapper(chip, &temp_u8,
				chip->base + CHARGE_INCREASE_STORAGE, 1);
	if (!rc && temp_u8 != 0xFF)
		chip->charge_increase = temp_u8;

	rc = qpnp_read_wrapper(chip, (u8 *)&temp_u16,
				chip->base + CHARGE_CYCLE_STORAGE_LSB, 2);
	if (!rc && temp_u16 != 0xFFFF)
		chip->charge_cycles = temp_u16;

	return rc;
}

static void
attempt_learning_new_fcc(struct qpnp_bms_chip *chip)
{
	pr_info("Total FCC sample count=%d\n", chip->fcc_sample_count);

	
	pr_info("Only for test, ignore to update new FCC.\n");
	if(0){
		if ((chip->fcc_sample_count == chip->min_fcc_learning_samples) &&
						chargecycles_in_range(chip))
		average_fcc_samples_and_readjust_fcc_table(chip);
	}
}

static int calculate_real_soc(struct qpnp_bms_chip *chip,
		int batt_temp, struct raw_soc_params *raw, int cc_uah)
{
	int fcc_uah, rc_uah;

	fcc_uah = calculate_fcc(chip, batt_temp);
	rc_uah = calculate_ocv_charge(chip, raw, fcc_uah);

	return ((rc_uah - cc_uah) * 100) / fcc_uah;
}

#define MAX_U8_VALUE		((u8)(~0U))

static int backup_new_fcc(struct qpnp_bms_chip *chip, int fcc_mah,
							int chargecycles)
{
	int rc, min_cycle, i;
	u8 fcc_new, chgcyl, pos = 0;
	struct fcc_sample *ft;

	if ((fcc_mah > (chip->fcc_resolution * MAX_U8_VALUE)) ||
		(chargecycles > (CHGCYL_RESOLUTION * MAX_U8_VALUE))) {
	pr_warn("FCC/Chgcyl beyond storage limit. fcc_mah =%d(%d * %d), chgcyl=%d(%d)\n",
							fcc_mah, chip->fcc_resolution, MAX_U8_VALUE, chargecycles, CHGCYL_RESOLUTION * MAX_U8_VALUE);

		return -EINVAL;
	}

	if (chip->fcc_sample_count == chip->min_fcc_learning_samples) {
		
		min_cycle = chip->fcc_learning_samples[0].chargecycles;
		for (i = 1; i < chip->min_fcc_learning_samples; i++) {
			if (min_cycle >
				chip->fcc_learning_samples[i].chargecycles) {
				pos = i;
				break;
			}
		}
	} else {
		
		for (i = 0; i < chip->min_fcc_learning_samples; i++) {
			ft = &chip->fcc_learning_samples[i];
			if (ft->fcc_new == 0 || (ft->fcc_new == 0xFF &&
						ft->chargecycles == 0xFF)) {
				pos = i;
				break;
			}
		}
		chip->fcc_sample_count++;
	}

	chip->fcc_learning_samples[pos].fcc_new = fcc_mah;
	chip->fcc_learning_samples[pos].chargecycles = chargecycles;

	fcc_new = DIV_ROUND_UP(fcc_mah, chip->fcc_resolution);

	pr_info("fcc_new=%d, fcc_mah=%d, chip->fcc_resolution=%d\n", fcc_new, fcc_mah, chip->fcc_resolution);

	rc = qpnp_write_wrapper(chip, (u8 *)&fcc_new,
			chip->base + BMS_FCC_BASE_REG + pos, 1);
	if (rc)
		return rc;

	chgcyl = DIV_ROUND_UP(chargecycles, CHGCYL_RESOLUTION);
	rc = qpnp_write_wrapper(chip, (u8 *)&chgcyl,
			chip->base + BMS_CHGCYL_BASE_REG + pos, 1);
	if (rc)
		return rc;

	pr_info("Store fcc_new=%d, chargecycle=%d, pos=%d\n",fcc_new, chgcyl, pos);

	return rc;
}

static void update_fcc_learning_table(struct qpnp_bms_chip *chip,
			int new_fcc_uah, int chargecycles, int batt_temp)
{
	int rc, fcc_default, fcc_temp;

	
	fcc_default = calculate_fcc(chip, FCC_DEFAULT_TEMP) / 1000;
	fcc_temp = calculate_fcc(chip, batt_temp) / 1000;
	new_fcc_uah = (new_fcc_uah / fcc_temp) * fcc_default;
	pr_info("Backup new FCC: fcc_default=%d, fcc_temp=%d, new_fcc_uah=%d\n",
						fcc_default, fcc_temp, new_fcc_uah);
	new_fcc_uah = (new_fcc_uah * 100) / 97;
	pr_info("new fcc(Add UUC)=%d\n",new_fcc_uah);

	rc = backup_new_fcc(chip, new_fcc_uah / 1000, chargecycles);
	if (rc) {
		pr_err("Unable to backup new FCC\n");
		return;
	}
	
	attempt_learning_new_fcc(chip);
}

static bool is_new_fcc_valid(int new_fcc_uah, int fcc_uah)
{
	if ((new_fcc_uah >= (fcc_uah / 2)) &&
		((new_fcc_uah * 100) <= (fcc_uah * 105)))
		return true;

	pr_info("FCC rejected - not within valid limit\n");
	return false;
}

static void fcc_learning_config(struct qpnp_bms_chip *chip, bool start)
{
	int rc, batt_temp;
	struct raw_soc_params raw;
	struct qpnp_vadc_result result;
	int fcc_uah, new_fcc_uah, delta_cc_uah, delta_soc;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &result);
	if (rc) {
		pr_err("Unable to read batt_temp\n");
		return;
	} else {
		batt_temp = (int)result.physical;
	}

	rc = read_soc_params_raw(chip, &raw, batt_temp);
	if (rc) {
		pr_err("Unable to read CC, cannot update FCC\n");
		return;
	}

	if (start) {
		chip->start_pc = interpolate_pc(chip->pc_temp_ocv_lut,
			batt_temp, raw.last_good_ocv_uv / 1000);
		chip->start_cc_uah = calculate_cc(chip, raw.cc, CC, NORESET);
		chip->start_real_soc = calculate_real_soc(chip,
				batt_temp, &raw, chip->start_cc_uah);
		pr_info("start_pc=%d, start_cc=%d, start_soc=%d real_soc=%d\n",
			chip->start_pc, chip->start_cc_uah,
			chip->start_soc, chip->start_real_soc);
		if((store_start_soc == 0) || (store_start_soc > chip->start_soc)){
			store_start_soc = chip->start_soc;
			store_start_cc = chip->start_cc_uah;
			store_start_pc = chip->start_pc;
			store_start_real_soc = chip->start_real_soc;
			pr_info("Store_start_pc=%d, start_cc=%d, start_soc=%d real_soc=%d\n",
				store_start_pc, store_start_cc,
				store_start_soc, store_start_real_soc);
		}
	} else {
		if(gbFCC_Update){
			chip->end_cc_uah = pre_cc_uah;
			pre_cc_uah = 0;
			gbFCC_Update = false;
			pr_info("using the stored end cc!!\n");
		}else{
			chip->end_cc_uah = calculate_cc(chip, raw.cc, CC, NORESET);
		}

		delta_soc = 100 - chip->start_real_soc;
		delta_cc_uah = abs(chip->end_cc_uah - chip->start_cc_uah);
		new_fcc_uah = div_u64(delta_cc_uah * 100, delta_soc);
		fcc_uah = calculate_fcc(chip, batt_temp);
		pr_info("start_soc=%d, start_pc=%d, start_real_soc=%d, start_cc=%d, end_cc=%d, new_fcc=%d\n",
			chip->start_soc, chip->start_pc, chip->start_real_soc,
			chip->start_cc_uah, chip->end_cc_uah, new_fcc_uah);

		if (is_new_fcc_valid(new_fcc_uah, fcc_uah))
			update_fcc_learning_table(chip, new_fcc_uah,
					chip->charge_cycles, batt_temp);
	}
}

#define MAX_CAL_TRIES	200
#define MIN_CAL_UA	3000
static void batfet_open_work(struct work_struct *work)
{
	int i;
	int rc;
	int result_ua;
	u8 orig_delay, sample_delay;
	struct qpnp_bms_chip *chip = container_of(work,
				struct qpnp_bms_chip,
				batfet_open_work);

	rc = qpnp_read_wrapper(chip, &orig_delay,
			chip->base + BMS1_S1_DELAY_CTL, 1);

	sample_delay = 0x0;
	rc = qpnp_write_wrapper(chip, &sample_delay,
			chip->base + BMS1_S1_DELAY_CTL, 1);


	for (i = 0; (!chip->batfet_closed) && i < MAX_CAL_TRIES; i++) {
		rc = qpnp_iadc_calibrate_for_trim(chip->iadc_dev, false);
		msleep(20);
		rc |= get_battery_current(chip, &result_ua);
		if (rc == 0 && abs(result_ua) <= MIN_CAL_UA) {
			pr_debug("good cal at %d attempt\n", i);
			break;
		}
	}
	pr_debug("batfet_closed = %d i = %d result_ua = %d\n",
			chip->batfet_closed, i, result_ua);

	rc = qpnp_write_wrapper(chip, &orig_delay,
			chip->base + BMS1_S1_DELAY_CTL, 1);
}

static void disable_sw_ocv_estimated_with_reason(bool disable, int reason)
{
	if (sw_ocv_estimated_stop_active_mask & reason) {
		if (disable)
			sw_ocv_estimated_stop_reason |= reason;
		else
			sw_ocv_estimated_stop_reason &= ~reason;
	}
}

static void charging_began(struct qpnp_bms_chip *chip)
{
	mutex_lock(&chip->last_soc_mutex);
	chip->charge_start_tm_sec = 0;
	chip->catch_up_time_sec = 0;
	mutex_unlock(&chip->last_soc_mutex);

	chip->start_soc = report_state_of_charge(chip);

	mutex_lock(&chip->last_ocv_uv_mutex);
	if (chip->enable_fcc_learning)
		fcc_learning_config(chip, true);
	chip->soc_at_cv = -EINVAL;
	chip->prev_chg_soc = -EINVAL;
	mutex_unlock(&chip->last_ocv_uv_mutex);
}

static void charging_ended(struct qpnp_bms_chip *chip)
{
	mutex_lock(&chip->last_soc_mutex);
	chip->charge_start_tm_sec = 0;
	chip->catch_up_time_sec = 0;
	mutex_unlock(&chip->last_soc_mutex);

	chip->end_soc = report_state_of_charge(chip);

	mutex_lock(&chip->last_ocv_uv_mutex);
	chip->soc_at_cv = -EINVAL;
	chip->prev_chg_soc = -EINVAL;

	
	if (chip->end_soc > chip->start_soc) {
		pr_info("Upate the chargecycle: end:%d, start:%d.\n",chip->end_soc,chip->start_soc);
		chip->charge_increase += (chip->end_soc - chip->start_soc);
		if (chip->charge_increase > 100) {
			chip->charge_cycles++;
			chip->charge_increase = chip->charge_increase % 100;
		}
		if (chip->enable_fcc_learning)
			backup_charge_cycle(chip);
		pr_info("Upate the chargecycle: cycle:%d, increase:%d.\n",chip->charge_cycles,chip->charge_increase);
	}

	if (get_battery_status(chip) == POWER_SUPPLY_STATUS_FULL) {
		if((store_start_soc >0) && (store_start_soc < chip->start_soc))
		{
			chip->start_soc = store_start_soc ;
			chip->start_cc_uah = store_start_cc;
			chip->start_pc = store_start_pc;
			chip->start_real_soc = store_start_real_soc;
			pr_info("re-store: start_pc=%d, start_cc=%d, start_soc=%d real_soc=%d\n",
			chip->start_pc, chip->start_cc_uah,
			chip->start_soc, chip->start_real_soc);
			store_start_soc = 0;
		}
		if (chip->enable_fcc_learning &&
			(chip->start_soc <= chip->min_fcc_learning_soc) &&
			(chip->start_pc <= chip->min_fcc_ocv_pc))
			fcc_learning_config(chip, false);
		pr_info("Charging Full,start_pc=%d, min_fcc_ocv_pc=%d, start_soc=%d min_fcc_learning_soc=%d\n",
			chip->start_pc, chip->min_fcc_ocv_pc, chip->start_soc, chip->min_fcc_learning_soc);
		
		chip->last_soc_invalid = true;
	} else if (chip->charging_adjusted_ocv > 0) {
		pr_debug("Charging stopped before full, adjusted OCV = %d\n",
				chip->charging_adjusted_ocv);
		chip->last_ocv_uv = chip->charging_adjusted_ocv;
	}

	chip->charging_adjusted_ocv = -EINVAL;

	mutex_unlock(&chip->last_ocv_uv_mutex);
}

static void battery_status_check(struct qpnp_bms_chip *chip)
{
	int status = get_battery_status(chip);
	struct timespec xtime;
	unsigned long currtime_ms;

	pr_info("Battery get status : %d -> %d \n", chip->battery_status, status);

	mutex_lock(&chip->status_lock);
	if (chip->battery_status != status) {
		pr_debug("status = %d, shadow status = %d\n",
				status, chip->battery_status);
		if (status == POWER_SUPPLY_STATUS_CHARGING) {
			pr_debug("charging started\n");
			charging_began(chip);
			gbFCC_Start = true;
			
			disable_sw_ocv_estimated_with_reason(false, OCV_UPDATE_STOP_BIT_CABLE_OUT);
			allow_sw_ocv_est_time = 0;
		} else if (chip->battery_status
				== POWER_SUPPLY_STATUS_CHARGING) {
			pr_debug("charging ended\n");
			charging_ended(chip);
			gbFCC_Start = false;
			gbFCC_Update = true;
			
			xtime = CURRENT_TIME;
			currtime_ms = xtime.tv_sec * MSEC_PER_SEC + xtime.tv_nsec / NSEC_PER_MSEC;
			disable_sw_ocv_estimated_with_reason(true, OCV_UPDATE_STOP_BIT_CABLE_OUT);
			allow_sw_ocv_est_time = currtime_ms + 1800000;
			pr_info("Inhibit estimating sw ocv, currtime=%lu, allow_sw_ocv_est_time=%lu\n",
					currtime_ms, allow_sw_ocv_est_time);
		}else{
			gbFCC_Start = false;
			gbFCC_Update = false;
			pre_cc_uah = 0;
		}

		if (status == POWER_SUPPLY_STATUS_FULL) {
			pr_debug("battery full\n");
			enable_bms_irq(&chip->ocv_thr_irq);
			
			recalculate_soc(chip);
		} else if (chip->battery_status
				== POWER_SUPPLY_STATUS_FULL) {
			pr_debug("battery not full any more\n");
			disable_bms_irq(&chip->ocv_thr_irq);
			
		}

		chip->battery_status = status;
		schedule_work(&chip->recalc_work);
	}
	mutex_unlock(&chip->status_lock);
}

#define CALIB_WRKARND_DIG_MAJOR_MAX		0x03
static void batfet_status_check(struct qpnp_bms_chip *chip)
{
	bool batfet_closed;

	if (chip->iadc_bms_revision2 > CALIB_WRKARND_DIG_MAJOR_MAX)
		return;

	batfet_closed = is_batfet_closed(chip);
	if (chip->batfet_closed != batfet_closed) {
		chip->batfet_closed = batfet_closed;
		if (batfet_closed == false) {
			
			schedule_work(&chip->batfet_open_work);
			qpnp_iadc_skip_calibration(chip->iadc_dev);
		} else {
			
			qpnp_iadc_calibrate_for_trim(chip->iadc_dev, true);
			qpnp_iadc_resume_calibration(chip->iadc_dev);
		}
	}
}

static void battery_insertion_check(struct qpnp_bms_chip *chip)
{
	int present = (int)is_battery_present(chip);
	int insertion_ocv_uv = get_battery_insertion_ocv_uv(chip);
	int insertion_ocv_taken = (insertion_ocv_uv > 0);

	mutex_lock(&chip->vbat_monitor_mutex);
	if (chip->battery_present != present
			&& (present == insertion_ocv_taken
				|| chip->battery_present == -EINVAL)) {
		pr_debug("status = %d, shadow status = %d, insertion_ocv_uv = %d\n",
				present, chip->battery_present,
				insertion_ocv_uv);
		if (chip->battery_present != -EINVAL) {
			if (present) {
				chip->insertion_ocv_uv = insertion_ocv_uv;
#if !(defined(CONFIG_HTC_BATT_8960))
				setup_vbat_monitoring(chip);
#endif
				chip->new_battery = true;
			} else {
				reset_vbat_monitoring(chip);
			}
		}
		chip->battery_present = present;
		schedule_work(&chip->recalc_work);
	}
	mutex_unlock(&chip->vbat_monitor_mutex);
}

static int get_prop_bms_capacity(struct qpnp_bms_chip *chip)
{
	return report_state_of_charge(chip);
}

static void qpnp_bms_external_power_changed(struct power_supply *psy)
{
	struct qpnp_bms_chip *chip = container_of(psy, struct qpnp_bms_chip,
								bms_psy);

	battery_insertion_check(chip);
	batfet_status_check(chip);
	battery_status_check(chip);
}

static int qpnp_bms_power_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct qpnp_bms_chip *chip = container_of(psy, struct qpnp_bms_chip,
								bms_psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = get_prop_bms_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->battery_status;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = get_prop_bms_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_RESISTANCE:
		val->intval = get_prop_bms_batt_resistance(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = get_prop_bms_charge_counter(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER_SHADOW:
		val->intval = get_prop_bms_charge_counter_shadow(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = get_prop_bms_charge_full_design(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = get_prop_bms_charge_full(chip);
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = chip->charge_cycles;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
	

static ssize_t kernel_write(struct file *file, const char *buf,
	size_t count, loff_t pos)
{
	mm_segment_t old_fs;
	ssize_t res;

	old_fs = get_fs();
	set_fs(get_ds());
	
	res = vfs_write(file, (const char __user *)buf, count, &pos);
	set_fs(old_fs);

	return res;
}

int emmc_misc_write(int val, int offset)
{
	char filename[32] = "";
	int w_val = val;
	struct file *filp = NULL;
	ssize_t nread;
	int pnum = get_partition_num_by_name("misc");

	if (pnum < 0) {
		pr_warn("unknown partition number for misc partition\n");
		return 0;
	}
	sprintf(filename, "/dev/block/mmcblk0p%d", pnum);

	filp = filp_open(filename, O_RDWR, 0);
	if (IS_ERR(filp)) {
		pr_info("unable to open file: %s\n", filename);
		return PTR_ERR(filp);
	}

	filp->f_pos = offset;
	nread = kernel_write(filp, (char *)&w_val, sizeof(int), filp->f_pos);
	pr_info("%X (%d)\n", w_val, nread);

	filp_close(filp, NULL);

	return 1;
}

int pm8941_bms_get_batt_current(int *result)
{
	if (!the_chip) {
		pr_warn("called before init\n");
		return -EINVAL;
	}

	*result = get_prop_bms_current_now(the_chip);
	return 0;
}
int pm8941_bms_get_batt_soc(int *result)
{
	int state_of_charge;
	struct timespec xtime;
	unsigned long currtime_ms;
	unsigned long time_since_last_update_ms, cur_jiffies;

	xtime = CURRENT_TIME;
	currtime_ms = xtime.tv_sec * MSEC_PER_SEC + xtime.tv_nsec / NSEC_PER_MSEC;

	if (!the_chip) {
		pr_warn("called before init\n");
		return -EINVAL;
	}

	batt_level = state_of_charge = *result = recalculate_soc(the_chip);

	if (new_boot_soc && allow_ocv_time &&
		(currtime_ms >= allow_ocv_time)) {
		pr_info("OCV can be update due to currtime(%lu) >= allow_ocv_time(%lu) "
				"(OCV_UPDATE_STOP_BIT_BOOT_UP)\n",
				currtime_ms, allow_ocv_time);
		new_boot_soc = 0;
		allow_ocv_time = 0;
		
		disable_ocv_update_with_reason(false, OCV_UPDATE_STOP_BIT_BOOT_UP);
	}

	if (allow_sw_ocv_est_time &&
			(currtime_ms >= allow_sw_ocv_est_time)) {
		pr_info("SW OCV can be estimated due to currtime(%lu) >= allow_sw_ocv_est_time(%lu) "
				"(OCV_UPDATE_STOP_BIT_CABLE_OUT)\n",
				currtime_ms, allow_sw_ocv_est_time);
		allow_sw_ocv_est_time = 0;
		
		disable_sw_ocv_estimated_with_reason(false, OCV_UPDATE_STOP_BIT_CABLE_OUT);
	}

	if (the_chip->store_batt_data_soc_thre > 0
			&& state_of_charge <= the_chip->store_batt_data_soc_thre
			&& (store_soc_ui >= 0 && store_soc_ui <= 100)) {
		store_emmc.store_soc = store_soc_ui;
		store_emmc.store_currtime_ms = currtime_ms;
	}

	
	if (the_chip->criteria_sw_est_ocv > 0) {
		cur_jiffies = jiffies;
		time_since_last_update_ms =
			(cur_jiffies - htc_batt_bms_timer.batt_system_jiffies) * MSEC_PER_SEC / HZ;
		htc_batt_bms_timer.no_ocv_update_period_ms += time_since_last_update_ms;
		htc_batt_bms_timer.batt_system_jiffies = cur_jiffies;
	}

	return 0;
}
int pm8941_bms_get_batt_cc(int *result)
{
	if (!the_chip) {
		pr_warn("called before init\n");
		return -EINVAL;
	}

	*result = get_prop_bms_charge_counter(the_chip);

	return 0;
}

int pm8941_bms_get_fcc(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	return get_prop_bms_charge_full(the_chip);
}

static int get_bms_reg(void *data, u64 *val)
{
	int addr = (int)data;
	int rc;
	u8 bms_sts;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	rc = qpnp_read_wrapper(the_chip, &bms_sts,
			the_chip->base + addr, 1);
	if (rc) {
		pr_err("failed to read BMS1 register sts %d\n", rc);
		return -EAGAIN;
	}
	pr_debug("addr:0x%X, val:0x%X\n", (the_chip->base + addr), bms_sts);
	*val = bms_sts;
	return 0;
}

static int get_iadc_reg(void *data, u64 *val)
{
	int addr = (int)data;
	int rc;
	u8 iadc_sts;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	rc = qpnp_read_wrapper(the_chip, &iadc_sts,
			the_chip->iadc_base + addr, 1);
	if (rc) {
		pr_err("failed to read IADC1 register sts %d\n", rc);
		return -EAGAIN;
	}
	pr_debug("addr:0x%X, val:0x%X\n", (the_chip->iadc_base + addr), iadc_sts);
	*val = iadc_sts;
	return 0;
}

static int dump_all(void)
{
	u64 val;
	unsigned int len =0;
	int batt_temp, rc;
	struct raw_soc_params raw;
	struct qpnp_vadc_result result;
	unsigned long flags;

	memset(batt_log_buf, 0, sizeof(BATT_LOG_BUF_LEN));

	
	get_bms_reg((void *)BMS1_STATUS1, &val);
	len += scnprintf(batt_log_buf + len, BATT_LOG_BUF_LEN - len,
		"BMS1_STATUS1=0x%02llx,", val);
	get_bms_reg((void *)BMS1_INT_RT_STS, &val);
	len += scnprintf(batt_log_buf + len, BATT_LOG_BUF_LEN - len,
		"BMS1_INT_RT_STS=0x%02llx,", val);
	get_bms_reg((void *)BMS1_TOL_CTL, &val);
	len += scnprintf(batt_log_buf + len, BATT_LOG_BUF_LEN - len,
		"TOL_CTL=0x%02llx,", val);
	get_bms_reg((void *)BMS1_OCV_USE_LOW_LIMIT_THR0, &val);
	len += scnprintf(batt_log_buf + len, BATT_LOG_BUF_LEN - len,
		"OCV_USE_LOW_LIMIT_THR0=0x%02llx,", val);
	get_bms_reg((void *)BMS1_OCV_USE_HIGH_LIMIT_THR0, &val);
	len += scnprintf(batt_log_buf + len, BATT_LOG_BUF_LEN - len,
		"OCV_USE_HIGH_LIMIT_THR0=0x%02llx,", val);
	get_bms_reg((void *)BMS1_OCV_USE_LIMIT_CTL, &val);
	len += scnprintf(batt_log_buf + len, BATT_LOG_BUF_LEN - len,
		"OCV_USE_LIMIT_CTL=0x%02llx,", val);
	get_bms_reg((void *)BMS1_S3_VSENSE_THR_CTL , &val);
	len += scnprintf(batt_log_buf + len, BATT_LOG_BUF_LEN - len,
		"S3_VSENSE_THR_CTL =0x%02llx,", val);

	
	rc = qpnp_vadc_read(the_chip->vadc_dev, LR_MUX1_BATT_THERM,
								&result);
	if (rc) {
		pr_err("error reading vadc LR_MUX1_BATT_THERM = %d, rc = %d\n",
			LR_MUX1_BATT_THERM, rc);
		return rc;
	}
	batt_temp = (int)result.physical;
	read_soc_params_raw(the_chip, &raw, batt_temp);

	len += scnprintf(batt_log_buf + len, BATT_LOG_BUF_LEN - len,
		"cc(uAh)=%d,", bms_dbg.cc_uah);
	get_bms_reg((void *)SOC_STORAGE_REG, &val);
	len += scnprintf(batt_log_buf + len, BATT_LOG_BUF_LEN - len,
		"SOC_STORAGE_REG=0x%lld,", val);
	get_bms_reg((void *)IAVG_STORAGE_REG, &val);
	len += scnprintf(batt_log_buf + len, BATT_LOG_BUF_LEN - len,
		"IAVG_STORAGE_REG=0x%lld,", val);

	
	get_iadc_reg((void *)IADC1_BMS_ADC_CH_SEL_CTL, &val);
	len += scnprintf(batt_log_buf + len, BATT_LOG_BUF_LEN - len,
		"ADC_CH_SEL_CTL=0x%lld ", val);

	
	local_irq_save(flags);
	len += scnprintf(batt_log_buf + len, BATT_LOG_BUF_LEN - len,
	"[irq]%d%d%d%d %d%d%d%d",
		irq_read_line(the_chip->cc_thr_irq.irq),
		irq_read_line(the_chip->ocv_for_r_irq.irq),
		irq_read_line(the_chip->good_ocv_irq.irq),
		irq_read_line(the_chip->charge_begin_irq.irq),
		irq_read_line(the_chip->sw_cc_thr_irq.irq),
		irq_read_line(the_chip->ocv_thr_irq.irq),
		irq_read_line(the_chip->vsense_avg_irq.irq),
		irq_read_line(the_chip->vsense_for_r_irq.irq));
	local_irq_restore(flags);

	
	if(BATT_LOG_BUF_LEN - len <= 1)
		pr_warn("batt log length maybe out of buffer range!!!");

	pr_info("%s\n", batt_log_buf);
	return 0;
}

static int bms_set_chgcyl_table(struct device_node *data_node, int check_batt_id)
{
	struct property *prop;
	const __be32 *data;
	int max_cols = 6, cols;
	int id_from_dt;
	int i, rc;
	struct device_node *node, *final_node=NULL;

	node = data_node;
	for_each_child_of_node(data_node, node) {
		rc = of_property_read_u32(node, "htc,batt_id", &id_from_dt);
		if (rc) {
			pr_warn("get batt_id from dt failed rc=%d\n", rc);
			continue;
		}

		pr_info("[%s]search ID = %d (%d)\n",node->name, id_from_dt, check_batt_id);
		
		if (check_batt_id == id_from_dt){
			break;
		}
	}

	final_node = of_find_node_by_name(node, "htc,fcc-chgcyl-tab");

	if (final_node == NULL) {
		pr_err("Couldn't find data node.\n");
		return -EINVAL;
	}

	prop = of_find_property(final_node, "htc,chgcyl", NULL);
	if (!prop) {
		pr_err("%s: No chgcyl found\n",final_node->name);
		return -EINVAL;
	} else if (!prop->value) {
		pr_err("%s: No chycyl value found\n", final_node->name);
		return -ENODATA;
	}

	cols = prop->length/sizeof(int);
	if (cols != max_cols) {
		pr_err("%s: data(%d) and size(%d) are incorrect.\n",final_node->name, cols, max_cols);
		return -EINVAL;
	}

	data = prop->value;
	for (i = 0; i < cols; i++){
		chgcyl_checking_table[i][0] = be32_to_cpup(data++);
		pr_info("table [%d][0]: %d\n",i, chgcyl_checking_table[i][0]);
	}

	prop = of_find_property(final_node, "htc,mini", NULL);
	if (!prop) {
		pr_err("%s: No mini found\n",final_node->name);
		return -EINVAL;
	} else if (!prop->value) {
		pr_err("%s: No mini value found\n", final_node->name);
		return -ENODATA;
	}

	cols = prop->length/sizeof(int);
	if (cols != max_cols) {
		pr_err("%s: data(%d) and size(%d) are incorrect.\n", final_node->name, cols, max_cols);
		return -EINVAL;
	}

	data = prop->value;
	for (i = 0; i < cols; i++){
		chgcyl_checking_table[i][1] = be32_to_cpup(data++);
		pr_info("table [%d][1]: %d\n",i, chgcyl_checking_table[i][1]);
	}
	return 0;
}

inline int pm8941_bms_dump_all(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	dump_all();
	battery_status_check(the_chip);
	return 0;
}

int pm8941_bms_get_attr_text(char *buf, int size)
{
	struct raw_soc_params raw;
	int len = 0;
	u64 val = 0;
	int i = 0;
	struct soc_params params;
	
	int batt_temp, rc, soc_rbatt, shdw_cc_uah;
	int remaining_usable_charge_uah;
	struct qpnp_vadc_result result;

	if (!the_chip) {
		pr_err("driver not initialized\n");
		return 0;
	}
	len += scnprintf(buf + len, size - len,
		"is_ocv_update_start: %d;\n", is_ocv_update_start);
	len += scnprintf(buf + len, size - len,
		"consistent_flag: %d;\n", consistent_flag);
	get_bms_reg((void *)BMS1_STATUS1, &val);
	len += scnprintf(buf + len, size - len,
		"BMS1_STATUS1: 0x%02llx;\n", val);
	get_bms_reg((void *)BMS1_INT_RT_STS, &val);
	len += scnprintf(buf + len, size - len,
		"BMS1_INT_RT_STS: 0x%02llx;\n", val);
	get_bms_reg((void *)BMS1_TOL_CTL, &val);
	len += scnprintf(buf + len, size - len,
		"TOL_CTL: 0x%02llx;\n", val);
	get_bms_reg((void *)BMS1_OCV_USE_LOW_LIMIT_THR0, &val);
	len += scnprintf(buf + len, size - len,
		"OCV_USE_LOW_LIMIT_THR0: 0x%02llx;\n", val);
	get_bms_reg((void *)BMS1_OCV_USE_HIGH_LIMIT_THR0, &val);
	len += scnprintf(buf + len, size - len,
		"OCV_USE_HIGH_LIMIT_THR0: 0x%02llx;\n", val);
	get_bms_reg((void *)BMS1_OCV_USE_LIMIT_CTL, &val);
	len += scnprintf(buf + len, size - len,
		"OCV_USE_LIMIT_CTL: 0x%02llx;\n", val);
	get_bms_reg((void *)BMS1_S3_VSENSE_THR_CTL , &val);
	len += scnprintf(buf + len, size - len,
		"S3_VSENSE_THR_CTL: 0x%02llx;\n", val);

	
	get_bms_reg((void *)SOC_STORAGE_REG, &val);
	len += scnprintf(buf + len, size - len,
		"SOC_STORAGE_REG: 0x%lld;\n", val);
	get_bms_reg((void *)IAVG_STORAGE_REG, &val);
	len += scnprintf(buf + len, size - len,
		"IAVG_STORAGE_REG: 0x%lld;\n", val);

	
	get_iadc_reg((void *)IADC1_BMS_ADC_CH_SEL_CTL, &val);
	len += scnprintf(buf + len, size - len,
		"ADC_CH_SEL_CTL: 0x%lld;\n", val);

	rc = qpnp_vadc_read(the_chip->vadc_dev, LR_MUX1_BATT_THERM,
								&result);
	if (rc) {
		pr_err("error reading vadc LR_MUX1_BATT_THERM = %d, rc = %d\n",
			LR_MUX1_BATT_THERM, rc);
		return len;
	}
	batt_temp = (int)result.physical;

	mutex_lock(&the_chip->last_ocv_uv_mutex);
	read_soc_params_raw(the_chip, &raw, batt_temp);
	len += scnprintf(buf + len, size - len,
			"LAST_GOOD_OCV_RAW: 0x%x;\n", raw.last_good_ocv_raw);
	len += scnprintf(buf + len, size - len,
			"CC_RAW: 0x%llx;\n", raw.cc);
	len += scnprintf(buf + len, size - len,
			"last_good_ocv_uv: %duV;\n", raw.last_good_ocv_uv);
	len += scnprintf(buf + len, size - len,
			"ori_last_good_ocv_uv: %duV;\n", bms_dbg.last_ocv_raw_uv);
	len += scnprintf(buf + len, size - len,
			"backup_last_good_ocv_uv: %duV;\n", the_chip->ocv_backup_uv);
	calculate_delta_time(&the_chip->tm_sec, &params.delta_time_s);
	params.fcc_uah = calculate_fcc(the_chip, batt_temp);
	params.ocv_charge_uah = calculate_ocv_charge(the_chip, &raw,
					params.fcc_uah);
	params.cc_uah = calculate_cc(the_chip, raw.cc, CC, NORESET);
	params.cc_uah -= the_chip->cc_backup_uah;
	shdw_cc_uah = calculate_cc(the_chip, raw.shdw_cc,
					SHDW_CC, NORESET);
	soc_rbatt = ((params.ocv_charge_uah - params.cc_uah) * 100) /params.fcc_uah;
	if (soc_rbatt < 0)
		soc_rbatt = 0;
	params.rbatt_mohm = get_rbatt(the_chip, soc_rbatt, batt_temp);
	calculate_iavg(the_chip, params.cc_uah, &params.iavg_ua,
						params.delta_time_s);
	params.uuc_uah = calculate_unusable_charge_uah(the_chip, &params,
							batt_temp);
	remaining_usable_charge_uah = params.ocv_charge_uah
					- params.cc_uah
					- params.uuc_uah;
	mutex_unlock(&the_chip->last_ocv_uv_mutex);
	len += scnprintf(buf + len, size - len,
			"rbatt(milliOhms): %d;\n", params.rbatt_mohm);
	len += scnprintf(buf + len, size - len,
			"rbatt_scalefactor: %d;\n", bms_dbg.rbatt_sf);
	len += scnprintf(buf + len, size - len,
			"soc_rbatt(%%): %d;\n", soc_rbatt);
	len += scnprintf(buf + len, size - len,
			"unusable_uv: %d;\n", bms_dbg.unusable_uv);
	len += scnprintf(buf + len, size - len,
			"pc_unusable(%%): %d;\n", bms_dbg.pc_unusable);
	len += scnprintf(buf + len, size - len,
			"rc_pc(%%): %d;\n", bms_dbg.rc_pc);
	len += scnprintf(buf + len, size - len,
			"fcc(uAh): %d;\n", params.fcc_uah);
	len += scnprintf(buf + len, size - len,
			"unusable_charge(uAh): %d;\n", params.uuc_uah);
	len += scnprintf(buf + len, size - len,
			"remaining_charge(uAh): %d;\n", params.ocv_charge_uah);
	len += scnprintf(buf + len, size - len,
			"remaining_usable_charge_uah: %d;\n",
			remaining_usable_charge_uah);
	len += scnprintf(buf + len, size - len,
			"uuc_iavg_ma: %d;\n", bms_dbg.uuc_iavg_ma);
	len += scnprintf(buf + len, size - len,
			"uuc_rbatt_mohm: %d;\n", bms_dbg.uuc_rbatt_mohm);
	len += scnprintf(buf + len, size - len,
			"ori_unusable_charge(uAh): %d;\n", bms_dbg.ori_uuc_uah);
	len += scnprintf(buf + len, size - len,
			"cc(uAh): %d;\n", params.cc_uah);
	len += scnprintf(buf + len, size - len,
			"ori_cc(uAh): %d;\n", bms_dbg.ori_cc_uah);
	len += scnprintf(buf + len, size - len,
			"backup_cc(uAh): %d;\n", the_chip->cc_backup_uah);
	len += scnprintf(buf + len, size - len,
			"start_soc: %d;\n", the_chip->start_soc);
	len += scnprintf(buf + len, size - len,
			"end_soc: %d;\n", the_chip->end_soc);
	len += scnprintf(buf + len, size - len,
			"shdw_cc_uah(uAh): %d;\n", shdw_cc_uah);
	len += scnprintf(buf + len, size - len,
			"pon_est_ocv: %d;\n", bms_dbg.pon_est_ocv);
	len += scnprintf(buf + len, size - len,
			"ibat_for_est_ocv: %d;\n", bms_dbg.ibat_for_est_ocv);
	len += scnprintf(buf + len, size - len,
			"vbat_for_est_ocv: %d;\n", bms_dbg.vbat_for_est_ocv);
	len += scnprintf(buf + len, size - len,
			"rbat_for_est_ocv: %d;\n", bms_dbg.rbat_for_est_ocv);
	
	
	

	
	if((the_chip) && (the_chip->enable_fcc_learning)){
		len += scnprintf(buf + len, size - len,
				"Charge_Cycles: %d\n", the_chip->charge_cycles);
		len += scnprintf(buf + len, size - len,
				"Charge_Increase: %d\n", the_chip->charge_increase);
		len += scnprintf(buf + len, size - len,
				"Sample_count: %d\n", the_chip->fcc_sample_count);
		for ( i = 0; i < the_chip->min_fcc_learning_samples; i++){
			len += scnprintf(buf + len, size - len,
					"Sample[%d]: %dmAh-%dcycles\n", i, the_chip->fcc_learning_samples[i].fcc_new,
					the_chip->fcc_learning_samples[i].chargecycles);
		}
	}
	return len;
}

int pm8941_get_batt_id(int *result)
{
	int64_t battery_id_raw;
	int battery_id_mv;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	battery_id_raw = read_battery_id(the_chip);
	if (battery_id_raw < 0) {
		pr_err("cannot read battery id err = %lld\n", battery_id_raw);
		return -EINVAL;
	}

	battery_id_mv = (int)battery_id_raw / 1000;
	
	*result = htc_battery_cell_find_and_set_id_auto(battery_id_mv);

	return 0;
}

int pm8941_get_batt_id_mv(int *result)
{
	int64_t battery_id_raw;
	int battery_id_mv;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	battery_id_raw = read_battery_id(the_chip);
	if (battery_id_raw < 0) {
		pr_err("cannot read battery id err = %lld\n", battery_id_raw);
		return -EINVAL;
	}

	battery_id_mv = (int)battery_id_raw / 1000;
	
	*result = battery_id_mv;

	return 0;
}

int pm8941_bms_get_percent_charge(struct qpnp_bms_chip *chip)
{
	int rc, batt_temp, soc;
	struct raw_soc_params raw;
	struct qpnp_vadc_result result;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM,
								&result);
	if (rc) {
		pr_err("error reading vadc LR_MUX1_BATT_THERM = %d, rc = %d\n",
			LR_MUX1_BATT_THERM, rc);
		return rc;
	}
	batt_temp = (int)result.physical;

	mutex_lock(&chip->last_ocv_uv_mutex);
	read_soc_params_raw(chip, &raw, batt_temp);
	soc = calculate_state_of_charge(chip, &raw, batt_temp);
	mutex_unlock(&chip->last_ocv_uv_mutex);

	return soc;
}

int pm8941_bms_store_battery_gauge_data_emmc(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	
	if (the_chip->store_batt_data_soc_thre > 0
		&& store_emmc.store_soc > 0
		&& store_emmc.store_soc <= the_chip->store_batt_data_soc_thre) {

		emmc_misc_write(BMS_STORE_MAGIC_NUM, BMS_STORE_MAGIC_OFFSET);
		emmc_misc_write(store_emmc.store_soc, BMS_STORE_SOC_OFFSET);
		emmc_misc_write(store_emmc.store_ocv_uv, BMS_STORE_OCV_OFFSET);
		emmc_misc_write(store_emmc.store_cc_uah, BMS_STORE_CC_OFFSET);
		emmc_misc_write(store_emmc.store_currtime_ms, BMS_STORE_CURRTIME_OFFSET);

		pr_info("Stored soc=%d,OCV=%d,ori_cc_uah=%d,stored_cc_uah:%d,currtime_ms=%lu\n",
			store_emmc.store_soc, store_emmc.store_ocv_uv, bms_dbg.ori_cc_uah,
			store_emmc.store_cc_uah, store_emmc.store_currtime_ms);
	}

	return 0;
}

int pm8941_bms_store_battery_ui_soc(int soc_ui)
{
	if (soc_ui < 0 || soc_ui > 100)
		return -EINVAL;

	store_soc_ui = soc_ui;

	return 0;
}
int pm8941_bms_get_battery_ui_soc(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	pr_debug("batt_stored_soc: %d\n", the_chip->batt_stored_soc);

	
	if (the_chip->batt_stored_soc <= 0 || the_chip->batt_stored_soc > 100 || !consistent_flag)
		return -EINVAL;

	return the_chip->batt_stored_soc;
}

int pm8941_bms_stop_ocv_updates(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	if (!is_ocv_update_start) {
		pr_info("ocv updates is already stopped");
		return -EINVAL;
	}
	is_ocv_update_start = 0;
	pr_info("stopping ocv updates, is_ocv_update_start=%d", is_ocv_update_start);
	return qpnp_masked_write(the_chip, BMS1_TOL_CTL,
			OCV_TOL_MASK, OCV_TOL_NO_OCV);
}

int pm8941_bms_start_ocv_updates(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	if (is_ocv_update_start) {
		pr_info("ocv updates is already started");
		return -EINVAL;
	}
	is_ocv_update_start = 1;
	pr_info("starting ocv updates, is_ocv_update_start=%d", is_ocv_update_start);
	return qpnp_masked_write(the_chip, BMS1_TOL_CTL,
			OCV_TOL_MASK, OCV_TOL_DEFAULT);
}

static void disable_ocv_update_with_reason(bool disable, int reason)
{
	int prev_ocv_update_stop_reason;
	mutex_lock(&ocv_update_lock);
	prev_ocv_update_stop_reason = ocv_update_stop_reason;
	if (ocv_update_stop_active_mask & reason) {
		if (disable)
			ocv_update_stop_reason |= reason;
		else
			ocv_update_stop_reason &= ~reason;

		if (prev_ocv_update_stop_reason ^ ocv_update_stop_reason) {
			pr_info("ocv_update_stop_reason:0x%x->0x%d\n",
							prev_ocv_update_stop_reason, ocv_update_stop_reason);
			if (!!prev_ocv_update_stop_reason != !!ocv_update_stop_reason) {
				if (!!ocv_update_stop_reason)
					pm8941_bms_stop_ocv_updates();
				else
					pm8941_bms_start_ocv_updates();
			}
		}
	}
	mutex_unlock(&ocv_update_lock);
}

static void pm8941_btm_voltage_alarm_notify(enum qpnp_tm_state state, void *ctx)
{
	struct qpnp_bms_chip *chip = ctx;
	int vbat_uv;
	struct qpnp_vadc_result result;

	qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &result);
	pr_debug("vbat = %lld, raw = 0x%x\n", result.physical, result.adc_code);

	get_battery_voltage(chip, &vbat_uv);
	pr_info("vbat is at %d, state is at %d\n", vbat_uv, state);

	if (state == ADC_TM_LOW_STATE) {
		pr_debug("low voltage btm notification triggered\n");
		if (vbat_uv - VBATT_ERROR_MARGIN
				< chip->vbat_monitor_params.low_thr) {
			pm8941_batt_lower_alarm_threshold_set(0);
			htc_gauge_event_notify(HTC_GAUGE_EVENT_LOW_VOLTAGE_ALARM);
		} else {
			pr_debug("faulty btm trigger, discarding\n");
			qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
					&chip->vbat_monitor_params);
		}
	} else if (state == ADC_TM_HIGH_STATE) {
		pr_debug("high voltage btm notification triggered\n");
	} else {
		pr_debug("unknown voltage notification state: %d\n", state);
	}
}

int pm8941_batt_lower_alarm_threshold_set(int threshold_mV)
{
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	the_chip->vbat_monitor_params.low_thr = threshold_mV * 1000;
	the_chip->vbat_monitor_params.high_thr = the_chip->max_voltage_uv * 2;
	the_chip->vbat_monitor_params.state_request = ADC_TM_LOW_THR_ENABLE;
	the_chip->vbat_monitor_params.channel = VBAT_SNS;
	the_chip->vbat_monitor_params.btm_ctx = (void *)the_chip;
	the_chip->vbat_monitor_params.timer_interval = ADC_MEAS1_INTERVAL_1S;
	the_chip->vbat_monitor_params.threshold_notification = &pm8941_btm_voltage_alarm_notify;
	pr_debug("set low thr to %d and high to %d\n",
			the_chip->vbat_monitor_params.low_thr,
			the_chip->vbat_monitor_params.high_thr);

	if (!is_battery_present(the_chip)) {
		pr_debug("no battery inserted, do not enable vbat monitoring\n");
		the_chip->vbat_monitor_params.state_request =
			ADC_TM_HIGH_LOW_THR_DISABLE;
	} else {
		rc = qpnp_adc_tm_channel_measure(the_chip->adc_tm_dev,
						&the_chip->vbat_monitor_params);
		if (rc) {
			pr_err("tm setup failed: %d\n", rc);
		return rc;
		}
	}

	pr_debug("setup complete\n");

	return 0;
}

int pm8941_bms_enter_qb_mode(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if(the_chip->qb_mode_cc_criteria_uAh) {
		qb_mode_enter = true;
		qb_mode_cc_start = bms_dbg.cc_uah;
		qb_mode_ocv_start = bms_dbg.last_ocv_raw_uv;
		qb_mode_cc_accumulation_uah = 0;
		qb_mode_time_accumulation = 0;
		qb_mode_prev_cc = 0;
		qb_mode_over_criteria_count = 0;
		htc_gauge_event_notify(HTC_GAUGE_EVENT_QB_MODE_ENTER);
	}
	return 0;
}

int pm8941_bms_exit_qb_mode(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if(the_chip->qb_mode_cc_criteria_uAh) {
		qb_mode_enter = false;
		qb_mode_cc_accumulation_uah = 0;
		qb_mode_cc_start = 0;
		qb_mode_ocv_start = 0;
		qb_mode_time_accumulation = 0;
		qb_mode_prev_cc = 0;
		qb_mode_over_criteria_count = 0;
	}
	return 0;
}

#define SIXTY_MINUTES_MS				(1000 * (3600 - 10))
int pm8941_qb_mode_pwr_consumption_check(unsigned long time_since_last_update_ms)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if(qb_mode_enter && the_chip->qb_mode_cc_criteria_uAh) {
		qb_mode_time_accumulation += time_since_last_update_ms;

		if(qb_mode_ocv_start != bms_dbg.last_ocv_raw_uv) {
			
			qb_mode_cc_accumulation_uah += bms_dbg.cc_uah;
			
			qb_mode_prev_cc = qb_mode_cc_start = bms_dbg.cc_uah;

			pr_info("ocv update happened OCV_uV/ori=%duV/%duV, cc_value:%d\n",
				bms_dbg.last_ocv_raw_uv, qb_mode_ocv_start, bms_dbg.cc_uah);
			qb_mode_ocv_start = bms_dbg.last_ocv_raw_uv;
		} else {
			if(!qb_mode_prev_cc)
				
				qb_mode_cc_accumulation_uah = (bms_dbg.cc_uah - qb_mode_cc_start);
			else
				qb_mode_cc_accumulation_uah += (bms_dbg.cc_uah - qb_mode_prev_cc);
			qb_mode_prev_cc = bms_dbg.cc_uah;
		}

		if(qb_mode_time_accumulation >= SIXTY_MINUTES_MS) {
			if(qb_mode_cc_accumulation_uah > the_chip->qb_mode_cc_criteria_uAh) {
				qb_mode_over_criteria_count++;
				pr_warn("QB mode cc over criteria, cc_accu=%d, time_accu=%lu, count=%d\n",
					qb_mode_cc_accumulation_uah, qb_mode_time_accumulation,
					qb_mode_over_criteria_count);
			} else
				qb_mode_over_criteria_count = 0;

			qb_mode_time_accumulation = 0;
			qb_mode_cc_accumulation_uah = 0;
			qb_mode_cc_start = bms_dbg.cc_uah;
		}

		pr_info("qb_start_ocv_uV=%d,qb_start_cc_uAh=%d,qb_current_cc_uAh=%d,"
				"qb_cc_accumulate_uAh=%d,qb_time_accumulate_us=%lu,"
				"qb_cc_criteria_uAh=%d,over_cc_criteria_count=%d\n",
			qb_mode_ocv_start, qb_mode_cc_start, qb_mode_prev_cc,
			qb_mode_cc_accumulation_uah, qb_mode_time_accumulation,
			the_chip->qb_mode_cc_criteria_uAh, qb_mode_over_criteria_count);

		if(qb_mode_over_criteria_count >= 3) {
			pr_info("Force device shutdown due to over QB mode CC criteria!\n");
			htc_gauge_event_notify(HTC_GAUGE_EVENT_QB_MODE_DO_REAL_POWEROFF);
		}
	} else {
		
	}
	return 0;
}

#define OCV_USE_LIMIT_EN		BIT(7)
static int set_ocv_voltage_thresholds(struct qpnp_bms_chip *chip,
					int low_voltage_threshold,
					int high_voltage_threshold)
{
	uint16_t low_voltage_raw, high_voltage_raw;
	int rc;

	low_voltage_raw = convert_vbatt_uv_to_raw(chip,
				low_voltage_threshold);
	high_voltage_raw = convert_vbatt_uv_to_raw(chip,
				high_voltage_threshold);
	rc = qpnp_write_wrapper(chip, (u8 *)&low_voltage_raw,
			chip->base + BMS1_OCV_USE_LOW_LIMIT_THR0, 2);
	if (rc) {
		pr_err("Failed to set ocv low voltage threshold: %d\n", rc);
		return rc;
	}
	rc = qpnp_write_wrapper(chip, (u8 *)&high_voltage_raw,
			chip->base + BMS1_OCV_USE_HIGH_LIMIT_THR0, 2);
	if (rc) {
		pr_err("Failed to set ocv high voltage threshold: %d\n", rc);
		return rc;
	}
	rc = qpnp_masked_write(chip, BMS1_OCV_USE_LIMIT_CTL,
				OCV_USE_LIMIT_EN, OCV_USE_LIMIT_EN);
	if (rc) {
		pr_err("Failed to enabled ocv voltage thresholds: %d\n", rc);
		return rc;
	}
	pr_debug("ocv low threshold set to %d uv or 0x%x raw\n",
				low_voltage_threshold, low_voltage_raw);
	pr_debug("ocv high threshold set to %d uv or 0x%x raw\n",
				high_voltage_threshold, high_voltage_raw);
	return 0;
}

static int read_shutdown_iavg_ma(struct qpnp_bms_chip *chip)
{
	u8 iavg;
	int rc;

	rc = qpnp_read_wrapper(chip, &iavg, chip->base + IAVG_STORAGE_REG, 1);
	if (rc) {
		pr_err("failed to read addr = %d %d assuming %d\n",
				chip->base + IAVG_STORAGE_REG, rc,
				MIN_IAVG_MA);
		return MIN_IAVG_MA;
	} else if (iavg == IAVG_INVALID) {
		pr_err("invalid iavg read from BMS1_DATA_REG_1, using %d\n",
				MIN_IAVG_MA);
		return MIN_IAVG_MA;
	} else {
		if (iavg == 0)
			return MIN_IAVG_MA;
		else
			return MIN_IAVG_MA + IAVG_STEP_SIZE_MA * iavg;
	}
}

static int read_shutdown_soc(struct qpnp_bms_chip *chip)
{
	u8 stored_soc;
	int rc, shutdown_soc;

	rc = qpnp_read_wrapper(chip, &stored_soc, chip->soc_storage_addr, 1);
	if (rc) {
		pr_err("failed to read addr = %d %d\n",
				chip->soc_storage_addr, rc);
		return SOC_INVALID;
	}

	if ((stored_soc >> 1) > 0)
		shutdown_soc = (stored_soc >> 1) - 1;
	else
		shutdown_soc = SOC_INVALID;

	pr_debug("stored soc = 0x%02x, shutdown_soc = %d\n",
			stored_soc, shutdown_soc);
	return shutdown_soc;
}

#define BAT_REMOVED_OFFMODE_BIT		BIT(6)
static bool is_battery_replaced_in_offmode(struct qpnp_bms_chip *chip)
{
	u8 batt_pres;
	int rc;

	if (chip->batt_pres_addr) {
		rc = qpnp_read_wrapper(chip, &batt_pres,
				chip->batt_pres_addr, 1);
		pr_debug("offmode removed: %02x\n", batt_pres);
		if (!rc && (batt_pres & BAT_REMOVED_OFFMODE_BIT))
			return true;
	}
	return false;
}

static void load_shutdown_data(struct qpnp_bms_chip *chip)
{
	int calculated_soc, shutdown_soc;
	bool invalid_stored_soc;
	bool offmode_battery_replaced;
	bool shutdown_soc_out_of_limit;

	shutdown_soc = read_shutdown_soc(chip);
	bms_dbg.shutdown_soc = shutdown_soc;
	invalid_stored_soc = (shutdown_soc == SOC_INVALID);

	calculated_soc = recalculate_raw_soc(chip);
	shutdown_soc_out_of_limit = (abs(shutdown_soc - calculated_soc)
			> chip->shutdown_soc_valid_limit);
	pr_debug("calculated_soc = %d, valid_limit = %d\n",
			calculated_soc, chip->shutdown_soc_valid_limit);

	offmode_battery_replaced = is_battery_replaced_in_offmode(chip);

	
	if (chip->ignore_shutdown_soc
			|| invalid_stored_soc
			|| offmode_battery_replaced
			|| shutdown_soc_out_of_limit) {
		chip->battery_removed = true;
		chip->shutdown_soc_invalid = true;
		chip->shutdown_iavg_ma = 0;
		pr_info("Ignoring shutdown SoC: invalid = %d, offmode = %d, out_of_limit = %d\n",
				invalid_stored_soc, offmode_battery_replaced,
				shutdown_soc_out_of_limit);
	} else {
		chip->shutdown_iavg_ma = read_shutdown_iavg_ma(chip);
		chip->shutdown_soc = shutdown_soc;
	}

	pr_info("raw_soc=%d,shutdown_soc=%d,shutdown_iavg=%d,"
			"shutdown_soc_invalid=%d,battery_removed=%d\n",
			calculated_soc, chip->shutdown_soc,
			chip->shutdown_iavg_ma, chip->shutdown_soc_invalid,
			chip->battery_removed);
}

static irqreturn_t bms_ocv_thr_irq_handler(int irq, void *_chip)
{
	struct qpnp_bms_chip *chip = _chip;

	pr_info("[irq]ocv_thr irq triggered\n");
	bms_stay_awake(&chip->soc_wake_source);
	schedule_work(&chip->recalc_work);
	return IRQ_HANDLED;
}


static int64_t read_battery_id(struct qpnp_bms_chip *chip)
{
	int rc;
	struct qpnp_vadc_result result;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX2_BAT_ID, &result);
	if (rc) {
		pr_err("error reading batt id channel = %d, rc = %d\n",
					LR_MUX2_BAT_ID, rc);
		return rc;
	}

	return result.physical;
}

static int set_battery_data(struct qpnp_bms_chip *chip)
{
	int64_t battery_id;
	int rc = 0, dt_data = false;
	struct bms_battery_data *batt_data;
	struct device_node *node;
	int id_result;

	if (chip->batt_type == BATT_DESAY) {
		batt_data = &desay_5200_data;
	} else if (chip->batt_type == BATT_PALLADIUM) {
		batt_data = &palladium_1500_data;
	} else if (chip->batt_type == BATT_OEM) {
		batt_data = &oem_batt_data;
	} else if (chip->batt_type == BATT_QRD_4V35_2000MAH) {
		batt_data = &QRD_4v35_2000mAh_data;
	} else if (chip->batt_type == BATT_QRD_4V2_1300MAH) {
		batt_data = &qrd_4v2_1300mah_data;
	} else {
#if !(defined(CONFIG_HTC_BATT_8960))
		battery_id = read_battery_id(chip);
		if (battery_id < 0) {
			pr_err("cannot read battery id err = %lld\n",
							battery_id);
			return battery_id;
		}
#else
		
		battery_id = (int)read_battery_id(chip) / 1000;
		
		id_result = htc_battery_cell_find_and_set_id_auto(battery_id);
		pr_info("batt ID vol= %lldmv, id_result= %d\n", battery_id, id_result);
#endif

		node = of_find_node_by_name(chip->spmi->dev.of_node,
				"qcom,battery-data");
		if (!node) {
			pr_warn("No available batterydata, using palladium 1500\n");
			batt_data = &palladium_1500_data;
			goto assign_data;
		}
		batt_data = devm_kzalloc(chip->dev,
				sizeof(struct bms_battery_data), GFP_KERNEL);
		if (!batt_data) {
			pr_err("Could not alloc battery data\n");
			batt_data = &palladium_1500_data;
			goto assign_data;
		}
		batt_data->fcc_temp_lut = devm_kzalloc(chip->dev,
				sizeof(struct single_row_lut),
				GFP_KERNEL);
		batt_data->pc_temp_ocv_lut = devm_kzalloc(chip->dev,
				sizeof(struct pc_temp_ocv_lut),
				GFP_KERNEL);
		batt_data->rbatt_sf_lut = devm_kzalloc(chip->dev,
				sizeof(struct sf_lut),
				GFP_KERNEL);
		batt_data->rbatt_est_ocv_lut = devm_kzalloc(chip->dev,
				sizeof(struct sf_lut),
				GFP_KERNEL);

		batt_data->max_voltage_uv = -1;
		batt_data->cutoff_uv = -1;
		batt_data->iterm_ua = -1;

		if(bms_set_chgcyl_table(node, id_result))
		{
			pr_info("No charging cycle setting\n");
			chgcyl_checking_setting = 0;
		}else{
			pr_info("Charging cycle setting is done.\n");
			chgcyl_checking_setting = 1;
		}

#if !(defined(CONFIG_HTC_BATT_8960))
		rc = of_batterydata_read_data(node, batt_data, battery_id);
#else
		rc = of_batterydata_read_data_by_id_result(node, batt_data, id_result);
#endif
		if (rc == 0 && batt_data->fcc_temp_lut
				&& batt_data->pc_temp_ocv_lut
				&& batt_data->rbatt_sf_lut) {
			dt_data = true;
		} else {
			pr_err("battery data load failed, using palladium 1500\n");
			devm_kfree(chip->dev, batt_data->fcc_temp_lut);
			devm_kfree(chip->dev, batt_data->pc_temp_ocv_lut);
			devm_kfree(chip->dev, batt_data->rbatt_sf_lut);
			devm_kfree(chip->dev, batt_data);
			batt_data = &palladium_1500_data;
		}
	}

assign_data:
	chip->fcc_mah = batt_data->fcc;
	chip->fcc_temp_lut = batt_data->fcc_temp_lut;
	chip->fcc_sf_lut = batt_data->fcc_sf_lut;
	chip->pc_temp_ocv_lut = batt_data->pc_temp_ocv_lut;
	chip->pc_sf_lut = batt_data->pc_sf_lut;
	chip->rbatt_sf_lut = batt_data->rbatt_sf_lut;
	chip->default_rbatt_mohm = batt_data->default_rbatt_mohm;
	chip->rbatt_capacitive_mohm = batt_data->rbatt_capacitive_mohm;
	chip->flat_ocv_threshold_uv = batt_data->flat_ocv_threshold_uv;
	chip->rbatt_est_ocv_lut = batt_data->rbatt_est_ocv_lut;

	
	if (batt_data->max_voltage_uv >= 0 && dt_data)
		chip->max_voltage_uv = batt_data->max_voltage_uv;
	if (batt_data->cutoff_uv >= 0 && dt_data)
		chip->v_cutoff_uv = batt_data->cutoff_uv;
	if (batt_data->iterm_ua >= 0 && dt_data)
		chip->chg_term_ua = batt_data->iterm_ua;

	if (chip->pc_temp_ocv_lut == NULL) {
		pr_err("temp ocv lut table has not been loaded\n");
		if (dt_data) {
			devm_kfree(chip->dev, batt_data->fcc_temp_lut);
			devm_kfree(chip->dev, batt_data->pc_temp_ocv_lut);
			devm_kfree(chip->dev, batt_data->rbatt_sf_lut);
			devm_kfree(chip->dev, batt_data);
		}
		return -EINVAL;
	}

	if (dt_data)
		devm_kfree(chip->dev, batt_data);

	return 0;
}

static int bms_get_adc(struct qpnp_bms_chip *chip,
					struct spmi_device *spmi)
{
	int rc = 0;

	chip->vadc_dev = qpnp_get_vadc(&spmi->dev, "bms");
	if (IS_ERR(chip->vadc_dev)) {
		rc = PTR_ERR(chip->vadc_dev);
		if (rc != -EPROBE_DEFER)
			pr_err("vadc property missing, rc=%d\n", rc);
		return rc;
	}

	chip->iadc_dev = qpnp_get_iadc(&spmi->dev, "bms");
	if (IS_ERR(chip->iadc_dev)) {
		rc = PTR_ERR(chip->iadc_dev);
		if (rc != -EPROBE_DEFER)
			pr_err("iadc property missing, rc=%d\n", rc);
		return rc;
	}

	chip->adc_tm_dev = qpnp_get_adc_tm(&spmi->dev, "bms");
	if (IS_ERR(chip->adc_tm_dev)) {
		rc = PTR_ERR(chip->adc_tm_dev);
		if (rc != -EPROBE_DEFER)
			pr_err("adc-tm not ready, defer probe\n");
		return rc;
	}

	return 0;
}

#define SPMI_PROP_READ(chip_prop, qpnp_spmi_property, retval, optional)\
do {									\
	if (retval)							\
		break;							\
	retval = of_property_read_u32(chip->spmi->dev.of_node,		\
				"qcom," qpnp_spmi_property,		\
					&chip->chip_prop);		\
	if ((retval == -EINVAL) && optional)				\
		retval = 0;								\
	else if (retval) {							\
		pr_err("Error reading " #qpnp_spmi_property		\
						" property %d\n", rc);	\
	}								\
} while (0)

#define SPMI_PROP_READ_BOOL(chip_prop, qpnp_spmi_property)		\
do {									\
	chip->chip_prop = of_property_read_bool(chip->spmi->dev.of_node,\
				"qcom," qpnp_spmi_property);		\
} while (0)

static inline int bms_read_properties(struct qpnp_bms_chip *chip)
{
	int rc = 0;

	SPMI_PROP_READ(r_sense_uohm, "r-sense-uohm", rc, false);
	SPMI_PROP_READ(v_cutoff_uv, "v-cutoff-uv", rc, false);
	SPMI_PROP_READ(max_voltage_uv, "max-voltage-uv", rc, false);
	SPMI_PROP_READ(r_conn_mohm, "r-conn-mohm", rc, false);
	SPMI_PROP_READ(chg_term_ua, "chg-term-ua", rc, false);
	SPMI_PROP_READ(shutdown_soc_valid_limit,
			"shutdown-soc-valid-limit", rc, false);
	SPMI_PROP_READ(adjust_soc_low_threshold,
			"adjust-soc-low-threshold", rc, false);
	SPMI_PROP_READ(batt_type, "batt-type", rc, false);
	SPMI_PROP_READ(low_soc_calc_threshold,
			"low-soc-calculate-soc-threshold", rc, false);
	SPMI_PROP_READ(low_soc_calculate_soc_ms,
			"low-soc-calculate-soc-ms", rc, false);
	SPMI_PROP_READ(low_voltage_calculate_soc_ms,
			"low-voltage-calculate-soc-ms", rc, false);
	SPMI_PROP_READ(calculate_soc_ms, "calculate-soc-ms", rc, false);
	SPMI_PROP_READ(high_ocv_correction_limit_uv,
			"high-ocv-correction-limit-uv", rc, false);
	SPMI_PROP_READ(low_ocv_correction_limit_uv,
			"low-ocv-correction-limit-uv", rc, false);
	SPMI_PROP_READ(hold_soc_est,
			"hold-soc-est", rc, false);
	SPMI_PROP_READ(criteria_sw_est_ocv,
			"criteria-sw-est-ocv", rc, true);
	SPMI_PROP_READ(rconn_mohm_sw_est_ocv,
			"rconn-mohm-sw-est-ocv", rc, true);
	SPMI_PROP_READ(ocv_high_threshold_uv,
			"ocv-voltage-high-threshold-uv", rc, false);
	SPMI_PROP_READ(ocv_low_threshold_uv,
			"ocv-voltage-low-threshold-uv", rc, false);
	SPMI_PROP_READ(low_voltage_threshold, "low-voltage-threshold", rc, false);
	SPMI_PROP_READ(temperature_margin, "tm-temp-margin", rc, false);
	SPMI_PROP_READ(shutdown_vol_criteria, "shutdown-vol-criteria", rc, true);

	SPMI_PROP_READ(batt_stored_magic_num, "stored-batt-magic-num", rc, true);
	SPMI_PROP_READ(batt_stored_soc, "stored-batt-soc", rc, true);
	SPMI_PROP_READ(batt_stored_update_time, "stored-batt-update-time", rc, true);
	SPMI_PROP_READ(store_batt_data_soc_thre, "store-batt-data-soc-thre", rc, true);
	SPMI_PROP_READ(enable_sw_ocv_in_eoc, "enable-sw-ocv-in-eoc", rc, true);
	SPMI_PROP_READ(enable_batt_full_fake_ocv, "enable-batt-full-fake-ocv", rc, true);
	SPMI_PROP_READ(qb_mode_cc_criteria_uAh, "qb-mode-cc-criteria-uah", rc, true);
	SPMI_PROP_READ(soc_overrate_criteria_for_sw_ocv, "soc-overrate-criteria-for-sw-ocv", rc, true);

	chip->use_external_rsense = of_property_read_bool(
			chip->spmi->dev.of_node,
			"qcom,use-external-rsense");
	chip->ignore_shutdown_soc = of_property_read_bool(
			chip->spmi->dev.of_node,
			"qcom,ignore-shutdown-soc");
	chip->use_voltage_soc = of_property_read_bool(chip->spmi->dev.of_node,
			"qcom,use-voltage-soc");
	chip->use_ocv_thresholds = of_property_read_bool(
			chip->spmi->dev.of_node,
			"qcom,use-ocv-thresholds");

	if (chip->adjust_soc_low_threshold >= 45)
		chip->adjust_soc_low_threshold = 45;

	SPMI_PROP_READ_BOOL(enable_fcc_learning, "enable-fcc-learning");
	if (chip->enable_fcc_learning) {
		SPMI_PROP_READ(min_fcc_learning_soc,
				"min-fcc-learning-soc", rc, false);
		SPMI_PROP_READ(min_fcc_ocv_pc,
				"min-fcc-ocv-pc", rc, false);
		SPMI_PROP_READ(min_fcc_learning_samples,
				"min-fcc-learning-samples", rc, false);
		SPMI_PROP_READ(fcc_resolution,
				"fcc-resolution", rc, false);
               if (chip->min_fcc_learning_samples > MAX_FCC_CYCLES)
                       chip->min_fcc_learning_samples = MAX_FCC_CYCLES;
		chip->fcc_learning_samples = devm_kzalloc(&chip->spmi->dev,
				(sizeof(struct fcc_sample) *
				chip->min_fcc_learning_samples), GFP_KERNEL);
		if (chip->fcc_learning_samples == NULL)
			return -ENOMEM;
		pr_info("min-fcc-learning_soc=%d, min-fcc-ocv_pc=%d, min-fcc-learning_samples=%d, fcc-resolution=%d\n",
			chip->min_fcc_learning_soc, chip->min_fcc_ocv_pc,
			chip->min_fcc_learning_samples, chip->fcc_resolution);
	}

	if (rc) {
		pr_err("Missing required properties.\n");
		return rc;
	}

	pr_debug("dts data: r_sense_uohm:%d, v_cutoff_uv:%d, max_v:%d\n",
			chip->r_sense_uohm, chip->v_cutoff_uv,
			chip->max_voltage_uv);
	pr_debug("r_conn:%d, shutdown_soc: %d, adjust_soc_low:%d\n",
			chip->r_conn_mohm, chip->shutdown_soc_valid_limit,
			chip->adjust_soc_low_threshold);
	pr_debug("chg_term_ua:%d, batt_type:%d\n",
			chip->chg_term_ua,
			chip->batt_type);
	pr_debug("ignore_shutdown_soc:%d, use_voltage_soc:%d\n",
			chip->ignore_shutdown_soc, chip->use_voltage_soc);
	pr_debug("use external rsense: %d\n", chip->use_external_rsense);
	pr_info("magic_num:0x%X, stored_soc:%d, update_time:%u, store_batt_data_soc_thre:%d\n",
			chip->batt_stored_magic_num, chip->batt_stored_soc,
			chip->batt_stored_update_time, chip->store_batt_data_soc_thre);
	return 0;
}

static inline int bms_read_batt_stored_properties(struct qpnp_bms_chip *chip)
{
	int rc = 0;

	SPMI_PROP_READ(batt_stored_ocv_uv, "stored-batt-ocv-uv", rc, true);
	SPMI_PROP_READ(batt_stored_cc_uah, "stored-batt-cc-uah", rc, true);

	pr_info("stored_ocv_uv:%d, stored_cc_uah:%d\n",
		chip->batt_stored_ocv_uv, chip->batt_stored_cc_uah);
	return 0;
}

static inline void bms_initialize_constants(struct qpnp_bms_chip *chip)
{
	chip->prev_pc_unusable = -EINVAL;
	chip->soc_at_cv = -EINVAL;
	chip->calculated_soc = -EINVAL;
	chip->last_soc = -EINVAL;
	chip->last_soc_est = -EINVAL;
	chip->battery_present = -EINVAL;
	chip->battery_status = POWER_SUPPLY_STATUS_UNKNOWN;
	chip->last_cc_uah = INT_MIN;
	chip->ocv_reading_at_100 = OCV_RAW_UNINITIALIZED;
	chip->prev_last_good_ocv_raw = OCV_RAW_UNINITIALIZED;
	chip->first_time_calc_soc = 1;
	chip->first_time_calc_uuc = 1;
	chip->batt_stored_ocv_uv = 0;
	chip->batt_stored_cc_uah = 0;
	chip->cc_backup_uah = 0;
	chip->ocv_backup_uv = 0;
}

#define SPMI_FIND_IRQ(chip, irq_name)					\
do {									\
	chip->irq_name##_irq.irq = spmi_get_irq_byname(chip->spmi,	\
					resource, #irq_name);		\
	if (chip->irq_name##_irq.irq < 0) {				\
		pr_err("Unable to get " #irq_name " irq\n");		\
		return -ENXIO;						\
	}								\
} while (0)

static int bms_find_irqs(struct qpnp_bms_chip *chip,
			struct spmi_resource *resource)
{
	SPMI_FIND_IRQ(chip, cc_thr);
	SPMI_FIND_IRQ(chip, ocv_for_r);
	SPMI_FIND_IRQ(chip, good_ocv);
	SPMI_FIND_IRQ(chip, charge_begin);
	SPMI_FIND_IRQ(chip, sw_cc_thr);
	SPMI_FIND_IRQ(chip, ocv_thr);
	SPMI_FIND_IRQ(chip, vsense_avg);
	SPMI_FIND_IRQ(chip, vsense_for_r);
	return 0;
}

#define SPMI_REQUEST_IRQ(chip, rc, irq_name)				\
do {									\
	rc = devm_request_irq(chip->dev, chip->irq_name##_irq.irq,	\
			bms_##irq_name##_irq_handler,			\
			IRQF_TRIGGER_RISING, #irq_name, chip);		\
	if (rc < 0) {							\
		pr_err("Unable to request " #irq_name " irq: %d\n", rc);\
		return -ENXIO;						\
	}								\
} while (0)

static int bms_request_irqs(struct qpnp_bms_chip *chip)
{
	int rc;

	SPMI_REQUEST_IRQ(chip, rc, ocv_thr);
	enable_irq_wake(chip->ocv_thr_irq.irq);
	return 0;
}

#define REG_OFFSET_PERP_TYPE			0x04
#define REG_OFFSET_PERP_SUBTYPE			0x05
#define BMS_BMS_TYPE				0xD
#define BMS_BMS1_SUBTYPE			0x1
#define BMS_IADC_TYPE				0x8
#define BMS_IADC1_SUBTYPE			0x3
#define BMS_IADC2_SUBTYPE			0x5

static int register_spmi(struct qpnp_bms_chip *chip, struct spmi_device *spmi)
{
	struct spmi_resource *spmi_resource;
	struct resource *resource;
	int rc;
	u8 type, subtype;

	chip->dev = &(spmi->dev);
	chip->spmi = spmi;

	spmi_for_each_container_dev(spmi_resource, spmi) {
		if (!spmi_resource) {
			pr_err("qpnp_bms: spmi resource absent\n");
			return -ENXIO;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			pr_err("node %s IO resource absent!\n",
				spmi->dev.of_node->full_name);
			return -ENXIO;
		}

		pr_debug("Node name = %s\n", spmi_resource->of_node->name);

		if (strcmp("qcom,batt-pres-status",
					spmi_resource->of_node->name) == 0) {
			chip->batt_pres_addr = resource->start;
			continue;
		} else if (strcmp("qcom,soc-storage-reg",
					spmi_resource->of_node->name) == 0) {
			chip->soc_storage_addr = resource->start;
			continue;
		}

		rc = qpnp_read_wrapper(chip, &type,
				resource->start + REG_OFFSET_PERP_TYPE, 1);
		if (rc) {
			pr_err("Peripheral type read failed rc=%d\n", rc);
			return rc;
		}
		rc = qpnp_read_wrapper(chip, &subtype,
				resource->start + REG_OFFSET_PERP_SUBTYPE, 1);
		if (rc) {
			pr_err("Peripheral subtype read failed rc=%d\n", rc);
			return rc;
		}

		if (type == BMS_BMS_TYPE && subtype == BMS_BMS1_SUBTYPE) {
			chip->base = resource->start;
			rc = bms_find_irqs(chip, spmi_resource);
			if (rc) {
				pr_err("Could not find irqs\n");
				return rc;
			}
		} else if (type == BMS_IADC_TYPE
				&& (subtype == BMS_IADC1_SUBTYPE
				|| subtype == BMS_IADC2_SUBTYPE)) {
			chip->iadc_base = resource->start;
		} else {
			pr_warn("Invalid peripheral start=0x%x type=0x%x, subtype=0x%x\n",
					resource->start, type, subtype);
		}
	}

	if (chip->base == 0) {
		dev_err(&spmi->dev, "BMS peripheral was not registered\n");
		return -EINVAL;
	}
	if (chip->iadc_base == 0) {
		dev_err(&spmi->dev, "BMS_IADC peripheral was not registered\n");
		return -EINVAL;
	}
	if (chip->soc_storage_addr == 0) {
		
		chip->soc_storage_addr = chip->base + SOC_STORAGE_REG;
	}

	pr_debug("bms-base = 0x%04x, iadc-base = 0x%04x, bat-pres-reg = 0x%04x, soc-storage-reg = 0x%04x\n",
			chip->base, chip->iadc_base,
			chip->batt_pres_addr, chip->soc_storage_addr);
	return 0;
}

#define ADC_CH_SEL_MASK				0x7
#define ADC_INT_RSNSN_CTL_MASK			0x3
#define ADC_INT_RSNSN_CTL_VALUE_EXT_RENSE	0x2
#define FAST_AVG_EN_MASK			0x80
#define FAST_AVG_EN_VALUE_EXT_RSENSE		0x80
static int read_iadc_channel_select(struct qpnp_bms_chip *chip)
{
	u8 iadc_channel_select;
	int32_t rds_rsense_nohm;
	int rc;

	rc = qpnp_read_wrapper(chip, &iadc_channel_select,
			chip->iadc_base + IADC1_BMS_ADC_CH_SEL_CTL, 1);
	if (rc) {
		pr_err("Error reading bms_iadc channel register %d\n", rc);
		return rc;
	}

	iadc_channel_select &= ADC_CH_SEL_MASK;
	if (iadc_channel_select != EXTERNAL_RSENSE
			&& iadc_channel_select != INTERNAL_RSENSE) {
		pr_err("IADC1_BMS_IADC configured incorrectly. Selected channel = %d\n",
						iadc_channel_select);
		return -EINVAL;
	}

	if (chip->use_external_rsense) {
		pr_debug("External rsense selected\n");
		if (iadc_channel_select == INTERNAL_RSENSE) {
			pr_info("Internal rsense detected; Changing rsense to external\n");
			rc = qpnp_masked_write_iadc(chip,
					IADC1_BMS_ADC_CH_SEL_CTL,
					ADC_CH_SEL_MASK,
					EXTERNAL_RSENSE);
			if (rc) {
				pr_err("Unable to set IADC1_BMS channel %x to %x: %d\n",
						IADC1_BMS_ADC_CH_SEL_CTL,
						EXTERNAL_RSENSE, rc);
				return rc;
			}
			reset_cc(chip, CLEAR_CC | CLEAR_SHDW_CC);
			chip->software_cc_uah = 0;
			chip->software_shdw_cc_uah = 0;
		}
	} else {
		pr_debug("Internal rsense selected\n");
		if (iadc_channel_select == EXTERNAL_RSENSE) {
			pr_info("External rsense detected; Changing rsense to internal\n");
			rc = qpnp_masked_write_iadc(chip,
					IADC1_BMS_ADC_CH_SEL_CTL,
					ADC_CH_SEL_MASK,
					INTERNAL_RSENSE);
			if (rc) {
				pr_err("Unable to set IADC1_BMS channel %x to %x: %d\n",
						IADC1_BMS_ADC_CH_SEL_CTL,
						INTERNAL_RSENSE, rc);
				return rc;
			}
			reset_cc(chip, CLEAR_CC | CLEAR_SHDW_CC);
			chip->software_shdw_cc_uah = 0;
		}

		rc = qpnp_iadc_get_rsense(chip->iadc_dev, &rds_rsense_nohm);
		if (rc) {
			pr_err("Unable to read RDS resistance value from IADC; rc = %d\n",
								rc);
			return rc;
		}
		chip->r_sense_uohm = rds_rsense_nohm/1000;
		pr_debug("rds_rsense = %d nOhm, saved as %d uOhm\n",
					rds_rsense_nohm, chip->r_sense_uohm);
	}
	
	if (chip->use_external_rsense) {
		if (chip->iadc_bms_revision2 > CALIB_WRKARND_DIG_MAJOR_MAX) {
			rc = qpnp_masked_write_iadc(chip,
					IADC1_BMS_ADC_INT_RSNSN_CTL,
					ADC_INT_RSNSN_CTL_MASK,
					ADC_INT_RSNSN_CTL_VALUE_EXT_RENSE);
			if (rc) {
				pr_err("Unable to set batfet config %x to %x: %d\n",
					IADC1_BMS_ADC_INT_RSNSN_CTL,
					ADC_INT_RSNSN_CTL_VALUE_EXT_RENSE, rc);
				return rc;
			}
		} else {
			
			rc = qpnp_masked_write_iadc(chip,
					IADC1_BMS_FAST_AVG_EN,
					FAST_AVG_EN_MASK,
					FAST_AVG_EN_VALUE_EXT_RSENSE);
			if (rc) {
				pr_err("Unable to set batfet config %x to %x: %d\n",
					IADC1_BMS_FAST_AVG_EN,
					FAST_AVG_EN_VALUE_EXT_RSENSE, rc);
				return rc;
			}
		}
	}

	return 0;
}

#if !(defined(CONFIG_HTC_BATT_8960))
static int refresh_die_temp_monitor(struct qpnp_bms_chip *chip)
{
	struct qpnp_vadc_result result;
	int rc;

	rc = qpnp_vadc_read(chip->vadc_dev, DIE_TEMP, &result);

	pr_debug("low = %lld, high = %lld\n",
			result.physical - chip->temperature_margin,
			result.physical + chip->temperature_margin);
	chip->die_temp_monitor_params.high_temp = result.physical
						+ chip->temperature_margin;
	chip->die_temp_monitor_params.low_temp = result.physical
						- chip->temperature_margin;
	chip->die_temp_monitor_params.state_request =
						ADC_TM_HIGH_LOW_THR_ENABLE;
	return qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
					&chip->die_temp_monitor_params);
}

static void btm_notify_die_temp(enum qpnp_tm_state state, void *ctx)
{
	struct qpnp_bms_chip *chip = ctx;
	struct qpnp_vadc_result result;
	int rc;

	rc = qpnp_vadc_read(chip->vadc_dev, DIE_TEMP, &result);

	if (state == ADC_TM_LOW_STATE)
		pr_debug("low state triggered\n");
	else if (state == ADC_TM_HIGH_STATE)
		pr_debug("high state triggered\n");
	pr_debug("die temp = %lld, raw = 0x%x\n",
			result.physical, result.adc_code);
	schedule_work(&chip->recalc_work);
	refresh_die_temp_monitor(chip);
}

static int setup_die_temp_monitoring(struct qpnp_bms_chip *chip)
{
	int rc;

	chip->die_temp_monitor_params.channel = DIE_TEMP;
	chip->die_temp_monitor_params.btm_ctx = (void *)chip;
	chip->die_temp_monitor_params.timer_interval = ADC_MEAS1_INTERVAL_1S;
	chip->die_temp_monitor_params.threshold_notification =
						&btm_notify_die_temp;
	rc = refresh_die_temp_monitor(chip);
	if (rc) {
		pr_err("tm setup failed: %d\n", rc);
		return rc;
	}
	pr_debug("setup complete\n");
	return 0;
}
#endif

static int __devinit qpnp_bms_probe(struct spmi_device *spmi)
{
	struct qpnp_bms_chip *chip;
	bool warm_reset;
	int rc, vbatt, curr_soc, batt_temp;
	struct timespec xtime;
	unsigned long currtime_ms;
	struct raw_soc_params raw;
	struct soc_params params;

	chip = devm_kzalloc(&spmi->dev, sizeof(struct qpnp_bms_chip),
			GFP_KERNEL);

	if (chip == NULL) {
		pr_err("kzalloc() failed.\n");
		return -ENOMEM;
	}

	rc = bms_get_adc(chip, spmi);
	if (rc < 0)
		goto error_read;

	mutex_init(&chip->bms_output_lock);
	mutex_init(&chip->last_ocv_uv_mutex);
	mutex_init(&chip->vbat_monitor_mutex);
	mutex_init(&chip->soc_invalidation_mutex);
	mutex_init(&chip->last_soc_mutex);
	mutex_init(&chip->status_lock);
	mutex_init(&ocv_update_lock);
	init_waitqueue_head(&chip->bms_wait_queue);

	warm_reset = qpnp_pon_is_warm_reset();
	rc = warm_reset;
	if (rc < 0)
		goto error_read;

	rc = register_spmi(chip, spmi);
	if (rc) {
		pr_err("error registering spmi resource %d\n", rc);
		goto error_resource;
	}

	rc = qpnp_read_wrapper(chip, &chip->revision1,
			chip->base + REVISION1, 1);
	if (rc) {
		pr_err("error reading version register %d\n", rc);
		goto error_read;
	}

	rc = qpnp_read_wrapper(chip, &chip->revision2,
			chip->base + REVISION2, 1);
	if (rc) {
		pr_err("Error reading version register %d\n", rc);
		goto error_read;
	}
	pr_debug("BMS version: %hhu.%hhu\n", chip->revision2, chip->revision1);

	rc = qpnp_read_wrapper(chip, &chip->iadc_bms_revision2,
			chip->iadc_base + REVISION2, 1);
	if (rc) {
		pr_err("Error reading version register %d\n", rc);
		goto error_read;
	}

	rc = qpnp_read_wrapper(chip, &chip->iadc_bms_revision1,
			chip->iadc_base + REVISION1, 1);
	if (rc) {
		pr_err("Error reading version register %d\n", rc);
		goto error_read;
	}
	pr_debug("IADC_BMS version: %hhu.%hhu\n",
			chip->iadc_bms_revision2, chip->iadc_bms_revision1);

	rc = bms_read_properties(chip);
	if (rc) {
		pr_err("Unable to read all bms properties, rc = %d\n", rc);
		goto error_read;
	}

	rc = read_iadc_channel_select(chip);
	if (rc) {
		pr_err("Unable to get iadc selected channel = %d\n", rc);
		goto error_read;
	}

	if (chip->use_ocv_thresholds) {
		rc = set_ocv_voltage_thresholds(chip,
				chip->ocv_low_threshold_uv,
				chip->ocv_high_threshold_uv);
		if (rc) {
			pr_err("Could not set ocv voltage thresholds: %d\n",
					rc);
			goto error_read;
		}
	}

	rc = set_battery_data(chip);
	if (rc) {
		pr_err("Bad battery data %d\n", rc);
		goto error_read;
	}

	bms_initialize_constants(chip);

	wakeup_source_init(&chip->soc_wake_source.source, "qpnp_soc_wake");
#if !(defined(CONFIG_HTC_BATT_8960))
	wake_lock_init(&chip->low_voltage_wake_lock, WAKE_LOCK_SUSPEND,
			"qpnp_low_voltage_lock");
	wake_lock_init(&chip->cv_wake_lock, WAKE_LOCK_SUSPEND,
			"qpnp_cv_lock");
#endif
	INIT_DELAYED_WORK(&chip->calculate_soc_delayed_work,
			calculate_soc_work);
	INIT_WORK(&chip->recalc_work, recalculate_work);
	INIT_WORK(&chip->batfet_open_work, batfet_open_work);

	dev_set_drvdata(&spmi->dev, chip);
	device_init_wakeup(&spmi->dev, 1);

	load_shutdown_data(chip);

	if (chip->criteria_sw_est_ocv)
		chip->criteria_sw_est_ocv = FIRST_SW_EST_OCV_THR_MS;

	if (chip->enable_fcc_learning) {
			pr_info("Re-store the FCC data!\n");
			rc = read_chgcycle_data_from_backup(chip);
			if (rc)
				pr_err("Unable to restore charge-cycle data\n");

			rc = read_fcc_data_from_backup(chip);
			if (rc)
				pr_err("Unable to restore FCC-learning data\n");
			else
				attempt_learning_new_fcc(chip);
	}

#if !(defined(CONFIG_HTC_BATT_8960))
	rc = setup_vbat_monitoring(chip);
	if (rc < 0) {
		pr_err("failed to set up voltage notifications: %d\n", rc);
		goto error_setup;
	}

	rc = setup_die_temp_monitoring(chip);
	if (rc < 0) {
		pr_err("failed to set up die temp notifications: %d\n", rc);
		goto error_setup;
	}
#endif

	battery_insertion_check(chip);
	batfet_status_check(chip);
	battery_status_check(chip);

	the_chip = chip;

	htc_batt_bms_timer.batt_system_jiffies = jiffies;
	
	rc = pm8941_bms_start_ocv_updates();
	if (rc) {
		pr_err("failed to enable HW OCV measurement: %d\n", rc);
		goto error_setup;
	}

	curr_soc = pm8941_bms_get_percent_charge(chip);
	rc = pm8941_get_batt_temperature(&batt_temp);
	if (rc)
		pr_err("get temperature failed err = %d\n", rc);

	xtime = CURRENT_TIME;
	currtime_ms = xtime.tv_sec * MSEC_PER_SEC + xtime.tv_nsec / NSEC_PER_MSEC;

	
	if (chip->batt_stored_magic_num == BMS_STORE_MAGIC_NUM
			&& chip->store_batt_data_soc_thre > 0 && batt_temp >= 0
			&& (curr_soc > chip->batt_stored_soc || chip->batt_stored_soc - curr_soc > 1)
			&& (currtime_ms - chip->batt_stored_update_time) < 3600000 ) {

		rc = bms_read_batt_stored_properties(chip);
		if (rc) {
			pr_err("Unable to read all bms properties, rc = %d\n", rc);
			goto error_read;
		}

		read_soc_params_raw(chip, &raw, batt_temp);
		calculate_soc_params(chip, &raw, &params, batt_temp);
		chip->ocv_backup_uv = chip->last_ocv_uv = chip->batt_stored_ocv_uv;
		chip->cc_backup_uah = bms_dbg.ori_cc_uah - chip->batt_stored_cc_uah;

		new_boot_soc = pm8941_bms_get_percent_charge(chip);
		
		disable_ocv_update_with_reason(true, OCV_UPDATE_STOP_BIT_BOOT_UP);
		
		allow_ocv_time = currtime_ms + 3600000;
		consistent_flag = true;
	}

	

	
	chip->bms_psy.name = "bms";
	chip->bms_psy.type = POWER_SUPPLY_TYPE_BMS;
	chip->bms_psy.properties = msm_bms_power_props;
	chip->bms_psy.num_properties = ARRAY_SIZE(msm_bms_power_props);
	chip->bms_psy.get_property = qpnp_bms_power_get_property;
	chip->bms_psy.external_power_changed =
		qpnp_bms_external_power_changed;
	chip->bms_psy.supplied_to = qpnp_bms_supplicants;
	chip->bms_psy.num_supplicants = ARRAY_SIZE(qpnp_bms_supplicants);

	rc = power_supply_register(chip->dev, &chip->bms_psy);

	if (rc < 0) {
		pr_err("power_supply_register bms failed rc = %d\n", rc);
		goto unregister_dc;
	}

	chip->bms_psy_registered = true;
	vbatt = 0;
	rc = get_battery_voltage(chip, &vbatt);
	if (rc) {
		pr_err("error reading vbat_sns adc channel = %d, rc = %d\n",
						VBAT_SNS, rc);
		goto unregister_dc;
	}

	rc = bms_request_irqs(chip);
	if (rc) {
		pr_err("error requesting bms irqs, rc = %d\n", rc);
		goto unregister_dc;
	}

	htc_gauge_event_notify(HTC_GAUGE_EVENT_READY);

	
	pm8941_fake_chg_gone_irq_handler();

	
	pm8941_fake_usbin_valid_irq_handler();

	
	pm8941_fake_coarse_det_usb_irq_handler();

	pr_info("curr_soc=%d,new_boot_soc:%d,stored_soc:%d,vbatt=%d,OCV=%d,r_sense_uohm=%u,"
			"warm_reset=%d,raw.cc:%lld,stored_cc:%d,cc_backup:%d,stored_ocv:%d,"
			"boot_currtime_ms:%lu,allow_ocv_time:%lu,stored_time:%u,batt_temp:%d\n",
			curr_soc, new_boot_soc, chip->batt_stored_soc, vbatt, chip->last_ocv_uv, chip->r_sense_uohm,
			warm_reset, raw.cc, chip->batt_stored_cc_uah, chip->cc_backup_uah, chip->batt_stored_ocv_uv,
			currtime_ms, allow_ocv_time, chip->batt_stored_update_time, batt_temp);
	return 0;

unregister_dc:
	if (chip->bms_psy_registered) {
		power_supply_unregister(&chip->bms_psy);
		chip->bms_psy_registered = false;
	}
error_setup:
	dev_set_drvdata(&spmi->dev, NULL);
	wakeup_source_trash(&chip->soc_wake_source.source);
#if !(defined(CONFIG_HTC_BATT_8960))
	wake_lock_destroy(&chip->low_voltage_wake_lock);
	wake_lock_destroy(&chip->cv_wake_lock);
#endif
error_resource:
error_read:
	return rc;
}

int pm8941_check_soc_for_sw_ocv(void)
{
	int is_full_eoc = 0;

	if (!the_chip) {
		pr_warn("called before init\n");
		return -EINVAL;
	}

	if (!the_chip->enable_sw_ocv_in_eoc)
		return 0;

	pm8941_is_batt_full_eoc_stop(&is_full_eoc);
	if (is_full_eoc) {
		pr_info("is_full_eoc=%d, store_soc_ui=%d, raw_soc=%d\n",
					is_full_eoc, store_soc_ui, bms_dbg.raw_soc);
		if (abs(store_soc_ui - bms_dbg.raw_soc) >= 5) {
			is_do_sw_ocv_in_eoc = 1;
			pm8941_bms_estimate_ocv();
		}
	}
	return 0;
}

static int qpnp_bms_remove(struct spmi_device *spmi)
{
	dev_set_drvdata(&spmi->dev, NULL);
	the_chip = NULL;
	return 0;
}

static int bms_suspend(struct device *dev)
{
	struct qpnp_bms_chip *chip = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&chip->calculate_soc_delayed_work);
	chip->was_charging_at_sleep = is_battery_charging(chip);
	return 0;
}

static int bms_resume(struct device *dev)
{
	int rc;
	int soc_calc_period;
	int time_until_next_recalc = 0;
	unsigned long time_since_last_recalc;
	unsigned long tm_now_sec;
	struct qpnp_bms_chip *chip = dev_get_drvdata(dev);

	rc = get_current_time(&tm_now_sec);
	if (rc) {
		pr_err("Could not read current time: %d\n", rc);
	} else {
		soc_calc_period = get_calculation_delay_ms(chip);
		time_since_last_recalc = tm_now_sec - chip->last_recalc_time;
		pr_debug("Time since last recalc: %lu\n",
				time_since_last_recalc);
		time_until_next_recalc = max(0, soc_calc_period
				- (int)(time_since_last_recalc * 1000));
	}

	if (time_until_next_recalc == 0)
		bms_stay_awake(&chip->soc_wake_source);
	schedule_delayed_work(&chip->calculate_soc_delayed_work,
		round_jiffies_relative(msecs_to_jiffies
		(time_until_next_recalc)));
	return 0;
}

static int bms_prepare(struct device *dev)
{
	unsigned long time_since_last_update_ms, cur_jiffies;
	struct timespec xtime;

	if (the_chip->criteria_sw_est_ocv <= 0)
		return 0;

	cur_jiffies = jiffies;
	time_since_last_update_ms =
		(cur_jiffies - htc_batt_bms_timer.batt_system_jiffies) * MSEC_PER_SEC / HZ;
	htc_batt_bms_timer.no_ocv_update_period_ms += time_since_last_update_ms;
	htc_batt_bms_timer.batt_system_jiffies = cur_jiffies;
	xtime = CURRENT_TIME;
	htc_batt_bms_timer.batt_suspend_ms = xtime.tv_sec * MSEC_PER_SEC +
		xtime.tv_nsec / NSEC_PER_MSEC;

	return 0;
}

static void bms_complete(struct device *dev)
{
	struct timespec xtime;
	unsigned long resume_ms, sr_time_period_ms;

	if (the_chip->criteria_sw_est_ocv <= 0)
		return;

	xtime = CURRENT_TIME;
	htc_batt_bms_timer.batt_system_jiffies = jiffies;
	resume_ms = xtime.tv_sec * MSEC_PER_SEC + xtime.tv_nsec / NSEC_PER_MSEC;
	sr_time_period_ms = resume_ms - htc_batt_bms_timer.batt_suspend_ms;
	htc_batt_bms_timer.no_ocv_update_period_ms += sr_time_period_ms;

	if (htc_batt_bms_timer.no_ocv_update_period_ms > the_chip->criteria_sw_est_ocv
		&& batt_level > DISABLE_SW_OCV_LEVEL_THRESHOLD
		&& !(!!sw_ocv_estimated_stop_reason))
		pm8941_bms_estimate_ocv();
}

static const struct dev_pm_ops qpnp_bms_pm_ops = {
	.prepare		= bms_prepare,
	.complete		= bms_complete,
	.resume		= bms_resume,
	.suspend	= bms_suspend,
};

static struct spmi_driver qpnp_bms_driver = {
	.probe		= qpnp_bms_probe,
	.remove		= __devexit_p(qpnp_bms_remove),
	.driver		= {
		.name		= QPNP_BMS_DEV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= qpnp_bms_match_table,
		.pm		= &qpnp_bms_pm_ops,
	},
};

static int __init qpnp_bms_init(void)
{
	pr_info("QPNP BMS INIT\n");
	flag_enable_bms_charger_log =
               (get_kernel_flag() & KERNEL_FLAG_ENABLE_BMS_CHARGER_LOG) ? 1 : 0;
	return spmi_driver_register(&qpnp_bms_driver);
}

static void __exit qpnp_bms_exit(void)
{
	pr_info("QPNP BMS EXIT\n");
	return spmi_driver_unregister(&qpnp_bms_driver);
}

module_init(qpnp_bms_init);
module_exit(qpnp_bms_exit);

MODULE_DESCRIPTION("QPNP BMS Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" QPNP_BMS_DEV_NAME);
