// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 MediaTek Inc.
 */

#ifndef __MTK_SOC_TEMP_LVTS_H__
#define __MTK_SOC_TEMP_LVTS_H__

#define PERIOD_UNIT				12
#define GROUP_INTERVAL_DELAY	1
#define FILTER_INTERVAL_DELAY	1
#define SENSOR_INTERVAL_DELAY	1

#define HW_REBOOT_TRIP_POINT	117000

#define FEATURE_DEVICE_AUTO_RCK	BIT(0)
#define NUM_EFUSE_ADDR			22
#define NUM_EFUSE_ADDR_MT8188		16
#define NUM_EFUSE_BLOCK_MT8188	1
#define NUM_EFUSE_BLOCK_MT8192	1
#define DEFAULT_GOLDEN_TEMP		50
#define DEFAULT_CUONT_R			35000
#define DEFAULT_CUONT_RC		2750
#define COEFF_A					-250460
#define COEFF_B					250460

#define CLOCK_26MHZ_CYCLE_NS	38
#define BUS_ACCESS_US			2
#define GOLDEN_TEMP_MAX			62

/* LVTS device register */
#define RG_TSFM_DATA_0			0x00
#define RG_TSFM_DATA_1			0x01
#define RG_TSFM_DATA_2			0x02
#define RG_TSFM_CTRL_0			0x03
#define RG_TSFM_CTRL_1			0x04
#define RG_TSFM_CTRL_2			0x05
#define RG_TSFM_CTRL_3			0x06
#define RG_TSFM_CTRL_4			0x07
#define RG_TSV2F_CTRL_0			0x08
#define RG_TSV2F_CTRL_1			0x09
#define RG_TSV2F_CTRL_2			0x0A
#define RG_TSV2F_CTRL_3			0x0B
#define RG_TSV2F_CTRL_4			0x0C
#define RG_TSV2F_CTRL_5			0x0D
#define RG_TSV2F_CTRL_6			0x0E
#define RG_TEMP_DATA_0			0x10
#define RG_TEMP_DATA_1			0x11
#define RG_TEMP_DATA_2			0x12
#define RG_TEMP_DATA_3			0x13
#define RG_RC_DATA_0			0x14
#define RG_RC_DATA_1			0x15
#define RG_RC_DATA_2			0x16
#define RG_RC_DATA_3			0x17
#define RG_DIV_DATA_0			0x18
#define RG_DIV_DATA_1			0x19
#define RG_DIV_DATA_2			0x1A
#define RG_DIV_DATA_3			0x1B
#define RG_TST_DATA_0			0x70
#define RG_TST_DATA_1			0x71
#define RG_TST_DATA_2			0x72
#define RG_TST_CTRL				0x73
#define RG_DBG_FQMTR			0xF0
#define RG_DBG_LPSEQ			0xF1
#define RG_DBG_STATE			0xF2
#define RG_DBG_CHKSUM			0xF3
#define RG_DID_LVTS				0xFC
#define RG_DID_REV				0xFD
#define RG_TSFM_RST				0xFF

/* LVTS controller register */
#define LVTSMONCTL0_0				0x000
#define ENABLE_SENSING_POINT(num)	(LVTS_SINGLE_SENSE | GENMASK(((num) - 1), 0))
#define DISABLE_SENSING_POINT		(LVTS_SINGLE_SENSE | 0x0)
#define LVTSMONCTL1_0				0x004
#define LVTSMONCTL2_0				0x008
#define LVTSMONINT_0				0x00C
#define STAGE3_INT_EN				BIT(31)
#define LVTSMONINTSTS_0				0x010
#define LVTSMONIDET0_0				0x014
#define LVTSMONIDET1_0				0x018
#define LVTSMONIDET2_0				0x01C
#define LVTSMONIDET3_0				0x020
#define LVTSH2NTHRE_0				0x024
#define LVTSHTHRE_0					0x028
#define LVTSCTHRE_0					0x02C
#define LVTSOFFSETH_0				0x030
#define LVTSOFFSETL_0				0x034
#define LVTSMSRCTL0_0				0x038
#define LVTSMSRCTL1_0				0x03C
#define LVTSTSSEL_0					0x040
#define SET_SENSOR_INDEX			0x13121110
#define LVTSDEVICETO_0				0x044
#define LVTSCALSCALE_0				0x048
#define SET_CALC_SCALE_RULES		0x00000300
#define LVTS_ID_0					0x04C
#define LVTS_CONFIG_0				0x050

#define SCK_ONLY					BIT(31)
#define BROADCAST_ID_UPDATE			BIT(26)
#define DEVICE_SENSING_STATUS		BIT(25)
#define DEVICE_ACCESS_STARTUS		BIT(24)
#define READ_32BIT_ACCESS			BIT(17)
#define WRITE_ACCESS				BIT(16)
#define LVTS_SINGLE_SENSE			BIT(9)
#define FEATURE_CK26M_ACTIVE		BIT(1)
#define DEVICE_REG_DATA				GENMASK(7, 0)

#define LVTSEDATA00_0				0x054
#define LVTSEDATA01_0				0x058
#define LVTSEDATA02_0				0x05C
#define LVTSEDATA03_0				0x060
#define LVTSMSR0_0					0x090
#define MRS_RAW_MASK				GENMASK(15, 0)
#define MRS_RAW_VALID_BIT			BIT(16)
#define LVTSMSR1_0					0x094
#define LVTSMSR2_0					0x098
#define LVTSMSR3_0					0x09C
#define LVTSIMMD0_0					0x0A0
#define LVTSIMMD1_0					0x0A4
#define LVTSIMMD2_0					0x0A8
#define LVTSIMMD3_0					0x0AC
#define LVTSRDATA0_0				0x0B0
#define LVTSRDATA1_0				0x0B4
#define LVTSRDATA2_0				0x0B8
#define LVTSRDATA3_0				0x0BC
#define LVTSPROTCTL_0				0x0C0
#define PROTOFFSET					GENMASK(15, 0)
#define LVTSPROTTA_0				0x0C4
#define LVTSPROTTB_0				0x0C8
#define LVTSPROTTC_0				0x0CC
#define LVTSCLKEN_0					0x0E4
#define ENABLE_LVTS_CTRL_CLK		(1)
#define DISABLE_LVTS_CTRL_CLK		(0)
#define LVTSDBGSEL_0				0x0E8
#define LVTSDBGSIG_0				0x0EC
#define LVTSSPARE0_0				0x0F0
#define LVTSSPARE1_0				0x0F4
#define LVTSSPARE2_0				0x0F8
#define LVTSSPARE3_0				0x0FC
#define THERMINTST					0xF04

/* LVTS register mask */
#define THERMAL_COLD_INTERRUPT_0			BIT(0)
#define THERMAL_HOT_INTERRUPT_0				BIT(1)
#define THERMAL_LOW_OFFSET_INTERRUPT_0		BIT(2)
#define THERMAL_HIGH_OFFSET_INTERRUPT_0		BIT(3)
#define THERMAL_HOT2NORMAL_INTERRUPT_0		BIT(4)
#define THERMAL_COLD_INTERRUPT_1			BIT(5)
#define THERMAL_HOT_INTERRUPT_1				BIT(6)
#define THERMAL_LOW_OFFSET_INTERRUPT_1		BIT(7)
#define THERMAL_HIGH_OFFSET_INTERRUPT_1		BIT(8)
#define THERMAL_HOT2NORMAL_INTERRUPT_1		BIT(9)
#define THERMAL_COLD_INTERRUPT_2			BIT(10)
#define THERMAL_HOT_INTERRUPT_2				BIT(11)
#define THERMAL_LOW_OFFSET_INTERRUPT_2		BIT(12)
#define THERMAL_HIGH_OFFSET_INTERRUPT_2		BIT(13)
#define THERMAL_HOT2NORMAL_INTERRUPT_2		BIT(14)
#define THERMAL_AHB_TIMEOUT_INTERRUPT		BIT(15)
#define THERMAL_DEVICE_TIMEOUT_INTERRUPT	BIT(15)
#define THERMAL_IMMEDIATE_INTERRUPT_0		BIT(16)
#define THERMAL_IMMEDIATE_INTERRUPT_1		BIT(17)
#define THERMAL_IMMEDIATE_INTERRUPT_2		BIT(18)
#define THERMAL_FILTER_INTERRUPT_0			BIT(19)
#define THERMAL_FILTER_INTERRUPT_1			BIT(20)
#define THERMAL_FILTER_INTERRUPT_2			BIT(21)
#define THERMAL_COLD_INTERRUPT_3			BIT(22)
#define THERMAL_HOT_INTERRUPT_3				BIT(23)
#define THERMAL_LOW_OFFSET_INTERRUPT_3		BIT(24)
#define THERMAL_HIGH_OFFSET_INTERRUPT_3		BIT(25)
#define THERMAL_HOT2NORMAL_INTERRUPT_3		BIT(26)
#define THERMAL_IMMEDIATE_INTERRUPT_3		BIT(27)
#define THERMAL_FILTER_INTERRUPT_3			BIT(28)
#define THERMAL_PROTECTION_STAGE_1			BIT(29)
#define THERMAL_PROTECTION_STAGE_2			BIT(30)
#define THERMAL_PROTECTION_STAGE_3			BIT(31)

#define CFG_REGISTER(reg, value)	(reg << 8 | value)

#define TSV2F_CHOP_CKSEL_AND_TSV2F_EN_V5	CFG_REGISTER(RG_TSV2F_CTRL_2, 0x8C)
#define TSBG_DEM_CKSEL_X_TSBG_CHOP_EN_V5	CFG_REGISTER(RG_TSV2F_CTRL_4, 0xFC)
#define SET_TS_CHOP_V5				CFG_REGISTER(RG_TSV2F_CTRL_0, 0xF1)

#define STOP_COUNTING_V4					CFG_REGISTER(RG_TSFM_CTRL_0, 0x00)
#define SET_RG_TSFM_LPDLY_V4				CFG_REGISTER(RG_TSFM_CTRL_4, 0xA6)
#define SET_COUNTING_WINDOW_20US1_V4		CFG_REGISTER(RG_TSFM_CTRL_2, 0x00)
#define SET_COUNTING_WINDOW_20US2_V4		CFG_REGISTER(RG_TSFM_CTRL_1, 0x20)
#define TSV2F_CHOP_CKSEL_AND_TSV2F_EN_V4	CFG_REGISTER(RG_TSV2F_CTRL_2, 0x84)
#define TSBG_DEM_CKSEL_X_TSBG_CHOP_EN_V4	CFG_REGISTER(RG_TSV2F_CTRL_4, 0x7C)
#define SET_TS_RSV_V4						CFG_REGISTER(RG_TSV2F_CTRL_1, 0x8D)
#define SET_TS_EN_V4						CFG_REGISTER(RG_TSV2F_CTRL_0, 0xF4)
#define TOGGLE_RG_TSV2F_VCO_RST1_V4			CFG_REGISTER(RG_TSV2F_CTRL_0, 0xFC)
#define TOGGLE_RG_TSV2F_VCO_RST2_V4			CFG_REGISTER(RG_TSV2F_CTRL_0, 0xF4)

#define SET_LVTS_AUTO_RCK_V4				CFG_REGISTER(RG_TSV2F_CTRL_6, 0x01)
#define SELECT_SENSOR_RCK_V4(id)			CFG_REGISTER(RG_TSV2F_CTRL_5, (id))
#define SET_DEVICE_SINGLE_MODE_V4			CFG_REGISTER(RG_TSFM_CTRL_3, 0x78)
#define KICK_OFF_RCK_COUNTING_V4			CFG_REGISTER(RG_TSFM_CTRL_0, 0x02)
#define SET_SENSOR_NO_RCK_V4				CFG_REGISTER(RG_TSV2F_CTRL_5, 0x10)
#define SET_DEVICE_LOW_POWER_SINGLE_MODE_V4	CFG_REGISTER(RG_TSFM_CTRL_3, 0xB8)

#define HAS_FEATURE(lvts_data, feature)		(lvts_data->feature_bitmap & (feature))
#define GET_BASE_ADDR(lvts_data, tc_id)		(lvts_data->base + lvts_data->tc[tc_id].addr_offset)
#define GET_CAL_DATA_BITMASK(index, lvts_data, h, l)	(((index) < lvts_data->num_efuse_addr) \
	? ((lvts_data->efuse[(index)] & GENMASK(h, l)) >> l) : 0)

#define GET_TC_SENSOR_NUM(lvts_data, tc_id)		(lvts_data->tc[tc_id].num_sensor)
#define ONE_SAMPLE		(lvts_data->counting_window_us + 2 * BUS_ACCESS_US)
#define NUM_OF_SAMPLE(tc_id)	((lvts_data->tc[tc_id].hw_filter < LVTS_FILTER_2) ? 1 :	\
	((lvts_data->tc[tc_id].hw_filter > LVTS_FILTER_16_OF_18) ? 1 :	\
	((lvts_data->tc[tc_id].hw_filter == LVTS_FILTER_16_OF_18) ? 18 : \
	((lvts_data->tc[tc_id].hw_filter == LVTS_FILTER_8_OF_10) ? 10 :	\
	(lvts_data->tc[tc_id].hw_filter * 2)))))

#define PERIOD_UNIT_US(tc_id)	((lvts_data->tc[tc_id].tc_speed->period_unit * 256 * \
	CLOCK_26MHZ_CYCLE_NS) / 1000)
#define FILTER_INT_US(tc_id)	(lvts_data->tc[tc_id].tc_speed->filter_interval_delay * \
	PERIOD_UNIT_US(tc_id))
#define SENSOR_INT_US(tc_id)	(lvts_data->tc[tc_id].tc_speed->sensor_interval_delay * \
	PERIOD_UNIT_US(tc_id))
#define GROUP_INT_US(tc_id)		(lvts_data->tc[tc_id].tc_speed->group_interval_delay * \
	PERIOD_UNIT_US(tc_id))
#define SENSOR_LATENCY_US(tc_id)	((NUM_OF_SAMPLE(tc_id) - 1) * FILTER_INT_US(tc_id) + \
	NUM_OF_SAMPLE(tc_id) * ONE_SAMPLE)
#define GROUP_LATENCY_US(tc_id)		(GET_TC_SENSOR_NUM(lvts_data, tc_id) * \
	SENSOR_LATENCY_US(tc_id) + (GET_TC_SENSOR_NUM(lvts_data, tc_id) - 1) * SENSOR_INT_US(tc_id) + \
	GROUP_INT_US(tc_id))

#define CK26M_ACTIVE(lvts_data)	(((lvts_data->feature_bitmap & FEATURE_CK26M_ACTIVE) ? 1 : 0) << 30)
#define DEVICE_ACCESS			(SCK_ONLY | DEVICE_ACCESS_STARTUS | READ_32BIT_ACCESS)
#define DEVICE_READ				(CK26M_ACTIVE(lvts_data) | DEVICE_ACCESS)
#define DEVICE_WRITE			(CK26M_ACTIVE(lvts_data) | DEVICE_ACCESS | WRITE_ACCESS)
#define READ_BACK_DEVICE_ID		(CK26M_ACTIVE(lvts_data) | DEVICE_ACCESS | BROADCAST_ID_UPDATE | \
	RG_DID_LVTS << 8)
#define READ_DEVICE_REG(reg_idx)	(DEVICE_READ | (reg_idx) << 8 | 0x00)
#define RESET_ALL_DEVICES			(DEVICE_WRITE | RG_TSFM_RST << 8 | 0xFF)

/*
 * LVTS HW filter settings
 * 000: Get one sample
 * 001: Get 2 samples and average them
 * 010: Get 4 samples, drop max and min, then average the rest of 2 samples
 * 011: Get 6 samples, drop max and min, then average the rest of 4 samples
 * 100: Get 10 samples, drop max and min, then average the rest of 8 samples
 * 101: Get 18 samples, drop max and min, then average the rest of 16 samples
 */
enum lvts_hw_filter {
	LVTS_FILTER_1,
	LVTS_FILTER_2,
	LVTS_FILTER_2_OF_4,
	LVTS_FILTER_4_OF_6,
	LVTS_FILTER_8_OF_10,
	LVTS_FILTER_16_OF_18
};

enum lvts_sensing_point {
	SENSING_POINT0,
	SENSING_POINT1,
	SENSING_POINT2,
	SENSING_POINT3,
	ALL_SENSING_POINTS
};

struct lvts_data;

/**
 * struct lvts_speed_settings - A structure to hold the data related to polling rate
 * @period_unit: Period unit is a base for all interval delays
 * @group_interval_delay:  Delay between different rounds
 * @filter_interval_delay: Delay between two samples of the same sensor
 * @sensor_interval_delay: Delay between two samples of differnet sensors
 *
 * Calculation is achieved with the following equations:
 * For the period unit: (period_us * 1000) / (256 * clock_26mhz_cycle_ns)
 * For the interval delays: delay / period_us
 */
struct lvts_speed_settings {
	unsigned int period_unit;
	unsigned int group_interval_delay;
	unsigned int filter_interval_delay;
	unsigned int sensor_interval_delay;
};

struct lvts_tc_settings {
	unsigned int dev_id;
	unsigned int addr_offset;
	unsigned int num_sensor;
	unsigned int ts_offset;
	unsigned int sensor_map[ALL_SENSING_POINTS];	/* In sensor ID */
	struct lvts_speed_settings *tc_speed;
	/*
	 * HW filter setting
	 * 000: Get one sample
	 * 001: Get 2 samples and average them
	 * 010: Get 4 samples, drop max and min, then average the rest of 2 samples
	 * 011: Get 6 samples, drop max and min, then average the rest of 4 samples
	 * 100: Get 10 samples, drop max and min, then average the rest of 8 samples
	 * 101: Get 18 samples, drop max and min, then average the rest of 16 samples
	 */
	unsigned int hw_filter;
	/*
	 * Dominator_sensing point is used to select a sensing point
	 * and reference its temperature to trigger Thermal HW Reboot
	 * When it is ALL_SENSING_POINTS, it will select all sensing points
	 */
	int dominator_sensing_point;
	int hw_reboot_trip_point;		/* -274000: Disable HW reboot */
	unsigned int irq_bit;
};

struct lvts_formula_coeff {
	int a;
	int b;
	unsigned int golden_temp;
};

struct lvts_sensor_cal_data {
	int use_fake_efuse;				/* 1: Use fake efuse, 0: Use real efuse */
	unsigned int golden_temp;
	unsigned int *count_r;
	unsigned int *count_rc;
	unsigned int *count_rc_now;
	unsigned int default_golden_temp;
	unsigned int default_count_r;
	unsigned int default_count_rc;
};

struct platform_ops {
	void (*efuse_to_cal_data)(struct lvts_data *lvts_data);
	void (*device_enable_and_init)(struct lvts_data *lvts_data);
	void (*device_enable_auto_rck)(struct lvts_data *lvts_data);
	int (*device_read_count_rc_n)(struct lvts_data *lvts_data);
	void (*set_cal_data)(struct lvts_data *lvts_data);
	void (*init_controller)(struct lvts_data *lvts_data);
};

struct lvts_data {
	struct device *dev;
	struct clk *clk;
	void __iomem *base;				/* LVTS base addresses */
	unsigned int irq_num;			/* LVTS interrupt numbers */
	struct reset_control *reset;
	int num_tc;						/* Number of LVTS thermal controllers */
	const struct lvts_tc_settings *tc;
	int counting_window_us;			/* LVTS device counting window */
	int num_sensor;					/* Number of sensors in this platform */
	void __iomem **reg;
	struct platform_ops ops;
	int feature_bitmap;				/* Show what features are enabled */
	unsigned int num_efuse_addr;
	unsigned int *efuse;
	unsigned int num_efuse_block;	/* Number of contiguous efuse indexes */
	struct lvts_sensor_cal_data cal_data;
	struct lvts_formula_coeff coeff;
};

struct soc_temp_tz {
	unsigned int id;
	struct lvts_data *lvts_data;
};

extern void lvts_device_enable_and_init_v5(struct lvts_data *lvts_data);
extern void lvts_device_enable_and_init_v4(struct lvts_data *lvts_data);
extern void lvts_device_enable_auto_rck_v4(struct lvts_data *lvts_data);
extern int lvts_device_read_count_rc_n_v4(struct lvts_data *lvts_data);
extern void lvts_set_calibration_data_v4(struct lvts_data *lvts_data);
extern void lvts_init_controller_v4(struct lvts_data *lvts_data);

extern int lvts_probe(struct platform_device *pdev);
extern int lvts_remove(struct platform_device *pdev);
extern int lvts_suspend(struct platform_device *pdev, pm_message_t state);
extern int lvts_resume(struct platform_device *pdev);

#endif /* __MTK_SOC_TEMP_LVTS_H__ */
