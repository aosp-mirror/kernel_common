// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 MediaTek Inc.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/thermal.h>
#include "soc_temp_lvts.h"

static int lvts_raw_to_temp(struct lvts_formula_coeff *co, unsigned int msr_raw)
{
	/* This function returns degree mC */

	int temp;

	temp = (co->a * ((unsigned long long)msr_raw)) >> 14;
	temp = temp + co->golden_temp * 500 + co->b;

	return temp;
}

static unsigned int lvts_temp_to_raw(struct lvts_formula_coeff *co, int temp)
{
	unsigned int msr_raw;

	msr_raw = div_s64((s64)((co->golden_temp * 500 + co->b - temp)) << 14,
		(-1 * co->a));

	return msr_raw;
}

static int soc_temp_lvts_read_temp(struct thermal_zone_device *tz, int *temperature)
{
	struct soc_temp_tz *lvts_tz = tz->devdata;
	struct lvts_data *lvts_data = lvts_tz->lvts_data;
	struct device *dev = lvts_data->dev;
	unsigned int msr_raw;

	msr_raw = readl(lvts_data->reg[lvts_tz->id]) & MRS_RAW_MASK;
	if (msr_raw == 0) {
		/* Prevents a false critical temperature trap */
		*temperature = 0;
		dev_dbg(dev, "LVTS not yet ready\n");
	} else
		*temperature = lvts_raw_to_temp(&lvts_data->coeff, msr_raw);

	return 0;
}

static const struct thermal_zone_device_ops soc_temp_lvts_ops = {
	.get_temp = soc_temp_lvts_read_temp,
};

static void lvts_write_device(struct lvts_data *lvts_data, unsigned int data,
	int tc_id)
{
	void __iomem *base = GET_BASE_ADDR(lvts_data, tc_id);

	writel(DEVICE_WRITE | data, LVTS_CONFIG_0 + base);

	usleep_range(5, 15);
}

static unsigned int lvts_read_device(struct lvts_data *lvts_data,
	unsigned int reg_idx, int tc_id)
{
	struct device *dev = lvts_data->dev;
	void __iomem *base = GET_BASE_ADDR(lvts_data, tc_id);
	unsigned int data;
	int ret;

	writel(READ_DEVICE_REG(reg_idx), LVTS_CONFIG_0 + base);

	usleep_range(5, 15);

	ret = readl_poll_timeout(LVTS_CONFIG_0 + base, data,
		!(data & DEVICE_ACCESS_STARTUS), 2, 200);
	if (ret)
		dev_err(dev,
			"Error: LVTS %d DEVICE_ACCESS_START is not ready\n", tc_id);

	data = readl(LVTSRDATA0_0 + base);

	return data;
}

static const char * const lvts_error_table[] = {"IDLE", "Write transaction",
	"Waiting for read after Write", "Disable Continue fetching on Device",
	"Read transaction", "Set Device special Register for Voltage threshold",
	"Set TSMCU number for Fetch"};

static void wait_all_tc_sensing_point_idle(struct lvts_data *lvts_data)
{
	struct device *dev = lvts_data->dev;
	unsigned int mask, error_code, is_error;
	void __iomem *base;
	int i, cnt, ret;

	mask = BIT(10) | BIT(7) | BIT(0);

	for (cnt = 0; cnt < 2; cnt++) {
		is_error = 0;
		for (i = 0; i < lvts_data->num_tc; i++) {
			base = GET_BASE_ADDR(lvts_data, i);
			ret = readl_poll_timeout(LVTSMSRCTL1_0 + base, error_code,
						 !(error_code & mask), 2, 200);

			error_code = ((error_code & BIT(10)) >> 8) +
				((error_code & BIT(7)) >> 6) +
				(error_code & BIT(0));

			if (ret)
				dev_err(dev, "LVTS %d error: %s\n",
					i, lvts_error_table[error_code]);

			if (error_code != 0)
				is_error = 1;
		}

		if (is_error == 0)
			break;
	}
}

static void lvts_reset(struct lvts_data *lvts_data)
{
	if (lvts_data->reset)
		reset_control_assert(lvts_data->reset);
	if (lvts_data->reset)
		reset_control_deassert(lvts_data->reset);
}

static void device_identification(struct lvts_data *lvts_data)
{
	struct device *dev = lvts_data->dev;
	unsigned int i, data;
	void __iomem *base;

	for (i = 0; i < lvts_data->num_tc; i++) {
		base = GET_BASE_ADDR(lvts_data, i);

		writel(ENABLE_LVTS_CTRL_CLK, LVTSCLKEN_0 + base);
		lvts_write_device(lvts_data, RESET_ALL_DEVICES, i);
		writel(READ_BACK_DEVICE_ID, LVTS_CONFIG_0 + base);

		usleep_range(5, 15);

		/* Check LVTS device ID */
		data = (readl(LVTS_ID_0 + base) & DEVICE_REG_DATA);
		if (data != (lvts_data->tc->dev_id + i))
			dev_err(dev, "LVTS_TC_%d, Device ID should be 0x%x, but 0x%x\n",
				i, (lvts_data->tc->dev_id + i), data);
	}
}

static void disable_all_sensing_points(struct lvts_data *lvts_data)
{
	unsigned int i;
	void __iomem *base;

	for (i = 0; i < lvts_data->num_tc; i++) {
		base = GET_BASE_ADDR(lvts_data, i);
		writel(DISABLE_SENSING_POINT, LVTSMONCTL0_0 + base);
	}
}

static void enable_all_sensing_points(struct lvts_data *lvts_data)
{
	struct device *dev = lvts_data->dev;
	const struct lvts_tc_settings *tc = lvts_data->tc;
	unsigned int i, num;
	void __iomem *base;

	for (i = 0; i < lvts_data->num_tc; i++) {
		base = GET_BASE_ADDR(lvts_data, i);
		num = tc[i].num_sensor;

		if (num > ALL_SENSING_POINTS) {
			dev_err(dev,
				"%s, LVTS%d, illegal number of sensors: %d\n",
				__func__, i, tc[i].num_sensor);
			continue;
		}

		if ((tc[i].ts_offset == 1) && (num == 1))
			writel(LVTS_SINGLE_SENSE | (0x1 << tc[i].ts_offset),
			       LVTSMONCTL0_0 + base);
		else
			writel(ENABLE_SENSING_POINT(num), LVTSMONCTL0_0 + base);
	}
}

static void set_polling_speed(struct lvts_data *lvts_data, int tc_id)
{
	struct device *dev = lvts_data->dev;
	const struct lvts_tc_settings *tc = lvts_data->tc;
	unsigned int lvts_mon_ctl_1, lvts_mon_ctl_2;
	void __iomem *base;

	base = GET_BASE_ADDR(lvts_data, tc_id);

	lvts_mon_ctl_1 = ((tc[tc_id].tc_speed->group_interval_delay << 20) & GENMASK(29, 20)) |
		(tc[tc_id].tc_speed->period_unit & GENMASK(9, 0));
	lvts_mon_ctl_2 = ((tc[tc_id].tc_speed->filter_interval_delay << 16) & GENMASK(25, 16)) |
		(tc[tc_id].tc_speed->sensor_interval_delay & GENMASK(9, 0));
	/*
	 * Clock source of LVTS thermal controller is 26MHz.
	 * Period unit is a base for all interval delays
	 * All interval delays must multiply it to convert a setting to time.
	 * Filter interval delay is a delay between two samples of the same sensor
	 * Sensor interval delay is a delay between two samples of differnet sensors
	 * Group interval delay is a delay between different rounds.
	 * For example:
	 *     If Period unit = C, filter delay = 1, sensor delay = 2, group delay = 1,
	 *     and two sensors, TS1 and TS2, are in a LVTS thermal controller
	 *     and then
	 *     Period unit = C * 1/26M * 256 = 12 * 38.46ns * 256 = 118.149us
	 *     Filter interval delay = 1 * Period unit = 118.149us
	 *     Sensor interval delay = 2 * Period unit = 236.298us
	 *     Group interval delay = 1 * Period unit = 118.149us
	 *
	 *     TS1    TS1 ... TS1    TS2    TS2 ... TS2    TS1...
	 *        <--> Filter interval delay
	 *                       <--> Sensor interval delay
	 *                                             <--> Group interval delay
	 */
	writel(lvts_mon_ctl_1, LVTSMONCTL1_0 + base);
	writel(lvts_mon_ctl_2, LVTSMONCTL2_0 + base);

	dev_dbg(dev, "LVTS_TC_%d, LVTSMONCTL1_0= 0x%x, LVTSMONCTL2_0= 0x%x\n",
		 tc_id, readl(LVTSMONCTL1_0 + base),
		 readl(LVTSMONCTL2_0 + base));
}

static void set_hw_filter(struct lvts_data *lvts_data, int tc_id)
{
	struct device *dev = lvts_data->dev;
	const struct lvts_tc_settings *tc = lvts_data->tc;
	unsigned int option;
	void __iomem *base;

	base = GET_BASE_ADDR(lvts_data, tc_id);
	option = tc[tc_id].hw_filter & 0x7;
	/*
	 * hw filter
	 * 000: Get one sample
	 * 001: Get 2 samples and average them
	 * 010: Get 4 samples, drop max and min, then average the rest of 2 samples
	 * 011: Get 6 samples, drop max and min, then average the rest of 4 samples
	 * 100: Get 10 samples, drop max and min, then average the rest of 8 samples
	 * 101: Get 18 samples, drop max and min, then average the rest of 16 samples
	 */
	option = (option << 9) | (option << 6) | (option << 3) | option;

	writel(option, LVTSMSRCTL0_0 + base);
	dev_dbg(dev, "LVTS_TC_%d, LVTSMSRCTL0_0= 0x%x\n",
		 tc_id, readl(LVTSMSRCTL0_0 + base));
}

static int get_dominator_index(struct lvts_data *lvts_data, int tc_id)
{
	struct device *dev = lvts_data->dev;
	const struct lvts_tc_settings *tc = lvts_data->tc;
	int d_index;

	if (tc[tc_id].dominator_sensing_point == ALL_SENSING_POINTS) {
		d_index = ALL_SENSING_POINTS;
	} else if ((tc[tc_id].dominator_sensing_point <
		tc[tc_id].num_sensor) || (tc[tc_id].ts_offset != 0)) {
		d_index = tc[tc_id].dominator_sensing_point;
	} else {
		dev_err(dev,
			"Error: LVTS%d, dominator_sensing_point= %d should smaller than num_sensor= %d\n",
			tc_id, tc[tc_id].dominator_sensing_point,
			tc[tc_id].num_sensor);

		dev_err(dev, "Use the sensing point 0 as the dominated sensor\n");
		d_index = SENSING_POINT0;
	}

	return d_index;
}

static void disable_hw_reboot_interrupt(struct lvts_data *lvts_data, int tc_id)
{
	unsigned int temp;
	void __iomem *base;

	base = GET_BASE_ADDR(lvts_data, tc_id);

	/*
	 * LVTS thermal controller has two interrupts for thermal HW reboot
	 * One is for AP SW and the other is for RGU
	 * The interrupt of AP SW can turn off by a bit of a register, but
	 * the other for RGU cannot.
	 * To prevent rebooting device accidentally, we are going to add
	 * a huge offset to LVTS and make LVTS always report extremely low
	 * temperature.
	 */

	/*
	 * After adding the huge offset 0x3FFF, LVTS alawys adds the
	 * offset to MSR_RAW.
	 * When MSR_RAW is larger, SW will convert lower temperature/
	 */
	temp = readl(LVTSPROTCTL_0 + base);
	writel(temp | 0x3FFF, LVTSPROTCTL_0 + base);

	/* Disable the interrupt of AP SW */
	temp = readl(LVTSMONINT_0 + base);
	writel(temp & ~(STAGE3_INT_EN), LVTSMONINT_0 + base);
}

static void enable_hw_reboot_interrupt(struct lvts_data *lvts_data, int tc_id)
{
	unsigned int temp;
	void __iomem *base;

	base = GET_BASE_ADDR(lvts_data, tc_id);

	/* Enable the interrupt of AP SW */
	temp = readl(LVTSMONINT_0 + base);
	writel(temp | STAGE3_INT_EN, LVTSMONINT_0 + base);
	/* Clear the offset */
	temp = readl(LVTSPROTCTL_0 + base);
	writel(temp & ~PROTOFFSET, LVTSPROTCTL_0 + base);
}

static void set_tc_hw_reboot_threshold(struct lvts_data *lvts_data,
	int trip_point, int tc_id)
{
	struct device *dev = lvts_data->dev;
	unsigned int msr_raw, temp, config, d_index;
	void __iomem *base;

	base = GET_BASE_ADDR(lvts_data, tc_id);
	d_index = get_dominator_index(lvts_data, tc_id);

	dev_info(dev, "lvts_tc_%d: dominator sensing point = %d\n", tc_id, d_index);

	disable_hw_reboot_interrupt(lvts_data, tc_id);

	temp = readl(LVTSPROTCTL_0 + base);
	if (d_index == ALL_SENSING_POINTS) {
		/* Maximum of 4 sensing points */
		config = (0x1 << 16);
		writel(config | temp, LVTSPROTCTL_0 + base);
	} else {
		/* Select protection sensor */
		config = ((d_index << 2) + 0x2) << 16;
		writel(config | temp, LVTSPROTCTL_0 + base);
	}

	msr_raw = lvts_temp_to_raw(&lvts_data->coeff, trip_point);
	writel(msr_raw, LVTSPROTTC_0 + base);

	enable_hw_reboot_interrupt(lvts_data, tc_id);
}

static void set_all_tc_hw_reboot(struct lvts_data *lvts_data)
{
	const struct lvts_tc_settings *tc = lvts_data->tc;
	int i, trip_point;

	for (i = 0; i < lvts_data->num_tc; i++) {
		trip_point = tc[i].hw_reboot_trip_point;

		if (tc[i].num_sensor == 0)
			continue;

		if (trip_point == THERMAL_TEMP_INVALID)
			continue;

		set_tc_hw_reboot_threshold(lvts_data, trip_point, i);
	}
}

static int lvts_init(struct lvts_data *lvts_data)
{
	struct platform_ops *ops = &lvts_data->ops;
	struct device *dev = lvts_data->dev;
	int ret;

	ret = clk_prepare_enable(lvts_data->clk);
	if (ret) {
		dev_err(dev,
			"Error: Failed to enable lvts controller clock: %d\n",
			ret);
		return ret;
	}

	lvts_reset(lvts_data);

	device_identification(lvts_data);
	if (ops->device_enable_and_init)
		ops->device_enable_and_init(lvts_data);

	if (HAS_FEATURE(lvts_data, FEATURE_DEVICE_AUTO_RCK)) {
		if (ops->device_enable_auto_rck)
			ops->device_enable_auto_rck(lvts_data);
	} else {
		if (ops->device_read_count_rc_n)
			ops->device_read_count_rc_n(lvts_data);
	}

	if (ops->set_cal_data)
		ops->set_cal_data(lvts_data);

	disable_all_sensing_points(lvts_data);
	wait_all_tc_sensing_point_idle(lvts_data);
	if (ops->init_controller)
		ops->init_controller(lvts_data);
	enable_all_sensing_points(lvts_data);

	set_all_tc_hw_reboot(lvts_data);

	return 0;
}

static int prepare_calibration_data(struct lvts_data *lvts_data)
{
	struct device *dev = lvts_data->dev;
	struct lvts_sensor_cal_data *cal_data = &lvts_data->cal_data;
	struct platform_ops *ops = &lvts_data->ops;
	int i, offset;
	char buffer[512];

	cal_data->count_r = devm_kcalloc(dev, lvts_data->num_sensor,
		sizeof(*cal_data->count_r), GFP_KERNEL);
	if (!cal_data->count_r)
		return -ENOMEM;

	cal_data->count_rc = devm_kcalloc(dev, lvts_data->num_sensor,
		sizeof(*cal_data->count_rc), GFP_KERNEL);
	if (!cal_data->count_rc)
		return -ENOMEM;

	if (ops->efuse_to_cal_data && !cal_data->use_fake_efuse)
		ops->efuse_to_cal_data(lvts_data);
	if (cal_data->golden_temp == 0 || cal_data->golden_temp > GOLDEN_TEMP_MAX)
		cal_data->use_fake_efuse = 1;

	if (cal_data->use_fake_efuse) {
		/* It means all efuse data are equal to 0 */
		dev_err(dev,
			"%s: This sample is not calibrated, fake !!\n", __func__);

		cal_data->golden_temp = cal_data->default_golden_temp;
		for (i = 0; i < lvts_data->num_sensor; i++) {
			cal_data->count_r[i] = cal_data->default_count_r;
			cal_data->count_rc[i] = cal_data->default_count_rc;
		}
	}

	lvts_data->coeff.golden_temp = cal_data->golden_temp;

	dev_dbg(dev, "golden_temp = %d\n", cal_data->golden_temp);

	offset = snprintf(buffer, sizeof(buffer), "[lvts_cal] num:g_count:g_count_rc ");
	for (i = 0; i < lvts_data->num_sensor; i++)
		offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%d:%d:%d ",
			i, cal_data->count_r[i], cal_data->count_rc[i]);

	buffer[offset] = '\0';

	return 0;
}

static int get_calibration_data(struct lvts_data *lvts_data)
{
	struct device *dev = lvts_data->dev;
	char cell_name[32];
	struct nvmem_cell *cell;
	u32 *buf;
	size_t len;
	int i, j, index = 0, ret;

	lvts_data->efuse = devm_kcalloc(dev, lvts_data->num_efuse_addr,
					sizeof(*lvts_data->efuse), GFP_KERNEL);
	if (!lvts_data->efuse)
		return -ENOMEM;

	for (i = 0; i < lvts_data->num_efuse_block; i++) {
		snprintf(cell_name, sizeof(cell_name), "lvts_calib_data%d", i + 1);
		cell = nvmem_cell_get(dev, cell_name);
		if (IS_ERR(cell)) {
			dev_err(dev, "Error: Failed to get nvmem cell %s\n", cell_name);
			return PTR_ERR(cell);
		}

		buf = (u32 *)nvmem_cell_read(cell, &len);
		nvmem_cell_put(cell);

		if (IS_ERR(buf))
			return PTR_ERR(buf);

		for (j = 0; j < (len / sizeof(u32)); j++) {
			if (index >= lvts_data->num_efuse_addr) {
				dev_err(dev, "Array efuse is going to overflow");
				kfree(buf);
				return -EINVAL;
			}

			lvts_data->efuse[index] = buf[j];
			index++;
		}

		kfree(buf);
	}

	ret = prepare_calibration_data(lvts_data);

	return ret;
}

static int lvts_init_tc_regs(struct device *dev, struct lvts_data *lvts_data)
{
	const struct lvts_tc_settings *tc = lvts_data->tc;
	unsigned int i, j, s_index, x;
	void __iomem *base;

	lvts_data->reg = devm_kcalloc(dev, lvts_data->num_sensor,
		sizeof(*lvts_data->reg), GFP_KERNEL);
	if (!lvts_data->reg)
		return -ENOMEM;

	for (i = 0; i < lvts_data->num_tc; i++) {
		base = GET_BASE_ADDR(lvts_data, i);
		for (j = 0; j < tc[i].num_sensor; j++) {
			s_index = tc[i].sensor_map[j];
			x = j + tc[i].ts_offset;
			lvts_data->reg[s_index] = LVTSMSR0_0 + base + 0x4 * x;
		}
	}

	return 0;
}

static int of_update_lvts_data(struct lvts_data *lvts_data,
	struct platform_device *pdev)
{
	struct device *dev = lvts_data->dev;
	struct resource *res;
	int ret;

	lvts_data->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(lvts_data->clk))
		return PTR_ERR(lvts_data->clk);

	/* Get base address */
	res = platform_get_mem_or_io(pdev, 0);
	if (!res) {
		dev_err(dev, "No IO resource\n");
		return -ENXIO;
	}

	lvts_data->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(lvts_data->base)) {
		dev_err(dev, "Failed to remap io\n");
		return PTR_ERR(lvts_data->base);
	}

	/* Get interrupt number */
	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(dev, "No irq resource\n");
		return -EINVAL;
	}
	lvts_data->irq_num = ret;

	/* Get reset control */
	lvts_data->reset = devm_reset_control_get_by_index(dev, 0);
	if (IS_ERR(lvts_data->reset)) {
		dev_err(dev, "Failed to get reset control\n");
		return PTR_ERR(lvts_data->reset);
	}

	ret = lvts_init_tc_regs(dev, lvts_data);
	if (ret)
		return ret;

	ret = get_calibration_data(lvts_data);
	if (ret)
		return ret;

	return 0;
}

static void lvts_device_close(struct lvts_data *lvts_data)
{
	unsigned int i;
	void __iomem *base;

	for (i = 0; i < lvts_data->num_tc; i++) {
		base = GET_BASE_ADDR(lvts_data, i);
		lvts_write_device(lvts_data, RESET_ALL_DEVICES, i);
		writel(DISABLE_LVTS_CTRL_CLK, LVTSCLKEN_0 + base);
	}
}

static void lvts_close(struct lvts_data *lvts_data)
{
	disable_all_sensing_points(lvts_data);
	wait_all_tc_sensing_point_idle(lvts_data);
	lvts_device_close(lvts_data);
	clk_disable_unprepare(lvts_data->clk);
}

static void tc_irq_handler(struct lvts_data *lvts_data, int tc_id)
{
	const struct device *dev = lvts_data->dev;
	unsigned int ret = 0;
	void __iomem *base;

	base = GET_BASE_ADDR(lvts_data, tc_id);

	ret = readl(LVTSMONINTSTS_0 + base);
	/* Write back to clear interrupt status */
	writel(ret, LVTSMONINTSTS_0 + base);

	dev_dbg(dev, "LVTS thermal controller %d, LVTSMONINTSTS=0x%08x\n", tc_id, ret);

	if (ret & THERMAL_PROTECTION_STAGE_3)
		dev_dbg(dev, "Thermal protection stage 3 interrupt triggered\n");
}

static irqreturn_t irq_handler(int irq, void *dev_id)
{
	struct lvts_data *lvts_data = (struct lvts_data *)dev_id;
	struct device *dev = lvts_data->dev;
	const struct lvts_tc_settings *tc = lvts_data->tc;
	unsigned int i, *irq_bitmap;
	void __iomem *base;

	irq_bitmap = kcalloc(1, sizeof(*irq_bitmap), GFP_ATOMIC);

	if (!irq_bitmap)
		return IRQ_NONE;

	base = lvts_data->base;
	*irq_bitmap = readl(THERMINTST + base);
	dev_dbg(dev, "THERMINTST = 0x%x\n", *irq_bitmap);

	for (i = 0; i < lvts_data->num_tc; i++) {
		if (tc[i].irq_bit == 0)
			tc_irq_handler(lvts_data, i);
	}

	kfree(irq_bitmap);

	return IRQ_HANDLED;
}

static int lvts_register_irq_handler(struct lvts_data *lvts_data)
{
	struct device *dev = lvts_data->dev;
	int ret;

	ret = devm_request_irq(dev, lvts_data->irq_num, irq_handler,
		IRQF_TRIGGER_HIGH, "mtk_lvts", lvts_data);

	if (ret) {
		dev_err(dev, "Failed to register LVTS IRQ, ret %d, irq_num %d\n",
			ret, lvts_data->irq_num);
		lvts_close(lvts_data);
		return ret;
	}

	return 0;
}

static int lvts_register_thermal_zones(struct lvts_data *lvts_data)
{
	struct device *dev = lvts_data->dev;
	struct thermal_zone_device *tzdev;
	struct soc_temp_tz *lvts_tz;
	int i, ret;

	for (i = 0; i < lvts_data->num_sensor; i++) {
		lvts_tz = devm_kzalloc(dev, sizeof(*lvts_tz), GFP_KERNEL);
		if (!lvts_tz) {
			lvts_close(lvts_data);
			return -ENOMEM;
		}

		lvts_tz->id = i;
		lvts_tz->lvts_data = lvts_data;

		tzdev = devm_thermal_of_zone_register(dev, lvts_tz->id,
			lvts_tz, &soc_temp_lvts_ops);

		if (IS_ERR(tzdev)) {
			if (lvts_tz->id != 0)
				return 0;

			ret = PTR_ERR(tzdev);
			dev_err(dev, "Error: Failed to register lvts tz %d, ret = %d\n",
				lvts_tz->id, ret);
			lvts_close(lvts_data);
			return ret;
		}
	}

	return 0;
}

void lvts_device_enable_and_init_v5(struct lvts_data *lvts_data)
{
	unsigned int i;

	for (i = 0; i < lvts_data->num_tc; i++) {
		lvts_write_device(lvts_data, STOP_COUNTING_V4, i);
		lvts_write_device(lvts_data, SET_RG_TSFM_LPDLY_V4, i);
		lvts_write_device(lvts_data, SET_COUNTING_WINDOW_20US1_V4, i);
		lvts_write_device(lvts_data, SET_COUNTING_WINDOW_20US2_V4, i);
		lvts_write_device(lvts_data, TSV2F_CHOP_CKSEL_AND_TSV2F_EN_V5, i);
		lvts_write_device(lvts_data, TSBG_DEM_CKSEL_X_TSBG_CHOP_EN_V5, i);
		lvts_write_device(lvts_data, SET_TS_RSV_V4, i);
		lvts_write_device(lvts_data, SET_TS_CHOP_V5, i);
	}
	lvts_data->counting_window_us = 20;
}
EXPORT_SYMBOL_GPL(lvts_device_enable_and_init_v5);

void lvts_device_enable_and_init_v4(struct lvts_data *lvts_data)
{
	unsigned int i;

	for (i = 0; i < lvts_data->num_tc; i++) {
		lvts_write_device(lvts_data, STOP_COUNTING_V4, i);
		lvts_write_device(lvts_data, SET_RG_TSFM_LPDLY_V4, i);
		lvts_write_device(lvts_data, SET_COUNTING_WINDOW_20US1_V4, i);
		lvts_write_device(lvts_data, SET_COUNTING_WINDOW_20US2_V4, i);
		lvts_write_device(lvts_data, TSV2F_CHOP_CKSEL_AND_TSV2F_EN_V4, i);
		lvts_write_device(lvts_data, TSBG_DEM_CKSEL_X_TSBG_CHOP_EN_V4, i);
		lvts_write_device(lvts_data, SET_TS_RSV_V4, i);
		lvts_write_device(lvts_data, SET_TS_EN_V4, i);
		lvts_write_device(lvts_data, TOGGLE_RG_TSV2F_VCO_RST1_V4, i);
		lvts_write_device(lvts_data, TOGGLE_RG_TSV2F_VCO_RST2_V4, i);
	}

	lvts_data->counting_window_us = 20;
}
EXPORT_SYMBOL_GPL(lvts_device_enable_and_init_v4);

void lvts_device_enable_auto_rck_v4(struct lvts_data *lvts_data)
{
	unsigned int i;

	for (i = 0; i < lvts_data->num_tc; i++)
		lvts_write_device(lvts_data, SET_LVTS_AUTO_RCK_V4, i);
}
EXPORT_SYMBOL_GPL(lvts_device_enable_auto_rck_v4);

int lvts_device_read_count_rc_n_v4(struct lvts_data *lvts_data)
{
	/* Resistor-Capacitor Calibration */
	/* count_RC_N: count RC now */
	struct device *dev = lvts_data->dev;
	const struct lvts_tc_settings *tc = lvts_data->tc;
	struct lvts_sensor_cal_data *cal_data = &lvts_data->cal_data;
	unsigned int offset, size, s_index, data;
	void __iomem *base;
	int ret, i, j;
	char buffer[512];

	cal_data->count_rc_now = devm_kcalloc(dev, lvts_data->num_sensor,
		sizeof(*cal_data->count_rc_now), GFP_KERNEL);
	if (!cal_data->count_rc_now)
		return -ENOMEM;

	for (i = 0; i < lvts_data->num_tc; i++) {
		base = GET_BASE_ADDR(lvts_data, i);
		for (j = 0; j < tc[i].num_sensor; j++) {
			s_index = tc[i].sensor_map[j];

			lvts_write_device(lvts_data, SELECT_SENSOR_RCK_V4(j), i);
			lvts_write_device(lvts_data, SET_DEVICE_SINGLE_MODE_V4, i);
			usleep_range(10, 20);

			lvts_write_device(lvts_data, KICK_OFF_RCK_COUNTING_V4, i);
			usleep_range(30, 40);

			ret = readl_poll_timeout(LVTS_CONFIG_0 + base, data,
						 !(data & DEVICE_SENSING_STATUS), 2, 200);
			if (ret)
				dev_err(dev,
					"Error: LVTS %d DEVICE_SENSING_STATUS didn't ready\n", i);

			data = lvts_read_device(lvts_data, 0x00, i);

			cal_data->count_rc_now[s_index] = (data & GENMASK(23, 0));
		}

		/* Recover Setting for Normal Access on
		 * temperature fetch
		 */
		lvts_write_device(lvts_data, SET_SENSOR_NO_RCK_V4, i);
		lvts_write_device(lvts_data, SET_DEVICE_LOW_POWER_SINGLE_MODE_V4, i);
	}

	size = sizeof(buffer);
	offset = snprintf(buffer, size, "[COUNT_RC_NOW] ");
	for (i = 0; i < lvts_data->num_sensor; i++)
		offset += snprintf(buffer + offset, size - offset, "%d:%d ",
			i, cal_data->count_rc_now[i]);

	buffer[offset] = '\0';
	dev_dbg(dev, "%s\n", buffer);

	return 0;
}
EXPORT_SYMBOL_GPL(lvts_device_read_count_rc_n_v4);

void lvts_set_calibration_data_v4(struct lvts_data *lvts_data)
{
	const struct lvts_tc_settings *tc = lvts_data->tc;
	struct lvts_sensor_cal_data *cal_data = &lvts_data->cal_data;
	unsigned int i, j, s_index, lvts_calib_data, x;
	void __iomem *base;

	for (i = 0; i < lvts_data->num_tc; i++) {
		base = GET_BASE_ADDR(lvts_data, i);

		for (j = 0; j < tc[i].num_sensor; j++) {
			s_index = tc[i].sensor_map[j];
			x = j + tc[i].ts_offset;

			if (HAS_FEATURE(lvts_data, FEATURE_DEVICE_AUTO_RCK))
				lvts_calib_data = cal_data->count_r[s_index];
			else
				lvts_calib_data = (((unsigned long long)
					cal_data->count_rc_now[s_index]) *
					cal_data->count_r[s_index]) >> 14;

			writel(lvts_calib_data, LVTSEDATA00_0 + base + 0x4 * x);
		}
	}
}
EXPORT_SYMBOL_GPL(lvts_set_calibration_data_v4);

void lvts_init_controller_v4(struct lvts_data *lvts_data)
{
	struct device *dev = lvts_data->dev;
	unsigned int i;
	void __iomem *base;

	for (i = 0; i < lvts_data->num_tc; i++) {
		base = GET_BASE_ADDR(lvts_data, i);

		lvts_write_device(lvts_data, SET_DEVICE_LOW_POWER_SINGLE_MODE_V4, i);

		writel(SET_SENSOR_INDEX, LVTSTSSEL_0 + base);
		writel(SET_CALC_SCALE_RULES, LVTSCALSCALE_0 + base);

		set_polling_speed(lvts_data, i);
		set_hw_filter(lvts_data, i);

		dev_info(dev, "lvts_tc_%d: read all %d sensors in %d us, one in %d us\n",
			i, GET_TC_SENSOR_NUM(lvts_data, i), GROUP_LATENCY_US(i), SENSOR_LATENCY_US(i));
	}
}
EXPORT_SYMBOL_GPL(lvts_init_controller_v4);

int lvts_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lvts_data *lvts_data;
	int ret;

	lvts_data = (struct lvts_data *)of_device_get_match_data(dev);

	if (!lvts_data)	{
		dev_err(dev, "Error: Failed to get lvts platform data\n");
		return -ENODATA;
	}

	lvts_data->dev = &pdev->dev;

	ret = of_update_lvts_data(lvts_data, pdev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, lvts_data);

	ret = lvts_init(lvts_data);
	if (ret)
		return ret;

	ret = lvts_register_irq_handler(lvts_data);
	if (ret)
		return ret;

	ret = lvts_register_thermal_zones(lvts_data);
	if (ret)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(lvts_probe);

int lvts_remove(struct platform_device *pdev)
{
	struct lvts_data *lvts_data;

	lvts_data = (struct lvts_data *)platform_get_drvdata(pdev);

	lvts_close(lvts_data);

	return 0;
}
EXPORT_SYMBOL_GPL(lvts_remove);

int lvts_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct lvts_data *lvts_data;

	lvts_data = (struct lvts_data *)platform_get_drvdata(pdev);

	lvts_close(lvts_data);

	return 0;
}
EXPORT_SYMBOL_GPL(lvts_suspend);

int lvts_resume(struct platform_device *pdev)
{
	int ret;
	struct lvts_data *lvts_data;

	lvts_data = (struct lvts_data *)platform_get_drvdata(pdev);

	ret = lvts_init(lvts_data);
	if (ret)
		return ret;

	return 0;
}
EXPORT_SYMBOL_GPL(lvts_resume);

MODULE_AUTHOR("Yu-Chia Chang <ethan.chang@mediatek.com>");
MODULE_AUTHOR("Michael Kao <michael.kao@mediatek.com>");
MODULE_DESCRIPTION("MediaTek soc temperature driver");
MODULE_LICENSE("GPL v2");
