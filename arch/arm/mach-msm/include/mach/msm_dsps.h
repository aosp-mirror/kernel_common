/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _MSM_DSPS_H_
#define _MSM_DSPS_H_

#include <linux/types.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>

#define DSPS_SIGNATURE	0x12345678

/**
 * DSPS Clocks Platform data.
 *
 * @name - clock name.
 * @rate - rate to set. zero if not relevant.
 * @clock - clock handle, reserved for the driver.
 */
struct dsps_clk_info {
	const char *name;
	u32 rate;
	struct clk *clock;
};

/**
 * DSPS GPIOs Platform data.
 *
 * @name - clock name.
 * @num - GPIO number.
 * @on_val - value to ouptput for ON (depends on polarity).
 * @off_val - value to ouptput for OFF (depends on polarity).
 * @is_owner - reserved for the driver.
 */
struct dsps_gpio_info {
	const char *name;
	int num;
	int on_val;
	int off_val;
	int is_owner;
};

/**
 * DSPS Power regulators Platform data.
 *
 * @name - regulator name.
 * @volt - required voltage (in uV).
 * @reg - reserved for the driver.
 */
struct dsps_regulator_info {
	const char *name;
	int volt;
	struct regulator *reg;
};

/**
 * DSPS Platform data.
 *
 * @pil_name - peripheral image name
 * @clks - array of clocks.
 * @clks_num - number of clocks in array.
 * @gpios - array of gpios.
 * @gpios_num - number of gpios.
 * @regs - array of regulators.
 * @regs_num - number of regulators.
 * @dsps_pwr_ctl_en - to enable DSPS to do power control if set 1
 *  otherwise the apps will do power control
 * @ppss_pause_reg - Offset to the PPSS_PAUSE register
 * @ppss_wdog_unmasked_int_en_reg - Offset to PPSS_WDOG_UNMASKED_INT_EN register
 * @signature - signature for validity check.
 */
struct msm_dsps_platform_data {
	const char *pil_name;
	struct dsps_clk_info *clks;
	int clks_num;
	struct dsps_gpio_info *gpios;
	int gpios_num;
	struct dsps_regulator_info *regs;
	int regs_num;
	int dsps_pwr_ctl_en;
	void (*init)(struct msm_dsps_platform_data *data);
	int ppss_pause_reg;
	int ppss_wdog_unmasked_int_en_reg;
	u32 signature;
};

#endif /* _MSM_DSPS_H_ */
