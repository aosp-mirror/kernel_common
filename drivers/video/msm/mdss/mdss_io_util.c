/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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
#include <linux/err.h>
#include <linux/io.h>
#include <linux/delay.h>
#include "mdss_io_util.h"

#define MAX_I2C_CMDS  16
void dss_reg_w(struct dss_io_data *io, u32 offset, u32 value, u32 debug)
{
	u32 in_val;

	if (!io || !io->base) {
		DEV_ERR("%pS->%s: invalid input\n",
			__builtin_return_address(0), __func__);
		return;
	}

	if (offset > io->len) {
		DEV_ERR("%pS->%s: offset out of range\n",
			__builtin_return_address(0), __func__);
		return;
	}

	writel_relaxed(value, io->base + offset);
	if (debug) {
		in_val = readl_relaxed(io->base + offset);
		DEV_DBG("[%08x] => %08x [%08x]\n", (u32)(io->base + offset),
			value, in_val);
	}
} /* dss_reg_w */

u32 dss_reg_r(struct dss_io_data *io, u32 offset, u32 debug)
{
	u32 value;
	if (!io || !io->base) {
		DEV_ERR("%pS->%s: invalid input\n",
			__builtin_return_address(0), __func__);
		return -EINVAL;
	}

	if (offset > io->len) {
		DEV_ERR("%pS->%s: offset out of range\n",
			__builtin_return_address(0), __func__);
		return -EINVAL;
	}

	value = readl_relaxed(io->base + offset);
	if (debug)
		DEV_DBG("[%08x] <= %08x\n", (u32)(io->base + offset), value);

	return value;
} /* dss_reg_r */

void dss_reg_dump(void __iomem *base, u32 length, const char *prefix,
	u32 debug)
{
	if (debug)
		print_hex_dump(KERN_INFO, prefix, DUMP_PREFIX_OFFSET, 32, 4,
			(void *)base, length, false);
} /* dss_reg_dump */

static struct resource *msm_dss_get_res_byname(struct platform_device *pdev,
	unsigned int type, const char *name)
{
	struct resource *res = NULL;

	res = platform_get_resource_byname(pdev, type, name);
	if (!res)
		DEV_ERR("%s: '%s' resource not found\n", __func__, name);

	return res;
} /* msm_dss_get_res_byname */

int msm_dss_ioremap_byname(struct platform_device *pdev,
	struct dss_io_data *io_data, const char *name)
{
	struct resource *res = NULL;

	if (!pdev || !io_data) {
		DEV_ERR("%pS->%s: invalid input\n",
			__builtin_return_address(0), __func__);
		return -EINVAL;
	}

	res = msm_dss_get_res_byname(pdev, IORESOURCE_MEM, name);
	if (!res) {
		DEV_ERR("%pS->%s: '%s' msm_dss_get_res_byname failed\n",
			__builtin_return_address(0), __func__, name);
		return -ENODEV;
	}

	io_data->len = resource_size(res);
	io_data->base = ioremap(res->start, io_data->len);
	if (!io_data->base) {
		DEV_ERR("%pS->%s: '%s' ioremap failed\n",
			__builtin_return_address(0), __func__, name);
		return -EIO;
	}

	return 0;
} /* msm_dss_ioremap_byname */

void msm_dss_iounmap(struct dss_io_data *io_data)
{
	if (!io_data) {
		DEV_ERR("%pS->%s: invalid input\n",
			__builtin_return_address(0), __func__);
		return;
	}

	if (io_data->base) {
		iounmap(io_data->base);
		io_data->base = NULL;
	}
	io_data->len = 0;
} /* msm_dss_iounmap */

int msm_dss_config_vreg(struct device *dev, struct dss_vreg *in_vreg,
	int num_vreg, int config)
{
	int i = 0, rc = 0;
	struct dss_vreg *curr_vreg = NULL;
	enum dss_vreg_type type;

	if (config) {
		for (i = 0; i < num_vreg; i++) {
			curr_vreg = &in_vreg[i];
			curr_vreg->vreg = regulator_get(dev,
				curr_vreg->vreg_name);
			rc = PTR_RET(curr_vreg->vreg);
			if (rc) {
				DEV_ERR("%pS->%s: %s get failed. rc=%d\n",
					 __builtin_return_address(0), __func__,
					 curr_vreg->vreg_name, rc);
				curr_vreg->vreg = NULL;
				goto vreg_get_fail;
			}
			type = (regulator_count_voltages(curr_vreg->vreg) > 0)
					? DSS_REG_LDO : DSS_REG_VS;
			if (type == DSS_REG_LDO) {
				rc = regulator_set_voltage(
					curr_vreg->vreg,
					curr_vreg->min_voltage,
					curr_vreg->max_voltage);
				if (rc < 0) {
					DEV_ERR("%pS->%s: %s set vltg fail\n",
						__builtin_return_address(0),
						__func__,
						curr_vreg->vreg_name);
					goto vreg_set_voltage_fail;
				}
			}
		}
	} else {
		for (i = num_vreg-1; i >= 0; i--) {
			curr_vreg = &in_vreg[i];
			if (curr_vreg->vreg) {
				type = (regulator_count_voltages(
					curr_vreg->vreg) > 0)
					? DSS_REG_LDO : DSS_REG_VS;
				if (type == DSS_REG_LDO) {
					regulator_set_voltage(curr_vreg->vreg,
						0, curr_vreg->max_voltage);
				}
				regulator_put(curr_vreg->vreg);
				curr_vreg->vreg = NULL;
			}
		}
	}
	return 0;

vreg_unconfig:
if (type == DSS_REG_LDO)
	regulator_set_optimum_mode(curr_vreg->vreg, 0);

vreg_set_voltage_fail:
	regulator_put(curr_vreg->vreg);
	curr_vreg->vreg = NULL;

vreg_get_fail:
	for (i--; i >= 0; i--) {
		curr_vreg = &in_vreg[i];
		type = (regulator_count_voltages(curr_vreg->vreg) > 0)
			? DSS_REG_LDO : DSS_REG_VS;
		goto vreg_unconfig;
	}
	return rc;
} /* msm_dss_config_vreg */

int msm_dss_enable_vreg(struct dss_vreg *in_vreg, int num_vreg, int enable)
{
	int i = 0, rc = 0;
	if (enable) {
		for (i = 0; i < num_vreg; i++) {
			rc = PTR_RET(in_vreg[i].vreg);
			if (rc) {
				DEV_ERR("%pS->%s: %s regulator error. rc=%d\n",
					__builtin_return_address(0), __func__,
					in_vreg[i].vreg_name, rc);
				goto vreg_set_opt_mode_fail;
			}
			if (in_vreg[i].pre_on_sleep)
				msleep(in_vreg[i].pre_on_sleep);
			rc = regulator_set_optimum_mode(in_vreg[i].vreg,
				in_vreg[i].enable_load);
			if (rc < 0) {
				DEV_ERR("%pS->%s: %s set opt m fail\n",
					__builtin_return_address(0), __func__,
					in_vreg[i].vreg_name);
				goto vreg_set_opt_mode_fail;
			}
			rc = regulator_enable(in_vreg[i].vreg);
			if (in_vreg[i].post_on_sleep)
				msleep(in_vreg[i].post_on_sleep);
			if (rc < 0) {
				DEV_ERR("%pS->%s: %s enable failed\n",
					__builtin_return_address(0), __func__,
					in_vreg[i].vreg_name);
				goto disable_vreg;
			}
		}
	} else {
		for (i = num_vreg-1; i >= 0; i--)
			if (regulator_is_enabled(in_vreg[i].vreg)) {
				if (in_vreg[i].pre_off_sleep)
					msleep(in_vreg[i].pre_off_sleep);
				regulator_set_optimum_mode(in_vreg[i].vreg,
					in_vreg[i].disable_load);
				regulator_disable(in_vreg[i].vreg);
				if (in_vreg[i].post_off_sleep)
					msleep(in_vreg[i].post_off_sleep);
			}
	}
	return rc;

disable_vreg:
	regulator_set_optimum_mode(in_vreg[i].vreg, in_vreg[i].disable_load);

vreg_set_opt_mode_fail:
	for (i--; i >= 0; i--) {
		if (in_vreg[i].pre_off_sleep)
			msleep(in_vreg[i].pre_off_sleep);
		regulator_set_optimum_mode(in_vreg[i].vreg,
			in_vreg[i].disable_load);
		regulator_disable(in_vreg[i].vreg);
		if (in_vreg[i].post_off_sleep)
			msleep(in_vreg[i].post_off_sleep);
	}

	return rc;
} /* msm_dss_enable_vreg */

int msm_dss_enable_gpio(struct dss_gpio *in_gpio, int num_gpio, int enable)
{
	int i = 0, rc = 0;
	if (enable) {
		for (i = 0; i < num_gpio; i++) {
			DEV_DBG("%pS->%s: %s enable\n",
				__builtin_return_address(0), __func__,
				in_gpio[i].gpio_name);

			rc = gpio_request(in_gpio[i].gpio,
				in_gpio[i].gpio_name);
			if (rc < 0) {
				DEV_ERR("%pS->%s: %s enable failed\n",
					__builtin_return_address(0), __func__,
					in_gpio[i].gpio_name);
				goto disable_gpio;
			}
			gpio_set_value(in_gpio[i].gpio, in_gpio[i].value);
		}
	} else {
		for (i = num_gpio-1; i >= 0; i--) {
			DEV_DBG("%pS->%s: %s disable\n",
				__builtin_return_address(0), __func__,
				in_gpio[i].gpio_name);
			if (in_gpio[i].gpio)
				gpio_free(in_gpio[i].gpio);
		}
	}
	return rc;

disable_gpio:
	for (i--; i >= 0; i--)
		if (in_gpio[i].gpio)
			gpio_free(in_gpio[i].gpio);

	return rc;
} /* msm_dss_enable_gpio */

void msm_dss_put_clk(struct dss_clk *clk_arry, int num_clk)
{
	int i;

	for (i = num_clk - 1; i >= 0; i--) {
		if (clk_arry[i].clk)
			clk_put(clk_arry[i].clk);
		clk_arry[i].clk = NULL;
	}
} /* msm_dss_put_clk */

int msm_dss_get_clk(struct device *dev, struct dss_clk *clk_arry, int num_clk)
{
	int i, rc = 0;

	for (i = 0; i < num_clk; i++) {
		clk_arry[i].clk = clk_get(dev, clk_arry[i].clk_name);
		rc = PTR_RET(clk_arry[i].clk);
		if (rc) {
			DEV_ERR("%pS->%s: '%s' get failed. rc=%d\n",
				__builtin_return_address(0), __func__,
				clk_arry[i].clk_name, rc);
			goto error;
		}
	}

	return rc;

error:
	msm_dss_put_clk(clk_arry, num_clk);

	return rc;
} /* msm_dss_get_clk */

int msm_dss_clk_set_rate(struct dss_clk *clk_arry, int num_clk)
{
	int i, rc = 0;

	for (i = 0; i < num_clk; i++) {
		if (clk_arry[i].clk) {
			if (DSS_CLK_AHB != clk_arry[i].type) {
				DEV_DBG("%pS->%s: '%s' rate %ld\n",
					__builtin_return_address(0), __func__,
					clk_arry[i].clk_name,
					clk_arry[i].rate);
				rc = clk_set_rate(clk_arry[i].clk,
					clk_arry[i].rate);
				if (rc) {
					DEV_ERR("%pS->%s: %s failed. rc=%d\n",
						__builtin_return_address(0),
						__func__,
						clk_arry[i].clk_name, rc);
					break;
				}
			}
		} else {
			DEV_ERR("%pS->%s: '%s' is not available\n",
				__builtin_return_address(0), __func__,
				clk_arry[i].clk_name);
			rc = -EPERM;
			break;
		}
	}

	return rc;
} /* msm_dss_clk_set_rate */

int msm_dss_enable_clk(struct dss_clk *clk_arry, int num_clk, int enable)
{
	int i, rc = 0;

	if (enable) {
		for (i = 0; i < num_clk; i++) {
			DEV_DBG("%pS->%s: enable '%s'\n",
				__builtin_return_address(0), __func__,
				clk_arry[i].clk_name);
			if (clk_arry[i].clk) {
				rc = clk_prepare_enable(clk_arry[i].clk);
				if (rc)
					DEV_ERR("%pS->%s: %s en fail. rc=%d\n",
						__builtin_return_address(0),
						__func__,
						clk_arry[i].clk_name, rc);
			} else {
				DEV_ERR("%pS->%s: '%s' is not available\n",
					__builtin_return_address(0), __func__,
					clk_arry[i].clk_name);
				rc = -EPERM;
			}

			if (rc) {
				msm_dss_enable_clk(&clk_arry[i],
					i, false);
				break;
			}
		}
	} else {
		for (i = num_clk - 1; i >= 0; i--) {
			DEV_DBG("%pS->%s: disable '%s'\n",
				__builtin_return_address(0), __func__,
				clk_arry[i].clk_name);

			if (clk_arry[i].clk)
				clk_disable_unprepare(clk_arry[i].clk);
			else
				DEV_ERR("%pS->%s: '%s' is not available\n",
					__builtin_return_address(0), __func__,
					clk_arry[i].clk_name);
		}
	}

	return rc;
} /* msm_dss_enable_clk */


int mdss_i2c_byte_read(struct i2c_client *client, uint8_t slave_addr,
			uint8_t reg_offset, uint8_t *read_buf)
{
	struct i2c_msg msgs[2];
	int ret = -1;

	pr_debug("%s: reading from slave_addr=[%x] and offset=[%x]\n",
		 __func__, slave_addr, reg_offset);

	msgs[0].addr = slave_addr >> 1;
	msgs[0].flags = 0;
	msgs[0].buf = &reg_offset;
	msgs[0].len = 1;

	msgs[1].addr = slave_addr >> 1;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = read_buf;
	msgs[1].len = 1;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 1) {
		pr_err("%s: I2C READ FAILED=[%d]\n", __func__, ret);
		return -EACCES;
	}
	pr_debug("%s: i2c buf is [%x]\n", __func__, *read_buf);
	return 0;
}

int mdss_i2c_byte_write(struct i2c_client *client, uint8_t slave_addr,
			uint8_t reg_offset, uint8_t *value)
{
	struct i2c_msg msgs[1];
	uint8_t data[2];
	int status = -EACCES;

	pr_debug("%s: writing from slave_addr=[%x] and offset=[%x]\n",
		 __func__, slave_addr, reg_offset);

	data[0] = reg_offset;
	data[1] = *value;

	msgs[0].addr = slave_addr >> 1;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = data;

	status = i2c_transfer(client->adapter, msgs, 1);
	if (status < 1) {
		pr_err("I2C WRITE FAILED=[%d]\n", status);
		return -EACCES;
	}
	pr_debug("%s: I2C write status=%x\n", __func__, status);
	return status;
}
