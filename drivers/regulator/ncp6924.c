/* drivers/regulator/ncp6924.c
 *
 * Copyright (C) 2013 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/of_gpio.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/i2c.h>
#include <linux/regulator/ncp6924.h>
#include <mach/devices_cmdline.h>

enum {
	VREG_TYPE_DCDC,
	VREG_TYPE_LDO,
	VREG_TYPE_MAX
};

struct vreg_type_lookup_table {
	uint32_t type;
	const char *type_name;
};

struct ncp6924_vreg {
	struct device *dev;
	struct list_head reg_list;
	struct regulator_desc rdesc;
	struct regulator_dev *rdev;
	struct regulator_init_data *init_data;
	const char *regulator_name;
	u32 resource_id;
	int regulator_type;
	u32 enable_addr;
	u32 enable_bit;
	u32 base_addr;
	u32 use_count;
	struct mutex mlock;
	u32 inited;
	bool always_on;
};

struct ncp6924_regulator {
	struct device *dev;
	struct ncp6924_vreg *ncp6924_vregs;
	int en_gpio;
	int total_vregs;
	bool is_enable;
};

#define LDO_UV_VMIN             1000000
#define LDO_UV_STEP             50000
#define DCDC_UV_VMIN            600000
#define DCDC_UV_STEP            12500

static int ncp6924_enable(struct ncp6924_regulator *reg, bool enable)
{
	int ret = 0;

	gpio_direction_output(reg->en_gpio, 1);
	if (enable) {
		gpio_set_value(reg->en_gpio, 1);
		reg->is_enable = true;
	} else {
		gpio_set_value(reg->en_gpio, 0);
		reg->is_enable = false;
	}

	return ret;
}

static bool ncp6924_is_enable(struct ncp6924_regulator *reg)
{
	return reg->is_enable;
}

static int ncp6924_i2c_write(struct device *dev, u8 reg_addr, u8 data)
{
	int res;
	int orig_state;
	struct i2c_client *client = to_i2c_client(dev);
	struct ncp6924_regulator *reg = i2c_get_clientdata(client);
	u8 values[] = {reg_addr, data};

	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= values,
		},
	};

	
	orig_state = ncp6924_is_enable(reg);
	if (orig_state == false)
		ncp6924_enable(reg, true);
	res = i2c_transfer(client->adapter, msg, 1);
	
	
	
	if ((orig_state == false && !(reg_addr == NCP6924_ENABLE && data != 0x00)) ||
	    (reg_addr == NCP6924_ENABLE && data == 0x00)) {
		ncp6924_enable(reg, false);
	}
	
	if (res > 0)
		res = 0;

	return res;
}

static int ncp6924_i2c_read(struct device *dev, u8 reg_addr, u8 *data)
{
	int res;
	int curr_state;
	struct i2c_client *client = to_i2c_client(dev);
	struct ncp6924_regulator *reg = i2c_get_clientdata(client);

	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &reg_addr,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= data,
		},
	};

	
	curr_state = ncp6924_is_enable(reg);
	if (curr_state == 0)
		ncp6924_enable(reg, true);
	res = i2c_transfer(client->adapter, msg, 2);
	
	if (curr_state == 0)
		ncp6924_enable(reg, false);
	
	if (res >= 0)
		res = 0;

	return res;
}

static int ncp6924_vreg_is_enabled(struct regulator_dev *rdev)
{
	struct ncp6924_vreg *vreg = rdev_get_drvdata(rdev);
	struct device *dev = vreg->dev;
	uint8_t val = 0;
	int rc = 0;

	if (vreg->inited != 1) {
		pr_info("%s: vreg not inited ready\n", __func__);
		return -1;
	}
	mutex_lock(&vreg->mlock);
	rc = ncp6924_i2c_read(dev, vreg->enable_addr, &val);
	mutex_unlock(&vreg->mlock);

	return ((val & (1 << vreg->enable_bit))? 1: 0);
}

static int ncp6924_vreg_enable(struct regulator_dev *rdev)
{
	struct ncp6924_vreg *vreg = rdev_get_drvdata(rdev);
	struct device *dev = vreg->dev;
	uint8_t val = 0;
	int rc = 0;

	mutex_lock(&vreg->mlock);
	rc = ncp6924_i2c_read(dev, vreg->enable_addr, &val);
	val |= (1 << vreg->enable_bit);
	rc = ncp6924_i2c_write(dev, vreg->enable_addr, val);
	mutex_unlock(&vreg->mlock);

	return rc;
}

static int ncp6924_vreg_disable(struct regulator_dev *rdev)
{
	struct ncp6924_vreg *vreg = rdev_get_drvdata(rdev);
	struct device *dev = vreg->dev;
	uint8_t val = 0;
	int rc = 0;

	mutex_lock(&vreg->mlock);
	rc = ncp6924_i2c_read(dev, vreg->enable_addr, &val);
	val &= ~(1 << vreg->enable_bit);
	rc = ncp6924_i2c_write(dev, vreg->enable_addr, val);
	mutex_unlock(&vreg->mlock);

	return rc;
}

static int ncp6924_vreg_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV,
			    unsigned *selector)
{
	struct ncp6924_vreg *vreg = rdev_get_drvdata(rdev);
	struct device *dev = vreg->dev;
	uint8_t uv_step = 0;
	int rc = 0;

	mutex_lock(&vreg->mlock);
	if (vreg->regulator_type == VREG_TYPE_LDO) {
		uv_step = (max_uV - LDO_UV_VMIN) / LDO_UV_STEP;
		ncp6924_i2c_write(dev, vreg->base_addr, uv_step);
	} else if (vreg->regulator_type == VREG_TYPE_DCDC) {
	        uv_step = (max_uV - DCDC_UV_VMIN) / DCDC_UV_STEP;
	        ncp6924_i2c_write(dev, vreg->base_addr, uv_step);
	} else
		pr_err("%s: non support vreg type %d\n", __func__, vreg->regulator_type);
	mutex_unlock(&vreg->mlock);

	return rc;
}

static int ncp6924_vreg_get_voltage(struct regulator_dev *rdev)
{
	struct ncp6924_vreg *vreg = rdev_get_drvdata(rdev);
	struct device *dev = vreg->dev;
	uint8_t val = 0;
	uint32_t vol = 0;

	if (vreg->inited != 1) {
		pr_info("%s: vreg not inited ready\n", __func__);
		return -1;
	}

	mutex_lock(&vreg->mlock);
	ncp6924_i2c_read(dev, vreg->base_addr, &val);
	if (vreg->regulator_type == VREG_TYPE_LDO)
		vol = val * LDO_UV_STEP + LDO_UV_VMIN;
	else if (vreg->regulator_type == VREG_TYPE_DCDC)
		vol = val * DCDC_UV_STEP + DCDC_UV_VMIN;
	else
		pr_err("%s: non support vreg type %d\n", __func__, vreg->regulator_type);
	mutex_unlock(&vreg->mlock);

	return vol;
}

static struct regulator_ops ncp6924_vreg_ops = {
	.enable		= ncp6924_vreg_enable,
	.disable	= ncp6924_vreg_disable,
	.is_enabled	= ncp6924_vreg_is_enabled,
	.set_voltage	= ncp6924_vreg_set_voltage,
	.get_voltage	= ncp6924_vreg_get_voltage,
};

static struct regulator_ops *vreg_ops[] = {
	[VREG_TYPE_DCDC]	= &ncp6924_vreg_ops,
	[VREG_TYPE_LDO]		= &ncp6924_vreg_ops,
};

static int __devinit ncp6924_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct device_node *child = NULL;
	struct ncp6924_vreg *ncp6924_vreg = NULL;
	struct ncp6924_regulator *reg;
	struct regulator_init_data *init_data;
	int en_gpio = 0;
	int num_vregs = 0;
	int vreg_idx = 0;
	int ret = 0;

	pr_info("%s.\n", __func__);

#ifdef CONFIG_REGULATOR_NCP6924_OFFMODE_CHARGE
	if (board_mfg_mode() == MFG_MODE_OFFMODE_CHARGING){

		dev_err(dev, "%s: offmode charge, ignore ncp6924 probe.\n", __func__);
		return -EACCES;
	}
#endif
	if (!dev->of_node) {
		dev_err(dev, "%s: device tree information missing\n", __func__);
		return -ENODEV;
	}

	reg = kzalloc(sizeof(struct ncp6924_regulator), GFP_KERNEL);
	if (reg == NULL) {
		dev_err(dev, "%s: could not allocate memory for reg.\n", __func__);
		return -ENOMEM;
	}

	reg->dev = dev;
	en_gpio = of_get_named_gpio(node, "ncp,enable-gpio", 0);
	if (gpio_is_valid(en_gpio)) {
		pr_info("%s: Read NCP6924_enable gpio: %d\n", __func__, en_gpio);
		reg->en_gpio = en_gpio;
		gpio_request(reg->en_gpio, "NCP6924_EN_GPIO");
	} else {
		pr_err("%s: Fail to read NCP6924_enable gpio: %d\n", __func__, en_gpio);
		goto fail_free_regulator;
	}

	
	for_each_child_of_node(node, child)
		num_vregs++;
	reg->total_vregs = num_vregs;

	reg->ncp6924_vregs = kzalloc(sizeof(struct ncp6924_vreg) * num_vregs, GFP_KERNEL);
	if (reg->ncp6924_vregs == NULL) {
		dev_err(dev, "%s: could not allocate memory for ncp6924_vreg\n", __func__);
		return -ENOMEM;
	}

	
	for_each_child_of_node(node, child) {
		ncp6924_vreg = &reg->ncp6924_vregs[vreg_idx++];
		ret = of_property_read_string(child, "regulator-name",
				&ncp6924_vreg->regulator_name);
		if (ret) {
			dev_err(dev, "%s: regulator-name missing in DT node\n", __func__);
			goto fail_free_vreg;
		}

		ret = of_property_read_u32(child, "ncp,resource-id",
				&ncp6924_vreg->resource_id);
		if (ret) {
			dev_err(dev, "%s: ncp,resource-id missing in DT node\n", __func__);
			goto fail_free_vreg;
		}

		ret = of_property_read_u32(child, "ncp,regulator-type",
				&ncp6924_vreg->regulator_type);
		if (ret) {
			dev_err(dev, "%s: ncp,regulator-type missing in DT node\n", __func__);
			goto fail_free_vreg;
		}

		if ((ncp6924_vreg->regulator_type < 0)
		    || (ncp6924_vreg->regulator_type >= VREG_TYPE_MAX)) {
			dev_err(dev, "%s: invalid regulator type: %d\n", __func__, ncp6924_vreg->regulator_type);
			ret = -EINVAL;
			goto fail_free_vreg;
		}
		ncp6924_vreg->rdesc.ops = vreg_ops[ncp6924_vreg->regulator_type];

		ret = of_property_read_u32(child, "ncp,enable-addr",
				&ncp6924_vreg->enable_addr);
		if (ret) {
			dev_err(dev, "%s: Fail to get vreg enable address.\n", __func__);
			goto fail_free_vreg;
		}

		ret = of_property_read_u32(child, "ncp,enable-bit",
				&ncp6924_vreg->enable_bit);
		if (ret) {
			dev_err(dev, "%s: Fail to get vreg enable bit.\n", __func__);
			goto fail_free_vreg;
		}

		ret = of_property_read_u32(child, "ncp,base-addr",
				&ncp6924_vreg->base_addr);
		if (ret) {
			dev_err(dev, "%s: Fail to get vreg base address.\n", __func__);
			goto fail_free_vreg;
		}

		if (of_property_read_bool(child, "ldo-always-on"))
			ncp6924_vreg->always_on = true;
		else
			ncp6924_vreg->always_on = false;

		init_data = of_get_regulator_init_data(dev, child);
		if (init_data == NULL) {
			dev_err(dev, "%s: unable to allocate memory\n", __func__);
			ret = -ENOMEM;
			goto fail_free_vreg;
		}

		if (init_data->constraints.name == NULL) {
			dev_err(dev, "%s: regulator name not specified\n", __func__);
			ret = -EINVAL;
			goto fail_free_vreg;
		}

		if (of_property_read_bool(child, "parent-supply"))
			init_data->supply_regulator = "ncp6924_ldo";

		ncp6924_vreg->rdesc.name 	= init_data->constraints.name;
		ncp6924_vreg->dev		= dev;
		init_data->constraints.valid_ops_mask |= REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE;

		INIT_LIST_HEAD(&ncp6924_vreg->reg_list);
		mutex_init(&ncp6924_vreg->mlock);
		ncp6924_vreg->rdev = regulator_register(&ncp6924_vreg->rdesc, dev, init_data, ncp6924_vreg, child);
		if (IS_ERR(ncp6924_vreg->rdev)) {
			ret = PTR_ERR(ncp6924_vreg->rdev);
			ncp6924_vreg->rdev = NULL;
			pr_err("%s: regulator register failed: %s, ret = %d\n", __func__, ncp6924_vreg->rdesc.name, ret);
			goto fail_free_vreg;
		}
		ncp6924_vreg->inited = 1;
	}
	i2c_set_clientdata(client, reg);
	ret = ncp6924_enable(reg, true);

	return ret;

fail_free_vreg:
	kfree(reg->ncp6924_vregs);

fail_free_regulator:
	kfree(reg);
	return ret;
}

static int ncp6924_suspend(struct i2c_client *client, pm_message_t state)
{
	struct ncp6924_regulator *reg;
	int total_vreg_num = 0, on_vreg_num = 0;
	int idx = 0;

	reg = i2c_get_clientdata(client);
	total_vreg_num = reg->total_vregs;

	for (idx = 0; idx < total_vreg_num; idx++) {
		if (reg->ncp6924_vregs[idx].always_on)
			on_vreg_num++;
		else
			ncp6924_vreg_disable(reg->ncp6924_vregs[idx].rdev);
	}

	if (on_vreg_num == 0)
		ncp6924_enable(reg, false);

	return 0;
}

static int ncp6924_resume(struct i2c_client *client)
{
	struct ncp6924_regulator *reg;

	reg = i2c_get_clientdata(client);
	if (ncp6924_is_enable(reg) == false)
		ncp6924_enable(reg, true);

	return 0;
}

static int __devexit ncp6924_remove(struct i2c_client *client)
{
	struct ncp6924_regulator *reg;

	reg = i2c_get_clientdata(client);
	kfree(reg->ncp6924_vregs);
	kfree(reg);

	return 0;
}

static struct of_device_id ncp6924_match_table[] = {
	{.compatible = "htc,ncp6924-regulator"},
	{},
};

static const struct i2c_device_id ncp6924_id[] = {
	{"ncp6924", 0},
	{},
};

static struct i2c_driver ncp6924_driver = {
	.driver = {
		.name		= "ncp6924",
		.owner		= THIS_MODULE,
		.of_match_table	= ncp6924_match_table,
	},
	.probe		= ncp6924_probe,
	.remove		= __devexit_p(ncp6924_remove),
	.suspend	= ncp6924_suspend,
	.resume		= ncp6924_resume,
	.id_table	= ncp6924_id,
};

int __init ncp6924_regulator_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&ncp6924_driver);
	if (ret)
		pr_err("%s: Driver registration failed\n", __func__);

	return ret;
}
EXPORT_SYMBOL(ncp6924_regulator_init);

static void __exit ncp6924_regulator_exit(void)
{
	i2c_del_driver(&ncp6924_driver);
}

MODULE_AUTHOR("Kenny Liu <kenny_liu@htc.com>");
MODULE_DESCRIPTION("NCP6924 regulator driver");
MODULE_LICENSE("GPL v2");

#ifdef CONFIG_ARCH_MSM8974
module_init(ncp6924_regulator_init);
#else
arch_initcall(ncp6924_regulator_init);
#endif
module_exit(ncp6924_regulator_exit);
