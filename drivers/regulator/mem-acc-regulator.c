/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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
#define pr_fmt(fmt)	"ACC: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#define MEM_ACC_SEL_MASK	0x3

enum {
	MEMORY_L1,
	MEMORY_L2,
	MEMORY_MAX,
};

struct mem_acc_regulator {
	struct device		*dev;
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;

	int			corner;
	bool			mem_acc_supported[MEMORY_MAX];

	u32			acc_sel_reg[MEMORY_MAX];
	u32			*acc_sel_mask[MEMORY_MAX];
	u32			*acc_sel_bit_pos[MEMORY_MAX];
	u32			num_acc_sel[MEMORY_MAX];
	u32			*acc_en_bit_pos;
	u32			num_acc_en;
	u32			*corner_acc_map;
	u32			num_corners;

	void __iomem		*acc_sel_base[MEMORY_MAX];
	void __iomem		*acc_en_base;
	phys_addr_t		acc_sel_addr[MEMORY_MAX];
	phys_addr_t		acc_en_addr;
};

static inline u32 apc_to_acc_corner(struct mem_acc_regulator *mem_acc_vreg,
								int corner)
{
	/*
	 * corner_acc_map maps the corner from index 0 and  APC corner value
	 * starts from the value 1
	 */
	return mem_acc_vreg->corner_acc_map[corner - 1];
}

static void __update_acc_sel(struct mem_acc_regulator *mem_acc_vreg,
						int corner, int mem_type)
{
	u32 acc_data, i, bit, acc_corner;

	acc_data = mem_acc_vreg->acc_sel_reg[mem_type];
	for (i = 0; i < mem_acc_vreg->num_acc_sel[mem_type]; i++) {
		bit = mem_acc_vreg->acc_sel_bit_pos[mem_type][i];
		acc_data &= ~mem_acc_vreg->acc_sel_mask[mem_type][i];
		acc_corner = apc_to_acc_corner(mem_acc_vreg, corner);
		acc_data |= (acc_corner << bit) &
			mem_acc_vreg->acc_sel_mask[mem_type][i];
	}
	pr_debug("corner=%d old_acc_sel=0x%02x new_acc_sel=0x%02x mem_type=%d\n",
			corner, mem_acc_vreg->acc_sel_reg[mem_type],
						acc_data, mem_type);
	writel_relaxed(acc_data, mem_acc_vreg->acc_sel_base[mem_type]);
	mem_acc_vreg->acc_sel_reg[mem_type] = acc_data;
}

static void update_acc_sel(struct mem_acc_regulator *mem_acc_vreg, int corner)
{
	int i;

	for (i = 0; i < MEMORY_MAX; i++) {
		if (mem_acc_vreg->mem_acc_supported[i])
			__update_acc_sel(mem_acc_vreg, corner, i);
	}
}

static int mem_acc_regulator_set_voltage(struct regulator_dev *rdev,
		int corner, int corner_max, unsigned *selector)
{
	struct mem_acc_regulator *mem_acc_vreg = rdev_get_drvdata(rdev);
	int i;

	if (corner > mem_acc_vreg->num_corners) {
		pr_err("Invalid corner=%d requested\n", corner);
		return -EINVAL;
	}

	pr_debug("old corner=%d, new corner=%d\n",
			mem_acc_vreg->corner, corner);

	if (corner == mem_acc_vreg->corner)
		return 0;

	/* go up or down one level at a time */
	if (corner > mem_acc_vreg->corner) {
		for (i = mem_acc_vreg->corner + 1; i <= corner; i++) {
			pr_debug("UP: to corner %d\n", i);
			update_acc_sel(mem_acc_vreg, i);
		}
	} else {
		for (i = mem_acc_vreg->corner - 1; i >= corner; i--) {
			pr_debug("DOWN: to corner %d\n", i);
			update_acc_sel(mem_acc_vreg, i);
		}
	}

	pr_debug("new voltage corner set %d\n", corner);

	mem_acc_vreg->corner = corner;

	return 0;
}

static int mem_acc_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct mem_acc_regulator *mem_acc_vreg = rdev_get_drvdata(rdev);

	return mem_acc_vreg->corner;
}

static struct regulator_ops mem_acc_corner_ops = {
	.set_voltage		= mem_acc_regulator_set_voltage,
	.get_voltage		= mem_acc_regulator_get_voltage,
};

static int __mem_acc_sel_init(struct mem_acc_regulator *mem_acc_vreg,
							int mem_type)
{
	int i;
	u32 bit;

	mem_acc_vreg->acc_sel_mask[mem_type] = devm_kzalloc(mem_acc_vreg->dev,
		mem_acc_vreg->num_acc_sel[mem_type] * sizeof(u32), GFP_KERNEL);
	if (!mem_acc_vreg->acc_sel_mask[mem_type]) {
		pr_err("Unable to allocate memory for mem_type=%d\n", mem_type);
		return -ENOMEM;
	}

	for (i = 0; i < mem_acc_vreg->num_acc_sel[mem_type]; i++) {
		bit = mem_acc_vreg->acc_sel_bit_pos[mem_type][i];
		mem_acc_vreg->acc_sel_mask[mem_type][i] =
					MEM_ACC_SEL_MASK << bit;
	}

	mem_acc_vreg->acc_sel_reg[mem_type] =
		readl_relaxed(mem_acc_vreg->acc_sel_base[mem_type]);

	return 0;
}

static int mem_acc_sel_init(struct mem_acc_regulator *mem_acc_vreg)
{
	int i, rc;

	for (i = 0; i < MEMORY_MAX; i++) {
		if (mem_acc_vreg->mem_acc_supported[i]) {
			rc = __mem_acc_sel_init(mem_acc_vreg, i);
			if (rc) {
				pr_err("Unable to intialize mem_type=%d rc=%d\n",
								i, rc);
				return rc;
			}
		}
	}

	return 0;
}

static void mem_acc_en_init(struct mem_acc_regulator *mem_acc_vreg)
{
	int i, bit;
	u32 acc_data;

	acc_data = readl_relaxed(mem_acc_vreg->acc_en_base);
	pr_debug("init: acc_en_register=%x\n", acc_data);
	for (i = 0; i < mem_acc_vreg->num_acc_en; i++) {
		bit = mem_acc_vreg->acc_en_bit_pos[i];
		acc_data |= BIT(bit);
	}
	pr_debug("final: acc_en_register=%x\n", acc_data);
	writel_relaxed(acc_data, mem_acc_vreg->acc_en_base);
}

static int populate_acc_data(struct mem_acc_regulator *mem_acc_vreg,
			const char *prop_name, u32 **value, u32 *len)
{
	int rc;

	if (!of_get_property(mem_acc_vreg->dev->of_node, prop_name, len)) {
		pr_err("Unable to find %s property\n", prop_name);
		return -EINVAL;
	}
	*len /= sizeof(u32);
	if (!(*len)) {
		pr_err("Incorrect entries in %s\n", prop_name);
		return -EINVAL;
	}

	*value = devm_kzalloc(mem_acc_vreg->dev, (*len) * sizeof(u32),
							GFP_KERNEL);
	if (!(*value)) {
		pr_err("Unable to allocate memory for %s\n", prop_name);
		return -ENOMEM;
	}

	pr_debug("Found %s, data-length = %d\n", prop_name, *len);

	rc = of_property_read_u32_array(mem_acc_vreg->dev->of_node,
					prop_name, *value, *len);
	if (rc) {
		pr_err("Unable to populate %s rc=%d\n", prop_name, rc);
		return rc;
	}

	return 0;
}

static int mem_acc_sel_setup(struct mem_acc_regulator *mem_acc_vreg,
			struct resource *res, int mem_type)
{
	int len, rc;
	char *mem_select_str;

	mem_acc_vreg->acc_sel_addr[mem_type] = res->start;
	len = res->end - res->start + 1;
	pr_debug("'acc_sel_addr' = %pa mem_type=%d (len=%d)\n",
					&res->start, mem_type, len);

	mem_acc_vreg->acc_sel_base[mem_type] = devm_ioremap(mem_acc_vreg->dev,
			mem_acc_vreg->acc_sel_addr[mem_type], len);
	if (!mem_acc_vreg->acc_sel_base[mem_type]) {
		pr_err("Unable to map 'acc_sel_addr' %pa for mem_type=%d\n",
			&mem_acc_vreg->acc_sel_addr[mem_type], mem_type);
		return -EINVAL;
	}

	switch (mem_type) {
	case MEMORY_L1:
		mem_select_str = "qcom,acc-sel-l1-bit-pos";
		break;
	case MEMORY_L2:
		mem_select_str = "qcom,acc-sel-l2-bit-pos";
		break;
	}

	rc = populate_acc_data(mem_acc_vreg, mem_select_str,
			&mem_acc_vreg->acc_sel_bit_pos[mem_type],
			&mem_acc_vreg->num_acc_sel[mem_type]);
	if (rc)
		pr_err("Unable to populate '%s' rc=%d\n", mem_select_str, rc);

	return rc;
}

static int mem_acc_init(struct platform_device *pdev,
		struct mem_acc_regulator *mem_acc_vreg)
{
	struct resource *res;
	int len, rc, i;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "acc-en");
	if (!res || !res->start) {
		pr_debug("'acc-en' resource missing or not used.\n");
	} else {
		mem_acc_vreg->acc_en_addr = res->start;
		len = res->end - res->start + 1;
		pr_debug("'acc_en_addr' = %pa (len=0x%x)\n", &res->start, len);

		mem_acc_vreg->acc_en_base = devm_ioremap(mem_acc_vreg->dev,
				mem_acc_vreg->acc_en_addr, len);
		if (!mem_acc_vreg->acc_en_base) {
			pr_err("Unable to map 'acc_en_addr' %pa\n",
					&mem_acc_vreg->acc_en_addr);
			return -EINVAL;
		}

		rc = populate_acc_data(mem_acc_vreg, "qcom,acc-en-bit-pos",
				&mem_acc_vreg->acc_en_bit_pos,
				&mem_acc_vreg->num_acc_en);
		if (rc) {
			pr_err("Unable to populate 'qcom,acc-en-bit-pos' rc=%d\n",
					rc);
			return rc;
		}
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "acc-sel-l1");
	if (!res || !res->start) {
		pr_debug("'acc-sel-l1' resource missing or not used.\n");
	} else {
		rc = mem_acc_sel_setup(mem_acc_vreg, res, MEMORY_L1);
		if (rc) {
			pr_err("Unable to setup mem-acc for mem_type=%d rc=%d\n",
					MEMORY_L1, rc);
			return rc;
		}
		mem_acc_vreg->mem_acc_supported[MEMORY_L1] = true;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "acc-sel-l2");
	if (!res || !res->start) {
		pr_debug("'acc-sel-l2' resource missing or not used.\n");
	} else {
		rc = mem_acc_sel_setup(mem_acc_vreg, res, MEMORY_L2);
		if (rc) {
			pr_err("Unable to setup mem-acc for mem_type=%d rc=%d\n",
					MEMORY_L2, rc);
			return rc;
		}
		mem_acc_vreg->mem_acc_supported[MEMORY_L2] = true;
	}

	rc = populate_acc_data(mem_acc_vreg, "qcom,corner-acc-map",
			&mem_acc_vreg->corner_acc_map,
			&mem_acc_vreg->num_corners);
	if (rc) {
		pr_err("Unable to find 'qcom,corner-acc-map' rc=%d\n", rc);
		return rc;
	}

	pr_debug("num_corners = %d\n", mem_acc_vreg->num_corners);

	/* Check if at least one valid mem-acc config. is specified */
	for (i = 0; i < MEMORY_MAX; i++) {
		if (mem_acc_vreg->mem_acc_supported[i])
			break;
	}
	if (i == MEMORY_MAX) {
		pr_err("No mem-acc configuration specified\n");
		return -EINVAL;
	}

	if (mem_acc_vreg->num_acc_en)
		mem_acc_en_init(mem_acc_vreg);

	rc = mem_acc_sel_init(mem_acc_vreg);
	if (rc) {
		pr_err("Unable to intialize mem_acc_sel reg rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int mem_acc_regulator_probe(struct platform_device *pdev)
{
	struct mem_acc_regulator *mem_acc_vreg;
	struct regulator_desc *rdesc;
	struct regulator_init_data *init_data;
	int rc;

	if (!pdev->dev.of_node) {
		pr_err("Device tree node is missing\n");
		return -EINVAL;
	}

	init_data = of_get_regulator_init_data(&pdev->dev, pdev->dev.of_node);
	if (!init_data) {
		pr_err("regulator init data is missing\n");
		return -EINVAL;
	} else {
		init_data->constraints.input_uV
			= init_data->constraints.max_uV;
		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_VOLTAGE;
	}

	mem_acc_vreg = devm_kzalloc(&pdev->dev, sizeof(*mem_acc_vreg),
			GFP_KERNEL);
	if (!mem_acc_vreg) {
		pr_err("Can't allocate mem_acc_vreg memory\n");
		return -ENOMEM;
	}
	mem_acc_vreg->dev = &pdev->dev;

	rc = mem_acc_init(pdev, mem_acc_vreg);
	if (rc) {
		pr_err("Unable to initialize mem_acc configuration rc=%d\n",
				rc);
		return rc;
	}

	rdesc			= &mem_acc_vreg->rdesc;
	rdesc->owner		= THIS_MODULE;
	rdesc->type		= REGULATOR_VOLTAGE;
	rdesc->ops		= &mem_acc_corner_ops;
	rdesc->name		= init_data->constraints.name;

	mem_acc_vreg->rdev = regulator_register(rdesc, &pdev->dev,
				init_data, mem_acc_vreg, pdev->dev.of_node);
	if (IS_ERR(mem_acc_vreg->rdev)) {
		rc = PTR_ERR(mem_acc_vreg->rdev);
		if (rc != -EPROBE_DEFER)
			pr_err("regulator_register failed: rc=%d\n", rc);
		return rc;
	}

	platform_set_drvdata(pdev, mem_acc_vreg);

	return 0;
}

static int mem_acc_regulator_remove(struct platform_device *pdev)
{
	struct mem_acc_regulator *mem_acc_vreg = platform_get_drvdata(pdev);

	regulator_unregister(mem_acc_vreg->rdev);

	return 0;
}

static struct of_device_id mem_acc_regulator_match_table[] = {
	{ .compatible = "qcom,mem-acc-regulator", },
	{}
};

static struct platform_driver mem_acc_regulator_driver = {
	.probe		= mem_acc_regulator_probe,
	.remove		= mem_acc_regulator_remove,
	.driver		= {
		.name		= "qcom,mem-acc-regulator",
		.of_match_table = mem_acc_regulator_match_table,
		.owner		= THIS_MODULE,
	},
};

int __init mem_acc_regulator_init(void)
{
	return platform_driver_register(&mem_acc_regulator_driver);
}
postcore_initcall(mem_acc_regulator_init);

static void __exit mem_acc_regulator_exit(void)
{
	platform_driver_unregister(&mem_acc_regulator_driver);
}
module_exit(mem_acc_regulator_exit);

MODULE_DESCRIPTION("MEM-ACC-SEL regulator driver");
MODULE_LICENSE("GPL v2");
