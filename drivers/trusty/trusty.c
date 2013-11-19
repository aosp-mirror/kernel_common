/*
 * Copyright (C) 2013 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/trusty/smcall.h>
#include <linux/trusty/sm_err.h>
#include <linux/trusty/trusty.h>

struct trusty_state {
	struct mutex smc_lock;
};

static inline u32 smc(u32 r0, u32 r1, u32 r2, u32 r3)
{
	register u32 _r0 asm("r0") = r0;
	register u32 _r1 asm("r1") = r1;
	register u32 _r2 asm("r2") = r2;
	register u32 _r3 asm("r3") = r3;

	asm volatile(
		__asmeq("%0", "r0")
		__asmeq("%1", "r1")
		__asmeq("%2", "r2")
		__asmeq("%3", "r3")
		__asmeq("%4", "r0")
		__asmeq("%5", "r1")
		__asmeq("%6", "r2")
		__asmeq("%7", "r3")
		".arch_extension sec\n"
		"smc	#0	@ switch to secure world\n"
		: "=r" (_r0), "=r" (_r1), "=r" (_r2), "=r" (_r3)
		: "r" (_r0), "r" (_r1), "r" (_r2), "r" (_r3)
		: "ip");
	return _r0;
}

s32 trusty_fast_call32(struct device *dev, u32 smcnr, u32 a0, u32 a1, u32 a2)
{
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	BUG_ON(!s);
	BUG_ON(!SMC_IS_FASTCALL(smcnr));

	return smc(smcnr, a0, a1, a2);
}
EXPORT_SYMBOL(trusty_fast_call32);

s32 trusty_std_call32(struct device *dev, u32 smcnr, u32 a0, u32 a1, u32 a2)
{
	int ret;
	struct trusty_state *s = platform_get_drvdata(to_platform_device(dev));

	BUG_ON(SMC_IS_FASTCALL(smcnr));

	mutex_lock(&s->smc_lock);

	dev_dbg(dev, "%s(0x%x 0x%x 0x%x 0x%x) started\n",
		__func__, smcnr, a0, a1, a2);
	ret = smc(smcnr, a0, a1, a2);

	while (ret == SM_ERR_INTERRUPTED) {
		dev_dbg(dev, "%s(0x%x 0x%x 0x%x 0x%x) interrupted\n",
			__func__, smcnr, a0, a1, a2);
		ret = smc(SMC_SC_RESTART_LAST, 0, 0, 0);
	}
	dev_dbg(dev, "%s(0x%x 0x%x 0x%x 0x%x) returned 0x%x\n",
		__func__, smcnr, a0, a1, a2, ret);

	mutex_unlock(&s->smc_lock);

	return ret;
}
EXPORT_SYMBOL(trusty_std_call32);

static int trusty_remove_child(struct device *dev, void *data)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}

static int trusty_probe(struct platform_device *pdev)
{
	int ret;
	struct trusty_state *s;
	struct device_node *node = pdev->dev.of_node;

	if (!node) {
		dev_err(&pdev->dev, "of_node required\n");
		return -EINVAL;
	}

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s) {
		ret = -ENOMEM;
		goto err_allocate_state;
	}
	mutex_init(&s->smc_lock);
	platform_set_drvdata(pdev, s);

	ret = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to add children: %d\n", ret);
		goto err_add_children;
	}

	return 0;

err_add_children:
	device_for_each_child(&pdev->dev, NULL, trusty_remove_child);
	mutex_destroy(&s->smc_lock);
	kfree(s);
err_allocate_state:
	return ret;
}

static int trusty_remove(struct platform_device *pdev)
{
	struct trusty_state *s = platform_get_drvdata(pdev);

	device_for_each_child(&pdev->dev, NULL, trusty_remove_child);
	mutex_destroy(&s->smc_lock);
	kfree(s);
	return 0;
}

static const struct of_device_id trusty_of_match[] = {
	{ .compatible = "android,trusty-smc-v1", },
	{},
};

static struct platform_driver trusty_driver = {
	.probe = trusty_probe,
	.remove = trusty_remove,
	.driver	= {
		.name = "trusty",
		.owner = THIS_MODULE,
		.of_match_table = trusty_of_match,
	},
};

module_platform_driver(trusty_driver);
