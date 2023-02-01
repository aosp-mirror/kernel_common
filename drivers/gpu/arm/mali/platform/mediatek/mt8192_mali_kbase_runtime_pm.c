/*
 * Copyright (C) 2020 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/version.h>
#include "mali_kbase_config_platform.h"
#include "mali_kbase_runtime_pm.h"

/* list of clocks required by GPU */
static const char * const mt8192_gpu_clks[] = {
#if (KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE)
	/* Our old downstream code defines many clocks */
	"clk_mux",
	"clk_main_parent",
	"clk_sub_parent",
	"subsys_mfg_cg",
#else
	/* Upstream binding only uses one clock */
	NULL,
#endif
};

const struct mtk_hw_config mt8192_hw_config = {
	.num_pm_domains = 5,
	.num_clks = ARRAY_SIZE(mt8192_gpu_clks),
	.clk_names = mt8192_gpu_clks,
	.mfg_compatible_name = "mediatek,mt8192-mfgcfg",
	.reg_mfg_timestamp = 0x130,
	.reg_mfg_qchannel_con = 0xb4,
	.reg_mfg_debug_sel = 0x170,
	.reg_mfg_debug_top = 0x178,
	.top_tsvalueb_en = 0x3,
	.bus_idle_bit = 0x4,
	.vgpu_min_microvolt = 562500,
	.vgpu_max_microvolt = 800000,
	.vsram_gpu_min_microvolt = 750000,
	.vsram_gpu_max_microvolt = 800000,
	.bias_min_microvolt = 0,
	.bias_max_microvolt = 250000,
	.supply_tolerance_microvolt = 125,
	.gpu_freq_min_khz = 358000,
	.gpu_freq_max_khz = 950000,
	.auto_suspend_delay_ms = 50,
};

struct mtk_platform_context mt8192_platform_context = {
#if (KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE)
	/* Since v6.1 all the auto-reparenting code has been merged */
	.manual_mux_reparent = true,
#endif
	.config = &mt8192_hw_config,
};

/**
 * mt8192_disable_acp() - Disable ACP (Accelerator Coherency Port) on MT8192
 *
 * The experiemental ACP path on MT8192 should be disabled by default to avoid
 * breaking IO coherency on MT6873.
 *
 * Return: 0 on success, or error code on failure.
 */
static int mt8192_disable_acp(void)
{
	const char *infracfg_compatible_name = "mediatek,mt8192-infracfg";
	const unsigned int INFRA_CTL = 0x290;
	const unsigned int DIS_MFG2ACP_BIT = 9;
	struct device_node *node;
	void __iomem *infracfg_base_addr;
	unsigned int val;

	node = of_find_compatible_node(NULL, NULL, infracfg_compatible_name);
	if (!node)
		return -ENODEV;

	infracfg_base_addr = of_iomap(node, 0);
	of_node_put(node);
	if (!infracfg_base_addr)
		return -ENOMEM;

	val = readl(infracfg_base_addr + INFRA_CTL) | BIT(DIS_MFG2ACP_BIT);
	writel(val, infracfg_base_addr + INFRA_CTL);

	iounmap(infracfg_base_addr);
	return 0;
}

static int platform_init(struct kbase_device *kbdev)
{
	struct mtk_platform_context *ctx = &mt8192_platform_context;
	int err;

	kbdev->platform_context = ctx;

	err = mtk_platform_init(kbdev);
	if (err)
		return err;

#if IS_ENABLED(CONFIG_MALI_DEVFREQ)
	kbdev->devfreq_ops.set_frequency = mtk_set_frequency;
	kbdev->devfreq_ops.voltage_range_check = mtk_voltage_range_check;
#endif

	err = mt8192_disable_acp();
	if (err) {
		dev_err(kbdev->dev, "Failed to disable ACP: %d", err);
		return err;
	}

	return 0;
}

struct kbase_platform_funcs_conf mt8192_platform_funcs = {
	.platform_init_func = platform_init,
	.platform_term_func = platform_term
};
