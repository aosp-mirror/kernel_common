/*
 * Copyright (C) 2021 MediaTek Inc.
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

#include "mali_kbase_config_platform.h"
#include "mali_kbase_runtime_pm.h"

/* list of clocks required by GPU */
static const char * const mt8195_gpu_clks[] = {
	"clk_mux",
	"clk_main_parent",
	"clk_sub_parent",
	"subsys_bg3d",
	"clk_pll_src",
};

const struct mtk_hw_config mt8195_hw_config = {
	.num_pm_domains = 5,
	.num_clks = ARRAY_SIZE(mt8195_gpu_clks),
	.clk_names = mt8195_gpu_clks,
	.mfg_compatible_name = "mediatek,mt8195-mfgcfg",
	.reg_mfg_timestamp = 0x130,
	.reg_mfg_qchannel_con = 0xb4,
	.reg_mfg_debug_sel = 0x170,
	.reg_mfg_debug_top = 0x178,
	.top_tsvalueb_en = 0x3,
	.bus_idle_bit = 0x4,
	.vgpu_min_microvolt = 625000,
	.vgpu_max_microvolt = 750000,
	.vsram_gpu_min_microvolt = 750000,
	.vsram_gpu_max_microvolt = 750000,
	.bias_min_microvolt = 0,
	.bias_max_microvolt = 250000,
	.supply_tolerance_microvolt = 125,
	.gpu_freq_min_khz = 390000,
	.gpu_freq_max_khz = 880000,
	.auto_suspend_delay_ms = 50,
};

struct mtk_platform_context mt8195_platform_context = {
	.manual_mux_reparent = true,
	.config = &mt8195_hw_config,
};

static int platform_init(struct kbase_device *kbdev)
{
	struct mtk_platform_context *ctx = &mt8195_platform_context;
	int err;

	kbdev->platform_context = ctx;

	err = mtk_platform_init(kbdev);
	if (err)
		return err;

#if IS_ENABLED(CONFIG_MALI_DEVFREQ)
	kbdev->devfreq_ops.set_frequency = mtk_set_frequency;
	kbdev->devfreq_ops.voltage_range_check = mtk_voltage_range_check;
#endif

	return 0;
}

struct kbase_platform_funcs_conf mt8195_platform_funcs = {
	.platform_init_func = platform_init,
	.platform_term_func = platform_term
};
