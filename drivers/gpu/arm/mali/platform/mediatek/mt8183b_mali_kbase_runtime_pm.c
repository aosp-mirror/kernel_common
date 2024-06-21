// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Google LLC
 *
 * Based on mt8183_mali_kbase_runtime_pm.c
 *
 * Copyright (C) 2018 MediaTek Inc.
 */

#include "mali_kbase_config_platform.h"
#include "mali_kbase_runtime_pm.h"

/* new upstream binding */
static const char * const mt8183b_gpu_clks[] = { NULL };

static const struct mtk_hw_config mt8183b_hw_config = {
	.num_pm_domains = 3,
	.num_clks = ARRAY_SIZE(mt8183b_gpu_clks),
	.clk_names = mt8183b_gpu_clks,
	.mfg_compatible_name = "mediatek,mt8183-mfgcfg",
	.reg_mfg_timestamp = 0x130,
	.reg_mfg_qchannel_con = 0xb4,
	.reg_mfg_debug_sel = 0x180,
	.reg_mfg_debug_top = 0x188,
	.top_tsvalueb_en = 0x3,
	.bus_idle_bit = 0x4,
	.vgpu_min_microvolt = 625000,
	.vgpu_max_microvolt = 825000,
	.vsram_gpu_min_microvolt = 850000,
	.vsram_gpu_max_microvolt = 925000,
	.bias_min_microvolt = 100000,
	.bias_max_microvolt = 250000,
	.supply_tolerance_microvolt = 125,
	.gpu_freq_min_khz = 300000,
	.gpu_freq_max_khz = 800000,
	.auto_suspend_delay_ms = 50,
};

static struct mtk_platform_context mt8183b_platform_context = {
	.config = &mt8183b_hw_config,
};

static int platform_init(struct kbase_device *kbdev)
{
	struct mtk_platform_context *ctx = &mt8183b_platform_context;
	int err;

	kbdev->platform_context = ctx;

	err = mtk_platform_init(kbdev);
	if (err)
		return err;

#if IS_ENABLED(CONFIG_MALI_DEVFREQ)
	kbdev->devfreq_ops.set_frequency = mtk_set_frequency;
	kbdev->devfreq_ops.voltage_range_check = mtk_voltage_range_check;
#if IS_ENABLED(CONFIG_REGULATOR)
	kbdev->devfreq_ops.set_voltages = mtk_set_voltages;
#endif
#endif

	return 0;
}

struct kbase_platform_funcs_conf mt8183b_platform_funcs = {
	.platform_init_func = platform_init,
	.platform_term_func = platform_term
};
