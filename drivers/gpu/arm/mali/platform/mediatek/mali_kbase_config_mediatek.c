// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2021 Mediatek Inc.

#include <linux/module.h>

#include <mali_kbase_config.h>

#include "mali_kbase_config_platform.h"

static struct kbase_platform_config dummy_platform_config;

struct kbase_platform_config *kbase_get_platform_config(void)
{
	return &dummy_platform_config;
}

#if IS_ENABLED(CONFIG_OF)
static const struct kbase_platform_specific_conf mediatek_mt8183_data = {
	.pm_callbacks = &mtk_pm_callbacks,
	.platform_funcs = &mt8183_platform_funcs,
};

static const struct kbase_platform_specific_conf mediatek_mt8183b_data = {
	.pm_callbacks = &mtk_pm_callbacks,
	.platform_funcs = &mt8183b_platform_funcs,
};

static const struct kbase_platform_specific_conf mediatek_mt8192_data = {
	.pm_callbacks = &mtk_pm_callbacks,
	.platform_funcs = &mt8192_platform_funcs,
};

static const struct kbase_platform_specific_conf mediatek_mt8195_data = {
	.pm_callbacks = &mtk_pm_callbacks,
	.platform_funcs = &mt8195_platform_funcs,
};

static const struct kbase_platform_specific_conf mediatek_mt8186_data = {
	.pm_callbacks = &mtk_pm_callbacks,
	.platform_funcs = &mt8186_platform_funcs,
};

static const struct kbase_platform_specific_conf mediatek_mt8188_data = {
	.pm_callbacks = &mtk_pm_callbacks,
	.platform_funcs = &mt8188_platform_funcs,
};

const struct of_device_id kbase_dt_ids[] = {
	{ .compatible = "arm,malit6xx" },
	{ .compatible = "arm,mali-midgard" },
	{ .compatible = "arm,mali-bifrost" },
	{ .compatible = "arm,mali-valhall" },
	{ .compatible = "mediatek,mt8183-mali", .data = &mediatek_mt8183_data },
	{ .compatible = "mediatek,mt8183b-mali", .data = &mediatek_mt8183b_data },
	{ .compatible = "mediatek,mt8192-mali", .data = &mediatek_mt8192_data },
	{ .compatible = "mediatek,mt8195-mali", .data = &mediatek_mt8195_data },
	{ .compatible = "mediatek,mt8186-mali", .data = &mediatek_mt8186_data },
	{ .compatible = "mediatek,mt8188-mali", .data = &mediatek_mt8188_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, kbase_dt_ids);
#else
int kbase_platform_register(void)
{
	return 0;
}

void kbase_platform_unregister(void)
{
}
#endif /* IS_ENABLED(CONFIG_OF) */
