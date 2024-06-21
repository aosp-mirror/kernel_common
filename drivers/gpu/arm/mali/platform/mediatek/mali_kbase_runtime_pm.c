// SPDX-License-Identifier: GPL-2.0
/* Copyright 2022 Google LLC. */

#include "mali_kbase_runtime_pm.h"

void mtk_voltage_range_check(struct kbase_device *kbdev, unsigned long *volts)
{
	struct mtk_platform_context *ctx = kbdev->platform_context;
	const struct mtk_hw_config *cfg = ctx->config;

	if (volts[1] < volts[0] + cfg->bias_min_microvolt ||
	    volts[1] > volts[0] + cfg->bias_max_microvolt)
		volts[1] = volts[0] + cfg->bias_min_microvolt;
	volts[1] = clamp_t(unsigned long, volts[1],
			   cfg->vsram_gpu_min_microvolt,
			   cfg->vsram_gpu_max_microvolt);
}

int mtk_set_frequency(struct kbase_device *kbdev, unsigned long freq)
{
	int err;
	struct mtk_platform_context *ctx = kbdev->platform_context;
	const struct mtk_hw_config *cfg = ctx->config;

	if (kbdev->current_freqs[0] == freq)
		return 0;

	if (ctx->manual_mux_reparent && cfg->num_clks > 1) {
		/*
		 * The mux clock doesn't automatically switch to a stable clock
		 * parent during PLL rate change, so we do it here.
		 */
		err = clk_set_parent(ctx->clks[mux].clk, ctx->clks[sub].clk);
		if (err) {
			dev_err(kbdev->dev, "Failed to set sub clock as src: %d\n", err);
			return err;
		}
	}

	/* Hard-coded rules about which clock should be used for freq setting */
	switch (cfg->num_clks) {
	case 5: /* there's pll clock */
		err = clk_set_rate(ctx->clks[pll].clk, freq);
		break;
	case 4:
		err = clk_set_rate(ctx->clks[main].clk, freq);
		break;
	case 1: /* upstream: only cg clock; set_rate is passed upwards; auto reparenting */
		err = clk_set_rate(ctx->clks[0].clk, freq);
		break;
	default:
		err = -EINVAL;
		break;
	}
	if (err) {
		dev_err(kbdev->dev, "Failed to set clock rate to %lu: %d\n", freq, err);
		return err;
	}
	kbdev->current_freqs[0] = freq;

	if (ctx->manual_mux_reparent && cfg->num_clks > 1) {
		err = clk_set_parent(ctx->clks[mux].clk, ctx->clks[main].clk);
		if (err) {
			dev_err(kbdev->dev, "Failed to set main clock as src: %d\n", err);
			return err;
		}
	}

	return 0;
}

static bool get_step_volt(struct kbase_device *kbdev, unsigned long *steps, unsigned long *targets,
			  bool inc)
{
	struct mtk_platform_context *ctx = kbdev->platform_context;
	const struct mtk_hw_config *cfg = ctx->config;
	int i;

	for (i = 0; i < kbdev->nr_regulators; i++)
		if (steps[i] != targets[i])
			break;

	if (i == kbdev->nr_regulators)
		return false;

	/*
	 * Do one round of *caterpillar move* - shrink the tail as much to the
	 * head as possible, and then step ahead as far as possible.
	 * Depending on the direction of voltage transition, a reversed
	 * sequence of extend-and-shrink may apply, which leads to the same
	 * result in the end.
	 */
	if (inc) {
		steps[0] = min(targets[0], steps[1] - cfg->bias_min_microvolt);
		steps[1] = min(targets[1], steps[0] + cfg->bias_max_microvolt);
	} else {
		steps[0] = max(targets[0], steps[1] - cfg->bias_max_microvolt);
		steps[1] = max(targets[1], steps[0] + cfg->bias_min_microvolt);
	}
	return true;
}

int mtk_set_voltages(struct kbase_device *kbdev, unsigned long *target_volts, bool inc)
{
	struct mtk_platform_context *ctx = kbdev->platform_context;
	const struct mtk_hw_config *cfg = ctx->config;
	const unsigned long reg_min_volt[BASE_MAX_NR_CLOCKS_REGULATORS] = {
		cfg->vgpu_min_microvolt,
		cfg->vsram_gpu_min_microvolt,
	};
	const unsigned long reg_max_volt[BASE_MAX_NR_CLOCKS_REGULATORS] = {
		cfg->vgpu_max_microvolt,
		cfg->vsram_gpu_max_microvolt,
	};
	unsigned long step_volts[BASE_MAX_NR_CLOCKS_REGULATORS];
	int i, err;

	/* Nothing to do if the direction of voltage transition is incorrect. */
	if ((inc && kbdev->current_voltages[0] > target_volts[0]) ||
	   (!inc && kbdev->current_voltages[0] < target_volts[0]))
		return 0;

	/* With upstream bindings, we let mtk-regulator-coupler driver handle vsram_gpu */
	if (kbdev->nr_regulators == 1) {
		err = regulator_set_voltage(kbdev->regulators[0], target_volts[0],
					    target_volts[0] + cfg->supply_tolerance_microvolt);
		if (err) {
			dev_err(kbdev->dev, "Failed to set regulator voltage: %d\n", err);
			return err;
		}

		kbdev->current_voltages[0] = target_volts[0];

		return 0;
	}

	for (i = 0; i < kbdev->nr_regulators; i++)
		step_volts[i] = kbdev->current_voltages[i];

	while (get_step_volt(kbdev, step_volts, target_volts, inc)) {
		for (i = 0; i < kbdev->nr_regulators; i++) {
			if (kbdev->current_voltages[i] == step_volts[i])
				continue;

			/* Assuming valid max voltages are always positive. */
			if (reg_max_volt[i] > 0 &&
			    (step_volts[i] < reg_min_volt[i] ||
			     step_volts[i] > reg_max_volt[i])) {
				dev_warn(kbdev->dev,
					 "Reg %d: invalid voltage %lu, clamp into [%lu, %lu]\n",
					 i, step_volts[i], reg_min_volt[i], reg_max_volt[i]);

				step_volts[i] = clamp_val(step_volts[i],
							  reg_min_volt[i],
							  reg_max_volt[i]);
			}

			err = regulator_set_voltage(kbdev->regulators[i],
						    step_volts[i],
						    step_volts[i] + cfg->supply_tolerance_microvolt);

			if (err) {
				dev_err(kbdev->dev,
					"Failed to set regulator %d voltage: %d\n",
					i, err);
				return err;
			}
			kbdev->current_voltages[i] = step_volts[i];
		}
	}

	return 0;
}

int mtk_map_mfg_base(struct mtk_platform_context *ctx)
{
	struct device_node *node;
	const struct mtk_hw_config *cfg = ctx->config;

	WARN_ON(cfg->mfg_compatible_name == NULL);
	node = of_find_compatible_node(NULL, NULL, cfg->mfg_compatible_name);
	if (!node)
		return -ENODEV;

	ctx->mfg_base_addr = of_iomap(node, 0);
	of_node_put(node);
	if (!ctx->mfg_base_addr)
		return -ENOMEM;

	return 0;
}

void mtk_unmap_mfg_base(struct mtk_platform_context *ctx)
{
	iounmap(ctx->mfg_base_addr);
}

void mtk_enable_timestamp_register(struct kbase_device *kbdev)
{
	struct mtk_platform_context *ctx = kbdev->platform_context;
	const struct mtk_hw_config *cfg = ctx->config;

	if (WARN_ON(!cfg->reg_mfg_timestamp) ||
	    WARN_ON(!cfg->top_tsvalueb_en)) {
		dev_err(kbdev->dev, "Cannot enable GPU timestamp due to missing configs\n");
		return;
	}

	/* Set register MFG_TIMESTAMP to TOP_TSVALEUB_EN */
	writel(cfg->top_tsvalueb_en, ctx->mfg_base_addr + cfg->reg_mfg_timestamp);
}

void mtk_check_bus_idle(struct kbase_device *kbdev)
{
	struct mtk_platform_context *ctx = kbdev->platform_context;
	const struct mtk_hw_config *cfg = ctx->config;
	u32 val;

	if (WARN_ON_ONCE(!cfg->reg_mfg_qchannel_con) ||
	    WARN_ON_ONCE(!cfg->reg_mfg_debug_sel) ||
	    WARN_ON_ONCE(!cfg->reg_mfg_debug_top) ||
	    WARN_ON_ONCE(!cfg->bus_idle_bit)) {
		dev_err(kbdev->dev, "Cannot check bus idelness due to missing configs\n");
		return;
	}

	/* Set register MFG_QCHANNEL_CON bit [1:0] = 0x1 */
	writel(0x1, ctx->mfg_base_addr + cfg->reg_mfg_qchannel_con);

	/* Set register MFG_DEBUG_SEL bit [7:0] = 0x3 */
	writel(0x3, ctx->mfg_base_addr + cfg->reg_mfg_debug_sel);

	/* Poll register MFG_DEBUG_TOP bit 2 = 0x1 */
	/* => 1 for bus idle, 0 for bus non-idle */
	do {
		val = readl(ctx->mfg_base_addr + cfg->reg_mfg_debug_top);
	} while ((val & cfg->bus_idle_bit) != cfg->bus_idle_bit);
}

int mtk_mfgsys_init(struct kbase_device *kbdev)
{
	int err, i;
	unsigned long volt;
	struct mtk_platform_context *ctx = kbdev->platform_context;
	const struct mtk_hw_config *cfg = ctx->config;

	/* Basic checks and pre-allocations */
	if (WARN_ON(cfg->num_clks <= 0))
		return -EINVAL;
	ctx->clks = devm_kcalloc(kbdev->dev, cfg->num_clks,
				     sizeof(*ctx->clks), GFP_KERNEL);
	if (!ctx->clks)
		return -ENOMEM;

	for (i = 0; i < cfg->num_clks; i++)
		ctx->clks[i].id = cfg->clk_names[i];

	for (i = 0; i < kbdev->nr_regulators; i++)
		if (!kbdev->regulators[i])
			return -EINVAL;


	/* The actual initialization starts here */
	err = devm_clk_bulk_get(kbdev->dev, cfg->num_clks, ctx->clks);
	if (err) {
		dev_err_probe(kbdev->dev, err, "Failed to devm_clk_bulk_get: %d\n", err);
		return err;
	}

	for (i = 0; i < kbdev->nr_regulators; i++) {
		volt = (i == 0) ? cfg->vgpu_max_microvolt
				: cfg->vsram_gpu_max_microvolt;
		err = regulator_set_voltage(kbdev->regulators[i],
			volt, volt + cfg->supply_tolerance_microvolt);
		if (err) {
			dev_err(kbdev->dev, "Failed to set reg %d voltage: %d\n", i, err);
			return err;
		}
#if IS_ENABLED(CONFIG_MALI_DEVFREQ)
		kbdev->current_voltages[i] = volt;
#endif
	}

	err = mtk_map_mfg_base(ctx);
	if (err) {
		dev_err(kbdev->dev, "Cannot find mfgcfg node: %d\n", err);
		return err;
	}

	ctx->gpu_is_powered = false;

	return 0;
}

int kbase_pm_domain_init(struct kbase_device *kbdev)
{
	int err, i, num_domains;
	struct mtk_platform_context *ctx = kbdev->platform_context;
	const struct mtk_hw_config *cfg = ctx->config;

	num_domains = of_count_phandle_with_args(kbdev->dev->of_node,
						 "power-domains",
						 "#power-domain-cells");

	if (WARN_ON(num_domains != kbdev->num_pm_domains)) {
		dev_err(kbdev->dev,
			"Incompatible power domain counts: %d provided, %d needed\n",
			num_domains, kbdev->num_pm_domains);
		return -EINVAL;
	}

	if (WARN_ON(num_domains > ARRAY_SIZE(kbdev->pm_domain_devs))) {
		dev_err(kbdev->dev, "Too many power domains: %d provided\n", num_domains);
		return -EINVAL;
	}

	/* Single power domain will be handled by the core. */
	if (num_domains < 2)
		return 0;

	for (i = 0; i < kbdev->num_pm_domains; i++) {
		kbdev->pm_domain_devs[i] =
			dev_pm_domain_attach_by_id(kbdev->dev, i);
		if (IS_ERR_OR_NULL(kbdev->pm_domain_devs[i])) {
			err = PTR_ERR(kbdev->pm_domain_devs[i]) ? : -ENODATA;
			kbdev->pm_domain_devs[i] = NULL;
			dev_err_probe(kbdev->dev, err, "Cannot attach PM domain %d\n", i);
			goto err;
		}

		if (cfg->auto_suspend_delay_ms) {
			pm_runtime_set_autosuspend_delay(kbdev->pm_domain_devs[i],
							 cfg->auto_suspend_delay_ms);
			pm_runtime_use_autosuspend(kbdev->pm_domain_devs[i]);
		}
	}

	return 0;
err:
	kbase_pm_domain_term(kbdev);
	return err;
}

void kbase_pm_domain_term(struct kbase_device *kbdev)
{
	int i;

	for (i = 0; i < kbdev->num_pm_domains; i++)
		if (kbdev->pm_domain_devs[i])
			dev_pm_domain_detach(kbdev->pm_domain_devs[i], true);
}

static int kbase_pm_runtime_callback_init(struct kbase_device *kbdev)
{
	return 0;
}

static void kbase_pm_runtime_callback_term(struct kbase_device *kbdev)
{
}

static int kbase_pm_runtime_callback_on(struct kbase_device *kbdev)
{
	return 0;
}

static void kbase_pm_runtime_callback_off(struct kbase_device *kbdev)
{
}

static int kbase_pm_callback_power_on(struct kbase_device *kbdev)
{
	int ret, err, reg_idx, pm_idx;
	struct mtk_platform_context *ctx = kbdev->platform_context;
	const struct mtk_hw_config *cfg = ctx->config;

	if (ctx->gpu_is_powered) {
		dev_dbg(kbdev->dev, "GPU is already powered on\n");
		return 0;
	}

	for (reg_idx = 0; reg_idx < kbdev->nr_regulators; reg_idx++) {
		ret = regulator_enable(kbdev->regulators[reg_idx]);
		if (ret < 0) {
			dev_err(kbdev->dev,
				"Failed to power on reg %d: %d\n",
				reg_idx, ret);
			goto reg_err;
		}
	}

	for (pm_idx = 0; pm_idx < kbdev->num_pm_domains; pm_idx++) {
		ret = pm_runtime_get_sync(kbdev->pm_domain_devs[pm_idx]);
		if (ret < 0) {
			dev_err(kbdev->dev,
				"Failed to power on power domain %d: %d\n",
				pm_idx, ret);
			goto pm_err;
		}
	}

	ret = clk_bulk_prepare_enable(cfg->num_clks, ctx->clks);
	if (ret < 0) {
		dev_err(kbdev->dev,
			"Failed to bulk-enable GPU clocks: %d\n",
			ret);
		goto clk_err;
	}

	mtk_enable_timestamp_register(kbdev);

	ctx->gpu_is_powered = true;

	return 1;

clk_err:
	clk_bulk_disable_unprepare(cfg->num_clks, ctx->clks);

pm_err:
	if (pm_idx >= kbdev->num_pm_domains)
		pm_idx = kbdev->num_pm_domains - 1;
	for (; pm_idx >= 0; pm_idx--) {
		pm_runtime_mark_last_busy(kbdev->pm_domain_devs[pm_idx]);
		err = pm_runtime_put_autosuspend(kbdev->pm_domain_devs[pm_idx]);
		if (err < 0)
			dev_err(kbdev->dev,
				"Failed to power off core %d: %d\n",
				pm_idx, err);
	}

reg_err:
	if (reg_idx >= kbdev->nr_regulators)
		reg_idx = kbdev->nr_regulators - 1;
	for (; reg_idx >= 0; reg_idx--) {
		err = regulator_disable(kbdev->regulators[reg_idx]);
		if (err < 0)
			dev_err(kbdev->dev,
				"Failed to power off reg %d: %d\n",
				reg_idx, err);
	}

	return ret;
}

static void kbase_pm_callback_power_off(struct kbase_device *kbdev)
{
	int err, i;
	struct mtk_platform_context *ctx = kbdev->platform_context;
	const struct mtk_hw_config *cfg = ctx->config;

	if (!ctx->gpu_is_powered) {
		dev_dbg(kbdev->dev, "GPU is already powered off\n");
		return;
	}

	ctx->gpu_is_powered = false;

	mtk_check_bus_idle(kbdev);

	clk_bulk_disable_unprepare(cfg->num_clks, ctx->clks);

	for (i = kbdev->num_pm_domains - 1; i >= 0; i--) {
		pm_runtime_mark_last_busy(kbdev->pm_domain_devs[i]);
		err = pm_runtime_put_autosuspend(kbdev->pm_domain_devs[i]);
		if (err < 0)
			dev_err(kbdev->dev,
				"Failed to power off core %d: %d\n",
				i, err);
	}

	for (i = kbdev->nr_regulators - 1; i >= 0; i--) {
		err = regulator_disable(kbdev->regulators[i]);
		if (err < 0)
			dev_err(kbdev->dev,
				"Failed to power off reg %d: %d\n",
				i, err);
	}
}

static void kbase_pm_callback_suspend(struct kbase_device *kbdev)
{
	kbase_pm_callback_power_off(kbdev);
}

static void kbase_pm_callback_resume(struct kbase_device *kbdev)
{
	kbase_pm_callback_power_on(kbdev);
}

struct kbase_pm_callback_conf mtk_pm_callbacks = {
	.power_on_callback = kbase_pm_callback_power_on,
	.power_off_callback = kbase_pm_callback_power_off,
	.power_suspend_callback = kbase_pm_callback_suspend,
	.power_resume_callback = kbase_pm_callback_resume,
#ifdef KBASE_PM_RUNTIME
	.power_runtime_init_callback = kbase_pm_runtime_callback_init,
	.power_runtime_term_callback = kbase_pm_runtime_callback_term,
	.power_runtime_on_callback = kbase_pm_runtime_callback_on,
	.power_runtime_off_callback = kbase_pm_runtime_callback_off,
#else				/* KBASE_PM_RUNTIME */
	.power_runtime_init_callback = NULL,
	.power_runtime_term_callback = NULL,
	.power_runtime_on_callback = NULL,
	.power_runtime_off_callback = NULL,
#endif				/* KBASE_PM_RUNTIME */
};

int mtk_platform_init(struct kbase_device *kbdev)
{
	struct mtk_platform_context *ctx = kbdev->platform_context;
	const struct mtk_hw_config *cfg = ctx->config;
	int err;

	if (WARN_ON(cfg->num_pm_domains <= 0))
		return -EINVAL;
	kbdev->num_pm_domains = cfg->num_pm_domains;

	err = kbase_pm_domain_init(kbdev);
	if (err)
		goto error;

	err = mtk_mfgsys_init(kbdev);
	if (err)
		goto error;

	err = mtk_set_frequency(kbdev, cfg->gpu_freq_max_khz * 1000);
	if (err)
		goto error;

	return 0;
error:
	platform_term(kbdev);
	return err;
}

void platform_term(struct kbase_device *kbdev)
{
	struct mtk_platform_context *ctx = kbdev->platform_context;

	mtk_unmap_mfg_base(ctx);
	kbdev->platform_context = NULL;
	kbase_pm_domain_term(kbdev);
}
