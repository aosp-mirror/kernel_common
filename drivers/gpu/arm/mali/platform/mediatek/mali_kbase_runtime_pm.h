/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2022 Google LLC. */

#ifndef _MALI_KBASE_RUNTIME_PM_H_
#define _MALI_KBASE_RUNTIME_PM_H_

#include <linux/of_address.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <mali_kbase.h>
#include <mali_kbase_defs.h>

/**
 * Note that reordering the clock indices without extra care may cause
 * fatal errors.
 *
 * Different MediaTek platforms expose different number of clocks to its GPU,
 * and some of them are used for calibrating the GPU frequency.
 *
 * At the moment of this writing, the following pattern applies:
 * - MT8183 and MT8186 have 4 clocks: mux, main, sub and cg.
 *   The main clock is used for calibrating the GPU frequency.
 * - MT8192 and MT8195 have 5 clocks: mux, main, sub, cg and pll.
 *   The pll clock is used for calibrating the GPU frequency.
 * Given the above we make some hard-coded assumption for the ease of migration.
 *
 * Please carefully change the order and the corresponding frequency-switching
 * logic if a new HW design breaks the above rules.
 */
enum gpu_clk_idx {mux, main, sub, cg, pll};

/**
 * mtk_hw_config - config of the hardware specific constants
 * @num_pm_domains: number of GPU power domains
 * @num_clks: number of GPU clocks
 * @clk_names: the list of GPU clock names
 * @mfg_compatible_name: MFG compatible name in DT
 * @reg_mfg_timestamp: MFG_TIMESTAMP register address
 * @reg_mfg_qchannel_con: MFG_QCHANNEL_CON register address, used in bus idle check
 * @reg_mfg_debug_sel: MFG_DEBUG_SEL register address, used in bus idle check
 * @reg_mfg_debug_top: MFG_DEBUG_TOP register address, used in bus idle check
 * @top_tsvalueb_en: MFG_TIMESTAMP enable bit
 * @bus_idle_bit: bus idle check bit
 * @vgpu_min_microvolt: Minimal required voltage for vgpu.
 * @vgpu_max_microvolt: Maximal acceptable voltage for vgpu.
 * @vsram_gpu_min_microvolt: Minimal required voltage for vsram-gpu.
 * @vsram_gpu_max_microvolt: Maximal acceptable voltage for vsram-gpu.
 * @bias_min_microvolt: Minimal required voltage bias between vgpu and vsram-gpu.
 * @bias_max_microvolt: Maximal acceptable voltage bias between vgpu and vsram-gpu.
 *		        @bias_min_microvolt <= vsram - vgpu <= @bias_max_microvolt
 * @supply_tolerance_microvolt: The voltage diff tolerance.
 * @gpu_freq_min_khz: Minimum frequency GPU will be clocked at.
 * @gpu_freq_max_khz: Maximum frequency GPU will be clocked at.
 * @auto_suspend_delay_ms: The delay time to be used for auto-suspend.
 */
struct mtk_hw_config {
	/* Power domain */
	int num_pm_domains;

	/* Clocks */
	int num_clks;
	const char * const *clk_names;

	/* MFG */
	const char *mfg_compatible_name;
	unsigned int reg_mfg_timestamp;
	unsigned int reg_mfg_qchannel_con;
	unsigned int reg_mfg_debug_sel;
	unsigned int reg_mfg_debug_top;
	unsigned int top_tsvalueb_en;
	unsigned int bus_idle_bit;

	/* Voltage configuration for PMIC regulators */
	unsigned long vgpu_min_microvolt;
	unsigned long vgpu_max_microvolt;
	unsigned long vsram_gpu_min_microvolt;
	unsigned long vsram_gpu_max_microvolt;
	unsigned long bias_min_microvolt;
	unsigned long bias_max_microvolt;
	unsigned long supply_tolerance_microvolt;

	/* GPU frequency */
	unsigned long gpu_freq_min_khz;
	unsigned long gpu_freq_max_khz;

	/* Auto-suspend */
	unsigned long auto_suspend_delay_ms;
};

/**
 * mtk_platform_context - MediaTek platform context
 * @clks: GPU clocks
 * @mfg_base_addr: MFG base address
 * @gpu_is_powered: GPU on/off status
 * @manual_mux_reparent: Whether the platform driver needs to reparent the mux
 *                       clock for pll clock rate change.
 *                       Set this to false if it's handled by the base clock
 *                       driver.
 * @config: pointer to the hardware config struct
 *
 * This holds general platform information e.g. data probed from device tree,
 * predefined hardware config etc.
 */
struct mtk_platform_context {
	struct clk_bulk_data *clks;
	void __iomem *mfg_base_addr;
	bool gpu_is_powered;
	bool manual_mux_reparent;

	const struct mtk_hw_config *config;
};

void mtk_voltage_range_check(struct kbase_device *kbdev, unsigned long *volts);
int mtk_set_frequency(struct kbase_device *kbdev, unsigned long freq);
int mtk_set_voltages(struct kbase_device *kbdev, unsigned long *target_volts, bool inc);

int mtk_map_mfg_base(struct mtk_platform_context *ctx);
void mtk_unmap_mfg_base(struct mtk_platform_context *ctx);

void mtk_enable_timestamp_register(struct kbase_device *kbdev);
void mtk_check_bus_idle(struct kbase_device *kbdev);

int mtk_mfgsys_init(struct kbase_device *kbdev);

int kbase_pm_domain_init(struct kbase_device *kbdev);
void kbase_pm_domain_term(struct kbase_device *kbdev);

int mtk_platform_init(struct kbase_device *kbdev);
void platform_term(struct kbase_device *kbdev);
#endif /* _MALI_KBASE_RUNTIME_PM_H_ */
