/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2014-2017, 2020-2021 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#include <linux/mod_devicetable.h>

/**
 * Power management configuration
 *
 * Attached value: pointer to @ref kbase_pm_callback_conf
 * Default value: See @ref kbase_pm_callback_conf
 */
#define POWER_MANAGEMENT_CALLBACKS (NULL)

/**
 * Platform specific configuration functions
 *
 * Attached value: pointer to @ref kbase_platform_funcs_conf
 * Default value: See @ref kbase_platform_funcs_conf
 */
#define PLATFORM_FUNCS (NULL)

#define CLK_RATE_TRACE_OPS (&clk_rate_trace_ops)

extern struct kbase_clk_rate_trace_op_conf clk_rate_trace_ops;
extern struct kbase_pm_callback_conf mtk_pm_callbacks;

extern struct kbase_platform_funcs_conf mt8183_platform_funcs;
extern struct kbase_platform_funcs_conf mt8183b_platform_funcs;
extern struct kbase_platform_funcs_conf mt8192_platform_funcs;
extern struct kbase_platform_funcs_conf mt8195_platform_funcs;
extern struct kbase_platform_funcs_conf mt8186_platform_funcs;
extern struct kbase_platform_funcs_conf mt8188_platform_funcs;

#if IS_ENABLED(CONFIG_OF)
extern const struct of_device_id kbase_dt_ids[];
#endif
