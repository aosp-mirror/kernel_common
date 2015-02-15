/*
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ARCH_ARM_MACH_MSM_CLOCK_PLL_H
#define __ARCH_ARM_MACH_MSM_CLOCK_PLL_H

#include <mach/clk-provider.h>

enum {
	PLL_TCXO	= -1,
	PLL_0	= 0,
	PLL_1,
	PLL_2,
	PLL_3,
	PLL_4,
	PLL_END,
};

struct pll_shared_clk {
	unsigned int id;
	void __iomem *const mode_reg;
	struct clk c;
	void *const __iomem *base;
};

extern struct clk_ops clk_ops_pll;

static inline struct pll_shared_clk *to_pll_shared_clk(struct clk *c)
{
	return container_of(c, struct pll_shared_clk, c);
}

void msm_shared_pll_control_init(void);

struct pll_freq_tbl {
	const u32 freq_hz;
	const u32 l_val;
	const u32 m_val;
	const u32 n_val;
	const u32 post_div_val;
	const u32 pre_div_val;
	const u32 vco_val;
};

struct pll_config_masks {
	u32 post_div_mask;
	u32 pre_div_mask;
	u32 vco_mask;
	u32 mn_en_mask;
	u32 main_output_mask;
};

#define PLL_FREQ_END	(UINT_MAX-1)
#define PLL_F_END { .freq_hz = PLL_FREQ_END }

struct pll_vote_clk {
	u32 *soft_vote;
	const u32 soft_vote_mask;
	void __iomem *const en_reg;
	const u32 en_mask;
	void __iomem *const status_reg;
	const u32 status_mask;

	struct clk c;
	void *const __iomem *base;
};

extern struct clk_ops clk_ops_pll_vote;
extern struct clk_ops clk_ops_pll_acpu_vote;

#define PLL_SOFT_VOTE_PRIMARY   BIT(0)
#define PLL_SOFT_VOTE_ACPU      BIT(1)

static inline struct pll_vote_clk *to_pll_vote_clk(struct clk *c)
{
	return container_of(c, struct pll_vote_clk, c);
}

struct pll_clk {
	void __iomem *const mode_reg;
	void __iomem *const l_reg;
	void __iomem *const m_reg;
	void __iomem *const n_reg;
	void __iomem *const config_reg;
	void __iomem *const status_reg;

	void *const __iomem *rcg_debug_base;
	u32 rcg_cfg_value;
	u32 rcg_cmd_value;

	struct pll_config_masks masks;
	struct pll_freq_tbl *freq_tbl;

	struct clk c;
	void *const __iomem *base;
};

extern struct clk_ops clk_ops_local_pll;
extern struct clk_ops clk_ops_sr2_pll;

static inline struct pll_clk *to_pll_clk(struct clk *c)
{
	return container_of(c, struct pll_clk, c);
}

int sr_pll_clk_enable(struct clk *c);
int sr_hpm_lp_pll_clk_enable(struct clk *c);

struct pll_config {
	u32 l;
	u32 m;
	u32 n;
	u32 vco_val;
	u32 vco_mask;
	u32 pre_div_val;
	u32 pre_div_mask;
	u32 post_div_val;
	u32 post_div_mask;
	u32 mn_ena_val;
	u32 mn_ena_mask;
	u32 main_output_val;
	u32 main_output_mask;
	u32 aux_output_val;
	u32 aux_output_mask;
};

struct pll_config_regs {
	void __iomem *l_reg;
	void __iomem *m_reg;
	void __iomem *n_reg;
	void __iomem *config_reg;
	void __iomem *mode_reg;
	void *const __iomem *base;
};

void configure_sr_pll(struct pll_config *config, struct pll_config_regs *regs,
				u32 ena_fsm_mode);
void configure_sr_hpm_lp_pll(struct pll_config *config,
				struct pll_config_regs *, u32 ena_fsm_mode);
#endif
