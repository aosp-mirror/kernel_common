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

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/remote_spinlock.h>

#include <mach/scm-io.h>
#include <mach/msm_iomap.h>
#include <mach/msm_smem.h>

#include "clock.h"
#include "clock-pll.h"

#ifdef CONFIG_MSM_SECURE_IO
#undef readl_relaxed
#undef writel_relaxed
#define readl_relaxed secure_readl
#define writel_relaxed secure_writel
#endif

#define PLL_OUTCTRL BIT(0)
#define PLL_BYPASSNL BIT(1)
#define PLL_RESET_N BIT(2)
#define PLL_MODE_MASK BM(3, 0)

#define PLL_EN_REG(x) ((x)->base ? (*(x)->base + (u32)((x)->en_reg)) : \
				((x)->en_reg))
#define PLL_STATUS_REG(x) ((x)->base ? (*(x)->base + (u32)((x)->status_reg)) : \
				((x)->status_reg))
#define PLL_MODE_REG(x) ((x)->base ? (*(x)->base + (u32)((x)->mode_reg)) : \
				((x)->mode_reg))
#define PLL_L_REG(x) ((x)->base ? (*(x)->base + (u32)((x)->l_reg)) : \
				((x)->l_reg))
#define PLL_M_REG(x) ((x)->base ? (*(x)->base + (u32)((x)->m_reg)) : \
				((x)->m_reg))
#define PLL_N_REG(x) ((x)->base ? (*(x)->base + (u32)((x)->n_reg)) : \
				((x)->n_reg))
#define PLL_CONFIG_REG(x) ((x)->base ? (*(x)->base + (u32)((x)->config_reg)) : \
				((x)->config_reg))

static DEFINE_SPINLOCK(pll_reg_lock);

#define ENABLE_WAIT_MAX_LOOPS 200
#define PLL_LOCKED_BIT BIT(16)

#ifdef CONFIG_ARCH_DUMMY
int pming = 0;
#endif

static int fixed_pll_clk_set_rate(struct clk *c, unsigned long rate)
{
	if (rate != c->rate)
		return -EINVAL;
	return 0;
}

static long fixed_pll_clk_round_rate(struct clk *c, unsigned long rate)
{
	return c->rate;
}

static int pll_vote_clk_enable(struct clk *c)
{
	u32 ena, count;
	unsigned long flags;
	struct pll_vote_clk *pllv = to_pll_vote_clk(c);

	spin_lock_irqsave(&pll_reg_lock, flags);
	ena = readl_relaxed(PLL_EN_REG(pllv));
	ena |= pllv->en_mask;
	writel_relaxed(ena, PLL_EN_REG(pllv));
	spin_unlock_irqrestore(&pll_reg_lock, flags);

	mb();

	
	for (count = ENABLE_WAIT_MAX_LOOPS; count > 0; count--) {
		if (readl_relaxed(PLL_STATUS_REG(pllv)) & pllv->status_mask)
			return 0;
		udelay(1);
	}

	WARN("PLL %s didn't enable after voting for it!\n", c->dbg_name);

	return -ETIMEDOUT;
}

static void pll_vote_clk_disable(struct clk *c)
{
	u32 ena;
	unsigned long flags;
	struct pll_vote_clk *pllv = to_pll_vote_clk(c);

	spin_lock_irqsave(&pll_reg_lock, flags);
	ena = readl_relaxed(PLL_EN_REG(pllv));
	ena &= ~(pllv->en_mask);
	writel_relaxed(ena, PLL_EN_REG(pllv));
	spin_unlock_irqrestore(&pll_reg_lock, flags);
}

static int pll_vote_clk_is_enabled(struct clk *c)
{
	struct pll_vote_clk *pllv = to_pll_vote_clk(c);
	return !!(readl_relaxed(PLL_STATUS_REG(pllv)) & pllv->status_mask);
}

static enum handoff pll_vote_clk_handoff(struct clk *c)
{
	struct pll_vote_clk *pllv = to_pll_vote_clk(c);
	if (readl_relaxed(PLL_EN_REG(pllv)) & pllv->en_mask)
		return HANDOFF_ENABLED_CLK;

	return HANDOFF_DISABLED_CLK;
}

struct clk_ops clk_ops_pll_vote = {
	.enable = pll_vote_clk_enable,
	.disable = pll_vote_clk_disable,
	.is_enabled = pll_vote_clk_is_enabled,
	.round_rate = fixed_pll_clk_round_rate,
	.set_rate = fixed_pll_clk_set_rate,
	.handoff = pll_vote_clk_handoff,
};

static void __pll_config_reg(void __iomem *pll_config, struct pll_freq_tbl *f,
			struct pll_config_masks *masks)
{
	u32 regval;

	regval = readl_relaxed(pll_config);

	
	if (f->m_val)
		regval |= masks->mn_en_mask;

	
	regval &= ~masks->pre_div_mask;
	regval |= f->pre_div_val;
	regval &= ~masks->post_div_mask;
	regval |= f->post_div_val;

	
	regval &= ~masks->vco_mask;
	regval |= f->vco_val;

	
	if (masks->main_output_mask && !(regval & masks->main_output_mask))
		regval |= masks->main_output_mask;

	writel_relaxed(regval, pll_config);
}

static int sr2_pll_clk_enable(struct clk *c)
{
	unsigned long flags;
	struct pll_clk *pll = to_pll_clk(c);
	int ret = 0, count;
	u32 mode = readl_relaxed(PLL_MODE_REG(pll));

	spin_lock_irqsave(&pll_reg_lock, flags);

	
	mode |= PLL_BYPASSNL;
	writel_relaxed(mode, PLL_MODE_REG(pll));

	mb();
	udelay(10);

	
	mode |= PLL_RESET_N;
	writel_relaxed(mode, PLL_MODE_REG(pll));

	
	for (count = ENABLE_WAIT_MAX_LOOPS; count > 0; count--) {
		if (readl_relaxed(PLL_STATUS_REG(pll)) & PLL_LOCKED_BIT)
			break;
		udelay(1);
	}

	if (!(readl_relaxed(PLL_STATUS_REG(pll)) & PLL_LOCKED_BIT))
		pr_err("PLL %s didn't lock after enabling it!\n", c->dbg_name);

	
	mode |= PLL_OUTCTRL;
	writel_relaxed(mode, PLL_MODE_REG(pll));

	
	mb();

	spin_unlock_irqrestore(&pll_reg_lock, flags);
	return ret;
}

static void __pll_clk_enable_reg(void __iomem *mode_reg)
{
	u32 mode = readl_relaxed(mode_reg);
	
	mode |= PLL_BYPASSNL;
	writel_relaxed(mode, mode_reg);

	mb();
	udelay(10);

	
	mode |= PLL_RESET_N;
	writel_relaxed(mode, mode_reg);

	
	mb();
	udelay(50);

	
	mode |= PLL_OUTCTRL;
	writel_relaxed(mode, mode_reg);

	
	mb();
}

static int local_pll_clk_enable(struct clk *c)
{
	unsigned long flags;
	struct pll_clk *pll = to_pll_clk(c);

	spin_lock_irqsave(&pll_reg_lock, flags);
	__pll_clk_enable_reg(PLL_MODE_REG(pll));
	spin_unlock_irqrestore(&pll_reg_lock, flags);

	return 0;
}

static void __pll_clk_disable_reg(void __iomem *mode_reg)
{
	u32 mode = readl_relaxed(mode_reg);
	mode &= ~PLL_MODE_MASK;
	writel_relaxed(mode, mode_reg);
}

static void local_pll_clk_disable(struct clk *c)
{
	unsigned long flags;
	struct pll_clk *pll = to_pll_clk(c);

	spin_lock_irqsave(&pll_reg_lock, flags);
	__pll_clk_disable_reg(PLL_MODE_REG(pll));
	spin_unlock_irqrestore(&pll_reg_lock, flags);
}

static enum handoff local_pll_clk_handoff(struct clk *c)
{
	struct pll_clk *pll = to_pll_clk(c);
	u32 mode = readl_relaxed(PLL_MODE_REG(pll));
	u32 mask = PLL_BYPASSNL | PLL_RESET_N | PLL_OUTCTRL;
	unsigned long parent_rate;
	u32 lval, mval, nval, userval;

	if ((mode & mask) != mask)
		return HANDOFF_DISABLED_CLK;

	
	if (c->rate)
		return HANDOFF_ENABLED_CLK;

	parent_rate = clk_get_rate(c->parent);
	lval = readl_relaxed(PLL_L_REG(pll));
	mval = readl_relaxed(PLL_M_REG(pll));
	nval = readl_relaxed(PLL_N_REG(pll));
	userval = readl_relaxed(PLL_CONFIG_REG(pll));

	c->rate = parent_rate * lval;

	if (pll->masks.mn_en_mask && userval) {
		if (!nval)
			nval = 1;
		c->rate += (parent_rate * mval) / nval;
	}

	return HANDOFF_ENABLED_CLK;
}

static long local_pll_clk_round_rate(struct clk *c, unsigned long rate)
{
	struct pll_freq_tbl *nf;
	struct pll_clk *pll = to_pll_clk(c);

	if (!pll->freq_tbl)
		return -EINVAL;

	for (nf = pll->freq_tbl; nf->freq_hz != PLL_FREQ_END; nf++)
		if (nf->freq_hz >= rate)
			return nf->freq_hz;

	nf--;
	return nf->freq_hz;
}

static int local_pll_clk_set_rate(struct clk *c, unsigned long rate)
{
	struct pll_freq_tbl *nf;
	struct pll_clk *pll = to_pll_clk(c);
	unsigned long flags;
	u32 regval;

	for (nf = pll->freq_tbl; nf->freq_hz != PLL_FREQ_END
			&& nf->freq_hz != rate; nf++)
		;

	if (nf->freq_hz == PLL_FREQ_END)
		return -EINVAL;

	spin_lock_irqsave(&c->lock, flags);
	pll->rcg_cmd_value = readl_relaxed(*pll->rcg_debug_base + 0x50);
	pll->rcg_cfg_value = readl_relaxed(*pll->rcg_debug_base + 0x54);
	regval = (pll->rcg_cfg_value & BM(10, 8)) >> 8;
	if (regval == 0x5) {
		pr_err("invalid pll setting, c->count = %d, 0x%x, 0x%x\n", c->count, pll->rcg_cmd_value, pll->rcg_cfg_value);
		pr_err("L = %d\n", readl_relaxed(PLL_L_REG(pll)));
		pr_err("M = %d\n", readl_relaxed(PLL_M_REG(pll)));
		pr_err("N = %d\n", readl_relaxed(PLL_N_REG(pll)));
		BUG_ON(1);
	}

#ifdef CONFIG_ARCH_DUMMY
	
	if (!strcmp(c->dbg_name, "a7sspll") && pming)
		pr_info("[PP2] %s: Prevent from performing a7sspll clock disable, pll count = %d\n", __func__, c->count);
	else if ((strcmp(c->dbg_name, "a7sspll") && c->count) || (!strcmp(c->dbg_name, "a7sspll") && c->count && !pming))
		c->ops->disable(c);
#else
	if (c->count)
		c->ops->disable(c);
#endif

	writel_relaxed(nf->l_val, PLL_L_REG(pll));
	writel_relaxed(nf->m_val, PLL_M_REG(pll));
	writel_relaxed(nf->n_val, PLL_N_REG(pll));

	__pll_config_reg(PLL_CONFIG_REG(pll), nf, &pll->masks);

#ifdef CONFIG_ARCH_DUMMY
	
	if (!strcmp(c->dbg_name, "a7sspll") && pming)
		pr_info("[PP2] %s: Prevent from performing a7sspll enable, pll count = %d\n", __func__, c->count);
	else if ((strcmp(c->dbg_name, "a7sspll") && c->count) || (!strcmp(c->dbg_name, "a7sspll") && c->count && !pming))
		c->ops->enable(c);
#else
	if (c->count)
		c->ops->enable(c);
#endif
	spin_unlock_irqrestore(&c->lock, flags);
	return 0;
}

int sr_pll_clk_enable(struct clk *c)
{
	u32 mode;
	unsigned long flags;
	struct pll_clk *pll = to_pll_clk(c);

	spin_lock_irqsave(&pll_reg_lock, flags);
	mode = readl_relaxed(PLL_MODE_REG(pll));
	
	mode |= PLL_RESET_N;
	writel_relaxed(mode, PLL_MODE_REG(pll));

	mb();
	udelay(10);

	
	mode |= PLL_BYPASSNL;
	writel_relaxed(mode, PLL_MODE_REG(pll));

	
	mb();
	udelay(60);

	
	mode |= PLL_OUTCTRL;
	writel_relaxed(mode, PLL_MODE_REG(pll));

	
	mb();

	spin_unlock_irqrestore(&pll_reg_lock, flags);

	return 0;
}

int sr_hpm_lp_pll_clk_enable(struct clk *c)
{
	unsigned long flags;
	struct pll_clk *pll = to_pll_clk(c);
	u32 count, mode;
	int ret = 0;

	spin_lock_irqsave(&pll_reg_lock, flags);

	
	mode = PLL_BYPASSNL | PLL_RESET_N;
	writel_relaxed(mode, PLL_MODE_REG(pll));

	
	for (count = ENABLE_WAIT_MAX_LOOPS; count > 0; count--) {
		if (readl_relaxed(PLL_STATUS_REG(pll)) & PLL_LOCKED_BIT)
			break;
		udelay(1);
	}

	if (!(readl_relaxed(PLL_STATUS_REG(pll)) & PLL_LOCKED_BIT)) {
		WARN("PLL %s didn't lock after enabling it!\n", c->dbg_name);
		ret = -ETIMEDOUT;
		goto out;
	}

	
	mode |= PLL_OUTCTRL;
	writel_relaxed(mode, PLL_MODE_REG(pll));

	
	mb();

out:
	spin_unlock_irqrestore(&pll_reg_lock, flags);
	return ret;
}

struct clk_ops clk_ops_local_pll = {
	.enable = local_pll_clk_enable,
	.disable = local_pll_clk_disable,
	.set_rate = local_pll_clk_set_rate,
	.handoff = local_pll_clk_handoff,
};

struct clk_ops clk_ops_sr2_pll = {
	.enable = sr2_pll_clk_enable,
	.disable = local_pll_clk_disable,
	.set_rate = local_pll_clk_set_rate,
	.round_rate = local_pll_clk_round_rate,
	.handoff = local_pll_clk_handoff,
};

struct pll_rate {
	unsigned int lvalue;
	unsigned long rate;
};

static struct pll_rate pll_l_rate[] = {
	{10, 196000000},
	{12, 245760000},
	{30, 589820000},
	{38, 737280000},
	{41, 800000000},
	{50, 960000000},
	{52, 1008000000},
	{57, 1104000000},
	{60, 1152000000},
	{62, 1200000000},
	{63, 1209600000},
	{73, 1401600000},
	{0, 0},
};

#define PLL_BASE	7

struct shared_pll_control {
	uint32_t	version;
	struct {
		uint32_t	on;
		uint32_t	votes;
	} pll[PLL_BASE + PLL_END];
};

static remote_spinlock_t pll_lock;
static struct shared_pll_control *pll_control;

void __init msm_shared_pll_control_init(void)
{
#define PLL_REMOTE_SPINLOCK_ID "S:7"
	unsigned smem_size;

	remote_spin_lock_init(&pll_lock, PLL_REMOTE_SPINLOCK_ID);

	pll_control = smem_get_entry(SMEM_CLKREGIM_SOURCES, &smem_size);
	if (!pll_control) {
		pr_err("Can't find shared PLL control data structure!\n");
		BUG();
	} else if (smem_size < sizeof(struct shared_pll_control)) {
			pr_err("Shared PLL control data"
					 "structure too small!\n");
			BUG();
	} else if (pll_control->version != 0xCCEE0001) {
			pr_err("Shared PLL control version mismatch!\n");
			BUG();
	} else {
		pr_info("Shared PLL control available.\n");
		return;
	}

}

static int pll_clk_enable(struct clk *c)
{
	struct pll_shared_clk *pll = to_pll_shared_clk(c);
	unsigned int pll_id = pll->id;

	remote_spin_lock(&pll_lock);

	pll_control->pll[PLL_BASE + pll_id].votes |= BIT(1);
	if (!pll_control->pll[PLL_BASE + pll_id].on) {
		__pll_clk_enable_reg(PLL_MODE_REG(pll));
		pll_control->pll[PLL_BASE + pll_id].on = 1;
	}

	remote_spin_unlock(&pll_lock);
	return 0;
}

static void pll_clk_disable(struct clk *c)
{
	struct pll_shared_clk *pll = to_pll_shared_clk(c);
	unsigned int pll_id = pll->id;

	remote_spin_lock(&pll_lock);

	pll_control->pll[PLL_BASE + pll_id].votes &= ~BIT(1);
	if (pll_control->pll[PLL_BASE + pll_id].on
	    && !pll_control->pll[PLL_BASE + pll_id].votes) {
		__pll_clk_disable_reg(PLL_MODE_REG(pll));
		pll_control->pll[PLL_BASE + pll_id].on = 0;
	}

	remote_spin_unlock(&pll_lock);
}

static int pll_clk_is_enabled(struct clk *c)
{
	return readl_relaxed(PLL_MODE_REG(to_pll_shared_clk(c))) & BIT(0);
}

static enum handoff pll_clk_handoff(struct clk *c)
{
	struct pll_shared_clk *pll = to_pll_shared_clk(c);
	unsigned int pll_lval;
	struct pll_rate *l;

	do {
		pll_lval = readl_relaxed(PLL_MODE_REG(pll) + 4) & 0x3ff;
		cpu_relax();
		udelay(50);
	} while (pll_lval == 0);

	
	for (l = pll_l_rate; l->rate != 0; l++) {
		if (l->lvalue == pll_lval) {
			c->rate = l->rate;
			break;
		}
	}

	if (!c->rate) {
		pr_crit("Unknown PLL's L value!\n");
		BUG();
	}

	if (!pll_clk_is_enabled(c))
		return HANDOFF_DISABLED_CLK;

	remote_spin_lock(&pll_lock);
	pll_control->pll[PLL_BASE + pll->id].votes |= BIT(1);
	pll_control->pll[PLL_BASE + pll->id].on = 1;
	remote_spin_unlock(&pll_lock);

	return HANDOFF_ENABLED_CLK;
}

struct clk_ops clk_ops_pll = {
	.enable = pll_clk_enable,
	.disable = pll_clk_disable,
	.round_rate = fixed_pll_clk_round_rate,
	.set_rate = fixed_pll_clk_set_rate,
	.handoff = pll_clk_handoff,
	.is_enabled = pll_clk_is_enabled,
};

static DEFINE_SPINLOCK(soft_vote_lock);

static int pll_acpu_vote_clk_enable(struct clk *c)
{
	int ret = 0;
	unsigned long flags;
	struct pll_vote_clk *pllv = to_pll_vote_clk(c);

	spin_lock_irqsave(&soft_vote_lock, flags);

	if (!*pllv->soft_vote)
		ret = pll_vote_clk_enable(c);
	if (ret == 0)
		*pllv->soft_vote |= (pllv->soft_vote_mask);

	spin_unlock_irqrestore(&soft_vote_lock, flags);
	return ret;
}

static void pll_acpu_vote_clk_disable(struct clk *c)
{
	unsigned long flags;
	struct pll_vote_clk *pllv = to_pll_vote_clk(c);

	spin_lock_irqsave(&soft_vote_lock, flags);

	*pllv->soft_vote &= ~(pllv->soft_vote_mask);
	if (!*pllv->soft_vote)
		pll_vote_clk_disable(c);

	spin_unlock_irqrestore(&soft_vote_lock, flags);
}

static enum handoff pll_acpu_vote_clk_handoff(struct clk *c)
{
	if (pll_vote_clk_handoff(c) == HANDOFF_DISABLED_CLK)
		return HANDOFF_DISABLED_CLK;

	if (pll_acpu_vote_clk_enable(c))
		return HANDOFF_DISABLED_CLK;

	return HANDOFF_ENABLED_CLK;
}

struct clk_ops clk_ops_pll_acpu_vote = {
	.enable = pll_acpu_vote_clk_enable,
	.disable = pll_acpu_vote_clk_disable,
	.round_rate = fixed_pll_clk_round_rate,
	.set_rate = fixed_pll_clk_set_rate,
	.is_enabled = pll_vote_clk_is_enabled,
	.handoff = pll_acpu_vote_clk_handoff,
};

static void __init __set_fsm_mode(void __iomem *mode_reg,
					u32 bias_count, u32 lock_count)
{
	u32 regval = readl_relaxed(mode_reg);

	
	regval &= ~BIT(21);
	writel_relaxed(regval, mode_reg);

	
	regval &= ~BM(19, 14);
	regval |= BVAL(19, 14, bias_count);
	writel_relaxed(regval, mode_reg);

	
	regval &= ~BM(13, 8);
	regval |= BVAL(13, 8, lock_count);
	writel_relaxed(regval, mode_reg);

	
	regval |= BIT(20);
	writel_relaxed(regval, mode_reg);
}

void __init __configure_pll(struct pll_config *config,
		struct pll_config_regs *regs, u32 ena_fsm_mode)
{
	u32 regval;

	writel_relaxed(config->l, PLL_L_REG(regs));
	writel_relaxed(config->m, PLL_M_REG(regs));
	writel_relaxed(config->n, PLL_N_REG(regs));

	regval = readl_relaxed(PLL_CONFIG_REG(regs));

	
	if (config->mn_ena_mask) {
		regval &= ~config->mn_ena_mask;
		regval |= config->mn_ena_val;
	}

	
	if (config->main_output_mask) {
		regval &= ~config->main_output_mask;
		regval |= config->main_output_val;
	}

	
	if (config->aux_output_mask) {
		regval &= ~config->aux_output_mask;
		regval |= config->aux_output_val;
	}

	
	regval &= ~config->pre_div_mask;
	regval |= config->pre_div_val;
	regval &= ~config->post_div_mask;
	regval |= config->post_div_val;

	
	regval &= ~config->vco_mask;
	regval |= config->vco_val;
	writel_relaxed(regval, PLL_CONFIG_REG(regs));
}

void __init configure_sr_pll(struct pll_config *config,
		struct pll_config_regs *regs, u32 ena_fsm_mode)
{
	__configure_pll(config, regs, ena_fsm_mode);
	if (ena_fsm_mode)
		__set_fsm_mode(PLL_MODE_REG(regs), 0x1, 0x8);
}

void __init configure_sr_hpm_lp_pll(struct pll_config *config,
		struct pll_config_regs *regs, u32 ena_fsm_mode)
{
	__configure_pll(config, regs, ena_fsm_mode);
	if (ena_fsm_mode)
		__set_fsm_mode(PLL_MODE_REG(regs), 0x1, 0x0);
}

