/*
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/spinlock.h>

#include <mach/clk-provider.h>
#include <mach/clock-generic.h>

#if defined(CONFIG_HTC_DEBUG_FOOTPRINT) && defined(CONFIG_MSM_CORTEX_A7)
#include <mach/htc_footprint.h>
#endif


int parent_to_src_sel(struct clk_src *parents, int num_parents, struct clk *p)
{
	int i;

	for (i = 0; i < num_parents; i++) {
		if (parents[i].src == p)
			return parents[i].sel;
	}

	return -EINVAL;
}

static int mux_set_parent(struct clk *c, struct clk *p)
{
	struct mux_clk *mux = to_mux_clk(c);
	int sel = parent_to_src_sel(mux->parents, mux->num_parents, p);
	struct clk *old_parent;
	int rc = 0, i;
	unsigned long flags;

	if (sel < 0 && mux->rec_set_par) {
		for (i = 0; i < mux->num_parents; i++) {
			rc = clk_set_parent(mux->parents[i].src, p);
			if (!rc) {
				sel = mux->parents[i].sel;
				p = mux->parents[i].src;
				break;
			}
		}
	}

	if (sel < 0)
		return sel;

	rc = __clk_pre_reparent(c, p, &flags);
	if (rc)
		goto out;

	rc = mux->ops->set_mux_sel(mux, sel);
	if (rc)
		goto set_fail;

	old_parent = c->parent;
	c->parent = p;
	c->rate = clk_get_rate(p);
	__clk_post_reparent(c, old_parent, &flags);

	return 0;

set_fail:
	__clk_post_reparent(c, p, &flags);
out:
	return rc;
}

static long mux_round_rate(struct clk *c, unsigned long rate)
{
	struct mux_clk *mux = to_mux_clk(c);
	int i;
	unsigned long prate, rrate = 0;

	for (i = 0; i < mux->num_parents; i++) {
		prate = clk_round_rate(mux->parents[i].src, rate);
		if (is_better_rate(rate, rrate, prate))
			rrate = prate;
	}
	if (!rrate)
		return -EINVAL;

	return rrate;
}

static int mux_set_rate(struct clk *c, unsigned long rate)
{
	struct mux_clk *mux = to_mux_clk(c);
	struct clk *new_parent = NULL;
	int rc = 0, i;
	unsigned long new_par_curr_rate;
	unsigned long flags;

	for (i = 0; i < mux->num_parents; i++) {
		if (clk_round_rate(mux->parents[i].src, rate) == rate) {
			new_parent = mux->parents[i].src;
			break;
		}
	}
	if (new_parent == NULL)
		return -EINVAL;

	if (mux->safe_sel >= 0) {
		spin_lock_irqsave(&c->lock, flags);
		rc = mux->ops->set_mux_sel(mux, mux->safe_sel);
		spin_unlock_irqrestore(&c->lock, flags);
	}
	if (rc)
		return rc;

	new_par_curr_rate = clk_get_rate(new_parent);
	rc = clk_set_rate(new_parent, rate);
	if (rc)
		goto set_rate_fail;

	rc = mux_set_parent(c, new_parent);
	if (rc)
		goto set_par_fail;

	return 0;

set_par_fail:
	clk_set_rate(new_parent, new_par_curr_rate);
set_rate_fail:
	WARN(mux->ops->set_mux_sel(mux,
		parent_to_src_sel(mux->parents, mux->num_parents, c->parent)),
		"Set rate failed for %s. Also in bad state!\n", c->dbg_name);
	return rc;
}

static int mux_enable(struct clk *c)
{
	struct mux_clk *mux = to_mux_clk(c);
	if (mux->ops->enable)
		return mux->ops->enable(mux);
	return 0;
}

static void mux_disable(struct clk *c)
{
	struct mux_clk *mux = to_mux_clk(c);
	if (mux->ops->disable)
		return mux->ops->disable(mux);
}

static struct clk *mux_get_parent(struct clk *c)
{
	struct mux_clk *mux = to_mux_clk(c);
	int sel = mux->ops->get_mux_sel(mux);
	int i;

	for (i = 0; i < mux->num_parents; i++) {
		if (mux->parents[i].sel == sel)
			return mux->parents[i].src;
	}

	
	return NULL;
}

static enum handoff mux_handoff(struct clk *c)
{
	struct mux_clk *mux = to_mux_clk(c);

	c->rate = clk_get_rate(c->parent);
	mux->safe_sel = parent_to_src_sel(mux->parents, mux->num_parents,
							mux->safe_parent);

	if (mux->en_mask && mux->ops && mux->ops->is_enabled)
		return mux->ops->is_enabled(mux)
			? HANDOFF_ENABLED_CLK
			: HANDOFF_DISABLED_CLK;

	return HANDOFF_DISABLED_CLK;
}

struct clk_ops clk_ops_gen_mux = {
	.enable = mux_enable,
	.disable = mux_disable,
	.set_parent = mux_set_parent,
	.round_rate = mux_round_rate,
	.set_rate = mux_set_rate,
	.handoff = mux_handoff,
	.get_parent = mux_get_parent,
};


static long __div_round_rate(struct div_data *data, unsigned long rate,
	struct clk *parent, unsigned int *best_div, unsigned long *best_prate)
{
	unsigned int div, min_div, max_div, _best_div = 1;
	unsigned long prate, _best_prate = 0, rrate = 0;

	rate = max(rate, 1UL);

	min_div = max(data->min_div, 1U);
	max_div = min(data->max_div, (unsigned int) (ULONG_MAX / rate));

	for (div = min_div; div <= max_div; div++) {
		prate = clk_round_rate(parent, rate * div);
		if (IS_ERR_VALUE(prate))
			break;

		if (is_better_rate(rate, rrate, prate / div)) {
			rrate = prate / div;
			_best_div = div;
			_best_prate = prate;
		}

		if (prate / div < rate)
			break;

		if (rrate <= rate + data->rate_margin)
			break;
	}

	if (!rrate)
		return -EINVAL;
	if (best_div)
		*best_div = _best_div;
	if (best_prate)
		*best_prate = _best_prate;

	return rrate;
}

static long div_round_rate(struct clk *c, unsigned long rate)
{
	struct div_clk *d = to_div_clk(c);

	return __div_round_rate(&d->data, rate, c->parent, NULL, NULL);
}

static int div_set_rate(struct clk *c, unsigned long rate)
{
	struct div_clk *d = to_div_clk(c);
	int div, rc = 0;
	long rrate, old_prate, new_prate;
	struct div_data *data = &d->data;

	rrate = __div_round_rate(data, rate, c->parent, &div, &new_prate);
	if (rrate != rate)
		return -EINVAL;

	if (div > data->div)
		rc = d->ops->set_div(d, div);
	if (rc)
		return rc;

	old_prate = clk_get_rate(c->parent);
	rc = clk_set_rate(c->parent, new_prate);
	if (rc)
		goto set_rate_fail;

	if (div < data->div)
		rc = d->ops->set_div(d, div);
	if (rc)
		goto div_dec_fail;

	data->div = div;

	return 0;

div_dec_fail:
	WARN(clk_set_rate(c->parent, old_prate),
		"Set rate failed for %s. Also in bad state!\n", c->dbg_name);
set_rate_fail:
	if (div > data->div)
		WARN(d->ops->set_div(d, data->div),
			"Set rate failed for %s. Also in bad state!\n",
			c->dbg_name);
	return rc;
}

static int div_enable(struct clk *c)
{
	struct div_clk *d = to_div_clk(c);
	if (d->ops && d->ops->enable)
		return d->ops->enable(d);
	return 0;
}

static void div_disable(struct clk *c)
{
	struct div_clk *d = to_div_clk(c);
	if (d->ops && d->ops->disable)
		return d->ops->disable(d);
}

static enum handoff div_handoff(struct clk *c)
{
	struct div_clk *d = to_div_clk(c);
	unsigned int div = d->data.div;

	if (d->ops && d->ops->get_div)
		div = max(d->ops->get_div(d), 1);
	div = max(div, 1U);
	c->rate = clk_get_rate(c->parent) / div;

	if (!d->ops || !d->ops->set_div)
		d->data.min_div = d->data.max_div = div;
	d->data.div = div;

	if (d->en_mask && d->ops && d->ops->is_enabled)
		return d->ops->is_enabled(d)
			? HANDOFF_ENABLED_CLK
			: HANDOFF_DISABLED_CLK;

	return HANDOFF_DISABLED_CLK;
}

struct clk_ops clk_ops_div = {
	.enable = div_enable,
	.disable = div_disable,
	.round_rate = div_round_rate,
	.set_rate = div_set_rate,
	.handoff = div_handoff,
};

static long __slave_div_round_rate(struct clk *c, unsigned long rate,
					int *best_div)
{
	struct div_clk *d = to_div_clk(c);
	unsigned int div, min_div, max_div;
	long p_rate;

	rate = max(rate, 1UL);

	min_div = d->data.min_div;
	max_div = d->data.max_div;

	p_rate = clk_get_rate(c->parent);
	div = p_rate / rate;
	div = max(div, min_div);
	div = min(div, max_div);
	if (best_div)
		*best_div = div;

	return p_rate / div;
}

static long slave_div_round_rate(struct clk *c, unsigned long rate)
{
	return __slave_div_round_rate(c, rate, NULL);
}

static int slave_div_set_rate(struct clk *c, unsigned long rate)
{
	struct div_clk *d = to_div_clk(c);
	int div, rc = 0;
	long rrate;

	rrate = __slave_div_round_rate(c, rate, &div);
	if (rrate != rate)
		return -EINVAL;

	if (div == d->data.div)
		return 0;

	rc = d->ops->set_div(d, div);
	if (rc)
		return rc;

	d->data.div = div;

	return 0;
}

static unsigned long slave_div_get_rate(struct clk *c)
{
	struct div_clk *d = to_div_clk(c);
	if (!d->data.div)
		return 0;
	return clk_get_rate(c->parent) / d->data.div;
}

struct clk_ops clk_ops_slave_div = {
	.enable = div_enable,
	.disable = div_disable,
	.round_rate = slave_div_round_rate,
	.set_rate = slave_div_set_rate,
	.get_rate = slave_div_get_rate,
	.handoff = div_handoff,
};



static long ext_round_rate(struct clk *c, unsigned long rate)
{
	return clk_round_rate(c->parent, rate);
}

static int ext_set_rate(struct clk *c, unsigned long rate)
{
	return clk_set_rate(c->parent, rate);
}

static unsigned long ext_get_rate(struct clk *c)
{
	return clk_get_rate(c->parent);
}

static int ext_set_parent(struct clk *c, struct clk *p)
{
	return clk_set_parent(c->parent, p);
}

static enum handoff ext_handoff(struct clk *c)
{
	c->rate = clk_get_rate(c->parent);
	
	return HANDOFF_DISABLED_CLK;
}

struct clk_ops clk_ops_ext = {
	.handoff = ext_handoff,
	.round_rate = ext_round_rate,
	.set_rate = ext_set_rate,
	.get_rate = ext_get_rate,
	.set_parent = ext_set_parent,
};



struct clk_logger {
	u64 timestamp;
	u32 fn;
	u32 cmd_reg;
	u32 cfg_reg;
	u32 rate;
};

#define BUF_SIZE (50)
#define FN_SET_RATE (1)
#define FN_ENABLE (2)
#define FN_DISABLE (3)
static struct clk_logger clk_log[BUF_SIZE];
static u32 buf_index;
static DEFINE_SPINLOCK(log_lock);

static void log_clk_call(struct clk *c, u32 fn, u32 rate) {
	struct mux_div_clk *md = to_mux_div_clk(c);
	struct clk_logger *log;
	unsigned long flags;

	spin_lock_irqsave(&log_lock, flags);
	log = &clk_log[buf_index];

	log->timestamp = sched_clock();
	log->fn = fn;
	log->cmd_reg = readl_relaxed(md->base + md->div_offset - 0x4);
	log->cfg_reg = readl_relaxed(md->base + md->div_offset);
	log->rate = rate;

	buf_index++;
	if (buf_index >= BUF_SIZE)
		buf_index = 0;

	spin_unlock_irqrestore(&log_lock, flags);
}

static int mux_div_clk_enable(struct clk *c)
{
	struct mux_div_clk *md = to_mux_div_clk(c);

	log_clk_call(c, FN_ENABLE, c->rate);

	if (md->ops->enable)
		return md->ops->enable(md);
	return 0;
}

static void mux_div_clk_disable(struct clk *c)
{
	struct mux_div_clk *md = to_mux_div_clk(c);

	log_clk_call(c, FN_DISABLE, 300000000);

	if (md->ops->disable)
		return md->ops->disable(md);
}

static long __mux_div_round_rate(struct clk *c, unsigned long rate,
	struct clk **best_parent, int *best_div, unsigned long *best_prate)
{
	struct mux_div_clk *md = to_mux_div_clk(c);
	unsigned int i;
	unsigned long rrate, best = 0, _best_div = 0, _best_prate = 0;
	struct clk *_best_parent = 0;

	for (i = 0; i < md->num_parents; i++) {
		int div;
		unsigned long prate;

		rrate = __div_round_rate(&md->data, rate, md->parents[i].src,
				&div, &prate);

		if (is_better_rate(rate, best, rrate)) {
			best = rrate;
			_best_div = div;
			_best_prate = prate;
			_best_parent = md->parents[i].src;
		}

		if (rate <= rrate && rrate <= rate + md->data.rate_margin)
			break;
	}

	if (best_div)
		*best_div = _best_div;
	if (best_prate)
		*best_prate = _best_prate;
	if (best_parent)
		*best_parent = _best_parent;

	if (best)
		return best;
	return -EINVAL;
}

static long mux_div_clk_round_rate(struct clk *c, unsigned long rate)
{
	return __mux_div_round_rate(c, rate, NULL, NULL, NULL);
}

static int __set_src_div(struct mux_div_clk *md, struct clk *parent, u32 div)
{
	u32 rc = 0, src_sel;

	src_sel = parent_to_src_sel(md->parents, md->num_parents, parent);

	
	WARN(!md->c.count, "ref count is zero! parent will not be switched to gpll0\n");

	if (md->c.count)
		rc = md->ops->set_src_div(md, src_sel, div);
	if (!rc) {
		md->data.div = div;
		md->src_sel = src_sel;
	}

	return rc;
}

static int set_src_div(struct mux_div_clk *md, struct clk *parent, u32 div)
{
	unsigned long flags;
	u32 rc;

	spin_lock_irqsave(&md->c.lock, flags);
	rc = __set_src_div(md, parent, div);
	spin_unlock_irqrestore(&md->c.lock, flags);

	return rc;
}

static int safe_parent_init_once(struct clk *c)
{
	unsigned long rrate;
	u32 best_div;
	struct clk *best_parent;
	struct mux_div_clk *md = to_mux_div_clk(c);

	if (IS_ERR(md->safe_parent))
		return -EINVAL;
	if (!md->safe_freq || md->safe_parent)
		return 0;

	rrate = __mux_div_round_rate(c, md->safe_freq, &best_parent,
			&best_div, NULL);

	if (rrate == md->safe_freq) {
		md->safe_div = best_div;
		md->safe_parent = best_parent;
	} else {
		md->safe_parent = ERR_PTR(-EINVAL);
		return -EINVAL;
	}
	return 0;
}

static int mux_div_clk_set_rate(struct clk *c, unsigned long rate)
{
	struct mux_div_clk *md = to_mux_div_clk(c);
	unsigned long flags, rrate;
	unsigned long new_prate, old_prate;
	struct clk *old_parent, *new_parent;
	u32 new_div, old_div;
	int rc;

#if defined(CONFIG_HTC_DEBUG_FOOTPRINT) && defined(CONFIG_MSM_CORTEX_A7)
	set_acpuclk_footprint(0, ACPU_BEFORE_SAFE_PARENT_INIT);
#endif

	log_clk_call(c, FN_SET_RATE, rate);

	rc = safe_parent_init_once(c);
	if (rc)
		return rc;

	rrate = __mux_div_round_rate(c, rate, &new_parent, &new_div,
							&new_prate);
	if (rrate != rate)
		return -EINVAL;

	old_parent = c->parent;
	old_div = md->data.div;
	old_prate = clk_get_rate(c->parent);

#if defined(CONFIG_HTC_DEBUG_FOOTPRINT) && defined(CONFIG_MSM_CORTEX_A7)
	set_acpuclk_footprint(0, ACPU_BEFORE_SET_SAFE_RATE);
#endif

	
	if (md->safe_freq)
		rc = set_src_div(md, md->safe_parent, md->safe_div);

	else if (new_parent == old_parent && new_div >= old_div) {
		rc = set_src_div(md, old_parent, new_div);
	}
	if (rc) {
		WARN(rc, "error switching to safe_parent freq=%ld\n", md->safe_freq);
		return rc;
	}
#if defined(CONFIG_HTC_DEBUG_FOOTPRINT) && defined(CONFIG_MSM_CORTEX_A7)
	set_acpuclk_footprint(0, ACPU_BEFORE_SET_PARENT_RATE);
#endif

	rc = clk_set_rate(new_parent, new_prate);
	if (rc) {
		pr_err("failed to set %s to %ld\n",
			new_parent->dbg_name, new_prate);
		goto err_set_rate;
	}

#if defined(CONFIG_HTC_DEBUG_FOOTPRINT) && defined(CONFIG_MSM_CORTEX_A7)
	set_acpuclk_footprint(0, ACPU_BEFORE_CLK_PREPARE);
#endif

	rc = __clk_pre_reparent(c, new_parent, &flags);
	if (rc)
		goto err_pre_reparent;

#if defined(CONFIG_HTC_DEBUG_FOOTPRINT) && defined(CONFIG_MSM_CORTEX_A7)
	set_acpuclk_footprint(0, ACPU_BEFORE_SET_RATE);
#endif

	
	rc = __set_src_div(md, new_parent, new_div);
	if (rc)
		goto err_set_src_div;

#if defined(CONFIG_HTC_DEBUG_FOOTPRINT) && defined(CONFIG_MSM_CORTEX_A7)
	
	set_acpuclk_cpu_freq_footprint(FT_CUR_RATE, 0, rrate);
	set_acpuclk_footprint(0, ACPU_BEFORE_CLK_UNPREPARE);
#endif

	c->parent = new_parent;

	__clk_post_reparent(c, old_parent, &flags);

#if defined(CONFIG_HTC_DEBUG_FOOTPRINT) && defined(CONFIG_MSM_CORTEX_A7)
	set_acpuclk_footprint(0, ACPU_BEFORE_RETURN);
#endif

	return 0;

err_set_src_div:
	
	WARN(rc, "disabling %s\n", new_parent->dbg_name);
#if defined(CONFIG_HTC_DEBUG_FOOTPRINT) && defined(CONFIG_MSM_CORTEX_A7)
	set_acpuclk_footprint(0, ACPU_BEFORE_ERR_CLK_UNPREPARE);
#endif

	
	__clk_post_reparent(c, new_parent, &flags);
err_pre_reparent:

#if defined(CONFIG_HTC_DEBUG_FOOTPRINT) && defined(CONFIG_MSM_CORTEX_A7)
	set_acpuclk_footprint(0, ACPU_BEFORE_ERR_SET_PARENT_RATE);
#endif

	WARN(rc, "%s: error changing parent (%s) rate to %ld\n",
		c->dbg_name, old_parent->dbg_name, old_prate);
	rc = clk_set_rate(old_parent, old_prate);
err_set_rate:

#if defined(CONFIG_HTC_DEBUG_FOOTPRINT) && defined(CONFIG_MSM_CORTEX_A7)
	set_acpuclk_footprint(0, ACPU_BEFORE_ERR_SET_RATE);
#endif

	WARN(rc, "%s: error changing back to original div (%d) and parent (%s)\n",
		c->dbg_name, old_div, old_parent->dbg_name);
	rc = set_src_div(md, old_parent, old_div);
#if defined(CONFIG_HTC_DEBUG_FOOTPRINT) && defined(CONFIG_MSM_CORTEX_A7)
	set_acpuclk_footprint(0, ACPU_BEFORE_ERR_RETURN);
#endif

	return rc;
}

static struct clk *mux_div_clk_get_parent(struct clk *c)
{
	struct mux_div_clk *md = to_mux_div_clk(c);
	u32 i, div, src_sel;

	md->ops->get_src_div(md, &src_sel, &div);

	md->data.div = div;
	md->src_sel = src_sel;

	for (i = 0; i < md->num_parents; i++) {
		if (md->parents[i].sel == src_sel)
			return md->parents[i].src;
	}

	return NULL;
}

static enum handoff mux_div_clk_handoff(struct clk *c)
{
	struct mux_div_clk *md = to_mux_div_clk(c);
	unsigned long parent_rate;

	parent_rate = clk_get_rate(c->parent);
	c->rate = parent_rate / md->data.div;

	if (!md->ops->is_enabled)
		return HANDOFF_DISABLED_CLK;
	if (md->ops->is_enabled(md))
		return HANDOFF_ENABLED_CLK;
	return HANDOFF_DISABLED_CLK;
}

struct clk_ops clk_ops_mux_div_clk = {
	.enable = mux_div_clk_enable,
	.disable = mux_div_clk_disable,
	.set_rate = mux_div_clk_set_rate,
	.round_rate = mux_div_clk_round_rate,
	.get_parent = mux_div_clk_get_parent,
	.handoff = mux_div_clk_handoff,
};
