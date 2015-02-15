/*
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/module.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/cpr-regulator.h>
#include <mach/scm.h>


#define REG_RBCPR_VERSION		0
#define RBCPR_VER_2			0x02

#define REG_RBCPR_GCNT_TARGET(n)	(0x60 + 4 * n)

#define RBCPR_GCNT_TARGET_GCNT_BITS	10
#define RBCPR_GCNT_TARGET_GCNT_SHIFT	12
#define RBCPR_GCNT_TARGET_GCNT_MASK	((1<<RBCPR_GCNT_TARGET_GCNT_BITS)-1)

#define REG_RBCPR_TIMER_INTERVAL	0x44
#define REG_RBIF_TIMER_ADJUST		0x4C

#define RBIF_TIMER_ADJ_CONS_UP_BITS	4
#define RBIF_TIMER_ADJ_CONS_UP_MASK	((1<<RBIF_TIMER_ADJ_CONS_UP_BITS)-1)
#define RBIF_TIMER_ADJ_CONS_DOWN_BITS	4
#define RBIF_TIMER_ADJ_CONS_DOWN_MASK	((1<<RBIF_TIMER_ADJ_CONS_DOWN_BITS)-1)
#define RBIF_TIMER_ADJ_CONS_DOWN_SHIFT	4

#define REG_RBIF_LIMIT			0x48
#define REG_RBCPR_STEP_QUOT		0x80
#define REG_RBIF_SW_VLEVEL		0x94

#define RBIF_LIMIT_CEILING_BITS		6
#define RBIF_LIMIT_CEILING_MASK		((1<<RBIF_LIMIT_CEILING_BITS)-1)
#define RBIF_LIMIT_CEILING_SHIFT	6
#define RBIF_LIMIT_FLOOR_BITS		6
#define RBIF_LIMIT_FLOOR_MASK		((1<<RBIF_LIMIT_FLOOR_BITS)-1)

#define RBIF_LIMIT_CEILING_DEFAULT	RBIF_LIMIT_CEILING_MASK
#define RBIF_LIMIT_FLOOR_DEFAULT	0
#define RBIF_SW_VLEVEL_DEFAULT		0x20

#define RBCPR_STEP_QUOT_STEPQUOT_BITS	8
#define RBCPR_STEP_QUOT_STEPQUOT_MASK	((1<<RBCPR_STEP_QUOT_STEPQUOT_BITS)-1)
#define RBCPR_STEP_QUOT_IDLE_CLK_BITS	4
#define RBCPR_STEP_QUOT_IDLE_CLK_MASK	((1<<RBCPR_STEP_QUOT_IDLE_CLK_BITS)-1)
#define RBCPR_STEP_QUOT_IDLE_CLK_SHIFT	8

#define REG_RBCPR_CTL			0x90

#define RBCPR_CTL_LOOP_EN			BIT(0)
#define RBCPR_CTL_TIMER_EN			BIT(3)
#define RBCPR_CTL_SW_AUTO_CONT_ACK_EN		BIT(5)
#define RBCPR_CTL_SW_AUTO_CONT_NACK_DN_EN	BIT(6)
#define RBCPR_CTL_COUNT_MODE			BIT(10)
#define RBCPR_CTL_UP_THRESHOLD_BITS	4
#define RBCPR_CTL_UP_THRESHOLD_MASK	((1<<RBCPR_CTL_UP_THRESHOLD_BITS)-1)
#define RBCPR_CTL_UP_THRESHOLD_SHIFT	24
#define RBCPR_CTL_DN_THRESHOLD_BITS	4
#define RBCPR_CTL_DN_THRESHOLD_MASK	((1<<RBCPR_CTL_DN_THRESHOLD_BITS)-1)
#define RBCPR_CTL_DN_THRESHOLD_SHIFT	28

#define REG_RBIF_CONT_ACK_CMD		0x98
#define REG_RBIF_CONT_NACK_CMD		0x9C

#define REG_RBCPR_RESULT_0		0xA0

#define RBCPR_RESULT0_BUSY_SHIFT	19
#define RBCPR_RESULT0_BUSY_MASK		BIT(RBCPR_RESULT0_BUSY_SHIFT)
#define RBCPR_RESULT0_ERROR_STEPS_SHIFT	2
#define RBCPR_RESULT0_ERROR_STEPS_BITS	4
#define RBCPR_RESULT0_ERROR_STEPS_MASK	((1<<RBCPR_RESULT0_ERROR_STEPS_BITS)-1)

#define REG_RBIF_IRQ_EN(n)		(0x100 + 4 * n)
#define REG_RBIF_IRQ_CLEAR		0x110
#define REG_RBIF_IRQ_STATUS		0x114

#define CPR_INT_DONE		BIT(0)
#define CPR_INT_MIN		BIT(1)
#define CPR_INT_DOWN		BIT(2)
#define CPR_INT_MID		BIT(3)
#define CPR_INT_UP		BIT(4)
#define CPR_INT_MAX		BIT(5)
#define CPR_INT_CLAMP		BIT(6)
#define CPR_INT_ALL	(CPR_INT_DONE | CPR_INT_MIN | CPR_INT_DOWN | \
			CPR_INT_MID | CPR_INT_UP | CPR_INT_MAX | CPR_INT_CLAMP)
#define CPR_INT_DEFAULT	(CPR_INT_UP | CPR_INT_DOWN)

#define CPR_NUM_RING_OSC	8

#define RBCPR_CLK_SEL_MASK	BIT(0)
#define RBCPR_CLK_SEL_19P2_MHZ	0
#define RBCPR_CLK_SEL_AHB_CLK	BIT(0)

#define CPR_FUSE_TARGET_QUOT_BITS	12
#define CPR_FUSE_TARGET_QUOT_BITS_MASK	((1<<CPR_FUSE_TARGET_QUOT_BITS)-1)
#define CPR_FUSE_RO_SEL_BITS		3
#define CPR_FUSE_RO_SEL_BITS_MASK	((1<<CPR_FUSE_RO_SEL_BITS)-1)

#define CPR_FUSE_MIN_QUOT_DIFF		100

#define BYTES_PER_FUSE_ROW		8

#define FLAGS_IGNORE_1ST_IRQ_STATUS	BIT(0)
#define FLAGS_SET_MIN_VOLTAGE		BIT(1)
#define FLAGS_UPLIFT_QUOT_VOLT		BIT(2)

struct quot_adjust_info {
	int speed_bin;
	int virtual_corner;
	int quot_adjust;
};

static const char * const vdd_apc_name[] =	{"vdd-apc-optional-prim",
						"vdd-apc-optional-sec",
						"vdd-apc"};

enum voltage_change_dir {
	NO_CHANGE,
	DOWN,
	UP,
};

struct cpr_regulator {
	struct regulator_desc		rdesc;
	struct regulator_dev		*rdev;
	bool				vreg_enabled;
	int				corner;
	int				ceiling_max;

	
	phys_addr_t	efuse_addr;
	void __iomem	*efuse_base;

	
	u32		pvs_corner_v[CPR_FUSE_CORNER_MAX];
	
	u32		pvs_bin;
	u32		speed_bin;
	u32		pvs_version;

	
	struct regulator	*vdd_apc;

	
	struct regulator	*vdd_mx;
	int			vdd_mx_vmax;
	int			vdd_mx_vmin_method;
	int			vdd_mx_vmin;
	int			vdd_mx_corner_map[CPR_FUSE_CORNER_MAX];

	
	struct regulator	*mem_acc_vreg;

	
	u64		cpr_fuse_bits;
	bool		cpr_fuse_disable;
	bool		cpr_fuse_local;
	int		cpr_fuse_target_quot[CPR_FUSE_CORNER_MAX];
	int		cpr_fuse_ro_sel[CPR_FUSE_CORNER_MAX];
	int		gcnt;

	unsigned int	cpr_irq;
	void __iomem	*rbcpr_base;
	phys_addr_t	rbcpr_clk_addr;
	struct mutex	cpr_mutex;

	int		ceiling_volt[CPR_FUSE_CORNER_MAX];
	int		floor_volt[CPR_FUSE_CORNER_MAX];
	int		*last_volt;
	int		step_volt;

	int		*save_ctl;
	int		*save_irq;

	
	bool		enable;
	u32		ref_clk_khz;
	u32		timer_delay_us;
	u32		timer_cons_up;
	u32		timer_cons_down;
	u32		irq_line;
	u32		step_quotient;
	u32		up_threshold;
	u32		down_threshold;
	u32		idle_clocks;
	u32		gcnt_time_us;
	u32		vdd_apc_step_up_limit;
	u32		vdd_apc_step_down_limit;
	u32		flags;
	int		*corner_map;
	u32		num_corners;
	int		*quot_adjust;
	bool		is_cpr_suspended;

#ifdef CONFIG_ARCH_DUMMY
	
	u32		htc_pvs_corner_v[CPR_FUSE_CORNER_MAX];
#endif

};

#define CPR_DEBUG_MASK_IRQ	BIT(0)
#define CPR_DEBUG_MASK_API	BIT(1)

static int cpr_debug_enable = CPR_DEBUG_MASK_IRQ;
static int cpr_enable;
static struct cpr_regulator *the_cpr;

module_param_named(debug_enable, cpr_debug_enable, int, S_IRUGO | S_IWUSR);
#define cpr_debug(message, ...) \
	do { \
		if (cpr_debug_enable & CPR_DEBUG_MASK_API) \
			pr_info(message, ##__VA_ARGS__); \
	} while (0)
#define cpr_debug_irq(message, ...) \
	do { \
		if (cpr_debug_enable & CPR_DEBUG_MASK_IRQ) \
			pr_info(message, ##__VA_ARGS__); \
		else \
			pr_debug(message, ##__VA_ARGS__); \
	} while (0)


static u64 cpr_read_efuse_row(struct cpr_regulator *cpr_vreg, u32 row_num,
				bool use_tz_api)
{
	int rc;
	u64 efuse_bits;
	struct cpr_read_req {
		u32 row_address;
		int addr_type;
	} req;

	struct cpr_read_rsp {
		u32 row_data[2];
		u32 status;
	} rsp;

	if (!use_tz_api) {
		efuse_bits = readll_relaxed(cpr_vreg->efuse_base
			+ row_num * BYTES_PER_FUSE_ROW);
		return efuse_bits;
	}

	req.row_address = cpr_vreg->efuse_addr + row_num * BYTES_PER_FUSE_ROW;
	req.addr_type = 0;
	efuse_bits = 0;

	rc = scm_call(SCM_SVC_FUSE, SCM_FUSE_READ,
			&req, sizeof(req), &rsp, sizeof(rsp));

	if (rc) {
		pr_err("read row %d failed, err code = %d", row_num, rc);
	} else {
		efuse_bits = ((u64)(rsp.row_data[1]) << 32) +
				(u64)rsp.row_data[0];
	}

	return efuse_bits;
}


static bool cpr_is_allowed(struct cpr_regulator *cpr_vreg)
{
	if (cpr_vreg->cpr_fuse_disable || !cpr_enable)
		return false;
	else
		return true;
}

static void cpr_write(struct cpr_regulator *cpr_vreg, u32 offset, u32 value)
{
	writel_relaxed(value, cpr_vreg->rbcpr_base + offset);
}

static u32 cpr_read(struct cpr_regulator *cpr_vreg, u32 offset)
{
	return readl_relaxed(cpr_vreg->rbcpr_base + offset);
}

static void cpr_masked_write(struct cpr_regulator *cpr_vreg, u32 offset,
			     u32 mask, u32 value)
{
	u32 reg_val;

	reg_val = readl_relaxed(cpr_vreg->rbcpr_base + offset);
	reg_val &= ~mask;
	reg_val |= value & mask;
	writel_relaxed(reg_val, cpr_vreg->rbcpr_base + offset);
}

static void cpr_irq_clr(struct cpr_regulator *cpr_vreg)
{
	cpr_write(cpr_vreg, REG_RBIF_IRQ_CLEAR, CPR_INT_ALL);
}

static void cpr_irq_clr_nack(struct cpr_regulator *cpr_vreg)
{
	cpr_irq_clr(cpr_vreg);
	cpr_write(cpr_vreg, REG_RBIF_CONT_NACK_CMD, 1);
}

static void cpr_irq_clr_ack(struct cpr_regulator *cpr_vreg)
{
	cpr_irq_clr(cpr_vreg);
	cpr_write(cpr_vreg, REG_RBIF_CONT_ACK_CMD, 1);
}

static void cpr_irq_set(struct cpr_regulator *cpr_vreg, u32 int_bits)
{
	cpr_write(cpr_vreg, REG_RBIF_IRQ_EN(cpr_vreg->irq_line), int_bits);
}

static void cpr_ctl_modify(struct cpr_regulator *cpr_vreg, u32 mask, u32 value)
{
	cpr_masked_write(cpr_vreg, REG_RBCPR_CTL, mask, value);
}

static void cpr_ctl_enable(struct cpr_regulator *cpr_vreg, int corner)
{
	u32 val;
	int fuse_corner = cpr_vreg->corner_map[corner];

	if (cpr_vreg->is_cpr_suspended)
		return;

	
	val = ((cpr_vreg->timer_cons_down & RBIF_TIMER_ADJ_CONS_DOWN_MASK)
			<< RBIF_TIMER_ADJ_CONS_DOWN_SHIFT) |
		(cpr_vreg->timer_cons_up & RBIF_TIMER_ADJ_CONS_UP_MASK);
	cpr_masked_write(cpr_vreg, REG_RBIF_TIMER_ADJUST,
			RBIF_TIMER_ADJ_CONS_UP_MASK |
			RBIF_TIMER_ADJ_CONS_DOWN_MASK, val);
	cpr_masked_write(cpr_vreg, REG_RBCPR_CTL,
			RBCPR_CTL_SW_AUTO_CONT_NACK_DN_EN |
			RBCPR_CTL_SW_AUTO_CONT_ACK_EN,
			cpr_vreg->save_ctl[corner]);
	cpr_irq_set(cpr_vreg, cpr_vreg->save_irq[corner]);

	if (cpr_is_allowed(cpr_vreg) &&
	    (cpr_vreg->ceiling_volt[fuse_corner] >
		cpr_vreg->floor_volt[fuse_corner]))
		val = RBCPR_CTL_LOOP_EN;
	else
		val = 0;
	cpr_ctl_modify(cpr_vreg, RBCPR_CTL_LOOP_EN, val);
}

static void cpr_ctl_disable(struct cpr_regulator *cpr_vreg)
{
	if (cpr_vreg->is_cpr_suspended)
		return;

	cpr_irq_set(cpr_vreg, 0);
	cpr_ctl_modify(cpr_vreg, RBCPR_CTL_SW_AUTO_CONT_NACK_DN_EN |
			RBCPR_CTL_SW_AUTO_CONT_ACK_EN, 0);
	cpr_masked_write(cpr_vreg, REG_RBIF_TIMER_ADJUST,
			RBIF_TIMER_ADJ_CONS_UP_MASK |
			RBIF_TIMER_ADJ_CONS_DOWN_MASK, 0);
	cpr_irq_clr(cpr_vreg);
	cpr_write(cpr_vreg, REG_RBIF_CONT_ACK_CMD, 1);
	cpr_write(cpr_vreg, REG_RBIF_CONT_NACK_CMD, 1);
	cpr_ctl_modify(cpr_vreg, RBCPR_CTL_LOOP_EN, 0);
}

static bool cpr_ctl_is_enabled(struct cpr_regulator *cpr_vreg)
{
	u32 reg_val;

	reg_val = cpr_read(cpr_vreg, REG_RBCPR_CTL);
	return reg_val & RBCPR_CTL_LOOP_EN;
}

static bool cpr_ctl_is_busy(struct cpr_regulator *cpr_vreg)
{
	u32 reg_val;

	reg_val = cpr_read(cpr_vreg, REG_RBCPR_RESULT_0);
	return reg_val & RBCPR_RESULT0_BUSY_MASK;
}

static void cpr_corner_save(struct cpr_regulator *cpr_vreg, int corner)
{
	cpr_vreg->save_ctl[corner] = cpr_read(cpr_vreg, REG_RBCPR_CTL);
	cpr_vreg->save_irq[corner] =
		cpr_read(cpr_vreg, REG_RBIF_IRQ_EN(cpr_vreg->irq_line));
}

static void cpr_corner_restore(struct cpr_regulator *cpr_vreg, int corner)
{
	u32 gcnt, ctl, irq, ro_sel;
	int fuse_corner = cpr_vreg->corner_map[corner];

	ro_sel = cpr_vreg->cpr_fuse_ro_sel[fuse_corner];
	gcnt = cpr_vreg->gcnt | (cpr_vreg->cpr_fuse_target_quot[fuse_corner] -
					cpr_vreg->quot_adjust[corner]);

	cpr_write(cpr_vreg, REG_RBCPR_GCNT_TARGET(ro_sel), gcnt);
	ctl = cpr_vreg->save_ctl[corner];
	cpr_write(cpr_vreg, REG_RBCPR_CTL, ctl);
	irq = cpr_vreg->save_irq[corner];
	cpr_irq_set(cpr_vreg, irq);
	cpr_debug("gcnt = 0x%08x, ctl = 0x%08x, irq = 0x%08x\n",
		  gcnt, ctl, irq);
}

static void cpr_corner_switch(struct cpr_regulator *cpr_vreg, int corner)
{
	if (cpr_vreg->corner == corner)
		return;

	cpr_corner_restore(cpr_vreg, corner);
}

static int cpr_enable_param_set(const char *val, const struct kernel_param *kp)
{
	int rc;
	int old_cpr_enable;

	if (!the_cpr) {
		pr_err("the_cpr = NULL\n");
		return -ENXIO;
	}

	mutex_lock(&the_cpr->cpr_mutex);

	old_cpr_enable = cpr_enable;
	rc = param_set_int(val, kp);
	if (rc) {
		pr_err("param_set_int: rc = %d\n", rc);
		goto _exit;
	}

	cpr_debug("%d -> %d [corner=%d, fuse_corner=%d]\n",
		  old_cpr_enable, cpr_enable, the_cpr->corner,
		  the_cpr->corner_map[the_cpr->corner]);

	if (the_cpr->cpr_fuse_disable) {
		
		pr_info("CPR disabled by fuse\n");
		goto _exit;
	}

	if ((old_cpr_enable != cpr_enable) && the_cpr->corner) {
		if (cpr_enable) {
			cpr_ctl_disable(the_cpr);
			cpr_irq_clr(the_cpr);
			cpr_corner_restore(the_cpr, the_cpr->corner);
			cpr_ctl_enable(the_cpr, the_cpr->corner);
		} else {
			cpr_ctl_disable(the_cpr);
			cpr_irq_set(the_cpr, 0);
		}
	}

_exit:
	mutex_unlock(&the_cpr->cpr_mutex);
	return 0;
}

static struct kernel_param_ops cpr_enable_ops = {
	.set = cpr_enable_param_set,
	.get = param_get_int,
};

module_param_cb(cpr_enable, &cpr_enable_ops, &cpr_enable, S_IRUGO | S_IWUSR);

static int cpr_apc_set(struct cpr_regulator *cpr_vreg, u32 new_volt)
{
	int max_volt, rc;

	max_volt = cpr_vreg->ceiling_max;
	rc = regulator_set_voltage(cpr_vreg->vdd_apc, new_volt, max_volt);
	if (rc)
		pr_err("set: vdd_apc = %d uV: rc=%d\n", new_volt, rc);
	return rc;
}

static int cpr_mx_get(struct cpr_regulator *cpr_vreg, int corner, int apc_volt)
{
	int vdd_mx;
	int fuse_corner = cpr_vreg->corner_map[corner];

	switch (cpr_vreg->vdd_mx_vmin_method) {
	case VDD_MX_VMIN_APC:
		vdd_mx = apc_volt;
		break;
	case VDD_MX_VMIN_APC_CORNER_CEILING:
		vdd_mx = cpr_vreg->ceiling_volt[fuse_corner];
		break;
	case VDD_MX_VMIN_APC_SLOW_CORNER_CEILING:
		vdd_mx = cpr_vreg->ceiling_volt[CPR_FUSE_CORNER_TURBO];
		break;
	case VDD_MX_VMIN_MX_VMAX:
		vdd_mx = cpr_vreg->vdd_mx_vmax;
		break;
	case VDD_MX_VMIN_APC_CORNER_MAP:
		vdd_mx = cpr_vreg->vdd_mx_corner_map[fuse_corner];
		break;
	default:
		vdd_mx = 0;
		break;
	}

	return vdd_mx;
}

static int cpr_mx_set(struct cpr_regulator *cpr_vreg, int corner,
		      int vdd_mx_vmin)
{
	int rc;
	int fuse_corner = cpr_vreg->corner_map[corner];

	rc = regulator_set_voltage(cpr_vreg->vdd_mx, vdd_mx_vmin,
				   cpr_vreg->vdd_mx_vmax);
	cpr_debug("[corner:%d, fuse_corner:%d] %d uV %d uV\n", corner,
			fuse_corner, vdd_mx_vmin, cpr_vreg->vdd_mx_vmax);

	if (!rc) {
		cpr_vreg->vdd_mx_vmin = vdd_mx_vmin;
	} else {
		pr_err("set: vdd_mx [corner:%d, fuse_corner:%d] = %d uV failed: rc=%d\n",
			corner, fuse_corner, vdd_mx_vmin, rc);
	}
	return rc;
}

static int cpr_scale_voltage(struct cpr_regulator *cpr_vreg, int corner,
			     int new_apc_volt, enum voltage_change_dir dir)
{
	int rc = 0, vdd_mx_vmin = 0;
	int fuse_corner = cpr_vreg->corner_map[corner];

	
	if (dir != NO_CHANGE && cpr_vreg->vdd_mx != NULL)
		vdd_mx_vmin = cpr_mx_get(cpr_vreg, corner, new_apc_volt);

	if (cpr_vreg->mem_acc_vreg && dir == DOWN)
		rc = regulator_set_voltage(cpr_vreg->mem_acc_vreg,
					fuse_corner, fuse_corner);

	if (vdd_mx_vmin && dir == UP) {
		if (vdd_mx_vmin != cpr_vreg->vdd_mx_vmin)
			rc = cpr_mx_set(cpr_vreg, corner, vdd_mx_vmin);
	}

	if (!rc)
		rc = cpr_apc_set(cpr_vreg, new_apc_volt);

	if (!rc && cpr_vreg->mem_acc_vreg && dir == UP)
		rc = regulator_set_voltage(cpr_vreg->mem_acc_vreg,
					fuse_corner, fuse_corner);

	if (!rc && vdd_mx_vmin && dir == DOWN) {
		if (vdd_mx_vmin != cpr_vreg->vdd_mx_vmin)
			rc = cpr_mx_set(cpr_vreg, corner, vdd_mx_vmin);
	}

	return rc;
}

static void cpr_scale(struct cpr_regulator *cpr_vreg,
		      enum voltage_change_dir dir)
{
	u32 reg_val, error_steps, reg_mask;
	int last_volt, new_volt, corner, fuse_corner;
	u32 gcnt, quot;

	corner = cpr_vreg->corner;
	fuse_corner = cpr_vreg->corner_map[corner];

	reg_val = cpr_read(cpr_vreg, REG_RBCPR_RESULT_0);

	error_steps = (reg_val >> RBCPR_RESULT0_ERROR_STEPS_SHIFT)
				& RBCPR_RESULT0_ERROR_STEPS_MASK;
	last_volt = cpr_vreg->last_volt[corner];

	cpr_debug_irq("last_volt[corner:%d, fuse_corner:%d] = %d uV\n", corner,
			fuse_corner, last_volt);

	gcnt = cpr_read(cpr_vreg, REG_RBCPR_GCNT_TARGET
			(cpr_vreg->cpr_fuse_ro_sel[fuse_corner]));
	quot = gcnt & ((1 << RBCPR_GCNT_TARGET_GCNT_SHIFT) - 1);

	if (dir == UP) {
		cpr_debug_irq("Up: cpr status = 0x%08x (error_steps=%d)\n",
			      reg_val, error_steps);

		if (last_volt >= cpr_vreg->ceiling_volt[fuse_corner]) {
			cpr_debug_irq(
			"[corn:%d, fuse_corn:%d] @ ceiling: %d >= %d: NACK\n",
				corner, fuse_corner, last_volt,
				cpr_vreg->ceiling_volt[fuse_corner]);
			cpr_irq_clr_nack(cpr_vreg);

			cpr_debug_irq("gcnt = 0x%08x (quot = %d)\n", gcnt,
					quot);

			
			reg_mask = RBCPR_CTL_UP_THRESHOLD_MASK <<
					RBCPR_CTL_UP_THRESHOLD_SHIFT;
			reg_val = reg_mask;
			cpr_ctl_modify(cpr_vreg, reg_mask, reg_val);

			
			cpr_irq_set(cpr_vreg, CPR_INT_DEFAULT & ~CPR_INT_UP);

			return;
		}

		if (error_steps > cpr_vreg->vdd_apc_step_up_limit) {
			cpr_debug_irq("%d is over up-limit(%d): Clamp\n",
				      error_steps,
				      cpr_vreg->vdd_apc_step_up_limit);
			error_steps = cpr_vreg->vdd_apc_step_up_limit;
		}

		
		new_volt = last_volt + (error_steps * cpr_vreg->step_volt);
		if (new_volt > cpr_vreg->ceiling_volt[fuse_corner]) {
			cpr_debug_irq("new_volt(%d) >= ceiling(%d): Clamp\n",
				      new_volt,
				      cpr_vreg->ceiling_volt[fuse_corner]);

			new_volt = cpr_vreg->ceiling_volt[fuse_corner];
		}

		if (cpr_scale_voltage(cpr_vreg, corner, new_volt, dir)) {
			cpr_irq_clr_nack(cpr_vreg);
			return;
		}
		cpr_vreg->last_volt[corner] = new_volt;

		
		reg_mask = RBCPR_CTL_SW_AUTO_CONT_NACK_DN_EN;
		reg_val = 0;

		cpr_ctl_modify(cpr_vreg, reg_mask, reg_val);

		
		cpr_irq_set(cpr_vreg, CPR_INT_DEFAULT);

		
		cpr_irq_clr_ack(cpr_vreg);

		cpr_debug_irq(
			"UP: -> new_volt[corner:%d, fuse_corner:%d] = %d uV\n",
			corner, fuse_corner, new_volt);
	} else if (dir == DOWN) {
		cpr_debug_irq("Down: cpr status = 0x%08x (error_steps=%d)\n",
			      reg_val, error_steps);

		if (last_volt <= cpr_vreg->floor_volt[fuse_corner]) {
			cpr_debug_irq(
			"[corn:%d, fuse_corner:%d] @ floor: %d <= %d: NACK\n",
				corner, fuse_corner, last_volt,
				cpr_vreg->floor_volt[fuse_corner]);
			cpr_irq_clr_nack(cpr_vreg);

			cpr_debug_irq("gcnt = 0x%08x (quot = %d)\n", gcnt,
					quot);

			
			reg_mask = RBCPR_CTL_SW_AUTO_CONT_NACK_DN_EN;
			reg_val = RBCPR_CTL_SW_AUTO_CONT_NACK_DN_EN;

			cpr_ctl_modify(cpr_vreg, reg_mask, reg_val);

			
			cpr_irq_set(cpr_vreg, CPR_INT_DEFAULT & ~CPR_INT_DOWN);

			return;
		}

		if (error_steps > cpr_vreg->vdd_apc_step_down_limit) {
			cpr_debug_irq("%d is over down-limit(%d): Clamp\n",
				      error_steps,
				      cpr_vreg->vdd_apc_step_down_limit);
			error_steps = cpr_vreg->vdd_apc_step_down_limit;
		}

		
		new_volt = last_volt - (error_steps * cpr_vreg->step_volt);
		if (new_volt < cpr_vreg->floor_volt[fuse_corner]) {
			cpr_debug_irq("new_volt(%d) < floor(%d): Clamp\n",
				      new_volt,
				      cpr_vreg->floor_volt[fuse_corner]);
			new_volt = cpr_vreg->floor_volt[fuse_corner];
		}

		if (cpr_scale_voltage(cpr_vreg, corner, new_volt, dir)) {
			cpr_irq_clr_nack(cpr_vreg);
			return;
		}
		cpr_vreg->last_volt[corner] = new_volt;

		
		reg_mask = RBCPR_CTL_UP_THRESHOLD_MASK <<
				RBCPR_CTL_UP_THRESHOLD_SHIFT;
		reg_val = cpr_vreg->up_threshold <<
				RBCPR_CTL_UP_THRESHOLD_SHIFT;
		cpr_ctl_modify(cpr_vreg, reg_mask, reg_val);

		
		cpr_irq_set(cpr_vreg, CPR_INT_DEFAULT);

		
		cpr_irq_clr_ack(cpr_vreg);

		cpr_debug_irq(
		"DOWN: -> new_volt[corner:%d, fuse_corner:%d] = %d uV\n",
			corner, fuse_corner, new_volt);
	}
}

static irqreturn_t cpr_irq_handler(int irq, void *dev)
{
	struct cpr_regulator *cpr_vreg = dev;
	u32 reg_val;

	mutex_lock(&cpr_vreg->cpr_mutex);

	reg_val = cpr_read(cpr_vreg, REG_RBIF_IRQ_STATUS);
	if (cpr_vreg->flags & FLAGS_IGNORE_1ST_IRQ_STATUS)
		reg_val = cpr_read(cpr_vreg, REG_RBIF_IRQ_STATUS);

	cpr_debug_irq("IRQ_STATUS = 0x%02X\n", reg_val);

	if (!cpr_ctl_is_enabled(cpr_vreg)) {
		cpr_debug_irq("CPR is disabled\n");
		goto _exit;
	} else if (cpr_ctl_is_busy(cpr_vreg)) {
		cpr_debug_irq("CPR measurement is not ready\n");
		goto _exit;
	} else if (!cpr_is_allowed(cpr_vreg)) {
		reg_val = cpr_read(cpr_vreg, REG_RBCPR_CTL);
		pr_err("Interrupt broken? RBCPR_CTL = 0x%02X\n", reg_val);
		goto _exit;
	}

	
	if (reg_val & CPR_INT_UP) {
		cpr_scale(cpr_vreg, UP);
	} else if (reg_val & CPR_INT_DOWN) {
		cpr_scale(cpr_vreg, DOWN);
	} else if (reg_val & CPR_INT_MIN) {
		cpr_irq_clr_nack(cpr_vreg);
	} else if (reg_val & CPR_INT_MAX) {
		cpr_irq_clr_nack(cpr_vreg);
	} else if (reg_val & CPR_INT_MID) {
		
		cpr_debug_irq("IRQ occured for Mid Flag\n");
	} else {
		pr_err("IRQ occured for unknown flag (0x%08x)\n", reg_val);
	}

	
	cpr_corner_save(cpr_vreg, cpr_vreg->corner);

_exit:
	mutex_unlock(&cpr_vreg->cpr_mutex);
	return IRQ_HANDLED;
}

static int cpr_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct cpr_regulator *cpr_vreg = rdev_get_drvdata(rdev);

	return cpr_vreg->vreg_enabled;
}

static int cpr_regulator_enable(struct regulator_dev *rdev)
{
	struct cpr_regulator *cpr_vreg = rdev_get_drvdata(rdev);
	int rc = 0;

	
	if (cpr_vreg->vdd_mx) {
		rc = regulator_enable(cpr_vreg->vdd_mx);
		if (rc) {
			pr_err("regulator_enable: vdd_mx: rc=%d\n", rc);
			return rc;
		}
	}

	rc = regulator_enable(cpr_vreg->vdd_apc);
	if (rc) {
		pr_err("regulator_enable: vdd_apc: rc=%d\n", rc);
		return rc;
	}

	cpr_vreg->vreg_enabled = true;

	mutex_lock(&cpr_vreg->cpr_mutex);
	if (cpr_is_allowed(cpr_vreg) && cpr_vreg->corner) {
		cpr_irq_clr(cpr_vreg);
		cpr_corner_switch(cpr_vreg, cpr_vreg->corner);
		cpr_ctl_enable(cpr_vreg, cpr_vreg->corner);
	}
	mutex_unlock(&cpr_vreg->cpr_mutex);

	return rc;
}

static int cpr_regulator_disable(struct regulator_dev *rdev)
{
	struct cpr_regulator *cpr_vreg = rdev_get_drvdata(rdev);
	int rc;

	rc = regulator_disable(cpr_vreg->vdd_apc);
	if (!rc) {
		if (cpr_vreg->vdd_mx)
			rc = regulator_disable(cpr_vreg->vdd_mx);

		if (rc) {
			pr_err("regulator_disable: vdd_mx: rc=%d\n", rc);
			return rc;
		}

		cpr_vreg->vreg_enabled = false;

		mutex_lock(&cpr_vreg->cpr_mutex);
		if (cpr_is_allowed(cpr_vreg))
			cpr_ctl_disable(cpr_vreg);
		mutex_unlock(&cpr_vreg->cpr_mutex);
	} else {
		pr_err("regulator_disable: vdd_apc: rc=%d\n", rc);
	}

	return rc;
}

#ifdef CONFIG_ARCH_DUMMY
bool htc_pvs_adjust = false;
u32 htc_pvs_adjust_seconds = 0;
#endif

static int cpr_regulator_set_voltage(struct regulator_dev *rdev,
		int corner, int corner_max, unsigned *selector)
{
	struct cpr_regulator *cpr_vreg = rdev_get_drvdata(rdev);
	int rc;
	int new_volt;
	enum voltage_change_dir change_dir = NO_CHANGE;
	int fuse_corner = cpr_vreg->corner_map[corner];

	mutex_lock(&cpr_vreg->cpr_mutex);

	if (cpr_is_allowed(cpr_vreg)) {
		cpr_ctl_disable(cpr_vreg);
		new_volt = cpr_vreg->last_volt[corner];
	} else {
#ifdef CONFIG_ARCH_DUMMY
		if (!htc_pvs_adjust) {
			new_volt = cpr_vreg->pvs_corner_v[fuse_corner];
		} else {
			new_volt = cpr_vreg->htc_pvs_corner_v[fuse_corner];
		}
#else
		new_volt = cpr_vreg->pvs_corner_v[fuse_corner];
#endif
	}

	cpr_debug("[corner:%d, fuse_corner:%d] = %d uV\n", corner, fuse_corner,
		new_volt);

	if (corner > cpr_vreg->corner)
		change_dir = UP;
	else if (corner < cpr_vreg->corner)
		change_dir = DOWN;

	rc = cpr_scale_voltage(cpr_vreg, corner, new_volt, change_dir);
	if (rc)
		goto _exit;

	if (cpr_is_allowed(cpr_vreg) && cpr_vreg->vreg_enabled) {
		cpr_irq_clr(cpr_vreg);
		cpr_corner_switch(cpr_vreg, corner);
		cpr_ctl_enable(cpr_vreg, corner);
	}

	cpr_vreg->corner = corner;

_exit:
	mutex_unlock(&cpr_vreg->cpr_mutex);

	return rc;
}

static int cpr_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct cpr_regulator *cpr_vreg = rdev_get_drvdata(rdev);

	return cpr_vreg->corner;
}

static struct regulator_ops cpr_corner_ops = {
	.enable			= cpr_regulator_enable,
	.disable		= cpr_regulator_disable,
	.is_enabled		= cpr_regulator_is_enabled,
	.set_voltage		= cpr_regulator_set_voltage,
	.get_voltage		= cpr_regulator_get_voltage,
};

#ifdef CONFIG_PM
static int cpr_suspend(struct cpr_regulator *cpr_vreg)
{
	cpr_debug("suspend\n");

	mutex_lock(&cpr_vreg->cpr_mutex);

	cpr_ctl_disable(cpr_vreg);

	cpr_irq_clr(cpr_vreg);

	cpr_vreg->is_cpr_suspended = true;

	mutex_unlock(&cpr_vreg->cpr_mutex);
	return 0;
}

static int cpr_resume(struct cpr_regulator *cpr_vreg)

{
	cpr_debug("resume\n");

	mutex_lock(&cpr_vreg->cpr_mutex);

	cpr_vreg->is_cpr_suspended = false;
	cpr_irq_clr(cpr_vreg);

	cpr_ctl_enable(cpr_vreg, cpr_vreg->corner);

	mutex_unlock(&cpr_vreg->cpr_mutex);
	return 0;
}

static int cpr_regulator_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct cpr_regulator *cpr_vreg = platform_get_drvdata(pdev);

	if (cpr_is_allowed(cpr_vreg))
		return cpr_suspend(cpr_vreg);
	else
		return 0;
}

static int cpr_regulator_resume(struct platform_device *pdev)
{
	struct cpr_regulator *cpr_vreg = platform_get_drvdata(pdev);

	if (cpr_is_allowed(cpr_vreg))
		return cpr_resume(cpr_vreg);
	else
		return 0;
}
#else
#define cpr_regulator_suspend NULL
#define cpr_regulator_resume NULL
#endif

static int __devinit cpr_config(struct cpr_regulator *cpr_vreg,
				struct device *dev)
{
	int i;
	u32 val, gcnt, reg;
	void __iomem *rbcpr_clk;
	int size;

	
	rbcpr_clk = ioremap(cpr_vreg->rbcpr_clk_addr, 4);
	if (!rbcpr_clk) {
		pr_err("Unable to map rbcpr_clk\n");
		return -EINVAL;
	}
	reg = readl_relaxed(rbcpr_clk);
	reg &= ~RBCPR_CLK_SEL_MASK;
	reg |= RBCPR_CLK_SEL_19P2_MHZ & RBCPR_CLK_SEL_MASK;
	writel_relaxed(reg, rbcpr_clk);
	iounmap(rbcpr_clk);

	
	cpr_write(cpr_vreg, REG_RBIF_IRQ_EN(cpr_vreg->irq_line), 0);
	cpr_write(cpr_vreg, REG_RBCPR_CTL, 0);

	
	val = ((RBIF_LIMIT_CEILING_DEFAULT & RBIF_LIMIT_CEILING_MASK)
			<< RBIF_LIMIT_CEILING_SHIFT)
		| (RBIF_LIMIT_FLOOR_DEFAULT & RBIF_LIMIT_FLOOR_MASK);
	cpr_write(cpr_vreg, REG_RBIF_LIMIT, val);
	cpr_write(cpr_vreg, REG_RBIF_SW_VLEVEL, RBIF_SW_VLEVEL_DEFAULT);

	
	for (i = 0; i < CPR_NUM_RING_OSC; i++)
		cpr_write(cpr_vreg, REG_RBCPR_GCNT_TARGET(i), 0);

	
	gcnt = (cpr_vreg->ref_clk_khz * cpr_vreg->gcnt_time_us) / 1000;
	gcnt = (gcnt & RBCPR_GCNT_TARGET_GCNT_MASK) <<
			RBCPR_GCNT_TARGET_GCNT_SHIFT;
	cpr_vreg->gcnt = gcnt;

	
	val = ((cpr_vreg->idle_clocks & RBCPR_STEP_QUOT_IDLE_CLK_MASK)
			<< RBCPR_STEP_QUOT_IDLE_CLK_SHIFT) |
		(cpr_vreg->step_quotient & RBCPR_STEP_QUOT_STEPQUOT_MASK);
	cpr_write(cpr_vreg, REG_RBCPR_STEP_QUOT, val);

	
	val = (cpr_vreg->ref_clk_khz * cpr_vreg->timer_delay_us) / 1000;
	cpr_write(cpr_vreg, REG_RBCPR_TIMER_INTERVAL, val);
	pr_info("Timer count: 0x%0x (for %d us)\n", val,
		cpr_vreg->timer_delay_us);

	
	val = ((cpr_vreg->timer_cons_down & RBIF_TIMER_ADJ_CONS_DOWN_MASK)
			<< RBIF_TIMER_ADJ_CONS_DOWN_SHIFT) |
		(cpr_vreg->timer_cons_up & RBIF_TIMER_ADJ_CONS_UP_MASK);
	cpr_write(cpr_vreg, REG_RBIF_TIMER_ADJUST, val);

	
	cpr_vreg->up_threshold &= RBCPR_CTL_UP_THRESHOLD_MASK;
	cpr_vreg->down_threshold &= RBCPR_CTL_DN_THRESHOLD_MASK;
	val = (cpr_vreg->up_threshold << RBCPR_CTL_UP_THRESHOLD_SHIFT)
		| (cpr_vreg->down_threshold << RBCPR_CTL_DN_THRESHOLD_SHIFT);
	val |= RBCPR_CTL_TIMER_EN | RBCPR_CTL_COUNT_MODE;
	val |= RBCPR_CTL_SW_AUTO_CONT_ACK_EN;
	cpr_write(cpr_vreg, REG_RBCPR_CTL, val);

	cpr_irq_set(cpr_vreg, CPR_INT_DEFAULT);

	val = cpr_read(cpr_vreg, REG_RBCPR_VERSION);
	if (val <= RBCPR_VER_2)
		cpr_vreg->flags |= FLAGS_IGNORE_1ST_IRQ_STATUS;

	size = cpr_vreg->num_corners + 1;
	cpr_vreg->save_ctl = devm_kzalloc(dev, sizeof(int) * size, GFP_KERNEL);
	cpr_vreg->save_irq = devm_kzalloc(dev, sizeof(int) * size, GFP_KERNEL);
	if (!cpr_vreg->save_ctl || !cpr_vreg->save_irq)
		return -ENOMEM;

	for (i = 1; i < size; i++)
		cpr_corner_save(cpr_vreg, i);

	return 0;
}

static int __devinit cpr_fuse_is_setting_expected(struct cpr_regulator *cpr_vreg,
					u32 sel_array[5])
{
	u64 fuse_bits;
	u32 ret;

	fuse_bits = cpr_read_efuse_row(cpr_vreg, sel_array[0], sel_array[4]);
	ret = (fuse_bits >> sel_array[1]) & ((1 << sel_array[2]) - 1);
	if (ret == sel_array[3])
		ret = 1;
	else
		ret = 0;

	pr_info("[row:%d] = 0x%llx @%d:%d == %d ?: %s\n",
			sel_array[0], fuse_bits,
			sel_array[1], sel_array[2],
			sel_array[3],
			(ret == 1) ? "yes" : "no");
	return ret;
}

static int cpr_voltage_uplift_wa_inc_volt(struct cpr_regulator *cpr_vreg,
					struct device_node *of_node)
{
	u32 uplift_voltage;
	u32 uplift_max_volt = 0;
	int rc;

	rc = of_property_read_u32(of_node,
		"qcom,cpr-uplift-voltage", &uplift_voltage);
	if (rc < 0) {
		pr_err("cpr-uplift-voltage is missing, rc = %d", rc);
		return rc;
	}
	rc = of_property_read_u32(of_node,
		"qcom,cpr-uplift-max-volt", &uplift_max_volt);
	if (rc < 0) {
		pr_err("cpr-uplift-max-volt is missing, rc = %d", rc);
		return rc;
	}

	cpr_vreg->pvs_corner_v[CPR_FUSE_CORNER_TURBO] += uplift_voltage;
	if (cpr_vreg->pvs_corner_v[CPR_FUSE_CORNER_TURBO] > uplift_max_volt)
		cpr_vreg->pvs_corner_v[CPR_FUSE_CORNER_TURBO] = uplift_max_volt;

	return rc;
}

static int __devinit cpr_pvs_init(struct platform_device *pdev,
			       struct cpr_regulator *cpr_vreg)
{
	struct device_node *of_node = pdev->dev.of_node;
	u64 efuse_bits;
	int rc, i, stripe_size;
	u32 pvs_fuse[4], pvs_fuse_redun_sel[5];
	bool redundant;
	size_t pvs_bins;
	u32 *tmp;

	rc = of_property_read_u32_array(of_node, "qcom,pvs-fuse-redun-sel",
					pvs_fuse_redun_sel, 5);
	if (rc < 0) {
		pr_err("pvs-fuse-redun-sel missing: rc=%d\n", rc);
		return rc;
	}

	redundant = cpr_fuse_is_setting_expected(cpr_vreg, pvs_fuse_redun_sel);

	if (redundant) {
		rc = of_property_read_u32_array(of_node, "qcom,pvs-fuse-redun",
						pvs_fuse, 4);
		if (rc < 0) {
			pr_err("pvs-fuse-redun missing: rc=%d\n", rc);
			return rc;
		}
	} else {
		rc = of_property_read_u32_array(of_node, "qcom,pvs-fuse",
						pvs_fuse, 4);
		if (rc < 0) {
			pr_err("pvs-fuse missing: rc=%d\n", rc);
			return rc;
		}
	}

	

	efuse_bits = cpr_read_efuse_row(cpr_vreg, pvs_fuse[0], pvs_fuse[3]);
	cpr_vreg->pvs_bin = (efuse_bits >> pvs_fuse[1]) &
				   ((1 << pvs_fuse[2]) - 1);

	pvs_bins = 1 << pvs_fuse[2];

	stripe_size = CPR_FUSE_CORNER_MAX - 1;
	tmp = kzalloc(sizeof(u32) * pvs_bins * stripe_size, GFP_KERNEL);
	if (!tmp) {
		pr_err("memory alloc failed\n");
		return -ENOMEM;
	}

	rc = of_property_read_u32_array(of_node, "qcom,pvs-voltage-table",
					tmp, pvs_bins * stripe_size);
	if (rc < 0) {
		pr_err("pvs-voltage-table missing: rc=%d\n", rc);
		kfree(tmp);
		return rc;
	}

	for (i = CPR_FUSE_CORNER_SVS; i < CPR_FUSE_CORNER_MAX; i++)
		cpr_vreg->pvs_corner_v[i] = tmp[cpr_vreg->pvs_bin *
						stripe_size + i - 1];
	kfree(tmp);

	if (cpr_vreg->flags & FLAGS_UPLIFT_QUOT_VOLT) {
		rc = cpr_voltage_uplift_wa_inc_volt(cpr_vreg, of_node);
		if (rc < 0) {
			pr_err("pvs volt uplift wa apply failed: %d", rc);
			return rc;
		}
	}

	if (cpr_vreg->pvs_corner_v[CPR_FUSE_CORNER_TURBO] >
		cpr_vreg->ceiling_volt[CPR_FUSE_CORNER_TURBO])
		cpr_vreg->ceiling_volt[CPR_FUSE_CORNER_TURBO] =
			cpr_vreg->pvs_corner_v[CPR_FUSE_CORNER_TURBO];

	for (i = CPR_FUSE_CORNER_SVS; i < CPR_FUSE_CORNER_TURBO; i++)
		if (cpr_vreg->pvs_corner_v[i] > cpr_vreg->ceiling_volt[i])
			cpr_vreg->pvs_corner_v[i] = cpr_vreg->ceiling_volt[i];
		else if (cpr_vreg->pvs_corner_v[i] < cpr_vreg->floor_volt[i])
			cpr_vreg->pvs_corner_v[i] = cpr_vreg->floor_volt[i];

	cpr_vreg->ceiling_max = cpr_vreg->ceiling_volt[CPR_FUSE_CORNER_TURBO];

	pr_info("pvs voltage: [%d %d %d] uV\n",
			cpr_vreg->pvs_corner_v[CPR_FUSE_CORNER_SVS],
			cpr_vreg->pvs_corner_v[CPR_FUSE_CORNER_NORMAL],
			cpr_vreg->pvs_corner_v[CPR_FUSE_CORNER_TURBO]);
	pr_info("ceiling voltage: [%d %d %d] uV\n",
			cpr_vreg->ceiling_volt[CPR_FUSE_CORNER_SVS],
			cpr_vreg->ceiling_volt[CPR_FUSE_CORNER_NORMAL],
			cpr_vreg->ceiling_volt[CPR_FUSE_CORNER_TURBO]);
	pr_info("floor voltage: [%d %d %d] uV\n",
			cpr_vreg->floor_volt[CPR_FUSE_CORNER_SVS],
			cpr_vreg->floor_volt[CPR_FUSE_CORNER_NORMAL],
			cpr_vreg->floor_volt[CPR_FUSE_CORNER_TURBO]);

	return 0;
}

#define CPR_PROP_READ_U32(of_node, cpr_property, cpr_config, rc)	\
do {									\
	if (!rc) {							\
		rc = of_property_read_u32(of_node,			\
				"qcom," cpr_property,			\
				cpr_config);				\
		if (rc) {						\
			pr_err("Missing " #cpr_property			\
				": rc = %d\n", rc);			\
		}							\
	}								\
} while (0)

static int __devinit cpr_apc_init(struct platform_device *pdev,
			       struct cpr_regulator *cpr_vreg)
{
	struct device_node *of_node = pdev->dev.of_node;
	int i, rc = 0;

	for (i = 0; i < ARRAY_SIZE(vdd_apc_name); i++) {
		cpr_vreg->vdd_apc = devm_regulator_get(&pdev->dev,
					vdd_apc_name[i]);
		rc = PTR_RET(cpr_vreg->vdd_apc);
		if (!IS_ERR_OR_NULL(cpr_vreg->vdd_apc))
			break;
	}

	if (rc) {
		if (rc != -EPROBE_DEFER)
			pr_err("devm_regulator_get: rc=%d\n", rc);
		return rc;
	}

	
	if (of_property_read_bool(of_node, "vdd-mx-supply")) {
		cpr_vreg->vdd_mx = devm_regulator_get(&pdev->dev, "vdd-mx");
		if (IS_ERR_OR_NULL(cpr_vreg->vdd_mx)) {
			rc = PTR_RET(cpr_vreg->vdd_mx);
			if (rc != -EPROBE_DEFER)
				pr_err("devm_regulator_get: vdd-mx: rc=%d\n",
				       rc);
			return rc;
		}
	}

	
	if (cpr_vreg->vdd_mx) {
		rc = of_property_read_u32(of_node, "qcom,vdd-mx-vmax",
				 &cpr_vreg->vdd_mx_vmax);
		if (rc < 0) {
			pr_err("vdd-mx-vmax missing: rc=%d\n", rc);
			return rc;
		}

		rc = of_property_read_u32(of_node, "qcom,vdd-mx-vmin-method",
				 &cpr_vreg->vdd_mx_vmin_method);
		if (rc < 0) {
			pr_err("vdd-mx-vmin-method missing: rc=%d\n", rc);
			return rc;
		}
		if (cpr_vreg->vdd_mx_vmin_method > VDD_MX_VMIN_APC_CORNER_MAP) {
			pr_err("Invalid vdd-mx-vmin-method(%d)\n",
				cpr_vreg->vdd_mx_vmin_method);
			return -EINVAL;
		}

		rc = of_property_read_u32_array(of_node,
					"qcom,vdd-mx-corner-map",
					&cpr_vreg->vdd_mx_corner_map[1],
					CPR_FUSE_CORNER_MAX - 1);
		if (rc && cpr_vreg->vdd_mx_vmin_method ==
			VDD_MX_VMIN_APC_CORNER_MAP) {
			pr_err("qcom,vdd-mx-corner-map missing: rc=%d\n",
				rc);
			return rc;
		}

	}

	return 0;
}

static void cpr_apc_exit(struct cpr_regulator *cpr_vreg)
{
	if (cpr_vreg->vreg_enabled) {
		regulator_disable(cpr_vreg->vdd_apc);

		if (cpr_vreg->vdd_mx)
			regulator_disable(cpr_vreg->vdd_mx);
	}
}

static int cpr_voltage_uplift_wa_inc_quot(struct cpr_regulator *cpr_vreg,
					struct device_node *of_node)
{
	u32 delta_quot[3];
	int rc, i;

	rc = of_property_read_u32_array(of_node,
			"qcom,cpr-uplift-quotient", delta_quot, 3);
	if (rc < 0) {
		pr_err("cpr-uplift-quotient is missing: %d", rc);
		return rc;
	}
	for (i = CPR_FUSE_CORNER_SVS; i < CPR_FUSE_CORNER_MAX; i++)
		cpr_vreg->cpr_fuse_target_quot[i] += delta_quot[i-1];
	return rc;
}

static void cpr_parse_pvs_version_fuse(struct cpr_regulator *cpr_vreg,
				struct device_node *of_node)
{
	int rc;
	u64 fuse_bits;
	u32 fuse_sel[4];

	rc = of_property_read_u32_array(of_node,
			"qcom,pvs-version-fuse-sel", fuse_sel, 4);
	if (!rc) {
		fuse_bits = cpr_read_efuse_row(cpr_vreg,
				fuse_sel[0], fuse_sel[3]);
		cpr_vreg->pvs_version = (fuse_bits >> fuse_sel[1]) &
			((1 << fuse_sel[2]) - 1);
		pr_info("[row: %d]: 0x%llx, pvs_version = %d\n",
				fuse_sel[0], fuse_bits, cpr_vreg->pvs_version);
	} else {
		cpr_vreg->pvs_version = UINT_MAX;
	}
}

static int cpr_get_corner_quot_adjustment(struct cpr_regulator *cpr_vreg,
					struct device *dev)
{
	int rc = 0;
	int i, size;
	struct property *prop;
	bool corners_mapped;
	u32 *tmp, *freq_mappings = NULL;
	u32 scaling, max_factor;
	u32 corner, turbo_corner = 0, normal_corner = 0, svs_corner = 0;
	u32 freq_turbo, freq_normal, freq_corner;

	prop = of_find_property(dev->of_node, "qcom,cpr-corner-map", NULL);

	if (prop) {
		size = prop->length / sizeof(u32);
		corners_mapped = true;
	} else {
		size = CPR_FUSE_CORNER_MAX - 1;
		corners_mapped = false;
	}

	cpr_vreg->corner_map = devm_kzalloc(dev, sizeof(int) * (size + 1),
					GFP_KERNEL);
	if (!cpr_vreg->corner_map) {
		pr_err("Can't allocate memory for cpr_vreg->corner_map\n");
		return -ENOMEM;
	}
	cpr_vreg->num_corners = size;

	cpr_vreg->quot_adjust = devm_kzalloc(dev,
			sizeof(u32) * (cpr_vreg->num_corners + 1),
			GFP_KERNEL);
	if (!cpr_vreg->quot_adjust) {
		pr_err("Can't allocate memory for cpr_vreg->quot_adjust\n");
		return -ENOMEM;
	}

	if (!corners_mapped) {
		for (i = CPR_FUSE_CORNER_SVS; i < CPR_FUSE_CORNER_MAX; i++)
			cpr_vreg->corner_map[i] = i;
		return 0;
	} else {
		rc = of_property_read_u32_array(dev->of_node,
			"qcom,cpr-corner-map", &cpr_vreg->corner_map[1], size);

		if (rc) {
			pr_err("qcom,cpr-corner-map missing, rc = %d\n", rc);
			return rc;
		}
	}

	prop = of_find_property(dev->of_node,
			"qcom,cpr-speed-bin-max-corners", NULL);
	if (!prop) {
		cpr_debug("qcom,cpr-speed-bin-max-corner missing\n");
		return 0;
	}

	size = prop->length / sizeof(u32);
	tmp = kzalloc(size * sizeof(u32), GFP_KERNEL);
	if (!tmp) {
		pr_err("memory alloc failed\n");
		return -ENOMEM;
	}
	rc = of_property_read_u32_array(dev->of_node,
		"qcom,cpr-speed-bin-max-corners", tmp, size);
	if (rc < 0) {
		kfree(tmp);
		pr_err("get cpr-speed-bin-max-corners failed, rc = %d\n", rc);
		return rc;
	}

	cpr_parse_pvs_version_fuse(cpr_vreg, dev->of_node);

	for (i = 0; i < size; i += 5) {
		if (tmp[i] == cpr_vreg->speed_bin &&
			tmp[i + 1] == cpr_vreg->pvs_version) {
			svs_corner = tmp[i + 2];
			normal_corner = tmp[i + 3];
			turbo_corner = tmp[i + 4];
			break;
		}
	}
	kfree(tmp);
	if (turbo_corner <= normal_corner ||
			turbo_corner > cpr_vreg->num_corners) {
		cpr_debug("turbo:%d should be larger than normal:%d\n",
				turbo_corner, normal_corner);
		return 0;
	}

	prop = of_find_property(dev->of_node,
			"qcom,cpr-corner-frequency-map", NULL);
	if (!prop) {
		cpr_debug("qcom,cpr-corner-frequency-map missing\n");
		return 0;
	}

	size = prop->length / sizeof(u32);
	tmp = kzalloc(sizeof(u32) * size, GFP_KERNEL);
	if (!tmp) {
		pr_err("memory alloc failed\n");
		return -ENOMEM;
	}
	rc = of_property_read_u32_array(dev->of_node,
		"qcom,cpr-corner-frequency-map", tmp, size);
	if (rc < 0) {
		pr_err("get cpr-corner-frequency-map failed, rc = %d\n", rc);
		kfree(tmp);
		return rc;
	}
	freq_mappings = kzalloc(sizeof(u32) * (cpr_vreg->num_corners + 1),
			GFP_KERNEL);
	if (!freq_mappings) {
		pr_err("memory alloc for freq_mappings failed!\n");
		kfree(tmp);
		return -ENOMEM;
	}
	for (i = 0; i < size; i += 2) {
		corner = tmp[i];
		if ((corner < 1) || (corner > cpr_vreg->num_corners)) {
			pr_err("corner should be in 1~%d range: %d\n",
					cpr_vreg->num_corners, corner);
			continue;
		}
		freq_mappings[corner] = tmp[i + 1];
		cpr_debug("Frequency at virtual corner %d is %d Hz.\n",
				corner, freq_mappings[corner]);
	}
	kfree(tmp);

	rc = of_property_read_u32(dev->of_node,
		"qcom,cpr-quot-adjust-scaling-factor-max",
		&max_factor);
	if (rc < 0) {
		cpr_debug("get cpr-quot-adjust-scaling-factor-max failed\n");
		kfree(freq_mappings);
		return 0;
	}


	freq_turbo = freq_mappings[turbo_corner];
	freq_normal = freq_mappings[normal_corner];
	if (freq_normal == 0 || freq_turbo <= freq_normal) {
		pr_err("freq_turbo: %d should larger than freq_normal: %d\n",
				freq_turbo, freq_normal);
		kfree(freq_mappings);
		return -EINVAL;
	}
	freq_turbo /= 1000000;	
	freq_normal /= 1000000;
	scaling = 1000 *
		(cpr_vreg->cpr_fuse_target_quot[CPR_FUSE_CORNER_TURBO] -
		cpr_vreg->cpr_fuse_target_quot[CPR_FUSE_CORNER_NORMAL]) /
		(freq_turbo - freq_normal);
	scaling = min(scaling, max_factor);
	pr_info("quotient adjustment scaling factor: %d.%03d\n",
			scaling / 1000, scaling % 1000);

	for (i = turbo_corner; i > normal_corner; i--) {
		freq_corner = freq_mappings[i] / 1000000; 
		if (freq_corner > 0) {
			cpr_vreg->quot_adjust[i] =
				scaling * (freq_turbo - freq_corner) / 1000;
		}
		pr_info("adjusted quotient[%d] = %d\n", i,
			(cpr_vreg->cpr_fuse_target_quot[cpr_vreg->corner_map[i]]
				- cpr_vreg->quot_adjust[i]));
	}
	kfree(freq_mappings);
	return 0;
}

#ifdef CONFIG_HTC_DEBUG_RBCPR_8226
#define CX_FUSE_TABLE_ROW_MSM8226 136
#define CX_FUSE_TABLE_TURBO_MASK_MSM8226 0xF
#define CX_FUSE_TABLE_TURBO_SHFT_MSM8226 0
static short cx_turbo_fuse_8226;
module_param_named(cx_turbo_fuse, cx_turbo_fuse_8226, short, S_IRUSR);
#endif

static int __devinit cpr_init_cpr_efuse(struct platform_device *pdev,
				     struct cpr_regulator *cpr_vreg)
{
	struct device_node *of_node = pdev->dev.of_node;
	int i, rc = 0;
	bool redundant;
	u32 cpr_fuse_redun_sel[5];
	char *targ_quot_str, *ro_sel_str;
	u32 cpr_fuse_row[2];
	u32 bp_cpr_disable, bp_scheme;
	int bp_target_quot[CPR_FUSE_CORNER_MAX];
	int bp_ro_sel[CPR_FUSE_CORNER_MAX];
	u32 ro_sel, val;
	u64 fuse_bits, fuse_bits_2;
	u32 quot_adjust[CPR_FUSE_CORNER_MAX];

	rc = of_property_read_u32_array(of_node, "qcom,cpr-fuse-redun-sel",
					cpr_fuse_redun_sel, 5);
	if (rc < 0) {
		pr_err("cpr-fuse-redun-sel missing: rc=%d\n", rc);
		return rc;
	}

	redundant = cpr_fuse_is_setting_expected(cpr_vreg, cpr_fuse_redun_sel);

	if (redundant) {
		rc = of_property_read_u32_array(of_node,
				"qcom,cpr-fuse-redun-row",
				cpr_fuse_row, 2);
		targ_quot_str = "qcom,cpr-fuse-redun-target-quot";
		ro_sel_str = "qcom,cpr-fuse-redun-ro-sel";
	} else {
		rc = of_property_read_u32_array(of_node,
				"qcom,cpr-fuse-row",
				cpr_fuse_row, 2);
		targ_quot_str = "qcom,cpr-fuse-target-quot";
		ro_sel_str = "qcom,cpr-fuse-ro-sel";
	}
	if (rc)
		return rc;

	rc = of_property_read_u32_array(of_node,
		targ_quot_str,
		&bp_target_quot[CPR_FUSE_CORNER_SVS],
		CPR_FUSE_CORNER_MAX - CPR_FUSE_CORNER_SVS);
	if (rc < 0) {
		pr_err("missing %s: rc=%d\n", targ_quot_str, rc);
		return rc;
	}

	rc = of_property_read_u32_array(of_node,
		ro_sel_str,
		&bp_ro_sel[CPR_FUSE_CORNER_SVS],
		CPR_FUSE_CORNER_MAX - CPR_FUSE_CORNER_SVS);
	if (rc < 0) {
		pr_err("missing %s: rc=%d\n", ro_sel_str, rc);
		return rc;
	}

	
	fuse_bits = cpr_read_efuse_row(cpr_vreg, cpr_fuse_row[0],
					cpr_fuse_row[1]);
	pr_info("[row:%d] = 0x%llx\n", cpr_fuse_row[0], fuse_bits);

	if (redundant) {
		if (of_property_read_bool(of_node,
				"qcom,cpr-fuse-redun-bp-cpr-disable")) {
			CPR_PROP_READ_U32(of_node,
					  "cpr-fuse-redun-bp-cpr-disable",
					  &bp_cpr_disable, rc);
			CPR_PROP_READ_U32(of_node,
					  "cpr-fuse-redun-bp-scheme",
					  &bp_scheme, rc);
			if (rc)
				return rc;
			fuse_bits_2 = fuse_bits;
		} else {
			u32 temp_row[2];

			
			CPR_PROP_READ_U32(of_node, "cpr-fuse-bp-cpr-disable",
					  &bp_cpr_disable, rc);
			CPR_PROP_READ_U32(of_node, "cpr-fuse-bp-scheme",
					  &bp_scheme, rc);
			rc = of_property_read_u32_array(of_node,
					"qcom,cpr-fuse-row",
					temp_row, 2);
			if (rc)
				return rc;

			fuse_bits_2 = cpr_read_efuse_row(cpr_vreg, temp_row[0],
							temp_row[1]);
			pr_info("[original row:%d] = 0x%llx\n",
				temp_row[0], fuse_bits_2);
		}
	} else {
		CPR_PROP_READ_U32(of_node, "cpr-fuse-bp-cpr-disable",
				  &bp_cpr_disable, rc);
		CPR_PROP_READ_U32(of_node, "cpr-fuse-bp-scheme",
				  &bp_scheme, rc);
		if (rc)
			return rc;
		fuse_bits_2 = fuse_bits;
	}

	cpr_vreg->cpr_fuse_disable = (fuse_bits_2 >> bp_cpr_disable) & 0x01;
	cpr_vreg->cpr_fuse_local = (fuse_bits_2 >> bp_scheme) & 0x01;

	pr_info("disable = %d, local = %d\n",
		cpr_vreg->cpr_fuse_disable, cpr_vreg->cpr_fuse_local);

	for (i = CPR_FUSE_CORNER_SVS; i < CPR_FUSE_CORNER_MAX; i++) {
		ro_sel = (fuse_bits >> bp_ro_sel[i])
				& CPR_FUSE_RO_SEL_BITS_MASK;
		val = (fuse_bits >> bp_target_quot[i])
				& CPR_FUSE_TARGET_QUOT_BITS_MASK;
		cpr_vreg->cpr_fuse_target_quot[i] = val;
		cpr_vreg->cpr_fuse_ro_sel[i] = ro_sel;
		pr_info("Corner[%d]: ro_sel = %d, target quot = %d\n",
			i, ro_sel, val);
	}

	rc = of_property_read_u32_array(of_node, "qcom,cpr-quotient-adjustment",
				&quot_adjust[1], CPR_FUSE_CORNER_MAX - 1);
	if (!rc) {
		for (i = CPR_FUSE_CORNER_SVS; i < CPR_FUSE_CORNER_MAX; i++) {
			cpr_vreg->cpr_fuse_target_quot[i] += quot_adjust[i];
			pr_info("Corner[%d]: adjusted target quot = %d\n",
				i, cpr_vreg->cpr_fuse_target_quot[i]);
		}
	}

	if (cpr_vreg->flags & FLAGS_UPLIFT_QUOT_VOLT) {
		cpr_voltage_uplift_wa_inc_quot(cpr_vreg, of_node);
		for (i = CPR_FUSE_CORNER_SVS; i < CPR_FUSE_CORNER_MAX; i++) {
			pr_info("Corner[%d]: uplifted target quot = %d\n",
				i, cpr_vreg->cpr_fuse_target_quot[i]);
		}
	}

	rc = cpr_get_corner_quot_adjustment(cpr_vreg, &pdev->dev);
	if (rc)
		return rc;

	cpr_vreg->cpr_fuse_bits = fuse_bits;
	if (!cpr_vreg->cpr_fuse_bits) {
		cpr_vreg->cpr_fuse_disable = 1;
		pr_err("cpr_fuse_bits = 0: set cpr_fuse_disable = 1\n");
	} else {
		
		int *quot = cpr_vreg->cpr_fuse_target_quot;
		bool valid_fuse = true;

		if ((quot[CPR_FUSE_CORNER_TURBO] >
			quot[CPR_FUSE_CORNER_NORMAL]) &&
		    (quot[CPR_FUSE_CORNER_NORMAL] >
			quot[CPR_FUSE_CORNER_SVS])) {
			if ((quot[CPR_FUSE_CORNER_TURBO] -
			     quot[CPR_FUSE_CORNER_NORMAL])
					<= CPR_FUSE_MIN_QUOT_DIFF)
				valid_fuse = false;
		} else {
			valid_fuse = false;
		}

		if (!valid_fuse) {
			cpr_vreg->cpr_fuse_disable = 1;
			pr_err("invalid quotient values\n");
		}
	}

#ifdef CONFIG_HTC_DEBUG_RBCPR_8226
	fuse_bits = cpr_read_efuse_row(cpr_vreg, CX_FUSE_TABLE_ROW_MSM8226, 0);
	cx_turbo_fuse_8226 = (fuse_bits >> CX_FUSE_TABLE_TURBO_SHFT_MSM8226) & CX_FUSE_TABLE_TURBO_MASK_MSM8226;
#endif
	return 0;
}

static int __devinit cpr_init_cpr_voltages(struct cpr_regulator *cpr_vreg,
			struct device *dev)
{
	int i;
	int size = cpr_vreg->num_corners + 1;

	cpr_vreg->last_volt = devm_kzalloc(dev, sizeof(int) * size, GFP_KERNEL);
	if (!cpr_vreg->last_volt)
		return -EINVAL;

	for (i = 1; i < size; i++) {
		cpr_vreg->last_volt[i] = cpr_vreg->pvs_corner_v
						[cpr_vreg->corner_map[i]];
	}

	return 0;
}

static int __devinit cpr_init_cpr_parameters(struct platform_device *pdev,
					  struct cpr_regulator *cpr_vreg)
{
	struct device_node *of_node = pdev->dev.of_node;
	int rc = 0;

	CPR_PROP_READ_U32(of_node, "cpr-ref-clk",
			  &cpr_vreg->ref_clk_khz, rc);
	if (rc)
		return rc;
	CPR_PROP_READ_U32(of_node, "cpr-timer-delay",
			  &cpr_vreg->timer_delay_us, rc);
	if (rc)
		return rc;
	CPR_PROP_READ_U32(of_node, "cpr-timer-cons-up",
			  &cpr_vreg->timer_cons_up, rc);
	if (rc)
		return rc;
	CPR_PROP_READ_U32(of_node, "cpr-timer-cons-down",
			  &cpr_vreg->timer_cons_down, rc);
	if (rc)
		return rc;
	CPR_PROP_READ_U32(of_node, "cpr-irq-line",
			  &cpr_vreg->irq_line, rc);
	if (rc)
		return rc;
	CPR_PROP_READ_U32(of_node, "cpr-step-quotient",
			  &cpr_vreg->step_quotient, rc);
	if (rc)
		return rc;
	CPR_PROP_READ_U32(of_node, "cpr-up-threshold",
			  &cpr_vreg->up_threshold, rc);
	if (rc)
		return rc;
	CPR_PROP_READ_U32(of_node, "cpr-down-threshold",
			  &cpr_vreg->down_threshold, rc);
	if (rc)
		return rc;
	CPR_PROP_READ_U32(of_node, "cpr-idle-clocks",
			  &cpr_vreg->idle_clocks, rc);
	if (rc)
		return rc;
	CPR_PROP_READ_U32(of_node, "cpr-gcnt-time",
			  &cpr_vreg->gcnt_time_us, rc);
	if (rc)
		return rc;
	CPR_PROP_READ_U32(of_node, "vdd-apc-step-up-limit",
			  &cpr_vreg->vdd_apc_step_up_limit, rc);
	if (rc)
		return rc;
	CPR_PROP_READ_U32(of_node, "vdd-apc-step-down-limit",
			  &cpr_vreg->vdd_apc_step_down_limit, rc);
	if (rc)
		return rc;
	CPR_PROP_READ_U32(of_node, "cpr-apc-volt-step",
			  &cpr_vreg->step_volt, rc);
	if (rc)
		return rc;

	
	cpr_vreg->enable = of_property_read_bool(of_node, "qcom,cpr-enable");
	cpr_enable = (int) cpr_vreg->enable;
	pr_info("CPR is %s by default.\n",
		cpr_vreg->enable ? "enabled" : "disabled");

	return rc;
}

static int __devinit cpr_init_cpr(struct platform_device *pdev,
			       struct cpr_regulator *cpr_vreg)
{
	struct resource *res;
	int rc = 0;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rbcpr_clk");
	if (!res || !res->start) {
		pr_err("missing rbcpr_clk address: res=%p\n", res);
		return -EINVAL;
	}
	cpr_vreg->rbcpr_clk_addr = res->start;

	rc = cpr_init_cpr_efuse(pdev, cpr_vreg);
	if (rc)
		return rc;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rbcpr");
	if (!res || !res->start) {
		pr_err("missing rbcpr address: res=%p\n", res);
		return -EINVAL;
	}
	cpr_vreg->rbcpr_base = devm_ioremap(&pdev->dev, res->start,
					    resource_size(res));

	
	rc = cpr_init_cpr_voltages(cpr_vreg, &pdev->dev);
	if (rc)
		return rc;

	
	rc = cpr_init_cpr_parameters(pdev, cpr_vreg);
	if (rc)
		return rc;

	
	cpr_vreg->cpr_irq = platform_get_irq(pdev, 0);
	if (!cpr_vreg->cpr_irq) {
		pr_err("missing CPR IRQ\n");
		return -EINVAL;
	}

	
	rc = cpr_config(cpr_vreg, &pdev->dev);
	if (rc)
		return rc;

	rc = request_threaded_irq(cpr_vreg->cpr_irq, NULL, cpr_irq_handler,
				  IRQF_TRIGGER_RISING, "cpr", cpr_vreg);
	if (rc) {
		pr_err("CPR: request irq failed for IRQ %d\n",
				cpr_vreg->cpr_irq);
		return rc;
	}

	return 0;
}

static int __devinit cpr_efuse_init(struct platform_device *pdev,
				 struct cpr_regulator *cpr_vreg)
{
	struct resource *res;
	int len;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "efuse_addr");
	if (!res || !res->start) {
		pr_err("efuse_addr missing: res=%p\n", res);
		return -EINVAL;
	}

	cpr_vreg->efuse_addr = res->start;
	len = res->end - res->start + 1;

	pr_info("efuse_addr = %pa (len=0x%x)\n", &res->start, len);

	cpr_vreg->efuse_base = ioremap(cpr_vreg->efuse_addr, len);
	if (!cpr_vreg->efuse_base) {
		pr_err("Unable to map efuse_addr %pa\n",
				&cpr_vreg->efuse_addr);
		return -EINVAL;
	}

	return 0;
}

static void cpr_efuse_free(struct cpr_regulator *cpr_vreg)
{
	iounmap(cpr_vreg->efuse_base);
}

static void cpr_parse_cond_min_volt_fuse(struct cpr_regulator *cpr_vreg,
						struct device_node *of_node)
{
	int rc;
	u32 fuse_sel[5];
	rc = of_property_read_u32_array(of_node,
			"qcom,cpr-fuse-cond-min-volt-sel", fuse_sel, 5);
	if (!rc) {
		if (!cpr_fuse_is_setting_expected(cpr_vreg, fuse_sel))
			cpr_vreg->flags |= FLAGS_SET_MIN_VOLTAGE;
	}
}

static void cpr_parse_speed_bin_fuse(struct cpr_regulator *cpr_vreg,
				struct device_node *of_node)
{
	int rc;
	u64 fuse_bits;
	u32 fuse_sel[4];
	u32 speed_bits;

	rc = of_property_read_u32_array(of_node,
			"qcom,speed-bin-fuse-sel", fuse_sel, 4);

	if (!rc) {
		fuse_bits = cpr_read_efuse_row(cpr_vreg,
				fuse_sel[0], fuse_sel[3]);
		speed_bits = (fuse_bits >> fuse_sel[1]) &
			((1 << fuse_sel[2]) - 1);
		pr_info("[row: %d]: 0x%llx, speed_bits = %d\n",
				fuse_sel[0], fuse_bits, speed_bits);
		cpr_vreg->speed_bin = speed_bits;
	} else {
		cpr_vreg->speed_bin = UINT_MAX;
	}
}

static int cpr_voltage_uplift_enable_check(struct cpr_regulator *cpr_vreg,
					struct device_node *of_node)
{
	int rc;
	u32 fuse_sel[5];
	u32 uplift_speed_bin;

	rc = of_property_read_u32_array(of_node,
			"qcom,cpr-fuse-uplift-sel", fuse_sel, 5);
	if (!rc) {
		rc = of_property_read_u32(of_node,
				"qcom,cpr-uplift-speed-bin",
				&uplift_speed_bin);
		if (rc < 0) {
			pr_err("qcom,cpr-uplift-speed-bin missing\n");
			return rc;
		}
		if (cpr_fuse_is_setting_expected(cpr_vreg, fuse_sel)
			&& (uplift_speed_bin == cpr_vreg->speed_bin)
			&& !(cpr_vreg->flags & FLAGS_SET_MIN_VOLTAGE)) {
			cpr_vreg->flags |= FLAGS_UPLIFT_QUOT_VOLT;
		}
	}
	return 0;
}

static int __devinit cpr_voltage_plan_init(struct platform_device *pdev,
					struct cpr_regulator *cpr_vreg)
{
	struct device_node *of_node = pdev->dev.of_node;
	int rc, i;
	u32 min_uv = 0;

	rc = of_property_read_u32_array(of_node, "qcom,cpr-voltage-ceiling",
		&cpr_vreg->ceiling_volt[CPR_FUSE_CORNER_SVS],
		CPR_FUSE_CORNER_MAX - 1);
	if (rc < 0) {
		pr_err("cpr-voltage-ceiling missing: rc=%d\n", rc);
		return rc;
	}

	rc = of_property_read_u32_array(of_node, "qcom,cpr-voltage-floor",
		&cpr_vreg->floor_volt[CPR_FUSE_CORNER_SVS],
		CPR_FUSE_CORNER_MAX - 1);
	if (rc < 0) {
		pr_err("cpr-voltage-floor missing: rc=%d\n", rc);
		return rc;
	}

	cpr_parse_cond_min_volt_fuse(cpr_vreg, of_node);
	cpr_parse_speed_bin_fuse(cpr_vreg, of_node);
	rc = cpr_voltage_uplift_enable_check(cpr_vreg, of_node);
	if (rc < 0) {
		pr_err("voltage uplift enable check failed, %d\n", rc);
		return rc;
	}
	if (cpr_vreg->flags & FLAGS_SET_MIN_VOLTAGE) {
		of_property_read_u32(of_node, "qcom,cpr-cond-min-voltage",
					&min_uv);
		for (i = CPR_FUSE_CORNER_SVS; i < CPR_FUSE_CORNER_MAX; i++)
			if (cpr_vreg->ceiling_volt[i] < min_uv) {
				cpr_vreg->ceiling_volt[i] = min_uv;
				cpr_vreg->floor_volt[i] = min_uv;
			} else if (cpr_vreg->floor_volt[i] < min_uv) {
				cpr_vreg->floor_volt[i] = min_uv;
			}
	}

	return 0;
}

#if CONFIG_ARCH_DUMMY
static int __devinit htc_init_corner(struct platform_device *pdev,
					struct cpr_regulator *cpr_vreg)
{
	struct device_node *of_node = pdev->dev.of_node;
	int rc = 0;
	int i;

	htc_pvs_adjust = of_property_read_bool(of_node, "htc,pvs-corner-adjust");
	if (htc_pvs_adjust) {
		rc = of_property_read_u32(of_node, "htc,pvs-corner-adjust-timetick", &htc_pvs_adjust_seconds);
		if (rc < 0) {
			pr_err("[HTC] pvs adjustment timetick missing, %d\n", rc);
			return rc;
		}
	} else {
		pr_info("[HTC] No HTC pvs corner adjustment.\n");
		return 0;
	}

	if (cpr_vreg->speed_bin == 0) {
		
		rc = of_property_read_u32_array(of_node,
				"htc,pvs-corner-ceiling-8926",
				&cpr_vreg->htc_pvs_corner_v[CPR_FUSE_CORNER_SVS],
				CPR_FUSE_CORNER_MAX - CPR_FUSE_CORNER_SVS);
		if (rc < 0) {
			pr_err("htc,pvs-corner-ceiling-8926 missing: rc=%d\n", rc);
			return rc;
		}
	} else {
		
		rc = of_property_read_u32_array(of_node,
				"htc,pvs-corner-ceiling-8928",
				&cpr_vreg->htc_pvs_corner_v[CPR_FUSE_CORNER_SVS],
				CPR_FUSE_CORNER_MAX - CPR_FUSE_CORNER_SVS);
		if (rc < 0) {
			pr_err("htc,pvs-corner-ceiling-8928 missing: rc=%d\n", rc);
			return rc;
		}
	}
	
	for (i = CPR_FUSE_CORNER_SVS; i < CPR_FUSE_CORNER_MAX; i++) {
		if(cpr_vreg->htc_pvs_corner_v[i] > cpr_vreg->ceiling_volt[i])
			cpr_vreg->ceiling_volt[i] = cpr_vreg->htc_pvs_corner_v[i];
	}

	if(cpr_vreg->htc_pvs_corner_v[CPR_FUSE_CORNER_TURBO] > cpr_vreg->ceiling_max)
		cpr_vreg->ceiling_max = cpr_vreg->htc_pvs_corner_v[CPR_FUSE_CORNER_TURBO];

	pr_info("[HTC] Adjust pvs-corner-ceiling to [%d %d %d] uV in %d seconds.\n",
			cpr_vreg->htc_pvs_corner_v[CPR_FUSE_CORNER_SVS],
			cpr_vreg->htc_pvs_corner_v[CPR_FUSE_CORNER_NORMAL],
			cpr_vreg->htc_pvs_corner_v[CPR_FUSE_CORNER_TURBO],
			htc_pvs_adjust_seconds);
	return 0;
}
#endif

static int cpr_mem_acc_init(struct platform_device *pdev,
				struct cpr_regulator *cpr_vreg)
{
	int rc;

	if (of_property_read_bool(pdev->dev.of_node, "mem-acc-supply")) {
		cpr_vreg->mem_acc_vreg = devm_regulator_get(&pdev->dev,
							"mem-acc");
		if (IS_ERR_OR_NULL(cpr_vreg->mem_acc_vreg)) {
			rc = PTR_RET(cpr_vreg->mem_acc_vreg);
			if (rc != -EPROBE_DEFER)
				pr_err("devm_regulator_get: mem-acc: rc=%d\n",
				       rc);
			return rc;
		}
	}
	return 0;
}

static int __devinit cpr_regulator_probe(struct platform_device *pdev)
{
	struct cpr_regulator *cpr_vreg;
	struct regulator_desc *rdesc;
	struct regulator_init_data *init_data = pdev->dev.platform_data;
	int rc;

	if (!pdev->dev.of_node) {
		pr_err("Device tree node is missing\n");
		return -EINVAL;
	}

	init_data = of_get_regulator_init_data(&pdev->dev, pdev->dev.of_node);
	if (!init_data) {
		pr_err("regulator init data is missing\n");
		return -EINVAL;
	} else {
		init_data->constraints.input_uV
			= init_data->constraints.max_uV;
		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS;
	}

	cpr_vreg = devm_kzalloc(&pdev->dev, sizeof(struct cpr_regulator),
				GFP_KERNEL);
	if (!cpr_vreg) {
		pr_err("Can't allocate cpr_regulator memory\n");
		return -ENOMEM;
	}

	rc = cpr_mem_acc_init(pdev, cpr_vreg);
	if (rc) {
		pr_err("mem_acc intialization error rc=%d\n", rc);
		return rc;
	}

	rc = cpr_efuse_init(pdev, cpr_vreg);
	if (rc) {
		pr_err("Wrong eFuse address specified: rc=%d\n", rc);
		return rc;
	}

	rc = cpr_voltage_plan_init(pdev, cpr_vreg);
	if (rc) {
		pr_err("Wrong DT parameter specified: rc=%d\n", rc);
		goto err_out;
	}

	rc = cpr_pvs_init(pdev, cpr_vreg);
	if (rc) {
		pr_err("Initialize PVS wrong: rc=%d\n", rc);
		goto err_out;
	}

	rc = cpr_apc_init(pdev, cpr_vreg);
	if (rc) {
		if (rc != -EPROBE_DEFER)
			pr_err("Initialize APC wrong: rc=%d\n", rc);
		goto err_out;
	}

	rc = cpr_init_cpr(pdev, cpr_vreg);
	if (rc) {
		pr_err("Initialize CPR failed: rc=%d\n", rc);
		goto err_out;
	}

#if CONFIG_ARCH_DUMMY
	rc = htc_init_corner(pdev, cpr_vreg);
	if (rc) {
		pr_err("Iniyialize HTC modifications failed: rc=%d\n", rc);
		goto err_out;
	}
#endif

	cpr_efuse_free(cpr_vreg);

	mutex_init(&cpr_vreg->cpr_mutex);

	rdesc			= &cpr_vreg->rdesc;
	rdesc->owner		= THIS_MODULE;
	rdesc->type		= REGULATOR_VOLTAGE;
	rdesc->ops		= &cpr_corner_ops;
	rdesc->name		= init_data->constraints.name;

	cpr_vreg->rdev = regulator_register(rdesc, &pdev->dev, init_data,
					    cpr_vreg, pdev->dev.of_node);
	if (IS_ERR(cpr_vreg->rdev)) {
		rc = PTR_ERR(cpr_vreg->rdev);
		pr_err("regulator_register failed: rc=%d\n", rc);

		cpr_apc_exit(cpr_vreg);
		return rc;
	}

	platform_set_drvdata(pdev, cpr_vreg);
	the_cpr = cpr_vreg;

	return 0;

err_out:
	cpr_efuse_free(cpr_vreg);
	return rc;
}

static int __devexit cpr_regulator_remove(struct platform_device *pdev)
{
	struct cpr_regulator *cpr_vreg;

	cpr_vreg = platform_get_drvdata(pdev);
	if (cpr_vreg) {
		
		if (cpr_is_allowed(cpr_vreg)) {
			cpr_ctl_disable(cpr_vreg);
			cpr_irq_set(cpr_vreg, 0);
		}

		cpr_apc_exit(cpr_vreg);
		regulator_unregister(cpr_vreg->rdev);
	}

	return 0;
}

static struct of_device_id cpr_regulator_match_table[] = {
	{ .compatible = CPR_REGULATOR_DRIVER_NAME, },
	{}
};

static struct platform_driver cpr_regulator_driver = {
	.driver		= {
		.name	= CPR_REGULATOR_DRIVER_NAME,
		.of_match_table = cpr_regulator_match_table,
		.owner = THIS_MODULE,
	},
	.probe		= cpr_regulator_probe,
	.remove		= __devexit_p(cpr_regulator_remove),
	.suspend	= cpr_regulator_suspend,
	.resume		= cpr_regulator_resume,
};

int __init cpr_regulator_init(void)
{
	static bool initialized;

	if (initialized)
		return 0;
	else
		initialized = true;

	return platform_driver_register(&cpr_regulator_driver);
}
EXPORT_SYMBOL(cpr_regulator_init);

static void __exit cpr_regulator_exit(void)
{
	platform_driver_unregister(&cpr_regulator_driver);
}

MODULE_DESCRIPTION("CPR regulator driver");
MODULE_LICENSE("GPL v2");

arch_initcall(cpr_regulator_init);
module_exit(cpr_regulator_exit);
