/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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

#include "mdss_mdp.h"
#include "mdss_panel.h"
#include "mdss_mdp_trace.h"

#include "mdss_dsi.h"

#define VSYNC_EXPIRE_TICK 4

#define START_THRESHOLD 4
#define CONTINUE_THRESHOLD 4

#define MAX_SESSIONS 2

#define KOFF_TIMEOUT msecs_to_jiffies(84)

#define STOP_TIMEOUT(hz) msecs_to_jiffies((1000 / hz) * (VSYNC_EXPIRE_TICK + 2))

static struct workqueue_struct *post_commit_cmd_wq = NULL;

struct mdss_mdp_cmd_ctx {
	struct mdss_mdp_ctl *ctl;
	u32 pp_num;
	u8 ref_cnt;
	struct completion pp_comp;
	struct completion stop_comp;
	struct list_head vsync_handlers;
	int panel_on;
	int koff_cnt;
	int clk_enabled;
	int vsync_enabled;
	int rdptr_enabled;
	struct mutex clk_mtx;
	spinlock_t clk_lock;
	struct work_struct clk_work;
	struct work_struct pp_done_work;
	struct work_struct post_commit_cmd_work;
	atomic_t pp_done_cnt;

	
	u8 tear_check;
	u16 height;	
	u16 vporch;	
	u16 start_threshold;
	u32 vclk_line;	
	struct mdss_panel_recovery recovery;

	int logging;
};

struct mdss_mdp_cmd_ctx mdss_mdp_cmd_ctx_list[MAX_SESSIONS];

static inline u32 mdss_mdp_cmd_line_count(struct mdss_mdp_ctl *ctl)
{
	struct mdss_mdp_mixer *mixer;
	u32 cnt = 0xffff;	
	u32 init;
	u32 height;

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON, false);

	mixer = mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_LEFT);
	if (!mixer) {
		mixer = mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_RIGHT);
		if (!mixer) {
			mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF, false);
			goto exit;
		}
	}

	init = mdss_mdp_pingpong_read
		(mixer, MDSS_MDP_REG_PP_VSYNC_INIT_VAL) & 0xffff;

	height = mdss_mdp_pingpong_read
		(mixer, MDSS_MDP_REG_PP_SYNC_CONFIG_HEIGHT) & 0xffff;

	if (height < init) {
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF, false);
		goto exit;
	}

	cnt = mdss_mdp_pingpong_read
		(mixer, MDSS_MDP_REG_PP_INT_COUNT_VAL) & 0xffff;

	if (cnt < init)		
		cnt += (height - init);
	else
		cnt -= init;

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF, false);

	pr_debug("cnt=%d init=%d height=%d\n", cnt, init, height);
exit:
	return cnt;
}

static int mdss_mdp_cmd_tearcheck_cfg(struct mdss_mdp_mixer *mixer,
			struct mdss_mdp_cmd_ctx *ctx, int enable)
{
	u32 cfg;

	cfg = BIT(19); 
	if (ctx->tear_check)
		cfg |= BIT(20);	
	cfg |= ctx->vclk_line;

	mdss_mdp_pingpong_write(mixer, MDSS_MDP_REG_PP_SYNC_CONFIG_VSYNC, cfg);
	mdss_mdp_pingpong_write(mixer, MDSS_MDP_REG_PP_SYNC_CONFIG_HEIGHT,
				0xfff0); 

	mdss_mdp_pingpong_write(mixer, MDSS_MDP_REG_PP_VSYNC_INIT_VAL,
						ctx->height);

	mdss_mdp_pingpong_write(mixer, MDSS_MDP_REG_PP_RD_PTR_IRQ,
						ctx->height + 1);

	mdss_mdp_pingpong_write(mixer, MDSS_MDP_REG_PP_START_POS,
						ctx->height);

	mdss_mdp_pingpong_write(mixer, MDSS_MDP_REG_PP_SYNC_THRESH,
		   (CONTINUE_THRESHOLD << 16) | (ctx->start_threshold));

	mdss_mdp_pingpong_write(mixer, MDSS_MDP_REG_PP_TEAR_CHECK_EN, enable);
	return 0;
}

static int mdss_mdp_cmd_tearcheck_setup(struct mdss_mdp_ctl *ctl, int enable)
{
	struct mdss_mdp_cmd_ctx *ctx = ctl->priv_data;
	struct mdss_panel_info *pinfo;
	struct mdss_mdp_mixer *mixer;

	pinfo = &ctl->panel_data->panel_info;

	if (pinfo->mipi.vsync_enable && enable) {
		u32 mdp_vsync_clk_speed_hz, total_lines;

		mdss_mdp_vsync_clk_enable(1);

		mdp_vsync_clk_speed_hz =
		mdss_mdp_get_clk_rate(MDSS_CLK_MDP_VSYNC);
		pr_debug("%s: vsync_clk_rate=%d\n", __func__,
					mdp_vsync_clk_speed_hz);

		if (mdp_vsync_clk_speed_hz == 0) {
			pr_err("can't get clk speed\n");
			return -EINVAL;
		}

		ctx->tear_check = pinfo->mipi.hw_vsync_mode;
		ctx->height = pinfo->yres;
		ctx->vporch = pinfo->lcdc.v_back_porch +
				    pinfo->lcdc.v_front_porch +
				    pinfo->lcdc.v_pulse_width;

		ctx->start_threshold = START_THRESHOLD;

		total_lines = ctx->height + ctx->vporch;
		total_lines *= pinfo->mipi.frame_rate;
		ctx->vclk_line = mdp_vsync_clk_speed_hz / total_lines;

		pr_debug("%s: fr=%d tline=%d vcnt=%d thold=%d vrate=%d\n",
			__func__, pinfo->mipi.frame_rate, total_lines,
				ctx->vclk_line, ctx->start_threshold,
				mdp_vsync_clk_speed_hz);
	} else {
		enable = 0;
	}

	mixer = mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_LEFT);
	if (mixer)
		mdss_mdp_cmd_tearcheck_cfg(mixer, ctx, enable);

	mixer = mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_RIGHT);
	if (mixer)
		mdss_mdp_cmd_tearcheck_cfg(mixer, ctx, enable);

	return 0;
}

static inline void mdss_mdp_cmd_clk_on(struct mdss_mdp_cmd_ctx *ctx)
{
	unsigned long flags;
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();
	int rc;
	mutex_lock(&ctx->clk_mtx);
	if (!ctx->clk_enabled) {
		ctx->clk_enabled = 1;

		rc = mdss_iommu_ctrl(1);
		if (IS_ERR_VALUE(rc))
			pr_err("IOMMU attach failed\n");
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON, false);
		mdss_mdp_ctl_intf_event
			(ctx->ctl, MDSS_EVENT_PANEL_CLK_CTRL, (void *)1);
		mdss_mdp_hist_intr_setup(&mdata->hist_intr, MDSS_IRQ_RESUME);
	}
	spin_lock_irqsave(&ctx->clk_lock, flags);
	if (!ctx->rdptr_enabled)
		mdss_mdp_irq_enable(MDSS_MDP_IRQ_PING_PONG_RD_PTR, ctx->pp_num);
	ctx->rdptr_enabled = VSYNC_EXPIRE_TICK;
	spin_unlock_irqrestore(&ctx->clk_lock, flags);
	mutex_unlock(&ctx->clk_mtx);
}

static inline void mdss_mdp_cmd_clk_off(struct mdss_mdp_cmd_ctx *ctx)
{
	unsigned long flags;
	struct mdss_data_type *mdata = mdss_mdp_get_mdata();
	int set_clk_off = 0;

	mutex_lock(&ctx->clk_mtx);
	spin_lock_irqsave(&ctx->clk_lock, flags);
	if (!ctx->rdptr_enabled)
		set_clk_off = 1;
	spin_unlock_irqrestore(&ctx->clk_lock, flags);

	if (ctx->clk_enabled && set_clk_off) {
		ctx->clk_enabled = 0;
		mdss_mdp_hist_intr_setup(&mdata->hist_intr, MDSS_IRQ_SUSPEND);
		mdss_mdp_ctl_intf_event
			(ctx->ctl, MDSS_EVENT_PANEL_CLK_CTRL, (void *)0);
		mdss_iommu_ctrl(0);
		mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF, false);
	}
	mutex_unlock(&ctx->clk_mtx);
}

static void mdss_mdp_cmd_readptr_done(void *arg)
{
	struct mdss_mdp_ctl *ctl = arg;
	struct mdss_mdp_cmd_ctx *ctx = ctl->priv_data;
	struct mdss_mdp_vsync_handler *tmp;
	ktime_t vsync_time;

	if (!ctx) {
		pr_err("invalid ctx\n");
		return;
	}

	vsync_time = ktime_get();
	ctl->vsync_cnt++;

	if (ctx->logging > 0) {
		pr_info("%s: cnt=%d, clk=%d, rdptr=%d\n",
			__func__, ctl->vsync_cnt, ctx->clk_enabled, ctx->rdptr_enabled);
		ctx->logging--;
	}

	spin_lock(&ctx->clk_lock);
	list_for_each_entry(tmp, &ctx->vsync_handlers, list) {
		if (tmp->enabled && !tmp->cmd_post_flush)
			tmp->vsync_handler(ctl, vsync_time);
	}

	if (!ctx->vsync_enabled) {
		if (ctx->rdptr_enabled)
			ctx->rdptr_enabled--;

		
		if (ctx->rdptr_enabled == 0 && ctx->koff_cnt)
			ctx->rdptr_enabled++;
	}

	if (ctx->rdptr_enabled == 0) {
		mdss_mdp_irq_disable_nosync
			(MDSS_MDP_IRQ_PING_PONG_RD_PTR, ctx->pp_num);
		complete(&ctx->stop_comp);
		schedule_work(&ctx->clk_work);
	}

	spin_unlock(&ctx->clk_lock);
}

static void mdss_mdp_cmd_underflow_recovery(void *data)
{
	struct mdss_mdp_cmd_ctx *ctx = data;
	unsigned long flags;

	if (!data) {
		pr_err("%s: invalid ctx\n", __func__);
		return;
	}

	if (!ctx->ctl)
		return;
	spin_lock_irqsave(&ctx->clk_lock, flags);
	if (ctx->koff_cnt) {
		mdss_mdp_ctl_reset(ctx->ctl);
		pr_debug("%s: intf_num=%d\n", __func__,
					ctx->ctl->intf_num);
		ctx->koff_cnt--;
		mdss_mdp_irq_disable_nosync(MDSS_MDP_IRQ_PING_PONG_COMP,
						ctx->pp_num);
		complete_all(&ctx->pp_comp);
	}
	spin_unlock_irqrestore(&ctx->clk_lock, flags);
}

void mdss_mdp_post_commit_dsi_cmd(struct mdss_mdp_ctl *ctl)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	ctrl_pdata = container_of(ctl->panel_data, struct mdss_dsi_ctrl_pdata,
								panel_data);

	if (ctrl_pdata->frame_suffix_cmds.cmd_cnt == 0)
		return;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = ctrl_pdata->frame_suffix_cmds.cmds;
	cmdreq.cmds_cnt = ctrl_pdata->frame_suffix_cmds.cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_REQ_HS_MODE;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	return;
}

static void mdss_mdp_cmd_pingpong_done(void *arg)
{
	struct mdss_mdp_ctl *ctl = arg;
	struct mdss_mdp_cmd_ctx *ctx = ctl->priv_data;
	struct mdss_mdp_vsync_handler *tmp;
	ktime_t vsync_time;

	if (!ctx) {
		pr_err("%s: invalid ctx\n", __func__);
		return;
	}

	spin_lock(&ctx->clk_lock);
	list_for_each_entry(tmp, &ctx->vsync_handlers, list) {
		if (tmp->enabled && tmp->cmd_post_flush)
			tmp->vsync_handler(ctl, vsync_time);
	}
	mdss_mdp_irq_disable_nosync(MDSS_MDP_IRQ_PING_PONG_COMP, ctx->pp_num);

	complete_all(&ctx->pp_comp);

	if (ctx->koff_cnt) {
		atomic_inc(&ctx->pp_done_cnt);
		schedule_work(&ctx->pp_done_work);
		ctx->koff_cnt--;
		if (ctx->koff_cnt) {
			pr_err("%s: too many kickoffs=%d!\n", __func__,
			       ctx->koff_cnt);
			ctx->koff_cnt = 0;
		}
	} else
		pr_err("%s: should not have pingpong interrupt!\n", __func__);

	trace_mdp_cmd_pingpong_done(ctl, ctx->pp_num, ctx->koff_cnt);
	pr_debug("%s: ctl_num=%d intf_num=%d ctx=%d kcnt=%d\n", __func__,
		ctl->num, ctl->intf_num, ctx->pp_num, ctx->koff_cnt);

	spin_unlock(&ctx->clk_lock);
}

static void pingpong_done_work(struct work_struct *work)
{
	struct mdss_mdp_cmd_ctx *ctx =
		container_of(work, typeof(*ctx), pp_done_work);

	if (ctx->ctl)
		while (atomic_add_unless(&ctx->pp_done_cnt, -1, 0))
			mdss_mdp_ctl_notify(ctx->ctl, MDP_NOTIFY_FRAME_DONE);
}

static void do_post_commit_cmd_work(struct work_struct *work)
{
	struct mdss_mdp_cmd_ctx *ctx =
		container_of(work, typeof(*ctx), post_commit_cmd_work);

	if (ctx->ctl)
		mdss_mdp_post_commit_dsi_cmd(ctx->ctl);
}

static void clk_ctrl_work(struct work_struct *work)
{
	struct mdss_mdp_cmd_ctx *ctx =
		container_of(work, typeof(*ctx), clk_work);

	if (!ctx) {
		pr_err("%s: invalid ctx\n", __func__);
		return;
	}

	mdss_mdp_cmd_clk_off(ctx);
}

static int mdss_mdp_cmd_add_vsync_handler(struct mdss_mdp_ctl *ctl,
		struct mdss_mdp_vsync_handler *handle)
{
	struct mdss_mdp_cmd_ctx *ctx;
	unsigned long flags;
	bool enable_rdptr = false;

	ctx = (struct mdss_mdp_cmd_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("%s: invalid ctx\n", __func__);
		return -ENODEV;
	}

	spin_lock_irqsave(&ctx->clk_lock, flags);
	if (!handle->enabled) {
		handle->enabled = true;
		list_add(&handle->list, &ctx->vsync_handlers);

		enable_rdptr = !handle->cmd_post_flush;
		if (enable_rdptr)
			ctx->vsync_enabled++;
	}
	spin_unlock_irqrestore(&ctx->clk_lock, flags);

	if (enable_rdptr)
		mdss_mdp_cmd_clk_on(ctx);

	return 0;
}

static int mdss_mdp_cmd_remove_vsync_handler(struct mdss_mdp_ctl *ctl,
		struct mdss_mdp_vsync_handler *handle)
{
	struct mdss_mdp_cmd_ctx *ctx;
	unsigned long flags;

	ctx = (struct mdss_mdp_cmd_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("%s: invalid ctx\n", __func__);
		return -ENODEV;
	}

	spin_lock_irqsave(&ctx->clk_lock, flags);
	if (handle->enabled) {
		handle->enabled = false;
		list_del_init(&handle->list);

		if (!handle->cmd_post_flush) {
			if (ctx->vsync_enabled)
				ctx->vsync_enabled--;
			else
				WARN(1, "unbalanced vsync disable");
		}
	}
	spin_unlock_irqrestore(&ctx->clk_lock, flags);
	return 0;
}

int mdss_mdp_cmd_reconfigure_splash_done(struct mdss_mdp_ctl *ctl, bool handoff)
{
	struct mdss_panel_data *pdata;
	int ret = 0;

	pdata = ctl->panel_data;

	pdata->panel_info.cont_splash_enabled = 0;

	mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_PANEL_CLK_CTRL, (void *)0);

	return ret;
}
#define RBUF_SIZE 4
static char power_mode_rbuf[RBUF_SIZE];
static void panel_power_mode(int len)
{
	unsigned *lp;

	lp = (uint32_t *)power_mode_rbuf;
	pr_info("%s: data=0x%x len=%d\n", __func__,*lp, len);
}
static char signal_mode_rbuf[RBUF_SIZE];
static void panel_signal_mode(int len)
{
	unsigned *lp;

	lp = (uint32_t *)signal_mode_rbuf;
	pr_info("%s: data=0x%x len=%d\n", __func__,*lp, len);
}
int mdss_mdp_read_panel_status(struct mdss_mdp_ctl *ctl)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	ctrl_pdata = container_of(ctl->panel_data, struct mdss_dsi_ctrl_pdata,
								panel_data);

	memset(power_mode_rbuf, 0, RBUF_SIZE);
	memset(signal_mode_rbuf, 0, RBUF_SIZE);

	mdss_dsi_panel_cmd_read(ctrl_pdata, 0x0A, 0x00, panel_power_mode, power_mode_rbuf, RBUF_SIZE); 
	mdss_dsi_panel_cmd_read(ctrl_pdata, 0x0E, 0x00, panel_signal_mode, signal_mode_rbuf, RBUF_SIZE); 

	return 0;
}

static int mdss_mdp_cmd_wait4pingpong(struct mdss_mdp_ctl *ctl, void *arg)
{
	struct mdss_mdp_cmd_ctx *ctx;
	unsigned long flags;
	int need_wait = 0;
	int rc = 0;

	ctx = (struct mdss_mdp_cmd_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

	spin_lock_irqsave(&ctx->clk_lock, flags);
	if (ctx->koff_cnt > 0)
		need_wait = 1;
	spin_unlock_irqrestore(&ctx->clk_lock, flags);

	pr_debug("%s: need_wait=%d  intf_num=%d ctx=%p\n",
			__func__, need_wait, ctl->intf_num, ctx);

	if (need_wait) {
		rc = wait_for_completion_timeout(
				&ctx->pp_comp, KOFF_TIMEOUT);

		trace_mdp_cmd_wait_pingpong(ctl->num, ctx->koff_cnt);

		if (rc <= 0) {
			ctx->logging = 60;
			mdss_mdp_read_panel_status(ctl); 
			WARN(1, "cmd kickoff timed out (%d) ctl=%d\n",
						rc, ctl->num);
			rc = -EPERM;
			mdss_mdp_ctl_notify(ctl, MDP_NOTIFY_FRAME_TIMEOUT);
		} else {
			rc = 0;
		}
	}

	return rc;
}

static int mdss_mdp_cmd_set_partial_roi(struct mdss_mdp_ctl *ctl)
{
	int rc = 0;
	if (ctl->roi.w && ctl->roi.h && ctl->roi_changed &&
			ctl->panel_data->panel_info.partial_update_enabled) {
		ctl->panel_data->panel_info.roi_x = ctl->roi.x;
		ctl->panel_data->panel_info.roi_y = ctl->roi.y;
		ctl->panel_data->panel_info.roi_w = ctl->roi.w;
		ctl->panel_data->panel_info.roi_h = ctl->roi.h;

		rc = mdss_mdp_ctl_intf_event(ctl,
				MDSS_EVENT_ENABLE_PARTIAL_UPDATE, NULL);
	}
	return rc;
}

int mdss_mdp_cmd_kickoff(struct mdss_mdp_ctl *ctl, void *arg)
{
	struct mdss_mdp_cmd_ctx *ctx;
	unsigned long flags;
	int rc;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	ctx = (struct mdss_mdp_cmd_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

	if (ctx->panel_on == 0) {
		rc = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_UNBLANK, NULL);
		WARN(rc, "intf %d unblank error (%d)\n", ctl->intf_num, rc);

		ctx->panel_on++;

		rc = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_PANEL_ON, NULL);
		WARN(rc, "intf %d panel on error (%d)\n", ctl->intf_num, rc);

		mdss_mdp_ctl_intf_event(ctl,
				MDSS_EVENT_REGISTER_RECOVERY_HANDLER,
				(void *)&ctx->recovery);
	}

	spin_lock_irqsave(&ctx->clk_lock, flags);
	ctx->koff_cnt++;
	spin_unlock_irqrestore(&ctx->clk_lock, flags);

	trace_mdp_cmd_kickoff(ctl->num, ctx->koff_cnt);
	mdss_mdp_cmd_clk_on(ctx);

	mdss_mdp_cmd_set_partial_roi(ctl);

	mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_DSI_CMDLIST_KOFF, NULL);
	INIT_COMPLETION(ctx->pp_comp);
	mdss_mdp_irq_enable(MDSS_MDP_IRQ_PING_PONG_COMP, ctx->pp_num);
	mdss_mdp_ctl_write(ctl, MDSS_MDP_REG_CTL_START, 1);
	mb();

	ctrl_pdata = container_of(ctl->panel_data, struct mdss_dsi_ctrl_pdata,
								panel_data);

	if (ctrl_pdata->frame_suffix_cmds.cmds != NULL)
		queue_work(post_commit_cmd_wq, &ctx->post_commit_cmd_work);

	return 0;
}

int mdss_mdp_cmd_stop(struct mdss_mdp_ctl *ctl)
{
	struct mdss_mdp_cmd_ctx *ctx;
	unsigned long flags;
	struct mdss_mdp_vsync_handler *tmp, *handle;
	int need_wait = 0;
	int ret = 0;
	int hz;

	ctx = (struct mdss_mdp_cmd_ctx *) ctl->priv_data;
	if (!ctx) {
		pr_err("invalid ctx\n");
		return -ENODEV;
	}

	cancel_work_sync(&ctx->post_commit_cmd_work);

	list_for_each_entry_safe(handle, tmp, &ctx->vsync_handlers, list)
		mdss_mdp_cmd_remove_vsync_handler(ctl, handle);

	spin_lock_irqsave(&ctx->clk_lock, flags);
	if (ctx->rdptr_enabled) {
		INIT_COMPLETION(ctx->stop_comp);
		need_wait = 1;
		ctx->logging = 8;
	}
	spin_unlock_irqrestore(&ctx->clk_lock, flags);

	hz = mdss_panel_get_framerate(&ctl->panel_data->panel_info);

	if (need_wait)
		if (wait_for_completion_timeout(&ctx->stop_comp,
					STOP_TIMEOUT(hz))
		    <= 0) {
				WARN(1, "stop cmd time out, vsync = %d, rdptr = %d\n",
					ctx->vsync_enabled, ctx->rdptr_enabled);

				mdss_mdp_irq_disable(MDSS_MDP_IRQ_PING_PONG_RD_PTR, ctx->pp_num);
				ctx->rdptr_enabled = 0;
			}

	if (cancel_work_sync(&ctx->clk_work))
		pr_debug("no pending clk work\n");

	mdss_mdp_ctl_intf_event(ctl,
			MDSS_EVENT_REGISTER_RECOVERY_HANDLER,
			NULL);

	mdss_mdp_cmd_clk_off(ctx);

	flush_work(&ctx->pp_done_work);

	ctx->panel_on = 0;

	mdss_mdp_set_intr_callback(MDSS_MDP_IRQ_PING_PONG_RD_PTR, ctx->pp_num,
				   NULL, NULL);
	mdss_mdp_set_intr_callback(MDSS_MDP_IRQ_PING_PONG_COMP, ctx->pp_num,
				   NULL, NULL);

	memset(ctx, 0, sizeof(*ctx));
	ctl->priv_data = NULL;

	ret = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_BLANK, NULL);
	WARN(ret, "intf %d unblank error (%d)\n", ctl->intf_num, ret);

	ret = mdss_mdp_ctl_intf_event(ctl, MDSS_EVENT_PANEL_OFF, NULL);
	WARN(ret, "intf %d unblank error (%d)\n", ctl->intf_num, ret);

	ctl->stop_fnc = NULL;
	ctl->display_fnc = NULL;
	ctl->wait_pingpong = NULL;
	ctl->add_vsync_handler = NULL;
	ctl->remove_vsync_handler = NULL;

	pr_debug("%s:-\n", __func__);

	return 0;
}

int mdss_mdp_cmd_start(struct mdss_mdp_ctl *ctl)
{
	struct mdss_mdp_cmd_ctx *ctx;
	struct mdss_mdp_mixer *mixer;
	int i, ret;

	pr_debug("%s:+\n", __func__);

	mixer = mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_LEFT);
	if (!mixer) {
		pr_err("mixer not setup correctly\n");
		return -ENODEV;
	}

	for (i = 0; i < MAX_SESSIONS; i++) {
		ctx = &mdss_mdp_cmd_ctx_list[i];
		if (!ctx) {
			pr_err("invalid ctx\n");
			return -ENODEV;
		}
		if (ctx->ref_cnt == 0) {
			ctx->ref_cnt++;
			break;
		}
	}
	if (i == MAX_SESSIONS) {
		pr_err("too many sessions\n");
		return -ENOMEM;
	}

	ctl->priv_data = ctx;

	ctx->ctl = ctl;
	ctx->pp_num = mixer->num;
	init_completion(&ctx->pp_comp);
	init_completion(&ctx->stop_comp);
	spin_lock_init(&ctx->clk_lock);
	mutex_init(&ctx->clk_mtx);
	INIT_WORK(&ctx->clk_work, clk_ctrl_work);
	INIT_WORK(&ctx->pp_done_work, pingpong_done_work);

	if (post_commit_cmd_wq == NULL) {
		post_commit_cmd_wq = alloc_workqueue("post_commit_cmd_wq", WQ_UNBOUND | WQ_HIGHPRI, 1);
	}

	INIT_WORK(&ctx->post_commit_cmd_work, do_post_commit_cmd_work);
	atomic_set(&ctx->pp_done_cnt, 0);
	INIT_LIST_HEAD(&ctx->vsync_handlers);

	ctx->recovery.fxn = mdss_mdp_cmd_underflow_recovery;
	ctx->recovery.data = ctx;
	ctx->logging = 2;

	pr_debug("%s: ctx=%p num=%d mixer=%d\n", __func__,
				ctx, ctx->pp_num, mixer->num);

	mdss_mdp_set_intr_callback(MDSS_MDP_IRQ_PING_PONG_RD_PTR, ctx->pp_num,
				   mdss_mdp_cmd_readptr_done, ctl);

	mdss_mdp_set_intr_callback(MDSS_MDP_IRQ_PING_PONG_COMP, ctx->pp_num,
				   mdss_mdp_cmd_pingpong_done, ctl);

	ret = mdss_mdp_cmd_tearcheck_setup(ctl, 1);
	if (ret) {
		pr_err("tearcheck setup failed\n");
		return ret;
	}

	ctl->stop_fnc = mdss_mdp_cmd_stop;
	ctl->display_fnc = mdss_mdp_cmd_kickoff;
	ctl->wait_pingpong = mdss_mdp_cmd_wait4pingpong;
	ctl->add_vsync_handler = mdss_mdp_cmd_add_vsync_handler;
	ctl->remove_vsync_handler = mdss_mdp_cmd_remove_vsync_handler;
	ctl->read_line_cnt_fnc = mdss_mdp_cmd_line_count;
	pr_debug("%s:-\n", __func__);

	return 0;
}

