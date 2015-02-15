/* Copyright (c) 2008-2012, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>

#include <mach/hardware.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/mach-types.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>

#include <linux/fb.h>

#include "mdp.h"
#include "msm_fb.h"
#include "mddihost.h"

static uint32 mdp_last_dma2_update_width;
static uint32 mdp_last_dma2_update_height;
static uint32 mdp_curr_dma2_update_width;
static uint32 mdp_curr_dma2_update_height;

ktime_t mdp_dma2_last_update_time = { 0 };

int mdp_lcd_rd_cnt_offset_slow = 20;
int mdp_lcd_rd_cnt_offset_fast = 20;
int mdp_vsync_usec_wait_line_too_short = 5;
uint32 mdp_dma2_update_time_in_usec;
uint32 mdp_total_vdopkts;

extern u32 msm_fb_debug_enabled;
extern struct workqueue_struct *mdp_dma_wq;

int vsync_start_y_adjust = 4;

static void mdp_dma2_update_lcd(struct msm_fb_data_type *mfd)
{
	MDPIBUF *iBuf = &mfd->ibuf;
	int mddi_dest = FALSE;
	int cmd_mode = FALSE;
	uint32 outBpp = iBuf->bpp;
	uint32 dma2_cfg_reg;
	uint8 *src;
	uint32 mddi_ld_param;
	uint16 mddi_vdo_packet_reg;
#ifndef CONFIG_FB_MSM_MDP303
	struct msm_fb_panel_data *pdata =
	    (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;
#endif
	uint32 ystride = mfd->fbi->fix.line_length;
	uint32 mddi_pkt_desc;

	dma2_cfg_reg = DMA_PACK_ALIGN_LSB |
		    DMA_OUT_SEL_AHB | DMA_IBUF_NONCONTIGUOUS;

#ifdef CONFIG_FB_MSM_MDP22
	dma2_cfg_reg |= DMA_PACK_TIGHT;
#endif

#ifdef CONFIG_FB_MSM_MDP30
	/*
	 * Software workaround:  On 7x25/7x27, the MDP will not
	 * respond if dma_w is 1 pixel.  Set the update width to
	 * 2 pixels and adjust the x offset if needed.
	 */
	if (iBuf->dma_w == 1) {
		iBuf->dma_w = 2;
		if (iBuf->dma_x == (iBuf->ibuf_width - 2))
			iBuf->dma_x--;
	}
#endif

	if (mfd->fb_imgType == MDP_BGR_565)
		dma2_cfg_reg |= DMA_PACK_PATTERN_BGR;
	else if (mfd->fb_imgType == MDP_RGBA_8888)
		dma2_cfg_reg |= DMA_PACK_PATTERN_BGR;
	else
		dma2_cfg_reg |= DMA_PACK_PATTERN_RGB;

	if (outBpp == 4) {
		dma2_cfg_reg |= DMA_IBUF_C3ALPHA_EN;
		dma2_cfg_reg |= DMA_IBUF_FORMAT_xRGB8888_OR_ARGB8888;
	}

	if (outBpp == 2)
		dma2_cfg_reg |= DMA_IBUF_FORMAT_RGB565;

	mddi_ld_param = 0;
	mddi_vdo_packet_reg = mfd->panel_info.mddi.vdopkt;

	if ((mfd->panel_info.type == MDDI_PANEL) ||
	    (mfd->panel_info.type == EXT_MDDI_PANEL)) {
		dma2_cfg_reg |= DMA_OUT_SEL_MDDI;
		mddi_dest = TRUE;

		if (mfd->panel_info.type == MDDI_PANEL) {
			mdp_total_vdopkts++;
			if (mfd->panel_info.pdest == DISPLAY_1) {
				dma2_cfg_reg |= DMA_MDDI_DMAOUT_LCD_SEL_PRIMARY;
				mddi_ld_param = 0;
#ifdef MDDI_HOST_WINDOW_WORKAROUND
				mddi_window_adjust(mfd, iBuf->dma_x,
						   iBuf->dma_w - 1, iBuf->dma_y,
						   iBuf->dma_h - 1);
#endif
			} else {
				dma2_cfg_reg |=
				    DMA_MDDI_DMAOUT_LCD_SEL_SECONDARY;
				mddi_ld_param = 1;
#ifdef MDDI_HOST_WINDOW_WORKAROUND
				mddi_window_adjust(mfd, iBuf->dma_x,
						   iBuf->dma_w - 1, iBuf->dma_y,
						   iBuf->dma_h - 1);
#endif
			}
		} else {
			dma2_cfg_reg |= DMA_MDDI_DMAOUT_LCD_SEL_EXTERNAL;
			mddi_ld_param = 2;
		}
#ifdef CONFIG_FB_MSM_MDP303
	} else if (mfd->panel_info.type == MIPI_CMD_PANEL) {
		cmd_mode = TRUE;
		dma2_cfg_reg |= DMA_OUT_SEL_DSI_CMD;
#endif
	} else {
		if (mfd->panel_info.pdest == DISPLAY_1) {
			dma2_cfg_reg |= DMA_AHBM_LCD_SEL_PRIMARY;
			outp32(MDP_EBI2_LCD0, mfd->data_port_phys);
		} else {
			dma2_cfg_reg |= DMA_AHBM_LCD_SEL_SECONDARY;
			outp32(MDP_EBI2_LCD1, mfd->data_port_phys);
		}
	}

	src = (uint8 *) iBuf->buf;
	/* starting input address */
	src += iBuf->dma_x * outBpp + iBuf->dma_y * ystride;

	mdp_curr_dma2_update_width = iBuf->dma_w;
	mdp_curr_dma2_update_height = iBuf->dma_h;

	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

#ifdef CONFIG_FB_MSM_MDP22
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x0184,
			(iBuf->dma_h << 16 | iBuf->dma_w));
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x0188, src);
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x018C, ystride);
#else
	if (cmd_mode)
		MDP_OUTP(MDP_BASE + 0x90004,
			(mfd->panel_info.yres << 16 | mfd->panel_info.xres));
	else
		MDP_OUTP(MDP_BASE + 0x90004, (iBuf->dma_h << 16 | iBuf->dma_w));

	MDP_OUTP(MDP_BASE + 0x90008, src);
	MDP_OUTP(MDP_BASE + 0x9000c, ystride);
#endif

	if (mfd->panel_info.bpp == 18) {
		mddi_pkt_desc = MDDI_VDO_PACKET_DESC;
		dma2_cfg_reg |= DMA_DSTC0G_6BITS |	/* 666 18BPP */
		    DMA_DSTC1B_6BITS | DMA_DSTC2R_6BITS;
	} else if (mfd->panel_info.bpp == 24) {
		mddi_pkt_desc = MDDI_VDO_PACKET_DESC_24;
		dma2_cfg_reg |= DMA_DSTC0G_8BITS |      /* 888 24BPP */
			DMA_DSTC1B_8BITS | DMA_DSTC2R_8BITS;
	} else {
		mddi_pkt_desc = MDDI_VDO_PACKET_DESC_16;
		dma2_cfg_reg |= DMA_DSTC0G_6BITS |	/* 565 16BPP */
		    DMA_DSTC1B_5BITS | DMA_DSTC2R_5BITS;
	}

#ifndef CONFIG_FB_MSM_MDP303

	if (mddi_dest) {
#ifdef CONFIG_FB_MSM_MDP22
		MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x0194,
			 (iBuf->dma_y << 16) | iBuf->dma_x);
		MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x01a0, mddi_ld_param);
		MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x01a4,
			 (mddi_pkt_desc << 16) | mddi_vdo_packet_reg);
#else
		MDP_OUTP(MDP_BASE + 0x90010, (iBuf->dma_y << 16) | iBuf->dma_x);
		MDP_OUTP(MDP_BASE + 0x00090, mddi_ld_param);
		MDP_OUTP(MDP_BASE + 0x00094,
			 (mddi_pkt_desc << 16) | mddi_vdo_packet_reg);
#endif
	} else {
		/* setting EBI2 LCDC write window */
		pdata->set_rect(iBuf->dma_x, iBuf->dma_y, iBuf->dma_w,
				iBuf->dma_h);
	}
#else
	if (mfd->panel_info.type == MIPI_CMD_PANEL) {
		/* dma_p = 0, dma_s = 1 */
		 MDP_OUTP(MDP_BASE + 0xF1000, 0x10);
		 /* enable dsi trigger on dma_p */
		 MDP_OUTP(MDP_BASE + 0xF1004, 0x01);
	}
#endif

	/* dma2 config register */
#ifdef MDP_HW_VSYNC
	MDP_OUTP(MDP_BASE + 0x90000, dma2_cfg_reg);

	if ((mfd->use_mdp_vsync) &&
	    (mfd->ibuf.vsync_enable) && (mfd->panel_info.lcd.vsync_enable)) {
		uint32 start_y;

		if (vsync_start_y_adjust <= iBuf->dma_y)
			start_y = iBuf->dma_y - vsync_start_y_adjust;
		else
			start_y =
			    (mfd->total_lcd_lines - 1) - (vsync_start_y_adjust -
							  iBuf->dma_y);

		/*
		 * MDP VSYNC clock must be On by now so, we don't have to
		 * re-enable it
		 */
		MDP_OUTP(MDP_BASE + 0x210, start_y);
		MDP_OUTP(MDP_BASE + 0x20c, 1);	/* enable prim vsync */
	} else {
		MDP_OUTP(MDP_BASE + 0x20c, 0);	/* disable prim vsync */
	}
#else
#ifdef CONFIG_FB_MSM_MDP22
	MDP_OUTP(MDP_CMD_DEBUG_ACCESS_BASE + 0x0180, dma2_cfg_reg);
#else
	MDP_OUTP(MDP_BASE + 0x90000, dma2_cfg_reg);
#endif
#endif /* MDP_HW_VSYNC */

	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
}

static ktime_t vt = { 0 };
int mdp_usec_diff_threshold = 100;
int mdp_expected_usec_wait;

enum hrtimer_restart mdp_dma2_vsync_hrtimer_handler(struct hrtimer *ht)
{
	struct msm_fb_data_type *mfd = NULL;

	mfd = container_of(ht, struct msm_fb_data_type, dma_hrtimer);

	mdp_pipe_kickoff(MDP_DMA2_TERM, mfd);

	if (msm_fb_debug_enabled) {
		ktime_t t;
		int usec_diff;
		int actual_wait;

		t = ktime_get_real();

		actual_wait = ktime_to_us(ktime_sub(t, vt));
		usec_diff = actual_wait - mdp_expected_usec_wait;

		if ((mdp_usec_diff_threshold < usec_diff) || (usec_diff < 0))
			MSM_FB_DEBUG
			    ("HRT Diff = %d usec Exp=%d usec  Act=%d usec\n",
			     usec_diff, mdp_expected_usec_wait, actual_wait);
	}

	return HRTIMER_NORESTART;
}


#ifdef CONFIG_FB_MSM_MDP303
static int busy_wait_cnt;

void	mdp3_dsi_cmd_dma_busy_wait(struct msm_fb_data_type *mfd)
{
	unsigned long flag;
	int need_wait = 0;

#ifdef DSI_CLK_CTRL
	mod_timer(&dsi_clock_timer, jiffies + HZ); /* one second */
#endif

	spin_lock_irqsave(&mdp_spin_lock, flag);
#ifdef DSI_CLK_CTRL

	spin_lock_bh(&dsi_clk_lock);
	if (mipi_dsi_clk_on == 0)
		mipi_dsi_turn_on_clks();
	spin_unlock_bh(&dsi_clk_lock);
#endif

	if (mfd->dma->busy == TRUE) {
		if (busy_wait_cnt == 0)
			INIT_COMPLETION(mfd->dma->comp);
		busy_wait_cnt++;
		need_wait++;
	}
	spin_unlock_irqrestore(&mdp_spin_lock, flag);

	if (need_wait) {
		/* wait until DMA finishes the current job */
		wait_for_completion(&mfd->dma->comp);
	}
}
#endif

static void mdp_dma_schedule(struct msm_fb_data_type *mfd, uint32 term)
{
	/*
	 * dma2 configure VSYNC block
	 * vsync supported on Primary LCD only for now
	 */
	int32 mdp_lcd_rd_cnt;
	uint32 usec_wait_time;
	uint32 start_y;

	/*
	 * ToDo: if we can move HRT timer callback to workqueue, we can
	 * move DMA2 power on under mdp_pipe_kickoff().
	 * This will save a power for hrt time wait.
	 * However if the latency for context switch (hrt irq -> workqueue)
	 * is too big, we will miss the vsync timing.
	 */
	if (term == MDP_DMA2_TERM)
		mdp_pipe_ctrl(MDP_DMA2_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

	mdp_dma2_update_time_in_usec = ktime_to_us(mdp_dma2_last_update_time);

	if ((!mfd->ibuf.vsync_enable) || (!mfd->panel_info.lcd.vsync_enable)
	    || (mfd->use_mdp_vsync)) {
		mdp_pipe_kickoff(term, mfd);
		return;
	}
	/* SW vsync logic starts here */

	/* get current rd counter */
	mdp_lcd_rd_cnt = mdp_get_lcd_line_counter(mfd);
	if (mdp_dma2_update_time_in_usec != 0) {
		uint32 num, den;

		/*
		 * roi width boundary calculation to know the size of pixel
		 * width that MDP can send faster or slower than LCD read
		 * pointer
		 */

		num = mdp_last_dma2_update_width * mdp_last_dma2_update_height;
		den =
		    (((mfd->panel_info.lcd.refx100 * mfd->total_lcd_lines) /
		      1000) * (mdp_dma2_update_time_in_usec / 100)) / 1000;

		if (den == 0)
			mfd->vsync_width_boundary[mdp_last_dma2_update_width] =
			    mfd->panel_info.xres + 1;
		else
			mfd->vsync_width_boundary[mdp_last_dma2_update_width] =
			    (int)(num / den);
	}

	if (mfd->vsync_width_boundary[mdp_last_dma2_update_width] >
	    mdp_curr_dma2_update_width) {
		/* MDP wrp is faster than LCD rdp */
		mdp_lcd_rd_cnt += mdp_lcd_rd_cnt_offset_fast;
	} else {
		/* MDP wrp is slower than LCD rdp */
		mdp_lcd_rd_cnt -= mdp_lcd_rd_cnt_offset_slow;
	}

	if (mdp_lcd_rd_cnt < 0)
		mdp_lcd_rd_cnt = mfd->total_lcd_lines + mdp_lcd_rd_cnt;
	else if (mdp_lcd_rd_cnt > mfd->total_lcd_lines)
		mdp_lcd_rd_cnt = mdp_lcd_rd_cnt - mfd->total_lcd_lines - 1;

	/* get wrt pointer position */
	start_y = mfd->ibuf.dma_y;

	/* measure line difference between start_y and rd counter */
	if (start_y > mdp_lcd_rd_cnt) {
		/*
		 * *100 for lcd_ref_hzx100 was already multiplied by 100
		 * *1000000 is for usec conversion
		 */

		if ((start_y - mdp_lcd_rd_cnt) <=
		    mdp_vsync_usec_wait_line_too_short)
			usec_wait_time = 0;
		else
			usec_wait_time =
			    ((start_y -
			      mdp_lcd_rd_cnt) * 1000000) /
			    ((mfd->total_lcd_lines *
			      mfd->panel_info.lcd.refx100) / 100);
	} else {
		if ((start_y + (mfd->total_lcd_lines - mdp_lcd_rd_cnt)) <=
		    mdp_vsync_usec_wait_line_too_short)
			usec_wait_time = 0;
		else
			usec_wait_time =
			    ((start_y +
			      (mfd->total_lcd_lines -
			       mdp_lcd_rd_cnt)) * 1000000) /
			    ((mfd->total_lcd_lines *
			      mfd->panel_info.lcd.refx100) / 100);
	}

	mdp_last_dma2_update_width = mdp_curr_dma2_update_width;
	mdp_last_dma2_update_height = mdp_curr_dma2_update_height;

	if (usec_wait_time == 0) {
		mdp_pipe_kickoff(term, mfd);
	} else {
		ktime_t wait_time;

		wait_time = ns_to_ktime(usec_wait_time * 1000);

		if (msm_fb_debug_enabled) {
			vt = ktime_get_real();
			mdp_expected_usec_wait = usec_wait_time;
		}
		hrtimer_start(&mfd->dma_hrtimer, wait_time, HRTIMER_MODE_REL);
	}
}

#ifdef MDDI_HOST_WINDOW_WORKAROUND
static void mdp_dma2_update_sub(struct msm_fb_data_type *mfd);
void mdp_dma2_update(struct msm_fb_data_type *mfd)
{
	MDPIBUF *iBuf;
	uint32 upper_height;

	if (mfd->panel.type == EXT_MDDI_PANEL) {
		mdp_dma2_update_sub(mfd);
		return;
	}

	iBuf = &mfd->ibuf;

	upper_height =
	    (uint32) mddi_assign_pkt_height((uint16) iBuf->dma_w,
					    (uint16) iBuf->dma_h, 18);

	if (upper_height >= iBuf->dma_h) {
		mdp_dma2_update_sub(mfd);
	} else {
		uint32 lower_height;

		/* sending the upper region first */
		lower_height = iBuf->dma_h - upper_height;
		iBuf->dma_h = upper_height;
		mdp_dma2_update_sub(mfd);

		/* sending the lower region second */
		iBuf->dma_h = lower_height;
		iBuf->dma_y += lower_height;
		iBuf->vsync_enable = FALSE;
		mdp_dma2_update_sub(mfd);
	}
}

static void mdp_dma2_update_sub(struct msm_fb_data_type *mfd)
#else
void mdp_dma2_update(struct msm_fb_data_type *mfd)
#endif
{
	unsigned long flag;
	static int first_vsync;
	int need_wait = 0;

	down(&mfd->dma->mutex);
	if ((mfd) && (mfd->panel_power_on)) {
		down(&mfd->sem);
		spin_lock_irqsave(&mdp_spin_lock, flag);
		if (mfd->dma->busy == TRUE)
			need_wait++;
		spin_unlock_irqrestore(&mdp_spin_lock, flag);

		if (need_wait)
			wait_for_completion_killable(&mfd->dma->comp);

		/* schedule DMA to start */
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
		mfd->ibuf_flushed = TRUE;
		mdp_dma2_update_lcd(mfd);

		spin_lock_irqsave(&mdp_spin_lock, flag);
		mdp_enable_irq(MDP_DMA2_TERM);
		mfd->dma->busy = TRUE;
		INIT_COMPLETION(mfd->dma->comp);
		INIT_COMPLETION(vsync_cntrl.vsync_comp);
		if (!vsync_cntrl.vsync_irq_enabled &&
				vsync_cntrl.disabled_clocks) {
			MDP_OUTP(MDP_BASE + 0x021c, 0x10); /* read pointer */
			outp32(MDP_INTR_CLEAR, MDP_PRIM_RDPTR);
			mdp_intr_mask |= MDP_PRIM_RDPTR;
			outp32(MDP_INTR_ENABLE, mdp_intr_mask);
			mdp_enable_irq(MDP_VSYNC_TERM);
			vsync_cntrl.vsync_dma_enabled = 1;
		}
		spin_unlock_irqrestore(&mdp_spin_lock, flag);
		/* schedule DMA to start */
		mdp_dma_schedule(mfd, MDP_DMA2_TERM);
		up(&mfd->sem);

		/* wait until Vsync finishes the current job */
		if (first_vsync) {
			if (!wait_for_completion_killable_timeout
					(&vsync_cntrl.vsync_comp, HZ/10))
				pr_err("Timedout DMA %s %d", __func__,
								__LINE__);
		} else {
			first_vsync = 1;
		}
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);

	/* signal if pan function is waiting for the update completion */
		if (mfd->pan_waiting) {
			mfd->pan_waiting = FALSE;
			complete(&mfd->pan_comp);
		}
	}
	up(&mfd->dma->mutex);
}

void mdp_dma_vsync_ctrl(int enable)
{
	unsigned long flag;
	int disabled_clocks;
	if (vsync_cntrl.vsync_irq_enabled == enable)
		return;

	spin_lock_irqsave(&mdp_spin_lock, flag);
	if (!enable)
		INIT_COMPLETION(vsync_cntrl.vsync_wait);

	vsync_cntrl.vsync_irq_enabled = enable;
	disabled_clocks = vsync_cntrl.disabled_clocks;
	spin_unlock_irqrestore(&mdp_spin_lock, flag);

	if (enable && disabled_clocks)
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

	spin_lock_irqsave(&mdp_spin_lock, flag);
	if (enable && vsync_cntrl.disabled_clocks &&
			!vsync_cntrl.vsync_dma_enabled) {
		MDP_OUTP(MDP_BASE + 0x021c, 0x10); /* read pointer */
		outp32(MDP_INTR_CLEAR, MDP_PRIM_RDPTR);
		mdp_intr_mask |= MDP_PRIM_RDPTR;
		outp32(MDP_INTR_ENABLE, mdp_intr_mask);
		mdp_enable_irq(MDP_VSYNC_TERM);
		vsync_cntrl.disabled_clocks = 0;
	} else if (enable && vsync_cntrl.disabled_clocks) {
		vsync_cntrl.disabled_clocks = 0;
	}
	spin_unlock_irqrestore(&mdp_spin_lock, flag);
	if (vsync_cntrl.vsync_irq_enabled &&
		atomic_read(&vsync_cntrl.suspend) == 0)
		atomic_set(&vsync_cntrl.vsync_resume, 1);
}

void mdp_lcd_update_workqueue_handler(struct work_struct *work)
{
	struct msm_fb_data_type *mfd = NULL;

	mfd = container_of(work, struct msm_fb_data_type, dma_update_worker);
	if (mfd)
		mfd->dma_fnc(mfd);
}

void mdp_set_dma_pan_info(struct fb_info *info, struct mdp_dirty_region *dirty,
			  boolean sync)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct fb_info *fbi = mfd->fbi;
	MDPIBUF *iBuf;
	int bpp = info->var.bits_per_pixel / 8;

	down(&mfd->sem);

	iBuf = &mfd->ibuf;

	if (mfd->display_iova)
		iBuf->buf = (uint8 *)mfd->display_iova;
	else
		iBuf->buf = (uint8 *) info->fix.smem_start;

	iBuf->buf += calc_fb_offset(mfd, fbi, bpp);

	iBuf->ibuf_width = info->var.xres_virtual;
	iBuf->bpp = bpp;

	iBuf->vsync_enable = sync;

	if (dirty) {
		/*
		 * ToDo: dirty region check inside var.xoffset+xres
		 * <-> var.yoffset+yres
		 */
		iBuf->dma_x = dirty->xoffset % info->var.xres;
		iBuf->dma_y = dirty->yoffset % info->var.yres;
		iBuf->dma_w = dirty->width;
		iBuf->dma_h = dirty->height;
	} else {
		iBuf->dma_x = 0;
		iBuf->dma_y = 0;
		iBuf->dma_w = info->var.xres;
		iBuf->dma_h = info->var.yres;
	}
	mfd->ibuf_flushed = FALSE;
	up(&mfd->sem);
}

void mdp_dma_pan_update(struct fb_info *info)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	MDPIBUF *iBuf;

	iBuf = &mfd->ibuf;

	if (mfd->sw_currently_refreshing) {
		/* we need to wait for the pending update */
		mfd->pan_waiting = TRUE;
		if (!mfd->ibuf_flushed) {
			wait_for_completion_killable(&mfd->pan_comp);
		}
		/* waiting for this update to complete */
		mfd->pan_waiting = TRUE;
		wait_for_completion_killable(&mfd->pan_comp);
	} else
		mfd->dma_fnc(mfd);
}

void mdp_refresh_screen(unsigned long data)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)data;

	if ((mfd->sw_currently_refreshing) && (mfd->sw_refreshing_enable)) {
		init_timer(&mfd->refresh_timer);
		mfd->refresh_timer.function = mdp_refresh_screen;
		mfd->refresh_timer.data = data;

		if (mfd->dma->busy)
			/* come back in 1 msec */
			mfd->refresh_timer.expires = jiffies + (HZ / 1000);
		else
			mfd->refresh_timer.expires =
			    jiffies + mfd->refresh_timer_duration;

		add_timer(&mfd->refresh_timer);

		if (!mfd->dma->busy) {
			if (!queue_work(mdp_dma_wq, &mfd->dma_update_worker)) {
				MSM_FB_DEBUG("mdp_dma: can't queue_work! -> \
			MDP/MDDI/LCD clock speed needs to be increased\n");
			}
		}
	} else {
		if (!mfd->hw_refresh)
			complete(&mfd->refresher_comp);
	}
}
