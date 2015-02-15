/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/bug.h>
#include <linux/of_gpio.h>

#include <asm/system.h>
#include <asm/mach-types.h>

#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/clk.h>
#include <mach/dma.h>

#include "mdss_panel.h"
#include "mdss_edp.h"

static char *edp_buf_init(struct edp_buf *eb, char *buf, int size)
{
	eb->start = buf;
	eb->size = size;
	eb->data = eb->start;
	eb->end = eb->start + eb->size;
	eb->len = 0;
	eb->trans_num = 0;
	eb->i2c = 0;
	return eb->data;
}

static char *edp_buf_reset(struct edp_buf *eb)
{
	eb->data = eb->start;
	eb->len = 0;
	eb->trans_num = 0;
	eb->i2c = 0;
	return eb->data;
}

static char *edp_buf_push(struct edp_buf *eb, int len)
{
	eb->data += len;
	eb->len += len;
	return eb->data;
}

static int edp_buf_trailing(struct edp_buf *eb)
{
	return (int)(eb->end - eb->data);
}

static int edp_buf_add_cmd(struct edp_buf *eb, struct edp_cmd *cmd)
{
	char data;
	char *bp, *cp;
	int i, len;

	if (cmd->read)	
		len = 4;
	else
		len = cmd->len + 4;

	if (edp_buf_trailing(eb) < len)
		return 0;

	if ((eb->len + len) > 128)
		return 0;

	bp = eb->data;
	data = cmd->addr >> 16;
	data &=  0x0f;	
	if (cmd->read)
		data |=  BIT(4);
	*bp++ = data;
	*bp++ = cmd->addr >> 8;
	*bp++ = cmd->addr;
	*bp++ = cmd->len - 1;

	if (!cmd->read) { 
		cp = cmd->datap;
		for (i = 0; i < cmd->len; i++)
			*bp++ = *cp++;
	}
	edp_buf_push(eb, len);

	if (cmd->i2c)
		eb->i2c++;

	eb->trans_num++;	

	return cmd->len - 1;
}

static int edp_cmd_fifo_tx(struct edp_buf *tp, unsigned char *base)
{
	u32 data;
	char *dp;
	int len, cnt;

	len = tp->len;	
	if (len == 0)
		return 0;

	cnt = 0;
	dp = tp->start;

	while (cnt < len) {
		data = *dp; 
		data <<= 8;
		data &= 0x00ff00; 
		if (cnt == 0)
			data |= BIT(31);  
		pr_debug("%s: data=%x\n", __func__, data);
		edp_write(base + EDP_AUX_DATA, data);
		cnt++;
		dp++;
	}

	data = (tp->trans_num - 1);
	if (tp->i2c)
		data |= BIT(8); 

	data |= BIT(9); 
	pr_debug("%s: data=%x\n", __func__, data);
	edp_write(base + EDP_AUX_TRANS_CTRL, data);

	return tp->len;
}

static int edp_cmd_fifo_rx(struct edp_buf *rp, int len, unsigned char *base)
{
	u32 data;
	char *dp;
	int i;

	data = 0; 
	data |= BIT(31);  
	data |= BIT(0);	
	edp_write(base + EDP_AUX_DATA, data);

	dp = rp->data;

	
	data = edp_read(base + EDP_AUX_DATA);
	for (i = 0; i < len; i++) {
		data = edp_read(base + EDP_AUX_DATA);
		pr_debug("%s: data=%x\n", __func__, data);
		*dp++ = (char)((data >> 8) & 0xff);
	}

	rp->len = len;
	return len;
}

static int edp_aux_write_cmds(struct mdss_edp_drv_pdata *ep,
					struct edp_cmd *cmd)
{
	struct edp_cmd *cm;
	struct edp_buf *tp;
	int len, ret;

	mutex_lock(&ep->aux_mutex);
	ep->aux_cmd_busy = 1;

	tp = &ep->txp;
	edp_buf_reset(tp);

	cm = cmd;
	while (cm) {
		pr_debug("%s: i2c=%d read=%d addr=%x len=%d next=%d\n",
			__func__, cm->i2c, cm->read, cm->addr, cm->len,
			cm->next);
		ret = edp_buf_add_cmd(tp, cm);
		if (ret <= 0)
			break;
		if (cm->next == 0)
			break;
		cm++;
	}

	if (tp->i2c)
		ep->aux_cmd_i2c = 1;
	else
		ep->aux_cmd_i2c = 0;

	INIT_COMPLETION(ep->aux_comp);

	len = edp_cmd_fifo_tx(&ep->txp, ep->base);

	wait_for_completion(&ep->aux_comp);

	if (ep->aux_error_num == EDP_AUX_ERR_NONE)
		ret = len;
	else
		ret = ep->aux_error_num;

	ep->aux_cmd_busy = 0;
	mutex_unlock(&ep->aux_mutex);
	return  ret;
}

static int edp_aux_read_cmds(struct mdss_edp_drv_pdata *ep,
				struct edp_cmd *cmds)
{
	struct edp_cmd *cm;
	struct edp_buf *tp;
	struct edp_buf *rp;
	int len, ret;

	mutex_lock(&ep->aux_mutex);
	ep->aux_cmd_busy = 1;

	tp = &ep->txp;
	rp = &ep->rxp;
	edp_buf_reset(tp);
	edp_buf_reset(rp);

	cm = cmds;
	len = 0;
	while (cm) {
		pr_debug("%s: i2c=%d read=%d addr=%x len=%d next=%d\n",
			__func__, cm->i2c, cm->read, cm->addr, cm->len,
			cm->next);
		ret = edp_buf_add_cmd(tp, cm);
		len += cm->len;
		if (ret <= 0)
			break;
		if (cm->next == 0)
			break;
		cm++;
	}

	if (tp->i2c)
		ep->aux_cmd_i2c = 1;
	else
		ep->aux_cmd_i2c = 0;

	INIT_COMPLETION(ep->aux_comp);

	edp_cmd_fifo_tx(tp, ep->base);

	wait_for_completion(&ep->aux_comp);

	if (ep->aux_error_num == EDP_AUX_ERR_NONE)
		ret = edp_cmd_fifo_rx(rp, len, ep->base);
	else
		ret = ep->aux_error_num;

	ep->aux_cmd_busy = 0;
	mutex_unlock(&ep->aux_mutex);

	return ret;
}

void edp_aux_native_handler(struct mdss_edp_drv_pdata *ep, u32 isr)
{

	pr_debug("%s: isr=%x\n", __func__, isr);

	if (isr & EDP_INTR_AUX_I2C_DONE)
		ep->aux_error_num = EDP_AUX_ERR_NONE;
	else if (isr & EDP_INTR_WRONG_ADDR)
		ep->aux_error_num = EDP_AUX_ERR_ADDR;
	else if (isr & EDP_INTR_TIMEOUT)
		ep->aux_error_num = EDP_AUX_ERR_TOUT;
	if (isr & EDP_INTR_NACK_DEFER)
		ep->aux_error_num = EDP_AUX_ERR_NACK;

	complete(&ep->aux_comp);
}

void edp_aux_i2c_handler(struct mdss_edp_drv_pdata *ep, u32 isr)
{

	pr_debug("%s: isr=%x\n", __func__, isr);

	if (isr & EDP_INTR_AUX_I2C_DONE) {
		if (isr & (EDP_INTR_I2C_NACK | EDP_INTR_I2C_DEFER))
			ep->aux_error_num = EDP_AUX_ERR_NACK;
		else
			ep->aux_error_num = EDP_AUX_ERR_NONE;
	} else {
		if (isr & EDP_INTR_WRONG_ADDR)
			ep->aux_error_num = EDP_AUX_ERR_ADDR;
		else if (isr & EDP_INTR_TIMEOUT)
			ep->aux_error_num = EDP_AUX_ERR_TOUT;
		if (isr & EDP_INTR_NACK_DEFER)
			ep->aux_error_num = EDP_AUX_ERR_NACK;
		if (isr & EDP_INTR_I2C_NACK)
			ep->aux_error_num = EDP_AUX_ERR_NACK;
		if (isr & EDP_INTR_I2C_DEFER)
			ep->aux_error_num = EDP_AUX_ERR_NACK;
	}

	complete(&ep->aux_comp);
}

static int edp_aux_write_buf(struct mdss_edp_drv_pdata *ep, u32 addr,
				char *buf, int len, int i2c)
{
	struct edp_cmd	cmd;

	cmd.read = 0;
	cmd.i2c = i2c;
	cmd.addr = addr;
	cmd.datap = buf;
	cmd.len = len & 0x0ff;
	cmd.next = 0;

	return edp_aux_write_cmds(ep, &cmd);
}

static int edp_aux_read_buf(struct mdss_edp_drv_pdata *ep, u32 addr,
				int len, int i2c)
{
	struct edp_cmd cmd;

	cmd.read = 1;
	cmd.i2c = i2c;
	cmd.addr = addr;
	cmd.datap = NULL;
	cmd.len = len & 0x0ff;
	cmd.next = 0;

	return edp_aux_read_cmds(ep, &cmd);
}

static char edid_hdr[8] = {0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00};

int edp_edid_buf_error(char *buf, int len)
{
	char *bp;
	int i;
	char csum = 0;

	bp = buf;
	if (len < 128) {
		pr_err("%s: Error: len=%x\n", __func__, len);
		return -EINVAL;
	}

	for (i = 0; i < 128; i++)
		csum += *bp++;

	if (csum != 0) {
		pr_err("%s: Error: csum=%x\n", __func__, csum);
		return -EINVAL;
	}

	if (strncmp(buf, edid_hdr, strlen(edid_hdr))) {
		pr_err("%s: Error: header\n", __func__);
		return -EINVAL;
	}

	return 0;
}


void edp_extract_edid_manufacturer(struct edp_edid *edid, char *buf)
{
	char *bp;
	char data;

	bp = &buf[8];
	data = *bp & 0x7f;
	data >>= 2;
	edid->id_name[0] = 'A' + data - 1;
	data = *bp & 0x03;
	data <<= 3;
	bp++;
	data |= (*bp >> 5);
	edid->id_name[1] = 'A' + data - 1;
	data = *bp & 0x1f;
	edid->id_name[2] = 'A' + data - 1;
	edid->id_name[3] = 0;

	pr_debug("%s: edid manufacturer = %s\n", __func__, edid->id_name);
}

void edp_extract_edid_product(struct edp_edid *edid, char *buf)
{
	char *bp;
	u32 data;

	bp = &buf[0x0a];
	data =  *bp;
	edid->id_product = *bp++;
	edid->id_product &= 0x0ff;
	data = *bp & 0x0ff;
	data <<= 8;
	edid->id_product |= data;

	pr_debug("%s: edid product = 0x%x\n", __func__, edid->id_product);
};

void edp_extract_edid_version(struct edp_edid *edid, char *buf)
{
	edid->version = buf[0x12];
	edid->revision = buf[0x13];
	pr_debug("%s: edid version = %d.%d\n", __func__, edid->version,
			edid->revision);
};

void edp_extract_edid_ext_block_cnt(struct edp_edid *edid, char *buf)
{
	edid->ext_block_cnt = buf[0x7e];
	pr_debug("%s: edid extension = %d\n", __func__,
			edid->ext_block_cnt);
};

void edp_extract_edid_video_support(struct edp_edid *edid, char *buf)
{
	char *bp;

	bp = &buf[0x14];
	if (*bp & 0x80) {
		edid->video_intf = *bp & 0x0f;
		
		edid->color_depth = ((*bp & 0x70) >> 4); 
		if (edid->color_depth) {
			edid->color_depth *= 2;
			edid->color_depth += 4;
		}
		pr_debug("%s: Digital Video intf=%d color_depth=%d\n",
			 __func__, edid->video_intf, edid->color_depth);
	} else {
		pr_err("%s: Error, Analog video interface\n", __func__);
	}
};

void edp_extract_edid_feature(struct edp_edid *edid, char *buf)
{
	char *bp;
	char data;

	bp = &buf[0x18];
	data = *bp;
	data &= 0xe0;
	data >>= 5;
	if (data == 0x01)
		edid->dpm = 1; 

	if (edid->video_intf) {
		if (*bp & 0x80) {
			
			edid->color_format = *bp & 0x18;
			edid->color_format >>= 3;
		}
	}

	pr_debug("%s: edid dpm=%d color_format=%d\n", __func__,
			edid->dpm, edid->color_format);
};

void edp_extract_edid_detailed_timing_description(struct edp_edid *edid,
		char *buf)
{
	char *bp;
	u32 data;
	struct display_timing_desc *dp;

	dp = &edid->timing[0];

	bp = &buf[0x36];
	dp->pclk = 0;
	dp->pclk = *bp++; 
	dp->pclk |= (*bp++ << 8); 

	dp->h_addressable = *bp++; 

	if (dp->pclk == 0 && dp->h_addressable == 0)
		return;	

	dp->pclk *= 10000;

	dp->h_blank = *bp++;
	data = *bp & 0xf0; 
	data  <<= 4;
	dp->h_addressable |= data;

	data = *bp++ & 0x0f;
	data <<= 8;
	dp->h_blank |= data;

	dp->v_addressable = *bp++; 
	dp->v_blank = *bp++; 
	data = *bp & 0xf0; 
	data  <<= 4;
	dp->v_addressable |= data;

	data = *bp++ & 0x0f;
	data <<= 8;
	dp->v_blank |= data;

	dp->h_fporch = *bp++; 
	dp->h_sync_pulse = *bp++; 

	dp->v_fporch = *bp & 0x0f0; 
	dp->v_fporch  >>= 4;
	dp->v_sync_pulse = *bp & 0x0f;

	bp++;
	data = *bp & 0xc0; 
	data <<= 2;
	dp->h_fporch |= data;

	data = *bp & 0x30;
	data <<= 4;
	dp->h_sync_pulse |= data;

	data = *bp & 0x0c;
	data <<= 2;
	dp->v_fporch |= data;

	data = *bp & 0x03;
	data <<= 4;
	dp->v_sync_pulse |= data;

	bp++;
	dp->width_mm = *bp++; 
	dp->height_mm = *bp++; 
	data = *bp & 0x0f0; 
	data <<= 4;
	dp->width_mm |= data;
	data = *bp & 0x0f;
	data <<= 8;
	dp->height_mm |= data;

	bp++;
	dp->h_border = *bp++; 
	dp->v_border = *bp++; 

	
	dp->interlaced = *bp & 0x80; 

	dp->stereo = *bp & 0x60;
	dp->stereo >>= 5;

	data = *bp & 0x1e; 
	data >>= 1;
	dp->sync_type = data & 0x08;
	dp->sync_type >>= 3;	
	if (dp->sync_type) {
		dp->sync_separate = data & 0x04;
		dp->sync_separate >>= 2;
		if (dp->sync_separate) {
			if (data & 0x02)
				dp->vsync_pol = 1; 
			else
				dp->vsync_pol = 0;

			if (data & 0x01)
				dp->hsync_pol = 1; 
			else
				dp->hsync_pol = 0; 
		}
	}

	pr_debug("%s: pixel_clock = %d\n", __func__, dp->pclk);

	pr_debug("%s: horizontal=%d, blank=%d, porch=%d, sync=%d\n"
			, __func__, dp->h_addressable, dp->h_blank,
			dp->h_fporch, dp->h_sync_pulse);
	pr_debug("%s: vertical=%d, blank=%d, porch=%d, vsync=%d\n"
			, __func__, dp->v_addressable, dp->v_blank,
			dp->v_fporch, dp->v_sync_pulse);
	pr_debug("%s: panel size in mm, width=%d height=%d\n", __func__,
			dp->width_mm, dp->height_mm);
	pr_debug("%s: panel border horizontal=%d vertical=%d\n", __func__,
				dp->h_border, dp->v_border);
	pr_debug("%s: flags: interlaced=%d stereo=%d sync_type=%d sync_sep=%d\n"
			, __func__, dp->interlaced, dp->stereo,
			dp->sync_type, dp->sync_separate);
	pr_debug("%s: polarity vsync=%d, hsync=%d", __func__,
			dp->vsync_pol, dp->hsync_pol);
}



static int edp_aux_chan_ready(struct mdss_edp_drv_pdata *ep)
{
	int cnt, ret;
	char data = 0;

	for (cnt = 5; cnt; cnt--) {
		ret = edp_aux_write_buf(ep, 0x50, &data, 1, 1);
		pr_debug("%s: ret=%d\n", __func__, ret);
		if (ret >= 0)
			break;
		msleep(100);
	}

	if (cnt <= 0) {
		pr_err("%s: aux chan NOT ready\n", __func__);
		return 0;
	}

	return 1;
}

static int edp_sink_edid_read(struct mdss_edp_drv_pdata *ep, int block)
{
	struct edp_buf *rp;
	int cnt, rlen;
	int ret = 0;

	ret = edp_aux_chan_ready(ep);
	if (ret == 0) {
		pr_err("%s: aux chan NOT ready\n", __func__);
		return ret;
	}

	for (cnt = 5; cnt; cnt--) {
		rlen = edp_aux_read_buf(ep, 0x50, 128, 1);
		if (rlen > 0) {
			pr_debug("%s: rlen=%d\n", __func__, rlen);

			rp = &ep->rxp;
			if (!edp_edid_buf_error(rp->data, rp->len))
				break;
		}
	}

	if (cnt <= 0) {
		pr_err("%s: Failed\n", __func__);
		return -EINVAL;
	}

	edp_extract_edid_manufacturer(&ep->edid, rp->data);
	edp_extract_edid_product(&ep->edid, rp->data);
	edp_extract_edid_version(&ep->edid, rp->data);
	edp_extract_edid_ext_block_cnt(&ep->edid, rp->data);
	edp_extract_edid_video_support(&ep->edid, rp->data);
	edp_extract_edid_feature(&ep->edid, rp->data);
	edp_extract_edid_detailed_timing_description(&ep->edid, rp->data);

	return 128;
}

static void edp_sink_capability_read(struct mdss_edp_drv_pdata *ep,
				int len)
{
	char *bp;
	char data;
	struct dpcd_cap *cap;
	struct edp_buf *rp;
	int rlen;

	rlen = edp_aux_read_buf(ep, 0, len, 0);
	if (rlen <= 0) {
		pr_err("%s: edp aux read failed\n", __func__);
		return;
	}
	rp = &ep->rxp;
	cap = &ep->dpcd;
	bp = rp->data;

	data = *bp++; 
	cap->major = (data >> 4) & 0x0f;
	cap->minor = data & 0x0f;
	if (--rlen <= 0)
		return;
	pr_debug("%s: version: %d.%d\n", __func__, cap->major, cap->minor);

	data = *bp++; 
	
	cap->max_link_rate = data;
	if (--rlen <= 0)
		return;
	pr_debug("%s: link_rate=%d\n", __func__, cap->max_link_rate);

	data = *bp++; 
	if (data & BIT(7))
		cap->enhanced_frame++;

	if (data & 0x40)
		cap->flags |=  DPCD_TPS3;
	data &= 0x0f;
	cap->max_lane_count = data;
	if (--rlen <= 0)
		return;
	pr_debug("%s: lane_count=%d\n", __func__, cap->max_lane_count);

	data = *bp++; 
	if (data & BIT(0)) {
		cap->flags |= DPCD_MAX_DOWNSPREAD_0_5;
		pr_debug("%s: max_downspread\n", __func__);
	}

	if (data & BIT(6)) {
		cap->flags |= DPCD_NO_AUX_HANDSHAKE;
		pr_debug("%s: NO Link Training\n", __func__);
	}
	if (--rlen <= 0)
		return;

	data = *bp++; 
	cap->num_rx_port = (data & BIT(0)) + 1;
	pr_debug("%s: rx_ports=%d", __func__, cap->num_rx_port);
	if (--rlen <= 0)
		return;

	bp += 3;	
	rlen -= 3;
	if (rlen <= 0)
		return;

	data = *bp++; 
	if (data & BIT(1)) {
		cap->flags |= DPCD_PORT_0_EDID_PRESENTED;
		pr_debug("%s: edid presented\n", __func__);
	}
	if (--rlen <= 0)
		return;

	data = *bp++; 
	cap->rx_port0_buf_size = (data + 1) * 32;
	pr_debug("%s: lane_buf_size=%d", __func__, cap->rx_port0_buf_size);
	if (--rlen <= 0)
		return;

	bp += 2; 
	rlen -= 2;
	if (rlen <= 0)
		return;

	data = *bp++;	
	cap->i2c_speed_ctrl = data;
	if (cap->i2c_speed_ctrl > 0)
		pr_debug("%s: i2c_rate=%d", __func__, cap->i2c_speed_ctrl);
	if (--rlen <= 0)
		return;

	data = *bp++;	
	cap->scrambler_reset = data & BIT(0);
	pr_debug("%s: scrambler_reset=%d\n", __func__,
					cap->scrambler_reset);

	if (data & BIT(1))
		cap->enhanced_frame++;

	pr_debug("%s: enhanced_framing=%d\n", __func__,
					cap->enhanced_frame);
	if (--rlen <= 0)
		return;

	data = *bp++; 
	if (data == 0)
		cap->training_read_interval = 4000; 
	else
		cap->training_read_interval = 4000 * data; 
	pr_debug("%s: training_interval=%d\n", __func__,
			 cap->training_read_interval);
}

static int edp_link_status_read(struct mdss_edp_drv_pdata *ep, int len)
{
	char *bp;
	char data;
	struct dpcd_link_status *sp;
	struct edp_buf *rp;
	int rlen;

	pr_debug("%s: len=%d", __func__, len);
	
	rlen = edp_aux_read_buf(ep, 0x202, len, 0);
	if (rlen < len) {
		pr_err("%s: edp aux read failed\n", __func__);
		return 0;
	}
	rp = &ep->rxp;
	bp = rp->data;
	sp = &ep->link_status;

	data = *bp++; 
	sp->lane_01_status = data; 

	data = *bp++; 
	sp->lane_23_status = data; 

	data = *bp++; 
	sp->interlane_align_done = (data & BIT(0));
	sp->downstream_port_status_changed = (data & BIT(6));
	sp->link_status_updated = (data & BIT(7));

	data = *bp++; 
	sp->port_0_in_sync = (data & BIT(0));
	sp->port_1_in_sync = (data & BIT(1));

	data = *bp++; 
	sp->req_voltage_swing[0] = data & 0x03;
	data >>= 2;
	sp->req_pre_emphasis[0] = data & 0x03;
	data >>= 2;
	sp->req_voltage_swing[1] = data & 0x03;
	data >>= 2;
	sp->req_pre_emphasis[1] = data & 0x03;

	data = *bp++; 
	sp->req_voltage_swing[2] = data & 0x03;
	data >>= 2;
	sp->req_pre_emphasis[2] = data & 0x03;
	data >>= 2;
	sp->req_voltage_swing[3] = data & 0x03;
	data >>= 2;
	sp->req_pre_emphasis[3] = data & 0x03;

	return len;
}

static int edp_cap_lane_rate_set(struct mdss_edp_drv_pdata *ep)
{
	char buf[4];
	int len = 0;
	struct dpcd_cap *cap;

	cap = &ep->dpcd;

	pr_debug("%s: bw=%x lane=%d\n", __func__, ep->link_rate, ep->lane_cnt);
	buf[0] = ep->link_rate;
	buf[1] = ep->lane_cnt;
	if (cap->enhanced_frame)
		buf[1] |= 0x80;
	len = edp_aux_write_buf(ep, 0x100, buf, 2, 0);

	return len;
}

static int edp_lane_set_write(struct mdss_edp_drv_pdata *ep, int voltage_level,
		int pre_emphasis_level)
{
	int i;
	char buf[4];

	if (voltage_level >= DPCD_LINK_VOLTAGE_MAX)
		voltage_level |= 0x04;

	if (pre_emphasis_level >= DPCD_LINK_PRE_EMPHASIS_MAX)
		pre_emphasis_level |= 0x04;

	pre_emphasis_level <<= 3;

	for (i = 0; i < 4; i++)
		buf[i] = voltage_level | pre_emphasis_level;

	pr_debug("%s: p|v=0x%x", __func__, voltage_level | pre_emphasis_level);
	return edp_aux_write_buf(ep, 0x103, buf, 4, 0);
}

static int edp_train_pattern_set_write(struct mdss_edp_drv_pdata *ep,
						int pattern)
{
	char buf[4];

	pr_debug("%s: pattern=%x\n", __func__, pattern);
	buf[0] = pattern;
	return edp_aux_write_buf(ep, 0x102, buf, 1, 0);
}

static int edp_sink_clock_recovery_done(struct mdss_edp_drv_pdata *ep)
{
	u32 mask;
	u32 data;

	if (ep->lane_cnt == 1) {
		mask = 0x01;	
		data = ep->link_status.lane_01_status;
	} else if (ep->lane_cnt == 2) {
		mask = 0x011; 
		data = ep->link_status.lane_01_status;
	} else {
		mask = 0x01111; 
		data = ep->link_status.lane_23_status;
		data <<= 8;
		data |= ep->link_status.lane_01_status;
	}

	pr_debug("%s: data=%x mask=%x\n", __func__, data, mask);
	data &= mask;
	if (data == mask) 
		return 1;

	return 0;
}

static int edp_sink_channel_eq_done(struct mdss_edp_drv_pdata *ep)
{
	u32 mask;
	u32 data;

	pr_debug("%s:\n", __func__);

	if (!ep->link_status.interlane_align_done) { 
		pr_err("%s: interlane align failed\n", __func__);
		return 0;
	}

	if (ep->lane_cnt == 1) {
		mask = 0x7;
		data = ep->link_status.lane_01_status;
	} else if (ep->lane_cnt == 2) {
		mask = 0x77;
		data = ep->link_status.lane_01_status;
	} else {
		mask = 0x7777;
		data = ep->link_status.lane_23_status;
		data <<= 8;
		data |= ep->link_status.lane_01_status;
	}

	pr_debug("%s: data=%x mask=%x\n", __func__, data, mask);

	data &= mask;
	if (data == mask)
		return 1;

	return 0;
}

void edp_sink_train_set_adjust(struct mdss_edp_drv_pdata *ep)
{
	int i;
	int max = 0;


	
	for (i = 0; i < ep->lane_cnt; i++) {
		pr_debug("%s: lane=%d req_voltage_swing=%d",
			__func__, i, ep->link_status.req_voltage_swing[i]);
		if (max < ep->link_status.req_voltage_swing[i])
			max = ep->link_status.req_voltage_swing[i];
	}

	ep->v_level = max;

	
	max = 0;
	for (i = 0; i < ep->lane_cnt; i++) {
		pr_debug(" %s: lane=%d req_pre_emphasis=%d",
			__func__, i, ep->link_status.req_pre_emphasis[i]);
		if (max < ep->link_status.req_pre_emphasis[i])
			max = ep->link_status.req_pre_emphasis[i];
	}

	ep->p_level = max;
	pr_debug("%s: v_level=%d, p_level=%d", __func__,
					ep->v_level, ep->p_level);
}

static void edp_host_train_set(struct mdss_edp_drv_pdata *ep, int train)
{
	int bit, cnt;
	u32 data;


	bit = 1;
	bit  <<=  (train - 1);
	pr_debug("%s: bit=%d train=%d\n", __func__, bit, train);
	edp_write(ep->base + EDP_STATE_CTRL, bit);

	bit = 8;
	bit <<= (train - 1);
	cnt = 10;
	while (cnt--) {
		data = edp_read(ep->base + EDP_MAINLINK_READY);
		if (data & bit)
			break;
	}

	if (cnt == 0)
		pr_err("%s: set link_train=%d failed\n", __func__, train);
}

char vm_pre_emphasis[4][4] = {
	{0x03, 0x06, 0x09, 0x0C},	
	{0x03, 0x06, 0x09, 0xFF},	
	{0x03, 0x06, 0xFF, 0xFF},	
	{0x03, 0xFF, 0xFF, 0xFF}	
};

char vm_voltage_swing[4][4] = {
	{0x14, 0x18, 0x1A, 0x1E}, 
	{0x18, 0x1A, 0x1E, 0xFF}, 
	{0x1A, 0x1E, 0xFF, 0xFF}, 
	{0x1E, 0xFF, 0xFF, 0xFF}  
};

static void edp_voltage_pre_emphasise_set(struct mdss_edp_drv_pdata *ep)
{
	u32 value0 = 0;
	u32 value1 = 0;

	pr_debug("%s: v=%d p=%d\n", __func__, ep->v_level, ep->p_level);

	value0 = vm_pre_emphasis[(int)(ep->v_level)][(int)(ep->p_level)];
	value1 = vm_voltage_swing[(int)(ep->v_level)][(int)(ep->p_level)];

	
	if (value0 != 0xFF && value1 != 0xFF) {
		edp_write(ep->base + EDP_PHY_EDPPHY_GLB_VM_CFG0, value0);
		edp_write(ep->base + EDP_PHY_EDPPHY_GLB_VM_CFG1, value1);
		pr_debug("%s: value0=0x%x value1=0x%x", __func__,
						value0, value1);
		edp_lane_set_write(ep, ep->v_level, ep->p_level);
	}

}

static int edp_start_link_train_1(struct mdss_edp_drv_pdata *ep)
{
	int tries, old_v_level;
	int ret = 0;

	pr_debug("%s:", __func__);

	edp_host_train_set(ep, 0x01); 
	edp_voltage_pre_emphasise_set(ep);
	edp_train_pattern_set_write(ep, 0x21); 

	tries = 0;
	old_v_level = ep->v_level;
	while (1) {
		usleep(ep->dpcd.training_read_interval);

		edp_link_status_read(ep, 6);
		if (edp_sink_clock_recovery_done(ep)) {
			ret = 0;
			break;
		}

		if (ep->v_level == DPCD_LINK_VOLTAGE_MAX) {
			ret = -1;
			break;	
		}

		if (old_v_level == ep->v_level) {
			tries++;
			if (tries >= 5) {
				ret = -1;
				break;	
			}
		} else {
			tries = 0;
			old_v_level = ep->v_level;
		}

		edp_sink_train_set_adjust(ep);
		edp_voltage_pre_emphasise_set(ep);
	}

	return ret;
}

static int edp_start_link_train_2(struct mdss_edp_drv_pdata *ep)
{
	int tries;
	int ret = 0;
	char pattern;

	pr_debug("%s:", __func__);

	if (ep->dpcd.flags & DPCD_TPS3)
		pattern = 0x03;
	else
		pattern = 0x02;

	edp_host_train_set(ep, pattern); 
	edp_voltage_pre_emphasise_set(ep);
	edp_train_pattern_set_write(ep, pattern | 0x20);

	tries = 0;
	while (1) {
		usleep(ep->dpcd.training_read_interval);

		edp_link_status_read(ep, 6);

		if (edp_sink_channel_eq_done(ep)) {
			ret = 0;
			break;
		}

		tries++;
		if (tries > 5) {
			ret = -1;
			break;
		}

		edp_sink_train_set_adjust(ep);
		edp_voltage_pre_emphasise_set(ep);
	}

	return ret;
}

static int edp_link_rate_down_shift(struct mdss_edp_drv_pdata *ep)
{
	u32 prate, lrate;
	int rate, lane, max_lane;
	int changed = 0;

	rate = ep->link_rate;
	lane = ep->lane_cnt;
	max_lane = ep->dpcd.max_lane_count;

	prate = ep->pixel_rate;
	prate /= 1000;	
	prate *= ep->bpp;
	prate /= 8; 

	if (rate > EDP_LINK_RATE_162 && rate <= EDP_LINK_RATE_MAX) {
		rate -= 4;		
		changed++;
	}

	if (changed) {
		if (lane >= 1 && lane < max_lane)
			lane <<= 1;	

		lrate = 270000000; 
		lrate /= 1000; 
		lrate *= rate;
		lrate /= 10; 
		lrate *= lane;

		pr_debug("%s: new lrate=%u prate=%u rate=%d lane=%d p=%d b=%d\n",
		__func__, lrate, prate, rate, lane, ep->pixel_rate, ep->bpp);

		if (lrate > prate) {
			ep->link_rate = rate;
			ep->lane_cnt = lane;
			pr_debug("%s: new rate=%d %d\n", __func__, rate, lane);
			return 0;
		}
	}

	
	return -EINVAL;
}

static void edp_clear_training_pattern(struct mdss_edp_drv_pdata *ep)
{
	pr_debug("%s:\n", __func__);
	edp_train_pattern_set_write(ep, 0);
	usleep(ep->dpcd.training_read_interval);
}

static int edp_aux_link_train(struct mdss_edp_drv_pdata *ep)
{
	int ret = 0;

	pr_debug("%s", __func__);
	ret = edp_aux_chan_ready(ep);
	if (ret == 0) {
		pr_err("%s: LINK Train failed: aux chan NOT ready\n", __func__);
		complete(&ep->train_comp);
		return ret;
	}

	edp_write(ep->base + EDP_MAINLINK_CTRL, 0x1);

	mdss_edp_sink_power_state(ep, SINK_POWER_ON);

train_start:
	ep->v_level = 0; 
	ep->p_level = 0;
	edp_cap_lane_rate_set(ep);
	mdss_edp_config_ctrl(ep);
	mdss_edp_lane_power_ctrl(ep, 1);

	mdss_edp_state_ctrl(ep, 0);
	edp_clear_training_pattern(ep);
	usleep(ep->dpcd.training_read_interval);

	ret = edp_start_link_train_1(ep);
	if (ret < 0) {
		if (edp_link_rate_down_shift(ep) == 0) {
			goto train_start;
		} else {
			pr_err("%s: Training 1 failed", __func__);
			ret = -1;
			goto clear;
		}
	}

	pr_debug("%s: Training 1 completed successfully", __func__);

	mdss_edp_state_ctrl(ep, 0);
	edp_clear_training_pattern(ep);
	ret = edp_start_link_train_2(ep);
	if (ret < 0) {
		if (edp_link_rate_down_shift(ep) == 0) {
			goto train_start;
		} else {
			pr_err("%s: Training 2 failed", __func__);
			ret = -1;
			goto clear;
		}
	}

	pr_debug("%s: Training 2 completed successfully", __func__);

	mdss_edp_state_ctrl(ep, ST_SEND_VIDEO);
clear:
	edp_clear_training_pattern(ep);

	complete(&ep->train_comp);
	return ret;
}

void mdss_edp_dpcd_cap_read(struct mdss_edp_drv_pdata *ep)
{
	edp_sink_capability_read(ep, 16);
}

int mdss_edp_dpcd_status_read(struct mdss_edp_drv_pdata *ep)
{
	struct dpcd_link_status *sp;
	int ret = 0; 

	ret = edp_link_status_read(ep, 6);

	if (ret) {
		sp = &ep->link_status;
		ret = sp->port_0_in_sync; 
	}

	return ret;
}

void mdss_edp_fill_link_cfg(struct mdss_edp_drv_pdata *ep)
{
	struct display_timing_desc *dp;

	dp = &ep->edid.timing[0];
	ep->pixel_rate = dp->pclk;
	ep->lane_cnt = ep->dpcd.max_lane_count;
	ep->link_rate = ep->dpcd.max_link_rate;

	pr_debug("%s: pclk=%d rate=%d lane=%d\n", __func__,
		ep->pixel_rate, ep->link_rate, ep->lane_cnt);

}

void mdss_edp_edid_read(struct mdss_edp_drv_pdata *ep, int block)
{
	edp_sink_edid_read(ep, block);
}

int mdss_edp_sink_power_state(struct mdss_edp_drv_pdata *ep, char state)
{
	int ret;

	ret = edp_aux_write_buf(ep, 0x600, &state, 1, 0);
	pr_debug("%s: state=%d ret=%d\n", __func__, state, ret);
	return ret;
}

int mdss_edp_link_train(struct mdss_edp_drv_pdata *ep)
{
	int ret;

	mutex_lock(&ep->train_mutex);
	ret = edp_aux_link_train(ep);
	mutex_unlock(&ep->train_mutex);
	return ret;
}

void mdss_edp_aux_init(struct mdss_edp_drv_pdata *ep)
{
	mutex_init(&ep->aux_mutex);
	mutex_init(&ep->train_mutex);
	init_completion(&ep->aux_comp);
	init_completion(&ep->train_comp);
	init_completion(&ep->idle_comp);
	init_completion(&ep->video_comp);
	complete(&ep->train_comp); 
	complete(&ep->video_comp); 

	edp_buf_init(&ep->txp, ep->txbuf, sizeof(ep->txbuf));
	edp_buf_init(&ep->rxp, ep->rxbuf, sizeof(ep->rxbuf));
}
