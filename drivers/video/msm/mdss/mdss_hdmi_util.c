/* Copyright (c) 2010-2013, The Linux Foundation. All rights reserved.
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

#include <linux/io.h>
#include <mach/board.h>
#include "mdss_hdmi_util.h"

static struct msm_hdmi_mode_timing_info
	hdmi_supported_video_mode_lut[HDMI_VFRMT_MAX];

void hdmi_del_supported_mode(u32 mode)
{
	struct msm_hdmi_mode_timing_info *ret = NULL;
	DEV_DBG("%s: removing %s\n", __func__,
		 msm_hdmi_mode_2string(mode));
	ret = &hdmi_supported_video_mode_lut[mode];
	if (ret != NULL && ret->supported)
		ret->supported = false;
}

const struct msm_hdmi_mode_timing_info *hdmi_get_supported_mode(u32 mode)
{
	const struct msm_hdmi_mode_timing_info *ret = NULL;

	if (mode >= HDMI_VFRMT_MAX)
		return NULL;

	ret = &hdmi_supported_video_mode_lut[mode];

	if (ret == NULL || !ret->supported)
		return NULL;

	return ret;
} 

int hdmi_get_video_id_code(struct msm_hdmi_mode_timing_info *timing_in)
{
	int i, vic = -1;
	struct msm_hdmi_mode_timing_info *supported_timing;

	if (!timing_in) {
		DEV_ERR("%s: invalid input\n", __func__);
		goto exit;
	}

	
	for (i = 0; i < HDMI_VFRMT_MAX; i++) {
		supported_timing = &hdmi_supported_video_mode_lut[i];

		if (!supported_timing->supported)
			continue;
		if (timing_in->active_h != supported_timing->active_h)
			continue;
		if (timing_in->front_porch_h != supported_timing->front_porch_h)
			continue;
		if (timing_in->pulse_width_h != supported_timing->pulse_width_h)
			continue;
		if (timing_in->back_porch_h != supported_timing->back_porch_h)
			continue;
		if (timing_in->active_v != supported_timing->active_v)
			continue;
		if (timing_in->front_porch_v != supported_timing->front_porch_v)
			continue;
		if (timing_in->pulse_width_v != supported_timing->pulse_width_v)
			continue;
		if (timing_in->back_porch_v != supported_timing->back_porch_v)
			continue;
		if (timing_in->pixel_freq != supported_timing->pixel_freq)
			continue;
		if (timing_in->refresh_rate != supported_timing->refresh_rate)
			continue;

		vic = (int)supported_timing->video_format;
		break;
	}

	if (vic < 0) {
		for (i = 0; i < HDMI_VFRMT_MAX; i++) {
			supported_timing = &hdmi_supported_video_mode_lut[i];
			if (!supported_timing->supported)
				continue;
			if (timing_in->active_h != supported_timing->active_h)
				continue;
			if (timing_in->active_v != supported_timing->active_v)
				continue;
			vic = (int)supported_timing->video_format;
			break;
		}
	}

	if (vic < 0) {
		DEV_ERR("%s: timing is not supported h=%d v=%d\n",
			__func__, timing_in->active_h, timing_in->active_v);
	}

exit:
	DEV_DBG("%s: vic = %d timing = %s\n", __func__, vic,
		msm_hdmi_mode_2string((u32)vic));

	return vic;
} 

void hdmi_setup_video_mode_lut(void)
{
	MSM_HDMI_MODES_INIT_TIMINGS(hdmi_supported_video_mode_lut);

	
	MSM_HDMI_MODES_SET_SUPP_TIMINGS(
		hdmi_supported_video_mode_lut, MSM_HDMI_MODES_CEA);

	
	MSM_HDMI_MODES_SET_SUPP_TIMINGS(
		hdmi_supported_video_mode_lut, MSM_HDMI_MODES_XTND);

	
	MSM_HDMI_MODES_SET_SUPP_TIMINGS(
		hdmi_supported_video_mode_lut, MSM_HDMI_MODES_DVI);
} 

const char *hdmi_get_single_video_3d_fmt_2string(u32 format)
{
	switch (format) {
	case TOP_AND_BOTTOM:	return "TAB";
	case FRAME_PACKING:	return "FP";
	case SIDE_BY_SIDE_HALF: return "SSH";
	}
	return "";
} 

ssize_t hdmi_get_video_3d_fmt_2string(u32 format, char *buf, u32 size)
{
	ssize_t ret, len = 0;
	ret = scnprintf(buf, size, "%s",
		hdmi_get_single_video_3d_fmt_2string(
			format & FRAME_PACKING));
	len += ret;

	if (len && (format & TOP_AND_BOTTOM))
		ret = scnprintf(buf + len, size - len, ":%s",
			hdmi_get_single_video_3d_fmt_2string(
				format & TOP_AND_BOTTOM));
	else
		ret = scnprintf(buf + len, size - len, "%s",
			hdmi_get_single_video_3d_fmt_2string(
				format & TOP_AND_BOTTOM));
	len += ret;

	if (len && (format & SIDE_BY_SIDE_HALF))
		ret = scnprintf(buf + len, size - len, ":%s",
			hdmi_get_single_video_3d_fmt_2string(
				format & SIDE_BY_SIDE_HALF));
	else
		ret = scnprintf(buf + len, size - len, "%s",
			hdmi_get_single_video_3d_fmt_2string(
				format & SIDE_BY_SIDE_HALF));
	len += ret;

	return len;
} 

static void hdmi_ddc_print_data(struct hdmi_tx_ddc_data *ddc_data,
	const char *caller)
{
	if (!ddc_data) {
		DEV_ERR("%s: invalid input\n", __func__);
		return;
	}

	DEV_DBG("%s: buf=%p, d_len=0x%x, d_addr=0x%x, no_align=%d\n",
		caller, ddc_data->data_buf, ddc_data->data_len,
		ddc_data->dev_addr, ddc_data->no_align);
	DEV_DBG("%s: offset=0x%x, req_len=0x%x, retry=%d, what=%s\n",
		caller, ddc_data->offset, ddc_data->request_len,
		ddc_data->retry, ddc_data->what);
} 

static int hdmi_ddc_clear_irq(struct hdmi_tx_ddc_ctrl *ddc_ctrl,
	char *what)
{
	u32 reg_val, time_out_count;

	if (!ddc_ctrl || !ddc_ctrl->io) {
		DEV_ERR("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	
	time_out_count = 0xFFFF;
	do {
		--time_out_count;
		
		DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_INT_CTRL,
			BIT(2) | BIT(1));
		reg_val = DSS_REG_R_ND(ddc_ctrl->io, HDMI_DDC_INT_CTRL);
	} while ((reg_val & BIT(0)) && time_out_count);

	if (!time_out_count) {
		DEV_ERR("%s[%s]: timedout\n", __func__, what);
		return -ETIMEDOUT;
	}

	return 0;
} 

static int hdmi_ddc_read_retry(struct hdmi_tx_ddc_ctrl *ddc_ctrl,
	struct hdmi_tx_ddc_data *ddc_data)
{
	u32 reg_val, ndx, time_out_count;
	int status = 0;
	int log_retry_fail;

	if (!ddc_ctrl || !ddc_ctrl->io || !ddc_data) {
		DEV_ERR("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	if (!ddc_data->data_buf) {
		status = -EINVAL;
		DEV_ERR("%s[%s]: invalid buf\n", __func__, ddc_data->what);
		goto error;
	}

	hdmi_ddc_print_data(ddc_data, __func__);

	log_retry_fail = ddc_data->retry != 1;
again:
	status = hdmi_ddc_clear_irq(ddc_ctrl, ddc_data->what);
	if (status)
		goto error;

	
	ddc_data->dev_addr &= 0xFE;

	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_DATA,
		BIT(31) | (ddc_data->dev_addr << 8));

	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_DATA, ddc_data->offset << 8);

	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_DATA,
		(ddc_data->dev_addr | BIT(0)) << 8);

	

	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_TRANS0, BIT(12) | BIT(16));

	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_TRANS1,
		BIT(0) | BIT(12) | BIT(13) | (ddc_data->request_len << 16));

	

	INIT_COMPLETION(ddc_ctrl->ddc_sw_done);
	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_CTRL, BIT(0) | BIT(20));

	time_out_count = wait_for_completion_timeout(
		&ddc_ctrl->ddc_sw_done, HZ/2);
	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_INT_CTRL, BIT(1));
	if (!time_out_count) {
		if (ddc_data->retry-- > 0) {
			DEV_INFO("%s: failed timout, retry=%d\n", __func__,
				ddc_data->retry);
			goto again;
		}
		status = -ETIMEDOUT;
		DEV_ERR("%s: timedout(7), Int Ctrl=%08x\n", __func__,
			DSS_REG_R(ddc_ctrl->io, HDMI_DDC_INT_CTRL));
		DEV_ERR("%s: DDC SW Status=%08x, HW Status=%08x\n",
			__func__,
			DSS_REG_R(ddc_ctrl->io, HDMI_DDC_SW_STATUS),
			DSS_REG_R(ddc_ctrl->io, HDMI_DDC_HW_STATUS));
		goto error;
	}

	
	reg_val = DSS_REG_R(ddc_ctrl->io, HDMI_DDC_SW_STATUS);
	reg_val &= BIT(12) | BIT(13) | BIT(14) | BIT(15);

	
	if (reg_val) {
		
		DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_CTRL, BIT(3));

		if (ddc_data->retry == 1)
			
			DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_CTRL, BIT(1));

		if (ddc_data->retry-- > 0) {
			DEV_DBG("%s(%s): failed NACK=0x%08x, retry=%d\n",
				__func__, ddc_data->what, reg_val,
				ddc_data->retry);
			DEV_DBG("%s: daddr=0x%02x,off=0x%02x,len=%d\n",
				__func__, ddc_data->dev_addr,
				ddc_data->offset, ddc_data->data_len);
			goto again;
		}
		status = -EIO;
		if (log_retry_fail) {
			DEV_ERR("%s(%s): failed NACK=0x%08x\n",
				__func__, ddc_data->what, reg_val);
			DEV_ERR("%s: daddr=0x%02x,off=0x%02x,len=%d\n",
				__func__, ddc_data->dev_addr,
				ddc_data->offset, ddc_data->data_len);
		}
		goto error;
	}

	
	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_DATA,
		BIT(0) | (3 << 16) | BIT(31));

	
	DSS_REG_R_ND(ddc_ctrl->io, HDMI_DDC_DATA);
	for (ndx = 0; ndx < ddc_data->data_len; ++ndx) {
		reg_val = DSS_REG_R_ND(ddc_ctrl->io, HDMI_DDC_DATA);
		ddc_data->data_buf[ndx] = (u8)((reg_val & 0x0000FF00) >> 8);
	}

	DEV_DBG("%s[%s] success\n", __func__, ddc_data->what);

error:
	return status;
} 

void hdmi_ddc_config(struct hdmi_tx_ddc_ctrl *ddc_ctrl, u16 ddc_ref_clk)
{
	if (!ddc_ctrl || !ddc_ctrl->io) {
		DEV_ERR("%s: invalid input\n", __func__);
		return;
	}

	
	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_SPEED, (10 << 16) | (2 << 0));

	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_SETUP, 0xFF000000);

	
	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_REF, (1 << 16) | ddc_ref_clk);
} 

int hdmi_ddc_isr(struct hdmi_tx_ddc_ctrl *ddc_ctrl)
{
	u32 ddc_int_ctrl;

	if (!ddc_ctrl || !ddc_ctrl->io) {
		DEV_ERR("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	ddc_int_ctrl = DSS_REG_R_ND(ddc_ctrl->io, HDMI_DDC_INT_CTRL);
	if ((ddc_int_ctrl & BIT(2)) && (ddc_int_ctrl & BIT(0))) {
		
		DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_INT_CTRL,
			ddc_int_ctrl | BIT(1));
		complete(&ddc_ctrl->ddc_sw_done);
	}

	DEV_DBG("%s: ddc_int_ctrl=%04x\n", __func__, ddc_int_ctrl);

	return 0;
} 

int hdmi_ddc_read(struct hdmi_tx_ddc_ctrl *ddc_ctrl,
	struct hdmi_tx_ddc_data *ddc_data)
{
	int rc = 0;

	if (!ddc_ctrl || !ddc_data) {
		DEV_ERR("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	rc = hdmi_ddc_read_retry(ddc_ctrl, ddc_data);
	if (!rc)
		return rc;

	if (ddc_data->no_align) {
		rc = hdmi_ddc_read_retry(ddc_ctrl, ddc_data);
	} else {
		ddc_data->request_len = 32 * ((ddc_data->data_len + 31) / 32);
		rc = hdmi_ddc_read_retry(ddc_ctrl, ddc_data);
	}

	return rc;
} 

int hdmi_ddc_read_seg(struct hdmi_tx_ddc_ctrl *ddc_ctrl,
	struct hdmi_tx_ddc_data *ddc_data)
{
	int status = 0;
	u32 reg_val, ndx, time_out_count;
	int log_retry_fail;
	int seg_addr = 0x60, seg_num = 0x01;

	if (!ddc_ctrl || !ddc_ctrl->io || !ddc_data) {
		DEV_ERR("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	if (!ddc_data->data_buf) {
		status = -EINVAL;
		DEV_ERR("%s[%s]: invalid buf\n", __func__, ddc_data->what);
		goto error;
	}

	log_retry_fail = ddc_data->retry != 1;

again:
	status = hdmi_ddc_clear_irq(ddc_ctrl, ddc_data->what);
	if (status)
		goto error;

	
	ddc_data->dev_addr &= 0xFE;

	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_DATA, BIT(31) | (seg_addr << 8));

	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_DATA, seg_num << 8);

	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_DATA, ddc_data->dev_addr << 8);
	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_DATA, ddc_data->offset << 8);
	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_DATA,
		(ddc_data->dev_addr | BIT(0)) << 8);

	

	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_TRANS0, BIT(12) | BIT(16));

	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_TRANS1, BIT(12) | BIT(16));

	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_TRANS2,
		BIT(0) | BIT(12) | BIT(13) | (ddc_data->request_len << 16));

	

	INIT_COMPLETION(ddc_ctrl->ddc_sw_done);
	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_CTRL, BIT(0) | BIT(21));

	time_out_count = wait_for_completion_timeout(
		&ddc_ctrl->ddc_sw_done, HZ/2);

	reg_val = DSS_REG_R(ddc_ctrl->io, HDMI_DDC_INT_CTRL);
	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_INT_CTRL, reg_val & (~BIT(2)));
	if (!time_out_count) {
		if (ddc_data->retry-- > 0) {
			DEV_INFO("%s: failed timout, retry=%d\n", __func__,
				ddc_data->retry);
			goto again;
		}
		status = -ETIMEDOUT;
		DEV_ERR("%s: timedout(7), Int Ctrl=%08x\n", __func__,
			DSS_REG_R(ddc_ctrl->io, HDMI_DDC_INT_CTRL));
		DEV_ERR("%s: DDC SW Status=%08x, HW Status=%08x\n",
			__func__,
			DSS_REG_R(ddc_ctrl->io, HDMI_DDC_SW_STATUS),
			DSS_REG_R(ddc_ctrl->io, HDMI_DDC_HW_STATUS));
		goto error;
	}

	
	reg_val = DSS_REG_R(ddc_ctrl->io, HDMI_DDC_SW_STATUS);
	reg_val &= BIT(12) | BIT(13) | BIT(14) | BIT(15);

	
	if (reg_val) {
		
		DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_CTRL, BIT(3));
		if (ddc_data->retry == 1)
			
			DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_CTRL, BIT(1));
		if (ddc_data->retry-- > 0) {
			DEV_DBG("%s(%s): failed NACK=0x%08x, retry=%d\n",
				__func__, ddc_data->what, reg_val,
				ddc_data->retry);
			DEV_DBG("%s: daddr=0x%02x,off=0x%02x,len=%d\n",
				__func__, ddc_data->dev_addr,
				ddc_data->offset, ddc_data->data_len);
			goto again;
		}
		status = -EIO;
		if (log_retry_fail) {
			DEV_ERR("%s(%s): failed NACK=0x%08x\n",
				__func__, ddc_data->what, reg_val);
			DEV_ERR("%s: daddr=0x%02x,off=0x%02x,len=%d\n",
				__func__, ddc_data->dev_addr,
				ddc_data->offset, ddc_data->data_len);
		}
		goto error;
	}

	
	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_DATA,
		BIT(0) | (5 << 16) | BIT(31));

	
	DSS_REG_R_ND(ddc_ctrl->io, HDMI_DDC_DATA);

	for (ndx = 0; ndx < ddc_data->data_len; ++ndx) {
		reg_val = DSS_REG_R_ND(ddc_ctrl->io, HDMI_DDC_DATA);
		ddc_data->data_buf[ndx] = (u8) ((reg_val & 0x0000FF00) >> 8);
	}

	DEV_DBG("%s[%s] success\n", __func__, ddc_data->what);

error:
	return status;
} 

int hdmi_ddc_write(struct hdmi_tx_ddc_ctrl *ddc_ctrl,
	struct hdmi_tx_ddc_data *ddc_data)
{
	u32 reg_val, ndx;
	int status = 0, retry = 10;
	u32 time_out_count;

	if (!ddc_ctrl || !ddc_ctrl->io || !ddc_data) {
		DEV_ERR("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	if (!ddc_data->data_buf) {
		status = -EINVAL;
		DEV_ERR("%s[%s]: invalid buf\n", __func__, ddc_data->what);
		goto error;
	}

again:
	status = hdmi_ddc_clear_irq(ddc_ctrl, ddc_data->what);
	if (status)
		goto error;

	
	ddc_data->dev_addr &= 0xFE;

	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_DATA,
		BIT(31) | (ddc_data->dev_addr << 8));

	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_DATA, ddc_data->offset << 8);

	for (ndx = 0; ndx < ddc_data->data_len; ++ndx)
		DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_DATA,
			((u32)ddc_data->data_buf[ndx]) << 8);

	

	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_TRANS0, BIT(12) | BIT(16));

	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_TRANS1,
		BIT(13) | ((ddc_data->data_len-1) << 16));

	
	INIT_COMPLETION(ddc_ctrl->ddc_sw_done);
	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_CTRL, BIT(0) | BIT(20));

	time_out_count = wait_for_completion_timeout(
		&ddc_ctrl->ddc_sw_done, HZ/2);

	reg_val = DSS_REG_R(ddc_ctrl->io, HDMI_DDC_INT_CTRL);
	DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_INT_CTRL, reg_val & (~BIT(2)));
	if (!time_out_count) {
		if (retry-- > 0) {
			DEV_INFO("%s[%s]: failed timout, retry=%d\n", __func__,
				ddc_data->what, retry);
			goto again;
		}
		status = -ETIMEDOUT;
		DEV_ERR("%s[%s]: timedout, Int Ctrl=%08x\n",
			__func__, ddc_data->what,
			DSS_REG_R(ddc_ctrl->io, HDMI_DDC_INT_CTRL));
		DEV_ERR("%s: DDC SW Status=%08x, HW Status=%08x\n",
			__func__,
			DSS_REG_R(ddc_ctrl->io, HDMI_DDC_SW_STATUS),
			DSS_REG_R(ddc_ctrl->io, HDMI_DDC_HW_STATUS));
		goto error;
	}

	
	reg_val = DSS_REG_R_ND(ddc_ctrl->io, HDMI_DDC_SW_STATUS);
	reg_val &= 0x00001000 | 0x00002000 | 0x00004000 | 0x00008000;

	
	if (reg_val) {
		if (retry > 1)
			
			DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_CTRL, BIT(3));
		else
			
			DSS_REG_W_ND(ddc_ctrl->io, HDMI_DDC_CTRL, BIT(1));

		if (retry-- > 0) {
			DEV_DBG("%s[%s]: failed NACK=%08x, retry=%d\n",
				__func__, ddc_data->what, reg_val, retry);
			msleep(100);
			goto again;
		}
		status = -EIO;
		DEV_ERR("%s[%s]: failed NACK: %08x\n", __func__,
			ddc_data->what, reg_val);
		goto error;
	}

	DEV_DBG("%s[%s] success\n", __func__, ddc_data->what);

error:
	return status;
} 
