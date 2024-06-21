/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2021-2022, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2022, MM Solutions EAD. All rights reserved.
 *	Author: Atanas Filipov <afilipov@mm-sol.com>
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

#ifndef CAM_JENC_V4L2_HAL_H_
#define CAM_JENC_V4L2_HAL_H_

#include <linux/types.h>
#include <linux/videodev2.h>
#include <media/videobuf2-v4l2.h>
#include <media/v4l2-fh.h>

#include <media/v4l2-ctrls.h>
#include <media/cam_jpeg.h>

#include "cam_hw.h"
#include "cam_hw_intf.h"
#include "cam_context.h"
#include "cam_context_utils.h"
#include "cam_packet_util.h"

#include "cam_jpeg_dev.h"
#include "cam_jenc_v4l2_defs.h"
#include "cam_jenc_v4l2_common.h"

#define JENC_PAYLOAD_DIV 3 // Chrome: Maximum payload of jpeg buffer
#define JENC_MAX_DMI_INDEX 64

struct jenc_bufq {
	struct v4l2_format vf;
	u32 sequence;
	struct {
		dma_addr_t iova;
		size_t length;
		u8 reg_id;
	} mapped[VB2_MAX_FRAME][CAM_PACKET_MAX_PLANES];
};

struct jenc_context {
	struct device *dev;
	struct jpeg_encoder *jenc;
	struct v4l2_fh fh;
	struct mutex vb_mutex;
	spinlock_t irqlock;
	struct jenc_bufq bufq[JENC_MAX_ID];
	struct v4l2_ctrl_handler ctrl_hdl;
	struct v4l2_ctrl *quality_ctl;
	struct mutex quality_mutex;
	u32 quality_requested;
	u32 quality_programmed;
	struct v4l2_selection sel;
	u32 rmap[JENC_S3_MMU_PF_CTL_L1_FILTER_REG];
	s32 iommu_hdl;
	size_t result_size;
	u8 dqt_table1[JENC_MAX_DMI_INDEX];
	u8 dqt_table2[JENC_MAX_DMI_INDEX];
};

void jenc_hal_setup_fe_engine(struct jenc_context *jctx, struct jenc_bufq *bq);

void jenc_hal_setup_we_engine(struct jenc_context *jctx, struct jenc_bufq *bq);

struct jenc_bufq *jenc_hal_get_bufq(struct jenc_context *jctx,
				    enum jenc_vid_id id);

int jenc_hal_dma_buffer_release(struct jenc_context *jctx, struct vb2_buffer *vb2);

int jenc_hal_dma_buffer_acquire(struct jenc_context *jctx, struct vb2_buffer *vb2, int plane);

int jenc_hal_process_exec(struct jenc_context *jctx,
			  struct vb2_v4l2_buffer *src_vb,
			  struct vb2_v4l2_buffer *dst_vb);

void jenc_hal_process_done(struct jenc_context *jctx, bool status);

int jenc_hal_prepare(struct jenc_context *jctx, bool hw_reset);

void jenc_hal_release(struct jenc_context *jctx);

#endif /* CAM_JENC_V4L2_HAL_H_*/
