/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
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

#ifndef _CAM_JPEG_DEV_H_
#define _CAM_JPEG_DEV_H_

#include <media/v4l2-device.h>

#include "cam_context.h"
#include "cam_hw.h"
#include "cam_hw_intf.h"
#include "cam_hw_mgr_intf.h"
#include "cam_jpeg_context.h"
#include "cam_subdev.h"
#include "jpeg_enc_dev.h"

#define JENC_QUALITY_MIN	1
#define JENC_QUALITY_DEF	98
#define JENC_QUALITY_MAX	100
#define JENC_QUALITY_MID	(JENC_QUALITY_MAX / 2)
#define JENC_QUALITY_STEP	1

/**
 * struct cam_jpeg_dev - Camera JPEG V4l2 device node
 *
 * @sd: Commone camera subdevice node
 * @node: Pointer to jpeg subdevice
 * @ctx: JPEG base context storage
 * @ctx_jpeg: JPEG private context storage
 * @jpeg_mutex: Jpeg dev mutex
 * @open_cnt: Open device count
 */
struct cam_jpeg_dev {
	struct cam_subdev sd;
	struct cam_node *node;
	struct cam_context ctx[CAM_CTX_MAX];
	struct cam_jpeg_context ctx_jpeg[CAM_CTX_MAX];
	struct mutex jpeg_mutex;
	int32_t open_cnt;
};

struct jpeg_encoder {
	u32		   hw_type;
	u32		   hw_idx;
	struct cam_hw_ops  hw_ops;
	void		  *hw_priv;
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct video_device vfd;
#ifdef CONFIG_MEDIA_CONTROLLER
	struct media_device mdev;
#endif
	struct mutex dev_mutex;
	struct v4l2_m2m_dev *m2m_dev;
	atomic_t ref_count;
	struct device *smmu;
	s32 iommu_hdl;
};

int jenc_v4l2_device_create(struct jpeg_encoder *jenc);

void jenc_v4l2_device_destroy(struct jpeg_encoder *jenc);

#endif /* __CAM_JPEG_DEV_H__ */
