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

#ifndef CAM_JENC_V4L2_COMMON_H_
#define CAM_JENC_V4L2_COMMON_H_

#include <linux/types.h>
#include <linux/videodev2.h>

/* Maximum number of planes supported by HW */
#define JENC_HW_MAX_PLANES	3

/* Offline Encode limist */
#define JENC_HW_MAX_WIDTH	5376
#define JENC_HW_MAX_HEIGHT	4032

#define JENC_HW_DST_DEF_WIDTH	640
#define JENC_HW_DST_DEF_HEIGHT	480
#define JENC_HW_DST_WALIGN	128
#define JENC_HW_DST_HALIGN	2

#define JENC_HW_SRC_DEF_WIDTH	2592
#define JENC_HW_SRC_DEF_HEIGHT	1944
#define JENC_HW_SRC_WALIGN	16
#define JENC_HW_SRC_HALIGN	2

#define JENC_HW_MIN_WIDTH	96
#define JENC_HW_MIN_HEIGHT	72

enum jenc_vid_id {
	JENC_SRC_ID = 0,
	JENC_DST_ID,
	JENC_MAX_ID
};

#define TYPE2ID(t) (V4L2_TYPE_IS_OUTPUT(t) ? JENC_SRC_ID : JENC_DST_ID)

#define JENC_IS_SRC(t) ((t) == JENC_SRC_ID)
#define JENC_IS_DST(t) ((t) == JENC_DST_ID)

struct jenc_context;

int jenc_v4l2_enum_fmt_src(struct v4l2_fmtdesc *f);

int jenc_v4l2_enum_fmt_dst(struct v4l2_fmtdesc *f);

int jenc_v4l2_try_format(struct jenc_context *jctx, struct v4l2_format *f);

int jenc_v4l2_set_format(struct jenc_context *jctx, struct v4l2_format *f);

int jenc_v4l2_get_format(struct jenc_context *jctx, struct v4l2_format *f);

int jenc_v4l2_enum_framesizes(struct file *file, void *priv,
			      struct v4l2_frmsizeenum *fsize);

u32 jenc_v4l2_set_sequence(struct jenc_context *jctx, enum v4l2_buf_type type,
			   u32 sequence);

void jenc_v4l2_set_defaults(struct jenc_context *jctx);

#endif /* CAM_JENC_V4L2_COMMON_H_*/
