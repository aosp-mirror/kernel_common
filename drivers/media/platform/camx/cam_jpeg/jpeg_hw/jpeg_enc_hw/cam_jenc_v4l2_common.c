// SPDX-License-Identifier: GPL-2.0-only
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

#include <media/v4l2-common.h>
#include <media/videobuf2-v4l2.h>
#include <media/v4l2-mem2mem.h>

#include "cam_jenc_v4l2_hal.h"
#include "cam_jenc_v4l2_common.h"

struct jenc_v4l2_format {
	u32 type;
	u32 fourcc;
	u8  deept[JENC_HW_MAX_PLANES];
	u8  x_div[JENC_HW_MAX_PLANES];
	u8  y_div[JENC_HW_MAX_PLANES];
	enum jenc_vid_id id;
};

static const struct jenc_v4l2_format jenc_src_formats[] = {
	{
		.fourcc	= V4L2_PIX_FMT_NV12,
		.type	= V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.id	= JENC_SRC_ID,
		.deept	= { 8, 8 },
		.x_div	= { 1, 1 },
		.y_div	= { 1, 2 }
	}, {
		.fourcc	= V4L2_PIX_FMT_NV21,
		.type	= V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.id	= JENC_SRC_ID,
		.deept	= { 8, 8 },
		.x_div	= { 1, 1 },
		.y_div	= { 1, 2 }
	}
};

#define JENC_NUM_OF_SRC_FORMATS ARRAY_SIZE(jenc_src_formats)

static const struct jenc_v4l2_format jenc_dst_formats[] = {
	{
		.fourcc	= V4L2_PIX_FMT_JPEG,
		.type	= V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.id	= JENC_DST_ID,
		.deept	= { 24 },
		.x_div	= { 1 },
		.y_div	= { 2 }	/* Compression is ~50% in worst case (Quality == 100) */
	}
};

#define JENC_NUM_OF_DST_FORMATS ARRAY_SIZE(jenc_dst_formats)

static const struct v4l2_frmsizeenum jenc_fsizes = {
	.stepwise = {
		.min_width	= JENC_HW_MIN_WIDTH,
		.max_width	= JENC_HW_MAX_WIDTH,
		.step_width	= JENC_HW_SRC_WALIGN,
		.min_height	= JENC_HW_MIN_HEIGHT,
		.max_height	= JENC_HW_MAX_HEIGHT,
		.step_height	= JENC_HW_SRC_HALIGN,
	},
	.type = V4L2_FRMSIZE_TYPE_STEPWISE
};

static const struct jenc_v4l2_format *jenc_find_pix_format(enum jenc_vid_id id,
							   u32 fourcc)
{
	const struct jenc_v4l2_format *jf;
	unsigned int i, count;

	if (JENC_IS_SRC(id)) {
		count = JENC_NUM_OF_SRC_FORMATS;
		jf = jenc_src_formats;
	} else {
		count = JENC_NUM_OF_DST_FORMATS;
		jf = jenc_dst_formats;
	}

	for (i = 0; i < count; i++) {
		if (jf[i].fourcc == fourcc)
			return &jf[i];
	}

	return NULL;
}

static  size_t jenc_count_planes(struct jenc_context *jctx, enum jenc_vid_id id)
{
	return JENC_IS_SRC(id) ? 2 : 1;
}

static void jenc_update_planes(struct jenc_context *jctx,
			       const struct jenc_v4l2_format *jf,
			       struct v4l2_pix_format_mplane *pix_mp,
			       u32 width, u32 height)
{
	struct v4l2_plane_pix_format *pfmt = pix_mp->plane_fmt;
	int i;

	memset(pfmt, 0, sizeof(struct v4l2_plane_pix_format));

	for (i = 0; i < pix_mp->num_planes; i++) {
		pfmt[i].bytesperline = width * jf->deept[i] / jf->x_div[i] / BITS_PER_BYTE;
		pfmt[i].sizeimage = pfmt[i].bytesperline * height / jf->y_div[i];
	}

	if (jf->fourcc == V4L2_PIX_FMT_JPEG)
	   pfmt[i].sizeimage = PAGE_ALIGN(pfmt[i].sizeimage);
}

int jenc_v4l2_enum_fmt_src(struct v4l2_fmtdesc *f)
{
	if (f->index >= JENC_NUM_OF_SRC_FORMATS)
		return -EINVAL;

	f->pixelformat = jenc_src_formats[f->index].fourcc;

	return 0;
}

int jenc_v4l2_enum_fmt_dst(struct v4l2_fmtdesc *f)
{
	if (f->index >= JENC_NUM_OF_DST_FORMATS)
		return -EINVAL;

	f->pixelformat = jenc_dst_formats[f->index].fourcc;

	return 0;
}

int jenc_v4l2_enum_framesizes(struct file *file, void *priv,
			      struct v4l2_frmsizeenum *fsize)
{
	const struct jenc_v4l2_format *jf;
	int i;

	if (fsize->index != 0)
		return -EINVAL;

	for (i = JENC_SRC_ID; i < JENC_MAX_ID; i++) {
		jf = jenc_find_pix_format(i, fsize->pixel_format);
		if (jf)
			break;
	}

	if (!jf)
		return -EINVAL;

	fsize->type = jenc_fsizes.type;
	fsize->stepwise = jenc_fsizes.stepwise;

	return 0;
}

int jenc_v4l2_try_format(struct jenc_context *jctx, struct v4l2_format *f)
{
	const struct jenc_v4l2_format *jf;
	struct v4l2_pix_format_mplane *mp = &f->fmt.pix_mp;
	int i;

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		jf = jenc_find_pix_format(JENC_SRC_ID,
					  f->fmt.pix_mp.pixelformat);
		if (!jf)
			jf = jenc_src_formats;

		f->fmt.pix_mp.num_planes = jenc_count_planes(jctx, JENC_SRC_ID);
	} else {
		jf = jenc_find_pix_format(JENC_DST_ID,
					  f->fmt.pix_mp.pixelformat);
		if (!jf)
			jf = jenc_dst_formats;
		f->fmt.pix_mp.num_planes = jenc_count_planes(jctx, JENC_DST_ID);
	}

	f->fmt.pix_mp.field = V4L2_FIELD_NONE;

	if (!v4l2_is_colorspace_valid(mp->colorspace))
		mp->colorspace = V4L2_COLORSPACE_DEFAULT;

	if (!v4l2_is_xfer_func_valid(mp->xfer_func))
		mp->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(mp->colorspace);

	if (!v4l2_is_ycbcr_enc_valid(mp->ycbcr_enc))
		mp->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(mp->colorspace);

	if (!v4l2_is_quant_valid(mp->quantization))
		mp->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(false,
								 mp->colorspace,
								 mp->ycbcr_enc);

	v4l2_apply_frmsize_constraints(&f->fmt.pix_mp.width, &f->fmt.pix_mp.height,
				       &jenc_fsizes.stepwise);

	for (i = 0; i < f->fmt.pix_mp.num_planes; i++)
		f->fmt.pix_mp.pixelformat = jf->fourcc;

	jenc_update_planes(jctx, jf, &f->fmt.pix_mp, f->fmt.pix_mp.width,
			   f->fmt.pix_mp.height);

	return 0;
}

int jenc_v4l2_set_format(struct jenc_context *jctx, struct v4l2_format *f)
{
	struct jenc_bufq *bq = jenc_hal_get_bufq(jctx, TYPE2ID(f->type));
	struct jenc_bufq *dq = jenc_hal_get_bufq(jctx, JENC_DST_ID);
	struct vb2_queue *vq;
	int i;

	vq = v4l2_m2m_get_vq(jctx->fh.m2m_ctx, f->type);
	if (!vq) {
		dev_err(jctx->dev, "Cannot get video queue\n");
		return -EINVAL;
	}

	if (vb2_is_busy(vq)) {
		dev_err(jctx->dev, "The queue is busy\n");
		return -EBUSY;
	}

	jenc_v4l2_try_format(jctx, f);

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		dq->vf.fmt.pix_mp.colorspace   = f->fmt.pix_mp.colorspace;
		dq->vf.fmt.pix_mp.ycbcr_enc    = f->fmt.pix_mp.ycbcr_enc;
		dq->vf.fmt.pix_mp.quantization = f->fmt.pix_mp.quantization;
		dq->vf.fmt.pix_mp.xfer_func    = f->fmt.pix_mp.xfer_func;

		jctx->sel.r.left   = 0;
		jctx->sel.r.top	   = 0;
		jctx->sel.r.width  = f->fmt.pix_mp.width;
		jctx->sel.r.height = f->fmt.pix_mp.height;
	}

	for (i = 0; i < f->fmt.pix_mp.num_planes; i++) {
		dev_dbg(jctx->dev, "type:%d %dx%d[%d] %c%c%c%c bpl:%d sz:%d\n",
			f->type, f->fmt.pix_mp.width, f->fmt.pix_mp.height, i,
			(f->fmt.pix_mp.pixelformat >>  0) & 0xff,
			(f->fmt.pix_mp.pixelformat >>  8) & 0xff,
			(f->fmt.pix_mp.pixelformat >> 16) & 0xff,
			(f->fmt.pix_mp.pixelformat >> 24) & 0xff,
			f->fmt.pix_mp.plane_fmt[i].bytesperline,
			f->fmt.pix_mp.plane_fmt[i].sizeimage);
	}

	bq->vf = *f;

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		bq->vf.fmt.pix_mp.width = dq->vf.fmt.pix_mp.width;
		bq->vf.fmt.pix_mp.height = dq->vf.fmt.pix_mp.height;
	}

	return 0;
}

int jenc_v4l2_get_format(struct jenc_context *jctx, struct v4l2_format *f)
{
	struct jenc_bufq *bq = jenc_hal_get_bufq(jctx, TYPE2ID(f->type));
	struct jenc_bufq *dq = jenc_hal_get_bufq(jctx, JENC_DST_ID);

	*f = bq->vf;

	if (V4L2_TYPE_IS_CAPTURE(f->type)) {
		f->fmt.pix_mp.colorspace   = dq->vf.fmt.pix_mp.colorspace;
		f->fmt.pix_mp.ycbcr_enc    = dq->vf.fmt.pix_mp.ycbcr_enc;
		f->fmt.pix_mp.quantization = dq->vf.fmt.pix_mp.quantization;
		f->fmt.pix_mp.xfer_func    = dq->vf.fmt.pix_mp.xfer_func;
	}

	return 0;
}

u32 jenc_v4l2_set_sequence(struct jenc_context *jctx, enum v4l2_buf_type type,
			   u32 sequence)
{
	struct jenc_bufq *bq = jenc_hal_get_bufq(jctx, TYPE2ID(type));

	if (!jctx)
		return -EINVAL;

	return bq->sequence = sequence;
}

void jenc_v4l2_set_defaults(struct jenc_context *jctx)
{
	struct jenc_bufq *s = jenc_hal_get_bufq(jctx, JENC_SRC_ID);
	struct jenc_bufq *d = jenc_hal_get_bufq(jctx, JENC_DST_ID);
	struct v4l2_format f = {0};

	f.type			 = jenc_src_formats->type;
	f.fmt.pix_mp.pixelformat = jenc_src_formats->fourcc;
	jenc_v4l2_try_format(jctx, &f);	s->vf = f;

	f.type			 = jenc_dst_formats->type;
	f.fmt.pix_mp.pixelformat = jenc_dst_formats->fourcc;
	jenc_v4l2_try_format(jctx, &f);	d->vf = f;
}
