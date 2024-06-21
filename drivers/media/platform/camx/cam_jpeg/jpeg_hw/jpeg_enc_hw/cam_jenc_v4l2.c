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

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-sg.h>

#include "jpeg_enc_core.h"
#include "cam_jpeg_dev.h"
#include "cam_jenc_v4l2_hal.h"
#include "cam_jenc_v4l2_common.h"

#include "cam_jpeg_hw_mgr.h"

#define JENC_MEDIA_ENT		MEDIA_ENT_F_PROC_VIDEO_ENCODER
#define JENC_DEF_TIMER_MS	5

#define JENC_MIN_NUM_BUFS	1

static inline struct jenc_context *jenc_file2ctx(struct file *file)
{
	return container_of(file->private_data, struct jenc_context, fh);
}

static int op_jenc_v4l2_set_ctrls(struct v4l2_ctrl *ctrl)
{
	struct jenc_context *jctx = container_of(ctrl->handler,
						 struct jenc_context, ctrl_hdl);

	switch (ctrl->id) {
	case V4L2_CID_JPEG_COMPRESSION_QUALITY:
		mutex_lock(&jctx->quality_mutex);
		jctx->quality_requested = ctrl->val;
		mutex_unlock(&jctx->quality_mutex);
		break;
	default:
		dev_err(jctx->dev, "%s Ivalid control:0x%x\n", __func__,
			ctrl->id);
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops jenc_v4l2_ctrl_ops = {
	.s_ctrl = op_jenc_v4l2_set_ctrls,
};

static int op_jenc_vb2_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
				   unsigned int *plns_per_buff,
				   unsigned int sizes[],
				   struct device *alloc_devs[])
{
	struct jenc_context *jctx = vb2_get_drv_priv(vq);
	struct v4l2_pix_format_mplane *pixfmt;
	struct jenc_bufq *bufq;
	int i, rc = 0;

	if (!jctx)
		return -EIO;

	bufq = jenc_hal_get_bufq(jctx, TYPE2ID(vq->type));
	pixfmt = &bufq->vf.fmt.pix_mp;

	if (*plns_per_buff) {
		if (*plns_per_buff != pixfmt->num_planes)
			return -EINVAL;
		for (i = 0; i < pixfmt->num_planes; ++i)
			if (sizes[i] < pixfmt->plane_fmt[i].sizeimage)
				return -EINVAL;
		return 0;
	}

	*plns_per_buff = pixfmt->num_planes;
	for (i = 0; i < pixfmt->num_planes; ++i)
		sizes[i] = pixfmt->plane_fmt[i].sizeimage;

	vq->min_buffers_needed = JENC_MIN_NUM_BUFS;

	return rc;
}

static int op_jenc_vb2_buf_out_validate(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (vbuf->field == V4L2_FIELD_ANY)
		vbuf->field = V4L2_FIELD_NONE;
	if (vbuf->field != V4L2_FIELD_NONE)
		return -EINVAL;

	return 0;
}

static int op_jenc_vb2_buf_prepare(struct vb2_buffer *vb)
{
	struct jenc_context *jctx = vb2_get_drv_priv(vb->vb2_queue);
	struct jenc_bufq *bufq;
	struct v4l2_pix_format_mplane *pix_mp;
	int i, rc = 0;

	if (!jctx)
		return -EIO;

	bufq = jenc_hal_get_bufq(jctx, TYPE2ID(vb->type));
	pix_mp = &bufq->vf.fmt.pix_mp;

	for (i = 0; i < pix_mp->num_planes; i++) {
		rc = jenc_hal_dma_buffer_acquire(jctx, vb, i);
		if (rc)
			break;
	}

	return rc;
}

static void op_jenc_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct jenc_context *jctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (!jctx)
		return;

	v4l2_m2m_buf_queue(jctx->fh.m2m_ctx, vbuf);
}

static int op_jenc_vb2_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct jenc_context *jctx = vb2_get_drv_priv(q);

	if (!jctx)
		return -EINVAL;

	jenc_v4l2_set_sequence(jctx, q->type, 0);

	return 0;
}

static void op_jenc_vb2_stop_streaming(struct vb2_queue *q)
{
	struct jenc_context *jctx = vb2_get_drv_priv(q);

	if (!jctx)
		return;

	jenc_hal_process_done(jctx, true);
}

static void op_jenc_vb2_buf_finish(struct vb2_buffer *vb)
{
	struct jenc_context *jctx = vb2_get_drv_priv(vb->vb2_queue);

	if (!jctx)
		return;

	jenc_hal_dma_buffer_release(jctx, vb);

	dev_dbg(jctx->dev, "Result size:%zd\n", jctx->result_size);
}

static const struct vb2_ops jenc_v4l2_vb2_ops = {
	.queue_setup		= op_jenc_vb2_queue_setup,
	.buf_out_validate	= op_jenc_vb2_buf_out_validate,
	.buf_prepare		= op_jenc_vb2_buf_prepare,
	.buf_queue		= op_jenc_vb2_buf_queue,
	.buf_finish		= op_jenc_vb2_buf_finish,
	.start_streaming	= op_jenc_vb2_start_streaming,
	.stop_streaming		= op_jenc_vb2_stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static void op_jenc_m2m_job_abort(void *priv)
{
	struct jenc_context *jctx = priv;

	if (!jctx)
		return;

	jenc_hal_process_done(jctx, true);

	v4l2_m2m_job_finish(jctx->jenc->m2m_dev, jctx->fh.m2m_ctx);
}

static void op_jenc_m2m_job_run(void *priv)
{
	struct jenc_context *jctx = priv;
	struct vb2_v4l2_buffer *src_vb, *dst_vb;
	struct jenc_bufq *sdsc, *ddsc;

	if (!jctx)
		return;

	src_vb = v4l2_m2m_next_src_buf(jctx->fh.m2m_ctx);
	dst_vb = v4l2_m2m_next_dst_buf(jctx->fh.m2m_ctx);

	if (!src_vb && !dst_vb)
		return;

	sdsc = jenc_hal_get_bufq(jctx, TYPE2ID(src_vb->vb2_buf.type));
	ddsc = jenc_hal_get_bufq(jctx, TYPE2ID(dst_vb->vb2_buf.type));

	src_vb->sequence = sdsc->sequence++;
	dst_vb->sequence = ddsc->sequence++;
	v4l2_m2m_buf_copy_metadata(src_vb, dst_vb, true);

	if (jenc_hal_process_exec(jctx, src_vb, dst_vb))
		op_jenc_m2m_job_abort(jctx);
}

static const struct v4l2_m2m_ops jenc_v4l2_m2m_ops = {
	.device_run	= op_jenc_m2m_job_run,
	.job_abort	= op_jenc_m2m_job_abort,
};

static int op_jenc_querycap(struct file *file, void *priv,
			    struct v4l2_capability *cap)
{
	strscpy(cap->driver, CAM_JPEC_ENC_NAME, sizeof(cap->driver));
	strscpy(cap->card, CAM_JPEC_ENC_NAME, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 CAM_JPEC_ENC_NAME);

	cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static bool is_invalid_src(struct jenc_context *jctx, u32 type)
{
	bool is_invalid = (V4L2_TYPE_IS_CAPTURE(type) ||
			   !V4L2_TYPE_IS_MULTIPLANAR(type));
	if (is_invalid)
		dev_err(jctx->dev, "Invalid src type or format\n");

	return is_invalid;
}

static bool is_invalid_dst(struct jenc_context *jctx, u32 type)
{
	bool is_invalid = (V4L2_TYPE_IS_OUTPUT(type) &&
			   !V4L2_TYPE_IS_MULTIPLANAR(type));

	if (is_invalid)
		dev_err(jctx->dev, "Invalid dst type or format\n");

	return is_invalid;
}

static int op_jenc_enum_fmt_vid_dst(struct file *file, void *priv,
				    struct v4l2_fmtdesc *f)
{
	struct jenc_context *jctx = jenc_file2ctx(file);

	if (!jctx)
		return -EIO;

	if (is_invalid_dst(jctx, f->type))
		return -EINVAL;

	return jenc_v4l2_enum_fmt_dst(f);
}

static int op_jenc_enum_fmt_vid_src(struct file *file, void *priv,
				    struct v4l2_fmtdesc *f)
{
	struct jenc_context *jctx = jenc_file2ctx(file);

	if (!jctx)
		return -EIO;

	if (is_invalid_src(jctx, f->type))
		return -EINVAL;

	return jenc_v4l2_enum_fmt_src(f);
}

static int op_jenc_enum_framesizes(struct file *file, void *priv,
				   struct v4l2_frmsizeenum *fsize)
{
	return jenc_v4l2_enum_framesizes(file, priv, fsize);
}

static int op_jenc_try_fmt_vid_dst(struct file *file, void *priv,
				   struct v4l2_format *f)
{
	struct jenc_context *jctx = jenc_file2ctx(file);

	if (!jctx)
		return -EIO;

	if (is_invalid_dst(jctx, f->type))
		return -EINVAL;

	return jenc_v4l2_try_format(jctx, f);
}

static int op_jenc_try_fmt_vid_src(struct file *file, void *priv,
				   struct v4l2_format *f)
{
	struct jenc_context *jctx = jenc_file2ctx(file);

	if (!jctx)
		return -EIO;

	if (is_invalid_src(jctx, f->type))
		return -EINVAL;

	return jenc_v4l2_try_format(jctx, f);
}

static int op_jenc_get_fmt_vid_dst(struct file *file, void *priv,
				   struct v4l2_format *f)
{
	struct jenc_context *jctx = jenc_file2ctx(file);

	if (!jctx)
		return -EIO;

	if (is_invalid_dst(jctx, f->type))
		return -EINVAL;

	return jenc_v4l2_get_format(jctx, f);
}

static int op_jenc_get_fmt_vid_src(struct file *file, void *priv,
				   struct v4l2_format *f)
{
	struct jenc_context *jctx = jenc_file2ctx(file);

	if (!jctx)
		return -EIO;

	if (is_invalid_src(jctx, f->type))
		return -EINVAL;

	return jenc_v4l2_get_format(jctx, f);
}

static int op_jenc_set_fmt_vid_dst(struct file *file, void *priv,
				   struct v4l2_format *f)
{
	struct jenc_context *jctx = jenc_file2ctx(file);

	if (!jctx)
		return -EIO;

	if (is_invalid_dst(jctx, f->type))
		return -EINVAL;

	return jenc_v4l2_set_format(jctx, f);
}

static int op_jenc_set_fmt_vid_src(struct file *file, void *priv,
				   struct v4l2_format *f)
{
	struct jenc_context *jctx = jenc_file2ctx(file);

	if (!jctx)
		return -EIO;

	if (is_invalid_src(jctx, f->type))
		return -EINVAL;

	return jenc_v4l2_set_format(jctx, f);
}

static int op_jenc_query_ext_ctrl(struct file *file, void *fh,
				  struct v4l2_query_ext_ctrl *xctrl)
{
	return 0;
}

static int op_jenc_g_selection(struct file *file, void *fh,
			       struct v4l2_selection *s)
{
	struct jenc_context *jctx = jenc_file2ctx(file);
	struct jenc_bufq *bq = jenc_hal_get_bufq(jctx, TYPE2ID(s->type));

	if (V4L2_TYPE_IS_CAPTURE(s->type))
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		s->r.left	= 0;
		s->r.top	= 0;
		s->r.width	= bq->vf.fmt.pix_mp.width;
		s->r.height	= bq->vf.fmt.pix_mp.height;
		break;
	case V4L2_SEL_TGT_CROP:
		*s = jctx->sel;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int op_jenc_s_selection(struct file *file, void *fh,
			       struct v4l2_selection *s)
{
	struct jenc_context *jctx = jenc_file2ctx(file);
	struct jenc_bufq *dbq = jenc_hal_get_bufq(jctx, TYPE2ID(s->type));

	if (V4L2_TYPE_IS_CAPTURE(s->type))
		return -EINVAL;

	s->r.width  = clamp_t(u32, s->r.width, 0, dbq->vf.fmt.pix_mp.width);
	s->r.height = clamp_t(u32, s->r.height, 0, dbq->vf.fmt.pix_mp.height);
	s->r.left   = clamp_t(u32, s->r.left, 0, JENC_HW_MAX_WIDTH - s->r.width);
	s->r.top    = clamp_t(u32, s->r.top, 0, JENC_HW_MAX_HEIGHT - s->r.height);

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		jctx->sel = *s;
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(jctx->dev, "Image:%dx%d Crop: L:%d W:%d T:%d H:%d\n",
		dbq->vf.fmt.pix_mp.width, dbq->vf.fmt.pix_mp.height,
		s->r.left, s->r.width, s->r.top, s->r.height);

	return 0;
}

static const struct v4l2_ioctl_ops jenc_v4l2_ioctl_ops = {
	.vidioc_querycap		= op_jenc_querycap,
	.vidioc_enum_fmt_vid_cap	= op_jenc_enum_fmt_vid_dst,
	.vidioc_enum_framesizes		= op_jenc_enum_framesizes,
	.vidioc_enum_fmt_vid_out	= op_jenc_enum_fmt_vid_src,

	.vidioc_g_fmt_vid_cap_mplane	= op_jenc_get_fmt_vid_dst,
	.vidioc_try_fmt_vid_cap_mplane	= op_jenc_try_fmt_vid_dst,
	.vidioc_s_fmt_vid_cap_mplane	= op_jenc_set_fmt_vid_dst,
	.vidioc_g_fmt_vid_out_mplane	= op_jenc_get_fmt_vid_src,
	.vidioc_try_fmt_vid_out_mplane	= op_jenc_try_fmt_vid_src,
	.vidioc_s_fmt_vid_out_mplane	= op_jenc_set_fmt_vid_src,

	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,

	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
	.vidioc_query_ext_ctrl		= op_jenc_query_ext_ctrl,

	.vidioc_g_selection		= op_jenc_g_selection,
	.vidioc_s_selection		= op_jenc_s_selection,
};

static int jenc_v4l2_init_queue(void *priv, struct vb2_queue *src_vq,
				struct vb2_queue *dst_vq)
{
	struct jenc_context *jctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = jctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &jenc_v4l2_vb2_ops;
	src_vq->dev = jctx->jenc->smmu;
	src_vq->mem_ops = &vb2_dma_sg_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &jctx->vb_mutex;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	*dst_vq  = *src_vq;
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

	return vb2_queue_init(dst_vq);
}

/*
 * File operations
 */
static int op_jenc_file_open(struct file *file)
{
	struct jpeg_encoder *jenc = video_drvdata(file);
	struct jenc_context *jctx;
	bool hw_reset = false;
	int rc = 0;

	if (!jenc)
		return -EIO;

	jctx = kzalloc(sizeof(*jctx), GFP_KERNEL);
	if (!jctx)
		return -ENOMEM;

	jctx->jenc	= jenc;
	jctx->dev	= jenc->dev;
	jctx->iommu_hdl	= cam_jpeg_get_iommu_hd();
	mutex_init(&jctx->vb_mutex);
	mutex_init(&jctx->quality_mutex);
	spin_lock_init(&jctx->irqlock);

	if (atomic_inc_return(&jenc->ref_count) == 1)
		hw_reset = true;

	rc = jenc_hal_prepare(jctx, hw_reset);
	if (rc)
		goto err_jctx_free;

	v4l2_fh_init(&jctx->fh, video_devdata(file));
	file->private_data = &jctx->fh;

	v4l2_ctrl_handler_init(&jctx->ctrl_hdl, 1);
	jctx->quality_ctl = v4l2_ctrl_new_std(&jctx->ctrl_hdl,
					      &jenc_v4l2_ctrl_ops,
					      V4L2_CID_JPEG_COMPRESSION_QUALITY,
					      JENC_QUALITY_MIN,
					      JENC_QUALITY_MAX,
					      JENC_QUALITY_STEP,
					      JENC_QUALITY_DEF);
	if (jctx->ctrl_hdl.error) {
		rc = jctx->ctrl_hdl.error;
		goto err_fh_exit;
	}

	jctx->fh.ctrl_handler = &jctx->ctrl_hdl;
	rc = v4l2_ctrl_handler_setup(&jctx->ctrl_hdl);
	if (rc)
		goto err_fh_exit;

	jctx->fh.m2m_ctx = v4l2_m2m_ctx_init(jenc->m2m_dev, jctx,
					     &jenc_v4l2_init_queue);
	if (IS_ERR(jctx->fh.m2m_ctx)) {
		rc = PTR_ERR(jctx->fh.m2m_ctx);
		goto err_handler_free;
	}

	v4l2_fh_add(&jctx->fh);

	/* Initialize the parameters by default */
	jenc_v4l2_set_defaults(jctx);

	dev_dbg(jctx->dev, "Acquiring instance %p\n", jctx);

	return rc;

err_handler_free:
	v4l2_ctrl_handler_free(&jctx->ctrl_hdl);
err_fh_exit:
	v4l2_fh_exit(&jctx->fh);
err_jctx_free:
	atomic_dec_return(&jenc->ref_count);
	kfree(jctx);

	return rc;
}

static int op_jenc_file_release(struct file *file)
{
	struct jpeg_encoder *jenc = video_drvdata(file);
	struct jenc_context *jctx = jenc_file2ctx(file);

	if (!atomic_dec_if_positive(&jenc->ref_count))
		jenc_hal_release(jctx);

	v4l2_fh_del(&jctx->fh);
	v4l2_fh_exit(&jctx->fh);
	v4l2_ctrl_handler_free(&jctx->ctrl_hdl);
	mutex_lock(&jenc->dev_mutex);
	v4l2_m2m_ctx_release(jctx->fh.m2m_ctx);
	mutex_unlock(&jenc->dev_mutex);

	dev_dbg(jctx->dev, "Releasing instance %p\n", jctx);

	kfree(jctx);

	return 0;
}

static const struct v4l2_file_operations jenc_v4l2_file_ops = {
	.owner		= THIS_MODULE,
	.open		= op_jenc_file_open,
	.release	= op_jenc_file_release,

	.poll		= v4l2_m2m_fop_poll,
	.mmap		= v4l2_m2m_fop_mmap,

	.unlocked_ioctl = video_ioctl2,
};

static void op_jenc_vdev_release(struct video_device *vdev)
{
	struct jpeg_encoder *jenc = container_of(vdev, struct jpeg_encoder,
						 vfd);

	v4l2_device_unregister(&jenc->v4l2_dev);
	v4l2_m2m_release(jenc->m2m_dev);
#ifdef CONFIG_MEDIA_CONTROLLER
	media_device_cleanup(&jenc->mdev);
#endif
	kfree(jenc);
}

static const struct video_device jenc_v4l2_vdev_ops = {
	.name		= CAM_JPEC_ENC_NAME,
	.vfl_dir	= VFL_DIR_M2M,
	.fops		= &jenc_v4l2_file_ops,
	.ioctl_ops	= &jenc_v4l2_ioctl_ops,
	.minor		= -1,
	.release	= op_jenc_vdev_release,
	.device_caps	= V4L2_CAP_STREAMING		|
			  V4L2_CAP_VIDEO_M2M_MPLANE
};

static const struct media_device_ops jenc_v4l2_mdev_ops = {
	.req_validate = vb2_request_validate,
	.req_queue = v4l2_m2m_request_queue,
};

int jenc_v4l2_device_create(struct jpeg_encoder *jenc)
{
	struct video_device *vfd;
	int rc;

	mutex_lock(&jenc->dev_mutex);

	rc = v4l2_device_register(jenc->dev, &jenc->v4l2_dev);
	if (rc)
		goto err_mutex_unlock;

	jenc->vfd = jenc_v4l2_vdev_ops;
	vfd = &jenc->vfd;
	vfd->lock = &jenc->dev_mutex;
	vfd->v4l2_dev = &jenc->v4l2_dev;

	rc = video_register_device(vfd, VFL_TYPE_VIDEO, 0);
	if (rc) {
		dev_err(jenc->dev, "Failed to register video device\n");
		goto err_v4l2_unregister;
	}

	jenc->m2m_dev = v4l2_m2m_init(&jenc_v4l2_m2m_ops);
	if (IS_ERR(jenc->m2m_dev)) {
		dev_err(jenc->dev, "Failed to init mem2mem device\n");
		rc = PTR_ERR(jenc->m2m_dev);
		jenc->m2m_dev = NULL;
		goto err_vdev_unregister;
	}

#ifdef CONFIG_MEDIA_CONTROLLER
	jenc->mdev.dev = jenc->dev;
	strscpy(jenc->mdev.model, CAM_JPEC_ENC_NAME,
		sizeof(jenc->mdev.model));
	strscpy(jenc->mdev.bus_info, "platform:" CAM_JPEC_ENC_NAME,
		sizeof(jenc->mdev.bus_info));
	media_device_init(&jenc->mdev);
	jenc->mdev.ops = &jenc_v4l2_mdev_ops;
	jenc->v4l2_dev.mdev = &jenc->mdev;

	rc = v4l2_m2m_register_media_controller(jenc->m2m_dev, vfd,
						JENC_MEDIA_ENT);
	if (rc) {
		dev_err(jenc->dev, "Failed to init m2m media controller\n");
		goto err_m2m_release;
	}

	rc = media_device_register(&jenc->mdev);
	if (rc) {
		dev_err(jenc->dev, "Failed to register m2m media device\n");
		goto err_mc_unregister;
	}
#endif
	video_set_drvdata(vfd, jenc);

	mutex_unlock(&jenc->dev_mutex);

	dev_info(jenc->dev, "Device registered as /jenc/video%d\n", vfd->num);

	return rc;

#ifdef CONFIG_MEDIA_CONTROLLER
err_mc_unregister:
	v4l2_m2m_unregister_media_controller(jenc->m2m_dev);
err_m2m_release:
	v4l2_m2m_release(jenc->m2m_dev);
	media_device_cleanup(&jenc->mdev);
#endif
err_vdev_unregister:
	video_unregister_device(&jenc->vfd);
err_v4l2_unregister:
	v4l2_device_unregister(&jenc->v4l2_dev);
err_mutex_unlock:
	mutex_unlock(&jenc->dev_mutex);
	mutex_destroy(&jenc->dev_mutex);

	return rc;
}

void jenc_v4l2_device_destroy(struct jpeg_encoder *jenc)
{
#ifdef CONFIG_MEDIA_CONTROLLER
	v4l2_m2m_unregister_media_controller(jenc->m2m_dev);
	media_device_unregister(&jenc->mdev);
	media_device_cleanup(&jenc->mdev);
#endif
	v4l2_m2m_release(jenc->m2m_dev);
	video_unregister_device(&jenc->vfd);
	v4l2_device_unregister(&jenc->v4l2_dev);
}
