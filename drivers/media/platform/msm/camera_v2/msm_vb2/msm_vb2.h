/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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

#ifndef _MSM_VB_H
#define _MSM_VB_H

#include <linux/version.h>
#include <linux/i2c.h>
#include <linux/videodev2.h>
#include <linux/pm_qos.h>
#include <linux/wakelock.h>
#include <linux/msm_ion.h>
#include <linux/iommu.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mediabus.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-msm-mem.h>
#include <media/msmb_camera.h>
#include <media/videobuf2-core.h>
#include "msm.h"
#include "msm_sd.h"

struct msm_vb2_buffer {
	struct vb2_buffer vb2_buf;
	struct list_head list;
	int in_freeq;
};

struct msm_vb2_private_data {
	void *vaddr;
	unsigned long size;
	
	void *alloc_ctx;
};

struct msm_stream {
	struct list_head list;

	unsigned int stream_id;
	
	struct vb2_queue *vb2_q;
	spinlock_t stream_lock;
	struct list_head queued_list;
	
	unsigned int num_total; 
	unsigned int num_lend;  
	
};

struct vb2_ops *msm_vb2_get_q_ops(void);
struct vb2_mem_ops *msm_vb2_get_q_mem_ops(void);
int msm_vb2_request_cb(struct msm_sd_req_vb2_q *req_sd);

#endif 
