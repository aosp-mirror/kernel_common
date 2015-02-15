/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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



#ifndef _SPSBAM_H_
#define _SPSBAM_H_

#include <linux/types.h>
#include <linux/completion.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>

#include "spsi.h"

#define BAM_HANDLE_INVALID         0

enum bam_irq {
	BAM_DEV_IRQ_RDY_TO_SLEEP = 0x00000001,
	BAM_DEV_IRQ_HRESP_ERROR = 0x00000002,
	BAM_DEV_IRQ_ERROR = 0x00000004,
	BAM_DEV_IRQ_TIMER = 0x00000010,
};

enum bam_pipe_irq {
	
	BAM_PIPE_IRQ_DESC_INT = 0x00000001,
	
	BAM_PIPE_IRQ_TIMER = 0x00000002,
	
	BAM_PIPE_IRQ_WAKE = 0x00000004,
	
	
	BAM_PIPE_IRQ_OUT_OF_DESC = 0x00000008,
	
	BAM_PIPE_IRQ_ERROR = 0x00000010,
	
	BAM_PIPE_IRQ_EOT = 0x00000020,
	
	BAM_PIPE_IRQ_RST_ERROR = 0x00000040,
	
	BAM_PIPE_IRQ_HRESP_ERROR = 0x00000080,
};

enum bam_halt {
	BAM_HALT_OFF = 0,
	BAM_HALT_ON = 1,
};

enum bam_dma_thresh_dma {
	BAM_DMA_THRESH_512 = 0x3,
	BAM_DMA_THRESH_256 = 0x2,
	BAM_DMA_THRESH_128 = 0x1,
	BAM_DMA_THRESH_64 = 0x0,
};

enum bam_dma_weight_dma {
	BAM_DMA_WEIGHT_HIGH = 7,
	BAM_DMA_WEIGHT_MED = 3,
	BAM_DMA_WEIGHT_LOW = 1,
	BAM_DMA_WEIGHT_DEFAULT = BAM_DMA_WEIGHT_LOW,
	BAM_DMA_WEIGHT_DISABLE = 0,
};


#define SPS_BAM_PIPE_INVALID  ((u32)(-1))

struct sps_bam_connect_param {
	
	enum sps_mode mode;

	
	u32 options;

	
	u32 irq_gen_addr;
	u32 irq_gen_data;

};

struct sps_bam_event_reg {
	
	struct completion *xfer_done;
	void (*callback)(struct sps_event_notify *notify);

	
	enum sps_trigger mode;

	
	void *user;

};

struct sps_bam_desc_cache {
	struct sps_iovec iovec;
	void *user; 
};

struct sps_bam;

struct sps_bam_sys_mode {
	
	u8 *desc_buf; 
	u32 desc_offset; /* Next new descriptor to be written to hardware */
	u32 acked_offset; 

	
	u8 *desc_cache; 
	u32 cache_offset; 

	
	void **user_ptrs;

	
	struct sps_bam_event_reg event_regs[SPS_EVENT_INDEX(SPS_EVENT_MAX)];
	struct list_head events_q;

	struct sps_q_event event;	
	int no_queue;	
	int ack_xfers;	
	int handler_eot; 

	
#ifdef SPS_BAM_STATISTICS
	u32 desc_wr_count;
	u32 desc_rd_count;
	u32 user_ptrs_count;
	u32 user_found;
	u32 int_flags;
	u32 eot_flags;
	u32 callback_events;
	u32 wait_events;
	u32 queued_events;
	u32 get_events;
	u32 get_iovecs;
#endif 
};

struct sps_pipe {
	struct list_head list;

	
	u32 client_state;
	struct sps_bam *bam;
	struct sps_connect connect;
	const struct sps_connection *map;

	
	u32 state;
	u32 pipe_index;
	u32 pipe_index_mask;
	u32 irq_mask;
	int polled;
	int hybrid;
	u32 irq_gen_addr;
	enum sps_mode mode;
	u32 num_descs; 
	u32 desc_size; 
	int wake_up_is_one_shot; 

	
	struct sps_bam_sys_mode sys;

	bool disconnecting;
};

struct sps_bam {
	struct list_head list;

	
	struct sps_bam_props props;

	
	u32 state;
	struct mutex lock;
	void *base; 
	u32 version;
	spinlock_t isr_lock;
	spinlock_t connection_lock;
	unsigned long irqsave_flags;

	
	u32 pipe_active_mask;
	u32 pipe_remote_mask;
	struct sps_pipe *pipes[BAM_MAX_PIPES];
	struct list_head pipes_q;

	
	u32 irq_from_disabled_pipe;
	u32 event_trigger_failures;

};

int sps_bam_driver_init(u32 options);

int sps_bam_device_init(struct sps_bam *dev);

int sps_bam_device_de_init(struct sps_bam *dev);

int sps_bam_reset(struct sps_bam *dev);

int sps_bam_enable(struct sps_bam *dev);

int sps_bam_disable(struct sps_bam *dev);

u32 sps_bam_pipe_alloc(struct sps_bam *dev, u32 pipe_index);

void sps_bam_pipe_free(struct sps_bam *dev, u32 pipe_index);

int sps_bam_pipe_connect(struct sps_pipe *client,
			const struct sps_bam_connect_param *params);

int sps_bam_pipe_disconnect(struct sps_bam *dev, u32 pipe_index);

int sps_bam_pipe_set_params(struct sps_bam *dev, u32 pipe_index, u32 options);

int sps_bam_pipe_enable(struct sps_bam *dev, u32 pipe_index);

int sps_bam_pipe_disable(struct sps_bam *dev, u32 pipe_index);

int sps_bam_pipe_reg_event(struct sps_bam *dev, u32 pipe_index,
			   struct sps_register_event *reg);

int sps_bam_pipe_transfer_one(struct sps_bam *dev, u32 pipe_index, u32 addr,
			      u32 size, void *user, u32 flags);

int sps_bam_pipe_transfer(struct sps_bam *dev, u32 pipe_index,
			 struct sps_transfer *transfer);

int sps_bam_pipe_get_event(struct sps_bam *dev, u32 pipe_index,
			   struct sps_event_notify *notify);

int sps_bam_pipe_get_iovec(struct sps_bam *dev, u32 pipe_index,
			   struct sps_iovec *iovec);

int sps_bam_pipe_is_empty(struct sps_bam *dev, u32 pipe_index, u32 *empty);

int sps_bam_get_free_count(struct sps_bam *dev, u32 pipe_index, u32 *count);

int sps_bam_set_satellite(struct sps_bam *dev, u32 pipe_index);

int sps_bam_pipe_timer_ctrl(struct sps_bam *dev, u32 pipe_index,
			    struct sps_timer_ctrl *timer_ctrl,
			    struct sps_timer_result *timer_result);


int sps_bam_pipe_get_unused_desc_num(struct sps_bam *dev, u32 pipe_index,
					u32 *desc_num);

#endif	
