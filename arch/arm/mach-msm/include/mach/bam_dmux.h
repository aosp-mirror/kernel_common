/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
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

#include <linux/types.h>
#include <linux/skbuff.h>

#ifndef _BAM_DMUX_H
#define _BAM_DMUX_H

#define BAM_DMUX_CH_NAME_MAX_LEN	20

enum {
	BAM_DMUX_DATA_RMNET_0,
	BAM_DMUX_DATA_RMNET_1,
	BAM_DMUX_DATA_RMNET_2,
	BAM_DMUX_DATA_RMNET_3,
	BAM_DMUX_DATA_RMNET_4,
	BAM_DMUX_DATA_RMNET_5,
	BAM_DMUX_DATA_RMNET_6,
	BAM_DMUX_DATA_RMNET_7,
	BAM_DMUX_USB_RMNET_0,
	BAM_DMUX_RESERVED_0, 
	BAM_DMUX_RESERVED_1,
	BAM_DMUX_RESERVED_2,
	BAM_DMUX_DATA_REV_RMNET_0,
	BAM_DMUX_DATA_REV_RMNET_1,
	BAM_DMUX_DATA_REV_RMNET_2,
	BAM_DMUX_DATA_REV_RMNET_3,
	BAM_DMUX_DATA_REV_RMNET_4,
	BAM_DMUX_DATA_REV_RMNET_5,
	BAM_DMUX_DATA_REV_RMNET_6,
	BAM_DMUX_DATA_REV_RMNET_7,
	BAM_DMUX_DATA_REV_RMNET_8,
	BAM_DMUX_NUM_CHANNELS
};

enum {
	BAM_DMUX_RECEIVE, 
	BAM_DMUX_WRITE_DONE, 
	BAM_DMUX_UL_CONNECTED, 
	BAM_DMUX_UL_DISCONNECTED, 
};

#ifdef CONFIG_MSM_BAM_DMUX
int msm_bam_dmux_open(uint32_t id, void *priv,
		       void (*notify)(void *priv, int event_type,
						unsigned long data));

int msm_bam_dmux_close(uint32_t id);

int msm_bam_dmux_write(uint32_t id, struct sk_buff *skb);

int msm_bam_dmux_kickoff_ul_wakeup(void);

int msm_bam_dmux_ul_power_vote(void);

int msm_bam_dmux_ul_power_unvote(void);

int msm_bam_dmux_is_ch_full(uint32_t id);

int msm_bam_dmux_is_ch_low(uint32_t id);

int msm_bam_dmux_reg_notify(void *priv,
		       void (*notify)(void *priv, int event_type,
						unsigned long data));
void bam_change_adaptive_timer(int mode);
#else
static inline int msm_bam_dmux_open(uint32_t id, void *priv,
		       void (*notify)(void *priv, int event_type,
						unsigned long data))
{
	return -ENODEV;
}

static inline int msm_bam_dmux_close(uint32_t id)
{
	return -ENODEV;
}

static inline int msm_bam_dmux_write(uint32_t id, struct sk_buff *skb)
{
	return -ENODEV;
}

static inline int msm_bam_dmux_kickoff_ul_wakeup(void)
{
	return -ENODEV;
}

static inline int msm_bam_dmux_ul_power_vote(void)
{
	return -ENODEV;
}

static inline int msm_bam_dmux_ul_power_unvote(void)
{
	return -ENODEV;
}

static inline int msm_bam_dmux_is_ch_full(uint32_t id)
{
	return -ENODEV;
}

static inline int msm_bam_dmux_is_ch_low(uint32_t id)
{
	return -ENODEV;
}

static inline int msm_bam_dmux_reg_notify(void *priv,
		       void (*notify)(void *priv, int event_type,
						unsigned long data))
{
	return -ENODEV;
}
#endif
#endif 
