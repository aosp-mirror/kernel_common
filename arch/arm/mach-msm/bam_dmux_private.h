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
 */

#ifndef _BAM_DMUX_PRIVATE_H
#define _BAM_DMUX_PRIVATE_H

#include <linux/types.h>
#include <linux/dma-mapping.h>

#include <mach/sps.h>

#define BAM_MUX_HDR_MAGIC_NO			0x33fc
#define BAM_MUX_HDR_CMD_DATA			0
#define BAM_MUX_HDR_CMD_OPEN			1
#define BAM_MUX_HDR_CMD_CLOSE			2
#define BAM_MUX_HDR_CMD_STATUS			3 
#define BAM_MUX_HDR_CMD_OPEN_NO_A2_PC		4
#define BUFFER_SIZE				2048

struct bam_ops_if {
	
	int (*smsm_change_state_ptr)(uint32_t smsm_entry,
		uint32_t clear_mask, uint32_t set_mask);

	uint32_t (*smsm_get_state_ptr)(uint32_t smsm_entry);

	int (*smsm_state_cb_register_ptr)(uint32_t smsm_entry, uint32_t mask,
		void (*notify)(void *, uint32_t old_state, uint32_t new_state),
		void *data);

	int (*smsm_state_cb_deregister_ptr)(uint32_t smsm_entry, uint32_t mask,
		void (*notify)(void *, uint32_t, uint32_t), void *data);

	
	int (*sps_connect_ptr)(struct sps_pipe *h, struct sps_connect *connect);

	int (*sps_disconnect_ptr)(struct sps_pipe *h);

	int (*sps_register_bam_device_ptr)(
		const struct sps_bam_props *bam_props,
		u32 *dev_handle);

	int (*sps_deregister_bam_device_ptr)(u32 dev_handle);

	struct sps_pipe *(*sps_alloc_endpoint_ptr)(void);

	int (*sps_free_endpoint_ptr)(struct sps_pipe *h);

	int (*sps_set_config_ptr)(struct sps_pipe *h,
		struct sps_connect *config);

	int (*sps_get_config_ptr)(struct sps_pipe *h,
		struct sps_connect *config);

	int (*sps_device_reset_ptr)(u32 dev);

	int (*sps_register_event_ptr)(struct sps_pipe *h,
		struct sps_register_event *reg);

	int (*sps_transfer_one_ptr)(struct sps_pipe *h,
		u32 addr, u32 size,
		void *user, u32 flags);

	int (*sps_get_iovec_ptr)(struct sps_pipe *h,
		struct sps_iovec *iovec);

	int (*sps_get_unused_desc_num_ptr)(struct sps_pipe *h,
		u32 *desc_num);

	enum dma_data_direction dma_to;

	enum dma_data_direction dma_from;
};

struct bam_mux_hdr {
	uint16_t magic_num;
	uint8_t reserved;
	uint8_t cmd;
	uint8_t pad_len;
	uint8_t ch_id;
	uint16_t pkt_len;
};

struct rx_pkt_info {
	struct sk_buff *skb;
	dma_addr_t dma_address;
	struct work_struct work;
	struct list_head list_node;
};

struct tx_pkt_info {
	struct sk_buff *skb;
	dma_addr_t dma_address;
	char is_cmd;
	uint32_t len;
	struct work_struct work;
	struct list_head list_node;
	unsigned ts_sec;
	unsigned long ts_nsec;
};

void msm_bam_dmux_set_bam_ops(struct bam_ops_if *ops);

void msm_bam_dmux_deinit(void);

void msm_bam_dmux_reinit(void);

#endif 
