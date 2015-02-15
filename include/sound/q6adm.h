/* Copyright (c) 2010-2012, The Linux Foundation. All rights reserved.
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
#ifndef __Q6_ADM_H__
#define __Q6_ADM_H__
#include <sound/q6afe.h>

#define ADM_PATH_PLAYBACK 0x1
#define ADM_PATH_LIVE_REC 0x2
#define ADM_PATH_NONLIVE_REC 0x3

/* multiple copp per stream. */
struct route_payload {
	unsigned int copp_ids[AFE_MAX_PORTS];
	unsigned short num_copps;
	unsigned int session_id;
};

int adm_open(int port, int path, int rate, int mode, int topology);

int adm_multi_ch_copp_open(int port, int path, int rate, int mode,
				int topology, int perfmode);

int adm_memory_map_regions(uint32_t *buf_add, uint32_t mempool_id,
				uint32_t *bufsz, uint32_t bufcnt);

int adm_memory_unmap_regions(uint32_t *buf_add, uint32_t *bufsz,
						uint32_t bufcnt);

int adm_close(int port);

int adm_pseudo_close(int port);

int adm_matrix_map(int session_id, int path, int num_copps,
				unsigned int *port_id, int copp_id);

int adm_connect_afe_port(int mode, int session_id, int port_id);
int adm_disconnect_afe_port(int mode, int session_id, int port_id);

void adm_ec_ref_rx_id(int  port_id);

int adm_connect_afe_port_v2(int mode, int session_id, int port_id,
					int sample_rate, int channels);

int adm_multi_ch_copp_pseudo_open_v3(int port_id, int path, int rate,
				int channel_mode, int topology);

#ifdef CONFIG_RTAC
int adm_get_copp_id(int port_id);
#endif

#endif /* __Q6_ADM_H__ */
