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


#include <linux/types.h>	
#include <linux/kernel.h>	
#include <linux/mutex.h>	
#include <linux/list.h>		
#include <linux/slab.h>		
#include <linux/memory.h>	
#include <linux/interrupt.h>

#include "spsi.h"
#include "sps_core.h"

#define SPSRM_MAX_DESC_FIFO_SIZE    0xffff
#define SPSRM_MAX_DATA_FIFO_SIZE    0xffff

static struct sps_rm *sps_rm;

int sps_rm_init(struct sps_rm *rm, u32 options)
{
	
	sps_rm = rm;

	
	INIT_LIST_HEAD(&sps_rm->connections_q);
	mutex_init(&sps_rm->lock);

	return 0;
}

void sps_rm_config_init(struct sps_connect *connect)
{
	memset(connect, SPSRM_CLEAR, sizeof(*connect));
}

static void sps_rm_remove_ref(struct sps_connection *map)
{
	
	map->refs--;
	if (map->refs <= 0) {
		if (map->client_src != NULL || map->client_dest != NULL)
			SPS_ERR("sps:Failed to allocate connection struct");

		list_del(&map->list);
		kfree(map);
	}
}

static int sps_rm_map_match(const struct sps_connect *cfg,
			    const struct sps_connection *map)
{
	if (cfg->source != map->src.dev ||
	    cfg->destination != map->dest.dev)
		return false;

	if (cfg->src_pipe_index != SPSRM_CLEAR &&
	    cfg->src_pipe_index != map->src.pipe_index)
		return false;

	if (cfg->dest_pipe_index != SPSRM_CLEAR &&
	    cfg->dest_pipe_index != map->dest.pipe_index)
		return false;

	if (cfg->config != map->config)
		return false;

	if (cfg->desc.size != SPSRM_CLEAR) {
		if (cfg->desc.size != map->desc.size)
			return false;

		if (cfg->desc.phys_base != SPSRM_CLEAR &&
		    cfg->desc.base != (void *)SPSRM_CLEAR &&
		    (cfg->desc.phys_base != map->desc.phys_base ||
		     cfg->desc.base != map->desc.base)) {
			return false;
		}
	}

	if (cfg->data.size != SPSRM_CLEAR) {
		if (cfg->data.size != map->data.size)
			return false;

		if (cfg->data.phys_base != SPSRM_CLEAR &&
		    cfg->data.base != (void *)SPSRM_CLEAR &&
		    (cfg->data.phys_base != map->data.phys_base ||
		     cfg->data.base != map->data.base))
			return false;
	}

	return true;
}

static struct sps_connection *find_unconnected(struct sps_pipe *pipe)
{
	struct sps_connect *cfg = &pipe->connect;
	struct sps_connection *map;

	
	list_for_each_entry(map, &sps_rm->connections_q, list) {
		if (sps_rm_map_match(cfg, map))
			if ((cfg->mode == SPS_MODE_SRC
			     && map->client_src == NULL)
			    || (cfg->mode != SPS_MODE_SRC
				&& map->client_dest == NULL))
				return map;	
	}

	return NULL;		
}

static int sps_rm_assign(struct sps_pipe *pipe,
			 struct sps_connection *map)
{
	struct sps_connect *cfg = &pipe->connect;

	
	if ((cfg->mode == SPS_MODE_SRC && map->client_src != NULL) ||
	    (cfg->mode != SPS_MODE_SRC && map->client_dest != NULL)) {
		SPS_ERR("sps:The end point is already connected.\n");
		return SPS_ERROR;
	}

	
	if ((cfg->mode == SPS_MODE_SRC && map->src.bam == NULL) ||
	    (cfg->mode != SPS_MODE_SRC && map->dest.bam == NULL)) {
		SPS_ERR("sps:The end point is empty.\n");
		return SPS_ERROR;
	}

	
	if (cfg->mode == SPS_MODE_SRC) {
		map->client_src = pipe;
		pipe->bam = map->src.bam;
		pipe->pipe_index = map->src.pipe_index;
		if (pipe->connect.event_thresh != SPSRM_CLEAR)
			map->src.event_threshold = pipe->connect.event_thresh;
		if (pipe->connect.lock_group != SPSRM_CLEAR)
			map->src.lock_group = pipe->connect.lock_group;
	} else {
		map->client_dest = pipe;
		pipe->bam = map->dest.bam;
		pipe->pipe_index = map->dest.pipe_index;
		if (pipe->connect.event_thresh != SPSRM_CLEAR)
			map->dest.event_threshold =
			pipe->connect.event_thresh;
		if (pipe->connect.lock_group != SPSRM_CLEAR)
			map->dest.lock_group = pipe->connect.lock_group;
	}
	pipe->map = map;

	SPS_DBG("sps:sps_rm_assign.bam 0x%x.pipe_index=%d\n",
			BAM_ID(pipe->bam), pipe->pipe_index);

	
	pipe->connect.src_pipe_index = map->src.pipe_index;
	pipe->connect.dest_pipe_index = map->dest.pipe_index;
	pipe->connect.desc = map->desc;
	pipe->connect.data = map->data;

	pipe->client_state = SPS_STATE_ALLOCATE;

	return 0;
}

static void sps_rm_free_map_rsrc(struct sps_connection *map)
{
	struct sps_bam *bam;

	if (map->client_src != NULL || map->client_dest != NULL)
		return;

	if (map->alloc_src_pipe != SPS_BAM_PIPE_INVALID) {
		bam = map->src.bam;
		sps_bam_pipe_free(bam, map->src.pipe_index);

		
#ifdef CONFIG_SPS_SUPPORT_BAMDMA
		if ((bam->props.options & SPS_BAM_OPT_BAMDMA))
			
			sps_dma_pipe_free(bam, map->src.pipe_index);
#endif
		map->alloc_src_pipe = SPS_BAM_PIPE_INVALID;
		map->src.pipe_index = SPS_BAM_PIPE_INVALID;
	}
	if (map->alloc_dest_pipe != SPS_BAM_PIPE_INVALID) {
		bam = map->dest.bam;
		sps_bam_pipe_free(bam, map->dest.pipe_index);

		
#ifdef CONFIG_SPS_SUPPORT_BAMDMA
		if ((bam->props.options & SPS_BAM_OPT_BAMDMA)) {
			
			sps_dma_pipe_free(bam, map->dest.pipe_index);
		}
#endif
		map->alloc_dest_pipe = SPS_BAM_PIPE_INVALID;
		map->dest.pipe_index = SPS_BAM_PIPE_INVALID;
	}
	if (map->alloc_desc_base != SPS_ADDR_INVALID) {
		sps_mem_free_io(map->alloc_desc_base, map->desc.size);

		map->alloc_desc_base = SPS_ADDR_INVALID;
		map->desc.phys_base = SPS_ADDR_INVALID;
	}
	if (map->alloc_data_base != SPS_ADDR_INVALID) {
		sps_mem_free_io(map->alloc_data_base, map->data.size);

		map->alloc_data_base = SPS_ADDR_INVALID;
		map->data.phys_base = SPS_ADDR_INVALID;
	}
}

static void sps_rm_init_map(struct sps_connection *map,
			    const struct sps_connect *cfg)
{
	
	memset(map, 0, sizeof(*map));
	map->desc.phys_base = SPS_ADDR_INVALID;
	map->data.phys_base = SPS_ADDR_INVALID;
	map->alloc_desc_base = SPS_ADDR_INVALID;
	map->alloc_data_base = SPS_ADDR_INVALID;
	map->alloc_src_pipe = SPS_BAM_PIPE_INVALID;
	map->alloc_dest_pipe = SPS_BAM_PIPE_INVALID;

	
	map->src.dev = cfg->source;
	map->dest.dev = cfg->destination;
	map->desc.size = cfg->desc.size;
	map->data.size = cfg->data.size;
	map->config = cfg->config;

	
	if (map->desc.size != SPSRM_CLEAR &&
	    cfg->desc.phys_base != SPSRM_CLEAR &&
	    cfg->desc.base != (void *)SPSRM_CLEAR)
		map->desc = cfg->desc;

	
	if (map->data.size != SPSRM_CLEAR &&
	    cfg->data.phys_base != SPSRM_CLEAR &&
	    cfg->data.base != (void *)SPSRM_CLEAR)
		map->data = cfg->data;

	
	if (cfg->src_pipe_index != SPSRM_CLEAR)
		map->src.pipe_index = cfg->src_pipe_index;
	else
		map->src.pipe_index = SPS_BAM_PIPE_INVALID;


	
	if (cfg->dest_pipe_index != SPSRM_CLEAR)
		map->dest.pipe_index = cfg->dest_pipe_index;
	else
		map->dest.pipe_index = SPS_BAM_PIPE_INVALID;
}

static struct sps_connection *sps_rm_create(struct sps_pipe *pipe)
{
	struct sps_connection *map;
	struct sps_bam *bam;
	u32 desc_size;
	u32 data_size;
	enum sps_mode dir;
	int success = false;

	
	map = kzalloc(sizeof(*map), GFP_KERNEL);
	if (map == NULL) {
		SPS_ERR("sps:Failed to allocate connection struct");
		return NULL;
	}

	
	sps_rm_init_map(map, &pipe->connect);
	dir = pipe->connect.mode;

	
	success = false;
	
	map->src.bam = sps_h2bam(map->src.dev);
	if (map->src.bam == NULL) {
		if (map->src.dev != SPS_DEV_HANDLE_MEM) {
			SPS_ERR("sps:Invalid BAM handle: 0x%x", map->src.dev);
			goto exit_err;
		}
		map->src.pipe_index = SPS_BAM_PIPE_INVALID;
	}
	map->dest.bam = sps_h2bam(map->dest.dev);
	if (map->dest.bam == NULL) {
		if (map->dest.dev != SPS_DEV_HANDLE_MEM) {
			SPS_ERR("sps:Invalid BAM handle: 0x%x", map->dest.dev);
			goto exit_err;
		}
		map->dest.pipe_index = SPS_BAM_PIPE_INVALID;
	}

	
	if ((dir == SPS_MODE_SRC && map->src.bam == NULL) ||
	    (dir != SPS_MODE_SRC && map->dest.bam == NULL)) {
		SPS_ERR("sps:Invalid BAM endpt: dir %d src 0x%x dest 0x%x",
			dir, map->src.dev, map->dest.dev);
		goto exit_err;
	}

	
	if (map->src.bam != NULL) {
		
		bam = map->src.bam;
		map->alloc_src_pipe = sps_bam_pipe_alloc(bam,
							map->src.pipe_index);
		if (map->alloc_src_pipe == SPS_BAM_PIPE_INVALID)
			goto exit_err;
		map->src.pipe_index = map->alloc_src_pipe;

		
#ifdef CONFIG_SPS_SUPPORT_BAMDMA
		if ((bam->props.options & SPS_BAM_OPT_BAMDMA)) {
			int rc;
			
			rc = sps_dma_pipe_alloc(bam, map->src.pipe_index,
						 SPS_MODE_SRC);
			if (rc) {
				SPS_ERR("sps:Failed to alloc BAM-DMA pipe: %d",
					map->src.pipe_index);
				goto exit_err;
			}
		}
#endif
		map->src.bam_phys = bam->props.phys_addr;
		map->src.event_threshold = bam->props.event_threshold;
	}
	if (map->dest.bam != NULL) {
		
		bam = map->dest.bam;
		map->alloc_dest_pipe = sps_bam_pipe_alloc(bam,
							 map->dest.pipe_index);
		if (map->alloc_dest_pipe == SPS_BAM_PIPE_INVALID)
			goto exit_err;

		map->dest.pipe_index = map->alloc_dest_pipe;

		
#ifdef CONFIG_SPS_SUPPORT_BAMDMA
		if ((bam->props.options & SPS_BAM_OPT_BAMDMA)) {
			int rc;
			
			rc = sps_dma_pipe_alloc(bam, map->dest.pipe_index,
					       SPS_MODE_DEST);
			if (rc) {
				SPS_ERR("sps:Failed to alloc BAM-DMA pipe: %d",
					map->dest.pipe_index);
				goto exit_err;
			}
		}
#endif
		map->dest.bam_phys = bam->props.phys_addr;
		map->dest.event_threshold =
		bam->props.event_threshold;
	}

	
	desc_size = 0;
	data_size = 0;
	if (map->src.bam != NULL) {
		bam = map->src.bam;
		desc_size = bam->props.desc_size;
		data_size = bam->props.data_size;
	}
	if (map->dest.bam != NULL) {
		bam = map->dest.bam;
		if (bam->props.desc_size > desc_size)
			desc_size = bam->props.desc_size;
		if (bam->props.data_size > data_size)
			data_size = bam->props.data_size;
	}

	
	if (map->desc.size == SPSRM_CLEAR)
		map->desc.size = desc_size;
	if (map->src.bam != NULL && map->dest.bam != NULL) {
		
		if (map->data.size == SPSRM_CLEAR)
			map->data.size = data_size;
	} else {
		map->data.size = 0;
	}
	if (map->desc.size > SPSRM_MAX_DESC_FIFO_SIZE) {
		SPS_ERR("sps:Invalid desc FIFO size: 0x%x", map->desc.size);
		goto exit_err;
	}
	if (map->src.bam != NULL && map->dest.bam != NULL &&
	    map->data.size > SPSRM_MAX_DATA_FIFO_SIZE) {
		SPS_ERR("sps:Invalid data FIFO size: 0x%x", map->data.size);
		goto exit_err;
	}

	
	if (map->desc.size && map->desc.phys_base == SPS_ADDR_INVALID) {
		map->alloc_desc_base = sps_mem_alloc_io(map->desc.size);
		if (map->alloc_desc_base == SPS_ADDR_INVALID) {
			SPS_ERR("sps:I/O memory allocation failure:0x%x",
				map->desc.size);
			goto exit_err;
		}
		map->desc.phys_base = map->alloc_desc_base;
		map->desc.base = spsi_get_mem_ptr(map->desc.phys_base);
		if (map->desc.base == NULL) {
			SPS_ERR("sps:Cannot get virt addr for I/O buffer:%pa",
				&map->desc.phys_base);
			goto exit_err;
		}
	}

	
	if (map->data.size && map->data.phys_base == SPS_ADDR_INVALID) {
		map->alloc_data_base = sps_mem_alloc_io(map->data.size);
		if (map->alloc_data_base == SPS_ADDR_INVALID) {
			SPS_ERR("sps:I/O memory allocation failure:0x%x",
				map->data.size);
			goto exit_err;
		}
		map->data.phys_base = map->alloc_data_base;
		map->data.base = spsi_get_mem_ptr(map->data.phys_base);
		if (map->data.base == NULL) {
			SPS_ERR("sps:Cannot get virt addr for I/O buffer:%pa",
				&map->data.phys_base);
			goto exit_err;
		}
	}

	
	if (sps_rm_assign(pipe, map)) {
		SPS_ERR("sps:failed to assign a connection to the client.\n");
		goto exit_err;
	}

	
	success = true;
exit_err:

	
	if (!success) {
		sps_rm_free_map_rsrc(map);
		kfree(map);
		return NULL;
	}

	return map;
}

static int sps_rm_free(struct sps_pipe *pipe)
{
	struct sps_connection *map = (void *)pipe->map;
	struct sps_connect *cfg = &pipe->connect;

	mutex_lock(&sps_rm->lock);

	
	if (cfg->mode == SPS_MODE_SRC)
		map->client_src = NULL;
	else
		map->client_dest = NULL;

	pipe->map = NULL;
	pipe->client_state = SPS_STATE_DISCONNECT;
	sps_rm_free_map_rsrc(map);

	sps_rm_remove_ref(map);

	mutex_unlock(&sps_rm->lock);

	return 0;
}

static int sps_rm_alloc(struct sps_pipe *pipe)
{
	struct sps_connection *map;
	int result = SPS_ERROR;

	if (pipe->connect.sps_reserved != SPSRM_CLEAR) {
		u32 source = pipe->connect.source;
		u32 destination = pipe->connect.destination;
		enum sps_mode mode = pipe->connect.mode;
		u32 config = pipe->connect.config;
		memset(&pipe->connect, SPSRM_CLEAR,
			      sizeof(pipe->connect));
		pipe->connect.source = source;
		pipe->connect.destination = destination;
		pipe->connect.mode = mode;
		pipe->connect.config = config;
	}
	if (pipe->connect.config == SPSRM_CLEAR)
		pipe->connect.config = SPS_CONFIG_DEFAULT;

	if (pipe->connect.config != SPS_CONFIG_DEFAULT) {
		if (sps_map_find(&pipe->connect)) {
			SPS_ERR("sps:Failed to find connection mapping");
			return SPS_ERROR;
		}
	}

	mutex_lock(&sps_rm->lock);
	
	if (IS_SPS_STATE_OK(pipe)) {
		SPS_ERR("sps:Client connection already allocated");
		goto exit_err;
	}

	
	map = find_unconnected(pipe);
	if (map != NULL) {
		
		if (sps_rm_assign(pipe, map))
			
			map = NULL;
	}

	
	if (map == NULL) {
		map = sps_rm_create(pipe);
		if (map == NULL) {
			SPS_ERR("sps:Failed to allocate connection");
			goto exit_err;
		}
		list_add_tail(&map->list, &sps_rm->connections_q);
	}

	
	map->refs++;

	
	result = 0;
exit_err:
	mutex_unlock(&sps_rm->lock);

	if (result)
		return SPS_ERROR;

	return 0;
}

static int sps_rm_disconnect(struct sps_pipe *pipe)
{
	sps_rm_free(pipe);
	return 0;
}

int sps_rm_state_change(struct sps_pipe *pipe, u32 state)
{
	int auto_enable = false;
	int result;

	
	if (pipe->client_state == SPS_STATE_DISCONNECT &&
	    state == SPS_STATE_ALLOCATE) {
		if (sps_rm_alloc(pipe)) {
			SPS_ERR("sps:Fail to allocate resource for"
				" BAM 0x%x pipe %d",
				(u32) pipe->bam, pipe->pipe_index);
			return SPS_ERROR;
		}
	}

	
	if (pipe->client_state == SPS_STATE_ALLOCATE &&
	    state == SPS_STATE_CONNECT) {
		
		struct sps_bam_connect_param params;
		memset(&params, 0, sizeof(params));
		params.mode = pipe->connect.mode;
		if (pipe->connect.options != SPSRM_CLEAR) {
			params.options = pipe->connect.options;
			params.irq_gen_addr = pipe->connect.irq_gen_addr;
			params.irq_gen_data = pipe->connect.irq_gen_data;
		}
		result = sps_bam_pipe_connect(pipe, &params);
		if (result) {
			SPS_ERR("sps:Failed to connect BAM 0x%x pipe %d",
				(u32) pipe->bam, pipe->pipe_index);
			return SPS_ERROR;
		}
		pipe->client_state = SPS_STATE_CONNECT;

		
		if (pipe->connect.source == SPS_DEV_HANDLE_MEM ||
		    pipe->connect.destination == SPS_DEV_HANDLE_MEM) {
			if (pipe->map->desc.size != 0 &&
			    pipe->map->desc.phys_base != SPS_ADDR_INVALID)
				auto_enable = true;
		}
	}

	
	if (pipe->client_state == SPS_STATE_CONNECT &&
	    !(state == SPS_STATE_DISABLE
	      || state == SPS_STATE_DISCONNECT)
	    && (state == SPS_STATE_ENABLE || auto_enable
		|| (pipe->connect.options & SPS_O_AUTO_ENABLE))) {
		result = sps_bam_pipe_enable(pipe->bam, pipe->pipe_index);
		if (result) {
			SPS_ERR("sps:Failed to set BAM 0x%x pipe %d flow on",
				pipe->bam->props.phys_addr,
				pipe->pipe_index);
			return SPS_ERROR;
		}

		
#ifdef CONFIG_SPS_SUPPORT_BAMDMA
		if ((pipe->bam->props.options & SPS_BAM_OPT_BAMDMA)) {
			
			result = sps_dma_pipe_enable(pipe->bam,
						     pipe->pipe_index);
			if (result) {
				SPS_ERR("sps:Failed to activate BAM-DMA"
					" pipe: %d", pipe->pipe_index);
				return SPS_ERROR;
			}
		}
#endif
		pipe->client_state = SPS_STATE_ENABLE;
	}

	
	if (pipe->client_state == SPS_STATE_ENABLE &&
	    (state == SPS_STATE_DISABLE	|| state == SPS_STATE_DISCONNECT)) {
		result = sps_bam_pipe_disable(pipe->bam, pipe->pipe_index);
		if (result) {
			SPS_ERR("sps:Failed to set BAM 0x%x pipe %d flow off",
				pipe->bam->props.phys_addr,
				pipe->pipe_index);
			return SPS_ERROR;
		}
		pipe->client_state = SPS_STATE_CONNECT;
	}

	
	if (pipe->client_state == SPS_STATE_CONNECT &&
	    state == SPS_STATE_DISCONNECT) {
		struct sps_connection *map;
		struct sps_bam *bam = pipe->bam;
		unsigned long flags;
		u32 pipe_index;

		if (pipe->connect.mode == SPS_MODE_SRC)
			pipe_index = pipe->map->src.pipe_index;
		else
			pipe_index = pipe->map->dest.pipe_index;

		if (bam->props.irq > 0)
			synchronize_irq(bam->props.irq);

		spin_lock_irqsave(&bam->isr_lock, flags);
		pipe->disconnecting = true;
		spin_unlock_irqrestore(&bam->isr_lock, flags);
		result = sps_bam_pipe_disconnect(pipe->bam, pipe_index);
		if (result) {
			SPS_ERR("sps:Failed to disconnect BAM 0x%x pipe %d",
				pipe->bam->props.phys_addr,
				pipe->pipe_index);
			return SPS_ERROR;
		}

		
		map = (void *)pipe->map;
		if (pipe->connect.mode == SPS_MODE_SRC)
			map->client_src = NULL;
		else if (pipe->connect.mode == SPS_MODE_DEST)
			map->client_dest = NULL;

		sps_rm_disconnect(pipe);

		
		pipe->map = NULL;
		pipe->bam = NULL;
		pipe->client_state = SPS_STATE_DISCONNECT;
	}

	return 0;
}
