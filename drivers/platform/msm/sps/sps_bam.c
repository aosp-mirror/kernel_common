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
#include <linux/interrupt.h>	
#include <linux/memory.h>	
#include <linux/vmalloc.h>

#include "sps_bam.h"
#include "bam.h"
#include "spsi.h"

#define BAM_IRQ_ALL (BAM_DEV_IRQ_HRESP_ERROR | BAM_DEV_IRQ_ERROR |   \
	BAM_DEV_IRQ_TIMER)

#define BAM_STATE_INIT     (1UL << 1)
#define BAM_STATE_IRQ      (1UL << 2)
#define BAM_STATE_ENABLED  (1UL << 3)
#define BAM_STATE_BAM2BAM  (1UL << 4)
#define BAM_STATE_MTI      (1UL << 5)
#define BAM_STATE_REMOTE   (1UL << 6)

#define BAM_IOVEC_FLAG_MASK   \
	(SPS_IOVEC_FLAG_INT | SPS_IOVEC_FLAG_EOT | SPS_IOVEC_FLAG_EOB |   \
	SPS_IOVEC_FLAG_NWD | SPS_IOVEC_FLAG_CMD | SPS_IOVEC_FLAG_LOCK |   \
	SPS_IOVEC_FLAG_UNLOCK | SPS_IOVEC_FLAG_IMME)

#define BAM2BAM_O_INVALID   \
	(SPS_O_DESC_DONE | \
	 SPS_O_EOT | \
	 SPS_O_POLL | \
	 SPS_O_NO_Q | \
	 SPS_O_ACK_TRANSFERS)

#define BAM_PIPE_UNASSIGNED   ((struct sps_pipe *)0x77777777)

#define BAM_PIPE_IS_ASSIGNED(p)  \
	(((p) != NULL) && ((p) != BAM_PIPE_UNASSIGNED))

#define BAM_VERSION_MTI_SUPPORT(ver)   (ver <= 2)

struct sps_bam_opt_event_table {
	enum sps_event event_id;
	enum sps_option option;
	enum bam_pipe_irq pipe_irq;
};

static const struct sps_bam_opt_event_table opt_event_table[] = {
	{SPS_EVENT_EOT, SPS_O_EOT, BAM_PIPE_IRQ_EOT},
	{SPS_EVENT_DESC_DONE, SPS_O_DESC_DONE, BAM_PIPE_IRQ_DESC_INT},
	{SPS_EVENT_WAKEUP, SPS_O_WAKEUP, BAM_PIPE_IRQ_WAKE},
	{SPS_EVENT_INACTIVE, SPS_O_INACTIVE, BAM_PIPE_IRQ_TIMER},
	{SPS_EVENT_OUT_OF_DESC, SPS_O_OUT_OF_DESC,
		BAM_PIPE_IRQ_OUT_OF_DESC},
	{SPS_EVENT_ERROR, SPS_O_ERROR, BAM_PIPE_IRQ_ERROR},
	{SPS_EVENT_RST_ERROR, SPS_O_RST_ERROR, BAM_PIPE_IRQ_RST_ERROR},
	{SPS_EVENT_HRESP_ERROR, SPS_O_HRESP_ERROR, BAM_PIPE_IRQ_HRESP_ERROR}
};

static void pipe_handler(struct sps_bam *dev,
			struct sps_pipe *pipe);

static void pipe_handler_eot(struct sps_bam *dev,
			   struct sps_pipe *pipe);

int sps_bam_driver_init(u32 options)
{
	int n;

	for (n = 0; n < ARRAY_SIZE(opt_event_table); n++) {
		if ((u32)opt_event_table[n].option !=
			(u32)opt_event_table[n].pipe_irq) {
			SPS_ERR("sps:SPS_O 0x%x != HAL IRQ 0x%x\n",
				opt_event_table[n].option,
				opt_event_table[n].pipe_irq);
			return SPS_ERROR;
		}
	}

	return 0;
}

static irqreturn_t bam_isr(int irq, void *ctxt)
{
	struct sps_bam *dev = ctxt;
	struct sps_pipe *pipe;
	u32 source;
	unsigned long flags = 0;


	spin_lock_irqsave(&dev->isr_lock, flags);

	
	if ((dev->state & BAM_STATE_MTI) == 0) {
		u32 mask = dev->pipe_active_mask;
		enum sps_callback_case cb_case;
		source = bam_check_irq_source(dev->base, dev->props.ee,
						mask, &cb_case);

		SPS_DBG1("sps:bam_isr:bam=0x%x;source=0x%x;mask=0x%x.\n",
				BAM_ID(dev), source, mask);

		if ((source & (1UL << 31)) && (dev->props.callback)) {
			SPS_DBG1("sps:bam_isr:bam=0x%x;callback for case %d.\n",
				BAM_ID(dev), cb_case);
			dev->props.callback(cb_case, dev->props.user);
		}

		
		source &= dev->pipe_active_mask;
	} else {
		
		source = dev->pipe_active_mask;

		SPS_DBG1("sps:bam_isr for MTI:bam=0x%x;source=0x%x.\n",
				BAM_ID(dev), source);
	}

	
	pipe = list_first_entry(&dev->pipes_q, struct sps_pipe, list);

	list_for_each_entry(pipe, &dev->pipes_q, list) {
		
		if (BAM_PIPE_IS_ASSIGNED(pipe)
				&& (!pipe->disconnecting)
				&& (source & pipe->pipe_index_mask)) {
			
			pipe_handler(dev, pipe);
			source &= ~pipe->pipe_index_mask;
		}
		if (source == 0)
			break;
	}

	
	if (source) {
		SPS_ERR("sps:IRQ from BAM 0x%x inactive pipe(s) 0x%x\n",
			BAM_ID(dev), source);
		dev->irq_from_disabled_pipe++;
	}

	spin_unlock_irqrestore(&dev->isr_lock, flags);

	return IRQ_HANDLED;
}

int sps_bam_enable(struct sps_bam *dev)
{
	u32 num_pipes;
	u32 irq_mask;
	int result;
	int rc;
	int MTIenabled;

	
	if ((dev->state & BAM_STATE_ENABLED))
		return 0;	

	
	if ((dev->props.manage & SPS_BAM_MGR_ACCESS_MASK) == SPS_BAM_MGR_NONE) {
		SPS_ERR("sps:No local access to BAM 0x%x\n", BAM_ID(dev));
		return SPS_ERROR;
	}

	
	if ((dev->props.options & SPS_BAM_OPT_IRQ_DISABLED) != 0 ||
	    dev->props.irq == SPS_IRQ_INVALID) {
		
		irq_mask = 0;
		dev->state &= ~BAM_STATE_IRQ;
	} else {
		
		if (dev->props.irq > 0)
			result = request_irq(dev->props.irq,
				    (irq_handler_t) bam_isr,
				    IRQF_TRIGGER_HIGH, "sps", dev);

		if (result) {
			SPS_ERR("sps:Failed to enable BAM 0x%x IRQ %d\n",
				BAM_ID(dev), dev->props.irq);
			return SPS_ERROR;
		}

		
		irq_mask = BAM_IRQ_ALL;
		dev->state |= BAM_STATE_IRQ;

		
		if (dev->props.options & SPS_BAM_OPT_IRQ_WAKEUP) {
			result = enable_irq_wake(dev->props.irq);

			if (result) {
				SPS_ERR(
					"sps:Fail to enable wakeup irq for BAM 0x%x IRQ %d\n",
					BAM_ID(dev), dev->props.irq);
				return SPS_ERROR;
			} else
				SPS_DBG2(
					"sps:Enable wakeup irq for BAM 0x%x IRQ %d\n",
					BAM_ID(dev), dev->props.irq);
		}
	}

	
	num_pipes = 0;
	if ((dev->props.manage & SPS_BAM_MGR_DEVICE_REMOTE) == 0)
		
		rc = bam_init(dev->base,
				  dev->props.ee,
				  (u16) dev->props.summing_threshold,
				  irq_mask,
				  &dev->version, &num_pipes,
				  dev->props.options);
	else
		
		rc = bam_check(dev->base, &dev->version,
				dev->props.ee, &num_pipes);

	if (rc) {
		SPS_ERR("sps:Fail to init BAM 0x%x IRQ %d\n",
			BAM_ID(dev), dev->props.irq);
		return SPS_ERROR;
	}

	MTIenabled = BAM_VERSION_MTI_SUPPORT(dev->version);

	if ((dev->props.manage & SPS_BAM_MGR_DEVICE_REMOTE) != 0 &&
			(dev->props.manage & SPS_BAM_MGR_MULTI_EE) != 0 &&
			dev->props.ee == 0 && MTIenabled) {
		SPS_ERR("sps:EE for satellite BAM must be set to non-zero.\n");
		return SPS_ERROR;
	}

	if ((dev->state & BAM_STATE_IRQ) != 0 &&
		(dev->props.manage & SPS_BAM_MGR_DEVICE_REMOTE) != 0 &&
		MTIenabled) {
		if (dev->props.irq_gen_addr == 0 ||
		    dev->props.irq_gen_addr == SPS_ADDR_INVALID) {
			SPS_ERR(
				"sps:MTI destination address not specified for BAM 0x%x\n",
				BAM_ID(dev));
			return SPS_ERROR;
		}
		dev->state |= BAM_STATE_MTI;
	}

	if (num_pipes) {
		dev->props.num_pipes = num_pipes;
		SPS_DBG1("sps:BAM 0x%x number of pipes reported by hw: %d\n",
				 BAM_ID(dev), dev->props.num_pipes);
	}

	
	if (!MTIenabled && dev->props.ee >= SPS_BAM_NUM_EES) {
		SPS_ERR("sps:Invalid EE BAM 0x%x: %d\n", BAM_ID(dev),
				dev->props.ee);
		return SPS_ERROR;
	}

	if (!MTIenabled && dev->props.sec_config == SPS_BAM_SEC_DO_CONFIG) {
		struct sps_bam_sec_config_props *p_sec =
						dev->props.p_sec_config_props;
		if (p_sec == NULL) {
			SPS_ERR(
				"sps:EE config table is not specified for BAM 0x%x\n",
				BAM_ID(dev));
			return SPS_ERROR;
		}

		dev->props.restricted_pipes =
					~p_sec->ees[dev->props.ee].pipe_mask;

		if ((dev->props.manage & SPS_BAM_MGR_DEVICE_REMOTE) == 0) {
			u32 ee;
			u32 pipe_mask;
			int n, i;

			for (n = 0; n < SPS_BAM_NUM_EES - 1; n++) {
				for (i = n + 1; i < SPS_BAM_NUM_EES; i++) {
					if ((p_sec->ees[n].pipe_mask &
						p_sec->ees[i].pipe_mask) != 0) {
						SPS_ERR(
							"sps:Overlapping pipe assignments for BAM 0x%x: EEs %d and %d\n",
							BAM_ID(dev), n, i);
						return SPS_ERROR;
					}
				}
			}

			for (ee = 0; ee < SPS_BAM_NUM_EES; ee++) {
				pipe_mask = p_sec->ees[ee].pipe_mask;
				if (ee == dev->props.ee)
					pipe_mask |= (1UL << 31);
				else
					pipe_mask &= ~(1UL << 31);

				bam_security_init(dev->base, ee,
						p_sec->ees[ee].vmid, pipe_mask);
			}
		}
	}

	if ((dev->props.manage & SPS_BAM_MGR_DEVICE_REMOTE) == 0
			&& MTIenabled) {
		u32 pipe_index;
		u32 pipe_mask;
		for (pipe_index = 0, pipe_mask = 1;
		    pipe_index < dev->props.num_pipes;
		    pipe_index++, pipe_mask <<= 1) {
			if ((pipe_mask & dev->props.restricted_pipes) == 0)
				continue;	

			bam_pipe_satellite_mti(dev->base, pipe_index, 0,
						       dev->props.ee);
		}
	}

	dev->state |= BAM_STATE_ENABLED;

	if (!dev->props.constrained_logging ||
		(dev->props.constrained_logging && dev->props.logging_number)) {
		if (dev->props.logging_number > 0)
			dev->props.logging_number--;
		SPS_INFO(
			"sps:BAM 0x%x (va:0x%x) enabled: ver:0x%x, number of pipes:%d\n",
			BAM_ID(dev), (u32) dev->base, dev->version,
			dev->props.num_pipes);
	} else
		SPS_DBG2(
			"sps:BAM 0x%x (va:0x%x) enabled: ver:0x%x, number of pipes:%d\n",
			BAM_ID(dev), (u32) dev->base, dev->version,
			dev->props.num_pipes);

	return 0;
}

int sps_bam_disable(struct sps_bam *dev)
{
	if ((dev->state & BAM_STATE_ENABLED) == 0)
		return 0;

	
	if ((dev->props.manage & SPS_BAM_MGR_ACCESS_MASK) == SPS_BAM_MGR_NONE) {
		SPS_ERR("sps:No local access to BAM 0x%x\n", BAM_ID(dev));
		return SPS_ERROR;
	}

	
	if ((dev->props.manage & SPS_BAM_MGR_DEVICE_REMOTE)) {
		
		dev->state &= ~BAM_STATE_ENABLED;
		if ((dev->state & BAM_STATE_IRQ) && (dev->props.irq > 0)) {
			free_irq(dev->props.irq, dev);
			dev->state &= ~BAM_STATE_IRQ;
		}
		return 0;
	}

	
	if ((dev->state & BAM_STATE_IRQ)) {
		bam_exit(dev->base, dev->props.ee);

		
		if ((dev->state & BAM_STATE_IRQ))
			if (dev->props.irq > 0)
				free_irq(dev->props.irq, dev);
		dev->state &= ~BAM_STATE_IRQ;
	}

	dev->state &= ~BAM_STATE_ENABLED;

	SPS_DBG2("sps:BAM 0x%x disabled\n", BAM_ID(dev));

	return 0;
}

int sps_bam_device_init(struct sps_bam *dev)
{
	if (dev->props.virt_addr == NULL) {
		SPS_ERR("sps:NULL BAM virtual address\n");
		return SPS_ERROR;
	}
	dev->base = (void *) dev->props.virt_addr;

	if (dev->props.num_pipes == 0) {
		
		dev->props.num_pipes = BAM_MAX_PIPES;
		SPS_DBG2("sps:BAM 0x%x: assuming max number of pipes: %d\n",
			BAM_ID(dev), dev->props.num_pipes);
	}

	
	dev->state = 0;
	dev->pipe_active_mask = 0;
	dev->pipe_remote_mask = 0;
	INIT_LIST_HEAD(&dev->pipes_q);

	spin_lock_init(&dev->isr_lock);

	spin_lock_init(&dev->connection_lock);

	if ((dev->props.options & SPS_BAM_OPT_ENABLE_AT_BOOT))
		if (sps_bam_enable(dev)) {
			SPS_ERR("sps:Fail to enable bam device\n");
			return SPS_ERROR;
		}

	SPS_DBG2("sps:BAM device: phys 0x%x IRQ %d\n",
			BAM_ID(dev), dev->props.irq);

	return 0;
}

int sps_bam_device_de_init(struct sps_bam *dev)
{
	int result;

	SPS_DBG2("sps:BAM device DEINIT: phys 0x%x IRQ %d\n",
		BAM_ID(dev), dev->props.irq);

	result = sps_bam_disable(dev);

	return result;
}

int sps_bam_reset(struct sps_bam *dev)
{
	struct sps_pipe *pipe;
	u32 pipe_index;
	int result;

	SPS_DBG2("sps:BAM device RESET: phys 0x%x IRQ %d\n",
		BAM_ID(dev), dev->props.irq);

	
	result = 0;
	if ((dev->state & BAM_STATE_ENABLED)) {
		
		for (pipe_index = 0; pipe_index < dev->props.num_pipes;
		      pipe_index++) {
			pipe = dev->pipes[pipe_index];
			if (BAM_PIPE_IS_ASSIGNED(pipe)) {
				SPS_ERR(
					"sps:BAM device 0x%x RESET failed: pipe %d in use\n",
					BAM_ID(dev), pipe_index);
				result = SPS_ERROR;
				break;
			}
		}

		if (result == 0)
			result = sps_bam_disable(dev);
	}

	
	if (result == 0)
		result = sps_bam_enable(dev);

	return result;
}

static void pipe_clear(struct sps_pipe *pipe)
{
	INIT_LIST_HEAD(&pipe->list);

	pipe->state = 0;
	pipe->pipe_index = SPS_BAM_PIPE_INVALID;
	pipe->pipe_index_mask = 0;
	pipe->irq_mask = 0;
	pipe->mode = -1;
	pipe->num_descs = 0;
	pipe->desc_size = 0;
	pipe->disconnecting = false;
	memset(&pipe->sys, 0, sizeof(pipe->sys));
	INIT_LIST_HEAD(&pipe->sys.events_q);
}

u32 sps_bam_pipe_alloc(struct sps_bam *dev, u32 pipe_index)
{
	u32 pipe_mask;

	if (pipe_index == SPS_BAM_PIPE_INVALID) {
		
		if ((dev->props.manage & SPS_BAM_MGR_PIPE_NO_ALLOC)) {
			SPS_ERR(
				"sps:Restricted from allocating pipes on BAM 0x%x\n",
				BAM_ID(dev));
			return SPS_BAM_PIPE_INVALID;
		}
		for (pipe_index = 0, pipe_mask = 1;
		    pipe_index < dev->props.num_pipes;
		    pipe_index++, pipe_mask <<= 1) {
			if ((pipe_mask & dev->props.restricted_pipes))
				continue;	

			if (dev->pipes[pipe_index] == NULL)
				break;	
		}
		if (pipe_index >= dev->props.num_pipes) {
			SPS_ERR("sps:Fail to allocate pipe on BAM 0x%x\n",
				BAM_ID(dev));
			return SPS_BAM_PIPE_INVALID;
		}
	} else {
		
		if (pipe_index >= dev->props.num_pipes) {
			SPS_ERR(
				"sps:Invalid pipe %d for allocate on BAM 0x%x\n",
				pipe_index, BAM_ID(dev));
			return SPS_BAM_PIPE_INVALID;
		}
		if ((dev->props.restricted_pipes & (1UL << pipe_index))) {
			SPS_ERR("sps:BAM 0x%x pipe %d is not local\n",
				BAM_ID(dev), pipe_index);
			return SPS_BAM_PIPE_INVALID;
		}
		if (dev->pipes[pipe_index] != NULL) {
			SPS_ERR("sps:Pipe %d already allocated on BAM 0x%x\n",
				pipe_index, BAM_ID(dev));
			return SPS_BAM_PIPE_INVALID;
		}
	}

	
	dev->pipes[pipe_index] = BAM_PIPE_UNASSIGNED;

	return pipe_index;
}

void sps_bam_pipe_free(struct sps_bam *dev, u32 pipe_index)
{
	struct sps_pipe *pipe;

	if (pipe_index >= dev->props.num_pipes) {
		SPS_ERR("sps:Invalid BAM 0x%x pipe: %d\n", BAM_ID(dev),
				pipe_index);
		return;
	}

	
	pipe = dev->pipes[pipe_index];
	dev->pipes[pipe_index] = NULL;

	
	if (pipe == NULL) {
		SPS_ERR("sps:Attempt to free unallocated pipe %d on BAM 0x%x\n",
			pipe_index, BAM_ID(dev));
		return;
	}

	if (pipe == BAM_PIPE_UNASSIGNED)
		return;		

	
	if (!list_empty(&pipe->sys.events_q)) {
		struct sps_q_event *sps_event;

		SPS_ERR("sps:Disconnect BAM 0x%x pipe %d with events pending\n",
			BAM_ID(dev), pipe_index);

		sps_event = list_entry((&pipe->sys.events_q)->next,
				typeof(*sps_event), list);

		while (&sps_event->list != (&pipe->sys.events_q)) {
			struct sps_q_event *sps_event_delete = sps_event;

			list_del(&sps_event->list);
			sps_event = list_entry(sps_event->list.next,
					typeof(*sps_event), list);
			kfree(sps_event_delete);
		}
	}

	
	pipe_clear(pipe);
}

int sps_bam_pipe_connect(struct sps_pipe *bam_pipe,
			 const struct sps_bam_connect_param *params)
{
	struct bam_pipe_parameters hw_params;
	struct sps_bam *dev;
	const struct sps_connection *map = bam_pipe->map;
	const struct sps_conn_end_pt *map_pipe;
	const struct sps_conn_end_pt *other_pipe;
	void *desc_buf = NULL;
	u32 pipe_index;
	int result;

	
	pipe_clear(bam_pipe);
	memset(&hw_params, 0, sizeof(hw_params));

	
	bam_pipe->mode = params->mode;

	
	if ((params->options & SPS_O_STREAMING) == 0)
		hw_params.stream_mode = BAM_STREAM_MODE_DISABLE;
	else
		hw_params.stream_mode = BAM_STREAM_MODE_ENABLE;

	
	if (bam_pipe->mode == SPS_MODE_SRC) {
		map_pipe = &map->src;
		other_pipe = &map->dest;
		hw_params.dir = BAM_PIPE_PRODUCER;
	} else {
		map_pipe = &map->dest;
		other_pipe = &map->src;
		hw_params.dir = BAM_PIPE_CONSUMER;
	}

	
	dev = map_pipe->bam;
	pipe_index = map_pipe->pipe_index;
	if (pipe_index >= dev->props.num_pipes) {
		SPS_ERR("sps:Invalid BAM 0x%x pipe: %d\n", BAM_ID(dev),
				pipe_index);
		return SPS_ERROR;
	}
	hw_params.event_threshold = (u16) map_pipe->event_threshold;
	hw_params.ee = dev->props.ee;
	hw_params.lock_group = map_pipe->lock_group;

	
	if ((dev->props.manage & SPS_BAM_MGR_PIPE_NO_CTRL) ||
	    (dev->props.restricted_pipes & (1UL << pipe_index))) {
		SPS_ERR("sps:BAM 0x%x pipe %d is not local\n",
			BAM_ID(dev), pipe_index);
		return SPS_ERROR;
	}

	
	if ((dev->props.manage & SPS_BAM_MGR_PIPE_NO_CONFIG)) {
		SPS_ERR("sps:BAM 0x%x pipe %d remote config is not supported\n",
			BAM_ID(dev), pipe_index);
		return SPS_ERROR;
	}

	
	if (other_pipe->bam != NULL) {
		
		bam_pipe->state |= BAM_STATE_BAM2BAM;
		hw_params.mode = BAM_PIPE_MODE_BAM2BAM;
		hw_params.peer_phys_addr =
			((struct sps_bam *) (other_pipe->bam))->props.phys_addr;
		hw_params.peer_pipe = other_pipe->pipe_index;

		
		if (map->desc.phys_base == SPS_ADDR_INVALID ||
		    map->data.phys_base == SPS_ADDR_INVALID ||
		    map->desc.size == 0 || map->data.size == 0) {
			SPS_ERR(
				"sps:FIFO buffers are not allocated for BAM 0x%x pipe %d.\n",
				BAM_ID(dev), pipe_index);
			return SPS_ERROR;
		}
		hw_params.data_base = map->data.phys_base;
		hw_params.data_size = map->data.size;

		
		if (map->data.base != NULL && bam_pipe->mode == SPS_MODE_SRC)
			memset(map->data.base, 0, hw_params.data_size);

		
		if (bam_pipe->mode == SPS_MODE_SRC) {
			if ((params->options & SPS_O_WRITE_NWD) == 0)
				hw_params.write_nwd = BAM_WRITE_NWD_DISABLE;
			else
				hw_params.write_nwd = BAM_WRITE_NWD_ENABLE;
		}
	} else {
		
		hw_params.mode = BAM_PIPE_MODE_SYSTEM;
		bam_pipe->sys.desc_buf = map->desc.base;
		bam_pipe->sys.desc_offset = 0;
		bam_pipe->sys.acked_offset = 0;
	}

	
	bam_pipe->pipe_index = pipe_index;
	bam_pipe->pipe_index_mask = 1UL << pipe_index;

	
	if (map->desc.phys_base != SPS_ADDR_INVALID) {
		if (map->desc.size < (2 * sizeof(struct sps_iovec))) {
			SPS_ERR(
				"sps:Invalid descriptor FIFO size for BAM 0x%x pipe %d: %d\n",
				BAM_ID(dev), pipe_index, map->desc.size);
			return SPS_ERROR;
		}
		desc_buf = map->desc.base;

		hw_params.desc_base = map->desc.phys_base;
		hw_params.desc_size = map->desc.size;
	}

	
	if (desc_buf != NULL)
		if (bam_pipe->mode == SPS_MODE_SRC ||
		    hw_params.mode == BAM_PIPE_MODE_SYSTEM)
			memset(desc_buf, 0, hw_params.desc_size);

	bam_pipe->desc_size = hw_params.desc_size;
	bam_pipe->num_descs = bam_pipe->desc_size / sizeof(struct sps_iovec);

	result = SPS_ERROR;
	
	if ((dev->state & BAM_STATE_ENABLED) == 0)
		if (sps_bam_enable(dev))
			goto exit_init_err;

	
	if (dev->pipes[pipe_index] != BAM_PIPE_UNASSIGNED) {
		SPS_ERR("sps:Invalid pipe %d on BAM 0x%x for connect\n",
			pipe_index, BAM_ID(dev));
		return SPS_ERROR;
	}

	if (bam_pipe_is_enabled(dev->base, pipe_index)) {
		if (params->options & SPS_O_NO_DISABLE)
			SPS_DBG("sps:BAM 0x%x pipe %d is already enabled\n",
				BAM_ID(dev), pipe_index);
		else {
			SPS_ERR("sps:BAM 0x%x pipe %d sharing violation\n",
				BAM_ID(dev), pipe_index);
			return SPS_ERROR;
		}
	}

	if (bam_pipe_init(dev->base, pipe_index, &hw_params, dev->props.ee)) {
		SPS_ERR("sps:BAM 0x%x pipe %d init error\n",
			BAM_ID(dev), pipe_index);
		goto exit_err;
	}

	
	dev->pipes[pipe_index] = bam_pipe;

	
	if (params->options != 0 ||
	    (bam_pipe->state & BAM_STATE_BAM2BAM) == 0) {
		
		u32 irq_gen_addr;

		
		irq_gen_addr = SPS_ADDR_INVALID;
		if ((params->options & SPS_O_IRQ_MTI))
			
			irq_gen_addr = params->irq_gen_addr;
		else if ((dev->state & BAM_STATE_MTI))
			
			irq_gen_addr = dev->props.irq_gen_addr;

		if (irq_gen_addr != SPS_ADDR_INVALID) {
			bam_pipe->state |= BAM_STATE_MTI;
			bam_pipe->irq_gen_addr = irq_gen_addr;
		}

		
		if (sps_bam_pipe_set_params(dev, pipe_index,
					  params->options)) {
			dev->pipes[pipe_index] = BAM_PIPE_UNASSIGNED;
			goto exit_err;
		}
	}

	
	dev->pipes[pipe_index] = bam_pipe;
	dev->pipe_active_mask |= 1UL << pipe_index;
	list_add_tail(&bam_pipe->list, &dev->pipes_q);

	bam_pipe->state |= BAM_STATE_INIT;
	result = 0;
exit_err:
	if (result) {
		if (params->options & SPS_O_NO_DISABLE)
			SPS_DBG("sps:BAM 0x%x pipe %d connection exits\n",
				BAM_ID(dev), pipe_index);
		else
			bam_pipe_exit(dev->base, pipe_index, dev->props.ee);
	}
exit_init_err:
	if (result) {
		
		pipe_clear(bam_pipe);
	}

	return result;
}

int sps_bam_pipe_disconnect(struct sps_bam *dev, u32 pipe_index)
{
	struct sps_pipe *pipe;
	int result;

	if (pipe_index >= dev->props.num_pipes) {
		SPS_ERR("sps:Invalid BAM 0x%x pipe: %d\n", BAM_ID(dev),
				pipe_index);
		return SPS_ERROR;
	}

	
	pipe = dev->pipes[pipe_index];
	if (BAM_PIPE_IS_ASSIGNED(pipe)) {
		if ((dev->pipe_active_mask & (1UL << pipe_index))) {
			list_del(&pipe->list);
			dev->pipe_active_mask &= ~(1UL << pipe_index);
		}
		dev->pipe_remote_mask &= ~(1UL << pipe_index);
		if (pipe->connect.options & SPS_O_NO_DISABLE)
			SPS_DBG("sps:BAM 0x%x pipe %d exits\n", BAM_ID(dev),
				pipe_index);
		else
			bam_pipe_exit(dev->base, pipe_index, dev->props.ee);
		if (pipe->sys.desc_cache != NULL) {
			u32 size = pipe->num_descs * sizeof(void *);
			if (pipe->desc_size + size <= PAGE_SIZE)
				kfree(pipe->sys.desc_cache);
			else
				vfree(pipe->sys.desc_cache);
			pipe->sys.desc_cache = NULL;
		}
		dev->pipes[pipe_index] = BAM_PIPE_UNASSIGNED;
		pipe_clear(pipe);
		result = 0;
	} else {
		result = SPS_ERROR;
	}

	if (result)
		SPS_ERR("sps:BAM 0x%x pipe %d already disconnected\n",
			BAM_ID(dev), pipe_index);

	return result;
}

static void pipe_set_irq(struct sps_bam *dev, u32 pipe_index,
				 u32 poll)
{
	struct sps_pipe *pipe = dev->pipes[pipe_index];
	enum bam_enable irq_enable;

	if (poll == 0 && pipe->irq_mask != 0 &&
	    (dev->state & BAM_STATE_IRQ)) {
		if ((pipe->state & BAM_STATE_BAM2BAM) != 0 &&
		    (pipe->state & BAM_STATE_IRQ) == 0) {
			(void)bam_pipe_get_and_clear_irq_status(dev->base,
							   pipe_index);
		}
		pipe->state |= BAM_STATE_IRQ;
		irq_enable = BAM_ENABLE;
		pipe->polled = false;
	} else {
		pipe->state &= ~BAM_STATE_IRQ;
		irq_enable = BAM_DISABLE;
		pipe->polled = true;
		if (poll == 0 && pipe->irq_mask)
			SPS_DBG2("sps:BAM 0x%x pipe %d forced to use polling\n",
				 BAM_ID(dev), pipe_index);
	}
	if ((pipe->state & BAM_STATE_MTI) == 0)
		bam_pipe_set_irq(dev->base, pipe_index, irq_enable,
					 pipe->irq_mask, dev->props.ee);
	else
		bam_pipe_set_mti(dev->base, pipe_index, irq_enable,
					 pipe->irq_mask, pipe->irq_gen_addr);

}

int sps_bam_pipe_set_params(struct sps_bam *dev, u32 pipe_index, u32 options)
{
	struct sps_pipe *pipe = dev->pipes[pipe_index];
	u32 mask;
	int wake_up_is_one_shot;
	int no_queue;
	int ack_xfers;
	u32 size;
	int n;

	
	wake_up_is_one_shot = ((options & SPS_O_WAKEUP_IS_ONESHOT));
	no_queue = ((options & SPS_O_NO_Q));
	ack_xfers = ((options & SPS_O_ACK_TRANSFERS));

	pipe->hybrid = options & SPS_O_HYBRID;

	
	mask = 0;
	for (n = 0; n < ARRAY_SIZE(opt_event_table); n++) {
		
		if ((options & opt_event_table[n].option) == 0)
			continue;	

		mask |= opt_event_table[n].pipe_irq;
	}

#ifdef SPS_BAM_STATISTICS
	
	if (pipe->sys.desc_wr_count > 0 &&
	    (no_queue != pipe->sys.no_queue
	     || ack_xfers != pipe->sys.ack_xfers)) {
		SPS_ERR(
			"sps:Queue/ack mode change after transfer: BAM 0x%x pipe %d opt 0x%x\n",
			BAM_ID(dev), pipe_index, options);
		return SPS_ERROR;
	}
#endif 

	
	if ((pipe->state & BAM_STATE_BAM2BAM) &&
	    (options & BAM2BAM_O_INVALID)) {
		SPS_ERR(
			"sps:Invalid option for BAM-to-BAM: BAM 0x%x pipe %d opt 0x%x\n",
			BAM_ID(dev), pipe_index, options);
		return SPS_ERROR;
	}

	
	if (!no_queue && pipe->sys.desc_cache == NULL && pipe->num_descs > 0
	    && (pipe->state & BAM_STATE_BAM2BAM) == 0) {
		
		size = pipe->num_descs * sizeof(void *);

		if (pipe->desc_size + size <= PAGE_SIZE)
			pipe->sys.desc_cache =
				kzalloc(pipe->desc_size + size, GFP_KERNEL);
		else {
			pipe->sys.desc_cache =
				vmalloc(pipe->desc_size + size);

			if (pipe->sys.desc_cache == NULL) {
				SPS_ERR(
					"sps:No memory for pipe %d of BAM 0x%x\n",
					pipe_index, BAM_ID(dev));
				return -ENOMEM;
			}

			memset(pipe->sys.desc_cache, 0, pipe->desc_size + size);
		}

		if (pipe->sys.desc_cache == NULL) {
			
			SPS_ERR("sps:Desc cache error: BAM 0x%x pipe %d: %d\n",
				BAM_ID(dev), pipe_index,
				pipe->desc_size + size);
			return SPS_ERROR;
		}
		pipe->sys.user_ptrs = (void **)(pipe->sys.desc_cache +
						 pipe->desc_size);
		pipe->sys.cache_offset = pipe->sys.acked_offset;
	}


	
	pipe->irq_mask = mask;
	pipe_set_irq(dev, pipe_index, (options & SPS_O_POLL));

	
	pipe->wake_up_is_one_shot = wake_up_is_one_shot;
	pipe->sys.no_queue = no_queue;
	pipe->sys.ack_xfers = ack_xfers;

	return 0;
}

int sps_bam_pipe_enable(struct sps_bam *dev, u32 pipe_index)
{
	struct sps_pipe *pipe = dev->pipes[pipe_index];

	
	bam_pipe_enable(dev->base, pipe_index);
	pipe->state |= BAM_STATE_ENABLED;

	return 0;
}

int sps_bam_pipe_disable(struct sps_bam *dev, u32 pipe_index)
{
	struct sps_pipe *pipe = dev->pipes[pipe_index];

	
	if (pipe->connect.options & SPS_O_NO_DISABLE)
		SPS_DBG("sps:BAM 0x%x pipe %d enters disable state\n",
			BAM_ID(dev), pipe_index);
	else
		bam_pipe_disable(dev->base, pipe_index);

	pipe->state &= ~BAM_STATE_ENABLED;

	return 0;
}

int sps_bam_pipe_reg_event(struct sps_bam *dev,
			   u32 pipe_index,
			   struct sps_register_event *reg)
{
	struct sps_pipe *pipe = dev->pipes[pipe_index];
	struct sps_bam_event_reg *event_reg;
	int n;

	if (pipe->sys.no_queue && reg->xfer_done != NULL &&
	    reg->mode != SPS_TRIGGER_CALLBACK) {
		SPS_ERR(
			"sps:Only callback events support for NO_Q: BAM 0x%x pipe %d mode %d\n",
			BAM_ID(dev), pipe_index, reg->mode);
		return SPS_ERROR;
	}

	for (n = 0; n < ARRAY_SIZE(opt_event_table); n++) {
		int index;

		
		if ((reg->options & opt_event_table[n].option) == 0)
			continue;	

		index = SPS_EVENT_INDEX(opt_event_table[n].event_id);
		if (index < 0)
			SPS_ERR(
				"sps:Negative event index: BAM 0x%x pipe %d mode %d\n",
				BAM_ID(dev), pipe_index, reg->mode);
		else {
			event_reg = &pipe->sys.event_regs[index];
			event_reg->xfer_done = reg->xfer_done;
			event_reg->callback = reg->callback;
			event_reg->mode = reg->mode;
			event_reg->user = reg->user;
		}
	}

	return 0;
}

int sps_bam_pipe_transfer_one(struct sps_bam *dev,
				    u32 pipe_index, u32 addr, u32 size,
				    void *user, u32 flags)
{
	struct sps_pipe *pipe = dev->pipes[pipe_index];
	struct sps_iovec *desc;
	struct sps_iovec iovec;
	u32 next_write;
	static int show_recom;

	
	if ((pipe->state & (BAM_STATE_BAM2BAM | BAM_STATE_REMOTE))) {
		SPS_ERR("sps:Transfer on BAM-to-BAM: BAM 0x%x pipe %d\n",
			BAM_ID(dev), pipe_index);
		return SPS_ERROR;
	}

	if (pipe->sys.no_queue && user != NULL) {
		SPS_ERR("sps:User pointer arg non-NULL: BAM 0x%x pipe %d\n",
			BAM_ID(dev), pipe_index);
		return SPS_ERROR;
	}

	
	next_write = pipe->sys.desc_offset + sizeof(struct sps_iovec);
	if (next_write >= pipe->desc_size)
		next_write = 0;

	if (next_write == pipe->sys.acked_offset) {
		if (!pipe->sys.ack_xfers && pipe->polled) {
			pipe_handler_eot(dev, pipe);
			if (next_write == pipe->sys.acked_offset) {
				if (!show_recom) {
					show_recom = true;
					SPS_ERR(
						"sps:Client of BAM 0x%x pipe %d is recommended to have flow control\n",
						BAM_ID(dev), pipe_index);
				}

				SPS_DBG2(
					"sps:Descriptor FIFO is full for BAM 0x%x pipe %d after pipe_handler_eot\n",
					BAM_ID(dev), pipe_index);
				return SPS_ERROR;
			}
		} else {
			if (!show_recom) {
				show_recom = true;
				SPS_ERR(
					"sps:Client of BAM 0x%x pipe %d is recommended to have flow control.\n",
					BAM_ID(dev), pipe_index);
			}

			SPS_DBG2(
				"sps:Descriptor FIFO is full for BAM 0x%x pipe %d\n",
				BAM_ID(dev), pipe_index);
			return SPS_ERROR;
		}
	}

	
	if (!pipe->sys.no_queue)
		desc = (struct sps_iovec *) (pipe->sys.desc_cache +
					      pipe->sys.desc_offset);
	else
		desc = &iovec;

	desc->addr = addr;
	desc->size = size;

	if ((flags & SPS_IOVEC_FLAG_DEFAULT) == 0) {
		desc->flags = (flags & BAM_IOVEC_FLAG_MASK)
				| DESC_UPPER_ADDR(flags);
	} else {
		if (pipe->mode == SPS_MODE_SRC)
			desc->flags = SPS_IOVEC_FLAG_INT
					| DESC_UPPER_ADDR(flags);
		else
			desc->flags = (SPS_IOVEC_FLAG_INT | SPS_IOVEC_FLAG_EOT)
					| DESC_UPPER_ADDR(flags);
	}

#ifdef SPS_BAM_STATISTICS
	if ((flags & SPS_IOVEC_FLAG_INT))
		pipe->sys.int_flags++;
	if ((flags & SPS_IOVEC_FLAG_EOT))
		pipe->sys.eot_flags++;
#endif 

	
	*((struct sps_iovec *) (pipe->sys.desc_buf + pipe->sys.desc_offset))
	= *desc;

	
	if (!pipe->sys.no_queue) {
		u32 index = pipe->sys.desc_offset / sizeof(struct sps_iovec);
		pipe->sys.user_ptrs[index] = user;
#ifdef SPS_BAM_STATISTICS
		if (user != NULL)
			pipe->sys.user_ptrs_count++;
#endif 
	}

	
	pipe->sys.desc_offset = next_write;

#ifdef SPS_BAM_STATISTICS
	
	pipe->sys.desc_wr_count++;
#endif 

	
	if ((flags & SPS_IOVEC_FLAG_NO_SUBMIT) == 0) {
		wmb(); 
		bam_pipe_set_desc_write_offset(dev->base, pipe_index,
					       next_write);
	}

	return 0;
}

int sps_bam_pipe_transfer(struct sps_bam *dev,
			 u32 pipe_index, struct sps_transfer *transfer)
{
	struct sps_iovec *iovec;
	u32 count;
	u32 flags;
	void *user;
	int n;
	int result;

	if (transfer->iovec_count == 0) {
		SPS_ERR("sps:iovec count zero: BAM 0x%x pipe %d\n",
			BAM_ID(dev), pipe_index);
		return SPS_ERROR;
	}

	sps_bam_get_free_count(dev, pipe_index, &count);
	if (count < transfer->iovec_count) {
		SPS_ERR("sps:Insufficient free desc: BAM 0x%x pipe %d: %d\n",
			BAM_ID(dev), pipe_index, count);
		return SPS_ERROR;
	}

	user = NULL;		
	for (n = (int)transfer->iovec_count - 1, iovec = transfer->iovec;
	    n >= 0; n--, iovec++) {
		if (n > 0) {
			
			flags = iovec->flags | SPS_IOVEC_FLAG_NO_SUBMIT;
		} else {
			
			flags = iovec->flags;
			user = transfer->user;
		}
		result = sps_bam_pipe_transfer_one(dev, pipe_index,
						 iovec->addr,
						 iovec->size, user,
						 flags);
		if (result)
			return SPS_ERROR;
	}

	return 0;
}

static struct sps_q_event *alloc_event(struct sps_pipe *pipe,
					struct sps_bam_event_reg *event_reg)
{
	struct sps_q_event *event;

	
	event = &pipe->sys.event;
	memset(event, 0, sizeof(*event));

	return event;
}

static void trigger_event(struct sps_bam *dev,
			  struct sps_pipe *pipe,
			  struct sps_bam_event_reg *event_reg,
			  struct sps_q_event *sps_event)
{
	if (sps_event == NULL) {
		SPS_DBG("sps:trigger_event.sps_event is NULL.\n");
		return;
	}

	if (event_reg->xfer_done) {
		complete(event_reg->xfer_done);
		SPS_DBG("sps:trigger_event.done=%d.\n",
			event_reg->xfer_done->done);
	}

	if (event_reg->callback) {
		SPS_DBG("sps:trigger_event.using callback.\n");
		event_reg->callback(&sps_event->notify);
	}

}

static void pipe_handler_generic(struct sps_bam *dev,
			       struct sps_pipe *pipe,
			       enum sps_event event_id)
{
	struct sps_bam_event_reg *event_reg;
	struct sps_q_event *sps_event;
	int index;

	index = SPS_EVENT_INDEX(event_id);
	if (index < 0 || index >= SPS_EVENT_INDEX(SPS_EVENT_MAX))
		return;

	event_reg = &pipe->sys.event_regs[index];
	sps_event = alloc_event(pipe, event_reg);
	if (sps_event != NULL) {
		sps_event->notify.event_id = event_id;
		sps_event->notify.user = event_reg->user;
		trigger_event(dev, pipe, event_reg, sps_event);
	}
}

static void pipe_handler_wakeup(struct sps_bam *dev, struct sps_pipe *pipe)
{
	struct sps_bam_event_reg *event_reg;
	struct sps_q_event *event;
	u32 pipe_index = pipe->pipe_index;

	if (pipe->wake_up_is_one_shot) {
		
		pipe->irq_mask &= ~BAM_PIPE_IRQ_WAKE;
		pipe_set_irq(dev, pipe_index, pipe->polled);
	}

	event_reg = &pipe->sys.event_regs[SPS_EVENT_INDEX(SPS_EVENT_WAKEUP)];
	event = alloc_event(pipe, event_reg);
	if (event != NULL) {
		event->notify.event_id = SPS_EVENT_WAKEUP;
		event->notify.user = event_reg->user;
		trigger_event(dev, pipe, event_reg, event);
	}
}

static void pipe_handler_eot(struct sps_bam *dev, struct sps_pipe *pipe)
{
	struct sps_bam_event_reg *event_reg;
	struct sps_q_event *event;
	struct sps_iovec *desc;
	struct sps_iovec *cache;
	void **user;
	u32 *update_offset;
	u32 pipe_index = pipe->pipe_index;
	u32 offset;
	u32 end_offset;
	enum sps_event event_id;
	u32 flags;
	u32 enabled;
	int producer = (pipe->mode == SPS_MODE_SRC);

	if (pipe->sys.handler_eot)
		return;

	pipe->sys.handler_eot = true;

	
	end_offset = bam_pipe_get_desc_read_offset(dev->base, pipe_index);

	
	if (pipe->sys.no_queue) {
		if (!pipe->sys.ack_xfers) {
			
			pipe->sys.acked_offset = end_offset;
		}
		pipe->sys.handler_eot = false;
		return;
	}

	if (!pipe->sys.ack_xfers) {
		update_offset = &pipe->sys.acked_offset;
		offset = *update_offset;
	} else {
		update_offset = &pipe->sys.cache_offset;
		offset = *update_offset;
	}

	
	if (offset == end_offset) {
		pipe->sys.handler_eot = false;
		return;
	}

	
	enabled = 0;
	if ((pipe->irq_mask & SPS_O_EOT))
		enabled |= SPS_IOVEC_FLAG_EOT;

	if ((pipe->irq_mask & SPS_O_DESC_DONE))
		enabled |= SPS_IOVEC_FLAG_INT;

	if (producer) {
		struct sps_iovec *desc_end;

		
		desc = (struct sps_iovec *) (pipe->sys.desc_buf + offset);
		cache =	(struct sps_iovec *) (pipe->sys.desc_cache + offset);

		
		if (end_offset < offset) {
			desc_end = (struct sps_iovec *)
				   (pipe->sys.desc_buf + pipe->desc_size);
			while (desc < desc_end)
				*cache++ = *desc++;

			desc = (void *)pipe->sys.desc_buf;
			cache = (void *)pipe->sys.desc_cache;
		}

		
		desc_end = (struct sps_iovec *)	(pipe->sys.desc_buf +
						 end_offset);
		while (desc < desc_end)
			*cache++ = *desc++;
	}

	
	cache = (struct sps_iovec *) (pipe->sys.desc_cache + offset);
	user = &pipe->sys.user_ptrs[offset / sizeof(struct sps_iovec)];
	for (;;) {
		offset += sizeof(struct sps_iovec);
		if (offset >= pipe->desc_size)
			
			offset = 0;

		*update_offset = offset;
#ifdef SPS_BAM_STATISTICS
		pipe->sys.desc_rd_count++;
#endif 

		
		flags = cache->flags & enabled;
		if (*user != NULL || flags) {
			int index;

			if ((flags & SPS_IOVEC_FLAG_EOT))
				event_id = SPS_EVENT_EOT;
			else
				event_id = SPS_EVENT_DESC_DONE;

			index = SPS_EVENT_INDEX(event_id);
			event_reg = &pipe->sys.event_regs[index];
			event = alloc_event(pipe, event_reg);
			if (event != NULL) {
				event->notify.data.transfer.iovec = *cache;
				event->notify.data.transfer.user = *user;

				event->notify.event_id = event_id;
				event->notify.user = event_reg->user;
				trigger_event(dev, pipe, event_reg, event);
			}
#ifdef SPS_BAM_STATISTICS
			if (*user != NULL)
				pipe->sys.user_found++;
#endif 
		}

		
		if (offset == end_offset)
			break;	

		if (offset) {
			cache++;
			user++;
		} else {
			cache = (void *)pipe->sys.desc_cache;
			user = pipe->sys.user_ptrs;
		}
	}

	pipe->sys.handler_eot = false;
}

static void pipe_handler(struct sps_bam *dev, struct sps_pipe *pipe)
{
	u32 pipe_index;
	u32 status;
	enum sps_event event_id;

	
	pipe_index = pipe->pipe_index;
	status = bam_pipe_get_and_clear_irq_status(dev->base, pipe_index);

	SPS_DBG("sps:pipe_handler.bam 0x%x.pipe %d.status=0x%x.\n",
			BAM_ID(dev), pipe_index, status);

	
	status &= pipe->irq_mask;
	if (status == 0)
		
		return;


	if ((status & (SPS_O_EOT | SPS_O_DESC_DONE)) &&
	    (pipe->state & BAM_STATE_BAM2BAM) == 0) {
		pipe_handler_eot(dev, pipe);
		if (pipe->sys.no_queue) {
			if ((status & SPS_O_EOT))
				event_id = SPS_EVENT_EOT;
			else
				event_id = SPS_EVENT_DESC_DONE;

			pipe_handler_generic(dev, pipe, event_id);
		}
		status &= ~(SPS_O_EOT | SPS_O_DESC_DONE);
		if (status == 0)
			return;
	}

	if ((status & SPS_O_WAKEUP)) {
		pipe_handler_wakeup(dev, pipe);
		status &= ~SPS_O_WAKEUP;
		if (status == 0)
			return;
	}

	if ((status & SPS_O_INACTIVE)) {
		pipe_handler_generic(dev, pipe, SPS_EVENT_INACTIVE);
		status &= ~SPS_O_INACTIVE;
		if (status == 0)
			return;
	}

	if ((status & SPS_O_OUT_OF_DESC)) {
		pipe_handler_generic(dev, pipe,
					     SPS_EVENT_OUT_OF_DESC);
		status &= ~SPS_O_OUT_OF_DESC;
		if (status == 0)
			return;
	}

	if ((status & SPS_O_RST_ERROR) && enhd_pipe) {
		SPS_ERR("sps:bam 0x%x ;pipe 0x%x irq status=0x%x.\n"
				"sps: BAM_PIPE_IRQ_RST_ERROR\n",
				BAM_ID(dev), pipe_index, status);
		bam_output_register_content(dev->base, dev->props.ee);
		pipe_handler_generic(dev, pipe,
					     SPS_EVENT_RST_ERROR);
		status &= ~SPS_O_RST_ERROR;
		if (status == 0)
			return;
	}

	if ((status & SPS_O_HRESP_ERROR) && enhd_pipe) {
		SPS_ERR("sps:bam 0x%x ;pipe 0x%x irq status=0x%x.\n"
				"sps: BAM_PIPE_IRQ_HRESP_ERROR\n",
				BAM_ID(dev), pipe_index, status);
		bam_output_register_content(dev->base, dev->props.ee);
		pipe_handler_generic(dev, pipe,
					     SPS_EVENT_HRESP_ERROR);
		status &= ~SPS_O_HRESP_ERROR;
		if (status == 0)
			return;
	}

	if ((status & SPS_EVENT_ERROR))
		pipe_handler_generic(dev, pipe, SPS_EVENT_ERROR);
}

int sps_bam_pipe_get_event(struct sps_bam *dev,
			   u32 pipe_index, struct sps_event_notify *notify)
{
	struct sps_pipe *pipe = dev->pipes[pipe_index];
	struct sps_q_event *event_queue;

	if (pipe->sys.no_queue) {
		SPS_ERR(
			"sps:Invalid connection for event: BAM 0x%x pipe %d context 0x%x\n",
			BAM_ID(dev), pipe_index, (u32) pipe);
		notify->event_id = SPS_EVENT_INVALID;
		return SPS_ERROR;
	}

	
	if (pipe->polled && (pipe->state & BAM_STATE_BAM2BAM) == 0)
		pipe_handler_eot(dev, pipe);

	
	if (list_empty(&pipe->sys.events_q)) {
		event_queue = NULL;
		SPS_DBG("sps:events_q of bam 0x%x is empty.\n", BAM_ID(dev));
	} else {
		SPS_DBG("sps:events_q of bam 0x%x is not empty.\n",
			BAM_ID(dev));
		event_queue =
		list_first_entry(&pipe->sys.events_q, struct sps_q_event,
				 list);
		list_del(&event_queue->list);
	}

	
	if (event_queue == NULL) {
		
		notify->event_id = SPS_EVENT_INVALID;
	} else {
		*notify = event_queue->notify;
		kfree(event_queue);
#ifdef SPS_BAM_STATISTICS
		pipe->sys.get_events++;
#endif 
	}

	return 0;
}

int sps_bam_pipe_get_iovec(struct sps_bam *dev, u32 pipe_index,
			   struct sps_iovec *iovec)
{
	struct sps_pipe *pipe = dev->pipes[pipe_index];
	struct sps_iovec *desc;
	u32 read_offset;

	
	if (!pipe->sys.ack_xfers ||
	    (pipe->state & BAM_STATE_BAM2BAM) != 0 ||
	    (pipe->state & BAM_STATE_REMOTE)) {
		return SPS_ERROR;
	}

	
	if ((pipe->polled || pipe->hybrid) && !pipe->sys.no_queue)
		pipe_handler_eot(dev, pipe);

	
	if (pipe->sys.no_queue)
		read_offset =
		bam_pipe_get_desc_read_offset(dev->base, pipe_index);
	else
		read_offset = pipe->sys.cache_offset;

	if (read_offset == pipe->sys.acked_offset) {
		
		memset(iovec, 0, sizeof(*iovec));
		return 0;
	}

	
	desc = (struct sps_iovec *) (pipe->sys.desc_buf +
				     pipe->sys.acked_offset);
	*iovec = *desc;
#ifdef SPS_BAM_STATISTICS
	pipe->sys.get_iovecs++;
#endif 

	
	pipe->sys.acked_offset += sizeof(struct sps_iovec);
	if (pipe->sys.acked_offset >= pipe->desc_size)
		pipe->sys.acked_offset = 0;

	return 0;
}

int sps_bam_pipe_is_empty(struct sps_bam *dev, u32 pipe_index,
				u32 *empty)
{
	struct sps_pipe *pipe = dev->pipes[pipe_index];
	u32 end_offset;
	u32 acked_offset;

	
	if ((pipe->state & BAM_STATE_REMOTE)) {
		SPS_ERR("sps:Is empty on remote: BAM 0x%x pipe %d\n",
			BAM_ID(dev), pipe_index);
		return SPS_ERROR;
	}

	
	end_offset = bam_pipe_get_desc_read_offset(dev->base, pipe_index);

	if ((pipe->state & BAM_STATE_BAM2BAM) == 0)
		
		acked_offset = pipe->sys.acked_offset;
	else
		
		acked_offset = bam_pipe_get_desc_write_offset(dev->base,
							  pipe_index);


	
	if (end_offset == acked_offset)
		*empty = true;
	else
		*empty = false;

	return 0;
}

int sps_bam_get_free_count(struct sps_bam *dev, u32 pipe_index,
				 u32 *count)
{
	struct sps_pipe *pipe = dev->pipes[pipe_index];
	u32 next_write;
	u32 free;

	
	if ((pipe->state & (BAM_STATE_BAM2BAM | BAM_STATE_REMOTE))) {
		SPS_ERR(
			"sps:Free count on BAM-to-BAM or remote: BAM 0x%x pipe %d\n",
			BAM_ID(dev), pipe_index);
		*count = 0;
		return SPS_ERROR;
	}

	
	next_write = pipe->sys.desc_offset + sizeof(struct sps_iovec);
	if (next_write >= pipe->desc_size)
		next_write = 0;

	if (pipe->sys.acked_offset >= next_write)
		free = pipe->sys.acked_offset - next_write;
	else
		free = pipe->desc_size - next_write + pipe->sys.acked_offset;

	free /= sizeof(struct sps_iovec);
	*count = free;

	return 0;
}

int sps_bam_set_satellite(struct sps_bam *dev, u32 pipe_index)
{
	struct sps_pipe *pipe = dev->pipes[pipe_index];

	if ((dev->props.manage & SPS_BAM_MGR_MULTI_EE) == 0 ||
	    (dev->props.manage & SPS_BAM_MGR_DEVICE_REMOTE)) {
		SPS_ERR(
			"sps:Cannot grant satellite control to BAM 0x%x pipe %d\n",
			BAM_ID(dev), pipe_index);
		return SPS_ERROR;
	}

	
	if ((dev->pipe_active_mask & (1UL << pipe_index)) == 0) {
		SPS_ERR("sps:BAM 0x%x pipe %d not local and active\n",
			BAM_ID(dev), pipe_index);
		return SPS_ERROR;
	}

	
	if (!pipe->polled)
		bam_pipe_set_irq(dev->base, pipe_index, BAM_DISABLE,
					 pipe->irq_mask, dev->props.ee);

	if (BAM_VERSION_MTI_SUPPORT(dev->version)) {
		if ((pipe->state & BAM_STATE_MTI) == 0 || pipe->polled) {
			bam_pipe_satellite_mti(dev->base, pipe_index, 0,
						       dev->props.ee);
			pipe->state |= BAM_STATE_MTI;
		}
	}

	
	list_del(&pipe->list);
	dev->pipe_active_mask &= ~(1UL << pipe_index);
	dev->pipe_remote_mask |= pipe->pipe_index_mask;
	pipe->state |= BAM_STATE_REMOTE;

	return 0;
}

int sps_bam_pipe_timer_ctrl(struct sps_bam *dev,
			    u32 pipe_index,
			    struct sps_timer_ctrl *timer_ctrl,
			    struct sps_timer_result *timer_result)
{
	enum bam_pipe_timer_mode mode;
	int result = 0;

	
	if ((dev->pipe_active_mask & (1UL << pipe_index)) == 0) {
		SPS_ERR("sps:BAM 0x%x pipe %d not local and active\n",
			BAM_ID(dev), pipe_index);
		return SPS_ERROR;
	}

	
	switch (timer_ctrl->op) {
	case SPS_TIMER_OP_CONFIG:
		mode = (timer_ctrl->mode == SPS_TIMER_MODE_ONESHOT) ?
			BAM_PIPE_TIMER_ONESHOT :
			BAM_PIPE_TIMER_PERIODIC;
		bam_pipe_timer_config(dev->base, pipe_index, mode,
				    timer_ctrl->timeout_msec * 8);
		break;
	case SPS_TIMER_OP_RESET:
		bam_pipe_timer_reset(dev->base, pipe_index);
		break;
	case SPS_TIMER_OP_READ:
		break;
	default:
		result = SPS_ERROR;
		break;
	}

	
	if (timer_result != NULL)
		timer_result->current_timer =
			bam_pipe_timer_get_count(dev->base, pipe_index);

	return result;
}

int sps_bam_pipe_get_unused_desc_num(struct sps_bam *dev, u32 pipe_index,
					u32 *desc_num)
{
	u32 sw_offset, peer_offset, fifo_size;
	u32 desc_size = sizeof(struct sps_iovec);
	struct sps_pipe *pipe = dev->pipes[pipe_index];

	if (pipe == NULL)
		return SPS_ERROR;

	fifo_size = pipe->desc_size;

	sw_offset = bam_pipe_get_desc_read_offset(dev->base, pipe_index);
	peer_offset = bam_pipe_get_desc_write_offset(dev->base, pipe_index);

	if (sw_offset <= peer_offset)
		*desc_num = (peer_offset - sw_offset) / desc_size;
	else
		*desc_num = (peer_offset + fifo_size - sw_offset) / desc_size;

	return 0;
}
