/**
 * gadget.c - DesignWare USB3 DRD Controller Gadget Framework Link
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *	    Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2, as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/ratelimit.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>

#include "core.h"
#include "gadget.h"
#include "debug.h"
#include "io.h"

static void dwc3_gadget_usb2_phy_suspend(struct dwc3 *dwc, int suspend);
static void dwc3_gadget_usb3_phy_suspend(struct dwc3 *dwc, int suspend);

int g_irq = 0;

int dwc3_gadget_set_test_mode(struct dwc3 *dwc, int mode)
{
	u32		reg;

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg &= ~DWC3_DCTL_TSTCTRL_MASK;

	switch (mode) {
	case TEST_J:
	case TEST_K:
	case TEST_SE0_NAK:
	case TEST_PACKET:
	case TEST_FORCE_EN:
		reg |= mode << 1;
		schedule_delayed_work(&dwc->chg_stop, 0);
		break;
	default:
		return -EINVAL;
	}

	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	return 0;
}

int dwc3_gadget_set_link_state(struct dwc3 *dwc, enum dwc3_link_state state)
{
	int		retries = 10000;
	u32		reg;

	if (dwc->revision >= DWC3_REVISION_194A) {
		while (--retries) {
			reg = dwc3_readl(dwc->regs, DWC3_DSTS);
			if (reg & DWC3_DSTS_DCNRD)
				udelay(5);
			else
				break;
		}

		if (retries <= 0)
			return -ETIMEDOUT;
	}

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg &= ~DWC3_DCTL_ULSTCHNGREQ_MASK;

	
	reg |= DWC3_DCTL_ULSTCHNGREQ(state);
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	if (dwc->revision >= DWC3_REVISION_194A)
		return 0;

	
	retries = 10000;
	while (--retries) {
		reg = dwc3_readl(dwc->regs, DWC3_DSTS);

		if (DWC3_DSTS_USBLNKST(reg) == state)
			return 0;

		udelay(5);
	}

	dev_vdbg(dwc->dev, "link state change request timed out\n");

	return -ETIMEDOUT;
}

int dwc3_gadget_resize_tx_fifos(struct dwc3 *dwc)
{
	int		last_fifo_depth = 0;
	int		ram1_depth;
	int		fifo_size;
	int		mdwidth;
	int		num;

	if (!dwc->needs_fifo_resize)
		return 0;

	ram1_depth = DWC3_RAM1_DEPTH(dwc->hwparams.hwparams7);
	mdwidth = DWC3_MDWIDTH(dwc->hwparams.hwparams0);

	
	mdwidth >>= 3;

	for (num = 0; num < DWC3_ENDPOINTS_NUM; num++) {
		struct dwc3_ep	*dep = dwc->eps[num];
		int		fifo_number = dep->number >> 1;
		int		mult = 1;
		int		tmp;

		if (!(dep->number & 1))
			continue;

		if (!(dep->flags & DWC3_EP_ENABLED))
			continue;

		if (((dep->endpoint.maxburst > 1) &&
				usb_endpoint_xfer_bulk(dep->endpoint.desc))
				|| usb_endpoint_xfer_isoc(dep->endpoint.desc))
			mult = 3;

		tmp = mult * (dep->endpoint.maxpacket + mdwidth);

		if (dwc->tx_fifo_size &&
			(usb_endpoint_xfer_bulk(dep->endpoint.desc)
			|| usb_endpoint_xfer_isoc(dep->endpoint.desc))) {
			if (!dwc->tx_fifo_reduced)
				tmp = 3 * (1024 + mdwidth);
			else
				tmp = mult * (1024 + mdwidth);
		}

		tmp += mdwidth;

		fifo_size = DIV_ROUND_UP(tmp, mdwidth);

		fifo_size |= (last_fifo_depth << 16);

		dev_vdbg(dwc->dev, "%s: Fifo Addr %04x Size %d\n",
				dep->name, last_fifo_depth, fifo_size & 0xffff);

		last_fifo_depth += (fifo_size & 0xffff);
		if (dwc->tx_fifo_size &&
				(last_fifo_depth >= dwc->tx_fifo_size)) {
			dev_err(dwc->dev, "Fifosize(%d) > available RAM(%d)\n",
					last_fifo_depth, dwc->tx_fifo_size);
			return -ENOMEM;
		}

		dwc3_writel(dwc->regs, DWC3_GTXFIFOSIZ(fifo_number),
				fifo_size);

	}

	return 0;
}

void dwc3_gadget_giveback(struct dwc3_ep *dep, struct dwc3_request *req,
		int status)
{
	struct dwc3			*dwc = dep->dwc;

	if (req->queued) {
		req->queued = false;

		if (req->request.num_mapped_sgs)
			dep->busy_slot += req->request.num_mapped_sgs;
		else
			dep->busy_slot++;

		if (((dep->busy_slot & DWC3_TRB_MASK) == DWC3_TRB_NUM - 1) &&
				usb_endpoint_xfer_isoc(dep->endpoint.desc))
			dep->busy_slot++;

		if (req->request.zero && req->ztrb) {
			dep->busy_slot++;
			req->ztrb = NULL;
			if (((dep->busy_slot & DWC3_TRB_MASK) ==
				DWC3_TRB_NUM - 1) &&
				usb_endpoint_xfer_isoc(dep->endpoint.desc))
				dep->busy_slot++;
		}
	}
	list_del(&req->list);
	req->trb = NULL;

	if (req->request.status == -EINPROGRESS)
		req->request.status = status;

	if (dwc->ep0_bounced && dep->number == 0)
		dwc->ep0_bounced = false;
	else
		usb_gadget_unmap_request(&dwc->gadget, &req->request,
				req->direction);

	dev_dbg(dwc->dev, "request %p from %s completed %d/%d ===> %d\n",
			req, dep->name, req->request.actual,
			req->request.length, status);

	dbg_done(dep->number, req->request.actual, req->request.status);
	spin_unlock(&dwc->lock);
	if (unlikely(!req->request.complete)) {
		printk(KERN_ERR "[USB] request.complete is NULL, epnum %d dir %d\n",dep->number,dep->direction);
		printk(KERN_ERR "[USB] request.complete is NULL, request 0x%p\n",&req->request);
	}
	req->request.complete(&dep->endpoint, &req->request);
	spin_lock(&dwc->lock);
}

static const char *dwc3_gadget_ep_cmd_string(u8 cmd)
{
	switch (cmd) {
	case DWC3_DEPCMD_DEPSTARTCFG:
		return "Start New Configuration";
	case DWC3_DEPCMD_ENDTRANSFER:
		return "End Transfer";
	case DWC3_DEPCMD_UPDATETRANSFER:
		return "Update Transfer";
	case DWC3_DEPCMD_STARTTRANSFER:
		return "Start Transfer";
	case DWC3_DEPCMD_CLEARSTALL:
		return "Clear Stall";
	case DWC3_DEPCMD_SETSTALL:
		return "Set Stall";
	case DWC3_DEPCMD_GETEPSTATE:
		return "Get Endpoint State";
	case DWC3_DEPCMD_SETTRANSFRESOURCE:
		return "Set Endpoint Transfer Resource";
	case DWC3_DEPCMD_SETEPCONFIG:
		return "Set Endpoint Configuration";
	default:
		return "UNKNOWN command";
	}
}

int dwc3_send_gadget_generic_command(struct dwc3 *dwc, int cmd, u32 param)
{
	u32		timeout = 500;
	u32		reg;
	bool hsphy_suspend_enabled;
	int ret;

	
	hsphy_suspend_enabled = (dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0)) &
						DWC3_GUSB2PHYCFG_SUSPHY);

	
	if (hsphy_suspend_enabled)
		dwc3_gadget_usb2_phy_suspend(dwc, false);

	dwc3_writel(dwc->regs, DWC3_DGCMDPAR, param);
	dwc3_writel(dwc->regs, DWC3_DGCMD, cmd | DWC3_DGCMD_CMDACT);

	do {
		reg = dwc3_readl(dwc->regs, DWC3_DGCMD);
		if (!(reg & DWC3_DGCMD_CMDACT)) {
			dev_vdbg(dwc->dev, "Command Complete --> %d\n",
					DWC3_DGCMD_STATUS(reg));
			ret = 0;
			break;
		}

		timeout--;
		if (!timeout) {
			ret = -ETIMEDOUT;
			break;
		}
		udelay(1);
	} while (1);

	
	if (hsphy_suspend_enabled)
		dwc3_gadget_usb2_phy_suspend(dwc, true);

	return ret;
}

int dwc3_send_gadget_ep_cmd(struct dwc3 *dwc, unsigned ep,
		unsigned cmd, struct dwc3_gadget_ep_cmd_params *params)
{
	struct dwc3_ep		*dep = dwc->eps[ep];
	u32			timeout = 500;
	u32			reg;
	bool hsphy_suspend_enabled;
	int ret;

	dev_vdbg(dwc->dev, "%s: cmd '%s' params %08x %08x %08x\n",
			dep->name,
			dwc3_gadget_ep_cmd_string(cmd), params->param0,
			params->param1, params->param2);

	
	hsphy_suspend_enabled = (dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0)) &
						DWC3_GUSB2PHYCFG_SUSPHY);

	
	if (hsphy_suspend_enabled)
		dwc3_gadget_usb2_phy_suspend(dwc, false);

	dwc3_writel(dwc->regs, DWC3_DEPCMDPAR0(ep), params->param0);
	dwc3_writel(dwc->regs, DWC3_DEPCMDPAR1(ep), params->param1);
	dwc3_writel(dwc->regs, DWC3_DEPCMDPAR2(ep), params->param2);

	dwc3_writel(dwc->regs, DWC3_DEPCMD(ep), cmd | DWC3_DEPCMD_CMDACT);
	do {
		reg = dwc3_readl(dwc->regs, DWC3_DEPCMD(ep));
		if (!(reg & DWC3_DEPCMD_CMDACT)) {
			dev_vdbg(dwc->dev, "Command Complete --> %d\n",
					DWC3_DEPCMD_STATUS(reg));
			if (reg & 0x2000)
				ret = -EAGAIN;
			else
				ret = 0;
			break;
		}

		timeout--;
		if (!timeout) {
			ret = -ETIMEDOUT;
			break;
		}

		udelay(1);
	} while (1);

	
	if (hsphy_suspend_enabled)
		dwc3_gadget_usb2_phy_suspend(dwc, true);

	return ret;
}

dma_addr_t dwc3_trb_dma_offset(struct dwc3_ep *dep,
		struct dwc3_trb *trb)
{
	u32		offset = (char *) trb - (char *) dep->trb_pool;

	return dep->trb_pool_dma + offset;
}

static int dwc3_alloc_trb_pool(struct dwc3_ep *dep)
{
	struct dwc3		*dwc = dep->dwc;

	if (dep->trb_pool)
		return 0;

	if (dep->number == 0 || dep->number == 1)
		return 0;

	dep->trb_pool = dma_alloc_coherent(dwc->dev,
			sizeof(struct dwc3_trb) * DWC3_TRB_NUM,
			&dep->trb_pool_dma, GFP_KERNEL);
	if (!dep->trb_pool) {
		dev_err(dep->dwc->dev, "failed to allocate trb pool for %s\n",
				dep->name);
		return -ENOMEM;
	}

	return 0;
}

static void dwc3_free_trb_pool(struct dwc3_ep *dep)
{
	struct dwc3		*dwc = dep->dwc;

	dma_free_coherent(dwc->dev, sizeof(struct dwc3_trb) * DWC3_TRB_NUM,
			dep->trb_pool, dep->trb_pool_dma);

	dep->trb_pool = NULL;
	dep->trb_pool_dma = 0;
}

static int dwc3_gadget_start_config(struct dwc3 *dwc, struct dwc3_ep *dep)
{
	struct dwc3_gadget_ep_cmd_params params;
	u32			cmd;

	memset(&params, 0x00, sizeof(params));

	if (dep->number != 1) {
		cmd = DWC3_DEPCMD_DEPSTARTCFG;
		
		if (dep->number > 1) {
			if (dwc->start_config_issued)
				return 0;
			dwc->start_config_issued = true;
			cmd |= DWC3_DEPCMD_PARAM(2);
		}

		return dwc3_send_gadget_ep_cmd(dwc, 0, cmd, &params);
	}

	return 0;
}

static int dwc3_gadget_set_ep_config(struct dwc3 *dwc, struct dwc3_ep *dep,
		const struct usb_endpoint_descriptor *desc,
		const struct usb_ss_ep_comp_descriptor *comp_desc,
		bool ignore)
{
	struct dwc3_gadget_ep_cmd_params params;

	memset(&params, 0x00, sizeof(params));

	params.param0 = DWC3_DEPCFG_EP_TYPE(usb_endpoint_type(desc))
		| DWC3_DEPCFG_MAX_PACKET_SIZE(usb_endpoint_maxp(desc));

	
	if (dwc->gadget.speed == USB_SPEED_SUPER) {
		u32 burst = dep->endpoint.maxburst - 1;

		params.param0 |= DWC3_DEPCFG_BURST_SIZE(burst);
	}

	if (ignore)
		params.param0 |= DWC3_DEPCFG_IGN_SEQ_NUM;

	params.param1 = DWC3_DEPCFG_XFER_COMPLETE_EN
		| DWC3_DEPCFG_XFER_NOT_READY_EN;

	if (usb_ss_max_streams(comp_desc) && usb_endpoint_xfer_bulk(desc)) {
		params.param1 |= DWC3_DEPCFG_STREAM_CAPABLE
			| DWC3_DEPCFG_STREAM_EVENT_EN;
		dep->stream_capable = true;
	}

	if ((usb_endpoint_xfer_isoc(desc))
	    || ((dep->endpoint.is_ncm) && !usb_endpoint_xfer_control(desc)))
		params.param1 |= DWC3_DEPCFG_XFER_IN_PROGRESS_EN;

	params.param1 |= DWC3_DEPCFG_EP_NUMBER(dep->number);

	if (dep->direction)
		params.param0 |= DWC3_DEPCFG_FIFO_NUMBER(dep->number >> 1);

	if (desc->bInterval) {
		params.param1 |= DWC3_DEPCFG_BINTERVAL_M1(desc->bInterval - 1);
		dep->interval = 1 << (desc->bInterval - 1);
	}

	return dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_SETEPCONFIG, &params);
}

static int dwc3_gadget_set_xfer_resource(struct dwc3 *dwc, struct dwc3_ep *dep)
{
	struct dwc3_gadget_ep_cmd_params params;

	memset(&params, 0x00, sizeof(params));

	params.param0 = DWC3_DEPXFERCFG_NUM_XFER_RES(1);

	return dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_SETTRANSFRESOURCE, &params);
}

static int __dwc3_gadget_ep_enable(struct dwc3_ep *dep,
		const struct usb_endpoint_descriptor *desc,
		const struct usb_ss_ep_comp_descriptor *comp_desc,
		bool ignore)
{
	struct dwc3		*dwc = dep->dwc;
	u32			reg;
	int			ret = -ENOMEM;

	if (!(dep->flags & DWC3_EP_ENABLED)) {
		ret = dwc3_gadget_start_config(dwc, dep);
		if (ret)
			return ret;
	}

	ret = dwc3_gadget_set_ep_config(dwc, dep, desc, comp_desc, ignore);
	if (ret)
		return ret;

	if (!(dep->flags & DWC3_EP_ENABLED)) {
		struct dwc3_trb	*trb_st_hw;
		struct dwc3_trb	*trb_link;

		ret = dwc3_gadget_set_xfer_resource(dwc, dep);
		if (ret)
			return ret;

		dep->endpoint.desc = desc;
		dep->comp_desc = comp_desc;
		dep->type = usb_endpoint_type(desc);
		dep->flags |= DWC3_EP_ENABLED;

		reg = dwc3_readl(dwc->regs, DWC3_DALEPENA);
		reg |= DWC3_DALEPENA_EP(dep->number);
		dwc3_writel(dwc->regs, DWC3_DALEPENA, reg);

        if (dep->endpoint.is_ncm)
            dwc3_gadget_resize_tx_fifos(dwc);

		if (!usb_endpoint_xfer_isoc(desc))
			return 0;

		memset(&trb_link, 0, sizeof(trb_link));

		
		trb_st_hw = &dep->trb_pool[0];

		trb_link = &dep->trb_pool[DWC3_TRB_NUM - 1];

		trb_link->bpl = lower_32_bits(dwc3_trb_dma_offset(dep, trb_st_hw));
		trb_link->bph = upper_32_bits(dwc3_trb_dma_offset(dep, trb_st_hw));
		trb_link->ctrl |= DWC3_TRBCTL_LINK_TRB;
		trb_link->ctrl |= DWC3_TRB_CTRL_HWO;
	}

	return 0;
}

static void dwc3_stop_active_transfer(struct dwc3 *dwc, u32 epnum);
static void dwc3_remove_requests(struct dwc3 *dwc, struct dwc3_ep *dep)
{
	struct dwc3_request		*req;
#if defined(CONFIG_HTC_DEBUG_RTB)
		unsigned long long timestamp, timestamp_ms;
#endif

	if (!list_empty(&dep->req_queued)) {
		dwc3_stop_active_transfer(dwc, dep->number);
#if defined(CONFIG_HTC_DEBUG_RTB)
						timestamp = sched_clock();
						timestamp_ms = timestamp;
						do_div(timestamp_ms, NSEC_PER_MSEC);
						uncached_logk(LOGK_LOGBUF,
							(void *)((unsigned long)timestamp_ms));
#endif

		
		while (!list_empty(&dep->req_queued)) {
			req = next_request(&dep->req_queued);

			dwc3_gadget_giveback(dep, req, -ESHUTDOWN);
		}
	}
#if defined(CONFIG_HTC_DEBUG_RTB)
				timestamp = sched_clock();
				timestamp_ms = timestamp;
				do_div(timestamp_ms, NSEC_PER_MSEC);
				uncached_logk(LOGK_LOGBUF,
					(void *)((unsigned long)timestamp_ms));
#endif
	while (!list_empty(&dep->request_list)) {
		req = next_request(&dep->request_list);

		dwc3_gadget_giveback(dep, req, -ESHUTDOWN);
	}
}

u32 g_debug_fp = 0x0;
static int __dwc3_gadget_ep_disable(struct dwc3_ep *dep)
{
	struct dwc3		*dwc = dep->dwc;
	u32			reg;

#if defined(CONFIG_HTC_DEBUG_RTB)
	unsigned long long timestamp, timestamp_ms;
	timestamp = sched_clock();
	timestamp_ms = timestamp;
	do_div(timestamp_ms, NSEC_PER_MSEC);
	uncached_logk(LOGK_LOGBUF,
		(void *)((unsigned long)timestamp_ms));
#endif

	g_debug_fp = 0xAA;

	dwc3_remove_requests(dwc, dep);
	reg = dwc3_readl(dwc->regs, DWC3_DALEPENA);
	reg &= ~DWC3_DALEPENA_EP(dep->number);
	dwc3_writel(dwc->regs, DWC3_DALEPENA, reg);

	dep->stream_capable = false;
	dep->endpoint.desc = NULL;
	dep->comp_desc = NULL;
	dep->type = 0;
	dep->flags = 0;
	g_debug_fp = 0xBB;
#if defined(CONFIG_HTC_DEBUG_RTB)
	timestamp = sched_clock();
	timestamp_ms = timestamp;
	do_div(timestamp_ms, NSEC_PER_MSEC);
	uncached_logk(LOGK_LOGBUF,
		(void *)((unsigned long)timestamp_ms));
#endif

	return 0;
}

static void dwc3_gadget_ep_nuke(struct usb_ep *ep)
{
	struct dwc3_ep			*dep;
	struct dwc3			*dwc;
	unsigned long flags;
	if (!ep) {
		pr_debug("dwc3: invalid parameters\n");
		return;
	}

	dep = to_dwc3_ep(ep);
	dwc = dep->dwc;
	spin_lock_irqsave(&dwc->lock, flags);
	dwc3_remove_requests(dwc, dep);
	spin_unlock_irqrestore(&dwc->lock, flags);

}


static int dwc3_gadget_ep0_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	return -EINVAL;
}

static int dwc3_gadget_ep0_disable(struct usb_ep *ep)
{
	return -EINVAL;
}


static int dwc3_gadget_ep_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct dwc3_ep			*dep;
	struct dwc3			*dwc;
	unsigned long			flags;
	int				ret;

	if (!ep || !desc || desc->bDescriptorType != USB_DT_ENDPOINT) {
		pr_debug("dwc3: invalid parameters\n");
		return -EINVAL;
	}

	if (!desc->wMaxPacketSize) {
		pr_debug("dwc3: missing wMaxPacketSize\n");
		return -EINVAL;
	}

	dep = to_dwc3_ep(ep);
	dwc = dep->dwc;

	if (dep->flags & DWC3_EP_ENABLED) {
		dev_WARN_ONCE(dwc->dev, true, "%s is already enabled\n",
				dep->name);
		return 0;
	}

	switch (usb_endpoint_type(desc)) {
	case USB_ENDPOINT_XFER_CONTROL:
		strlcat(dep->name, "-control", sizeof(dep->name));
		break;
	case USB_ENDPOINT_XFER_ISOC:
		strlcat(dep->name, "-isoc", sizeof(dep->name));
		break;
	case USB_ENDPOINT_XFER_BULK:
		strlcat(dep->name, "-bulk", sizeof(dep->name));
		break;
	case USB_ENDPOINT_XFER_INT:
		strlcat(dep->name, "-int", sizeof(dep->name));
		break;
	default:
		dev_err(dwc->dev, "invalid endpoint transfer type\n");
	}

	dev_vdbg(dwc->dev, "Enabling %s\n", dep->name);

	spin_lock_irqsave(&dwc->lock, flags);
#ifdef CONFIG_HTC_USB_DEBUG_FLAG
	dbg_event(0xFF, "ENABLE", dep->number);
#endif
	ret = __dwc3_gadget_ep_enable(dep, desc, ep->comp_desc, false);
#ifdef CONFIG_HTC_USB_DEBUG_FLAG
	dbg_event(0xFF, "ENAEND", dep->number);
#else
	dbg_event(dep->number, "ENABLE", ret);
#endif
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_ep_disable(struct usb_ep *ep)
{
	struct dwc3_ep			*dep;
	struct dwc3			*dwc;
	unsigned long			flags;
	int				ret;

	if (!ep) {
		pr_debug("dwc3: invalid parameters\n");
		return -EINVAL;
	}

	dep = to_dwc3_ep(ep);
	dwc = dep->dwc;

	if (!(dep->flags & DWC3_EP_ENABLED)) {
		dev_WARN_ONCE(dwc->dev, true, "%s is already disabled\n",
				dep->name);
		return 0;
	}

	snprintf(dep->name, sizeof(dep->name), "ep%d%s",
			dep->number >> 1,
			(dep->number & 1) ? "in" : "out");

	spin_lock_irqsave(&dwc->lock, flags);
#ifdef CONFIG_HTC_USB_DEBUG_FLAG
	dbg_event(0xFF, "DISABLE", dep->number);
#endif
	ret = __dwc3_gadget_ep_disable(dep);

#ifdef CONFIG_HTC_USB_DEBUG_FLAG
	dbg_event(0xFF, "DIS END", dep->number);
#else
	dbg_event(dep->number, "DISABLE", ret);
#endif
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static struct usb_request *dwc3_gadget_ep_alloc_request(struct usb_ep *ep,
	gfp_t gfp_flags)
{
	struct dwc3_request		*req;
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req) {
		dev_err(dwc->dev, "not enough memory\n");
		return NULL;
	}

	req->epnum	= dep->number;
	req->dep	= dep;

	return &req->request;
}

static void dwc3_gadget_ep_free_request(struct usb_ep *ep,
		struct usb_request *request)
{
	struct dwc3_request		*req = to_dwc3_request(request);

	kfree(req);
}

static void dwc3_prepare_one_trb(struct dwc3_ep *dep,
		struct dwc3_request *req, dma_addr_t dma,
		unsigned length, unsigned last, unsigned chain)
{
	struct dwc3		*dwc = dep->dwc;
	struct dwc3_trb		*trb;

	unsigned int		cur_slot;

	dev_vdbg(dwc->dev, "%s: req %p dma %08llx length %d%s%s\n",
			dep->name, req, (unsigned long long) dma,
			length, last ? " last" : "",
			chain ? " chain" : "");

	trb = &dep->trb_pool[dep->free_slot & DWC3_TRB_MASK];
	cur_slot = dep->free_slot;
	dep->free_slot++;

	
	if (((dep->free_slot & DWC3_TRB_MASK) == DWC3_TRB_NUM - 1) &&
			usb_endpoint_xfer_isoc(dep->endpoint.desc))
		dep->free_slot++;

	if (!req->trb) {
		dwc3_gadget_move_request_queued(req);
		req->trb = trb;
		req->trb_dma = dwc3_trb_dma_offset(dep, trb);
	}

update_trb:
	trb->size = DWC3_TRB_SIZE_LENGTH(length);
	trb->bpl = lower_32_bits(dma);
	trb->bph = upper_32_bits(dma);

	switch (usb_endpoint_type(dep->endpoint.desc)) {
	case USB_ENDPOINT_XFER_CONTROL:
		trb->ctrl = DWC3_TRBCTL_CONTROL_SETUP;
		break;

	case USB_ENDPOINT_XFER_ISOC:
		trb->ctrl = DWC3_TRBCTL_ISOCHRONOUS_FIRST;

		if (!req->request.no_interrupt)
			trb->ctrl |= DWC3_TRB_CTRL_IOC;
		break;

	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		trb->ctrl = DWC3_TRBCTL_NORMAL;
		break;
	default:
		BUG();
	}

	if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
		trb->ctrl |= DWC3_TRB_CTRL_ISP_IMI;
		trb->ctrl |= DWC3_TRB_CTRL_CSP;
	} else {
		if (chain)
			trb->ctrl |= DWC3_TRB_CTRL_CHN;
        if (dep->endpoint.is_ncm)
            trb->ctrl |= DWC3_TRB_CTRL_CSP;
	}

	if (usb_endpoint_xfer_bulk(dep->endpoint.desc) && dep->stream_capable)
		trb->ctrl |= DWC3_TRB_CTRL_SID_SOFN(req->request.stream_id);

	trb->ctrl |= DWC3_TRB_CTRL_HWO;

	if (req->request.zero && length &&
			(length % usb_endpoint_maxp(dep->endpoint.desc) == 0)) {
		trb = &dep->trb_pool[dep->free_slot & DWC3_TRB_MASK];
		dep->free_slot++;

		
		if (((dep->free_slot & DWC3_TRB_MASK) == DWC3_TRB_NUM - 1) &&
			usb_endpoint_xfer_isoc(dep->endpoint.desc))
			dep->free_slot++;

		req->ztrb = trb;
		length = 0;

		goto update_trb;
	}

	if (!usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
		if (last)
			trb->ctrl |= DWC3_TRB_CTRL_LST;
		else if (dep->endpoint.is_ncm && (!req->request.no_interrupt) && (dep->direction != 1))
			trb->ctrl |= DWC3_TRB_CTRL_IOC;
	}
}

static void dwc3_prepare_trbs(struct dwc3_ep *dep, bool starting)
{
	struct dwc3_request	*req, *n;
	u32			trbs_left;
	u32			max;
	unsigned int		last_one = 0;

	BUILD_BUG_ON_NOT_POWER_OF_2(DWC3_TRB_NUM);

	
	trbs_left = (dep->busy_slot - dep->free_slot) & DWC3_TRB_MASK;

	
	if (!usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
		max = DWC3_TRB_NUM - (dep->free_slot & DWC3_TRB_MASK);
		if (trbs_left > max)
			trbs_left = max;
	}

	if (!trbs_left) {
		if (!starting)
			return;
		trbs_left = DWC3_TRB_NUM;
		if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			dep->busy_slot = 1;
			dep->free_slot = 1;
		} else {
			dep->busy_slot = 0;
			dep->free_slot = 0;
		}
	}

	
	if ((trbs_left <= 1) && usb_endpoint_xfer_isoc(dep->endpoint.desc))
		return;

	list_for_each_entry_safe(req, n, &dep->request_list, list) {
		unsigned	length;
		dma_addr_t	dma;

		if (req->request.num_mapped_sgs > 0) {
			struct usb_request *request = &req->request;
			struct scatterlist *sg = request->sg;
			struct scatterlist *s;
			int		i;

			for_each_sg(sg, s, request->num_mapped_sgs, i) {
				unsigned chain = true;

				length = sg_dma_len(s);
				dma = sg_dma_address(s);

				if (i == (request->num_mapped_sgs - 1) ||
						sg_is_last(s)) {
					last_one = true;
					chain = false;
				}

				trbs_left--;
				if (!trbs_left)
					last_one = true;

				if (last_one)
					chain = false;

				dwc3_prepare_one_trb(dep, req, dma, length,
						last_one, chain);

				if (last_one)
					break;
			}
			dbg_queue(dep->number, &req->request, 0);
		} else {
			struct dwc3_request	*req1;
			int maxpkt_size = usb_endpoint_maxp(dep->endpoint.desc);

			dma = req->request.dma;
			length = req->request.length;
			trbs_left--;

			if (req->request.zero && length &&
						(length % maxpkt_size == 0))
				trbs_left--;

			if (!trbs_left) {
				last_one = 1;
			} else if (dep->direction && (trbs_left <= 1)) {
				req1 = next_request(&req->list);
				if (req1->request.zero && req1->request.length
				 && (req1->request.length % maxpkt_size == 0))
					last_one = 1;
			}

			
			if (list_is_last(&req->list, &dep->request_list))
				last_one = 1;

			dwc3_prepare_one_trb(dep, req, dma, length,
					last_one, false);

			dbg_queue(dep->number, &req->request, 0);
			if (last_one)
				break;
		}
	}
}

static int __dwc3_gadget_kick_transfer(struct dwc3_ep *dep, u16 cmd_param,
		int start_new)
{
	struct dwc3_gadget_ep_cmd_params params;
	struct dwc3_request		*req, *req1, *n;
	struct dwc3			*dwc = dep->dwc;
	int				ret;
	u32				cmd;

	if (start_new && (dep->flags & DWC3_EP_BUSY)) {
		dev_vdbg(dwc->dev, "%s: endpoint busy\n", dep->name);
		return -EBUSY;
	}
	dep->flags &= ~DWC3_EP_PENDING_REQUEST;

	if (start_new) {
		if (list_empty(&dep->req_queued))
			dwc3_prepare_trbs(dep, start_new);

		
		req = next_request(&dep->req_queued);
	} else {
		dwc3_prepare_trbs(dep, start_new);

		req = next_request(&dep->req_queued);
	}
	if (!req) {
		dep->flags |= DWC3_EP_PENDING_REQUEST;
#ifdef CONFIG_HTC_USB_DEBUG_FLAG
		dbg_event(0xFF, "NO REQ", dep->number);
#else
		dbg_event(dep->number, "NO REQ", 0);
#endif
		return 0;
	}

	memset(&params, 0, sizeof(params));
	params.param0 = upper_32_bits(req->trb_dma);
	params.param1 = lower_32_bits(req->trb_dma);

	if (start_new)
		cmd = DWC3_DEPCMD_STARTTRANSFER;
	else
		cmd = DWC3_DEPCMD_UPDATETRANSFER;

	cmd |= DWC3_DEPCMD_PARAM(cmd_param);
	ret = dwc3_send_gadget_ep_cmd(dwc, dep->number, cmd, &params);
	if (ret < 0) {
		dev_dbg(dwc->dev, "failed to send STARTTRANSFER command\n");
#ifdef CONFIG_HTC_USB_DEBUG_FLAG
		pr_info("failed to send STARTTRANSFER command\n");
#endif
		if ((ret == -EAGAIN) && start_new &&
				usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			if (!dep->resource_index) {
				dep->resource_index =
					 dwc3_gadget_ep_get_transfer_index(dwc,
								dep->number);
				WARN_ON_ONCE(!dep->resource_index);
			}
			dwc3_stop_active_transfer(dwc, dep->number);
			list_for_each_entry_safe_reverse(req1, n,
						 &dep->req_queued, list) {
				req1->trb = NULL;
				dwc3_gadget_move_request_list_front(req1);
				if (req->request.num_mapped_sgs)
					dep->busy_slot +=
						 req->request.num_mapped_sgs;
				else
					dep->busy_slot++;
				if ((dep->busy_slot & DWC3_TRB_MASK) ==
							DWC3_TRB_NUM - 1)
					dep->busy_slot++;
			}
			return ret;
		} else {
			usb_gadget_unmap_request(&dwc->gadget, &req->request,
				req->direction);
			list_del(&req->list);
			return ret;
		}
	}

	dep->flags |= DWC3_EP_BUSY;

	if (start_new) {
		dep->resource_index = dwc3_gadget_ep_get_transfer_index(dwc,
				dep->number);
		WARN_ON_ONCE(!dep->resource_index);
	}

	return 0;
}

static void __dwc3_gadget_start_isoc(struct dwc3 *dwc,
		struct dwc3_ep *dep, u32 cur_uf)
{
	u32 uf;
	int ret;

	dep->current_uf = cur_uf;

	if (list_empty(&dep->request_list)) {
		dev_vdbg(dwc->dev, "ISOC ep %s run out for requests.\n",
			dep->name);
		dep->flags |= DWC3_EP_PENDING_REQUEST;
		return;
	}

	
	uf = cur_uf + dep->interval * 4;

	ret = __dwc3_gadget_kick_transfer(dep, uf, 1);
	if (ret < 0)
		dbg_event(dep->number, "QUEUE", ret);
}

static void dwc3_gadget_start_isoc(struct dwc3 *dwc,
		struct dwc3_ep *dep, const struct dwc3_event_depevt *event)
{
	u32 cur_uf, mask;

	mask = ~(dep->interval - 1);
	cur_uf = event->parameters & mask;

	__dwc3_gadget_start_isoc(dwc, dep, cur_uf);
}

static int __dwc3_gadget_ep_queue(struct dwc3_ep *dep, struct dwc3_request *req)
{
	struct dwc3		*dwc = dep->dwc;
	int			ret;

	if (req->request.status == -EINPROGRESS) {
		ret = -EBUSY;
		dev_err(dwc->dev, "%s: %p request already in queue",
					dep->name, req);
		return ret;
	}

	req->request.actual	= 0;
	req->request.status	= -EINPROGRESS;
	req->direction		= dep->direction;
	req->epnum		= dep->number;

	ret = usb_gadget_map_request(&dwc->gadget, &req->request,
			dep->direction);
	if (ret)
		return ret;

	uncached_logk(LOGK_LOGBUF,
		(void *)((unsigned long)dep->number));

	list_add_tail(&req->list, &dep->request_list);

	if (dep->flags & DWC3_EP_PENDING_REQUEST) {
		int	ret;

		if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			if (list_empty(&dep->req_queued) && dep->resource_index)
				dwc3_stop_active_transfer(dwc, dep->number);
			else
				__dwc3_gadget_start_isoc(dwc, dep,
							dep->current_uf);
			dep->flags &= ~DWC3_EP_PENDING_REQUEST;
			return 0;
		}

		ret = __dwc3_gadget_kick_transfer(dep, 0, true);
		if (ret && ret != -EBUSY) {
			dbg_event(dep->number, "QUEUE", ret);
			dev_dbg(dwc->dev, "%s: failed to kick transfers\n",
					dep->name);
		}
	}

	if (usb_endpoint_xfer_isoc(dep->endpoint.desc) &&
			(dep->flags & DWC3_EP_BUSY) &&
			!(dep->flags & DWC3_EP_MISSED_ISOC)) {
		WARN_ON_ONCE(!dep->resource_index);
		ret = __dwc3_gadget_kick_transfer(dep, dep->resource_index,
				false);
		if (ret && ret != -EBUSY) {
			dbg_event(dep->number, "QUEUE", ret);
			dev_dbg(dwc->dev, "%s: failed to kick transfers\n",
					dep->name);
		}
	}

	return 0;
}

static int dwc3_gadget_ep_queue(struct usb_ep *ep, struct usb_request *request,
	gfp_t gfp_flags)
{
	struct dwc3_request		*req = to_dwc3_request(request);
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	unsigned long			flags;

	int				ret;

	spin_lock_irqsave(&dwc->lock, flags);

	if (!dep->endpoint.desc) {
		spin_unlock_irqrestore(&dwc->lock, flags);
		dev_dbg(dwc->dev, "trying to queue request %p to disabled %s\n",
				request, ep->name);
		return -ESHUTDOWN;
	}

	dev_vdbg(dwc->dev, "queing request %p to %s length %d\n",
			request, ep->name, request->length);

	WARN(!dep->direction && (request->length % ep->desc->wMaxPacketSize),
		"trying to queue unaligned request (%d)\n", request->length);

	ret = __dwc3_gadget_ep_queue(dep, req);
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_ep_dequeue(struct usb_ep *ep,
		struct usb_request *request)
{
	struct dwc3_request		*req = to_dwc3_request(request);
	struct dwc3_request		*r = NULL;

	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	unsigned long			flags;
	int				ret = 0;

	spin_lock_irqsave(&dwc->lock, flags);

	list_for_each_entry(r, &dep->request_list, list) {
		if (r == req)
			break;
	}

	if (r != req) {
		list_for_each_entry(r, &dep->req_queued, list) {
			if (r == req)
				break;
		}
		if (r == req) {
			
			dwc3_stop_active_transfer(dwc, dep->number);
			goto out1;
		}
		dev_err(dwc->dev, "request %p was not queued to %s\n",
				request, ep->name);
		ret = -EINVAL;
		goto out0;
	}

out1:
	dbg_event(dep->number, "DEQUEUE", 0);
	
	dwc3_gadget_giveback(dep, req, -ECONNRESET);

out0:
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

int __dwc3_gadget_ep_set_halt(struct dwc3_ep *dep, int value)
{
	struct dwc3_gadget_ep_cmd_params	params;
	struct dwc3				*dwc = dep->dwc;
	int					ret;

	memset(&params, 0x00, sizeof(params));

	if (value) {
		ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_SETSTALL, &params);
		if (ret)
			dev_err(dwc->dev, "failed to %s STALL on %s\n",
					value ? "set" : "clear",
					dep->name);
		else
			dep->flags |= DWC3_EP_STALL;
	} else {
		ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_CLEARSTALL, &params);
		if (ret)
			dev_err(dwc->dev, "failed to %s STALL on %s\n",
					value ? "set" : "clear",
					dep->name);
		else
			dep->flags &= ~(DWC3_EP_STALL | DWC3_EP_WEDGE);
	}

	return ret;
}

static int dwc3_gadget_ep_set_halt(struct usb_ep *ep, int value)
{
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	unsigned long			flags;

	int				ret;

	spin_lock_irqsave(&dwc->lock, flags);

	if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
		dev_err(dwc->dev, "%s is of Isochronous type\n", dep->name);
		ret = -EINVAL;
		goto out;
	}

	dbg_event(dep->number, "HALT", value);
	ret = __dwc3_gadget_ep_set_halt(dep, value);
out:
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_ep_set_wedge(struct usb_ep *ep)
{
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;
	unsigned long			flags;

	spin_lock_irqsave(&dwc->lock, flags);
	dbg_event(dep->number, "WEDGE", 0);
	dep->flags |= DWC3_EP_WEDGE;
	spin_unlock_irqrestore(&dwc->lock, flags);

	if (dep->number == 0 || dep->number == 1)
		return dwc3_gadget_ep0_set_halt(ep, 1);
	else
		return dwc3_gadget_ep_set_halt(ep, 1);
}


static struct usb_endpoint_descriptor dwc3_gadget_ep0_desc = {
	.bLength	= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bmAttributes	= USB_ENDPOINT_XFER_CONTROL,
};

static const struct usb_ep_ops dwc3_gadget_ep0_ops = {
	.enable		= dwc3_gadget_ep0_enable,
	.disable	= dwc3_gadget_ep0_disable,
	.alloc_request	= dwc3_gadget_ep_alloc_request,
	.free_request	= dwc3_gadget_ep_free_request,
	.queue		= dwc3_gadget_ep0_queue,
	.dequeue	= dwc3_gadget_ep_dequeue,
	.set_halt	= dwc3_gadget_ep0_set_halt,
	.set_wedge	= dwc3_gadget_ep_set_wedge,
};

static const struct usb_ep_ops dwc3_gadget_ep_ops = {
	.enable		= dwc3_gadget_ep_enable,
	.disable	= dwc3_gadget_ep_disable,
	.alloc_request	= dwc3_gadget_ep_alloc_request,
	.free_request	= dwc3_gadget_ep_free_request,
	.queue		= dwc3_gadget_ep_queue,
	.dequeue	= dwc3_gadget_ep_dequeue,
	.set_halt	= dwc3_gadget_ep_set_halt,
	.set_wedge	= dwc3_gadget_ep_set_wedge,
	.nuke		= dwc3_gadget_ep_nuke,
};


static int dwc3_gadget_get_frame(struct usb_gadget *g)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	u32			reg;

	reg = dwc3_readl(dwc->regs, DWC3_DSTS);
	return DWC3_DSTS_SOFFN(reg);
}

static int dwc3_gadget_wakeup(struct usb_gadget *g)
{
	struct dwc3		*dwc = gadget_to_dwc(g);

	unsigned long		timeout;
	unsigned long		flags;

	u32			reg;

	int			ret = 0;

	u8			link_state;
	u8			speed;

	spin_lock_irqsave(&dwc->lock, flags);

	reg = dwc3_readl(dwc->regs, DWC3_DSTS);

	speed = reg & DWC3_DSTS_CONNECTSPD;
	if (speed == DWC3_DSTS_SUPERSPEED) {
		dev_dbg(dwc->dev, "no wakeup on SuperSpeed\n");
		ret = -EINVAL;
		goto out;
	}

	link_state = DWC3_DSTS_USBLNKST(reg);

	switch (link_state) {
	case DWC3_LINK_STATE_RX_DET:	
	case DWC3_LINK_STATE_U3:	
		break;
	default:
		dev_dbg(dwc->dev, "can't wakeup from link state %d\n",
				link_state);
		ret = -EINVAL;
		goto out;
	}

	ret = dwc3_gadget_set_link_state(dwc, DWC3_LINK_STATE_RECOV);
	if (ret < 0) {
		dev_err(dwc->dev, "failed to put link in Recovery\n");
		goto out;
	}

	
	if (dwc->revision < DWC3_REVISION_194A) {
		
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg &= ~DWC3_DCTL_ULSTCHNGREQ_MASK;
		dwc3_writel(dwc->regs, DWC3_DCTL, reg);
	}

	
	timeout = jiffies + msecs_to_jiffies(100);

	while (!time_after(jiffies, timeout)) {
		reg = dwc3_readl(dwc->regs, DWC3_DSTS);

		
		if (DWC3_DSTS_USBLNKST(reg) == DWC3_LINK_STATE_U0)
			break;
	}

	if (DWC3_DSTS_USBLNKST(reg) != DWC3_LINK_STATE_U0) {
		dev_err(dwc->dev, "failed to send remote wakeup\n");
		ret = -EINVAL;
	}

out:
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_set_selfpowered(struct usb_gadget *g,
		int is_selfpowered)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	unsigned long		flags;

	spin_lock_irqsave(&dwc->lock, flags);
	dwc->is_selfpowered = !!is_selfpowered;
	spin_unlock_irqrestore(&dwc->lock, flags);

	return 0;
}

#define DWC3_SOFT_RESET_TIMEOUT	10  
static int dwc3_gadget_run_stop(struct dwc3 *dwc, int is_on)
{
	u32			reg;
	u32			timeout = 500;
	ktime_t start, diff;

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	if (is_on) {
		if (dwc->revision <= DWC3_REVISION_187A) {
			reg &= ~DWC3_DCTL_TRGTULST_MASK;
			reg |= DWC3_DCTL_TRGTULST_RX_DET;
		}

		if (dwc->revision >= DWC3_REVISION_194A)
			reg &= ~DWC3_DCTL_KEEP_CONNECT;

		start = ktime_get();
		
		dwc3_writel(dwc->regs, DWC3_DCTL, reg | DWC3_DCTL_CSFTRST);
		do {
			reg = dwc3_readl(dwc->regs, DWC3_DCTL);
			if (!(reg & DWC3_DCTL_CSFTRST))
				break;

			diff = ktime_sub(ktime_get(), start);
			
			if (ktime_to_ms(diff) > DWC3_SOFT_RESET_TIMEOUT) {
				printk_ratelimited(KERN_ERR
					"%s:core Reset Timed Out\n", __func__);
				break;
			}
			cpu_relax();
		} while (true);

		dwc3_event_buffers_setup(dwc);
		dwc3_gadget_restart(dwc);
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg |= DWC3_DCTL_RUN_STOP;
	} else {
		reg &= ~DWC3_DCTL_RUN_STOP;
	}

	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	do {
		reg = dwc3_readl(dwc->regs, DWC3_DSTS);
		if (is_on) {
			if (!(reg & DWC3_DSTS_DEVCTRLHLT))
				break;
		} else {
			if (reg & DWC3_DSTS_DEVCTRLHLT)
				break;
		}
		timeout--;
		if (!timeout)
			return -ETIMEDOUT;
		udelay(1);
	} while (1);

	dev_vdbg(dwc->dev, "gadget %s data soft-%s\n",
			dwc->gadget_driver
			? dwc->gadget_driver->function : "no-function",
			is_on ? "connect" : "disconnect");
#ifdef CONFIG_HTC_USB_DEBUG_FLAG
	pr_info("gadget %s data soft-%s\n",
			dwc->gadget_driver
			? dwc->gadget_driver->function : "no-functio",
			is_on ? "connect" : "disconnect");
#endif
	return 0;
}

static int dwc3_gadget_vbus_draw(struct usb_gadget *g, unsigned mA)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	struct dwc3_otg		*dotg = dwc->dotg;

	if (dotg && dotg->otg.phy)
		return usb_phy_set_power(dotg->otg.phy, mA);

	return -ENOTSUPP;
}

static int dwc3_gadget_pullup(struct usb_gadget *g, int is_on)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	unsigned long		flags;
	int			ret;

	is_on = !!is_on;
	printk(KERN_INFO "[USB] %s %d\n",__func__,is_on);

	
	g->ats_reset_irq_count	= 0;

	if (is_on)
		enable_irq(g_irq);
	else
		disable_irq(g_irq);

	spin_lock_irqsave(&dwc->lock, flags);

	dwc->softconnect = is_on;

	if ((dwc->dotg && !dwc->vbus_active) ||
		!dwc->gadget_driver) {

		spin_unlock_irqrestore(&dwc->lock, flags);

		return 0;
	}
#ifdef CONFIG_HTC_USB_DEBUG_FLAG
	dbg_event(0xFF, "RUNSTOP",0);
#endif
	ret = dwc3_gadget_run_stop(dwc, is_on);
#ifdef CONFIG_HTC_USB_DEBUG_FLAG
	dbg_event(0xFF, "STOPEND",ret);
#endif
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static void dwc3_gadget_disconnect_interrupt(struct dwc3 *dwc);

static int dwc3_gadget_vbus_session(struct usb_gadget *_gadget, int is_active)
{
	struct dwc3 *dwc = gadget_to_dwc(_gadget);
	unsigned long flags;
	int ret = 0;
	printk(KERN_INFO "[USB] %s dwc->dotg %x ,gadge driver %d ,softconnect %d,active %d\n",__func__, (unsigned int)dwc->dotg, (unsigned int)dwc->gadget_driver,dwc->softconnect,is_active);
	if (!dwc->dotg)
		return -EPERM;

	is_active = !!is_active;

	spin_lock_irqsave(&dwc->lock, flags);

	
	dwc->vbus_active = is_active;

	
	_gadget->ats_reset_irq_count	= 0;

	if (dwc->gadget_driver && dwc->softconnect) {
		if (dwc->vbus_active) {
			ret = dwc3_gadget_run_stop(dwc, 1);
		} else {
			ret = dwc3_gadget_run_stop(dwc, 0);
		}
	}

	dwc->start_config_issued = true;
	printk("%s: dwc->vbus_active= %d, dwc->start_config_issued= %d\n",
		__func__, dwc->vbus_active, dwc->start_config_issued);

	if (!dwc->vbus_active && dwc->start_config_issued) {
		dev_dbg(dwc->dev, "calling disconnect from %s\n", __func__);
		dwc3_gadget_disconnect_interrupt(dwc);
	}
	printk("%s: disconnect_interrupt finish!\n", __func__);
	spin_unlock_irqrestore(&dwc->lock, flags);
	return ret;
}

void dwc3_gadget_restart(struct dwc3 *dwc)
{
	struct dwc3_ep		*dep;
	int			ret = 0;
	u32			reg;

	
	reg = (DWC3_DEVTEN_EVNTOVERFLOWEN |
			DWC3_DEVTEN_CMDCMPLTEN |
			DWC3_DEVTEN_ERRTICERREN |
			DWC3_DEVTEN_WKUPEVTEN |
			DWC3_DEVTEN_ULSTCNGEN |
			DWC3_DEVTEN_CONNECTDONEEN |
			DWC3_DEVTEN_USBRSTEN |
			DWC3_DEVTEN_DISCONNEVTEN);
	dwc3_writel(dwc->regs, DWC3_DEVTEN, reg);

	
	if (dwc->revision >= DWC3_REVISION_194A) {
		reg = dwc3_readl(dwc->regs, DWC3_DCFG);
		reg |= DWC3_DCFG_LPM_CAP;
		dwc3_writel(dwc->regs, DWC3_DCFG, reg);

		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg &= ~(DWC3_DCTL_HIRD_THRES_MASK | DWC3_DCTL_L1_HIBER_EN);

		
		reg |= DWC3_DCTL_HIRD_THRES(28);

		dwc3_writel(dwc->regs, DWC3_DCTL, reg);

		dwc3_gadget_usb2_phy_suspend(dwc, true);
		dwc3_gadget_usb3_phy_suspend(dwc, true);
	}

	reg = dwc3_readl(dwc->regs, DWC3_DCFG);
	reg &= ~(DWC3_DCFG_SPEED_MASK);

	if (dwc->revision < DWC3_REVISION_220A)
		reg |= DWC3_DCFG_SUPERSPEED;
	else
		reg |= dwc->maximum_speed;
	dwc3_writel(dwc->regs, DWC3_DCFG, reg);

	dwc->start_config_issued = false;

	
	dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(512);

	dwc->delayed_status = false;
	
	dep = dwc->eps[0];
	dep->flags = 0;
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, false);
	if (ret) {
		dev_err(dwc->dev, "failed to enable %s\n", dep->name);
		return;
	}

	dep = dwc->eps[1];
	dep->flags = 0;
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, false);
	if (ret) {
		dev_err(dwc->dev, "failed to enable %s\n", dep->name);
		return;
	}

	
	dwc->ep0state = EP0_SETUP_PHASE;
	dwc3_ep0_out_start(dwc);
}

static int dwc3_gadget_start(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	struct dwc3_ep		*dep;
	unsigned long		flags;
	int			ret = 0;
	u32			reg;

	pm_runtime_get_sync(dwc->dev);
	spin_lock_irqsave(&dwc->lock, flags);

	if (dwc->gadget_driver) {
		dev_err(dwc->dev, "%s is already bound to %s\n",
				dwc->gadget.name,
				dwc->gadget_driver->driver.name);
		ret = -EBUSY;
		goto err0;
	}

	dwc->gadget_driver	= driver;
	dwc->gadget.dev.driver	= &driver->driver;

	reg = dwc3_readl(dwc->regs, DWC3_DCFG);
	reg &= ~(DWC3_DCFG_SPEED_MASK);

	if (dwc->revision < DWC3_REVISION_220A)
		reg |= DWC3_DCFG_SUPERSPEED;
	else
		reg |= dwc->maximum_speed;
	dwc3_writel(dwc->regs, DWC3_DCFG, reg);

	dwc->start_config_issued = false;

	
	dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(512);

	dep = dwc->eps[0];
	dep->endpoint.maxburst = 1;
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, false);
	if (ret) {
		dev_err(dwc->dev, "failed to enable %s\n", dep->name);
		goto err0;
	}

	dep = dwc->eps[1];
	dep->endpoint.maxburst = 1;
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, false);
	if (ret) {
		dev_err(dwc->dev, "failed to enable %s\n", dep->name);
		goto err1;
	}

	
	dwc->ep0state = EP0_SETUP_PHASE;
	dwc3_ep0_out_start(dwc);

	spin_unlock_irqrestore(&dwc->lock, flags);
	pm_runtime_put(dwc->dev);

	return 0;

err1:
	__dwc3_gadget_ep_disable(dwc->eps[0]);

err0:
	spin_unlock_irqrestore(&dwc->lock, flags);
	pm_runtime_put(dwc->dev);

	return ret;
}

static int dwc3_gadget_stop(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	unsigned long		flags;

	spin_lock_irqsave(&dwc->lock, flags);

	__dwc3_gadget_ep_disable(dwc->eps[0]);
	__dwc3_gadget_ep_disable(dwc->eps[1]);

	dwc->gadget_driver	= NULL;
	dwc->gadget.dev.driver	= NULL;

	spin_unlock_irqrestore(&dwc->lock, flags);

	return 0;
}

static const struct usb_gadget_ops dwc3_gadget_ops = {
	.get_frame		= dwc3_gadget_get_frame,
	.wakeup			= dwc3_gadget_wakeup,
	.set_selfpowered	= dwc3_gadget_set_selfpowered,
	.vbus_session		= dwc3_gadget_vbus_session,
	.vbus_draw		= dwc3_gadget_vbus_draw,
	.pullup			= dwc3_gadget_pullup,
	.udc_start		= dwc3_gadget_start,
	.udc_stop		= dwc3_gadget_stop,
};


static int __devinit dwc3_gadget_init_endpoints(struct dwc3 *dwc)
{
	struct dwc3_ep			*dep;
	u8				epnum;

	INIT_LIST_HEAD(&dwc->gadget.ep_list);

	for (epnum = 0; epnum < DWC3_ENDPOINTS_NUM; epnum++) {
		dep = kzalloc(sizeof(*dep), GFP_KERNEL);
		if (!dep) {
			dev_err(dwc->dev, "can't allocate endpoint %d\n",
					epnum);
			return -ENOMEM;
		}

		dep->dwc = dwc;
		dep->number = epnum;
		dwc->eps[epnum] = dep;

		snprintf(dep->name, sizeof(dep->name), "ep%d%s", epnum >> 1,
				(epnum & 1) ? "in" : "out");
		dep->endpoint.name = dep->name;
		dep->direction = (epnum & 1);

		if (epnum == 0 || epnum == 1) {
			dep->endpoint.maxpacket = 512;
			dep->endpoint.ops = &dwc3_gadget_ep0_ops;
			if (!epnum)
				dwc->gadget.ep0 = &dep->endpoint;
		} else {
			int		ret;

			dep->endpoint.maxpacket = 1024;
			dep->endpoint.max_streams = 15;
			dep->endpoint.ops = &dwc3_gadget_ep_ops;
			list_add_tail(&dep->endpoint.ep_list,
					&dwc->gadget.ep_list);

			ret = dwc3_alloc_trb_pool(dep);
			if (ret)
				return ret;
		}

		INIT_LIST_HEAD(&dep->request_list);
		INIT_LIST_HEAD(&dep->req_queued);
	}

	return 0;
}

static void dwc3_gadget_free_endpoints(struct dwc3 *dwc)
{
	struct dwc3_ep			*dep;
	u8				epnum;

	for (epnum = 0; epnum < DWC3_ENDPOINTS_NUM; epnum++) {
		dep = dwc->eps[epnum];
		dwc3_free_trb_pool(dep);

		if (epnum != 0 && epnum != 1)
			list_del(&dep->endpoint.ep_list);

		kfree(dep);
	}
}

static void dwc3_gadget_release(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);
}

static int dwc3_cleanup_done_reqs(struct dwc3 *dwc, struct dwc3_ep *dep,
		const struct dwc3_event_depevt *event, int status)
{
	struct dwc3_request	*req;
	struct dwc3_trb		*trb;
	unsigned int		count;
	unsigned int		s_pkt = 0;
	unsigned int		trb_status;

	do {
		req = next_request(&dep->req_queued);
		if (!req) {
			WARN_ON_ONCE(1);
			return 1;
		}

		trb = req->trb;

		if ((trb->ctrl & DWC3_TRB_CTRL_HWO) && status != -ESHUTDOWN)
			/*
			 * We continue despite the error. There is not much we
			 * can do. If we don't clean it up we loop forever. If
			 * we skip the TRB then it gets overwritten after a
			 * while since we use them in a ring buffer. A BUG()
			 * would help. Lets hope that if this occurs, someone
			 * fixes the root cause instead of looking away :)
			 */
			dev_err(dwc->dev, "%s's TRB (%p) still owned by HW\n",
					dep->name, req->trb);
		count = trb->size & DWC3_TRB_SIZE_MASK;

		if (dep->direction) {
			if (count) {
				trb_status = DWC3_TRB_SIZE_TRBSTS(trb->size);
				if (trb_status == DWC3_TRBSTS_MISSED_ISOC) {
					dev_dbg(dwc->dev, "incomplete IN transfer %s\n",
							dep->name);
					dep->flags |= DWC3_EP_MISSED_ISOC;
					dbg_event(dep->number, "MISSED ISOC",
									status);
				} else {
					dev_err(dwc->dev, "incomplete IN transfer %s\n",
							dep->name);
					status = -ECONNRESET;
				}
			} else {
				dep->flags &= ~DWC3_EP_MISSED_ISOC;
			}
		} else {
			if (count && (event->status & DEPEVT_STATUS_SHORT))
				s_pkt = 1;
		}

		if (req->ztrb)
			trb = req->ztrb;
		req->request.actual += req->request.length - count;
		dwc3_gadget_giveback(dep, req, status);
		if (s_pkt)
			break;
		if ((event->status & DEPEVT_STATUS_LST) &&
				(trb->ctrl & (DWC3_TRB_CTRL_LST |
						DWC3_TRB_CTRL_HWO)))
			break;
		if ((event->status & DEPEVT_STATUS_IOC) &&
				(trb->ctrl & DWC3_TRB_CTRL_IOC))
			break;
	} while (1);
	if (!dep->endpoint.desc) {
		pr_info("endpoint.desc is null\n");
		return 1;
	}

	dwc->gadget.xfer_isr_count++;

	if (usb_endpoint_xfer_isoc(dep->endpoint.desc) &&
			list_empty(&dep->req_queued)) {
		if (list_empty(&dep->request_list))
			dep->flags |= DWC3_EP_PENDING_REQUEST;
		else
			dwc3_stop_active_transfer(dwc, dep->number);
		dep->flags &= ~DWC3_EP_MISSED_ISOC;
		return 1;
	}

	if ((event->status & DEPEVT_STATUS_IOC) &&
			(trb->ctrl & DWC3_TRB_CTRL_IOC))
		return 0;
	return 1;
}

static void dwc3_endpoint_transfer_complete(struct dwc3 *dwc,
		struct dwc3_ep *dep, const struct dwc3_event_depevt *event,
		int start_new)
{
	unsigned		status = 0;
	int			clean_busy;

	if (event->status & DEPEVT_STATUS_BUSERR)
		status = -ECONNRESET;

	clean_busy = dwc3_cleanup_done_reqs(dwc, dep, event, status);
	if (clean_busy)
		dep->flags &= ~DWC3_EP_BUSY;

	if (dwc->revision < DWC3_REVISION_183A) {
		u32		reg;
		int		i;

		for (i = 0; i < DWC3_ENDPOINTS_NUM; i++) {
			dep = dwc->eps[i];

			if (!(dep->flags & DWC3_EP_ENABLED))
				continue;

			if (!list_empty(&dep->req_queued))
				return;
		}

		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg |= dwc->u1u2;
		dwc3_writel(dwc->regs, DWC3_DCTL, reg);

		dwc->u1u2 = 0;
	}
}

static void dwc3_endpoint_interrupt(struct dwc3 *dwc,
		const struct dwc3_event_depevt *event)
{
	struct dwc3_ep		*dep;
	u8			epnum = event->endpoint_number;

	dep = dwc->eps[epnum];

	if (!(dep->flags & DWC3_EP_ENABLED))
		return;

	dev_vdbg(dwc->dev, "%s: %s\n", dep->name,
			dwc3_ep_event_string(event->endpoint_event));

	if (epnum == 0 || epnum == 1) {
		dwc3_ep0_interrupt(dwc, event);
		return;
	}

	switch (event->endpoint_event) {
	case DWC3_DEPEVT_XFERCOMPLETE:
		dep->resource_index = 0;

		if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			dev_dbg(dwc->dev, "%s is an Isochronous endpoint\n",
					dep->name);
			return;
		}

		dbg_event(dep->number, "XFRCOMP", 0);
		dwc3_endpoint_transfer_complete(dwc, dep, event, 1);
		break;
	case DWC3_DEPEVT_XFERINPROGRESS:
		if (!usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			dev_dbg(dwc->dev, "%s is not an Isochronous endpoint\n",
					dep->name);
        if (!dep->endpoint.is_ncm)
            return;
		}

		dbg_event(dep->number, "XFRPROG", 0);
		dwc3_endpoint_transfer_complete(dwc, dep, event, 0);
		break;
	case DWC3_DEPEVT_XFERNOTREADY:
		dbg_event(dep->number, "XFRNRDY", 0);
		if (usb_endpoint_xfer_isoc(dep->endpoint.desc)) {
			dwc3_gadget_start_isoc(dwc, dep, event);
		} else {
			int ret;

			dev_vdbg(dwc->dev, "%s: reason %s\n",
					dep->name, event->status &
					DEPEVT_STATUS_TRANSFER_ACTIVE
					? "Transfer Active"
					: "Transfer Not Active");

			ret = __dwc3_gadget_kick_transfer(dep, 0, 1);
			if (!ret || ret == -EBUSY)
				return;
			else
				dbg_event(dep->number, "QUEUE", ret);

			dev_dbg(dwc->dev, "%s: failed to kick transfers\n",
					dep->name);
		}

		break;
	case DWC3_DEPEVT_STREAMEVT:
		if (!usb_endpoint_xfer_bulk(dep->endpoint.desc)) {
			dev_err(dwc->dev, "Stream event for non-Bulk %s\n",
					dep->name);
			return;
		}

		switch (event->status) {
		case DEPEVT_STREAMEVT_FOUND:
			dev_vdbg(dwc->dev, "Stream %d found and started\n",
					event->parameters);

			break;
		case DEPEVT_STREAMEVT_NOTFOUND:
			
		default:
			dev_dbg(dwc->dev, "Couldn't find suitable stream\n");
		}
		break;
	case DWC3_DEPEVT_RXTXFIFOEVT:
		dev_dbg(dwc->dev, "%s FIFO Overrun\n", dep->name);
		break;
	case DWC3_DEPEVT_EPCMDCMPLT:
		dev_vdbg(dwc->dev, "Endpoint Command Complete\n");
		break;
	}
}

static void dwc3_disconnect_gadget(struct dwc3 *dwc,int mute)
{
	if (dwc->gadget_driver && dwc->gadget_driver->disconnect) {
		spin_unlock(&dwc->lock);
		if (mute) {
			dwc->gadget_driver->mute_disconnect(&dwc->gadget);
		} else {
		dwc->gadget_driver->disconnect(&dwc->gadget);
		}
		spin_lock(&dwc->lock);
	}
	dwc->gadget.xfer_isr_count = 0;
}

static void dwc3_stop_active_transfer(struct dwc3 *dwc, u32 epnum)
{
	struct dwc3_ep *dep;
	struct dwc3_gadget_ep_cmd_params params;
	u32 cmd;
	int ret;

	dep = dwc->eps[epnum];

	if (!dep->resource_index)
		return;


	cmd = DWC3_DEPCMD_ENDTRANSFER;
	cmd |= DWC3_DEPCMD_HIPRI_FORCERM | DWC3_DEPCMD_CMDIOC;
	cmd |= DWC3_DEPCMD_PARAM(dep->resource_index);
	memset(&params, 0, sizeof(params));
	ret = dwc3_send_gadget_ep_cmd(dwc, dep->number, cmd, &params);
	WARN_ON_ONCE(ret);
	dep->resource_index = 0;
	dep->flags &= ~DWC3_EP_BUSY;
	udelay(100);
}

static void dwc3_stop_active_transfers(struct dwc3 *dwc)
{
	u32 epnum;

	for (epnum = 2; epnum < DWC3_ENDPOINTS_NUM; epnum++) {
		struct dwc3_ep *dep;

		dep = dwc->eps[epnum];
		if (!(dep->flags & DWC3_EP_ENABLED))
			continue;

		dwc3_remove_requests(dwc, dep);
	}
}

static void dwc3_clear_stall_all_ep(struct dwc3 *dwc)
{
	u32 epnum;

	for (epnum = 1; epnum < DWC3_ENDPOINTS_NUM; epnum++) {
		struct dwc3_ep *dep;
		struct dwc3_gadget_ep_cmd_params params;
		int ret;

		dep = dwc->eps[epnum];

		if (!(dep->flags & DWC3_EP_STALL))
			continue;

		dep->flags &= ~DWC3_EP_STALL;

		memset(&params, 0, sizeof(params));
		ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
				DWC3_DEPCMD_CLEARSTALL, &params);
		WARN_ON_ONCE(ret);
	}
}

static void dwc3_gadget_disconnect_interrupt(struct dwc3 *dwc)
{
	int			reg;

	dev_vdbg(dwc->dev, "%s\n", __func__);

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg &= ~DWC3_DCTL_INITU1ENA;
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	reg &= ~DWC3_DCTL_INITU2ENA;
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	dbg_event(0xFF, "DISCONNECT", 0);
	dwc3_disconnect_gadget(dwc, 0);
#ifdef CONFIG_HTC_USB_DEBUG_FLAG
	dbg_event(0xFF, "DISCONNEND", 0);
#endif
	dwc->start_config_issued = false;

	dwc->gadget.speed = USB_SPEED_UNKNOWN;
	dwc->setup_packet_pending = false;
}

static void dwc3_gadget_usb3_phy_suspend(struct dwc3 *dwc, int suspend)
{
	u32			reg;

	reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));

	if (suspend)
		reg |= DWC3_GUSB3PIPECTL_SUSPHY;
	else
		reg &= ~DWC3_GUSB3PIPECTL_SUSPHY;

	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);
}

static void dwc3_gadget_usb2_phy_suspend(struct dwc3 *dwc, int suspend)
{
	u32			reg;

	reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));

	if (suspend)
		reg |= DWC3_GUSB2PHYCFG_SUSPHY;
	else
		reg &= ~DWC3_GUSB2PHYCFG_SUSPHY;

	dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);
}

static void dwc3_gadget_reset_interrupt(struct dwc3 *dwc)
{
	u32			reg;
	struct dwc3_otg		*dotg = dwc->dotg;

	dev_vdbg(dwc->dev, "%s\n", __func__);

	if (dwc->revision < DWC3_REVISION_188A) {
		if (dwc->setup_packet_pending)
			dwc3_gadget_disconnect_interrupt(dwc);
	}

#ifdef CONFIG_HTC_USB_DEBUG_FLAG
	dbg_event(0xFF, "RST STR", 0);
#else
	dbg_event(0xFF, "BUS RST", 0);
#endif
	
	dwc->dev_state = DWC3_DEFAULT_STATE;

	
	if (dwc->revision < DWC3_REVISION_194A) {
		
		dwc3_gadget_usb2_phy_suspend(dwc, false);
		dwc3_gadget_usb3_phy_suspend(dwc, false);
	}

	if (dotg && dotg->otg.phy)
		usb_phy_set_power(dotg->otg.phy, 0);

	
	if (dotg && dotg->otg.phy)
		dotg->otg.phy->notify_usb_attached(dotg->otg.phy);

	if (dwc->gadget.speed != USB_SPEED_UNKNOWN)
		dwc3_disconnect_gadget(dwc, 1);

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg &= ~DWC3_DCTL_TSTCTRL_MASK;
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);
	dwc->test_mode = false;

	dwc3_stop_active_transfers(dwc);
	dwc3_clear_stall_all_ep(dwc);
	dwc->start_config_issued = false;

	
	reg = dwc3_readl(dwc->regs, DWC3_DCFG);
	reg &= ~(DWC3_DCFG_DEVADDR_MASK);
	dwc3_writel(dwc->regs, DWC3_DCFG, reg);
#ifdef CONFIG_HTC_USB_DEBUG_FLAG
	dbg_event(0xFF, "RST END", 0);
#endif
}

static void dwc3_update_ram_clk_sel(struct dwc3 *dwc, u32 speed)
{
	u32 reg;
	u32 usb30_clock = DWC3_GCTL_CLK_BUS;


	if (speed != DWC3_DSTS_SUPERSPEED)
		return;

	if (!usb30_clock)
		return;

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg |= DWC3_GCTL_RAMCLKSEL(usb30_clock);
	dwc3_writel(dwc->regs, DWC3_GCTL, reg);
}

static void dwc3_gadget_phy_suspend(struct dwc3 *dwc, u8 speed)
{
	switch (speed) {
	case USB_SPEED_SUPER:
		dwc3_gadget_usb2_phy_suspend(dwc, true);
		break;
	case USB_SPEED_HIGH:
	case USB_SPEED_FULL:
	case USB_SPEED_LOW:
		dwc3_gadget_usb3_phy_suspend(dwc, true);
		break;
	}
}

static void dwc3_gadget_conndone_interrupt(struct dwc3 *dwc)
{
	struct dwc3_gadget_ep_cmd_params params;
	struct dwc3_ep		*dep;
	int			ret;
	u32			reg;
	u8			speed;

	dev_vdbg(dwc->dev, "%s\n", __func__);

	memset(&params, 0x00, sizeof(params));

	reg = dwc3_readl(dwc->regs, DWC3_DSTS);
	speed = reg & DWC3_DSTS_CONNECTSPD;
	dwc->speed = speed;

	dwc3_update_ram_clk_sel(dwc, speed);

	switch (speed) {
	case DWC3_DCFG_SUPERSPEED:
		if (dwc->revision < DWC3_REVISION_190A)
			dwc3_gadget_reset_interrupt(dwc);

		dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(512);
		dwc->gadget.ep0->maxpacket = 512;
		dwc->gadget.speed = USB_SPEED_SUPER;
		printk(KERN_INFO "[USB] USB_SPEED_SUPER\n");
		break;
	case DWC3_DCFG_HIGHSPEED:
		dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(64);
		dwc->gadget.ep0->maxpacket = 64;
		dwc->gadget.speed = USB_SPEED_HIGH;
		printk(KERN_INFO "[USB] USB_SPEED_HIGH\n");
		break;
	case DWC3_DCFG_FULLSPEED2:
	case DWC3_DCFG_FULLSPEED1:
		dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(64);
		dwc->gadget.ep0->maxpacket = 64;
		dwc->gadget.speed = USB_SPEED_FULL;
		printk(KERN_INFO "[USB] USB_SPEED_FULL\n");
		break;
	case DWC3_DCFG_LOWSPEED:
		dwc3_gadget_ep0_desc.wMaxPacketSize = cpu_to_le16(8);
		dwc->gadget.ep0->maxpacket = 8;
		dwc->gadget.speed = USB_SPEED_LOW;
		printk(KERN_INFO "[USB] USB_SPEED_LOW\n");
		break;
	}

	
	if (dwc->revision < DWC3_REVISION_194A) {
		
		dwc3_gadget_phy_suspend(dwc, dwc->gadget.speed);
	}

	dep = dwc->eps[0];
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, true);
	if (ret) {
		dev_err(dwc->dev, "failed to enable %s\n", dep->name);
		return;
	}

	dep = dwc->eps[1];
	ret = __dwc3_gadget_ep_enable(dep, &dwc3_gadget_ep0_desc, NULL, true);
	if (ret) {
		dev_err(dwc->dev, "failed to enable %s\n", dep->name);
		return;
	}

}

static void dwc3_gadget_wakeup_interrupt(struct dwc3 *dwc)
{
	dev_vdbg(dwc->dev, "%s\n", __func__);


	dbg_event(0xFF, "WAKEUP", 0);
	dwc->gadget_driver->resume(&dwc->gadget);
}

static void dwc3_gadget_linksts_change_interrupt(struct dwc3 *dwc,
		unsigned int evtinfo)
{
	enum dwc3_link_state	next = evtinfo & DWC3_LINK_STATE_MASK;

	if (dwc->revision < DWC3_REVISION_183A) {
		if (next == DWC3_LINK_STATE_U0) {
			u32	u1u2;
			u32	reg;

			switch (dwc->link_state) {
			case DWC3_LINK_STATE_U1:
			case DWC3_LINK_STATE_U2:
				reg = dwc3_readl(dwc->regs, DWC3_DCTL);
				u1u2 = reg & (DWC3_DCTL_INITU2ENA
						| DWC3_DCTL_ACCEPTU2ENA
						| DWC3_DCTL_INITU1ENA
						| DWC3_DCTL_ACCEPTU1ENA);

				if (!dwc->u1u2)
					dwc->u1u2 = reg & u1u2;

				reg &= ~u1u2;

				dwc3_writel(dwc->regs, DWC3_DCTL, reg);
				break;
			default:
				
				break;
			}
		}
	}

	if (next == DWC3_LINK_STATE_U0) {
		if (dwc->link_state == DWC3_LINK_STATE_U3) {
			printk(KERN_INFO "[USB] gadget irq resume\n");
			dbg_event(0xFF, "RESUME", 0);
			dwc->gadget_driver->resume(&dwc->gadget);
		}
	} else if (next == DWC3_LINK_STATE_U3) {
		printk(KERN_INFO "[USB] gadget irq suspend\n");
		dbg_event(0xFF, "SUSPEND", 0);
		dwc->gadget_driver->suspend(&dwc->gadget);
	}

	dwc->link_state = next;

	dev_vdbg(dwc->dev, "%s link %d\n", __func__, dwc->link_state);
}

static void dwc3_dump_reg_info(struct dwc3 *dwc)
{
	dbg_event(0xFF, "REGDUMP", 0);

	dbg_print_reg("GUSB3PIPCTL", dwc3_readl(dwc->regs,
							DWC3_GUSB3PIPECTL(0)));
	dbg_print_reg("GUSB2PHYCONFIG", dwc3_readl(dwc->regs,
							DWC3_GUSB2PHYCFG(0)));
	dbg_print_reg("GCTL", dwc3_readl(dwc->regs, DWC3_GCTL));
	dbg_print_reg("GUCTL", dwc3_readl(dwc->regs, DWC3_GUCTL));
	dbg_print_reg("GDBGLTSSM", dwc3_readl(dwc->regs, DWC3_GDBGLTSSM));
	dbg_print_reg("DCFG", dwc3_readl(dwc->regs, DWC3_DCFG));
	dbg_print_reg("DCTL", dwc3_readl(dwc->regs, DWC3_DCTL));
	dbg_print_reg("DEVTEN", dwc3_readl(dwc->regs, DWC3_DEVTEN));
	dbg_print_reg("DSTS", dwc3_readl(dwc->regs, DWC3_DSTS));
	dbg_print_reg("DALPENA", dwc3_readl(dwc->regs, DWC3_DALEPENA));
	dbg_print_reg("DGCMD", dwc3_readl(dwc->regs, DWC3_DGCMD));

	dbg_print_reg("OCFG", dwc3_readl(dwc->regs, DWC3_OCFG));
	dbg_print_reg("OCTL", dwc3_readl(dwc->regs, DWC3_OCTL));
	dbg_print_reg("OEVT", dwc3_readl(dwc->regs, DWC3_OEVT));
	dbg_print_reg("OSTS", dwc3_readl(dwc->regs, DWC3_OSTS));

	dwc3_notify_event(dwc, DWC3_CONTROLLER_ERROR_EVENT);
}

static void dwc3_gadget_interrupt(struct dwc3 *dwc,
		const struct dwc3_event_devt *event)
{
	switch (event->type) {
	case DWC3_DEVICE_EVENT_DISCONNECT:
		dwc3_gadget_disconnect_interrupt(dwc);
		printk(KERN_INFO "[USB] gadget irq disconnect\n");
		break;
	case DWC3_DEVICE_EVENT_RESET:
		if (dwc->gadget.ats_reset_irq_count == 50) {
			dwc->gadget.ats_reset_irq_count++;
				if (dwc->gadget_driver->broadcast_abnormal_usb_reset) {
						printk(KERN_INFO "[USB] gadget irq :abnormal the amount of reset irq!\n");
						dwc->gadget_driver->broadcast_abnormal_usb_reset();
				}
		} else if (dwc->gadget.ats_reset_irq_count < 50)
			dwc->gadget.ats_reset_irq_count++;

		dwc3_gadget_reset_interrupt(dwc);
		printk(KERN_INFO "[USB] gadget irq reset,count %d\n",dwc->gadget.ats_reset_irq_count);
		break;
	case DWC3_DEVICE_EVENT_CONNECT_DONE:
		dwc3_gadget_conndone_interrupt(dwc);
		break;
	case DWC3_DEVICE_EVENT_WAKEUP:
		dwc3_gadget_wakeup_interrupt(dwc);
		break;
	case DWC3_DEVICE_EVENT_LINK_STATUS_CHANGE:
		dwc3_gadget_linksts_change_interrupt(dwc, event->event_info);
		break;
	case DWC3_DEVICE_EVENT_EOPF:
		dev_vdbg(dwc->dev, "End of Periodic Frame\n");
		break;
	case DWC3_DEVICE_EVENT_SOF:
		dev_vdbg(dwc->dev, "Start of Periodic Frame\n");
		break;
	case DWC3_DEVICE_EVENT_ERRATIC_ERROR:
		if (!dwc->err_evt_seen) {
			dbg_event(0xFF, "ERROR", 0);
			dev_vdbg(dwc->dev, "Erratic Error\n");
			dwc3_dump_reg_info(dwc);
		}
		break;
	case DWC3_DEVICE_EVENT_CMD_CMPL:
		dev_vdbg(dwc->dev, "Command Complete\n");
		break;
	case DWC3_DEVICE_EVENT_OVERFLOW:
		dbg_event(0xFF, "OVERFL", 0);
		dev_vdbg(dwc->dev, "Overflow\n");
		/*
		 * Controllers prior to 2.30a revision has a bug where
		 * Overflow Event may overwrite an unacknowledged event
		 * in the event buffer.  The severity of the issue depends
		 * on the overwritten event type.  Add a warning message
		 * saying that an event is overwritten.
		 *
		 * TODO: In future we may need to see if we can re-enumerate
		 * with host.
		 */
		if (dwc->revision < DWC3_REVISION_230A)
			dev_warn(dwc->dev, "Unacknowledged event overwritten\n");
		break;
	case DWC3_DEVICE_EVENT_VENDOR_DEV_TEST_LMP:
		dbg_event(0xFF, "TSTLMP", 0);
		if (dwc->revision < DWC3_REVISION_230A)
			dev_warn(dwc->dev, "Vendor Device Test LMP Received\n");
		break;
	default:
		dev_dbg(dwc->dev, "UNKNOWN IRQ %d\n", event->type);
	}

	dwc->err_evt_seen = (event->type == DWC3_DEVICE_EVENT_ERRATIC_ERROR);
}

static void dwc3_process_event_entry(struct dwc3 *dwc,
		const union dwc3_event *event)
{
	
	if (event->type.is_devspec == 0) {
		
		return dwc3_endpoint_interrupt(dwc, &event->depevt);
	}

	switch (event->type.type) {
	case DWC3_EVENT_TYPE_DEV:
		dwc3_gadget_interrupt(dwc, &event->devt);
		break;
	
	default:
		dev_err(dwc->dev, "UNKNOWN IRQ type %d\n", event->raw);
	}
}

static irqreturn_t dwc3_process_event_buf(struct dwc3 *dwc, u32 buf)
{
	struct dwc3_event_buffer *evt;
	int left;
	u32 count;
#if defined(CONFIG_HTC_DEBUG_RTB)
	unsigned long long timestamp, timestamp_ms;
#endif
	count = dwc3_readl(dwc->regs, DWC3_GEVNTCOUNT(buf));
	count &= DWC3_GEVNTCOUNT_MASK;
	if (!count)
		return IRQ_NONE;

	evt = dwc->ev_buffs[buf];
	left = count;
#if defined(CONFIG_HTC_DEBUG_RTB)
	timestamp = sched_clock();
	timestamp_ms = timestamp;
	do_div(timestamp_ms, NSEC_PER_MSEC);
	uncached_logk(LOGK_LOGBUF,
		(void *)((unsigned long)timestamp_ms));
	uncached_logk(LOGK_LOGBUF,
		(void *)((unsigned long)left));
#endif
	while (left > 0) {
		union dwc3_event event;

		event.raw = *(u32 *) (evt->buf + evt->lpos);

		dwc3_process_event_entry(dwc, &event);
		evt->lpos = (evt->lpos + 4) % DWC3_EVENT_BUFFERS_SIZE;
		left -= 4;

		dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(buf), 4);
	}

	return IRQ_HANDLED;
}


static irqreturn_t dwc3_interrupt(int irq, void *_dwc)
{
	struct dwc3			*dwc = _dwc;
	int				i;
	irqreturn_t			ret = IRQ_NONE;
#if defined(CONFIG_HTC_DEBUG_RTB)
	unsigned long long timestamp, timestamp_ms;
	timestamp = sched_clock();
	timestamp_ms = timestamp;
	do_div(timestamp_ms, NSEC_PER_MSEC);
	uncached_logk(LOGK_LOGBUF,
		(void *)((unsigned long)timestamp_ms));
	uncached_logk(LOGK_LOGBUF,
		(void *)(dwc->num_event_buffers));
#endif
	spin_lock(&dwc->lock);
#ifdef CONFIG_HTC_USB_DEBUG_FLAG
	dbg_event(0xFF,"DWC3STR",0);
#endif
	for (i = 0; i < dwc->num_event_buffers; i++) {
		irqreturn_t status;

		status = dwc3_process_event_buf(dwc, i);
		if (status == IRQ_HANDLED)
			ret = status;
	}
#ifdef CONFIG_HTC_USB_DEBUG_FLAG
	dbg_event(0xFF,"DWC3END",0);
#endif
	spin_unlock(&dwc->lock);

	return ret;
}

int __devinit dwc3_gadget_init(struct dwc3 *dwc)
{
	u32					reg;
	int					ret;
	int					irq;

	dwc->ctrl_req = dma_alloc_coherent(dwc->dev, sizeof(*dwc->ctrl_req),
			&dwc->ctrl_req_addr, GFP_KERNEL);
	if (!dwc->ctrl_req) {
		dev_err(dwc->dev, "failed to allocate ctrl request\n");
		ret = -ENOMEM;
		goto err0;
	}

	dwc->ep0_trb = dma_alloc_coherent(dwc->dev, sizeof(*dwc->ep0_trb),
			&dwc->ep0_trb_addr, GFP_KERNEL);
	if (!dwc->ep0_trb) {
		dev_err(dwc->dev, "failed to allocate ep0 trb\n");
		ret = -ENOMEM;
		goto err1;
	}

	dwc->setup_buf = kzalloc(DWC3_EP0_BOUNCE_SIZE, GFP_KERNEL);
	if (!dwc->setup_buf) {
		dev_err(dwc->dev, "failed to allocate setup buffer\n");
		ret = -ENOMEM;
		goto err2;
	}

	dwc->ep0_bounce = dma_alloc_coherent(dwc->dev,
			DWC3_EP0_BOUNCE_SIZE, &dwc->ep0_bounce_addr,
			GFP_KERNEL);
	if (!dwc->ep0_bounce) {
		dev_err(dwc->dev, "failed to allocate ep0 bounce buffer\n");
		ret = -ENOMEM;
		goto err3;
	}

	dev_set_name(&dwc->gadget.dev, "gadget");

	dwc->gadget.ops			= &dwc3_gadget_ops;
	dwc->gadget.max_speed		= USB_SPEED_SUPER;
	dwc->gadget.speed		= USB_SPEED_UNKNOWN;
	dwc->gadget.dev.parent		= dwc->dev;
	dwc->gadget.sg_supported	= true;

	dma_set_coherent_mask(&dwc->gadget.dev, dwc->dev->coherent_dma_mask);

	dwc->gadget.dev.dma_parms	= dwc->dev->dma_parms;
	dwc->gadget.dev.dma_mask	= dwc->dev->dma_mask;
	dwc->gadget.dev.release		= dwc3_gadget_release;
	dwc->gadget.name		= "dwc3-gadget";


	ret = dwc3_gadget_init_endpoints(dwc);
	if (ret)
		goto err4;

	irq = platform_get_irq(to_platform_device(dwc->dev), 0);
	g_irq = irq;
	ret = request_irq(irq, dwc3_interrupt, IRQF_SHARED,
			"dwc3", dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to request irq #%d --> %d\n",
				irq, ret);
		goto err5;
	}

	reg = dwc3_readl(dwc->regs, DWC3_DCFG);
	reg |= DWC3_DCFG_LPM_CAP;
	dwc3_writel(dwc->regs, DWC3_DCFG, reg);

	
	reg = (DWC3_DEVTEN_EVNTOVERFLOWEN |
			DWC3_DEVTEN_CMDCMPLTEN |
			DWC3_DEVTEN_ERRTICERREN |
			DWC3_DEVTEN_WKUPEVTEN |
			DWC3_DEVTEN_ULSTCNGEN |
			DWC3_DEVTEN_CONNECTDONEEN |
			DWC3_DEVTEN_USBRSTEN |
			DWC3_DEVTEN_DISCONNEVTEN);
	dwc3_writel(dwc->regs, DWC3_DEVTEN, reg);

	
	if (dwc->revision >= DWC3_REVISION_194A) {
		reg = dwc3_readl(dwc->regs, DWC3_DCFG);
		reg |= DWC3_DCFG_LPM_CAP;
		dwc3_writel(dwc->regs, DWC3_DCFG, reg);

		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		reg &= ~(DWC3_DCTL_HIRD_THRES_MASK | DWC3_DCTL_L1_HIBER_EN);

		
		reg |= DWC3_DCTL_HIRD_THRES(28);

		dwc3_writel(dwc->regs, DWC3_DCTL, reg);

		dwc3_gadget_usb2_phy_suspend(dwc, false);
		dwc3_gadget_usb3_phy_suspend(dwc, true);
	}

	ret = device_register(&dwc->gadget.dev);
	if (ret) {
		dev_err(dwc->dev, "failed to register gadget device\n");
		put_device(&dwc->gadget.dev);
		goto err6;
	}

	ret = usb_add_gadget_udc(dwc->dev, &dwc->gadget);
	if (ret) {
		dev_err(dwc->dev, "failed to register udc\n");
		goto err7;
	}

	if (dwc->dotg) {
		
		ret = otg_set_peripheral(&dwc->dotg->otg, &dwc->gadget);
		if (ret) {
			dev_err(dwc->dev, "failed to set peripheral to otg\n");
			goto err7;
		}
	} else {
		pm_runtime_no_callbacks(&dwc->gadget.dev);
		pm_runtime_set_active(&dwc->gadget.dev);
		pm_runtime_enable(&dwc->gadget.dev);
		pm_runtime_get(&dwc->gadget.dev);
	}

	return 0;

err7:
	device_unregister(&dwc->gadget.dev);

err6:
	dwc3_writel(dwc->regs, DWC3_DEVTEN, 0x00);
	free_irq(irq, dwc);

err5:
	dwc3_gadget_free_endpoints(dwc);

err4:
	dma_free_coherent(dwc->dev, DWC3_EP0_BOUNCE_SIZE,
			dwc->ep0_bounce, dwc->ep0_bounce_addr);

err3:
	kfree(dwc->setup_buf);

err2:
	dma_free_coherent(dwc->dev, sizeof(*dwc->ep0_trb),
			dwc->ep0_trb, dwc->ep0_trb_addr);

err1:
	dma_free_coherent(dwc->dev, sizeof(*dwc->ctrl_req),
			dwc->ctrl_req, dwc->ctrl_req_addr);

err0:
	return ret;
}

void dwc3_gadget_exit(struct dwc3 *dwc)
{
	int			irq;

	if (dwc->dotg) {
		pm_runtime_put(&dwc->gadget.dev);
		pm_runtime_disable(&dwc->gadget.dev);
	}

	usb_del_gadget_udc(&dwc->gadget);
	irq = platform_get_irq(to_platform_device(dwc->dev), 0);

	dwc3_writel(dwc->regs, DWC3_DEVTEN, 0x00);
	free_irq(irq, dwc);

	dwc3_gadget_free_endpoints(dwc);

	dma_free_coherent(dwc->dev, DWC3_EP0_BOUNCE_SIZE,
			dwc->ep0_bounce, dwc->ep0_bounce_addr);

	kfree(dwc->setup_buf);

	dma_free_coherent(dwc->dev, sizeof(*dwc->ep0_trb),
			dwc->ep0_trb, dwc->ep0_trb_addr);

	dma_free_coherent(dwc->dev, sizeof(*dwc->ctrl_req),
			dwc->ctrl_req, dwc->ctrl_req_addr);

	device_unregister(&dwc->gadget.dev);
}
