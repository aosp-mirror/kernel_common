/**
 * core.c - DesignWare USB3 DRD Controller Core file
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include "core.h"
#include "gadget.h"
#include "io.h"

#include "debug.h"
#include <mach/htc_battery_common.h>

static char *maximum_speed = "super";
module_param(maximum_speed, charp, 0);
MODULE_PARM_DESC(maximum_speed, "Maximum supported speed.");

/* -------------------------------------------------------------------------- */

#define DWC3_DEVS_POSSIBLE	32

static DECLARE_BITMAP(dwc3_devs, DWC3_DEVS_POSSIBLE);

int dwc3_get_device_id(void)
{
	int		id;

again:
	id = find_first_zero_bit(dwc3_devs, DWC3_DEVS_POSSIBLE);
	if (id < DWC3_DEVS_POSSIBLE) {
		int old;

		old = test_and_set_bit(id, dwc3_devs);
		if (old)
			goto again;
	} else {
		pr_err("dwc3: no space for new device\n");
		id = -ENOMEM;
	}

	return id;
}
EXPORT_SYMBOL_GPL(dwc3_get_device_id);

void dwc3_put_device_id(int id)
{
	int			ret;

	if (id < 0)
		return;

	ret = test_bit(id, dwc3_devs);
	WARN(!ret, "dwc3: ID %d not in use\n", id);
	smp_mb__before_clear_bit();
	clear_bit(id, dwc3_devs);
}
EXPORT_SYMBOL_GPL(dwc3_put_device_id);

void dwc3_set_mode(struct dwc3 *dwc, u32 mode)
{
	u32 reg;

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg &= ~(DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_OTG));
	reg |= DWC3_GCTL_PRTCAPDIR(mode);
	/*
	 * Set this bit so that device attempts three more times at SS, even
	 * if it failed previously to operate in SS mode.
	 */
	reg |= DWC3_GCTL_U2RSTECN;
	reg &= ~(DWC3_GCTL_SOFITPSYNC);
	reg &= ~(DWC3_GCTL_PWRDNSCALEMASK);
	reg |= DWC3_GCTL_PWRDNSCALE(2);
	reg |= DWC3_GCTL_U2EXIT_LFPS;
	dwc3_writel(dwc->regs, DWC3_GCTL, reg);

	if (mode == DWC3_GCTL_PRTCAP_OTG || mode == DWC3_GCTL_PRTCAP_HOST) {
		/*
		 * Allow ITP generated off of ref clk based counter instead
		 * of UTMI/ULPI clk based counter, when superspeed only is
		 * active so that UTMI/ULPI PHY can be suspened.
		 *
		 * Starting with revision 2.50A, GFLADJ_REFCLK_LPM_SEL is used
		 * instead.
		 */
		if (dwc->revision < DWC3_REVISION_250A) {
			reg = dwc3_readl(dwc->regs, DWC3_GCTL);
			reg |= DWC3_GCTL_SOFITPSYNC;
			dwc3_writel(dwc->regs, DWC3_GCTL, reg);
		} else {
			reg = dwc3_readl(dwc->regs, DWC3_GFLADJ);
			reg |= DWC3_GFLADJ_REFCLK_LPM_SEL;
			dwc3_writel(dwc->regs, DWC3_GFLADJ, reg);
		}
	}

	reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
	reg |= DWC3_GUSB3PIPECTL_SUSPHY;
	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);
	reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
	reg |= DWC3_GUSB2PHYCFG_SUSPHY;
	dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);
}

/**
 * dwc3_core_soft_reset - Issues core soft reset and PHY reset
 * @dwc: pointer to our context structure
 */
static void dwc3_core_soft_reset(struct dwc3 *dwc)
{
	u32		reg;

	/* Before Resetting PHY, put Core in Reset */
	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg |= DWC3_GCTL_CORESOFTRESET;
	dwc3_writel(dwc->regs, DWC3_GCTL, reg);

	dwc3_notify_event(dwc, DWC3_CONTROLLER_RESET_EVENT);

	/* Assert USB3 PHY reset */
	reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
	reg |= DWC3_GUSB3PIPECTL_PHYSOFTRST;
	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);

	/* Assert USB2 PHY reset */
	reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
	reg |= DWC3_GUSB2PHYCFG_PHYSOFTRST;
	dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);

	mdelay(10);

	/* Clear USB3 PHY reset */
	reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
	reg &= ~DWC3_GUSB3PIPECTL_PHYSOFTRST;
	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);

	/* Clear USB2 PHY reset */
	reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
	reg &= ~DWC3_GUSB2PHYCFG_PHYSOFTRST;
	dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);

	mdelay(10);

	/* After PHYs are stable we can take Core out of reset state */
	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg &= ~DWC3_GCTL_CORESOFTRESET;
	dwc3_writel(dwc->regs, DWC3_GCTL, reg);

	dwc3_notify_event(dwc, DWC3_CONTROLLER_POST_RESET_EVENT);
}

/**
 * dwc3_free_one_event_buffer - Frees one event buffer
 * @dwc: Pointer to our controller context structure
 * @evt: Pointer to event buffer to be freed
 */
static void dwc3_free_one_event_buffer(struct dwc3 *dwc,
		struct dwc3_event_buffer *evt)
{
	dma_free_coherent(dwc->dev, evt->length, evt->buf, evt->dma);
	kfree(evt);
}

/**
 * dwc3_alloc_one_event_buffer - Allocates one event buffer structure
 * @dwc: Pointer to our controller context structure
 * @length: size of the event buffer
 *
 * Returns a pointer to the allocated event buffer structure on success
 * otherwise ERR_PTR(errno).
 */
static struct dwc3_event_buffer *__devinit
dwc3_alloc_one_event_buffer(struct dwc3 *dwc, unsigned length)
{
	struct dwc3_event_buffer	*evt;

	evt = kzalloc(sizeof(*evt), GFP_KERNEL);
	if (!evt)
		return ERR_PTR(-ENOMEM);

	evt->dwc	= dwc;
	evt->length	= length;
	evt->buf	= dma_alloc_coherent(dwc->dev, length,
			&evt->dma, GFP_KERNEL);
	if (!evt->buf) {
		kfree(evt);
		return ERR_PTR(-ENOMEM);
	}

	return evt;
}

/**
 * dwc3_free_event_buffers - frees all allocated event buffers
 * @dwc: Pointer to our controller context structure
 */
static void dwc3_free_event_buffers(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;
	int i;

	for (i = 0; i < dwc->num_event_buffers; i++) {
		evt = dwc->ev_buffs[i];
		if (evt)
			dwc3_free_one_event_buffer(dwc, evt);
	}

	kfree(dwc->ev_buffs);
}

/**
 * dwc3_alloc_event_buffers - Allocates @num event buffers of size @length
 * @dwc: pointer to our controller context structure
 * @length: size of event buffer
 *
 * Returns 0 on success otherwise negative errno. In the error case, dwc
 * may contain some buffers allocated but not all which were requested.
 */
static int __devinit dwc3_alloc_event_buffers(struct dwc3 *dwc, unsigned length)
{
	int			num;
	int			i;

	num = DWC3_NUM_INT(dwc->hwparams.hwparams1);
	dwc->num_event_buffers = num;

	dwc->ev_buffs = kzalloc(sizeof(*dwc->ev_buffs) * num, GFP_KERNEL);
	if (!dwc->ev_buffs) {
		dev_err(dwc->dev, "can't allocate event buffers array\n");
		return -ENOMEM;
	}

	for (i = 0; i < num; i++) {
		struct dwc3_event_buffer	*evt;

		/*
		 * As SW workaround, allocate 8 bytes more than size of event
		 * buffer given to USB Controller to avoid possible memory
		 * corruption caused by event buffer overflow when Hw writes
		 * Vendor Device test event which could be of 12 bytes.
		 */
		evt = dwc3_alloc_one_event_buffer(dwc, (length + 8));
		if (IS_ERR(evt)) {
			dev_err(dwc->dev, "can't allocate event buffer\n");
			return PTR_ERR(evt);
		}
		dwc->ev_buffs[i] = evt;
	}

	return 0;
}

/**
 * dwc3_event_buffers_setup - setup our allocated event buffers
 * @dwc: pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
int dwc3_event_buffers_setup(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;
	int				n;

	for (n = 0; n < dwc->num_event_buffers; n++) {
		evt = dwc->ev_buffs[n];
		dev_dbg(dwc->dev, "Event buf %p dma %08llx length %d\n",
				evt->buf, (unsigned long long) evt->dma,
				evt->length);

		evt->lpos = 0;

		dwc3_writel(dwc->regs, DWC3_GEVNTADRLO(n),
				lower_32_bits(evt->dma));
		dwc3_writel(dwc->regs, DWC3_GEVNTADRHI(n),
				upper_32_bits(evt->dma));
		dwc3_writel(dwc->regs, DWC3_GEVNTSIZ(n),
				(evt->length - 8) & 0xffff);
		dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(n), 0);
	}

	return 0;
}

static void dwc3_event_buffers_cleanup(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;
	int				n;

	for (n = 0; n < dwc->num_event_buffers; n++) {
		evt = dwc->ev_buffs[n];

		evt->lpos = 0;

		dwc3_writel(dwc->regs, DWC3_GEVNTADRLO(n), 0);
		dwc3_writel(dwc->regs, DWC3_GEVNTADRHI(n), 0);
		dwc3_writel(dwc->regs, DWC3_GEVNTSIZ(n), 0);
		dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(n), 0);
	}
}

static void dwc3_cache_hwparams(struct dwc3 *dwc)
{
	struct dwc3_hwparams	*parms = &dwc->hwparams;

	parms->hwparams0 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS0);
	parms->hwparams1 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS1);
	parms->hwparams2 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS2);
	parms->hwparams3 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS3);
	parms->hwparams4 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS4);
	parms->hwparams5 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS5);
	parms->hwparams6 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS6);
	parms->hwparams7 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS7);
	parms->hwparams8 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS8);
}

/**
 * dwc3_core_init - Low-level initialization of DWC3 Core
 * @dwc: Pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
static int dwc3_core_init(struct dwc3 *dwc)
{
	unsigned long		timeout;
	u32			reg;
	int			ret;

	reg = dwc3_readl(dwc->regs, DWC3_GSNPSID);
	/* This should read as U3 followed by revision number */
	if ((reg & DWC3_GSNPSID_MASK) != 0x55330000) {
		dev_err(dwc->dev, "this is not a DesignWare USB3 DRD Core\n");
		ret = -ENODEV;
		goto err0;
	}
	dwc->revision = reg;

	/* issue device SoftReset too */
	timeout = jiffies + msecs_to_jiffies(500);
	dwc3_writel(dwc->regs, DWC3_DCTL, DWC3_DCTL_CSFTRST);
	do {
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		if (!(reg & DWC3_DCTL_CSFTRST))
			break;

		if (time_after(jiffies, timeout)) {
			dev_err(dwc->dev, "Reset Timed Out\n");
			ret = -ETIMEDOUT;
			goto err0;
		}

		cpu_relax();
	} while (true);

	dwc3_core_soft_reset(dwc);

	dwc3_cache_hwparams(dwc);

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg &= ~DWC3_GCTL_SCALEDOWN_MASK;
	reg &= ~DWC3_GCTL_DISSCRAMBLE;

	switch (DWC3_GHWPARAMS1_EN_PWROPT(dwc->hwparams.hwparams1)) {
	case DWC3_GHWPARAMS1_EN_PWROPT_CLK:
		reg &= ~DWC3_GCTL_DSBLCLKGTNG;
		break;
	default:
		dev_dbg(dwc->dev, "No power optimization available\n");
	}

	/*
	 * WORKAROUND: DWC3 revisions <1.90a have a bug
	 * where the device can fail to connect at SuperSpeed
	 * and falls back to high-speed mode which causes
	 * the device to enter a Connect/Disconnect loop
	 */
	if (dwc->revision < DWC3_REVISION_190A)
		reg |= DWC3_GCTL_U2RSTECN;

	dwc3_writel(dwc->regs, DWC3_GCTL, reg);

	/*
	 * The default value of GUCTL[31:22] should be 0x8. But on cores
	 * revision < 2.30a, the default value is mistakenly overridden
	 * with 0x0. Restore the correct default value.
	 */
	if (dwc->revision < DWC3_REVISION_230A) {
		reg = dwc3_readl(dwc->regs, DWC3_GUCTL);
		reg &= ~DWC3_GUCTL_REFCLKPER;
		reg |= 0x8 << __ffs(DWC3_GUCTL_REFCLKPER);
		dwc3_writel(dwc->regs, DWC3_GUCTL, reg);
	}
	/*
	 * Currently, the default and the recommended value for GUSB3PIPECTL
	 * [21:19] in the RTL is 3'b100 or 32 consecutive errors. Based on
	 * analysis and experiments in the lab, it is found that there is a
	 * relatively low probability of getting 32 consecutive word errors
	 * in the presence of random recovered noise (during electrical idle).
	 * This can delay the entry to a low power state such that for
	 * applications where the link stays in a non-U0 state for a short
	 * duration (< 1 microsecond), the local PHY does not enter the low
	 * power state prior to receiving a potential LFPS wakeup. This causes
	 * the PHY CDR (Clock and Data Recovery) operation to be unstable for
	 * some Synopsys PHYs.
	 *
	 * The proposal now is to change the default and the recommended value
	 * for GUSB3PIPECTL[21:19] in the RTL from 3'b100 to a minimum of
	 * 3'b001. Perform the same in software for controllers prior to 2.30a
	 * revision.
	 */

	if (dwc->revision < DWC3_REVISION_230A) {
		reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
		reg &= ~DWC3_GUSB3PIPECTL_DELAY_P1P2P3;
		reg |= 1 << __ffs(DWC3_GUSB3PIPECTL_DELAY_P1P2P3);
		/*
		 * Receiver Detection in U3/Rx.Det is mistakenly disabled in
		 * cores < 2.30a. Fix it here.
		 */
		reg &= ~DWC3_GUSB3PIPECTL_DIS_RXDET_U3_RXDET;
		dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);
	}
	/*
	 * clear Elastic buffer mode in GUSBPIPE_CTRL(0) register, otherwise
	 * it results in high link errors and could cause SS mode transfer
	 * failure.
	 */
	reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
	reg &= ~DWC3_GUSB3PIPECTL_ELASTIC_BUF_MODE;
	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);

	if (!dwc->ev_buffs) {
		ret = dwc3_alloc_event_buffers(dwc, DWC3_EVENT_BUFFERS_SIZE);
		if (ret) {
			dev_err(dwc->dev, "failed to allocate event buffers\n");
			ret = -ENOMEM;
			goto err1;
		}
	}

	ret = dwc3_event_buffers_setup(dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to setup event buffers\n");
		goto err1;
	}

	return 0;

err1:
	dwc3_free_event_buffers(dwc);

err0:
	return ret;
}

static void dwc3_core_exit(struct dwc3 *dwc)
{
	dwc3_event_buffers_cleanup(dwc);
	dwc3_free_event_buffers(dwc);
}

/* XHCI reset, resets other CORE registers as well, re-init those */
void dwc3_post_host_reset_core_init(struct dwc3 *dwc)
{
	dwc3_core_init(dwc);
	dwc3_gadget_restart(dwc);
	dwc3_notify_event(dwc, DWC3_CONTROLLER_POST_INITIALIZATION_EVENT);
}

static void usb_chg_stop(struct work_struct *w)
{
#if (defined(CONFIG_HTC_BATT_8960))
	printk(KERN_INFO "[USB] disable charger\n");
	htc_battery_pwrsrc_disable();
#endif
}

static void (*notify_event) (struct dwc3 *, unsigned);
void dwc3_set_notifier(void (*notify)(struct dwc3 *, unsigned))
{
	notify_event = notify;
}
EXPORT_SYMBOL(dwc3_set_notifier);

void dwc3_notify_event(struct dwc3 *dwc, unsigned event)
{
	if (dwc->notify_event)
		dwc->notify_event(dwc, event);
}
EXPORT_SYMBOL(dwc3_notify_event);

#define DWC3_ALIGN_MASK		(16 - 1)

static u64 dwc3_dma_mask = DMA_BIT_MASK(64);
static int __devinit dwc3_probe(struct platform_device *pdev)
{
	struct device_node	*node = pdev->dev.of_node;
	struct resource		*res;
	struct dwc3		*dwc;
	struct device		*dev = &pdev->dev;

	int			ret = -ENOMEM;

	void __iomem		*regs;
	void			*mem;

	u8			mode;
	bool			host_only_mode;

	mem = devm_kzalloc(dev, sizeof(*dwc) + DWC3_ALIGN_MASK, GFP_KERNEL);
	if (!mem) {
		dev_err(dev, "not enough memory\n");
		return -ENOMEM;
	}
	dwc = PTR_ALIGN(mem, DWC3_ALIGN_MASK + 1);
	dwc->mem = mem;

	if (!dev->dma_mask)
		dev->dma_mask = &dwc3_dma_mask;
	if (!dev->coherent_dma_mask)
		dev->coherent_dma_mask = DMA_BIT_MASK(64);

	dwc->notify_event = notify_event;
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "missing IRQ\n");
		return -ENODEV;
	}
	dwc->xhci_resources[1].start = res->start;
	dwc->xhci_resources[1].end = res->end;
	dwc->xhci_resources[1].flags = res->flags;
	dwc->xhci_resources[1].name = res->name;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "missing memory resource\n");
		return -ENODEV;
	}
	dwc->xhci_resources[0].start = res->start;
	dwc->xhci_resources[0].end = dwc->xhci_resources[0].start +
					DWC3_XHCI_REGS_END;
	dwc->xhci_resources[0].flags = res->flags;
	dwc->xhci_resources[0].name = res->name;

	 /*
	  * Request memory region but exclude xHCI regs,
	  * since it will be requested by the xhci-plat driver.
	  */
	res = devm_request_mem_region(dev, res->start + DWC3_GLOBALS_REGS_START,
			resource_size(res) - DWC3_GLOBALS_REGS_START,
			dev_name(dev));

	if (!res) {
		dev_err(dev, "can't request mem region\n");
		return -ENOMEM;
	}

	regs = devm_ioremap_nocache(dev, res->start, resource_size(res));
	if (!regs) {
		dev_err(dev, "ioremap failed\n");
		return -ENOMEM;
	}

	spin_lock_init(&dwc->lock);
	platform_set_drvdata(pdev, dwc);

	dwc->regs	= regs;
	dwc->regs_size	= resource_size(res);
	dwc->dev	= dev;

	if (!strncmp("super", maximum_speed, 5))
		dwc->maximum_speed = DWC3_DCFG_SUPERSPEED;
	else if (!strncmp("high", maximum_speed, 4))
		dwc->maximum_speed = DWC3_DCFG_HIGHSPEED;
	else if (!strncmp("full", maximum_speed, 4))
		dwc->maximum_speed = DWC3_DCFG_FULLSPEED1;
	else if (!strncmp("low", maximum_speed, 3))
		dwc->maximum_speed = DWC3_DCFG_LOWSPEED;
	else
		dwc->maximum_speed = DWC3_DCFG_SUPERSPEED;

	dwc->needs_fifo_resize = of_property_read_bool(node, "tx-fifo-resize");
	host_only_mode = of_property_read_bool(node, "host-only-mode");

	pm_runtime_no_callbacks(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = dwc3_core_init(dwc);
	if (ret) {
		dev_err(dev, "failed to initialize core\n");
		return ret;
	}

	mode = DWC3_MODE(dwc->hwparams.hwparams0);

	/* Override mode if user selects host-only config with DRD core */
	if (host_only_mode && (mode == DWC3_MODE_DRD)) {
		dev_dbg(dev, "host only mode selected\n");
		mode = DWC3_MODE_HOST;
	}
	INIT_DELAYED_WORK(&dwc->chg_stop, usb_chg_stop);

	switch (mode) {
	case DWC3_MODE_DEVICE:
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_DEVICE);
		ret = dwc3_gadget_init(dwc);
		if (ret) {
			dev_err(dev, "failed to initialize gadget\n");
			goto err1;
		}
		break;
	case DWC3_MODE_HOST:
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_HOST);
		ret = dwc3_host_init(dwc);
		if (ret) {
			dev_err(dev, "failed to initialize host\n");
			goto err1;
		}
		break;
	case DWC3_MODE_DRD:
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_OTG);
		ret = dwc3_otg_init(dwc);
		if (ret) {
			dev_err(dev, "failed to initialize otg\n");
			goto err1;
		}

		ret = dwc3_host_init(dwc);
		if (ret) {
			dev_err(dev, "failed to initialize host\n");
			dwc3_otg_exit(dwc);
			goto err1;
		}

		ret = dwc3_gadget_init(dwc);
		if (ret) {
			dev_err(dev, "failed to initialize gadget\n");
			dwc3_host_exit(dwc);
			dwc3_otg_exit(dwc);
			goto err1;
		}
		break;
	default:
		dev_err(dev, "Unsupported mode of operation %d\n", mode);
		goto err1;
	}
	dwc->mode = mode;

	ret = dwc3_debugfs_init(dwc);
	if (ret) {
		dev_err(dev, "failed to initialize debugfs\n");
		goto err2;
	}

	dwc3_notify_event(dwc, DWC3_CONTROLLER_POST_INITIALIZATION_EVENT);

	return 0;

err2:
	switch (mode) {
	case DWC3_MODE_DEVICE:
		dwc3_gadget_exit(dwc);
		break;
	case DWC3_MODE_HOST:
		dwc3_host_exit(dwc);
		break;
	case DWC3_MODE_DRD:
		dwc3_gadget_exit(dwc);
		dwc3_host_exit(dwc);
		dwc3_otg_exit(dwc);
		break;
	default:
		/* do nothing */
		break;
	}

err1:
	dwc3_core_exit(dwc);

	return ret;
}

static int __devexit dwc3_remove(struct platform_device *pdev)
{
	struct dwc3	*dwc = platform_get_drvdata(pdev);
	struct resource	*res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	pm_runtime_disable(&pdev->dev);

	dwc3_debugfs_exit(dwc);

	switch (dwc->mode) {
	case DWC3_MODE_DEVICE:
		dwc3_gadget_exit(dwc);
		break;
	case DWC3_MODE_HOST:
		dwc3_host_exit(dwc);
		break;
	case DWC3_MODE_DRD:
		dwc3_gadget_exit(dwc);
		dwc3_host_exit(dwc);
		dwc3_otg_exit(dwc);
		break;
	default:
		/* do nothing */
		break;
	}

	dwc3_core_exit(dwc);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id of_dwc3_match[] = {
	{
		.compatible = "synopsys,dwc3"
	},
	{ },
};
MODULE_DEVICE_TABLE(of, of_dwc3_match);
#endif

static struct platform_driver dwc3_driver = {
	.probe		= dwc3_probe,
	.remove		= __devexit_p(dwc3_remove),
	.driver		= {
		.name	= "dwc3",
		.of_match_table	= of_match_ptr(of_dwc3_match),
	},
};

module_platform_driver(dwc3_driver);

MODULE_ALIAS("platform:dwc3");
MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("DesignWare USB3 DRD Controller Driver");
