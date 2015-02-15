/*
 * xhci-plat.c - xHCI host controller driver platform Bus Glue.
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com
 * Author: Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * A lot of code borrowed from the Linux xHCI driver.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb/otg.h>
#include <linux/usb/msm_hsusb.h>

#include "xhci.h"

#define SYNOPSIS_DWC3_VENDOR	0x5533

static struct usb_phy *phy;

static void xhci_plat_quirks(struct device *dev, struct xhci_hcd *xhci)
{
	struct xhci_plat_data *pdata = dev->platform_data;

	/*
	 * As of now platform drivers don't provide MSI support so we ensure
	 * here that the generic code does not try to make a pci_dev from our
	 * dev struct in order to setup MSI
	 */
	xhci->quirks |= XHCI_BROKEN_MSI;

	if (!pdata)
		return;

	if (pdata->vendor == SYNOPSIS_DWC3_VENDOR && pdata->revision < 0x230A)
		xhci->quirks |= XHCI_PORTSC_DELAY;

	if (pdata->vendor == SYNOPSIS_DWC3_VENDOR && pdata->revision == 0x250A)
		xhci->quirks |= XHCI_RESET_DELAY;
}

/* called during probe() after chip reset completes */
static int xhci_plat_setup(struct usb_hcd *hcd)
{
	return xhci_gen_setup(hcd, xhci_plat_quirks);
}

static void xhci_plat_phy_autosuspend(struct usb_hcd *hcd,
						int enable_autosuspend)
{
	if (!phy || !phy->set_phy_autosuspend)
		return;

	usb_phy_set_autosuspend(phy, enable_autosuspend);

	return;
}

static const struct hc_driver xhci_plat_xhci_driver = {
	.description =		"xhci-hcd",
	.product_desc =		"xHCI Host Controller",
	.hcd_priv_size =	sizeof(struct xhci_hcd *),

	/*
	 * generic hardware linkage
	 */
	.irq =			xhci_irq,
	.flags =		HCD_MEMORY | HCD_USB3 | HCD_SHARED,

	/*
	 * basic lifecycle operations
	 */
	.reset =		xhci_plat_setup,
	.start =		xhci_run,
	.stop =			xhci_stop,
	.shutdown =		xhci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		xhci_urb_enqueue,
	.urb_dequeue =		xhci_urb_dequeue,
	.alloc_dev =		xhci_alloc_dev,
	.free_dev =		xhci_free_dev,
	.alloc_streams =	xhci_alloc_streams,
	.free_streams =		xhci_free_streams,
	.add_endpoint =		xhci_add_endpoint,
	.drop_endpoint =	xhci_drop_endpoint,
	.endpoint_reset =	xhci_endpoint_reset,
	.check_bandwidth =	xhci_check_bandwidth,
	.reset_bandwidth =	xhci_reset_bandwidth,
	.address_device =	xhci_address_device,
	.update_hub_device =	xhci_update_hub_device,
	.reset_device =		xhci_discover_or_reset_device,

	/*
	 * scheduling support
	 */
	.get_frame_number =	xhci_get_frame,

	/* Root hub support */
	.hub_control =		xhci_hub_control,
	.hub_status_data =	xhci_hub_status_data,
	.bus_suspend =		xhci_bus_suspend,
	.bus_resume =		xhci_bus_resume,
	.set_autosuspend =	xhci_plat_phy_autosuspend,
};

static int xhci_plat_probe(struct platform_device *pdev)
{
	const struct hc_driver	*driver;
	struct xhci_hcd		*xhci;
	struct resource         *res;
	struct usb_hcd		*hcd;
	int			ret;
	int			irq;

	if (usb_disabled())
		return -ENODEV;

	driver = &xhci_plat_xhci_driver;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -ENODEV;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd)
		return -ENOMEM;

	hcd_to_bus(hcd)->skip_resume = true;
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,
				driver->description)) {
		dev_dbg(&pdev->dev, "controller already in use\n");
		ret = -EBUSY;
		goto put_hcd;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		dev_dbg(&pdev->dev, "error mapping memory\n");
		ret = -EFAULT;
		goto release_mem_region;
	}

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	ret = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (ret)
		goto unmap_registers;

	/* USB 2.0 roothub is stored in the platform_device now. */
	hcd = dev_get_drvdata(&pdev->dev);
	xhci = hcd_to_xhci(hcd);
	xhci->shared_hcd = usb_create_shared_hcd(driver, &pdev->dev,
			dev_name(&pdev->dev), hcd);
	if (!xhci->shared_hcd) {
		ret = -ENOMEM;
		goto dealloc_usb2_hcd;
	}

	hcd_to_bus(xhci->shared_hcd)->skip_resume = true;
	/*
	 * Set the xHCI pointer before xhci_plat_setup() (aka hcd_driver.reset)
	 * is called by usb_add_hcd().
	 */
	*((struct xhci_hcd **) xhci->shared_hcd->hcd_priv) = xhci;

	ret = usb_add_hcd(xhci->shared_hcd, irq, IRQF_SHARED);
	if (ret)
		goto put_usb3_hcd;

	phy = usb_get_transceiver();
	/* Register with OTG if present, ignore USB2 OTG using other PHY */
	if (phy && phy->otg && !(phy->flags & ENABLE_SECONDARY_PHY)) {
		dev_dbg(&pdev->dev, "%s otg support available\n", __func__);
		ret = otg_set_host(phy->otg, &hcd->self);
		if (ret) {
			dev_err(&pdev->dev, "%s otg_set_host failed\n",
				__func__);
			usb_put_transceiver(phy);
			goto put_usb3_hcd;
		}
	} else {
		pm_runtime_no_callbacks(&pdev->dev);
	}

	/*don't allow usb to suspend even if no otg device is attached*/
/*	pm_runtime_put(&pdev->dev);*/

	return 0;

put_usb3_hcd:
	usb_put_hcd(xhci->shared_hcd);

dealloc_usb2_hcd:
	usb_remove_hcd(hcd);

unmap_registers:
	iounmap(hcd->regs);

release_mem_region:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);

put_hcd:
	usb_put_hcd(hcd);

	return ret;
}

static int xhci_plat_remove(struct platform_device *dev)
{
	struct usb_hcd	*hcd = platform_get_drvdata(dev);
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);


	usb_remove_hcd(xhci->shared_hcd);
	usb_put_hcd(xhci->shared_hcd);

	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
	kfree(xhci);

	if (phy && phy->otg) {
		otg_set_host(phy->otg, NULL);
		usb_put_transceiver(phy);
	}

	return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int xhci_msm_runtime_idle(struct device *dev)
{
	dev_dbg(dev, "xhci msm runtime idle\n");
	return 0;
}

static int xhci_msm_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "xhci msm runtime suspend\n");
	/*
	 * Notify OTG about suspend.  It takes care of
	 * putting the hardware in LPM.
	 */
	if (phy)
		return usb_phy_set_suspend(phy, 1);

	return 0;
}

static int xhci_msm_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "xhci msm runtime resume\n");

	if (phy)
		return usb_phy_set_suspend(phy, 0);

	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int xhci_msm_pm_suspend(struct device *dev)
{
	dev_dbg(dev, "xhci-msm PM suspend\n");

	if (phy)
		return usb_phy_set_suspend(phy, 1);

	return 0;
}

static int xhci_msm_pm_resume(struct device *dev)
{
	dev_dbg(dev, "xhci-msm PM resume\n");

	if (pm_runtime_suspended(dev))
		return 0;

	if (phy)
		return usb_phy_set_suspend(phy, 0);

	return 0;
}
#endif

static const struct dev_pm_ops xhci_msm_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(xhci_msm_pm_suspend, xhci_msm_pm_resume)
	SET_RUNTIME_PM_OPS(xhci_msm_runtime_suspend, xhci_msm_runtime_resume,
				xhci_msm_runtime_idle)
};

static struct platform_driver usb_xhci_driver = {
	.probe	= xhci_plat_probe,
	.remove	= xhci_plat_remove,
	.driver	= {
		.name = "xhci-hcd",
		.pm = &xhci_msm_dev_pm_ops,
	},
};
MODULE_ALIAS("platform:xhci-hcd");

int xhci_register_plat(void)
{
	return platform_driver_register(&usb_xhci_driver);
}

void xhci_unregister_plat(void)
{
	platform_driver_unregister(&usb_xhci_driver);
}
