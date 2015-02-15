/**
 * dwc3_otg.c - DesignWare USB3 DRD Controller OTG
 *
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include "core.h"
#include "dwc3_otg.h"
#include "io.h"
#include "xhci.h"
#include <linux/usb/htc_info.h>
#include <mach/board.h>
#include <linux/usb/msm_hsusb.h>
#include <mach/cable_detect.h>

static struct dwc3_otg *the_dwc3_otg;
extern int vbus;
static DEFINE_MUTEX(smwork_sem);
static DEFINE_MUTEX(notify_sem);


int htc_dwc3_chg_det_check_linestate(void *mdwc);
int htc_dwc3_get_cable_type(void);


int htc_dwc3_get_line_state(struct dwc3 *dwc)
{
	struct platform_device *pdev_dwc_msm = container_of(dwc->dev->parent, struct platform_device, dev);
	void *msm = (void *)platform_get_drvdata(pdev_dwc_msm);
	return htc_dwc3_chg_det_check_linestate(msm);

}

static void send_usb_connect_notify(struct work_struct *w)
{
	static struct t_usb_status_notifier *notifier;
	struct dwc3_otg *dotg = container_of(w, struct dwc3_otg,  notifier_work);
	if (!dotg)
		return;

	dotg->connect_type_ready = 1;
	USBH_INFO("send connect type %d\n", dotg->connect_type);
	mutex_lock(&notify_sem);
	list_for_each_entry(notifier, &g_lh_usb_notifier_list, notifier_link) {
		if (notifier->func != NULL) {
			
			notifier->func(dotg->connect_type);
		}
	}
	mutex_unlock(&notify_sem);
}

int htc_dwc3_usb_register_notifier(struct t_usb_status_notifier *notifier)
{
	if (!notifier || !notifier->name || !notifier->func)
		return -EINVAL;

	mutex_lock(&notify_sem);
	list_add(&notifier->notifier_link,
			&g_lh_usb_notifier_list);
	mutex_unlock(&notify_sem);
	return 0;
}

int usb_is_connect_type_ready(void)
{
	if (!the_dwc3_otg)
		return 0;
	return the_dwc3_otg->connect_type_ready;
}

int usb_get_connect_type(void)
{
	if (!the_dwc3_otg)
		return 0;
#ifdef CONFIG_MACH_VERDI_LTE
	if (the_dwc3_otg->connect_type == CONNECT_TYPE_USB_9V_AC)
		return CONNECT_TYPE_9V_AC;
#endif
	return the_dwc3_otg->connect_type;
}

#define VBUS_REG_CHECK_DELAY	(msecs_to_jiffies(1000))
#define MAX_INVALID_CHRGR_RETRY 3
static int max_chgr_retry_count = MAX_INVALID_CHRGR_RETRY;
module_param(max_chgr_retry_count, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(max_chgr_retry_count, "Max invalid charger retry count");
static void dwc3_otg_reset(struct dwc3_otg *dotg);

static void dwc3_otg_notify_host_mode(struct usb_otg *otg, int host_mode);
static void dwc3_otg_reset(struct dwc3_otg *dotg);

static void dwc3_otg_set_host_regs(struct dwc3_otg *dotg)
{
	u32 reg;
	struct dwc3 *dwc = dotg->dwc;
	struct dwc3_ext_xceiv *ext_xceiv = dotg->ext_xceiv;

	if (ext_xceiv && !ext_xceiv->otg_capability) {
		
		reg = dwc3_readl(dotg->regs, DWC3_OCTL);
		reg &= ~DWC3_OTG_OCTL_PERIMODE;
		dwc3_writel(dotg->regs, DWC3_OCTL, reg);
	} else {
		reg = dwc3_readl(dwc->regs, DWC3_GCTL);
		reg &= ~(DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_OTG));
		reg |= DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_HOST);
		reg |= DWC3_GCTL_SOFITPSYNC;
		reg |= DWC3_GCTL_U2RSTECN;
		reg &= ~(DWC3_GCTL_PWRDNSCALEMASK);
		reg |= DWC3_GCTL_PWRDNSCALE(2);
		dwc3_writel(dwc->regs, DWC3_GCTL, reg);
	}
}

static int dwc3_otg_set_suspend(struct usb_phy *phy, int suspend)
{
	struct usb_otg *otg = phy->otg;
	struct dwc3_otg *dotg = container_of(otg, struct dwc3_otg, otg);

	if (dotg->host_bus_suspend == suspend)
		return 0;

	dotg->host_bus_suspend = suspend;
	if (suspend) {
		pm_runtime_put_sync(phy->dev);
	} else {
		pm_runtime_get_noresume(phy->dev);
		pm_runtime_resume(phy->dev);
	}

	return 0;
}

static void dwc3_otg_set_hsphy_auto_suspend(struct dwc3_otg *dotg, bool susp);
static int dwc3_otg_set_autosuspend(struct usb_phy *phy, int enable_autosuspend)
{
	struct usb_otg *otg = phy->otg;
	struct dwc3_otg *dotg = container_of(otg, struct dwc3_otg, otg);

	dwc3_otg_set_hsphy_auto_suspend(dotg, enable_autosuspend);

	return 0;
}

static void dwc3_otg_set_hsphy_auto_suspend(struct dwc3_otg *dotg, bool susp)
{
	struct dwc3 *dwc = dotg->dwc;
	u32 reg;

	reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
	if (susp)
		reg |= DWC3_GUSB2PHYCFG_SUSPHY;
	else
		reg &= ~(DWC3_GUSB2PHYCFG_SUSPHY);
	dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);
}

void dwc3_otg_set_host_power(struct dwc3_otg *dotg)
{
	u32 osts;

	osts = dwc3_readl(dotg->regs, DWC3_OSTS);
	if (!(osts & 0x8))
		dev_err(dotg->dwc->dev, "%s: xHCIPrtPower not set\n", __func__);

	dwc3_writel(dotg->regs, DWC3_OCTL, DWC3_OTG_OCTL_PRTPWRCTL);
}

static void dwc3_otg_set_peripheral_regs(struct dwc3_otg *dotg)
{
	u32 reg;
	struct dwc3 *dwc = dotg->dwc;
	struct dwc3_ext_xceiv *ext_xceiv = dotg->ext_xceiv;

	if (ext_xceiv && !ext_xceiv->otg_capability) {
		
		reg = dwc3_readl(dotg->regs, DWC3_OCTL);
		reg |= DWC3_OTG_OCTL_PERIMODE;
		dwc3_writel(dotg->regs, DWC3_OCTL, reg);
	} else {
		reg = dwc3_readl(dwc->regs, DWC3_GCTL);
		reg &= ~(DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_OTG));
		reg |= DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_DEVICE);
		reg |= DWC3_GCTL_U2RSTECN;
		reg &= ~(DWC3_GCTL_PWRDNSCALEMASK);
		reg |= DWC3_GCTL_PWRDNSCALE(2);
		reg &= ~(DWC3_GCTL_SOFITPSYNC);
		dwc3_writel(dwc->regs, DWC3_GCTL, reg);
	}
}

static int dwc3_otg_start_host(struct usb_otg *otg, int on)
{
	struct dwc3_otg *dotg = container_of(otg, struct dwc3_otg, otg);
	struct dwc3_ext_xceiv *ext_xceiv = dotg->ext_xceiv;
	struct dwc3 *dwc = dotg->dwc;
	int ret = 0;

	if (!dwc->xhci)
		return -EINVAL;

	if (!dotg->vbus_otg) {
		dotg->vbus_otg = devm_regulator_get(dwc->dev->parent,
							"vbus_dwc3");
		if (IS_ERR(dotg->vbus_otg)) {
			dev_err(dwc->dev, "Failed to get vbus regulator\n");
			ret = PTR_ERR(dotg->vbus_otg);
			dotg->vbus_otg = 0;
			return ret;
		}
	}

	if (on) {
		dev_dbg(otg->phy->dev, "%s: turn on host\n", __func__);

		dwc3_otg_notify_host_mode(otg, on);
		ret = regulator_enable(dotg->vbus_otg);
		if (ret) {
			dev_err(otg->phy->dev, "unable to enable vbus_otg\n");
			dwc3_otg_notify_host_mode(otg, 0);
			return ret;
		}

		dwc3_otg_set_hsphy_auto_suspend(dotg, true);
		dwc3_otg_set_host_regs(dotg);
		pm_runtime_init(&dwc->xhci->dev);
		ret = platform_device_add(dwc->xhci);
		if (ret) {
			dev_err(otg->phy->dev,
				"%s: failed to add XHCI pdev ret=%d\n",
				__func__, ret);
			regulator_disable(dotg->vbus_otg);
			dwc3_otg_notify_host_mode(otg, 0);
			return ret;
		}

		dotg->connect_type = CONNECT_TYPE_INTERNAL;
		queue_work(dotg->usb_wq, &dotg->notifier_work);

		
		if (ext_xceiv && !ext_xceiv->otg_capability)
			dwc3_otg_reset(dotg);
	} else {
		dev_dbg(otg->phy->dev, "%s: turn off host\n", __func__);

		dotg->connect_type = CONNECT_TYPE_CLEAR;
		queue_work(dotg->usb_wq, &dotg->notifier_work);
		ret = regulator_disable(dotg->vbus_otg);
		if (ret) {
			dev_err(otg->phy->dev, "unable to disable vbus_otg\n");
			return ret;
		}
		dwc3_otg_notify_host_mode(otg, on);

		platform_device_del(dwc->xhci);
		if (ext_xceiv && ext_xceiv->otg_capability &&
						ext_xceiv->ext_block_reset)
			ext_xceiv->ext_block_reset(ext_xceiv, true);

		dwc3_otg_set_hsphy_auto_suspend(dotg, false);
		dwc3_otg_set_peripheral_regs(dotg);

		
		dwc3_post_host_reset_core_init(dwc);
		if (ext_xceiv && !ext_xceiv->otg_capability)
			dwc3_otg_reset(dotg);
	}

	return 0;
}

static int dwc3_otg_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	struct dwc3_otg *dotg = container_of(otg, struct dwc3_otg, otg);

	if (host) {
		dev_dbg(otg->phy->dev, "%s: set host %s, portpower\n",
					__func__, host->bus_name);
		otg->host = host;
		msleep(300);
		dwc3_otg_set_host_power(dotg);
	} else {
		otg->host = NULL;
	}

	return 0;
}

static int dwc3_otg_start_peripheral(struct usb_otg *otg, int on)
{
	struct dwc3_otg *dotg = container_of(otg, struct dwc3_otg, otg);
	struct dwc3_ext_xceiv *ext_xceiv = dotg->ext_xceiv;

	if (!otg->gadget)
		return -EINVAL;

	if (on) {
		dev_dbg(otg->phy->dev, "%s: turn on gadget %s\n",
					__func__, otg->gadget->name);

		if (ext_xceiv && ext_xceiv->otg_capability &&
						ext_xceiv->ext_block_reset)
			ext_xceiv->ext_block_reset(ext_xceiv, false);

		dwc3_otg_set_hsphy_auto_suspend(dotg, true);
		dwc3_otg_set_peripheral_regs(dotg);
		usb_gadget_vbus_connect(otg->gadget);
	} else {
		dev_dbg(otg->phy->dev, "%s: turn off gadget %s\n",
					__func__, otg->gadget->name);
		usb_gadget_vbus_disconnect(otg->gadget);
		dwc3_otg_set_hsphy_auto_suspend(dotg, false);
	}

	return 0;
}

static int dwc3_otg_set_peripheral(struct usb_otg *otg,
				struct usb_gadget *gadget)
{
	struct dwc3_otg *dotg = container_of(otg, struct dwc3_otg, otg);
	printk(KERN_INFO "[USB] %s ,gadget %x\n",__func__,(unsigned int)gadget);
	if (gadget) {
		dev_dbg(otg->phy->dev, "%s: set gadget %s\n",
					__func__, gadget->name);
		otg->gadget = gadget;
		queue_delayed_work(system_nrt_wq, &dotg->sm_work, 0);
	} else {
		if (otg->phy->state == OTG_STATE_B_PERIPHERAL) {
			dwc3_otg_start_peripheral(otg, 0);
			otg->gadget = NULL;
			otg->phy->state = OTG_STATE_UNDEFINED;
			queue_delayed_work(system_nrt_wq, &dotg->sm_work, 0);
		} else {
			otg->gadget = NULL;
		}
	}

	return 0;
}

static void dwc3_ext_chg_det_done(struct usb_otg *otg, struct dwc3_charger *chg)
{
	struct dwc3_otg *dotg = container_of(otg, struct dwc3_otg, otg);
	printk("[USB]%s type %d\n",__func__,chg->chg_type);
	switch (chg->chg_type) {
	case DWC3_DCP_CHARGER:
	case DWC3_CDP_CHARGER:
	case DWC3_PROPRIETARY_CHARGER:
		dotg->connect_type = CONNECT_TYPE_AC;
		break;
	case DWC3_UNSUPPORTED_CHARGER:
		
		chg->chg_type = DWC3_SDP_CHARGER;
	default:
		dotg->connect_type = CONNECT_TYPE_UNKNOWN;
		dotg->ac_detect_count = 0;
		queue_delayed_work(system_nrt_wq, &dotg->ac_detect_work, 2 * HZ);
		break;
	}
	if (test_bit(B_SESS_VLD, &dotg->inputs)) {
		queue_delayed_work(system_nrt_wq, &dotg->sm_work, 0);
		queue_work(dotg->usb_wq, &dotg->notifier_work);
	}
}

int dwc3_set_charger(struct usb_otg *otg, struct dwc3_charger *charger)
{
	struct dwc3_otg *dotg = container_of(otg, struct dwc3_otg, otg);

	dotg->charger = charger;
	if (charger)
		charger->notify_detection_complete = dwc3_ext_chg_det_done;

	return 0;
}

static void dwc3_ext_event_notify(struct usb_otg *otg,
					enum dwc3_ext_events event)
{
	static bool init;
	struct dwc3_otg *dotg = container_of(otg, struct dwc3_otg, otg);
	struct dwc3_ext_xceiv *ext_xceiv = dotg->ext_xceiv;
	struct usb_phy *phy = dotg->otg.phy;
	int ret = 0;

	printk(KERN_INFO "[USB] %s : init %d ,event %d ,id %d ,vbus %d\n",__func__,init,event,ext_xceiv->id,ext_xceiv->bsv);
	
	if (init)
		flush_delayed_work(&dotg->sm_work);

	if (event == DWC3_EVENT_PHY_RESUME) {
		if (!pm_runtime_status_suspended(phy->dev)) {
			dev_warn(phy->dev, "PHY_RESUME event out of LPM!!!!\n");
		} else {
			dev_dbg(phy->dev, "ext PHY_RESUME event received\n");
			
			ret = pm_runtime_get(phy->dev);
			if ((phy->state == OTG_STATE_A_HOST) &&
							dotg->host_bus_suspend)
				dotg->host_bus_suspend = 0;
			if (ret == -EACCES) {
				pm_runtime_disable(phy->dev);
				pm_runtime_set_active(phy->dev);
				pm_runtime_enable(phy->dev);
			} else if (ret < 0) {
				dev_warn(phy->dev, "pm_runtime_get failed!\n");
			}
		}
	} else if (event == DWC3_EVENT_XCEIV_STATE) {
		if (pm_runtime_status_suspended(phy->dev)) {
			dev_warn(phy->dev, "PHY_STATE event in LPM!!!!\n");
			ret = pm_runtime_get(phy->dev);
			if (ret < 0)
				dev_warn(phy->dev, "pm_runtime_get failed!!\n");
		}
		if (ext_xceiv->id == DWC3_ID_FLOAT) {
			dev_dbg(phy->dev, "XCVR: ID set\n");
			set_bit(ID, &dotg->inputs);
		} else {
			dev_dbg(phy->dev, "XCVR: ID clear\n");
			clear_bit(ID, &dotg->inputs);
		}

		if (ext_xceiv->bsv) {
			dev_dbg(phy->dev, "XCVR: BSV set\n");
			set_bit(B_SESS_VLD, &dotg->inputs);
		} else {
			dev_dbg(phy->dev, "XCVR: BSV clear\n");
			clear_bit(B_SESS_VLD, &dotg->inputs);
		}

		if (!init) {
			init = true;
			if (!work_busy(&dotg->sm_work.work))
				queue_delayed_work(system_nrt_wq,
							&dotg->sm_work, 0);

			complete(&dotg->dwc3_xcvr_vbus_init);
			dev_dbg(phy->dev, "XCVR: BSV init complete\n");
			return;
		}

		queue_delayed_work(system_nrt_wq, &dotg->sm_work, 0);
	}
}

int dwc3_set_ext_xceiv(struct usb_otg *otg, struct dwc3_ext_xceiv *ext_xceiv)
{
	struct dwc3_otg *dotg = container_of(otg, struct dwc3_otg, otg);

	dotg->ext_xceiv = ext_xceiv;
	if (ext_xceiv)
		ext_xceiv->notify_ext_events = dwc3_ext_event_notify;

	return 0;
}

static void dwc3_otg_notify_host_mode(struct usb_otg *otg, int host_mode)
{
	struct dwc3_otg *dotg = container_of(otg, struct dwc3_otg, otg);

	if (!dotg->psy) {
		dev_err(otg->phy->dev, "no usb power supply registered\n");
		return;
	}

	if (host_mode)
		power_supply_set_scope(dotg->psy, POWER_SUPPLY_SCOPE_SYSTEM);
	else
		power_supply_set_scope(dotg->psy, POWER_SUPPLY_SCOPE_DEVICE);
}

static int dwc3_otg_set_power(struct usb_phy *phy, unsigned mA)
{
	static int power_supply_type;
	struct dwc3_otg *dotg = container_of(phy->otg, struct dwc3_otg, otg);

#if defined(CONFIG_HTC_BATT_8960)
	return 0;
#endif
	if (!dotg->psy || !dotg->charger) {
		dev_err(phy->dev, "no usb power supply/charger registered\n");
		return 0;
	}

	if (dotg->charger->charging_disabled)
		return 0;

	if (dotg->charger->chg_type == DWC3_SDP_CHARGER ||
			dotg->charger->chg_type == DWC3_PROPRIETARY_CHARGER)
		power_supply_type = POWER_SUPPLY_TYPE_USB;
	else if (dotg->charger->chg_type == DWC3_CDP_CHARGER)
		power_supply_type = POWER_SUPPLY_TYPE_USB_CDP;
	else if (dotg->charger->chg_type == DWC3_DCP_CHARGER )
		power_supply_type = POWER_SUPPLY_TYPE_USB_DCP;
	else
		power_supply_type = POWER_SUPPLY_TYPE_UNKNOWN;

	power_supply_set_supply_type(dotg->psy, power_supply_type);

	if ((dotg->charger->chg_type == DWC3_CDP_CHARGER) && mA > 2)
		mA = DWC3_IDEV_CHG_MAX;

	if (dotg->charger->max_power == mA)
		return 0;

	dev_info(phy->dev, "Avail curr from USB = %u\n", mA);

	if (dotg->charger->max_power <= 2 && mA > 2) {
		
		if (power_supply_set_online(dotg->psy, true))
			goto psy_error;
		if (power_supply_set_current_limit(dotg->psy, 1000*mA))
			goto psy_error;
	} else if (dotg->charger->max_power > 0 && (mA == 0 || mA == 2)) {
		
		if (power_supply_set_online(dotg->psy, false))
			goto psy_error;
		
		if (power_supply_set_current_limit(dotg->psy, 0))
			goto psy_error;
	}

	power_supply_changed(dotg->psy);
	dotg->charger->max_power = mA;
	return 0;

psy_error:
	dev_dbg(phy->dev, "power supply error when setting property\n");
	return -ENXIO;
}

#define DWC3_OEVT_MASK		(DWC3_OEVTEN_OTGCONIDSTSCHNGEVNT | \
				 DWC3_OEVTEN_OTGBDEVVBUSCHNGEVNT)

static irqreturn_t dwc3_otg_interrupt(int irq, void *_dotg)
{
	struct dwc3_otg *dotg = (struct dwc3_otg *)_dotg;
	u32 osts, oevt_reg;
	int ret = IRQ_NONE;
	int handled_irqs = 0;
	__maybe_unused struct usb_phy *phy = dotg->otg.phy;

	oevt_reg = dwc3_readl(dotg->regs, DWC3_OEVT);

	if (!(oevt_reg & DWC3_OEVT_MASK))
		return IRQ_NONE;

	osts = dwc3_readl(dotg->regs, DWC3_OSTS);

	if ((oevt_reg & DWC3_OEVTEN_OTGCONIDSTSCHNGEVNT) ||
	    (oevt_reg & DWC3_OEVTEN_OTGBDEVVBUSCHNGEVNT)) {

		if (oevt_reg & DWC3_OEVTEN_OTGCONIDSTSCHNGEVNT) {
			if (osts & DWC3_OTG_OSTS_CONIDSTS) {
				dev_dbg(phy->dev, "ID set\n");
				set_bit(ID, &dotg->inputs);
			} else {
				dev_dbg(phy->dev, "ID clear\n");
				clear_bit(ID, &dotg->inputs);
			}
			handled_irqs |= DWC3_OEVTEN_OTGCONIDSTSCHNGEVNT;
		}

		if (oevt_reg & DWC3_OEVTEN_OTGBDEVVBUSCHNGEVNT) {
			if (osts & DWC3_OTG_OSTS_BSESVALID) {
				dev_dbg(phy->dev, "BSV set\n");
				set_bit(B_SESS_VLD, &dotg->inputs);
			} else {
				dev_dbg(phy->dev, "BSV clear\n");
				clear_bit(B_SESS_VLD, &dotg->inputs);
			}
			handled_irqs |= DWC3_OEVTEN_OTGBDEVVBUSCHNGEVNT;
		}

		queue_delayed_work(system_nrt_wq, &dotg->sm_work, 0);

		ret = IRQ_HANDLED;

		
		dwc3_writel(dotg->regs, DWC3_OEVT, handled_irqs);
	}

	return ret;
}

void dwc3_otg_init_sm(struct dwc3_otg *dotg)
{
	u32 osts = dwc3_readl(dotg->regs, DWC3_OSTS);
	__maybe_unused struct usb_phy *phy = dotg->otg.phy;
	struct dwc3_ext_xceiv *ext_xceiv;
	int ret;

	dev_dbg(phy->dev, "Initialize OTG inputs, osts: 0x%x\n", osts);

	ret = wait_for_completion_timeout(&dotg->dwc3_xcvr_vbus_init, HZ * 15);
	if (!ret) {
		dev_err(phy->dev, "%s: completion timeout\n", __func__);
		
		set_bit(ID, &dotg->inputs);
	}

	ext_xceiv = dotg->ext_xceiv;
	dwc3_otg_reset(dotg);
}

static void dwc3_otg_sm_work(struct work_struct *w)
{
	struct dwc3_otg *dotg = container_of(w, struct dwc3_otg, sm_work.work);
	struct usb_phy *phy = dotg->otg.phy;
	struct dwc3_charger *charger = dotg->charger;
	bool work = 0;
	int ret = 0;
	unsigned long delay = 0;

	mutex_lock(&smwork_sem);
	pm_runtime_resume(phy->dev);
	printk(KERN_INFO "[USB] %s , state : %s\n",__func__,otg_state_string(phy->state));
	
	switch (phy->state) {
	case OTG_STATE_UNDEFINED:
		dwc3_otg_init_sm(dotg);
		if (!dotg->psy) {
			dotg->psy = power_supply_get_by_name("usb");

			if (!dotg->psy)
				dev_err(phy->dev,
					 "couldn't get usb power supply\n");
		}

		
		if (!test_bit(ID, &dotg->inputs)) {
			dev_dbg(phy->dev, "!id\n");
			phy->state = OTG_STATE_A_IDLE;
			work = 1;
		} else if (test_bit(B_SESS_VLD, &dotg->inputs)) {
			dev_dbg(phy->dev, "b_sess_vld\n");
			phy->state = OTG_STATE_B_IDLE;
			work = 1;
		} else {
			phy->state = OTG_STATE_B_IDLE;
			if (dotg->connect_type != CONNECT_TYPE_NONE) {
				dotg->connect_type = CONNECT_TYPE_NONE;
				queue_work(dotg->usb_wq, &dotg->notifier_work);
			}
			dev_dbg(phy->dev, "No device, trying to suspend\n");
			pm_runtime_put_sync(phy->dev);
		}
		break;

	case OTG_STATE_B_IDLE:
		if (!test_bit(ID, &dotg->inputs)) {
			dev_dbg(phy->dev, "!id\n");
			phy->state = OTG_STATE_A_IDLE;
			work = 1;
			__cancel_delayed_work(&dotg->ac_detect_work);
			if (dotg->connect_type != CONNECT_TYPE_NONE) {
				dotg->connect_type = CONNECT_TYPE_NONE;
				queue_work(dotg->usb_wq, &dotg->notifier_work);
			}

			dotg->charger_retry_count = 0;
			if (charger) {
				if (charger->chg_type == DWC3_INVALID_CHARGER)
					charger->start_detection(dotg->charger,
									false);
				else
					charger->chg_type =
							DWC3_INVALID_CHARGER;
			}
		} else if (test_bit(B_SESS_VLD, &dotg->inputs)) {
			dev_dbg(phy->dev, "b_sess_vld\n");
			if (charger) {
				printk(KERN_INFO "[USB] %s , b_sess_vld, chg_type : %d\n",__func__, charger->chg_type);
				
				switch (charger->chg_type) {
				case DWC3_STOP_CHARGE_CASE:
					
					dev_dbg(phy->dev, "lpm, invalid charger\n");
					dwc3_otg_set_power(phy, 0);
					pm_runtime_put_sync(phy->dev);
					break;
				case DWC3_DCP_CHARGER:
					dev_dbg(phy->dev, "lpm, DCP charger\n");
					dwc3_otg_set_power(phy,
							DWC3_IDEV_CHG_MAX);
					pm_runtime_put_sync(phy->dev);
					break;
				case DWC3_CDP_CHARGER:
					dwc3_otg_set_power(phy,
							DWC3_IDEV_CHG_MAX);
					dwc3_otg_start_peripheral(&dotg->otg,
									1);
					phy->state = OTG_STATE_B_PERIPHERAL;
					work = 1;
					break;
				case DWC3_SDP_CHARGER:
				case DWC3_PROPRIETARY_CHARGER:
					printk("%s: usb_disable = %d\n", __func__, dotg->charger->usb_disable);
					if (dotg->charger->usb_disable)
						break;
					dwc3_otg_start_peripheral(&dotg->otg,
									1);
					phy->state = OTG_STATE_B_PERIPHERAL;
					work = 1;
					break;
				case DWC3_FLOATED_CHARGER:
					if (dotg->charger_retry_count <
							max_chgr_retry_count)
						dotg->charger_retry_count++;
					if (dotg->charger_retry_count ==
						max_chgr_retry_count) {
						dwc3_otg_set_power(phy, 0);
						pm_runtime_put_sync(phy->dev);
						break;
					}
					charger->start_detection(dotg->charger,
									false);
					dev_dbg(phy->dev, "chg_det started\n");
					charger->start_detection(charger, true);
					break;
				case DWC3_UNSUPPORTED_CHARGER:
					dotg->charger_retry_count++;
					if (dotg->charger_retry_count ==
						max_chgr_retry_count) {
						dwc3_otg_set_power(phy, 0);
						pm_runtime_put_sync(phy->dev);
						break;
					}
					charger->start_detection(dotg->charger,
									false);

				default:
					dev_dbg(phy->dev, "chg_det started\n");
					charger->start_detection(charger, true);
					break;
				}
			} else {
				
				if (dwc3_otg_start_peripheral(&dotg->otg, 1)) {
					dev_err(phy->dev, "enter lpm as\n"
						"unable to start B-device\n");
					phy->state = OTG_STATE_UNDEFINED;
					pm_runtime_put_sync(phy->dev);
					return;
				}
			}
		} else {
			__cancel_delayed_work(&dotg->ac_detect_work);
			if (charger)
				charger->start_detection(dotg->charger, false);

			dotg->charger_retry_count = 0;
			dwc3_otg_set_power(phy, 0);
			if (dotg->connect_type != CONNECT_TYPE_NONE) {
				if (charger)
					if (vbus && dotg->charger->usb_disable)
						break;
				dotg->connect_type = CONNECT_TYPE_NONE;
				queue_work(dotg->usb_wq, &dotg->notifier_work);
			}
			dev_dbg(phy->dev, "No device, trying to suspend\n");
			pm_runtime_put_sync(phy->dev);
		}
		break;

	case OTG_STATE_B_PERIPHERAL:
		if (!test_bit(B_SESS_VLD, &dotg->inputs) ||
				!test_bit(ID, &dotg->inputs)) {
			dotg->ac_detect_count = 0;
			cancel_delayed_work_sync(&dotg->ac_detect_work);
			if (dotg->connect_type != CONNECT_TYPE_NONE && !dotg->charger->usb_disable) {
				dotg->connect_type = CONNECT_TYPE_NONE;
				queue_work(dotg->usb_wq, &dotg->notifier_work);
			}
			dev_dbg(phy->dev, "!id || !bsv\n");
			dwc3_otg_start_peripheral(&dotg->otg, 0);
			phy->state = OTG_STATE_B_IDLE;
			if (charger)
				charger->chg_type = DWC3_INVALID_CHARGER;
			work = 1;
		}
		break;

	case OTG_STATE_A_IDLE:
		
		if (test_bit(ID, &dotg->inputs)) {
			dev_dbg(phy->dev, "id\n");
			phy->state = OTG_STATE_B_IDLE;
			dotg->vbus_retry_count = 0;
			work = 1;
		} else {
			phy->state = OTG_STATE_A_HOST;
			ret = dwc3_otg_start_host(&dotg->otg, 1);
			if ((ret == -EPROBE_DEFER) &&
						dotg->vbus_retry_count < 3) {
				phy->state = OTG_STATE_A_IDLE;
				dev_dbg(phy->dev, "Unable to get vbus regulator. Retrying...\n");
				delay = VBUS_REG_CHECK_DELAY;
				work = 1;
				dotg->vbus_retry_count++;
			} else if (ret) {
				dev_dbg(phy->dev, "enter lpm as\n"
					"unable to start A-device\n");
				phy->state = OTG_STATE_UNDEFINED;
				pm_runtime_put_sync(phy->dev);
				return;
			}
		}
		break;

	case OTG_STATE_A_HOST:
		if (test_bit(ID, &dotg->inputs)) {
			dev_dbg(phy->dev, "id\n");
			dwc3_otg_start_host(&dotg->otg, 0);
			phy->state = OTG_STATE_B_IDLE;
			clear_bit(B_SESS_VLD, &dotg->inputs);
			dotg->vbus_retry_count = 0;
			work = 1;
		}
		break;

	default:
		dev_err(phy->dev, "%s: invalid otg-state\n", __func__);

	}

	mutex_unlock(&smwork_sem);
	if (work)
		queue_delayed_work(system_nrt_wq, &dotg->sm_work, delay);
}


static void dwc3_otg_reset(struct dwc3_otg *dotg)
{
	static int once;
	struct dwc3_ext_xceiv *ext_xceiv = dotg->ext_xceiv;

	if (ext_xceiv && !ext_xceiv->otg_capability)
		dwc3_writel(dotg->regs, DWC3_OCFG, 0x4);

	if (!once) {
		if (ext_xceiv && !ext_xceiv->otg_capability)
			dwc3_writel(dotg->regs, DWC3_OCTL, 0x40);
		once++;
	}

	
	dwc3_writel(dotg->regs, DWC3_OEVT, 0xFFFF);

	
	if (ext_xceiv && !ext_xceiv->otg_capability)
		dwc3_writel(dotg->regs, DWC3_OEVTEN,
				DWC3_OEVTEN_OTGCONIDSTSCHNGEVNT |
				DWC3_OEVTEN_OTGBDEVVBUSCHNGEVNT);
}

static void dwc3_notify_usb_attached(struct usb_phy *phy)
{
	struct usb_otg *otg = phy->otg;
	struct dwc3_otg *dotg = container_of(otg, struct dwc3_otg, otg);

	if (dotg->connect_type != CONNECT_TYPE_USB) {
		dotg->connect_type = CONNECT_TYPE_USB;
		queue_work(dotg->usb_wq, &dotg->notifier_work);
	}

	dotg->ac_detect_count = 0;
	__cancel_delayed_work(&dotg->ac_detect_work);
}



static void ac_detect_expired_work(struct work_struct *w)
{
	struct delayed_work *dw = container_of(w, struct delayed_work, work);
	struct dwc3_otg *dotg = container_of(dw, struct dwc3_otg, ac_detect_work);
	u32 line_state;

	line_state = htc_dwc3_get_line_state(dotg->dwc);
	USBH_INFO("%s: count = %d, connect_type = %d\n", __func__,
			dotg->ac_detect_count, dotg->connect_type);

	if (dotg->connect_type == CONNECT_TYPE_USB || dotg->ac_detect_count >= 10)
		return;

	printk("[USB] %s: line state = %x\n",__func__,(line_state & (3 << 8)));
	
	if (line_state != (3 << 8)) {
#ifdef CONFIG_CABLE_DETECT_ACCESSORY
		if (cable_get_accessory_type() == DOCK_STATE_CAR
			||cable_get_accessory_type() == DOCK_STATE_AUDIO_DOCK) {
				USBH_INFO("car/audio dock mode charger\n");
				dotg->connect_type = CONNECT_TYPE_AC;
				if (dotg->charger)
					dotg->charger->chg_type = DWC3_DCP_CHARGER;
				dotg->ac_detect_count = 0;
				dotg->otg.phy->state = OTG_STATE_B_IDLE;

				dwc3_otg_start_peripheral(&dotg->otg, 0);
				queue_delayed_work(system_nrt_wq, &dotg->sm_work, 0);
				queue_work(dotg->usb_wq, &dotg->notifier_work);
				return;
		}
		{
			int dock_result = check_three_pogo_dock();
			if (dock_result == 2) {
				USBH_INFO("three pogo dock AC type\n");
				dotg->connect_type = CONNECT_TYPE_AC;
				if (dotg->charger)
					dotg->charger->chg_type = DWC3_DCP_CHARGER;
				dotg->ac_detect_count = 0;
				dotg->otg.phy->state = OTG_STATE_B_IDLE;
				dwc3_otg_start_peripheral(&dotg->otg, 0);

				queue_delayed_work(system_nrt_wq, &dotg->sm_work, 0);
				queue_work(dotg->usb_wq, &dotg->notifier_work);
				return;
			} else if (dock_result == 1) {
				USBH_INFO("three pogo dock USB type\n");
				dotg->connect_type = CONNECT_TYPE_NONE;
				if (dotg->charger)
					dotg->charger->chg_type = DWC3_STOP_CHARGE_CASE;
				dotg->ac_detect_count = 0;
				dotg->otg.phy->state = OTG_STATE_B_IDLE;
				dwc3_otg_start_peripheral(&dotg->otg, 0);
				
				queue_delayed_work(system_nrt_wq, &dotg->sm_work, 0);
				queue_work(dotg->usb_wq, &dotg->notifier_work);
				return;
			}
		}
#endif
		dotg->ac_detect_count++;
		queue_delayed_work(system_nrt_wq, &dotg->ac_detect_work, 2 * HZ);
	} else {
		USBH_INFO("AC charger\n");
		dotg->connect_type = CONNECT_TYPE_AC;
		if (dotg->charger)
			dotg->charger->chg_type = DWC3_DCP_CHARGER;
		dotg->ac_detect_count = 0;
		dotg->otg.phy->state = OTG_STATE_B_IDLE;
		dwc3_otg_start_peripheral(&dotg->otg, 0);

		queue_delayed_work(system_nrt_wq, &dotg->sm_work, 0);
		queue_work(dotg->usb_wq, &dotg->notifier_work);
	}
}


int htc_dwc3_get_cable_type(void)
{
	if (!the_dwc3_otg) {
		printk(KERN_INFO "[USB] %s : usb function not ready\n",__func__);
		return 0;
	}
	return the_dwc3_otg->charger->chg_type;
}


static const char *event_string(enum usb_otg_event event)
{
	switch (event) {
	case OTG_EVENT_DEV_CONN_TMOUT:
		return "DEV_CONN_TMOUT";
	case OTG_EVENT_NO_RESP_FOR_HNP_ENABLE:
		return "NO_RESP_FOR_HNP_ENABLE";
	case OTG_EVENT_HUB_NOT_SUPPORTED:
		return "HUB_NOT_SUPPORTED";
	case OTG_EVENT_DEV_NOT_SUPPORTED:
		return "DEV_NOT_SUPPORTED";
	case OTG_EVENT_HNP_FAILED:
		return "HNP_FAILED";
	case OTG_EVENT_NO_RESP_FOR_SRP:
		return "NO_RESP_FOR_SRP";
	case OTG_EVENT_INSUFFICIENT_POWER:
		return "DEV_NOT_SUPPORTED";
	default:
		return "UNDEFINED";
	}
}

static int dwc3_otg_send_event(struct usb_otg *otg, enum usb_otg_event event)
{

	char module_name[16];
	char udev_event[128];
	char *envp[] = { module_name, udev_event, NULL };
	int ret = 0;
	
	switch (event) {
		case OTG_EVENT_INSUFFICIENT_POWER:
		case OTG_EVENT_DEV_NOT_SUPPORTED:
			printk(KERN_INFO "[USB] sending %s event\n", event_string(event));
			snprintf(module_name, 16, "MODULE=%s", "dwc3");
			snprintf(udev_event, 128, "EVENT=%s", event_string(event));
			ret = kobject_uevent_env(&otg->phy->dev->kobj, KOBJ_CHANGE, envp);
			if (ret < 0)
				printk(KERN_INFO "[USB] uevent sending failed with ret = %d\n", ret);
			break;
		default:
			break;
	}
	return ret;

}

int dwc3_otg_init(struct dwc3 *dwc)
{
	u32	reg;
	int ret = 0;
	struct dwc3_otg *dotg;

	dev_dbg(dwc->dev, "dwc3_otg_init\n");


	reg = dwc3_readl(dwc->regs, DWC3_GHWPARAMS6);
	if (!(reg & DWC3_GHWPARAMS6_SRP_SUPPORT)) {
		dev_dbg(dwc->dev, "dwc3_otg address space is not supported\n");
		return 0;
	}

	
	dotg = kzalloc(sizeof(struct dwc3_otg), GFP_KERNEL);
	if (!dotg) {
		dev_err(dwc->dev, "unable to allocate dwc3_otg\n");
		return -ENOMEM;
	}

	
	dotg->irq = platform_get_irq_byname(to_platform_device(dwc->dev),
								"otg_irq");
	if (dotg->irq < 0) {
		dev_err(dwc->dev, "%s: missing OTG IRQ\n", __func__);
		ret = -ENODEV;
		goto err1;
	}

	dotg->regs = dwc->regs;

	dotg->otg.set_peripheral = dwc3_otg_set_peripheral;
	dotg->otg.set_host = dwc3_otg_set_host;
	dotg->otg.send_event = dwc3_otg_send_event;

	
	dwc->dotg = dotg;
	the_dwc3_otg = dotg;
	dotg->connect_type_ready = 0;

	dotg->otg.phy = kzalloc(sizeof(struct usb_phy), GFP_KERNEL);
	if (!dotg->otg.phy) {
		dev_err(dwc->dev, "unable to allocate dwc3_otg.phy\n");
		ret = -ENOMEM;
		goto err1;
	}

	dotg->dwc = dwc;
	dotg->otg.phy->otg = &dotg->otg;
	dotg->otg.phy->dev = dwc->dev;
	dotg->otg.phy->set_power = dwc3_otg_set_power;
	dotg->otg.phy->set_suspend = dwc3_otg_set_suspend;
	dotg->otg.phy->notify_usb_attached = dwc3_notify_usb_attached;

	dotg->ac_detect_count = 0;
	set_bit(ID, &dotg->inputs);
	dotg->otg.phy->set_phy_autosuspend = dwc3_otg_set_autosuspend;

	ret = usb_set_transceiver(dotg->otg.phy);
	if (ret) {
		dev_err(dotg->otg.phy->dev,
			"%s: failed to set transceiver, already exists\n",
			__func__);
		goto err2;
	}

	dotg->otg.phy->state = OTG_STATE_UNDEFINED;

	init_completion(&dotg->dwc3_xcvr_vbus_init);
	INIT_DELAYED_WORK(&dotg->sm_work, dwc3_otg_sm_work);
	INIT_DELAYED_WORK(&dotg->ac_detect_work, ac_detect_expired_work);

	dotg->usb_wq = create_singlethread_workqueue("msm_hsusb");
	if (dotg->usb_wq == 0) {
		USB_ERR("fail to create workqueue\n");
		goto err3;
	}
	INIT_WORK(&dotg->notifier_work, send_usb_connect_notify);

	ret = request_irq(dotg->irq, dwc3_otg_interrupt, IRQF_SHARED,
				"dwc3_otg", dotg);
	if (ret) {
		dev_err(dotg->otg.phy->dev, "failed to request irq #%d --> %d\n",
				dotg->irq, ret);
		goto err3;
	}

	pm_runtime_get(dwc->dev);

	return 0;

err3:
	cancel_delayed_work_sync(&dotg->sm_work);
	usb_set_transceiver(NULL);
err2:
	kfree(dotg->otg.phy);
err1:
	dwc->dotg = NULL;
	kfree(dotg);

	return ret;
}

void dwc3_otg_exit(struct dwc3 *dwc)
{
	struct dwc3_otg *dotg = dwc->dotg;

	
	if (dotg) {
		if (dotg->charger)
			dotg->charger->start_detection(dotg->charger, false);
		cancel_delayed_work_sync(&dotg->sm_work);
		usb_set_transceiver(NULL);
		pm_runtime_put(dwc->dev);
		free_irq(dotg->irq, dotg);
		kfree(dotg->otg.phy);
		kfree(dotg);
		dwc->dotg = NULL;
	}
}
