/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/idr.h>
#include <linux/pm_runtime.h>
#include <linux/slimbus/slimbus.h>

#define SLIM_PORT_HDL(la, f, p) ((la)<<24 | (f) << 16 | (p))

#define SLIM_HDL_TO_LA(hdl)	((u32)((hdl) & 0xFF000000) >> 24)
#define SLIM_HDL_TO_FLOW(hdl)	(((u32)(hdl) & 0xFF0000) >> 16)
#define SLIM_HDL_TO_PORT(hdl)	((u32)(hdl) & 0xFF)

#define SLIM_HDL_TO_CHIDX(hdl)	((u16)(hdl) & 0xFF)
#define SLIM_GRP_TO_NCHAN(hdl)	((u16)(hdl >> 8) & 0xFF)

#define SLIM_SLAVE_PORT(p, la)	(((la)<<16) | (p))
#define SLIM_MGR_PORT(p)	((0xFF << 16) | (p))
#define SLIM_LA_MANAGER		0xFF

#define SLIM_START_GRP		(1 << 8)
#define SLIM_END_GRP		(1 << 9)

#define SLIM_MAX_INTR_COEFF_3	(SLIM_SL_PER_SUPERFRAME/3)
#define SLIM_MAX_INTR_COEFF_1	SLIM_SL_PER_SUPERFRAME

static DEFINE_MUTEX(slim_lock);
static DEFINE_IDR(ctrl_idr);
static struct device_type slim_dev_type;
static struct device_type slim_ctrl_type;

static const struct slim_device_id *slim_match(const struct slim_device_id *id,
					const struct slim_device *slim_dev)
{
	while (id->name[0]) {
		if (strncmp(slim_dev->name, id->name, SLIMBUS_NAME_SIZE) == 0)
			return id;
		id++;
	}
	return NULL;
}

static int slim_device_match(struct device *dev, struct device_driver *driver)
{
	struct slim_device *slim_dev;
	struct slim_driver *drv = to_slim_driver(driver);

	if (dev->type == &slim_dev_type)
		slim_dev = to_slim_device(dev);
	else
		return 0;
	if (drv->id_table)
		return slim_match(drv->id_table, slim_dev) != NULL;

	if (driver->name)
		return strncmp(slim_dev->name, driver->name, SLIMBUS_NAME_SIZE)
			== 0;
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int slim_legacy_suspend(struct device *dev, pm_message_t mesg)
{
	struct slim_device *slim_dev = NULL;
	struct slim_driver *driver;
	if (dev->type == &slim_dev_type)
		slim_dev = to_slim_device(dev);

	if (!slim_dev || !dev->driver)
		return 0;

	driver = to_slim_driver(dev->driver);
	if (!driver->suspend)
		return 0;

	return driver->suspend(slim_dev, mesg);
}

static int slim_legacy_resume(struct device *dev)
{
	struct slim_device *slim_dev = NULL;
	struct slim_driver *driver;
	if (dev->type == &slim_dev_type)
		slim_dev = to_slim_device(dev);

	if (!slim_dev || !dev->driver)
		return 0;

	driver = to_slim_driver(dev->driver);
	if (!driver->resume)
		return 0;

	return driver->resume(slim_dev);
}

static int slim_pm_suspend(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_suspend(dev);
	else
		return slim_legacy_suspend(dev, PMSG_SUSPEND);
}

static int slim_pm_resume(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_resume(dev);
	else
		return slim_legacy_resume(dev);
}

#else
#define slim_pm_suspend		NULL
#define slim_pm_resume		NULL
#endif

static const struct dev_pm_ops slimbus_pm = {
	.suspend = slim_pm_suspend,
	.resume = slim_pm_resume,
	SET_RUNTIME_PM_OPS(
		pm_generic_suspend,
		pm_generic_resume,
		pm_generic_runtime_idle
		)
};
struct bus_type slimbus_type = {
	.name		= "slimbus",
	.match		= slim_device_match,
	.pm		= &slimbus_pm,
};
EXPORT_SYMBOL_GPL(slimbus_type);

struct device slimbus_dev = {
	.init_name = "slimbus",
};

static void __exit slimbus_exit(void)
{
	device_unregister(&slimbus_dev);
	bus_unregister(&slimbus_type);
}

static int __init slimbus_init(void)
{
	int retval;

	retval = bus_register(&slimbus_type);
	if (!retval)
		retval = device_register(&slimbus_dev);

	if (retval)
		bus_unregister(&slimbus_type);

	return retval;
}
postcore_initcall(slimbus_init);
module_exit(slimbus_exit);

static int slim_drv_probe(struct device *dev)
{
	const struct slim_driver *sdrv = to_slim_driver(dev->driver);
	struct slim_device *sbdev = to_slim_device(dev);
	struct slim_controller *ctrl = sbdev->ctrl;

	if (sdrv->probe) {
		int ret;
		ret = sdrv->probe(sbdev);
		if (ret)
			return ret;
		if (sdrv->device_up)
			queue_work(ctrl->wq, &sbdev->wd);
		return 0;
	}
	return -ENODEV;
}

static int slim_drv_remove(struct device *dev)
{
	const struct slim_driver *sdrv = to_slim_driver(dev->driver);
	struct slim_device *sbdev = to_slim_device(dev);

	sbdev->notified = false;
	if (sdrv->remove)
		return sdrv->remove(to_slim_device(dev));
	return -ENODEV;
}

static void slim_drv_shutdown(struct device *dev)
{
	const struct slim_driver *sdrv = to_slim_driver(dev->driver);

	if (sdrv->shutdown)
		sdrv->shutdown(to_slim_device(dev));
}

int slim_driver_register(struct slim_driver *drv)
{
	drv->driver.bus = &slimbus_type;
	if (drv->probe)
		drv->driver.probe = slim_drv_probe;

	if (drv->remove)
		drv->driver.remove = slim_drv_remove;

	if (drv->shutdown)
		drv->driver.shutdown = slim_drv_shutdown;

	return driver_register(&drv->driver);
}
EXPORT_SYMBOL_GPL(slim_driver_register);

void slim_driver_unregister(struct slim_driver *drv)
{
	if (drv)
		driver_unregister(&drv->driver);
}
EXPORT_SYMBOL_GPL(slim_driver_unregister);

#define slim_ctrl_attr_gr NULL

static void slim_ctrl_release(struct device *dev)
{
	struct slim_controller *ctrl = to_slim_controller(dev);

	complete(&ctrl->dev_released);
}

static struct device_type slim_ctrl_type = {
	.groups		= slim_ctrl_attr_gr,
	.release	= slim_ctrl_release,
};

static struct slim_controller *slim_ctrl_get(struct slim_controller *ctrl)
{
	if (!ctrl || !get_device(&ctrl->dev))
		return NULL;

	return ctrl;
}

static void slim_ctrl_put(struct slim_controller *ctrl)
{
	if (ctrl)
		put_device(&ctrl->dev);
}

#define slim_device_attr_gr NULL
#define slim_device_uevent NULL
static void slim_dev_release(struct device *dev)
{
	struct slim_device *sbdev = to_slim_device(dev);
	slim_ctrl_put(sbdev->ctrl);
}

static struct device_type slim_dev_type = {
	.groups		= slim_device_attr_gr,
	.uevent		= slim_device_uevent,
	.release	= slim_dev_release,
};

static void slim_report(struct work_struct *work)
{
	struct slim_driver *sbdrv;
	struct slim_device *sbdev =
			container_of(work, struct slim_device, wd);

	pr_info("%s\n",__func__);
	if (!sbdev->dev.driver)
		return;
	
	if ((!sbdev->reported && !sbdev->notified) ||
			(sbdev->reported && sbdev->notified))
		return;

	sbdrv = to_slim_driver(sbdev->dev.driver);
	if (sbdev->notified && !sbdev->reported) {
		sbdev->notified = false;
		if (sbdrv->device_down)
			sbdrv->device_down(sbdev);
	} else if (!sbdev->notified && sbdev->reported) {
		sbdev->notified = true;
		if (sbdrv->device_up)
			sbdrv->device_up(sbdev);
	}
}

int slim_add_device(struct slim_controller *ctrl, struct slim_device *sbdev)
{
	sbdev->dev.bus = &slimbus_type;
	sbdev->dev.parent = ctrl->dev.parent;
	sbdev->dev.type = &slim_dev_type;
	sbdev->dev.driver = NULL;
	sbdev->ctrl = ctrl;
	slim_ctrl_get(ctrl);
	dev_set_name(&sbdev->dev, "%s", sbdev->name);
	mutex_init(&sbdev->sldev_reconf);
	INIT_LIST_HEAD(&sbdev->mark_define);
	INIT_LIST_HEAD(&sbdev->mark_suspend);
	INIT_LIST_HEAD(&sbdev->mark_removal);
	INIT_WORK(&sbdev->wd, slim_report);
	mutex_lock(&ctrl->m_ctrl);
	list_add_tail(&sbdev->dev_list, &ctrl->devs);
	pr_info("add slim dev %s\n",dev_name(&sbdev->dev));
	mutex_unlock(&ctrl->m_ctrl);
	
	return device_register(&sbdev->dev);
}
EXPORT_SYMBOL_GPL(slim_add_device);

struct sbi_boardinfo {
	struct list_head	list;
	struct slim_boardinfo	board_info;
};

static LIST_HEAD(board_list);
static LIST_HEAD(slim_ctrl_list);
static DEFINE_MUTEX(board_lock);

static void slim_match_ctrl_to_boardinfo(struct slim_controller *ctrl,
				struct slim_boardinfo *bi)
{
	int ret;
	if (ctrl->nr != bi->bus_num)
		return;

	ret = slim_add_device(ctrl, bi->slim_slave);
	if (ret != 0)
		dev_err(ctrl->dev.parent, "can't create new device for %s\n",
			bi->slim_slave->name);
}

int slim_register_board_info(struct slim_boardinfo const *info, unsigned n)
{
	struct sbi_boardinfo *bi;
	int i;

	bi = kzalloc(n * sizeof(*bi), GFP_KERNEL);
	if (!bi)
		return -ENOMEM;

	for (i = 0; i < n; i++, bi++, info++) {
		struct slim_controller *ctrl;

		memcpy(&bi->board_info, info, sizeof(*info));
		mutex_lock(&board_lock);
		list_add_tail(&bi->list, &board_list);
		list_for_each_entry(ctrl, &slim_ctrl_list, list)
			slim_match_ctrl_to_boardinfo(ctrl, &bi->board_info);
		mutex_unlock(&board_lock);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(slim_register_board_info);

void slim_ctrl_add_boarddevs(struct slim_controller *ctrl)
{
	struct sbi_boardinfo *bi;
	mutex_lock(&board_lock);
	list_add_tail(&ctrl->list, &slim_ctrl_list);
	list_for_each_entry(bi, &board_list, list)
		slim_match_ctrl_to_boardinfo(ctrl, &bi->board_info);
	mutex_unlock(&board_lock);
}
EXPORT_SYMBOL_GPL(slim_ctrl_add_boarddevs);

struct slim_controller *slim_busnum_to_ctrl(u32 bus_num)
{
	struct slim_controller *ctrl;
	mutex_lock(&board_lock);
	list_for_each_entry(ctrl, &slim_ctrl_list, list)
		if (bus_num == ctrl->nr) {
			mutex_unlock(&board_lock);
			return ctrl;
		}
	mutex_unlock(&board_lock);
	return NULL;
}
EXPORT_SYMBOL_GPL(slim_busnum_to_ctrl);

static int slim_register_controller(struct slim_controller *ctrl)
{
	int ret = 0;

	
	if (WARN_ON(!slimbus_type.p)) {
		ret = -EAGAIN;
		goto out_list;
	}

	dev_set_name(&ctrl->dev, "sb-%d", ctrl->nr);
	ctrl->dev.bus = &slimbus_type;
	ctrl->dev.type = &slim_ctrl_type;
	ctrl->num_dev = 0;
	if (!ctrl->min_cg)
		ctrl->min_cg = SLIM_MIN_CLK_GEAR;
	if (!ctrl->max_cg)
		ctrl->max_cg = SLIM_MAX_CLK_GEAR;
	mutex_init(&ctrl->m_ctrl);
	mutex_init(&ctrl->sched.m_reconf);
	ret = device_register(&ctrl->dev);
	if (ret)
		goto out_list;

	dev_dbg(&ctrl->dev, "Bus [%s] registered:dev:%x\n", ctrl->name,
							(u32)&ctrl->dev);

	if (ctrl->nports) {
		ctrl->ports = kzalloc(ctrl->nports * sizeof(struct slim_port),
					GFP_KERNEL);
		if (!ctrl->ports) {
			ret = -ENOMEM;
			goto err_port_failed;
		}
	}
	if (ctrl->nchans) {
		ctrl->chans = kzalloc(ctrl->nchans * sizeof(struct slim_ich),
					GFP_KERNEL);
		if (!ctrl->chans) {
			ret = -ENOMEM;
			goto err_chan_failed;
		}

		ctrl->sched.chc1 =
			kzalloc(ctrl->nchans * sizeof(struct slim_ich *),
			GFP_KERNEL);
		if (!ctrl->sched.chc1) {
			kfree(ctrl->chans);
			ret = -ENOMEM;
			goto err_chan_failed;
		}
		ctrl->sched.chc3 =
			kzalloc(ctrl->nchans * sizeof(struct slim_ich *),
			GFP_KERNEL);
		if (!ctrl->sched.chc3) {
			kfree(ctrl->sched.chc1);
			kfree(ctrl->chans);
			ret = -ENOMEM;
			goto err_chan_failed;
		}
	}
#ifdef DEBUG
	ctrl->sched.slots = kzalloc(SLIM_SL_PER_SUPERFRAME, GFP_KERNEL);
#endif
	init_completion(&ctrl->pause_comp);

	INIT_LIST_HEAD(&ctrl->devs);
	ctrl->wq = create_singlethread_workqueue(dev_name(&ctrl->dev));
	if (!ctrl->wq)
		goto err_workq_failed;

	return 0;

err_workq_failed:
	kfree(ctrl->sched.chc3);
	kfree(ctrl->sched.chc1);
	kfree(ctrl->chans);
err_chan_failed:
	kfree(ctrl->ports);
err_port_failed:
	device_unregister(&ctrl->dev);
out_list:
	mutex_lock(&slim_lock);
	idr_remove(&ctrl_idr, ctrl->nr);
	mutex_unlock(&slim_lock);
	return ret;
}

void slim_remove_device(struct slim_device *sbdev)
{
	device_unregister(&sbdev->dev);
}
EXPORT_SYMBOL_GPL(slim_remove_device);

static void slim_ctrl_remove_device(struct slim_controller *ctrl,
				struct slim_boardinfo *bi)
{
	if (ctrl->nr == bi->bus_num)
		slim_remove_device(bi->slim_slave);
}

int slim_del_controller(struct slim_controller *ctrl)
{
	struct slim_controller *found;
	struct sbi_boardinfo *bi;

	
	mutex_lock(&slim_lock);
	found = idr_find(&ctrl_idr, ctrl->nr);
	mutex_unlock(&slim_lock);
	if (found != ctrl)
		return -EINVAL;

	
	mutex_lock(&board_lock);
	list_for_each_entry(bi, &board_list, list)
		slim_ctrl_remove_device(ctrl, &bi->board_info);
	mutex_unlock(&board_lock);

	init_completion(&ctrl->dev_released);
	device_unregister(&ctrl->dev);

	wait_for_completion(&ctrl->dev_released);
	list_del(&ctrl->list);
	destroy_workqueue(ctrl->wq);
	
	mutex_lock(&slim_lock);
	idr_remove(&ctrl_idr, ctrl->nr);
	mutex_unlock(&slim_lock);

	kfree(ctrl->sched.chc1);
	kfree(ctrl->sched.chc3);
#ifdef DEBUG
	kfree(ctrl->sched.slots);
#endif
	kfree(ctrl->chans);
	kfree(ctrl->ports);

	return 0;
}
EXPORT_SYMBOL_GPL(slim_del_controller);

int slim_add_numbered_controller(struct slim_controller *ctrl)
{
	int	id;
	int	status;

	if (ctrl->nr & ~MAX_ID_MASK)
		return -EINVAL;

retry:
	if (idr_pre_get(&ctrl_idr, GFP_KERNEL) == 0)
		return -ENOMEM;

	mutex_lock(&slim_lock);
	status = idr_get_new_above(&ctrl_idr, ctrl, ctrl->nr, &id);
	if (status == 0 && id != ctrl->nr) {
		status = -EAGAIN;
		idr_remove(&ctrl_idr, id);
	}
	mutex_unlock(&slim_lock);
	if (status == -EAGAIN)
		goto retry;

	if (status == 0)
		status = slim_register_controller(ctrl);
	return status;
}
EXPORT_SYMBOL_GPL(slim_add_numbered_controller);

void slim_report_absent(struct slim_device *sbdev)
{
	struct slim_controller *ctrl;
	int i;
	if (!sbdev)
		return;
	ctrl = sbdev->ctrl;
	if (!ctrl)
		return;
	
	mutex_lock(&ctrl->m_ctrl);
	for (i = 0; i < ctrl->num_dev; i++) {
		if (sbdev->laddr == ctrl->addrt[i].laddr)
			ctrl->addrt[i].valid = false;
	}
	mutex_unlock(&ctrl->m_ctrl);
	sbdev->reported = false;
	queue_work(ctrl->wq, &sbdev->wd);
}
EXPORT_SYMBOL(slim_report_absent);

void slim_framer_booted(struct slim_controller *ctrl)
{
	struct slim_device *sbdev;
	struct list_head *pos, *next;
	if (!ctrl)
		return;
	mutex_lock(&ctrl->m_ctrl);
	list_for_each_safe(pos, next, &ctrl->devs) {
		struct slim_driver *sbdrv;
		if (pos)
			sbdev = list_entry(pos, struct slim_device, dev_list);
		mutex_unlock(&ctrl->m_ctrl);
		if (sbdev && sbdev->dev.driver) {
			sbdrv = to_slim_driver(sbdev->dev.driver);
			if (sbdrv->reset_device)
				sbdrv->reset_device(sbdev);
		}
		mutex_lock(&ctrl->m_ctrl);
	}
	mutex_unlock(&ctrl->m_ctrl);
}
EXPORT_SYMBOL(slim_framer_booted);

void slim_msg_response(struct slim_controller *ctrl, u8 *reply, u8 tid, u8 len)
{
	int i;
	struct slim_msg_txn *txn;

	mutex_lock(&ctrl->m_ctrl);
	txn = ctrl->txnt[tid];
	if (txn == NULL || txn->rbuf == NULL) {
		if (txn == NULL)
			dev_err(&ctrl->dev, "Got response to invalid TID:%d, len:%d",
				tid, len);
		else
			dev_err(&ctrl->dev, "Invalid client buffer passed\n");
		mutex_unlock(&ctrl->m_ctrl);
		return;
	}
	for (i = 0; i < len; i++)
		txn->rbuf[i] = reply[i];
	if (txn->comp)
		complete(txn->comp);
	ctrl->txnt[tid] = NULL;
	mutex_unlock(&ctrl->m_ctrl);
	kfree(txn);
}
EXPORT_SYMBOL_GPL(slim_msg_response);

static int slim_processtxn(struct slim_controller *ctrl, u8 dt, u16 mc, u16 ec,
			u8 mt, u8 *rbuf, const u8 *wbuf, u8 len, u8 mlen,
			struct completion *comp, u8 la, u8 *tid)
{
	u8 i = 0;
	int ret = 0;
	struct slim_msg_txn *txn = kmalloc(sizeof(struct slim_msg_txn),
					GFP_KERNEL);
	if (!txn)
		return -ENOMEM;
	if (tid) {
		mutex_lock(&ctrl->m_ctrl);
		for (i = 0; i < ctrl->last_tid; i++) {
			if (ctrl->txnt[i] == NULL)
				break;
		}
		if (i >= ctrl->last_tid) {
			if (ctrl->last_tid == 255) {
				mutex_unlock(&ctrl->m_ctrl);
				kfree(txn);
				return -ENOMEM;
			}
			ctrl->txnt = krealloc(ctrl->txnt,
					(i + 1) * sizeof(struct slim_msg_txn *),
					GFP_KERNEL);
			if (!ctrl->txnt) {
				mutex_unlock(&ctrl->m_ctrl);
				kfree(txn);
				return -ENOMEM;
			}
			ctrl->last_tid++;
		}
		ctrl->txnt[i] = txn;
		mutex_unlock(&ctrl->m_ctrl);
		txn->tid = i;
		*tid = i;
	}
	txn->mc = mc;
	txn->mt = mt;
	txn->dt = dt;
	txn->ec = ec;
	txn->la = la;
	txn->rbuf = rbuf;
	txn->wbuf = wbuf;
	txn->rl = mlen;
	txn->len = len;
	txn->comp = comp;

	ret = ctrl->xfer_msg(ctrl, txn);
	if (!tid)
		kfree(txn);
	return ret;
}

static int ctrl_getlogical_addr(struct slim_controller *ctrl, const u8 *eaddr,
				u8 e_len, u8 *entry)
{
	u8 i;
	for (i = 0; i < ctrl->num_dev; i++) {
		if (ctrl->addrt[i].valid &&
			memcmp(ctrl->addrt[i].eaddr, eaddr, e_len) == 0) {
			*entry = i;
			return 0;
		}
	}
	return -ENXIO;
}

int slim_assign_laddr(struct slim_controller *ctrl, const u8 *e_addr,
				u8 e_len, u8 *laddr, bool valid)
{
	int ret;
	u8 i = 0;
	bool exists = false;
	struct slim_device *sbdev;
	struct list_head *pos, *next;

	mutex_lock(&ctrl->m_ctrl);
	
	if (ctrl_getlogical_addr(ctrl, e_addr, e_len, &i) == 0) {
		*laddr = ctrl->addrt[i].laddr;
		exists = true;
	} else {
		if (ctrl->num_dev >= 254) {
			ret = -EXFULL;
			goto ret_assigned_laddr;
		}
		for (i = 0; i < ctrl->num_dev; i++) {
			if (ctrl->addrt[i].valid == false)
				break;
		}
		if (i == ctrl->num_dev) {
			ctrl->addrt = krealloc(ctrl->addrt,
					(ctrl->num_dev + 1) *
					sizeof(struct slim_addrt),
					GFP_KERNEL);
			if (!ctrl->addrt) {
				ret = -ENOMEM;
				goto ret_assigned_laddr;
			}
			ctrl->num_dev++;
		}
		memcpy(ctrl->addrt[i].eaddr, e_addr, e_len);
		ctrl->addrt[i].valid = true;
		
		if (!valid)
			*laddr = i;
	}

	ret = ctrl->set_laddr(ctrl, (const u8 *)&ctrl->addrt[i].eaddr, 6,
				*laddr);
	if (ret) {
		ctrl->addrt[i].valid = false;
		goto ret_assigned_laddr;
	}
	ctrl->addrt[i].laddr = *laddr;

	dev_dbg(&ctrl->dev, "setting slimbus l-addr:%x\n", *laddr);
ret_assigned_laddr:
	mutex_unlock(&ctrl->m_ctrl);
	if (exists || ret)
		return ret;

	pr_info("slimbus:%d laddr:0x%x, EAPC:0x%x:0x%x", ctrl->nr, *laddr,
				e_addr[1], e_addr[2]);
	mutex_lock(&ctrl->m_ctrl);
	list_for_each_safe(pos, next, &ctrl->devs) {
		sbdev = list_entry(pos, struct slim_device, dev_list);
		if (memcmp(sbdev->e_addr, e_addr, 6) == 0) {
			struct slim_driver *sbdrv;
			sbdev->laddr = *laddr;
			sbdev->reported = true;
			if (sbdev->dev.driver) {
				sbdrv = to_slim_driver(sbdev->dev.driver);
				if (sbdrv->device_up)
					queue_work(ctrl->wq, &sbdev->wd);
			}
			break;
		}
	}
	mutex_unlock(&ctrl->m_ctrl);
	return 0;
}
EXPORT_SYMBOL_GPL(slim_assign_laddr);

int slim_get_logical_addr(struct slim_device *sb, const u8 *e_addr,
				u8 e_len, u8 *laddr)
{
	int ret = 0;
	u8 entry;
	struct slim_controller *ctrl = sb->ctrl;
	if (!ctrl || !laddr || !e_addr || e_len != 6)
		return -EINVAL;
	mutex_lock(&ctrl->m_ctrl);
	ret = ctrl_getlogical_addr(ctrl, e_addr, e_len, &entry);
	if (!ret)
		*laddr = ctrl->addrt[entry].laddr;
	mutex_unlock(&ctrl->m_ctrl);
	if (ret == -ENXIO && ctrl->get_laddr) {
		ret = ctrl->get_laddr(ctrl, e_addr, e_len, laddr);
		if (!ret)
			ret = slim_assign_laddr(ctrl, e_addr, e_len, laddr,
						true);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(slim_get_logical_addr);

static int slim_ele_access_sanity(struct slim_ele_access *msg, int oper,
				u8 *rbuf, const u8 *wbuf, u8 len)
{
	if (!msg || msg->num_bytes > 16 || msg->start_offset + len > 0xC00)
		return -EINVAL;
	switch (oper) {
	case SLIM_MSG_MC_REQUEST_VALUE:
	case SLIM_MSG_MC_REQUEST_INFORMATION:
		if (rbuf == NULL)
			return -EINVAL;
		return 0;
	case SLIM_MSG_MC_CHANGE_VALUE:
	case SLIM_MSG_MC_CLEAR_INFORMATION:
		if (wbuf == NULL)
			return -EINVAL;
		return 0;
	case SLIM_MSG_MC_REQUEST_CHANGE_VALUE:
	case SLIM_MSG_MC_REQUEST_CLEAR_INFORMATION:
		if (rbuf == NULL || wbuf == NULL)
			return -EINVAL;
		return 0;
	default:
		return -EINVAL;
	}
}

static u16 slim_slicecodefromsize(u32 req)
{
	u8 codetosize[8] = {1, 2, 3, 4, 6, 8, 12, 16};
	if (req >= 8)
		return 0;
	else
		return codetosize[req];
}

static u16 slim_slicesize(u32 code)
{
	u8 sizetocode[16] = {0, 1, 2, 3, 3, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7};
	if (code == 0)
		code = 1;
	if (code > 16)
		code = 16;
	return sizetocode[code - 1];
}



/*
 * Message API access routines.
 * @sb: client handle requesting elemental message reads, writes.
 * @msg: Input structure for start-offset, number of bytes to read.
 * @rbuf: data buffer to be filled with values read.
 * @len: data buffer size
 * @wbuf: data buffer containing value/information to be written
 * context: can sleep
 * Returns:
 * -EINVAL: Invalid parameters
 * -ETIMEDOUT: If controller could not complete the request. This may happen if
 *  the bus lines are not clocked, controller is not powered-on, slave with
 *  given address is not enumerated/responding.
 */
int slim_request_val_element(struct slim_device *sb,
				struct slim_ele_access *msg, u8 *buf, u8 len)
{
	struct slim_controller *ctrl = sb->ctrl;
	if (!ctrl)
		return -EINVAL;
	return slim_xfer_msg(ctrl, sb, msg, SLIM_MSG_MC_REQUEST_VALUE, buf,
			NULL, len);
}
EXPORT_SYMBOL_GPL(slim_request_val_element);

int slim_request_inf_element(struct slim_device *sb,
				struct slim_ele_access *msg, u8 *buf, u8 len)
{
	struct slim_controller *ctrl = sb->ctrl;
	if (!ctrl)
		return -EINVAL;
	return slim_xfer_msg(ctrl, sb, msg, SLIM_MSG_MC_REQUEST_INFORMATION,
			buf, NULL, len);
}
EXPORT_SYMBOL_GPL(slim_request_inf_element);

int slim_change_val_element(struct slim_device *sb, struct slim_ele_access *msg,
				const u8 *buf, u8 len)
{
	struct slim_controller *ctrl = sb->ctrl;
	if (!ctrl)
		return -EINVAL;
	return slim_xfer_msg(ctrl, sb, msg, SLIM_MSG_MC_CHANGE_VALUE, NULL, buf,
					len);
}
EXPORT_SYMBOL_GPL(slim_change_val_element);

int slim_clear_inf_element(struct slim_device *sb, struct slim_ele_access *msg,
				u8 *buf, u8 len)
{
	struct slim_controller *ctrl = sb->ctrl;
	if (!ctrl)
		return -EINVAL;
	return slim_xfer_msg(ctrl, sb, msg, SLIM_MSG_MC_CLEAR_INFORMATION, NULL,
					buf, len);
}
EXPORT_SYMBOL_GPL(slim_clear_inf_element);

int slim_request_change_val_element(struct slim_device *sb,
					struct slim_ele_access *msg, u8 *rbuf,
					const u8 *wbuf, u8 len)
{
	struct slim_controller *ctrl = sb->ctrl;
	if (!ctrl)
		return -EINVAL;
	return slim_xfer_msg(ctrl, sb, msg, SLIM_MSG_MC_REQUEST_CHANGE_VALUE,
					rbuf, wbuf, len);
}
EXPORT_SYMBOL_GPL(slim_request_change_val_element);

int slim_request_clear_inf_element(struct slim_device *sb,
					struct slim_ele_access *msg, u8 *rbuf,
					const u8 *wbuf, u8 len)
{
	struct slim_controller *ctrl = sb->ctrl;
	if (!ctrl)
		return -EINVAL;
	return slim_xfer_msg(ctrl, sb, msg,
					SLIM_MSG_MC_REQUEST_CLEAR_INFORMATION,
					rbuf, wbuf, len);
}
EXPORT_SYMBOL_GPL(slim_request_clear_inf_element);

int slim_xfer_msg(struct slim_controller *ctrl, struct slim_device *sbdev,
			struct slim_ele_access *msg, u16 mc, u8 *rbuf,
			const u8 *wbuf, u8 len)
{
	DECLARE_COMPLETION_ONSTACK(complete);
	int ret;
	u16 sl, cur;
	u16 ec;
	u8 tid, mlen = 6;

	ret = slim_ele_access_sanity(msg, mc, rbuf, wbuf, len);
	if (ret)
		goto xfer_err;

	sl = slim_slicesize(len);
	dev_dbg(&ctrl->dev, "SB xfer msg:os:%x, len:%d, MC:%x, sl:%x\n",
				msg->start_offset, len, mc, sl);

	cur = slim_slicecodefromsize(sl);
	ec = ((sl | (1 << 3)) | ((msg->start_offset & 0xFFF) << 4));

	if (wbuf)
		mlen += len;
	if (rbuf) {
		mlen++;
		if (!msg->comp)
			ret = slim_processtxn(ctrl, SLIM_MSG_DEST_LOGICALADDR,
				mc, ec, SLIM_MSG_MT_CORE, rbuf, wbuf, len, mlen,
				&complete, sbdev->laddr, &tid);
		else
			ret = slim_processtxn(ctrl, SLIM_MSG_DEST_LOGICALADDR,
				mc, ec, SLIM_MSG_MT_CORE, rbuf, wbuf, len, mlen,
				msg->comp, sbdev->laddr, &tid);
		
		if (!ret && !msg->comp) {
			ret = wait_for_completion_timeout(&complete, HZ);
			if (!ret) {
				struct slim_msg_txn *txn;
				dev_err(&ctrl->dev, "slimbus Read timed out");
				mutex_lock(&ctrl->m_ctrl);
				txn = ctrl->txnt[tid];
				
				ctrl->txnt[tid] = NULL;
				mutex_unlock(&ctrl->m_ctrl);
				kfree(txn);
				ret = -ETIMEDOUT;
			} else
				ret = 0;
		} else if (ret < 0 && !msg->comp) {
			struct slim_msg_txn *txn;
			dev_err(&ctrl->dev, "slimbus Read error");
			mutex_lock(&ctrl->m_ctrl);
			txn = ctrl->txnt[tid];
			
			ctrl->txnt[tid] = NULL;
			mutex_unlock(&ctrl->m_ctrl);
			kfree(txn);
		}

	} else
		ret = slim_processtxn(ctrl, SLIM_MSG_DEST_LOGICALADDR, mc, ec,
				SLIM_MSG_MT_CORE, rbuf, wbuf, len, mlen,
				NULL, sbdev->laddr, NULL);
xfer_err:
	return ret;
}
EXPORT_SYMBOL_GPL(slim_xfer_msg);

int slim_user_msg(struct slim_device *sb, u8 la, u8 mt, u8 mc,
				struct slim_ele_access *msg, u8 *buf, u8 len)
{
	if (!sb || !sb->ctrl || !msg || mt == SLIM_MSG_MT_CORE)
		return -EINVAL;
	if (!sb->ctrl->xfer_user_msg)
		return -EPROTONOSUPPORT;
	return sb->ctrl->xfer_user_msg(sb->ctrl, la, mt, mc, msg, buf, len);
}
EXPORT_SYMBOL(slim_user_msg);

int slim_alloc_mgrports(struct slim_device *sb, enum slim_port_req req,
				int nports, u32 *rh, int hsz)
{
	int i, j;
	int ret = -EINVAL;
	int nphysp = nports;
	struct slim_controller *ctrl = sb->ctrl;

	if (!rh || !ctrl)
		return -EINVAL;
	if (req == SLIM_REQ_HALF_DUP)
		nphysp *= 2;
	if (hsz/sizeof(u32) < nphysp)
		return -EINVAL;
	mutex_lock(&ctrl->m_ctrl);

	for (i = 0; i < ctrl->nports; i++) {
		bool multiok = true;
		if (ctrl->ports[i].state != SLIM_P_FREE)
			continue;
		
		if (req == SLIM_REQ_HALF_DUP && (i % 2))
			continue;
		
		if (ctrl->nports < (i + nphysp)) {
			i = ctrl->nports;
			break;
		}
		if (req == SLIM_REQ_MULTI_CH) {
			multiok = true;
			for (j = i; j < i + nphysp; j++) {
				if (ctrl->ports[j].state != SLIM_P_FREE) {
					multiok = false;
					break;
				}
			}
			if (!multiok)
				continue;
		}
		break;
	}
	if (i >= ctrl->nports) {
		ret = -EDQUOT;
		goto alloc_err;
	}
	ret = 0;
	for (j = i; j < i + nphysp; j++) {
		ctrl->ports[j].state = SLIM_P_UNCFG;
		ctrl->ports[j].req = req;
		if (req == SLIM_REQ_HALF_DUP && (j % 2))
			ctrl->ports[j].flow = SLIM_SINK;
		else
			ctrl->ports[j].flow = SLIM_SRC;
		if (ctrl->alloc_port)
			ret = ctrl->alloc_port(ctrl, j);
		if (ret) {
			for (; j >= i; j--)
				ctrl->ports[j].state = SLIM_P_FREE;
			goto alloc_err;
		}
		*rh++ = SLIM_PORT_HDL(SLIM_LA_MANAGER, 0, j);
	}
alloc_err:
	mutex_unlock(&ctrl->m_ctrl);
	return ret;
}
EXPORT_SYMBOL_GPL(slim_alloc_mgrports);

int slim_dealloc_mgrports(struct slim_device *sb, u32 *hdl, int nports)
{
	int i;
	struct slim_controller *ctrl = sb->ctrl;

	if (!ctrl || !hdl)
		return -EINVAL;

	mutex_lock(&ctrl->m_ctrl);

	for (i = 0; i < nports; i++) {
		u8 pn;
		pn = SLIM_HDL_TO_PORT(hdl[i]);

		if (pn >= ctrl->nports || ctrl->ports[pn].state == SLIM_P_CFG) {
			int j, ret;
			if (pn >= ctrl->nports) {
				dev_err(&ctrl->dev, "invalid port number");
				ret = -EINVAL;
			} else {
				dev_err(&ctrl->dev,
					"Can't dealloc connected port:%d", i);
				ret = -EISCONN;
			}
			for (j = i - 1; j >= 0; j--) {
				pn = SLIM_HDL_TO_PORT(hdl[j]);
				ctrl->ports[pn].state = SLIM_P_UNCFG;
			}
			mutex_unlock(&ctrl->m_ctrl);
			return ret;
		}
		if (ctrl->dealloc_port)
			ctrl->dealloc_port(ctrl, pn);
		ctrl->ports[pn].state = SLIM_P_FREE;
	}
	mutex_unlock(&ctrl->m_ctrl);
	return 0;
}
EXPORT_SYMBOL_GPL(slim_dealloc_mgrports);

int slim_get_slaveport(u8 la, int idx, u32 *rh, enum slim_port_flow flw)
{
	if (rh == NULL)
		return -EINVAL;
	*rh = SLIM_PORT_HDL(la, flw, idx);
	return 0;
}
EXPORT_SYMBOL_GPL(slim_get_slaveport);

static int connect_port_ch(struct slim_controller *ctrl, u8 ch, u32 ph,
				enum slim_port_flow flow)
{
	int ret;
	u16 mc;
	u8 buf[2];
	u32 la = SLIM_HDL_TO_LA(ph);
	u8 pn = (u8)SLIM_HDL_TO_PORT(ph);

	if (flow == SLIM_SRC)
		mc = SLIM_MSG_MC_CONNECT_SOURCE;
	else
		mc = SLIM_MSG_MC_CONNECT_SINK;
	buf[0] = pn;
	buf[1] = ctrl->chans[ch].chan;
	if (la == SLIM_LA_MANAGER)
		ctrl->ports[pn].flow = flow;
	ret = slim_processtxn(ctrl, SLIM_MSG_DEST_LOGICALADDR, mc, 0,
				SLIM_MSG_MT_CORE, NULL, buf, 2, 6, NULL, la,
				NULL);
	if (!ret && la == SLIM_LA_MANAGER)
		ctrl->ports[pn].state = SLIM_P_CFG;
	return ret;
}

static int disconnect_port_ch(struct slim_controller *ctrl, u32 ph)
{
	int ret;
	u16 mc;
	u32 la = SLIM_HDL_TO_LA(ph);
	u8 pn = (u8)SLIM_HDL_TO_PORT(ph);

	mc = SLIM_MSG_MC_DISCONNECT_PORT;
	ret = slim_processtxn(ctrl, SLIM_MSG_DEST_LOGICALADDR, mc, 0,
				SLIM_MSG_MT_CORE, NULL, &pn, 1, 5,
				NULL, la, NULL);
	if (ret)
		return ret;
	if (la == SLIM_LA_MANAGER)
		ctrl->ports[pn].state = SLIM_P_UNCFG;
	return 0;
}

int slim_connect_src(struct slim_device *sb, u32 srch, u16 chanh)
{
	struct slim_controller *ctrl = sb->ctrl;
	int ret;
	u8 chan = SLIM_HDL_TO_CHIDX(chanh);
	struct slim_ich *slc = &ctrl->chans[chan];
	enum slim_port_flow flow = SLIM_HDL_TO_FLOW(srch);
	u8 la = SLIM_HDL_TO_LA(srch);

	
	if (la != SLIM_LA_MANAGER && flow != SLIM_SRC)
		return -EINVAL;

	mutex_lock(&ctrl->sched.m_reconf);

	if (la == SLIM_LA_MANAGER) {
		u8 pn = SLIM_HDL_TO_PORT(srch);
		if (pn >= ctrl->nports ||
			ctrl->ports[pn].state != SLIM_P_UNCFG) {
			ret = -EINVAL;
			goto connect_src_err;
		}
	}

	if (slc->state == SLIM_CH_FREE) {
		ret = -ENOTCONN;
		goto connect_src_err;
	}
	if (slc->srch) {
		ret = -EALREADY;
		goto connect_src_err;
	}

	ret = connect_port_ch(ctrl, chan, srch, SLIM_SRC);

	if (!ret)
		slc->srch = srch;

connect_src_err:
	mutex_unlock(&ctrl->sched.m_reconf);
	return ret;
}
EXPORT_SYMBOL_GPL(slim_connect_src);

int slim_connect_sink(struct slim_device *sb, u32 *sinkh, int nsink, u16 chanh)
{
	struct slim_controller *ctrl = sb->ctrl;
	int j;
	int ret = 0;
	u8 chan = SLIM_HDL_TO_CHIDX(chanh);
	struct slim_ich *slc = &ctrl->chans[chan];

	if (!sinkh || !nsink)
		return -EINVAL;

	mutex_lock(&ctrl->sched.m_reconf);

	if (slc->state == SLIM_CH_FREE) {
		ret = -ENOTCONN;
		goto connect_sink_err;
	}

	for (j = 0; j < nsink; j++) {
		enum slim_port_flow flow = SLIM_HDL_TO_FLOW(sinkh[j]);
		u8 la = SLIM_HDL_TO_LA(sinkh[j]);
		u8 pn = SLIM_HDL_TO_PORT(sinkh[j]);
		if (la != SLIM_LA_MANAGER && flow != SLIM_SINK)
			ret = -EINVAL;
		else if (la == SLIM_LA_MANAGER &&
				(pn >= ctrl->nports ||
				ctrl->ports[pn].state != SLIM_P_UNCFG))
				ret = -EINVAL;
		else
			ret = connect_port_ch(ctrl, chan, sinkh[j], SLIM_SINK);
		if (ret) {
			for (j = j - 1; j >= 0; j--)
				disconnect_port_ch(ctrl, sinkh[j]);
			goto connect_sink_err;
		}
	}

	slc->sinkh = krealloc(slc->sinkh, (sizeof(u32) * (slc->nsink + nsink)),
				GFP_KERNEL);
	if (!slc->sinkh) {
		ret = -ENOMEM;
		for (j = 0; j < nsink; j++)
			disconnect_port_ch(ctrl, sinkh[j]);
		goto connect_sink_err;
	}

	memcpy(slc->sinkh + slc->nsink, sinkh, (sizeof(u32) * nsink));
	slc->nsink += nsink;

connect_sink_err:
	mutex_unlock(&ctrl->sched.m_reconf);
	return ret;
}
EXPORT_SYMBOL_GPL(slim_connect_sink);

int slim_disconnect_ports(struct slim_device *sb, u32 *ph, int nph)
{
	struct slim_controller *ctrl = sb->ctrl;
	int i;

	mutex_lock(&ctrl->sched.m_reconf);

	for (i = 0; i < nph; i++)
		disconnect_port_ch(ctrl, ph[i]);
	mutex_unlock(&ctrl->sched.m_reconf);
	return 0;
}
EXPORT_SYMBOL_GPL(slim_disconnect_ports);

int slim_port_xfer(struct slim_device *sb, u32 ph, phys_addr_t iobuf, u32 len,
				struct completion *comp)
{
	struct slim_controller *ctrl = sb->ctrl;
	u8 pn = SLIM_HDL_TO_PORT(ph);
	dev_dbg(&ctrl->dev, "port xfer: num:%d", pn);
	return ctrl->port_xfer(ctrl, pn, iobuf, len, comp);
}
EXPORT_SYMBOL_GPL(slim_port_xfer);

enum slim_port_err slim_port_get_xfer_status(struct slim_device *sb, u32 ph,
			phys_addr_t *done_buf, u32 *done_len)
{
	struct slim_controller *ctrl = sb->ctrl;
	u8 pn = SLIM_HDL_TO_PORT(ph);
	u32 la = SLIM_HDL_TO_LA(ph);
	enum slim_port_err err;
	dev_dbg(&ctrl->dev, "get status port num:%d", pn);
	if (la != SLIM_LA_MANAGER) {
		if (done_buf)
			*done_buf = 0;
		if (done_len)
			*done_len = 0;
		return SLIM_P_NOT_OWNED;
	}
	err = ctrl->port_xfer_status(ctrl, pn, done_buf, done_len);
	if (err == SLIM_P_INPROGRESS)
		err = ctrl->ports[pn].err;
	return err;
}
EXPORT_SYMBOL_GPL(slim_port_get_xfer_status);

static void slim_add_ch(struct slim_controller *ctrl, struct slim_ich *slc)
{
	struct slim_ich **arr;
	int i, j;
	int *len;
	int sl = slc->seglen << slc->rootexp;
	
	if (slc->state >= SLIM_CH_ACTIVE)
		return;
	if (slc->coeff == SLIM_COEFF_1) {
		arr = ctrl->sched.chc1;
		len = &ctrl->sched.num_cc1;
	} else {
		arr = ctrl->sched.chc3;
		len = &ctrl->sched.num_cc3;
		sl *= 3;
	}

	*len += 1;

	
	for (i = 0; i < *len - 1; i++) {
		if ((slc->rootexp > arr[i]->rootexp) ||
			((slc->rootexp == arr[i]->rootexp) &&
			(slc->seglen < arr[i]->seglen)))
			continue;
		else
			break;
	}
	for (j = *len - 1; j > i; j--)
		arr[j] = arr[j - 1];
	arr[i] = slc;
	if (!ctrl->allocbw)
		ctrl->sched.usedslots += sl;

	return;
}

static int slim_remove_ch(struct slim_controller *ctrl, struct slim_ich *slc)
{
	struct slim_ich **arr;
	int i;
	u32 la, ph;
	int *len;
	if (slc->coeff == SLIM_COEFF_1) {
		arr = ctrl->sched.chc1;
		len = &ctrl->sched.num_cc1;
	} else {
		arr = ctrl->sched.chc3;
		len = &ctrl->sched.num_cc3;
	}

	for (i = 0; i < *len; i++) {
		if (arr[i] == slc)
			break;
	}
	if (i >= *len)
		return -EXFULL;
	for (; i < *len - 1; i++)
		arr[i] = arr[i + 1];
	*len -= 1;
	arr[*len] = NULL;

	slc->state = SLIM_CH_ALLOCATED;
	slc->newintr = 0;
	slc->newoff = 0;
	for (i = 0; i < slc->nsink; i++) {
		ph = slc->sinkh[i];
		la = SLIM_HDL_TO_LA(ph);
		if (la == SLIM_LA_MANAGER)
			ctrl->ports[SLIM_HDL_TO_PORT(ph)].state = SLIM_P_UNCFG;
	}

	ph = slc->srch;
	la = SLIM_HDL_TO_LA(ph);
	if (la == SLIM_LA_MANAGER)
		ctrl->ports[SLIM_HDL_TO_PORT(ph)].state = SLIM_P_UNCFG;

	kfree(slc->sinkh);
	slc->sinkh = NULL;
	slc->srch = 0;
	slc->nsink = 0;
	return 0;
}

static u32 slim_calc_prrate(struct slim_controller *ctrl, struct slim_ch *prop)
{
	u32 rate = 0, rate4k = 0, rate11k = 0;
	u32 exp = 0;
	u32 pr = 0;
	bool exact = true;
	bool done = false;
	enum slim_ch_rate ratefam;

	if (prop->prot >= SLIM_PUSH)
		return 0;
	if (prop->baser == SLIM_RATE_1HZ) {
		rate = prop->ratem / 4000;
		rate4k = rate;
		if (rate * 4000 == prop->ratem)
			ratefam = SLIM_RATE_4000HZ;
		else {
			rate = prop->ratem / 11025;
			rate11k = rate;
			if (rate * 11025 == prop->ratem)
				ratefam = SLIM_RATE_11025HZ;
			else
				ratefam = SLIM_RATE_1HZ;
		}
	} else {
		ratefam = prop->baser;
		rate = prop->ratem;
	}
	if (ratefam == SLIM_RATE_1HZ) {
		exact = false;
		if ((rate4k + 1) * 4000 < (rate11k + 1) * 11025) {
			rate = rate4k + 1;
			ratefam = SLIM_RATE_4000HZ;
		} else {
			rate = rate11k + 1;
			ratefam = SLIM_RATE_11025HZ;
		}
	}
	
	while (!done) {
		while ((rate & 0x1) != 0x1) {
			rate >>= 1;
			exp++;
		}
		if (rate > 3) {
			
			rate++;
			exact = false;
		} else
			done = true;
	}
	if (ratefam == SLIM_RATE_4000HZ) {
		if (rate == 1)
			pr = 0x10;
		else {
			pr = 0;
			exp++;
		}
	} else {
		pr = 8;
		exp++;
	}
	if (exp <= 7) {
		pr |= exp;
		if (exact)
			pr |= 0x80;
	} else
		pr = 0;
	return pr;
}

static int slim_nextdefine_ch(struct slim_device *sb, u8 chan)
{
	struct slim_controller *ctrl = sb->ctrl;
	u32 chrate = 0;
	u32 exp = 0;
	u32 coeff = 0;
	bool exact = true;
	bool done = false;
	int ret = 0;
	struct slim_ich *slc = &ctrl->chans[chan];
	struct slim_ch *prop = &slc->prop;

	slc->prrate = slim_calc_prrate(ctrl, prop);
	dev_dbg(&ctrl->dev, "ch:%d, chan PR rate:%x\n", chan, slc->prrate);
	if (prop->baser == SLIM_RATE_4000HZ)
		chrate = 4000 * prop->ratem;
	else if (prop->baser == SLIM_RATE_11025HZ)
		chrate = 11025 * prop->ratem;
	else
		chrate = prop->ratem;
	
	if (chrate > 3600000)
		return -EDQUOT;
	if (prop->baser == SLIM_RATE_4000HZ &&
			ctrl->a_framer->superfreq == 4000)
		coeff = prop->ratem;
	else if (prop->baser == SLIM_RATE_11025HZ &&
			ctrl->a_framer->superfreq == 3675)
		coeff = 3 * prop->ratem;
	else {
		u32 tempr = 0;
		tempr = chrate * SLIM_CL_PER_SUPERFRAME_DIV8;
		coeff = tempr / ctrl->a_framer->rootfreq;
		if (coeff * ctrl->a_framer->rootfreq != tempr) {
			coeff++;
			exact = false;
		}
	}

	
	exp = 0;
	while (!done) {
		while ((coeff & 0x1) != 0x1) {
			coeff >>= 1;
			exp++;
		}
		if (coeff > 3) {
			coeff++;
			exact = false;
		} else
			done = true;
	}
	if (prop->prot == SLIM_HARD_ISO && !exact)
		return -EPROTONOSUPPORT;
	else if (prop->prot == SLIM_AUTO_ISO) {
		if (exact)
			prop->prot = SLIM_HARD_ISO;
		else {
			
			return -EPROTONOSUPPORT;
		}
	}
	slc->rootexp = exp;
	slc->seglen = prop->sampleszbits/SLIM_CL_PER_SL;
	if (prop->prot != SLIM_HARD_ISO)
		slc->seglen++;
	if (prop->prot >= SLIM_EXT_SMPLX)
		slc->seglen++;
	
	if (coeff == 1) {
		if (exp > 9)
			ret = -EIO;
		coeff = SLIM_COEFF_1;
	} else {
		if (exp > 8)
			ret = -EIO;
		coeff = SLIM_COEFF_3;
	}
	slc->coeff = coeff;

	return ret;
}

int slim_alloc_ch(struct slim_device *sb, u16 *chanh)
{
	struct slim_controller *ctrl = sb->ctrl;
	u16 i;

	if (!ctrl)
		return -EINVAL;
	mutex_lock(&ctrl->sched.m_reconf);
	for (i = 0; i < ctrl->nchans; i++) {
		if (ctrl->chans[i].state == SLIM_CH_FREE)
			break;
	}
	if (i >= ctrl->nchans) {
		mutex_unlock(&ctrl->sched.m_reconf);
		return -EXFULL;
	}
	*chanh = i;
	ctrl->chans[i].nextgrp = 0;
	ctrl->chans[i].state = SLIM_CH_ALLOCATED;
	ctrl->chans[i].chan = (u8)(ctrl->reserved + i);

	mutex_unlock(&ctrl->sched.m_reconf);
	return 0;
}
EXPORT_SYMBOL_GPL(slim_alloc_ch);

int slim_query_ch(struct slim_device *sb, u8 ch, u16 *chanh)
{
	struct slim_controller *ctrl = sb->ctrl;
	u16 i, j;
	int ret = 0;
	if (!ctrl || !chanh)
		return -EINVAL;
	mutex_lock(&ctrl->sched.m_reconf);
	
	i = ch % ctrl->nchans;

	for (j = 0; j < ctrl->nchans; j++) {
		if (ctrl->chans[i].chan == ch) {
			*chanh = i;
			ctrl->chans[i].ref++;
			if (ctrl->chans[i].state == SLIM_CH_FREE)
				ctrl->chans[i].state = SLIM_CH_ALLOCATED;
			goto query_out;
		}
		i = (i + 1) % ctrl->nchans;
	}

	
	ret = -EXFULL;
	for (j = 0; j < ctrl->nchans; j++) {
		if (ctrl->chans[i].state == SLIM_CH_FREE) {
			ctrl->chans[i].state =
				SLIM_CH_ALLOCATED;
			*chanh = i;
			ctrl->chans[i].ref++;
			ctrl->chans[i].chan = ch;
			ctrl->chans[i].nextgrp = 0;
			ret = 0;
			break;
		}
		i = (i + 1) % ctrl->nchans;
	}
query_out:
	mutex_unlock(&ctrl->sched.m_reconf);
	dev_dbg(&ctrl->dev, "query ch:%d,hdl:%d,ref:%d,ret:%d",
				ch, i, ctrl->chans[i].ref, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(slim_query_ch);

int slim_dealloc_ch(struct slim_device *sb, u16 chanh)
{
	struct slim_controller *ctrl = sb->ctrl;
	u8 chan = SLIM_HDL_TO_CHIDX(chanh);
	struct slim_ich *slc = &ctrl->chans[chan];
	if (!ctrl)
		return -EINVAL;

	mutex_lock(&ctrl->sched.m_reconf);
	if (slc->state == SLIM_CH_FREE) {
		mutex_unlock(&ctrl->sched.m_reconf);
		return -ENOTCONN;
	}
	if (slc->ref > 1) {
		slc->ref--;
		mutex_unlock(&ctrl->sched.m_reconf);
		dev_dbg(&ctrl->dev, "remove chan:%d,hdl:%d,ref:%d",
					slc->chan, chanh, slc->ref);
		return 0;
	}
	if (slc->state >= SLIM_CH_PENDING_ACTIVE) {
		dev_err(&ctrl->dev, "Channel:%d should be removed first", chan);
		mutex_unlock(&ctrl->sched.m_reconf);
		return -EISCONN;
	}
	slc->ref--;
	slc->state = SLIM_CH_FREE;
	mutex_unlock(&ctrl->sched.m_reconf);
	dev_dbg(&ctrl->dev, "remove chan:%d,hdl:%d,ref:%d",
				slc->chan, chanh, slc->ref);
	return 0;
}
EXPORT_SYMBOL_GPL(slim_dealloc_ch);

enum slim_ch_state slim_get_ch_state(struct slim_device *sb, u16 chanh)
{
	u8 chan = SLIM_HDL_TO_CHIDX(chanh);
	struct slim_ich *slc = &sb->ctrl->chans[chan];
	return slc->state;
}
EXPORT_SYMBOL_GPL(slim_get_ch_state);

int slim_define_ch(struct slim_device *sb, struct slim_ch *prop, u16 *chanh,
			u8 nchan, bool grp, u16 *grph)
{
	struct slim_controller *ctrl = sb->ctrl;
	int i, ret = 0;

	if (!ctrl || !chanh || !prop || !nchan)
		return -EINVAL;
	mutex_lock(&ctrl->sched.m_reconf);
	for (i = 0; i < nchan; i++) {
		u8 chan = SLIM_HDL_TO_CHIDX(chanh[i]);
		struct slim_ich *slc = &ctrl->chans[chan];
		dev_dbg(&ctrl->dev, "define_ch: ch:%d, state:%d", chan,
				(int)ctrl->chans[chan].state);
		if (slc->state < SLIM_CH_ALLOCATED) {
			ret = -ENXIO;
			goto err_define_ch;
		}
		if (slc->state >= SLIM_CH_DEFINED && slc->ref >= 2) {
			if (prop->ratem != slc->prop.ratem ||
			prop->sampleszbits != slc->prop.sampleszbits ||
			prop->baser != slc->prop.baser) {
				ret = -EISCONN;
				goto err_define_ch;
			}
		} else if (slc->state > SLIM_CH_DEFINED) {
			ret = -EISCONN;
			goto err_define_ch;
		} else {
			ctrl->chans[chan].prop = *prop;
			ret = slim_nextdefine_ch(sb, chan);
			if (ret)
				goto err_define_ch;
		}
		if (i < (nchan - 1))
			ctrl->chans[chan].nextgrp = chanh[i + 1];
		if (i == 0)
			ctrl->chans[chan].nextgrp |= SLIM_START_GRP;
		if (i == (nchan - 1))
			ctrl->chans[chan].nextgrp |= SLIM_END_GRP;
	}

	if (grp)
		*grph = ((nchan << 8) | SLIM_HDL_TO_CHIDX(chanh[0]));
	for (i = 0; i < nchan; i++) {
		u8 chan = SLIM_HDL_TO_CHIDX(chanh[i]);
		struct slim_ich *slc = &ctrl->chans[chan];
		if (slc->state == SLIM_CH_ALLOCATED)
			slc->state = SLIM_CH_DEFINED;
	}
err_define_ch:
	dev_dbg(&ctrl->dev, "define_ch: ch:%d, ret:%d", *chanh, ret);
	mutex_unlock(&ctrl->sched.m_reconf);
	return ret;
}
EXPORT_SYMBOL_GPL(slim_define_ch);

static u32 getsubfrmcoding(u32 *ctrlw, u32 *subfrml, u32 *msgsl)
{
	u32 code = 0;
	if (*ctrlw == *subfrml) {
		*ctrlw = 8;
		*subfrml = 8;
		*msgsl = SLIM_SL_PER_SUPERFRAME - SLIM_FRM_SLOTS_PER_SUPERFRAME
				- SLIM_GDE_SLOTS_PER_SUPERFRAME;
		return 0;
	}
	if (*subfrml == 6) {
		code = 0;
		*msgsl = 256;
	} else if (*subfrml == 8) {
		code = 1;
		*msgsl = 192;
	} else if (*subfrml == 24) {
		code = 2;
		*msgsl = 64;
	} else { 
		code = 3;
		*msgsl = 48;
	}

	if (*ctrlw < 8) {
		if (*ctrlw >= 6) {
			*ctrlw = 6;
			code |= 0x14;
		} else {
			if (*ctrlw == 5)
				*ctrlw = 4;
			code |= (*ctrlw << 2);
		}
	} else {
		code -= 2;
		if (*ctrlw >= 24) {
			*ctrlw = 24;
			code |= 0x1e;
		} else if (*ctrlw >= 16) {
			*ctrlw = 16;
			code |= 0x1c;
		} else if (*ctrlw >= 12) {
			*ctrlw = 12;
			code |= 0x1a;
		} else {
			*ctrlw = 8;
			code |= 0x18;
		}
	}

	*msgsl = (*msgsl * *ctrlw) - SLIM_FRM_SLOTS_PER_SUPERFRAME -
				SLIM_GDE_SLOTS_PER_SUPERFRAME;
	return code;
}

static void shiftsegoffsets(struct slim_controller *ctrl, struct slim_ich **ach,
				int sz, u32 shft)
{
	int i;
	u32 oldoff;
	for (i = 0; i < sz; i++) {
		struct slim_ich *slc;
		if (ach[i] == NULL)
			continue;
		slc = ach[i];
		if (slc->state == SLIM_CH_PENDING_REMOVAL)
			continue;
		oldoff = slc->newoff;
		slc->newoff += shft;
		
		if (slc->newoff >= slc->newintr)
			slc->newoff -= slc->newintr;
	}
}

static int slim_sched_chans(struct slim_device *sb, u32 clkgear,
			u32 *ctrlw, u32 *subfrml)
{
	int coeff1, coeff3;
	enum slim_ch_coeff bias;
	struct slim_controller *ctrl = sb->ctrl;
	int last1 = ctrl->sched.num_cc1 - 1;
	int last3 = ctrl->sched.num_cc3 - 1;

	for (coeff3 = 0; coeff3 < ctrl->sched.num_cc3; coeff3++) {
		struct slim_ich *slc = ctrl->sched.chc3[coeff3];
		if (slc->state == SLIM_CH_PENDING_REMOVAL)
			continue;
		else
			break;
	}
	for (coeff1 = 0; coeff1 < ctrl->sched.num_cc1; coeff1++) {
		struct slim_ich *slc = ctrl->sched.chc1[coeff1];
		if (slc->state == SLIM_CH_PENDING_REMOVAL)
			continue;
		else
			break;
	}
	if (coeff3 == ctrl->sched.num_cc3 && coeff1 == ctrl->sched.num_cc1) {
		*ctrlw = 8;
		*subfrml = 8;
		return 0;
	} else if (coeff3 == ctrl->sched.num_cc3)
		bias = SLIM_COEFF_1;
	else
		bias = SLIM_COEFF_3;

	while (last1 >= 0) {
		if (ctrl->sched.chc1[last1] != NULL &&
			(ctrl->sched.chc1[last1])->state !=
			SLIM_CH_PENDING_REMOVAL)
			break;
		last1--;
	}
	while (last3 >= 0) {
		if (ctrl->sched.chc3[last3] != NULL &&
			(ctrl->sched.chc3[last3])->state !=
			SLIM_CH_PENDING_REMOVAL)
			break;
		last3--;
	}

	if (bias == SLIM_COEFF_1) {
		struct slim_ich *slc1 = ctrl->sched.chc1[coeff1];
		u32 expshft = SLIM_MAX_CLK_GEAR - clkgear;
		int curexp, finalexp;
		u32 curintr, curmaxsl;
		int opensl1[2];
		int maxctrlw1;

		finalexp = (ctrl->sched.chc1[last1])->rootexp;
		curexp = (int)expshft - 1;

		curintr = (SLIM_MAX_INTR_COEFF_1 * 2) >> (curexp + 1);
		curmaxsl = curintr >> 1;
		opensl1[0] = opensl1[1] = curmaxsl;

		while ((coeff1 < ctrl->sched.num_cc1) || (curintr > 24)) {
			curintr >>= 1;
			curmaxsl >>= 1;

			
			if (opensl1[1] < opensl1[0])
				opensl1[1] -= curmaxsl;
			else
				opensl1[1] = opensl1[0] - curmaxsl;
			opensl1[0] = curmaxsl;
			if (opensl1[1] < 0) {
				opensl1[0] += opensl1[1];
				opensl1[1] = 0;
			}
			if (opensl1[0] <= 0) {
				dev_dbg(&ctrl->dev, "reconfig failed:%d\n",
						__LINE__);
				return -EXFULL;
			}
			curexp++;
			

			while ((coeff1 < ctrl->sched.num_cc1) && (curexp ==
					(int)(slc1->rootexp + expshft))) {
				if (slc1->state == SLIM_CH_PENDING_REMOVAL) {
					coeff1++;
					slc1 = ctrl->sched.chc1[coeff1];
					continue;
				}
				if (opensl1[1] >= opensl1[0] ||
					(finalexp == (int)slc1->rootexp &&
					 curintr <= 24 &&
					 opensl1[0] == curmaxsl)) {
					opensl1[1] -= slc1->seglen;
					slc1->newoff = curmaxsl + opensl1[1];
					if (opensl1[1] < 0 &&
						opensl1[0] == curmaxsl) {
						opensl1[0] += opensl1[1];
						opensl1[1] = 0;
						if (opensl1[0] < 0) {
							dev_dbg(&ctrl->dev,
							"reconfig failed:%d\n",
							__LINE__);
							return -EXFULL;
						}
					}
				} else {
					if (slc1->seglen > opensl1[0]) {
						dev_dbg(&ctrl->dev,
						"reconfig failed:%d\n",
						__LINE__);
						return -EXFULL;
					}
					slc1->newoff = opensl1[0] -
							slc1->seglen;
					opensl1[0] = slc1->newoff;
				}
				slc1->newintr = curintr;
				coeff1++;
				slc1 = ctrl->sched.chc1[coeff1];
			}
		}
		
		if (opensl1[1] <= 0 && opensl1[0] <= 0)
			return -EXFULL;
		if (opensl1[1] > opensl1[0]) {
			int temp = opensl1[0];
			opensl1[0] = opensl1[1];
			opensl1[1] = temp;
			shiftsegoffsets(ctrl, ctrl->sched.chc1,
					ctrl->sched.num_cc1, curmaxsl);
		}
		
		maxctrlw1 = opensl1[0];
		if (opensl1[0] == curmaxsl)
			maxctrlw1 += opensl1[1];
		if (curintr >= 24) {
			*subfrml = 24;
			*ctrlw = maxctrlw1;
		} else if (curintr == 12) {
			if (maxctrlw1 > opensl1[1] * 4) {
				*subfrml = 24;
				*ctrlw = maxctrlw1;
			} else {
				*subfrml = 6;
				*ctrlw = opensl1[1];
			}
		} else {
			*subfrml = 6;
			*ctrlw = maxctrlw1;
		}
	} else {
		struct slim_ich *slc1 = NULL;
		struct slim_ich *slc3 = ctrl->sched.chc3[coeff3];
		u32 expshft = SLIM_MAX_CLK_GEAR - clkgear;
		int curexp, finalexp, exp1;
		u32 curintr, curmaxsl;
		int opensl3[2];
		int opensl1[6];
		bool opensl1valid = false;
		int maxctrlw1, maxctrlw3, i;
		finalexp = (ctrl->sched.chc3[last3])->rootexp;
		if (last1 >= 0) {
			slc1 = ctrl->sched.chc1[coeff1];
			exp1 = (ctrl->sched.chc1[last1])->rootexp;
			if (exp1 > finalexp)
				finalexp = exp1;
		}
		curexp = (int)expshft - 1;

		curintr = (SLIM_MAX_INTR_COEFF_3 * 2) >> (curexp + 1);
		curmaxsl = curintr >> 1;
		opensl3[0] = opensl3[1] = curmaxsl;

		while (coeff1 < ctrl->sched.num_cc1 ||
			coeff3 < ctrl->sched.num_cc3 ||
			curintr > 32) {
			curintr >>= 1;
			curmaxsl >>= 1;

			
			if (opensl3[1] < opensl3[0])
				opensl3[1] -= curmaxsl;
			else
				opensl3[1] = opensl3[0] - curmaxsl;
			opensl3[0] = curmaxsl;
			if (opensl3[1] < 0) {
				opensl3[0] += opensl3[1];
				opensl3[1] = 0;
			}
			if (opensl3[0] <= 0) {
				dev_dbg(&ctrl->dev, "reconfig failed:%d\n",
						__LINE__);
				return -EXFULL;
			}
			curexp++;

			
			while (coeff3 < ctrl->sched.num_cc3 &&
				curexp == (int)slc3->rootexp + expshft) {
				if (slc3->state == SLIM_CH_PENDING_REMOVAL) {
					coeff3++;
					slc3 = ctrl->sched.chc3[coeff3];
					continue;
				}
				opensl1valid = false;
				if (opensl3[1] >= opensl3[0] ||
					(finalexp == (int)slc3->rootexp &&
					 curintr <= 32 &&
					 opensl3[0] == curmaxsl &&
					 last1 < 0)) {
					opensl3[1] -= slc3->seglen;
					slc3->newoff = curmaxsl + opensl3[1];
					if (opensl3[1] < 0 &&
						opensl3[0] == curmaxsl) {
						opensl3[0] += opensl3[1];
						opensl3[1] = 0;
					}
					if (opensl3[0] < 0) {
						dev_dbg(&ctrl->dev,
						"reconfig failed:%d\n",
						__LINE__);
						return -EXFULL;
					}
				} else {
					if (slc3->seglen > opensl3[0]) {
						dev_dbg(&ctrl->dev,
						"reconfig failed:%d\n",
						__LINE__);
						return -EXFULL;
					}
					slc3->newoff = opensl3[0] -
							slc3->seglen;
					opensl3[0] = slc3->newoff;
				}
				slc3->newintr = curintr;
				coeff3++;
				slc3 = ctrl->sched.chc3[coeff3];
			}
			
			if (opensl1valid == false) {
				for (i = 0; i < 3; i++) {
					opensl1[i * 2] = opensl3[0];
					opensl1[(i * 2) + 1] = opensl3[1];
				}
			} else {
				int opensl1p[6];
				memcpy(opensl1p, opensl1, sizeof(opensl1));
				for (i = 0; i < 3; i++) {
					if (opensl1p[i] < opensl1p[i + 3])
						opensl1[(i * 2) + 1] =
							opensl1p[i];
					else
						opensl1[(i * 2) + 1] =
							opensl1p[i + 3];
				}
				for (i = 0; i < 3; i++) {
					opensl1[(i * 2) + 1] -= curmaxsl;
					opensl1[i * 2] = curmaxsl;
					if (opensl1[(i * 2) + 1] < 0) {
						opensl1[i * 2] +=
							opensl1[(i * 2) + 1];
						opensl1[(i * 2) + 1] = 0;
					}
					if (opensl1[i * 2] < 0) {
						dev_dbg(&ctrl->dev,
						"reconfig failed:%d\n",
						__LINE__);
						return -EXFULL;
					}
				}
			}
			
			while (coeff1 < ctrl->sched.num_cc1 &&
				curexp == (int)slc1->rootexp + expshft) {
				
				static const int srcho[] = { 5, 2, 4, 1, 3, 0 };
				int maxopensl = 0;
				int maxi = 0;
				if (slc1->state == SLIM_CH_PENDING_REMOVAL) {
					coeff1++;
					slc1 = ctrl->sched.chc1[coeff1];
					continue;
				}
				opensl1valid = true;
				for (i = 0; i < 6; i++) {
					if (opensl1[srcho[i]] > maxopensl) {
						maxopensl = opensl1[srcho[i]];
						maxi = srcho[i];
					}
				}
				opensl1[maxi] -= slc1->seglen;
				slc1->newoff = (curmaxsl * maxi) +
						opensl1[maxi];
				if (opensl1[maxi] < 0) {
					if (((maxi & 1) == 1) &&
					(opensl1[maxi - 1] == curmaxsl)) {
						opensl1[maxi - 1] +=
							opensl1[maxi];
						if (opensl3[0] >
							opensl1[maxi - 1])
							opensl3[0] =
							opensl1[maxi - 1];
						opensl3[1] = 0;
						opensl1[maxi] = 0;
						if (opensl1[maxi - 1] < 0) {
							dev_dbg(&ctrl->dev,
							"reconfig failed:%d\n",
							__LINE__);
							return -EXFULL;
						}
					} else {
						dev_dbg(&ctrl->dev,
						"reconfig failed:%d\n",
						__LINE__);
						return -EXFULL;
					}
				} else {
					if (opensl3[maxi & 1] > opensl1[maxi])
						opensl3[maxi & 1] =
							opensl1[maxi];
				}
				slc1->newintr = curintr * 3;
				coeff1++;
				slc1 = ctrl->sched.chc1[coeff1];
			}
		}
		
		if (opensl3[1] <= 0 && opensl3[0] <= 0)
			return -EXFULL;
		
		if (opensl3[1] > opensl3[0]) {
			int temp = opensl3[0];
			opensl3[0] = opensl3[1];
			opensl3[1] = temp;
			temp = opensl1[5];
			opensl1[5] = opensl1[4];
			opensl1[4] = opensl1[3];
			opensl1[3] = opensl1[2];
			opensl1[2] = opensl1[1];
			opensl1[1] = opensl1[0];
			opensl1[0] = temp;
			shiftsegoffsets(ctrl, ctrl->sched.chc1,
					ctrl->sched.num_cc1, curmaxsl);
			shiftsegoffsets(ctrl, ctrl->sched.chc3,
					ctrl->sched.num_cc3, curmaxsl);
		}
		
		maxctrlw3 = opensl3[0];
		maxctrlw1 = opensl1[0];
		if (opensl3[0] == curmaxsl)
			maxctrlw3 += opensl3[1];
		for (i = 0; i < 5 && opensl1[i] == curmaxsl; i++)
			maxctrlw1 += opensl1[i + 1];
		if (curintr >= 32) {
			*subfrml = 32;
			*ctrlw = maxctrlw3;
		} else if (curintr == 16) {
			if (maxctrlw3 > (opensl3[1] * 4)) {
				*subfrml = 32;
				*ctrlw = maxctrlw3;
			} else {
				*subfrml = 8;
				*ctrlw = opensl3[1];
			}
		} else {
			if ((maxctrlw1 * 8) >= (maxctrlw3 * 24)) {
				*subfrml = 24;
				*ctrlw = maxctrlw1;
			} else {
				*subfrml = 8;
				*ctrlw = maxctrlw3;
			}
		}
	}
	return 0;
}

#ifdef DEBUG
static int slim_verifychansched(struct slim_controller *ctrl, u32 ctrlw,
				u32 subfrml, u32 clkgear)
{
	int sl, i;
	int cc1 = 0;
	int cc3 = 0;
	struct slim_ich *slc = NULL;
	if (!ctrl->sched.slots)
		return 0;
	memset(ctrl->sched.slots, 0, SLIM_SL_PER_SUPERFRAME);
	dev_dbg(&ctrl->dev, "Clock gear is:%d\n", clkgear);
	for (sl = 0; sl < SLIM_SL_PER_SUPERFRAME; sl += subfrml) {
		for (i = 0; i < ctrlw; i++)
			ctrl->sched.slots[sl + i] = 33;
	}
	while (cc1 < ctrl->sched.num_cc1) {
		slc = ctrl->sched.chc1[cc1];
		if (slc == NULL) {
			dev_err(&ctrl->dev, "SLC1 null in verify: chan%d\n",
				cc1);
			return -EIO;
		}
		dev_dbg(&ctrl->dev, "chan:%d, offset:%d, intr:%d, seglen:%d\n",
				(slc - ctrl->chans), slc->newoff,
				slc->newintr, slc->seglen);

		if (slc->state != SLIM_CH_PENDING_REMOVAL) {
			for (sl = slc->newoff;
				sl < SLIM_SL_PER_SUPERFRAME;
				sl += slc->newintr) {
				for (i = 0; i < slc->seglen; i++) {
					if (ctrl->sched.slots[sl + i])
						return -EXFULL;
					ctrl->sched.slots[sl + i] = cc1 + 1;
				}
			}
		}
		cc1++;
	}
	while (cc3 < ctrl->sched.num_cc3) {
		slc = ctrl->sched.chc3[cc3];
		if (slc == NULL) {
			dev_err(&ctrl->dev, "SLC3 null in verify: chan%d\n",
				cc3);
			return -EIO;
		}
		dev_dbg(&ctrl->dev, "chan:%d, offset:%d, intr:%d, seglen:%d\n",
				(slc - ctrl->chans), slc->newoff,
				slc->newintr, slc->seglen);
		if (slc->state != SLIM_CH_PENDING_REMOVAL) {
			for (sl = slc->newoff;
				sl < SLIM_SL_PER_SUPERFRAME;
				sl += slc->newintr) {
				for (i = 0; i < slc->seglen; i++) {
					if (ctrl->sched.slots[sl + i])
						return -EXFULL;
					ctrl->sched.slots[sl + i] = cc3 + 1;
				}
			}
		}
		cc3++;
	}

	return 0;
}
#else
static int slim_verifychansched(struct slim_controller *ctrl, u32 ctrlw,
				u32 subfrml, u32 clkgear)
{
	return 0;
}
#endif

static void slim_sort_chan_grp(struct slim_controller *ctrl,
				struct slim_ich *slc)
{
	u8  last = (u8)-1;
	u8 second = 0;

	for (; last > 0; last--) {
		struct slim_ich *slc1 = slc;
		struct slim_ich *slc2;
		u8 next = SLIM_HDL_TO_CHIDX(slc1->nextgrp);
		slc2 = &ctrl->chans[next];
		for (second = 1; second <= last && slc2 &&
			(slc2->state == SLIM_CH_ACTIVE ||
			 slc2->state == SLIM_CH_PENDING_ACTIVE); second++) {
			if (slc1->newoff > slc2->newoff) {
				u32 temp = slc2->newoff;
				slc2->newoff = slc1->newoff;
				slc1->newoff = temp;
			}
			if (slc2->nextgrp & SLIM_END_GRP) {
				last = second;
				break;
			}
			slc1 = slc2;
			next = SLIM_HDL_TO_CHIDX(slc1->nextgrp);
			slc2 = &ctrl->chans[next];
		}
		if (slc2 == NULL)
			last = second - 1;
	}
}


static int slim_allocbw(struct slim_device *sb, int *subfrmc, int *clkgear)
{
	u32 msgsl = 0;
	u32 ctrlw = 0;
	u32 subfrml = 0;
	int ret = -EIO;
	struct slim_controller *ctrl = sb->ctrl;
	u32 usedsl = ctrl->sched.usedslots + ctrl->sched.pending_msgsl;
	u32 availsl = SLIM_SL_PER_SUPERFRAME - SLIM_FRM_SLOTS_PER_SUPERFRAME -
			SLIM_GDE_SLOTS_PER_SUPERFRAME;
	*clkgear = SLIM_MAX_CLK_GEAR;

	dev_dbg(&ctrl->dev, "used sl:%u, availlable sl:%u\n", usedsl, availsl);
	dev_dbg(&ctrl->dev, "pending:chan sl:%u, :msg sl:%u, clkgear:%u\n",
				ctrl->sched.usedslots,
				ctrl->sched.pending_msgsl, *clkgear);
	if (ctrl->sched.usedslots != 0) {
		while ((usedsl * 2 <= availsl) && (*clkgear > ctrl->min_cg)) {
			*clkgear -= 1;
			usedsl *= 2;
		}
	}

	for (; *clkgear <= ctrl->max_cg; (*clkgear)++) {
		ret = slim_sched_chans(sb, *clkgear, &ctrlw, &subfrml);

		if (ret == 0) {
			*subfrmc = getsubfrmcoding(&ctrlw, &subfrml, &msgsl);
			if ((msgsl >> (ctrl->max_cg - *clkgear) <
				ctrl->sched.pending_msgsl) &&
				(*clkgear < ctrl->max_cg))
				continue;
			else
				break;
		}
	}
	if (ret == 0) {
		int i;
		
		for (i = 0; i < ctrl->sched.num_cc1; i++) {
			struct slim_ich *slc = ctrl->sched.chc1[i];
			if (slc->state == SLIM_CH_PENDING_REMOVAL)
				continue;
			if ((slc->nextgrp & SLIM_START_GRP) &&
				!(slc->nextgrp & SLIM_END_GRP)) {
				slim_sort_chan_grp(ctrl, slc);
			}
		}
		for (i = 0; i < ctrl->sched.num_cc3; i++) {
			struct slim_ich *slc = ctrl->sched.chc3[i];
			if (slc->state == SLIM_CH_PENDING_REMOVAL)
				continue;
			if ((slc->nextgrp & SLIM_START_GRP) &&
				!(slc->nextgrp & SLIM_END_GRP)) {
				slim_sort_chan_grp(ctrl, slc);
			}
		}

		ret = slim_verifychansched(ctrl, ctrlw, subfrml, *clkgear);
	}

	return ret;
}

static void slim_change_existing_chans(struct slim_controller *ctrl, int coeff)
{
	struct slim_ich **arr;
	int len, i;
	if (coeff == SLIM_COEFF_1) {
		arr = ctrl->sched.chc1;
		len = ctrl->sched.num_cc1;
	} else {
		arr = ctrl->sched.chc3;
		len = ctrl->sched.num_cc3;
	}
	for (i = 0; i < len; i++) {
		struct slim_ich *slc = arr[i];
		if (slc->state == SLIM_CH_ACTIVE ||
			slc->state == SLIM_CH_SUSPENDED)
			slc->offset = slc->newoff;
			slc->interval = slc->newintr;
	}
}
static void slim_chan_changes(struct slim_device *sb, bool revert)
{
	struct slim_controller *ctrl = sb->ctrl;
	while (!list_empty(&sb->mark_define)) {
		struct slim_ich *slc;
		struct slim_pending_ch *pch =
				list_entry(sb->mark_define.next,
					struct slim_pending_ch, pending);
		slc = &ctrl->chans[pch->chan];
		if (revert) {
			if (slc->state == SLIM_CH_PENDING_ACTIVE) {
				u32 sl = slc->seglen << slc->rootexp;
				if (slc->coeff == SLIM_COEFF_3)
					sl *= 3;
				if (!ctrl->allocbw)
					ctrl->sched.usedslots -= sl;
				slim_remove_ch(ctrl, slc);
				slc->state = SLIM_CH_DEFINED;
			}
		} else {
			slc->state = SLIM_CH_ACTIVE;
			slc->def++;
		}
		list_del_init(&pch->pending);
		kfree(pch);
	}

	while (!list_empty(&sb->mark_removal)) {
		struct slim_pending_ch *pch =
				list_entry(sb->mark_removal.next,
					struct slim_pending_ch, pending);
		struct slim_ich *slc = &ctrl->chans[pch->chan];
		u32 sl = slc->seglen << slc->rootexp;
		if (revert || slc->def > 0) {
			if (slc->coeff == SLIM_COEFF_3)
				sl *= 3;
			if (!ctrl->allocbw)
				ctrl->sched.usedslots += sl;
			if (revert)
				slc->def++;
			slc->state = SLIM_CH_ACTIVE;
		} else
			slim_remove_ch(ctrl, slc);
		list_del_init(&pch->pending);
		kfree(pch);
	}

	while (!list_empty(&sb->mark_suspend)) {
		struct slim_pending_ch *pch =
				list_entry(sb->mark_suspend.next,
					struct slim_pending_ch, pending);
		struct slim_ich *slc = &ctrl->chans[pch->chan];
		if (revert)
			slc->state = SLIM_CH_ACTIVE;
		list_del_init(&pch->pending);
		kfree(pch);
	}
	
	if (!revert) {
		slim_change_existing_chans(ctrl, SLIM_COEFF_1);
		slim_change_existing_chans(ctrl, SLIM_COEFF_3);
	}
}

int slim_reconfigure_now(struct slim_device *sb)
{
	u8 i;
	u8 wbuf[4];
	u32 clkgear, subframe;
	u32 curexp;
	int ret;
	struct slim_controller *ctrl = sb->ctrl;
	u32 expshft;
	u32 segdist;
	struct slim_pending_ch *pch;

	mutex_lock(&ctrl->sched.m_reconf);
	if (sb->pending_msgsl == sb->cur_msgsl &&
		list_empty(&sb->mark_define) &&
		list_empty(&sb->mark_suspend)) {
		struct list_head *pos, *next;
		list_for_each_safe(pos, next, &sb->mark_removal) {
			struct slim_ich *slc;
			pch = list_entry(pos, struct slim_pending_ch, pending);
			slc = &ctrl->chans[pch->chan];
			if (slc->def > 0)
				slc->def--;
			
			if (SLIM_HDL_TO_LA(slc->srch) == sb->laddr)
				slc->srch = 0;
			if (slc->def != 0 && !ctrl->allocbw) {
				list_del(&pch->pending);
				kfree(pch);
			}
		}
		if (list_empty(&sb->mark_removal)) {
			mutex_unlock(&ctrl->sched.m_reconf);
			pr_info("SLIM_CL: skip reconfig sequence");
			return 0;
		}
	}

	ctrl->sched.pending_msgsl += sb->pending_msgsl - sb->cur_msgsl;
	list_for_each_entry(pch, &sb->mark_define, pending) {
		struct slim_ich *slc = &ctrl->chans[pch->chan];
		slim_add_ch(ctrl, slc);
		if (slc->state < SLIM_CH_ACTIVE)
			slc->state = SLIM_CH_PENDING_ACTIVE;
	}

	list_for_each_entry(pch, &sb->mark_removal, pending) {
		struct slim_ich *slc = &ctrl->chans[pch->chan];
		u32 sl = slc->seglen << slc->rootexp;
		if (slc->coeff == SLIM_COEFF_3)
			sl *= 3;
		if (!ctrl->allocbw)
			ctrl->sched.usedslots -= sl;
		slc->state = SLIM_CH_PENDING_REMOVAL;
	}
	list_for_each_entry(pch, &sb->mark_suspend, pending) {
		struct slim_ich *slc = &ctrl->chans[pch->chan];
		slc->state = SLIM_CH_SUSPENDED;
	}

	if (ctrl->allocbw)
		ret = ctrl->allocbw(sb, &subframe, &clkgear);
	else
		ret = slim_allocbw(sb, &subframe, &clkgear);

	if (!ret) {
		ret = slim_processtxn(ctrl, SLIM_MSG_DEST_BROADCAST,
			SLIM_MSG_MC_BEGIN_RECONFIGURATION, 0, SLIM_MSG_MT_CORE,
			NULL, NULL, 0, 3, NULL, 0, NULL);
		dev_dbg(&ctrl->dev, "sending begin_reconfig:ret:%d\n", ret);
	}

	if (!ret && subframe != ctrl->sched.subfrmcode) {
		wbuf[0] = (u8)(subframe & 0xFF);
		ret = slim_processtxn(ctrl, SLIM_MSG_DEST_BROADCAST,
			SLIM_MSG_MC_NEXT_SUBFRAME_MODE, 0, SLIM_MSG_MT_CORE,
			NULL, (u8 *)&subframe, 1, 4, NULL, 0, NULL);
		dev_dbg(&ctrl->dev, "sending subframe:%d,ret:%d\n",
				(int)wbuf[0], ret);
	}
	if (!ret && clkgear != ctrl->clkgear) {
		wbuf[0] = (u8)(clkgear & 0xFF);
		ret = slim_processtxn(ctrl, SLIM_MSG_DEST_BROADCAST,
			SLIM_MSG_MC_NEXT_CLOCK_GEAR, 0, SLIM_MSG_MT_CORE,
			NULL, wbuf, 1, 4, NULL, 0, NULL);
		dev_dbg(&ctrl->dev, "sending clkgear:%d,ret:%d\n",
				(int)wbuf[0], ret);
	}
	if (ret)
		goto revert_reconfig;

	expshft = SLIM_MAX_CLK_GEAR - clkgear;
	
	list_for_each_entry(pch, &sb->mark_define, pending) {
		struct slim_ich *slc = &ctrl->chans[pch->chan];
		
		wbuf[0] = slc->chan;
		wbuf[1] = slc->prrate;
		wbuf[2] = slc->prop.dataf | (slc->prop.auxf << 4);
		wbuf[3] = slc->prop.sampleszbits / SLIM_CL_PER_SL;
		dev_dbg(&ctrl->dev, "define content, activate:%x, %x, %x, %x\n",
				wbuf[0], wbuf[1], wbuf[2], wbuf[3]);
		
		ret = slim_processtxn(ctrl, SLIM_MSG_DEST_BROADCAST,
				SLIM_MSG_MC_NEXT_DEFINE_CONTENT, 0,
				SLIM_MSG_MT_CORE, NULL, (u8 *)&wbuf, 4, 7,
				NULL, 0, NULL);
		if (ret)
			goto revert_reconfig;

		ret = slim_processtxn(ctrl, SLIM_MSG_DEST_BROADCAST,
				SLIM_MSG_MC_NEXT_ACTIVATE_CHANNEL, 0,
				SLIM_MSG_MT_CORE, NULL, (u8 *)&wbuf, 1, 4,
				NULL, 0, NULL);
		if (ret)
			goto revert_reconfig;
	}

	list_for_each_entry(pch, &sb->mark_removal, pending) {
		struct slim_ich *slc = &ctrl->chans[pch->chan];
		dev_dbg(&ctrl->dev, "remove chan:%x\n", pch->chan);
		wbuf[0] = slc->chan;
		ret = slim_processtxn(ctrl, SLIM_MSG_DEST_BROADCAST,
				SLIM_MSG_MC_NEXT_REMOVE_CHANNEL, 0,
				SLIM_MSG_MT_CORE, NULL, wbuf, 1, 4,
				NULL, 0, NULL);
		if (ret)
			goto revert_reconfig;
	}
	list_for_each_entry(pch, &sb->mark_suspend, pending) {
		struct slim_ich *slc = &ctrl->chans[pch->chan];
		dev_dbg(&ctrl->dev, "suspend chan:%x\n", pch->chan);
		wbuf[0] = slc->chan;
		ret = slim_processtxn(ctrl, SLIM_MSG_DEST_BROADCAST,
				SLIM_MSG_MC_NEXT_DEACTIVATE_CHANNEL, 0,
				SLIM_MSG_MT_CORE, NULL, wbuf, 1, 4,
				NULL, 0, NULL);
		if (ret)
			goto revert_reconfig;
	}

	
	for (i = 0; i < ctrl->sched.num_cc1; i++) {
		struct slim_ich *slc = ctrl->sched.chc1[i];
		if (slc->state == SLIM_CH_PENDING_REMOVAL)
			continue;
		curexp = slc->rootexp + expshft;
		segdist = (slc->newoff << curexp) & 0x1FF;
		expshft = SLIM_MAX_CLK_GEAR - clkgear;
		dev_dbg(&ctrl->dev, "new-intr:%d, old-intr:%d, dist:%d\n",
				slc->newintr, slc->interval, segdist);
		dev_dbg(&ctrl->dev, "new-off:%d, old-off:%d\n",
				slc->newoff, slc->offset);

		if (slc->state < SLIM_CH_ACTIVE || slc->def < slc->ref ||
			slc->newintr != slc->interval ||
			slc->newoff != slc->offset) {
			segdist |= 0x200;
			segdist >>= curexp;
			segdist |= (slc->newoff << (curexp + 1)) & 0xC00;
			wbuf[0] = slc->chan;
			wbuf[1] = (u8)(segdist & 0xFF);
			wbuf[2] = (u8)((segdist & 0xF00) >> 8) |
					(slc->prop.prot << 4);
			wbuf[3] = slc->seglen;
			ret = slim_processtxn(ctrl, SLIM_MSG_DEST_BROADCAST,
					SLIM_MSG_MC_NEXT_DEFINE_CHANNEL, 0,
					SLIM_MSG_MT_CORE, NULL, (u8 *)wbuf, 4,
					7, NULL, 0, NULL);
			if (ret)
				goto revert_reconfig;
		}
	}

	
	for (i = 0; i < ctrl->sched.num_cc3; i++) {
		struct slim_ich *slc = ctrl->sched.chc3[i];
		if (slc->state == SLIM_CH_PENDING_REMOVAL)
			continue;
		curexp = slc->rootexp + expshft;
		segdist = (slc->newoff << curexp) & 0x1FF;
		expshft = SLIM_MAX_CLK_GEAR - clkgear;
		dev_dbg(&ctrl->dev, "new-intr:%d, old-intr:%d, dist:%d\n",
				slc->newintr, slc->interval, segdist);
		dev_dbg(&ctrl->dev, "new-off:%d, old-off:%d\n",
				slc->newoff, slc->offset);

		if (slc->state < SLIM_CH_ACTIVE || slc->def < slc->ref ||
			slc->newintr != slc->interval ||
			slc->newoff != slc->offset) {
			segdist |= 0x200;
			segdist >>= curexp;
			segdist |= 0xC00;
			wbuf[0] = slc->chan;
			wbuf[1] = (u8)(segdist & 0xFF);
			wbuf[2] = (u8)((segdist & 0xF00) >> 8) |
					(slc->prop.prot << 4);
			wbuf[3] = (u8)(slc->seglen);
			ret = slim_processtxn(ctrl, SLIM_MSG_DEST_BROADCAST,
					SLIM_MSG_MC_NEXT_DEFINE_CHANNEL, 0,
					SLIM_MSG_MT_CORE, NULL, (u8 *)wbuf, 4,
					7, NULL, 0, NULL);
			if (ret)
				goto revert_reconfig;
		}
	}
	ret = slim_processtxn(ctrl, SLIM_MSG_DEST_BROADCAST,
			SLIM_MSG_MC_RECONFIGURE_NOW, 0, SLIM_MSG_MT_CORE, NULL,
			NULL, 0, 3, NULL, 0, NULL);
	dev_dbg(&ctrl->dev, "reconfig now:ret:%d\n", ret);
	if (!ret) {
		ctrl->sched.subfrmcode = subframe;
		ctrl->clkgear = clkgear;
		ctrl->sched.msgsl = ctrl->sched.pending_msgsl;
		sb->cur_msgsl = sb->pending_msgsl;
		slim_chan_changes(sb, false);
		mutex_unlock(&ctrl->sched.m_reconf);
		return 0;
	}

revert_reconfig:
	
	slim_chan_changes(sb, true);
	mutex_unlock(&ctrl->sched.m_reconf);
	return ret;
}
EXPORT_SYMBOL_GPL(slim_reconfigure_now);

static int add_pending_ch(struct list_head *listh, u8 chan)
{
	struct slim_pending_ch *pch;
	pch = kmalloc(sizeof(struct slim_pending_ch), GFP_KERNEL);
	if (!pch)
		return -ENOMEM;
	pch->chan = chan;
	list_add_tail(&pch->pending, listh);
	return 0;
}

int slim_control_ch(struct slim_device *sb, u16 chanh,
			enum slim_ch_control chctrl, bool commit)
{
	struct slim_controller *ctrl = sb->ctrl;
	int ret = 0;
	
	u8 chan = SLIM_HDL_TO_CHIDX(chanh);
	u8 nchan = 0;
	struct slim_ich *slc = &ctrl->chans[chan];
	if (!(slc->nextgrp & SLIM_START_GRP))
		return -EINVAL;

	mutex_lock(&sb->sldev_reconf);
	do {
		struct slim_pending_ch *pch;
		u8 add_mark_removal  = true;

		slc = &ctrl->chans[chan];
		dev_dbg(&ctrl->dev, "chan:%d,ctrl:%d,def:%d", chan, chctrl,
					slc->def);
		if (slc->state < SLIM_CH_DEFINED) {
			ret = -ENOTCONN;
			break;
		}
		if (chctrl == SLIM_CH_SUSPEND) {
			ret = add_pending_ch(&sb->mark_suspend, chan);
			if (ret)
				break;
		} else if (chctrl == SLIM_CH_ACTIVATE) {
			if (slc->state > SLIM_CH_ACTIVE) {
				ret = -EISCONN;
				break;
			}
			ret = add_pending_ch(&sb->mark_define, chan);
			if (ret)
				break;
		} else {
			if (slc->state < SLIM_CH_ACTIVE) {
				ret = -ENOTCONN;
				break;
			}
			if (!list_empty(&sb->mark_define)) {
				struct list_head *pos, *next;
				list_for_each_safe(pos, next,
						  &sb->mark_define) {
					pch = list_entry(pos,
						struct slim_pending_ch,
						pending);
					if (pch->chan == chan) {
						list_del(&pch->pending);
						kfree(pch);
						add_mark_removal = false;
						break;
					}
				}
			}
			if (add_mark_removal == true) {
				ret = add_pending_ch(&sb->mark_removal, chan);
				if (ret)
					break;
			}
		}

		nchan++;
		if (nchan < SLIM_GRP_TO_NCHAN(chanh))
			chan = SLIM_HDL_TO_CHIDX(slc->nextgrp);
	} while (nchan < SLIM_GRP_TO_NCHAN(chanh));
	if (!ret && commit == true)
		ret = slim_reconfigure_now(sb);
	mutex_unlock(&sb->sldev_reconf);
	return ret;
}
EXPORT_SYMBOL_GPL(slim_control_ch);

int slim_reservemsg_bw(struct slim_device *sb, u32 bw_bps, bool commit)
{
	struct slim_controller *ctrl = sb->ctrl;
	int ret = 0;
	int sl;
	mutex_lock(&sb->sldev_reconf);
	if ((bw_bps >> 3) >= ctrl->a_framer->rootfreq)
		sl = SLIM_SL_PER_SUPERFRAME;
	else {
		sl = (bw_bps * (SLIM_CL_PER_SUPERFRAME_DIV8/SLIM_CL_PER_SL/2) +
			(ctrl->a_framer->rootfreq/2 - 1)) /
			(ctrl->a_framer->rootfreq/2);
	}
	dev_dbg(&ctrl->dev, "request:bw:%d, slots:%d, current:%d\n", bw_bps, sl,
						sb->cur_msgsl);
	sb->pending_msgsl = sl;
	if (commit == true)
		ret = slim_reconfigure_now(sb);
	mutex_unlock(&sb->sldev_reconf);
	return ret;
}
EXPORT_SYMBOL_GPL(slim_reservemsg_bw);

int slim_ctrl_clk_pause(struct slim_controller *ctrl, bool wakeup, u8 restart)
{
	int ret = 0;
	int i;

	if (wakeup == false && restart > SLIM_CLK_UNSPECIFIED)
		return -EINVAL;
	mutex_lock(&ctrl->m_ctrl);
	if (wakeup) {
		if (ctrl->clk_state == SLIM_CLK_ACTIVE) {
			mutex_unlock(&ctrl->m_ctrl);
			return 0;
		}
		wait_for_completion(&ctrl->pause_comp);
		if (ctrl->clk_state == SLIM_CLK_PAUSED && ctrl->wakeup)
			ret = ctrl->wakeup(ctrl);
		if (!ret)
			ctrl->clk_state = SLIM_CLK_ACTIVE;
		else
			complete(&ctrl->pause_comp);
		mutex_unlock(&ctrl->m_ctrl);
		return ret;
	} else {
		switch (ctrl->clk_state) {
		case SLIM_CLK_ENTERING_PAUSE:
		case SLIM_CLK_PAUSE_FAILED:
			wait_for_completion(&ctrl->pause_comp);
			
			if (ctrl->clk_state == SLIM_CLK_PAUSE_FAILED) {
				ctrl->clk_state = SLIM_CLK_ACTIVE;
				break;
			} else {
				mutex_unlock(&ctrl->m_ctrl);
				complete(&ctrl->pause_comp);
				return 0;
			}
			break;
		case SLIM_CLK_PAUSED:
			
			mutex_unlock(&ctrl->m_ctrl);
			return 0;
		case SLIM_CLK_ACTIVE:
		default:
			break;
		}
	}
	
	for (i = 0; i < ctrl->last_tid; i++) {
		if (ctrl->txnt[i]) {
			ret = -EBUSY;
			pr_info("slim_clk_pause: txn-rsp for %d pending", i);
			mutex_unlock(&ctrl->m_ctrl);
			return -EBUSY;
		}
	}
	ctrl->clk_state = SLIM_CLK_ENTERING_PAUSE;
	mutex_unlock(&ctrl->m_ctrl);

	mutex_lock(&ctrl->sched.m_reconf);
	
	if (ctrl->sched.usedslots) {
		pr_info("slim_clk_pause: data channel active");
		ret = -EBUSY;
		goto clk_pause_ret;
	}

	ret = slim_processtxn(ctrl, SLIM_MSG_DEST_BROADCAST,
		SLIM_MSG_CLK_PAUSE_SEQ_FLG | SLIM_MSG_MC_BEGIN_RECONFIGURATION,
		0, SLIM_MSG_MT_CORE, NULL, NULL, 0, 3, NULL, 0, NULL);
	if (ret)
		goto clk_pause_ret;

	ret = slim_processtxn(ctrl, SLIM_MSG_DEST_BROADCAST,
		SLIM_MSG_CLK_PAUSE_SEQ_FLG | SLIM_MSG_MC_NEXT_PAUSE_CLOCK, 0,
		SLIM_MSG_MT_CORE, NULL, &restart, 1, 4, NULL, 0, NULL);
	if (ret)
		goto clk_pause_ret;

	ret = slim_processtxn(ctrl, SLIM_MSG_DEST_BROADCAST,
		SLIM_MSG_CLK_PAUSE_SEQ_FLG | SLIM_MSG_MC_RECONFIGURE_NOW, 0,
		SLIM_MSG_MT_CORE, NULL, NULL, 0, 3, NULL, 0, NULL);
	if (ret)
		goto clk_pause_ret;

clk_pause_ret:
	if (ret)
		ctrl->clk_state = SLIM_CLK_PAUSE_FAILED;
	else
		ctrl->clk_state = SLIM_CLK_PAUSED;
	complete(&ctrl->pause_comp);
	mutex_unlock(&ctrl->sched.m_reconf);
	return ret;
}
EXPORT_SYMBOL_GPL(slim_ctrl_clk_pause);

MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1");
MODULE_DESCRIPTION("Slimbus module");
MODULE_ALIAS("platform:slimbus");
