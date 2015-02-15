/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/idr.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/spmi.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <mach/devices_cmdline.h>

#include "spmi-dbgfs.h"

struct spmii_boardinfo {
	struct list_head	list;
	struct spmi_boardinfo	board_info;
};

static DEFINE_MUTEX(board_lock);
static LIST_HEAD(board_list);
static DEFINE_IDR(ctrl_idr);
static struct device_type spmi_dev_type;
static struct device_type spmi_ctrl_type;

struct bus_type spmi_bus_type;
static int spmi_register_controller(struct spmi_controller *ctrl);

struct spmi_controller *spmi_busnum_to_ctrl(u32 bus_num)
{
	struct spmi_controller *ctrl;

	mutex_lock(&board_lock);
	ctrl = idr_find(&ctrl_idr, bus_num);
	mutex_unlock(&board_lock);

	return ctrl;
}
EXPORT_SYMBOL_GPL(spmi_busnum_to_ctrl);

int spmi_add_controller(struct spmi_controller *ctrl)
{
	int	id;
	int	status;

	if (!ctrl)
		return -EINVAL;

	pr_debug("adding controller for bus %d (0x%p)\n", ctrl->nr, ctrl);

	if (ctrl->nr & ~MAX_ID_MASK) {
		pr_err("invalid bus identifier %d\n", ctrl->nr);
		return -EINVAL;
	}

retry:
	if (idr_pre_get(&ctrl_idr, GFP_KERNEL) == 0) {
		pr_err("no free memory for idr\n");
		return -ENOMEM;
	}

	mutex_lock(&board_lock);
	status = idr_get_new_above(&ctrl_idr, ctrl, ctrl->nr, &id);
	if (status == 0 && id != ctrl->nr) {
		status = -EBUSY;
		idr_remove(&ctrl_idr, id);
	}
	mutex_unlock(&board_lock);
	if (status == -EAGAIN)
		goto retry;

	if (status == 0)
		status = spmi_register_controller(ctrl);
	return status;
}
EXPORT_SYMBOL_GPL(spmi_add_controller);

static int spmi_ctrl_remove_device(struct device *dev, void *data)
{
	struct spmi_device *spmidev = to_spmi_device(dev);
	struct spmi_controller *ctrl = data;

	if (dev->type == &spmi_dev_type && spmidev->ctrl == ctrl)
		spmi_remove_device(spmidev);

	return 0;
}

int spmi_del_controller(struct spmi_controller *ctrl)
{
	struct spmi_controller *found;

	if (!ctrl)
		return -EINVAL;

	
	mutex_lock(&board_lock);
	found = idr_find(&ctrl_idr, ctrl->nr);
	mutex_unlock(&board_lock);
	if (found != ctrl)
		return -EINVAL;

	
	mutex_lock(&board_lock);
	bus_for_each_dev(&spmi_bus_type, NULL, ctrl, spmi_ctrl_remove_device);
	mutex_unlock(&board_lock);

	spmi_dfs_del_controller(ctrl);

	mutex_lock(&board_lock);
	idr_remove(&ctrl_idr, ctrl->nr);
	mutex_unlock(&board_lock);

	init_completion(&ctrl->dev_released);
	device_unregister(&ctrl->dev);
	wait_for_completion(&ctrl->dev_released);

	return 0;
}
EXPORT_SYMBOL_GPL(spmi_del_controller);

#define spmi_ctrl_attr_gr NULL
static void spmi_ctrl_release(struct device *dev)
{
	struct spmi_controller *ctrl = to_spmi_controller(dev);

	complete(&ctrl->dev_released);
}

static struct device_type spmi_ctrl_type = {
	.groups		= spmi_ctrl_attr_gr,
	.release	= spmi_ctrl_release,
};

#define spmi_device_attr_gr NULL
#define spmi_device_uevent NULL
static void spmi_dev_release(struct device *dev)
{
	struct spmi_device *spmidev = to_spmi_device(dev);
	kfree(spmidev);
}

static struct device_type spmi_dev_type = {
	.groups		= spmi_device_attr_gr,
	.uevent		= spmi_device_uevent,
	.release	= spmi_dev_release,
};

struct spmi_device *spmi_alloc_device(struct spmi_controller *ctrl)
{
	struct spmi_device *spmidev;

	if (!ctrl || !spmi_busnum_to_ctrl(ctrl->nr)) {
		pr_err("Missing SPMI controller\n");
		return NULL;
	}

	spmidev = kzalloc(sizeof(*spmidev), GFP_KERNEL);
	if (!spmidev) {
		dev_err(&ctrl->dev, "unable to allocate spmi_device\n");
		return NULL;
	}

	spmidev->ctrl = ctrl;
	spmidev->dev.parent = ctrl->dev.parent;
	spmidev->dev.bus = &spmi_bus_type;
	spmidev->dev.type = &spmi_dev_type;
	device_initialize(&spmidev->dev);

	return spmidev;
}
EXPORT_SYMBOL_GPL(spmi_alloc_device);

static struct device *get_valid_device(struct spmi_device *spmidev)
{
	struct device *dev;

	if (!spmidev)
		return NULL;

	dev = &spmidev->dev;
	if (dev->bus != &spmi_bus_type || dev->type != &spmi_dev_type)
		return NULL;

	if (!spmidev->ctrl || !spmi_busnum_to_ctrl(spmidev->ctrl->nr))
		return NULL;

	return dev;
}

int spmi_add_device(struct spmi_device *spmidev)
{
	int rc;
	struct device *dev = get_valid_device(spmidev);

	if (!dev) {
		pr_err("invalid SPMI device\n");
		return -EINVAL;
	}

	
	dev_set_name(dev, "%s-%p", spmidev->name, spmidev);

	
	rc = device_add(dev);

	if (rc < 0)
		dev_err(dev, "Can't add %s, status %d\n", dev_name(dev), rc);
	else
		dev_dbg(dev, "device %s registered\n", dev_name(dev));

	return rc;
}
EXPORT_SYMBOL_GPL(spmi_add_device);

struct spmi_device *spmi_new_device(struct spmi_controller *ctrl,
					struct spmi_boardinfo const *info)
{
	struct spmi_device *spmidev;
	int rc;

	if (!ctrl || !info)
		return NULL;

	spmidev = spmi_alloc_device(ctrl);
	if (!spmidev)
		return NULL;

	spmidev->name = info->name;
	spmidev->sid  = info->slave_id;
	spmidev->dev.of_node = info->of_node;
	spmidev->dev.platform_data = (void *)info->platform_data;
	spmidev->num_dev_node = info->num_dev_node;
	spmidev->dev_node = info->dev_node;
	spmidev->res = info->res;

	rc = spmi_add_device(spmidev);
	if (rc < 0) {
		spmi_dev_put(spmidev);
		return NULL;
	}

	return spmidev;
}
EXPORT_SYMBOL_GPL(spmi_new_device);

void spmi_remove_device(struct spmi_device *spmi_dev)
{
	device_unregister(&spmi_dev->dev);
}
EXPORT_SYMBOL_GPL(spmi_remove_device);

static void spmi_match_ctrl_to_boardinfo(struct spmi_controller *ctrl,
				struct spmi_boardinfo *bi)
{
	struct spmi_device *spmidev;

	spmidev = spmi_new_device(ctrl, bi);
	if (!spmidev)
		dev_err(ctrl->dev.parent, "can't create new device for %s\n",
			bi->name);
}

int spmi_register_board_info(int busnum,
			struct spmi_boardinfo const *info, unsigned n)
{
	int i;
	struct spmii_boardinfo *bi;
	struct spmi_controller *ctrl;

	bi = kzalloc(n * sizeof(*bi), GFP_KERNEL);
	if (!bi)
		return -ENOMEM;

	ctrl = spmi_busnum_to_ctrl(busnum);

	for (i = 0; i < n; i++, bi++, info++) {

		memcpy(&bi->board_info, info, sizeof(*info));
		mutex_lock(&board_lock);
		list_add_tail(&bi->list, &board_list);

		if (ctrl)
			spmi_match_ctrl_to_boardinfo(ctrl, &bi->board_info);
		mutex_unlock(&board_lock);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(spmi_register_board_info);


static inline int
spmi_cmd(struct spmi_controller *ctrl, u8 opcode, u8 sid)
{
	if (!ctrl || !ctrl->cmd || ctrl->dev.type != &spmi_ctrl_type)
		return -EINVAL;

	return ctrl->cmd(ctrl, opcode, sid);
}

static inline int spmi_read_cmd(struct spmi_controller *ctrl,
				u8 opcode, u8 sid, u16 addr, u8 bc, u8 *buf)
{
	if (!ctrl || !ctrl->read_cmd || ctrl->dev.type != &spmi_ctrl_type)
		return -EINVAL;

	return ctrl->read_cmd(ctrl, opcode, sid, addr, bc, buf);
}

static inline int spmi_write_cmd(struct spmi_controller *ctrl,
				u8 opcode, u8 sid, u16 addr, u8 bc, u8 *buf)
{
	if (!ctrl || !ctrl->write_cmd || ctrl->dev.type != &spmi_ctrl_type)
		return -EINVAL;

	return ctrl->write_cmd(ctrl, opcode, sid, addr, bc, buf);
}


int spmi_register_read(struct spmi_controller *ctrl, u8 sid, u8 addr, u8 *buf)
{
	
	if (sid > SPMI_MAX_SLAVE_ID || addr > 0x1F)
		return -EINVAL;

	return spmi_read_cmd(ctrl, SPMI_CMD_READ, sid, addr, 0, buf);
}
EXPORT_SYMBOL_GPL(spmi_register_read);

int spmi_ext_register_read(struct spmi_controller *ctrl,
				u8 sid, u8 addr, u8 *buf, int len)
{
	
	if (sid > SPMI_MAX_SLAVE_ID || len <= 0 || len > 16)
		return -EINVAL;

	return spmi_read_cmd(ctrl, SPMI_CMD_EXT_READ, sid, addr, len - 1, buf);
}
EXPORT_SYMBOL_GPL(spmi_ext_register_read);

int spmi_ext_register_readl(struct spmi_controller *ctrl,
				u8 sid, u16 addr, u8 *buf, int len)
{
	
	if (sid > SPMI_MAX_SLAVE_ID || len <= 0 || len > 8)
		return -EINVAL;

	return spmi_read_cmd(ctrl, SPMI_CMD_EXT_READL, sid, addr, len - 1, buf);
}
EXPORT_SYMBOL_GPL(spmi_ext_register_readl);

int spmi_register_write(struct spmi_controller *ctrl, u8 sid, u8 addr, u8 *buf)
{
	u8 op = SPMI_CMD_WRITE;

	
	if (sid > SPMI_MAX_SLAVE_ID || addr > 0x1F)
		return -EINVAL;

	return spmi_write_cmd(ctrl, op, sid, addr, 0, buf);
}
EXPORT_SYMBOL_GPL(spmi_register_write);

/**
 * spmi_register_zero_write() - register zero write
 * @dev: SPMI device.
 * @sid: slave identifier.
 * @data: the data to be written to register 0 (7-bits).
 *
 * Writes data to register 0 of the Slave device.
 */
int spmi_register_zero_write(struct spmi_controller *ctrl, u8 sid, u8 data)
{
	u8 op = SPMI_CMD_ZERO_WRITE;

	
	if (sid > SPMI_MAX_SLAVE_ID)
		return -EINVAL;

	return spmi_write_cmd(ctrl, op, sid, 0, 0, &data);
}
EXPORT_SYMBOL_GPL(spmi_register_zero_write);

int spmi_ext_register_write(struct spmi_controller *ctrl,
				u8 sid, u8 addr, u8 *buf, int len)
{
	u8 op = SPMI_CMD_EXT_WRITE;

	
	if (sid > SPMI_MAX_SLAVE_ID || len <= 0 || len > 16)
		return -EINVAL;

	return spmi_write_cmd(ctrl, op, sid, addr, len - 1, buf);
}
EXPORT_SYMBOL_GPL(spmi_ext_register_write);

int spmi_ext_register_writel(struct spmi_controller *ctrl,
				u8 sid, u16 addr, u8 *buf, int len)
{
	u8 op = SPMI_CMD_EXT_WRITEL;

	
	if (sid > SPMI_MAX_SLAVE_ID || len <= 0 || len > 8)
		return -EINVAL;

	return spmi_write_cmd(ctrl, op, sid, addr, len - 1, buf);
}
EXPORT_SYMBOL_GPL(spmi_ext_register_writel);

int spmi_command_reset(struct spmi_controller *ctrl, u8 sid)
{
	if (sid > SPMI_MAX_SLAVE_ID)
		return -EINVAL;
	return spmi_cmd(ctrl, SPMI_CMD_RESET, sid);
}
EXPORT_SYMBOL_GPL(spmi_command_reset);

int spmi_command_sleep(struct spmi_controller *ctrl, u8 sid)
{
	if (sid > SPMI_MAX_SLAVE_ID)
		return -EINVAL;
	return spmi_cmd(ctrl, SPMI_CMD_SLEEP, sid);
}
EXPORT_SYMBOL_GPL(spmi_command_sleep);

int spmi_command_wakeup(struct spmi_controller *ctrl, u8 sid)
{
	if (sid > SPMI_MAX_SLAVE_ID)
		return -EINVAL;
	return spmi_cmd(ctrl, SPMI_CMD_WAKEUP, sid);
}
EXPORT_SYMBOL_GPL(spmi_command_wakeup);

int spmi_command_shutdown(struct spmi_controller *ctrl, u8 sid)
{
	if (sid > SPMI_MAX_SLAVE_ID)
		return -EINVAL;
	return spmi_cmd(ctrl, SPMI_CMD_SHUTDOWN, sid);
}
EXPORT_SYMBOL_GPL(spmi_command_shutdown);


static const struct spmi_device_id *spmi_match(const struct spmi_device_id *id,
		const struct spmi_device *spmi_dev)
{
	while (id->name[0]) {
		if (strncmp(spmi_dev->name, id->name, SPMI_NAME_SIZE) == 0)
			return id;
		id++;
	}
	return NULL;
}

static int spmi_device_match(struct device *dev, struct device_driver *drv)
{
	struct spmi_device *spmi_dev;
	struct spmi_driver *sdrv = to_spmi_driver(drv);

	if (dev->type == &spmi_dev_type)
		spmi_dev = to_spmi_device(dev);
	else
		return 0;

	
	if (of_driver_match_device(dev, drv))
		return 1;

	if (sdrv->id_table)
		return spmi_match(sdrv->id_table, spmi_dev) != NULL;

	if (drv->name)
		return strncmp(spmi_dev->name, drv->name, SPMI_NAME_SIZE) == 0;
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int spmi_legacy_suspend(struct device *dev, pm_message_t mesg)
{
	struct spmi_device *spmi_dev = NULL;
	struct spmi_driver *driver;
	if (dev->type == &spmi_dev_type)
		spmi_dev = to_spmi_device(dev);

	if (!spmi_dev || !dev->driver)
		return 0;

	driver = to_spmi_driver(dev->driver);
	if (!driver->suspend)
		return 0;

	return driver->suspend(spmi_dev, mesg);
}

static int spmi_legacy_resume(struct device *dev)
{
	struct spmi_device *spmi_dev = NULL;
	struct spmi_driver *driver;
	if (dev->type == &spmi_dev_type)
		spmi_dev = to_spmi_device(dev);

	if (!spmi_dev || !dev->driver)
		return 0;

	driver = to_spmi_driver(dev->driver);
	if (!driver->resume)
		return 0;

	return driver->resume(spmi_dev);
}

static int spmi_pm_suspend(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_suspend(dev);
	else
		return spmi_legacy_suspend(dev, PMSG_SUSPEND);
}

static int spmi_pm_resume(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_resume(dev);
	else
		return spmi_legacy_resume(dev);
}

#else
#define spmi_pm_suspend		NULL
#define spmi_pm_resume		NULL
#endif

static const struct dev_pm_ops spmi_pm_ops = {
	.suspend = spmi_pm_suspend,
	.resume = spmi_pm_resume,
	SET_RUNTIME_PM_OPS(
		pm_generic_suspend,
		pm_generic_resume,
		pm_generic_runtime_idle
		)
};
struct bus_type spmi_bus_type = {
	.name		= "spmi",
	.match		= spmi_device_match,
	.pm		= &spmi_pm_ops,
};
EXPORT_SYMBOL_GPL(spmi_bus_type);

struct device spmi_dev = {
	.init_name = "spmi",
};

static int spmi_drv_probe(struct device *dev)
{
	const struct spmi_driver *sdrv = to_spmi_driver(dev->driver);

	return sdrv->probe(to_spmi_device(dev));
}

static int spmi_drv_remove(struct device *dev)
{
	const struct spmi_driver *sdrv = to_spmi_driver(dev->driver);

	return sdrv->remove(to_spmi_device(dev));
}

static void spmi_drv_shutdown(struct device *dev)
{
	const struct spmi_driver *sdrv = to_spmi_driver(dev->driver);

	sdrv->shutdown(to_spmi_device(dev));
}

int spmi_driver_register(struct spmi_driver *drv)
{
	drv->driver.bus = &spmi_bus_type;

	if (drv->probe)
		drv->driver.probe = spmi_drv_probe;

	if (drv->remove)
		drv->driver.remove = spmi_drv_remove;

	if (drv->shutdown)
		drv->driver.shutdown = spmi_drv_shutdown;

	return driver_register(&drv->driver);
}
EXPORT_SYMBOL_GPL(spmi_driver_register);

#ifdef CONFIG_HTC_POWER_DEBUG
#define MAX_REG_PER_TRANSACTION	(8)

#define PON_REVISION2				0x801
#define PON_PON_REASON1				0x808
#define PON_WARM_RESET_REASON1			0x80A
#define PON_WARM_RESET_REASON2			0x80B
#define PON_POFF_REASON1			0x80C
#define PON_POFF_REASON2			0x80D
#define PON_SOFT_RESET_REASON1			0x80E
#define PON_SOFT_RESET_REASON2			0x80F

enum {
        HARD_RESET_TRIGGERED_BIT,
        SMPL_TRIGGERED_BIT,
        RTC_TRIGGERED_BIT,
        DC_CHG_TRIGGERED_BIT,
        USB_CHG_TRIGGERED_BIT,
        PON1_TRIGGERED_BIT,
        CBLPWR_N_TRIGGERED_BIT,
        KPDPWR_N_TRIGGERED_BIT,
        PON_BIT_MAX,
} pon_reason_bit;

enum {
	SOFT_TRIGGERED_BIT,
	PS_HOLD_TRIGGERED_BIT,
	PMIC_WD_TRIGGERED_BIT,
	GP1_TRIGGERED_BIT,
	GP2_TRIGGERED_BIT,
	KPDPWR_AND_RESIN_TRIGGERED_BIT,
	RESIN_N_TRIGGERED_BIT,
	KPDPWR_TRIGGERED_BIT,
	WARM_REASON1_BIT_MAX,
} warm_soft_reset_poff_reason1_bit;

enum {
	AFP_TRIGGERED_BIT = 4,
	WARM_REASON2_BIT_MAX,
} warm_soft_reset_reason2_bit;

enum {
	CHARGER_TRIGGERED_BIT = 3,
	POFF_AFP_TRIGGERED_BIT,
	UVLO_TRIGGERED_BIT,
	OTST3_TRIGGERED_BIT,
	STAGE3_TRIGGERED_BIT,
	POFF_REASON2_BIT_MAX,
} poff_reason2_bit;

char *pon_reason[PON_BIT_MAX] = {
	"Hard Reset",
	"SMPL",
	"RTC",
	"DC Charger",
	"USB Charger",
	"Pon1",
	"CBL_PWR1",
	"Keypad Power"
  };

char *warm_reset_reason1[WARM_REASON1_BIT_MAX] = {
	"Software",
	"PS Hold",
	"PMIC Watchdog",
	"Keypad Reset1",
	"Keypad Reset2",
	"Kpdpwr + Resin",
	"Resin",
	"Keypad Power"
  };

char *warm_reset_reason2[WARM_REASON2_BIT_MAX] = {
	"AFP",
  };

char *poff_reason2[POFF_REASON2_BIT_MAX] = {
	"Charger",
	"AFP",
	"UVLO",
	"OTST3",
	"Stage3"
  };

void htc_print_reset_reason(int type, uint8_t value)
{
	int bit_idx = 0;
	int start_bit = 0;
	int end_bit = 0;
	char **reason_desc = NULL;

	switch (type) {
	case PON_PON_REASON1:
		start_bit = HARD_RESET_TRIGGERED_BIT;
		end_bit = PON_BIT_MAX;
		reason_desc = pon_reason;
		break;
	case PON_WARM_RESET_REASON1:
	case PON_SOFT_RESET_REASON1:
	case PON_POFF_REASON1:
		start_bit = SOFT_TRIGGERED_BIT;
		end_bit = WARM_REASON1_BIT_MAX;
		reason_desc = warm_reset_reason1;
		break;
	case PON_WARM_RESET_REASON2:
	case PON_SOFT_RESET_REASON2:
		start_bit = AFP_TRIGGERED_BIT;
		end_bit = WARM_REASON2_BIT_MAX;
		reason_desc = warm_reset_reason2;
		break;
	case PON_POFF_REASON2:
		start_bit = CHARGER_TRIGGERED_BIT;
		end_bit = POFF_REASON2_BIT_MAX;
		reason_desc = poff_reason2;
		break;
	default:
		break;
	}

	for (bit_idx = start_bit; bit_idx < end_bit; bit_idx++) {
		if (value & (1 << bit_idx)) {
			printk(KERN_INFO "%s, (0x%x)", reason_desc[bit_idx - start_bit], value);
		}
	}
}

int htc_spmi_read_data(struct spmi_controller *ctrl, uint8_t *buf, int offset, int cnt)
{
	int ret = 0;
	int len;
	uint8_t sid;
	uint16_t addr;

	while (cnt > 0) {
		sid = (offset >> 16) & 0xF;
		addr = offset & 0xFFFF;
		len = min(cnt, MAX_REG_PER_TRANSACTION);

		ret = spmi_ext_register_readl(ctrl, sid, addr, buf, len);
		if (ret < 0) {
			pr_err("SPMI read failed, err = %d\n", ret);
			goto done;
		}

		cnt -= len;
		buf += len;
		offset += len;
	}

done:
	return ret;
}

uint8_t reason_1 = 0xFF;
uint8_t warm_reset_reason_1 = 0xFF;
uint8_t warm_reset_reason_2 = 0xFF;
uint8_t soft_reset_reason_1 = 0xFF;
uint8_t soft_reset_reason_2 = 0xFF;
uint8_t poff_reason_1 = 0xFF;
uint8_t poff_reason_2 = 0xFF;

void htc_get_pon_boot_reason(struct spmi_controller *ctrl)
{
	
	if (reason_1 == 0xFF)
		htc_spmi_read_data(ctrl, &reason_1, PON_PON_REASON1, 1);

	
	if (warm_reset_reason_1 == 0xFF)
		htc_spmi_read_data(ctrl, &warm_reset_reason_1, PON_WARM_RESET_REASON1, 1);
	if (warm_reset_reason_2 == 0xFF)
		htc_spmi_read_data(ctrl, &warm_reset_reason_2, PON_WARM_RESET_REASON2, 1);

	
	if (soft_reset_reason_1 == 0xFF)
		htc_spmi_read_data(ctrl, &soft_reset_reason_1, PON_SOFT_RESET_REASON1, 1);

	if (soft_reset_reason_2 == 0xFF)
		htc_spmi_read_data(ctrl, &soft_reset_reason_2, PON_SOFT_RESET_REASON2, 1);

	
	if (poff_reason_1 == 0xFF)
		htc_spmi_read_data(ctrl, &poff_reason_1, PON_POFF_REASON1, 1);
	if (poff_reason_2 == 0xFF)
		htc_spmi_read_data(ctrl, &poff_reason_2, PON_POFF_REASON2, 1);
}

void htc_print_pon_boot_reason(void)
{
	printk(KERN_INFO "------ Reset Reason ------\n");
	printk(KERN_INFO "[PON_PON_REASON]");
	htc_print_reset_reason(PON_PON_REASON1, reason_1);

	
	printk(KERN_INFO "[PON_WARM_RESET_REASON]");
	htc_print_reset_reason(PON_WARM_RESET_REASON1, warm_reset_reason_1);
	htc_print_reset_reason(PON_WARM_RESET_REASON2, warm_reset_reason_2);

	printk(KERN_INFO "[PON_SOFT_RESET_REASON]");
	htc_print_reset_reason(PON_SOFT_RESET_REASON1, soft_reset_reason_1);

	htc_print_reset_reason(PON_SOFT_RESET_REASON2, soft_reset_reason_2);

	
	printk(KERN_INFO "[PON_POFF_REASON]");
	htc_print_reset_reason(PON_POFF_REASON1, poff_reason_1);
	htc_print_reset_reason(PON_POFF_REASON2, poff_reason_2);

	printk(KERN_INFO "-------------------------\n");
}

EXPORT_SYMBOL_GPL(htc_print_pon_boot_reason);
#endif
static int spmi_register_controller(struct spmi_controller *ctrl)
{
	int ret = 0;

	
	if (WARN_ON(!spmi_bus_type.p)) {
		ret = -EAGAIN;
		goto exit;
	}

	dev_set_name(&ctrl->dev, "spmi-%d", ctrl->nr);
	ctrl->dev.bus = &spmi_bus_type;
	ctrl->dev.type = &spmi_ctrl_type;
	ret = device_register(&ctrl->dev);
	if (ret)
		goto exit;

	dev_dbg(&ctrl->dev, "Bus spmi-%d registered: dev:0x%p\n",
					ctrl->nr, &ctrl->dev);

	
	if (get_tamper_sf() == 0 && board_is_super_cid())
		spmi_dfs_add_controller(ctrl);

#ifdef CONFIG_HTC_POWER_DEBUG
	
	htc_get_pon_boot_reason(ctrl);
	htc_print_pon_boot_reason();
#endif
	return 0;

exit:
	mutex_lock(&board_lock);
	idr_remove(&ctrl_idr, ctrl->nr);
	mutex_unlock(&board_lock);
	return ret;
}

static void __exit spmi_exit(void)
{
	device_unregister(&spmi_dev);
	bus_unregister(&spmi_bus_type);
}

static int __init spmi_init(void)
{
	int retval;

	retval = bus_register(&spmi_bus_type);
	if (!retval)
		retval = device_register(&spmi_dev);

	if (retval)
		bus_unregister(&spmi_bus_type);

	return retval;
}
postcore_initcall(spmi_init);
module_exit(spmi_exit);

MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
MODULE_DESCRIPTION("SPMI module");
MODULE_ALIAS("platform:spmi");
