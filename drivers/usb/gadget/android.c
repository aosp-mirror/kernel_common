/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *         Benoit Goby <benoit@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/of.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/usb/android.h>

#include <mach/diag_dload.h>
#include <mach/devices_cmdline.h>
#include <mach/board_htc.h>

#include "gadget_chips.h"
#ifdef CONFIG_HTC_USB_DEBUG_FLAG
#include "debug.h"
#endif


enum {
	OS_NOT_YET,
	OS_MAC,
	OS_LINUX,
	OS_WINDOWS,
};

static int mac_mtp_mode;
static int os_type;

int board_get_usb_ats(void);
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"

#include "f_diag.c"
#include "f_qdss.c"
#include "f_rmnet_smd.c"
#include "f_rmnet_sdio.c"
#include "f_rmnet_smd_sdio.c"
#include "f_rmnet.c"
#include "f_gps.c"
#ifdef CONFIG_SND_PCM
#include "f_audio_source.c"
#endif
#include "f_mass_storage.c"
#include "u_serial.c"
#include "u_sdio.c"
#include "u_smd.c"
#include "u_bam.c"
#include "u_rmnet_ctrl_smd.c"
#include "u_rmnet_ctrl_qti.c"
#include "u_ctrl_hsic.c"
#include "u_data_hsic.c"
#include "u_ctrl_hsuart.c"
#include "u_data_hsuart.c"
#include "f_serial.c"
#include "f_acm.c"
#include "f_adb.c"
#include "f_ccid.c"
#include "f_mtp.c"
#include "f_accessory.c"
#define USB_ETH_RNDIS y
#include "f_rndis.c"
#include "rndis.c"
#include "f_qc_ecm.c"
#include "f_mbim.c"
#include "u_bam_data.c"
#include "f_ecm.c"
#include "f_ncm.c"
#include "f_qc_rndis.c"
#include "u_ether.c"
#include "u_qc_ether.c"
#ifdef CONFIG_TARGET_CORE
#include "f_tcm.c"
#endif
#ifdef CONFIG_SND_PCM
#include "u_uac1.c"
#include "f_uac1.c"
#endif

#include <linux/usb/htc_info.h>
#include "f_projector.c"
#include "f_projector2.c"

#ifdef CONFIG_PERFLOCK
#include <mach/perflock.h>
#endif

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static DEFINE_MUTEX(function_bind_sem);

static const char longname[] = "Gadget Android";

#define VENDOR_ID		0x18D1
#define PRODUCT_ID		0x0001

static bool connect2pc;

#define ANDROID_DEVICE_NODE_NAME_LENGTH 11

struct android_usb_function {
	char *name;
	void *config;

	struct device *dev;
	char *dev_name;
	struct device_attribute **attributes;

	struct android_dev *android_dev;

	
#if 0
	struct list_head enabled_list;
#endif
	
	int (*init)(struct android_usb_function *, struct usb_composite_dev *);
	
	void (*cleanup)(struct android_usb_function *);
	void (*enable)(struct android_usb_function *);
	
	void (*disable)(struct android_usb_function *);

	int (*bind_config)(struct android_usb_function *,
			   struct usb_configuration *);

	
	void (*unbind_config)(struct android_usb_function *,
			      struct usb_configuration *);
	
	int (*ctrlrequest)(struct android_usb_function *,
					struct usb_composite_dev *,
					const struct usb_ctrlrequest *);
	
	int performance_lock;
};

struct android_usb_function_holder {

	struct android_usb_function *f;

	
	struct list_head enabled_list;
};

struct android_dev {
	const char *name;
	struct android_usb_function **functions;
	struct usb_composite_dev *cdev;
	struct device *dev;

	bool enabled;
	int disable_depth;
	struct mutex mutex;
	struct android_usb_platform_data *pdata;

	bool connected;
	bool sw_connected;
	bool suspended;
	bool sw_suspended;
	char pm_qos[5];
	struct pm_qos_request pm_qos_req_dma;
	struct work_struct work;

	
	struct list_head configs;
	int configs_num;

	
	struct list_head list_item;

	struct platform_device *pdev;
	struct android_usb_product *products;
	int num_products;
	int num_functions;

	int autobot_mode;
};

struct android_configuration {
	struct usb_configuration usb_config;

	
	struct list_head enabled_functions;

	
	struct list_head list_item;
};

struct dload_struct __iomem *diag_dload;
static struct class *android_class;
static struct android_dev *_android_dev;
static struct list_head android_dev_list;
static int android_dev_count;
static int android_bind_config(struct usb_configuration *c);
static void android_unbind_config(struct usb_configuration *c);
static struct android_dev *cdev_to_android_dev(struct usb_composite_dev *cdev);
static struct android_configuration *alloc_android_config
						(struct android_dev *dev);
static void free_android_config(struct android_dev *dev,
				struct android_configuration *conf);
static int usb_diag_update_pid_and_serial_num(uint32_t pid, const char *snum);
static int diag_update_supported = 0; 


#ifdef CONFIG_PERFLOCK
static struct perf_lock android_usb_perf_lock;
#endif


#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

static char manufacturer_string[256];
static char product_string[256];
static char serial_string[256];

static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = manufacturer_string,
	[STRING_PRODUCT_IDX].s = product_string,
	[STRING_SERIAL_IDX].s = serial_string,
	{  }			
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,
	.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
	.bcdOTG               = __constant_cpu_to_le16(0x0200),
};

static const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	NULL,
};

enum android_device_state {
	USB_DISCONNECTED,
	USB_CONNECTED,
	USB_CONFIGURED,
	USB_SUSPENDED,
	USB_RESUMED
};
#ifdef CONFIG_PERFLOCK
static unsigned int check_enable_function_performancelock(void)
{
	struct android_dev *dev = _android_dev;
	struct android_configuration *conf;
	struct android_usb_function_holder *f_holder;
	
	conf = list_entry(dev->configs.next, struct android_configuration, list_item);

	list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list) {
		if (f_holder->f->performance_lock == 1)
			return 1;
	}
	return 0;
}
#endif

static void android_pm_qos_update_latency(struct android_dev *dev, int vote)
{
	struct android_usb_platform_data *pdata = dev->pdata;
	u32 swfi_latency = 0;
	static int last_vote = -1;

	if (!pdata || vote == last_vote
		|| !pdata->swfi_latency)
		return;

	swfi_latency = pdata->swfi_latency + 1;
	if (vote)
		pm_qos_update_request(&dev->pm_qos_req_dma,
				swfi_latency);
	else
		pm_qos_update_request(&dev->pm_qos_req_dma,
				PM_QOS_DEFAULT_VALUE);
	last_vote = vote;
}


void android_broadcast_abnormal_usb_reset(void);

static void android_work(struct work_struct *data)
{
	struct android_dev *dev = container_of(data, struct android_dev, work);
	struct usb_composite_dev *cdev = dev->cdev;
	char *disconnected[2] = { "USB_STATE=DISCONNECTED", NULL };
	char *connected[2]    = { "USB_STATE=CONNECTED", NULL };
	char *configured[2]   = { "USB_STATE=CONFIGURED", NULL };
	char *suspended[2]   = { "USB_STATE=SUSPENDED", NULL };
	char *resumed[2]   = { "USB_STATE=RESUMED", NULL };
	char **uevent_envp = NULL;
	static enum android_device_state last_uevent, next_state;
	unsigned long flags;
#ifdef CONFIG_PERFLOCK
	static int perflock_state = 0;
#endif
	int pm_qos_vote = -1;

	printk(KERN_INFO "[USB] android_work : sw_suspended %d  suspended %d config %d,connect2pc %d",dev->sw_suspended,dev->suspended,cdev->config?1:0,connect2pc);
	printk(KERN_INFO "[USB] android_work : sw_connected %d  connected %d last_uevent %d",dev->sw_connected,dev->connected,last_uevent);
	spin_lock_irqsave(&cdev->lock, flags);
	if (dev->suspended != dev->sw_suspended && cdev->config) {
		if (strncmp(dev->pm_qos, "low", 3))
			pm_qos_vote = dev->suspended ? 0 : 1;
		next_state = dev->suspended ? USB_SUSPENDED : USB_RESUMED;
		uevent_envp = dev->suspended ? suspended : resumed;
	} else if (cdev->config) {
		uevent_envp = configured;
		next_state = USB_CONFIGURED;
#ifdef CONFIG_PERFLOCK
		perflock_state = check_enable_function_performancelock();
#endif
	} else if (dev->connected != dev->sw_connected) {
		uevent_envp = dev->connected ? connected : disconnected;
		next_state = dev->connected ? USB_CONNECTED : USB_DISCONNECTED;
		if (dev->connected && strncmp(dev->pm_qos, "low", 3))
			pm_qos_vote = 1;
		else if (!dev->connected || !strncmp(dev->pm_qos, "low", 3))
			pm_qos_vote = 0;
	}
	dev->sw_connected = dev->connected;
	dev->sw_suspended = dev->suspended;
	spin_unlock_irqrestore(&cdev->lock, flags);

	if (pm_qos_vote != -1)
		android_pm_qos_update_latency(dev, pm_qos_vote);


#ifdef CONFIG_PERFLOCK
	if (!cdev->config) {
		if (is_perf_lock_active(&android_usb_perf_lock)) {
			printk(KERN_INFO "[USB] Performance lock released\n");
			perf_unlock(&android_usb_perf_lock);
		}
	} else {
		if (perflock_state) {
			if (!is_perf_lock_active(&android_usb_perf_lock)) {
				printk(KERN_INFO "[USB] Performance lock requested\n");
				perf_lock(&android_usb_perf_lock);
			}
		} else {
			if (is_perf_lock_active(&android_usb_perf_lock)) {
				printk(KERN_INFO "[USB] Performance lock released\n");
				perf_unlock(&android_usb_perf_lock);
			}
		}
	}
#endif

	if (uevent_envp) {
		if (((uevent_envp == connected) &&
		      (last_uevent != USB_DISCONNECTED)) ||
		    ((uevent_envp == configured) &&
		      (last_uevent == USB_CONFIGURED))) {
			pr_info("%s: sent missed DISCONNECT event\n", __func__);
			kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE,
								disconnected);
			msleep(20);
			pr_info("%s: sent CONNECT event\n", __func__);
			kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE,
					connected);
			msleep(20);
		}
		if (uevent_envp == configured)
			msleep(50);

		
		if (next_state != USB_SUSPENDED && next_state != USB_RESUMED) {
			kobject_uevent_env(&dev->dev->kobj, KOBJ_CHANGE,
					   uevent_envp);
			last_uevent = next_state;
		}
		pr_info("%s: sent uevent %s\n", __func__, uevent_envp[0]);
		pr_info("%s: skip trigger unmount uevent\n", __func__);
	} else {
	}

	if (connect2pc != dev->sw_connected) {
		connect2pc = dev->sw_connected;
		switch_set_state(&cdev->sw_connect2pc, connect2pc ? 1 : 0);
		pr_info("set usb_connect2pc = %d\n", connect2pc);
		if (!connect2pc) {
			pr_info("%s: OS_NOT_YET\n", __func__);
			os_type = OS_NOT_YET;
			mtp_update_mode(0);
			fsg_update_mode(0);
		}
	}

	if (dev->connected == 0 && check_htc_mode_status() != NOT_ON_AUTOBOT) {
		htc_mode_enable(0);
		android_switch_default();
		setup_usb_denied(0);
	}


}

static int android_enable(struct android_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct android_configuration *conf;
	int err = 0;

	printk(KERN_INFO "[USB]%s disable_depth %d\n",__func__,dev->disable_depth);
	if (WARN_ON(!dev->disable_depth))
		return err;

	if (--dev->disable_depth == 0) {

		list_for_each_entry(conf, &dev->configs, list_item) {
			err = usb_add_config(cdev, &conf->usb_config,
						android_bind_config);
			if (err < 0) {
				pr_err("%s: usb_add_config failed : err: %d\n",
						__func__, err);
				return err;
			}
		}
#ifdef CONFIG_HTC_USB_DEBUG_FLAG
		dbg_event(0xFF,"ANDENA",0);
#endif
		usb_gadget_connect(cdev->gadget);
#ifdef CONFIG_HTC_USB_DEBUG_FLAG
		dbg_event(0xFF,"CONNEND",0);
#endif
	}

	return err;
}

static void android_disable(struct android_dev *dev)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct android_configuration *conf;

	printk(KERN_INFO "[USB]%s disable_depth %d\n",__func__,dev->disable_depth);
	if (dev->disable_depth++ == 0) {
#ifdef CONFIG_HTC_USB_DEBUG_FLAG
		dbg_event(0xFF,"ANDDIS",0);
#endif
		usb_gadget_disconnect(cdev->gadget);
#ifdef CONFIG_HTC_USB_DEBUG_FLAG
		dbg_event(0xFF,"DISCEND",0);
#endif
		
		usb_ep_dequeue(cdev->gadget->ep0, cdev->req);

		list_for_each_entry(conf, &dev->configs, list_item)
			usb_remove_config(cdev, &conf->usb_config);
	}
}


static ssize_t func_en_show(struct device *pdev, struct device_attribute *attr,
		char *buf)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function *func = dev_get_drvdata(pdev);
	struct android_usb_function_holder *f_holder;
	struct android_configuration *conf;
	int ebl = 0;

	mutex_lock(&dev->mutex);
	list_for_each_entry(conf, &dev->configs, list_item)
		list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list) {
			if (!strcmp(func->name, f_holder->f->name)) {
				ebl = 1;
				break;
			}
		}
	mutex_unlock(&dev->mutex);
	return sprintf(buf, "%d", ebl);
}

static ssize_t func_en_store(
		struct device *pdev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function *func = dev_get_drvdata(pdev);

	struct android_usb_function_holder *f_holder;
	struct android_configuration *conf;
	int ebl = 0;
	int value;
	int acm_off = 0;

	sscanf(buf, "%d", &value);
	mutex_lock(&function_bind_sem);
	list_for_each_entry(conf, &dev->configs, list_item)
		list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list) {
			if (!strcmp(func->name, f_holder->f->name)) {
				ebl = 1;
				break;
			}
		}

	list_for_each_entry(conf, &dev->configs, list_item)
		list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list) {
			if (!strcmp(func->name, "modem")||!strcmp(func->name, "diag")||!strcmp(func->name, "diag_mdm")
				||!strcmp(func->name, "modem_mdm")||!strcmp(func->name, "serial")) {
				acm_off = 1;
				break;
			}
		}

	mutex_unlock(&function_bind_sem);
	if (!!value == ebl) {
		pr_info("%s function is already %s\n", func->name
			, ebl ? "enable" : "disable");
		return size;
	}
	if (acm_off)
		htc_usb_enable_function("acm", 0);

	if (value)
		htc_usb_enable_function(func->name, 1);
	else
		htc_usb_enable_function(func->name, 0);
	return size;
}
static DEVICE_ATTR(on, S_IRUGO | S_IWUSR | S_IWGRP, func_en_show, func_en_store);

struct adb_data {
	bool opened;
	bool enabled;
	struct android_dev *dev;
};

static int
adb_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct adb_data), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return adb_setup();
}

static void adb_function_cleanup(struct android_usb_function *f)
{
	adb_cleanup();
	kfree(f->config);
}

static int
adb_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return adb_bind_config(c);
}

static void adb_android_function_enable(struct android_usb_function *f)
{
#if 0
	struct android_dev *dev = f->android_dev;
	struct adb_data *data = f->config;

	data->enabled = true;


	
	if (!data->opened)
		android_disable(dev);
#endif
}

static void adb_android_function_disable(struct android_usb_function *f)
{
#if 0
	struct android_dev *dev = f->android_dev;
	struct adb_data *data = f->config;

	data->enabled = false;

	
	if (!data->opened)
		android_enable(dev);
#endif
}

static struct android_usb_function adb_function = {
	.name		= "adb",
	.enable		= adb_android_function_enable,
	.disable	= adb_android_function_disable,
	.init		= adb_function_init,
	.cleanup	= adb_function_cleanup,
	.bind_config	= adb_function_bind_config,
};

#if 0
static void adb_ready_callback(void)
{
	struct android_dev *dev = adb_function.android_dev;
	struct adb_data *data = adb_function.config;

	
	if (dev)
		mutex_lock(&dev->mutex);

	
	data->dev = dev;
	data->opened = true;

	if (data->enabled && dev)
		android_enable(dev);

	if (dev)
		mutex_unlock(&dev->mutex);
}

static void adb_closed_callback(void)
{
	struct adb_data *data = adb_function.config;
	struct android_dev *dev = adb_function.android_dev;

	
	if (!dev)
		dev = data->dev;

	if (!dev)
		pr_err("adb_closed_callback: data->dev is NULL");

	if (dev)
		mutex_lock(&dev->mutex);

	data->opened = false;

	if (data->enabled && dev)
		android_disable(dev);

	data->dev = NULL;

	if (dev)
		mutex_unlock(&dev->mutex);
}
#endif

static void adb_read_timeout(void)
{
	pr_info("%s: adb read timeout, re-connect to PC\n",__func__);

	android_force_reset();
}


static int rmnet_smd_function_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_smd_bind_config(c);
}

static struct android_usb_function rmnet_smd_function = {
	.name		= "rmnet_smd",
	.bind_config	= rmnet_smd_function_bind_config,
};

static int rmnet_sdio_function_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_sdio_function_add(c);
}

static struct android_usb_function rmnet_sdio_function = {
	.name		= "rmnet_sdio",
	.bind_config	= rmnet_sdio_function_bind_config,
	.performance_lock = 1,
};

static int rmnet_smd_sdio_function_init(struct android_usb_function *f,
				 struct usb_composite_dev *cdev)
{
	return rmnet_smd_sdio_init();
}

static void rmnet_smd_sdio_function_cleanup(struct android_usb_function *f)
{
	rmnet_smd_sdio_cleanup();
}

static int rmnet_smd_sdio_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return rmnet_smd_sdio_function_add(c);
}

static struct device_attribute *rmnet_smd_sdio_attributes[] = {
					&dev_attr_transport, NULL };

static struct android_usb_function rmnet_smd_sdio_function = {
	.name		= "rmnet_smd_sdio",
	.init		= rmnet_smd_sdio_function_init,
	.cleanup	= rmnet_smd_sdio_function_cleanup,
	.bind_config	= rmnet_smd_sdio_bind_config,
	.attributes	= rmnet_smd_sdio_attributes,
	.performance_lock = 1,
};

#define MAX_XPORT_STR_LEN 50
static char rmnet_transports[MAX_XPORT_STR_LEN];

static char rmnet_xport_names[MAX_XPORT_STR_LEN];

static void rmnet_function_cleanup(struct android_usb_function *f)
{
	frmnet_cleanup();
}

static int rmnet_function_bind_config(struct android_usb_function *f,
					 struct usb_configuration *c)
{
	int i;
	int err = 0;
	char *ctrl_name;
	char *data_name;
	char *tname = NULL;
	char buf[MAX_XPORT_STR_LEN], *b;
	char xport_name_buf[MAX_XPORT_STR_LEN], *tb;
	static int rmnet_initialized, ports;

	if (!rmnet_initialized) {
		rmnet_initialized = 1;
		strlcpy(buf, rmnet_transports, sizeof(buf));
		b = strim(buf);

		strlcpy(xport_name_buf, rmnet_xport_names,
				sizeof(xport_name_buf));
		tb = strim(xport_name_buf);

		while (b) {
			ctrl_name = strsep(&b, ",");
			data_name = strsep(&b, ",");
			if (ctrl_name && data_name) {
				if (tb)
					tname = strsep(&tb, ",");
				err = frmnet_init_port(ctrl_name, data_name,
						tname);
				if (err) {
					pr_err("rmnet: Cannot open ctrl port:"
						"'%s' data port:'%s'\n",
						ctrl_name, data_name);
					goto out;
				}
				ports++;
			}
		}

		err = rmnet_gport_setup();
		if (err) {
			pr_err("rmnet: Cannot setup transports");
			goto out;
		}
	}

	for (i = 0; i < ports; i++) {
		err = frmnet_bind_config(c, i);
		if (err) {
			pr_err("Could not bind rmnet%u config\n", i);
			break;
		}
	}
out:
	return err;
}

static ssize_t rmnet_transports_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", rmnet_transports);
}

static ssize_t rmnet_transports_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(rmnet_transports, buff, sizeof(rmnet_transports));

	return size;
}

static ssize_t rmnet_xport_names_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", rmnet_xport_names);
}

static ssize_t rmnet_xport_names_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(rmnet_xport_names, buff, sizeof(rmnet_xport_names));

	return size;
}

static struct device_attribute dev_attr_rmnet_transports =
					__ATTR(transports, S_IRUGO | S_IWUSR,
						rmnet_transports_show,
						rmnet_transports_store);

static struct device_attribute dev_attr_rmnet_xport_names =
				__ATTR(transport_names, S_IRUGO | S_IWUSR,
				rmnet_xport_names_show,
				rmnet_xport_names_store);

static struct device_attribute *rmnet_function_attributes[] = {
					&dev_attr_rmnet_transports,
					&dev_attr_rmnet_xport_names,
					NULL };

static struct android_usb_function rmnet_function = {
	.name		= "rmnet",
	.cleanup	= rmnet_function_cleanup,
	.bind_config	= rmnet_function_bind_config,
	.attributes	= rmnet_function_attributes,
	.performance_lock = 1,
};

static void gps_function_cleanup(struct android_usb_function *f)
{
	gps_cleanup();
}

static int gps_function_bind_config(struct android_usb_function *f,
					 struct usb_configuration *c)
{
	int err;
	static int gps_initialized;

	if (!gps_initialized) {
		gps_initialized = 1;
		err = gps_init_port();
		if (err) {
			pr_err("gps: Cannot init gps port");
			return err;
		}
	}

	err = gps_gport_setup();
	if (err) {
		pr_err("gps: Cannot setup transports");
		return err;
	}
	err = gps_bind_config(c);
	if (err) {
		pr_err("Could not bind gps config\n");
		return err;
	}

	return 0;
}

static struct android_usb_function gps_function = {
	.name		= "gps",
	.cleanup	= gps_function_cleanup,
	.bind_config	= gps_function_bind_config,
};

static char ecm_transports[MAX_XPORT_STR_LEN];

struct ecm_function_config {
	u8      ethaddr[ETH_ALEN];
};

static int ecm_function_init(struct android_usb_function *f,
				struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct ecm_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;
	return 0;
}

static void ecm_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int ecm_qc_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int ret;
	char *trans;
	struct ecm_function_config *ecm = f->config;

	if (!ecm) {
		pr_err("%s: ecm_pdata\n", __func__);
		return -EINVAL;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		ecm->ethaddr[0], ecm->ethaddr[1], ecm->ethaddr[2],
		ecm->ethaddr[3], ecm->ethaddr[4], ecm->ethaddr[5]);

	pr_debug("%s: ecm_transport is %s", __func__, ecm_transports);

	trans = strim(ecm_transports);
	if (strcmp("BAM2BAM_IPA", trans)) {
		ret = gether_qc_setup_name(c->cdev->gadget,
						ecm->ethaddr, "ecm");
		if (ret) {
			pr_err("%s: gether_setup failed\n", __func__);
			return ret;
		}
	}

	return ecm_qc_bind_config(c, ecm->ethaddr, trans);
}

static void ecm_qc_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	char *trans = strim(ecm_transports);

	if (strcmp("BAM2BAM_IPA", trans))
		gether_qc_cleanup_name("ecm0");
}

static ssize_t ecm_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ecm_function_config *ecm = f->config;
	return snprintf(buf, PAGE_SIZE, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		ecm->ethaddr[0], ecm->ethaddr[1], ecm->ethaddr[2],
		ecm->ethaddr[3], ecm->ethaddr[4], ecm->ethaddr[5]);
}

static ssize_t ecm_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ecm_function_config *ecm = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&ecm->ethaddr[0], (int *)&ecm->ethaddr[1],
		    (int *)&ecm->ethaddr[2], (int *)&ecm->ethaddr[3],
		    (int *)&ecm->ethaddr[4], (int *)&ecm->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ecm_ethaddr, S_IRUGO | S_IWUSR, ecm_ethaddr_show,
					       ecm_ethaddr_store);

static ssize_t ecm_transports_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", ecm_transports);
}

static ssize_t ecm_transports_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	strlcpy(ecm_transports, buf, sizeof(ecm_transports));
	return size;
}

static DEVICE_ATTR(ecm_transports, S_IRUGO | S_IWUSR, ecm_transports_show,
					       ecm_transports_store);

static struct device_attribute *ecm_function_attributes[] = {
	&dev_attr_ecm_transports,
	&dev_attr_ecm_ethaddr,
	NULL
};

static struct android_usb_function ecm_qc_function = {
	.name		= "ecm_qc",
	.init		= ecm_function_init,
	.cleanup	= ecm_function_cleanup,
	.bind_config	= ecm_qc_function_bind_config,
	.unbind_config	= ecm_qc_function_unbind_config,
	.attributes	= ecm_function_attributes,
};


struct ncm_function_config {
	u8      ethaddr[ETH_ALEN];
};

static int ncm_function_init(struct android_usb_function *f, struct usb_composite_dev *c)
{
	struct ncm_function_config *ncm;

	f->config = kzalloc(sizeof(struct ncm_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	ncm = f->config;
	return 0;
}

static void ncm_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int ncm_function_bind_config(struct android_usb_function *f,
				struct usb_configuration *c)
{
	int ret;
	struct ncm_function_config *ncm = f->config;

	if (!ncm) {
		pr_err("%s: ncm config is null\n", __func__);
		return -EINVAL;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		ncm->ethaddr[0], ncm->ethaddr[1], ncm->ethaddr[2],
		ncm->ethaddr[3], ncm->ethaddr[4], ncm->ethaddr[5]);

    if (c->cdev->gadget)
        c->cdev->gadget->miMaxMtu = ETH_FRAME_LEN_MAX - ETH_HLEN;
	ret = gether_setup_name(c->cdev->gadget, ncm->ethaddr, "ncm");
	if (ret) {
		pr_err("%s: gether setup failed err:%d\n", __func__, ret);
		return ret;
	}

	ret = ncm_bind_config(c, ncm->ethaddr);
	if (ret) {
		pr_err("%s: ncm bind config failed err:%d", __func__, ret);
		gether_cleanup();
		return ret;
	}

	return ret;
}

static void ncm_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	if (c->cdev->gadget)
		c->cdev->gadget->miMaxMtu = 0;
	gether_cleanup();
}

static ssize_t ncm_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ncm_function_config *ncm = f->config;
	return snprintf(buf, PAGE_SIZE, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		ncm->ethaddr[0], ncm->ethaddr[1], ncm->ethaddr[2],
		ncm->ethaddr[3], ncm->ethaddr[4], ncm->ethaddr[5]);
}

static ssize_t ncm_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct ncm_function_config *ncm = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&ncm->ethaddr[0], (int *)&ncm->ethaddr[1],
		    (int *)&ncm->ethaddr[2], (int *)&ncm->ethaddr[3],
		    (int *)&ncm->ethaddr[4], (int *)&ncm->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ncm_ethaddr, S_IRUGO | S_IWUSR, ncm_ethaddr_show,
					       ncm_ethaddr_store);
static struct device_attribute *ncm_function_attributes[] = {
	&dev_attr_ncm_ethaddr,
	NULL
};

static struct android_usb_function ncm_function = {
	.name		= "cdc_network",
	.init		= ncm_function_init,
	.cleanup	= ncm_function_cleanup,
	.bind_config	= ncm_function_bind_config,
	.unbind_config	= ncm_function_unbind_config,
	.attributes	= ncm_function_attributes,
	.performance_lock = 1,
};

#define MAX_MBIM_INSTANCES 1

static int mbim_function_init(struct android_usb_function *f,
					 struct usb_composite_dev *cdev)
{
	return mbim_init(MAX_MBIM_INSTANCES);
}

static void mbim_function_cleanup(struct android_usb_function *f)
{
	fmbim_cleanup();
}


static char mbim_transports[MAX_XPORT_STR_LEN];

static int mbim_function_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	char *trans;

	pr_debug("%s: mbim transport is %s", __func__, mbim_transports);
	trans = strim(mbim_transports);
	return mbim_bind_config(c, 0, trans);
}

static int mbim_function_ctrlrequest(struct android_usb_function *f,
					struct usb_composite_dev *cdev,
					const struct usb_ctrlrequest *c)
{
	return mbim_ctrlrequest(cdev, c);
}

static ssize_t mbim_transports_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", mbim_transports);
}

static ssize_t mbim_transports_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	strlcpy(mbim_transports, buf, sizeof(mbim_transports));
	return size;
}

static DEVICE_ATTR(mbim_transports, S_IRUGO | S_IWUSR, mbim_transports_show,
				   mbim_transports_store);

static struct device_attribute *mbim_function_attributes[] = {
	&dev_attr_mbim_transports,
	NULL
};

static struct android_usb_function mbim_function = {
	.name		= "usb_mbim",
	.cleanup	= mbim_function_cleanup,
	.bind_config	= mbim_function_bind_config,
	.init		= mbim_function_init,
	.ctrlrequest	= mbim_function_ctrlrequest,
	.attributes		= mbim_function_attributes,
};

#ifdef CONFIG_SND_PCM
static int audio_function_bind_config(struct android_usb_function *f,
					  struct usb_configuration *c)
{
	return audio_bind_config(c);
}

static struct android_usb_function audio_function = {
	.name		= "audio",
	.bind_config	= audio_function_bind_config,
};
#endif


static char diag_clients[32];	    
static ssize_t clients_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(diag_clients, buff, sizeof(diag_clients));

	return size;
}

static DEVICE_ATTR(clients, S_IWUSR, NULL, clients_store);
static struct device_attribute *diag_function_attributes[] =
					 { &dev_attr_clients, NULL };

static int diag_function_init(struct android_usb_function *f,
				 struct usb_composite_dev *cdev)
{
	return diag_setup();
}

static void diag_function_cleanup(struct android_usb_function *f)
{
	diag_cleanup();
}

static int diag_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	char *name;
	char buf[32], *b;
	int once = 0, err = -1;
	int (*notify)(uint32_t, const char *);
	struct android_dev *dev = cdev_to_android_dev(c->cdev);

	strlcpy(buf, diag_clients, sizeof(buf));
	b = strim(buf);

	while (b) {
		notify = NULL;
		name = strsep(&b, ",");
		
		if (!once++) {
			if (dev->pdata && dev->pdata->update_pid_and_serial_num)
				notify = dev->pdata->update_pid_and_serial_num;
			else
				notify = usb_diag_update_pid_and_serial_num;
		}

		if (name) {
			err = diag_function_add(c, name, notify);
			if (err)
				pr_err("diag: Cannot open channel '%s'", name);
		}
	}

	return err;
}

static struct android_usb_function diag_function = {
	.name		= "diag",
	.init		= diag_function_init,
	.cleanup	= diag_function_cleanup,
	.bind_config	= diag_function_bind_config,
	.attributes	= diag_function_attributes,
};

static int qdss_function_init(struct android_usb_function *f,
	struct usb_composite_dev *cdev)
{
	return qdss_setup();
}

static void qdss_function_cleanup(struct android_usb_function *f)
{
	qdss_cleanup();
}

static int qdss_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int  err = -1;

	err = qdss_bind_config(c, "qdss");
	if (err)
		pr_err("qdss: Cannot open channel qdss");

	return err;
}

static struct android_usb_function qdss_function = {
	.name		= "qdss",
	.init		= qdss_function_init,
	.cleanup	= qdss_function_cleanup,
	.bind_config	= qdss_function_bind_config,
};

#define MAX_TRANSPORT_STRING 128
static char serial_transports[MAX_TRANSPORT_STRING];	
static int serial_nports;

#if 0
static ssize_t serial_transports_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(serial_transports, buff, sizeof(serial_transports));

	return size;
}

static DEVICE_ATTR(transports, S_IWUSR, NULL, serial_transports_store);
static struct device_attribute *serial_function_attributes[] =
					 { &dev_attr_transports, NULL };
#endif
static int serial_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	char *name, *str[2];
	char buf[MAX_TRANSPORT_STRING], *b;
	int err = -1;
	struct android_dev *dev = cdev_to_android_dev(cdev);

	if (dev->pdata->fserial_init_string)
		strcpy(serial_transports, dev->pdata->fserial_init_string);
	else
		strcpy(serial_transports, "smd:modem,tty,tty:autobot,tty:serial,tty:autobot");

	strncpy(buf, serial_transports, sizeof(buf));
	buf[MAX_TRANSPORT_STRING - 1] = 0;
	pr_info("%s: init string: %s\n", __func__, buf);

	b = strim(buf);

	while (b) {
		str[0] = str[1] = 0;
		name = strsep(&b, ",");
		if (name) {
			str[0] = strsep(&name, ":");
			if (str[0])
				str[1] = strsep(&name, ":");
		}
		err = gserial_init_port(serial_nports, str[0], str[1]);
		if (err) {
			pr_err("serial: Cannot open port '%s'\n", str[0]);
			goto out;
		}
		serial_nports++;
	}

	err = gport_setup(cdev);
	if (err) {
		pr_err("serial: Cannot setup transports");
		goto out;
	}
	return 0;
out:
	return err;
}


static void serial_function_cleanup(struct android_usb_function *f)
{
	gserial_cleanup();
}

static int serial_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int err = -1;
	int i, ports, car_mode = _android_dev->autobot_mode;

	ports = serial_nports;
	if (ports < 0)
		goto out;
	for (i = 0; i < ports; i++) {
		if ((gserial_ports[i].func_type == USB_FSER_FUNC_SERIAL) ||
			(car_mode && gserial_ports[i].func_type == USB_FSER_FUNC_AUTOBOT)) {
		err = gser_bind_config(c, i);
			if (err) {
			pr_err("serial: bind_config failed for port %d", i);
				goto out;
			}
		}
	}
out:
	return err;
}

static struct android_usb_function serial_function = {
	.name		= "serial",
	.cleanup	= serial_function_cleanup,
	.bind_config	= serial_function_bind_config,
	.init		= serial_function_init,
};

static int modem_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int err = -1;
	int i, ports;

	ports = serial_nports;
	if (ports < 0)
		goto out;


	for (i = 0; i < ports; i++) {
		if (gserial_ports[i].func_type == USB_FSER_FUNC_MODEM) {
			err = gser_bind_config(c, i);
			if (err) {
				pr_err("serial: bind_config failed for port %d", i);
				goto out;
			}
		}
	}

out:
	return err;
}

static struct android_usb_function modem_function = {
	.name		= "modem",
	.cleanup	= serial_function_cleanup,
	.bind_config	= modem_function_bind_config,
	.performance_lock = 1,
};

static char acm_transports[32];	
static ssize_t acm_transports_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	strlcpy(acm_transports, buff, sizeof(acm_transports));

	return size;
}

static ssize_t acm_baud_rate_store(
		struct device *device, struct device_attribute *attr,
		const char *buff, size_t size)
{
	unsigned u, port_num;
	ssize_t ret;
	struct tty_struct *port_tty;

	ret = strict_strtoul(buff, 10, (unsigned long *)&u);
	if (ret < 0) {
		USB_ERR("%s: %d\n", __func__, ret);
		return size;
	}

	if (_f_acm == NULL) {
		USB_WARNING("%s: acm didnt init\n", __func__);
		return size;
	}

	port_num = gserial_ports[_f_acm->port_num].client_port_num;;

	if (ports[port_num].port && ports[port_num].port->port_tty) {
		port_tty = ports[port_num].port->port_tty;
		USB_INFO("%s: %s%d set baudrate as %d\n", __func__, PREFIX, port_num, u);
		tty_termios_encode_baud_rate(port_tty->termios, u, u);
	} else {
		USB_ERR("%s: tty bridge was not existed\n", __func__);
	}

	return size;
}

static ssize_t acm_baud_rate_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	speed_t o_baud, i_baud;
	ssize_t size = 0;
	unsigned port_num;
	struct tty_struct *port_tty = NULL;

	if (_f_acm == NULL) {
		USB_WARNING("%s: acm didnt init\n", __func__);
		return size;
	}

	port_num = gserial_ports[_f_acm->port_num].client_port_num;;

	if (ports[port_num].port && ports[port_num].port->port_tty) {
		port_tty = ports[port_num].port->port_tty;
		o_baud = tty_termios_baud_rate(port_tty->termios);
		i_baud = tty_termios_input_baud_rate(port_tty->termios);

		size += sprintf(buf + size, "%s%d: %u/%u\n",
				PREFIX, port_num, o_baud, i_baud);
	} else {
		size += sprintf(buf + size, "%s%d: tty not found\n", PREFIX, port_num);
	}

	return size;
}

static DEVICE_ATTR(acm_transports, S_IWUSR, NULL, acm_transports_store);
static DEVICE_ATTR(baud_rate, S_IRUGO | S_IWUSR | S_IWGRP, acm_baud_rate_show, acm_baud_rate_store);
static struct device_attribute *acm_function_attributes[] = {
		&dev_attr_acm_transports,
		&dev_attr_baud_rate,
		NULL };

static void acm_function_cleanup(struct android_usb_function *f)
{
	gserial_cleanup();
}

static int acm_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int err = -1;
	int i, ports;

	ports = serial_nports; 
	if (ports < 0)
		goto out;
	for (i = 0; i < ports; i++) {
		if (gserial_ports[i].func_type == USB_FSER_FUNC_ACM) {
			err = acm_bind_config(c, i);
			if (err) {
				pr_err("acm: bind_config failed for port %d", i);
				goto out;
			}
		}
	}
out:
	return err;
}
static struct android_usb_function acm_function = {
	.name		= "acm",
	.cleanup	= acm_function_cleanup,
	.bind_config	= acm_function_bind_config,
	.attributes	= acm_function_attributes,
};

static int ccid_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return ccid_setup();
}

static void ccid_function_cleanup(struct android_usb_function *f)
{
	ccid_cleanup();
}

static int ccid_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return ccid_bind_config(c);
}

static struct android_usb_function ccid_function = {
	.name		= "ccid",
	.init		= ccid_function_init,
	.cleanup	= ccid_function_cleanup,
	.bind_config	= ccid_function_bind_config,
};

static int
mtp_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	return mtp_setup();
}

static void mtp_function_cleanup(struct android_usb_function *f)
{
	mtp_cleanup();
}

static int
mtp_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return mtp_bind_config(c, false);
}

static int
ptp_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	
	return 0;
}

static void ptp_function_cleanup(struct android_usb_function *f)
{
	
}

static int
ptp_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return mtp_bind_config(c, true);
}

static int mtp_function_ctrlrequest(struct android_usb_function *f,
					struct usb_composite_dev *cdev,
					const struct usb_ctrlrequest *c)
{
	return mtp_ctrlrequest(cdev, c);
}

static ssize_t mtp_debug_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", htc_mtp_performance_debug);
}

static ssize_t mtp_debug_level_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		htc_mtp_performance_debug = buf[0] - '0';
	return size;
}

static ssize_t mtp_iobusy_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	
	return sprintf(buf, "%d\n", _mtp_dev->mtp_perf_lock_on?1:0);
}
static ssize_t mtp_open_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", htc_mtp_open_state);
}

static DEVICE_ATTR(mtp_debug_level, S_IRUGO | S_IWUSR, mtp_debug_level_show,
						    mtp_debug_level_store);
static DEVICE_ATTR(iobusy, S_IRUGO, mtp_iobusy_show, NULL);
static DEVICE_ATTR(mtp_open_state, S_IRUGO, mtp_open_state_show, NULL);
static struct device_attribute *mtp_function_attributes[] = {
	&dev_attr_mtp_debug_level,
	&dev_attr_iobusy,
	&dev_attr_mtp_open_state,
	NULL
};

static struct android_usb_function mtp_function = {
	.name		= "mtp",
	.init		= mtp_function_init,
	.cleanup	= mtp_function_cleanup,
	.bind_config	= mtp_function_bind_config,
	.ctrlrequest	= mtp_function_ctrlrequest,
	.attributes 	= mtp_function_attributes,
};

static struct android_usb_function ptp_function = {
	.name		= "ptp",
	.init		= ptp_function_init,
	.cleanup	= ptp_function_cleanup,
	.bind_config	= ptp_function_bind_config,
};


struct rndis_function_config {
	u8      ethaddr[ETH_ALEN];
	u32     vendorID;
	u8      max_pkt_per_xfer;
	char	manufacturer[256];
	
	bool	wceis;
};

static int
rndis_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct rndis_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;
	return 0;
}

static void rndis_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int rndis_qc_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct rndis_function_config), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return rndis_qc_init();
}

static void rndis_qc_function_cleanup(struct android_usb_function *f)
{
	rndis_qc_cleanup();
	kfree(f->config);
}

static int
rndis_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	int ret;
	struct rndis_function_config *rndis = f->config;

	if (!rndis) {
		pr_err("%s: rndis_pdata\n", __func__);
		return -1;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);

	if (rndis->ethaddr[0])
		ret = gether_setup_name(c->cdev->gadget, NULL, "usb");
	else
		ret = gether_setup_name(c->cdev->gadget, rndis->ethaddr,
								"usb");
	if (ret) {
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}
	rndis->wceis = false;
	if (rndis->wceis) {
		
		rndis_iad_descriptor.bFunctionClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_iad_descriptor.bFunctionSubClass = 0x01;
		rndis_iad_descriptor.bFunctionProtocol = 0x03;
		rndis_control_intf.bInterfaceClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_control_intf.bInterfaceSubClass =	 0x01;
		rndis_control_intf.bInterfaceProtocol =	 0x03;
	}

	return rndis_bind_config_vendor(c, rndis->ethaddr, rndis->vendorID,
					   rndis->manufacturer);
}

static int rndis_qc_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int ret;
	struct rndis_function_config *rndis = f->config;

	if (!rndis) {
		pr_err("%s: rndis_pdata\n", __func__);
		return -EINVAL;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);

	ret = gether_qc_setup_name(c->cdev->gadget, rndis->ethaddr, "rndis");
	if (ret) {
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}

	rndis->wceis = false;
	if (rndis->wceis) {
		
		rndis_qc_iad_descriptor.bFunctionClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_qc_iad_descriptor.bFunctionSubClass = 0x01;
		rndis_qc_iad_descriptor.bFunctionProtocol = 0x03;
		rndis_qc_control_intf.bInterfaceClass =
						USB_CLASS_WIRELESS_CONTROLLER;
		rndis_qc_control_intf.bInterfaceSubClass =	 0x01;
		rndis_qc_control_intf.bInterfaceProtocol =	 0x03;
	}

	return rndis_qc_bind_config_vendor(c, rndis->ethaddr, rndis->vendorID,
				    rndis->manufacturer,
					rndis->max_pkt_per_xfer);
}

static void rndis_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	gether_cleanup();
}

static void rndis_qc_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	gether_qc_cleanup_name("rndis0");
}

static ssize_t rndis_manufacturer_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;

	return snprintf(buf, PAGE_SIZE, "%s\n", config->manufacturer);
}

static ssize_t rndis_manufacturer_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;

	if (size >= sizeof(config->manufacturer))
		return -EINVAL;

	if (sscanf(buf, "%255s", config->manufacturer) == 1)
		return size;
	return -1;
}

static DEVICE_ATTR(manufacturer, S_IRUGO | S_IWUSR, rndis_manufacturer_show,
						    rndis_manufacturer_store);

static ssize_t rndis_wceis_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;

	return snprintf(buf, PAGE_SIZE, "%d\n", config->wceis);
}

static ssize_t rndis_wceis_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%d", &value) == 1) {
		config->wceis = value;
		printk("[USB] %s %d\n",__func__,value);
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(wceis, S_IRUGO | S_IWUSR, rndis_wceis_show,
					     rndis_wceis_store);

static ssize_t rndis_ethaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;

	return snprintf(buf, PAGE_SIZE, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		rndis->ethaddr[0], rndis->ethaddr[1], rndis->ethaddr[2],
		rndis->ethaddr[3], rndis->ethaddr[4], rndis->ethaddr[5]);
}

static ssize_t rndis_ethaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *rndis = f->config;

	if (sscanf(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
		    (int *)&rndis->ethaddr[0], (int *)&rndis->ethaddr[1],
		    (int *)&rndis->ethaddr[2], (int *)&rndis->ethaddr[3],
		    (int *)&rndis->ethaddr[4], (int *)&rndis->ethaddr[5]) == 6)
		return size;
	return -EINVAL;
}

static DEVICE_ATTR(ethaddr, S_IRUGO | S_IWUSR, rndis_ethaddr_show,
					       rndis_ethaddr_store);

static ssize_t rndis_vendorID_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;

	return snprintf(buf, PAGE_SIZE, "%04x\n", config->vendorID);
}

static ssize_t rndis_vendorID_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%04x", &value) == 1) {
		config->vendorID = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(vendorID, S_IRUGO | S_IWUSR, rndis_vendorID_show,
						rndis_vendorID_store);

static ssize_t rndis_max_pkt_per_xfer_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->max_pkt_per_xfer);
}

static ssize_t rndis_max_pkt_per_xfer_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct rndis_function_config *config = f->config;
	int value;

	if (sscanf(buf, "%d", &value) == 1) {
		config->max_pkt_per_xfer = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(max_pkt_per_xfer, S_IRUGO | S_IWUSR,
				   rndis_max_pkt_per_xfer_show,
				   rndis_max_pkt_per_xfer_store);

static struct device_attribute *rndis_function_attributes[] = {
	&dev_attr_manufacturer,
	&dev_attr_wceis,
	&dev_attr_ethaddr,
	&dev_attr_vendorID,
	&dev_attr_max_pkt_per_xfer,
	NULL
};

static struct android_usb_function rndis_function = {
	.name		= "rndis",
	.init		= rndis_function_init,
	.cleanup	= rndis_function_cleanup,
	.bind_config	= rndis_function_bind_config,
	.unbind_config	= rndis_function_unbind_config,
	.attributes	= rndis_function_attributes,
	.performance_lock = 1,
};

static struct android_usb_function rndis_qc_function = {
	.name		= "rndis_qc",
	.init		= rndis_qc_function_init,
	.cleanup	= rndis_qc_function_cleanup,
	.bind_config	= rndis_qc_function_bind_config,
	.unbind_config	= rndis_qc_function_unbind_config,
	.attributes	= rndis_function_attributes,
};

static int ecm_function_bind_config(struct android_usb_function *f,
					struct usb_configuration *c)
{
	int ret;
	struct ecm_function_config *ecm = f->config;

	if (!ecm) {
		pr_err("%s: ecm_pdata\n", __func__);
		return -EINVAL;
	}

	pr_info("%s MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", __func__,
		ecm->ethaddr[0], ecm->ethaddr[1], ecm->ethaddr[2],
		ecm->ethaddr[3], ecm->ethaddr[4], ecm->ethaddr[5]);

	ret = gether_setup_name(c->cdev->gadget, ecm->ethaddr, "usb");
	if (ret) {
		pr_err("%s: gether_setup failed\n", __func__);
		return ret;
	}

	ret = ecm_bind_config(c, ecm->ethaddr);
	if (ret) {
		pr_err("%s: ecm_bind_config failed\n", __func__);
		gether_cleanup();
	}
	return ret;
}

static void ecm_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	gether_cleanup();
}

static struct android_usb_function ecm_function = {
	.name		= "cdc_ethernet",
	.init		= ecm_function_init,
	.cleanup	= ecm_function_cleanup,
	.bind_config	= ecm_function_bind_config,
	.unbind_config	= ecm_function_unbind_config,
	.attributes	= ecm_function_attributes,
	.performance_lock = 1,
};

struct mass_storage_function_config {
	struct fsg_config fsg;
	struct fsg_common *common;
};

#define MAX_LUN_NAME 8
static int mass_storage_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	struct android_dev *dev = cdev_to_android_dev(cdev);
	struct mass_storage_function_config *config;
	struct fsg_common *common;
	int err;
	int i;

	config = kzalloc(sizeof(struct mass_storage_function_config),
								GFP_KERNEL);
	if (!config)
		return -ENOMEM;

	if (dev->pdata) {
		config->fsg.nluns = dev->pdata->nluns;
		config->fsg.vendor_name = dev->pdata->manufacturer_name;
		config->fsg.product_name = dev->pdata->product_name;
		if (config->fsg.nluns > FSG_MAX_LUNS)
			config->fsg.nluns = FSG_MAX_LUNS;
		for (i = 0; i < config->fsg.nluns; i++) {
			if (dev->pdata->cdrom_lun & (1 << i)) {
				config->fsg.luns[i].cdrom = 1;
				config->fsg.luns[i].removable = 1;
				config->fsg.luns[i].ro = 1;
			} else {
				config->fsg.luns[i].cdrom = 0;
				config->fsg.luns[i].removable = 1;
				config->fsg.luns[i].ro = 0;
			}
		}
	} else {
		config->fsg.nluns = 1;
		config->fsg.luns[0].removable = 1;
	}

	common = fsg_common_init(NULL, cdev, &config->fsg);
	if (IS_ERR(common)) {
		kfree(config);
		return PTR_ERR(common);
	}

	for (i = 0; i < config->fsg.nluns; i++) {
		err = sysfs_create_link(&f->dev->kobj,
					&common->luns[i].dev.kobj,
					common->luns[i].dev.kobj.name);
		if (err)
			goto error;
	}

	config->common = common;
	f->config = config;
	return 0;
error:
	for (; i > 0 ; i--)
		sysfs_remove_link(&f->dev->kobj, common->luns[i].dev.kobj.name);

	fsg_common_release(&common->ref);
	kfree(config);
	return err;
}

static void mass_storage_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
	f->config = NULL;
}

static int mass_storage_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct mass_storage_function_config *config = f->config;
	return fsg_bind_config(c->cdev, c, config->common);
}

static ssize_t mass_storage_inquiry_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%s\n", config->common->inquiry_string);
}

static ssize_t mass_storage_inquiry_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct mass_storage_function_config *config = f->config;
	if (size >= sizeof(config->common->inquiry_string))
		return -EINVAL;
	if (sscanf(buf, "%28s", config->common->inquiry_string) != 1)
		return -EINVAL;
	return size;
}

static DEVICE_ATTR(inquiry_string, S_IRUGO | S_IWUSR,
					mass_storage_inquiry_show,
					mass_storage_inquiry_store);

static struct device_attribute *mass_storage_function_attributes[] = {
	&dev_attr_inquiry_string,
	NULL
};

static struct android_usb_function mass_storage_function = {
	.name		= "mass_storage",
	.init		= mass_storage_function_init,
	.cleanup	= mass_storage_function_cleanup,
	.bind_config	= mass_storage_function_bind_config,
	.attributes	= mass_storage_function_attributes,
};


static int accessory_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return acc_setup();
}

static void accessory_function_cleanup(struct android_usb_function *f)
{
	acc_cleanup();
}

static int accessory_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return acc_bind_config(c);
}

static int accessory_function_ctrlrequest(struct android_usb_function *f,
						struct usb_composite_dev *cdev,
						const struct usb_ctrlrequest *c)
{
	return acc_ctrlrequest(cdev, c);
}

static struct android_usb_function accessory_function = {
	.name		= "accessory",
	.init		= accessory_function_init,
	.cleanup	= accessory_function_cleanup,
	.bind_config	= accessory_function_bind_config,
	.ctrlrequest	= accessory_function_ctrlrequest,
};

#ifdef CONFIG_SND_PCM
static int audio_source_function_init(struct android_usb_function *f,
			struct usb_composite_dev *cdev)
{
	struct audio_source_config *config;

	config = kzalloc(sizeof(struct audio_source_config), GFP_KERNEL);
	if (!config)
		return -ENOMEM;
	config->card = -1;
	config->device = -1;
	f->config = config;
	return 0;
}

static void audio_source_function_cleanup(struct android_usb_function *f)
{
	kfree(f->config);
}

static int audio_source_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct audio_source_config *config = f->config;

	return audio_source_bind_config(c, config);
}

static void audio_source_function_unbind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	struct audio_source_config *config = f->config;

	config->card = -1;
	config->device = -1;
}

static ssize_t audio_source_pcm_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct audio_source_config *config = f->config;

	
	return sprintf(buf, "%d %d\n", config->card, config->device);
}

static DEVICE_ATTR(pcm, S_IRUGO | S_IWUSR, audio_source_pcm_show, NULL);

static struct device_attribute *audio_source_function_attributes[] = {
	&dev_attr_pcm,
	NULL
};

static struct android_usb_function audio_source_function = {
	.name		= "audio_source",
	.init		= audio_source_function_init,
	.cleanup	= audio_source_function_cleanup,
	.bind_config	= audio_source_function_bind_config,
	.unbind_config	= audio_source_function_unbind_config,
	.attributes	= audio_source_function_attributes,
};
#endif

static int android_uasp_connect_cb(bool connect)
{
	pr_debug("UASP %s\n", connect ? "connect" : "disconnect");

	return 0;
}

static int uasp_function_init(struct android_usb_function *f,
					struct usb_composite_dev *cdev)
{
	return f_tcm_init(&android_uasp_connect_cb);
}

static void uasp_function_cleanup(struct android_usb_function *f)
{
	f_tcm_exit();
}

static int uasp_function_bind_config(struct android_usb_function *f,
						struct usb_configuration *c)
{
	return tcm_bind_config(c);
}

static struct android_usb_function uasp_function = {
	.name		= "uasp",
	.init		= uasp_function_init,
	.cleanup	= uasp_function_cleanup,
	.bind_config	= uasp_function_bind_config,
};


static int projector_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct htcmode_protocol), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return projector_setup(f->config);
}

static void projector_function_cleanup(struct android_usb_function *f)
{
	projector_cleanup();
	kfree(f->config);
}

static int projector_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return projector_bind_config(c);
}


static ssize_t projector_width_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->server_info.width);
}

static DEVICE_ATTR(width, S_IRUGO | S_IWUSR, projector_width_show,
						    NULL);

static ssize_t projector_height_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->server_info.height);
}

static DEVICE_ATTR(height, S_IRUGO | S_IWUSR, projector_height_show,
						    NULL);

static ssize_t projector_rotation_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", (config->client_info.display_conf & CLIENT_INFO_SERVER_ROTATE_USED));
}

static DEVICE_ATTR(rotation, S_IRUGO | S_IWUSR, projector_rotation_show,
						    NULL);

static ssize_t projector_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->version);
}

static DEVICE_ATTR(version, S_IRUGO | S_IWUSR, projector_version_show,
						    NULL);

static ssize_t projector_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->vendor);
}

static DEVICE_ATTR(vendor, S_IRUGO | S_IWUSR, projector_vendor_show,
						    NULL);

static ssize_t projector_server_nonce_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	memcpy(buf, config->nonce, HSML_SERVER_NONCE_SIZE);
	return HSML_SERVER_NONCE_SIZE;
}

static DEVICE_ATTR(server_nonce, S_IRUGO | S_IWUSR, projector_server_nonce_show,
						    NULL);

static ssize_t projector_client_sig_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	memcpy(buf, config->client_sig, HSML_CLIENT_SIG_SIZE);
	return HSML_CLIENT_SIG_SIZE;
}

static DEVICE_ATTR(client_sig, S_IRUGO | S_IWUSR, projector_client_sig_show,
						    NULL);

static ssize_t projector_server_sig_store(
		struct device *dev, struct device_attribute *attr,
		const char *buff, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	memcpy(config->server_sig, buff, HSML_SERVER_SIG_SIZE);
	return HSML_SERVER_SIG_SIZE;
}

static DEVICE_ATTR(server_sig, S_IWUSR, NULL,
		projector_server_sig_store);

static ssize_t projector_auth_store(
		struct device *dev, struct device_attribute *attr,
		const char *buff, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	memcpy(&config->auth_result, buff, sizeof(config->auth_result));
	config->auth_in_progress = 0;
	return sizeof(config->auth_result);
}

static DEVICE_ATTR(auth, S_IWUSR, NULL,
		projector_auth_store);

static ssize_t projector_debug_mode_store(
		struct device *dev, struct device_attribute *attr,
		const char *buff, size_t size)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct htcmode_protocol *config = f->config;
	int value, i;
	int framesize = DEFAULT_PROJ_HEIGHT * DEFAULT_PROJ_WIDTH;

	if (sscanf(buff, "%d", &value) == 1) {

		if (!test_frame)
			test_frame = kzalloc(framesize * 2, GFP_KERNEL);

		if (test_frame)
			for (i = 0 ; i < framesize ; i++)
				if (i < framesize/4)
					test_frame[i] = 0xF800;
				else if (i < framesize*2/4)
					test_frame[i] = 0x7E0;
				else if (i < framesize*3/4)
					test_frame[i] = 0x1F;
				else
					test_frame[i] = 0xFFFF;

		config->debug_mode = value;
		return size;
	}
	return -EINVAL;
}

static DEVICE_ATTR(debug_mode, S_IWUSR, NULL,
		projector_debug_mode_store);

static struct device_attribute *projector_function_attributes[] = {
	&dev_attr_width,
	&dev_attr_height,
	&dev_attr_rotation,
	&dev_attr_version,
	&dev_attr_vendor,
	&dev_attr_server_nonce,
	&dev_attr_client_sig,
	&dev_attr_server_sig,
	&dev_attr_auth,
	&dev_attr_debug_mode,
	NULL
};


struct android_usb_function projector_function = {
	.name		= "projector",
	.init		= projector_function_init,
	.cleanup	= projector_function_cleanup,
	.bind_config	= projector_function_bind_config,
	.attributes = projector_function_attributes
};

static int projector2_function_init(struct android_usb_function *f,
		struct usb_composite_dev *cdev)
{
	f->config = kzalloc(sizeof(struct hsml_protocol), GFP_KERNEL);
	if (!f->config)
		return -ENOMEM;

	return projector2_setup(f->config);
}

static void projector2_function_cleanup(struct android_usb_function *f)
{

	projector2_cleanup();

	if (f->config) {
		kfree(f->config);
		f->config = NULL;
	}
}

static int projector2_function_bind_config(struct android_usb_function *f,
		struct usb_configuration *c)
{
	return projector2_bind_config(c);
}

static ssize_t projector2_width_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct hsml_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->set_display_info.wWidth);
}

#if HSML_VERSION_12
#define cHSML_WIDTH_SIZE        2

static ssize_t projector2_width_store(struct device *dev,
        struct device_attribute *attr, const char *buff, size_t size)
{
struct android_usb_function *f = dev_get_drvdata(dev);
struct hsml_protocol *config = f->config;
u16 uValue;
u8 aucWidth[cHSML_WIDTH_SIZE];

    if (size <= cHSML_WIDTH_SIZE) {
        memset(aucWidth, 0, sizeof(aucWidth));
        memcpy(aucWidth, buff, size);
        uValue = be16_to_cpu(*((__le16 *) aucWidth));
        config->set_display_info.wWidth = uValue;
        return size;
    } else {
        printk(KERN_ERR "%s: size is invalid %d/%d\n", __func__, size, cHSML_WIDTH_SIZE);
    }
    return -EINVAL;
}
#endif

static DEVICE_ATTR(client_width, S_IRUGO | S_IWUSR, projector2_width_show,
#if !HSML_VERSION_12
                            NULL
#else
                            projector2_width_store
#endif
        );

static ssize_t projector2_height_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct hsml_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->set_display_info.wHeight);
}

#if HSML_VERSION_12
static ssize_t projector2_height_store(struct device *dev,
        struct device_attribute *attr, const char *buff, size_t size)
{
struct android_usb_function *f = dev_get_drvdata(dev);
struct hsml_protocol *config = f->config;
u16 uValue;
u8 aucWidth[cHSML_WIDTH_SIZE];

    if (size <= cHSML_WIDTH_SIZE) {
        memset(aucWidth, 0, sizeof(aucWidth));
        memcpy(aucWidth, buff, size);
        uValue = be16_to_cpu(*((__le16 *) aucWidth));
        config->set_display_info.wHeight = uValue;
        return size;
    } else {
        printk(KERN_ERR "%s: size is invalid %d/%d\n", __func__, size, cHSML_WIDTH_SIZE);
    }

    return -EINVAL;
}
#endif

static DEVICE_ATTR(client_height, S_IRUGO | S_IWUSR, projector2_height_show,
#if !HSML_VERSION_12
                            NULL
#else
                            projector2_height_store
#endif
                            );

static ssize_t projector2_maxfps_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct hsml_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->MaxFPS);
}

static DEVICE_ATTR(client_maxfps, S_IRUGO | S_IWUSR, projector2_maxfps_show,
						    NULL);

static ssize_t projector2_pixel_format_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct android_usb_function *f = dev_get_drvdata(dev);
	struct hsml_protocol *config = f->config;
	return snprintf(buf, PAGE_SIZE, "%d\n", config->set_display_info.bPixelFormat);
}

static DEVICE_ATTR(client_pixel_format, S_IRUGO | S_IWUSR, projector2_pixel_format_show,
						    NULL);

static DEVICE_ATTR(client_context_info, S_IRUGO | S_IWUSR, NULL, context_info_store);
#if HSML_VERSION_12
static DEVICE_ATTR(client_ver, S_IRUGO | S_IWUSR, projector2_ver_show, NULL);
static DEVICE_ATTR(client_cap, S_IRUGO | S_IWUSR, projector2_cap_show, NULL);
static DEVICE_ATTR(client_uuid, S_IRUGO | S_IWUSR, NULL, projector2_uuid_store);
#endif

static struct device_attribute *projector2_function_attributes[] = {
	&dev_attr_client_width,
	&dev_attr_client_height,
	&dev_attr_client_maxfps,
	&dev_attr_client_pixel_format,
	&dev_attr_client_context_info,
#if HSML_VERSION_12
	&dev_attr_client_ver,
	&dev_attr_client_cap,
	&dev_attr_client_uuid,
#endif
	NULL
};

struct android_usb_function projector2_function = {
	.name		= "projector2",
	.init		= projector2_function_init,
	.cleanup	= projector2_function_cleanup,
	.bind_config	= projector2_function_bind_config,
	.attributes = projector2_function_attributes
};


static struct android_usb_function *supported_functions[] = {
	&rndis_function,
	&rndis_qc_function,
	&accessory_function,
#ifdef CONFIG_SND_PCM
	&audio_source_function,
#endif
	&mtp_function,
	&ptp_function,
	&ncm_function,

	&adb_function,
	&mass_storage_function,
	&ecm_function,

	&diag_function,
	&modem_function,
	&serial_function,
	&projector_function,
	&projector2_function,
	&acm_function,

	&rmnet_smd_function,
	&rmnet_sdio_function,
	&rmnet_smd_sdio_function,
	&rmnet_function,
	&mbim_function,
	&ecm_qc_function,
#ifdef CONFIG_SND_PCM
	&audio_function,
#endif
	&gps_function,
	&qdss_function,
	&ccid_function,
	&uasp_function,
	NULL
};

static void android_cleanup_functions(struct android_usb_function **functions)
{
	struct android_usb_function *f;
	struct device_attribute **attrs;
	struct device_attribute *attr;

	while (*functions) {
		f = *functions++;

		if (f->dev) {
			device_destroy(android_class, f->dev->devt);
			kfree(f->dev_name);
		} else
			continue;

		if (f->cleanup)
			f->cleanup(f);

		attrs = f->attributes;
		if (attrs) {
			while ((attr = *attrs++))
				device_remove_file(f->dev, attr);
		}
	}
}

static int android_init_functions(struct android_usb_function **functions,
				  struct usb_composite_dev *cdev)
{
	struct android_dev *dev = cdev_to_android_dev(cdev);
	struct android_usb_function *f;
	struct device_attribute **attrs;
	struct device_attribute *attr;
	int err = 0;
	int index = 1; 

	for (; (f = *functions++); index++) {
		f->dev_name = kasprintf(GFP_KERNEL, "f_%s", f->name);
		f->android_dev = NULL;
		if (!f->dev_name) {
			err = -ENOMEM;
			goto err_out;
		}
		f->dev = device_create(android_class, dev->dev,
				MKDEV(0, index), f, f->dev_name);
		if (IS_ERR(f->dev)) {
			pr_err("%s: Failed to create dev %s", __func__,
							f->dev_name);
			err = PTR_ERR(f->dev);
			f->dev = NULL;
			goto err_create;
		}

		if (device_create_file(f->dev, &dev_attr_on) < 0) {
			pr_err("%s: Failed to create dev file %s", __func__,
					f->dev_name);
			goto err_create;
		}

		if (f->init) {
			err = f->init(f, cdev);
			if (err) {
				pr_err("%s: Failed to init %s", __func__,
								f->name);
				goto err_init;
			}
		}

		attrs = f->attributes;
		if (attrs) {
			while ((attr = *attrs++) && !err)
				err = device_create_file(f->dev, attr);
		}
		if (err) {
			pr_err("%s: Failed to create function %s attributes",
					__func__, f->name);
			goto err_attrs;
		}
		pr_info("%s %s init\n", __func__, f->name);
	}
	return 0;

err_attrs:
	for (attr = *(attrs -= 2); attrs != f->attributes; attr = *(attrs--))
		device_remove_file(f->dev, attr);
	if (f->cleanup)
		f->cleanup(f);
err_init:
	device_destroy(android_class, f->dev->devt);
err_create:
	f->dev = NULL;
	kfree(f->dev_name);
err_out:
	android_cleanup_functions(dev->functions);
	return err;
}

static int
android_bind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function_holder *f_holder;
	struct android_configuration *conf =
		container_of(c, struct android_configuration, usb_config);
	int ret;

	list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list) {
		pr_info("%s bind name: %s\n", __func__, f_holder->f->name);
		ret = f_holder->f->bind_config(f_holder->f, c);
		if (ret) {
			pr_err("%s: %s failed\n", __func__, f_holder->f->name);
			while (!list_empty(&c->functions)) {
				struct usb_function		*f;

				f = list_first_entry(&c->functions,
					struct usb_function, list);
				list_del(&f->list);
				if (f->unbind)
					f->unbind(c, f);
			}
			if (c->unbind)
				c->unbind(c);
			return ret;
		}
	}
	return 0;
}

static void
android_unbind_enabled_functions(struct android_dev *dev,
			       struct usb_configuration *c)
{
	struct android_usb_function_holder *f_holder;
	struct android_configuration *conf =
		container_of(c, struct android_configuration, usb_config);

	list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list) {
		if (f_holder->f->unbind_config)
			f_holder->f->unbind_config(f_holder->f, c);
	}
}

static inline void check_streaming_func(struct usb_gadget *gadget,
		struct android_usb_platform_data *pdata,
		char *name)
{
	int i;

	for (i = 0; i < pdata->streaming_func_count; i++) {
		if (!strcmp(name,
			pdata->streaming_func[i])) {
			pr_debug("set streaming_enabled to true\n");
			gadget->streaming_enabled = true;
			break;
		}
	}
}

static int android_enable_function(struct android_dev *dev,
				   struct android_configuration *conf,
				   char *name)
{
	struct android_usb_function **functions = dev->functions;
	struct android_usb_function *f;
	struct android_usb_function_holder *f_holder;
	struct android_usb_platform_data *pdata = dev->pdata;
	struct usb_gadget *gadget = dev->cdev->gadget;

	while ((f = *functions++)) {
		if (!strcmp(name, f->name)) {
			if (f->android_dev && f->android_dev != dev)
				pr_err("%s is enabled in other device\n",
					f->name);
			else {
				f_holder = kzalloc(sizeof(*f_holder),
						GFP_KERNEL);
				if (!f_holder) {
					pr_err("Failed to alloc f_holder\n");
					return -ENOMEM;
				}

				f->android_dev = dev;
				f_holder->f = f;
				list_add_tail(&f_holder->enabled_list,
					      &conf->enabled_functions);
				pr_debug("func:%s is enabled.\n", f->name);
				check_streaming_func(gadget, pdata, f->name);

				return 0;
			}
		}
	}
	return -EINVAL;
}

#include "htc_attr.c"


static ssize_t remote_wakeup_show(struct device *pdev,
		struct device_attribute *attr, char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct android_configuration *conf;

	if (dev->configs_num == 0)
		return 0;
	conf = list_entry(dev->configs.next,
			  struct android_configuration,
			  list_item);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			!!(conf->usb_config.bmAttributes &
				USB_CONFIG_ATT_WAKEUP));
}

static ssize_t remote_wakeup_store(struct device *pdev,
		struct device_attribute *attr, const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct android_configuration *conf;
	int enable = 0;

	sscanf(buff, "%d", &enable);

	pr_debug("android_usb: %s remote wakeup\n",
			enable ? "enabling" : "disabling");

	list_for_each_entry(conf, &dev->configs, list_item)
		if (enable)
			conf->usb_config.bmAttributes |=
					USB_CONFIG_ATT_WAKEUP;
		else
			conf->usb_config.bmAttributes &=
					~USB_CONFIG_ATT_WAKEUP;

	return size;
}

static ssize_t restart_adbd_store(struct device *pdev,
		struct device_attribute *attr, const char *buff, size_t size)
{
	int enable = 0;

	sscanf(buff, "%d", &enable);

	pr_debug("restart adbd = %d\n", enable);

	if (enable){
		android_force_reset();
	}

	return size;
}

static ssize_t
functions_show(struct device *pdev, struct device_attribute *attr, char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct android_configuration *conf;
	struct android_usb_function_holder *f_holder;
	char *buff = buf;

	mutex_lock(&dev->mutex);

	list_for_each_entry(conf, &dev->configs, list_item) {
		if (buff != buf)
			*(buff-1) = ':';
		list_for_each_entry(f_holder, &conf->enabled_functions,
					enabled_list)
			buff += snprintf(buff, PAGE_SIZE, "%s,",
					f_holder->f->name);
	}

	mutex_unlock(&dev->mutex);

	if (buff != buf)
		*(buff-1) = '\n';
	return buff - buf;
}

static ssize_t
functions_store(struct device *pdev, struct device_attribute *attr,
			       const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct list_head *curr_conf = &dev->configs;
	struct android_configuration *conf;
	char *conf_str;
	struct android_usb_function_holder *f_holder;
	char *name;
	char buf[256], *b;
	int err;

	
	strlcpy(buf, buff, sizeof(buf));
	buf[size] = 0;
	printk(KERN_INFO "[USB]%s %s\n",__func__,buf);
	return size;

	mutex_lock(&dev->mutex);

	if (dev->enabled) {
		mutex_unlock(&dev->mutex);
		return -EBUSY;
	}

	
	list_for_each_entry(conf, &dev->configs, list_item) {
		while (conf->enabled_functions.next !=
				&conf->enabled_functions) {
			f_holder = list_entry(conf->enabled_functions.next,
					typeof(*f_holder),
					enabled_list);
			f_holder->f->android_dev = NULL;
			list_del(&f_holder->enabled_list);
			kfree(f_holder);
		}
		INIT_LIST_HEAD(&conf->enabled_functions);
	}

	strlcpy(buf, buff, sizeof(buf));
	b = strim(buf);

	while (b) {
		conf_str = strsep(&b, ":");
		if (conf_str) {
			
			if (curr_conf->next != &dev->configs)
				conf = list_entry(curr_conf->next,
						  struct android_configuration,
						  list_item);
			else
				conf = alloc_android_config(dev);

			curr_conf = curr_conf->next;
		}

		while (conf_str) {
			name = strsep(&conf_str, ",");
			if (name) {
				err = android_enable_function(dev, conf, name);
				if (err)
					pr_err("android_usb: Cannot enable %s",
						name);
			}
		}
	}

	
	while (curr_conf->next != &dev->configs) {
		conf = list_entry(curr_conf->next,
				  struct android_configuration, list_item);
		free_android_config(dev, conf);
	}

	mutex_unlock(&dev->mutex);

	return size;
}

static ssize_t __maybe_unused enable_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);

	printk(KERN_INFO "[USB]%s\n",__func__);
	return 0;

	return snprintf(buf, PAGE_SIZE, "%d\n", dev->enabled);
}

static ssize_t __maybe_unused enable_store(struct device *pdev, struct device_attribute *attr,
			    const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	struct android_usb_function_holder *f_holder;
	struct android_configuration *conf;
	int enabled = 0;
	bool audio_enabled = false;
	static DEFINE_RATELIMIT_STATE(rl, 10*HZ, 1);
	int err = 0;

	printk(KERN_INFO "[USB]%s\n",__func__);
	return size;

	if (!cdev)
		return -ENODEV;

	mutex_lock(&dev->mutex);

	sscanf(buff, "%d", &enabled);
	if (enabled && !dev->enabled) {
		cdev->desc.idVendor = device_desc.idVendor;
		cdev->desc.idProduct = device_desc.idProduct;
		cdev->desc.bcdDevice = device_desc.bcdDevice;
		cdev->desc.bDeviceClass = device_desc.bDeviceClass;
		cdev->desc.bDeviceSubClass = device_desc.bDeviceSubClass;
		cdev->desc.bDeviceProtocol = device_desc.bDeviceProtocol;

		list_for_each_entry(conf, &dev->configs, list_item)
			list_for_each_entry(f_holder, &conf->enabled_functions,
						enabled_list) {
				if (f_holder->f->enable)
					f_holder->f->enable(f_holder->f);
				if (!strncmp(f_holder->f->name,
						"audio_source", 12))
					audio_enabled = true;
			}
		if (audio_enabled)
			msleep(100);
		err = android_enable(dev);
		if (err < 0) {
			pr_err("%s: android_enable failed\n", __func__);
			dev->connected = 0;
			dev->enabled = false;
			mutex_unlock(&dev->mutex);
			return size;
		}
		dev->enabled = true;
	} else if (!enabled && dev->enabled) {
		android_disable(dev);
		list_for_each_entry(conf, &dev->configs, list_item)
			list_for_each_entry(f_holder, &conf->enabled_functions,
						enabled_list) {
				if (f_holder->f->disable)
					f_holder->f->disable(f_holder->f);
			}
		dev->enabled = false;
	} else if (__ratelimit(&rl)) {
		pr_err("android_usb: already %s\n",
				dev->enabled ? "enabled" : "disabled");
	}

	mutex_unlock(&dev->mutex);

	return size;
}

static ssize_t pm_qos_show(struct device *pdev,
			   struct device_attribute *attr, char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);

	return snprintf(buf, PAGE_SIZE, "%s\n", dev->pm_qos);
}

static ssize_t pm_qos_store(struct device *pdev,
			   struct device_attribute *attr,
			   const char *buff, size_t size)
{
	struct android_dev *dev = dev_get_drvdata(pdev);

	strlcpy(dev->pm_qos, buff, sizeof(dev->pm_qos));

	return size;
}

static ssize_t state_show(struct device *pdev, struct device_attribute *attr,
			   char *buf)
{
	struct android_dev *dev = dev_get_drvdata(pdev);
	struct usb_composite_dev *cdev = dev->cdev;
	char *state = "DISCONNECTED";
	unsigned long flags;

	if (!cdev)
		goto out;

	spin_lock_irqsave(&cdev->lock, flags);
	if (cdev->config)
		state = "CONFIGURED";
	else if (dev->connected)
		state = "CONNECTED";
	spin_unlock_irqrestore(&cdev->lock, flags);
out:
	return snprintf(buf, PAGE_SIZE, "%s\n", state);
}

static ssize_t bugreport_debug_store(struct device *pdev,
		struct device_attribute *attr, const char *buff, size_t size)
{
	int enable = 0, ats = 0;
	sscanf(buff, "%d", &enable);
	ats = board_get_usb_ats();

	if (enable == 5 && ats)
		bugreport_debug = 1;
	else if (enable == 0 && ats) {
		bugreport_debug = 0;
		del_timer(&adb_read_timer);
	}

	pr_info("bugreport_debug = %d, enable=%d, ats = %d\n", bugreport_debug, enable, ats);

	return size;
}
static ssize_t adb_reboot_debug_store(struct device *pdev,
		struct device_attribute *attr, const char *buff, size_t size)
{
	char buf[256];
	strlcpy(buf, buff, sizeof(buf));
	buf[size] = 0;
	pr_info("%s\n",buf);

	return size;
}

#define DESCRIPTOR_ATTR(field, format_string)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE,					\
			format_string, device_desc.field);		\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	int value;							\
	if (sscanf(buf, format_string, &value) == 1) {			\
		device_desc.field = value;				\
		return size;						\
	}								\
	return -1;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

#define DESCRIPTOR_STRING_ATTR(field, buffer)				\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE, "%s", buffer);			\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	pr_info("%s: %s\n", __func__, buf);				\
	if (size >= sizeof(buffer))					\
		return -EINVAL;						\
	strlcpy(buffer, buf, sizeof(buffer));				\
	strim(buffer);							\
	return size;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

#if 0
DESCRIPTOR_ATTR(idVendor, "%04x\n")
DESCRIPTOR_ATTR(idProduct, "%04x\n")
DESCRIPTOR_ATTR(bcdDevice, "%04x\n")
DESCRIPTOR_ATTR(bDeviceClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceSubClass, "%d\n")
DESCRIPTOR_ATTR(bDeviceProtocol, "%d\n")
DESCRIPTOR_STRING_ATTR(iManufacturer, manufacturer_string)
DESCRIPTOR_STRING_ATTR(iProduct, product_string)
DESCRIPTOR_STRING_ATTR(iSerial, serial_string)
#endif
static DEVICE_ATTR(functions, S_IRUGO | S_IWUSR, functions_show,
						 functions_store);
static DEVICE_ATTR(restart_adbd, 0664, NULL, restart_adbd_store);
static DEVICE_ATTR(pm_qos, S_IRUGO | S_IWUSR,
		pm_qos_show, pm_qos_store);
static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);
static DEVICE_ATTR(remote_wakeup, S_IRUGO | S_IWUSR,
		remote_wakeup_show, remote_wakeup_store);
static DEVICE_ATTR(bugreport_debug, 0664, NULL, bugreport_debug_store);
static DEVICE_ATTR(adb_reboot_debug, 0664, NULL, adb_reboot_debug_store);

static struct device_attribute *android_usb_attributes[] = {
	#if 0
	&dev_attr_idVendor,
	&dev_attr_idProduct,
	&dev_attr_bcdDevice,
	&dev_attr_bDeviceClass,
	&dev_attr_bDeviceSubClass,
	&dev_attr_bDeviceProtocol,
	&dev_attr_iManufacturer,
	&dev_attr_iProduct,
	&dev_attr_iSerial,
	#endif
	&dev_attr_functions,
	&dev_attr_restart_adbd,
	&dev_attr_pm_qos,
	&dev_attr_state,
	&dev_attr_remote_wakeup,
	&dev_attr_bugreport_debug,
	&dev_attr_adb_reboot_debug,
	NULL
};


static int android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = cdev_to_android_dev(c->cdev);
	int ret = 0;

	ret = android_bind_enabled_functions(dev, c);
	if (ret)
		return ret;

	return 0;
}

static void android_unbind_config(struct usb_configuration *c)
{
	struct android_dev *dev = cdev_to_android_dev(c->cdev);

	if (c->cdev->gadget->streaming_enabled) {
		c->cdev->gadget->streaming_enabled = false;
		pr_debug("setting streaming_enabled to false.\n");
	}
	android_unbind_enabled_functions(dev, c);
}

static int android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev;
	struct usb_gadget	*gadget = cdev->gadget;
	struct android_configuration *conf;
	int			gcnum, id, ret;

	
	dev = list_entry(android_dev_list.prev, struct android_dev, list_item);

	dev->cdev = cdev;

	usb_gadget_disconnect(gadget);

	
	if (android_dev_count == 1) {
		ret = android_init_functions(dev->functions, cdev);
		if (ret)
			return ret;
	}

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

	if (gadget_is_otg(cdev->gadget))
		list_for_each_entry(conf, &dev->configs, list_item) {
			printk("[USB]=====insert otg_desc\n");
			conf->usb_config.descriptors = otg_desc;
		}

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	cdev->sw_connect2pc.name = "usb_connect2pc";
	ret = switch_dev_register(&cdev->sw_connect2pc);
	if (ret < 0)
		pr_err("switch_dev_register fail:usb_connect2pc\n");

	return 0;
}

static int android_usb_unbind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = cdev_to_android_dev(cdev);

	manufacturer_string[0] = '\0';
	product_string[0] = '\0';
	serial_string[0] = '0';
	cancel_work_sync(&dev->work);
	android_cleanup_functions(dev->functions);
	switch_dev_unregister(&cdev->sw_connect2pc);
	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.unbind		= android_usb_unbind,
	.max_speed	= USB_SPEED_SUPER
};

static int
android_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *c)
{
	struct usb_composite_dev	*cdev = get_gadget_data(gadget);
	struct android_dev		*dev = cdev_to_android_dev(cdev);
	struct usb_request		*req = cdev->req;
	struct android_usb_function	*f;
	struct android_usb_function_holder *f_holder;
	struct android_configuration	*conf;
	int value = -EOPNOTSUPP;
	unsigned long flags;
	bool do_work = false;
	bool prev_configured = false;

	req->zero = 0;
	req->complete = composite_setup_complete;
	req->length = 0;
	gadget->ep0->driver_data = cdev;


	value = android_switch_setup(gadget, c);
	if (value >= 0)
		return value;

	list_for_each_entry(conf, &dev->configs, list_item)
		list_for_each_entry(f_holder,
				    &conf->enabled_functions,
				    enabled_list) {
			f = f_holder->f;
			if (f->ctrlrequest) {
				value = f->ctrlrequest(f, cdev, c);
				if (value >= 0)
					break;
			}
		}

	if (cdev->config)
		prev_configured = true;
	if (value < 0)
		value = acc_ctrlrequest(cdev, c);

	if (value < 0)
		value = projector_ctrlrequest(cdev, c);

	if (value < 0)
		value = projector2_ctrlrequest(cdev, c);

	if (value < 0)
		value = composite_setup(gadget, c);

	spin_lock_irqsave(&cdev->lock, flags);
	if (!dev->connected) {
		dev->connected = 1;
		do_work = true;
	} else if (c->bRequest == USB_REQ_SET_CONFIGURATION &&
						cdev->config) {
		if (!prev_configured)
			do_work = true;
	}
	spin_unlock_irqrestore(&cdev->lock, flags);
	if (do_work)
		schedule_work(&dev->work);
	return value;
}

static void android_mute_disconnect(struct usb_gadget *gadget)
{
	composite_disconnect(gadget);

	
	
}

static void android_disconnect(struct usb_gadget *gadget)
{
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	struct android_dev *dev = cdev_to_android_dev(cdev);
	unsigned long flags;

	composite_disconnect(gadget);
	acc_disconnect();

	spin_lock_irqsave(&cdev->lock, flags);
	dev->connected = 0;
	schedule_work(&dev->work);
	spin_unlock_irqrestore(&cdev->lock, flags);
}

static void android_suspend(struct usb_gadget *gadget)
{
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	struct android_dev *dev = cdev_to_android_dev(cdev);
	unsigned long flags;

	spin_lock_irqsave(&cdev->lock, flags);
	if (!dev->suspended) {
		dev->suspended = 1;
		schedule_work(&dev->work);
	}
	spin_unlock_irqrestore(&cdev->lock, flags);

	composite_suspend(gadget);
}

static void android_resume(struct usb_gadget *gadget)
{
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	struct android_dev *dev = cdev_to_android_dev(cdev);
	unsigned long flags;

	spin_lock_irqsave(&cdev->lock, flags);
	if (dev->suspended) {
		dev->suspended = 0;
		schedule_work(&dev->work);
	}
	spin_unlock_irqrestore(&cdev->lock, flags);

	composite_resume(gadget);
}


static int android_create_device(struct android_dev *dev, u8 usb_core_id)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;
	char device_node_name[ANDROID_DEVICE_NODE_NAME_LENGTH];
	int err;

	snprintf(device_node_name, ANDROID_DEVICE_NODE_NAME_LENGTH,
		 "android%d", usb_core_id);
	dev->dev = device_create(android_class, NULL,
					MKDEV(0, 0), NULL, device_node_name);
	if (IS_ERR(dev->dev))
		return PTR_ERR(dev->dev);

	dev_set_drvdata(dev->dev, dev);

	while ((attr = *attrs++)) {
		err = device_create_file(dev->dev, attr);
		if (err) {
			device_destroy(android_class, dev->dev->devt);
			return err;
		}
	}
	return 0;
}

static void android_destroy_device(struct android_dev *dev)
{
	struct device_attribute **attrs = android_usb_attributes;
	struct device_attribute *attr;

	while ((attr = *attrs++))
		device_remove_file(dev->dev, attr);
	device_destroy(android_class, dev->dev->devt);
}

static struct android_dev *cdev_to_android_dev(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = NULL;

	
	list_for_each_entry(dev, &android_dev_list, list_item) {
		if (dev->cdev == cdev)
			break;
	}

	return dev;
}

static struct android_configuration *alloc_android_config
						(struct android_dev *dev)
{
	struct android_configuration *conf;

	conf = kzalloc(sizeof(*conf), GFP_KERNEL);
	if (!conf) {
		pr_err("%s(): Failed to alloc memory for android conf\n",
			__func__);
		return ERR_PTR(-ENOMEM);
	}

	dev->configs_num++;
	conf->usb_config.label = dev->name;
	conf->usb_config.unbind = android_unbind_config;
	conf->usb_config.bConfigurationValue = dev->configs_num;
	conf->usb_config.bmAttributes = USB_CONFIG_ATT_SELFPOWER;
	INIT_LIST_HEAD(&conf->enabled_functions);

	list_add_tail(&conf->list_item, &dev->configs);

	return conf;
}

static void free_android_config(struct android_dev *dev,
			     struct android_configuration *conf)
{
	list_del(&conf->list_item);
	dev->configs_num--;
	kfree(conf);
}

static int usb_diag_update_pid_and_serial_num(u32 pid, const char *snum)
{
	struct dload_struct local_diag_dload = { 0 };
	int *src, *dst, i;

	if (!diag_dload || !diag_update_supported) {
		pr_debug("%s: unable to update PID and serial_no\n", __func__);
		return -ENODEV;
	}

	pr_debug("%s: dload:%p pid:%x serial_num:%s\n",
				__func__, diag_dload, pid, snum);

	
	local_diag_dload.magic_struct.pid = PID_MAGIC_ID;
	local_diag_dload.pid = pid;

	
	if (!snum) {
		local_diag_dload.magic_struct.serial_num = 0;
		memset(&local_diag_dload.serial_number, 0,
				SERIAL_NUMBER_LENGTH);
	} else {
		local_diag_dload.magic_struct.serial_num = SERIAL_NUM_MAGIC_ID;
		strlcpy((char *)&local_diag_dload.serial_number, snum,
				SERIAL_NUMBER_LENGTH);
	}

	
	src = (int *)&local_diag_dload;
	dst = (int *)diag_dload;

	for (i = 0; i < sizeof(*diag_dload) / 4; i++)
		*dst++ = *src++;

	return 0;
}

static int __devinit android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata;
	struct android_dev *android_dev;
	struct resource *res;
	int ret = 0, i, len = 0;

	init_mfg_serialno();

	if (pdev->dev.of_node) {
		dev_dbg(&pdev->dev, "device tree enabled\n");
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			pr_err("unable to allocate platform data\n");
			return -ENOMEM;
		}

		of_property_read_u32(pdev->dev.of_node,
				"qcom,android-usb-swfi-latency",
				&pdata->swfi_latency);
		pdata->cdrom = of_property_read_bool(pdev->dev.of_node,
				"qcom,android-usb-cdrom");
		pdata->internal_ums = of_property_read_bool(pdev->dev.of_node,
				"qcom,android-usb-internal-ums");
		len = of_property_count_strings(pdev->dev.of_node,
				"qcom,streaming-func");
		if (len > MAX_STREAMING_FUNCS) {
			pr_err("Invalid number of functions used.\n");
			return -EINVAL;
		}

		for (i = 0; i < len; i++) {
			const char *name = NULL;

			of_property_read_string_index(pdev->dev.of_node,
				"qcom,streaming-func", i, &name);
			if (!name)
				continue;

			if (sizeof(name) > FUNC_NAME_LEN) {
				pr_err("Function name is bigger than allowed.\n");
				continue;
			}

			strlcpy(pdata->streaming_func[i], name,
				sizeof(pdata->streaming_func[i]));
			pr_debug("name of streaming function:%s\n",
				pdata->streaming_func[i]);
		}

		pdata->streaming_func_count = len;

		ret = of_property_read_u32(pdev->dev.of_node,
				"qcom,android-usb-uicc-nluns",
				&pdata->uicc_nluns);
	} else {
		pdata = pdev->dev.platform_data;
	}

	if (!android_class) {
		android_class = class_create(THIS_MODULE, "android_usb");
		if (IS_ERR(android_class))
			return PTR_ERR(android_class);
	}

	android_dev = kzalloc(sizeof(*android_dev), GFP_KERNEL);
	if (!android_dev) {
		pr_err("%s(): Failed to alloc memory for android_dev\n",
			__func__);
		ret = -ENOMEM;
		goto err_alloc;
	}

	android_dev->pdev = pdev;
	android_dev->name = pdev->name;
	android_dev->disable_depth = 1;
	android_dev->functions = supported_functions;
	android_dev->configs_num = 0;
	INIT_LIST_HEAD(&android_dev->configs);
	INIT_WORK(&android_dev->work, android_work);
	mutex_init(&android_dev->mutex);

	android_dev->pdata = pdata;

	list_add_tail(&android_dev->list_item, &android_dev_list);
	android_dev_count++;

	if (pdata) {
		composite_driver.usb_core_id = pdata->usb_core_id;
		pdata->swfi_latency = 1;
	}
	else
		composite_driver.usb_core_id = 0; 

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		diag_dload = devm_ioremap(&pdev->dev, res->start,
							resource_size(res));
		if (!diag_dload) {
			dev_err(&pdev->dev, "ioremap failed\n");
			ret = -ENOMEM;
			goto err_dev;
		}
	} else {
		dev_dbg(&pdev->dev, "failed to get mem resource\n");
	}

#ifdef CONFIG_PERFLOCK
	perf_lock_init(&android_usb_perf_lock, TYPE_PERF_LOCK, PERF_LOCK_LOW, "android_usb");
#endif

	ret = android_create_device(android_dev, composite_driver.usb_core_id);
	if (ret) {
		pr_err("%s(): android_create_device failed\n", __func__);
		goto err_dev;
	}

	ret = usb_composite_probe(&android_usb_driver, android_bind);
	if (ret) {
		pr_err("%s(): Failed to register android "
				 "composite driver\n", __func__);
		goto err_probe;
	}

	
	if (pdata && pdata->swfi_latency)
		pm_qos_add_request(&android_dev->pm_qos_req_dma,
			PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	strlcpy(android_dev->pm_qos, "high", sizeof(android_dev->pm_qos));

	setup_vendor_info(android_dev);

	return ret;
err_probe:
	android_destroy_device(android_dev);
err_dev:
	list_del(&android_dev->list_item);
	android_dev_count--;
	kfree(android_dev);
err_alloc:
	if (list_empty(&android_dev_list)) {
		class_destroy(android_class);
		android_class = NULL;
	}
	return ret;
}

static int android_remove(struct platform_device *pdev)
{
	struct android_dev *dev = NULL;
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	int usb_core_id = 0;

	if (pdata)
		usb_core_id = pdata->usb_core_id;

	
	list_for_each_entry(dev, &android_dev_list, list_item) {
		if (!dev->pdata)
			break; 
		if (dev->pdata->usb_core_id == usb_core_id)
			break;
	}

	if (dev) {
		android_destroy_device(dev);
		if (pdata && pdata->swfi_latency)
			pm_qos_remove_request(&dev->pm_qos_req_dma);
		list_del(&dev->list_item);
		android_dev_count--;
		kfree(dev);
		_android_dev = NULL;
	}

	if (list_empty(&android_dev_list)) {
		class_destroy(android_class);
		android_class = NULL;
		usb_composite_unregister(&android_usb_driver);
	}

	return 0;
}


static void android_shutdown(struct platform_device *pdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_composite_dev *cdev = dev->cdev;
	int ret;
	ret = usb_gadget_disconnect(cdev->gadget);
	printk(KERN_INFO "[USB] %s ret : %d\n",__func__,ret);
}

static const struct platform_device_id android_id_table[] __devinitconst = {
	{
		.name = "android_usb",
	},
	{
		.name = "android_usb_hsic",
	},
};
#if 0 
static struct of_device_id usb_android_dt_match[] = {
	{	.compatible = "qcom,android-usb",
	},
	{}
};
#endif

static struct platform_driver android_platform_driver = {
	.driver = {
		.name = "android_usb",
#if 0 
		.of_match_table = usb_android_dt_match,
#endif
	},
	.probe = android_probe,
	.remove = android_remove,
	.id_table = android_id_table,
	.shutdown = android_shutdown,
};

void android_broadcast_abnormal_usb_reset(void)
{

	char *envp[] = {"EVENT=ABNORMAL_USB_RESET", NULL };
	int ats = board_get_usb_ats();
	int ret;
	if (!ats)
		return;
	ret = kobject_uevent_env(&_android_dev->dev->kobj, KOBJ_CHANGE,envp);
	if (!ret)
		printk(KERN_INFO "[USB] broadcast abnormal usb reset\n");
	else
		printk(KERN_INFO "[USB] fail to broadcast abnormal usb reset\n");

}

static int __init init(void)
{
	int ret;

	connect2pc = false;

	
	composite_driver.setup = android_setup;
	composite_driver.disconnect = android_disconnect;
	composite_driver.suspend = android_suspend;
	composite_driver.resume = android_resume;
	composite_driver.mute_disconnect = android_mute_disconnect;
	composite_driver.broadcast_abnormal_usb_reset = android_broadcast_abnormal_usb_reset;

	INIT_LIST_HEAD(&android_dev_list);
	INIT_WORK(&switch_adb_work, do_switch_adb_work);
	android_dev_count = 0;

	ret = platform_driver_register(&android_platform_driver);
	if (ret) {
		pr_err("%s(): Failed to register android"
				 "platform driver\n", __func__);
	}

	return ret;
}
module_init(init);

static void __exit cleanup(void)
{
	platform_driver_unregister(&android_platform_driver);
}
module_exit(cleanup);
