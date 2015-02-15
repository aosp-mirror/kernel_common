/*
 * Copyright (C) 2011 HTC, Inc.
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

#include <mach/board.h>
#include <linux/gpio.h>
#include <linux/usb/htc_usb.h>
#include <mach/devices_cmdline.h>
#include <mach/devices_dtb.h>

int rom_stockui = 0;

enum {
	USB_FUNCTION_UMS = 0,
	USB_FUNCTION_ADB = 1,
	USB_FUNCTION_RNDIS,
	USB_FUNCTION_DIAG,
	USB_FUNCTION_SERIAL,
	USB_FUNCTION_PROJECTOR,
	USB_FUNCTION_FSYNC,
	USB_FUNCTION_MTP,
	USB_FUNCTION_MODEM, 
	USB_FUNCTION_ECM,
	USB_FUNCTION_ACM,
	USB_FUNCTION_DIAG_MDM, 
	USB_FUNCTION_RMNET,
	USB_FUNCTION_ACCESSORY,
	USB_FUNCTION_MODEM_MDM, 
	USB_FUNCTION_NCM,
	USB_FUNCTION_PROJECTOR2,
	USB_FUNCTION_AUDIO_SOURCE, 
	USB_FUNCTION_PTP, 
	USB_FUNCTION_AUTOBOT = 30,
	USB_FUNCTION_RNDIS_IPT = 31,
};

struct usb_string_node{
	u32 usb_function_flag;
	char *name;
};

static struct usb_string_node usb_string_array[] = {
	{
		.usb_function_flag = 1 << USB_FUNCTION_UMS,
		.name = "mass_storage",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_ADB,
		.name = "adb",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_RNDIS,
		.name = "rndis",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_DIAG,
		.name = "diag",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_SERIAL,
		.name = "serial",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_PROJECTOR,
		.name = "projector",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_MODEM,
		.name = "modem",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_ECM,
		.name = "cdc_ethernet",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_NCM,
		.name = "cdc_network",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_ACM,
		.name = "acm",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_DIAG_MDM,
		.name = "diag_mdm",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_RMNET,
		.name = "rmnet",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_ACCESSORY,
		.name = "accessory",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_MODEM_MDM,
		.name = "modem_mdm",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_MTP,
		.name = "mtp",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_PROJECTOR2,
		.name = "projector2",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_AUDIO_SOURCE,
		.name = "audio_source",
	},
	{
		.usb_function_flag = 1 << USB_FUNCTION_PTP,
		.name = "ptp",
	},

};

static int use_mfg_serialno;
static int manual_serialno_flag = 0;
static char mfg_df_serialno[16];
static char mfg_whiteline_serialno[] = "000000000003";
static int intrsharing;

#define PID_RNDIS		0x0ffe
#define PID_ECM			0x0ff8
#define PID_NCM			0x0f93
#define PID_ACM			0x0ff4
#define PID_MTPUMS		0x0f91
#define PID_MTPUMS_STOCKUI	0x0f26
#define PID_STOCKUI	0x060d
#define PID_UL 0x061A

#define PDATA_NOT_DEFINED(field) \
	printk(KERN_INFO "[USB] %s: %s isnt defined\n",	__func__, field);

void android_force_reset(void)
{
	struct android_dev *dev = _android_dev;

	
	mutex_lock(&function_bind_sem);
	if (dev) {
		android_disable(dev);
		dev->enabled = false;

		msleep(500);

		android_enable(dev);
		dev->enabled = true;
	} else
		pr_info("force reset fails: no device.\n");
	mutex_unlock(&function_bind_sem);
}
#if 0

static bool isFunctionDisabled(struct android_usb_function *function)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function_holder *f_holder;
	struct android_configuration *conf;
	struct list_head *list;

	if (dev->configs_num == 0)
		return 0;

	
	conf = list_entry(dev->configs.next, struct android_configuration, list_item);

	list = &conf->enabled_functions;

	list_for_each_entry(f_holder, list, enabled_list) {
		if (!strcmp(function->name, f_holder->f->name))
			return false;
	}
	return true;
}

static int product_has_function(struct android_usb_product *p,
		struct android_usb_function *f)
{
	char **functions = p->functions;
	int count = p->num_functions;
	const char *name = f->name;
	int i;

	for (i = 0; i < count; i++) {
		if (!strncmp(name, functions[i], strlen(name)))
			return 1;
	}
	return 0;
}

static int product_matches_functions(struct android_usb_product *p,
	struct list_head *list)
{
	int count = 0;

	struct android_usb_function_holder *f_holder;
	list_for_each_entry(f_holder, list, enabled_list) {
		count++;
		if (product_has_function(p, f_holder->f) == isFunctionDisabled(f_holder->f))
			return 0;
	}

	if (count == p->num_functions)
		return 1;
	else
		return 0;
}
static __maybe_unused int get_product_id(struct android_dev *dev, struct list_head *list)
{
	struct android_usb_product *p = usb_products;
	int count = ARRAY_SIZE(usb_products);
	int i;

	if (p) {
		for (i = 0; i < count; i++, p++) {
			if (product_matches_functions(p, list))
				return p->product_id;
		}
	}
	
	if (dev->pdata)
		return dev->pdata->product_id;
	else {
		PDATA_NOT_DEFINED("product id");
		return 0;
	}
}
#endif

static unsigned int Name2Flag(char *name)
{
	int x, length = ARRAY_SIZE(usb_string_array);
	for (x = 0; x < length; x++) {
		if (strcmp(usb_string_array[x].name, name) == 0)
			return usb_string_array[x].usb_function_flag;
	}
	printk(KERN_INFO "[USB]Name2Flag fail to match string %s\n",name);
	return 0;
}

static unsigned int get_string_array_value(char **functions_string, int function_num)
{
	int i,val = 0;
	for (i = 0; i < function_num;i++) {
		val |= Name2Flag(functions_string[i]);
	}
	return val;
}

static unsigned int get_function_holder_value(struct list_head *list)
{
	struct android_usb_function_holder *f_holder;
	int val = 0;
	list_for_each_entry(f_holder, list, enabled_list)
		val |= Name2Flag(f_holder->f->name);
	return val;
}

static struct android_usb_product *get_product(struct android_dev *dev, struct list_head *list)
{
	struct android_usb_product *p = usb_products;
	int count = ARRAY_SIZE(usb_products);
	int i,list_val,product_array_val;
	
	list_val = get_function_holder_value(list);
	for (i = 0; i < count; i++, p++) {
		product_array_val = get_string_array_value(p->functions,p->num_functions);
		if (list_val == product_array_val)
			return p;
	}
	return NULL;
}

static unsigned int htc_usb_get_func_combine_value(void)
{
	struct android_dev *dev = _android_dev;
	struct android_configuration *conf;

	if (dev->configs_num == 0)
		return 0;

	
	conf = list_entry(dev->configs.next, struct android_configuration, list_item);
	return get_function_holder_value(&conf->enabled_functions);
}

int htc_usb_enable_function(char *name, int ebl)
{
	unsigned val,enable_val;

	mutex_lock(&function_bind_sem);

	val = htc_usb_get_func_combine_value();
	enable_val = Name2Flag(name);

	if(val & enable_val) {
		if(ebl) {
			pr_info("%s: '%s' is already enabled\n", __func__, name);
			mutex_unlock(&function_bind_sem);
			return 0;
		}
		val &= ~enable_val;
	} else {
		if(!ebl) {
			pr_info("%s: '%s' is already disabled\n", __func__, name);
			mutex_unlock(&function_bind_sem);
			return 0;
		}
		val |= enable_val;
	}
	mutex_unlock(&function_bind_sem);
	return android_switch_function(val);
}


int android_show_function(char *buf)
{
	unsigned length = 0;
	struct android_dev *dev = _android_dev;
	struct android_configuration *conf;
	
	struct android_usb_function_holder *f_holder;
	char *ebl_str[2] = {"disable", "enable"};
	char *p;
	int i;

	if (dev->configs_num == 0)
		return length;

	
	conf = list_entry(dev->configs.next, struct android_configuration, list_item);

	for (i = 0; dev->functions[i] != NULL; i++) {
		p = ebl_str[0];

		list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list) {
			if (!strcmp(dev->functions[i]->name, f_holder->f->name)) {
				p = ebl_str[1];
				break;
			}
		}

		length += sprintf(buf + length, "%s:%s\n",
				dev->functions[i]->name, p);

	}
	return length;
}

int android_usb_function_holder_list_add_tail(struct android_usb_function *f, struct list_head *list,struct android_dev *dev)
{
	struct android_usb_function_holder *f_holder;
	f_holder = kzalloc(sizeof(*f_holder), GFP_KERNEL);
	if (!f_holder) {
		pr_err("Failed to alloc f_holder\n");
		return -ENOMEM;
	}
	f->android_dev = dev;
	f_holder->f = f;
	list_add_tail(&f_holder->enabled_list, list);
	return 0;
}

static bool is_mtp_enable(void)
{
	unsigned val;

	mutex_lock(&function_bind_sem);
	val = htc_usb_get_func_combine_value();
	mutex_unlock(&function_bind_sem);

	if (val & (1 << USB_FUNCTION_MTP))
		return true;
	else
		return false;
}

extern int rom_stockui;

int android_switch_function(unsigned func)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_function **functions = dev->functions;
	struct android_usb_function *f;
	
	struct android_usb_function_holder *f_holder;
	struct android_usb_function *fadb = NULL;
	struct android_usb_function *fums = NULL;
	struct android_configuration *conf;
	struct android_usb_product *product;
	int product_id = 0, vendor_id = 0;
	unsigned val, comm_class = 0;

	mutex_lock(&function_bind_sem);

	
	if (dev->enabled != true) {
		pr_info("%s: USB driver is not initialize\n", __func__);
		mutex_unlock(&function_bind_sem);
		return 0;
	}

	
	if (board_mfg_mode() == 2) {
		printk("[USB] recovery mode only accept UMS or ADB + UMS combination\n");
		func &= (1 << USB_FUNCTION_UMS) | (1 << USB_FUNCTION_ADB);
	}

	val = htc_usb_get_func_combine_value();

	pr_info(" %u, before %u\n", func, val);

	if (func == val && rom_stockui != 1) {
		pr_info("%s: SKIP due the function is the same ,%u\n" , __func__, func);
		mutex_unlock(&function_bind_sem);
		return 0;
	}
	
	android_disable(dev);
	list_for_each_entry(conf, &dev->configs, list_item)
		list_for_each_entry(f_holder, &conf->enabled_functions,
				enabled_list) {
			if (f_holder->f->disable)
				f_holder->f->disable(f_holder->f);
		}
	dev->enabled = false;
	
	list_for_each_entry(conf, &dev->configs, list_item) {
		while (conf->enabled_functions.next != &conf->enabled_functions) {
			f_holder = list_entry(conf->enabled_functions.next, typeof(*f_holder), enabled_list);
			f_holder->f->android_dev = NULL;
			list_del(&f_holder->enabled_list);
			kfree(f_holder);
		}
		INIT_LIST_HEAD(&conf->enabled_functions);
	}

	
	conf = list_entry(dev->configs.next, struct android_configuration, list_item);

	while ((f = *functions++)) {
		if ((func & (1 << USB_FUNCTION_UMS)) && !strcmp(f->name, "mass_storage")) {
			if (func == ((1 << USB_FUNCTION_UMS) | (1 << USB_FUNCTION_ADB)))
				fums = f;
			else {
				if (android_usb_function_holder_list_add_tail(f, &conf->enabled_functions, dev))
					pr_err("android_switch_function: Cannot add %s\n", f->name);
			}

		} else if ((func & (1 << USB_FUNCTION_ADB)) && !strcmp(f->name, "adb")) {
			if (func == ((1 << USB_FUNCTION_UMS) | (1 << USB_FUNCTION_ADB)))
				fadb = f;
			else {
				if (android_usb_function_holder_list_add_tail(f, &conf->enabled_functions, dev))
					pr_err("android_switch_function: Cannot add %s\n", f->name);
			}

		} else if ((func & (1 << USB_FUNCTION_ECM)) && !strcmp(f->name, "cdc_ethernet")) {
			if (android_usb_function_holder_list_add_tail(f, &conf->enabled_functions, dev))
				pr_err("android_switch_function: Cannot add %s\n", f->name);
			comm_class = 1;

		} else if ((func & (1 << USB_FUNCTION_ACM)) && !strcmp(f->name, "acm")) {
			if (android_usb_function_holder_list_add_tail(f, &conf->enabled_functions, dev))
				pr_err("android_switch_function: Cannot add %s\n", f->name);

		} else if ((func & (1 << USB_FUNCTION_NCM)) && !strcmp(f->name, "cdc_network")) {
			if (android_usb_function_holder_list_add_tail(f, &conf->enabled_functions, dev))
				pr_err("android_switch_function: Cannot add %s\n", f->name);

		} else if ((func & (1 << USB_FUNCTION_RNDIS)) && !strcmp(f->name, "rndis")) {
			if (android_usb_function_holder_list_add_tail(f, &conf->enabled_functions, dev))
				pr_err("android_switch_function: Cannot add %s\n", f->name);
			intrsharing = !((func >> USB_FUNCTION_RNDIS_IPT) & 1);

		} else if ((func & (1 << USB_FUNCTION_DIAG)) && !strcmp(f->name, "diag")) {
			if (android_usb_function_holder_list_add_tail(f, &conf->enabled_functions, dev))
				pr_err("android_switch_function: Cannot add %s\n", f->name);
#ifdef CONFIG_USB_ANDROID_MDM9K_DIAG
			func |= 1 << USB_FUNCTION_DIAG_MDM;
#endif

		} else if ((func & (1 << USB_FUNCTION_MODEM)) && !strcmp(f->name, "modem")) {
			if (android_usb_function_holder_list_add_tail(f, &conf->enabled_functions, dev))
				pr_err("android_switch_function: Cannot add %s\n", f->name);
#ifdef CONFIG_USB_ANDROID_MDM9K_MODEM
			func |= 1 << USB_FUNCTION_MODEM_MDM;
#endif

		} else if ((func & (1 << USB_FUNCTION_SERIAL)) && !strcmp(f->name, "serial")) {
			if (android_usb_function_holder_list_add_tail(f, &conf->enabled_functions, dev))
				pr_err("android_switch_function: Cannot add %s\n", f->name);

		} else if ((func & (1 << USB_FUNCTION_MTP)) && !strcmp(f->name, "mtp")) {
			if (android_usb_function_holder_list_add_tail(f, &conf->enabled_functions, dev))
				pr_err("android_switch_function: Cannot add %s\n", f->name);

		} else if ((func & (1 << USB_FUNCTION_PTP)) && !strcmp(f->name, "ptp")) {
			if (android_usb_function_holder_list_add_tail(f, &conf->enabled_functions, dev))
				pr_err("android_switch_function: Cannot add %s\n", f->name);

		} else if ((func & (1 << USB_FUNCTION_ACCESSORY)) && !strcmp(f->name, "accessory")) {
			if (android_usb_function_holder_list_add_tail(f, &conf->enabled_functions, dev))
				pr_err("android_switch_function: Cannot add %s\n", f->name);

		} else if ((func & (1 << USB_FUNCTION_PROJECTOR)) && !strcmp(f->name, "projector")) {
			if (android_usb_function_holder_list_add_tail(f, &conf->enabled_functions, dev))
				pr_err("android_switch_function: Cannot add %s\n", f->name);

		} else if ((func & (1 << USB_FUNCTION_PROJECTOR2)) && !strcmp(f->name, "projector2")) {
			if (android_usb_function_holder_list_add_tail(f, &conf->enabled_functions, dev))
				pr_err("android_switch_function: Cannot add %s\n", f->name);

		}
#ifdef CONFIG_USB_ANDROID_MDM9K_DIAG
		else if ((func & (1 << USB_FUNCTION_DIAG_MDM)) && !strcmp(f->name, "diag_mdm")) {
			if (func & (1 << USB_FUNCTION_DIAG)) {
				if (android_usb_function_holder_list_add_tail(f, &conf->enabled_functions, dev))
					pr_err("android_switch_function: Cannot add %s\n", f->name);
			} else
				func &= ~(1 << USB_FUNCTION_DIAG_MDM);
		}
#endif
		else if ((func & (1 << USB_FUNCTION_RMNET)) && !strcmp(f->name, "rmnet")) {
			if (android_usb_function_holder_list_add_tail(f, &conf->enabled_functions, dev))
				pr_err("android_switch_function: Cannot add %s\n", f->name);

		}
#ifdef CONFIG_USB_ANDROID_MDM9K_MODEM
		else if ((func & (1 << USB_FUNCTION_MODEM_MDM)) && !strcmp(f->name, "modem_mdm")) {
			if (func & (1 << USB_FUNCTION_MODEM)) {
				if (android_usb_function_holder_list_add_tail(f, &conf->enabled_functions, dev))
					pr_err("android_switch_function: Cannot add %s\n", f->name);
			} else
				func &= ~(1 << USB_FUNCTION_MODEM_MDM);
		}
#endif
		else if ((func & (1 << USB_FUNCTION_AUDIO_SOURCE)) && !strcmp(f->name, "audio_source")) {
			if (android_usb_function_holder_list_add_tail(f, &conf->enabled_functions, dev))
				pr_err("android_switch_function: Cannot add %s\n", f->name);

		}
	}
	
	if (func == ((1 << USB_FUNCTION_UMS) | (1 << USB_FUNCTION_ADB))) {
		if (fums) {
			if (android_usb_function_holder_list_add_tail(fums, &conf->enabled_functions, dev))
				pr_err("android_switch_function: Cannot add %s\n", fums->name);
		}
		if (fadb) {
			if (android_usb_function_holder_list_add_tail(fadb, &conf->enabled_functions, dev))
				pr_err("android_switch_function: Cannot add %s\n", fadb->name);
		}
	}

	list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list)
		pr_debug("# %s\n", f_holder->f->name);

	product = get_product(dev, &conf->enabled_functions);

	if (product && dev->pdata) {
		vendor_id = product->vendor_id ? product->vendor_id : dev->pdata->vendor_id;
		product_id = product->product_id;
	} else if (dev->pdata) {
		vendor_id = dev->pdata->vendor_id;
		product_id =  dev->pdata->product_id;
	} else
		PDATA_NOT_DEFINED("vendor/product id");


	if (dev->pdata && dev->pdata->match)
		product_id = dev->pdata->match(product_id, intrsharing);

	if (rom_stockui && (product_id == PID_MTPUMS)) {
		product_id = PID_MTPUMS_STOCKUI;
		pr_info("%s: stockUI ROM for mtp+ums\n", __func__);
	} else if (rom_stockui && (product_id == PID_UL)) {
		product_id = PID_STOCKUI;
		pr_info("%s: stockUI ROM for default function\n", __func__);
	}
	pr_info("%s: rom_stockui=%d\n", __func__, rom_stockui);

	pr_info("%s: vendor_id=0x%x, product_id=0x%x\n",
			__func__, vendor_id, product_id);

	device_desc.idVendor = __constant_cpu_to_le16(vendor_id);
	device_desc.idProduct = __constant_cpu_to_le16(product_id);

	dev->cdev->desc.idVendor = device_desc.idVendor;
	dev->cdev->desc.idProduct = device_desc.idProduct;

	if (product_id == PID_RNDIS || product_id == PID_ACM || comm_class)
		dev->cdev->desc.bDeviceClass = USB_CLASS_COMM;
	else
		dev->cdev->desc.bDeviceClass = USB_CLASS_PER_INTERFACE;

	device_desc.bDeviceClass = dev->cdev->desc.bDeviceClass;


	list_for_each_entry(conf, &dev->configs, list_item)
		list_for_each_entry(f_holder, &conf->enabled_functions,
				enabled_list) {
			if (f_holder->f->enable)
				f_holder->f->enable(f_holder->f);
		}
	mdelay(200);
	android_enable(dev);
	dev->enabled = true;

	mutex_unlock(&function_bind_sem);
	return 0;
}


struct work_struct	switch_adb_work;
static char enable_adb;
static void do_switch_adb_work(struct work_struct *work)
{
	int	call_us_ret = -1;
	char *envp[] = {
		"HOME=/",
		"PATH=/sbin:/system/sbin:/system/bin:/system/xbin",
		NULL,
	};
	char *exec_path[2] = {"/system/bin/stop", "/system/bin/start" };
	char *argv_stop[] = { exec_path[0], "adbd", NULL, };
	char *argv_start[] = { exec_path[1], "adbd", NULL, };

	if (enable_adb) {
		call_us_ret = call_usermodehelper(exec_path[1],
			argv_start, envp, UMH_WAIT_PROC);
	} else {
		call_us_ret = call_usermodehelper(exec_path[0],
			argv_stop, envp, UMH_WAIT_PROC);
	}
	htc_usb_enable_function("adb", enable_adb);
}
static int android_switch_setup(struct usb_gadget *gadget,
		const struct usb_ctrlrequest *c)
{
	int value = -EOPNOTSUPP;
	u16 wIndex = le16_to_cpu(c->wIndex);
	u16 wValue = le16_to_cpu(c->wValue);
#if 0
	u16 wLength = le16_to_cpu(c->wLength);
#endif
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	struct usb_request *req = cdev->req;
	

	switch (c->bRequestType & USB_TYPE_MASK) {
	case USB_TYPE_VENDOR:

		switch (c->bRequest) {
		case USB_REQ_HTC_FUNCTION:

			switch (wValue) {
			case USB_WVAL_ADB:

				value = 0;
				req->zero = 0;
				req->length = value;
				if (usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC))
					printk(KERN_ERR "ep0 in queue failed\n");

				if (wIndex == 1) {
					enable_adb = 1;
					schedule_work(&switch_adb_work);
				} else if (wIndex == 0) {
					enable_adb = 0;
					schedule_work(&switch_adb_work);
				}
				break;

			default:
				break;
			}

			break;

		default:
			break;
		}
		break;
	default:
		break;
	}

	return value;
}


static void setup_usb_denied(int htc_mode)
{
	if (htc_mode)
		_android_dev->autobot_mode = 1;
	else
		_android_dev->autobot_mode = 0;
}


static int usb_autobot_mode(void)
{
	if (_android_dev->autobot_mode)
		return 1;
	else
		return 0;
}

void android_switch_default(void)
{
	unsigned val;

	mutex_lock(&function_bind_sem);
	val = htc_usb_get_func_combine_value();
	mutex_unlock(&function_bind_sem);

	if (val & (1 << USB_FUNCTION_ADB))
		android_switch_function(
				(1 << USB_FUNCTION_MTP) |
				(1 << USB_FUNCTION_ADB) |
				(1 << USB_FUNCTION_UMS));
	else
		android_switch_function(
				(1 << USB_FUNCTION_MTP) |
				(1 << USB_FUNCTION_UMS));
}

void android_switch_htc_mode(void)
{
	android_switch_function((1 << USB_FUNCTION_ADB) |
		(1 << USB_FUNCTION_PROJECTOR) |
		(1 << USB_FUNCTION_SERIAL) |
		(1 << USB_FUNCTION_UMS) |
		(1 << USB_FUNCTION_AUTOBOT));
}

void android_set_serialno(char *serialno)
{
	strings_dev[STRING_SERIAL_IDX].s = serialno;
}

void init_mfg_serialno(void)
{
	char *serialno = "000000000000";

	use_mfg_serialno = (board_mfg_mode() == 1) ? 1 : 0;
	strncpy(mfg_df_serialno, serialno, strlen(serialno));
}
static ssize_t show_usb_ac_cable_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;

#ifdef CONFIG_USB_DWC3
	length = sprintf(buf, "%d",usb_get_connect_type());
#else
	length = sprintf(buf, "%d",msm_usb_get_connect_type());
#endif
	return length;
}

static int cdrom_unmount;
static ssize_t show_is_cdrom(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d\n", cdrom_unmount);
	return length;
}

static ssize_t store_is_cdrom(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct usb_composite_dev *cdev = _android_dev->cdev;
	int value;
	sscanf(buf, "%d", &value);
	cdrom_unmount = value;

	if(value == 1) {
		printk(KERN_INFO "Trigger unmount uevent after 30 seconds\n");
		cancel_delayed_work(&cdev->cdusbcmd_vzw_unmount_work);
		cdev->unmount_cdrom_mask = 1 << 3 | 1 << 4;
		schedule_delayed_work(&cdev->cdusbcmd_vzw_unmount_work,30 * HZ);
	}

	return count;
}

static int usb_disable;
static ssize_t show_usb_cable_connect(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;
	struct android_dev *and_dev = _android_dev;

	length = sprintf(buf, "%d",(and_dev->connected == 1) && !usb_disable ? 1 : 0);
	return length;
}

static ssize_t show_usb_function_switch(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return android_show_function(buf);
}

static ssize_t store_usb_function_switch(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned u;
	ssize_t  ret;

	ret = strict_strtoul(buf, 10, (unsigned long *)&u);
	if (ret < 0) {
		USB_ERR("%s: %d\n", __func__, ret);
		return 0;
	}

	ret = android_switch_function(u);

	if (ret == 0)
		return count;
	else
		return 0;
}

int cable_get_usb_id_level(void);

static ssize_t show_USB_ID_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;
	int value;

	value = cable_get_usb_id_level();

	length = sprintf(buf, "%d", value);
	return length;
}

static ssize_t show_usb_serial_number(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;
	struct android_usb_platform_data *pdata = dev->platform_data;

	if (pdata)
		length = sprintf(buf, "%s", pdata->serial_number);
	else {
		PDATA_NOT_DEFINED("serial number");
		length = sprintf(buf, "%s", "000000000000");
	}
	return length;
}

static ssize_t store_usb_serial_number(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct android_usb_platform_data *pdata = dev->platform_data;
	char *serialno = "000000000000";
	manual_serialno_flag = 1;
	if (buf[0] == '0' || buf[0] == '1') {
		memset(mfg_df_serialno, 0x0, sizeof(mfg_df_serialno));
		if (buf[0] == '0') {
			strncpy(mfg_df_serialno, serialno, strlen(serialno));
			use_mfg_serialno = 1;
			android_set_serialno(mfg_df_serialno);
		} else if (pdata) {
			strncpy(mfg_df_serialno, pdata->serial_number,
					strlen(pdata->serial_number));
			use_mfg_serialno = 0;
			android_set_serialno(pdata->serial_number);
		} else {
			PDATA_NOT_DEFINED("serial number");
			return count;
		}
		
		android_force_reset();
	}

	return count;
}

void mfg_check_white_line(void)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_platform_data *pdata = dev->pdata;
	int mode,id_level;
	mode = board_mfg_mode();
	id_level = cable_get_usb_id_level();
       if (manual_serialno_flag)
                return;
	if (board_mfg_mode() == MFG_MODE_FACTORY2 && cable_get_usb_id_level() == 0)
		android_set_serialno(mfg_whiteline_serialno);
	else
		android_set_serialno(pdata->serial_number);
}

static ssize_t show_dummy_usb_serial_number(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length = 0;
	struct android_usb_platform_data *pdata = dev->platform_data;

	if (use_mfg_serialno)
		length = sprintf(buf, "%s", mfg_df_serialno); 
	else if (pdata)
		length = sprintf(buf, "%s", pdata->serial_number); 
	else
		PDATA_NOT_DEFINED("serial number");
	return length;
}

static ssize_t store_dummy_usb_serial_number(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int data_buff_size = (sizeof(mfg_df_serialno) > strlen(buf))?
		strlen(buf):sizeof(mfg_df_serialno);
	int loop_i;
	manual_serialno_flag = 1;
	
	if (data_buff_size == 16)
		data_buff_size--;

	for (loop_i = 0; loop_i < data_buff_size; loop_i++)     {
		if (buf[loop_i] >= 0x30 && buf[loop_i] <= 0x39) 
			continue;
		else if (buf[loop_i] >= 0x41 && buf[loop_i] <= 0x5A) 
			continue;
		if (buf[loop_i] == 0x0A) 
			continue;
		else {
			printk(KERN_INFO "%s(): get invaild char (0x%2.2X)\n",
					__func__, buf[loop_i]);
			return -EINVAL;
		}
	}

	use_mfg_serialno = 1;
	memset(mfg_df_serialno, 0x0, sizeof(mfg_df_serialno));
	strncpy(mfg_df_serialno, buf, data_buff_size);
	android_set_serialno(mfg_df_serialno);
	
	android_force_reset();

	return count;
}

static ssize_t
show_usb_car_kit_enable(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	unsigned length;
	int value = 0;
#ifdef CONFIG_CABLE_DETECT_ACCESSORY
#include <mach/cable_detect.h>
	value = (cable_get_accessory_type() == DOCK_STATE_UNDOCKED) ? 0 : 1;
	printk(KERN_INFO "USB_car_kit_enable %d\n", cable_get_accessory_type());
#else
	value = 0;
	printk(KERN_INFO "USB_car_kit_enable: CABLE_DETECT_ACCESSORY was not defined\n");
#endif

	length = sprintf(buf, "%d", value);
	return length;
}

#if 0
static ssize_t show_usb_phy_setting(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return otg_show_usb_phy_setting(buf);
}
static ssize_t store_usb_phy_setting(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	return otg_store_usb_phy_setting(buf, count);
}
#endif

static ssize_t show_usb_disable_setting(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d\n", usb_disable);
	return length;
}

void htc_dwc3_disable_usb(int disable_usb);
void msm_otg_set_disable_usb(int disable_usb_function);
static ssize_t store_usb_disable_setting(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int disable_usb_function;
	ssize_t  ret;

	ret = kstrtouint(buf, 2, &disable_usb_function);
	if (ret < 0) {
		USB_ERR("%s: %d\n", __func__, ret);
		return count;
	}
	printk(KERN_INFO "USB_disable set %d\n", disable_usb_function);
	usb_disable = disable_usb_function;
#ifdef CONFIG_USB_DWC3_MSM
	htc_dwc3_disable_usb(disable_usb_function);
#else
	msm_otg_set_disable_usb(disable_usb_function);
#endif

	return count;
}

void dwc3_otg_set_id_state(int id);
static ssize_t store_usb_host_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned u, enable;
	ssize_t  ret;

	ret = strict_strtoul(buf, 10, (unsigned long *)&u);
	if (ret < 0) {
		USB_ERR("%s: %d\n", __func__, ret);
		return count;
	}

	enable = u ? 1 : 0;
#ifdef CONFIG_USB_DWC3_MSM
	dwc3_otg_set_id_state(!enable);
#endif

	USB_INFO("%s USB host\n", enable ? "Enable" : "Disable");

	return count;
}
static DEVICE_ATTR(host_mode, 0220,
		NULL, store_usb_host_mode);

static ssize_t show_is_usb_denied(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;
	int deny = 0;

	if (usb_autobot_mode()) {
		deny = 1;;
	}

	length = sprintf(buf, "%d\n", deny);
	USB_INFO("%s: %s\n", __func__, buf);
	return length;
}

static ssize_t show_os_type(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d\n", os_type);
	USB_INFO("%s: %s\n", __func__, buf);
	return length;
}
static ssize_t show_ats(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length;

	length = sprintf(buf, "%d\n", board_get_usb_ats());
	USB_INFO("%s: %s\n", __func__, buf);
	return length;
}

static ssize_t store_ats(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	static int type;

	sscanf(buf, "%d ", &type);
	board_set_usb_ats(type);
	return count;
}

static DEVICE_ATTR(usb_ac_cable_status, 0444, show_usb_ac_cable_status, NULL);
static DEVICE_ATTR(usb_cable_connect, 0444, show_usb_cable_connect, NULL);
static DEVICE_ATTR(usb_function_switch, 0664,
		show_usb_function_switch, store_usb_function_switch);
static DEVICE_ATTR(USB_ID_status, 0444, show_USB_ID_status, NULL);
static DEVICE_ATTR(usb_serial_number, 0644,
		show_usb_serial_number, store_usb_serial_number);
static DEVICE_ATTR(dummy_usb_serial_number, 0644,
		show_dummy_usb_serial_number, store_dummy_usb_serial_number);
static DEVICE_ATTR(usb_car_kit_enable, 0444, show_usb_car_kit_enable, NULL);
#if 0
static DEVICE_ATTR(usb_phy_setting, 0664,
		show_usb_phy_setting, store_usb_phy_setting);
#endif
static DEVICE_ATTR(usb_disable, 0664,show_usb_disable_setting, store_usb_disable_setting);
static DEVICE_ATTR(usb_denied, 0444, show_is_usb_denied, NULL);
static DEVICE_ATTR(os_type, 0444, show_os_type, NULL);
static DEVICE_ATTR(ats, 0664, show_ats, store_ats);
static DEVICE_ATTR(cdrom_unmount, 0644, show_is_cdrom, store_is_cdrom);

static __maybe_unused struct attribute *android_htc_usb_attributes[] = {
	&dev_attr_usb_ac_cable_status.attr,
	&dev_attr_usb_cable_connect.attr,
	&dev_attr_usb_function_switch.attr,
	&dev_attr_USB_ID_status.attr, 
	&dev_attr_usb_serial_number.attr, 
	&dev_attr_dummy_usb_serial_number.attr, 
	&dev_attr_usb_car_kit_enable.attr,
	#if 0
	&dev_attr_usb_phy_setting.attr,
	#endif
	&dev_attr_host_mode.attr,
	&dev_attr_usb_disable.attr,
	&dev_attr_usb_denied.attr,
	&dev_attr_os_type.attr,
	&dev_attr_ats.attr,
	&dev_attr_cdrom_unmount.attr,
	NULL
};

static __maybe_unused const struct attribute_group android_usb_attr_group = {
	.attrs = android_htc_usb_attributes,
};

static __maybe_unused int alloc_platform_data(struct android_dev *dev, struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata;
	struct device_node *node = dev->pdev->dev.of_node;
	__u32 product, vendor;

	pdev->dev.platform_data = pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);

	if (!pdata) {
		pr_err("%s(): Failed to alloc memory for pdata\n",
				__func__);
		return -ENOMEM;
	}

	of_property_read_u32(node, "htc,vendor", &vendor);
	of_property_read_u32(node, "htc,product", &product);
	of_property_read_string(node, "htc,fserial", &pdata->fserial_init_string);
	of_property_read_string(node, "htc,rmnet_interface", &pdata->usb_rmnet_interface);
	of_property_read_string(node, "htc,diag_interface", &pdata->usb_diag_interface);

	pdata->vendor_id = __constant_cpu_to_le16(vendor);
	pdata->product_id = __constant_cpu_to_le16(product);

	pr_info("%s: vendor: %04x\n", __func__, pdata->vendor_id);
	pr_info("%s: product: %04x\n", __func__, pdata->product_id);
	pr_info("%s: fserial: %s\n", __func__, pdata->fserial_init_string);
	pr_info("%s: rmnet_interface: %s\n", __func__, pdata->usb_rmnet_interface);
	pr_info("%s: diag_interface: %s\n", __func__, pdata->usb_diag_interface);

	dev->pdata = pdata;

	return 0;
}

static void setup_vendor_info(struct android_dev *dev) {
	struct android_configuration *conf;
	struct usb_composite_dev *cdev = dev->cdev;
	
#if 0
	struct android_usb_function *f;
#endif
	struct android_usb_function_holder *f_holder;
	struct android_usb_product *product;
	int product_id = 0, vendor_id = 0;

	
	strlcpy(product_string, HTC_PRODUCT_STRING, sizeof(product_string) - 1);
	strlcpy(manufacturer_string, HTC_COMPANY_STRING, sizeof(manufacturer_string) - 1);
	strlcpy(serial_string, "123456789012", sizeof(serial_string) - 1);

	
	if (dev->pdata && dev->pdata->product_name)
		strlcpy(product_string, dev->pdata->product_name,
			sizeof(product_string) - 1);
	if (dev->pdata && dev->pdata->manufacturer_name)
		strlcpy(manufacturer_string, dev->pdata->manufacturer_name,
		sizeof(manufacturer_string) - 1);
	if (dev->pdata && dev->pdata->serial_number)
		strlcpy(serial_string, dev->pdata->serial_number,
			sizeof(serial_string) - 1);


	if (sysfs_create_group(&dev->pdev->dev.kobj, &android_usb_attr_group))
		pr_err("%s: fail to create sysfs\n", __func__);

	if (dev->pdata) {
		if (dev->pdata->product_id)
			usb_products[0].product_id = dev->pdata->product_id;

		if (dev->pdata->usb_diag_interface) {
			strlcpy(diag_clients, dev->pdata->usb_diag_interface, sizeof(diag_clients));
		}

		if (dev->pdata->usb_rmnet_interface)
			strlcpy(rmnet_transports, dev->pdata->usb_rmnet_interface, sizeof(rmnet_transports));
	}
	_android_dev = dev;
	dev->enabled = false;


	
	list_for_each_entry(conf, &dev->configs, list_item) {
		list_for_each_entry(f_holder, &conf->enabled_functions, enabled_list)
			f_holder->f->android_dev = NULL;
		INIT_LIST_HEAD(&conf->enabled_functions);
		free_android_config(dev, conf);
	}
	
	conf = alloc_android_config(dev);

	if (get_radio_flag() & BIT(17)) {
		ANDROID_USB_ENABLE_FUNC(dev, conf, "mtp");
		ANDROID_USB_ENABLE_FUNC(dev, conf, "mass_storage");
		ANDROID_USB_ENABLE_FUNC(dev, conf, "diag");
		ANDROID_USB_ENABLE_FUNC(dev, conf, "modem");
		ANDROID_USB_ENABLE_FUNC(dev, conf, "rmnet");
	} else if (board_mfg_mode() == 2) {
		ANDROID_USB_ENABLE_FUNC(dev, conf, "mass_storage");
	} else {
		if (!rom_stockui) {
			ANDROID_USB_ENABLE_FUNC(dev, conf, "mtp");
			ANDROID_USB_ENABLE_FUNC(dev, conf, "mass_storage");
		}
	}

	product = get_product(dev, &conf->enabled_functions);
	if (product && dev->pdata) {
		vendor_id = product->vendor_id ? product->vendor_id : dev->pdata->vendor_id;
		product_id = product->product_id;
	} else if (dev->pdata) {
		vendor_id = dev->pdata->vendor_id;
		product_id =  dev->pdata->product_id;
	} else
		PDATA_NOT_DEFINED("vendor/product id");

	if (rom_stockui && (product_id == PID_MTPUMS)) {
		product_id = PID_MTPUMS_STOCKUI;
		pr_info("%s: stockUI ROM\n", __func__);
	} else if (rom_stockui && (product_id == PID_UL)) {
		product_id = PID_STOCKUI;
		pr_info("%s: stockUI ROM for default function\n", __func__);
	}

	pr_info("%s: rom_stockui=%d\n", __func__, rom_stockui);

	pr_info("%s: vendor_id=0x%x, product_id=0x%x\n", __func__, vendor_id, product_id);

	device_desc.idVendor = __constant_cpu_to_le16(vendor_id);
	device_desc.idProduct = __constant_cpu_to_le16(product_id);

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
		}
	android_enable(dev);
	dev->enabled = true;
}

