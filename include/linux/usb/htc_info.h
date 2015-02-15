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

#ifndef __HTC_INFO__
#define __HTC_INFO__

#include <mach/board.h>
struct usb_info {
	int *phy_init_seq;
	void (*phy_reset)(void);
	void (*hw_reset)(bool en);
	void (*usb_uart_switch)(int);
	void (*serial_debug_gpios)(int);
	void (*usb_hub_enable)(bool);
	int (*china_ac_detect)(void);
	void (*disable_usb_charger)(void);
	void (*change_phy_voltage)(int);
	int (*ldo_init) (int init);
	int (*ldo_enable) (int enable);
	void (*usb_mhl_switch)(bool);

	
	int connect_type_ready;
	void (*usb_connected)(int);

	
	#if 0
	enum usb_connect_type connect_type;
	#endif
	struct delayed_work chg_stop;
};

extern ssize_t otg_show_usb_phy_setting(char *buf);
extern ssize_t otg_store_usb_phy_setting(const char *buf, size_t count);

extern int usb_get_connect_type(void);
extern int msm_usb_get_connect_type(void);
extern int android_switch_function(unsigned func);
extern int android_show_function(char *buf);
extern void android_set_serialno(char *serialno);
extern void android_force_reset(void);
extern int htc_usb_enable_function(char *name, int ebl);

extern void htc_mode_enable(int enable);
extern int check_htc_mode_status(void);
extern void android_switch_default(void);
extern void android_switch_htc_mode(void);

#ifdef err
#undef err
#endif
#ifdef warn
#undef warn
#endif
#ifdef info
#undef info
#endif

#define USB_ERR(fmt, args...) \
	printk(KERN_ERR "[USB:ERR] " fmt, ## args)
#define USB_WARNING(fmt, args...) \
	printk(KERN_WARNING "[USB] " fmt, ## args)
#define USB_INFO(fmt, args...) \
	printk(KERN_INFO "[USB] " fmt, ## args)
#define USB_DEBUG(fmt, args...) \
	printk(KERN_DEBUG "[USB] " fmt, ## args)

#define USBH_ERR(fmt, args...) \
	printk(KERN_ERR "[USBH:ERR] " fmt, ## args)
#define USBH_WARNING(fmt, args...) \
	printk(KERN_WARNING "[USBH] " fmt, ## args)
#define USBH_INFO(fmt, args...) \
	printk(KERN_INFO "[USBH] " fmt, ## args)
#define USBH_DEBUG(fmt, args...) \
	printk(KERN_DEBUG "[USBH] " fmt, ## args)
#if 0
#ifdef dev_err
#undef dev_err
#endif
#define dev_err(dev, fmt, args...) \
	printk(KERN_ERR "[USB] " pr_fmt(fmt), ## args)

#ifdef dev_dbg
#undef dev_dbg
#endif
#define dev_dbg(dev, fmt, args...) \
	printk(KERN_INFO "[USB] " pr_fmt(fmt), ## args)

#ifdef dev_info
#undef dev_info
#endif
#define dev_info(dev, fmt, args...) \
	printk(KERN_INFO "[USB] " pr_fmt(fmt), ## args)
#endif
#ifdef pr_debug
#undef pr_debug
#endif
#define pr_debug(fmt, args...) \
	printk(KERN_INFO "[USB] " pr_fmt(fmt), ## args)

#ifdef pr_err
#undef pr_err
#endif
#define pr_err(fmt, args...) \
	printk(KERN_ERR "[USB] " pr_fmt(fmt), ## args)

#ifdef pr_info
#undef pr_info
#endif
#define pr_info(fmt, args...) \
	printk(KERN_INFO "[USB] " pr_fmt(fmt), ## args)

#define ANDROID_USB_ENABLE_FUNC(dev, conf, func) 		\
	if (android_enable_function(dev, conf, func)) {		\
		pr_err("android_usb: Cannot enable %s", func);	\
	}

#endif 

