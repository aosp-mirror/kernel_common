/*
 * Platform data for Android USB
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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
#ifndef	__LINUX_USB_ANDROID_H
#define	__LINUX_USB_ANDROID_H

#include <linux/usb/composite.h>

#define MAX_STREAMING_FUNCS 3
#define FUNC_NAME_LEN 10
struct android_usb_platform_data {
	int (*update_pid_and_serial_num)(uint32_t, const char *);
	u32 swfi_latency;
	u8 usb_core_id;
	bool cdrom;

	
	__u16 vendor_id;
	
	__u16 product_id;

	char *product_name;
	char *manufacturer_name;
	char *serial_number;

	
	int usb_id_pin_gpio;
	const char *fserial_init_string;

	const char *usb_rmnet_interface;
	const char *usb_diag_interface;

	unsigned char diag_init:1;
	unsigned char modem_init:1;
	unsigned char rmnet_init:1;
	unsigned char reserved:5;

	int (*match)(int product_id, int intrsharing);

	

	
	int nluns;
	int cdrom_lun;
	int cdrom_cttype;
	bool internal_ums;
	char streaming_func[MAX_STREAMING_FUNCS][FUNC_NAME_LEN];
	int  streaming_func_count;
    int vzw_unmount_cdrom;
	u32 uicc_nluns;
};

#ifndef CONFIG_TARGET_CORE
static inline int f_tcm_init(int (*connect_cb)(bool connect))
{
	return 0;
}
static inline void f_tcm_exit(void)
{
}
static inline int tcm_bind_config(struct usb_configuration *c)
{
	return -ENODEV;
}
#endif

#endif	
