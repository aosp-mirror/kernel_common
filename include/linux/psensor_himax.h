/* include/linux/cm3629.h
 *
 * Copyright (C) 2010 HTC, Inc.
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

#ifndef __LINUX_PROXIMITY_H
#define __LINUX_PROXIMITY_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define CAPELLA_CM3602_IOCTL_MAGIC 'c'
#define CAPELLA_CM3602_IOCTL_GET_ENABLED \
		_IOR(CAPELLA_CM3602_IOCTL_MAGIC, 1, int *)
#define CAPELLA_CM3602_IOCTL_ENABLE \
		_IOW(CAPELLA_CM3602_IOCTL_MAGIC, 2, int *)

#ifdef __KERNEL__
#define CAPELLA_CM3602 "capella_cm3602"
#define LS_PWR_ON					(1 << 0)
#define PS_PWR_ON					(1 << 1)


struct psensor_platform_data {
	int intr;
	uint32_t irq_gpio_flags;
};

extern void touch_report_psensor_input_event(int status);
extern int psensor_enable_by_touch_driver(int on);

#endif


#endif
