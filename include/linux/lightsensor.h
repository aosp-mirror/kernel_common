/* include/linux/lightsensor.h
 *
 * Copyright (C) 2009 Google, Inc.
 * Author: Iliyan Malchev <malchev@google.com>
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

#ifndef __LINUX_LIGHTSENSOR_H
#define __LINUX_LIGHTSENSOR_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define LIGHTSENSOR_IOCTL_MAGIC 'l'

#define LIGHTSENSOR_IOCTL_GET_ENABLED _IOR(LIGHTSENSOR_IOCTL_MAGIC, 1, int *)
#define LIGHTSENSOR_IOCTL_ENABLE _IOW(LIGHTSENSOR_IOCTL_MAGIC, 2, int *)

struct lightsensor_mpp_config_data {
	uint32_t lightsensor_mpp;
	uint32_t lightsensor_amux;
};

struct lightsensor_smd_platform_data {
	const char      *name;
	uint16_t        levels[10];
	uint16_t        golden_adc;
	uint16_t		m_voltage;
	int             (*ls_power)(int, uint8_t);
	struct lightsensor_mpp_config_data mpp_data;
};

#endif
