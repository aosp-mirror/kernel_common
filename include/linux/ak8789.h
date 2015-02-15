
/* drivers/input/misc/hall_sensor.c - Ak8789 Hall sensor driver
 *
 * Copyright (C) 2013 HTC Corporation.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef HTC_HALL_SENSOR_H
#define HTC_HALL_SENSOR_H

#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/rtc.h>
#include <linux/wakelock.h>

#define HL_INPUTDEV_NAME "AK8789_HALL_SENSOR"

#define HL_LOG(fmt, arg...) \
	printk(KERN_INFO "[" DRIVER_NAME "] (%s) " fmt "\n", __func__, ## arg)

#define HL_LOG_TIME(fmt, arg...) do { \
	struct timespec ts; \
	struct rtc_time tm; \
	getnstimeofday(&ts); \
	rtc_time_to_tm(ts.tv_sec, &tm); \
	printk(KERN_INFO "[" DRIVER_NAME "] (%s) " fmt \
		" (%02d-%02d %02d:%02d:%02d.%03lu)\n", __func__, \
		## arg, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, \
		tm.tm_min, tm.tm_sec, ts.tv_nsec / 1000000); \
	} while (0)

#define HL_ERR(fmt, arg...) \
	printk(KERN_INFO "[" DRIVER_NAME "_ERR] (%s) " fmt "\n", \
		__func__, ## arg)

struct hall_platform_data {
	uint32_t gpio_att:16;
	uint32_t gpio_att_s:16;
	uint8_t att_used;
};
#endif
