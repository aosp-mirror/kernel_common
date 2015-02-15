/*
 *
 * /arch/arm/mach-msm/include/mach/htc_headset_gpio.h
 *
 * HTC GPIO headset driver.
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

#ifndef HTC_HEADSET_GPIO_H
#define HTC_HEADSET_GPIO_H

struct htc_headset_gpio_platform_data {
	unsigned int uart_gpio;
	unsigned int hpin_gpio;
	unsigned int key_gpio;
	unsigned int key_enable_gpio;
	unsigned int mic_select_gpio;
	void (*config_headset_gpio)(void);
};

struct htc_headset_gpio_info {
	struct htc_headset_gpio_platform_data pdata;
	unsigned int hpin_irq;
	unsigned int hpin_debounce;
	unsigned int key_irq;
	unsigned int key_irq_type;
	int headset_state;
	struct wake_lock hs_wake_lock;
};

#endif
