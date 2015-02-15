/* drivers/input/misc/dummy_keypad.c
 *
 * Copyright (C) 2007 HTC Corporation.
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

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/mach-types.h>

#define DRIVER_NAME "dummy_keypad"
const unsigned short _usb_hut_keymap[29] = {
	KEY_1, KEY_2, KEY_3, KEY_4,
	KEY_5, KEY_6, KEY_7, KEY_8,
	KEY_9, KEY_0,
	KEY_BACKSPACE, KEY_ENTER, KEY_HOME, KEY_END,
	KEY_POWER, KEY_COMPOSE, KEY_MENU, KEY_BACK,
	KEY_SWITCHVIDEOMODE, KEY_KBDILLUMTOGGLE,
	KEY_UP, KEY_LEFT, KEY_RIGHT, KEY_DOWN,
	KEY_VOLUMEDOWN, KEY_VOLUMEUP,
	KEY_SEND,
	/* STAR(*) and POUND(#) */
	KEY_SWITCHVIDEOMODE, KEY_KBDILLUMTOGGLE
};
/* FIXME EVM without SPEEDY MACHINE DEFINE*/
#if 0
const unsigned short _usb_hut_keymap_speedy[29] = {
	KEY_1, KEY_2, KEY_3, KEY_4,
	KEY_5, KEY_6, KEY_7, KEY_8,
	KEY_9, KEY_0,
	KEY_HOME, KEY_END, KEY_COMPOSE, KEY_BACK,
	KEY_SWITCHVIDEOMODE, KEY_KBDILLUMTOGGLE,
	KEY_SEND
};
#endif

struct dummy_keypad_struct {
	uint32_t version;
	struct input_dev *input_dev;
	const char *keypad_name;
	const unsigned short *usb_hut_keymap;
	size_t usb_hut_keymap_size;
};

static struct dummy_keypad_struct *dummy_keypad;

static int dummy_keypad_init_func(void)
{
	u16 loop_i;
	dummy_keypad = kzalloc(sizeof(*dummy_keypad), GFP_KERNEL);
	if (dummy_keypad == NULL) {
		printk(KERN_ERR "%s: Can't allocate memory\n", __func__);
		goto kzalloc_fail;
	}
	dummy_keypad->keypad_name = DRIVER_NAME;
/* FIXME EVM without SPEEDY MACHINE DEFINE*/
#if 0
	if (machine_is_speedy())
		dummy_keypad->usb_hut_keymap = _usb_hut_keymap_speedy;
	else
#endif
		dummy_keypad->usb_hut_keymap = _usb_hut_keymap;
	dummy_keypad->usb_hut_keymap_size = ARRAY_SIZE(_usb_hut_keymap);

	dummy_keypad->input_dev = input_allocate_device();
	if (dummy_keypad->input_dev == NULL) {
		printk(KERN_ERR "%s: Failed to allocate input device\n", __func__);
		goto input_allocate_device_fail;
	}
	dummy_keypad->input_dev->name = DRIVER_NAME;
	set_bit(EV_KEY, dummy_keypad->input_dev->evbit);
	/* Setting USB HUT keycode */
	for (loop_i = 0; loop_i < dummy_keypad->usb_hut_keymap_size; loop_i++)	{
		if (dummy_keypad->usb_hut_keymap[loop_i])
			set_bit(dummy_keypad->usb_hut_keymap[loop_i] & KEY_MAX,
				dummy_keypad->input_dev->keybit);
	}

	if (input_register_device(dummy_keypad->input_dev)) {
		printk(KERN_ERR "%s: Unable to register %s input device\n", __func__,
				dummy_keypad->input_dev->name);
		goto input_register_device_fail;
	}

	return 0;
input_register_device_fail:
	input_free_device(dummy_keypad->input_dev);
input_allocate_device_fail:
	kfree(dummy_keypad);
kzalloc_fail:
	return -ENOMEM;
}

static int dummy_keypad_exit_func(void)
{
	input_unregister_device(dummy_keypad->input_dev);
	input_free_device(dummy_keypad->input_dev);
	kfree(dummy_keypad);
	return 0;
}

static int __init dummy_keypad_init(void)
{
	pr_info("%s: %s: initial running...\n", __func__, DRIVER_NAME);
	return dummy_keypad_init_func();
}

static void __exit dummy_keypad_exit(void)
{
	dummy_keypad_exit_func();
	pr_info("%s: %s: exit...\n", __func__, DRIVER_NAME);
}

module_init(dummy_keypad_init);
module_exit(dummy_keypad_exit);
