/*
 * Linux 2.6.32 and later Kernel module for VMware MVP OEK Test
 *
 * Copyright (C) 2010-2013 VMware, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; see the file COPYING.  If not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#line 5

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/input.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>

#define DRV_NAME	"mvp-fake-kb"
#define ENABLE_FILE	"enable"
#define SEND_KEY_FILE	"key"

/**
 * HOWTO:
 * This driver is a fake keyboard. It allows injecting Linux keycodes in the
 * system from user space.
 * Two files manage this keyboard in the proc fs (in /proc/DRV_NAME):
 *     - ENABLE_FILE:
 *           Reading it gives the current status of the keyboard 0 means
 *           unregistered and 1 means registered.
 *           Writing to it allows to register/unregister the keyboard.
 *           The syntax is "<enable> <timeout>". Enable set to 0 disable
 *           the keyboard while a positive value enable it. Specifying
 *           a timeout in seconds activates a watchdog to disable the kb if
 *           no manual unregister happens before. Registering a kb
 *           already registered with a later timeout allows to modify
 *           it. If no timeout is provided, the kb must be disabled
 *           manually.
 *
 *     - SEND_KEY_FILE:
 *           Writing to it allows to send a keycode if the kb is registered.
 *           The syntax is "<keycode> <action>". Keycode is the linux keycode,
 *           action is 1 for ACTION_DOWN (press) and 0 for ACTION_UP (release).
 */

static struct proc_dir_entry *dir;

static struct input_dev *key_dev;

/*
 * Mutex protects key_dev access to avoid sending keys while disabling keyboard
 * It also protects time_to_unregister.
 */
static DEFINE_MUTEX(dev_mutex);

static void unregister_keyboard_handler(struct work_struct *work);
/*
 * Work used to schedule keyboard unregistration.
 */
static DECLARE_DELAYED_WORK(unregister_keyboard_work,
			    unregister_keyboard_handler);

/**
 * Time for the next unregistration.
 */
static unsigned long time_to_unregister = INITIAL_JIFFIES;

/**
 * Register the fake keyboard
 *
 * @param timeout time to automatically unregister the keyboard if not manually
 *        unregistered.
 * @note: must be called with dev_mutex locked
 */
static void
register_keyboard_locked(int timeout)
{
	/* Register keyboard if needed */
	if (!key_dev) {
		int i;
		key_dev = input_allocate_device();
		if (!key_dev) {
			printk(KERN_ERR "Cannot allocate input device\n");
			return;
		}

		key_dev->name = DRV_NAME;
		key_dev->evbit[0] = BIT_MASK(EV_KEY);
		/* Allow all the keycodes */
		for (i = 0; i <= BIT_WORD(KEY_MAX); i++)
			key_dev->keybit[i] = (unsigned long) -1;

		if (input_register_device(key_dev)) {
			printk(KERN_ERR "Cannot register input device\n");
			input_free_device(key_dev);
			key_dev = NULL;
			return;
		}
	}

	/* Manage automatic unregistration */
	if (timeout > 0) {
		/* Shedule unregister or update timeout */
		unsigned long new_timeout =
			jiffies + msecs_to_jiffies(timeout * 1000);
		if (time_after(new_timeout, time_to_unregister)) {
			cancel_delayed_work(&unregister_keyboard_work);
			schedule_delayed_work(&unregister_keyboard_work,
					      msecs_to_jiffies(timeout * 1000));
			time_to_unregister = new_timeout;
		}
	} else {
		/* keep the keyboard enabled until manual disable */
		cancel_delayed_work(&unregister_keyboard_work);
		time_to_unregister = MAX_JIFFY_OFFSET;
	}
}

/**
 * Unregister the fake keyboard
 *
 * @note: must be called with dev_mutex locked
 */
static void
unregister_keyboard_locked(void)
{
	/* Cancel delayed work for automatic unregistration */
	cancel_delayed_work(&unregister_keyboard_work);
	time_to_unregister = INITIAL_JIFFIES;
	if (key_dev != NULL) {
		input_unregister_device(key_dev);
		key_dev = NULL;
	}
}

static void
unregister_keyboard_handler(struct work_struct *work)
{
	mutex_lock(&dev_mutex);
	/* Check that our work was not already running while updating timeout */
	if (time_is_before_jiffies(time_to_unregister)) {
		unregister_keyboard_locked();
		printk(KERN_INFO "Keyboard unregistered after timeout\n");
	}
	mutex_unlock(&dev_mutex);
}

static int
keyboard_state(char *buffer,
	       char **start,
	       off_t offset,
	       int count,
	       int *eof,
	       void *data)
{
	int len;

	/*
	 * Implemented way to return data is "way 0" described in
	 * fs/proc/generic.c.
	 * We always send everything on the first call (offset=0),
	 * and just need 3 bytes
	 */
	if (offset > 0 || count < 3) {
		*eof = 1;
		return 0;
	}

	mutex_lock(&dev_mutex);
	len = sprintf(buffer, "%c\n", (key_dev != NULL) ? '1' : '0');
	*eof = 1;
	mutex_unlock(&dev_mutex);

	return len;
}

static int
enable_keyboard(struct file *file,
		const char __user *buffer,
		unsigned long count,
		void *data)
{
	int enable = -1;
	int timeout = 0;
	int items;

	items = sscanf(buffer, "%d %d", &enable, &timeout);
	if (items < 1 || items > 2 || enable < 0) {
		printk(KERN_ERR "Malformed value to enable/disable keyboard\n");
		return -1;
	}

	mutex_lock(&dev_mutex);
	if (enable > 0) /* Enable the keyboard */
		register_keyboard_locked(timeout);
	else /* Disable the keyboard */
		unregister_keyboard_locked();
	mutex_unlock(&dev_mutex);

	return count;
}

static int
send_key(struct file *file,
	 const char __user *buffer,
	 unsigned long count,
	 void *data)
{
	int key = -1;
	int value = -1;
	int items = 0;

	items = sscanf(buffer, "%d %d", &key, &value);
	if (items != 2 || key < 0 || value < 0) {
		printk(KERN_ERR "Malformed value to send a key\n");
		return -1;
	}

	mutex_lock(&dev_mutex);
	if (key_dev != NULL) {
		printk(KERN_INFO "Send key=%d Value=%d\n", key, value);
		input_report_key(key_dev, key, value);
		input_sync(key_dev);
	}
	mutex_unlock(&dev_mutex);
	return count;
}

static int __init
mvp_keypad_init(void)
{
	int ret = 0;
	struct proc_dir_entry *enable_file, *key_file;
	dir = proc_mkdir(DRV_NAME, NULL);
	if (!dir) {
		ret = -EIO;
		goto error;
	}

	/* Create a file to manage enable/disable of the keyboard */
	enable_file = create_proc_entry(ENABLE_FILE, 0666, dir);
	if (!enable_file) {
		ret = -EIO;
		goto err_remove_dir;
	}
	enable_file->write_proc = enable_keyboard;
	enable_file->read_proc = keyboard_state;

	/* Create file to manage key sending */
	key_file = create_proc_entry(SEND_KEY_FILE, 0222, dir);
	if (!key_file) {
		ret = -EIO;
		goto err_remove_dir_and_key;
	}
	key_file->write_proc = send_key;

	printk(KERN_INFO "Init done for "DRV_NAME"\n");
	return ret;

err_remove_dir_and_key:
	remove_proc_entry("enable", dir);
err_remove_dir:
	remove_proc_entry(DRV_NAME, NULL);
error:
	return ret;
}

static void __exit
mvp_keypad_exit(void)
{
	remove_proc_entry(SEND_KEY_FILE, dir);
	remove_proc_entry(ENABLE_FILE, dir);
	remove_proc_entry(DRV_NAME, NULL);

	mutex_lock(&dev_mutex);
	unregister_keyboard_locked();
	mutex_unlock(&dev_mutex);
}

module_init(mvp_keypad_init);
module_exit(mvp_keypad_exit);

MODULE_AUTHOR("VMware");
MODULE_DESCRIPTION("MVP fake keyboard");
MODULE_LICENSE("GPL");
