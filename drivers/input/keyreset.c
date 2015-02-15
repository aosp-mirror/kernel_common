/* drivers/input/keyreset.c
 *
 * Copyright (C) 2008 Google, Inc.
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

#include <linux/input.h>
#include <linux/keyreset.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <mach/board.h>
/*#include <mach/msm_watchdog.h>*/
#include <mach/devices_cmdline.h>
#include <mach/board_htc.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#define KEYRESET_DELAY 3*HZ

struct keyreset_state {
	struct input_handler input_handler;
	unsigned long keybit[BITS_TO_LONGS(KEY_CNT)];
	unsigned long upbit[BITS_TO_LONGS(KEY_CNT)];
	unsigned long key[BITS_TO_LONGS(KEY_CNT)];
	spinlock_t lock;
	int key_down_target;
	int key_down;
	int key_up;
	int restart_disabled;
	int (*reset_fn)(void);
};

static int restart_requested;
static unsigned long restart_timeout;

static void deferred_restart(struct work_struct *dummy)
{
	restart_requested = 2;
	sys_sync();
	restart_requested = 3;
	kernel_restart(NULL);
}
static DECLARE_DELAYED_WORK(restart_work, deferred_restart);

static void keyreset_event(struct input_handle *handle, unsigned int type,
			   unsigned int code, int value)
{
	unsigned long flags;
	struct keyreset_state *state = handle->private;

	if (type != EV_KEY)
		return;

	if (code >= KEY_MAX)
		return;

	if (!test_bit(code, state->keybit))
		return;

	spin_lock_irqsave(&state->lock, flags);
	if (!test_bit(code, state->key) == !value)
		goto done;
	__change_bit(code, state->key);
	if (test_bit(code, state->upbit)) {
		if (value) {
			state->restart_disabled = 1;
			state->key_up++;
		} else
			state->key_up--;
	} else {
		if (value)
			state->key_down++;
		else
			state->key_down--;
	}
	if (state->key_down == 0 && state->key_up == 0)
		state->restart_disabled = 0;

	pr_debug("reset key changed %d %d new state %d-%d-%d\n", code, value,
		 state->key_down, state->key_up, state->restart_disabled);

	if (value && !state->restart_disabled &&
	    state->key_down == state->key_down_target) {
		state->restart_disabled = 1;
		if (restart_requested) {
			//msm_watchdog_suspend(NULL);
			/* show blocked processes to debug hang problems */
			printk(KERN_INFO "\n### Show Blocked State ###\n");
			show_state_filter(TASK_UNINTERRUPTIBLE);
			//msm_watchdog_resume(NULL);
			if (time_after(jiffies, restart_timeout))
				panic("keyboard reset failed, %d", restart_requested);
			return;
		}
		if (state->reset_fn) {
			restart_requested = state->reset_fn();
		} else {
			pr_info("keyboard reset\n");
			schedule_delayed_work(&restart_work, KEYRESET_DELAY);
			restart_requested = 1;
			restart_timeout = jiffies + 20 * HZ;
			//msm_watchdog_suspend(NULL);
			/* show blocked processes to debug hang problems */
			printk(KERN_INFO "\n### Show Blocked State ###\n");
			show_state_filter(TASK_UNINTERRUPTIBLE);
			//msm_watchdog_resume(NULL);
		}
	} else if (restart_requested == 1) {
		if (cancel_delayed_work(&restart_work)) {
			pr_info("%s: cancel restart work\n", __func__);
			restart_requested = 0;
		} else
			pr_info("%s: cancel failed\n", __func__);
	}
done:
	spin_unlock_irqrestore(&state->lock, flags);
}

static int keyreset_connect(struct input_handler *handler,
					  struct input_dev *dev,
					  const struct input_device_id *id)
{
	int i;
	int ret;
	struct input_handle *handle;
	struct keyreset_state *state =
		container_of(handler, struct keyreset_state, input_handler);

	for (i = 0; i < KEY_MAX; i++) {
		if (test_bit(i, state->keybit) && test_bit(i, dev->keybit))
			break;
	}
	if (i == KEY_MAX)
		return -ENODEV;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "keyreset";
	handle->private = state;

	ret = input_register_handle(handle);
	if (ret)
		goto err_input_register_handle;

	ret = input_open_device(handle);
	if (ret)
		goto err_input_open_device;

	pr_info("using input dev %s for key reset\n", dev->name);

	return 0;

err_input_open_device:
	input_unregister_handle(handle);
err_input_register_handle:
	kfree(handle);
	return ret;
}

static void keyreset_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id keyreset_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};
MODULE_DEVICE_TABLE(input, keyreset_ids);

static struct keyreset_platform_data reset_key_pdata = {
	.keys_down = {
		KEY_POWER,
		KEY_VOLUMEDOWN,
		KEY_VOLUMEUP,
		0
	},
};

#ifdef CONFIG_OF
static bool keyreset_dt_parser(struct device *dev)
{
	const char *parser = {"keyreset,driver_state"}, *drstate;
	struct device_node *node = dev->of_node;

	pr_info("[KEYRESET] %s", __func__);

	if (of_property_read_string(node, parser, &drstate) == 0) {
		if (!strncmp(drstate, "enable", 6))
			return true;
		else
			pr_info("[KEYRESET] driver state parser failed(%s)", drstate);
	} else
		pr_info("[KEYRESET] dt parser faile, can't find driver state");
	return false;
}
#endif

static int keyreset_probe(struct platform_device *pdev)
{
	int ret = 0;
	int key, *keyp;
	struct keyreset_state *state;
#ifndef CONFIG_OF
	struct keyreset_platform_data *pdata = pdev->dev.platform_data;
#else
	uint8_t idx = 0;
	struct keyreset_platform_data *pdata;
	pr_info("[KEYRESET] ++%s++", __func__);
#endif
	if (!board_build_flag()) {
		pr_info("[KEYRESET] Ship code, disable key reset");
		return 0;
	}
	if (pdev->dev.of_node) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL)
			pr_err("[KEYRESET] alloc memory fail");
		if (keyreset_dt_parser(&pdev->dev)) {
			while (reset_key_pdata.keys_down[idx] != 0)
				idx++;
			pr_info("[KEYRESET] idx=%d ", idx);

			memcpy(pdata->keys_down, &reset_key_pdata.keys_down,
				sizeof(int)*(idx+1));
		} else {
			ret = -ENOMEM;
			goto err_driver_state;
		}
	}

	if (!pdata)
		return -EINVAL;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	spin_lock_init(&state->lock);
	keyp = pdata->keys_down;
	while ((key = *keyp++)) {
		if (key >= KEY_MAX)
			continue;
		state->key_down_target++;
		__set_bit(key, state->keybit);
	}
	if (pdata->keys_up) {
		keyp = pdata->keys_up;
		while ((key = *keyp++)) {
			if (key >= KEY_MAX)
				continue;
			__set_bit(key, state->keybit);
			__set_bit(key, state->upbit);
		}
	}

	if (pdata->reset_fn)
		state->reset_fn = pdata->reset_fn;

	state->input_handler.event = keyreset_event;
	state->input_handler.connect = keyreset_connect;
	state->input_handler.disconnect = keyreset_disconnect;
	state->input_handler.name = KEYRESET_NAME;
	state->input_handler.id_table = keyreset_ids;
	ret = input_register_handler(&state->input_handler);
	if (ret) {
		kfree(state);
		return ret;
	}
	platform_set_drvdata(pdev, state);
	pr_info("[KEYRESET] --%s--", __func__);
	return 0;
err_driver_state:
	kfree(pdata);
	return ret;
}

int keyreset_remove(struct platform_device *pdev)
{
	struct keyreset_state *state = platform_get_drvdata(pdev);
	if (board_build_flag()) {
		input_unregister_handler(&state->input_handler);
		kfree(state);
	}
	return 0;
}

static const struct of_device_id keyreset_mttable[] = {
	{ .compatible = "keyreset_driver" },
	{ },
};

struct platform_driver keyreset_driver = {
	.driver.name = KEYRESET_NAME,
	.probe = keyreset_probe,
	.remove = keyreset_remove,
	.driver = {
		.name = KEYRESET_NAME,
		.of_match_table = keyreset_mttable,
	},
};

static int __init keyreset_init(void)
{
	return platform_driver_register(&keyreset_driver);
}

static void __exit keyreset_exit(void)
{
	return platform_driver_unregister(&keyreset_driver);
}

module_init(keyreset_init);
module_exit(keyreset_exit);

MODULE_DESCRIPTION("KEYRESET Driver");
MODULE_LICENSE("GPL");
