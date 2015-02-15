/*
 * kernel/power/autosleep.c
 *
 * Opportunistic sleep support.
 *
 * Copyright (C) 2012 Rafael J. Wysocki <rjw@sisk.pl>
 */

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>

#include "power.h"
#ifdef CONFIG_HTC_POWER_DEBUG
#include <linux/pm_wakeup.h>
#endif

static suspend_state_t autosleep_state;
static struct workqueue_struct *autosleep_wq;
/*
 * Note: it is only safe to mutex_lock(&autosleep_lock) if a wakeup_source
 * is active, otherwise a deadlock with try_to_suspend() is possible.
 * Alternatively mutex_lock_interruptible() can be used.  This will then fail
 * if an auto_sleep cycle tries to freeze processes.
 */
static DEFINE_MUTEX(autosleep_lock);
static struct wakeup_source *autosleep_ws;

static void try_to_suspend(struct work_struct *work)
{
	unsigned int initial_count, final_count;

	if (!pm_get_wakeup_count(&initial_count, true)) {
#ifdef CONFIG_HTC_POWER_DEBUG
		pr_info("[P] suspend abort, wakeup event nonzero\n");
		htc_print_active_wakeup_sources();
#endif
		goto out;
	}

	mutex_lock(&autosleep_lock);

	if (!pm_save_wakeup_count(initial_count)) {
#ifdef CONFIG_HTC_POWER_DEBUG
		pr_info("[P] suspend abort, events not matched or being processed\n");
#endif
		mutex_unlock(&autosleep_lock);
		goto out;
	}

	if (autosleep_state == PM_SUSPEND_ON) {
#ifdef CONFIG_HTC_POWER_DEBUG
		pr_info("[P] suspend abort, autosleep_state is ON\n");
#endif
		mutex_unlock(&autosleep_lock);
		return;
	}
	if (autosleep_state >= PM_SUSPEND_MAX)
		hibernate();
	else {
#ifdef CONFIG_HTC_POWER_DEBUG
		pr_info("[R] suspend start\n");
#endif
		pm_suspend(autosleep_state);
	}

	mutex_unlock(&autosleep_lock);

	if (!pm_get_wakeup_count(&final_count, false)) {

#ifdef CONFIG_HTC_POWER_DEBUG
		pr_info("[R] resume end\n");
#endif
		goto out;
	}
	/*
	 * If the wakeup occured for an unknown reason, wait to prevent the
	 * system from trying to suspend and waking up in a tight loop.
	 */
	if (final_count == initial_count) {
#ifdef CONFIG_HTC_POWER_DEBUG
		pr_info("[P] wakeup occured for an unknown reason, wait HZ/2\n");
#endif
		schedule_timeout_uninterruptible(HZ / 2);
	}
#ifdef CONFIG_HTC_POWER_DEBUG
	pr_info("[R] resume end\n");
#endif

 out:
	queue_up_suspend_work();
}

static DECLARE_WORK(suspend_work, try_to_suspend);

void queue_up_suspend_work(void)
{
	if (!work_pending(&suspend_work) && autosleep_state > PM_SUSPEND_ON)
		queue_work(autosleep_wq, &suspend_work);
}

suspend_state_t pm_autosleep_state(void)
{
	return autosleep_state;
}

int pm_autosleep_lock(void)
{
	return mutex_lock_interruptible(&autosleep_lock);
}

void pm_autosleep_unlock(void)
{
	mutex_unlock(&autosleep_lock);
}

int pm_autosleep_set_state(suspend_state_t state)
{

#ifndef CONFIG_HIBERNATION
	if (state >= PM_SUSPEND_MAX)
		return -EINVAL;
#endif

	__pm_stay_awake(autosleep_ws);

	mutex_lock(&autosleep_lock);

	autosleep_state = state;

	__pm_relax(autosleep_ws);

	if (state > PM_SUSPEND_ON) {
		pm_wakep_autosleep_enabled(true);
		queue_up_suspend_work();
	} else {
		pm_wakep_autosleep_enabled(false);
	}

	mutex_unlock(&autosleep_lock);
	return 0;
}


static int wait_for_fb_status= 1;
static ssize_t wait_for_fb_status_show(struct kobject *kobj,
				       struct kobj_attribute *attr, char *buf)
{
	int ret = 0;

	if (wait_for_fb_status == 1)
		ret = sprintf(buf, "on\n");
	else
		ret = sprintf(buf, "off\n");

	return ret;
}

static ssize_t wait_for_fb_status_store(struct kobject *kobj, struct kobj_attribute *attr,
                const char *buf, size_t n)
{
	int val;

	if (sscanf(buf, "%d", &val) == 1) {
		wait_for_fb_status = !!val;
		sysfs_notify(kobj, NULL, "wait_for_fb_status");
		return n;
	}

	return -EINVAL;
}
power_attr(wait_for_fb_status);

static struct attribute *g[] = {
	&wait_for_fb_status_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

int __init pm_autosleep_init(void)
{
	int ret;

	ret = sysfs_create_group(power_kobj, &attr_group);
	if (ret) {
		pr_err("pm_autosleep_init: sysfs_create_group failed\n");
	}

	autosleep_ws = wakeup_source_register("autosleep");
	if (!autosleep_ws)
		return -ENOMEM;

	autosleep_wq = alloc_ordered_workqueue("autosleep", 0);
	if (autosleep_wq)
		return 0;

	wakeup_source_unregister(autosleep_ws);
	return -ENOMEM;
}
