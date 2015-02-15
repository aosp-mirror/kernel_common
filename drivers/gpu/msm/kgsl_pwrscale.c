/* Copyright (c) 2010-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/export.h>
#include <linux/kernel.h>

#include "kgsl.h"
#include "kgsl_pwrscale.h"
#include "kgsl_device.h"
#include "kgsl_trace.h"

#define FAST_BUS 1
#define SLOW_BUS -1

static void do_devfreq_suspend(struct work_struct *work);
static void do_devfreq_resume(struct work_struct *work);
static void do_devfreq_notify(struct work_struct *work);

void kgsl_pwrscale_sleep(struct kgsl_device *device)
{
	BUG_ON(!mutex_is_locked(&device->mutex));
	if (!device->pwrscale.enabled)
		return;
	device->pwrscale.time = device->pwrscale.on_time = 0;

	
	queue_work(device->pwrscale.devfreq_wq,
		&device->pwrscale.devfreq_suspend_ws);
}
EXPORT_SYMBOL(kgsl_pwrscale_sleep);

void kgsl_pwrscale_wake(struct kgsl_device *device)
{
	struct kgsl_power_stats stats;
	BUG_ON(!mutex_is_locked(&device->mutex));

	if (!device->pwrscale.enabled)
		return;
	
	memset(&device->pwrscale.accum_stats, 0,
		sizeof(device->pwrscale.accum_stats));

	
	device->ftbl->power_stats(device, &stats);

	device->pwrscale.time = ktime_to_us(ktime_get());

	device->pwrscale.next_governor_call = 0;

	
	queue_work(device->pwrscale.devfreq_wq,
		&device->pwrscale.devfreq_resume_ws);
}
EXPORT_SYMBOL(kgsl_pwrscale_wake);

void kgsl_pwrscale_busy(struct kgsl_device *device)
{
	BUG_ON(!mutex_is_locked(&device->mutex));
	if (!device->pwrscale.enabled)
		return;
	if (device->pwrscale.on_time == 0)
		device->pwrscale.on_time = ktime_to_us(ktime_get());
}
EXPORT_SYMBOL(kgsl_pwrscale_busy);

void kgsl_pwrscale_update(struct kgsl_device *device)
{
	struct kgsl_power_stats stats;
	BUG_ON(!mutex_is_locked(&device->mutex));

	if (!device->pwrscale.enabled)
		return;

	if (device->pwrscale.next_governor_call == 0)
		device->pwrscale.next_governor_call = jiffies;

	if (time_before(jiffies, device->pwrscale.next_governor_call))
		return;

	device->pwrscale.next_governor_call = jiffies
			+ msecs_to_jiffies(KGSL_GOVERNOR_CALL_INTERVAL);

	if (device->state == KGSL_STATE_ACTIVE) {
		device->ftbl->power_stats(device, &stats);
		device->pwrscale.accum_stats.busy_time += stats.busy_time;
		device->pwrscale.accum_stats.ram_time += stats.ram_time;
		device->pwrscale.accum_stats.ram_wait += stats.ram_wait;
	}

	
	if (device->requested_state != KGSL_STATE_SLUMBER)
		queue_work(device->pwrscale.devfreq_wq,
			&device->pwrscale.devfreq_notify_ws);
}
EXPORT_SYMBOL(kgsl_pwrscale_update);

void kgsl_pwrscale_disable(struct kgsl_device *device)
{
	BUG_ON(!mutex_is_locked(&device->mutex));

	if (device->pwrscale.enabled) {
		queue_work(device->pwrscale.devfreq_wq,
			&device->pwrscale.devfreq_suspend_ws);
		device->pwrscale.enabled = false;
		kgsl_pwrctrl_pwrlevel_change(device, KGSL_PWRLEVEL_TURBO);
	}
}
EXPORT_SYMBOL(kgsl_pwrscale_disable);

void kgsl_pwrscale_enable(struct kgsl_device *device)
{
	BUG_ON(!mutex_is_locked(&device->mutex));

	if (!device->pwrscale.enabled) {
		device->pwrscale.enabled = true;
		queue_work(device->pwrscale.devfreq_wq,
			&device->pwrscale.devfreq_resume_ws);
	}
}
EXPORT_SYMBOL(kgsl_pwrscale_enable);

int kgsl_devfreq_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct kgsl_device *device = dev_get_drvdata(dev);
	struct kgsl_pwrctrl *pwr;
	int level, i, b;
	unsigned long cur_freq;

	if (device == NULL)
		return -ENODEV;
	if (freq == NULL)
		return -EINVAL;
	if (!device->pwrscale.enabled)
		return 0;

	pwr = &device->pwrctrl;

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
	cur_freq = kgsl_pwrctrl_active_freq(pwr);
	level = pwr->active_pwrlevel;

	if (*freq != cur_freq) {
		level = pwr->max_pwrlevel;
		for (i = pwr->min_pwrlevel; i >= pwr->max_pwrlevel; i--)
			if (*freq <= pwr->pwrlevels[i].gpu_freq) {
				level = i;
				break;
			}
	} else if (flags && pwr->bus_control) {
		b = pwr->bus_mod;
		if ((flags & DEVFREQ_FLAG_FAST_HINT) &&
			(pwr->bus_mod != FAST_BUS))
			pwr->bus_mod = (pwr->bus_mod == SLOW_BUS) ?
					0 : FAST_BUS;
		else if ((flags & DEVFREQ_FLAG_SLOW_HINT) &&
			(pwr->bus_mod != SLOW_BUS))
			pwr->bus_mod = (pwr->bus_mod == FAST_BUS) ?
					0 : SLOW_BUS;
		if (pwr->bus_mod != b)
			kgsl_pwrctrl_buslevel_update(device, true);
	}

	if ((pwr->constraint.type != KGSL_CONSTRAINT_NONE) &&
		(!time_after(jiffies, pwr->constraint.expires)) &&
		(level >= pwr->constraint.hint.pwrlevel.level))
			*freq = cur_freq;
	else {
		
		kgsl_pwrctrl_pwrlevel_change(device, level);

		
		pwr->constraint.type = KGSL_CONSTRAINT_NONE;
		pwr->constraint.expires = 0;

		*freq = kgsl_pwrctrl_active_freq(pwr);
	}

	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
	return 0;
}
EXPORT_SYMBOL(kgsl_devfreq_target);

int kgsl_devfreq_get_dev_status(struct device *dev,
				struct devfreq_dev_status *stat)
{
	struct kgsl_device *device = dev_get_drvdata(dev);
	struct kgsl_pwrscale *pwrscale;
	s64 tmp;

	if (device == NULL)
		return -ENODEV;
	if (stat == NULL)
		return -EINVAL;

	pwrscale = &device->pwrscale;

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
	
	if (device->state == KGSL_STATE_ACTIVE) {
		struct kgsl_power_stats extra;
		device->ftbl->power_stats(device, &extra);
		device->pwrscale.accum_stats.busy_time += extra.busy_time;
		device->pwrscale.accum_stats.ram_time += extra.ram_time;
		device->pwrscale.accum_stats.ram_wait += extra.ram_wait;
	}

	tmp = ktime_to_us(ktime_get());
	stat->total_time = tmp - pwrscale->time;
	pwrscale->time = tmp;

	stat->busy_time = pwrscale->accum_stats.busy_time;

	stat->current_frequency = kgsl_pwrctrl_active_freq(&device->pwrctrl);

	if (stat->private_data) {
		struct xstats *b = (struct xstats *)stat->private_data;
		b->ram_time = device->pwrscale.accum_stats.ram_time;
		b->ram_wait = device->pwrscale.accum_stats.ram_wait;
		b->mod = device->pwrctrl.bus_mod;
	}

	trace_kgsl_pwrstats(device, stat->total_time, &pwrscale->accum_stats);
	memset(&pwrscale->accum_stats, 0, sizeof(pwrscale->accum_stats));

	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);

	return 0;
}
EXPORT_SYMBOL(kgsl_devfreq_get_dev_status);

int kgsl_devfreq_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct kgsl_device *device = dev_get_drvdata(dev);

	if (device == NULL)
		return -ENODEV;
	if (freq == NULL)
		return -EINVAL;

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
	*freq = kgsl_pwrctrl_active_freq(&device->pwrctrl);
	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);

	return 0;
}
EXPORT_SYMBOL(kgsl_devfreq_get_cur_freq);

int kgsl_devfreq_add_notifier(struct device *dev, struct notifier_block *nb)
{
	struct kgsl_device *device = dev_get_drvdata(dev);

	if (device == NULL)
		return -ENODEV;

	if (nb == NULL)
		return -EINVAL;

	return srcu_notifier_chain_register(&device->pwrscale.nh, nb);
}

void kgsl_pwrscale_idle(struct kgsl_device *device)
{
	BUG_ON(!mutex_is_locked(&device->mutex));
	queue_work(device->pwrscale.devfreq_wq,
		&device->pwrscale.devfreq_notify_ws);
}
EXPORT_SYMBOL(kgsl_pwrscale_idle);

int kgsl_devfreq_del_notifier(struct device *dev, struct notifier_block *nb)
{
	struct kgsl_device *device = dev_get_drvdata(dev);

	if (device == NULL)
		return -ENODEV;

	if (nb == NULL)
		return -EINVAL;

	return srcu_notifier_chain_unregister(&device->pwrscale.nh, nb);
}
EXPORT_SYMBOL(kgsl_devfreq_del_notifier);

int kgsl_pwrscale_init(struct device *dev, const char *governor)
{
	struct kgsl_device *device;
	struct kgsl_pwrscale *pwrscale;
	struct kgsl_pwrctrl *pwr;
	struct devfreq *devfreq;
	struct devfreq_dev_profile *profile;
	struct devfreq_msm_adreno_tz_data *data;
	int i, out = 0;
	int ret;

	device = dev_get_drvdata(dev);
	if (device == NULL)
		return -ENODEV;

	pwrscale = &device->pwrscale;
	pwr = &device->pwrctrl;
	profile = &pwrscale->profile;

	srcu_init_notifier_head(&pwrscale->nh);

	profile->initial_freq =
		pwr->pwrlevels[pwr->default_pwrlevel].gpu_freq;
	
	profile->polling_ms = 10;

	
	for (i = 0; i < (pwr->num_pwrlevels - 1); i++)
		pwrscale->freq_table[out++] = pwr->pwrlevels[i].gpu_freq;

	profile->max_state = out;
	
	profile->freq_table = pwrscale->freq_table;

	
	if (profile->max_state == 1)
		governor = "performance";

	
	for (i = 0; i < profile->num_governor_data; i++) {
		if (strncmp("msm-adreno-tz",
				profile->governor_data[i].name,
				DEVFREQ_NAME_LEN) == 0) {
			data = (struct devfreq_msm_adreno_tz_data *)
				profile->governor_data[i].data;
			if (pwr->bus_control) {
				out = 0;
				while (pwr->bus_ib[out]) {
					pwr->bus_ib[out] =
						pwr->bus_ib[out] >> 20;
					out++;
				}
				data->bus.num = out;
				data->bus.ib = &pwr->bus_ib[0];
				data->bus.index = &pwr->bus_index[0];
				printk("kgsl: num bus is %d\n", out);
			} else {
				data->bus.num = 0;
			}
		}
	}

	devfreq = devfreq_add_device(dev, &pwrscale->profile, governor, NULL);
	if (IS_ERR(devfreq))
		return PTR_ERR(devfreq);

	pwrscale->devfreq = devfreq;

	ret = sysfs_create_link(&device->dev->kobj,
			&devfreq->dev.kobj, "devfreq");
	if(ret)
		pr_err("kgsl: sysfs_create_link-devfreq fail\n");

	pwrscale->devfreq_wq = create_freezable_workqueue("kgsl_devfreq_wq");
	INIT_WORK(&pwrscale->devfreq_suspend_ws, do_devfreq_suspend);
	INIT_WORK(&pwrscale->devfreq_resume_ws, do_devfreq_resume);
	INIT_WORK(&pwrscale->devfreq_notify_ws, do_devfreq_notify);

	pwrscale->next_governor_call = 0;

	return 0;
}
EXPORT_SYMBOL(kgsl_pwrscale_init);

void kgsl_pwrscale_close(struct kgsl_device *device)
{
	struct kgsl_pwrscale *pwrscale;

	BUG_ON(!mutex_is_locked(&device->mutex));

	pwrscale = &device->pwrscale;
	flush_workqueue(pwrscale->devfreq_wq);
	destroy_workqueue(pwrscale->devfreq_wq);
	devfreq_remove_device(device->pwrscale.devfreq);
	device->pwrscale.devfreq = NULL;
	srcu_cleanup_notifier_head(&device->pwrscale.nh);
}
EXPORT_SYMBOL(kgsl_pwrscale_close);

static void do_devfreq_suspend(struct work_struct *work)
{
	struct kgsl_pwrscale *pwrscale = container_of(work,
			struct kgsl_pwrscale, devfreq_suspend_ws);
	struct devfreq *devfreq = pwrscale->devfreq;

	devfreq_suspend_device(devfreq);
}

static void do_devfreq_resume(struct work_struct *work)
{
	struct kgsl_pwrscale *pwrscale = container_of(work,
			struct kgsl_pwrscale, devfreq_resume_ws);
	struct devfreq *devfreq = pwrscale->devfreq;

	devfreq_resume_device(devfreq);
}

static void do_devfreq_notify(struct work_struct *work)
{
	struct kgsl_pwrscale *pwrscale = container_of(work,
			struct kgsl_pwrscale, devfreq_notify_ws);
	struct devfreq *devfreq = pwrscale->devfreq;
	srcu_notifier_call_chain(&pwrscale->nh,
				 ADRENO_DEVFREQ_NOTIFY_RETIRE,
				 devfreq);
}
