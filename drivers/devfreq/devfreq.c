/*
 * devfreq: Generic Dynamic Voltage and Frequency Scaling (DVFS) Framework
 *	    for Non-CPU Devices.
 *
 * Copyright (C) 2011 Samsung Electronics
 *	MyungJoo Ham <myungjoo.ham@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/opp.h>
#include <linux/devfreq.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/printk.h>
#include <linux/hrtimer.h>
#include "governor.h"

static struct class *devfreq_class;

static struct workqueue_struct *devfreq_wq;

static LIST_HEAD(devfreq_governor_list);
static LIST_HEAD(devfreq_list);
static DEFINE_MUTEX(devfreq_list_lock);

static struct devfreq *find_device_devfreq(struct device *dev)
{
	struct devfreq *tmp_devfreq;

	if (unlikely(IS_ERR_OR_NULL(dev))) {
		pr_err("DEVFREQ: %s: Invalid parameters\n", __func__);
		return ERR_PTR(-EINVAL);
	}
	WARN(!mutex_is_locked(&devfreq_list_lock),
	     "devfreq_list_lock must be locked.");

	list_for_each_entry(tmp_devfreq, &devfreq_list, node) {
		if (tmp_devfreq->dev.parent == dev)
			return tmp_devfreq;
	}

	return ERR_PTR(-ENODEV);
}

static void devfreq_set_freq_limits(struct devfreq *devfreq)
{
	int idx;
	unsigned long min = ~0, max = 0;

	if (!devfreq->profile->freq_table)
		return;

	for (idx = 0; idx < devfreq->profile->max_state; idx++) {
		if (min > devfreq->profile->freq_table[idx])
			min = devfreq->profile->freq_table[idx];
		if (max < devfreq->profile->freq_table[idx])
			max = devfreq->profile->freq_table[idx];
	}

	devfreq->min_freq = min;
	devfreq->max_freq = max;
}

int devfreq_get_freq_level(struct devfreq *devfreq, unsigned long freq)
{
	int lev;

	for (lev = 0; lev < devfreq->profile->max_state; lev++)
		if (freq == devfreq->profile->freq_table[lev])
			return lev;

	return -EINVAL;
}
EXPORT_SYMBOL(devfreq_get_freq_level);

static int devfreq_update_status(struct devfreq *devfreq, unsigned long freq)
{
	int lev, prev_lev;
	unsigned long cur_time;

	lev = devfreq_get_freq_level(devfreq, freq);
	if (lev < 0)
		return lev;

	cur_time = jiffies;
	devfreq->time_in_state[lev] +=
			 cur_time - devfreq->last_stat_updated;
	devfreq->last_stat_updated = cur_time;

	if (freq == devfreq->previous_freq)
		return 0;

	prev_lev = devfreq_get_freq_level(devfreq, devfreq->previous_freq);
	if (prev_lev < 0)
		return 0;

	if (lev != prev_lev) {
		devfreq->trans_table[(prev_lev *
				devfreq->profile->max_state) + lev]++;
		devfreq->total_trans++;
	}

	return 0;
}

static struct devfreq_governor *find_devfreq_governor(const char *name)
{
	struct devfreq_governor *tmp_governor;

	if (unlikely(IS_ERR_OR_NULL(name))) {
		pr_err("DEVFREQ: %s: Invalid parameters\n", __func__);
		return ERR_PTR(-EINVAL);
	}
	WARN(!mutex_is_locked(&devfreq_list_lock),
	     "devfreq_list_lock must be locked.");

	list_for_each_entry(tmp_governor, &devfreq_governor_list, node) {
		if (!strncmp(tmp_governor->name, name, DEVFREQ_NAME_LEN))
			return tmp_governor;
	}

	return ERR_PTR(-ENODEV);
}


int update_devfreq(struct devfreq *devfreq)
{
	unsigned long freq;
	int err = 0;
	u32 flags = 0;

	if (!mutex_is_locked(&devfreq->lock)) {
		WARN(true, "devfreq->lock must be locked by the caller.\n");
		return -EINVAL;
	}

	if (!devfreq->governor)
		return -EINVAL;

	
	err = devfreq->governor->get_target_freq(devfreq, &freq, &flags);
	if (err)
		return err;


	if (devfreq->min_freq && freq < devfreq->min_freq) {
		freq = devfreq->min_freq;
		flags &= ~DEVFREQ_FLAG_LEAST_UPPER_BOUND; 
	}
	if (devfreq->max_freq && freq > devfreq->max_freq) {
		freq = devfreq->max_freq;
		flags |= DEVFREQ_FLAG_LEAST_UPPER_BOUND; 
	}

	err = devfreq->profile->target(devfreq->dev.parent, &freq, flags);
	if (err)
		return err;

	if (devfreq->profile->freq_table)
		if (devfreq_update_status(devfreq, freq))
			dev_err(&devfreq->dev,
				"Couldn't update frequency transition information.\n");

	devfreq->previous_freq = freq;
	return err;
}
EXPORT_SYMBOL(update_devfreq);

static void devfreq_monitor(struct work_struct *work)
{
	int err;
	struct devfreq *devfreq = container_of(work,
					struct devfreq, work.work);

	mutex_lock(&devfreq->lock);
	err = update_devfreq(devfreq);
	if (err)
		dev_err(&devfreq->dev, "dvfs failed with (%d) error\n", err);

	queue_delayed_work(devfreq_wq, &devfreq->work,
				msecs_to_jiffies(devfreq->profile->polling_ms));
	mutex_unlock(&devfreq->lock);
}

void devfreq_monitor_start(struct devfreq *devfreq)
{
	INIT_DELAYED_WORK_DEFERRABLE(&devfreq->work, devfreq_monitor);
	if (devfreq->profile->polling_ms)
		queue_delayed_work(devfreq_wq, &devfreq->work,
			msecs_to_jiffies(devfreq->profile->polling_ms));
}
EXPORT_SYMBOL(devfreq_monitor_start);

void devfreq_monitor_stop(struct devfreq *devfreq)
{
	cancel_delayed_work_sync(&devfreq->work);
}
EXPORT_SYMBOL(devfreq_monitor_stop);

void devfreq_monitor_suspend(struct devfreq *devfreq)
{
	mutex_lock(&devfreq->lock);
	if (devfreq->stop_polling) {
		mutex_unlock(&devfreq->lock);
		return;
	}

	devfreq->stop_polling = true;
	mutex_unlock(&devfreq->lock);
	cancel_delayed_work_sync(&devfreq->work);
}
EXPORT_SYMBOL(devfreq_monitor_suspend);

void devfreq_monitor_resume(struct devfreq *devfreq)
{
	mutex_lock(&devfreq->lock);
	if (!devfreq->stop_polling)
		goto out;

	if (!delayed_work_pending(&devfreq->work) &&
			devfreq->profile->polling_ms)
		queue_delayed_work(devfreq_wq, &devfreq->work,
			msecs_to_jiffies(devfreq->profile->polling_ms));
	devfreq->stop_polling = false;

out:
	mutex_unlock(&devfreq->lock);
}
EXPORT_SYMBOL(devfreq_monitor_resume);

void devfreq_interval_update(struct devfreq *devfreq, unsigned int *delay)
{
	unsigned int cur_delay = devfreq->profile->polling_ms;
	unsigned int new_delay = *delay;

	mutex_lock(&devfreq->lock);
	devfreq->profile->polling_ms = new_delay;

	if (devfreq->stop_polling)
		goto out;

	
	if (!new_delay) {
		mutex_unlock(&devfreq->lock);
		cancel_delayed_work_sync(&devfreq->work);
		return;
	}

	
	if (!cur_delay) {
		queue_delayed_work(devfreq_wq, &devfreq->work,
			msecs_to_jiffies(devfreq->profile->polling_ms));
		goto out;
	}

	
	if (cur_delay > new_delay) {
		mutex_unlock(&devfreq->lock);
		cancel_delayed_work_sync(&devfreq->work);
		mutex_lock(&devfreq->lock);
		if (!devfreq->stop_polling)
			queue_delayed_work(devfreq_wq, &devfreq->work,
			      msecs_to_jiffies(devfreq->profile->polling_ms));
	}
out:
	mutex_unlock(&devfreq->lock);
}
EXPORT_SYMBOL(devfreq_interval_update);

static int devfreq_notifier_call(struct notifier_block *nb, unsigned long type,
				 void *devp)
{
	struct devfreq *devfreq = container_of(nb, struct devfreq, nb);
	int ret;

	mutex_lock(&devfreq->lock);
	ret = update_devfreq(devfreq);
	mutex_unlock(&devfreq->lock);

	return ret;
}

static void _remove_devfreq(struct devfreq *devfreq, bool skip)
{
	mutex_lock(&devfreq_list_lock);
	if (IS_ERR(find_device_devfreq(devfreq->dev.parent))) {
		mutex_unlock(&devfreq_list_lock);
		dev_warn(&devfreq->dev, "releasing devfreq which doesn't exist\n");
		return;
	}
	list_del(&devfreq->node);
	mutex_unlock(&devfreq_list_lock);

	if (devfreq->governor)
		devfreq->governor->event_handler(devfreq,
						 DEVFREQ_GOV_STOP, NULL);

	if (devfreq->profile->exit)
		devfreq->profile->exit(devfreq->dev.parent);

	if (!skip && get_device(&devfreq->dev)) {
		device_unregister(&devfreq->dev);
		put_device(&devfreq->dev);
	}

	mutex_destroy(&devfreq->lock);
	kfree(devfreq);
}

static void devfreq_dev_release(struct device *dev)
{
	struct devfreq *devfreq = to_devfreq(dev);

	_remove_devfreq(devfreq, true);
}

static void *find_governor_data(struct devfreq_dev_profile *profile,
				const char *governor_name)
{
	void *data = NULL;
	int i;

	if (profile->governor_data == NULL)
		return NULL;

	for (i = 0; i < profile->num_governor_data; i++) {
		if (strncmp(governor_name, profile->governor_data[i].name,
			     DEVFREQ_NAME_LEN) == 0) {
			data = profile->governor_data[i].data;
			break;
		}
	}
	return data;
}

struct devfreq *devfreq_add_device(struct device *dev,
				   struct devfreq_dev_profile *profile,
				   const char *governor_name,
				   void *data)
{
	struct devfreq *devfreq;
	struct devfreq_governor *governor;
	int err = 0;

	if (!dev || !profile || !governor_name) {
		dev_err(dev, "%s: Invalid parameters.\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	mutex_lock(&devfreq_list_lock);
	devfreq = find_device_devfreq(dev);
	mutex_unlock(&devfreq_list_lock);
	if (!IS_ERR(devfreq)) {
		dev_err(dev, "%s: Unable to create devfreq for the device. It already has one.\n", __func__);
		err = -EINVAL;
		goto err_out;
	}

	devfreq = kzalloc(sizeof(struct devfreq), GFP_KERNEL);
	if (!devfreq) {
		dev_err(dev, "%s: Unable to create devfreq for the device\n",
			__func__);
		err = -ENOMEM;
		goto err_out;
	}

	mutex_init(&devfreq->lock);
	mutex_lock(&devfreq->lock);
	devfreq->dev.parent = dev;
	devfreq->dev.class = devfreq_class;
	devfreq->dev.release = devfreq_dev_release;
	devfreq->profile = profile;
	strncpy(devfreq->governor_name, governor_name, DEVFREQ_NAME_LEN - 1);
	devfreq->previous_freq = profile->initial_freq;

	devfreq->data = data ? data : find_governor_data(devfreq->profile,
							 governor_name);

	devfreq->nb.notifier_call = devfreq_notifier_call;

	devfreq->trans_table =	devm_kzalloc(dev, sizeof(unsigned int) *
						devfreq->profile->max_state *
						devfreq->profile->max_state,
						GFP_KERNEL);
	devfreq->time_in_state = devm_kzalloc(dev, sizeof(unsigned int) *
						devfreq->profile->max_state,
						GFP_KERNEL);
	devfreq->last_stat_updated = jiffies;
	devfreq_set_freq_limits(devfreq);

	dev_set_name(&devfreq->dev, dev_name(dev));
	err = device_register(&devfreq->dev);
	if (err) {
		put_device(&devfreq->dev);
		mutex_unlock(&devfreq->lock);
		goto err_dev;
	}

	mutex_unlock(&devfreq->lock);

	mutex_lock(&devfreq_list_lock);
	list_add(&devfreq->node, &devfreq_list);

	governor = find_devfreq_governor(devfreq->governor_name);
	if (!IS_ERR(governor))
		devfreq->governor = governor;
	if (devfreq->governor)
		err = devfreq->governor->event_handler(devfreq,
					DEVFREQ_GOV_START, NULL);
	mutex_unlock(&devfreq_list_lock);
	if (err) {
		dev_err(dev, "%s: Unable to start governor for the device\n",
			__func__);
		goto err_init;
	}

	return devfreq;

err_init:
	list_del(&devfreq->node);
	device_unregister(&devfreq->dev);
err_dev:
	kfree(devfreq);
err_out:
	return ERR_PTR(err);
}
EXPORT_SYMBOL(devfreq_add_device);

int devfreq_remove_device(struct devfreq *devfreq)
{
	if (!devfreq)
		return -EINVAL;

	_remove_devfreq(devfreq, false);

	return 0;
}
EXPORT_SYMBOL(devfreq_remove_device);

int devfreq_suspend_device(struct devfreq *devfreq)
{
	if (!devfreq)
		return -EINVAL;

	if (!devfreq->governor)
		return 0;

	return devfreq->governor->event_handler(devfreq,
				DEVFREQ_GOV_SUSPEND, NULL);
}
EXPORT_SYMBOL(devfreq_suspend_device);

int devfreq_resume_device(struct devfreq *devfreq)
{
	if (!devfreq)
		return -EINVAL;

	if (!devfreq->governor)
		return 0;

	return devfreq->governor->event_handler(devfreq,
				DEVFREQ_GOV_RESUME, NULL);
}
EXPORT_SYMBOL(devfreq_resume_device);

int devfreq_add_governor(struct devfreq_governor *governor)
{
	struct devfreq_governor *g;
	struct devfreq *devfreq;
	int err = 0;

	if (!governor) {
		pr_err("%s: Invalid parameters.\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&devfreq_list_lock);
	g = find_devfreq_governor(governor->name);
	if (!IS_ERR(g)) {
		pr_err("%s: governor %s already registered\n", __func__,
		       g->name);
		err = -EINVAL;
		goto err_out;
	}

	list_add(&governor->node, &devfreq_governor_list);

	list_for_each_entry(devfreq, &devfreq_list, node) {
		int ret = 0;
		struct device *dev = devfreq->dev.parent;

		if (!strncmp(devfreq->governor_name, governor->name,
			     DEVFREQ_NAME_LEN)) {
			
			if (devfreq->governor) {
				dev_warn(dev,
					 "%s: Governor %s already present\n",
					 __func__, devfreq->governor->name);
				ret = devfreq->governor->event_handler(devfreq,
							DEVFREQ_GOV_STOP, NULL);
				if (ret) {
					dev_warn(dev,
						 "%s: Governor %s stop = %d\n",
						 __func__,
						 devfreq->governor->name, ret);
				}
				
			}
			devfreq->governor = governor;
			ret = devfreq->governor->event_handler(devfreq,
						DEVFREQ_GOV_START, NULL);
			if (ret) {
				dev_warn(dev, "%s: Governor %s start=%d\n",
					 __func__, devfreq->governor->name,
					 ret);
			}
		}
	}

err_out:
	mutex_unlock(&devfreq_list_lock);

	return err;
}
EXPORT_SYMBOL(devfreq_add_governor);

int devfreq_remove_governor(struct devfreq_governor *governor)
{
	struct devfreq_governor *g;
	struct devfreq *devfreq;
	int err = 0;

	if (!governor) {
		pr_err("%s: Invalid parameters.\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&devfreq_list_lock);
	g = find_devfreq_governor(governor->name);
	if (IS_ERR(g)) {
		pr_err("%s: governor %s not registered\n", __func__,
		       governor->name);
		err = PTR_ERR(g);
		goto err_out;
	}
	list_for_each_entry(devfreq, &devfreq_list, node) {
		int ret;
		struct device *dev = devfreq->dev.parent;

		if (!strncmp(devfreq->governor_name, governor->name,
			     DEVFREQ_NAME_LEN)) {
			
			if (!devfreq->governor) {
				dev_warn(dev, "%s: Governor %s NOT present\n",
					 __func__, governor->name);
				continue;
				
			}
			ret = devfreq->governor->event_handler(devfreq,
						DEVFREQ_GOV_STOP, NULL);
			if (ret) {
				dev_warn(dev, "%s: Governor %s stop=%d\n",
					 __func__, devfreq->governor->name,
					 ret);
			}
			devfreq->governor = NULL;
		}
	}

	list_del(&governor->node);
err_out:
	mutex_unlock(&devfreq_list_lock);

	return err;
}
EXPORT_SYMBOL(devfreq_remove_governor);

static ssize_t show_governor(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	if (!to_devfreq(dev)->governor)
		return -EINVAL;

	return sprintf(buf, "%s\n", to_devfreq(dev)->governor->name);
}

static ssize_t store_governor(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct devfreq *df = to_devfreq(dev);
	int ret;
	char str_governor[DEVFREQ_NAME_LEN + 1];
	struct devfreq_governor *governor;

	ret = sscanf(buf, "%" __stringify(DEVFREQ_NAME_LEN) "s", str_governor);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&devfreq_list_lock);
	governor = find_devfreq_governor(str_governor);
	if (IS_ERR(governor)) {
		ret = PTR_ERR(governor);
		goto out;
	}
	if (df->governor == governor)
		goto out;

	if (df->governor) {
		ret = df->governor->event_handler(df, DEVFREQ_GOV_STOP, NULL);
		if (ret) {
			dev_warn(dev, "%s: Governor %s not stopped(%d)\n",
				 __func__, df->governor->name, ret);
			goto out;
		}
	}
	df->data = find_governor_data(df->profile, str_governor);
	df->governor = governor;
	strncpy(df->governor_name, governor->name, DEVFREQ_NAME_LEN);
	ret = df->governor->event_handler(df, DEVFREQ_GOV_START, NULL);
	if (ret)
		dev_warn(dev, "%s: Governor %s not started(%d)\n",
			 __func__, df->governor->name, ret);
out:
	mutex_unlock(&devfreq_list_lock);

	if (!ret)
		ret = count;
	return ret;
}
static ssize_t show_available_governors(struct device *d,
				    struct device_attribute *attr,
				    char *buf)
{
	struct devfreq_governor *tmp_governor;
	ssize_t count = 0;

	mutex_lock(&devfreq_list_lock);
	list_for_each_entry(tmp_governor, &devfreq_governor_list, node)
		count += scnprintf(&buf[count], (PAGE_SIZE - count - 2),
				   "%s ", tmp_governor->name);
	mutex_unlock(&devfreq_list_lock);

	
	if (count)
		count--;

	count += sprintf(&buf[count], "\n");

	return count;
}

static ssize_t show_freq(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	unsigned long freq;
	struct devfreq *devfreq = to_devfreq(dev);

	if (devfreq->profile->get_cur_freq &&
		!devfreq->profile->get_cur_freq(devfreq->dev.parent, &freq))
			return sprintf(buf, "%lu\n", freq);

	return sprintf(buf, "%lu\n", devfreq->previous_freq);
}

static ssize_t show_target_freq(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", to_devfreq(dev)->previous_freq);
}

static ssize_t show_polling_interval(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", to_devfreq(dev)->profile->polling_ms);
}

static ssize_t store_polling_interval(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct devfreq *df = to_devfreq(dev);
	unsigned int value;
	int ret;

	if (!df->governor)
		return -EINVAL;

	ret = sscanf(buf, "%u", &value);
	if (ret != 1)
		return -EINVAL;

	df->governor->event_handler(df, DEVFREQ_GOV_INTERVAL, &value);
	ret = count;

	return ret;
}

static ssize_t store_min_freq(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct devfreq *df = to_devfreq(dev);
	unsigned long value;
	int ret;
	unsigned long max;

	ret = sscanf(buf, "%lu", &value);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&df->lock);
	max = df->max_freq;
	if (value && max && value > max) {
		ret = -EINVAL;
		goto unlock;
	}

	df->min_freq = value;
	update_devfreq(df);
	ret = count;
unlock:
	mutex_unlock(&df->lock);
	return ret;
}

static ssize_t show_min_freq(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%lu\n", to_devfreq(dev)->min_freq);
}

static ssize_t store_max_freq(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct devfreq *df = to_devfreq(dev);
	unsigned long value;
	int ret;
	unsigned long min;

	ret = sscanf(buf, "%lu", &value);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&df->lock);
	min = df->min_freq;
	if (value && min && value < min) {
		ret = -EINVAL;
		goto unlock;
	}

	df->max_freq = value;
	update_devfreq(df);
	ret = count;
unlock:
	mutex_unlock(&df->lock);
	return ret;
}

static ssize_t show_max_freq(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%lu\n", to_devfreq(dev)->max_freq);
}

static ssize_t show_available_freqs(struct device *d,
				    struct device_attribute *attr,
				    char *buf)
{
	struct devfreq *df = to_devfreq(d);
	struct device *dev = df->dev.parent;
	struct opp *opp;
	ssize_t count = 0;
	unsigned long freq = 0;

	rcu_read_lock();
	do {
		opp = opp_find_freq_ceil(dev, &freq);
		if (IS_ERR(opp))
			break;

		count += scnprintf(&buf[count], (PAGE_SIZE - count - 2),
				   "%lu ", freq);
		freq++;
	} while (1);
	rcu_read_unlock();

	
	if (count)
		count--;

	count += sprintf(&buf[count], "\n");

	return count;
}

static ssize_t show_trans_table(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct devfreq *devfreq = to_devfreq(dev);
	ssize_t len;
	int i, j, err;
	unsigned int max_state = devfreq->profile->max_state;

	err = devfreq_update_status(devfreq, devfreq->previous_freq);
	if (err)
		return 0;

	len = sprintf(buf, "   From  :   To\n");
	len += sprintf(buf + len, "         :");
	for (i = 0; i < max_state; i++)
		len += sprintf(buf + len, "%8u",
				devfreq->profile->freq_table[i]);

	len += sprintf(buf + len, "   time(ms)\n");

	for (i = 0; i < max_state; i++) {
		if (devfreq->profile->freq_table[i]
					== devfreq->previous_freq) {
			len += sprintf(buf + len, "*");
		} else {
			len += sprintf(buf + len, " ");
		}
		len += sprintf(buf + len, "%8u:",
				devfreq->profile->freq_table[i]);
		for (j = 0; j < max_state; j++)
			len += sprintf(buf + len, "%8u",
				devfreq->trans_table[(i * max_state) + j]);
		len += sprintf(buf + len, "%10u\n",
			jiffies_to_msecs(devfreq->time_in_state[i]));
	}

	len += sprintf(buf + len, "Total transition : %u\n",
					devfreq->total_trans);
	return len;
}

static struct device_attribute devfreq_attrs[] = {
	__ATTR(governor, S_IRUGO | S_IWUSR, show_governor, store_governor),
	__ATTR(available_governors, S_IRUGO, show_available_governors, NULL),
	__ATTR(cur_freq, S_IRUGO, show_freq, NULL),
	__ATTR(available_frequencies, S_IRUGO, show_available_freqs, NULL),
	__ATTR(target_freq, S_IRUGO, show_target_freq, NULL),
	__ATTR(polling_interval, S_IRUGO | S_IWUSR, show_polling_interval,
	       store_polling_interval),
	__ATTR(min_freq, S_IRUGO | S_IWUSR, show_min_freq, store_min_freq),
	__ATTR(max_freq, S_IRUGO | S_IWUSR, show_max_freq, store_max_freq),
	__ATTR(trans_stat, S_IRUGO, show_trans_table, NULL),
	{ },
};

static int __init devfreq_init(void)
{
	devfreq_class = class_create(THIS_MODULE, "devfreq");
	if (IS_ERR(devfreq_class)) {
		pr_err("%s: couldn't create class\n", __FILE__);
		return PTR_ERR(devfreq_class);
	}

	devfreq_wq = create_freezable_workqueue("devfreq_wq");
	if (IS_ERR(devfreq_wq)) {
		class_destroy(devfreq_class);
		pr_err("%s: couldn't create workqueue\n", __FILE__);
		return PTR_ERR(devfreq_wq);
	}
	devfreq_class->dev_attrs = devfreq_attrs;

	return 0;
}
subsys_initcall(devfreq_init);

static void __exit devfreq_exit(void)
{
	class_destroy(devfreq_class);
	destroy_workqueue(devfreq_wq);
}
module_exit(devfreq_exit);


struct opp *devfreq_recommended_opp(struct device *dev, unsigned long *freq,
				    u32 flags)
{
	struct opp *opp;

	if (flags & DEVFREQ_FLAG_LEAST_UPPER_BOUND) {
		
		opp = opp_find_freq_floor(dev, freq);

		
		if (opp == ERR_PTR(-ERANGE))
			opp = opp_find_freq_ceil(dev, freq);
	} else {
		
		opp = opp_find_freq_ceil(dev, freq);

		
		if (opp == ERR_PTR(-ERANGE))
			opp = opp_find_freq_floor(dev, freq);
	}

	return opp;
}

int devfreq_register_opp_notifier(struct device *dev, struct devfreq *devfreq)
{
	struct srcu_notifier_head *nh;
	int ret = 0;

	rcu_read_lock();
	nh = opp_get_notifier(dev);
	if (IS_ERR(nh))
		ret = PTR_ERR(nh);
	rcu_read_unlock();
	if (!ret)
		ret = srcu_notifier_chain_register(nh, &devfreq->nb);

	return ret;
}

int devfreq_unregister_opp_notifier(struct device *dev, struct devfreq *devfreq)
{
	struct srcu_notifier_head *nh;
	int ret = 0;

	rcu_read_lock();
	nh = opp_get_notifier(dev);
	if (IS_ERR(nh))
		ret = PTR_ERR(nh);
	rcu_read_unlock();
	if (!ret)
		ret = srcu_notifier_chain_unregister(nh, &devfreq->nb);

	return ret;
}

MODULE_AUTHOR("MyungJoo Ham <myungjoo.ham@samsung.com>");
MODULE_DESCRIPTION("devfreq class support");
MODULE_LICENSE("GPL");
