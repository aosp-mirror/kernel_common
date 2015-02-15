/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "subsys-restart: %s(): " fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/time.h>
#include <linux/wakelock.h>
#include <linux/suspend.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/idr.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>

#include <asm/current.h>

#include <mach/socinfo.h>
#include <mach/subsystem_notif.h>
#include <mach/subsystem_restart.h>

#include "smd_private.h"

static int enable_debug;
module_param(enable_debug, int, S_IRUGO | S_IWUSR);

enum p_subsys_state {
	SUBSYS_NORMAL,
	SUBSYS_CRASHED,
	SUBSYS_RESTARTING,
};

enum subsys_state {
	SUBSYS_OFFLINE,
	SUBSYS_ONLINE,
};

static const char * const subsys_states[] = {
	[SUBSYS_OFFLINE] = "OFFLINE",
	[SUBSYS_ONLINE] = "ONLINE",
};

static const char * const restart_levels[] = {
	[RESET_SOC] = "SYSTEM",
	[RESET_SUBSYS_COUPLED] = "RELATED",
};

#if defined(CONFIG_HTC_FEATURES_SSR)
static const char * const enable_ramdumps[] = {
	[DISABLE_RAMDUMP] = "DISABLE",
	[ENABLE_RAMDUMP] = "ENABLE",
};
#endif


#if defined(CONFIG_HTC_DEBUG_SSR)

#define RD_BUF_SIZE			  256
#define MODEM_ERRMSG_LIST_LEN 10

struct msm_msr_info {
	int valid;
	struct timespec msr_time;
	char modem_errmsg[RD_BUF_SIZE];
};
int msm_msr_index = 0;
static struct msm_msr_info msr_info_list[MODEM_ERRMSG_LIST_LEN];

static ssize_t subsystem_restart_reason_nonblock_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{

	int i = 0;
	char tmp[RD_BUF_SIZE+30];

	for( i=0; i<MODEM_ERRMSG_LIST_LEN; i++ ) {
		if( msr_info_list[i].valid != 0 ) {
			
			snprintf(tmp, RD_BUF_SIZE+30, "%ld-%s|\n\r", msr_info_list[i].msr_time.tv_sec, msr_info_list[i].modem_errmsg);
			strcat(buf, tmp);
			memset(tmp, 0, RD_BUF_SIZE+30);
		}
		msr_info_list[i].valid = 0;
		memset(msr_info_list[i].modem_errmsg, 0, RD_BUF_SIZE);
	}
	strcat(buf, "\n\r\0");

	return strlen(buf);
}

void subsystem_restart_reason_nonblock_init(void)
{
	int i = 0;
	msm_msr_index = 0;
	for( i=0; i<MODEM_ERRMSG_LIST_LEN; i++ ) {
		msr_info_list[i].valid = 0;
		memset(msr_info_list[i].modem_errmsg, 0, RD_BUF_SIZE);
	}
}

#define subsystem_restart_ro_attr(_name) \
	static struct kobj_attribute _name##_attr = {  \
		.attr   = {                             \
			.name = __stringify(_name),     \
			.mode = 0444,                   \
		},                                      \
		.show   = _name##_show,                 \
		.store  = NULL,         \
	}


subsystem_restart_ro_attr(subsystem_restart_reason_nonblock);


static struct attribute *g[] = {
	&subsystem_restart_reason_nonblock_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

#endif

struct subsys_tracking {
	enum p_subsys_state p_state;
	spinlock_t s_lock;
	enum subsys_state state;
	struct mutex lock;
};

struct subsys_soc_restart_order {
	const char * const *subsystem_list;
	int count;

	struct subsys_tracking track;
	struct subsys_device *subsys_ptrs[];
};

struct restart_log {
	struct timeval time;
	struct subsys_device *dev;
	struct list_head list;
};

struct subsys_device {
	struct subsys_desc *desc;
	struct wake_lock wake_lock;
	char wlname[64];
	struct work_struct work;

	struct subsys_tracking track;

	void *notify;
	struct device dev;
	struct module *owner;
	int count;
	int id;
	int restart_level;
#if defined(CONFIG_HTC_FEATURES_SSR)
	int enable_ramdump;
#endif
#if defined(CONFIG_HTC_DEBUG_SSR)
#define HTC_DEBUG_SSR_REASON_LEN 80
	char restart_reason[HTC_DEBUG_SSR_REASON_LEN];
#endif
	struct subsys_soc_restart_order *restart_order;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dentry;
#endif
	bool do_ramdump_on_put;
	struct miscdevice misc_dev;
	char miscdevice_name[32];
	struct completion err_ready;
	bool crashed;
};

static struct subsys_device *to_subsys(struct device *d)
{
	return container_of(d, struct subsys_device, dev);
}

static ssize_t name_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", to_subsys(dev)->desc->name);
}

static ssize_t state_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	enum subsys_state state = to_subsys(dev)->track.state;
	return snprintf(buf, PAGE_SIZE, "%s\n", subsys_states[state]);
}

static ssize_t
restart_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int level = to_subsys(dev)->restart_level;
	return snprintf(buf, PAGE_SIZE, "%s\n", restart_levels[level]);
}

#if defined(CONFIG_HTC_FEATURES_SSR)
static ssize_t restart_trigger_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct subsys_device *subsys = to_subsys(dev);
	char *s = buf;
	int ret = 0;

	if(subsys->restart_level == RESET_SUBSYS_COUPLED)
	{
		pr_info("%s: trigger %s restart\n", __func__, subsys->desc->name);

		ret = subsystem_restart_dev(subsys);

		if(ret == 0)
			pr_info("%s: subsystem_restart_dev success\n", __func__);
		else
		{
			pr_info("%s: subsystem_restart_dev faild => ret = %d\n", __func__, ret);
			s += sprintf(buf, "Failed");
			return s - buf;
		}

		s += sprintf(buf, "Success");
	}
	else
	{
		pr_info("%s: %s restart did not enable", __func__, subsys->desc->name);
		s += sprintf(buf, "Please check whether %s ssr enabled or not", subsys->desc->name);
	}

	return s - buf;
}
#endif

static ssize_t restart_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct subsys_device *subsys = to_subsys(dev);
	int i;
	const char *p;

	p = memchr(buf, '\n', count);
	if (p)
		count = p - buf;

	for (i = 0; i < ARRAY_SIZE(restart_levels); i++)
		if (!strncasecmp(buf, restart_levels[i], count)) {
			subsys->restart_level = i;
			return count;
		}
	return -EPERM;
}

#if defined(CONFIG_HTC_FEATURES_SSR)
static ssize_t enable_ramdump_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int enable_ramdump = to_subsys(dev)->enable_ramdump;
	return snprintf(buf, PAGE_SIZE, "%s\n", enable_ramdumps[enable_ramdump]);
}

static ssize_t enable_ramdump_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct subsys_device *subsys = to_subsys(dev);
	int i;
	const char *p;

	p = memchr(buf, '\n', count);
	if (p)
		count = p - buf;

	for (i = 0; i < ARRAY_SIZE(enable_ramdumps); i++)
		if (!strncasecmp(buf, enable_ramdumps[i], count)) {
			subsys->enable_ramdump = i;
			return count;
		}
	return -EPERM;
}

void subsys_set_enable_ramdump(struct subsys_device *dev, int enable)
{
	dev->enable_ramdump = enable;
}
EXPORT_SYMBOL(subsys_set_enable_ramdump);

void subsys_set_restart_level(struct subsys_device *dev, int level)
{
	dev->restart_level = level;
}
EXPORT_SYMBOL(subsys_set_restart_level);
#endif

int subsys_get_restart_level(struct subsys_device *dev)
{
	return dev->restart_level;
}
EXPORT_SYMBOL(subsys_get_restart_level);

#if defined(CONFIG_HTC_DEBUG_SSR)
void subsys_set_restart_reason(struct subsys_device *dev, const char* reason)
{
	if (!dev || !reason)
		return;
	snprintf(dev->restart_reason, sizeof(dev->restart_reason) - 1, "%s", reason);
}
EXPORT_SYMBOL(subsys_set_restart_reason);
#endif 

static ssize_t crashed_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
    bool crashed = to_subsys(dev)->crashed;
    if (crashed)
	    return snprintf(buf, PAGE_SIZE, "%s\n", "TRUE");
    else
        return snprintf(buf, PAGE_SIZE, "%s\n", "FALSE");
}

static ssize_t crashed_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct subsys_device *subsys = to_subsys(dev);
	const char *p;

	p = memchr(buf, '\n', count);
	if (p)
		count = p - buf;

	if (!strncasecmp(buf, "FALSE", count)) {
			subsys->crashed = false;
			return count;
	}
	return -EPERM;
}

static void subsys_set_state(struct subsys_device *subsys,
			     enum subsys_state state)
{
	unsigned long flags;

	spin_lock_irqsave(&subsys->track.s_lock, flags);
	if (subsys->track.state != state) {
		subsys->track.state = state;
		spin_unlock_irqrestore(&subsys->track.s_lock, flags);
		sysfs_notify(&subsys->dev.kobj, NULL, "state");
		return;
	}
	spin_unlock_irqrestore(&subsys->track.s_lock, flags);
}

void subsys_default_online(struct subsys_device *dev)
{
	subsys_set_state(dev, SUBSYS_ONLINE);
}
EXPORT_SYMBOL(subsys_default_online);

static struct device_attribute subsys_attrs[] = {
	__ATTR_RO(name),
	__ATTR_RO(state),
#if defined(CONFIG_HTC_FEATURES_SSR)
	__ATTR_RO(restart_trigger),
#endif
	__ATTR(restart_level, 0644, restart_level_show, restart_level_store),
#if defined(CONFIG_HTC_FEATURES_SSR)
	__ATTR(enable_ramdump, 0644, enable_ramdump_show, enable_ramdump_store),
#endif
	__ATTR(crashed, 0644, crashed_show, crashed_store),
	__ATTR_NULL,
};

static struct bus_type subsys_bus_type = {
	.name		= "msm_subsys",
	.dev_attrs	= subsys_attrs,
};

static DEFINE_IDA(subsys_ida);

#if !defined(CONFIG_HTC_FEATURES_SSR)
static int enable_ramdumps;
module_param(enable_ramdumps, int, S_IRUGO | S_IWUSR);
#endif

struct workqueue_struct *ssr_wq;

static LIST_HEAD(restart_log_list);
static DEFINE_MUTEX(soc_order_reg_lock);
static DEFINE_MUTEX(restart_log_mutex);


#define DEFINE_SINGLE_RESTART_ORDER(name, order)		\
	static struct subsys_soc_restart_order __##name = {	\
		.subsystem_list = order,			\
		.count = ARRAY_SIZE(order),			\
		.subsys_ptrs = {[ARRAY_SIZE(order)] = NULL}	\
	};							\
	static struct subsys_soc_restart_order *name[] = {      \
		&__##name,					\
	}

static const char * const _order_8x60_all[] = {
	"external_modem",  "modem", "adsp"
};
DEFINE_SINGLE_RESTART_ORDER(orders_8x60_all, _order_8x60_all);

static const char * const _order_8x60_modems[] = {"external_modem", "modem"};
DEFINE_SINGLE_RESTART_ORDER(orders_8x60_modems, _order_8x60_modems);

static const char * const order_8960_sglte[] = {"external_modem",
						"modem"};

static struct subsys_soc_restart_order restart_orders_8960_fusion_sglte = {
	.subsystem_list = order_8960_sglte,
	.count = ARRAY_SIZE(order_8960_sglte),
	.subsys_ptrs = {[ARRAY_SIZE(order_8960_sglte)] = NULL}
	};

static struct subsys_soc_restart_order *restart_orders_8960_sglte[] = {
	&restart_orders_8960_fusion_sglte,
	};

static struct subsys_soc_restart_order **restart_orders;
static int n_restart_orders;

static struct subsys_soc_restart_order *
update_restart_order(struct subsys_device *dev)
{
	int i, j;
	struct subsys_soc_restart_order *order;
	const char *name = dev->desc->name;
	int len = SUBSYS_NAME_MAX_LENGTH;

	mutex_lock(&soc_order_reg_lock);
	for (j = 0; j < n_restart_orders; j++) {
		order = restart_orders[j];
		for (i = 0; i < order->count; i++) {
			if (!strncmp(order->subsystem_list[i], name, len)) {
				order->subsys_ptrs[i] = dev;
				goto found;
			}
		}
	}
	order = NULL;
found:
	mutex_unlock(&soc_order_reg_lock);

	return order;
}

static int max_restarts;
module_param(max_restarts, int, 0644);

static long max_history_time = 3600;
module_param(max_history_time, long, 0644);

static void do_epoch_check(struct subsys_device *dev)
{
	int n = 0;
	struct timeval *time_first = NULL, *curr_time;
	struct restart_log *r_log, *temp;
	static int max_restarts_check;
	static long max_history_time_check;

	mutex_lock(&restart_log_mutex);

	max_restarts_check = max_restarts;
	max_history_time_check = max_history_time;

	
	if (!max_restarts_check)
		goto out;

	r_log = kmalloc(sizeof(struct restart_log), GFP_KERNEL);
	if (!r_log)
		goto out;
	r_log->dev = dev;
	do_gettimeofday(&r_log->time);
	curr_time = &r_log->time;
	INIT_LIST_HEAD(&r_log->list);

	list_add_tail(&r_log->list, &restart_log_list);

	list_for_each_entry_safe(r_log, temp, &restart_log_list, list) {

		if ((curr_time->tv_sec - r_log->time.tv_sec) >
				max_history_time_check) {

			pr_debug("Deleted node with restart_time = %ld\n",
					r_log->time.tv_sec);
			list_del(&r_log->list);
			kfree(r_log);
			continue;
		}
		if (!n) {
			time_first = &r_log->time;
			pr_debug("Time_first: %ld\n", time_first->tv_sec);
		}
		n++;
		pr_debug("Restart_time: %ld\n", r_log->time.tv_sec);
	}

	if (time_first && n >= max_restarts_check) {
		if ((curr_time->tv_sec - time_first->tv_sec) <
				max_history_time_check)
			panic("Subsystems have crashed %d times in less than "
				"%ld seconds!", max_restarts_check,
				max_history_time_check);
	}

out:
	mutex_unlock(&restart_log_mutex);
}

static void for_each_subsys_device(struct subsys_device **list, unsigned count,
		void *data, void (*fn)(struct subsys_device *, void *))
{
	while (count--) {
		struct subsys_device *dev = *list++;
		if (!dev)
			continue;
		fn(dev, data);
	}
}

static void notify_each_subsys_device(struct subsys_device **list,
		unsigned count,
		enum subsys_notif_type notif, void *data)
{
	while (count--) {
		enum subsys_notif_type type = (enum subsys_notif_type)type;
		struct subsys_device *dev = *list++;
		if (!dev)
			continue;
		subsys_notif_queue_notification(dev->notify, notif, data);
	}
}

static int wait_for_err_ready(struct subsys_device *subsys)
{
	int ret;

	if (!subsys->desc->err_ready_irq || enable_debug == 1)
		return 0;

	ret = wait_for_completion_timeout(&subsys->err_ready,
					  msecs_to_jiffies(10000));
	if (!ret) {
		pr_err("[%s]: Error ready timed out\n", subsys->desc->name);
		return -ETIMEDOUT;
	}

	return 0;
}

static void subsystem_shutdown(struct subsys_device *dev, void *data)
{
	const char *name = dev->desc->name;

	pr_info("[%p]: Shutting down %s\n", current, name);
	if (dev->desc->shutdown(dev->desc) < 0)
		panic("subsys-restart: [%p]: Failed to shutdown %s!",
			current, name);
	subsys_set_state(dev, SUBSYS_OFFLINE);
}

static void subsystem_ramdump(struct subsys_device *dev, void *data)
{
	const char *name = dev->desc->name;

	if (dev->desc->ramdump)
#if defined(CONFIG_HTC_FEATURES_SSR)
		if (dev->desc->ramdump(dev->enable_ramdump, dev->desc) < 0)
#else
		if (dev->desc->ramdump(enable_ramdumps, dev->desc) < 0)
#endif
			pr_warn("%s[%p]: Ramdump failed.\n", name, current);
	dev->do_ramdump_on_put = false;
}

static void subsystem_powerup(struct subsys_device *dev, void *data)
{
	const char *name = dev->desc->name;
	int ret;

	pr_info("[%p]: Powering up %s\n", current, name);
	init_completion(&dev->err_ready);

	if (dev->desc->powerup(dev->desc) < 0) {
		notify_each_subsys_device(&dev, 1, SUBSYS_POWERUP_FAILURE,
								NULL);
		panic("[%p]: Powerup error: %s!", current, name);
	}

	ret = wait_for_err_ready(dev);
	if (ret) {
		notify_each_subsys_device(&dev, 1, SUBSYS_POWERUP_FAILURE,
								NULL);
		panic("[%p]: Timed out waiting for error ready: %s!",
			current, name);
	}
	subsys_set_state(dev, SUBSYS_ONLINE);
}

static int __find_subsys(struct device *dev, void *data)
{
	struct subsys_device *subsys = to_subsys(dev);
	return !strcmp(subsys->desc->name, data);
}

static struct subsys_device *find_subsys(const char *str)
{
	struct device *dev;

	if (!str)
		return NULL;

	dev = bus_find_device(&subsys_bus_type, NULL, (void *)str,
			__find_subsys);
	return dev ? to_subsys(dev) : NULL;
}

static int subsys_start(struct subsys_device *subsys)
{
	int ret;

	init_completion(&subsys->err_ready);
	ret = subsys->desc->start(subsys->desc);
	if (ret){
		notify_each_subsys_device(&subsys, 1, SUBSYS_POWERUP_FAILURE,
									NULL);
		return ret;
	}

	if (subsys->desc->is_not_loadable) {
		subsys_set_state(subsys, SUBSYS_ONLINE);
		return 0;
	}

	ret = wait_for_err_ready(subsys);
	if (ret) {
		notify_each_subsys_device(&subsys, 1, SUBSYS_POWERUP_FAILURE,
									NULL);
		subsys->desc->stop(subsys->desc);
	} else
		subsys_set_state(subsys, SUBSYS_ONLINE);

	return ret;
}

static void subsys_stop(struct subsys_device *subsys)
{
	subsys->desc->stop(subsys->desc);
	subsys_set_state(subsys, SUBSYS_OFFLINE);
}

static struct subsys_tracking *subsys_get_track(struct subsys_device *subsys)
{
	struct subsys_soc_restart_order *order = subsys->restart_order;

	if (order)
		return &order->track;
	else
		return &subsys->track;
}

void *subsystem_get(const char *name)
{
	struct subsys_device *subsys;
	struct subsys_device *subsys_d;
	int ret;
	void *retval;
	struct subsys_tracking *track;

	if (!name)
		return NULL;

	subsys = retval = find_subsys(name);
	if (!subsys)
		return ERR_PTR(-ENODEV);
	if (!try_module_get(subsys->owner)) {
		retval = ERR_PTR(-ENODEV);
		goto err_module;
	}

	subsys_d = subsystem_get(subsys->desc->depends_on);
	if (IS_ERR(subsys_d)) {
		retval = subsys_d;
		goto err_depends;
	}

	track = subsys_get_track(subsys);
	mutex_lock(&track->lock);
	if (!subsys->count) {
		ret = subsys_start(subsys);
		if (ret) {
			retval = ERR_PTR(ret);
			goto err_start;
		}
	}
	subsys->count++;
	mutex_unlock(&track->lock);
	return retval;
err_start:
	mutex_unlock(&track->lock);
	subsystem_put(subsys_d);
err_depends:
	module_put(subsys->owner);
err_module:
	put_device(&subsys->dev);
	return retval;
}
EXPORT_SYMBOL(subsystem_get);

void subsystem_put(void *subsystem)
{
	struct subsys_device *subsys_d, *subsys = subsystem;
	struct subsys_tracking *track;

	if (IS_ERR_OR_NULL(subsys))
		return;

	if (subsys->crashed)
		return;

	track = subsys_get_track(subsys);
	mutex_lock(&track->lock);
	if (WARN(!subsys->count, "%s: %s: Reference count mismatch\n",
			subsys->desc->name, __func__))
		goto err_out;
	if (!--subsys->count) {
		subsys_stop(subsys);
		if (subsys->do_ramdump_on_put)
			subsystem_ramdump(subsys, NULL);
	}
	mutex_unlock(&track->lock);

	subsys_d = find_subsys(subsys->desc->depends_on);
	if (subsys_d) {
		subsystem_put(subsys_d);
		put_device(&subsys_d->dev);
	}
	module_put(subsys->owner);
	put_device(&subsys->dev);
	return;
err_out:
	mutex_unlock(&track->lock);
}
EXPORT_SYMBOL(subsystem_put);

static void subsystem_restart_wq_func(struct work_struct *work)
{
	struct subsys_device *dev = container_of(work,
						struct subsys_device, work);
	struct subsys_device **list;
	struct subsys_desc *desc = dev->desc;
	struct subsys_soc_restart_order *order = dev->restart_order;
	struct subsys_tracking *track;
	unsigned count;
	unsigned long flags;

	if (order) {
		list = order->subsys_ptrs;
		count = order->count;
		track = &order->track;
	} else {
		list = &dev;
		count = 1;
		track = &dev->track;
	}

	mutex_lock(&track->lock);
	do_epoch_check(dev);

	mutex_lock(&soc_order_reg_lock);

	pr_debug("[%p]: Starting restart sequence for %s\n", current,
			desc->name);
	notify_each_subsys_device(list, count, SUBSYS_BEFORE_SHUTDOWN, NULL);
	for_each_subsys_device(list, count, NULL, subsystem_shutdown);
	notify_each_subsys_device(list, count, SUBSYS_AFTER_SHUTDOWN, NULL);

	notify_each_subsys_device(list, count, SUBSYS_RAMDUMP_NOTIFICATION,
#if defined(CONFIG_HTC_FEATURES_SSR)
							  &(dev->enable_ramdump));
#else
							  &enable_ramdumps);
#endif

	spin_lock_irqsave(&track->s_lock, flags);
	track->p_state = SUBSYS_RESTARTING;
	spin_unlock_irqrestore(&track->s_lock, flags);

	
	for_each_subsys_device(list, count, NULL, subsystem_ramdump);

	notify_each_subsys_device(list, count, SUBSYS_BEFORE_POWERUP, NULL);
	for_each_subsys_device(list, count, NULL, subsystem_powerup);
	notify_each_subsys_device(list, count, SUBSYS_AFTER_POWERUP, NULL);

	pr_info("[%p]: Restart sequence for %s completed.\n",
			current, desc->name);

	mutex_unlock(&soc_order_reg_lock);
	mutex_unlock(&track->lock);

	spin_lock_irqsave(&track->s_lock, flags);
	track->p_state = SUBSYS_NORMAL;
	dev->crashed = false;
	wake_unlock(&dev->wake_lock);
	spin_unlock_irqrestore(&track->s_lock, flags);
}

static void __subsystem_restart_dev(struct subsys_device *dev)
{
	struct subsys_desc *desc = dev->desc;
	const char *name = dev->desc->name;
	struct subsys_tracking *track;
	unsigned long flags;

#if defined(CONFIG_HTC_DEBUG_SSR)
	
	if (!strncmp(name, "modem",
				SUBSYS_NAME_MAX_LENGTH)) {
	msr_info_list[msm_msr_index].valid = 1;
	msr_info_list[msm_msr_index].msr_time = current_kernel_time();
	snprintf(msr_info_list[msm_msr_index].modem_errmsg, RD_BUF_SIZE, "%s", dev->restart_reason);
	if(++msm_msr_index >= MODEM_ERRMSG_LIST_LEN)
	msm_msr_index = 0;
		}
   
#endif

#if defined(CONFIG_HTC_FEATURES_SSR)
	pr_info("Restarting %s [level=%s]!\n", desc->name,
			restart_levels[dev->restart_level]);
#else
	pr_debug("Restarting %s [level=%s]!\n", desc->name,
			restart_levels[dev->restart_level]);
#endif

	track = subsys_get_track(dev);
	spin_lock_irqsave(&track->s_lock, flags);
	if (track->p_state != SUBSYS_CRASHED) {
		if (dev->track.state == SUBSYS_ONLINE &&
		    track->p_state != SUBSYS_RESTARTING) {
			track->p_state = SUBSYS_CRASHED;
			wake_lock(&dev->wake_lock);
			queue_work(ssr_wq, &dev->work);
		} else {
			panic("Subsystem %s crashed during SSR!", name);
		}
	}
	spin_unlock_irqrestore(&track->s_lock, flags);
}

int subsystem_restart_dev(struct subsys_device *dev)
{
	const char *name;

	if (!get_device(&dev->dev))
		return -ENODEV;

	if (!try_module_get(dev->owner)) {
		put_device(&dev->dev);
		return -ENODEV;
	}

	name = dev->desc->name;
	if (system_state == SYSTEM_RESTART
		|| system_state == SYSTEM_POWER_OFF) {
		pr_err("%s crashed during a system poweroff/shutdown.\n", name);
		return -EBUSY;
	}

	pr_info("Restart sequence requested for %s, restart_level = %s.\n",
		name, restart_levels[dev->restart_level]);

	switch (dev->restart_level) {

	case RESET_SUBSYS_COUPLED:
		__subsystem_restart_dev(dev);
		break;
	case RESET_SOC:
#if defined(CONFIG_HTC_DEBUG_SSR)
		panic("subsys-restart: %s crashed. %s", name, dev->restart_reason);
#else
		panic("subsys-restart: Resetting the SoC - %s crashed.", name);
#endif
		break;
	default:
		panic("subsys-restart: Unknown restart level!\n");
		break;
	}
	module_put(dev->owner);
	put_device(&dev->dev);

	return 0;
}
EXPORT_SYMBOL(subsystem_restart_dev);

int subsystem_restart(const char *name)
{
	int ret;
	struct subsys_device *dev = find_subsys(name);

	if (!dev)
		return -ENODEV;

	
	
	if (!strncmp(name, "modem", SUBSYS_NAME_MAX_LENGTH) || !strncmp(name, "wcnss", SUBSYS_NAME_MAX_LENGTH))
	{
		dev->crashed = true;
	}

	ret = subsystem_restart_dev(dev);
	put_device(&dev->dev);
	return ret;
}
EXPORT_SYMBOL(subsystem_restart);

int subsystem_crashed(const char *name)
{
	struct subsys_device *dev = find_subsys(name);
	struct subsys_tracking *track;

	if (!dev)
		return -ENODEV;

	if (!get_device(&dev->dev))
		return -ENODEV;

	track = subsys_get_track(dev);

	mutex_lock(&track->lock);
	dev->do_ramdump_on_put = true;
	mutex_unlock(&track->lock);

	put_device(&dev->dev);
	return 0;
}
EXPORT_SYMBOL(subsystem_crashed);

void subsys_set_crash_status(struct subsys_device *dev, bool crashed)
{
	dev->crashed = true;
}

bool subsys_get_crash_status(struct subsys_device *dev)
{
	return dev->crashed;
}
#ifdef CONFIG_DEBUG_FS
static ssize_t subsys_debugfs_read(struct file *filp, char __user *ubuf,
		size_t cnt, loff_t *ppos)
{
	int r;
	char buf[40];
	struct subsys_device *subsys = filp->private_data;

	r = snprintf(buf, sizeof(buf)-1, "%d\n", subsys->count);
	return simple_read_from_buffer(ubuf, cnt, ppos, buf, r);
}

static ssize_t subsys_debugfs_write(struct file *filp,
		const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	struct subsys_device *subsys = filp->private_data;
	char buf[10];
	char *cmp;

	cnt = min(cnt, sizeof(buf) - 1);
	if (copy_from_user(&buf, ubuf, cnt))
		return -EFAULT;
	buf[cnt] = '\0';
	cmp = strstrip(buf);

	if (!strcmp(cmp, "restart")) {
		if (subsystem_restart_dev(subsys))
			return -EIO;
	} else if (!strcmp(cmp, "get")) {
		if (subsystem_get(subsys->desc->name))
			return -EIO;
	} else if (!strcmp(cmp, "put")) {
		subsystem_put(subsys);
	} else {
		return -EINVAL;
	}

	return cnt;
}

static const struct file_operations subsys_debugfs_fops = {
	.open	= simple_open,
	.read	= subsys_debugfs_read,
	.write	= subsys_debugfs_write,
};

static struct dentry *subsys_base_dir;

static int __init subsys_debugfs_init(void)
{
	subsys_base_dir = debugfs_create_dir("msm_subsys", NULL);
	return !subsys_base_dir ? -ENOMEM : 0;
}

static void subsys_debugfs_exit(void)
{
	debugfs_remove_recursive(subsys_base_dir);
}

static int subsys_debugfs_add(struct subsys_device *subsys)
{
	if (!subsys_base_dir)
		return -ENOMEM;

	subsys->dentry = debugfs_create_file(subsys->desc->name,
				S_IRUGO | S_IWUSR, subsys_base_dir,
				subsys, &subsys_debugfs_fops);
	return !subsys->dentry ? -ENOMEM : 0;
}

static void subsys_debugfs_remove(struct subsys_device *subsys)
{
	debugfs_remove(subsys->dentry);
}
#else
static int __init subsys_debugfs_init(void) { return 0; };
static void subsys_debugfs_exit(void) { }
static int subsys_debugfs_add(struct subsys_device *subsys) { return 0; }
static void subsys_debugfs_remove(struct subsys_device *subsys) { }
#endif

static int subsys_device_open(struct inode *inode, struct file *file)
{
	void *retval;
	struct subsys_device *subsys_dev = container_of(file->private_data,
		struct subsys_device, misc_dev);

	if (!file->private_data)
		return -EINVAL;

	retval = subsystem_get(subsys_dev->desc->name);
	if (IS_ERR(retval))
		return PTR_ERR(retval);

	return 0;
}

static int subsys_device_close(struct inode *inode, struct file *file)
{
	struct subsys_device *subsys_dev = container_of(file->private_data,
		struct subsys_device, misc_dev);

	if (!file->private_data)
		return -EINVAL;

	subsystem_put(subsys_dev);

	return 0;
}

static const struct file_operations subsys_device_fops = {
		.owner = THIS_MODULE,
		.open = subsys_device_open,
		.release = subsys_device_close,
};

static void subsys_device_release(struct device *dev)
{
	struct subsys_device *subsys = to_subsys(dev);

	wake_lock_destroy(&subsys->wake_lock);
	mutex_destroy(&subsys->track.lock);
	ida_simple_remove(&subsys_ida, subsys->id);
	kfree(subsys);
}
static irqreturn_t subsys_err_ready_intr_handler(int irq, void *subsys)
{
	struct subsys_device *subsys_dev = subsys;
	dev_info(subsys_dev->desc->dev,
		"Subsystem error monitoring/handling services are up\n");

	if (subsys_dev->desc->is_not_loadable)
		return IRQ_HANDLED;

	complete(&subsys_dev->err_ready);
	return IRQ_HANDLED;
}

static int subsys_misc_device_add(struct subsys_device *subsys_dev)
{
	int ret;
	memset(subsys_dev->miscdevice_name, 0,
			ARRAY_SIZE(subsys_dev->miscdevice_name));
	snprintf(subsys_dev->miscdevice_name,
			 sizeof(subsys_dev->miscdevice_name) - 1, "subsys_%s",
			 subsys_dev->desc->name);

	subsys_dev->misc_dev.minor = MISC_DYNAMIC_MINOR;
	subsys_dev->misc_dev.name = subsys_dev->miscdevice_name;
	subsys_dev->misc_dev.fops = &subsys_device_fops;
	subsys_dev->misc_dev.parent = &subsys_dev->dev;

	ret = misc_register(&subsys_dev->misc_dev);
	if (ret) {
		pr_err("%s: misc_register() failed for %s (%d)", __func__,
				subsys_dev->miscdevice_name, ret);
	}
	return ret;
}

static void subsys_misc_device_remove(struct subsys_device *subsys_dev)
{
	misc_deregister(&subsys_dev->misc_dev);
}

static int __get_gpio(struct subsys_desc *desc, const char *prop,
		int *gpio)
{
	struct device_node *dnode = desc->dev->of_node;
	int ret = -ENOENT;

	if (of_find_property(dnode, prop, NULL)) {
		*gpio = of_get_named_gpio(dnode, prop, 0);
		ret = *gpio < 0 ? *gpio : 0;
	}

	return ret;
}

static int __get_irq(struct subsys_desc *desc, const char *prop,
		unsigned int *irq)
{
	int ret, gpio, irql;

	ret = __get_gpio(desc, prop, &gpio);
	if (ret)
		return ret;

	irql = gpio_to_irq(gpio);

	if (irql == -ENOENT)
		irql = -ENXIO;

	if (irql < 0) {
		pr_err("[%s]: Error getting IRQ \"%s\"\n", desc->name,
				prop);
		return irql;
	} else {
		*irq = irql;
	}

	return 0;
}

static int subsys_parse_devicetree(struct subsys_desc *desc)
{
	int ret;
	struct platform_device *pdev = container_of(desc->dev,
					struct platform_device, dev);

	ret = __get_irq(desc, "qcom,gpio-err-fatal", &desc->err_fatal_irq);
	if (ret && ret != -ENOENT)
		return ret;

	ret = __get_irq(desc, "qcom,gpio-err-ready", &desc->err_ready_irq);
	if (ret && ret != -ENOENT)
		return ret;

	ret = __get_irq(desc, "qcom,gpio-stop-ack", &desc->stop_ack_irq);
	if (ret && ret != -ENOENT)
		return ret;

	ret = __get_gpio(desc, "qcom,gpio-force-stop", &desc->force_stop_gpio);
	if (ret && ret != -ENOENT)
		return ret;

	desc->wdog_bite_irq = platform_get_irq(pdev, 0);
	if (desc->wdog_bite_irq < 0)
		return desc->wdog_bite_irq;

	return 0;
}

static int subsys_setup_irqs(struct subsys_device *subsys)
{
	struct subsys_desc *desc = subsys->desc;
	int ret;

	if (desc->err_fatal_irq && desc->err_fatal_handler) {
		ret = devm_request_irq(desc->dev, desc->err_fatal_irq,
				desc->err_fatal_handler,
				IRQF_TRIGGER_RISING, desc->name, desc);
		if (ret < 0) {
			dev_err(desc->dev, "[%s]: Unable to register error fatal IRQ handler!: %d\n",
				desc->name, ret);
			return ret;
		}
	}

	if (desc->stop_ack_irq && desc->stop_ack_handler) {
		ret = devm_request_irq(desc->dev, desc->stop_ack_irq,
			desc->stop_ack_handler,
			IRQF_TRIGGER_RISING, desc->name, desc);
		if (ret < 0) {
			dev_err(desc->dev, "[%s]: Unable to register stop ack handler!: %d\n",
				desc->name, ret);
			return ret;
		}
	}

	if (desc->wdog_bite_irq && desc->wdog_bite_handler) {
		ret = devm_request_irq(desc->dev, desc->wdog_bite_irq,
			desc->wdog_bite_handler,
			IRQF_TRIGGER_RISING, desc->name, desc);
		if (ret < 0) {
			dev_err(desc->dev, "[%s]: Unable to register wdog bite handler!: %d\n",
				desc->name, ret);
			return ret;
		}
	}

	if (desc->err_ready_irq) {
		ret = devm_request_irq(desc->dev,
					desc->err_ready_irq,
					subsys_err_ready_intr_handler,
					IRQF_TRIGGER_RISING,
					"error_ready_interrupt", subsys);
		if (ret < 0) {
			dev_err(desc->dev,
				"[%s]: Unable to register err ready handler\n",
				desc->name);
			return ret;
		}
	}

	return 0;
}

struct subsys_device *subsys_register(struct subsys_desc *desc)
{
	struct subsys_device *subsys;
	int ret;

	subsys = kzalloc(sizeof(*subsys), GFP_KERNEL);
	if (!subsys)
		return ERR_PTR(-ENOMEM);

	subsys->desc = desc;
	subsys->owner = desc->owner;
	subsys->dev.parent = desc->dev;
	subsys->dev.bus = &subsys_bus_type;
	subsys->dev.release = subsys_device_release;

	subsys->notify = subsys_notif_add_subsys(desc->name);
	subsys->restart_order = update_restart_order(subsys);
	ret = subsys_parse_devicetree(desc);
	if (ret)
		goto err_dtree;

#if defined(CONFIG_HTC_DEBUG_SSR)
	memset(subsys->restart_reason, 0, sizeof(subsys->restart_reason));
#endif

	snprintf(subsys->wlname, sizeof(subsys->wlname)-1, "ssr(%s)", desc->name);
	wake_lock_init(&subsys->wake_lock, WAKE_LOCK_SUSPEND, subsys->wlname);
	INIT_WORK(&subsys->work, subsystem_restart_wq_func);
	spin_lock_init(&subsys->track.s_lock);

	subsys->id = ida_simple_get(&subsys_ida, 0, 0, GFP_KERNEL);
	if (subsys->id < 0) {
		ret = subsys->id;
		goto err_ida;
	}
	dev_set_name(&subsys->dev, "subsys%d", subsys->id);

	mutex_init(&subsys->track.lock);

	ret = subsys_debugfs_add(subsys);
	if (ret)
		goto err_debugfs;

	ret = device_register(&subsys->dev);
	if (ret) {
		device_unregister(&subsys->dev);
		goto err_register;
	}

	ret = subsys_misc_device_add(subsys);
	if (ret) {
		put_device(&subsys->dev);
		goto err_register;
	}

	ret = subsys_setup_irqs(subsys);
	if (ret < 0)
		goto err_misc_device;

	return subsys;

err_misc_device:
	subsys_misc_device_remove(subsys);
err_register:
	subsys_debugfs_remove(subsys);
err_debugfs:
	mutex_destroy(&subsys->track.lock);
	ida_simple_remove(&subsys_ida, subsys->id);
err_ida:
	wake_lock_destroy(&subsys->wake_lock);
err_dtree:
	kfree(subsys);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(subsys_register);

void subsys_unregister(struct subsys_device *subsys)
{
	if (IS_ERR_OR_NULL(subsys))
		return;

	if (get_device(&subsys->dev)) {
		mutex_lock(&subsys->track.lock);
		WARN_ON(subsys->count);
		device_unregister(&subsys->dev);
		mutex_unlock(&subsys->track.lock);
		subsys_debugfs_remove(subsys);
		subsys_misc_device_remove(subsys);
		put_device(&subsys->dev);
	}
}
EXPORT_SYMBOL(subsys_unregister);

static int subsys_panic(struct device *dev, void *data)
{
	struct subsys_device *subsys = to_subsys(dev);

	if (subsys->desc->crash_shutdown)
		subsys->desc->crash_shutdown(subsys->desc);
	return 0;
}

static int ssr_panic_handler(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	bus_for_each_dev(&subsys_bus_type, NULL, NULL, subsys_panic);
	return NOTIFY_DONE;
}

static struct notifier_block panic_nb = {
	.notifier_call  = ssr_panic_handler,
};

static int __init ssr_init_soc_restart_orders(void)
{
	int i;

	atomic_notifier_chain_register(&panic_notifier_list,
			&panic_nb);

	if (cpu_is_msm8x60()) {
		for (i = 0; i < ARRAY_SIZE(orders_8x60_all); i++) {
			mutex_init(&orders_8x60_all[i]->track.lock);
			spin_lock_init(&orders_8x60_all[i]->track.s_lock);
		}

		for (i = 0; i < ARRAY_SIZE(orders_8x60_modems); i++) {
			mutex_init(&orders_8x60_modems[i]->track.lock);
			spin_lock_init(&orders_8x60_modems[i]->track.s_lock);
		}

		restart_orders = orders_8x60_all;
		n_restart_orders = ARRAY_SIZE(orders_8x60_all);
	}

	if (socinfo_get_platform_subtype() == PLATFORM_SUBTYPE_SGLTE) {
		restart_orders = restart_orders_8960_sglte;
		n_restart_orders = ARRAY_SIZE(restart_orders_8960_sglte);
	}

	for (i = 0; i < n_restart_orders; i++) {
		mutex_init(&restart_orders[i]->track.lock);
		spin_lock_init(&restart_orders[i]->track.s_lock);
	}

	return 0;
}

static int __init subsys_restart_init(void)
{
	int ret;
#if defined(CONFIG_HTC_DEBUG_SSR)
	struct kobject *properties_kobj;
	
	subsystem_restart_reason_nonblock_init();
	properties_kobj = kobject_create_and_add("subsystem_restart_properties", NULL);
	if (properties_kobj) {
		ret = sysfs_create_group(properties_kobj, &attr_group);
		if (ret) {
			pr_err("subsys_restart_init: sysfs_create_group failed\n");
			return ret;
		}
	}
	
#endif
	ssr_wq = alloc_workqueue("ssr_wq", WQ_CPU_INTENSIVE, 0);
	BUG_ON(!ssr_wq);

	ret = bus_register(&subsys_bus_type);
	if (ret)
		goto err_bus;
	ret = subsys_debugfs_init();
	if (ret)
		goto err_debugfs;
	ret = ssr_init_soc_restart_orders();
	if (ret)
		goto err_soc;
	return 0;

err_soc:
	subsys_debugfs_exit();
err_debugfs:
	bus_unregister(&subsys_bus_type);
err_bus:
	destroy_workqueue(ssr_wq);
	return ret;
}
arch_initcall(subsys_restart_init);

MODULE_DESCRIPTION("Subsystem Restart Driver");
MODULE_LICENSE("GPL v2");
