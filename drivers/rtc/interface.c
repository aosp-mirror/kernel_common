/*
 * RTC subsystem, interface functions
 *
 * Copyright (C) 2005 Tower Technologies
 * Author: Alessandro Zummo <a.zummo@towertech.it>
 *
 * based on arch/arm/common/rtctime.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/rtc.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/log2.h>
#include <linux/workqueue.h>

static int rtc_timer_enqueue(struct rtc_device *rtc, struct rtc_timer *timer);
static void rtc_timer_remove(struct rtc_device *rtc, struct rtc_timer *timer);

static int __rtc_read_time(struct rtc_device *rtc, struct rtc_time *tm)
{
	int err;
	if (!rtc->ops)
		err = -ENODEV;
	else if (!rtc->ops->read_time)
		err = -EINVAL;
	else {
		memset(tm, 0, sizeof(struct rtc_time));
		err = rtc->ops->read_time(rtc->dev.parent, tm);
	}
	return err;
}

int rtc_read_time(struct rtc_device *rtc, struct rtc_time *tm)
{
	int err;

	err = mutex_lock_interruptible(&rtc->ops_lock);
	if (err)
		return err;

	err = __rtc_read_time(rtc, tm);
	mutex_unlock(&rtc->ops_lock);
	return err;
}
EXPORT_SYMBOL_GPL(rtc_read_time);

int rtc_set_time(struct rtc_device *rtc, struct rtc_time *tm)
{
	int err;
	unsigned long secs;

	err = rtc_valid_tm(tm);
	if (err != 0)
		return err;

	err = mutex_lock_interruptible(&rtc->ops_lock);
	if (err)
		return err;

	if (!rtc->ops)
		err = -ENODEV;
	else if (rtc->ops->set_time)
		err = rtc->ops->set_time(rtc->dev.parent, tm);
	else if (rtc->ops->set_mmss) {
		unsigned long secs;
		err = rtc_tm_to_time(tm, &secs);
		if (err == 0)
			err = rtc->ops->set_mmss(rtc->dev.parent, secs);
	} else
		err = -EINVAL;

	mutex_unlock(&rtc->ops_lock);
	rtc_tm_to_time(tm, &secs);
	pr_info("rtc: %s (%d:%d) set RTC time %lu secs to RTC device %s (err=%d)\n",
			current->comm, current->tgid, current->pid, secs, rtc->name, err);
	dump_stack();
	
	schedule_work(&rtc->irqwork);
	return err;
}
EXPORT_SYMBOL_GPL(rtc_set_time);

int rtc_set_mmss(struct rtc_device *rtc, unsigned long secs)
{
	int err;

	err = mutex_lock_interruptible(&rtc->ops_lock);
	if (err)
		return err;

	if (!rtc->ops)
		err = -ENODEV;
	else if (rtc->ops->set_mmss)
		err = rtc->ops->set_mmss(rtc->dev.parent, secs);
	else if (rtc->ops->read_time && rtc->ops->set_time) {
		struct rtc_time new, old;

		err = rtc->ops->read_time(rtc->dev.parent, &old);
		if (err == 0) {
			rtc_time_to_tm(secs, &new);

			if (!((old.tm_hour == 23 && old.tm_min == 59) ||
				(new.tm_hour == 23 && new.tm_min == 59)))
				err = rtc->ops->set_time(rtc->dev.parent,
						&new);
		}
	}
	else
		err = -EINVAL;

	mutex_unlock(&rtc->ops_lock);
	
	schedule_work(&rtc->irqwork);

	return err;
}
EXPORT_SYMBOL_GPL(rtc_set_mmss);

static int rtc_read_alarm_internal(struct rtc_device *rtc, struct rtc_wkalrm *alarm)
{
	int err;

	err = mutex_lock_interruptible(&rtc->ops_lock);
	if (err)
		return err;

	if (rtc->ops == NULL)
		err = -ENODEV;
	else if (!rtc->ops->read_alarm)
		err = -EINVAL;
	else {
		memset(alarm, 0, sizeof(struct rtc_wkalrm));
		err = rtc->ops->read_alarm(rtc->dev.parent, alarm);
	}

	mutex_unlock(&rtc->ops_lock);
	return err;
}

int __rtc_read_alarm(struct rtc_device *rtc, struct rtc_wkalrm *alarm)
{
	int err;
	struct rtc_time before, now;
	int first_time = 1;
	unsigned long t_now, t_alm;
	enum { none, day, month, year } missing = none;
	unsigned days;


	
	err = rtc_read_time(rtc, &before);
	if (err < 0)
		return err;
	do {
		if (!first_time)
			memcpy(&before, &now, sizeof(struct rtc_time));
		first_time = 0;

		
		err = rtc_read_alarm_internal(rtc, alarm);
		if (err)
			return err;

		
		if (rtc_valid_tm(&alarm->time) == 0)
			return 0;

		
		err = rtc_read_time(rtc, &now);
		if (err < 0)
			return err;

		
	} while (   before.tm_min   != now.tm_min
		 || before.tm_hour  != now.tm_hour
		 || before.tm_mon   != now.tm_mon
		 || before.tm_year  != now.tm_year);

	if (alarm->time.tm_sec == -1)
		alarm->time.tm_sec = now.tm_sec;
	if (alarm->time.tm_min == -1)
		alarm->time.tm_min = now.tm_min;
	if (alarm->time.tm_hour == -1)
		alarm->time.tm_hour = now.tm_hour;

	
	if (alarm->time.tm_mday < 1 || alarm->time.tm_mday > 31) {
		alarm->time.tm_mday = now.tm_mday;
		missing = day;
	}
	if ((unsigned)alarm->time.tm_mon >= 12) {
		alarm->time.tm_mon = now.tm_mon;
		if (missing == none)
			missing = month;
	}
	if (alarm->time.tm_year == -1) {
		alarm->time.tm_year = now.tm_year;
		if (missing == none)
			missing = year;
	}

	
	rtc_tm_to_time(&now, &t_now);
	rtc_tm_to_time(&alarm->time, &t_alm);
	if (t_now < t_alm)
		goto done;

	switch (missing) {

	case day:
		dev_dbg(&rtc->dev, "alarm rollover: %s\n", "day");
		t_alm += 24 * 60 * 60;
		rtc_time_to_tm(t_alm, &alarm->time);
		break;

	case month:
		dev_dbg(&rtc->dev, "alarm rollover: %s\n", "month");
		do {
			if (alarm->time.tm_mon < 11)
				alarm->time.tm_mon++;
			else {
				alarm->time.tm_mon = 0;
				alarm->time.tm_year++;
			}
			days = rtc_month_days(alarm->time.tm_mon,
					alarm->time.tm_year);
		} while (days < alarm->time.tm_mday);
		break;

	
	case year:
		dev_dbg(&rtc->dev, "alarm rollover: %s\n", "year");
		do {
			alarm->time.tm_year++;
		} while (rtc_valid_tm(&alarm->time) != 0);
		break;

	default:
		dev_warn(&rtc->dev, "alarm rollover not handled\n");
	}

done:
	return 0;
}

int rtc_read_alarm(struct rtc_device *rtc, struct rtc_wkalrm *alarm)
{
	int err;

	err = mutex_lock_interruptible(&rtc->ops_lock);
	if (err)
		return err;
	if (rtc->ops == NULL)
		err = -ENODEV;
	else if (!rtc->ops->read_alarm)
		err = -EINVAL;
	else {
		memset(alarm, 0, sizeof(struct rtc_wkalrm));
		alarm->enabled = rtc->aie_timer.enabled;
		alarm->time = rtc_ktime_to_tm(rtc->aie_timer.node.expires);
	}
	mutex_unlock(&rtc->ops_lock);

	return err;
}
EXPORT_SYMBOL_GPL(rtc_read_alarm);

#ifdef CONFIG_HTC_POWERSAVE_MODE_ALARM
extern int powersave_enabled;
#endif

static int __rtc_set_alarm(struct rtc_device *rtc, struct rtc_wkalrm *alarm)
{
	struct rtc_time tm;
	long now, scheduled;
	int err;

	err = rtc_valid_tm(&alarm->time);
	if (err)
		return err;
	rtc_tm_to_time(&alarm->time, &scheduled);

	
	err = __rtc_read_time(rtc, &tm);
	rtc_tm_to_time(&tm, &now);
	if (scheduled <= now)
		return -ETIME;

	if (!rtc->ops)
		err = -ENODEV;
	else if (!rtc->ops->set_alarm)
		err = -EINVAL;
	else {
#ifdef CONFIG_HTC_POWERSAVE_MODE_ALARM
		if ((powersave_enabled == 2) && (alarm->time.tm_sec > 0)) {
			alarm->time.tm_sec = 0;
			alarm->time.tm_min++;
		}
#endif
		err = rtc->ops->set_alarm(rtc->dev.parent, alarm);
	}

	return err;
}

int rtc_set_alarm(struct rtc_device *rtc, struct rtc_wkalrm *alarm)
{
	int err;

	err = rtc_valid_tm(&alarm->time);
	if (err != 0)
		return err;

	err = mutex_lock_interruptible(&rtc->ops_lock);
	if (err)
		return err;
	if (rtc->aie_timer.enabled) {
		rtc_timer_remove(rtc, &rtc->aie_timer);
	}
	rtc->aie_timer.node.expires = rtc_tm_to_ktime(alarm->time);
	rtc->aie_timer.period = ktime_set(0, 0);
	if (alarm->enabled) {
		err = rtc_timer_enqueue(rtc, &rtc->aie_timer);
	}
	mutex_unlock(&rtc->ops_lock);
	return err;
}
EXPORT_SYMBOL_GPL(rtc_set_alarm);

int rtc_initialize_alarm(struct rtc_device *rtc, struct rtc_wkalrm *alarm)
{
	int err;
	struct rtc_time now;

	err = rtc_valid_tm(&alarm->time);
	if (err != 0)
		return err;

	err = rtc_read_time(rtc, &now);
	if (err)
		return err;

	err = mutex_lock_interruptible(&rtc->ops_lock);
	if (err)
		return err;

	rtc->aie_timer.node.expires = rtc_tm_to_ktime(alarm->time);
	rtc->aie_timer.period = ktime_set(0, 0);

	
	if (alarm->enabled && (rtc_tm_to_ktime(now).tv64 <
			 rtc->aie_timer.node.expires.tv64)) {

		rtc->aie_timer.enabled = 1;
		timerqueue_add(&rtc->timerqueue, &rtc->aie_timer.node);
	}
	mutex_unlock(&rtc->ops_lock);
	return err;
}
EXPORT_SYMBOL_GPL(rtc_initialize_alarm);



int rtc_alarm_irq_enable(struct rtc_device *rtc, unsigned int enabled)
{
	int err = mutex_lock_interruptible(&rtc->ops_lock);
	if (err)
		return err;

	if (rtc->aie_timer.enabled != enabled) {
		if (enabled)
			err = rtc_timer_enqueue(rtc, &rtc->aie_timer);
		else
			rtc_timer_remove(rtc, &rtc->aie_timer);
	}

	if (err)
		;
	else if (!rtc->ops)
		err = -ENODEV;
	else if (!rtc->ops->alarm_irq_enable)
		err = -EINVAL;
	else
		err = rtc->ops->alarm_irq_enable(rtc->dev.parent, enabled);

	mutex_unlock(&rtc->ops_lock);
	return err;
}
EXPORT_SYMBOL_GPL(rtc_alarm_irq_enable);

int rtc_update_irq_enable(struct rtc_device *rtc, unsigned int enabled)
{
	int err = mutex_lock_interruptible(&rtc->ops_lock);
	if (err)
		return err;

#ifdef CONFIG_RTC_INTF_DEV_UIE_EMUL
	if (enabled == 0 && rtc->uie_irq_active) {
		mutex_unlock(&rtc->ops_lock);
		return rtc_dev_update_irq_enable_emul(rtc, 0);
	}
#endif
	
	if (rtc->uie_rtctimer.enabled == enabled)
		goto out;

	if (rtc->uie_unsupported) {
		err = -EINVAL;
		goto out;
	}

	if (enabled) {
		struct rtc_time tm;
		ktime_t now, onesec;

		__rtc_read_time(rtc, &tm);
		onesec = ktime_set(1, 0);
		now = rtc_tm_to_ktime(tm);
		rtc->uie_rtctimer.node.expires = ktime_add(now, onesec);
		rtc->uie_rtctimer.period = ktime_set(1, 0);
		err = rtc_timer_enqueue(rtc, &rtc->uie_rtctimer);
	} else
		rtc_timer_remove(rtc, &rtc->uie_rtctimer);

out:
	mutex_unlock(&rtc->ops_lock);
#ifdef CONFIG_RTC_INTF_DEV_UIE_EMUL
	if (err == -EINVAL)
		err = rtc_dev_update_irq_enable_emul(rtc, enabled);
#endif
	return err;

}
EXPORT_SYMBOL_GPL(rtc_update_irq_enable);


void rtc_handle_legacy_irq(struct rtc_device *rtc, int num, int mode)
{
	unsigned long flags;

	
	spin_lock_irqsave(&rtc->irq_lock, flags);
	rtc->irq_data = (rtc->irq_data + (num << 8)) | (RTC_IRQF|mode);
	spin_unlock_irqrestore(&rtc->irq_lock, flags);

	
	spin_lock_irqsave(&rtc->irq_task_lock, flags);
	if (rtc->irq_task)
		rtc->irq_task->func(rtc->irq_task->private_data);
	spin_unlock_irqrestore(&rtc->irq_task_lock, flags);

	wake_up_interruptible(&rtc->irq_queue);
	kill_fasync(&rtc->async_queue, SIGIO, POLL_IN);
}


void rtc_aie_update_irq(void *private)
{
	struct rtc_device *rtc = (struct rtc_device *)private;
	rtc_handle_legacy_irq(rtc, 1, RTC_AF);
}


void rtc_uie_update_irq(void *private)
{
	struct rtc_device *rtc = (struct rtc_device *)private;
	rtc_handle_legacy_irq(rtc, 1,  RTC_UF);
}


enum hrtimer_restart rtc_pie_update_irq(struct hrtimer *timer)
{
	struct rtc_device *rtc;
	ktime_t period;
	int count;
	rtc = container_of(timer, struct rtc_device, pie_timer);

	period = ktime_set(0, NSEC_PER_SEC/rtc->irq_freq);
	count = hrtimer_forward_now(timer, period);

	rtc_handle_legacy_irq(rtc, count, RTC_PF);

	return HRTIMER_RESTART;
}

void rtc_update_irq(struct rtc_device *rtc,
		unsigned long num, unsigned long events)
{
	schedule_work(&rtc->irqwork);
}
EXPORT_SYMBOL_GPL(rtc_update_irq);

static int __rtc_match(struct device *dev, void *data)
{
	char *name = (char *)data;

	if (strcmp(dev_name(dev), name) == 0)
		return 1;
	return 0;
}

struct rtc_device *rtc_class_open(char *name)
{
	struct device *dev;
	struct rtc_device *rtc = NULL;

	dev = class_find_device(rtc_class, NULL, name, __rtc_match);
	if (dev)
		rtc = to_rtc_device(dev);

	if (rtc) {
		if (!try_module_get(rtc->owner)) {
			put_device(dev);
			rtc = NULL;
		}
	}

	return rtc;
}
EXPORT_SYMBOL_GPL(rtc_class_open);

void rtc_class_close(struct rtc_device *rtc)
{
	module_put(rtc->owner);
	put_device(&rtc->dev);
}
EXPORT_SYMBOL_GPL(rtc_class_close);

int rtc_irq_register(struct rtc_device *rtc, struct rtc_task *task)
{
	int retval = -EBUSY;

	if (task == NULL || task->func == NULL)
		return -EINVAL;

	
	if (test_and_set_bit_lock(RTC_DEV_BUSY, &rtc->flags))
		return -EBUSY;

	spin_lock_irq(&rtc->irq_task_lock);
	if (rtc->irq_task == NULL) {
		rtc->irq_task = task;
		retval = 0;
	}
	spin_unlock_irq(&rtc->irq_task_lock);

	clear_bit_unlock(RTC_DEV_BUSY, &rtc->flags);

	return retval;
}
EXPORT_SYMBOL_GPL(rtc_irq_register);

void rtc_irq_unregister(struct rtc_device *rtc, struct rtc_task *task)
{
	spin_lock_irq(&rtc->irq_task_lock);
	if (rtc->irq_task == task)
		rtc->irq_task = NULL;
	spin_unlock_irq(&rtc->irq_task_lock);
}
EXPORT_SYMBOL_GPL(rtc_irq_unregister);

static int rtc_update_hrtimer(struct rtc_device *rtc, int enabled)
{
	if (hrtimer_try_to_cancel(&rtc->pie_timer) < 0)
		return -1;

	if (enabled) {
		ktime_t period = ktime_set(0, NSEC_PER_SEC / rtc->irq_freq);

		hrtimer_start(&rtc->pie_timer, period, HRTIMER_MODE_REL);
	}
	return 0;
}

int rtc_irq_set_state(struct rtc_device *rtc, struct rtc_task *task, int enabled)
{
	int err = 0;
	unsigned long flags;

retry:
	spin_lock_irqsave(&rtc->irq_task_lock, flags);
	if (rtc->irq_task != NULL && task == NULL)
		err = -EBUSY;
	if (rtc->irq_task != task)
		err = -EACCES;
	if (!err) {
		if (rtc_update_hrtimer(rtc, enabled) < 0) {
			spin_unlock_irqrestore(&rtc->irq_task_lock, flags);
			cpu_relax();
			goto retry;
		}
		rtc->pie_enabled = enabled;
	}
	spin_unlock_irqrestore(&rtc->irq_task_lock, flags);
	return err;
}
EXPORT_SYMBOL_GPL(rtc_irq_set_state);

int rtc_irq_set_freq(struct rtc_device *rtc, struct rtc_task *task, int freq)
{
	int err = 0;
	unsigned long flags;

	if (freq <= 0 || freq > RTC_MAX_FREQ)
		return -EINVAL;
retry:
	spin_lock_irqsave(&rtc->irq_task_lock, flags);
	if (rtc->irq_task != NULL && task == NULL)
		err = -EBUSY;
	if (rtc->irq_task != task)
		err = -EACCES;
	if (!err) {
		rtc->irq_freq = freq;
		if (rtc->pie_enabled && rtc_update_hrtimer(rtc, 1) < 0) {
			spin_unlock_irqrestore(&rtc->irq_task_lock, flags);
			cpu_relax();
			goto retry;
		}
	}
	spin_unlock_irqrestore(&rtc->irq_task_lock, flags);
	return err;
}
EXPORT_SYMBOL_GPL(rtc_irq_set_freq);

static int rtc_timer_enqueue(struct rtc_device *rtc, struct rtc_timer *timer)
{
	timer->enabled = 1;
	timerqueue_add(&rtc->timerqueue, &timer->node);
	if (&timer->node == timerqueue_getnext(&rtc->timerqueue)) {
		struct rtc_wkalrm alarm;
		int err;
		alarm.time = rtc_ktime_to_tm(timer->node.expires);
		alarm.enabled = 1;
		err = __rtc_set_alarm(rtc, &alarm);
		if (err == -ETIME)
			schedule_work(&rtc->irqwork);
		else if (err) {
			timerqueue_del(&rtc->timerqueue, &timer->node);
			timer->enabled = 0;
			return err;
		}
	}
	return 0;
}

static void rtc_alarm_disable(struct rtc_device *rtc)
{
	if (!rtc->ops || !rtc->ops->alarm_irq_enable)
		return;

	rtc->ops->alarm_irq_enable(rtc->dev.parent, false);
}

static void rtc_timer_remove(struct rtc_device *rtc, struct rtc_timer *timer)
{
	struct timerqueue_node *next = timerqueue_getnext(&rtc->timerqueue);
	timerqueue_del(&rtc->timerqueue, &timer->node);
	timer->enabled = 0;
	if (next == &timer->node) {
		struct rtc_wkalrm alarm;
		int err;
		next = timerqueue_getnext(&rtc->timerqueue);
		if (!next) {
			rtc_alarm_disable(rtc);
			return;
		}
		alarm.time = rtc_ktime_to_tm(next->expires);
		alarm.enabled = 1;
		err = __rtc_set_alarm(rtc, &alarm);
		if (err == -ETIME)
			schedule_work(&rtc->irqwork);
	}
}

void rtc_timer_do_work(struct work_struct *work)
{
	struct rtc_timer *timer;
	struct timerqueue_node *next;
	ktime_t now;
	struct rtc_time tm;

	struct rtc_device *rtc =
		container_of(work, struct rtc_device, irqwork);

	mutex_lock(&rtc->ops_lock);
again:
	__rtc_read_time(rtc, &tm);
	now = rtc_tm_to_ktime(tm);
	while ((next = timerqueue_getnext(&rtc->timerqueue))) {
		if (next->expires.tv64 > now.tv64)
			break;

		
		timer = container_of(next, struct rtc_timer, node);
		timerqueue_del(&rtc->timerqueue, &timer->node);
		timer->enabled = 0;
		if (timer->task.func)
			timer->task.func(timer->task.private_data);

		
		if (ktime_to_ns(timer->period)) {
			timer->node.expires = ktime_add(timer->node.expires,
							timer->period);
			timer->enabled = 1;
			timerqueue_add(&rtc->timerqueue, &timer->node);
		}
	}

	
	if (next) {
		struct rtc_wkalrm alarm;
		int err;
		alarm.time = rtc_ktime_to_tm(next->expires);
		alarm.enabled = 1;
		err = __rtc_set_alarm(rtc, &alarm);
		if (err == -ETIME)
			goto again;
	} else
		rtc_alarm_disable(rtc);

	mutex_unlock(&rtc->ops_lock);
}


void rtc_timer_init(struct rtc_timer *timer, void (*f)(void* p), void* data)
{
	timerqueue_init(&timer->node);
	timer->enabled = 0;
	timer->task.func = f;
	timer->task.private_data = data;
}

int rtc_timer_start(struct rtc_device *rtc, struct rtc_timer* timer,
			ktime_t expires, ktime_t period)
{
	int ret = 0;
	mutex_lock(&rtc->ops_lock);
	if (timer->enabled)
		rtc_timer_remove(rtc, timer);

	timer->node.expires = expires;
	timer->period = period;

	ret = rtc_timer_enqueue(rtc, timer);

	mutex_unlock(&rtc->ops_lock);
	return ret;
}

int rtc_timer_cancel(struct rtc_device *rtc, struct rtc_timer* timer)
{
	int ret = 0;
	mutex_lock(&rtc->ops_lock);
	if (timer->enabled)
		rtc_timer_remove(rtc, timer);
	mutex_unlock(&rtc->ops_lock);
	return ret;
}


