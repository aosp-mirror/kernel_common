/* include/linux/android_alarm.h
 *
 * Copyright (C) 2006-2007 Google, Inc.
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

#ifndef _LINUX_ANDROID_ALARM_H
#define _LINUX_ANDROID_ALARM_H

#include <linux/ioctl.h>
#include <linux/time.h>

enum android_alarm_type {
	/* return code bit numbers or set alarm arg */
	ANDROID_ALARM_RTC_WAKEUP,
	ANDROID_ALARM_RTC,
	ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
	ANDROID_ALARM_ELAPSED_REALTIME,
	ANDROID_ALARM_RTC_POWEROFF_WAKEUP,
	ANDROID_ALARM_SYSTEMTIME,

	ANDROID_ALARM_TYPE_COUNT,

	/* return code bit numbers */
	/* ANDROID_ALARM_TIME_CHANGE = 16 */
};

#ifdef __KERNEL__

#include <linux/ktime.h>
#include <linux/rbtree.h>

/*
 * The alarm interface is similar to the hrtimer interface but adds support
 * for wakeup from suspend. It also adds an elapsed realtime clock that can
 * be used for periodic timers that need to keep runing while the system is
 * suspended and not be disrupted when the wall time is set.
 */

/**
 * struct alarm - the basic alarm structure
 * @node:	red black tree node for time ordered insertion
 * @type:	alarm type. rtc/elapsed-realtime/systemtime, wakeup/non-wakeup.
 * @softexpires: the absolute earliest expiry time of the alarm.
 * @expires:	the absolute expiry time.
 * @function:	alarm expiry callback function
 *
 * The alarm structure must be initialized by alarm_init()
 *
 */

struct alarm {
	struct rb_node 		node;
	enum android_alarm_type type;
	ktime_t			softexpires;
	ktime_t			expires;
	void			(*function)(struct alarm *);
};

void alarm_init(struct alarm *alarm,
	enum android_alarm_type type, void (*function)(struct alarm *));
void alarm_start_range(struct alarm *alarm, ktime_t start, ktime_t end);
int alarm_try_to_cancel(struct alarm *alarm);
int alarm_cancel(struct alarm *alarm);
void set_power_on_alarm(long secs, bool enable);
ktime_t alarm_get_elapsed_realtime(void);

/* set rtc while preserving elapsed realtime */
int alarm_set_rtc(const struct timespec ts);
void alarm_update_timedelta(struct timespec tv, struct timespec ts);

#endif

enum android_alarm_return_flags {
	ANDROID_ALARM_RTC_WAKEUP_MASK = 1U << ANDROID_ALARM_RTC_WAKEUP,
	ANDROID_ALARM_RTC_MASK = 1U << ANDROID_ALARM_RTC,
	ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP_MASK =
				1U << ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
	ANDROID_ALARM_ELAPSED_REALTIME_MASK =
				1U << ANDROID_ALARM_ELAPSED_REALTIME,
	ANDROID_ALARM_RTC_POWEROFF_WAKEUP_MASK = 1U << ANDROID_ALARM_RTC_POWEROFF_WAKEUP,
	ANDROID_ALARM_SYSTEMTIME_MASK = 1U << ANDROID_ALARM_SYSTEMTIME,
	ANDROID_ALARM_TIME_CHANGE_MASK = 1U << 16
};

/* Disable alarm */
#define ANDROID_ALARM_CLEAR(type)           _IO('a', 0 | ((type) << 4))

/* Ack last alarm and wait for next */
#define ANDROID_ALARM_WAIT                  _IO('a', 1)

#define ALARM_IOW(c, type, size)            _IOW('a', (c) | ((type) << 4), size)
/* Set alarm */
#define ANDROID_ALARM_SET(type)             ALARM_IOW(2, type, struct timespec)
#define ANDROID_ALARM_SET_AND_WAIT(type)    ALARM_IOW(3, type, struct timespec)
#define ANDROID_ALARM_GET_TIME(type)        ALARM_IOW(4, type, struct timespec)
#define ANDROID_ALARM_SET_RTC               _IOW('a', 5, struct timespec)
#define ANDROID_ALARM_BASE_CMD(cmd)         (cmd & ~(_IOC(0, 0, 0xf0, 0)))
#define ANDROID_ALARM_IOCTL_TO_TYPE(cmd)    (_IOC_NR(cmd) >> 4)

#endif
