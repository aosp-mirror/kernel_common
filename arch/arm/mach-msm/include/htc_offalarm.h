#ifndef _ASM_ARCH_HTC_OFFALARM_H_
#define _ASM_ARCH_HTC_OFFALARM_H_

#define PM_PWR_ON_EVENT_RTC 0x2

struct htc_off_alarm {
	int alarm_id;
	int alarm_time;
};
extern int rtc_alarm_trigger;
extern struct htc_off_alarm htc_offalarm;

#endif
