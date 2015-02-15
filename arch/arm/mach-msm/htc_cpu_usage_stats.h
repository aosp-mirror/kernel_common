#ifndef __HTC_CPU_USAGE_STATS_H__
#define __HTC_CPU_USAGE_STATS_H__

#define SUBSYSTEM_CPU_USAGE_STATS   "SUBSYSTEM=cpu_usage_stats"

extern void send_cpu_usage_stats_kobject_uevent(char *buf_pid);

#endif /* __HTC_CPU_USAGE_STATS_H__ */

