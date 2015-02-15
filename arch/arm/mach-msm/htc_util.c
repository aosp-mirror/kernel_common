#include <linux/io.h>
#include <linux/rtc.h>
#include <linux/time.h>
#include <linux/tick.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/pm_wakeup.h>
#include <linux/kernel_stat.h>
#include <mach/devices_dtb.h>
#include <mach/htc_util.h>
#include <linux/irq.h>
#include "pm.h"
#include "rpm_stats.h"
#include "htc_cpu_usage_stats.h"

#define USE_STATISTICS_STRATEGY_CONTINUOUS_3    0
#define SEND_KOBJECT_UEVENT_ENV_ENABLED         0

#define NUM_BUSY_THREAD_CHECK                   5
#define HTC_KERNEL_TOP_CPU_USAGE_THRESHOLD      30
#define BUFFER_WARN_LEN                         64
#define BUFFER_TEMP_LEN                         32

#define FORCE_CHARGE			(1<<2)
#define Y_CABLE			(1<<26)

struct process_monitor_statistic {
       unsigned int pid;
       char *ppid_name;
       unsigned int cnt;
       unsigned char set_warn;
       unsigned char is_found;
#if SEND_KOBJECT_UEVENT_ENV_ENABLED
       unsigned char sent_uevent;
#endif 
};

static int pm_monitor_enabled = 0;

static struct workqueue_struct *htc_pm_monitor_wq = NULL;
static struct workqueue_struct *htc_kernel_top_monitor_wq = NULL;

#define MAX_PID 32768
#define NUM_BUSY_THREAD_CHECK 5

struct _htc_kernel_top {
	struct delayed_work dwork;
	unsigned int *prev_proc_stat;
	int *curr_proc_delta;
	int *curr_proc_pid;
	struct task_struct **task_ptr_array;
	struct kernel_cpustat curr_cpustat;
	struct kernel_cpustat prev_cpustat;
	unsigned long cpustat_time;
	int top_loading_pid[NUM_BUSY_THREAD_CHECK];
	spinlock_t lock;
};

struct st_htc_idle_statistic {
	u32 count;
	u32 time;
};

struct st_htc_idle_statistic htc_idle_stat[CONFIG_NR_CPUS][3];

static int msm_htc_util_delay_time = 10000;
module_param_named(kmonitor_delay, msm_htc_util_delay_time, int, S_IRUGO | S_IWUSR | S_IWGRP);
static int msm_htc_util_top_delay_time = 60000;
module_param_named(ktop_delay, msm_htc_util_top_delay_time, int, S_IRUGO | S_IWUSR | S_IWGRP);
extern unsigned int get_tamper_sf(void);
extern unsigned int get_radio_flag_ex1(void);
extern unsigned int get_radio_flag_ex2(void);

enum {
	KERNEL_TOP,
	KERNEL_TOP_ACCU, 
};

#if USE_STATISTICS_STRATEGY_CONTINUOUS_3
#define MAX_CONSECUTIVE_THRES_TIMES             3
#define SIZE_OF_CURR_PID_FOUND_ARRAY                    ARRAY_SIZE(current_pid_found_array)
#define SIZE_OF_PROCESS_MONITOR_CONTINUOUS_3_ARRAY      ARRAY_SIZE(process_monitor_continuous_3_array)
struct current_pid_found {
       unsigned char pid_found;
       unsigned char need_to_add;
};

static struct current_pid_found current_pid_found_array[NUM_BUSY_THREAD_CHECK];
static struct process_monitor_statistic process_monitor_continuous_3_array[NUM_BUSY_THREAD_CHECK];

#else 
#define MAX_OVER_THRES_TIMES                    5
#define HTC_KERNEL_TOP_MONITOR_PERIOD           10
#define MAX_PROCESS_MONITOR_ARRAY_FIELDS        (HTC_KERNEL_TOP_MONITOR_PERIOD * NUM_BUSY_THREAD_CHECK)
#define PROCESS_MONITOR_ARRAY_5_IN_10_SIZE      MAX_PROCESS_MONITOR_ARRAY_FIELDS
#define BUFFER_WARN_5_IN_10_SIZE                HTC_KERNEL_TOP_MONITOR_PERIOD
#define SIZE_OF_PROCESS_MONITOR_5_IN_10_ARRAY           ARRAY_SIZE(process_monitor_5_in_10_array)

static int statistic_monitor_period = 1;
static struct process_monitor_statistic process_monitor_5_in_10_array[PROCESS_MONITOR_ARRAY_5_IN_10_SIZE];
#endif 

static void clear_process_monitor_array(struct process_monitor_statistic *pArray, int array_size)
{
	int j;

	for (j = 0; j < array_size; j++) {
		(pArray + j)->pid = 0;
		(pArray + j)->ppid_name = NULL;
		(pArray + j)->cnt = 0;
		(pArray + j)->set_warn = 0;
		(pArray + j)->is_found = 0;
#if SEND_KOBJECT_UEVENT_ENV_ENABLED
		(pArray + j)->sent_uevent = 0;
#endif 
	}
} 

#if USE_STATISTICS_STRATEGY_CONTINUOUS_3
static void clear_current_pid_found_array(void)
{
	int i;

	for (i = 0; i < SIZE_OF_CURR_PID_FOUND_ARRAY; i++) {
		current_pid_found_array[i].pid_found = 0;
		current_pid_found_array[i].need_to_add = 0;
	}
}

static int htc_kernel_top_statistics_continuous_3(struct _htc_kernel_top *ktop)
{
	int rtn = 0;
	int i, j, cpu_usage;
#if SEND_KOBJECT_UEVENT_ENV_ENABLED
	int ok_to_send_uevent = 0;
	char buf_warn[SIZE_OF_PROCESS_MONITOR_CONTINUOUS_3_ARRAY * BUFFER_WARN_LEN];
	char buf_temp[BUFFER_TEMP_LEN];
#endif 
	unsigned long delta_time = ktop->cpustat_time;
	int *ptr_top_loading = ktop->top_loading_pid;

	for (i = 0 ; i < SIZE_OF_CURR_PID_FOUND_ARRAY ; i++) {
		cpu_usage = ktop->curr_proc_delta[*(ptr_top_loading + i)] * 100 / delta_time;
		
		if (cpu_usage >= HTC_KERNEL_TOP_CPU_USAGE_THRESHOLD) {
			
			for (j = 0; j < SIZE_OF_PROCESS_MONITOR_CONTINUOUS_3_ARRAY; j++) {
				
				if (process_monitor_continuous_3_array[j].pid == *(ptr_top_loading + i)) {
					
					process_monitor_continuous_3_array[j].cnt++;
					process_monitor_continuous_3_array[j].is_found = 1;
					
					current_pid_found_array[i].pid_found = 1;
					if ((process_monitor_continuous_3_array[j].cnt >= MAX_CONSECUTIVE_THRES_TIMES) &&
							(!process_monitor_continuous_3_array[j].set_warn)) {
						process_monitor_continuous_3_array[j].set_warn = 1;
						pr_err("[K] CPU_Sniffer: PID=[%d], name=[%s], over-cpu-usage-threshold.\n",
						process_monitor_continuous_3_array[j].pid, process_monitor_continuous_3_array[j].ppid_name);
					}
					break;
				}
			}
			if (!current_pid_found_array[i].pid_found) {
				current_pid_found_array[i].need_to_add = 1;
			}
		}
	}

#if SEND_KOBJECT_UEVENT_ENV_ENABLED
	
	memset(buf_warn, 0x0, sizeof(buf_warn));
	strcpy(buf_warn, "");
	for (j = 0; j < SIZE_OF_PROCESS_MONITOR_CONTINUOUS_3_ARRAY; j++) {
		if ((process_monitor_continuous_3_array[j].set_warn == 1) &&
			(process_monitor_continuous_3_array[j].sent_uevent == 0)) {
			strcat(buf_warn, "PID=");
			sprintf(buf_temp, "%d", process_monitor_continuous_3_array[j].pid);
			strcat(buf_warn, buf_temp);
			strcat(buf_warn, ",0,0,0;");
			process_monitor_continuous_3_array[j].sent_uevent = 1;
			ok_to_send_uevent++;
		}
	}

	
	if (ok_to_send_uevent) {
		
		strcat(buf_warn, "PID=0,0,0,0;");
		strcat(buf_warn, "#");
		send_cpu_usage_stats_kobject_uevent(&buf_warn[0]);
	}
#endif 

	
	for (j = 0; j < SIZE_OF_PROCESS_MONITOR_CONTINUOUS_3_ARRAY; j++) {
		if (!process_monitor_continuous_3_array[j].is_found) {
			
			process_monitor_continuous_3_array[j].pid = 0;
			process_monitor_continuous_3_array[j].ppid_name = NULL;
			process_monitor_continuous_3_array[j].cnt = 0;
			process_monitor_continuous_3_array[j].set_warn = 0;
#if SEND_KOBJECT_UEVENT_ENV_ENABLED
			process_monitor_continuous_3_array[j].sent_uevent = 0;
#endif 
		}
		
		process_monitor_continuous_3_array[j].is_found = 0;
	}

	
	for (i = 0 ; i < SIZE_OF_CURR_PID_FOUND_ARRAY ; i++) {
		
		for (j = 0; j < SIZE_OF_PROCESS_MONITOR_CONTINUOUS_3_ARRAY; j++) {
			if (current_pid_found_array[i].need_to_add && !process_monitor_continuous_3_array[j].pid) {
				process_monitor_continuous_3_array[j].pid = *(ptr_top_loading + i);
				process_monitor_continuous_3_array[j].ppid_name = ktop->task_ptr_array[*(ptr_top_loading + i)]->comm;
				process_monitor_continuous_3_array[j].cnt++;
				current_pid_found_array[i].need_to_add = 0;
				break;
			}
		}
	}
	clear_current_pid_found_array();

	return rtn;
}
#else 
static int htc_kernel_top_statistics_5_in_10(struct _htc_kernel_top *ktop)
{
	int rtn = 0;
	int i, j, cpu_usage;
#if SEND_KOBJECT_UEVENT_ENV_ENABLED
	int ok_to_send_uevent = 0;
	char buf_warn[BUFFER_WARN_5_IN_10_SIZE * BUFFER_WARN_LEN];
	char buf_temp[BUFFER_TEMP_LEN];
#endif 
	unsigned long delta_time = ktop->cpustat_time;
	int *ptr_top_loading = ktop->top_loading_pid;

	for (i = 0 ; i < NUM_BUSY_THREAD_CHECK ; i++) {
		cpu_usage = ktop->curr_proc_delta[*(ptr_top_loading + i)] * 100 / delta_time;
		
		if (cpu_usage >= HTC_KERNEL_TOP_CPU_USAGE_THRESHOLD) {
			
			for (j = 0; j < SIZE_OF_PROCESS_MONITOR_5_IN_10_ARRAY; j++) {
				
				if (process_monitor_5_in_10_array[j].pid == *(ptr_top_loading + i)) {
					
					process_monitor_5_in_10_array[j].cnt++;
					if ((process_monitor_5_in_10_array[j].cnt >= MAX_OVER_THRES_TIMES) &&
							(process_monitor_5_in_10_array[j].set_warn == 0)) {
						process_monitor_5_in_10_array[j].set_warn = 1;
						process_monitor_5_in_10_array[j].ppid_name =
								ktop->task_ptr_array[*(ptr_top_loading + i)]->comm;
					}
					break;
				}
				
				else if (process_monitor_5_in_10_array[j].pid == 0) {
		                    process_monitor_5_in_10_array[j].pid = *(ptr_top_loading + i);
				    process_monitor_5_in_10_array[j].cnt++;
		                    break;
				}
			}
		}
	}

	if (statistic_monitor_period < HTC_KERNEL_TOP_MONITOR_PERIOD) {
	        
		statistic_monitor_period++;
	} else {
	        
#if SEND_KOBJECT_UEVENT_ENV_ENABLED
		
	        memset(buf_warn, 0x0, sizeof(buf_warn));
		strcpy(buf_warn, "");
#endif 
	        for (j = 0; j < SIZE_OF_PROCESS_MONITOR_5_IN_10_ARRAY; j++) {
			if (process_monitor_5_in_10_array[j].set_warn == 1) {
#if SEND_KOBJECT_UEVENT_ENV_ENABLED
				strcat(buf_warn, "PID=");
			        sprintf(buf_temp, "%d", process_monitor_5_in_10_array[j].pid);
				strcat(buf_warn, buf_temp);
			        strcat(buf_warn, ",0,0,0;");
#endif 
				pr_err("[K] CPU_Sniffer: PID=[%d], name=[%s], over-cpu-usage-threshold.\n",
				process_monitor_5_in_10_array[j].pid, process_monitor_5_in_10_array[j].ppid_name);
#if SEND_KOBJECT_UEVENT_ENV_ENABLED
				process_monitor_5_in_10_array[j].sent_uevent = 1;
				ok_to_send_uevent++;
#endif 
				rtn = 1;
			}
		}

#if SEND_KOBJECT_UEVENT_ENV_ENABLED
	        
		if (ok_to_send_uevent) {
	            
		    strcat(buf_warn, "PID=0,0,0,0;");
	            strcat(buf_warn, "#");
		    send_cpu_usage_stats_kobject_uevent(&buf_warn[0]);
	        }
#endif 

		if (pm_monitor_enabled)
	            pr_debug("[K] [KTOP] Reach the number of statistics monitor period.\n");
	        statistic_monitor_period = 1;
		clear_process_monitor_array(&process_monitor_5_in_10_array[0], SIZE_OF_PROCESS_MONITOR_5_IN_10_ARRAY);
	}

	return rtn;
}
#endif 

void htc_idle_stat_add(int sleep_mode, u32 time)
{
	int cpu = smp_processor_id();

	if (cpu < CONFIG_NR_CPUS) {
		switch (sleep_mode) {
		case MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT:
			htc_idle_stat[cpu][0].count++;
			htc_idle_stat[cpu][0].time += time;
			break;
		case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE:
			htc_idle_stat[cpu][1].count++;
			htc_idle_stat[cpu][1].time += time;
			break;
		case MSM_PM_SLEEP_MODE_POWER_COLLAPSE:
			htc_idle_stat[cpu][2].count++;
			htc_idle_stat[cpu][2].time += time;
			break;
		default:
			break;
		}
	}
}

static void htc_idle_stat_clear(void)
{
	memset(htc_idle_stat, 0, sizeof(htc_idle_stat));
}

static void htc_xo_vddmin_stat_show(void)
{
	uint32_t xo_count = 0, vddmin_count = 0;
	uint64_t xo_time = 0, vddmin_time = 0;
	static uint32_t prev_xo_count = 0, prev_vddmin_count = 0;
	static uint64_t prev_xo_time = 0, prev_vddmin_time = 0;

	if (htc_get_xo_vddmin_info(&xo_count, &xo_time, &vddmin_count, &vddmin_time)) {
		if (xo_count > prev_xo_count) {
			pr_info("[K] XO: %u, %llums\n", xo_count - prev_xo_count,
							xo_time - prev_xo_time);
			prev_xo_count = xo_count;
			prev_xo_time = xo_time;
		}

		if (vddmin_count > prev_vddmin_count) {
			pr_info("[K] Vdd-min: %u, %llums\n", vddmin_count - prev_vddmin_count,
							vddmin_time - prev_vddmin_time);
			prev_vddmin_count = vddmin_count;
			prev_vddmin_time = vddmin_time;
		}
	}
}

static void htc_idle_stat_show(void)
{
	int i = 0, cpu = 0;

	pr_info("[K] cpu_id\tcpu_state\tidle_count\tidle_time\n");
	for (cpu = 0; cpu < CONFIG_NR_CPUS; cpu++) {
		for (i = 0; i < 3; i++) {
			if (htc_idle_stat[cpu][i].count) {
				pr_info("[K]\t%d\tC%d\t\t%d\t\t%dms\n", cpu, i,
					htc_idle_stat[cpu][i].count, htc_idle_stat[cpu][i].time / 1000);
			}
		}
	}
	htc_xo_vddmin_stat_show();
	msm_rpm_dump_stat();
}

static void htc_debug_flag_show(void)
{

	unsigned int cfg = 0 ;
    
	if(get_tamper_sf() == 0){
		if((get_kernel_flag() & FORCE_CHARGE) || (get_kernel_flag() & Y_CABLE))
			cfg = 1 ;
		pr_info("[K] CFG:0x%x", cfg);
	 }
}

#ifdef arch_idle_time
static cputime64_t get_idle_time(int cpu)
{
        cputime64_t idle;

        idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
        if (cpu_online(cpu) && !nr_iowait_cpu(cpu))
                idle += arch_idle_time(cpu);
        return idle;
}

static cputime64_t get_iowait_time(int cpu)
{
        cputime64_t iowait;

        iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
        if (cpu_online(cpu) && nr_iowait_cpu(cpu))
                iowait += arch_idle_time(cpu);
        return iowait;
}

#else

static u64 get_idle_time(int cpu)
{
        u64 idle, idle_time = get_cpu_idle_time_us(cpu, NULL);

        if (idle_time == -1ULL)
                
                idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
        else
                idle = usecs_to_cputime64(idle_time);

        return idle;
}

static u64 get_iowait_time(int cpu)
{
        u64 iowait, iowait_time = get_cpu_iowait_time_us(cpu, NULL);

        if (iowait_time == -1ULL)
                
                iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
        else
                iowait = usecs_to_cputime64(iowait_time);

        return iowait;
}

#endif

static void get_all_cpustat(struct kernel_cpustat *cpu_stat)
{
	int cpu;

	if (!cpu_stat)
		return;

	memset(cpu_stat, 0, sizeof(struct kernel_cpustat));

	for_each_possible_cpu(cpu) {
		cpu_stat->cpustat[CPUTIME_USER] += kcpustat_cpu(cpu).cpustat[CPUTIME_USER];
		cpu_stat->cpustat[CPUTIME_NICE] += kcpustat_cpu(cpu).cpustat[CPUTIME_NICE];
		cpu_stat->cpustat[CPUTIME_SYSTEM] += kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM];
		cpu_stat->cpustat[CPUTIME_IDLE] += get_idle_time(cpu);
		cpu_stat->cpustat[CPUTIME_IOWAIT] += get_iowait_time(cpu);
		cpu_stat->cpustat[CPUTIME_IRQ] += kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
		cpu_stat->cpustat[CPUTIME_SOFTIRQ] += kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
		cpu_stat->cpustat[CPUTIME_STEAL] += kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL];
		cpu_stat->cpustat[CPUTIME_GUEST] += kcpustat_cpu(cpu).cpustat[CPUTIME_GUEST];
		cpu_stat->cpustat[CPUTIME_GUEST_NICE] += kcpustat_cpu(cpu).cpustat[CPUTIME_GUEST_NICE];
	}
}

static void sort_cputime_by_pid(int *source, int *pid_pos, int pid_cnt, int *result)
{
    int i = 0, j = 0, k = 0, l = 0;
    int pid_found = 0;

    for (i = 0; i < NUM_BUSY_THREAD_CHECK; i++) {
        result[i] = 0;
        if (i == 0) {
            for (j = 0; j < pid_cnt; j++) {
                k = pid_pos[j];
                
                if(source[result[i]] < source[k]) {
                    result[i] = k;
                }
            }
        } else {
            for (j = 0; j < pid_cnt; j++) {
                k = pid_pos[j];
                
                for (l = 0; l < i; l++) {
                    
                    if (result[l] == k) {
                        pid_found = 1;
                        break;
                    }
                }
                
                if (pid_found) {
                    pid_found = 0;
                    continue;
                }

                
                if(source[result[i]] < source[k]) {
                    result[i] = k;
                }
            }
        }
    }
}

static unsigned long htc_calculate_cpustat_time(struct kernel_cpustat curr_cpustat,
						struct kernel_cpustat prev_cpustat)
{
	unsigned long user_time = 0, system_time = 0, io_time = 0;
	unsigned long irq_time = 0, idle_time = 0;

	user_time = (unsigned long) ((curr_cpustat.cpustat[CPUTIME_USER] +
					curr_cpustat.cpustat[CPUTIME_NICE]) -
					(prev_cpustat.cpustat[CPUTIME_USER] +
					prev_cpustat.cpustat[CPUTIME_NICE]));
	system_time = (unsigned long) (curr_cpustat.cpustat[CPUTIME_SYSTEM] -
					prev_cpustat.cpustat[CPUTIME_SYSTEM]);
	io_time = (unsigned long) (curr_cpustat.cpustat[CPUTIME_IOWAIT] -
					prev_cpustat.cpustat[CPUTIME_IOWAIT]);
	irq_time = (unsigned long) ((curr_cpustat.cpustat[CPUTIME_IRQ] +
					curr_cpustat.cpustat[CPUTIME_SOFTIRQ]) -
					(prev_cpustat.cpustat[CPUTIME_IRQ] +
					 prev_cpustat.cpustat[CPUTIME_SOFTIRQ]));
	idle_time = (unsigned long) ((curr_cpustat.cpustat[CPUTIME_IDLE] > prev_cpustat.cpustat[CPUTIME_IDLE]) ?
					curr_cpustat.cpustat[CPUTIME_IDLE] - prev_cpustat.cpustat[CPUTIME_IDLE] : 0);
	idle_time += (unsigned long) ((curr_cpustat.cpustat[CPUTIME_STEAL] +
					curr_cpustat.cpustat[CPUTIME_GUEST]) -
					(prev_cpustat.cpustat[CPUTIME_STEAL] +
					prev_cpustat.cpustat[CPUTIME_GUEST]));

	return (user_time + system_time + io_time + irq_time + idle_time);
}

static void htc_kernel_top_cal(struct _htc_kernel_top *ktop, int type)
{
	int pid_cnt = 0;
	struct task_struct *process;
	struct task_cputime cputime;
	ulong flags;

	if (ktop->task_ptr_array == NULL ||
	    ktop->curr_proc_delta == NULL ||
	    ktop->curr_proc_pid == NULL ||
	    ktop->prev_proc_stat == NULL)
	  return;

	spin_lock_irqsave(&ktop->lock, flags);

	
	for_each_process(process) {
		thread_group_cputime(process, &cputime);
		if (process->pid < MAX_PID) {
			ktop->curr_proc_delta[process->pid] =
				(cputime.utime + cputime.stime) - ktop->prev_proc_stat[process->pid];
			ktop->task_ptr_array[process->pid] = process;

			if (ktop->curr_proc_delta[process->pid] > 0) {
				ktop->curr_proc_pid[pid_cnt] = process->pid;
				pid_cnt++;
			}
		}
	}
	sort_cputime_by_pid(ktop->curr_proc_delta, ktop->curr_proc_pid, pid_cnt, ktop->top_loading_pid);

	
	get_all_cpustat(&ktop->curr_cpustat);
	ktop->cpustat_time = htc_calculate_cpustat_time(ktop->curr_cpustat, ktop->prev_cpustat);

	if (type == KERNEL_TOP_ACCU) {
#if USE_STATISTICS_STRATEGY_CONTINUOUS_3
		htc_kernel_top_statistics_continuous_3(ktop);
#else 
		htc_kernel_top_statistics_5_in_10(ktop);
#endif
	}

	
	for_each_process(process) {
		if (process->pid < MAX_PID) {
			thread_group_cputime(process, &cputime);
			ktop->prev_proc_stat[process->pid] = cputime.stime + cputime.utime;
		}
	}
	memcpy(&ktop->prev_cpustat, &ktop->curr_cpustat, sizeof(struct kernel_cpustat));
	spin_unlock_irqrestore(&ktop->lock, flags);
}

static void htc_kernel_top_show(struct _htc_kernel_top *ktop, int type)
{
	int top_n_pid = 0, i;

	
	pr_info("[K]%sCPU Usage\t\tPID\t\tName\n", type == KERNEL_TOP_ACCU ? "[KTOP]" : " ");
	for (i = 0; i < NUM_BUSY_THREAD_CHECK; i++) {
		top_n_pid = ktop->top_loading_pid[i];
		pr_info("[K]%s%8lu%%\t\t%d\t\t%s\t\t%d\n", type == KERNEL_TOP_ACCU ? "[KTOP]" : " ",
			ktop->curr_proc_delta[top_n_pid] * 100 / ktop->cpustat_time,
			top_n_pid,
			ktop->task_ptr_array[top_n_pid]->comm,
			ktop->curr_proc_delta[top_n_pid]);
	}
	memset(ktop->curr_proc_delta, 0, sizeof(int) * MAX_PID);
	memset(ktop->task_ptr_array, 0, sizeof(int) * MAX_PID);
	memset(ktop->curr_proc_pid, 0, sizeof(int) * MAX_PID);
}

static void htc_pm_monitor_work_func(struct work_struct *work)
{
        struct _htc_kernel_top *ktop = container_of(work, struct _htc_kernel_top,
		                        dwork.work);
	struct timespec ts;
	struct rtc_time tm;

	if (!htc_pm_monitor_wq) {
		pr_info("[K] htc_pm_monitor_wq is unavaliable.\n");
		return;
	}

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec - (sys_tz.tz_minuteswest * 60), &tm);
	pr_info("[K][PM] hTC PM Statistic start (%02d-%02d %02d:%02d:%02d)\n",
		tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

	
	htc_show_interrupts();

	
	htc_idle_stat_show();
	htc_idle_stat_clear();

	
	htc_timer_stats_onoff('0');
	htc_timer_stats_show(300); 
	htc_timer_stats_onoff('1');

	
	htc_print_active_wakeup_sources();

	queue_delayed_work(htc_pm_monitor_wq, &ktop->dwork, msecs_to_jiffies(msm_htc_util_delay_time));
	htc_kernel_top_cal(ktop, KERNEL_TOP);
	htc_kernel_top_show(ktop, KERNEL_TOP);

	
	htc_debug_flag_show();
	pr_info("[K][PM] hTC PM Statistic done\n");
}

static void htc_kernel_top_accumulation_monitor_work_func(struct work_struct *work)
{
        struct _htc_kernel_top *ktop = container_of(work, struct _htc_kernel_top,
		                        dwork.work);
	struct timespec ts;
	struct rtc_time tm;

	if (htc_kernel_top_monitor_wq == NULL){
		if (pm_monitor_enabled)
			printk(KERN_INFO "[K] hTc Kernel Top statistic is NULL.\n");
		return;
	}

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec - (sys_tz.tz_minuteswest * 60), &tm);
	if (pm_monitor_enabled)
		printk("[K][KTOP] hTC Kernel Top Statistic start (%02d-%02d %02d:%02d:%02d) \n",
			tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

	queue_delayed_work(htc_kernel_top_monitor_wq, &ktop->dwork, msecs_to_jiffies(msm_htc_util_top_delay_time));
	htc_kernel_top_cal(ktop, KERNEL_TOP_ACCU);
	htc_kernel_top_show(ktop, KERNEL_TOP_ACCU);
	if (pm_monitor_enabled)
		printk("[K][KTOP] hTC Kernel Top Statistic done\n");
}

void htc_monitor_init(void)
{
	struct _htc_kernel_top *htc_kernel_top;
	struct _htc_kernel_top *htc_kernel_top_accu;

	if ((get_kernel_flag() & KERNEL_FLAG_PM_MONITOR) ||
		!(get_kernel_flag() & KERNEL_FLAG_TEST_PWR_SUPPLY)) {
		pm_monitor_enabled = 1;
	} else
		pm_monitor_enabled = 0;

	if (pm_monitor_enabled) {
		if (htc_pm_monitor_wq == NULL)
			
			htc_pm_monitor_wq = create_workqueue("htc_pm_monitor_wq");

		if (!htc_pm_monitor_wq)
			return;

		pr_info("[K] Success to create htc_pm_monitor_wq (0x%x).\n",
						(unsigned int) htc_pm_monitor_wq);
		htc_kernel_top = vmalloc(sizeof(*htc_kernel_top));
		spin_lock_init(&htc_kernel_top->lock);

		htc_kernel_top->prev_proc_stat = vmalloc(sizeof(int) * MAX_PID);
		htc_kernel_top->curr_proc_delta = vmalloc(sizeof(int) * MAX_PID);
		htc_kernel_top->task_ptr_array = vmalloc(sizeof(int) * MAX_PID);
		htc_kernel_top->curr_proc_pid = vmalloc(sizeof(int) * MAX_PID);

		memset(htc_kernel_top->prev_proc_stat, 0, sizeof(int) * MAX_PID);
		memset(htc_kernel_top->curr_proc_delta, 0, sizeof(int) * MAX_PID);
		memset(htc_kernel_top->task_ptr_array, 0, sizeof(int) * MAX_PID);
		memset(htc_kernel_top->curr_proc_pid, 0, sizeof(int) * MAX_PID);

		get_all_cpustat(&htc_kernel_top->curr_cpustat);
	        get_all_cpustat(&htc_kernel_top->prev_cpustat);

		INIT_DELAYED_WORK(&htc_kernel_top->dwork, htc_pm_monitor_work_func);
		queue_delayed_work(htc_pm_monitor_wq, &htc_kernel_top->dwork,
						msecs_to_jiffies(msm_htc_util_delay_time));
	}

	if (htc_kernel_top_monitor_wq == NULL) {
		
		htc_kernel_top_monitor_wq = create_workqueue("htc_kernel_top_monitor_wq");
		printk(KERN_INFO "[K][KTOP] Create HTC private workqueue(0x%x)...\n",
						(unsigned int)htc_kernel_top_monitor_wq);
	}

	if (!htc_kernel_top_monitor_wq)
		return;

	printk(KERN_INFO "[K][KTOP] Success to create htc_kernel_top_monitor_wq (0x%x).\n",
						(unsigned int)htc_kernel_top_monitor_wq);
#if USE_STATISTICS_STRATEGY_CONTINUOUS_3
	clear_current_pid_found_array();
	clear_process_monitor_array(&process_monitor_continuous_3_array[0],
						SIZE_OF_PROCESS_MONITOR_CONTINUOUS_3_ARRAY);
#else 
	clear_process_monitor_array(&process_monitor_5_in_10_array[0],
						SIZE_OF_PROCESS_MONITOR_5_IN_10_ARRAY);
#endif 

	htc_kernel_top_accu = vmalloc(sizeof(*htc_kernel_top_accu));
	spin_lock_init(&htc_kernel_top_accu->lock);

        htc_kernel_top_accu->prev_proc_stat = vmalloc(sizeof(int) * MAX_PID);
	htc_kernel_top_accu->curr_proc_delta = vmalloc(sizeof(int) * MAX_PID);
        htc_kernel_top_accu->task_ptr_array = vmalloc(sizeof(int) * MAX_PID);
	htc_kernel_top_accu->curr_proc_pid = vmalloc(sizeof(int) * MAX_PID);
        memset(htc_kernel_top_accu->prev_proc_stat, 0, sizeof(int) * MAX_PID);
        memset(htc_kernel_top_accu->curr_proc_delta, 0, sizeof(int) * MAX_PID);
        memset(htc_kernel_top_accu->task_ptr_array, 0, sizeof(int) * MAX_PID);
        memset(htc_kernel_top_accu->curr_proc_pid, 0, sizeof(int) * MAX_PID);

        get_all_cpustat(&htc_kernel_top_accu->curr_cpustat);
        get_all_cpustat(&htc_kernel_top_accu->prev_cpustat);
	INIT_DELAYED_WORK(&htc_kernel_top_accu->dwork,
					htc_kernel_top_accumulation_monitor_work_func);
	queue_delayed_work(htc_kernel_top_monitor_wq, &htc_kernel_top_accu->dwork,
						msecs_to_jiffies(msm_htc_util_top_delay_time));
}
