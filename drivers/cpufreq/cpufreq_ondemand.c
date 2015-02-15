/*
 *  drivers/cpufreq/cpufreq_ondemand.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *            (c)  2013 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>


#include <trace/events/cpufreq_interactive.h>


#define DEF_SAMPLING_RATE                              (50000)
#define DEF_FREQUENCY_DOWN_DIFFERENTIAL		(10)
#define DEF_FREQUENCY_UP_THRESHOLD		(80)
#define DEF_TWO_PHASE_FREQUENCY			(1200000)
#define DEF_SAMPLING_DOWN_FACTOR		(1)
#define MAX_SAMPLING_DOWN_FACTOR		(100000)
#define MICRO_FREQUENCY_DOWN_DIFFERENTIAL	(3)
#define MICRO_FREQUENCY_UP_THRESHOLD		(95)
#define MICRO_FREQUENCY_MIN_SAMPLE_RATE		(10000)
#define MIN_FREQUENCY_UP_THRESHOLD		(11)
#define MAX_FREQUENCY_UP_THRESHOLD		(100)
#define MIN_FREQUENCY_DOWN_DIFFERENTIAL		(1)
#define DBS_INPUT_EVENT_MIN_FREQ		(1036800)
#define DBS_SWITCH_MODE_TIMEOUT			(1000)
#define CPU0							(0)
#ifdef CONFIG_ARCH_MSM8974
#define DEF_FREQ_DOWN_STEP				(550000)
#define DEF_FREQ_DOWN_STEP_BARRIAR		(1728000)
#else
#define DEF_FREQ_DOWN_STEP				(0)
#define DEF_FREQ_DOWN_STEP_BARRIAR		(0)
#endif

#define MIN_SAMPLING_RATE_RATIO			(2)

static unsigned int min_sampling_rate;

#define LATENCY_MULTIPLIER			(1000)
#define MIN_LATENCY_MULTIPLIER			(100)
#define TRANSITION_LATENCY_LIMIT		(10 * 1000 * 1000)

#define POWERSAVE_BIAS_MAXLEVEL			(1000)
#define POWERSAVE_BIAS_MINLEVEL			(-1000)

static void do_dbs_timer(struct work_struct *work);
static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND
static
#endif
struct cpufreq_governor cpufreq_gov_ondemand = {
       .name                   = "ondemand",
       .governor               = cpufreq_governor_dbs,
       .max_transition_latency = TRANSITION_LATENCY_LIMIT,
       .owner                  = THIS_MODULE,
};

enum {DBS_NORMAL_SAMPLE, DBS_SUB_SAMPLE};

struct cpu_dbs_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_iowait;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	struct delayed_work work;
	struct cpufreq_frequency_table *freq_table;
	unsigned int freq_lo;
	unsigned int freq_lo_jiffies;
	unsigned int freq_hi_jiffies;
	unsigned int rate_mult;
	unsigned int prev_load;
	unsigned int max_load;
	int input_event_freq;
	int cpu;
	unsigned int sample_type:1;
	struct mutex timer_mutex;

#ifndef CONFIG_ARCH_MSM_CORTEXMP
	struct task_struct *sync_thread;
	wait_queue_head_t sync_wq;
	atomic_t src_sync_cpu;
	atomic_t sync_enabled;
#endif
};
static DEFINE_PER_CPU(struct cpu_dbs_info_s, od_cpu_dbs_info);

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info);
static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info);

static unsigned int dbs_enable;	

static DEFINE_PER_CPU(struct task_struct *, up_task);
static spinlock_t input_boost_lock;
static bool input_event_boost = false;
static unsigned long input_event_boost_expired = 0;

#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_2_PHASE
#define TABLE_SIZE			1
#else
#define TABLE_SIZE			5
#endif
#define MAX(x,y)			(x > y ? x : y)
#define MIN(x,y)			(x < y ? x : y)
#define FREQ_NEED_BUSRT(x)	(x < 600000 ? 1 : 0)

static struct cpufreq_frequency_table *tbl = NULL;
static unsigned int *tblmap[TABLE_SIZE] __read_mostly;
static unsigned int tbl_select[4] = {0};
static int input_event_counter = 0;

#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_2_PHASE
static unsigned int up_threshold_level[2] __read_mostly = {90, 90};
static inline void switch_turbo_mode(unsigned timeout) {};
static inline void switch_normal_mode(void) {};
static void reset_freq_map_table(struct cpufreq_policy *);
#else
static unsigned int up_threshold_level[2] __read_mostly = {95, 85};
struct timer_list freq_mode_timer;
static inline void switch_turbo_mode(unsigned);
static inline void switch_normal_mode(void);
static void reset_freq_map_table(struct cpufreq_policy *policy) {};
#endif

static DEFINE_MUTEX(dbs_mutex);

static struct workqueue_struct *dbs_wq;

static struct dbs_tuners {
	unsigned int sampling_rate;
	unsigned int up_threshold;
	unsigned int up_threshold_multi_core;
	unsigned int down_differential;
	unsigned int down_differential_multi_core;
	unsigned int optimal_freq;
	unsigned int up_threshold_any_cpu_load;
	unsigned int sync_freq;
	unsigned int ignore_nice;
	unsigned int sampling_down_factor;
	int          powersave_bias;
	unsigned int io_is_busy;
	unsigned int shortcut;
	unsigned int two_phase_freq;
	unsigned int freq_down_step;
	unsigned int freq_down_step_barriar;
} dbs_tuners_ins = {
	.up_threshold_multi_core = DEF_FREQUENCY_UP_THRESHOLD,
	.up_threshold = DEF_FREQUENCY_UP_THRESHOLD,
	.sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR,
	.down_differential = DEF_FREQUENCY_DOWN_DIFFERENTIAL,
	.down_differential_multi_core = MICRO_FREQUENCY_DOWN_DIFFERENTIAL,
	.up_threshold_any_cpu_load = DEF_FREQUENCY_UP_THRESHOLD,
	.ignore_nice = 0,
	.powersave_bias = 0,
	.sync_freq = 0,
	.optimal_freq = 0,
	.shortcut = 0,
	.two_phase_freq = DEF_TWO_PHASE_FREQUENCY,
	.freq_down_step = DEF_FREQ_DOWN_STEP,
	.freq_down_step_barriar = DEF_FREQ_DOWN_STEP_BARRIAR,
};

static inline u64 get_cpu_idle_time_jiffy(unsigned int cpu, u64 *wall)
{
	u64 idle_time;
	u64 cur_wall_time;
	u64 busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());

	busy_time  = kcpustat_cpu(cpu).cpustat[CPUTIME_USER];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_NICE];

	idle_time = cur_wall_time - busy_time;
	if (wall)
		*wall = jiffies_to_usecs(cur_wall_time);

	return jiffies_to_usecs(idle_time);
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu, cputime64_t *wall)
{
	u64 idle_time = get_cpu_idle_time_us(cpu, NULL);

	if (idle_time == -1ULL)
		return get_cpu_idle_time_jiffy(cpu, wall);
	else
		idle_time += get_cpu_iowait_time_us(cpu, wall);

	return idle_time;
}

static inline cputime64_t get_cpu_iowait_time(unsigned int cpu, cputime64_t *wall)
{
	u64 iowait_time = get_cpu_iowait_time_us(cpu, wall);

	if (iowait_time == -1ULL)
		return 0;

	return iowait_time;
}

static unsigned int powersave_bias_target(struct cpufreq_policy *policy,
					  unsigned int freq_next,
					  unsigned int relation)
{
	unsigned int freq_req, freq_avg;
	unsigned int freq_hi, freq_lo;
	unsigned int index = 0;
	unsigned int jiffies_total, jiffies_hi, jiffies_lo;
	int freq_reduc;
	struct cpu_dbs_info_s *dbs_info = &per_cpu(od_cpu_dbs_info,
						   policy->cpu);

	if (!dbs_info->freq_table) {
		dbs_info->freq_lo = 0;
		dbs_info->freq_lo_jiffies = 0;
		return freq_next;
	}

	cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_next,
			relation, &index);
	freq_req = dbs_info->freq_table[index].frequency;
	freq_reduc = freq_req * dbs_tuners_ins.powersave_bias / 1000;
	freq_avg = freq_req - freq_reduc;

	
	index = 0;
	cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_avg,
			CPUFREQ_RELATION_H, &index);
	freq_lo = dbs_info->freq_table[index].frequency;
	index = 0;
	cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_avg,
			CPUFREQ_RELATION_L, &index);
	freq_hi = dbs_info->freq_table[index].frequency;

	
	if (freq_hi == freq_lo) {
		dbs_info->freq_lo = 0;
		dbs_info->freq_lo_jiffies = 0;
		return freq_lo;
	}
	jiffies_total = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);
	jiffies_hi = (freq_avg - freq_lo) * jiffies_total;
	jiffies_hi += ((freq_hi - freq_lo) / 2);
	jiffies_hi /= (freq_hi - freq_lo);
	jiffies_lo = jiffies_total - jiffies_hi;
	dbs_info->freq_lo = freq_lo;
	dbs_info->freq_lo_jiffies = jiffies_lo;
	dbs_info->freq_hi_jiffies = jiffies_hi;
	return freq_hi;
}

static int ondemand_powersave_bias_setspeed(struct cpufreq_policy *policy,
					    struct cpufreq_policy *altpolicy,
					    int level)
{
	if (level == POWERSAVE_BIAS_MAXLEVEL) {
		
		__cpufreq_driver_target(policy,
			(altpolicy) ? altpolicy->min : policy->min,
			CPUFREQ_RELATION_L);
		return 1;
	} else if (level == POWERSAVE_BIAS_MINLEVEL) {
		
		__cpufreq_driver_target(policy,
			(altpolicy) ? altpolicy->max : policy->max,
			CPUFREQ_RELATION_H);
		return 1;
	}
	return 0;
}

static void ondemand_powersave_bias_init_cpu(int cpu)
{
	struct cpu_dbs_info_s *dbs_info = &per_cpu(od_cpu_dbs_info, cpu);
	dbs_info->freq_table = cpufreq_frequency_get_table(cpu);
	dbs_info->freq_lo = 0;
}

static void ondemand_powersave_bias_init(void)
{
	int i;
	for_each_online_cpu(i) {
		ondemand_powersave_bias_init_cpu(i);
	}
}


static ssize_t show_sampling_rate_min(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", min_sampling_rate);
}

define_one_global_ro(sampling_rate_min);

#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)              \
{									\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}
show_one(sampling_rate, sampling_rate);
show_one(io_is_busy, io_is_busy);
show_one(shortcut, shortcut);
show_one(up_threshold, up_threshold);
show_one(up_threshold_multi_core, up_threshold_multi_core);
show_one(down_differential, down_differential);
show_one(sampling_down_factor, sampling_down_factor);
show_one(ignore_nice_load, ignore_nice);
show_one(down_differential_multi_core, down_differential_multi_core);
show_one(optimal_freq, optimal_freq);
show_one(up_threshold_any_cpu_load, up_threshold_any_cpu_load);
show_one(sync_freq, sync_freq);
show_one(freq_down_step, freq_down_step);
show_one(freq_down_step_barriar, freq_down_step_barriar);

static ssize_t show_powersave_bias
(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", dbs_tuners_ins.powersave_bias);
}

static void update_sampling_rate(unsigned int new_rate)
{
	int cpu;

	dbs_tuners_ins.sampling_rate = new_rate
				     = max(new_rate, min_sampling_rate);

	get_online_cpus();
	for_each_online_cpu(cpu) {
		struct cpufreq_policy *policy;
		struct cpu_dbs_info_s *dbs_info;
		unsigned long next_sampling, appointed_at;

		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;
		dbs_info = &per_cpu(od_cpu_dbs_info, policy->cpu);
		cpufreq_cpu_put(policy);

		mutex_lock(&dbs_info->timer_mutex);

		if (!delayed_work_pending(&dbs_info->work)) {
			mutex_unlock(&dbs_info->timer_mutex);
			continue;
		}

		next_sampling  = jiffies + usecs_to_jiffies(new_rate);
		appointed_at = dbs_info->work.timer.expires;

		if (time_before(next_sampling, appointed_at)) {

			mutex_unlock(&dbs_info->timer_mutex);
			cancel_delayed_work_sync(&dbs_info->work);
			mutex_lock(&dbs_info->timer_mutex);

			queue_delayed_work_on(dbs_info->cpu, dbs_wq,
				&dbs_info->work, usecs_to_jiffies(new_rate));

		}
		mutex_unlock(&dbs_info->timer_mutex);
	}
	put_online_cpus();
}

static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	update_sampling_rate(input);
	return count;
}

static ssize_t store_sync_freq(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.sync_freq = input;
	return count;
}

static ssize_t store_io_is_busy(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.io_is_busy = !!input;
	return count;
}

static ssize_t store_shortcut(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (dbs_tuners_ins.shortcut != input) {
		dbs_tuners_ins.shortcut = input;
	}

	return count;
}

static ssize_t store_down_differential_multi_core(struct kobject *a,
			struct attribute *b, const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.down_differential_multi_core = input;
	return count;
}


static ssize_t store_optimal_freq(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.optimal_freq = input;
	return count;
}

static ssize_t store_up_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
			input < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}
	dbs_tuners_ins.up_threshold = input;
	return count;
}

static ssize_t store_up_threshold_multi_core(struct kobject *a,
			struct attribute *b, const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
			input < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}
	dbs_tuners_ins.up_threshold_multi_core = input;
	return count;
}

static ssize_t store_up_threshold_any_cpu_load(struct kobject *a,
			struct attribute *b, const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
			input < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}
	dbs_tuners_ins.up_threshold_any_cpu_load = input;
	return count;
}

static ssize_t store_down_differential(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input >= dbs_tuners_ins.up_threshold ||
			input < MIN_FREQUENCY_DOWN_DIFFERENTIAL) {
		return -EINVAL;
	}

	dbs_tuners_ins.down_differential = input;

	return count;
}

static ssize_t store_sampling_down_factor(struct kobject *a,
			struct attribute *b, const char *buf, size_t count)
{
	unsigned int input, j;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;
	dbs_tuners_ins.sampling_down_factor = input;

	
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->rate_mult = 1;
	}
	return count;
}

static ssize_t store_ignore_nice_load(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == dbs_tuners_ins.ignore_nice) { 
		return count;
	}
	dbs_tuners_ins.ignore_nice = input;

	
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&dbs_info->prev_cpu_wall);
		if (dbs_tuners_ins.ignore_nice)
			dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];

	}
	return count;
}

static ssize_t store_powersave_bias(struct kobject *a, struct attribute *b,
				    const char *buf, size_t count)
{
	int input  = 0;
	int bypass = 0;
	int ret, cpu, reenable_timer, j;
	struct cpu_dbs_info_s *dbs_info;

	struct cpumask cpus_timer_done;
	cpumask_clear(&cpus_timer_done);

	ret = sscanf(buf, "%d", &input);

	if (ret != 1)
		return -EINVAL;

	if (input >= POWERSAVE_BIAS_MAXLEVEL) {
		input  = POWERSAVE_BIAS_MAXLEVEL;
		bypass = 1;
	} else if (input <= POWERSAVE_BIAS_MINLEVEL) {
		input  = POWERSAVE_BIAS_MINLEVEL;
		bypass = 1;
	}

	if (input == dbs_tuners_ins.powersave_bias) {
		
		return count;
	}

	reenable_timer = ((dbs_tuners_ins.powersave_bias ==
				POWERSAVE_BIAS_MAXLEVEL) ||
				(dbs_tuners_ins.powersave_bias ==
				POWERSAVE_BIAS_MINLEVEL));

	dbs_tuners_ins.powersave_bias = input;

	get_online_cpus();
	mutex_lock(&dbs_mutex);

	if (!bypass) {
		if (reenable_timer) {
			
			for_each_online_cpu(cpu) {
				if (lock_policy_rwsem_write(cpu) < 0)
					continue;

				dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

				if (!dbs_info->cur_policy) {
					pr_err("Dbs policy is NULL\n");
					goto skip_this_cpu;
				}

				for_each_cpu(j, &cpus_timer_done) {
					if (cpumask_test_cpu(j, dbs_info->
							cur_policy->cpus))
						goto skip_this_cpu;
				}

				cpumask_set_cpu(cpu, &cpus_timer_done);
				if (dbs_info->cur_policy) {
					dbs_timer_exit(dbs_info);
					
					mutex_lock(&dbs_info->timer_mutex);
					dbs_timer_init(dbs_info);
#ifndef CONFIG_ARCH_MSM_CORTEXMP
					mutex_unlock(&dbs_info->timer_mutex);
					atomic_set(&dbs_info->sync_enabled, 1);
#endif
				}
skip_this_cpu:
				unlock_policy_rwsem_write(cpu);
			}
		}
		ondemand_powersave_bias_init();
	} else {
		for_each_online_cpu(cpu) {
			if (lock_policy_rwsem_write(cpu) < 0)
				continue;

			dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

			if (!dbs_info->cur_policy) {
				pr_err("Dbs policy is NULL\n");
				goto skip_this_cpu_bypass;
			}

			for_each_cpu(j, &cpus_timer_done) {
				if (cpumask_test_cpu(j, dbs_info->
							cur_policy->cpus))
					goto skip_this_cpu_bypass;
			}

			cpumask_set_cpu(cpu, &cpus_timer_done);

			if (dbs_info->cur_policy) {
				
				dbs_timer_exit(dbs_info);
#ifndef CONFIG_ARCH_MSM_CORTEXMP
				atomic_set(&dbs_info->sync_enabled, 0);
#endif

				mutex_lock(&dbs_info->timer_mutex);
				ondemand_powersave_bias_setspeed(
					dbs_info->cur_policy,
					NULL,
					input);
				mutex_unlock(&dbs_info->timer_mutex);

			}
skip_this_cpu_bypass:
			unlock_policy_rwsem_write(cpu);
		}
	}

	mutex_unlock(&dbs_mutex);
	put_online_cpus();

	return count;
}

static int input_event_min_freq_array[NR_CPUS] = {[0 ... NR_CPUS-1] = DBS_INPUT_EVENT_MIN_FREQ} ;

static ssize_t show_input_event_min_freq(struct kobject *kobj, struct attribute *attr, char *buf)
{
	int i = 0;
	int shift = 0;
	char *buf_pos = buf;
	for ( i = 0 ; i < NR_CPUS; i++) {
		shift = sprintf(buf_pos,"%d,",input_event_min_freq_array[i]);
		buf_pos += shift;
	}
	*(buf_pos-1) = '\n';
	*buf_pos = '\0';
	buf_pos++;
	return strlen(buf);
}

static ssize_t store_input_event_min_freq(struct kobject *a, struct attribute *b,
		const char *buf, size_t count)
{
	int ret = 0;
	if (NR_CPUS == 1)
		ret = sscanf(buf,"%u",&input_event_min_freq_array[0]);
	else if (NR_CPUS == 2)
		ret = sscanf(buf,"%u,%u",&input_event_min_freq_array[0],
				&input_event_min_freq_array[1]);
	else if (NR_CPUS == 4)
		ret = sscanf(buf, "%u,%u,%u,%u", &input_event_min_freq_array[0],
				&input_event_min_freq_array[1],
				&input_event_min_freq_array[2],
				&input_event_min_freq_array[3]);
	if (ret < NR_CPUS)
		return -EINVAL;

	return count;
}

int freq_cnt = 0;
static ssize_t show_multi_phase_freq_tbl(struct kobject *kobj, struct attribute *attr,
				char *buf)
{
	int i = 0,j = 0;
	int shift = 0;
	char *buf_pos = buf;

	for (i = 0; i < TABLE_SIZE; i++) {
		shift = snprintf(buf_pos,PAGE_SIZE-(buf_pos - buf),"%s %d %s","Table",i+1,"shows:\n");
		buf_pos += shift;
		for (j = 0; j < freq_cnt; j++) {
			shift = snprintf(buf_pos,PAGE_SIZE-(buf_pos - buf),"%02d: %8u\n",j,tblmap[i][j]);
			buf_pos += shift;
		}
	}

	*(buf_pos) = '\0';

	return strlen(buf);
}

static ssize_t store_multi_phase_freq_tbl(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{

	return 0;
}

show_one(two_phase_freq, two_phase_freq);

static ssize_t store_two_phase_freq(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	struct cpu_dbs_info_s *dbs_info;
	struct cpufreq_policy *policy;
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	
	dbs_info = &per_cpu(od_cpu_dbs_info, 0);
	policy = dbs_info->cur_policy;
	if (policy) {
		if (input < policy->cpuinfo.min_freq || input > policy->cpuinfo.max_freq)
			return -EINVAL;
		if (dbs_tuners_ins.two_phase_freq != input) {
			dbs_tuners_ins.two_phase_freq = input;
			reset_freq_map_table(policy);
		}
	}

	return count;
}

static ssize_t store_freq_down_step(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.freq_down_step = input;
	return count;
}

static ssize_t store_freq_down_step_barriar(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.freq_down_step_barriar = input;
	return count;
}

define_one_global_rw(sampling_rate);
define_one_global_rw(io_is_busy);
define_one_global_rw(shortcut);
define_one_global_rw(up_threshold);
define_one_global_rw(down_differential);
define_one_global_rw(sampling_down_factor);
define_one_global_rw(ignore_nice_load);
define_one_global_rw(powersave_bias);
define_one_global_rw(up_threshold_multi_core);
define_one_global_rw(down_differential_multi_core);
define_one_global_rw(optimal_freq);
define_one_global_rw(up_threshold_any_cpu_load);
define_one_global_rw(sync_freq);
define_one_global_rw(input_event_min_freq);
define_one_global_rw(multi_phase_freq_tbl);
define_one_global_rw(two_phase_freq);
define_one_global_rw(freq_down_step);
define_one_global_rw(freq_down_step_barriar);

static struct attribute *dbs_attributes[] = {
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&up_threshold.attr,
	&down_differential.attr,
	&sampling_down_factor.attr,
	&ignore_nice_load.attr,
	&powersave_bias.attr,
	&io_is_busy.attr,
	&shortcut.attr,
	&up_threshold_multi_core.attr,
	&down_differential_multi_core.attr,
	&optimal_freq.attr,
	&up_threshold_any_cpu_load.attr,
	&sync_freq.attr,
	&input_event_min_freq.attr,
	&multi_phase_freq_tbl.attr,
	&two_phase_freq.attr,
	&freq_down_step.attr,
	&freq_down_step_barriar.attr,
	NULL
};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = "ondemand",
};


#ifndef CONFIG_CPU_FREQ_GOV_ONDEMAND_2_PHASE
static inline void switch_turbo_mode(unsigned timeout)
{
	if (timeout > 0)
		mod_timer(&freq_mode_timer, jiffies + msecs_to_jiffies(timeout));

	tbl_select[0] = 2;
	tbl_select[1] = 3;
	tbl_select[2] = 4;
	tbl_select[3] = 4;
}

static inline void switch_normal_mode(void)
{
	if (input_event_counter > 0)
		return;

	tbl_select[0] = 0;
	tbl_select[1] = 1;
	tbl_select[2] = 2;
	tbl_select[3] = 3;
}

static void switch_mode_timer(unsigned long data)
{
	switch_normal_mode();
}
#endif

static int adjust_freq_map_table(int freq, int cnt, struct cpufreq_policy *policy)
{
	int i, upper, lower;

	if (policy && tbl) {
		upper = policy->cpuinfo.max_freq;
		lower = policy->cpuinfo.min_freq;
	}
	else
		return freq;

	for(i = 0; i < cnt; i++)
	{
		if(freq >= tbl[i].frequency)
		{
			lower = tbl[i].frequency;
		}
		else
		{
			upper = tbl[i].frequency;
			break;
		}
	}

	return (freq - lower < upper - freq)?lower:upper;
}

#ifdef CONFIG_CPU_FREQ_GOV_ONDEMAND_2_PHASE

static void reset_freq_map_table(struct cpufreq_policy *policy)
{
	unsigned int real_freq;
	int index;

	if (!tbl || !tblmap[0])
		return;

	
	real_freq = adjust_freq_map_table(dbs_tuners_ins.two_phase_freq, freq_cnt, policy);

	for (index = 0; index < freq_cnt; index++) {
		
		if (tbl[index].frequency < real_freq)
			tblmap[0][index] = real_freq;

		
		else if (tbl[index].frequency == real_freq && index != freq_cnt-1)
			tblmap[0][index] = tbl[index+1].frequency;

		
		else
			tblmap[0][index] = policy->cpuinfo.max_freq;
	}
}

static void dbs_init_freq_map_table(struct cpufreq_policy *policy)
{
	tbl = cpufreq_frequency_get_table(0);

	for (freq_cnt = 0; (tbl[freq_cnt].frequency != CPUFREQ_TABLE_END); freq_cnt++)
		;

	tblmap[0] = kmalloc(sizeof(unsigned int) * freq_cnt, GFP_KERNEL);
	BUG_ON(!tblmap[0]);

	reset_freq_map_table(policy);
}

#else

static void dbs_init_freq_map_table(struct cpufreq_policy *policy)
{
	unsigned int min_diff, top1, top2;
	int i, j;

	tbl = cpufreq_frequency_get_table(0);
	min_diff = policy->cpuinfo.max_freq;

	
	for (freq_cnt = 0; (tbl[freq_cnt].frequency != CPUFREQ_TABLE_END); freq_cnt++) {
		if (freq_cnt > 0)
			min_diff = MIN(tbl[freq_cnt].frequency - tbl[freq_cnt-1].frequency, min_diff);
	}

	
	top1 = (policy->cpuinfo.max_freq + policy->cpuinfo.min_freq) / 2;
	top2 = (policy->cpuinfo.max_freq + top1) / 2;

	for (i = 0; i < TABLE_SIZE; i++) {
		
		tblmap[i] = kmalloc(sizeof(unsigned int) * freq_cnt, GFP_KERNEL);
		BUG_ON(!tblmap[i]);
		
		for (j = 0; j < freq_cnt; j++)
			tblmap[i][j] = tbl[j].frequency;
	}

	for (j = 0; j < freq_cnt; j++) {
		
		if (tbl[j].frequency < top1) {
			tblmap[0][j] += MAX((top1 - tbl[j].frequency)/3, min_diff);
		}
		
		if (tbl[j].frequency < top2) {
			tblmap[1][j] += MAX((top2 - tbl[j].frequency)/3, min_diff);
			tblmap[2][j] += MAX(((top2 - tbl[j].frequency)*2)/5, min_diff);
			tblmap[3][j] += MAX((top2 - tbl[j].frequency)/2, min_diff);
		}
		else {
			tblmap[3][j] += MAX((policy->cpuinfo.max_freq - tbl[j].frequency)/3, min_diff);
		}
		
		tblmap[4][j] += MAX((policy->cpuinfo.max_freq - tbl[j].frequency)/2, min_diff);
	}

	for (i = 0; i < TABLE_SIZE; i++) {
		for (j = 0; j < freq_cnt; j++)
			tblmap[i][j] = adjust_freq_map_table(tblmap[i][j], freq_cnt, policy);
	}

	for (j = 0; j < freq_cnt - 1; j++) {
		if (tblmap[3][j] == tbl[j].frequency)
			tblmap[3][j] = tbl[j+1].frequency;
	}

	switch_normal_mode();

	
	init_timer(&freq_mode_timer);
	freq_mode_timer.function = switch_mode_timer;
	freq_mode_timer.data = 0;

#if 0
	
	for (i = 0; i < TABLE_SIZE; i++) {
		pr_info("Table %d shows:\n", i+1);
		for (j = 0; j < freq_cnt; j++) {
			pr_info("%02d: %8u\n", j, tblmap[i][j]);
		}
	}
#endif
}
#endif

static void dbs_deinit_freq_map_table(void)
{
	int i;

	if (!tbl)
		return;

	tbl = NULL;

	for (i = 0; i < TABLE_SIZE; i++)
		kfree(tblmap[i]);

#ifndef CONFIG_CPU_FREQ_GOV_ONDEMAND_2_PHASE
	del_timer(&freq_mode_timer);
#endif
}

static inline int get_cpu_freq_index(unsigned int freq)
{
	static int saved_index = 0;
	int index;

	if (!tbl) {
		pr_warn("tbl is NULL, use previous value %d\n", saved_index);
		return saved_index;
	}

	for (index = 0; (tbl[index].frequency != CPUFREQ_TABLE_END); index++) {
		if (tbl[index].frequency >= freq) {
			saved_index = index;
			break;
		}
	}

	return index;
}

static void dbs_freq_increase(struct cpufreq_policy *p, unsigned load, unsigned int freq)
{
	if (dbs_tuners_ins.powersave_bias)
		freq = powersave_bias_target(p, freq, CPUFREQ_RELATION_H);
	else if (p->cur == p->max){
		trace_cpufreq_interactive_already (p->cpu, load, p->cur, p->cur, p->max);
		return;
	}

	trace_cpufreq_interactive_target (p->cpu, load, freq, p->cur, freq);

	__cpufreq_driver_target(p, freq, (dbs_tuners_ins.powersave_bias || freq < p->max) ?
			CPUFREQ_RELATION_L : CPUFREQ_RELATION_H);

	trace_cpufreq_interactive_setspeed (p->cpu, freq, p->cur);
}

int input_event_boosted(void)
{
	unsigned long flags;

	
	spin_lock_irqsave(&input_boost_lock, flags);
	if (input_event_boost) {
		if (time_before(jiffies, input_event_boost_expired)) {
			spin_unlock_irqrestore(&input_boost_lock, flags);
			return 1;
		}
		input_event_boost = false;
	}
	spin_unlock_irqrestore(&input_boost_lock, flags);

	return 0;
}

static void dbs_check_cpu(struct cpu_dbs_info_s *this_dbs_info)
{
	unsigned int max_load_freq;
	unsigned int max_load_other_cpu = 0;
	struct cpufreq_policy *policy;
	unsigned int j, max_cur_load = 0, prev_load = 0;
	struct cpu_dbs_info_s *j_dbs_info;
	unsigned int up_threshold = 85;

	this_dbs_info->freq_lo = 0;
	policy = this_dbs_info->cur_policy;

#ifdef CONFIG_ARCH_MSM_CORTEXMP
	for (j = 1; j < NR_CPUS; j++) {
		j_dbs_info = &per_cpu(od_cpu_dbs_info, j);
		if (j_dbs_info->prev_load && !cpu_online(j))
			j_dbs_info->prev_load = 0;
	}
#endif


	
	max_load_freq = 0;

	for_each_cpu(j, policy->cpus) {
		cputime64_t cur_wall_time, cur_idle_time, cur_iowait_time;
		unsigned int idle_time, wall_time, iowait_time;
		unsigned int load_freq, cur_load;
		int freq_avg;

		j_dbs_info = &per_cpu(od_cpu_dbs_info, j);

		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time);
		cur_iowait_time = get_cpu_iowait_time(j, &cur_wall_time);

		wall_time = (unsigned int)
			(cur_wall_time - j_dbs_info->prev_cpu_wall);
		j_dbs_info->prev_cpu_wall = cur_wall_time;

		idle_time = (unsigned int)
			(cur_idle_time - j_dbs_info->prev_cpu_idle);
		j_dbs_info->prev_cpu_idle = cur_idle_time;

		iowait_time = (unsigned int)
			(cur_iowait_time - j_dbs_info->prev_cpu_iowait);
		j_dbs_info->prev_cpu_iowait = cur_iowait_time;

		if (dbs_tuners_ins.ignore_nice) {
			u64 cur_nice;
			unsigned long cur_nice_jiffies;

			cur_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE] -
					 j_dbs_info->prev_cpu_nice;
			cur_nice_jiffies = (unsigned long)
					cputime64_to_jiffies64(cur_nice);

			j_dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
			idle_time += jiffies_to_usecs(cur_nice_jiffies);
		}


		if (dbs_tuners_ins.io_is_busy && idle_time >= iowait_time)
			idle_time -= iowait_time;

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		if (prev_load < j_dbs_info->prev_load)
			prev_load = j_dbs_info->prev_load;

		cur_load = 100 * (wall_time - idle_time) / wall_time;
		j_dbs_info->max_load  = max(cur_load, j_dbs_info->prev_load);
		j_dbs_info->prev_load = cur_load;
		freq_avg = __cpufreq_driver_getavg(policy, j);
		if (freq_avg <= 0)
			freq_avg = policy->cur;

		load_freq = cur_load * freq_avg;
		if (load_freq > max_load_freq) {
			max_load_freq = load_freq;
			max_cur_load = cur_load;
		}
	}

	for_each_online_cpu(j) {
		j_dbs_info = &per_cpu(od_cpu_dbs_info, j);

		if (j == policy->cpu)
			continue;

		if (max_load_other_cpu < j_dbs_info->max_load)
			max_load_other_cpu = j_dbs_info->max_load;

		if ((j_dbs_info->cur_policy != NULL)
			&& (j_dbs_info->cur_policy->cur ==
					j_dbs_info->cur_policy->max)) {

			if (policy->cur >= dbs_tuners_ins.optimal_freq)
				max_load_other_cpu =
				dbs_tuners_ins.up_threshold_any_cpu_load;
		}
	}

	if (dbs_tuners_ins.shortcut)
		up_threshold = dbs_tuners_ins.up_threshold;
	else
		up_threshold = up_threshold_level[1];

	
	if (max_load_freq > up_threshold * policy->cur) {
		unsigned int freq_next;
		unsigned int avg_load;
		int index;

		
		if (dbs_tuners_ins.shortcut) {
			freq_next = policy->cpuinfo.max_freq;
			goto set_freq;
		}

		avg_load = (prev_load + max_cur_load) >> 1;
		index = get_cpu_freq_index(policy->cur);

		
		if (FREQ_NEED_BUSRT(policy->cur) && max_cur_load > up_threshold_level[0]) {
			freq_next = tblmap[tbl_select[3]][index];
		}
		
		else if (prev_load == 0) {
			if (max_cur_load > up_threshold_level[0])
				freq_next = tblmap[tbl_select[3]][index];
			else
				freq_next = tblmap[tbl_select[2]][index];
		}
		
		else if (avg_load > up_threshold_level[0]) {
			freq_next = tblmap[tbl_select[3]][index];
		}
		
		else if (avg_load <= up_threshold_level[1]) {
			freq_next = tblmap[tbl_select[0]][index];
		}
		
		else {
			
			if (max_cur_load > up_threshold_level[0]) {
				freq_next = tblmap[tbl_select[2]][index];
			}
			
			else {
				freq_next = tblmap[tbl_select[1]][index];
			}
		}

set_freq:
		dbs_freq_increase(policy, max_cur_load, freq_next);
		
		if (policy->cur == policy->max)
			this_dbs_info->rate_mult = dbs_tuners_ins.sampling_down_factor;
		return;
	}

	if (input_event_boosted()) {
		trace_cpufreq_interactive_already (policy->cpu, max_cur_load, policy->cur, policy->cur, policy->cur);
		return;
	}

	if (num_online_cpus() > 1) {

#ifndef CONFIG_ARCH_MSM_CORTEXMP
		if (max_load_other_cpu >
				dbs_tuners_ins.up_threshold_any_cpu_load) {
			if (policy->cur < dbs_tuners_ins.sync_freq)
				dbs_freq_increase(policy, max_cur_load,
						dbs_tuners_ins.sync_freq);
			else
				trace_cpufreq_interactive_already (policy->cpu, max_cur_load, policy->cur, policy->cur, policy->cur);
			return;
		}
#endif

		if (max_load_freq > dbs_tuners_ins.up_threshold_multi_core *
								policy->cur) {
			if (policy->cur < dbs_tuners_ins.optimal_freq)
				dbs_freq_increase(policy, max_cur_load,
						dbs_tuners_ins.optimal_freq);
			else
				trace_cpufreq_interactive_already (policy->cpu, max_cur_load, policy->cur, policy->cur, policy->cur);
			return;
		}
	}

	
	
	if (policy->cur == policy->min){
		trace_cpufreq_interactive_already (policy->cpu, max_cur_load, policy->cur, policy->cur,policy->min);
		return;
	}

	if (max_load_freq <
	    (dbs_tuners_ins.up_threshold - dbs_tuners_ins.down_differential) *
	     policy->cur) {
		unsigned int freq_next;
		freq_next = max_load_freq /
				(dbs_tuners_ins.up_threshold -
				 dbs_tuners_ins.down_differential);

		
		this_dbs_info->rate_mult = 1;

		if (freq_next < policy->min)
			freq_next = policy->min;

		if (num_online_cpus() > 1) {
#ifndef CONFIG_ARCH_MSM_CORTEXMP
			if (dbs_tuners_ins.sync_freq > policy->min && max_load_other_cpu >
			    (dbs_tuners_ins.up_threshold_multi_core -
			    dbs_tuners_ins.down_differential) &&
			    freq_next < dbs_tuners_ins.sync_freq)
				freq_next = dbs_tuners_ins.sync_freq;
#endif
			if (dbs_tuners_ins.optimal_freq > policy->min && max_load_freq >
			    ((dbs_tuners_ins.up_threshold_multi_core -
			    dbs_tuners_ins.down_differential_multi_core) *
			    policy->cur) &&
			    freq_next < dbs_tuners_ins.optimal_freq)
				freq_next = dbs_tuners_ins.optimal_freq;
		}

		if (dbs_tuners_ins.powersave_bias) {
			freq_next = powersave_bias_target(policy, freq_next,
					CPUFREQ_RELATION_L);
		}

		if (dbs_tuners_ins.freq_down_step) {
			unsigned int new_freq_next = freq_next;
			if ((policy->cur - freq_next) > dbs_tuners_ins.freq_down_step) {
				new_freq_next = policy->cur - dbs_tuners_ins.freq_down_step;
			}

			if (dbs_tuners_ins.freq_down_step_barriar) {
				if (dbs_tuners_ins.freq_down_step_barriar < new_freq_next) {
					new_freq_next = dbs_tuners_ins.freq_down_step_barriar;
				}

				if (policy->cur <= dbs_tuners_ins.freq_down_step_barriar) {
					new_freq_next = freq_next;
				}
			}

			freq_next = new_freq_next;
		}

		trace_cpufreq_interactive_target(policy->cpu, max_cur_load, freq_next, policy->cur, freq_next);
		__cpufreq_driver_target(policy, freq_next,
			CPUFREQ_RELATION_L);

		trace_cpufreq_interactive_setspeed(policy->cpu, freq_next, policy->cur);
	}
}

static void do_dbs_timer(struct work_struct *work)
{
	struct cpu_dbs_info_s *dbs_info =
		container_of(work, struct cpu_dbs_info_s, work.work);
	unsigned int cpu = dbs_info->cpu;
	int sample_type = dbs_info->sample_type;

	int delay;

	if (unlikely(!cpu_online(cpu) || !dbs_info->cur_policy))
		return;

	mutex_lock(&dbs_info->timer_mutex);

	
	dbs_info->sample_type = DBS_NORMAL_SAMPLE;
	if (!dbs_tuners_ins.powersave_bias ||
	    sample_type == DBS_NORMAL_SAMPLE) {
		dbs_check_cpu(dbs_info);
		if (dbs_info->freq_lo) {
			
			dbs_info->sample_type = DBS_SUB_SAMPLE;
			delay = dbs_info->freq_hi_jiffies;
		} else {
			delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate
				* dbs_info->rate_mult);
		}
	} else {
		delay = dbs_info->freq_lo_jiffies;
		if (input_event_boosted())
			goto sched_wait;

		__cpufreq_driver_target(dbs_info->cur_policy,
			dbs_info->freq_lo, CPUFREQ_RELATION_H);
	}

sched_wait:
	queue_delayed_work_on(cpu, dbs_wq, &dbs_info->work, delay);
	mutex_unlock(&dbs_info->timer_mutex);
}

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info)
{
	
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);

	if (num_online_cpus() > 1)
		delay -= jiffies % delay;

	dbs_info->sample_type = DBS_NORMAL_SAMPLE;
	INIT_DELAYED_WORK_DEFERRABLE(&dbs_info->work, do_dbs_timer);
	queue_delayed_work_on(dbs_info->cpu, dbs_wq, &dbs_info->work, delay);
}

static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info)
{
	cancel_delayed_work_sync(&dbs_info->work);
}

static int should_io_be_busy(void)
{
#if defined(CONFIG_X86)
	if (boot_cpu_data.x86_vendor == X86_VENDOR_INTEL &&
	    boot_cpu_data.x86 == 6 &&
	    boot_cpu_data.x86_model >= 15)
		return 1;
#endif
	return 0;
}

#ifndef CONFIG_ARCH_MSM_CORTEXMP
static int dbs_migration_notify(struct notifier_block *nb,
				unsigned long target_cpu, void *arg)
{
	struct cpu_dbs_info_s *target_dbs_info =
		&per_cpu(od_cpu_dbs_info, target_cpu);

	atomic_set(&target_dbs_info->src_sync_cpu, (int)arg);
	wake_up(&target_dbs_info->sync_wq);

	return NOTIFY_OK;
}

static struct notifier_block dbs_migration_nb = {
	.notifier_call = dbs_migration_notify,
};

static int sync_pending(struct cpu_dbs_info_s *this_dbs_info)
{
	return atomic_read(&this_dbs_info->src_sync_cpu) >= 0;
}

static int dbs_sync_thread(void *data)
{
	int src_cpu, cpu = (int)data;
	unsigned int src_freq, src_max_load;
	struct cpu_dbs_info_s *this_dbs_info, *src_dbs_info;
	struct cpufreq_policy *policy;
	int delay;

	this_dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

	while (1) {
		wait_event(this_dbs_info->sync_wq,
			   sync_pending(this_dbs_info) ||
			   kthread_should_stop());

		if (kthread_should_stop())
			break;

		get_online_cpus();

		src_cpu = atomic_read(&this_dbs_info->src_sync_cpu);
		src_dbs_info = &per_cpu(od_cpu_dbs_info, src_cpu);
		if (src_dbs_info != NULL &&
		    src_dbs_info->cur_policy != NULL) {
			src_freq = src_dbs_info->cur_policy->cur;
			src_max_load = src_dbs_info->max_load;
		} else {
			src_freq = dbs_tuners_ins.sync_freq;
			src_max_load = 0;
		}

		if (lock_policy_rwsem_write(cpu) < 0)
			goto bail_acq_sema_failed;

		if (!atomic_read(&this_dbs_info->sync_enabled)) {
			atomic_set(&this_dbs_info->src_sync_cpu, -1);
			unlock_policy_rwsem_write(cpu);
			put_online_cpus();
			continue;
		}

		policy = this_dbs_info->cur_policy;
		if (!policy) {
			
			goto bail_incorrect_governor;
		}
		delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);


		if (policy->cur < src_freq) {
			
			cancel_delayed_work_sync(&this_dbs_info->work);

			if (__cpufreq_driver_target(policy, src_freq,
						    CPUFREQ_RELATION_L) >= 0) {
				if (src_max_load > this_dbs_info->max_load) {
					this_dbs_info->max_load = src_max_load;
					this_dbs_info->prev_load = src_max_load;
				}
			}

			
			mutex_lock(&this_dbs_info->timer_mutex);
			queue_delayed_work_on(cpu, dbs_wq,
					      &this_dbs_info->work, delay);
			mutex_unlock(&this_dbs_info->timer_mutex);
		}

bail_incorrect_governor:
		unlock_policy_rwsem_write(cpu);
bail_acq_sema_failed:
		put_online_cpus();
		atomic_set(&this_dbs_info->src_sync_cpu, -1);
	}

	return 0;
}
#endif

static void dbs_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	int i;
	struct cpu_dbs_info_s *dbs_info;
	unsigned long flags;
	int input_event_min_freq = 0;

	if ((dbs_tuners_ins.powersave_bias == POWERSAVE_BIAS_MAXLEVEL) ||
		(dbs_tuners_ins.powersave_bias == POWERSAVE_BIAS_MINLEVEL)) {
		
		return;
	}

	if (type == EV_SYN && code == SYN_REPORT) {
		
		dbs_tuners_ins.powersave_bias = 0;
	}
	else if (type == EV_ABS && code == ABS_MT_TRACKING_ID) {
		
		if (value != -1) {
			input_event_counter++;
			input_event_min_freq = input_event_min_freq_array[num_online_cpus() - 1];
			
			switch_turbo_mode(0);
		}
		
		else {
			if (likely(input_event_counter > 0))
				input_event_counter--;
			else
				pr_warning("dbs_input_event: Touch isn't paired!\n");

			input_event_min_freq = 0;
			
			switch_turbo_mode(DBS_SWITCH_MODE_TIMEOUT);
		}
	}
	else if (type == EV_KEY && value == 1 &&
			(code == KEY_POWER || code == KEY_VOLUMEUP || code == KEY_VOLUMEDOWN))
	{
		input_event_min_freq = input_event_min_freq_array[num_online_cpus() - 1];
		
		switch_turbo_mode(DBS_SWITCH_MODE_TIMEOUT);
	}

	if (input_event_min_freq > 0) {
		
		spin_lock_irqsave(&input_boost_lock, flags);
		input_event_boost = true;
		input_event_boost_expired = jiffies + usecs_to_jiffies(dbs_tuners_ins.sampling_rate * 4);
		spin_unlock_irqrestore(&input_boost_lock, flags);

		for_each_online_cpu(i)
		{
#ifdef CONFIG_ARCH_MSM_CORTEXMP
			if (i != CPU0)
				break;
#endif
			dbs_info = &per_cpu(od_cpu_dbs_info, i);
			 if (dbs_info->cur_policy &&             
				dbs_info->cur_policy->cur < input_event_min_freq) {
				dbs_info->input_event_freq = input_event_min_freq;
				wake_up_process(per_cpu(up_task, i));
			}
		}
	}
}

static int input_dev_filter(const char *input_dev_name)
{
	if (strstr(input_dev_name, "touchscreen") ||
	    strstr(input_dev_name, "keypad")) {
		return 0; 
	} else {
		return 1;
	}
}

static int dbs_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	
	if (input_dev_filter(dev->name))
		return -ENODEV;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpufreq";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void dbs_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id dbs_ids[] = {
	
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			BIT_MASK(ABS_MT_POSITION_X) |
			BIT_MASK(ABS_MT_POSITION_Y) },
	},
	
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT |
			INPUT_DEVICE_ID_MATCH_ABSBIT,
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
		.absbit = { [BIT_WORD(ABS_X)] =
			BIT_MASK(ABS_X) | BIT_MASK(ABS_Y) },
	},
	
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};

static struct input_handler dbs_input_handler = {
	.event		= dbs_input_event,
	.connect	= dbs_input_connect,
	.disconnect	= dbs_input_disconnect,
	.name		= "cpufreq_ond",
	.id_table	= dbs_ids,
};

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				   unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int j;
	int rc;

	this_dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&dbs_mutex);

		dbs_enable++;
		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_info_s *j_dbs_info;
			j_dbs_info = &per_cpu(od_cpu_dbs_info, j);
			j_dbs_info->cur_policy = policy;

			j_dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&j_dbs_info->prev_cpu_wall);
			if (dbs_tuners_ins.ignore_nice)
				j_dbs_info->prev_cpu_nice =
						kcpustat_cpu(j).cpustat[CPUTIME_NICE];
#ifndef CONFIG_ARCH_MSM_CORTEXMP
			set_cpus_allowed(j_dbs_info->sync_thread,
					 *cpumask_of(j));
			if (!dbs_tuners_ins.powersave_bias)
				atomic_set(&j_dbs_info->sync_enabled, 1);
#endif
		}
		this_dbs_info->cpu = cpu;
		this_dbs_info->rate_mult = 1;
		ondemand_powersave_bias_init_cpu(cpu);
		if (dbs_enable == 1) {
			unsigned int latency;

			rc = sysfs_create_group(cpufreq_global_kobject,
						&dbs_attr_group);
			if (rc) {
				mutex_unlock(&dbs_mutex);
				return rc;
			}

			
			latency = policy->cpuinfo.transition_latency / 1000;
			if (latency == 0)
				latency = 1;
			
			min_sampling_rate = max(min_sampling_rate,
					MIN_LATENCY_MULTIPLIER * latency);
			dbs_tuners_ins.sampling_rate =
				max(min_sampling_rate,
				    latency * LATENCY_MULTIPLIER);

			if (dbs_tuners_ins.sampling_rate < DEF_SAMPLING_RATE)
				 dbs_tuners_ins.sampling_rate = DEF_SAMPLING_RATE;

			dbs_tuners_ins.io_is_busy = should_io_be_busy();

			if (dbs_tuners_ins.optimal_freq == 0)
				dbs_tuners_ins.optimal_freq = policy->min;

			if (dbs_tuners_ins.sync_freq == 0)
				dbs_tuners_ins.sync_freq = policy->min;

			dbs_init_freq_map_table(policy);

#ifndef CONFIG_ARCH_MSM_CORTEXMP
			atomic_notifier_chain_register(&migration_notifier_head,
					&dbs_migration_nb);
#endif
		}
		if (!cpu)
			rc = input_register_handler(&dbs_input_handler);
		mutex_unlock(&dbs_mutex);

		if (!ondemand_powersave_bias_setspeed(
					this_dbs_info->cur_policy,
					NULL,
					dbs_tuners_ins.powersave_bias))
			dbs_timer_init(this_dbs_info);
		break;

	case CPUFREQ_GOV_STOP:
		dbs_timer_exit(this_dbs_info);
		this_dbs_info->prev_load = 0;

		mutex_lock(&dbs_mutex);
		dbs_enable--;

#ifndef CONFIG_ARCH_MSM_CORTEXMP
		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_info_s *j_dbs_info;
			j_dbs_info = &per_cpu(od_cpu_dbs_info, j);
			atomic_set(&j_dbs_info->sync_enabled, 0);
		}
#endif

		this_dbs_info->cur_policy = NULL;
		if (!cpu)
			input_unregister_handler(&dbs_input_handler);
		if (!dbs_enable) {
			dbs_deinit_freq_map_table();

			sysfs_remove_group(cpufreq_global_kobject,
					   &dbs_attr_group);

#ifndef CONFIG_ARCH_MSM_CORTEXMP
			atomic_notifier_chain_unregister(
				&migration_notifier_head,
				&dbs_migration_nb);
#endif
		}

		mutex_unlock(&dbs_mutex);
		trace_cpufreq_interactive_target (cpu, 0, 0, 0, 0);
		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&this_dbs_info->timer_mutex);
		if(this_dbs_info->cur_policy) {
			if (policy->max < this_dbs_info->cur_policy->cur)
				__cpufreq_driver_target(this_dbs_info->cur_policy,
					policy->max, CPUFREQ_RELATION_H);
			else if (policy->min > this_dbs_info->cur_policy->cur)
				__cpufreq_driver_target(this_dbs_info->cur_policy,
					policy->min, CPUFREQ_RELATION_L);
			else if (dbs_tuners_ins.powersave_bias != 0)
				ondemand_powersave_bias_setspeed(
					this_dbs_info->cur_policy,
					policy,
					dbs_tuners_ins.powersave_bias);
		}
		mutex_unlock(&this_dbs_info->timer_mutex);
		break;
	}
	return 0;
}

static int cpufreq_gov_dbs_up_task(void *data)
{
	struct cpufreq_policy *policy;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int cpu = smp_processor_id();

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();

		if (kthread_should_stop())
			break;

		set_current_state(TASK_RUNNING);

		get_online_cpus();

		if (lock_policy_rwsem_write(cpu) < 0)
			goto bail_acq_sema_failed;

		this_dbs_info = &per_cpu(od_cpu_dbs_info, cpu);
		policy = this_dbs_info->cur_policy;
		if (!policy) {
			
			goto bail_incorrect_governor;
		}

		mutex_lock(&this_dbs_info->timer_mutex);

		
		dbs_tuners_ins.powersave_bias = 0;
		dbs_freq_increase(policy, this_dbs_info->prev_load, this_dbs_info->input_event_freq);
		this_dbs_info->prev_cpu_idle = get_cpu_idle_time(cpu, &this_dbs_info->prev_cpu_wall);

		mutex_unlock(&this_dbs_info->timer_mutex);

bail_incorrect_governor:
		unlock_policy_rwsem_write(cpu);

bail_acq_sema_failed:
		put_online_cpus();
	}

	return 0;
}

static int __init cpufreq_gov_dbs_init(void)
{
	u64 idle_time;
	unsigned int i;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO-1 };
	struct task_struct *pthread;
	int cpu = get_cpu();

	idle_time = get_cpu_idle_time_us(cpu, NULL);
	put_cpu();
	if (idle_time != -1ULL) {
		
		dbs_tuners_ins.up_threshold = MICRO_FREQUENCY_UP_THRESHOLD;
		dbs_tuners_ins.down_differential =
					MICRO_FREQUENCY_DOWN_DIFFERENTIAL;
		min_sampling_rate = MICRO_FREQUENCY_MIN_SAMPLE_RATE;
	} else {
		
		min_sampling_rate =
			MIN_SAMPLING_RATE_RATIO * jiffies_to_usecs(10);
	}

	dbs_wq = alloc_workqueue("ondemand_dbs_wq", WQ_HIGHPRI, 0);
	if (!dbs_wq) {
		printk(KERN_ERR "Failed to create ondemand_dbs_wq workqueue\n");
		return -EFAULT;
	}

	spin_lock_init(&input_boost_lock);

	for_each_possible_cpu(i) {
		struct cpu_dbs_info_s *this_dbs_info =
			&per_cpu(od_cpu_dbs_info, i);

#ifdef CONFIG_ARCH_MSM_CORTEXMP
		if (i == CPU0)
#endif
		{
			pthread = kthread_create_on_node(cpufreq_gov_dbs_up_task,
								NULL, cpu_to_node(i),
								"kdbs_up/%d", i);
			if (likely(!IS_ERR(pthread))) {
				kthread_bind(pthread, i);
				sched_setscheduler_nocheck(pthread, SCHED_FIFO, &param);
				get_task_struct(pthread);
				per_cpu(up_task, i) = pthread;
			}
		}

		mutex_init(&this_dbs_info->timer_mutex);

#ifndef CONFIG_ARCH_MSM_CORTEXMP
		atomic_set(&this_dbs_info->src_sync_cpu, -1);
		init_waitqueue_head(&this_dbs_info->sync_wq);

		this_dbs_info->sync_thread = kthread_run(dbs_sync_thread,
							 (void *)i,
							 "dbs_sync/%d", i);
#endif
	}

	return cpufreq_register_governor(&cpufreq_gov_ondemand);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	unsigned int i;

	cpufreq_unregister_governor(&cpufreq_gov_ondemand);
	for_each_possible_cpu(i) {
		struct cpu_dbs_info_s *this_dbs_info =
			&per_cpu(od_cpu_dbs_info, i);
#ifdef CONFIG_ARCH_MSM_CORTEXMP
		if (i == CPU0)
#endif
		if (per_cpu(up_task, i)) {
			kthread_stop(per_cpu(up_task, i));
			put_task_struct(per_cpu(up_task, i));
		}

		mutex_destroy(&this_dbs_info->timer_mutex);
#ifndef CONFIG_ARCH_MSM_CORTEXMP
		kthread_stop(this_dbs_info->sync_thread);
#endif
	}
	destroy_workqueue(dbs_wq);
}


MODULE_AUTHOR("Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>");
MODULE_AUTHOR("Alexey Starikovskiy <alexey.y.starikovskiy@intel.com>");
MODULE_DESCRIPTION("'cpufreq_ondemand' - A dynamic cpufreq governor for "
	"Low Latency Frequency Transition capable processors");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
