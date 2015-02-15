/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
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
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/types.h>
#include <linux/of.h>
#include <asm/uaccess.h>
#include <asm/arch_timer.h>
#include <mach/msm_iomap.h>
#include "rpm_stats.h"

enum {
	ID_COUNTER,
	ID_ACCUM_TIME_SCLK,
	ID_MAX,
};

static char *msm_rpmstats_id_labels[ID_MAX] = {
	[ID_COUNTER] = "Count",
	[ID_ACCUM_TIME_SCLK] = "Total time(uSec)",
};

#define SCLK_HZ 32768
#define MSM_ARCH_TIMER_FREQ 19200000

struct msm_rpmstats_record{
	char		name[32];
	uint32_t	id;
	uint32_t	val;
};
struct msm_rpmstats_private_data{
	void __iomem *reg_base;
	u32 num_records;
	u32 read_idx;
	u32 len;
	char buf[512];
	struct msm_rpmstats_platform_data *platform_data;
};

struct msm_rpm_stats_data_v2 {
	u32 stat_type;
	u32 count;
	u64 last_entered_at;
	u64 last_exited_at;
	u64 accumulated;
	u32 client_votes;
	u32 reserved[3];
};

#if defined (CONFIG_HTC_POWER_DEBUG) || (CONFIG_HTC_MONITOR_SUBSYS_SLEEP_TIME)
struct msm_rpm_stats_data_v3 {
	u32 count;
	u32 is_sleep_mode;
	u64 sleep_timestamp;
	u64 total_duration;
};

struct msm_rpm_stats_data {
	void __iomem *reg_base;
	u32 num_records;
	u32 version;
	char init;
};

enum {
	DEV_V2,
	DEV_V3,
	DEV_MAX,
};

#define DEV_V2_RECORD 2
#define DEV_V3_RECORD 4

static struct msm_rpm_stats_data rpm_stats_dev[DEV_MAX];
#endif

static inline u64 get_time_in_sec(u64 counter)
{
	do_div(counter, MSM_ARCH_TIMER_FREQ);
	return counter;
}

static inline u64 get_time_in_msec(u64 counter)
{
	do_div(counter, MSM_ARCH_TIMER_FREQ);
	counter *= MSEC_PER_SEC;
	return counter;
}

static inline int msm_rpmstats_append_data_to_buf(char *buf,
		struct msm_rpm_stats_data_v2 *data, int buflength)
{
	char stat_type[5];
	u64 time_in_last_mode;
	u64 time_since_last_mode;
	u64 actual_last_sleep;

	stat_type[4] = 0;
	memcpy(stat_type, &data->stat_type, sizeof(u32));

	time_in_last_mode = data->last_exited_at - data->last_entered_at;
	time_in_last_mode = get_time_in_msec(time_in_last_mode);
	time_since_last_mode = arch_counter_get_cntpct() - data->last_exited_at;
	time_since_last_mode = get_time_in_sec(time_since_last_mode);
	actual_last_sleep = get_time_in_msec(data->accumulated);

	return  snprintf(buf , buflength,
		"RPM Mode:%s\n\t count:%d\ntime in last mode(msec):%llu\n"
		"time since last mode(sec):%llu\nactual last sleep(msec):%llu\n"
		"client votes: %#010x\n\n",
		stat_type, data->count, time_in_last_mode,
		time_since_last_mode, actual_last_sleep,
		data->client_votes);
}

static inline u32 msm_rpmstats_read_long_register_v2(void __iomem *regbase,
		int index, int offset)
{
	return readl_relaxed(regbase + offset +
			index * sizeof(struct msm_rpm_stats_data_v2));
}

static inline u64 msm_rpmstats_read_quad_register_v2(void __iomem *regbase,
		int index, int offset)
{
	u64 dst;
	memcpy_fromio(&dst,
		regbase + offset + index * sizeof(struct msm_rpm_stats_data_v2),
		8);
	return dst;
}

static inline int msm_rpmstats_copy_stats_v2(
			struct msm_rpmstats_private_data *prvdata)
{
	void __iomem *reg;
	struct msm_rpm_stats_data_v2 data;
	int i, length;

	reg = prvdata->reg_base;

	for (i = 0, length = 0; i < prvdata->num_records; i++) {

		data.stat_type = msm_rpmstats_read_long_register_v2(reg, i,
				offsetof(struct msm_rpm_stats_data_v2,
					stat_type));
		data.count = msm_rpmstats_read_long_register_v2(reg, i,
				offsetof(struct msm_rpm_stats_data_v2, count));
		data.last_entered_at = msm_rpmstats_read_quad_register_v2(reg,
				i, offsetof(struct msm_rpm_stats_data_v2,
					last_entered_at));
		data.last_exited_at = msm_rpmstats_read_quad_register_v2(reg,
				i, offsetof(struct msm_rpm_stats_data_v2,
					last_exited_at));

		data.accumulated = msm_rpmstats_read_quad_register_v2(reg,
				i, offsetof(struct msm_rpm_stats_data_v2,
					accumulated));
		data.client_votes = msm_rpmstats_read_long_register_v2(reg,
				i, offsetof(struct msm_rpm_stats_data_v2,
					client_votes));
		length += msm_rpmstats_append_data_to_buf(prvdata->buf + length,
				&data, sizeof(prvdata->buf) - length);
		prvdata->read_idx++;
	}
	return length;
}

#if defined (CONFIG_HTC_POWER_DEBUG) || (CONFIG_HTC_MONITOR_SUBSYS_SLEEP_TIME)
static inline u32 msm_rpmstats_read_long_register_v3(void __iomem *regbase,
		int index, int offset)
{
	return readl_relaxed(regbase + offset +
			index * sizeof(struct msm_rpm_stats_data_v3));
}

static inline u64 msm_rpmstats_read_quad_register_v3(void __iomem *regbase,
		int index, int offset)
{
	u64 dst;
	memcpy_fromio(&dst,
		regbase + offset + index * sizeof(struct msm_rpm_stats_data_v3),
		8);
	return dst;
}

static inline int msm_rpmstats_append_data_to_buf_v3(char *buf,
		struct msm_rpm_stats_data_v3 *data, int buflength, int index)
{

	u64 total_time;

	total_time = data->total_duration;
	if (data->is_sleep_mode)
		total_time += (arch_counter_get_cntpct() - data->sleep_timestamp);

	total_time = get_time_in_msec(total_time);
	return  snprintf(buf , buflength,
		"sleep_info.%d (%d)\n count:%d\n total time(msec):%llu\n",
		index, data->is_sleep_mode, data->count, total_time);
}

static inline int msm_rpmstats_copy_stats_v3(
			struct msm_rpmstats_private_data *prvdata)
{
	void __iomem *reg;
	struct msm_rpm_stats_data_v3 data;
	int i, length;

	reg = prvdata->reg_base;

	for (i = 0, length = 0; i < prvdata->num_records; i++) {

		data.is_sleep_mode = msm_rpmstats_read_long_register_v3(reg, i,
				offsetof(struct msm_rpm_stats_data_v3,
					is_sleep_mode));
		data.count = msm_rpmstats_read_long_register_v3(reg, i,
				offsetof(struct msm_rpm_stats_data_v3, count));
		data.sleep_timestamp = msm_rpmstats_read_quad_register_v3(reg,
				i, offsetof(struct msm_rpm_stats_data_v3,
					sleep_timestamp));
		data.total_duration = msm_rpmstats_read_quad_register_v3(reg,
				i, offsetof(struct msm_rpm_stats_data_v3,
					total_duration));

		length += msm_rpmstats_append_data_to_buf_v3(prvdata->buf + length,
				&data, sizeof(prvdata->buf) - length, i);
		prvdata->read_idx++;
	}
	return length;
}

#ifdef CONFIG_HTC_MONITOR_SUBSYS_SLEEP_TIME

struct trackStick {
	u64 prev_duration;
	struct timeval start_time;
};

static struct trackStick aCPU;
static struct trackStick wlanFW;

void rpm_resetSubsysStickTime(int n)
{
	switch (n)
	{
	case 0:
		memset(&aCPU, 0, sizeof(struct trackStick));
		break;
#ifdef CONFIG_ARCH_MSM8916
	case 2:
#endif
	case 3:
		memset(&wlanFW, 0, sizeof(struct trackStick));
		break;
	default:
		pr_info("invalid number when rpm_resetSubsysStickTime");
		break;
	}
}
EXPORT_SYMBOL(rpm_resetSubsysStickTime);

int rpm_getSubsysStickTime(int n, struct timeval *cur_t)
{
	void __iomem *reg;
	struct msm_rpm_stats_data_v3 data_v3;
	int ret = 0;

	if (rpm_stats_dev[DEV_V3].init) {
		reg = rpm_stats_dev[DEV_V3].reg_base;

		data_v3.total_duration = msm_rpmstats_read_quad_register_v3(reg,
			n, offsetof(struct msm_rpm_stats_data_v3, total_duration));

		switch (n)
		{
		case 0:
			if (aCPU.prev_duration == get_time_in_msec(data_v3.total_duration)) {
				if (!aCPU.start_time.tv_sec) {
					do_gettimeofday(&aCPU.start_time);
				} else {
					if (aCPU.start_time.tv_sec > 0 &&
						cur_t->tv_sec > aCPU.start_time.tv_sec)
						return cur_t->tv_sec - aCPU.start_time.tv_sec;
				}
			} else {
				aCPU.prev_duration = get_time_in_msec(data_v3.total_duration);
				if (aCPU.start_time.tv_sec)
					memset(&aCPU, 0, sizeof(struct trackStick));
			}
			break;
#ifdef CONFIG_ARCH_MSM8916
		case 2:
#endif
		case 3:
			if (wlanFW.prev_duration == get_time_in_msec(data_v3.total_duration)) {
				if (!wlanFW.start_time.tv_sec) {
					do_gettimeofday(&wlanFW.start_time);
				} else {
					if (wlanFW.start_time.tv_sec > 0 &&
						cur_t->tv_sec > wlanFW.start_time.tv_sec)
						return cur_t->tv_sec - wlanFW.start_time.tv_sec;
				}
			} else {
				wlanFW.prev_duration = get_time_in_msec(data_v3.total_duration);
				if (wlanFW.start_time.tv_sec)
					memset(&wlanFW, 0, sizeof(struct trackStick));
			}
			break;
		default:
			pr_info("invalid number when getSubsysStickTime");
			ret = -1;
			break;
		}
	}
	return ret;
}
EXPORT_SYMBOL(rpm_getSubsysStickTime);
#endif 

void msm_rpm_dump_stat(void)
{
	void __iomem *reg;
	struct msm_rpm_stats_data_v3 data_v3;

	int i;

	if (rpm_stats_dev[DEV_V2].init) {
		reg = rpm_stats_dev[DEV_V2].reg_base;
		pr_info("%s: %u, %llums, %u, %llums\n", __func__,
			msm_rpmstats_read_long_register_v2(reg, 0, offsetof(struct msm_rpm_stats_data_v2, count)),
			get_time_in_msec(msm_rpmstats_read_quad_register_v2(reg, 0,
							offsetof(struct msm_rpm_stats_data_v2, accumulated))),
			msm_rpmstats_read_long_register_v2(reg, 1, offsetof(struct msm_rpm_stats_data_v2, count)),
			get_time_in_msec(msm_rpmstats_read_quad_register_v2(reg, 1,
							offsetof(struct msm_rpm_stats_data_v2, accumulated))));
	}

	if (rpm_stats_dev[DEV_V3].init) {
		reg = rpm_stats_dev[DEV_V3].reg_base;
		for (i = 0; i < rpm_stats_dev[DEV_V3].num_records; i++) {
			data_v3.is_sleep_mode = msm_rpmstats_read_long_register_v3(reg, i,
				offsetof(struct msm_rpm_stats_data_v3, is_sleep_mode));
			data_v3.count = msm_rpmstats_read_long_register_v3(reg, i,
				offsetof(struct msm_rpm_stats_data_v3, count));
			data_v3.sleep_timestamp = msm_rpmstats_read_quad_register_v3(reg,
				i, offsetof(struct msm_rpm_stats_data_v3, sleep_timestamp));
			data_v3.total_duration = msm_rpmstats_read_quad_register_v3(reg,
				i, offsetof(struct msm_rpm_stats_data_v3, total_duration));

			if (data_v3.is_sleep_mode)
				data_v3.total_duration += (arch_counter_get_cntpct() - data_v3.sleep_timestamp);

			pr_info("[K] sleep_info_m.%d - %u (%d), %llums\n", i, data_v3.count,
									data_v3.is_sleep_mode,
									get_time_in_msec(data_v3.total_duration));
		}
	}
}

int htc_get_xo_vddmin_info(uint32_t *xo_count, uint64_t *xo_time, uint32_t *vddmin_count, uint64_t *vddmin_time)
{
	void __iomem *reg;

	if (!rpm_stats_dev[DEV_V2].init)
		return 0;

	reg = rpm_stats_dev[DEV_V2].reg_base;
	*xo_count = msm_rpmstats_read_long_register_v2(reg, 0,
					offsetof(struct msm_rpm_stats_data_v2, count));
	*xo_time = get_time_in_msec(msm_rpmstats_read_quad_register_v2(reg, 0,
					offsetof(struct msm_rpm_stats_data_v2, accumulated)));
	*vddmin_count = msm_rpmstats_read_long_register_v2(reg, 1,
					offsetof(struct msm_rpm_stats_data_v2, count));
	*vddmin_time = get_time_in_msec(msm_rpmstats_read_quad_register_v2(reg, 1,
					offsetof(struct msm_rpm_stats_data_v2, accumulated)));

	return 1;
}
#endif

static inline unsigned long  msm_rpmstats_read_register(void __iomem *regbase,
		int index, int offset)
{
	return  readl_relaxed(regbase + index * 12 + (offset + 1) * 4);
}
static void msm_rpmstats_strcpy(char *dest, char  *src)
{
	union {
		char ch[4];
		unsigned long word;
	} string;
	int index = 0;

	do  {
		int i;
		string.word = readl_relaxed(src + 4 * index);
		for (i = 0; i < 4; i++) {
			*dest++ = string.ch[i];
			if (!string.ch[i])
				break;
		}
		index++;
	} while (*(dest-1));

}
static int msm_rpmstats_copy_stats(struct msm_rpmstats_private_data *pdata)
{

	struct msm_rpmstats_record record;
	unsigned long ptr;
	unsigned long offset;
	char *str;
	uint64_t usec;

	ptr = msm_rpmstats_read_register(pdata->reg_base, pdata->read_idx, 0);
	offset = (ptr - (unsigned long)pdata->platform_data->phys_addr_base);

	if (offset > pdata->platform_data->phys_size)
		str = (char *)ioremap(ptr, SZ_256);
	else
		str = (char *) pdata->reg_base + offset;

	msm_rpmstats_strcpy(record.name, str);

	if (offset > pdata->platform_data->phys_size)
		iounmap(str);

	record.id = msm_rpmstats_read_register(pdata->reg_base,
						pdata->read_idx, 1);
	record.val = msm_rpmstats_read_register(pdata->reg_base,
						pdata->read_idx, 2);

	if (record.id == ID_ACCUM_TIME_SCLK) {
		usec = record.val * USEC_PER_SEC;
		do_div(usec, SCLK_HZ);
	}  else
		usec = (unsigned long)record.val;

	pdata->read_idx++;

	return snprintf(pdata->buf, sizeof(pdata->buf),
			"RPM Mode:%s\n\t%s:%llu\n",
			record.name,
			msm_rpmstats_id_labels[record.id],
			usec);
}

static int msm_rpmstats_file_read(struct file *file, char __user *bufu,
				  size_t count, loff_t *ppos)
{
	struct msm_rpmstats_private_data *prvdata;
	prvdata = file->private_data;

	if (!prvdata)
		return -EINVAL;

	if (!bufu || count == 0)
		return -EINVAL;

	if (prvdata->platform_data->version == 1) {
		if (!prvdata->num_records)
			prvdata->num_records = readl_relaxed(prvdata->reg_base);
	}

	if ((*ppos >= prvdata->len)
		&& (prvdata->read_idx < prvdata->num_records)) {
			if (prvdata->platform_data->version == 1)
				prvdata->len = msm_rpmstats_copy_stats(prvdata);
			else if (prvdata->platform_data->version == 2)
				prvdata->len = msm_rpmstats_copy_stats_v2(
						prvdata);
#if defined (CONFIG_HTC_POWER_DEBUG) || (CONFIG_HTC_MONITOR_SUBSYS_SLEEP_TIME)
			else if (prvdata->platform_data->version == 3)
				prvdata->len = msm_rpmstats_copy_stats_v3(
						prvdata);
#endif
			*ppos = 0;
	}

	return simple_read_from_buffer(bufu, count, ppos,
			prvdata->buf, prvdata->len);
}

static int msm_rpmstats_file_open(struct inode *inode, struct file *file)
{
	struct msm_rpmstats_private_data *prvdata;
	struct msm_rpmstats_platform_data *pdata;

	pdata = inode->i_private;

	file->private_data =
		kmalloc(sizeof(struct msm_rpmstats_private_data), GFP_KERNEL);

	if (!file->private_data)
		return -ENOMEM;
	prvdata = file->private_data;

	prvdata->reg_base = ioremap_nocache(pdata->phys_addr_base,
					pdata->phys_size);
	if (!prvdata->reg_base) {
		kfree(file->private_data);
		prvdata = NULL;
		pr_err("%s: ERROR could not ioremap start=%p, len=%u\n",
			__func__, (void *)pdata->phys_addr_base,
			pdata->phys_size);
		return -EBUSY;
	}

	prvdata->read_idx = prvdata->num_records =  prvdata->len = 0;
	prvdata->platform_data = pdata;
#if defined (CONFIG_HTC_POWER_DEBUG) || (CONFIG_HTC_MONITOR_SUBSYS_SLEEP_TIME)
	if (pdata->version == 2)
		prvdata->num_records = DEV_V2_RECORD;
	else if (pdata->version == 3)
		prvdata->num_records = DEV_V3_RECORD;
#else
	if (pdata->version == 2)
		prvdata->num_records = 2;
#endif
	return 0;
}

static int msm_rpmstats_file_close(struct inode *inode, struct file *file)
{
	struct msm_rpmstats_private_data *private = file->private_data;

	if (private->reg_base)
		iounmap(private->reg_base);
	kfree(file->private_data);

	return 0;
}

static const struct file_operations msm_rpmstats_fops = {
	.owner	  = THIS_MODULE,
	.open	  = msm_rpmstats_file_open,
	.read	  = msm_rpmstats_file_read,
	.release  = msm_rpmstats_file_close,
	.llseek   = no_llseek,
};

static  int __devinit msm_rpmstats_probe(struct platform_device *pdev)
{
	struct dentry *dent = NULL;
	struct msm_rpmstats_platform_data *pdata;
	struct msm_rpmstats_platform_data *pd;
	struct resource *res = NULL;
	struct device_node *node = NULL;
	int ret = 0;

	if (!pdev)
		return -EINVAL;

	pdata = kzalloc(sizeof(struct msm_rpmstats_platform_data), GFP_KERNEL);

	if (!pdata)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!res)
		return -EINVAL;

	pdata->phys_addr_base  = res->start;

	pdata->phys_size = resource_size(res);
	node = pdev->dev.of_node;
	if (pdev->dev.platform_data) {
		pd = pdev->dev.platform_data;
		pdata->version = pd->version;

	} else if (node)
		ret = of_property_read_u32(node,
			"qcom,sleep-stats-version", &pdata->version);

	if (!ret) {
#if defined (CONFIG_HTC_POWER_DEBUG) || (CONFIG_HTC_MONITOR_SUBSYS_SLEEP_TIME)
		if (pdata->version == 2) {
			dent = debugfs_create_file("rpm_stats", S_IRUGO, NULL,
					pdata, &msm_rpmstats_fops);

			if (!dent) {
				pr_err("%s: ERROR debugfs_create_file failed\n",
						__func__);
				kfree(pdata);
				return -ENOMEM;
			}

			if (!rpm_stats_dev[DEV_V2].init) {
				rpm_stats_dev[DEV_V2].reg_base = ioremap_nocache(pdata->phys_addr_base,
										pdata->phys_size);
				rpm_stats_dev[DEV_V2].init = 1;
				rpm_stats_dev[DEV_V2].num_records = DEV_V2_RECORD;
			}
		} else if (pdata->version == 3) {
			dent = debugfs_create_file("sleep_stats", S_IRUGO, NULL,
						pdata, &msm_rpmstats_fops);
			if (!dent) {
				pr_err("%s: ERROR debugfs_create_file failed\n", __func__);
				kfree(pdata);
				return -ENOMEM;
			}
			if (!rpm_stats_dev[DEV_V3].init) {
				rpm_stats_dev[DEV_V3].reg_base = ioremap_nocache(pdata->phys_addr_base,
										pdata->phys_size);
				rpm_stats_dev[DEV_V3].init = 1;
				rpm_stats_dev[DEV_V3].num_records = DEV_V3_RECORD;
			}
		}
#else
		dent = debugfs_create_file("rpm_stats", S_IRUGO, NULL,
				pdata, &msm_rpmstats_fops);

		if (!dent) {
			pr_err("%s: ERROR debugfs_create_file failed\n",
					__func__);
			kfree(pdata);
			return -ENOMEM;
		}
#endif
	} else {
		kfree(pdata);
		return -EINVAL;
	}
	platform_set_drvdata(pdev, dent);
	return 0;
}

static int __devexit msm_rpmstats_remove(struct platform_device *pdev)
{
	struct dentry *dent;

	dent = platform_get_drvdata(pdev);
	debugfs_remove(dent);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct of_device_id rpm_stats_table[] = {
	       {.compatible = "qcom,rpm-stats"},
	       {},
};

static struct platform_driver msm_rpmstats_driver = {
	.probe	= msm_rpmstats_probe,
	.remove = __devexit_p(msm_rpmstats_remove),
	.driver = {
		.name = "msm_rpm_stat",
		.owner = THIS_MODULE,
		.of_match_table = rpm_stats_table,
	},
};
static int __init msm_rpmstats_init(void)
{
#if defined (CONFIG_HTC_POWER_DEBUG) || (CONFIG_HTC_MONITOR_SUBSYS_SLEEP_TIME)
	memset(rpm_stats_dev, 0, sizeof(struct msm_rpm_stats_data)*DEV_MAX);
#endif

#ifdef CONFIG_HTC_MONITOR_SUBSYS_SLEEP_TIME
	memset(&aCPU, 0, sizeof(struct trackStick));
	memset(&wlanFW, 0, sizeof(struct trackStick));
#endif

	return platform_driver_register(&msm_rpmstats_driver);
}
static void __exit msm_rpmstats_exit(void)
{
	platform_driver_unregister(&msm_rpmstats_driver);
}
module_init(msm_rpmstats_init);
module_exit(msm_rpmstats_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM RPM Statistics driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:msm_stat_log");
