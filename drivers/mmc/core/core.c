/*
 *  linux/drivers/mmc/core/core.c
 *
 *  Copyright (C) 2003-2004 Russell King, All Rights Reserved.
 *  SD support Copyright (C) 2004 Ian Molton, All Rights Reserved.
 *  Copyright (C) 2005-2008 Pierre Ossman, All Rights Reserved.
 *  MMCv4 support Copyright (C) 2006 Philip Langdale, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/pagemap.h>
#include <linux/err.h>
#include <linux/leds.h>
#include <linux/scatterlist.h>
#include <linux/log2.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/suspend.h>
#include <linux/fault-inject.h>
#include <linux/random.h>
#include <linux/wakelock.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/statfs.h>
#include <linux/debugfs.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/list_sort.h>

#include "core.h"
#include "bus.h"
#include "host.h"
#include "sdio_bus.h"

#include "mmc_ops.h"
#include "sd_ops.h"
#include "sdio_ops.h"

#define CREATE_TRACE_POINTS
#include <trace/events/mmc.h>
#include <trace/events/mmcio.h>
#include <trace/events/f2fs.h>

static void mmc_clk_scaling(struct mmc_host *host, bool from_wq);

#define MMC_CORE_TIMEOUT_MS	(10 * 60 * 1000) 

#define MMC_BKOPS_MAX_TIMEOUT	(30 * 1000) 

#define MMC_FLUSH_REQ_TIMEOUT_MS 90000 
#define MMC_CACHE_DISBALE_TIMEOUT_MS 180000 

#define MMC_WORKLOAD_DURATION (60 * 60 * 1000) 

#define SD_REMOVE_DEBUG		1
#if SD_REMOVE_DEBUG
char last_claim_comm[16];
ktime_t last_claim_getime;
char last_release_comm[16];
ktime_t last_release_getime;
char last_claim_code_stack1[64];
char last_claim_code_stack2[64];
char last_claim_code_stack3[64];
char last_release_code_stack1[64];
char last_release_code_stack2[64];
char last_release_code_stack3[64];
char *claim_stack[3] = {last_claim_code_stack1, last_claim_code_stack2, last_claim_code_stack3};
char *release_stack[3] = {last_release_code_stack1, last_release_code_stack2, last_release_code_stack3};
struct stackframe {
unsigned long fp;
unsigned long sp;
unsigned long lr;
unsigned long pc;
};
extern int unwind_frame(struct stackframe *frame);
extern void dump_backtrace_entry(unsigned long where, unsigned long from, unsigned long frame);
struct timer_list sd_remove_tout_timer;
#endif

static struct workqueue_struct *workqueue;
struct workqueue_struct *stats_workqueue = NULL;
static struct wake_lock mmc_removal_work_wake_lock;

extern unsigned int get_tamper_sf(void);

extern int powersave_enabled;
extern bool ac_status;

enum {
    PP_NORMAL = 0,
    PP_POWERSAVE = 1,
    PP_EXTREMELY_POWERSAVE = 2,
    PP_PERFORMANCE = 4,
};

bool use_spi_crc = 1;
module_param(use_spi_crc, bool, 0);

#ifdef CONFIG_MMC_UNSAFE_RESUME
bool mmc_assume_removable;
#else
bool mmc_assume_removable = 1;
#endif
EXPORT_SYMBOL(mmc_assume_removable);
module_param_named(removable, mmc_assume_removable, bool, 0644);
MODULE_PARM_DESC(
	removable,
	"MMC/SD cards are removable and may be removed during suspend");

#define MMC_UPDATE_BKOPS_STATS_HPI(stats)	\
	do {					\
		spin_lock(&stats.lock);		\
		if (stats.enabled)		\
			stats.hpi++;		\
		spin_unlock(&stats.lock);	\
	} while (0);
#define MMC_UPDATE_BKOPS_STATS_SUSPEND(stats)	\
	do {					\
		spin_lock(&stats.lock);		\
		if (stats.enabled)		\
			stats.suspend++;	\
		spin_unlock(&stats.lock);	\
	} while (0);
#define MMC_UPDATE_STATS_BKOPS_SEVERITY_LEVEL(stats, level)		\
	do {								\
		if (level <= 0 || level > BKOPS_NUM_OF_SEVERITY_LEVELS)	\
			break;						\
		spin_lock(&stats.lock);					\
		if (stats.enabled)					\
			stats.bkops_level[level-1]++;			\
		spin_unlock(&stats.lock);				\
	} while (0);

#define IOTOP_INTERVAL	9500
#define IOREAD_DUMP_THRESHOLD	10485760 
#define IOWRITE_DUMP_THRESHOLD	1048576 
#define IOREAD_DUMP_TOTAL_THRESHOLD		10485760 
#define IOWRITE_DUMP_TOTAL_THRESHOLD	10485760 
struct io_account {
	char task_name[TASK_COMM_LEN];
	unsigned int pid;
	u64 io_amount;
	struct list_head list;
};
static LIST_HEAD(ioread_list);
static LIST_HEAD(iowrite_list);
static spinlock_t iolist_lock;
static unsigned long jiffies_next_iotop = 0;
static int iotop_cmp(void *priv, struct list_head *a, struct list_head *b)
{
	struct io_account *ioa = container_of(a, struct io_account, list);
	struct io_account *iob = container_of(b, struct io_account, list);

	return !(ioa->io_amount > iob->io_amount);
}

void collect_io_stats(size_t rw_bytes, int type)
{
	struct task_struct *process = current;
	struct io_account *io_act, *tmp;
	int found;
	struct list_head *io_list;
	unsigned long flags;

	if (get_tamper_sf() == 1)
		return;

	if (!rw_bytes)
		return;

	if (type == READ)
		io_list = &ioread_list;
	else if (type == WRITE)
		io_list = &iowrite_list;
	else
		return;

	found = 0;
	spin_lock_irqsave(&iolist_lock, flags);
	list_for_each_entry_safe(io_act, tmp, io_list, list) {
		if (!strcmp(process->comm, io_act->task_name)) {
			io_act->io_amount += rw_bytes;
			io_act->pid = process->pid;
			found = 1;
			break;
		}
	}
	spin_unlock_irqrestore(&iolist_lock, flags);

	if (!found) {
		io_act = kmalloc(sizeof(struct io_account), GFP_ATOMIC);
		if (io_act) {
			snprintf(io_act->task_name, sizeof(io_act->task_name), "%s", process->comm);
			io_act->pid = process->pid;
			io_act->io_amount = rw_bytes;
			spin_lock_irqsave(&iolist_lock, flags);
			list_add_tail(&io_act->list, io_list);
			spin_unlock_irqrestore(&iolist_lock, flags);
		}
	}
}

static void show_iotop(void)
{
	struct io_account *io_act, *tmp;
	int i = 0;
	unsigned int task_cnt = 0;
	unsigned long long total_bytes;
	unsigned long flags;

	if (get_tamper_sf() == 1)
		return;

	spin_lock_irqsave(&iolist_lock, flags);
	list_sort(NULL, &ioread_list, iotop_cmp);
	total_bytes = 0;
	list_for_each_entry_safe(io_act, tmp, &ioread_list, list) {
		list_del_init(&io_act->list);
		if (i++ < 5 && io_act->io_amount > IOREAD_DUMP_THRESHOLD)
			pr_info("[READ IOTOP%d] %s(%u): %llu KB\n", i, io_act->task_name,
				io_act->pid, io_act->io_amount / 1024);
		kfree(io_act);
		task_cnt++;
		total_bytes += io_act->io_amount;
	}
	if (total_bytes > IOREAD_DUMP_TOTAL_THRESHOLD)
		pr_info("[IOTOP] READ total %u tasks, %llu KB\n", task_cnt, total_bytes / 1024);

	list_sort(NULL, &iowrite_list, iotop_cmp);
	i = 0;
	total_bytes = 0;
	task_cnt = 0;
	list_for_each_entry_safe(io_act, tmp, &iowrite_list, list) {
		list_del_init(&io_act->list);
		if (i++ < 5 && io_act->io_amount >= IOWRITE_DUMP_THRESHOLD)
			pr_info("[WRITE IOTOP%d] %s(%u): %llu KB\n", i, io_act->task_name,
				io_act->pid, io_act->io_amount / 1024);
		kfree(io_act);
		task_cnt++;
		total_bytes += io_act->io_amount;
	}
	spin_unlock_irqrestore(&iolist_lock, flags);
	if (total_bytes > IOWRITE_DUMP_TOTAL_THRESHOLD)
		pr_info("[IOTOP] WRITE total %u tasks, %llu KB\n", task_cnt, total_bytes / 1024);
}

static int stats_interval = MMC_STATS_INTERVAL;
#define K(x) ((x) << (PAGE_SHIFT - 10))
void mmc_stats(struct work_struct *work)
{
	struct mmc_host *host =
		container_of(work, struct mmc_host, stats_work.work);
	unsigned long rtime, wtime;
	unsigned long rbytes, wbytes, rcnt, wcnt;
	unsigned long wperf = 0, rperf = 0;
	unsigned long flags;
	u64 val;
	struct kstatfs stat;
	unsigned long free = 0;
	int reset_low_perf_data = 1;
	
	unsigned long rtime_rand = 0, wtime_rand = 0;
	unsigned long rbytes_rand = 0, wbytes_rand = 0, rcnt_rand = 0, wcnt_rand = 0;
	unsigned long wperf_rand = 0, rperf_rand = 0;
	
	unsigned long erase_time; 
	unsigned long erase_blks;
	unsigned long erase_rq;
	
	ktime_t workload_diff;
	unsigned long workload_time; 

	if (!host || !host->perf_enable || !stats_workqueue)
		return;

	spin_lock_irqsave(&host->lock, flags);

	rbytes = host->perf.rbytes_drv;
	wbytes = host->perf.wbytes_drv;
	rcnt = host->perf.rcount;
	wcnt = host->perf.wcount;
	rtime = (unsigned long)ktime_to_us(host->perf.rtime_drv);
	wtime = (unsigned long)ktime_to_us(host->perf.wtime_drv);
	host->perf.rbytes_drv = host->perf.wbytes_drv = 0;
	host->perf.rcount = host->perf.wcount = 0;
	host->perf.rtime_drv = ktime_set(0, 0);
	host->perf.wtime_drv = ktime_set(0, 0);

	
	erase_time = (unsigned long)ktime_to_ms(host->perf.erase_time);
	erase_blks = host->perf.erase_blks;
	erase_rq = host->perf.erase_rq;
	host->perf.erase_blks = 0;
	host->perf.erase_rq = 0;
	host->perf.erase_time = ktime_set(0, 0);

	
	if (host->debug_mask & MMC_DEBUG_RANDOM_RW) {
		rbytes_rand = host->perf.rbytes_drv_rand;
		wbytes_rand = host->perf.wbytes_drv_rand;
		rcnt_rand = host->perf.rcount_rand;
		wcnt_rand = host->perf.wcount_rand;
		rtime_rand = (unsigned long)ktime_to_us(host->perf.rtime_drv_rand);
		wtime_rand = (unsigned long)ktime_to_us(host->perf.wtime_drv_rand);

		host->perf.rbytes_drv_rand = host->perf.wbytes_drv_rand = 0;
		host->perf.rcount_rand = host->perf.wcount_rand = 0;
		host->perf.rtime_drv_rand = ktime_set(0, 0);
		host->perf.wtime_drv_rand = ktime_set(0, 0);

	}
	

	spin_unlock_irqrestore(&host->lock, flags);

	if (wtime) {
		val = ((u64)wbytes / 1024) * 1000000;
		do_div(val, wtime);
		wperf = (unsigned long)val;
	}
	if (rtime) {
		val = ((u64)rbytes / 1024) * 1000000;
		do_div(val, rtime);
		rperf = (unsigned long)val;
	}

	if (host->debug_mask & MMC_DEBUG_FREE_SPACE) {
		struct file *file;
		file = filp_open("/data", O_RDONLY, 0);
		if (!IS_ERR(file)) {
			vfs_statfs(&file->f_path, &stat);
			filp_close(file, NULL);
			free = (unsigned long)stat.f_bfree;
			free /= 256; 
		}
	}

	
	wtime /= 1000;
	rtime /= 1000;

	
	if (!wtime)
		reset_low_perf_data = 0;
	else if (wperf < 100) {
		host->perf.lp_duration += stats_interval; 
		host->perf.wbytes_low_perf += wbytes;
		host->perf.wtime_low_perf += wtime; 
		if (host->perf.lp_duration >= 600000) {
			unsigned long perf;
			val = ((u64)host->perf.wbytes_low_perf / 1024) * 1000;
			do_div(val, host->perf.wtime_low_perf);
			perf = (unsigned long)val;
			pr_err("%s Statistics: write %lu KB in %lu ms, perf %lu KB/s, duration %lu sec\n",
					mmc_hostname(host), host->perf.wbytes_low_perf / 1024,
					host->perf.wtime_low_perf,
					perf, host->perf.lp_duration / 1000);
		} else
			reset_low_perf_data = 0;
	}
	if (host->perf.lp_duration && reset_low_perf_data) {
		host->perf.lp_duration = 0;
		host->perf.wbytes_low_perf = 0;
		host->perf.wtime_low_perf = 0;
	}

	
	if ((wtime > 500) || (wtime && (stats_interval == MMC_STATS_LOG_INTERVAL)))  {
		#if 0
		pr_info("%s Statistics: dirty %luKB, writeback %luKB\n", mmc_hostname(host),
				K(global_page_state(NR_FILE_DIRTY)), K(global_page_state(NR_WRITEBACK)));
		#endif
		if (host->debug_mask & MMC_DEBUG_FREE_SPACE)
			pr_info("%s Statistics: write %lu KB in %lu ms, perf %lu KB/s, rq %lu, /data free %lu MB\n",
					mmc_hostname(host), wbytes / 1024, wtime, wperf, wcnt, free);
		else
			pr_info("%s Statistics: write %lu KB in %lu ms, perf %lu KB/s, rq %lu\n",
					mmc_hostname(host), wbytes / 1024, wtime, wperf, wcnt);

		if (rtime) {
			if (host->debug_mask & MMC_DEBUG_FREE_SPACE)
				pr_info("%s Statistics: read %lu KB in %lu ms, perf %lu KB/s, rq %lu, /data free %lu MB\n",
						mmc_hostname(host), rbytes / 1024, rtime, rperf, rcnt, free);
			else
				pr_info("%s Statistics: read %lu KB in %lu ms, perf %lu KB/s, rq %lu\n",
						mmc_hostname(host), rbytes / 1024, rtime, rperf, rcnt);
		}
	}
	else if ((rtime > 500) || (rtime && (stats_interval == MMC_STATS_LOG_INTERVAL)))  {
		if (host->debug_mask & MMC_DEBUG_FREE_SPACE)
			pr_info("%s Statistics: read %lu KB in %lu ms, perf %lu KB/s, rq %lu, /data free %lu MB\n",
					mmc_hostname(host), rbytes / 1024, rtime, rperf, rcnt, free);
		else
			pr_info("%s Statistics: read %lu KB in %lu ms, perf %lu KB/s, rq %lu\n",
					mmc_hostname(host), rbytes / 1024, rtime, rperf, rcnt);
	}

	
	workload_diff = ktime_sub(ktime_get(), host->perf.workload_time);
        workload_time = (unsigned long)ktime_to_ms(workload_diff);
	if (workload_time >= MMC_WORKLOAD_DURATION) {
		pr_info("%s Statistics: workload write %lu KB (%lu MB)\n",
				mmc_hostname(host),
				host->perf.wkbytes_drv, host->perf.wkbytes_drv / 1024);
		host->perf.wkbytes_drv = 0;
		host->perf.workload_time = ktime_get();
	}

	
	if (erase_time > 500)
		pr_info("%s Statistics: erase %lu blocks in %lu ms, rq %lu\n",
			mmc_hostname(host), erase_blks, erase_time, erase_rq);

	if (!jiffies_next_iotop || time_after(jiffies, jiffies_next_iotop)) {
		jiffies_next_iotop = jiffies + msecs_to_jiffies(IOTOP_INTERVAL);
		show_iotop();
	}
	
	if (host->debug_mask & MMC_DEBUG_RANDOM_RW) {
		if (wtime_rand) {
			val = ((u64)wbytes_rand / 1024) * 1000000;
			do_div(val, wtime_rand);
			wperf_rand = (unsigned long)val;
		}
		if (rtime_rand) {
			val = ((u64)rbytes_rand / 1024) * 1000000;
			do_div(val, rtime_rand);
			rperf_rand = (unsigned long)val;
		}
		wtime_rand /= 1000;
		rtime_rand /= 1000;
		if (wperf_rand && wtime_rand) {
			if (host->debug_mask & MMC_DEBUG_FREE_SPACE)
				pr_info("%s Statistics: random write %lu KB in %lu ms, perf %lu KB/s, rq %lu, /data free %lu MB\n",
						mmc_hostname(host), wbytes_rand / 1024, wtime_rand, wperf_rand, wcnt_rand, free);
			else
				pr_info("%s Statistics: random write %lu KB in %lu ms, perf %lu KB/s, rq %lu\n",
						mmc_hostname(host), wbytes_rand / 1024, wtime_rand, wperf_rand, wcnt_rand);
		}
		if (rperf_rand && rtime_rand) {
			if (host->debug_mask & MMC_DEBUG_FREE_SPACE)
				pr_info("%s Statistics: random read %lu KB in %lu ms, perf %lu KB/s, rq %lu, /data free %lu MB\n",
						mmc_hostname(host), rbytes_rand / 1024, rtime_rand, rperf_rand, rcnt_rand, free);
			else
				pr_info("%s Statistics: random read %lu KB in %lu ms, perf %lu KB/s, rq %lu\n",
						mmc_hostname(host), rbytes_rand / 1024, rtime_rand, rperf_rand, rcnt_rand);
		}
	}
	

	if (host->debug_mask & MMC_DEBUG_MEMORY) {
		struct sysinfo mi;
		long cached;
		si_meminfo(&mi);
		cached = global_page_state(NR_FILE_PAGES) -
			total_swapcache_pages - mi.bufferram;
		pr_info("meminfo: total %lu KB, free %lu KB, buffers %lu KB, cached %lu KB \n",
				K(mi.totalram),
				K(mi.freeram),
				K(mi.bufferram),
				K(cached));
	}

	queue_delayed_work(stats_workqueue, &host->stats_work, msecs_to_jiffies(stats_interval));
	return;
}

int mmc_schedule_card_removal_work(struct delayed_work *work,
                                    unsigned long delay)
{
	wake_lock(&mmc_removal_work_wake_lock);
	return queue_delayed_work(workqueue, work, delay);
}

static int mmc_schedule_delayed_work(struct delayed_work *work,
				     unsigned long delay)
{
	return queue_delayed_work(workqueue, work, delay);
}

static void mmc_flush_scheduled_work(void)
{
	flush_workqueue(workqueue);
}

#ifdef CONFIG_FAIL_MMC_REQUEST

static void mmc_should_fail_request(struct mmc_host *host,
				    struct mmc_request *mrq)
{
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	static const int data_errors[] = {
		-ETIMEDOUT,
		-EILSEQ,
		-EIO,
	};

	if (!data)
		return;

	if (cmd->error || data->error ||
	    !should_fail(&host->fail_mmc_request, data->blksz * data->blocks))
		return;

	data->error = data_errors[random32() % ARRAY_SIZE(data_errors)];
	data->bytes_xfered = (random32() % (data->bytes_xfered >> 9)) << 9;
	data->fault_injected = true;
}

#else 

static inline void mmc_should_fail_request(struct mmc_host *host,
					   struct mmc_request *mrq)
{
}

#endif 

static inline void
mmc_clk_scaling_update_state(struct mmc_host *host, struct mmc_request *mrq)
{
	if (mrq) {
		switch (mrq->cmd->opcode) {
		case MMC_READ_SINGLE_BLOCK:
		case MMC_READ_MULTIPLE_BLOCK:
		case MMC_WRITE_BLOCK:
		case MMC_WRITE_MULTIPLE_BLOCK:
			host->clk_scaling.invalid_state = false;
			break;
		default:
			host->clk_scaling.invalid_state = true;
			break;
		}
	} else {
		host->clk_scaling.invalid_state = false;
	}

	return;
}

static inline void mmc_update_clk_scaling(struct mmc_host *host)
{
	if (host->clk_scaling.enable && !host->clk_scaling.invalid_state) {
		host->clk_scaling.busy_time_us +=
			ktime_to_us(ktime_sub(ktime_get(),
					host->clk_scaling.start_busy));
		host->clk_scaling.start_busy = ktime_get();
	}
}
void mmc_request_done(struct mmc_host *host, struct mmc_request *mrq)
{
	struct mmc_command *cmd = mrq->cmd;
	int err = cmd->error;
	ktime_t diff;
	if (host->card)
		mmc_update_clk_scaling(host);

	if (err && cmd->retries && mmc_host_is_spi(host)) {
		if (cmd->resp[0] & R1_SPI_ILLEGAL_COMMAND)
			cmd->retries = 0;
	}

	if (err && cmd->retries && !mmc_card_removed(host->card)) {
		if (mrq->done)
			mrq->done(mrq);
	} else {
		mmc_should_fail_request(host, mrq);

		

		pr_debug("%s: req done (CMD%u): %d: %08x %08x %08x %08x\n",
			mmc_hostname(host), cmd->opcode, err,
			cmd->resp[0], cmd->resp[1],
			cmd->resp[2], cmd->resp[3]);

		if (mrq->data) {
			if (host->perf_enable && cmd->opcode != MMC_SEND_EXT_CSD) {
				unsigned long flags;
				diff = ktime_sub(ktime_get(), host->perf.start);
				if (host->card && mmc_card_mmc(host->card))
					trace_mmc_request_done(&host->class_dev,
							cmd->opcode, mrq->cmd->arg,
							mrq->data->blocks, ktime_to_ms(diff));
				spin_lock_irqsave(&host->lock, flags);
				if (mrq->data->flags == MMC_DATA_READ) {
					host->perf.rbytes_drv +=
							mrq->data->bytes_xfered;
					host->perf.rtime_drv =
						ktime_add(host->perf.rtime_drv,
							diff);
					host->perf.rcount++;
					if (host->debug_mask & MMC_DEBUG_RANDOM_RW) {
						if (mrq->data->bytes_xfered <= 32*1024) {
							host->perf.rbytes_drv_rand +=
								mrq->data->bytes_xfered;
							host->perf.rtime_drv_rand =
								ktime_add(host->perf.rtime_drv_rand,
										diff);
							host->perf.rcount_rand++;
						}
					}
				} else {
					host->perf.wbytes_drv +=
						mrq->data->bytes_xfered;
					host->perf.wkbytes_drv +=
                                                (mrq->data->bytes_xfered / 1024);
					host->perf.wtime_drv =
						ktime_add(host->perf.wtime_drv,
							diff);
					host->perf.wcount++;
					if (host->debug_mask & MMC_DEBUG_RANDOM_RW) {
						if (mrq->data->bytes_xfered <= 32*1024) {
							host->perf.wbytes_drv_rand +=
								mrq->data->bytes_xfered;
							host->perf.wtime_drv_rand =
								ktime_add(host->perf.wtime_drv_rand,
										diff);
							host->perf.wcount_rand++;
						}
					}
				}
				spin_unlock_irqrestore(&host->lock, flags);
			}
			pr_debug("%s:     %d bytes transferred: %d\n",
				mmc_hostname(host),
				mrq->data->bytes_xfered, mrq->data->error);
		}

		if (mrq->stop) {
			pr_debug("%s:     (CMD%u): %d: %08x %08x %08x %08x\n",
				mmc_hostname(host), mrq->stop->opcode,
				mrq->stop->error,
				mrq->stop->resp[0], mrq->stop->resp[1],
				mrq->stop->resp[2], mrq->stop->resp[3]);
		}

		if (mrq->done)
			mrq->done(mrq);

		mmc_host_clk_release(host);
	}
}

EXPORT_SYMBOL(mmc_request_done);

static void
mmc_start_request(struct mmc_host *host, struct mmc_request *mrq)
{
#ifdef CONFIG_MMC_DEBUG
	unsigned int i, sz;
	struct scatterlist *sg;
#endif

	if (mrq->sbc) {
		pr_debug("<%s: starting CMD%u arg %08x flags %08x>\n",
			 mmc_hostname(host), mrq->sbc->opcode,
			 mrq->sbc->arg, mrq->sbc->flags);
	}

	pr_debug("%s: starting CMD%u arg %08x flags %08x\n",
		 mmc_hostname(host), mrq->cmd->opcode,
		 mrq->cmd->arg, mrq->cmd->flags);

	if (mrq->data) {
		pr_debug("%s:     blksz %d blocks %d flags %08x "
			"tsac %d ms nsac %d\n",
			mmc_hostname(host), mrq->data->blksz,
			mrq->data->blocks, mrq->data->flags,
			mrq->data->timeout_ns / 1000000,
			mrq->data->timeout_clks);
		if (host->card && mmc_card_mmc(host->card))
			trace_mmc_req_start(&host->class_dev, mrq->cmd->opcode,
				mrq->cmd->arg, mrq->data->blocks);
	}

	if (mrq->stop) {
		pr_debug("%s:     CMD%u arg %08x flags %08x\n",
			 mmc_hostname(host), mrq->stop->opcode,
			 mrq->stop->arg, mrq->stop->flags);
	}

	WARN_ON(!host->claimed);

	mrq->cmd->error = 0;
	mrq->cmd->mrq = mrq;
	if (mrq->data) {
		BUG_ON(mrq->data->blksz > host->max_blk_size);
		BUG_ON(mrq->data->blocks > host->max_blk_count);
		BUG_ON(mrq->data->blocks * mrq->data->blksz >
			host->max_req_size);

#ifdef CONFIG_MMC_DEBUG
		sz = 0;
		for_each_sg(mrq->data->sg, sg, mrq->data->sg_len, i)
			sz += sg->length;
		BUG_ON(sz != mrq->data->blocks * mrq->data->blksz);
#endif

		mrq->cmd->data = mrq->data;
		mrq->data->error = 0;
		mrq->data->mrq = mrq;
		if (mrq->stop) {
			mrq->data->stop = mrq->stop;
			mrq->stop->error = 0;
			mrq->stop->mrq = mrq;
		}
		if (host->perf_enable)
			host->perf.start = ktime_get();
	}
	mmc_host_clk_hold(host);
	

	if (host->card && host->clk_scaling.enable) {
		mmc_clk_scaling_update_state(host, mrq);
		if (!host->clk_scaling.invalid_state) {
			mmc_clk_scaling(host, false);
			host->clk_scaling.start_busy = ktime_get();
		}
	}

	host->ops->request(host, mrq);
}

void mmc_blk_init_bkops_statistics(struct mmc_card *card)
{
	int i;
	struct mmc_bkops_stats *bkops_stats;

	if (!card)
		return;

	bkops_stats = &card->bkops_info.bkops_stats;

	spin_lock(&bkops_stats->lock);

	for (i = 0 ; i < BKOPS_NUM_OF_SEVERITY_LEVELS ; ++i)
		bkops_stats->bkops_level[i] = 0;

	bkops_stats->suspend = 0;
	bkops_stats->hpi = 0;
	bkops_stats->enabled = true;

	spin_unlock(&bkops_stats->lock);
}
EXPORT_SYMBOL(mmc_blk_init_bkops_statistics);

void mmc_start_delayed_bkops(struct mmc_card *card)
{
	if (!card || !card->ext_csd.bkops_en || mmc_card_doing_bkops(card))
		return;

	if (card->bkops_info.sectors_changed <
	    card->bkops_info.min_sectors_to_queue_delayed_work)
		return;

	pr_debug("%s: %s: queueing delayed_bkops_work\n",
		 mmc_hostname(card->host), __func__);

	card->bkops_info.cancel_delayed_work = false;
	queue_delayed_work(system_nrt_wq, &card->bkops_info.dw,
			   msecs_to_jiffies(
				   card->bkops_info.delay_ms));
}
EXPORT_SYMBOL(mmc_start_delayed_bkops);

int mmc_card_start_bkops(struct mmc_card *card)
{
	int err = 0;
	unsigned long flags;
	struct mmc_host *host = card->host;

	
	if ((powersave_enabled == PP_EXTREMELY_POWERSAVE) && !ac_status)  {
		pr_debug("%s: skip bkops due to extreme powersave mode\n", __func__);
		return 0;
	}

	mmc_claim_host(host);
	err = __mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			EXT_CSD_BKOPS_START, 1, 0, false, false);
	if (err)
		pr_err("%s: error %d starting bkops\n",
				mmc_hostname(host), err);
	else {
		pr_info("%s: start bkops\n", mmc_hostname(host));
		spin_lock_irqsave(&host->lock, flags);
		mmc_card_set_doing_bkops(card);
		spin_unlock_irqrestore(&host->lock, flags);
	}
	mmc_release_host(host);

	return err;
}
EXPORT_SYMBOL(mmc_card_start_bkops);

int mmc_card_stop_bkops(struct mmc_card *card)
{
	int err = -1;
	u32 status;
	int complete = 0;
	unsigned long flags;
	struct mmc_host *host = card->host;

	if (!mmc_card_doing_bkops(card))
		return 1;

	mmc_rpm_hold(host, &card->dev);
	mmc_claim_host(host);
	err = mmc_send_status(card, &status);
	mmc_release_host(host);
	if (err) {
		pr_err("%s: Get card status fail, err %d\n",
				mmc_hostname(host), err);
		goto out;
	}

	if (R1_CURRENT_STATE(status) == R1_STATE_PRG) {
		err = mmc_interrupt_hpi(host->card);
		if (err)
			pr_err("%s: send hpi fail, err %d\n",
					mmc_hostname(host), err);
	} else
		complete = 1;

	MMC_UPDATE_BKOPS_STATS_HPI(host->card->bkops_info.bkops_stats);
out:
	mmc_rpm_release(host, &card->dev);
	spin_lock_irqsave(&host->lock, flags);
	pr_info("%s: bkops %s\n", mmc_hostname(host),
			complete ? "completed" : "interrupted");
	mmc_card_clr_doing_bkops(card);
	spin_unlock_irqrestore(&host->lock, flags);

	return err;
}
void mmc_start_bkops(struct mmc_card *card, bool from_exception)
{
	int err;

	BUG_ON(!card);
	if (!card->ext_csd.bkops_en)
		return;

	if ((card->bkops_info.cancel_delayed_work) && !from_exception) {
		pr_debug("%s: %s: cancel_delayed_work was set, exit\n",
			 mmc_hostname(card->host), __func__);
		card->bkops_info.cancel_delayed_work = false;
		return;
	}

	
	if ((powersave_enabled == PP_EXTREMELY_POWERSAVE) && !ac_status)  {
		pr_debug("%s: skip bkops due to extreme powersave mode\n", __func__);
		return;
	}

	mmc_rpm_hold(card->host, &card->dev);
	
	if (!mmc_try_claim_host(card->host)) {
		mmc_rpm_release(card->host, &card->dev);
		return;
	}

	if ((card->bkops_info.cancel_delayed_work) && !from_exception) {
		pr_debug("%s: %s: cancel_delayed_work was set, exit\n",
			 mmc_hostname(card->host), __func__);
		card->bkops_info.cancel_delayed_work = false;
		goto out;
	}

	if (mmc_card_doing_bkops(card)) {
		pr_debug("%s: %s: already doing bkops, exit\n",
			 mmc_hostname(card->host), __func__);
		goto out;
	}

	if (from_exception && mmc_card_need_bkops(card))
		goto out;

	if (!mmc_card_need_bkops(card)) {
		err = mmc_read_bkops_status(card);
		if (err) {
			pr_err("%s: %s: Failed to read bkops status: %d\n",
			       mmc_hostname(card->host), __func__, err);
			goto out;
		}

		if (!card->ext_csd.raw_bkops_status)
			goto out;

		pr_info("%s: %s: raw_bkops_status=0x%x, from_exception=%d\n",
			mmc_hostname(card->host), __func__,
			card->ext_csd.raw_bkops_status,
			from_exception);
	}

#ifdef CONFIG_MMC_NEED_BKOPS_IN_SUSPEND
	
	if (card->ext_csd.raw_bkops_status > 0)
		mmc_card_set_need_bkops_in_suspend(card);
#endif

	if (from_exception) {
		pr_debug("%s: %s: Level %d from exception, exit",
			 mmc_hostname(card->host), __func__,
			 card->ext_csd.raw_bkops_status);
		mmc_card_set_need_bkops(card);
		goto out;
	}
	pr_info("%s: %s: Starting bkops\n", mmc_hostname(card->host), __func__);

	err = __mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			EXT_CSD_BKOPS_START, 1, 0, false, false);
	if (err) {
		pr_warn("%s: %s: Error %d when starting bkops\n",
			mmc_hostname(card->host), __func__, err);
		goto out;
	}
	MMC_UPDATE_STATS_BKOPS_SEVERITY_LEVEL(card->bkops_info.bkops_stats,
					card->ext_csd.raw_bkops_status);
	mmc_card_clr_need_bkops(card);

	mmc_card_set_doing_bkops(card);
out:
	mmc_release_host(card->host);
	mmc_rpm_release(card->host, &card->dev);
}
EXPORT_SYMBOL(mmc_start_bkops);

void mmc_start_idle_time_bkops(struct work_struct *work)
{
	struct mmc_card *card = container_of(work, struct mmc_card,
			bkops_info.dw.work);
	struct mmc_host *host = card->host;

	if (card->bkops_info.cancel_delayed_work)
		return;

	if (host && mmc_bus_needs_resume(host))
		return;

	mmc_start_bkops(card, false);
}
EXPORT_SYMBOL(mmc_start_idle_time_bkops);

static void mmc_wait_data_done(struct mmc_request *mrq)
{
    unsigned long flags;
	struct mmc_context_info *context_info = &mrq->host->context_info;

    spin_lock_irqsave(&context_info->lock, flags);
	mrq->host->context_info.is_done_rcv = true;
	wake_up_interruptible(&mrq->host->context_info.wait);
	spin_unlock_irqrestore(&context_info->lock, flags);
}

static void mmc_wait_done(struct mmc_request *mrq)
{
	complete(&mrq->completion);
}

static int __mmc_start_data_req(struct mmc_host *host, struct mmc_request *mrq)
{
	mrq->done = mmc_wait_data_done;
	mrq->host = host;
	if (mmc_card_removed(host->card)) {
		mrq->cmd->error = -ENOMEDIUM;
		mmc_wait_data_done(mrq);
		return -ENOMEDIUM;
	}
	mmc_start_request(host, mrq);

	return 0;
}

static int __mmc_start_req(struct mmc_host *host, struct mmc_request *mrq)
{
	init_completion(&mrq->completion);
	mrq->done = mmc_wait_done;
	if (mmc_card_removed(host->card)) {
		mrq->cmd->error = -ENOMEDIUM;
		complete(&mrq->completion);
		return -ENOMEDIUM;
	}
	mmc_start_request(host, mrq);
	return 0;
}

static bool mmc_should_stop_curr_req(struct mmc_host *host)
{
	int remainder;

	if (host->areq->cmd_flags & REQ_URGENT ||
	    !(host->areq->cmd_flags & REQ_WRITE) ||
	    (host->areq->cmd_flags & REQ_FUA))
		return false;

	mmc_host_clk_hold(host);
	remainder = (host->ops->get_xfer_remain) ?
		host->ops->get_xfer_remain(host) : -1;
	mmc_host_clk_release(host);
	return (remainder > 0);
}

static int mmc_stop_request(struct mmc_host *host)
{
	struct mmc_command cmd = {0};
	struct mmc_card *card = host->card;
	int err = 0;
	u32 status;

	if (!host->ops->stop_request || !card->ext_csd.hpi_en) {
		pr_warn("%s: host ops stop_request() or HPI not supported\n",
				mmc_hostname(host));
		return -ENOTSUPP;
	}
	mmc_host_clk_hold(host);
	err = host->ops->stop_request(host);
	if (err) {
		pr_err("%s: Call to host->ops->stop_request() failed (%d)\n",
				mmc_hostname(host), err);
		goto out;
	}

	cmd.opcode = MMC_STOP_TRANSMISSION;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	err = mmc_wait_for_cmd(host, &cmd, 0);
	if (err) {
		err = mmc_send_status(card, &status);
		if (err) {
			pr_err("%s: Get card status fail\n",
					mmc_hostname(card->host));
			goto out;
		}
		switch (R1_CURRENT_STATE(status)) {
		case R1_STATE_DATA:
		case R1_STATE_RCV:
			pr_err("%s: CMD12 fails with error (%d)\n",
					mmc_hostname(host), err);
			goto out;
		default:
			break;
		}
	}
	err = mmc_interrupt_hpi(card);
	if (err) {
		pr_err("%s: mmc_interrupt_hpi() failed (%d)\n",
				mmc_hostname(host), err);
		goto out;
	}
out:
	mmc_host_clk_release(host);
	return err;
}

static int mmc_wait_for_data_req_done(struct mmc_host *host,
				      struct mmc_request *mrq,
				      struct mmc_async_req *next_req)
{
	struct mmc_command *cmd;
	struct mmc_context_info *context_info = &host->context_info;
	bool pending_is_urgent = false;
	bool is_urgent = false;
	bool is_done_rcv = false;
	int err;
	unsigned long flags;

	while (1) {
		wait_io_event_interruptible(context_info->wait,
				(context_info->is_done_rcv ||
				 context_info->is_new_req  ||
				 context_info->is_urgent));
		spin_lock_irqsave(&context_info->lock, flags);
		is_urgent = context_info->is_urgent;
		is_done_rcv = context_info->is_done_rcv;
		context_info->is_waiting_last_req = false;
		spin_unlock_irqrestore(&context_info->lock, flags);
		if (is_done_rcv) {
			context_info->is_done_rcv = false;
			context_info->is_new_req = false;
			cmd = mrq->cmd;
			if (!cmd->error || !cmd->retries ||
					mmc_card_removed(host->card)) {
				err = host->areq->err_check(host->card,
						host->areq);
				if (pending_is_urgent || is_urgent) {
					if ((err == MMC_BLK_PARTIAL) ||
						(err == MMC_BLK_SUCCESS))
						err = pending_is_urgent ?
						       MMC_BLK_URGENT_DONE
						       : MMC_BLK_URGENT;

					
					context_info->is_urgent = false;
				}
				break; 
			} else {
				pr_info("%s: req failed (CMD%u): %d, retrying...\n",
						mmc_hostname(host),
						cmd->opcode, cmd->error);
				cmd->retries--;
				cmd->error = 0;
				host->ops->request(host, mrq);
				context_info->is_urgent = false;
				
				continue;
			}
		} else if (context_info->is_new_req && !is_urgent) {
			context_info->is_new_req = false;
			if (!next_req) {
				err = MMC_BLK_NEW_REQUEST;
				break; 
			}
		} else {
			if (pending_is_urgent)
				continue; 

			context_info->is_urgent = false;
			context_info->is_new_req = false;
			if (mmc_should_stop_curr_req(host)) {
				mmc_update_clk_scaling(host);
				err = mmc_stop_request(host);
				if (err == MMC_BLK_NO_REQ_TO_STOP) {
					pending_is_urgent = true;
					
					continue;
				} else if (err && !context_info->is_done_rcv) {
					err = MMC_BLK_ABORT;
					break;
				}
				
				if (context_info->is_done_rcv) {
					err = host->areq->err_check(host->card,
							host->areq);
					context_info->is_done_rcv = false;
					break; 
				} else {
					mmc_host_clk_release(host);
				}
				err = host->areq->update_interrupted_req(
						host->card, host->areq);
				if (!err)
					err = MMC_BLK_URGENT;
				break; 
			} else {
				pending_is_urgent = true;
				continue; 
			}
		}
	} 
	return err;
}

#define MMC_REQUEST_TIMEOUT	10000 
static void mmc_wait_for_req_done(struct mmc_host *host,
				  struct mmc_request *mrq)
{
	struct mmc_command *cmd;
	unsigned long timeout;
	int ret;

	while (1) {
		cmd = mrq->cmd;
		
		if (cmd->cmd_timeout_ms > MMC_REQUEST_TIMEOUT)
			timeout = (cmd->cmd_timeout_ms * 2) + 3000;
		else
			timeout = MMC_REQUEST_TIMEOUT + 3000;
		ret = wait_for_completion_io_timeout(&mrq->completion,
			msecs_to_jiffies(timeout));

		if (ret == 0) {
			pr_err("%s: CMD%u %s timeout (%lums)\n",
					mmc_hostname(host), cmd->opcode,
					__func__, timeout);
			cmd->error = -ETIMEDOUT;
			break;
		}
		if (cmd->ignore_timeout && cmd->error == -ETIMEDOUT)
			break;

		if (!cmd->error || !cmd->retries ||
		    mmc_card_removed(host->card))
			break;

		pr_debug("%s: req failed (CMD%u): %d, retrying...\n",
			 mmc_hostname(host), cmd->opcode, cmd->error);
		cmd->retries--;
		cmd->error = 0;
		host->ops->request(host, mrq);
	}
}

static void mmc_pre_req(struct mmc_host *host, struct mmc_request *mrq,
		 bool is_first_req)
{
	if (host->ops->pre_req) {
		mmc_host_clk_hold(host);
		host->ops->pre_req(host, mrq, is_first_req);
		mmc_host_clk_release(host);
	}
}

static void mmc_post_req(struct mmc_host *host, struct mmc_request *mrq,
			 int err)
{
	if (host->ops->post_req) {
		mmc_host_clk_hold(host);
		host->ops->post_req(host, mrq, err);
		mmc_host_clk_release(host);
	}
}

struct mmc_async_req *mmc_start_req(struct mmc_host *host,
				    struct mmc_async_req *areq, int *error)
{
	int err = 0;
	int start_err = 0;
	struct mmc_async_req *data = host->areq;
	unsigned long flags;
	bool is_urgent;

	
	if (areq) {
		mmc_pre_req(host, areq->mrq, !host->areq);
	}

	if (host->areq) {
		err = mmc_wait_for_data_req_done(host, host->areq->mrq,
				areq);
		if (err == MMC_BLK_URGENT || err == MMC_BLK_URGENT_DONE) {
			mmc_post_req(host, host->areq->mrq, 0);
			host->areq = NULL;
			if (areq) {
				if (!(areq->cmd_flags &
						MMC_REQ_NOREINSERT_MASK)) {
					areq->reinsert_req(areq);
					mmc_post_req(host, areq->mrq, 0);
				} else {
					start_err = __mmc_start_data_req(host,
							areq->mrq);
					if (start_err)
						mmc_post_req(host, areq->mrq,
								-EINVAL);
					else
						host->areq = areq;
				}
			}
			goto exit;
		} else if (err == MMC_BLK_NEW_REQUEST) {
			if (error)
				*error = err;
			return NULL;
		}
		if (host->card && mmc_card_mmc(host->card) &&
		    ((mmc_resp_type(host->areq->mrq->cmd) == MMC_RSP_R1) ||
		     (mmc_resp_type(host->areq->mrq->cmd) == MMC_RSP_R1B)) &&
		    (host->areq->mrq->cmd->resp[0] & R1_EXCEPTION_EVENT)) {
			mmc_start_bkops(host->card, true);
			pr_debug("%s: %s: completed BKOPs due to exception",
				 mmc_hostname(host), __func__);
		}
	}
	if (!err && areq) {
		
		spin_lock_irqsave(&host->context_info.lock, flags);
		is_urgent = host->context_info.is_urgent;
		host->context_info.is_urgent = false;
		spin_unlock_irqrestore(&host->context_info.lock, flags);

		if (!is_urgent || (areq->cmd_flags & REQ_URGENT)) {
			start_err = __mmc_start_data_req(host, areq->mrq);
		} else {
			
			err = MMC_BLK_URGENT_DONE;
			if (host->areq) {
				mmc_post_req(host, host->areq->mrq, 0);
				host->areq = NULL;
			}
			areq->reinsert_req(areq);
			mmc_post_req(host, areq->mrq, 0);
			goto exit;
		}
	}

	if (host->areq)
		mmc_post_req(host, host->areq->mrq, 0);

	 
	if ((err || start_err) && areq)
		mmc_post_req(host, areq->mrq, -EINVAL);

	if (err)
		host->areq = NULL;
	else
		host->areq = areq;

exit:
	if (error)
		*error = err;
	return data;
}
EXPORT_SYMBOL(mmc_start_req);

void mmc_wait_for_req(struct mmc_host *host, struct mmc_request *mrq)
{
#ifdef CONFIG_MMC_BLOCK_DEFERRED_RESUME
	if (mmc_bus_needs_resume(host))
		mmc_resume_bus(host);
#endif
	__mmc_start_req(host, mrq);
	mmc_wait_for_req_done(host, mrq);
}
EXPORT_SYMBOL(mmc_wait_for_req);

bool mmc_card_is_prog_state(struct mmc_card *card)
{
	bool rc;
	struct mmc_command cmd;

	mmc_claim_host(card->host);
	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = MMC_SEND_STATUS;
	if (!mmc_host_is_spi(card->host))
		cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	rc = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (rc) {
		pr_err("%s: Get card status fail. rc=%d\n",
		       mmc_hostname(card->host), rc);
		rc = false;
		goto out;
	}

	if (R1_CURRENT_STATE(cmd.resp[0]) == R1_STATE_PRG)
		rc = true;
	else
		rc = false;
out:
	mmc_release_host(card->host);
	return rc;
}
EXPORT_SYMBOL(mmc_card_is_prog_state);

int mmc_interrupt_hpi(struct mmc_card *card)
{
	int err;
	u32 status;
	unsigned long prg_wait;

	BUG_ON(!card);

	if (!card->ext_csd.hpi_en) {
		pr_info("%s: HPI enable bit unset\n", mmc_hostname(card->host));
		return 1;
	}

	mmc_claim_host(card->host);
	err = mmc_send_status(card, &status);
	if (err) {
		pr_err("%s: Get card status fail\n", mmc_hostname(card->host));
		goto out;
	}

	switch (R1_CURRENT_STATE(status)) {
	case R1_STATE_IDLE:
	case R1_STATE_READY:
	case R1_STATE_STBY:
	case R1_STATE_TRAN:
		goto out;
	case R1_STATE_PRG:
		break;
	default:
		
		pr_debug("%s: HPI cannot be sent. Card state=%d\n",
			mmc_hostname(card->host), R1_CURRENT_STATE(status));
		err = -EINVAL;
		goto out;
	}

	err = mmc_send_hpi_cmd(card, &status);

	prg_wait = jiffies + msecs_to_jiffies(card->ext_csd.out_of_int_time);
	do {
		err = mmc_send_status(card, &status);

		if (!err && R1_CURRENT_STATE(status) == R1_STATE_TRAN)
			break;
		if (time_after(jiffies, prg_wait)) {
			err = mmc_send_status(card, &status);
			if (!err && R1_CURRENT_STATE(status) != R1_STATE_TRAN)
				err = -ETIMEDOUT;
			else
				break;
		}
	} while (!err);

out:
	mmc_release_host(card->host);
	return err;
}
EXPORT_SYMBOL(mmc_interrupt_hpi);

int mmc_wait_for_cmd(struct mmc_host *host, struct mmc_command *cmd, int retries)
{
	struct mmc_request mrq = {NULL};

	WARN_ON(!host->claimed);

	memset(cmd->resp, 0, sizeof(cmd->resp));
	cmd->retries = retries;

	mrq.cmd = cmd;
	cmd->data = NULL;

	mmc_wait_for_req(host, &mrq);

	return cmd->error;
}

EXPORT_SYMBOL(mmc_wait_for_cmd);

int mmc_stop_bkops(struct mmc_card *card)
{
	int err = 0;

	BUG_ON(!card);

	card->bkops_info.cancel_delayed_work = true;
	if (delayed_work_pending(&card->bkops_info.dw))
		cancel_delayed_work_sync(&card->bkops_info.dw);
	if (!mmc_card_doing_bkops(card))
		goto out;

	if (!mmc_use_core_runtime_pm(card->host) && mmc_card_doing_bkops(card)
	    && (card->host->parent->power.runtime_status == RPM_SUSPENDING)
	    && mmc_card_is_prog_state(card)) {
		err = -EBUSY;
		goto out;
	}

	err = mmc_interrupt_hpi(card);

	if (!err || (err == -EINVAL)) {
		mmc_card_clr_doing_bkops(card);
		err = 0;
	}

	MMC_UPDATE_BKOPS_STATS_HPI(card->bkops_info.bkops_stats);

out:
	return err;
}
EXPORT_SYMBOL(mmc_stop_bkops);

int mmc_read_bkops_status(struct mmc_card *card)
{
	int err;
	u8 *ext_csd;

	ext_csd = kmalloc(512, GFP_KERNEL);
	if (!ext_csd) {
		pr_err("%s: could not allocate buffer to receive the ext_csd.\n",
		       mmc_hostname(card->host));
		return -ENOMEM;
	}

	if (card->bkops_info.bkops_stats.ignore_card_bkops_status) {
		pr_debug("%s: skipping read raw_bkops_status in unittest mode",
			 __func__);
		return 0;
	}

	mmc_claim_host(card->host);
	err = mmc_send_ext_csd(card, ext_csd);
	mmc_release_host(card->host);
	if (err)
		goto out;

	card->ext_csd.raw_bkops_status = ext_csd[EXT_CSD_BKOPS_STATUS];
	card->ext_csd.raw_exception_status = ext_csd[EXT_CSD_EXP_EVENTS_STATUS];
out:
	kfree(ext_csd);
	return err;
}
EXPORT_SYMBOL(mmc_read_bkops_status);

void mmc_set_data_timeout(struct mmc_data *data, const struct mmc_card *card)
{
	unsigned int mult;

	if (mmc_card_sdio(card)) {
		data->timeout_ns = 1000000000;
		data->timeout_clks = 0;
		return;
	}

	mult = mmc_card_sd(card) ? 100 : 10;

	if (data->flags & MMC_DATA_WRITE)
		mult <<= card->csd.r2w_factor;

	data->timeout_ns = card->csd.tacc_ns * mult;
	data->timeout_clks = card->csd.tacc_clks * mult;

	if (mmc_card_sd(card)) {
		unsigned int timeout_us, limit_us;

		timeout_us = data->timeout_ns / 1000;
		if (mmc_host_clk_rate(card->host))
			timeout_us += data->timeout_clks * 1000 /
				(mmc_host_clk_rate(card->host) / 1000);

		if (data->flags & MMC_DATA_WRITE)
			limit_us = 3000000;
		else
			limit_us = 250000;

		if (timeout_us > limit_us || mmc_card_blockaddr(card)) {
			data->timeout_ns = limit_us * 1000;
			data->timeout_clks = 0;
		}
	}

	if (mmc_card_long_read_time(card) && data->flags & MMC_DATA_READ) {
		data->timeout_ns = 300000000;
		data->timeout_clks = 0;
	}

	if (mmc_host_is_spi(card->host)) {
		if (data->flags & MMC_DATA_WRITE) {
			if (data->timeout_ns < 1000000000)
				data->timeout_ns = 1000000000;	
		} else {
			if (data->timeout_ns < 100000000)
				data->timeout_ns =  100000000;	
		}
	}
	
	if (card->quirks & MMC_QUIRK_INAND_DATA_TIMEOUT) {
		data->timeout_ns = 4000000000u; 
		data->timeout_clks = 0;
	}
	
	if (card->quirks & MMC_QUIRK_BROKEN_DATA_TIMEOUT) {
		if (data->timeout_ns <  4000000000u)
			data->timeout_ns = 4000000000u;	
	}
}
EXPORT_SYMBOL(mmc_set_data_timeout);

unsigned int mmc_align_data_size(struct mmc_card *card, unsigned int sz)
{
	sz = ((sz + 3) / 4) * 4;

	return sz;
}
EXPORT_SYMBOL(mmc_align_data_size);

int __mmc_claim_host(struct mmc_host *host, atomic_t *abort)
{
	DECLARE_WAITQUEUE(wait, current);
	unsigned long flags;
	int stop;

	might_sleep();

	add_wait_queue(&host->wq, &wait);

	spin_lock_irqsave(&host->lock, flags);
	while (1) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		stop = abort ? atomic_read(abort) : 0;
		if (stop || !host->claimed || host->claimer == current)
			break;
		spin_unlock_irqrestore(&host->lock, flags);
		schedule();
		spin_lock_irqsave(&host->lock, flags);
	}
	set_current_state(TASK_RUNNING);
	if (!stop) {
		host->claimed = 1;
		host->claimer = current;
		host->claim_cnt += 1;
	} else
		wake_up(&host->wq);
	spin_unlock_irqrestore(&host->lock, flags);
	remove_wait_queue(&host->wq, &wait);
	if (host->ops->enable && !stop && host->claim_cnt == 1)
		host->ops->enable(host);
#if SD_REMOVE_DEBUG
	if (mmc_is_sd_host(host)) {
	       memcpy(last_claim_comm, current->comm, 16);
	       last_claim_getime = ktime_get();
	}
	if (mmc_is_sd_host(host)) {
	       int loop, urc = 0;
	       struct stackframe frame;
	       register unsigned long current_sp asm ("sp");
	       frame.fp = (unsigned long)__builtin_frame_address(0);
	       frame.sp = current_sp;
	       frame.lr = (unsigned long)__builtin_return_address(0);
	       frame.pc = (unsigned long)__mmc_claim_host;
	       for (loop = 0; loop < 3 && urc >= 0; loop++) {
	               urc = unwind_frame(&frame);
	               sprintf(claim_stack[loop], "mmc1 %pS\n", (void *)frame.pc);
	       }
	}
#endif
	return stop;
}

EXPORT_SYMBOL(__mmc_claim_host);

int mmc_try_claim_host(struct mmc_host *host)
{
	int claimed_host = 0;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	if (!host->claimed || host->claimer == current) {
		host->claimed = 1;
		host->claimer = current;
		host->claim_cnt += 1;
		claimed_host = 1;
	}
	spin_unlock_irqrestore(&host->lock, flags);
	if (host->ops->enable && claimed_host && host->claim_cnt == 1)
		host->ops->enable(host);
	return claimed_host;
}
EXPORT_SYMBOL(mmc_try_claim_host);

void mmc_release_host(struct mmc_host *host)
{
	unsigned long flags;

	WARN_ON(!host->claimed);

	if (host->ops->disable && host->claim_cnt == 1)
		host->ops->disable(host);

	spin_lock_irqsave(&host->lock, flags);
	if (--host->claim_cnt) {
		
		spin_unlock_irqrestore(&host->lock, flags);
	} else {
		host->claimed = 0;
		host->claimer = NULL;
		spin_unlock_irqrestore(&host->lock, flags);
		wake_up(&host->wq);
	}
#if SD_REMOVE_DEBUG
	if (mmc_is_sd_host(host)) {
	       memcpy(last_release_comm, current->comm, 16);
	       last_release_getime = ktime_get();
	}
	if (mmc_is_sd_host(host)) {
	       int loop, urc = 0;
	       struct stackframe frame;
	       register unsigned long current_sp asm ("sp");
	       frame.fp = (unsigned long)__builtin_frame_address(0);
	       frame.sp = current_sp;
	       frame.lr = (unsigned long)__builtin_return_address(0);
	       frame.pc = (unsigned long)mmc_release_host;
	       for (loop = 0; loop < 3 && urc >= 0; loop++) {
	               urc = unwind_frame(&frame);
	               sprintf(release_stack[loop], "mmc1 %pS\n", (void *)frame.pc);
	       }
	}
#endif
}
EXPORT_SYMBOL(mmc_release_host);

void mmc_set_ios(struct mmc_host *host)
{
	struct mmc_ios *ios = &host->ios;

	pr_debug("%s: clock %uHz busmode %u powermode %u cs %u Vdd %u "
		"width %u timing %u\n",
		 mmc_hostname(host), ios->clock, ios->bus_mode,
		 ios->power_mode, ios->chip_select, ios->vdd,
		 ios->bus_width, ios->timing);

	if (ios->clock > 0)
		mmc_set_ungated(host);
	host->ops->set_ios(host, ios);
	if (ios->old_rate != ios->clock) {
		if (likely(ios->clk_ts)) {
			char trace_info[80];
			snprintf(trace_info, 80,
				"%s: freq_KHz %d --> %d | t = %d",
				mmc_hostname(host), ios->old_rate / 1000,
				ios->clock / 1000, jiffies_to_msecs(
					(long)jiffies - (long)ios->clk_ts));
			trace_mmc_clk(trace_info);
		}
		ios->old_rate = ios->clock;
		ios->clk_ts = jiffies;
	}
}
EXPORT_SYMBOL(mmc_set_ios);

void mmc_set_chip_select(struct mmc_host *host, int mode)
{
	mmc_host_clk_hold(host);
	host->ios.chip_select = mode;
	mmc_set_ios(host);
	mmc_host_clk_release(host);
}

static void __mmc_set_clock(struct mmc_host *host, unsigned int hz)
{
	WARN_ON(hz < host->f_min);

	if (hz > host->f_max)
		hz = host->f_max;

	host->ios.clock = hz;
	mmc_set_ios(host);
}

void mmc_set_clock(struct mmc_host *host, unsigned int hz)
{
	mmc_host_clk_hold(host);
	__mmc_set_clock(host, hz);
	mmc_host_clk_release(host);
}

#ifdef CONFIG_MMC_CLKGATE
void mmc_gate_clock(struct mmc_host *host)
{
	unsigned long flags;

	WARN_ON(!host->ios.clock);

	spin_lock_irqsave(&host->clk_lock, flags);
	host->clk_old = host->ios.clock;
	host->ios.clock = 0;
	host->clk_gated = true;
	spin_unlock_irqrestore(&host->clk_lock, flags);
	mmc_set_ios(host);
}

void mmc_ungate_clock(struct mmc_host *host)
{
	if (host->clk_old) {
		WARN_ON(host->ios.clock);
		
		__mmc_set_clock(host, host->clk_old);
	}
}

void mmc_set_ungated(struct mmc_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->clk_lock, flags);
	host->clk_gated = false;
	spin_unlock_irqrestore(&host->clk_lock, flags);
}

#else
void mmc_set_ungated(struct mmc_host *host)
{
}
#endif

void mmc_set_bus_mode(struct mmc_host *host, unsigned int mode)
{
	mmc_host_clk_hold(host);
	host->ios.bus_mode = mode;
	mmc_set_ios(host);
	mmc_host_clk_release(host);
}

void mmc_set_bus_width(struct mmc_host *host, unsigned int width)
{
	mmc_host_clk_hold(host);
	host->ios.bus_width = width;
	mmc_set_ios(host);
	mmc_host_clk_release(host);
}

static int mmc_vdd_to_ocrbitnum(int vdd, bool low_bits)
{
	const int max_bit = ilog2(MMC_VDD_35_36);
	int bit;

	if (vdd < 1650 || vdd > 3600)
		return -EINVAL;

	if (vdd >= 1650 && vdd <= 1950)
		return ilog2(MMC_VDD_165_195);

	if (low_bits)
		vdd -= 1;

	
	bit = (vdd - 2000) / 100 + 8;
	if (bit > max_bit)
		return max_bit;
	return bit;
}

u32 mmc_vddrange_to_ocrmask(int vdd_min, int vdd_max)
{
	u32 mask = 0;

	if (vdd_max < vdd_min)
		return 0;

	
	vdd_max = mmc_vdd_to_ocrbitnum(vdd_max, false);
	if (vdd_max < 0)
		return 0;

	
	vdd_min = mmc_vdd_to_ocrbitnum(vdd_min, true);
	if (vdd_min < 0)
		return 0;

	
	while (vdd_max >= vdd_min)
		mask |= 1 << vdd_max--;

	return mask;
}
EXPORT_SYMBOL(mmc_vddrange_to_ocrmask);

#ifdef CONFIG_REGULATOR

int mmc_regulator_get_ocrmask(struct regulator *supply)
{
	int			result = 0;
	int			count;
	int			i;

	count = regulator_count_voltages(supply);
	if (count < 0)
		return count;

	for (i = 0; i < count; i++) {
		int		vdd_uV;
		int		vdd_mV;

		vdd_uV = regulator_list_voltage(supply, i);
		if (vdd_uV <= 0)
			continue;

		vdd_mV = vdd_uV / 1000;
		result |= mmc_vddrange_to_ocrmask(vdd_mV, vdd_mV);
	}

	return result;
}
EXPORT_SYMBOL(mmc_regulator_get_ocrmask);

int mmc_regulator_set_ocr(struct mmc_host *mmc,
			struct regulator *supply,
			unsigned short vdd_bit)
{
	int			result = 0;
	int			min_uV, max_uV;

	if (vdd_bit) {
		int		tmp;
		int		voltage;

		tmp = vdd_bit - ilog2(MMC_VDD_165_195);
		if (tmp == 0) {
			min_uV = 1650 * 1000;
			max_uV = 1950 * 1000;
		} else {
			min_uV = 1900 * 1000 + tmp * 100 * 1000;
			max_uV = min_uV + 100 * 1000;
		}

		voltage = regulator_get_voltage(supply);

		if (mmc->caps2 & MMC_CAP2_BROKEN_VOLTAGE)
			min_uV = max_uV = voltage;

		if (voltage < 0)
			result = voltage;
		else if (voltage < min_uV || voltage > max_uV)
			result = regulator_set_voltage(supply, min_uV, max_uV);
		else
			result = 0;

		if (result == 0 && !mmc->regulator_enabled) {
			result = regulator_enable(supply);
			if (!result)
				mmc->regulator_enabled = true;
		}
	} else if (mmc->regulator_enabled) {
		result = regulator_disable(supply);
		if (result == 0)
			mmc->regulator_enabled = false;
	}

	if (result)
		dev_err(mmc_dev(mmc),
			"could not set regulator OCR (%d)\n", result);
	return result;
}
EXPORT_SYMBOL(mmc_regulator_set_ocr);

#endif 

u32 mmc_select_voltage(struct mmc_host *host, u32 ocr)
{
	int bit;

	ocr &= host->ocr_avail;

	bit = ffs(ocr);
	if (bit) {
		bit -= 1;

		ocr &= 3 << bit;

		mmc_host_clk_hold(host);
		host->ios.vdd = bit;
		mmc_set_ios(host);
		mmc_host_clk_release(host);
	} else {
		pr_warning("%s: host doesn't support card's voltages\n",
				mmc_hostname(host));
		ocr = 0;
	}

	return ocr;
}

int mmc_set_signal_voltage(struct mmc_host *host, int signal_voltage, bool cmd11)
{
	struct mmc_command cmd = {0};
	int err = 0;

	BUG_ON(!host);

	if ((signal_voltage != MMC_SIGNAL_VOLTAGE_330) && cmd11) {
		cmd.opcode = SD_SWITCH_VOLTAGE;
		cmd.arg = 0;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

		err = mmc_wait_for_cmd(host, &cmd, 0);
		if (err)
			return err;

		if (!mmc_host_is_spi(host) && (cmd.resp[0] & R1_ERROR))
			return -EIO;
	}

	host->ios.signal_voltage = signal_voltage;

	if (host->ops->start_signal_voltage_switch) {
		mmc_host_clk_hold(host);
		err = host->ops->start_signal_voltage_switch(host, &host->ios);
		mmc_host_clk_release(host);
	}

	return err;
}

void mmc_set_timing(struct mmc_host *host, unsigned int timing)
{
	mmc_host_clk_hold(host);
	host->ios.timing = timing;
	mmc_set_ios(host);
	mmc_host_clk_release(host);
}

void mmc_set_driver_type(struct mmc_host *host, unsigned int drv_type)
{
	mmc_host_clk_hold(host);
	host->ios.drv_type = drv_type;
	mmc_set_ios(host);
	mmc_host_clk_release(host);
}

void mmc_power_up(struct mmc_host *host)
{
	int bit;

	mmc_host_clk_hold(host);

	
	if (host->ocr)
		bit = ffs(host->ocr) - 1;
	else
		bit = fls(host->ocr_avail) - 1;

	host->ios.vdd = bit;
	if (mmc_host_is_spi(host))
		host->ios.chip_select = MMC_CS_HIGH;
	else {
		host->ios.chip_select = MMC_CS_DONTCARE;
		host->ios.bus_mode = MMC_BUSMODE_OPENDRAIN;
	}
	host->ios.power_mode = MMC_POWER_UP;
	host->ios.bus_width = MMC_BUS_WIDTH_1;
	host->ios.timing = MMC_TIMING_LEGACY;
	mmc_set_ios(host);

	mmc_delay(10);

	host->ios.clock = host->f_init;

	host->ios.power_mode = MMC_POWER_ON;
	mmc_set_ios(host);

	mmc_delay(10);

	mmc_host_clk_release(host);
}

void mmc_power_off(struct mmc_host *host)
{
	int err;

	if (host && mmc_is_sd_host(host) && host->card && mmc_card_present(host->card) && !host->card->do_remove) {
		mmc_claim_host(host);
		err = mmc_go_idle(host);
		if (err)
			pr_err("%s: cmd0 err %d\n", mmc_hostname(host), err);
		mmc_release_host(host);
	}

	mmc_host_clk_hold(host);

	host->ios.clock = 0;
	host->ios.vdd = 0;

	host->ocr = 1 << (fls(host->ocr_avail) - 1);

	if (!mmc_host_is_spi(host)) {
		host->ios.bus_mode = MMC_BUSMODE_OPENDRAIN;
		host->ios.chip_select = MMC_CS_DONTCARE;
	}
	host->ios.power_mode = MMC_POWER_OFF;
	host->ios.bus_width = MMC_BUS_WIDTH_1;
	host->ios.timing = MMC_TIMING_LEGACY;
	mmc_set_ios(host);

	mmc_delay(1);

	mmc_host_clk_release(host);
}

void mmc_power_cycle(struct mmc_host *host)
{
	mmc_power_off(host);
	
	mmc_delay(50);
	mmc_power_up(host);
}

static void __mmc_release_bus(struct mmc_host *host)
{
	BUG_ON(!host);
	BUG_ON(host->bus_refs);
	BUG_ON(!host->bus_dead);

	host->bus_ops = NULL;
}

static inline void mmc_bus_get(struct mmc_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	host->bus_refs++;
	spin_unlock_irqrestore(&host->lock, flags);
}

static inline void mmc_bus_put(struct mmc_host *host)
{
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	host->bus_refs--;
	if ((host->bus_refs == 0) && host->bus_ops)
		__mmc_release_bus(host);
	spin_unlock_irqrestore(&host->lock, flags);
}

int mmc_resume_bus(struct mmc_host *host)
{
	unsigned long flags;
	int ret = 0;
	int need_manual_resume = 0;

	if (!mmc_bus_needs_resume(host))
		return -EINVAL;

	printk("%s: Starting deferred resume\n", mmc_hostname(host));
	if (!pm_runtime_suspended(&host->class_dev) || mmc_is_sd_host(host))
		need_manual_resume = 1;
	mmc_rpm_hold(host, &host->card->dev);
	mmc_claim_host(host);
	if (need_manual_resume) {
		ret = mmc_resume_host(host);
		if (ret < 0) {
			pr_err("%s: %s: resume host: failed: ret: %d\n",
					mmc_hostname(host), __func__, ret);
		}
	}
	spin_lock_irqsave(&host->lock, flags);
	host->bus_resume_flags &= ~MMC_BUSRESUME_NEEDS_RESUME;
	spin_unlock_irqrestore(&host->lock, flags);
	mmc_release_host(host);
	mmc_rpm_release(host, &host->card->dev);
	pr_info("%s: Deferred resume %s\n", mmc_hostname(host),
		(ret == 0 ? "completed" : "Fail"));
	return ret;
	#if 0
	printk("%s: Starting deferred resume\n", mmc_hostname(host));
	spin_lock_irqsave(&host->lock, flags);
	host->bus_resume_flags &= ~MMC_BUSRESUME_NEEDS_RESUME;
	host->rescan_disable = 0;
	spin_unlock_irqrestore(&host->lock, flags);

	mmc_bus_get(host);
	if (host->bus_ops && !host->bus_dead) {
		mmc_power_up(host);
		BUG_ON(!host->bus_ops->resume);
		host->bus_ops->resume(host);
	}

	mmc_bus_put(host);
	printk("%s: Deferred resume completed\n", mmc_hostname(host));
	return 0;
	#endif
}

EXPORT_SYMBOL(mmc_resume_bus);

void mmc_attach_bus(struct mmc_host *host, const struct mmc_bus_ops *ops)
{
	unsigned long flags;

	BUG_ON(!host);
	BUG_ON(!ops);

	WARN_ON(!host->claimed);

	spin_lock_irqsave(&host->lock, flags);

	BUG_ON(host->bus_ops);
	BUG_ON(host->bus_refs);

	host->bus_ops = ops;
	host->bus_refs = 1;
	host->bus_dead = 0;

	spin_unlock_irqrestore(&host->lock, flags);
}

void mmc_detach_bus(struct mmc_host *host)
{
	unsigned long flags;

	BUG_ON(!host);

	WARN_ON(!host->claimed);
	WARN_ON(!host->bus_ops);

	spin_lock_irqsave(&host->lock, flags);

	host->bus_dead = 1;

	spin_unlock_irqrestore(&host->lock, flags);

	mmc_bus_put(host);
}

void mmc_detect_change(struct mmc_host *host, unsigned long delay)
{
#ifdef CONFIG_MMC_DEBUG
	unsigned long flags;
	spin_lock_irqsave(&host->lock, flags);
	WARN_ON(host->removed);
	spin_unlock_irqrestore(&host->lock, flags);
#endif
	host->detect_change = 1;

	wake_lock_timeout(&host->detect_wake_lock, HZ * MMC_WAKELOCK_TIMEOUT);
	mmc_schedule_delayed_work(&host->detect, delay);
}

EXPORT_SYMBOL(mmc_detect_change);

int mmc_reinit_card(struct mmc_host *host)
{
	int err = 0;
	printk(KERN_INFO "%s: %s\n", mmc_hostname(host),
		__func__);

	mmc_bus_get(host);
	if (host->bus_ops && !host->bus_dead &&
		host->bus_ops->reinit) {
		if (host->card && mmc_card_sd(host->card)) {
			mmc_power_off(host);
			msleep(100);
		}
		mmc_power_up(host);
		err = host->bus_ops->reinit(host);
	}

	mmc_bus_put(host);
	printk(KERN_INFO "%s: %s return %d\n", mmc_hostname(host),
		__func__, err);
	return err;
}

void mmc_remove_sd_card(struct work_struct *work)
{
	struct mmc_host *host =
		container_of(work, struct mmc_host, remove.work);
	const unsigned int REDETECT_MAX = 5;

	printk(KERN_INFO "%s: %s enter\n", mmc_hostname(host),
	       __func__);
	mod_timer(&sd_remove_tout_timer, (jiffies + msecs_to_jiffies(5000)));
	mmc_bus_get(host);
	if (host->bus_ops && !host->bus_dead) {
		if (host->bus_ops->remove)
			host->bus_ops->remove(host);
		mmc_rpm_hold(host, &host->class_dev);
		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_power_off(host);
		mmc_release_host(host);
		mmc_rpm_release(host, &host->class_dev);
	}
	mmc_bus_put(host);
	wake_unlock(&mmc_removal_work_wake_lock);
	wake_lock_timeout(&mmc_removal_work_wake_lock, HZ * 2);

	printk(KERN_INFO "%s: %s exit\n", mmc_hostname(host),
		__func__);

	del_timer_sync(&sd_remove_tout_timer);

	if (host->redetect_cnt++ < REDETECT_MAX) {
	       pr_info("%s : detect try : %d, cd-pin : %d\n", mmc_hostname(host),
	               host->redetect_cnt, host->ops->get_cd(host));
		cancel_delayed_work(&host->detect);
		mmc_detect_change(host, msecs_to_jiffies(1000));
	}
}

void mmc_init_erase(struct mmc_card *card)
{
	unsigned int sz;

	if (is_power_of_2(card->erase_size))
		card->erase_shift = ffs(card->erase_size) - 1;
	else
		card->erase_shift = 0;

	if (mmc_card_sd(card) && card->ssr.au) {
		card->pref_erase = card->ssr.au;
		card->erase_shift = ffs(card->ssr.au) - 1;
	} else if (card->ext_csd.hc_erase_size) {
		card->pref_erase = card->ext_csd.hc_erase_size;
	} else {
		sz = (card->csd.capacity << (card->csd.read_blkbits - 9)) >> 11;
		if (sz < 128)
			card->pref_erase = 512 * 1024 / 512;
		else if (sz < 512)
			card->pref_erase = 1024 * 1024 / 512;
		else if (sz < 1024)
			card->pref_erase = 2 * 1024 * 1024 / 512;
		else
			card->pref_erase = 4 * 1024 * 1024 / 512;
		if (card->pref_erase < card->erase_size)
			card->pref_erase = card->erase_size;
		else {
			sz = card->pref_erase % card->erase_size;
			if (sz)
				card->pref_erase += card->erase_size - sz;
		}
	}
}

static unsigned int mmc_mmc_erase_timeout(struct mmc_card *card,
				          unsigned int arg, unsigned int qty)
{
	unsigned int erase_timeout;

	if (arg == MMC_DISCARD_ARG ||
	    (arg == MMC_TRIM_ARG && card->ext_csd.rev >= 6)) {
		erase_timeout = card->ext_csd.trim_timeout;
	} else if (card->ext_csd.erase_group_def & 1) {
		
		if (arg == MMC_TRIM_ARG)
			erase_timeout = card->ext_csd.trim_timeout;
		else
			erase_timeout = card->ext_csd.hc_erase_timeout;
	} else {
		
		unsigned int mult = (10 << card->csd.r2w_factor);
		unsigned int timeout_clks = card->csd.tacc_clks * mult;
		unsigned int timeout_us;

		
		if (card->csd.tacc_ns < 1000000)
			timeout_us = (card->csd.tacc_ns * mult) / 1000;
		else
			timeout_us = (card->csd.tacc_ns / 1000) * mult;

		timeout_clks <<= 1;
		timeout_us += (timeout_clks * 1000) /
			      (mmc_host_clk_rate(card->host) / 1000);

		erase_timeout = timeout_us / 1000;

		if (!erase_timeout)
			erase_timeout = 1;
	}

	

	erase_timeout *= qty;

	if (mmc_host_is_spi(card->host) && erase_timeout < 1000)
		erase_timeout = 1000;

	return erase_timeout;
}

static unsigned int mmc_sd_erase_timeout(struct mmc_card *card,
					 unsigned int arg,
					 unsigned int qty)
{
	unsigned int erase_timeout;

	if (card->ssr.erase_timeout) {
		
		erase_timeout = card->ssr.erase_timeout * qty +
				card->ssr.erase_offset;
	} else {
		erase_timeout = 250 * qty;
	}

	
	if (erase_timeout < 1000)
		erase_timeout = 1000;

	return erase_timeout;
}

static unsigned int mmc_erase_timeout(struct mmc_card *card,
				      unsigned int arg,
				      unsigned int qty)
{
	if (mmc_card_sd(card))
		return mmc_sd_erase_timeout(card, arg, qty);
	else
		return mmc_mmc_erase_timeout(card, arg, qty);
}

int mmc_send_single_read(struct mmc_card *card, struct mmc_host *host, unsigned int from)
{
	struct mmc_request mrq = {NULL};
	struct mmc_command cmd = {0};
	struct mmc_data data = {0};
	struct scatterlist sg;
	void *data_buf;
	int len = 512;

	data_buf = kmalloc(len, GFP_KERNEL);
	if (data_buf == NULL)
		return -ENOMEM;

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = MMC_READ_SINGLE_BLOCK;
	cmd.arg = from;

	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = 512;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	sg_init_one(&sg, data_buf, len);
	mmc_set_data_timeout(&data, card);
	mmc_wait_for_req(host, &mrq);

	kfree(data_buf);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	return 0;
}


static int mmc_do_erase(struct mmc_card *card, unsigned int from,
			unsigned int to, unsigned int arg)
{
	struct mmc_command cmd = {0};
	unsigned int qty = 0;
	unsigned long timeout;
	int err;
	ktime_t start, diff;

	if (card->erase_shift)
		qty += ((to >> card->erase_shift) -
			(from >> card->erase_shift)) + 1;
	else if (mmc_card_sd(card))
		qty += to - from + 1;
	else
		qty += ((to / card->erase_size) -
			(from / card->erase_size)) + 1;

	if (!mmc_card_blockaddr(card)) {
		from <<= 9;
		to <<= 9;
	}

	if (card->cid.manfid == HYNIX_MMC)
		if (mmc_send_single_read(card, card->host, from) != 0)
			pr_err("%s, Dummy read failed\n", __func__);


	if (mmc_card_sd(card))
		cmd.opcode = SD_ERASE_WR_BLK_START;
	else
		cmd.opcode = MMC_ERASE_GROUP_START;
	cmd.arg = from;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err) {
		pr_err("mmc_erase: group start error %d, "
		       "status %#x\n", err, cmd.resp[0]);
		err = -EIO;
		goto out;
	}

	memset(&cmd, 0, sizeof(struct mmc_command));
	if (mmc_card_sd(card))
		cmd.opcode = SD_ERASE_WR_BLK_END;
	else
		cmd.opcode = MMC_ERASE_GROUP_END;
	cmd.arg = to;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err) {
		pr_err("mmc_erase: group end error %d, status %#x\n",
		       err, cmd.resp[0]);
		err = -EIO;
		goto out;
	}

	if (mmc_card_mmc(card))
		trace_mmc_req_start(&(card->host->class_dev), MMC_ERASE,
			from, to - from + 1);
	start = ktime_get();

	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = MMC_ERASE;
	cmd.arg = arg;
	cmd.flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
	cmd.cmd_timeout_ms = mmc_erase_timeout(card, arg, qty);
	err = mmc_wait_for_cmd(card->host, &cmd, 0);
	if (err) {
		pr_err("mmc_erase: erase error %d, status %#x\n",
		       err, cmd.resp[0]);
		err = -EIO;
		goto out;
	}

	if (mmc_host_is_spi(card->host))
		goto out;

	timeout = jiffies + msecs_to_jiffies(MMC_CORE_TIMEOUT_MS);
	do {
		memset(&cmd, 0, sizeof(struct mmc_command));
		cmd.opcode = MMC_SEND_STATUS;
		cmd.arg = card->rca << 16;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
		
		err = mmc_wait_for_cmd(card->host, &cmd, 0);
		if (err || (cmd.resp[0] & 0xFDF92000)) {
			pr_err("error %d requesting status %#x\n",
				err, cmd.resp[0]);
			if (err)
				err = -EIO;
			goto out;
		}

		if (time_after(jiffies, timeout)) {
			pr_err("%s: Card stuck in programming state! %s\n",
				mmc_hostname(card->host), __func__);
			err =  -EIO;
			goto out;
		}

	} while (!(cmd.resp[0] & R1_READY_FOR_DATA) ||
		 (R1_CURRENT_STATE(cmd.resp[0]) == R1_STATE_PRG));

	diff = ktime_sub(ktime_get(), start);
	if (ktime_to_ms(diff) >= 3000)
		pr_info("%s: erase(sector %u to %u) takes %lld ms\n",
			mmc_hostname(card->host), from, to, ktime_to_ms(diff));

	card->host->perf.erase_blks += (to - from + 1);
	card->host->perf.erase_time =
		ktime_add(card->host->perf.erase_time, diff);
	card->host->perf.erase_rq++;

	if (mmc_card_mmc(card))
		trace_mmc_request_done(&(card->host->class_dev), MMC_ERASE,
			from, to - from + 1, ktime_to_ms(diff));
out:
	return err;
}

int mmc_erase(struct mmc_card *card, unsigned int from, unsigned int nr,
	      unsigned int arg)
{
	unsigned int rem, to = from + nr;

	if (!(card->host->caps & MMC_CAP_ERASE) ||
	    !(card->csd.cmdclass & CCC_ERASE))
		return -EOPNOTSUPP;

	if (!card->erase_size)
		return -EOPNOTSUPP;

	if (mmc_card_sd(card) && arg != MMC_ERASE_ARG)
		return -EOPNOTSUPP;

	if ((arg & MMC_SECURE_ARGS) &&
	    !(card->ext_csd.sec_feature_support & EXT_CSD_SEC_ER_EN))
		return -EOPNOTSUPP;

	if ((arg & MMC_TRIM_ARGS) &&
	    !(card->ext_csd.sec_feature_support & EXT_CSD_SEC_GB_CL_EN))
		return -EOPNOTSUPP;


	if (arg == MMC_ERASE_ARG) {
		rem = from % card->erase_size;
		if (rem) {
			rem = card->erase_size - rem;
			from += rem;
			if (nr > rem)
				nr -= rem;
			else
				return 0;
		}
		rem = nr % card->erase_size;
		if (rem)
			nr -= rem;
	}

	if (nr == 0)
		return 0;

	to = from + nr;

	if (to <= from)
		return -EINVAL;

	
	to -= 1;

	return mmc_do_erase(card, from, to, arg);
}
EXPORT_SYMBOL(mmc_erase);

int mmc_can_erase(struct mmc_card *card)
{
	if ((card->host->caps & MMC_CAP_ERASE) &&
	    (card->csd.cmdclass & CCC_ERASE) && card->erase_size)
		return 1;
	return 0;
}
EXPORT_SYMBOL(mmc_can_erase);

int mmc_can_trim(struct mmc_card *card)
{
	if (card->ext_csd.sec_feature_support & EXT_CSD_SEC_GB_CL_EN)
		return 1;
	return 0;
}
EXPORT_SYMBOL(mmc_can_trim);

int mmc_can_discard(struct mmc_card *card)
{
	if (card->ext_csd.feature_support & MMC_DISCARD_FEATURE)
		return 1;
	return 0;
}
EXPORT_SYMBOL(mmc_can_discard);

int mmc_can_sanitize(struct mmc_card *card)
{
#if 0 
	if (!mmc_can_trim(card) && !mmc_can_erase(card))
		return 0;
	if (card->ext_csd.sec_feature_support & EXT_CSD_SEC_SANITIZE)
		return 1;
#endif
	return 0;
}
EXPORT_SYMBOL(mmc_can_sanitize);

int mmc_can_secure_erase_trim(struct mmc_card *card)
{
	if (card->ext_csd.sec_feature_support & EXT_CSD_SEC_ER_EN)
		return 1;
	return 0;
}
EXPORT_SYMBOL(mmc_can_secure_erase_trim);

int mmc_erase_group_aligned(struct mmc_card *card, unsigned int from,
			    unsigned int nr)
{
	if (!card->erase_size)
		return 0;
	if (from % card->erase_size || nr % card->erase_size)
		return 0;
	return 1;
}
EXPORT_SYMBOL(mmc_erase_group_aligned);

static unsigned int mmc_do_calc_max_discard(struct mmc_card *card,
					    unsigned int arg)
{
	struct mmc_host *host = card->host;
	unsigned int max_discard, x, y, qty = 0, max_qty, timeout;
	unsigned int last_timeout = 0;

	if (card->erase_shift)
		max_qty = UINT_MAX >> card->erase_shift;
	else if (mmc_card_sd(card))
		max_qty = UINT_MAX;
	else
		max_qty = UINT_MAX / card->erase_size;

	
	do {
		y = 0;
		for (x = 1; x && x <= max_qty && max_qty - x >= qty; x <<= 1) {
			timeout = mmc_erase_timeout(card, arg, qty + x);
			if (timeout > host->max_discard_to)
				break;
			if (timeout < last_timeout)
				break;
			last_timeout = timeout;
			y = x;
		}
		qty += y;
	} while (y);

	if (!qty)
		return 0;

	if (qty == 1)
		return 1;

	
	if (card->erase_shift)
		max_discard = --qty << card->erase_shift;
	else if (mmc_card_sd(card))
		max_discard = qty;
	else
		max_discard = --qty * card->erase_size;

	return max_discard;
}

unsigned int mmc_calc_max_discard(struct mmc_card *card)
{
	struct mmc_host *host = card->host;
	unsigned int max_discard, max_trim;

	if (!host->max_discard_to)
		return UINT_MAX;

	if (mmc_card_mmc(card) && !(card->ext_csd.erase_group_def & 1))
		return card->pref_erase;

	max_discard = mmc_do_calc_max_discard(card, MMC_ERASE_ARG);
	if (mmc_can_trim(card)) {
		max_trim = mmc_do_calc_max_discard(card, MMC_TRIM_ARG);
		if (max_trim < max_discard)
			max_discard = max_trim;
	} else if (max_discard < card->erase_size) {
		max_discard = 0;
	}
	pr_debug("%s: calculated max. discard sectors %u for timeout %u ms\n",
		 mmc_hostname(host), max_discard, host->max_discard_to);
	return max_discard;
}
EXPORT_SYMBOL(mmc_calc_max_discard);

int mmc_set_blocklen(struct mmc_card *card, unsigned int blocklen)
{
	struct mmc_command cmd = {0};

	if (mmc_card_blockaddr(card) || mmc_card_ddr_mode(card))
		return 0;

	cmd.opcode = MMC_SET_BLOCKLEN;
	cmd.arg = blocklen;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	return mmc_wait_for_cmd(card->host, &cmd, 5);
}
EXPORT_SYMBOL(mmc_set_blocklen);

int mmc_set_blockcount(struct mmc_card *card, unsigned int blockcount,
			bool is_rel_write)
{
	struct mmc_command cmd = {0};

	cmd.opcode = MMC_SET_BLOCK_COUNT;
	cmd.arg = blockcount & 0x0000FFFF;
	if (is_rel_write)
		cmd.arg |= 1 << 31;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	return mmc_wait_for_cmd(card->host, &cmd, 5);
}
EXPORT_SYMBOL(mmc_set_blockcount);

static void mmc_hw_reset_for_init(struct mmc_host *host)
{
	if (!(host->caps & MMC_CAP_HW_RESET) || !host->ops->hw_reset)
		return;
	mmc_host_clk_hold(host);
	host->ops->hw_reset(host);
	mmc_host_clk_release(host);
}

int mmc_can_reset(struct mmc_card *card)
{
	u8 rst_n_function;

	if (mmc_card_sdio(card))
		return 0;

	if (mmc_card_mmc(card) && (card->host->caps & MMC_CAP_HW_RESET)) {
		rst_n_function = card->ext_csd.rst_n_function;
		if ((rst_n_function & EXT_CSD_RST_N_EN_MASK) !=
		    EXT_CSD_RST_N_ENABLED)
			return 0;
	}
	return 1;
}
EXPORT_SYMBOL(mmc_can_reset);

static int mmc_do_hw_reset(struct mmc_host *host, int check)
{
	struct mmc_card *card = host->card;

	if (!host->bus_ops->power_restore)
		return -EOPNOTSUPP;

	if (!card)
		return -EINVAL;

	if (!mmc_can_reset(card))
		return -EOPNOTSUPP;

	mmc_host_clk_hold(host);
	mmc_set_clock(host, host->f_init);

	if (mmc_card_mmc(card) && host->ops->hw_reset)
		host->ops->hw_reset(host);
	else
		mmc_power_cycle(host);

	
	if (check) {
		struct mmc_command cmd = {0};
		int err;

		cmd.opcode = MMC_SEND_STATUS;
		if (!mmc_host_is_spi(card->host))
			cmd.arg = card->rca << 16;
		cmd.flags = MMC_RSP_SPI_R2 | MMC_RSP_R1 | MMC_CMD_AC;
		err = mmc_wait_for_cmd(card->host, &cmd, 0);
		if (!err) {
			mmc_host_clk_release(host);
			return -ENOSYS;
		}
	}

	host->card->state &= ~(MMC_STATE_HIGHSPEED | MMC_STATE_HIGHSPEED_DDR);
	if (mmc_host_is_spi(host)) {
		host->ios.chip_select = MMC_CS_HIGH;
		host->ios.bus_mode = MMC_BUSMODE_PUSHPULL;
	} else {
		host->ios.chip_select = MMC_CS_DONTCARE;
		host->ios.bus_mode = MMC_BUSMODE_OPENDRAIN;
	}
	host->ios.bus_width = MMC_BUS_WIDTH_1;
	host->ios.timing = MMC_TIMING_LEGACY;
	mmc_set_ios(host);

	mmc_host_clk_release(host);

	return host->bus_ops->power_restore(host);
}

int mmc_hw_reset(struct mmc_host *host)
{
	return mmc_do_hw_reset(host, 0);
}
EXPORT_SYMBOL(mmc_hw_reset);

int mmc_hw_reset_check(struct mmc_host *host)
{
	return mmc_do_hw_reset(host, 1);
}
EXPORT_SYMBOL(mmc_hw_reset_check);

void mmc_reset_clk_scale_stats(struct mmc_host *host)
{
	host->clk_scaling.busy_time_us = 0;
	host->clk_scaling.window_time = jiffies;
}
EXPORT_SYMBOL_GPL(mmc_reset_clk_scale_stats);

unsigned long mmc_get_max_frequency(struct mmc_host *host)
{
	unsigned long freq;
	unsigned char timing;

	if (host->ops && host->ops->get_max_frequency) {
		freq = host->ops->get_max_frequency(host);
		goto out;
	}

	if (mmc_card_hs400(host->card))
		timing = MMC_TIMING_MMC_HS400;
	else
		timing = host->ios.timing;

	switch (timing) {
	case MMC_TIMING_UHS_SDR50:
		freq = UHS_SDR50_MAX_DTR;
		break;
	case MMC_TIMING_UHS_SDR104:
		freq = UHS_SDR104_MAX_DTR;
		break;
	case MMC_TIMING_MMC_HS200:
		freq = MMC_HS200_MAX_DTR;
		break;
	case MMC_TIMING_UHS_DDR50:
		freq = UHS_DDR50_MAX_DTR;
		break;
	case MMC_TIMING_MMC_HS400:
		freq = MMC_HS400_MAX_DTR;
		break;
	default:
		mmc_host_clk_hold(host);
		freq = host->ios.clock;
		mmc_host_clk_release(host);
		break;
	}

out:
	return freq;
}
EXPORT_SYMBOL_GPL(mmc_get_max_frequency);

static unsigned long mmc_get_min_frequency(struct mmc_host *host)
{
	unsigned long freq;

	if (host->ops && host->ops->get_min_frequency) {
		freq = host->ops->get_min_frequency(host);
		goto out;
	}

	switch (host->ios.timing) {
	case MMC_TIMING_UHS_SDR50:
	case MMC_TIMING_UHS_SDR104:
		freq = UHS_SDR25_MAX_DTR;
		break;
	case MMC_TIMING_MMC_HS200:
		freq = MMC_HIGH_52_MAX_DTR;
		break;
	case MMC_TIMING_MMC_HS400:
		freq = MMC_HIGH_52_MAX_DTR;
		break;
	case MMC_TIMING_UHS_DDR50:
		freq = UHS_DDR50_MAX_DTR / 2;
		break;
	default:
		mmc_host_clk_hold(host);
		freq = host->ios.clock;
		mmc_host_clk_release(host);
		break;
	}

out:
	return freq;
}

static void mmc_clk_scale_work(struct work_struct *work)
{
	struct mmc_host *host = container_of(work, struct mmc_host,
					      clk_scaling.work.work);

	if (!host->card || !host->bus_ops ||
			!host->bus_ops->change_bus_speed ||
			!host->clk_scaling.enable || !host->ios.clock)
		return;

	mmc_rpm_hold(host, &host->card->dev);
	if (!mmc_try_claim_host(host)) {
		
		queue_delayed_work(system_nrt_wq, &host->clk_scaling.work, 1);
		goto out;
	}

	mmc_clk_scaling(host, true);
	mmc_release_host(host);
out:
	mmc_rpm_release(host, &host->card->dev);
	return;
}

static bool mmc_is_vaild_state_for_clk_scaling(struct mmc_host *host)
{
	struct mmc_card *card = host->card;
	u32 status;
	bool ret = false;

	if (!card || (mmc_card_mmc(card) &&
			card->part_curr == EXT_CSD_PART_CONFIG_ACC_RPMB)
			|| host->clk_scaling.invalid_state)
		goto out;

	if (mmc_send_status(card, &status)) {
		pr_err("%s: Get card status fail\n", mmc_hostname(card->host));
		goto out;
	}

	switch (R1_CURRENT_STATE(status)) {
	case R1_STATE_TRAN:
		ret = true;
		break;
	default:
		break;
	}
out:
	return ret;
}

static int mmc_clk_update_freq(struct mmc_host *host,
		unsigned long freq, enum mmc_load state)
{
	int err = 0;

	if (host->ops->notify_load) {
		err = host->ops->notify_load(host, state);
		if (err)
			goto out;
	}

	if (freq != host->clk_scaling.curr_freq) {
		if (!mmc_is_vaild_state_for_clk_scaling(host)) {
			err = -EAGAIN;
			goto error;
		}

		err = host->bus_ops->change_bus_speed(host, &freq);
		if (!err)
			host->clk_scaling.curr_freq = freq;
		else
			pr_err("%s: %s: failed (%d) at freq=%lu\n",
				mmc_hostname(host), __func__, err, freq);
	}
error:
	if (err) {
		
		if (host->ops->notify_load)
			host->ops->notify_load(host, host->clk_scaling.state);
	}
out:
	return err;
}

static void mmc_clk_scaling(struct mmc_host *host, bool from_wq)
{
	int err = 0;
	struct mmc_card *card = host->card;
	unsigned long total_time_ms = 0;
	unsigned long busy_time_ms = 0;
	unsigned long freq;
	unsigned int up_threshold = host->clk_scaling.up_threshold;
	unsigned int down_threshold = host->clk_scaling.down_threshold;
	bool queue_scale_down_work = false;
	enum mmc_load state;

	if (!card || !host->bus_ops || !host->bus_ops->change_bus_speed) {
		pr_err("%s: %s: invalid entry\n", mmc_hostname(host), __func__);
		goto out;
	}

	
	if (!host->ios.clock)
		goto out;

	if (time_is_after_jiffies(host->clk_scaling.window_time +
			msecs_to_jiffies(host->clk_scaling.polling_delay_ms)))
		goto out;

	
	total_time_ms = jiffies_to_msecs((long)jiffies -
			(long)host->clk_scaling.window_time);

	
	if (unlikely(host->clk_scaling.in_progress))
		goto out;

	host->clk_scaling.in_progress = true;

	busy_time_ms = host->clk_scaling.busy_time_us / USEC_PER_MSEC;

	freq = host->clk_scaling.curr_freq;
	state = host->clk_scaling.state;

	if ((busy_time_ms * 100 > total_time_ms * up_threshold)) {
		freq = mmc_get_max_frequency(host);
		state = MMC_LOAD_HIGH;
	} else if ((busy_time_ms * 100 < total_time_ms * down_threshold)) {
		if (!from_wq)
			queue_scale_down_work = true;
		freq = mmc_get_min_frequency(host);
		state = MMC_LOAD_LOW;
	}

	if (state != host->clk_scaling.state) {
		if (!queue_scale_down_work) {
			if (!from_wq)
				cancel_delayed_work_sync(
						&host->clk_scaling.work);
			err = mmc_clk_update_freq(host, freq, state);
			if (!err)
				host->clk_scaling.state = state;
			else if (err == -EAGAIN)
				goto no_reset_stats;
		} else {
			queue_delayed_work(system_nrt_wq,
					&host->clk_scaling.work, 1);
			goto no_reset_stats;
		}
	}

	mmc_reset_clk_scale_stats(host);
no_reset_stats:
	host->clk_scaling.in_progress = false;
out:
	return;
}

void mmc_disable_clk_scaling(struct mmc_host *host)
{
	cancel_delayed_work_sync(&host->clk_scaling.work);
	host->clk_scaling.enable = false;
}
EXPORT_SYMBOL_GPL(mmc_disable_clk_scaling);

bool mmc_can_scale_clk(struct mmc_host *host)
{
	return host->clk_scaling.initialized;
}
EXPORT_SYMBOL_GPL(mmc_can_scale_clk);

void mmc_init_clk_scaling(struct mmc_host *host)
{
	if (!host->card || !(host->caps2 & MMC_CAP2_CLK_SCALE))
		return;

	INIT_DELAYED_WORK(&host->clk_scaling.work, mmc_clk_scale_work);
	host->clk_scaling.curr_freq = mmc_get_max_frequency(host);
	if (host->ops->notify_load)
		host->ops->notify_load(host, MMC_LOAD_HIGH);
	host->clk_scaling.state = MMC_LOAD_HIGH;
	mmc_reset_clk_scale_stats(host);
	host->clk_scaling.enable = true;
	host->clk_scaling.initialized = true;
	pr_debug("%s: clk scaling enabled\n", mmc_hostname(host));
}
EXPORT_SYMBOL_GPL(mmc_init_clk_scaling);

void mmc_exit_clk_scaling(struct mmc_host *host)
{
	cancel_delayed_work_sync(&host->clk_scaling.work);
	memset(&host->clk_scaling, 0, sizeof(host->clk_scaling));
}
EXPORT_SYMBOL_GPL(mmc_exit_clk_scaling);

static int mmc_rescan_try_freq(struct mmc_host *host, unsigned freq)
{
	host->f_init = freq;

#ifdef CONFIG_MMC_DEBUG
	pr_info("%s: %s: trying to init card at %u Hz\n",
		mmc_hostname(host), __func__, host->f_init);
#endif
	mmc_power_up(host);

	mmc_hw_reset_for_init(host);

	
	mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_330, 0);

	sdio_reset(host);
	mmc_go_idle(host);

	mmc_send_if_cond(host, host->ocr_avail);

	
	if (!mmc_attach_sdio(host)) {
	       pr_info("%s: Find a SDIO card\n", __func__);
		return 0;
	}
	if (!host->ios.vdd)
	       mmc_power_up(host);
	if (mmc_is_sd_host(host) && !mmc_attach_sd(host)) {
		pr_info("%s: Find a SD card\n", __func__);
		return 0;
	}
	if (!host->ios.vdd)
	       mmc_power_up(host);
	if (mmc_is_mmc_host(host) && !mmc_attach_mmc(host)) {
		pr_info("%s: Find a MMC/eMMC card\n", __func__);
		return 0;
	}
	mmc_power_off(host);
	pr_info("%s: %s Can not find a card type. A dummy card ?\n",
		mmc_hostname(host), __func__);
	return -EIO;
}

int _mmc_detect_card_removed(struct mmc_host *host)
{
	int ret;

	if ((host->caps & MMC_CAP_NONREMOVABLE) || !host->bus_ops->alive)
		return 0;

	if (!host->card || mmc_card_removed(host->card))
		return 1;

	ret = host->bus_ops->alive(host);

	if (!ret && host->ops->get_cd && !host->ops->get_cd(host)) {
		mmc_detect_change(host, msecs_to_jiffies(200));
		pr_debug("%s: card removed too slowly\n", mmc_hostname(host));
	}

	if (ret) {
		if(mmc_card_sd(host->card))
			host->card->do_remove = 1;
		else
			mmc_card_set_removed(host->card);
		pr_info("%s: card remove detected\n", mmc_hostname(host));
	}

	return ret;
}

int mmc_detect_card_removed(struct mmc_host *host)
{
	struct mmc_card *card = host->card;
	int ret;

	WARN_ON(!host->claimed);

	if (!card)
		return 1;

	ret = mmc_card_removed(card);
	if (!host->detect_change && !(host->caps & MMC_CAP_NEEDS_POLL) &&
	    !(host->caps2 & MMC_CAP2_DETECT_ON_ERR))
		return ret;

	host->detect_change = 0;
	if (!ret) {
		ret = _mmc_detect_card_removed(host);
#if 0
		if (ret && (host->caps2 & MMC_CAP2_DETECT_ON_ERR)) {
			cancel_delayed_work(&host->detect);
			mmc_detect_change(host, 0);
		}
#endif
	}

	return ret;
}
EXPORT_SYMBOL(mmc_detect_card_removed);

void mmc_rescan(struct work_struct *work)
{
	struct mmc_host *host =
		container_of(work, struct mmc_host, detect.work);

	if (host->rescan_disable)
		return;

	mmc_bus_get(host);
	mmc_rpm_hold(host, &host->class_dev);

	if (host->bus_ops && host->bus_ops->detect && !host->bus_dead
	    && !(host->caps & MMC_CAP_NONREMOVABLE))
		host->bus_ops->detect(host);

	host->detect_change = 0;

	mmc_bus_put(host);
	mmc_bus_get(host);

	
	if (host->bus_ops != NULL) {
		mmc_rpm_release(host, &host->class_dev);
		mmc_bus_put(host);
		goto out;
	}

	mmc_rpm_release(host, &host->class_dev);

	mmc_bus_put(host);
#if 0
	if (host->ops->get_cd && host->ops->get_cd(host) == 0)
		goto out;
#endif
	mmc_rpm_hold(host, &host->class_dev);
	mmc_claim_host(host);
	mmc_rescan_try_freq(host, host->f_min);
	mmc_release_host(host);
	mmc_rpm_release(host, &host->class_dev);
 out:
	if (host->caps & MMC_CAP_NEEDS_POLL) {
		wake_lock_timeout(&host->detect_wake_lock, HZ * MMC_WAKELOCK_TIMEOUT);
		mmc_schedule_delayed_work(&host->detect, HZ);
	}
}

void mmc_start_host(struct mmc_host *host)
{
	mmc_power_off(host);
	mmc_detect_change(host, 0);
}

void mmc_stop_host(struct mmc_host *host)
{
#ifdef CONFIG_MMC_DEBUG
	unsigned long flags;
	spin_lock_irqsave(&host->lock, flags);
	host->removed = 1;
	spin_unlock_irqrestore(&host->lock, flags);
#endif

	if (cancel_delayed_work_sync(&host->detect))
		wake_unlock(&host->detect_wake_lock);

	mmc_flush_scheduled_work();

	
	host->pm_flags = 0;

	mmc_bus_get(host);
	if (host->bus_ops && !host->bus_dead) {
		
		if (host->bus_ops->remove)
			host->bus_ops->remove(host);

		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_power_off(host);
		mmc_release_host(host);
		mmc_bus_put(host);
		return;
	}
	mmc_bus_put(host);

	BUG_ON(host->card);

	mmc_power_off(host);
}

int mmc_power_save_host(struct mmc_host *host)
{
	int ret = 0;

#ifdef CONFIG_MMC_DEBUG
	pr_info("%s: %s: powering down\n", mmc_hostname(host), __func__);
#endif

	mmc_bus_get(host);

	if (!host->bus_ops || host->bus_dead || !host->bus_ops->power_restore) {
		mmc_bus_put(host);
		return -EINVAL;
	}

	if (host->bus_ops->power_save)
		ret = host->bus_ops->power_save(host);

	mmc_bus_put(host);

	mmc_power_off(host);

	return ret;
}
EXPORT_SYMBOL(mmc_power_save_host);

int mmc_power_restore_host(struct mmc_host *host)
{
	int ret;

#ifdef CONFIG_MMC_DEBUG
	pr_info("%s: %s: powering up\n", mmc_hostname(host), __func__);
#endif

	mmc_bus_get(host);

	if (!host->bus_ops || host->bus_dead || !host->bus_ops->power_restore) {
		mmc_bus_put(host);
		return -EINVAL;
	}

	mmc_power_up(host);
	ret = host->bus_ops->power_restore(host);

	mmc_bus_put(host);

	return ret;
}
EXPORT_SYMBOL(mmc_power_restore_host);

int mmc_card_awake(struct mmc_host *host)
{
	int err = -ENOSYS;

	if (host->caps2 & MMC_CAP2_NO_SLEEP_CMD)
		return 0;

	mmc_bus_get(host);

	if (host->bus_ops && !host->bus_dead && host->bus_ops->awake)
		err = host->bus_ops->awake(host);

	mmc_bus_put(host);

	return err;
}
EXPORT_SYMBOL(mmc_card_awake);

int mmc_card_sleep(struct mmc_host *host)
{
	int err = -ENOSYS;

	if (host->caps2 & MMC_CAP2_NO_SLEEP_CMD)
		return 0;

	mmc_bus_get(host);

	if (host->bus_ops && !host->bus_dead && host->bus_ops->sleep)
		err = host->bus_ops->sleep(host);

	mmc_bus_put(host);

	return err;
}
EXPORT_SYMBOL(mmc_card_sleep);

int mmc_card_can_sleep(struct mmc_host *host)
{
	struct mmc_card *card = host->card;

	if (card && mmc_card_mmc(card) && card->ext_csd.rev >= 3)
		return 1;
	return 0;
}
EXPORT_SYMBOL(mmc_card_can_sleep);

int mmc_flush_cache(struct mmc_card *card)
{
	struct mmc_host *host = card->host;
	int err = 0;

	if (!(host->caps2 & MMC_CAP2_CACHE_CTRL) ||
	     (card->quirks & MMC_QUIRK_CACHE_DISABLE))
		return err;

	if (mmc_card_mmc(card) &&
			(card->ext_csd.cache_size > 0) &&
			(card->ext_csd.cache_ctrl & 1)) {
		int retries = 0;
		while (retries++ < 3) {
			u32 status;
			err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
					EXT_CSD_FLUSH_CACHE, 1, 0);
			if (err) {
				err = mmc_send_status(card, &status);
				if (err)
					break;

				switch (R1_CURRENT_STATE(status)) {
				case R1_STATE_IDLE:
				case R1_STATE_READY:
				case R1_STATE_STBY:
				case R1_STATE_TRAN:
					err = 0;
					break;
				case R1_STATE_PRG:
				default:
					err = -1;
					break;
				}
			}

			if (!err)
				break;
		}

		if (err) {
			pr_err("%s: cache flush error %d\n",
					mmc_hostname(card->host), err);
		}
	}

	return err;
}
EXPORT_SYMBOL(mmc_flush_cache);

int mmc_cache_ctrl(struct mmc_host *host, u8 enable)
{
	struct mmc_card *card = host->card;
	unsigned int timeout = card->ext_csd.generic_cmd6_time;
	int err = 0, rc;

	if (!(host->caps2 & MMC_CAP2_CACHE_CTRL) ||
			mmc_card_is_removable(host) ||
			(card->quirks & MMC_QUIRK_CACHE_DISABLE))
		return err;

	if (card && mmc_card_mmc(card) &&
			(card->ext_csd.cache_size > 0)) {
		enable = !!enable;

		if (card->ext_csd.cache_ctrl ^ enable) {
			if (!enable)
				timeout = MMC_CACHE_DISBALE_TIMEOUT_MS;

			err = mmc_switch_ignore_timeout(card,
					EXT_CSD_CMD_SET_NORMAL,
					EXT_CSD_CACHE_CTRL, enable, timeout);

			if (err == -ETIMEDOUT && !enable) {
				pr_err("%s:cache disable operation timeout\n",
						mmc_hostname(card->host));
				rc = mmc_interrupt_hpi(card);
				if (rc)
					pr_err("%s: mmc_interrupt_hpi() failed (%d)\n",
							mmc_hostname(host), rc);
			} else if (err) {
				pr_err("%s: cache %s error %d\n",
						mmc_hostname(card->host),
						enable ? "on" : "off",
						err);
			} else {
				card->ext_csd.cache_ctrl = enable;
			}
		}
	}

	return err;
}
EXPORT_SYMBOL(mmc_cache_ctrl);

#ifdef CONFIG_PM

int mmc_suspend_host(struct mmc_host *host)
{
	int err = 0;

	if (mmc_bus_needs_resume(host))
		return 0;

	mmc_bus_get(host);
	if (host->bus_ops && !host->bus_dead) {
		if (!(host->card && mmc_card_sdio(host->card)))
			if (!mmc_try_claim_host(host))
				err = -EBUSY;

		if (!err) {
			if (host->bus_ops->suspend) {
				if (mmc_is_mmc_host(host) &&
					host->card && !mmc_card_need_bkops_in_suspend(host->card)) {
					err = mmc_stop_bkops(host->card);
					if (err)
						goto stop_bkops_err;
				}
				err = host->bus_ops->suspend(host);
				if (mmc_is_mmc_host(host))
					MMC_UPDATE_BKOPS_STATS_SUSPEND(host->
							card->bkops_info.bkops_stats);
			}
			if (!(host->card && mmc_card_sdio(host->card)))
				mmc_release_host(host);

			if (err == -ENOSYS || !host->bus_ops->resume) {
				if (host->bus_ops->remove)
					host->bus_ops->remove(host);
				mmc_claim_host(host);
				mmc_detach_bus(host);
				mmc_power_off(host);
				mmc_release_host(host);
				host->pm_flags = 0;
				err = 0;
			}
		}
	}
	mmc_bus_put(host);

	if (!err && host->card && mmc_card_need_bkops_in_suspend(host->card)) {
#ifdef CONFIG_MMC_CLKGATE
		unsigned long flags;

		cancel_delayed_work_sync(&host->clk_gate_work);
		mutex_lock(&host->clk_gate_mutex);
		spin_lock_irqsave(&host->clk_lock, flags);
		if (!host->clk_gated) {
			spin_unlock_irqrestore(&host->clk_lock, flags);
			mmc_gate_clock(host);
		}
		else {
			spin_unlock_irqrestore(&host->clk_lock, flags);
		}
		mutex_unlock(&host->clk_gate_mutex);
#endif
	} else if (!err && !mmc_card_keep_power(host))
		mmc_power_off(host);

	return err;
stop_bkops_err:
	if (!(host->card && mmc_card_sdio(host->card)))
		mmc_release_host(host);
	return err;
}

EXPORT_SYMBOL(mmc_suspend_host);

int mmc_resume_host(struct mmc_host *host)
{
	int err = 0;

	mmc_bus_get(host);
	#if 0
	if (mmc_bus_manual_resume(host)) {
		host->bus_resume_flags |= MMC_BUSRESUME_NEEDS_RESUME;
		mmc_bus_put(host);
		return 0;
	}
	#endif

	if (!((host->card) && mmc_card_mmc(host->card) &&
				mmc_card_need_bkops_in_suspend(host->card))) {
		if (host->bus_ops && !host->bus_dead) {
			if (!mmc_card_keep_power(host)) {
				mmc_power_up(host);
				mmc_select_voltage(host, host->ocr);
				if (mmc_card_sdio(host->card) &&
						(host->caps & MMC_CAP_POWER_OFF_CARD)) {
					pm_runtime_disable(&host->card->dev);
					pm_runtime_set_active(&host->card->dev);
					pm_runtime_enable(&host->card->dev);
				}
			}
			BUG_ON(!host->bus_ops->resume);
			err = host->bus_ops->resume(host);
			if (err) {
				pr_warning("%s: error %d during resume "
						"(card was removed?)\n",
						mmc_hostname(host), err);
				err = 0;
			}
		}
	}
	host->pm_flags &= ~MMC_PM_KEEP_POWER;
	mmc_bus_put(host);

	return err;
}
EXPORT_SYMBOL(mmc_resume_host);

int mmc_pm_notify(struct notifier_block *notify_block,
					unsigned long mode, void *unused)
{
	struct mmc_host *host = container_of(
		notify_block, struct mmc_host, pm_notify);
	unsigned long flags;
	int err = 0;

	switch (mode) {
	case PM_HIBERNATION_PREPARE:
	case PM_SUSPEND_PREPARE:
		if (host->card && mmc_card_mmc(host->card)) {
			mmc_claim_host(host);
			err = mmc_stop_bkops(host->card);
			mmc_release_host(host);
			if (err) {
				pr_err("%s: didn't stop bkops\n",
					mmc_hostname(host));
				return err;
			}
		}

		spin_lock_irqsave(&host->lock, flags);
		if (mmc_bus_needs_resume(host)) {
			spin_unlock_irqrestore(&host->lock, flags);
			break;
		}
		spin_unlock_irqrestore(&host->lock, flags);
#if 0
		
		if (!(host->caps & MMC_CAP_NEEDS_POLL))
			flush_work(&host->detect.work);

		spin_lock_irqsave(&host->lock, flags);
		host->rescan_disable = 1;
		spin_unlock_irqrestore(&host->lock, flags);
#endif

		if (cancel_delayed_work_sync(&host->detect))
			cancel_delayed_work_sync(&host->detect);

		if (!host->bus_ops || host->bus_ops->suspend)
			break;

		
		if (host->bus_ops->remove)
			host->bus_ops->remove(host);

		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_power_off(host);
		mmc_release_host(host);
		host->pm_flags = 0;
		break;

	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:

		spin_lock_irqsave(&host->lock, flags);
		if (mmc_bus_manual_resume(host)) {
			spin_unlock_irqrestore(&host->lock, flags);
			break;
		}
		host->rescan_disable = 0;
		spin_unlock_irqrestore(&host->lock, flags);
#if 0
		mmc_detect_change(host, 0);
#endif
		break;

	default:
		return -EINVAL;
	}

	return 0;
}
#endif

#ifdef CONFIG_MMC_EMBEDDED_SDIO
void mmc_set_embedded_sdio_data(struct mmc_host *host,
				struct sdio_cis *cis,
				struct sdio_cccr *cccr,
				struct sdio_embedded_func *funcs,
				int num_funcs)
{
	host->embedded_sdio_data.cis = cis;
	host->embedded_sdio_data.cccr = cccr;
	host->embedded_sdio_data.funcs = funcs;
	host->embedded_sdio_data.num_funcs = num_funcs;
}

EXPORT_SYMBOL(mmc_set_embedded_sdio_data);
#endif

#ifdef CONFIG_PM_RUNTIME
void mmc_dump_dev_pm_state(struct mmc_host *host, struct device *dev)
{
	pr_err("%s: %s: err: runtime_error: %d\n", dev_name(dev),
	       mmc_hostname(host), dev->power.runtime_error);
	pr_err("%s: %s: disable_depth: %d runtime_status: %d idle_notification: %d\n",
	       dev_name(dev), mmc_hostname(host), dev->power.disable_depth,
	       dev->power.runtime_status,
	       dev->power.idle_notification);
	pr_err("%s: %s: request_pending: %d, request: %d\n",
	       dev_name(dev), mmc_hostname(host),
	       dev->power.request_pending, dev->power.request);
}

void mmc_rpm_hold(struct mmc_host *host, struct device *dev)
{
	int ret = 0;

	if (!mmc_use_core_runtime_pm(host))
		return;

	ret = pm_runtime_get_sync(dev);
	if ((ret < 0) &&
	    (dev->power.runtime_error || (dev->power.disable_depth > 0))) {
		pr_err("%s: %s: %s: pm_runtime_get_sync: err: %d\n",
		       dev_name(dev), mmc_hostname(host), __func__, ret);
		mmc_dump_dev_pm_state(host, dev);
		if (pm_runtime_suspended(dev))
			BUG_ON(1);
	}
}

EXPORT_SYMBOL(mmc_rpm_hold);

void mmc_rpm_release(struct mmc_host *host, struct device *dev)
{
	int ret = 0;

	if (!mmc_use_core_runtime_pm(host))
		return;

	ret = pm_runtime_put_sync(dev);
	if ((ret < 0) &&
	    (dev->power.runtime_error || (dev->power.disable_depth > 0))) {
		pr_err("%s: %s: %s: pm_runtime_put_sync: err: %d\n",
		       dev_name(dev), mmc_hostname(host), __func__, ret);
		mmc_dump_dev_pm_state(host, dev);
	}
}

EXPORT_SYMBOL(mmc_rpm_release);
#else
void mmc_rpm_hold(struct mmc_host *host, struct device *dev) {}
EXPORT_SYMBOL(mmc_rpm_hold);

void mmc_rpm_release(struct mmc_host *host, struct device *dev) {}
EXPORT_SYMBOL(mmc_rpm_release);
#endif

void mmc_init_context_info(struct mmc_host *host)
{
	spin_lock_init(&host->context_info.lock);
	host->context_info.is_new_req = false;
	host->context_info.is_done_rcv = false;
	host->context_info.is_waiting_last_req = false;
	init_waitqueue_head(&host->context_info.wait);
}

static void mmc_detect_tout_timer_hdlr(unsigned long data)
{
	pr_info("[mmc-SD] : detect time out dump start\n");
	show_state_filter(TASK_UNINTERRUPTIBLE);

	pr_info("[mmc-SD] : last claim caller %s, time : %lld\n", last_claim_comm, ktime_to_us(last_claim_getime));
	pr_info("mmc1 start to dump claim code stack\n%s\n%s\n%s\n",
	               claim_stack[0], claim_stack[1], claim_stack[2]);

	pr_info("[mmc-SD] : last release caller %s, time : %lld\n", last_release_comm, ktime_to_us(last_release_getime));
	pr_info("mmc1 start to dump release code stack\n%s\n%s\n%s\n",
	       release_stack[0], release_stack[1], release_stack[2]);

	pr_info("[mmc-SD] : detect time out dump finish \n");
	del_timer(&sd_remove_tout_timer);
}

static unsigned int logfile_prealloc_size = 1024 * 1024; 
int get_logfile_prealloc_size(void)
{
	return logfile_prealloc_size;
}

#if defined(CONFIG_DEBUG_FS)
static int
logfile_prealloc_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t logfile_prealloc_write(struct file *file, const char __user *ubuf,
		       size_t count, loff_t *ppos)
{
	sscanf(ubuf, "%u", &logfile_prealloc_size);
	if (logfile_prealloc_size > 10 * 1024 * 1024)
		logfile_prealloc_size = 10 * 1024 * 1024; 
	return count;
}

static ssize_t logfile_prealloc_read(struct file *filp, char __user *ubuf,
				size_t count, loff_t *ppos)
{
	char buf[200];
	int i;

	i = sprintf(buf, "%u", logfile_prealloc_size);
	return simple_read_from_buffer(ubuf, count, ppos, buf, i);
}

static const struct file_operations logfile_prealloc_ops = {
	.open	= logfile_prealloc_open,
	.write	= logfile_prealloc_write,
	.read	= logfile_prealloc_read,
};
#endif

extern unsigned int get_tamper_sf(void);
static int __init mmc_init(void)
{
	int ret;

	workqueue = alloc_ordered_workqueue("kmmcd", 0);
	if (!workqueue)
		return -ENOMEM;
	stats_workqueue = create_singlethread_workqueue("mmc_stats");
	if (!stats_workqueue)
		return -ENOMEM;
	if (get_tamper_sf() == 1)
		stats_interval = MMC_STATS_LOG_INTERVAL;

	wake_lock_init(&mmc_removal_work_wake_lock, WAKE_LOCK_SUSPEND,
		"mmc_removal_work");

	setup_timer(&sd_remove_tout_timer, mmc_detect_tout_timer_hdlr,
		(unsigned long)NULL);

	ret = mmc_register_bus();
	if (ret)
		goto destroy_workqueue;

	ret = mmc_register_host_class();
	if (ret)
		goto unregister_bus;

	ret = sdio_register_bus();
	if (ret)
		goto unregister_host_class;

#if defined(CONFIG_DEBUG_FS)
	debugfs_create_file("logfile_prealloc_size", 0644, 0, 0,
			&logfile_prealloc_ops);
#endif
	spin_lock_init(&iolist_lock);
	return 0;

unregister_host_class:
	mmc_unregister_host_class();
unregister_bus:
	mmc_unregister_bus();
destroy_workqueue:
	destroy_workqueue(workqueue);
	if (stats_workqueue)
		destroy_workqueue(stats_workqueue);
	wake_lock_destroy(&mmc_removal_work_wake_lock);
	return ret;
}

static void __exit mmc_exit(void)
{
	sdio_unregister_bus();
	mmc_unregister_host_class();
	mmc_unregister_bus();
	destroy_workqueue(workqueue);
	if (stats_workqueue)
		destroy_workqueue(stats_workqueue);
	wake_lock_destroy(&mmc_removal_work_wake_lock);
	del_timer(&sd_remove_tout_timer);
}

subsys_initcall(mmc_init);
module_exit(mmc_exit);

MODULE_LICENSE("GPL");
