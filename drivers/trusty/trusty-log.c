// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2015 Google, Inc.
 */
#include <linux/platform_device.h>
#include <linux/trusty/smcall.h>
#include <linux/trusty/trusty.h>
#include <linux/notifier.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/log2.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/seq_file.h>
#include <asm/page.h>
#include "trusty-log.h"

/*
 * Rationale for the chosen default log buffer size:
 *  - the log buffer shall contain unthrottled Trusty crash dump.
 *  - the register list portion of a crash dump is about 1KB
 *  - the memory-around-registers portion of a crash dump can be up to 12 KB
 *  - an average size backtrace is about 1 KB
 *  - average length of non-crash trusty logs during boot is about 85 characters
 *  - a crash dump with 50 lines of context therefore requires up to 18 KB
 *  - buffer size needs to be power-of-two number of bytes
 *  - rounding up to power of two from 18 KB gives 32 KB
 *  The log size can be adjusted by setting the "trusty_log.log_size" parameter
 *  on the kernel command line. The specified value will be adjusted as needed.
 */

#define TRUSTY_LOG_DEFAULT_SIZE (32768)
#define TRUSTY_LOG_MIN_SIZE (PAGE_SIZE / 2)
#define TRUSTY_LOG_MAX_SIZE (1 * 1024 * 1024 * 1024)
#define TRUSTY_LINE_BUFFER_SIZE (256)

static size_t log_size_param = TRUSTY_LOG_DEFAULT_SIZE;

static int trusty_log_size_set(const char *val, const struct kernel_param *kp)
{
	unsigned long long requested = memparse(val, NULL);

	if (requested < TRUSTY_LOG_MIN_SIZE)
		requested = TRUSTY_LOG_MIN_SIZE;
	if (requested > TRUSTY_LOG_MAX_SIZE)
		requested = TRUSTY_LOG_MAX_SIZE;
	requested = rounddown_pow_of_two(requested);
	log_size_param = requested;
	return 0;
}

static int trusty_log_size_get(char *buffer, const struct kernel_param *kp)
{
	return scnprintf(buffer, PAGE_SIZE, "%zu\n", log_size_param);
}

module_param_call(log_size, trusty_log_size_set, trusty_log_size_get, NULL,
		  0644);

/*
 * The "log_to_dmesg" parameter can have three values: "never", "always",
 * and "until_first_reader". "never" indicates Trusty logs will never be
 * copied to the linux kernel log, while "always" indicates they will always
 * be copied to the linux kernel log. In both cases Trusty logs can still
 * be read from the /dev/trusty-logX virtual file. In the case of "always"
 * that may mean logs show up duplicated in logcat.
 * The third option, "until_first_reader", copies Trusty logs to the linux
 * kernel log, but only until /dev/trusty-logX is first opened. After that
 * Trusty logs will no longer be copied to the kernel log and are only
 * available from /dev/trusty-logX.
 */

enum log_to_dmesg_options {
	NEVER,
	ALWAYS,
	UNTIL_FIRST_READER
};

static const char * const log_to_dmesg_opt_names[] = {
	"never", "always", "until_first_reader"
};

static int log_to_dmesg_param = NEVER;

static int trusty_log_mode_set(const char *val, const struct kernel_param *kp)
{
	int i = sysfs_match_string(log_to_dmesg_opt_names, val);
	if (i < 0)
		return i;

	log_to_dmesg_param = i;
	return 0;
}

static int trusty_log_mode_get(char *buffer, const struct kernel_param *kp)
{
	int i;

	/*
	 * Output buffer is PAGE_SIZE, of which we'll use only 35 bytes,
	 * so bounds checks are not necessary in the following code.
	 */
	*buffer = 0;
	for (i = 0; i < ARRAY_SIZE(log_to_dmesg_opt_names); i++) {
		if (log_to_dmesg_param == i)
			strcat(buffer, "[");
		strcat(buffer, log_to_dmesg_opt_names[i]);
		if (log_to_dmesg_param == i)
			strcat(buffer, "]");
		strcat(buffer, " ");
	}
	strcat(buffer, "\n");
	return strlen(buffer);
}

module_param_call(log_to_dmesg, trusty_log_mode_set, trusty_log_mode_get, NULL,
		  0644);
/*
 * If we log too much and a UART or other slow source is connected, we can stall
 * out another thread which is doing printk.
 *
 * Trusty crash logs are currently ~16 lines, so 100 should include context and
 * the crash most of the time.
 */
static struct ratelimit_state trusty_log_rate_limit =
	RATELIMIT_STATE_INIT("trusty_log", 1 * HZ, 100);

module_param_named(log_ratelimit_burst, trusty_log_rate_limit.burst, int, 0644);
module_param_named(log_ratelimit_interval, trusty_log_rate_limit.interval, int,
		   0644);

/**
 * struct trusty_log_sfile - trusty log misc device state
 *
 * @misc:          misc device created for the trusty log virtual file
 * @device_name:   misc device name following the convention
 *                 "trusty-<name><id>"
 */
struct trusty_log_sfile {
	struct miscdevice misc;
	char device_name[64];
};

/**
 * struct trusty_log_sink_state - trusty log sink state
 *
 * @get:              current read unwrapped index
 * @last_successful_next:
 *                    index for the next line after the last successful get
 * @trusty_panicked:  trusty panic status at the start of the sink interation
 *                    (only used for kernel log sink)
 * @sfile:            seq_file used for sinking to a virtual file (misc device);
 *                    set to NULL for the kernel log sink.
 * @ignore_overflow:  ignore_overflow used to coalesce overflow messages and
 *                    avoid reporting an overflow when sinking the oldest
 *                    line to the virtual file (only used for virtual file sink)
 *
 * A sink state structure is used for both the kernel log sink
 * and the virtual device sink.
 * An instance of the sink state structure is dynamically created
 * for each read iteration of the trusty log virtual file (misc device).
 *
 */
struct trusty_log_sink_state {
	u32 get;
	u32 last_successful_next;
	bool trusty_panicked;

	/* virtual file sink specific attributes */
	struct seq_file *sfile;
	bool ignore_overflow;
};

struct trusty_log_state {
	struct device *dev;
	struct device *trusty_dev;
	struct trusty_log_sfile log_sfile;
	struct kref refcount;

	/*
	 * This lock is here to ensure only one consumer will read
	 * from the log ring buffer at a time.
	 */
	spinlock_t lock;
	struct log_rb *log;
	struct trusty_log_sink_state klog_sink;

	u32 log_num_pages;
	struct scatterlist *sg;
	trusty_shared_mem_id_t log_pages_shared_mem_id;

	struct notifier_block call_notifier;
	struct notifier_block panic_notifier;
	char line_buffer[TRUSTY_LINE_BUFFER_SIZE];
	wait_queue_head_t poll_waiters;
	/* this lock protects access to wake_put */
	spinlock_t wake_up_lock;
	u32 last_wake_put;
	bool have_first_reader;
	bool registered;
};

static inline u32 u32_add_overflow(u32 a, u32 b)
{
	u32 d;

	if (check_add_overflow(a, b, &d)) {
		/*
		 * silence the overflow,
		 * what matters in the log buffer context
		 * is the casted addition
		 */
	}
	return d;
}

static inline u32 u32_sub_overflow(u32 a, u32 b)
{
	u32 d;

	if (check_sub_overflow(a, b, &d)) {
		/*
		 * silence the overflow,
		 * what matters in the log buffer context
		 * is the casted substraction
		 */
	}
	return d;
}

static int log_read_line(struct trusty_log_state *s, u32 put, u32 get)
{
	struct log_rb *log = s->log;
	int i;
	char c = '\0';
	size_t max_to_read =
		min_t(size_t,
		      u32_sub_overflow(put, get),
		      sizeof(s->line_buffer) - 1);
	size_t mask = log->sz - 1;

	for (i = 0; i < max_to_read && c != '\n';) {
		c = log->data[get & mask];
		s->line_buffer[i++] = c;
		get = u32_add_overflow(get, 1);
	}
	s->line_buffer[i] = '\0';

	return i;
}

/**
 * trusty_log_has_data() - returns true when more data is available to sink
 * @s:         Current log state.
 * @sink:      trusty_log_sink_state holding the get index on a given sink
 *
 * Return: true if data is available.
 */
static bool trusty_log_has_data(struct trusty_log_state *s,
				struct trusty_log_sink_state *sink)
{
	struct log_rb *log = s->log;

	return (log->put != sink->get);
}

/**
 * trusty_log_start() - initialize the sink iteration either to kernel log
 * or to secondary log_sfile
 * @s:         Current log state.
 * @sink:      trusty_log_sink_state holding the get index on a given sink
 * @index:     Unwrapped ring buffer index from where iteration shall start
 *
 * Return: 0 if successful, negative error code otherwise
 */
static int trusty_log_start(struct trusty_log_state *s,
			    struct trusty_log_sink_state *sink,
			    u32 index)
{
	struct log_rb *log;

	if (WARN_ON(!s))
		return -EINVAL;

	log = s->log;
	if (WARN_ON(!is_power_of_2(log->sz)))
		return -EINVAL;

	sink->get = index;
	return 0;
}

/**
 * trusty_log_show() - sink log entry at current iteration
 * @s:         Current log state.
 * @sink:      trusty_log_sink_state holding the get index on a given sink
 */
static void trusty_log_show(struct trusty_log_state *s,
			    struct trusty_log_sink_state *sink)
{
	struct log_rb *log = s->log;
	u32 alloc, put, get;
	int read_chars;

	if (sink->sfile && sink == &s->klog_sink)
		dev_warn(s->dev, "klog_sink has seq_file\n");

	/*
	 * For this ring buffer, at any given point, alloc >= put >= get.
	 * The producer side of the buffer is not locked, so the put and alloc
	 * pointers must be read in a defined order (put before alloc) so
	 * that the above condition is maintained. A read barrier is needed
	 * to make sure the hardware and compiler keep the reads ordered.
	 */
	get = sink->get;
	put = log->put;

	/* Make sure that the read of put occurs before the read of log data */
	rmb();

	/* Read a line from the log */
	read_chars = log_read_line(s, put, get);

	/* Force the loads from log_read_line to complete. */
	rmb();
	alloc = log->alloc;

	/*
	 * Discard the line that was just read if the data could
	 * have been corrupted by the producer.
	 */
	if (u32_sub_overflow(alloc, get) > log->sz) {
		/*
		 * this condition is acceptable in the case of the sfile sink
		 * when attempting to read the oldest entry (at alloc-log->sz)
		 * which may be overrun by a new one when ring buffer write
		 * index wraps around.
		 * So the overrun is not reported in case the oldest line
		 * was being read.
		 */
		if (sink->sfile) {
			if (!sink->ignore_overflow)
				seq_puts(sink->sfile, "log overflow.\n");
			/* coalesce subsequent contiguous overflows. */
			sink->ignore_overflow = true;
		} else {
			dev_err(s->dev, "log overflow.\n");
		}
		sink->get = u32_sub_overflow(alloc, log->sz);
		return;
	}
	/* compute next line index */
	sink->get = u32_add_overflow(get, read_chars);
	/* once a line is valid, ignore_overflow must be disabled */
	sink->ignore_overflow = false;
	if (sink->sfile) {
		seq_printf(sink->sfile, "%s", s->line_buffer);
	} else {
		if (sink->trusty_panicked ||
		    __ratelimit(&trusty_log_rate_limit)) {
			dev_info(s->dev, "%s", s->line_buffer);
			/* next line after last successful get */
			sink->last_successful_next = sink->get;
		}
	}
}

static void *trusty_log_seq_start(struct seq_file *sfile, loff_t *pos)
{
	struct trusty_log_sfile *lb;
	struct trusty_log_state *s;
	struct log_rb *log;
	struct trusty_log_sink_state *log_sfile_sink;
	u32 index;
	int rc;

	if (WARN_ON(!pos))
		return ERR_PTR(-EINVAL);

	lb = sfile->private;
	if (WARN_ON(!lb))
		return ERR_PTR(-EINVAL);

	log_sfile_sink = kzalloc(sizeof(*log_sfile_sink), GFP_KERNEL);
	if (!log_sfile_sink)
		return ERR_PTR(-ENOMEM);

	s = container_of(lb, struct trusty_log_state, log_sfile);
	log_sfile_sink->sfile = sfile;
	log = s->log;
	if (*pos == 0) {
		/* start at the oldest line */
		index = 0;
		if (log->alloc > log->sz)
			index = u32_sub_overflow(log->alloc, log->sz);
	} else {
		/*
		 * '*pos>0': pos hold the 32bits unwrapped index from where
		 * to start iterating
		 */
		index = (u32)*pos;
	}
	pr_debug("%s start=%u\n", __func__, index);

	log_sfile_sink->ignore_overflow = true;
	rc = trusty_log_start(s, log_sfile_sink, index);
	if (rc < 0)
		goto free_sink;

	if (!trusty_log_has_data(s, log_sfile_sink))
		goto free_sink;

	return log_sfile_sink;

free_sink:
	pr_debug("%s kfree\n", __func__);
	kfree(log_sfile_sink);
	return rc < 0 ? ERR_PTR(rc) : NULL;
}

static void *trusty_log_seq_next(struct seq_file *sfile, void *v, loff_t *pos)
{
	struct trusty_log_sfile *lb;
	struct trusty_log_state *s;
	struct trusty_log_sink_state *log_sfile_sink = v;
	int rc = 0;

	if (WARN_ON(!log_sfile_sink))
		return ERR_PTR(-EINVAL);

	lb = sfile->private;
	if (WARN_ON(!lb)) {
		rc = -EINVAL;
		goto end_of_iter;
	}
	s = container_of(lb, struct trusty_log_state, log_sfile);

	if (WARN_ON(!pos)) {
		rc = -EINVAL;
		goto end_of_iter;
	}
	/*
	 * When starting a virtual file sink, the start function is invoked
	 * with a pos argument which value is set to zero.
	 * Subsequent starts are invoked with pos being set to
	 * the unwrapped read index (get).
	 * Upon u32 wraparound, the get index could be reset to zero.
	 * Thus a msb is used to distinguish the `get` zero value
	 * from the `start of file` zero value.
	 */
	*pos = (1UL << 32) + log_sfile_sink->get;
	if (!trusty_log_has_data(s, log_sfile_sink))
		goto end_of_iter;

	return log_sfile_sink;

end_of_iter:
	pr_debug("%s kfree\n", __func__);
	kfree(log_sfile_sink);
	return rc < 0 ? ERR_PTR(rc) : NULL;
}

static void trusty_log_seq_stop(struct seq_file *sfile, void *v)
{
	/*
	 * When iteration completes or on error, the next callback frees
	 * the sink structure and returns NULL/error-code.
	 * In that case stop (being invoked with void* v set to the last next
	 * return value) would be invoked with v == NULL or error code.
	 * When user space stops the iteration earlier than the end
	 * (in case of user-space memory allocation limit for example)
	 * then the stop function receives a non NULL get pointer
	 * and is in charge or freeing the sink structure.
	 */
	struct trusty_log_sink_state *log_sfile_sink = v;

	/* nothing to do - sink structure already freed */
	if (IS_ERR_OR_NULL(log_sfile_sink))
		return;

	kfree(log_sfile_sink);

	pr_debug("%s kfree\n", __func__);
}

static int trusty_log_seq_show(struct seq_file *sfile, void *v)
{
	struct trusty_log_sfile *lb;
	struct trusty_log_state *s;
	struct trusty_log_sink_state *log_sfile_sink = v;

	if (WARN_ON(!log_sfile_sink))
		return -EINVAL;

	lb = sfile->private;
	if (WARN_ON(!lb))
		return -EINVAL;

	s = container_of(lb, struct trusty_log_state, log_sfile);

	trusty_log_show(s, log_sfile_sink);
	return 0;
}

static void trusty_dump_logs(struct trusty_log_state *s)
{
	u32 start;
	int rc;
	/*
	 * note: klopg_sink.get and last_successful_next
	 * initialized to zero by kzalloc
	 */
	s->klog_sink.trusty_panicked = trusty_get_panic_status(s->trusty_dev);

	start = s->klog_sink.trusty_panicked ?
			s->klog_sink.last_successful_next :
			s->klog_sink.get;
	rc = trusty_log_start(s, &s->klog_sink, start);
	if (rc < 0)
		return;

	while (trusty_log_has_data(s, &s->klog_sink))
		trusty_log_show(s, &s->klog_sink);
}

static int trusty_log_call_notify(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	struct trusty_log_state *s;
	unsigned long flags;
	u32 cur_put;

	if (action != TRUSTY_CALL_RETURNED)
		return NOTIFY_DONE;

	s = container_of(nb, struct trusty_log_state, call_notifier);
	spin_lock_irqsave(&s->wake_up_lock, flags);
	cur_put = s->log->put;
	if (cur_put != s->last_wake_put) {
		s->last_wake_put = cur_put;
		wake_up_all(&s->poll_waiters);
	}
	spin_unlock_irqrestore(&s->wake_up_lock, flags);
	if (log_to_dmesg_param == ALWAYS ||
	    (log_to_dmesg_param == UNTIL_FIRST_READER &&
	     !s->have_first_reader)) {
		spin_lock_irqsave(&s->lock, flags);
		trusty_dump_logs(s);
		spin_unlock_irqrestore(&s->lock, flags);
	}
	return NOTIFY_OK;
}

static int trusty_log_panic_notify(struct notifier_block *nb,
				   unsigned long action, void *data)
{
	struct trusty_log_state *s;

	/*
	 * Don't grab the spin lock to hold up the panic notifier, even
	 * though this is racy.
	 */
	s = container_of(nb, struct trusty_log_state, panic_notifier);
	dev_info(s->dev, "panic notifier - trusty version %s",
		 trusty_version_str_get(s->trusty_dev));
	trusty_dump_logs(s);
	return NOTIFY_OK;
}

const struct seq_operations trusty_log_seq_ops = {
	.start = trusty_log_seq_start,
	.stop = trusty_log_seq_stop,
	.next = trusty_log_seq_next,
	.show = trusty_log_seq_show,
};

static int trusty_log_sfile_dev_open(struct inode *inode, struct file *file)
{
	struct trusty_log_sfile *ls;
	struct trusty_log_state *s;
	struct seq_file *sfile;
	int rc;

	/*
	 * file->private_data contains a pointer to the misc_device struct
	 * passed to misc_register()
	 */
	if (WARN_ON(!file->private_data))
		return -EINVAL;

	ls = container_of(file->private_data, struct trusty_log_sfile, misc);

	/*
	 * seq_open uses file->private_data to store the seq_file associated
	 * with the struct file, but it must be NULL when seq_open is called
	 */
	file->private_data = NULL;
	rc = seq_open(file, &trusty_log_seq_ops);
	if (rc < 0)
		return rc;

	sfile = file->private_data;
	if (WARN_ON(!sfile))
		return -EINVAL;

	sfile->private = ls;
	s = container_of(ls, struct trusty_log_state, log_sfile);
	s->have_first_reader = true;
	kref_get(&s->refcount);
	return 0;
}

static unsigned int trusty_log_sfile_dev_poll(struct file *filp,
					      struct poll_table_struct *wait)
{
	struct seq_file *sfile;
	struct trusty_log_sfile *lb;
	struct trusty_log_state *s;
	struct log_rb *log;

	/*
	 * trusty_log_sfile_dev_open() pointed filp->private_data to a
	 * seq_file, and that seq_file->private to the trusty_log_sfile
	 * field of a trusty_log_state
	 */
	sfile = filp->private_data;
	lb = sfile->private;
	s = container_of(lb, struct trusty_log_state, log_sfile);

	if (!s->registered) {
		dev_err(s->dev, "invalid poll fd\n");
		return -EINVAL;
	}

	poll_wait(filp, &s->poll_waiters, wait);
	log = s->log;

	/*
	 * Userspace has read up to sfile->index so far. Update klog_sink
	 * to indicate that, so that we don't end up dumping the entire
	 * Trusty log in case of panic. Only do this when not logging to
	 * klog_sink, since logging to klog_sink already updates this.
	 */
	if (log_to_dmesg_param != ALWAYS)
		s->klog_sink.last_successful_next = (u32)sfile->index;

	if (log->put != (u32)sfile->index) {
		/* data ready to read */
		return EPOLLIN | EPOLLRDNORM;
	}
	/* no data available, go to sleep */
	return 0;
}

static void trusty_log_cleanup(struct kref *ref);

static int trusty_log_sfile_dev_release(struct inode *inode,
					struct file *filp)
{
	struct seq_file *sfile;
	struct trusty_log_sfile *lb;
	struct trusty_log_state *s;

	sfile = filp->private_data;
	lb = sfile->private;
	s = container_of(lb, struct trusty_log_state, log_sfile);

	kref_put(&s->refcount, trusty_log_cleanup);

	seq_release(inode, filp);
	return 0;
}

ssize_t trusty_log_sfile_dev_read(struct file *filp, char __user *buf,
				  size_t size, loff_t *ppos)
{
	struct seq_file *sfile;
	struct trusty_log_sfile *lb;
	struct trusty_log_state *s;

	sfile = filp->private_data;
	lb = sfile->private;
	s = container_of(lb, struct trusty_log_state, log_sfile);

	if (!s->registered) {
		dev_err(s->dev, "invalid read fd\n");
		return -EINVAL;
	}

	return seq_read(filp, buf, size, ppos);
}

static const struct file_operations log_sfile_dev_operations = {
	.owner = THIS_MODULE,
	.open = trusty_log_sfile_dev_open,
	.poll = trusty_log_sfile_dev_poll,
	.read = trusty_log_sfile_dev_read,
	.release = trusty_log_sfile_dev_release,
};

static int trusty_log_sfile_register(struct trusty_log_state *s)
{
	int ret;
	struct trusty_log_sfile *ls = &s->log_sfile;

	if (WARN_ON(!ls))
		return -EINVAL;

	snprintf(ls->device_name, sizeof(ls->device_name),
		 "trusty-log%d", s->dev->id);
	ls->misc.minor = MISC_DYNAMIC_MINOR;
	ls->misc.name = ls->device_name;
	ls->misc.fops = &log_sfile_dev_operations;

	ret = misc_register(&ls->misc);
	if (ret) {
		dev_err(s->dev,
			"log_sfile error while doing misc_register ret=%d\n",
			ret);
		return ret;
	}
	s->registered = true;
	dev_info(s->dev, "/dev/%s registered\n",
		 ls->device_name);
	return 0;
}

static void trusty_log_sfile_unregister(struct trusty_log_state *s)
{
	struct trusty_log_sfile *ls = &s->log_sfile;

	misc_deregister(&ls->misc);
	if (s->dev) {
		dev_info(s->dev, "/dev/%s unregistered\n",
			 ls->misc.name);
		s->registered = false;
	}
}

static bool trusty_supports_logging(struct device *device)
{
	int result;

	result = trusty_std_call32(device, SMC_SC_SHARED_LOG_VERSION,
				   TRUSTY_LOG_API_VERSION, 0, 0);
	if (result == SM_ERR_UNDEFINED_SMC) {
		dev_info(device, "trusty-log not supported on secure side.\n");
		return false;
	} else if (result < 0) {
		dev_err(device,
			"trusty std call (SMC_SC_SHARED_LOG_VERSION) failed: %d\n",
			result);
		return false;
	}

	if (result != TRUSTY_LOG_API_VERSION) {
		dev_info(device, "unsupported api version: %d, supported: %d\n",
			 result, TRUSTY_LOG_API_VERSION);
		return false;
	}
	return true;
}

static int trusty_log_init(struct platform_device *pdev)
{
	struct trusty_log_state *s;
	struct scatterlist *sg;
	unsigned char *mem;
	int i;
	int result;
	trusty_shared_mem_id_t mem_id;
	int log_size;

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s) {
		result = -ENOMEM;
		goto error_alloc_state;
	}

	spin_lock_init(&s->lock);
	s->dev = &pdev->dev;
	s->trusty_dev = s->dev->parent;

	s->log_num_pages = DIV_ROUND_UP(log_size_param + sizeof(struct log_rb),
					PAGE_SIZE);
	s->sg = kcalloc(s->log_num_pages, sizeof(*s->sg), GFP_KERNEL);
	if (!s->sg) {
		result = -ENOMEM;
		goto error_alloc_sg;
	}

	log_size = s->log_num_pages * PAGE_SIZE;
	mem = vzalloc(log_size);
	if (!mem) {
		result = -ENOMEM;
		goto error_alloc_log;
	}

	s->log = (struct log_rb *)mem;

	sg_init_table(s->sg, s->log_num_pages);
	for_each_sg(s->sg, sg, s->log_num_pages, i) {
		struct page *pg = vmalloc_to_page(mem + (i * PAGE_SIZE));

		if (!pg) {
			result = -ENOMEM;
			goto err_share_memory;
		}
		sg_set_page(sg, pg, PAGE_SIZE, 0);
	}
	/*
	 * This will fail for Trusty api version < TRUSTY_API_VERSION_MEM_OBJ
	 * if s->log_num_pages > 1
	 * Use trusty_share_memory_compat instead of trusty_share_memory in case
	 * s->log_num_pages == 1 and api version < TRUSTY_API_VERSION_MEM_OBJ,
	 * In that case SMC_SC_SHARED_LOG_ADD expects a different value than
	 * what trusty_share_memory returns
	 */
	result = trusty_share_memory_compat(s->trusty_dev, &mem_id, s->sg,
					    s->log_num_pages, PAGE_KERNEL);
	if (result) {
		dev_err(s->dev, "trusty_share_memory failed: %d\n", result);
		goto err_share_memory;
	}
	s->log_pages_shared_mem_id = mem_id;

	result = trusty_std_call32(s->trusty_dev,
				   SMC_SC_SHARED_LOG_ADD,
				   (u32)(mem_id), (u32)(mem_id >> 32),
				   log_size);
	if (result < 0) {
		dev_err(s->dev,
			"trusty std call (SMC_SC_SHARED_LOG_ADD) failed: %d 0x%llx\n",
			result, mem_id);
		goto error_std_call;
	}

	init_waitqueue_head(&s->poll_waiters);
	spin_lock_init(&s->wake_up_lock);

	s->call_notifier.notifier_call = trusty_log_call_notify;
	result = trusty_call_notifier_register(s->trusty_dev,
					       &s->call_notifier);
	if (result < 0) {
		dev_err(&pdev->dev,
			"failed to register trusty call notifier\n");
		goto error_call_notifier;
	}

	s->panic_notifier.notifier_call = trusty_log_panic_notify;
	result = atomic_notifier_chain_register(&panic_notifier_list,
						&s->panic_notifier);
	if (result < 0) {
		dev_err(&pdev->dev,
			"failed to register panic notifier\n");
		goto error_panic_notifier;
	}

	result = trusty_log_sfile_register(s);
	if (result < 0) {
		dev_err(&pdev->dev, "failed to register log_sfile\n");
		goto error_log_sfile;
	}

	kref_init(&s->refcount);
	platform_set_drvdata(pdev, s);

	return 0;

error_log_sfile:
	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &s->panic_notifier);
error_panic_notifier:
	trusty_call_notifier_unregister(s->trusty_dev, &s->call_notifier);
error_call_notifier:
	trusty_std_call32(s->trusty_dev, SMC_SC_SHARED_LOG_RM,
			  (u32)mem_id, (u32)(mem_id >> 32), 0);
error_std_call:
	if (WARN_ON(trusty_reclaim_memory(s->trusty_dev, mem_id, s->sg,
					  s->log_num_pages))) {
		dev_err(&pdev->dev, "trusty_revoke_memory failed: %d 0x%llx\n",
			result, mem_id);
		/*
		 * It is not safe to free this memory if trusty_revoke_memory
		 * fails. Leak it in that case.
		 */
	} else {
err_share_memory:
		vfree(s->log);
	}
error_alloc_log:
	kfree(s->sg);
error_alloc_sg:
	kfree(s);
error_alloc_state:
	return result;
}

static int trusty_log_probe(struct platform_device *pdev)
{
	int rc;

	if (!trusty_supports_logging(pdev->dev.parent))
		return -ENXIO;

	rc = trusty_log_init(pdev);
	if (rc && log_size_param > TRUSTY_LOG_MIN_SIZE) {
		dev_warn(&pdev->dev, "init failed, retrying with 1-page log\n");
		log_size_param = TRUSTY_LOG_MIN_SIZE;
		rc = trusty_log_init(pdev);
	}
	return rc;
}

static int trusty_log_remove(struct platform_device *pdev)
{
	struct trusty_log_state *s = platform_get_drvdata(pdev);

	trusty_log_sfile_unregister(s);
	kref_put(&s->refcount, trusty_log_cleanup);
	return 0;
}

static void trusty_log_cleanup(struct kref *ref)
{
	int result;
	struct trusty_log_state *s;
	trusty_shared_mem_id_t mem_id;

	s = container_of(ref, struct trusty_log_state, refcount);
	dev_info(s->dev, "log_cleanup\n");
	mem_id = s->log_pages_shared_mem_id;

	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &s->panic_notifier);
	trusty_call_notifier_unregister(s->trusty_dev, &s->call_notifier);

	result = trusty_std_call32(s->trusty_dev, SMC_SC_SHARED_LOG_RM,
				   (u32)mem_id, (u32)(mem_id >> 32), 0);
	if (result) {
		dev_err(s->dev,
			"trusty std call (SMC_SC_SHARED_LOG_RM) failed: %d\n",
			result);
	}
	result = trusty_reclaim_memory(s->trusty_dev, mem_id, s->sg,
				       s->log_num_pages);
	if (WARN_ON(result)) {
		dev_err(s->dev,
			"trusty failed to remove shared memory: %d\n", result);
	} else {
		/*
		 * It is not safe to free this memory if trusty_revoke_memory
		 * fails. Leak it in that case.
		 */
		vfree(s->log);
	}
	kfree(s->sg);
	kfree(s);
}

static const struct of_device_id trusty_test_of_match[] = {
	{ .compatible = "android,trusty-log-v1", },
	{},
};

MODULE_DEVICE_TABLE(trusty, trusty_test_of_match);

static struct platform_driver trusty_log_driver = {
	.probe = trusty_log_probe,
	.remove = trusty_log_remove,
	.driver = {
		.name = "trusty-log",
		.of_match_table = trusty_test_of_match,
	},
};

module_platform_driver(trusty_log_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Trusty logging driver");
