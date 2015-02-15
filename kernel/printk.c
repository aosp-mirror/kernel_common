/*
 *  linux/kernel/printk.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 * Modified to make sys_syslog() more flexible: added commands to
 * return the last 4k of kernel messages, regardless of whether
 * they've been read or not.  Added option to suppress kernel printk's
 * to the console.  Added hook for sending the console messages
 * elsewhere, in preparation for a serial line console (someday).
 * Ted Ts'o, 2/11/93.
 * Modified for sysctl support, 1/8/97, Chris Horn.
 * Fixed SMP synchronization, 08/08/99, Manfred Spraul
 *     manfred@colorfullife.com
 * Rewrote bits to get rid of console_lock
 *	01Mar01 Andrew Morton
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/console.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/nmi.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>			
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/security.h>
#include <linux/bootmem.h>
#include <linux/memblock.h>
#include <linux/syscalls.h>
#include <linux/kexec.h>
#include <linux/kdb.h>
#include <linux/ratelimit.h>
#include <linux/kmsg_dump.h>
#include <linux/syslog.h>
#include <linux/cpu.h>
#include <linux/notifier.h>
#include <linux/rculist.h>

#include <asm/uaccess.h>

#include <mach/msm_rtb.h>
#define CREATE_TRACE_POINTS
#include <trace/events/printk.h>

void asmlinkage __attribute__((weak)) early_printk(const char *fmt, ...)
{
}

#define __LOG_BUF_LEN	(1 << CONFIG_LOG_BUF_SHIFT)

#define DEFAULT_MESSAGE_LOGLEVEL CONFIG_DEFAULT_MESSAGE_LOGLEVEL

#define MINIMUM_CONSOLE_LOGLEVEL 1 
#define DEFAULT_CONSOLE_LOGLEVEL 7 

DECLARE_WAIT_QUEUE_HEAD(log_wait);

int console_printk[4] = {
	DEFAULT_CONSOLE_LOGLEVEL,	
	DEFAULT_MESSAGE_LOGLEVEL,	
	MINIMUM_CONSOLE_LOGLEVEL,	
	DEFAULT_CONSOLE_LOGLEVEL,	
};

int oops_in_progress;
EXPORT_SYMBOL(oops_in_progress);

static DEFINE_SEMAPHORE(console_sem);
struct console *console_drivers;
EXPORT_SYMBOL_GPL(console_drivers);

static int console_locked, console_suspended;

static DEFINE_RAW_SPINLOCK(logbuf_lock);

#define LOG_BUF_MASK (log_buf_len-1)
#define LOG_BUF(idx) (log_buf[(idx) & LOG_BUF_MASK])

static unsigned log_start;	
static unsigned con_start;	
static unsigned log_end;	/* Index into log_buf: most-recently-written-char + 1 */

static struct console *exclusive_console;

struct console_cmdline
{
	char	name[8];			
	int	index;				
	char	*options;			
#ifdef CONFIG_A11Y_BRAILLE_CONSOLE
	char	*brl_options;			
#endif
};

#define MAX_CMDLINECONSOLES 8

static struct console_cmdline console_cmdline[MAX_CMDLINECONSOLES];
static int selected_console = -1;
static int preferred_console = -1;
int console_set_on_cmdline;
EXPORT_SYMBOL(console_set_on_cmdline);

static int console_may_schedule;

#ifdef CONFIG_PRINTK

static char __log_buf[__LOG_BUF_LEN];
static char *log_buf = __log_buf;
static int log_buf_len = __LOG_BUF_LEN;
static unsigned logged_chars; 
static int saved_console_loglevel = -1;

#ifdef CONFIG_KEXEC
void log_buf_kexec_setup(void)
{
	VMCOREINFO_SYMBOL(log_buf);
	VMCOREINFO_SYMBOL(log_end);
	VMCOREINFO_SYMBOL(log_buf_len);
	VMCOREINFO_SYMBOL(logged_chars);
}
#endif

static unsigned long __initdata new_log_buf_len;

static int __init log_buf_len_setup(char *str)
{
	unsigned size = memparse(str, &str);

	if (size)
		size = roundup_pow_of_two(size);
	if (size > log_buf_len)
		new_log_buf_len = size;

	return 0;
}
early_param("log_buf_len", log_buf_len_setup);

void __init setup_log_buf(int early)
{
	unsigned long flags;
	unsigned start, dest_idx, offset;
	char *new_log_buf;
	int free;

	if (!new_log_buf_len)
		return;

	if (early) {
		unsigned long mem;

		mem = memblock_alloc(new_log_buf_len, PAGE_SIZE);
		if (!mem)
			return;
		new_log_buf = __va(mem);
	} else {
		new_log_buf = alloc_bootmem_nopanic(new_log_buf_len);
	}

	if (unlikely(!new_log_buf)) {
		pr_err("log_buf_len: %ld bytes not available\n",
			new_log_buf_len);
		return;
	}

	raw_spin_lock_irqsave(&logbuf_lock, flags);
	log_buf_len = new_log_buf_len;
	log_buf = new_log_buf;
	new_log_buf_len = 0;
	free = __LOG_BUF_LEN - log_end;

	offset = start = min(con_start, log_start);
	dest_idx = 0;
	while (start != log_end) {
		unsigned log_idx_mask = start & (__LOG_BUF_LEN - 1);

		log_buf[dest_idx] = __log_buf[log_idx_mask];
		start++;
		dest_idx++;
	}
	log_start -= offset;
	con_start -= offset;
	log_end -= offset;
	raw_spin_unlock_irqrestore(&logbuf_lock, flags);

	pr_info("log_buf_len: %d\n", log_buf_len);
	pr_info("early log buf free: %d(%d%%)\n",
		free, (free * 100) / __LOG_BUF_LEN);
}

#ifdef CONFIG_BOOT_PRINTK_DELAY

static int boot_delay; 
static unsigned long long loops_per_msec;	

static int __init boot_delay_setup(char *str)
{
	unsigned long lpj;

	lpj = preset_lpj ? preset_lpj : 1000000;	
	loops_per_msec = (unsigned long long)lpj / 1000 * HZ;

	get_option(&str, &boot_delay);
	if (boot_delay > 10 * 1000)
		boot_delay = 0;

	pr_debug("boot_delay: %u, preset_lpj: %ld, lpj: %lu, "
		"HZ: %d, loops_per_msec: %llu\n",
		boot_delay, preset_lpj, lpj, HZ, loops_per_msec);
	return 1;
}
__setup("boot_delay=", boot_delay_setup);

static void boot_delay_msec(void)
{
	unsigned long long k;
	unsigned long timeout;

	if (boot_delay == 0 || system_state != SYSTEM_BOOTING)
		return;

	k = (unsigned long long)loops_per_msec * boot_delay;

	timeout = jiffies + msecs_to_jiffies(boot_delay);
	while (k) {
		k--;
		cpu_relax();
		if (time_after(jiffies, timeout))
			break;
		touch_nmi_watchdog();
	}
}
#else
static inline void boot_delay_msec(void)
{
}
#endif

static int log_buf_get_len(void)
{
	return logged_chars;
}

void log_buf_clear(void)
{
	logged_chars = 0;
}

int log_buf_copy(char *dest, int idx, int len)
{
	int ret, max;
	bool took_lock = false;

	if (!oops_in_progress) {
		raw_spin_lock_irq(&logbuf_lock);
		took_lock = true;
	}

	max = log_buf_get_len();
	if (idx < 0 || idx >= max) {
		ret = -1;
	} else {
		if (len > max - idx)
			len = max - idx;
		ret = len;
		idx += (log_end - max);
		while (len-- > 0)
			dest[len] = LOG_BUF(idx + len);
	}

	if (took_lock)
		raw_spin_unlock_irq(&logbuf_lock);

	return ret;
}

#ifdef CONFIG_SECURITY_DMESG_RESTRICT
int dmesg_restrict = 1;
#else
int dmesg_restrict;
#endif

static int syslog_action_restricted(int type)
{
	if (dmesg_restrict)
		return 1;
	
	return type != SYSLOG_ACTION_READ_ALL && type != SYSLOG_ACTION_SIZE_BUFFER;
}

static int check_syslog_permissions(int type, bool from_file)
{
	if (from_file && type != SYSLOG_ACTION_OPEN)
		return 0;

	if (syslog_action_restricted(type)) {
		if (capable(CAP_SYSLOG))
			return 0;
		
		if (capable(CAP_SYS_ADMIN)) {
			printk_once(KERN_WARNING "%s (%d): "
				 "Attempt to access syslog with CAP_SYS_ADMIN "
				 "but no CAP_SYSLOG (deprecated).\n",
				 current->comm, task_pid_nr(current));
			return 0;
		}
		return -EPERM;
	}
	return 0;
}

int do_syslog(int type, char __user *buf, int len, bool from_file)
{
	unsigned i, j, limit, count;
	int do_clear = 0;
	char c;
	int error;

	error = check_syslog_permissions(type, from_file);
	if (error)
		goto out;

	error = security_syslog(type);
	if (error)
		return error;

	switch (type) {
	case SYSLOG_ACTION_CLOSE:	
		break;
	case SYSLOG_ACTION_OPEN:	
		break;
	case SYSLOG_ACTION_READ:	
		error = -EINVAL;
		if (!buf || len < 0)
			goto out;
		error = 0;
		if (!len)
			goto out;
		if (!access_ok(VERIFY_WRITE, buf, len)) {
			error = -EFAULT;
			goto out;
		}
		error = wait_event_interruptible(log_wait,
							(log_start - log_end));
		if (error)
			goto out;
		i = 0;
		raw_spin_lock_irq(&logbuf_lock);
		while (!error && (log_start != log_end) && i < len) {
			c = LOG_BUF(log_start);
			log_start++;
			raw_spin_unlock_irq(&logbuf_lock);
			error = __put_user(c,buf);
			buf++;
			i++;
			cond_resched();
			raw_spin_lock_irq(&logbuf_lock);
		}
		raw_spin_unlock_irq(&logbuf_lock);
		if (!error)
			error = i;
		break;
	
	case SYSLOG_ACTION_READ_CLEAR:
		do_clear = 1;
		
	
	case SYSLOG_ACTION_READ_ALL:
		error = -EINVAL;
		if (!buf || len < 0)
			goto out;
		error = 0;
		if (!len)
			goto out;
		if (!access_ok(VERIFY_WRITE, buf, len)) {
			error = -EFAULT;
			goto out;
		}
		count = len;
		if (count > log_buf_len)
			count = log_buf_len;
		raw_spin_lock_irq(&logbuf_lock);
		if (count > logged_chars)
			count = logged_chars;
		if (do_clear)
			logged_chars = 0;
		limit = log_end;
		for (i = 0; i < count && !error; i++) {
			j = limit-1-i;
			if (j + log_buf_len < log_end)
				break;
			c = LOG_BUF(j);
			raw_spin_unlock_irq(&logbuf_lock);
			error = __put_user(c,&buf[count-1-i]);
			cond_resched();
			raw_spin_lock_irq(&logbuf_lock);
		}
		raw_spin_unlock_irq(&logbuf_lock);
		if (error)
			break;
		error = i;
		if (i != count) {
			int offset = count-error;
			
			for (i = 0; i < error; i++) {
				if (__get_user(c,&buf[i+offset]) ||
				    __put_user(c,&buf[i])) {
					error = -EFAULT;
					break;
				}
				cond_resched();
			}
		}
		break;
	
	case SYSLOG_ACTION_CLEAR:
		logged_chars = 0;
		break;
	
	case SYSLOG_ACTION_CONSOLE_OFF:
		if (saved_console_loglevel == -1)
			saved_console_loglevel = console_loglevel;
		console_loglevel = minimum_console_loglevel;
		break;
	
	case SYSLOG_ACTION_CONSOLE_ON:
		if (saved_console_loglevel != -1) {
			console_loglevel = saved_console_loglevel;
			saved_console_loglevel = -1;
		}
		break;
	
	case SYSLOG_ACTION_CONSOLE_LEVEL:
		error = -EINVAL;
		if (len < 1 || len > 8)
			goto out;
		if (len < minimum_console_loglevel)
			len = minimum_console_loglevel;
		console_loglevel = len;
		
		saved_console_loglevel = -1;
		error = 0;
		break;
	
	case SYSLOG_ACTION_SIZE_UNREAD:
		error = log_end - log_start;
		break;
	
	case SYSLOG_ACTION_SIZE_BUFFER:
		error = log_buf_len;
		break;
	default:
		error = -EINVAL;
		break;
	}
out:
	return error;
}

SYSCALL_DEFINE3(syslog, int, type, char __user *, buf, int, len)
{
	return do_syslog(type, buf, len, SYSLOG_FROM_CALL);
}

#ifdef	CONFIG_KGDB_KDB
void kdb_syslog_data(char *syslog_data[4])
{
	syslog_data[0] = log_buf;
	syslog_data[1] = log_buf + log_buf_len;
	syslog_data[2] = log_buf + log_end -
		(logged_chars < log_buf_len ? logged_chars : log_buf_len);
	syslog_data[3] = log_buf + log_end;
}
#endif	

static void __call_console_drivers(unsigned start, unsigned end)
{
	struct console *con;

	for_each_console(con) {
		if (exclusive_console && con != exclusive_console)
			continue;
		if ((con->flags & CON_ENABLED) && con->write &&
				(cpu_online(smp_processor_id()) ||
				(con->flags & CON_ANYTIME)))
			con->write(con, &LOG_BUF(start), end - start);
	}
}

static bool __read_mostly ignore_loglevel;

static int __init ignore_loglevel_setup(char *str)
{
	ignore_loglevel = 1;
	printk(KERN_INFO "debug: ignoring loglevel setting.\n");

	return 0;
}

early_param("ignore_loglevel", ignore_loglevel_setup);
module_param(ignore_loglevel, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(ignore_loglevel, "ignore loglevel setting, to"
	"print all kernel messages to the console.");

static void _call_console_drivers(unsigned start,
				unsigned end, int msg_log_level)
{
	trace_console(&LOG_BUF(0), start, end, log_buf_len);

	if ((msg_log_level < console_loglevel || ignore_loglevel) &&
			console_drivers && start != end) {
		if ((start & LOG_BUF_MASK) > (end & LOG_BUF_MASK)) {
			
			__call_console_drivers(start & LOG_BUF_MASK,
						log_buf_len);
			__call_console_drivers(0, end & LOG_BUF_MASK);
		} else {
			__call_console_drivers(start, end);
		}
	}
}

static size_t log_prefix(const char *p, unsigned int *level, char *special)
{
	unsigned int lev = 0;
	char sp = '\0';
	size_t len;

	if (p[0] != '<' || !p[1])
		return 0;
	if (p[2] == '>') {
		
		switch (p[1]) {
		case '0' ... '7':
			lev = p[1] - '0';
			break;
		case 'c': 
		case 'd': 
			sp = p[1];
			break;
		default:
			return 0;
		}
		len = 3;
	} else {
		
		char *endp = NULL;

		lev = (simple_strtoul(&p[1], &endp, 10) & 7);
		if (endp == NULL || endp[0] != '>')
			return 0;
		len = (endp + 1) - p;
	}

	
	if (sp && !special)
		return 0;

	if (special) {
		*special = sp;
		
		if (sp)
			return len;
	}

	if (level)
		*level = lev;
	return len;
}

static void call_console_drivers(unsigned start, unsigned end)
{
	unsigned cur_index, start_print;
	static int msg_level = -1;

	BUG_ON(((int)(start - end)) > 0);

	cur_index = start;
	start_print = start;
	while (cur_index != end) {
		if (msg_level < 0 && ((end - cur_index) > 2)) {
			
			cur_index += log_prefix(&LOG_BUF(cur_index), &msg_level, NULL);
			start_print = cur_index;
		}
		while (cur_index != end) {
			char c = LOG_BUF(cur_index);

			cur_index++;
			if (c == '\n') {
				if (msg_level < 0) {
					msg_level = default_message_loglevel;
				}
				_call_console_drivers(start_print, cur_index, msg_level);
				msg_level = -1;
				start_print = cur_index;
				break;
			}
		}
	}
	_call_console_drivers(start_print, end, msg_level);
}

static void emit_log_char(char c)
{
	LOG_BUF(log_end) = c;
	log_end++;
	if (log_end - log_start > log_buf_len)
		log_start = log_end - log_buf_len;
	if (log_end - con_start > log_buf_len)
		con_start = log_end - log_buf_len;
	if (logged_chars < log_buf_len)
		logged_chars++;
}

static void zap_locks(void)
{
	static unsigned long oops_timestamp;

	if (time_after_eq(jiffies, oops_timestamp) &&
			!time_after(jiffies, oops_timestamp + 30 * HZ))
		return;

	oops_timestamp = jiffies;

	debug_locks_off();
	
	raw_spin_lock_init(&logbuf_lock);
	
	sema_init(&console_sem, 1);
}

#if defined(CONFIG_PRINTK_TIME)
static bool printk_time = 1;
#else
static bool printk_time = 0;
#endif
module_param_named(time, printk_time, bool, S_IRUGO | S_IWUSR);

#if defined(CONFIG_PRINTK_CPU_ID)
static int printk_cpu_id = 1;
#else
static int printk_cpu_id = 0;
#endif
module_param_named(cpu, printk_cpu_id, int, S_IRUGO | S_IWUSR);

#if defined(CONFIG_PRINTK_PID)
static int printk_pid = 1;
#else
static int printk_pid = 0;
#endif
module_param_named(pid, printk_pid, int, S_IRUGO | S_IWUSR);


static bool always_kmsg_dump;
module_param_named(always_kmsg_dump, always_kmsg_dump, bool, S_IRUGO | S_IWUSR);

static int have_callable_console(void)
{
	struct console *con;

	for_each_console(con)
		if (con->flags & CON_ANYTIME)
			return 1;

	return 0;
}


asmlinkage int printk(const char *fmt, ...)
{
	va_list args;
	int r;
#ifdef CONFIG_MSM_RTB
	void *caller = __builtin_return_address(0);

	uncached_logk_pc(LOGK_LOGBUF, caller, (void *)log_end);
#endif

#ifdef CONFIG_KGDB_KDB
	if (unlikely(kdb_trap_printk)) {
		va_start(args, fmt);
		r = vkdb_printf(fmt, args);
		va_end(args);
		return r;
	}
#endif
	va_start(args, fmt);
	r = vprintk(fmt, args);
	va_end(args);

	return r;
}

static volatile unsigned int printk_cpu = UINT_MAX;

static inline int can_use_console(unsigned int cpu)
{
	return cpu_online(cpu) || have_callable_console();
}

static int console_trylock_for_printk(unsigned int cpu)
	__releases(&logbuf_lock)
{
	int retval = 0, wake = 0;

	if (console_trylock()) {
		retval = 1;

		if (!can_use_console(cpu)) {
			console_locked = 0;
			wake = 1;
			retval = 0;
		}
	}
	printk_cpu = UINT_MAX;
	if (wake)
		up(&console_sem);
	raw_spin_unlock(&logbuf_lock);
	return retval;
}
static const char recursion_bug_msg [] =
		KERN_CRIT "BUG: recent printk recursion!\n";
static int recursion_bug;
static int new_text_line = 1;
static char printk_buf[1024];

int printk_delay_msec __read_mostly;

static inline void printk_delay(void)
{
	if (unlikely(printk_delay_msec)) {
		int m = printk_delay_msec;

		while (m--) {
			mdelay(1);
			touch_nmi_watchdog();
		}
	}
}

asmlinkage int vprintk(const char *fmt, va_list args)
{
	int printed_len = 0;
	int current_log_level = default_message_loglevel;
	unsigned long flags;
	int this_cpu;
	char *p;
	size_t plen;
	char special;

	boot_delay_msec();
	printk_delay();

	
	local_irq_save(flags);
	this_cpu = smp_processor_id();

	if (unlikely(printk_cpu == this_cpu)) {
		if (!oops_in_progress && !lockdep_recursing(current)) {
			recursion_bug = 1;
			goto out_restore_irqs;
		}
		zap_locks();
	}

	lockdep_off();
	raw_spin_lock(&logbuf_lock);
	printk_cpu = this_cpu;

	if (recursion_bug) {
		recursion_bug = 0;
		strcpy(printk_buf, recursion_bug_msg);
		printed_len = strlen(recursion_bug_msg);
	}
	
	printed_len += vscnprintf(printk_buf + printed_len,
				  sizeof(printk_buf) - printed_len, fmt, args);


	p = printk_buf;

	
	plen = log_prefix(p, &current_log_level, &special);
	if (plen) {
		p += plen;

		switch (special) {
		case 'c': 
			plen = 0;
			break;
		case 'd': 
			plen = 0;
		default:
			if (!new_text_line) {
				emit_log_char('\n');
				new_text_line = 1;
			}
		}
	}

	for (; *p; p++) {
		if (new_text_line) {
			new_text_line = 0;

			if (plen) {
				
				int i;

				for (i = 0; i < plen; i++)
					emit_log_char(printk_buf[i]);
				printed_len += plen;
			} else {
				
				emit_log_char('<');
				emit_log_char(current_log_level + '0');
				emit_log_char('>');
				printed_len += 3;
			}

			if (printk_time) {
				
				char tbuf[50], *tp;
				unsigned tlen;
				unsigned long long t;
				unsigned long nanosec_rem;

				t = cpu_clock(printk_cpu);
				nanosec_rem = do_div(t, 1000000000);
				tlen = sprintf(tbuf, "[%5lu.%06lu] ",
						(unsigned long) t,
						nanosec_rem / 1000);

				for (tp = tbuf; tp < tbuf + tlen; tp++)
					emit_log_char(*tp);
				printed_len += tlen;
			}

			if (printk_cpu_id) {
				
				char tbuf[10], *tp;
				unsigned tlen;

				tlen = sprintf(tbuf, "c%u ", printk_cpu);

				for (tp = tbuf; tp < tbuf + tlen; tp++)
					emit_log_char(*tp);
				printed_len += tlen;
			}

			if (printk_pid) {
				
				char tbuf[10], *tp;
				unsigned tlen;

				tlen = sprintf(tbuf, "%6u ", current->pid);

				for (tp = tbuf; tp < tbuf + tlen; tp++)
					emit_log_char(*tp);
				printed_len += tlen;
			}

			if (!*p)
				break;
		}

		emit_log_char(*p);
		if (*p == '\n')
			new_text_line = 1;
	}

	if (console_trylock_for_printk(this_cpu))
		console_unlock();

	lockdep_on();
out_restore_irqs:
	local_irq_restore(flags);

	return printed_len;
}
EXPORT_SYMBOL(printk);
EXPORT_SYMBOL(vprintk);

#else

static void call_console_drivers(unsigned start, unsigned end)
{
}

#endif

static int __add_preferred_console(char *name, int idx, char *options,
				   char *brl_options)
{
	struct console_cmdline *c;
	int i;

	for (i = 0; i < MAX_CMDLINECONSOLES && console_cmdline[i].name[0]; i++)
		if (strcmp(console_cmdline[i].name, name) == 0 &&
			  console_cmdline[i].index == idx) {
				if (!brl_options)
					selected_console = i;
				return 0;
		}
	if (i == MAX_CMDLINECONSOLES)
		return -E2BIG;
	if (!brl_options)
		selected_console = i;
	c = &console_cmdline[i];
	strlcpy(c->name, name, sizeof(c->name));
	c->options = options;
#ifdef CONFIG_A11Y_BRAILLE_CONSOLE
	c->brl_options = brl_options;
#endif
	c->index = idx;
	return 0;
}
static int __init console_setup(char *str)
{
	char buf[sizeof(console_cmdline[0].name) + 4]; 
	char *s, *options, *brl_options = NULL;
	int idx;

#ifdef CONFIG_A11Y_BRAILLE_CONSOLE
	if (!memcmp(str, "brl,", 4)) {
		brl_options = "";
		str += 4;
	} else if (!memcmp(str, "brl=", 4)) {
		brl_options = str + 4;
		str = strchr(brl_options, ',');
		if (!str) {
			printk(KERN_ERR "need port name after brl=\n");
			return 1;
		}
		*(str++) = 0;
	}
#endif

	if (str[0] >= '0' && str[0] <= '9') {
		strcpy(buf, "ttyS");
		strncpy(buf + 4, str, sizeof(buf) - 5);
	} else {
		strncpy(buf, str, sizeof(buf) - 1);
	}
	buf[sizeof(buf) - 1] = 0;
	if ((options = strchr(str, ',')) != NULL)
		*(options++) = 0;
#ifdef __sparc__
	if (!strcmp(str, "ttya"))
		strcpy(buf, "ttyS0");
	if (!strcmp(str, "ttyb"))
		strcpy(buf, "ttyS1");
#endif
	for (s = buf; *s; s++)
		if ((*s >= '0' && *s <= '9') || *s == ',')
			break;
	idx = simple_strtoul(s, NULL, 10);
	*s = 0;

	__add_preferred_console(buf, idx, options, brl_options);
	console_set_on_cmdline = 1;
	return 1;
}
__setup("console=", console_setup);

int add_preferred_console(char *name, int idx, char *options)
{
	return __add_preferred_console(name, idx, options, NULL);
}

int update_console_cmdline(char *name, int idx, char *name_new, int idx_new, char *options)
{
	struct console_cmdline *c;
	int i;

	for (i = 0; i < MAX_CMDLINECONSOLES && console_cmdline[i].name[0]; i++)
		if (strcmp(console_cmdline[i].name, name) == 0 &&
			  console_cmdline[i].index == idx) {
				c = &console_cmdline[i];
				strlcpy(c->name, name_new, sizeof(c->name));
				c->name[sizeof(c->name) - 1] = 0;
				c->options = options;
				c->index = idx_new;
				return i;
		}
	
	return -1;
}

bool console_suspend_enabled = 1;
EXPORT_SYMBOL(console_suspend_enabled);

static int __init console_suspend_disable(char *str)
{
	console_suspend_enabled = 0;
	return 1;
}
__setup("no_console_suspend", console_suspend_disable);

int suspend_console_deferred;
module_param_named(
	suspend_console_deferred, suspend_console_deferred, int, S_IRUGO | S_IWUSR | S_IWGRP
);

module_param_named(console_suspend, console_suspend_enabled,
		bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(console_suspend, "suspend console during suspend"
	" and hibernate operations");

void suspend_console(void)
{
	if (!console_suspend_enabled)
		return;
	printk("Suspending console(s) (use no_console_suspend to debug)\n");
	console_lock();
	console_suspended = 1;
	up(&console_sem);
}

void resume_console(void)
{
	if (!console_suspend_enabled)
		return;
	down(&console_sem);
	console_suspended = 0;
	console_unlock();
}

static void __cpuinit console_flush(struct work_struct *work)
{
	console_lock();
	console_unlock();
}

static __cpuinitdata DECLARE_WORK(console_cpu_notify_work, console_flush);

static int __cpuinit console_cpu_notify(struct notifier_block *self,
	unsigned long action, void *hcpu)
{
	switch (action) {
	case CPU_DEAD:
	case CPU_DOWN_FAILED:
	case CPU_UP_CANCELED:
		console_lock();
		console_unlock();
		break;
	case CPU_ONLINE:
	case CPU_DYING:
		
		if (!console_trylock())
			schedule_work(&console_cpu_notify_work);
		else
			console_unlock();
	}
	return NOTIFY_OK;
}

void console_lock(void)
{
	BUG_ON(in_interrupt());
	down(&console_sem);
	if (console_suspended)
		return;
	console_locked = 1;
	console_may_schedule = 1;
}
EXPORT_SYMBOL(console_lock);

int console_trylock(void)
{
	if (down_trylock(&console_sem))
		return 0;
	if (console_suspended) {
		up(&console_sem);
		return 0;
	}
	console_locked = 1;
	console_may_schedule = 0;
	return 1;
}
EXPORT_SYMBOL(console_trylock);

int is_console_locked(void)
{
	return console_locked;
}

#define PRINTK_BUF_SIZE		512

#define PRINTK_PENDING_WAKEUP	0x01
#define PRINTK_PENDING_SCHED	0x02

static DEFINE_PER_CPU(int, printk_pending);
static DEFINE_PER_CPU(char [PRINTK_BUF_SIZE], printk_sched_buf);

void printk_tick(void)
{
	if (__this_cpu_read(printk_pending)) {
		int pending = __this_cpu_xchg(printk_pending, 0);
		if (pending & PRINTK_PENDING_SCHED) {
			char *buf = __get_cpu_var(printk_sched_buf);
			printk(KERN_WARNING "[sched_delayed] %s", buf);
		}
		if (pending & PRINTK_PENDING_WAKEUP)
			wake_up_interruptible(&log_wait);
	}
}

int printk_needs_cpu(int cpu)
{
	if (cpu_is_offline(cpu))
		printk_tick();
	return __this_cpu_read(printk_pending);
}

void wake_up_klogd(void)
{
	if (waitqueue_active(&log_wait))
		this_cpu_or(printk_pending, PRINTK_PENDING_WAKEUP);
}

void console_unlock(void)
{
	unsigned long flags;
	unsigned _con_start, _log_end;
	unsigned wake_klogd = 0, retry = 0;

	if (console_suspended) {
		up(&console_sem);
		return;
	}

	console_may_schedule = 0;

again:
	for ( ; ; ) {
		raw_spin_lock_irqsave(&logbuf_lock, flags);
		wake_klogd |= log_start - log_end;
		if (con_start == log_end)
			break;			
		_con_start = con_start;
		_log_end = log_end;
		con_start = log_end;		
		raw_spin_unlock(&logbuf_lock);
		stop_critical_timings();	
		call_console_drivers(_con_start, _log_end);
		start_critical_timings();
		local_irq_restore(flags);
	}
	console_locked = 0;

	
	if (unlikely(exclusive_console))
		exclusive_console = NULL;

	raw_spin_unlock(&logbuf_lock);

	up(&console_sem);

	raw_spin_lock(&logbuf_lock);
	if (con_start != log_end)
		retry = 1;
	else
		retry = 0;
	raw_spin_unlock_irqrestore(&logbuf_lock, flags);

	if (retry && console_trylock())
		goto again;

	if (wake_klogd)
		wake_up_klogd();
}
EXPORT_SYMBOL(console_unlock);

void __sched console_conditional_schedule(void)
{
	if (console_may_schedule)
		cond_resched();
}
EXPORT_SYMBOL(console_conditional_schedule);

void console_unblank(void)
{
	struct console *c;

	if (oops_in_progress) {
		if (down_trylock(&console_sem) != 0)
			return;
	} else
		console_lock();

	console_locked = 1;
	console_may_schedule = 0;
	for_each_console(c)
		if ((c->flags & CON_ENABLED) && c->unblank)
			c->unblank();
	console_unlock();
}

struct tty_driver *console_device(int *index)
{
	struct console *c;
	struct tty_driver *driver = NULL;

	console_lock();
	for_each_console(c) {
		if (!c->device)
			continue;
		driver = c->device(c, index);
		if (driver)
			break;
	}
	console_unlock();
	return driver;
}

void console_stop(struct console *console)
{
	console_lock();
	console->flags &= ~CON_ENABLED;
	console_unlock();
}
EXPORT_SYMBOL(console_stop);

void console_start(struct console *console)
{
	console_lock();
	console->flags |= CON_ENABLED;
	console_unlock();
}
EXPORT_SYMBOL(console_start);

static int __read_mostly keep_bootcon;

static int __init keep_bootcon_setup(char *str)
{
	keep_bootcon = 1;
	printk(KERN_INFO "debug: skip boot console de-registration.\n");

	return 0;
}

early_param("keep_bootcon", keep_bootcon_setup);

void register_console(struct console *newcon)
{
	int i;
	unsigned long flags;
	struct console *bcon = NULL;

	if (console_drivers && newcon->flags & CON_BOOT) {
		
		for_each_console(bcon) {
			if (!(bcon->flags & CON_BOOT)) {
				printk(KERN_INFO "Too late to register bootconsole %s%d\n",
					newcon->name, newcon->index);
				return;
			}
		}
	}

	if (console_drivers && console_drivers->flags & CON_BOOT)
		bcon = console_drivers;

	if (preferred_console < 0 || bcon || !console_drivers)
		preferred_console = selected_console;

	if (newcon->early_setup)
		newcon->early_setup();

	if (preferred_console < 0) {
		if (newcon->index < 0)
			newcon->index = 0;
		if (newcon->setup == NULL ||
		    newcon->setup(newcon, NULL) == 0) {
			newcon->flags |= CON_ENABLED;
			if (newcon->device) {
				newcon->flags |= CON_CONSDEV;
				preferred_console = 0;
			}
		}
	}

	for (i = 0; i < MAX_CMDLINECONSOLES && console_cmdline[i].name[0];
			i++) {
		if (strcmp(console_cmdline[i].name, newcon->name) != 0)
			continue;
		if (newcon->index >= 0 &&
		    newcon->index != console_cmdline[i].index)
			continue;
		if (newcon->index < 0)
			newcon->index = console_cmdline[i].index;
#ifdef CONFIG_A11Y_BRAILLE_CONSOLE
		if (console_cmdline[i].brl_options) {
			newcon->flags |= CON_BRL;
			braille_register_console(newcon,
					console_cmdline[i].index,
					console_cmdline[i].options,
					console_cmdline[i].brl_options);
			return;
		}
#endif
		if (newcon->setup &&
		    newcon->setup(newcon, console_cmdline[i].options) != 0)
			break;
		newcon->flags |= CON_ENABLED;
		newcon->index = console_cmdline[i].index;
		if (i == selected_console) {
			newcon->flags |= CON_CONSDEV;
			preferred_console = selected_console;
		}
		break;
	}

	if (!(newcon->flags & CON_ENABLED))
		return;

	if (bcon && ((newcon->flags & (CON_CONSDEV | CON_BOOT)) == CON_CONSDEV))
		newcon->flags &= ~CON_PRINTBUFFER;

	console_lock();
	if ((newcon->flags & CON_CONSDEV) || console_drivers == NULL) {
		newcon->next = console_drivers;
		console_drivers = newcon;
		if (newcon->next)
			newcon->next->flags &= ~CON_CONSDEV;
	} else {
		newcon->next = console_drivers->next;
		console_drivers->next = newcon;
	}
	if (newcon->flags & CON_PRINTBUFFER) {
		raw_spin_lock_irqsave(&logbuf_lock, flags);
		con_start = log_start;
		raw_spin_unlock_irqrestore(&logbuf_lock, flags);
		exclusive_console = newcon;
	}
	console_unlock();
	console_sysfs_notify();

	if (bcon &&
	    ((newcon->flags & (CON_CONSDEV | CON_BOOT)) == CON_CONSDEV) &&
	    !keep_bootcon) {
		printk(KERN_INFO "console [%s%d] enabled, bootconsole disabled\n",
			newcon->name, newcon->index);
		for_each_console(bcon)
			if (bcon->flags & CON_BOOT)
				unregister_console(bcon);
	} else {
		printk(KERN_INFO "%sconsole [%s%d] enabled\n",
			(newcon->flags & CON_BOOT) ? "boot" : "" ,
			newcon->name, newcon->index);
	}
}
EXPORT_SYMBOL(register_console);

int unregister_console(struct console *console)
{
        struct console *a, *b;
	int res = 1;

#ifdef CONFIG_A11Y_BRAILLE_CONSOLE
	if (console->flags & CON_BRL)
		return braille_unregister_console(console);
#endif

	console_lock();
	if (console_drivers == console) {
		console_drivers=console->next;
		res = 0;
	} else if (console_drivers) {
		for (a=console_drivers->next, b=console_drivers ;
		     a; b=a, a=b->next) {
			if (a == console) {
				b->next = a->next;
				res = 0;
				break;
			}
		}
	}

	if (console_drivers != NULL && console->flags & CON_CONSDEV)
		console_drivers->flags |= CON_CONSDEV;

	console_unlock();
	console_sysfs_notify();
	return res;
}
EXPORT_SYMBOL(unregister_console);

static int __init printk_late_init(void)
{
	struct console *con;

	for_each_console(con) {
		if (!keep_bootcon && con->flags & CON_BOOT) {
			printk(KERN_INFO "turn off boot console %s%d\n",
				con->name, con->index);
			unregister_console(con);
		}
	}
	hotcpu_notifier(console_cpu_notify, 0);
	return 0;
}
late_initcall(printk_late_init);

#if defined CONFIG_PRINTK

int printk_sched(const char *fmt, ...)
{
	unsigned long flags;
	va_list args;
	char *buf;
	int r;

	local_irq_save(flags);
	buf = __get_cpu_var(printk_sched_buf);

	va_start(args, fmt);
	r = vsnprintf(buf, PRINTK_BUF_SIZE, fmt, args);
	va_end(args);

	__this_cpu_or(printk_pending, PRINTK_PENDING_SCHED);
	local_irq_restore(flags);

	return r;
}

DEFINE_RATELIMIT_STATE(printk_ratelimit_state, 5 * HZ, 10);

int __printk_ratelimit(const char *func)
{
	return ___ratelimit(&printk_ratelimit_state, func);
}
EXPORT_SYMBOL(__printk_ratelimit);

bool printk_timed_ratelimit(unsigned long *caller_jiffies,
			unsigned int interval_msecs)
{
	if (*caller_jiffies == 0
			|| !time_in_range(jiffies, *caller_jiffies,
					*caller_jiffies
					+ msecs_to_jiffies(interval_msecs))) {
		*caller_jiffies = jiffies;
		return true;
	}
	return false;
}
EXPORT_SYMBOL(printk_timed_ratelimit);

static DEFINE_SPINLOCK(dump_list_lock);
static LIST_HEAD(dump_list);

int kmsg_dump_register(struct kmsg_dumper *dumper)
{
	unsigned long flags;
	int err = -EBUSY;

	
	if (!dumper->dump)
		return -EINVAL;

	spin_lock_irqsave(&dump_list_lock, flags);
	
	if (!dumper->registered) {
		dumper->registered = 1;
		list_add_tail_rcu(&dumper->list, &dump_list);
		err = 0;
	}
	spin_unlock_irqrestore(&dump_list_lock, flags);

	return err;
}
EXPORT_SYMBOL_GPL(kmsg_dump_register);

int kmsg_dump_unregister(struct kmsg_dumper *dumper)
{
	unsigned long flags;
	int err = -EINVAL;

	spin_lock_irqsave(&dump_list_lock, flags);
	if (dumper->registered) {
		dumper->registered = 0;
		list_del_rcu(&dumper->list);
		err = 0;
	}
	spin_unlock_irqrestore(&dump_list_lock, flags);
	synchronize_rcu();

	return err;
}
EXPORT_SYMBOL_GPL(kmsg_dump_unregister);

void kmsg_dump(enum kmsg_dump_reason reason)
{
	unsigned long end;
	unsigned chars;
	struct kmsg_dumper *dumper;
	const char *s1, *s2;
	unsigned long l1, l2;
	unsigned long flags;

	if ((reason > KMSG_DUMP_OOPS) && !always_kmsg_dump)
		return;

	raw_spin_lock_irqsave(&logbuf_lock, flags);
	end = log_end & LOG_BUF_MASK;
	chars = logged_chars;
	raw_spin_unlock_irqrestore(&logbuf_lock, flags);

	if (chars > end) {
		s1 = log_buf + log_buf_len - chars + end;
		l1 = chars - end;

		s2 = log_buf;
		l2 = end;
	} else {
		s1 = "";
		l1 = 0;

		s2 = log_buf + end - chars;
		l2 = chars;
	}

	rcu_read_lock();
	list_for_each_entry_rcu(dumper, &dump_list, list)
		dumper->dump(dumper, reason, s1, l1, s2, l2);
	rcu_read_unlock();
}
#endif
