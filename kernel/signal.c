/*
 *  linux/kernel/signal.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 *  1997-11-02  Modified for POSIX.1b signals by Richard Henderson
 *
 *  2003-06-02  Jim Houston - Concurrent Computer Corp.
 *		Changes to use preallocated sigqueue structures
 *		to allow signals to be sent reliably.
 */

#include <linux/slab.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/tty.h>
#include <linux/binfmts.h>
#include <linux/security.h>
#include <linux/syscalls.h>
#include <linux/ptrace.h>
#include <linux/signal.h>
#include <linux/signalfd.h>
#include <linux/ratelimit.h>
#include <linux/tracehook.h>
#include <linux/capability.h>
#include <linux/freezer.h>
#include <linux/pid_namespace.h>
#include <linux/nsproxy.h>
#include <linux/user_namespace.h>
#define CREATE_TRACE_POINTS
#include <trace/events/signal.h>

#include <asm/param.h>
#include <asm/uaccess.h>
#include <asm/unistd.h>
#include <asm/siginfo.h>
#include <asm/cacheflush.h>
#include <htc_debug/stability/htc_process_debug.h>
#include "audit.h"	


static struct kmem_cache *sigqueue_cachep;

int print_fatal_signals __read_mostly;

static void __user *sig_handler(struct task_struct *t, int sig)
{
	return t->sighand->action[sig - 1].sa.sa_handler;
}

static int sig_handler_ignored(void __user *handler, int sig)
{
	
	return handler == SIG_IGN ||
		(handler == SIG_DFL && sig_kernel_ignore(sig));
}

static int sig_task_ignored(struct task_struct *t, int sig, bool force)
{
	void __user *handler;

	handler = sig_handler(t, sig);

	if (unlikely(t->signal->flags & SIGNAL_UNKILLABLE) &&
			handler == SIG_DFL && !force)
		return 1;

	return sig_handler_ignored(handler, sig);
}

static int sig_ignored(struct task_struct *t, int sig, bool force)
{
	if (sigismember(&t->blocked, sig) || sigismember(&t->real_blocked, sig))
		return 0;

	if (!sig_task_ignored(t, sig, force))
		return 0;

	return !t->ptrace;
}

static inline int has_pending_signals(sigset_t *signal, sigset_t *blocked)
{
	unsigned long ready;
	long i;

	switch (_NSIG_WORDS) {
	default:
		for (i = _NSIG_WORDS, ready = 0; --i >= 0 ;)
			ready |= signal->sig[i] &~ blocked->sig[i];
		break;

	case 4: ready  = signal->sig[3] &~ blocked->sig[3];
		ready |= signal->sig[2] &~ blocked->sig[2];
		ready |= signal->sig[1] &~ blocked->sig[1];
		ready |= signal->sig[0] &~ blocked->sig[0];
		break;

	case 2: ready  = signal->sig[1] &~ blocked->sig[1];
		ready |= signal->sig[0] &~ blocked->sig[0];
		break;

	case 1: ready  = signal->sig[0] &~ blocked->sig[0];
	}
	return ready !=	0;
}

#define PENDING(p,b) has_pending_signals(&(p)->signal, (b))

static int recalc_sigpending_tsk(struct task_struct *t)
{
	if ((t->jobctl & JOBCTL_PENDING_MASK) ||
	    PENDING(&t->pending, &t->blocked) ||
	    PENDING(&t->signal->shared_pending, &t->blocked)) {
		set_tsk_thread_flag(t, TIF_SIGPENDING);
		return 1;
	}
	return 0;
}

void recalc_sigpending_and_wake(struct task_struct *t)
{
	if (recalc_sigpending_tsk(t))
		signal_wake_up(t, 0);
}

void recalc_sigpending(void)
{
	if (!recalc_sigpending_tsk(current) && !freezing(current))
		clear_thread_flag(TIF_SIGPENDING);

}


#define SYNCHRONOUS_MASK \
	(sigmask(SIGSEGV) | sigmask(SIGBUS) | sigmask(SIGILL) | \
	 sigmask(SIGTRAP) | sigmask(SIGFPE))

int next_signal(struct sigpending *pending, sigset_t *mask)
{
	unsigned long i, *s, *m, x;
	int sig = 0;

	s = pending->signal.sig;
	m = mask->sig;

	x = *s &~ *m;
	if (x) {
		if (x & SYNCHRONOUS_MASK)
			x &= SYNCHRONOUS_MASK;
		sig = ffz(~x) + 1;
		return sig;
	}

	switch (_NSIG_WORDS) {
	default:
		for (i = 1; i < _NSIG_WORDS; ++i) {
			x = *++s &~ *++m;
			if (!x)
				continue;
			sig = ffz(~x) + i*_NSIG_BPW + 1;
			break;
		}
		break;

	case 2:
		x = s[1] &~ m[1];
		if (!x)
			break;
		sig = ffz(~x) + _NSIG_BPW + 1;
		break;

	case 1:
		
		break;
	}

	return sig;
}

static inline void print_dropped_signal(int sig)
{
	static DEFINE_RATELIMIT_STATE(ratelimit_state, 5 * HZ, 10);

	if (!print_fatal_signals)
		return;

	if (!__ratelimit(&ratelimit_state))
		return;

	printk(KERN_INFO "%s/%d: reached RLIMIT_SIGPENDING, dropped signal %d\n",
				current->comm, current->pid, sig);
}

bool task_set_jobctl_pending(struct task_struct *task, unsigned int mask)
{
	BUG_ON(mask & ~(JOBCTL_PENDING_MASK | JOBCTL_STOP_CONSUME |
			JOBCTL_STOP_SIGMASK | JOBCTL_TRAPPING));
	BUG_ON((mask & JOBCTL_TRAPPING) && !(mask & JOBCTL_PENDING_MASK));

	if (unlikely(fatal_signal_pending(task) || (task->flags & PF_EXITING)))
		return false;

	if (mask & JOBCTL_STOP_SIGMASK)
		task->jobctl &= ~JOBCTL_STOP_SIGMASK;

	task->jobctl |= mask;
	return true;
}

void task_clear_jobctl_trapping(struct task_struct *task)
{
	if (unlikely(task->jobctl & JOBCTL_TRAPPING)) {
		task->jobctl &= ~JOBCTL_TRAPPING;
		wake_up_bit(&task->jobctl, JOBCTL_TRAPPING_BIT);
	}
}

void task_clear_jobctl_pending(struct task_struct *task, unsigned int mask)
{
	BUG_ON(mask & ~JOBCTL_PENDING_MASK);

	if (mask & JOBCTL_STOP_PENDING)
		mask |= JOBCTL_STOP_CONSUME | JOBCTL_STOP_DEQUEUED;

	task->jobctl &= ~mask;

	if (!(task->jobctl & JOBCTL_PENDING_MASK))
		task_clear_jobctl_trapping(task);
}

static bool task_participate_group_stop(struct task_struct *task)
{
	struct signal_struct *sig = task->signal;
	bool consume = task->jobctl & JOBCTL_STOP_CONSUME;

	WARN_ON_ONCE(!(task->jobctl & JOBCTL_STOP_PENDING));

	task_clear_jobctl_pending(task, JOBCTL_STOP_PENDING);

	if (!consume)
		return false;

	if (!WARN_ON_ONCE(sig->group_stop_count == 0))
		sig->group_stop_count--;

	if (!sig->group_stop_count && !(sig->flags & SIGNAL_STOP_STOPPED)) {
		sig->flags = SIGNAL_STOP_STOPPED;
		return true;
	}
	return false;
}

static struct sigqueue *
__sigqueue_alloc(int sig, struct task_struct *t, gfp_t flags, int override_rlimit)
{
	struct sigqueue *q = NULL;
	struct user_struct *user;

	rcu_read_lock();
	user = get_uid(__task_cred(t)->user);
	atomic_inc(&user->sigpending);
	rcu_read_unlock();

	if (override_rlimit ||
	    atomic_read(&user->sigpending) <=
			task_rlimit(t, RLIMIT_SIGPENDING)) {
		q = kmem_cache_alloc(sigqueue_cachep, flags);
	} else {
		print_dropped_signal(sig);
	}

	if (unlikely(q == NULL)) {
		atomic_dec(&user->sigpending);
		free_uid(user);
	} else {
		INIT_LIST_HEAD(&q->list);
		q->flags = 0;
		q->user = user;
	}

	return q;
}

static void __sigqueue_free(struct sigqueue *q)
{
	if (q->flags & SIGQUEUE_PREALLOC)
		return;
	atomic_dec(&q->user->sigpending);
	free_uid(q->user);
	kmem_cache_free(sigqueue_cachep, q);
}

void flush_sigqueue(struct sigpending *queue)
{
	struct sigqueue *q;

	sigemptyset(&queue->signal);
	while (!list_empty(&queue->list)) {
		q = list_entry(queue->list.next, struct sigqueue , list);
		list_del_init(&q->list);
		__sigqueue_free(q);
	}
}

void __flush_signals(struct task_struct *t)
{
	clear_tsk_thread_flag(t, TIF_SIGPENDING);
	flush_sigqueue(&t->pending);
	flush_sigqueue(&t->signal->shared_pending);
}

void flush_signals(struct task_struct *t)
{
	unsigned long flags;

	spin_lock_irqsave(&t->sighand->siglock, flags);
	__flush_signals(t);
	spin_unlock_irqrestore(&t->sighand->siglock, flags);
}

static void __flush_itimer_signals(struct sigpending *pending)
{
	sigset_t signal, retain;
	struct sigqueue *q, *n;

	signal = pending->signal;
	sigemptyset(&retain);

	list_for_each_entry_safe(q, n, &pending->list, list) {
		int sig = q->info.si_signo;

		if (likely(q->info.si_code != SI_TIMER)) {
			sigaddset(&retain, sig);
		} else {
			sigdelset(&signal, sig);
			list_del_init(&q->list);
			__sigqueue_free(q);
		}
	}

	sigorsets(&pending->signal, &signal, &retain);
}

void flush_itimer_signals(void)
{
	struct task_struct *tsk = current;
	unsigned long flags;

	spin_lock_irqsave(&tsk->sighand->siglock, flags);
	__flush_itimer_signals(&tsk->pending);
	__flush_itimer_signals(&tsk->signal->shared_pending);
	spin_unlock_irqrestore(&tsk->sighand->siglock, flags);
}

void ignore_signals(struct task_struct *t)
{
	int i;

	for (i = 0; i < _NSIG; ++i)
		t->sighand->action[i].sa.sa_handler = SIG_IGN;

	flush_signals(t);
}


void
flush_signal_handlers(struct task_struct *t, int force_default)
{
	int i;
	struct k_sigaction *ka = &t->sighand->action[0];
	for (i = _NSIG ; i != 0 ; i--) {
		if (force_default || ka->sa.sa_handler != SIG_IGN)
			ka->sa.sa_handler = SIG_DFL;
		ka->sa.sa_flags = 0;
#ifdef SA_RESTORER
		ka->sa.sa_restorer = NULL;
#endif
		sigemptyset(&ka->sa.sa_mask);
		ka++;
	}
}

int unhandled_signal(struct task_struct *tsk, int sig)
{
	void __user *handler = tsk->sighand->action[sig-1].sa.sa_handler;
	if (is_global_init(tsk))
		return 1;
	if (handler != SIG_IGN && handler != SIG_DFL)
		return 0;
	
	return !tsk->ptrace;
}

void
block_all_signals(int (*notifier)(void *priv), void *priv, sigset_t *mask)
{
	unsigned long flags;

	spin_lock_irqsave(&current->sighand->siglock, flags);
	current->notifier_mask = mask;
	current->notifier_data = priv;
	current->notifier = notifier;
	spin_unlock_irqrestore(&current->sighand->siglock, flags);
}


void
unblock_all_signals(void)
{
	unsigned long flags;

	spin_lock_irqsave(&current->sighand->siglock, flags);
	current->notifier = NULL;
	current->notifier_data = NULL;
	recalc_sigpending();
	spin_unlock_irqrestore(&current->sighand->siglock, flags);
}

static void collect_signal(int sig, struct sigpending *list, siginfo_t *info)
{
	struct sigqueue *q, *first = NULL;

	list_for_each_entry(q, &list->list, list) {
		if (q->info.si_signo == sig) {
			if (first)
				goto still_pending;
			first = q;
		}
	}

	sigdelset(&list->signal, sig);

	if (first) {
still_pending:
		list_del_init(&first->list);
		copy_siginfo(info, &first->info);
		__sigqueue_free(first);
	} else {
		info->si_signo = sig;
		info->si_errno = 0;
		info->si_code = SI_USER;
		info->si_pid = 0;
		info->si_uid = 0;
	}
}

static int __dequeue_signal(struct sigpending *pending, sigset_t *mask,
			siginfo_t *info)
{
	int sig = next_signal(pending, mask);

	if (sig) {
		if (current->notifier) {
			if (sigismember(current->notifier_mask, sig)) {
				if (!(current->notifier)(current->notifier_data)) {
					clear_thread_flag(TIF_SIGPENDING);
					return 0;
				}
			}
		}

		collect_signal(sig, pending, info);
	}

	return sig;
}

int dequeue_signal(struct task_struct *tsk, sigset_t *mask, siginfo_t *info)
{
	int signr;

	signr = __dequeue_signal(&tsk->pending, mask, info);
	if (!signr) {
		signr = __dequeue_signal(&tsk->signal->shared_pending,
					 mask, info);
		if (unlikely(signr == SIGALRM)) {
			struct hrtimer *tmr = &tsk->signal->real_timer;

			if (!hrtimer_is_queued(tmr) &&
			    tsk->signal->it_real_incr.tv64 != 0) {
				hrtimer_forward(tmr, tmr->base->get_time(),
						tsk->signal->it_real_incr);
				hrtimer_restart(tmr);
			}
		}
	}

	recalc_sigpending();
	if (!signr)
		return 0;

	if (unlikely(sig_kernel_stop(signr))) {
		current->jobctl |= JOBCTL_STOP_DEQUEUED;
	}
	if ((info->si_code & __SI_MASK) == __SI_TIMER && info->si_sys_private) {
		spin_unlock(&tsk->sighand->siglock);
		do_schedule_next_timer(info);
		spin_lock(&tsk->sighand->siglock);
	}
	return signr;
}

void signal_wake_up(struct task_struct *t, int resume)
{
	unsigned int mask;

	set_tsk_thread_flag(t, TIF_SIGPENDING);

	mask = TASK_INTERRUPTIBLE;
	if (resume)
		mask |= TASK_WAKEKILL;
	if (!wake_up_state(t, mask))
		kick_process(t);
}

static int rm_from_queue_full(sigset_t *mask, struct sigpending *s)
{
	struct sigqueue *q, *n;
	sigset_t m;

	sigandsets(&m, mask, &s->signal);
	if (sigisemptyset(&m))
		return 0;

	sigandnsets(&s->signal, &s->signal, mask);
	list_for_each_entry_safe(q, n, &s->list, list) {
		if (sigismember(mask, q->info.si_signo)) {
			list_del_init(&q->list);
			__sigqueue_free(q);
		}
	}
	return 1;
}
static int rm_from_queue(unsigned long mask, struct sigpending *s)
{
	struct sigqueue *q, *n;

	if (!sigtestsetmask(&s->signal, mask))
		return 0;

	sigdelsetmask(&s->signal, mask);
	list_for_each_entry_safe(q, n, &s->list, list) {
		if (q->info.si_signo < SIGRTMIN &&
		    (mask & sigmask(q->info.si_signo))) {
			list_del_init(&q->list);
			__sigqueue_free(q);
		}
	}
	return 1;
}

static inline int is_si_special(const struct siginfo *info)
{
	return info <= SEND_SIG_FORCED;
}

static inline bool si_fromuser(const struct siginfo *info)
{
	return info == SEND_SIG_NOINFO ||
		(!is_si_special(info) && SI_FROMUSER(info));
}

static int kill_ok_by_cred(struct task_struct *t)
{
	const struct cred *cred = current_cred();
	const struct cred *tcred = __task_cred(t);

	if (cred->user->user_ns == tcred->user->user_ns &&
	    (cred->euid == tcred->suid ||
	     cred->euid == tcred->uid ||
	     cred->uid  == tcred->suid ||
	     cred->uid  == tcred->uid))
		return 1;

	if (ns_capable(tcred->user->user_ns, CAP_KILL))
		return 1;

	return 0;
}

static int check_kill_permission(int sig, struct siginfo *info,
				 struct task_struct *t)
{
	struct pid *sid;
	int error;

	if (!valid_signal(sig))
		return -EINVAL;

	if (!si_fromuser(info))
		return 0;

	error = audit_signal_info(sig, t); 
	if (error)
		return error;

	if (!same_thread_group(current, t) &&
	    !kill_ok_by_cred(t)) {
		switch (sig) {
		case SIGCONT:
			sid = task_session(t);
			if (!sid || sid == task_session(current))
				break;
		default:
			return -EPERM;
		}
	}

	return security_task_kill(t, info, sig, 0);
}

static void ptrace_trap_notify(struct task_struct *t)
{
	WARN_ON_ONCE(!(t->ptrace & PT_SEIZED));
	assert_spin_locked(&t->sighand->siglock);

	task_set_jobctl_pending(t, JOBCTL_TRAP_NOTIFY);
	signal_wake_up(t, t->jobctl & JOBCTL_LISTENING);
}

static int prepare_signal(int sig, struct task_struct *p, bool force)
{
	struct signal_struct *signal = p->signal;
	struct task_struct *t;

	if (unlikely(signal->flags & SIGNAL_GROUP_EXIT)) {
	} else if (sig_kernel_stop(sig)) {
		rm_from_queue(sigmask(SIGCONT), &signal->shared_pending);
		t = p;
		do {
			rm_from_queue(sigmask(SIGCONT), &t->pending);
		} while_each_thread(p, t);
	} else if (sig == SIGCONT) {
		unsigned int why;
		rm_from_queue(SIG_KERNEL_STOP_MASK, &signal->shared_pending);
		t = p;
		do {
			task_clear_jobctl_pending(t, JOBCTL_STOP_PENDING);
			rm_from_queue(SIG_KERNEL_STOP_MASK, &t->pending);
			if (likely(!(t->ptrace & PT_SEIZED)))
				wake_up_state(t, __TASK_STOPPED);
			else
				ptrace_trap_notify(t);
		} while_each_thread(p, t);

		why = 0;
		if (signal->flags & SIGNAL_STOP_STOPPED)
			why |= SIGNAL_CLD_CONTINUED;
		else if (signal->group_stop_count)
			why |= SIGNAL_CLD_STOPPED;

		if (why) {
			signal->flags = why | SIGNAL_STOP_CONTINUED;
			signal->group_stop_count = 0;
			signal->group_exit_code = 0;
		}
	}

	return !sig_ignored(p, sig, force);
}

static inline int wants_signal(int sig, struct task_struct *p)
{
	if (sigismember(&p->blocked, sig))
		return 0;
	if (p->flags & PF_EXITING)
		return 0;
	if (sig == SIGKILL)
		return 1;
	if (task_is_stopped_or_traced(p))
		return 0;
	return task_curr(p) || !signal_pending(p);
}

static void complete_signal(int sig, struct task_struct *p, int group)
{
	struct signal_struct *signal = p->signal;
	struct task_struct *t;

	if (wants_signal(sig, p))
		t = p;
	else if (!group || thread_group_empty(p))
		return;
	else {
		t = signal->curr_target;
		while (!wants_signal(sig, t)) {
			t = next_thread(t);
			if (t == signal->curr_target)
				return;
		}
		signal->curr_target = t;
	}

	if (sig_fatal(p, sig) &&
	    !(signal->flags & (SIGNAL_UNKILLABLE | SIGNAL_GROUP_EXIT)) &&
	    !sigismember(&t->real_blocked, sig) &&
	    (sig == SIGKILL || !t->ptrace)) {
		if (!sig_kernel_coredump(sig)) {
			signal->flags = SIGNAL_GROUP_EXIT;
			signal->group_exit_code = sig;
			signal->group_stop_count = 0;
			t = p;
			do {
				task_clear_jobctl_pending(t, JOBCTL_PENDING_MASK);
				sigaddset(&t->pending.signal, SIGKILL);
				signal_wake_up(t, 1);
			} while_each_thread(p, t);
			return;
		}
	}

	signal_wake_up(t, sig == SIGKILL);
	return;
}

static inline int legacy_queue(struct sigpending *signals, int sig)
{
	return (sig < SIGRTMIN) && sigismember(&signals->signal, sig);
}

static inline uid_t map_cred_ns(const struct cred *cred,
				struct user_namespace *ns)
{
	return user_ns_map_uid(ns, cred, cred->uid);
}

#ifdef CONFIG_USER_NS
static inline void userns_fixup_signal_uid(struct siginfo *info, struct task_struct *t)
{
	if (current_user_ns() == task_cred_xxx(t, user_ns))
		return;

	if (SI_FROMKERNEL(info))
		return;

	info->si_uid = user_ns_map_uid(task_cred_xxx(t, user_ns),
					current_cred(), info->si_uid);
}
#else
static inline void userns_fixup_signal_uid(struct siginfo *info, struct task_struct *t)
{
	return;
}
#endif

static int __send_signal(int sig, struct siginfo *info, struct task_struct *t,
			int group, int from_ancestor_ns)
{
	struct sigpending *pending;
	struct sigqueue *q;
	int override_rlimit;
	int ret = 0, result;

	assert_spin_locked(&t->sighand->siglock);

	result = TRACE_SIGNAL_IGNORED;
	if (!prepare_signal(sig, t,
			from_ancestor_ns || (info == SEND_SIG_FORCED)))
		goto ret;

	pending = group ? &t->signal->shared_pending : &t->pending;
	result = TRACE_SIGNAL_ALREADY_PENDING;
	if (legacy_queue(pending, sig))
		goto ret;

	result = TRACE_SIGNAL_DELIVERED;
	if (info == SEND_SIG_FORCED)
		goto out_set;

	if (sig < SIGRTMIN)
		override_rlimit = (is_si_special(info) || info->si_code >= 0);
	else
		override_rlimit = 0;

	q = __sigqueue_alloc(sig, t, GFP_ATOMIC | __GFP_NOTRACK_FALSE_POSITIVE,
		override_rlimit);
	if (q) {
		list_add_tail(&q->list, &pending->list);
		switch ((unsigned long) info) {
		case (unsigned long) SEND_SIG_NOINFO:
			q->info.si_signo = sig;
			q->info.si_errno = 0;
			q->info.si_code = SI_USER;
			q->info.si_pid = task_tgid_nr_ns(current,
							task_active_pid_ns(t));
			q->info.si_uid = current_uid();
			break;
		case (unsigned long) SEND_SIG_PRIV:
			q->info.si_signo = sig;
			q->info.si_errno = 0;
			q->info.si_code = SI_KERNEL;
			q->info.si_pid = 0;
			q->info.si_uid = 0;
			break;
		default:
			copy_siginfo(&q->info, info);
			if (from_ancestor_ns)
				q->info.si_pid = 0;
			break;
		}

		userns_fixup_signal_uid(&q->info, t);

	} else if (!is_si_special(info)) {
		if (sig >= SIGRTMIN && info->si_code != SI_USER) {
			result = TRACE_SIGNAL_OVERFLOW_FAIL;
			ret = -EAGAIN;
			goto ret;
		} else {
			result = TRACE_SIGNAL_LOSE_INFO;
		}
	}

out_set:
	signalfd_notify(t, sig);
	sigaddset(&pending->signal, sig);
	complete_signal(sig, t, group);
ret:
	trace_signal_generate(sig, info, t, group, result);
	return ret;
}

#if defined(CONFIG_HTC_DEBUG_DYING_PROCS)
#include <linux/proc_fs.h>
#define MAX_DYING_PROC_COUNT (10)
struct dying_pid {
	pid_t pid;
	unsigned long jiffy;
};
static DEFINE_SPINLOCK(dying_pid_lock);
static int sigkill_pending(struct task_struct *tsk);
static struct dying_pid dying_pid_buf[MAX_DYING_PROC_COUNT];
static unsigned int dying_pid_buf_idx;

static int dying_processors_read_proc(char *page, char **start, off_t off,
			   int count, int *eof, void *data)
{
	char *p = page;
	unsigned long jiffy = jiffies;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&dying_pid_lock, flags);
	for (i = 0; count > 0 && i < MAX_DYING_PROC_COUNT; i++)
		if (dying_pid_buf[i].pid) {
			int writes = scnprintf(p, count,
					"%ld:%ld\n",
					(long int) dying_pid_buf[i].pid,
					jiffy - dying_pid_buf[i].jiffy);
			count -= writes;
			p += writes;
		}
	spin_unlock_irqrestore(&dying_pid_lock, flags);

	return p - page;
}

static int __init dying_processors_init(void)
{
	memset(dying_pid_buf, 0, sizeof(dying_pid_buf));
	create_proc_read_entry("dying_processes", 0, NULL,
			dying_processors_read_proc, NULL);
	return 0;
}
module_init(dying_processors_init);
#endif

static int send_signal(int sig, struct siginfo *info, struct task_struct *t,
			int group)
{
	int from_ancestor_ns = 0;
#ifdef CONFIG_HTC_PROCESS_DEBUG
	send_signal_debug_dump(sig, t);
#endif

#if defined(CONFIG_HTC_DEBUG_DYING_PROCS)
	if (sig == SIGKILL && !sigkill_pending(t)) {
		unsigned long flags;
		spin_lock_irqsave(&dying_pid_lock, flags);
		dying_pid_buf_idx = ((dying_pid_buf_idx + 1) % MAX_DYING_PROC_COUNT);
		dying_pid_buf[dying_pid_buf_idx].pid = t->pid;
		dying_pid_buf[dying_pid_buf_idx].jiffy = jiffies;
		spin_unlock_irqrestore(&dying_pid_lock, flags);
	}
#endif

#ifdef CONFIG_PID_NS
	from_ancestor_ns = si_fromuser(info) &&
			   !task_pid_nr_ns(current, task_active_pid_ns(t));
#endif

	return __send_signal(sig, info, t, group, from_ancestor_ns);
}

static void print_fatal_signal(struct pt_regs *regs, int signr)
{
	printk("%s/%d: potentially unexpected fatal signal %d.\n",
		current->comm, task_pid_nr(current), signr);

#if defined(__i386__) && !defined(__arch_um__)
	printk("code at %08lx: ", regs->ip);
	{
		int i;
		for (i = 0; i < 16; i++) {
			unsigned char insn;

			if (get_user(insn, (unsigned char *)(regs->ip + i)))
				break;
			printk("%02x ", insn);
		}
	}
#endif
	printk("\n");
	preempt_disable();
	show_regs(regs);
	preempt_enable();
}

static int __init setup_print_fatal_signals(char *str)
{
	get_option (&str, &print_fatal_signals);

	return 1;
}

__setup("print-fatal-signals=", setup_print_fatal_signals);

int
__group_send_sig_info(int sig, struct siginfo *info, struct task_struct *p)
{
	return send_signal(sig, info, p, 1);
}

static int
specific_send_sig_info(int sig, struct siginfo *info, struct task_struct *t)
{
	return send_signal(sig, info, t, 0);
}

int do_send_sig_info(int sig, struct siginfo *info, struct task_struct *p,
			bool group)
{
	unsigned long flags;
	int ret = -ESRCH;

	if (lock_task_sighand(p, &flags)) {
		ret = send_signal(sig, info, p, group);
		unlock_task_sighand(p, &flags);
	}

	return ret;
}

int
force_sig_info(int sig, struct siginfo *info, struct task_struct *t)
{
	unsigned long int flags;
	int ret, blocked, ignored;
	struct k_sigaction *action;

	spin_lock_irqsave(&t->sighand->siglock, flags);
	action = &t->sighand->action[sig-1];
	ignored = action->sa.sa_handler == SIG_IGN;
	blocked = sigismember(&t->blocked, sig);
	if (blocked || ignored) {
		action->sa.sa_handler = SIG_DFL;
		if (blocked) {
			sigdelset(&t->blocked, sig);
			recalc_sigpending_and_wake(t);
		}
	}
	if (action->sa.sa_handler == SIG_DFL)
		t->signal->flags &= ~SIGNAL_UNKILLABLE;
	ret = specific_send_sig_info(sig, info, t);
	spin_unlock_irqrestore(&t->sighand->siglock, flags);

	return ret;
}

int zap_other_threads(struct task_struct *p)
{
	struct task_struct *t = p;
	int count = 0;

	p->signal->group_stop_count = 0;

	while_each_thread(p, t) {
		task_clear_jobctl_pending(t, JOBCTL_PENDING_MASK);
		count++;

		
		if (t->exit_state)
			continue;
		sigaddset(&t->pending.signal, SIGKILL);
		signal_wake_up(t, 1);
	}

	return count;
}

struct sighand_struct *__lock_task_sighand(struct task_struct *tsk,
					   unsigned long *flags)
{
	struct sighand_struct *sighand;

	for (;;) {
		local_irq_save(*flags);
		rcu_read_lock();
		sighand = rcu_dereference(tsk->sighand);
		if (unlikely(sighand == NULL)) {
			rcu_read_unlock();
			local_irq_restore(*flags);
			break;
		}

		spin_lock(&sighand->siglock);
		if (likely(sighand == tsk->sighand)) {
			rcu_read_unlock();
			break;
		}
		spin_unlock(&sighand->siglock);
		rcu_read_unlock();
		local_irq_restore(*flags);
	}

	return sighand;
}

int group_send_sig_info(int sig, struct siginfo *info, struct task_struct *p)
{
	int ret;

	rcu_read_lock();
	ret = check_kill_permission(sig, info, p);
	rcu_read_unlock();

	if (!ret && sig)
		ret = do_send_sig_info(sig, info, p, true);

	return ret;
}

int __kill_pgrp_info(int sig, struct siginfo *info, struct pid *pgrp)
{
	struct task_struct *p = NULL;
	int retval, success;

	success = 0;
	retval = -ESRCH;
	do_each_pid_task(pgrp, PIDTYPE_PGID, p) {
		int err = group_send_sig_info(sig, info, p);
		success |= !err;
		retval = err;
	} while_each_pid_task(pgrp, PIDTYPE_PGID, p);
	return success ? 0 : retval;
}

int kill_pid_info(int sig, struct siginfo *info, struct pid *pid)
{
	int error = -ESRCH;
	struct task_struct *p;

	rcu_read_lock();
retry:
	p = pid_task(pid, PIDTYPE_PID);
	if (p) {
		error = group_send_sig_info(sig, info, p);
		if (unlikely(error == -ESRCH))
			goto retry;
	}
	rcu_read_unlock();

	return error;
}

int kill_proc_info(int sig, struct siginfo *info, pid_t pid)
{
	int error;
	rcu_read_lock();
	error = kill_pid_info(sig, info, find_vpid(pid));
	rcu_read_unlock();
	return error;
}

static int kill_as_cred_perm(const struct cred *cred,
			     struct task_struct *target)
{
	const struct cred *pcred = __task_cred(target);
	if (cred->user_ns != pcred->user_ns)
		return 0;
	if (cred->euid != pcred->suid && cred->euid != pcred->uid &&
	    cred->uid  != pcred->suid && cred->uid  != pcred->uid)
		return 0;
	return 1;
}

int kill_pid_info_as_cred(int sig, struct siginfo *info, struct pid *pid,
			 const struct cred *cred, u32 secid)
{
	int ret = -EINVAL;
	struct task_struct *p;
	unsigned long flags;

	if (!valid_signal(sig))
		return ret;

	rcu_read_lock();
	p = pid_task(pid, PIDTYPE_PID);
	if (!p) {
		ret = -ESRCH;
		goto out_unlock;
	}
	if (si_fromuser(info) && !kill_as_cred_perm(cred, p)) {
		ret = -EPERM;
		goto out_unlock;
	}
	ret = security_task_kill(p, info, sig, secid);
	if (ret)
		goto out_unlock;

	if (sig) {
		if (lock_task_sighand(p, &flags)) {
			ret = __send_signal(sig, info, p, 1, 0);
			unlock_task_sighand(p, &flags);
		} else
			ret = -ESRCH;
	}
out_unlock:
	rcu_read_unlock();
	return ret;
}
EXPORT_SYMBOL_GPL(kill_pid_info_as_cred);


static int kill_something_info(int sig, struct siginfo *info, pid_t pid)
{
	int ret;

	if (pid > 0) {
		rcu_read_lock();
		ret = kill_pid_info(sig, info, find_vpid(pid));
		rcu_read_unlock();
		return ret;
	}

	read_lock(&tasklist_lock);
	if (pid != -1) {
		ret = __kill_pgrp_info(sig, info,
				pid ? find_vpid(-pid) : task_pgrp(current));
	} else {
		int retval = 0, count = 0;
		struct task_struct * p;

		for_each_process(p) {
			if (task_pid_vnr(p) > 1 &&
					!same_thread_group(p, current)) {
				int err = group_send_sig_info(sig, info, p);
				++count;
				if (err != -EPERM)
					retval = err;
			}
		}
		ret = count ? retval : -ESRCH;
	}
	read_unlock(&tasklist_lock);

	return ret;
}


int send_sig_info(int sig, struct siginfo *info, struct task_struct *p)
{
	if (!valid_signal(sig))
		return -EINVAL;

	return do_send_sig_info(sig, info, p, false);
}

#define __si_special(priv) \
	((priv) ? SEND_SIG_PRIV : SEND_SIG_NOINFO)

int
send_sig(int sig, struct task_struct *p, int priv)
{
	return send_sig_info(sig, __si_special(priv), p);
}

void
force_sig(int sig, struct task_struct *p)
{
	force_sig_info(sig, SEND_SIG_PRIV, p);
}

int
force_sigsegv(int sig, struct task_struct *p)
{
	if (sig == SIGSEGV) {
		unsigned long flags;
		spin_lock_irqsave(&p->sighand->siglock, flags);
		p->sighand->action[sig - 1].sa.sa_handler = SIG_DFL;
		spin_unlock_irqrestore(&p->sighand->siglock, flags);
	}
	force_sig(SIGSEGV, p);
	return 0;
}

int kill_pgrp(struct pid *pid, int sig, int priv)
{
	int ret;

	read_lock(&tasklist_lock);
	ret = __kill_pgrp_info(sig, __si_special(priv), pid);
	read_unlock(&tasklist_lock);

	return ret;
}
EXPORT_SYMBOL(kill_pgrp);

int kill_pid(struct pid *pid, int sig, int priv)
{
	return kill_pid_info(sig, __si_special(priv), pid);
}
EXPORT_SYMBOL(kill_pid);

struct sigqueue *sigqueue_alloc(void)
{
	struct sigqueue *q = __sigqueue_alloc(-1, current, GFP_KERNEL, 0);

	if (q)
		q->flags |= SIGQUEUE_PREALLOC;

	return q;
}

void sigqueue_free(struct sigqueue *q)
{
	unsigned long flags;
	spinlock_t *lock = &current->sighand->siglock;

	BUG_ON(!(q->flags & SIGQUEUE_PREALLOC));
	spin_lock_irqsave(lock, flags);
	q->flags &= ~SIGQUEUE_PREALLOC;
	if (!list_empty(&q->list))
		q = NULL;
	spin_unlock_irqrestore(lock, flags);

	if (q)
		__sigqueue_free(q);
}

int send_sigqueue(struct sigqueue *q, struct task_struct *t, int group)
{
	int sig = q->info.si_signo;
	struct sigpending *pending;
	unsigned long flags;
	int ret, result;

	BUG_ON(!(q->flags & SIGQUEUE_PREALLOC));

	ret = -1;
	if (!likely(lock_task_sighand(t, &flags)))
		goto ret;

	ret = 1; 
	result = TRACE_SIGNAL_IGNORED;
	if (!prepare_signal(sig, t, false))
		goto out;

	ret = 0;
	if (unlikely(!list_empty(&q->list))) {
		BUG_ON(q->info.si_code != SI_TIMER);
		q->info.si_overrun++;
		result = TRACE_SIGNAL_ALREADY_PENDING;
		goto out;
	}
	q->info.si_overrun = 0;

	signalfd_notify(t, sig);
	pending = group ? &t->signal->shared_pending : &t->pending;
	list_add_tail(&q->list, &pending->list);
	sigaddset(&pending->signal, sig);
	complete_signal(sig, t, group);
	result = TRACE_SIGNAL_DELIVERED;
out:
	trace_signal_generate(sig, &q->info, t, group, result);
	unlock_task_sighand(t, &flags);
ret:
	return ret;
}

bool do_notify_parent(struct task_struct *tsk, int sig)
{
	struct siginfo info;
	unsigned long flags;
	struct sighand_struct *psig;
	bool autoreap = false;

	BUG_ON(sig == -1);

 	
 	BUG_ON(task_is_stopped_or_traced(tsk));

	BUG_ON(!tsk->ptrace &&
	       (tsk->group_leader != tsk || !thread_group_empty(tsk)));

	if (sig != SIGCHLD) {
		if (tsk->parent_exec_id != tsk->parent->self_exec_id)
			sig = SIGCHLD;
	}

	info.si_signo = sig;
	info.si_errno = 0;
	rcu_read_lock();
	info.si_pid = task_pid_nr_ns(tsk, tsk->parent->nsproxy->pid_ns);
	info.si_uid = map_cred_ns(__task_cred(tsk),
			task_cred_xxx(tsk->parent, user_ns));
	rcu_read_unlock();

	info.si_utime = cputime_to_clock_t(tsk->utime + tsk->signal->utime);
	info.si_stime = cputime_to_clock_t(tsk->stime + tsk->signal->stime);

	info.si_status = tsk->exit_code & 0x7f;
	if (tsk->exit_code & 0x80)
		info.si_code = CLD_DUMPED;
	else if (tsk->exit_code & 0x7f)
		info.si_code = CLD_KILLED;
	else {
		info.si_code = CLD_EXITED;
		info.si_status = tsk->exit_code >> 8;
	}

	psig = tsk->parent->sighand;
	spin_lock_irqsave(&psig->siglock, flags);
	if (!tsk->ptrace && sig == SIGCHLD &&
	    (psig->action[SIGCHLD-1].sa.sa_handler == SIG_IGN ||
	     (psig->action[SIGCHLD-1].sa.sa_flags & SA_NOCLDWAIT))) {
		autoreap = true;
		if (psig->action[SIGCHLD-1].sa.sa_handler == SIG_IGN)
			sig = 0;
	}
	if (valid_signal(sig) && sig)
		__group_send_sig_info(sig, &info, tsk->parent);
	__wake_up_parent(tsk, tsk->parent);
	spin_unlock_irqrestore(&psig->siglock, flags);

	return autoreap;
}

static void do_notify_parent_cldstop(struct task_struct *tsk,
				     bool for_ptracer, int why)
{
	struct siginfo info;
	unsigned long flags;
	struct task_struct *parent;
	struct sighand_struct *sighand;

	if (for_ptracer) {
		parent = tsk->parent;
	} else {
		tsk = tsk->group_leader;
		parent = tsk->real_parent;
	}

	info.si_signo = SIGCHLD;
	info.si_errno = 0;
	rcu_read_lock();
	info.si_pid = task_pid_nr_ns(tsk, parent->nsproxy->pid_ns);
	info.si_uid = map_cred_ns(__task_cred(tsk),
			task_cred_xxx(parent, user_ns));
	rcu_read_unlock();

	info.si_utime = cputime_to_clock_t(tsk->utime);
	info.si_stime = cputime_to_clock_t(tsk->stime);

 	info.si_code = why;
 	switch (why) {
 	case CLD_CONTINUED:
 		info.si_status = SIGCONT;
 		break;
 	case CLD_STOPPED:
 		info.si_status = tsk->signal->group_exit_code & 0x7f;
 		break;
 	case CLD_TRAPPED:
 		info.si_status = tsk->exit_code & 0x7f;
 		break;
 	default:
 		BUG();
 	}

	sighand = parent->sighand;
	spin_lock_irqsave(&sighand->siglock, flags);
	if (sighand->action[SIGCHLD-1].sa.sa_handler != SIG_IGN &&
	    !(sighand->action[SIGCHLD-1].sa.sa_flags & SA_NOCLDSTOP))
		__group_send_sig_info(SIGCHLD, &info, parent);
	__wake_up_parent(tsk, parent);
	spin_unlock_irqrestore(&sighand->siglock, flags);
}

static inline int may_ptrace_stop(void)
{
	if (!likely(current->ptrace))
		return 0;
	if (unlikely(current->mm->core_state) &&
	    unlikely(current->mm == current->parent->mm))
		return 0;

	return 1;
}

static int sigkill_pending(struct task_struct *tsk)
{
	return	sigismember(&tsk->pending.signal, SIGKILL) ||
		sigismember(&tsk->signal->shared_pending.signal, SIGKILL);
}

static void ptrace_stop(int exit_code, int why, int clear_code, siginfo_t *info)
	__releases(&current->sighand->siglock)
	__acquires(&current->sighand->siglock)
{
	bool gstop_done = false;

	if (arch_ptrace_stop_needed(exit_code, info)) {
		spin_unlock_irq(&current->sighand->siglock);
		arch_ptrace_stop(exit_code, info);
		spin_lock_irq(&current->sighand->siglock);
		if (sigkill_pending(current))
			return;
	}

	set_current_state(TASK_TRACED);

	current->last_siginfo = info;
	current->exit_code = exit_code;

	if (why == CLD_STOPPED && (current->jobctl & JOBCTL_STOP_PENDING))
		gstop_done = task_participate_group_stop(current);

	
	task_clear_jobctl_pending(current, JOBCTL_TRAP_STOP);
	if (info && info->si_code >> 8 == PTRACE_EVENT_STOP)
		task_clear_jobctl_pending(current, JOBCTL_TRAP_NOTIFY);

	
	task_clear_jobctl_trapping(current);

	spin_unlock_irq(&current->sighand->siglock);
	read_lock(&tasklist_lock);
	if (may_ptrace_stop()) {
		do_notify_parent_cldstop(current, true, why);
		if (gstop_done && ptrace_reparented(current))
			do_notify_parent_cldstop(current, false, why);

		preempt_disable();
		read_unlock(&tasklist_lock);
		preempt_enable_no_resched();
		schedule();
	} else {
		if (gstop_done)
			do_notify_parent_cldstop(current, false, why);

		__set_current_state(TASK_RUNNING);
		if (clear_code)
			current->exit_code = 0;
		read_unlock(&tasklist_lock);
	}

	try_to_freeze();

	spin_lock_irq(&current->sighand->siglock);
	current->last_siginfo = NULL;

	
	current->jobctl &= ~JOBCTL_LISTENING;

	recalc_sigpending_tsk(current);
}

static void ptrace_do_notify(int signr, int exit_code, int why)
{
	siginfo_t info;

	memset(&info, 0, sizeof info);
	info.si_signo = signr;
	info.si_code = exit_code;
	info.si_pid = task_pid_vnr(current);
	info.si_uid = current_uid();

	
	ptrace_stop(exit_code, why, 1, &info);
}

void ptrace_notify(int exit_code)
{
	BUG_ON((exit_code & (0x7f | ~0xffff)) != SIGTRAP);

	spin_lock_irq(&current->sighand->siglock);
	ptrace_do_notify(SIGTRAP, exit_code, CLD_TRAPPED);
	spin_unlock_irq(&current->sighand->siglock);
}

static bool do_signal_stop(int signr)
	__releases(&current->sighand->siglock)
{
	struct signal_struct *sig = current->signal;

	if (!(current->jobctl & JOBCTL_STOP_PENDING)) {
		unsigned int gstop = JOBCTL_STOP_PENDING | JOBCTL_STOP_CONSUME;
		struct task_struct *t;

		
		WARN_ON_ONCE(signr & ~JOBCTL_STOP_SIGMASK);

		if (!likely(current->jobctl & JOBCTL_STOP_DEQUEUED) ||
		    unlikely(signal_group_exit(sig)))
			return false;
		if (!(sig->flags & SIGNAL_STOP_STOPPED))
			sig->group_exit_code = signr;

		sig->group_stop_count = 0;

		if (task_set_jobctl_pending(current, signr | gstop))
			sig->group_stop_count++;

		for (t = next_thread(current); t != current;
		     t = next_thread(t)) {
			if (!task_is_stopped(t) &&
			    task_set_jobctl_pending(t, signr | gstop)) {
				sig->group_stop_count++;
				if (likely(!(t->ptrace & PT_SEIZED)))
					signal_wake_up(t, 0);
				else
					ptrace_trap_notify(t);
			}
		}
	}

	if (likely(!current->ptrace)) {
		int notify = 0;

		if (task_participate_group_stop(current))
			notify = CLD_STOPPED;

		__set_current_state(TASK_STOPPED);
		spin_unlock_irq(&current->sighand->siglock);

		if (notify) {
			read_lock(&tasklist_lock);
			do_notify_parent_cldstop(current, false, notify);
			read_unlock(&tasklist_lock);
		}

		
		schedule();
		return true;
	} else {
		task_set_jobctl_pending(current, JOBCTL_TRAP_STOP);
		return false;
	}
}

static void do_jobctl_trap(void)
{
	struct signal_struct *signal = current->signal;
	int signr = current->jobctl & JOBCTL_STOP_SIGMASK;

	if (current->ptrace & PT_SEIZED) {
		if (!signal->group_stop_count &&
		    !(signal->flags & SIGNAL_STOP_STOPPED))
			signr = SIGTRAP;
		WARN_ON_ONCE(!signr);
		ptrace_do_notify(signr, signr | (PTRACE_EVENT_STOP << 8),
				 CLD_STOPPED);
	} else {
		WARN_ON_ONCE(!signr);
		ptrace_stop(signr, CLD_STOPPED, 0, NULL);
		current->exit_code = 0;
	}
}

static int ptrace_signal(int signr, siginfo_t *info,
			 struct pt_regs *regs, void *cookie)
{
	ptrace_signal_deliver(regs, cookie);
	current->jobctl |= JOBCTL_STOP_DEQUEUED;
	ptrace_stop(signr, CLD_TRAPPED, 0, info);

	
	signr = current->exit_code;
	if (signr == 0)
		return signr;

	current->exit_code = 0;

	if (signr != info->si_signo) {
		info->si_signo = signr;
		info->si_errno = 0;
		info->si_code = SI_USER;
		rcu_read_lock();
		info->si_pid = task_pid_vnr(current->parent);
		info->si_uid = map_cred_ns(__task_cred(current->parent),
				current_user_ns());
		rcu_read_unlock();
	}

	
	if (sigismember(&current->blocked, signr)) {
		specific_send_sig_info(signr, info, current);
		signr = 0;
	}

	return signr;
}

int get_signal_to_deliver(siginfo_t *info, struct k_sigaction *return_ka,
			  struct pt_regs *regs, void *cookie)
{
	struct sighand_struct *sighand = current->sighand;
	struct signal_struct *signal = current->signal;
	int signr;

relock:
	try_to_freeze();

	spin_lock_irq(&sighand->siglock);
	if (unlikely(signal->flags & SIGNAL_CLD_MASK)) {
		int why;

		if (signal->flags & SIGNAL_CLD_CONTINUED)
			why = CLD_CONTINUED;
		else
			why = CLD_STOPPED;

		signal->flags &= ~SIGNAL_CLD_MASK;

		spin_unlock_irq(&sighand->siglock);

		read_lock(&tasklist_lock);
		do_notify_parent_cldstop(current, false, why);

		if (ptrace_reparented(current->group_leader))
			do_notify_parent_cldstop(current->group_leader,
						true, why);
		read_unlock(&tasklist_lock);

		goto relock;
	}

	for (;;) {
		struct k_sigaction *ka;

		if (unlikely(current->jobctl & JOBCTL_STOP_PENDING) &&
		    do_signal_stop(0))
			goto relock;

		if (unlikely(current->jobctl & JOBCTL_TRAP_MASK)) {
			do_jobctl_trap();
			spin_unlock_irq(&sighand->siglock);
			goto relock;
		}

		signr = dequeue_signal(current, &current->blocked, info);

		if (!signr)
			break; 

		if (unlikely(current->ptrace) && signr != SIGKILL) {
			signr = ptrace_signal(signr, info,
					      regs, cookie);
			if (!signr)
				continue;
		}

		ka = &sighand->action[signr-1];

		
		trace_signal_deliver(signr, info, ka);

		if (ka->sa.sa_handler == SIG_IGN) 
			continue;
		if (ka->sa.sa_handler != SIG_DFL) {
			
			*return_ka = *ka;

			if (ka->sa.sa_flags & SA_ONESHOT)
				ka->sa.sa_handler = SIG_DFL;

			break; 
		}

		if (sig_kernel_ignore(signr)) 
			continue;

		if (unlikely(signal->flags & SIGNAL_UNKILLABLE) &&
				!sig_kernel_only(signr))
			continue;

		if (sig_kernel_stop(signr)) {
			if (signr != SIGSTOP) {
				spin_unlock_irq(&sighand->siglock);

				

				if (is_current_pgrp_orphaned())
					goto relock;

				spin_lock_irq(&sighand->siglock);
			}

			if (likely(do_signal_stop(info->si_signo))) {
				
				goto relock;
			}

			continue;
		}

		spin_unlock_irq(&sighand->siglock);

		current->flags |= PF_SIGNALED;

		if (sig_kernel_coredump(signr)) {
			if (print_fatal_signals)
				print_fatal_signal(regs, info->si_signo);
			do_coredump(info->si_signo, info->si_signo, regs);
		}

		do_group_exit(info->si_signo);
		
	}
	spin_unlock_irq(&sighand->siglock);
	return signr;
}

void block_sigmask(struct k_sigaction *ka, int signr)
{
	sigset_t blocked;

	sigorsets(&blocked, &current->blocked, &ka->sa.sa_mask);
	if (!(ka->sa.sa_flags & SA_NODEFER))
		sigaddset(&blocked, signr);
	set_current_blocked(&blocked);
}

static void retarget_shared_pending(struct task_struct *tsk, sigset_t *which)
{
	sigset_t retarget;
	struct task_struct *t;

	sigandsets(&retarget, &tsk->signal->shared_pending.signal, which);
	if (sigisemptyset(&retarget))
		return;

	t = tsk;
	while_each_thread(tsk, t) {
		if (t->flags & PF_EXITING)
			continue;

		if (!has_pending_signals(&retarget, &t->blocked))
			continue;
		
		sigandsets(&retarget, &retarget, &t->blocked);

		if (!signal_pending(t))
			signal_wake_up(t, 0);

		if (sigisemptyset(&retarget))
			break;
	}
}

void exit_signals(struct task_struct *tsk)
{
	int group_stop = 0;
	sigset_t unblocked;

	threadgroup_change_begin(tsk);

	if (thread_group_empty(tsk) || signal_group_exit(tsk->signal)) {
		tsk->flags |= PF_EXITING;
		threadgroup_change_end(tsk);
		return;
	}

	spin_lock_irq(&tsk->sighand->siglock);
	tsk->flags |= PF_EXITING;

	threadgroup_change_end(tsk);

	if (!signal_pending(tsk))
		goto out;

	unblocked = tsk->blocked;
	signotset(&unblocked);
	retarget_shared_pending(tsk, &unblocked);

	if (unlikely(tsk->jobctl & JOBCTL_STOP_PENDING) &&
	    task_participate_group_stop(tsk))
		group_stop = CLD_STOPPED;
out:
	spin_unlock_irq(&tsk->sighand->siglock);

	if (unlikely(group_stop)) {
		read_lock(&tasklist_lock);
		do_notify_parent_cldstop(tsk, false, group_stop);
		read_unlock(&tasklist_lock);
	}
}

EXPORT_SYMBOL(recalc_sigpending);
EXPORT_SYMBOL_GPL(dequeue_signal);
EXPORT_SYMBOL(flush_signals);
EXPORT_SYMBOL(force_sig);
EXPORT_SYMBOL(send_sig);
EXPORT_SYMBOL(send_sig_info);
EXPORT_SYMBOL(sigprocmask);
EXPORT_SYMBOL(block_all_signals);
EXPORT_SYMBOL(unblock_all_signals);



SYSCALL_DEFINE0(restart_syscall)
{
	struct restart_block *restart = &current_thread_info()->restart_block;
	return restart->fn(restart);
}

long do_no_restart_syscall(struct restart_block *param)
{
	return -EINTR;
}

static void __set_task_blocked(struct task_struct *tsk, const sigset_t *newset)
{
	if (signal_pending(tsk) && !thread_group_empty(tsk)) {
		sigset_t newblocked;
		
		sigandnsets(&newblocked, newset, &current->blocked);
		retarget_shared_pending(tsk, &newblocked);
	}
	tsk->blocked = *newset;
	recalc_sigpending();
}

void set_current_blocked(const sigset_t *newset)
{
	struct task_struct *tsk = current;

	spin_lock_irq(&tsk->sighand->siglock);
	__set_task_blocked(tsk, newset);
	spin_unlock_irq(&tsk->sighand->siglock);
}

int sigprocmask(int how, sigset_t *set, sigset_t *oldset)
{
	struct task_struct *tsk = current;
	sigset_t newset;

	
	if (oldset)
		*oldset = tsk->blocked;

	switch (how) {
	case SIG_BLOCK:
		sigorsets(&newset, &tsk->blocked, set);
		break;
	case SIG_UNBLOCK:
		sigandnsets(&newset, &tsk->blocked, set);
		break;
	case SIG_SETMASK:
		newset = *set;
		break;
	default:
		return -EINVAL;
	}

	set_current_blocked(&newset);
	return 0;
}

SYSCALL_DEFINE4(rt_sigprocmask, int, how, sigset_t __user *, nset,
		sigset_t __user *, oset, size_t, sigsetsize)
{
	sigset_t old_set, new_set;
	int error;

	
	if (sigsetsize != sizeof(sigset_t))
		return -EINVAL;

	old_set = current->blocked;

	if (nset) {
		if (copy_from_user(&new_set, nset, sizeof(sigset_t)))
			return -EFAULT;
		sigdelsetmask(&new_set, sigmask(SIGKILL)|sigmask(SIGSTOP));

		error = sigprocmask(how, &new_set, NULL);
		if (error)
			return error;
	}

	if (oset) {
		if (copy_to_user(oset, &old_set, sizeof(sigset_t)))
			return -EFAULT;
	}

	return 0;
}

long do_sigpending(void __user *set, unsigned long sigsetsize)
{
	long error = -EINVAL;
	sigset_t pending;

	if (sigsetsize > sizeof(sigset_t))
		goto out;

	spin_lock_irq(&current->sighand->siglock);
	sigorsets(&pending, &current->pending.signal,
		  &current->signal->shared_pending.signal);
	spin_unlock_irq(&current->sighand->siglock);

	
	sigandsets(&pending, &current->blocked, &pending);

	error = -EFAULT;
	if (!copy_to_user(set, &pending, sigsetsize))
		error = 0;

out:
	return error;
}

SYSCALL_DEFINE2(rt_sigpending, sigset_t __user *, set, size_t, sigsetsize)
{
	return do_sigpending(set, sigsetsize);
}

#ifndef HAVE_ARCH_COPY_SIGINFO_TO_USER

int copy_siginfo_to_user(siginfo_t __user *to, siginfo_t *from)
{
	int err;

	if (!access_ok (VERIFY_WRITE, to, sizeof(siginfo_t)))
		return -EFAULT;
	if (from->si_code < 0)
		return __copy_to_user(to, from, sizeof(siginfo_t))
			? -EFAULT : 0;
	err = __put_user(from->si_signo, &to->si_signo);
	err |= __put_user(from->si_errno, &to->si_errno);
	err |= __put_user((short)from->si_code, &to->si_code);
	switch (from->si_code & __SI_MASK) {
	case __SI_KILL:
		err |= __put_user(from->si_pid, &to->si_pid);
		err |= __put_user(from->si_uid, &to->si_uid);
		break;
	case __SI_TIMER:
		 err |= __put_user(from->si_tid, &to->si_tid);
		 err |= __put_user(from->si_overrun, &to->si_overrun);
		 err |= __put_user(from->si_ptr, &to->si_ptr);
		break;
	case __SI_POLL:
		err |= __put_user(from->si_band, &to->si_band);
		err |= __put_user(from->si_fd, &to->si_fd);
		break;
	case __SI_FAULT:
		err |= __put_user(from->si_addr, &to->si_addr);
#ifdef __ARCH_SI_TRAPNO
		err |= __put_user(from->si_trapno, &to->si_trapno);
#endif
#ifdef BUS_MCEERR_AO
		if (from->si_code == BUS_MCEERR_AR || from->si_code == BUS_MCEERR_AO)
			err |= __put_user(from->si_addr_lsb, &to->si_addr_lsb);
#endif
		break;
	case __SI_CHLD:
		err |= __put_user(from->si_pid, &to->si_pid);
		err |= __put_user(from->si_uid, &to->si_uid);
		err |= __put_user(from->si_status, &to->si_status);
		err |= __put_user(from->si_utime, &to->si_utime);
		err |= __put_user(from->si_stime, &to->si_stime);
		break;
	case __SI_RT: 
	case __SI_MESGQ: 
		err |= __put_user(from->si_pid, &to->si_pid);
		err |= __put_user(from->si_uid, &to->si_uid);
		err |= __put_user(from->si_ptr, &to->si_ptr);
		break;
	default: 
		err |= __put_user(from->si_pid, &to->si_pid);
		err |= __put_user(from->si_uid, &to->si_uid);
		break;
	}
	return err;
}

#endif

int do_sigtimedwait(const sigset_t *which, siginfo_t *info,
			const struct timespec *ts)
{
	struct task_struct *tsk = current;
	long timeout = MAX_SCHEDULE_TIMEOUT;
	sigset_t mask = *which;
	int sig;

	if (ts) {
		if (!timespec_valid(ts))
			return -EINVAL;
		timeout = timespec_to_jiffies(ts);
		if (ts->tv_sec || ts->tv_nsec)
			timeout++;
	}

	sigdelsetmask(&mask, sigmask(SIGKILL) | sigmask(SIGSTOP));
	signotset(&mask);

	spin_lock_irq(&tsk->sighand->siglock);
	sig = dequeue_signal(tsk, &mask, info);
	if (!sig && timeout) {
		tsk->real_blocked = tsk->blocked;
		sigandsets(&tsk->blocked, &tsk->blocked, &mask);
		recalc_sigpending();
		spin_unlock_irq(&tsk->sighand->siglock);

		timeout = schedule_timeout_interruptible(timeout);

		spin_lock_irq(&tsk->sighand->siglock);
		__set_task_blocked(tsk, &tsk->real_blocked);
		siginitset(&tsk->real_blocked, 0);
		sig = dequeue_signal(tsk, &mask, info);
	}
	spin_unlock_irq(&tsk->sighand->siglock);

	if (sig)
		return sig;
	return timeout ? -EINTR : -EAGAIN;
}

SYSCALL_DEFINE4(rt_sigtimedwait, const sigset_t __user *, uthese,
		siginfo_t __user *, uinfo, const struct timespec __user *, uts,
		size_t, sigsetsize)
{
	sigset_t these;
	struct timespec ts;
	siginfo_t info;
	int ret;

	
	if (sigsetsize != sizeof(sigset_t))
		return -EINVAL;

	if (copy_from_user(&these, uthese, sizeof(these)))
		return -EFAULT;

	if (uts) {
		if (copy_from_user(&ts, uts, sizeof(ts)))
			return -EFAULT;
	}

	ret = do_sigtimedwait(&these, &info, uts ? &ts : NULL);

	if (ret > 0 && uinfo) {
		if (copy_siginfo_to_user(uinfo, &info))
			ret = -EFAULT;
	}

	return ret;
}

SYSCALL_DEFINE2(kill, pid_t, pid, int, sig)
{
	struct siginfo info;

	info.si_signo = sig;
	info.si_errno = 0;
	info.si_code = SI_USER;
	info.si_pid = task_tgid_vnr(current);
	info.si_uid = current_uid();

	return kill_something_info(sig, &info, pid);
}

static int
do_send_specific(pid_t tgid, pid_t pid, int sig, struct siginfo *info)
{
	struct task_struct *p;
	int error = -ESRCH;

	rcu_read_lock();
	p = find_task_by_vpid(pid);
	if (p && (tgid <= 0 || task_tgid_vnr(p) == tgid)) {
		error = check_kill_permission(sig, info, p);
		if (!error && sig) {
			error = do_send_sig_info(sig, info, p, false);
			if (unlikely(error == -ESRCH))
				error = 0;
		}
	}
	rcu_read_unlock();

	return error;
}

static int do_tkill(pid_t tgid, pid_t pid, int sig)
{
	struct siginfo info;

	info.si_signo = sig;
	info.si_errno = 0;
	info.si_code = SI_TKILL;
	info.si_pid = task_tgid_vnr(current);
	info.si_uid = current_uid();

	return do_send_specific(tgid, pid, sig, &info);
}

SYSCALL_DEFINE3(tgkill, pid_t, tgid, pid_t, pid, int, sig)
{
	
	if (pid <= 0 || tgid <= 0)
		return -EINVAL;

	return do_tkill(tgid, pid, sig);
}

SYSCALL_DEFINE2(tkill, pid_t, pid, int, sig)
{
	
	if (pid <= 0)
		return -EINVAL;

	return do_tkill(0, pid, sig);
}

SYSCALL_DEFINE3(rt_sigqueueinfo, pid_t, pid, int, sig,
		siginfo_t __user *, uinfo)
{
	siginfo_t info;

	if (copy_from_user(&info, uinfo, sizeof(siginfo_t)))
		return -EFAULT;

	if (info.si_code >= 0 || info.si_code == SI_TKILL) {
		
		WARN_ON_ONCE(info.si_code < 0);
		return -EPERM;
	}
	info.si_signo = sig;

	
	return kill_proc_info(sig, &info, pid);
}

long do_rt_tgsigqueueinfo(pid_t tgid, pid_t pid, int sig, siginfo_t *info)
{
	
	if (pid <= 0 || tgid <= 0)
		return -EINVAL;

	if (info->si_code >= 0 || info->si_code == SI_TKILL) {
		
		WARN_ON_ONCE(info->si_code < 0);
		return -EPERM;
	}
	info->si_signo = sig;

	return do_send_specific(tgid, pid, sig, info);
}

SYSCALL_DEFINE4(rt_tgsigqueueinfo, pid_t, tgid, pid_t, pid, int, sig,
		siginfo_t __user *, uinfo)
{
	siginfo_t info;

	if (copy_from_user(&info, uinfo, sizeof(siginfo_t)))
		return -EFAULT;

	return do_rt_tgsigqueueinfo(tgid, pid, sig, &info);
}

int do_sigaction(int sig, struct k_sigaction *act, struct k_sigaction *oact)
{
	struct task_struct *t = current;
	struct k_sigaction *k;
	sigset_t mask;

	if (!valid_signal(sig) || sig < 1 || (act && sig_kernel_only(sig)))
		return -EINVAL;

	k = &t->sighand->action[sig-1];

	spin_lock_irq(&current->sighand->siglock);
	if (oact)
		*oact = *k;

	if (act) {
		sigdelsetmask(&act->sa.sa_mask,
			      sigmask(SIGKILL) | sigmask(SIGSTOP));
		*k = *act;
		if (sig_handler_ignored(sig_handler(t, sig), sig)) {
			sigemptyset(&mask);
			sigaddset(&mask, sig);
			rm_from_queue_full(&mask, &t->signal->shared_pending);
			do {
				rm_from_queue_full(&mask, &t->pending);
				t = next_thread(t);
			} while (t != current);
		}
	}

	spin_unlock_irq(&current->sighand->siglock);
	return 0;
}

int 
do_sigaltstack (const stack_t __user *uss, stack_t __user *uoss, unsigned long sp)
{
	stack_t oss;
	int error;

	oss.ss_sp = (void __user *) current->sas_ss_sp;
	oss.ss_size = current->sas_ss_size;
	oss.ss_flags = sas_ss_flags(sp);

	if (uss) {
		void __user *ss_sp;
		size_t ss_size;
		int ss_flags;

		error = -EFAULT;
		if (!access_ok(VERIFY_READ, uss, sizeof(*uss)))
			goto out;
		error = __get_user(ss_sp, &uss->ss_sp) |
			__get_user(ss_flags, &uss->ss_flags) |
			__get_user(ss_size, &uss->ss_size);
		if (error)
			goto out;

		error = -EPERM;
		if (on_sig_stack(sp))
			goto out;

		error = -EINVAL;
		/*
		 * Note - this code used to test ss_flags incorrectly:
		 *  	  old code may have been written using ss_flags==0
		 *	  to mean ss_flags==SS_ONSTACK (as this was the only
		 *	  way that worked) - this fix preserves that older
		 *	  mechanism.
		 */
		if (ss_flags != SS_DISABLE && ss_flags != SS_ONSTACK && ss_flags != 0)
			goto out;

		if (ss_flags == SS_DISABLE) {
			ss_size = 0;
			ss_sp = NULL;
		} else {
			error = -ENOMEM;
			if (ss_size < MINSIGSTKSZ)
				goto out;
		}

		current->sas_ss_sp = (unsigned long) ss_sp;
		current->sas_ss_size = ss_size;
	}

	error = 0;
	if (uoss) {
		error = -EFAULT;
		if (!access_ok(VERIFY_WRITE, uoss, sizeof(*uoss)))
			goto out;
		error = __put_user(oss.ss_sp, &uoss->ss_sp) |
			__put_user(oss.ss_size, &uoss->ss_size) |
			__put_user(oss.ss_flags, &uoss->ss_flags);
	}

out:
	return error;
}

#ifdef __ARCH_WANT_SYS_SIGPENDING

SYSCALL_DEFINE1(sigpending, old_sigset_t __user *, set)
{
	return do_sigpending(set, sizeof(*set));
}

#endif

#ifdef __ARCH_WANT_SYS_SIGPROCMASK

SYSCALL_DEFINE3(sigprocmask, int, how, old_sigset_t __user *, nset,
		old_sigset_t __user *, oset)
{
	old_sigset_t old_set, new_set;
	sigset_t new_blocked;

	old_set = current->blocked.sig[0];

	if (nset) {
		if (copy_from_user(&new_set, nset, sizeof(*nset)))
			return -EFAULT;
		new_set &= ~(sigmask(SIGKILL) | sigmask(SIGSTOP));

		new_blocked = current->blocked;

		switch (how) {
		case SIG_BLOCK:
			sigaddsetmask(&new_blocked, new_set);
			break;
		case SIG_UNBLOCK:
			sigdelsetmask(&new_blocked, new_set);
			break;
		case SIG_SETMASK:
			new_blocked.sig[0] = new_set;
			break;
		default:
			return -EINVAL;
		}

		set_current_blocked(&new_blocked);
	}

	if (oset) {
		if (copy_to_user(oset, &old_set, sizeof(*oset)))
			return -EFAULT;
	}

	return 0;
}
#endif 

#ifdef __ARCH_WANT_SYS_RT_SIGACTION
SYSCALL_DEFINE4(rt_sigaction, int, sig,
		const struct sigaction __user *, act,
		struct sigaction __user *, oact,
		size_t, sigsetsize)
{
	struct k_sigaction new_sa, old_sa;
	int ret = -EINVAL;

	
	if (sigsetsize != sizeof(sigset_t))
		goto out;

	if (act) {
		if (copy_from_user(&new_sa.sa, act, sizeof(new_sa.sa)))
			return -EFAULT;
	}

	ret = do_sigaction(sig, act ? &new_sa : NULL, oact ? &old_sa : NULL);

	if (!ret && oact) {
		if (copy_to_user(oact, &old_sa.sa, sizeof(old_sa.sa)))
			return -EFAULT;
	}
out:
	return ret;
}
#endif 

#ifdef __ARCH_WANT_SYS_SGETMASK

SYSCALL_DEFINE0(sgetmask)
{
	
	return current->blocked.sig[0];
}

SYSCALL_DEFINE1(ssetmask, int, newmask)
{
	int old = current->blocked.sig[0];
	sigset_t newset;

	siginitset(&newset, newmask & ~(sigmask(SIGKILL) | sigmask(SIGSTOP)));
	set_current_blocked(&newset);

	return old;
}
#endif 

#ifdef __ARCH_WANT_SYS_SIGNAL
SYSCALL_DEFINE2(signal, int, sig, __sighandler_t, handler)
{
	struct k_sigaction new_sa, old_sa;
	int ret;

	new_sa.sa.sa_handler = handler;
	new_sa.sa.sa_flags = SA_ONESHOT | SA_NOMASK;
	sigemptyset(&new_sa.sa.sa_mask);

	ret = do_sigaction(sig, &new_sa, &old_sa);

	return ret ? ret : (unsigned long)old_sa.sa.sa_handler;
}
#endif 

#ifdef __ARCH_WANT_SYS_PAUSE

SYSCALL_DEFINE0(pause)
{
	while (!signal_pending(current)) {
		current->state = TASK_INTERRUPTIBLE;
		schedule();
	}
	return -ERESTARTNOHAND;
}

#endif

#ifdef __ARCH_WANT_SYS_RT_SIGSUSPEND
SYSCALL_DEFINE2(rt_sigsuspend, sigset_t __user *, unewset, size_t, sigsetsize)
{
	sigset_t newset;

	
	if (sigsetsize != sizeof(sigset_t))
		return -EINVAL;

	if (copy_from_user(&newset, unewset, sizeof(newset)))
		return -EFAULT;
	sigdelsetmask(&newset, sigmask(SIGKILL)|sigmask(SIGSTOP));

	current->saved_sigmask = current->blocked;
	set_current_blocked(&newset);

	current->state = TASK_INTERRUPTIBLE;
	schedule();
	set_restore_sigmask();
	return -ERESTARTNOHAND;
}
#endif 

__attribute__((weak)) const char *arch_vma_name(struct vm_area_struct *vma)
{
	return NULL;
}

void __init signals_init(void)
{
	sigqueue_cachep = KMEM_CACHE(sigqueue, SLAB_PANIC);
}

#ifdef CONFIG_KGDB_KDB
#include <linux/kdb.h>
void
kdb_send_sig_info(struct task_struct *t, struct siginfo *info)
{
	static struct task_struct *kdb_prev_t;
	int sig, new_t;
	if (!spin_trylock(&t->sighand->siglock)) {
		kdb_printf("Can't do kill command now.\n"
			   "The sigmask lock is held somewhere else in "
			   "kernel, try again later\n");
		return;
	}
	spin_unlock(&t->sighand->siglock);
	new_t = kdb_prev_t != t;
	kdb_prev_t = t;
	if (t->state != TASK_RUNNING && new_t) {
		kdb_printf("Process is not RUNNING, sending a signal from "
			   "kdb risks deadlock\n"
			   "on the run queue locks. "
			   "The signal has _not_ been sent.\n"
			   "Reissue the kill command if you want to risk "
			   "the deadlock.\n");
		return;
	}
	sig = info->si_signo;
	if (send_sig_info(sig, info, t))
		kdb_printf("Fail to deliver Signal %d to process %d.\n",
			   sig, t->pid);
	else
		kdb_printf("Signal %d is sent to process %d.\n", sig, t->pid);
}
#endif	
