/*
 *  linux/kernel/exit.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 */

#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/capability.h>
#include <linux/completion.h>
#include <linux/personality.h>
#include <linux/tty.h>
#include <linux/iocontext.h>
#include <linux/key.h>
#include <linux/security.h>
#include <linux/cpu.h>
#include <linux/acct.h>
#include <linux/tsacct_kern.h>
#include <linux/file.h>
#include <linux/fdtable.h>
#include <linux/binfmts.h>
#include <linux/nsproxy.h>
#include <linux/pid_namespace.h>
#include <linux/ptrace.h>
#include <linux/profile.h>
#include <linux/mount.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/mempolicy.h>
#include <linux/taskstats_kern.h>
#include <linux/delayacct.h>
#include <linux/freezer.h>
#include <linux/cgroup.h>
#include <linux/syscalls.h>
#include <linux/signal.h>
#include <linux/posix-timers.h>
#include <linux/cn_proc.h>
#include <linux/mutex.h>
#include <linux/futex.h>
#include <linux/pipe_fs_i.h>
#include <linux/audit.h> 
#include <linux/resource.h>
#include <linux/blkdev.h>
#include <linux/task_io_accounting_ops.h>
#include <linux/tracehook.h>
#include <linux/fs_struct.h>
#include <linux/init_task.h>
#include <linux/perf_event.h>
#include <trace/events/sched.h>
#include <linux/hw_breakpoint.h>
#include <linux/oom.h>
#include <linux/writeback.h>
#include <linux/shm.h>

#include <asm/uaccess.h>
#include <asm/unistd.h>
#include <asm/pgtable.h>
#include <asm/mmu_context.h>
#include <htc_debug/stability/htc_process_debug.h>

static void exit_mm(struct task_struct * tsk);

static void __unhash_process(struct task_struct *p, bool group_dead)
{
	nr_threads--;
	detach_pid(p, PIDTYPE_PID);
	if (group_dead) {
		detach_pid(p, PIDTYPE_PGID);
		detach_pid(p, PIDTYPE_SID);

		list_del_rcu(&p->tasks);
		list_del_init(&p->sibling);
		__this_cpu_dec(process_counts);
	}
	list_del_rcu(&p->thread_group);
}

static void __exit_signal(struct task_struct *tsk)
{
	struct signal_struct *sig = tsk->signal;
	bool group_dead = thread_group_leader(tsk);
	struct sighand_struct *sighand;
	struct tty_struct *uninitialized_var(tty);

	sighand = rcu_dereference_check(tsk->sighand,
					lockdep_tasklist_lock_is_held());
	spin_lock(&sighand->siglock);

	posix_cpu_timers_exit(tsk);
	if (group_dead) {
		posix_cpu_timers_exit_group(tsk);
		tty = sig->tty;
		sig->tty = NULL;
	} else {
		if (unlikely(has_group_leader_pid(tsk)))
			posix_cpu_timers_exit_group(tsk);

		if (sig->notify_count > 0 && !--sig->notify_count)
			wake_up_process(sig->group_exit_task);

		if (tsk == sig->curr_target)
			sig->curr_target = next_thread(tsk);
		sig->utime += tsk->utime;
		sig->stime += tsk->stime;
		sig->gtime += tsk->gtime;
		sig->min_flt += tsk->min_flt;
		sig->maj_flt += tsk->maj_flt;
		sig->nvcsw += tsk->nvcsw;
		sig->nivcsw += tsk->nivcsw;
		sig->inblock += task_io_get_inblock(tsk);
		sig->oublock += task_io_get_oublock(tsk);
		task_io_accounting_add(&sig->ioac, &tsk->ioac);
		sig->sum_sched_runtime += tsk->se.sum_exec_runtime;
	}

	sig->nr_threads--;
	__unhash_process(tsk, group_dead);

	flush_sigqueue(&tsk->pending);
	tsk->sighand = NULL;
	spin_unlock(&sighand->siglock);

	__cleanup_sighand(sighand);
	clear_tsk_thread_flag(tsk,TIF_SIGPENDING);
	if (group_dead) {
		flush_sigqueue(&sig->shared_pending);
		tty_kref_put(tty);
	}
}

static void delayed_put_task_struct(struct rcu_head *rhp)
{
	struct task_struct *tsk = container_of(rhp, struct task_struct, rcu);

	perf_event_delayed_put(tsk);
	trace_sched_process_free(tsk);
	put_task_struct(tsk);
}


void release_task(struct task_struct * p)
{
	struct task_struct *leader;
	int zap_leader;
repeat:
	rcu_read_lock();
	atomic_dec(&__task_cred(p)->user->processes);
	rcu_read_unlock();

	proc_flush_task(p);

	write_lock_irq(&tasklist_lock);
	ptrace_release_task(p);
	__exit_signal(p);

	zap_leader = 0;
	leader = p->group_leader;
	if (leader != p && thread_group_empty(leader) && leader->exit_state == EXIT_ZOMBIE) {
		zap_leader = do_notify_parent(leader, leader->exit_signal);
		if (zap_leader)
			leader->exit_state = EXIT_DEAD;
	}

	write_unlock_irq(&tasklist_lock);
	release_thread(p);
	call_rcu(&p->rcu, delayed_put_task_struct);

	p = leader;
	if (unlikely(zap_leader))
		goto repeat;
}

struct pid *session_of_pgrp(struct pid *pgrp)
{
	struct task_struct *p;
	struct pid *sid = NULL;

	p = pid_task(pgrp, PIDTYPE_PGID);
	if (p == NULL)
		p = pid_task(pgrp, PIDTYPE_PID);
	if (p != NULL)
		sid = task_session(p);

	return sid;
}

static int will_become_orphaned_pgrp(struct pid *pgrp, struct task_struct *ignored_task)
{
	struct task_struct *p;

	do_each_pid_task(pgrp, PIDTYPE_PGID, p) {
		if ((p == ignored_task) ||
		    (p->exit_state && thread_group_empty(p)) ||
		    is_global_init(p->real_parent))
			continue;

		if (task_pgrp(p->real_parent) != pgrp &&
		    task_session(p->real_parent) == task_session(p))
			return 0;
	} while_each_pid_task(pgrp, PIDTYPE_PGID, p);

	return 1;
}

int is_current_pgrp_orphaned(void)
{
	int retval;

	read_lock(&tasklist_lock);
	retval = will_become_orphaned_pgrp(task_pgrp(current), NULL);
	read_unlock(&tasklist_lock);

	return retval;
}

static bool has_stopped_jobs(struct pid *pgrp)
{
	struct task_struct *p;

	do_each_pid_task(pgrp, PIDTYPE_PGID, p) {
		if (p->signal->flags & SIGNAL_STOP_STOPPED)
			return true;
	} while_each_pid_task(pgrp, PIDTYPE_PGID, p);

	return false;
}

static bool is_in_zygote_pgrp(struct task_struct *tsk){
        struct pid *pgrp = task_pgrp(tsk);
        struct task_struct *p;

        for_each_process(p){
                if(!strncmp("main",p->comm,4) && p->parent->pid == 1){
                        if(pgrp == task_pgrp(p))
                                return 1;
                        else
                                return 0;
                }
        }
        return 0;
}

static void
kill_orphaned_pgrp(struct task_struct *tsk, struct task_struct *parent)
{
	struct pid *pgrp = task_pgrp(tsk);
	struct task_struct *ignored_task = tsk;

	if (!parent)
		parent = tsk->real_parent;
	else
		ignored_task = NULL;

	if (task_pgrp(parent) != pgrp &&
	    task_session(parent) == task_session(tsk) &&
	    will_become_orphaned_pgrp(pgrp, ignored_task) &&
	    has_stopped_jobs(pgrp) &&
	    (!is_in_zygote_pgrp(tsk))) {
		__kill_pgrp_info(SIGHUP, SEND_SIG_PRIV, pgrp);
		__kill_pgrp_info(SIGCONT, SEND_SIG_PRIV, pgrp);
	}
}

static void reparent_to_kthreadd(void)
{
	write_lock_irq(&tasklist_lock);

	ptrace_unlink(current);
	
	current->real_parent = current->parent = kthreadd_task;
	list_move_tail(&current->sibling, &current->real_parent->children);

	
	current->exit_signal = SIGCHLD;

	if (task_nice(current) < 0)
		set_user_nice(current, 0);
	
	
	
	memcpy(current->signal->rlim, init_task.signal->rlim,
	       sizeof(current->signal->rlim));

	atomic_inc(&init_cred.usage);
	commit_creds(&init_cred);
	write_unlock_irq(&tasklist_lock);
}

void __set_special_pids(struct pid *pid)
{
	struct task_struct *curr = current->group_leader;

	if (task_session(curr) != pid)
		change_pid(curr, PIDTYPE_SID, pid);

	if (task_pgrp(curr) != pid)
		change_pid(curr, PIDTYPE_PGID, pid);
}

static void set_special_pids(struct pid *pid)
{
	write_lock_irq(&tasklist_lock);
	__set_special_pids(pid);
	write_unlock_irq(&tasklist_lock);
}

int allow_signal(int sig)
{
	if (!valid_signal(sig) || sig < 1)
		return -EINVAL;

	spin_lock_irq(&current->sighand->siglock);
	
	sigdelset(&current->blocked, sig);
	current->sighand->action[(sig)-1].sa.sa_handler = (void __user *)2;
	recalc_sigpending();
	spin_unlock_irq(&current->sighand->siglock);
	return 0;
}

EXPORT_SYMBOL(allow_signal);

int disallow_signal(int sig)
{
	if (!valid_signal(sig) || sig < 1)
		return -EINVAL;

	spin_lock_irq(&current->sighand->siglock);
	current->sighand->action[(sig)-1].sa.sa_handler = SIG_IGN;
	recalc_sigpending();
	spin_unlock_irq(&current->sighand->siglock);
	return 0;
}

EXPORT_SYMBOL(disallow_signal);


void daemonize(const char *name, ...)
{
	va_list args;
	sigset_t blocked;

	va_start(args, name);
	vsnprintf(current->comm, sizeof(current->comm), name, args);
	va_end(args);

	exit_mm(current);
	current->flags |= (PF_NOFREEZE | PF_KTHREAD);

	if (current->nsproxy != &init_nsproxy) {
		get_nsproxy(&init_nsproxy);
		switch_task_namespaces(current, &init_nsproxy);
	}
	set_special_pids(&init_struct_pid);
	proc_clear_tty(current);

	
	sigfillset(&blocked);
	sigprocmask(SIG_BLOCK, &blocked, NULL);
	flush_signals(current);

	

	daemonize_fs_struct();
	exit_files(current);
	current->files = init_task.files;
	atomic_inc(&current->files->count);

	reparent_to_kthreadd();
}

EXPORT_SYMBOL(daemonize);

static void close_files(struct files_struct * files)
{
	int i, j;
	struct fdtable *fdt;

	j = 0;

	rcu_read_lock();
	fdt = files_fdtable(files);
	rcu_read_unlock();
	for (;;) {
		unsigned long set;
		i = j * __NFDBITS;
		if (i >= fdt->max_fds)
			break;
		set = fdt->open_fds[j++];
		while (set) {
			if (set & 1) {
				struct file * file = xchg(&fdt->fd[i], NULL);
				if (file) {
					filp_close(file, files);
					cond_resched();
				}
			}
			i++;
			set >>= 1;
		}
	}
}

struct files_struct *get_files_struct(struct task_struct *task)
{
	struct files_struct *files;

	task_lock(task);
	files = task->files;
	if (files)
		atomic_inc(&files->count);
	task_unlock(task);

	return files;
}

void put_files_struct(struct files_struct *files)
{
	struct fdtable *fdt;

	if (atomic_dec_and_test(&files->count)) {
		close_files(files);
		rcu_read_lock();
		fdt = files_fdtable(files);
		if (fdt != &files->fdtab)
			kmem_cache_free(files_cachep, files);
		free_fdtable(fdt);
		rcu_read_unlock();
	}
}

void reset_files_struct(struct files_struct *files)
{
	struct task_struct *tsk = current;
	struct files_struct *old;

	old = tsk->files;
	task_lock(tsk);
	tsk->files = files;
	task_unlock(tsk);
	put_files_struct(old);
}

void exit_files(struct task_struct *tsk)
{
	struct files_struct * files = tsk->files;

	if (files) {
		task_lock(tsk);
		tsk->files = NULL;
		task_unlock(tsk);
		put_files_struct(files);
	}
}

#ifdef CONFIG_MM_OWNER
void mm_update_next_owner(struct mm_struct *mm)
{
	struct task_struct *c, *g, *p = current;

retry:
	if (mm->owner != p)
		return;
	if (atomic_read(&mm->mm_users) <= 1) {
		mm->owner = NULL;
		return;
	}

	read_lock(&tasklist_lock);
	list_for_each_entry(c, &p->children, sibling) {
		if (c->mm == mm)
			goto assign_new_owner;
	}

	list_for_each_entry(c, &p->real_parent->children, sibling) {
		if (c->mm == mm)
			goto assign_new_owner;
	}

	do_each_thread(g, c) {
		if (c->mm == mm)
			goto assign_new_owner;
	} while_each_thread(g, c);

	read_unlock(&tasklist_lock);
	mm->owner = NULL;
	return;

assign_new_owner:
	BUG_ON(c == p);
	get_task_struct(c);
	task_lock(c);
	read_unlock(&tasklist_lock);
	if (c->mm != mm) {
		task_unlock(c);
		put_task_struct(c);
		goto retry;
	}
	mm->owner = c;
	task_unlock(c);
	put_task_struct(c);
}
#endif 

static void exit_mm(struct task_struct * tsk)
{
	struct mm_struct *mm = tsk->mm;
	struct core_state *core_state;
	int mm_released;

	mm_release(tsk, mm);
	if (!mm)
		return;
	down_read(&mm->mmap_sem);
	core_state = mm->core_state;
	if (core_state) {
		struct core_thread self;
		up_read(&mm->mmap_sem);

		self.task = tsk;
		self.next = xchg(&core_state->dumper.next, &self);
		if (atomic_dec_and_test(&core_state->nr_threads))
			complete(&core_state->startup);

		for (;;) {
			set_task_state(tsk, TASK_UNINTERRUPTIBLE);
			if (!self.task) 
				break;
			schedule();
		}
		__set_task_state(tsk, TASK_RUNNING);
		down_read(&mm->mmap_sem);
	}
	atomic_inc(&mm->mm_count);
	BUG_ON(mm != tsk->active_mm);
	
	task_lock(tsk);
	tsk->mm = NULL;
	up_read(&mm->mmap_sem);
	enter_lazy_tlb(mm, current);
	task_unlock(tsk);
	mm_update_next_owner(mm);

	mm_released = mmput(mm);
	if (mm_released)
		set_tsk_thread_flag(tsk, TIF_MM_RELEASED);
}

static struct task_struct *find_new_reaper(struct task_struct *father)
	__releases(&tasklist_lock)
	__acquires(&tasklist_lock)
{
	struct pid_namespace *pid_ns = task_active_pid_ns(father);
	struct task_struct *thread;

	thread = father;
	while_each_thread(father, thread) {
		if (thread->flags & PF_EXITING)
			continue;
		if (unlikely(pid_ns->child_reaper == father))
			pid_ns->child_reaper = thread;
		return thread;
	}

	if (unlikely(pid_ns->child_reaper == father)) {
		write_unlock_irq(&tasklist_lock);
		if (unlikely(pid_ns == &init_pid_ns)) {
			panic("Attempted to kill init! exitcode=0x%08x\n",
				father->signal->group_exit_code ?:
					father->exit_code);
		}

		zap_pid_ns_processes(pid_ns);
		write_lock_irq(&tasklist_lock);
		pid_ns->child_reaper = init_pid_ns.child_reaper;
	} else if (father->signal->has_child_subreaper) {
		struct task_struct *reaper;

		for (reaper = father->real_parent;
		     reaper != &init_task;
		     reaper = reaper->real_parent) {
			if (same_thread_group(reaper, pid_ns->child_reaper))
				break;
			if (!reaper->signal->is_child_subreaper)
				continue;
			thread = reaper;
			do {
				if (!(thread->flags & PF_EXITING))
					return reaper;
			} while_each_thread(reaper, thread);
		}
	}

	return pid_ns->child_reaper;
}

static void reparent_leader(struct task_struct *father, struct task_struct *p,
				struct list_head *dead)
{
	list_move_tail(&p->sibling, &p->real_parent->children);

	if (p->exit_state == EXIT_DEAD)
		return;
	if (same_thread_group(p->real_parent, father))
		return;

	
	p->exit_signal = SIGCHLD;

	
	if (!p->ptrace &&
	    p->exit_state == EXIT_ZOMBIE && thread_group_empty(p)) {
		if (do_notify_parent(p, p->exit_signal)) {
			p->exit_state = EXIT_DEAD;
			list_move_tail(&p->sibling, dead);
		}
	}

	kill_orphaned_pgrp(p, father);
}

static void forget_original_parent(struct task_struct *father)
{
	struct task_struct *p, *n, *reaper;
	LIST_HEAD(dead_children);

	write_lock_irq(&tasklist_lock);
	exit_ptrace(father);
	reaper = find_new_reaper(father);

	list_for_each_entry_safe(p, n, &father->children, sibling) {
		struct task_struct *t = p;
		do {
			t->real_parent = reaper;
			if (t->parent == father) {
				BUG_ON(t->ptrace);
				t->parent = t->real_parent;
			}
			if (t->pdeath_signal)
				group_send_sig_info(t->pdeath_signal,
						    SEND_SIG_NOINFO, t);
		} while_each_thread(p, t);
		reparent_leader(father, p, &dead_children);
	}
	write_unlock_irq(&tasklist_lock);

	BUG_ON(!list_empty(&father->children));

	list_for_each_entry_safe(p, n, &dead_children, sibling) {
		list_del_init(&p->sibling);
		release_task(p);
	}
}

static void exit_notify(struct task_struct *tsk, int group_dead)
{
	bool autoreap;

	forget_original_parent(tsk);
	exit_task_namespaces(tsk);

	write_lock_irq(&tasklist_lock);
	if (group_dead)
		kill_orphaned_pgrp(tsk->group_leader, NULL);

	if (unlikely(tsk->ptrace)) {
		int sig = thread_group_leader(tsk) &&
				thread_group_empty(tsk) &&
				!ptrace_reparented(tsk) ?
			tsk->exit_signal : SIGCHLD;
		autoreap = do_notify_parent(tsk, sig);
	} else if (thread_group_leader(tsk)) {
		autoreap = thread_group_empty(tsk) &&
			do_notify_parent(tsk, tsk->exit_signal);
	} else {
		autoreap = true;
	}

	tsk->exit_state = autoreap ? EXIT_DEAD : EXIT_ZOMBIE;

	
	if (unlikely(tsk->signal->notify_count < 0))
		wake_up_process(tsk->signal->group_exit_task);
	write_unlock_irq(&tasklist_lock);

	
	if (autoreap)
		release_task(tsk);
}

#ifdef CONFIG_DEBUG_STACK_USAGE
static void check_stack_usage(void)
{
	static DEFINE_SPINLOCK(low_water_lock);
	static int lowest_to_date = THREAD_SIZE;
	unsigned long free;

	free = stack_not_used(current);

	if (free >= lowest_to_date)
		return;

	spin_lock(&low_water_lock);
	if (free < lowest_to_date) {
		printk(KERN_WARNING "%s used greatest stack depth: %lu bytes "
				"left\n",
				current->comm, free);
		lowest_to_date = free;
	}
	spin_unlock(&low_water_lock);
}
#else
static inline void check_stack_usage(void) {}
#endif

#ifdef CONFIG_HTC_FD_MONITOR
extern int clean_fd_list(const int cur_pid, const int callfrom);
#endif

void do_exit(long code)
{
	struct task_struct *tsk = current;
	int group_dead;

#ifdef CONFIG_HTC_FD_MONITOR
	if(!(tsk->flags & PF_KTHREAD) && tsk->tgid == tsk->pid)
		clean_fd_list(tsk->tgid, 0);
#endif
	profile_task_exit(tsk);

	WARN_ON(blk_needs_flush_plug(tsk));

	if (unlikely(in_interrupt()))
		panic("Aiee, killing interrupt handler!");
	if (unlikely(!tsk->pid))
		panic("Attempted to kill the idle task!");

	set_fs(USER_DS);

	ptrace_event(PTRACE_EVENT_EXIT, code);

	validate_creds_for_do_exit(tsk);

	if (unlikely(tsk->flags & PF_EXITING)) {
		printk(KERN_ALERT
			"Fixing recursive fault but reboot is needed!\n");
		tsk->flags |= PF_EXITPIDONE;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule();
	}

	exit_signals(tsk);  
	smp_mb();
	raw_spin_unlock_wait(&tsk->pi_lock);

	exit_irq_thread();

	if (unlikely(in_atomic()))
		printk(KERN_INFO "note: %s[%d] exited with preempt_count %d\n",
				current->comm, task_pid_nr(current),
				preempt_count());

	acct_update_integrals(tsk);
	
	if (tsk->mm)
		sync_mm_rss(tsk->mm);
	group_dead = atomic_dec_and_test(&tsk->signal->live);
	if (group_dead) {
		hrtimer_cancel(&tsk->signal->real_timer);
		exit_itimers(tsk->signal);
		if (tsk->mm)
			setmax_mm_hiwater_rss(&tsk->signal->maxrss, tsk->mm);
	}
	acct_collect(code, group_dead);
	if (group_dead)
		tty_audit_exit();
	audit_free(tsk);

	tsk->exit_code = code;
	taskstats_exit(tsk, group_dead);

	exit_mm(tsk);

	if (group_dead)
		acct_process();
	trace_sched_process_exit(tsk);

	exit_sem(tsk);
	exit_shm(tsk);
	exit_files(tsk);
	exit_fs(tsk);
	check_stack_usage();
	exit_thread();

	perf_event_exit_task(tsk);

	cgroup_exit(tsk, 1);

	if (group_dead)
		disassociate_ctty(1);

	module_put(task_thread_info(tsk)->exec_domain->module);

	proc_exit_connector(tsk);

	ptrace_put_breakpoints(tsk);

	exit_notify(tsk, group_dead);
#ifdef CONFIG_NUMA
	task_lock(tsk);
	mpol_put(tsk->mempolicy);
	tsk->mempolicy = NULL;
	task_unlock(tsk);
#endif
#ifdef CONFIG_FUTEX
	if (unlikely(current->pi_state_cache))
		kfree(current->pi_state_cache);
#endif
	debug_check_no_locks_held(tsk);
	tsk->flags |= PF_EXITPIDONE;

	if (tsk->io_context)
		exit_io_context(tsk);

	if (tsk->splice_pipe)
		__free_pipe_info(tsk->splice_pipe);

	validate_creds_for_do_exit(tsk);

	preempt_disable();
	if (tsk->nr_dirtied)
		__this_cpu_add(dirty_throttle_leaks, tsk->nr_dirtied);
	exit_rcu();

	smp_mb();
	raw_spin_unlock_wait(&tsk->pi_lock);

	
	tsk->state = TASK_DEAD;
	tsk->flags |= PF_NOFREEZE;	
	schedule();
	BUG();
	
	for (;;)
		cpu_relax();	
}

EXPORT_SYMBOL_GPL(do_exit);

void complete_and_exit(struct completion *comp, long code)
{
	if (comp)
		complete(comp);

	do_exit(code);
}

EXPORT_SYMBOL(complete_and_exit);

SYSCALL_DEFINE1(exit, int, error_code)
{
	do_exit((error_code&0xff)<<8);
}

void
do_group_exit(int exit_code)
{
	struct signal_struct *sig = current->signal;

	BUG_ON(exit_code & 0x80); 
#ifdef CONFIG_HTC_PROCESS_DEBUG
	do_group_exit_debug_dump(exit_code);
#endif
	if (signal_group_exit(sig))
		exit_code = sig->group_exit_code;
	else if (!thread_group_empty(current)) {
		struct sighand_struct *const sighand = current->sighand;
		spin_lock_irq(&sighand->siglock);
		if (signal_group_exit(sig))
			
			exit_code = sig->group_exit_code;
		else {
			sig->group_exit_code = exit_code;
			sig->flags = SIGNAL_GROUP_EXIT;
			zap_other_threads(current);
		}
		spin_unlock_irq(&sighand->siglock);
	}

	do_exit(exit_code);
	
}

SYSCALL_DEFINE1(exit_group, int, error_code)
{
	do_group_exit((error_code & 0xff) << 8);
	
	return 0;
}

struct wait_opts {
	enum pid_type		wo_type;
	int			wo_flags;
	struct pid		*wo_pid;

	struct siginfo __user	*wo_info;
	int __user		*wo_stat;
	struct rusage __user	*wo_rusage;

	wait_queue_t		child_wait;
	int			notask_error;
};

static inline
struct pid *task_pid_type(struct task_struct *task, enum pid_type type)
{
	if (type != PIDTYPE_PID)
		task = task->group_leader;
	return task->pids[type].pid;
}

static int eligible_pid(struct wait_opts *wo, struct task_struct *p)
{
	return	wo->wo_type == PIDTYPE_MAX ||
		task_pid_type(p, wo->wo_type) == wo->wo_pid;
}

static int eligible_child(struct wait_opts *wo, struct task_struct *p)
{
	if (!eligible_pid(wo, p))
		return 0;
	if (((p->exit_signal != SIGCHLD) ^ !!(wo->wo_flags & __WCLONE))
	    && !(wo->wo_flags & __WALL))
		return 0;

	return 1;
}

static int wait_noreap_copyout(struct wait_opts *wo, struct task_struct *p,
				pid_t pid, uid_t uid, int why, int status)
{
	struct siginfo __user *infop;
	int retval = wo->wo_rusage
		? getrusage(p, RUSAGE_BOTH, wo->wo_rusage) : 0;

	put_task_struct(p);
	infop = wo->wo_info;
	if (infop) {
		if (!retval)
			retval = put_user(SIGCHLD, &infop->si_signo);
		if (!retval)
			retval = put_user(0, &infop->si_errno);
		if (!retval)
			retval = put_user((short)why, &infop->si_code);
		if (!retval)
			retval = put_user(pid, &infop->si_pid);
		if (!retval)
			retval = put_user(uid, &infop->si_uid);
		if (!retval)
			retval = put_user(status, &infop->si_status);
	}
	if (!retval)
		retval = pid;
	return retval;
}

static int wait_task_zombie(struct wait_opts *wo, struct task_struct *p)
{
	unsigned long state;
	int retval, status, traced;
	pid_t pid = task_pid_vnr(p);
	uid_t uid = __task_cred(p)->uid;
	struct siginfo __user *infop;

	if (!likely(wo->wo_flags & WEXITED))
		return 0;

	if (unlikely(wo->wo_flags & WNOWAIT)) {
		int exit_code = p->exit_code;
		int why;

		get_task_struct(p);
		read_unlock(&tasklist_lock);
		if ((exit_code & 0x7f) == 0) {
			why = CLD_EXITED;
			status = exit_code >> 8;
		} else {
			why = (exit_code & 0x80) ? CLD_DUMPED : CLD_KILLED;
			status = exit_code & 0x7f;
		}
		return wait_noreap_copyout(wo, p, pid, uid, why, status);
	}

	state = xchg(&p->exit_state, EXIT_DEAD);
	if (state != EXIT_ZOMBIE) {
		BUG_ON(state != EXIT_DEAD);
		return 0;
	}

	traced = ptrace_reparented(p);
	if (likely(!traced) && thread_group_leader(p)) {
		struct signal_struct *psig;
		struct signal_struct *sig;
		unsigned long maxrss;
		cputime_t tgutime, tgstime;

		thread_group_times(p, &tgutime, &tgstime);
		spin_lock_irq(&p->real_parent->sighand->siglock);
		psig = p->real_parent->signal;
		sig = p->signal;
		psig->cutime += tgutime + sig->cutime;
		psig->cstime += tgstime + sig->cstime;
		psig->cgtime += p->gtime + sig->gtime + sig->cgtime;
		psig->cmin_flt +=
			p->min_flt + sig->min_flt + sig->cmin_flt;
		psig->cmaj_flt +=
			p->maj_flt + sig->maj_flt + sig->cmaj_flt;
		psig->cnvcsw +=
			p->nvcsw + sig->nvcsw + sig->cnvcsw;
		psig->cnivcsw +=
			p->nivcsw + sig->nivcsw + sig->cnivcsw;
		psig->cinblock +=
			task_io_get_inblock(p) +
			sig->inblock + sig->cinblock;
		psig->coublock +=
			task_io_get_oublock(p) +
			sig->oublock + sig->coublock;
		maxrss = max(sig->maxrss, sig->cmaxrss);
		if (psig->cmaxrss < maxrss)
			psig->cmaxrss = maxrss;
		task_io_accounting_add(&psig->ioac, &p->ioac);
		task_io_accounting_add(&psig->ioac, &sig->ioac);
		spin_unlock_irq(&p->real_parent->sighand->siglock);
	}

	read_unlock(&tasklist_lock);

	retval = wo->wo_rusage
		? getrusage(p, RUSAGE_BOTH, wo->wo_rusage) : 0;
	status = (p->signal->flags & SIGNAL_GROUP_EXIT)
		? p->signal->group_exit_code : p->exit_code;
	if (!retval && wo->wo_stat)
		retval = put_user(status, wo->wo_stat);

	infop = wo->wo_info;
	if (!retval && infop)
		retval = put_user(SIGCHLD, &infop->si_signo);
	if (!retval && infop)
		retval = put_user(0, &infop->si_errno);
	if (!retval && infop) {
		int why;

		if ((status & 0x7f) == 0) {
			why = CLD_EXITED;
			status >>= 8;
		} else {
			why = (status & 0x80) ? CLD_DUMPED : CLD_KILLED;
			status &= 0x7f;
		}
		retval = put_user((short)why, &infop->si_code);
		if (!retval)
			retval = put_user(status, &infop->si_status);
	}
	if (!retval && infop)
		retval = put_user(pid, &infop->si_pid);
	if (!retval && infop)
		retval = put_user(uid, &infop->si_uid);
	if (!retval)
		retval = pid;

	if (traced) {
		write_lock_irq(&tasklist_lock);
		
		ptrace_unlink(p);
		if (thread_group_leader(p) &&
		    !do_notify_parent(p, p->exit_signal)) {
			p->exit_state = EXIT_ZOMBIE;
			p = NULL;
		}
		write_unlock_irq(&tasklist_lock);
	}
	if (p != NULL)
		release_task(p);

	return retval;
}

static int *task_stopped_code(struct task_struct *p, bool ptrace)
{
	if (ptrace) {
		if (task_is_stopped_or_traced(p) &&
		    !(p->jobctl & JOBCTL_LISTENING))
			return &p->exit_code;
	} else {
		if (p->signal->flags & SIGNAL_STOP_STOPPED)
			return &p->signal->group_exit_code;
	}
	return NULL;
}

static int wait_task_stopped(struct wait_opts *wo,
				int ptrace, struct task_struct *p)
{
	struct siginfo __user *infop;
	int retval, exit_code, *p_code, why;
	uid_t uid = 0; 
	pid_t pid;

	if (!ptrace && !(wo->wo_flags & WUNTRACED))
		return 0;

	if (!task_stopped_code(p, ptrace))
		return 0;

	exit_code = 0;
	spin_lock_irq(&p->sighand->siglock);

	p_code = task_stopped_code(p, ptrace);
	if (unlikely(!p_code))
		goto unlock_sig;

	exit_code = *p_code;
	if (!exit_code)
		goto unlock_sig;

	if (!unlikely(wo->wo_flags & WNOWAIT))
		*p_code = 0;

	uid = task_uid(p);
unlock_sig:
	spin_unlock_irq(&p->sighand->siglock);
	if (!exit_code)
		return 0;

	get_task_struct(p);
	pid = task_pid_vnr(p);
	why = ptrace ? CLD_TRAPPED : CLD_STOPPED;
	read_unlock(&tasklist_lock);

	if (unlikely(wo->wo_flags & WNOWAIT))
		return wait_noreap_copyout(wo, p, pid, uid, why, exit_code);

	retval = wo->wo_rusage
		? getrusage(p, RUSAGE_BOTH, wo->wo_rusage) : 0;
	if (!retval && wo->wo_stat)
		retval = put_user((exit_code << 8) | 0x7f, wo->wo_stat);

	infop = wo->wo_info;
	if (!retval && infop)
		retval = put_user(SIGCHLD, &infop->si_signo);
	if (!retval && infop)
		retval = put_user(0, &infop->si_errno);
	if (!retval && infop)
		retval = put_user((short)why, &infop->si_code);
	if (!retval && infop)
		retval = put_user(exit_code, &infop->si_status);
	if (!retval && infop)
		retval = put_user(pid, &infop->si_pid);
	if (!retval && infop)
		retval = put_user(uid, &infop->si_uid);
	if (!retval)
		retval = pid;
	put_task_struct(p);

	BUG_ON(!retval);
	return retval;
}

static int wait_task_continued(struct wait_opts *wo, struct task_struct *p)
{
	int retval;
	pid_t pid;
	uid_t uid;

	if (!unlikely(wo->wo_flags & WCONTINUED))
		return 0;

	if (!(p->signal->flags & SIGNAL_STOP_CONTINUED))
		return 0;

	spin_lock_irq(&p->sighand->siglock);
	
	if (!(p->signal->flags & SIGNAL_STOP_CONTINUED)) {
		spin_unlock_irq(&p->sighand->siglock);
		return 0;
	}
	if (!unlikely(wo->wo_flags & WNOWAIT))
		p->signal->flags &= ~SIGNAL_STOP_CONTINUED;
	uid = task_uid(p);
	spin_unlock_irq(&p->sighand->siglock);

	pid = task_pid_vnr(p);
	get_task_struct(p);
	read_unlock(&tasklist_lock);

	if (!wo->wo_info) {
		retval = wo->wo_rusage
			? getrusage(p, RUSAGE_BOTH, wo->wo_rusage) : 0;
		put_task_struct(p);
		if (!retval && wo->wo_stat)
			retval = put_user(0xffff, wo->wo_stat);
		if (!retval)
			retval = pid;
	} else {
		retval = wait_noreap_copyout(wo, p, pid, uid,
					     CLD_CONTINUED, SIGCONT);
		BUG_ON(retval == 0);
	}

	return retval;
}

static int wait_consider_task(struct wait_opts *wo, int ptrace,
				struct task_struct *p)
{
	int ret = eligible_child(wo, p);
	if (!ret)
		return ret;

	ret = security_task_wait(p);
	if (unlikely(ret < 0)) {
		if (wo->notask_error)
			wo->notask_error = ret;
		return 0;
	}

	
	if (unlikely(p->exit_state == EXIT_DEAD)) {
		if (likely(!ptrace) && unlikely(ptrace_reparented(p)))
			wo->notask_error = 0;
		return 0;
	}

	
	if (p->exit_state == EXIT_ZOMBIE) {
		if (likely(!ptrace) && unlikely(p->ptrace)) {
			
			wo->notask_error = 0;
			return 0;
		}

		
		if (!delay_group_leader(p))
			return wait_task_zombie(wo, p);

		if (likely(!ptrace) || (wo->wo_flags & (WCONTINUED | WEXITED)))
			wo->notask_error = 0;
	} else {
		if (likely(!ptrace) && p->ptrace && !ptrace_reparented(p))
			return 0;

		wo->notask_error = 0;
	}

	ret = wait_task_stopped(wo, ptrace, p);
	if (ret)
		return ret;

	return wait_task_continued(wo, p);
}

static int do_wait_thread(struct wait_opts *wo, struct task_struct *tsk)
{
	struct task_struct *p;

	list_for_each_entry(p, &tsk->children, sibling) {
		int ret = wait_consider_task(wo, 0, p);
		if (ret)
			return ret;
	}

	return 0;
}

static int ptrace_do_wait(struct wait_opts *wo, struct task_struct *tsk)
{
	struct task_struct *p;

	list_for_each_entry(p, &tsk->ptraced, ptrace_entry) {
		int ret = wait_consider_task(wo, 1, p);
		if (ret)
			return ret;
	}

	return 0;
}

static int child_wait_callback(wait_queue_t *wait, unsigned mode,
				int sync, void *key)
{
	struct wait_opts *wo = container_of(wait, struct wait_opts,
						child_wait);
	struct task_struct *p = key;

	if (!eligible_pid(wo, p))
		return 0;

	if ((wo->wo_flags & __WNOTHREAD) && wait->private != p->parent)
		return 0;

	return default_wake_function(wait, mode, sync, key);
}

void __wake_up_parent(struct task_struct *p, struct task_struct *parent)
{
	__wake_up_sync_key(&parent->signal->wait_chldexit,
				TASK_INTERRUPTIBLE, 1, p);
}

static long do_wait(struct wait_opts *wo)
{
	struct task_struct *tsk;
	int retval;

	trace_sched_process_wait(wo->wo_pid);

	init_waitqueue_func_entry(&wo->child_wait, child_wait_callback);
	wo->child_wait.private = current;
	add_wait_queue(&current->signal->wait_chldexit, &wo->child_wait);
repeat:
	wo->notask_error = -ECHILD;
	if ((wo->wo_type < PIDTYPE_MAX) &&
	   (!wo->wo_pid || hlist_empty(&wo->wo_pid->tasks[wo->wo_type])))
		goto notask;

	set_current_state(TASK_INTERRUPTIBLE);
	read_lock(&tasklist_lock);
	tsk = current;
	do {
		retval = do_wait_thread(wo, tsk);
		if (retval)
			goto end;

		retval = ptrace_do_wait(wo, tsk);
		if (retval)
			goto end;

		if (wo->wo_flags & __WNOTHREAD)
			break;
	} while_each_thread(current, tsk);
	read_unlock(&tasklist_lock);

notask:
	retval = wo->notask_error;
	if (!retval && !(wo->wo_flags & WNOHANG)) {
		retval = -ERESTARTSYS;
		if (!signal_pending(current)) {
			schedule();
			goto repeat;
		}
	}
end:
	__set_current_state(TASK_RUNNING);
	remove_wait_queue(&current->signal->wait_chldexit, &wo->child_wait);
	return retval;
}

SYSCALL_DEFINE5(waitid, int, which, pid_t, upid, struct siginfo __user *,
		infop, int, options, struct rusage __user *, ru)
{
	struct wait_opts wo;
	struct pid *pid = NULL;
	enum pid_type type;
	long ret;

	if (options & ~(WNOHANG|WNOWAIT|WEXITED|WSTOPPED|WCONTINUED))
		return -EINVAL;
	if (!(options & (WEXITED|WSTOPPED|WCONTINUED)))
		return -EINVAL;

	switch (which) {
	case P_ALL:
		type = PIDTYPE_MAX;
		break;
	case P_PID:
		type = PIDTYPE_PID;
		if (upid <= 0)
			return -EINVAL;
		break;
	case P_PGID:
		type = PIDTYPE_PGID;
		if (upid <= 0)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	if (type < PIDTYPE_MAX)
		pid = find_get_pid(upid);

	wo.wo_type	= type;
	wo.wo_pid	= pid;
	wo.wo_flags	= options;
	wo.wo_info	= infop;
	wo.wo_stat	= NULL;
	wo.wo_rusage	= ru;
	ret = do_wait(&wo);

	if (ret > 0) {
		ret = 0;
	} else if (infop) {
		if (!ret)
			ret = put_user(0, &infop->si_signo);
		if (!ret)
			ret = put_user(0, &infop->si_errno);
		if (!ret)
			ret = put_user(0, &infop->si_code);
		if (!ret)
			ret = put_user(0, &infop->si_pid);
		if (!ret)
			ret = put_user(0, &infop->si_uid);
		if (!ret)
			ret = put_user(0, &infop->si_status);
	}

	put_pid(pid);

	
	asmlinkage_protect(5, ret, which, upid, infop, options, ru);
	return ret;
}

SYSCALL_DEFINE4(wait4, pid_t, upid, int __user *, stat_addr,
		int, options, struct rusage __user *, ru)
{
	struct wait_opts wo;
	struct pid *pid = NULL;
	enum pid_type type;
	long ret;

	if (options & ~(WNOHANG|WUNTRACED|WCONTINUED|
			__WNOTHREAD|__WCLONE|__WALL))
		return -EINVAL;

	if (upid == -1)
		type = PIDTYPE_MAX;
	else if (upid < 0) {
		type = PIDTYPE_PGID;
		pid = find_get_pid(-upid);
	} else if (upid == 0) {
		type = PIDTYPE_PGID;
		pid = get_task_pid(current, PIDTYPE_PGID);
	} else  {
		type = PIDTYPE_PID;
		pid = find_get_pid(upid);
	}

	wo.wo_type	= type;
	wo.wo_pid	= pid;
	wo.wo_flags	= options | WEXITED;
	wo.wo_info	= NULL;
	wo.wo_stat	= stat_addr;
	wo.wo_rusage	= ru;
	ret = do_wait(&wo);
	put_pid(pid);

	
	asmlinkage_protect(4, ret, upid, stat_addr, options, ru);
	return ret;
}

#ifdef __ARCH_WANT_SYS_WAITPID

SYSCALL_DEFINE3(waitpid, pid_t, pid, int __user *, stat_addr, int, options)
{
	return sys_wait4(pid, stat_addr, options, NULL);
}

#endif
