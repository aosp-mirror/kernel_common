/*
 *  linux/fs/proc/base.c
 *
 *  Copyright (C) 1991, 1992 Linus Torvalds
 *
 *  proc base directory handling functions
 *
 *  1999, Al Viro. Rewritten. Now it covers the whole per-process part.
 *  Instead of using magical inumbers to determine the kind of object
 *  we allocate and fill in-core inodes upon lookup. They don't even
 *  go into icache. We cache the reference to task_struct upon lookup too.
 *  Eventually it should become a filesystem in its own. We don't use the
 *  rest of procfs anymore.
 *
 *
 *  Changelog:
 *  17-Jan-2005
 *  Allan Bezerra
 *  Bruna Moreira <bruna.moreira@indt.org.br>
 *  Edjard Mota <edjard.mota@indt.org.br>
 *  Ilias Biris <ilias.biris@indt.org.br>
 *  Mauricio Lin <mauricio.lin@indt.org.br>
 *
 *  Embedded Linux Lab - 10LE Instituto Nokia de Tecnologia - INdT
 *
 *  A new process specific entry (smaps) included in /proc. It shows the
 *  size of rss for each memory area. The maps entry lacks information
 *  about physical memory size (rss) for each mapped file, i.e.,
 *  rss information for executables and library files.
 *  This additional information is useful for any tools that need to know
 *  about physical memory consumption for a process specific library.
 *
 *  Changelog:
 *  21-Feb-2005
 *  Embedded Linux Lab - 10LE Instituto Nokia de Tecnologia - INdT
 *  Pud inclusion in the page table walking.
 *
 *  ChangeLog:
 *  10-Mar-2005
 *  10LE Instituto Nokia de Tecnologia - INdT:
 *  A better way to walks through the page table as suggested by Hugh Dickins.
 *
 *  Simo Piiroinen <simo.piiroinen@nokia.com>:
 *  Smaps information related to shared, private, clean and dirty pages.
 *
 *  Paul Mundt <paul.mundt@nokia.com>:
 *  Overall revision about smaps.
 */

#include <asm/uaccess.h>

#include <linux/errno.h>
#include <linux/time.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <linux/task_io_accounting_ops.h>
#include <linux/init.h>
#include <linux/capability.h>
#include <linux/file.h>
#include <linux/fdtable.h>
#include <linux/string.h>
#include <linux/seq_file.h>
#include <linux/namei.h>
#include <linux/mnt_namespace.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/rcupdate.h>
#include <linux/kallsyms.h>
#include <linux/stacktrace.h>
#include <linux/resource.h>
#include <linux/module.h>
#include <linux/mount.h>
#include <linux/security.h>
#include <linux/ptrace.h>
#include <linux/tracehook.h>
#include <linux/cgroup.h>
#include <linux/cpuset.h>
#include <linux/audit.h>
#include <linux/poll.h>
#include <linux/nsproxy.h>
#include <linux/oom.h>
#include <linux/elf.h>
#include <linux/pid_namespace.h>
#include <linux/fs_struct.h>
#include <linux/slab.h>
#include <linux/flex_array.h>
#ifdef CONFIG_HARDWALL
#include <asm/hardwall.h>
#endif
#include <trace/events/oom.h>
#include "internal.h"


struct pid_entry {
	char *name;
	int len;
	umode_t mode;
	const struct inode_operations *iop;
	const struct file_operations *fop;
	union proc_op op;
};

#define NOD(NAME, MODE, IOP, FOP, OP) {			\
	.name = (NAME),					\
	.len  = sizeof(NAME) - 1,			\
	.mode = MODE,					\
	.iop  = IOP,					\
	.fop  = FOP,					\
	.op   = OP,					\
}

#define DIR(NAME, MODE, iops, fops)	\
	NOD(NAME, (S_IFDIR|(MODE)), &iops, &fops, {} )
#define LNK(NAME, get_link)					\
	NOD(NAME, (S_IFLNK|S_IRWXUGO),				\
		&proc_pid_link_inode_operations, NULL,		\
		{ .proc_get_link = get_link } )
#define REG(NAME, MODE, fops)				\
	NOD(NAME, (S_IFREG|(MODE)), NULL, &fops, {})
#define INF(NAME, MODE, read)				\
	NOD(NAME, (S_IFREG|(MODE)), 			\
		NULL, &proc_info_file_operations,	\
		{ .proc_read = read } )
#define ONE(NAME, MODE, show)				\
	NOD(NAME, (S_IFREG|(MODE)), 			\
		NULL, &proc_single_file_operations,	\
		{ .proc_show = show } )

static int proc_fd_permission(struct inode *inode, int mask);

#define ANDROID(NAME, MODE, OTYPE)			\
	NOD(NAME, (S_IFREG|(MODE)),			\
		&proc_##OTYPE##_inode_operations,	\
		&proc_##OTYPE##_operations, {})

static unsigned int pid_entry_count_dirs(const struct pid_entry *entries,
	unsigned int n)
{
	unsigned int i;
	unsigned int count;

	count = 0;
	for (i = 0; i < n; ++i) {
		if (S_ISDIR(entries[i].mode))
			++count;
	}

	return count;
}

static int get_task_root(struct task_struct *task, struct path *root)
{
	int result = -ENOENT;

	task_lock(task);
	if (task->fs) {
		get_fs_root(task->fs, root);
		result = 0;
	}
	task_unlock(task);
	return result;
}

static int proc_cwd_link(struct dentry *dentry, struct path *path)
{
	struct task_struct *task = get_proc_task(dentry->d_inode);
	int result = -ENOENT;

	if (task) {
		task_lock(task);
		if (task->fs) {
			get_fs_pwd(task->fs, path);
			result = 0;
		}
		task_unlock(task);
		put_task_struct(task);
	}
	return result;
}

static int proc_root_link(struct dentry *dentry, struct path *path)
{
	struct task_struct *task = get_proc_task(dentry->d_inode);
	int result = -ENOENT;

	if (task) {
		result = get_task_root(task, path);
		put_task_struct(task);
	}
	return result;
}

struct mm_struct *mm_for_maps(struct task_struct *task)
{
	return mm_access(task, PTRACE_MODE_READ);
}

static int proc_pid_cmdline(struct task_struct *task, char * buffer)
{
	int res = 0;
	unsigned int len;
	struct mm_struct *mm = get_task_mm(task);
	if (!mm)
		goto out;
	if (!mm->arg_end)
		goto out_mm;	

 	len = mm->arg_end - mm->arg_start;
 
	if (len > PAGE_SIZE)
		len = PAGE_SIZE;
 
	res = access_process_vm(task, mm->arg_start, buffer, len, 0);

	// If the nul at the end of args has been overwritten, then
	
	if (res > 0 && buffer[res-1] != '\0' && len < PAGE_SIZE) {
		len = strnlen(buffer, res);
		if (len < res) {
		    res = len;
		} else {
			len = mm->env_end - mm->env_start;
			if (len > PAGE_SIZE - res)
				len = PAGE_SIZE - res;
			res += access_process_vm(task, mm->env_start, buffer+res, len, 0);
			res = strnlen(buffer, res);
		}
	}
out_mm:
	mmput(mm);
out:
	return res;
}

static int proc_pid_auxv(struct task_struct *task, char *buffer)
{
	struct mm_struct *mm = mm_for_maps(task);
	int res = PTR_ERR(mm);
	if (mm && !IS_ERR(mm)) {
		unsigned int nwords = 0;
		do {
			nwords += 2;
		} while (mm->saved_auxv[nwords - 2] != 0); 
		res = nwords * sizeof(mm->saved_auxv[0]);
		if (res > PAGE_SIZE)
			res = PAGE_SIZE;
		memcpy(buffer, mm->saved_auxv, res);
		mmput(mm);
	}
	return res;
}


#ifdef CONFIG_KALLSYMS
static int proc_pid_wchan(struct task_struct *task, char *buffer)
{
	unsigned long wchan;
	char symname[KSYM_NAME_LEN];

	wchan = get_wchan(task);

	if (lookup_symbol_name(wchan, symname) < 0)
		if (!ptrace_may_access(task, PTRACE_MODE_READ))
			return 0;
		else
			return sprintf(buffer, "%lu", wchan);
	else
		return sprintf(buffer, "%s", symname);
}
#endif 

static int lock_trace(struct task_struct *task)
{
	int err = mutex_lock_killable(&task->signal->cred_guard_mutex);
	if (err)
		return err;
	if (!ptrace_may_access(task, PTRACE_MODE_ATTACH)) {
		mutex_unlock(&task->signal->cred_guard_mutex);
		return -EPERM;
	}
	return 0;
}

static void unlock_trace(struct task_struct *task)
{
	mutex_unlock(&task->signal->cred_guard_mutex);
}

#ifdef CONFIG_STACKTRACE

#define MAX_STACK_TRACE_DEPTH	64

static int proc_pid_stack(struct seq_file *m, struct pid_namespace *ns,
			  struct pid *pid, struct task_struct *task)
{
	struct stack_trace trace;
	unsigned long *entries;
	int err;
	int i;

	entries = kmalloc(MAX_STACK_TRACE_DEPTH * sizeof(*entries), GFP_KERNEL);
	if (!entries)
		return -ENOMEM;

	trace.nr_entries	= 0;
	trace.max_entries	= MAX_STACK_TRACE_DEPTH;
	trace.entries		= entries;
	trace.skip		= 0;

	err = lock_trace(task);
	if (!err) {
		save_stack_trace_tsk(task, &trace);

		for (i = 0; i < trace.nr_entries; i++) {
			seq_printf(m, "[<%pK>] %pS\n",
				   (void *)entries[i], (void *)entries[i]);
		}
		unlock_trace(task);
	}
	kfree(entries);

	return err;
}
#endif

#ifdef CONFIG_SCHEDSTATS
static int proc_pid_schedstat(struct task_struct *task, char *buffer)
{
	return sprintf(buffer, "%llu %llu %lu\n",
			(unsigned long long)task->se.sum_exec_runtime,
			(unsigned long long)task->sched_info.run_delay,
			task->sched_info.pcount);
}
#endif

#ifdef CONFIG_LATENCYTOP
static int lstats_show_proc(struct seq_file *m, void *v)
{
	int i;
	struct inode *inode = m->private;
	struct task_struct *task = get_proc_task(inode);

	if (!task)
		return -ESRCH;
	seq_puts(m, "Latency Top version : v0.1\n");
	for (i = 0; i < 32; i++) {
		struct latency_record *lr = &task->latency_record[i];
		if (lr->backtrace[0]) {
			int q;
			seq_printf(m, "%i %li %li",
				   lr->count, lr->time, lr->max);
			for (q = 0; q < LT_BACKTRACEDEPTH; q++) {
				unsigned long bt = lr->backtrace[q];
				if (!bt)
					break;
				if (bt == ULONG_MAX)
					break;
				seq_printf(m, " %ps", (void *)bt);
			}
			seq_putc(m, '\n');
		}

	}
	put_task_struct(task);
	return 0;
}

static int lstats_open(struct inode *inode, struct file *file)
{
	return single_open(file, lstats_show_proc, inode);
}

static ssize_t lstats_write(struct file *file, const char __user *buf,
			    size_t count, loff_t *offs)
{
	struct task_struct *task = get_proc_task(file->f_dentry->d_inode);

	if (!task)
		return -ESRCH;
	clear_all_latency_tracing(task);
	put_task_struct(task);

	return count;
}

static const struct file_operations proc_lstats_operations = {
	.open		= lstats_open,
	.read		= seq_read,
	.write		= lstats_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#endif

static int proc_oom_score(struct task_struct *task, char *buffer)
{
	unsigned long points = 0;

	read_lock(&tasklist_lock);
	if (pid_alive(task))
		points = oom_badness(task, NULL, NULL,
					totalram_pages + total_swap_pages);
	read_unlock(&tasklist_lock);
	return sprintf(buffer, "%lu\n", points);
}

struct limit_names {
	char *name;
	char *unit;
};

static const struct limit_names lnames[RLIM_NLIMITS] = {
	[RLIMIT_CPU] = {"Max cpu time", "seconds"},
	[RLIMIT_FSIZE] = {"Max file size", "bytes"},
	[RLIMIT_DATA] = {"Max data size", "bytes"},
	[RLIMIT_STACK] = {"Max stack size", "bytes"},
	[RLIMIT_CORE] = {"Max core file size", "bytes"},
	[RLIMIT_RSS] = {"Max resident set", "bytes"},
	[RLIMIT_NPROC] = {"Max processes", "processes"},
	[RLIMIT_NOFILE] = {"Max open files", "files"},
	[RLIMIT_MEMLOCK] = {"Max locked memory", "bytes"},
	[RLIMIT_AS] = {"Max address space", "bytes"},
	[RLIMIT_LOCKS] = {"Max file locks", "locks"},
	[RLIMIT_SIGPENDING] = {"Max pending signals", "signals"},
	[RLIMIT_MSGQUEUE] = {"Max msgqueue size", "bytes"},
	[RLIMIT_NICE] = {"Max nice priority", NULL},
	[RLIMIT_RTPRIO] = {"Max realtime priority", NULL},
	[RLIMIT_RTTIME] = {"Max realtime timeout", "us"},
};

static int proc_pid_limits(struct task_struct *task, char *buffer)
{
	unsigned int i;
	int count = 0;
	unsigned long flags;
	char *bufptr = buffer;

	struct rlimit rlim[RLIM_NLIMITS];

	if (!lock_task_sighand(task, &flags))
		return 0;
	memcpy(rlim, task->signal->rlim, sizeof(struct rlimit) * RLIM_NLIMITS);
	unlock_task_sighand(task, &flags);

	count += sprintf(&bufptr[count], "%-25s %-20s %-20s %-10s\n",
			"Limit", "Soft Limit", "Hard Limit", "Units");

	for (i = 0; i < RLIM_NLIMITS; i++) {
		if (rlim[i].rlim_cur == RLIM_INFINITY)
			count += sprintf(&bufptr[count], "%-25s %-20s ",
					 lnames[i].name, "unlimited");
		else
			count += sprintf(&bufptr[count], "%-25s %-20lu ",
					 lnames[i].name, rlim[i].rlim_cur);

		if (rlim[i].rlim_max == RLIM_INFINITY)
			count += sprintf(&bufptr[count], "%-20s ", "unlimited");
		else
			count += sprintf(&bufptr[count], "%-20lu ",
					 rlim[i].rlim_max);

		if (lnames[i].unit)
			count += sprintf(&bufptr[count], "%-10s\n",
					 lnames[i].unit);
		else
			count += sprintf(&bufptr[count], "\n");
	}

	return count;
}

#ifdef CONFIG_HAVE_ARCH_TRACEHOOK
static int proc_pid_syscall(struct task_struct *task, char *buffer)
{
	long nr;
	unsigned long args[6], sp, pc;
	int res = lock_trace(task);
	if (res)
		return res;

	if (task_current_syscall(task, &nr, args, 6, &sp, &pc))
		res = sprintf(buffer, "running\n");
	else if (nr < 0)
		res = sprintf(buffer, "%ld 0x%lx 0x%lx\n", nr, sp, pc);
	else
		res = sprintf(buffer,
		       "%ld 0x%lx 0x%lx 0x%lx 0x%lx 0x%lx 0x%lx 0x%lx 0x%lx\n",
		       nr,
		       args[0], args[1], args[2], args[3], args[4], args[5],
		       sp, pc);
	unlock_trace(task);
	return res;
}
#endif 


static int proc_fd_access_allowed(struct inode *inode)
{
	struct task_struct *task;
	int allowed = 0;
	task = get_proc_task(inode);
	if (task) {
		allowed = ptrace_may_access(task, PTRACE_MODE_READ);
		put_task_struct(task);
	}
	return allowed;
}

int proc_setattr(struct dentry *dentry, struct iattr *attr)
{
	int error;
	struct inode *inode = dentry->d_inode;

	if (attr->ia_valid & ATTR_MODE)
		return -EPERM;

	error = inode_change_ok(inode, attr);
	if (error)
		return error;

	if ((attr->ia_valid & ATTR_SIZE) &&
	    attr->ia_size != i_size_read(inode)) {
		error = vmtruncate(inode, attr->ia_size);
		if (error)
			return error;
	}

	setattr_copy(inode, attr);
	mark_inode_dirty(inode);
	return 0;
}

static bool has_pid_permissions(struct pid_namespace *pid,
				 struct task_struct *task,
				 int hide_pid_min)
{
	if (pid->hide_pid < hide_pid_min)
		return true;
	if (in_group_p(pid->pid_gid))
		return true;
	return ptrace_may_access(task, PTRACE_MODE_READ);
}


static int proc_pid_permission(struct inode *inode, int mask)
{
	struct pid_namespace *pid = inode->i_sb->s_fs_info;
	struct task_struct *task;
	bool has_perms;

	task = get_proc_task(inode);
	if (!task)
		return -ESRCH;
	has_perms = has_pid_permissions(pid, task, 1);
	put_task_struct(task);

	if (!has_perms) {
		if (pid->hide_pid == 2) {
			return -ENOENT;
		}

		return -EPERM;
	}
	return generic_permission(inode, mask);
}



static const struct inode_operations proc_def_inode_operations = {
	.setattr	= proc_setattr,
};

#define PROC_BLOCK_SIZE	(3*1024)		

static ssize_t proc_info_read(struct file * file, char __user * buf,
			  size_t count, loff_t *ppos)
{
	struct inode * inode = file->f_path.dentry->d_inode;
	unsigned long page;
	ssize_t length;
	struct task_struct *task = get_proc_task(inode);

	length = -ESRCH;
	if (!task)
		goto out_no_task;

	if (count > PROC_BLOCK_SIZE)
		count = PROC_BLOCK_SIZE;

	length = -ENOMEM;
	if (!(page = __get_free_page(GFP_TEMPORARY)))
		goto out;

	length = PROC_I(inode)->op.proc_read(task, (char*)page);

	if (length >= 0)
		length = simple_read_from_buffer(buf, count, ppos, (char *)page, length);
	free_page(page);
out:
	put_task_struct(task);
out_no_task:
	return length;
}

static const struct file_operations proc_info_file_operations = {
	.read		= proc_info_read,
	.llseek		= generic_file_llseek,
};

static int proc_single_show(struct seq_file *m, void *v)
{
	struct inode *inode = m->private;
	struct pid_namespace *ns;
	struct pid *pid;
	struct task_struct *task;
	int ret;

	ns = inode->i_sb->s_fs_info;
	pid = proc_pid(inode);
	task = get_pid_task(pid, PIDTYPE_PID);
	if (!task)
		return -ESRCH;

	ret = PROC_I(inode)->op.proc_show(m, ns, pid, task);

	put_task_struct(task);
	return ret;
}

static int proc_single_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, proc_single_show, inode);
}

static const struct file_operations proc_single_file_operations = {
	.open		= proc_single_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int mem_open(struct inode* inode, struct file* file)
{
	struct task_struct *task = get_proc_task(file->f_path.dentry->d_inode);
	struct mm_struct *mm;

	if (!task)
		return -ESRCH;

	mm = mm_access(task, PTRACE_MODE_ATTACH);
	put_task_struct(task);

	if (IS_ERR(mm))
		return PTR_ERR(mm);

	if (mm) {
		
		atomic_inc(&mm->mm_count);
		
		mmput(mm);
	}

	
	file->f_mode |= FMODE_UNSIGNED_OFFSET;
	file->private_data = mm;

	return 0;
}

static ssize_t mem_rw(struct file *file, char __user *buf,
			size_t count, loff_t *ppos, int write)
{
	struct mm_struct *mm = file->private_data;
	unsigned long addr = *ppos;
	ssize_t copied;
	char *page;

	if (!mm)
		return 0;

	page = (char *)__get_free_page(GFP_TEMPORARY);
	if (!page)
		return -ENOMEM;

	copied = 0;
	if (!atomic_inc_not_zero(&mm->mm_users))
		goto free;

	while (count > 0) {
		int this_len = min_t(int, count, PAGE_SIZE);

		if (write && copy_from_user(page, buf, this_len)) {
			copied = -EFAULT;
			break;
		}

		this_len = access_remote_vm(mm, addr, page, this_len, write);
		if (!this_len) {
			if (!copied)
				copied = -EIO;
			break;
		}

		if (!write && copy_to_user(buf, page, this_len)) {
			copied = -EFAULT;
			break;
		}

		buf += this_len;
		addr += this_len;
		copied += this_len;
		count -= this_len;
	}
	*ppos = addr;

	mmput(mm);
free:
	free_page((unsigned long) page);
	return copied;
}

static ssize_t mem_read(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	return mem_rw(file, buf, count, ppos, 0);
}

static ssize_t mem_write(struct file *file, const char __user *buf,
			 size_t count, loff_t *ppos)
{
	return mem_rw(file, (char __user*)buf, count, ppos, 1);
}

loff_t mem_lseek(struct file *file, loff_t offset, int orig)
{
	switch (orig) {
	case 0:
		file->f_pos = offset;
		break;
	case 1:
		file->f_pos += offset;
		break;
	default:
		return -EINVAL;
	}
	force_successful_syscall_return();
	return file->f_pos;
}

static int mem_release(struct inode *inode, struct file *file)
{
	struct mm_struct *mm = file->private_data;
	if (mm)
		mmdrop(mm);
	return 0;
}

static const struct file_operations proc_mem_operations = {
	.llseek		= mem_lseek,
	.read		= mem_read,
	.write		= mem_write,
	.open		= mem_open,
	.release	= mem_release,
};

static ssize_t environ_read(struct file *file, char __user *buf,
			size_t count, loff_t *ppos)
{
	struct task_struct *task = get_proc_task(file->f_dentry->d_inode);
	char *page;
	unsigned long src = *ppos;
	int ret = -ESRCH;
	struct mm_struct *mm;

	if (!task)
		goto out_no_task;

	ret = -ENOMEM;
	page = (char *)__get_free_page(GFP_TEMPORARY);
	if (!page)
		goto out;


	mm = mm_for_maps(task);
	ret = PTR_ERR(mm);
	if (!mm || IS_ERR(mm))
		goto out_free;

	ret = 0;
	while (count > 0) {
		int this_len, retval, max_len;

		this_len = mm->env_end - (mm->env_start + src);

		if (this_len <= 0)
			break;

		max_len = (count > PAGE_SIZE) ? PAGE_SIZE : count;
		this_len = (this_len > max_len) ? max_len : this_len;

		retval = access_process_vm(task, (mm->env_start + src),
			page, this_len, 0);

		if (retval <= 0) {
			ret = retval;
			break;
		}

		if (copy_to_user(buf, page, retval)) {
			ret = -EFAULT;
			break;
		}

		ret += retval;
		src += retval;
		buf += retval;
		count -= retval;
	}
	*ppos = src;

	mmput(mm);
out_free:
	free_page((unsigned long) page);
out:
	put_task_struct(task);
out_no_task:
	return ret;
}

static const struct file_operations proc_environ_operations = {
	.read		= environ_read,
	.llseek		= generic_file_llseek,
};

static ssize_t oom_adjust_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	struct task_struct *task = get_proc_task(file->f_path.dentry->d_inode);
	char buffer[PROC_NUMBUF];
	size_t len;
	int oom_adjust = OOM_DISABLE;
	unsigned long flags;

	if (!task)
		return -ESRCH;

	if (lock_task_sighand(task, &flags)) {
		oom_adjust = task->signal->oom_adj;
		unlock_task_sighand(task, &flags);
	}

	put_task_struct(task);

	len = snprintf(buffer, sizeof(buffer), "%i\n", oom_adjust);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static ssize_t oom_adjust_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct task_struct *task;
	char buffer[PROC_NUMBUF];
	int oom_adjust;
	unsigned long flags;
	int err;

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count)) {
		err = -EFAULT;
		goto out;
	}

	err = kstrtoint(strstrip(buffer), 0, &oom_adjust);
	if (err)
		goto out;
	if ((oom_adjust < OOM_ADJUST_MIN || oom_adjust > OOM_ADJUST_MAX) &&
	     oom_adjust != OOM_DISABLE) {
		err = -EINVAL;
		goto out;
	}

	task = get_proc_task(file->f_path.dentry->d_inode);
	if (!task) {
		err = -ESRCH;
		goto out;
	}

	task_lock(task);
	if (!task->mm) {
		err = -EINVAL;
		goto err_task_lock;
	}

	if (!lock_task_sighand(task, &flags)) {
		err = -ESRCH;
		goto err_task_lock;
	}

	if (oom_adjust < task->signal->oom_adj && !capable(CAP_SYS_RESOURCE)) {
		err = -EACCES;
		goto err_sighand;
	}

	printk_once(KERN_WARNING "%s (%d): /proc/%d/oom_adj is deprecated, please use /proc/%d/oom_score_adj instead.\n",
		  current->comm, task_pid_nr(current), task_pid_nr(task),
		  task_pid_nr(task));
	task->signal->oom_adj = oom_adjust;
	if (task->signal->oom_adj == OOM_ADJUST_MAX)
		task->signal->oom_score_adj = OOM_SCORE_ADJ_MAX;
	else
		task->signal->oom_score_adj = (oom_adjust * OOM_SCORE_ADJ_MAX) /
								-OOM_DISABLE;
	trace_oom_score_adj_update(task);
err_sighand:
	unlock_task_sighand(task, &flags);
err_task_lock:
	task_unlock(task);
	put_task_struct(task);
out:
	return err < 0 ? err : count;
}

static int oom_adjust_permission(struct inode *inode, int mask)
{
	uid_t uid;
	struct task_struct *p;

	p = get_proc_task(inode);
	if(p) {
		uid = task_uid(p);
		put_task_struct(p);
	}

	if (p && (current_fsuid() == 1000) && (uid >= 1000)) {
		if (inode->i_mode >> 6 & mask) {
			return 0;
		}
	}

	
	return generic_permission(inode, mask);
}

static const struct inode_operations proc_oom_adjust_inode_operations = {
	.permission	= oom_adjust_permission,
};

static const struct file_operations proc_oom_adjust_operations = {
	.read		= oom_adjust_read,
	.write		= oom_adjust_write,
	.llseek		= generic_file_llseek,
};

static ssize_t oom_score_adj_read(struct file *file, char __user *buf,
					size_t count, loff_t *ppos)
{
	struct task_struct *task = get_proc_task(file->f_path.dentry->d_inode);
	char buffer[PROC_NUMBUF];
	int oom_score_adj = OOM_SCORE_ADJ_MIN;
	unsigned long flags;
	size_t len;

	if (!task)
		return -ESRCH;
	if (lock_task_sighand(task, &flags)) {
		oom_score_adj = task->signal->oom_score_adj;
		unlock_task_sighand(task, &flags);
	}
	put_task_struct(task);
	len = snprintf(buffer, sizeof(buffer), "%d\n", oom_score_adj);
	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static ssize_t oom_score_adj_write(struct file *file, const char __user *buf,
					size_t count, loff_t *ppos)
{
	struct task_struct *task;
	char buffer[PROC_NUMBUF];
	unsigned long flags;
	int oom_score_adj;
	int err;

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count)) {
		err = -EFAULT;
		goto out;
	}

	err = kstrtoint(strstrip(buffer), 0, &oom_score_adj);
	if (err)
		goto out;
	if (oom_score_adj < OOM_SCORE_ADJ_MIN ||
			oom_score_adj > OOM_SCORE_ADJ_MAX) {
		err = -EINVAL;
		goto out;
	}

	task = get_proc_task(file->f_path.dentry->d_inode);
	if (!task) {
		err = -ESRCH;
		goto out;
	}

	task_lock(task);
	if (!task->mm) {
		err = -EINVAL;
		goto err_task_lock;
	}

	if (!lock_task_sighand(task, &flags)) {
		err = -ESRCH;
		goto err_task_lock;
	}

	if (oom_score_adj < task->signal->oom_score_adj_min &&
			!capable(CAP_SYS_RESOURCE)) {
		err = -EACCES;
		goto err_sighand;
	}

	task->signal->oom_score_adj = oom_score_adj;
	if (has_capability_noaudit(current, CAP_SYS_RESOURCE))
		task->signal->oom_score_adj_min = oom_score_adj;
	trace_oom_score_adj_update(task);
	if (task->signal->oom_score_adj == OOM_SCORE_ADJ_MAX) {
		task->signal->oom_adj = OOM_ADJUST_MAX;
	} else {
		int mult = 1;
		if (task->signal->oom_score_adj < 0)
			mult = -1;
			task->signal->oom_adj = roundup(mult * task->signal->oom_score_adj *
					-OOM_DISABLE, OOM_SCORE_ADJ_MAX) /
					OOM_SCORE_ADJ_MAX * mult;
	}
err_sighand:
	unlock_task_sighand(task, &flags);
err_task_lock:
	task_unlock(task);
	put_task_struct(task);
out:
	return err < 0 ? err : count;
}

static const struct file_operations proc_oom_score_adj_operations = {
	.read		= oom_score_adj_read,
	.write		= oom_score_adj_write,
	.llseek		= default_llseek,
};

#ifdef CONFIG_AUDITSYSCALL
#define TMPBUFLEN 21
static ssize_t proc_loginuid_read(struct file * file, char __user * buf,
				  size_t count, loff_t *ppos)
{
	struct inode * inode = file->f_path.dentry->d_inode;
	struct task_struct *task = get_proc_task(inode);
	ssize_t length;
	char tmpbuf[TMPBUFLEN];

	if (!task)
		return -ESRCH;
	length = scnprintf(tmpbuf, TMPBUFLEN, "%u",
				audit_get_loginuid(task));
	put_task_struct(task);
	return simple_read_from_buffer(buf, count, ppos, tmpbuf, length);
}

static ssize_t proc_loginuid_write(struct file * file, const char __user * buf,
				   size_t count, loff_t *ppos)
{
	struct inode * inode = file->f_path.dentry->d_inode;
	char *page, *tmp;
	ssize_t length;
	uid_t loginuid;

	rcu_read_lock();
	if (current != pid_task(proc_pid(inode), PIDTYPE_PID)) {
		rcu_read_unlock();
		return -EPERM;
	}
	rcu_read_unlock();

	if (count >= PAGE_SIZE)
		count = PAGE_SIZE - 1;

	if (*ppos != 0) {
		
		return -EINVAL;
	}
	page = (char*)__get_free_page(GFP_TEMPORARY);
	if (!page)
		return -ENOMEM;
	length = -EFAULT;
	if (copy_from_user(page, buf, count))
		goto out_free_page;

	page[count] = '\0';
	loginuid = simple_strtoul(page, &tmp, 10);
	if (tmp == page) {
		length = -EINVAL;
		goto out_free_page;

	}
	length = audit_set_loginuid(loginuid);
	if (likely(length == 0))
		length = count;

out_free_page:
	free_page((unsigned long) page);
	return length;
}

static const struct file_operations proc_loginuid_operations = {
	.read		= proc_loginuid_read,
	.write		= proc_loginuid_write,
	.llseek		= generic_file_llseek,
};

static ssize_t proc_sessionid_read(struct file * file, char __user * buf,
				  size_t count, loff_t *ppos)
{
	struct inode * inode = file->f_path.dentry->d_inode;
	struct task_struct *task = get_proc_task(inode);
	ssize_t length;
	char tmpbuf[TMPBUFLEN];

	if (!task)
		return -ESRCH;
	length = scnprintf(tmpbuf, TMPBUFLEN, "%u",
				audit_get_sessionid(task));
	put_task_struct(task);
	return simple_read_from_buffer(buf, count, ppos, tmpbuf, length);
}

static const struct file_operations proc_sessionid_operations = {
	.read		= proc_sessionid_read,
	.llseek		= generic_file_llseek,
};
#endif

#ifdef CONFIG_FAULT_INJECTION
static ssize_t proc_fault_inject_read(struct file * file, char __user * buf,
				      size_t count, loff_t *ppos)
{
	struct task_struct *task = get_proc_task(file->f_dentry->d_inode);
	char buffer[PROC_NUMBUF];
	size_t len;
	int make_it_fail;

	if (!task)
		return -ESRCH;
	make_it_fail = task->make_it_fail;
	put_task_struct(task);

	len = snprintf(buffer, sizeof(buffer), "%i\n", make_it_fail);

	return simple_read_from_buffer(buf, count, ppos, buffer, len);
}

static ssize_t proc_fault_inject_write(struct file * file,
			const char __user * buf, size_t count, loff_t *ppos)
{
	struct task_struct *task;
	char buffer[PROC_NUMBUF], *end;
	int make_it_fail;

	if (!capable(CAP_SYS_RESOURCE))
		return -EPERM;
	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count))
		return -EFAULT;
	make_it_fail = simple_strtol(strstrip(buffer), &end, 0);
	if (*end)
		return -EINVAL;
	task = get_proc_task(file->f_dentry->d_inode);
	if (!task)
		return -ESRCH;
	task->make_it_fail = make_it_fail;
	put_task_struct(task);

	return count;
}

static const struct file_operations proc_fault_inject_operations = {
	.read		= proc_fault_inject_read,
	.write		= proc_fault_inject_write,
	.llseek		= generic_file_llseek,
};
#endif


#ifdef CONFIG_SCHED_DEBUG
static int sched_show(struct seq_file *m, void *v)
{
	struct inode *inode = m->private;
	struct task_struct *p;

	p = get_proc_task(inode);
	if (!p)
		return -ESRCH;
	proc_sched_show_task(p, m);

	put_task_struct(p);

	return 0;
}

static ssize_t
sched_write(struct file *file, const char __user *buf,
	    size_t count, loff_t *offset)
{
	struct inode *inode = file->f_path.dentry->d_inode;
	struct task_struct *p;

	p = get_proc_task(inode);
	if (!p)
		return -ESRCH;
	proc_sched_set_task(p);

	put_task_struct(p);

	return count;
}

static int sched_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, sched_show, inode);
}

static const struct file_operations proc_pid_sched_operations = {
	.open		= sched_open,
	.read		= seq_read,
	.write		= sched_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#endif

#ifdef CONFIG_SCHED_AUTOGROUP
static int sched_autogroup_show(struct seq_file *m, void *v)
{
	struct inode *inode = m->private;
	struct task_struct *p;

	p = get_proc_task(inode);
	if (!p)
		return -ESRCH;
	proc_sched_autogroup_show_task(p, m);

	put_task_struct(p);

	return 0;
}

static ssize_t
sched_autogroup_write(struct file *file, const char __user *buf,
	    size_t count, loff_t *offset)
{
	struct inode *inode = file->f_path.dentry->d_inode;
	struct task_struct *p;
	char buffer[PROC_NUMBUF];
	int nice;
	int err;

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	err = kstrtoint(strstrip(buffer), 0, &nice);
	if (err < 0)
		return err;

	p = get_proc_task(inode);
	if (!p)
		return -ESRCH;

	err = proc_sched_autogroup_set_nice(p, nice);
	if (err)
		count = err;

	put_task_struct(p);

	return count;
}

static int sched_autogroup_open(struct inode *inode, struct file *filp)
{
	int ret;

	ret = single_open(filp, sched_autogroup_show, NULL);
	if (!ret) {
		struct seq_file *m = filp->private_data;

		m->private = inode;
	}
	return ret;
}

static const struct file_operations proc_pid_sched_autogroup_operations = {
	.open		= sched_autogroup_open,
	.read		= seq_read,
	.write		= sched_autogroup_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#endif 

static ssize_t comm_write(struct file *file, const char __user *buf,
				size_t count, loff_t *offset)
{
	struct inode *inode = file->f_path.dentry->d_inode;
	struct task_struct *p;
	char buffer[TASK_COMM_LEN];

	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	p = get_proc_task(inode);
	if (!p)
		return -ESRCH;

	if (same_thread_group(current, p))
		set_task_comm(p, buffer);
	else
		count = -EINVAL;

	put_task_struct(p);

	return count;
}

static int comm_show(struct seq_file *m, void *v)
{
	struct inode *inode = m->private;
	struct task_struct *p;

	p = get_proc_task(inode);
	if (!p)
		return -ESRCH;

	task_lock(p);
	seq_printf(m, "%s\n", p->comm);
	task_unlock(p);

	put_task_struct(p);

	return 0;
}

static int comm_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, comm_show, inode);
}

static const struct file_operations proc_pid_set_comm_operations = {
	.open		= comm_open,
	.read		= seq_read,
	.write		= comm_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int proc_exe_link(struct dentry *dentry, struct path *exe_path)
{
	struct task_struct *task;
	struct mm_struct *mm;
	struct file *exe_file;

	task = get_proc_task(dentry->d_inode);
	if (!task)
		return -ENOENT;
	mm = get_task_mm(task);
	put_task_struct(task);
	if (!mm)
		return -ENOENT;
	exe_file = get_mm_exe_file(mm);
	mmput(mm);
	if (exe_file) {
		*exe_path = exe_file->f_path;
		path_get(&exe_file->f_path);
		fput(exe_file);
		return 0;
	} else
		return -ENOENT;
}

static void *proc_pid_follow_link(struct dentry *dentry, struct nameidata *nd)
{
	struct inode *inode = dentry->d_inode;
	int error = -EACCES;

	
	path_put(&nd->path);

	
	if (!proc_fd_access_allowed(inode))
		goto out;

	error = PROC_I(inode)->op.proc_get_link(dentry, &nd->path);
out:
	return ERR_PTR(error);
}

static int do_proc_readlink(struct path *path, char __user *buffer, int buflen)
{
	char *tmp = (char*)__get_free_page(GFP_TEMPORARY);
	char *pathname;
	int len;

	if (!tmp)
		return -ENOMEM;

	pathname = d_path(path, tmp, PAGE_SIZE);
	len = PTR_ERR(pathname);
	if (IS_ERR(pathname))
		goto out;
	len = tmp + PAGE_SIZE - 1 - pathname;

	if (len > buflen)
		len = buflen;
	if (copy_to_user(buffer, pathname, len))
		len = -EFAULT;
 out:
	free_page((unsigned long)tmp);
	return len;
}

static int proc_pid_readlink(struct dentry * dentry, char __user * buffer, int buflen)
{
	int error = -EACCES;
	struct inode *inode = dentry->d_inode;
	struct path path;

	
	if (!proc_fd_access_allowed(inode))
		goto out;

	error = PROC_I(inode)->op.proc_get_link(dentry, &path);
	if (error)
		goto out;

	error = do_proc_readlink(&path, buffer, buflen);
	path_put(&path);
out:
	return error;
}

static const struct inode_operations proc_pid_link_inode_operations = {
	.readlink	= proc_pid_readlink,
	.follow_link	= proc_pid_follow_link,
	.setattr	= proc_setattr,
};



static int task_dumpable(struct task_struct *task)
{
	int dumpable = 0;
	struct mm_struct *mm;

	task_lock(task);
	mm = task->mm;
	if (mm)
		dumpable = get_dumpable(mm);
	task_unlock(task);
	if(dumpable == 1)
		return 1;
	return 0;
}

struct inode *proc_pid_make_inode(struct super_block * sb, struct task_struct *task)
{
	struct inode * inode;
	struct proc_inode *ei;
	const struct cred *cred;

	

	inode = new_inode(sb);
	if (!inode)
		goto out;

	
	ei = PROC_I(inode);
	inode->i_ino = get_next_ino();
	inode->i_mtime = inode->i_atime = inode->i_ctime = CURRENT_TIME;
	inode->i_op = &proc_def_inode_operations;

	ei->pid = get_task_pid(task, PIDTYPE_PID);
	if (!ei->pid)
		goto out_unlock;

	if (task_dumpable(task)) {
		rcu_read_lock();
		cred = __task_cred(task);
		inode->i_uid = cred->euid;
		inode->i_gid = cred->egid;
		rcu_read_unlock();
	}
	security_task_to_inode(task, inode);

out:
	return inode;

out_unlock:
	iput(inode);
	return NULL;
}

int pid_getattr(struct vfsmount *mnt, struct dentry *dentry, struct kstat *stat)
{
	struct inode *inode = dentry->d_inode;
	struct task_struct *task;
	const struct cred *cred;
	struct pid_namespace *pid = dentry->d_sb->s_fs_info;

	generic_fillattr(inode, stat);

	rcu_read_lock();
	stat->uid = 0;
	stat->gid = 0;
	task = pid_task(proc_pid(inode), PIDTYPE_PID);
	if (task) {
		if (!has_pid_permissions(pid, task, 2)) {
			rcu_read_unlock();
			return -ENOENT;
		}
		if ((inode->i_mode == (S_IFDIR|S_IRUGO|S_IXUGO)) ||
		    task_dumpable(task)) {
			cred = __task_cred(task);
			stat->uid = cred->euid;
			stat->gid = cred->egid;
		}
	}
	rcu_read_unlock();
	return 0;
}


int pid_revalidate(struct dentry *dentry, struct nameidata *nd)
{
	struct inode *inode;
	struct task_struct *task;
	const struct cred *cred;

	if (nd && nd->flags & LOOKUP_RCU)
		return -ECHILD;

	inode = dentry->d_inode;
	task = get_proc_task(inode);

	if (task) {
		if ((inode->i_mode == (S_IFDIR|S_IRUGO|S_IXUGO)) ||
		    task_dumpable(task)) {
			rcu_read_lock();
			cred = __task_cred(task);
			inode->i_uid = cred->euid;
			inode->i_gid = cred->egid;
			rcu_read_unlock();
		} else {
			inode->i_uid = 0;
			inode->i_gid = 0;
		}
		inode->i_mode &= ~(S_ISUID | S_ISGID);
		security_task_to_inode(task, inode);
		put_task_struct(task);
		return 1;
	}
	d_drop(dentry);
	return 0;
}

static int pid_delete_dentry(const struct dentry * dentry)
{
	return !proc_pid(dentry->d_inode)->tasks[PIDTYPE_PID].first;
}

const struct dentry_operations pid_dentry_operations =
{
	.d_revalidate	= pid_revalidate,
	.d_delete	= pid_delete_dentry,
};


int proc_fill_cache(struct file *filp, void *dirent, filldir_t filldir,
	const char *name, int len,
	instantiate_t instantiate, struct task_struct *task, const void *ptr)
{
	struct dentry *child, *dir = filp->f_path.dentry;
	struct inode *inode;
	struct qstr qname;
	ino_t ino = 0;
	unsigned type = DT_UNKNOWN;

	qname.name = name;
	qname.len  = len;
	qname.hash = full_name_hash(name, len);

	child = d_lookup(dir, &qname);
	if (!child) {
		struct dentry *new;
		new = d_alloc(dir, &qname);
		if (new) {
			child = instantiate(dir->d_inode, new, task, ptr);
			if (child)
				dput(new);
			else
				child = new;
		}
	}
	if (!child || IS_ERR(child) || !child->d_inode)
		goto end_instantiate;
	inode = child->d_inode;
	if (inode) {
		ino = inode->i_ino;
		type = inode->i_mode >> 12;
	}
	dput(child);
end_instantiate:
	if (!ino)
		ino = find_inode_number(dir, &qname);
	if (!ino)
		ino = 1;
	return filldir(dirent, name, len, filp->f_pos, ino, type);
}

static unsigned name_to_int(struct dentry *dentry)
{
	const char *name = dentry->d_name.name;
	int len = dentry->d_name.len;
	unsigned n = 0;

	if (len > 1 && *name == '0')
		goto out;
	while (len-- > 0) {
		unsigned c = *name++ - '0';
		if (c > 9)
			goto out;
		if (n >= (~0U-9)/10)
			goto out;
		n *= 10;
		n += c;
	}
	return n;
out:
	return ~0U;
}

#define PROC_FDINFO_MAX 64

static int proc_fd_info(struct inode *inode, struct path *path, char *info)
{
	struct task_struct *task = get_proc_task(inode);
	struct files_struct *files = NULL;
	struct file *file;
	int fd = proc_fd(inode);

	if (task) {
		files = get_files_struct(task);
		put_task_struct(task);
	}
	if (files) {
		spin_lock(&files->file_lock);
		file = fcheck_files(files, fd);
		if (file) {
			unsigned int f_flags;
			struct fdtable *fdt;

			fdt = files_fdtable(files);
			f_flags = file->f_flags & ~O_CLOEXEC;
			if (close_on_exec(fd, fdt))
				f_flags |= O_CLOEXEC;

			if (path) {
				*path = file->f_path;
				path_get(&file->f_path);
			}
			if (info)
				snprintf(info, PROC_FDINFO_MAX,
					 "pos:\t%lli\n"
					 "flags:\t0%o\n",
					 (long long) file->f_pos,
					 f_flags);
			spin_unlock(&files->file_lock);
			put_files_struct(files);
			return 0;
		}
		spin_unlock(&files->file_lock);
		put_files_struct(files);
	}
	return -ENOENT;
}

static int proc_fd_link(struct dentry *dentry, struct path *path)
{
	return proc_fd_info(dentry->d_inode, path, NULL);
}

static int tid_fd_revalidate(struct dentry *dentry, struct nameidata *nd)
{
	struct inode *inode;
	struct task_struct *task;
	int fd;
	struct files_struct *files;
	const struct cred *cred;

	if (nd && nd->flags & LOOKUP_RCU)
		return -ECHILD;

	inode = dentry->d_inode;
	task = get_proc_task(inode);
	fd = proc_fd(inode);

	if (task) {
		files = get_files_struct(task);
		if (files) {
			struct file *file;
			rcu_read_lock();
			file = fcheck_files(files, fd);
			if (file) {
				unsigned i_mode, f_mode = file->f_mode;

				rcu_read_unlock();
				put_files_struct(files);

				if (task_dumpable(task)) {
					rcu_read_lock();
					cred = __task_cred(task);
					inode->i_uid = cred->euid;
					inode->i_gid = cred->egid;
					rcu_read_unlock();
				} else {
					inode->i_uid = 0;
					inode->i_gid = 0;
				}

				i_mode = S_IFLNK;
				if (f_mode & FMODE_READ)
					i_mode |= S_IRUSR | S_IXUSR;
				if (f_mode & FMODE_WRITE)
					i_mode |= S_IWUSR | S_IXUSR;
				inode->i_mode = i_mode;

				security_task_to_inode(task, inode);
				put_task_struct(task);
				return 1;
			}
			rcu_read_unlock();
			put_files_struct(files);
		}
		put_task_struct(task);
	}
	d_drop(dentry);
	return 0;
}

static const struct dentry_operations tid_fd_dentry_operations =
{
	.d_revalidate	= tid_fd_revalidate,
	.d_delete	= pid_delete_dentry,
};

static struct dentry *proc_fd_instantiate(struct inode *dir,
	struct dentry *dentry, struct task_struct *task, const void *ptr)
{
	unsigned fd = *(const unsigned *)ptr;
 	struct inode *inode;
 	struct proc_inode *ei;
	struct dentry *error = ERR_PTR(-ENOENT);

	inode = proc_pid_make_inode(dir->i_sb, task);
	if (!inode)
		goto out;
	ei = PROC_I(inode);
	ei->fd = fd;

	inode->i_op = &proc_pid_link_inode_operations;
	inode->i_size = 64;
	ei->op.proc_get_link = proc_fd_link;
	d_set_d_op(dentry, &tid_fd_dentry_operations);
	d_add(dentry, inode);
	
	if (tid_fd_revalidate(dentry, NULL))
		error = NULL;

 out:
	return error;
}

static struct dentry *proc_lookupfd_common(struct inode *dir,
					   struct dentry *dentry,
					   instantiate_t instantiate)
{
	struct task_struct *task = get_proc_task(dir);
	unsigned fd = name_to_int(dentry);
	struct dentry *result = ERR_PTR(-ENOENT);

	if (!task)
		goto out_no_task;
	if (fd == ~0U)
		goto out;

	result = instantiate(dir, dentry, task, &fd);
out:
	put_task_struct(task);
out_no_task:
	return result;
}

static int proc_readfd_common(struct file * filp, void * dirent,
			      filldir_t filldir, instantiate_t instantiate)
{
	struct dentry *dentry = filp->f_path.dentry;
	struct inode *inode = dentry->d_inode;
	struct task_struct *p = get_proc_task(inode);
	unsigned int fd, ino;
	int retval;
	struct files_struct * files;

	retval = -ENOENT;
	if (!p)
		goto out_no_task;
	retval = 0;

	fd = filp->f_pos;
	switch (fd) {
		case 0:
			if (filldir(dirent, ".", 1, 0, inode->i_ino, DT_DIR) < 0)
				goto out;
			filp->f_pos++;
		case 1:
			ino = parent_ino(dentry);
			if (filldir(dirent, "..", 2, 1, ino, DT_DIR) < 0)
				goto out;
			filp->f_pos++;
		default:
			files = get_files_struct(p);
			if (!files)
				goto out;
			rcu_read_lock();
			for (fd = filp->f_pos-2;
			     fd < files_fdtable(files)->max_fds;
			     fd++, filp->f_pos++) {
				char name[PROC_NUMBUF];
				int len;

				if (!fcheck_files(files, fd))
					continue;
				rcu_read_unlock();

				len = snprintf(name, sizeof(name), "%d", fd);
				if (proc_fill_cache(filp, dirent, filldir,
						    name, len, instantiate,
						    p, &fd) < 0) {
					rcu_read_lock();
					break;
				}
				rcu_read_lock();
			}
			rcu_read_unlock();
			put_files_struct(files);
	}
out:
	put_task_struct(p);
out_no_task:
	return retval;
}

static struct dentry *proc_lookupfd(struct inode *dir, struct dentry *dentry,
				    struct nameidata *nd)
{
	return proc_lookupfd_common(dir, dentry, proc_fd_instantiate);
}

static int proc_readfd(struct file *filp, void *dirent, filldir_t filldir)
{
	return proc_readfd_common(filp, dirent, filldir, proc_fd_instantiate);
}

static ssize_t proc_fdinfo_read(struct file *file, char __user *buf,
				      size_t len, loff_t *ppos)
{
	char tmp[PROC_FDINFO_MAX];
	int err = proc_fd_info(file->f_path.dentry->d_inode, NULL, tmp);
	if (!err)
		err = simple_read_from_buffer(buf, len, ppos, tmp, strlen(tmp));
	return err;
}

static const struct file_operations proc_fdinfo_file_operations = {
	.open           = nonseekable_open,
	.read		= proc_fdinfo_read,
	.llseek		= no_llseek,
};

static const struct file_operations proc_fd_operations = {
	.read		= generic_read_dir,
	.readdir	= proc_readfd,
	.llseek		= default_llseek,
};

#ifdef CONFIG_CHECKPOINT_RESTORE

static int dname_to_vma_addr(struct dentry *dentry,
			     unsigned long *start, unsigned long *end)
{
	if (sscanf(dentry->d_name.name, "%lx-%lx", start, end) != 2)
		return -EINVAL;

	return 0;
}

static int map_files_d_revalidate(struct dentry *dentry, struct nameidata *nd)
{
	unsigned long vm_start, vm_end;
	bool exact_vma_exists = false;
	struct mm_struct *mm = NULL;
	struct task_struct *task;
	const struct cred *cred;
	struct inode *inode;
	int status = 0;

	if (nd && nd->flags & LOOKUP_RCU)
		return -ECHILD;

	if (!capable(CAP_SYS_ADMIN)) {
		status = -EACCES;
		goto out_notask;
	}

	inode = dentry->d_inode;
	task = get_proc_task(inode);
	if (!task)
		goto out_notask;

	if (!ptrace_may_access(task, PTRACE_MODE_READ))
		goto out;

	mm = get_task_mm(task);
	if (!mm)
		goto out;

	if (!dname_to_vma_addr(dentry, &vm_start, &vm_end)) {
		down_read(&mm->mmap_sem);
		exact_vma_exists = !!find_exact_vma(mm, vm_start, vm_end);
		up_read(&mm->mmap_sem);
	}

	mmput(mm);

	if (exact_vma_exists) {
		if (task_dumpable(task)) {
			rcu_read_lock();
			cred = __task_cred(task);
			inode->i_uid = cred->euid;
			inode->i_gid = cred->egid;
			rcu_read_unlock();
		} else {
			inode->i_uid = 0;
			inode->i_gid = 0;
		}
		security_task_to_inode(task, inode);
		status = 1;
	}

out:
	put_task_struct(task);

out_notask:
	if (status <= 0)
		d_drop(dentry);

	return status;
}

static const struct dentry_operations tid_map_files_dentry_operations = {
	.d_revalidate	= map_files_d_revalidate,
	.d_delete	= pid_delete_dentry,
};

static int proc_map_files_get_link(struct dentry *dentry, struct path *path)
{
	unsigned long vm_start, vm_end;
	struct vm_area_struct *vma;
	struct task_struct *task;
	struct mm_struct *mm;
	int rc;

	rc = -ENOENT;
	task = get_proc_task(dentry->d_inode);
	if (!task)
		goto out;

	mm = get_task_mm(task);
	put_task_struct(task);
	if (!mm)
		goto out;

	rc = dname_to_vma_addr(dentry, &vm_start, &vm_end);
	if (rc)
		goto out_mmput;

	down_read(&mm->mmap_sem);
	vma = find_exact_vma(mm, vm_start, vm_end);
	if (vma && vma->vm_file) {
		*path = vma->vm_file->f_path;
		path_get(path);
		rc = 0;
	}
	up_read(&mm->mmap_sem);

out_mmput:
	mmput(mm);
out:
	return rc;
}

struct map_files_info {
	struct file	*file;
	unsigned long	len;
	unsigned char	name[4*sizeof(long)+2]; 
};

static struct dentry *
proc_map_files_instantiate(struct inode *dir, struct dentry *dentry,
			   struct task_struct *task, const void *ptr)
{
	const struct file *file = ptr;
	struct proc_inode *ei;
	struct inode *inode;

	if (!file)
		return ERR_PTR(-ENOENT);

	inode = proc_pid_make_inode(dir->i_sb, task);
	if (!inode)
		return ERR_PTR(-ENOENT);

	ei = PROC_I(inode);
	ei->op.proc_get_link = proc_map_files_get_link;

	inode->i_op = &proc_pid_link_inode_operations;
	inode->i_size = 64;
	inode->i_mode = S_IFLNK;

	if (file->f_mode & FMODE_READ)
		inode->i_mode |= S_IRUSR;
	if (file->f_mode & FMODE_WRITE)
		inode->i_mode |= S_IWUSR;

	d_set_d_op(dentry, &tid_map_files_dentry_operations);
	d_add(dentry, inode);

	return NULL;
}

static struct dentry *proc_map_files_lookup(struct inode *dir,
		struct dentry *dentry, struct nameidata *nd)
{
	unsigned long vm_start, vm_end;
	struct vm_area_struct *vma;
	struct task_struct *task;
	struct dentry *result;
	struct mm_struct *mm;

	result = ERR_PTR(-EACCES);
	if (!capable(CAP_SYS_ADMIN))
		goto out;

	result = ERR_PTR(-ENOENT);
	task = get_proc_task(dir);
	if (!task)
		goto out;

	result = ERR_PTR(-EACCES);
	if (!ptrace_may_access(task, PTRACE_MODE_READ))
		goto out_put_task;

	result = ERR_PTR(-ENOENT);
	if (dname_to_vma_addr(dentry, &vm_start, &vm_end))
		goto out_put_task;

	mm = get_task_mm(task);
	if (!mm)
		goto out_put_task;

	down_read(&mm->mmap_sem);
	vma = find_exact_vma(mm, vm_start, vm_end);
	if (!vma)
		goto out_no_vma;

	result = proc_map_files_instantiate(dir, dentry, task, vma->vm_file);

out_no_vma:
	up_read(&mm->mmap_sem);
	mmput(mm);
out_put_task:
	put_task_struct(task);
out:
	return result;
}

static const struct inode_operations proc_map_files_inode_operations = {
	.lookup		= proc_map_files_lookup,
	.permission	= proc_fd_permission,
	.setattr	= proc_setattr,
};

static int
proc_map_files_readdir(struct file *filp, void *dirent, filldir_t filldir)
{
	struct dentry *dentry = filp->f_path.dentry;
	struct inode *inode = dentry->d_inode;
	struct vm_area_struct *vma;
	struct task_struct *task;
	struct mm_struct *mm;
	ino_t ino;
	int ret;

	ret = -EACCES;
	if (!capable(CAP_SYS_ADMIN))
		goto out;

	ret = -ENOENT;
	task = get_proc_task(inode);
	if (!task)
		goto out;

	ret = -EACCES;
	if (!ptrace_may_access(task, PTRACE_MODE_READ))
		goto out_put_task;

	ret = 0;
	switch (filp->f_pos) {
	case 0:
		ino = inode->i_ino;
		if (filldir(dirent, ".", 1, 0, ino, DT_DIR) < 0)
			goto out_put_task;
		filp->f_pos++;
	case 1:
		ino = parent_ino(dentry);
		if (filldir(dirent, "..", 2, 1, ino, DT_DIR) < 0)
			goto out_put_task;
		filp->f_pos++;
	default:
	{
		unsigned long nr_files, pos, i;
		struct flex_array *fa = NULL;
		struct map_files_info info;
		struct map_files_info *p;

		mm = get_task_mm(task);
		if (!mm)
			goto out_put_task;
		down_read(&mm->mmap_sem);

		nr_files = 0;


		for (vma = mm->mmap, pos = 2; vma; vma = vma->vm_next) {
			if (vma->vm_file && ++pos > filp->f_pos)
				nr_files++;
		}

		if (nr_files) {
			fa = flex_array_alloc(sizeof(info), nr_files,
						GFP_KERNEL);
			if (!fa || flex_array_prealloc(fa, 0, nr_files,
							GFP_KERNEL)) {
				ret = -ENOMEM;
				if (fa)
					flex_array_free(fa);
				up_read(&mm->mmap_sem);
				mmput(mm);
				goto out_put_task;
			}
			for (i = 0, vma = mm->mmap, pos = 2; vma;
					vma = vma->vm_next) {
				if (!vma->vm_file)
					continue;
				if (++pos <= filp->f_pos)
					continue;

				get_file(vma->vm_file);
				info.file = vma->vm_file;
				info.len = snprintf(info.name,
						sizeof(info.name), "%lx-%lx",
						vma->vm_start, vma->vm_end);
				if (flex_array_put(fa, i++, &info, GFP_KERNEL))
					BUG();
			}
		}
		up_read(&mm->mmap_sem);

		for (i = 0; i < nr_files; i++) {
			p = flex_array_get(fa, i);
			ret = proc_fill_cache(filp, dirent, filldir,
					      p->name, p->len,
					      proc_map_files_instantiate,
					      task, p->file);
			if (ret)
				break;
			filp->f_pos++;
			fput(p->file);
		}
		for (; i < nr_files; i++) {
			p = flex_array_get(fa, i);
			fput(p->file);
		}
		if (fa)
			flex_array_free(fa);
		mmput(mm);
	}
	}

out_put_task:
	put_task_struct(task);
out:
	return ret;
}

static const struct file_operations proc_map_files_operations = {
	.read		= generic_read_dir,
	.readdir	= proc_map_files_readdir,
	.llseek		= default_llseek,
};

#endif 

static int proc_fd_permission(struct inode *inode, int mask)
{
	int rv = generic_permission(inode, mask);
	if (rv == 0)
		return 0;
	if (task_pid(current) == proc_pid(inode))
		rv = 0;
	return rv;
}

static const struct inode_operations proc_fd_inode_operations = {
	.lookup		= proc_lookupfd,
	.permission	= proc_fd_permission,
	.setattr	= proc_setattr,
};

static struct dentry *proc_fdinfo_instantiate(struct inode *dir,
	struct dentry *dentry, struct task_struct *task, const void *ptr)
{
	unsigned fd = *(unsigned *)ptr;
 	struct inode *inode;
 	struct proc_inode *ei;
	struct dentry *error = ERR_PTR(-ENOENT);

	inode = proc_pid_make_inode(dir->i_sb, task);
	if (!inode)
		goto out;
	ei = PROC_I(inode);
	ei->fd = fd;
	inode->i_mode = S_IFREG | S_IRUSR;
	inode->i_fop = &proc_fdinfo_file_operations;
	d_set_d_op(dentry, &tid_fd_dentry_operations);
	d_add(dentry, inode);
	
	if (tid_fd_revalidate(dentry, NULL))
		error = NULL;

 out:
	return error;
}

static struct dentry *proc_lookupfdinfo(struct inode *dir,
					struct dentry *dentry,
					struct nameidata *nd)
{
	return proc_lookupfd_common(dir, dentry, proc_fdinfo_instantiate);
}

static int proc_readfdinfo(struct file *filp, void *dirent, filldir_t filldir)
{
	return proc_readfd_common(filp, dirent, filldir,
				  proc_fdinfo_instantiate);
}

static const struct file_operations proc_fdinfo_operations = {
	.read		= generic_read_dir,
	.readdir	= proc_readfdinfo,
	.llseek		= default_llseek,
};

static const struct inode_operations proc_fdinfo_inode_operations = {
	.lookup		= proc_lookupfdinfo,
	.setattr	= proc_setattr,
};


static struct dentry *proc_pident_instantiate(struct inode *dir,
	struct dentry *dentry, struct task_struct *task, const void *ptr)
{
	const struct pid_entry *p = ptr;
	struct inode *inode;
	struct proc_inode *ei;
	struct dentry *error = ERR_PTR(-ENOENT);

	inode = proc_pid_make_inode(dir->i_sb, task);
	if (!inode)
		goto out;

	ei = PROC_I(inode);
	inode->i_mode = p->mode;
	if (S_ISDIR(inode->i_mode))
		set_nlink(inode, 2);	
	if (p->iop)
		inode->i_op = p->iop;
	if (p->fop)
		inode->i_fop = p->fop;
	ei->op = p->op;
	d_set_d_op(dentry, &pid_dentry_operations);
	d_add(dentry, inode);
	
	if (pid_revalidate(dentry, NULL))
		error = NULL;
out:
	return error;
}

static struct dentry *proc_pident_lookup(struct inode *dir, 
					 struct dentry *dentry,
					 const struct pid_entry *ents,
					 unsigned int nents)
{
	struct dentry *error;
	struct task_struct *task = get_proc_task(dir);
	const struct pid_entry *p, *last;

	error = ERR_PTR(-ENOENT);

	if (!task)
		goto out_no_task;

	last = &ents[nents - 1];
	for (p = ents; p <= last; p++) {
		if (p->len != dentry->d_name.len)
			continue;
		if (!memcmp(dentry->d_name.name, p->name, p->len))
			break;
	}
	if (p > last)
		goto out;

	error = proc_pident_instantiate(dir, dentry, task, p);
out:
	put_task_struct(task);
out_no_task:
	return error;
}

static int proc_pident_fill_cache(struct file *filp, void *dirent,
	filldir_t filldir, struct task_struct *task, const struct pid_entry *p)
{
	return proc_fill_cache(filp, dirent, filldir, p->name, p->len,
				proc_pident_instantiate, task, p);
}

static int proc_pident_readdir(struct file *filp,
		void *dirent, filldir_t filldir,
		const struct pid_entry *ents, unsigned int nents)
{
	int i;
	struct dentry *dentry = filp->f_path.dentry;
	struct inode *inode = dentry->d_inode;
	struct task_struct *task = get_proc_task(inode);
	const struct pid_entry *p, *last;
	ino_t ino;
	int ret;

	ret = -ENOENT;
	if (!task)
		goto out_no_task;

	ret = 0;
	i = filp->f_pos;
	switch (i) {
	case 0:
		ino = inode->i_ino;
		if (filldir(dirent, ".", 1, i, ino, DT_DIR) < 0)
			goto out;
		i++;
		filp->f_pos++;
		
	case 1:
		ino = parent_ino(dentry);
		if (filldir(dirent, "..", 2, i, ino, DT_DIR) < 0)
			goto out;
		i++;
		filp->f_pos++;
		
	default:
		i -= 2;
		if (i >= nents) {
			ret = 1;
			goto out;
		}
		p = ents + i;
		last = &ents[nents - 1];
		while (p <= last) {
			if (proc_pident_fill_cache(filp, dirent, filldir, task, p) < 0)
				goto out;
			filp->f_pos++;
			p++;
		}
	}

	ret = 1;
out:
	put_task_struct(task);
out_no_task:
	return ret;
}

#ifdef CONFIG_SECURITY
static ssize_t proc_pid_attr_read(struct file * file, char __user * buf,
				  size_t count, loff_t *ppos)
{
	struct inode * inode = file->f_path.dentry->d_inode;
	char *p = NULL;
	ssize_t length;
	struct task_struct *task = get_proc_task(inode);

	if (!task)
		return -ESRCH;

	length = security_getprocattr(task,
				      (char*)file->f_path.dentry->d_name.name,
				      &p);
	put_task_struct(task);
	if (length > 0)
		length = simple_read_from_buffer(buf, count, ppos, p, length);
	kfree(p);
	return length;
}

static ssize_t proc_pid_attr_write(struct file * file, const char __user * buf,
				   size_t count, loff_t *ppos)
{
	struct inode * inode = file->f_path.dentry->d_inode;
	char *page;
	ssize_t length;
	struct task_struct *task = get_proc_task(inode);

	length = -ESRCH;
	if (!task)
		goto out_no_task;
	if (count > PAGE_SIZE)
		count = PAGE_SIZE;

	
	length = -EINVAL;
	if (*ppos != 0)
		goto out;

	length = -ENOMEM;
	page = (char*)__get_free_page(GFP_TEMPORARY);
	if (!page)
		goto out;

	length = -EFAULT;
	if (copy_from_user(page, buf, count))
		goto out_free;

	
	length = mutex_lock_interruptible(&task->signal->cred_guard_mutex);
	if (length < 0)
		goto out_free;

	length = security_setprocattr(task,
				      (char*)file->f_path.dentry->d_name.name,
				      (void*)page, count);
	mutex_unlock(&task->signal->cred_guard_mutex);
out_free:
	free_page((unsigned long) page);
out:
	put_task_struct(task);
out_no_task:
	return length;
}

static const struct file_operations proc_pid_attr_operations = {
	.read		= proc_pid_attr_read,
	.write		= proc_pid_attr_write,
	.llseek		= generic_file_llseek,
};

static const struct pid_entry attr_dir_stuff[] = {
	REG("current",    S_IRUGO|S_IWUGO, proc_pid_attr_operations),
	REG("prev",       S_IRUGO,	   proc_pid_attr_operations),
	REG("exec",       S_IRUGO|S_IWUGO, proc_pid_attr_operations),
	REG("fscreate",   S_IRUGO|S_IWUGO, proc_pid_attr_operations),
	REG("keycreate",  S_IRUGO|S_IWUGO, proc_pid_attr_operations),
	REG("sockcreate", S_IRUGO|S_IWUGO, proc_pid_attr_operations),
};

static int proc_attr_dir_readdir(struct file * filp,
			     void * dirent, filldir_t filldir)
{
	return proc_pident_readdir(filp,dirent,filldir,
				   attr_dir_stuff,ARRAY_SIZE(attr_dir_stuff));
}

static const struct file_operations proc_attr_dir_operations = {
	.read		= generic_read_dir,
	.readdir	= proc_attr_dir_readdir,
	.llseek		= default_llseek,
};

static struct dentry *proc_attr_dir_lookup(struct inode *dir,
				struct dentry *dentry, struct nameidata *nd)
{
	return proc_pident_lookup(dir, dentry,
				  attr_dir_stuff, ARRAY_SIZE(attr_dir_stuff));
}

static const struct inode_operations proc_attr_dir_inode_operations = {
	.lookup		= proc_attr_dir_lookup,
	.getattr	= pid_getattr,
	.setattr	= proc_setattr,
};

#endif

#ifdef CONFIG_ELF_CORE
static ssize_t proc_coredump_filter_read(struct file *file, char __user *buf,
					 size_t count, loff_t *ppos)
{
	struct task_struct *task = get_proc_task(file->f_dentry->d_inode);
	struct mm_struct *mm;
	char buffer[PROC_NUMBUF];
	size_t len;
	int ret;

	if (!task)
		return -ESRCH;

	ret = 0;
	mm = get_task_mm(task);
	if (mm) {
		len = snprintf(buffer, sizeof(buffer), "%08lx\n",
			       ((mm->flags & MMF_DUMP_FILTER_MASK) >>
				MMF_DUMP_FILTER_SHIFT));
		mmput(mm);
		ret = simple_read_from_buffer(buf, count, ppos, buffer, len);
	}

	put_task_struct(task);

	return ret;
}

static ssize_t proc_coredump_filter_write(struct file *file,
					  const char __user *buf,
					  size_t count,
					  loff_t *ppos)
{
	struct task_struct *task;
	struct mm_struct *mm;
	char buffer[PROC_NUMBUF], *end;
	unsigned int val;
	int ret;
	int i;
	unsigned long mask;

	ret = -EFAULT;
	memset(buffer, 0, sizeof(buffer));
	if (count > sizeof(buffer) - 1)
		count = sizeof(buffer) - 1;
	if (copy_from_user(buffer, buf, count))
		goto out_no_task;

	ret = -EINVAL;
	val = (unsigned int)simple_strtoul(buffer, &end, 0);
	if (*end == '\n')
		end++;
	if (end - buffer == 0)
		goto out_no_task;

	ret = -ESRCH;
	task = get_proc_task(file->f_dentry->d_inode);
	if (!task)
		goto out_no_task;

	ret = end - buffer;
	mm = get_task_mm(task);
	if (!mm)
		goto out_no_mm;

	for (i = 0, mask = 1; i < MMF_DUMP_FILTER_BITS; i++, mask <<= 1) {
		if (val & mask)
			set_bit(i + MMF_DUMP_FILTER_SHIFT, &mm->flags);
		else
			clear_bit(i + MMF_DUMP_FILTER_SHIFT, &mm->flags);
	}

	mmput(mm);
 out_no_mm:
	put_task_struct(task);
 out_no_task:
	return ret;
}

static const struct file_operations proc_coredump_filter_operations = {
	.read		= proc_coredump_filter_read,
	.write		= proc_coredump_filter_write,
	.llseek		= generic_file_llseek,
};
#endif

static int proc_self_readlink(struct dentry *dentry, char __user *buffer,
			      int buflen)
{
	struct pid_namespace *ns = dentry->d_sb->s_fs_info;
	pid_t tgid = task_tgid_nr_ns(current, ns);
	char tmp[PROC_NUMBUF];
	if (!tgid)
		return -ENOENT;
	sprintf(tmp, "%d", tgid);
	return vfs_readlink(dentry,buffer,buflen,tmp);
}

static void *proc_self_follow_link(struct dentry *dentry, struct nameidata *nd)
{
	struct pid_namespace *ns = dentry->d_sb->s_fs_info;
	pid_t tgid = task_tgid_nr_ns(current, ns);
	char *name = ERR_PTR(-ENOENT);
	if (tgid) {
		name = __getname();
		if (!name)
			name = ERR_PTR(-ENOMEM);
		else
			sprintf(name, "%d", tgid);
	}
	nd_set_link(nd, name);
	return NULL;
}

static void proc_self_put_link(struct dentry *dentry, struct nameidata *nd,
				void *cookie)
{
	char *s = nd_get_link(nd);
	if (!IS_ERR(s))
		__putname(s);
}

static const struct inode_operations proc_self_inode_operations = {
	.readlink	= proc_self_readlink,
	.follow_link	= proc_self_follow_link,
	.put_link	= proc_self_put_link,
};

static const struct pid_entry proc_base_stuff[] = {
	NOD("self", S_IFLNK|S_IRWXUGO,
		&proc_self_inode_operations, NULL, {}),
};

static struct dentry *proc_base_instantiate(struct inode *dir,
	struct dentry *dentry, struct task_struct *task, const void *ptr)
{
	const struct pid_entry *p = ptr;
	struct inode *inode;
	struct proc_inode *ei;
	struct dentry *error;

	
	error = ERR_PTR(-ENOMEM);
	inode = new_inode(dir->i_sb);
	if (!inode)
		goto out;

	
	ei = PROC_I(inode);
	inode->i_ino = get_next_ino();
	inode->i_mtime = inode->i_atime = inode->i_ctime = CURRENT_TIME;

	ei->pid = get_task_pid(task, PIDTYPE_PID);
	if (!ei->pid)
		goto out_iput;

	inode->i_mode = p->mode;
	if (S_ISDIR(inode->i_mode))
		set_nlink(inode, 2);
	if (S_ISLNK(inode->i_mode))
		inode->i_size = 64;
	if (p->iop)
		inode->i_op = p->iop;
	if (p->fop)
		inode->i_fop = p->fop;
	ei->op = p->op;
	d_add(dentry, inode);
	error = NULL;
out:
	return error;
out_iput:
	iput(inode);
	goto out;
}

static struct dentry *proc_base_lookup(struct inode *dir, struct dentry *dentry)
{
	struct dentry *error;
	struct task_struct *task = get_proc_task(dir);
	const struct pid_entry *p, *last;

	error = ERR_PTR(-ENOENT);

	if (!task)
		goto out_no_task;

	
	last = &proc_base_stuff[ARRAY_SIZE(proc_base_stuff) - 1];
	for (p = proc_base_stuff; p <= last; p++) {
		if (p->len != dentry->d_name.len)
			continue;
		if (!memcmp(dentry->d_name.name, p->name, p->len))
			break;
	}
	if (p > last)
		goto out;

	error = proc_base_instantiate(dir, dentry, task, p);

out:
	put_task_struct(task);
out_no_task:
	return error;
}

static int proc_base_fill_cache(struct file *filp, void *dirent,
	filldir_t filldir, struct task_struct *task, const struct pid_entry *p)
{
	return proc_fill_cache(filp, dirent, filldir, p->name, p->len,
				proc_base_instantiate, task, p);
}

#ifdef CONFIG_TASK_IO_ACCOUNTING
static int do_io_accounting(struct task_struct *task, char *buffer, int whole)
{
	struct task_io_accounting acct = task->ioac;
	unsigned long flags;
	int result;

	result = mutex_lock_killable(&task->signal->cred_guard_mutex);
	if (result)
		return result;

	if (!ptrace_may_access(task, PTRACE_MODE_READ)) {
		result = -EACCES;
		goto out_unlock;
	}

	if (whole && lock_task_sighand(task, &flags)) {
		struct task_struct *t = task;

		task_io_accounting_add(&acct, &task->signal->ioac);
		while_each_thread(task, t)
			task_io_accounting_add(&acct, &t->ioac);

		unlock_task_sighand(task, &flags);
	}
	result = sprintf(buffer,
			"rchar: %llu\n"
			"wchar: %llu\n"
			"syscr: %llu\n"
			"syscw: %llu\n"
			"read_bytes: %llu\n"
			"write_bytes: %llu\n"
			"cancelled_write_bytes: %llu\n",
			(unsigned long long)acct.rchar,
			(unsigned long long)acct.wchar,
			(unsigned long long)acct.syscr,
			(unsigned long long)acct.syscw,
			(unsigned long long)acct.read_bytes,
			(unsigned long long)acct.write_bytes,
			(unsigned long long)acct.cancelled_write_bytes);
out_unlock:
	mutex_unlock(&task->signal->cred_guard_mutex);
	return result;
}

static int proc_tid_io_accounting(struct task_struct *task, char *buffer)
{
	return do_io_accounting(task, buffer, 0);
}

static int proc_tgid_io_accounting(struct task_struct *task, char *buffer)
{
	return do_io_accounting(task, buffer, 1);
}
#endif 

static int proc_pid_personality(struct seq_file *m, struct pid_namespace *ns,
				struct pid *pid, struct task_struct *task)
{
	int err = lock_trace(task);
	if (!err) {
		seq_printf(m, "%08x\n", task->personality);
		unlock_trace(task);
	}
	return err;
}

static const struct file_operations proc_task_operations;
static const struct inode_operations proc_task_inode_operations;

static const struct pid_entry tgid_base_stuff[] = {
	DIR("task",       S_IRUGO|S_IXUGO, proc_task_inode_operations, proc_task_operations),
	DIR("fd",         S_IRUSR|S_IXUSR, proc_fd_inode_operations, proc_fd_operations),
#ifdef CONFIG_CHECKPOINT_RESTORE
	DIR("map_files",  S_IRUSR|S_IXUSR, proc_map_files_inode_operations, proc_map_files_operations),
#endif
	DIR("fdinfo",     S_IRUSR|S_IXUSR, proc_fdinfo_inode_operations, proc_fdinfo_operations),
	DIR("ns",	  S_IRUSR|S_IXUGO, proc_ns_dir_inode_operations, proc_ns_dir_operations),
#ifdef CONFIG_NET
	DIR("net",        S_IRUGO|S_IXUGO, proc_net_inode_operations, proc_net_operations),
#endif
	REG("environ",    S_IRUSR, proc_environ_operations),
	INF("auxv",       S_IRUSR, proc_pid_auxv),
	ONE("status",     S_IRUGO, proc_pid_status),
	ONE("personality", S_IRUGO, proc_pid_personality),
	INF("limits",	  S_IRUGO, proc_pid_limits),
#ifdef CONFIG_SCHED_DEBUG
	REG("sched",      S_IRUGO|S_IWUSR, proc_pid_sched_operations),
#endif
#ifdef CONFIG_SCHED_AUTOGROUP
	REG("autogroup",  S_IRUGO|S_IWUSR, proc_pid_sched_autogroup_operations),
#endif
	REG("comm",      S_IRUGO|S_IWUSR, proc_pid_set_comm_operations),
#ifdef CONFIG_HAVE_ARCH_TRACEHOOK
	INF("syscall",    S_IRUGO, proc_pid_syscall),
#endif
	INF("cmdline",    S_IRUGO, proc_pid_cmdline),
	ONE("stat",       S_IRUGO, proc_tgid_stat),
	ONE("statm",      S_IRUGO, proc_pid_statm),
	REG("maps",       S_IRUGO, proc_pid_maps_operations),
#ifdef CONFIG_NUMA
	REG("numa_maps",  S_IRUGO, proc_pid_numa_maps_operations),
#endif
	REG("mem",        S_IRUSR|S_IWUSR, proc_mem_operations),
	LNK("cwd",        proc_cwd_link),
	LNK("root",       proc_root_link),
	LNK("exe",        proc_exe_link),
	REG("mounts",     S_IRUGO, proc_mounts_operations),
	REG("mountinfo",  S_IRUGO, proc_mountinfo_operations),
	REG("mountstats", S_IRUSR, proc_mountstats_operations),
#ifdef CONFIG_PROC_PAGE_MONITOR
	REG("clear_refs", S_IWUSR, proc_clear_refs_operations),
	REG("smaps",      S_IRUGO, proc_pid_smaps_operations),
	REG("pagemap",    S_IRUGO, proc_pagemap_operations),
#endif
#ifdef CONFIG_SECURITY
	DIR("attr",       S_IRUGO|S_IXUGO, proc_attr_dir_inode_operations, proc_attr_dir_operations),
#endif
#ifdef CONFIG_KALLSYMS
	INF("wchan",      S_IRUGO, proc_pid_wchan),
#endif
#ifdef CONFIG_STACKTRACE
	ONE("stack",      S_IRUGO, proc_pid_stack),
#endif
#ifdef CONFIG_SCHEDSTATS
	INF("schedstat",  S_IRUGO, proc_pid_schedstat),
#endif
#ifdef CONFIG_LATENCYTOP
	REG("latency",  S_IRUGO, proc_lstats_operations),
#endif
#ifdef CONFIG_PROC_PID_CPUSET
	REG("cpuset",     S_IRUGO, proc_cpuset_operations),
#endif
#ifdef CONFIG_CGROUPS
	REG("cgroup",  S_IRUGO, proc_cgroup_operations),
#endif
	INF("oom_score",  S_IRUGO, proc_oom_score),
	ANDROID("oom_adj",S_IRUGO|S_IWUSR, oom_adjust),
	REG("oom_score_adj", S_IRUGO|S_IWUSR, proc_oom_score_adj_operations),
#ifdef CONFIG_AUDITSYSCALL
	REG("loginuid",   S_IWUSR|S_IRUGO, proc_loginuid_operations),
	REG("sessionid",  S_IRUGO, proc_sessionid_operations),
#endif
#ifdef CONFIG_FAULT_INJECTION
	REG("make-it-fail", S_IRUGO|S_IWUSR, proc_fault_inject_operations),
#endif
#ifdef CONFIG_ELF_CORE
	REG("coredump_filter", S_IRUGO|S_IWUSR, proc_coredump_filter_operations),
#endif
#ifdef CONFIG_TASK_IO_ACCOUNTING
	INF("io",	S_IRUSR, proc_tgid_io_accounting),
#endif
#ifdef CONFIG_HARDWALL
	INF("hardwall",   S_IRUGO, proc_pid_hardwall),
#endif
};

static int proc_tgid_base_readdir(struct file * filp,
			     void * dirent, filldir_t filldir)
{
	return proc_pident_readdir(filp,dirent,filldir,
				   tgid_base_stuff,ARRAY_SIZE(tgid_base_stuff));
}

static const struct file_operations proc_tgid_base_operations = {
	.read		= generic_read_dir,
	.readdir	= proc_tgid_base_readdir,
	.llseek		= default_llseek,
};

static struct dentry *proc_tgid_base_lookup(struct inode *dir, struct dentry *dentry, struct nameidata *nd){
	return proc_pident_lookup(dir, dentry,
				  tgid_base_stuff, ARRAY_SIZE(tgid_base_stuff));
}

static const struct inode_operations proc_tgid_base_inode_operations = {
	.lookup		= proc_tgid_base_lookup,
	.getattr	= pid_getattr,
	.setattr	= proc_setattr,
	.permission	= proc_pid_permission,
};

static void proc_flush_task_mnt(struct vfsmount *mnt, pid_t pid, pid_t tgid)
{
	struct dentry *dentry, *leader, *dir;
	char buf[PROC_NUMBUF];
	struct qstr name;

	name.name = buf;
	name.len = snprintf(buf, sizeof(buf), "%d", pid);
	dentry = d_hash_and_lookup(mnt->mnt_root, &name);
	if (dentry) {
		shrink_dcache_parent(dentry);
		d_drop(dentry);
		dput(dentry);
	}

	name.name = buf;
	name.len = snprintf(buf, sizeof(buf), "%d", tgid);
	leader = d_hash_and_lookup(mnt->mnt_root, &name);
	if (!leader)
		goto out;

	name.name = "task";
	name.len = strlen(name.name);
	dir = d_hash_and_lookup(leader, &name);
	if (!dir)
		goto out_put_leader;

	name.name = buf;
	name.len = snprintf(buf, sizeof(buf), "%d", pid);
	dentry = d_hash_and_lookup(dir, &name);
	if (dentry) {
		shrink_dcache_parent(dentry);
		d_drop(dentry);
		dput(dentry);
	}

	dput(dir);
out_put_leader:
	dput(leader);
out:
	return;
}


void proc_flush_task(struct task_struct *task)
{
	int i;
	struct pid *pid, *tgid;
	struct upid *upid;

	pid = task_pid(task);
	tgid = task_tgid(task);

	for (i = 0; i <= pid->level; i++) {
		upid = &pid->numbers[i];
		proc_flush_task_mnt(upid->ns->proc_mnt, upid->nr,
					tgid->numbers[i].nr);
	}

	upid = &pid->numbers[pid->level];
	if (upid->nr == 1)
		pid_ns_release_proc(upid->ns);
}

static struct dentry *proc_pid_instantiate(struct inode *dir,
					   struct dentry * dentry,
					   struct task_struct *task, const void *ptr)
{
	struct dentry *error = ERR_PTR(-ENOENT);
	struct inode *inode;

	inode = proc_pid_make_inode(dir->i_sb, task);
	if (!inode)
		goto out;

	inode->i_mode = S_IFDIR|S_IRUGO|S_IXUGO;
	inode->i_op = &proc_tgid_base_inode_operations;
	inode->i_fop = &proc_tgid_base_operations;
	inode->i_flags|=S_IMMUTABLE;

	set_nlink(inode, 2 + pid_entry_count_dirs(tgid_base_stuff,
						  ARRAY_SIZE(tgid_base_stuff)));

	d_set_d_op(dentry, &pid_dentry_operations);

	d_add(dentry, inode);
	
	if (pid_revalidate(dentry, NULL))
		error = NULL;
out:
	return error;
}

struct dentry *proc_pid_lookup(struct inode *dir, struct dentry * dentry, struct nameidata *nd)
{
	struct dentry *result;
	struct task_struct *task;
	unsigned tgid;
	struct pid_namespace *ns;

	result = proc_base_lookup(dir, dentry);
	if (!IS_ERR(result) || PTR_ERR(result) != -ENOENT)
		goto out;

	tgid = name_to_int(dentry);
	if (tgid == ~0U)
		goto out;

	ns = dentry->d_sb->s_fs_info;
	rcu_read_lock();
	task = find_task_by_pid_ns(tgid, ns);
	if (task)
		get_task_struct(task);
	rcu_read_unlock();
	if (!task)
		goto out;

	result = proc_pid_instantiate(dir, dentry, task, NULL);
	put_task_struct(task);
out:
	return result;
}

struct tgid_iter {
	unsigned int tgid;
	struct task_struct *task;
};
static struct tgid_iter next_tgid(struct pid_namespace *ns, struct tgid_iter iter)
{
	struct pid *pid;

	if (iter.task)
		put_task_struct(iter.task);
	rcu_read_lock();
retry:
	iter.task = NULL;
	pid = find_ge_pid(iter.tgid, ns);
	if (pid) {
		iter.tgid = pid_nr_ns(pid, ns);
		iter.task = pid_task(pid, PIDTYPE_PID);
		if (!iter.task || !has_group_leader_pid(iter.task)) {
			iter.tgid += 1;
			goto retry;
		}
		get_task_struct(iter.task);
	}
	rcu_read_unlock();
	return iter;
}

#define TGID_OFFSET (FIRST_PROCESS_ENTRY + ARRAY_SIZE(proc_base_stuff))

static int proc_pid_fill_cache(struct file *filp, void *dirent, filldir_t filldir,
	struct tgid_iter iter)
{
	char name[PROC_NUMBUF];
	int len = snprintf(name, sizeof(name), "%d", iter.tgid);
	return proc_fill_cache(filp, dirent, filldir, name, len,
				proc_pid_instantiate, iter.task, NULL);
}

static int fake_filldir(void *buf, const char *name, int namelen,
			loff_t offset, u64 ino, unsigned d_type)
{
	return 0;
}

int proc_pid_readdir(struct file * filp, void * dirent, filldir_t filldir)
{
	unsigned int nr;
	struct task_struct *reaper;
	struct tgid_iter iter;
	struct pid_namespace *ns;
	filldir_t __filldir;

	if (filp->f_pos >= PID_MAX_LIMIT + TGID_OFFSET)
		goto out_no_task;
	nr = filp->f_pos - FIRST_PROCESS_ENTRY;

	reaper = get_proc_task(filp->f_path.dentry->d_inode);
	if (!reaper)
		goto out_no_task;

	for (; nr < ARRAY_SIZE(proc_base_stuff); filp->f_pos++, nr++) {
		const struct pid_entry *p = &proc_base_stuff[nr];
		if (proc_base_fill_cache(filp, dirent, filldir, reaper, p) < 0)
			goto out;
	}

	ns = filp->f_dentry->d_sb->s_fs_info;
	iter.task = NULL;
	iter.tgid = filp->f_pos - TGID_OFFSET;
	for (iter = next_tgid(ns, iter);
	     iter.task;
	     iter.tgid += 1, iter = next_tgid(ns, iter)) {
		if (has_pid_permissions(ns, iter.task, 2))
			__filldir = filldir;
		else
			__filldir = fake_filldir;

		filp->f_pos = iter.tgid + TGID_OFFSET;
		if (proc_pid_fill_cache(filp, dirent, __filldir, iter) < 0) {
			put_task_struct(iter.task);
			goto out;
		}
	}
	filp->f_pos = PID_MAX_LIMIT + TGID_OFFSET;
out:
	put_task_struct(reaper);
out_no_task:
	return 0;
}

static const struct pid_entry tid_base_stuff[] = {
	DIR("fd",        S_IRUSR|S_IXUSR, proc_fd_inode_operations, proc_fd_operations),
	DIR("fdinfo",    S_IRUSR|S_IXUSR, proc_fdinfo_inode_operations, proc_fdinfo_operations),
	DIR("ns",	 S_IRUSR|S_IXUGO, proc_ns_dir_inode_operations, proc_ns_dir_operations),
	REG("environ",   S_IRUSR, proc_environ_operations),
	INF("auxv",      S_IRUSR, proc_pid_auxv),
	ONE("status",    S_IRUGO, proc_pid_status),
	ONE("personality", S_IRUGO, proc_pid_personality),
	INF("limits",	 S_IRUGO, proc_pid_limits),
#ifdef CONFIG_SCHED_DEBUG
	REG("sched",     S_IRUGO|S_IWUSR, proc_pid_sched_operations),
#endif
	REG("comm",      S_IRUGO|S_IWUSR, proc_pid_set_comm_operations),
#ifdef CONFIG_HAVE_ARCH_TRACEHOOK
	INF("syscall",   S_IRUGO, proc_pid_syscall),
#endif
	INF("cmdline",   S_IRUGO, proc_pid_cmdline),
	ONE("stat",      S_IRUGO, proc_tid_stat),
	ONE("statm",     S_IRUGO, proc_pid_statm),
	REG("maps",      S_IRUGO, proc_tid_maps_operations),
#ifdef CONFIG_NUMA
	REG("numa_maps", S_IRUGO, proc_tid_numa_maps_operations),
#endif
	REG("mem",       S_IRUSR|S_IWUSR, proc_mem_operations),
	LNK("cwd",       proc_cwd_link),
	LNK("root",      proc_root_link),
	LNK("exe",       proc_exe_link),
	REG("mounts",    S_IRUGO, proc_mounts_operations),
	REG("mountinfo",  S_IRUGO, proc_mountinfo_operations),
#ifdef CONFIG_PROC_PAGE_MONITOR
	REG("clear_refs", S_IWUSR, proc_clear_refs_operations),
	REG("smaps",     S_IRUGO, proc_tid_smaps_operations),
	REG("pagemap",    S_IRUGO, proc_pagemap_operations),
#endif
#ifdef CONFIG_SECURITY
	DIR("attr",      S_IRUGO|S_IXUGO, proc_attr_dir_inode_operations, proc_attr_dir_operations),
#endif
#ifdef CONFIG_KALLSYMS
	INF("wchan",     S_IRUGO, proc_pid_wchan),
#endif
#ifdef CONFIG_STACKTRACE
	ONE("stack",      S_IRUGO, proc_pid_stack),
#endif
#ifdef CONFIG_SCHEDSTATS
	INF("schedstat", S_IRUGO, proc_pid_schedstat),
#endif
#ifdef CONFIG_LATENCYTOP
	REG("latency",  S_IRUGO, proc_lstats_operations),
#endif
#ifdef CONFIG_PROC_PID_CPUSET
	REG("cpuset",    S_IRUGO, proc_cpuset_operations),
#endif
#ifdef CONFIG_CGROUPS
	REG("cgroup",  S_IRUGO, proc_cgroup_operations),
#endif
	INF("oom_score", S_IRUGO, proc_oom_score),
	REG("oom_adj",   S_IRUGO|S_IWUSR, proc_oom_adjust_operations),
	REG("oom_score_adj", S_IRUGO|S_IWUSR, proc_oom_score_adj_operations),
#ifdef CONFIG_AUDITSYSCALL
	REG("loginuid",  S_IWUSR|S_IRUGO, proc_loginuid_operations),
	REG("sessionid",  S_IRUGO, proc_sessionid_operations),
#endif
#ifdef CONFIG_FAULT_INJECTION
	REG("make-it-fail", S_IRUGO|S_IWUSR, proc_fault_inject_operations),
#endif
#ifdef CONFIG_TASK_IO_ACCOUNTING
	INF("io",	S_IRUSR, proc_tid_io_accounting),
#endif
#ifdef CONFIG_HARDWALL
	INF("hardwall",   S_IRUGO, proc_pid_hardwall),
#endif
};

static int proc_tid_base_readdir(struct file * filp,
			     void * dirent, filldir_t filldir)
{
	return proc_pident_readdir(filp,dirent,filldir,
				   tid_base_stuff,ARRAY_SIZE(tid_base_stuff));
}

static struct dentry *proc_tid_base_lookup(struct inode *dir, struct dentry *dentry, struct nameidata *nd){
	return proc_pident_lookup(dir, dentry,
				  tid_base_stuff, ARRAY_SIZE(tid_base_stuff));
}

static const struct file_operations proc_tid_base_operations = {
	.read		= generic_read_dir,
	.readdir	= proc_tid_base_readdir,
	.llseek		= default_llseek,
};

static const struct inode_operations proc_tid_base_inode_operations = {
	.lookup		= proc_tid_base_lookup,
	.getattr	= pid_getattr,
	.setattr	= proc_setattr,
};

static struct dentry *proc_task_instantiate(struct inode *dir,
	struct dentry *dentry, struct task_struct *task, const void *ptr)
{
	struct dentry *error = ERR_PTR(-ENOENT);
	struct inode *inode;
	inode = proc_pid_make_inode(dir->i_sb, task);

	if (!inode)
		goto out;
	inode->i_mode = S_IFDIR|S_IRUGO|S_IXUGO;
	inode->i_op = &proc_tid_base_inode_operations;
	inode->i_fop = &proc_tid_base_operations;
	inode->i_flags|=S_IMMUTABLE;

	set_nlink(inode, 2 + pid_entry_count_dirs(tid_base_stuff,
						  ARRAY_SIZE(tid_base_stuff)));

	d_set_d_op(dentry, &pid_dentry_operations);

	d_add(dentry, inode);
	
	if (pid_revalidate(dentry, NULL))
		error = NULL;
out:
	return error;
}

static struct dentry *proc_task_lookup(struct inode *dir, struct dentry * dentry, struct nameidata *nd)
{
	struct dentry *result = ERR_PTR(-ENOENT);
	struct task_struct *task;
	struct task_struct *leader = get_proc_task(dir);
	unsigned tid;
	struct pid_namespace *ns;

	if (!leader)
		goto out_no_task;

	tid = name_to_int(dentry);
	if (tid == ~0U)
		goto out;

	ns = dentry->d_sb->s_fs_info;
	rcu_read_lock();
	task = find_task_by_pid_ns(tid, ns);
	if (task)
		get_task_struct(task);
	rcu_read_unlock();
	if (!task)
		goto out;
	if (!same_thread_group(leader, task))
		goto out_drop_task;

	result = proc_task_instantiate(dir, dentry, task, NULL);
out_drop_task:
	put_task_struct(task);
out:
	put_task_struct(leader);
out_no_task:
	return result;
}

static struct task_struct *first_tid(struct task_struct *leader,
		int tid, int nr, struct pid_namespace *ns)
{
	struct task_struct *pos;

	rcu_read_lock();
	
	if (tid && (nr > 0)) {
		pos = find_task_by_pid_ns(tid, ns);
		if (pos && (pos->group_leader == leader))
			goto found;
	}

	
	pos = NULL;
	if (nr && nr >= get_nr_threads(leader))
		goto out;

	for (pos = leader; nr > 0; --nr) {
		pos = next_thread(pos);
		if (pos == leader) {
			pos = NULL;
			goto out;
		}
	}
found:
	get_task_struct(pos);
out:
	rcu_read_unlock();
	return pos;
}

static struct task_struct *next_tid(struct task_struct *start)
{
	struct task_struct *pos = NULL;
	rcu_read_lock();
	if (pid_alive(start)) {
		pos = next_thread(start);
		if (thread_group_leader(pos))
			pos = NULL;
		else
			get_task_struct(pos);
	}
	rcu_read_unlock();
	put_task_struct(start);
	return pos;
}

static int proc_task_fill_cache(struct file *filp, void *dirent, filldir_t filldir,
	struct task_struct *task, int tid)
{
	char name[PROC_NUMBUF];
	int len = snprintf(name, sizeof(name), "%d", tid);
	return proc_fill_cache(filp, dirent, filldir, name, len,
				proc_task_instantiate, task, NULL);
}

static int proc_task_readdir(struct file * filp, void * dirent, filldir_t filldir)
{
	struct dentry *dentry = filp->f_path.dentry;
	struct inode *inode = dentry->d_inode;
	struct task_struct *leader = NULL;
	struct task_struct *task;
	int retval = -ENOENT;
	ino_t ino;
	int tid;
	struct pid_namespace *ns;

	task = get_proc_task(inode);
	if (!task)
		goto out_no_task;
	rcu_read_lock();
	if (pid_alive(task)) {
		leader = task->group_leader;
		get_task_struct(leader);
	}
	rcu_read_unlock();
	put_task_struct(task);
	if (!leader)
		goto out_no_task;
	retval = 0;

	switch ((unsigned long)filp->f_pos) {
	case 0:
		ino = inode->i_ino;
		if (filldir(dirent, ".", 1, filp->f_pos, ino, DT_DIR) < 0)
			goto out;
		filp->f_pos++;
		
	case 1:
		ino = parent_ino(dentry);
		if (filldir(dirent, "..", 2, filp->f_pos, ino, DT_DIR) < 0)
			goto out;
		filp->f_pos++;
		
	}

	ns = filp->f_dentry->d_sb->s_fs_info;
	tid = (int)filp->f_version;
	filp->f_version = 0;
	for (task = first_tid(leader, tid, filp->f_pos - 2, ns);
	     task;
	     task = next_tid(task), filp->f_pos++) {
		tid = task_pid_nr_ns(task, ns);
		if (proc_task_fill_cache(filp, dirent, filldir, task, tid) < 0) {
			filp->f_version = (u64)tid;
			put_task_struct(task);
			break;
		}
	}
out:
	put_task_struct(leader);
out_no_task:
	return retval;
}

static int proc_task_getattr(struct vfsmount *mnt, struct dentry *dentry, struct kstat *stat)
{
	struct inode *inode = dentry->d_inode;
	struct task_struct *p = get_proc_task(inode);
	generic_fillattr(inode, stat);

	if (p) {
		stat->nlink += get_nr_threads(p);
		put_task_struct(p);
	}

	return 0;
}

static const struct inode_operations proc_task_inode_operations = {
	.lookup		= proc_task_lookup,
	.getattr	= proc_task_getattr,
	.setattr	= proc_setattr,
	.permission	= proc_pid_permission,
};

static const struct file_operations proc_task_operations = {
	.read		= generic_read_dir,
	.readdir	= proc_task_readdir,
	.llseek		= default_llseek,
};
