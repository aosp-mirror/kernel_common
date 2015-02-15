/*
 *  linux/fs/file.c
 *
 *  Copyright (C) 1998-1999, Stephen Tweedie and Bill Hawes
 *
 *  Manage the dynamic fd arrays in the process files_struct.
 */

#include <linux/export.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/file.h>
#include <linux/fdtable.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/rcupdate.h>
#include <linux/workqueue.h>
#include <linux/crc32c.h>
#include <linux/rtc.h>

struct fdtable_defer {
	spinlock_t lock;
	struct work_struct wq;
	struct fdtable *next;
};

int sysctl_nr_open __read_mostly = 1024*1024;
int sysctl_nr_open_min = BITS_PER_LONG;
int sysctl_nr_open_max = 1024 * 1024; 

static DEFINE_PER_CPU(struct fdtable_defer, fdtable_defer_list);

static void *alloc_fdmem(size_t size)
{
	if (size <= (PAGE_SIZE << PAGE_ALLOC_COSTLY_ORDER)) {
		void *data = kmalloc(size, GFP_KERNEL|__GFP_NOWARN);
		if (data != NULL)
			return data;
	}
	return vmalloc(size);
}

static void free_fdmem(void *ptr)
{
	is_vmalloc_addr(ptr) ? vfree(ptr) : kfree(ptr);
}

static void __free_fdtable(struct fdtable *fdt)
{
	free_fdmem(fdt->fd);
	free_fdmem(fdt->open_fds);
	free_fdmem(fdt->user);
	kfree(fdt);
}

static void free_fdtable_work(struct work_struct *work)
{
	struct fdtable_defer *f =
		container_of(work, struct fdtable_defer, wq);
	struct fdtable *fdt;

	spin_lock_bh(&f->lock);
	fdt = f->next;
	f->next = NULL;
	spin_unlock_bh(&f->lock);
	while(fdt) {
		struct fdtable *next = fdt->next;

		__free_fdtable(fdt);
		fdt = next;
	}
}

void free_fdtable_rcu(struct rcu_head *rcu)
{
	struct fdtable *fdt = container_of(rcu, struct fdtable, rcu);
	struct fdtable_defer *fddef;

	BUG_ON(!fdt);

	if (fdt->max_fds <= NR_OPEN_DEFAULT) {
		kmem_cache_free(files_cachep,
				container_of(fdt, struct files_struct, fdtab));
		return;
	}
	if (!is_vmalloc_addr(fdt->fd) && !is_vmalloc_addr(fdt->open_fds)
		    && !is_vmalloc_addr(fdt->user)) {
		kfree(fdt->fd);
		kfree(fdt->open_fds);
		kfree(fdt->user);
		kfree(fdt);
	} else {
		fddef = &get_cpu_var(fdtable_defer_list);
		spin_lock(&fddef->lock);
		fdt->next = fddef->next;
		fddef->next = fdt;
		
		schedule_work(&fddef->wq);
		spin_unlock(&fddef->lock);
		put_cpu_var(fdtable_defer_list);
	}
}

static void copy_fdtable(struct fdtable *nfdt, struct fdtable *ofdt)
{
	unsigned int cpy, set;

	BUG_ON(nfdt->max_fds < ofdt->max_fds);

	cpy = ofdt->max_fds * sizeof(struct file *);
	set = (nfdt->max_fds - ofdt->max_fds) * sizeof(struct file *);
	memcpy(nfdt->fd, ofdt->fd, cpy);
	memset((char *)(nfdt->fd) + cpy, 0, set);

	cpy = ofdt->max_fds / BITS_PER_BYTE;
	set = (nfdt->max_fds - ofdt->max_fds) / BITS_PER_BYTE;
	memcpy(nfdt->open_fds, ofdt->open_fds, cpy);
	memset((char *)(nfdt->open_fds) + cpy, 0, set);
	memcpy(nfdt->close_on_exec, ofdt->close_on_exec, cpy);
	memset((char *)(nfdt->close_on_exec) + cpy, 0, set);
	memcpy(nfdt->user, ofdt->user,
		ofdt->max_fds * sizeof(*nfdt->user));
	memset(nfdt->user + ofdt->max_fds, 0,
		(nfdt->max_fds - ofdt->max_fds) * sizeof(*nfdt->user));
}

static struct fdtable * alloc_fdtable(unsigned int nr)
{
	struct fdtable *fdt;
	void *data;

	nr /= (1024 / sizeof(struct file *));
	nr = roundup_pow_of_two(nr + 1);
	nr *= (1024 / sizeof(struct file *));
	if (unlikely(nr > sysctl_nr_open))
		nr = ((sysctl_nr_open - 1) | (BITS_PER_LONG - 1)) + 1;

	fdt = kmalloc(sizeof(struct fdtable), GFP_KERNEL);
	if (!fdt)
		goto out;
	fdt->max_fds = nr;
	data = alloc_fdmem(nr * sizeof(struct file *));
	if (!data)
		goto out_fdt;
	fdt->fd = data;

	data = alloc_fdmem(max_t(size_t,
				 2 * nr / BITS_PER_BYTE, L1_CACHE_BYTES));
	if (!data)
		goto out_arr;
	fdt->open_fds = data;
	data += nr / BITS_PER_BYTE;
	fdt->close_on_exec = data;
	fdt->next = NULL;
	data = alloc_fdmem(sizeof(*fdt->user) * nr);
	if (!data)
		goto out_open;
	fdt->user = (struct fdt_user*) data;

	return fdt;

out_open:
	free_fdmem(fdt->open_fds);
out_arr:
	free_fdmem(fdt->fd);
out_fdt:
	kfree(fdt);
out:
	return NULL;
}

static int expand_fdtable(struct files_struct *files, int nr)
	__releases(files->file_lock)
	__acquires(files->file_lock)
{
	struct fdtable *new_fdt, *cur_fdt;

	spin_unlock(&files->file_lock);
	new_fdt = alloc_fdtable(nr);
	spin_lock(&files->file_lock);
	if (!new_fdt)
		return -ENOMEM;
	if (unlikely(new_fdt->max_fds <= nr)) {
		__free_fdtable(new_fdt);
		return -EMFILE;
	}
	cur_fdt = files_fdtable(files);
	if (nr >= cur_fdt->max_fds) {
		
		copy_fdtable(new_fdt, cur_fdt);
		rcu_assign_pointer(files->fdt, new_fdt);
		if (cur_fdt->max_fds > NR_OPEN_DEFAULT)
			free_fdtable(cur_fdt);
	} else {
		
		__free_fdtable(new_fdt);
	}
	return 1;
}

int expand_files(struct files_struct *files, int nr)
{
	struct fdtable *fdt;

	fdt = files_fdtable(files);

	if (nr >= rlimit(RLIMIT_NOFILE))
		return -EMFILE;

	
	if (nr < fdt->max_fds)
		return 0;

	
	if (nr >= sysctl_nr_open)
		return -EMFILE;

	
	return expand_fdtable(files, nr);
}

static int count_open_files(struct fdtable *fdt)
{
	int size = fdt->max_fds;
	int i;

	
	for (i = size / BITS_PER_LONG; i > 0; ) {
		if (fdt->open_fds[--i])
			break;
	}
	i = (i + 1) * BITS_PER_LONG;
	return i;
}

struct files_struct *dup_fd(struct files_struct *oldf, int *errorp)
{
	struct files_struct *newf;
	struct file **old_fds, **new_fds;
	int open_files, size, i;
	struct fdtable *old_fdt, *new_fdt;

	*errorp = -ENOMEM;
	newf = kmem_cache_alloc(files_cachep, GFP_KERNEL);
	if (!newf)
		goto out;

	atomic_set(&newf->count, 1);

	spin_lock_init(&newf->file_lock);
	newf->next_fd = 0;
	new_fdt = &newf->fdtab;
	new_fdt->max_fds = NR_OPEN_DEFAULT;
	new_fdt->close_on_exec = newf->close_on_exec_init;
	new_fdt->open_fds = newf->open_fds_init;
	new_fdt->fd = &newf->fd_array[0];
	new_fdt->next = NULL;
	new_fdt->user = &newf->user_array[0];

	spin_lock(&oldf->file_lock);
	old_fdt = files_fdtable(oldf);
	open_files = count_open_files(old_fdt);

	while (unlikely(open_files > new_fdt->max_fds)) {
		spin_unlock(&oldf->file_lock);

		if (new_fdt != &newf->fdtab)
			__free_fdtable(new_fdt);

		new_fdt = alloc_fdtable(open_files - 1);
		if (!new_fdt) {
			*errorp = -ENOMEM;
			goto out_release;
		}

		
		if (unlikely(new_fdt->max_fds < open_files)) {
			__free_fdtable(new_fdt);
			*errorp = -EMFILE;
			goto out_release;
		}

		spin_lock(&oldf->file_lock);
		old_fdt = files_fdtable(oldf);
		open_files = count_open_files(old_fdt);
	}

	old_fds = old_fdt->fd;
	new_fds = new_fdt->fd;

	memcpy(new_fdt->open_fds, old_fdt->open_fds, open_files / 8);
	memcpy(new_fdt->close_on_exec, old_fdt->close_on_exec, open_files / 8);
	memset(new_fdt->user, 0,
		open_files * sizeof(*old_fdt->user));

	for (i = open_files; i != 0; i--) {
		struct file *f = *old_fds++;
		if (f) {
			get_file(f);
		} else {
			__clear_open_fd(open_files - i, new_fdt);
		}
		rcu_assign_pointer(*new_fds++, f);
	}
	spin_unlock(&oldf->file_lock);

	
	size = (new_fdt->max_fds - open_files) * sizeof(struct file *);

	
	memset(new_fds, 0, size);

	if (new_fdt->max_fds > open_files) {
		int left = (new_fdt->max_fds - open_files) / 8;
		int start = open_files / BITS_PER_LONG;

		memset(&new_fdt->open_fds[start], 0, left);
		memset(&new_fdt->close_on_exec[start], 0, left);
		memset(&new_fdt->user[open_files], 0,
			(new_fdt->max_fds - open_files) * sizeof(*new_fdt->user));
	}

	rcu_assign_pointer(newf->fdt, new_fdt);

	return newf;

out_release:
	kmem_cache_free(files_cachep, newf);
out:
	return NULL;
}

static void __devinit fdtable_defer_list_init(int cpu)
{
	struct fdtable_defer *fddef = &per_cpu(fdtable_defer_list, cpu);
	spin_lock_init(&fddef->lock);
	INIT_WORK(&fddef->wq, free_fdtable_work);
	fddef->next = NULL;
}

void __init files_defer_init(void)
{
	int i;
	for_each_possible_cpu(i)
		fdtable_defer_list_init(i);
	sysctl_nr_open_max = min((size_t)INT_MAX, ~(size_t)0/sizeof(void *)) &
			     -BITS_PER_LONG;
}

struct files_struct init_files = {
	.count		= ATOMIC_INIT(1),
	.fdt		= &init_files.fdtab,
	.fdtab		= {
		.max_fds	= NR_OPEN_DEFAULT,
		.fd		= &init_files.fd_array[0],
		.close_on_exec	= init_files.close_on_exec_init,
		.open_fds	= init_files.open_fds_init,
		.user		= &init_files.user_array[0],
	},
	.file_lock	= __SPIN_LOCK_UNLOCKED(init_task.file_lock),
};

static void fdtable_usage_dump(struct fdtable *fdt)
{
	int i;
	char* buf;
	struct timespec ts;
	struct rtc_time tm;

	buf = (char*) kmalloc(PATH_MAX, GFP_ATOMIC);
	if (!buf) {
		pr_err("%s: fail to alloc buffer\n", __func__);
		return;
	}

	rcu_read_lock();
	for (i = 0; i < fdt->max_fds; i++) {
		struct file* file;
		struct task_struct* user = NULL;
		int pid;
		char* path;

		file = fdt->fd[i];
		if (!file)
			continue;

		pid = fdt->user[i].installer;
		ts = fdt->user[i].open_time;
		rtc_time_to_tm(ts.tv_sec - (sys_tz.tz_minuteswest * 60), &tm);

		user = find_task_by_vpid(pid);
		if (user)
			get_task_struct(user);

		path = d_path(&file->f_path, buf, PATH_MAX);

		if (IS_ERR(path))
			path = "<unknown>";
		else {
			char* spath = strstr(path, ":[");
			if (spath) spath[0] = '\0';
		}

		pr_warn("%d->fd[%d] file: %s, user: %d (%s %d:%d), (%02d-%02d %02d:%02d:%02d)\n",
				current->tgid, i, path, pid,
				user ? user->comm : "<unknown>",
				user ? user->tgid : -1,
				user ? user->pid  : -1,
				tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

		if (user)
			put_task_struct(user);
	}
	rcu_read_unlock();
	kfree(buf);
}

int alloc_fd(unsigned start, unsigned flags)
{
	struct files_struct *files = current->files;
	unsigned int fd;
	int error;
	struct fdtable *fdt;

	spin_lock(&files->file_lock);
repeat:
	fdt = files_fdtable(files);
	fd = start;
	if (fd < files->next_fd)
		fd = files->next_fd;

	if (fd < fdt->max_fds)
		fd = find_next_zero_bit(fdt->open_fds, fdt->max_fds, fd);

	error = expand_files(files, fd);
	if (error < 0)
		goto out;

	if (error)
		goto repeat;

	if (start <= files->next_fd)
		files->next_fd = fd + 1;

	__set_open_fd(fd, fdt);
	if (flags & O_CLOEXEC)
		__set_close_on_exec(fd, fdt);
	else
		__clear_close_on_exec(fd, fdt);
	memset(&fdt->user[fd], 0, sizeof(*fdt->user));
	error = fd;
#if 1
	
	if (rcu_dereference_raw(fdt->fd[fd]) != NULL) {
		printk(KERN_WARNING "alloc_fd: slot %d not NULL!\n", fd);
		rcu_assign_pointer(fdt->fd[fd], NULL);
	}
#endif

out:
	if (unlikely(error == -EMFILE)) {
		static unsigned long debugging_ratelimit = 0;
		const unsigned long debugging_delay_ms = 30000;

		if (jiffies > debugging_ratelimit) {
			debugging_ratelimit = jiffies + msecs_to_jiffies(debugging_delay_ms);

			pr_warn("[%s] Too many open files (%d/%u), dump all fdt users:\n",
					__func__, count_open_files(fdt), fdt->max_fds);
			dump_stack();
			fdtable_usage_dump(fdt);
			pr_warn("[%s] end of dump\n", __func__);
		}
	}
	spin_unlock(&files->file_lock);
	return error;
}
EXPORT_SYMBOL(alloc_fd);

int get_unused_fd(void)
{
	return alloc_fd(0, 0);
}
EXPORT_SYMBOL(get_unused_fd);
