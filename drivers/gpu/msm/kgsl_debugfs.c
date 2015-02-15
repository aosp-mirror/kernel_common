/* Copyright (c) 2002,2008-2014, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/debugfs.h>

#include "kgsl.h"
#include "kgsl_device.h"
#include "kgsl_sharedmem.h"

#define KGSL_LOG_LEVEL_MAX     7

struct dentry *kgsl_debugfs_dir;
static struct dentry *pm_d_debugfs;
struct dentry *proc_d_debugfs;

static int pm_dump_set(void *data, u64 val)
{
	struct kgsl_device *device = data;

	if (val) {
		kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
		kgsl_postmortem_dump(device, 1);
		kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
	}

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(pm_dump_fops,
			NULL,
			pm_dump_set, "%llu\n");

static int pm_regs_enabled_set(void *data, u64 val)
{
	struct kgsl_device *device = data;
	device->pm_regs_enabled = val ? 1 : 0;
	return 0;
}

static int pm_regs_enabled_get(void *data, u64 *val)
{
	struct kgsl_device *device = data;
	*val = device->pm_regs_enabled;
	return 0;
}

static int pm_ib_enabled_set(void *data, u64 val)
{
	struct kgsl_device *device = data;
	device->pm_ib_enabled = val ? 1 : 0;
	return 0;
}

static int pm_ib_enabled_get(void *data, u64 *val)
{
	struct kgsl_device *device = data;
	*val = device->pm_ib_enabled;
	return 0;
}

static int pm_enabled_set(void *data, u64 val)
{
	struct kgsl_device *device = data;
	device->pm_dump_enable = val;
	return 0;
}

static int pm_enabled_get(void *data, u64 *val)
{
	struct kgsl_device *device = data;
	*val = device->pm_dump_enable;
	return 0;
}


DEFINE_SIMPLE_ATTRIBUTE(pm_regs_enabled_fops,
			pm_regs_enabled_get,
			pm_regs_enabled_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(pm_ib_enabled_fops,
			pm_ib_enabled_get,
			pm_ib_enabled_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(pm_enabled_fops,
			pm_enabled_get,
			pm_enabled_set, "%llu\n");

static inline int kgsl_log_set(unsigned int *log_val, void *data, u64 val)
{
	*log_val = min((unsigned int)val, (unsigned int)KGSL_LOG_LEVEL_MAX);
	return 0;
}

#define KGSL_DEBUGFS_LOG(__log)                         \
static int __log ## _set(void *data, u64 val)           \
{                                                       \
	struct kgsl_device *device = data;              \
	return kgsl_log_set(&device->__log, data, val); \
}                                                       \
static int __log ## _get(void *data, u64 *val)	        \
{                                                       \
	struct kgsl_device *device = data;              \
	*val = device->__log;                           \
	return 0;                                       \
}                                                       \
DEFINE_SIMPLE_ATTRIBUTE(__log ## _fops,                 \
__log ## _get, __log ## _set, "%llu\n");                \

KGSL_DEBUGFS_LOG(drv_log);
KGSL_DEBUGFS_LOG(cmd_log);
KGSL_DEBUGFS_LOG(ctxt_log);
KGSL_DEBUGFS_LOG(mem_log);
KGSL_DEBUGFS_LOG(pwr_log);

void kgsl_device_debugfs_init(struct kgsl_device *device)
{
	if (kgsl_debugfs_dir && !IS_ERR(kgsl_debugfs_dir))
		device->d_debugfs = debugfs_create_dir(device->name,
						       kgsl_debugfs_dir);

	if (!device->d_debugfs || IS_ERR(device->d_debugfs))
		return;

	debugfs_create_file("log_level_cmd", 0644, device->d_debugfs, device,
			    &cmd_log_fops);
	debugfs_create_file("log_level_ctxt", 0644, device->d_debugfs, device,
			    &ctxt_log_fops);
	debugfs_create_file("log_level_drv", 0644, device->d_debugfs, device,
			    &drv_log_fops);
	debugfs_create_file("log_level_mem", 0644, device->d_debugfs, device,
				&mem_log_fops);
	debugfs_create_file("log_level_pwr", 0644, device->d_debugfs, device,
				&pwr_log_fops);

	

	pm_d_debugfs = debugfs_create_dir("postmortem", device->d_debugfs);

	if (IS_ERR(pm_d_debugfs))
		return;

	debugfs_create_file("dump",  0600, pm_d_debugfs, device,
			    &pm_dump_fops);
	debugfs_create_file("regs_enabled", 0644, pm_d_debugfs, device,
			    &pm_regs_enabled_fops);
	debugfs_create_file("ib_enabled", 0644, pm_d_debugfs, device,
				    &pm_ib_enabled_fops);
	debugfs_create_file("enable", 0644, pm_d_debugfs, device,
				    &pm_enabled_fops);

}

static const char * const memtype_strings[] = {
	"gpumem",
	"pmem",
	"ashmem",
	"usermap",
	"ion",
};

static const char *memtype_str(int memtype)
{
	if (memtype < ARRAY_SIZE(memtype_strings))
		return memtype_strings[memtype];
	return "unknown";
}

static char get_alignflag(const struct kgsl_memdesc *m)
{
	int align = kgsl_memdesc_get_align(m);
	if (align >= ilog2(SZ_1M))
		return 'L';
	else if (align >= ilog2(SZ_64K))
		return 'l';
	return '-';
}

static char get_cacheflag(const struct kgsl_memdesc *m)
{
	static const char table[] = {
		[KGSL_CACHEMODE_WRITECOMBINE] = '-',
		[KGSL_CACHEMODE_UNCACHED] = 'u',
		[KGSL_CACHEMODE_WRITEBACK] = 'b',
		[KGSL_CACHEMODE_WRITETHROUGH] = 't',
	};
	return table[kgsl_memdesc_get_cachemode(m)];
}

static void print_mem_entry(struct seq_file *s, struct kgsl_mem_entry *entry)
{
	char flags[7];
	char usage[16];
	struct kgsl_memdesc *m = &entry->memdesc;

	flags[0] = kgsl_memdesc_is_global(m) ?  'g' : '-';
	flags[1] = m->flags & KGSL_MEMFLAGS_GPUREADONLY ? 'r' : '-';
	flags[2] = get_alignflag(m);
	flags[3] = get_cacheflag(m);
	flags[4] = kgsl_memdesc_use_cpu_map(m) ? 'p' : '-';
	flags[5] = (m->useraddr) ? 'Y' : 'N';
	flags[6] = '\0';

	kgsl_get_memory_usage(usage, sizeof(usage), m->flags);

	seq_printf(s, "%pK %pK %8zd %5d %6s %10s %16s %5d\n",
			(unsigned long *) m->gpuaddr,
			(unsigned long *) m->useraddr,
			m->size, entry->id, flags,
			memtype_str(entry->memtype), usage, m->sglen);
}

static int process_mem_print(struct seq_file *s, void *unused)
{
	struct kgsl_mem_entry *entry;
	struct rb_node *node;
	struct kgsl_process_private *private = s->private;
	int next = 0;

	seq_printf(s, "%8s %8s %8s %5s %6s %10s %16s %5s\n",
		   "gpuaddr", "useraddr", "size", "id", "flags", "type",
		   "usage", "sglen");

	
	spin_lock(&private->mem_lock);

	for (node = rb_first(&private->mem_rb); node; node = rb_next(node)) {
		entry = rb_entry(node, struct kgsl_mem_entry, node);
		print_mem_entry(s, entry);
	}


	
	while (1) {
		entry = idr_get_next(&private->mem_idr, &next);
		if (entry == NULL)
			break;
		if (entry->memdesc.gpuaddr == 0)
			print_mem_entry(s, entry);
		next++;
	}
	spin_unlock(&private->mem_lock);

	return 0;
}

static int process_mem_open(struct inode *inode, struct file *file)
{
	int ret;
	pid_t pid = (pid_t) (unsigned long) inode->i_private;
	struct kgsl_process_private *private = NULL;

	private = kgsl_process_private_find(pid);

	if (!private)
		return -ENODEV;

	ret = single_open(file, process_mem_print, private);
	if (ret)
		kgsl_process_private_put(private);

	return ret;
}

static int process_mem_release(struct inode *inode, struct file *file)
{
	struct kgsl_process_private *private =
		((struct seq_file *)file->private_data)->private;

	if (private)
		kgsl_process_private_put(private);

	return single_release(inode, file);
}

static const struct file_operations process_mem_fops = {
	.open = process_mem_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = process_mem_release,
};


int
kgsl_process_init_debugfs(struct kgsl_process_private *private)
{
	unsigned char name[16];
	int ret = 0;
	struct dentry *dentry;

	snprintf(name, sizeof(name), "%d", private->pid);

	private->debug_root = debugfs_create_dir(name, proc_d_debugfs);

	if (!private->debug_root)
		return -EINVAL;

	private->debug_root->d_inode->i_uid = proc_d_debugfs->d_inode->i_uid;
	private->debug_root->d_inode->i_gid = proc_d_debugfs->d_inode->i_gid;

	dentry = debugfs_create_file("mem", 0444, private->debug_root,
		(void *) ((unsigned long) private->pid), &process_mem_fops);

	if (IS_ERR(dentry)) {
		ret = PTR_ERR(dentry);

		if (ret == -ENODEV)
			ret = 0;
	} else if (dentry) {
		dentry->d_inode->i_uid = proc_d_debugfs->d_inode->i_uid;
		dentry->d_inode->i_gid = proc_d_debugfs->d_inode->i_gid;
	}

	return ret;
}

void kgsl_core_debugfs_init(void)
{
	kgsl_debugfs_dir = debugfs_create_dir("kgsl", 0);
	proc_d_debugfs = debugfs_create_dir("proc", kgsl_debugfs_dir);
}

void kgsl_core_debugfs_close(void)
{
	debugfs_remove_recursive(kgsl_debugfs_dir);
}
