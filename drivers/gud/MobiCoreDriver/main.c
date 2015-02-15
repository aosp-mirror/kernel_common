/*
 * MobiCore Driver Kernel Module.
 *
 * This driver represents the command proxy on the lowest layer, from the
 * secure world to the non secure world, and vice versa.

 * This driver offers IOCTL commands, for access to the secure world, and has
 * the interface from the secure world to the normal world.
 * The access to the driver is possible with a file descriptor,
 * which has to be created by the fd = open(/dev/mobicore) command or
 * fd = open(/dev/mobicore-user)
 *
 * <-- Copyright Giesecke & Devrient GmbH 2009-2012 -->
 * <-- Copyright Trustonic Limited 2013 -->
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/highmem.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/completion.h>
#include <linux/fdtable.h>
#include <linux/cdev.h>
#include <net/net_namespace.h>
#include <net/sock.h>
#include <net/tcp_states.h>
#include <net/af_unix.h>

#include "main.h"
#include "fastcall.h"

#include "arm.h"
#include "mem.h"
#include "ops.h"
#include "pm.h"
#include "debug.h"
#include "logging.h"

/* Define a MobiCore device structure for use with dev_debug() etc */
struct device_driver mcd_debug_name = {
	.name = "MobiCore"
};

struct device mcd_debug_subname = {
	.driver = &mcd_debug_name
};

struct device *mcd = &mcd_debug_subname;

/* We need 2 devices for admin and user interface*/
#define MC_DEV_MAX 2

/* Need to discover a chrdev region for the driver */
static dev_t mc_dev_admin, mc_dev_user;
struct cdev mc_admin_cdev, mc_user_cdev;
/* Device class for the driver assigned major */
static struct class *mc_device_class;

#ifndef FMODE_PATH
 #define FMODE_PATH 0x0
#endif

static struct sock *__get_socket(struct file *filp)
{
	struct sock *u_sock = NULL;
	struct inode *inode = filp->f_path.dentry->d_inode;

	/*
	 *	Socket ?
	 */
	if (S_ISSOCK(inode->i_mode) && !(filp->f_mode & FMODE_PATH)) {
		struct socket *sock = SOCKET_I(inode);
		struct sock *s = sock->sk;

		/*
		 *	PF_UNIX ?
		 */
		if (s && sock->ops && sock->ops->family == PF_UNIX)
			u_sock = s;
	}
	return u_sock;
}


/* MobiCore interrupt context data */
struct mc_context ctx;

/* Get process context from file pointer */
static struct mc_instance *get_instance(struct file *file)
{
	return (struct mc_instance *)(file->private_data);
}

/* Get a unique ID */
unsigned int get_unique_id(void)
{
	return (unsigned int)atomic_inc_return(&ctx.unique_counter);
}

/* Clears the reserved bit of each page and frees the pages */
static inline void free_continguous_pages(void *addr, unsigned int order)
{
	int i;
	struct page *page = virt_to_page(addr);
	for (i = 0; i < (1<<order); i++) {
		MCDRV_DBG_VERBOSE(mcd, "free page at 0x%p", page);
		clear_bit(PG_reserved, &page->flags);
		page++;
	}

	MCDRV_DBG_VERBOSE(mcd, "freeing addr:%p, order:%x", addr, order);
	free_pages((unsigned long)addr, order);
}

/* Frees the memory associated with a buffer */
static int free_buffer(struct mc_buffer *buffer, bool unlock)
{
	if (buffer->handle == 0)
		return -EINVAL;

	if (buffer->addr == 0)
		return -EINVAL;

	MCDRV_DBG_VERBOSE(mcd,
			  "handle=%u phys_addr=0x%llx, virt_addr=0x%p len=%u",
		  buffer->handle, (u64)buffer->phys,
		  buffer->addr, buffer->len);

	if (!atomic_dec_and_test(&buffer->usage)) {
		MCDRV_DBG_VERBOSE(mcd, "Could not free %u", buffer->handle);
		return 0;
	}

	list_del(&buffer->list);

	free_continguous_pages(buffer->addr, buffer->order);
	kfree(buffer);
	return 0;
}

static uint32_t mc_find_cont_wsm_addr(struct mc_instance *instance, void *uaddr,
	void **addr, uint32_t len)
{
	int ret = 0;
	struct mc_buffer *buffer;

	if (WARN(!instance, "No instance data available"))
		return -EFAULT;

	mutex_lock(&instance->lock);

	mutex_lock(&ctx.bufs_lock);

	/* search for the given handle in the buffers list */
	list_for_each_entry(buffer, &ctx.cont_bufs, list) {
		if (buffer->uaddr == uaddr && buffer->len == len) {
			*addr = buffer->addr;
			goto found;
		}
	}

	/* Coundn't find the buffer */
	ret = -EINVAL;

found:
	mutex_unlock(&ctx.bufs_lock);
	mutex_unlock(&instance->lock);

	return ret;
}

bool mc_check_owner_fd(struct mc_instance *instance, int32_t fd)
{
#ifndef __ARM_VE_A9X4_STD__
	struct file *fp;
	struct sock *s;
	struct files_struct *files;
	struct task_struct *peer = NULL;
	bool ret = false;

	MCDRV_DBG_VERBOSE(mcd, "Finding wsm for fd = %d", fd);
	if (!instance)
		return false;

	if (is_daemon(instance))
		return true;

	fp = fcheck_files(current->files, fd);
	if (!fp)
		goto out;
	s = __get_socket(fp);
	if (s) {
		peer = get_pid_task(s->sk_peer_pid, PIDTYPE_PID);
		MCDRV_DBG_VERBOSE(mcd, "Found pid for fd %d", peer->pid);
	}
	if (peer) {
		task_lock(peer);
		files = peer->files;
		if (!files)
			goto out;
		for (fd = 0; fd < files_fdtable(files)->max_fds; fd++) {
			fp = fcheck_files(files, fd);
			if (!fp)
				continue;
			if (fp->private_data == instance) {
				MCDRV_DBG_VERBOSE(mcd, "Found owner!");
				ret = true;
				goto out;
			}
		}
	} else {
		MCDRV_DBG(mcd, "Owner not found!");
		return false;
	}
out:
	if (peer)
		task_unlock(peer);
	if (!ret)
		MCDRV_DBG(mcd, "Owner not found!");
	return ret;
#else
	return true;
#endif
}
static uint32_t mc_find_cont_wsm(struct mc_instance *instance, uint32_t handle,
	int32_t fd, phys_addr_t *phys, uint32_t *len)
{
	int ret = 0;
	struct mc_buffer *buffer;

	if (WARN(!instance, "No instance data available"))
		return -EFAULT;

	if (WARN_ON(!is_daemon(instance))) {
		MCDRV_DBG_ERROR(mcd, "caller not MobiCore Daemon");
		return -EPERM;
	}

	mutex_lock(&instance->lock);

	mutex_lock(&ctx.bufs_lock);

	/* search for the given handle in the buffers list */
	list_for_each_entry(buffer, &ctx.cont_bufs, list) {
		if (buffer->handle == handle) {
			if (mc_check_owner_fd(buffer->instance, fd)) {
				*phys = buffer->phys;
				*len = buffer->len;
				goto found;
			} else {
				break;
			}
		}
	}

	/* Couldn't find the buffer */
	ret = -EINVAL;

found:
	mutex_unlock(&ctx.bufs_lock);
	mutex_unlock(&instance->lock);

	return ret;
}

/*
 * __free_buffer - Free a WSM buffer allocated with mobicore_allocate_wsm
 *
 * @instance
 * @handle		handle of the buffer
 *
 * Returns 0 if no error
 *
 */
static int __free_buffer(struct mc_instance *instance, uint32_t handle,
		bool unlock)
{
	int ret = 0;
	struct mc_buffer *buffer;
	void *uaddr = NULL;
	size_t len = 0;
#ifndef MC_VM_UNMAP
	struct mm_struct *mm = current->mm;
#endif

	if (WARN(!instance, "No instance data available"))
		return -EFAULT;

	mutex_lock(&ctx.bufs_lock);
	/* search for the given handle in the buffers list */
	list_for_each_entry(buffer, &ctx.cont_bufs, list) {
		if (buffer->handle == handle) {
			uaddr = buffer->uaddr;
			len = buffer->len;
			goto found_buffer;
		}
	}
	ret = -EINVAL;
	goto err;
found_buffer:
	if (!is_daemon(instance) && buffer->instance != instance) {
		ret = -EPERM;
		goto err;
	}
	mutex_unlock(&ctx.bufs_lock);
	/* Only unmap if the request is comming from the user space and
	 * it hasn't already been unmapped */
	if (unlock == false && uaddr != NULL) {
#ifndef MC_VM_UNMAP
		/* do_munmap must be done with mm->mmap_sem taken */
		down_write(&mm->mmap_sem);
		ret = do_munmap(mm, (long unsigned int)uaddr, len);
		up_write(&mm->mmap_sem);

#else
		ret = vm_munmap((long unsigned int)uaddr, len);
#endif
		if (ret < 0) {
			/* Something is not right if we end up here, better not
			 * clean the buffer so we just leak memory instead of
			 * creating security issues */
			MCDRV_DBG_ERROR(mcd, "Memory can't be unmapped");
			return -EINVAL;
		}
	}

	mutex_lock(&ctx.bufs_lock);
	/* search for the given handle in the buffers list */
	list_for_each_entry(buffer, &ctx.cont_bufs, list) {
		if (buffer->handle == handle)
			goto del_buffer;
	}
	ret = -EINVAL;
	goto err;

del_buffer:
	if (is_daemon(instance) || buffer->instance == instance)
		ret = free_buffer(buffer, unlock);
	else
		ret = -EPERM;
err:
	mutex_unlock(&ctx.bufs_lock);
	return ret;
}

int mc_free_buffer(struct mc_instance *instance, uint32_t handle)
{
	int ret = 0;

	if (WARN(!instance, "No instance data available"))
		return -EFAULT;

	mutex_lock(&instance->lock);

	ret = __free_buffer(instance, handle, false);
	mutex_unlock(&instance->lock);
	return ret;
}


int mc_get_buffer(struct mc_instance *instance,
	struct mc_buffer **buffer, unsigned long len)
{
	struct mc_buffer *cbuffer = NULL;
	void *addr = 0;
	phys_addr_t phys = 0;
	unsigned int order;
	unsigned long allocated_size;
	int ret = 0;

	if (WARN(!instance, "No instance data available"))
		return -EFAULT;

	if (len == 0) {
		MCDRV_DBG_WARN(mcd, "cannot allocate size 0");
		return -ENOMEM;
	}

	order = get_order(len);
	if (order > MAX_ORDER) {
		MCDRV_DBG_WARN(mcd, "Buffer size too large");
		return -ENOMEM;
	}
	allocated_size = (1 << order) * PAGE_SIZE;

	if (mutex_lock_interruptible(&instance->lock))
		return -ERESTARTSYS;

	/* allocate a new buffer. */
	cbuffer = kzalloc(sizeof(struct mc_buffer), GFP_KERNEL);

	if (cbuffer == NULL) {
		MCDRV_DBG_WARN(mcd,
			       "MMAP_WSM request: could not allocate buffer");
		ret = -ENOMEM;
		goto unlock_instance;
	}
	mutex_lock(&ctx.bufs_lock);

	MCDRV_DBG_VERBOSE(mcd, "size %ld -> order %d --> %ld (2^n pages)",
			  len, order, allocated_size);

	addr = (void *)__get_free_pages(GFP_USER | __GFP_ZERO, order);

	if (addr == NULL) {
		MCDRV_DBG_WARN(mcd, "get_free_pages failed");
		ret = -ENOMEM;
		goto err;
	}
	phys = virt_to_phys(addr);
	cbuffer->handle = get_unique_id();
	cbuffer->phys = phys;
	cbuffer->addr = addr;
	cbuffer->order = order;
	cbuffer->len = len;
	cbuffer->instance = instance;
	cbuffer->uaddr = 0;
	/* Refcount +1 because the TLC is requesting it */
	atomic_set(&cbuffer->usage, 1);

	INIT_LIST_HEAD(&cbuffer->list);
	list_add(&cbuffer->list, &ctx.cont_bufs);

	MCDRV_DBG_VERBOSE(mcd,
			  "allocated phys=0x%llx - 0x%llx, size=%ld, kvirt=0x%p"
			  ", h=%d",
			  (u64)phys,
			  (u64)(phys+allocated_size),
			  allocated_size, addr, cbuffer->handle);
	*buffer = cbuffer;
	goto unlock;

err:
	kfree(cbuffer);
unlock:
	mutex_unlock(&ctx.bufs_lock);
unlock_instance:
	mutex_unlock(&instance->lock);
	return ret;
}

/*
 * __lock_buffer() - Locks a contiguous buffer - +1 refcount.
 * Assumes the instance lock is already taken!
 */
static int __lock_buffer(struct mc_instance *instance, uint32_t handle)
{
	int ret = 0;
	struct mc_buffer *buffer;

	if (WARN(!instance, "No instance data available"))
		return -EFAULT;

	if (WARN_ON(!is_daemon(instance))) {
		MCDRV_DBG_ERROR(mcd, "caller not MobiCore Daemon");
		return -EPERM;
	}

	mutex_lock(&ctx.bufs_lock);
	/* search for the given handle in the buffers list */
	list_for_each_entry(buffer, &ctx.cont_bufs, list) {
		if (buffer->handle == handle) {
			atomic_inc(&buffer->usage);
			goto unlock;
		}
	}
	ret = -EINVAL;

unlock:
	mutex_unlock(&ctx.bufs_lock);
	return ret;
}

static phys_addr_t get_mci_base_phys(unsigned int len)
{
	if (ctx.mci_base.phys) {
		return ctx.mci_base.phys;
	} else {
		unsigned int order = get_order(len);
		ctx.mcp = NULL;
		ctx.mci_base.order = order;
		ctx.mci_base.addr =
			(void *)__get_free_pages(GFP_USER | __GFP_ZERO, order);
		if (ctx.mci_base.addr == NULL) {
			MCDRV_DBG_WARN(mcd, "get_free_pages failed");
			memset(&ctx.mci_base, 0, sizeof(ctx.mci_base));
			return 0;
		}
		ctx.mci_base.phys = virt_to_phys(ctx.mci_base.addr);
		return ctx.mci_base.phys;
	}
}

/*
 * Create a MMU table from a virtual memory buffer which can be vmalloc
 * or user space virtual memory
 */
int mc_register_wsm_mmu(struct mc_instance *instance,
	void *buffer, uint32_t len,
	uint32_t *handle, phys_addr_t *phys)
{
	int ret = 0;
	struct mc_mmu_table *table = NULL;
	struct task_struct *task = current;
	void *kbuff = NULL;

	if (WARN(!instance, "No instance data available"))
		return -EFAULT;

	if (len == 0) {
		MCDRV_DBG_ERROR(mcd, "len=0 is not supported!");
		return -EINVAL;
	}

	MCDRV_DBG_VERBOSE(mcd, "buffer: %p, len=%08x", buffer, len);

	if (!mc_find_cont_wsm_addr(instance, buffer, &kbuff, len))
		table = mc_alloc_mmu_table(instance, NULL, kbuff, len);
	else
		table = mc_alloc_mmu_table(instance, task, buffer, len);

	if (IS_ERR(table)) {
		MCDRV_DBG_ERROR(mcd, "mc_alloc_mmu_table() failed");
		return -EINVAL;
	}

	/* set response */
	*handle = table->handle;
	/* WARNING: daemon shouldn't know this either, but live with it */
	if (is_daemon(instance))
		*phys = table->phys;
	else
		*phys = 0;

	MCDRV_DBG_VERBOSE(mcd, "handle: %d, phys=0x%llX",
			  *handle, (u64)(*phys));

	MCDRV_DBG_VERBOSE(mcd, "exit with %d/0x%08X", ret, ret);

	return ret;
}

int mc_unregister_wsm_mmu(struct mc_instance *instance, uint32_t handle)
{
	int ret = 0;

	if (WARN(!instance, "No instance data available"))
		return -EFAULT;

	/* free table (if no further locks exist) */
	mc_free_mmu_table(instance, handle);

	return ret;
}
/* Lock the object from handle, it could be a WSM MMU table or a cont buffer! */
static int mc_lock_handle(struct mc_instance *instance, uint32_t handle)
{
	int ret = 0;

	if (WARN(!instance, "No instance data available"))
		return -EFAULT;

	if (WARN_ON(!is_daemon(instance))) {
		MCDRV_DBG_ERROR(mcd, "caller not MobiCore Daemon");
		return -EPERM;
	}

	mutex_lock(&instance->lock);
	ret = mc_lock_mmu_table(instance, handle);

	/* Handle was not a MMU table but a cont buffer */
	if (ret == -EINVAL) {
		/* Call the non locking variant! */
		ret = __lock_buffer(instance, handle);
	}

	mutex_unlock(&instance->lock);

	return ret;
}

static int mc_unlock_handle(struct mc_instance *instance, uint32_t handle)
{
	int ret = 0;

	if (WARN(!instance, "No instance data available"))
		return -EFAULT;

	if (WARN_ON(!is_daemon(instance))) {
		MCDRV_DBG_ERROR(mcd, "caller not MobiCore Daemon");
		return -EPERM;
	}

	mutex_lock(&instance->lock);
	ret = mc_free_mmu_table(instance, handle);

	/* Not a MMU table, then it must be a buffer */
	if (ret == -EINVAL) {
		/* Call the non locking variant! */
		ret = __free_buffer(instance, handle, true);
	}
	mutex_unlock(&instance->lock);

	return ret;
}

static phys_addr_t mc_find_wsm_mmu(struct mc_instance *instance,
	uint32_t handle, int32_t fd)
{
	if (WARN(!instance, "No instance data available"))
		return 0;

	if (WARN_ON(!is_daemon(instance))) {
		MCDRV_DBG_ERROR(mcd, "caller not MobiCore Daemon");
		return 0;
	}

	return mc_find_mmu_table(handle, fd);
}

static int mc_clean_wsm_mmu(struct mc_instance *instance)
{
	if (WARN(!instance, "No instance data available"))
		return -EFAULT;

	if (WARN_ON(!is_daemon(instance))) {
		MCDRV_DBG_ERROR(mcd, "caller not MobiCore Daemon");
		return -EPERM;
	}

	mc_clean_mmu_tables();

	return 0;
}

static int mc_fd_mmap(struct file *file, struct vm_area_struct *vmarea)
{
	struct mc_instance *instance = get_instance(file);
	unsigned long len = vmarea->vm_end - vmarea->vm_start;
	phys_addr_t paddr = (vmarea->vm_pgoff << PAGE_SHIFT);
	unsigned int pfn;
	struct mc_buffer *buffer = 0;
	int ret = 0;

	MCDRV_DBG_VERBOSE(mcd, "enter (vma start=0x%p, size=%ld, mci=0x%llX)",
			  (void *)vmarea->vm_start, len,
			  (u64)ctx.mci_base.phys);

	if (WARN(!instance, "No instance data available"))
		return -EFAULT;

	if (len == 0) {
		MCDRV_DBG_ERROR(mcd, "cannot allocate size 0");
		return -ENOMEM;
	}
	if (paddr) {
		mutex_lock(&ctx.bufs_lock);

		/* search for the buffer list. */
		list_for_each_entry(buffer, &ctx.cont_bufs, list) {
			/* Only allow mapping if the client owns it!*/
			if (buffer->phys == paddr &&
			    buffer->instance == instance) {
				/* We shouldn't do remap with larger size */
				if (buffer->len > len)
					break;
				/* We can't allow mapping the buffer twice */
				if (!buffer->uaddr)
					goto found;
				else
					break;
			}
		}
		/* Nothing found return */
		mutex_unlock(&ctx.bufs_lock);
		return -EINVAL;

found:
		buffer->uaddr = (void *)vmarea->vm_start;
		vmarea->vm_flags |= VM_IO;
		/*
		 * Convert kernel address to user address. Kernel address begins
		 * at PAGE_OFFSET, user address range is below PAGE_OFFSET.
		 * Remapping the area is always done, so multiple mappings
		 * of one region are possible. Now remap kernel address
		 * space into user space
		 */
		pfn = (unsigned int)paddr >> PAGE_SHIFT;
		ret = (int)remap_pfn_range(vmarea, vmarea->vm_start, pfn,
			buffer->len, vmarea->vm_page_prot);
		/* If the remap failed then don't mark this buffer as marked
		 * since the unmaping will also fail */
		if (ret)
			buffer->uaddr = NULL;
		mutex_unlock(&ctx.bufs_lock);
	} else {
		if (!is_daemon(instance))
			return -EPERM;

		paddr = get_mci_base_phys(len);
		if (!paddr)
			return -EFAULT;

		vmarea->vm_flags |= VM_IO;
		/*
		 * Convert kernel address to user address. Kernel address begins
		 * at PAGE_OFFSET, user address range is below PAGE_OFFSET.
		 * Remapping the area is always done, so multiple mappings
		 * of one region are possible. Now remap kernel address
		 * space into user space
		 */
		pfn = (unsigned int)paddr >> PAGE_SHIFT;
		ret = (int)remap_pfn_range(vmarea, vmarea->vm_start, pfn, len,
			vmarea->vm_page_prot);
	}

	MCDRV_DBG_VERBOSE(mcd, "exit with %d/0x%08X", ret, ret);

	return ret;
}

static inline int ioctl_check_pointer(unsigned int cmd, int __user *uarg)
{
	int err = 0;
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, uarg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, uarg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	return 0;
}

/*
 * mc_fd_user_ioctl() - Will be called from user space as ioctl(..)
 * @file	pointer to file
 * @cmd		command
 * @arg		arguments
 *
 * Returns 0 for OK and an errno in case of error
 */
static long mc_fd_user_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	struct mc_instance *instance = get_instance(file);
	int __user *uarg = (int __user *)arg;
	int ret = -EINVAL;

	if (WARN(!instance, "No instance data available"))
		return -EFAULT;

	if (ioctl_check_pointer(cmd, uarg))
		return -EFAULT;

	switch (cmd) {
	case MC_IO_FREE:
		ret = mc_free_buffer(instance, (uint32_t)arg);
		break;

	case MC_IO_REG_WSM:{
		struct mc_ioctl_reg_wsm reg;
		phys_addr_t phys;
		if (copy_from_user(&reg, uarg, sizeof(reg)))
			return -EFAULT;

		ret = mc_register_wsm_mmu(instance, (void *)reg.buffer,
			reg.len, &reg.handle, &phys);
		reg.table_phys = phys;

		if (!ret) {
			if (copy_to_user(uarg, &reg, sizeof(reg))) {
				ret = -EFAULT;
				mc_unregister_wsm_mmu(instance, reg.handle);
			}
		}
		break;
	}
	case MC_IO_UNREG_WSM:
		ret = mc_unregister_wsm_mmu(instance, (uint32_t)arg);
		break;

	case MC_IO_VERSION:
		ret = put_user(mc_get_version(), uarg);
		if (ret)
			MCDRV_DBG_ERROR(mcd,
					"IOCTL_GET_VERSION failed to put data");
		break;

	case MC_IO_MAP_WSM:{
		struct mc_ioctl_map map;
		struct mc_buffer *buffer = 0;
		if (copy_from_user(&map, uarg, sizeof(map)))
			return -EFAULT;

		/* Setup the WSM buffer structure! */
		if (mc_get_buffer(instance, &buffer, map.len))
			return -EFAULT;

		map.handle = buffer->handle;
		map.phys_addr = buffer->phys;
		map.reused = 0;
		if (copy_to_user(uarg, &map, sizeof(map)))
			ret = -EFAULT;
		else
			ret = 0;
		break;
	}
	default:
		MCDRV_DBG_ERROR(mcd, "unsupported cmd=0x%x", cmd);
		ret = -ENOIOCTLCMD;
		break;

	} /* end switch(cmd) */

#ifdef MC_MEM_TRACES
	mobicore_log_read();
#endif

	return (int)ret;
}

static long mc_fd_admin_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	struct mc_instance *instance = get_instance(file);
	int __user *uarg = (int __user *)arg;
	int ret = -EINVAL;

	if (WARN(!instance, "No instance data available"))
		return -EFAULT;

	if (WARN_ON(!is_daemon(instance))) {
		MCDRV_DBG_ERROR(mcd, "caller not MobiCore Daemon");
		return -EPERM;
	}

	if (ioctl_check_pointer(cmd, uarg))
		return -EFAULT;

	switch (cmd) {
	case MC_IO_INIT: {
		struct mc_ioctl_init init;
		ctx.mcp = NULL;
		if (!ctx.mci_base.phys) {
			MCDRV_DBG_ERROR(mcd,
					"Cannot init MobiCore without MCI!");
			return -EINVAL;
		}
		if (copy_from_user(&init, uarg, sizeof(init)))
			return -EFAULT;

		ctx.mcp = ctx.mci_base.addr + init.mcp_offset;
		ret = mc_init(ctx.mci_base.phys, init.nq_length,
			init.mcp_offset, init.mcp_length);
		break;
	}
	case MC_IO_INFO: {
		struct mc_ioctl_info info;
		if (copy_from_user(&info, uarg, sizeof(info)))
			return -EFAULT;

		ret = mc_info(info.ext_info_id, &info.state,
			&info.ext_info);

		if (!ret) {
			if (copy_to_user(uarg, &info, sizeof(info)))
				ret = -EFAULT;
		}
		break;
	}
	case MC_IO_YIELD:
		ret = mc_yield();
		break;

	case MC_IO_NSIQ:
		ret = mc_nsiq();
		break;

	case MC_IO_LOCK_WSM: {
		ret = mc_lock_handle(instance, (uint32_t)arg);
		break;
	}
	case MC_IO_UNLOCK_WSM:
		ret = mc_unlock_handle(instance, (uint32_t)arg);
		break;
	case MC_IO_CLEAN_WSM:
		ret = mc_clean_wsm_mmu(instance);
		break;
	case MC_IO_RESOLVE_WSM: {
		phys_addr_t phys;
		struct mc_ioctl_resolv_wsm wsm;
		if (copy_from_user(&wsm, uarg, sizeof(wsm)))
			return -EFAULT;
		phys = mc_find_wsm_mmu(instance, wsm.handle, wsm.fd);
		if (!phys)
			return -EINVAL;

		wsm.phys = phys;
		if (copy_to_user(uarg, &wsm, sizeof(wsm)))
			return -EFAULT;
		ret = 0;
		break;
	}
	case MC_IO_RESOLVE_CONT_WSM: {
		struct mc_ioctl_resolv_cont_wsm cont_wsm;
		phys_addr_t phys = 0;
		uint32_t len = 0;
		if (copy_from_user(&cont_wsm, uarg, sizeof(cont_wsm)))
			return -EFAULT;
		ret = mc_find_cont_wsm(instance, cont_wsm.handle, cont_wsm.fd,
					&phys, &len);
		if (!ret) {
			cont_wsm.phys = phys;
			cont_wsm.length = len;
			if (copy_to_user(uarg, &cont_wsm, sizeof(cont_wsm)))
				ret = -EFAULT;
		}
		break;
	}
	case MC_IO_MAP_MCI:{
		struct mc_ioctl_map map;
		if (copy_from_user(&map, uarg, sizeof(map)))
			return -EFAULT;

		map.reused = (ctx.mci_base.phys != 0);
		map.phys_addr = get_mci_base_phys(map.len);
		if (!map.phys_addr) {
			MCDRV_DBG_ERROR(mcd, "Failed to setup MCI buffer!");
			return -EFAULT;
		}

		if (copy_to_user(uarg, &map, sizeof(map)))
			ret = -EFAULT;
		ret = 0;
		break;
	}
	case MC_IO_LOG_SETUP: {
#ifdef MC_MEM_TRACES
		ret = mobicore_log_setup();
#endif
		break;
	}

	/* The rest is handled commonly by user IOCTL */
	default:
		ret = mc_fd_user_ioctl(file, cmd, arg);
	} /* end switch(cmd) */

#ifdef MC_MEM_TRACES
	mobicore_log_read();
#endif

	return (int)ret;
}

/*
 * mc_fd_read() - This will be called from user space as read(...)
 * @file:	file pointer
 * @buffer:	buffer where to copy to(userspace)
 * @buffer_len:	number of requested data
 * @pos:	not used
 *
 * The read function is blocking until a interrupt occurs. In that case the
 * event counter is copied into user space and the function is finished.
 *
 * If OK this function returns the number of copied data otherwise it returns
 * errno
 */
static ssize_t mc_fd_read(struct file *file, char *buffer, size_t buffer_len,
			  loff_t *pos)
{
	int ret = 0, ssiq_counter;
	struct mc_instance *instance = get_instance(file);

	if (WARN(!instance, "No instance data available"))
		return -EFAULT;

	/* avoid debug output on non-error, because this is call quite often */
	MCDRV_DBG_VERBOSE(mcd, "enter");

	/* only the MobiCore Daemon is allowed to call this function */
	if (WARN_ON(!is_daemon(instance))) {
		MCDRV_DBG_ERROR(mcd, "caller not MobiCore Daemon");
		return -EPERM;
	}

	if (buffer_len < sizeof(unsigned int)) {
		MCDRV_DBG_ERROR(mcd, "invalid length");
		return -EINVAL;
	}

	for (;;) {
		if (wait_for_completion_interruptible(&ctx.isr_comp)) {
			MCDRV_DBG_VERBOSE(mcd, "read interrupted");
			return -ERESTARTSYS;
		}

		ssiq_counter = atomic_read(&ctx.isr_counter);
		MCDRV_DBG_VERBOSE(mcd, "ssiq_counter=%i, ctx.counter=%i",
				  ssiq_counter, ctx.evt_counter);

		if (ssiq_counter != ctx.evt_counter) {
			/* read data and exit loop without error */
			ctx.evt_counter = ssiq_counter;
			ret = 0;
			break;
		}

		/* end loop if non-blocking */
		if (file->f_flags & O_NONBLOCK) {
			MCDRV_DBG_ERROR(mcd, "non-blocking read");
			return -EAGAIN;
		}

		if (signal_pending(current)) {
			MCDRV_DBG_VERBOSE(mcd, "received signal.");
			return -ERESTARTSYS;
		}
	}

	/* read data and exit loop */
	ret = copy_to_user(buffer, &ctx.evt_counter, sizeof(unsigned int));

	if (ret != 0) {
		MCDRV_DBG_ERROR(mcd, "copy_to_user failed");
		return -EFAULT;
	}

	ret = sizeof(unsigned int);

	return (ssize_t)ret;
}

/*
 * Initialize a new mobicore API instance object
 *
 * @return Instance or NULL if no allocation was possible.
 */
struct mc_instance *mc_alloc_instance(void)
{
	struct mc_instance *instance;

	instance = kzalloc(sizeof(*instance), GFP_KERNEL);
	if (instance == NULL)
		return NULL;

	/* get a unique ID for this instance (PIDs are not unique) */
	instance->handle = get_unique_id();

	mutex_init(&instance->lock);

	return instance;
}

#if defined(TBASE_CORE_SWITCHER) && defined(DEBUG)
static ssize_t mc_fd_write(struct file *file, const char __user *buffer,
			size_t buffer_len, loff_t *x)
{
	uint32_t cpu_new;
	/* we only consider one digit */
	char buf[2];
	struct mc_instance *instance = get_instance(file);

	if (WARN(!instance, "No instance data available"))
		return -EFAULT;

	/* Invalid data, nothing to do */
	if (buffer_len < 1)
		return -EINVAL;

	/* Invalid data, nothing to do */
	if (copy_from_user(buf, buffer, min(sizeof(buf), buffer_len)))
		return -EFAULT;

	if (buf[0] == 'n') {
		mc_nsiq();
	/* If it's a digit then switch cores */
	} else if ((buf[0] >= '0') && (buf[0] <= '9')) {
		cpu_new = buf[0] - '0';
		if (cpu_new <= 8) {
			MCDRV_DBG_VERBOSE(mcd, "Set Active Cpu: %d\n", cpu_new);
			mc_switch_core(cpu_new);
		}
	} else {
		return -EINVAL;
	}

	return buffer_len;
}
#endif

/*
 * Release a mobicore instance object and all objects related to it
 * @instance:	instance
 * Returns 0 if Ok or -E ERROR
 */
int mc_release_instance(struct mc_instance *instance)
{
	struct mc_buffer *buffer, *tmp;

	if (WARN(!instance, "No instance data available"))
		return -EFAULT;

	mutex_lock(&instance->lock);
	mc_clear_mmu_tables(instance);

	mutex_lock(&ctx.bufs_lock);
	/* release all mapped data */

	/* Check if some buffers are orphaned. */
	list_for_each_entry_safe(buffer, tmp, &ctx.cont_bufs, list) {
		/* It's safe here to only call free_buffer() without unmapping
		 * because mmap() takes a refcount to the file's fd so only
		 * time we end up here is when everything has been unmaped or
		 * the process called exit() */
		if (buffer->instance == instance) {
			buffer->instance = NULL;
			free_buffer(buffer, false);
		}
	}
	mutex_unlock(&ctx.bufs_lock);

	mutex_unlock(&instance->lock);

	/* release instance context */
	kfree(instance);

	return 0;
}

/*
 * mc_fd_user_open() - Will be called from user space as fd = open(...)
 * A set of internal instance data are created and initialized.
 *
 * @inode
 * @file
 * Returns 0 if OK or -ENOMEM if no allocation was possible.
 */
static int mc_fd_user_open(struct inode *inode, struct file *file)
{
	struct mc_instance *instance;

	MCDRV_DBG_VERBOSE(mcd, "enter");

	instance = mc_alloc_instance();
	if (instance == NULL)
		return -ENOMEM;

	/* store instance data reference */
	file->private_data = instance;

	return 0;
}

static int mc_fd_admin_open(struct inode *inode, struct file *file)
{
	struct mc_instance *instance;

	/*
	 * The daemon is already set so we can't allow anybody else to open
	 * the admin interface.
	 */
	if (ctx.daemon_inst) {
		MCDRV_DBG_ERROR(mcd, "Daemon is already connected");
		return -EPERM;
	}
	/* Setup the usual variables */
	if (mc_fd_user_open(inode, file))
		return -ENOMEM;
	instance = get_instance(file);

	MCDRV_DBG(mcd, "accept this as MobiCore Daemon");

	ctx.daemon_inst = instance;
	ctx.daemon = current;
	instance->admin = true;
	init_completion(&ctx.isr_comp);
	/* init ssiq event counter */
	ctx.evt_counter = atomic_read(&(ctx.isr_counter));

	return 0;
}

/*
 * mc_fd_release() - This function will be called from user space as close(...)
 * The instance data are freed and the associated memory pages are unreserved.
 *
 * @inode
 * @file
 *
 * Returns 0
 */
static int mc_fd_release(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct mc_instance *instance = get_instance(file);

	if (WARN(!instance, "No instance data available"))
		return -EFAULT;

	/* check if daemon closes us. */
	if (is_daemon(instance)) {
		MCDRV_DBG_WARN(mcd, "MobiCore Daemon died");
		ctx.daemon_inst = NULL;
		ctx.daemon = NULL;
	}

	ret = mc_release_instance(instance);

	/*
	 * ret is quite irrelevant here as most apps don't care about the
	 * return value from close() and it's quite difficult to recover
	 */
	MCDRV_DBG_VERBOSE(mcd, "exit with %d/0x%08X", ret, ret);

	return (int)ret;
}

/*
 * This function represents the interrupt function of the mcDrvModule.
 * It signals by incrementing of an event counter and the start of the read
 * waiting queue, the read function a interrupt has occurred.
 */
static irqreturn_t mc_ssiq_isr(int intr, void *context)
{
	/* increment interrupt event counter */
	atomic_inc(&(ctx.isr_counter));

	/* signal the daemon */
	complete(&ctx.isr_comp);
#ifdef MC_MEM_TRACES
	mobicore_log_read();
#endif
	return IRQ_HANDLED;
}

/* function table structure of this device driver. */
static const struct file_operations mc_admin_fops = {
	.owner		= THIS_MODULE,
	.open		= mc_fd_admin_open,
	.release	= mc_fd_release,
	.unlocked_ioctl	= mc_fd_admin_ioctl,
	.mmap		= mc_fd_mmap,
	.read		= mc_fd_read,
};

/* function table structure of this device driver. */
static const struct file_operations mc_user_fops = {
	.owner		= THIS_MODULE,
	.open		= mc_fd_user_open,
	.release	= mc_fd_release,
	.unlocked_ioctl	= mc_fd_user_ioctl,
	.mmap		= mc_fd_mmap,
#if defined(TBASE_CORE_SWITCHER) && defined(DEBUG)
	.write          = mc_fd_write,
#endif
};

static int create_devices(void)
{
	int ret = 0;

	cdev_init(&mc_admin_cdev, &mc_admin_fops);
	cdev_init(&mc_user_cdev, &mc_user_fops);

	mc_device_class = class_create(THIS_MODULE, "mobicore");
	if (IS_ERR(mc_device_class)) {
		MCDRV_DBG_ERROR(mcd, "failed to create device class");
		ret = PTR_ERR(mc_device_class);
		goto out;
	}

	ret = alloc_chrdev_region(&mc_dev_admin, 0, MC_DEV_MAX, "mobicore");
	if (ret < 0) {
		MCDRV_DBG_ERROR(mcd, "failed to allocate char dev region");
		goto error;
	}
	mc_dev_user = MKDEV(MAJOR(mc_dev_admin), 1);

	MCDRV_DBG_VERBOSE(mcd, "%s: dev %d", "mobicore", MAJOR(mc_dev_admin));

	/* First the ADMIN node */
	ret = cdev_add(&mc_admin_cdev,  mc_dev_admin, 1);
	if (ret != 0) {
		MCDRV_DBG_ERROR(mcd, "admin device register failed");
		goto error;
	}
	mc_admin_cdev.owner = THIS_MODULE;
	device_create(mc_device_class, NULL, mc_dev_admin, NULL,
		      MC_ADMIN_DEVNODE);

	/* Then the user node */

	ret = cdev_add(&mc_user_cdev, mc_dev_user, 1);
	if (ret != 0) {
		MCDRV_DBG_ERROR(mcd, "user device register failed");
		goto error_unregister;
	}
	mc_user_cdev.owner = THIS_MODULE;
	device_create(mc_device_class, NULL, mc_dev_user, NULL,
		      MC_USER_DEVNODE);

	goto out;
error_unregister:
	device_destroy(mc_device_class, mc_dev_admin);
	device_destroy(mc_device_class, mc_dev_user);

	cdev_del(&mc_admin_cdev);
	cdev_del(&mc_user_cdev);
	unregister_chrdev_region(mc_dev_admin, MC_DEV_MAX);
error:
	class_destroy(mc_device_class);
out:
	return ret;
}

/*
 * This function is called the kernel during startup or by a insmod command.
 * This device is installed and registered as cdev, then interrupt and
 * queue handling is set up
 */
static int __init mobicore_init(void)
{
	int ret = 0;
	dev_set_name(mcd, "mcd");

	dev_info(mcd, "MobiCore Driver, Build: " __TIMESTAMP__ "\n");
	dev_info(mcd, "MobiCore mcDrvModuleApi version is %i.%i\n",
		 MCDRVMODULEAPI_VERSION_MAJOR,
		 MCDRVMODULEAPI_VERSION_MINOR);
#ifdef MOBICORE_COMPONENT_BUILD_TAG
	dev_info(mcd, "MobiCore %s\n", MOBICORE_COMPONENT_BUILD_TAG);
#endif
	/* Hardware does not support ARM TrustZone -> Cannot continue! */
	if (!has_security_extensions()) {
		MCDRV_DBG_ERROR(mcd,
				"Hardware doesn't support ARM TrustZone!");
		return -ENODEV;
	}

	/* Running in secure mode -> Cannot load the driver! */
	if (is_secure_mode()) {
		MCDRV_DBG_ERROR(mcd, "Running in secure MODE!");
		return -ENODEV;
	}

	ret = mc_fastcall_init(&ctx);
	if (ret)
		goto error;

	init_completion(&ctx.isr_comp);

	/* initialize event counter for signaling of an IRQ to zero */
	atomic_set(&ctx.isr_counter, 0);

	/* set up S-SIQ interrupt handler ************************/
	ret = request_irq(MC_INTR_SSIQ, mc_ssiq_isr, IRQF_TRIGGER_RISING,
			MC_ADMIN_DEVNODE, &ctx);
	if (ret != 0) {
		MCDRV_DBG_ERROR(mcd, "interrupt request failed");
		goto err_req_irq;
	}

#ifdef MC_PM_RUNTIME
	ret = mc_pm_initialize(&ctx);
	if (ret != 0) {
		MCDRV_DBG_ERROR(mcd, "Power Management init failed!");
		goto free_isr;
	}
#endif

	ret = create_devices();
	if (ret != 0)
		goto free_pm;

	ret = mc_init_mmu_tables();

#ifdef MC_CRYPTO_CLOCK_MANAGEMENT
	ret = mc_pm_clock_initialize();
#endif

	/*
	 * initialize unique number counter which we can use for
	 * handles. It is limited to 2^32, but this should be
	 * enough to be roll-over safe for us. We start with 1
	 * instead of 0.
	 */
	atomic_set(&ctx.unique_counter, 1);

	/* init list for contiguous buffers  */
	INIT_LIST_HEAD(&ctx.cont_bufs);

	/* init lock for the buffers list */
	mutex_init(&ctx.bufs_lock);

	memset(&ctx.mci_base, 0, sizeof(ctx.mci_base));
	MCDRV_DBG(mcd, "initialized");
	return 0;

free_pm:
#ifdef MC_PM_RUNTIME
	mc_pm_free();
free_isr:
	free_irq(MC_INTR_SSIQ, &ctx);
#endif
err_req_irq:
	mc_fastcall_destroy();
error:
	return ret;
}

/*
 * This function removes this device driver from the Linux device manager .
 */
static void __exit mobicore_exit(void)
{
	MCDRV_DBG_VERBOSE(mcd, "enter");
#ifdef MC_MEM_TRACES
	mobicore_log_free();
#endif

	mc_release_mmu_tables();

#ifdef MC_PM_RUNTIME
	mc_pm_free();
#endif

	device_destroy(mc_device_class, mc_dev_admin);
	device_destroy(mc_device_class, mc_dev_user);
	class_destroy(mc_device_class);
	unregister_chrdev_region(mc_dev_admin, MC_DEV_MAX);

	free_irq(MC_INTR_SSIQ, &ctx);

	mc_fastcall_destroy();

#ifdef MC_CRYPTO_CLOCK_MANAGEMENT
	mc_pm_clock_finalize();
#endif

	MCDRV_DBG_VERBOSE(mcd, "exit");
}

bool mc_sleep_ready(void)
{
#ifdef MC_PM_RUNTIME
	return mc_pm_sleep_ready();
#else
	return true;
#endif
}

/* Linux Driver Module Macros */
module_init(mobicore_init);
module_exit(mobicore_exit);
MODULE_AUTHOR("Giesecke & Devrient GmbH");
MODULE_AUTHOR("Trustonic Limited");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MobiCore driver");
