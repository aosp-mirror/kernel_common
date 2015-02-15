/* mm/ashmem.c
**
** Anonymous Shared Memory Subsystem, ashmem
**
** Copyright (C) 2008 Google, Inc.
**
** Robert Love <rlove@google.com>
**
** This software is licensed under the terms of the GNU General Public
** License version 2, as published by the Free Software Foundation, and
** may be copied, distributed, and modified under those terms.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
*/

#include <linux/module.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/security.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/uaccess.h>
#include <linux/personality.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/shmem_fs.h>
#include <linux/ashmem.h>
#include <asm/cacheflush.h>

#define ASHMEM_NAME_PREFIX "dev/ashmem/"
#define ASHMEM_NAME_PREFIX_LEN (sizeof(ASHMEM_NAME_PREFIX) - 1)
#define ASHMEM_FULL_NAME_LEN (ASHMEM_NAME_LEN + ASHMEM_NAME_PREFIX_LEN)

struct ashmem_area {
	char name[ASHMEM_FULL_NAME_LEN]; 
	struct list_head unpinned_list;	 
	struct file *file;		 
	size_t size;			 
	unsigned long vm_start;		 
	unsigned long prot_mask;	 
};

struct ashmem_range {
	struct list_head lru;		
	struct list_head unpinned;	
	struct ashmem_area *asma;	
	size_t pgstart;			
	size_t pgend;			
	unsigned int purged;		
};

static LIST_HEAD(ashmem_lru_list);

static unsigned long lru_count;

static DEFINE_MUTEX(ashmem_mutex);

static struct kmem_cache *ashmem_area_cachep __read_mostly;
static struct kmem_cache *ashmem_range_cachep __read_mostly;

#define range_size(range) \
	((range)->pgend - (range)->pgstart + 1)

#define range_on_lru(range) \
	((range)->purged == ASHMEM_NOT_PURGED)

#define page_range_subsumes_range(range, start, end) \
	(((range)->pgstart >= (start)) && ((range)->pgend <= (end)))

#define page_range_subsumed_by_range(range, start, end) \
	(((range)->pgstart <= (start)) && ((range)->pgend >= (end)))

#define page_in_range(range, page) \
	(((range)->pgstart <= (page)) && ((range)->pgend >= (page)))

#define page_range_in_range(range, start, end) \
	(page_in_range(range, start) || page_in_range(range, end) || \
		page_range_subsumes_range(range, start, end))

#define range_before_page(range, page) \
	((range)->pgend < (page))

#define PROT_MASK		(PROT_EXEC | PROT_READ | PROT_WRITE)

static inline void lru_add(struct ashmem_range *range)
{
	list_add_tail(&range->lru, &ashmem_lru_list);
	lru_count += range_size(range);
}

static inline void lru_del(struct ashmem_range *range)
{
	list_del(&range->lru);
	lru_count -= range_size(range);
}

static int range_alloc(struct ashmem_area *asma,
		       struct ashmem_range *prev_range, unsigned int purged,
		       size_t start, size_t end)
{
	struct ashmem_range *range;

	range = kmem_cache_zalloc(ashmem_range_cachep, GFP_KERNEL);
	if (unlikely(!range)) {
		printk(KERN_INFO "ashmem: %s failed on %s(%d)\n", __func__, current->comm, current->pid);
		show_mem(SHOW_MEM_FILTER_NODES);
		dump_stack();
		return -ENOMEM;
	}

	range->asma = asma;
	range->pgstart = start;
	range->pgend = end;
	range->purged = purged;

	list_add_tail(&range->unpinned, &prev_range->unpinned);

	if (range_on_lru(range))
		lru_add(range);

	return 0;
}

static void range_del(struct ashmem_range *range)
{
	list_del(&range->unpinned);
	if (range_on_lru(range))
		lru_del(range);
	kmem_cache_free(ashmem_range_cachep, range);
}

static inline void range_shrink(struct ashmem_range *range,
				size_t start, size_t end)
{
	size_t pre = range_size(range);

	range->pgstart = start;
	range->pgend = end;

	if (range_on_lru(range))
		lru_count -= pre - range_size(range);
}

static int ashmem_open(struct inode *inode, struct file *file)
{
	struct ashmem_area *asma;
	int ret;

	ret = generic_file_open(inode, file);
	if (unlikely(ret))
		return ret;

	asma = kmem_cache_zalloc(ashmem_area_cachep, GFP_KERNEL);
	if (unlikely(!asma)) {
		printk(KERN_INFO "ashmem: %s failed on %s(%d)\n", __func__, current->comm, current->pid);
		show_mem(SHOW_MEM_FILTER_NODES);
		dump_stack();
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&asma->unpinned_list);
	memcpy(asma->name, ASHMEM_NAME_PREFIX, ASHMEM_NAME_PREFIX_LEN);
	asma->prot_mask = PROT_MASK;
	file->private_data = asma;

	return 0;
}

static int ashmem_release(struct inode *ignored, struct file *file)
{
	struct ashmem_area *asma = file->private_data;
	struct ashmem_range *range, *next;

	mutex_lock(&ashmem_mutex);
	list_for_each_entry_safe(range, next, &asma->unpinned_list, unpinned)
		range_del(range);
	mutex_unlock(&ashmem_mutex);

	if (asma->file)
		fput(asma->file);
	kmem_cache_free(ashmem_area_cachep, asma);

	return 0;
}

static ssize_t ashmem_read(struct file *file, char __user *buf,
			   size_t len, loff_t *pos)
{
	struct ashmem_area *asma = file->private_data;
	int ret = 0;

	mutex_lock(&ashmem_mutex);

	
	if (asma->size == 0)
		goto out_unlock;

	if (!asma->file) {
		ret = -EBADF;
		goto out_unlock;
	}

	mutex_unlock(&ashmem_mutex);

	ret = asma->file->f_op->read(asma->file, buf, len, pos);
	if (ret >= 0) {
		
		asma->file->f_pos = *pos;
	}
	return ret;

out_unlock:
	mutex_unlock(&ashmem_mutex);
	return ret;
}

static loff_t ashmem_llseek(struct file *file, loff_t offset, int origin)
{
	struct ashmem_area *asma = file->private_data;
	int ret;

	mutex_lock(&ashmem_mutex);

	if (asma->size == 0) {
		ret = -EINVAL;
		goto out;
	}

	if (!asma->file) {
		ret = -EBADF;
		goto out;
	}

	ret = asma->file->f_op->llseek(asma->file, offset, origin);
	if (ret < 0)
		goto out;

	
	file->f_pos = asma->file->f_pos;

out:
	mutex_unlock(&ashmem_mutex);
	return ret;
}

static inline unsigned long calc_vm_may_flags(unsigned long prot)
{
	return _calc_vm_trans(prot, PROT_READ,  VM_MAYREAD) |
	       _calc_vm_trans(prot, PROT_WRITE, VM_MAYWRITE) |
	       _calc_vm_trans(prot, PROT_EXEC,  VM_MAYEXEC);
}

static int ashmem_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct ashmem_area *asma = file->private_data;
	int ret = 0;

	mutex_lock(&ashmem_mutex);

	
	if (unlikely(!asma->size)) {
		ret = -EINVAL;
		goto out;
	}

	
	if (unlikely((vma->vm_flags & ~calc_vm_prot_bits(asma->prot_mask)) &
		     calc_vm_prot_bits(PROT_MASK))) {
		ret = -EPERM;
		goto out;
	}
	vma->vm_flags &= ~calc_vm_may_flags(~asma->prot_mask);

	if (!asma->file) {
		char *name = ASHMEM_NAME_DEF;
		struct file *vmfile;

		if (asma->name[ASHMEM_NAME_PREFIX_LEN] != '\0')
			name = asma->name;

		
		vmfile = shmem_file_setup(name, asma->size, vma->vm_flags);
		if (unlikely(IS_ERR(vmfile))) {
			ret = PTR_ERR(vmfile);
			goto out;
		}
		asma->file = vmfile;
	}
	get_file(asma->file);

	if (vma->vm_flags & VM_SHARED)
		shmem_set_file(vma, asma->file);
	else {
		if (vma->vm_file)
			fput(vma->vm_file);
		vma->vm_file = asma->file;
	}
	vma->vm_flags |= VM_CAN_NONLINEAR;
	asma->vm_start = vma->vm_start;

out:
	mutex_unlock(&ashmem_mutex);
	return ret;
}

static int ashmem_shrink(struct shrinker *s, struct shrink_control *sc)
{
	struct ashmem_range *range, *next;

	
	if (sc->nr_to_scan && !(sc->gfp_mask & __GFP_FS))
		return -1;
	if (!sc->nr_to_scan)
		return lru_count;

	if (!mutex_trylock(&ashmem_mutex))
		return -1;

	list_for_each_entry_safe(range, next, &ashmem_lru_list, lru) {
		struct inode *inode = range->asma->file->f_dentry->d_inode;
		loff_t start = range->pgstart * PAGE_SIZE;
		loff_t end = (range->pgend + 1) * PAGE_SIZE - 1;

		vmtruncate_range(inode, start, end);
		range->purged = ASHMEM_WAS_PURGED;
		lru_del(range);

		sc->nr_to_scan -= range_size(range);
		if (sc->nr_to_scan <= 0)
			break;
	}
	mutex_unlock(&ashmem_mutex);

	return lru_count;
}

static struct shrinker ashmem_shrinker = {
	.shrink = ashmem_shrink,
	.seeks = DEFAULT_SEEKS * 4,
};

static int set_prot_mask(struct ashmem_area *asma, unsigned long prot)
{
	int ret = 0;

	mutex_lock(&ashmem_mutex);

	
	if (unlikely((asma->prot_mask & prot) != prot)) {
		ret = -EINVAL;
		goto out;
	}

	
	if ((prot & PROT_READ) && (current->personality & READ_IMPLIES_EXEC))
		prot |= PROT_EXEC;

	asma->prot_mask = prot;

out:
	mutex_unlock(&ashmem_mutex);
	return ret;
}

static int set_name(struct ashmem_area *asma, void __user *name)
{
	int len;
	int ret = 0;
	char local_name[ASHMEM_NAME_LEN];

	len = strncpy_from_user(local_name, name, ASHMEM_NAME_LEN);
	if (len < 0)
		return len;
	if (len == ASHMEM_NAME_LEN)
		local_name[ASHMEM_NAME_LEN - 1] = '\0';
	mutex_lock(&ashmem_mutex);
	
	if (unlikely(asma->file))
		ret = -EINVAL;
	else
		strcpy(asma->name + ASHMEM_NAME_PREFIX_LEN, local_name);

	mutex_unlock(&ashmem_mutex);
	return ret;
}

static int get_name(struct ashmem_area *asma, void __user *name)
{
	int ret = 0;
	size_t len;
	char local_name[ASHMEM_NAME_LEN];

	mutex_lock(&ashmem_mutex);
	if (asma->name[ASHMEM_NAME_PREFIX_LEN] != '\0') {

		len = strlen(asma->name + ASHMEM_NAME_PREFIX_LEN) + 1;
		memcpy(local_name, asma->name + ASHMEM_NAME_PREFIX_LEN, len);
	} else {
		len = sizeof(ASHMEM_NAME_DEF);
		memcpy(local_name, ASHMEM_NAME_DEF, len);
	}
	mutex_unlock(&ashmem_mutex);

	if (unlikely(copy_to_user(name, local_name, len)))
		ret = -EFAULT;
	return ret;
}

static int ashmem_pin(struct ashmem_area *asma, size_t pgstart, size_t pgend)
{
	struct ashmem_range *range, *next;
	int ret = ASHMEM_NOT_PURGED;

	list_for_each_entry_safe(range, next, &asma->unpinned_list, unpinned) {
		
		if (range_before_page(range, pgstart))
			break;

		if (page_range_in_range(range, pgstart, pgend)) {
			ret |= range->purged;

			
			if (page_range_subsumes_range(range, pgstart, pgend)) {
				range_del(range);
				continue;
			}

			
			if (range->pgstart >= pgstart) {
				range_shrink(range, pgend + 1, range->pgend);
				continue;
			}

			
			if (range->pgend <= pgend) {
				range_shrink(range, range->pgstart, pgstart-1);
				continue;
			}

			range_alloc(asma, range, range->purged,
				    pgend + 1, range->pgend);
			range_shrink(range, range->pgstart, pgstart - 1);
			break;
		}
	}

	return ret;
}

static int ashmem_unpin(struct ashmem_area *asma, size_t pgstart, size_t pgend)
{
	struct ashmem_range *range, *next;
	unsigned int purged = ASHMEM_NOT_PURGED;

restart:
	list_for_each_entry_safe(range, next, &asma->unpinned_list, unpinned) {
		
		if (range_before_page(range, pgstart))
			break;

		if (page_range_subsumed_by_range(range, pgstart, pgend))
			return 0;
		if (page_range_in_range(range, pgstart, pgend)) {
			pgstart = min_t(size_t, range->pgstart, pgstart),
			pgend = max_t(size_t, range->pgend, pgend);
			purged |= range->purged;
			range_del(range);
			goto restart;
		}
	}

	return range_alloc(asma, range, purged, pgstart, pgend);
}

static int ashmem_get_pin_status(struct ashmem_area *asma, size_t pgstart,
				 size_t pgend)
{
	struct ashmem_range *range;
	int ret = ASHMEM_IS_PINNED;

	list_for_each_entry(range, &asma->unpinned_list, unpinned) {
		if (range_before_page(range, pgstart))
			break;
		if (page_range_in_range(range, pgstart, pgend)) {
			ret = ASHMEM_IS_UNPINNED;
			break;
		}
	}

	return ret;
}

static int ashmem_pin_unpin(struct ashmem_area *asma, unsigned long cmd,
			    void __user *p)
{
	struct ashmem_pin pin;
	size_t pgstart, pgend;
	int ret = -EINVAL;

	if (unlikely(!asma->file))
		return -EINVAL;

	if (unlikely(copy_from_user(&pin, p, sizeof(pin))))
		return -EFAULT;

	
	if (!pin.len)
		pin.len = PAGE_ALIGN(asma->size) - pin.offset;

	if (unlikely((pin.offset | pin.len) & ~PAGE_MASK))
		return -EINVAL;

	if (unlikely(((__u32) -1) - pin.offset < pin.len))
		return -EINVAL;

	if (unlikely(PAGE_ALIGN(asma->size) < pin.offset + pin.len))
		return -EINVAL;

	pgstart = pin.offset / PAGE_SIZE;
	pgend = pgstart + (pin.len / PAGE_SIZE) - 1;

	mutex_lock(&ashmem_mutex);

	switch (cmd) {
	case ASHMEM_PIN:
		ret = ashmem_pin(asma, pgstart, pgend);
		break;
	case ASHMEM_UNPIN:
		ret = ashmem_unpin(asma, pgstart, pgend);
		break;
	case ASHMEM_GET_PIN_STATUS:
		ret = ashmem_get_pin_status(asma, pgstart, pgend);
		break;
	}

	mutex_unlock(&ashmem_mutex);

	return ret;
}

#ifdef CONFIG_OUTER_CACHE
static unsigned int virtaddr_to_physaddr(unsigned int virtaddr)
{
	unsigned int physaddr = 0;
	pgd_t *pgd_ptr = NULL;
	pud_t *pud_ptr = NULL;
	pmd_t *pmd_ptr = NULL;
	pte_t *pte_ptr = NULL, pte;

	spin_lock(&current->mm->page_table_lock);
	pgd_ptr = pgd_offset(current->mm, virtaddr);
	if (pgd_none(*pgd_ptr) || pgd_bad(*pgd_ptr)) {
		pr_err("Failed to convert virtaddr %x to pgd_ptr\n",
			virtaddr);
		goto done;
	}

	pud_ptr = pud_offset(pgd_ptr, virtaddr);
	if (pud_none(*pud_ptr) || pud_bad(*pud_ptr)) {
		pr_err("Failed to convert pgd_ptr %p to pud_ptr\n",
			(void *)pgd_ptr);
		goto done;
	}

	pmd_ptr = pmd_offset(pud_ptr, virtaddr);
	if (pmd_none(*pmd_ptr) || pmd_bad(*pmd_ptr)) {
		pr_err("Failed to convert pud_ptr %p to pmd_ptr\n",
			(void *)pud_ptr);
		goto done;
	}

	pte_ptr = pte_offset_map(pmd_ptr, virtaddr);
	if (!pte_ptr) {
		pr_err("Failed to convert pmd_ptr %p to pte_ptr\n",
			(void *)pmd_ptr);
		goto done;
	}
	pte = *pte_ptr;
	physaddr = pte_pfn(pte);
	pte_unmap(pte_ptr);
done:
	spin_unlock(&current->mm->page_table_lock);
	physaddr <<= PAGE_SHIFT;
	return physaddr;
}
#endif

static int ashmem_cache_op(struct ashmem_area *asma,
	void (*cache_func)(unsigned long vstart, unsigned long length,
				unsigned long pstart))
{
	int ret = 0;
	struct vm_area_struct *vma;
#ifdef CONFIG_OUTER_CACHE
	unsigned long vaddr;
#endif
	if (!asma->vm_start)
		return -EINVAL;

	down_read(&current->mm->mmap_sem);
	vma = find_vma(current->mm, asma->vm_start);
	if (!vma) {
		ret = -EINVAL;
		goto done;
	}
	if (vma->vm_file != asma->file) {
		ret = -EINVAL;
		goto done;
	}
	if ((asma->vm_start + asma->size) > vma->vm_end) {
		ret = -EINVAL;
		goto done;
	}
#ifndef CONFIG_OUTER_CACHE
	cache_func(asma->vm_start, asma->size, 0);
#else
	for (vaddr = asma->vm_start; vaddr < asma->vm_start + asma->size;
		vaddr += PAGE_SIZE) {
		unsigned long physaddr;
		physaddr = virtaddr_to_physaddr(vaddr);
		if (!physaddr)
			return -EINVAL;
		cache_func(vaddr, PAGE_SIZE, physaddr);
	}
#endif
done:
	up_read(&current->mm->mmap_sem);
	if (ret)
		asma->vm_start = 0;
	return ret;
}

static long ashmem_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ashmem_area *asma = file->private_data;
	long ret = -ENOTTY;

	switch (cmd) {
	case ASHMEM_SET_NAME:
		ret = set_name(asma, (void __user *) arg);
		break;
	case ASHMEM_GET_NAME:
		ret = get_name(asma, (void __user *) arg);
		break;
	case ASHMEM_SET_SIZE:
		ret = -EINVAL;
		if (!asma->file) {
			ret = 0;
			asma->size = (size_t) arg;
		}
		break;
	case ASHMEM_GET_SIZE:
		ret = asma->size;
		break;
	case ASHMEM_SET_PROT_MASK:
		ret = set_prot_mask(asma, arg);
		break;
	case ASHMEM_GET_PROT_MASK:
		ret = asma->prot_mask;
		break;
	case ASHMEM_PIN:
	case ASHMEM_UNPIN:
	case ASHMEM_GET_PIN_STATUS:
		ret = ashmem_pin_unpin(asma, cmd, (void __user *) arg);
		break;
	case ASHMEM_PURGE_ALL_CACHES:
		ret = -EPERM;
		if (capable(CAP_SYS_ADMIN)) {
			struct shrink_control sc = {
				.gfp_mask = GFP_KERNEL,
				.nr_to_scan = 0,
			};
			ret = ashmem_shrink(&ashmem_shrinker, &sc);
			sc.nr_to_scan = ret;
			ashmem_shrink(&ashmem_shrinker, &sc);
		}
		break;
	case ASHMEM_CACHE_FLUSH_RANGE:
		ret = ashmem_cache_op(asma, &clean_and_invalidate_caches);
		break;
	case ASHMEM_CACHE_CLEAN_RANGE:
		ret = ashmem_cache_op(asma, &clean_caches);
		break;
	case ASHMEM_CACHE_INV_RANGE:
		ret = ashmem_cache_op(asma, &invalidate_caches);
		break;
	}

	return ret;
}

static int is_ashmem_file(struct file *file)
{
	char fname[256], *name;
	name = dentry_path(file->f_dentry, fname, 256);
	return strcmp(name, "/ashmem") ? 0 : 1;
}

int get_ashmem_file(int fd, struct file **filp, struct file **vm_file,
			unsigned long *len)
{
	int ret = -1;
	struct file *file = fget(fd);
	*filp = NULL;
	*vm_file = NULL;
	if (unlikely(file == NULL)) {
		pr_err("ashmem: %s: requested data from file "
			"descriptor that doesn't exist.\n", __func__);
	} else {
		char currtask_name[FIELD_SIZEOF(struct task_struct, comm) + 1];
		pr_debug("filp %p rdev %d pid %u(%s) file %p(%ld)"
			" dev id: %d\n", filp,
			file->f_dentry->d_inode->i_rdev,
			current->pid, get_task_comm(currtask_name, current),
			file, file_count(file),
			MINOR(file->f_dentry->d_inode->i_rdev));
		if (is_ashmem_file(file)) {
			struct ashmem_area *asma = file->private_data;
			*filp = file;
			*vm_file = asma->file;
			*len = asma->size;
			ret = 0;
		} else {
			pr_err("file descriptor is not an ashmem "
				"region fd: %d\n", fd);
			fput(file);
		}
	}
	return ret;
}
EXPORT_SYMBOL(get_ashmem_file);

void put_ashmem_file(struct file *file)
{
	char currtask_name[FIELD_SIZEOF(struct task_struct, comm) + 1];
	pr_debug("rdev %d pid %u(%s) file %p(%ld)" " dev id: %d\n",
		file->f_dentry->d_inode->i_rdev, current->pid,
		get_task_comm(currtask_name, current), file,
		file_count(file), MINOR(file->f_dentry->d_inode->i_rdev));
	if (file && is_ashmem_file(file))
		fput(file);
}
EXPORT_SYMBOL(put_ashmem_file);

static const struct file_operations ashmem_fops = {
	.owner = THIS_MODULE,
	.open = ashmem_open,
	.release = ashmem_release,
	.read = ashmem_read,
	.llseek = ashmem_llseek,
	.mmap = ashmem_mmap,
	.unlocked_ioctl = ashmem_ioctl,
	.compat_ioctl = ashmem_ioctl,
};

static struct miscdevice ashmem_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ashmem",
	.fops = &ashmem_fops,
};

static int __init ashmem_init(void)
{
	int ret;

	ashmem_area_cachep = kmem_cache_create("ashmem_area_cache",
					  sizeof(struct ashmem_area),
					  0, 0, NULL);
	if (unlikely(!ashmem_area_cachep)) {
		printk(KERN_ERR "ashmem: failed to create slab cache\n");
		return -ENOMEM;
	}

	ashmem_range_cachep = kmem_cache_create("ashmem_range_cache",
					  sizeof(struct ashmem_range),
					  0, 0, NULL);
	if (unlikely(!ashmem_range_cachep)) {
		printk(KERN_ERR "ashmem: failed to create slab cache\n");
		return -ENOMEM;
	}

	ret = misc_register(&ashmem_misc);
	if (unlikely(ret)) {
		printk(KERN_ERR "ashmem: failed to register misc device!\n");
		return ret;
	}

	register_shrinker(&ashmem_shrinker);

	printk(KERN_INFO "ashmem: initialized\n");

	return 0;
}

static void __exit ashmem_exit(void)
{
	int ret;

	unregister_shrinker(&ashmem_shrinker);

	ret = misc_deregister(&ashmem_misc);
	if (unlikely(ret))
		printk(KERN_ERR "ashmem: failed to unregister misc device!\n");

	kmem_cache_destroy(ashmem_range_cachep);
	kmem_cache_destroy(ashmem_area_cachep);

	printk(KERN_INFO "ashmem: unloaded\n");
}

module_init(ashmem_init);
module_exit(ashmem_exit);

MODULE_LICENSE("GPL");
