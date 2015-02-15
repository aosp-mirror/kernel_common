/*
 *  linux/fs/buffer.c
 *
 *  Copyright (C) 1991, 1992, 2002  Linus Torvalds
 */


#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/percpu.h>
#include <linux/slab.h>
#include <linux/capability.h>
#include <linux/blkdev.h>
#include <linux/file.h>
#include <linux/quotaops.h>
#include <linux/highmem.h>
#include <linux/export.h>
#include <linux/writeback.h>
#include <linux/hash.h>
#include <linux/suspend.h>
#include <linux/buffer_head.h>
#include <linux/task_io_accounting_ops.h>
#include <linux/bio.h>
#include <linux/notifier.h>
#include <linux/cpu.h>
#include <linux/bitops.h>
#include <linux/mpage.h>
#include <linux/bit_spinlock.h>

static int fsync_buffers_list(spinlock_t *lock, struct list_head *list);

#define BH_ENTRY(list) list_entry((list), struct buffer_head, b_assoc_buffers)

inline void
init_buffer(struct buffer_head *bh, bh_end_io_t *handler, void *private)
{
	bh->b_end_io = handler;
	bh->b_private = private;
}
EXPORT_SYMBOL(init_buffer);

static int sleep_on_buffer(void *word)
{
	io_schedule();
	return 0;
}

void __lock_buffer(struct buffer_head *bh)
{
	wait_on_bit_lock(&bh->b_state, BH_Lock, sleep_on_buffer,
							TASK_UNINTERRUPTIBLE);
}
EXPORT_SYMBOL(__lock_buffer);

void unlock_buffer(struct buffer_head *bh)
{
	clear_bit_unlock(BH_Lock, &bh->b_state);
	smp_mb__after_clear_bit();
	wake_up_bit(&bh->b_state, BH_Lock);
}
EXPORT_SYMBOL(unlock_buffer);

void __wait_on_buffer(struct buffer_head * bh)
{
	wait_on_bit(&bh->b_state, BH_Lock, sleep_on_buffer, TASK_UNINTERRUPTIBLE);
}
EXPORT_SYMBOL(__wait_on_buffer);

static void
__clear_page_buffers(struct page *page)
{
	ClearPagePrivate(page);
	set_page_private(page, 0);
	page_cache_release(page);
}


static int quiet_error(struct buffer_head *bh)
{
	if (!test_bit(BH_Quiet, &bh->b_state) && printk_ratelimit())
		return 0;
	return 1;
}


static void buffer_io_error(struct buffer_head *bh)
{
	char b[BDEVNAME_SIZE];
	printk(KERN_ERR "Buffer I/O error on device %s, logical block %Lu\n",
			bdevname(bh->b_bdev, b),
			(unsigned long long)bh->b_blocknr);
}

static void __end_buffer_read_notouch(struct buffer_head *bh, int uptodate)
{
	if (uptodate) {
		set_buffer_uptodate(bh);
	} else {
		
		clear_buffer_uptodate(bh);
	}
	unlock_buffer(bh);
}

void end_buffer_read_sync(struct buffer_head *bh, int uptodate)
{
	__end_buffer_read_notouch(bh, uptodate);
	put_bh(bh);
}
EXPORT_SYMBOL(end_buffer_read_sync);

void end_buffer_write_sync(struct buffer_head *bh, int uptodate)
{
	char b[BDEVNAME_SIZE];

	if (uptodate) {
		set_buffer_uptodate(bh);
	} else {
		if (!quiet_error(bh)) {
			buffer_io_error(bh);
			printk(KERN_WARNING "lost page write due to "
					"I/O error on %s\n",
				       bdevname(bh->b_bdev, b));
		}
		set_buffer_write_io_error(bh);
		clear_buffer_uptodate(bh);
	}
	unlock_buffer(bh);
	put_bh(bh);
}
EXPORT_SYMBOL(end_buffer_write_sync);

static struct buffer_head *
__find_get_block_slow(struct block_device *bdev, sector_t block)
{
	struct inode *bd_inode = bdev->bd_inode;
	struct address_space *bd_mapping = bd_inode->i_mapping;
	struct buffer_head *ret = NULL;
	pgoff_t index;
	struct buffer_head *bh;
	struct buffer_head *head;
	struct page *page;
	int all_mapped = 1;

	index = block >> (PAGE_CACHE_SHIFT - bd_inode->i_blkbits);
	page = find_get_page(bd_mapping, index);
	if (!page)
		goto out;

	spin_lock(&bd_mapping->private_lock);
	if (!page_has_buffers(page))
		goto out_unlock;
	head = page_buffers(page);
	bh = head;
	do {
		if (!buffer_mapped(bh))
			all_mapped = 0;
		else if (bh->b_blocknr == block) {
			ret = bh;
			get_bh(bh);
			goto out_unlock;
		}
		bh = bh->b_this_page;
	} while (bh != head);

	if (all_mapped) {
		char b[BDEVNAME_SIZE];

		printk("__find_get_block_slow() failed. "
			"block=%llu, b_blocknr=%llu\n",
			(unsigned long long)block,
			(unsigned long long)bh->b_blocknr);
		printk("b_state=0x%08lx, b_size=%zu\n",
			bh->b_state, bh->b_size);
		printk("device %s blocksize: %d\n", bdevname(bdev, b),
			1 << bd_inode->i_blkbits);
	}
out_unlock:
	spin_unlock(&bd_mapping->private_lock);
	page_cache_release(page);
out:
	return ret;
}

static void free_more_memory(void)
{
	struct zone *zone;
	int nid;

	wakeup_flusher_threads(1024, WB_REASON_FREE_MORE_MEM);
	yield();

	for_each_online_node(nid) {
		(void)first_zones_zonelist(node_zonelist(nid, GFP_NOFS),
						gfp_zone(GFP_NOFS), NULL,
						&zone);
		if (zone)
			try_to_free_pages(node_zonelist(nid, GFP_NOFS), 0,
						GFP_NOFS, NULL);
	}
}

static void end_buffer_async_read(struct buffer_head *bh, int uptodate)
{
	unsigned long flags;
	struct buffer_head *first;
	struct buffer_head *tmp;
	struct page *page;
	int page_uptodate = 1;

	BUG_ON(!buffer_async_read(bh));

	page = bh->b_page;
	if (!page_has_buffers(page)) {
		WARN(1, "%s: page %p has no buffers\n", __func__, page);
		return;
	}
	if (uptodate) {
		set_buffer_uptodate(bh);
	} else {
		clear_buffer_uptodate(bh);
		if (!quiet_error(bh))
			buffer_io_error(bh);
		SetPageError(page);
	}

	first = page_buffers(page);
	local_irq_save(flags);
	bit_spin_lock(BH_Uptodate_Lock, &first->b_state);
	clear_buffer_async_read(bh);
	unlock_buffer(bh);
	tmp = bh;
	do {
		if (!buffer_uptodate(tmp))
			page_uptodate = 0;
		if (buffer_async_read(tmp)) {
			BUG_ON(!buffer_locked(tmp));
			goto still_busy;
		}
		tmp = tmp->b_this_page;
	} while (tmp != bh);
	bit_spin_unlock(BH_Uptodate_Lock, &first->b_state);
	local_irq_restore(flags);

	if (page_uptodate && !PageError(page))
		SetPageUptodate(page);
	unlock_page(page);
	return;

still_busy:
	bit_spin_unlock(BH_Uptodate_Lock, &first->b_state);
	local_irq_restore(flags);
	return;
}

void end_buffer_async_write(struct buffer_head *bh, int uptodate)
{
	char b[BDEVNAME_SIZE];
	unsigned long flags;
	struct buffer_head *first;
	struct buffer_head *tmp;
	struct page *page;

	BUG_ON(!buffer_async_write(bh));

	page = bh->b_page;
	if (uptodate) {
		set_buffer_uptodate(bh);
	} else {
		if (!quiet_error(bh)) {
			buffer_io_error(bh);
			printk(KERN_WARNING "lost page write due to "
					"I/O error on %s\n",
			       bdevname(bh->b_bdev, b));
		}
		set_bit(AS_EIO, &page->mapping->flags);
		set_buffer_write_io_error(bh);
		clear_buffer_uptodate(bh);
		SetPageError(page);
	}

	first = page_buffers(page);
	local_irq_save(flags);
	bit_spin_lock(BH_Uptodate_Lock, &first->b_state);

	clear_buffer_async_write(bh);
	unlock_buffer(bh);
	tmp = bh->b_this_page;
	while (tmp != bh) {
		if (buffer_async_write(tmp)) {
			BUG_ON(!buffer_locked(tmp));
			goto still_busy;
		}
		tmp = tmp->b_this_page;
	}
	bit_spin_unlock(BH_Uptodate_Lock, &first->b_state);
	local_irq_restore(flags);
	end_page_writeback(page);
	return;

still_busy:
	bit_spin_unlock(BH_Uptodate_Lock, &first->b_state);
	local_irq_restore(flags);
	return;
}
EXPORT_SYMBOL(end_buffer_async_write);

static void mark_buffer_async_read(struct buffer_head *bh)
{
	bh->b_end_io = end_buffer_async_read;
	set_buffer_async_read(bh);
}

static void mark_buffer_async_write_endio(struct buffer_head *bh,
					  bh_end_io_t *handler)
{
	bh->b_end_io = handler;
	set_buffer_async_write(bh);
}

void mark_buffer_async_write(struct buffer_head *bh)
{
	mark_buffer_async_write_endio(bh, end_buffer_async_write);
}
EXPORT_SYMBOL(mark_buffer_async_write);


/*
 * fs/buffer.c contains helper functions for buffer-backed address space's
 * fsync functions.  A common requirement for buffer-based filesystems is
 * that certain data from the backing blockdev needs to be written out for
 * a successful fsync().  For example, ext2 indirect blocks need to be
 * written back and waited upon before fsync() returns.
 *
 * The functions mark_buffer_inode_dirty(), fsync_inode_buffers(),
 * inode_has_buffers() and invalidate_inode_buffers() are provided for the
 * management of a list of dependent buffers at ->i_mapping->private_list.
 *
 * Locking is a little subtle: try_to_free_buffers() will remove buffers
 * from their controlling inode's queue when they are being freed.  But
 * try_to_free_buffers() will be operating against the *blockdev* mapping
 * at the time, not against the S_ISREG file which depends on those buffers.
 * So the locking for private_list is via the private_lock in the address_space
 * which backs the buffers.  Which is different from the address_space 
 * against which the buffers are listed.  So for a particular address_space,
 * mapping->private_lock does *not* protect mapping->private_list!  In fact,
 * mapping->private_list will always be protected by the backing blockdev's
 * ->private_lock.
 *
 * Which introduces a requirement: all buffers on an address_space's
 * ->private_list must be from the same address_space: the blockdev's.
 *
 * address_spaces which do not place buffers at ->private_list via these
 * utility functions are free to use private_lock and private_list for
 * whatever they want.  The only requirement is that list_empty(private_list)
 * be true at clear_inode() time.
 *
 * FIXME: clear_inode should not call invalidate_inode_buffers().  The
 * filesystems should do that.  invalidate_inode_buffers() should just go
 * BUG_ON(!list_empty).
 *
 * FIXME: mark_buffer_dirty_inode() is a data-plane operation.  It should
 * take an address_space, not an inode.  And it should be called
 * mark_buffer_dirty_fsync() to clearly define why those buffers are being
 * queued up.
 *
 * FIXME: mark_buffer_dirty_inode() doesn't need to add the buffer to the
 * list if it is already on a list.  Because if the buffer is on a list,
 * it *must* already be on the right one.  If not, the filesystem is being
 * silly.  This will save a ton of locking.  But first we have to ensure
 * that buffers are taken *off* the old inode's list when they are freed
 * (presumably in truncate).  That requires careful auditing of all
 * filesystems (do it inside bforget()).  It could also be done by bringing
 * b_inode back.
 */

static void __remove_assoc_queue(struct buffer_head *bh)
{
	list_del_init(&bh->b_assoc_buffers);
	WARN_ON(!bh->b_assoc_map);
	if (buffer_write_io_error(bh))
		set_bit(AS_EIO, &bh->b_assoc_map->flags);
	bh->b_assoc_map = NULL;
}

int inode_has_buffers(struct inode *inode)
{
	return !list_empty(&inode->i_data.private_list);
}

static int osync_buffers_list(spinlock_t *lock, struct list_head *list)
{
	struct buffer_head *bh;
	struct list_head *p;
	int err = 0;

	spin_lock(lock);
repeat:
	list_for_each_prev(p, list) {
		bh = BH_ENTRY(p);
		if (buffer_locked(bh)) {
			get_bh(bh);
			spin_unlock(lock);
			wait_on_buffer(bh);
			if (!buffer_uptodate(bh))
				err = -EIO;
			brelse(bh);
			spin_lock(lock);
			goto repeat;
		}
	}
	spin_unlock(lock);
	return err;
}

static void do_thaw_one(struct super_block *sb, void *unused)
{
	char b[BDEVNAME_SIZE];
	while (sb->s_bdev && !thaw_bdev(sb->s_bdev, sb))
		printk(KERN_WARNING "Emergency Thaw on %s\n",
		       bdevname(sb->s_bdev, b));
}

static void do_thaw_all(struct work_struct *work)
{
	iterate_supers(do_thaw_one, NULL);
	kfree(work);
	printk(KERN_WARNING "Emergency Thaw complete\n");
}

void emergency_thaw_all(void)
{
	struct work_struct *work;

	work = kmalloc(sizeof(*work), GFP_ATOMIC);
	if (work) {
		INIT_WORK(work, do_thaw_all);
		schedule_work(work);
	}
}

/**
 * sync_mapping_buffers - write out & wait upon a mapping's "associated" buffers
 * @mapping: the mapping which wants those buffers written
 *
 * Starts I/O against the buffers at mapping->private_list, and waits upon
 * that I/O.
 *
 * Basically, this is a convenience function for fsync().
 * @mapping is a file or directory which needs those buffers to be written for
 * a successful fsync().
 */
int sync_mapping_buffers(struct address_space *mapping)
{
	struct address_space *buffer_mapping = mapping->assoc_mapping;

	if (buffer_mapping == NULL || list_empty(&mapping->private_list))
		return 0;

	return fsync_buffers_list(&buffer_mapping->private_lock,
					&mapping->private_list);
}
EXPORT_SYMBOL(sync_mapping_buffers);

/*
 * Called when we've recently written block `bblock', and it is known that
 * `bblock' was for a buffer_boundary() buffer.  This means that the block at
 * `bblock + 1' is probably a dirty indirect block.  Hunt it down and, if it's
 * dirty, schedule it for IO.  So that indirects merge nicely with their data.
 */
void write_boundary_block(struct block_device *bdev,
			sector_t bblock, unsigned blocksize)
{
	struct buffer_head *bh = __find_get_block(bdev, bblock + 1, blocksize);
	if (bh) {
		if (buffer_dirty(bh))
			ll_rw_block(WRITE, 1, &bh);
		put_bh(bh);
	}
}

void mark_buffer_dirty_inode(struct buffer_head *bh, struct inode *inode)
{
	struct address_space *mapping = inode->i_mapping;
	struct address_space *buffer_mapping = bh->b_page->mapping;

	mark_buffer_dirty(bh);
	if (!mapping->assoc_mapping) {
		mapping->assoc_mapping = buffer_mapping;
	} else {
		BUG_ON(mapping->assoc_mapping != buffer_mapping);
	}
	if (!bh->b_assoc_map) {
		spin_lock(&buffer_mapping->private_lock);
		list_move_tail(&bh->b_assoc_buffers,
				&mapping->private_list);
		bh->b_assoc_map = mapping;
		spin_unlock(&buffer_mapping->private_lock);
	}
}
EXPORT_SYMBOL(mark_buffer_dirty_inode);

static void __set_page_dirty(struct page *page,
		struct address_space *mapping, int warn)
{
	spin_lock_irq(&mapping->tree_lock);
	if (page->mapping) {	
		WARN_ON_ONCE(warn && !PageUptodate(page));
		account_page_dirtied(page, mapping);
		radix_tree_tag_set(&mapping->page_tree,
				page_index(page), PAGECACHE_TAG_DIRTY);
	}
	spin_unlock_irq(&mapping->tree_lock);
	__mark_inode_dirty(mapping->host, I_DIRTY_PAGES);
}

int __set_page_dirty_buffers(struct page *page)
{
	int newly_dirty;
	struct address_space *mapping = page_mapping(page);

	if (unlikely(!mapping))
		return !TestSetPageDirty(page);

	spin_lock(&mapping->private_lock);
	if (page_has_buffers(page)) {
		struct buffer_head *head = page_buffers(page);
		struct buffer_head *bh = head;

		do {
			set_buffer_dirty(bh);
			bh = bh->b_this_page;
		} while (bh != head);
	}
	newly_dirty = !TestSetPageDirty(page);
	spin_unlock(&mapping->private_lock);

	if (newly_dirty)
		__set_page_dirty(page, mapping, 1);
	return newly_dirty;
}
EXPORT_SYMBOL(__set_page_dirty_buffers);

static int fsync_buffers_list(spinlock_t *lock, struct list_head *list)
{
	struct buffer_head *bh;
	struct list_head tmp;
	struct address_space *mapping;
	int err = 0, err2;
	struct blk_plug plug;

	INIT_LIST_HEAD(&tmp);
	blk_start_plug(&plug);

	spin_lock(lock);
	while (!list_empty(list)) {
		bh = BH_ENTRY(list->next);
		mapping = bh->b_assoc_map;
		__remove_assoc_queue(bh);
		smp_mb();
		if (buffer_dirty(bh) || buffer_locked(bh)) {
			list_add(&bh->b_assoc_buffers, &tmp);
			bh->b_assoc_map = mapping;
			if (buffer_dirty(bh)) {
				get_bh(bh);
				spin_unlock(lock);
				write_dirty_buffer(bh, WRITE_SYNC);

				brelse(bh);
				spin_lock(lock);
			}
		}
	}

	spin_unlock(lock);
	blk_finish_plug(&plug);
	spin_lock(lock);

	while (!list_empty(&tmp)) {
		bh = BH_ENTRY(tmp.prev);
		get_bh(bh);
		mapping = bh->b_assoc_map;
		__remove_assoc_queue(bh);
		smp_mb();
		if (buffer_dirty(bh)) {
			list_add(&bh->b_assoc_buffers,
				 &mapping->private_list);
			bh->b_assoc_map = mapping;
		}
		spin_unlock(lock);
		wait_on_buffer(bh);
		if (!buffer_uptodate(bh))
			err = -EIO;
		brelse(bh);
		spin_lock(lock);
	}
	
	spin_unlock(lock);
	err2 = osync_buffers_list(lock, list);
	if (err)
		return err;
	else
		return err2;
}

void invalidate_inode_buffers(struct inode *inode)
{
	if (inode_has_buffers(inode)) {
		struct address_space *mapping = &inode->i_data;
		struct list_head *list = &mapping->private_list;
		struct address_space *buffer_mapping = mapping->assoc_mapping;

		spin_lock(&buffer_mapping->private_lock);
		while (!list_empty(list))
			__remove_assoc_queue(BH_ENTRY(list->next));
		spin_unlock(&buffer_mapping->private_lock);
	}
}
EXPORT_SYMBOL(invalidate_inode_buffers);

int remove_inode_buffers(struct inode *inode)
{
	int ret = 1;

	if (inode_has_buffers(inode)) {
		struct address_space *mapping = &inode->i_data;
		struct list_head *list = &mapping->private_list;
		struct address_space *buffer_mapping = mapping->assoc_mapping;

		spin_lock(&buffer_mapping->private_lock);
		while (!list_empty(list)) {
			struct buffer_head *bh = BH_ENTRY(list->next);
			if (buffer_dirty(bh)) {
				ret = 0;
				break;
			}
			__remove_assoc_queue(bh);
		}
		spin_unlock(&buffer_mapping->private_lock);
	}
	return ret;
}

struct buffer_head *alloc_page_buffers(struct page *page, unsigned long size,
		int retry)
{
	struct buffer_head *bh, *head;
	long offset;

try_again:
	head = NULL;
	offset = PAGE_SIZE;
	while ((offset -= size) >= 0) {
		bh = alloc_buffer_head(GFP_NOFS);
		if (!bh)
			goto no_grow;

		bh->b_bdev = NULL;
		bh->b_this_page = head;
		bh->b_blocknr = -1;
		head = bh;

		bh->b_state = 0;
		atomic_set(&bh->b_count, 0);
		bh->b_size = size;

		
		set_bh_page(bh, page, offset);

		init_buffer(bh, NULL, NULL);
	}
	return head;
no_grow:
	if (head) {
		do {
			bh = head;
			head = head->b_this_page;
			free_buffer_head(bh);
		} while (head);
	}

	if (!retry)
		return NULL;

	free_more_memory();
	goto try_again;
}
EXPORT_SYMBOL_GPL(alloc_page_buffers);

static inline void
link_dev_buffers(struct page *page, struct buffer_head *head)
{
	struct buffer_head *bh, *tail;

	bh = head;
	do {
		tail = bh;
		bh = bh->b_this_page;
	} while (bh);
	tail->b_this_page = head;
	attach_page_buffers(page, head);
}

 
static void
init_page_buffers(struct page *page, struct block_device *bdev,
			sector_t block, int size)
{
	struct buffer_head *head = page_buffers(page);
	struct buffer_head *bh = head;
	int uptodate = PageUptodate(page);
	sector_t end_block = blkdev_max_block(I_BDEV(bdev->bd_inode));

	do {
		if (!buffer_mapped(bh)) {
			init_buffer(bh, NULL, NULL);
			bh->b_bdev = bdev;
			bh->b_blocknr = block;
			if (uptodate)
				set_buffer_uptodate(bh);
			if (block < end_block)
				set_buffer_mapped(bh);
		}
		block++;
		bh = bh->b_this_page;
	} while (bh != head);
}

static struct page *
grow_dev_page(struct block_device *bdev, sector_t block,
		pgoff_t index, int size)
{
	struct inode *inode = bdev->bd_inode;
	struct page *page;
	struct buffer_head *bh;

	page = find_or_create_page(inode->i_mapping, index,
		(mapping_gfp_mask(inode->i_mapping) & ~__GFP_FS)|__GFP_MOVABLE);
	if (!page)
		return NULL;

	BUG_ON(!PageLocked(page));

	if (page_has_buffers(page)) {
		bh = page_buffers(page);
		if (bh->b_size == size) {
			init_page_buffers(page, bdev, block, size);
			return page;
		}
		if (!try_to_free_buffers(page))
			goto failed;
	}

	bh = alloc_page_buffers(page, size, 0);
	if (!bh)
		goto failed;

	spin_lock(&inode->i_mapping->private_lock);
	link_dev_buffers(page, bh);
	init_page_buffers(page, bdev, block, size);
	spin_unlock(&inode->i_mapping->private_lock);
	return page;

failed:
	unlock_page(page);
	page_cache_release(page);
	return NULL;
}

static int
grow_buffers(struct block_device *bdev, sector_t block, int size)
{
	struct page *page;
	pgoff_t index;
	int sizebits;

	sizebits = -1;
	do {
		sizebits++;
	} while ((size << sizebits) < PAGE_SIZE);

	index = block >> sizebits;

	if (unlikely(index != block >> sizebits)) {
		char b[BDEVNAME_SIZE];

		printk(KERN_ERR "%s: requested out-of-range block %llu for "
			"device %s\n",
			__func__, (unsigned long long)block,
			bdevname(bdev, b));
		return -EIO;
	}
	block = index << sizebits;
	
	page = grow_dev_page(bdev, block, index, size);
	if (!page)
		return 0;
	unlock_page(page);
	page_cache_release(page);
	return 1;
}

static struct buffer_head *
__getblk_slow(struct block_device *bdev, sector_t block, int size)
{
	sector_t maxsector;
	unsigned int nr_sectors;

	
	if (unlikely(size & (bdev_logical_block_size(bdev)-1) ||
			(size < 512 || size > PAGE_SIZE))) {
		printk(KERN_ERR "getblk(): invalid block size %d requested\n",
					size);
		printk(KERN_ERR "logical block size: %d\n",
					bdev_logical_block_size(bdev));

		dump_stack();
		return NULL;
	}
	maxsector = i_size_read(bdev->bd_inode) >> 9;
	nr_sectors = size >> 9;
	if (maxsector && (block + nr_sectors > maxsector)) {
		char b[BDEVNAME_SIZE];
		pr_err("%s(%d): %s access(block %llu,size %u) beyond end of device (%s max %llu)\n",
				current->comm, current->pid, __func__,
				(unsigned long long)block, nr_sectors,
				bdevname(bdev, b), (unsigned long long)maxsector);
		dump_stack();
		return NULL;
	}

	for (;;) {
		struct buffer_head * bh;
		int ret;

		bh = __find_get_block(bdev, block, size);
		if (bh)
			return bh;

		ret = grow_buffers(bdev, block, size);
		if (ret < 0)
			return NULL;
		if (ret == 0)
			free_more_memory();
	}
}


void mark_buffer_dirty(struct buffer_head *bh)
{
	WARN_ON_ONCE(!buffer_uptodate(bh));

	if (buffer_dirty(bh)) {
		smp_mb();
		if (buffer_dirty(bh))
			return;
	}

	if (!test_set_buffer_dirty(bh)) {
		struct page *page = bh->b_page;
		if (!TestSetPageDirty(page)) {
			struct address_space *mapping = page_mapping(page);
			if (mapping)
				__set_page_dirty(page, mapping, 0);
		}
	}
}
EXPORT_SYMBOL(mark_buffer_dirty);

void __brelse(struct buffer_head * buf)
{
	if (atomic_read(&buf->b_count)) {
		put_bh(buf);
		return;
	}
	WARN(1, KERN_ERR "VFS: brelse: Trying to free free buffer\n");
}
EXPORT_SYMBOL(__brelse);

void __bforget(struct buffer_head *bh)
{
	clear_buffer_dirty(bh);
	if (bh->b_assoc_map) {
		struct address_space *buffer_mapping = bh->b_page->mapping;

		spin_lock(&buffer_mapping->private_lock);
		list_del_init(&bh->b_assoc_buffers);
		bh->b_assoc_map = NULL;
		spin_unlock(&buffer_mapping->private_lock);
	}
	__brelse(bh);
}
EXPORT_SYMBOL(__bforget);

static struct buffer_head *__bread_slow(struct buffer_head *bh)
{
	lock_buffer(bh);
	if (buffer_uptodate(bh)) {
		unlock_buffer(bh);
		return bh;
	} else {
		get_bh(bh);
		bh->b_end_io = end_buffer_read_sync;
		submit_bh(READ, bh);
		wait_on_buffer(bh);
		if (buffer_uptodate(bh))
			return bh;
	}
	brelse(bh);
	return NULL;
}


#define BH_LRU_SIZE	8

struct bh_lru {
	struct buffer_head *bhs[BH_LRU_SIZE];
};

static DEFINE_PER_CPU(struct bh_lru, bh_lrus) = {{ NULL }};

#ifdef CONFIG_SMP
#define bh_lru_lock()	local_irq_disable()
#define bh_lru_unlock()	local_irq_enable()
#else
#define bh_lru_lock()	preempt_disable()
#define bh_lru_unlock()	preempt_enable()
#endif

static inline void check_irqs_on(void)
{
#ifdef irqs_disabled
	BUG_ON(irqs_disabled());
#endif
}

static void bh_lru_install(struct buffer_head *bh)
{
	struct buffer_head *evictee = NULL;

	check_irqs_on();
	bh_lru_lock();
	if (__this_cpu_read(bh_lrus.bhs[0]) != bh) {
		struct buffer_head *bhs[BH_LRU_SIZE];
		int in;
		int out = 0;

		get_bh(bh);
		bhs[out++] = bh;
		for (in = 0; in < BH_LRU_SIZE; in++) {
			struct buffer_head *bh2 =
				__this_cpu_read(bh_lrus.bhs[in]);

			if (bh2 == bh) {
				__brelse(bh2);
			} else {
				if (out >= BH_LRU_SIZE) {
					BUG_ON(evictee != NULL);
					evictee = bh2;
				} else {
					bhs[out++] = bh2;
				}
			}
		}
		while (out < BH_LRU_SIZE)
			bhs[out++] = NULL;
		memcpy(__this_cpu_ptr(&bh_lrus.bhs), bhs, sizeof(bhs));
	}
	bh_lru_unlock();

	if (evictee)
		__brelse(evictee);
}

static struct buffer_head *
lookup_bh_lru(struct block_device *bdev, sector_t block, unsigned size)
{
	struct buffer_head *ret = NULL;
	unsigned int i;

	check_irqs_on();
	bh_lru_lock();
	for (i = 0; i < BH_LRU_SIZE; i++) {
		struct buffer_head *bh = __this_cpu_read(bh_lrus.bhs[i]);

		if (bh && bh->b_bdev == bdev &&
				bh->b_blocknr == block && bh->b_size == size) {
			if (i) {
				while (i) {
					__this_cpu_write(bh_lrus.bhs[i],
						__this_cpu_read(bh_lrus.bhs[i - 1]));
					i--;
				}
				__this_cpu_write(bh_lrus.bhs[0], bh);
			}
			get_bh(bh);
			ret = bh;
			break;
		}
	}
	bh_lru_unlock();
	return ret;
}

struct buffer_head *
__find_get_block(struct block_device *bdev, sector_t block, unsigned size)
{
	struct buffer_head *bh = lookup_bh_lru(bdev, block, size);

	if (bh == NULL) {
		bh = __find_get_block_slow(bdev, block);
		if (bh)
			bh_lru_install(bh);
	}
	if (bh)
		touch_buffer(bh);
	return bh;
}
EXPORT_SYMBOL(__find_get_block);

struct buffer_head *
__getblk(struct block_device *bdev, sector_t block, unsigned size)
{
	struct buffer_head *bh = __find_get_block(bdev, block, size);

	might_sleep();
	if (bh == NULL)
		bh = __getblk_slow(bdev, block, size);
	return bh;
}
EXPORT_SYMBOL(__getblk);

void __breadahead(struct block_device *bdev, sector_t block, unsigned size)
{
	struct buffer_head *bh = __getblk(bdev, block, size);
	if (likely(bh)) {
		ll_rw_block(READA, 1, &bh);
		brelse(bh);
	}
}
EXPORT_SYMBOL(__breadahead);

struct buffer_head *
__bread(struct block_device *bdev, sector_t block, unsigned size)
{
	struct buffer_head *bh = __getblk(bdev, block, size);

	if (likely(bh) && !buffer_uptodate(bh))
		bh = __bread_slow(bh);
	return bh;
}
EXPORT_SYMBOL(__bread);

static void invalidate_bh_lru(void *arg)
{
	struct bh_lru *b = &get_cpu_var(bh_lrus);
	int i;

	for (i = 0; i < BH_LRU_SIZE; i++) {
		brelse(b->bhs[i]);
		b->bhs[i] = NULL;
	}
	put_cpu_var(bh_lrus);
}

static bool has_bh_in_lru(int cpu, void *dummy)
{
	struct bh_lru *b = per_cpu_ptr(&bh_lrus, cpu);
	int i;
	
	for (i = 0; i < BH_LRU_SIZE; i++) {
		if (b->bhs[i])
			return 1;
	}

	return 0;
}

static void __evict_bh_lru(void *arg)
{
	struct bh_lru *b = &get_cpu_var(bh_lrus);
	struct buffer_head *bh = arg;
	int i;

	for (i = 0; i < BH_LRU_SIZE; i++) {
		if (b->bhs[i] == bh) {
			brelse(b->bhs[i]);
			b->bhs[i] = NULL;
			goto out;
		}
	}
out:
	put_cpu_var(bh_lrus);
}

static bool bh_exists_in_lru(int cpu, void *arg)
{
	struct bh_lru *b = per_cpu_ptr(&bh_lrus, cpu);
	struct buffer_head *bh = arg;
	int i;

	for (i = 0; i < BH_LRU_SIZE; i++) {
		if (b->bhs[i] == bh)
			return 1;
	}

	return 0;

}
void invalidate_bh_lrus(void)
{
	on_each_cpu_cond(has_bh_in_lru, invalidate_bh_lru, NULL, 1, GFP_KERNEL);
}
EXPORT_SYMBOL_GPL(invalidate_bh_lrus);

void evict_bh_lrus(struct buffer_head *bh)
{
	on_each_cpu_cond(bh_exists_in_lru, __evict_bh_lru, bh, 1, GFP_ATOMIC);
}
EXPORT_SYMBOL_GPL(evict_bh_lrus);

void set_bh_page(struct buffer_head *bh,
		struct page *page, unsigned long offset)
{
	bh->b_page = page;
	BUG_ON(offset >= PAGE_SIZE);
	if (PageHighMem(page))
		bh->b_data = (char *)(0 + offset);
	else
		bh->b_data = page_address(page) + offset;
}
EXPORT_SYMBOL(set_bh_page);

static void discard_buffer(struct buffer_head * bh)
{
	lock_buffer(bh);
	clear_buffer_dirty(bh);
	bh->b_bdev = NULL;
	clear_buffer_mapped(bh);
	clear_buffer_req(bh);
	clear_buffer_new(bh);
	clear_buffer_delay(bh);
	clear_buffer_unwritten(bh);
	unlock_buffer(bh);
}

void block_invalidatepage(struct page *page, unsigned long offset)
{
	struct buffer_head *head, *bh, *next;
	unsigned int curr_off = 0;

	BUG_ON(!PageLocked(page));
	if (!page_has_buffers(page))
		goto out;

	head = page_buffers(page);
	bh = head;
	do {
		unsigned int next_off = curr_off + bh->b_size;
		next = bh->b_this_page;

		if (offset <= curr_off)
			discard_buffer(bh);
		curr_off = next_off;
		bh = next;
	} while (bh != head);

	if (offset == 0)
		try_to_release_page(page, 0);
out:
	return;
}
EXPORT_SYMBOL(block_invalidatepage);

void create_empty_buffers(struct page *page,
			unsigned long blocksize, unsigned long b_state)
{
	struct buffer_head *bh, *head, *tail;

	head = alloc_page_buffers(page, blocksize, 1);
	bh = head;
	do {
		bh->b_state |= b_state;
		tail = bh;
		bh = bh->b_this_page;
	} while (bh);
	tail->b_this_page = head;

	spin_lock(&page->mapping->private_lock);
	if (PageUptodate(page) || PageDirty(page)) {
		bh = head;
		do {
			if (PageDirty(page))
				set_buffer_dirty(bh);
			if (PageUptodate(page))
				set_buffer_uptodate(bh);
			bh = bh->b_this_page;
		} while (bh != head);
	}
	attach_page_buffers(page, head);
	spin_unlock(&page->mapping->private_lock);
}
EXPORT_SYMBOL(create_empty_buffers);

void unmap_underlying_metadata(struct block_device *bdev, sector_t block)
{
	struct buffer_head *old_bh;

	might_sleep();

	old_bh = __find_get_block_slow(bdev, block);
	if (old_bh) {
		clear_buffer_dirty(old_bh);
		wait_on_buffer(old_bh);
		clear_buffer_req(old_bh);
		__brelse(old_bh);
	}
}
EXPORT_SYMBOL(unmap_underlying_metadata);


/*
 * While block_write_full_page is writing back the dirty buffers under
 * the page lock, whoever dirtied the buffers may decide to clean them
 * again at any time.  We handle that by only looking at the buffer
 * state inside lock_buffer().
 *
 * If block_write_full_page() is called for regular writeback
 * (wbc->sync_mode == WB_SYNC_NONE) then it will redirty a page which has a
 * locked buffer.   This only can happen if someone has written the buffer
 * directly, with submit_bh().  At the address_space level PageWriteback
 * prevents this contention from occurring.
 *
 * If block_write_full_page() is called with wbc->sync_mode ==
 * WB_SYNC_ALL, the writes are posted using WRITE_SYNC; this
 * causes the writes to be flagged as synchronous writes.
 */
static int __block_write_full_page(struct inode *inode, struct page *page,
			get_block_t *get_block, struct writeback_control *wbc,
			bh_end_io_t *handler)
{
	int err;
	sector_t block;
	sector_t last_block;
	struct buffer_head *bh, *head;
	const unsigned blocksize = 1 << inode->i_blkbits;
	int nr_underway = 0;
	int write_op = (wbc->sync_mode == WB_SYNC_ALL ?
			WRITE_SYNC : WRITE);

	BUG_ON(!PageLocked(page));

	last_block = (i_size_read(inode) - 1) >> inode->i_blkbits;

	if (!page_has_buffers(page)) {
		create_empty_buffers(page, blocksize,
					(1 << BH_Dirty)|(1 << BH_Uptodate));
	}


	block = (sector_t)page->index << (PAGE_CACHE_SHIFT - inode->i_blkbits);
	head = page_buffers(page);
	bh = head;

	do {
		if (block > last_block) {
			clear_buffer_dirty(bh);
			set_buffer_uptodate(bh);
		} else if ((!buffer_mapped(bh) || buffer_delay(bh)) &&
			   buffer_dirty(bh)) {
			WARN_ON(bh->b_size != blocksize);
			err = get_block(inode, block, bh, 1);
			if (err)
				goto recover;
			clear_buffer_delay(bh);
			if (buffer_new(bh)) {
				
				clear_buffer_new(bh);
				unmap_underlying_metadata(bh->b_bdev,
							bh->b_blocknr);
			}
		}
		bh = bh->b_this_page;
		block++;
	} while (bh != head);

	do {
		if (!buffer_mapped(bh))
			continue;
		if (wbc->sync_mode != WB_SYNC_NONE) {
			lock_buffer(bh);
		} else if (!trylock_buffer(bh)) {
			redirty_page_for_writepage(wbc, page);
			continue;
		}
		if (test_clear_buffer_dirty(bh)) {
			mark_buffer_async_write_endio(bh, handler);
		} else {
			unlock_buffer(bh);
		}
	} while ((bh = bh->b_this_page) != head);

	BUG_ON(PageWriteback(page));
	set_page_writeback(page);

	do {
		struct buffer_head *next = bh->b_this_page;
		if (buffer_async_write(bh)) {
			submit_bh(write_op, bh);
			nr_underway++;
		}
		bh = next;
	} while (bh != head);
	unlock_page(page);

	err = 0;
done:
	if (nr_underway == 0) {
		end_page_writeback(page);

	}
	return err;

recover:
	bh = head;
	
	do {
		if (buffer_mapped(bh) && buffer_dirty(bh) &&
		    !buffer_delay(bh)) {
			lock_buffer(bh);
			mark_buffer_async_write_endio(bh, handler);
		} else {
			clear_buffer_dirty(bh);
		}
	} while ((bh = bh->b_this_page) != head);
	SetPageError(page);
	BUG_ON(PageWriteback(page));
	mapping_set_error(page->mapping, err);
	set_page_writeback(page);
	do {
		struct buffer_head *next = bh->b_this_page;
		if (buffer_async_write(bh)) {
			clear_buffer_dirty(bh);
			submit_bh(write_op, bh);
			nr_underway++;
		}
		bh = next;
	} while (bh != head);
	unlock_page(page);
	goto done;
}

/*
 * If a page has any new buffers, zero them out here, and mark them uptodate
 * and dirty so they'll be written out (in order to prevent uninitialised
 * block data from leaking). And clear the new bit.
 */
void page_zero_new_buffers(struct page *page, unsigned from, unsigned to)
{
	unsigned int block_start, block_end;
	struct buffer_head *head, *bh;

	BUG_ON(!PageLocked(page));
	if (!page_has_buffers(page))
		return;

	bh = head = page_buffers(page);
	block_start = 0;
	do {
		block_end = block_start + bh->b_size;

		if (buffer_new(bh)) {
			if (block_end > from && block_start < to) {
				if (!PageUptodate(page)) {
					unsigned start, size;

					start = max(from, block_start);
					size = min(to, block_end) - start;

					zero_user(page, start, size);
					set_buffer_uptodate(bh);
				}

				clear_buffer_new(bh);
				mark_buffer_dirty(bh);
			}
		}

		block_start = block_end;
		bh = bh->b_this_page;
	} while (bh != head);
}
EXPORT_SYMBOL(page_zero_new_buffers);

int __block_write_begin(struct page *page, loff_t pos, unsigned len,
		get_block_t *get_block)
{
	unsigned from = pos & (PAGE_CACHE_SIZE - 1);
	unsigned to = from + len;
	struct inode *inode = page->mapping->host;
	unsigned block_start, block_end;
	sector_t block;
	int err = 0;
	unsigned blocksize, bbits;
	struct buffer_head *bh, *head, *wait[2], **wait_bh=wait;

	BUG_ON(!PageLocked(page));
	BUG_ON(from > PAGE_CACHE_SIZE);
	BUG_ON(to > PAGE_CACHE_SIZE);
	BUG_ON(from > to);

	blocksize = 1 << inode->i_blkbits;
	if (!page_has_buffers(page))
		create_empty_buffers(page, blocksize, 0);
	head = page_buffers(page);

	bbits = inode->i_blkbits;
	block = (sector_t)page->index << (PAGE_CACHE_SHIFT - bbits);

	for(bh = head, block_start = 0; bh != head || !block_start;
	    block++, block_start=block_end, bh = bh->b_this_page) {
		block_end = block_start + blocksize;
		if (block_end <= from || block_start >= to) {
			if (PageUptodate(page)) {
				if (!buffer_uptodate(bh))
					set_buffer_uptodate(bh);
			}
			continue;
		}
		if (buffer_new(bh))
			clear_buffer_new(bh);
		if (!buffer_mapped(bh)) {
			WARN_ON(bh->b_size != blocksize);
			err = get_block(inode, block, bh, 1);
			if (err)
				break;
			if (buffer_new(bh)) {
				unmap_underlying_metadata(bh->b_bdev,
							bh->b_blocknr);
				if (PageUptodate(page)) {
					clear_buffer_new(bh);
					set_buffer_uptodate(bh);
					mark_buffer_dirty(bh);
					continue;
				}
				if (block_end > to || block_start < from)
					zero_user_segments(page,
						to, block_end,
						block_start, from);
				continue;
			}
		}
		if (PageUptodate(page)) {
			if (!buffer_uptodate(bh))
				set_buffer_uptodate(bh);
			continue; 
		}
		if (!buffer_uptodate(bh) && !buffer_delay(bh) &&
		    !buffer_unwritten(bh) &&
		     (block_start < from || block_end > to)) {
			ll_rw_block(READ, 1, &bh);
			*wait_bh++=bh;
		}
	}
	while(wait_bh > wait) {
		wait_on_buffer(*--wait_bh);
		if (!buffer_uptodate(*wait_bh))
			err = -EIO;
	}
	if (unlikely(err))
		page_zero_new_buffers(page, from, to);
	return err;
}
EXPORT_SYMBOL(__block_write_begin);

static int __block_commit_write(struct inode *inode, struct page *page,
		unsigned from, unsigned to)
{
	unsigned block_start, block_end;
	int partial = 0;
	unsigned blocksize;
	struct buffer_head *bh, *head;

	blocksize = 1 << inode->i_blkbits;

	for(bh = head = page_buffers(page), block_start = 0;
	    bh != head || !block_start;
	    block_start=block_end, bh = bh->b_this_page) {
		block_end = block_start + blocksize;
		if (block_end <= from || block_start >= to) {
			if (!buffer_uptodate(bh))
				partial = 1;
		} else {
			set_buffer_uptodate(bh);
			mark_buffer_dirty(bh);
		}
		clear_buffer_new(bh);
	}

	if (!partial)
		SetPageUptodate(page);
	return 0;
}

int block_write_begin(struct address_space *mapping, loff_t pos, unsigned len,
		unsigned flags, struct page **pagep, get_block_t *get_block)
{
	pgoff_t index = pos >> PAGE_CACHE_SHIFT;
	struct page *page;
	int status;

	page = grab_cache_page_write_begin(mapping, index, flags);
	if (!page)
		return -ENOMEM;

	status = __block_write_begin(page, pos, len, get_block);
	if (unlikely(status)) {
		unlock_page(page);
		page_cache_release(page);
		page = NULL;
	}

	*pagep = page;
	return status;
}
EXPORT_SYMBOL(block_write_begin);

int block_write_end(struct file *file, struct address_space *mapping,
			loff_t pos, unsigned len, unsigned copied,
			struct page *page, void *fsdata)
{
	struct inode *inode = mapping->host;
	unsigned start;

	start = pos & (PAGE_CACHE_SIZE - 1);

	if (unlikely(copied < len)) {
		/*
		 * The buffers that were written will now be uptodate, so we
		 * don't have to worry about a readpage reading them and
		 * overwriting a partial write. However if we have encountered
		 * a short write and only partially written into a buffer, it
		 * will not be marked uptodate, so a readpage might come in and
		 * destroy our partial write.
		 *
		 * Do the simplest thing, and just treat any short write to a
		 * non uptodate page as a zero-length write, and force the
		 * caller to redo the whole thing.
		 */
		if (!PageUptodate(page))
			copied = 0;

		page_zero_new_buffers(page, start+copied, start+len);
	}
	flush_dcache_page(page);

	
	__block_commit_write(inode, page, start, start+copied);

	return copied;
}
EXPORT_SYMBOL(block_write_end);

int generic_write_end(struct file *file, struct address_space *mapping,
			loff_t pos, unsigned len, unsigned copied,
			struct page *page, void *fsdata)
{
	struct inode *inode = mapping->host;
	int i_size_changed = 0;

	copied = block_write_end(file, mapping, pos, len, copied, page, fsdata);

	if (pos+copied > inode->i_size) {
		i_size_write(inode, pos+copied);
		i_size_changed = 1;
	}

	unlock_page(page);
	page_cache_release(page);

	if (i_size_changed)
		mark_inode_dirty(inode);

	return copied;
}
EXPORT_SYMBOL(generic_write_end);

int block_is_partially_uptodate(struct page *page, read_descriptor_t *desc,
					unsigned long from)
{
	struct inode *inode = page->mapping->host;
	unsigned block_start, block_end, blocksize;
	unsigned to;
	struct buffer_head *bh, *head;
	int ret = 1;

	if (!page_has_buffers(page))
		return 0;

	blocksize = 1 << inode->i_blkbits;
	to = min_t(unsigned, PAGE_CACHE_SIZE - from, desc->count);
	to = from + to;
	if (from < blocksize && to > PAGE_CACHE_SIZE - blocksize)
		return 0;

	head = page_buffers(page);
	bh = head;
	block_start = 0;
	do {
		block_end = block_start + blocksize;
		if (block_end > from && block_start < to) {
			if (!buffer_uptodate(bh)) {
				ret = 0;
				break;
			}
			if (block_end >= to)
				break;
		}
		block_start = block_end;
		bh = bh->b_this_page;
	} while (bh != head);

	return ret;
}
EXPORT_SYMBOL(block_is_partially_uptodate);

int block_read_full_page(struct page *page, get_block_t *get_block)
{
	struct inode *inode = page->mapping->host;
	sector_t iblock, lblock;
	struct buffer_head *bh, *head, *arr[MAX_BUF_PER_PAGE];
	unsigned int blocksize;
	int nr, i;
	int fully_mapped = 1;

	BUG_ON(!PageLocked(page));
	blocksize = 1 << inode->i_blkbits;
	if (!page_has_buffers(page))
		create_empty_buffers(page, blocksize, 0);
	head = page_buffers(page);

	iblock = (sector_t)page->index << (PAGE_CACHE_SHIFT - inode->i_blkbits);
	lblock = (i_size_read(inode)+blocksize-1) >> inode->i_blkbits;
	bh = head;
	nr = 0;
	i = 0;

	do {
		if (buffer_uptodate(bh))
			continue;

		if (!buffer_mapped(bh)) {
			int err = 0;

			fully_mapped = 0;
			if (iblock < lblock) {
				WARN_ON(bh->b_size != blocksize);
				err = get_block(inode, iblock, bh, 0);
				if (err)
					SetPageError(page);
			}
			if (!buffer_mapped(bh)) {
				zero_user(page, i * blocksize, blocksize);
				if (!err)
					set_buffer_uptodate(bh);
				continue;
			}
			if (buffer_uptodate(bh))
				continue;
		}
		arr[nr++] = bh;
	} while (i++, iblock++, (bh = bh->b_this_page) != head);

	if (fully_mapped)
		SetPageMappedToDisk(page);

	if (!nr) {
		if (!PageError(page))
			SetPageUptodate(page);
		unlock_page(page);
		return 0;
	}

	
	for (i = 0; i < nr; i++) {
		bh = arr[i];
		lock_buffer(bh);
		mark_buffer_async_read(bh);
	}

	for (i = 0; i < nr; i++) {
		bh = arr[i];
		if (buffer_uptodate(bh))
			end_buffer_async_read(bh, 1);
		else
			submit_bh(READ, bh);
	}
	return 0;
}
EXPORT_SYMBOL(block_read_full_page);

int generic_cont_expand_simple(struct inode *inode, loff_t size)
{
	struct address_space *mapping = inode->i_mapping;
	struct page *page;
	void *fsdata;
	int err;

	err = inode_newsize_ok(inode, size);
	if (err)
		goto out;

	err = pagecache_write_begin(NULL, mapping, size, 0,
				AOP_FLAG_UNINTERRUPTIBLE|AOP_FLAG_CONT_EXPAND,
				&page, &fsdata);
	if (err)
		goto out;

	err = pagecache_write_end(NULL, mapping, size, 0, 0, page, fsdata);
	BUG_ON(err > 0);

out:
	return err;
}
EXPORT_SYMBOL(generic_cont_expand_simple);

static int cont_expand_zero(struct file *file, struct address_space *mapping,
			    loff_t pos, loff_t *bytes)
{
	struct inode *inode = mapping->host;
	unsigned blocksize = 1 << inode->i_blkbits;
	struct page *page;
	void *fsdata;
	pgoff_t index, curidx;
	loff_t curpos;
	unsigned zerofrom, offset, len;
	int err = 0;

	index = pos >> PAGE_CACHE_SHIFT;
	offset = pos & ~PAGE_CACHE_MASK;

	while (index > (curidx = (curpos = *bytes)>>PAGE_CACHE_SHIFT)) {
		zerofrom = curpos & ~PAGE_CACHE_MASK;
		if (zerofrom & (blocksize-1)) {
			*bytes |= (blocksize-1);
			(*bytes)++;
		}
		len = PAGE_CACHE_SIZE - zerofrom;

		err = pagecache_write_begin(file, mapping, curpos, len,
						AOP_FLAG_UNINTERRUPTIBLE,
						&page, &fsdata);
		if (err)
			goto out;
		zero_user(page, zerofrom, len);
		err = pagecache_write_end(file, mapping, curpos, len, len,
						page, fsdata);
		if (err < 0)
			goto out;
		BUG_ON(err != len);
		err = 0;

		balance_dirty_pages_ratelimited(mapping);
	}

	
	if (index == curidx) {
		zerofrom = curpos & ~PAGE_CACHE_MASK;
		
		if (offset <= zerofrom) {
			goto out;
		}
		if (zerofrom & (blocksize-1)) {
			*bytes |= (blocksize-1);
			(*bytes)++;
		}
		len = offset - zerofrom;

		err = pagecache_write_begin(file, mapping, curpos, len,
						AOP_FLAG_UNINTERRUPTIBLE,
						&page, &fsdata);
		if (err)
			goto out;
		zero_user(page, zerofrom, len);
		err = pagecache_write_end(file, mapping, curpos, len, len,
						page, fsdata);
		if (err < 0)
			goto out;
		BUG_ON(err != len);
		err = 0;
	}
out:
	return err;
}

int cont_write_begin(struct file *file, struct address_space *mapping,
			loff_t pos, unsigned len, unsigned flags,
			struct page **pagep, void **fsdata,
			get_block_t *get_block, loff_t *bytes)
{
	struct inode *inode = mapping->host;
	unsigned blocksize = 1 << inode->i_blkbits;
	unsigned zerofrom;
	int err;

	err = cont_expand_zero(file, mapping, pos, bytes);
	if (err)
		return err;

	zerofrom = *bytes & ~PAGE_CACHE_MASK;
	if (pos+len > *bytes && zerofrom & (blocksize-1)) {
		*bytes |= (blocksize-1);
		(*bytes)++;
	}

	return block_write_begin(mapping, pos, len, flags, pagep, get_block);
}
EXPORT_SYMBOL(cont_write_begin);

int block_commit_write(struct page *page, unsigned from, unsigned to)
{
	struct inode *inode = page->mapping->host;
	__block_commit_write(inode,page,from,to);
	return 0;
}
EXPORT_SYMBOL(block_commit_write);

/*
 * block_page_mkwrite() is not allowed to change the file size as it gets
 * called from a page fault handler when a page is first dirtied. Hence we must
 * be careful to check for EOF conditions here. We set the page up correctly
 * for a written page which means we get ENOSPC checking when writing into
 * holes and correct delalloc and unwritten extent mapping on filesystems that
 * support these features.
 *
 * We are not allowed to take the i_mutex here so we have to play games to
 * protect against truncate races as the page could now be beyond EOF.  Because
 * truncate writes the inode size before removing pages, once we have the
 * page lock we can determine safely if the page is beyond EOF. If it is not
 * beyond EOF, then the page is guaranteed safe against truncation until we
 * unlock the page.
 *
 * Direct callers of this function should call vfs_check_frozen() so that page
 * fault does not busyloop until the fs is thawed.
 */
int __block_page_mkwrite(struct vm_area_struct *vma, struct vm_fault *vmf,
			 get_block_t get_block)
{
	struct page *page = vmf->page;
	struct inode *inode = vma->vm_file->f_path.dentry->d_inode;
	unsigned long end;
	loff_t size;
	int ret;

	lock_page(page);
	size = i_size_read(inode);
	if ((page->mapping != inode->i_mapping) ||
	    (page_offset(page) > size)) {
		
		ret = -EFAULT;
		goto out_unlock;
	}

	
	if (((page->index + 1) << PAGE_CACHE_SHIFT) > size)
		end = size & ~PAGE_CACHE_MASK;
	else
		end = PAGE_CACHE_SIZE;

	ret = __block_write_begin(page, 0, end, get_block);
	if (!ret)
		ret = block_commit_write(page, 0, end);

	if (unlikely(ret < 0))
		goto out_unlock;
	set_page_dirty(page);
	if (inode->i_sb->s_frozen != SB_UNFROZEN) {
		ret = -EAGAIN;
		goto out_unlock;
	}
	wait_on_page_writeback(page);
	return 0;
out_unlock:
	unlock_page(page);
	return ret;
}
EXPORT_SYMBOL(__block_page_mkwrite);

int block_page_mkwrite(struct vm_area_struct *vma, struct vm_fault *vmf,
		   get_block_t get_block)
{
	int ret;
	struct super_block *sb = vma->vm_file->f_path.dentry->d_inode->i_sb;

	vfs_check_frozen(sb, SB_FREEZE_WRITE);
	ret = __block_page_mkwrite(vma, vmf, get_block);
	return block_page_mkwrite_return(ret);
}
EXPORT_SYMBOL(block_page_mkwrite);

static void end_buffer_read_nobh(struct buffer_head *bh, int uptodate)
{
	__end_buffer_read_notouch(bh, uptodate);
}

static void attach_nobh_buffers(struct page *page, struct buffer_head *head)
{
	struct buffer_head *bh;

	BUG_ON(!PageLocked(page));

	spin_lock(&page->mapping->private_lock);
	bh = head;
	do {
		if (PageDirty(page))
			set_buffer_dirty(bh);
		if (!bh->b_this_page)
			bh->b_this_page = head;
		bh = bh->b_this_page;
	} while (bh != head);
	attach_page_buffers(page, head);
	spin_unlock(&page->mapping->private_lock);
}

int nobh_write_begin(struct address_space *mapping,
			loff_t pos, unsigned len, unsigned flags,
			struct page **pagep, void **fsdata,
			get_block_t *get_block)
{
	struct inode *inode = mapping->host;
	const unsigned blkbits = inode->i_blkbits;
	const unsigned blocksize = 1 << blkbits;
	struct buffer_head *head, *bh;
	struct page *page;
	pgoff_t index;
	unsigned from, to;
	unsigned block_in_page;
	unsigned block_start, block_end;
	sector_t block_in_file;
	int nr_reads = 0;
	int ret = 0;
	int is_mapped_to_disk = 1;

	index = pos >> PAGE_CACHE_SHIFT;
	from = pos & (PAGE_CACHE_SIZE - 1);
	to = from + len;

	page = grab_cache_page_write_begin(mapping, index, flags);
	if (!page)
		return -ENOMEM;
	*pagep = page;
	*fsdata = NULL;

	if (page_has_buffers(page)) {
		ret = __block_write_begin(page, pos, len, get_block);
		if (unlikely(ret))
			goto out_release;
		return ret;
	}

	if (PageMappedToDisk(page))
		return 0;

	head = alloc_page_buffers(page, blocksize, 0);
	if (!head) {
		ret = -ENOMEM;
		goto out_release;
	}

	block_in_file = (sector_t)page->index << (PAGE_CACHE_SHIFT - blkbits);

	for (block_start = 0, block_in_page = 0, bh = head;
		  block_start < PAGE_CACHE_SIZE;
		  block_in_page++, block_start += blocksize, bh = bh->b_this_page) {
		int create;

		block_end = block_start + blocksize;
		bh->b_state = 0;
		create = 1;
		if (block_start >= to)
			create = 0;
		ret = get_block(inode, block_in_file + block_in_page,
					bh, create);
		if (ret)
			goto failed;
		if (!buffer_mapped(bh))
			is_mapped_to_disk = 0;
		if (buffer_new(bh))
			unmap_underlying_metadata(bh->b_bdev, bh->b_blocknr);
		if (PageUptodate(page)) {
			set_buffer_uptodate(bh);
			continue;
		}
		if (buffer_new(bh) || !buffer_mapped(bh)) {
			zero_user_segments(page, block_start, from,
							to, block_end);
			continue;
		}
		if (buffer_uptodate(bh))
			continue;	
		if (block_start < from || block_end > to) {
			lock_buffer(bh);
			bh->b_end_io = end_buffer_read_nobh;
			submit_bh(READ, bh);
			nr_reads++;
		}
	}

	if (nr_reads) {
		for (bh = head; bh; bh = bh->b_this_page) {
			wait_on_buffer(bh);
			if (!buffer_uptodate(bh))
				ret = -EIO;
		}
		if (ret)
			goto failed;
	}

	if (is_mapped_to_disk)
		SetPageMappedToDisk(page);

	*fsdata = head; 

	return 0;

failed:
	BUG_ON(!ret);
	/*
	 * Error recovery is a bit difficult. We need to zero out blocks that
	 * were newly allocated, and dirty them to ensure they get written out.
	 * Buffers need to be attached to the page at this point, otherwise
	 * the handling of potential IO errors during writeout would be hard
	 * (could try doing synchronous writeout, but what if that fails too?)
	 */
	attach_nobh_buffers(page, head);
	page_zero_new_buffers(page, from, to);

out_release:
	unlock_page(page);
	page_cache_release(page);
	*pagep = NULL;

	return ret;
}
EXPORT_SYMBOL(nobh_write_begin);

int nobh_write_end(struct file *file, struct address_space *mapping,
			loff_t pos, unsigned len, unsigned copied,
			struct page *page, void *fsdata)
{
	struct inode *inode = page->mapping->host;
	struct buffer_head *head = fsdata;
	struct buffer_head *bh;
	BUG_ON(fsdata != NULL && page_has_buffers(page));

	if (unlikely(copied < len) && head)
		attach_nobh_buffers(page, head);
	if (page_has_buffers(page))
		return generic_write_end(file, mapping, pos, len,
					copied, page, fsdata);

	SetPageUptodate(page);
	set_page_dirty(page);
	if (pos+copied > inode->i_size) {
		i_size_write(inode, pos+copied);
		mark_inode_dirty(inode);
	}

	unlock_page(page);
	page_cache_release(page);

	while (head) {
		bh = head;
		head = head->b_this_page;
		free_buffer_head(bh);
	}

	return copied;
}
EXPORT_SYMBOL(nobh_write_end);

int nobh_writepage(struct page *page, get_block_t *get_block,
			struct writeback_control *wbc)
{
	struct inode * const inode = page->mapping->host;
	loff_t i_size = i_size_read(inode);
	const pgoff_t end_index = i_size >> PAGE_CACHE_SHIFT;
	unsigned offset;
	int ret;

	
	if (page->index < end_index)
		goto out;

	
	offset = i_size & (PAGE_CACHE_SIZE-1);
	if (page->index >= end_index+1 || !offset) {
#if 0
		
		if (page->mapping->a_ops->invalidatepage)
			page->mapping->a_ops->invalidatepage(page, offset);
#endif
		unlock_page(page);
		return 0; 
	}

	/*
	 * The page straddles i_size.  It must be zeroed out on each and every
	 * writepage invocation because it may be mmapped.  "A file is mapped
	 * in multiples of the page size.  For a file that is not a multiple of
	 * the  page size, the remaining memory is zeroed when mapped, and
	 * writes to that region are not written out to the file."
	 */
	zero_user_segment(page, offset, PAGE_CACHE_SIZE);
out:
	ret = mpage_writepage(page, get_block, wbc);
	if (ret == -EAGAIN)
		ret = __block_write_full_page(inode, page, get_block, wbc,
					      end_buffer_async_write);
	return ret;
}
EXPORT_SYMBOL(nobh_writepage);

int nobh_truncate_page(struct address_space *mapping,
			loff_t from, get_block_t *get_block)
{
	pgoff_t index = from >> PAGE_CACHE_SHIFT;
	unsigned offset = from & (PAGE_CACHE_SIZE-1);
	unsigned blocksize;
	sector_t iblock;
	unsigned length, pos;
	struct inode *inode = mapping->host;
	struct page *page;
	struct buffer_head map_bh;
	int err;

	blocksize = 1 << inode->i_blkbits;
	length = offset & (blocksize - 1);

	
	if (!length)
		return 0;

	length = blocksize - length;
	iblock = (sector_t)index << (PAGE_CACHE_SHIFT - inode->i_blkbits);

	page = grab_cache_page(mapping, index);
	err = -ENOMEM;
	if (!page)
		goto out;

	if (page_has_buffers(page)) {
has_buffers:
		unlock_page(page);
		page_cache_release(page);
		return block_truncate_page(mapping, from, get_block);
	}

	
	pos = blocksize;
	while (offset >= pos) {
		iblock++;
		pos += blocksize;
	}

	map_bh.b_size = blocksize;
	map_bh.b_state = 0;
	err = get_block(inode, iblock, &map_bh, 0);
	if (err)
		goto unlock;
	
	if (!buffer_mapped(&map_bh))
		goto unlock;

	
	if (!PageUptodate(page)) {
		err = mapping->a_ops->readpage(NULL, page);
		if (err) {
			page_cache_release(page);
			goto out;
		}
		lock_page(page);
		if (!PageUptodate(page)) {
			err = -EIO;
			goto unlock;
		}
		if (page_has_buffers(page))
			goto has_buffers;
	}
	zero_user(page, offset, length);
	set_page_dirty(page);
	err = 0;

unlock:
	unlock_page(page);
	page_cache_release(page);
out:
	return err;
}
EXPORT_SYMBOL(nobh_truncate_page);

int block_truncate_page(struct address_space *mapping,
			loff_t from, get_block_t *get_block)
{
	pgoff_t index = from >> PAGE_CACHE_SHIFT;
	unsigned offset = from & (PAGE_CACHE_SIZE-1);
	unsigned blocksize;
	sector_t iblock;
	unsigned length, pos;
	struct inode *inode = mapping->host;
	struct page *page;
	struct buffer_head *bh;
	int err;

	blocksize = 1 << inode->i_blkbits;
	length = offset & (blocksize - 1);

	
	if (!length)
		return 0;

	length = blocksize - length;
	iblock = (sector_t)index << (PAGE_CACHE_SHIFT - inode->i_blkbits);
	
	page = grab_cache_page(mapping, index);
	err = -ENOMEM;
	if (!page)
		goto out;

	if (!page_has_buffers(page))
		create_empty_buffers(page, blocksize, 0);

	
	bh = page_buffers(page);
	pos = blocksize;
	while (offset >= pos) {
		bh = bh->b_this_page;
		iblock++;
		pos += blocksize;
	}

	err = 0;
	if (!buffer_mapped(bh)) {
		WARN_ON(bh->b_size != blocksize);
		err = get_block(inode, iblock, bh, 0);
		if (err)
			goto unlock;
		
		if (!buffer_mapped(bh))
			goto unlock;
	}

	
	if (PageUptodate(page))
		set_buffer_uptodate(bh);

	if (!buffer_uptodate(bh) && !buffer_delay(bh) && !buffer_unwritten(bh)) {
		err = -EIO;
		ll_rw_block(READ, 1, &bh);
		wait_on_buffer(bh);
		
		if (!buffer_uptodate(bh))
			goto unlock;
	}

	zero_user(page, offset, length);
	mark_buffer_dirty(bh);
	err = 0;

unlock:
	unlock_page(page);
	page_cache_release(page);
out:
	return err;
}
EXPORT_SYMBOL(block_truncate_page);

int block_write_full_page_endio(struct page *page, get_block_t *get_block,
			struct writeback_control *wbc, bh_end_io_t *handler)
{
	struct inode * const inode = page->mapping->host;
	loff_t i_size = i_size_read(inode);
	const pgoff_t end_index = i_size >> PAGE_CACHE_SHIFT;
	unsigned offset;

	
	if (page->index < end_index)
		return __block_write_full_page(inode, page, get_block, wbc,
					       handler);

	
	offset = i_size & (PAGE_CACHE_SIZE-1);
	if (page->index >= end_index+1 || !offset) {
		do_invalidatepage(page, 0);
		unlock_page(page);
		return 0; 
	}

	/*
	 * The page straddles i_size.  It must be zeroed out on each and every
	 * writepage invocation because it may be mmapped.  "A file is mapped
	 * in multiples of the page size.  For a file that is not a multiple of
	 * the  page size, the remaining memory is zeroed when mapped, and
	 * writes to that region are not written out to the file."
	 */
	zero_user_segment(page, offset, PAGE_CACHE_SIZE);
	return __block_write_full_page(inode, page, get_block, wbc, handler);
}
EXPORT_SYMBOL(block_write_full_page_endio);

int block_write_full_page(struct page *page, get_block_t *get_block,
			struct writeback_control *wbc)
{
	return block_write_full_page_endio(page, get_block, wbc,
					   end_buffer_async_write);
}
EXPORT_SYMBOL(block_write_full_page);

sector_t generic_block_bmap(struct address_space *mapping, sector_t block,
			    get_block_t *get_block)
{
	struct buffer_head tmp;
	struct inode *inode = mapping->host;
	tmp.b_state = 0;
	tmp.b_blocknr = 0;
	tmp.b_size = 1 << inode->i_blkbits;
	get_block(inode, block, &tmp, 0);
	return tmp.b_blocknr;
}
EXPORT_SYMBOL(generic_block_bmap);

static void end_bio_bh_io_sync(struct bio *bio, int err)
{
	struct buffer_head *bh = bio->bi_private;

	if (err == -EOPNOTSUPP) {
		set_bit(BIO_EOPNOTSUPP, &bio->bi_flags);
	}

	if (unlikely (test_bit(BIO_QUIET,&bio->bi_flags)))
		set_bit(BH_Quiet, &bh->b_state);

	bh->b_end_io(bh, test_bit(BIO_UPTODATE, &bio->bi_flags));
	bio_put(bio);
}

int submit_bh(int rw, struct buffer_head * bh)
{
	struct bio *bio;
	int ret = 0;

	BUG_ON(!buffer_locked(bh));
	BUG_ON(!buffer_mapped(bh));
	BUG_ON(!bh->b_end_io);
	BUG_ON(buffer_delay(bh));
	BUG_ON(buffer_unwritten(bh));

	if (test_set_buffer_req(bh) && (rw & WRITE))
		clear_buffer_write_io_error(bh);

	bio = bio_alloc(GFP_NOIO, 1);

	bio->bi_sector = bh->b_blocknr * (bh->b_size >> 9);
	bio->bi_bdev = bh->b_bdev;
	bio->bi_io_vec[0].bv_page = bh->b_page;
	bio->bi_io_vec[0].bv_len = bh->b_size;
	bio->bi_io_vec[0].bv_offset = bh_offset(bh);

	bio->bi_vcnt = 1;
	bio->bi_idx = 0;
	bio->bi_size = bh->b_size;

	bio->bi_end_io = end_bio_bh_io_sync;
	bio->bi_private = bh;

	bio_get(bio);
	submit_bio(rw, bio);

	if (bio_flagged(bio, BIO_EOPNOTSUPP))
		ret = -EOPNOTSUPP;

	bio_put(bio);
	return ret;
}
EXPORT_SYMBOL(submit_bh);

void ll_rw_block(int rw, int nr, struct buffer_head *bhs[])
{
	int i;

	for (i = 0; i < nr; i++) {
		struct buffer_head *bh = bhs[i];

		if (!trylock_buffer(bh))
			continue;
		if (rw == WRITE) {
			if (test_clear_buffer_dirty(bh)) {
				bh->b_end_io = end_buffer_write_sync;
				get_bh(bh);
				submit_bh(WRITE, bh);
				continue;
			}
		} else {
			if (!buffer_uptodate(bh)) {
				bh->b_end_io = end_buffer_read_sync;
				get_bh(bh);
				submit_bh(rw, bh);
				continue;
			}
		}
		unlock_buffer(bh);
	}
}
EXPORT_SYMBOL(ll_rw_block);

void write_dirty_buffer(struct buffer_head *bh, int rw)
{
	lock_buffer(bh);
	if (!test_clear_buffer_dirty(bh)) {
		unlock_buffer(bh);
		return;
	}
	bh->b_end_io = end_buffer_write_sync;
	get_bh(bh);
	submit_bh(rw, bh);
}
EXPORT_SYMBOL(write_dirty_buffer);

int __sync_dirty_buffer(struct buffer_head *bh, int rw)
{
	int ret = 0;

	WARN_ON(atomic_read(&bh->b_count) < 1);
	lock_buffer(bh);
	if (test_clear_buffer_dirty(bh)) {
		get_bh(bh);
		bh->b_end_io = end_buffer_write_sync;
		ret = submit_bh(rw, bh);
		wait_on_buffer(bh);
		if (!ret && !buffer_uptodate(bh))
			ret = -EIO;
	} else {
		unlock_buffer(bh);
	}
	return ret;
}
EXPORT_SYMBOL(__sync_dirty_buffer);

int sync_dirty_buffer(struct buffer_head *bh)
{
	return __sync_dirty_buffer(bh, WRITE_SYNC);
}
EXPORT_SYMBOL(sync_dirty_buffer);

static inline int buffer_busy(struct buffer_head *bh)
{
	return atomic_read(&bh->b_count) |
		(bh->b_state & ((1 << BH_Dirty) | (1 << BH_Lock)));
}

static int
drop_buffers(struct page *page, struct buffer_head **buffers_to_free)
{
	struct buffer_head *head = page_buffers(page);
	struct buffer_head *bh;

	bh = head;
	do {
		if (buffer_write_io_error(bh) && page->mapping)
			set_bit(AS_EIO, &page->mapping->flags);
		if (buffer_busy(bh)) {
			evict_bh_lrus(bh);
			if (buffer_busy(bh))
				goto failed;
		}
		bh = bh->b_this_page;
	} while (bh != head);

	do {
		struct buffer_head *next = bh->b_this_page;

		if (bh->b_assoc_map)
			__remove_assoc_queue(bh);
		bh = next;
	} while (bh != head);
	*buffers_to_free = head;
	__clear_page_buffers(page);
	return 1;
failed:
	return 0;
}

int try_to_free_buffers(struct page *page)
{
	struct address_space * const mapping = page->mapping;
	struct buffer_head *buffers_to_free = NULL;
	int ret = 0;

	BUG_ON(!PageLocked(page));
	if (PageWriteback(page))
		return 0;

	if (mapping == NULL) {		
		ret = drop_buffers(page, &buffers_to_free);
		goto out;
	}

	spin_lock(&mapping->private_lock);
	ret = drop_buffers(page, &buffers_to_free);

	if (ret)
		cancel_dirty_page(page, PAGE_CACHE_SIZE);
	spin_unlock(&mapping->private_lock);
out:
	if (buffers_to_free) {
		struct buffer_head *bh = buffers_to_free;

		do {
			struct buffer_head *next = bh->b_this_page;
			free_buffer_head(bh);
			bh = next;
		} while (bh != buffers_to_free);
	}
	return ret;
}
EXPORT_SYMBOL(try_to_free_buffers);

SYSCALL_DEFINE2(bdflush, int, func, long, data)
{
	static int msg_count;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	if (msg_count < 5) {
		msg_count++;
		printk(KERN_INFO
			"warning: process `%s' used the obsolete bdflush"
			" system call\n", current->comm);
		printk(KERN_INFO "Fix your initscripts?\n");
	}

	if (func == 1)
		do_exit(0);
	return 0;
}

static struct kmem_cache *bh_cachep;

static int max_buffer_heads;

int buffer_heads_over_limit;

struct bh_accounting {
	int nr;			
	int ratelimit;		
};

static DEFINE_PER_CPU(struct bh_accounting, bh_accounting) = {0, 0};

static void recalc_bh_state(void)
{
	int i;
	int tot = 0;

	if (__this_cpu_inc_return(bh_accounting.ratelimit) - 1 < 4096)
		return;
	__this_cpu_write(bh_accounting.ratelimit, 0);
	for_each_online_cpu(i)
		tot += per_cpu(bh_accounting, i).nr;
	buffer_heads_over_limit = (tot > max_buffer_heads);
}

struct buffer_head *alloc_buffer_head(gfp_t gfp_flags)
{
	struct buffer_head *ret = kmem_cache_zalloc(bh_cachep, gfp_flags);
	if (ret) {
		INIT_LIST_HEAD(&ret->b_assoc_buffers);
		preempt_disable();
		__this_cpu_inc(bh_accounting.nr);
		recalc_bh_state();
		preempt_enable();
	}
	return ret;
}
EXPORT_SYMBOL(alloc_buffer_head);

void free_buffer_head(struct buffer_head *bh)
{
	BUG_ON(!list_empty(&bh->b_assoc_buffers));
	kmem_cache_free(bh_cachep, bh);
	preempt_disable();
	__this_cpu_dec(bh_accounting.nr);
	recalc_bh_state();
	preempt_enable();
}
EXPORT_SYMBOL(free_buffer_head);

static void buffer_exit_cpu(int cpu)
{
	int i;
	struct bh_lru *b = &per_cpu(bh_lrus, cpu);

	for (i = 0; i < BH_LRU_SIZE; i++) {
		brelse(b->bhs[i]);
		b->bhs[i] = NULL;
	}
	this_cpu_add(bh_accounting.nr, per_cpu(bh_accounting, cpu).nr);
	per_cpu(bh_accounting, cpu).nr = 0;
}

static int buffer_cpu_notify(struct notifier_block *self,
			      unsigned long action, void *hcpu)
{
	if (action == CPU_DEAD || action == CPU_DEAD_FROZEN)
		buffer_exit_cpu((unsigned long)hcpu);
	return NOTIFY_OK;
}

int bh_uptodate_or_lock(struct buffer_head *bh)
{
	if (!buffer_uptodate(bh)) {
		lock_buffer(bh);
		if (!buffer_uptodate(bh))
			return 0;
		unlock_buffer(bh);
	}
	return 1;
}
EXPORT_SYMBOL(bh_uptodate_or_lock);

int bh_submit_read(struct buffer_head *bh)
{
	BUG_ON(!buffer_locked(bh));

	if (buffer_uptodate(bh)) {
		unlock_buffer(bh);
		return 0;
	}

	get_bh(bh);
	bh->b_end_io = end_buffer_read_sync;
	submit_bh(READ, bh);
	wait_on_buffer(bh);
	if (buffer_uptodate(bh))
		return 0;
	return -EIO;
}
EXPORT_SYMBOL(bh_submit_read);

void __init buffer_init(void)
{
	int nrpages;

	bh_cachep = kmem_cache_create("buffer_head",
			sizeof(struct buffer_head), 0,
				(SLAB_RECLAIM_ACCOUNT|SLAB_PANIC|
				SLAB_MEM_SPREAD),
				NULL);

	nrpages = (nr_free_buffer_pages() * 10) / 100;
	max_buffer_heads = nrpages * (PAGE_SIZE / sizeof(struct buffer_head));
	hotcpu_notifier(buffer_cpu_notify, 0);
}
