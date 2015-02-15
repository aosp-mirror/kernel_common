/*
 * fs/direct-io.c
 *
 * Copyright (C) 2002, Linus Torvalds.
 *
 * O_DIRECT
 *
 * 04Jul2002	Andrew Morton
 *		Initial version
 * 11Sep2002	janetinc@us.ibm.com
 * 		added readv/writev support.
 * 29Oct2002	Andrew Morton
 *		rewrote bio_add_page() support.
 * 30Oct2002	pbadari@us.ibm.com
 *		added support for non-aligned IO.
 * 06Nov2002	pbadari@us.ibm.com
 *		added asynchronous IO support.
 * 21Jul2003	nathans@sgi.com
 *		added IO completion notifier.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/highmem.h>
#include <linux/pagemap.h>
#include <linux/task_io_accounting_ops.h>
#include <linux/bio.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/blkdev.h>
#include <linux/buffer_head.h>
#include <linux/rwsem.h>
#include <linux/uio.h>
#include <linux/atomic.h>
#include <linux/prefetch.h>

#define DIO_PAGES	64



struct dio_submit {
	struct bio *bio;		
	unsigned blkbits;		
	unsigned blkfactor;		
	unsigned start_zero_done;	
	int pages_in_io;		
	size_t	size;			
	sector_t block_in_file;		
	unsigned blocks_available;	
	int reap_counter;		
	sector_t final_block_in_request;
	unsigned first_block_in_page;	
	int boundary;			
	get_block_t *get_block;		
	dio_submit_t *submit_io;	

	loff_t logical_offset_in_bio;	
	sector_t final_block_in_bio;	
	sector_t next_block_for_io;	

	struct page *cur_page;		
	unsigned cur_page_offset;	
	unsigned cur_page_len;		
	sector_t cur_page_block;	
	loff_t cur_page_fs_offset;	

	int curr_page;			
	int total_pages;		
	unsigned long curr_user_address;

	unsigned head;			
	unsigned tail;			
};

struct dio {
	int flags;			
	int rw;
	struct inode *inode;
	loff_t i_size;			
	dio_iodone_t *end_io;		

	void *private;			

	
	spinlock_t bio_lock;		
	int page_errors;		
	int is_async;			
	int io_error;			
	unsigned long refcount;		
	struct bio *bio_list;		
	struct task_struct *waiter;	

	
	struct kiocb *iocb;		
	ssize_t result;                 

	struct page *pages[DIO_PAGES];	
} ____cacheline_aligned_in_smp;

static struct kmem_cache *dio_cache __read_mostly;

static void __inode_dio_wait(struct inode *inode)
{
	wait_queue_head_t *wq = bit_waitqueue(&inode->i_state, __I_DIO_WAKEUP);
	DEFINE_WAIT_BIT(q, &inode->i_state, __I_DIO_WAKEUP);

	do {
		prepare_to_wait(wq, &q.wait, TASK_UNINTERRUPTIBLE);
		if (atomic_read(&inode->i_dio_count))
			schedule();
	} while (atomic_read(&inode->i_dio_count));
	finish_wait(wq, &q.wait);
}

void inode_dio_wait(struct inode *inode)
{
	if (atomic_read(&inode->i_dio_count))
		__inode_dio_wait(inode);
}
EXPORT_SYMBOL(inode_dio_wait);

void inode_dio_done(struct inode *inode)
{
	if (atomic_dec_and_test(&inode->i_dio_count))
		wake_up_bit(&inode->i_state, __I_DIO_WAKEUP);
}
EXPORT_SYMBOL(inode_dio_done);

static inline unsigned dio_pages_present(struct dio_submit *sdio)
{
	return sdio->tail - sdio->head;
}

static inline int dio_refill_pages(struct dio *dio, struct dio_submit *sdio)
{
	int ret;
	int nr_pages;

	nr_pages = min(sdio->total_pages - sdio->curr_page, DIO_PAGES);
	ret = get_user_pages_fast(
		sdio->curr_user_address,		
		nr_pages,			
		dio->rw == READ,		
		&dio->pages[0]);		

	if (ret < 0 && sdio->blocks_available && (dio->rw & WRITE)) {
		struct page *page = ZERO_PAGE(0);
		if (dio->page_errors == 0)
			dio->page_errors = ret;
		page_cache_get(page);
		dio->pages[0] = page;
		sdio->head = 0;
		sdio->tail = 1;
		ret = 0;
		goto out;
	}

	if (ret >= 0) {
		sdio->curr_user_address += ret * PAGE_SIZE;
		sdio->curr_page += ret;
		sdio->head = 0;
		sdio->tail = ret;
		ret = 0;
	}
out:
	return ret;	
}

static inline struct page *dio_get_page(struct dio *dio,
		struct dio_submit *sdio)
{
	if (dio_pages_present(sdio) == 0) {
		int ret;

		ret = dio_refill_pages(dio, sdio);
		if (ret)
			return ERR_PTR(ret);
		BUG_ON(dio_pages_present(sdio) == 0);
	}
	return dio->pages[sdio->head++];
}

static ssize_t dio_complete(struct dio *dio, loff_t offset, ssize_t ret, bool is_async)
{
	ssize_t transferred = 0;

	if (ret == -EIOCBQUEUED)
		ret = 0;

	if (dio->result) {
		transferred = dio->result;

		
		if ((dio->rw == READ) && ((offset + transferred) > dio->i_size))
			transferred = dio->i_size - offset;
	}

	if (ret == 0)
		ret = dio->page_errors;
	if (ret == 0)
		ret = dio->io_error;
	if (ret == 0)
		ret = transferred;

	if (dio->end_io && dio->result) {
		dio->end_io(dio->iocb, offset, transferred,
			    dio->private, ret, is_async);
	} else {
		if (is_async)
			aio_complete(dio->iocb, ret, 0);
		inode_dio_done(dio->inode);
	}

	return ret;
}

static int dio_bio_complete(struct dio *dio, struct bio *bio);
static void dio_bio_end_aio(struct bio *bio, int error)
{
	struct dio *dio = bio->bi_private;
	unsigned long remaining;
	unsigned long flags;

	
	dio_bio_complete(dio, bio);

	spin_lock_irqsave(&dio->bio_lock, flags);
	remaining = --dio->refcount;
	if (remaining == 1 && dio->waiter)
		wake_up_process(dio->waiter);
	spin_unlock_irqrestore(&dio->bio_lock, flags);

	if (remaining == 0) {
		dio_complete(dio, dio->iocb->ki_pos, 0, true);
		kmem_cache_free(dio_cache, dio);
	}
}

static void dio_bio_end_io(struct bio *bio, int error)
{
	struct dio *dio = bio->bi_private;
	unsigned long flags;

	spin_lock_irqsave(&dio->bio_lock, flags);
	bio->bi_private = dio->bio_list;
	dio->bio_list = bio;
	if (--dio->refcount == 1 && dio->waiter)
		wake_up_process(dio->waiter);
	spin_unlock_irqrestore(&dio->bio_lock, flags);
}

void dio_end_io(struct bio *bio, int error)
{
	struct dio *dio = bio->bi_private;

	if (dio->is_async)
		dio_bio_end_aio(bio, error);
	else
		dio_bio_end_io(bio, error);
}
EXPORT_SYMBOL_GPL(dio_end_io);

static inline void
dio_bio_alloc(struct dio *dio, struct dio_submit *sdio,
	      struct block_device *bdev,
	      sector_t first_sector, int nr_vecs)
{
	struct bio *bio;

	bio = bio_alloc(GFP_KERNEL, nr_vecs);

	bio->bi_bdev = bdev;
	bio->bi_sector = first_sector;
	if (dio->is_async)
		bio->bi_end_io = dio_bio_end_aio;
	else
		bio->bi_end_io = dio_bio_end_io;

	sdio->bio = bio;
	sdio->logical_offset_in_bio = sdio->cur_page_fs_offset;
}

/*
 * In the AIO read case we speculatively dirty the pages before starting IO.
 * During IO completion, any of these pages which happen to have been written
 * back will be redirtied by bio_check_pages_dirty().
 *
 * bios hold a dio reference between submit_bio and ->end_io.
 */
static inline void dio_bio_submit(struct dio *dio, struct dio_submit *sdio)
{
	struct bio *bio = sdio->bio;
	unsigned long flags;

	bio->bi_private = dio;

	spin_lock_irqsave(&dio->bio_lock, flags);
	dio->refcount++;
	spin_unlock_irqrestore(&dio->bio_lock, flags);

	if (dio->is_async && dio->rw == READ)
		bio_set_pages_dirty(bio);

	bio->bi_dio_inode = dio->inode;

	if (sdio->submit_io)
		sdio->submit_io(dio->rw, bio, dio->inode,
			       sdio->logical_offset_in_bio);
	else
		submit_bio(dio->rw, bio);

	sdio->bio = NULL;
	sdio->boundary = 0;
	sdio->logical_offset_in_bio = 0;
}

struct inode *dio_bio_get_inode(struct bio *bio)
{
	struct inode *inode = NULL;

	if (bio == NULL)
		return NULL;

	inode = bio->bi_dio_inode;

	return inode;
}
EXPORT_SYMBOL(dio_bio_get_inode);

static inline void dio_cleanup(struct dio *dio, struct dio_submit *sdio)
{
	while (dio_pages_present(sdio))
		page_cache_release(dio_get_page(dio, sdio));
}

static struct bio *dio_await_one(struct dio *dio)
{
	unsigned long flags;
	struct bio *bio = NULL;

	spin_lock_irqsave(&dio->bio_lock, flags);

	while (dio->refcount > 1 && dio->bio_list == NULL) {
		__set_current_state(TASK_UNINTERRUPTIBLE);
		dio->waiter = current;
		spin_unlock_irqrestore(&dio->bio_lock, flags);
		io_schedule();
		
		spin_lock_irqsave(&dio->bio_lock, flags);
		dio->waiter = NULL;
	}
	if (dio->bio_list) {
		bio = dio->bio_list;
		dio->bio_list = bio->bi_private;
	}
	spin_unlock_irqrestore(&dio->bio_lock, flags);
	return bio;
}

static int dio_bio_complete(struct dio *dio, struct bio *bio)
{
	const int uptodate = test_bit(BIO_UPTODATE, &bio->bi_flags);
	struct bio_vec *bvec = bio->bi_io_vec;
	int page_no;

	if (!uptodate)
		dio->io_error = -EIO;

	if (dio->is_async && dio->rw == READ) {
		bio_check_pages_dirty(bio);	
	} else {
		for (page_no = 0; page_no < bio->bi_vcnt; page_no++) {
			struct page *page = bvec[page_no].bv_page;

			if (dio->rw == READ && !PageCompound(page))
				set_page_dirty_lock(page);
			page_cache_release(page);
		}
		bio_put(bio);
	}
	return uptodate ? 0 : -EIO;
}

static void dio_await_completion(struct dio *dio)
{
	struct bio *bio;
	do {
		bio = dio_await_one(dio);
		if (bio)
			dio_bio_complete(dio, bio);
	} while (bio);
}

static inline int dio_bio_reap(struct dio *dio, struct dio_submit *sdio)
{
	int ret = 0;

	if (sdio->reap_counter++ >= 64) {
		while (dio->bio_list) {
			unsigned long flags;
			struct bio *bio;
			int ret2;

			spin_lock_irqsave(&dio->bio_lock, flags);
			bio = dio->bio_list;
			dio->bio_list = bio->bi_private;
			spin_unlock_irqrestore(&dio->bio_lock, flags);
			ret2 = dio_bio_complete(dio, bio);
			if (ret == 0)
				ret = ret2;
		}
		sdio->reap_counter = 0;
	}
	return ret;
}

static int get_more_blocks(struct dio *dio, struct dio_submit *sdio,
			   struct buffer_head *map_bh)
{
	int ret;
	sector_t fs_startblk;	
	sector_t fs_endblk;	
	unsigned long fs_count;	
	int create;

	/*
	 * If there was a memory error and we've overwritten all the
	 * mapped blocks then we can now return that memory error
	 */
	ret = dio->page_errors;
	if (ret == 0) {
		BUG_ON(sdio->block_in_file >= sdio->final_block_in_request);
		fs_startblk = sdio->block_in_file >> sdio->blkfactor;
		fs_endblk = (sdio->final_block_in_request - 1) >>
					sdio->blkfactor;
		fs_count = fs_endblk - fs_startblk + 1;

		map_bh->b_state = 0;
		map_bh->b_size = fs_count << dio->inode->i_blkbits;

		create = dio->rw & WRITE;
		if (dio->flags & DIO_SKIP_HOLES) {
			if (sdio->block_in_file < (i_size_read(dio->inode) >>
							sdio->blkbits))
				create = 0;
		}

		ret = (*sdio->get_block)(dio->inode, fs_startblk,
						map_bh, create);

		
		dio->private = map_bh->b_private;
	}
	return ret;
}

static inline int dio_new_bio(struct dio *dio, struct dio_submit *sdio,
		sector_t start_sector, struct buffer_head *map_bh)
{
	sector_t sector;
	int ret, nr_pages;

	ret = dio_bio_reap(dio, sdio);
	if (ret)
		goto out;
	sector = start_sector << (sdio->blkbits - 9);
	nr_pages = min(sdio->pages_in_io, bio_get_nr_vecs(map_bh->b_bdev));
	nr_pages = min(nr_pages, BIO_MAX_PAGES);
	BUG_ON(nr_pages <= 0);
	dio_bio_alloc(dio, sdio, map_bh->b_bdev, sector, nr_pages);
	sdio->boundary = 0;
out:
	return ret;
}

static inline int dio_bio_add_page(struct dio_submit *sdio)
{
	int ret;

	ret = bio_add_page(sdio->bio, sdio->cur_page,
			sdio->cur_page_len, sdio->cur_page_offset);
	if (ret == sdio->cur_page_len) {
		if ((sdio->cur_page_len + sdio->cur_page_offset) == PAGE_SIZE)
			sdio->pages_in_io--;
		page_cache_get(sdio->cur_page);
		sdio->final_block_in_bio = sdio->cur_page_block +
			(sdio->cur_page_len >> sdio->blkbits);
		ret = 0;
	} else {
		ret = 1;
	}
	return ret;
}
		
static inline int dio_send_cur_page(struct dio *dio, struct dio_submit *sdio,
		struct buffer_head *map_bh)
{
	int ret = 0;

	if (sdio->bio) {
		loff_t cur_offset = sdio->cur_page_fs_offset;
		loff_t bio_next_offset = sdio->logical_offset_in_bio +
			sdio->bio->bi_size;

		if (sdio->final_block_in_bio != sdio->cur_page_block ||
		    cur_offset != bio_next_offset)
			dio_bio_submit(dio, sdio);
		else if (sdio->boundary)
			dio_bio_submit(dio, sdio);
	}

	if (sdio->bio == NULL) {
		ret = dio_new_bio(dio, sdio, sdio->cur_page_block, map_bh);
		if (ret)
			goto out;
	}

	if (dio_bio_add_page(sdio) != 0) {
		dio_bio_submit(dio, sdio);
		ret = dio_new_bio(dio, sdio, sdio->cur_page_block, map_bh);
		if (ret == 0) {
			ret = dio_bio_add_page(sdio);
			BUG_ON(ret != 0);
		}
	}
out:
	return ret;
}

static inline int
submit_page_section(struct dio *dio, struct dio_submit *sdio, struct page *page,
		    unsigned offset, unsigned len, sector_t blocknr,
		    struct buffer_head *map_bh)
{
	int ret = 0;

	if (dio->rw & WRITE) {
		task_io_account_write(len);
		collect_io_stats(len, WRITE);
	}

	if (sdio->cur_page == page &&
	    sdio->cur_page_offset + sdio->cur_page_len == offset &&
	    sdio->cur_page_block +
	    (sdio->cur_page_len >> sdio->blkbits) == blocknr) {
		sdio->cur_page_len += len;

		if (sdio->boundary) {
			ret = dio_send_cur_page(dio, sdio, map_bh);
			page_cache_release(sdio->cur_page);
			sdio->cur_page = NULL;
		}
		goto out;
	}

	if (sdio->cur_page) {
		ret = dio_send_cur_page(dio, sdio, map_bh);
		page_cache_release(sdio->cur_page);
		sdio->cur_page = NULL;
		if (ret)
			goto out;
	}

	page_cache_get(page);		
	sdio->cur_page = page;
	sdio->cur_page_offset = offset;
	sdio->cur_page_len = len;
	sdio->cur_page_block = blocknr;
	sdio->cur_page_fs_offset = sdio->block_in_file << sdio->blkbits;
out:
	return ret;
}

static void clean_blockdev_aliases(struct dio *dio, struct buffer_head *map_bh)
{
	unsigned i;
	unsigned nblocks;

	nblocks = map_bh->b_size >> dio->inode->i_blkbits;

	for (i = 0; i < nblocks; i++) {
		unmap_underlying_metadata(map_bh->b_bdev,
					  map_bh->b_blocknr + i);
	}
}

static inline void dio_zero_block(struct dio *dio, struct dio_submit *sdio,
		int end, struct buffer_head *map_bh)
{
	unsigned dio_blocks_per_fs_block;
	unsigned this_chunk_blocks;	
	unsigned this_chunk_bytes;
	struct page *page;

	sdio->start_zero_done = 1;
	if (!sdio->blkfactor || !buffer_new(map_bh))
		return;

	dio_blocks_per_fs_block = 1 << sdio->blkfactor;
	this_chunk_blocks = sdio->block_in_file & (dio_blocks_per_fs_block - 1);

	if (!this_chunk_blocks)
		return;

	if (end) 
		this_chunk_blocks = dio_blocks_per_fs_block - this_chunk_blocks;

	this_chunk_bytes = this_chunk_blocks << sdio->blkbits;

	page = ZERO_PAGE(0);
	if (submit_page_section(dio, sdio, page, 0, this_chunk_bytes,
				sdio->next_block_for_io, map_bh))
		return;

	sdio->next_block_for_io += this_chunk_blocks;
}

static int do_direct_IO(struct dio *dio, struct dio_submit *sdio,
			struct buffer_head *map_bh)
{
	const unsigned blkbits = sdio->blkbits;
	const unsigned blocks_per_page = PAGE_SIZE >> blkbits;
	struct page *page;
	unsigned block_in_page;
	int ret = 0;

	
	block_in_page = sdio->first_block_in_page;

	while (sdio->block_in_file < sdio->final_block_in_request) {
		page = dio_get_page(dio, sdio);
		if (IS_ERR(page)) {
			ret = PTR_ERR(page);
			goto out;
		}

		while (block_in_page < blocks_per_page) {
			unsigned offset_in_page = block_in_page << blkbits;
			unsigned this_chunk_bytes;	
			unsigned this_chunk_blocks;	
			unsigned u;

			if (sdio->blocks_available == 0) {
				unsigned long blkmask;
				unsigned long dio_remainder;

				ret = get_more_blocks(dio, sdio, map_bh);
				if (ret) {
					page_cache_release(page);
					goto out;
				}
				if (!buffer_mapped(map_bh))
					goto do_holes;

				sdio->blocks_available =
						map_bh->b_size >> sdio->blkbits;
				sdio->next_block_for_io =
					map_bh->b_blocknr << sdio->blkfactor;
				if (buffer_new(map_bh))
					clean_blockdev_aliases(dio, map_bh);

				if (!sdio->blkfactor)
					goto do_holes;

				blkmask = (1 << sdio->blkfactor) - 1;
				dio_remainder = (sdio->block_in_file & blkmask);

				if (!buffer_new(map_bh))
					sdio->next_block_for_io += dio_remainder;
				sdio->blocks_available -= dio_remainder;
			}
do_holes:
			
			if (!buffer_mapped(map_bh)) {
				loff_t i_size_aligned;

				
				if (dio->rw & WRITE) {
					page_cache_release(page);
					return -ENOTBLK;
				}

				i_size_aligned = ALIGN(i_size_read(dio->inode),
							1 << blkbits);
				if (sdio->block_in_file >=
						i_size_aligned >> blkbits) {
					
					page_cache_release(page);
					goto out;
				}
				zero_user(page, block_in_page << blkbits,
						1 << blkbits);
				sdio->block_in_file++;
				block_in_page++;
				goto next_block;
			}

			if (unlikely(sdio->blkfactor && !sdio->start_zero_done))
				dio_zero_block(dio, sdio, 0, map_bh);

			this_chunk_blocks = sdio->blocks_available;
			u = (PAGE_SIZE - offset_in_page) >> blkbits;
			if (this_chunk_blocks > u)
				this_chunk_blocks = u;
			u = sdio->final_block_in_request - sdio->block_in_file;
			if (this_chunk_blocks > u)
				this_chunk_blocks = u;
			this_chunk_bytes = this_chunk_blocks << blkbits;
			BUG_ON(this_chunk_bytes == 0);

			sdio->boundary = buffer_boundary(map_bh);
			ret = submit_page_section(dio, sdio, page,
						  offset_in_page,
						  this_chunk_bytes,
						  sdio->next_block_for_io,
						  map_bh);
			if (ret) {
				page_cache_release(page);
				goto out;
			}
			sdio->next_block_for_io += this_chunk_blocks;

			sdio->block_in_file += this_chunk_blocks;
			block_in_page += this_chunk_blocks;
			sdio->blocks_available -= this_chunk_blocks;
next_block:
			BUG_ON(sdio->block_in_file > sdio->final_block_in_request);
			if (sdio->block_in_file == sdio->final_block_in_request)
				break;
		}

		
		page_cache_release(page);
		block_in_page = 0;
	}
out:
	return ret;
}

static inline int drop_refcount(struct dio *dio)
{
	int ret2;
	unsigned long flags;

	spin_lock_irqsave(&dio->bio_lock, flags);
	ret2 = --dio->refcount;
	spin_unlock_irqrestore(&dio->bio_lock, flags);
	return ret2;
}

static inline ssize_t
do_blockdev_direct_IO(int rw, struct kiocb *iocb, struct inode *inode,
	struct block_device *bdev, const struct iovec *iov, loff_t offset, 
	unsigned long nr_segs, get_block_t get_block, dio_iodone_t end_io,
	dio_submit_t submit_io,	int flags)
{
	int seg;
	size_t size;
	unsigned long addr;
	unsigned blkbits = inode->i_blkbits;
	unsigned blocksize_mask = (1 << blkbits) - 1;
	ssize_t retval = -EINVAL;
	loff_t end = offset;
	struct dio *dio;
	struct dio_submit sdio = { 0, };
	unsigned long user_addr;
	size_t bytes;
	struct buffer_head map_bh = { 0, };

	if (rw & WRITE)
		rw = WRITE_ODIRECT;


	if (offset & blocksize_mask) {
		if (bdev)
			blkbits = blksize_bits(bdev_logical_block_size(bdev));
		blocksize_mask = (1 << blkbits) - 1;
		if (offset & blocksize_mask)
			goto out;
	}

	
	for (seg = 0; seg < nr_segs; seg++) {
		addr = (unsigned long)iov[seg].iov_base;
		size = iov[seg].iov_len;
		end += size;
		if (unlikely((addr & blocksize_mask) ||
			     (size & blocksize_mask))) {
			if (bdev)
				blkbits = blksize_bits(
					 bdev_logical_block_size(bdev));
			blocksize_mask = (1 << blkbits) - 1;
			if ((addr & blocksize_mask) || (size & blocksize_mask))
				goto out;
		}
	}

	
	if (rw == READ && end == offset)
		return 0;

	dio = kmem_cache_alloc(dio_cache, GFP_KERNEL);
	retval = -ENOMEM;
	if (!dio)
		goto out;
	memset(dio, 0, offsetof(struct dio, pages));

	dio->flags = flags;
	if (dio->flags & DIO_LOCKING) {
		if (rw == READ) {
			struct address_space *mapping =
					iocb->ki_filp->f_mapping;

			
			mutex_lock(&inode->i_mutex);

			retval = filemap_write_and_wait_range(mapping, offset,
							      end - 1);
			if (retval) {
				mutex_unlock(&inode->i_mutex);
				kmem_cache_free(dio_cache, dio);
				goto out;
			}
		}
	}

	atomic_inc(&inode->i_dio_count);

	dio->is_async = !is_sync_kiocb(iocb) && !((rw & WRITE) &&
		(end > i_size_read(inode)));

	retval = 0;

	dio->inode = inode;
	dio->rw = rw;
	sdio.blkbits = blkbits;
	sdio.blkfactor = inode->i_blkbits - blkbits;
	sdio.block_in_file = offset >> blkbits;

	sdio.get_block = get_block;
	dio->end_io = end_io;
	sdio.submit_io = submit_io;
	sdio.final_block_in_bio = -1;
	sdio.next_block_for_io = -1;

	dio->iocb = iocb;
	dio->i_size = i_size_read(inode);

	spin_lock_init(&dio->bio_lock);
	dio->refcount = 1;

	if (unlikely(sdio.blkfactor))
		sdio.pages_in_io = 2;

	for (seg = 0; seg < nr_segs; seg++) {
		user_addr = (unsigned long)iov[seg].iov_base;
		sdio.pages_in_io +=
			((user_addr + iov[seg].iov_len + PAGE_SIZE-1) /
				PAGE_SIZE - user_addr / PAGE_SIZE);
	}

	for (seg = 0; seg < nr_segs; seg++) {
		user_addr = (unsigned long)iov[seg].iov_base;
		sdio.size += bytes = iov[seg].iov_len;

		
		sdio.first_block_in_page = (user_addr & ~PAGE_MASK) >> blkbits;
		sdio.final_block_in_request = sdio.block_in_file +
						(bytes >> blkbits);
		
		sdio.head = 0;
		sdio.tail = 0;
		sdio.curr_page = 0;

		sdio.total_pages = 0;
		if (user_addr & (PAGE_SIZE-1)) {
			sdio.total_pages++;
			bytes -= PAGE_SIZE - (user_addr & (PAGE_SIZE - 1));
		}
		sdio.total_pages += (bytes + PAGE_SIZE - 1) / PAGE_SIZE;
		sdio.curr_user_address = user_addr;

		retval = do_direct_IO(dio, &sdio, &map_bh);

		dio->result += iov[seg].iov_len -
			((sdio.final_block_in_request - sdio.block_in_file) <<
					blkbits);

		if (retval) {
			dio_cleanup(dio, &sdio);
			break;
		}
	} 

	if (retval == -ENOTBLK) {
		retval = 0;
	}
	/*
	 * There may be some unwritten disk at the end of a part-written
	 * fs-block-sized block.  Go zero that now.
	 */
	dio_zero_block(dio, &sdio, 1, &map_bh);

	if (sdio.cur_page) {
		ssize_t ret2;

		ret2 = dio_send_cur_page(dio, &sdio, &map_bh);
		if (retval == 0)
			retval = ret2;
		page_cache_release(sdio.cur_page);
		sdio.cur_page = NULL;
	}
	if (sdio.bio)
		dio_bio_submit(dio, &sdio);

	dio_cleanup(dio, &sdio);

	if (rw == READ && (dio->flags & DIO_LOCKING))
		mutex_unlock(&dio->inode->i_mutex);

	BUG_ON(retval == -EIOCBQUEUED);
	if (dio->is_async && retval == 0 && dio->result &&
	    ((rw & READ) || (dio->result == sdio.size)))
		retval = -EIOCBQUEUED;

	if (retval != -EIOCBQUEUED)
		dio_await_completion(dio);

	if (drop_refcount(dio) == 0) {
		retval = dio_complete(dio, offset, retval, false);
		kmem_cache_free(dio_cache, dio);
	} else
		BUG_ON(retval != -EIOCBQUEUED);

out:
	return retval;
}

ssize_t
__blockdev_direct_IO(int rw, struct kiocb *iocb, struct inode *inode,
	struct block_device *bdev, const struct iovec *iov, loff_t offset,
	unsigned long nr_segs, get_block_t get_block, dio_iodone_t end_io,
	dio_submit_t submit_io,	int flags)
{
	prefetch(&bdev->bd_disk->part_tbl);
	prefetch(bdev->bd_queue);
	prefetch((char *)bdev->bd_queue + SMP_CACHE_BYTES);

	return do_blockdev_direct_IO(rw, iocb, inode, bdev, iov, offset,
				     nr_segs, get_block, end_io,
				     submit_io, flags);
}

EXPORT_SYMBOL(__blockdev_direct_IO);

static __init int dio_init(void)
{
	dio_cache = KMEM_CACHE(dio, SLAB_PANIC);
	return 0;
}
module_init(dio_init)
