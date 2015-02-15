/*
 * Compressed RAM block device
 *
 * Copyright (C) 2008, 2009, 2010  Nitin Gupta
 *               2012, 2013 Minchan Kim
 *
 * This code is released using a dual license strategy: BSD/GPL
 * You can choose the licence that better fits your requirements.
 *
 * Released under the terms of 3-clause BSD License
 * Released under the terms of GNU General Public License Version 2.0
 *
 */

#ifndef _ZRAM_DRV_H_
#define _ZRAM_DRV_H_

#include <linux/spinlock.h>
#include <linux/mutex.h>

#include "../zsmalloc/zsmalloc.h"

static const unsigned max_num_devices = 32;


static const size_t max_zpage_size = PAGE_SIZE / 10 * 9;



#define SECTOR_SHIFT		9
#define SECTOR_SIZE		(1 << SECTOR_SHIFT)
#define SECTORS_PER_PAGE_SHIFT	(PAGE_SHIFT - SECTOR_SHIFT)
#define SECTORS_PER_PAGE	(1 << SECTORS_PER_PAGE_SHIFT)
#define ZRAM_LOGICAL_BLOCK_SHIFT 12
#define ZRAM_LOGICAL_BLOCK_SIZE	(1 << ZRAM_LOGICAL_BLOCK_SHIFT)
#define ZRAM_SECTOR_PER_LOGICAL_BLOCK	\
	(1 << (ZRAM_LOGICAL_BLOCK_SHIFT - SECTOR_SHIFT))

enum zram_pageflags {
	
	ZRAM_ZERO,

	__NR_ZRAM_PAGEFLAGS,
};


struct table {
	unsigned long handle;
	u16 size;	
	u8 count;	
	u8 flags;
} __aligned(4);

struct zram_stats {
	atomic64_t compr_size;	
	atomic64_t num_reads;	
	atomic64_t num_writes;	
	atomic64_t failed_reads;	
	atomic64_t failed_writes;	
	atomic64_t invalid_io;	
	atomic64_t notify_free;	
	atomic_t pages_zero;		
	atomic_t pages_stored;	
	atomic_t good_compress;	
	atomic_t bad_compress;	
};

struct zram_meta {
	rwlock_t tb_lock;	
	void *compress_workmem;
	void *compress_buffer;
	struct table *table;
	struct zs_pool *mem_pool;
	struct mutex buffer_lock; 
};

struct zram {
	struct zram_meta *meta;
	struct request_queue *queue;
	struct gendisk *disk;
	int init_done;
	
	struct rw_semaphore init_lock;
	u64 disksize;	

	struct zram_stats stats;
};
#endif
