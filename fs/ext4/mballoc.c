/*
 * Copyright (c) 2003-2006, Cluster File Systems, Inc, info@clusterfs.com
 * Written by Alex Tomas <alex@clusterfs.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public Licens
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-
 */



#include "ext4_jbd2.h"
#include "mballoc.h"
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <trace/events/ext4.h>





static struct kmem_cache *ext4_pspace_cachep;
static struct kmem_cache *ext4_ac_cachep;
static struct kmem_cache *ext4_free_data_cachep;

#define NR_GRPINFO_CACHES 8
static struct kmem_cache *ext4_groupinfo_caches[NR_GRPINFO_CACHES];

static const char *ext4_groupinfo_slab_names[NR_GRPINFO_CACHES] = {
	"ext4_groupinfo_1k", "ext4_groupinfo_2k", "ext4_groupinfo_4k",
	"ext4_groupinfo_8k", "ext4_groupinfo_16k", "ext4_groupinfo_32k",
	"ext4_groupinfo_64k", "ext4_groupinfo_128k"
};

static void ext4_mb_generate_from_pa(struct super_block *sb, void *bitmap,
					ext4_group_t group);
static void ext4_mb_generate_from_freelist(struct super_block *sb, void *bitmap,
						ext4_group_t group);
static void ext4_free_data_callback(struct super_block *sb,
				struct ext4_journal_cb_entry *jce, int rc);

static inline void *mb_correct_addr_and_bit(int *bit, void *addr)
{
#if BITS_PER_LONG == 64
	*bit += ((unsigned long) addr & 7UL) << 3;
	addr = (void *) ((unsigned long) addr & ~7UL);
#elif BITS_PER_LONG == 32
	*bit += ((unsigned long) addr & 3UL) << 3;
	addr = (void *) ((unsigned long) addr & ~3UL);
#else
#error "how many bits you are?!"
#endif
	return addr;
}

static inline int mb_test_bit(int bit, void *addr)
{
	addr = mb_correct_addr_and_bit(&bit, addr);
	return ext4_test_bit(bit, addr);
}

static inline void mb_set_bit(int bit, void *addr)
{
	addr = mb_correct_addr_and_bit(&bit, addr);
	ext4_set_bit(bit, addr);
}

static inline void mb_clear_bit(int bit, void *addr)
{
	addr = mb_correct_addr_and_bit(&bit, addr);
	ext4_clear_bit(bit, addr);
}

static inline int mb_find_next_zero_bit(void *addr, int max, int start)
{
	int fix = 0, ret, tmpmax;
	addr = mb_correct_addr_and_bit(&fix, addr);
	tmpmax = max + fix;
	start += fix;

	ret = ext4_find_next_zero_bit(addr, tmpmax, start) - fix;
	if (ret > max)
		return max;
	return ret;
}

static inline int mb_find_next_bit(void *addr, int max, int start)
{
	int fix = 0, ret, tmpmax;
	addr = mb_correct_addr_and_bit(&fix, addr);
	tmpmax = max + fix;
	start += fix;

	ret = ext4_find_next_bit(addr, tmpmax, start) - fix;
	if (ret > max)
		return max;
	return ret;
}

static void *mb_find_buddy(struct ext4_buddy *e4b, int order, int *max)
{
	char *bb;

	BUG_ON(e4b->bd_bitmap == e4b->bd_buddy);
	BUG_ON(max == NULL);

	if (order > e4b->bd_blkbits + 1) {
		*max = 0;
		return NULL;
	}

	
	if (order == 0) {
		*max = 1 << (e4b->bd_blkbits + 3);
		return e4b->bd_bitmap;
	}

	bb = e4b->bd_buddy + EXT4_SB(e4b->bd_sb)->s_mb_offsets[order];
	*max = EXT4_SB(e4b->bd_sb)->s_mb_maxs[order];

	return bb;
}

#ifdef DOUBLE_CHECK
static void mb_free_blocks_double(struct inode *inode, struct ext4_buddy *e4b,
			   int first, int count)
{
	int i;
	struct super_block *sb = e4b->bd_sb;

	if (unlikely(e4b->bd_info->bb_bitmap == NULL))
		return;
	assert_spin_locked(ext4_group_lock_ptr(sb, e4b->bd_group));
	for (i = 0; i < count; i++) {
		if (!mb_test_bit(first + i, e4b->bd_info->bb_bitmap)) {
			ext4_fsblk_t blocknr;

			blocknr = ext4_group_first_block_no(sb, e4b->bd_group);
			blocknr += EXT4_C2B(EXT4_SB(sb), first + i);
			ext4_grp_locked_error(sb, e4b->bd_group,
					      inode ? inode->i_ino : 0,
					      blocknr,
					      "freeing block already freed "
					      "(bit %u)",
					      first + i);
		}
		mb_clear_bit(first + i, e4b->bd_info->bb_bitmap);
	}
}

static void mb_mark_used_double(struct ext4_buddy *e4b, int first, int count)
{
	int i;

	if (unlikely(e4b->bd_info->bb_bitmap == NULL))
		return;
	assert_spin_locked(ext4_group_lock_ptr(e4b->bd_sb, e4b->bd_group));
	for (i = 0; i < count; i++) {
		BUG_ON(mb_test_bit(first + i, e4b->bd_info->bb_bitmap));
		mb_set_bit(first + i, e4b->bd_info->bb_bitmap);
	}
}

static void mb_cmp_bitmaps(struct ext4_buddy *e4b, void *bitmap)
{
	if (memcmp(e4b->bd_info->bb_bitmap, bitmap, e4b->bd_sb->s_blocksize)) {
		unsigned char *b1, *b2;
		int i;
		b1 = (unsigned char *) e4b->bd_info->bb_bitmap;
		b2 = (unsigned char *) bitmap;
		for (i = 0; i < e4b->bd_sb->s_blocksize; i++) {
			if (b1[i] != b2[i]) {
				ext4_msg(e4b->bd_sb, KERN_ERR,
					 "corruption in group %u "
					 "at byte %u(%u): %x in copy != %x "
					 "on disk/prealloc",
					 e4b->bd_group, i, i * 8, b1[i], b2[i]);
				BUG();
			}
		}
	}
}

#else
static inline void mb_free_blocks_double(struct inode *inode,
				struct ext4_buddy *e4b, int first, int count)
{
	return;
}
static inline void mb_mark_used_double(struct ext4_buddy *e4b,
						int first, int count)
{
	return;
}
static inline void mb_cmp_bitmaps(struct ext4_buddy *e4b, void *bitmap)
{
	return;
}
#endif

#ifdef AGGRESSIVE_CHECK

#define MB_CHECK_ASSERT(assert)						\
do {									\
	if (!(assert)) {						\
		printk(KERN_EMERG					\
			"Assertion failure in %s() at %s:%d: \"%s\"\n",	\
			function, file, line, # assert);		\
		BUG();							\
	}								\
} while (0)

static int __mb_check_buddy(struct ext4_buddy *e4b, char *file,
				const char *function, int line)
{
	struct super_block *sb = e4b->bd_sb;
	int order = e4b->bd_blkbits + 1;
	int max;
	int max2;
	int i;
	int j;
	int k;
	int count;
	struct ext4_group_info *grp;
	int fragments = 0;
	int fstart;
	struct list_head *cur;
	void *buddy;
	void *buddy2;

	{
		static int mb_check_counter;
		if (mb_check_counter++ % 100 != 0)
			return 0;
	}

	while (order > 1) {
		buddy = mb_find_buddy(e4b, order, &max);
		MB_CHECK_ASSERT(buddy);
		buddy2 = mb_find_buddy(e4b, order - 1, &max2);
		MB_CHECK_ASSERT(buddy2);
		MB_CHECK_ASSERT(buddy != buddy2);
		MB_CHECK_ASSERT(max * 2 == max2);

		count = 0;
		for (i = 0; i < max; i++) {

			if (mb_test_bit(i, buddy)) {
				
				if (!mb_test_bit(i << 1, buddy2)) {
					MB_CHECK_ASSERT(
						mb_test_bit((i<<1)+1, buddy2));
				} else if (!mb_test_bit((i << 1) + 1, buddy2)) {
					MB_CHECK_ASSERT(
						mb_test_bit(i << 1, buddy2));
				}
				continue;
			}

			
			MB_CHECK_ASSERT(mb_test_bit(i << 1, buddy2));
			MB_CHECK_ASSERT(mb_test_bit((i << 1) + 1, buddy2));

			for (j = 0; j < (1 << order); j++) {
				k = (i * (1 << order)) + j;
				MB_CHECK_ASSERT(
					!mb_test_bit(k, e4b->bd_bitmap));
			}
			count++;
		}
		MB_CHECK_ASSERT(e4b->bd_info->bb_counters[order] == count);
		order--;
	}

	fstart = -1;
	buddy = mb_find_buddy(e4b, 0, &max);
	for (i = 0; i < max; i++) {
		if (!mb_test_bit(i, buddy)) {
			MB_CHECK_ASSERT(i >= e4b->bd_info->bb_first_free);
			if (fstart == -1) {
				fragments++;
				fstart = i;
			}
			continue;
		}
		fstart = -1;
		
		for (j = 0; j < e4b->bd_blkbits + 1; j++) {
			buddy2 = mb_find_buddy(e4b, j, &max2);
			k = i >> j;
			MB_CHECK_ASSERT(k < max2);
			MB_CHECK_ASSERT(mb_test_bit(k, buddy2));
		}
	}
	MB_CHECK_ASSERT(!EXT4_MB_GRP_NEED_INIT(e4b->bd_info));
	MB_CHECK_ASSERT(e4b->bd_info->bb_fragments == fragments);

	grp = ext4_get_group_info(sb, e4b->bd_group);
	list_for_each(cur, &grp->bb_prealloc_list) {
		ext4_group_t groupnr;
		struct ext4_prealloc_space *pa;
		pa = list_entry(cur, struct ext4_prealloc_space, pa_group_list);
		ext4_get_group_no_and_offset(sb, pa->pa_pstart, &groupnr, &k);
		MB_CHECK_ASSERT(groupnr == e4b->bd_group);
		for (i = 0; i < pa->pa_len; i++)
			MB_CHECK_ASSERT(mb_test_bit(k + i, buddy));
	}
	return 0;
}
#undef MB_CHECK_ASSERT
#define mb_check_buddy(e4b) __mb_check_buddy(e4b,	\
					__FILE__, __func__, __LINE__)
#else
#define mb_check_buddy(e4b)
#endif

static void ext4_mb_mark_free_simple(struct super_block *sb,
				void *buddy, ext4_grpblk_t first, ext4_grpblk_t len,
					struct ext4_group_info *grp)
{
	struct ext4_sb_info *sbi = EXT4_SB(sb);
	ext4_grpblk_t min;
	ext4_grpblk_t max;
	ext4_grpblk_t chunk;
	unsigned short border;

	BUG_ON(len > EXT4_CLUSTERS_PER_GROUP(sb));

	border = 2 << sb->s_blocksize_bits;

	while (len > 0) {
		
		max = ffs(first | border) - 1;

		
		min = fls(len) - 1;

		if (max < min)
			min = max;
		chunk = 1 << min;

		
		grp->bb_counters[min]++;
		if (min > 0)
			mb_clear_bit(first >> min,
				     buddy + sbi->s_mb_offsets[min]);

		len -= chunk;
		first += chunk;
	}
}

static void
mb_set_largest_free_order(struct super_block *sb, struct ext4_group_info *grp)
{
	int i;
	int bits;

	grp->bb_largest_free_order = -1; 

	bits = sb->s_blocksize_bits + 1;
	for (i = bits; i >= 0; i--) {
		if (grp->bb_counters[i] > 0) {
			grp->bb_largest_free_order = i;
			break;
		}
	}
}

static noinline_for_stack
void ext4_mb_generate_buddy(struct super_block *sb,
				void *buddy, void *bitmap, ext4_group_t group)
{
	struct ext4_group_info *grp = ext4_get_group_info(sb, group);
	ext4_grpblk_t max = EXT4_CLUSTERS_PER_GROUP(sb);
	ext4_grpblk_t i = 0;
	ext4_grpblk_t first;
	ext4_grpblk_t len;
	unsigned free = 0;
	unsigned fragments = 0;
	unsigned long long period = get_cycles();

	i = mb_find_next_zero_bit(bitmap, max, 0);
	grp->bb_first_free = i;
	while (i < max) {
		fragments++;
		first = i;
		i = mb_find_next_bit(bitmap, max, i);
		len = i - first;
		free += len;
		if (len > 1)
			ext4_mb_mark_free_simple(sb, buddy, first, len, grp);
		else
			grp->bb_counters[0]++;
		if (i < max)
			i = mb_find_next_zero_bit(bitmap, max, i);
	}
	grp->bb_fragments = fragments;

	if (free != grp->bb_free) {
		ext4_grp_locked_error(sb, group, 0, 0,
				      "%u clusters in bitmap, %u in gd",
				      free, grp->bb_free);
		grp->bb_free = free;
	}
	mb_set_largest_free_order(sb, grp);

	clear_bit(EXT4_GROUP_INFO_NEED_INIT_BIT, &(grp->bb_state));

	period = get_cycles() - period;
	spin_lock(&EXT4_SB(sb)->s_bal_lock);
	EXT4_SB(sb)->s_mb_buddies_generated++;
	EXT4_SB(sb)->s_mb_generation_time += period;
	spin_unlock(&EXT4_SB(sb)->s_bal_lock);
}


static int ext4_mb_init_cache(struct page *page, char *incore)
{
	ext4_group_t ngroups;
	int blocksize;
	int blocks_per_page;
	int groups_per_page;
	int err = 0;
	int i;
	ext4_group_t first_group, group;
	int first_block;
	struct super_block *sb;
	struct buffer_head *bhs;
	struct buffer_head **bh;
	struct inode *inode;
	char *data;
	char *bitmap;
	struct ext4_group_info *grinfo;

	mb_debug(1, "init page %lu\n", page->index);

	inode = page->mapping->host;
	sb = inode->i_sb;
	ngroups = ext4_get_groups_count(sb);
	blocksize = 1 << inode->i_blkbits;
	blocks_per_page = PAGE_CACHE_SIZE / blocksize;

	groups_per_page = blocks_per_page >> 1;
	if (groups_per_page == 0)
		groups_per_page = 1;

	
	if (groups_per_page > 1) {
		i = sizeof(struct buffer_head *) * groups_per_page;
		bh = kzalloc(i, GFP_NOFS);
		if (bh == NULL) {
			err = -ENOMEM;
			goto out;
		}
	} else
		bh = &bhs;

	first_group = page->index * blocks_per_page / 2;

	
	for (i = 0, group = first_group; i < groups_per_page; i++, group++) {
		if (group >= ngroups)
			break;

		grinfo = ext4_get_group_info(sb, group);
		if (PageUptodate(page) && !EXT4_MB_GRP_NEED_INIT(grinfo)) {
			bh[i] = NULL;
			continue;
		}
		if (!(bh[i] = ext4_read_block_bitmap_nowait(sb, group))) {
			err = -ENOMEM;
			goto out;
		}
		mb_debug(1, "read bitmap for group %u\n", group);
	}

	
	for (i = 0, group = first_group; i < groups_per_page; i++, group++) {
		if (bh[i] && ext4_wait_block_bitmap(sb, group, bh[i])) {
			err = -EIO;
			goto out;
		}
	}

	first_block = page->index * blocks_per_page;
	for (i = 0; i < blocks_per_page; i++) {
		int group;

		group = (first_block + i) >> 1;
		if (group >= ngroups)
			break;

		if (!bh[group - first_group])
			
			continue;

		data = page_address(page) + (i * blocksize);
		bitmap = bh[group - first_group]->b_data;

		if ((first_block + i) & 1) {
			
			BUG_ON(incore == NULL);
			mb_debug(1, "put buddy for group %u in page %lu/%x\n",
				group, page->index, i * blocksize);
			trace_ext4_mb_buddy_bitmap_load(sb, group);
			grinfo = ext4_get_group_info(sb, group);
			grinfo->bb_fragments = 0;
			memset(grinfo->bb_counters, 0,
			       sizeof(*grinfo->bb_counters) *
				(sb->s_blocksize_bits+2));
			ext4_lock_group(sb, group);
			
			memset(data, 0xff, blocksize);
			ext4_mb_generate_buddy(sb, data, incore, group);
			ext4_unlock_group(sb, group);
			incore = NULL;
		} else {
			
			BUG_ON(incore != NULL);
			mb_debug(1, "put bitmap for group %u in page %lu/%x\n",
				group, page->index, i * blocksize);
			trace_ext4_mb_bitmap_load(sb, group);

			
			ext4_lock_group(sb, group);
			memcpy(data, bitmap, blocksize);

			
			ext4_mb_generate_from_pa(sb, data, group);
			ext4_mb_generate_from_freelist(sb, data, group);
			ext4_unlock_group(sb, group);

			incore = data;
		}
	}
	SetPageUptodate(page);

out:
	if (bh) {
		for (i = 0; i < groups_per_page; i++)
			brelse(bh[i]);
		if (bh != &bhs)
			kfree(bh);
	}
	return err;
}

static int ext4_mb_get_buddy_page_lock(struct super_block *sb,
		ext4_group_t group, struct ext4_buddy *e4b)
{
	struct inode *inode = EXT4_SB(sb)->s_buddy_cache;
	int block, pnum, poff;
	int blocks_per_page;
	struct page *page;

	e4b->bd_buddy_page = NULL;
	e4b->bd_bitmap_page = NULL;

	blocks_per_page = PAGE_CACHE_SIZE / sb->s_blocksize;
	block = group * 2;
	pnum = block / blocks_per_page;
	poff = block % blocks_per_page;
	page = find_or_create_page(inode->i_mapping, pnum, GFP_NOFS);
	if (!page)
		return -EIO;
	BUG_ON(page->mapping != inode->i_mapping);
	e4b->bd_bitmap_page = page;
	e4b->bd_bitmap = page_address(page) + (poff * sb->s_blocksize);

	if (blocks_per_page >= 2) {
		
		return 0;
	}

	block++;
	pnum = block / blocks_per_page;
	poff = block % blocks_per_page;
	page = find_or_create_page(inode->i_mapping, pnum, GFP_NOFS);
	if (!page)
		return -EIO;
	BUG_ON(page->mapping != inode->i_mapping);
	e4b->bd_buddy_page = page;
	return 0;
}

static void ext4_mb_put_buddy_page_lock(struct ext4_buddy *e4b)
{
	if (e4b->bd_bitmap_page) {
		unlock_page(e4b->bd_bitmap_page);
		page_cache_release(e4b->bd_bitmap_page);
	}
	if (e4b->bd_buddy_page) {
		unlock_page(e4b->bd_buddy_page);
		page_cache_release(e4b->bd_buddy_page);
	}
}

static noinline_for_stack
int ext4_mb_init_group(struct super_block *sb, ext4_group_t group)
{

	struct ext4_group_info *this_grp;
	struct ext4_buddy e4b;
	struct page *page;
	int ret = 0;

	mb_debug(1, "init group %u\n", group);
	this_grp = ext4_get_group_info(sb, group);
	ret = ext4_mb_get_buddy_page_lock(sb, group, &e4b);
	if (ret || !EXT4_MB_GRP_NEED_INIT(this_grp)) {
		goto err;
	}

	page = e4b.bd_bitmap_page;
	ret = ext4_mb_init_cache(page, NULL);
	if (ret)
		goto err;
	if (!PageUptodate(page)) {
		ret = -EIO;
		goto err;
	}
	mark_page_accessed(page);

	if (e4b.bd_buddy_page == NULL) {
		ret = 0;
		goto err;
	}
	
	page = e4b.bd_buddy_page;
	ret = ext4_mb_init_cache(page, e4b.bd_bitmap);
	if (ret)
		goto err;
	if (!PageUptodate(page)) {
		ret = -EIO;
		goto err;
	}
	mark_page_accessed(page);
err:
	ext4_mb_put_buddy_page_lock(&e4b);
	return ret;
}

static noinline_for_stack int
ext4_mb_load_buddy(struct super_block *sb, ext4_group_t group,
					struct ext4_buddy *e4b)
{
	int blocks_per_page;
	int block;
	int pnum;
	int poff;
	struct page *page;
	int ret;
	struct ext4_group_info *grp;
	struct ext4_sb_info *sbi = EXT4_SB(sb);
	struct inode *inode = sbi->s_buddy_cache;

	mb_debug(1, "load group %u\n", group);

	blocks_per_page = PAGE_CACHE_SIZE / sb->s_blocksize;
	grp = ext4_get_group_info(sb, group);

	e4b->bd_blkbits = sb->s_blocksize_bits;
	e4b->bd_info = grp;
	e4b->bd_sb = sb;
	e4b->bd_group = group;
	e4b->bd_buddy_page = NULL;
	e4b->bd_bitmap_page = NULL;

	if (unlikely(EXT4_MB_GRP_NEED_INIT(grp))) {
		ret = ext4_mb_init_group(sb, group);
		if (ret)
			return ret;
	}

	block = group * 2;
	pnum = block / blocks_per_page;
	poff = block % blocks_per_page;

	page = find_get_page(inode->i_mapping, pnum);
	if (page == NULL || !PageUptodate(page)) {
		if (page)
			page_cache_release(page);
		page = find_or_create_page(inode->i_mapping, pnum, GFP_NOFS);
		if (page) {
			BUG_ON(page->mapping != inode->i_mapping);
			if (!PageUptodate(page)) {
				ret = ext4_mb_init_cache(page, NULL);
				if (ret) {
					unlock_page(page);
					goto err;
				}
				mb_cmp_bitmaps(e4b, page_address(page) +
					       (poff * sb->s_blocksize));
			}
			unlock_page(page);
		}
	}
	if (page == NULL || !PageUptodate(page)) {
		ret = -EIO;
		goto err;
	}
	e4b->bd_bitmap_page = page;
	e4b->bd_bitmap = page_address(page) + (poff * sb->s_blocksize);
	mark_page_accessed(page);

	block++;
	pnum = block / blocks_per_page;
	poff = block % blocks_per_page;

	page = find_get_page(inode->i_mapping, pnum);
	if (page == NULL || !PageUptodate(page)) {
		if (page)
			page_cache_release(page);
		page = find_or_create_page(inode->i_mapping, pnum, GFP_NOFS);
		if (page) {
			BUG_ON(page->mapping != inode->i_mapping);
			if (!PageUptodate(page)) {
				ret = ext4_mb_init_cache(page, e4b->bd_bitmap);
				if (ret) {
					unlock_page(page);
					goto err;
				}
			}
			unlock_page(page);
		}
	}
	if (page == NULL || !PageUptodate(page)) {
		ret = -EIO;
		goto err;
	}
	e4b->bd_buddy_page = page;
	e4b->bd_buddy = page_address(page) + (poff * sb->s_blocksize);
	mark_page_accessed(page);

	BUG_ON(e4b->bd_bitmap_page == NULL);
	BUG_ON(e4b->bd_buddy_page == NULL);

	return 0;

err:
	if (page)
		page_cache_release(page);
	if (e4b->bd_bitmap_page)
		page_cache_release(e4b->bd_bitmap_page);
	if (e4b->bd_buddy_page)
		page_cache_release(e4b->bd_buddy_page);
	e4b->bd_buddy = NULL;
	e4b->bd_bitmap = NULL;
	return ret;
}

static void ext4_mb_unload_buddy(struct ext4_buddy *e4b)
{
	if (e4b->bd_bitmap_page)
		page_cache_release(e4b->bd_bitmap_page);
	if (e4b->bd_buddy_page)
		page_cache_release(e4b->bd_buddy_page);
}


static int mb_find_order_for_block(struct ext4_buddy *e4b, int block)
{
	int order = 1;
	void *bb;

	BUG_ON(e4b->bd_bitmap == e4b->bd_buddy);
	BUG_ON(block >= (1 << (e4b->bd_blkbits + 3)));

	bb = e4b->bd_buddy;
	while (order <= e4b->bd_blkbits + 1) {
		block = block >> 1;
		if (!mb_test_bit(block, bb)) {
			
			return order;
		}
		bb += 1 << (e4b->bd_blkbits - order);
		order++;
	}
	return 0;
}

static void mb_clear_bits(void *bm, int cur, int len)
{
	__u32 *addr;

	len = cur + len;
	while (cur < len) {
		if ((cur & 31) == 0 && (len - cur) >= 32) {
			
			addr = bm + (cur >> 3);
			*addr = 0;
			cur += 32;
			continue;
		}
		mb_clear_bit(cur, bm);
		cur++;
	}
}

void ext4_set_bits(void *bm, int cur, int len)
{
	__u32 *addr;

	len = cur + len;
	while (cur < len) {
		if ((cur & 31) == 0 && (len - cur) >= 32) {
			
			addr = bm + (cur >> 3);
			*addr = 0xffffffff;
			cur += 32;
			continue;
		}
		mb_set_bit(cur, bm);
		cur++;
	}
}

static void mb_free_blocks(struct inode *inode, struct ext4_buddy *e4b,
			  int first, int count)
{
	int block = 0;
	int max = 0;
	int order;
	void *buddy;
	void *buddy2;
	struct super_block *sb = e4b->bd_sb;

	BUG_ON(first + count > (sb->s_blocksize << 3));
	assert_spin_locked(ext4_group_lock_ptr(sb, e4b->bd_group));
	mb_check_buddy(e4b);
	mb_free_blocks_double(inode, e4b, first, count);

	e4b->bd_info->bb_free += count;
	if (first < e4b->bd_info->bb_first_free)
		e4b->bd_info->bb_first_free = first;

	
	if (first != 0)
		block = !mb_test_bit(first - 1, e4b->bd_bitmap);
	if (first + count < EXT4_SB(sb)->s_mb_maxs[0])
		max = !mb_test_bit(first + count, e4b->bd_bitmap);
	if (block && max)
		e4b->bd_info->bb_fragments--;
	else if (!block && !max)
		e4b->bd_info->bb_fragments++;

	
	while (count-- > 0) {
		block = first++;
		order = 0;

		if (!mb_test_bit(block, e4b->bd_bitmap)) {
			ext4_fsblk_t blocknr;

			blocknr = ext4_group_first_block_no(sb, e4b->bd_group);
			blocknr += EXT4_C2B(EXT4_SB(sb), block);
			ext4_grp_locked_error(sb, e4b->bd_group,
					      inode ? inode->i_ino : 0,
					      blocknr,
					      "freeing already freed block "
					      "(bit %u)", block);
		}
		mb_clear_bit(block, e4b->bd_bitmap);
		e4b->bd_info->bb_counters[order]++;

		
		buddy = mb_find_buddy(e4b, order, &max);

		do {
			block &= ~1UL;
			if (mb_test_bit(block, buddy) ||
					mb_test_bit(block + 1, buddy))
				break;

			
			buddy2 = mb_find_buddy(e4b, order + 1, &max);

			if (!buddy2)
				break;

			if (order > 0) {
				mb_set_bit(block, buddy);
				mb_set_bit(block + 1, buddy);
			}
			e4b->bd_info->bb_counters[order]--;
			e4b->bd_info->bb_counters[order]--;

			block = block >> 1;
			order++;
			e4b->bd_info->bb_counters[order]++;

			mb_clear_bit(block, buddy2);
			buddy = buddy2;
		} while (1);
	}
	mb_set_largest_free_order(sb, e4b->bd_info);
	mb_check_buddy(e4b);
}

static int mb_find_extent(struct ext4_buddy *e4b, int order, int block,
				int needed, struct ext4_free_extent *ex)
{
	int next = block;
	int max;
	void *buddy;

	assert_spin_locked(ext4_group_lock_ptr(e4b->bd_sb, e4b->bd_group));
	BUG_ON(ex == NULL);

	buddy = mb_find_buddy(e4b, order, &max);
	BUG_ON(buddy == NULL);
	BUG_ON(block >= max);
	if (mb_test_bit(block, buddy)) {
		ex->fe_len = 0;
		ex->fe_start = 0;
		ex->fe_group = 0;
		return 0;
	}

	
	if (likely(order == 0)) {
		
		order = mb_find_order_for_block(e4b, block);
		block = block >> order;
	}

	ex->fe_len = 1 << order;
	ex->fe_start = block << order;
	ex->fe_group = e4b->bd_group;

	
	next = next - ex->fe_start;
	ex->fe_len -= next;
	ex->fe_start += next;

	while (needed > ex->fe_len &&
	       (buddy = mb_find_buddy(e4b, order, &max))) {

		if (block + 1 >= max)
			break;

		next = (block + 1) * (1 << order);
		if (mb_test_bit(next, e4b->bd_bitmap))
			break;

		order = mb_find_order_for_block(e4b, next);

		block = next >> order;
		ex->fe_len += 1 << order;
	}

	BUG_ON(ex->fe_start + ex->fe_len > (1 << (e4b->bd_blkbits + 3)));
	return ex->fe_len;
}

static int mb_mark_used(struct ext4_buddy *e4b, struct ext4_free_extent *ex)
{
	int ord;
	int mlen = 0;
	int max = 0;
	int cur;
	int start = ex->fe_start;
	int len = ex->fe_len;
	unsigned ret = 0;
	int len0 = len;
	void *buddy;

	BUG_ON(start + len > (e4b->bd_sb->s_blocksize << 3));
	BUG_ON(e4b->bd_group != ex->fe_group);
	assert_spin_locked(ext4_group_lock_ptr(e4b->bd_sb, e4b->bd_group));
	mb_check_buddy(e4b);
	mb_mark_used_double(e4b, start, len);

	e4b->bd_info->bb_free -= len;
	if (e4b->bd_info->bb_first_free == start)
		e4b->bd_info->bb_first_free += len;

	
	if (start != 0)
		mlen = !mb_test_bit(start - 1, e4b->bd_bitmap);
	if (start + len < EXT4_SB(e4b->bd_sb)->s_mb_maxs[0])
		max = !mb_test_bit(start + len, e4b->bd_bitmap);
	if (mlen && max)
		e4b->bd_info->bb_fragments++;
	else if (!mlen && !max)
		e4b->bd_info->bb_fragments--;

	
	while (len) {
		ord = mb_find_order_for_block(e4b, start);

		if (((start >> ord) << ord) == start && len >= (1 << ord)) {
			
			mlen = 1 << ord;
			buddy = mb_find_buddy(e4b, ord, &max);
			BUG_ON((start >> ord) >= max);
			mb_set_bit(start >> ord, buddy);
			e4b->bd_info->bb_counters[ord]--;
			start += mlen;
			len -= mlen;
			BUG_ON(len < 0);
			continue;
		}

		
		if (ret == 0)
			ret = len | (ord << 16);

		
		BUG_ON(ord <= 0);
		buddy = mb_find_buddy(e4b, ord, &max);
		mb_set_bit(start >> ord, buddy);
		e4b->bd_info->bb_counters[ord]--;

		ord--;
		cur = (start >> ord) & ~1U;
		buddy = mb_find_buddy(e4b, ord, &max);
		mb_clear_bit(cur, buddy);
		mb_clear_bit(cur + 1, buddy);
		e4b->bd_info->bb_counters[ord]++;
		e4b->bd_info->bb_counters[ord]++;
	}
	mb_set_largest_free_order(e4b->bd_sb, e4b->bd_info);

	ext4_set_bits(e4b->bd_bitmap, ex->fe_start, len0);
	mb_check_buddy(e4b);

	return ret;
}

static void ext4_mb_use_best_found(struct ext4_allocation_context *ac,
					struct ext4_buddy *e4b)
{
	struct ext4_sb_info *sbi = EXT4_SB(ac->ac_sb);
	int ret;

	BUG_ON(ac->ac_b_ex.fe_group != e4b->bd_group);
	BUG_ON(ac->ac_status == AC_STATUS_FOUND);

	ac->ac_b_ex.fe_len = min(ac->ac_b_ex.fe_len, ac->ac_g_ex.fe_len);
	ac->ac_b_ex.fe_logical = ac->ac_g_ex.fe_logical;
	ret = mb_mark_used(e4b, &ac->ac_b_ex);

	ac->ac_f_ex = ac->ac_b_ex;

	ac->ac_status = AC_STATUS_FOUND;
	ac->ac_tail = ret & 0xffff;
	ac->ac_buddy = ret >> 16;

	ac->ac_bitmap_page = e4b->bd_bitmap_page;
	get_page(ac->ac_bitmap_page);
	ac->ac_buddy_page = e4b->bd_buddy_page;
	get_page(ac->ac_buddy_page);
	
	if (ac->ac_flags & EXT4_MB_STREAM_ALLOC) {
		spin_lock(&sbi->s_md_lock);
		sbi->s_mb_last_group = ac->ac_f_ex.fe_group;
		sbi->s_mb_last_start = ac->ac_f_ex.fe_start;
		spin_unlock(&sbi->s_md_lock);
	}
}


static void ext4_mb_check_limits(struct ext4_allocation_context *ac,
					struct ext4_buddy *e4b,
					int finish_group)
{
	struct ext4_sb_info *sbi = EXT4_SB(ac->ac_sb);
	struct ext4_free_extent *bex = &ac->ac_b_ex;
	struct ext4_free_extent *gex = &ac->ac_g_ex;
	struct ext4_free_extent ex;
	int max;

	if (ac->ac_status == AC_STATUS_FOUND)
		return;
	if (ac->ac_found > sbi->s_mb_max_to_scan &&
			!(ac->ac_flags & EXT4_MB_HINT_FIRST)) {
		ac->ac_status = AC_STATUS_BREAK;
		return;
	}

	if (bex->fe_len < gex->fe_len)
		return;

	if ((finish_group || ac->ac_found > sbi->s_mb_min_to_scan)
			&& bex->fe_group == e4b->bd_group) {
		max = mb_find_extent(e4b, 0, bex->fe_start, gex->fe_len, &ex);
		if (max >= gex->fe_len) {
			ext4_mb_use_best_found(ac, e4b);
			return;
		}
	}
}

static void ext4_mb_measure_extent(struct ext4_allocation_context *ac,
					struct ext4_free_extent *ex,
					struct ext4_buddy *e4b)
{
	struct ext4_free_extent *bex = &ac->ac_b_ex;
	struct ext4_free_extent *gex = &ac->ac_g_ex;

	BUG_ON(ex->fe_len <= 0);
	BUG_ON(ex->fe_len > EXT4_CLUSTERS_PER_GROUP(ac->ac_sb));
	BUG_ON(ex->fe_start >= EXT4_CLUSTERS_PER_GROUP(ac->ac_sb));
	BUG_ON(ac->ac_status != AC_STATUS_CONTINUE);

	ac->ac_found++;

	if (unlikely(ac->ac_flags & EXT4_MB_HINT_FIRST)) {
		*bex = *ex;
		ext4_mb_use_best_found(ac, e4b);
		return;
	}

	if (ex->fe_len == gex->fe_len) {
		*bex = *ex;
		ext4_mb_use_best_found(ac, e4b);
		return;
	}

	if (bex->fe_len == 0) {
		*bex = *ex;
		return;
	}

	if (bex->fe_len < gex->fe_len) {
		if (ex->fe_len > bex->fe_len)
			*bex = *ex;
	} else if (ex->fe_len > gex->fe_len) {
		if (ex->fe_len < bex->fe_len)
			*bex = *ex;
	}

	ext4_mb_check_limits(ac, e4b, 0);
}

static noinline_for_stack
int ext4_mb_try_best_found(struct ext4_allocation_context *ac,
					struct ext4_buddy *e4b)
{
	struct ext4_free_extent ex = ac->ac_b_ex;
	ext4_group_t group = ex.fe_group;
	int max;
	int err;

	BUG_ON(ex.fe_len <= 0);
	err = ext4_mb_load_buddy(ac->ac_sb, group, e4b);
	if (err)
		return err;

	ext4_lock_group(ac->ac_sb, group);
	max = mb_find_extent(e4b, 0, ex.fe_start, ex.fe_len, &ex);

	if (max > 0) {
		ac->ac_b_ex = ex;
		ext4_mb_use_best_found(ac, e4b);
	}

	ext4_unlock_group(ac->ac_sb, group);
	ext4_mb_unload_buddy(e4b);

	return 0;
}

static noinline_for_stack
int ext4_mb_find_by_goal(struct ext4_allocation_context *ac,
				struct ext4_buddy *e4b)
{
	ext4_group_t group = ac->ac_g_ex.fe_group;
	int max;
	int err;
	struct ext4_sb_info *sbi = EXT4_SB(ac->ac_sb);
	struct ext4_free_extent ex;

	if (!(ac->ac_flags & EXT4_MB_HINT_TRY_GOAL))
		return 0;

	err = ext4_mb_load_buddy(ac->ac_sb, group, e4b);
	if (err)
		return err;

	ext4_lock_group(ac->ac_sb, group);
	max = mb_find_extent(e4b, 0, ac->ac_g_ex.fe_start,
			     ac->ac_g_ex.fe_len, &ex);

	if (max >= ac->ac_g_ex.fe_len && ac->ac_g_ex.fe_len == sbi->s_stripe) {
		ext4_fsblk_t start;

		start = ext4_group_first_block_no(ac->ac_sb, e4b->bd_group) +
			ex.fe_start;
		
		if (do_div(start, sbi->s_stripe) == 0) {
			ac->ac_found++;
			ac->ac_b_ex = ex;
			ext4_mb_use_best_found(ac, e4b);
		}
	} else if (max >= ac->ac_g_ex.fe_len) {
		BUG_ON(ex.fe_len <= 0);
		BUG_ON(ex.fe_group != ac->ac_g_ex.fe_group);
		BUG_ON(ex.fe_start != ac->ac_g_ex.fe_start);
		ac->ac_found++;
		ac->ac_b_ex = ex;
		ext4_mb_use_best_found(ac, e4b);
	} else if (max > 0 && (ac->ac_flags & EXT4_MB_HINT_MERGE)) {
		BUG_ON(ex.fe_len <= 0);
		BUG_ON(ex.fe_group != ac->ac_g_ex.fe_group);
		BUG_ON(ex.fe_start != ac->ac_g_ex.fe_start);
		ac->ac_found++;
		ac->ac_b_ex = ex;
		ext4_mb_use_best_found(ac, e4b);
	}
	ext4_unlock_group(ac->ac_sb, group);
	ext4_mb_unload_buddy(e4b);

	return 0;
}

static noinline_for_stack
void ext4_mb_simple_scan_group(struct ext4_allocation_context *ac,
					struct ext4_buddy *e4b)
{
	struct super_block *sb = ac->ac_sb;
	struct ext4_group_info *grp = e4b->bd_info;
	void *buddy;
	int i;
	int k;
	int max;

	BUG_ON(ac->ac_2order <= 0);
	for (i = ac->ac_2order; i <= sb->s_blocksize_bits + 1; i++) {
		if (grp->bb_counters[i] == 0)
			continue;

		buddy = mb_find_buddy(e4b, i, &max);
		BUG_ON(buddy == NULL);

		k = mb_find_next_zero_bit(buddy, max, 0);
		BUG_ON(k >= max);

		ac->ac_found++;

		ac->ac_b_ex.fe_len = 1 << i;
		ac->ac_b_ex.fe_start = k << i;
		ac->ac_b_ex.fe_group = e4b->bd_group;

		ext4_mb_use_best_found(ac, e4b);

		BUG_ON(ac->ac_b_ex.fe_len != ac->ac_g_ex.fe_len);

		if (EXT4_SB(sb)->s_mb_stats)
			atomic_inc(&EXT4_SB(sb)->s_bal_2orders);

		break;
	}
}

static noinline_for_stack
void ext4_mb_complex_scan_group(struct ext4_allocation_context *ac,
					struct ext4_buddy *e4b)
{
	struct super_block *sb = ac->ac_sb;
	void *bitmap = e4b->bd_bitmap;
	struct ext4_free_extent ex;
	int i;
	int free;

	free = e4b->bd_info->bb_free;
	BUG_ON(free <= 0);

	i = e4b->bd_info->bb_first_free;

	while (free && ac->ac_status == AC_STATUS_CONTINUE) {
		i = mb_find_next_zero_bit(bitmap,
						EXT4_CLUSTERS_PER_GROUP(sb), i);
		if (i >= EXT4_CLUSTERS_PER_GROUP(sb)) {
			ext4_grp_locked_error(sb, e4b->bd_group, 0, 0,
					"%d free clusters as per "
					"group info. But bitmap says 0",
					free);
			break;
		}

		mb_find_extent(e4b, 0, i, ac->ac_g_ex.fe_len, &ex);
		BUG_ON(ex.fe_len <= 0);
		if (free < ex.fe_len) {
			ext4_grp_locked_error(sb, e4b->bd_group, 0, 0,
					"%d free clusters as per "
					"group info. But got %d blocks",
					free, ex.fe_len);
			break;
		}

		ext4_mb_measure_extent(ac, &ex, e4b);

		i += ex.fe_len;
		free -= ex.fe_len;
	}

	ext4_mb_check_limits(ac, e4b, 1);
}

static noinline_for_stack
void ext4_mb_scan_aligned(struct ext4_allocation_context *ac,
				 struct ext4_buddy *e4b)
{
	struct super_block *sb = ac->ac_sb;
	struct ext4_sb_info *sbi = EXT4_SB(sb);
	void *bitmap = e4b->bd_bitmap;
	struct ext4_free_extent ex;
	ext4_fsblk_t first_group_block;
	ext4_fsblk_t a;
	ext4_grpblk_t i;
	int max;

	BUG_ON(sbi->s_stripe == 0);

	
	first_group_block = ext4_group_first_block_no(sb, e4b->bd_group);

	a = first_group_block + sbi->s_stripe - 1;
	do_div(a, sbi->s_stripe);
	i = (a * sbi->s_stripe) - first_group_block;

	while (i < EXT4_CLUSTERS_PER_GROUP(sb)) {
		if (!mb_test_bit(i, bitmap)) {
			max = mb_find_extent(e4b, 0, i, sbi->s_stripe, &ex);
			if (max >= sbi->s_stripe) {
				ac->ac_found++;
				ac->ac_b_ex = ex;
				ext4_mb_use_best_found(ac, e4b);
				break;
			}
		}
		i += sbi->s_stripe;
	}
}

static int ext4_mb_good_group(struct ext4_allocation_context *ac,
				ext4_group_t group, int cr)
{
	unsigned free, fragments;
	int flex_size = ext4_flex_bg_size(EXT4_SB(ac->ac_sb));
	struct ext4_group_info *grp = ext4_get_group_info(ac->ac_sb, group);

	BUG_ON(cr < 0 || cr >= 4);

	
	if (unlikely(EXT4_MB_GRP_NEED_INIT(grp))) {
		int ret = ext4_mb_init_group(ac->ac_sb, group);
		if (ret)
			return 0;
	}

	free = grp->bb_free;
	fragments = grp->bb_fragments;
	if (free == 0)
		return 0;
	if (fragments == 0)
		return 0;

	switch (cr) {
	case 0:
		BUG_ON(ac->ac_2order == 0);

		if (grp->bb_largest_free_order < ac->ac_2order)
			return 0;

		
		if ((ac->ac_flags & EXT4_MB_HINT_DATA) &&
		    (flex_size >= EXT4_FLEX_SIZE_DIR_ALLOC_SCHEME) &&
		    ((group % flex_size) == 0))
			return 0;

		return 1;
	case 1:
		if ((free / fragments) >= ac->ac_g_ex.fe_len)
			return 1;
		break;
	case 2:
		if (free >= ac->ac_g_ex.fe_len)
			return 1;
		break;
	case 3:
		return 1;
	default:
		BUG();
	}

	return 0;
}

static noinline_for_stack int
ext4_mb_regular_allocator(struct ext4_allocation_context *ac)
{
	ext4_group_t ngroups, group, i;
	int cr;
	int err = 0;
	struct ext4_sb_info *sbi;
	struct super_block *sb;
	struct ext4_buddy e4b;

	sb = ac->ac_sb;
	sbi = EXT4_SB(sb);
	ngroups = ext4_get_groups_count(sb);
	
	if (!(ext4_test_inode_flag(ac->ac_inode, EXT4_INODE_EXTENTS)))
		ngroups = sbi->s_blockfile_groups;

	BUG_ON(ac->ac_status == AC_STATUS_FOUND);

	
	err = ext4_mb_find_by_goal(ac, &e4b);
	if (err || ac->ac_status == AC_STATUS_FOUND)
		goto out;

	if (unlikely(ac->ac_flags & EXT4_MB_HINT_GOAL_ONLY))
		goto out;

	i = fls(ac->ac_g_ex.fe_len);
	ac->ac_2order = 0;
	if (i >= sbi->s_mb_order2_reqs) {
		if ((ac->ac_g_ex.fe_len & (~(1 << (i - 1)))) == 0)
			ac->ac_2order = i - 1;
	}

	
	if (ac->ac_flags & EXT4_MB_STREAM_ALLOC) {
		
		spin_lock(&sbi->s_md_lock);
		ac->ac_g_ex.fe_group = sbi->s_mb_last_group;
		ac->ac_g_ex.fe_start = sbi->s_mb_last_start;
		spin_unlock(&sbi->s_md_lock);
	}

	
	cr = ac->ac_2order ? 0 : 1;
repeat:
	for (; cr < 4 && ac->ac_status == AC_STATUS_CONTINUE; cr++) {
		ac->ac_criteria = cr;
		group = ac->ac_g_ex.fe_group;

		for (i = 0; i < ngroups; group++, i++) {
			if (group == ngroups)
				group = 0;

			
			if (!ext4_mb_good_group(ac, group, cr))
				continue;

			err = ext4_mb_load_buddy(sb, group, &e4b);
			if (err)
				goto out;

			ext4_lock_group(sb, group);

			if (!ext4_mb_good_group(ac, group, cr)) {
				ext4_unlock_group(sb, group);
				ext4_mb_unload_buddy(&e4b);
				continue;
			}

			ac->ac_groups_scanned++;
			if (cr == 0)
				ext4_mb_simple_scan_group(ac, &e4b);
			else if (cr == 1 && sbi->s_stripe &&
					!(ac->ac_g_ex.fe_len % sbi->s_stripe))
				ext4_mb_scan_aligned(ac, &e4b);
			else
				ext4_mb_complex_scan_group(ac, &e4b);

			ext4_unlock_group(sb, group);
			ext4_mb_unload_buddy(&e4b);

			if (ac->ac_status != AC_STATUS_CONTINUE)
				break;
		}
	}

	if (ac->ac_b_ex.fe_len > 0 && ac->ac_status != AC_STATUS_FOUND &&
	    !(ac->ac_flags & EXT4_MB_HINT_FIRST)) {

		ext4_mb_try_best_found(ac, &e4b);
		if (ac->ac_status != AC_STATUS_FOUND) {
			ac->ac_b_ex.fe_group = 0;
			ac->ac_b_ex.fe_start = 0;
			ac->ac_b_ex.fe_len = 0;
			ac->ac_status = AC_STATUS_CONTINUE;
			ac->ac_flags |= EXT4_MB_HINT_FIRST;
			cr = 3;
			atomic_inc(&sbi->s_mb_lost_chunks);
			goto repeat;
		}
	}
out:
	return err;
}

static void *ext4_mb_seq_groups_start(struct seq_file *seq, loff_t *pos)
{
	struct super_block *sb = seq->private;
	ext4_group_t group;

	if (*pos < 0 || *pos >= ext4_get_groups_count(sb))
		return NULL;
	group = *pos + 1;
	return (void *) ((unsigned long) group);
}

static void *ext4_mb_seq_groups_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct super_block *sb = seq->private;
	ext4_group_t group;

	++*pos;
	if (*pos < 0 || *pos >= ext4_get_groups_count(sb))
		return NULL;
	group = *pos + 1;
	return (void *) ((unsigned long) group);
}

static int ext4_mb_seq_groups_show(struct seq_file *seq, void *v)
{
	struct super_block *sb = seq->private;
	ext4_group_t group = (ext4_group_t) ((unsigned long) v);
	int i;
	int err;
	struct ext4_buddy e4b;
	struct sg {
		struct ext4_group_info info;
		ext4_grpblk_t counters[16];
	} sg;

	group--;
	if (group == 0)
		seq_printf(seq, "#%-5s: %-5s %-5s %-5s "
				"[ %-5s %-5s %-5s %-5s %-5s %-5s %-5s "
				  "%-5s %-5s %-5s %-5s %-5s %-5s %-5s ]\n",
			   "group", "free", "frags", "first",
			   "2^0", "2^1", "2^2", "2^3", "2^4", "2^5", "2^6",
			   "2^7", "2^8", "2^9", "2^10", "2^11", "2^12", "2^13");

	i = (sb->s_blocksize_bits + 2) * sizeof(sg.info.bb_counters[0]) +
		sizeof(struct ext4_group_info);
	err = ext4_mb_load_buddy(sb, group, &e4b);
	if (err) {
		seq_printf(seq, "#%-5u: I/O error\n", group);
		return 0;
	}
	ext4_lock_group(sb, group);
	memcpy(&sg, ext4_get_group_info(sb, group), i);
	ext4_unlock_group(sb, group);
	ext4_mb_unload_buddy(&e4b);

	seq_printf(seq, "#%-5u: %-5u %-5u %-5u [", group, sg.info.bb_free,
			sg.info.bb_fragments, sg.info.bb_first_free);
	for (i = 0; i <= 13; i++)
		seq_printf(seq, " %-5u", i <= sb->s_blocksize_bits + 1 ?
				sg.info.bb_counters[i] : 0);
	seq_printf(seq, " ]\n");

	return 0;
}

static void ext4_mb_seq_groups_stop(struct seq_file *seq, void *v)
{
}

static const struct seq_operations ext4_mb_seq_groups_ops = {
	.start  = ext4_mb_seq_groups_start,
	.next   = ext4_mb_seq_groups_next,
	.stop   = ext4_mb_seq_groups_stop,
	.show   = ext4_mb_seq_groups_show,
};

static int ext4_mb_seq_groups_open(struct inode *inode, struct file *file)
{
	struct super_block *sb = PDE(inode)->data;
	int rc;

	rc = seq_open(file, &ext4_mb_seq_groups_ops);
	if (rc == 0) {
		struct seq_file *m = file->private_data;
		m->private = sb;
	}
	return rc;

}

static const struct file_operations ext4_mb_seq_groups_fops = {
	.owner		= THIS_MODULE,
	.open		= ext4_mb_seq_groups_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static struct kmem_cache *get_groupinfo_cache(int blocksize_bits)
{
	int cache_index = blocksize_bits - EXT4_MIN_BLOCK_LOG_SIZE;
	struct kmem_cache *cachep = ext4_groupinfo_caches[cache_index];

	BUG_ON(!cachep);
	return cachep;
}

int ext4_mb_add_groupinfo(struct super_block *sb, ext4_group_t group,
			  struct ext4_group_desc *desc)
{
	int i;
	int metalen = 0;
	struct ext4_sb_info *sbi = EXT4_SB(sb);
	struct ext4_group_info **meta_group_info;
	struct kmem_cache *cachep = get_groupinfo_cache(sb->s_blocksize_bits);

	if (group % EXT4_DESC_PER_BLOCK(sb) == 0) {
		metalen = sizeof(*meta_group_info) <<
			EXT4_DESC_PER_BLOCK_BITS(sb);
		meta_group_info = kmalloc(metalen, GFP_KERNEL);
		if (meta_group_info == NULL) {
			ext4_msg(sb, KERN_ERR, "can't allocate mem "
				 "for a buddy group");
			goto exit_meta_group_info;
		}
		sbi->s_group_info[group >> EXT4_DESC_PER_BLOCK_BITS(sb)] =
			meta_group_info;
	}

	meta_group_info =
		sbi->s_group_info[group >> EXT4_DESC_PER_BLOCK_BITS(sb)];
	i = group & (EXT4_DESC_PER_BLOCK(sb) - 1);

	meta_group_info[i] = kmem_cache_alloc(cachep, GFP_KERNEL);
	if (meta_group_info[i] == NULL) {
		ext4_msg(sb, KERN_ERR, "can't allocate buddy mem");
		goto exit_group_info;
	}
	memset(meta_group_info[i], 0, kmem_cache_size(cachep));
	set_bit(EXT4_GROUP_INFO_NEED_INIT_BIT,
		&(meta_group_info[i]->bb_state));

	if (desc->bg_flags & cpu_to_le16(EXT4_BG_BLOCK_UNINIT)) {
		meta_group_info[i]->bb_free =
			ext4_free_clusters_after_init(sb, group, desc);
	} else {
		meta_group_info[i]->bb_free =
			ext4_free_group_clusters(sb, desc);
	}

	INIT_LIST_HEAD(&meta_group_info[i]->bb_prealloc_list);
	init_rwsem(&meta_group_info[i]->alloc_sem);
	meta_group_info[i]->bb_free_root = RB_ROOT;
	meta_group_info[i]->bb_largest_free_order = -1;  

#ifdef DOUBLE_CHECK
	{
		struct buffer_head *bh;
		meta_group_info[i]->bb_bitmap =
			kmalloc(sb->s_blocksize, GFP_KERNEL);
		BUG_ON(meta_group_info[i]->bb_bitmap == NULL);
		bh = ext4_read_block_bitmap(sb, group);
		BUG_ON(bh == NULL);
		memcpy(meta_group_info[i]->bb_bitmap, bh->b_data,
			sb->s_blocksize);
		put_bh(bh);
	}
#endif

	return 0;

exit_group_info:
	
	if (group % EXT4_DESC_PER_BLOCK(sb) == 0) {
		kfree(sbi->s_group_info[group >> EXT4_DESC_PER_BLOCK_BITS(sb)]);
		sbi->s_group_info[group >> EXT4_DESC_PER_BLOCK_BITS(sb)] = NULL;
	}
exit_meta_group_info:
	return -ENOMEM;
} 

static int ext4_mb_init_backend(struct super_block *sb)
{
	ext4_group_t ngroups = ext4_get_groups_count(sb);
	ext4_group_t i;
	struct ext4_sb_info *sbi = EXT4_SB(sb);
	struct ext4_super_block *es = sbi->s_es;
	int num_meta_group_infos;
	int num_meta_group_infos_max;
	int array_size;
	struct ext4_group_desc *desc;
	struct kmem_cache *cachep;

	
	num_meta_group_infos = (ngroups + EXT4_DESC_PER_BLOCK(sb) -
				1) >> EXT4_DESC_PER_BLOCK_BITS(sb);

	num_meta_group_infos_max = num_meta_group_infos +
				le16_to_cpu(es->s_reserved_gdt_blocks);

	array_size = 1;
	while (array_size < sizeof(*sbi->s_group_info) *
	       num_meta_group_infos_max)
		array_size = array_size << 1;
	sbi->s_group_info = ext4_kvzalloc(array_size, GFP_KERNEL);
	if (sbi->s_group_info == NULL) {
		ext4_msg(sb, KERN_ERR, "can't allocate buddy meta group");
		return -ENOMEM;
	}
	sbi->s_buddy_cache = new_inode(sb);
	if (sbi->s_buddy_cache == NULL) {
		ext4_msg(sb, KERN_ERR, "can't get new inode");
		goto err_freesgi;
	}
	sbi->s_buddy_cache->i_ino = EXT4_BAD_INO;
	EXT4_I(sbi->s_buddy_cache)->i_disksize = 0;
	for (i = 0; i < ngroups; i++) {
		desc = ext4_get_group_desc(sb, i, NULL);
		if (desc == NULL) {
			ext4_msg(sb, KERN_ERR, "can't read descriptor %u", i);
			goto err_freebuddy;
		}
		if (ext4_mb_add_groupinfo(sb, i, desc) != 0)
			goto err_freebuddy;
	}

	return 0;

err_freebuddy:
	cachep = get_groupinfo_cache(sb->s_blocksize_bits);
	while (i-- > 0)
		kmem_cache_free(cachep, ext4_get_group_info(sb, i));
	i = num_meta_group_infos;
	while (i-- > 0)
		kfree(sbi->s_group_info[i]);
	iput(sbi->s_buddy_cache);
err_freesgi:
	ext4_kvfree(sbi->s_group_info);
	return -ENOMEM;
}

static void ext4_groupinfo_destroy_slabs(void)
{
	int i;

	for (i = 0; i < NR_GRPINFO_CACHES; i++) {
		if (ext4_groupinfo_caches[i])
			kmem_cache_destroy(ext4_groupinfo_caches[i]);
		ext4_groupinfo_caches[i] = NULL;
	}
}

static int ext4_groupinfo_create_slab(size_t size)
{
	static DEFINE_MUTEX(ext4_grpinfo_slab_create_mutex);
	int slab_size;
	int blocksize_bits = order_base_2(size);
	int cache_index = blocksize_bits - EXT4_MIN_BLOCK_LOG_SIZE;
	struct kmem_cache *cachep;

	if (cache_index >= NR_GRPINFO_CACHES)
		return -EINVAL;

	if (unlikely(cache_index < 0))
		cache_index = 0;

	mutex_lock(&ext4_grpinfo_slab_create_mutex);
	if (ext4_groupinfo_caches[cache_index]) {
		mutex_unlock(&ext4_grpinfo_slab_create_mutex);
		return 0;	
	}

	slab_size = offsetof(struct ext4_group_info,
				bb_counters[blocksize_bits + 2]);

	cachep = kmem_cache_create(ext4_groupinfo_slab_names[cache_index],
					slab_size, 0, SLAB_RECLAIM_ACCOUNT,
					NULL);

	ext4_groupinfo_caches[cache_index] = cachep;

	mutex_unlock(&ext4_grpinfo_slab_create_mutex);
	if (!cachep) {
		printk(KERN_EMERG
		       "EXT4-fs: no memory for groupinfo slab cache\n");
		return -ENOMEM;
	}

	return 0;
}

int ext4_mb_init(struct super_block *sb, int needs_recovery)
{
	struct ext4_sb_info *sbi = EXT4_SB(sb);
	unsigned i, j;
	unsigned offset;
	unsigned max;
	int ret;

	i = (sb->s_blocksize_bits + 2) * sizeof(*sbi->s_mb_offsets);

	sbi->s_mb_offsets = kmalloc(i, GFP_KERNEL);
	if (sbi->s_mb_offsets == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	i = (sb->s_blocksize_bits + 2) * sizeof(*sbi->s_mb_maxs);
	sbi->s_mb_maxs = kmalloc(i, GFP_KERNEL);
	if (sbi->s_mb_maxs == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	ret = ext4_groupinfo_create_slab(sb->s_blocksize);
	if (ret < 0)
		goto out;

	
	sbi->s_mb_maxs[0] = sb->s_blocksize << 3;
	sbi->s_mb_offsets[0] = 0;

	i = 1;
	offset = 0;
	max = sb->s_blocksize << 2;
	do {
		sbi->s_mb_offsets[i] = offset;
		sbi->s_mb_maxs[i] = max;
		offset += 1 << (sb->s_blocksize_bits - i);
		max = max >> 1;
		i++;
	} while (i <= sb->s_blocksize_bits + 1);

	spin_lock_init(&sbi->s_md_lock);
	spin_lock_init(&sbi->s_bal_lock);

	sbi->s_mb_max_to_scan = MB_DEFAULT_MAX_TO_SCAN;
	sbi->s_mb_min_to_scan = MB_DEFAULT_MIN_TO_SCAN;
	sbi->s_mb_stats = MB_DEFAULT_STATS;
	sbi->s_mb_stream_request = MB_DEFAULT_STREAM_THRESHOLD;
	sbi->s_mb_order2_reqs = MB_DEFAULT_ORDER2_REQS;
	sbi->s_mb_group_prealloc = max(MB_DEFAULT_GROUP_PREALLOC >>
				       sbi->s_cluster_bits, 32);
	if (sbi->s_stripe > 1) {
		sbi->s_mb_group_prealloc = roundup(
			sbi->s_mb_group_prealloc, sbi->s_stripe);
	}

	sbi->s_locality_groups = alloc_percpu(struct ext4_locality_group);
	if (sbi->s_locality_groups == NULL) {
		ret = -ENOMEM;
		goto out_free_groupinfo_slab;
	}
	for_each_possible_cpu(i) {
		struct ext4_locality_group *lg;
		lg = per_cpu_ptr(sbi->s_locality_groups, i);
		mutex_init(&lg->lg_mutex);
		for (j = 0; j < PREALLOC_TB_SIZE; j++)
			INIT_LIST_HEAD(&lg->lg_prealloc_list[j]);
		spin_lock_init(&lg->lg_prealloc_lock);
	}

	
	ret = ext4_mb_init_backend(sb);
	if (ret != 0)
		goto out_free_locality_groups;

	if (sbi->s_proc)
		proc_create_data("mb_groups", S_IRUGO, sbi->s_proc,
				 &ext4_mb_seq_groups_fops, sb);

	return 0;

out_free_locality_groups:
	free_percpu(sbi->s_locality_groups);
	sbi->s_locality_groups = NULL;
out_free_groupinfo_slab:
	ext4_groupinfo_destroy_slabs();
out:
	kfree(sbi->s_mb_offsets);
	sbi->s_mb_offsets = NULL;
	kfree(sbi->s_mb_maxs);
	sbi->s_mb_maxs = NULL;
	return ret;
}

static void ext4_mb_cleanup_pa(struct ext4_group_info *grp)
{
	struct ext4_prealloc_space *pa;
	struct list_head *cur, *tmp;
	int count = 0;

	list_for_each_safe(cur, tmp, &grp->bb_prealloc_list) {
		pa = list_entry(cur, struct ext4_prealloc_space, pa_group_list);
		list_del(&pa->pa_group_list);
		count++;
		kmem_cache_free(ext4_pspace_cachep, pa);
	}
	if (count)
		mb_debug(1, "mballoc: %u PAs left\n", count);

}

int ext4_mb_release(struct super_block *sb)
{
	ext4_group_t ngroups = ext4_get_groups_count(sb);
	ext4_group_t i;
	int num_meta_group_infos;
	struct ext4_group_info *grinfo;
	struct ext4_sb_info *sbi = EXT4_SB(sb);
	struct kmem_cache *cachep = get_groupinfo_cache(sb->s_blocksize_bits);

	if (sbi->s_group_info) {
		for (i = 0; i < ngroups; i++) {
			grinfo = ext4_get_group_info(sb, i);
#ifdef DOUBLE_CHECK
			kfree(grinfo->bb_bitmap);
#endif
			ext4_lock_group(sb, i);
			ext4_mb_cleanup_pa(grinfo);
			ext4_unlock_group(sb, i);
			kmem_cache_free(cachep, grinfo);
		}
		num_meta_group_infos = (ngroups +
				EXT4_DESC_PER_BLOCK(sb) - 1) >>
			EXT4_DESC_PER_BLOCK_BITS(sb);
		for (i = 0; i < num_meta_group_infos; i++)
			kfree(sbi->s_group_info[i]);
		ext4_kvfree(sbi->s_group_info);
	}
	kfree(sbi->s_mb_offsets);
	kfree(sbi->s_mb_maxs);
	if (sbi->s_buddy_cache)
		iput(sbi->s_buddy_cache);
	if (sbi->s_mb_stats) {
		ext4_msg(sb, KERN_INFO,
		       "mballoc: %u blocks %u reqs (%u success)",
				atomic_read(&sbi->s_bal_allocated),
				atomic_read(&sbi->s_bal_reqs),
				atomic_read(&sbi->s_bal_success));
		ext4_msg(sb, KERN_INFO,
		      "mballoc: %u extents scanned, %u goal hits, "
				"%u 2^N hits, %u breaks, %u lost",
				atomic_read(&sbi->s_bal_ex_scanned),
				atomic_read(&sbi->s_bal_goals),
				atomic_read(&sbi->s_bal_2orders),
				atomic_read(&sbi->s_bal_breaks),
				atomic_read(&sbi->s_mb_lost_chunks));
		ext4_msg(sb, KERN_INFO,
		       "mballoc: %lu generated and it took %Lu",
				sbi->s_mb_buddies_generated,
				sbi->s_mb_generation_time);
		ext4_msg(sb, KERN_INFO,
		       "mballoc: %u preallocated, %u discarded",
				atomic_read(&sbi->s_mb_preallocated),
				atomic_read(&sbi->s_mb_discarded));
	}

	free_percpu(sbi->s_locality_groups);
	if (sbi->s_proc)
		remove_proc_entry("mb_groups", sbi->s_proc);

	return 0;
}

static inline int ext4_issue_discard(struct super_block *sb,
		ext4_group_t block_group, ext4_grpblk_t cluster, int count)
{
	ext4_fsblk_t discard_block;

	discard_block = (EXT4_C2B(EXT4_SB(sb), cluster) +
			 ext4_group_first_block_no(sb, block_group));
	count = EXT4_C2B(EXT4_SB(sb), count);
	trace_ext4_discard_blocks(sb,
			(unsigned long long) discard_block, count);
	return sb_issue_discard(sb, discard_block, count, GFP_NOFS, 0);
}

static void ext4_free_data_callback(struct super_block *sb,
				    struct ext4_journal_cb_entry *jce,
				    int rc)
{
	struct ext4_free_data *entry = (struct ext4_free_data *)jce;
	struct ext4_buddy e4b;
	struct ext4_group_info *db;
	int err, count = 0, count2 = 0;
	journal_t *journal = EXT4_SB(sb)->s_journal;

	mb_debug(1, "gonna free %u blocks in group %u (0x%p):",
		 entry->efd_count, entry->efd_group, entry);

	if (test_opt(sb, DISCARD) && (atomic_read(&journal->j_log_wait) == 0))
		ext4_issue_discard(sb, entry->efd_group,
				   entry->efd_start_cluster, entry->efd_count);

	err = ext4_mb_load_buddy(sb, entry->efd_group, &e4b);
	
	BUG_ON(err != 0);


	db = e4b.bd_info;
	
	count += entry->efd_count;
	count2++;
	ext4_lock_group(sb, entry->efd_group);
	
	rb_erase(&entry->efd_node, &(db->bb_free_root));
	mb_free_blocks(NULL, &e4b, entry->efd_start_cluster, entry->efd_count);

	if (!test_opt(sb, DISCARD))
		EXT4_MB_GRP_CLEAR_TRIMMED(db);

	if (!db->bb_free_root.rb_node) {
		page_cache_release(e4b.bd_buddy_page);
		page_cache_release(e4b.bd_bitmap_page);
	}
	ext4_unlock_group(sb, entry->efd_group);
	kmem_cache_free(ext4_free_data_cachep, entry);
	ext4_mb_unload_buddy(&e4b);

	mb_debug(1, "freed %u blocks in %u structures\n", count, count2);
}

#ifdef CONFIG_EXT4_DEBUG
u8 mb_enable_debug __read_mostly;

static struct dentry *debugfs_dir;
static struct dentry *debugfs_debug;

static void __init ext4_create_debugfs_entry(void)
{
	debugfs_dir = debugfs_create_dir("ext4", NULL);
	if (debugfs_dir)
		debugfs_debug = debugfs_create_u8("mballoc-debug",
						  S_IRUGO | S_IWUSR,
						  debugfs_dir,
						  &mb_enable_debug);
}

static void ext4_remove_debugfs_entry(void)
{
	debugfs_remove(debugfs_debug);
	debugfs_remove(debugfs_dir);
}

#else

static void __init ext4_create_debugfs_entry(void)
{
}

static void ext4_remove_debugfs_entry(void)
{
}

#endif

int __init ext4_init_mballoc(void)
{
	ext4_pspace_cachep = KMEM_CACHE(ext4_prealloc_space,
					SLAB_RECLAIM_ACCOUNT);
	if (ext4_pspace_cachep == NULL)
		return -ENOMEM;

	ext4_ac_cachep = KMEM_CACHE(ext4_allocation_context,
				    SLAB_RECLAIM_ACCOUNT);
	if (ext4_ac_cachep == NULL) {
		kmem_cache_destroy(ext4_pspace_cachep);
		return -ENOMEM;
	}

	ext4_free_data_cachep = KMEM_CACHE(ext4_free_data,
					   SLAB_RECLAIM_ACCOUNT);
	if (ext4_free_data_cachep == NULL) {
		kmem_cache_destroy(ext4_pspace_cachep);
		kmem_cache_destroy(ext4_ac_cachep);
		return -ENOMEM;
	}
	ext4_create_debugfs_entry();
	return 0;
}

void ext4_exit_mballoc(void)
{
	rcu_barrier();
	kmem_cache_destroy(ext4_pspace_cachep);
	kmem_cache_destroy(ext4_ac_cachep);
	kmem_cache_destroy(ext4_free_data_cachep);
	ext4_groupinfo_destroy_slabs();
	ext4_remove_debugfs_entry();
}


static noinline_for_stack int
ext4_mb_mark_diskspace_used(struct ext4_allocation_context *ac,
				handle_t *handle, unsigned int reserv_clstrs)
{
	struct buffer_head *bitmap_bh = NULL;
	struct ext4_group_desc *gdp;
	struct buffer_head *gdp_bh;
	struct ext4_sb_info *sbi;
	struct super_block *sb;
	ext4_fsblk_t block;
	int err, len;

	BUG_ON(ac->ac_status != AC_STATUS_FOUND);
	BUG_ON(ac->ac_b_ex.fe_len <= 0);

	sb = ac->ac_sb;
	sbi = EXT4_SB(sb);

	err = -EIO;
	bitmap_bh = ext4_read_block_bitmap(sb, ac->ac_b_ex.fe_group);
	if (!bitmap_bh)
		goto out_err;

	err = ext4_journal_get_write_access(handle, bitmap_bh);
	if (err)
		goto out_err;

	err = -EIO;
	gdp = ext4_get_group_desc(sb, ac->ac_b_ex.fe_group, &gdp_bh);
	if (!gdp)
		goto out_err;

	ext4_debug("using block group %u(%d)\n", ac->ac_b_ex.fe_group,
			ext4_free_group_clusters(sb, gdp));

	err = ext4_journal_get_write_access(handle, gdp_bh);
	if (err)
		goto out_err;

	block = ext4_grp_offs_to_block(sb, &ac->ac_b_ex);

	len = EXT4_C2B(sbi, ac->ac_b_ex.fe_len);
	if (!ext4_data_block_valid(sbi, block, len)) {
		ext4_error(sb, "Allocating blocks %llu-%llu which overlap "
			   "fs metadata", block, block+len);
		ext4_lock_group(sb, ac->ac_b_ex.fe_group);
		ext4_set_bits(bitmap_bh->b_data, ac->ac_b_ex.fe_start,
			      ac->ac_b_ex.fe_len);
		ext4_unlock_group(sb, ac->ac_b_ex.fe_group);
		err = ext4_handle_dirty_metadata(handle, NULL, bitmap_bh);
		if (!err)
			err = -EAGAIN;
		goto out_err;
	}

	ext4_lock_group(sb, ac->ac_b_ex.fe_group);
#ifdef AGGRESSIVE_CHECK
	{
		int i;
		for (i = 0; i < ac->ac_b_ex.fe_len; i++) {
			BUG_ON(mb_test_bit(ac->ac_b_ex.fe_start + i,
						bitmap_bh->b_data));
		}
	}
#endif
	ext4_set_bits(bitmap_bh->b_data, ac->ac_b_ex.fe_start,
		      ac->ac_b_ex.fe_len);
	if (gdp->bg_flags & cpu_to_le16(EXT4_BG_BLOCK_UNINIT)) {
		gdp->bg_flags &= cpu_to_le16(~EXT4_BG_BLOCK_UNINIT);
		ext4_free_group_clusters_set(sb, gdp,
					     ext4_free_clusters_after_init(sb,
						ac->ac_b_ex.fe_group, gdp));
	}
	len = ext4_free_group_clusters(sb, gdp) - ac->ac_b_ex.fe_len;
	ext4_free_group_clusters_set(sb, gdp, len);
	gdp->bg_checksum = ext4_group_desc_csum(sbi, ac->ac_b_ex.fe_group, gdp);

	ext4_unlock_group(sb, ac->ac_b_ex.fe_group);
	percpu_counter_sub(&sbi->s_freeclusters_counter, ac->ac_b_ex.fe_len);
	if (!(ac->ac_flags & EXT4_MB_DELALLOC_RESERVED))
		
		percpu_counter_sub(&sbi->s_dirtyclusters_counter,
				   reserv_clstrs);

	if (sbi->s_log_groups_per_flex) {
		ext4_group_t flex_group = ext4_flex_group(sbi,
							  ac->ac_b_ex.fe_group);
		atomic_sub(ac->ac_b_ex.fe_len,
			   &sbi->s_flex_groups[flex_group].free_clusters);
	}

	err = ext4_handle_dirty_metadata(handle, NULL, bitmap_bh);
	if (err)
		goto out_err;
	err = ext4_handle_dirty_metadata(handle, NULL, gdp_bh);

out_err:
	ext4_mark_super_dirty(sb);
	brelse(bitmap_bh);
	return err;
}

static void ext4_mb_normalize_group_request(struct ext4_allocation_context *ac)
{
	struct super_block *sb = ac->ac_sb;
	struct ext4_locality_group *lg = ac->ac_lg;

	BUG_ON(lg == NULL);
	ac->ac_g_ex.fe_len = EXT4_SB(sb)->s_mb_group_prealloc;
	mb_debug(1, "#%u: goal %u blocks for locality group\n",
		current->pid, ac->ac_g_ex.fe_len);
}

static noinline_for_stack void
ext4_mb_normalize_request(struct ext4_allocation_context *ac,
				struct ext4_allocation_request *ar)
{
	struct ext4_sb_info *sbi = EXT4_SB(ac->ac_sb);
	int bsbits, max;
	ext4_lblk_t end;
	loff_t size, start_off;
	loff_t orig_size __maybe_unused;
	ext4_lblk_t start;
	struct ext4_inode_info *ei = EXT4_I(ac->ac_inode);
	struct ext4_prealloc_space *pa;

	if (!(ac->ac_flags & EXT4_MB_HINT_DATA))
		return;

	
	if (unlikely(ac->ac_flags & EXT4_MB_HINT_GOAL_ONLY))
		return;

	if (ac->ac_flags & EXT4_MB_HINT_NOPREALLOC)
		return;

	if (ac->ac_flags & EXT4_MB_HINT_GROUP_ALLOC) {
		ext4_mb_normalize_group_request(ac);
		return ;
	}

	bsbits = ac->ac_sb->s_blocksize_bits;

	size = ac->ac_o_ex.fe_logical + EXT4_C2B(sbi, ac->ac_o_ex.fe_len);
	size = size << bsbits;
	if (size < i_size_read(ac->ac_inode))
		size = i_size_read(ac->ac_inode);
	orig_size = size;

	
	max = 2 << bsbits;

#define NRL_CHECK_SIZE(req, size, max, chunk_size)	\
		(req <= (size) || max <= (chunk_size))

	
	
	start_off = 0;
	if (size <= 16 * 1024) {
		size = 16 * 1024;
	} else if (size <= 32 * 1024) {
		size = 32 * 1024;
	} else if (size <= 64 * 1024) {
		size = 64 * 1024;
	} else if (size <= 128 * 1024) {
		size = 128 * 1024;
	} else if (size <= 256 * 1024) {
		size = 256 * 1024;
	} else if (size <= 512 * 1024) {
		size = 512 * 1024;
	} else if (size <= 1024 * 1024) {
		size = 1024 * 1024;
	} else if (NRL_CHECK_SIZE(size, 4 * 1024 * 1024, max, 2 * 1024)) {
		start_off = ((loff_t)ac->ac_o_ex.fe_logical >>
						(21 - bsbits)) << 21;
		size = 2 * 1024 * 1024;
	} else if (NRL_CHECK_SIZE(size, 8 * 1024 * 1024, max, 4 * 1024)) {
		start_off = ((loff_t)ac->ac_o_ex.fe_logical >>
							(22 - bsbits)) << 22;
		size = 4 * 1024 * 1024;
	} else if (NRL_CHECK_SIZE(ac->ac_o_ex.fe_len,
					(8<<20)>>bsbits, max, 8 * 1024)) {
		start_off = ((loff_t)ac->ac_o_ex.fe_logical >>
							(23 - bsbits)) << 23;
		size = 8 * 1024 * 1024;
	} else {
		start_off = (loff_t)ac->ac_o_ex.fe_logical << bsbits;
		size	  = ac->ac_o_ex.fe_len << bsbits;
	}
	size = size >> bsbits;
	start = start_off >> bsbits;

	
	if (ar->pleft && start <= ar->lleft) {
		size -= ar->lleft + 1 - start;
		start = ar->lleft + 1;
	}
	if (ar->pright && start + size - 1 >= ar->lright)
		size -= start + size - ar->lright;

	end = start + size;

	
	rcu_read_lock();
	list_for_each_entry_rcu(pa, &ei->i_prealloc_list, pa_inode_list) {
		ext4_lblk_t pa_end;

		if (pa->pa_deleted)
			continue;
		spin_lock(&pa->pa_lock);
		if (pa->pa_deleted) {
			spin_unlock(&pa->pa_lock);
			continue;
		}

		pa_end = pa->pa_lstart + EXT4_C2B(EXT4_SB(ac->ac_sb),
						  pa->pa_len);

		
		BUG_ON(!(ac->ac_o_ex.fe_logical >= pa_end ||
			ac->ac_o_ex.fe_logical < pa->pa_lstart));

		
		if (pa->pa_lstart >= end || pa_end <= start) {
			spin_unlock(&pa->pa_lock);
			continue;
		}
		BUG_ON(pa->pa_lstart <= start && pa_end >= end);

		
		if (pa_end <= ac->ac_o_ex.fe_logical) {
			BUG_ON(pa_end < start);
			start = pa_end;
		} else if (pa->pa_lstart > ac->ac_o_ex.fe_logical) {
			BUG_ON(pa->pa_lstart > end);
			end = pa->pa_lstart;
		}
		spin_unlock(&pa->pa_lock);
	}
	rcu_read_unlock();
	size = end - start;

	
	rcu_read_lock();
	list_for_each_entry_rcu(pa, &ei->i_prealloc_list, pa_inode_list) {
		ext4_lblk_t pa_end;

		spin_lock(&pa->pa_lock);
		if (pa->pa_deleted == 0) {
			pa_end = pa->pa_lstart + EXT4_C2B(EXT4_SB(ac->ac_sb),
							  pa->pa_len);
			BUG_ON(!(start >= pa_end || end <= pa->pa_lstart));
		}
		spin_unlock(&pa->pa_lock);
	}
	rcu_read_unlock();

	if (start + size <= ac->ac_o_ex.fe_logical &&
			start > ac->ac_o_ex.fe_logical) {
		ext4_msg(ac->ac_sb, KERN_ERR,
			 "start %lu, size %lu, fe_logical %lu",
			 (unsigned long) start, (unsigned long) size,
			 (unsigned long) ac->ac_o_ex.fe_logical);
	}
	BUG_ON(start + size <= ac->ac_o_ex.fe_logical &&
			start > ac->ac_o_ex.fe_logical);
	BUG_ON(size <= 0 || size > EXT4_CLUSTERS_PER_GROUP(ac->ac_sb));

	

	ac->ac_g_ex.fe_logical = start;
	ac->ac_g_ex.fe_len = EXT4_NUM_B2C(sbi, size);

	
	if (ar->pright && (ar->lright == (start + size))) {
		
		ext4_get_group_no_and_offset(ac->ac_sb, ar->pright - size,
						&ac->ac_f_ex.fe_group,
						&ac->ac_f_ex.fe_start);
		ac->ac_flags |= EXT4_MB_HINT_TRY_GOAL;
	}
	if (ar->pleft && (ar->lleft + 1 == start)) {
		
		ext4_get_group_no_and_offset(ac->ac_sb, ar->pleft + 1,
						&ac->ac_f_ex.fe_group,
						&ac->ac_f_ex.fe_start);
		ac->ac_flags |= EXT4_MB_HINT_TRY_GOAL;
	}

	mb_debug(1, "goal: %u(was %u) blocks at %u\n", (unsigned) size,
		(unsigned) orig_size, (unsigned) start);
}

static void ext4_mb_collect_stats(struct ext4_allocation_context *ac)
{
	struct ext4_sb_info *sbi = EXT4_SB(ac->ac_sb);

	if (sbi->s_mb_stats && ac->ac_g_ex.fe_len > 1) {
		atomic_inc(&sbi->s_bal_reqs);
		atomic_add(ac->ac_b_ex.fe_len, &sbi->s_bal_allocated);
		if (ac->ac_b_ex.fe_len >= ac->ac_o_ex.fe_len)
			atomic_inc(&sbi->s_bal_success);
		atomic_add(ac->ac_found, &sbi->s_bal_ex_scanned);
		if (ac->ac_g_ex.fe_start == ac->ac_b_ex.fe_start &&
				ac->ac_g_ex.fe_group == ac->ac_b_ex.fe_group)
			atomic_inc(&sbi->s_bal_goals);
		if (ac->ac_found > sbi->s_mb_max_to_scan)
			atomic_inc(&sbi->s_bal_breaks);
	}

	if (ac->ac_op == EXT4_MB_HISTORY_ALLOC)
		trace_ext4_mballoc_alloc(ac);
	else
		trace_ext4_mballoc_prealloc(ac);
}

static void ext4_discard_allocated_blocks(struct ext4_allocation_context *ac)
{
	struct ext4_prealloc_space *pa = ac->ac_pa;
	int len;

	if (pa && pa->pa_type == MB_INODE_PA) {
		len = ac->ac_b_ex.fe_len;
		pa->pa_free += len;
	}

}

static void ext4_mb_use_inode_pa(struct ext4_allocation_context *ac,
				struct ext4_prealloc_space *pa)
{
	struct ext4_sb_info *sbi = EXT4_SB(ac->ac_sb);
	ext4_fsblk_t start;
	ext4_fsblk_t end;
	int len;

	
	start = pa->pa_pstart + (ac->ac_o_ex.fe_logical - pa->pa_lstart);
	end = min(pa->pa_pstart + EXT4_C2B(sbi, pa->pa_len),
		  start + EXT4_C2B(sbi, ac->ac_o_ex.fe_len));
	len = EXT4_NUM_B2C(sbi, end - start);
	ext4_get_group_no_and_offset(ac->ac_sb, start, &ac->ac_b_ex.fe_group,
					&ac->ac_b_ex.fe_start);
	ac->ac_b_ex.fe_len = len;
	ac->ac_status = AC_STATUS_FOUND;
	ac->ac_pa = pa;

	BUG_ON(start < pa->pa_pstart);
	BUG_ON(end > pa->pa_pstart + EXT4_C2B(sbi, pa->pa_len));
	BUG_ON(pa->pa_free < len);
	pa->pa_free -= len;

	mb_debug(1, "use %llu/%u from inode pa %p\n", start, len, pa);
}

static void ext4_mb_use_group_pa(struct ext4_allocation_context *ac,
				struct ext4_prealloc_space *pa)
{
	unsigned int len = ac->ac_o_ex.fe_len;

	ext4_get_group_no_and_offset(ac->ac_sb, pa->pa_pstart,
					&ac->ac_b_ex.fe_group,
					&ac->ac_b_ex.fe_start);
	ac->ac_b_ex.fe_len = len;
	ac->ac_status = AC_STATUS_FOUND;
	ac->ac_pa = pa;

	mb_debug(1, "use %u/%u from group pa %p\n", pa->pa_lstart-len, len, pa);
}

static struct ext4_prealloc_space *
ext4_mb_check_group_pa(ext4_fsblk_t goal_block,
			struct ext4_prealloc_space *pa,
			struct ext4_prealloc_space *cpa)
{
	ext4_fsblk_t cur_distance, new_distance;

	if (cpa == NULL) {
		atomic_inc(&pa->pa_count);
		return pa;
	}
	cur_distance = abs(goal_block - cpa->pa_pstart);
	new_distance = abs(goal_block - pa->pa_pstart);

	if (cur_distance <= new_distance)
		return cpa;

	
	atomic_dec(&cpa->pa_count);
	atomic_inc(&pa->pa_count);
	return pa;
}

static noinline_for_stack int
ext4_mb_use_preallocated(struct ext4_allocation_context *ac)
{
	struct ext4_sb_info *sbi = EXT4_SB(ac->ac_sb);
	int order, i;
	struct ext4_inode_info *ei = EXT4_I(ac->ac_inode);
	struct ext4_locality_group *lg;
	struct ext4_prealloc_space *pa, *cpa = NULL;
	ext4_fsblk_t goal_block;

	
	if (!(ac->ac_flags & EXT4_MB_HINT_DATA))
		return 0;

	
	rcu_read_lock();
	list_for_each_entry_rcu(pa, &ei->i_prealloc_list, pa_inode_list) {

		if (ac->ac_o_ex.fe_logical < pa->pa_lstart ||
		    ac->ac_o_ex.fe_logical >= (pa->pa_lstart +
					       EXT4_C2B(sbi, pa->pa_len)))
			continue;

		
		if (!(ext4_test_inode_flag(ac->ac_inode, EXT4_INODE_EXTENTS)) &&
		    (pa->pa_pstart + EXT4_C2B(sbi, pa->pa_len) >
		     EXT4_MAX_BLOCK_FILE_PHYS))
			continue;

		
		spin_lock(&pa->pa_lock);
		if (pa->pa_deleted == 0 && pa->pa_free) {
			atomic_inc(&pa->pa_count);
			ext4_mb_use_inode_pa(ac, pa);
			spin_unlock(&pa->pa_lock);
			ac->ac_criteria = 10;
			rcu_read_unlock();
			return 1;
		}
		spin_unlock(&pa->pa_lock);
	}
	rcu_read_unlock();

	
	if (!(ac->ac_flags & EXT4_MB_HINT_GROUP_ALLOC))
		return 0;

	
	lg = ac->ac_lg;
	if (lg == NULL)
		return 0;
	order  = fls(ac->ac_o_ex.fe_len) - 1;
	if (order > PREALLOC_TB_SIZE - 1)
		
		order = PREALLOC_TB_SIZE - 1;

	goal_block = ext4_grp_offs_to_block(ac->ac_sb, &ac->ac_g_ex);
	for (i = order; i < PREALLOC_TB_SIZE; i++) {
		rcu_read_lock();
		list_for_each_entry_rcu(pa, &lg->lg_prealloc_list[i],
					pa_inode_list) {
			spin_lock(&pa->pa_lock);
			if (pa->pa_deleted == 0 &&
					pa->pa_free >= ac->ac_o_ex.fe_len) {

				cpa = ext4_mb_check_group_pa(goal_block,
								pa, cpa);
			}
			spin_unlock(&pa->pa_lock);
		}
		rcu_read_unlock();
	}
	if (cpa) {
		ext4_mb_use_group_pa(ac, cpa);
		ac->ac_criteria = 20;
		return 1;
	}
	return 0;
}

static void ext4_mb_generate_from_freelist(struct super_block *sb, void *bitmap,
						ext4_group_t group)
{
	struct rb_node *n;
	struct ext4_group_info *grp;
	struct ext4_free_data *entry;

	grp = ext4_get_group_info(sb, group);
	n = rb_first(&(grp->bb_free_root));

	while (n) {
		entry = rb_entry(n, struct ext4_free_data, efd_node);
		ext4_set_bits(bitmap, entry->efd_start_cluster, entry->efd_count);
		n = rb_next(n);
	}
	return;
}

static noinline_for_stack
void ext4_mb_generate_from_pa(struct super_block *sb, void *bitmap,
					ext4_group_t group)
{
	struct ext4_group_info *grp = ext4_get_group_info(sb, group);
	struct ext4_prealloc_space *pa;
	struct list_head *cur;
	ext4_group_t groupnr;
	ext4_grpblk_t start;
	int preallocated = 0;
	int len;

	list_for_each(cur, &grp->bb_prealloc_list) {
		pa = list_entry(cur, struct ext4_prealloc_space, pa_group_list);
		spin_lock(&pa->pa_lock);
		ext4_get_group_no_and_offset(sb, pa->pa_pstart,
					     &groupnr, &start);
		len = pa->pa_len;
		spin_unlock(&pa->pa_lock);
		if (unlikely(len == 0))
			continue;
		BUG_ON(groupnr != group);
		ext4_set_bits(bitmap, start, len);
		preallocated += len;
	}
	mb_debug(1, "prellocated %u for group %u\n", preallocated, group);
}

static void ext4_mb_pa_callback(struct rcu_head *head)
{
	struct ext4_prealloc_space *pa;
	pa = container_of(head, struct ext4_prealloc_space, u.pa_rcu);
	kmem_cache_free(ext4_pspace_cachep, pa);
}

static void ext4_mb_put_pa(struct ext4_allocation_context *ac,
			struct super_block *sb, struct ext4_prealloc_space *pa)
{
	ext4_group_t grp;
	ext4_fsblk_t grp_blk;

	if (!atomic_dec_and_test(&pa->pa_count) || pa->pa_free != 0)
		return;

	
	spin_lock(&pa->pa_lock);
	if (pa->pa_deleted == 1) {
		spin_unlock(&pa->pa_lock);
		return;
	}

	pa->pa_deleted = 1;
	spin_unlock(&pa->pa_lock);

	grp_blk = pa->pa_pstart;
	if (pa->pa_type == MB_GROUP_PA)
		grp_blk--;

	ext4_get_group_no_and_offset(sb, grp_blk, &grp, NULL);

	ext4_lock_group(sb, grp);
	list_del(&pa->pa_group_list);
	ext4_unlock_group(sb, grp);

	spin_lock(pa->pa_obj_lock);
	list_del_rcu(&pa->pa_inode_list);
	spin_unlock(pa->pa_obj_lock);

	call_rcu(&(pa)->u.pa_rcu, ext4_mb_pa_callback);
}

static noinline_for_stack int
ext4_mb_new_inode_pa(struct ext4_allocation_context *ac)
{
	struct super_block *sb = ac->ac_sb;
	struct ext4_sb_info *sbi = EXT4_SB(sb);
	struct ext4_prealloc_space *pa;
	struct ext4_group_info *grp;
	struct ext4_inode_info *ei;

	
	BUG_ON(ac->ac_o_ex.fe_len >= ac->ac_b_ex.fe_len);
	BUG_ON(ac->ac_status != AC_STATUS_FOUND);
	BUG_ON(!S_ISREG(ac->ac_inode->i_mode));

	pa = kmem_cache_alloc(ext4_pspace_cachep, GFP_NOFS);
	if (pa == NULL)
		return -ENOMEM;

	if (ac->ac_b_ex.fe_len < ac->ac_g_ex.fe_len) {
		int winl;
		int wins;
		int win;
		int offs;

		BUG_ON(ac->ac_g_ex.fe_logical > ac->ac_o_ex.fe_logical);
		BUG_ON(ac->ac_g_ex.fe_len < ac->ac_o_ex.fe_len);

		winl = ac->ac_o_ex.fe_logical - ac->ac_g_ex.fe_logical;

		
		wins = EXT4_C2B(sbi, ac->ac_b_ex.fe_len - ac->ac_o_ex.fe_len);

		
		win = min(winl, wins);

		offs = ac->ac_o_ex.fe_logical %
			EXT4_C2B(sbi, ac->ac_b_ex.fe_len);
		if (offs && offs < win)
			win = offs;

		ac->ac_b_ex.fe_logical = ac->ac_o_ex.fe_logical -
			EXT4_B2C(sbi, win);
		BUG_ON(ac->ac_o_ex.fe_logical < ac->ac_b_ex.fe_logical);
		BUG_ON(ac->ac_o_ex.fe_len > ac->ac_b_ex.fe_len);
	}

	ac->ac_f_ex = ac->ac_b_ex;

	pa->pa_lstart = ac->ac_b_ex.fe_logical;
	pa->pa_pstart = ext4_grp_offs_to_block(sb, &ac->ac_b_ex);
	pa->pa_len = ac->ac_b_ex.fe_len;
	pa->pa_free = pa->pa_len;
	atomic_set(&pa->pa_count, 1);
	spin_lock_init(&pa->pa_lock);
	INIT_LIST_HEAD(&pa->pa_inode_list);
	INIT_LIST_HEAD(&pa->pa_group_list);
	pa->pa_deleted = 0;
	pa->pa_type = MB_INODE_PA;

	mb_debug(1, "new inode pa %p: %llu/%u for %u\n", pa,
			pa->pa_pstart, pa->pa_len, pa->pa_lstart);
	trace_ext4_mb_new_inode_pa(ac, pa);

	ext4_mb_use_inode_pa(ac, pa);
	atomic_add(pa->pa_free, &sbi->s_mb_preallocated);

	ei = EXT4_I(ac->ac_inode);
	grp = ext4_get_group_info(sb, ac->ac_b_ex.fe_group);

	pa->pa_obj_lock = &ei->i_prealloc_lock;
	pa->pa_inode = ac->ac_inode;

	ext4_lock_group(sb, ac->ac_b_ex.fe_group);
	list_add(&pa->pa_group_list, &grp->bb_prealloc_list);
	ext4_unlock_group(sb, ac->ac_b_ex.fe_group);

	spin_lock(pa->pa_obj_lock);
	list_add_rcu(&pa->pa_inode_list, &ei->i_prealloc_list);
	spin_unlock(pa->pa_obj_lock);

	return 0;
}

static noinline_for_stack int
ext4_mb_new_group_pa(struct ext4_allocation_context *ac)
{
	struct super_block *sb = ac->ac_sb;
	struct ext4_locality_group *lg;
	struct ext4_prealloc_space *pa;
	struct ext4_group_info *grp;

	
	BUG_ON(ac->ac_o_ex.fe_len >= ac->ac_b_ex.fe_len);
	BUG_ON(ac->ac_status != AC_STATUS_FOUND);
	BUG_ON(!S_ISREG(ac->ac_inode->i_mode));

	BUG_ON(ext4_pspace_cachep == NULL);
	pa = kmem_cache_alloc(ext4_pspace_cachep, GFP_NOFS);
	if (pa == NULL)
		return -ENOMEM;

	ac->ac_f_ex = ac->ac_b_ex;

	pa->pa_pstart = ext4_grp_offs_to_block(sb, &ac->ac_b_ex);
	pa->pa_lstart = pa->pa_pstart;
	pa->pa_len = ac->ac_b_ex.fe_len;
	pa->pa_free = pa->pa_len;
	atomic_set(&pa->pa_count, 1);
	spin_lock_init(&pa->pa_lock);
	INIT_LIST_HEAD(&pa->pa_inode_list);
	INIT_LIST_HEAD(&pa->pa_group_list);
	pa->pa_deleted = 0;
	pa->pa_type = MB_GROUP_PA;

	mb_debug(1, "new group pa %p: %llu/%u for %u\n", pa,
			pa->pa_pstart, pa->pa_len, pa->pa_lstart);
	trace_ext4_mb_new_group_pa(ac, pa);

	ext4_mb_use_group_pa(ac, pa);
	atomic_add(pa->pa_free, &EXT4_SB(sb)->s_mb_preallocated);

	grp = ext4_get_group_info(sb, ac->ac_b_ex.fe_group);
	lg = ac->ac_lg;
	BUG_ON(lg == NULL);

	pa->pa_obj_lock = &lg->lg_prealloc_lock;
	pa->pa_inode = NULL;

	ext4_lock_group(sb, ac->ac_b_ex.fe_group);
	list_add(&pa->pa_group_list, &grp->bb_prealloc_list);
	ext4_unlock_group(sb, ac->ac_b_ex.fe_group);

	return 0;
}

static int ext4_mb_new_preallocation(struct ext4_allocation_context *ac)
{
	int err;

	if (ac->ac_flags & EXT4_MB_HINT_GROUP_ALLOC)
		err = ext4_mb_new_group_pa(ac);
	else
		err = ext4_mb_new_inode_pa(ac);
	return err;
}

static noinline_for_stack int
ext4_mb_release_inode_pa(struct ext4_buddy *e4b, struct buffer_head *bitmap_bh,
			struct ext4_prealloc_space *pa)
{
	struct super_block *sb = e4b->bd_sb;
	struct ext4_sb_info *sbi = EXT4_SB(sb);
	unsigned int end;
	unsigned int next;
	ext4_group_t group;
	ext4_grpblk_t bit;
	unsigned long long grp_blk_start;
	int err = 0;
	int free = 0;

	BUG_ON(pa->pa_deleted == 0);
	ext4_get_group_no_and_offset(sb, pa->pa_pstart, &group, &bit);
	grp_blk_start = pa->pa_pstart - EXT4_C2B(sbi, bit);
	BUG_ON(group != e4b->bd_group && pa->pa_len != 0);
	end = bit + pa->pa_len;

	while (bit < end) {
		bit = mb_find_next_zero_bit(bitmap_bh->b_data, end, bit);
		if (bit >= end)
			break;
		next = mb_find_next_bit(bitmap_bh->b_data, end, bit);
		mb_debug(1, "    free preallocated %u/%u in group %u\n",
			 (unsigned) ext4_group_first_block_no(sb, group) + bit,
			 (unsigned) next - bit, (unsigned) group);
		free += next - bit;

		trace_ext4_mballoc_discard(sb, NULL, group, bit, next - bit);
		trace_ext4_mb_release_inode_pa(pa, (grp_blk_start +
						    EXT4_C2B(sbi, bit)),
					       next - bit);
		mb_free_blocks(pa->pa_inode, e4b, bit, next - bit);
		bit = next + 1;
	}
	if (free != pa->pa_free) {
		ext4_msg(e4b->bd_sb, KERN_CRIT,
			 "pa %p: logic %lu, phys. %lu, len %lu",
			 pa, (unsigned long) pa->pa_lstart,
			 (unsigned long) pa->pa_pstart,
			 (unsigned long) pa->pa_len);
		ext4_grp_locked_error(sb, group, 0, 0, "free %u, pa_free %u",
					free, pa->pa_free);
	}
	atomic_add(free, &sbi->s_mb_discarded);

	return err;
}

static noinline_for_stack int
ext4_mb_release_group_pa(struct ext4_buddy *e4b,
				struct ext4_prealloc_space *pa)
{
	struct super_block *sb = e4b->bd_sb;
	ext4_group_t group;
	ext4_grpblk_t bit;

	trace_ext4_mb_release_group_pa(sb, pa);
	BUG_ON(pa->pa_deleted == 0);
	ext4_get_group_no_and_offset(sb, pa->pa_pstart, &group, &bit);
	BUG_ON(group != e4b->bd_group && pa->pa_len != 0);
	mb_free_blocks(pa->pa_inode, e4b, bit, pa->pa_len);
	atomic_add(pa->pa_len, &EXT4_SB(sb)->s_mb_discarded);
	trace_ext4_mballoc_discard(sb, NULL, group, bit, pa->pa_len);

	return 0;
}

static noinline_for_stack int
ext4_mb_discard_group_preallocations(struct super_block *sb,
					ext4_group_t group, int needed)
{
	struct ext4_group_info *grp = ext4_get_group_info(sb, group);
	struct buffer_head *bitmap_bh = NULL;
	struct ext4_prealloc_space *pa, *tmp;
	struct list_head list;
	struct ext4_buddy e4b;
	int err;
	int busy = 0;
	int free = 0;

	mb_debug(1, "discard preallocation for group %u\n", group);

	if (list_empty(&grp->bb_prealloc_list))
		return 0;

	bitmap_bh = ext4_read_block_bitmap(sb, group);
	if (bitmap_bh == NULL) {
		ext4_error(sb, "Error reading block bitmap for %u", group);
		return 0;
	}

	err = ext4_mb_load_buddy(sb, group, &e4b);
	if (err) {
		ext4_error(sb, "Error loading buddy information for %u", group);
		put_bh(bitmap_bh);
		return 0;
	}

	if (needed == 0)
		needed = EXT4_CLUSTERS_PER_GROUP(sb) + 1;

	INIT_LIST_HEAD(&list);
repeat:
	ext4_lock_group(sb, group);
	list_for_each_entry_safe(pa, tmp,
				&grp->bb_prealloc_list, pa_group_list) {
		spin_lock(&pa->pa_lock);
		if (atomic_read(&pa->pa_count)) {
			spin_unlock(&pa->pa_lock);
			busy = 1;
			continue;
		}
		if (pa->pa_deleted) {
			spin_unlock(&pa->pa_lock);
			continue;
		}

		
		pa->pa_deleted = 1;

		
		free += pa->pa_free;

		spin_unlock(&pa->pa_lock);

		list_del(&pa->pa_group_list);
		list_add(&pa->u.pa_tmp_list, &list);
	}

	
	if (free < needed && busy) {
		busy = 0;
		ext4_unlock_group(sb, group);
		yield();
		goto repeat;
	}

	
	if (list_empty(&list)) {
		BUG_ON(free != 0);
		goto out;
	}

	
	list_for_each_entry_safe(pa, tmp, &list, u.pa_tmp_list) {

		
		spin_lock(pa->pa_obj_lock);
		list_del_rcu(&pa->pa_inode_list);
		spin_unlock(pa->pa_obj_lock);

		if (pa->pa_type == MB_GROUP_PA)
			ext4_mb_release_group_pa(&e4b, pa);
		else
			ext4_mb_release_inode_pa(&e4b, bitmap_bh, pa);

		list_del(&pa->u.pa_tmp_list);
		call_rcu(&(pa)->u.pa_rcu, ext4_mb_pa_callback);
	}

out:
	ext4_unlock_group(sb, group);
	ext4_mb_unload_buddy(&e4b);
	put_bh(bitmap_bh);
	return free;
}

void ext4_discard_preallocations(struct inode *inode)
{
	struct ext4_inode_info *ei = EXT4_I(inode);
	struct super_block *sb = inode->i_sb;
	struct buffer_head *bitmap_bh = NULL;
	struct ext4_prealloc_space *pa, *tmp;
	ext4_group_t group = 0;
	struct list_head list;
	struct ext4_buddy e4b;
	int err;

	if (!S_ISREG(inode->i_mode)) {
		
		return;
	}

	mb_debug(1, "discard preallocation for inode %lu\n", inode->i_ino);
	trace_ext4_discard_preallocations(inode);

	INIT_LIST_HEAD(&list);

repeat:
	
	spin_lock(&ei->i_prealloc_lock);
	while (!list_empty(&ei->i_prealloc_list)) {
		pa = list_entry(ei->i_prealloc_list.next,
				struct ext4_prealloc_space, pa_inode_list);
		BUG_ON(pa->pa_obj_lock != &ei->i_prealloc_lock);
		spin_lock(&pa->pa_lock);
		if (atomic_read(&pa->pa_count)) {
			spin_unlock(&pa->pa_lock);
			spin_unlock(&ei->i_prealloc_lock);
			ext4_msg(sb, KERN_ERR,
				 "uh-oh! used pa while discarding");
			WARN_ON(1);
			schedule_timeout_uninterruptible(HZ);
			goto repeat;

		}
		if (pa->pa_deleted == 0) {
			pa->pa_deleted = 1;
			spin_unlock(&pa->pa_lock);
			list_del_rcu(&pa->pa_inode_list);
			list_add(&pa->u.pa_tmp_list, &list);
			continue;
		}

		
		spin_unlock(&pa->pa_lock);
		spin_unlock(&ei->i_prealloc_lock);


		schedule_timeout_uninterruptible(HZ);
		goto repeat;
	}
	spin_unlock(&ei->i_prealloc_lock);

	list_for_each_entry_safe(pa, tmp, &list, u.pa_tmp_list) {
		BUG_ON(pa->pa_type != MB_INODE_PA);
		ext4_get_group_no_and_offset(sb, pa->pa_pstart, &group, NULL);

		err = ext4_mb_load_buddy(sb, group, &e4b);
		if (err) {
			ext4_error(sb, "Error loading buddy information for %u",
					group);
			continue;
		}

		bitmap_bh = ext4_read_block_bitmap(sb, group);
		if (bitmap_bh == NULL) {
			ext4_error(sb, "Error reading block bitmap for %u",
					group);
			ext4_mb_unload_buddy(&e4b);
			continue;
		}

		ext4_lock_group(sb, group);
		list_del(&pa->pa_group_list);
		ext4_mb_release_inode_pa(&e4b, bitmap_bh, pa);
		ext4_unlock_group(sb, group);

		ext4_mb_unload_buddy(&e4b);
		put_bh(bitmap_bh);

		list_del(&pa->u.pa_tmp_list);
		call_rcu(&(pa)->u.pa_rcu, ext4_mb_pa_callback);
	}
}

#ifdef CONFIG_EXT4_DEBUG
static void ext4_mb_show_ac(struct ext4_allocation_context *ac)
{
	struct super_block *sb = ac->ac_sb;
	ext4_group_t ngroups, i;

	if (!mb_enable_debug ||
	    (EXT4_SB(sb)->s_mount_flags & EXT4_MF_FS_ABORTED))
		return;

	ext4_msg(ac->ac_sb, KERN_ERR, "Can't allocate:"
			" Allocation context details:");
	ext4_msg(ac->ac_sb, KERN_ERR, "status %d flags %d",
			ac->ac_status, ac->ac_flags);
	ext4_msg(ac->ac_sb, KERN_ERR, "orig %lu/%lu/%lu@%lu, "
		 	"goal %lu/%lu/%lu@%lu, "
			"best %lu/%lu/%lu@%lu cr %d",
			(unsigned long)ac->ac_o_ex.fe_group,
			(unsigned long)ac->ac_o_ex.fe_start,
			(unsigned long)ac->ac_o_ex.fe_len,
			(unsigned long)ac->ac_o_ex.fe_logical,
			(unsigned long)ac->ac_g_ex.fe_group,
			(unsigned long)ac->ac_g_ex.fe_start,
			(unsigned long)ac->ac_g_ex.fe_len,
			(unsigned long)ac->ac_g_ex.fe_logical,
			(unsigned long)ac->ac_b_ex.fe_group,
			(unsigned long)ac->ac_b_ex.fe_start,
			(unsigned long)ac->ac_b_ex.fe_len,
			(unsigned long)ac->ac_b_ex.fe_logical,
			(int)ac->ac_criteria);
	ext4_msg(ac->ac_sb, KERN_ERR, "%lu scanned, %d found",
		 ac->ac_ex_scanned, ac->ac_found);
	ext4_msg(ac->ac_sb, KERN_ERR, "groups: ");
	ngroups = ext4_get_groups_count(sb);
	for (i = 0; i < ngroups; i++) {
		struct ext4_group_info *grp = ext4_get_group_info(sb, i);
		struct ext4_prealloc_space *pa;
		ext4_grpblk_t start;
		struct list_head *cur;
		ext4_lock_group(sb, i);
		list_for_each(cur, &grp->bb_prealloc_list) {
			pa = list_entry(cur, struct ext4_prealloc_space,
					pa_group_list);
			spin_lock(&pa->pa_lock);
			ext4_get_group_no_and_offset(sb, pa->pa_pstart,
						     NULL, &start);
			spin_unlock(&pa->pa_lock);
			printk(KERN_ERR "PA:%u:%d:%u \n", i,
			       start, pa->pa_len);
		}
		ext4_unlock_group(sb, i);

		if (grp->bb_free == 0)
			continue;
		printk(KERN_ERR "%u: %d/%d \n",
		       i, grp->bb_free, grp->bb_fragments);
	}
	printk(KERN_ERR "\n");
}
#else
static inline void ext4_mb_show_ac(struct ext4_allocation_context *ac)
{
	return;
}
#endif

static void ext4_mb_group_or_file(struct ext4_allocation_context *ac)
{
	struct ext4_sb_info *sbi = EXT4_SB(ac->ac_sb);
	int bsbits = ac->ac_sb->s_blocksize_bits;
	loff_t size, isize;

	if (!(ac->ac_flags & EXT4_MB_HINT_DATA))
		return;

	if (unlikely(ac->ac_flags & EXT4_MB_HINT_GOAL_ONLY))
		return;

	size = ac->ac_o_ex.fe_logical + EXT4_C2B(sbi, ac->ac_o_ex.fe_len);
	isize = (i_size_read(ac->ac_inode) + ac->ac_sb->s_blocksize - 1)
		>> bsbits;

	if ((size == isize) &&
	    !ext4_fs_is_busy(sbi) &&
	    (atomic_read(&ac->ac_inode->i_writecount) == 0)) {
		ac->ac_flags |= EXT4_MB_HINT_NOPREALLOC;
		return;
	}

	if (sbi->s_mb_group_prealloc <= 0) {
		ac->ac_flags |= EXT4_MB_STREAM_ALLOC;
		return;
	}

	
	size = max(size, isize);
	if (size > sbi->s_mb_stream_request) {
		ac->ac_flags |= EXT4_MB_STREAM_ALLOC;
		return;
	}

	BUG_ON(ac->ac_lg != NULL);
	ac->ac_lg = __this_cpu_ptr(sbi->s_locality_groups);

	
	ac->ac_flags |= EXT4_MB_HINT_GROUP_ALLOC;

	
	mutex_lock(&ac->ac_lg->lg_mutex);
}

static noinline_for_stack int
ext4_mb_initialize_context(struct ext4_allocation_context *ac,
				struct ext4_allocation_request *ar)
{
	struct super_block *sb = ar->inode->i_sb;
	struct ext4_sb_info *sbi = EXT4_SB(sb);
	struct ext4_super_block *es = sbi->s_es;
	ext4_group_t group;
	unsigned int len;
	ext4_fsblk_t goal;
	ext4_grpblk_t block;

	
	len = ar->len;

	
	if (len >= EXT4_CLUSTERS_PER_GROUP(sb) - 10)
		len = EXT4_CLUSTERS_PER_GROUP(sb) - 10;

	
	goal = ar->goal;
	if (goal < le32_to_cpu(es->s_first_data_block) ||
			goal >= ext4_blocks_count(es))
		goal = le32_to_cpu(es->s_first_data_block);
	ext4_get_group_no_and_offset(sb, goal, &group, &block);

	
	memset(ac, 0, sizeof(struct ext4_allocation_context));
	ac->ac_b_ex.fe_logical = ar->logical & ~(sbi->s_cluster_ratio - 1);
	ac->ac_status = AC_STATUS_CONTINUE;
	ac->ac_sb = sb;
	ac->ac_inode = ar->inode;
	ac->ac_o_ex.fe_logical = ac->ac_b_ex.fe_logical;
	ac->ac_o_ex.fe_group = group;
	ac->ac_o_ex.fe_start = block;
	ac->ac_o_ex.fe_len = len;
	ac->ac_g_ex = ac->ac_o_ex;
	ac->ac_flags = ar->flags;

	ext4_mb_group_or_file(ac);

	mb_debug(1, "init ac: %u blocks @ %u, goal %u, flags %x, 2^%d, "
			"left: %u/%u, right %u/%u to %swritable\n",
			(unsigned) ar->len, (unsigned) ar->logical,
			(unsigned) ar->goal, ac->ac_flags, ac->ac_2order,
			(unsigned) ar->lleft, (unsigned) ar->pleft,
			(unsigned) ar->lright, (unsigned) ar->pright,
			atomic_read(&ar->inode->i_writecount) ? "" : "non-");
	return 0;

}

static noinline_for_stack void
ext4_mb_discard_lg_preallocations(struct super_block *sb,
					struct ext4_locality_group *lg,
					int order, int total_entries)
{
	ext4_group_t group = 0;
	struct ext4_buddy e4b;
	struct list_head discard_list;
	struct ext4_prealloc_space *pa, *tmp;

	mb_debug(1, "discard locality group preallocation\n");

	INIT_LIST_HEAD(&discard_list);

	spin_lock(&lg->lg_prealloc_lock);
	list_for_each_entry_rcu(pa, &lg->lg_prealloc_list[order],
						pa_inode_list) {
		spin_lock(&pa->pa_lock);
		if (atomic_read(&pa->pa_count)) {
			spin_unlock(&pa->pa_lock);
			continue;
		}
		if (pa->pa_deleted) {
			spin_unlock(&pa->pa_lock);
			continue;
		}
		
		BUG_ON(pa->pa_type != MB_GROUP_PA);

		
		pa->pa_deleted = 1;
		spin_unlock(&pa->pa_lock);

		list_del_rcu(&pa->pa_inode_list);
		list_add(&pa->u.pa_tmp_list, &discard_list);

		total_entries--;
		if (total_entries <= 5) {
			break;
		}
	}
	spin_unlock(&lg->lg_prealloc_lock);

	list_for_each_entry_safe(pa, tmp, &discard_list, u.pa_tmp_list) {

		ext4_get_group_no_and_offset(sb, pa->pa_pstart, &group, NULL);
		if (ext4_mb_load_buddy(sb, group, &e4b)) {
			ext4_error(sb, "Error loading buddy information for %u",
					group);
			continue;
		}
		ext4_lock_group(sb, group);
		list_del(&pa->pa_group_list);
		ext4_mb_release_group_pa(&e4b, pa);
		ext4_unlock_group(sb, group);

		ext4_mb_unload_buddy(&e4b);
		list_del(&pa->u.pa_tmp_list);
		call_rcu(&(pa)->u.pa_rcu, ext4_mb_pa_callback);
	}
}


static void ext4_mb_add_n_trim(struct ext4_allocation_context *ac)
{
	int order, added = 0, lg_prealloc_count = 1;
	struct super_block *sb = ac->ac_sb;
	struct ext4_locality_group *lg = ac->ac_lg;
	struct ext4_prealloc_space *tmp_pa, *pa = ac->ac_pa;

	order = fls(pa->pa_free) - 1;
	if (order > PREALLOC_TB_SIZE - 1)
		
		order = PREALLOC_TB_SIZE - 1;
	
	rcu_read_lock();
	list_for_each_entry_rcu(tmp_pa, &lg->lg_prealloc_list[order],
						pa_inode_list) {
		spin_lock(&tmp_pa->pa_lock);
		if (tmp_pa->pa_deleted) {
			spin_unlock(&tmp_pa->pa_lock);
			continue;
		}
		if (!added && pa->pa_free < tmp_pa->pa_free) {
			
			list_add_tail_rcu(&pa->pa_inode_list,
						&tmp_pa->pa_inode_list);
			added = 1;
		}
		spin_unlock(&tmp_pa->pa_lock);
		lg_prealloc_count++;
	}
	if (!added)
		list_add_tail_rcu(&pa->pa_inode_list,
					&lg->lg_prealloc_list[order]);
	rcu_read_unlock();

	
	if (lg_prealloc_count > 8) {
		ext4_mb_discard_lg_preallocations(sb, lg,
						order, lg_prealloc_count);
		return;
	}
	return ;
}

static int ext4_mb_release_context(struct ext4_allocation_context *ac)
{
	struct ext4_sb_info *sbi = EXT4_SB(ac->ac_sb);
	struct ext4_prealloc_space *pa = ac->ac_pa;
	if (pa) {
		if (pa->pa_type == MB_GROUP_PA) {
			
			spin_lock(&pa->pa_lock);
			pa->pa_pstart += EXT4_C2B(sbi, ac->ac_b_ex.fe_len);
			pa->pa_lstart += EXT4_C2B(sbi, ac->ac_b_ex.fe_len);
			pa->pa_free -= ac->ac_b_ex.fe_len;
			pa->pa_len -= ac->ac_b_ex.fe_len;
			spin_unlock(&pa->pa_lock);
		}
	}
	if (pa) {
		if ((pa->pa_type == MB_GROUP_PA) && likely(pa->pa_free)) {
			spin_lock(pa->pa_obj_lock);
			list_del_rcu(&pa->pa_inode_list);
			spin_unlock(pa->pa_obj_lock);
			ext4_mb_add_n_trim(ac);
		}
		ext4_mb_put_pa(ac, ac->ac_sb, pa);
	}
	if (ac->ac_bitmap_page)
		page_cache_release(ac->ac_bitmap_page);
	if (ac->ac_buddy_page)
		page_cache_release(ac->ac_buddy_page);
	if (ac->ac_flags & EXT4_MB_HINT_GROUP_ALLOC)
		mutex_unlock(&ac->ac_lg->lg_mutex);
	ext4_mb_collect_stats(ac);
	return 0;
}

static int ext4_mb_discard_preallocations(struct super_block *sb, int needed)
{
	ext4_group_t i, ngroups = ext4_get_groups_count(sb);
	int ret;
	int freed = 0;

	trace_ext4_mb_discard_preallocations(sb, needed);
	for (i = 0; i < ngroups && needed > 0; i++) {
		ret = ext4_mb_discard_group_preallocations(sb, i, needed);
		freed += ret;
		needed -= ret;
	}

	return freed;
}

ext4_fsblk_t ext4_mb_new_blocks(handle_t *handle,
				struct ext4_allocation_request *ar, int *errp)
{
	int freed;
	struct ext4_allocation_context *ac = NULL;
	struct ext4_sb_info *sbi;
	struct super_block *sb;
	ext4_fsblk_t block = 0;
	unsigned int inquota = 0;
	unsigned int reserv_clstrs = 0;

	sb = ar->inode->i_sb;
	sbi = EXT4_SB(sb);

	trace_ext4_request_blocks(ar);

	
	if (IS_NOQUOTA(ar->inode))
		ar->flags |= EXT4_MB_USE_ROOT_BLOCKS;

	if (ext4_test_inode_state(ar->inode, EXT4_STATE_DELALLOC_RESERVED))
		ar->flags |= EXT4_MB_DELALLOC_RESERVED;
	else {
		while (ar->len &&
			ext4_claim_free_clusters(sbi, ar->len, ar->flags)) {

			
			yield();
			ar->len = ar->len >> 1;
		}
		if (!ar->len) {
			*errp = -ENOSPC;
			return 0;
		}
		reserv_clstrs = ar->len;
		if (ar->flags & EXT4_MB_USE_ROOT_BLOCKS) {
			dquot_alloc_block_nofail(ar->inode,
						 EXT4_C2B(sbi, ar->len));
		} else {
			while (ar->len &&
				dquot_alloc_block(ar->inode,
						  EXT4_C2B(sbi, ar->len))) {

				ar->flags |= EXT4_MB_HINT_NOPREALLOC;
				ar->len--;
			}
		}
		inquota = ar->len;
		if (ar->len == 0) {
			*errp = -EDQUOT;
			goto out;
		}
	}

	ac = kmem_cache_alloc(ext4_ac_cachep, GFP_NOFS);
	if (!ac) {
		ar->len = 0;
		*errp = -ENOMEM;
		goto out;
	}

	*errp = ext4_mb_initialize_context(ac, ar);
	if (*errp) {
		ar->len = 0;
		goto out;
	}

	ac->ac_op = EXT4_MB_HISTORY_PREALLOC;
	if (!ext4_mb_use_preallocated(ac)) {
		ac->ac_op = EXT4_MB_HISTORY_ALLOC;
		ext4_mb_normalize_request(ac, ar);
repeat:
		
		*errp = ext4_mb_regular_allocator(ac);
		if (*errp)
			goto errout;

		if (ac->ac_status == AC_STATUS_FOUND &&
				ac->ac_o_ex.fe_len < ac->ac_b_ex.fe_len)
			ext4_mb_new_preallocation(ac);
	}
	if (likely(ac->ac_status == AC_STATUS_FOUND)) {
		*errp = ext4_mb_mark_diskspace_used(ac, handle, reserv_clstrs);
		if (*errp == -EAGAIN) {
			ext4_mb_release_context(ac);
			ac->ac_b_ex.fe_group = 0;
			ac->ac_b_ex.fe_start = 0;
			ac->ac_b_ex.fe_len = 0;
			ac->ac_status = AC_STATUS_CONTINUE;
			goto repeat;
		} else if (*errp)
		errout:
			ext4_discard_allocated_blocks(ac);
		else {
			block = ext4_grp_offs_to_block(sb, &ac->ac_b_ex);
			ar->len = ac->ac_b_ex.fe_len;
		}
	} else {
		freed  = ext4_mb_discard_preallocations(sb, ac->ac_o_ex.fe_len);
		if (freed)
			goto repeat;
		*errp = -ENOSPC;
	}

	if (*errp) {
		ac->ac_b_ex.fe_len = 0;
		ar->len = 0;
		ext4_mb_show_ac(ac);
	}
	ext4_mb_release_context(ac);
out:
	if (ac)
		kmem_cache_free(ext4_ac_cachep, ac);
	if (inquota && ar->len < inquota)
		dquot_free_block(ar->inode, EXT4_C2B(sbi, inquota - ar->len));
	if (!ar->len) {
		if (!ext4_test_inode_state(ar->inode,
					   EXT4_STATE_DELALLOC_RESERVED))
			
			percpu_counter_sub(&sbi->s_dirtyclusters_counter,
						reserv_clstrs);
	}

	trace_ext4_allocate_blocks(ar, (unsigned long long)block);

	return block;
}

static int can_merge(struct ext4_free_data *entry1,
			struct ext4_free_data *entry2)
{
	if ((entry1->efd_tid == entry2->efd_tid) &&
	    (entry1->efd_group == entry2->efd_group) &&
	    ((entry1->efd_start_cluster + entry1->efd_count) == entry2->efd_start_cluster))
		return 1;
	return 0;
}

static noinline_for_stack int
ext4_mb_free_metadata(handle_t *handle, struct ext4_buddy *e4b,
		      struct ext4_free_data *new_entry)
{
	ext4_group_t group = e4b->bd_group;
	ext4_grpblk_t cluster;
	struct ext4_free_data *entry;
	struct ext4_group_info *db = e4b->bd_info;
	struct super_block *sb = e4b->bd_sb;
	struct ext4_sb_info *sbi = EXT4_SB(sb);
	struct rb_node **n = &db->bb_free_root.rb_node, *node;
	struct rb_node *parent = NULL, *new_node;

	BUG_ON(!ext4_handle_valid(handle));
	BUG_ON(e4b->bd_bitmap_page == NULL);
	BUG_ON(e4b->bd_buddy_page == NULL);

	new_node = &new_entry->efd_node;
	cluster = new_entry->efd_start_cluster;

	if (!*n) {
		page_cache_get(e4b->bd_buddy_page);
		page_cache_get(e4b->bd_bitmap_page);
	}
	while (*n) {
		parent = *n;
		entry = rb_entry(parent, struct ext4_free_data, efd_node);
		if (cluster < entry->efd_start_cluster)
			n = &(*n)->rb_left;
		else if (cluster >= (entry->efd_start_cluster + entry->efd_count))
			n = &(*n)->rb_right;
		else {
			ext4_grp_locked_error(sb, group, 0,
				ext4_group_first_block_no(sb, group) +
				EXT4_C2B(sbi, cluster),
				"Block already on to-be-freed list");
			return 0;
		}
	}

	rb_link_node(new_node, parent, n);
	rb_insert_color(new_node, &db->bb_free_root);

	
	node = rb_prev(new_node);
	if (node) {
		entry = rb_entry(node, struct ext4_free_data, efd_node);
		if (can_merge(entry, new_entry) &&
		    ext4_journal_callback_try_del(handle, &entry->efd_jce)) {
			new_entry->efd_start_cluster = entry->efd_start_cluster;
			new_entry->efd_count += entry->efd_count;
			rb_erase(node, &(db->bb_free_root));
			kmem_cache_free(ext4_free_data_cachep, entry);
		}
	}

	node = rb_next(new_node);
	if (node) {
		entry = rb_entry(node, struct ext4_free_data, efd_node);
		if (can_merge(new_entry, entry) &&
		    ext4_journal_callback_try_del(handle, &entry->efd_jce)) {
			new_entry->efd_count += entry->efd_count;
			rb_erase(node, &(db->bb_free_root));
			kmem_cache_free(ext4_free_data_cachep, entry);
		}
	}
	
	ext4_journal_callback_add(handle, ext4_free_data_callback,
				  &new_entry->efd_jce);
	return 0;
}

void ext4_free_blocks(handle_t *handle, struct inode *inode,
		      struct buffer_head *bh, ext4_fsblk_t block,
		      unsigned long count, int flags)
{
	struct buffer_head *bitmap_bh = NULL;
	struct super_block *sb = inode->i_sb;
	struct ext4_group_desc *gdp;
	unsigned long freed = 0;
	unsigned int overflow;
	ext4_grpblk_t bit;
	struct buffer_head *gd_bh;
	ext4_group_t block_group;
	struct ext4_sb_info *sbi;
	struct ext4_buddy e4b;
	unsigned int count_clusters;
	int err = 0;
	int ret;

	if (bh) {
		if (block)
			BUG_ON(block != bh->b_blocknr);
		else
			block = bh->b_blocknr;
	}

	sbi = EXT4_SB(sb);
	if (!(flags & EXT4_FREE_BLOCKS_VALIDATED) &&
	    !ext4_data_block_valid(sbi, block, count)) {
		ext4_error(sb, "Freeing blocks not in datazone - "
			   "block = %llu, count = %lu", block, count);
		goto error_return;
	}

	ext4_debug("freeing block %llu\n", block);
	trace_ext4_free_blocks(inode, block, count, flags);

	if (flags & EXT4_FREE_BLOCKS_FORGET) {
		struct buffer_head *tbh = bh;
		int i;

		BUG_ON(bh && (count > 1));

		for (i = 0; i < count; i++) {
			if (!bh)
				tbh = sb_find_get_block(inode->i_sb,
							block + i);
			if (unlikely(!tbh))
				continue;
			ext4_forget(handle, flags & EXT4_FREE_BLOCKS_METADATA,
				    inode, tbh, block + i);
		}
	}

	/*
	 * We need to make sure we don't reuse the freed block until
	 * after the transaction is committed, which we can do by
	 * treating the block as metadata, below.  We make an
	 * exception if the inode is to be written in writeback mode
	 * since writeback mode has weak data consistency guarantees.
	 */
	if (!ext4_should_writeback_data(inode))
		flags |= EXT4_FREE_BLOCKS_METADATA;

	overflow = block & (sbi->s_cluster_ratio - 1);
	if (overflow) {
		if (flags & EXT4_FREE_BLOCKS_NOFREE_FIRST_CLUSTER) {
			overflow = sbi->s_cluster_ratio - overflow;
			block += overflow;
			if (count > overflow)
				count -= overflow;
			else
				return;
		} else {
			block -= overflow;
			count += overflow;
		}
	}
	overflow = count & (sbi->s_cluster_ratio - 1);
	if (overflow) {
		if (flags & EXT4_FREE_BLOCKS_NOFREE_LAST_CLUSTER) {
			if (count > overflow)
				count -= overflow;
			else
				return;
		} else
			count += sbi->s_cluster_ratio - overflow;
	}

do_more:
	overflow = 0;
	ext4_get_group_no_and_offset(sb, block, &block_group, &bit);

	if (EXT4_C2B(sbi, bit) + count > EXT4_BLOCKS_PER_GROUP(sb)) {
		overflow = EXT4_C2B(sbi, bit) + count -
			EXT4_BLOCKS_PER_GROUP(sb);
		count -= overflow;
	}
	count_clusters = EXT4_B2C(sbi, count);
	bitmap_bh = ext4_read_block_bitmap(sb, block_group);
	if (!bitmap_bh) {
		err = -EIO;
		goto error_return;
	}
	gdp = ext4_get_group_desc(sb, block_group, &gd_bh);
	if (!gdp) {
		err = -EIO;
		goto error_return;
	}

	if (in_range(ext4_block_bitmap(sb, gdp), block, count) ||
	    in_range(ext4_inode_bitmap(sb, gdp), block, count) ||
	    in_range(block, ext4_inode_table(sb, gdp),
		     EXT4_SB(sb)->s_itb_per_group) ||
	    in_range(block + count - 1, ext4_inode_table(sb, gdp),
		     EXT4_SB(sb)->s_itb_per_group)) {

		ext4_error(sb, "Freeing blocks in system zone - "
			   "Block = %llu, count = %lu", block, count);
		
		goto error_return;
	}

	BUFFER_TRACE(bitmap_bh, "getting write access");
	err = ext4_journal_get_write_access(handle, bitmap_bh);
	if (err)
		goto error_return;

	BUFFER_TRACE(gd_bh, "get_write_access");
	err = ext4_journal_get_write_access(handle, gd_bh);
	if (err)
		goto error_return;
#ifdef AGGRESSIVE_CHECK
	{
		int i;
		for (i = 0; i < count_clusters; i++)
			BUG_ON(!mb_test_bit(bit + i, bitmap_bh->b_data));
	}
#endif
	trace_ext4_mballoc_free(sb, inode, block_group, bit, count_clusters);

	err = ext4_mb_load_buddy(sb, block_group, &e4b);
	if (err)
		goto error_return;

	if ((flags & EXT4_FREE_BLOCKS_METADATA) && ext4_handle_valid(handle)) {
		struct ext4_free_data *new_entry;
		new_entry = kmem_cache_alloc(ext4_free_data_cachep, GFP_NOFS);
		if (!new_entry) {
			err = -ENOMEM;
			goto error_return;
		}
		new_entry->efd_start_cluster = bit;
		new_entry->efd_group = block_group;
		new_entry->efd_count = count_clusters;
		new_entry->efd_tid = handle->h_transaction->t_tid;

		ext4_lock_group(sb, block_group);
		mb_clear_bits(bitmap_bh->b_data, bit, count_clusters);
		ext4_mb_free_metadata(handle, &e4b, new_entry);
	} else {
		ext4_lock_group(sb, block_group);
		mb_clear_bits(bitmap_bh->b_data, bit, count_clusters);
		mb_free_blocks(inode, &e4b, bit, count_clusters);
	}

	ret = ext4_free_group_clusters(sb, gdp) + count_clusters;
	ext4_free_group_clusters_set(sb, gdp, ret);
	gdp->bg_checksum = ext4_group_desc_csum(sbi, block_group, gdp);
	ext4_unlock_group(sb, block_group);
	percpu_counter_add(&sbi->s_freeclusters_counter, count_clusters);

	if (sbi->s_log_groups_per_flex) {
		ext4_group_t flex_group = ext4_flex_group(sbi, block_group);
		atomic_add(count_clusters,
			   &sbi->s_flex_groups[flex_group].free_clusters);
	}

	ext4_mb_unload_buddy(&e4b);

	freed += count;

	if (!(flags & EXT4_FREE_BLOCKS_NO_QUOT_UPDATE))
		dquot_free_block(inode, EXT4_C2B(sbi, count_clusters));

	
	BUFFER_TRACE(bitmap_bh, "dirtied bitmap block");
	err = ext4_handle_dirty_metadata(handle, NULL, bitmap_bh);

	
	BUFFER_TRACE(gd_bh, "dirtied group descriptor block");
	ret = ext4_handle_dirty_metadata(handle, NULL, gd_bh);
	if (!err)
		err = ret;

	if (overflow && !err) {
		block += count;
		count = overflow;
		put_bh(bitmap_bh);
		goto do_more;
	}
	ext4_mark_super_dirty(sb);
error_return:
	brelse(bitmap_bh);
	ext4_std_error(sb, err);
	return;
}

int ext4_group_add_blocks(handle_t *handle, struct super_block *sb,
			 ext4_fsblk_t block, unsigned long count)
{
	struct buffer_head *bitmap_bh = NULL;
	struct buffer_head *gd_bh;
	ext4_group_t block_group;
	ext4_grpblk_t bit;
	unsigned int i;
	struct ext4_group_desc *desc;
	struct ext4_sb_info *sbi = EXT4_SB(sb);
	struct ext4_buddy e4b;
	int err = 0, ret, blk_free_count;
	ext4_grpblk_t blocks_freed;

	ext4_debug("Adding block(s) %llu-%llu\n", block, block + count - 1);

	if (count == 0)
		return 0;

	ext4_get_group_no_and_offset(sb, block, &block_group, &bit);
	if (bit + count > EXT4_BLOCKS_PER_GROUP(sb)) {
		ext4_warning(sb, "too much blocks added to group %u\n",
			     block_group);
		err = -EINVAL;
		goto error_return;
	}

	bitmap_bh = ext4_read_block_bitmap(sb, block_group);
	if (!bitmap_bh) {
		err = -EIO;
		goto error_return;
	}

	desc = ext4_get_group_desc(sb, block_group, &gd_bh);
	if (!desc) {
		err = -EIO;
		goto error_return;
	}

	if (in_range(ext4_block_bitmap(sb, desc), block, count) ||
	    in_range(ext4_inode_bitmap(sb, desc), block, count) ||
	    in_range(block, ext4_inode_table(sb, desc), sbi->s_itb_per_group) ||
	    in_range(block + count - 1, ext4_inode_table(sb, desc),
		     sbi->s_itb_per_group)) {
		ext4_error(sb, "Adding blocks in system zones - "
			   "Block = %llu, count = %lu",
			   block, count);
		err = -EINVAL;
		goto error_return;
	}

	BUFFER_TRACE(bitmap_bh, "getting write access");
	err = ext4_journal_get_write_access(handle, bitmap_bh);
	if (err)
		goto error_return;

	BUFFER_TRACE(gd_bh, "get_write_access");
	err = ext4_journal_get_write_access(handle, gd_bh);
	if (err)
		goto error_return;

	for (i = 0, blocks_freed = 0; i < count; i++) {
		BUFFER_TRACE(bitmap_bh, "clear bit");
		if (!mb_test_bit(bit + i, bitmap_bh->b_data)) {
			ext4_error(sb, "bit already cleared for block %llu",
				   (ext4_fsblk_t)(block + i));
			BUFFER_TRACE(bitmap_bh, "bit already cleared");
		} else {
			blocks_freed++;
		}
	}

	err = ext4_mb_load_buddy(sb, block_group, &e4b);
	if (err)
		goto error_return;

	ext4_lock_group(sb, block_group);
	mb_clear_bits(bitmap_bh->b_data, bit, count);
	mb_free_blocks(NULL, &e4b, bit, count);
	blk_free_count = blocks_freed + ext4_free_group_clusters(sb, desc);
	ext4_free_group_clusters_set(sb, desc, blk_free_count);
	desc->bg_checksum = ext4_group_desc_csum(sbi, block_group, desc);
	ext4_unlock_group(sb, block_group);
	percpu_counter_add(&sbi->s_freeclusters_counter,
			   EXT4_B2C(sbi, blocks_freed));

	if (sbi->s_log_groups_per_flex) {
		ext4_group_t flex_group = ext4_flex_group(sbi, block_group);
		atomic_add(EXT4_B2C(sbi, blocks_freed),
			   &sbi->s_flex_groups[flex_group].free_clusters);
	}

	ext4_mb_unload_buddy(&e4b);

	
	BUFFER_TRACE(bitmap_bh, "dirtied bitmap block");
	err = ext4_handle_dirty_metadata(handle, NULL, bitmap_bh);

	
	BUFFER_TRACE(gd_bh, "dirtied group descriptor block");
	ret = ext4_handle_dirty_metadata(handle, NULL, gd_bh);
	if (!err)
		err = ret;

error_return:
	brelse(bitmap_bh);
	ext4_std_error(sb, err);
	return err;
}

static void ext4_trim_extent(struct super_block *sb, int start, int count,
			     ext4_group_t group, struct ext4_buddy *e4b)
{
	struct ext4_free_extent ex;

	trace_ext4_trim_extent(sb, group, start, count);

	assert_spin_locked(ext4_group_lock_ptr(sb, group));

	ex.fe_start = start;
	ex.fe_group = group;
	ex.fe_len = count;

	mb_mark_used(e4b, &ex);
	ext4_unlock_group(sb, group);
	ext4_issue_discard(sb, group, start, count);
	ext4_lock_group(sb, group);
	mb_free_blocks(NULL, e4b, start, ex.fe_len);
}

static ext4_grpblk_t
ext4_trim_all_free(struct super_block *sb, ext4_group_t group,
		   ext4_grpblk_t start, ext4_grpblk_t max,
		   ext4_grpblk_t minblocks)
{
	void *bitmap;
	ext4_grpblk_t next, count = 0, free_count = 0;
	struct ext4_buddy e4b;
	int ret;

	trace_ext4_trim_all_free(sb, group, start, max);

	ret = ext4_mb_load_buddy(sb, group, &e4b);
	if (ret) {
		ext4_error(sb, "Error in loading buddy "
				"information for %u", group);
		return ret;
	}
	bitmap = e4b.bd_bitmap;

	ext4_lock_group(sb, group);
	if (EXT4_MB_GRP_WAS_TRIMMED(e4b.bd_info) &&
	    minblocks >= atomic_read(&EXT4_SB(sb)->s_last_trim_minblks))
		goto out;

	start = (e4b.bd_info->bb_first_free > start) ?
		e4b.bd_info->bb_first_free : start;

	while (start <= max) {
		start = mb_find_next_zero_bit(bitmap, max + 1, start);
		if (start > max)
			break;
		next = mb_find_next_bit(bitmap, max + 1, start);

		if ((next - start) >= minblocks) {
			ext4_trim_extent(sb, start,
					 next - start, group, &e4b);
			count += next - start;
		}
		free_count += next - start;
		start = next + 1;

		if (fatal_signal_pending(current)) {
			count = -ERESTARTSYS;
			break;
		}

		if (need_resched()) {
			ext4_unlock_group(sb, group);
			cond_resched();
			ext4_lock_group(sb, group);
		}

		if ((e4b.bd_info->bb_free - free_count) < minblocks)
			break;
	}

	if (!ret)
		EXT4_MB_GRP_SET_TRIMMED(e4b.bd_info);
out:
	ext4_unlock_group(sb, group);
	ext4_mb_unload_buddy(&e4b);

	ext4_debug("trimmed %d blocks in the group %d\n",
		count, group);

	return count;
}

int ext4_trim_fs(struct super_block *sb, struct fstrim_range *range)
{
	struct ext4_group_info *grp;
	ext4_group_t group, first_group, last_group;
	ext4_grpblk_t cnt = 0, first_cluster, last_cluster;
	uint64_t start, end, minlen, trimmed = 0;
	ext4_fsblk_t first_data_blk =
			le32_to_cpu(EXT4_SB(sb)->s_es->s_first_data_block);
	ext4_fsblk_t max_blks = ext4_blocks_count(EXT4_SB(sb)->s_es);
	int ret = 0;

	start = range->start >> sb->s_blocksize_bits;
	end = start + (range->len >> sb->s_blocksize_bits) - 1;
	minlen = range->minlen >> sb->s_blocksize_bits;

	if (unlikely(minlen > EXT4_CLUSTERS_PER_GROUP(sb)) ||
	    unlikely(start >= max_blks))
		return -EINVAL;
	if (end >= max_blks)
		end = max_blks - 1;
	if (end <= first_data_blk)
		goto out;
	if (start < first_data_blk)
		start = first_data_blk;

	
	ext4_get_group_no_and_offset(sb, (ext4_fsblk_t) start,
				     &first_group, &first_cluster);
	ext4_get_group_no_and_offset(sb, (ext4_fsblk_t) end,
				     &last_group, &last_cluster);

	
	end = EXT4_CLUSTERS_PER_GROUP(sb) - 1;

	for (group = first_group; group <= last_group; group++) {
		grp = ext4_get_group_info(sb, group);
		
		if (unlikely(EXT4_MB_GRP_NEED_INIT(grp))) {
			ret = ext4_mb_init_group(sb, group);
			if (ret)
				break;
		}

		if (group == last_group)
			end = last_cluster;

		if (grp->bb_free >= minlen) {
			cnt = ext4_trim_all_free(sb, group, first_cluster,
						end, minlen);
			if (cnt < 0) {
				ret = cnt;
				break;
			}
			trimmed += cnt;
		}

		first_cluster = 0;
	}

	if (!ret)
		atomic_set(&EXT4_SB(sb)->s_last_trim_minblks, minlen);

out:
	range->len = trimmed * sb->s_blocksize;
	return ret;
}
