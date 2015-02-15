/*
 * Copyright (c) 2003-2006, Cluster File Systems, Inc, info@clusterfs.com
 * Written by Alex Tomas <alex@clusterfs.com>
 *
 * Architecture independence:
 *   Copyright (c) 2005, Bull S.A.
 *   Written by Pierre Peiffer <pierre.peiffer@bull.net>
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


#include <linux/fs.h>
#include <linux/time.h>
#include <linux/jbd2.h>
#include <linux/highuid.h>
#include <linux/pagemap.h>
#include <linux/quotaops.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/falloc.h>
#include <asm/uaccess.h>
#include <linux/fiemap.h>
#include "ext4_jbd2.h"

#include <trace/events/ext4.h>

#define EXT4_EXT_MAY_ZEROOUT	0x1  
#define EXT4_EXT_MARK_UNINIT1	0x2  
#define EXT4_EXT_MARK_UNINIT2	0x4  

static int ext4_split_extent(handle_t *handle,
				struct inode *inode,
				struct ext4_ext_path *path,
				struct ext4_map_blocks *map,
				int split_flag,
				int flags);

static int ext4_split_extent_at(handle_t *handle,
			     struct inode *inode,
			     struct ext4_ext_path *path,
			     ext4_lblk_t split,
			     int split_flag,
			     int flags);

static int ext4_ext_truncate_extend_restart(handle_t *handle,
					    struct inode *inode,
					    int needed)
{
	int err;

	if (!ext4_handle_valid(handle))
		return 0;
	if (handle->h_buffer_credits > needed)
		return 0;
	err = ext4_journal_extend(handle, needed);
	if (err <= 0)
		return err;
	err = ext4_truncate_restart_trans(handle, inode, needed);
	if (err == 0)
		err = -EAGAIN;

	return err;
}

static int ext4_ext_get_access(handle_t *handle, struct inode *inode,
				struct ext4_ext_path *path)
{
	if (path->p_bh) {
		
		return ext4_journal_get_write_access(handle, path->p_bh);
	}
	
	
	return 0;
}

#define ext4_ext_dirty(handle, inode, path) \
		__ext4_ext_dirty(__func__, __LINE__, (handle), (inode), (path))
static int __ext4_ext_dirty(const char *where, unsigned int line,
			    handle_t *handle, struct inode *inode,
			    struct ext4_ext_path *path)
{
	int err;
	if (path->p_bh) {
		
		err = __ext4_handle_dirty_metadata(where, line, handle,
						   inode, path->p_bh);
	} else {
		
		err = ext4_mark_inode_dirty(handle, inode);
	}
	return err;
}

static ext4_fsblk_t ext4_ext_find_goal(struct inode *inode,
			      struct ext4_ext_path *path,
			      ext4_lblk_t block)
{
	if (path) {
		int depth = path->p_depth;
		struct ext4_extent *ex;

		ex = path[depth].p_ext;
		if (ex) {
			ext4_fsblk_t ext_pblk = ext4_ext_pblock(ex);
			ext4_lblk_t ext_block = le32_to_cpu(ex->ee_block);

			if (block > ext_block)
				return ext_pblk + (block - ext_block);
			else
				return ext_pblk - (ext_block - block);
		}

		if (path[depth].p_bh)
			return path[depth].p_bh->b_blocknr;
	}

	
	return ext4_inode_to_goal_block(inode);
}

static ext4_fsblk_t
ext4_ext_new_meta_block(handle_t *handle, struct inode *inode,
			struct ext4_ext_path *path,
			struct ext4_extent *ex, int *err, unsigned int flags)
{
	ext4_fsblk_t goal, newblock;

	goal = ext4_ext_find_goal(inode, path, le32_to_cpu(ex->ee_block));
	newblock = ext4_new_meta_blocks(handle, inode, goal, flags,
					NULL, err);
	return newblock;
}

static inline int ext4_ext_space_block(struct inode *inode, int check)
{
	int size;

	size = (inode->i_sb->s_blocksize - sizeof(struct ext4_extent_header))
			/ sizeof(struct ext4_extent);
#ifdef AGGRESSIVE_TEST
	if (!check && size > 6)
		size = 6;
#endif
	return size;
}

static inline int ext4_ext_space_block_idx(struct inode *inode, int check)
{
	int size;

	size = (inode->i_sb->s_blocksize - sizeof(struct ext4_extent_header))
			/ sizeof(struct ext4_extent_idx);
#ifdef AGGRESSIVE_TEST
	if (!check && size > 5)
		size = 5;
#endif
	return size;
}

static inline int ext4_ext_space_root(struct inode *inode, int check)
{
	int size;

	size = sizeof(EXT4_I(inode)->i_data);
	size -= sizeof(struct ext4_extent_header);
	size /= sizeof(struct ext4_extent);
#ifdef AGGRESSIVE_TEST
	if (!check && size > 3)
		size = 3;
#endif
	return size;
}

static inline int ext4_ext_space_root_idx(struct inode *inode, int check)
{
	int size;

	size = sizeof(EXT4_I(inode)->i_data);
	size -= sizeof(struct ext4_extent_header);
	size /= sizeof(struct ext4_extent_idx);
#ifdef AGGRESSIVE_TEST
	if (!check && size > 4)
		size = 4;
#endif
	return size;
}

int ext4_ext_calc_metadata_amount(struct inode *inode, ext4_lblk_t lblock)
{
	struct ext4_inode_info *ei = EXT4_I(inode);
	int idxs;

	idxs = ((inode->i_sb->s_blocksize - sizeof(struct ext4_extent_header))
		/ sizeof(struct ext4_extent_idx));

	if (ei->i_da_metadata_calc_len &&
	    ei->i_da_metadata_calc_last_lblock+1 == lblock) {
		int num = 0;

		if ((ei->i_da_metadata_calc_len % idxs) == 0)
			num++;
		if ((ei->i_da_metadata_calc_len % (idxs*idxs)) == 0)
			num++;
		if ((ei->i_da_metadata_calc_len % (idxs*idxs*idxs)) == 0) {
			num++;
			ei->i_da_metadata_calc_len = 0;
		} else
			ei->i_da_metadata_calc_len++;
		ei->i_da_metadata_calc_last_lblock++;
		return num;
	}

	ei->i_da_metadata_calc_len = 1;
	ei->i_da_metadata_calc_last_lblock = lblock;
	return ext_depth(inode) + 1;
}

static int
ext4_ext_max_entries(struct inode *inode, int depth)
{
	int max;

	if (depth == ext_depth(inode)) {
		if (depth == 0)
			max = ext4_ext_space_root(inode, 1);
		else
			max = ext4_ext_space_root_idx(inode, 1);
	} else {
		if (depth == 0)
			max = ext4_ext_space_block(inode, 1);
		else
			max = ext4_ext_space_block_idx(inode, 1);
	}

	return max;
}

static int ext4_valid_extent(struct inode *inode, struct ext4_extent *ext)
{
	ext4_fsblk_t block = ext4_ext_pblock(ext);
	int len = ext4_ext_get_actual_len(ext);

	if (len == 0)
		return 0;
	return ext4_data_block_valid(EXT4_SB(inode->i_sb), block, len);
}

static int ext4_valid_extent_idx(struct inode *inode,
				struct ext4_extent_idx *ext_idx)
{
	ext4_fsblk_t block = ext4_idx_pblock(ext_idx);

	return ext4_data_block_valid(EXT4_SB(inode->i_sb), block, 1);
}

static int ext4_valid_extent_entries(struct inode *inode,
				struct ext4_extent_header *eh,
				int depth)
{
	unsigned short entries;
	if (eh->eh_entries == 0)
		return 1;

	entries = le16_to_cpu(eh->eh_entries);

	if (depth == 0) {
		
		struct ext4_extent *ext = EXT_FIRST_EXTENT(eh);
		while (entries) {
			if (!ext4_valid_extent(inode, ext))
				return 0;
			ext++;
			entries--;
		}
	} else {
		struct ext4_extent_idx *ext_idx = EXT_FIRST_INDEX(eh);
		while (entries) {
			if (!ext4_valid_extent_idx(inode, ext_idx))
				return 0;
			ext_idx++;
			entries--;
		}
	}
	return 1;
}

static int __ext4_ext_check(const char *function, unsigned int line,
			    struct inode *inode, struct ext4_extent_header *eh,
			    int depth)
{
	const char *error_msg;
	int max = 0;

	if (unlikely(eh->eh_magic != EXT4_EXT_MAGIC)) {
		error_msg = "invalid magic";
		goto corrupted;
	}
	if (unlikely(le16_to_cpu(eh->eh_depth) != depth)) {
		error_msg = "unexpected eh_depth";
		goto corrupted;
	}
	if (unlikely(eh->eh_max == 0)) {
		error_msg = "invalid eh_max";
		goto corrupted;
	}
	max = ext4_ext_max_entries(inode, depth);
	if (unlikely(le16_to_cpu(eh->eh_max) > max)) {
		error_msg = "too large eh_max";
		goto corrupted;
	}
	if (unlikely(le16_to_cpu(eh->eh_entries) > le16_to_cpu(eh->eh_max))) {
		error_msg = "invalid eh_entries";
		goto corrupted;
	}
	if (!ext4_valid_extent_entries(inode, eh, depth)) {
		error_msg = "invalid extent entries";
		goto corrupted;
	}
	return 0;

corrupted:
	ext4_error_inode(inode, function, line, 0,
			"bad header/extent: %s - magic %x, "
			"entries %u, max %u(%u), depth %u(%u)",
			error_msg, le16_to_cpu(eh->eh_magic),
			le16_to_cpu(eh->eh_entries), le16_to_cpu(eh->eh_max),
			max, le16_to_cpu(eh->eh_depth), depth);

	return -EIO;
}

#define ext4_ext_check(inode, eh, depth)	\
	__ext4_ext_check(__func__, __LINE__, inode, eh, depth)

int ext4_ext_check_inode(struct inode *inode)
{
	return ext4_ext_check(inode, ext_inode_hdr(inode), ext_depth(inode));
}

#ifdef EXT_DEBUG
static void ext4_ext_show_path(struct inode *inode, struct ext4_ext_path *path)
{
	int k, l = path->p_depth;

	ext_debug("path:");
	for (k = 0; k <= l; k++, path++) {
		if (path->p_idx) {
		  ext_debug("  %d->%llu", le32_to_cpu(path->p_idx->ei_block),
			    ext4_idx_pblock(path->p_idx));
		} else if (path->p_ext) {
			ext_debug("  %d:[%d]%d:%llu ",
				  le32_to_cpu(path->p_ext->ee_block),
				  ext4_ext_is_uninitialized(path->p_ext),
				  ext4_ext_get_actual_len(path->p_ext),
				  ext4_ext_pblock(path->p_ext));
		} else
			ext_debug("  []");
	}
	ext_debug("\n");
}

static void ext4_ext_show_leaf(struct inode *inode, struct ext4_ext_path *path)
{
	int depth = ext_depth(inode);
	struct ext4_extent_header *eh;
	struct ext4_extent *ex;
	int i;

	if (!path)
		return;

	eh = path[depth].p_hdr;
	ex = EXT_FIRST_EXTENT(eh);

	ext_debug("Displaying leaf extents for inode %lu\n", inode->i_ino);

	for (i = 0; i < le16_to_cpu(eh->eh_entries); i++, ex++) {
		ext_debug("%d:[%d]%d:%llu ", le32_to_cpu(ex->ee_block),
			  ext4_ext_is_uninitialized(ex),
			  ext4_ext_get_actual_len(ex), ext4_ext_pblock(ex));
	}
	ext_debug("\n");
}

static void ext4_ext_show_move(struct inode *inode, struct ext4_ext_path *path,
			ext4_fsblk_t newblock, int level)
{
	int depth = ext_depth(inode);
	struct ext4_extent *ex;

	if (depth != level) {
		struct ext4_extent_idx *idx;
		idx = path[level].p_idx;
		while (idx <= EXT_MAX_INDEX(path[level].p_hdr)) {
			ext_debug("%d: move %d:%llu in new index %llu\n", level,
					le32_to_cpu(idx->ei_block),
					ext4_idx_pblock(idx),
					newblock);
			idx++;
		}

		return;
	}

	ex = path[depth].p_ext;
	while (ex <= EXT_MAX_EXTENT(path[depth].p_hdr)) {
		ext_debug("move %d:%llu:[%d]%d in new leaf %llu\n",
				le32_to_cpu(ex->ee_block),
				ext4_ext_pblock(ex),
				ext4_ext_is_uninitialized(ex),
				ext4_ext_get_actual_len(ex),
				newblock);
		ex++;
	}
}

#else
#define ext4_ext_show_path(inode, path)
#define ext4_ext_show_leaf(inode, path)
#define ext4_ext_show_move(inode, path, newblock, level)
#endif

void ext4_ext_drop_refs(struct ext4_ext_path *path)
{
	int depth = path->p_depth;
	int i;

	for (i = 0; i <= depth; i++, path++)
		if (path->p_bh) {
			brelse(path->p_bh);
			path->p_bh = NULL;
		}
}

static void
ext4_ext_binsearch_idx(struct inode *inode,
			struct ext4_ext_path *path, ext4_lblk_t block)
{
	struct ext4_extent_header *eh = path->p_hdr;
	struct ext4_extent_idx *r, *l, *m;


	ext_debug("binsearch for %u(idx):  ", block);

	l = EXT_FIRST_INDEX(eh) + 1;
	r = EXT_LAST_INDEX(eh);
	while (l <= r) {
		m = l + (r - l) / 2;
		if (block < le32_to_cpu(m->ei_block))
			r = m - 1;
		else
			l = m + 1;
		ext_debug("%p(%u):%p(%u):%p(%u) ", l, le32_to_cpu(l->ei_block),
				m, le32_to_cpu(m->ei_block),
				r, le32_to_cpu(r->ei_block));
	}

	path->p_idx = l - 1;
	ext_debug("  -> %d->%lld ", le32_to_cpu(path->p_idx->ei_block),
		  ext4_idx_pblock(path->p_idx));

#ifdef CHECK_BINSEARCH
	{
		struct ext4_extent_idx *chix, *ix;
		int k;

		chix = ix = EXT_FIRST_INDEX(eh);
		for (k = 0; k < le16_to_cpu(eh->eh_entries); k++, ix++) {
		  if (k != 0 &&
		      le32_to_cpu(ix->ei_block) <= le32_to_cpu(ix[-1].ei_block)) {
				printk(KERN_DEBUG "k=%d, ix=0x%p, "
				       "first=0x%p\n", k,
				       ix, EXT_FIRST_INDEX(eh));
				printk(KERN_DEBUG "%u <= %u\n",
				       le32_to_cpu(ix->ei_block),
				       le32_to_cpu(ix[-1].ei_block));
			}
			BUG_ON(k && le32_to_cpu(ix->ei_block)
					   <= le32_to_cpu(ix[-1].ei_block));
			if (block < le32_to_cpu(ix->ei_block))
				break;
			chix = ix;
		}
		BUG_ON(chix != path->p_idx);
	}
#endif

}

static void
ext4_ext_binsearch(struct inode *inode,
		struct ext4_ext_path *path, ext4_lblk_t block)
{
	struct ext4_extent_header *eh = path->p_hdr;
	struct ext4_extent *r, *l, *m;

	if (eh->eh_entries == 0) {
		return;
	}

	ext_debug("binsearch for %u:  ", block);

	l = EXT_FIRST_EXTENT(eh) + 1;
	r = EXT_LAST_EXTENT(eh);

	while (l <= r) {
		m = l + (r - l) / 2;
		if (block < le32_to_cpu(m->ee_block))
			r = m - 1;
		else
			l = m + 1;
		ext_debug("%p(%u):%p(%u):%p(%u) ", l, le32_to_cpu(l->ee_block),
				m, le32_to_cpu(m->ee_block),
				r, le32_to_cpu(r->ee_block));
	}

	path->p_ext = l - 1;
	ext_debug("  -> %d:%llu:[%d]%d ",
			le32_to_cpu(path->p_ext->ee_block),
			ext4_ext_pblock(path->p_ext),
			ext4_ext_is_uninitialized(path->p_ext),
			ext4_ext_get_actual_len(path->p_ext));

#ifdef CHECK_BINSEARCH
	{
		struct ext4_extent *chex, *ex;
		int k;

		chex = ex = EXT_FIRST_EXTENT(eh);
		for (k = 0; k < le16_to_cpu(eh->eh_entries); k++, ex++) {
			BUG_ON(k && le32_to_cpu(ex->ee_block)
					  <= le32_to_cpu(ex[-1].ee_block));
			if (block < le32_to_cpu(ex->ee_block))
				break;
			chex = ex;
		}
		BUG_ON(chex != path->p_ext);
	}
#endif

}

int ext4_ext_tree_init(handle_t *handle, struct inode *inode)
{
	struct ext4_extent_header *eh;

	eh = ext_inode_hdr(inode);
	eh->eh_depth = 0;
	eh->eh_entries = 0;
	eh->eh_magic = EXT4_EXT_MAGIC;
	eh->eh_max = cpu_to_le16(ext4_ext_space_root(inode, 0));
	ext4_mark_inode_dirty(handle, inode);
	ext4_ext_invalidate_cache(inode);
	return 0;
}

struct ext4_ext_path *
ext4_ext_find_extent(struct inode *inode, ext4_lblk_t block,
					struct ext4_ext_path *path)
{
	struct ext4_extent_header *eh;
	struct buffer_head *bh;
	short int depth, i, ppos = 0, alloc = 0;

	eh = ext_inode_hdr(inode);
	depth = ext_depth(inode);

	
	if (!path) {
		path = kzalloc(sizeof(struct ext4_ext_path) * (depth + 2),
				GFP_NOFS);
		if (!path)
			return ERR_PTR(-ENOMEM);
		alloc = 1;
	}
	path[0].p_hdr = eh;
	path[0].p_bh = NULL;

	i = depth;
	
	while (i) {
		int need_to_validate = 0;

		ext_debug("depth %d: num %d, max %d\n",
			  ppos, le16_to_cpu(eh->eh_entries), le16_to_cpu(eh->eh_max));

		ext4_ext_binsearch_idx(inode, path + ppos, block);
		path[ppos].p_block = ext4_idx_pblock(path[ppos].p_idx);
		path[ppos].p_depth = i;
		path[ppos].p_ext = NULL;

		bh = sb_getblk(inode->i_sb, path[ppos].p_block);
		if (unlikely(!bh))
			goto err;
		if (!bh_uptodate_or_lock(bh)) {
			trace_ext4_ext_load_extent(inode, block,
						path[ppos].p_block);
			if (bh_submit_read(bh) < 0) {
				put_bh(bh);
				goto err;
			}
			
			need_to_validate = 1;
		}
		eh = ext_block_hdr(bh);
		ppos++;
		if (unlikely(ppos > depth)) {
			put_bh(bh);
			EXT4_ERROR_INODE(inode,
					 "ppos %d > depth %d", ppos, depth);
			goto err;
		}
		path[ppos].p_bh = bh;
		path[ppos].p_hdr = eh;
		i--;

		if (need_to_validate && ext4_ext_check(inode, eh, i))
			goto err;
	}

	path[ppos].p_depth = i;
	path[ppos].p_ext = NULL;
	path[ppos].p_idx = NULL;

	
	ext4_ext_binsearch(inode, path + ppos, block);
	
	if (path[ppos].p_ext)
		path[ppos].p_block = ext4_ext_pblock(path[ppos].p_ext);

	ext4_ext_show_path(inode, path);

	return path;

err:
	ext4_ext_drop_refs(path);
	if (alloc)
		kfree(path);
	return ERR_PTR(-EIO);
}

static int ext4_ext_insert_index(handle_t *handle, struct inode *inode,
				 struct ext4_ext_path *curp,
				 int logical, ext4_fsblk_t ptr)
{
	struct ext4_extent_idx *ix;
	int len, err;

	err = ext4_ext_get_access(handle, inode, curp);
	if (err)
		return err;

	if (unlikely(logical == le32_to_cpu(curp->p_idx->ei_block))) {
		EXT4_ERROR_INODE(inode,
				 "logical %d == ei_block %d!",
				 logical, le32_to_cpu(curp->p_idx->ei_block));
		return -EIO;
	}

	if (unlikely(le16_to_cpu(curp->p_hdr->eh_entries)
			     >= le16_to_cpu(curp->p_hdr->eh_max))) {
		EXT4_ERROR_INODE(inode,
				 "eh_entries %d >= eh_max %d!",
				 le16_to_cpu(curp->p_hdr->eh_entries),
				 le16_to_cpu(curp->p_hdr->eh_max));
		return -EIO;
	}

	if (logical > le32_to_cpu(curp->p_idx->ei_block)) {
		
		ext_debug("insert new index %d after: %llu\n", logical, ptr);
		ix = curp->p_idx + 1;
	} else {
		
		ext_debug("insert new index %d before: %llu\n", logical, ptr);
		ix = curp->p_idx;
	}

	len = EXT_LAST_INDEX(curp->p_hdr) - ix + 1;
	BUG_ON(len < 0);
	if (len > 0) {
		ext_debug("insert new index %d: "
				"move %d indices from 0x%p to 0x%p\n",
				logical, len, ix, ix + 1);
		memmove(ix + 1, ix, len * sizeof(struct ext4_extent_idx));
	}

	if (unlikely(ix > EXT_MAX_INDEX(curp->p_hdr))) {
		EXT4_ERROR_INODE(inode, "ix > EXT_MAX_INDEX!");
		return -EIO;
	}

	ix->ei_block = cpu_to_le32(logical);
	ext4_idx_store_pblock(ix, ptr);
	le16_add_cpu(&curp->p_hdr->eh_entries, 1);

	if (unlikely(ix > EXT_LAST_INDEX(curp->p_hdr))) {
		EXT4_ERROR_INODE(inode, "ix > EXT_LAST_INDEX!");
		return -EIO;
	}

	err = ext4_ext_dirty(handle, inode, curp);
	ext4_std_error(inode->i_sb, err);

	return err;
}

static int ext4_ext_split(handle_t *handle, struct inode *inode,
			  unsigned int flags,
			  struct ext4_ext_path *path,
			  struct ext4_extent *newext, int at)
{
	struct buffer_head *bh = NULL;
	int depth = ext_depth(inode);
	struct ext4_extent_header *neh;
	struct ext4_extent_idx *fidx;
	int i = at, k, m, a;
	ext4_fsblk_t newblock, oldblock;
	__le32 border;
	ext4_fsblk_t *ablocks = NULL; 
	int err = 0;

	
	

	if (unlikely(path[depth].p_ext > EXT_MAX_EXTENT(path[depth].p_hdr))) {
		EXT4_ERROR_INODE(inode, "p_ext > EXT_MAX_EXTENT!");
		return -EIO;
	}
	if (path[depth].p_ext != EXT_MAX_EXTENT(path[depth].p_hdr)) {
		border = path[depth].p_ext[1].ee_block;
		ext_debug("leaf will be split."
				" next leaf starts at %d\n",
				  le32_to_cpu(border));
	} else {
		border = newext->ee_block;
		ext_debug("leaf will be added."
				" next leaf starts at %d\n",
				le32_to_cpu(border));
	}


	ablocks = kzalloc(sizeof(ext4_fsblk_t) * depth, GFP_NOFS);
	if (!ablocks)
		return -ENOMEM;

	
	ext_debug("allocate %d blocks for indexes/leaf\n", depth - at);
	for (a = 0; a < depth - at; a++) {
		newblock = ext4_ext_new_meta_block(handle, inode, path,
						   newext, &err, flags);
		if (newblock == 0)
			goto cleanup;
		ablocks[a] = newblock;
	}

	
	newblock = ablocks[--a];
	if (unlikely(newblock == 0)) {
		EXT4_ERROR_INODE(inode, "newblock == 0!");
		err = -EIO;
		goto cleanup;
	}
	bh = sb_getblk(inode->i_sb, newblock);
	if (!bh) {
		err = -EIO;
		goto cleanup;
	}
	lock_buffer(bh);

	err = ext4_journal_get_create_access(handle, bh);
	if (err)
		goto cleanup;

	neh = ext_block_hdr(bh);
	neh->eh_entries = 0;
	neh->eh_max = cpu_to_le16(ext4_ext_space_block(inode, 0));
	neh->eh_magic = EXT4_EXT_MAGIC;
	neh->eh_depth = 0;

	
	if (unlikely(path[depth].p_hdr->eh_entries !=
		     path[depth].p_hdr->eh_max)) {
		EXT4_ERROR_INODE(inode, "eh_entries %d != eh_max %d!",
				 path[depth].p_hdr->eh_entries,
				 path[depth].p_hdr->eh_max);
		err = -EIO;
		goto cleanup;
	}
	
	m = EXT_MAX_EXTENT(path[depth].p_hdr) - path[depth].p_ext++;
	ext4_ext_show_move(inode, path, newblock, depth);
	if (m) {
		struct ext4_extent *ex;
		ex = EXT_FIRST_EXTENT(neh);
		memmove(ex, path[depth].p_ext, sizeof(struct ext4_extent) * m);
		le16_add_cpu(&neh->eh_entries, m);
	}

	set_buffer_uptodate(bh);
	unlock_buffer(bh);

	err = ext4_handle_dirty_metadata(handle, inode, bh);
	if (err)
		goto cleanup;
	brelse(bh);
	bh = NULL;

	
	if (m) {
		err = ext4_ext_get_access(handle, inode, path + depth);
		if (err)
			goto cleanup;
		le16_add_cpu(&path[depth].p_hdr->eh_entries, -m);
		err = ext4_ext_dirty(handle, inode, path + depth);
		if (err)
			goto cleanup;

	}

	
	k = depth - at - 1;
	if (unlikely(k < 0)) {
		EXT4_ERROR_INODE(inode, "k %d < 0!", k);
		err = -EIO;
		goto cleanup;
	}
	if (k)
		ext_debug("create %d intermediate indices\n", k);
	
	
	i = depth - 1;
	while (k--) {
		oldblock = newblock;
		newblock = ablocks[--a];
		bh = sb_getblk(inode->i_sb, newblock);
		if (!bh) {
			err = -EIO;
			goto cleanup;
		}
		lock_buffer(bh);

		err = ext4_journal_get_create_access(handle, bh);
		if (err)
			goto cleanup;

		neh = ext_block_hdr(bh);
		neh->eh_entries = cpu_to_le16(1);
		neh->eh_magic = EXT4_EXT_MAGIC;
		neh->eh_max = cpu_to_le16(ext4_ext_space_block_idx(inode, 0));
		neh->eh_depth = cpu_to_le16(depth - i);
		fidx = EXT_FIRST_INDEX(neh);
		fidx->ei_block = border;
		ext4_idx_store_pblock(fidx, oldblock);

		ext_debug("int.index at %d (block %llu): %u -> %llu\n",
				i, newblock, le32_to_cpu(border), oldblock);

		
		if (unlikely(EXT_MAX_INDEX(path[i].p_hdr) !=
					EXT_LAST_INDEX(path[i].p_hdr))) {
			EXT4_ERROR_INODE(inode,
					 "EXT_MAX_INDEX != EXT_LAST_INDEX ee_block %d!",
					 le32_to_cpu(path[i].p_ext->ee_block));
			err = -EIO;
			goto cleanup;
		}
		
		m = EXT_MAX_INDEX(path[i].p_hdr) - path[i].p_idx++;
		ext_debug("cur 0x%p, last 0x%p\n", path[i].p_idx,
				EXT_MAX_INDEX(path[i].p_hdr));
		ext4_ext_show_move(inode, path, newblock, i);
		if (m) {
			memmove(++fidx, path[i].p_idx,
				sizeof(struct ext4_extent_idx) * m);
			le16_add_cpu(&neh->eh_entries, m);
		}
		set_buffer_uptodate(bh);
		unlock_buffer(bh);

		err = ext4_handle_dirty_metadata(handle, inode, bh);
		if (err)
			goto cleanup;
		brelse(bh);
		bh = NULL;

		
		if (m) {
			err = ext4_ext_get_access(handle, inode, path + i);
			if (err)
				goto cleanup;
			le16_add_cpu(&path[i].p_hdr->eh_entries, -m);
			err = ext4_ext_dirty(handle, inode, path + i);
			if (err)
				goto cleanup;
		}

		i--;
	}

	
	err = ext4_ext_insert_index(handle, inode, path + at,
				    le32_to_cpu(border), newblock);

cleanup:
	if (bh) {
		if (buffer_locked(bh))
			unlock_buffer(bh);
		brelse(bh);
	}

	if (err) {
		
		for (i = 0; i < depth; i++) {
			if (!ablocks[i])
				continue;
			ext4_free_blocks(handle, inode, NULL, ablocks[i], 1,
					 EXT4_FREE_BLOCKS_METADATA);
		}
	}
	kfree(ablocks);

	return err;
}

static int ext4_ext_grow_indepth(handle_t *handle, struct inode *inode,
				 unsigned int flags,
				 struct ext4_extent *newext)
{
	struct ext4_extent_header *neh;
	struct buffer_head *bh;
	ext4_fsblk_t newblock;
	int err = 0;

	newblock = ext4_ext_new_meta_block(handle, inode, NULL,
		newext, &err, flags);
	if (newblock == 0)
		return err;

	bh = sb_getblk(inode->i_sb, newblock);
	if (!bh) {
		err = -EIO;
		ext4_std_error(inode->i_sb, err);
		return err;
	}
	lock_buffer(bh);

	err = ext4_journal_get_create_access(handle, bh);
	if (err) {
		unlock_buffer(bh);
		goto out;
	}

	
	memmove(bh->b_data, EXT4_I(inode)->i_data,
		sizeof(EXT4_I(inode)->i_data));

	
	neh = ext_block_hdr(bh);
	if (ext_depth(inode))
		neh->eh_max = cpu_to_le16(ext4_ext_space_block_idx(inode, 0));
	else
		neh->eh_max = cpu_to_le16(ext4_ext_space_block(inode, 0));
	neh->eh_magic = EXT4_EXT_MAGIC;
	set_buffer_uptodate(bh);
	unlock_buffer(bh);

	err = ext4_handle_dirty_metadata(handle, inode, bh);
	if (err)
		goto out;

	
	neh = ext_inode_hdr(inode);
	neh->eh_entries = cpu_to_le16(1);
	ext4_idx_store_pblock(EXT_FIRST_INDEX(neh), newblock);
	if (neh->eh_depth == 0) {
		
		neh->eh_max = cpu_to_le16(ext4_ext_space_root_idx(inode, 0));
		EXT_FIRST_INDEX(neh)->ei_block =
			EXT_FIRST_EXTENT(neh)->ee_block;
	}
	ext_debug("new root: num %d(%d), lblock %d, ptr %llu\n",
		  le16_to_cpu(neh->eh_entries), le16_to_cpu(neh->eh_max),
		  le32_to_cpu(EXT_FIRST_INDEX(neh)->ei_block),
		  ext4_idx_pblock(EXT_FIRST_INDEX(neh)));

	neh->eh_depth = cpu_to_le16(le16_to_cpu(neh->eh_depth) + 1);
	ext4_mark_inode_dirty(handle, inode);
out:
	brelse(bh);

	return err;
}

static int ext4_ext_create_new_leaf(handle_t *handle, struct inode *inode,
				    unsigned int flags,
				    struct ext4_ext_path *path,
				    struct ext4_extent *newext)
{
	struct ext4_ext_path *curp;
	int depth, i, err = 0;

repeat:
	i = depth = ext_depth(inode);

	
	curp = path + depth;
	while (i > 0 && !EXT_HAS_FREE_INDEX(curp)) {
		i--;
		curp--;
	}

	if (EXT_HAS_FREE_INDEX(curp)) {
		err = ext4_ext_split(handle, inode, flags, path, newext, i);
		if (err)
			goto out;

		
		ext4_ext_drop_refs(path);
		path = ext4_ext_find_extent(inode,
				    (ext4_lblk_t)le32_to_cpu(newext->ee_block),
				    path);
		if (IS_ERR(path))
			err = PTR_ERR(path);
	} else {
		
		err = ext4_ext_grow_indepth(handle, inode, flags, newext);
		if (err)
			goto out;

		
		ext4_ext_drop_refs(path);
		path = ext4_ext_find_extent(inode,
				   (ext4_lblk_t)le32_to_cpu(newext->ee_block),
				    path);
		if (IS_ERR(path)) {
			err = PTR_ERR(path);
			goto out;
		}

		depth = ext_depth(inode);
		if (path[depth].p_hdr->eh_entries == path[depth].p_hdr->eh_max) {
			
			goto repeat;
		}
	}

out:
	return err;
}

static int ext4_ext_search_left(struct inode *inode,
				struct ext4_ext_path *path,
				ext4_lblk_t *logical, ext4_fsblk_t *phys)
{
	struct ext4_extent_idx *ix;
	struct ext4_extent *ex;
	int depth, ee_len;

	if (unlikely(path == NULL)) {
		EXT4_ERROR_INODE(inode, "path == NULL *logical %d!", *logical);
		return -EIO;
	}
	depth = path->p_depth;
	*phys = 0;

	if (depth == 0 && path->p_ext == NULL)
		return 0;


	ex = path[depth].p_ext;
	ee_len = ext4_ext_get_actual_len(ex);
	if (*logical < le32_to_cpu(ex->ee_block)) {
		if (unlikely(EXT_FIRST_EXTENT(path[depth].p_hdr) != ex)) {
			EXT4_ERROR_INODE(inode,
					 "EXT_FIRST_EXTENT != ex *logical %d ee_block %d!",
					 *logical, le32_to_cpu(ex->ee_block));
			return -EIO;
		}
		while (--depth >= 0) {
			ix = path[depth].p_idx;
			if (unlikely(ix != EXT_FIRST_INDEX(path[depth].p_hdr))) {
				EXT4_ERROR_INODE(inode,
				  "ix (%d) != EXT_FIRST_INDEX (%d) (depth %d)!",
				  ix != NULL ? le32_to_cpu(ix->ei_block) : 0,
				  EXT_FIRST_INDEX(path[depth].p_hdr) != NULL ?
		le32_to_cpu(EXT_FIRST_INDEX(path[depth].p_hdr)->ei_block) : 0,
				  depth);
				return -EIO;
			}
		}
		return 0;
	}

	if (unlikely(*logical < (le32_to_cpu(ex->ee_block) + ee_len))) {
		EXT4_ERROR_INODE(inode,
				 "logical %d < ee_block %d + ee_len %d!",
				 *logical, le32_to_cpu(ex->ee_block), ee_len);
		return -EIO;
	}

	*logical = le32_to_cpu(ex->ee_block) + ee_len - 1;
	*phys = ext4_ext_pblock(ex) + ee_len - 1;
	return 0;
}

static int ext4_ext_search_right(struct inode *inode,
				 struct ext4_ext_path *path,
				 ext4_lblk_t *logical, ext4_fsblk_t *phys,
				 struct ext4_extent **ret_ex)
{
	struct buffer_head *bh = NULL;
	struct ext4_extent_header *eh;
	struct ext4_extent_idx *ix;
	struct ext4_extent *ex;
	ext4_fsblk_t block;
	int depth;	
	int ee_len;

	if (unlikely(path == NULL)) {
		EXT4_ERROR_INODE(inode, "path == NULL *logical %d!", *logical);
		return -EIO;
	}
	depth = path->p_depth;
	*phys = 0;

	if (depth == 0 && path->p_ext == NULL)
		return 0;


	ex = path[depth].p_ext;
	ee_len = ext4_ext_get_actual_len(ex);
	if (*logical < le32_to_cpu(ex->ee_block)) {
		if (unlikely(EXT_FIRST_EXTENT(path[depth].p_hdr) != ex)) {
			EXT4_ERROR_INODE(inode,
					 "first_extent(path[%d].p_hdr) != ex",
					 depth);
			return -EIO;
		}
		while (--depth >= 0) {
			ix = path[depth].p_idx;
			if (unlikely(ix != EXT_FIRST_INDEX(path[depth].p_hdr))) {
				EXT4_ERROR_INODE(inode,
						 "ix != EXT_FIRST_INDEX *logical %d!",
						 *logical);
				return -EIO;
			}
		}
		goto found_extent;
	}

	if (unlikely(*logical < (le32_to_cpu(ex->ee_block) + ee_len))) {
		EXT4_ERROR_INODE(inode,
				 "logical %d < ee_block %d + ee_len %d!",
				 *logical, le32_to_cpu(ex->ee_block), ee_len);
		return -EIO;
	}

	if (ex != EXT_LAST_EXTENT(path[depth].p_hdr)) {
		
		ex++;
		goto found_extent;
	}

	
	while (--depth >= 0) {
		ix = path[depth].p_idx;
		if (ix != EXT_LAST_INDEX(path[depth].p_hdr))
			goto got_index;
	}

	
	return 0;

got_index:
	ix++;
	block = ext4_idx_pblock(ix);
	while (++depth < path->p_depth) {
		bh = sb_bread(inode->i_sb, block);
		if (bh == NULL)
			return -EIO;
		eh = ext_block_hdr(bh);
		
		if (ext4_ext_check(inode, eh, path->p_depth - depth)) {
			put_bh(bh);
			return -EIO;
		}
		ix = EXT_FIRST_INDEX(eh);
		block = ext4_idx_pblock(ix);
		put_bh(bh);
	}

	bh = sb_bread(inode->i_sb, block);
	if (bh == NULL)
		return -EIO;
	eh = ext_block_hdr(bh);
	if (ext4_ext_check(inode, eh, path->p_depth - depth)) {
		put_bh(bh);
		return -EIO;
	}
	ex = EXT_FIRST_EXTENT(eh);
found_extent:
	*logical = le32_to_cpu(ex->ee_block);
	*phys = ext4_ext_pblock(ex);
	*ret_ex = ex;
	if (bh)
		put_bh(bh);
	return 0;
}

static ext4_lblk_t
ext4_ext_next_allocated_block(struct ext4_ext_path *path)
{
	int depth;

	BUG_ON(path == NULL);
	depth = path->p_depth;

	if (depth == 0 && path->p_ext == NULL)
		return EXT_MAX_BLOCKS;

	while (depth >= 0) {
		if (depth == path->p_depth) {
			
			if (path[depth].p_ext &&
				path[depth].p_ext !=
					EXT_LAST_EXTENT(path[depth].p_hdr))
			  return le32_to_cpu(path[depth].p_ext[1].ee_block);
		} else {
			
			if (path[depth].p_idx !=
					EXT_LAST_INDEX(path[depth].p_hdr))
			  return le32_to_cpu(path[depth].p_idx[1].ei_block);
		}
		depth--;
	}

	return EXT_MAX_BLOCKS;
}

static ext4_lblk_t ext4_ext_next_leaf_block(struct ext4_ext_path *path)
{
	int depth;

	BUG_ON(path == NULL);
	depth = path->p_depth;

	
	if (depth == 0)
		return EXT_MAX_BLOCKS;

	
	depth--;

	while (depth >= 0) {
		if (path[depth].p_idx !=
				EXT_LAST_INDEX(path[depth].p_hdr))
			return (ext4_lblk_t)
				le32_to_cpu(path[depth].p_idx[1].ei_block);
		depth--;
	}

	return EXT_MAX_BLOCKS;
}

static int ext4_ext_correct_indexes(handle_t *handle, struct inode *inode,
				struct ext4_ext_path *path)
{
	struct ext4_extent_header *eh;
	int depth = ext_depth(inode);
	struct ext4_extent *ex;
	__le32 border;
	int k, err = 0;

	eh = path[depth].p_hdr;
	ex = path[depth].p_ext;

	if (unlikely(ex == NULL || eh == NULL)) {
		EXT4_ERROR_INODE(inode,
				 "ex %p == NULL or eh %p == NULL", ex, eh);
		return -EIO;
	}

	if (depth == 0) {
		
		return 0;
	}

	if (ex != EXT_FIRST_EXTENT(eh)) {
		
		return 0;
	}

	k = depth - 1;
	border = path[depth].p_ext->ee_block;
	err = ext4_ext_get_access(handle, inode, path + k);
	if (err)
		return err;
	path[k].p_idx->ei_block = border;
	err = ext4_ext_dirty(handle, inode, path + k);
	if (err)
		return err;

	while (k--) {
		
		if (path[k+1].p_idx != EXT_FIRST_INDEX(path[k+1].p_hdr))
			break;
		err = ext4_ext_get_access(handle, inode, path + k);
		if (err)
			break;
		path[k].p_idx->ei_block = border;
		err = ext4_ext_dirty(handle, inode, path + k);
		if (err)
			break;
	}

	return err;
}

int
ext4_can_extents_be_merged(struct inode *inode, struct ext4_extent *ex1,
				struct ext4_extent *ex2)
{
	unsigned short ext1_ee_len, ext2_ee_len, max_len;

	if (ext4_ext_is_uninitialized(ex1) ^ ext4_ext_is_uninitialized(ex2))
		return 0;

	if (ext4_ext_is_uninitialized(ex1))
		max_len = EXT_UNINIT_MAX_LEN;
	else
		max_len = EXT_INIT_MAX_LEN;

	ext1_ee_len = ext4_ext_get_actual_len(ex1);
	ext2_ee_len = ext4_ext_get_actual_len(ex2);

	if (le32_to_cpu(ex1->ee_block) + ext1_ee_len !=
			le32_to_cpu(ex2->ee_block))
		return 0;

	if (ext1_ee_len + ext2_ee_len > max_len)
		return 0;
#ifdef AGGRESSIVE_TEST
	if (ext1_ee_len >= 4)
		return 0;
#endif

	if (ext4_ext_pblock(ex1) + ext1_ee_len == ext4_ext_pblock(ex2))
		return 1;
	return 0;
}

static int ext4_ext_try_to_merge_right(struct inode *inode,
				 struct ext4_ext_path *path,
				 struct ext4_extent *ex)
{
	struct ext4_extent_header *eh;
	unsigned int depth, len;
	int merge_done = 0;
	int uninitialized = 0;

	depth = ext_depth(inode);
	BUG_ON(path[depth].p_hdr == NULL);
	eh = path[depth].p_hdr;

	while (ex < EXT_LAST_EXTENT(eh)) {
		if (!ext4_can_extents_be_merged(inode, ex, ex + 1))
			break;
		
		if (ext4_ext_is_uninitialized(ex))
			uninitialized = 1;
		ex->ee_len = cpu_to_le16(ext4_ext_get_actual_len(ex)
				+ ext4_ext_get_actual_len(ex + 1));
		if (uninitialized)
			ext4_ext_mark_uninitialized(ex);

		if (ex + 1 < EXT_LAST_EXTENT(eh)) {
			len = (EXT_LAST_EXTENT(eh) - ex - 1)
				* sizeof(struct ext4_extent);
			memmove(ex + 1, ex + 2, len);
		}
		le16_add_cpu(&eh->eh_entries, -1);
		merge_done = 1;
		WARN_ON(eh->eh_entries == 0);
		if (!eh->eh_entries)
			EXT4_ERROR_INODE(inode, "eh->eh_entries = 0!");
	}

	return merge_done;
}

static int ext4_ext_try_to_merge(struct inode *inode,
				  struct ext4_ext_path *path,
				  struct ext4_extent *ex) {
	struct ext4_extent_header *eh;
	unsigned int depth;
	int merge_done = 0;
	int ret = 0;

	depth = ext_depth(inode);
	BUG_ON(path[depth].p_hdr == NULL);
	eh = path[depth].p_hdr;

	if (ex > EXT_FIRST_EXTENT(eh))
		merge_done = ext4_ext_try_to_merge_right(inode, path, ex - 1);

	if (!merge_done)
		ret = ext4_ext_try_to_merge_right(inode, path, ex);

	return ret;
}

static unsigned int ext4_ext_check_overlap(struct ext4_sb_info *sbi,
					   struct inode *inode,
					   struct ext4_extent *newext,
					   struct ext4_ext_path *path)
{
	ext4_lblk_t b1, b2;
	unsigned int depth, len1;
	unsigned int ret = 0;

	b1 = le32_to_cpu(newext->ee_block);
	len1 = ext4_ext_get_actual_len(newext);
	depth = ext_depth(inode);
	if (!path[depth].p_ext)
		goto out;
	b2 = le32_to_cpu(path[depth].p_ext->ee_block);
	b2 &= ~(sbi->s_cluster_ratio - 1);

	if (b2 < b1) {
		b2 = ext4_ext_next_allocated_block(path);
		if (b2 == EXT_MAX_BLOCKS)
			goto out;
		b2 &= ~(sbi->s_cluster_ratio - 1);
	}

	
	if (b1 + len1 < b1) {
		len1 = EXT_MAX_BLOCKS - b1;
		newext->ee_len = cpu_to_le16(len1);
		ret = 1;
	}

	
	if (b1 + len1 > b2) {
		newext->ee_len = cpu_to_le16(b2 - b1);
		ret = 1;
	}
out:
	return ret;
}

int ext4_ext_insert_extent(handle_t *handle, struct inode *inode,
				struct ext4_ext_path *path,
				struct ext4_extent *newext, int flag)
{
	struct ext4_extent_header *eh;
	struct ext4_extent *ex, *fex;
	struct ext4_extent *nearex; 
	struct ext4_ext_path *npath = NULL;
	int depth, len, err;
	ext4_lblk_t next;
	unsigned uninitialized = 0;
	int flags = 0;

	if (unlikely(ext4_ext_get_actual_len(newext) == 0)) {
		EXT4_ERROR_INODE(inode, "ext4_ext_get_actual_len(newext) == 0");
		return -EIO;
	}
	depth = ext_depth(inode);
	ex = path[depth].p_ext;
	if (unlikely(path[depth].p_hdr == NULL)) {
		EXT4_ERROR_INODE(inode, "path[%d].p_hdr == NULL", depth);
		return -EIO;
	}

	
	if (ex && !(flag & EXT4_GET_BLOCKS_PRE_IO)
		&& ext4_can_extents_be_merged(inode, ex, newext)) {
		ext_debug("append [%d]%d block to %u:[%d]%d (from %llu)\n",
			  ext4_ext_is_uninitialized(newext),
			  ext4_ext_get_actual_len(newext),
			  le32_to_cpu(ex->ee_block),
			  ext4_ext_is_uninitialized(ex),
			  ext4_ext_get_actual_len(ex),
			  ext4_ext_pblock(ex));
		err = ext4_ext_get_access(handle, inode, path + depth);
		if (err)
			return err;

		if (ext4_ext_is_uninitialized(ex))
			uninitialized = 1;
		ex->ee_len = cpu_to_le16(ext4_ext_get_actual_len(ex)
					+ ext4_ext_get_actual_len(newext));
		if (uninitialized)
			ext4_ext_mark_uninitialized(ex);
		eh = path[depth].p_hdr;
		nearex = ex;
		goto merge;
	}

	depth = ext_depth(inode);
	eh = path[depth].p_hdr;
	if (le16_to_cpu(eh->eh_entries) < le16_to_cpu(eh->eh_max))
		goto has_space;

	
	fex = EXT_LAST_EXTENT(eh);
	next = EXT_MAX_BLOCKS;
	if (le32_to_cpu(newext->ee_block) > le32_to_cpu(fex->ee_block))
		next = ext4_ext_next_leaf_block(path);
	if (next != EXT_MAX_BLOCKS) {
		ext_debug("next leaf block - %u\n", next);
		BUG_ON(npath != NULL);
		npath = ext4_ext_find_extent(inode, next, NULL);
		if (IS_ERR(npath))
			return PTR_ERR(npath);
		BUG_ON(npath->p_depth != path->p_depth);
		eh = npath[depth].p_hdr;
		if (le16_to_cpu(eh->eh_entries) < le16_to_cpu(eh->eh_max)) {
			ext_debug("next leaf isn't full(%d)\n",
				  le16_to_cpu(eh->eh_entries));
			path = npath;
			goto has_space;
		}
		ext_debug("next leaf has no free space(%d,%d)\n",
			  le16_to_cpu(eh->eh_entries), le16_to_cpu(eh->eh_max));
	}

	if (flag & EXT4_GET_BLOCKS_PUNCH_OUT_EXT)
		flags = EXT4_MB_USE_ROOT_BLOCKS;
	err = ext4_ext_create_new_leaf(handle, inode, flags, path, newext);
	if (err)
		goto cleanup;
	depth = ext_depth(inode);
	eh = path[depth].p_hdr;

has_space:
	nearex = path[depth].p_ext;

	err = ext4_ext_get_access(handle, inode, path + depth);
	if (err)
		goto cleanup;

	if (!nearex) {
		
		ext_debug("first extent in the leaf: %u:%llu:[%d]%d\n",
				le32_to_cpu(newext->ee_block),
				ext4_ext_pblock(newext),
				ext4_ext_is_uninitialized(newext),
				ext4_ext_get_actual_len(newext));
		nearex = EXT_FIRST_EXTENT(eh);
	} else {
		if (le32_to_cpu(newext->ee_block)
			   > le32_to_cpu(nearex->ee_block)) {
			
			ext_debug("insert %u:%llu:[%d]%d before: "
					"nearest %p\n",
					le32_to_cpu(newext->ee_block),
					ext4_ext_pblock(newext),
					ext4_ext_is_uninitialized(newext),
					ext4_ext_get_actual_len(newext),
					nearex);
			nearex++;
		} else {
			
			BUG_ON(newext->ee_block == nearex->ee_block);
			ext_debug("insert %u:%llu:[%d]%d after: "
					"nearest %p\n",
					le32_to_cpu(newext->ee_block),
					ext4_ext_pblock(newext),
					ext4_ext_is_uninitialized(newext),
					ext4_ext_get_actual_len(newext),
					nearex);
		}
		len = EXT_LAST_EXTENT(eh) - nearex + 1;
		if (len > 0) {
			ext_debug("insert %u:%llu:[%d]%d: "
					"move %d extents from 0x%p to 0x%p\n",
					le32_to_cpu(newext->ee_block),
					ext4_ext_pblock(newext),
					ext4_ext_is_uninitialized(newext),
					ext4_ext_get_actual_len(newext),
					len, nearex, nearex + 1);
			memmove(nearex + 1, nearex,
				len * sizeof(struct ext4_extent));
		}
	}

	le16_add_cpu(&eh->eh_entries, 1);
	path[depth].p_ext = nearex;
	nearex->ee_block = newext->ee_block;
	ext4_ext_store_pblock(nearex, ext4_ext_pblock(newext));
	nearex->ee_len = newext->ee_len;

merge:
	
	if (!(flag & EXT4_GET_BLOCKS_PRE_IO))
		ext4_ext_try_to_merge(inode, path, nearex);

	

	
	err = ext4_ext_correct_indexes(handle, inode, path);
	if (err)
		goto cleanup;

	err = ext4_ext_dirty(handle, inode, path + depth);

cleanup:
	if (npath) {
		ext4_ext_drop_refs(npath);
		kfree(npath);
	}
	ext4_ext_invalidate_cache(inode);
	return err;
}

static int ext4_ext_walk_space(struct inode *inode, ext4_lblk_t block,
			       ext4_lblk_t num, ext_prepare_callback func,
			       void *cbdata)
{
	struct ext4_ext_path *path = NULL;
	struct ext4_ext_cache cbex;
	struct ext4_extent *ex;
	ext4_lblk_t next, start = 0, end = 0;
	ext4_lblk_t last = block + num;
	int depth, exists, err = 0;

	BUG_ON(func == NULL);
	BUG_ON(inode == NULL);

	while (block < last && block != EXT_MAX_BLOCKS) {
		num = last - block;
		
		down_read(&EXT4_I(inode)->i_data_sem);
		path = ext4_ext_find_extent(inode, block, path);
		up_read(&EXT4_I(inode)->i_data_sem);
		if (IS_ERR(path)) {
			err = PTR_ERR(path);
			path = NULL;
			break;
		}

		depth = ext_depth(inode);
		if (unlikely(path[depth].p_hdr == NULL)) {
			EXT4_ERROR_INODE(inode, "path[%d].p_hdr == NULL", depth);
			err = -EIO;
			break;
		}
		ex = path[depth].p_ext;
		next = ext4_ext_next_allocated_block(path);

		exists = 0;
		if (!ex) {
			start = block;
			end = block + num;
		} else if (le32_to_cpu(ex->ee_block) > block) {
			
			start = block;
			end = le32_to_cpu(ex->ee_block);
			if (block + num < end)
				end = block + num;
		} else if (block >= le32_to_cpu(ex->ee_block)
					+ ext4_ext_get_actual_len(ex)) {
			
			start = block;
			end = block + num;
			if (end >= next)
				end = next;
		} else if (block >= le32_to_cpu(ex->ee_block)) {
			start = block;
			end = le32_to_cpu(ex->ee_block)
				+ ext4_ext_get_actual_len(ex);
			if (block + num < end)
				end = block + num;
			exists = 1;
		} else {
			BUG();
		}
		BUG_ON(end <= start);

		if (!exists) {
			cbex.ec_block = start;
			cbex.ec_len = end - start;
			cbex.ec_start = 0;
		} else {
			cbex.ec_block = le32_to_cpu(ex->ee_block);
			cbex.ec_len = ext4_ext_get_actual_len(ex);
			cbex.ec_start = ext4_ext_pblock(ex);
		}

		if (unlikely(cbex.ec_len == 0)) {
			EXT4_ERROR_INODE(inode, "cbex.ec_len == 0");
			err = -EIO;
			break;
		}
		err = func(inode, next, &cbex, ex, cbdata);
		ext4_ext_drop_refs(path);

		if (err < 0)
			break;

		if (err == EXT_REPEAT)
			continue;
		else if (err == EXT_BREAK) {
			err = 0;
			break;
		}

		if (ext_depth(inode) != depth) {
			
			kfree(path);
			path = NULL;
		}

		block = cbex.ec_block + cbex.ec_len;
	}

	if (path) {
		ext4_ext_drop_refs(path);
		kfree(path);
	}

	return err;
}

static void
ext4_ext_put_in_cache(struct inode *inode, ext4_lblk_t block,
			__u32 len, ext4_fsblk_t start)
{
	struct ext4_ext_cache *cex;
	WARN_ON(len == 0);
	if (len == 0) {
		EXT4_ERROR_INODE(inode, "extent.ee_len = 0");
		return;
	}
	spin_lock(&EXT4_I(inode)->i_block_reservation_lock);
	trace_ext4_ext_put_in_cache(inode, block, len, start);
	cex = &EXT4_I(inode)->i_cached_extent;
	cex->ec_block = block;
	cex->ec_len = len;
	cex->ec_start = start;
	spin_unlock(&EXT4_I(inode)->i_block_reservation_lock);
}

static void
ext4_ext_put_gap_in_cache(struct inode *inode, struct ext4_ext_path *path,
				ext4_lblk_t block)
{
	int depth = ext_depth(inode);
	unsigned long len;
	ext4_lblk_t lblock;
	struct ext4_extent *ex;

	ex = path[depth].p_ext;
	if (ex == NULL) {
		
		lblock = 0;
		len = EXT_MAX_BLOCKS;
		ext_debug("cache gap(whole file):");
	} else if (block < le32_to_cpu(ex->ee_block)) {
		lblock = block;
		len = le32_to_cpu(ex->ee_block) - block;
		ext_debug("cache gap(before): %u [%u:%u]",
				block,
				le32_to_cpu(ex->ee_block),
				 ext4_ext_get_actual_len(ex));
	} else if (block >= le32_to_cpu(ex->ee_block)
			+ ext4_ext_get_actual_len(ex)) {
		ext4_lblk_t next;
		lblock = le32_to_cpu(ex->ee_block)
			+ ext4_ext_get_actual_len(ex);

		next = ext4_ext_next_allocated_block(path);
		ext_debug("cache gap(after): [%u:%u] %u",
				le32_to_cpu(ex->ee_block),
				ext4_ext_get_actual_len(ex),
				block);
		BUG_ON(next == lblock);
		len = next - lblock;
	} else {
		lblock = len = 0;
		BUG();
	}

	ext_debug(" -> %u:%lu\n", lblock, len);
	ext4_ext_put_in_cache(inode, lblock, len, 0);
}

static int ext4_ext_check_cache(struct inode *inode, ext4_lblk_t block,
	struct ext4_ext_cache *ex){
	struct ext4_ext_cache *cex;
	struct ext4_sb_info *sbi;
	int ret = 0;

	spin_lock(&EXT4_I(inode)->i_block_reservation_lock);
	cex = &EXT4_I(inode)->i_cached_extent;
	sbi = EXT4_SB(inode->i_sb);

	
	if (cex->ec_len == 0)
		goto errout;

	if (in_range(block, cex->ec_block, cex->ec_len)) {
		memcpy(ex, cex, sizeof(struct ext4_ext_cache));
		ext_debug("%u cached by %u:%u:%llu\n",
				block,
				cex->ec_block, cex->ec_len, cex->ec_start);
		ret = 1;
	}
errout:
	trace_ext4_ext_in_cache(inode, block, ret);
	spin_unlock(&EXT4_I(inode)->i_block_reservation_lock);
	return ret;
}

static int
ext4_ext_in_cache(struct inode *inode, ext4_lblk_t block,
			struct ext4_extent *ex)
{
	struct ext4_ext_cache cex = {0, 0, 0};
	int ret = 0;

	if (ext4_ext_check_cache(inode, block, &cex)) {
		ex->ee_block = cpu_to_le32(cex.ec_block);
		ext4_ext_store_pblock(ex, cex.ec_start);
		ex->ee_len = cpu_to_le16(cex.ec_len);
		ret = 1;
	}

	return ret;
}


static int ext4_ext_rm_idx(handle_t *handle, struct inode *inode,
			struct ext4_ext_path *path, int depth)
{
	int err;
	ext4_fsblk_t leaf;

	
	depth--;
	path = path + depth;
	leaf = ext4_idx_pblock(path->p_idx);
	if (unlikely(path->p_hdr->eh_entries == 0)) {
		EXT4_ERROR_INODE(inode, "path->p_hdr->eh_entries == 0");
		return -EIO;
	}
	err = ext4_ext_get_access(handle, inode, path);
	if (err)
		return err;

	if (path->p_idx != EXT_LAST_INDEX(path->p_hdr)) {
		int len = EXT_LAST_INDEX(path->p_hdr) - path->p_idx;
		len *= sizeof(struct ext4_extent_idx);
		memmove(path->p_idx, path->p_idx + 1, len);
	}

	le16_add_cpu(&path->p_hdr->eh_entries, -1);
	err = ext4_ext_dirty(handle, inode, path);
	if (err)
		return err;
	ext_debug("index is empty, remove it, free block %llu\n", leaf);
	trace_ext4_ext_rm_idx(inode, leaf);

	ext4_free_blocks(handle, inode, NULL, leaf, 1,
			 EXT4_FREE_BLOCKS_METADATA | EXT4_FREE_BLOCKS_FORGET);

	while (--depth >= 0) {
		if (path->p_idx != EXT_FIRST_INDEX(path->p_hdr))
			break;
		path--;
		err = ext4_ext_get_access(handle, inode, path);
		if (err)
			break;
		path->p_idx->ei_block = (path+1)->p_idx->ei_block;
		err = ext4_ext_dirty(handle, inode, path);
		if (err)
			break;
	}
	return err;
}

int ext4_ext_calc_credits_for_single_extent(struct inode *inode, int nrblocks,
						struct ext4_ext_path *path)
{
	if (path) {
		int depth = ext_depth(inode);
		int ret = 0;

		
		if (le16_to_cpu(path[depth].p_hdr->eh_entries)
				< le16_to_cpu(path[depth].p_hdr->eh_max)) {

			
			ret = 2 + EXT4_META_TRANS_BLOCKS(inode->i_sb);
			return ret;
		}
	}

	return ext4_chunk_trans_blocks(inode, nrblocks);
}

int ext4_ext_index_trans_blocks(struct inode *inode, int nrblocks, int chunk)
{
	int index;
	int depth = ext_depth(inode);

	if (chunk)
		index = depth * 2;
	else
		index = depth * 3;

	return index;
}

static int ext4_remove_blocks(handle_t *handle, struct inode *inode,
			      struct ext4_extent *ex,
			      ext4_fsblk_t *partial_cluster,
			      ext4_lblk_t from, ext4_lblk_t to)
{
	struct ext4_sb_info *sbi = EXT4_SB(inode->i_sb);
	unsigned short ee_len =  ext4_ext_get_actual_len(ex);
	ext4_fsblk_t pblk;
	int flags = EXT4_FREE_BLOCKS_FORGET;

	if (S_ISDIR(inode->i_mode) || S_ISLNK(inode->i_mode))
		flags |= EXT4_FREE_BLOCKS_METADATA;
	flags |= EXT4_FREE_BLOCKS_NOFREE_FIRST_CLUSTER;

	trace_ext4_remove_blocks(inode, ex, from, to, *partial_cluster);
	pblk = ext4_ext_pblock(ex) + ee_len - 1;
	if (*partial_cluster && (EXT4_B2C(sbi, pblk) != *partial_cluster)) {
		ext4_free_blocks(handle, inode, NULL,
				 EXT4_C2B(sbi, *partial_cluster),
				 sbi->s_cluster_ratio, flags);
		*partial_cluster = 0;
	}

#ifdef EXTENTS_STATS
	{
		struct ext4_sb_info *sbi = EXT4_SB(inode->i_sb);
		spin_lock(&sbi->s_ext_stats_lock);
		sbi->s_ext_blocks += ee_len;
		sbi->s_ext_extents++;
		if (ee_len < sbi->s_ext_min)
			sbi->s_ext_min = ee_len;
		if (ee_len > sbi->s_ext_max)
			sbi->s_ext_max = ee_len;
		if (ext_depth(inode) > sbi->s_depth_max)
			sbi->s_depth_max = ext_depth(inode);
		spin_unlock(&sbi->s_ext_stats_lock);
	}
#endif
	if (from >= le32_to_cpu(ex->ee_block)
	    && to == le32_to_cpu(ex->ee_block) + ee_len - 1) {
		
		ext4_lblk_t num;

		num = le32_to_cpu(ex->ee_block) + ee_len - from;
		pblk = ext4_ext_pblock(ex) + ee_len - num;
		ext_debug("free last %u blocks starting %llu\n", num, pblk);
		ext4_free_blocks(handle, inode, NULL, pblk, num, flags);
		if (pblk & (sbi->s_cluster_ratio - 1) &&
		    (ee_len == num))
			*partial_cluster = EXT4_B2C(sbi, pblk);
		else
			*partial_cluster = 0;
	} else if (from == le32_to_cpu(ex->ee_block)
		   && to <= le32_to_cpu(ex->ee_block) + ee_len - 1) {
		
		ext4_lblk_t num;
		ext4_fsblk_t start;

		num = to - from;
		start = ext4_ext_pblock(ex);

		ext_debug("free first %u blocks starting %llu\n", num, start);
		ext4_free_blocks(handle, inode, NULL, start, num, flags);

	} else {
		printk(KERN_INFO "strange request: removal(2) "
				"%u-%u from %u:%u\n",
				from, to, le32_to_cpu(ex->ee_block), ee_len);
	}
	return 0;
}


static int
ext4_ext_rm_leaf(handle_t *handle, struct inode *inode,
		 struct ext4_ext_path *path, ext4_fsblk_t *partial_cluster,
		 ext4_lblk_t start, ext4_lblk_t end)
{
	struct ext4_sb_info *sbi = EXT4_SB(inode->i_sb);
	int err = 0, correct_index = 0;
	int depth = ext_depth(inode), credits;
	struct ext4_extent_header *eh;
	ext4_lblk_t a, b;
	unsigned num;
	ext4_lblk_t ex_ee_block;
	unsigned short ex_ee_len;
	unsigned uninitialized = 0;
	struct ext4_extent *ex;

	
	ext_debug("truncate since %u in leaf to %u\n", start, end);
	if (!path[depth].p_hdr)
		path[depth].p_hdr = ext_block_hdr(path[depth].p_bh);
	eh = path[depth].p_hdr;
	if (unlikely(path[depth].p_hdr == NULL)) {
		EXT4_ERROR_INODE(inode, "path[%d].p_hdr == NULL", depth);
		return -EIO;
	}
	
	ex = EXT_LAST_EXTENT(eh);

	ex_ee_block = le32_to_cpu(ex->ee_block);
	ex_ee_len = ext4_ext_get_actual_len(ex);

	trace_ext4_ext_rm_leaf(inode, start, ex, *partial_cluster);

	while (ex >= EXT_FIRST_EXTENT(eh) &&
			ex_ee_block + ex_ee_len > start) {

		if (ext4_ext_is_uninitialized(ex))
			uninitialized = 1;
		else
			uninitialized = 0;

		ext_debug("remove ext %u:[%d]%d\n", ex_ee_block,
			 uninitialized, ex_ee_len);
		path[depth].p_ext = ex;

		a = ex_ee_block > start ? ex_ee_block : start;
		b = ex_ee_block+ex_ee_len - 1 < end ?
			ex_ee_block+ex_ee_len - 1 : end;

		ext_debug("  border %u:%u\n", a, b);

		
		if (end < ex_ee_block) {
			ex--;
			ex_ee_block = le32_to_cpu(ex->ee_block);
			ex_ee_len = ext4_ext_get_actual_len(ex);
			continue;
		} else if (b != ex_ee_block + ex_ee_len - 1) {
			EXT4_ERROR_INODE(inode,
					 "can not handle truncate %u:%u "
					 "on extent %u:%u",
					 start, end, ex_ee_block,
					 ex_ee_block + ex_ee_len - 1);
			err = -EIO;
			goto out;
		} else if (a != ex_ee_block) {
			
			num = a - ex_ee_block;
		} else {
			
			num = 0;
		}
		credits = 7 + 2*(ex_ee_len/EXT4_BLOCKS_PER_GROUP(inode->i_sb));
		if (ex == EXT_FIRST_EXTENT(eh)) {
			correct_index = 1;
			credits += (ext_depth(inode)) + 1;
		}
		credits += EXT4_MAXQUOTAS_TRANS_BLOCKS(inode->i_sb);

		err = ext4_ext_truncate_extend_restart(handle, inode, credits);
		if (err)
			goto out;

		err = ext4_ext_get_access(handle, inode, path + depth);
		if (err)
			goto out;

		err = ext4_remove_blocks(handle, inode, ex, partial_cluster,
					 a, b);
		if (err)
			goto out;

		if (num == 0)
			
			ext4_ext_store_pblock(ex, 0);

		ex->ee_len = cpu_to_le16(num);
		if (uninitialized && num)
			ext4_ext_mark_uninitialized(ex);
		if (num == 0) {
			if (end != EXT_MAX_BLOCKS - 1) {
				memmove(ex, ex+1, (EXT_LAST_EXTENT(eh) - ex) *
					sizeof(struct ext4_extent));

				
				memset(EXT_LAST_EXTENT(eh), 0,
					sizeof(struct ext4_extent));
			}
			le16_add_cpu(&eh->eh_entries, -1);
		} else
			*partial_cluster = 0;

		err = ext4_ext_dirty(handle, inode, path + depth);
		if (err)
			goto out;

		ext_debug("new extent: %u:%u:%llu\n", ex_ee_block, num,
				ext4_ext_pblock(ex));
		ex--;
		ex_ee_block = le32_to_cpu(ex->ee_block);
		ex_ee_len = ext4_ext_get_actual_len(ex);
	}

	if (correct_index && eh->eh_entries)
		err = ext4_ext_correct_indexes(handle, inode, path);

	if (*partial_cluster && ex >= EXT_FIRST_EXTENT(eh) &&
	    (EXT4_B2C(sbi, ext4_ext_pblock(ex) + ex_ee_len - 1) !=
	     *partial_cluster)) {
		int flags = EXT4_FREE_BLOCKS_FORGET;

		if (S_ISDIR(inode->i_mode) || S_ISLNK(inode->i_mode))
			flags |= EXT4_FREE_BLOCKS_METADATA;

		ext4_free_blocks(handle, inode, NULL,
				 EXT4_C2B(sbi, *partial_cluster),
				 sbi->s_cluster_ratio, flags);
		*partial_cluster = 0;
	}

	if (err == 0 && eh->eh_entries == 0 && path[depth].p_bh != NULL)
		err = ext4_ext_rm_idx(handle, inode, path, depth);

out:
	return err;
}

static int
ext4_ext_more_to_rm(struct ext4_ext_path *path)
{
	BUG_ON(path->p_idx == NULL);

	if (path->p_idx < EXT_FIRST_INDEX(path->p_hdr))
		return 0;

	if (le16_to_cpu(path->p_hdr->eh_entries) == path->p_block)
		return 0;
	return 1;
}

static int ext4_ext_remove_space(struct inode *inode, ext4_lblk_t start,
				 ext4_lblk_t end)
{
	struct super_block *sb = inode->i_sb;
	int depth = ext_depth(inode);
	struct ext4_ext_path *path;
	ext4_fsblk_t partial_cluster = 0;
	handle_t *handle;
	int i, err;

	ext_debug("truncate since %u to %u\n", start, end);

	
	handle = ext4_journal_start(inode, depth + 1);
	if (IS_ERR(handle))
		return PTR_ERR(handle);

again:
	ext4_ext_invalidate_cache(inode);

	trace_ext4_ext_remove_space(inode, start, depth);

	if (end < EXT_MAX_BLOCKS - 1) {
		struct ext4_extent *ex;
		ext4_lblk_t ee_block;

		
		path = ext4_ext_find_extent(inode, end, NULL);
		if (IS_ERR(path)) {
			ext4_journal_stop(handle);
			return PTR_ERR(path);
		}
		depth = ext_depth(inode);
		ex = path[depth].p_ext;
		if (!ex)
			goto cont;

		ee_block = le32_to_cpu(ex->ee_block);

		if (end >= ee_block &&
		    end < ee_block + ext4_ext_get_actual_len(ex) - 1) {
			int split_flag = 0;

			if (ext4_ext_is_uninitialized(ex))
				split_flag = EXT4_EXT_MARK_UNINIT1 |
					     EXT4_EXT_MARK_UNINIT2;

			err = ext4_split_extent_at(handle, inode, path,
						end + 1, split_flag,
						EXT4_GET_BLOCKS_PRE_IO |
						EXT4_GET_BLOCKS_PUNCH_OUT_EXT);

			if (err < 0)
				goto out;
		}
		ext4_ext_drop_refs(path);
		kfree(path);
	}
cont:

	depth = ext_depth(inode);
	path = kzalloc(sizeof(struct ext4_ext_path) * (depth + 1), GFP_NOFS);
	if (path == NULL) {
		ext4_journal_stop(handle);
		return -ENOMEM;
	}
	path[0].p_depth = depth;
	path[0].p_hdr = ext_inode_hdr(inode);

	if (ext4_ext_check(inode, path[0].p_hdr, depth)) {
		err = -EIO;
		goto out;
	}
	i = err = 0;

	while (i >= 0 && err == 0) {
		if (i == depth) {
			
			err = ext4_ext_rm_leaf(handle, inode, path,
					       &partial_cluster, start,
					       end);
			
			brelse(path[i].p_bh);
			path[i].p_bh = NULL;
			i--;
			continue;
		}

		
		if (!path[i].p_hdr) {
			ext_debug("initialize header\n");
			path[i].p_hdr = ext_block_hdr(path[i].p_bh);
		}

		if (!path[i].p_idx) {
			
			path[i].p_idx = EXT_LAST_INDEX(path[i].p_hdr);
			path[i].p_block = le16_to_cpu(path[i].p_hdr->eh_entries)+1;
			ext_debug("init index ptr: hdr 0x%p, num %d\n",
				  path[i].p_hdr,
				  le16_to_cpu(path[i].p_hdr->eh_entries));
		} else {
			
			path[i].p_idx--;
		}

		ext_debug("level %d - index, first 0x%p, cur 0x%p\n",
				i, EXT_FIRST_INDEX(path[i].p_hdr),
				path[i].p_idx);
		if (ext4_ext_more_to_rm(path + i)) {
			struct buffer_head *bh;
			
			ext_debug("move to level %d (block %llu)\n",
				  i + 1, ext4_idx_pblock(path[i].p_idx));
			memset(path + i + 1, 0, sizeof(*path));
			bh = sb_bread(sb, ext4_idx_pblock(path[i].p_idx));
			if (!bh) {
				
				err = -EIO;
				break;
			}
			if (WARN_ON(i + 1 > depth)) {
				err = -EIO;
				break;
			}
			if (ext4_ext_check(inode, ext_block_hdr(bh),
							depth - i - 1)) {
				err = -EIO;
				break;
			}
			path[i + 1].p_bh = bh;

			path[i].p_block = le16_to_cpu(path[i].p_hdr->eh_entries);
			i++;
		} else {
			
			if (path[i].p_hdr->eh_entries == 0 && i > 0) {
				err = ext4_ext_rm_idx(handle, inode, path, i);
			}
			
			brelse(path[i].p_bh);
			path[i].p_bh = NULL;
			i--;
			ext_debug("return to level %d\n", i);
		}
	}

	trace_ext4_ext_remove_space_done(inode, start, depth, partial_cluster,
			path->p_hdr->eh_entries);

	if (partial_cluster && path->p_hdr->eh_entries == 0) {
		int flags = EXT4_FREE_BLOCKS_FORGET;

		if (S_ISDIR(inode->i_mode) || S_ISLNK(inode->i_mode))
			flags |= EXT4_FREE_BLOCKS_METADATA;

		ext4_free_blocks(handle, inode, NULL,
				 EXT4_C2B(EXT4_SB(sb), partial_cluster),
				 EXT4_SB(sb)->s_cluster_ratio, flags);
		partial_cluster = 0;
	}

	
	if (path->p_hdr->eh_entries == 0) {
		err = ext4_ext_get_access(handle, inode, path);
		if (err == 0) {
			ext_inode_hdr(inode)->eh_depth = 0;
			ext_inode_hdr(inode)->eh_max =
				cpu_to_le16(ext4_ext_space_root(inode, 0));
			err = ext4_ext_dirty(handle, inode, path);
		}
	}
out:
	ext4_ext_drop_refs(path);
	kfree(path);
	if (err == -EAGAIN)
		goto again;
	ext4_journal_stop(handle);

	return err;
}

void ext4_ext_init(struct super_block *sb)
{

	if (EXT4_HAS_INCOMPAT_FEATURE(sb, EXT4_FEATURE_INCOMPAT_EXTENTS)) {
#if defined(AGGRESSIVE_TEST) || defined(CHECK_BINSEARCH) || defined(EXTENTS_STATS)
		printk(KERN_INFO "EXT4-fs: file extents enabled"
#ifdef AGGRESSIVE_TEST
		       ", aggressive tests"
#endif
#ifdef CHECK_BINSEARCH
		       ", check binsearch"
#endif
#ifdef EXTENTS_STATS
		       ", stats"
#endif
		       "\n");
#endif
#ifdef EXTENTS_STATS
		spin_lock_init(&EXT4_SB(sb)->s_ext_stats_lock);
		EXT4_SB(sb)->s_ext_min = 1 << 30;
		EXT4_SB(sb)->s_ext_max = 0;
#endif
	}
}

void ext4_ext_release(struct super_block *sb)
{
	if (!EXT4_HAS_INCOMPAT_FEATURE(sb, EXT4_FEATURE_INCOMPAT_EXTENTS))
		return;

#ifdef EXTENTS_STATS
	if (EXT4_SB(sb)->s_ext_blocks && EXT4_SB(sb)->s_ext_extents) {
		struct ext4_sb_info *sbi = EXT4_SB(sb);
		printk(KERN_ERR "EXT4-fs: %lu blocks in %lu extents (%lu ave)\n",
			sbi->s_ext_blocks, sbi->s_ext_extents,
			sbi->s_ext_blocks / sbi->s_ext_extents);
		printk(KERN_ERR "EXT4-fs: extents: %lu min, %lu max, max depth %lu\n",
			sbi->s_ext_min, sbi->s_ext_max, sbi->s_depth_max);
	}
#endif
}

static int ext4_ext_zeroout(struct inode *inode, struct ext4_extent *ex)
{
	ext4_fsblk_t ee_pblock;
	unsigned int ee_len;
	int ret;

	ee_len    = ext4_ext_get_actual_len(ex);
	ee_pblock = ext4_ext_pblock(ex);

	ret = sb_issue_zeroout(inode->i_sb, ee_pblock, ee_len, GFP_NOFS);
	if (ret > 0)
		ret = 0;

	return ret;
}

static int ext4_split_extent_at(handle_t *handle,
			     struct inode *inode,
			     struct ext4_ext_path *path,
			     ext4_lblk_t split,
			     int split_flag,
			     int flags)
{
	ext4_fsblk_t newblock;
	ext4_lblk_t ee_block;
	struct ext4_extent *ex, newex, orig_ex;
	struct ext4_extent *ex2 = NULL;
	unsigned int ee_len, depth;
	int err = 0;

	ext_debug("ext4_split_extents_at: inode %lu, logical"
		"block %llu\n", inode->i_ino, (unsigned long long)split);

	ext4_ext_show_leaf(inode, path);

	depth = ext_depth(inode);
	ex = path[depth].p_ext;
	ee_block = le32_to_cpu(ex->ee_block);
	ee_len = ext4_ext_get_actual_len(ex);
	newblock = split - ee_block + ext4_ext_pblock(ex);

	BUG_ON(split < ee_block || split >= (ee_block + ee_len));

	err = ext4_ext_get_access(handle, inode, path + depth);
	if (err)
		goto out;

	if (split == ee_block) {
		if (split_flag & EXT4_EXT_MARK_UNINIT2)
			ext4_ext_mark_uninitialized(ex);
		else
			ext4_ext_mark_initialized(ex);

		if (!(flags & EXT4_GET_BLOCKS_PRE_IO))
			ext4_ext_try_to_merge(inode, path, ex);

		err = ext4_ext_dirty(handle, inode, path + depth);
		goto out;
	}

	
	memcpy(&orig_ex, ex, sizeof(orig_ex));
	ex->ee_len = cpu_to_le16(split - ee_block);
	if (split_flag & EXT4_EXT_MARK_UNINIT1)
		ext4_ext_mark_uninitialized(ex);

	err = ext4_ext_dirty(handle, inode, path + depth);
	if (err)
		goto fix_extent_len;

	ex2 = &newex;
	ex2->ee_block = cpu_to_le32(split);
	ex2->ee_len   = cpu_to_le16(ee_len - (split - ee_block));
	ext4_ext_store_pblock(ex2, newblock);
	if (split_flag & EXT4_EXT_MARK_UNINIT2)
		ext4_ext_mark_uninitialized(ex2);

	err = ext4_ext_insert_extent(handle, inode, path, &newex, flags);
	if (err == -ENOSPC && (EXT4_EXT_MAY_ZEROOUT & split_flag)) {
		err = ext4_ext_zeroout(inode, &orig_ex);
		if (err)
			goto fix_extent_len;
		
		ex->ee_len = cpu_to_le16(ee_len);
		ext4_ext_try_to_merge(inode, path, ex);
		err = ext4_ext_dirty(handle, inode, path + depth);
		goto out;
	} else if (err)
		goto fix_extent_len;

out:
	ext4_ext_show_leaf(inode, path);
	return err;

fix_extent_len:
	ex->ee_len = orig_ex.ee_len;
	ext4_ext_dirty(handle, inode, path + depth);
	return err;
}

static int ext4_split_extent(handle_t *handle,
			      struct inode *inode,
			      struct ext4_ext_path *path,
			      struct ext4_map_blocks *map,
			      int split_flag,
			      int flags)
{
	ext4_lblk_t ee_block;
	struct ext4_extent *ex;
	unsigned int ee_len, depth;
	int err = 0;
	int uninitialized;
	int split_flag1, flags1;

	depth = ext_depth(inode);
	ex = path[depth].p_ext;
	ee_block = le32_to_cpu(ex->ee_block);
	ee_len = ext4_ext_get_actual_len(ex);
	uninitialized = ext4_ext_is_uninitialized(ex);

	if (map->m_lblk + map->m_len < ee_block + ee_len) {
		split_flag1 = split_flag & EXT4_EXT_MAY_ZEROOUT ?
			      EXT4_EXT_MAY_ZEROOUT : 0;
		flags1 = flags | EXT4_GET_BLOCKS_PRE_IO;
		if (uninitialized)
			split_flag1 |= EXT4_EXT_MARK_UNINIT1 |
				       EXT4_EXT_MARK_UNINIT2;
		err = ext4_split_extent_at(handle, inode, path,
				map->m_lblk + map->m_len, split_flag1, flags1);
		if (err)
			goto out;
	}

	ext4_ext_drop_refs(path);
	path = ext4_ext_find_extent(inode, map->m_lblk, path);
	if (IS_ERR(path))
		return PTR_ERR(path);

	if (map->m_lblk >= ee_block) {
		split_flag1 = split_flag & EXT4_EXT_MAY_ZEROOUT ?
			      EXT4_EXT_MAY_ZEROOUT : 0;
		if (uninitialized)
			split_flag1 |= EXT4_EXT_MARK_UNINIT1;
		if (split_flag & EXT4_EXT_MARK_UNINIT2)
			split_flag1 |= EXT4_EXT_MARK_UNINIT2;
		err = ext4_split_extent_at(handle, inode, path,
				map->m_lblk, split_flag1, flags);
		if (err)
			goto out;
	}

	ext4_ext_show_leaf(inode, path);
out:
	return err ? err : map->m_len;
}

#define EXT4_EXT_ZERO_LEN 7
static int ext4_ext_convert_to_initialized(handle_t *handle,
					   struct inode *inode,
					   struct ext4_map_blocks *map,
					   struct ext4_ext_path *path)
{
	struct ext4_extent_header *eh;
	struct ext4_map_blocks split_map;
	struct ext4_extent zero_ex;
	struct ext4_extent *ex;
	ext4_lblk_t ee_block, eof_block;
	unsigned int ee_len, depth;
	int allocated;
	int err = 0;
	int split_flag = 0;

	ext_debug("ext4_ext_convert_to_initialized: inode %lu, logical"
		"block %llu, max_blocks %u\n", inode->i_ino,
		(unsigned long long)map->m_lblk, map->m_len);

	eof_block = (inode->i_size + inode->i_sb->s_blocksize - 1) >>
		inode->i_sb->s_blocksize_bits;
	if (eof_block < map->m_lblk + map->m_len)
		eof_block = map->m_lblk + map->m_len;

	depth = ext_depth(inode);
	eh = path[depth].p_hdr;
	ex = path[depth].p_ext;
	ee_block = le32_to_cpu(ex->ee_block);
	ee_len = ext4_ext_get_actual_len(ex);
	allocated = ee_len - (map->m_lblk - ee_block);

	trace_ext4_ext_convert_to_initialized_enter(inode, map, ex);

	
	BUG_ON(!ext4_ext_is_uninitialized(ex));
	BUG_ON(!in_range(map->m_lblk, ee_block, ee_len));

	if ((map->m_lblk == ee_block) &&	
		(map->m_len < ee_len) &&	
		(ex > EXT_FIRST_EXTENT(eh))) {	
		struct ext4_extent *prev_ex;
		ext4_lblk_t prev_lblk;
		ext4_fsblk_t prev_pblk, ee_pblk;
		unsigned int prev_len, write_len;

		prev_ex = ex - 1;
		prev_lblk = le32_to_cpu(prev_ex->ee_block);
		prev_len = ext4_ext_get_actual_len(prev_ex);
		prev_pblk = ext4_ext_pblock(prev_ex);
		ee_pblk = ext4_ext_pblock(ex);
		write_len = map->m_len;

		if ((!ext4_ext_is_uninitialized(prev_ex)) &&		
			((prev_lblk + prev_len) == ee_block) &&		
			((prev_pblk + prev_len) == ee_pblk) &&		
			(prev_len < (EXT_INIT_MAX_LEN - write_len))) {	
			err = ext4_ext_get_access(handle, inode, path + depth);
			if (err)
				goto out;

			trace_ext4_ext_convert_to_initialized_fastpath(inode,
				map, ex, prev_ex);

			
			ex->ee_block = cpu_to_le32(ee_block + write_len);
			ext4_ext_store_pblock(ex, ee_pblk + write_len);
			ex->ee_len = cpu_to_le16(ee_len - write_len);
			ext4_ext_mark_uninitialized(ex); 

			
			prev_ex->ee_len = cpu_to_le16(prev_len + write_len);

			
			ext4_ext_dirty(handle, inode, path + depth);

			
			path[depth].p_ext = prev_ex;

			
			allocated = write_len;
			goto out;
		}
	}

	WARN_ON(map->m_lblk < ee_block);
	split_flag |= ee_block + ee_len <= eof_block ? EXT4_EXT_MAY_ZEROOUT : 0;

	
	if (ee_len <= 2*EXT4_EXT_ZERO_LEN &&
	    (EXT4_EXT_MAY_ZEROOUT & split_flag)) {
		err = ext4_ext_zeroout(inode, ex);
		if (err)
			goto out;

		err = ext4_ext_get_access(handle, inode, path + depth);
		if (err)
			goto out;
		ext4_ext_mark_initialized(ex);
		ext4_ext_try_to_merge(inode, path, ex);
		err = ext4_ext_dirty(handle, inode, path + depth);
		goto out;
	}

	split_map.m_lblk = map->m_lblk;
	split_map.m_len = map->m_len;

	if (allocated > map->m_len) {
		if (allocated <= EXT4_EXT_ZERO_LEN &&
		    (EXT4_EXT_MAY_ZEROOUT & split_flag)) {
			
			zero_ex.ee_block =
					 cpu_to_le32(map->m_lblk);
			zero_ex.ee_len = cpu_to_le16(allocated);
			ext4_ext_store_pblock(&zero_ex,
				ext4_ext_pblock(ex) + map->m_lblk - ee_block);
			err = ext4_ext_zeroout(inode, &zero_ex);
			if (err)
				goto out;
			split_map.m_lblk = map->m_lblk;
			split_map.m_len = allocated;
		} else if ((map->m_lblk - ee_block + map->m_len <
			   EXT4_EXT_ZERO_LEN) &&
			   (EXT4_EXT_MAY_ZEROOUT & split_flag)) {
			
			if (map->m_lblk != ee_block) {
				zero_ex.ee_block = ex->ee_block;
				zero_ex.ee_len = cpu_to_le16(map->m_lblk -
							ee_block);
				ext4_ext_store_pblock(&zero_ex,
						      ext4_ext_pblock(ex));
				err = ext4_ext_zeroout(inode, &zero_ex);
				if (err)
					goto out;
			}

			split_map.m_lblk = ee_block;
			split_map.m_len = map->m_lblk - ee_block + map->m_len;
			allocated = map->m_len;
		}
	}

	allocated = ext4_split_extent(handle, inode, path,
				       &split_map, split_flag, 0);
	if (allocated < 0)
		err = allocated;

out:
	return err ? err : allocated;
}

/*
 * This function is called by ext4_ext_map_blocks() from
 * ext4_get_blocks_dio_write() when DIO to write
 * to an uninitialized extent.
 *
 * Writing to an uninitialized extent may result in splitting the uninitialized
 * extent into multiple /initialized uninitialized extents (up to three)
 * There are three possibilities:
 *   a> There is no split required: Entire extent should be uninitialized
 *   b> Splits in two extents: Write is happening at either end of the extent
 *   c> Splits in three extents: Somone is writing in middle of the extent
 *
 * One of more index blocks maybe needed if the extent tree grow after
 * the uninitialized extent split. To prevent ENOSPC occur at the IO
 * complete, we need to split the uninitialized extent before DIO submit
 * the IO. The uninitialized extent called at this time will be split
 * into three uninitialized extent(at most). After IO complete, the part
 * being filled will be convert to initialized by the end_io callback function
 * via ext4_convert_unwritten_extents().
 *
 * Returns the size of uninitialized extent to be written on success.
 */
static int ext4_split_unwritten_extents(handle_t *handle,
					struct inode *inode,
					struct ext4_map_blocks *map,
					struct ext4_ext_path *path,
					int flags)
{
	ext4_lblk_t eof_block;
	ext4_lblk_t ee_block;
	struct ext4_extent *ex;
	unsigned int ee_len;
	int split_flag = 0, depth;

	ext_debug("ext4_split_unwritten_extents: inode %lu, logical"
		"block %llu, max_blocks %u\n", inode->i_ino,
		(unsigned long long)map->m_lblk, map->m_len);

	eof_block = (inode->i_size + inode->i_sb->s_blocksize - 1) >>
		inode->i_sb->s_blocksize_bits;
	if (eof_block < map->m_lblk + map->m_len)
		eof_block = map->m_lblk + map->m_len;
	depth = ext_depth(inode);
	ex = path[depth].p_ext;
	ee_block = le32_to_cpu(ex->ee_block);
	ee_len = ext4_ext_get_actual_len(ex);

	split_flag |= ee_block + ee_len <= eof_block ? EXT4_EXT_MAY_ZEROOUT : 0;
	split_flag |= EXT4_EXT_MARK_UNINIT2;

	flags |= EXT4_GET_BLOCKS_PRE_IO;
	return ext4_split_extent(handle, inode, path, map, split_flag, flags);
}

static int ext4_convert_unwritten_extents_endio(handle_t *handle,
					      struct inode *inode,
					      struct ext4_ext_path *path)
{
	struct ext4_extent *ex;
	int depth;
	int err = 0;

	depth = ext_depth(inode);
	ex = path[depth].p_ext;

	ext_debug("ext4_convert_unwritten_extents_endio: inode %lu, logical"
		"block %llu, max_blocks %u\n", inode->i_ino,
		(unsigned long long)le32_to_cpu(ex->ee_block),
		ext4_ext_get_actual_len(ex));

	err = ext4_ext_get_access(handle, inode, path + depth);
	if (err)
		goto out;
	
	ext4_ext_mark_initialized(ex);

	ext4_ext_try_to_merge(inode, path, ex);

	
	err = ext4_ext_dirty(handle, inode, path + depth);
out:
	ext4_ext_show_leaf(inode, path);
	return err;
}

static void unmap_underlying_metadata_blocks(struct block_device *bdev,
			sector_t block, int count)
{
	int i;
	for (i = 0; i < count; i++)
                unmap_underlying_metadata(bdev, block + i);
}

static int check_eofblocks_fl(handle_t *handle, struct inode *inode,
			      ext4_lblk_t lblk,
			      struct ext4_ext_path *path,
			      unsigned int len)
{
	int i, depth;
	struct ext4_extent_header *eh;
	struct ext4_extent *last_ex;

	if (!ext4_test_inode_flag(inode, EXT4_INODE_EOFBLOCKS))
		return 0;

	depth = ext_depth(inode);
	eh = path[depth].p_hdr;

	if (unlikely(!eh->eh_entries))
		goto out;
	last_ex = EXT_LAST_EXTENT(eh);
	if (lblk + len < le32_to_cpu(last_ex->ee_block) +
	    ext4_ext_get_actual_len(last_ex))
		return 0;
	for (i = depth-1; i >= 0; i--)
		if (path[i].p_idx != EXT_LAST_INDEX(path[i].p_hdr))
			return 0;
out:
	ext4_clear_inode_flag(inode, EXT4_INODE_EOFBLOCKS);
	return ext4_mark_inode_dirty(handle, inode);
}

static int ext4_find_delalloc_range(struct inode *inode,
				    ext4_lblk_t lblk_start,
				    ext4_lblk_t lblk_end,
				    int search_hint_reverse)
{
	struct address_space *mapping = inode->i_mapping;
	struct buffer_head *head, *bh = NULL;
	struct page *page;
	ext4_lblk_t i, pg_lblk;
	pgoff_t index;

	if (!test_opt(inode->i_sb, DELALLOC))
		return 0;

	
	if (inode->i_blkbits < PAGE_CACHE_SHIFT)
		search_hint_reverse = 0;

	if (search_hint_reverse)
		i = lblk_end;
	else
		i = lblk_start;

	index = i >> (PAGE_CACHE_SHIFT - inode->i_blkbits);

	while ((i >= lblk_start) && (i <= lblk_end)) {
		page = find_get_page(mapping, index);
		if (!page)
			goto nextpage;

		if (!page_has_buffers(page))
			goto nextpage;

		head = page_buffers(page);
		if (!head)
			goto nextpage;

		bh = head;
		pg_lblk = index << (PAGE_CACHE_SHIFT -
						inode->i_blkbits);
		do {
			if (unlikely(pg_lblk < lblk_start)) {
				pg_lblk++;
				continue;
			}

			if (buffer_delay(bh) && !buffer_da_mapped(bh)) {
				page_cache_release(page);
				trace_ext4_find_delalloc_range(inode,
						lblk_start, lblk_end,
						search_hint_reverse,
						1, i);
				return 1;
			}
			if (search_hint_reverse)
				i--;
			else
				i++;
		} while ((i >= lblk_start) && (i <= lblk_end) &&
				((bh = bh->b_this_page) != head));
nextpage:
		if (page)
			page_cache_release(page);
		if (search_hint_reverse)
			index--;
		else
			index++;
		i = index << (PAGE_CACHE_SHIFT - inode->i_blkbits);
	}

	trace_ext4_find_delalloc_range(inode, lblk_start, lblk_end,
					search_hint_reverse, 0, 0);
	return 0;
}

int ext4_find_delalloc_cluster(struct inode *inode, ext4_lblk_t lblk,
			       int search_hint_reverse)
{
	struct ext4_sb_info *sbi = EXT4_SB(inode->i_sb);
	ext4_lblk_t lblk_start, lblk_end;
	lblk_start = lblk & (~(sbi->s_cluster_ratio - 1));
	lblk_end = lblk_start + sbi->s_cluster_ratio - 1;

	return ext4_find_delalloc_range(inode, lblk_start, lblk_end,
					search_hint_reverse);
}

/**
 * Determines how many complete clusters (out of those specified by the 'map')
 * are under delalloc and were reserved quota for.
 * This function is called when we are writing out the blocks that were
 * originally written with their allocation delayed, but then the space was
 * allocated using fallocate() before the delayed allocation could be resolved.
 * The cases to look for are:
 * ('=' indicated delayed allocated blocks
 *  '-' indicates non-delayed allocated blocks)
 * (a) partial clusters towards beginning and/or end outside of allocated range
 *     are not delalloc'ed.
 *	Ex:
 *	|----c---=|====c====|====c====|===-c----|
 *	         |++++++ allocated ++++++|
 *	==> 4 complete clusters in above example
 *
 * (b) partial cluster (outside of allocated range) towards either end is
 *     marked for delayed allocation. In this case, we will exclude that
 *     cluster.
 *	Ex:
 *	|----====c========|========c========|
 *	     |++++++ allocated ++++++|
 *	==> 1 complete clusters in above example
 *
 *	Ex:
 *	|================c================|
 *            |++++++ allocated ++++++|
 *	==> 0 complete clusters in above example
 *
 * The ext4_da_update_reserve_space will be called only if we
 * determine here that there were some "entire" clusters that span
 * this 'allocated' range.
 * In the non-bigalloc case, this function will just end up returning num_blks
 * without ever calling ext4_find_delalloc_range.
 */
static unsigned int
get_reserved_cluster_alloc(struct inode *inode, ext4_lblk_t lblk_start,
			   unsigned int num_blks)
{
	struct ext4_sb_info *sbi = EXT4_SB(inode->i_sb);
	ext4_lblk_t alloc_cluster_start, alloc_cluster_end;
	ext4_lblk_t lblk_from, lblk_to, c_offset;
	unsigned int allocated_clusters = 0;

	alloc_cluster_start = EXT4_B2C(sbi, lblk_start);
	alloc_cluster_end = EXT4_B2C(sbi, lblk_start + num_blks - 1);

	
	allocated_clusters = alloc_cluster_end - alloc_cluster_start + 1;

	trace_ext4_get_reserved_cluster_alloc(inode, lblk_start, num_blks);

	
	c_offset = lblk_start & (sbi->s_cluster_ratio - 1);
	if (c_offset) {
		lblk_from = lblk_start & (~(sbi->s_cluster_ratio - 1));
		lblk_to = lblk_from + c_offset - 1;

		if (ext4_find_delalloc_range(inode, lblk_from, lblk_to, 0))
			allocated_clusters--;
	}

	
	c_offset = (lblk_start + num_blks) & (sbi->s_cluster_ratio - 1);
	if (allocated_clusters && c_offset) {
		lblk_from = lblk_start + num_blks;
		lblk_to = lblk_from + (sbi->s_cluster_ratio - c_offset) - 1;

		if (ext4_find_delalloc_range(inode, lblk_from, lblk_to, 0))
			allocated_clusters--;
	}

	return allocated_clusters;
}

static int
ext4_ext_handle_uninitialized_extents(handle_t *handle, struct inode *inode,
			struct ext4_map_blocks *map,
			struct ext4_ext_path *path, int flags,
			unsigned int allocated, ext4_fsblk_t newblock)
{
	int ret = 0;
	int err = 0;
	ext4_io_end_t *io = EXT4_I(inode)->cur_aio_dio;

	ext_debug("ext4_ext_handle_uninitialized_extents: inode %lu, logical "
		  "block %llu, max_blocks %u, flags %x, allocated %u\n",
		  inode->i_ino, (unsigned long long)map->m_lblk, map->m_len,
		  flags, allocated);
	ext4_ext_show_leaf(inode, path);

	trace_ext4_ext_handle_uninitialized_extents(inode, map, allocated,
						    newblock);

	
	if ((flags & EXT4_GET_BLOCKS_PRE_IO)) {
		ret = ext4_split_unwritten_extents(handle, inode, map,
						   path, flags);
		/*
		 * Flag the inode(non aio case) or end_io struct (aio case)
		 * that this IO needs to conversion to written when IO is
		 * completed
		 */
		if (io)
			ext4_set_io_unwritten_flag(inode, io);
		else
			ext4_set_inode_state(inode, EXT4_STATE_DIO_UNWRITTEN);
		if (ext4_should_dioread_nolock(inode))
			map->m_flags |= EXT4_MAP_UNINIT;
		goto out;
	}
	/* IO end_io complete, convert the filled extent to written */
	if ((flags & EXT4_GET_BLOCKS_CONVERT)) {
		ret = ext4_convert_unwritten_extents_endio(handle, inode,
							path);
		if (ret >= 0) {
			ext4_update_inode_fsync_trans(handle, inode, 1);
			err = check_eofblocks_fl(handle, inode, map->m_lblk,
						 path, map->m_len);
		} else
			err = ret;
		goto out2;
	}
	
	/*
	 * repeat fallocate creation request
	 * we already have an unwritten extent
	 */
	if (flags & EXT4_GET_BLOCKS_UNINIT_EXT)
		goto map_out;

	
	if ((flags & EXT4_GET_BLOCKS_CREATE) == 0) {
		map->m_flags |= EXT4_MAP_UNWRITTEN;
		goto out1;
	}

	
	ret = ext4_ext_convert_to_initialized(handle, inode, map, path);
	if (ret >= 0)
		ext4_update_inode_fsync_trans(handle, inode, 1);
out:
	if (ret <= 0) {
		err = ret;
		goto out2;
	} else
		allocated = ret;
	map->m_flags |= EXT4_MAP_NEW;
	if (allocated > map->m_len) {
		unmap_underlying_metadata_blocks(inode->i_sb->s_bdev,
					newblock + map->m_len,
					allocated - map->m_len);
		allocated = map->m_len;
	}

	if (flags & EXT4_GET_BLOCKS_DELALLOC_RESERVE) {
		unsigned int reserved_clusters;
		reserved_clusters = get_reserved_cluster_alloc(inode,
				map->m_lblk, map->m_len);
		if (reserved_clusters)
			ext4_da_update_reserve_space(inode,
						     reserved_clusters,
						     0);
	}

map_out:
	map->m_flags |= EXT4_MAP_MAPPED;
	if ((flags & EXT4_GET_BLOCKS_KEEP_SIZE) == 0) {
		err = check_eofblocks_fl(handle, inode, map->m_lblk, path,
					 map->m_len);
		if (err < 0)
			goto out2;
	}
out1:
	if (allocated > map->m_len)
		allocated = map->m_len;
	ext4_ext_show_leaf(inode, path);
	map->m_pblk = newblock;
	map->m_len = allocated;
out2:
	if (path) {
		ext4_ext_drop_refs(path);
		kfree(path);
	}
	return err ? err : allocated;
}

static int get_implied_cluster_alloc(struct super_block *sb,
				     struct ext4_map_blocks *map,
				     struct ext4_extent *ex,
				     struct ext4_ext_path *path)
{
	struct ext4_sb_info *sbi = EXT4_SB(sb);
	ext4_lblk_t c_offset = map->m_lblk & (sbi->s_cluster_ratio-1);
	ext4_lblk_t ex_cluster_start, ex_cluster_end;
	ext4_lblk_t rr_cluster_start;
	ext4_lblk_t ee_block = le32_to_cpu(ex->ee_block);
	ext4_fsblk_t ee_start = ext4_ext_pblock(ex);
	unsigned short ee_len = ext4_ext_get_actual_len(ex);

	
	ex_cluster_start = EXT4_B2C(sbi, ee_block);
	ex_cluster_end = EXT4_B2C(sbi, ee_block + ee_len - 1);

	
	rr_cluster_start = EXT4_B2C(sbi, map->m_lblk);

	if ((rr_cluster_start == ex_cluster_end) ||
	    (rr_cluster_start == ex_cluster_start)) {
		if (rr_cluster_start == ex_cluster_end)
			ee_start += ee_len - 1;
		map->m_pblk = (ee_start & ~(sbi->s_cluster_ratio - 1)) +
			c_offset;
		map->m_len = min(map->m_len,
				 (unsigned) sbi->s_cluster_ratio - c_offset);

		if (map->m_lblk < ee_block)
			map->m_len = min(map->m_len, ee_block - map->m_lblk);

		if (map->m_lblk > ee_block) {
			ext4_lblk_t next = ext4_ext_next_allocated_block(path);
			map->m_len = min(map->m_len, next - map->m_lblk);
		}

		trace_ext4_get_implied_cluster_alloc_exit(sb, map, 1);
		return 1;
	}

	trace_ext4_get_implied_cluster_alloc_exit(sb, map, 0);
	return 0;
}


int ext4_ext_map_blocks(handle_t *handle, struct inode *inode,
			struct ext4_map_blocks *map, int flags)
{
	struct ext4_ext_path *path = NULL;
	struct ext4_extent newex, *ex, *ex2;
	struct ext4_sb_info *sbi = EXT4_SB(inode->i_sb);
	ext4_fsblk_t newblock = 0;
	int free_on_err = 0, err = 0, depth, ret;
	unsigned int allocated = 0, offset = 0;
	unsigned int allocated_clusters = 0;
	struct ext4_allocation_request ar;
	ext4_io_end_t *io = EXT4_I(inode)->cur_aio_dio;
	ext4_lblk_t cluster_offset;

	ext_debug("blocks %u/%u requested for inode %lu\n",
		  map->m_lblk, map->m_len, inode->i_ino);
	trace_ext4_ext_map_blocks_enter(inode, map->m_lblk, map->m_len, flags);

	
	if (ext4_ext_in_cache(inode, map->m_lblk, &newex)) {
		if (!newex.ee_start_lo && !newex.ee_start_hi) {
			if ((sbi->s_cluster_ratio > 1) &&
			    ext4_find_delalloc_cluster(inode, map->m_lblk, 0))
				map->m_flags |= EXT4_MAP_FROM_CLUSTER;

			if ((flags & EXT4_GET_BLOCKS_CREATE) == 0) {
				goto out2;
			}
			
		} else {
			
			if (sbi->s_cluster_ratio > 1)
				map->m_flags |= EXT4_MAP_FROM_CLUSTER;
			newblock = map->m_lblk
				   - le32_to_cpu(newex.ee_block)
				   + ext4_ext_pblock(&newex);
			
			allocated = ext4_ext_get_actual_len(&newex) -
				(map->m_lblk - le32_to_cpu(newex.ee_block));
			goto out;
		}
	}

	
	path = ext4_ext_find_extent(inode, map->m_lblk, NULL);
	if (IS_ERR(path)) {
		err = PTR_ERR(path);
		path = NULL;
		goto out2;
	}

	depth = ext_depth(inode);

	if (unlikely(path[depth].p_ext == NULL && depth != 0)) {
		EXT4_ERROR_INODE(inode, "bad extent address "
				 "lblock: %lu, depth: %d pblock %lld",
				 (unsigned long) map->m_lblk, depth,
				 path[depth].p_block);
		err = -EIO;
		goto out2;
	}

	ex = path[depth].p_ext;
	if (ex) {
		ext4_lblk_t ee_block = le32_to_cpu(ex->ee_block);
		ext4_fsblk_t ee_start = ext4_ext_pblock(ex);
		unsigned short ee_len;

		ee_len = ext4_ext_get_actual_len(ex);

		trace_ext4_ext_show_extent(inode, ee_block, ee_start, ee_len);

		
		if (in_range(map->m_lblk, ee_block, ee_len)) {
			newblock = map->m_lblk - ee_block + ee_start;
			
			allocated = ee_len - (map->m_lblk - ee_block);
			ext_debug("%u fit into %u:%d -> %llu\n", map->m_lblk,
				  ee_block, ee_len, newblock);

			if (!ext4_ext_is_uninitialized(ex)) {
				ext4_ext_put_in_cache(inode, ee_block,
					ee_len, ee_start);
				goto out;
			}
			ret = ext4_ext_handle_uninitialized_extents(
				handle, inode, map, path, flags,
				allocated, newblock);
			return ret;
		}
	}

	if ((sbi->s_cluster_ratio > 1) &&
	    ext4_find_delalloc_cluster(inode, map->m_lblk, 0))
		map->m_flags |= EXT4_MAP_FROM_CLUSTER;

	if ((flags & EXT4_GET_BLOCKS_CREATE) == 0) {
		ext4_ext_put_gap_in_cache(inode, path, map->m_lblk);
		goto out2;
	}

	map->m_flags &= ~EXT4_MAP_FROM_CLUSTER;
	newex.ee_block = cpu_to_le32(map->m_lblk);
	cluster_offset = map->m_lblk & (sbi->s_cluster_ratio-1);

	if (cluster_offset && ex &&
	    get_implied_cluster_alloc(inode->i_sb, map, ex, path)) {
		ar.len = allocated = map->m_len;
		newblock = map->m_pblk;
		map->m_flags |= EXT4_MAP_FROM_CLUSTER;
		goto got_allocated_blocks;
	}

	
	ar.lleft = map->m_lblk;
	err = ext4_ext_search_left(inode, path, &ar.lleft, &ar.pleft);
	if (err)
		goto out2;
	ar.lright = map->m_lblk;
	ex2 = NULL;
	err = ext4_ext_search_right(inode, path, &ar.lright, &ar.pright, &ex2);
	if (err)
		goto out2;

	if ((sbi->s_cluster_ratio > 1) && ex2 &&
	    get_implied_cluster_alloc(inode->i_sb, map, ex2, path)) {
		ar.len = allocated = map->m_len;
		newblock = map->m_pblk;
		map->m_flags |= EXT4_MAP_FROM_CLUSTER;
		goto got_allocated_blocks;
	}

	if (map->m_len > EXT_INIT_MAX_LEN &&
	    !(flags & EXT4_GET_BLOCKS_UNINIT_EXT))
		map->m_len = EXT_INIT_MAX_LEN;
	else if (map->m_len > EXT_UNINIT_MAX_LEN &&
		 (flags & EXT4_GET_BLOCKS_UNINIT_EXT))
		map->m_len = EXT_UNINIT_MAX_LEN;

	
	newex.ee_len = cpu_to_le16(map->m_len);
	err = ext4_ext_check_overlap(sbi, inode, &newex, path);
	if (err)
		allocated = ext4_ext_get_actual_len(&newex);
	else
		allocated = map->m_len;

	
	ar.inode = inode;
	ar.goal = ext4_ext_find_goal(inode, path, map->m_lblk);
	ar.logical = map->m_lblk;
	offset = map->m_lblk & (sbi->s_cluster_ratio - 1);
	ar.len = EXT4_NUM_B2C(sbi, offset+allocated);
	ar.goal -= offset;
	ar.logical -= offset;
	if (S_ISREG(inode->i_mode))
		ar.flags = EXT4_MB_HINT_DATA;
	else
		
		ar.flags = 0;
	if (flags & EXT4_GET_BLOCKS_NO_NORMALIZE)
		ar.flags |= EXT4_MB_HINT_NOPREALLOC;
	newblock = ext4_mb_new_blocks(handle, &ar, &err);
	if (!newblock)
		goto out2;
	ext_debug("allocate new block: goal %llu, found %llu/%u\n",
		  ar.goal, newblock, allocated);
	free_on_err = 1;
	allocated_clusters = ar.len;
	ar.len = EXT4_C2B(sbi, ar.len) - offset;
	if (ar.len > allocated)
		ar.len = allocated;

got_allocated_blocks:
	
	ext4_ext_store_pblock(&newex, newblock + offset);
	newex.ee_len = cpu_to_le16(ar.len);
	
	if (flags & EXT4_GET_BLOCKS_UNINIT_EXT){
		ext4_ext_mark_uninitialized(&newex);
		if ((flags & EXT4_GET_BLOCKS_PRE_IO)) {
			if (io)
				ext4_set_io_unwritten_flag(inode, io);
			else
				ext4_set_inode_state(inode,
						     EXT4_STATE_DIO_UNWRITTEN);
		}
		if (ext4_should_dioread_nolock(inode))
			map->m_flags |= EXT4_MAP_UNINIT;
	}

	err = 0;
	if ((flags & EXT4_GET_BLOCKS_KEEP_SIZE) == 0)
		err = check_eofblocks_fl(handle, inode, map->m_lblk,
					 path, ar.len);
	if (!err)
		err = ext4_ext_insert_extent(handle, inode, path,
					     &newex, flags);
	if (err && free_on_err) {
		int fb_flags = flags & EXT4_GET_BLOCKS_DELALLOC_RESERVE ?
			EXT4_FREE_BLOCKS_NO_QUOT_UPDATE : 0;
		
		ext4_discard_preallocations(inode);
		ext4_free_blocks(handle, inode, NULL, ext4_ext_pblock(&newex),
				 ext4_ext_get_actual_len(&newex), fb_flags);
		goto out2;
	}

	
	newblock = ext4_ext_pblock(&newex);
	allocated = ext4_ext_get_actual_len(&newex);
	if (allocated > map->m_len)
		allocated = map->m_len;
	map->m_flags |= EXT4_MAP_NEW;

	if (flags & EXT4_GET_BLOCKS_DELALLOC_RESERVE) {
		unsigned int reserved_clusters;
		reserved_clusters = get_reserved_cluster_alloc(inode,
						map->m_lblk, allocated);
		if (map->m_flags & EXT4_MAP_FROM_CLUSTER) {
			if (reserved_clusters) {
				ext4_da_update_reserve_space(inode,
						reserved_clusters, 0);
			}
		} else {
			BUG_ON(allocated_clusters < reserved_clusters);
			
			ext4_da_update_reserve_space(inode, allocated_clusters,
							1);
			if (reserved_clusters < allocated_clusters) {
				struct ext4_inode_info *ei = EXT4_I(inode);
				int reservation = allocated_clusters -
						  reserved_clusters;
				/*
				 * It seems we claimed few clusters outside of
				 * the range of this allocation. We should give
				 * it back to the reservation pool. This can
				 * happen in the following case:
				 *
				 * * Suppose s_cluster_ratio is 4 (i.e., each
				 *   cluster has 4 blocks. Thus, the clusters
				 *   are [0-3],[4-7],[8-11]...
				 * * First comes delayed allocation write for
				 *   logical blocks 10 & 11. Since there were no
				 *   previous delayed allocated blocks in the
				 *   range [8-11], we would reserve 1 cluster
				 *   for this write.
				 * * Next comes write for logical blocks 3 to 8.
				 *   In this case, we will reserve 2 clusters
				 *   (for [0-3] and [4-7]; and not for [8-11] as
				 *   that range has a delayed allocated blocks.
				 *   Thus total reserved clusters now becomes 3.
				 * * Now, during the delayed allocation writeout
				 *   time, we will first write blocks [3-8] and
				 *   allocate 3 clusters for writing these
				 *   blocks. Also, we would claim all these
				 *   three clusters above.
				 * * Now when we come here to writeout the
				 *   blocks [10-11], we would expect to claim
				 *   the reservation of 1 cluster we had made
				 *   (and we would claim it since there are no
				 *   more delayed allocated blocks in the range
				 *   [8-11]. But our reserved cluster count had
				 *   already gone to 0.
				 *
				 *   Thus, at the step 4 above when we determine
				 *   that there are still some unwritten delayed
				 *   allocated blocks outside of our current
				 *   block range, we should increment the
				 *   reserved clusters count so that when the
				 *   remaining blocks finally gets written, we
				 *   could claim them.
				 */
				dquot_reserve_block(inode,
						EXT4_C2B(sbi, reservation));
				spin_lock(&ei->i_block_reservation_lock);
				ei->i_reserved_data_blocks += reservation;
				spin_unlock(&ei->i_block_reservation_lock);
			}
		}
	}

	if ((flags & EXT4_GET_BLOCKS_UNINIT_EXT) == 0) {
		ext4_ext_put_in_cache(inode, map->m_lblk, allocated, newblock);
		ext4_update_inode_fsync_trans(handle, inode, 1);
	} else
		ext4_update_inode_fsync_trans(handle, inode, 0);
out:
	if (allocated > map->m_len)
		allocated = map->m_len;
	ext4_ext_show_leaf(inode, path);
	map->m_flags |= EXT4_MAP_MAPPED;
	map->m_pblk = newblock;
	map->m_len = allocated;
out2:
	if (path) {
		ext4_ext_drop_refs(path);
		kfree(path);
	}

	trace_ext4_ext_map_blocks_exit(inode, map->m_lblk,
		newblock, map->m_len, err ? err : allocated);

	return err ? err : allocated;
}

void ext4_ext_truncate(struct inode *inode)
{
	struct address_space *mapping = inode->i_mapping;
	struct super_block *sb = inode->i_sb;
	ext4_lblk_t last_block;
	handle_t *handle;
	loff_t page_len;
	int err = 0;

	ext4_flush_completed_IO(inode);

	err = ext4_writepage_trans_blocks(inode);
	handle = ext4_journal_start(inode, err);
	if (IS_ERR(handle))
		return;

	if (inode->i_size % PAGE_CACHE_SIZE != 0) {
		page_len = PAGE_CACHE_SIZE -
			(inode->i_size & (PAGE_CACHE_SIZE - 1));

		err = ext4_discard_partial_page_buffers(handle,
			mapping, inode->i_size, page_len, 0);

		if (err)
			goto out_stop;
	}

	if (ext4_orphan_add(handle, inode))
		goto out_stop;

	down_write(&EXT4_I(inode)->i_data_sem);
	ext4_ext_invalidate_cache(inode);

	ext4_discard_preallocations(inode);


	
	EXT4_I(inode)->i_disksize = inode->i_size;
	ext4_mark_inode_dirty(handle, inode);

	last_block = (inode->i_size + sb->s_blocksize - 1)
			>> EXT4_BLOCK_SIZE_BITS(sb);
	err = ext4_ext_remove_space(inode, last_block, EXT_MAX_BLOCKS - 1);

	if (IS_SYNC(inode))
		ext4_handle_sync(handle);

	up_write(&EXT4_I(inode)->i_data_sem);

out_stop:
	if (inode->i_nlink)
		ext4_orphan_del(handle, inode);

	inode->i_mtime = inode->i_ctime = ext4_current_time(inode);
	ext4_mark_inode_dirty(handle, inode);
	ext4_journal_stop(handle);
}

static void ext4_falloc_update_inode(struct inode *inode,
				int mode, loff_t new_size, int update_ctime)
{
	struct timespec now;

	if (update_ctime) {
		now = current_fs_time(inode->i_sb);
		if (!timespec_equal(&inode->i_ctime, &now))
			inode->i_ctime = now;
	}
	if (!(mode & FALLOC_FL_KEEP_SIZE)) {
		if (new_size > i_size_read(inode))
			i_size_write(inode, new_size);
		if (new_size > EXT4_I(inode)->i_disksize)
			ext4_update_i_disksize(inode, new_size);
	} else {
		if (new_size > i_size_read(inode))
			ext4_set_inode_flag(inode, EXT4_INODE_EOFBLOCKS);
	}

}

long ext4_fallocate(struct file *file, int mode, loff_t offset, loff_t len)
{
	struct inode *inode = file->f_path.dentry->d_inode;
	handle_t *handle;
	loff_t new_size;
	unsigned int max_blocks;
	int ret = 0;
	int ret2 = 0;
	int retries = 0;
	int flags;
	struct ext4_map_blocks map;
	unsigned int credits, blkbits = inode->i_blkbits;

	if (!(ext4_test_inode_flag(inode, EXT4_INODE_EXTENTS)))
		return -EOPNOTSUPP;

	
	if (mode & ~(FALLOC_FL_KEEP_SIZE | FALLOC_FL_PUNCH_HOLE))
		return -EOPNOTSUPP;

	if (mode & FALLOC_FL_PUNCH_HOLE)
		return ext4_punch_hole(file, offset, len);

	trace_ext4_fallocate_enter(inode, offset, len, mode);
	map.m_lblk = offset >> blkbits;
	max_blocks = (EXT4_BLOCK_ALIGN(len + offset, blkbits) >> blkbits)
		- map.m_lblk;
	credits = ext4_chunk_trans_blocks(inode, max_blocks);
	mutex_lock(&inode->i_mutex);
	ret = inode_newsize_ok(inode, (len + offset));
	if (ret) {
		mutex_unlock(&inode->i_mutex);
		trace_ext4_fallocate_exit(inode, offset, max_blocks, ret);
		return ret;
	}
	flags = EXT4_GET_BLOCKS_CREATE_UNINIT_EXT;
	if (mode & FALLOC_FL_KEEP_SIZE)
		flags |= EXT4_GET_BLOCKS_KEEP_SIZE;
	if (len <= EXT_UNINIT_MAX_LEN << blkbits)
		flags |= EXT4_GET_BLOCKS_NO_NORMALIZE;
retry:
	while (ret >= 0 && ret < max_blocks) {
		map.m_lblk = map.m_lblk + ret;
		map.m_len = max_blocks = max_blocks - ret;
		handle = ext4_journal_start(inode, credits);
		if (IS_ERR(handle)) {
			ret = PTR_ERR(handle);
			break;
		}
		ret = ext4_map_blocks(handle, inode, &map, flags);
		if (ret <= 0) {
#ifdef EXT4FS_DEBUG
			WARN_ON(ret <= 0);
			printk(KERN_ERR "%s: ext4_ext_map_blocks "
				    "returned error inode#%lu, block=%u, "
				    "max_blocks=%u", __func__,
				    inode->i_ino, map.m_lblk, max_blocks);
#endif
			ext4_mark_inode_dirty(handle, inode);
			ret2 = ext4_journal_stop(handle);
			break;
		}
		if ((map.m_lblk + ret) >= (EXT4_BLOCK_ALIGN(offset + len,
						blkbits) >> blkbits))
			new_size = offset + len;
		else
			new_size = ((loff_t) map.m_lblk + ret) << blkbits;

		ext4_falloc_update_inode(inode, mode, new_size,
					 (map.m_flags & EXT4_MAP_NEW));
		ext4_mark_inode_dirty(handle, inode);
		ret2 = ext4_journal_stop(handle);
		if (ret2)
			break;
	}
	if (ret == -ENOSPC &&
			ext4_should_retry_alloc(inode->i_sb, &retries)) {
		ret = 0;
		goto retry;
	}
	mutex_unlock(&inode->i_mutex);
	trace_ext4_fallocate_exit(inode, offset, max_blocks,
				ret > 0 ? ret2 : ret);
	return ret > 0 ? ret2 : ret;
}

/*
 * This function convert a range of blocks to written extents
 * The caller of this function will pass the start offset and the size.
 * all unwritten extents within this range will be converted to
 * written extents.
 *
 * This function is called from the direct IO end io call back
 * function, to convert the fallocated extents after IO is completed.
 * Returns 0 on success.
 */
int ext4_convert_unwritten_extents(struct inode *inode, loff_t offset,
				    ssize_t len)
{
	handle_t *handle;
	unsigned int max_blocks;
	int ret = 0;
	int ret2 = 0;
	struct ext4_map_blocks map;
	unsigned int credits, blkbits = inode->i_blkbits;

	map.m_lblk = offset >> blkbits;
	max_blocks = ((EXT4_BLOCK_ALIGN(len + offset, blkbits) >> blkbits) -
		      map.m_lblk);
	credits = ext4_chunk_trans_blocks(inode, max_blocks);
	while (ret >= 0 && ret < max_blocks) {
		map.m_lblk += ret;
		map.m_len = (max_blocks -= ret);
		handle = ext4_journal_start(inode, credits);
		if (IS_ERR(handle)) {
			ret = PTR_ERR(handle);
			break;
		}
		ret = ext4_map_blocks(handle, inode, &map,
				      EXT4_GET_BLOCKS_IO_CONVERT_EXT);
		if (ret <= 0) {
			WARN_ON(ret <= 0);
			ext4_msg(inode->i_sb, KERN_ERR,
				 "%s:%d: inode #%lu: block %u: len %u: "
				 "ext4_ext_map_blocks returned %d",
				 __func__, __LINE__, inode->i_ino, map.m_lblk,
				 map.m_len, ret);
		}
		ext4_mark_inode_dirty(handle, inode);
		ret2 = ext4_journal_stop(handle);
		if (ret <= 0 || ret2 )
			break;
	}
	return ret > 0 ? ret2 : ret;
}

static int ext4_ext_fiemap_cb(struct inode *inode, ext4_lblk_t next,
		       struct ext4_ext_cache *newex, struct ext4_extent *ex,
		       void *data)
{
	__u64	logical;
	__u64	physical;
	__u64	length;
	__u32	flags = 0;
	int		ret = 0;
	struct fiemap_extent_info *fieinfo = data;
	unsigned char blksize_bits;

	blksize_bits = inode->i_sb->s_blocksize_bits;
	logical = (__u64)newex->ec_block << blksize_bits;

	if (newex->ec_start == 0) {
		ext4_lblk_t	end = 0;
		pgoff_t		last_offset;
		pgoff_t		offset;
		pgoff_t		index;
		pgoff_t		start_index = 0;
		struct page	**pages = NULL;
		struct buffer_head *bh = NULL;
		struct buffer_head *head = NULL;
		unsigned int nr_pages = PAGE_SIZE / sizeof(struct page *);

		pages = kmalloc(PAGE_SIZE, GFP_KERNEL);
		if (pages == NULL)
			return -ENOMEM;

		offset = logical >> PAGE_SHIFT;
repeat:
		last_offset = offset;
		head = NULL;
		ret = find_get_pages_tag(inode->i_mapping, &offset,
					PAGECACHE_TAG_DIRTY, nr_pages, pages);

		if (!(flags & FIEMAP_EXTENT_DELALLOC)) {
			
			if (ret == 0) {
out:
				for (index = 0; index < ret; index++)
					page_cache_release(pages[index]);
				
				kfree(pages);
				return EXT_CONTINUE;
			}
			index = 0;

next_page:
			
			end = ((__u64)pages[index]->index << PAGE_SHIFT) >>
				  blksize_bits;
			if (!page_has_buffers(pages[index]))
				goto out;
			head = page_buffers(pages[index]);
			if (!head)
				goto out;

			index++;
			bh = head;
			do {
				if (end >= newex->ec_block +
					newex->ec_len)
					goto out;

				if (buffer_mapped(bh) &&
				    end >= newex->ec_block) {
					start_index = index - 1;
					
					goto found_mapped_buffer;
				}

				bh = bh->b_this_page;
				end++;
			} while (bh != head);

			if (index >= ret) {
				newex->ec_len = end - newex->ec_block;
				goto out;
			}
			goto next_page;
		} else {
			
			if (ret > 0 && pages[0]->index == last_offset)
				head = page_buffers(pages[0]);
			bh = head;
			index = 1;
			start_index = 0;
		}

found_mapped_buffer:
		if (bh != NULL && buffer_delay(bh)) {
			
			if (!(flags & FIEMAP_EXTENT_DELALLOC)) {
				flags |= FIEMAP_EXTENT_DELALLOC;
				newex->ec_block = end;
				logical = (__u64)end << blksize_bits;
			}
			
			do {
				if (!buffer_delay(bh))
					goto found_delayed_extent;
				bh = bh->b_this_page;
				end++;
			} while (bh != head);

			for (; index < ret; index++) {
				if (!page_has_buffers(pages[index])) {
					bh = NULL;
					break;
				}
				head = page_buffers(pages[index]);
				if (!head) {
					bh = NULL;
					break;
				}

				if (pages[index]->index !=
				    pages[start_index]->index + index
				    - start_index) {
					
					bh = NULL;
					break;
				}
				bh = head;
				do {
					if (!buffer_delay(bh))
						
						goto found_delayed_extent;
					bh = bh->b_this_page;
					end++;
				} while (bh != head);
			}
		} else if (!(flags & FIEMAP_EXTENT_DELALLOC))
			
			goto out;

found_delayed_extent:
		newex->ec_len = min(end - newex->ec_block,
						(ext4_lblk_t)EXT_INIT_MAX_LEN);
		if (ret == nr_pages && bh != NULL &&
			newex->ec_len < EXT_INIT_MAX_LEN &&
			buffer_delay(bh)) {
			
			for (index = 0; index < ret; index++)
				page_cache_release(pages[index]);
			goto repeat;
		}

		for (index = 0; index < ret; index++)
			page_cache_release(pages[index]);
		kfree(pages);
	}

	physical = (__u64)newex->ec_start << blksize_bits;
	length =   (__u64)newex->ec_len << blksize_bits;

	if (ex && ext4_ext_is_uninitialized(ex))
		flags |= FIEMAP_EXTENT_UNWRITTEN;

	if (next == EXT_MAX_BLOCKS)
		flags |= FIEMAP_EXTENT_LAST;

	ret = fiemap_fill_next_extent(fieinfo, logical, physical,
					length, flags);
	if (ret < 0)
		return ret;
	if (ret == 1)
		return EXT_BREAK;
	return EXT_CONTINUE;
}
#define EXT4_FIEMAP_FLAGS	(FIEMAP_FLAG_SYNC|FIEMAP_FLAG_XATTR)

static int ext4_xattr_fiemap(struct inode *inode,
				struct fiemap_extent_info *fieinfo)
{
	__u64 physical = 0;
	__u64 length;
	__u32 flags = FIEMAP_EXTENT_LAST;
	int blockbits = inode->i_sb->s_blocksize_bits;
	int error = 0;

	
	if (ext4_test_inode_state(inode, EXT4_STATE_XATTR)) {
		struct ext4_iloc iloc;
		int offset;	

		error = ext4_get_inode_loc(inode, &iloc);
		if (error)
			return error;
		physical = iloc.bh->b_blocknr << blockbits;
		offset = EXT4_GOOD_OLD_INODE_SIZE +
				EXT4_I(inode)->i_extra_isize;
		physical += offset;
		length = EXT4_SB(inode->i_sb)->s_inode_size - offset;
		flags |= FIEMAP_EXTENT_DATA_INLINE;
		brelse(iloc.bh);
	} else { 
		physical = EXT4_I(inode)->i_file_acl << blockbits;
		length = inode->i_sb->s_blocksize;
	}

	if (physical)
		error = fiemap_fill_next_extent(fieinfo, 0, physical,
						length, flags);
	return (error < 0 ? error : 0);
}

int ext4_ext_punch_hole(struct file *file, loff_t offset, loff_t length)
{
	struct inode *inode = file->f_path.dentry->d_inode;
	struct super_block *sb = inode->i_sb;
	ext4_lblk_t first_block, stop_block;
	struct address_space *mapping = inode->i_mapping;
	handle_t *handle;
	loff_t first_page, last_page, page_len;
	loff_t first_page_offset, last_page_offset;
	int credits, err = 0;

	
	if (offset >= inode->i_size)
		return 0;

	if (offset + length > inode->i_size) {
		length = inode->i_size +
		   PAGE_CACHE_SIZE - (inode->i_size & (PAGE_CACHE_SIZE - 1)) -
		   offset;
	}

	first_page = (offset + PAGE_CACHE_SIZE - 1) >> PAGE_CACHE_SHIFT;
	last_page = (offset + length) >> PAGE_CACHE_SHIFT;

	first_page_offset = first_page << PAGE_CACHE_SHIFT;
	last_page_offset = last_page << PAGE_CACHE_SHIFT;

	if (mapping->nrpages && mapping_tagged(mapping, PAGECACHE_TAG_DIRTY)) {
		err = filemap_write_and_wait_range(mapping,
			offset, offset + length - 1);

		if (err)
			return err;
	}

	
	if (last_page_offset > first_page_offset) {
		truncate_inode_pages_range(mapping, first_page_offset,
					   last_page_offset-1);
	}

	
	ext4_flush_completed_IO(inode);

	credits = ext4_writepage_trans_blocks(inode);
	handle = ext4_journal_start(inode, credits);
	if (IS_ERR(handle))
		return PTR_ERR(handle);

	err = ext4_orphan_add(handle, inode);
	if (err)
		goto out;

	if (first_page > last_page) {
		err = ext4_discard_partial_page_buffers(handle,
			mapping, offset, length, 0);

		if (err)
			goto out;
	} else {
		page_len  = first_page_offset - offset;
		if (page_len > 0) {
			err = ext4_discard_partial_page_buffers(handle, mapping,
						   offset, page_len, 0);
			if (err)
				goto out;
		}

		page_len = offset + length - last_page_offset;
		if (page_len > 0) {
			err = ext4_discard_partial_page_buffers(handle, mapping,
					last_page_offset, page_len, 0);
			if (err)
				goto out;
		}
	}

	if (inode->i_size >> PAGE_CACHE_SHIFT == last_page &&
	   inode->i_size % PAGE_CACHE_SIZE != 0) {

		page_len = PAGE_CACHE_SIZE -
			(inode->i_size & (PAGE_CACHE_SIZE - 1));

		if (page_len > 0) {
			err = ext4_discard_partial_page_buffers(handle,
			  mapping, inode->i_size, page_len, 0);

			if (err)
				goto out;
		}
	}

	first_block = (offset + sb->s_blocksize - 1) >>
		EXT4_BLOCK_SIZE_BITS(sb);
	stop_block = (offset + length) >> EXT4_BLOCK_SIZE_BITS(sb);

	
	if (first_block >= stop_block)
		goto out;

	down_write(&EXT4_I(inode)->i_data_sem);
	ext4_ext_invalidate_cache(inode);
	ext4_discard_preallocations(inode);

	err = ext4_ext_remove_space(inode, first_block, stop_block - 1);

	ext4_ext_invalidate_cache(inode);
	ext4_discard_preallocations(inode);

	if (IS_SYNC(inode))
		ext4_handle_sync(handle);

	up_write(&EXT4_I(inode)->i_data_sem);

out:
	ext4_orphan_del(handle, inode);
	inode->i_mtime = inode->i_ctime = ext4_current_time(inode);
	ext4_mark_inode_dirty(handle, inode);
	ext4_journal_stop(handle);
	return err;
}
int ext4_fiemap(struct inode *inode, struct fiemap_extent_info *fieinfo,
		__u64 start, __u64 len)
{
	ext4_lblk_t start_blk;
	int error = 0;

	
	if (!(ext4_test_inode_flag(inode, EXT4_INODE_EXTENTS)))
		return generic_block_fiemap(inode, fieinfo, start, len,
			ext4_get_block);

	if (fiemap_check_flags(fieinfo, EXT4_FIEMAP_FLAGS))
		return -EBADR;

	if (fieinfo->fi_flags & FIEMAP_FLAG_XATTR) {
		error = ext4_xattr_fiemap(inode, fieinfo);
	} else {
		ext4_lblk_t len_blks;
		__u64 last_blk;

		start_blk = start >> inode->i_sb->s_blocksize_bits;
		last_blk = (start + len - 1) >> inode->i_sb->s_blocksize_bits;
		if (last_blk >= EXT_MAX_BLOCKS)
			last_blk = EXT_MAX_BLOCKS-1;
		len_blks = ((ext4_lblk_t) last_blk) - start_blk + 1;

		error = ext4_ext_walk_space(inode, start_blk, len_blks,
					  ext4_ext_fiemap_cb, fieinfo);
	}

	return error;
}
