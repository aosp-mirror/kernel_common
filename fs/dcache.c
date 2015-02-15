

#include <linux/syscalls.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/fsnotify.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/hash.h>
#include <linux/cache.h>
#include <linux/export.h>
#include <linux/mount.h>
#include <linux/file.h>
#include <asm/uaccess.h>
#include <linux/security.h>
#include <linux/seqlock.h>
#include <linux/swap.h>
#include <linux/bootmem.h>
#include <linux/fs_struct.h>
#include <linux/hardirq.h>
#include <linux/bit_spinlock.h>
#include <linux/rculist_bl.h>
#include <linux/prefetch.h>
#include <linux/ratelimit.h>
#include "internal.h"
#include "mount.h"

int sysctl_vfs_cache_pressure __read_mostly = 100;
EXPORT_SYMBOL_GPL(sysctl_vfs_cache_pressure);

static __cacheline_aligned_in_smp DEFINE_SPINLOCK(dcache_lru_lock);
__cacheline_aligned_in_smp DEFINE_SEQLOCK(rename_lock);

EXPORT_SYMBOL(rename_lock);

static struct kmem_cache *dentry_cache __read_mostly;

#define D_HASHBITS     d_hash_shift
#define D_HASHMASK     d_hash_mask

static unsigned int d_hash_mask __read_mostly;
static unsigned int d_hash_shift __read_mostly;

static struct hlist_bl_head *dentry_hashtable __read_mostly;

static inline struct hlist_bl_head *d_hash(const struct dentry *parent,
					unsigned int hash)
{
	hash += (unsigned long) parent / L1_CACHE_BYTES;
	hash = hash + (hash >> D_HASHBITS);
	return dentry_hashtable + (hash & D_HASHMASK);
}

struct dentry_stat_t dentry_stat = {
	.age_limit = 45,
};

static DEFINE_PER_CPU(unsigned int, nr_dentry);

#if defined(CONFIG_SYSCTL) && defined(CONFIG_PROC_FS)
static int get_nr_dentry(void)
{
	int i;
	int sum = 0;
	for_each_possible_cpu(i)
		sum += per_cpu(nr_dentry, i);
	return sum < 0 ? 0 : sum;
}

int proc_nr_dentry(ctl_table *table, int write, void __user *buffer,
		   size_t *lenp, loff_t *ppos)
{
	dentry_stat.nr_dentry = get_nr_dentry();
	return proc_dointvec(table, write, buffer, lenp, ppos);
}
#endif

#ifdef CONFIG_DCACHE_WORD_ACCESS

#include <asm/word-at-a-time.h>
static inline int dentry_cmp(const unsigned char *cs, size_t scount,
				const unsigned char *ct, size_t tcount)
{
	unsigned long a,b,mask;

	if (unlikely(scount != tcount))
		return 1;

	for (;;) {
		a = load_unaligned_zeropad(cs);
		b = load_unaligned_zeropad(ct);
		if (tcount < sizeof(unsigned long))
			break;
		if (unlikely(a != b))
			return 1;
		cs += sizeof(unsigned long);
		ct += sizeof(unsigned long);
		tcount -= sizeof(unsigned long);
		if (!tcount)
			return 0;
	}
	mask = ~(~0ul << tcount*8);
	return unlikely(!!((a ^ b) & mask));
}

#else

static inline int dentry_cmp(const unsigned char *cs, size_t scount,
				const unsigned char *ct, size_t tcount)
{
	if (scount != tcount)
		return 1;

	do {
		if (*cs != *ct)
			return 1;
		cs++;
		ct++;
		tcount--;
	} while (tcount);
	return 0;
}

#endif

static void __d_free(struct rcu_head *head)
{
	struct dentry *dentry = container_of(head, struct dentry, d_u.d_rcu);

	WARN_ON(!list_empty(&dentry->d_alias));
	if (dname_external(dentry))
		kfree(dentry->d_name.name);
	kmem_cache_free(dentry_cache, dentry); 
}

static void d_free(struct dentry *dentry)
{
	BUG_ON(dentry->d_count);
	this_cpu_dec(nr_dentry);
	if (dentry->d_op && dentry->d_op->d_release)
		dentry->d_op->d_release(dentry);

	
	if (!(dentry->d_flags & DCACHE_RCUACCESS))
		__d_free(&dentry->d_u.d_rcu);
	else
		call_rcu(&dentry->d_u.d_rcu, __d_free);
}

static inline void dentry_rcuwalk_barrier(struct dentry *dentry)
{
	assert_spin_locked(&dentry->d_lock);
	
	write_seqcount_barrier(&dentry->d_seq);
}

static void dentry_iput(struct dentry * dentry)
	__releases(dentry->d_lock)
	__releases(dentry->d_inode->i_lock)
{
	struct inode *inode = dentry->d_inode;
	if (inode) {
		dentry->d_inode = NULL;
		list_del_init(&dentry->d_alias);
		spin_unlock(&dentry->d_lock);
		spin_unlock(&inode->i_lock);
		if (!inode->i_nlink)
			fsnotify_inoderemove(inode);
		if (dentry->d_op && dentry->d_op->d_iput)
			dentry->d_op->d_iput(dentry, inode);
		else
			iput(inode);
	} else {
		spin_unlock(&dentry->d_lock);
	}
}

static void dentry_unlink_inode(struct dentry * dentry)
	__releases(dentry->d_lock)
	__releases(dentry->d_inode->i_lock)
{
	struct inode *inode = dentry->d_inode;
	dentry->d_inode = NULL;
	list_del_init(&dentry->d_alias);
	dentry_rcuwalk_barrier(dentry);
	spin_unlock(&dentry->d_lock);
	spin_unlock(&inode->i_lock);
	if (!inode->i_nlink)
		fsnotify_inoderemove(inode);
	if (dentry->d_op && dentry->d_op->d_iput)
		dentry->d_op->d_iput(dentry, inode);
	else
		iput(inode);
}

static void dentry_lru_add(struct dentry *dentry)
{
	if (list_empty(&dentry->d_lru)) {
		spin_lock(&dcache_lru_lock);
		list_add(&dentry->d_lru, &dentry->d_sb->s_dentry_lru);
		dentry->d_sb->s_nr_dentry_unused++;
		dentry_stat.nr_unused++;
		spin_unlock(&dcache_lru_lock);
	}
}

static void __dentry_lru_del(struct dentry *dentry)
{
	list_del_init(&dentry->d_lru);
	dentry->d_flags &= ~DCACHE_SHRINK_LIST;
	dentry->d_sb->s_nr_dentry_unused--;
	dentry_stat.nr_unused--;
}

static void dentry_lru_del(struct dentry *dentry)
{
	if (!list_empty(&dentry->d_lru)) {
		spin_lock(&dcache_lru_lock);
		__dentry_lru_del(dentry);
		spin_unlock(&dcache_lru_lock);
	}
}

static void dentry_lru_prune(struct dentry *dentry)
{
	if (!list_empty(&dentry->d_lru)) {
		if (dentry->d_flags & DCACHE_OP_PRUNE)
			dentry->d_op->d_prune(dentry);

		spin_lock(&dcache_lru_lock);
		__dentry_lru_del(dentry);
		spin_unlock(&dcache_lru_lock);
	}
}

static void dentry_lru_move_list(struct dentry *dentry, struct list_head *list)
{
	spin_lock(&dcache_lru_lock);
	if (list_empty(&dentry->d_lru)) {
		list_add_tail(&dentry->d_lru, list);
		dentry->d_sb->s_nr_dentry_unused++;
		dentry_stat.nr_unused++;
	} else {
		list_move_tail(&dentry->d_lru, list);
	}
	spin_unlock(&dcache_lru_lock);
}

static struct dentry *d_kill(struct dentry *dentry, struct dentry *parent)
	__releases(dentry->d_lock)
	__releases(parent->d_lock)
	__releases(dentry->d_inode->i_lock)
{
	list_del(&dentry->d_u.d_child);
	dentry->d_flags |= DCACHE_DISCONNECTED;
	if (parent)
		spin_unlock(&parent->d_lock);
	dentry_iput(dentry);
	d_free(dentry);
	return parent;
}

static void __d_shrink(struct dentry *dentry)
{
	if (!d_unhashed(dentry)) {
		struct hlist_bl_head *b;
		if (unlikely(dentry->d_flags & DCACHE_DISCONNECTED))
			b = &dentry->d_sb->s_anon;
		else
			b = d_hash(dentry->d_parent, dentry->d_name.hash);

		hlist_bl_lock(b);
		__hlist_bl_del(&dentry->d_hash);
		dentry->d_hash.pprev = NULL;
		hlist_bl_unlock(b);
	}
}

void __d_drop(struct dentry *dentry)
{
	if (!d_unhashed(dentry)) {
		__d_shrink(dentry);
		dentry_rcuwalk_barrier(dentry);
	}
}
EXPORT_SYMBOL(__d_drop);

void d_drop(struct dentry *dentry)
{
	spin_lock(&dentry->d_lock);
	__d_drop(dentry);
	spin_unlock(&dentry->d_lock);
}
EXPORT_SYMBOL(d_drop);

void d_clear_need_lookup(struct dentry *dentry)
{
	spin_lock(&dentry->d_lock);
	__d_drop(dentry);
	dentry->d_flags &= ~DCACHE_NEED_LOOKUP;
	spin_unlock(&dentry->d_lock);
}
EXPORT_SYMBOL(d_clear_need_lookup);

static inline struct dentry *dentry_kill(struct dentry *dentry, int ref)
	__releases(dentry->d_lock)
{
	struct inode *inode;
	struct dentry *parent;

	inode = dentry->d_inode;
	if (inode && !spin_trylock(&inode->i_lock)) {
relock:
		spin_unlock(&dentry->d_lock);
		cpu_relax();
		return dentry; 
	}
	if (IS_ROOT(dentry))
		parent = NULL;
	else
		parent = dentry->d_parent;
	if (parent && !spin_trylock(&parent->d_lock)) {
		if (inode)
			spin_unlock(&inode->i_lock);
		goto relock;
	}

	if (ref)
		dentry->d_count--;
	dentry_lru_prune(dentry);
	
	__d_drop(dentry);
	return d_kill(dentry, parent);
}


void dput(struct dentry *dentry)
{
	if (!dentry)
		return;

repeat:
	if (dentry->d_count == 1)
		might_sleep();
	spin_lock(&dentry->d_lock);
	BUG_ON(!dentry->d_count);
	if (dentry->d_count > 1) {
		dentry->d_count--;
		spin_unlock(&dentry->d_lock);
		return;
	}

	if (dentry->d_flags & DCACHE_OP_DELETE) {
		if (dentry->d_op->d_delete(dentry))
			goto kill_it;
	}

	
 	if (d_unhashed(dentry))
		goto kill_it;

	if (!d_need_lookup(dentry))
		dentry->d_flags |= DCACHE_REFERENCED;
	dentry_lru_add(dentry);

	dentry->d_count--;
	spin_unlock(&dentry->d_lock);
	return;

kill_it:
	dentry = dentry_kill(dentry, 1);
	if (dentry)
		goto repeat;
}
EXPORT_SYMBOL(dput);

 
int d_invalidate(struct dentry * dentry)
{
	spin_lock(&dentry->d_lock);
	if (d_unhashed(dentry)) {
		spin_unlock(&dentry->d_lock);
		return 0;
	}
	if (!list_empty(&dentry->d_subdirs)) {
		spin_unlock(&dentry->d_lock);
		shrink_dcache_parent(dentry);
		spin_lock(&dentry->d_lock);
	}

	if (dentry->d_count > 1 && dentry->d_inode) {
		if (S_ISDIR(dentry->d_inode->i_mode) || d_mountpoint(dentry)) {
			spin_unlock(&dentry->d_lock);
			return -EBUSY;
		}
	}

	__d_drop(dentry);
	spin_unlock(&dentry->d_lock);
	return 0;
}
EXPORT_SYMBOL(d_invalidate);

static inline void __dget_dlock(struct dentry *dentry)
{
	dentry->d_count++;
}

static inline void __dget(struct dentry *dentry)
{
	spin_lock(&dentry->d_lock);
	__dget_dlock(dentry);
	spin_unlock(&dentry->d_lock);
}

struct dentry *dget_parent(struct dentry *dentry)
{
	struct dentry *ret;

repeat:
	rcu_read_lock();
	ret = dentry->d_parent;
	spin_lock(&ret->d_lock);
	if (unlikely(ret != dentry->d_parent)) {
		spin_unlock(&ret->d_lock);
		rcu_read_unlock();
		goto repeat;
	}
	rcu_read_unlock();
	BUG_ON(!ret->d_count);
	ret->d_count++;
	spin_unlock(&ret->d_lock);
	return ret;
}
EXPORT_SYMBOL(dget_parent);

static struct dentry *__d_find_alias(struct inode *inode, int want_discon)
{
	struct dentry *alias, *discon_alias;

again:
	discon_alias = NULL;
	list_for_each_entry(alias, &inode->i_dentry, d_alias) {
		spin_lock(&alias->d_lock);
 		if (S_ISDIR(inode->i_mode) || !d_unhashed(alias)) {
			if (IS_ROOT(alias) &&
			    (alias->d_flags & DCACHE_DISCONNECTED)) {
				discon_alias = alias;
			} else if (!want_discon) {
				__dget_dlock(alias);
				spin_unlock(&alias->d_lock);
				return alias;
			}
		}
		spin_unlock(&alias->d_lock);
	}
	if (discon_alias) {
		alias = discon_alias;
		spin_lock(&alias->d_lock);
		if (S_ISDIR(inode->i_mode) || !d_unhashed(alias)) {
			if (IS_ROOT(alias) &&
			    (alias->d_flags & DCACHE_DISCONNECTED)) {
				__dget_dlock(alias);
				spin_unlock(&alias->d_lock);
				return alias;
			}
		}
		spin_unlock(&alias->d_lock);
		goto again;
	}
	return NULL;
}

struct dentry *d_find_alias(struct inode *inode)
{
	struct dentry *de = NULL;

	if (!list_empty(&inode->i_dentry)) {
		spin_lock(&inode->i_lock);
		de = __d_find_alias(inode, 0);
		spin_unlock(&inode->i_lock);
	}
	return de;
}
EXPORT_SYMBOL(d_find_alias);

void d_prune_aliases(struct inode *inode)
{
	struct dentry *dentry;
restart:
	spin_lock(&inode->i_lock);
	list_for_each_entry(dentry, &inode->i_dentry, d_alias) {
		spin_lock(&dentry->d_lock);
		if (!dentry->d_count) {
			__dget_dlock(dentry);
			__d_drop(dentry);
			spin_unlock(&dentry->d_lock);
			spin_unlock(&inode->i_lock);
			dput(dentry);
			goto restart;
		}
		spin_unlock(&dentry->d_lock);
	}
	spin_unlock(&inode->i_lock);
}
EXPORT_SYMBOL(d_prune_aliases);

static void try_prune_one_dentry(struct dentry *dentry)
	__releases(dentry->d_lock)
{
	struct dentry *parent;

	parent = dentry_kill(dentry, 0);
	if (!parent)
		return;
	if (parent == dentry)
		return;

	
	dentry = parent;
	while (dentry) {
		spin_lock(&dentry->d_lock);
		if (dentry->d_count > 1) {
			dentry->d_count--;
			spin_unlock(&dentry->d_lock);
			return;
		}
		dentry = dentry_kill(dentry, 1);
	}
}

static void shrink_dentry_list(struct list_head *list)
{
	struct dentry *dentry;

	rcu_read_lock();
	for (;;) {
		dentry = list_entry_rcu(list->prev, struct dentry, d_lru);
		if (&dentry->d_lru == list)
			break; 
		spin_lock(&dentry->d_lock);
		if (dentry != list_entry(list->prev, struct dentry, d_lru)) {
			spin_unlock(&dentry->d_lock);
			continue;
		}

		if (dentry->d_count) {
			dentry_lru_del(dentry);
			spin_unlock(&dentry->d_lock);
			continue;
		}

		rcu_read_unlock();

		try_prune_one_dentry(dentry);

		rcu_read_lock();
	}
	rcu_read_unlock();
}

void prune_dcache_sb(struct super_block *sb, int count)
{
	struct dentry *dentry;
	LIST_HEAD(referenced);
	LIST_HEAD(tmp);

relock:
	spin_lock(&dcache_lru_lock);
	while (!list_empty(&sb->s_dentry_lru)) {
		dentry = list_entry(sb->s_dentry_lru.prev,
				struct dentry, d_lru);
		BUG_ON(dentry->d_sb != sb);

		if (!spin_trylock(&dentry->d_lock)) {
			spin_unlock(&dcache_lru_lock);
			cpu_relax();
			goto relock;
		}

		if (dentry->d_flags & DCACHE_REFERENCED) {
			dentry->d_flags &= ~DCACHE_REFERENCED;
			list_move(&dentry->d_lru, &referenced);
			spin_unlock(&dentry->d_lock);
		} else {
			list_move_tail(&dentry->d_lru, &tmp);
			dentry->d_flags |= DCACHE_SHRINK_LIST;
			spin_unlock(&dentry->d_lock);
			if (!--count)
				break;
		}
		cond_resched_lock(&dcache_lru_lock);
	}
	if (!list_empty(&referenced))
		list_splice(&referenced, &sb->s_dentry_lru);
	spin_unlock(&dcache_lru_lock);

	shrink_dentry_list(&tmp);
}

void shrink_dcache_sb(struct super_block *sb)
{
	LIST_HEAD(tmp);

	spin_lock(&dcache_lru_lock);
	while (!list_empty(&sb->s_dentry_lru)) {
		list_splice_init(&sb->s_dentry_lru, &tmp);
		spin_unlock(&dcache_lru_lock);
		shrink_dentry_list(&tmp);
		spin_lock(&dcache_lru_lock);
	}
	spin_unlock(&dcache_lru_lock);
}
EXPORT_SYMBOL(shrink_dcache_sb);

static void shrink_dcache_for_umount_subtree(struct dentry *dentry)
{
	struct dentry *parent;

	BUG_ON(!IS_ROOT(dentry));

	for (;;) {
		
		while (!list_empty(&dentry->d_subdirs))
			dentry = list_entry(dentry->d_subdirs.next,
					    struct dentry, d_u.d_child);

		do {
			struct inode *inode;

			dentry_lru_prune(dentry);
			__d_shrink(dentry);

			if (dentry->d_count != 0) {
				printk(KERN_ERR
				       "BUG: Dentry %p{i=%lx,n=%s}"
				       " still in use (%d)"
				       " [unmount of %s %s]\n",
				       dentry,
				       dentry->d_inode ?
				       dentry->d_inode->i_ino : 0UL,
				       dentry->d_name.name,
				       dentry->d_count,
				       dentry->d_sb->s_type->name,
				       dentry->d_sb->s_id);
				BUG();
			}

			if (IS_ROOT(dentry)) {
				parent = NULL;
				list_del(&dentry->d_u.d_child);
			} else {
				parent = dentry->d_parent;
				parent->d_count--;
				list_del(&dentry->d_u.d_child);
			}

			inode = dentry->d_inode;
			if (inode) {
				dentry->d_inode = NULL;
				list_del_init(&dentry->d_alias);
				if (dentry->d_op && dentry->d_op->d_iput)
					dentry->d_op->d_iput(dentry, inode);
				else
					iput(inode);
			}

			d_free(dentry);

			if (!parent)
				return;
			dentry = parent;
		} while (list_empty(&dentry->d_subdirs));

		dentry = list_entry(dentry->d_subdirs.next,
				    struct dentry, d_u.d_child);
	}
}

void shrink_dcache_for_umount(struct super_block *sb)
{
	struct dentry *dentry;

	if (down_read_trylock(&sb->s_umount))
		BUG();

	dentry = sb->s_root;
	sb->s_root = NULL;
	dentry->d_count--;
	shrink_dcache_for_umount_subtree(dentry);

	while (!hlist_bl_empty(&sb->s_anon)) {
		dentry = hlist_bl_entry(hlist_bl_first(&sb->s_anon), struct dentry, d_hash);
		shrink_dcache_for_umount_subtree(dentry);
	}
}

static struct dentry *try_to_ascend(struct dentry *old, int locked, unsigned seq)
{
	struct dentry *new = old->d_parent;

	rcu_read_lock();
	spin_unlock(&old->d_lock);
	spin_lock(&new->d_lock);

	if (new != old->d_parent ||
		 (old->d_flags & DCACHE_DISCONNECTED) ||
		 (!locked && read_seqretry(&rename_lock, seq))) {
		spin_unlock(&new->d_lock);
		new = NULL;
	}
	rcu_read_unlock();
	return new;
}


 
int have_submounts(struct dentry *parent)
{
	struct dentry *this_parent;
	struct list_head *next;
	unsigned seq;
	int locked = 0;

	seq = read_seqbegin(&rename_lock);
again:
	this_parent = parent;

	if (d_mountpoint(parent))
		goto positive;
	spin_lock(&this_parent->d_lock);
repeat:
	next = this_parent->d_subdirs.next;
resume:
	while (next != &this_parent->d_subdirs) {
		struct list_head *tmp = next;
		struct dentry *dentry = list_entry(tmp, struct dentry, d_u.d_child);
		next = tmp->next;

		spin_lock_nested(&dentry->d_lock, DENTRY_D_LOCK_NESTED);
		
		if (d_mountpoint(dentry)) {
			spin_unlock(&dentry->d_lock);
			spin_unlock(&this_parent->d_lock);
			goto positive;
		}
		if (!list_empty(&dentry->d_subdirs)) {
			spin_unlock(&this_parent->d_lock);
			spin_release(&dentry->d_lock.dep_map, 1, _RET_IP_);
			this_parent = dentry;
			spin_acquire(&this_parent->d_lock.dep_map, 0, 1, _RET_IP_);
			goto repeat;
		}
		spin_unlock(&dentry->d_lock);
	}
	if (this_parent != parent) {
		struct dentry *child = this_parent;
		this_parent = try_to_ascend(this_parent, locked, seq);
		if (!this_parent)
			goto rename_retry;
		next = child->d_u.d_child.next;
		goto resume;
	}
	spin_unlock(&this_parent->d_lock);
	if (!locked && read_seqretry(&rename_lock, seq))
		goto rename_retry;
	if (locked)
		write_sequnlock(&rename_lock);
	return 0; 
positive:
	if (!locked && read_seqretry(&rename_lock, seq))
		goto rename_retry;
	if (locked)
		write_sequnlock(&rename_lock);
	return 1;

rename_retry:
	locked = 1;
	write_seqlock(&rename_lock);
	goto again;
}
EXPORT_SYMBOL(have_submounts);

static int select_parent(struct dentry *parent, struct list_head *dispose)
{
	struct dentry *this_parent;
	struct list_head *next;
	unsigned seq;
	int found = 0;
	int locked = 0;

	seq = read_seqbegin(&rename_lock);
again:
	this_parent = parent;
	spin_lock(&this_parent->d_lock);
repeat:
	next = this_parent->d_subdirs.next;
resume:
	while (next != &this_parent->d_subdirs) {
		struct list_head *tmp = next;
		struct dentry *dentry = list_entry(tmp, struct dentry, d_u.d_child);
		next = tmp->next;

		spin_lock_nested(&dentry->d_lock, DENTRY_D_LOCK_NESTED);

		if (dentry->d_count) {
			dentry_lru_del(dentry);
		} else if (!(dentry->d_flags & DCACHE_SHRINK_LIST)) {
			dentry_lru_move_list(dentry, dispose);
			dentry->d_flags |= DCACHE_SHRINK_LIST;
			found++;
		}
		if (found && need_resched()) {
			spin_unlock(&dentry->d_lock);
			goto out;
		}

		if (!list_empty(&dentry->d_subdirs)) {
			spin_unlock(&this_parent->d_lock);
			spin_release(&dentry->d_lock.dep_map, 1, _RET_IP_);
			this_parent = dentry;
			spin_acquire(&this_parent->d_lock.dep_map, 0, 1, _RET_IP_);
			goto repeat;
		}

		spin_unlock(&dentry->d_lock);
	}
	if (this_parent != parent) {
		struct dentry *child = this_parent;
		this_parent = try_to_ascend(this_parent, locked, seq);
		if (!this_parent)
			goto rename_retry;
		next = child->d_u.d_child.next;
		goto resume;
	}
out:
	spin_unlock(&this_parent->d_lock);
	if (!locked && read_seqretry(&rename_lock, seq))
		goto rename_retry;
	if (locked)
		write_sequnlock(&rename_lock);
	return found;

rename_retry:
	if (found)
		return found;
	locked = 1;
	write_seqlock(&rename_lock);
	goto again;
}

void shrink_dcache_parent(struct dentry * parent)
{
	LIST_HEAD(dispose);
	int found;

	while ((found = select_parent(parent, &dispose)) != 0)
		shrink_dentry_list(&dispose);
}
EXPORT_SYMBOL(shrink_dcache_parent);

 
struct dentry *__d_alloc(struct super_block *sb, const struct qstr *name)
{
	struct dentry *dentry;
	char *dname;

	dentry = kmem_cache_alloc(dentry_cache, GFP_KERNEL);
	if (!dentry)
		return NULL;

	if (name->len > DNAME_INLINE_LEN-1) {
		dname = kmalloc(name->len + 1, GFP_KERNEL);
		if (!dname) {
			kmem_cache_free(dentry_cache, dentry); 
			return NULL;
		}
	} else  {
		dname = dentry->d_iname;
	}	
	dentry->d_name.name = dname;

	dentry->d_name.len = name->len;
	dentry->d_name.hash = name->hash;
	memcpy(dname, name->name, name->len);
	dname[name->len] = 0;

	dentry->d_count = 1;
	dentry->d_flags = 0;
	spin_lock_init(&dentry->d_lock);
	seqcount_init(&dentry->d_seq);
	dentry->d_inode = NULL;
	dentry->d_parent = dentry;
	dentry->d_sb = sb;
	dentry->d_op = NULL;
	dentry->d_fsdata = NULL;
	INIT_HLIST_BL_NODE(&dentry->d_hash);
	INIT_LIST_HEAD(&dentry->d_lru);
	INIT_LIST_HEAD(&dentry->d_subdirs);
	INIT_LIST_HEAD(&dentry->d_alias);
	INIT_LIST_HEAD(&dentry->d_u.d_child);
	d_set_d_op(dentry, dentry->d_sb->s_d_op);

	this_cpu_inc(nr_dentry);

	return dentry;
}

struct dentry *d_alloc(struct dentry * parent, const struct qstr *name)
{
	struct dentry *dentry = __d_alloc(parent->d_sb, name);
	if (!dentry)
		return NULL;

	spin_lock(&parent->d_lock);
	__dget_dlock(parent);
	dentry->d_parent = parent;
	list_add(&dentry->d_u.d_child, &parent->d_subdirs);
	spin_unlock(&parent->d_lock);

	return dentry;
}
EXPORT_SYMBOL(d_alloc);

struct dentry *d_alloc_pseudo(struct super_block *sb, const struct qstr *name)
{
	struct dentry *dentry = __d_alloc(sb, name);
	if (dentry)
		dentry->d_flags |= DCACHE_DISCONNECTED;
	return dentry;
}
EXPORT_SYMBOL(d_alloc_pseudo);

struct dentry *d_alloc_name(struct dentry *parent, const char *name)
{
	struct qstr q;

	q.name = name;
	q.len = strlen(name);
	q.hash = full_name_hash(q.name, q.len);
	return d_alloc(parent, &q);
}
EXPORT_SYMBOL(d_alloc_name);

void d_set_d_op(struct dentry *dentry, const struct dentry_operations *op)
{
	WARN_ON_ONCE(dentry->d_op);
	WARN_ON_ONCE(dentry->d_flags & (DCACHE_OP_HASH	|
				DCACHE_OP_COMPARE	|
				DCACHE_OP_REVALIDATE	|
				DCACHE_OP_DELETE ));
	dentry->d_op = op;
	if (!op)
		return;
	if (op->d_hash)
		dentry->d_flags |= DCACHE_OP_HASH;
	if (op->d_compare)
		dentry->d_flags |= DCACHE_OP_COMPARE;
	if (op->d_revalidate)
		dentry->d_flags |= DCACHE_OP_REVALIDATE;
	if (op->d_delete)
		dentry->d_flags |= DCACHE_OP_DELETE;
	if (op->d_prune)
		dentry->d_flags |= DCACHE_OP_PRUNE;

}
EXPORT_SYMBOL(d_set_d_op);

static void __d_instantiate(struct dentry *dentry, struct inode *inode)
{
	spin_lock(&dentry->d_lock);
	if (inode) {
		if (unlikely(IS_AUTOMOUNT(inode)))
			dentry->d_flags |= DCACHE_NEED_AUTOMOUNT;
		list_add(&dentry->d_alias, &inode->i_dentry);
	}
	dentry->d_inode = inode;
	dentry_rcuwalk_barrier(dentry);
	spin_unlock(&dentry->d_lock);
	fsnotify_d_instantiate(dentry, inode);
}

 
void d_instantiate(struct dentry *entry, struct inode * inode)
{
	BUG_ON(!list_empty(&entry->d_alias));
	if (inode)
		spin_lock(&inode->i_lock);
	__d_instantiate(entry, inode);
	if (inode)
		spin_unlock(&inode->i_lock);
	security_d_instantiate(entry, inode);
}
EXPORT_SYMBOL(d_instantiate);

static struct dentry *__d_instantiate_unique(struct dentry *entry,
					     struct inode *inode)
{
	struct dentry *alias;
	int len = entry->d_name.len;
	const char *name = entry->d_name.name;
	unsigned int hash = entry->d_name.hash;

	if (!inode) {
		__d_instantiate(entry, NULL);
		return NULL;
	}

	list_for_each_entry(alias, &inode->i_dentry, d_alias) {
		struct qstr *qstr = &alias->d_name;

		if (qstr->hash != hash)
			continue;
		if (alias->d_parent != entry->d_parent)
			continue;
		if (dentry_cmp(qstr->name, qstr->len, name, len))
			continue;
		__dget(alias);
		return alias;
	}

	__d_instantiate(entry, inode);
	return NULL;
}

struct dentry *d_instantiate_unique(struct dentry *entry, struct inode *inode)
{
	struct dentry *result;

	BUG_ON(!list_empty(&entry->d_alias));

	if (inode)
		spin_lock(&inode->i_lock);
	result = __d_instantiate_unique(entry, inode);
	if (inode)
		spin_unlock(&inode->i_lock);

	if (!result) {
		security_d_instantiate(entry, inode);
		return NULL;
	}

	BUG_ON(!d_unhashed(result));
	iput(inode);
	return result;
}

EXPORT_SYMBOL(d_instantiate_unique);

struct dentry *d_make_root(struct inode *root_inode)
{
	struct dentry *res = NULL;

	if (root_inode) {
		static const struct qstr name = { .name = "/", .len = 1 };

		res = __d_alloc(root_inode->i_sb, &name);
		if (res)
			d_instantiate(res, root_inode);
		else
			iput(root_inode);
	}
	return res;
}
EXPORT_SYMBOL(d_make_root);

static struct dentry * __d_find_any_alias(struct inode *inode)
{
	struct dentry *alias;

	if (list_empty(&inode->i_dentry))
		return NULL;
	alias = list_first_entry(&inode->i_dentry, struct dentry, d_alias);
	__dget(alias);
	return alias;
}

struct dentry *d_find_any_alias(struct inode *inode)
{
	struct dentry *de;

	spin_lock(&inode->i_lock);
	de = __d_find_any_alias(inode);
	spin_unlock(&inode->i_lock);
	return de;
}
EXPORT_SYMBOL(d_find_any_alias);

struct dentry *d_obtain_alias(struct inode *inode)
{
	static const struct qstr anonstring = { .name = "" };
	struct dentry *tmp;
	struct dentry *res;

	if (!inode)
		return ERR_PTR(-ESTALE);
	if (IS_ERR(inode))
		return ERR_CAST(inode);

	res = d_find_any_alias(inode);
	if (res)
		goto out_iput;

	tmp = __d_alloc(inode->i_sb, &anonstring);
	if (!tmp) {
		res = ERR_PTR(-ENOMEM);
		goto out_iput;
	}

	spin_lock(&inode->i_lock);
	res = __d_find_any_alias(inode);
	if (res) {
		spin_unlock(&inode->i_lock);
		dput(tmp);
		goto out_iput;
	}

	
	spin_lock(&tmp->d_lock);
	tmp->d_inode = inode;
	tmp->d_flags |= DCACHE_DISCONNECTED;
	list_add(&tmp->d_alias, &inode->i_dentry);
	hlist_bl_lock(&tmp->d_sb->s_anon);
	hlist_bl_add_head(&tmp->d_hash, &tmp->d_sb->s_anon);
	hlist_bl_unlock(&tmp->d_sb->s_anon);
	spin_unlock(&tmp->d_lock);
	spin_unlock(&inode->i_lock);
	security_d_instantiate(tmp, inode);

	return tmp;

 out_iput:
	if (res && !IS_ERR(res))
		security_d_instantiate(res, inode);
	iput(inode);
	return res;
}
EXPORT_SYMBOL(d_obtain_alias);

struct dentry *d_splice_alias(struct inode *inode, struct dentry *dentry)
{
	struct dentry *new = NULL;

	if (IS_ERR(inode))
		return ERR_CAST(inode);

	if (inode && S_ISDIR(inode->i_mode)) {
		spin_lock(&inode->i_lock);
		new = __d_find_alias(inode, 1);
		if (new) {
			BUG_ON(!(new->d_flags & DCACHE_DISCONNECTED));
			spin_unlock(&inode->i_lock);
			security_d_instantiate(new, inode);
			d_move(new, dentry);
			iput(inode);
		} else {
			
			__d_instantiate(dentry, inode);
			spin_unlock(&inode->i_lock);
			security_d_instantiate(dentry, inode);
			d_rehash(dentry);
		}
	} else
		d_add(dentry, inode);
	return new;
}
EXPORT_SYMBOL(d_splice_alias);

struct dentry *d_add_ci(struct dentry *dentry, struct inode *inode,
			struct qstr *name)
{
	int error;
	struct dentry *found;
	struct dentry *new;

	found = d_hash_and_lookup(dentry->d_parent, name);
	if (!found) {
		new = d_alloc(dentry->d_parent, name);
		if (!new) {
			error = -ENOMEM;
			goto err_out;
		}

		found = d_splice_alias(inode, new);
		if (found) {
			dput(new);
			return found;
		}
		return new;
	}

	if (found->d_inode) {
		if (unlikely(found->d_inode != inode)) {
			
			BUG_ON(!is_bad_inode(inode));
			BUG_ON(!is_bad_inode(found->d_inode));
		}
		iput(inode);
		return found;
	}

	if (unlikely(d_need_lookup(found)))
		d_clear_need_lookup(found);

	new = d_splice_alias(inode, found);
	if (new) {
		dput(found);
		found = new;
	}
	return found;

err_out:
	iput(inode);
	return ERR_PTR(error);
}
EXPORT_SYMBOL(d_add_ci);

struct dentry *__d_lookup_rcu(const struct dentry *parent,
				const struct qstr *name,
				unsigned *seqp, struct inode **inode)
{
	unsigned int len = name->len;
	unsigned int hash = name->hash;
	const unsigned char *str = name->name;
	struct hlist_bl_head *b = d_hash(parent, hash);
	struct hlist_bl_node *node;
	struct dentry *dentry;


	hlist_bl_for_each_entry_rcu(dentry, node, b, d_hash) {
		unsigned seq;
		struct inode *i;
		const char *tname;
		int tlen;

		if (dentry->d_name.hash != hash)
			continue;

seqretry:
		seq = read_seqcount_begin(&dentry->d_seq);
		if (dentry->d_parent != parent)
			continue;
		if (d_unhashed(dentry))
			continue;
		tlen = dentry->d_name.len;
		tname = dentry->d_name.name;
		i = dentry->d_inode;
		prefetch(tname);
		if (read_seqcount_retry(&dentry->d_seq, seq))
			goto seqretry;
		if (unlikely(parent->d_flags & DCACHE_OP_COMPARE)) {
			if (parent->d_op->d_compare(parent, *inode,
						dentry, i,
						tlen, tname, name))
				continue;
		} else {
			if (dentry_cmp(tname, tlen, str, len))
				continue;
		}
		*seqp = seq;
		*inode = i;
		return dentry;
	}
	return NULL;
}

struct dentry *d_lookup(struct dentry *parent, struct qstr *name)
{
	struct dentry *dentry;
	unsigned seq;

        do {
                seq = read_seqbegin(&rename_lock);
                dentry = __d_lookup(parent, name);
                if (dentry)
			break;
	} while (read_seqretry(&rename_lock, seq));
	return dentry;
}
EXPORT_SYMBOL(d_lookup);

struct dentry *__d_lookup(struct dentry *parent, struct qstr *name)
{
	unsigned int len = name->len;
	unsigned int hash = name->hash;
	const unsigned char *str = name->name;
	struct hlist_bl_head *b = d_hash(parent, hash);
	struct hlist_bl_node *node;
	struct dentry *found = NULL;
	struct dentry *dentry;


	rcu_read_lock();
	
	hlist_bl_for_each_entry_rcu(dentry, node, b, d_hash) {
		const char *tname;
		int tlen;

		if (dentry->d_name.hash != hash)
			continue;

		spin_lock(&dentry->d_lock);
		if (dentry->d_parent != parent)
			goto next;
		if (d_unhashed(dentry))
			goto next;

		tlen = dentry->d_name.len;
		tname = dentry->d_name.name;
		if (parent->d_flags & DCACHE_OP_COMPARE) {
			if (parent->d_op->d_compare(parent, parent->d_inode,
						dentry, dentry->d_inode,
						tlen, tname, name))
				goto next;
		} else {
			if (dentry_cmp(tname, tlen, str, len))
				goto next;
		}

		dentry->d_count++;
		found = dentry;
		spin_unlock(&dentry->d_lock);
		break;
next:
		spin_unlock(&dentry->d_lock);
 	}
 	rcu_read_unlock();

 	return found;
}

struct dentry *d_hash_and_lookup(struct dentry *dir, struct qstr *name)
{
	struct dentry *dentry = NULL;

	name->hash = full_name_hash(name->name, name->len);
	if (dir->d_flags & DCACHE_OP_HASH) {
		if (dir->d_op->d_hash(dir, dir->d_inode, name) < 0)
			goto out;
	}
	dentry = d_lookup(dir, name);
out:
	return dentry;
}

int d_validate(struct dentry *dentry, struct dentry *dparent)
{
	struct dentry *child;

	spin_lock(&dparent->d_lock);
	list_for_each_entry(child, &dparent->d_subdirs, d_u.d_child) {
		if (dentry == child) {
			spin_lock_nested(&dentry->d_lock, DENTRY_D_LOCK_NESTED);
			__dget_dlock(dentry);
			spin_unlock(&dentry->d_lock);
			spin_unlock(&dparent->d_lock);
			return 1;
		}
	}
	spin_unlock(&dparent->d_lock);

	return 0;
}
EXPORT_SYMBOL(d_validate);

 
 
void d_delete(struct dentry * dentry)
{
	struct inode *inode;
	int isdir = 0;
again:
	spin_lock(&dentry->d_lock);
	inode = dentry->d_inode;
	isdir = S_ISDIR(inode->i_mode);
	if (dentry->d_count == 1) {
		if (inode && !spin_trylock(&inode->i_lock)) {
			spin_unlock(&dentry->d_lock);
			cpu_relax();
			goto again;
		}
		dentry->d_flags &= ~DCACHE_CANT_MOUNT;
		dentry_unlink_inode(dentry);
		fsnotify_nameremove(dentry, isdir);
		return;
	}

	if (!d_unhashed(dentry))
		__d_drop(dentry);

	spin_unlock(&dentry->d_lock);

	fsnotify_nameremove(dentry, isdir);
}
EXPORT_SYMBOL(d_delete);

static void __d_rehash(struct dentry * entry, struct hlist_bl_head *b)
{
	WARN_ON(!d_unhashed(entry));
	if (!d_unhashed(entry))
		return;

	hlist_bl_lock(b);
	entry->d_flags |= DCACHE_RCUACCESS;
	hlist_bl_add_head_rcu(&entry->d_hash, b);
	hlist_bl_unlock(b);
}

static void _d_rehash(struct dentry * entry)
{
	__d_rehash(entry, d_hash(entry->d_parent, entry->d_name.hash));
}

 
void d_rehash(struct dentry * entry)
{
	spin_lock(&entry->d_lock);
	_d_rehash(entry);
	spin_unlock(&entry->d_lock);
}
EXPORT_SYMBOL(d_rehash);

void dentry_update_name_case(struct dentry *dentry, struct qstr *name)
{
	BUG_ON(!mutex_is_locked(&dentry->d_parent->d_inode->i_mutex));
	BUG_ON(dentry->d_name.len != name->len); 

	spin_lock(&dentry->d_lock);
	write_seqcount_begin(&dentry->d_seq);
	memcpy((unsigned char *)dentry->d_name.name, name->name, name->len);
	write_seqcount_end(&dentry->d_seq);
	spin_unlock(&dentry->d_lock);
}
EXPORT_SYMBOL(dentry_update_name_case);

static void switch_names(struct dentry *dentry, struct dentry *target)
{
	if (dname_external(target)) {
		if (dname_external(dentry)) {
			swap(target->d_name.name, dentry->d_name.name);
		} else {
			memcpy(target->d_iname, dentry->d_name.name,
					dentry->d_name.len + 1);
			dentry->d_name.name = target->d_name.name;
			target->d_name.name = target->d_iname;
		}
	} else {
		if (dname_external(dentry)) {
			memcpy(dentry->d_iname, target->d_name.name,
					target->d_name.len + 1);
			target->d_name.name = dentry->d_name.name;
			dentry->d_name.name = dentry->d_iname;
		} else {
			memcpy(dentry->d_iname, target->d_name.name,
					target->d_name.len + 1);
			dentry->d_name.len = target->d_name.len;
			return;
		}
	}
	swap(dentry->d_name.len, target->d_name.len);
}

static void dentry_lock_for_move(struct dentry *dentry, struct dentry *target)
{
	if (IS_ROOT(dentry) || dentry->d_parent == target->d_parent)
		spin_lock(&target->d_parent->d_lock);
	else {
		if (d_ancestor(dentry->d_parent, target->d_parent)) {
			spin_lock(&dentry->d_parent->d_lock);
			spin_lock_nested(&target->d_parent->d_lock,
						DENTRY_D_LOCK_NESTED);
		} else {
			spin_lock(&target->d_parent->d_lock);
			spin_lock_nested(&dentry->d_parent->d_lock,
						DENTRY_D_LOCK_NESTED);
		}
	}
	if (target < dentry) {
		spin_lock_nested(&target->d_lock, 2);
		spin_lock_nested(&dentry->d_lock, 3);
	} else {
		spin_lock_nested(&dentry->d_lock, 2);
		spin_lock_nested(&target->d_lock, 3);
	}
}

static void dentry_unlock_parents_for_move(struct dentry *dentry,
					struct dentry *target)
{
	if (target->d_parent != dentry->d_parent)
		spin_unlock(&dentry->d_parent->d_lock);
	if (target->d_parent != target)
		spin_unlock(&target->d_parent->d_lock);
}

static void __d_move(struct dentry * dentry, struct dentry * target)
{
	if (!dentry->d_inode)
		printk(KERN_WARNING "VFS: moving negative dcache entry\n");

	BUG_ON(d_ancestor(dentry, target));
	BUG_ON(d_ancestor(target, dentry));

	dentry_lock_for_move(dentry, target);

	write_seqcount_begin(&dentry->d_seq);
	write_seqcount_begin(&target->d_seq);

	

	__d_drop(dentry);
	__d_rehash(dentry, d_hash(target->d_parent, target->d_name.hash));

	
	__d_drop(target);

	list_del(&dentry->d_u.d_child);
	list_del(&target->d_u.d_child);

	
	switch_names(dentry, target);
	swap(dentry->d_name.hash, target->d_name.hash);

	
	if (IS_ROOT(dentry)) {
		dentry->d_parent = target->d_parent;
		target->d_parent = target;
		INIT_LIST_HEAD(&target->d_u.d_child);
	} else {
		swap(dentry->d_parent, target->d_parent);

		
		list_add(&target->d_u.d_child, &target->d_parent->d_subdirs);
	}

	list_add(&dentry->d_u.d_child, &dentry->d_parent->d_subdirs);

	write_seqcount_end(&target->d_seq);
	write_seqcount_end(&dentry->d_seq);

	dentry_unlock_parents_for_move(dentry, target);
	spin_unlock(&target->d_lock);
	fsnotify_d_move(dentry);
	spin_unlock(&dentry->d_lock);
}

void d_move(struct dentry *dentry, struct dentry *target)
{
	write_seqlock(&rename_lock);
	__d_move(dentry, target);
	write_sequnlock(&rename_lock);
}
EXPORT_SYMBOL(d_move);

struct dentry *d_ancestor(struct dentry *p1, struct dentry *p2)
{
	struct dentry *p;

	for (p = p2; !IS_ROOT(p); p = p->d_parent) {
		if (p->d_parent == p1)
			return p;
	}
	return NULL;
}

static struct dentry *__d_unalias(struct inode *inode,
		struct dentry *dentry, struct dentry *alias)
{
	struct mutex *m1 = NULL, *m2 = NULL;
	struct dentry *ret;

	
	if (alias->d_parent == dentry->d_parent)
		goto out_unalias;

	
	ret = ERR_PTR(-EBUSY);
	if (!mutex_trylock(&dentry->d_sb->s_vfs_rename_mutex))
		goto out_err;
	m1 = &dentry->d_sb->s_vfs_rename_mutex;
	if (!mutex_trylock(&alias->d_parent->d_inode->i_mutex))
		goto out_err;
	m2 = &alias->d_parent->d_inode->i_mutex;
out_unalias:
	__d_move(alias, dentry);
	ret = alias;
out_err:
	spin_unlock(&inode->i_lock);
	if (m2)
		mutex_unlock(m2);
	if (m1)
		mutex_unlock(m1);
	return ret;
}

static void __d_materialise_dentry(struct dentry *dentry, struct dentry *anon)
{
	struct dentry *dparent, *aparent;

	dentry_lock_for_move(anon, dentry);

	write_seqcount_begin(&dentry->d_seq);
	write_seqcount_begin(&anon->d_seq);

	dparent = dentry->d_parent;
	aparent = anon->d_parent;

	switch_names(dentry, anon);
	swap(dentry->d_name.hash, anon->d_name.hash);

	dentry->d_parent = (aparent == anon) ? dentry : aparent;
	list_del(&dentry->d_u.d_child);
	if (!IS_ROOT(dentry))
		list_add(&dentry->d_u.d_child, &dentry->d_parent->d_subdirs);
	else
		INIT_LIST_HEAD(&dentry->d_u.d_child);

	anon->d_parent = (dparent == dentry) ? anon : dparent;
	list_del(&anon->d_u.d_child);
	if (!IS_ROOT(anon))
		list_add(&anon->d_u.d_child, &anon->d_parent->d_subdirs);
	else
		INIT_LIST_HEAD(&anon->d_u.d_child);

	write_seqcount_end(&dentry->d_seq);
	write_seqcount_end(&anon->d_seq);

	dentry_unlock_parents_for_move(anon, dentry);
	spin_unlock(&dentry->d_lock);

	
	anon->d_flags &= ~DCACHE_DISCONNECTED;
}

struct dentry *d_materialise_unique(struct dentry *dentry, struct inode *inode)
{
	struct dentry *actual;

	BUG_ON(!d_unhashed(dentry));

	if (!inode) {
		actual = dentry;
		__d_instantiate(dentry, NULL);
		d_rehash(actual);
		goto out_nolock;
	}

	spin_lock(&inode->i_lock);

	if (S_ISDIR(inode->i_mode)) {
		struct dentry *alias;

		
		alias = __d_find_alias(inode, 0);
		if (alias) {
			actual = alias;
			write_seqlock(&rename_lock);

			if (d_ancestor(alias, dentry)) {
				
				actual = ERR_PTR(-ELOOP);
				spin_unlock(&inode->i_lock);
			} else if (IS_ROOT(alias)) {
				__d_materialise_dentry(dentry, alias);
				write_sequnlock(&rename_lock);
				__d_drop(alias);
				goto found;
			} else {
				actual = __d_unalias(inode, dentry, alias);
			}
			write_sequnlock(&rename_lock);
			if (IS_ERR(actual)) {
				if (PTR_ERR(actual) == -ELOOP)
					pr_warn_ratelimited(
						"VFS: Lookup of '%s' in %s %s"
						" would have caused loop\n",
						dentry->d_name.name,
						inode->i_sb->s_type->name,
						inode->i_sb->s_id);
				dput(alias);
			}
			goto out_nolock;
		}
	}

	
	actual = __d_instantiate_unique(dentry, inode);
	if (!actual)
		actual = dentry;
	else
		BUG_ON(!d_unhashed(actual));

	spin_lock(&actual->d_lock);
found:
	_d_rehash(actual);
	spin_unlock(&actual->d_lock);
	spin_unlock(&inode->i_lock);
out_nolock:
	if (actual == dentry) {
		security_d_instantiate(dentry, inode);
		return NULL;
	}

	iput(inode);
	return actual;
}
EXPORT_SYMBOL_GPL(d_materialise_unique);

static int prepend(char **buffer, int *buflen, const char *str, int namelen)
{
	*buflen -= namelen;
	if (*buflen < 0)
		return -ENAMETOOLONG;
	*buffer -= namelen;
	memcpy(*buffer, str, namelen);
	return 0;
}

static int prepend_name(char **buffer, int *buflen, struct qstr *name)
{
	return prepend(buffer, buflen, name->name, name->len);
}

static int prepend_path(const struct path *path,
			const struct path *root,
			char **buffer, int *buflen)
{
	struct dentry *dentry = path->dentry;
	struct vfsmount *vfsmnt = path->mnt;
	struct mount *mnt = real_mount(vfsmnt);
	bool slash = false;
	int error = 0;

	br_read_lock(vfsmount_lock);
	while (dentry != root->dentry || vfsmnt != root->mnt) {
		struct dentry * parent;

		if (dentry == vfsmnt->mnt_root || IS_ROOT(dentry)) {
			
			if (!mnt_has_parent(mnt))
				goto global_root;
			dentry = mnt->mnt_mountpoint;
			mnt = mnt->mnt_parent;
			vfsmnt = &mnt->mnt;
			continue;
		}
		parent = dentry->d_parent;
		prefetch(parent);
		spin_lock(&dentry->d_lock);
		error = prepend_name(buffer, buflen, &dentry->d_name);
		spin_unlock(&dentry->d_lock);
		if (!error)
			error = prepend(buffer, buflen, "/", 1);
		if (error)
			break;

		slash = true;
		dentry = parent;
	}

	if (!error && !slash)
		error = prepend(buffer, buflen, "/", 1);

out:
	br_read_unlock(vfsmount_lock);
	return error;

global_root:
	if (IS_ROOT(dentry) &&
	    (dentry->d_name.len != 1 || dentry->d_name.name[0] != '/')) {
		WARN(1, "Root dentry has weird name <%.*s>\n",
		     (int) dentry->d_name.len, dentry->d_name.name);
	}
	if (!slash)
		error = prepend(buffer, buflen, "/", 1);
	if (!error)
		error = real_mount(vfsmnt)->mnt_ns ? 1 : 2;
	goto out;
}

char *__d_path(const struct path *path,
	       const struct path *root,
	       char *buf, int buflen)
{
	char *res = buf + buflen;
	int error;

	prepend(&res, &buflen, "\0", 1);
	write_seqlock(&rename_lock);
	error = prepend_path(path, root, &res, &buflen);
	write_sequnlock(&rename_lock);

	if (error < 0)
		return ERR_PTR(error);
	if (error > 0)
		return NULL;
	return res;
}

char *d_absolute_path(const struct path *path,
	       char *buf, int buflen)
{
	struct path root = {};
	char *res = buf + buflen;
	int error;

	prepend(&res, &buflen, "\0", 1);
	write_seqlock(&rename_lock);
	error = prepend_path(path, &root, &res, &buflen);
	write_sequnlock(&rename_lock);

	if (error > 1)
		error = -EINVAL;
	if (error < 0)
		return ERR_PTR(error);
	return res;
}

static int path_with_deleted(const struct path *path,
			     const struct path *root,
			     char **buf, int *buflen)
{
	prepend(buf, buflen, "\0", 1);
	if (d_unlinked(path->dentry)) {
		int error = prepend(buf, buflen, " (deleted)", 10);
		if (error)
			return error;
	}

	return prepend_path(path, root, buf, buflen);
}

static int prepend_unreachable(char **buffer, int *buflen)
{
	return prepend(buffer, buflen, "(unreachable)", 13);
}

char *d_path(const struct path *path, char *buf, int buflen)
{
	char *res = buf + buflen;
	struct path root;
	int error;

	if (path->dentry->d_op && path->dentry->d_op->d_dname)
		return path->dentry->d_op->d_dname(path->dentry, buf, buflen);

	get_fs_root(current->fs, &root);
	write_seqlock(&rename_lock);
	error = path_with_deleted(path, &root, &res, &buflen);
	if (error < 0)
		res = ERR_PTR(error);
	write_sequnlock(&rename_lock);
	path_put(&root);
	return res;
}
EXPORT_SYMBOL(d_path);

char *d_path_with_unreachable(const struct path *path, char *buf, int buflen)
{
	char *res = buf + buflen;
	struct path root;
	int error;

	if (path->dentry->d_op && path->dentry->d_op->d_dname)
		return path->dentry->d_op->d_dname(path->dentry, buf, buflen);

	get_fs_root(current->fs, &root);
	write_seqlock(&rename_lock);
	error = path_with_deleted(path, &root, &res, &buflen);
	if (error > 0)
		error = prepend_unreachable(&res, &buflen);
	write_sequnlock(&rename_lock);
	path_put(&root);
	if (error)
		res =  ERR_PTR(error);

	return res;
}

char *dynamic_dname(struct dentry *dentry, char *buffer, int buflen,
			const char *fmt, ...)
{
	va_list args;
	char temp[64];
	int sz;

	va_start(args, fmt);
	sz = vsnprintf(temp, sizeof(temp), fmt, args) + 1;
	va_end(args);

	if (sz > sizeof(temp) || sz > buflen)
		return ERR_PTR(-ENAMETOOLONG);

	buffer += buflen - sz;
	return memcpy(buffer, temp, sz);
}

static char *__dentry_path(struct dentry *dentry, char *buf, int buflen)
{
	char *end = buf + buflen;
	char *retval;

	prepend(&end, &buflen, "\0", 1);
	if (buflen < 1)
		goto Elong;
	
	retval = end-1;
	*retval = '/';

	while (!IS_ROOT(dentry)) {
		struct dentry *parent = dentry->d_parent;
		int error;

		prefetch(parent);
		spin_lock(&dentry->d_lock);
		error = prepend_name(&end, &buflen, &dentry->d_name);
		spin_unlock(&dentry->d_lock);
		if (error != 0 || prepend(&end, &buflen, "/", 1) != 0)
			goto Elong;

		retval = end;
		dentry = parent;
	}
	return retval;
Elong:
	return ERR_PTR(-ENAMETOOLONG);
}

char *dentry_path_raw(struct dentry *dentry, char *buf, int buflen)
{
	char *retval;

	write_seqlock(&rename_lock);
	retval = __dentry_path(dentry, buf, buflen);
	write_sequnlock(&rename_lock);

	return retval;
}
EXPORT_SYMBOL(dentry_path_raw);

char *dentry_path(struct dentry *dentry, char *buf, int buflen)
{
	char *p = NULL;
	char *retval;

	write_seqlock(&rename_lock);
	if (d_unlinked(dentry)) {
		p = buf + buflen;
		if (prepend(&p, &buflen, "//deleted", 10) != 0)
			goto Elong;
		buflen++;
	}
	retval = __dentry_path(dentry, buf, buflen);
	write_sequnlock(&rename_lock);
	if (!IS_ERR(retval) && p)
		*p = '/';	
	return retval;
Elong:
	return ERR_PTR(-ENAMETOOLONG);
}

SYSCALL_DEFINE2(getcwd, char __user *, buf, unsigned long, size)
{
	int error;
	struct path pwd, root;
	char *page = (char *) __get_free_page(GFP_USER);

	if (!page)
		return -ENOMEM;

	get_fs_root_and_pwd(current->fs, &root, &pwd);

	error = -ENOENT;
	write_seqlock(&rename_lock);
	if (!d_unlinked(pwd.dentry)) {
		unsigned long len;
		char *cwd = page + PAGE_SIZE;
		int buflen = PAGE_SIZE;

		prepend(&cwd, &buflen, "\0", 1);
		error = prepend_path(&pwd, &root, &cwd, &buflen);
		write_sequnlock(&rename_lock);

		if (error < 0)
			goto out;

		
		if (error > 0) {
			error = prepend_unreachable(&cwd, &buflen);
			if (error)
				goto out;
		}

		error = -ERANGE;
		len = PAGE_SIZE + page - cwd;
		if (len <= size) {
			error = len;
			if (copy_to_user(buf, cwd, len))
				error = -EFAULT;
		}
	} else {
		write_sequnlock(&rename_lock);
	}

out:
	path_put(&pwd);
	path_put(&root);
	free_page((unsigned long) page);
	return error;
}


  
int is_subdir(struct dentry *new_dentry, struct dentry *old_dentry)
{
	int result;
	unsigned seq;

	if (new_dentry == old_dentry)
		return 1;

	do {
		
		seq = read_seqbegin(&rename_lock);
		rcu_read_lock();
		if (d_ancestor(old_dentry, new_dentry))
			result = 1;
		else
			result = 0;
		rcu_read_unlock();
	} while (read_seqretry(&rename_lock, seq));

	return result;
}

void d_genocide(struct dentry *root)
{
	struct dentry *this_parent;
	struct list_head *next;
	unsigned seq;
	int locked = 0;

	seq = read_seqbegin(&rename_lock);
again:
	this_parent = root;
	spin_lock(&this_parent->d_lock);
repeat:
	next = this_parent->d_subdirs.next;
resume:
	while (next != &this_parent->d_subdirs) {
		struct list_head *tmp = next;
		struct dentry *dentry = list_entry(tmp, struct dentry, d_u.d_child);
		next = tmp->next;

		spin_lock_nested(&dentry->d_lock, DENTRY_D_LOCK_NESTED);
		if (d_unhashed(dentry) || !dentry->d_inode) {
			spin_unlock(&dentry->d_lock);
			continue;
		}
		if (!list_empty(&dentry->d_subdirs)) {
			spin_unlock(&this_parent->d_lock);
			spin_release(&dentry->d_lock.dep_map, 1, _RET_IP_);
			this_parent = dentry;
			spin_acquire(&this_parent->d_lock.dep_map, 0, 1, _RET_IP_);
			goto repeat;
		}
		if (!(dentry->d_flags & DCACHE_GENOCIDE)) {
			dentry->d_flags |= DCACHE_GENOCIDE;
			dentry->d_count--;
		}
		spin_unlock(&dentry->d_lock);
	}
	if (this_parent != root) {
		struct dentry *child = this_parent;
		if (!(this_parent->d_flags & DCACHE_GENOCIDE)) {
			this_parent->d_flags |= DCACHE_GENOCIDE;
			this_parent->d_count--;
		}
		this_parent = try_to_ascend(this_parent, locked, seq);
		if (!this_parent)
			goto rename_retry;
		next = child->d_u.d_child.next;
		goto resume;
	}
	spin_unlock(&this_parent->d_lock);
	if (!locked && read_seqretry(&rename_lock, seq))
		goto rename_retry;
	if (locked)
		write_sequnlock(&rename_lock);
	return;

rename_retry:
	locked = 1;
	write_seqlock(&rename_lock);
	goto again;
}

 
ino_t find_inode_number(struct dentry *dir, struct qstr *name)
{
	struct dentry * dentry;
	ino_t ino = 0;

	dentry = d_hash_and_lookup(dir, name);
	if (dentry) {
		if (dentry->d_inode)
			ino = dentry->d_inode->i_ino;
		dput(dentry);
	}
	return ino;
}
EXPORT_SYMBOL(find_inode_number);

static __initdata unsigned long dhash_entries;
static int __init set_dhash_entries(char *str)
{
	if (!str)
		return 0;
	dhash_entries = simple_strtoul(str, &str, 0);
	return 1;
}
__setup("dhash_entries=", set_dhash_entries);

static void __init dcache_init_early(void)
{
	unsigned int loop;

	if (hashdist)
		return;

	dentry_hashtable =
		alloc_large_system_hash("Dentry cache",
					sizeof(struct hlist_bl_head),
					dhash_entries,
					13,
					HASH_EARLY,
					&d_hash_shift,
					&d_hash_mask,
					0);

	for (loop = 0; loop < (1U << d_hash_shift); loop++)
		INIT_HLIST_BL_HEAD(dentry_hashtable + loop);
}

static void __init dcache_init(void)
{
	unsigned int loop;

	dentry_cache = KMEM_CACHE(dentry,
		SLAB_RECLAIM_ACCOUNT|SLAB_PANIC|SLAB_MEM_SPREAD);

	
	if (!hashdist)
		return;

	dentry_hashtable =
		alloc_large_system_hash("Dentry cache",
					sizeof(struct hlist_bl_head),
					dhash_entries,
					13,
					0,
					&d_hash_shift,
					&d_hash_mask,
					0);

	for (loop = 0; loop < (1U << d_hash_shift); loop++)
		INIT_HLIST_BL_HEAD(dentry_hashtable + loop);
}

struct kmem_cache *names_cachep __read_mostly;
EXPORT_SYMBOL(names_cachep);

EXPORT_SYMBOL(d_genocide);

void __init vfs_caches_init_early(void)
{
	dcache_init_early();
	inode_init_early();
}

void __init vfs_caches_init(unsigned long mempages)
{
	unsigned long reserve;


	reserve = min((mempages - nr_free_pages()) * 3/2, mempages - 1);
	mempages -= reserve;

	names_cachep = kmem_cache_create("names_cache", PATH_MAX, 0,
			SLAB_HWCACHE_ALIGN|SLAB_PANIC, NULL);

	dcache_init();
	inode_init();
	files_init(mempages);
	mnt_init();
	bdev_cache_init();
	chrdev_init();
}
