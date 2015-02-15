/*
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation; either version
 *   2 of the License, or (at your option) any later version.
 *
 *   Robert Olsson <robert.olsson@its.uu.se> Uppsala Universitet
 *     & Swedish University of Agricultural Sciences.
 *
 *   Jens Laas <jens.laas@data.slu.se> Swedish University of
 *     Agricultural Sciences.
 *
 *   Hans Liss <hans.liss@its.uu.se>  Uppsala Universitet
 *
 * This work is based on the LPC-trie which is originally described in:
 *
 * An experimental study of compression methods for dynamic tries
 * Stefan Nilsson and Matti Tikkanen. Algorithmica, 33(1):19-33, 2002.
 * http://www.csc.kth.se/~snilsson/software/dyntrie2/
 *
 *
 * IP-address lookup using LC-tries. Stefan Nilsson and Gunnar Karlsson
 * IEEE Journal on Selected Areas in Communications, 17(6):1083-1092, June 1999
 *
 *
 * Code from fib_hash has been reused which includes the following header:
 *
 *
 * INET		An implementation of the TCP/IP protocol suite for the LINUX
 *		operating system.  INET is implemented using the  BSD Socket
 *		interface as the means of communication with the user level.
 *
 *		IPv4 FIB: lookup engine and maintenance routines.
 *
 *
 * Authors:	Alexey Kuznetsov, <kuznet@ms2.inr.ac.ru>
 *
 *		This program is free software; you can redistribute it and/or
 *		modify it under the terms of the GNU General Public License
 *		as published by the Free Software Foundation; either version
 *		2 of the License, or (at your option) any later version.
 *
 * Substantial contributions to this work comes from:
 *
 *		David S. Miller, <davem@davemloft.net>
 *		Stephen Hemminger <shemminger@osdl.org>
 *		Paul E. McKenney <paulmck@us.ibm.com>
 *		Patrick McHardy <kaber@trash.net>
 */

#define VERSION "0.409"

#include <asm/uaccess.h>
#include <linux/bitops.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/socket.h>
#include <linux/sockios.h>
#include <linux/errno.h>
#include <linux/in.h>
#include <linux/inet.h>
#include <linux/inetdevice.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/proc_fs.h>
#include <linux/rcupdate.h>
#include <linux/skbuff.h>
#include <linux/netlink.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/prefetch.h>
#include <linux/export.h>
#include <net/net_namespace.h>
#include <net/ip.h>
#include <net/protocol.h>
#include <net/route.h>
#include <net/tcp.h>
#include <net/sock.h>
#include <net/ip_fib.h>
#include "fib_lookup.h"
#ifdef CONFIG_HTC_NETWORK_MODIFY
#include <linux/uaccess.h>
#endif

#define MAX_STAT_DEPTH 32

#define KEYLENGTH (8*sizeof(t_key))

typedef unsigned int t_key;

#define T_TNODE 0
#define T_LEAF  1
#define NODE_TYPE_MASK	0x1UL
#define NODE_TYPE(node) ((node)->parent & NODE_TYPE_MASK)

#define IS_TNODE(n) (!(n->parent & T_LEAF))
#define IS_LEAF(n) (n->parent & T_LEAF)

struct rt_trie_node {
	unsigned long parent;
	t_key key;
};

struct leaf {
	unsigned long parent;
	t_key key;
	struct hlist_head list;
	struct rcu_head rcu;
};

struct leaf_info {
	struct hlist_node hlist;
	int plen;
	u32 mask_plen; 
	struct list_head falh;
	struct rcu_head rcu;
};

struct tnode {
	unsigned long parent;
	t_key key;
	unsigned char pos;		
	unsigned char bits;		
	unsigned int full_children;	
	unsigned int empty_children;	
	union {
		struct rcu_head rcu;
		struct work_struct work;
		struct tnode *tnode_free;
	};
	struct rt_trie_node __rcu *child[0];
};

#ifdef CONFIG_IP_FIB_TRIE_STATS
struct trie_use_stats {
	unsigned int gets;
	unsigned int backtrack;
	unsigned int semantic_match_passed;
	unsigned int semantic_match_miss;
	unsigned int null_node_hit;
	unsigned int resize_node_skipped;
};
#endif

struct trie_stat {
	unsigned int totdepth;
	unsigned int maxdepth;
	unsigned int tnodes;
	unsigned int leaves;
	unsigned int nullpointers;
	unsigned int prefixes;
	unsigned int nodesizes[MAX_STAT_DEPTH];
};

struct trie {
	struct rt_trie_node __rcu *trie;
#ifdef CONFIG_IP_FIB_TRIE_STATS
	struct trie_use_stats stats;
#endif
};

static void put_child(struct trie *t, struct tnode *tn, int i, struct rt_trie_node *n);
static void tnode_put_child_reorg(struct tnode *tn, int i, struct rt_trie_node *n,
				  int wasfull);
static struct rt_trie_node *resize(struct trie *t, struct tnode *tn);
static struct tnode *inflate(struct trie *t, struct tnode *tn);
static struct tnode *halve(struct trie *t, struct tnode *tn);
static struct tnode *tnode_free_head;
static size_t tnode_free_size;

static const int sync_pages = 128;

static struct kmem_cache *fn_alias_kmem __read_mostly;
static struct kmem_cache *trie_leaf_kmem __read_mostly;

static inline struct tnode *node_parent(const struct rt_trie_node *node)
{
	unsigned long parent;

	parent = rcu_dereference_index_check(node->parent, lockdep_rtnl_is_held());

	return (struct tnode *)(parent & ~NODE_TYPE_MASK);
}

static inline struct tnode *node_parent_rcu(const struct rt_trie_node *node)
{
	unsigned long parent;

	parent = rcu_dereference_index_check(node->parent, rcu_read_lock_held() ||
							   lockdep_rtnl_is_held());

	return (struct tnode *)(parent & ~NODE_TYPE_MASK);
}

static inline void node_set_parent(struct rt_trie_node *node, struct tnode *ptr)
{
	smp_wmb();
	node->parent = (unsigned long)ptr | NODE_TYPE(node);
}

static inline struct rt_trie_node *tnode_get_child(const struct tnode *tn, unsigned int i)
{
	BUG_ON(i >= 1U << tn->bits);

	return rtnl_dereference(tn->child[i]);
}

static inline struct rt_trie_node *tnode_get_child_rcu(const struct tnode *tn, unsigned int i)
{
	BUG_ON(i >= 1U << tn->bits);

	return rcu_dereference_rtnl(tn->child[i]);
}

static inline int tnode_child_length(const struct tnode *tn)
{
	return 1 << tn->bits;
}

static inline t_key mask_pfx(t_key k, unsigned int l)
{
	return (l == 0) ? 0 : k >> (KEYLENGTH-l) << (KEYLENGTH-l);
}

static inline t_key tkey_extract_bits(t_key a, unsigned int offset, unsigned int bits)
{
	if (offset < KEYLENGTH)
		return ((t_key)(a << offset)) >> (KEYLENGTH - bits);
	else
		return 0;
}

static inline int tkey_equals(t_key a, t_key b)
{
	return a == b;
}

static inline int tkey_sub_equals(t_key a, int offset, int bits, t_key b)
{
	if (bits == 0 || offset >= KEYLENGTH)
		return 1;
	bits = bits > KEYLENGTH ? KEYLENGTH : bits;
	return ((a ^ b) << offset) >> (KEYLENGTH - bits) == 0;
}

static inline int tkey_mismatch(t_key a, int offset, t_key b)
{
	t_key diff = a ^ b;
	int i = offset;

	if (!diff)
		return 0;
	while ((diff << i) >> (KEYLENGTH-1) == 0)
		i++;
	return i;
}


static inline void check_tnode(const struct tnode *tn)
{
	WARN_ON(tn && tn->pos+tn->bits > 32);
}

static const int halve_threshold = 25;
static const int inflate_threshold = 50;
static const int halve_threshold_root = 15;
static const int inflate_threshold_root = 30;

static void __alias_free_mem(struct rcu_head *head)
{
	struct fib_alias *fa = container_of(head, struct fib_alias, rcu);
	kmem_cache_free(fn_alias_kmem, fa);
}

static inline void alias_free_mem_rcu(struct fib_alias *fa)
{
	call_rcu(&fa->rcu, __alias_free_mem);
}

static void __leaf_free_rcu(struct rcu_head *head)
{
	struct leaf *l = container_of(head, struct leaf, rcu);
	kmem_cache_free(trie_leaf_kmem, l);
}

static inline void free_leaf(struct leaf *l)
{
	call_rcu(&l->rcu, __leaf_free_rcu);
}

static inline void free_leaf_info(struct leaf_info *leaf)
{
	kfree_rcu(leaf, rcu);
}

static struct tnode *tnode_alloc(size_t size)
{
	if (size <= PAGE_SIZE)
		return kzalloc(size, GFP_KERNEL);
	else
		return vzalloc(size);
}

static void __tnode_vfree(struct work_struct *arg)
{
	struct tnode *tn = container_of(arg, struct tnode, work);
	vfree(tn);
}

static void __tnode_free_rcu(struct rcu_head *head)
{
	struct tnode *tn = container_of(head, struct tnode, rcu);
	size_t size = sizeof(struct tnode) +
		      (sizeof(struct rt_trie_node *) << tn->bits);

	if (size <= PAGE_SIZE)
		kfree(tn);
	else {
		INIT_WORK(&tn->work, __tnode_vfree);
		schedule_work(&tn->work);
	}
}

static inline void tnode_free(struct tnode *tn)
{
	if (IS_LEAF(tn))
		free_leaf((struct leaf *) tn);
	else
		call_rcu(&tn->rcu, __tnode_free_rcu);
}

static void tnode_free_safe(struct tnode *tn)
{
	BUG_ON(IS_LEAF(tn));
	tn->tnode_free = tnode_free_head;
	tnode_free_head = tn;
	tnode_free_size += sizeof(struct tnode) +
			   (sizeof(struct rt_trie_node *) << tn->bits);
}

static void tnode_free_flush(void)
{
	struct tnode *tn;

	while ((tn = tnode_free_head)) {
		tnode_free_head = tn->tnode_free;
		tn->tnode_free = NULL;
		tnode_free(tn);
	}

	if (tnode_free_size >= PAGE_SIZE * sync_pages) {
		tnode_free_size = 0;
		synchronize_rcu();
	}
}

static struct leaf *leaf_new(void)
{
	struct leaf *l = kmem_cache_alloc(trie_leaf_kmem, GFP_KERNEL);
	if (l) {
		l->parent = T_LEAF;
		INIT_HLIST_HEAD(&l->list);
	}
	return l;
}

static struct leaf_info *leaf_info_new(int plen)
{
	struct leaf_info *li = kmalloc(sizeof(struct leaf_info),  GFP_KERNEL);
	if (li) {
		li->plen = plen;
		li->mask_plen = ntohl(inet_make_mask(plen));
		INIT_LIST_HEAD(&li->falh);
	}
	return li;
}

static struct tnode *tnode_new(t_key key, int pos, int bits)
{
	size_t sz = sizeof(struct tnode) + (sizeof(struct rt_trie_node *) << bits);
	struct tnode *tn = tnode_alloc(sz);

	if (tn) {
		tn->parent = T_TNODE;
		tn->pos = pos;
		tn->bits = bits;
		tn->key = key;
		tn->full_children = 0;
		tn->empty_children = 1<<bits;
	}

	pr_debug("AT %p s=%zu %zu\n", tn, sizeof(struct tnode),
		 sizeof(struct rt_trie_node) << bits);
	return tn;
}


static inline int tnode_full(const struct tnode *tn, const struct rt_trie_node *n)
{
	if (n == NULL || IS_LEAF(n))
		return 0;

	return ((struct tnode *) n)->pos == tn->pos + tn->bits;
}

static inline void put_child(struct trie *t, struct tnode *tn, int i,
			     struct rt_trie_node *n)
{
	tnode_put_child_reorg(tn, i, n, -1);
}


static void tnode_put_child_reorg(struct tnode *tn, int i, struct rt_trie_node *n,
				  int wasfull)
{
	struct rt_trie_node *chi = rtnl_dereference(tn->child[i]);
	int isfull;

	BUG_ON(i >= 1<<tn->bits);

	
	if (n == NULL && chi != NULL)
		tn->empty_children++;
	else if (n != NULL && chi == NULL)
		tn->empty_children--;

	
	if (wasfull == -1)
		wasfull = tnode_full(tn, chi);

	isfull = tnode_full(tn, n);
	if (wasfull && !isfull)
		tn->full_children--;
	else if (!wasfull && isfull)
		tn->full_children++;

	if (n)
		node_set_parent(n, tn);

	rcu_assign_pointer(tn->child[i], n);
}

#define MAX_WORK 10
static struct rt_trie_node *resize(struct trie *t, struct tnode *tn)
{
	int i;
	struct tnode *old_tn;
	int inflate_threshold_use;
	int halve_threshold_use;
	int max_work;

	if (!tn)
		return NULL;

	pr_debug("In tnode_resize %p inflate_threshold=%d threshold=%d\n",
		 tn, inflate_threshold, halve_threshold);

	
	if (tn->empty_children == tnode_child_length(tn)) {
		tnode_free_safe(tn);
		return NULL;
	}
	
	if (tn->empty_children == tnode_child_length(tn) - 1)
		goto one_child;


	check_tnode(tn);

	

	if (!node_parent((struct rt_trie_node *)tn)) {
		inflate_threshold_use = inflate_threshold_root;
		halve_threshold_use = halve_threshold_root;
	} else {
		inflate_threshold_use = inflate_threshold;
		halve_threshold_use = halve_threshold;
	}

	max_work = MAX_WORK;
	while ((tn->full_children > 0 &&  max_work-- &&
		50 * (tn->full_children + tnode_child_length(tn)
		      - tn->empty_children)
		>= inflate_threshold_use * tnode_child_length(tn))) {

		old_tn = tn;
		tn = inflate(t, tn);

		if (IS_ERR(tn)) {
			tn = old_tn;
#ifdef CONFIG_IP_FIB_TRIE_STATS
			t->stats.resize_node_skipped++;
#endif
			break;
		}
	}

	check_tnode(tn);

	
	if (max_work != MAX_WORK)
		return (struct rt_trie_node *) tn;


	max_work = MAX_WORK;
	while (tn->bits > 1 &&  max_work-- &&
	       100 * (tnode_child_length(tn) - tn->empty_children) <
	       halve_threshold_use * tnode_child_length(tn)) {

		old_tn = tn;
		tn = halve(t, tn);
		if (IS_ERR(tn)) {
			tn = old_tn;
#ifdef CONFIG_IP_FIB_TRIE_STATS
			t->stats.resize_node_skipped++;
#endif
			break;
		}
	}


	
	if (tn->empty_children == tnode_child_length(tn) - 1) {
one_child:
		for (i = 0; i < tnode_child_length(tn); i++) {
			struct rt_trie_node *n;

			n = rtnl_dereference(tn->child[i]);
			if (!n)
				continue;

			

			node_set_parent(n, NULL);
			tnode_free_safe(tn);
			return n;
		}
	}
	return (struct rt_trie_node *) tn;
}


static void tnode_clean_free(struct tnode *tn)
{
	int i;
	struct tnode *tofree;

	for (i = 0; i < tnode_child_length(tn); i++) {
		tofree = (struct tnode *)rtnl_dereference(tn->child[i]);
		if (tofree)
			tnode_free(tofree);
	}
	tnode_free(tn);
}

static struct tnode *inflate(struct trie *t, struct tnode *tn)
{
	struct tnode *oldtnode = tn;
	int olen = tnode_child_length(tn);
	int i;

	pr_debug("In inflate\n");

	tn = tnode_new(oldtnode->key, oldtnode->pos, oldtnode->bits + 1);

	if (!tn)
		return ERR_PTR(-ENOMEM);


	for (i = 0; i < olen; i++) {
		struct tnode *inode;

		inode = (struct tnode *) tnode_get_child(oldtnode, i);
		if (inode &&
		    IS_TNODE(inode) &&
		    inode->pos == oldtnode->pos + oldtnode->bits &&
		    inode->bits > 1) {
			struct tnode *left, *right;
			t_key m = ~0U << (KEYLENGTH - 1) >> inode->pos;

			left = tnode_new(inode->key&(~m), inode->pos + 1,
					 inode->bits - 1);
			if (!left)
				goto nomem;

			right = tnode_new(inode->key|m, inode->pos + 1,
					  inode->bits - 1);

			if (!right) {
				tnode_free(left);
				goto nomem;
			}

			put_child(t, tn, 2*i, (struct rt_trie_node *) left);
			put_child(t, tn, 2*i+1, (struct rt_trie_node *) right);
		}
	}

	for (i = 0; i < olen; i++) {
		struct tnode *inode;
		struct rt_trie_node *node = tnode_get_child(oldtnode, i);
		struct tnode *left, *right;
		int size, j;

		
		if (node == NULL)
			continue;

		

		if (IS_LEAF(node) || ((struct tnode *) node)->pos >
		   tn->pos + tn->bits - 1) {
			if (tkey_extract_bits(node->key,
					      oldtnode->pos + oldtnode->bits,
					      1) == 0)
				put_child(t, tn, 2*i, node);
			else
				put_child(t, tn, 2*i+1, node);
			continue;
		}

		
		inode = (struct tnode *) node;

		if (inode->bits == 1) {
			put_child(t, tn, 2*i, rtnl_dereference(inode->child[0]));
			put_child(t, tn, 2*i+1, rtnl_dereference(inode->child[1]));

			tnode_free_safe(inode);
			continue;
		}

		



		left = (struct tnode *) tnode_get_child(tn, 2*i);
		put_child(t, tn, 2*i, NULL);

		BUG_ON(!left);

		right = (struct tnode *) tnode_get_child(tn, 2*i+1);
		put_child(t, tn, 2*i+1, NULL);

		BUG_ON(!right);

		size = tnode_child_length(left);
		for (j = 0; j < size; j++) {
			put_child(t, left, j, rtnl_dereference(inode->child[j]));
			put_child(t, right, j, rtnl_dereference(inode->child[j + size]));
		}
		put_child(t, tn, 2*i, resize(t, left));
		put_child(t, tn, 2*i+1, resize(t, right));

		tnode_free_safe(inode);
	}
	tnode_free_safe(oldtnode);
	return tn;
nomem:
	tnode_clean_free(tn);
	return ERR_PTR(-ENOMEM);
}

static struct tnode *halve(struct trie *t, struct tnode *tn)
{
	struct tnode *oldtnode = tn;
	struct rt_trie_node *left, *right;
	int i;
	int olen = tnode_child_length(tn);

	pr_debug("In halve\n");

	tn = tnode_new(oldtnode->key, oldtnode->pos, oldtnode->bits - 1);

	if (!tn)
		return ERR_PTR(-ENOMEM);


	for (i = 0; i < olen; i += 2) {
		left = tnode_get_child(oldtnode, i);
		right = tnode_get_child(oldtnode, i+1);

		
		if (left && right) {
			struct tnode *newn;

			newn = tnode_new(left->key, tn->pos + tn->bits, 1);

			if (!newn)
				goto nomem;

			put_child(t, tn, i/2, (struct rt_trie_node *)newn);
		}

	}

	for (i = 0; i < olen; i += 2) {
		struct tnode *newBinNode;

		left = tnode_get_child(oldtnode, i);
		right = tnode_get_child(oldtnode, i+1);

		
		if (left == NULL) {
			if (right == NULL)    
				continue;
			put_child(t, tn, i/2, right);
			continue;
		}

		if (right == NULL) {
			put_child(t, tn, i/2, left);
			continue;
		}

		
		newBinNode = (struct tnode *) tnode_get_child(tn, i/2);
		put_child(t, tn, i/2, NULL);
		put_child(t, newBinNode, 0, left);
		put_child(t, newBinNode, 1, right);
		put_child(t, tn, i/2, resize(t, newBinNode));
	}
	tnode_free_safe(oldtnode);
	return tn;
nomem:
	tnode_clean_free(tn);
	return ERR_PTR(-ENOMEM);
}


static struct leaf_info *find_leaf_info(struct leaf *l, int plen)
{
	struct hlist_head *head = &l->list;
	struct hlist_node *node;
	struct leaf_info *li;

	hlist_for_each_entry_rcu(li, node, head, hlist)
		if (li->plen == plen)
			return li;

	return NULL;
}

static inline struct list_head *get_fa_head(struct leaf *l, int plen)
{
	struct leaf_info *li = find_leaf_info(l, plen);

	if (!li)
		return NULL;

	return &li->falh;
}

static void insert_leaf_info(struct hlist_head *head, struct leaf_info *new)
{
	struct leaf_info *li = NULL, *last = NULL;
	struct hlist_node *node;

	if (hlist_empty(head)) {
		hlist_add_head_rcu(&new->hlist, head);
	} else {
		hlist_for_each_entry(li, node, head, hlist) {
			if (new->plen > li->plen)
				break;

			last = li;
		}
		if (last)
			hlist_add_after_rcu(&last->hlist, &new->hlist);
		else
			hlist_add_before_rcu(&new->hlist, &li->hlist);
	}
}


static struct leaf *
fib_find_node(struct trie *t, u32 key)
{
	int pos;
	struct tnode *tn;
	struct rt_trie_node *n;

	pos = 0;
	n = rcu_dereference_rtnl(t->trie);

	while (n != NULL &&  NODE_TYPE(n) == T_TNODE) {
		tn = (struct tnode *) n;

		check_tnode(tn);

		if (tkey_sub_equals(tn->key, pos, tn->pos-pos, key)) {
			pos = tn->pos + tn->bits;
			n = tnode_get_child_rcu(tn,
						tkey_extract_bits(key,
								  tn->pos,
								  tn->bits));
		} else
			break;
	}
	

	if (n != NULL && IS_LEAF(n) && tkey_equals(key, n->key))
		return (struct leaf *)n;

	return NULL;
}

static void trie_rebalance(struct trie *t, struct tnode *tn)
{
	int wasfull;
	t_key cindex, key;
	struct tnode *tp;

	key = tn->key;

	while (tn != NULL && (tp = node_parent((struct rt_trie_node *)tn)) != NULL) {
		cindex = tkey_extract_bits(key, tp->pos, tp->bits);
		wasfull = tnode_full(tp, tnode_get_child(tp, cindex));
		tn = (struct tnode *) resize(t, (struct tnode *)tn);

		tnode_put_child_reorg((struct tnode *)tp, cindex,
				      (struct rt_trie_node *)tn, wasfull);

		tp = node_parent((struct rt_trie_node *) tn);
		if (!tp)
			rcu_assign_pointer(t->trie, (struct rt_trie_node *)tn);

		tnode_free_flush();
		if (!tp)
			break;
		tn = tp;
	}

	
	if (IS_TNODE(tn))
		tn = (struct tnode *)resize(t, (struct tnode *)tn);

	rcu_assign_pointer(t->trie, (struct rt_trie_node *)tn);
	tnode_free_flush();
}


static struct list_head *fib_insert_node(struct trie *t, u32 key, int plen)
{
	int pos, newpos;
	struct tnode *tp = NULL, *tn = NULL;
	struct rt_trie_node *n;
	struct leaf *l;
	int missbit;
	struct list_head *fa_head = NULL;
	struct leaf_info *li;
	t_key cindex;

	pos = 0;
	n = rtnl_dereference(t->trie);


	while (n != NULL &&  NODE_TYPE(n) == T_TNODE) {
		tn = (struct tnode *) n;

		check_tnode(tn);

		if (tkey_sub_equals(tn->key, pos, tn->pos-pos, key)) {
			tp = tn;
			pos = tn->pos + tn->bits;
			n = tnode_get_child(tn,
					    tkey_extract_bits(key,
							      tn->pos,
							      tn->bits));

			BUG_ON(n && node_parent(n) != tn);
		} else
			break;
	}


	BUG_ON(tp && IS_LEAF(tp));

	

	if (n != NULL && IS_LEAF(n) && tkey_equals(key, n->key)) {
		l = (struct leaf *) n;
		li = leaf_info_new(plen);

		if (!li)
			return NULL;

		fa_head = &li->falh;
		insert_leaf_info(&l->list, li);
		goto done;
	}
	l = leaf_new();

	if (!l)
		return NULL;

	l->key = key;
	li = leaf_info_new(plen);

	if (!li) {
		free_leaf(l);
		return NULL;
	}

	fa_head = &li->falh;
	insert_leaf_info(&l->list, li);

	if (t->trie && n == NULL) {
		

		node_set_parent((struct rt_trie_node *)l, tp);

		cindex = tkey_extract_bits(key, tp->pos, tp->bits);
		put_child(t, (struct tnode *)tp, cindex, (struct rt_trie_node *)l);
	} else {
		

		if (tp)
			pos = tp->pos+tp->bits;
		else
			pos = 0;

		if (n) {
			newpos = tkey_mismatch(key, pos, n->key);
			tn = tnode_new(n->key, newpos, 1);
		} else {
			newpos = 0;
			tn = tnode_new(key, newpos, 1); 
		}

		if (!tn) {
			free_leaf_info(li);
			free_leaf(l);
			return NULL;
		}

		node_set_parent((struct rt_trie_node *)tn, tp);

		missbit = tkey_extract_bits(key, newpos, 1);
		put_child(t, tn, missbit, (struct rt_trie_node *)l);
		put_child(t, tn, 1-missbit, n);

		if (tp) {
			cindex = tkey_extract_bits(key, tp->pos, tp->bits);
			put_child(t, (struct tnode *)tp, cindex,
				  (struct rt_trie_node *)tn);
		} else {
			rcu_assign_pointer(t->trie, (struct rt_trie_node *)tn);
			tp = tn;
		}
	}

	if (tp && tp->pos + tp->bits > 32)
		pr_warn("fib_trie tp=%p pos=%d, bits=%d, key=%0x plen=%d\n",
			tp, tp->pos, tp->bits, key, plen);

	

	trie_rebalance(t, tp);
done:
	return fa_head;
}

int fib_table_insert(struct fib_table *tb, struct fib_config *cfg)
{
	struct trie *t = (struct trie *) tb->tb_data;
	struct fib_alias *fa, *new_fa;
	struct list_head *fa_head = NULL;
	struct fib_info *fi;
	int plen = cfg->fc_dst_len;
	u8 tos = cfg->fc_tos;
	u32 key, mask;
	int err;
	struct leaf *l;

	if (plen > 32)
		return -EINVAL;

	key = ntohl(cfg->fc_dst);

	pr_debug("Insert table=%u %08x/%d\n", tb->tb_id, key, plen);
#ifdef CONFIG_HTC_FIB_RULE_DEBUG
	printk(KERN_DEBUG "[NET][CORE][RULE] %s Insert table=%u %08x/%d\n", __func__,tb->tb_id,key, plen);
#endif

	mask = ntohl(inet_make_mask(plen));

	if (key & ~mask)
		return -EINVAL;

	key = key & mask;

	fi = fib_create_info(cfg);
	if (IS_ERR(fi)) {
		err = PTR_ERR(fi);
		goto err;
	}

	l = fib_find_node(t, key);
	fa = NULL;

	if (l) {
		fa_head = get_fa_head(l, plen);
		fa = fib_find_alias(fa_head, tos, fi->fib_priority);
	}


	if (fa && fa->fa_tos == tos &&
	    fa->fa_info->fib_priority == fi->fib_priority) {
		struct fib_alias *fa_first, *fa_match;

		err = -EEXIST;
		if (cfg->fc_nlflags & NLM_F_EXCL)
			goto out;

		fa_match = NULL;
		fa_first = fa;
		fa = list_entry(fa->fa_list.prev, struct fib_alias, fa_list);
		list_for_each_entry_continue(fa, fa_head, fa_list) {
			if (fa->fa_tos != tos)
				break;
			if (fa->fa_info->fib_priority != fi->fib_priority)
				break;
			if (fa->fa_type == cfg->fc_type &&
			    fa->fa_info == fi) {
				fa_match = fa;
				break;
			}
		}

		if (cfg->fc_nlflags & NLM_F_REPLACE) {
			struct fib_info *fi_drop;
			u8 state;

			fa = fa_first;
			if (fa_match) {
				if (fa == fa_match)
					err = 0;
				goto out;
			}
			err = -ENOBUFS;
			new_fa = kmem_cache_alloc(fn_alias_kmem, GFP_KERNEL);
			if (new_fa == NULL)
				goto out;

			fi_drop = fa->fa_info;
			new_fa->fa_tos = fa->fa_tos;
			new_fa->fa_info = fi;
			new_fa->fa_type = cfg->fc_type;
			state = fa->fa_state;
			new_fa->fa_state = state & ~FA_S_ACCESSED;

			list_replace_rcu(&fa->fa_list, &new_fa->fa_list);
			alias_free_mem_rcu(fa);

			fib_release_info(fi_drop);
			if (state & FA_S_ACCESSED)
				rt_cache_flush(cfg->fc_nlinfo.nl_net, -1);
			rtmsg_fib(RTM_NEWROUTE, htonl(key), new_fa, plen,
				tb->tb_id, &cfg->fc_nlinfo, NLM_F_REPLACE);

			goto succeeded;
		}
		if (fa_match)
			goto out;

		if (!(cfg->fc_nlflags & NLM_F_APPEND))
			fa = fa_first;
	}
	err = -ENOENT;
	if (!(cfg->fc_nlflags & NLM_F_CREATE))
		goto out;

	err = -ENOBUFS;
	new_fa = kmem_cache_alloc(fn_alias_kmem, GFP_KERNEL);
	if (new_fa == NULL)
		goto out;

	new_fa->fa_info = fi;
	new_fa->fa_tos = tos;
	new_fa->fa_type = cfg->fc_type;
	new_fa->fa_state = 0;

	if (!fa_head) {
		fa_head = fib_insert_node(t, key, plen);
		if (unlikely(!fa_head)) {
			err = -ENOMEM;
			goto out_free_new_fa;
		}
	}

	if (!plen)
		tb->tb_num_default++;

	list_add_tail_rcu(&new_fa->fa_list,
			  (fa ? &fa->fa_list : fa_head));

	rt_cache_flush(cfg->fc_nlinfo.nl_net, -1);
	rtmsg_fib(RTM_NEWROUTE, htonl(key), new_fa, plen, tb->tb_id,
		  &cfg->fc_nlinfo, 0);
succeeded:
	return 0;

out_free_new_fa:
	kmem_cache_free(fn_alias_kmem, new_fa);
out:
	fib_release_info(fi);
err:
	return err;
}

static int check_leaf(struct fib_table *tb, struct trie *t, struct leaf *l,
		      t_key key,  const struct flowi4 *flp,
		      struct fib_result *res, int fib_flags)
{
	struct leaf_info *li;
	struct hlist_head *hhead = &l->list;
	struct hlist_node *node;

	hlist_for_each_entry_rcu(li, node, hhead, hlist) {
		struct fib_alias *fa;

		if (l->key != (key & li->mask_plen))
			continue;

		list_for_each_entry_rcu(fa, &li->falh, fa_list) {
			struct fib_info *fi = fa->fa_info;
			int nhsel, err;

			if (fa->fa_tos && fa->fa_tos != flp->flowi4_tos)
				continue;
			if (fi->fib_dead)
				continue;
			if (fa->fa_info->fib_scope < flp->flowi4_scope)
				continue;
			fib_alias_accessed(fa);
			err = fib_props[fa->fa_type].error;
			if (err) {
#ifdef CONFIG_IP_FIB_TRIE_STATS
				t->stats.semantic_match_passed++;
#endif
				return err;
			}
			if (fi->fib_flags & RTNH_F_DEAD)
				continue;
			for (nhsel = 0; nhsel < fi->fib_nhs; nhsel++) {
				const struct fib_nh *nh = &fi->fib_nh[nhsel];

				if (nh->nh_flags & RTNH_F_DEAD)
					continue;
				if (flp->flowi4_oif && flp->flowi4_oif != nh->nh_oif)
					continue;

#ifdef CONFIG_IP_FIB_TRIE_STATS
				t->stats.semantic_match_passed++;
#endif
				res->prefixlen = li->plen;
				res->nh_sel = nhsel;
				res->type = fa->fa_type;
				res->scope = fa->fa_info->fib_scope;
				res->fi = fi;
				res->table = tb;
				res->fa_head = &li->falh;
				if (!(fib_flags & FIB_LOOKUP_NOREF))
					atomic_inc(&fi->fib_clntref);
				return 0;
			}
		}

#ifdef CONFIG_IP_FIB_TRIE_STATS
		t->stats.semantic_match_miss++;
#endif
	}

	return 1;
}

int fib_table_lookup(struct fib_table *tb, const struct flowi4 *flp,
		     struct fib_result *res, int fib_flags)
{
	struct trie *t = (struct trie *) tb->tb_data;
	int ret;
	struct rt_trie_node *n;
	struct tnode *pn;
	unsigned int pos, bits;
	t_key key = ntohl(flp->daddr);
	unsigned int chopped_off;
	t_key cindex = 0;
	unsigned int current_prefix_length = KEYLENGTH;
	struct tnode *cn;
	t_key pref_mismatch;

	rcu_read_lock();

	n = rcu_dereference(t->trie);
	if (!n)
		goto failed;

#ifdef CONFIG_IP_FIB_TRIE_STATS
	t->stats.gets++;
#endif

	
	if (IS_LEAF(n)) {
		ret = check_leaf(tb, t, (struct leaf *)n, key, flp, res, fib_flags);
		goto found;
	}

	pn = (struct tnode *) n;
	chopped_off = 0;

	while (pn) {
		pos = pn->pos;
		bits = pn->bits;

		if (!chopped_off)
			cindex = tkey_extract_bits(mask_pfx(key, current_prefix_length),
						   pos, bits);

		n = tnode_get_child_rcu(pn, cindex);

		if (n == NULL) {
#ifdef CONFIG_IP_FIB_TRIE_STATS
			t->stats.null_node_hit++;
#endif
			goto backtrace;
		}

		if (IS_LEAF(n)) {
			ret = check_leaf(tb, t, (struct leaf *)n, key, flp, res, fib_flags);
			if (ret > 0)
				goto backtrace;
			goto found;
		}

		cn = (struct tnode *)n;




		if (current_prefix_length < pos+bits) {
			if (tkey_extract_bits(cn->key, current_prefix_length,
						cn->pos - current_prefix_length)
			    || !(cn->child[0]))
				goto backtrace;
		}




		pref_mismatch = mask_pfx(cn->key ^ key, cn->pos);

		if (pref_mismatch) {
			int mp = KEYLENGTH - fls(pref_mismatch);

			if (tkey_extract_bits(cn->key, mp, cn->pos - mp) != 0)
				goto backtrace;

			if (current_prefix_length >= cn->pos)
				current_prefix_length = mp;
		}

		pn = (struct tnode *)n; 
		chopped_off = 0;
		continue;

backtrace:
		chopped_off++;

		
		while ((chopped_off <= pn->bits)
		       && !(cindex & (1<<(chopped_off-1))))
			chopped_off++;

		
		if (current_prefix_length > pn->pos + pn->bits - chopped_off)
			current_prefix_length = pn->pos + pn->bits
				- chopped_off;


		if (chopped_off <= pn->bits) {
			cindex &= ~(1 << (chopped_off-1));
		} else {
			struct tnode *parent = node_parent_rcu((struct rt_trie_node *) pn);
			if (!parent)
				goto failed;

			
			cindex = tkey_extract_bits(pn->key, parent->pos, parent->bits);
			pn = parent;
			chopped_off = 0;

#ifdef CONFIG_IP_FIB_TRIE_STATS
			t->stats.backtrack++;
#endif
			goto backtrace;
		}
	}
failed:
	ret = 1;
found:
	rcu_read_unlock();
	return ret;
}
EXPORT_SYMBOL_GPL(fib_table_lookup);

static void trie_leaf_remove(struct trie *t, struct leaf *l)
{
	struct tnode *tp = node_parent((struct rt_trie_node *) l);

	pr_debug("entering trie_leaf_remove(%p)\n", l);

	if (tp) {
		t_key cindex = tkey_extract_bits(l->key, tp->pos, tp->bits);
		put_child(t, (struct tnode *)tp, cindex, NULL);
		trie_rebalance(t, tp);
	} else
		RCU_INIT_POINTER(t->trie, NULL);

	free_leaf(l);
}

int fib_table_delete(struct fib_table *tb, struct fib_config *cfg)
{
	struct trie *t = (struct trie *) tb->tb_data;
	u32 key, mask;
	int plen = cfg->fc_dst_len;
	u8 tos = cfg->fc_tos;
	struct fib_alias *fa, *fa_to_delete;
	struct list_head *fa_head;
	struct leaf *l;
	struct leaf_info *li;

	if (plen > 32)
		return -EINVAL;

	key = ntohl(cfg->fc_dst);
	mask = ntohl(inet_make_mask(plen));

	if (key & ~mask)
		return -EINVAL;

	key = key & mask;
	l = fib_find_node(t, key);

	if (!l)
		return -ESRCH;

	fa_head = get_fa_head(l, plen);
	fa = fib_find_alias(fa_head, tos, 0);

	if (!fa)
		return -ESRCH;

	pr_debug("Deleting %08x/%d tos=%d t=%p\n", key, plen, tos, t);
#ifdef CONFIG_HTC_FIB_RULE_DEBUG
	printk(KERN_DEBUG "[NET][CORE][RULE] %s Deleting %08x/%d tos=%d t=%p\n", __func__, key, plen, tos, t);
#endif

	fa_to_delete = NULL;
	fa = list_entry(fa->fa_list.prev, struct fib_alias, fa_list);
	list_for_each_entry_continue(fa, fa_head, fa_list) {
		struct fib_info *fi = fa->fa_info;

		if (fa->fa_tos != tos)
			break;

		if ((!cfg->fc_type || fa->fa_type == cfg->fc_type) &&
		    (cfg->fc_scope == RT_SCOPE_NOWHERE ||
		     fa->fa_info->fib_scope == cfg->fc_scope) &&
		    (!cfg->fc_prefsrc ||
		     fi->fib_prefsrc == cfg->fc_prefsrc) &&
		    (!cfg->fc_protocol ||
		     fi->fib_protocol == cfg->fc_protocol) &&
		    fib_nh_match(cfg, fi) == 0) {
			fa_to_delete = fa;
			break;
		}
	}

	if (!fa_to_delete)
		return -ESRCH;

	fa = fa_to_delete;
	rtmsg_fib(RTM_DELROUTE, htonl(key), fa, plen, tb->tb_id,
		  &cfg->fc_nlinfo, 0);

	l = fib_find_node(t, key);
	li = find_leaf_info(l, plen);

	list_del_rcu(&fa->fa_list);

	if (!plen)
		tb->tb_num_default--;

	if (list_empty(fa_head)) {
		hlist_del_rcu(&li->hlist);
		free_leaf_info(li);
	}

	if (hlist_empty(&l->list))
		trie_leaf_remove(t, l);

	if (fa->fa_state & FA_S_ACCESSED)
		rt_cache_flush(cfg->fc_nlinfo.nl_net, -1);

	fib_release_info(fa->fa_info);
	alias_free_mem_rcu(fa);
	return 0;
}

static int trie_flush_list(struct list_head *head)
{
	struct fib_alias *fa, *fa_node;
	int found = 0;

	list_for_each_entry_safe(fa, fa_node, head, fa_list) {
		struct fib_info *fi = fa->fa_info;

		if (fi && (fi->fib_flags & RTNH_F_DEAD)) {
			list_del_rcu(&fa->fa_list);
			fib_release_info(fa->fa_info);
			alias_free_mem_rcu(fa);
			found++;
		}
	}
	return found;
}

static int trie_flush_leaf(struct leaf *l)
{
	int found = 0;
	struct hlist_head *lih = &l->list;
	struct hlist_node *node, *tmp;
	struct leaf_info *li = NULL;

	hlist_for_each_entry_safe(li, node, tmp, lih, hlist) {
		found += trie_flush_list(&li->falh);

		if (list_empty(&li->falh)) {
			hlist_del_rcu(&li->hlist);
			free_leaf_info(li);
		}
	}
	return found;
}

static struct leaf *leaf_walk_rcu(struct tnode *p, struct rt_trie_node *c)
{

#ifdef CONFIG_HTC_NETWORK_MODIFY
	void *pq;
	if ((!p) || (IS_ERR(p)) || (probe_kernel_address(p,pq))) {
		pr_err("[NET][WARN] p is illegal in %s \n", __func__);
		return NULL; 
	}
#endif

	do {
		t_key idx;
#ifdef CONFIG_HTC_NETWORK_MODIFY
		void *q;
#endif

#ifdef CONFIG_HTC_NETWORK_MODIFY
		if ((c) && (!IS_ERR(c)))
#else
		if (c)
#endif
			idx = tkey_extract_bits(c->key, p->pos, p->bits) + 1;
		else
			idx = 0;

		while (idx < 1u << p->bits) {
			c = tnode_get_child_rcu(p, idx++);

#ifdef CONFIG_HTC_NETWORK_MODIFY
			if ((!c) || (IS_ERR(c))) {
				pr_err("[NET] c is NULL in %s , idx=%d\n", __func__,idx);
				continue;
			}

			if (probe_kernel_address(c,q)) {
				pr_err("[NET] c is in %s illegal, going to next round,idx = %d\n", __func__,idx);
				continue;
			}
#else
			if (!c)
				continue;
#endif

			if (IS_LEAF(c)) {
				prefetch(rcu_dereference_rtnl(p->child[idx]));
				return (struct leaf *) c;
			}

			
			p = (struct tnode *) c;
			idx = 0;
		}

		
		c = (struct rt_trie_node *) p;
	} while ((p = node_parent_rcu(c)) != NULL);

	return NULL; 
}

static struct leaf *trie_firstleaf(struct trie *t)
{
	struct tnode *n = (struct tnode *)rcu_dereference_rtnl(t->trie);

	if (!n)
		return NULL;

	if (IS_LEAF(n))          
		return (struct leaf *) n;

	return leaf_walk_rcu(n, NULL);
}

static struct leaf *trie_nextleaf(struct leaf *l)
{
	struct rt_trie_node *c = (struct rt_trie_node *) l;
	struct tnode *p = node_parent_rcu(c);

#ifdef CONFIG_HTC_NETWORK_MODIFY
	if ((!p) || (IS_ERR(p)))
#else
	if (!p)
#endif
		return NULL;	

	return leaf_walk_rcu(p, c);
}

static struct leaf *trie_leafindex(struct trie *t, int index)
{
	struct leaf *l = trie_firstleaf(t);

	while (l && index-- > 0)
		l = trie_nextleaf(l);

	return l;
}


int fib_table_flush(struct fib_table *tb)
{
	struct trie *t = (struct trie *) tb->tb_data;
	struct leaf *l, *ll = NULL;
	int found = 0;
#ifdef CONFIG_HTC_FIB_RULE_DEBUG
	printk(KERN_DEBUG  "[NET][CORE][RULE]%s+\n", __func__);
#endif
	for (l = trie_firstleaf(t); l; l = trie_nextleaf(l)) {
		found += trie_flush_leaf(l);

		if (ll && hlist_empty(&ll->list))
			trie_leaf_remove(t, ll);
		ll = l;
	}

	if (ll && hlist_empty(&ll->list))
		trie_leaf_remove(t, ll);

	pr_debug("trie_flush found=%d\n", found);
#ifdef CONFIG_HTC_FIB_RULE_DEBUG
	printk(KERN_DEBUG "[NET][CORE][RULE] %s-; trie_flush found=%d\n", __func__, found);
#endif
	return found;
}

void fib_free_table(struct fib_table *tb)
{
	kfree(tb);
}

static int fn_trie_dump_fa(t_key key, int plen, struct list_head *fah,
			   struct fib_table *tb,
			   struct sk_buff *skb, struct netlink_callback *cb)
{
	int i, s_i;
	struct fib_alias *fa;
	__be32 xkey = htonl(key);

	s_i = cb->args[5];
	i = 0;

	

	list_for_each_entry_rcu(fa, fah, fa_list) {
		if (i < s_i) {
			i++;
			continue;
		}

		if (fib_dump_info(skb, NETLINK_CB(cb->skb).pid,
				  cb->nlh->nlmsg_seq,
				  RTM_NEWROUTE,
				  tb->tb_id,
				  fa->fa_type,
				  xkey,
				  plen,
				  fa->fa_tos,
				  fa->fa_info, NLM_F_MULTI) < 0) {
			cb->args[5] = i;
			return -1;
		}
		i++;
	}
	cb->args[5] = i;
	return skb->len;
}

static int fn_trie_dump_leaf(struct leaf *l, struct fib_table *tb,
			struct sk_buff *skb, struct netlink_callback *cb)
{
	struct leaf_info *li;
	struct hlist_node *node;
	int i, s_i;

	s_i = cb->args[4];
	i = 0;

	
	hlist_for_each_entry_rcu(li, node, &l->list, hlist) {
		if (i < s_i) {
			i++;
			continue;
		}

		if (i > s_i)
			cb->args[5] = 0;

		if (list_empty(&li->falh))
			continue;

		if (fn_trie_dump_fa(l->key, li->plen, &li->falh, tb, skb, cb) < 0) {
			cb->args[4] = i;
			return -1;
		}
		i++;
	}

	cb->args[4] = i;
	return skb->len;
}

int fib_table_dump(struct fib_table *tb, struct sk_buff *skb,
		   struct netlink_callback *cb)
{
	struct leaf *l;
	struct trie *t = (struct trie *) tb->tb_data;
	t_key key = cb->args[2];
	int count = cb->args[3];

#ifdef CONFIG_HTC_FIB_RULE_DEBUG
	printk(KERN_DEBUG  "[NET][CORE][RULE]%s\n", __func__);
#endif

	rcu_read_lock();
	if (count == 0)
		l = trie_firstleaf(t);
	else {
		l = fib_find_node(t, key);
		if (!l)
			l = trie_leafindex(t, count);
	}

	while (l) {
		cb->args[2] = l->key;
		if (fn_trie_dump_leaf(l, tb, skb, cb) < 0) {
			cb->args[3] = count;
			rcu_read_unlock();
			return -1;
		}

		++count;
		l = trie_nextleaf(l);
		memset(&cb->args[4], 0,
		       sizeof(cb->args) - 4*sizeof(cb->args[0]));
	}
	cb->args[3] = count;
	rcu_read_unlock();

	return skb->len;
}

void __init fib_trie_init(void)
{
	fn_alias_kmem = kmem_cache_create("ip_fib_alias",
					  sizeof(struct fib_alias),
					  0, SLAB_PANIC, NULL);

	trie_leaf_kmem = kmem_cache_create("ip_fib_trie",
					   max(sizeof(struct leaf),
					       sizeof(struct leaf_info)),
					   0, SLAB_PANIC, NULL);
}


struct fib_table *fib_trie_table(u32 id)
{
	struct fib_table *tb;
	struct trie *t;

	tb = kmalloc(sizeof(struct fib_table) + sizeof(struct trie),
		     GFP_KERNEL);
	if (tb == NULL)
		return NULL;

	tb->tb_id = id;
	tb->tb_default = -1;
	tb->tb_num_default = 0;

	t = (struct trie *) tb->tb_data;
	memset(t, 0, sizeof(*t));

	return tb;
}

#ifdef CONFIG_PROC_FS
struct fib_trie_iter {
	struct seq_net_private p;
	struct fib_table *tb;
	struct tnode *tnode;
	unsigned int index;
	unsigned int depth;
};

static struct rt_trie_node *fib_trie_get_next(struct fib_trie_iter *iter)
{
	struct tnode *tn = iter->tnode;
	unsigned int cindex = iter->index;
	struct tnode *p;

	
	if (!tn)
		return NULL;

	pr_debug("get_next iter={node=%p index=%d depth=%d}\n",
		 iter->tnode, iter->index, iter->depth);
rescan:
	while (cindex < (1<<tn->bits)) {
		struct rt_trie_node *n = tnode_get_child_rcu(tn, cindex);

		if (n) {
			if (IS_LEAF(n)) {
				iter->tnode = tn;
				iter->index = cindex + 1;
			} else {
				
				iter->tnode = (struct tnode *) n;
				iter->index = 0;
				++iter->depth;
			}
			return n;
		}

		++cindex;
	}

	
	p = node_parent_rcu((struct rt_trie_node *)tn);
	if (p) {
		cindex = tkey_extract_bits(tn->key, p->pos, p->bits)+1;
		tn = p;
		--iter->depth;
		goto rescan;
	}

	
	return NULL;
}

static struct rt_trie_node *fib_trie_get_first(struct fib_trie_iter *iter,
				       struct trie *t)
{
	struct rt_trie_node *n;

	if (!t)
		return NULL;

	n = rcu_dereference(t->trie);
	if (!n)
		return NULL;

	if (IS_TNODE(n)) {
		iter->tnode = (struct tnode *) n;
		iter->index = 0;
		iter->depth = 1;
	} else {
		iter->tnode = NULL;
		iter->index = 0;
		iter->depth = 0;
	}

	return n;
}

static void trie_collect_stats(struct trie *t, struct trie_stat *s)
{
	struct rt_trie_node *n;
	struct fib_trie_iter iter;

	memset(s, 0, sizeof(*s));

	rcu_read_lock();
	for (n = fib_trie_get_first(&iter, t); n; n = fib_trie_get_next(&iter)) {
		if (IS_LEAF(n)) {
			struct leaf *l = (struct leaf *)n;
			struct leaf_info *li;
			struct hlist_node *tmp;

			s->leaves++;
			s->totdepth += iter.depth;
			if (iter.depth > s->maxdepth)
				s->maxdepth = iter.depth;

			hlist_for_each_entry_rcu(li, tmp, &l->list, hlist)
				++s->prefixes;
		} else {
			const struct tnode *tn = (const struct tnode *) n;
			int i;

			s->tnodes++;
			if (tn->bits < MAX_STAT_DEPTH)
				s->nodesizes[tn->bits]++;

			for (i = 0; i < (1<<tn->bits); i++)
				if (!tn->child[i])
					s->nullpointers++;
		}
	}
	rcu_read_unlock();
}

static void trie_show_stats(struct seq_file *seq, struct trie_stat *stat)
{
	unsigned int i, max, pointers, bytes, avdepth;

	if (stat->leaves)
		avdepth = stat->totdepth*100 / stat->leaves;
	else
		avdepth = 0;

	seq_printf(seq, "\tAver depth:     %u.%02d\n",
		   avdepth / 100, avdepth % 100);
	seq_printf(seq, "\tMax depth:      %u\n", stat->maxdepth);

	seq_printf(seq, "\tLeaves:         %u\n", stat->leaves);
	bytes = sizeof(struct leaf) * stat->leaves;

	seq_printf(seq, "\tPrefixes:       %u\n", stat->prefixes);
	bytes += sizeof(struct leaf_info) * stat->prefixes;

	seq_printf(seq, "\tInternal nodes: %u\n\t", stat->tnodes);
	bytes += sizeof(struct tnode) * stat->tnodes;

	max = MAX_STAT_DEPTH;
	while (max > 0 && stat->nodesizes[max-1] == 0)
		max--;

	pointers = 0;
	for (i = 1; i <= max; i++)
		if (stat->nodesizes[i] != 0) {
			seq_printf(seq, "  %u: %u",  i, stat->nodesizes[i]);
			pointers += (1<<i) * stat->nodesizes[i];
		}
	seq_putc(seq, '\n');
	seq_printf(seq, "\tPointers: %u\n", pointers);

	bytes += sizeof(struct rt_trie_node *) * pointers;
	seq_printf(seq, "Null ptrs: %u\n", stat->nullpointers);
	seq_printf(seq, "Total size: %u  kB\n", (bytes + 1023) / 1024);
}

#ifdef CONFIG_IP_FIB_TRIE_STATS
static void trie_show_usage(struct seq_file *seq,
			    const struct trie_use_stats *stats)
{
	seq_printf(seq, "\nCounters:\n---------\n");
	seq_printf(seq, "gets = %u\n", stats->gets);
	seq_printf(seq, "backtracks = %u\n", stats->backtrack);
	seq_printf(seq, "semantic match passed = %u\n",
		   stats->semantic_match_passed);
	seq_printf(seq, "semantic match miss = %u\n",
		   stats->semantic_match_miss);
	seq_printf(seq, "null node hit= %u\n", stats->null_node_hit);
	seq_printf(seq, "skipped node resize = %u\n\n",
		   stats->resize_node_skipped);
}
#endif 

static void fib_table_print(struct seq_file *seq, struct fib_table *tb)
{
	if (tb->tb_id == RT_TABLE_LOCAL)
		seq_puts(seq, "Local:\n");
	else if (tb->tb_id == RT_TABLE_MAIN)
		seq_puts(seq, "Main:\n");
	else
		seq_printf(seq, "Id %d:\n", tb->tb_id);
}


static int fib_triestat_seq_show(struct seq_file *seq, void *v)
{
	struct net *net = (struct net *)seq->private;
	unsigned int h;

	seq_printf(seq,
		   "Basic info: size of leaf:"
		   " %Zd bytes, size of tnode: %Zd bytes.\n",
		   sizeof(struct leaf), sizeof(struct tnode));

	for (h = 0; h < FIB_TABLE_HASHSZ; h++) {
		struct hlist_head *head = &net->ipv4.fib_table_hash[h];
		struct hlist_node *node;
		struct fib_table *tb;

		hlist_for_each_entry_rcu(tb, node, head, tb_hlist) {
			struct trie *t = (struct trie *) tb->tb_data;
			struct trie_stat stat;

			if (!t)
				continue;

			fib_table_print(seq, tb);

			trie_collect_stats(t, &stat);
			trie_show_stats(seq, &stat);
#ifdef CONFIG_IP_FIB_TRIE_STATS
			trie_show_usage(seq, &t->stats);
#endif
		}
	}

	return 0;
}

static int fib_triestat_seq_open(struct inode *inode, struct file *file)
{
	return single_open_net(inode, file, fib_triestat_seq_show);
}

static const struct file_operations fib_triestat_fops = {
	.owner	= THIS_MODULE,
	.open	= fib_triestat_seq_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release = single_release_net,
};

static struct rt_trie_node *fib_trie_get_idx(struct seq_file *seq, loff_t pos)
{
	struct fib_trie_iter *iter = seq->private;
	struct net *net = seq_file_net(seq);
	loff_t idx = 0;
	unsigned int h;

	for (h = 0; h < FIB_TABLE_HASHSZ; h++) {
		struct hlist_head *head = &net->ipv4.fib_table_hash[h];
		struct hlist_node *node;
		struct fib_table *tb;

		hlist_for_each_entry_rcu(tb, node, head, tb_hlist) {
			struct rt_trie_node *n;

			for (n = fib_trie_get_first(iter,
						    (struct trie *) tb->tb_data);
			     n; n = fib_trie_get_next(iter))
				if (pos == idx++) {
					iter->tb = tb;
					return n;
				}
		}
	}

	return NULL;
}

static void *fib_trie_seq_start(struct seq_file *seq, loff_t *pos)
	__acquires(RCU)
{
	rcu_read_lock();
	return fib_trie_get_idx(seq, *pos);
}

static void *fib_trie_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct fib_trie_iter *iter = seq->private;
	struct net *net = seq_file_net(seq);
	struct fib_table *tb = iter->tb;
	struct hlist_node *tb_node;
	unsigned int h;
	struct rt_trie_node *n;

	++*pos;
	
	n = fib_trie_get_next(iter);
	if (n)
		return n;

	
	h = tb->tb_id & (FIB_TABLE_HASHSZ - 1);
	while ((tb_node = rcu_dereference(hlist_next_rcu(&tb->tb_hlist)))) {
		tb = hlist_entry(tb_node, struct fib_table, tb_hlist);
		n = fib_trie_get_first(iter, (struct trie *) tb->tb_data);
		if (n)
			goto found;
	}

	
	while (++h < FIB_TABLE_HASHSZ) {
		struct hlist_head *head = &net->ipv4.fib_table_hash[h];
		hlist_for_each_entry_rcu(tb, tb_node, head, tb_hlist) {
			n = fib_trie_get_first(iter, (struct trie *) tb->tb_data);
			if (n)
				goto found;
		}
	}
	return NULL;

found:
	iter->tb = tb;
	return n;
}

static void fib_trie_seq_stop(struct seq_file *seq, void *v)
	__releases(RCU)
{
	rcu_read_unlock();
}

static void seq_indent(struct seq_file *seq, int n)
{
	while (n-- > 0)
		seq_puts(seq, "   ");
}

static inline const char *rtn_scope(char *buf, size_t len, enum rt_scope_t s)
{
	switch (s) {
	case RT_SCOPE_UNIVERSE: return "universe";
	case RT_SCOPE_SITE:	return "site";
	case RT_SCOPE_LINK:	return "link";
	case RT_SCOPE_HOST:	return "host";
	case RT_SCOPE_NOWHERE:	return "nowhere";
	default:
		snprintf(buf, len, "scope=%d", s);
		return buf;
	}
}

static const char *const rtn_type_names[__RTN_MAX] = {
	[RTN_UNSPEC] = "UNSPEC",
	[RTN_UNICAST] = "UNICAST",
	[RTN_LOCAL] = "LOCAL",
	[RTN_BROADCAST] = "BROADCAST",
	[RTN_ANYCAST] = "ANYCAST",
	[RTN_MULTICAST] = "MULTICAST",
	[RTN_BLACKHOLE] = "BLACKHOLE",
	[RTN_UNREACHABLE] = "UNREACHABLE",
	[RTN_PROHIBIT] = "PROHIBIT",
	[RTN_THROW] = "THROW",
	[RTN_NAT] = "NAT",
	[RTN_XRESOLVE] = "XRESOLVE",
};

static inline const char *rtn_type(char *buf, size_t len, unsigned int t)
{
	if (t < __RTN_MAX && rtn_type_names[t])
		return rtn_type_names[t];
	snprintf(buf, len, "type %u", t);
	return buf;
}

static int fib_trie_seq_show(struct seq_file *seq, void *v)
{
	const struct fib_trie_iter *iter = seq->private;
	struct rt_trie_node *n = v;

	if (!node_parent_rcu(n))
		fib_table_print(seq, iter->tb);

	if (IS_TNODE(n)) {
		struct tnode *tn = (struct tnode *) n;
		__be32 prf = htonl(mask_pfx(tn->key, tn->pos));

		seq_indent(seq, iter->depth-1);
		seq_printf(seq, "  +-- %pI4/%d %d %d %d\n",
			   &prf, tn->pos, tn->bits, tn->full_children,
			   tn->empty_children);

	} else {
		struct leaf *l = (struct leaf *) n;
		struct leaf_info *li;
		struct hlist_node *node;
		__be32 val = htonl(l->key);

		seq_indent(seq, iter->depth);
		seq_printf(seq, "  |-- %pI4\n", &val);

		hlist_for_each_entry_rcu(li, node, &l->list, hlist) {
			struct fib_alias *fa;

			list_for_each_entry_rcu(fa, &li->falh, fa_list) {
				char buf1[32], buf2[32];

				seq_indent(seq, iter->depth+1);
				seq_printf(seq, "  /%d %s %s", li->plen,
					   rtn_scope(buf1, sizeof(buf1),
						     fa->fa_info->fib_scope),
					   rtn_type(buf2, sizeof(buf2),
						    fa->fa_type));
				if (fa->fa_tos)
					seq_printf(seq, " tos=%d", fa->fa_tos);
				seq_putc(seq, '\n');
			}
		}
	}

	return 0;
}

static const struct seq_operations fib_trie_seq_ops = {
	.start  = fib_trie_seq_start,
	.next   = fib_trie_seq_next,
	.stop   = fib_trie_seq_stop,
	.show   = fib_trie_seq_show,
};

static int fib_trie_seq_open(struct inode *inode, struct file *file)
{
	return seq_open_net(inode, file, &fib_trie_seq_ops,
			    sizeof(struct fib_trie_iter));
}

static const struct file_operations fib_trie_fops = {
	.owner  = THIS_MODULE,
	.open   = fib_trie_seq_open,
	.read   = seq_read,
	.llseek = seq_lseek,
	.release = seq_release_net,
};

struct fib_route_iter {
	struct seq_net_private p;
	struct trie *main_trie;
	loff_t	pos;
	t_key	key;
};

static struct leaf *fib_route_get_idx(struct fib_route_iter *iter, loff_t pos)
{
	struct leaf *l = NULL;
	struct trie *t = iter->main_trie;

	
	if (iter->pos > 0 && pos >= iter->pos && (l = fib_find_node(t, iter->key)))
		pos -= iter->pos;
	else {
		iter->pos = 0;
		l = trie_firstleaf(t);
	}

	while (l && pos-- > 0) {
		iter->pos++;
		l = trie_nextleaf(l);
	}

	if (l)
		iter->key = pos;	
	else
		iter->pos = 0;		

	return l;
}

static void *fib_route_seq_start(struct seq_file *seq, loff_t *pos)
	__acquires(RCU)
{
	struct fib_route_iter *iter = seq->private;
	struct fib_table *tb;

	rcu_read_lock();
	tb = fib_get_table(seq_file_net(seq), RT_TABLE_MAIN);
	if (!tb)
		return NULL;

	iter->main_trie = (struct trie *) tb->tb_data;
	if (*pos == 0)
		return SEQ_START_TOKEN;
	else
		return fib_route_get_idx(iter, *pos - 1);
}

static void *fib_route_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct fib_route_iter *iter = seq->private;
	struct leaf *l = v;

	++*pos;
	if (v == SEQ_START_TOKEN) {
		iter->pos = 0;
		l = trie_firstleaf(iter->main_trie);
	} else {
		iter->pos++;
		l = trie_nextleaf(l);
	}

	if (l)
		iter->key = l->key;
	else
		iter->pos = 0;
	return l;
}

static void fib_route_seq_stop(struct seq_file *seq, void *v)
	__releases(RCU)
{
	rcu_read_unlock();
}

static unsigned int fib_flag_trans(int type, __be32 mask, const struct fib_info *fi)
{
	unsigned int flags = 0;

	if (type == RTN_UNREACHABLE || type == RTN_PROHIBIT)
		flags = RTF_REJECT;
	if (fi && fi->fib_nh->nh_gw)
		flags |= RTF_GATEWAY;
	if (mask == htonl(0xFFFFFFFF))
		flags |= RTF_HOST;
	flags |= RTF_UP;
	return flags;
}

static int fib_route_seq_show(struct seq_file *seq, void *v)
{
	struct leaf *l = v;
	struct leaf_info *li;
	struct hlist_node *node;

	if (v == SEQ_START_TOKEN) {
		seq_printf(seq, "%-127s\n", "Iface\tDestination\tGateway "
			   "\tFlags\tRefCnt\tUse\tMetric\tMask\t\tMTU"
			   "\tWindow\tIRTT");
		return 0;
	}

	hlist_for_each_entry_rcu(li, node, &l->list, hlist) {
		struct fib_alias *fa;
		__be32 mask, prefix;

#ifdef CONFIG_HTC_NETWORK_MODIFY
		if (( !li)  ||  IS_ERR(li)) {
			printk(KERN_ERR "[NET] li is NULL in %s!\n", __func__);
			return 0;

		}
#endif
		mask = inet_make_mask(li->plen);
		prefix = htonl(l->key);

		list_for_each_entry_rcu(fa, &li->falh, fa_list) {
			const struct fib_info *fi = fa->fa_info;
			unsigned int flags = fib_flag_trans(fa->fa_type, mask, fi);

			if (fa->fa_type == RTN_BROADCAST
			    || fa->fa_type == RTN_MULTICAST)
				continue;

			seq_setwidth(seq, 127);
#ifdef CONFIG_HTC_NETWORK_MODIFY
			if ((fi) && (!IS_ERR(fi)))
#else
			if (fi)
#endif
				seq_printf(seq,
					 "%s\t%08X\t%08X\t%04X\t%d\t%u\t"
					 "%d\t%08X\t%d\t%u\t%u",
					 fi->fib_dev ? fi->fib_dev->name : "*",
					 prefix,
					 fi->fib_nh->nh_gw, flags, 0, 0,
					 fi->fib_priority,
					 mask,
					 (fi->fib_advmss ?
					  fi->fib_advmss + 40 : 0),
					 fi->fib_window,
					 fi->fib_rtt >> 3);
			else
				seq_printf(seq,
					 "*\t%08X\t%08X\t%04X\t%d\t%u\t"
					 "%d\t%08X\t%d\t%u\t%u",
					 prefix, 0, flags, 0, 0, 0,
					 mask, 0, 0, 0);

			seq_pad(seq, '\n');
		}
	}

	return 0;
}

static const struct seq_operations fib_route_seq_ops = {
	.start  = fib_route_seq_start,
	.next   = fib_route_seq_next,
	.stop   = fib_route_seq_stop,
	.show   = fib_route_seq_show,
};

static int fib_route_seq_open(struct inode *inode, struct file *file)
{
	return seq_open_net(inode, file, &fib_route_seq_ops,
			    sizeof(struct fib_route_iter));
}

static const struct file_operations fib_route_fops = {
	.owner  = THIS_MODULE,
	.open   = fib_route_seq_open,
	.read   = seq_read,
	.llseek = seq_lseek,
	.release = seq_release_net,
};

int __net_init fib_proc_init(struct net *net)
{
	if (!proc_net_fops_create(net, "fib_trie", S_IRUGO, &fib_trie_fops))
		goto out1;

	if (!proc_net_fops_create(net, "fib_triestat", S_IRUGO,
				  &fib_triestat_fops))
		goto out2;

	if (!proc_net_fops_create(net, "route", S_IRUGO, &fib_route_fops))
		goto out3;

	return 0;

out3:
	proc_net_remove(net, "fib_triestat");
out2:
	proc_net_remove(net, "fib_trie");
out1:
	return -ENOMEM;
}

void __net_exit fib_proc_exit(struct net *net)
{
	proc_net_remove(net, "fib_trie");
	proc_net_remove(net, "fib_triestat");
	proc_net_remove(net, "route");
}

#endif 
