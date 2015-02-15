
/* (C) 1999-2001 Paul `Rusty' Russell
 * (C) 2002-2006 Netfilter Core Team <coreteam@netfilter.org>
 * (C) 2003,2004 USAGI/WIDE Project <http://www.linux-ipv6.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/netfilter.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/stddef.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/jhash.h>
#include <linux/err.h>
#include <linux/percpu.h>
#include <linux/moduleparam.h>
#include <linux/notifier.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/socket.h>
#include <linux/mm.h>
#include <linux/nsproxy.h>
#include <linux/rculist_nulls.h>

#include <net/netfilter/nf_conntrack.h>
#include <net/netfilter/nf_conntrack_l3proto.h>
#include <net/netfilter/nf_conntrack_l4proto.h>
#include <net/netfilter/nf_conntrack_expect.h>
#include <net/netfilter/nf_conntrack_helper.h>
#include <net/netfilter/nf_conntrack_core.h>
#include <net/netfilter/nf_conntrack_extend.h>
#include <net/netfilter/nf_conntrack_acct.h>
#include <net/netfilter/nf_conntrack_ecache.h>
#include <net/netfilter/nf_conntrack_zones.h>
#include <net/netfilter/nf_conntrack_timestamp.h>
#include <net/netfilter/nf_conntrack_timeout.h>
#include <net/netfilter/nf_nat.h>
#include <net/netfilter/nf_nat_core.h>

#define NF_CONNTRACK_VERSION	"0.5.0"

int (*nfnetlink_parse_nat_setup_hook)(struct nf_conn *ct,
				      enum nf_nat_manip_type manip,
				      const struct nlattr *attr) __read_mostly;
EXPORT_SYMBOL_GPL(nfnetlink_parse_nat_setup_hook);

DEFINE_SPINLOCK(nf_conntrack_lock);
EXPORT_SYMBOL_GPL(nf_conntrack_lock);

unsigned int nf_conntrack_htable_size __read_mostly;
EXPORT_SYMBOL_GPL(nf_conntrack_htable_size);

unsigned int nf_conntrack_max __read_mostly;
EXPORT_SYMBOL_GPL(nf_conntrack_max);

DEFINE_PER_CPU(struct nf_conn, nf_conntrack_untracked);
EXPORT_PER_CPU_SYMBOL(nf_conntrack_untracked);

unsigned int nf_conntrack_hash_rnd __read_mostly;
EXPORT_SYMBOL_GPL(nf_conntrack_hash_rnd);

static u32 hash_conntrack_raw(const struct nf_conntrack_tuple *tuple, u16 zone)
{
	unsigned int n;

	n = (sizeof(tuple->src) + sizeof(tuple->dst.u3)) / sizeof(u32);
	return jhash2((u32 *)tuple, n, zone ^ nf_conntrack_hash_rnd ^
		      (((__force __u16)tuple->dst.u.all << 16) |
		      tuple->dst.protonum));
}

static u32 __hash_bucket(u32 hash, unsigned int size)
{
	return ((u64)hash * size) >> 32;
}

static u32 hash_bucket(u32 hash, const struct net *net)
{
	return __hash_bucket(hash, net->ct.htable_size);
}

static u_int32_t __hash_conntrack(const struct nf_conntrack_tuple *tuple,
				  u16 zone, unsigned int size)
{
	return __hash_bucket(hash_conntrack_raw(tuple, zone), size);
}

static inline u_int32_t hash_conntrack(const struct net *net, u16 zone,
				       const struct nf_conntrack_tuple *tuple)
{
	return __hash_conntrack(tuple, zone, net->ct.htable_size);
}

bool
nf_ct_get_tuple(const struct sk_buff *skb,
		unsigned int nhoff,
		unsigned int dataoff,
		u_int16_t l3num,
		u_int8_t protonum,
		struct nf_conntrack_tuple *tuple,
		const struct nf_conntrack_l3proto *l3proto,
		const struct nf_conntrack_l4proto *l4proto)
{
	memset(tuple, 0, sizeof(*tuple));

	tuple->src.l3num = l3num;
	if (l3proto->pkt_to_tuple(skb, nhoff, tuple) == 0)
		return false;

	tuple->dst.protonum = protonum;
	tuple->dst.dir = IP_CT_DIR_ORIGINAL;

	return l4proto->pkt_to_tuple(skb, dataoff, tuple);
}
EXPORT_SYMBOL_GPL(nf_ct_get_tuple);

bool nf_ct_get_tuplepr(const struct sk_buff *skb, unsigned int nhoff,
		       u_int16_t l3num, struct nf_conntrack_tuple *tuple)
{
	struct nf_conntrack_l3proto *l3proto;
	struct nf_conntrack_l4proto *l4proto;
	unsigned int protoff;
	u_int8_t protonum;
	int ret;

	rcu_read_lock();

	l3proto = __nf_ct_l3proto_find(l3num);
	ret = l3proto->get_l4proto(skb, nhoff, &protoff, &protonum);
	if (ret != NF_ACCEPT) {
		rcu_read_unlock();
		return false;
	}

	l4proto = __nf_ct_l4proto_find(l3num, protonum);

	ret = nf_ct_get_tuple(skb, nhoff, protoff, l3num, protonum, tuple,
			      l3proto, l4proto);

	rcu_read_unlock();
	return ret;
}
EXPORT_SYMBOL_GPL(nf_ct_get_tuplepr);

bool
nf_ct_invert_tuple(struct nf_conntrack_tuple *inverse,
		   const struct nf_conntrack_tuple *orig,
		   const struct nf_conntrack_l3proto *l3proto,
		   const struct nf_conntrack_l4proto *l4proto)
{
	memset(inverse, 0, sizeof(*inverse));

	inverse->src.l3num = orig->src.l3num;
	if (l3proto->invert_tuple(inverse, orig) == 0)
		return false;

	inverse->dst.dir = !orig->dst.dir;

	inverse->dst.protonum = orig->dst.protonum;
	return l4proto->invert_tuple(inverse, orig);
}
EXPORT_SYMBOL_GPL(nf_ct_invert_tuple);

static void
clean_from_lists(struct nf_conn *ct)
{
	pr_debug("clean_from_lists(%p)\n", ct);
	hlist_nulls_del_rcu(&ct->tuplehash[IP_CT_DIR_ORIGINAL].hnnode);
	hlist_nulls_del_rcu(&ct->tuplehash[IP_CT_DIR_REPLY].hnnode);

	
	nf_ct_remove_expectations(ct);
}

static void
destroy_conntrack(struct nf_conntrack *nfct)
{
	struct nf_conn *ct = (struct nf_conn *)nfct;
	struct net *net = nf_ct_net(ct);
	struct nf_conntrack_l4proto *l4proto;

	pr_debug("destroy_conntrack(%p)\n", ct);
	NF_CT_ASSERT(atomic_read(&nfct->use) == 0);
	NF_CT_ASSERT(!timer_pending(&ct->timeout));

	rcu_read_lock();
	l4proto = __nf_ct_l4proto_find(nf_ct_l3num(ct), nf_ct_protonum(ct));
	if (l4proto && l4proto->destroy)
		l4proto->destroy(ct);

	rcu_read_unlock();

	spin_lock_bh(&nf_conntrack_lock);
	nf_ct_remove_expectations(ct);

	
	if (!nf_ct_is_confirmed(ct)) {
		BUG_ON(hlist_nulls_unhashed(&ct->tuplehash[IP_CT_DIR_ORIGINAL].hnnode));
		hlist_nulls_del_rcu(&ct->tuplehash[IP_CT_DIR_ORIGINAL].hnnode);
	}

	NF_CT_STAT_INC(net, delete);
	spin_unlock_bh(&nf_conntrack_lock);

	if (ct->master)
		nf_ct_put(ct->master);

	pr_debug("destroy_conntrack: returning ct=%p to slab\n", ct);
	nf_conntrack_free(ct);
}

void nf_ct_delete_from_lists(struct nf_conn *ct)
{
	struct net *net = nf_ct_net(ct);

	nf_ct_helper_destroy(ct);
	spin_lock_bh(&nf_conntrack_lock);
	NF_CT_STAT_INC(net, delete_list);
	clean_from_lists(ct);
	spin_unlock_bh(&nf_conntrack_lock);
}
EXPORT_SYMBOL_GPL(nf_ct_delete_from_lists);

static void death_by_event(unsigned long ul_conntrack)
{
	struct nf_conn *ct = (void *)ul_conntrack;
	struct net *net = nf_ct_net(ct);

	if (nf_conntrack_event(IPCT_DESTROY, ct) < 0) {
		
		ct->timeout.expires = jiffies +
			(random32() % net->ct.sysctl_events_retry_timeout);
		add_timer(&ct->timeout);
		return;
	}
	
	set_bit(IPS_DYING_BIT, &ct->status);
	spin_lock(&nf_conntrack_lock);
	hlist_nulls_del(&ct->tuplehash[IP_CT_DIR_ORIGINAL].hnnode);
	spin_unlock(&nf_conntrack_lock);
	nf_ct_put(ct);
}

void nf_ct_insert_dying_list(struct nf_conn *ct)
{
	struct net *net = nf_ct_net(ct);

	
	spin_lock_bh(&nf_conntrack_lock);
	hlist_nulls_add_head(&ct->tuplehash[IP_CT_DIR_ORIGINAL].hnnode,
			     &net->ct.dying);
	spin_unlock_bh(&nf_conntrack_lock);
	
	setup_timer(&ct->timeout, death_by_event, (unsigned long)ct);
	ct->timeout.expires = jiffies +
		(random32() % net->ct.sysctl_events_retry_timeout);
	add_timer(&ct->timeout);
}
EXPORT_SYMBOL_GPL(nf_ct_insert_dying_list);

static void death_by_timeout(unsigned long ul_conntrack)
{
	struct nf_conn *ct = (void *)ul_conntrack;
	struct nf_conn_tstamp *tstamp;

	tstamp = nf_conn_tstamp_find(ct);
	if (tstamp && tstamp->stop == 0)
		tstamp->stop = ktime_to_ns(ktime_get_real());

	if (!test_bit(IPS_DYING_BIT, &ct->status) &&
	    unlikely(nf_conntrack_event(IPCT_DESTROY, ct) < 0)) {
		
		nf_ct_delete_from_lists(ct);
		nf_ct_insert_dying_list(ct);
		return;
	}
	set_bit(IPS_DYING_BIT, &ct->status);
	nf_ct_delete_from_lists(ct);
	nf_ct_put(ct);
}

static struct nf_conntrack_tuple_hash *
____nf_conntrack_find(struct net *net, u16 zone,
		      const struct nf_conntrack_tuple *tuple, u32 hash)
{
	struct nf_conntrack_tuple_hash *h;
	struct hlist_nulls_node *n;
	unsigned int bucket = hash_bucket(hash, net);

	local_bh_disable();
begin:
	hlist_nulls_for_each_entry_rcu(h, n, &net->ct.hash[bucket], hnnode) {
		if (nf_ct_tuple_equal(tuple, &h->tuple) &&
		    nf_ct_zone(nf_ct_tuplehash_to_ctrack(h)) == zone) {
			NF_CT_STAT_INC(net, found);
			local_bh_enable();
			return h;
		}
		NF_CT_STAT_INC(net, searched);
	}
	if (get_nulls_value(n) != bucket) {
		NF_CT_STAT_INC(net, search_restart);
		goto begin;
	}
	local_bh_enable();

	return NULL;
}

struct nf_conntrack_tuple_hash *
__nf_conntrack_find(struct net *net, u16 zone,
		    const struct nf_conntrack_tuple *tuple)
{
	return ____nf_conntrack_find(net, zone, tuple,
				     hash_conntrack_raw(tuple, zone));
}
EXPORT_SYMBOL_GPL(__nf_conntrack_find);

static struct nf_conntrack_tuple_hash *
__nf_conntrack_find_get(struct net *net, u16 zone,
			const struct nf_conntrack_tuple *tuple, u32 hash)
{
	struct nf_conntrack_tuple_hash *h;
	struct nf_conn *ct;

	rcu_read_lock();
begin:
	h = ____nf_conntrack_find(net, zone, tuple, hash);
	if (h) {
		ct = nf_ct_tuplehash_to_ctrack(h);
		if (unlikely(nf_ct_is_dying(ct) ||
			     !atomic_inc_not_zero(&ct->ct_general.use)))
			h = NULL;
		else {
			if (unlikely(!nf_ct_tuple_equal(tuple, &h->tuple) ||
				     nf_ct_zone(ct) != zone)) {
				nf_ct_put(ct);
				goto begin;
			}
		}
	}
	rcu_read_unlock();

	return h;
}

struct nf_conntrack_tuple_hash *
nf_conntrack_find_get(struct net *net, u16 zone,
		      const struct nf_conntrack_tuple *tuple)
{
	return __nf_conntrack_find_get(net, zone, tuple,
				       hash_conntrack_raw(tuple, zone));
}
EXPORT_SYMBOL_GPL(nf_conntrack_find_get);

static void __nf_conntrack_hash_insert(struct nf_conn *ct,
				       unsigned int hash,
				       unsigned int repl_hash)
{
	struct net *net = nf_ct_net(ct);

	hlist_nulls_add_head_rcu(&ct->tuplehash[IP_CT_DIR_ORIGINAL].hnnode,
			   &net->ct.hash[hash]);
	hlist_nulls_add_head_rcu(&ct->tuplehash[IP_CT_DIR_REPLY].hnnode,
			   &net->ct.hash[repl_hash]);
}

int
nf_conntrack_hash_check_insert(struct nf_conn *ct)
{
	struct net *net = nf_ct_net(ct);
	unsigned int hash, repl_hash;
	struct nf_conntrack_tuple_hash *h;
	struct hlist_nulls_node *n;
	u16 zone;

	zone = nf_ct_zone(ct);
	hash = hash_conntrack(net, zone,
			      &ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple);
	repl_hash = hash_conntrack(net, zone,
				   &ct->tuplehash[IP_CT_DIR_REPLY].tuple);

	spin_lock_bh(&nf_conntrack_lock);

	
	hlist_nulls_for_each_entry(h, n, &net->ct.hash[hash], hnnode)
		if (nf_ct_tuple_equal(&ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple,
				      &h->tuple) &&
		    zone == nf_ct_zone(nf_ct_tuplehash_to_ctrack(h)))
			goto out;
	hlist_nulls_for_each_entry(h, n, &net->ct.hash[repl_hash], hnnode)
		if (nf_ct_tuple_equal(&ct->tuplehash[IP_CT_DIR_REPLY].tuple,
				      &h->tuple) &&
		    zone == nf_ct_zone(nf_ct_tuplehash_to_ctrack(h)))
			goto out;

	add_timer(&ct->timeout);
	nf_conntrack_get(&ct->ct_general);
	__nf_conntrack_hash_insert(ct, hash, repl_hash);
	NF_CT_STAT_INC(net, insert);
	spin_unlock_bh(&nf_conntrack_lock);

	return 0;

out:
	NF_CT_STAT_INC(net, insert_failed);
	spin_unlock_bh(&nf_conntrack_lock);
	return -EEXIST;
}
EXPORT_SYMBOL_GPL(nf_conntrack_hash_check_insert);

int
__nf_conntrack_confirm(struct sk_buff *skb)
{
	unsigned int hash, repl_hash;
	struct nf_conntrack_tuple_hash *h;
	struct nf_conn *ct;
	struct nf_conn_help *help;
	struct nf_conn_tstamp *tstamp;
	struct hlist_nulls_node *n;
	enum ip_conntrack_info ctinfo;
	struct net *net;
	u16 zone;

	ct = nf_ct_get(skb, &ctinfo);
	net = nf_ct_net(ct);

	if (CTINFO2DIR(ctinfo) != IP_CT_DIR_ORIGINAL)
		return NF_ACCEPT;

	zone = nf_ct_zone(ct);
	
	hash = *(unsigned long *)&ct->tuplehash[IP_CT_DIR_REPLY].hnnode.pprev;
	hash = hash_bucket(hash, net);
	repl_hash = hash_conntrack(net, zone,
				   &ct->tuplehash[IP_CT_DIR_REPLY].tuple);

	

	NF_CT_ASSERT(!nf_ct_is_confirmed(ct));
	pr_debug("Confirming conntrack %p\n", ct);

	spin_lock_bh(&nf_conntrack_lock);


	if (unlikely(nf_ct_is_dying(ct))) {
		spin_unlock_bh(&nf_conntrack_lock);
		return NF_ACCEPT;
	}

	hlist_nulls_for_each_entry(h, n, &net->ct.hash[hash], hnnode)
		if (nf_ct_tuple_equal(&ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple,
				      &h->tuple) &&
		    zone == nf_ct_zone(nf_ct_tuplehash_to_ctrack(h)))
			goto out;
	hlist_nulls_for_each_entry(h, n, &net->ct.hash[repl_hash], hnnode)
		if (nf_ct_tuple_equal(&ct->tuplehash[IP_CT_DIR_REPLY].tuple,
				      &h->tuple) &&
		    zone == nf_ct_zone(nf_ct_tuplehash_to_ctrack(h)))
			goto out;

	
	hlist_nulls_del_rcu(&ct->tuplehash[IP_CT_DIR_ORIGINAL].hnnode);

	ct->timeout.expires += jiffies;
	add_timer(&ct->timeout);
	atomic_inc(&ct->ct_general.use);
	ct->status |= IPS_CONFIRMED;

	
	tstamp = nf_conn_tstamp_find(ct);
	if (tstamp) {
		if (skb->tstamp.tv64 == 0)
			__net_timestamp((struct sk_buff *)skb);

		tstamp->start = ktime_to_ns(skb->tstamp);
	}
	__nf_conntrack_hash_insert(ct, hash, repl_hash);
	NF_CT_STAT_INC(net, insert);
	spin_unlock_bh(&nf_conntrack_lock);

	help = nfct_help(ct);
	if (help && help->helper)
		nf_conntrack_event_cache(IPCT_HELPER, ct);

	nf_conntrack_event_cache(master_ct(ct) ?
				 IPCT_RELATED : IPCT_NEW, ct);
	return NF_ACCEPT;

out:
	NF_CT_STAT_INC(net, insert_failed);
	spin_unlock_bh(&nf_conntrack_lock);
	return NF_DROP;
}
EXPORT_SYMBOL_GPL(__nf_conntrack_confirm);

int
nf_conntrack_tuple_taken(const struct nf_conntrack_tuple *tuple,
			 const struct nf_conn *ignored_conntrack)
{
	struct net *net = nf_ct_net(ignored_conntrack);
	struct nf_conntrack_tuple_hash *h;
	struct hlist_nulls_node *n;
	struct nf_conn *ct;
	u16 zone = nf_ct_zone(ignored_conntrack);
	unsigned int hash = hash_conntrack(net, zone, tuple);

	rcu_read_lock_bh();
	hlist_nulls_for_each_entry_rcu(h, n, &net->ct.hash[hash], hnnode) {
		ct = nf_ct_tuplehash_to_ctrack(h);
		if (ct != ignored_conntrack &&
		    nf_ct_tuple_equal(tuple, &h->tuple) &&
		    nf_ct_zone(ct) == zone) {
			NF_CT_STAT_INC(net, found);
			rcu_read_unlock_bh();
			return 1;
		}
		NF_CT_STAT_INC(net, searched);
	}
	rcu_read_unlock_bh();

	return 0;
}
EXPORT_SYMBOL_GPL(nf_conntrack_tuple_taken);

#define NF_CT_EVICTION_RANGE	8

static noinline int early_drop(struct net *net, unsigned int hash)
{
	
	struct nf_conntrack_tuple_hash *h;
	struct nf_conn *ct = NULL, *tmp;
	struct hlist_nulls_node *n;
	unsigned int i, cnt = 0;
	int dropped = 0;

	rcu_read_lock();
	for (i = 0; i < net->ct.htable_size; i++) {
		hlist_nulls_for_each_entry_rcu(h, n, &net->ct.hash[hash],
					 hnnode) {
			tmp = nf_ct_tuplehash_to_ctrack(h);
			if (!test_bit(IPS_ASSURED_BIT, &tmp->status))
				ct = tmp;
			cnt++;
		}

		if (ct != NULL) {
			if (likely(!nf_ct_is_dying(ct) &&
				   atomic_inc_not_zero(&ct->ct_general.use)))
				break;
			else
				ct = NULL;
		}

		if (cnt >= NF_CT_EVICTION_RANGE)
			break;

		hash = (hash + 1) % net->ct.htable_size;
	}
	rcu_read_unlock();

	if (!ct)
		return dropped;

	if (del_timer(&ct->timeout)) {
		death_by_timeout((unsigned long)ct);
		if (test_bit(IPS_DYING_BIT, &ct->status)) {
			dropped = 1;
			NF_CT_STAT_INC_ATOMIC(net, early_drop);
		}
	}
	nf_ct_put(ct);
	return dropped;
}

void init_nf_conntrack_hash_rnd(void)
{
	unsigned int rand;

	do {
		get_random_bytes(&rand, sizeof(rand));
	} while (!rand);
	cmpxchg(&nf_conntrack_hash_rnd, 0, rand);
}

static struct nf_conn *
__nf_conntrack_alloc(struct net *net, u16 zone,
		     const struct nf_conntrack_tuple *orig,
		     const struct nf_conntrack_tuple *repl,
		     gfp_t gfp, u32 hash)
{
	struct nf_conn *ct;

	if (unlikely(!nf_conntrack_hash_rnd)) {
		init_nf_conntrack_hash_rnd();
		
		hash = hash_conntrack_raw(orig, zone);
	}

	
	atomic_inc(&net->ct.count);

	if (nf_conntrack_max &&
	    unlikely(atomic_read(&net->ct.count) > nf_conntrack_max)) {
		if (!early_drop(net, hash_bucket(hash, net))) {
			atomic_dec(&net->ct.count);
			if (net_ratelimit())
				printk(KERN_WARNING
				       "nf_conntrack: table full, dropping"
				       " packet.\n");
			return ERR_PTR(-ENOMEM);
		}
	}

	ct = kmem_cache_alloc(net->ct.nf_conntrack_cachep, gfp);
	if (ct == NULL) {
		atomic_dec(&net->ct.count);
		return ERR_PTR(-ENOMEM);
	}
	memset(&ct->tuplehash[IP_CT_DIR_MAX], 0,
	       offsetof(struct nf_conn, proto) -
	       offsetof(struct nf_conn, tuplehash[IP_CT_DIR_MAX]));
	spin_lock_init(&ct->lock);
	ct->tuplehash[IP_CT_DIR_ORIGINAL].tuple = *orig;
	ct->tuplehash[IP_CT_DIR_ORIGINAL].hnnode.pprev = NULL;
	ct->tuplehash[IP_CT_DIR_REPLY].tuple = *repl;
	
	*(unsigned long *)(&ct->tuplehash[IP_CT_DIR_REPLY].hnnode.pprev) = hash;
	
	setup_timer(&ct->timeout, death_by_timeout, (unsigned long)ct);
	write_pnet(&ct->ct_net, net);
#if defined(CONFIG_IP_NF_TARGET_NATTYPE_MODULE)
	ct->nattype_entry = 0;
#endif
#ifdef CONFIG_NF_CONNTRACK_ZONES
	if (zone) {
		struct nf_conntrack_zone *nf_ct_zone;

		nf_ct_zone = nf_ct_ext_add(ct, NF_CT_EXT_ZONE, GFP_ATOMIC);
		if (!nf_ct_zone)
			goto out_free;
		nf_ct_zone->id = zone;
	}
#endif
	smp_wmb();
	atomic_set(&ct->ct_general.use, 1);
	return ct;

#ifdef CONFIG_NF_CONNTRACK_ZONES
out_free:
	atomic_dec(&net->ct.count);
	kmem_cache_free(net->ct.nf_conntrack_cachep, ct);
	return ERR_PTR(-ENOMEM);
#endif
}

struct nf_conn *nf_conntrack_alloc(struct net *net, u16 zone,
				   const struct nf_conntrack_tuple *orig,
				   const struct nf_conntrack_tuple *repl,
				   gfp_t gfp)
{
	return __nf_conntrack_alloc(net, zone, orig, repl, gfp, 0);
}
EXPORT_SYMBOL_GPL(nf_conntrack_alloc);

void nf_conntrack_free(struct nf_conn *ct)
{
	struct net *net = nf_ct_net(ct);

	nf_ct_ext_destroy(ct);
	nf_ct_ext_free(ct);
	kmem_cache_free(net->ct.nf_conntrack_cachep, ct);
	smp_mb__before_atomic_dec();
	atomic_dec(&net->ct.count);
}
EXPORT_SYMBOL_GPL(nf_conntrack_free);

static struct nf_conntrack_tuple_hash *
init_conntrack(struct net *net, struct nf_conn *tmpl,
	       const struct nf_conntrack_tuple *tuple,
	       struct nf_conntrack_l3proto *l3proto,
	       struct nf_conntrack_l4proto *l4proto,
	       struct sk_buff *skb,
	       unsigned int dataoff, u32 hash)
{
	struct nf_conn *ct;
	struct nf_conn_help *help;
	struct nf_conntrack_tuple repl_tuple;
	struct nf_conntrack_ecache *ecache;
	struct nf_conntrack_expect *exp;
	u16 zone = tmpl ? nf_ct_zone(tmpl) : NF_CT_DEFAULT_ZONE;
	struct nf_conn_timeout *timeout_ext;
	unsigned int *timeouts;

	if (!nf_ct_invert_tuple(&repl_tuple, tuple, l3proto, l4proto)) {
		pr_debug("Can't invert tuple.\n");
		return NULL;
	}

	ct = __nf_conntrack_alloc(net, zone, tuple, &repl_tuple, GFP_ATOMIC,
				  hash);
	if (IS_ERR(ct))
		return (struct nf_conntrack_tuple_hash *)ct;

	timeout_ext = tmpl ? nf_ct_timeout_find(tmpl) : NULL;
	if (timeout_ext)
		timeouts = NF_CT_TIMEOUT_EXT_DATA(timeout_ext);
	else
		timeouts = l4proto->get_timeouts(net);

	if (!l4proto->new(ct, skb, dataoff, timeouts)) {
		nf_conntrack_free(ct);
		pr_debug("init conntrack: can't track with proto module\n");
		return NULL;
	}

	if (timeout_ext)
		nf_ct_timeout_ext_add(ct, timeout_ext->timeout, GFP_ATOMIC);

	nf_ct_acct_ext_add(ct, GFP_ATOMIC);
	nf_ct_tstamp_ext_add(ct, GFP_ATOMIC);

	ecache = tmpl ? nf_ct_ecache_find(tmpl) : NULL;
	nf_ct_ecache_ext_add(ct, ecache ? ecache->ctmask : 0,
				 ecache ? ecache->expmask : 0,
			     GFP_ATOMIC);

	spin_lock_bh(&nf_conntrack_lock);
	exp = nf_ct_find_expectation(net, zone, tuple);
	if (exp) {
		pr_debug("conntrack: expectation arrives ct=%p exp=%p\n",
			 ct, exp);
		
		__set_bit(IPS_EXPECTED_BIT, &ct->status);
		ct->master = exp->master;
		if (exp->helper) {
			help = nf_ct_helper_ext_add(ct, GFP_ATOMIC);
			if (help)
				rcu_assign_pointer(help->helper, exp->helper);
		}

#ifdef CONFIG_NF_CONNTRACK_MARK
		ct->mark = exp->master->mark;
#endif
#ifdef CONFIG_NF_CONNTRACK_SECMARK
		ct->secmark = exp->master->secmark;
#endif
#if defined(CONFIG_IP_NF_TARGET_NATTYPE_MODULE)
		ct->nattype_entry = 0;
#endif
		nf_conntrack_get(&ct->master->ct_general);
		NF_CT_STAT_INC(net, expect_new);
	} else {
		__nf_ct_try_assign_helper(ct, tmpl, GFP_ATOMIC);
		NF_CT_STAT_INC(net, new);
	}

	
	hlist_nulls_add_head_rcu(&ct->tuplehash[IP_CT_DIR_ORIGINAL].hnnode,
		       &net->ct.unconfirmed);

	spin_unlock_bh(&nf_conntrack_lock);

	if (exp) {
		if (exp->expectfn)
			exp->expectfn(ct, exp);
		nf_ct_expect_put(exp);
	}

	return &ct->tuplehash[IP_CT_DIR_ORIGINAL];
}

static inline struct nf_conn *
resolve_normal_ct(struct net *net, struct nf_conn *tmpl,
		  struct sk_buff *skb,
		  unsigned int dataoff,
		  u_int16_t l3num,
		  u_int8_t protonum,
		  struct nf_conntrack_l3proto *l3proto,
		  struct nf_conntrack_l4proto *l4proto,
		  int *set_reply,
		  enum ip_conntrack_info *ctinfo)
{
	struct nf_conntrack_tuple tuple;
	struct nf_conntrack_tuple_hash *h;
	struct nf_conn *ct;
	u16 zone = tmpl ? nf_ct_zone(tmpl) : NF_CT_DEFAULT_ZONE;
	u32 hash;

	if (!nf_ct_get_tuple(skb, skb_network_offset(skb),
			     dataoff, l3num, protonum, &tuple, l3proto,
			     l4proto)) {
		pr_debug("resolve_normal_ct: Can't get tuple\n");
		return NULL;
	}

	
	hash = hash_conntrack_raw(&tuple, zone);
	h = __nf_conntrack_find_get(net, zone, &tuple, hash);
	if (!h) {
		h = init_conntrack(net, tmpl, &tuple, l3proto, l4proto,
				   skb, dataoff, hash);
		if (!h)
			return NULL;
		if (IS_ERR(h))
			return (void *)h;
	}
	ct = nf_ct_tuplehash_to_ctrack(h);

	
	if (NF_CT_DIRECTION(h) == IP_CT_DIR_REPLY) {
		*ctinfo = IP_CT_ESTABLISHED_REPLY;
		
		*set_reply = 1;
	} else {
		
		if (test_bit(IPS_SEEN_REPLY_BIT, &ct->status)) {
			pr_debug("nf_conntrack_in: normal packet for %p\n", ct);
			*ctinfo = IP_CT_ESTABLISHED;
		} else if (test_bit(IPS_EXPECTED_BIT, &ct->status)) {
			pr_debug("nf_conntrack_in: related packet for %p\n",
				 ct);
			*ctinfo = IP_CT_RELATED;
		} else {
			pr_debug("nf_conntrack_in: new packet for %p\n", ct);
			*ctinfo = IP_CT_NEW;
		}
		*set_reply = 0;
	}
	skb->nfct = &ct->ct_general;
	skb->nfctinfo = *ctinfo;
	return ct;
}

unsigned int
nf_conntrack_in(struct net *net, u_int8_t pf, unsigned int hooknum,
		struct sk_buff *skb)
{
	struct nf_conn *ct, *tmpl = NULL;
	enum ip_conntrack_info ctinfo;
	struct nf_conntrack_l3proto *l3proto;
	struct nf_conntrack_l4proto *l4proto;
	struct nf_conn_timeout *timeout_ext;
	unsigned int *timeouts;
	unsigned int dataoff;
	u_int8_t protonum;
	int set_reply = 0;
	int ret;

	if (skb->nfct) {
		
		tmpl = (struct nf_conn *)skb->nfct;
		if (!nf_ct_is_template(tmpl)) {
			NF_CT_STAT_INC_ATOMIC(net, ignore);
			return NF_ACCEPT;
		}
		skb->nfct = NULL;
	}

	
	l3proto = __nf_ct_l3proto_find(pf);
	ret = l3proto->get_l4proto(skb, skb_network_offset(skb),
				   &dataoff, &protonum);
	if (ret <= 0) {
		pr_debug("not prepared to track yet or error occurred\n");
		NF_CT_STAT_INC_ATOMIC(net, error);
		NF_CT_STAT_INC_ATOMIC(net, invalid);
		ret = -ret;
		goto out;
	}

	l4proto = __nf_ct_l4proto_find(pf, protonum);

	if (l4proto != NULL && l4proto->error != NULL) {
		ret = l4proto->error(net, tmpl, skb, dataoff, &ctinfo,
				     pf, hooknum);
		if (ret <= 0) {
			NF_CT_STAT_INC_ATOMIC(net, error);
			NF_CT_STAT_INC_ATOMIC(net, invalid);
			ret = -ret;
			goto out;
		}
		
		if (skb->nfct)
			goto out;
	}

	ct = resolve_normal_ct(net, tmpl, skb, dataoff, pf, protonum,
			       l3proto, l4proto, &set_reply, &ctinfo);
	if (!ct) {
		
		NF_CT_STAT_INC_ATOMIC(net, invalid);
		ret = NF_ACCEPT;
		goto out;
	}

	if (IS_ERR(ct)) {
		
		NF_CT_STAT_INC_ATOMIC(net, drop);
		ret = NF_DROP;
		goto out;
	}

	NF_CT_ASSERT(skb->nfct);

	
	timeout_ext = nf_ct_timeout_find(ct);
	if (timeout_ext)
		timeouts = NF_CT_TIMEOUT_EXT_DATA(timeout_ext);
	else
		timeouts = l4proto->get_timeouts(net);

	ret = l4proto->packet(ct, skb, dataoff, ctinfo, pf, hooknum, timeouts);
	if (ret <= 0) {
		pr_debug("nf_conntrack_in: Can't track with proto module\n");
		nf_conntrack_put(skb->nfct);
		skb->nfct = NULL;
		NF_CT_STAT_INC_ATOMIC(net, invalid);
		if (ret == -NF_DROP)
			NF_CT_STAT_INC_ATOMIC(net, drop);
		ret = -ret;
		goto out;
	}

	if (set_reply && !test_and_set_bit(IPS_SEEN_REPLY_BIT, &ct->status))
		nf_conntrack_event_cache(IPCT_REPLY, ct);
out:
	if (tmpl) {
		if (ret == NF_REPEAT)
			skb->nfct = (struct nf_conntrack *)tmpl;
		else
			nf_ct_put(tmpl);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(nf_conntrack_in);

bool nf_ct_invert_tuplepr(struct nf_conntrack_tuple *inverse,
			  const struct nf_conntrack_tuple *orig)
{
	bool ret;

	rcu_read_lock();
	ret = nf_ct_invert_tuple(inverse, orig,
				 __nf_ct_l3proto_find(orig->src.l3num),
				 __nf_ct_l4proto_find(orig->src.l3num,
						      orig->dst.protonum));
	rcu_read_unlock();
	return ret;
}
EXPORT_SYMBOL_GPL(nf_ct_invert_tuplepr);

void nf_conntrack_alter_reply(struct nf_conn *ct,
			      const struct nf_conntrack_tuple *newreply)
{
	struct nf_conn_help *help = nfct_help(ct);

	
	NF_CT_ASSERT(!nf_ct_is_confirmed(ct));

	pr_debug("Altering reply tuple of %p to ", ct);
	nf_ct_dump_tuple(newreply);

	ct->tuplehash[IP_CT_DIR_REPLY].tuple = *newreply;
	if (ct->master || (help && !hlist_empty(&help->expectations)))
		return;

	rcu_read_lock();
	__nf_ct_try_assign_helper(ct, NULL, GFP_ATOMIC);
	rcu_read_unlock();
}
EXPORT_SYMBOL_GPL(nf_conntrack_alter_reply);

void __nf_ct_refresh_acct(struct nf_conn *ct,
			  enum ip_conntrack_info ctinfo,
			  const struct sk_buff *skb,
			  unsigned long extra_jiffies,
			  int do_acct)
{
	NF_CT_ASSERT(ct->timeout.data == (unsigned long)ct);
	NF_CT_ASSERT(skb);

	
	if (test_bit(IPS_FIXED_TIMEOUT_BIT, &ct->status))
		goto acct;

	
	if (!nf_ct_is_confirmed(ct)) {
		ct->timeout.expires = extra_jiffies;
	} else {
		unsigned long newtime = jiffies + extra_jiffies;

		if (newtime - ct->timeout.expires >= HZ)
			mod_timer_pending(&ct->timeout, newtime);
	}

#if defined(CONFIG_IP_NF_TARGET_NATTYPE_MODULE)
	(void)nattype_refresh_timer(ct->nattype_entry, ct->timeout.expires);
#endif

acct:
	if (do_acct) {
		struct nf_conn_counter *acct;

		acct = nf_conn_acct_find(ct);
		if (acct) {
			atomic64_inc(&acct[CTINFO2DIR(ctinfo)].packets);
			atomic64_add(skb->len, &acct[CTINFO2DIR(ctinfo)].bytes);
		}
	}
}
EXPORT_SYMBOL_GPL(__nf_ct_refresh_acct);

bool __nf_ct_kill_acct(struct nf_conn *ct,
		       enum ip_conntrack_info ctinfo,
		       const struct sk_buff *skb,
		       int do_acct)
{
	if (do_acct) {
		struct nf_conn_counter *acct;

		acct = nf_conn_acct_find(ct);
		if (acct) {
			atomic64_inc(&acct[CTINFO2DIR(ctinfo)].packets);
			atomic64_add(skb->len - skb_network_offset(skb),
				     &acct[CTINFO2DIR(ctinfo)].bytes);
		}
	}

	if (del_timer(&ct->timeout)) {
		ct->timeout.function((unsigned long)ct);
		return true;
	}
	return false;
}
EXPORT_SYMBOL_GPL(__nf_ct_kill_acct);

#ifdef CONFIG_NF_CONNTRACK_ZONES
static struct nf_ct_ext_type nf_ct_zone_extend __read_mostly = {
	.len	= sizeof(struct nf_conntrack_zone),
	.align	= __alignof__(struct nf_conntrack_zone),
	.id	= NF_CT_EXT_ZONE,
};
#endif

#if IS_ENABLED(CONFIG_NF_CT_NETLINK)

#include <linux/netfilter/nfnetlink.h>
#include <linux/netfilter/nfnetlink_conntrack.h>
#include <linux/mutex.h>

int nf_ct_port_tuple_to_nlattr(struct sk_buff *skb,
			       const struct nf_conntrack_tuple *tuple)
{
	NLA_PUT_BE16(skb, CTA_PROTO_SRC_PORT, tuple->src.u.tcp.port);
	NLA_PUT_BE16(skb, CTA_PROTO_DST_PORT, tuple->dst.u.tcp.port);
	return 0;

nla_put_failure:
	return -1;
}
EXPORT_SYMBOL_GPL(nf_ct_port_tuple_to_nlattr);

const struct nla_policy nf_ct_port_nla_policy[CTA_PROTO_MAX+1] = {
	[CTA_PROTO_SRC_PORT]  = { .type = NLA_U16 },
	[CTA_PROTO_DST_PORT]  = { .type = NLA_U16 },
};
EXPORT_SYMBOL_GPL(nf_ct_port_nla_policy);

int nf_ct_port_nlattr_to_tuple(struct nlattr *tb[],
			       struct nf_conntrack_tuple *t)
{
	if (!tb[CTA_PROTO_SRC_PORT] || !tb[CTA_PROTO_DST_PORT])
		return -EINVAL;

	t->src.u.tcp.port = nla_get_be16(tb[CTA_PROTO_SRC_PORT]);
	t->dst.u.tcp.port = nla_get_be16(tb[CTA_PROTO_DST_PORT]);

	return 0;
}
EXPORT_SYMBOL_GPL(nf_ct_port_nlattr_to_tuple);

int nf_ct_port_nlattr_tuple_size(void)
{
	return nla_policy_len(nf_ct_port_nla_policy, CTA_PROTO_MAX + 1);
}
EXPORT_SYMBOL_GPL(nf_ct_port_nlattr_tuple_size);
#endif

static void nf_conntrack_attach(struct sk_buff *nskb, struct sk_buff *skb)
{
	struct nf_conn *ct;
	enum ip_conntrack_info ctinfo;

	
	ct = nf_ct_get(skb, &ctinfo);
	if (CTINFO2DIR(ctinfo) == IP_CT_DIR_ORIGINAL)
		ctinfo = IP_CT_RELATED_REPLY;
	else
		ctinfo = IP_CT_RELATED;

	
	nskb->nfct = &ct->ct_general;
	nskb->nfctinfo = ctinfo;
	nf_conntrack_get(nskb->nfct);
}

static struct nf_conn *
get_next_corpse(struct net *net, int (*iter)(struct nf_conn *i, void *data),
		void *data, unsigned int *bucket)
{
	struct nf_conntrack_tuple_hash *h;
	struct nf_conn *ct;
	struct hlist_nulls_node *n;

	spin_lock_bh(&nf_conntrack_lock);
	for (; *bucket < net->ct.htable_size; (*bucket)++) {
		hlist_nulls_for_each_entry(h, n, &net->ct.hash[*bucket], hnnode) {
			ct = nf_ct_tuplehash_to_ctrack(h);
			if (iter(ct, data))
				goto found;
		}
	}
	hlist_nulls_for_each_entry(h, n, &net->ct.unconfirmed, hnnode) {
		ct = nf_ct_tuplehash_to_ctrack(h);
		if (iter(ct, data))
			set_bit(IPS_DYING_BIT, &ct->status);
	}
	spin_unlock_bh(&nf_conntrack_lock);
	return NULL;
found:
	atomic_inc(&ct->ct_general.use);
	spin_unlock_bh(&nf_conntrack_lock);
	return ct;
}

void nf_ct_iterate_cleanup(struct net *net,
			   int (*iter)(struct nf_conn *i, void *data),
			   void *data)
{
	struct nf_conn *ct;
	unsigned int bucket = 0;

	while ((ct = get_next_corpse(net, iter, data, &bucket)) != NULL) {
		
		if (del_timer(&ct->timeout))
			death_by_timeout((unsigned long)ct);
		

		nf_ct_put(ct);
	}
}
EXPORT_SYMBOL_GPL(nf_ct_iterate_cleanup);

struct __nf_ct_flush_report {
	u32 pid;
	int report;
};

static int kill_report(struct nf_conn *i, void *data)
{
	struct __nf_ct_flush_report *fr = (struct __nf_ct_flush_report *)data;
	struct nf_conn_tstamp *tstamp;

	tstamp = nf_conn_tstamp_find(i);
	if (tstamp && tstamp->stop == 0)
		tstamp->stop = ktime_to_ns(ktime_get_real());

	
	if (nf_conntrack_event_report(IPCT_DESTROY, i,
				      fr->pid, fr->report) < 0)
		return 1;

	
	set_bit(IPS_DYING_BIT, &i->status);
	return 1;
}

static int kill_all(struct nf_conn *i, void *data)
{
	return 1;
}

void nf_ct_free_hashtable(void *hash, unsigned int size)
{
	if (is_vmalloc_addr(hash))
		vfree(hash);
	else
		free_pages((unsigned long)hash,
			   get_order(sizeof(struct hlist_head) * size));
}
EXPORT_SYMBOL_GPL(nf_ct_free_hashtable);

void nf_conntrack_flush_report(struct net *net, u32 pid, int report)
{
	struct __nf_ct_flush_report fr = {
		.pid 	= pid,
		.report = report,
	};
	nf_ct_iterate_cleanup(net, kill_report, &fr);
}
EXPORT_SYMBOL_GPL(nf_conntrack_flush_report);

static void nf_ct_release_dying_list(struct net *net)
{
	struct nf_conntrack_tuple_hash *h;
	struct nf_conn *ct;
	struct hlist_nulls_node *n;

	spin_lock_bh(&nf_conntrack_lock);
	hlist_nulls_for_each_entry(h, n, &net->ct.dying, hnnode) {
		ct = nf_ct_tuplehash_to_ctrack(h);
		
		nf_ct_kill(ct);
	}
	spin_unlock_bh(&nf_conntrack_lock);
}

static int untrack_refs(void)
{
	int cnt = 0, cpu;

	for_each_possible_cpu(cpu) {
		struct nf_conn *ct = &per_cpu(nf_conntrack_untracked, cpu);

		cnt += atomic_read(&ct->ct_general.use) - 1;
	}
	return cnt;
}

static void nf_conntrack_cleanup_init_net(void)
{
	while (untrack_refs() > 0)
		schedule();

	nf_conntrack_helper_fini();
	nf_conntrack_proto_fini();
#ifdef CONFIG_NF_CONNTRACK_ZONES
	nf_ct_extend_unregister(&nf_ct_zone_extend);
#endif
}

static void nf_conntrack_cleanup_net(struct net *net)
{
 i_see_dead_people:
	nf_ct_iterate_cleanup(net, kill_all, NULL);
	nf_ct_release_dying_list(net);
	if (atomic_read(&net->ct.count) != 0) {
		schedule();
		goto i_see_dead_people;
	}

	nf_ct_free_hashtable(net->ct.hash, net->ct.htable_size);
	nf_conntrack_timeout_fini(net);
	nf_conntrack_ecache_fini(net);
	nf_conntrack_tstamp_fini(net);
	nf_conntrack_acct_fini(net);
	nf_conntrack_expect_fini(net);
	kmem_cache_destroy(net->ct.nf_conntrack_cachep);
	kfree(net->ct.slabname);
	free_percpu(net->ct.stat);
}

void nf_conntrack_cleanup(struct net *net)
{
	if (net_eq(net, &init_net))
		RCU_INIT_POINTER(ip_ct_attach, NULL);

	synchronize_net();

	nf_conntrack_cleanup_net(net);

	if (net_eq(net, &init_net)) {
		RCU_INIT_POINTER(nf_ct_destroy, NULL);
		nf_conntrack_cleanup_init_net();
	}
}

void *nf_ct_alloc_hashtable(unsigned int *sizep, int nulls)
{
	struct hlist_nulls_head *hash;
	unsigned int nr_slots, i;
	size_t sz;

	BUILD_BUG_ON(sizeof(struct hlist_nulls_head) != sizeof(struct hlist_head));
	nr_slots = *sizep = roundup(*sizep, PAGE_SIZE / sizeof(struct hlist_nulls_head));
	sz = nr_slots * sizeof(struct hlist_nulls_head);
	hash = (void *)__get_free_pages(GFP_KERNEL | __GFP_NOWARN | __GFP_ZERO,
					get_order(sz));
	if (!hash) {
		printk(KERN_WARNING "nf_conntrack: falling back to vmalloc.\n");
		hash = vzalloc(sz);
	}

	if (hash && nulls)
		for (i = 0; i < nr_slots; i++)
			INIT_HLIST_NULLS_HEAD(&hash[i], i);

	return hash;
}
EXPORT_SYMBOL_GPL(nf_ct_alloc_hashtable);

int nf_conntrack_set_hashsize(const char *val, struct kernel_param *kp)
{
	int i, bucket;
	unsigned int hashsize, old_size;
	struct hlist_nulls_head *hash, *old_hash;
	struct nf_conntrack_tuple_hash *h;
	struct nf_conn *ct;

	if (current->nsproxy->net_ns != &init_net)
		return -EOPNOTSUPP;

	
	if (!nf_conntrack_htable_size)
		return param_set_uint(val, kp);

	hashsize = simple_strtoul(val, NULL, 0);
	if (!hashsize)
		return -EINVAL;

	hash = nf_ct_alloc_hashtable(&hashsize, 1);
	if (!hash)
		return -ENOMEM;

	spin_lock_bh(&nf_conntrack_lock);
	for (i = 0; i < init_net.ct.htable_size; i++) {
		while (!hlist_nulls_empty(&init_net.ct.hash[i])) {
			h = hlist_nulls_entry(init_net.ct.hash[i].first,
					struct nf_conntrack_tuple_hash, hnnode);
			ct = nf_ct_tuplehash_to_ctrack(h);
			hlist_nulls_del_rcu(&h->hnnode);
			bucket = __hash_conntrack(&h->tuple, nf_ct_zone(ct),
						  hashsize);
			hlist_nulls_add_head_rcu(&h->hnnode, &hash[bucket]);
		}
	}
	old_size = init_net.ct.htable_size;
	old_hash = init_net.ct.hash;

	init_net.ct.htable_size = nf_conntrack_htable_size = hashsize;
	init_net.ct.hash = hash;
	spin_unlock_bh(&nf_conntrack_lock);

	nf_ct_free_hashtable(old_hash, old_size);
	return 0;
}
EXPORT_SYMBOL_GPL(nf_conntrack_set_hashsize);

module_param_call(hashsize, nf_conntrack_set_hashsize, param_get_uint,
		  &nf_conntrack_htable_size, 0600);

void nf_ct_untracked_status_or(unsigned long bits)
{
	int cpu;

	for_each_possible_cpu(cpu)
		per_cpu(nf_conntrack_untracked, cpu).status |= bits;
}
EXPORT_SYMBOL_GPL(nf_ct_untracked_status_or);

static int nf_conntrack_init_init_net(void)
{
	int max_factor = 8;
	int ret, cpu;

	if (!nf_conntrack_htable_size) {
		nf_conntrack_htable_size
			= (((totalram_pages << PAGE_SHIFT) / 16384)
			   / sizeof(struct hlist_head));
		if (totalram_pages > (1024 * 1024 * 1024 / PAGE_SIZE))
			nf_conntrack_htable_size = 16384;
		if (nf_conntrack_htable_size < 32)
			nf_conntrack_htable_size = 32;

		max_factor = 4;
	}
	nf_conntrack_max = max_factor * nf_conntrack_htable_size;

	printk(KERN_INFO "nf_conntrack version %s (%u buckets, %d max)\n",
	       NF_CONNTRACK_VERSION, nf_conntrack_htable_size,
	       nf_conntrack_max);

	ret = nf_conntrack_proto_init();
	if (ret < 0)
		goto err_proto;

	ret = nf_conntrack_helper_init();
	if (ret < 0)
		goto err_helper;

#ifdef CONFIG_NF_CONNTRACK_ZONES
	ret = nf_ct_extend_register(&nf_ct_zone_extend);
	if (ret < 0)
		goto err_extend;
#endif
	
	for_each_possible_cpu(cpu) {
		struct nf_conn *ct = &per_cpu(nf_conntrack_untracked, cpu);
		write_pnet(&ct->ct_net, &init_net);
		atomic_set(&ct->ct_general.use, 1);
	}
	
	nf_ct_untracked_status_or(IPS_CONFIRMED | IPS_UNTRACKED);
	return 0;

#ifdef CONFIG_NF_CONNTRACK_ZONES
err_extend:
	nf_conntrack_helper_fini();
#endif
err_helper:
	nf_conntrack_proto_fini();
err_proto:
	return ret;
}

#define UNCONFIRMED_NULLS_VAL	((1<<30)+0)
#define DYING_NULLS_VAL		((1<<30)+1)

static int nf_conntrack_init_net(struct net *net)
{
	int ret;

	atomic_set(&net->ct.count, 0);
	INIT_HLIST_NULLS_HEAD(&net->ct.unconfirmed, UNCONFIRMED_NULLS_VAL);
	INIT_HLIST_NULLS_HEAD(&net->ct.dying, DYING_NULLS_VAL);
	net->ct.stat = alloc_percpu(struct ip_conntrack_stat);
	if (!net->ct.stat) {
		ret = -ENOMEM;
		goto err_stat;
	}

	net->ct.slabname = kasprintf(GFP_KERNEL, "nf_conntrack_%p", net);
	if (!net->ct.slabname) {
		ret = -ENOMEM;
		goto err_slabname;
	}

	net->ct.nf_conntrack_cachep = kmem_cache_create(net->ct.slabname,
							sizeof(struct nf_conn), 0,
							SLAB_DESTROY_BY_RCU, NULL);
	if (!net->ct.nf_conntrack_cachep) {
		printk(KERN_ERR "Unable to create nf_conn slab cache\n");
		ret = -ENOMEM;
		goto err_cache;
	}

	net->ct.htable_size = nf_conntrack_htable_size;
	net->ct.hash = nf_ct_alloc_hashtable(&net->ct.htable_size, 1);
	if (!net->ct.hash) {
		ret = -ENOMEM;
		printk(KERN_ERR "Unable to create nf_conntrack_hash\n");
		goto err_hash;
	}
	ret = nf_conntrack_expect_init(net);
	if (ret < 0)
		goto err_expect;
	ret = nf_conntrack_acct_init(net);
	if (ret < 0)
		goto err_acct;
	ret = nf_conntrack_tstamp_init(net);
	if (ret < 0)
		goto err_tstamp;
	ret = nf_conntrack_ecache_init(net);
	if (ret < 0)
		goto err_ecache;
	ret = nf_conntrack_timeout_init(net);
	if (ret < 0)
		goto err_timeout;

	return 0;

err_timeout:
	nf_conntrack_ecache_fini(net);
err_ecache:
	nf_conntrack_tstamp_fini(net);
err_tstamp:
	nf_conntrack_acct_fini(net);
err_acct:
	nf_conntrack_expect_fini(net);
err_expect:
	nf_ct_free_hashtable(net->ct.hash, net->ct.htable_size);
err_hash:
	kmem_cache_destroy(net->ct.nf_conntrack_cachep);
err_cache:
	kfree(net->ct.slabname);
err_slabname:
	free_percpu(net->ct.stat);
err_stat:
	return ret;
}

s16 (*nf_ct_nat_offset)(const struct nf_conn *ct,
			enum ip_conntrack_dir dir,
			u32 seq);
EXPORT_SYMBOL_GPL(nf_ct_nat_offset);

int nf_conntrack_init(struct net *net)
{
	int ret;

	if (net_eq(net, &init_net)) {
		ret = nf_conntrack_init_init_net();
		if (ret < 0)
			goto out_init_net;
	}
	ret = nf_conntrack_init_net(net);
	if (ret < 0)
		goto out_net;

	if (net_eq(net, &init_net)) {
		
		RCU_INIT_POINTER(ip_ct_attach, nf_conntrack_attach);
		RCU_INIT_POINTER(nf_ct_destroy, destroy_conntrack);

		
		RCU_INIT_POINTER(nf_ct_nat_offset, NULL);
	}
	return 0;

out_net:
	if (net_eq(net, &init_net))
		nf_conntrack_cleanup_init_net();
out_init_net:
	return ret;
}
