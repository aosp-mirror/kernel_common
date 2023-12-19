#ifndef __IWL_CHROME
#define __IWL_CHROME
/* This file is pre-included from the Makefile (cc command line)
 *
 * ChromeOS backport definitions
 * Copyright (C) 2016-2017 Intel Deutschland GmbH
 * Copyright (C) 2018-2023 Intel Corporation
 */

#include <linux/version.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/idr.h>
#include <linux/vmalloc.h>

/* get the CPTCFG_* preprocessor symbols */
#include <hdrs/config.h>

#include <hdrs/mac80211-exp.h>

#define LINUX_VERSION_IS_LESS(x1,x2,x3) (LINUX_VERSION_CODE < KERNEL_VERSION(x1,x2,x3))
#define LINUX_VERSION_IS_GEQ(x1,x2,x3)  (LINUX_VERSION_CODE >= KERNEL_VERSION(x1,x2,x3))
#define LINUX_VERSION_IN_RANGE(x1,x2,x3, y1,y2,y3) \
        (LINUX_VERSION_IS_GEQ(x1,x2,x3) && LINUX_VERSION_IS_LESS(y1,y2,y3))
#define LINUX_BACKPORT(sym) backport_ ## sym

/* this must be before including rhashtable.h */
#if LINUX_VERSION_IS_LESS(4,15,0)
#ifndef CONFIG_LOCKDEP
struct lockdep_map { };
#endif /* CONFIG_LOCKDEP */
#endif /* LINUX_VERSION_IS_LESS(4,15,0) */

/* include rhashtable this way to get our copy if another exists */
#include <linux/list_nulls.h>
#include "linux/rhashtable.h"

#include <net/genetlink.h>
#include <linux/crypto.h>
#include <linux/moduleparam.h>
#include <linux/debugfs.h>
#include <linux/hrtimer.h>
#include <crypto/algapi.h>
#include <linux/pci.h>
#include <linux/if_vlan.h>
#include "net/fq.h"

#include <hdrs/net/dropreason.h>

#if LINUX_VERSION_IS_LESS(3,20,0)
#define get_net_ns_by_fd LINUX_BACKPORT(get_net_ns_by_fd)
static inline struct net *get_net_ns_by_fd(int fd)
{
	return ERR_PTR(-EINVAL);
}
#endif

#ifndef DECLARE_FLEX_ARRAY
/**
 * __DECLARE_FLEX_ARRAY() - Declare a flexible array usable in a union
 *
 * @TYPE: The type of each flexible array element
 * @NAME: The name of the flexible array member
 *
 * In order to have a flexible array member in a union or alone in a
 * struct, it needs to be wrapped in an anonymous struct with at least 1
 * named member, but that member can be empty.
+ */
#define __DECLARE_FLEX_ARRAY(TYPE, NAME)       \
	struct {			       \
		struct { } __empty_ ## NAME;   \
		TYPE NAME[];		       \
	}

/**
 * DECLARE_FLEX_ARRAY() - Declare a flexible array usable in a union
 *
 * @TYPE: The type of each flexible array element
 * @NAME: The name of the flexible array member
 *
 * In order to have a flexible array member in a union or alone in a
 * struct, it needs to be wrapped in an anonymous struct with at least 1
 * named member, but that member can be empty.
 */
#define DECLARE_FLEX_ARRAY(TYPE, NAME) \
	__DECLARE_FLEX_ARRAY(TYPE, NAME)
#endif

#ifndef __struct_group

/**
 * __struct_group() - Create a mirrored named and anonyomous struct
 *
 * @TAG: The tag name for the named sub-struct (usually empty)
 * @NAME: The identifier name of the mirrored sub-struct
 * @ATTRS: Any struct attributes (usually empty)
 * @MEMBERS: The member declarations for the mirrored structs
 *
 * Used to create an anonymous union of two structs with identical layout
 * and size: one anonymous and one named. The former's members can be used
 * normally without sub-struct naming, and the latter can be used to
 * reason about the start, end, and size of the group of struct members.
 * The named struct can also be explicitly tagged for layer reuse, as well
 * as both having struct attributes appended.
 */
#define __struct_group(TAG, NAME, ATTRS, MEMBERS...) \
	union { \
		struct { MEMBERS } ATTRS; \
		struct TAG { MEMBERS } ATTRS NAME; \
	}

#endif /* __struct_group */

#ifndef struct_group

/**
 * struct_group() - Wrap a set of declarations in a mirrored struct
 *
 * @NAME: The identifier name of the mirrored sub-struct
 * @MEMBERS: The member declarations for the mirrored structs
 *
 * Used to create an anonymous union of two structs with identical
 * layout and size: one anonymous and one named. The former can be
 * used normally without sub-struct naming, and the latter can be
 * used to reason about the start, end, and size of the group of
 * struct members.
 */
#define struct_group(NAME, MEMBERS...)	\
	__struct_group(/* no tag */, NAME, /* no attrs */, MEMBERS)

#endif /* struct_group */

/*
 * Need to include these here, otherwise we get the regular kernel ones
 * pre-including them makes it work, even though later the kernel ones
 * are included again, but they (hopefully) have the same include guard
 * ifdef/define so the second time around nothing happens
 *
 * We still keep them in the correct directory so if they don't exist in
 * the kernel (e.g. bitfield.h won't) the preprocessor can find them.
 */
#include <hdrs/linux/bitfield.h>
#include <hdrs/linux/ieee80211.h>
#include <hdrs/linux/average.h>
#include <hdrs/net/ieee80211_radiotap.h>
#define IEEE80211RADIOTAP_H 1 /* older kernels used this include protection */

/* mac80211 & backport - order matters, need this inbetween */
#include <hdrs/mac80211-bp.h>

#include <hdrs/net/codel.h>
#include <hdrs/net/mac80211.h>

/* artifacts of backports - never in upstream */
#define genl_info_snd_portid(__genl_info) (__genl_info->snd_portid)
#define NETLINK_CB_PORTID(__skb) NETLINK_CB(cb->skb).portid
#define netlink_notify_portid(__notify) __notify->portid
#define __genl_const const

static inline struct netlink_ext_ack *genl_info_extack(struct genl_info *info)
{
	return info->extack;
}

#if LINUX_VERSION_IS_LESS(5,3,0)
#define ktime_get_boottime_ns ktime_get_boot_ns
#define ktime_get_coarse_boottime_ns ktime_get_boot_ns
#endif

#ifndef NETIF_F_CSUM_MASK
#define NETIF_F_CSUM_MASK (NETIF_F_V4_CSUM | NETIF_F_V6_CSUM)
#endif

#define __genl_ro_after_init __ro_after_init

#ifndef __BUILD_BUG_ON_NOT_POWER_OF_2
#define __BUILD_BUG_ON_NOT_POWER_OF_2(...)
#endif

#define ATTRIBUTE_GROUPS_BACKPORT(_name) \
static struct BP_ATTR_GRP_STRUCT _name##_dev_attrs[ARRAY_SIZE(_name##_attrs)];\
static void init_##_name##_attrs(void)				\
{									\
	int i;								\
	for (i = 0; _name##_attrs[i]; i++)				\
		_name##_dev_attrs[i] =				\
			*container_of(_name##_attrs[i],		\
				      struct BP_ATTR_GRP_STRUCT,	\
				      attr);				\
}

#ifndef __ATTRIBUTE_GROUPS
#define __ATTRIBUTE_GROUPS(_name)				\
static const struct attribute_group *_name##_groups[] = {	\
	&_name##_group,						\
	NULL,							\
}
#endif /* __ATTRIBUTE_GROUPS */

#undef ATTRIBUTE_GROUPS
#define ATTRIBUTE_GROUPS(_name)					\
static const struct attribute_group _name##_group = {		\
	.attrs = _name##_attrs,					\
};								\
static inline void init_##_name##_attrs(void) {}		\
__ATTRIBUTE_GROUPS(_name)

int __alloc_bucket_spinlocks(spinlock_t **locks, unsigned int *lock_mask,
			     size_t max_size, unsigned int cpu_mult,
			     gfp_t gfp, const char *name,
			     struct lock_class_key *key);

#define alloc_bucket_spinlocks(locks, lock_mask, max_size, cpu_mult, gfp)    \
	({								     \
		static struct lock_class_key key;			     \
		int ret;						     \
									     \
		ret = __alloc_bucket_spinlocks(locks, lock_mask, max_size,   \
					       cpu_mult, gfp, #locks, &key); \
		ret;							\
	})
void free_bucket_spinlocks(spinlock_t *locks);

#if LINUX_VERSION_IS_LESS(4,19,0)
#ifndef atomic_fetch_add_unless
static inline int atomic_fetch_add_unless(atomic_t *v, int a, int u)
{
		return __atomic_add_unless(v, a, u);
}
#endif
#endif /* LINUX_VERSION_IS_LESS(4,19,0) */

#if LINUX_VERSION_IS_LESS(4,20,0)
static inline void rcu_head_init(struct rcu_head *rhp)
{
	rhp->func = (void *)~0L;
}

static inline bool
rcu_head_after_call_rcu(struct rcu_head *rhp, void *f)
{
	if (READ_ONCE(rhp->func) == f)
		return true;
	WARN_ON_ONCE(READ_ONCE(rhp->func) != (void *)~0L);
	return false;
}
#endif /* LINUX_VERSION_IS_LESS(4,20,0) */

#if LINUX_VERSION_IS_LESS(5,4,0)
#include <linux/pci-aspm.h>
#define EXPORT_SYMBOL_NS_GPL(sym, ns) EXPORT_SYMBOL_GPL(sym)
#define MODULE_IMPORT_NS(ns)
#endif

#if LINUX_VERSION_IS_LESS(5,5,0)
#include <linux/debugfs.h>

#define debugfs_create_xul iwl7000_debugfs_create_xul
static inline void debugfs_create_xul(const char *name, umode_t mode,
				      struct dentry *parent,
				      unsigned long *value)
{
	if (sizeof(*value) == sizeof(u32))
		debugfs_create_x32(name, mode, parent, (u32 *)value);
	else
		debugfs_create_x64(name, mode, parent, (u64 *)value);
}
#endif

#ifndef skb_list_walk_safe
#define skb_list_walk_safe(first, skb, next_skb)				\
	for ((skb) = (first), (next_skb) = (skb) ? (skb)->next : NULL; (skb);	\
	     (skb) = (next_skb), (next_skb) = (skb) ? (skb)->next : NULL)
#endif

#if LINUX_VERSION_IS_LESS(4,18,0)
#define firmware_request_nowarn(fw, name, device) request_firmware(fw, name, device)
#endif

#endif /* __IWL_CHROME */

#if LINUX_VERSION_IS_LESS(5,4,0)

/**
 * list_for_each_entry_rcu	-	iterate over rcu list of given type
 * @pos:	the type * to use as a loop cursor.
 * @head:	the head for your list.
 * @member:	the name of the list_head within the struct.
 * @cond...:	optional lockdep expression if called from non-RCU protection.
 *
 * This list-traversal primitive may safely run concurrently with
 * the _rcu list-mutation primitives such as list_add_rcu()
 * as long as the traversal is guarded by rcu_read_lock().
 */
#undef list_for_each_entry_rcu
#define list_for_each_entry_rcu(pos, head, member, cond...)		\
	for (pos = list_entry_rcu((head)->next, typeof(*pos), member); \
		&pos->member != (head); \
		pos = list_entry_rcu(pos->member.next, typeof(*pos), member))
#endif /* < 5.4 */

#if LINUX_VERSION_IS_LESS(5,7,0)
#define efi_rt_services_supported(...) efi_enabled(EFI_RUNTIME_SERVICES)
#endif

#if LINUX_VERSION_IS_LESS(5,10,0)
#define DECLARE_TRACEPOINT(tp) \
	extern struct tracepoint __tracepoint_##tp
#ifdef CONFIG_TRACEPOINTS
# define tracepoint_enabled(tp) \
	static_key_false(&(__tracepoint_##tp).key)
#else
# define tracepoint_enabled(tracepoint) false
#endif
#endif /* < 5.10 */

#if LINUX_VERSION_IS_LESS(5,11,0)

enum rfkill_hard_block_reasons {
	RFKILL_HARD_BLOCK_SIGNAL        = 1 << 0,
	RFKILL_HARD_BLOCK_NOT_OWNER     = 1 << 1,
};
#endif /* < v5.11 */

#if LINUX_VERSION_IS_LESS(5,13,0)
/* This will get enum rfkill_hard_block_reasons used below */
#include <uapi/linux/rfkill.h>

static inline void
wiphy_rfkill_set_hw_state_reason(struct wiphy *wiphy, bool blocked,
				 enum rfkill_hard_block_reasons reason)
{
	wiphy_rfkill_set_hw_state(wiphy, blocked);
}

#endif /* < v5.13 */

#if LINUX_VERSION_IS_LESS(5,14,0)
/* make this code disappear, rfkill moved from rdev to wiphy */
#define rfkill_blocked(__rkfill) false
#endif /* < v5.14 */

#if LINUX_VERSION_IS_LESS(5,17,0)
#define rfkill_soft_blocked(__rfkill) rfkill_blocked(__rfkill)

static inline void __noreturn
kthread_complete_and_exit(struct completion *c, long ret)
{
	complete_and_exit(c, ret);
}
#endif /* <v5.17 */

#if LINUX_VERSION_IS_LESS(6,1,0)
static inline u32 get_random_u32_below(u32 ceil)
{
	return prandom_u32_max(ceil);
}

static inline u32 get_random_u32_inclusive(u32 floor, u32 ceil)
{
	BUILD_BUG_ON_MSG(__builtin_constant_p(floor) && __builtin_constant_p(ceil) &&
			 (floor > ceil || ceil - floor == U32_MAX),
			 "get_random_u32_inclusive() must take floor <= ceil");
	return floor + get_random_u32_below(ceil - floor + 1);
}
#endif
