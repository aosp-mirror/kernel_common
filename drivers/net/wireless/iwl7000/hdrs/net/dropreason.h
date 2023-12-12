#ifndef __BACKPORT_DROPREASON_H
#define __BACKPORT_DROPREASON_H

#if LINUX_VERSION_IS_GEQ(6,0,0)
#include_next <net/dropreason.h>
#else
#include <linux/skbuff.h>
#endif

#if LINUX_VERSION_IS_LESS(5,17,0)
#define SKB_DROP_REASON_MAX	1
#endif

#if LINUX_VERSION_IS_LESS(5,18,0)
/*
 * Same as SKB_DROP_REASON_NOT_SPECIFIED on some kernels,
 * but that's OK since we won't report these reasons to
 * the kernel anyway until 6.4, see kfree_skb_reason().
 */
#define SKB_NOT_DROPPED_YET	0
#endif

#if LINUX_VERSION_IS_LESS(6,2,0)
#define SKB_CONSUMED		(SKB_DROP_REASON_MAX + 1)
#endif

#if LINUX_VERSION_IS_LESS(6,4,0)
enum skb_drop_reason_subsys {
	SKB_DROP_REASON_SUBSYS_CORE,
	SKB_DROP_REASON_SUBSYS_MAC80211_UNUSABLE,
	SKB_DROP_REASON_SUBSYS_MAC80211_MONITOR,
	SKB_DROP_REASON_SUBSYS_NUM
};

struct drop_reason_list {
	const char * const *reasons;
	size_t n_reasons;
};

#define SKB_DROP_REASON_SUBSYS_SHIFT	16
#define SKB_DROP_REASON_SUBSYS_MASK	0xffff0000

static inline void
drop_reasons_register_subsys(enum skb_drop_reason_subsys subsys,
			     const struct drop_reason_list *list)
{}

static inline void
drop_reasons_unregister_subsys(enum skb_drop_reason_subsys subsys)
{}
#endif /* <= 6.4 */

#endif
