#ifndef _LINUX_COMPACTION_H
#define _LINUX_COMPACTION_H

/* Return values for compact_zone() and try_to_compact_pages() */
/* compaction didn't start as it was not possible or direct reclaim was more suitable */
#define COMPACT_SKIPPED		0
/* compaction should continue to another pageblock */
#define COMPACT_CONTINUE	1
/* direct compaction partially compacted a zone and there are suitable pages */
#define COMPACT_PARTIAL		2
/* The full zone was compacted */
#define COMPACT_COMPLETE	3

#ifdef CONFIG_COMPACTION
extern int sysctl_compact_memory;
extern int sysctl_compaction_handler(struct ctl_table *table, int write,
			void __user *buffer, size_t *length, loff_t *ppos);
extern int sysctl_extfrag_threshold;
extern int sysctl_extfrag_handler(struct ctl_table *table, int write,
			void __user *buffer, size_t *length, loff_t *ppos);

extern int fragmentation_index(struct zone *zone, unsigned int order);
extern unsigned long try_to_compact_pages(struct zonelist *zonelist,
			int order, gfp_t gfp_mask, nodemask_t *mask,
			bool sync, bool *contended);
extern int compact_pgdat(pg_data_t *pgdat, int order);
extern void reset_isolation_suitable(pg_data_t *pgdat);
extern unsigned long compaction_suitable(struct zone *zone, int order);

/* Do not skip compaction more than 64 times */
#define COMPACT_MAX_DEFER_SHIFT 6

/*
 * Compaction is deferred when compaction fails to result in a page
 * allocation success. 1 << compact_defer_limit compactions are skipped up
 * to a limit of 1 << COMPACT_MAX_DEFER_SHIFT
 */
static inline void defer_compaction(struct zone *zone, int order)
{
	zone->compact_considered = 0;
	zone->compact_defer_shift++;

	if (order < zone->compact_order_failed)
		zone->compact_order_failed = order;

	if (zone->compact_defer_shift > COMPACT_MAX_DEFER_SHIFT)
		zone->compact_defer_shift = COMPACT_MAX_DEFER_SHIFT;
}

/* Returns true if compaction should be skipped this time */
static inline bool compaction_deferred(struct zone *zone, int order)
{
	unsigned long defer_limit = 1UL << zone->compact_defer_shift;

	if (order < zone->compact_order_failed)
		return false;

	/* Avoid possible overflow */
	if (++zone->compact_considered > defer_limit)
		zone->compact_considered = defer_limit;

	return zone->compact_considered < (1UL << zone->compact_defer_shift);
}

/* Returns true if restarting compaction after many failures */
static inline bool compaction_restarting(struct zone *zone, int order)
{
	if (order < zone->compact_order_failed)
		return false;

	return zone->compact_defer_shift == COMPACT_MAX_DEFER_SHIFT &&
		zone->compact_considered >= 1UL << zone->compact_defer_shift;
}

#else
static inline unsigned long try_to_compact_pages(struct zonelist *zonelist,
			int order, gfp_t gfp_mask, nodemask_t *nodemask,
			bool sync, bool *contended)
{
	return COMPACT_CONTINUE;
}

static inline int compact_pgdat(pg_data_t *pgdat, int order)
{
	return COMPACT_CONTINUE;
}

static inline void reset_isolation_suitable(pg_data_t *pgdat)
{
}

static inline unsigned long compaction_suitable(struct zone *zone, int order)
{
	return COMPACT_SKIPPED;
}

static inline void defer_compaction(struct zone *zone, int order)
{
}

static inline bool compaction_deferred(struct zone *zone, int order)
{
	return 1;
}

#endif /* CONFIG_COMPACTION */

#if defined(CONFIG_COMPACTION) && defined(CONFIG_SYSFS) && defined(CONFIG_NUMA)
extern int compaction_register_node(struct node *node);
extern void compaction_unregister_node(struct node *node);

#else

static inline int compaction_register_node(struct node *node)
{
	return 0;
}

static inline void compaction_unregister_node(struct node *node)
{
}
#endif /* CONFIG_COMPACTION && CONFIG_SYSFS && CONFIG_NUMA */

#endif /* _LINUX_COMPACTION_H */
