
#include <linux/mm.h>
#include <linux/swap.h> 
#include <linux/module.h>
#include <linux/bit_spinlock.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kmemcheck.h>
#include <linux/cpu.h>
#include <linux/cpuset.h>
#include <linux/mempolicy.h>
#include <linux/ctype.h>
#include <linux/debugobjects.h>
#include <linux/kallsyms.h>
#include <linux/memory.h>
#include <linux/math64.h>
#include <linux/fault-inject.h>
#include <linux/stacktrace.h>
#include <linux/prefetch.h>

#include <trace/events/kmem.h>
#include <htc_debug/stability/htc_report_meminfo.h>



#define SLAB_DEBUG_FLAGS (SLAB_RED_ZONE | SLAB_POISON | SLAB_STORE_USER | \
		SLAB_TRACE | SLAB_DEBUG_FREE)

static inline int kmem_cache_debug(struct kmem_cache *s)
{
#ifdef CONFIG_SLUB_DEBUG
	return unlikely(s->flags & SLAB_DEBUG_FLAGS);
#else
	return 0;
#endif
}


#undef SLUB_RESILIENCY_TEST

#undef SLUB_DEBUG_CMPXCHG

#define MIN_PARTIAL 5

#define MAX_PARTIAL 10

#ifndef CONFIG_SLUB_LIGHT_WEIGHT_DEBUG_ON
#define DEBUG_DEFAULT_FLAGS (SLAB_DEBUG_FREE | SLAB_RED_ZONE | \
				SLAB_POISON | SLAB_STORE_USER)
#else
#define DEBUG_DEFAULT_FLAGS (SLAB_DEBUG_FREE | SLAB_STORE_USER)
#endif

#define DEBUG_METADATA_FLAGS (SLAB_RED_ZONE | SLAB_POISON | SLAB_STORE_USER)

#define SLUB_NEVER_MERGE (SLAB_RED_ZONE | SLAB_POISON | SLAB_STORE_USER | \
		SLAB_TRACE | SLAB_DESTROY_BY_RCU | SLAB_NOLEAKTRACE | \
		SLAB_FAILSLAB)

#define SLUB_MERGE_SAME (SLAB_DEBUG_FREE | SLAB_RECLAIM_ACCOUNT | \
		SLAB_CACHE_DMA | SLAB_NOTRACK)

#define OO_SHIFT	16
#define OO_MASK		((1 << OO_SHIFT) - 1)
#define MAX_OBJS_PER_PAGE	32767 

#define __OBJECT_POISON		0x80000000UL 
#define __CMPXCHG_DOUBLE	0x40000000UL 

static int kmem_size = sizeof(struct kmem_cache);

#ifdef CONFIG_SMP
static struct notifier_block slab_notifier;
#endif

static enum {
	DOWN,		
	PARTIAL,	
	UP,		
	SYSFS		
} slab_state = DOWN;

static DECLARE_RWSEM(slub_lock);
static LIST_HEAD(slab_caches);

#ifndef CONFIG_SLUB_LIGHT_WEIGHT_DEBUG_ON
#define TRACK_ADDRS_COUNT 16
struct track {
	unsigned long addr;	
#ifdef CONFIG_STACKTRACE
	unsigned long addrs[TRACK_ADDRS_COUNT];	
#endif
	int cpu;		
	int pid;		
	unsigned long when;	
};
#else
struct track {
	unsigned long addr;
};
#endif

enum track_item { TRACK_ALLOC, TRACK_FREE };

#ifdef CONFIG_SYSFS
static int sysfs_slab_add(struct kmem_cache *);
static int sysfs_slab_alias(struct kmem_cache *, const char *);
static void sysfs_slab_remove(struct kmem_cache *);

#else
static inline int sysfs_slab_add(struct kmem_cache *s) { return 0; }
static inline int sysfs_slab_alias(struct kmem_cache *s, const char *p)
							{ return 0; }
static inline void sysfs_slab_remove(struct kmem_cache *s)
{
	kfree(s->name);
	kfree(s);
}

#endif

static inline void stat(const struct kmem_cache *s, enum stat_item si)
{
#ifdef CONFIG_SLUB_STATS
	__this_cpu_inc(s->cpu_slab->stat[si]);
#endif
}


int slab_is_available(void)
{
	return slab_state >= UP;
}

static inline struct kmem_cache_node *get_node(struct kmem_cache *s, int node)
{
	return s->node[node];
}

static inline int check_valid_pointer(struct kmem_cache *s,
				struct page *page, const void *object)
{
	void *base;

	if (!object)
		return 1;

	base = page_address(page);
	if (object < base || object >= base + page->objects * s->size ||
		(object - base) % s->size) {
		return 0;
	}

	return 1;
}

static inline void *get_freepointer(struct kmem_cache *s, void *object)
{
	return *(void **)(object + s->offset);
}

static void prefetch_freepointer(const struct kmem_cache *s, void *object)
{
	prefetch(object + s->offset);
}

static inline void *get_freepointer_safe(struct kmem_cache *s, void *object)
{
	void *p;

#ifdef CONFIG_DEBUG_PAGEALLOC
	probe_kernel_read(&p, (void **)(object + s->offset), sizeof(p));
#else
	p = get_freepointer(s, object);
#endif
	return p;
}

static inline void set_freepointer(struct kmem_cache *s, void *object, void *fp)
{
	*(void **)(object + s->offset) = fp;
}

#define for_each_object(__p, __s, __addr, __objects) \
	for (__p = (__addr); __p < (__addr) + (__objects) * (__s)->size;\
			__p += (__s)->size)

static inline int slab_index(void *p, struct kmem_cache *s, void *addr)
{
	return (p - addr) / s->size;
}

static inline size_t slab_ksize(const struct kmem_cache *s)
{
#ifdef CONFIG_SLUB_DEBUG
	if (s->flags & (SLAB_RED_ZONE | SLAB_POISON))
		return s->objsize;

#endif
	if (s->flags & (SLAB_DESTROY_BY_RCU | SLAB_STORE_USER))
		return s->inuse;
	return s->size;
}

static inline int order_objects(int order, unsigned long size, int reserved)
{
	return ((PAGE_SIZE << order) - reserved) / size;
}

static inline struct kmem_cache_order_objects oo_make(int order,
		unsigned long size, int reserved)
{
	struct kmem_cache_order_objects x = {
		(order << OO_SHIFT) + order_objects(order, size, reserved)
	};

	return x;
}

static inline int oo_order(struct kmem_cache_order_objects x)
{
	return x.x >> OO_SHIFT;
}

static inline int oo_objects(struct kmem_cache_order_objects x)
{
	return x.x & OO_MASK;
}

static __always_inline void slab_lock(struct page *page)
{
	bit_spin_lock(PG_locked, &page->flags);
}

static __always_inline void slab_unlock(struct page *page)
{
	__bit_spin_unlock(PG_locked, &page->flags);
}

static inline bool __cmpxchg_double_slab(struct kmem_cache *s, struct page *page,
		void *freelist_old, unsigned long counters_old,
		void *freelist_new, unsigned long counters_new,
		const char *n)
{
	VM_BUG_ON(!irqs_disabled());
#if defined(CONFIG_HAVE_CMPXCHG_DOUBLE) && \
    defined(CONFIG_HAVE_ALIGNED_STRUCT_PAGE)
	if (s->flags & __CMPXCHG_DOUBLE) {
		if (cmpxchg_double(&page->freelist, &page->counters,
			freelist_old, counters_old,
			freelist_new, counters_new))
		return 1;
	} else
#endif
	{
		slab_lock(page);
		if (page->freelist == freelist_old && page->counters == counters_old) {
			page->freelist = freelist_new;
			page->counters = counters_new;
			slab_unlock(page);
			return 1;
		}
		slab_unlock(page);
	}

	cpu_relax();
	stat(s, CMPXCHG_DOUBLE_FAIL);

#ifdef SLUB_DEBUG_CMPXCHG
	printk(KERN_INFO "%s %s: cmpxchg double redo ", n, s->name);
#endif

	return 0;
}

static inline bool cmpxchg_double_slab(struct kmem_cache *s, struct page *page,
		void *freelist_old, unsigned long counters_old,
		void *freelist_new, unsigned long counters_new,
		const char *n)
{
#if defined(CONFIG_HAVE_CMPXCHG_DOUBLE) && \
    defined(CONFIG_HAVE_ALIGNED_STRUCT_PAGE)
	if (s->flags & __CMPXCHG_DOUBLE) {
		if (cmpxchg_double(&page->freelist, &page->counters,
			freelist_old, counters_old,
			freelist_new, counters_new))
		return 1;
	} else
#endif
	{
		unsigned long flags;

		local_irq_save(flags);
		slab_lock(page);
		if (page->freelist == freelist_old && page->counters == counters_old) {
			page->freelist = freelist_new;
			page->counters = counters_new;
			slab_unlock(page);
			local_irq_restore(flags);
			return 1;
		}
		slab_unlock(page);
		local_irq_restore(flags);
	}

	cpu_relax();
	stat(s, CMPXCHG_DOUBLE_FAIL);

#ifdef SLUB_DEBUG_CMPXCHG
	printk(KERN_INFO "%s %s: cmpxchg double redo ", n, s->name);
#endif

	return 0;
}

#ifdef CONFIG_SLUB_DEBUG
static void get_map(struct kmem_cache *s, struct page *page, unsigned long *map)
{
	void *p;
	void *addr = page_address(page);

	for (p = page->freelist; p; p = get_freepointer(s, p))
		set_bit(slab_index(p, s, addr), map);
}

#ifdef CONFIG_SLUB_DEBUG_ON
static int slub_debug = DEBUG_DEFAULT_FLAGS;
#else
static int slub_debug;
#endif

static char *slub_debug_slabs;
static int disable_higher_order_debug;

static void print_section(char *text, u8 *addr, unsigned int length)
{
	print_hex_dump(KERN_ERR, text, DUMP_PREFIX_ADDRESS, 16, 1, addr,
			length, 1);
}

static struct track *get_track(struct kmem_cache *s, void *object,
	enum track_item alloc)
{
	struct track *p;

	if (s->offset)
		p = object + s->offset + sizeof(void *);
	else
		p = object + s->inuse;

	return p + alloc;
}

static void set_track(struct kmem_cache *s, void *object,
			enum track_item alloc, unsigned long addr)
{
	struct track *p = get_track(s, object, alloc);

	if (addr) {
#ifndef CONFIG_SLUB_LIGHT_WEIGHT_DEBUG_ON
#ifdef CONFIG_STACKTRACE
		struct stack_trace trace;
		int i;

		trace.nr_entries = 0;
		trace.max_entries = TRACK_ADDRS_COUNT;
		trace.entries = p->addrs;
		trace.skip = 3;
		save_stack_trace(&trace);

		
		if (trace.nr_entries != 0 &&
		    trace.entries[trace.nr_entries - 1] == ULONG_MAX)
			trace.nr_entries--;

		for (i = trace.nr_entries; i < TRACK_ADDRS_COUNT; i++)
			p->addrs[i] = 0;
#endif
		p->cpu = smp_processor_id();
		p->pid = current->pid;
		p->when = jiffies;
#endif
		p->addr = addr;
	} else
		memset(p, 0, sizeof(struct track));
}

static void init_tracking(struct kmem_cache *s, void *object)
{
	if (!(s->flags & SLAB_STORE_USER))
		return;

	set_track(s, object, TRACK_FREE, 0UL);
	set_track(s, object, TRACK_ALLOC, 0UL);
}

static void print_track(const char *s, struct track *t)
{
	if (!t->addr)
		return;

#ifndef CONFIG_SLUB_LIGHT_WEIGHT_DEBUG_ON
	printk(KERN_ERR "INFO: %s in %pS age=%lu cpu=%u pid=%d\n",
		s, (void *)t->addr, jiffies - t->when, t->cpu, t->pid);
#ifdef CONFIG_STACKTRACE
	{
		int i;
		for (i = 0; i < TRACK_ADDRS_COUNT; i++)
			if (t->addrs[i])
				printk(KERN_ERR "\t%pS\n", (void *)t->addrs[i]);
			else
				break;
	}
#endif
#else
	printk(KERN_ERR "INFO: %s in %pS",s, (void *)t->addr);
#endif
}

static void print_tracking(struct kmem_cache *s, void *object)
{
	if (!(s->flags & SLAB_STORE_USER))
		return;

	print_track("Allocated", get_track(s, object, TRACK_ALLOC));
	print_track("Freed", get_track(s, object, TRACK_FREE));
}

static void print_page_info(struct page *page)
{
	printk(KERN_ERR "INFO: Slab 0x%p objects=%u used=%u fp=0x%p flags=0x%04lx\n",
		page, page->objects, page->inuse, page->freelist, page->flags);

}

static void slab_bug(struct kmem_cache *s, char *fmt, ...)
{
	va_list args;
	char buf[100];

	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	printk(KERN_ERR "========================================"
			"=====================================\n");
	printk(KERN_ERR "BUG %s (%s): %s\n", s->name, print_tainted(), buf);
	printk(KERN_ERR "----------------------------------------"
			"-------------------------------------\n\n");
}

#ifndef CONFIG_SLUB_LIGHT_WEIGHT_DEBUG_ON
static void slab_fix(struct kmem_cache *s, char *fmt, ...)
{
	va_list args;
	char buf[100];

	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	printk(KERN_ERR "FIX %s: %s\n", s->name, buf);
}
#else
static void slab_fix(struct kmem_cache *s, char *fmt, ...)
{
}
#endif

static void print_trailer(struct kmem_cache *s, struct page *page, u8 *p)
{
	unsigned int off;	
	u8 *addr = page_address(page);

	print_tracking(s, p);

	print_page_info(page);

	printk(KERN_ERR "INFO: Object 0x%p @offset=%tu fp=0x%p\n\n",
			p, p - addr, get_freepointer(s, p));

	if (p > addr + 16)
		print_section("Bytes b4 ", p - 16, 16);

	print_section("Object ", p, min_t(unsigned long, s->objsize,
				PAGE_SIZE));
	if (s->flags & SLAB_RED_ZONE)
		print_section("Redzone ", p + s->objsize,
			s->inuse - s->objsize);

	if (s->offset)
		off = s->offset + sizeof(void *);
	else
		off = s->inuse;

	if (s->flags & SLAB_STORE_USER)
		off += 2 * sizeof(struct track);

	if (off != s->size)
		
		print_section("Padding ", p + off, s->size - off);

	dump_stack();
}

static void object_err(struct kmem_cache *s, struct page *page,
			u8 *object, char *reason)
{
	slab_bug(s, "%s", reason);
	print_trailer(s, page, object);
}

static void slab_err(struct kmem_cache *s, struct page *page, char *fmt, ...)
{
	va_list args;
	char buf[100];

	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	slab_bug(s, "%s", buf);
	print_page_info(page);
	dump_stack();
}

static void init_object(struct kmem_cache *s, void *object, u8 val)
{
	u8 *p = object;

	if (s->flags & __OBJECT_POISON) {
		memset(p, POISON_FREE, s->objsize - 1);
		p[s->objsize - 1] = POISON_END;
	}

	if (s->flags & SLAB_RED_ZONE)
		memset(p + s->objsize, val, s->inuse - s->objsize);
}

static void restore_bytes(struct kmem_cache *s, char *message, u8 data,
						void *from, void *to)
{
	slab_fix(s, "Restoring 0x%p-0x%p=0x%x\n", from, to - 1, data);
	memset(from, data, to - from);
}

static int check_bytes_and_report(struct kmem_cache *s, struct page *page,
			u8 *object, char *what,
			u8 *start, unsigned int value, unsigned int bytes)
{
	u8 *fault;
	u8 *end;

	fault = memchr_inv(start, value, bytes);
	if (!fault)
		return 1;

	end = start + bytes;
	while (end > fault && end[-1] == value)
		end--;

	slab_bug(s, "%s overwritten", what);
	printk(KERN_ERR "INFO: 0x%p-0x%p. First byte 0x%x instead of 0x%x\n",
					fault, end - 1, fault[0], value);
	print_trailer(s, page, object);

	restore_bytes(s, what, value, fault, end);
	return 0;
}


static int check_pad_bytes(struct kmem_cache *s, struct page *page, u8 *p)
{
	unsigned long off = s->inuse;	

	if (s->offset)
		
		off += sizeof(void *);

	if (s->flags & SLAB_STORE_USER)
		
		off += 2 * sizeof(struct track);

	if (s->size == off)
		return 1;

	return check_bytes_and_report(s, page, p, "Object padding",
				p + off, POISON_INUSE, s->size - off);
}

static int slab_pad_check(struct kmem_cache *s, struct page *page)
{
	u8 *start;
	u8 *fault;
	u8 *end;
	int length;
	int remainder;

	if (!(s->flags & SLAB_POISON))
		return 1;

	start = page_address(page);
	length = (PAGE_SIZE << compound_order(page)) - s->reserved;
	end = start + length;
	remainder = length % s->size;
	if (!remainder)
		return 1;

	fault = memchr_inv(end - remainder, POISON_INUSE, remainder);
	if (!fault)
		return 1;
	while (end > fault && end[-1] == POISON_INUSE)
		end--;

	slab_err(s, page, "Padding overwritten. 0x%p-0x%p", fault, end - 1);
	print_section("Padding ", end - remainder, remainder);

	restore_bytes(s, "slab padding", POISON_INUSE, end - remainder, end);
	return 0;
}

static int check_object(struct kmem_cache *s, struct page *page,
					void *object, u8 val)
{
	u8 *p = object;
	u8 *endobject = object + s->objsize;

	if (s->flags & SLAB_RED_ZONE) {
		if (!check_bytes_and_report(s, page, object, "Redzone",
			endobject, val, s->inuse - s->objsize))
			return 0;
	} else {
		if ((s->flags & SLAB_POISON) && s->objsize < s->inuse) {
			check_bytes_and_report(s, page, p, "Alignment padding",
				endobject, POISON_INUSE, s->inuse - s->objsize);
		}
	}

	if (s->flags & SLAB_POISON) {
		if (val != SLUB_RED_ACTIVE && (s->flags & __OBJECT_POISON) &&
			(!check_bytes_and_report(s, page, p, "Poison", p,
					POISON_FREE, s->objsize - 1) ||
			 !check_bytes_and_report(s, page, p, "Poison",
				p + s->objsize - 1, POISON_END, 1)))
			return 0;
		check_pad_bytes(s, page, p);
	}

	if (!s->offset && val == SLUB_RED_ACTIVE)
		return 1;

	
	if (!check_valid_pointer(s, page, get_freepointer(s, p))) {
		object_err(s, page, p, "Freepointer corrupt");
#ifndef CONFIG_SLUB_LIGHT_WEIGHT_DEBUG_ON
		set_freepointer(s, p, NULL);
#endif
		return 0;
	}
	return 1;
}

static int check_slab(struct kmem_cache *s, struct page *page)
{
	int maxobj;

	VM_BUG_ON(!irqs_disabled());

	if (!PageSlab(page)) {
		slab_err(s, page, "Not a valid slab page");
		return 0;
	}

	maxobj = order_objects(compound_order(page), s->size, s->reserved);
	if (page->objects > maxobj) {
		slab_err(s, page, "objects %u > max %u",
			s->name, page->objects, maxobj);
		return 0;
	}
	if (page->inuse > page->objects) {
		slab_err(s, page, "inuse %u > max %u",
			s->name, page->inuse, page->objects);
		return 0;
	}
	
	slab_pad_check(s, page);
	return 1;
}

static int on_freelist(struct kmem_cache *s, struct page *page, void *search)
{
	int nr = 0;
	void *fp;
	void *object = NULL;
	unsigned long max_objects;

	fp = page->freelist;
	while (fp && nr <= page->objects) {
		if (fp == search)
			return 1;
		if (!check_valid_pointer(s, page, fp)) {
			if (object) {
				object_err(s, page, object,
					"Freechain corrupt");
#ifndef CONFIG_SLUB_LIGHT_WEIGHT_DEBUG_ON
				set_freepointer(s, object, NULL);
#endif
				break;
			} else {
				slab_err(s, page, "Freepointer corrupt");
#ifndef CONFIG_SLUB_LIGHT_WEIGHT_DEBUG_ON
				page->freelist = NULL;
				page->inuse = page->objects;
				slab_fix(s, "Freelist cleared");
#endif
				return 0;
			}
			break;
		}
		object = fp;
		fp = get_freepointer(s, object);
		nr++;
	}

	max_objects = order_objects(compound_order(page), s->size, s->reserved);
	if (max_objects > MAX_OBJS_PER_PAGE)
		max_objects = MAX_OBJS_PER_PAGE;

	if (page->objects != max_objects) {
		slab_err(s, page, "Wrong number of objects. Found %d but "
			"should be %d", page->objects, max_objects);
#ifndef CONFIG_SLUB_LIGHT_WEIGHT_DEBUG_ON
		page->objects = max_objects;
#endif
		slab_fix(s, "Number of objects adjusted.");
	}
	if (page->inuse != page->objects - nr) {
		slab_err(s, page, "Wrong object count. Counter is %d but "
			"counted were %d", page->inuse, page->objects - nr);
#ifndef CONFIG_SLUB_LIGHT_WEIGHT_DEBUG_ON
		page->inuse = page->objects - nr;
#endif
		slab_fix(s, "Object count adjusted.");
	}
	return search == NULL;
}

static void trace(struct kmem_cache *s, struct page *page, void *object,
								int alloc)
{
	if (s->flags & SLAB_TRACE) {
		printk(KERN_INFO "TRACE %s %s 0x%p inuse=%d fp=0x%p\n",
			s->name,
			alloc ? "alloc" : "free",
			object, page->inuse,
			page->freelist);

		if (!alloc)
			print_section("Object ", (void *)object, s->objsize);

		dump_stack();
	}
}

static inline int slab_pre_alloc_hook(struct kmem_cache *s, gfp_t flags)
{
	flags &= gfp_allowed_mask;
	lockdep_trace_alloc(flags);
	might_sleep_if(flags & __GFP_WAIT);

	return should_failslab(s->objsize, flags, s->flags);
}

static inline void slab_post_alloc_hook(struct kmem_cache *s, gfp_t flags, void *object)
{
	flags &= gfp_allowed_mask;
	kmemcheck_slab_alloc(s, flags, object, slab_ksize(s));
	kmemleak_alloc_recursive(object, s->objsize, 1, s->flags, flags);
}

static inline void slab_free_hook(struct kmem_cache *s, void *x)
{
	kmemleak_free_recursive(x, s->flags);

#if defined(CONFIG_KMEMCHECK) || defined(CONFIG_LOCKDEP)
	{
		unsigned long flags;

		local_irq_save(flags);
		kmemcheck_slab_free(s, x, s->objsize);
		debug_check_no_locks_freed(x, s->objsize);
		local_irq_restore(flags);
	}
#endif
	if (!(s->flags & SLAB_DEBUG_OBJECTS))
		debug_check_no_obj_freed(x, s->objsize);
}

static void add_full(struct kmem_cache *s,
	struct kmem_cache_node *n, struct page *page)
{
	if (!(s->flags & SLAB_STORE_USER))
		return;

	list_add(&page->lru, &n->full);
}

static void remove_full(struct kmem_cache *s, struct page *page)
{
	if (!(s->flags & SLAB_STORE_USER))
		return;

	list_del(&page->lru);
}

static inline unsigned long slabs_node(struct kmem_cache *s, int node)
{
	struct kmem_cache_node *n = get_node(s, node);

	return atomic_long_read(&n->nr_slabs);
}

static inline unsigned long node_nr_slabs(struct kmem_cache_node *n)
{
	return atomic_long_read(&n->nr_slabs);
}

static inline void inc_slabs_node(struct kmem_cache *s, int node, int objects)
{
	struct kmem_cache_node *n = get_node(s, node);

	if (n) {
		atomic_long_inc(&n->nr_slabs);
		atomic_long_add(objects, &n->total_objects);
	}
}
static inline void dec_slabs_node(struct kmem_cache *s, int node, int objects)
{
	struct kmem_cache_node *n = get_node(s, node);

	atomic_long_dec(&n->nr_slabs);
	atomic_long_sub(objects, &n->total_objects);
}

static void setup_object_debug(struct kmem_cache *s, struct page *page,
								void *object)
{
	if (!(s->flags & (SLAB_STORE_USER|SLAB_RED_ZONE|__OBJECT_POISON)))
		return;

	init_object(s, object, SLUB_RED_INACTIVE);
	init_tracking(s, object);
}

static noinline int alloc_debug_processing(struct kmem_cache *s, struct page *page,
					void *object, unsigned long addr)
{
	if (!check_slab(s, page))
		goto bad;

	if (!check_valid_pointer(s, page, object)) {
		object_err(s, page, object, "Freelist Pointer check fails");
		goto bad;
	}

	if (!check_object(s, page, object, SLUB_RED_INACTIVE))
		goto bad;

	
	if (s->flags & SLAB_STORE_USER)
		set_track(s, object, TRACK_ALLOC, addr);
	trace(s, page, object, 1);
	init_object(s, object, SLUB_RED_ACTIVE);
	return 1;

bad:
#ifndef CONFIG_SLUB_LIGHT_WEIGHT_DEBUG_ON
	if (PageSlab(page)) {
		slab_fix(s, "Marking all objects used");
		page->inuse = page->objects;
		page->freelist = NULL;
	}
	return 0;
#else
	return 1;
#endif
}

static noinline int free_debug_processing(struct kmem_cache *s,
		 struct page *page, void *object, unsigned long addr)
{
	unsigned long flags;
	int rc = 0;

	local_irq_save(flags);
	slab_lock(page);

	if (!check_slab(s, page))
		goto fail;

	if (!check_valid_pointer(s, page, object)) {
		slab_err(s, page, "Invalid object pointer 0x%p", object);
		goto fail;
	}

	if (on_freelist(s, page, object)) {
		object_err(s, page, object, "Object already free");
		goto fail;
	}

	if (!check_object(s, page, object, SLUB_RED_ACTIVE))
		goto out;

	if (unlikely(s != page->slab)) {
		if (!PageSlab(page)) {
			slab_err(s, page, "Attempt to free object(0x%p) "
				"outside of slab", object);
		} else if (!page->slab) {
			printk(KERN_ERR
				"SLUB <none>: no slab for object 0x%p.\n",
						object);
			dump_stack();
		} else
			object_err(s, page, object,
					"page slab pointer corrupt.");
		goto fail;
	}

	if (s->flags & SLAB_STORE_USER)
		set_track(s, object, TRACK_FREE, addr);
	trace(s, page, object, 0);
	init_object(s, object, SLUB_RED_INACTIVE);
	rc = 1;
out:
	slab_unlock(page);
	local_irq_restore(flags);
#ifndef CONFIG_SLUB_LIGHT_WEIGHT_DEBUG_ON
	return rc;
#else
	return 1;
#endif

fail:
	slab_fix(s, "Object at 0x%p not freed", object);
	goto out;
}

static int __init setup_slub_debug(char *str)
{
	slub_debug = DEBUG_DEFAULT_FLAGS;
	if (*str++ != '=' || !*str)
		goto out;

	if (*str == ',')
		goto check_slabs;

	if (tolower(*str) == 'o') {
		disable_higher_order_debug = 1;
		goto out;
	}

	slub_debug = 0;
	if (*str == '-')
		goto out;

	for (; *str && *str != ','; str++) {
		switch (tolower(*str)) {
		case 'f':
			slub_debug |= SLAB_DEBUG_FREE;
			break;
		case 'z':
			slub_debug |= SLAB_RED_ZONE;
			break;
		case 'p':
			slub_debug |= SLAB_POISON;
			break;
		case 'u':
			slub_debug |= SLAB_STORE_USER;
			break;
		case 't':
			slub_debug |= SLAB_TRACE;
			break;
		case 'a':
			slub_debug |= SLAB_FAILSLAB;
			break;
		default:
			printk(KERN_ERR "slub_debug option '%c' "
				"unknown. skipped\n", *str);
		}
	}

check_slabs:
	if (*str == ',')
		slub_debug_slabs = str + 1;
out:
	return 1;
}

__setup("slub_debug", setup_slub_debug);

static unsigned long kmem_cache_flags(unsigned long objsize,
	unsigned long flags, const char *name,
	void (*ctor)(void *))
{
	if (slub_debug && (!slub_debug_slabs ||
		!strncmp(slub_debug_slabs, name, strlen(slub_debug_slabs))))
		flags |= slub_debug;

	return flags;
}
#else
static inline void setup_object_debug(struct kmem_cache *s,
			struct page *page, void *object) {}

static inline int alloc_debug_processing(struct kmem_cache *s,
	struct page *page, void *object, unsigned long addr) { return 0; }

static inline int free_debug_processing(struct kmem_cache *s,
	struct page *page, void *object, unsigned long addr) { return 0; }

static inline int slab_pad_check(struct kmem_cache *s, struct page *page)
			{ return 1; }
static inline int check_object(struct kmem_cache *s, struct page *page,
			void *object, u8 val) { return 1; }
static inline void add_full(struct kmem_cache *s, struct kmem_cache_node *n,
					struct page *page) {}
static inline void remove_full(struct kmem_cache *s, struct page *page) {}
static inline unsigned long kmem_cache_flags(unsigned long objsize,
	unsigned long flags, const char *name,
	void (*ctor)(void *))
{
	return flags;
}
#define slub_debug 0

#define disable_higher_order_debug 0

static inline unsigned long slabs_node(struct kmem_cache *s, int node)
							{ return 0; }
static inline unsigned long node_nr_slabs(struct kmem_cache_node *n)
							{ return 0; }
static inline void inc_slabs_node(struct kmem_cache *s, int node,
							int objects) {}
static inline void dec_slabs_node(struct kmem_cache *s, int node,
							int objects) {}

static inline int slab_pre_alloc_hook(struct kmem_cache *s, gfp_t flags)
							{ return 0; }

static inline void slab_post_alloc_hook(struct kmem_cache *s, gfp_t flags,
		void *object) {}

static inline void slab_free_hook(struct kmem_cache *s, void *x) {}

#endif 

static inline struct page *alloc_slab_page(gfp_t flags, int node,
					struct kmem_cache_order_objects oo)
{
	int order = oo_order(oo);

	flags |= __GFP_NOTRACK;

	if (node == NUMA_NO_NODE)
		return alloc_pages(flags, order);
	else
		return alloc_pages_exact_node(node, flags, order);
}

static struct page *allocate_slab(struct kmem_cache *s, gfp_t flags, int node)
{
	struct page *page;
	struct kmem_cache_order_objects oo = s->oo;
	gfp_t alloc_gfp;

	flags &= gfp_allowed_mask;

	if (flags & __GFP_WAIT)
		local_irq_enable();

	flags |= s->allocflags;

	alloc_gfp = (flags | __GFP_NOWARN | __GFP_NORETRY) & ~__GFP_NOFAIL;

	page = alloc_slab_page(alloc_gfp, node, oo);
	if (unlikely(!page)) {
		oo = s->min;
		page = alloc_slab_page(flags, node, oo);

		if (page)
			stat(s, ORDER_FALLBACK);
	}

	if (flags & __GFP_WAIT)
		local_irq_disable();

	if (!page)
		return NULL;

	if (kmemcheck_enabled
		&& !(s->flags & (SLAB_NOTRACK | DEBUG_DEFAULT_FLAGS))) {
		int pages = 1 << oo_order(oo);

		kmemcheck_alloc_shadow(page, oo_order(oo), flags, node);

		if (s->ctor)
			kmemcheck_mark_uninitialized_pages(page, pages);
		else
			kmemcheck_mark_unallocated_pages(page, pages);
	}

	page->objects = oo_objects(oo);
	mod_zone_page_state(page_zone(page),
		(s->flags & SLAB_RECLAIM_ACCOUNT) ?
		NR_SLAB_RECLAIMABLE : NR_SLAB_UNRECLAIMABLE,
		1 << oo_order(oo));

	return page;
}

static void setup_object(struct kmem_cache *s, struct page *page,
				void *object)
{
	setup_object_debug(s, page, object);
	if (unlikely(s->ctor))
		s->ctor(object);
}

static struct page *new_slab(struct kmem_cache *s, gfp_t flags, int node)
{
	struct page *page;
	void *start;
	void *last;
	void *p;

	BUG_ON(flags & GFP_SLAB_BUG_MASK);

	page = allocate_slab(s,
		flags & (GFP_RECLAIM_MASK | GFP_CONSTRAINT_MASK), node);
	if (!page)
		goto out;

	inc_slabs_node(s, page_to_nid(page), page->objects);
	page->slab = s;
	page->flags |= 1 << PG_slab;

	start = page_address(page);

	if (unlikely(s->flags & SLAB_POISON))
		memset(start, POISON_INUSE, PAGE_SIZE << compound_order(page));

	last = start;
	for_each_object(p, s, start, page->objects) {
		setup_object(s, page, last);
		set_freepointer(s, last, p);
		last = p;
	}
	setup_object(s, page, last);
	set_freepointer(s, last, NULL);

	page->freelist = start;
	page->inuse = page->objects;
	page->frozen = 1;
out:
	return page;
}

static void __free_slab(struct kmem_cache *s, struct page *page)
{
	int order = compound_order(page);
	int pages = 1 << order;

	if (kmem_cache_debug(s)) {
		void *p;

		slab_pad_check(s, page);
		for_each_object(p, s, page_address(page),
						page->objects)
			check_object(s, page, p, SLUB_RED_INACTIVE);
	}

	kmemcheck_free_shadow(page, compound_order(page));

	mod_zone_page_state(page_zone(page),
		(s->flags & SLAB_RECLAIM_ACCOUNT) ?
		NR_SLAB_RECLAIMABLE : NR_SLAB_UNRECLAIMABLE,
		-pages);

	__ClearPageSlab(page);
	reset_page_mapcount(page);
	if (current->reclaim_state)
		current->reclaim_state->reclaimed_slab += pages;
	__free_pages(page, order);
}

#define need_reserve_slab_rcu						\
	(sizeof(((struct page *)NULL)->lru) < sizeof(struct rcu_head))

static void rcu_free_slab(struct rcu_head *h)
{
	struct page *page;

	if (need_reserve_slab_rcu)
		page = virt_to_head_page(h);
	else
		page = container_of((struct list_head *)h, struct page, lru);

	__free_slab(page->slab, page);
}

static void free_slab(struct kmem_cache *s, struct page *page)
{
	if (unlikely(s->flags & SLAB_DESTROY_BY_RCU)) {
		struct rcu_head *head;

		if (need_reserve_slab_rcu) {
			int order = compound_order(page);
			int offset = (PAGE_SIZE << order) - s->reserved;

			VM_BUG_ON(s->reserved != sizeof(*head));
			head = page_address(page) + offset;
		} else {
			head = (void *)&page->lru;
		}

		call_rcu(head, rcu_free_slab);
	} else
		__free_slab(s, page);
}

static void discard_slab(struct kmem_cache *s, struct page *page)
{
	dec_slabs_node(s, page_to_nid(page), page->objects);
	free_slab(s, page);
}

static inline void add_partial(struct kmem_cache_node *n,
				struct page *page, int tail)
{
	n->nr_partial++;
	if (tail == DEACTIVATE_TO_TAIL)
		list_add_tail(&page->lru, &n->partial);
	else
		list_add(&page->lru, &n->partial);
}

static inline void remove_partial(struct kmem_cache_node *n,
					struct page *page)
{
	list_del(&page->lru);
	n->nr_partial--;
}

static inline void *acquire_slab(struct kmem_cache *s,
		struct kmem_cache_node *n, struct page *page,
		int mode)
{
	void *freelist;
	unsigned long counters;
	struct page new;

	do {
		freelist = page->freelist;
		counters = page->counters;
		new.counters = counters;
		if (mode)
			new.inuse = page->objects;

		VM_BUG_ON(new.frozen);
		new.frozen = 1;

	} while (!__cmpxchg_double_slab(s, page,
			freelist, counters,
			NULL, new.counters,
			"lock and freeze"));

	remove_partial(n, page);
	return freelist;
}

static int put_cpu_partial(struct kmem_cache *s, struct page *page, int drain);

static void *get_partial_node(struct kmem_cache *s,
		struct kmem_cache_node *n, struct kmem_cache_cpu *c)
{
	struct page *page, *page2;
	void *object = NULL;

	if (!n || !n->nr_partial)
		return NULL;

	spin_lock(&n->list_lock);
	list_for_each_entry_safe(page, page2, &n->partial, lru) {
		void *t = acquire_slab(s, n, page, object == NULL);
		int available;

		if (!t)
			break;

		if (!object) {
			c->page = page;
			c->node = page_to_nid(page);
			stat(s, ALLOC_FROM_PARTIAL);
			object = t;
			available =  page->objects - page->inuse;
		} else {
			page->freelist = t;
			available = put_cpu_partial(s, page, 0);
			stat(s, CPU_PARTIAL_NODE);
		}
		if (kmem_cache_debug(s) || available > s->cpu_partial / 2)
			break;

	}
	spin_unlock(&n->list_lock);
	return object;
}

static struct page *get_any_partial(struct kmem_cache *s, gfp_t flags,
		struct kmem_cache_cpu *c)
{
#ifdef CONFIG_NUMA
	struct zonelist *zonelist;
	struct zoneref *z;
	struct zone *zone;
	enum zone_type high_zoneidx = gfp_zone(flags);
	void *object;
	unsigned int cpuset_mems_cookie;

	if (!s->remote_node_defrag_ratio ||
			get_cycles() % 1024 > s->remote_node_defrag_ratio)
		return NULL;

	do {
		cpuset_mems_cookie = get_mems_allowed();
		zonelist = node_zonelist(slab_node(current->mempolicy), flags);
		for_each_zone_zonelist(zone, z, zonelist, high_zoneidx) {
			struct kmem_cache_node *n;

			n = get_node(s, zone_to_nid(zone));

			if (n && cpuset_zone_allowed_hardwall(zone, flags) &&
					n->nr_partial > s->min_partial) {
				object = get_partial_node(s, n, c);
				if (object) {
					put_mems_allowed(cpuset_mems_cookie);
					return object;
				}
			}
		}
	} while (!put_mems_allowed(cpuset_mems_cookie));
#endif
	return NULL;
}

static void *get_partial(struct kmem_cache *s, gfp_t flags, int node,
		struct kmem_cache_cpu *c)
{
	void *object;
	int searchnode = (node == NUMA_NO_NODE) ? numa_node_id() : node;

	object = get_partial_node(s, get_node(s, searchnode), c);
	if (object || node != NUMA_NO_NODE)
		return object;

	return get_any_partial(s, flags, c);
}

#ifdef CONFIG_PREEMPT
#define TID_STEP  roundup_pow_of_two(CONFIG_NR_CPUS)
#else
#define TID_STEP 1
#endif

static inline unsigned long next_tid(unsigned long tid)
{
	return tid + TID_STEP;
}

static inline unsigned int tid_to_cpu(unsigned long tid)
{
	return tid % TID_STEP;
}

static inline unsigned long tid_to_event(unsigned long tid)
{
	return tid / TID_STEP;
}

static inline unsigned int init_tid(int cpu)
{
	return cpu;
}

static inline void note_cmpxchg_failure(const char *n,
		const struct kmem_cache *s, unsigned long tid)
{
#ifdef SLUB_DEBUG_CMPXCHG
	unsigned long actual_tid = __this_cpu_read(s->cpu_slab->tid);

	printk(KERN_INFO "%s %s: cmpxchg redo ", n, s->name);

#ifdef CONFIG_PREEMPT
	if (tid_to_cpu(tid) != tid_to_cpu(actual_tid))
		printk("due to cpu change %d -> %d\n",
			tid_to_cpu(tid), tid_to_cpu(actual_tid));
	else
#endif
	if (tid_to_event(tid) != tid_to_event(actual_tid))
		printk("due to cpu running other code. Event %ld->%ld\n",
			tid_to_event(tid), tid_to_event(actual_tid));
	else
		printk("for unknown reason: actual=%lx was=%lx target=%lx\n",
			actual_tid, tid, next_tid(tid));
#endif
	stat(s, CMPXCHG_DOUBLE_CPU_FAIL);
}

void init_kmem_cache_cpus(struct kmem_cache *s)
{
	int cpu;

	for_each_possible_cpu(cpu)
		per_cpu_ptr(s->cpu_slab, cpu)->tid = init_tid(cpu);
}

static void deactivate_slab(struct kmem_cache *s, struct kmem_cache_cpu *c)
{
	enum slab_modes { M_NONE, M_PARTIAL, M_FULL, M_FREE };
	struct page *page = c->page;
	struct kmem_cache_node *n = get_node(s, page_to_nid(page));
	int lock = 0;
	enum slab_modes l = M_NONE, m = M_NONE;
	void *freelist;
	void *nextfree;
	int tail = DEACTIVATE_TO_HEAD;
	struct page new;
	struct page old;

	if (page->freelist) {
		stat(s, DEACTIVATE_REMOTE_FREES);
		tail = DEACTIVATE_TO_TAIL;
	}

	c->tid = next_tid(c->tid);
	c->page = NULL;
	freelist = c->freelist;
	c->freelist = NULL;

	while (freelist && (nextfree = get_freepointer(s, freelist))) {
		void *prior;
		unsigned long counters;

		do {
			prior = page->freelist;
			counters = page->counters;
			set_freepointer(s, freelist, prior);
			new.counters = counters;
			new.inuse--;
			VM_BUG_ON(!new.frozen);

		} while (!__cmpxchg_double_slab(s, page,
			prior, counters,
			freelist, new.counters,
			"drain percpu freelist"));

		freelist = nextfree;
	}

redo:

	old.freelist = page->freelist;
	old.counters = page->counters;
	VM_BUG_ON(!old.frozen);

	
	new.counters = old.counters;
	if (freelist) {
		new.inuse--;
		set_freepointer(s, freelist, old.freelist);
		new.freelist = freelist;
	} else
		new.freelist = old.freelist;

	new.frozen = 0;

	if (!new.inuse && n->nr_partial > s->min_partial)
		m = M_FREE;
	else if (new.freelist) {
		m = M_PARTIAL;
		if (!lock) {
			lock = 1;
			spin_lock(&n->list_lock);
		}
	} else {
		m = M_FULL;
		if (kmem_cache_debug(s) && !lock) {
			lock = 1;
			spin_lock(&n->list_lock);
		}
	}

	if (l != m) {

		if (l == M_PARTIAL)

			remove_partial(n, page);

		else if (l == M_FULL)

			remove_full(s, page);

		if (m == M_PARTIAL) {

			add_partial(n, page, tail);
			stat(s, tail);

		} else if (m == M_FULL) {

			stat(s, DEACTIVATE_FULL);
			add_full(s, n, page);

		}
	}

	l = m;
	if (!__cmpxchg_double_slab(s, page,
				old.freelist, old.counters,
				new.freelist, new.counters,
				"unfreezing slab"))
		goto redo;

	if (lock)
		spin_unlock(&n->list_lock);

	if (m == M_FREE) {
		stat(s, DEACTIVATE_EMPTY);
		discard_slab(s, page);
		stat(s, FREE_SLAB);
	}
}

static void unfreeze_partials(struct kmem_cache *s)
{
	struct kmem_cache_node *n = NULL;
	struct kmem_cache_cpu *c = this_cpu_ptr(s->cpu_slab);
	struct page *page, *discard_page = NULL;

	while ((page = c->partial)) {
		enum slab_modes { M_PARTIAL, M_FREE };
		enum slab_modes l, m;
		struct page new;
		struct page old;

		c->partial = page->next;
		l = M_FREE;

		do {

			old.freelist = page->freelist;
			old.counters = page->counters;
			VM_BUG_ON(!old.frozen);

			new.counters = old.counters;
			new.freelist = old.freelist;

			new.frozen = 0;

			if (!new.inuse && (!n || n->nr_partial > s->min_partial))
				m = M_FREE;
			else {
				struct kmem_cache_node *n2 = get_node(s,
							page_to_nid(page));

				m = M_PARTIAL;
				if (n != n2) {
					if (n)
						spin_unlock(&n->list_lock);

					n = n2;
					spin_lock(&n->list_lock);
				}
			}

			if (l != m) {
				if (l == M_PARTIAL) {
					remove_partial(n, page);
					stat(s, FREE_REMOVE_PARTIAL);
				} else {
					add_partial(n, page,
						DEACTIVATE_TO_TAIL);
					stat(s, FREE_ADD_PARTIAL);
				}

				l = m;
			}

		} while (!cmpxchg_double_slab(s, page,
				old.freelist, old.counters,
				new.freelist, new.counters,
				"unfreezing slab"));

		if (m == M_FREE) {
			page->next = discard_page;
			discard_page = page;
		}
	}

	if (n)
		spin_unlock(&n->list_lock);

	while (discard_page) {
		page = discard_page;
		discard_page = discard_page->next;

		stat(s, DEACTIVATE_EMPTY);
		discard_slab(s, page);
		stat(s, FREE_SLAB);
	}
}

int put_cpu_partial(struct kmem_cache *s, struct page *page, int drain)
{
	struct page *oldpage;
	int pages;
	int pobjects;

	do {
		pages = 0;
		pobjects = 0;
		oldpage = this_cpu_read(s->cpu_slab->partial);

		if (oldpage) {
			pobjects = oldpage->pobjects;
			pages = oldpage->pages;
			if (drain && pobjects > s->cpu_partial) {
				unsigned long flags;
				local_irq_save(flags);
				unfreeze_partials(s);
				local_irq_restore(flags);
				pobjects = 0;
				pages = 0;
				stat(s, CPU_PARTIAL_DRAIN);
			}
		}

		pages++;
		pobjects += page->objects - page->inuse;

		page->pages = pages;
		page->pobjects = pobjects;
		page->next = oldpage;

	} while (this_cpu_cmpxchg(s->cpu_slab->partial, oldpage, page) != oldpage);
	return pobjects;
}

static inline void flush_slab(struct kmem_cache *s, struct kmem_cache_cpu *c)
{
	stat(s, CPUSLAB_FLUSH);
	deactivate_slab(s, c);
}

static inline void __flush_cpu_slab(struct kmem_cache *s, int cpu)
{
	struct kmem_cache_cpu *c = per_cpu_ptr(s->cpu_slab, cpu);

	if (likely(c)) {
		if (c->page)
			flush_slab(s, c);

		unfreeze_partials(s);
	}
}

static void flush_cpu_slab(void *d)
{
	struct kmem_cache *s = d;

	__flush_cpu_slab(s, smp_processor_id());
}

static bool has_cpu_slab(int cpu, void *info)
{
	struct kmem_cache *s = info;
	struct kmem_cache_cpu *c = per_cpu_ptr(s->cpu_slab, cpu);

	return c->page || c->partial;
}

static void flush_all(struct kmem_cache *s)
{
	on_each_cpu_cond(has_cpu_slab, flush_cpu_slab, s, 1, GFP_ATOMIC);
}

static inline int node_match(struct kmem_cache_cpu *c, int node)
{
#ifdef CONFIG_NUMA
	if (node != NUMA_NO_NODE && c->node != node)
		return 0;
#endif
	return 1;
}

static int count_free(struct page *page)
{
	return page->objects - page->inuse;
}

static unsigned long count_partial(struct kmem_cache_node *n,
					int (*get_count)(struct page *))
{
	unsigned long flags;
	unsigned long x = 0;
	struct page *page;

	spin_lock_irqsave(&n->list_lock, flags);
	list_for_each_entry(page, &n->partial, lru)
		x += get_count(page);
	spin_unlock_irqrestore(&n->list_lock, flags);
	return x;
}

static inline unsigned long node_nr_objs(struct kmem_cache_node *n)
{
#ifdef CONFIG_SLUB_DEBUG
	return atomic_long_read(&n->total_objects);
#else
	return 0;
#endif
}

static noinline void
slab_out_of_memory(struct kmem_cache *s, gfp_t gfpflags, int nid)
{
	int node;

	printk(KERN_WARNING
		"SLUB: Unable to allocate memory on node %d (gfp=0x%x)\n",
		nid, gfpflags);
	printk(KERN_WARNING "  cache: %s, object size: %d, buffer size: %d, "
		"default order: %d, min order: %d\n", s->name, s->objsize,
		s->size, oo_order(s->oo), oo_order(s->min));

	if (oo_order(s->min) > get_order(s->objsize))
		printk(KERN_WARNING "  %s debugging increased min order, use "
		       "slub_debug=O to disable.\n", s->name);

	for_each_online_node(node) {
		struct kmem_cache_node *n = get_node(s, node);
		unsigned long nr_slabs;
		unsigned long nr_objs;
		unsigned long nr_free;

		if (!n)
			continue;

		nr_free  = count_partial(n, count_free);
		nr_slabs = node_nr_slabs(n);
		nr_objs  = node_nr_objs(n);

		printk(KERN_WARNING
			"  node %d: slabs: %ld, objs: %ld, free: %ld\n",
			node, nr_slabs, nr_objs, nr_free);
	}
}

static inline void *new_slab_objects(struct kmem_cache *s, gfp_t flags,
			int node, struct kmem_cache_cpu **pc)
{
	void *object;
	struct kmem_cache_cpu *c;
	struct page *page = new_slab(s, flags, node);

	if (page) {
		c = __this_cpu_ptr(s->cpu_slab);
		if (c->page)
			flush_slab(s, c);

		object = page->freelist;
		page->freelist = NULL;

		stat(s, ALLOC_SLAB);
		c->node = page_to_nid(page);
		c->page = page;
		*pc = c;
	} else
		object = NULL;

	return object;
}

static inline void *get_freelist(struct kmem_cache *s, struct page *page)
{
	struct page new;
	unsigned long counters;
	void *freelist;

	do {
		freelist = page->freelist;
		counters = page->counters;
		new.counters = counters;
		VM_BUG_ON(!new.frozen);

		new.inuse = page->objects;
		new.frozen = freelist != NULL;

	} while (!cmpxchg_double_slab(s, page,
		freelist, counters,
		NULL, new.counters,
		"get_freelist"));

	return freelist;
}

static void *__slab_alloc(struct kmem_cache *s, gfp_t gfpflags, int node,
			  unsigned long addr, struct kmem_cache_cpu *c)
{
	void **object;
	unsigned long flags;

	local_irq_save(flags);
#ifdef CONFIG_PREEMPT
	c = this_cpu_ptr(s->cpu_slab);
#endif

	if (!c->page)
		goto new_slab;
redo:
	if (unlikely(!node_match(c, node))) {
		stat(s, ALLOC_NODE_MISMATCH);
		deactivate_slab(s, c);
		goto new_slab;
	}

	
	object = c->freelist;
	if (object)
		goto load_freelist;

	stat(s, ALLOC_SLOWPATH);

	object = get_freelist(s, c->page);

	if (!object) {
		c->page = NULL;
		stat(s, DEACTIVATE_BYPASS);
		goto new_slab;
	}

	stat(s, ALLOC_REFILL);

load_freelist:
	c->freelist = get_freepointer(s, object);
	c->tid = next_tid(c->tid);
	local_irq_restore(flags);
	return object;

new_slab:

	if (c->partial) {
		c->page = c->partial;
		c->partial = c->page->next;
		c->node = page_to_nid(c->page);
		stat(s, CPU_PARTIAL_ALLOC);
		c->freelist = NULL;
		goto redo;
	}

	
	object = get_partial(s, gfpflags, node, c);

	if (unlikely(!object)) {

		object = new_slab_objects(s, gfpflags, node, &c);

		if (unlikely(!object)) {
			if (!(gfpflags & __GFP_NOWARN) && printk_ratelimit())
				slab_out_of_memory(s, gfpflags, node);

			local_irq_restore(flags);
			return NULL;
		}
	}

	if (likely(!kmem_cache_debug(s)))
		goto load_freelist;

	
	if (!alloc_debug_processing(s, c->page, object, addr))
		goto new_slab;	

	c->freelist = get_freepointer(s, object);
	deactivate_slab(s, c);
	c->node = NUMA_NO_NODE;
	local_irq_restore(flags);
	return object;
}

static __always_inline void *slab_alloc(struct kmem_cache *s,
		gfp_t gfpflags, int node, unsigned long addr)
{
	void **object;
	struct kmem_cache_cpu *c;
	unsigned long tid;

	if (slab_pre_alloc_hook(s, gfpflags))
		return NULL;

redo:
	preempt_disable();
	c = __this_cpu_ptr(s->cpu_slab);

	tid = c->tid;
	preempt_enable();

	object = c->freelist;
	if (unlikely(!object || !node_match(c, node)))

		object = __slab_alloc(s, gfpflags, node, addr, c);

	else {
		void *next_object = get_freepointer_safe(s, object);

		if (unlikely(!this_cpu_cmpxchg_double(
				s->cpu_slab->freelist, s->cpu_slab->tid,
				object, tid,
				next_object, next_tid(tid)))) {

			note_cmpxchg_failure("slab_alloc", s, tid);
			goto redo;
		}
		prefetch_freepointer(s, next_object);
		stat(s, ALLOC_FASTPATH);
	}

	if (unlikely(gfpflags & __GFP_ZERO) && object)
		memset(object, 0, s->objsize);

	slab_post_alloc_hook(s, gfpflags, object);

	return object;
}

void *kmem_cache_alloc(struct kmem_cache *s, gfp_t gfpflags)
{
	void *ret = slab_alloc(s, gfpflags, NUMA_NO_NODE, _RET_IP_);

	trace_kmem_cache_alloc(_RET_IP_, ret, s->objsize, s->size, gfpflags);

	return ret;
}
EXPORT_SYMBOL(kmem_cache_alloc);

#ifdef CONFIG_TRACING
void *kmem_cache_alloc_trace(struct kmem_cache *s, gfp_t gfpflags, size_t size)
{
	void *ret = slab_alloc(s, gfpflags, NUMA_NO_NODE, _RET_IP_);
	trace_kmalloc(_RET_IP_, ret, size, s->size, gfpflags);
	return ret;
}
EXPORT_SYMBOL(kmem_cache_alloc_trace);

void *kmalloc_order_trace(size_t size, gfp_t flags, unsigned int order)
{
	void *ret = kmalloc_order(size, flags, order);

	if (ret) {
		struct page *page = virt_to_page(ret);
		kmalloc_count(page, 1);
	}
	trace_kmalloc(_RET_IP_, ret, size, PAGE_SIZE << order, flags);
	return ret;
}
EXPORT_SYMBOL(kmalloc_order_trace);
#endif

#ifdef CONFIG_NUMA
void *kmem_cache_alloc_node(struct kmem_cache *s, gfp_t gfpflags, int node)
{
	void *ret = slab_alloc(s, gfpflags, node, _RET_IP_);

	trace_kmem_cache_alloc_node(_RET_IP_, ret,
				    s->objsize, s->size, gfpflags, node);

	return ret;
}
EXPORT_SYMBOL(kmem_cache_alloc_node);

#ifdef CONFIG_TRACING
void *kmem_cache_alloc_node_trace(struct kmem_cache *s,
				    gfp_t gfpflags,
				    int node, size_t size)
{
	void *ret = slab_alloc(s, gfpflags, node, _RET_IP_);

	trace_kmalloc_node(_RET_IP_, ret,
			   size, s->size, gfpflags, node);
	return ret;
}
EXPORT_SYMBOL(kmem_cache_alloc_node_trace);
#endif
#endif

static void __slab_free(struct kmem_cache *s, struct page *page,
			void *x, unsigned long addr)
{
	void *prior;
	void **object = (void *)x;
	int was_frozen;
	int inuse;
	struct page new;
	unsigned long counters;
	struct kmem_cache_node *n = NULL;
	unsigned long uninitialized_var(flags);

	stat(s, FREE_SLOWPATH);

	if (kmem_cache_debug(s) && !free_debug_processing(s, page, x, addr))
		return;

	do {
		prior = page->freelist;
		counters = page->counters;
		set_freepointer(s, object, prior);
		new.counters = counters;
		was_frozen = new.frozen;
		new.inuse--;
		if ((!new.inuse || !prior) && !was_frozen && !n) {

			if (!kmem_cache_debug(s) && !prior)

				new.frozen = 1;

			else { 

	                        n = get_node(s, page_to_nid(page));
				spin_lock_irqsave(&n->list_lock, flags);

			}
		}
		inuse = new.inuse;

	} while (!cmpxchg_double_slab(s, page,
		prior, counters,
		object, new.counters,
		"__slab_free"));

	if (likely(!n)) {

		if (new.frozen && !was_frozen) {
			put_cpu_partial(s, page, 1);
			stat(s, CPU_PARTIAL_FREE);
		}
                if (was_frozen)
                        stat(s, FREE_FROZEN);
                return;
        }

	if (was_frozen)
		stat(s, FREE_FROZEN);
	else {
		if (unlikely(!inuse && n->nr_partial > s->min_partial))
                        goto slab_empty;

		if (unlikely(!prior)) {
			remove_full(s, page);
			add_partial(n, page, DEACTIVATE_TO_TAIL);
			stat(s, FREE_ADD_PARTIAL);
		}
	}
	spin_unlock_irqrestore(&n->list_lock, flags);
	return;

slab_empty:
	if (prior) {
		remove_partial(n, page);
		stat(s, FREE_REMOVE_PARTIAL);
	} else
		
		remove_full(s, page);

	spin_unlock_irqrestore(&n->list_lock, flags);
	stat(s, FREE_SLAB);
	discard_slab(s, page);
}

static __always_inline void slab_free(struct kmem_cache *s,
			struct page *page, void *x, unsigned long addr)
{
	void **object = (void *)x;
	struct kmem_cache_cpu *c;
	unsigned long tid;

	slab_free_hook(s, x);

redo:
	preempt_disable();
	c = __this_cpu_ptr(s->cpu_slab);

	tid = c->tid;
	preempt_enable();

	if (likely(page == c->page)) {
		set_freepointer(s, object, c->freelist);

		if (unlikely(!this_cpu_cmpxchg_double(
				s->cpu_slab->freelist, s->cpu_slab->tid,
				c->freelist, tid,
				object, next_tid(tid)))) {

			note_cmpxchg_failure("slab_free", s, tid);
			goto redo;
		}
		stat(s, FREE_FASTPATH);
	} else
		__slab_free(s, page, x, addr);

}

void kmem_cache_free(struct kmem_cache *s, void *x)
{
	struct page *page;

	page = virt_to_head_page(x);

	slab_free(s, page, x, _RET_IP_);

	trace_kmem_cache_free(_RET_IP_, x);
}
EXPORT_SYMBOL(kmem_cache_free);


static int slub_min_order;
static int slub_max_order = PAGE_ALLOC_COSTLY_ORDER;
static int slub_min_objects;

static int slub_nomerge;

static inline int slab_order(int size, int min_objects,
				int max_order, int fract_leftover, int reserved)
{
	int order;
	int rem;
	int min_order = slub_min_order;

	if (order_objects(min_order, size, reserved) > MAX_OBJS_PER_PAGE)
		return get_order(size * MAX_OBJS_PER_PAGE) - 1;

	for (order = max(min_order,
				fls(min_objects * size - 1) - PAGE_SHIFT);
			order <= max_order; order++) {

		unsigned long slab_size = PAGE_SIZE << order;

		if (slab_size < min_objects * size + reserved)
			continue;

		rem = (slab_size - reserved) % size;

		if (rem <= slab_size / fract_leftover)
			break;

	}

	return order;
}

static inline int calculate_order(int size, int reserved)
{
	int order;
	int min_objects;
	int fraction;
	int max_objects;

	min_objects = slub_min_objects;
	if (!min_objects)
		min_objects = 4 * (fls(nr_cpu_ids) + 1);
	max_objects = order_objects(slub_max_order, size, reserved);
	min_objects = min(min_objects, max_objects);

	while (min_objects > 1) {
		fraction = 16;
		while (fraction >= 4) {
			order = slab_order(size, min_objects,
					slub_max_order, fraction, reserved);
			if (order <= slub_max_order)
				return order;
			fraction /= 2;
		}
		min_objects--;
	}

	order = slab_order(size, 1, slub_max_order, 1, reserved);
	if (order <= slub_max_order)
		return order;

	order = slab_order(size, 1, MAX_ORDER, 1, reserved);
	if (order < MAX_ORDER)
		return order;
	return -ENOSYS;
}

static unsigned long calculate_alignment(unsigned long flags,
		unsigned long align, unsigned long size)
{
	if (flags & SLAB_HWCACHE_ALIGN) {
		unsigned long ralign = cache_line_size();
		while (size <= ralign / 2)
			ralign /= 2;
		align = max(align, ralign);
	}

	if (align < ARCH_SLAB_MINALIGN)
		align = ARCH_SLAB_MINALIGN;

	return ALIGN(align, sizeof(void *));
}

static void
init_kmem_cache_node(struct kmem_cache_node *n, struct kmem_cache *s)
{
	n->nr_partial = 0;
	spin_lock_init(&n->list_lock);
	INIT_LIST_HEAD(&n->partial);
#ifdef CONFIG_SLUB_DEBUG
	atomic_long_set(&n->nr_slabs, 0);
	atomic_long_set(&n->total_objects, 0);
	INIT_LIST_HEAD(&n->full);
#endif
}

static inline int alloc_kmem_cache_cpus(struct kmem_cache *s)
{
	BUILD_BUG_ON(PERCPU_DYNAMIC_EARLY_SIZE <
			SLUB_PAGE_SHIFT * sizeof(struct kmem_cache_cpu));

	s->cpu_slab = __alloc_percpu(sizeof(struct kmem_cache_cpu),
				     2 * sizeof(void *));

	if (!s->cpu_slab)
		return 0;

	init_kmem_cache_cpus(s);

	return 1;
}

static struct kmem_cache *kmem_cache_node;

static void early_kmem_cache_node_alloc(int node)
{
	struct page *page;
	struct kmem_cache_node *n;

	BUG_ON(kmem_cache_node->size < sizeof(struct kmem_cache_node));

	page = new_slab(kmem_cache_node, GFP_NOWAIT, node);

	BUG_ON(!page);
	if (page_to_nid(page) != node) {
		printk(KERN_ERR "SLUB: Unable to allocate memory from "
				"node %d\n", node);
		printk(KERN_ERR "SLUB: Allocating a useless per node structure "
				"in order to be able to continue\n");
	}

	n = page->freelist;
	BUG_ON(!n);
	page->freelist = get_freepointer(kmem_cache_node, n);
	page->inuse = 1;
	page->frozen = 0;
	kmem_cache_node->node[node] = n;
#ifdef CONFIG_SLUB_DEBUG
	init_object(kmem_cache_node, n, SLUB_RED_ACTIVE);
	init_tracking(kmem_cache_node, n);
#endif
	init_kmem_cache_node(n, kmem_cache_node);
	inc_slabs_node(kmem_cache_node, node, page->objects);

	add_partial(n, page, DEACTIVATE_TO_HEAD);
}

static void free_kmem_cache_nodes(struct kmem_cache *s)
{
	int node;

	for_each_node_state(node, N_NORMAL_MEMORY) {
		struct kmem_cache_node *n = s->node[node];

		if (n)
			kmem_cache_free(kmem_cache_node, n);

		s->node[node] = NULL;
	}
}

static int init_kmem_cache_nodes(struct kmem_cache *s)
{
	int node;

	for_each_node_state(node, N_NORMAL_MEMORY) {
		struct kmem_cache_node *n;

		if (slab_state == DOWN) {
			early_kmem_cache_node_alloc(node);
			continue;
		}
		n = kmem_cache_alloc_node(kmem_cache_node,
						GFP_KERNEL, node);

		if (!n) {
			free_kmem_cache_nodes(s);
			return 0;
		}

		s->node[node] = n;
		init_kmem_cache_node(n, s);
	}
	return 1;
}

static void set_min_partial(struct kmem_cache *s, unsigned long min)
{
	if (min < MIN_PARTIAL)
		min = MIN_PARTIAL;
	else if (min > MAX_PARTIAL)
		min = MAX_PARTIAL;
	s->min_partial = min;
}

static int calculate_sizes(struct kmem_cache *s, int forced_order)
{
	unsigned long flags = s->flags;
	unsigned long size = s->objsize;
	unsigned long align = s->align;
	int order;

	size = ALIGN(size, sizeof(void *));

#ifdef CONFIG_SLUB_DEBUG
	if ((flags & SLAB_POISON) && !(flags & SLAB_DESTROY_BY_RCU) &&
			!s->ctor)
		s->flags |= __OBJECT_POISON;
	else
		s->flags &= ~__OBJECT_POISON;


	if ((flags & SLAB_RED_ZONE) && size == s->objsize)
		size += sizeof(void *);
#endif

	s->inuse = size;

	if (((flags & (SLAB_DESTROY_BY_RCU | SLAB_POISON)) ||
		s->ctor)) {
		s->offset = size;
		size += sizeof(void *);
	}

#ifdef CONFIG_SLUB_DEBUG
	if (flags & SLAB_STORE_USER)
		size += 2 * sizeof(struct track);

	if (flags & SLAB_RED_ZONE)
		size += sizeof(void *);
#endif

	align = calculate_alignment(flags, align, s->objsize);
	s->align = align;

	size = ALIGN(size, align);
	s->size = size;
	if (forced_order >= 0)
		order = forced_order;
	else
		order = calculate_order(size, s->reserved);

	if (order < 0)
		return 0;

	s->allocflags = 0;
	if (order)
		s->allocflags |= __GFP_COMP;

	if (s->flags & SLAB_CACHE_DMA)
		s->allocflags |= SLUB_DMA;

	if (s->flags & SLAB_RECLAIM_ACCOUNT)
		s->allocflags |= __GFP_RECLAIMABLE;

	s->oo = oo_make(order, size, s->reserved);
	s->min = oo_make(get_order(size), size, s->reserved);
	if (oo_objects(s->oo) > oo_objects(s->max))
		s->max = s->oo;

	return !!oo_objects(s->oo);

}

static int kmem_cache_open(struct kmem_cache *s,
		const char *name, size_t size,
		size_t align, unsigned long flags,
		void (*ctor)(void *))
{
	memset(s, 0, kmem_size);
	s->name = name;
	s->ctor = ctor;
	s->objsize = size;
	s->align = align;
	s->flags = kmem_cache_flags(size, flags, name, ctor);
	s->reserved = 0;

	if (need_reserve_slab_rcu && (s->flags & SLAB_DESTROY_BY_RCU))
		s->reserved = sizeof(struct rcu_head);

	if (!calculate_sizes(s, -1))
		goto error;
	if (disable_higher_order_debug) {
		if (get_order(s->size) > get_order(s->objsize)) {
			s->flags &= ~DEBUG_METADATA_FLAGS;
			s->offset = 0;
			if (!calculate_sizes(s, -1))
				goto error;
		}
	}

#if defined(CONFIG_HAVE_CMPXCHG_DOUBLE) && \
    defined(CONFIG_HAVE_ALIGNED_STRUCT_PAGE)
	if (system_has_cmpxchg_double() && (s->flags & SLAB_DEBUG_FLAGS) == 0)
		
		s->flags |= __CMPXCHG_DOUBLE;
#endif

	set_min_partial(s, ilog2(s->size) / 2);

	if (kmem_cache_debug(s))
		s->cpu_partial = 0;
	else if (s->size >= PAGE_SIZE)
		s->cpu_partial = 2;
	else if (s->size >= 1024)
		s->cpu_partial = 6;
	else if (s->size >= 256)
		s->cpu_partial = 13;
	else
		s->cpu_partial = 30;

	s->refcount = 1;
#ifdef CONFIG_NUMA
	s->remote_node_defrag_ratio = 1000;
#endif
	if (!init_kmem_cache_nodes(s))
		goto error;

	if (alloc_kmem_cache_cpus(s))
		return 1;

	free_kmem_cache_nodes(s);
error:
	if (flags & SLAB_PANIC)
		panic("Cannot create slab %s size=%lu realsize=%u "
			"order=%u offset=%u flags=%lx\n",
			s->name, (unsigned long)size, s->size, oo_order(s->oo),
			s->offset, flags);
	return 0;
}

unsigned int kmem_cache_size(struct kmem_cache *s)
{
	return s->objsize;
}
EXPORT_SYMBOL(kmem_cache_size);

static void list_slab_objects(struct kmem_cache *s, struct page *page,
							const char *text)
{
#ifdef CONFIG_SLUB_DEBUG
	void *addr = page_address(page);
	void *p;
	unsigned long *map = kzalloc(BITS_TO_LONGS(page->objects) *
				     sizeof(long), GFP_ATOMIC);
	if (!map)
		return;
	slab_err(s, page, "%s", text);
	slab_lock(page);

	get_map(s, page, map);
	for_each_object(p, s, addr, page->objects) {

		if (!test_bit(slab_index(p, s, addr), map)) {
			printk(KERN_ERR "INFO: Object 0x%p @offset=%tu\n",
							p, p - addr);
			print_tracking(s, p);
		}
	}
	slab_unlock(page);
	kfree(map);
#endif
}

static void free_partial(struct kmem_cache *s, struct kmem_cache_node *n)
{
	struct page *page, *h;

	list_for_each_entry_safe(page, h, &n->partial, lru) {
		if (!page->inuse) {
			remove_partial(n, page);
			discard_slab(s, page);
		} else {
			list_slab_objects(s, page,
				"Objects remaining on kmem_cache_close()");
		}
	}
}

static inline int kmem_cache_close(struct kmem_cache *s)
{
	int node;

	flush_all(s);
	free_percpu(s->cpu_slab);
	
	for_each_node_state(node, N_NORMAL_MEMORY) {
		struct kmem_cache_node *n = get_node(s, node);

		free_partial(s, n);
		if (n->nr_partial || slabs_node(s, node))
			return 1;
	}
	free_kmem_cache_nodes(s);
	return 0;
}

void kmem_cache_destroy(struct kmem_cache *s)
{
	down_write(&slub_lock);
	s->refcount--;
	if (!s->refcount) {
		list_del(&s->list);
		up_write(&slub_lock);
		if (kmem_cache_close(s)) {
			printk(KERN_ERR "SLUB %s: %s called for cache that "
				"still has objects.\n", s->name, __func__);
			dump_stack();
		}
		if (s->flags & SLAB_DESTROY_BY_RCU)
			rcu_barrier();
		sysfs_slab_remove(s);
	} else
		up_write(&slub_lock);
}
EXPORT_SYMBOL(kmem_cache_destroy);


struct kmem_cache *kmalloc_caches[SLUB_PAGE_SHIFT];
EXPORT_SYMBOL(kmalloc_caches);

static struct kmem_cache *kmem_cache;

#ifdef CONFIG_ZONE_DMA
static struct kmem_cache *kmalloc_dma_caches[SLUB_PAGE_SHIFT];
#endif

static int __init setup_slub_min_order(char *str)
{
	get_option(&str, &slub_min_order);

	return 1;
}

__setup("slub_min_order=", setup_slub_min_order);

static int __init setup_slub_max_order(char *str)
{
	get_option(&str, &slub_max_order);
	slub_max_order = min(slub_max_order, MAX_ORDER - 1);

	return 1;
}

__setup("slub_max_order=", setup_slub_max_order);

static int __init setup_slub_min_objects(char *str)
{
	get_option(&str, &slub_min_objects);

	return 1;
}

__setup("slub_min_objects=", setup_slub_min_objects);

static int __init setup_slub_nomerge(char *str)
{
	slub_nomerge = 1;
	return 1;
}

__setup("slub_nomerge", setup_slub_nomerge);

static struct kmem_cache *__init create_kmalloc_cache(const char *name,
						int size, unsigned int flags)
{
	struct kmem_cache *s;

	s = kmem_cache_alloc(kmem_cache, GFP_NOWAIT);

	if (!kmem_cache_open(s, name, size, ARCH_KMALLOC_MINALIGN,
								flags, NULL))
		goto panic;

	list_add(&s->list, &slab_caches);
	return s;

panic:
	panic("Creation of kmalloc slab %s size=%d failed.\n", name, size);
	return NULL;
}

static s8 size_index[24] = {
	3,	
	4,	
	5,	
	5,	
	6,	
	6,	
	6,	
	6,	
	1,	
	1,	
	1,	
	1,	
	7,	
	7,	
	7,	
	7,	
	2,	
	2,	
	2,	
	2,	
	2,	
	2,	
	2,	
	2	
};

static inline int size_index_elem(size_t bytes)
{
	return (bytes - 1) / 8;
}

static struct kmem_cache *get_slab(size_t size, gfp_t flags)
{
	int index;

	if (size <= 192) {
		if (!size)
			return ZERO_SIZE_PTR;

		index = size_index[size_index_elem(size)];
	} else
		index = fls(size - 1);

#ifdef CONFIG_ZONE_DMA
	if (unlikely((flags & SLUB_DMA)))
		return kmalloc_dma_caches[index];

#endif
	return kmalloc_caches[index];
}

void *__kmalloc(size_t size, gfp_t flags)
{
	struct kmem_cache *s;
	void *ret;

	if (unlikely(size > SLUB_MAX_SIZE))
		return kmalloc_large(size, flags);

	s = get_slab(size, flags);

	if (unlikely(ZERO_OR_NULL_PTR(s)))
		return s;

	ret = slab_alloc(s, flags, NUMA_NO_NODE, _RET_IP_);

	trace_kmalloc(_RET_IP_, ret, size, s->size, flags);

	return ret;
}
EXPORT_SYMBOL(__kmalloc);

#ifdef CONFIG_NUMA
static void *kmalloc_large_node(size_t size, gfp_t flags, int node)
{
	struct page *page;
	void *ptr = NULL;

	flags |= __GFP_COMP | __GFP_NOTRACK;
	page = alloc_pages_node(node, flags, get_order(size));
	if (page)
		ptr = page_address(page);

	kmemleak_alloc(ptr, size, 1, flags);
	return ptr;
}

void *__kmalloc_node(size_t size, gfp_t flags, int node)
{
	struct kmem_cache *s;
	void *ret;

	if (unlikely(size > SLUB_MAX_SIZE)) {
		ret = kmalloc_large_node(size, flags, node);

		trace_kmalloc_node(_RET_IP_, ret,
				   size, PAGE_SIZE << get_order(size),
				   flags, node);

		return ret;
	}

	s = get_slab(size, flags);

	if (unlikely(ZERO_OR_NULL_PTR(s)))
		return s;

	ret = slab_alloc(s, flags, node, _RET_IP_);

	trace_kmalloc_node(_RET_IP_, ret, size, s->size, flags, node);

	return ret;
}
EXPORT_SYMBOL(__kmalloc_node);
#endif

size_t ksize(const void *object)
{
	struct page *page;

	if (unlikely(object == ZERO_SIZE_PTR))
		return 0;

	page = virt_to_head_page(object);

	if (unlikely(!PageSlab(page))) {
		WARN_ON(!PageCompound(page));
		return PAGE_SIZE << compound_order(page);
	}

	return slab_ksize(page->slab);
}
EXPORT_SYMBOL(ksize);

#ifdef CONFIG_SLUB_DEBUG
bool verify_mem_not_deleted(const void *x)
{
	struct page *page;
	void *object = (void *)x;
	unsigned long flags;
	bool rv;

	if (unlikely(ZERO_OR_NULL_PTR(x)))
		return false;

	local_irq_save(flags);

	page = virt_to_head_page(x);
	if (unlikely(!PageSlab(page))) {
		
		rv = true;
		goto out_unlock;
	}

	slab_lock(page);
	if (on_freelist(page->slab, page, object)) {
		object_err(page->slab, page, object, "Object is on free-list");
		rv = false;
	} else {
		rv = true;
	}
	slab_unlock(page);

out_unlock:
	local_irq_restore(flags);
	return rv;
}
EXPORT_SYMBOL(verify_mem_not_deleted);
#endif

void kfree(const void *x)
{
	struct page *page;
	void *object = (void *)x;

	trace_kfree(_RET_IP_, x);

	if (unlikely(ZERO_OR_NULL_PTR(x)))
		return;

	page = virt_to_head_page(x);
	if (unlikely(!page)) {
        kmemleak_free(x);
		return;
	}
	if (unlikely(!PageSlab(page))) {
		BUG_ON(!PageCompound(page));
		kmemleak_free(x);
		put_page(page);
		return;
	}
	slab_free(page->slab, page, object, _RET_IP_);
}
EXPORT_SYMBOL(kfree);

int kmem_cache_shrink(struct kmem_cache *s)
{
	int node;
	int i;
	struct kmem_cache_node *n;
	struct page *page;
	struct page *t;
	int objects = oo_objects(s->max);
	struct list_head *slabs_by_inuse =
		kmalloc(sizeof(struct list_head) * objects, GFP_KERNEL);
	unsigned long flags;

	if (!slabs_by_inuse)
		return -ENOMEM;

	flush_all(s);
	for_each_node_state(node, N_NORMAL_MEMORY) {
		n = get_node(s, node);

		if (!n->nr_partial)
			continue;

		for (i = 0; i < objects; i++)
			INIT_LIST_HEAD(slabs_by_inuse + i);

		spin_lock_irqsave(&n->list_lock, flags);

		list_for_each_entry_safe(page, t, &n->partial, lru) {
			list_move(&page->lru, slabs_by_inuse + page->inuse);
			if (!page->inuse)
				n->nr_partial--;
		}

		for (i = objects - 1; i > 0; i--)
			list_splice(slabs_by_inuse + i, n->partial.prev);

		spin_unlock_irqrestore(&n->list_lock, flags);

		
		list_for_each_entry_safe(page, t, slabs_by_inuse, lru)
			discard_slab(s, page);
	}

	kfree(slabs_by_inuse);
	return 0;
}
EXPORT_SYMBOL(kmem_cache_shrink);

#if defined(CONFIG_MEMORY_HOTPLUG)
static int slab_mem_going_offline_callback(void *arg)
{
	struct kmem_cache *s;

	down_read(&slub_lock);
	list_for_each_entry(s, &slab_caches, list)
		kmem_cache_shrink(s);
	up_read(&slub_lock);

	return 0;
}

static void slab_mem_offline_callback(void *arg)
{
	struct kmem_cache_node *n;
	struct kmem_cache *s;
	struct memory_notify *marg = arg;
	int offline_node;

	offline_node = marg->status_change_nid;

	if (offline_node < 0)
		return;

	down_read(&slub_lock);
	list_for_each_entry(s, &slab_caches, list) {
		n = get_node(s, offline_node);
		if (n) {
			BUG_ON(slabs_node(s, offline_node));

			s->node[offline_node] = NULL;
			kmem_cache_free(kmem_cache_node, n);
		}
	}
	up_read(&slub_lock);
}

static int slab_mem_going_online_callback(void *arg)
{
	struct kmem_cache_node *n;
	struct kmem_cache *s;
	struct memory_notify *marg = arg;
	int nid = marg->status_change_nid;
	int ret = 0;

	if (nid < 0)
		return 0;

	down_read(&slub_lock);
	list_for_each_entry(s, &slab_caches, list) {
		n = kmem_cache_alloc(kmem_cache_node, GFP_KERNEL);
		if (!n) {
			ret = -ENOMEM;
			goto out;
		}
		init_kmem_cache_node(n, s);
		s->node[nid] = n;
	}
out:
	up_read(&slub_lock);
	return ret;
}

static int slab_memory_callback(struct notifier_block *self,
				unsigned long action, void *arg)
{
	int ret = 0;

	switch (action) {
	case MEM_GOING_ONLINE:
		ret = slab_mem_going_online_callback(arg);
		break;
	case MEM_GOING_OFFLINE:
		ret = slab_mem_going_offline_callback(arg);
		break;
	case MEM_OFFLINE:
	case MEM_CANCEL_ONLINE:
		slab_mem_offline_callback(arg);
		break;
	case MEM_ONLINE:
	case MEM_CANCEL_OFFLINE:
		break;
	}
	if (ret)
		ret = notifier_from_errno(ret);
	else
		ret = NOTIFY_OK;
	return ret;
}

#endif 



static void __init kmem_cache_bootstrap_fixup(struct kmem_cache *s)
{
	int node;

	list_add(&s->list, &slab_caches);
	s->refcount = -1;

	for_each_node_state(node, N_NORMAL_MEMORY) {
		struct kmem_cache_node *n = get_node(s, node);
		struct page *p;

		if (n) {
			list_for_each_entry(p, &n->partial, lru)
				p->slab = s;

#ifdef CONFIG_SLUB_DEBUG
			list_for_each_entry(p, &n->full, lru)
				p->slab = s;
#endif
		}
	}
}

void __init kmem_cache_init(void)
{
	int i;
	int caches = 0;
	struct kmem_cache *temp_kmem_cache;
	int order;
	struct kmem_cache *temp_kmem_cache_node;
	unsigned long kmalloc_size;

	if (debug_guardpage_minorder())
		slub_max_order = 0;

	kmem_size = offsetof(struct kmem_cache, node) +
				nr_node_ids * sizeof(struct kmem_cache_node *);

	
	kmalloc_size = ALIGN(kmem_size, cache_line_size());
	order = get_order(2 * kmalloc_size);
	kmem_cache = (void *)__get_free_pages(GFP_NOWAIT, order);

	kmem_cache_node = (void *)kmem_cache + kmalloc_size;

	kmem_cache_open(kmem_cache_node, "kmem_cache_node",
		sizeof(struct kmem_cache_node),
		0, SLAB_HWCACHE_ALIGN | SLAB_PANIC, NULL);

	hotplug_memory_notifier(slab_memory_callback, SLAB_CALLBACK_PRI);

	
	slab_state = PARTIAL;

	temp_kmem_cache = kmem_cache;
	kmem_cache_open(kmem_cache, "kmem_cache", kmem_size,
		0, SLAB_HWCACHE_ALIGN | SLAB_PANIC, NULL);
	kmem_cache = kmem_cache_alloc(kmem_cache, GFP_NOWAIT);
	memcpy(kmem_cache, temp_kmem_cache, kmem_size);

	temp_kmem_cache_node = kmem_cache_node;

	kmem_cache_node = kmem_cache_alloc(kmem_cache, GFP_NOWAIT);
	memcpy(kmem_cache_node, temp_kmem_cache_node, kmem_size);

	kmem_cache_bootstrap_fixup(kmem_cache_node);

	caches++;
	kmem_cache_bootstrap_fixup(kmem_cache);
	caches++;
	
	free_pages((unsigned long)temp_kmem_cache, order);

	

	BUILD_BUG_ON(KMALLOC_MIN_SIZE > 256 ||
		(KMALLOC_MIN_SIZE & (KMALLOC_MIN_SIZE - 1)));

	for (i = 8; i < KMALLOC_MIN_SIZE; i += 8) {
		int elem = size_index_elem(i);
		if (elem >= ARRAY_SIZE(size_index))
			break;
		size_index[elem] = KMALLOC_SHIFT_LOW;
	}

	if (KMALLOC_MIN_SIZE == 64) {
		for (i = 64 + 8; i <= 96; i += 8)
			size_index[size_index_elem(i)] = 7;
	} else if (KMALLOC_MIN_SIZE == 128) {
		for (i = 128 + 8; i <= 192; i += 8)
			size_index[size_index_elem(i)] = 8;
	}

	
	if (KMALLOC_MIN_SIZE <= 32) {
		kmalloc_caches[1] = create_kmalloc_cache("kmalloc-96", 96, 0);
		caches++;
	}

	if (KMALLOC_MIN_SIZE <= 64) {
		kmalloc_caches[2] = create_kmalloc_cache("kmalloc-192", 192, 0);
		caches++;
	}

	for (i = KMALLOC_SHIFT_LOW; i < SLUB_PAGE_SHIFT; i++) {
		kmalloc_caches[i] = create_kmalloc_cache("kmalloc", 1 << i, 0);
		caches++;
	}

	slab_state = UP;

	
	if (KMALLOC_MIN_SIZE <= 32) {
		kmalloc_caches[1]->name = kstrdup(kmalloc_caches[1]->name, GFP_NOWAIT);
		BUG_ON(!kmalloc_caches[1]->name);
	}

	if (KMALLOC_MIN_SIZE <= 64) {
		kmalloc_caches[2]->name = kstrdup(kmalloc_caches[2]->name, GFP_NOWAIT);
		BUG_ON(!kmalloc_caches[2]->name);
	}

	for (i = KMALLOC_SHIFT_LOW; i < SLUB_PAGE_SHIFT; i++) {
		char *s = kasprintf(GFP_NOWAIT, "kmalloc-%d", 1 << i);

		BUG_ON(!s);
		kmalloc_caches[i]->name = s;
	}

#ifdef CONFIG_SMP
	register_cpu_notifier(&slab_notifier);
#endif

#ifdef CONFIG_ZONE_DMA
	for (i = 0; i < SLUB_PAGE_SHIFT; i++) {
		struct kmem_cache *s = kmalloc_caches[i];

		if (s && s->size) {
			char *name = kasprintf(GFP_NOWAIT,
				 "dma-kmalloc-%d", s->objsize);

			BUG_ON(!name);
			kmalloc_dma_caches[i] = create_kmalloc_cache(name,
				s->objsize, SLAB_CACHE_DMA);
		}
	}
#endif
	printk(KERN_INFO
		"SLUB: Genslabs=%d, HWalign=%d, Order=%d-%d, MinObjects=%d,"
		" CPUs=%d, Nodes=%d\n",
		caches, cache_line_size(),
		slub_min_order, slub_max_order, slub_min_objects,
		nr_cpu_ids, nr_node_ids);
}

void __init kmem_cache_init_late(void)
{
}

static int slab_unmergeable(struct kmem_cache *s)
{
	if (slub_nomerge || (s->flags & SLUB_NEVER_MERGE))
		return 1;

	if (s->ctor)
		return 1;

	if (s->refcount < 0)
		return 1;

	return 0;
}

static struct kmem_cache *find_mergeable(size_t size,
		size_t align, unsigned long flags, const char *name,
		void (*ctor)(void *))
{
	struct kmem_cache *s;

	if (slub_nomerge || (flags & SLUB_NEVER_MERGE))
		return NULL;

	if (ctor)
		return NULL;

	size = ALIGN(size, sizeof(void *));
	align = calculate_alignment(flags, align, size);
	size = ALIGN(size, align);
	flags = kmem_cache_flags(size, flags, name, NULL);

	list_for_each_entry(s, &slab_caches, list) {
		if (slab_unmergeable(s))
			continue;

		if (size > s->size)
			continue;

		if ((flags & SLUB_MERGE_SAME) != (s->flags & SLUB_MERGE_SAME))
				continue;
		if ((s->size & ~(align - 1)) != s->size)
			continue;

		if (s->size - size >= sizeof(void *))
			continue;

		return s;
	}
	return NULL;
}

struct kmem_cache *kmem_cache_create(const char *name, size_t size,
		size_t align, unsigned long flags, void (*ctor)(void *))
{
	struct kmem_cache *s;
	char *n;

	if (WARN_ON(!name))
		return NULL;

	down_write(&slub_lock);
	s = find_mergeable(size, align, flags, name, ctor);
	if (s) {
		s->refcount++;
		s->objsize = max(s->objsize, (int)size);
		s->inuse = max_t(int, s->inuse, ALIGN(size, sizeof(void *)));

		if (sysfs_slab_alias(s, name)) {
			s->refcount--;
			goto err;
		}
		up_write(&slub_lock);
		return s;
	}

	n = kstrdup(name, GFP_KERNEL);
	if (!n)
		goto err;

	s = kmalloc(kmem_size, GFP_KERNEL);
	if (s) {
		if (kmem_cache_open(s, n,
				size, align, flags, ctor)) {
			list_add(&s->list, &slab_caches);
			up_write(&slub_lock);
			if (sysfs_slab_add(s)) {
				down_write(&slub_lock);
				list_del(&s->list);
				kfree(n);
				kfree(s);
				goto err;
			}
			return s;
		}
		kfree(n);
		kfree(s);
	}
err:
	up_write(&slub_lock);

	if (flags & SLAB_PANIC)
		panic("Cannot create slabcache %s\n", name);
	else
		s = NULL;
	return s;
}
EXPORT_SYMBOL(kmem_cache_create);

#ifdef CONFIG_SMP
static int __cpuinit slab_cpuup_callback(struct notifier_block *nfb,
		unsigned long action, void *hcpu)
{
	long cpu = (long)hcpu;
	struct kmem_cache *s;
	unsigned long flags;

	switch (action) {
	case CPU_UP_CANCELED:
	case CPU_UP_CANCELED_FROZEN:
	case CPU_DEAD:
	case CPU_DEAD_FROZEN:
		down_read(&slub_lock);
		list_for_each_entry(s, &slab_caches, list) {
			local_irq_save(flags);
			__flush_cpu_slab(s, cpu);
			local_irq_restore(flags);
		}
		up_read(&slub_lock);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block __cpuinitdata slab_notifier = {
	.notifier_call = slab_cpuup_callback
};

#endif

void *__kmalloc_track_caller(size_t size, gfp_t gfpflags, unsigned long caller)
{
	struct kmem_cache *s;
	void *ret;

	if (unlikely(size > SLUB_MAX_SIZE))
		return kmalloc_large(size, gfpflags);

	s = get_slab(size, gfpflags);

	if (unlikely(ZERO_OR_NULL_PTR(s)))
		return s;

	ret = slab_alloc(s, gfpflags, NUMA_NO_NODE, caller);

	
	trace_kmalloc(caller, ret, size, s->size, gfpflags);

	return ret;
}

#ifdef CONFIG_NUMA
void *__kmalloc_node_track_caller(size_t size, gfp_t gfpflags,
					int node, unsigned long caller)
{
	struct kmem_cache *s;
	void *ret;

	if (unlikely(size > SLUB_MAX_SIZE)) {
		ret = kmalloc_large_node(size, gfpflags, node);

		trace_kmalloc_node(caller, ret,
				   size, PAGE_SIZE << get_order(size),
				   gfpflags, node);

		return ret;
	}

	s = get_slab(size, gfpflags);

	if (unlikely(ZERO_OR_NULL_PTR(s)))
		return s;

	ret = slab_alloc(s, gfpflags, node, caller);

	
	trace_kmalloc_node(caller, ret, size, s->size, gfpflags, node);

	return ret;
}
#endif

#ifdef CONFIG_SYSFS
static int count_inuse(struct page *page)
{
	return page->inuse;
}

static int count_total(struct page *page)
{
	return page->objects;
}
#endif

#ifdef CONFIG_SLUB_DEBUG
static int validate_slab(struct kmem_cache *s, struct page *page,
						unsigned long *map)
{
	void *p;
	void *addr = page_address(page);

	if (!check_slab(s, page) ||
			!on_freelist(s, page, NULL))
		return 0;

	
	bitmap_zero(map, page->objects);

	get_map(s, page, map);
	for_each_object(p, s, addr, page->objects) {
		if (test_bit(slab_index(p, s, addr), map))
			if (!check_object(s, page, p, SLUB_RED_INACTIVE))
				return 0;
	}

	for_each_object(p, s, addr, page->objects)
		if (!test_bit(slab_index(p, s, addr), map))
			if (!check_object(s, page, p, SLUB_RED_ACTIVE))
				return 0;
	return 1;
}

static void validate_slab_slab(struct kmem_cache *s, struct page *page,
						unsigned long *map)
{
	slab_lock(page);
	validate_slab(s, page, map);
	slab_unlock(page);
}

static int validate_slab_node(struct kmem_cache *s,
		struct kmem_cache_node *n, unsigned long *map)
{
	unsigned long count = 0;
	struct page *page;
	unsigned long flags;

	spin_lock_irqsave(&n->list_lock, flags);

	list_for_each_entry(page, &n->partial, lru) {
		validate_slab_slab(s, page, map);
		count++;
	}
	if (count != n->nr_partial)
		printk(KERN_ERR "SLUB %s: %ld partial slabs counted but "
			"counter=%ld\n", s->name, count, n->nr_partial);

	if (!(s->flags & SLAB_STORE_USER))
		goto out;

	list_for_each_entry(page, &n->full, lru) {
		validate_slab_slab(s, page, map);
		count++;
	}
	if (count != atomic_long_read(&n->nr_slabs))
		printk(KERN_ERR "SLUB: %s %ld slabs counted but "
			"counter=%ld\n", s->name, count,
			atomic_long_read(&n->nr_slabs));

out:
	spin_unlock_irqrestore(&n->list_lock, flags);
	return count;
}

static long validate_slab_cache(struct kmem_cache *s)
{
	int node;
	unsigned long count = 0;
	unsigned long *map = kmalloc(BITS_TO_LONGS(oo_objects(s->max)) *
				sizeof(unsigned long), GFP_KERNEL);

	if (!map)
		return -ENOMEM;

	flush_all(s);
	for_each_node_state(node, N_NORMAL_MEMORY) {
		struct kmem_cache_node *n = get_node(s, node);

		count += validate_slab_node(s, n, map);
	}
	kfree(map);
	return count;
}

#ifndef CONFIG_SLUB_LIGHT_WEIGHT_DEBUG_ON
struct location {
	unsigned long count;
	unsigned long addr;
	long long sum_time;
	long min_time;
	long max_time;
	long min_pid;
	long max_pid;
	DECLARE_BITMAP(cpus, NR_CPUS);
	nodemask_t nodes;
};

struct loc_track {
	unsigned long max;
	unsigned long count;
	struct location *loc;
};

static void free_loc_track(struct loc_track *t)
{
	if (t->max)
		free_pages((unsigned long)t->loc,
			get_order(sizeof(struct location) * t->max));
}

static int alloc_loc_track(struct loc_track *t, unsigned long max, gfp_t flags)
{
	struct location *l;
	int order;

	order = get_order(sizeof(struct location) * max);

	l = (void *)__get_free_pages(flags, order);
	if (!l)
		return 0;

	if (t->count) {
		memcpy(l, t->loc, sizeof(struct location) * t->count);
		free_loc_track(t);
	}
	t->max = max;
	t->loc = l;
	return 1;
}

static int add_location(struct loc_track *t, struct kmem_cache *s,
				const struct track *track)
{
	long start, end, pos;
	struct location *l;
	unsigned long caddr;
	unsigned long age = jiffies - track->when;

	start = -1;
	end = t->count;

	for ( ; ; ) {
		pos = start + (end - start + 1) / 2;

		if (pos == end)
			break;

		caddr = t->loc[pos].addr;
		if (track->addr == caddr) {

			l = &t->loc[pos];
			l->count++;
			if (track->when) {
				l->sum_time += age;
				if (age < l->min_time)
					l->min_time = age;
				if (age > l->max_time)
					l->max_time = age;

				if (track->pid < l->min_pid)
					l->min_pid = track->pid;
				if (track->pid > l->max_pid)
					l->max_pid = track->pid;

				cpumask_set_cpu(track->cpu,
						to_cpumask(l->cpus));
			}
			node_set(page_to_nid(virt_to_page(track)), l->nodes);
			return 1;
		}

		if (track->addr < caddr)
			end = pos;
		else
			start = pos;
	}

	if (t->count >= t->max && !alloc_loc_track(t, 2 * t->max, GFP_ATOMIC))
		return 0;

	l = t->loc + pos;
	if (pos < t->count)
		memmove(l + 1, l,
			(t->count - pos) * sizeof(struct location));
	t->count++;
	l->count = 1;
	l->addr = track->addr;
	l->sum_time = age;
	l->min_time = age;
	l->max_time = age;
	l->min_pid = track->pid;
	l->max_pid = track->pid;
	cpumask_clear(to_cpumask(l->cpus));
	cpumask_set_cpu(track->cpu, to_cpumask(l->cpus));
	nodes_clear(l->nodes);
	node_set(page_to_nid(virt_to_page(track)), l->nodes);
	return 1;
}

static void process_slab(struct loc_track *t, struct kmem_cache *s,
		struct page *page, enum track_item alloc,
		unsigned long *map)
{
	void *addr = page_address(page);
	void *p;

	bitmap_zero(map, page->objects);
	get_map(s, page, map);

	for_each_object(p, s, addr, page->objects)
		if (!test_bit(slab_index(p, s, addr), map))
			add_location(t, s, get_track(s, p, alloc));
}

static int list_locations(struct kmem_cache *s, char *buf,
					enum track_item alloc)
{
	int len = 0;
	unsigned long i;
	struct loc_track t = { 0, 0, NULL };
	int node;
	unsigned long *map = kmalloc(BITS_TO_LONGS(oo_objects(s->max)) *
				     sizeof(unsigned long), GFP_KERNEL);

	if (!map || !alloc_loc_track(&t, PAGE_SIZE / sizeof(struct location),
				     GFP_TEMPORARY)) {
		kfree(map);
		return sprintf(buf, "Out of memory\n");
	}
	
	flush_all(s);

	for_each_node_state(node, N_NORMAL_MEMORY) {
		struct kmem_cache_node *n = get_node(s, node);
		unsigned long flags;
		struct page *page;

		if (!atomic_long_read(&n->nr_slabs))
			continue;

		spin_lock_irqsave(&n->list_lock, flags);
		list_for_each_entry(page, &n->partial, lru)
			process_slab(&t, s, page, alloc, map);
		list_for_each_entry(page, &n->full, lru)
			process_slab(&t, s, page, alloc, map);
		spin_unlock_irqrestore(&n->list_lock, flags);
	}

	for (i = 0; i < t.count; i++) {
		struct location *l = &t.loc[i];

		if (len > PAGE_SIZE - KSYM_SYMBOL_LEN - 100)
			break;
		len += sprintf(buf + len, "%7ld ", l->count);

		if (l->addr)
			len += sprintf(buf + len, "%pS", (void *)l->addr);
		else
			len += sprintf(buf + len, "<not-available>");

		if (l->sum_time != l->min_time) {
			len += sprintf(buf + len, " age=%ld/%ld/%ld",
				l->min_time,
				(long)div_u64(l->sum_time, l->count),
				l->max_time);
		} else
			len += sprintf(buf + len, " age=%ld",
				l->min_time);

		if (l->min_pid != l->max_pid)
			len += sprintf(buf + len, " pid=%ld-%ld",
				l->min_pid, l->max_pid);
		else
			len += sprintf(buf + len, " pid=%ld",
				l->min_pid);

		if (num_online_cpus() > 1 &&
				!cpumask_empty(to_cpumask(l->cpus)) &&
				len < PAGE_SIZE - 60) {
			len += sprintf(buf + len, " cpus=");
			len += cpulist_scnprintf(buf + len, PAGE_SIZE - len - 50,
						 to_cpumask(l->cpus));
		}

		if (nr_online_nodes > 1 && !nodes_empty(l->nodes) &&
				len < PAGE_SIZE - 60) {
			len += sprintf(buf + len, " nodes=");
			len += nodelist_scnprintf(buf + len, PAGE_SIZE - len - 50,
					l->nodes);
		}

		len += sprintf(buf + len, "\n");
	}

	free_loc_track(&t);
	kfree(map);
	if (!t.count)
		len += sprintf(buf, "No data\n");
	return len;
}
#endif
#endif

#ifdef SLUB_RESILIENCY_TEST
static void resiliency_test(void)
{
	u8 *p;

	BUILD_BUG_ON(KMALLOC_MIN_SIZE > 16 || SLUB_PAGE_SHIFT < 10);

	printk(KERN_ERR "SLUB resiliency testing\n");
	printk(KERN_ERR "-----------------------\n");
	printk(KERN_ERR "A. Corruption after allocation\n");

	p = kzalloc(16, GFP_KERNEL);
	p[16] = 0x12;
	printk(KERN_ERR "\n1. kmalloc-16: Clobber Redzone/next pointer"
			" 0x12->0x%p\n\n", p + 16);

	validate_slab_cache(kmalloc_caches[4]);

	
	p = kzalloc(32, GFP_KERNEL);
	p[32 + sizeof(void *)] = 0x34;
	printk(KERN_ERR "\n2. kmalloc-32: Clobber next pointer/next slab"
			" 0x34 -> -0x%p\n", p);
	printk(KERN_ERR
		"If allocated object is overwritten then not detectable\n\n");

	validate_slab_cache(kmalloc_caches[5]);
	p = kzalloc(64, GFP_KERNEL);
	p += 64 + (get_cycles() & 0xff) * sizeof(void *);
	*p = 0x56;
	printk(KERN_ERR "\n3. kmalloc-64: corrupting random byte 0x56->0x%p\n",
									p);
	printk(KERN_ERR
		"If allocated object is overwritten then not detectable\n\n");
	validate_slab_cache(kmalloc_caches[6]);

	printk(KERN_ERR "\nB. Corruption after free\n");
	p = kzalloc(128, GFP_KERNEL);
	kfree(p);
	*p = 0x78;
	printk(KERN_ERR "1. kmalloc-128: Clobber first word 0x78->0x%p\n\n", p);
	validate_slab_cache(kmalloc_caches[7]);

	p = kzalloc(256, GFP_KERNEL);
	kfree(p);
	p[50] = 0x9a;
	printk(KERN_ERR "\n2. kmalloc-256: Clobber 50th byte 0x9a->0x%p\n\n",
			p);
	validate_slab_cache(kmalloc_caches[8]);

	p = kzalloc(512, GFP_KERNEL);
	kfree(p);
	p[512] = 0xab;
	printk(KERN_ERR "\n3. kmalloc-512: Clobber redzone 0xab->0x%p\n\n", p);
	validate_slab_cache(kmalloc_caches[9]);
}
#else
#ifdef CONFIG_SYSFS
static void resiliency_test(void) {};
#endif
#endif

#ifdef CONFIG_SYSFS
enum slab_stat_type {
	SL_ALL,			
	SL_PARTIAL,		
	SL_CPU,			
	SL_OBJECTS,		
	SL_TOTAL		
};

#define SO_ALL		(1 << SL_ALL)
#define SO_PARTIAL	(1 << SL_PARTIAL)
#define SO_CPU		(1 << SL_CPU)
#define SO_OBJECTS	(1 << SL_OBJECTS)
#define SO_TOTAL	(1 << SL_TOTAL)

static ssize_t show_slab_objects(struct kmem_cache *s,
			    char *buf, unsigned long flags)
{
	unsigned long total = 0;
	int node;
	int x;
	unsigned long *nodes;
	unsigned long *per_cpu;

	nodes = kzalloc(2 * sizeof(unsigned long) * nr_node_ids, GFP_KERNEL);
	if (!nodes)
		return -ENOMEM;
	per_cpu = nodes + nr_node_ids;

	if (flags & SO_CPU) {
		int cpu;

		for_each_possible_cpu(cpu) {
			struct kmem_cache_cpu *c = per_cpu_ptr(s->cpu_slab, cpu);
			int node = ACCESS_ONCE(c->node);
			struct page *page;

			if (node < 0)
				continue;
			page = ACCESS_ONCE(c->page);
			if (page) {
				if (flags & SO_TOTAL)
					x = page->objects;
				else if (flags & SO_OBJECTS)
					x = page->inuse;
				else
					x = 1;

				total += x;
				nodes[node] += x;
			}
			page = c->partial;

			if (page) {
				x = page->pobjects;
				total += x;
				nodes[node] += x;
			}
			per_cpu[node]++;
		}
	}

	lock_memory_hotplug();
#ifdef CONFIG_SLUB_DEBUG
	if (flags & SO_ALL) {
		for_each_node_state(node, N_NORMAL_MEMORY) {
			struct kmem_cache_node *n = get_node(s, node);

		if (flags & SO_TOTAL)
			x = atomic_long_read(&n->total_objects);
		else if (flags & SO_OBJECTS)
			x = atomic_long_read(&n->total_objects) -
				count_partial(n, count_free);

			else
				x = atomic_long_read(&n->nr_slabs);
			total += x;
			nodes[node] += x;
		}

	} else
#endif
	if (flags & SO_PARTIAL) {
		for_each_node_state(node, N_NORMAL_MEMORY) {
			struct kmem_cache_node *n = get_node(s, node);

			if (flags & SO_TOTAL)
				x = count_partial(n, count_total);
			else if (flags & SO_OBJECTS)
				x = count_partial(n, count_inuse);
			else
				x = n->nr_partial;
			total += x;
			nodes[node] += x;
		}
	}
	x = sprintf(buf, "%lu", total);
#ifdef CONFIG_NUMA
	for_each_node_state(node, N_NORMAL_MEMORY)
		if (nodes[node])
			x += sprintf(buf + x, " N%d=%lu",
					node, nodes[node]);
#endif
	unlock_memory_hotplug();
	kfree(nodes);
	return x + sprintf(buf + x, "\n");
}

#ifdef CONFIG_SLUB_DEBUG
static int any_slab_objects(struct kmem_cache *s)
{
	int node;

	for_each_online_node(node) {
		struct kmem_cache_node *n = get_node(s, node);

		if (!n)
			continue;

		if (atomic_long_read(&n->total_objects))
			return 1;
	}
	return 0;
}
#endif

#define to_slab_attr(n) container_of(n, struct slab_attribute, attr)
#define to_slab(n) container_of(n, struct kmem_cache, kobj)

struct slab_attribute {
	struct attribute attr;
	ssize_t (*show)(struct kmem_cache *s, char *buf);
	ssize_t (*store)(struct kmem_cache *s, const char *x, size_t count);
};

#define SLAB_ATTR_RO(_name) \
	static struct slab_attribute _name##_attr = \
	__ATTR(_name, 0400, _name##_show, NULL)

#define SLAB_ATTR(_name) \
	static struct slab_attribute _name##_attr =  \
	__ATTR(_name, 0600, _name##_show, _name##_store)

static ssize_t slab_size_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%d\n", s->size);
}
SLAB_ATTR_RO(slab_size);

static ssize_t align_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%d\n", s->align);
}
SLAB_ATTR_RO(align);

static ssize_t object_size_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%d\n", s->objsize);
}
SLAB_ATTR_RO(object_size);

static ssize_t objs_per_slab_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%d\n", oo_objects(s->oo));
}
SLAB_ATTR_RO(objs_per_slab);

static ssize_t order_store(struct kmem_cache *s,
				const char *buf, size_t length)
{
	unsigned long order;
	int err;

	err = strict_strtoul(buf, 10, &order);
	if (err)
		return err;

	if (order > slub_max_order || order < slub_min_order)
		return -EINVAL;

	calculate_sizes(s, order);
	return length;
}

static ssize_t order_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%d\n", oo_order(s->oo));
}
SLAB_ATTR(order);

static ssize_t min_partial_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%lu\n", s->min_partial);
}

static ssize_t min_partial_store(struct kmem_cache *s, const char *buf,
				 size_t length)
{
	unsigned long min;
	int err;

	err = strict_strtoul(buf, 10, &min);
	if (err)
		return err;

	set_min_partial(s, min);
	return length;
}
SLAB_ATTR(min_partial);

static ssize_t cpu_partial_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%u\n", s->cpu_partial);
}

static ssize_t cpu_partial_store(struct kmem_cache *s, const char *buf,
				 size_t length)
{
	unsigned long objects;
	int err;

	err = strict_strtoul(buf, 10, &objects);
	if (err)
		return err;
	if (objects && kmem_cache_debug(s))
		return -EINVAL;

	s->cpu_partial = objects;
	flush_all(s);
	return length;
}
SLAB_ATTR(cpu_partial);

static ssize_t ctor_show(struct kmem_cache *s, char *buf)
{
	if (!s->ctor)
		return 0;
	return sprintf(buf, "%pS\n", s->ctor);
}
SLAB_ATTR_RO(ctor);

static ssize_t aliases_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%d\n", s->refcount - 1);
}
SLAB_ATTR_RO(aliases);

static ssize_t partial_show(struct kmem_cache *s, char *buf)
{
	return show_slab_objects(s, buf, SO_PARTIAL);
}
SLAB_ATTR_RO(partial);

static ssize_t cpu_slabs_show(struct kmem_cache *s, char *buf)
{
	return show_slab_objects(s, buf, SO_CPU);
}
SLAB_ATTR_RO(cpu_slabs);

static ssize_t objects_show(struct kmem_cache *s, char *buf)
{
	return show_slab_objects(s, buf, SO_ALL|SO_OBJECTS);
}
SLAB_ATTR_RO(objects);

static ssize_t objects_partial_show(struct kmem_cache *s, char *buf)
{
	return show_slab_objects(s, buf, SO_PARTIAL|SO_OBJECTS);
}
SLAB_ATTR_RO(objects_partial);

static ssize_t slabs_cpu_partial_show(struct kmem_cache *s, char *buf)
{
	int objects = 0;
	int pages = 0;
	int cpu;
	int len;

	for_each_online_cpu(cpu) {
		struct page *page = per_cpu_ptr(s->cpu_slab, cpu)->partial;

		if (page) {
			pages += page->pages;
			objects += page->pobjects;
		}
	}

	len = sprintf(buf, "%d(%d)", objects, pages);

#ifdef CONFIG_SMP
	for_each_online_cpu(cpu) {
		struct page *page = per_cpu_ptr(s->cpu_slab, cpu) ->partial;

		if (page && len < PAGE_SIZE - 20)
			len += sprintf(buf + len, " C%d=%d(%d)", cpu,
				page->pobjects, page->pages);
	}
#endif
	return len + sprintf(buf + len, "\n");
}
SLAB_ATTR_RO(slabs_cpu_partial);

static ssize_t reclaim_account_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%d\n", !!(s->flags & SLAB_RECLAIM_ACCOUNT));
}

static ssize_t reclaim_account_store(struct kmem_cache *s,
				const char *buf, size_t length)
{
	s->flags &= ~SLAB_RECLAIM_ACCOUNT;
	if (buf[0] == '1')
		s->flags |= SLAB_RECLAIM_ACCOUNT;
	return length;
}
SLAB_ATTR(reclaim_account);

static ssize_t hwcache_align_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%d\n", !!(s->flags & SLAB_HWCACHE_ALIGN));
}
SLAB_ATTR_RO(hwcache_align);

#ifdef CONFIG_ZONE_DMA
static ssize_t cache_dma_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%d\n", !!(s->flags & SLAB_CACHE_DMA));
}
SLAB_ATTR_RO(cache_dma);
#endif

static ssize_t destroy_by_rcu_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%d\n", !!(s->flags & SLAB_DESTROY_BY_RCU));
}
SLAB_ATTR_RO(destroy_by_rcu);

static ssize_t reserved_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%d\n", s->reserved);
}
SLAB_ATTR_RO(reserved);

#ifdef CONFIG_SLUB_DEBUG
static ssize_t slabs_show(struct kmem_cache *s, char *buf)
{
	return show_slab_objects(s, buf, SO_ALL);
}
SLAB_ATTR_RO(slabs);

static ssize_t total_objects_show(struct kmem_cache *s, char *buf)
{
	return show_slab_objects(s, buf, SO_ALL|SO_TOTAL);
}
SLAB_ATTR_RO(total_objects);

static ssize_t sanity_checks_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%d\n", !!(s->flags & SLAB_DEBUG_FREE));
}

static ssize_t sanity_checks_store(struct kmem_cache *s,
				const char *buf, size_t length)
{
	s->flags &= ~SLAB_DEBUG_FREE;
	if (buf[0] == '1') {
		s->flags &= ~__CMPXCHG_DOUBLE;
		s->flags |= SLAB_DEBUG_FREE;
	}
	return length;
}
SLAB_ATTR(sanity_checks);

static ssize_t trace_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%d\n", !!(s->flags & SLAB_TRACE));
}

static ssize_t trace_store(struct kmem_cache *s, const char *buf,
							size_t length)
{
	s->flags &= ~SLAB_TRACE;
	if (buf[0] == '1') {
		s->flags &= ~__CMPXCHG_DOUBLE;
		s->flags |= SLAB_TRACE;
	}
	return length;
}
SLAB_ATTR(trace);

static ssize_t red_zone_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%d\n", !!(s->flags & SLAB_RED_ZONE));
}

static ssize_t red_zone_store(struct kmem_cache *s,
				const char *buf, size_t length)
{
	if (any_slab_objects(s))
		return -EBUSY;

	s->flags &= ~SLAB_RED_ZONE;
	if (buf[0] == '1') {
		s->flags &= ~__CMPXCHG_DOUBLE;
		s->flags |= SLAB_RED_ZONE;
	}
	calculate_sizes(s, -1);
	return length;
}
SLAB_ATTR(red_zone);

static ssize_t poison_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%d\n", !!(s->flags & SLAB_POISON));
}

static ssize_t poison_store(struct kmem_cache *s,
				const char *buf, size_t length)
{
	if (any_slab_objects(s))
		return -EBUSY;

	s->flags &= ~SLAB_POISON;
	if (buf[0] == '1') {
		s->flags &= ~__CMPXCHG_DOUBLE;
		s->flags |= SLAB_POISON;
	}
	calculate_sizes(s, -1);
	return length;
}
SLAB_ATTR(poison);

static ssize_t store_user_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%d\n", !!(s->flags & SLAB_STORE_USER));
}

static ssize_t store_user_store(struct kmem_cache *s,
				const char *buf, size_t length)
{
	if (any_slab_objects(s))
		return -EBUSY;

	s->flags &= ~SLAB_STORE_USER;
	if (buf[0] == '1') {
		s->flags &= ~__CMPXCHG_DOUBLE;
		s->flags |= SLAB_STORE_USER;
	}
	calculate_sizes(s, -1);
	return length;
}
SLAB_ATTR(store_user);

static ssize_t validate_show(struct kmem_cache *s, char *buf)
{
	return 0;
}

static ssize_t validate_store(struct kmem_cache *s,
			const char *buf, size_t length)
{
	int ret = -EINVAL;

	if (buf[0] == '1') {
		ret = validate_slab_cache(s);
		if (ret >= 0)
			ret = length;
	}
	return ret;
}
SLAB_ATTR(validate);

#ifndef CONFIG_SLUB_LIGHT_WEIGHT_DEBUG_ON
static ssize_t alloc_calls_show(struct kmem_cache *s, char *buf)
{
	if (!(s->flags & SLAB_STORE_USER))
		return -ENOSYS;
	return list_locations(s, buf, TRACK_ALLOC);
}
SLAB_ATTR_RO(alloc_calls);

static ssize_t free_calls_show(struct kmem_cache *s, char *buf)
{
	if (!(s->flags & SLAB_STORE_USER))
		return -ENOSYS;
	return list_locations(s, buf, TRACK_FREE);
}
SLAB_ATTR_RO(free_calls);
#endif
#endif 

#ifdef CONFIG_FAILSLAB
static ssize_t failslab_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%d\n", !!(s->flags & SLAB_FAILSLAB));
}

static ssize_t failslab_store(struct kmem_cache *s, const char *buf,
							size_t length)
{
	s->flags &= ~SLAB_FAILSLAB;
	if (buf[0] == '1')
		s->flags |= SLAB_FAILSLAB;
	return length;
}
SLAB_ATTR(failslab);
#endif

static ssize_t shrink_show(struct kmem_cache *s, char *buf)
{
	return 0;
}

static ssize_t shrink_store(struct kmem_cache *s,
			const char *buf, size_t length)
{
	if (buf[0] == '1') {
		int rc = kmem_cache_shrink(s);

		if (rc)
			return rc;
	} else
		return -EINVAL;
	return length;
}
SLAB_ATTR(shrink);

#ifdef CONFIG_NUMA
static ssize_t remote_node_defrag_ratio_show(struct kmem_cache *s, char *buf)
{
	return sprintf(buf, "%d\n", s->remote_node_defrag_ratio / 10);
}

static ssize_t remote_node_defrag_ratio_store(struct kmem_cache *s,
				const char *buf, size_t length)
{
	unsigned long ratio;
	int err;

	err = strict_strtoul(buf, 10, &ratio);
	if (err)
		return err;

	if (ratio <= 100)
		s->remote_node_defrag_ratio = ratio * 10;

	return length;
}
SLAB_ATTR(remote_node_defrag_ratio);
#endif

#ifdef CONFIG_SLUB_STATS
static int show_stat(struct kmem_cache *s, char *buf, enum stat_item si)
{
	unsigned long sum  = 0;
	int cpu;
	int len;
	int *data = kmalloc(nr_cpu_ids * sizeof(int), GFP_KERNEL);

	if (!data)
		return -ENOMEM;

	for_each_online_cpu(cpu) {
		unsigned x = per_cpu_ptr(s->cpu_slab, cpu)->stat[si];

		data[cpu] = x;
		sum += x;
	}

	len = sprintf(buf, "%lu", sum);

#ifdef CONFIG_SMP
	for_each_online_cpu(cpu) {
		if (data[cpu] && len < PAGE_SIZE - 20)
			len += sprintf(buf + len, " C%d=%u", cpu, data[cpu]);
	}
#endif
	kfree(data);
	return len + sprintf(buf + len, "\n");
}

static void clear_stat(struct kmem_cache *s, enum stat_item si)
{
	int cpu;

	for_each_online_cpu(cpu)
		per_cpu_ptr(s->cpu_slab, cpu)->stat[si] = 0;
}

#define STAT_ATTR(si, text) 					\
static ssize_t text##_show(struct kmem_cache *s, char *buf)	\
{								\
	return show_stat(s, buf, si);				\
}								\
static ssize_t text##_store(struct kmem_cache *s,		\
				const char *buf, size_t length)	\
{								\
	if (buf[0] != '0')					\
		return -EINVAL;					\
	clear_stat(s, si);					\
	return length;						\
}								\
SLAB_ATTR(text);						\

STAT_ATTR(ALLOC_FASTPATH, alloc_fastpath);
STAT_ATTR(ALLOC_SLOWPATH, alloc_slowpath);
STAT_ATTR(FREE_FASTPATH, free_fastpath);
STAT_ATTR(FREE_SLOWPATH, free_slowpath);
STAT_ATTR(FREE_FROZEN, free_frozen);
STAT_ATTR(FREE_ADD_PARTIAL, free_add_partial);
STAT_ATTR(FREE_REMOVE_PARTIAL, free_remove_partial);
STAT_ATTR(ALLOC_FROM_PARTIAL, alloc_from_partial);
STAT_ATTR(ALLOC_SLAB, alloc_slab);
STAT_ATTR(ALLOC_REFILL, alloc_refill);
STAT_ATTR(ALLOC_NODE_MISMATCH, alloc_node_mismatch);
STAT_ATTR(FREE_SLAB, free_slab);
STAT_ATTR(CPUSLAB_FLUSH, cpuslab_flush);
STAT_ATTR(DEACTIVATE_FULL, deactivate_full);
STAT_ATTR(DEACTIVATE_EMPTY, deactivate_empty);
STAT_ATTR(DEACTIVATE_TO_HEAD, deactivate_to_head);
STAT_ATTR(DEACTIVATE_TO_TAIL, deactivate_to_tail);
STAT_ATTR(DEACTIVATE_REMOTE_FREES, deactivate_remote_frees);
STAT_ATTR(DEACTIVATE_BYPASS, deactivate_bypass);
STAT_ATTR(ORDER_FALLBACK, order_fallback);
STAT_ATTR(CMPXCHG_DOUBLE_CPU_FAIL, cmpxchg_double_cpu_fail);
STAT_ATTR(CMPXCHG_DOUBLE_FAIL, cmpxchg_double_fail);
STAT_ATTR(CPU_PARTIAL_ALLOC, cpu_partial_alloc);
STAT_ATTR(CPU_PARTIAL_FREE, cpu_partial_free);
STAT_ATTR(CPU_PARTIAL_NODE, cpu_partial_node);
STAT_ATTR(CPU_PARTIAL_DRAIN, cpu_partial_drain);
#endif

static struct attribute *slab_attrs[] = {
	&slab_size_attr.attr,
	&object_size_attr.attr,
	&objs_per_slab_attr.attr,
	&order_attr.attr,
	&min_partial_attr.attr,
	&cpu_partial_attr.attr,
	&objects_attr.attr,
	&objects_partial_attr.attr,
	&partial_attr.attr,
	&cpu_slabs_attr.attr,
	&ctor_attr.attr,
	&aliases_attr.attr,
	&align_attr.attr,
	&hwcache_align_attr.attr,
	&reclaim_account_attr.attr,
	&destroy_by_rcu_attr.attr,
	&shrink_attr.attr,
	&reserved_attr.attr,
	&slabs_cpu_partial_attr.attr,
#ifdef CONFIG_SLUB_DEBUG
	&total_objects_attr.attr,
	&slabs_attr.attr,
	&sanity_checks_attr.attr,
	&trace_attr.attr,
	&red_zone_attr.attr,
	&poison_attr.attr,
	&store_user_attr.attr,
	&validate_attr.attr,
#ifndef CONFIG_SLUB_LIGHT_WEIGHT_DEBUG_ON
	&alloc_calls_attr.attr,
	&free_calls_attr.attr,
#endif
#endif
#ifdef CONFIG_ZONE_DMA
	&cache_dma_attr.attr,
#endif
#ifdef CONFIG_NUMA
	&remote_node_defrag_ratio_attr.attr,
#endif
#ifdef CONFIG_SLUB_STATS
	&alloc_fastpath_attr.attr,
	&alloc_slowpath_attr.attr,
	&free_fastpath_attr.attr,
	&free_slowpath_attr.attr,
	&free_frozen_attr.attr,
	&free_add_partial_attr.attr,
	&free_remove_partial_attr.attr,
	&alloc_from_partial_attr.attr,
	&alloc_slab_attr.attr,
	&alloc_refill_attr.attr,
	&alloc_node_mismatch_attr.attr,
	&free_slab_attr.attr,
	&cpuslab_flush_attr.attr,
	&deactivate_full_attr.attr,
	&deactivate_empty_attr.attr,
	&deactivate_to_head_attr.attr,
	&deactivate_to_tail_attr.attr,
	&deactivate_remote_frees_attr.attr,
	&deactivate_bypass_attr.attr,
	&order_fallback_attr.attr,
	&cmpxchg_double_fail_attr.attr,
	&cmpxchg_double_cpu_fail_attr.attr,
	&cpu_partial_alloc_attr.attr,
	&cpu_partial_free_attr.attr,
	&cpu_partial_node_attr.attr,
	&cpu_partial_drain_attr.attr,
#endif
#ifdef CONFIG_FAILSLAB
	&failslab_attr.attr,
#endif

	NULL
};

static struct attribute_group slab_attr_group = {
	.attrs = slab_attrs,
};

static ssize_t slab_attr_show(struct kobject *kobj,
				struct attribute *attr,
				char *buf)
{
	struct slab_attribute *attribute;
	struct kmem_cache *s;
	int err;

	attribute = to_slab_attr(attr);
	s = to_slab(kobj);

	if (!attribute->show)
		return -EIO;

	err = attribute->show(s, buf);

	return err;
}

static ssize_t slab_attr_store(struct kobject *kobj,
				struct attribute *attr,
				const char *buf, size_t len)
{
	struct slab_attribute *attribute;
	struct kmem_cache *s;
	int err;

	attribute = to_slab_attr(attr);
	s = to_slab(kobj);

	if (!attribute->store)
		return -EIO;

	err = attribute->store(s, buf, len);

	return err;
}

static void kmem_cache_release(struct kobject *kobj)
{
	struct kmem_cache *s = to_slab(kobj);

	kfree(s->name);
	kfree(s);
}

static const struct sysfs_ops slab_sysfs_ops = {
	.show = slab_attr_show,
	.store = slab_attr_store,
};

static struct kobj_type slab_ktype = {
	.sysfs_ops = &slab_sysfs_ops,
	.release = kmem_cache_release
};

static int uevent_filter(struct kset *kset, struct kobject *kobj)
{
	struct kobj_type *ktype = get_ktype(kobj);

	if (ktype == &slab_ktype)
		return 1;
	return 0;
}

static const struct kset_uevent_ops slab_uevent_ops = {
	.filter = uevent_filter,
};

static struct kset *slab_kset;

#define ID_STR_LENGTH 64

static char *create_unique_id(struct kmem_cache *s)
{
	char *name = kmalloc(ID_STR_LENGTH, GFP_KERNEL);
	char *p = name;

	BUG_ON(!name);

	*p++ = ':';
	if (s->flags & SLAB_CACHE_DMA)
		*p++ = 'd';
	if (s->flags & SLAB_RECLAIM_ACCOUNT)
		*p++ = 'a';
	if (s->flags & SLAB_DEBUG_FREE)
		*p++ = 'F';
	if (!(s->flags & SLAB_NOTRACK))
		*p++ = 't';
	if (p != name + 1)
		*p++ = '-';
	p += sprintf(p, "%07d", s->size);
	BUG_ON(p > name + ID_STR_LENGTH - 1);
	return name;
}

static int sysfs_slab_add(struct kmem_cache *s)
{
	int err;
	const char *name;
	int unmergeable;

	if (slab_state < SYSFS)
		
		return 0;

	unmergeable = slab_unmergeable(s);
	if (unmergeable) {
		sysfs_remove_link(&slab_kset->kobj, s->name);
		name = s->name;
	} else {
		name = create_unique_id(s);
	}

	s->kobj.kset = slab_kset;
	err = kobject_init_and_add(&s->kobj, &slab_ktype, NULL, name);
	if (err) {
		kobject_put(&s->kobj);
		return err;
	}

	err = sysfs_create_group(&s->kobj, &slab_attr_group);
	if (err) {
		kobject_del(&s->kobj);
		kobject_put(&s->kobj);
		return err;
	}
	kobject_uevent(&s->kobj, KOBJ_ADD);
	if (!unmergeable) {
		
		sysfs_slab_alias(s, s->name);
		kfree(name);
	}
	return 0;
}

static void sysfs_slab_remove(struct kmem_cache *s)
{
	if (slab_state < SYSFS)
		return;

	kobject_uevent(&s->kobj, KOBJ_REMOVE);
	kobject_del(&s->kobj);
	kobject_put(&s->kobj);
}

struct saved_alias {
	struct kmem_cache *s;
	const char *name;
	struct saved_alias *next;
};

static struct saved_alias *alias_list;

static int sysfs_slab_alias(struct kmem_cache *s, const char *name)
{
	struct saved_alias *al;

	if (slab_state == SYSFS) {
		sysfs_remove_link(&slab_kset->kobj, name);
		return sysfs_create_link(&slab_kset->kobj, &s->kobj, name);
	}

	al = kmalloc(sizeof(struct saved_alias), GFP_KERNEL);
	if (!al)
		return -ENOMEM;

	al->s = s;
	al->name = name;
	al->next = alias_list;
	alias_list = al;
	return 0;
}

static int __init slab_sysfs_init(void)
{
	struct kmem_cache *s;
	int err;

	down_write(&slub_lock);

	slab_kset = kset_create_and_add("slab", &slab_uevent_ops, kernel_kobj);
	if (!slab_kset) {
		up_write(&slub_lock);
		printk(KERN_ERR "Cannot register slab subsystem.\n");
		return -ENOSYS;
	}

	slab_state = SYSFS;

	list_for_each_entry(s, &slab_caches, list) {
		err = sysfs_slab_add(s);
		if (err)
			printk(KERN_ERR "SLUB: Unable to add boot slab %s"
						" to sysfs\n", s->name);
	}

	while (alias_list) {
		struct saved_alias *al = alias_list;

		alias_list = alias_list->next;
		err = sysfs_slab_alias(al->s, al->name);
		if (err)
			printk(KERN_ERR "SLUB: Unable to add boot slab alias"
					" %s to sysfs\n", s->name);
		kfree(al);
	}

	up_write(&slub_lock);
	resiliency_test();
	return 0;
}

__initcall(slab_sysfs_init);
#endif 

#ifdef CONFIG_SLABINFO
static void print_slabinfo_header(struct seq_file *m)
{
	seq_puts(m, "slabinfo - version: 2.1\n");
	seq_puts(m, "# name            <active_objs> <num_objs> <objsize> "
		 "<objperslab> <pagesperslab>");
	seq_puts(m, " : tunables <limit> <batchcount> <sharedfactor>");
	seq_puts(m, " : slabdata <active_slabs> <num_slabs> <sharedavail>");
	seq_putc(m, '\n');
}

static void *s_start(struct seq_file *m, loff_t *pos)
{
	loff_t n = *pos;

	down_read(&slub_lock);
	if (!n)
		print_slabinfo_header(m);

	return seq_list_start(&slab_caches, *pos);
}

static void *s_next(struct seq_file *m, void *p, loff_t *pos)
{
	return seq_list_next(p, &slab_caches, pos);
}

static void s_stop(struct seq_file *m, void *p)
{
	up_read(&slub_lock);
}

static int s_show(struct seq_file *m, void *p)
{
	unsigned long nr_partials = 0;
	unsigned long nr_slabs = 0;
	unsigned long nr_inuse = 0;
	unsigned long nr_objs = 0;
	unsigned long nr_free = 0;
	struct kmem_cache *s;
	int node;

	s = list_entry(p, struct kmem_cache, list);

	for_each_online_node(node) {
		struct kmem_cache_node *n = get_node(s, node);

		if (!n)
			continue;

		nr_partials += n->nr_partial;
		nr_slabs += atomic_long_read(&n->nr_slabs);
		nr_objs += atomic_long_read(&n->total_objects);
		nr_free += count_partial(n, count_free);
	}

	nr_inuse = nr_objs - nr_free;

	seq_printf(m, "%-17s %6lu %6lu %6u %4u %4d", s->name, nr_inuse,
		   nr_objs, s->size, oo_objects(s->oo),
		   (1 << oo_order(s->oo)));
	seq_printf(m, " : tunables %4u %4u %4u", 0, 0, 0);
	seq_printf(m, " : slabdata %6lu %6lu %6lu", nr_slabs, nr_slabs,
		   0UL);
	seq_putc(m, '\n');
	return 0;
}

static const struct seq_operations slabinfo_op = {
	.start = s_start,
	.next = s_next,
	.stop = s_stop,
	.show = s_show,
};

static int slabinfo_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &slabinfo_op);
}

static const struct file_operations proc_slabinfo_operations = {
	.open		= slabinfo_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int __init slab_proc_init(void)
{
	proc_create("slabinfo", S_IRUSR, NULL, &proc_slabinfo_operations);
	return 0;
}
module_init(slab_proc_init);
#endif 
