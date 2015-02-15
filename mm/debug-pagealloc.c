#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/page-debug-flags.h>
#include <linux/poison.h>
#include <linux/ratelimit.h>
#include <linux/stacktrace.h>
#include <linux/debugfs.h>
#include <linux/sched.h>

#ifndef mark_addr_rdonly
#define mark_addr_rdonly(a)
#endif

#ifndef mark_addr_rdwrite
#define mark_addr_rdwrite(a)
#endif

#ifndef mark_addr_rdonly
#define mark_addr_rdonly(a)
#endif

#ifndef mark_addr_rdwrite
#define mark_addr_rdwrite(a)
#endif

static inline void set_page_poison(struct page *page)
{
	__set_bit(PAGE_DEBUG_FLAG_POISON, &page->debug_flags);
}

static inline void clear_page_poison(struct page *page)
{
	__clear_bit(PAGE_DEBUG_FLAG_POISON, &page->debug_flags);
}

static inline bool page_poison(struct page *page)
{
	return test_bit(PAGE_DEBUG_FLAG_POISON, &page->debug_flags);
}

static void poison_page(struct page *page)
{
	void *addr = kmap_atomic(page);

	set_page_poison(page);
	memset(addr, PAGE_POISON, PAGE_SIZE);
	mark_addr_rdonly(addr);
	kunmap_atomic(addr);
}

static void poison_pages(struct page *page, int n)
{
	int i;

	for (i = 0; i < n; i++)
		poison_page(page + i);
}

static bool single_bit_flip(unsigned char a, unsigned char b)
{
	unsigned char error = a ^ b;

	return error && !(error & (error - 1));
}

static void check_poison_mem(unsigned char *mem, size_t bytes)
{
	static DEFINE_RATELIMIT_STATE(ratelimit, 5 * HZ, 10);
	unsigned char *start;
	unsigned char *end;

	start = memchr_inv(mem, PAGE_POISON, bytes);
	if (!start)
		return;

	for (end = mem + bytes - 1; end > start; end--) {
		if (*end != PAGE_POISON)
			break;
	}

	if (!__ratelimit(&ratelimit))
		return;
	else if (start == end && single_bit_flip(*start, PAGE_POISON))
		printk(KERN_ERR "pagealloc: single bit error\n");
	else
		printk(KERN_ERR "pagealloc: memory corruption\n");

	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 16, 1, start,
			end - start + 1, 1);
	BUG_ON(PANIC_CORRUPTION);
	dump_stack();
}

static void unpoison_page(struct page *page)
{
	void *addr;

	if (!page_poison(page))
		return;

	addr = kmap_atomic(page);
	check_poison_mem(addr, PAGE_SIZE);
	mark_addr_rdwrite(addr);
	clear_page_poison(page);
	kunmap_atomic(addr);
}

static void unpoison_pages(struct page *page, int n)
{
	int i;

	for (i = 0; i < n; i++)
		unpoison_page(page + i);
}

#ifdef CONFIG_HTC_DEBUG_PAGE_USER_TRACE

#define page_to_trace(page, free) \
	(free ? \
		&page->trace_free : \
		&page->trace_alloc)
#define page_to_entries_size(page, free) \
	(free ? \
		ARRAY_SIZE(page->trace_free.entries) : \
		ARRAY_SIZE(page->trace_alloc.entries))

static
void htc_trace_pages_user(struct page *page, int numpages, int free)
{
	struct page_user_trace* user_trace;
	struct stack_trace trace;
	unsigned long* entries;
	int nr_entries;

	if (unlikely(!page))
		return;

	user_trace = page_to_trace(page, free);
	entries = user_trace->entries;
	nr_entries = page_to_entries_size(page, free);

	user_trace->pid = current->pid;
	user_trace->tgid = current->tgid;
	memcpy(user_trace->comm, current->comm,
			sizeof(user_trace->comm));
	memcpy(user_trace->tgcomm, current->group_leader->comm,
			sizeof(user_trace->comm));

	if (unlikely(!nr_entries))
		return;

	memset(entries, 0, nr_entries * sizeof(*entries));
	trace.max_entries = nr_entries;
	trace.entries = entries;
	trace.nr_entries = 0;
	trace.skip = 3;
	save_stack_trace(&trace);

	for (; page++, numpages > 1; numpages--)
		memcpy(page_to_trace(page, free), user_trace, sizeof(*user_trace));
}
#else
static inline
void htc_trace_pages_user(struct page *page, int numpages, int free) { }
#endif 

void kernel_map_pages(struct page *page, int numpages, int enable)
{
	if (enable)
		unpoison_pages(page, numpages);
	else
		poison_pages(page, numpages);

	htc_trace_pages_user(page, numpages, !enable);
}
