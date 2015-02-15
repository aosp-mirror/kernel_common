#ifndef _HTC_DEBUG_STABILITY_REPORT_MEMINFO
#define _HTC_DEBUG_STABILITY_REPORT_MEMINFO

struct page;
struct seq_file;
struct sysinfo;

enum meminfo_stat_item {
	NR_KMALLOC_PAGES,
	NR_VMALLOC_PAGES,
	NR_DMA_PAGES,
	NR_IOMMU_PAGETABLES_PAGES,
	NR_DRIVER_ALLOC_PAGES,
	NR_DRIVER_ALLOC_BASE = NR_DRIVER_ALLOC_PAGES,
	NR_HASHTABLES_PAGES,
	NR_MEMPOOL_ALLOC_PAGES,
	NR_MEMINFO_STAT_ITEMS};

extern unsigned long ftrace_total_pages(void);

#ifdef CONFIG_HTC_DEBUG_REPORT_MEMINFO

void kmalloc_count(struct page *page, int to_alloc);

void mapped_count(struct page *page, int to_map);

static inline void pgd_alloc_count(unsigned long addr)
{
	if (likely(addr)) {
		struct page *page = virt_to_page(addr);
		__mod_zone_page_state(page_zone(page), NR_PAGETABLE, 1 << 2);
	}
}

static inline void pgd_free_count(unsigned long addr)
{
	if (likely(addr && virt_addr_valid((void *)addr))) {
		struct page *page;

		page = virt_to_page(addr);
		__mod_zone_page_state(page_zone(page), NR_PAGETABLE, -(1 << 2));
	}
}

unsigned long meminfo_total_pages(enum meminfo_stat_item item);

static inline unsigned long vmalloc_alloc_pages(void)
{
	return meminfo_total_pages(NR_VMALLOC_PAGES);
}

unsigned long cached_unmapped_pages(struct sysinfo *i);

unsigned long kgsl_unmapped_pages(void);

void inc_meminfo_total_pages(enum meminfo_stat_item item);

void dec_meminfo_total_pages(enum meminfo_stat_item item);

void add_meminfo_total_pages(enum meminfo_stat_item item, int delta);

void sub_meminfo_total_pages(enum meminfo_stat_item item, int delta);

static inline void inc_meminfo_total_pages_on(enum meminfo_stat_item item, bool cond)
{
	if (likely(cond))
		inc_meminfo_total_pages(item);

}

static inline void dec_meminfo_total_pages_on(enum meminfo_stat_item item, bool cond)
{
	if (likely(cond))
		dec_meminfo_total_pages(item);
}

static inline void add_meminfo_total_pages_on(enum meminfo_stat_item item, int delta, bool cond)
{
	if (likely(cond))
		add_meminfo_total_pages(item, delta);
}

static inline void sub_meminfo_total_pages_on(enum meminfo_stat_item item, int delta, bool cond)
{
	if (likely(cond))
		sub_meminfo_total_pages(item, delta);
}

void report_meminfo_item(struct seq_file *m, enum meminfo_stat_item item);

void report_meminfo(struct seq_file *m, struct sysinfo *sysinfo);

#else 

static inline void kmalloc_count(struct page *page, int to_alloc)
{
}

static inline void mapped_count(struct page *page, int to_map)
{
}

static inline void pgd_alloc_count(unsigned long addr)
{
}

static inline void pgd_free_count(unsigned long addr)
{
}

static inline unsigned long meminfo_total_pages(enum meminfo_stat_item item)
{
	return 0UL;
}

static inline unsigned long vmalloc_alloc_pages(void)
{
	struct vmalloc_info vmi;

	get_vmalloc_info(&vmi);

	return vmi.alloc >> PAGE_SHIFT;
}

static inline unsigned long cached_unmapped_pages(struct sysinfo *i)
{
	return 0UL;
}

static inline unsigned long kgsl_unmapped_pages()
{
	return 0UL;
}

static inline void inc_meminfo_total_pages(enum meminfo_stat_item item)
{
}

static inline void dec_meminfo_total_pages(enum meminfo_stat_item item)
{
}

static inline void add_meminfo_total_pages(enum meminfo_stat_item item, int delta)
{
}

static inline void sub_meminfo_total_pages(enum meminfo_stat_item item, int delta)
{
}

static inline void inc_meminfo_total_pages_on(enum meminfo_stat_item item, bool cond)
{
}

static inline void dec_meminfo_total_pages_on(enum meminfo_stat_item item, bool cond)
{
}

static inline void add_meminfo_total_pages_on(enum meminfo_stat_item item, int delta, bool cond)
{
}

static inline void sub_meminfo_total_pages_on(enum meminfo_stat_item item, int delta, bool cond)
{
}

static inline void report_meminfo_item(struct seq_file *m, enum meminfo_stat_item item)
{
}

static inline void report_meminfo(struct seq_file *m, struct sysinfo *sysinfo)
{
}

#endif 

#endif 
