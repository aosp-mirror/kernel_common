#include <linux/mm_types.h>
#include <linux/mm.h>
#include <linux/seq_file.h>
#include <linux/swap.h>
#include <linux/fs.h>
#include <linux/msm_kgsl.h>
#include <htc_debug/stability/htc_report_meminfo.h>

static atomic_long_t meminfo_stat[NR_MEMINFO_STAT_ITEMS] =
	{[0 ... NR_MEMINFO_STAT_ITEMS - 1] = ATOMIC_LONG_INIT(0)};

const char * const meminfo_stat_text[] = {
	[NR_KMALLOC_PAGES] = "Kmalloc",
	[NR_VMALLOC_PAGES] = "VmallocAlloc",
	[NR_DMA_PAGES] = "DmaAlloc",
	[NR_IOMMU_PAGETABLES_PAGES] = "IommuPgd",
	[NR_DRIVER_ALLOC_PAGES] = "DriverAlloc",
	[NR_HASHTABLES_PAGES] = "HashTables",
	[NR_MEMPOOL_ALLOC_PAGES] = "MemPoolAlloc",
};

static atomic_long_t cached_mapped_stat = ATOMIC_LONG_INIT(0);

static atomic_long_t kgsl_mapped_stat = ATOMIC_LONG_INIT(0);

void kmalloc_count(struct page *page, int to_alloc)
{
	if (unlikely(!page))
		return;

	if (to_alloc) {
		int order = compound_order(page);

		SetPageKmalloc(page);
		add_meminfo_total_pages(NR_KMALLOC_PAGES, 1 << order);
	} else if (PageKmalloc(page)) {
		int order = compound_order(page);

		ClearPageKmalloc(page);
		sub_meminfo_total_pages(NR_KMALLOC_PAGES, 1 << order);
	}
}

struct super_block;
extern int sb_is_blkdev_sb(struct super_block *sb);

static inline int page_is_cached(struct page *page)
{
	return	page_mapping(page) && !PageSwapCache(page) &&
		page->mapping->host && page->mapping->host->i_sb &&
		!sb_is_blkdev_sb(page->mapping->host->i_sb);
}

void mapped_count(struct page *page, int to_map)
{
	if (unlikely(!page))
		return;

	if (PageKgsl(page)) {
		if (to_map)
			atomic_long_inc(&kgsl_mapped_stat);
		else
			atomic_long_dec(&kgsl_mapped_stat);
	} else if (page_is_cached(page)) {
		if (to_map)
			atomic_long_inc(&cached_mapped_stat);
		else
			atomic_long_dec(&cached_mapped_stat);
	}
}

static unsigned long driver_alloc_total_pages(void)
{
	long total = 0UL;
	int i;

	for (i = NR_DRIVER_ALLOC_BASE; i < NR_MEMINFO_STAT_ITEMS; i++)
		total += atomic_long_read(&meminfo_stat[i]);

	if (total < 0)
		total = 0;
	return (unsigned long) total;
}

static inline unsigned long cached_pages(struct sysinfo *i)
{
	long cached = global_page_state(NR_FILE_PAGES) -
	              total_swapcache_pages - i->bufferram;

	if (cached < 0)
		cached = 0;

	return cached;
}

unsigned long cached_unmapped_pages(struct sysinfo *i)
{
	long cached_unmapped = (long)cached_pages(i) -
	                       atomic_long_read(&cached_mapped_stat);

	if (cached_unmapped < 0)
		cached_unmapped = 0;

	return cached_unmapped;
}

unsigned long kgsl_unmapped_pages(void)
{
	long kgsl_alloc = kgsl_get_alloc_size(false) >> PAGE_SHIFT;
	long kgsl_unmapped = kgsl_alloc -
	                atomic_long_read(&kgsl_mapped_stat);

	if (kgsl_unmapped < 0)
		kgsl_unmapped = 0;

	return kgsl_unmapped;
}


unsigned long meminfo_total_pages(enum meminfo_stat_item item)
{
	long total = atomic_long_read(&meminfo_stat[item]);

	if (item == NR_DRIVER_ALLOC_PAGES)
		return driver_alloc_total_pages();

	if (total < 0)
		total = 0;
	return (unsigned long) total;
}

void inc_meminfo_total_pages(enum meminfo_stat_item item)
{
	atomic_long_inc(&meminfo_stat[item]);
}

void dec_meminfo_total_pages(enum meminfo_stat_item item)
{
	atomic_long_dec(&meminfo_stat[item]);
}

void add_meminfo_total_pages(enum meminfo_stat_item item, int delta)
{
	atomic_long_add(delta, &meminfo_stat[item]);
}

void sub_meminfo_total_pages(enum meminfo_stat_item item, int delta)
{
	atomic_long_sub(delta, &meminfo_stat[item]);
}

void report_meminfo_item(struct seq_file *m, enum meminfo_stat_item item)
{
	unsigned long total = meminfo_total_pages(item);
	char buf[32];

	snprintf(buf, 32, "%s:", meminfo_stat_text[item]);
#define K(x) ((x) << (PAGE_SHIFT - 10))
	seq_printf(m, "%-16s%8lu kB\n", buf, K(total));
#undef K
}

static void report_unmapped(struct seq_file *m, struct sysinfo *sysinfo)
{
	unsigned long cached_unmapped = cached_unmapped_pages(sysinfo);
	unsigned long kgsl_unmapped = kgsl_unmapped_pages();

#define K(x) ((x) << (PAGE_SHIFT - 10))
	seq_printf(m, "%-16s%8lu kB\n", "CachedUnmapped: ", K(cached_unmapped));
	seq_printf(m, "%-16s%8lu kB\n", "KgslUnmapped: ", K(kgsl_unmapped));
#undef K
}

void report_meminfo(struct seq_file *m, struct sysinfo *sysinfo)
{
	int i;

	for (i = 0; i <= NR_DRIVER_ALLOC_PAGES; i++) {
		if (i == NR_VMALLOC_PAGES)
			continue;
		report_meminfo_item(m, i);
	}

	report_unmapped(m, sysinfo);

#define K(x) ((x) << (PAGE_SHIFT - 10))
	seq_printf(m, "%-16s%8lu kB\n", "Ftrace: ", K(ftrace_total_pages()));
#undef K
}
