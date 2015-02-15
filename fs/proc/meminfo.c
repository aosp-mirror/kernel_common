#include <linux/fs.h>
#include <linux/hugetlb.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/mmzone.h>
#include <linux/proc_fs.h>
#include <linux/quicklist.h>
#include <linux/seq_file.h>
#include <linux/swap.h>
#include <linux/vmstat.h>
#include <linux/atomic.h>
#include <linux/msm_kgsl.h>
#include <linux/msm_ion.h>
#include <linux/vmalloc.h>
#include <asm/page.h>
#include <asm/pgtable.h>

#include <htc_debug/stability/htc_report_meminfo.h>

#include "internal.h"

void __attribute__((weak)) arch_report_meminfo(struct seq_file *m)
{
}

static inline unsigned long free_cma_pages(void)
{
#ifdef CONFIG_CMA
	return global_page_state(NR_FREE_CMA_PAGES);
#else
	return 0UL;
#endif
}

void driver_report_meminfo(struct seq_file *m)
{
	unsigned long kgsl_alloc = kgsl_get_alloc_size(false);
	uintptr_t ion_alloc = msm_ion_heap_meminfo(true);
	uintptr_t ion_inuse = msm_ion_heap_meminfo(false);
	unsigned long free_cma = free_cma_pages();

#define K(x) ((x) << (PAGE_SHIFT - 10))

	seq_printf(m,
		"KgslAlloc:      %8lu kB\n"
		"IonTotal:       %8lu kB\n"
		"IonInUse:       %8lu kB\n"
		"FreeCma:        %8lu kB\n",
		(kgsl_alloc >> 10),
		(ion_alloc >> 10),
		(ion_inuse >> 10),
		K(free_cma));

#undef K
}

static unsigned long subtotal_pages(struct sysinfo *i)
{
	unsigned long ion_total_pages = msm_ion_heap_meminfo(true) >> PAGE_SHIFT;
	unsigned long kernel_stack_pages =
	                (global_page_state(NR_KERNEL_STACK) * THREAD_SIZE) >> PAGE_SHIFT;
	unsigned long slab_pages = global_page_state(NR_SLAB_RECLAIMABLE) +
	                           global_page_state(NR_SLAB_UNRECLAIMABLE);


	return global_page_state(NR_ANON_PAGES) +               
	       i->bufferram +                                   
	       cached_unmapped_pages(i) +                       
	       meminfo_total_pages(NR_DMA_PAGES) +              
	       meminfo_total_pages(NR_DRIVER_ALLOC_PAGES) +     
	       0UL +                                            
	       meminfo_total_pages(NR_IOMMU_PAGETABLES_PAGES) + 
	       ion_total_pages +                                
	       kernel_stack_pages +                             
	       kgsl_unmapped_pages() +                          
	       meminfo_total_pages(NR_KMALLOC_PAGES) +          
	       global_page_state(NR_FILE_MAPPED) +              
	       i->freeram +                                     
	       global_page_state(NR_PAGETABLE) +                
	       slab_pages +                                     
	       total_swapcache_pages +                          
	       vmalloc_alloc_pages();                           

}

static inline unsigned long cached_pages(struct sysinfo *i)
{
	long cached = global_page_state(NR_FILE_PAGES) -
	              total_swapcache_pages - i->bufferram;

	if (cached < 0)
		cached = 0;

	return cached;
}

void show_meminfo(void)
{
	struct sysinfo i;
	long cached;
	unsigned long kgsl_alloc = kgsl_get_alloc_size(true);
	uintptr_t ion_alloc = msm_ion_heap_meminfo(true);
	uintptr_t ion_inuse = msm_ion_heap_meminfo(false);
	unsigned long vmalloc_alloc = vmalloc_alloc_pages();
	unsigned long subtotal;

#define K(x) ((x) << (PAGE_SHIFT - 10))
	si_meminfo(&i);
	si_swapinfo(&i);
	cached = cached_pages(&i);
	subtotal = subtotal_pages(&i);

	printk("MemFree:        %8lu kB\n"
			"Buffers:        %8lu kB\n"
			"Mapped:         %8lu kB\n"
			"Cached:         %8lu kB\n"
			"Shmem:          %8lu kB\n"
			"Mlocked:        %8lu kB\n"
			"AnonPages:      %8lu kB\n"
			"Slab:           %8lu kB\n"
			"PageTables:     %8lu kB\n"
			"KernelStack:    %8lu kB\n"
			"VmallocAlloc:   %8lu kB\n"
			"Kmalloc:        %8lu kB\n"
			"KgslAlloc:      %8lu kB\n"
			"IonTotal:       %8lu kB\n"
			"IonInUse:       %8lu kB\n"
			"Subtotal:       %8lu kB\n",
			K(i.freeram),
			K(i.bufferram),
			K(global_page_state(NR_FILE_MAPPED)),
			K(cached),
			K(global_page_state(NR_SHMEM)),
			K(global_page_state(NR_MLOCK)),
			K(global_page_state(NR_ANON_PAGES)),
			K(global_page_state(NR_SLAB_RECLAIMABLE) + global_page_state(NR_SLAB_UNRECLAIMABLE)),
			K(global_page_state(NR_PAGETABLE)),
			global_page_state(NR_KERNEL_STACK) * THREAD_SIZE / 1024,
			K(vmalloc_alloc),
			K(meminfo_total_pages(NR_KMALLOC_PAGES)),
			(kgsl_alloc >> 10),
			(ion_alloc >> 10),
			(ion_inuse >> 10),
			K(subtotal));
#undef K
}

static int meminfo_proc_show(struct seq_file *m, void *v)
{
	struct sysinfo i;
	unsigned long committed;
	unsigned long allowed;
	struct vmalloc_info vmi;
	long cached;
	unsigned long pages[NR_LRU_LISTS];
	int lru;
#define K(x) ((x) << (PAGE_SHIFT - 10))
	si_meminfo(&i);
	si_swapinfo(&i);
	committed = percpu_counter_read_positive(&vm_committed_as);
	allowed = ((totalram_pages - hugetlb_total_pages())
		* sysctl_overcommit_ratio / 100) + total_swap_pages;

	cached = global_page_state(NR_FILE_PAGES) -
			total_swapcache_pages - i.bufferram;
	if (cached < 0)
		cached = 0;

	get_vmalloc_info(&vmi);

	for (lru = LRU_BASE; lru < NR_LRU_LISTS; lru++)
		pages[lru] = global_page_state(NR_LRU_BASE + lru);

	seq_printf(m,
		"MemTotal:       %8lu kB\n"
		"MemFree:        %8lu kB\n"
		"Buffers:        %8lu kB\n"
		"Cached:         %8lu kB\n"
		"SwapCached:     %8lu kB\n"
		"Active:         %8lu kB\n"
		"Inactive:       %8lu kB\n"
		"Active(anon):   %8lu kB\n"
		"Inactive(anon): %8lu kB\n"
		"Active(file):   %8lu kB\n"
		"Inactive(file): %8lu kB\n"
		"Unevictable:    %8lu kB\n"
		"Mlocked:        %8lu kB\n"
#ifdef CONFIG_HIGHMEM
		"HighTotal:      %8lu kB\n"
		"HighFree:       %8lu kB\n"
		"LowTotal:       %8lu kB\n"
		"LowFree:        %8lu kB\n"
#endif
#ifndef CONFIG_MMU
		"MmapCopy:       %8lu kB\n"
#endif
		"SwapTotal:      %8lu kB\n"
		"SwapFree:       %8lu kB\n"
		"Dirty:          %8lu kB\n"
		"Writeback:      %8lu kB\n"
		"AnonPages:      %8lu kB\n"
		"Mapped:         %8lu kB\n"
		"Shmem:          %8lu kB\n"
		"Slab:           %8lu kB\n"
		"SReclaimable:   %8lu kB\n"
		"SUnreclaim:     %8lu kB\n"
		"KernelStack:    %8lu kB\n"
		"PageTables:     %8lu kB\n"
#ifdef CONFIG_QUICKLIST
		"Quicklists:     %8lu kB\n"
#endif
		"NFS_Unstable:   %8lu kB\n"
		"Bounce:         %8lu kB\n"
		"WritebackTmp:   %8lu kB\n"
		"CommitLimit:    %8lu kB\n"
		"Committed_AS:   %8lu kB\n"
		"VmallocTotal:   %8lu kB\n"
		"VmallocUsed:    %8lu kB\n"
		"VmallocIoRemap: %8lu kB\n"
		"VmallocAlloc:   %8lu kB\n"
		"VmallocMap:     %8lu kB\n"
		"VmallocUserMap: %8lu kB\n"
		"VmallocVpage:   %8lu kB\n"
		"VmallocChunk:   %8lu kB\n"
#ifdef CONFIG_MEMORY_FAILURE
		"HardwareCorrupted: %5lu kB\n"
#endif
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
		"AnonHugePages:  %8lu kB\n"
#endif
		,
		K(i.totalram),
		K(i.freeram),
		K(i.bufferram),
		K(cached),
		K(total_swapcache_pages),
		K(pages[LRU_ACTIVE_ANON]   + pages[LRU_ACTIVE_FILE]),
		K(pages[LRU_INACTIVE_ANON] + pages[LRU_INACTIVE_FILE]),
		K(pages[LRU_ACTIVE_ANON]),
		K(pages[LRU_INACTIVE_ANON]),
		K(pages[LRU_ACTIVE_FILE]),
		K(pages[LRU_INACTIVE_FILE]),
		K(pages[LRU_UNEVICTABLE]),
		K(global_page_state(NR_MLOCK)),
#ifdef CONFIG_HIGHMEM
		K(i.totalhigh),
		K(i.freehigh),
		K(i.totalram-i.totalhigh),
		K(i.freeram-i.freehigh),
#endif
#ifndef CONFIG_MMU
		K((unsigned long) atomic_long_read(&mmap_pages_allocated)),
#endif
		K(i.totalswap),
		K(i.freeswap),
		K(global_page_state(NR_FILE_DIRTY)),
		K(global_page_state(NR_WRITEBACK)),
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
		K(global_page_state(NR_ANON_PAGES)
		  + global_page_state(NR_ANON_TRANSPARENT_HUGEPAGES) *
		  HPAGE_PMD_NR),
#else
		K(global_page_state(NR_ANON_PAGES)),
#endif
		K(global_page_state(NR_FILE_MAPPED)),
		K(global_page_state(NR_SHMEM)),
		K(global_page_state(NR_SLAB_RECLAIMABLE) +
				global_page_state(NR_SLAB_UNRECLAIMABLE)),
		K(global_page_state(NR_SLAB_RECLAIMABLE)),
		K(global_page_state(NR_SLAB_UNRECLAIMABLE)),
		global_page_state(NR_KERNEL_STACK) * THREAD_SIZE / 1024,
		K(global_page_state(NR_PAGETABLE)),
#ifdef CONFIG_QUICKLIST
		K(quicklist_total_size()),
#endif
		K(global_page_state(NR_UNSTABLE_NFS)),
		K(global_page_state(NR_BOUNCE)),
		K(global_page_state(NR_WRITEBACK_TEMP)),
		K(allowed),
		K(committed),
		(unsigned long)(VMALLOC_TOTAL + report_vmalloc_saving_size()) >> 10,
		vmi.used >> 10,
		vmi.ioremap >> 10,
#ifdef CONFIG_HTC_DEBUG_REPORT_MEMINFO
		K(meminfo_total_pages(NR_VMALLOC_PAGES)),
#else
		vmi.alloc >> 10,
#endif
		vmi.map >> 10,
		vmi.usermap >> 10,
		vmi.vpages >> 10,
		vmi.largest_chunk >> 10
#ifdef CONFIG_MEMORY_FAILURE
		,atomic_long_read(&mce_bad_pages) << (PAGE_SHIFT - 10)
#endif
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
		,K(global_page_state(NR_ANON_TRANSPARENT_HUGEPAGES) *
		   HPAGE_PMD_NR)
#endif
		);

	hugetlb_report_meminfo(m);

	arch_report_meminfo(m);

	driver_report_meminfo(m);

	report_meminfo(m, &i);

	return 0;
#undef K
}

static int meminfo_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, meminfo_proc_show, NULL);
}

static const struct file_operations meminfo_proc_fops = {
	.open		= meminfo_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_meminfo_init(void)
{
	proc_create("meminfo", 0, NULL, &meminfo_proc_fops);
	return 0;
}
module_init(proc_meminfo_init);
