#ifndef _LINUX_MM_TYPES_H
#define _LINUX_MM_TYPES_H

#include <linux/auxvec.h>
#include <linux/types.h>
#include <linux/threads.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/prio_tree.h>
#include <linux/rbtree.h>
#include <linux/rwsem.h>
#include <linux/completion.h>
#include <linux/cpumask.h>
#include <linux/page-debug-flags.h>
#include <asm/page.h>
#include <asm/mmu.h>

#ifndef AT_VECTOR_SIZE_ARCH
#define AT_VECTOR_SIZE_ARCH 0
#endif
#define AT_VECTOR_SIZE (2*(AT_VECTOR_SIZE_ARCH + AT_VECTOR_SIZE_BASE + 1))

struct address_space;

#define USE_SPLIT_PTLOCKS	(NR_CPUS >= CONFIG_SPLIT_PTLOCK_CPUS)

struct page_user_trace {
	pid_t pid;
	char comm[16];
	pid_t tgid;
	char tgcomm[16];
	unsigned long entries[UL(CONFIG_HTC_DEBUG_PAGE_ENTRIES_NR)];
};

struct page {
	
	unsigned long flags;		
	struct address_space *mapping;	
	
	struct {
		union {
			pgoff_t index;		
			void *freelist;		
		};

		union {
			
			unsigned long counters;

			struct {

				union {
					atomic_t _mapcount;

					struct {
						unsigned inuse:16;
						unsigned objects:15;
						unsigned frozen:1;
					};
				};
				atomic_t _count;		
			};
		};
	};

	
	union {
		struct list_head lru;	
		struct {		
			struct page *next;	
#ifdef CONFIG_64BIT
			int pages;	
			int pobjects;	
#else
			short int pages;
			short int pobjects;
#endif
		};
	};

	
	union {
		unsigned long private;		
#if USE_SPLIT_PTLOCKS
		spinlock_t ptl;
#endif
		struct kmem_cache *slab;	
		struct page *first_page;	
	};

#if defined(WANT_PAGE_VIRTUAL)
	void *virtual;			
#endif 
#ifdef CONFIG_WANT_PAGE_DEBUG_FLAGS
	unsigned long debug_flags;	
#endif

#ifdef CONFIG_KMEMCHECK
	void *shadow;
#endif
#ifdef CONFIG_HTC_DEBUG_PAGE_USER_TRACE
	struct page_user_trace trace_alloc;
	struct page_user_trace trace_free;
#endif
}
#ifdef CONFIG_HAVE_ALIGNED_STRUCT_PAGE
	__aligned(2 * sizeof(unsigned long))
#endif
;

struct page_frag {
	struct page *page;
#if (BITS_PER_LONG > 32) || (PAGE_SIZE >= 65536)
	__u32 offset;
	__u32 size;
#else
	__u16 offset;
	__u16 size;
#endif
};

typedef unsigned long __nocast vm_flags_t;

struct vm_region {
	struct rb_node	vm_rb;		
	vm_flags_t	vm_flags;	
	unsigned long	vm_start;	
	unsigned long	vm_end;		
	unsigned long	vm_top;		
	unsigned long	vm_pgoff;	
	struct file	*vm_file;	

	int		vm_usage;	
	bool		vm_icache_flushed : 1; 
};

struct vm_area_struct {
	struct mm_struct * vm_mm;	
	unsigned long vm_start;		
	unsigned long vm_end;		

	
	struct vm_area_struct *vm_next, *vm_prev;

	pgprot_t vm_page_prot;		
	unsigned long vm_flags;		

	struct rb_node vm_rb;

	union {
		struct {
			struct list_head list;
			void *parent;	
			struct vm_area_struct *head;
		} vm_set;

		struct raw_prio_tree_node prio_tree_node;
		const char __user *anon_name;
	} shared;

	struct list_head anon_vma_chain; 
	struct anon_vma *anon_vma;	

	
	const struct vm_operations_struct *vm_ops;

	
	unsigned long vm_pgoff;		
	struct file * vm_file;		
	void * vm_private_data;		

#ifndef CONFIG_MMU
	struct vm_region *vm_region;	
#endif
#ifdef CONFIG_NUMA
	struct mempolicy *vm_policy;	
#endif
};

struct core_thread {
	struct task_struct *task;
	struct core_thread *next;
};

struct core_state {
	atomic_t nr_threads;
	struct core_thread dumper;
	struct completion startup;
};

enum {
	MM_FILEPAGES,
	MM_ANONPAGES,
	MM_SWAPENTS,
	NR_MM_COUNTERS
};

#if USE_SPLIT_PTLOCKS && defined(CONFIG_MMU)
#define SPLIT_RSS_COUNTING
struct task_rss_stat {
	int events;	
	int count[NR_MM_COUNTERS];
};
#endif 

struct mm_rss_stat {
	atomic_long_t count[NR_MM_COUNTERS];
};

struct mm_struct {
	struct vm_area_struct * mmap;		
	struct rb_root mm_rb;
	struct vm_area_struct * mmap_cache;	
#ifdef CONFIG_MMU
	unsigned long (*get_unmapped_area) (struct file *filp,
				unsigned long addr, unsigned long len,
				unsigned long pgoff, unsigned long flags);
	void (*unmap_area) (struct mm_struct *mm, unsigned long addr);
#endif
	unsigned long mmap_base;		
	unsigned long task_size;		
	unsigned long cached_hole_size; 	
	unsigned long free_area_cache;		
	pgd_t * pgd;
	atomic_t mm_users;			
	atomic_t mm_count;			
	int map_count;				

	spinlock_t page_table_lock;		
	struct rw_semaphore mmap_sem;

	struct list_head mmlist;		


	unsigned long hiwater_rss;	
	unsigned long hiwater_vm;	

	unsigned long total_vm;		
	unsigned long locked_vm;	
	unsigned long pinned_vm;	
	unsigned long shared_vm;	
	unsigned long exec_vm;		
	unsigned long stack_vm;		
	unsigned long reserved_vm;	
	unsigned long def_flags;
	unsigned long nr_ptes;		
	unsigned long start_code, end_code, start_data, end_data;
	unsigned long start_brk, brk, start_stack;
	unsigned long arg_start, arg_end, env_start, env_end;

	unsigned long saved_auxv[AT_VECTOR_SIZE]; 

	struct mm_rss_stat rss_stat;

	struct linux_binfmt *binfmt;

	cpumask_var_t cpu_vm_mask_var;

	
	mm_context_t context;

	unsigned long flags; 

	struct core_state *core_state; 
#ifdef CONFIG_AIO
	spinlock_t		ioctx_lock;
	struct hlist_head	ioctx_list;
#endif
#ifdef CONFIG_MM_OWNER
	struct task_struct __rcu *owner;
#endif

	
	struct file *exe_file;
	unsigned long num_exe_file_vmas;
#ifdef CONFIG_MMU_NOTIFIER
	struct mmu_notifier_mm *mmu_notifier_mm;
#endif
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	pgtable_t pmd_huge_pte; 
#endif
#ifdef CONFIG_CPUMASK_OFFSTACK
	struct cpumask cpumask_allocation;
#endif
};

static inline void mm_init_cpumask(struct mm_struct *mm)
{
#ifdef CONFIG_CPUMASK_OFFSTACK
	mm->cpu_vm_mask_var = &mm->cpumask_allocation;
#endif
}

static inline cpumask_t *mm_cpumask(struct mm_struct *mm)
{
	return mm->cpu_vm_mask_var;
}


static inline const char __user *vma_get_anon_name(struct vm_area_struct *vma)
{
	if (vma->vm_file)
		return NULL;

	return vma->shared.anon_name;
}

#endif 
