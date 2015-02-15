/*
 *  linux/mm/vmalloc.c
 *
 *  Copyright (C) 1993  Linus Torvalds
 *  Support of BIGMEM added by Gerhard Wichert, Siemens AG, July 1999
 *  SMP-safe vmalloc/vfree/ioremap, Tigran Aivazian <tigran@veritas.com>, May 2000
 *  Major rework to support vmap/vunmap, Christoph Hellwig, SGI, August 2002
 *  Numa awareness, Christoph Lameter, SGI, June 2005
 */

#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/highmem.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/debugobjects.h>
#include <linux/kallsyms.h>
#include <linux/list.h>
#include <linux/rbtree.h>
#include <linux/radix-tree.h>
#include <linux/rcupdate.h>
#include <linux/pfn.h>
#include <linux/kmemleak.h>
#include <linux/atomic.h>
#include <asm/uaccess.h>
#include <asm/tlbflush.h>
#include <asm/shmparam.h>
#include <asm/setup.h>

#include <htc_debug/stability/htc_report_meminfo.h>


static void vunmap_pte_range(pmd_t *pmd, unsigned long addr, unsigned long end)
{
	pte_t *pte;

	pte = pte_offset_kernel(pmd, addr);
	do {
		pte_t ptent = ptep_get_and_clear(&init_mm, addr, pte);
		WARN_ON(!pte_none(ptent) && !pte_present(ptent));
	} while (pte++, addr += PAGE_SIZE, addr != end);
}

static void vunmap_pmd_range(pud_t *pud, unsigned long addr, unsigned long end)
{
	pmd_t *pmd;
	unsigned long next;

	pmd = pmd_offset(pud, addr);
	do {
		next = pmd_addr_end(addr, end);
		if (pmd_none_or_clear_bad(pmd))
			continue;
		vunmap_pte_range(pmd, addr, next);
	} while (pmd++, addr = next, addr != end);
}

static void vunmap_pud_range(pgd_t *pgd, unsigned long addr, unsigned long end)
{
	pud_t *pud;
	unsigned long next;

	pud = pud_offset(pgd, addr);
	do {
		next = pud_addr_end(addr, end);
		if (pud_none_or_clear_bad(pud))
			continue;
		vunmap_pmd_range(pud, addr, next);
	} while (pud++, addr = next, addr != end);
}

static void vunmap_page_range(unsigned long addr, unsigned long end)
{
	pgd_t *pgd;
	unsigned long next;

	BUG_ON(addr >= end);
	pgd = pgd_offset_k(addr);
	do {
		next = pgd_addr_end(addr, end);
		if (pgd_none_or_clear_bad(pgd))
			continue;
		vunmap_pud_range(pgd, addr, next);
	} while (pgd++, addr = next, addr != end);
}

static int vmap_pte_range(pmd_t *pmd, unsigned long addr,
		unsigned long end, pgprot_t prot, struct page **pages, int *nr)
{
	pte_t *pte;


	pte = pte_alloc_kernel(pmd, addr);
	if (!pte)
		return -ENOMEM;
	do {
		struct page *page = pages[*nr];

		if (WARN_ON(!pte_none(*pte)))
			return -EBUSY;
		if (WARN_ON(!page))
			return -ENOMEM;
		set_pte_at(&init_mm, addr, pte, mk_pte(page, prot));
		(*nr)++;
	} while (pte++, addr += PAGE_SIZE, addr != end);
	return 0;
}

static int vmap_pmd_range(pud_t *pud, unsigned long addr,
		unsigned long end, pgprot_t prot, struct page **pages, int *nr)
{
	pmd_t *pmd;
	unsigned long next;

	pmd = pmd_alloc(&init_mm, pud, addr);
	if (!pmd)
		return -ENOMEM;
	do {
		next = pmd_addr_end(addr, end);
		if (vmap_pte_range(pmd, addr, next, prot, pages, nr))
			return -ENOMEM;
	} while (pmd++, addr = next, addr != end);
	return 0;
}

static int vmap_pud_range(pgd_t *pgd, unsigned long addr,
		unsigned long end, pgprot_t prot, struct page **pages, int *nr)
{
	pud_t *pud;
	unsigned long next;

	pud = pud_alloc(&init_mm, pgd, addr);
	if (!pud)
		return -ENOMEM;
	do {
		next = pud_addr_end(addr, end);
		if (vmap_pmd_range(pud, addr, next, prot, pages, nr))
			return -ENOMEM;
	} while (pud++, addr = next, addr != end);
	return 0;
}

static int vmap_page_range_noflush(unsigned long start, unsigned long end,
				   pgprot_t prot, struct page **pages)
{
	pgd_t *pgd;
	unsigned long next;
	unsigned long addr = start;
	int err = 0;
	int nr = 0;

	BUG_ON(addr >= end);
	pgd = pgd_offset_k(addr);
	do {
		next = pgd_addr_end(addr, end);
		err = vmap_pud_range(pgd, addr, next, prot, pages, &nr);
		if (err)
			return err;
	} while (pgd++, addr = next, addr != end);

	return nr;
}

static int vmap_page_range(unsigned long start, unsigned long end,
			   pgprot_t prot, struct page **pages)
{
	int ret;

	ret = vmap_page_range_noflush(start, end, prot, pages);
	flush_cache_vmap(start, end);
	return ret;
}

int is_vmalloc_or_module_addr(const void *x)
{
#if defined(CONFIG_MODULES) && defined(MODULES_VADDR)
	unsigned long addr = (unsigned long)x;
	if (addr >= MODULES_VADDR && addr < MODULES_END)
		return 1;
#endif
	return is_vmalloc_addr(x);
}

struct page *vmalloc_to_page(const void *vmalloc_addr)
{
	unsigned long addr = (unsigned long) vmalloc_addr;
	struct page *page = NULL;
	pgd_t *pgd = pgd_offset_k(addr);

	VIRTUAL_BUG_ON(!is_vmalloc_or_module_addr(vmalloc_addr));

	if (!pgd_none(*pgd)) {
		pud_t *pud = pud_offset(pgd, addr);
		if (!pud_none(*pud)) {
			pmd_t *pmd = pmd_offset(pud, addr);
			if (!pmd_none(*pmd)) {
				pte_t *ptep, pte;

				ptep = pte_offset_map(pmd, addr);
				pte = *ptep;
				if (pte_present(pte))
					page = pte_page(pte);
				pte_unmap(ptep);
			}
		}
	}
	return page;
}
EXPORT_SYMBOL(vmalloc_to_page);

unsigned long vmalloc_to_pfn(const void *vmalloc_addr)
{
	return page_to_pfn(vmalloc_to_page(vmalloc_addr));
}
EXPORT_SYMBOL(vmalloc_to_pfn);



#define VM_LAZY_FREE	0x01
#define VM_LAZY_FREEING	0x02
#define VM_VM_AREA	0x04

struct vmap_area {
	unsigned long va_start;
	unsigned long va_end;
	unsigned long flags;
	struct rb_node rb_node;		
	struct list_head list;		
	struct list_head purge_list;	
	struct vm_struct *vm;
	struct rcu_head rcu_head;
};

static DEFINE_SPINLOCK(vmap_area_lock);
static LIST_HEAD(vmap_area_list);
static struct rb_root vmap_area_root = RB_ROOT;

static struct rb_node *free_vmap_cache;
static unsigned long cached_hole_size;
static unsigned long cached_vstart;
static unsigned long cached_align;

static unsigned long vmap_area_pcpu_hole;

#ifdef CONFIG_ENABLE_VMALLOC_SAVING
#define POSSIBLE_VMALLOC_START	PAGE_OFFSET

#define VMALLOC_BITMAP_SIZE	((VMALLOC_END - PAGE_OFFSET) >> \
					PAGE_SHIFT)
#define VMALLOC_TO_BIT(addr)	((addr - PAGE_OFFSET) >> PAGE_SHIFT)
#define BIT_TO_VMALLOC(i)	(PAGE_OFFSET + i * PAGE_SIZE)

DECLARE_BITMAP(possible_areas, VMALLOC_BITMAP_SIZE);

void mark_vmalloc_reserved_area(void *x, unsigned long size)
{
	unsigned long addr = (unsigned long)x;

	bitmap_set(possible_areas, VMALLOC_TO_BIT(addr), size >> PAGE_SHIFT);
}

phys_addr_t report_vmalloc_saving_size(void)
{
	phys_addr_t vmalloc_saving_size = 0;
	int i;

	for (i = meminfo.nr_banks - 1; i >= 1; i--) {
		if (meminfo.bank[i-1].start + meminfo.bank[i-1].size !=
			  meminfo.bank[i].start) {
			if (meminfo.bank[i-1].start + meminfo.bank[i-1].size
				   <= MAX_HOLE_ADDRESS) {
				phys_addr_t hole_start =
					meminfo.bank[i-1].start + meminfo.bank[i-1].size;
				phys_addr_t hole_end = meminfo.bank[i].start;

				vmalloc_saving_size += hole_end - hole_start;
			}
		}
	}

	return vmalloc_saving_size;
}

int is_vmalloc_addr(const void *x)
{
	unsigned long addr = (unsigned long)x;

	if (addr < POSSIBLE_VMALLOC_START || addr >= VMALLOC_END)
		return 0;

	if (test_bit(VMALLOC_TO_BIT(addr), possible_areas))
		return 0;

	return 1;
}
#else
int is_vmalloc_addr(const void *x)
{
	unsigned long addr = (unsigned long)x;

	return addr >= VMALLOC_START && addr < VMALLOC_END;
}
#endif
EXPORT_SYMBOL(is_vmalloc_addr);



static struct vmap_area *__find_vmap_area(unsigned long addr)
{
	struct rb_node *n = vmap_area_root.rb_node;

	while (n) {
		struct vmap_area *va;

		va = rb_entry(n, struct vmap_area, rb_node);
		if (addr < va->va_start)
			n = n->rb_left;
		else if (addr > va->va_start)
			n = n->rb_right;
		else
			return va;
	}

	return NULL;
}

static void __insert_vmap_area(struct vmap_area *va)
{
	struct rb_node **p = &vmap_area_root.rb_node;
	struct rb_node *parent = NULL;
	struct rb_node *tmp;

	while (*p) {
		struct vmap_area *tmp_va;

		parent = *p;
		tmp_va = rb_entry(parent, struct vmap_area, rb_node);
		if (va->va_start < tmp_va->va_end)
			p = &(*p)->rb_left;
		else if (va->va_end > tmp_va->va_start)
			p = &(*p)->rb_right;
		else
			BUG();
	}

	rb_link_node(&va->rb_node, parent, p);
	rb_insert_color(&va->rb_node, &vmap_area_root);

	
	tmp = rb_prev(&va->rb_node);
	if (tmp) {
		struct vmap_area *prev;
		prev = rb_entry(tmp, struct vmap_area, rb_node);
		list_add_rcu(&va->list, &prev->list);
	} else
		list_add_rcu(&va->list, &vmap_area_list);
}

static void purge_vmap_area_lazy(void);

#ifdef CONFIG_HTC_DEBUG_VMALLOC_DUMP

#define DUMP_VMALLOC_INTERVAL  10*HZ 
#define MLM(b, t) b, t, ((t) - (b)) >> 20

void dump_vmallocinfo(void)
{
	struct vm_struct *v=vmlist;
	unsigned int used = 0;

	read_lock(&vmlist_lock);
	printk("[K] prepare to dump vmallocinfo\n");

	while(v) {
		printk("0x%p-0x%p %7ld", v->addr, v->addr + v->size, v->size);
		used += v->size;

		if(v->caller)
			printk(" %pS", v->caller);

		if (v->nr_pages)
			printk(" pages=%d", v->nr_pages);

		if (v->phys_addr)
			printk(" phys=%llx", (unsigned long long)v->phys_addr);

		if (v->flags & VM_IOREMAP)
			printk(" ioremap");

		if (v->flags & VM_ALLOC)
			printk(" vmalloc");

		if (v->flags & VM_MAP)
			printk(" vmap");

		if (v->flags & VM_USERMAP)
			printk(" user");

		if (v->flags & VM_VPAGES)
			printk(" vpages");

		printk("\n");

		v = v->next;
	}

	printk("[K] vmalloc : 0x%08lx - 0x%08lx   (%4ld MB)\n", MLM(VMALLOC_START, VMALLOC_END) );
	printk("[K] vmalloc used : %u bytes\n", used );
	printk("[K] end of dump vmallocinfo\n");

	read_unlock(&vmlist_lock);
}
EXPORT_SYMBOL(dump_vmallocinfo);

static unsigned long last_dump_jiffies =0; 

#endif

static struct vmap_area *alloc_vmap_area(unsigned long size,
				unsigned long align,
				unsigned long vstart, unsigned long vend,
				int node, gfp_t gfp_mask)
{
	struct vmap_area *va;
	struct rb_node *n;
	unsigned long addr;
	int purged = 0;
	struct vmap_area *first;

	BUG_ON(!size);
	BUG_ON(size & ~PAGE_MASK);
	BUG_ON(!is_power_of_2(align));

	va = kmalloc_node(sizeof(struct vmap_area),
			gfp_mask & GFP_RECLAIM_MASK, node);
	if (unlikely(!va))
		return ERR_PTR(-ENOMEM);

retry:
	spin_lock(&vmap_area_lock);
	if (!free_vmap_cache ||
			size < cached_hole_size ||
			vstart < cached_vstart ||
			align < cached_align) {
nocache:
		cached_hole_size = 0;
		free_vmap_cache = NULL;
	}
	
	cached_vstart = vstart;
	cached_align = align;

	
	if (free_vmap_cache) {
		first = rb_entry(free_vmap_cache, struct vmap_area, rb_node);
		addr = ALIGN(first->va_end, align);
		if (addr < vstart)
			goto nocache;
		if (addr + size - 1 < addr)
			goto overflow;

	} else {
		addr = ALIGN(vstart, align);
		if (addr + size - 1 < addr)
			goto overflow;

		n = vmap_area_root.rb_node;
		first = NULL;

		while (n) {
			struct vmap_area *tmp;
			tmp = rb_entry(n, struct vmap_area, rb_node);
			if (tmp->va_end >= addr) {
				first = tmp;
				if (tmp->va_start <= addr)
					break;
				n = n->rb_left;
			} else
				n = n->rb_right;
		}

		if (!first)
			goto found;
	}

	
	while (addr + size > first->va_start && addr + size <= vend) {
		if (addr + cached_hole_size < first->va_start)
			cached_hole_size = first->va_start - addr;
		addr = ALIGN(first->va_end, align);
		if (addr + size - 1 < addr)
			goto overflow;

		n = rb_next(&first->rb_node);
		if (n)
			first = rb_entry(n, struct vmap_area, rb_node);
		else
			goto found;
	}

found:
	if (addr + size > vend)
		goto overflow;

	va->va_start = addr;
	va->va_end = addr + size;
	va->flags = 0;
	__insert_vmap_area(va);
	free_vmap_cache = &va->rb_node;
	spin_unlock(&vmap_area_lock);

	BUG_ON(va->va_start & (align-1));
	BUG_ON(va->va_start < vstart);
	BUG_ON(va->va_end > vend);

	return va;

overflow:
	spin_unlock(&vmap_area_lock);
	if (!purged) {
		purge_vmap_area_lazy();
		purged = 1;
		goto retry;
	}
	if (printk_ratelimit())
		printk(KERN_WARNING
			"vmap allocation for size %lu failed: "
			"use vmalloc=<size> to increase size.\n", size);

#ifdef CONFIG_HTC_DEBUG_VMALLOC_DUMP
	if((last_dump_jiffies == 0) || time_is_before_jiffies(last_dump_jiffies + DUMP_VMALLOC_INTERVAL) ) {
		dump_vmallocinfo();
		last_dump_jiffies = jiffies;
	}
#endif
	kfree(va);
	return ERR_PTR(-EBUSY);
}

static void __free_vmap_area(struct vmap_area *va)
{
	BUG_ON(RB_EMPTY_NODE(&va->rb_node));

	if (free_vmap_cache) {
		if (va->va_end < cached_vstart) {
			free_vmap_cache = NULL;
		} else {
			struct vmap_area *cache;
			cache = rb_entry(free_vmap_cache, struct vmap_area, rb_node);
			if (va->va_start <= cache->va_start) {
				free_vmap_cache = rb_prev(&va->rb_node);
			}
		}
	}
	rb_erase(&va->rb_node, &vmap_area_root);
	RB_CLEAR_NODE(&va->rb_node);
	list_del_rcu(&va->list);

	if (va->va_end > VMALLOC_START && va->va_end <= VMALLOC_END)
		vmap_area_pcpu_hole = max(vmap_area_pcpu_hole, va->va_end);

	kfree_rcu(va, rcu_head);
}

static void free_vmap_area(struct vmap_area *va)
{
	spin_lock(&vmap_area_lock);
	__free_vmap_area(va);
	spin_unlock(&vmap_area_lock);
}

static void unmap_vmap_area(struct vmap_area *va)
{
	vunmap_page_range(va->va_start, va->va_end);
}

static void vmap_debug_free_range(unsigned long start, unsigned long end)
{
#ifdef CONFIG_DEBUG_PAGEALLOC
	vunmap_page_range(start, end);
	flush_tlb_kernel_range(start, end);
#endif
}

static unsigned long lazy_max_pages(void)
{
	unsigned int log;

	log = fls(num_online_cpus());

	return log * (32UL * 1024 * 1024 / PAGE_SIZE);
}

static atomic_t vmap_lazy_nr = ATOMIC_INIT(0);

static void purge_fragmented_blocks_allcpus(void);

void set_iounmap_nonlazy(void)
{
	atomic_set(&vmap_lazy_nr, lazy_max_pages()+1);
}

static void __purge_vmap_area_lazy(unsigned long *start, unsigned long *end,
					int sync, int force_flush)
{
	static DEFINE_SPINLOCK(purge_lock);
	LIST_HEAD(valist);
	struct vmap_area *va;
	struct vmap_area *n_va;
	int nr = 0;

	if (!sync && !force_flush) {
		if (!spin_trylock(&purge_lock))
			return;
	} else
		spin_lock(&purge_lock);

	if (sync)
		purge_fragmented_blocks_allcpus();

	rcu_read_lock();
	list_for_each_entry_rcu(va, &vmap_area_list, list) {
		if (va->flags & VM_LAZY_FREE) {
			if (va->va_start < *start)
				*start = va->va_start;
			if (va->va_end > *end)
				*end = va->va_end;
			nr += (va->va_end - va->va_start) >> PAGE_SHIFT;
			list_add_tail(&va->purge_list, &valist);
			va->flags |= VM_LAZY_FREEING;
			va->flags &= ~VM_LAZY_FREE;
		}
	}
	rcu_read_unlock();

	if (nr)
		atomic_sub(nr, &vmap_lazy_nr);

	if (nr || force_flush)
		flush_tlb_kernel_range(*start, *end);

	if (nr) {
		spin_lock(&vmap_area_lock);
		list_for_each_entry_safe(va, n_va, &valist, purge_list)
			__free_vmap_area(va);
		spin_unlock(&vmap_area_lock);
	}
	spin_unlock(&purge_lock);
}

static void try_purge_vmap_area_lazy(void)
{
	unsigned long start = ULONG_MAX, end = 0;

	__purge_vmap_area_lazy(&start, &end, 0, 0);
}

static void purge_vmap_area_lazy(void)
{
	unsigned long start = ULONG_MAX, end = 0;

	__purge_vmap_area_lazy(&start, &end, 1, 0);
}

static void free_vmap_area_noflush(struct vmap_area *va)
{
	va->flags |= VM_LAZY_FREE;
	atomic_add((va->va_end - va->va_start) >> PAGE_SHIFT, &vmap_lazy_nr);
	if (unlikely(atomic_read(&vmap_lazy_nr) > lazy_max_pages()))
		try_purge_vmap_area_lazy();
}

static void free_unmap_vmap_area_noflush(struct vmap_area *va)
{
	unmap_vmap_area(va);
	free_vmap_area_noflush(va);
}

static void free_unmap_vmap_area(struct vmap_area *va)
{
	flush_cache_vunmap(va->va_start, va->va_end);
	free_unmap_vmap_area_noflush(va);
}

static struct vmap_area *find_vmap_area(unsigned long addr)
{
	struct vmap_area *va;

	spin_lock(&vmap_area_lock);
	va = __find_vmap_area(addr);
	spin_unlock(&vmap_area_lock);

	return va;
}

static void free_unmap_vmap_area_addr(unsigned long addr)
{
	struct vmap_area *va;

	va = find_vmap_area(addr);
	BUG_ON(!va);
	free_unmap_vmap_area(va);
}



#if BITS_PER_LONG == 32
#define VMALLOC_SPACE		(128UL*1024*1024)
#else
#define VMALLOC_SPACE		(128UL*1024*1024*1024)
#endif

#define VMALLOC_PAGES		(VMALLOC_SPACE / PAGE_SIZE)
#define VMAP_MAX_ALLOC		BITS_PER_LONG	
#define VMAP_BBMAP_BITS_MAX	1024	
#define VMAP_BBMAP_BITS_MIN	(VMAP_MAX_ALLOC*2)
#define VMAP_MIN(x, y)		((x) < (y) ? (x) : (y)) 
#define VMAP_MAX(x, y)		((x) > (y) ? (x) : (y)) 
#define VMAP_BBMAP_BITS		\
		VMAP_MIN(VMAP_BBMAP_BITS_MAX,	\
		VMAP_MAX(VMAP_BBMAP_BITS_MIN,	\
			VMALLOC_PAGES / roundup_pow_of_two(NR_CPUS) / 16))

#define VMAP_BLOCK_SIZE		(VMAP_BBMAP_BITS * PAGE_SIZE)

static bool vmap_initialized __read_mostly = false;

struct vmap_block_queue {
	spinlock_t lock;
	struct list_head free;
};

struct vmap_block {
	spinlock_t lock;
	struct vmap_area *va;
	struct vmap_block_queue *vbq;
	unsigned long free, dirty;
	DECLARE_BITMAP(alloc_map, VMAP_BBMAP_BITS);
	DECLARE_BITMAP(dirty_map, VMAP_BBMAP_BITS);
	struct list_head free_list;
	struct rcu_head rcu_head;
	struct list_head purge;
};

static DEFINE_PER_CPU(struct vmap_block_queue, vmap_block_queue);

static DEFINE_SPINLOCK(vmap_block_tree_lock);
static RADIX_TREE(vmap_block_tree, GFP_ATOMIC);


static unsigned long addr_to_vb_idx(unsigned long addr)
{
	addr -= VMALLOC_START & ~(VMAP_BLOCK_SIZE-1);
	addr /= VMAP_BLOCK_SIZE;
	return addr;
}

static struct vmap_block *new_vmap_block(gfp_t gfp_mask)
{
	struct vmap_block_queue *vbq;
	struct vmap_block *vb;
	struct vmap_area *va;
	unsigned long vb_idx;
	int node, err;

	node = numa_node_id();

	vb = kmalloc_node(sizeof(struct vmap_block),
			gfp_mask & GFP_RECLAIM_MASK, node);
	if (unlikely(!vb))
		return ERR_PTR(-ENOMEM);

	va = alloc_vmap_area(VMAP_BLOCK_SIZE, VMAP_BLOCK_SIZE,
					VMALLOC_START, VMALLOC_END,
					node, gfp_mask);
	if (IS_ERR(va)) {
		kfree(vb);
		return ERR_CAST(va);
	}

	err = radix_tree_preload(gfp_mask);
	if (unlikely(err)) {
		kfree(vb);
		free_vmap_area(va);
		return ERR_PTR(err);
	}

	spin_lock_init(&vb->lock);
	vb->va = va;
	vb->free = VMAP_BBMAP_BITS;
	vb->dirty = 0;
	bitmap_zero(vb->alloc_map, VMAP_BBMAP_BITS);
	bitmap_zero(vb->dirty_map, VMAP_BBMAP_BITS);
	INIT_LIST_HEAD(&vb->free_list);

	vb_idx = addr_to_vb_idx(va->va_start);
	spin_lock(&vmap_block_tree_lock);
	err = radix_tree_insert(&vmap_block_tree, vb_idx, vb);
	spin_unlock(&vmap_block_tree_lock);
	BUG_ON(err);
	radix_tree_preload_end();

	vbq = &get_cpu_var(vmap_block_queue);
	vb->vbq = vbq;
	spin_lock(&vbq->lock);
	list_add_rcu(&vb->free_list, &vbq->free);
	spin_unlock(&vbq->lock);
	put_cpu_var(vmap_block_queue);

	return vb;
}

static void free_vmap_block(struct vmap_block *vb)
{
	struct vmap_block *tmp;
	unsigned long vb_idx;

	vb_idx = addr_to_vb_idx(vb->va->va_start);
	spin_lock(&vmap_block_tree_lock);
	tmp = radix_tree_delete(&vmap_block_tree, vb_idx);
	spin_unlock(&vmap_block_tree_lock);
	BUG_ON(tmp != vb);

	free_vmap_area_noflush(vb->va);
	kfree_rcu(vb, rcu_head);
}

static void purge_fragmented_blocks(int cpu)
{
	LIST_HEAD(purge);
	struct vmap_block *vb;
	struct vmap_block *n_vb;
	struct vmap_block_queue *vbq = &per_cpu(vmap_block_queue, cpu);

	rcu_read_lock();
	list_for_each_entry_rcu(vb, &vbq->free, free_list) {

		if (!(vb->free + vb->dirty == VMAP_BBMAP_BITS && vb->dirty != VMAP_BBMAP_BITS))
			continue;

		spin_lock(&vb->lock);
		if (vb->free + vb->dirty == VMAP_BBMAP_BITS && vb->dirty != VMAP_BBMAP_BITS) {
			vb->free = 0; 
			vb->dirty = VMAP_BBMAP_BITS; 
			bitmap_fill(vb->alloc_map, VMAP_BBMAP_BITS);
			bitmap_fill(vb->dirty_map, VMAP_BBMAP_BITS);
			spin_lock(&vbq->lock);
			list_del_rcu(&vb->free_list);
			spin_unlock(&vbq->lock);
			spin_unlock(&vb->lock);
			list_add_tail(&vb->purge, &purge);
		} else
			spin_unlock(&vb->lock);
	}
	rcu_read_unlock();

	list_for_each_entry_safe(vb, n_vb, &purge, purge) {
		list_del(&vb->purge);
		free_vmap_block(vb);
	}
}

static void purge_fragmented_blocks_thiscpu(void)
{
	purge_fragmented_blocks(smp_processor_id());
}

static void purge_fragmented_blocks_allcpus(void)
{
	int cpu;

	for_each_possible_cpu(cpu)
		purge_fragmented_blocks(cpu);
}

static void *vb_alloc(unsigned long size, gfp_t gfp_mask)
{
	struct vmap_block_queue *vbq;
	struct vmap_block *vb;
	unsigned long addr = 0;
	unsigned int order;
	int purge = 0;

	BUG_ON(size & ~PAGE_MASK);
	BUG_ON(size > PAGE_SIZE*VMAP_MAX_ALLOC);
	order = get_order(size);

again:
	rcu_read_lock();
	vbq = &get_cpu_var(vmap_block_queue);
	list_for_each_entry_rcu(vb, &vbq->free, free_list) {
		int i;

		spin_lock(&vb->lock);
		if (vb->free < 1UL << order)
			goto next;

		i = bitmap_find_free_region(vb->alloc_map,
						VMAP_BBMAP_BITS, order);

		if (i < 0) {
			if (vb->free + vb->dirty == VMAP_BBMAP_BITS) {
				
				BUG_ON(vb->dirty != VMAP_BBMAP_BITS);
				purge = 1;
			}
			goto next;
		}
		addr = vb->va->va_start + (i << PAGE_SHIFT);
		BUG_ON(addr_to_vb_idx(addr) !=
				addr_to_vb_idx(vb->va->va_start));
		vb->free -= 1UL << order;
		if (vb->free == 0) {
			spin_lock(&vbq->lock);
			list_del_rcu(&vb->free_list);
			spin_unlock(&vbq->lock);
		}
		spin_unlock(&vb->lock);
		break;
next:
		spin_unlock(&vb->lock);
	}

	if (purge)
		purge_fragmented_blocks_thiscpu();

	put_cpu_var(vmap_block_queue);
	rcu_read_unlock();

	if (!addr) {
		vb = new_vmap_block(gfp_mask);
		if (IS_ERR(vb))
			return vb;
		goto again;
	}

	return (void *)addr;
}

static void vb_free(const void *addr, unsigned long size)
{
	unsigned long offset;
	unsigned long vb_idx;
	unsigned int order;
	struct vmap_block *vb;

	BUG_ON(size & ~PAGE_MASK);
	BUG_ON(size > PAGE_SIZE*VMAP_MAX_ALLOC);

	flush_cache_vunmap((unsigned long)addr, (unsigned long)addr + size);

	order = get_order(size);

	offset = (unsigned long)addr & (VMAP_BLOCK_SIZE - 1);

	vb_idx = addr_to_vb_idx((unsigned long)addr);
	rcu_read_lock();
	vb = radix_tree_lookup(&vmap_block_tree, vb_idx);
	rcu_read_unlock();
	BUG_ON(!vb);

	vunmap_page_range((unsigned long)addr, (unsigned long)addr + size);

	spin_lock(&vb->lock);
	BUG_ON(bitmap_allocate_region(vb->dirty_map, offset >> PAGE_SHIFT, order));

	vb->dirty += 1UL << order;
	if (vb->dirty == VMAP_BBMAP_BITS) {
		BUG_ON(vb->free);
		spin_unlock(&vb->lock);
		free_vmap_block(vb);
	} else
		spin_unlock(&vb->lock);
}

void vm_unmap_aliases(void)
{
	unsigned long start = ULONG_MAX, end = 0;
	int cpu;
	int flush = 0;

	if (unlikely(!vmap_initialized))
		return;

	for_each_possible_cpu(cpu) {
		struct vmap_block_queue *vbq = &per_cpu(vmap_block_queue, cpu);
		struct vmap_block *vb;

		rcu_read_lock();
		list_for_each_entry_rcu(vb, &vbq->free, free_list) {
			int i;

			spin_lock(&vb->lock);
			i = find_first_bit(vb->dirty_map, VMAP_BBMAP_BITS);
			while (i < VMAP_BBMAP_BITS) {
				unsigned long s, e;
				int j;
				j = find_next_zero_bit(vb->dirty_map,
					VMAP_BBMAP_BITS, i);

				s = vb->va->va_start + (i << PAGE_SHIFT);
				e = vb->va->va_start + (j << PAGE_SHIFT);
				flush = 1;

				if (s < start)
					start = s;
				if (e > end)
					end = e;

				i = j;
				i = find_next_bit(vb->dirty_map,
							VMAP_BBMAP_BITS, i);
			}
			spin_unlock(&vb->lock);
		}
		rcu_read_unlock();
	}

	__purge_vmap_area_lazy(&start, &end, 1, flush);
}
EXPORT_SYMBOL_GPL(vm_unmap_aliases);

void vm_unmap_ram(const void *mem, unsigned int count)
{
	unsigned long size = count << PAGE_SHIFT;
	unsigned long addr = (unsigned long)mem;

	BUG_ON(!addr);
	BUG_ON(addr < VMALLOC_START);
	BUG_ON(addr > VMALLOC_END);
	BUG_ON(addr & (PAGE_SIZE-1));

	debug_check_no_locks_freed(mem, size);
	vmap_debug_free_range(addr, addr+size);

	if (likely(count <= VMAP_MAX_ALLOC))
		vb_free(mem, size);
	else
		free_unmap_vmap_area_addr(addr);
}
EXPORT_SYMBOL(vm_unmap_ram);

void *vm_map_ram(struct page **pages, unsigned int count, int node, pgprot_t prot)
{
	unsigned long size = count << PAGE_SHIFT;
	unsigned long addr;
	void *mem;

	if (likely(count <= VMAP_MAX_ALLOC)) {
		mem = vb_alloc(size, GFP_KERNEL);
		if (IS_ERR(mem))
			return NULL;
		addr = (unsigned long)mem;
	} else {
		struct vmap_area *va;
		va = alloc_vmap_area(size, PAGE_SIZE,
				VMALLOC_START, VMALLOC_END, node, GFP_KERNEL);
		if (IS_ERR(va))
			return NULL;

		addr = va->va_start;
		mem = (void *)addr;
	}
	if (vmap_page_range(addr, addr + size, prot, pages) < 0) {
		vm_unmap_ram(mem, count);
		return NULL;
	}
	return mem;
}
EXPORT_SYMBOL(vm_map_ram);
int __init vm_area_check_early(struct vm_struct *vm)
{
	struct vm_struct *tmp, **p;

	BUG_ON(vmap_initialized);
	for (p = &vmlist; (tmp = *p) != NULL; p = &tmp->next) {
		if (tmp->addr >= vm->addr) {
			if (tmp->addr < vm->addr + vm->size)
				return 1;
		} else {
			if (tmp->addr + tmp->size > vm->addr)
				return 1;
		}
	}
	return 0;
}
void __init vm_area_add_early(struct vm_struct *vm)
{
	struct vm_struct *tmp, **p;

	BUG_ON(vmap_initialized);
	for (p = &vmlist; (tmp = *p) != NULL; p = &tmp->next) {
		if (tmp->addr >= vm->addr) {
			BUG_ON(tmp->addr < vm->addr + vm->size);
			break;
		} else
			BUG_ON(tmp->addr + tmp->size > vm->addr);
	}
	vm->next = *p;
	*p = vm;
}

void __init vm_area_register_early(struct vm_struct *vm, size_t align)
{
	static size_t vm_init_off __initdata;
	unsigned long addr;

	addr = ALIGN(VMALLOC_START + vm_init_off, align);
	vm_init_off = PFN_ALIGN(addr + vm->size) - VMALLOC_START;

	vm->addr = (void *)addr;

	vm_area_add_early(vm);
}

void __init vmalloc_init(void)
{
	struct vmap_area *va;
	struct vm_struct *tmp;
	int i;

	for_each_possible_cpu(i) {
		struct vmap_block_queue *vbq;

		vbq = &per_cpu(vmap_block_queue, i);
		spin_lock_init(&vbq->lock);
		INIT_LIST_HEAD(&vbq->free);
	}

	
	for (tmp = vmlist; tmp; tmp = tmp->next) {
		va = kzalloc(sizeof(struct vmap_area), GFP_NOWAIT);
		va->flags = VM_VM_AREA;
		va->va_start = (unsigned long)tmp->addr;
		va->va_end = va->va_start + tmp->size;
		va->vm = tmp;
		__insert_vmap_area(va);
	}

	vmap_area_pcpu_hole = VMALLOC_END;

	vmap_initialized = true;
}

int map_kernel_range_noflush(unsigned long addr, unsigned long size,
			     pgprot_t prot, struct page **pages)
{
	return vmap_page_range_noflush(addr, addr + size, prot, pages);
}

void unmap_kernel_range_noflush(unsigned long addr, unsigned long size)
{
	vunmap_page_range(addr, addr + size);
}
EXPORT_SYMBOL_GPL(unmap_kernel_range_noflush);

void unmap_kernel_range(unsigned long addr, unsigned long size)
{
	unsigned long end = addr + size;

	flush_cache_vunmap(addr, end);
	vunmap_page_range(addr, end);
	flush_tlb_kernel_range(addr, end);
}

int map_vm_area(struct vm_struct *area, pgprot_t prot, struct page ***pages)
{
	unsigned long addr = (unsigned long)area->addr;
	unsigned long end = addr + area->size - PAGE_SIZE;
	int err;

	err = vmap_page_range(addr, end, prot, *pages);
	if (err > 0) {
		*pages += err;
		err = 0;
	}

	return err;
}
EXPORT_SYMBOL_GPL(map_vm_area);

DEFINE_RWLOCK(vmlist_lock);
struct vm_struct *vmlist;

static void setup_vmalloc_vm(struct vm_struct *vm, struct vmap_area *va,
			      unsigned long flags, const void *caller)
{
	vm->flags = flags;
	vm->addr = (void *)va->va_start;
	vm->size = va->va_end - va->va_start;
	vm->caller = caller;
	va->vm = vm;
	va->flags |= VM_VM_AREA;
}

static void insert_vmalloc_vmlist(struct vm_struct *vm)
{
	struct vm_struct *tmp, **p;

	vm->flags &= ~VM_UNLIST;
	write_lock(&vmlist_lock);
	for (p = &vmlist; (tmp = *p) != NULL; p = &tmp->next) {
		if (tmp->addr >= vm->addr)
			break;
	}
	vm->next = *p;
	*p = vm;
	write_unlock(&vmlist_lock);
}

static void insert_vmalloc_vm(struct vm_struct *vm, struct vmap_area *va,
			      unsigned long flags, const void *caller)
{
	setup_vmalloc_vm(vm, va, flags, caller);
	insert_vmalloc_vmlist(vm);
}

static struct vm_struct *__get_vm_area_node(unsigned long size,
		unsigned long align, unsigned long flags, unsigned long start,
		unsigned long end, int node, gfp_t gfp_mask, const void *caller)
{
	struct vmap_area *va;
	struct vm_struct *area;

	BUG_ON(in_interrupt());
	if (flags & VM_IOREMAP) {
		int bit = fls(size);

		if (bit > IOREMAP_MAX_ORDER)
			bit = IOREMAP_MAX_ORDER;
		else if (bit < PAGE_SHIFT)
			bit = PAGE_SHIFT;

		align = 1ul << bit;
	}

	size = PAGE_ALIGN(size);
	if (unlikely(!size))
		return NULL;

	area = kzalloc_node(sizeof(*area), gfp_mask & GFP_RECLAIM_MASK, node);
	if (unlikely(!area))
		return NULL;

	size += PAGE_SIZE;

	va = alloc_vmap_area(size, align, start, end, node, gfp_mask);
	if (IS_ERR(va)) {
		kfree(area);
		return NULL;
	}

	if (flags & VM_UNLIST)
		setup_vmalloc_vm(area, va, flags, caller);
	else
		insert_vmalloc_vm(area, va, flags, caller);

	return area;
}

struct vm_struct *__get_vm_area(unsigned long size, unsigned long flags,
				unsigned long start, unsigned long end)
{
	return __get_vm_area_node(size, 1, flags, start, end, -1, GFP_KERNEL,
						__builtin_return_address(0));
}
EXPORT_SYMBOL_GPL(__get_vm_area);

struct vm_struct *__get_vm_area_caller(unsigned long size, unsigned long flags,
				       unsigned long start, unsigned long end,
				       const void *caller)
{
	return __get_vm_area_node(size, 1, flags, start, end, -1, GFP_KERNEL,
				  caller);
}

struct vm_struct *get_vm_area(unsigned long size, unsigned long flags)
{
#ifdef CONFIG_ENABLE_VMALLOC_SAVING
	return __get_vm_area_node(size, 1, flags, PAGE_OFFSET, VMALLOC_END,
				-1, GFP_KERNEL, __builtin_return_address(0));
#else
	return __get_vm_area_node(size, 1, flags, VMALLOC_START, VMALLOC_END,
				-1, GFP_KERNEL, __builtin_return_address(0));
#endif

}

struct vm_struct *get_vm_area_caller(unsigned long size, unsigned long flags,
				const void *caller)
{
#ifdef CONFIG_ENABLE_VMALLOC_SAVING
	return __get_vm_area_node(size, 1, flags, PAGE_OFFSET, VMALLOC_END,
						-1, GFP_KERNEL, caller);
#else
	return __get_vm_area_node(size, 1, flags, VMALLOC_START, VMALLOC_END,
				-1, GFP_KERNEL, __builtin_return_address(0));
#endif
}

struct vm_struct *find_vm_area(const void *addr)
{
	struct vmap_area *va;

	va = find_vmap_area((unsigned long)addr);
	if (va && va->flags & VM_VM_AREA)
		return va->vm;

	return NULL;
}

struct vm_struct *remove_vm_area(const void *addr)
{
	struct vmap_area *va;

	va = find_vmap_area((unsigned long)addr);
	if (va && va->flags & VM_VM_AREA) {
		struct vm_struct *vm = va->vm;

		if (!(vm->flags & VM_UNLIST)) {
			struct vm_struct *tmp, **p;
			write_lock(&vmlist_lock);
			for (p = &vmlist; (tmp = *p) != vm; p = &tmp->next)
				;
			*p = tmp->next;
			write_unlock(&vmlist_lock);
		}

		vmap_debug_free_range(va->va_start, va->va_end);
		free_unmap_vmap_area(va);
		vm->size -= PAGE_SIZE;

		return vm;
	}
	return NULL;
}

static void __vunmap(const void *addr, int deallocate_pages)
{
	struct vm_struct *area;

	if (!addr)
		return;

	if ((PAGE_SIZE-1) & (unsigned long)addr) {
		WARN(1, KERN_ERR "Trying to vfree() bad address (%p)\n", addr);
		return;
	}

	area = remove_vm_area(addr);
	if (unlikely(!area)) {
		WARN(1, KERN_ERR "Trying to vfree() nonexistent vm area (%p)\n",
				addr);
		return;
	}

	debug_check_no_locks_freed(addr, area->size);
	debug_check_no_obj_freed(addr, area->size);

	if (deallocate_pages) {
		int i;

		for (i = 0; i < area->nr_pages; i++) {
			struct page *page = area->pages[i];

			BUG_ON(!page);
			__free_page(page);
		}

		sub_meminfo_total_pages(NR_VMALLOC_PAGES, area->nr_pages);

		if (area->flags & VM_VPAGES)
			vfree(area->pages);
		else
			kfree(area->pages);
	}

	kfree(area);
	return;
}

void vfree(const void *addr)
{
	BUG_ON(in_interrupt());

	kmemleak_free(addr);

	__vunmap(addr, 1);
}
EXPORT_SYMBOL(vfree);

void vunmap(const void *addr)
{
	BUG_ON(in_interrupt());
	might_sleep();
	__vunmap(addr, 0);
}
EXPORT_SYMBOL(vunmap);

void *vmap(struct page **pages, unsigned int count,
		unsigned long flags, pgprot_t prot)
{
	struct vm_struct *area;

	might_sleep();

	if (count > totalram_pages)
		return NULL;

	area = get_vm_area_caller((count << PAGE_SHIFT), flags,
					__builtin_return_address(0));
	if (!area)
		return NULL;

	if (map_vm_area(area, prot, &pages)) {
		vunmap(area->addr);
		return NULL;
	}

	return area->addr;
}
EXPORT_SYMBOL(vmap);

static void *__vmalloc_node(unsigned long size, unsigned long align,
			    gfp_t gfp_mask, pgprot_t prot,
			    int node, const void *caller);
static void *__vmalloc_area_node(struct vm_struct *area, gfp_t gfp_mask,
				 pgprot_t prot, int node, const void *caller)
{
	const int order = 0;
	struct page **pages;
	unsigned int nr_pages, array_size, i;
	gfp_t nested_gfp = (gfp_mask & GFP_RECLAIM_MASK) | __GFP_ZERO;

	nr_pages = (area->size - PAGE_SIZE) >> PAGE_SHIFT;
	array_size = (nr_pages * sizeof(struct page *));

	area->nr_pages = nr_pages;
	
	if (array_size > PAGE_SIZE) {
		pages = __vmalloc_node(array_size, 1, nested_gfp|__GFP_HIGHMEM,
				PAGE_KERNEL, node, caller);
		area->flags |= VM_VPAGES;
	} else {
		pages = kmalloc_node(array_size, nested_gfp, node);
	}
	area->pages = pages;
	area->caller = caller;
	if (!area->pages) {
		remove_vm_area(area->addr);
		kfree(area);
		return NULL;
	}

	for (i = 0; i < area->nr_pages; i++) {
		struct page *page;
		gfp_t tmp_mask = gfp_mask | __GFP_NOWARN;

		if (node < 0)
			page = alloc_page(tmp_mask);
		else
			page = alloc_pages_node(node, tmp_mask, order);

		if (unlikely(!page)) {
			
			area->nr_pages = i;
			goto fail;
		}
		area->pages[i] = page;
	}

	if (map_vm_area(area, prot, &pages))
		goto fail;
	return area->addr;

fail:
	warn_alloc_failed(gfp_mask, order,
			  "vmalloc: allocation failure, allocated %ld of %ld bytes\n",
			  (area->nr_pages*PAGE_SIZE), area->size);
	vfree(area->addr);
	return NULL;
}

void *__vmalloc_node_range(unsigned long size, unsigned long align,
			unsigned long start, unsigned long end, gfp_t gfp_mask,
			pgprot_t prot, int node, const void *caller)
{
	struct vm_struct *area;
	void *addr;
	unsigned long real_size = size;
#ifdef CONFIG_FIX_MOVABLE_ZONE
	unsigned long total_pages = total_unmovable_pages;
#else
	unsigned long total_pages = totalram_pages;
#endif

	size = PAGE_ALIGN(size);
	if (!size || (size >> PAGE_SHIFT) > total_pages)
		goto fail;

	area = __get_vm_area_node(size, align, VM_ALLOC | VM_UNLIST,
				  start, end, node, gfp_mask, caller);
	if (!area)
		goto fail;

	addr = __vmalloc_area_node(area, gfp_mask, prot, node, caller);
	if (!addr)
		return NULL;

	add_meminfo_total_pages(NR_VMALLOC_PAGES, area->nr_pages);

	insert_vmalloc_vmlist(area);

	kmemleak_alloc(addr, real_size, 3, gfp_mask);

	return addr;

fail:
	warn_alloc_failed(gfp_mask, 0,
			  "vmalloc: allocation failure: %lu bytes\n",
			  real_size);
	return NULL;
}

static void *__vmalloc_node(unsigned long size, unsigned long align,
			    gfp_t gfp_mask, pgprot_t prot,
			    int node, const void *caller)
{
	return __vmalloc_node_range(size, align, VMALLOC_START, VMALLOC_END,
				gfp_mask, prot, node, caller);
}

void *__vmalloc(unsigned long size, gfp_t gfp_mask, pgprot_t prot)
{
	return __vmalloc_node(size, 1, gfp_mask, prot, -1,
				__builtin_return_address(0));
}
EXPORT_SYMBOL(__vmalloc);

static inline void *__vmalloc_node_flags(unsigned long size,
					int node, gfp_t flags)
{
	return __vmalloc_node(size, 1, flags, PAGE_KERNEL,
					node, __builtin_return_address(0));
}

void *vmalloc(unsigned long size)
{
	return __vmalloc_node_flags(size, -1, GFP_KERNEL | __GFP_HIGHMEM);
}
EXPORT_SYMBOL(vmalloc);

void *vzalloc(unsigned long size)
{
	return __vmalloc_node_flags(size, -1,
				GFP_KERNEL | __GFP_HIGHMEM | __GFP_ZERO);
}
EXPORT_SYMBOL(vzalloc);

void *vmalloc_user(unsigned long size)
{
	struct vm_struct *area;
	void *ret;

	ret = __vmalloc_node(size, SHMLBA,
			     GFP_KERNEL | __GFP_HIGHMEM | __GFP_ZERO,
			     PAGE_KERNEL, -1, __builtin_return_address(0));
	if (ret) {
		area = find_vm_area(ret);
		area->flags |= VM_USERMAP;
	}
	return ret;
}
EXPORT_SYMBOL(vmalloc_user);

void *vmalloc_node(unsigned long size, int node)
{
	return __vmalloc_node(size, 1, GFP_KERNEL | __GFP_HIGHMEM, PAGE_KERNEL,
					node, __builtin_return_address(0));
}
EXPORT_SYMBOL(vmalloc_node);

void *vzalloc_node(unsigned long size, int node)
{
	return __vmalloc_node_flags(size, node,
			 GFP_KERNEL | __GFP_HIGHMEM | __GFP_ZERO);
}
EXPORT_SYMBOL(vzalloc_node);

#ifndef PAGE_KERNEL_EXEC
# define PAGE_KERNEL_EXEC PAGE_KERNEL
#endif


void *vmalloc_exec(unsigned long size)
{
	return __vmalloc_node(size, 1, GFP_KERNEL | __GFP_HIGHMEM, PAGE_KERNEL_EXEC,
			      -1, __builtin_return_address(0));
}

#if defined(CONFIG_64BIT) && defined(CONFIG_ZONE_DMA32)
#define GFP_VMALLOC32 GFP_DMA32 | GFP_KERNEL
#elif defined(CONFIG_64BIT) && defined(CONFIG_ZONE_DMA)
#define GFP_VMALLOC32 GFP_DMA | GFP_KERNEL
#else
#define GFP_VMALLOC32 GFP_KERNEL
#endif

void *vmalloc_32(unsigned long size)
{
	return __vmalloc_node(size, 1, GFP_VMALLOC32, PAGE_KERNEL,
			      -1, __builtin_return_address(0));
}
EXPORT_SYMBOL(vmalloc_32);

void *vmalloc_32_user(unsigned long size)
{
	struct vm_struct *area;
	void *ret;

	ret = __vmalloc_node(size, 1, GFP_VMALLOC32 | __GFP_ZERO, PAGE_KERNEL,
			     -1, __builtin_return_address(0));
	if (ret) {
		area = find_vm_area(ret);
		area->flags |= VM_USERMAP;
	}
	return ret;
}
EXPORT_SYMBOL(vmalloc_32_user);


static int aligned_vread(char *buf, char *addr, unsigned long count)
{
	struct page *p;
	int copied = 0;

	while (count) {
		unsigned long offset, length;

		offset = (unsigned long)addr & ~PAGE_MASK;
		length = PAGE_SIZE - offset;
		if (length > count)
			length = count;
		p = vmalloc_to_page(addr);
		if (p) {
			void *map = kmap_atomic(p);
			memcpy(buf, map + offset, length);
			kunmap_atomic(map);
		} else
			memset(buf, 0, length);

		addr += length;
		buf += length;
		copied += length;
		count -= length;
	}
	return copied;
}

static int aligned_vwrite(char *buf, char *addr, unsigned long count)
{
	struct page *p;
	int copied = 0;

	while (count) {
		unsigned long offset, length;

		offset = (unsigned long)addr & ~PAGE_MASK;
		length = PAGE_SIZE - offset;
		if (length > count)
			length = count;
		p = vmalloc_to_page(addr);
		if (p) {
			void *map = kmap_atomic(p);
			memcpy(map + offset, buf, length);
			kunmap_atomic(map);
		}
		addr += length;
		buf += length;
		copied += length;
		count -= length;
	}
	return copied;
}


long vread(char *buf, char *addr, unsigned long count)
{
	struct vm_struct *tmp;
	char *vaddr, *buf_start = buf;
	unsigned long buflen = count;
	unsigned long n;

	
	if ((unsigned long) addr + count < count)
		count = -(unsigned long) addr;

	read_lock(&vmlist_lock);
	for (tmp = vmlist; count && tmp; tmp = tmp->next) {
		vaddr = (char *) tmp->addr;
		if (addr >= vaddr + tmp->size - PAGE_SIZE)
			continue;
		while (addr < vaddr) {
			if (count == 0)
				goto finished;
			*buf = '\0';
			buf++;
			addr++;
			count--;
		}
		n = vaddr + tmp->size - PAGE_SIZE - addr;
		if (n > count)
			n = count;
		if (!(tmp->flags & VM_IOREMAP))
			aligned_vread(buf, addr, n);
		else 
			memset(buf, 0, n);
		buf += n;
		addr += n;
		count -= n;
	}
finished:
	read_unlock(&vmlist_lock);

	if (buf == buf_start)
		return 0;
	
	if (buf != buf_start + buflen)
		memset(buf, 0, buflen - (buf - buf_start));

	return buflen;
}


long vwrite(char *buf, char *addr, unsigned long count)
{
	struct vm_struct *tmp;
	char *vaddr;
	unsigned long n, buflen;
	int copied = 0;

	
	if ((unsigned long) addr + count < count)
		count = -(unsigned long) addr;
	buflen = count;

	read_lock(&vmlist_lock);
	for (tmp = vmlist; count && tmp; tmp = tmp->next) {
		vaddr = (char *) tmp->addr;
		if (addr >= vaddr + tmp->size - PAGE_SIZE)
			continue;
		while (addr < vaddr) {
			if (count == 0)
				goto finished;
			buf++;
			addr++;
			count--;
		}
		n = vaddr + tmp->size - PAGE_SIZE - addr;
		if (n > count)
			n = count;
		if (!(tmp->flags & VM_IOREMAP)) {
			aligned_vwrite(buf, addr, n);
			copied++;
		}
		buf += n;
		addr += n;
		count -= n;
	}
finished:
	read_unlock(&vmlist_lock);
	if (!copied)
		return 0;
	return buflen;
}

int remap_vmalloc_range(struct vm_area_struct *vma, void *addr,
						unsigned long pgoff)
{
	struct vm_struct *area;
	unsigned long uaddr = vma->vm_start;
	unsigned long usize = vma->vm_end - vma->vm_start;

	if ((PAGE_SIZE-1) & (unsigned long)addr)
		return -EINVAL;

	area = find_vm_area(addr);
	if (!area)
		return -EINVAL;

	if (!(area->flags & VM_USERMAP))
		return -EINVAL;

	if (usize + (pgoff << PAGE_SHIFT) > area->size - PAGE_SIZE)
		return -EINVAL;

	addr += pgoff << PAGE_SHIFT;
	do {
		struct page *page = vmalloc_to_page(addr);
		int ret;

		ret = vm_insert_page(vma, uaddr, page);
		if (ret)
			return ret;

		uaddr += PAGE_SIZE;
		addr += PAGE_SIZE;
		usize -= PAGE_SIZE;
	} while (usize > 0);

	
	vma->vm_flags |= VM_RESERVED;

	return 0;
}
EXPORT_SYMBOL(remap_vmalloc_range);

void  __attribute__((weak)) vmalloc_sync_all(void)
{
}


static int f(pte_t *pte, pgtable_t table, unsigned long addr, void *data)
{
	pte_t ***p = data;

	if (p) {
		*(*p) = pte;
		(*p)++;
	}
	return 0;
}

struct vm_struct *alloc_vm_area(size_t size, pte_t **ptes)
{
	struct vm_struct *area;

	area = get_vm_area_caller(size, VM_IOREMAP,
				__builtin_return_address(0));
	if (area == NULL)
		return NULL;

	if (apply_to_page_range(&init_mm, (unsigned long)area->addr,
				size, f, ptes ? &ptes : NULL)) {
		free_vm_area(area);
		return NULL;
	}

	vmalloc_sync_all();

	return area;
}
EXPORT_SYMBOL_GPL(alloc_vm_area);

void free_vm_area(struct vm_struct *area)
{
	struct vm_struct *ret;
	ret = remove_vm_area(area->addr);
	BUG_ON(ret != area);
	kfree(area);
}
EXPORT_SYMBOL_GPL(free_vm_area);

#ifdef CONFIG_SMP
static struct vmap_area *node_to_va(struct rb_node *n)
{
	return n ? rb_entry(n, struct vmap_area, rb_node) : NULL;
}

static bool pvm_find_next_prev(unsigned long end,
			       struct vmap_area **pnext,
			       struct vmap_area **pprev)
{
	struct rb_node *n = vmap_area_root.rb_node;
	struct vmap_area *va = NULL;

	while (n) {
		va = rb_entry(n, struct vmap_area, rb_node);
		if (end < va->va_end)
			n = n->rb_left;
		else if (end > va->va_end)
			n = n->rb_right;
		else
			break;
	}

	if (!va)
		return false;

	if (va->va_end > end) {
		*pnext = va;
		*pprev = node_to_va(rb_prev(&(*pnext)->rb_node));
	} else {
		*pprev = va;
		*pnext = node_to_va(rb_next(&(*pprev)->rb_node));
	}
	return true;
}

static unsigned long pvm_determine_end(struct vmap_area **pnext,
				       struct vmap_area **pprev,
				       unsigned long align)
{
	const unsigned long vmalloc_end = VMALLOC_END & ~(align - 1);
	unsigned long addr;

	if (*pnext)
		addr = min((*pnext)->va_start & ~(align - 1), vmalloc_end);
	else
		addr = vmalloc_end;

	while (*pprev && (*pprev)->va_end > addr) {
		*pnext = *pprev;
		*pprev = node_to_va(rb_prev(&(*pnext)->rb_node));
	}

	return addr;
}

struct vm_struct **pcpu_get_vm_areas(const unsigned long *offsets,
				     const size_t *sizes, int nr_vms,
				     size_t align)
{
	const unsigned long vmalloc_start = ALIGN(VMALLOC_START, align);
	const unsigned long vmalloc_end = VMALLOC_END & ~(align - 1);
	struct vmap_area **vas, *prev, *next;
	struct vm_struct **vms;
	int area, area2, last_area, term_area;
	unsigned long base, start, end, last_end;
	bool purged = false;

	
	BUG_ON(align & ~PAGE_MASK || !is_power_of_2(align));
	for (last_area = 0, area = 0; area < nr_vms; area++) {
		start = offsets[area];
		end = start + sizes[area];

		
		BUG_ON(!IS_ALIGNED(offsets[area], align));
		BUG_ON(!IS_ALIGNED(sizes[area], align));

		
		if (start > offsets[last_area])
			last_area = area;

		for (area2 = 0; area2 < nr_vms; area2++) {
			unsigned long start2 = offsets[area2];
			unsigned long end2 = start2 + sizes[area2];

			if (area2 == area)
				continue;

			BUG_ON(start2 >= start && start2 < end);
			BUG_ON(end2 <= end && end2 > start);
		}
	}
	last_end = offsets[last_area] + sizes[last_area];

	if (vmalloc_end - vmalloc_start < last_end) {
		WARN_ON(true);
		return NULL;
	}

	vms = kzalloc(sizeof(vms[0]) * nr_vms, GFP_KERNEL);
	vas = kzalloc(sizeof(vas[0]) * nr_vms, GFP_KERNEL);
	if (!vas || !vms)
		goto err_free2;

	for (area = 0; area < nr_vms; area++) {
		vas[area] = kzalloc(sizeof(struct vmap_area), GFP_KERNEL);
		vms[area] = kzalloc(sizeof(struct vm_struct), GFP_KERNEL);
		if (!vas[area] || !vms[area])
			goto err_free;
	}
retry:
	spin_lock(&vmap_area_lock);

	
	area = term_area = last_area;
	start = offsets[area];
	end = start + sizes[area];

	if (!pvm_find_next_prev(vmap_area_pcpu_hole, &next, &prev)) {
		base = vmalloc_end - last_end;
		goto found;
	}
	base = pvm_determine_end(&next, &prev, align) - end;

	while (true) {
		BUG_ON(next && next->va_end <= base + end);
		BUG_ON(prev && prev->va_end > base + end);

		if (base + last_end < vmalloc_start + last_end) {
			spin_unlock(&vmap_area_lock);
			if (!purged) {
				purge_vmap_area_lazy();
				purged = true;
				goto retry;
			}
			goto err_free;
		}

		if (next && next->va_start < base + end) {
			base = pvm_determine_end(&next, &prev, align) - end;
			term_area = area;
			continue;
		}

		if (prev && prev->va_end > base + start)  {
			next = prev;
			prev = node_to_va(rb_prev(&next->rb_node));
			base = pvm_determine_end(&next, &prev, align) - end;
			term_area = area;
			continue;
		}

		area = (area + nr_vms - 1) % nr_vms;
		if (area == term_area)
			break;
		start = offsets[area];
		end = start + sizes[area];
		pvm_find_next_prev(base + end, &next, &prev);
	}
found:
	
	for (area = 0; area < nr_vms; area++) {
		struct vmap_area *va = vas[area];

		va->va_start = base + offsets[area];
		va->va_end = va->va_start + sizes[area];
		__insert_vmap_area(va);
	}

	vmap_area_pcpu_hole = base + offsets[last_area];

	spin_unlock(&vmap_area_lock);

	
	for (area = 0; area < nr_vms; area++)
		insert_vmalloc_vm(vms[area], vas[area], VM_ALLOC,
				  pcpu_get_vm_areas);

	kfree(vas);
	return vms;

err_free:
	for (area = 0; area < nr_vms; area++) {
		kfree(vas[area]);
		kfree(vms[area]);
	}
err_free2:
	kfree(vas);
	kfree(vms);
	return NULL;
}

void pcpu_free_vm_areas(struct vm_struct **vms, int nr_vms)
{
	int i;

	for (i = 0; i < nr_vms; i++)
		free_vm_area(vms[i]);
	kfree(vms);
}
#endif	

#ifdef CONFIG_PROC_FS
static void *s_start(struct seq_file *m, loff_t *pos)
	__acquires(&vmlist_lock)
{
	loff_t n = *pos;
	struct vm_struct *v;

	read_lock(&vmlist_lock);
	v = vmlist;
	while (n > 0 && v) {
		n--;
		v = v->next;
	}
	if (!n)
		return v;

	return NULL;

}

static void *s_next(struct seq_file *m, void *p, loff_t *pos)
{
	struct vm_struct *v = p;

	++*pos;
	return v->next;
}

static void s_stop(struct seq_file *m, void *p)
	__releases(&vmlist_lock)
{
	read_unlock(&vmlist_lock);
}

static void show_numa_info(struct seq_file *m, struct vm_struct *v)
{
	if (NUMA_BUILD) {
		unsigned int nr, *counters = m->private;

		if (!counters)
			return;

		memset(counters, 0, nr_node_ids * sizeof(unsigned int));

		for (nr = 0; nr < v->nr_pages; nr++)
			counters[page_to_nid(v->pages[nr])]++;

		for_each_node_state(nr, N_HIGH_MEMORY)
			if (counters[nr])
				seq_printf(m, " N%u=%u", nr, counters[nr]);
	}
}

static int s_show(struct seq_file *m, void *p)
{
	struct vm_struct *v = p;

	seq_printf(m, "0x%p-0x%p %7ld",
		v->addr, v->addr + v->size, v->size);

	if (v->caller)
		seq_printf(m, " %pS", v->caller);

	if (v->nr_pages)
		seq_printf(m, " pages=%d", v->nr_pages);

	if (v->phys_addr)
		seq_printf(m, " phys=%llx", (unsigned long long)v->phys_addr);

	if (v->flags & VM_IOREMAP)
		seq_printf(m, " ioremap");

	if (v->flags & VM_ALLOC)
		seq_printf(m, " vmalloc");

	if (v->flags & VM_MAP)
		seq_printf(m, " vmap");

	if (v->flags & VM_USERMAP)
		seq_printf(m, " user");

	if (v->flags & VM_VPAGES)
		seq_printf(m, " vpages");

	if (v->flags & VM_LOWMEM)
		seq_printf(m, " lowmem");

	show_numa_info(m, v);
	seq_putc(m, '\n');
	return 0;
}

static const struct seq_operations vmalloc_op = {
	.start = s_start,
	.next = s_next,
	.stop = s_stop,
	.show = s_show,
};

static int vmalloc_open(struct inode *inode, struct file *file)
{
	unsigned int *ptr = NULL;
	int ret;

	if (NUMA_BUILD) {
		ptr = kmalloc(nr_node_ids * sizeof(unsigned int), GFP_KERNEL);
		if (ptr == NULL)
			return -ENOMEM;
	}
	ret = seq_open(file, &vmalloc_op);
	if (!ret) {
		struct seq_file *m = file->private_data;
		m->private = ptr;
	} else
		kfree(ptr);
	return ret;
}

static const struct file_operations proc_vmalloc_operations = {
	.open		= vmalloc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release_private,
};

static int __init proc_vmalloc_init(void)
{
	proc_create("vmallocinfo", S_IRUSR, NULL, &proc_vmalloc_operations);
	return 0;
}
module_init(proc_vmalloc_init);

void get_vmalloc_info(struct vmalloc_info *vmi)
{
	struct vm_struct *vma;
	unsigned long free_area_size;
	unsigned long prev_end;

	vmi->used = 0;
	vmi->ioremap = 0;
	vmi->alloc = 0;
	vmi->map = 0;
	vmi->usermap = 0;
	vmi->vpages = 0;

	if (!vmlist) {
		vmi->largest_chunk = VMALLOC_TOTAL;
	}
	else {
		vmi->largest_chunk = 0;

		prev_end = VMALLOC_START;

		read_lock(&vmlist_lock);

		for (vma = vmlist; vma; vma = vma->next) {
			unsigned long addr = (unsigned long) vma->addr;

			if (addr < VMALLOC_START)
				continue;
			if (addr >= VMALLOC_END)
				break;

			vmi->used += vma->size;
			if (vma->flags & VM_IOREMAP)
				vmi->ioremap += vma->size;
			if (vma->flags & VM_ALLOC)
				vmi->alloc += vma->size;
			if (vma->flags & VM_MAP)
				vmi->map += vma->size;
			if (vma->flags & VM_USERMAP)
				vmi->usermap += vma->size;
			if (vma->flags & VM_VPAGES)
				vmi->vpages += vma->size;

			free_area_size = addr - prev_end;
			if (vmi->largest_chunk < free_area_size)
				vmi->largest_chunk = free_area_size;

			prev_end = vma->size + addr;
		}

		if (VMALLOC_END - prev_end > vmi->largest_chunk)
			vmi->largest_chunk = VMALLOC_END - prev_end;

		read_unlock(&vmlist_lock);
	}
}

#endif

