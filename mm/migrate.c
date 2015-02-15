/*
 * Memory Migration functionality - linux/mm/migration.c
 *
 * Copyright (C) 2006 Silicon Graphics, Inc., Christoph Lameter
 *
 * Page migration was first developed in the context of the memory hotplug
 * project. The main authors of the migration code are:
 *
 * IWAMOTO Toshihiro <iwamoto@valinux.co.jp>
 * Hirokazu Takahashi <taka@valinux.co.jp>
 * Dave Hansen <haveblue@us.ibm.com>
 * Christoph Lameter
 */

#include <linux/migrate.h>
#include <linux/export.h>
#include <linux/swap.h>
#include <linux/swapops.h>
#include <linux/pagemap.h>
#include <linux/buffer_head.h>
#include <linux/mm_inline.h>
#include <linux/nsproxy.h>
#include <linux/pagevec.h>
#include <linux/ksm.h>
#include <linux/rmap.h>
#include <linux/topology.h>
#include <linux/cpu.h>
#include <linux/cpuset.h>
#include <linux/writeback.h>
#include <linux/mempolicy.h>
#include <linux/vmalloc.h>
#include <linux/security.h>
#include <linux/memcontrol.h>
#include <linux/syscalls.h>
#include <linux/hugetlb.h>
#include <linux/gfp.h>
#include <trace/events/kmem.h>

#include <asm/tlbflush.h>

#include "internal.h"

int migrate_prep(void)
{
	lru_add_drain_all();

	return 0;
}

int migrate_prep_local(void)
{
	lru_add_drain();

	return 0;
}

void putback_lru_pages(struct list_head *l)
{
	struct page *page;
	struct page *page2;

	list_for_each_entry_safe(page, page2, l, lru) {
		list_del(&page->lru);
		dec_zone_page_state(page, NR_ISOLATED_ANON +
				page_is_file_cache(page));
		putback_lru_page(page);
	}
}

static int remove_migration_pte(struct page *new, struct vm_area_struct *vma,
				 unsigned long addr, void *old)
{
	struct mm_struct *mm = vma->vm_mm;
	swp_entry_t entry;
 	pgd_t *pgd;
 	pud_t *pud;
 	pmd_t *pmd;
	pte_t *ptep, pte;
 	spinlock_t *ptl;

	if (unlikely(PageHuge(new))) {
		ptep = huge_pte_offset(mm, addr);
		if (!ptep)
			goto out;
		ptl = &mm->page_table_lock;
	} else {
		pgd = pgd_offset(mm, addr);
		if (!pgd_present(*pgd))
			goto out;

		pud = pud_offset(pgd, addr);
		if (!pud_present(*pud))
			goto out;

		pmd = pmd_offset(pud, addr);
		if (pmd_trans_huge(*pmd))
			goto out;
		if (!pmd_present(*pmd))
			goto out;

		ptep = pte_offset_map(pmd, addr);


		ptl = pte_lockptr(mm, pmd);
	}

 	spin_lock(ptl);
	pte = *ptep;
	if (!is_swap_pte(pte))
		goto unlock;

	entry = pte_to_swp_entry(pte);

	if (!is_migration_entry(entry) ||
	    migration_entry_to_page(entry) != old)
		goto unlock;

	get_page(new);
	pte = pte_mkold(mk_pte(new, vma->vm_page_prot));
	if (is_write_migration_entry(entry))
		pte = pte_mkwrite(pte);
#ifdef CONFIG_HUGETLB_PAGE
	if (PageHuge(new))
		pte = pte_mkhuge(pte);
#endif
	flush_cache_page(vma, addr, pte_pfn(pte));
	set_pte_at(mm, addr, ptep, pte);

	if (PageHuge(new)) {
		if (PageAnon(new))
			hugepage_add_anon_rmap(new, vma, addr);
		else
			page_dup_rmap(new);
	} else if (PageAnon(new))
		page_add_anon_rmap(new, vma, addr);
	else
		page_add_file_rmap(new);

	
	update_mmu_cache(vma, addr, ptep);
unlock:
	pte_unmap_unlock(ptep, ptl);
out:
	return SWAP_AGAIN;
}

static void remove_migration_ptes(struct page *old, struct page *new)
{
	rmap_walk(new, remove_migration_pte, old);
}

void migration_entry_wait(struct mm_struct *mm, pmd_t *pmd,
				unsigned long address)
{
	pte_t *ptep, pte;
	spinlock_t *ptl;
	swp_entry_t entry;
	struct page *page;

	ptep = pte_offset_map_lock(mm, pmd, address, &ptl);
	pte = *ptep;
	if (!is_swap_pte(pte))
		goto out;

	entry = pte_to_swp_entry(pte);
	if (!is_migration_entry(entry))
		goto out;

	page = migration_entry_to_page(entry);

	if (!get_page_unless_zero(page))
		goto out;
	pte_unmap_unlock(ptep, ptl);
	wait_on_page_locked(page);
	put_page(page);
	return;
out:
	pte_unmap_unlock(ptep, ptl);
}

#ifdef CONFIG_BLOCK
static bool buffer_migrate_lock_buffers(struct buffer_head *head,
							enum migrate_mode mode)
{
	struct buffer_head *bh = head;

	
	if (mode != MIGRATE_ASYNC) {
		do {
			get_bh(bh);
			lock_buffer(bh);
			bh = bh->b_this_page;

		} while (bh != head);

		return true;
	}

	
	do {
		get_bh(bh);
		if (!trylock_buffer(bh)) {
			struct buffer_head *failed_bh = bh;
			put_bh(failed_bh);
			bh = head;
			while (bh != failed_bh) {
				unlock_buffer(bh);
				put_bh(bh);
				bh = bh->b_this_page;
			}
			return false;
		}

		bh = bh->b_this_page;
	} while (bh != head);
	return true;
}
#else
static inline bool buffer_migrate_lock_buffers(struct buffer_head *head,
							enum migrate_mode mode)
{
	return true;
}
#endif 

static int migrate_page_move_mapping(struct address_space *mapping,
		struct page *newpage, struct page *page,
		struct buffer_head *head, enum migrate_mode mode)
{
	int expected_count;
	void **pslot;

	if (!mapping) {
		
		if (page_count(page) != 1)
			return -EAGAIN;
		return 0;
	}

	spin_lock_irq(&mapping->tree_lock);

	pslot = radix_tree_lookup_slot(&mapping->page_tree,
 					page_index(page));

	expected_count = 2 + page_has_private(page);
	if (page_count(page) != expected_count ||
		radix_tree_deref_slot_protected(pslot, &mapping->tree_lock) != page) {
		spin_unlock_irq(&mapping->tree_lock);
		return -EAGAIN;
	}

	if (!page_freeze_refs(page, expected_count)) {
		spin_unlock_irq(&mapping->tree_lock);
		return -EAGAIN;
	}

	if (mode == MIGRATE_ASYNC && head &&
			!buffer_migrate_lock_buffers(head, mode)) {
		page_unfreeze_refs(page, expected_count);
		spin_unlock_irq(&mapping->tree_lock);
		return -EAGAIN;
	}

	get_page(newpage);	
	if (PageSwapCache(page)) {
		SetPageSwapCache(newpage);
		set_page_private(newpage, page_private(page));
	}

	radix_tree_replace_slot(pslot, newpage);

	page_unfreeze_refs(page, expected_count - 1);

	__dec_zone_page_state(page, NR_FILE_PAGES);
	__inc_zone_page_state(newpage, NR_FILE_PAGES);
	if (!PageSwapCache(page) && PageSwapBacked(page)) {
		__dec_zone_page_state(page, NR_SHMEM);
		__inc_zone_page_state(newpage, NR_SHMEM);
	}
	spin_unlock_irq(&mapping->tree_lock);

	return 0;
}

int migrate_huge_page_move_mapping(struct address_space *mapping,
				   struct page *newpage, struct page *page)
{
	int expected_count;
	void **pslot;

	if (!mapping) {
		if (page_count(page) != 1)
			return -EAGAIN;
		return 0;
	}

	spin_lock_irq(&mapping->tree_lock);

	pslot = radix_tree_lookup_slot(&mapping->page_tree,
					page_index(page));

	expected_count = 2 + page_has_private(page);
	if (page_count(page) != expected_count ||
		radix_tree_deref_slot_protected(pslot, &mapping->tree_lock) != page) {
		spin_unlock_irq(&mapping->tree_lock);
		return -EAGAIN;
	}

	if (!page_freeze_refs(page, expected_count)) {
		spin_unlock_irq(&mapping->tree_lock);
		return -EAGAIN;
	}

	get_page(newpage);

	radix_tree_replace_slot(pslot, newpage);

	page_unfreeze_refs(page, expected_count - 1);

	spin_unlock_irq(&mapping->tree_lock);
	return 0;
}

void migrate_page_copy(struct page *newpage, struct page *page)
{
#ifdef CONFIG_HTC_DEBUG_PAGE_USER_TRACE
	memcpy(&newpage->trace_alloc, &page->trace_alloc, sizeof(page->trace_alloc));
	memcpy(&newpage->trace_free, &page->trace_free, sizeof(page->trace_free));
#endif

	if (PageHuge(page))
		copy_huge_page(newpage, page);
	else
		copy_highpage(newpage, page);

	if (PageError(page))
		SetPageError(newpage);
	if (PageReferenced(page))
		SetPageReferenced(newpage);
	if (PageUptodate(page))
		SetPageUptodate(newpage);
	if (TestClearPageActive(page)) {
		VM_BUG_ON(PageUnevictable(page));
		SetPageActive(newpage);
	} else if (TestClearPageUnevictable(page))
		SetPageUnevictable(newpage);
	if (PageChecked(page))
		SetPageChecked(newpage);
	if (PageMappedToDisk(page))
		SetPageMappedToDisk(newpage);

	if (PageDirty(page)) {
		clear_page_dirty_for_io(page);
		__set_page_dirty_nobuffers(newpage);
 	}

	mlock_migrate_page(newpage, page);
	ksm_migrate_page(newpage, page);

	ClearPageSwapCache(page);
	ClearPagePrivate(page);
	set_page_private(page, 0);

	if (PageWriteback(newpage))
		end_page_writeback(newpage);
}


int fail_migrate_page(struct address_space *mapping,
			struct page *newpage, struct page *page)
{
	return -EIO;
}
EXPORT_SYMBOL(fail_migrate_page);

int migrate_page(struct address_space *mapping,
		struct page *newpage, struct page *page,
		enum migrate_mode mode)
{
	int rc;

	BUG_ON(PageWriteback(page));	

	rc = migrate_page_move_mapping(mapping, newpage, page, NULL, mode);

	if (rc)
		return rc;

	migrate_page_copy(newpage, page);
	return 0;
}
EXPORT_SYMBOL(migrate_page);

#ifdef CONFIG_BLOCK
int buffer_migrate_page(struct address_space *mapping,
		struct page *newpage, struct page *page, enum migrate_mode mode)
{
	struct buffer_head *bh, *head;
	int rc;

	if (!page_has_buffers(page))
		return migrate_page(mapping, newpage, page, mode);

	head = page_buffers(page);

	rc = migrate_page_move_mapping(mapping, newpage, page, head, mode);

	if (rc)
		return rc;

	if (mode != MIGRATE_ASYNC)
		BUG_ON(!buffer_migrate_lock_buffers(head, mode));

	ClearPagePrivate(page);
	set_page_private(newpage, page_private(page));
	set_page_private(page, 0);
	put_page(page);
	get_page(newpage);

	bh = head;
	do {
		set_bh_page(bh, newpage, bh_offset(bh));
		bh = bh->b_this_page;

	} while (bh != head);

	SetPagePrivate(newpage);

	migrate_page_copy(newpage, page);

	bh = head;
	do {
		unlock_buffer(bh);
 		put_bh(bh);
		bh = bh->b_this_page;

	} while (bh != head);

	return 0;
}
EXPORT_SYMBOL(buffer_migrate_page);
#endif

static int writeout(struct address_space *mapping, struct page *page)
{
	struct writeback_control wbc = {
		.sync_mode = WB_SYNC_NONE,
		.nr_to_write = 1,
		.range_start = 0,
		.range_end = LLONG_MAX,
		.for_reclaim = 1
	};
	int rc;

	if (!mapping->a_ops->writepage)
		
		return -EINVAL;

	if (!clear_page_dirty_for_io(page))
		
		return -EAGAIN;

	remove_migration_ptes(page, page);

	rc = mapping->a_ops->writepage(page, &wbc);

	if (rc != AOP_WRITEPAGE_ACTIVATE)
		
		lock_page(page);

	return (rc < 0) ? -EIO : -EAGAIN;
}

static int fallback_migrate_page(struct address_space *mapping,
	struct page *newpage, struct page *page, enum migrate_mode mode)
{
	if (PageDirty(page)) {
		
		if (mode != MIGRATE_SYNC)
			return -EBUSY;
		return writeout(mapping, page);
	}

	if (page_has_private(page) &&
	    !try_to_release_page(page, GFP_KERNEL))
		return -EAGAIN;

	return migrate_page(mapping, newpage, page, mode);
}

static int move_to_new_page(struct page *newpage, struct page *page,
				int remap_swapcache, enum migrate_mode mode)
{
	struct address_space *mapping;
	int rc;

	if (!trylock_page(newpage))
		BUG();

	
	newpage->index = page->index;
	newpage->mapping = page->mapping;
	if (PageSwapBacked(page))
		SetPageSwapBacked(newpage);

	mapping = page_mapping(page);
	if (!mapping)
		rc = migrate_page(mapping, newpage, page, mode);
	else if (mapping->a_ops->migratepage)
		rc = mapping->a_ops->migratepage(mapping,
						newpage, page, mode);
	else
		rc = fallback_migrate_page(mapping, newpage, page, mode);

	if (rc) {
		newpage->mapping = NULL;
	} else {
		if (remap_swapcache)
			remove_migration_ptes(page, newpage);
		page->mapping = NULL;
	}

	unlock_page(newpage);

	return rc;
}

static int __unmap_and_move(struct page *page, struct page *newpage,
			int force, bool offlining, enum migrate_mode mode)
{
	int rc = -EAGAIN;
	int remap_swapcache = 1;
	int charge = 0;
	struct mem_cgroup *mem;
	struct anon_vma *anon_vma = NULL;

	if (!trylock_page(page)) {
		if (!force || mode == MIGRATE_ASYNC)
			goto out;

		if (current->flags & PF_MEMALLOC)
			goto out;

		lock_page(page);
	}

	if (PageKsm(page) && !offlining) {
		rc = -EBUSY;
		goto unlock;
	}

	
	charge = mem_cgroup_prepare_migration(page, newpage, &mem, GFP_KERNEL);
	if (charge == -ENOMEM) {
		rc = -ENOMEM;
		goto unlock;
	}
	BUG_ON(charge);

	if (PageWriteback(page)) {
		if (mode != MIGRATE_SYNC) {
			rc = -EBUSY;
			goto uncharge;
		}
		if (!force)
			goto uncharge;
		wait_on_page_writeback(page);
	}
	if (PageAnon(page)) {
		anon_vma = page_get_anon_vma(page);
		if (anon_vma) {
		} else if (PageSwapCache(page)) {
			remap_swapcache = 0;
		} else {
			goto uncharge;
		}
	}

	if (!page->mapping) {
		VM_BUG_ON(PageAnon(page));
		if (page_has_private(page)) {
			try_to_free_buffers(page);
			goto uncharge;
		}
		goto skip_unmap;
	}

	
	try_to_unmap(page, TTU_MIGRATION|TTU_IGNORE_MLOCK|TTU_IGNORE_ACCESS);

skip_unmap:
	if (!page_mapped(page))
		rc = move_to_new_page(newpage, page, remap_swapcache, mode);

	if (rc && remap_swapcache)
		remove_migration_ptes(page, page);

	
	if (anon_vma)
		put_anon_vma(anon_vma);

uncharge:
	if (!charge)
		mem_cgroup_end_migration(mem, page, newpage, rc == 0);
unlock:
	unlock_page(page);
out:
	return rc;
}

static int unmap_and_move(new_page_t get_new_page, unsigned long private,
			struct page *page, int force, bool offlining,
			enum migrate_mode mode)
{
	int rc = 0;
	int *result = NULL;
	struct page *newpage = get_new_page(page, private, &result);

	if (!newpage)
		return -ENOMEM;

	if (page_count(page) == 1) {
		
		goto out;
	}

	if (unlikely(PageTransHuge(page)))
		if (unlikely(split_huge_page(page)))
			goto out;

	rc = __unmap_and_move(page, newpage, force, offlining, mode);
out:
	if (rc != -EAGAIN) {
		list_del(&page->lru);
		dec_zone_page_state(page, NR_ISOLATED_ANON +
				page_is_file_cache(page));
		putback_lru_page(page);
	}
	putback_lru_page(newpage);
	if (result) {
		if (rc)
			*result = rc;
		else
			*result = page_to_nid(newpage);
	}
	return rc;
}

static int unmap_and_move_huge_page(new_page_t get_new_page,
				unsigned long private, struct page *hpage,
				int force, bool offlining,
				enum migrate_mode mode)
{
	int rc = 0;
	int *result = NULL;
	struct page *new_hpage = get_new_page(hpage, private, &result);
	struct anon_vma *anon_vma = NULL;

	if (!new_hpage)
		return -ENOMEM;

	rc = -EAGAIN;

	if (!trylock_page(hpage)) {
		if (!force || mode != MIGRATE_SYNC)
			goto out;
		lock_page(hpage);
	}

	if (PageAnon(hpage))
		anon_vma = page_get_anon_vma(hpage);

	try_to_unmap(hpage, TTU_MIGRATION|TTU_IGNORE_MLOCK|TTU_IGNORE_ACCESS);

	if (!page_mapped(hpage))
		rc = move_to_new_page(new_hpage, hpage, 1, mode);

	if (rc)
		remove_migration_ptes(hpage, hpage);

	if (anon_vma)
		put_anon_vma(anon_vma);
	unlock_page(hpage);

out:
	if (rc != -EAGAIN) {
		list_del(&hpage->lru);
		put_page(hpage);
	}

	put_page(new_hpage);

	if (result) {
		if (rc)
			*result = rc;
		else
			*result = page_to_nid(new_hpage);
	}
	return rc;
}

int migrate_pages(struct list_head *from,
		new_page_t get_new_page, unsigned long private, bool offlining,
		enum migrate_mode mode)
{
	int retry = 1;
	int nr_failed = 0;
	int nr_succeeded = 0;
	int pass = 0;
	struct page *page;
	struct page *page2;
	int swapwrite = current->flags & PF_SWAPWRITE;
	int rc;

	trace_migrate_pages_start(mode);
	if (!swapwrite)
		current->flags |= PF_SWAPWRITE;

	for(pass = 0; pass < 10 && retry; pass++) {
		retry = 0;

		list_for_each_entry_safe(page, page2, from, lru) {
			cond_resched();

			rc = unmap_and_move(get_new_page, private,
						page, pass > 2, offlining,
						mode);

			switch(rc) {
			case -ENOMEM:
				goto out;
			case -EAGAIN:
				retry++;
				trace_migrate_retry(retry);
				break;
			case 0:
				nr_succeeded++;
				break;
			default:
				
				nr_failed++;
				break;
			}
		}
	}
	rc = 0;
out:
	if (nr_succeeded)
		count_vm_events(PGMIGRATE_SUCCESS, nr_succeeded);
	if (nr_failed)
		count_vm_events(PGMIGRATE_FAIL, nr_failed);
	if (!swapwrite)
		current->flags &= ~PF_SWAPWRITE;

	trace_migrate_pages_end(mode);
	if (rc)
		return rc;

	return nr_failed + retry;
}

int migrate_huge_pages(struct list_head *from,
		new_page_t get_new_page, unsigned long private, bool offlining,
		enum migrate_mode mode)
{
	int retry = 1;
	int nr_failed = 0;
	int pass = 0;
	struct page *page;
	struct page *page2;
	int rc;

	for (pass = 0; pass < 10 && retry; pass++) {
		retry = 0;

		list_for_each_entry_safe(page, page2, from, lru) {
			cond_resched();

			rc = unmap_and_move_huge_page(get_new_page,
					private, page, pass > 2, offlining,
					mode);

			switch(rc) {
			case -ENOMEM:
				goto out;
			case -EAGAIN:
				retry++;
				break;
			case 0:
				break;
			default:
				
				nr_failed++;
				break;
			}
		}
	}
	rc = 0;
out:
	if (rc)
		return rc;

	return nr_failed + retry;
}

#ifdef CONFIG_NUMA
struct page_to_node {
	unsigned long addr;
	struct page *page;
	int node;
	int status;
};

static struct page *new_page_node(struct page *p, unsigned long private,
		int **result)
{
	struct page_to_node *pm = (struct page_to_node *)private;

	while (pm->node != MAX_NUMNODES && pm->page != p)
		pm++;

	if (pm->node == MAX_NUMNODES)
		return NULL;

	*result = &pm->status;

	return alloc_pages_exact_node(pm->node,
				GFP_HIGHUSER_MOVABLE | GFP_THISNODE, 0);
}

static int do_move_page_to_node_array(struct mm_struct *mm,
				      struct page_to_node *pm,
				      int migrate_all)
{
	int err;
	struct page_to_node *pp;
	LIST_HEAD(pagelist);

	down_read(&mm->mmap_sem);

	for (pp = pm; pp->node != MAX_NUMNODES; pp++) {
		struct vm_area_struct *vma;
		struct page *page;

		err = -EFAULT;
		vma = find_vma(mm, pp->addr);
		if (!vma || pp->addr < vma->vm_start || !vma_migratable(vma))
			goto set_status;

		page = follow_page(vma, pp->addr, FOLL_GET|FOLL_SPLIT);

		err = PTR_ERR(page);
		if (IS_ERR(page))
			goto set_status;

		err = -ENOENT;
		if (!page)
			goto set_status;

		
		if (PageReserved(page) || PageKsm(page))
			goto put_and_set;

		pp->page = page;
		err = page_to_nid(page);

		if (err == pp->node)
			goto put_and_set;

		err = -EACCES;
		if (page_mapcount(page) > 1 &&
				!migrate_all)
			goto put_and_set;

		err = isolate_lru_page(page);
		if (!err) {
			list_add_tail(&page->lru, &pagelist);
			inc_zone_page_state(page, NR_ISOLATED_ANON +
					    page_is_file_cache(page));
		}
put_and_set:
		put_page(page);
set_status:
		pp->status = err;
	}

	err = 0;
	if (!list_empty(&pagelist)) {
		err = migrate_pages(&pagelist, new_page_node,
				(unsigned long)pm, 0, MIGRATE_SYNC);
		if (err)
			putback_lru_pages(&pagelist);
	}

	up_read(&mm->mmap_sem);
	return err;
}

static int do_pages_move(struct mm_struct *mm, nodemask_t task_nodes,
			 unsigned long nr_pages,
			 const void __user * __user *pages,
			 const int __user *nodes,
			 int __user *status, int flags)
{
	struct page_to_node *pm;
	unsigned long chunk_nr_pages;
	unsigned long chunk_start;
	int err;

	err = -ENOMEM;
	pm = (struct page_to_node *)__get_free_page(GFP_KERNEL);
	if (!pm)
		goto out;

	migrate_prep();

	chunk_nr_pages = (PAGE_SIZE / sizeof(struct page_to_node)) - 1;

	for (chunk_start = 0;
	     chunk_start < nr_pages;
	     chunk_start += chunk_nr_pages) {
		int j;

		if (chunk_start + chunk_nr_pages > nr_pages)
			chunk_nr_pages = nr_pages - chunk_start;

		
		for (j = 0; j < chunk_nr_pages; j++) {
			const void __user *p;
			int node;

			err = -EFAULT;
			if (get_user(p, pages + j + chunk_start))
				goto out_pm;
			pm[j].addr = (unsigned long) p;

			if (get_user(node, nodes + j + chunk_start))
				goto out_pm;

			err = -ENODEV;
			if (node < 0 || node >= MAX_NUMNODES)
				goto out_pm;

			if (!node_state(node, N_HIGH_MEMORY))
				goto out_pm;

			err = -EACCES;
			if (!node_isset(node, task_nodes))
				goto out_pm;

			pm[j].node = node;
		}

		
		pm[chunk_nr_pages].node = MAX_NUMNODES;

		
		err = do_move_page_to_node_array(mm, pm,
						 flags & MPOL_MF_MOVE_ALL);
		if (err < 0)
			goto out_pm;

		
		for (j = 0; j < chunk_nr_pages; j++)
			if (put_user(pm[j].status, status + j + chunk_start)) {
				err = -EFAULT;
				goto out_pm;
			}
	}
	err = 0;

out_pm:
	free_page((unsigned long)pm);
out:
	return err;
}

static void do_pages_stat_array(struct mm_struct *mm, unsigned long nr_pages,
				const void __user **pages, int *status)
{
	unsigned long i;

	down_read(&mm->mmap_sem);

	for (i = 0; i < nr_pages; i++) {
		unsigned long addr = (unsigned long)(*pages);
		struct vm_area_struct *vma;
		struct page *page;
		int err = -EFAULT;

		vma = find_vma(mm, addr);
		if (!vma || addr < vma->vm_start)
			goto set_status;

		page = follow_page(vma, addr, 0);

		err = PTR_ERR(page);
		if (IS_ERR(page))
			goto set_status;

		err = -ENOENT;
		
		if (!page || PageReserved(page) || PageKsm(page))
			goto set_status;

		err = page_to_nid(page);
set_status:
		*status = err;

		pages++;
		status++;
	}

	up_read(&mm->mmap_sem);
}

static int do_pages_stat(struct mm_struct *mm, unsigned long nr_pages,
			 const void __user * __user *pages,
			 int __user *status)
{
#define DO_PAGES_STAT_CHUNK_NR 16
	const void __user *chunk_pages[DO_PAGES_STAT_CHUNK_NR];
	int chunk_status[DO_PAGES_STAT_CHUNK_NR];

	while (nr_pages) {
		unsigned long chunk_nr;

		chunk_nr = nr_pages;
		if (chunk_nr > DO_PAGES_STAT_CHUNK_NR)
			chunk_nr = DO_PAGES_STAT_CHUNK_NR;

		if (copy_from_user(chunk_pages, pages, chunk_nr * sizeof(*chunk_pages)))
			break;

		do_pages_stat_array(mm, chunk_nr, chunk_pages, chunk_status);

		if (copy_to_user(status, chunk_status, chunk_nr * sizeof(*status)))
			break;

		pages += chunk_nr;
		status += chunk_nr;
		nr_pages -= chunk_nr;
	}
	return nr_pages ? -EFAULT : 0;
}

SYSCALL_DEFINE6(move_pages, pid_t, pid, unsigned long, nr_pages,
		const void __user * __user *, pages,
		const int __user *, nodes,
		int __user *, status, int, flags)
{
	const struct cred *cred = current_cred(), *tcred;
	struct task_struct *task;
	struct mm_struct *mm;
	int err;
	nodemask_t task_nodes;

	
	if (flags & ~(MPOL_MF_MOVE|MPOL_MF_MOVE_ALL))
		return -EINVAL;

	if ((flags & MPOL_MF_MOVE_ALL) && !capable(CAP_SYS_NICE))
		return -EPERM;

	
	rcu_read_lock();
	task = pid ? find_task_by_vpid(pid) : current;
	if (!task) {
		rcu_read_unlock();
		return -ESRCH;
	}
	get_task_struct(task);

	tcred = __task_cred(task);
	if (cred->euid != tcred->suid && cred->euid != tcred->uid &&
	    cred->uid  != tcred->suid && cred->uid  != tcred->uid &&
	    !capable(CAP_SYS_NICE)) {
		rcu_read_unlock();
		err = -EPERM;
		goto out;
	}
	rcu_read_unlock();

 	err = security_task_movememory(task);
 	if (err)
		goto out;

	task_nodes = cpuset_mems_allowed(task);
	mm = get_task_mm(task);
	put_task_struct(task);

	if (!mm)
		return -EINVAL;

	if (nodes)
		err = do_pages_move(mm, task_nodes, nr_pages, pages,
				    nodes, status, flags);
	else
		err = do_pages_stat(mm, nr_pages, pages, status);

	mmput(mm);
	return err;

out:
	put_task_struct(task);
	return err;
}

int migrate_vmas(struct mm_struct *mm, const nodemask_t *to,
	const nodemask_t *from, unsigned long flags)
{
 	struct vm_area_struct *vma;
 	int err = 0;

	for (vma = mm->mmap; vma && !err; vma = vma->vm_next) {
 		if (vma->vm_ops && vma->vm_ops->migrate) {
 			err = vma->vm_ops->migrate(vma, to, from, flags);
 			if (err)
 				break;
 		}
 	}
 	return err;
}
#endif
