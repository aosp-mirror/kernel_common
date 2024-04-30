// SPDX-License-Identifier: GPL-2.0

// Copyright (C) 2024 Google LLC.

use kernel::{page::PAGE_SIZE, prelude::*, seq_file::SeqFile, task::Pid};

mod tree;
use self::tree::{ReserveNewTreeAlloc, TreeRangeAllocator};

/// Represents a range of pages that have just become completely free.
#[derive(Copy, Clone)]
pub(crate) struct FreedRange {
    pub(crate) start_page_idx: usize,
    pub(crate) end_page_idx: usize,
}

impl FreedRange {
    fn interior_pages(offset: usize, size: usize) -> FreedRange {
        FreedRange {
            // Divide round up
            start_page_idx: (offset + (PAGE_SIZE - 1)) / PAGE_SIZE,
            // Divide round down
            end_page_idx: (offset + size) / PAGE_SIZE,
        }
    }
}

pub(crate) struct RangeAllocator<T> {
    inner: Impl<T>,
}

enum Impl<T> {
    Tree(TreeRangeAllocator<T>),
}

impl<T> RangeAllocator<T> {
    pub(crate) fn new(size: usize) -> Result<Self> {
        Ok(Self {
            inner: Impl::Tree(TreeRangeAllocator::new(size)?),
        })
    }

    pub(crate) fn free_oneway_space(&self) -> usize {
        match &self.inner {
            Impl::Tree(tree) => tree.free_oneway_space(),
        }
    }

    pub(crate) fn count_buffers(&self) -> usize {
        match &self.inner {
            Impl::Tree(tree) => tree.count_buffers(),
        }
    }

    pub(crate) fn debug_print(&self, m: &mut SeqFile) -> Result<()> {
        match &self.inner {
            Impl::Tree(tree) => tree.debug_print(m),
        }
    }

    /// Try to reserve a new buffer, using the provided allocation if necessary.
    pub(crate) fn reserve_new(&mut self, args: ReserveNewArgs<T>) -> Result<ReserveNew<T>> {
        match &mut self.inner {
            Impl::Tree(tree) => {
                let alloc = match args.tree_alloc {
                    Some(alloc) => alloc,
                    None => {
                        return Ok(ReserveNew::NeedAlloc(ReserveNewNeedAlloc {
                            args,
                            need_tree_alloc: true,
                        }));
                    }
                };
                let (offset, oneway_spam_detected) =
                    tree.reserve_new(args.debug_id, args.size, args.is_oneway, args.pid, alloc)?;
                Ok(ReserveNew::Success(ReserveNewSuccess {
                    offset,
                    oneway_spam_detected,
                    _tree_alloc: None,
                }))
            }
        }
    }

    /// Deletes the allocations at `offset`.
    pub(crate) fn reservation_abort(&mut self, offset: usize) -> Result<FreedRange> {
        match &mut self.inner {
            Impl::Tree(tree) => tree.reservation_abort(offset),
        }
    }

    /// Called when an allocation is no longer in use by the kernel.
    pub(crate) fn reservation_commit(&mut self, offset: usize, data: Option<T>) -> Result {
        match &mut self.inner {
            Impl::Tree(tree) => tree.reservation_commit(offset, data),
        }
    }

    /// Called when the kernel starts using an allocation.
    ///
    /// Returns the size of the existing entry and the data associated with it.
    pub(crate) fn reserve_existing(&mut self, offset: usize) -> Result<(usize, usize, Option<T>)> {
        match &mut self.inner {
            Impl::Tree(tree) => tree.reserve_existing(offset),
        }
    }

    /// Call the provided callback at every allocated region.
    ///
    /// This destroys the range allocator. Used only during shutdown.
    pub(crate) fn take_for_each<F: Fn(usize, usize, usize, Option<T>)>(&mut self, callback: F) {
        match &mut self.inner {
            Impl::Tree(tree) => tree.take_for_each(callback),
        }
    }
}

/// The arguments for `reserve_new`.
#[derive(Default)]
pub(crate) struct ReserveNewArgs<T> {
    pub(crate) size: usize,
    pub(crate) is_oneway: bool,
    pub(crate) debug_id: usize,
    pub(crate) pid: Pid,
    pub(crate) tree_alloc: Option<ReserveNewTreeAlloc<T>>,
}

/// The return type of `ReserveNew`.
pub(crate) enum ReserveNew<T> {
    Success(ReserveNewSuccess<T>),
    NeedAlloc(ReserveNewNeedAlloc<T>),
}

/// Returned by `reserve_new` when the reservation was successul.
pub(crate) struct ReserveNewSuccess<T> {
    pub(crate) offset: usize,
    pub(crate) oneway_spam_detected: bool,

    // If the user supplied an allocation that we did not end up using, then we return it here.
    // The caller will kfree it outside of the lock.
    _tree_alloc: Option<ReserveNewTreeAlloc<T>>,
}

/// Returned by `reserve_new` to request the caller to make an allocation before calling the method
/// again.
pub(crate) struct ReserveNewNeedAlloc<T> {
    args: ReserveNewArgs<T>,
    need_tree_alloc: bool,
}

impl<T> ReserveNewNeedAlloc<T> {
    /// Make the necessary allocations for another call to `reserve_new`.
    pub(crate) fn make_alloc(mut self) -> Result<ReserveNewArgs<T>> {
        if self.need_tree_alloc && self.args.tree_alloc.is_none() {
            self.args.tree_alloc = Some(ReserveNewTreeAlloc::try_new()?);
        }
        Ok(self.args)
    }
}
