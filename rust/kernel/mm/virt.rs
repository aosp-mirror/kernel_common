// SPDX-License-Identifier: GPL-2.0

// Copyright (C) 2024 Google LLC.

//! Virtual memory.

use crate::{
    bindings,
    error::{to_result, Result},
    page::Page,
    types::Opaque,
};

use core::pin::Pin;

/// A wrapper for the kernel's `struct vm_area_struct`.
///
/// It represents an area of virtual memory.
///
/// # Invariants
///
/// * If the caller has shared access to this type, then they must hold the mmap read lock.
/// * If the caller has exclusive access to this type, then they must hold the mmap write lock.
#[repr(transparent)]
pub struct VmArea {
    vma: Opaque<bindings::vm_area_struct>,
}

impl VmArea {
    /// Access a virtual memory area given a raw pointer.
    ///
    /// # Safety
    ///
    /// Callers must ensure that `vma` is valid for the duration of 'a, and that the mmap read lock
    /// (or write lock) is held for at least the duration of 'a.
    #[inline]
    pub unsafe fn from_raw<'a>(vma: *const bindings::vm_area_struct) -> &'a Self {
        // SAFETY: The caller ensures that the invariants are satisfied for the duration of 'a.
        unsafe { &*vma.cast() }
    }

    /// Access a virtual memory area given a raw pointer.
    ///
    /// # Safety
    ///
    /// Callers must ensure that `vma` is valid for the duration of 'a, and that the mmap write
    /// lock is held for at least the duration of 'a.
    #[inline]
    pub unsafe fn from_raw_mut<'a>(vma: *mut bindings::vm_area_struct) -> Pin<&'a mut Self> {
        // SAFETY: The caller ensures that the invariants are satisfied for the duration of 'a.
        unsafe { Pin::new_unchecked(&mut *vma.cast()) }
    }

    /// Returns a raw pointer to this area.
    #[inline]
    pub fn as_ptr(&self) -> *mut bindings::vm_area_struct {
        self.vma.get()
    }

    /// Returns the flags associated with the virtual memory area.
    ///
    /// The possible flags are a combination of the constants in [`flags`].
    #[inline]
    pub fn flags(&self) -> usize {
        // SAFETY: By the type invariants, the caller holds at least the mmap read lock, so this
        // access is not a data race.
        unsafe { (*self.as_ptr()).__bindgen_anon_2.vm_flags as _ }
    }

    /// Sets the flags associated with the virtual memory area.
    ///
    /// The possible flags are a combination of the constants in [`flags`].
    #[inline]
    pub fn set_flags(self: Pin<&mut Self>, flags: usize) {
        // SAFETY: By the type invariants, the caller holds the mmap write lock, so this access is
        // not a data race.
        unsafe { (*self.as_ptr()).__bindgen_anon_2.vm_flags = flags as _ };
    }

    /// Returns the start address of the virtual memory area.
    #[inline]
    pub fn start(&self) -> usize {
        // SAFETY: By the type invariants, the caller holds at least the mmap read lock, so this
        // access is not a data race.
        unsafe { (*self.as_ptr()).__bindgen_anon_1.__bindgen_anon_1.vm_start as _ }
    }

    /// Returns the end address of the virtual memory area.
    #[inline]
    pub fn end(&self) -> usize {
        // SAFETY: By the type invariants, the caller holds at least the mmap read lock, so this
        // access is not a data race.
        unsafe { (*self.as_ptr()).__bindgen_anon_1.__bindgen_anon_1.vm_end as _ }
    }

    /// Make this vma anonymous.
    #[inline]
    pub fn set_anonymous(self: Pin<&mut Self>) {
        // SAFETY: By the type invariants, the caller holds the mmap write lock, so this access is
        // not a data race.
        unsafe { (*self.as_ptr()).vm_ops = core::ptr::null() };
    }

    /// Maps a single page at the given address within the virtual memory area.
    ///
    /// This operation does not take ownership of the page.
    #[inline]
    pub fn vm_insert_page(self: Pin<&mut Self>, address: usize, page: &Page) -> Result {
        // SAFETY: By the type invariants, the caller holds the mmap write lock, so this access is
        // not a data race. The page is guaranteed to be valid and of order 0. The range of
        // `address` is already checked by `vm_insert_page`.
        to_result(unsafe { bindings::vm_insert_page(self.as_ptr(), address as _, page.as_ptr()) })
    }

    /// Unmap pages in the given page range.
    #[inline]
    pub fn zap_page_range_single(&self, address: usize, size: usize) {
        // SAFETY: By the type invariants, the caller holds at least the mmap read lock, so this
        // access is okay. Any value of `address` and `size` is allowed.
        unsafe {
            bindings::zap_page_range_single(
                self.as_ptr(),
                address as _,
                size as _,
                core::ptr::null_mut(),
            )
        };
    }
}

/// Container for [`VmArea`] flags.
pub mod flags {
    use crate::bindings;

    /// No flags are set.
    pub const NONE: usize = bindings::VM_NONE as _;

    /// Mapping allows reads.
    pub const READ: usize = bindings::VM_READ as _;

    /// Mapping allows writes.
    pub const WRITE: usize = bindings::VM_WRITE as _;

    /// Mapping allows execution.
    pub const EXEC: usize = bindings::VM_EXEC as _;

    /// Mapping is shared.
    pub const SHARED: usize = bindings::VM_SHARED as _;

    /// Mapping may be updated to allow reads.
    pub const MAYREAD: usize = bindings::VM_MAYREAD as _;

    /// Mapping may be updated to allow writes.
    pub const MAYWRITE: usize = bindings::VM_MAYWRITE as _;

    /// Mapping may be updated to allow execution.
    pub const MAYEXEC: usize = bindings::VM_MAYEXEC as _;

    /// Mapping may be updated to be shared.
    pub const MAYSHARE: usize = bindings::VM_MAYSHARE as _;

    /// Do not copy this vma on fork.
    pub const DONTCOPY: usize = bindings::VM_DONTCOPY as _;

    /// Cannot expand with mremap().
    pub const DONTEXPAND: usize = bindings::VM_DONTEXPAND as _;

    /// Lock the pages covered when they are faulted in.
    pub const LOCKONFAULT: usize = bindings::VM_LOCKONFAULT as _;

    /// Is a VM accounted object.
    pub const ACCOUNT: usize = bindings::VM_ACCOUNT as _;

    /// should the VM suppress accounting.
    pub const NORESERVE: usize = bindings::VM_NORESERVE as _;

    /// Huge TLB Page VM.
    pub const HUGETLB: usize = bindings::VM_HUGETLB as _;

    /// Synchronous page faults.
    pub const SYNC: usize = bindings::VM_SYNC as _;

    /// Architecture-specific flag.
    pub const ARCH_1: usize = bindings::VM_ARCH_1 as _;

    /// Wipe VMA contents in child..
    pub const WIPEONFORK: usize = bindings::VM_WIPEONFORK as _;

    /// Do not include in the core dump.
    pub const DONTDUMP: usize = bindings::VM_DONTDUMP as _;

    /// Not soft dirty clean area.
    pub const SOFTDIRTY: usize = bindings::VM_SOFTDIRTY as _;

    /// Can contain "struct page" and pure PFN pages.
    pub const MIXEDMAP: usize = bindings::VM_MIXEDMAP as _;

    /// MADV_HUGEPAGE marked this vma.
    pub const HUGEPAGE: usize = bindings::VM_HUGEPAGE as _;

    /// MADV_NOHUGEPAGE marked this vma.
    pub const NOHUGEPAGE: usize = bindings::VM_NOHUGEPAGE as _;

    /// KSM may merge identical pages.
    pub const MERGEABLE: usize = bindings::VM_MERGEABLE as _;
}
