// SPDX-License-Identifier: GPL-2.0

// Copyright (C) 2024 Google LLC.

//! Memory management.
//!
//! C header: [`include/linux/mm.h`](srctree/include/linux/mm.h)

use crate::{
    bindings,
    types::{ARef, AlwaysRefCounted, Opaque},
};

use core::{
    ops::Deref,
    pin::Pin,
    ptr::{self, NonNull},
};

pub mod virt;

/// A wrapper for the kernel's `struct mm_struct`.
///
/// Since `mm_users` may be zero, the associated address space may not exist anymore. You can use
/// [`mmget_not_zero`] to be able to access the address space.
///
/// The `ARef<Mm>` smart pointer holds an `mmgrab` refcount. Its destructor may sleep.
///
/// # Invariants
///
/// Values of this type are always refcounted using `mmgrab`.
///
/// [`mmget_not_zero`]: Mm::mmget_not_zero
pub struct Mm {
    mm: Opaque<bindings::mm_struct>,
}

// SAFETY: It is safe to call `mmdrop` on another thread than where `mmgrab` was called.
unsafe impl Send for Mm {}
// SAFETY: All methods on `Mm` can be called in parallel from several threads.
unsafe impl Sync for Mm {}

// SAFETY: By the type invariants, this type is always refcounted.
unsafe impl AlwaysRefCounted for Mm {
    fn inc_ref(&self) {
        // SAFETY: The pointer is valid since self is a reference.
        unsafe { bindings::mmgrab(self.as_raw()) };
    }

    unsafe fn dec_ref(obj: NonNull<Self>) {
        // SAFETY: The caller is giving up their refcount.
        unsafe { bindings::mmdrop(obj.cast().as_ptr()) };
    }
}

/// A wrapper for the kernel's `struct mm_struct`.
///
/// This type is like [`Mm`], but with non-zero `mm_users`. It can only be used when `mm_users` can
/// be proven to be non-zero at compile-time, usually because the relevant code holds an `mmget`
/// refcount. It can be used to access the associated address space.
///
/// The `ARef<MmWithUser>` smart pointer holds an `mmget` refcount. Its destructor may sleep.
///
/// # Invariants
///
/// Values of this type are always refcounted using `mmget`. The value of `mm_users` is non-zero.
/// #[repr(transparent)]
pub struct MmWithUser {
    mm: Mm,
}

// SAFETY: It is safe to call `mmput` on another thread than where `mmget` was called.
unsafe impl Send for MmWithUser {}
// SAFETY: All methods on `MmWithUser` can be called in parallel from several threads.
unsafe impl Sync for MmWithUser {}

// SAFETY: By the type invariants, this type is always refcounted.
unsafe impl AlwaysRefCounted for MmWithUser {
    fn inc_ref(&self) {
        // SAFETY: The pointer is valid since self is a reference.
        unsafe { bindings::mmget(self.as_raw()) };
    }

    unsafe fn dec_ref(obj: NonNull<Self>) {
        // SAFETY: The caller is giving up their refcount.
        unsafe { bindings::mmput(obj.cast().as_ptr()) };
    }
}

// Make all `Mm` methods available on `MmWithUser`.
impl Deref for MmWithUser {
    type Target = Mm;

    #[inline]
    fn deref(&self) -> &Mm {
        &self.mm
    }
}

/// A wrapper for the kernel's `struct mm_struct`.
///
/// This type is identical to `MmWithUser` except that it uses `mmput_async` when dropping a
/// refcount. This means that the destructor of `ARef<MmWithUserAsync>` is safe to call in atomic
/// context.
///
/// # Invariants
///
/// Values of this type are always refcounted using `mmget`. The value of `mm_users` is non-zero.
/// #[repr(transparent)]
#[repr(transparent)]
pub struct MmWithUserAsync {
    mm: MmWithUser,
}

// SAFETY: It is safe to call `mmput_async` on another thread than where `mmget` was called.
unsafe impl Send for MmWithUserAsync {}
// SAFETY: All methods on `MmWithUserAsync` can be called in parallel from several threads.
unsafe impl Sync for MmWithUserAsync {}

// SAFETY: By the type invariants, this type is always refcounted.
unsafe impl AlwaysRefCounted for MmWithUserAsync {
    fn inc_ref(&self) {
        // SAFETY: The pointer is valid since self is a reference.
        unsafe { bindings::mmget(self.as_raw()) };
    }

    unsafe fn dec_ref(obj: NonNull<Self>) {
        // SAFETY: The caller is giving up their refcount.
        unsafe { bindings::mmput_async(obj.cast().as_ptr()) };
    }
}

// Make all `MmWithUser` methods available on `MmWithUserAsync`.
impl Deref for MmWithUserAsync {
    type Target = MmWithUser;

    #[inline]
    fn deref(&self) -> &MmWithUser {
        &self.mm
    }
}

// These methods are safe to call even if `mm_users` is zero.
impl Mm {
    /// Call `mmgrab` on `current.mm`.
    #[inline]
    pub fn mmgrab_current() -> Option<ARef<Mm>> {
        // SAFETY: It's safe to get the `mm` field from current.
        let mm = unsafe {
            let current = bindings::get_current();
            (*current).mm
        };

        if mm.is_null() {
            return None;
        }

        // SAFETY: The value of `current->mm` is guaranteed to be null or a valid `mm_struct`. We
        // just checked that it's not null. Furthermore, the returned `&Mm` is valid only for the
        // duration of this function, and `current->mm` will stay valid for that long.
        let mm = unsafe { Mm::from_raw(mm) };

        // This increments the refcount using `mmgrab`.
        Some(ARef::from(mm))
    }

    /// Returns a raw pointer to the inner `mm_struct`.
    #[inline]
    pub fn as_raw(&self) -> *mut bindings::mm_struct {
        self.mm.get()
    }

    /// Obtain a reference from a raw pointer.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `ptr` points at an `mm_struct`, and that it is not deallocated
    /// during the lifetime 'a.
    #[inline]
    pub unsafe fn from_raw<'a>(ptr: *const bindings::mm_struct) -> &'a Mm {
        // SAFETY: Caller promises that the pointer is valid for 'a. Layouts are compatible due to
        // repr(transparent).
        unsafe { &*ptr.cast() }
    }

    /// Check whether this vma is associated with this mm.
    #[inline]
    pub fn is_same_mm(&self, area: &virt::VmArea) -> bool {
        // SAFETY: The `vm_mm` field of the area is immutable, so we can read it without
        // synchronization.
        let vm_mm = unsafe { (*area.as_ptr()).vm_mm };

        ptr::eq(vm_mm, self.as_raw())
    }

    /// Calls `mmget_not_zero` and returns a handle if it succeeds.
    #[inline]
    pub fn mmget_not_zero(&self) -> Option<ARef<MmWithUser>> {
        // SAFETY: The pointer is valid since self is a reference.
        let success = unsafe { bindings::mmget_not_zero(self.as_raw()) };

        if success {
            // SAFETY: We just created an `mmget` refcount.
            Some(unsafe { ARef::from_raw(NonNull::new_unchecked(self.as_raw().cast())) })
        } else {
            None
        }
    }
}

// These methods require `mm_users` to be non-zero.
impl MmWithUser {
    /// Obtain a reference from a raw pointer.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `ptr` points at an `mm_struct`, and that `mm_users` remains
    /// non-zero for the duration of the lifetime 'a.
    #[inline]
    pub unsafe fn from_raw<'a>(ptr: *const bindings::mm_struct) -> &'a MmWithUser {
        // SAFETY: Caller promises that the pointer is valid for 'a. The layout is compatible due
        // to repr(transparent).
        unsafe { &*ptr.cast() }
    }

    /// Use `mmput_async` when dropping this refcount.
    pub fn use_mmput_async(me: ARef<MmWithUser>) -> ARef<MmWithUserAsync> {
        // SAFETY: The layouts and invariants are compatible.
        unsafe { ARef::from_raw(ARef::into_raw(me).cast()) }
    }

    /// Lock the mmap write lock.
    #[inline]
    pub fn mmap_write_lock(&self) -> MmapWriteLock<'_> {
        // SAFETY: The pointer is valid since self is a reference.
        unsafe { bindings::mmap_write_lock(self.as_raw()) };

        // INVARIANT: We just acquired the write lock.
        MmapWriteLock { mm: self }
    }

    /// Lock the mmap read lock.
    #[inline]
    pub fn mmap_read_lock(&self) -> MmapReadLock<'_> {
        // SAFETY: The pointer is valid since self is a reference.
        unsafe { bindings::mmap_read_lock(self.as_raw()) };

        // INVARIANT: We just acquired the read lock.
        MmapReadLock { mm: self }
    }

    /// Try to lock the mmap read lock.
    #[inline]
    pub fn mmap_read_trylock(&self) -> Option<MmapReadLock<'_>> {
        // SAFETY: The pointer is valid since self is a reference.
        let success = unsafe { bindings::mmap_read_trylock(self.as_raw()) };

        if success {
            // INVARIANT: We just acquired the read lock.
            Some(MmapReadLock { mm: self })
        } else {
            None
        }
    }
}

impl MmWithUserAsync {
    /// Use `mmput` when dropping this refcount.
    pub fn use_mmput(me: ARef<MmWithUserAsync>) -> ARef<MmWithUser> {
        // SAFETY: The layouts and invariants are compatible.
        unsafe { ARef::from_raw(ARef::into_raw(me).cast()) }
    }
}

/// A guard for the mmap read lock.
///
/// # Invariants
///
/// This `MmapReadLock` guard owns the mmap read lock.
pub struct MmapReadLock<'a> {
    mm: &'a MmWithUser,
}

impl<'a> MmapReadLock<'a> {
    /// Look up a vma at the given address.
    #[inline]
    pub fn vma_lookup(&self, vma_addr: usize) -> Option<&virt::VmArea> {
        // SAFETY: We hold a reference to the mm, so the pointer must be valid. Any value is okay
        // for `vma_addr`.
        let vma = unsafe { bindings::vma_lookup(self.mm.as_raw(), vma_addr as _) };

        if vma.is_null() {
            None
        } else {
            // SAFETY: We just checked that a vma was found, so the pointer is valid. Furthermore,
            // the returned area will borrow from this read lock guard, so it can only be used
            // while the read lock is still held. The returned reference is immutable, so the
            // reference cannot be used to modify the area.
            unsafe { Some(virt::VmArea::from_raw(vma)) }
        }
    }
}

impl Drop for MmapReadLock<'_> {
    #[inline]
    fn drop(&mut self) {
        // SAFETY: We hold the read lock by the type invariants.
        unsafe { bindings::mmap_read_unlock(self.mm.as_raw()) };
    }
}

/// A guard for the mmap write lock.
///
/// # Invariants
///
/// This `MmapReadLock` guard owns the mmap write lock.
pub struct MmapWriteLock<'a> {
    mm: &'a MmWithUser,
}

impl<'a> MmapWriteLock<'a> {
    /// Look up a vma at the given address.
    #[inline]
    pub fn vma_lookup(&mut self, vma_addr: usize) -> Option<Pin<&mut virt::VmArea>> {
        // SAFETY: We hold a reference to the mm, so the pointer must be valid. Any value is okay
        // for `vma_addr`.
        let vma = unsafe { bindings::vma_lookup(self.mm.as_raw(), vma_addr as _) };

        if vma.is_null() {
            None
        } else {
            // SAFETY: We just checked that a vma was found, so the pointer is valid. Furthermore,
            // the returned area will borrow from this write lock guard, so it can only be used
            // while the write lock is still held. We hold the write lock, so mutable operations on
            // the area are okay.
            unsafe { Some(virt::VmArea::from_raw_mut(vma)) }
        }
    }
}

impl Drop for MmapWriteLock<'_> {
    #[inline]
    fn drop(&mut self) {
        // SAFETY: We hold the write lock by the type invariants.
        unsafe { bindings::mmap_write_unlock(self.mm.as_raw()) };
    }
}
