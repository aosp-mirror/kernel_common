// SPDX-License-Identifier: GPL-2.0

// Copyright (C) 2024 Google LLC.

use kernel::error::Result;
use kernel::tracepoint::declare_trace;

use core::ffi::{c_int, c_uint, c_ulong};

declare_trace! {
    unsafe fn rust_binder_ioctl(cmd: c_uint, arg: c_ulong);
    unsafe fn rust_binder_ioctl_done(ret: c_int);
    unsafe fn rust_binder_transaction(debug_id: c_int, reply: bool);
}

#[inline]
fn to_errno(ret: Result) -> i32 {
    match ret {
        Ok(()) => 0,
        Err(err) => err.to_errno(),
    }
}

#[inline]
pub(crate) fn trace_ioctl(cmd: u32, arg: usize) {
    // SAFETY: Always safe to call.
    unsafe { rust_binder_ioctl(cmd, arg as c_ulong) }
}

#[inline]
pub(crate) fn trace_ioctl_done(ret: Result) {
    // SAFETY: Always safe to call.
    unsafe { rust_binder_ioctl_done(to_errno(ret)) }
}

#[inline]
pub(crate) fn trace_transaction(debug_id: usize, reply: bool) {
    // SAFETY: No safety requirements for this tracepoint.
    unsafe { rust_binder_transaction(debug_id as c_int, reply) }
}
