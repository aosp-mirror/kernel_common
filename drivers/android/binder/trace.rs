// SPDX-License-Identifier: GPL-2.0

// Copyright (C) 2024 Google LLC.

use crate::{thread::Thread, transaction::Transaction};

use kernel::bindings::{rust_binder_thread, rust_binder_transaction};
use kernel::error::Result;
use kernel::tracepoint::declare_trace;

use core::ffi::{c_int, c_uint, c_ulong};

declare_trace! {
    unsafe fn rust_binder_ioctl(cmd: c_uint, arg: c_ulong);
    unsafe fn rust_binder_ioctl_done(ret: c_int);
    unsafe fn rust_binder_read_done(ret: c_int);
    unsafe fn rust_binder_write_done(ret: c_int);
    unsafe fn rust_binder_wait_for_work(proc_work: bool, transaction_stack: bool, thread_todo: bool);
    unsafe fn rust_binder_transaction(reply: bool, t: rust_binder_transaction);
    unsafe fn rust_binder_transaction_received(t: rust_binder_transaction);
    unsafe fn rust_binder_transaction_thread_selected(t: rust_binder_transaction, thread: rust_binder_thread);
}

#[inline]
fn raw_transaction(t: &Transaction) -> rust_binder_transaction {
    t as *const Transaction as rust_binder_transaction
}

#[inline]
fn raw_thread(t: &Thread) -> rust_binder_thread {
    t as *const Thread as rust_binder_thread
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
pub(crate) fn trace_read_done(ret: Result) {
    // SAFETY: Always safe to call.
    unsafe { rust_binder_read_done(to_errno(ret)) }
}

#[inline]
pub(crate) fn trace_write_done(ret: Result) {
    // SAFETY: Always safe to call.
    unsafe { rust_binder_write_done(to_errno(ret)) }
}

#[inline]
pub(crate) fn trace_wait_for_work(proc_work: bool, transaction_stack: bool, thread_todo: bool) {
    // SAFETY: Always safe to call.
    unsafe { rust_binder_wait_for_work(proc_work, transaction_stack, thread_todo) }
}

#[inline]
pub(crate) fn trace_transaction(reply: bool, t: &Transaction) {
    // SAFETY: The raw transaction is valid for the duration of this call.
    unsafe { rust_binder_transaction(reply, raw_transaction(t)) }
}

#[inline]
pub(crate) fn trace_transaction_received(t: &Transaction) {
    // SAFETY: The raw transaction is valid for the duration of this call.
    unsafe { rust_binder_transaction_received(raw_transaction(t)) }
}

#[inline]
pub(crate) fn trace_transaction_thread_selected(t: &Transaction, th: &Thread) {
    // SAFETY: The raw transaction is valid for the duration of this call.
    unsafe { rust_binder_transaction_thread_selected(raw_transaction(t), raw_thread(th)) }
}
