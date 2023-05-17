// SPDX-License-Identifier: GPL-2.0

// Copyright (C) 2024 Google LLC.

//! This module defines the `Process` type, which represents a process using a particular binder
//! context.
//!
//! The `Process` object keeps track of all of the resources that this process owns in the binder
//! context.
//!
//! There is one `Process` object for each binder fd that a process has opened, so processes using
//! several binder contexts have several `Process` objects. This ensures that the contexts are
//! fully separated.

use kernel::{
    fs::File,
    mm,
    prelude::*,
    sync::poll::PollTable,
    sync::{Arc, ArcBorrow},
};

use crate::context::Context;

/// A process using binder.
///
/// Strictly speaking, there can be multiple of these per process. There is one for each binder fd
/// that a process has opened, so processes using several binder contexts have several `Process`
/// objects. This ensures that the contexts are fully separated.
#[pin_data]
pub(crate) struct Process {
    pub(crate) ctx: Arc<Context>,
}

impl Process {
    fn new(ctx: Arc<Context>) -> Result<Arc<Self>> {
        let process = Arc::new(Process { ctx }, GFP_KERNEL)?;

        Ok(process)
    }
}

/// The file operations supported by `Process`.
impl Process {
    pub(crate) fn open(ctx: ArcBorrow<'_, Context>, _file: &File) -> Result<Arc<Process>> {
        Self::new(ctx.into())
    }

    pub(crate) fn release(_this: Arc<Process>, _file: &File) {}

    pub(crate) fn flush(_this: ArcBorrow<'_, Process>) -> Result {
        Ok(())
    }

    pub(crate) fn ioctl(
        _this: ArcBorrow<'_, Process>,
        _file: &File,
        _cmd: u32,
        _arg: *mut core::ffi::c_void,
    ) -> Result<i32> {
        Err(EINVAL)
    }

    pub(crate) fn compat_ioctl(
        _this: ArcBorrow<'_, Process>,
        _file: &File,
        _cmd: u32,
        _arg: *mut core::ffi::c_void,
    ) -> Result<i32> {
        Err(EINVAL)
    }

    pub(crate) fn mmap(
        _this: ArcBorrow<'_, Process>,
        _file: &File,
        _vma: &mut mm::virt::VmArea,
    ) -> Result {
        Err(EINVAL)
    }

    pub(crate) fn poll(
        _this: ArcBorrow<'_, Process>,
        _file: &File,
        _table: &mut PollTable,
    ) -> Result<u32> {
        Err(EINVAL)
    }
}
