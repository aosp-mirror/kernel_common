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
    bindings,
    fs::File,
    mm,
    prelude::*,
    sync::poll::PollTable,
    sync::{Arc, ArcBorrow},
    uaccess::{UserSlice, UserSliceReader},
};

use crate::{context::Context, defs::*};

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

    fn version(&self, data: UserSlice) -> Result {
        data.writer().write(&BinderVersion::current())
    }
}

/// The ioctl handler.
impl Process {
    /// Ioctls that are write-only from the perspective of userspace.
    ///
    /// The kernel will only read from the pointer that userspace provided to us.
    fn ioctl_write_only(
        _this: ArcBorrow<'_, Process>,
        _file: &File,
        _cmd: u32,
        _reader: &mut UserSliceReader,
    ) -> Result {
        Err(EINVAL)
    }

    /// Ioctls that are read/write from the perspective of userspace.
    ///
    /// The kernel will both read from and write to the pointer that userspace provided to us.
    fn ioctl_write_read(
        this: ArcBorrow<'_, Process>,
        _file: &File,
        cmd: u32,
        data: UserSlice,
    ) -> Result {
        match cmd {
            bindings::BINDER_VERSION => this.version(data)?,
            _ => return Err(EINVAL),
        }
        Ok(())
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

    pub(crate) fn ioctl(this: ArcBorrow<'_, Process>, file: &File, cmd: u32, arg: usize) -> Result {
        use kernel::ioctl::{_IOC_DIR, _IOC_SIZE};
        use kernel::uapi::{_IOC_READ, _IOC_WRITE};

        crate::trace::trace_ioctl(cmd, arg as usize);

        let user_slice = UserSlice::new(arg, _IOC_SIZE(cmd));

        const _IOC_READ_WRITE: u32 = _IOC_READ | _IOC_WRITE;

        let res = match _IOC_DIR(cmd) {
            _IOC_WRITE => Self::ioctl_write_only(this, file, cmd, &mut user_slice.reader()),
            _IOC_READ_WRITE => Self::ioctl_write_read(this, file, cmd, user_slice),
            _ => Err(EINVAL),
        };

        crate::trace::trace_ioctl_done(res);
        res
    }

    pub(crate) fn compat_ioctl(
        this: ArcBorrow<'_, Process>,
        file: &File,
        cmd: u32,
        arg: usize,
    ) -> Result {
        Self::ioctl(this, file, cmd, arg)
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
