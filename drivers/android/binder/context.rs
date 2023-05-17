// SPDX-License-Identifier: GPL-2.0

// Copyright (C) 2024 Google LLC.

use kernel::{
    prelude::*,
    str::{CStr, CString},
    sync::Arc,
};

/// There is one context per binder file (/dev/binder, /dev/hwbinder, etc)
#[pin_data]
pub(crate) struct Context {
    pub(crate) name: CString,
}

impl Context {
    pub(crate) fn new(name: &CStr) -> Result<Arc<Self>> {
        let ctx = Arc::new(
            Context {
                name: CString::try_from(name)?,
            },
            GFP_KERNEL,
        )?;

        Ok(ctx)
    }
}
