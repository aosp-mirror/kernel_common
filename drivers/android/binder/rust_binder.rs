// SPDX-License-Identifier: GPL-2.0

// Copyright (C) 2024 Google LLC.

//! Binder -- the Android IPC mechanism.

use kernel::prelude::*;

module! {
    type: BinderModule,
    name: "rust_binder",
    author: "Wedson Almeida Filho, Alice Ryhl",
    description: "Android Binder",
    license: "GPL",
}

struct BinderModule {}

impl kernel::Module for BinderModule {
    fn init(_module: &'static kernel::ThisModule) -> Result<Self> {
        // SAFETY: This just accesses global booleans.
        #[cfg(CONFIG_ANDROID_BINDER_IPC)]
        unsafe {
            extern "C" {
                static mut binder_use_rust: bool;
                static mut binder_driver_initialized: bool;
            }

            if !binder_use_rust {
                return Ok(Self {});
            }
            binder_driver_initialized = true;
        }

        Ok(Self {})
    }
}
