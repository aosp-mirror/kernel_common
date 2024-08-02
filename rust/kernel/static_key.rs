// SPDX-License-Identifier: GPL-2.0

// Copyright (C) 2024 Google LLC.

//! Logic for static keys.

use crate::bindings::*;

/// Branch based on a static key.
///
/// Takes three arguments:
///
/// * `key` - the path to the static variable containing the `static_key`.
/// * `keytyp` - the type of `key`.
/// * `field` - the name of the field of `key` that contains the `static_key`.
#[macro_export]
macro_rules! static_key_false {
    ($key:path, $keytyp:ty, $field:ident) => {{
        // Assert that `$key` has type `$keytyp` and that `$key.$field` has type `static_key`.
        //
        // SAFETY: We know that `$key` is a static because otherwise the inline assembly will not
        // compile. The raw pointers created in this block are in-bounds of `$key`.
        static _TY_ASSERT: () = unsafe {
            let key: *const $keytyp = ::core::ptr::addr_of!($key);
            let _: *const $crate::bindings::static_key = ::core::ptr::addr_of!((*key).$field);
        };

        $crate::arch::jump_label::arch_static_branch! { $key, $keytyp, $field, false }
    }};
}

pub use static_key_false;
