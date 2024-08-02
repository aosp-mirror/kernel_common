// SPDX-License-Identifier: GPL-2.0

// Copyright (C) 2024 Google LLC.

//! Loongarch Rust implementation of jump_label.h

/// loongarch implementation of arch_static_branch
#[doc(hidden)]
#[macro_export]
#[cfg(target_arch = "loongarch64")]
macro_rules! arch_static_branch {
    ($key:path, $keytyp:ty, $field:ident, $branch:expr) => {'my_label: {
        core::arch::asm!(
            r#"
            1: nop

            .pushsection __jump_table,  "aw"
            .align 3
            .long 1b - ., {0} - .
            .quad {1} + {2} + {3} - .
            .popsection
            "#,
            label {
                break 'my_label true;
            },
            sym $key,
            const ::core::mem::offset_of!($keytyp, $field),
            const $crate::arch::bool_to_int($branch),
        );

        break 'my_label false;
    }};
}

pub use arch_static_branch;
