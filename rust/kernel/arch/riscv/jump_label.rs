// SPDX-License-Identifier: GPL-2.0

// Copyright (C) 2024 Google LLC.

//! RiscV Rust implementation of jump_label.h

/// riscv implementation of arch_static_branch
#[macro_export]
#[cfg(target_arch = "riscv64")]
macro_rules! arch_static_branch {
    ($key:path, $keytyp:ty, $field:ident, $branch:expr) => {'my_label: {
        core::arch::asm!(
            r#"
            .align  2
            .option push
            .option norelax
            .option norvc
            1: nop
            .option pop
            .pushsection __jump_table,  "aw"
            .align 3
            .long 1b - ., {0} - .
            .dword {1} + {2} + {3} - .
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
