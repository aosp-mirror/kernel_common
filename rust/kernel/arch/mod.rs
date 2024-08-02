// SPDX-License-Identifier: GPL-2.0

// Copyright (C) 2024 Google LLC.

//! Architecture specific code.

#[cfg_attr(target_arch = "aarch64", path = "arm64")]
#[cfg_attr(target_arch = "x86_64", path = "x86")]
#[cfg_attr(target_arch = "loongarch64", path = "loongarch")]
#[cfg_attr(target_arch = "riscv64", path = "riscv")]
mod inner {
    pub mod jump_label;
}

pub use self::inner::*;

/// A helper used by inline assembly to pass a boolean to as a `const` parameter.
///
/// Using this function instead of a cast lets you assert that the input is a boolean, rather than
/// some other type that can be cast to an integer.
#[doc(hidden)]
pub const fn bool_to_int(b: bool) -> i32 {
    b as i32
}
