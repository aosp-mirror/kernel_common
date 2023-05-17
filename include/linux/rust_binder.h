/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_RUST_BINDER_H
#define _LINUX_RUST_BINDER_H

#include <uapi/linux/android/binder.h>
#include <uapi/linux/android/binderfs.h>

/*
 * This symbol is exposed by `rust_binderfs.c` and exists here so that Rust
 * Binder can call it.
 */
int init_rust_binderfs(void);

#endif
