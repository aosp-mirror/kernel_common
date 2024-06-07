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

/*
 * The internal data types in the Rust Binder driver are opaque to C, so we use
 * void pointer typedefs for these types.
 */
typedef void *rust_binder_transaction;

struct rb_transaction_layout {
	size_t debug_id;
	size_t code;
	size_t flags;
};

struct rust_binder_layout {
	struct rb_transaction_layout t;
};

extern const struct rust_binder_layout RUST_BINDER_LAYOUT;

static inline size_t rust_binder_transaction_debug_id(rust_binder_transaction t)
{
	return * (size_t *) (t + RUST_BINDER_LAYOUT.t.debug_id);
}

static inline u32 rust_binder_transaction_code(rust_binder_transaction t)
{
	return * (u32 *) (t + RUST_BINDER_LAYOUT.t.code);
}

static inline u32 rust_binder_transaction_flags(rust_binder_transaction t)
{
	return * (u32 *) (t + RUST_BINDER_LAYOUT.t.flags);
}

#endif
