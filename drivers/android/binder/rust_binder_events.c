// SPDX-License-Identifier: GPL-2.0-only
/* rust_binder_events.c
 *
 * Rust Binder tracepoints.
 *
 * Copyright 2024 Google LLC
 */

#include <linux/rust_binder.h>

#define CREATE_TRACE_POINTS
#define CREATE_RUST_TRACE_POINTS
#include <trace/events/rust_binder.h>
