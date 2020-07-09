// SPDX-License-Identifier: GPL-2.0-only
/* vendor_hook.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */

#define CREATE_TRACE_POINTS
#include <trace/hooks/vendor_hooks.h>
#include <linux/tracepoint.h>
#include <trace/hooks/mpam.h>
#include <trace/hooks/debug.h>
#include <trace/hooks/iommu.h>

/*
 * Export tracepoints that act as a bare tracehook (ie: have no trace event
 * associated with them) to allow external modules to probe them.
 */
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_mpam_set);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ipi_stop);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_iommu_setup_dma_ops);
