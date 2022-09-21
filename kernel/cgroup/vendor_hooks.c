// SPDX-License-Identifier: GPL-2.0-only
/* vendor_hook.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2022 Google LLC
 */
#ifndef __GENKSYMS__
#include "cgroup-internal.h"
#else
/*
 * Needed to preserve CRC for cgroup-related hooks
 */
#include <linux/cpufreq.h>
#include <../drivers/gpio/gpiolib.h>
#endif

#define CREATE_TRACE_POINTS
#include <trace/hooks/vendor_hooks.h>
#include <linux/tracepoint.h>
#include <trace/hooks/cgroup.h>

/*
 * Export tracepoints that act as a bare tracehook (ie: have no trace event
 * associated with them) to allow external modules to probe them.
 */
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_cgroup_set_task);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_cpuset_fork);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_cgroup_force_kthread_migration);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_refrigerator);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_cgroup_attach);

/*
 * For type visibility
 */
const struct cgroup_taskset *GKI_struct_cgroup_taskset;
EXPORT_SYMBOL_GPL(GKI_struct_cgroup_taskset);
