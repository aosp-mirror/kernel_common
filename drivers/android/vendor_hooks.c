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

#include <trace/hooks/cpuidle.h>
#include <trace/hooks/mpam.h>
#include <trace/hooks/wqlockup.h>
#include <trace/hooks/debug.h>
#include <trace/hooks/sysrqcrash.h>
#include <trace/hooks/printk.h>
#include <trace/hooks/epoch.h>
#include <trace/hooks/cpufreq.h>
#include <trace/hooks/ufshcd.h>
#include <trace/hooks/cgroup.h>
#include <trace/hooks/sys.h>
#include <trace/hooks/iommu.h>
#include <trace/hooks/net.h>
#include <trace/hooks/pm_domain.h>
#include <trace/hooks/cpuidle_psci.h>
#include <trace/hooks/vmscan.h>
#include <trace/hooks/creds.h>
#include <trace/hooks/syscall_check.h>
#include <trace/hooks/timer.h>

/*
 * Export tracepoints that act as a bare tracehook (ie: have no trace event
 * associated with them) to allow external modules to probe them.
 */
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_cpu_idle_enter);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_cpu_idle_exit);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_mpam_set);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_wq_lockup_pool);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ipi_stop);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_sysrq_crash);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_printk_hotplug);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_show_suspend_epoch_val);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_show_resume_epoch_val);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_freq_table_limits);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_cpufreq_online);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_cpu_cgroup_attach);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_cpu_cgroup_online);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ufs_fill_prdt);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ufs_prepare_command);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ufs_update_sysfs);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ufs_send_command);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ufs_compl_command);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_cgroup_set_task);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_syscall_prctl_finished);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ufs_send_uic_command);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ufs_send_tm_command);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ufs_check_int_errors);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ufs_update_sdev);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_cgroup_attach);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_iommu_setup_dma_ops);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_iommu_iovad_alloc_iova);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_iommu_iovad_free_iova);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_ptype_head);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_allow_domain_state);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_cpuidle_psci_enter);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_cpuidle_psci_exit);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_set_balance_anon_file_reclaim);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_show_max_freq);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_commit_creds);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_exit_creds);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_override_creds);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_rvh_revert_creds);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_check_mmap_file);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_check_bpf_syscall);
EXPORT_TRACEPOINT_SYMBOL_GPL(android_vh_timer_calc_index);
