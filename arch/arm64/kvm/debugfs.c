// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021 Red Hat Inc.
 */

#include <linux/kvm_host.h>
#include <linux/debugfs.h>

#include <kvm/arm_arch_timer.h>

static int vcpu_get_cntv_offset(void *data, u64 *val)
{
	struct kvm_vcpu *vcpu = (struct kvm_vcpu *)data;

	*val = timer_get_offset(vcpu_vtimer(vcpu));

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vcpu_cntvoff_fops, vcpu_get_cntv_offset, NULL, "%lld\n");

void kvm_arch_create_vcpu_debugfs(struct kvm_vcpu *vcpu, struct dentry *debugfs_dentry)
{
	debugfs_create_file("cntvoff", 0444, debugfs_dentry, vcpu, &vcpu_cntvoff_fops);
}
