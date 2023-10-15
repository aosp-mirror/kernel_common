// SPDX-License-Identifier: GPL-2.0-only
/*
 * Kernel-based Virtual Machine driver for Linux
 *
 * Copyright 2016 Red Hat, Inc. and/or its affiliates.
 */
#include <linux/kvm_host.h>
#include <linux/debugfs.h>
#include "lapic.h"

static int vcpu_get_timer_advance_ns(void *data, u64 *val)
{
	struct kvm_vcpu *vcpu = (struct kvm_vcpu *) data;
	*val = vcpu->arch.apic->lapic_timer.timer_advance_ns;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vcpu_timer_advance_ns_fops, vcpu_get_timer_advance_ns, NULL, "%llu\n");

static int vcpu_get_tsc_offset(void *data, u64 *val)
{
	struct kvm_vcpu *vcpu = (struct kvm_vcpu *) data;
	*val = vcpu->arch.tsc_offset;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vcpu_tsc_offset_fops, vcpu_get_tsc_offset, NULL, "%lld\n");

static int vcpu_get_tsc_scaling_ratio(void *data, u64 *val)
{
	struct kvm_vcpu *vcpu = (struct kvm_vcpu *) data;
	*val = vcpu->arch.tsc_scaling_ratio;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vcpu_tsc_scaling_fops, vcpu_get_tsc_scaling_ratio, NULL, "%llu\n");

static int vcpu_get_tsc_scaling_frac_bits(void *data, u64 *val)
{
	*val = kvm_tsc_scaling_ratio_frac_bits;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vcpu_tsc_scaling_frac_fops, vcpu_get_tsc_scaling_frac_bits, NULL, "%llu\n");

#ifdef CONFIG_PARAVIRT_SCHED_KVM
#include <linux/sched.h>

static int vcpu_pv_sched_get_enabled(void *data, u64 *val)
{
	struct kvm_vcpu *vcpu = (struct kvm_vcpu *) data;
	*val = kvm_vcpu_sched_enabled(vcpu);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vcpu_pv_sched_enabled_fops, vcpu_pv_sched_get_enabled, NULL, "%llu\n");

static int vcpu_get_boosted(void *data, u64 *val)
{
	struct kvm_vcpu *vcpu = (struct kvm_vcpu *) data;
	*val = kvm_arch_vcpu_is_boosted(&vcpu->arch);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vcpu_boosted_fops, vcpu_get_boosted, NULL, "%llu\n");

static int get_kerncs_prio(void *data, u64 *val)
{
	struct kvm_vcpu *vcpu = (struct kvm_vcpu *) data;
	*val = kvm_arch_vcpu_kerncs_prio(&vcpu->arch);
	return 0;
}

static int set_kerncs_prio(void *data, u64 val)
{
	struct kvm_vcpu *vcpu = (struct kvm_vcpu *) data;

	return kvm_arch_vcpu_set_kerncs_prio(&vcpu->arch, val);
}

DEFINE_SIMPLE_ATTRIBUTE(kerncs_prio_fops, get_kerncs_prio,
		set_kerncs_prio, "%llu\n");

static int kerncs_policy_show(struct seq_file *m, void *data)
{
	char *policy;
	struct kvm_vcpu *vcpu = (struct kvm_vcpu *) m->private;

	if (kvm_arch_vcpu_kerncs_policy(&vcpu->arch) == SCHED_FIFO)
		policy = "SCHED_FIFO";
	else
		policy = "SCHED_RR";
	seq_printf(m, "%s\n", policy);

	return 0;
}

static ssize_t
kerncs_policy_write(struct file *filp, const char __user *ubuf,
		size_t cnt, loff_t *ppos)
{
	int ret;
	char *cmp;
	int policy;
	char buf[12];
	struct inode *inode;
	struct kvm_vcpu *vcpu;

	if (cnt > 12)
		cnt = 12;

	if (copy_from_user(&buf, ubuf, cnt))
		return -EFAULT;

	cmp = strstrip(buf);
	if (!strcmp(cmp, "SCHED_FIFO"))
		policy = SCHED_FIFO;
	else if (!strcmp(cmp, "SCHED_RR"))
		policy = SCHED_RR;
	else
		return -EINVAL;

	inode = file_inode(filp);
	inode_lock(inode);
	vcpu = (struct kvm_vcpu *)inode->i_private;
	ret = kvm_arch_vcpu_set_kerncs_policy(&vcpu->arch, policy);
	inode_unlock(inode);

	if (ret)
		return ret;

	*ppos += cnt;
	return cnt;
}

static int kerncs_policy_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, kerncs_policy_show, inode->i_private);
}

static const struct file_operations kerncs_policy_fops = {
	.open		= kerncs_policy_open,
	.write		= kerncs_policy_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

void kvm_arch_create_vcpu_debugfs(struct kvm_vcpu *vcpu, struct dentry *debugfs_dentry)
{
	debugfs_create_file("tsc-offset", 0444, debugfs_dentry, vcpu,
			    &vcpu_tsc_offset_fops);

	if (lapic_in_kernel(vcpu))
		debugfs_create_file("lapic_timer_advance_ns", 0444,
				    debugfs_dentry, vcpu,
				    &vcpu_timer_advance_ns_fops);

	if (kvm_has_tsc_control) {
		debugfs_create_file("tsc-scaling-ratio", 0444,
				    debugfs_dentry, vcpu,
				    &vcpu_tsc_scaling_fops);
		debugfs_create_file("tsc-scaling-ratio-frac-bits", 0444,
				    debugfs_dentry, vcpu,
				    &vcpu_tsc_scaling_frac_fops);
	}

#ifdef CONFIG_PARAVIRT_SCHED_KVM
	debugfs_create_file("pv_sched_enabled", 0444,
			    debugfs_dentry, vcpu,
			    &vcpu_pv_sched_enabled_fops);
	debugfs_create_file("pv_sched_boosted", 0444,
			    debugfs_dentry, vcpu,
			    &vcpu_boosted_fops);
	debugfs_create_file("kerncs_policy", 0644,
			    debugfs_dentry, vcpu,
			    &kerncs_policy_fops);
	debugfs_create_file("kerncs_prio", 0644,
			    debugfs_dentry, vcpu,
			    &kerncs_prio_fops);
#endif
}
