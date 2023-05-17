/* SPDX-License-Identifier: GPL-2.0 */
/* rust_binder_internal.h
 *
 * This file contains internal data structures used by Rust Binder. Mostly,
 * these are type definitions used only by binderfs or things that Rust Binder
 * define and export to binderfs.
 *
 * It does not include things exported by binderfs to Rust Binder since this
 * file is not included as input to bindgen.
 *
 * Copyright (C) 2024 Google LLC.
 */

#ifndef _LINUX_RUST_BINDER_INTERNAL_H
#define _LINUX_RUST_BINDER_INTERNAL_H

#include <linux/seq_file.h>
#include <uapi/linux/android/binder.h>
#include <uapi/linux/android/binderfs.h>

/*
 * The internal data types in the Rust Binder driver are opaque to C, so we use
 * void pointer typedefs for these types.
 */
typedef void *rust_binder_device;

int rust_binder_stats_show(struct seq_file *m, void *unused);
int rust_binder_state_show(struct seq_file *m, void *unused);
int rust_binder_transactions_show(struct seq_file *m, void *unused);
int rust_binder_transaction_log_show(struct seq_file *m, void *unused);

extern const struct file_operations rust_binder_fops;
rust_binder_device rust_binder_new_device(char *name);
void rust_binder_remove_device(rust_binder_device device);

/**
 * struct binder_device - information about a binder device node
 * @hlist:          list of binder devices (only used for devices requested via
 *                  CONFIG_ANDROID_BINDER_DEVICES)
 * @miscdev:        information about a binder character device node
 * @binderfs_inode: This is the inode of the root dentry of the super block
 *                  belonging to a binderfs mount.
 */
struct binder_device {
	struct hlist_node hlist;
	struct miscdevice miscdev;
	struct inode *binderfs_inode;
	refcount_t ref;
};

/**
 * binderfs_mount_opts - mount options for binderfs
 * @max: maximum number of allocatable binderfs binder devices
 * @stats_mode: enable binder stats in binderfs.
 */
struct binderfs_mount_opts {
	int max;
	int stats_mode;
};

/**
 * binderfs_info - information about a binderfs mount
 * @ipc_ns:         The ipc namespace the binderfs mount belongs to.
 * @control_dentry: This records the dentry of this binderfs mount
 *                  binder-control device.
 * @root_uid:       uid that needs to be used when a new binder device is
 *                  created.
 * @root_gid:       gid that needs to be used when a new binder device is
 *                  created.
 * @mount_opts:     The mount options in use.
 * @device_count:   The current number of allocated binder devices.
 * @proc_log_dir:   Pointer to the directory dentry containing process-specific
 *                  logs.
 */
struct binderfs_info {
	struct ipc_namespace *ipc_ns;
	struct dentry *control_dentry;
	kuid_t root_uid;
	kgid_t root_gid;
	struct binderfs_mount_opts mount_opts;
	int device_count;
	struct dentry *proc_log_dir;
};

#endif /* _LINUX_RUST_BINDER_INTERNAL_H */
