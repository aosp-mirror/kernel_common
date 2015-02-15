/* Copyright (c) 2002,2008-2011,2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _KGSL_DEBUGFS_H
#define _KGSL_DEBUGFS_H

struct kgsl_device;
struct kgsl_process_private;

#ifdef CONFIG_DEBUG_FS
void kgsl_core_debugfs_init(void);
void kgsl_core_debugfs_close(void);

int kgsl_device_debugfs_init(struct kgsl_device *device);

extern struct dentry *kgsl_debugfs_dir;
static inline struct dentry *kgsl_get_debugfs_dir(void)
{
	return kgsl_debugfs_dir;
}

int kgsl_process_init_debugfs(struct kgsl_process_private *);
#else
static inline void kgsl_core_debugfs_init(void) { }
static inline void kgsl_device_debugfs_init(struct kgsl_device *device) { }
static inline void kgsl_core_debugfs_close(void) { }
static inline struct dentry *kgsl_get_debugfs_dir(void) { return NULL; }
static inline int kgsl_process_init_debugfs(struct kgsl_process_private *priv)
{
	return 0;
}

#endif

#endif
