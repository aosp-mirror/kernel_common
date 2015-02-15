/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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

#ifndef MDSS_DEBUG_H
#define MDSS_DEBUG_H

#include "mdss.h"
#include "mdss_mdp_trace.h"

#define MISR_POLL_SLEEP		2000
#define MISR_POLL_TIMEOUT	32000
#define MISR_CRC_BATCH_CFG	0x101

#define ATRACE_END(name) trace_tracing_mark_write(current->tgid, name, 0)
#define ATRACE_BEGIN(name) trace_tracing_mark_write(current->tgid, name, 1)
#define ATRACE_FUNC() ATRACE_BEGIN(__func__)

#define ATRACE_INT(name, value) \
	trace_mdp_trace_counter(current->tgid, name, value)

#ifdef CONFIG_DEBUG_FS
int mdss_debugfs_init(struct mdss_data_type *mdata);
int mdss_debugfs_remove(struct mdss_data_type *mdata);
int mdss_debug_register_base(const char *name, void __iomem *base,
				    size_t max_offset);
int mdss_misr_set(struct mdss_data_type *mdata, struct mdp_misr *req,
			struct mdss_mdp_ctl *ctl);
int mdss_misr_get(struct mdss_data_type *mdata, struct mdp_misr *resp,
			struct mdss_mdp_ctl *ctl);
void mdss_misr_crc_collect(struct mdss_data_type *mdata, int block_id);
#else
static inline int mdss_debugfs_init(struct mdss_data_type *mdata) { return 0; }
static inline int mdss_debugfs_remove(struct mdss_data_type *mdata)
{ return 0; }
static inline int mdss_debug_register_base(const char *name, void __iomem *base,
					size_t max_offset) { return 0; }
static inline int mdss_misr_set(struct mdss_data_type *mdata,
					struct mdp_misr *req,
					struct mdss_mdp_ctl *ctl)
{ return 0; }
static inline int mdss_misr_get(struct mdss_data_type *mdata,
					struct mdp_misr *resp,
					struct mdss_mdp_ctl *ctl)
{ return 0; }
static inline void mdss_misr_crc_collect(struct mdss_data_type *mdata,
						int block_id) { }
#endif
#endif 
