/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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
#ifndef _IPC_LOGGING_H
#define _IPC_LOGGING_H

#include <mach/msm_ipc_logging.h>

#define IPC_LOG_VERSION 0x0001
#define IPC_LOG_MAX_CONTEXT_NAME_LEN 20

/**
 * struct ipc_log_page_header - Individual log page header
 *
 * @magic: Magic number (used for log extraction)
 * @nmagic: Inverse of magic number (used for log extraction)
 * @page_num: Index of page (0.. N - 1) (note top bit is always set)
 * @read_offset:  Read offset in page
 * @write_offset: Write offset in page (or 0xFFFF if full)
 * @log_id: ID of logging context that owns this page
 * @start_time:  Scheduler clock for first write time in page
 * @end_time:  Scheduler clock for last write time in page
 * @ctx_offset:  Signed offset from page to the logging context.  Used to
 *               optimize ram-dump extraction.
 *
 * @list:  Linked list of pages that make up a log
 * @nd_read_offset:  Non-destructive read offset used for debugfs
 *
 * The first part of the structure defines data that is used to extract the
 * logs from a memory dump and elements in this section should not be changed
 * or re-ordered.  New local data structures can be added to the end of the
 * structure since they will be ignored by the extraction tool.
 */
struct ipc_log_page_header {
	uint32_t magic;
	uint32_t nmagic;
	uint32_t page_num;
	uint16_t read_offset;
	uint16_t write_offset;
	uint64_t log_id;
	uint64_t start_time;
	uint64_t end_time;
	int64_t ctx_offset;

	/* add local data structures after this point */
	struct list_head list;
	uint16_t nd_read_offset;
};

/**
 * struct ipc_log_page - Individual log page
 *
 * @hdr: Log page header
 * @data: Log data
 *
 * Each log consists of 1 to N log pages.  Data size is adjusted to always fit
 * the structure into a single kernel page.
 */
struct ipc_log_page {
	struct ipc_log_page_header hdr;
	char data[PAGE_SIZE - sizeof(struct ipc_log_page_header)];
};

/**
 * struct ipc_log_context - main logging context
 *
 * @magic:  Magic number (used for log extraction)
 * @nmagic:  Inverse of magic number (used for log extraction)
 * @version:  IPC Logging version of log format
 * @user_version:  Version number for user-defined messages
 * @header_size:  Size of the log header which is used to determine the offset
 *                of ipc_log_page::data
 * @log_id:  Log ID (assigned when log is created)
 * @name:  Name of the log used to uniquely identify the log during extraction
 *
 * @list:  List of log contexts (struct ipc_log_context)
 * @page_list:  List of log pages (struct ipc_log_page)
 * @first_page:  First page in list of logging pages
 * @last_page:  Last page in list of logging pages
 * @write_page:  Current write page
 * @read_page:  Current read page (for internal reads)
 * @nd_read_page:  Current debugfs extraction page (non-destructive)
 *
 * @write_avail:  Number of bytes available to write in all pages
 * @dent:  Debugfs node for run-time log extraction
 * @dfunc_info_list:  List of deserialization functions
 * @context_lock_lhb1:  Lock for entire structure
 * @read_avail:  Completed when new data is added to the log
 */
struct ipc_log_context {
	uint32_t magic;
	uint32_t nmagic;
	uint32_t version;
	uint16_t user_version;
	uint16_t header_size;
	uint64_t log_id;
	char name[IPC_LOG_MAX_CONTEXT_NAME_LEN];

	/* add local data structures after this point */
	struct list_head list;
	struct list_head page_list;
	struct ipc_log_page *first_page;
	struct ipc_log_page *last_page;
	struct ipc_log_page *write_page;
	struct ipc_log_page *read_page;
	struct ipc_log_page *nd_read_page;

	uint32_t write_avail;
	struct dentry *dent;
	struct list_head dfunc_info_list;
	spinlock_t context_lock_lhb1;
	struct completion read_avail;
};

struct dfunc_info {
	struct list_head list;
	int type;
	void (*dfunc) (struct encode_context *, struct decode_context *);
};

enum {
	TSV_TYPE_INVALID,
	TSV_TYPE_TIMESTAMP,
	TSV_TYPE_POINTER,
	TSV_TYPE_INT32,
	TSV_TYPE_BYTE_ARRAY,
};

enum {
	OUTPUT_DEBUGFS,
};

#define IPC_LOG_CONTEXT_MAGIC_NUM 0x25874452
#define IPC_LOGGING_MAGIC_NUM 0x52784425
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define IS_MSG_TYPE(x) (((x) > TSV_TYPE_MSG_START) && \
			((x) < TSV_TYPE_MSG_END))
#define MAX_MSG_DECODED_SIZE (MAX_MSG_SIZE*4)

#if (defined(CONFIG_DEBUG_FS))
void check_and_create_debugfs(void);

void create_ctx_debugfs(struct ipc_log_context *ctxt,
			const char *mod_name);
#else
void check_and_create_debugfs(void)
{
}

void create_ctx_debugfs(struct ipc_log_context *ctxt, const char *mod_name)
{
}
#endif

#endif
