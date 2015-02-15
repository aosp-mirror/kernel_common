/*******************************************************************************
 * Filename:  target_core_rd.c
 *
 * This file contains the Storage Engine <-> Ramdisk transport
 * specific functions.
 *
 * Copyright (c) 2003, 2004, 2005 PyX Technologies, Inc.
 * Copyright (c) 2005, 2006, 2007 SBE, Inc.
 * Copyright (c) 2007-2010 Rising Tide Systems
 * Copyright (c) 2008-2010 Linux-iSCSI.org
 *
 * Nicholas A. Bellinger <nab@kernel.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 ******************************************************************************/

#include <linux/string.h>
#include <linux/parser.h>
#include <linux/timer.h>
#include <linux/blkdev.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <scsi/scsi.h>
#include <scsi/scsi_host.h>

#include <target/target_core_base.h>
#include <target/target_core_backend.h>

#include "target_core_rd.h"

static struct se_subsystem_api rd_mcp_template;

/*	rd_attach_hba(): (Part of se_subsystem_api_t template)
 *
 *
 */
static int rd_attach_hba(struct se_hba *hba, u32 host_id)
{
	struct rd_host *rd_host;

	rd_host = kzalloc(sizeof(struct rd_host), GFP_KERNEL);
	if (!rd_host) {
		pr_err("Unable to allocate memory for struct rd_host\n");
		return -ENOMEM;
	}

	rd_host->rd_host_id = host_id;

	hba->hba_ptr = rd_host;

	pr_debug("CORE_HBA[%d] - TCM Ramdisk HBA Driver %s on"
		" Generic Target Core Stack %s\n", hba->hba_id,
		RD_HBA_VERSION, TARGET_CORE_MOD_VERSION);
	pr_debug("CORE_HBA[%d] - Attached Ramdisk HBA: %u to Generic"
		" MaxSectors: %u\n", hba->hba_id,
		rd_host->rd_host_id, RD_MAX_SECTORS);

	return 0;
}

static void rd_detach_hba(struct se_hba *hba)
{
	struct rd_host *rd_host = hba->hba_ptr;

	pr_debug("CORE_HBA[%d] - Detached Ramdisk HBA: %u from"
		" Generic Target Core\n", hba->hba_id, rd_host->rd_host_id);

	kfree(rd_host);
	hba->hba_ptr = NULL;
}

static u32 rd_release_sgl_table(struct rd_dev *rd_dev, struct rd_dev_sg_table *sg_table,
				 u32 sg_table_count)
{
	struct page *pg;
	struct scatterlist *sg;
	u32 i, j, page_count = 0, sg_per_table;

	for (i = 0; i < sg_table_count; i++) {
		sg = sg_table[i].sg_table;
		sg_per_table = sg_table[i].rd_sg_count;

		for (j = 0; j < sg_per_table; j++) {
			pg = sg_page(&sg[j]);
			if (pg) {
				__free_page(pg);
				page_count++;
			}
		}
		kfree(sg);
	}

	kfree(sg_table);
	return page_count;
}

static void rd_release_device_space(struct rd_dev *rd_dev)
{
	u32 page_count;

	if (!rd_dev->sg_table_array || !rd_dev->sg_table_count)
		return;

	page_count = rd_release_sgl_table(rd_dev, rd_dev->sg_table_array,
					  rd_dev->sg_table_count);

	pr_debug("CORE_RD[%u] - Released device space for Ramdisk"
		" Device ID: %u, pages %u in %u tables total bytes %lu\n",
		rd_dev->rd_host->rd_host_id, rd_dev->rd_dev_id, page_count,
		rd_dev->sg_table_count, (unsigned long)page_count * PAGE_SIZE);

	rd_dev->sg_table_array = NULL;
	rd_dev->sg_table_count = 0;
}


/*	rd_build_device_space():
 *
 *
 */
static int rd_allocate_sgl_table(struct rd_dev *rd_dev, struct rd_dev_sg_table *sg_table,
				 u32 total_sg_needed, unsigned char init_payload)
{
	u32 i = 0, j, page_offset = 0, sg_per_table;
	u32 max_sg_per_table = (RD_MAX_ALLOCATION_SIZE /
				sizeof(struct scatterlist));
	struct page *pg;
	struct scatterlist *sg;
	unsigned char *p;

	while (total_sg_needed) {
		sg_per_table = (total_sg_needed > max_sg_per_table) ?
			max_sg_per_table : total_sg_needed;

		sg = kzalloc(sg_per_table * sizeof(struct scatterlist),
				GFP_KERNEL);
		if (!sg) {
			pr_err("Unable to allocate scatterlist array"
				" for struct rd_dev\n");
			return -ENOMEM;
		}

		sg_init_table(sg, sg_per_table);

		sg_table[i].sg_table = sg;
		sg_table[i].rd_sg_count = sg_per_table;
		sg_table[i].page_start_offset = page_offset;
		sg_table[i++].page_end_offset = (page_offset + sg_per_table)
						- 1;

		for (j = 0; j < sg_per_table; j++) {
			pg = alloc_pages(GFP_KERNEL, 0);
			if (!pg) {
				pr_err("Unable to allocate scatterlist"
					" pages for struct rd_dev_sg_table\n");
				return -ENOMEM;
			}
			sg_assign_page(&sg[j], pg);
			sg[j].length = PAGE_SIZE;

			p = kmap(pg);
			memset(p, init_payload, PAGE_SIZE);
			kunmap(pg);
		}

		page_offset += sg_per_table;
		total_sg_needed -= sg_per_table;
	}

	return 0;
}

static int rd_build_device_space(struct rd_dev *rd_dev)
{
	struct rd_dev_sg_table *sg_table;
	u32 sg_tables, total_sg_needed;
	u32 max_sg_per_table = (RD_MAX_ALLOCATION_SIZE /
				sizeof(struct scatterlist));
	int rc;

	if (rd_dev->rd_page_count <= 0) {
		pr_err("Illegal page count: %u for Ramdisk device\n",
		       rd_dev->rd_page_count);
		return -EINVAL;
	}

	/* Don't need backing pages for NULLIO */
	if (rd_dev->rd_flags & RDF_NULLIO)
		return 0;

	total_sg_needed = rd_dev->rd_page_count;

	sg_tables = (total_sg_needed / max_sg_per_table) + 1;

	sg_table = kzalloc(sg_tables * sizeof(struct rd_dev_sg_table), GFP_KERNEL);
	if (!sg_table) {
		pr_err("Unable to allocate memory for Ramdisk"
		       " scatterlist tables\n");
		return -ENOMEM;
	}

	rd_dev->sg_table_array = sg_table;
	rd_dev->sg_table_count = sg_tables;

	rc = rd_allocate_sgl_table(rd_dev, sg_table, total_sg_needed, 0x00);
	if (rc)
		return rc;

	pr_debug("CORE_RD[%u] - Built Ramdisk Device ID: %u space of"
		 " %u pages in %u tables\n", rd_dev->rd_host->rd_host_id,
		 rd_dev->rd_dev_id, rd_dev->rd_page_count,
		 rd_dev->sg_table_count);

	return 0;
}

static void *rd_allocate_virtdevice(
	struct se_hba *hba,
	const char *name,
	int rd_direct)
{
	struct rd_dev *rd_dev;
	struct rd_host *rd_host = hba->hba_ptr;

	rd_dev = kzalloc(sizeof(struct rd_dev), GFP_KERNEL);
	if (!rd_dev) {
		pr_err("Unable to allocate memory for struct rd_dev\n");
		return NULL;
	}

	rd_dev->rd_host = rd_host;
	rd_dev->rd_direct = rd_direct;

	return rd_dev;
}

static void *rd_MEMCPY_allocate_virtdevice(struct se_hba *hba, const char *name)
{
	return rd_allocate_virtdevice(hba, name, 0);
}

/*	rd_create_virtdevice():
 *
 *
 */
static struct se_device *rd_create_virtdevice(
	struct se_hba *hba,
	struct se_subsystem_dev *se_dev,
	void *p,
	int rd_direct)
{
	struct se_device *dev;
	struct se_dev_limits dev_limits;
	struct rd_dev *rd_dev = p;
	struct rd_host *rd_host = hba->hba_ptr;
	int dev_flags = 0, ret;
	char prod[16], rev[4];

	memset(&dev_limits, 0, sizeof(struct se_dev_limits));

	ret = rd_build_device_space(rd_dev);
	if (ret < 0)
		goto fail;

	snprintf(prod, 16, "RAMDISK-%s", (rd_dev->rd_direct) ? "DR" : "MCP");
	snprintf(rev, 4, "%s", (rd_dev->rd_direct) ? RD_DR_VERSION :
						RD_MCP_VERSION);

	dev_limits.limits.logical_block_size = RD_BLOCKSIZE;
	dev_limits.limits.max_hw_sectors = RD_MAX_SECTORS;
	dev_limits.limits.max_sectors = RD_MAX_SECTORS;
	dev_limits.hw_queue_depth = RD_MAX_DEVICE_QUEUE_DEPTH;
	dev_limits.queue_depth = RD_DEVICE_QUEUE_DEPTH;

	dev = transport_add_device_to_core_hba(hba,
			&rd_mcp_template, se_dev, dev_flags, rd_dev,
			&dev_limits, prod, rev);
	if (!dev)
		goto fail;

	rd_dev->rd_dev_id = rd_host->rd_host_dev_id_count++;
	rd_dev->rd_queue_depth = dev->queue_depth;

	pr_debug("CORE_RD[%u] - Added TCM %s Ramdisk Device ID: %u of"
		" %u pages in %u tables, %lu total bytes\n",
		rd_host->rd_host_id, (!rd_dev->rd_direct) ? "MEMCPY" :
		"DIRECT", rd_dev->rd_dev_id, rd_dev->rd_page_count,
		rd_dev->sg_table_count,
		(unsigned long)(rd_dev->rd_page_count * PAGE_SIZE));

	return dev;

fail:
	rd_release_device_space(rd_dev);
	return ERR_PTR(ret);
}

static struct se_device *rd_MEMCPY_create_virtdevice(
	struct se_hba *hba,
	struct se_subsystem_dev *se_dev,
	void *p)
{
	return rd_create_virtdevice(hba, se_dev, p, 0);
}

/*	rd_free_device(): (Part of se_subsystem_api_t template)
 *
 *
 */
static void rd_free_device(void *p)
{
	struct rd_dev *rd_dev = p;

	rd_release_device_space(rd_dev);
	kfree(rd_dev);
}

static inline struct rd_request *RD_REQ(struct se_task *task)
{
	return container_of(task, struct rd_request, rd_task);
}

static struct se_task *
rd_alloc_task(unsigned char *cdb)
{
	struct rd_request *rd_req;

	rd_req = kzalloc(sizeof(struct rd_request), GFP_KERNEL);
	if (!rd_req) {
		pr_err("Unable to allocate struct rd_request\n");
		return NULL;
	}

	return &rd_req->rd_task;
}

/*	rd_get_sg_table():
 *
 *
 */
static struct rd_dev_sg_table *rd_get_sg_table(struct rd_dev *rd_dev, u32 page)
{
	u32 i;
	struct rd_dev_sg_table *sg_table;

	for (i = 0; i < rd_dev->sg_table_count; i++) {
		sg_table = &rd_dev->sg_table_array[i];
		if ((sg_table->page_start_offset <= page) &&
		    (sg_table->page_end_offset >= page))
			return sg_table;
	}

	pr_err("Unable to locate struct rd_dev_sg_table for page: %u\n",
			page);

	return NULL;
}

static int rd_MEMCPY(struct rd_request *req, u32 read_rd)
{
	struct se_task *task = &req->rd_task;
	struct rd_dev *dev = req->rd_task.task_se_cmd->se_dev->dev_ptr;
	struct rd_dev_sg_table *table;
	struct scatterlist *rd_sg;
	struct sg_mapping_iter m;
	u32 rd_offset = req->rd_offset;
	u32 src_len;

	table = rd_get_sg_table(dev, req->rd_page);
	if (!table)
		return -EINVAL;

	rd_sg = &table->sg_table[req->rd_page - table->page_start_offset];

	pr_debug("RD[%u]: %s LBA: %llu, Size: %u Page: %u, Offset: %u\n",
			dev->rd_dev_id, read_rd ? "Read" : "Write",
			task->task_lba, req->rd_size, req->rd_page,
			rd_offset);

	src_len = PAGE_SIZE - rd_offset;
	sg_miter_start(&m, task->task_sg, task->task_sg_nents,
			read_rd ? SG_MITER_TO_SG : SG_MITER_FROM_SG);
	while (req->rd_size) {
		u32 len;
		void *rd_addr;

		sg_miter_next(&m);
		len = min((u32)m.length, src_len);
		m.consumed = len;

		rd_addr = sg_virt(rd_sg) + rd_offset;

		if (read_rd)
			memcpy(m.addr, rd_addr, len);
		else
			memcpy(rd_addr, m.addr, len);

		req->rd_size -= len;
		if (!req->rd_size)
			continue;

		src_len -= len;
		if (src_len) {
			rd_offset += len;
			continue;
		}

		/* rd page completed, next one please */
		req->rd_page++;
		rd_offset = 0;
		src_len = PAGE_SIZE;
		if (req->rd_page <= table->page_end_offset) {
			rd_sg++;
			continue;
		}

		table = rd_get_sg_table(dev, req->rd_page);
		if (!table) {
			sg_miter_stop(&m);
			return -EINVAL;
		}

		/* since we increment, the first sg entry is correct */
		rd_sg = table->sg_table;
	}
	sg_miter_stop(&m);
	return 0;
}

/*	rd_MEMCPY_do_task(): (Part of se_subsystem_api_t template)
 *
 *
 */
static int rd_MEMCPY_do_task(struct se_task *task)
{
	struct se_device *dev = task->task_se_cmd->se_dev;
	struct rd_request *req = RD_REQ(task);
	u64 tmp;
	int ret;

	tmp = task->task_lba * dev->se_sub_dev->se_dev_attrib.block_size;
	req->rd_offset = do_div(tmp, PAGE_SIZE);
	req->rd_page = tmp;
	req->rd_size = task->task_size;

	ret = rd_MEMCPY(req, task->task_data_direction == DMA_FROM_DEVICE);
	if (ret != 0)
		return ret;

	task->task_scsi_status = GOOD;
	transport_complete_task(task, 1);
	return 0;
}

/*	rd_free_task(): (Part of se_subsystem_api_t template)
 *
 *
 */
static void rd_free_task(struct se_task *task)
{
	kfree(RD_REQ(task));
}

enum {
	Opt_rd_pages, Opt_err
};

static match_table_t tokens = {
	{Opt_rd_pages, "rd_pages=%d"},
	{Opt_err, NULL}
};

static ssize_t rd_set_configfs_dev_params(
	struct se_hba *hba,
	struct se_subsystem_dev *se_dev,
	const char *page,
	ssize_t count)
{
	struct rd_dev *rd_dev = se_dev->se_dev_su_ptr;
	char *orig, *ptr, *opts;
	substring_t args[MAX_OPT_ARGS];
	int ret = 0, arg, token;

	opts = kstrdup(page, GFP_KERNEL);
	if (!opts)
		return -ENOMEM;

	orig = opts;

	while ((ptr = strsep(&opts, ",\n")) != NULL) {
		if (!*ptr)
			continue;

		token = match_token(ptr, tokens, args);
		switch (token) {
		case Opt_rd_pages:
			match_int(args, &arg);
			rd_dev->rd_page_count = arg;
			pr_debug("RAMDISK: Referencing Page"
				" Count: %u\n", rd_dev->rd_page_count);
			rd_dev->rd_flags |= RDF_HAS_PAGE_COUNT;
			break;
		default:
			break;
		}
	}

	kfree(orig);
	return (!ret) ? count : ret;
}

static ssize_t rd_check_configfs_dev_params(struct se_hba *hba, struct se_subsystem_dev *se_dev)
{
	struct rd_dev *rd_dev = se_dev->se_dev_su_ptr;

	if (!(rd_dev->rd_flags & RDF_HAS_PAGE_COUNT)) {
		pr_debug("Missing rd_pages= parameter\n");
		return -EINVAL;
	}

	return 0;
}

static ssize_t rd_show_configfs_dev_params(
	struct se_hba *hba,
	struct se_subsystem_dev *se_dev,
	char *b)
{
	struct rd_dev *rd_dev = se_dev->se_dev_su_ptr;
	ssize_t bl = sprintf(b, "TCM RamDisk ID: %u  RamDisk Makeup: %s\n",
			rd_dev->rd_dev_id, (rd_dev->rd_direct) ?
			"rd_direct" : "rd_mcp");
	bl += sprintf(b + bl, "        PAGES/PAGE_SIZE: %u*%lu"
			"  SG_table_count: %u\n", rd_dev->rd_page_count,
			PAGE_SIZE, rd_dev->sg_table_count);
	return bl;
}

static u32 rd_get_device_rev(struct se_device *dev)
{
	return SCSI_SPC_2; /* Returns SPC-3 in Initiator Data */
}

static u32 rd_get_device_type(struct se_device *dev)
{
	return TYPE_DISK;
}

static sector_t rd_get_blocks(struct se_device *dev)
{
	struct rd_dev *rd_dev = dev->dev_ptr;
	unsigned long long blocks_long = ((rd_dev->rd_page_count * PAGE_SIZE) /
			dev->se_sub_dev->se_dev_attrib.block_size) - 1;

	return blocks_long;
}

static struct se_subsystem_api rd_mcp_template = {
	.name			= "rd_mcp",
	.transport_type		= TRANSPORT_PLUGIN_VHBA_VDEV,
	.attach_hba		= rd_attach_hba,
	.detach_hba		= rd_detach_hba,
	.allocate_virtdevice	= rd_MEMCPY_allocate_virtdevice,
	.create_virtdevice	= rd_MEMCPY_create_virtdevice,
	.free_device		= rd_free_device,
	.alloc_task		= rd_alloc_task,
	.do_task		= rd_MEMCPY_do_task,
	.free_task		= rd_free_task,
	.check_configfs_dev_params = rd_check_configfs_dev_params,
	.set_configfs_dev_params = rd_set_configfs_dev_params,
	.show_configfs_dev_params = rd_show_configfs_dev_params,
	.get_device_rev		= rd_get_device_rev,
	.get_device_type	= rd_get_device_type,
	.get_blocks		= rd_get_blocks,
};

int __init rd_module_init(void)
{
	int ret;

	ret = transport_subsystem_register(&rd_mcp_template);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

void rd_module_exit(void)
{
	transport_subsystem_release(&rd_mcp_template);
}
