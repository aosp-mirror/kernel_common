/* Copyright (c) 2010-2013, The Linux Foundation. All rights reserved.
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
#include <linux/memory_alloc.h>
#include <linux/delay.h>
#include <mach/iommu_domains.h>
#include <mach/subsystem_restart.h>
#include "vcd_ddl_utils.h"
#include "vcd_ddl.h"
#include "vcd_res_tracker_api.h"

struct time_data {
	unsigned int ddl_t1;
	unsigned int ddl_ttotal;
	unsigned int ddl_count;
};
static struct time_data proc_time[MAX_TIME_DATA];
#define DDL_MSG_TIME(x...) printk(KERN_DEBUG x)

#ifdef DDL_BUF_LOG
static void ddl_print_buffer(struct ddl_context *ddl_context,
	struct ddl_buf_addr *buf, u32 idx, u8 *str);
static void ddl_print_port(struct ddl_context *ddl_context,
	struct ddl_buf_addr *buf);
static void ddl_print_buffer_port(struct ddl_context *ddl_context,
	struct ddl_buf_addr *buf, u32 idx, u8 *str);
#endif
void *ddl_pmem_alloc(struct ddl_buf_addr *addr, size_t sz, u32 alignment)
{
	u32 alloc_size, offset = 0 ;
	struct ddl_context *ddl_context;
	unsigned long iova = 0;
	unsigned long buffer_size = 0;
	unsigned long *kernel_vaddr = NULL;
	int ret = 0;
	ion_phys_addr_t phyaddr = 0;
	size_t len = 0;
	int rc = 0;
	DBG_PMEM("\n%s() IN: Requested alloc size(%u)", __func__, (u32)sz);
	if (!addr) {
		DDL_MSG_ERROR("\n%s() Invalid Parameters", __func__);
		goto bail_out;
	}
	ddl_context = ddl_get_context();
	res_trk_set_mem_type(addr->mem_type);
	alloc_size = (sz + alignment);
	if (res_trk_get_enable_ion()) {
		if (!ddl_context->video_ion_client)
			ddl_context->video_ion_client =
				res_trk_get_ion_client();
		if (!ddl_context->video_ion_client) {
			DDL_MSG_ERROR("%s() :DDL ION Client Invalid handle\n",
						 __func__);
			goto bail_out;
		}
		alloc_size = (alloc_size+4095) & ~4095;
		addr->alloc_handle = ion_alloc(
		ddl_context->video_ion_client, alloc_size, SZ_4K,
			res_trk_get_mem_type(), res_trk_get_ion_flags());
		if (IS_ERR_OR_NULL(addr->alloc_handle)) {
			DDL_MSG_ERROR("%s() :DDL ION alloc failed\n",
						 __func__);
			goto bail_out;
		}
		kernel_vaddr = (unsigned long *) ion_map_kernel(
					ddl_context->video_ion_client,
					addr->alloc_handle);
		if (IS_ERR_OR_NULL(kernel_vaddr)) {
				DDL_MSG_ERROR("%s() :DDL ION map failed\n",
							 __func__);
				goto free_ion_alloc;
		}
		addr->virtual_base_addr = (u8 *) kernel_vaddr;
		if (res_trk_check_for_sec_session()) {
			rc = ion_phys(ddl_context->video_ion_client,
				addr->alloc_handle, &phyaddr,
					&len);
			if (rc || !phyaddr) {
				DDL_MSG_ERROR(
				"%s():DDL ION client physical failed\n",
				__func__);
				goto unmap_ion_alloc;
			}
			addr->alloced_phys_addr = phyaddr;
		} else {
			ret = ion_map_iommu(ddl_context->video_ion_client,
					addr->alloc_handle,
					VIDEO_DOMAIN,
					VIDEO_MAIN_POOL,
					SZ_4K,
					0,
					&iova,
					&buffer_size,
					0, 0);
			if (ret || !iova) {
				DDL_MSG_ERROR(
				"%s():DDL ION ion map iommu failed, ret = %d iova = 0x%lx\n",
					__func__, ret, iova);
				goto unmap_ion_alloc;
			}
			addr->alloced_phys_addr = (phys_addr_t) iova;
		}
		if (!addr->alloced_phys_addr) {
			DDL_MSG_ERROR("%s():DDL ION client physical failed\n",
						__func__);
			goto unmap_ion_alloc;
		}
		addr->mapped_buffer = NULL;
		addr->physical_base_addr = (u8 *) addr->alloced_phys_addr;
		addr->align_physical_addr = (u8 *) DDL_ALIGN((u32)
			addr->physical_base_addr, alignment);
		offset = (u32)(addr->align_physical_addr -
				addr->physical_base_addr);
		addr->align_virtual_addr = addr->virtual_base_addr + offset;
		addr->buffer_size = alloc_size;
	} else {
		pr_err("ION must be enabled.");
		goto bail_out;
	}
	return addr->virtual_base_addr;
unmap_ion_alloc:
	ion_unmap_kernel(ddl_context->video_ion_client,
		addr->alloc_handle);
	addr->virtual_base_addr = NULL;
	addr->alloced_phys_addr = (phys_addr_t)NULL;
free_ion_alloc:
	ion_free(ddl_context->video_ion_client,
		addr->alloc_handle);
	addr->alloc_handle = NULL;
bail_out:
	return NULL;
}

void ddl_pmem_free(struct ddl_buf_addr *addr)
{
	struct ddl_context *ddl_context;
	ddl_context = ddl_get_context();
	if (!addr) {
		pr_err("%s() invalid args\n", __func__);
		return;
	}
	if (ddl_context->video_ion_client) {
		if (!IS_ERR_OR_NULL(addr->alloc_handle)) {
			ion_unmap_kernel(ddl_context->video_ion_client,
					addr->alloc_handle);
			if (!res_trk_check_for_sec_session()) {
				ion_unmap_iommu(ddl_context->video_ion_client,
					addr->alloc_handle,
					VIDEO_DOMAIN,
					VIDEO_MAIN_POOL);
			}
			ion_free(ddl_context->video_ion_client,
				addr->alloc_handle);
			}
	}
	memset(addr, 0, sizeof(struct ddl_buf_addr));
}

#ifdef DDL_BUF_LOG

static void ddl_print_buffer(struct ddl_context *ddl_context,
	struct ddl_buf_addr *buf, u32 idx, u8 *str)
{
	struct ddl_buf_addr *base_ram;
	s32  offset;
	size_t sz, KB = 0;

	base_ram = &ddl_context->dram_base_a;
	offset = (s32) DDL_ADDR_OFFSET(*base_ram, *buf);
	sz = buf->buffer_size;
	if (sz > 0) {
		if (!(sz % 1024)) {
			sz /= 1024;
			KB++;
			if (!(sz % 1024)) {
				sz /= 1024;
				KB++;
			}
		}
	}
	DDL_MSG_LOW("\n%12s [%2d]:  0x%08x [0x%04x],  0x%08x(%d%s),  %s",
		str, idx, (u32) buf->align_physical_addr,
		(offset > 0) ? offset : 0, buf->buffer_size, sz,
		((2 == KB) ? "MB" : (1 == KB) ? "KB" : ""),
		(((u32) buf->virtual_base_addr) ? "Alloc" : ""));
}

static void ddl_print_port(struct ddl_context *ddl_context,
	struct ddl_buf_addr *buf)
{
	struct ddl_buf_addr *a = &ddl_context->dram_base_a;
	struct ddl_buf_addr *b = &ddl_context->dram_base_b;

	if (!buf->align_physical_addr || !buf->buffer_size)
		return;
	if (buf->align_physical_addr >= a->align_physical_addr &&
		buf->align_physical_addr + buf->buffer_size <=
		a->align_physical_addr + a->buffer_size)
		DDL_MSG_LOW(" -A [0x%x]-", DDL_ADDR_OFFSET(*a, *buf));
	else if (buf->align_physical_addr >= b->align_physical_addr &&
		buf->align_physical_addr + buf->buffer_size <=
		b->align_physical_addr + b->buffer_size)
		DDL_MSG_LOW(" -B [0x%x]-", DDL_ADDR_OFFSET(*b, *buf));
	else
		DDL_MSG_LOW(" -?-");
}

static void ddl_print_buffer_port(struct ddl_context *ddl_context,
	struct ddl_buf_addr *buf, u32 idx, u8 *str)
{
	DDL_MSG_LOW("\n");
	ddl_print_buffer(ddl_context, buf, idx, str);
	ddl_print_port(ddl_context, buf);
}

void ddl_list_buffers(struct ddl_client_context *ddl)
{
	struct ddl_context *ddl_context;
	u32 i;

	ddl_context = ddl->ddl_context;
	DDL_MSG_LOW("\n\n");
	DDL_MSG_LOW("\n      Buffer     :     Start    [offs],      Size    \
	(Size),     Alloc/Port");
	DDL_MSG_LOW("\n-------------------------------------------------------\
	-------------------------");
	ddl_print_buffer(ddl_context, &ddl_context->dram_base_a, 0,
		"dram_base_a");
	ddl_print_buffer(ddl_context, &ddl_context->dram_base_b, 0,
		"dram_base_b");
	if (ddl->codec_data.hdr.decoding) {
		struct ddl_dec_buffers  *dec_bufs =
			&ddl->codec_data.decoder.hw_bufs;
		for (i = 0; i < 32; i++)
			ddl_print_buffer_port(ddl_context,
				&dec_bufs->h264Mv[i], i, "h264Mv");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->h264Vert_nb_mv, 0, "h264Vert_nb_mv");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->h264Nb_ip, 0, "h264Nb_ip");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->nb_dcac, 0, "nb_dcac");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->upnb_mv, 0, "upnb_mv");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->sub_anchor_mv, 0, "sub_anchor_mv");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->overlay_xform, 0, "overlay_xform");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->bit_plane3, 0, "bit_plane3");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->bit_plane2, 0, "bit_plane2");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->bit_plane1, 0, "bit_plane1");
		ddl_print_buffer_port(ddl_context,
			dec_bufs->stx_parser, 0, "stx_parser");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->desc, 0, "desc");
		ddl_print_buffer_port(ddl_context,
			&dec_bufs->context, 0, "context");
	} else {
		struct ddl_enc_buffers  *enc_bufs =
			&ddl->codec_data.encoder.hw_bufs;

		for (i = 0; i < 4; i++)
			ddl_print_buffer_port(ddl_context,
				&enc_bufs->dpb_y[i], i, "dpb_y");
		for (i = 0; i < 4; i++)
			ddl_print_buffer_port(ddl_context,
				&enc_bufs->dpb_c[i], i, "dpb_c");
		ddl_print_buffer_port(ddl_context, &enc_bufs->mv, 0, "mv");
		ddl_print_buffer_port(ddl_context,
			&enc_bufs->col_zero, 0, "col_zero");
		ddl_print_buffer_port(ddl_context, &enc_bufs->md, 0, "md");
		ddl_print_buffer_port(ddl_context,
			&enc_bufs->pred, 0, "pred");
		ddl_print_buffer_port(ddl_context,
			&enc_bufs->nbor_info, 0, "nbor_info");
		ddl_print_buffer_port(ddl_context,
			&enc_bufs->acdc_coef, 0, "acdc_coef");
		ddl_print_buffer_port(ddl_context,
			&enc_bufs->context, 0, "context");
	}
}
#endif

u32 ddl_fw_init(struct ddl_buf_addr *dram_base)
{
	u8 *dest_addr;
	dest_addr = DDL_GET_ALIGNED_VITUAL(*dram_base);
	DDL_MSG_LOW("FW Addr / FW Size : %x/%d", (u32)vidc_video_codec_fw,
		vidc_video_codec_fw_size);
	if (res_trk_check_for_sec_session() && res_trk_is_cp_enabled()) {
		if (res_trk_enable_footswitch()) {
			pr_err("Failed to enable footswitch");
			return false;
		}
		if (res_trk_enable_iommu_clocks()) {
			res_trk_disable_footswitch();
			pr_err("Failed to enable iommu clocks\n");
			return false;
		}
		dram_base->pil_cookie = subsystem_get("vidc");
		if (res_trk_disable_iommu_clocks())
			pr_err("Failed to disable iommu clocks\n");
		if (IS_ERR_OR_NULL(dram_base->pil_cookie)) {
			res_trk_disable_footswitch();
			pr_err("subsystem_get failed\n");
			return false;
		}
	} else {
		if (vidc_video_codec_fw_size > dram_base->buffer_size ||
				!vidc_video_codec_fw)
			return false;
		memcpy(dest_addr, vidc_video_codec_fw,
				vidc_video_codec_fw_size);
	}
	return true;
}

void ddl_fw_release(struct ddl_buf_addr *dram_base)
{
	void *cookie = dram_base->pil_cookie;
	if (res_trk_is_cp_enabled() &&
		res_trk_check_for_sec_session()) {
		res_trk_close_secure_session();
		if (IS_ERR_OR_NULL(cookie)) {
			pr_err("Invalid params");
		return;
	}
	if (res_trk_enable_footswitch()) {
		pr_err("Failed to enable footswitch");
		return;
	}
	if (res_trk_enable_iommu_clocks()) {
		res_trk_disable_footswitch();
		pr_err("Failed to enable iommu clocks\n");
		return;
	}
	subsystem_put(cookie);
	if (res_trk_disable_iommu_clocks())
		pr_err("Failed to disable iommu clocks\n");
	if (res_trk_disable_footswitch())
		pr_err("Failed to disable footswitch\n");
	} else {
	if (res_trk_check_for_sec_session())
		res_trk_close_secure_session();
		res_trk_release_fw_addr();
	}
}

void ddl_set_core_start_time(const char *func_name, u32 index)
{
	u32 act_time;
	struct timeval ddl_tv;
	struct time_data *time_data = &proc_time[index];
	do_gettimeofday(&ddl_tv);
	act_time = (ddl_tv.tv_sec * 1000) + (ddl_tv.tv_usec / 1000);
	if (!time_data->ddl_t1) {
		time_data->ddl_t1 = act_time;
		DDL_MSG_LOW("\n%s(): Start Time (%u)", func_name, act_time);
	} else if (vidc_msg_timing) {
		DDL_MSG_LOW("\n%s(): Timer already started! St(%u) Act(%u)",
			func_name, time_data->ddl_t1, act_time);
	}
}

void ddl_calc_core_proc_time(const char *func_name, u32 index,
			struct ddl_client_context *ddl)
{
	struct time_data *time_data = &proc_time[index];
	struct ddl_decoder_data *decoder = NULL;
	if (time_data->ddl_t1) {
		int ddl_t2;
		struct timeval ddl_tv;
		do_gettimeofday(&ddl_tv);
		ddl_t2 = (ddl_tv.tv_sec * 1000) + (ddl_tv.tv_usec / 1000);
		time_data->ddl_ttotal += (ddl_t2 - time_data->ddl_t1);
		time_data->ddl_count++;
		if (vidc_msg_timing) {
			DDL_MSG_TIME("\n%s(): cnt(%u) End Time (%u)"
				"Diff(%u) Avg(%u)",
				func_name, time_data->ddl_count, ddl_t2,
				ddl_t2 - time_data->ddl_t1,
				time_data->ddl_ttotal/time_data->ddl_count);
		}
		if ((index == DEC_OP_TIME) && (time_data->ddl_count > 2) &&
					(time_data->ddl_count < 6)) {
			decoder = &(ddl->codec_data.decoder);
			decoder->dec_time_sum = decoder->dec_time_sum +
				ddl_t2 - time_data->ddl_t1;
			if (time_data->ddl_count == 5)
				decoder->avg_dec_time =
					decoder->dec_time_sum / 3;
		}
		time_data->ddl_t1 = 0;
	}
}

void ddl_calc_core_proc_time_cnt(const char *func_name, u32 index, u32 count)
{
	struct time_data *time_data = &proc_time[index];
	if (time_data->ddl_t1) {
		int ddl_t2;
		struct timeval ddl_tv;
		do_gettimeofday(&ddl_tv);
		ddl_t2 = (ddl_tv.tv_sec * 1000) + (ddl_tv.tv_usec / 1000);
		time_data->ddl_ttotal += (ddl_t2 - time_data->ddl_t1);
		time_data->ddl_count += count;
		DDL_MSG_TIME("\n%s(): cnt(%u) End Time (%u) Diff(%u) Avg(%u)",
			func_name, time_data->ddl_count, ddl_t2,
			ddl_t2 - time_data->ddl_t1,
			time_data->ddl_ttotal/time_data->ddl_count);
		time_data->ddl_t1 = 0;
	}
}

void ddl_update_core_start_time(const char *func_name, u32 index)
{
	u32 act_time;
	struct timeval ddl_tv;
	struct time_data *time_data = &proc_time[index];
	do_gettimeofday(&ddl_tv);
	act_time = (ddl_tv.tv_sec * 1000) + (ddl_tv.tv_usec / 1000);
	time_data->ddl_t1 = act_time;
	DDL_MSG_LOW("\n%s(): Start time updated Act(%u)",
				func_name, act_time);
}

void ddl_reset_core_time_variables(u32 index)
{
	proc_time[index].ddl_t1 = 0;
	proc_time[index].ddl_ttotal = 0;
	proc_time[index].ddl_count = 0;
}

int ddl_get_core_decode_proc_time(u32 *ddl_handle)
{
	int avg_time = 0;
	struct ddl_client_context *ddl =
		(struct ddl_client_context *) ddl_handle;
	avg_time = ddl_vidc_decode_get_avg_time(ddl);
	return avg_time;
}

void ddl_reset_avg_dec_time(u32 *ddl_handle)
{
	struct ddl_client_context *ddl =
		(struct ddl_client_context *) ddl_handle;
	ddl_vidc_decode_reset_avg_time(ddl);
}
