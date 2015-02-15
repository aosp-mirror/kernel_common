/* Copyright (c) 2010-2013, Linux Foundation. All rights reserved.
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
#include <linux/msm_ion.h>
#include <mach/msm_memtypes.h>
#include "vcd_ddl.h"
#include "vcd_ddl_shared_mem.h"
#include "vcd_res_tracker_api.h"

struct ddl_context *ddl_get_context(void)
{
	static struct ddl_context ddl_context;
	return &ddl_context;
}

#if DDL_MSG_LOG
s8 *ddl_get_state_string(enum ddl_client_state client_state)
{
	s8 *ptr;

	switch (client_state) {
	case DDL_CLIENT_INVALID:
		ptr = "INVALID        ";
	break;
	case DDL_CLIENT_OPEN:
		ptr = "OPEN   ";
	break;
	case DDL_CLIENT_WAIT_FOR_CHDONE:
		ptr = "WAIT_FOR_CHDONE       ";
	break;
	case DDL_CLIENT_WAIT_FOR_INITCODEC:
		ptr = "WAIT_FOR_INITCODEC    ";
	break;
	case DDL_CLIENT_WAIT_FOR_INITCODECDONE:
		ptr = "WAIT_FOR_INITCODECDONE";
	break;
	case DDL_CLIENT_WAIT_FOR_DPB:
		ptr = "WAIT_FOR_DPB   ";
	break;
	case DDL_CLIENT_WAIT_FOR_DPBDONE:
		ptr = "WAIT_FOR_DPBDONE";
	break;
	case DDL_CLIENT_WAIT_FOR_FRAME:
		ptr = "WAIT_FOR_FRAME ";
	break;
	case DDL_CLIENT_WAIT_FOR_FRAME_DONE:
		ptr = "WAIT_FOR_FRAME_DONE   ";
	break;
	case DDL_CLIENT_WAIT_FOR_EOS_DONE:
		ptr = "WAIT_FOR_EOS_DONE     ";
	break;
	case DDL_CLIENT_WAIT_FOR_CHEND:
		ptr = "WAIT_FOR_CHEND ";
	break;
	case DDL_CLIENT_FATAL_ERROR:
		ptr = "FATAL_ERROR";
	break;
	default:
		ptr = "UNKNOWN        ";
	break;
	}
	return ptr;
}
#endif

u32 ddl_client_transact(u32 operation,
	struct ddl_client_context **pddl_client)
{
	struct ddl_context *ddl_context;
	u32 ret_status = VCD_ERR_FAIL;
	s32 counter;

	ddl_context = ddl_get_context();
	switch (operation) {
	case DDL_FREE_CLIENT:
		ret_status = VCD_ERR_MAX_CLIENT;
		for (counter = 0; (counter < VCD_MAX_NO_CLIENT) &&
			(ret_status == VCD_ERR_MAX_CLIENT); ++counter) {
			if (*pddl_client == ddl_context->ddl_clients
				[counter]) {
					kfree(*pddl_client);
					*pddl_client = NULL;
					ddl_context->ddl_clients[counter]
						= NULL;
				ret_status = VCD_S_SUCCESS;
			}
		}
	break;
	case DDL_GET_CLIENT:
		ret_status = VCD_ERR_MAX_CLIENT;
		for (counter = (VCD_MAX_NO_CLIENT - 1); (counter >= 0) &&
			(ret_status == VCD_ERR_MAX_CLIENT); --counter) {
			if (!ddl_context->ddl_clients[counter]) {
				*pddl_client =
					(struct ddl_client_context *)
					kmalloc(sizeof(struct
					ddl_client_context), GFP_KERNEL);
				if (!*pddl_client)
					ret_status = VCD_ERR_ALLOC_FAIL;
				else {
					memset(*pddl_client, 0,
						sizeof(struct
						ddl_client_context));
					ddl_context->ddl_clients
						[counter] = *pddl_client;
					(*pddl_client)->ddl_context =
						ddl_context;
					ret_status = VCD_S_SUCCESS;
				}
			}
		}
	break;
	case DDL_INIT_CLIENTS:
		for (counter = 0; counter < VCD_MAX_NO_CLIENT; ++counter)
			ddl_context->ddl_clients[counter] = NULL;
		ret_status = VCD_S_SUCCESS;
	break;
	case DDL_ACTIVE_CLIENT:
		for (counter = 0; counter < VCD_MAX_NO_CLIENT;
			++counter) {
			if (ddl_context->ddl_clients[counter]) {
				ret_status = VCD_S_SUCCESS;
				break;
			}
		}
	break;
	default:
		ret_status = VCD_ERR_ILLEGAL_PARM;
	break;
	}
	return ret_status;
}

u32 ddl_decoder_dpb_transact(struct ddl_decoder_data *decoder,
	struct ddl_frame_data_tag  *in_out_frame, u32 operation)
{
	struct ddl_frame_data_tag *found_frame = NULL;
	struct ddl_mask *dpb_mask = &decoder->dpb_mask;
	u32 vcd_status = VCD_S_SUCCESS, loopc;

	switch (operation) {
	case DDL_DPB_OP_MARK_BUSY:
	case DDL_DPB_OP_MARK_FREE:
		for (loopc = 0; !found_frame && loopc <
			decoder->dp_buf.no_of_dec_pic_buf; ++loopc) {
			if (in_out_frame->vcd_frm.physical ==
				decoder->dp_buf.dec_pic_buffers[loopc].
				vcd_frm.physical) {
				found_frame = &(decoder->dp_buf.
					dec_pic_buffers[loopc]);
			break;
			}
		}
		if (found_frame) {
			if (operation == DDL_DPB_OP_MARK_BUSY) {
				dpb_mask->hw_mask &=
					(~(u32)(0x1 << loopc));
				*in_out_frame = *found_frame;
			} else if (operation == DDL_DPB_OP_MARK_FREE) {
				dpb_mask->client_mask |= (0x1 << loopc);
				*found_frame = *in_out_frame;
				if ((decoder->meta_data_enable_flag) &&
				    (in_out_frame->vcd_frm.buff_ion_handle)) {
					struct ddl_context *ddl_context =
						ddl_get_context();
					unsigned long *vaddr =
						(unsigned long *)((u32)
						in_out_frame->vcd_frm.virtual +
						decoder->meta_data_offset);
					DDL_MSG_LOW("%s: Cache clean: vaddr"\
						" (%p), offset %u, size %u",
						__func__,
						in_out_frame->vcd_frm.virtual,
						decoder->meta_data_offset,
						decoder->suffix);
					msm_ion_do_cache_op(
						ddl_context->video_ion_client,
						in_out_frame->vcd_frm.\
						buff_ion_handle,
						vaddr,
						(unsigned long)decoder->suffix,
						ION_IOC_CLEAN_CACHES);
				}
			}
		} else {
			in_out_frame->vcd_frm.physical = NULL;
			in_out_frame->vcd_frm.virtual = NULL;
			vcd_status = VCD_ERR_BAD_POINTER;
			DDL_MSG_ERROR("BUF_NOT_FOUND");
		}
	break;
	case DDL_DPB_OP_SET_MASK:
		dpb_mask->hw_mask |= dpb_mask->client_mask;
		dpb_mask->client_mask = 0;
	break;
	case DDL_DPB_OP_INIT:
	{
		u32 dpb_size;
		dpb_size = (!decoder->meta_data_offset) ?
		decoder->dp_buf.dec_pic_buffers[0].vcd_frm.alloc_len :
			decoder->meta_data_offset;
	}
	break;
	case DDL_DPB_OP_RETRIEVE:
	{
		u32 position;
		if (dpb_mask->client_mask) {
			position = 0x1;
			for (loopc = 0; loopc <
				decoder->dp_buf.no_of_dec_pic_buf &&
				!found_frame; ++loopc) {
				if (dpb_mask->client_mask & position) {
					found_frame = &decoder->dp_buf.
						dec_pic_buffers[loopc];
					dpb_mask->client_mask &=
						~(position);
				}
				position <<= 1;
			}
		} else if (dpb_mask->hw_mask) {
			position = 0x1;
			for (loopc = 0; loopc <
				decoder->dp_buf.no_of_dec_pic_buf &&
				!found_frame; ++loopc) {
				if (dpb_mask->hw_mask & position) {
					found_frame = &decoder->dp_buf.
						dec_pic_buffers[loopc];
					dpb_mask->hw_mask &= ~(position);
				}
				position <<= 1;
			}
		}
		if (found_frame)
			*in_out_frame = *found_frame;
		else {
			in_out_frame->vcd_frm.physical = NULL;
			in_out_frame->vcd_frm.virtual = NULL;
		}
	}
	break;
	default:
	break;
	}
	return vcd_status;
}

u32 ddl_decoder_dpb_init(struct ddl_client_context *ddl)
{
	struct ddl_context *ddl_context = ddl->ddl_context;
	struct ddl_decoder_data *decoder = &ddl->codec_data.decoder;
	struct ddl_dec_buffers *dec_buffers = &decoder->hw_bufs;
	struct ddl_frame_data_tag *frame;
	u32 luma[DDL_MAX_BUFFER_COUNT], chroma[DDL_MAX_BUFFER_COUNT];
	u32 mv[DDL_MAX_BUFFER_COUNT], luma_size, i, dpb;
	frame = &decoder->dp_buf.dec_pic_buffers[0];
	luma_size = ddl_get_yuv_buf_size(decoder->frame_size.width,
			decoder->frame_size.height, DDL_YUV_BUF_TYPE_TILE);
	dpb = decoder->dp_buf.no_of_dec_pic_buf;
	DDL_MSG_LOW("%s Decoder num DPB buffers = %u Luma Size = %u",
				 __func__, dpb, luma_size);
	if (dpb > DDL_MAX_BUFFER_COUNT)
		dpb = DDL_MAX_BUFFER_COUNT;
	for (i = 0; i < dpb; i++) {
		if (!(res_trk_check_for_sec_session()) &&
			frame[i].vcd_frm.virtual) {
			if (luma_size <= frame[i].vcd_frm.alloc_len) {
				memset(frame[i].vcd_frm.virtual,
					 0x10101010, luma_size);
				memset(frame[i].vcd_frm.virtual + luma_size,
					 0x80808080,
					frame[i].vcd_frm.alloc_len - luma_size);
				if (frame[i].vcd_frm.ion_flag
					== ION_FLAG_CACHED) {
					msm_ion_do_cache_op(
					ddl_context->video_ion_client,
					frame[i].vcd_frm.buff_ion_handle,
					(unsigned long *)frame[i].
					vcd_frm.virtual,
					(unsigned long)frame[i].
					vcd_frm.alloc_len,
					ION_IOC_CLEAN_INV_CACHES);
				}
			} else {
				DDL_MSG_ERROR("luma size error");
				return VCD_ERR_FAIL;
			}
		}

		luma[i] = DDL_OFFSET(ddl_context->dram_base_a.
			align_physical_addr, frame[i].vcd_frm.physical);
		chroma[i] = luma[i] + luma_size;
		DDL_MSG_LOW("%s Decoder Luma address = %x Chroma address = %x",
					__func__, luma[i], chroma[i]);
	}
	switch (decoder->codec.codec) {
	case VCD_CODEC_MPEG1:
	case VCD_CODEC_MPEG2:
		vidc_1080p_set_decode_recon_buffers(dpb, luma, chroma);
	break;
	case VCD_CODEC_DIVX_3:
	case VCD_CODEC_DIVX_4:
	case VCD_CODEC_DIVX_5:
	case VCD_CODEC_DIVX_6:
	case VCD_CODEC_XVID:
	case VCD_CODEC_MPEG4:
		vidc_1080p_set_decode_recon_buffers(dpb, luma, chroma);
		vidc_1080p_set_mpeg4_divx_decode_work_buffers(
		DDL_ADDR_OFFSET(ddl_context->dram_base_a,
			dec_buffers->nb_dcac),
		DDL_ADDR_OFFSET(ddl_context->dram_base_a,
			dec_buffers->upnb_mv),
		DDL_ADDR_OFFSET(ddl_context->dram_base_a,
			dec_buffers->sub_anchor_mv),
		DDL_ADDR_OFFSET(ddl_context->dram_base_a,
			dec_buffers->overlay_xform),
		DDL_ADDR_OFFSET(ddl_context->dram_base_a,
			dec_buffers->stx_parser));
	break;
	case VCD_CODEC_H263:
		vidc_1080p_set_decode_recon_buffers(dpb, luma, chroma);
		vidc_1080p_set_h263_decode_work_buffers(
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				dec_buffers->nb_dcac),
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				dec_buffers->upnb_mv),
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				dec_buffers->sub_anchor_mv),
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				dec_buffers->overlay_xform));
	break;
	case VCD_CODEC_VC1:
	case VCD_CODEC_VC1_RCV:
		vidc_1080p_set_decode_recon_buffers(dpb, luma, chroma);
		vidc_1080p_set_vc1_decode_work_buffers(
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				dec_buffers->nb_dcac),
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				dec_buffers->upnb_mv),
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				dec_buffers->sub_anchor_mv),
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				dec_buffers->overlay_xform),
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				dec_buffers->bit_plane1),
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				dec_buffers->bit_plane2),
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				dec_buffers->bit_plane3));
	break;
	case VCD_CODEC_H264:
		for (i = 0; i < dpb; i++)
			mv[i] = DDL_ADDR_OFFSET(ddl_context->dram_base_a,
					dec_buffers->h264_mv[i]);
		vidc_1080p_set_h264_decode_buffers(dpb,
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				dec_buffers->h264_vert_nb_mv),
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				dec_buffers->h264_nb_ip),
			luma, chroma, mv);
	break;
	default:
	break;
	}
	return VCD_S_SUCCESS;
}

void ddl_release_context_buffers(struct ddl_context *ddl_context)
{
	ddl_pmem_free(&ddl_context->metadata_shared_input);
	ddl_fw_release(&ddl_context->dram_base_a);
}

void ddl_release_client_internal_buffers(struct ddl_client_context *ddl)
{
	if (ddl->decoding) {
		struct ddl_decoder_data *decoder =
			&(ddl->codec_data.decoder);
		kfree(decoder->dp_buf.dec_pic_buffers);
		decoder->dp_buf.dec_pic_buffers = NULL;
		ddl_vidc_decode_dynamic_property(ddl, false);
		decoder->decode_config.sequence_header_len = 0;
		decoder->decode_config.sequence_header = NULL;
		decoder->dpb_mask.client_mask = 0;
		decoder->dpb_mask.hw_mask = 0;
		decoder->dp_buf.no_of_dec_pic_buf = 0;
		decoder->dynamic_prop_change = 0;
		ddl_free_dec_hw_buffers(ddl);
	} else {
		struct ddl_encoder_data *encoder =
			&(ddl->codec_data.encoder);
		ddl_pmem_free(&encoder->seq_header);
		ddl_pmem_free(&encoder->batch_frame.slice_batch_in);
		ddl_pmem_free(&encoder->batch_frame.slice_batch_out);
		ddl_vidc_encode_dynamic_property(ddl, false);
		encoder->dynamic_prop_change = 0;
		ddl_free_enc_hw_buffers(ddl);
		ddl_free_ltr_list(&encoder->ltr_control);
	}
	ddl_pmem_free(&ddl->shared_mem[0]);
	ddl_pmem_free(&ddl->shared_mem[1]);
}

u32 ddl_codec_type_transact(struct ddl_client_context *ddl,
	u32 remove, enum vcd_codec requested_codec)
{
	if (requested_codec > VCD_CODEC_VC1_RCV ||
		requested_codec < VCD_CODEC_H264)
		return false;
	if (!ddl->decoding && requested_codec != VCD_CODEC_MPEG4 &&
		requested_codec != VCD_CODEC_H264 &&
		requested_codec != VCD_CODEC_H263)
		return false;

	return true;
}

u32 ddl_take_command_channel(struct ddl_context *ddl_context,
	struct ddl_client_context *ddl, void *client_data)
{
	u32  status = true;

	if (!ddl_context->current_ddl[0]) {
		ddl_context->current_ddl[0] = ddl;
		ddl->client_data = client_data;
		ddl->command_channel = 0;
	} else if (!ddl_context->current_ddl[1]) {
		ddl_context->current_ddl[1] = ddl;
		ddl->client_data = client_data;
		ddl->command_channel = 1;
	} else
		status = false;
	if (status) {
		if (ddl_context->current_ddl[0] &&
			ddl_context->current_ddl[1])
			DDL_BUSY(ddl_context);
		else
			DDL_RUN(ddl_context);
	}
	return status;
}

void ddl_release_command_channel(struct ddl_context *ddl_context,
	u32 command_channel)
{
	ddl_context->current_ddl[command_channel]->client_data = NULL;
	ddl_context->current_ddl[command_channel] = NULL;
	if (!ddl_context->current_ddl[0] &&
		!ddl_context->current_ddl[1])
		DDL_IDLE(ddl_context);
	else
		DDL_RUN(ddl_context);
}

struct ddl_client_context *ddl_get_current_ddl_client_for_channel_id(
	struct ddl_context *ddl_context, u32 channel_id)
{
	struct ddl_client_context *ddl;

	if (ddl_context->current_ddl[0] && channel_id ==
		ddl_context->current_ddl[0]->command_channel)
		ddl = ddl_context->current_ddl[0];
	else if (ddl_context->current_ddl[1] && channel_id ==
		ddl_context->current_ddl[1]->command_channel)
		ddl = ddl_context->current_ddl[1];
	else {
		DDL_MSG_LOW("STATE-CRITICAL-FRMRUN");
		DDL_MSG_LOW("Unexpected channel ID = %d", channel_id);
		ddl = NULL;
	}
	return ddl;
}

struct ddl_client_context *ddl_get_current_ddl_client_for_command(
	struct ddl_context *ddl_context,
	enum ddl_cmd_state cmd_state)
{
	struct ddl_client_context *ddl;

	if (ddl_context->current_ddl[0] &&
		cmd_state == ddl_context->current_ddl[0]->cmd_state)
		ddl = ddl_context->current_ddl[0];
	else if (ddl_context->current_ddl[1] &&
		cmd_state == ddl_context->current_ddl[1]->cmd_state)
		ddl = ddl_context->current_ddl[1];
	else {
		DDL_MSG_LOW("STATE-CRITICAL-FRMRUN");
		DDL_MSG_ERROR("Error: Unexpected cmd_state = %d",
			cmd_state);
		ddl = NULL;
	}
	return ddl;
}

u32 ddl_get_yuv_buf_size(u32 width, u32 height, u32 format)
{
	u32 mem_size, width_round_up, height_round_up, align;

	width_round_up  = width;
	height_round_up = height;
	align = SZ_4K;

	if (format == DDL_YUV_BUF_TYPE_TILE) {
		width_round_up  = DDL_ALIGN(width, DDL_TILE_ALIGN_WIDTH);
		height_round_up = DDL_ALIGN(height, DDL_TILE_ALIGN_HEIGHT);
		align = DDL_TILE_MULTIPLY_FACTOR;
	}
	if (format == DDL_YUV_BUF_TYPE_LINEAR) {
		width_round_up = DDL_ALIGN(width, DDL_LINEAR_ALIGN_WIDTH);
		align = DDL_LINEAR_MULTIPLY_FACTOR;
	}
	mem_size = (width_round_up * height_round_up);
	mem_size = DDL_ALIGN(mem_size, align);
	return mem_size;
}
void ddl_free_dec_hw_buffers(struct ddl_client_context *ddl)
{
	struct ddl_dec_buffers *dec_bufs =
		&ddl->codec_data.decoder.hw_bufs;
	ddl_pmem_free(&dec_bufs->h264_nb_ip);
	ddl_pmem_free(&dec_bufs->h264_vert_nb_mv);
	ddl_pmem_free(&dec_bufs->nb_dcac);
	ddl_pmem_free(&dec_bufs->upnb_mv);
	ddl_pmem_free(&dec_bufs->sub_anchor_mv);
	ddl_pmem_free(&dec_bufs->overlay_xform);
	ddl_pmem_free(&dec_bufs->bit_plane3);
	ddl_pmem_free(&dec_bufs->bit_plane2);
	ddl_pmem_free(&dec_bufs->bit_plane1);
	ddl_pmem_free(&dec_bufs->stx_parser);
	ddl_pmem_free(&dec_bufs->desc);
	ddl_pmem_free(&dec_bufs->context);
	ddl_pmem_free(&dec_bufs->extnuserdata);
	memset(dec_bufs, 0, sizeof(struct ddl_dec_buffers));
}

void ddl_free_enc_hw_buffers(struct ddl_client_context *ddl)
{
	struct ddl_enc_buffers *enc_bufs =
		&ddl->codec_data.encoder.hw_bufs;
	u32 i;

	for (i = 0; i < enc_bufs->dpb_count; i++) {
		ddl_pmem_free(&enc_bufs->dpb_y[i]);
		ddl_pmem_free(&enc_bufs->dpb_c[i]);
	}
	ddl_pmem_free(&enc_bufs->mv);
	ddl_pmem_free(&enc_bufs->col_zero);
	ddl_pmem_free(&enc_bufs->md);
	ddl_pmem_free(&enc_bufs->pred);
	ddl_pmem_free(&enc_bufs->nbor_info);
	ddl_pmem_free(&enc_bufs->acdc_coef);
	ddl_pmem_free(&enc_bufs->context);
	memset(enc_bufs, 0, sizeof(struct ddl_enc_buffers));
}

u32 ddl_get_input_frame_from_pool(struct ddl_client_context *ddl,
	u8 *input_buffer_address)
{
	u32 vcd_status = VCD_S_SUCCESS, i, found = false;

	for (i = 0; i < DDL_MAX_NUM_IN_INPUTFRAME_POOL && !found; i++) {
		if (input_buffer_address ==
			ddl->input_frame_pool[i].vcd_frm.physical) {
			found = true;
			ddl->input_frame = ddl->input_frame_pool[i];
			memset(&ddl->input_frame_pool[i], 0,
				sizeof(struct ddl_frame_data_tag));
		}
	}
	if (!found)
		vcd_status = VCD_ERR_FAIL;

	return vcd_status;
}

u32 ddl_insert_input_frame_to_pool(struct ddl_client_context *ddl,
	struct ddl_frame_data_tag *ddl_input_frame)
{
	u32 vcd_status = VCD_S_SUCCESS, i, found = false;

	for (i = 0; i < DDL_MAX_NUM_IN_INPUTFRAME_POOL && !found; i++) {
		if (!ddl->input_frame_pool[i].vcd_frm.physical) {
			found = true;
			ddl->input_frame_pool[i] = *ddl_input_frame;
		}
	}
	if (!found)
		vcd_status = VCD_ERR_FAIL;

	return vcd_status;
}

void ddl_calc_dec_hw_buffers_size(enum vcd_codec codec, u32 width,
	u32 height, u32 dpb, struct ddl_dec_buffer_size *buf_size)
{
	u32 sz_dpb0 = 0, sz_dpb1 = 0, sz_mv = 0;
	u32 sz_luma = 0, sz_chroma = 0, sz_nb_dcac = 0, sz_upnb_mv = 0;
	u32 sz_sub_anchor_mv = 0, sz_overlap_xform = 0, sz_bit_plane3 = 0;
	u32 sz_bit_plane2 = 0, sz_bit_plane1 = 0, sz_stx_parser = 0;
	u32 sz_desc, sz_cpb, sz_context, sz_vert_nb_mv = 0, sz_nb_ip = 0;
	u32 sz_extnuserdata = 0;

	if (codec == VCD_CODEC_H264) {
		sz_mv = ddl_get_yuv_buf_size(width,
			height>>2, DDL_YUV_BUF_TYPE_TILE);
		sz_nb_ip = DDL_KILO_BYTE(32);
		sz_vert_nb_mv = DDL_KILO_BYTE(16);
	} else {
		if ((codec == VCD_CODEC_MPEG4) ||
			(codec == VCD_CODEC_DIVX_3) ||
			(codec == VCD_CODEC_DIVX_4) ||
			(codec == VCD_CODEC_DIVX_5) ||
			(codec == VCD_CODEC_DIVX_6) ||
			(codec == VCD_CODEC_XVID) ||
			(codec == VCD_CODEC_H263)) {
			sz_nb_dcac = DDL_KILO_BYTE(16);
			sz_upnb_mv = DDL_KILO_BYTE(68);
			sz_sub_anchor_mv = DDL_KILO_BYTE(136);
			sz_overlap_xform = DDL_KILO_BYTE(32);
			if (codec != VCD_CODEC_H263)
				sz_stx_parser = DDL_KILO_BYTE(68);
		} else if ((codec == VCD_CODEC_VC1) ||
			(codec == VCD_CODEC_VC1_RCV)) {
			sz_nb_dcac = DDL_KILO_BYTE(16);
			sz_upnb_mv = DDL_KILO_BYTE(68);
			sz_sub_anchor_mv = DDL_KILO_BYTE(136);
			sz_overlap_xform = DDL_KILO_BYTE(32);
			sz_bit_plane3 = DDL_KILO_BYTE(2);
			sz_bit_plane2 = DDL_KILO_BYTE(2);
			sz_bit_plane1 = DDL_KILO_BYTE(2);
		} else if (codec == VCD_CODEC_MPEG2)
			sz_extnuserdata = DDL_KILO_BYTE(2);
	}
	sz_desc = DDL_KILO_BYTE(128);
	sz_cpb = VCD_DEC_CPB_SIZE;
	if (codec == VCD_CODEC_H264)
		sz_context = DDL_FW_H264DEC_CONTEXT_SPACE_SIZE;
	else
		sz_context = DDL_FW_OTHER_CONTEXT_SPACE_SIZE;
	if (buf_size) {
		buf_size->sz_dpb0           = sz_dpb0;
		buf_size->sz_dpb1           = sz_dpb1;
		buf_size->sz_mv             = sz_mv;
		buf_size->sz_vert_nb_mv     = sz_vert_nb_mv;
		buf_size->sz_nb_ip          = sz_nb_ip;
		buf_size->sz_luma           = sz_luma;
		buf_size->sz_chroma         = sz_chroma;
		buf_size->sz_nb_dcac        = sz_nb_dcac;
		buf_size->sz_upnb_mv        = sz_upnb_mv;
		buf_size->sz_sub_anchor_mv  = sz_sub_anchor_mv;
		buf_size->sz_overlap_xform  = sz_overlap_xform;
		buf_size->sz_bit_plane3     = sz_bit_plane3;
		buf_size->sz_bit_plane2     = sz_bit_plane2;
		buf_size->sz_bit_plane1     = sz_bit_plane1;
		buf_size->sz_stx_parser     = sz_stx_parser;
		buf_size->sz_desc           = sz_desc;
		buf_size->sz_cpb            = sz_cpb;
		buf_size->sz_context        = sz_context;
		buf_size->sz_extnuserdata   = sz_extnuserdata;
	}
}

u32 ddl_allocate_dec_hw_buffers(struct ddl_client_context *ddl)
{
	struct ddl_dec_buffers *dec_bufs;
	struct ddl_dec_buffer_size buf_size;
	u32 status = VCD_S_SUCCESS, dpb = 0;
	u32 width = 0, height = 0;
	u8 *ptr;
	struct ddl_context *ddl_context = ddl->ddl_context;

	dec_bufs = &ddl->codec_data.decoder.hw_bufs;
	ddl_calc_dec_hw_buffers_size(ddl->codec_data.decoder.
		codec.codec, width, height, dpb, &buf_size);
	if (buf_size.sz_context > 0) {
		dec_bufs->context.mem_type = DDL_MM_MEM;
		ptr = ddl_pmem_alloc(&dec_bufs->context, buf_size.sz_context,
			DDL_KILO_BYTE(2));
		if (!ptr)
			goto fail_free_exit;
		else
			msm_ion_do_cache_op(ddl_context->video_ion_client,
					dec_bufs->context.alloc_handle,
					dec_bufs->context.virtual_base_addr,
					dec_bufs->context.buffer_size,
					ION_IOC_CLEAN_INV_CACHES);
	}
	if (buf_size.sz_nb_ip > 0) {
		dec_bufs->h264_nb_ip.mem_type = DDL_MM_MEM;
		ptr = ddl_pmem_alloc(&dec_bufs->h264_nb_ip, buf_size.sz_nb_ip,
			DDL_KILO_BYTE(2));
		if (!ptr)
			goto fail_free_exit;
	}
	if (buf_size.sz_vert_nb_mv > 0) {
		dec_bufs->h264_vert_nb_mv.mem_type = DDL_MM_MEM;
		ptr = ddl_pmem_alloc(&dec_bufs->h264_vert_nb_mv,
			buf_size.sz_vert_nb_mv, DDL_KILO_BYTE(2));
		if (!ptr)
			goto fail_free_exit;
	}
	if (buf_size.sz_nb_dcac > 0) {
		dec_bufs->nb_dcac.mem_type = DDL_MM_MEM;
		ptr = ddl_pmem_alloc(&dec_bufs->nb_dcac, buf_size.sz_nb_dcac,
			DDL_KILO_BYTE(2));
		if (!ptr)
			goto fail_free_exit;
	}
	if (buf_size.sz_upnb_mv > 0) {
		dec_bufs->upnb_mv.mem_type = DDL_MM_MEM;
		ptr = ddl_pmem_alloc(&dec_bufs->upnb_mv, buf_size.sz_upnb_mv,
			DDL_KILO_BYTE(2));
		if (!ptr)
			goto fail_free_exit;
	}
	if (buf_size.sz_sub_anchor_mv > 0) {
		dec_bufs->sub_anchor_mv.mem_type = DDL_MM_MEM;
		ptr = ddl_pmem_alloc(&dec_bufs->sub_anchor_mv,
			buf_size.sz_sub_anchor_mv, DDL_KILO_BYTE(2));
		if (!ptr)
			goto fail_free_exit;
	}
	if (buf_size.sz_overlap_xform > 0) {
		dec_bufs->overlay_xform.mem_type = DDL_MM_MEM;
		ptr = ddl_pmem_alloc(&dec_bufs->overlay_xform,
			buf_size.sz_overlap_xform, DDL_KILO_BYTE(2));
		if (!ptr)
			goto fail_free_exit;
	}
	if (buf_size.sz_bit_plane3 > 0) {
		dec_bufs->bit_plane3.mem_type = DDL_MM_MEM;
		ptr = ddl_pmem_alloc(&dec_bufs->bit_plane3,
			buf_size.sz_bit_plane3, DDL_KILO_BYTE(2));
		if (!ptr)
			goto fail_free_exit;
	}
	if (buf_size.sz_bit_plane2 > 0) {
		dec_bufs->bit_plane2.mem_type = DDL_MM_MEM;
		ptr = ddl_pmem_alloc(&dec_bufs->bit_plane2,
			buf_size.sz_bit_plane2, DDL_KILO_BYTE(2));
		if (!ptr)
			goto fail_free_exit;
	}
	if (buf_size.sz_bit_plane1 > 0) {
		dec_bufs->bit_plane1.mem_type = DDL_MM_MEM;
		ptr = ddl_pmem_alloc(&dec_bufs->bit_plane1,
			buf_size.sz_bit_plane1, DDL_KILO_BYTE(2));
		if (!ptr)
			goto fail_free_exit;
	}
	if (buf_size.sz_stx_parser > 0) {
		dec_bufs->stx_parser.mem_type = DDL_MM_MEM;
		ptr = ddl_pmem_alloc(&dec_bufs->stx_parser,
			buf_size.sz_stx_parser, DDL_KILO_BYTE(2));
		if (!ptr)
			goto fail_free_exit;
	}
	if (buf_size.sz_desc > 0) {
		dec_bufs->desc.mem_type = DDL_MM_MEM;
		ptr = ddl_pmem_alloc(&dec_bufs->desc, buf_size.sz_desc,
			DDL_KILO_BYTE(2));
		if (!ptr)
			goto fail_free_exit;
		else {
			if (!res_trk_check_for_sec_session()) {
				memset(dec_bufs->desc.align_virtual_addr,
					   0, buf_size.sz_desc);
				msm_ion_do_cache_op(
					ddl_context->video_ion_client,
					dec_bufs->desc.alloc_handle,
					dec_bufs->desc.virtual_base_addr,
					dec_bufs->desc.buffer_size,
					ION_IOC_CLEAN_INV_CACHES);
			}
		}
	}
	if (buf_size.sz_extnuserdata > 0) {
		dec_bufs->extnuserdata.mem_type = DDL_CMD_MEM;
		ptr = ddl_pmem_alloc(&dec_bufs->extnuserdata,
				buf_size.sz_extnuserdata, DDL_KILO_BYTE(2));
		if (!ptr)
			goto fail_free_exit;
		else
			memset(dec_bufs->extnuserdata.align_virtual_addr,
				0, buf_size.sz_extnuserdata);
	}
	return status;
fail_free_exit:
	status = VCD_ERR_ALLOC_FAIL;
	ddl_free_dec_hw_buffers(ddl);
	return status;
}

u32 ddl_calc_enc_hw_buffers_size(enum vcd_codec codec, u32 width,
	u32 height, enum vcd_yuv_buffer_format input_format,
	struct ddl_client_context *ddl,
	struct ddl_enc_buffer_size *buf_size)
{
	u32 status = VCD_S_SUCCESS, mb_x, mb_y;
	u32 sz_cur_y, sz_cur_c, sz_dpb_y, sz_dpb_c, sz_strm = 0, sz_mv;
	u32 sz_md = 0, sz_pred = 0, sz_nbor_info = 0 , sz_acdc_coef = 0;
	u32 sz_mb_info = 0, sz_context, sz_col_zero = 0;

	mb_x = (width + 15) / 16;
	mb_y = (height + 15) / 16;
	sz_dpb_y = ddl_get_yuv_buf_size(width,
		height, DDL_YUV_BUF_TYPE_TILE);
	sz_dpb_c = ddl_get_yuv_buf_size(width, height>>1,
		DDL_YUV_BUF_TYPE_TILE);
	if (input_format ==
		VCD_BUFFER_FORMAT_NV12_16M2KA) {
		sz_cur_y = ddl_get_yuv_buf_size(width, height,
			DDL_YUV_BUF_TYPE_LINEAR);
		sz_cur_c = ddl_get_yuv_buf_size(width, height>>1,
			DDL_YUV_BUF_TYPE_LINEAR);
	} else if (VCD_BUFFER_FORMAT_TILE_4x2 == input_format) {
		sz_cur_y = sz_dpb_y;
		sz_cur_c = sz_dpb_c;
	} else
		status = VCD_ERR_NOT_SUPPORTED;
	sz_context = DDL_FW_OTHER_CONTEXT_SPACE_SIZE;
	if (!status) {
		sz_strm = DDL_ALIGN(ddl_get_yuv_buf_size(width, height,
			DDL_YUV_BUF_TYPE_LINEAR) + ddl_get_yuv_buf_size(width,
			height/2, DDL_YUV_BUF_TYPE_LINEAR), DDL_KILO_BYTE(4));
		sz_mv = DDL_ALIGN(2 * mb_x * 8, DDL_KILO_BYTE(2));
		if ((codec == VCD_CODEC_MPEG4) ||
			(codec == VCD_CODEC_H264)) {
			sz_col_zero = DDL_ALIGN(((mb_x * mb_y + 7) / 8) *
					8, DDL_KILO_BYTE(2));
		}
		if ((codec == VCD_CODEC_MPEG4) ||
			(codec == VCD_CODEC_H263)) {
			sz_acdc_coef = DDL_ALIGN((width / 2) * 8,
						DDL_KILO_BYTE(2));
		} else if (codec == VCD_CODEC_H264) {
			sz_md = DDL_ALIGN(mb_x * 48, DDL_KILO_BYTE(2));
			sz_pred = DDL_ALIGN(2 * 8 * 1024, DDL_KILO_BYTE(2));
			sz_context = DDL_FW_H264ENC_CONTEXT_SPACE_SIZE;
			if (ddl) {
				if (ddl->codec_data.encoder.
					entropy_control.entropy_sel ==
					VCD_ENTROPY_SEL_CAVLC)
					sz_nbor_info = DDL_ALIGN(8 * 8 * mb_x,
						DDL_KILO_BYTE(2));
				else if (ddl->codec_data.encoder.
					entropy_control.entropy_sel ==
					VCD_ENTROPY_SEL_CABAC)
					sz_nbor_info = DDL_ALIGN(8 * 24 *
						mb_x, DDL_KILO_BYTE(2));
				if ((ddl->codec_data.encoder.
					mb_info_enable) &&
					(codec == VCD_CODEC_H264)) {
					sz_mb_info = DDL_ALIGN(mb_x * mb_y *
						6 * 8, DDL_KILO_BYTE(2));
				}
			}
		} else {
			sz_nbor_info = DDL_ALIGN(8 * 24 * mb_x,
						DDL_KILO_BYTE(2));
			sz_mb_info = DDL_ALIGN(mb_x * mb_y * 6 * 8,
					DDL_KILO_BYTE(2));
		}
		if (buf_size) {
			buf_size->sz_cur_y      = sz_cur_y;
			buf_size->sz_cur_c      = sz_cur_c;
			buf_size->sz_dpb_y      = sz_dpb_y;
			buf_size->sz_dpb_c      = sz_dpb_c;
			buf_size->sz_strm       = sz_strm;
			buf_size->sz_mv         = sz_mv;
			buf_size->sz_col_zero   = sz_col_zero;
			buf_size->sz_md         = sz_md;
			buf_size->sz_pred       = sz_pred;
			buf_size->sz_nbor_info  = sz_nbor_info;
			buf_size->sz_acdc_coef  = sz_acdc_coef;
			buf_size->sz_mb_info    = sz_mb_info;
			buf_size->sz_context    = sz_context;
		}
	}
	return status;
}

u32 ddl_allocate_enc_hw_buffers(struct ddl_client_context *ddl)
{
	struct ddl_enc_buffers *enc_bufs;
	struct ddl_enc_buffer_size buf_size;
	void *ptr;
	u32 status = VCD_S_SUCCESS;
	struct ddl_context *ddl_context = ddl->ddl_context;

	enc_bufs = &ddl->codec_data.encoder.hw_bufs;
	enc_bufs->dpb_count = DDL_ENC_MIN_DPB_BUFFERS;

	if ((ddl->codec_data.encoder.i_period.b_frames >
		DDL_MIN_NUM_OF_B_FRAME) ||
		(ddl->codec_data.encoder.num_references_for_p_frame
		> DDL_MIN_NUM_REF_FOR_P_FRAME))
		enc_bufs->dpb_count = DDL_ENC_MAX_DPB_BUFFERS;
		DDL_MSG_HIGH("Encoder num DPB buffers allocated = %d",
			enc_bufs->dpb_count);

	status = ddl_calc_enc_hw_buffers_size(
		ddl->codec_data.encoder.codec.codec,
		ddl->codec_data.encoder.frame_size.width,
		ddl->codec_data.encoder.frame_size.height,
		ddl->codec_data.encoder.buf_format.buffer_format,
		ddl, &buf_size);
	buf_size.sz_strm = ddl->codec_data.encoder.
		client_output_buf_req.sz;
	if (!status) {
		enc_bufs->sz_dpb_y = buf_size.sz_dpb_y;
		enc_bufs->sz_dpb_c = buf_size.sz_dpb_c;
		if (buf_size.sz_mv > 0) {
			enc_bufs->mv.mem_type = DDL_MM_MEM;
			ptr = ddl_pmem_alloc(&enc_bufs->mv, buf_size.sz_mv,
				DDL_KILO_BYTE(2));
			if (!ptr)
				goto fail_enc_free_exit;
		}
		if (buf_size.sz_col_zero > 0) {
			enc_bufs->col_zero.mem_type = DDL_MM_MEM;
			ptr = ddl_pmem_alloc(&enc_bufs->col_zero,
				buf_size.sz_col_zero, DDL_KILO_BYTE(2));
			if (!ptr)
				goto fail_enc_free_exit;
		}
		if (buf_size.sz_md > 0) {
			enc_bufs->md.mem_type = DDL_MM_MEM;
			ptr = ddl_pmem_alloc(&enc_bufs->md, buf_size.sz_md,
				DDL_KILO_BYTE(2));
			if (!ptr)
				goto fail_enc_free_exit;
		}
		if (buf_size.sz_pred > 0) {
			enc_bufs->pred.mem_type =
				res_trk_check_for_sec_session() ?
				DDL_MM_MEM : DDL_FW_MEM;
			ptr = ddl_pmem_alloc(&enc_bufs->pred,
				buf_size.sz_pred, DDL_KILO_BYTE(2));
			if (!ptr)
				goto fail_enc_free_exit;
		}
		if (buf_size.sz_nbor_info > 0) {
			enc_bufs->nbor_info.mem_type = DDL_MM_MEM;
			ptr = ddl_pmem_alloc(&enc_bufs->nbor_info,
				buf_size.sz_nbor_info, DDL_KILO_BYTE(2));
			if (!ptr)
				goto fail_enc_free_exit;
		}
		if (buf_size.sz_acdc_coef > 0) {
			enc_bufs->acdc_coef.mem_type = DDL_MM_MEM;
			ptr = ddl_pmem_alloc(&enc_bufs->acdc_coef,
				buf_size.sz_acdc_coef, DDL_KILO_BYTE(2));
			if (!ptr)
				goto fail_enc_free_exit;
		}
		if (buf_size.sz_mb_info > 0) {
			enc_bufs->mb_info.mem_type = DDL_MM_MEM;
			ptr = ddl_pmem_alloc(&enc_bufs->mb_info,
				buf_size.sz_mb_info, DDL_KILO_BYTE(2));
			if (!ptr)
				goto fail_enc_free_exit;
		}
		if (buf_size.sz_context > 0) {
			enc_bufs->context.mem_type = DDL_MM_MEM;
			ptr = ddl_pmem_alloc(&enc_bufs->context,
				buf_size.sz_context, DDL_KILO_BYTE(2));
			if (!ptr)
				goto fail_enc_free_exit;
			else
				msm_ion_do_cache_op(
					ddl_context->video_ion_client,
					enc_bufs->context.alloc_handle,
					enc_bufs->context.virtual_base_addr,
					enc_bufs->context.buffer_size,
					ION_IOC_CLEAN_INV_CACHES);
		}
	}
	return status;
fail_enc_free_exit:
	status = VCD_ERR_ALLOC_FAIL;
	ddl_free_enc_hw_buffers(ddl);
	return status;
}

void ddl_decoder_chroma_dpb_change(struct ddl_client_context *ddl)
{
	struct ddl_context *ddl_context = ddl->ddl_context;
	struct ddl_decoder_data *decoder = &ddl->codec_data.decoder;
	struct ddl_frame_data_tag *frame =
			&(decoder->dp_buf.dec_pic_buffers[0]);
	u32 luma[DDL_MAX_BUFFER_COUNT];
	u32 chroma[DDL_MAX_BUFFER_COUNT];
	u32 luma_size, i, dpb;
	luma_size = decoder->dpb_buf_size.size_y;
	dpb = decoder->dp_buf.no_of_dec_pic_buf;
	DDL_MSG_HIGH("%s Decoder num DPB buffers = %u Luma Size = %u",
			 __func__, dpb, luma_size);
	if (dpb > DDL_MAX_BUFFER_COUNT)
		dpb = DDL_MAX_BUFFER_COUNT;
	for (i = 0; i < dpb; i++) {
		luma[i] = DDL_OFFSET(
			ddl_context->dram_base_a.align_physical_addr,
			frame[i].vcd_frm.physical);
		chroma[i] = luma[i] + luma_size;
		DDL_MSG_LOW("%s Decoder Luma address = %x"
			"Chroma address = %x", __func__, luma[i], chroma[i]);
	}
	vidc_1080p_set_decode_recon_buffers(dpb, luma, chroma);
}

u32 ddl_check_reconfig(struct ddl_client_context *ddl)
{
	u32 need_reconfig = true;
	struct ddl_decoder_data *decoder = &ddl->codec_data.decoder;
	if (decoder->cont_mode) {
		if ((decoder->actual_output_buf_req.sz <=
			 decoder->client_output_buf_req.sz) &&
			(decoder->actual_output_buf_req.actual_count <=
			 decoder->client_output_buf_req.actual_count)) {
			need_reconfig = false;
			if (decoder->min_dpb_num >
				decoder->min_output_buf_req.min_count) {
				decoder->min_output_buf_req =
					decoder->actual_output_buf_req;
			}
			DDL_MSG_LOW("%s Decoder width = %u height = %u "
				"Client width = %u height = %u\n",
				__func__, decoder->frame_size.width,
				 decoder->frame_size.height,
				 decoder->client_frame_size.width,
				 decoder->client_frame_size.height);
		}
	} else {
		if ((decoder->frame_size.width ==
			decoder->client_frame_size.width) &&
			(decoder->frame_size.height ==
			decoder->client_frame_size.height) &&
			(decoder->actual_output_buf_req.sz <=
			decoder->client_output_buf_req.sz) &&
			(decoder->actual_output_buf_req.min_count ==
			decoder->client_output_buf_req.min_count) &&
			(decoder->actual_output_buf_req.actual_count ==
			decoder->client_output_buf_req.actual_count) &&
			(decoder->frame_size.scan_lines ==
			decoder->client_frame_size.scan_lines) &&
			(decoder->frame_size.stride ==
			decoder->client_frame_size.stride) &&
			decoder->progressive_only)
				need_reconfig = false;
	}
	return need_reconfig;
}

void ddl_handle_reconfig(u32 res_change, struct ddl_client_context *ddl)
{
	struct ddl_decoder_data *decoder = &ddl->codec_data.decoder;
	if ((decoder->cont_mode) &&
		(res_change == DDL_RESL_CHANGE_DECREASED)) {
		DDL_MSG_LOW("%s Resolution decreased, continue decoding\n",
				 __func__);
		vidc_sm_get_min_yc_dpb_sizes(
					&ddl->shared_mem[ddl->command_channel],
					&decoder->dpb_buf_size.size_y,
					&decoder->dpb_buf_size.size_c);
		DDL_MSG_LOW(" %s Resolution decreased, size_y = %u"
				" size_c = %u\n",
				__func__,
				decoder->dpb_buf_size.size_y,
				decoder->dpb_buf_size.size_c);
		ddl_decoder_chroma_dpb_change(ddl);
		vidc_sm_set_chroma_addr_change(
				&ddl->shared_mem[ddl->command_channel],
				true);
		ddl_vidc_decode_frame_run(ddl);
	} else {
		DDL_MSG_LOW("%s Resolution change, start realloc\n",
				 __func__);
		decoder->reconfig_detected = true;
		ddl->client_state = DDL_CLIENT_WAIT_FOR_EOS_DONE;
		ddl->cmd_state = DDL_CMD_EOS;
		vidc_1080p_frame_start_realloc(ddl->instance_id);
	}
}

void ddl_fill_dec_desc_buffer(struct ddl_client_context *ddl)
{
	struct ddl_decoder_data *decoder = &ddl->codec_data.decoder;
	struct vcd_frame_data *ip_bitstream = &(ddl->input_frame.vcd_frm);
	struct ddl_buf_addr *dec_desc_buf = &(decoder->hw_bufs.desc);

	if (ip_bitstream->desc_buf &&
		ip_bitstream->desc_size < DDL_KILO_BYTE(128))
		memcpy(dec_desc_buf->align_virtual_addr,
			   ip_bitstream->desc_buf,
			   ip_bitstream->desc_size);
}

void ddl_set_vidc_timeout(struct ddl_client_context *ddl)
{
	u32 vidc_time_out = 0;
	s32 multiplier = 1;
	u32 temp = DDL_VIDC_1080P_200MHZ_TIMEOUT_VALUE;
	struct ddl_decoder_data *decoder = &ddl->codec_data.decoder;
	struct vcd_frame_data *ip_bitstream = &(ddl->input_frame.vcd_frm);

	if (ddl->codec_data.decoder.idr_only_decoding)
		vidc_time_out = 2 * DDL_VIDC_1080P_200MHZ_TIMEOUT_VALUE;
	else {
		vidc_time_out = DDL_VIDC_1080P_200MHZ_TIMEOUT_VALUE;
		multiplier = decoder->yuv_size - (ip_bitstream->data_len +
						(ip_bitstream->data_len / 2));
		if (multiplier <= 0) {
			multiplier = decoder->yuv_size - ip_bitstream->data_len;
			if (multiplier <= 0) {
				if (ip_bitstream->data_len)
					multiplier =
					DDL_VIDC_1080P_MAX_TIMEOUT_MULTIPLIER;
			}
		}
		if (multiplier == DDL_VIDC_1080P_MAX_TIMEOUT_MULTIPLIER)
			vidc_time_out = vidc_time_out *
				DDL_VIDC_1080P_MAX_TIMEOUT_MULTIPLIER;
		else if (multiplier > 1) {
			temp = (decoder->yuv_size * 1000) / multiplier;
			temp = (temp * vidc_time_out) / 1000;
			if (temp > (u32)(vidc_time_out *
				DDL_VIDC_1080P_MAX_TIMEOUT_MULTIPLIER))
				vidc_time_out = vidc_time_out *
					DDL_VIDC_1080P_MAX_TIMEOUT_MULTIPLIER;
			else
				vidc_time_out = temp;
		}
	}
	DDL_MSG_LOW("%s Video core time out value = 0x%x",
		 __func__, vidc_time_out);
	vidc_sm_set_video_core_timeout_value(
		&ddl->shared_mem[ddl->command_channel], vidc_time_out);
}

void ddl_handle_ltr_in_framedone(struct ddl_client_context *ddl)
{
	struct ddl_ltr_encoding_type *ltr_control =
		&ddl->codec_data.encoder.ltr_control;
	DDL_MSG_LOW("%s:", __func__);
	if (ltr_control->storing) {
		ltr_control->ltr_list[ltr_control->storing_idx].ltr_id =
			ltr_control->curr_ltr_id;
		DDL_MSG_MED("Encoder output stores LTR ID %d into entry %d",
			ltr_control->curr_ltr_id, ltr_control->storing_idx);
		ltr_control->meta_data_reqd = true;
		ltr_control->storing = false;
	}
	ltr_control->out_frame_cnt_before_next_idr++;
	if (ltr_control->out_frame_cnt_to_use_this_ltr) {
		ltr_control->out_frame_cnt_to_use_this_ltr--;
		if (!ltr_control->out_frame_cnt_to_use_this_ltr)
			ddl_clear_ltr_list(ltr_control, true);
	}
}

s32 ddl_encoder_ltr_control(struct ddl_client_context *ddl)
{
	s32 vcd_status = VCD_S_SUCCESS;
	struct ddl_encoder_data *encoder = &ddl->codec_data.encoder;
	struct ddl_ltr_encoding_type *ltr_ctrl = &encoder->ltr_control;
	bool intra_period_reached = false;

	DDL_MSG_LOW("%s:", __func__);
	ddl_print_ltr_list(ltr_ctrl);

	if (DDL_IS_LTR_IN_AUTO_MODE(encoder)) {
		bool finite_i_period, infinite_i_period;
		DDL_MSG_LOW("%s: before LTR encoding: output "\
			"count before next IDR %d", __func__,
			ltr_ctrl->out_frame_cnt_before_next_idr);
		finite_i_period =
			(DDL_MAX_P_FRAMES_IN_INTRA_INTERVAL !=
			encoder->i_period.p_frames) &&
			(!(ltr_ctrl->out_frame_cnt_before_next_idr %
			(encoder->i_period.p_frames + 1)));
		infinite_i_period =
			((DDL_MAX_P_FRAMES_IN_INTRA_INTERVAL ==
			encoder->i_period.p_frames) &&
			(!ltr_ctrl->out_frame_cnt_before_next_idr));
		if (finite_i_period || infinite_i_period) {
			DDL_MSG_HIGH("%s: Intra period reached. "\
			"finite_i_period (%u), infinite_i_period (%u)",
			__func__, (u32)finite_i_period,
			(u32)infinite_i_period);
			intra_period_reached = true;
		}
		if (intra_period_reached ||
			ltr_ctrl->store_for_intraframe_insertion ||
			encoder->intra_period_changed) {
			ddl_clear_ltr_list(ltr_ctrl, false);
			ltr_ctrl->out_frame_cnt_before_next_idr = 0;
			ltr_ctrl->first_ltr_use_arvd = false;
			ltr_ctrl->store_for_intraframe_insertion = false;
		} else {
			if (ltr_ctrl->first_ltr_use_arvd == false) {
				ddl_use_ltr_from_list(ltr_ctrl, 0);
				ltr_ctrl->out_frame_cnt_to_use_this_ltr =
					0xFFFFFFFF;
				ltr_ctrl->use_ltr_reqd = true;
			}
		}
		if (!(ltr_ctrl->out_frame_cnt_before_next_idr %
			ltr_ctrl->ltr_period)) {
			s32 idx;
			DDL_MSG_HIGH("%s: reached LTR period "\
				"out_frame_cnt_before_next_idr %d",
				__func__, ltr_ctrl->\
				out_frame_cnt_before_next_idr);
			idx = ddl_find_oldest_ltr_not_in_use(
					ltr_ctrl);
			if (idx >= 0) {
				ltr_ctrl->storing = true;
				ltr_ctrl->storing_idx = idx;
				if (idx == 0)
					ltr_ctrl->store_ltr0 = true;
				else if (idx == 1)
					ltr_ctrl->store_ltr1 = true;
			}
		}
	}
	if (encoder->intra_frame_insertion) {
		DDL_MSG_HIGH("%s: I-frame insertion requested, "\
			"delay LTR store for one frame", __func__);
		ltr_ctrl->store_for_intraframe_insertion = true;
	}
	if (ltr_ctrl->pending_chg_ltr_useframes) {
		ltr_ctrl->out_frame_cnt_to_use_this_ltr =
			ltr_ctrl->ltr_use_frames;
		ltr_ctrl->pending_chg_ltr_useframes = false;
	}
	if (ltr_ctrl->out_frame_cnt_to_use_this_ltr)
		ltr_ctrl->use_ltr_reqd = true;
	if (ltr_ctrl->use_ltr_reqd) {
		s32 idx;
		idx = ddl_find_ltr_in_use(ltr_ctrl);
		if (idx == 0)
			ltr_ctrl->use_ltr0 = true;
		else if (idx == 1)
			ltr_ctrl->use_ltr1 = true;
		ltr_ctrl->using = true;
		ltr_ctrl->use_ltr_reqd = false;
	} else {
		DDL_MSG_HIGH("%s: use_ltr_reqd skipped", __func__);
	}

	return vcd_status;
}


s32 ddl_allocate_ltr_list(struct ddl_ltr_encoding_type *ltr_control)
{
	s32 vcd_status = VCD_S_SUCCESS;

	DDL_MSG_LOW("%s: lrr_cout = %u", __func__, ltr_control->ltr_count);
	if (!ltr_control->ltr_list) {
		if (ltr_control->ltr_count) {
			ltr_control->ltr_list = (struct ddl_ltrlist *)
				kmalloc(sizeof(struct ddl_ltrlist)*
					ltr_control->ltr_count, GFP_KERNEL);
			if (!ltr_control->ltr_list) {
				DDL_MSG_ERROR("ddl_allocate_ltr_list failed");
				vcd_status = VCD_ERR_ALLOC_FAIL;
			}
		} else {
			DDL_MSG_ERROR("%s: failed, zero LTR count", __func__);
			vcd_status = VCD_ERR_FAIL;
		}
	} else {
		DDL_MSG_HIGH("WARN: ltr_list already allocated");
	}

	return vcd_status;
}

s32 ddl_free_ltr_list(struct ddl_ltr_encoding_type *ltr_control)
{
	s32 vcd_status = VCD_S_SUCCESS;

	DDL_MSG_LOW("%s:", __func__);
	kfree(ltr_control->ltr_list);
	ltr_control->ltr_list = NULL;

	return vcd_status;
}

s32 ddl_clear_ltr_list(struct ddl_ltr_encoding_type *ltr_control,
	bool only_use_flag)
{
	s32 vcd_status = VCD_S_SUCCESS;
	u32 i;

	DDL_MSG_LOW("%s:", __func__);
	for (i = 0; i < ltr_control->ltr_count; i++) {
		ltr_control->ltr_list[i].ltr_in_use = false;
		if (!only_use_flag)
			ltr_control->ltr_list[i].ltr_id = 0;
	}

	return vcd_status;
}

s32 ddl_find_oldest_ltr_not_in_use(struct ddl_ltr_encoding_type *ltr_control)
{
	s32 found_idx = -1;
	u32 i;

	if (ltr_control->ltr_list) {
		if (ltr_control->ltr_count == 1)
			found_idx = 0;
		else {
			for (i = 0; i < ltr_control->ltr_count; i++) {
				if ((ltr_control->ltr_list[i].ltr_in_use ==
					false) && (found_idx < 0)) {
					found_idx = i;
				}
				if ((found_idx >= 0) &&
					(ltr_control->ltr_list[i].\
					ltr_in_use == false) &&
					(ltr_control->ltr_list[i].ltr_id <
					ltr_control->ltr_list[found_idx].\
					ltr_id)) {
					found_idx = i;
				}
			}
		}
	}

	DDL_MSG_LOW("%s: found_idx = %d", __func__, found_idx);
	return found_idx;
}

s32 ddl_find_ltr_in_use(struct ddl_ltr_encoding_type *ltr_control)
{
	s32 found_idx = -1;
	u32 i;

	if (ltr_control->ltr_list) {
		for (i = 0; i < ltr_control->ltr_count; i++) {
			if (ltr_control->ltr_list[i].ltr_in_use == true)
				found_idx = i;
		}
	}

	DDL_MSG_LOW("%s: found_idx = %d", __func__, found_idx);
	return found_idx;
}

s32 ddl_find_ltr_from_list(struct ddl_ltr_encoding_type *ltr_control,
	u32 ltr_id)
{
	s32 found_idx = -1;
	u32 i;

	if (ltr_control->ltr_list) {
		for (i = 0; i < ltr_control->ltr_count; i++) {
			if (ltr_control->ltr_list[i].ltr_id == ltr_id) {
				found_idx = i;
				break;
			}
		}
	} else {
		DDL_MSG_ERROR("%s: ltr_list is NULL", __func__);
	}

	DDL_MSG_LOW("%s: found_idx = %d", __func__, found_idx);
	return found_idx;
}

s32 ddl_use_ltr_from_list(struct ddl_ltr_encoding_type *ltr_control,
	u32 ltr_idx)
{
	s32 vcd_status = VCD_S_SUCCESS;
	u32 i;

	DDL_MSG_LOW("%s: ltr_idx = %u", __func__, ltr_idx);
	if (ltr_idx > ltr_control->ltr_count) {
		DDL_MSG_ERROR("%s: fail, idx %d larger than "\
			"the list array count %d", __func__,
			ltr_idx, ltr_control->ltr_count);
		vcd_status = VCD_ERR_FAIL;
	} else {
		for (i = 0; i < ltr_control->ltr_count; i++) {
			if (i == ltr_idx)
				ltr_control->ltr_list[ltr_idx].ltr_in_use =
					true;
			else
				ltr_control->ltr_list[i].ltr_in_use = false;
		}
	}

	return vcd_status;
}

void ddl_encoder_use_ltr_fail_callback(struct ddl_client_context *ddl)
{
	struct ddl_encoder_data *encoder = &(ddl->codec_data.encoder);
	struct ddl_context *ddl_context = ddl->ddl_context;

	DDL_MSG_ERROR("%s: LTR use failed, callback "\
		"requested with LTR ID %d", __func__,
		encoder->ltr_control.failed_use_cmd.ltr_id);

	ddl_context->ddl_callback(VCD_EVT_IND_INFO_LTRUSE_FAILED,
			VCD_ERR_ILLEGAL_PARM,
			&(encoder->ltr_control.failed_use_cmd),
			sizeof(struct vcd_property_ltruse_type),
			(u32 *)ddl,
			ddl->client_data);
}

void ddl_print_ltr_list(struct ddl_ltr_encoding_type *ltr_control)
{
	u32 i;

	for (i = 0; i < ltr_control->ltr_count; i++) {
		DDL_MSG_MED("%s: ltr_id: %d, ltr_in_use: %d",
			__func__, ltr_control->ltr_list[i].ltr_id,
			ltr_control->ltr_list[i].ltr_in_use);
	}
}
