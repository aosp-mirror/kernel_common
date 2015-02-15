/* Copyright (c) 2010-2012, The Linux Foundation. All rights reserved.
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

#include "vcd_ddl.h"
#include "vcd_ddl_metadata.h"
#include "vcd_ddl_shared_mem.h"
#include "vcd_core.h"
#include "vcd_res_tracker_api.h"

#if defined(PIX_CACHE_DISABLE)
#define DDL_PIX_CACHE_ENABLE  false
#else
#define DDL_PIX_CACHE_ENABLE  true
#endif
static unsigned int run_cnt;

void ddl_vidc_core_init(struct ddl_context *ddl_context)
{
	struct vidc_1080P_pix_cache_config pixel_cache_config;

	vidc_1080p_do_sw_reset(VIDC_1080P_RESET_IN_SEQ_FIRST_STAGE);
	msleep(DDL_SW_RESET_SLEEP);
	vidc_1080p_do_sw_reset(VIDC_1080P_RESET_IN_SEQ_SECOND_STAGE);
	vidc_1080p_init_memory_controller(
		(u32) ddl_context->dram_base_a.align_physical_addr,
		(u32) ddl_context->dram_base_b.align_physical_addr);
	vidc_1080p_clear_returned_channel_inst_id();
	ddl_context->vidc_decode_seq_start[0] =
		vidc_1080p_decode_seq_start_ch0;
	ddl_context->vidc_decode_seq_start[1] =
		vidc_1080p_decode_seq_start_ch1;
	ddl_context->vidc_decode_init_buffers[0] =
		vidc_1080p_decode_init_buffers_ch0;
	ddl_context->vidc_decode_init_buffers[1] =
		vidc_1080p_decode_init_buffers_ch1;
	ddl_context->vidc_decode_frame_start[0] =
		vidc_1080p_decode_frame_start_ch0;
	ddl_context->vidc_decode_frame_start[1] =
		vidc_1080p_decode_frame_start_ch1;
	ddl_context->vidc_set_dec_resolution[0] =
		vidc_1080p_set_dec_resolution_ch0;
	ddl_context->vidc_set_dec_resolution[1] =
		vidc_1080p_set_dec_resolution_ch1;
	ddl_context->vidc_encode_seq_start[0] =
		vidc_1080p_encode_seq_start_ch0;
	ddl_context->vidc_encode_seq_start[1] =
		vidc_1080p_encode_seq_start_ch1;
	ddl_context->vidc_encode_frame_start[0] =
		vidc_1080p_encode_frame_start_ch0;
	ddl_context->vidc_encode_frame_start[1] =
		vidc_1080p_encode_frame_start_ch1;
	ddl_context->vidc_encode_slice_batch_start[0] =
		vidc_1080p_encode_slice_batch_start_ch0;
	ddl_context->vidc_encode_slice_batch_start[1] =
		vidc_1080p_encode_slice_batch_start_ch1;
	vidc_1080p_release_sw_reset();
	ddl_context->pix_cache_enable = DDL_PIX_CACHE_ENABLE;
	if (ddl_context->pix_cache_enable) {
		vidc_pix_cache_sw_reset();
		pixel_cache_config.cache_enable = true;
		pixel_cache_config.prefetch_en = true;
		pixel_cache_config.port_select = VIDC_1080P_PIX_CACHE_PORT_B;
		pixel_cache_config.statistics_off = true;
		pixel_cache_config.page_size =
			VIDC_1080P_PIX_CACHE_PAGE_SIZE_1K;
		vidc_pix_cache_init_config(&pixel_cache_config);
	}
}

void ddl_vidc_core_term(struct ddl_context *ddl_context)
{
	if (ddl_context->pix_cache_enable) {
		u32 pix_cache_idle = false;
		u32 counter = 0;

		vidc_pix_cache_set_halt(true);

		do {
			msleep(DDL_SW_RESET_SLEEP);
			vidc_pix_cache_get_status_idle(&pix_cache_idle);
			counter++;
		} while (!pix_cache_idle &&
			counter < DDL_PIXEL_CACHE_STATUS_READ_RETRY);

		if (!pix_cache_idle) {
			ddl_context->cmd_err_status =
				DDL_PIXEL_CACHE_NOT_IDLE;
			ddl_handle_core_errors(ddl_context);
		}
	}
}

void ddl_vidc_channel_set(struct ddl_client_context *ddl)
{
	struct ddl_context *ddl_context = ddl->ddl_context;
	enum vcd_codec *vcd_codec;
	enum vidc_1080p_codec codec = VIDC_1080P_H264_DECODE;
	const enum vidc_1080p_decode_p_cache_enable
		dec_pix_cache = VIDC_1080P_DECODE_PCACHE_DISABLE;
	const enum vidc_1080p_encode_p_cache_enable
		enc_pix_cache = VIDC_1080P_ENCODE_PCACHE_ENABLE;
	u32 pix_cache_ctrl, ctxt_mem_offset, ctxt_mem_size, arg1 = 0;
	u8 *hw_ctxt = NULL;

	if (ddl->decoding) {
		ddl_set_core_start_time(__func__, DEC_OP_TIME);
		vcd_codec = &(ddl->codec_data.decoder.codec.codec);
		pix_cache_ctrl = (u32)dec_pix_cache;
		ctxt_mem_offset = DDL_ADDR_OFFSET(ddl_context->dram_base_a,
		ddl->codec_data.decoder.hw_bufs.context) >> 11;
		hw_ctxt =
		ddl->codec_data.decoder.hw_bufs.context.virtual_base_addr;
		ctxt_mem_size =
			ddl->codec_data.decoder.hw_bufs.context.buffer_size;
	} else {
		vcd_codec = &(ddl->codec_data.encoder.codec.codec);
		pix_cache_ctrl = (u32)enc_pix_cache;
		ctxt_mem_offset = DDL_ADDR_OFFSET(ddl_context->dram_base_a,
			ddl->codec_data.encoder.hw_bufs.context) >> 11;
		hw_ctxt =
		ddl->codec_data.encoder.hw_bufs.context.virtual_base_addr;
		ctxt_mem_size =
			ddl->codec_data.encoder.hw_bufs.context.buffer_size;
	}
	if (!res_trk_check_for_sec_session() && hw_ctxt) {
		memset(hw_ctxt, 0, ctxt_mem_size);
		arg1 = 1 << 29;
	}
	switch (*vcd_codec) {
	default:
	case VCD_CODEC_MPEG4:
		if (ddl->decoding)
			codec = VIDC_1080P_MPEG4_DECODE;
		else
			codec = VIDC_1080P_MPEG4_ENCODE;
	break;
	case VCD_CODEC_H264:
		if (ddl->decoding)
			codec = VIDC_1080P_H264_DECODE;
		else
			codec = VIDC_1080P_H264_ENCODE;
	break;
	case VCD_CODEC_DIVX_3:
		if (ddl->decoding)
			codec = VIDC_1080P_DIVX311_DECODE;
	break;
	case VCD_CODEC_DIVX_4:
		if (ddl->decoding)
			codec = VIDC_1080P_DIVX412_DECODE;
	break;
	case VCD_CODEC_DIVX_5:
		if (ddl->decoding)
			codec = VIDC_1080P_DIVX502_DECODE;
	break;
	case VCD_CODEC_DIVX_6:
		if (ddl->decoding)
			codec = VIDC_1080P_DIVX503_DECODE;
	break;
	case VCD_CODEC_XVID:
		if (ddl->decoding)
			codec = VIDC_1080P_MPEG4_DECODE;
	break;
	case VCD_CODEC_H263:
		if (ddl->decoding)
			codec = VIDC_1080P_H263_DECODE;
		else
			codec = VIDC_1080P_H263_ENCODE;
	break;
	case VCD_CODEC_MPEG1:
	case VCD_CODEC_MPEG2:
		if (ddl->decoding)
			codec = VIDC_1080P_MPEG2_DECODE;
	break;
	case VCD_CODEC_VC1:
		if (ddl->decoding)
			codec = VIDC_1080P_VC1_DECODE;
	break;
	case VCD_CODEC_VC1_RCV:
		if (ddl->decoding)
			codec = VIDC_1080P_VC1_RCV_DECODE;
	break;
	}
	ddl->cmd_state = DDL_CMD_CHANNEL_SET;
	DDL_MSG_LOW("ddl_state_transition: %s ~~> DDL_CLIENT_WAIT_FOR_CHDONE",
	ddl_get_state_string(ddl->client_state));
	ddl->client_state = DDL_CLIENT_WAIT_FOR_CHDONE;
	arg1 |= (u32)codec;
	vidc_1080p_set_host2risc_cmd(VIDC_1080P_HOST2RISC_CMD_OPEN_CH,
		arg1, pix_cache_ctrl, ctxt_mem_offset,
		ctxt_mem_size);
}

void ddl_vidc_decode_init_codec(struct ddl_client_context *ddl)
{
	struct ddl_context  *ddl_context = ddl->ddl_context;
	struct ddl_decoder_data *decoder = &(ddl->codec_data.decoder);
	struct vidc_1080p_dec_seq_start_param seq_start_param;
	u32 seq_size;

	ddl_set_core_start_time(__func__, DEC_OP_TIME);
	vidc_1080p_set_decode_mpeg4_pp_filter(decoder->post_filter.post_filter);
	vidc_sm_set_concealment_color(&ddl->shared_mem[ddl->command_channel],
		DDL_CONCEALMENT_Y_COLOR, DDL_CONCEALMENT_C_COLOR);

	vidc_sm_set_error_concealment_config(
		&ddl->shared_mem[ddl->command_channel],
		VIDC_SM_ERR_CONCEALMENT_INTER_SLICE_MB_COPY,
		VIDC_SM_ERR_CONCEALMENT_INTRA_SLICE_COLOR_CONCEALMENT,
		VIDC_SM_ERR_CONCEALMENT_ENABLE);

	ddl_vidc_metadata_enable(ddl);
	vidc_sm_set_metadata_start_address(&ddl->shared_mem
		[ddl->command_channel],
		DDL_ADDR_OFFSET(ddl_context->dram_base_a,
		ddl->codec_data.decoder.meta_data_input));

	vidc_sm_set_idr_decode_only(&ddl->shared_mem[ddl->command_channel],
			decoder->idr_only_decoding);

	if ((decoder->codec.codec == VCD_CODEC_DIVX_3) ||
	   (decoder->codec.codec == VCD_CODEC_VC1_RCV ||
		decoder->codec.codec == VCD_CODEC_VC1))
		ddl_context->vidc_set_dec_resolution
		[ddl->command_channel](decoder->client_frame_size.width,
		decoder->client_frame_size.height);
	else
	ddl_context->vidc_set_dec_resolution
	[ddl->command_channel](0x0, 0x0);
	DDL_MSG_LOW("HEADER-PARSE-START");
	DDL_MSG_LOW("ddl_state_transition: %s ~~>"
	"DDL_CLIENT_WAIT_FOR_INITCODECDONE",
	ddl_get_state_string(ddl->client_state));
	ddl->client_state = DDL_CLIENT_WAIT_FOR_INITCODECDONE;
	ddl->cmd_state = DDL_CMD_HEADER_PARSE;
	seq_start_param.cmd_seq_num = ++ddl_context->cmd_seq_num;
	seq_start_param.inst_id = ddl->instance_id;
	seq_start_param.shared_mem_addr_offset = DDL_ADDR_OFFSET(
	ddl_context->dram_base_a, ddl->shared_mem
		[ddl->command_channel]);
	seq_start_param.stream_buffer_addr_offset =
	DDL_OFFSET(ddl_context->dram_base_a.align_physical_addr,
	decoder->decode_config.sequence_header);
	seq_start_param.stream_buffersize =
		decoder->client_input_buf_req.sz;
	seq_size = decoder->decode_config.sequence_header_len +
		DDL_LINEAR_BUFFER_ALIGN_BYTES + VCD_SEQ_HDR_PADDING_BYTES;
	if (seq_start_param.stream_buffersize < seq_size)
		seq_start_param.stream_buffersize = seq_size;
	seq_start_param.stream_frame_size =
		decoder->decode_config.sequence_header_len;
	seq_start_param.descriptor_buffer_addr_offset =
		DDL_ADDR_OFFSET(ddl_context->dram_base_a,
		decoder->hw_bufs.desc),
	seq_start_param.descriptor_buffer_size =
		decoder->hw_bufs.desc.buffer_size;
	if ((decoder->codec.codec == VCD_CODEC_MPEG4) ||
		(decoder->codec.codec == VCD_CODEC_DIVX_4) ||
		(decoder->codec.codec == VCD_CODEC_DIVX_5) ||
		(decoder->codec.codec == VCD_CODEC_DIVX_6) ||
		(decoder->codec.codec == VCD_CODEC_XVID))
		vidc_sm_set_mpeg4_profile_override(
			&ddl->shared_mem[ddl->command_channel],
			VIDC_SM_PROFILE_INFO_ASP);
	if (VCD_CODEC_MPEG2 == decoder->codec.codec)
		vidc_sm_set_mp2datadumpbuffer(
			&ddl->shared_mem[ddl->command_channel],
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
			ddl->codec_data.decoder.hw_bufs.extnuserdata),
			DDL_KILO_BYTE(2));
	if (VCD_CODEC_H264 == decoder->codec.codec)
		vidc_sm_set_decoder_sei_enable(
			&ddl->shared_mem[ddl->command_channel],
			VIDC_SM_RECOVERY_POINT_SEI);
	ddl_context->vidc_decode_seq_start[ddl->command_channel](
		&seq_start_param);

	vidc_sm_set_decoder_stuff_bytes_consumption(
		&ddl->shared_mem[ddl->command_channel],
		VIDC_SM_NUM_STUFF_BYTES_CONSUME_NONE);
}

void ddl_vidc_decode_dynamic_property(struct ddl_client_context *ddl,
	u32 enable)
{
	struct ddl_decoder_data *decoder = &(ddl->codec_data.decoder);
	struct vcd_frame_data *bit_stream =
		&(ddl->input_frame.vcd_frm);
	struct ddl_context *ddl_context = ddl->ddl_context;

	if (!enable) {
		if (decoder->dynmic_prop_change_req)
			decoder->dynmic_prop_change_req = false;
		return;
	}
	if ((decoder->dynamic_prop_change & DDL_DEC_REQ_OUTPUT_FLUSH)) {
		decoder->dynmic_prop_change_req = true;
		decoder->dynamic_prop_change &= ~(DDL_DEC_REQ_OUTPUT_FLUSH);
		decoder->dpb_mask.hw_mask = 0;
		decoder->flush_pending = true;
	}
	if (((decoder->meta_data_enable_flag & VCD_METADATA_PASSTHROUGH)) &&
		((VCD_FRAME_FLAG_EXTRADATA & bit_stream->flags))) {
		u32 extradata_presence = true;
		u8* tmp = ((u8 *) bit_stream->physical +
				bit_stream->offset +
				bit_stream->data_len + 3);
		u32 extra_data_start = (u32) ((u32)tmp & ~3);

		extra_data_start = extra_data_start -
			(u32)ddl_context->dram_base_a.align_physical_addr;
		decoder->dynmic_prop_change_req = true;
		vidc_sm_set_extradata_addr(&ddl->shared_mem
			[ddl->command_channel], extra_data_start);
		vidc_sm_set_extradata_presence(&ddl->shared_mem
			[ddl->command_channel], extradata_presence);
	}
}

void ddl_vidc_encode_dynamic_property(struct ddl_client_context *ddl,
	u32 enable)
{
	struct ddl_encoder_data *encoder = &(ddl->codec_data.encoder);
	u32 frame_rate_change = false, bit_rate_change = false;
	u32 reset_req = false;

	if (!enable) {
		if (encoder->dynmic_prop_change_req) {
			reset_req = true;
			encoder->dynmic_prop_change_req = false;
		}
	} else {
		if ((encoder->dynamic_prop_change & DDL_ENC_REQ_IFRAME)) {
			encoder->intra_frame_insertion = true;
			encoder->dynamic_prop_change &=
				~(DDL_ENC_REQ_IFRAME);
		}
		if (encoder->dynamic_prop_change & DDL_ENC_LTR_USE_FRAME) {
			if (encoder->ltr_control.callback_reqd) {
				DDL_MSG_ERROR("%s: LTR use failed", __func__);
				ddl_encoder_use_ltr_fail_callback(ddl);
				encoder->ltr_control.callback_reqd = false;
			} else {
				encoder->ltr_control.use_ltr_reqd = true;
			}
			encoder->dynamic_prop_change &=
				~(DDL_ENC_LTR_USE_FRAME);
		}
		if ((encoder->dynamic_prop_change &
			DDL_ENC_CHANGE_BITRATE)) {
			bit_rate_change = true;
			vidc_sm_set_encoder_new_bit_rate(
				&ddl->shared_mem[ddl->command_channel],
				encoder->target_bit_rate.target_bitrate);
			encoder->dynamic_prop_change &=
				~(DDL_ENC_CHANGE_BITRATE);
		}
		if ((encoder->dynamic_prop_change
			& DDL_ENC_CHANGE_IPERIOD)) {
			encoder->intra_period_changed = true;
			vidc_sm_set_encoder_new_i_period(
				&ddl->shared_mem[ddl->command_channel],
				encoder->i_period.p_frames);
			encoder->dynamic_prop_change &=
				~(DDL_ENC_CHANGE_IPERIOD);
		}
		if ((encoder->dynamic_prop_change
			& DDL_ENC_CHANGE_FRAMERATE)) {
			frame_rate_change = true;
			vidc_sm_set_encoder_new_frame_rate(
				&ddl->shared_mem[ddl->command_channel],
				(u32)(DDL_FRAMERATE_SCALE(encoder->\
				frame_rate.fps_numerator) /
				encoder->frame_rate.fps_denominator));
			if (encoder->vui_timinginfo_enable &&
				encoder->frame_rate.fps_denominator) {
				vidc_sm_set_h264_encoder_timing_info(
					&ddl->shared_mem[ddl->command_channel],
					DDL_FRAMERATE_SCALE_FACTOR,
					(u32)(DDL_FRAMERATE_SCALE(encoder->\
					frame_rate.fps_numerator) / encoder->\
					frame_rate.fps_denominator) << 1);
			}
			encoder->dynamic_prop_change &=
				~(DDL_ENC_CHANGE_FRAMERATE);
		}
	}
	if ((enable) || (reset_req)) {
		vidc_sm_set_encoder_param_change(
			&ddl->shared_mem[ddl->command_channel],
			bit_rate_change, frame_rate_change,
			encoder->intra_period_changed);
	}
}

static void ddl_vidc_encode_set_profile_level(
	struct ddl_client_context *ddl)
{
	struct ddl_encoder_data *encoder = &(ddl->codec_data.encoder);
	u32  encode_profile, level = 0;

	switch (encoder->profile.profile) {
	default:
	case VCD_PROFILE_MPEG4_SP:
		encode_profile = VIDC_1080P_PROFILE_MPEG4_SIMPLE;
	break;
	case VCD_PROFILE_MPEG4_ASP:
		encode_profile = VIDC_1080P_PROFILE_MPEG4_ADV_SIMPLE;
	break;
	case VCD_PROFILE_H264_BASELINE:
		encode_profile = VIDC_1080P_PROFILE_H264_CONSTRAINED_BASELINE;
	break;
	case VCD_PROFILE_H264_MAIN:
		encode_profile = VIDC_1080P_PROFILE_H264_MAIN;
	break;
	case VCD_PROFILE_H264_HIGH:
		encode_profile = VIDC_1080P_PROFILE_H264_HIGH;
	break;
	}
	switch (encoder->level.level) {
	default:
	case VCD_LEVEL_MPEG4_0:
		level = VIDC_1080P_MPEG4_LEVEL0;
	break;
	case VCD_LEVEL_MPEG4_0b:
		level = VIDC_1080P_MPEG4_LEVEL0b;
	break;
	case VCD_LEVEL_MPEG4_1:
		level = VIDC_1080P_MPEG4_LEVEL1;
	break;
	case VCD_LEVEL_MPEG4_2:
		level = VIDC_1080P_MPEG4_LEVEL2;
	break;
	case VCD_LEVEL_MPEG4_3:
		level = VIDC_1080P_MPEG4_LEVEL3;
	break;
	case VCD_LEVEL_MPEG4_3b:
		level = VIDC_1080P_MPEG4_LEVEL3b;
	break;
	case VCD_LEVEL_MPEG4_4:
		level = VIDC_1080P_MPEG4_LEVEL4;
	break;
	case VCD_LEVEL_MPEG4_4a:
		level = VIDC_1080P_MPEG4_LEVEL4a;
	break;
	case VCD_LEVEL_MPEG4_5:
		level = VIDC_1080P_MPEG4_LEVEL5;
	break;
	case VCD_LEVEL_MPEG4_6:
		level = VIDC_1080P_MPEG4_LEVEL6;
	break;
	case VCD_LEVEL_MPEG4_7:
		level = VIDC_1080P_MPEG4_LEVEL7;
	break;
	case VCD_LEVEL_H264_1:
		level = VIDC_1080P_H264_LEVEL1;
	break;
	case VCD_LEVEL_H264_1b:
		level = VIDC_1080P_H264_LEVEL1b;
	break;
	case VCD_LEVEL_H264_1p1:
		level = VIDC_1080P_H264_LEVEL1p1;
	break;
	case VCD_LEVEL_H264_1p2:
		level = VIDC_1080P_H264_LEVEL1p2;
	break;
	case VCD_LEVEL_H264_1p3:
		level = VIDC_1080P_H264_LEVEL1p3;
	break;
	case VCD_LEVEL_H264_2:
		level = VIDC_1080P_H264_LEVEL2;
	break;
	case VCD_LEVEL_H264_2p1:
		level = VIDC_1080P_H264_LEVEL2p1;
	break;
	case VCD_LEVEL_H264_2p2:
		level = VIDC_1080P_H264_LEVEL2p2;
	break;
	case VCD_LEVEL_H264_3:
		level = VIDC_1080P_H264_LEVEL3;
	break;
	case VCD_LEVEL_H264_3p1:
		level = VIDC_1080P_H264_LEVEL3p1;
	break;
	case VCD_LEVEL_H264_3p2:
		level = VIDC_1080P_H264_LEVEL3p2;
	break;
	case VCD_LEVEL_H264_4:
		level = VIDC_1080P_H264_LEVEL4;
	break;
	case VCD_LEVEL_H263_10:
		level = VIDC_1080P_H263_LEVEL10;
	break;
	case VCD_LEVEL_H263_20:
		level = VIDC_1080P_H263_LEVEL20;
	break;
	case VCD_LEVEL_H263_30:
		level = VIDC_1080P_H263_LEVEL30;
	break;
	case VCD_LEVEL_H263_40:
		level = VIDC_1080P_H263_LEVEL40;
	break;
	case VCD_LEVEL_H263_45:
		level = VIDC_1080P_H263_LEVEL45;
	break;
	case VCD_LEVEL_H263_50:
		level = VIDC_1080P_H263_LEVEL50;
	break;
	case VCD_LEVEL_H263_60:
		level = VIDC_1080P_H263_LEVEL60;
	break;
	case VCD_LEVEL_H263_70:
		level = VIDC_1080P_H263_LEVEL70;
	break;
	}
	vidc_1080p_set_encode_profile_level(encode_profile, level);
}

static void ddl_vidc_encode_set_multi_slice_info(
	struct ddl_encoder_data *encoder)
{
	enum vidc_1080p_MSlice_selection m_slice_sel;
	u32 i_multi_slice_size = 0, i_multi_slice_byte = 0;

	if (!encoder) {
		DDL_MSG_ERROR("Invalid Parameter");
		return;
	}

	switch (encoder->multi_slice.m_slice_sel) {
	default:
	case VCD_MSLICE_OFF:
		m_slice_sel = VIDC_1080P_MSLICE_DISABLE;
	break;
	case VCD_MSLICE_BY_GOB:
	case VCD_MSLICE_BY_MB_COUNT:
		m_slice_sel = VIDC_1080P_MSLICE_BY_MB_COUNT;
		i_multi_slice_size = encoder->multi_slice.m_slice_size;
	break;
	case VCD_MSLICE_BY_BYTE_COUNT:
		m_slice_sel = VIDC_1080P_MSLICE_BY_BYTE_COUNT;
		i_multi_slice_byte = encoder->multi_slice.m_slice_size;
	break;
	}
	vidc_1080p_set_encode_multi_slice_control(m_slice_sel,
		i_multi_slice_size, i_multi_slice_byte);
}

static void ddl_vidc_encode_set_batch_slice_info(
	struct ddl_client_context *ddl)
{
	struct ddl_context *ddl_context = ddl->ddl_context;
	struct ddl_encoder_data *encoder = &(ddl->codec_data.encoder);
	DDL_MSG_LOW("%s\n", __func__);
	encoder->batch_frame.slice_batch_out.buffer_size =
					encoder->output_buf_req.sz;
	DDL_MSG_LOW("encoder->batch_frame.slice_batch_out.buffer_size = %d\n",
			encoder->batch_frame.slice_batch_out.buffer_size);
	vidc_sm_set_encoder_batch_config(
			&ddl->shared_mem[ddl->command_channel],
			1,
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
			encoder->batch_frame.slice_batch_in),
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
			encoder->batch_frame.slice_batch_out),
			encoder->batch_frame.slice_batch_out.buffer_size);
	vidc_sm_set_encoder_slice_batch_int_ctrl(
			&ddl->shared_mem[ddl->command_channel],
			0);
}

void ddl_vidc_encode_init_codec(struct ddl_client_context *ddl)
{
	struct ddl_context *ddl_context = ddl->ddl_context;
	struct ddl_encoder_data *encoder = &(ddl->codec_data.encoder);
	struct ddl_enc_buffers *enc_buffers = &encoder->hw_bufs;
	struct vidc_1080p_enc_seq_start_param seq_start_param;
	enum vidc_1080p_memory_access_method mem_access_method;
	enum vidc_1080p_DBConfig db_config;
	enum VIDC_SM_frame_skip r_cframe_skip =
		VIDC_SM_FRAME_SKIP_DISABLE;
	u32 index, luma[4], chroma[4], hdr_ext_control = false;
	const u32 recon_bufs = 4;
	u32 h263_cpfc_enable = false;
	u32 scaled_frame_rate, ltr_enable;

	ddl_vidc_encode_set_profile_level(ddl);
	vidc_1080p_set_encode_frame_size(encoder->frame_size.width,
		encoder->frame_size.height);
	vidc_1080p_encode_set_qp_params(encoder->qp_range.max_qp,
		encoder->qp_range.min_qp);
	vidc_1080p_encode_set_rc_config(encoder->rc_level.frame_level_rc,
		encoder->rc_level.mb_level_rc,
		encoder->session_qp.i_frame_qp);
	if (encoder->hdr_ext_control > 0)
		hdr_ext_control = true;
	if (encoder->r_cframe_skip > 0)
		r_cframe_skip = VIDC_SM_FRAME_SKIP_ENABLE_LEVEL;
	scaled_frame_rate = DDL_FRAMERATE_SCALE(encoder->\
			frame_rate.fps_numerator) /
			encoder->frame_rate.fps_denominator;
	if ((encoder->codec.codec == VCD_CODEC_H263) &&
		(DDL_FRAMERATE_SCALE(DDL_INITIAL_FRAME_RATE)
		 != scaled_frame_rate) && encoder->plusptype_enable)
		h263_cpfc_enable = true;
	ltr_enable = DDL_IS_LTR_ENABLED(encoder);
	DDL_MSG_HIGH("ltr_enable = %u", ltr_enable);
	vidc_sm_set_extended_encoder_control(&ddl->shared_mem
		[ddl->command_channel], hdr_ext_control,
		r_cframe_skip, false, 0,
		h263_cpfc_enable, encoder->sps_pps.sps_pps_for_idr_enable_flag,
		encoder->closed_gop, encoder->avc_delimiter_enable,
		encoder->vui_timinginfo_enable, ltr_enable);
	if (encoder->vui_timinginfo_enable) {
		vidc_sm_set_h264_encoder_timing_info(
			&ddl->shared_mem[ddl->command_channel],
			DDL_FRAMERATE_SCALE_FACTOR,
			scaled_frame_rate << 1);
	}
	vidc_sm_set_encoder_init_rc_value(&ddl->shared_mem
		[ddl->command_channel],
		encoder->target_bit_rate.target_bitrate);
	vidc_sm_set_encoder_hec_period(&ddl->shared_mem
		[ddl->command_channel], encoder->hdr_ext_control);
		vidc_sm_set_encoder_vop_time(&ddl->shared_mem
			[ddl->command_channel], true,
			encoder->vop_timing.vop_time_resolution, 0);
	if (encoder->rc_level.frame_level_rc)
		vidc_1080p_encode_set_frame_level_rc_params(
			scaled_frame_rate,
			encoder->target_bit_rate.target_bitrate,
			encoder->frame_level_rc.reaction_coeff);
	if (encoder->rc_level.mb_level_rc)
		vidc_1080p_encode_set_mb_level_rc_params(
			encoder->adaptive_rc.disable_dark_region_as_flag,
			encoder->adaptive_rc.disable_smooth_region_as_flag,
			encoder->adaptive_rc.disable_static_region_as_flag,
			encoder->adaptive_rc.disable_activity_region_flag);
	if ((!encoder->rc_level.frame_level_rc) &&
		(!encoder->rc_level.mb_level_rc))
		vidc_sm_set_pand_b_frame_qp(
			&ddl->shared_mem[ddl->command_channel],
			encoder->session_qp.b_frame_qp,
			encoder->session_qp.p_frame_qp);
	if (encoder->codec.codec == VCD_CODEC_MPEG4) {
		vidc_1080p_set_mpeg4_encode_quarter_pel_control(false);
		vidc_1080p_set_encode_field_picture_structure(false);
	}
	if (encoder->codec.codec == VCD_CODEC_H264) {
		enum vidc_1080p_entropy_sel entropy_sel;
		switch (encoder->entropy_control.entropy_sel) {
		default:
		case VCD_ENTROPY_SEL_CAVLC:
			entropy_sel = VIDC_1080P_ENTROPY_SEL_CAVLC;
		break;
		case VCD_ENTROPY_SEL_CABAC:
			entropy_sel = VIDC_1080P_ENTROPY_SEL_CABAC;
		break;
	}
	vidc_1080p_set_h264_encode_entropy(entropy_sel);
	switch (encoder->db_control.db_config) {
	default:
	case VCD_DB_ALL_BLOCKING_BOUNDARY:
		db_config = VIDC_1080P_DB_ALL_BLOCKING_BOUNDARY;
	break;
	case VCD_DB_DISABLE:
		db_config = VIDC_1080P_DB_DISABLE;
	break;
	case VCD_DB_SKIP_SLICE_BOUNDARY:
		db_config = VIDC_1080P_DB_SKIP_SLICE_BOUNDARY;
	break;
	}
	vidc_1080p_set_h264_encode_loop_filter(db_config,
		encoder->db_control.slice_alpha_offset,
		encoder->db_control.slice_beta_offset);
	vidc_1080p_set_h264_encoder_p_frame_ref_count(encoder->\
		num_references_for_p_frame);
	if (encoder->profile.profile == VCD_PROFILE_H264_HIGH)
		vidc_1080p_set_h264_encode_8x8transform_control(true);
	}
	vidc_1080p_set_encode_picture(encoder->i_period.p_frames,
		encoder->i_period.b_frames);
	vidc_1080p_set_encode_circular_intra_refresh(
		encoder->intra_refresh.cir_mb_number);
	ddl_vidc_encode_set_multi_slice_info(encoder);
	ddl_vidc_metadata_enable(ddl);
	if (encoder->meta_data_enable_flag)
		vidc_sm_set_metadata_start_address(&ddl->shared_mem
			[ddl->command_channel], DDL_ADDR_OFFSET(
			ddl_context->dram_base_a,
			ddl->codec_data.encoder.meta_data_input));
	luma[0] = DDL_ADDR_OFFSET(ddl_context->dram_base_a,
			enc_buffers->dpb_y[0]);
	luma[1] = DDL_ADDR_OFFSET(ddl_context->dram_base_a,
			enc_buffers->dpb_y[1]);
	if (encoder->hw_bufs.dpb_count == DDL_ENC_MAX_DPB_BUFFERS) {
		luma[2] = DDL_ADDR_OFFSET(ddl_context->dram_base_b,
			enc_buffers->dpb_y[2]);
		luma[3] = DDL_ADDR_OFFSET(ddl_context->dram_base_b,
			enc_buffers->dpb_y[3]);
	}
	for (index = 0; index < recon_bufs; index++)
		chroma[index] = DDL_ADDR_OFFSET(ddl_context->dram_base_b,
					enc_buffers->dpb_c[index]);
	vidc_1080p_set_encode_recon_buffers(recon_bufs, luma, chroma);
	switch (encoder->codec.codec) {
	case VCD_CODEC_MPEG4:
		vidc_1080p_set_mpeg4_encode_work_buffers(
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				enc_buffers->col_zero),
				DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				enc_buffers->acdc_coef),
				DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				enc_buffers->mv));
	break;
	case VCD_CODEC_H263:
		vidc_1080p_set_h263_encode_work_buffers(
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				enc_buffers->mv),
				DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				enc_buffers->acdc_coef));
	break;
	case VCD_CODEC_H264:
		vidc_1080p_set_h264_encode_work_buffers(
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				enc_buffers->mv),
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				enc_buffers->col_zero),
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				enc_buffers->md),
			DDL_ADDR_OFFSET(ddl_context->dram_base_b,
				enc_buffers->pred),
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				enc_buffers->nbor_info),
			DDL_ADDR_OFFSET(ddl_context->dram_base_a,
				enc_buffers->mb_info));
	break;
	default:
	break;
	}
	if (encoder->buf_format.buffer_format ==
		VCD_BUFFER_FORMAT_NV12_16M2KA)
		mem_access_method = VIDC_1080P_TILE_LINEAR;
	else
		mem_access_method = VIDC_1080P_TILE_64x32;
	vidc_1080p_set_encode_input_frame_format(mem_access_method);
	vidc_1080p_set_encode_padding_control(0, 0, 0, 0);
	DDL_MSG_LOW("ddl_state_transition: %s ~~>"
		"DDL_CLIENT_WAIT_FOR_INITCODECDONE",
		ddl_get_state_string(ddl->client_state));
	ddl->client_state = DDL_CLIENT_WAIT_FOR_INITCODECDONE;
	ddl->cmd_state = DDL_CMD_INIT_CODEC;
	vidc_1080p_set_encode_field_picture_structure(false);
	seq_start_param.cmd_seq_num = ++ddl_context->cmd_seq_num;
	seq_start_param.inst_id = ddl->instance_id;
	seq_start_param.shared_mem_addr_offset = DDL_ADDR_OFFSET(
		ddl_context->dram_base_a, ddl->shared_mem
		[ddl->command_channel]);
	seq_start_param.stream_buffer_addr_offset = DDL_ADDR_OFFSET(
		ddl_context->dram_base_a, encoder->seq_header);
	seq_start_param.stream_buffer_size =
		encoder->seq_header.buffer_size;
	encoder->seq_header_length = 0;
	ddl_context->vidc_encode_seq_start[ddl->command_channel](
		&seq_start_param);
}

void ddl_vidc_channel_end(struct ddl_client_context *ddl)
{
	DDL_MSG_LOW("ddl_state_transition: %s ~~> DDL_CLIENT_WAIT_FOR_CHEND",
	ddl_get_state_string(ddl->client_state));
	ddl->client_state = DDL_CLIENT_WAIT_FOR_CHEND;
	ddl->cmd_state = DDL_CMD_CHANNEL_END;
	vidc_1080p_set_host2risc_cmd(VIDC_1080P_HOST2RISC_CMD_CLOSE_CH,
		ddl->instance_id, 0, 0, 0);
}

void ddl_vidc_encode_frame_run(struct ddl_client_context *ddl)
{
	struct vidc_1080p_enc_frame_start_param enc_param;
	struct ddl_context *ddl_context = ddl->ddl_context;
	struct ddl_encoder_data  *encoder = &(ddl->codec_data.encoder);
	struct ddl_enc_buffers *enc_buffers = &(encoder->hw_bufs);
	struct vcd_frame_data *stream = &(ddl->output_frame.vcd_frm);
	struct vcd_frame_data *input_vcd_frm =
		&(ddl->input_frame.vcd_frm);
	u32 dpb_addr_y[4], dpb_addr_c[4];
	u32 index, y_addr, c_addr;

	DDL_MSG_LOW("%s\n", __func__);
	ddl_vidc_encode_set_metadata_output_buf(ddl);

	encoder->enc_frame_info.meta_data_exists = false;

	y_addr = DDL_OFFSET(ddl_context->dram_base_b.align_physical_addr,
			input_vcd_frm->physical);
	c_addr = (y_addr + encoder->input_buf_size.size_y);
	if (input_vcd_frm->flags & VCD_FRAME_FLAG_EOS) {
		enc_param.encode = VIDC_1080P_ENC_TYPE_LAST_FRAME_DATA;
		DDL_MSG_LOW("ddl_state_transition: %s ~~>"
			"DDL_CLIENT_WAIT_FOR_EOS_DONE",
			ddl_get_state_string(ddl->client_state));
		ddl->client_state = DDL_CLIENT_WAIT_FOR_EOS_DONE;
	} else {
		enc_param.encode = VIDC_1080P_ENC_TYPE_FRAME_DATA;
		DDL_MSG_LOW("ddl_state_transition: %s ~~>"
			"DDL_CLIENT_WAIT_FOR_FRAME_DONE",
			ddl_get_state_string(ddl->client_state));
		ddl->client_state = DDL_CLIENT_WAIT_FOR_FRAME_DONE;
	}
	ddl->cmd_state = DDL_CMD_ENCODE_FRAME;
	if (encoder->dynamic_prop_change) {
		encoder->dynmic_prop_change_req = true;
		ddl_vidc_encode_dynamic_property(ddl, true);
	}
	if (DDL_IS_LTR_ENABLED(encoder))
		ddl_encoder_ltr_control(ddl);
	vidc_1080p_set_encode_circular_intra_refresh(
		encoder->intra_refresh.cir_mb_number);
	ddl_vidc_encode_set_multi_slice_info(encoder);
	enc_param.cmd_seq_num = ++ddl_context->cmd_seq_num;
	enc_param.inst_id = ddl->instance_id;
	enc_param.shared_mem_addr_offset = DDL_ADDR_OFFSET(
			ddl_context->dram_base_a,
			ddl->shared_mem[ddl->command_channel]);
	enc_param.current_y_addr_offset = y_addr;
	enc_param.current_c_addr_offset = c_addr;
	enc_param.stream_buffer_addr_offset = DDL_OFFSET(
	ddl_context->dram_base_a.align_physical_addr, stream->physical);
	enc_param.stream_buffer_size =
		encoder->client_output_buf_req.sz;
	enc_param.intra_frame = encoder->intra_frame_insertion;
	enc_param.input_flush = false;
	enc_param.slice_enable = false;
	enc_param.store_ltr0 = encoder->ltr_control.store_ltr0;
	enc_param.store_ltr1 = encoder->ltr_control.store_ltr1;
	enc_param.use_ltr0 = encoder->ltr_control.use_ltr0;
	enc_param.use_ltr1 = encoder->ltr_control.use_ltr1;

	encoder->intra_frame_insertion = false;
	encoder->intra_period_changed = false;
	encoder->ltr_control.store_ltr0 = false;
	encoder->ltr_control.store_ltr1 = false;
	encoder->ltr_control.use_ltr0 = false;
	encoder->ltr_control.use_ltr1 = false;
	vidc_sm_set_encoder_vop_time(
			&ddl->shared_mem[ddl->command_channel], true,
			encoder->vop_timing.vop_time_resolution,
			ddl->input_frame.frm_delta);
	vidc_sm_set_frame_tag(&ddl->shared_mem[ddl->command_channel],
	ddl->input_frame.vcd_frm.ip_frm_tag);
	if (ddl_context->pix_cache_enable) {
		for (index = 0; index < enc_buffers->dpb_count;
			index++) {
			dpb_addr_y[index] =
				(u32) VIDC_1080P_DEC_DPB_RESET_VALUE;
			dpb_addr_c[index] = (u32) enc_buffers->dpb_c
				[index].align_physical_addr;
		}

		dpb_addr_y[index] = (u32) input_vcd_frm->physical;
		dpb_addr_c[index] = (u32) input_vcd_frm->physical +
			encoder->input_buf_size.size_y;

		vidc_pix_cache_init_luma_chroma_base_addr(
			enc_buffers->dpb_count + 1, dpb_addr_y, dpb_addr_c);
		vidc_pix_cache_set_frame_size(encoder->frame_size.width,
			encoder->frame_size.height);
		vidc_pix_cache_set_frame_range(enc_buffers->sz_dpb_y,
			enc_buffers->sz_dpb_c);
		vidc_pix_cache_clear_cache_tags();
	}
	ddl_context->vidc_encode_frame_start[ddl->command_channel] (
		&enc_param);
}

void ddl_vidc_encode_frame_continue(struct ddl_client_context *ddl)
{
	struct ddl_context *ddl_context = ddl->ddl_context;
	struct vcd_frame_data *input_vcd_frm = &(ddl->input_frame.vcd_frm);
	u32 address_offset;
	address_offset = (u32)(ddl->output_frame.vcd_frm.physical -
			ddl_context->dram_base_a.align_physical_addr) >>
			DDL_VIDC_1080P_BASE_OFFSET_SHIFT;
	DDL_MSG_LOW("%s\n", __func__);
	if (VCD_FRAME_FLAG_EOS & input_vcd_frm->flags)
		ddl->client_state = DDL_CLIENT_WAIT_FOR_EOS_DONE;
	else
		ddl->client_state = DDL_CLIENT_WAIT_FOR_FRAME_DONE;
	ddl->cmd_state = DDL_CMD_ENCODE_CONTINUE;
	vidc_1080p_set_host2risc_cmd(VIDC_1080P_HOST2RISC_CMD_CONTINUE_ENC,
		address_offset,
		0, 0, 0);
}

void ddl_vidc_encode_slice_batch_run(struct ddl_client_context *ddl)
{
	struct vidc_1080p_enc_frame_start_param enc_param;
	struct ddl_context *ddl_context = ddl->ddl_context;
	struct ddl_encoder_data  *encoder = &(ddl->codec_data.encoder);
	struct ddl_enc_buffers *enc_buffers = &(encoder->hw_bufs);
	struct vcd_frame_data *input_vcd_frm =
		&(ddl->input_frame.vcd_frm);
	u32 dpb_addr_y[4], dpb_addr_c[4];
	u32 index, y_addr, c_addr;
	u32 bitstream_size;
	struct vidc_1080p_enc_slice_batch_in_param *slice_batch_in =
		(struct vidc_1080p_enc_slice_batch_in_param *)
		encoder->batch_frame.slice_batch_in.align_virtual_addr;
	DDL_MSG_LOW("%s\n", __func__);
	DDL_MEMSET(slice_batch_in, 0,
		sizeof(struct vidc_1080p_enc_slice_batch_in_param));
	DDL_MEMSET(encoder->batch_frame.slice_batch_in.align_virtual_addr, 0,
		sizeof(struct vidc_1080p_enc_slice_batch_in_param));
	encoder->batch_frame.out_frm_next_frmindex = 0;
	bitstream_size = encoder->batch_frame.output_frame[0].vcd_frm.alloc_len;
	encoder->output_buf_req.sz = bitstream_size;
	y_addr = DDL_OFFSET(ddl_context->dram_base_b.align_physical_addr,
			input_vcd_frm->physical);
	c_addr = (y_addr + encoder->input_buf_size.size_y);
	enc_param.encode = VIDC_1080P_ENC_TYPE_SLICE_BATCH_START;
	DDL_MSG_LOW("ddl_state_transition: %s ~~>"
		"DDL_CLIENT_WAIT_FOR_FRAME_DONE",
		ddl_get_state_string(ddl->client_state));
	slice_batch_in->cmd_type = VIDC_1080P_ENC_TYPE_SLICE_BATCH_START;
	ddl->client_state = DDL_CLIENT_WAIT_FOR_FRAME_DONE;
	ddl->cmd_state = DDL_CMD_ENCODE_FRAME;
	vidc_1080p_set_encode_circular_intra_refresh(
		encoder->intra_refresh.cir_mb_number);
	ddl_vidc_encode_set_multi_slice_info(encoder);
	enc_param.cmd_seq_num = ++ddl_context->cmd_seq_num;
	enc_param.inst_id = ddl->instance_id;
	enc_param.shared_mem_addr_offset = DDL_ADDR_OFFSET(
			ddl_context->dram_base_a,
			ddl->shared_mem[ddl->command_channel]);
	enc_param.current_y_addr_offset = y_addr;
	enc_param.current_c_addr_offset = c_addr;
	enc_param.stream_buffer_size = bitstream_size;
	slice_batch_in->num_stream_buffer =
		encoder->batch_frame.num_output_frames;
	slice_batch_in->stream_buffer_size = bitstream_size;
	DDL_MSG_LOW("%s slice_batch_in->num_stream_buffer = %u size = %u\n",
			 __func__, slice_batch_in->num_stream_buffer,
			slice_batch_in->stream_buffer_size);
	for (index = 0; index < encoder->batch_frame.num_output_frames;
		index++) {
		slice_batch_in->stream_buffer_addr_offset[index] =
		((DDL_OFFSET(ddl_context->dram_base_b.align_physical_addr,
		encoder->batch_frame.output_frame[index].vcd_frm.physical)) >>
			DDL_VIDC_1080P_BASE_OFFSET_SHIFT);
	}
	slice_batch_in->input_size = VIDC_1080P_SLICE_BATCH_IN_SIZE(index);
	enc_param.intra_frame = encoder->intra_frame_insertion;
	if (encoder->intra_frame_insertion)
		encoder->intra_frame_insertion = false;
	enc_param.input_flush = false;
	enc_param.slice_enable =
		encoder->slice_delivery_info.enable;
	vidc_sm_set_encoder_vop_time(
			&ddl->shared_mem[ddl->command_channel], true,
			encoder->vop_timing.vop_time_resolution,
			ddl->input_frame.frm_delta);
	vidc_sm_set_frame_tag(&ddl->shared_mem[ddl->command_channel],
			ddl->input_frame.vcd_frm.ip_frm_tag);
	DDL_MSG_LOW("%sdpb_count = %d\n", __func__, enc_buffers->dpb_count);
	if (ddl_context->pix_cache_enable) {
		for (index = 0; index < enc_buffers->dpb_count;
			index++) {
			dpb_addr_y[index] =
				(u32) VIDC_1080P_DEC_DPB_RESET_VALUE;
			dpb_addr_c[index] = (u32) enc_buffers->dpb_c
				[index].align_physical_addr;
		}

		dpb_addr_y[index] = (u32) input_vcd_frm->physical;
		dpb_addr_c[index] = (u32) input_vcd_frm->physical +
				encoder->input_buf_size.size_y;

		vidc_pix_cache_init_luma_chroma_base_addr(
			enc_buffers->dpb_count + 1, dpb_addr_y, dpb_addr_c);
		vidc_pix_cache_set_frame_size(encoder->frame_size.width,
			encoder->frame_size.height);
		vidc_pix_cache_set_frame_range(enc_buffers->sz_dpb_y,
			enc_buffers->sz_dpb_c);
		vidc_pix_cache_clear_cache_tags();
	}
	if ((!encoder->rc_level.frame_level_rc) &&
		(!encoder->rc_level.mb_level_rc)) {
		encoder->session_qp.p_frame_qp++;
		if (encoder->session_qp.p_frame_qp > encoder->qp_range.max_qp)
			encoder->session_qp.p_frame_qp =
						encoder->qp_range.min_qp;
		vidc_sm_set_pand_b_frame_qp(
			&ddl->shared_mem[ddl->command_channel],
				encoder->session_qp.b_frame_qp,
				encoder->session_qp.p_frame_qp);
	}

	if (vidc_msg_timing) {
		if (run_cnt < 2) {
			ddl_reset_core_time_variables(ENC_OP_TIME);
			ddl_reset_core_time_variables(ENC_SLICE_OP_TIME);
			run_cnt++;
		 }
		ddl_update_core_start_time(__func__, ENC_SLICE_OP_TIME);
		ddl_set_core_start_time(__func__, ENC_OP_TIME);
	}
	encoder->num_slices_comp = 0;
	ddl_vidc_encode_set_batch_slice_info(ddl);
	ddl_context->vidc_encode_slice_batch_start[ddl->command_channel] (
			&enc_param);
}

u32 ddl_vidc_decode_set_buffers(struct ddl_client_context *ddl)
{
	struct ddl_context *ddl_context = ddl->ddl_context;
	struct ddl_decoder_data *decoder = &(ddl->codec_data.decoder);
	u32 vcd_status = VCD_S_SUCCESS;
	struct vidc_1080p_dec_init_buffers_param init_buf_param;
	u32 size_y = 0;
	u32 size_c = 0;

	if (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_DPB)) {
		DDL_MSG_ERROR("STATE-CRITICAL");
		return VCD_ERR_FAIL;
	}
	ddl_set_vidc_timeout(ddl);
	ddl_vidc_decode_set_metadata_output(decoder);
	if (decoder->dp_buf.no_of_dec_pic_buf <
		decoder->client_output_buf_req.actual_count)
		return VCD_ERR_BAD_STATE;
	if (decoder->codec.codec == VCD_CODEC_H264) {
		vidc_sm_set_allocated_h264_mv_size(
			&ddl->shared_mem[ddl->command_channel],
			decoder->hw_bufs.h264_mv[0].buffer_size);
	}
	if (vcd_status)
		return vcd_status;
#ifdef DDL_BUF_LOG
	ddl_list_buffers(ddl);
#endif
	ddl_set_core_start_time(__func__, DEC_OP_TIME);
	ddl_decoder_dpb_transact(decoder, NULL, DDL_DPB_OP_INIT);
	if (ddl_decoder_dpb_init(ddl) == VCD_ERR_FAIL)
		return VCD_ERR_FAIL;
	DDL_MSG_LOW("ddl_state_transition: %s ~~> DDL_CLIENT_WAIT_FOR_DPBDONE",
	ddl_get_state_string(ddl->client_state));
	ddl->client_state = DDL_CLIENT_WAIT_FOR_DPBDONE;
	ddl->cmd_state = DDL_CMD_DECODE_SET_DPB;
	if (decoder->cont_mode) {
		size_y = ddl_get_yuv_buf_size(decoder->client_frame_size.width,
				decoder->client_frame_size.height,
				DDL_YUV_BUF_TYPE_TILE);
		size_c = ddl_get_yuv_buf_size(decoder->client_frame_size.width,
				(decoder->client_frame_size.height >> 1),
				DDL_YUV_BUF_TYPE_TILE);
		vidc_sm_set_allocated_dpb_size(
			&ddl->shared_mem[ddl->command_channel],
			size_y,
			size_c);
	} else {
		vidc_sm_set_allocated_dpb_size(
			&ddl->shared_mem[ddl->command_channel],
			decoder->dpb_buf_size.size_y,
			decoder->dpb_buf_size.size_c);
	}
	init_buf_param.cmd_seq_num = ++ddl_context->cmd_seq_num;
	init_buf_param.inst_id = ddl->instance_id;
	init_buf_param.shared_mem_addr_offset = DDL_ADDR_OFFSET(
				ddl_context->dram_base_a, ddl->shared_mem
				[ddl->command_channel]);
	init_buf_param.dpb_count = decoder->dp_buf.no_of_dec_pic_buf;
	init_buf_param.dmx_disable = decoder->dmx_disable;
	ddl_context->vidc_decode_init_buffers[ddl->command_channel] (
		&init_buf_param);
	return VCD_S_SUCCESS;
}

void ddl_vidc_decode_frame_run(struct ddl_client_context *ddl)
{
	struct ddl_context *ddl_context = ddl->ddl_context;
	struct ddl_decoder_data *decoder = &(ddl->codec_data.decoder);
	struct vcd_frame_data *bit_stream =
		&(ddl->input_frame.vcd_frm);
	struct ddl_dec_buffers *dec_buffers = &decoder->hw_bufs;
	struct ddl_mask *dpb_mask = &ddl->codec_data.decoder.dpb_mask;
	struct vidc_1080p_dec_frame_start_param dec_param;
	u32 dpb_addr_y[32], index;
	ddl_set_core_start_time(__func__, DEC_OP_TIME);
	ddl_set_core_start_time(__func__, DEC_IP_TIME);
	if ((!bit_stream->data_len) || (!bit_stream->physical)) {
		ddl_vidc_decode_eos_run(ddl);
		return;
	}
	DDL_MSG_LOW("ddl_state_transition: %s ~~"
		"DDL_CLIENT_WAIT_FOR_FRAME_DONE",
		ddl_get_state_string(ddl->client_state));
	ddl->client_state = DDL_CLIENT_WAIT_FOR_FRAME_DONE;
	ddl_vidc_decode_dynamic_property(ddl, true);
	ddl_decoder_dpb_transact(decoder, NULL, DDL_DPB_OP_SET_MASK);
	ddl->cmd_state = DDL_CMD_DECODE_FRAME;
	dec_param.cmd_seq_num = ++ddl_context->cmd_seq_num;
	dec_param.inst_id = ddl->instance_id;
	dec_param.shared_mem_addr_offset = DDL_ADDR_OFFSET(
				ddl_context->dram_base_a, ddl->shared_mem
				[ddl->command_channel]);
	dec_param.stream_buffer_addr_offset = DDL_OFFSET(
			ddl_context->dram_base_a.align_physical_addr,
			bit_stream->physical);
	dec_param.stream_frame_size = bit_stream->data_len;
	dec_param.stream_buffersize = decoder->client_input_buf_req.sz;
	dec_param.descriptor_buffer_addr_offset = DDL_ADDR_OFFSET(
	ddl_context->dram_base_a, dec_buffers->desc);
	dec_param.descriptor_buffer_size = dec_buffers->desc.buffer_size;
	dec_param.release_dpb_bit_mask = dpb_mask->hw_mask;
	dec_param.decode = VIDC_1080P_DEC_TYPE_FRAME_DATA;
	dec_param.dpb_count = decoder->dp_buf.no_of_dec_pic_buf;
	dec_param.dmx_disable = decoder->dmx_disable;
	if (decoder->dmx_disable)
		ddl_fill_dec_desc_buffer(ddl);
	if (decoder->flush_pending) {
		dec_param.dpb_flush = true;
		decoder->flush_pending = false;
	} else
		dec_param.dpb_flush = false;
	ddl_set_vidc_timeout(ddl);
	vidc_sm_set_frame_tag(&ddl->shared_mem[ddl->command_channel],
		bit_stream->ip_frm_tag);
	if (ddl_context->pix_cache_enable) {
		for (index = 0; index <
			decoder->dp_buf.no_of_dec_pic_buf; index++) {
			dpb_addr_y[index] = (u32)
			decoder->dp_buf.dec_pic_buffers
				[index].vcd_frm.physical;
		}
		vidc_pix_cache_init_luma_chroma_base_addr(
			decoder->dp_buf.no_of_dec_pic_buf,
			dpb_addr_y, NULL);
		vidc_pix_cache_set_frame_range(decoder->dpb_buf_size.size_y,
			decoder->dpb_buf_size.size_c);
		vidc_pix_cache_clear_cache_tags();
	}
	ddl_context->vidc_decode_frame_start[ddl->command_channel] (
		&dec_param);
}

void ddl_vidc_decode_eos_run(struct ddl_client_context *ddl)
{
	struct ddl_context *ddl_context = ddl->ddl_context;
	struct ddl_decoder_data *decoder = &(ddl->codec_data.decoder);
	struct vcd_frame_data *bit_stream =
		&(ddl->input_frame.vcd_frm);
	struct ddl_dec_buffers *dec_buffers = &(decoder->hw_bufs);
	struct ddl_mask *dpb_mask =
		&(ddl->codec_data.decoder.dpb_mask);
	struct vidc_1080p_dec_frame_start_param dec_param;

	DDL_MSG_LOW("ddl_state_transition: %s ~~> DDL_CLIENT_WAIT_FOR_EOS_DONE",
	ddl_get_state_string(ddl->client_state));
	ddl->client_state = DDL_CLIENT_WAIT_FOR_EOS_DONE;
	if (decoder->output_order == VCD_DEC_ORDER_DECODE)
		decoder->dynamic_prop_change |= DDL_DEC_REQ_OUTPUT_FLUSH;
	ddl_vidc_decode_dynamic_property(ddl, true);
	ddl_decoder_dpb_transact(decoder, NULL, DDL_DPB_OP_SET_MASK);
	decoder->dynmic_prop_change_req = true;
	ddl->cmd_state = DDL_CMD_EOS;
	memset(&dec_param, 0, sizeof(dec_param));
	dec_param.cmd_seq_num = ++ddl_context->cmd_seq_num;
	dec_param.inst_id = ddl->instance_id;
	dec_param.shared_mem_addr_offset = DDL_ADDR_OFFSET(
			ddl_context->dram_base_a,
			ddl->shared_mem[ddl->command_channel]);
	dec_param.descriptor_buffer_addr_offset = DDL_ADDR_OFFSET(
	ddl_context->dram_base_a, dec_buffers->desc);
	dec_param.descriptor_buffer_size = dec_buffers->desc.buffer_size;
	dec_param.release_dpb_bit_mask = dpb_mask->hw_mask;
	dec_param.decode = VIDC_1080P_DEC_TYPE_LAST_FRAME_DATA;
	dec_param.dpb_count = decoder->dp_buf.no_of_dec_pic_buf;
	if (decoder->flush_pending) {
		dec_param.dpb_flush = true;
		decoder->flush_pending = false;
	} else
		dec_param.dpb_flush = false;
	vidc_sm_set_frame_tag(&ddl->shared_mem[ddl->command_channel],
	bit_stream->ip_frm_tag);
	ddl_context->vidc_decode_frame_start[ddl->command_channel] (
		&dec_param);
}

void ddl_vidc_encode_eos_run(struct ddl_client_context *ddl)
{
	struct vidc_1080p_enc_frame_start_param enc_param;
	struct ddl_context *ddl_context = ddl->ddl_context;
	struct ddl_encoder_data *encoder = &(ddl->codec_data.encoder);
	DDL_MSG_LOW("%s\n", __func__);
	ddl->client_state = DDL_CLIENT_WAIT_FOR_EOS_DONE;
	ddl_vidc_encode_dynamic_property(ddl, true);
	ddl->client_state = DDL_CMD_EOS;
	DDL_MEMSET(&enc_param, 0, sizeof(enc_param));
	enc_param.encode = VIDC_1080P_ENC_TYPE_LAST_FRAME_DATA;
	enc_param.cmd_seq_num = ++ddl_context->cmd_seq_num;
	enc_param.inst_id = ddl->instance_id;
	enc_param.shared_mem_addr_offset =
		DDL_ADDR_OFFSET(ddl_context->dram_base_a,
		ddl->shared_mem[ddl->command_channel]);
	enc_param.current_y_addr_offset = 0;
	enc_param.current_c_addr_offset = 0;
	enc_param.stream_buffer_size = 0;
	enc_param.intra_frame = encoder->intra_frame_insertion;
	vidc_sm_set_frame_tag(&ddl->shared_mem[ddl->command_channel],
				ddl->input_frame.vcd_frm.ip_frm_tag);
	ddl_context->vidc_encode_frame_start[ddl->command_channel](
						&enc_param);
}

int ddl_vidc_decode_get_avg_time(struct ddl_client_context *ddl)
{
	int avg_time = 0;
	struct ddl_decoder_data *decoder = &(ddl->codec_data.decoder);
	avg_time = decoder->avg_dec_time;
	return avg_time;
}

void ddl_vidc_decode_reset_avg_time(struct ddl_client_context *ddl)
{
	struct ddl_decoder_data *decoder = &(ddl->codec_data.decoder);
	decoder->avg_dec_time = 0;
	decoder->dec_time_sum = 0;
}
