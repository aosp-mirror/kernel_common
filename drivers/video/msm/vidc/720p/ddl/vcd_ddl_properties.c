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

#include <media/msm/vidc_type.h>
#include "vcd_ddl_utils.h"
#include "vcd_ddl_metadata.h"

static u32 ddl_set_dec_property(struct ddl_client_context *pddl,
				struct vcd_property_hdr *property_hdr,
				void *property_value);
static u32 ddl_set_enc_property(struct ddl_client_context *pddl,
				struct vcd_property_hdr *property_hdr,
				void *property_value);
static u32 ddl_get_dec_property(struct ddl_client_context *pddl,
				struct vcd_property_hdr *property_hdr,
				void *property_value);
static u32 ddl_get_enc_property(struct ddl_client_context *pddl,
				struct vcd_property_hdr *property_hdr,
				void *property_value);
static u32 ddl_set_enc_dynamic_property(struct ddl_client_context *ddl,
				struct vcd_property_hdr *property_hdr,
				void *property_value);
static void ddl_set_default_enc_property(struct ddl_client_context *ddl);
static void ddl_set_default_enc_profile(struct ddl_encoder_data
					*encoder);
static void ddl_set_default_enc_level(struct ddl_encoder_data *encoder);
static void ddl_set_default_enc_vop_timing(struct ddl_encoder_data
					   *encoder);
static void ddl_set_default_enc_intra_period(struct ddl_encoder_data
					     *encoder);
static void ddl_set_default_enc_rc_params(struct ddl_encoder_data
					  *encoder);
static u32 ddl_valid_buffer_requirement(struct vcd_buffer_requirement
					*original_buf_req,
					struct vcd_buffer_requirement
					*req_buf_req);
static u32 ddl_decoder_min_num_dpb(struct ddl_decoder_data *decoder);
static u32 ddl_set_dec_buffers
    (struct ddl_decoder_data *decoder,
     struct ddl_property_dec_pic_buffers *dpb);

u32 ddl_set_property(u32 *ddl_handle,
     struct vcd_property_hdr *property_hdr, void *property_value)
{
	u32 vcd_status;
	struct ddl_context *ddl_context;
	struct ddl_client_context *ddl =
	    (struct ddl_client_context *)ddl_handle;

	if (!property_hdr || !property_value) {
		VIDC_LOGERR_STRING("ddl_set_prop:Bad_argument");
		return VCD_ERR_ILLEGAL_PARM;
	}
	ddl_context = ddl_get_context();

	if (!DDL_IS_INITIALIZED(ddl_context)) {
		VIDC_LOGERR_STRING("ddl_set_prop:Not_inited");
		return VCD_ERR_ILLEGAL_OP;
	}

	if (!ddl) {
		VIDC_LOGERR_STRING("ddl_set_prop:Bad_handle");
		return VCD_ERR_BAD_HANDLE;
	}
	if (ddl->decoding) {
		vcd_status =
		    ddl_set_dec_property(ddl, property_hdr,
					 property_value);
	} else {
		vcd_status =
		    ddl_set_enc_property(ddl, property_hdr,
					 property_value);
	}
	if (vcd_status)
		VIDC_LOGERR_STRING("ddl_set_prop:FAILED");

	return vcd_status;
}

u32 ddl_get_property(u32 *ddl_handle,
     struct vcd_property_hdr *property_hdr, void *property_value)
{

	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	struct ddl_context *ddl_context;
	struct ddl_client_context *ddl =
	    (struct ddl_client_context *)ddl_handle;

	if (!property_hdr || !property_value)
		return VCD_ERR_ILLEGAL_PARM;

	if (property_hdr->prop_id == DDL_I_CAPABILITY) {
		if (sizeof(struct ddl_property_capability) ==
		    property_hdr->sz) {
			struct ddl_property_capability *ddl_capability =
			    (struct ddl_property_capability *)
			    property_value;
			ddl_capability->max_num_client = VCD_MAX_NO_CLIENT;
			ddl_capability->exclusive =
				VCD_COMMAND_EXCLUSIVE;
			ddl_capability->frame_command_depth =
				VCD_FRAME_COMMAND_DEPTH;
			ddl_capability->general_command_depth =
				VCD_GENERAL_COMMAND_DEPTH;
			ddl_capability->ddl_time_out_in_ms =
				DDL_HW_TIMEOUT_IN_MS;
			vcd_status = VCD_S_SUCCESS;
		}
		return vcd_status;
	}
	ddl_context = ddl_get_context();
	if (!DDL_IS_INITIALIZED(ddl_context))
		return VCD_ERR_ILLEGAL_OP;

	if (!ddl)
		return VCD_ERR_BAD_HANDLE;

	if (ddl->decoding) {
		vcd_status =
		    ddl_get_dec_property(ddl, property_hdr,
					 property_value);
	} else {
		vcd_status =
		    ddl_get_enc_property(ddl, property_hdr,
					 property_value);
	}
	if (vcd_status)
		VIDC_LOGERR_STRING("ddl_get_prop:FAILED");

	return vcd_status;
}

u32 ddl_decoder_ready_to_start(struct ddl_client_context *ddl,
     struct vcd_sequence_hdr *header)
{
	struct ddl_decoder_data *decoder = &(ddl->codec_data.decoder);
	if (!decoder->codec.codec) {
		VIDC_LOGERR_STRING("ddl_dec_start_check:Codec_not_set");
		return false;
	}
	if ((!header) &&
	    (!decoder->client_frame_size.height ||
	     !decoder->client_frame_size.width)
	    ) {
		VIDC_LOGERR_STRING
		    ("ddl_dec_start_check:Client_height_width_default");
		return false;
	}
	return true;
}

u32 ddl_encoder_ready_to_start(struct ddl_client_context *ddl)
{
	struct ddl_encoder_data *encoder = &(ddl->codec_data.encoder);

	if (!encoder->codec.codec ||
	    !encoder->frame_size.height ||
	    !encoder->frame_size.width ||
	    !encoder->frame_rate.fps_denominator ||
	    !encoder->frame_rate.fps_numerator ||
	    !encoder->target_bit_rate.target_bitrate) {
		return false;
	}
	return true;
}

static u32 ddl_set_dec_property
    (struct ddl_client_context *ddl,
     struct vcd_property_hdr *property_hdr, void *property_value) {
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	struct ddl_decoder_data *decoder = &(ddl->codec_data.decoder);
	switch (property_hdr->prop_id) {
	case DDL_I_DPB_RELEASE:
		{
			if (sizeof(struct ddl_frame_data_tag) ==
			    property_hdr->sz
			    && decoder->dp_buf.no_of_dec_pic_buf) {
				vcd_status =
				    ddl_decoder_dpb_transact(decoder,
					     (struct ddl_frame_data_tag *)
					     property_value,
					     DDL_DPB_OP_MARK_FREE);
			}
			break;
		}
	case DDL_I_DPB:
		{
			struct ddl_property_dec_pic_buffers *dpb =
			    (struct ddl_property_dec_pic_buffers *)
			    property_value;

			if (sizeof(struct ddl_property_dec_pic_buffers) ==
			    property_hdr->sz &&
			    (DDLCLIENT_STATE_IS
			     (ddl, DDL_CLIENT_WAIT_FOR_INITCODEC)
			     || DDLCLIENT_STATE_IS(ddl,
						   DDL_CLIENT_WAIT_FOR_DPB)
			    ) &&
			    dpb->no_of_dec_pic_buf >=
			    decoder->client_output_buf_req.actual_count) {
				vcd_status =
				    ddl_set_dec_buffers(decoder, dpb);
			}
			break;
		}
	case DDL_I_REQ_OUTPUT_FLUSH:
		{
			if (sizeof(u32) == property_hdr->sz) {
				decoder->dynamic_prop_change |=
				    DDL_DEC_REQ_OUTPUT_FLUSH;
				decoder->dpb_mask.client_mask = 0;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_INPUT_BUF_REQ:
		{
			struct vcd_buffer_requirement *buffer_req =
			    (struct vcd_buffer_requirement *)
			    property_value;
			if (sizeof(struct vcd_buffer_requirement) ==
			    property_hdr->sz &&
			    (ddl_valid_buffer_requirement(
						&decoder->min_input_buf_req,
						buffer_req))) {
				decoder->client_input_buf_req = *buffer_req;
				decoder->client_input_buf_req.min_count =
					decoder->min_input_buf_req.min_count;
				decoder->client_input_buf_req.max_count =
					decoder->min_input_buf_req.max_count;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_OUTPUT_BUF_REQ:
		{
			struct vcd_buffer_requirement *buffer_req =
			    (struct vcd_buffer_requirement *)
			    property_value;
			if (sizeof(struct vcd_buffer_requirement) ==
			    property_hdr->sz &&
			    (ddl_valid_buffer_requirement(
						&decoder->min_output_buf_req,
						buffer_req))) {
				decoder->client_output_buf_req =
				    *buffer_req;
				decoder->client_output_buf_req.min_count =
					decoder->min_output_buf_req.min_count;
				decoder->client_output_buf_req.max_count =
					decoder->min_output_buf_req.max_count;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}

	case VCD_I_CODEC:
		{
			struct vcd_property_codec *codec =
			    (struct vcd_property_codec *)property_value;
			if (sizeof(struct vcd_property_codec) ==
			    property_hdr->sz
			    && DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN)
			    ) {
				u32 status;
				vcd_fw_transact(false, true,
					decoder->codec.codec);
				status = vcd_fw_transact(true, true,
					codec->codec);
				if (status) {
					decoder->codec = *codec;
					ddl_set_default_dec_property(ddl);
					vcd_status = VCD_S_SUCCESS;
				} else {
					status = vcd_fw_transact(true, true,
						decoder->codec.codec);
					vcd_status = VCD_ERR_NOT_SUPPORTED;
				}
			}
			break;
		}
	case VCD_I_POST_FILTER:
		{
			if (sizeof(struct vcd_property_post_filter) ==
			    property_hdr->sz
			    && DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN) &&
			    (decoder->codec.codec == VCD_CODEC_MPEG4 ||
			     decoder->codec.codec == VCD_CODEC_MPEG2)
			    ) {
				decoder->post_filter =
				    *(struct vcd_property_post_filter *)
				    property_value;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_FRAME_SIZE:
		{
			struct vcd_property_frame_size *frame_size =
			    (struct vcd_property_frame_size *)
			    property_value;

			if ((sizeof(struct vcd_property_frame_size) ==
					property_hdr->sz) &&
				(DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN))) {
				if (decoder->client_frame_size.height !=
				    frame_size->height
				    || decoder->client_frame_size.width !=
				    frame_size->width) {
					decoder->client_frame_size =
					    *frame_size;
					ddl_set_default_decoder_buffer_req
					    (decoder, true);
				}
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_BUFFER_FORMAT:
		{
			struct vcd_property_buffer_format *tile =
			    (struct vcd_property_buffer_format *)
			    property_value;
			if (sizeof(struct vcd_property_buffer_format) ==
			    property_hdr->sz &&
			    DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN) &&
			    (tile->buffer_format == VCD_BUFFER_FORMAT_NV12
			     || tile->buffer_format ==
			     VCD_BUFFER_FORMAT_TILE_4x2)
			    ) {
				if (tile->buffer_format !=
				    decoder->buf_format.buffer_format) {
					decoder->buf_format = *tile;
					ddl_set_default_decoder_buffer_req
					    (decoder, true);
				}
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_METADATA_ENABLE:
	case VCD_I_METADATA_HEADER:
		{
			vcd_status = ddl_set_metadata_params(ddl,
							     property_hdr,
							     property_value);
			break;
		}
	case VCD_I_OUTPUT_ORDER:
		{
			if (sizeof(u32) == property_hdr->sz &&
				DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN)) {
					decoder->output_order =
						*(u32 *)property_value;
					vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_DEC_PICTYPE:
		{
			if ((sizeof(u32) == property_hdr->sz) &&
				DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN)) {
				decoder->idr_only_decoding =
					*(u32 *)property_value;
				ddl_set_default_decoder_buffer_req(
						decoder, true);
				vcd_status = VCD_S_SUCCESS;
			}
		}
		break;
	case VCD_I_FRAME_RATE:
		{
			vcd_status = VCD_S_SUCCESS;
			break;
		}
	default:
		{
			vcd_status = VCD_ERR_ILLEGAL_OP;
			break;
		}
	}
	return vcd_status;
}

static u32 ddl_set_enc_property(struct ddl_client_context *ddl,
	struct vcd_property_hdr *property_hdr, void *property_value)
{
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	struct ddl_encoder_data *encoder = &(ddl->codec_data.encoder);

	if (DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_FRAME) ||
	   (DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_FRAME_DONE) ||
		DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN)))
		vcd_status = ddl_set_enc_dynamic_property(ddl,
			property_hdr, property_value);
	if (vcd_status == VCD_S_SUCCESS)
		return vcd_status;

	if (!DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN) ||
		vcd_status != VCD_ERR_ILLEGAL_OP) {
		VIDC_LOGERR_STRING
			("ddl_set_enc_property:Fails_as_not_in_open_state");
		return VCD_ERR_ILLEGAL_OP;
	}

	switch (property_hdr->prop_id) {
	case VCD_I_FRAME_SIZE:
		{
			struct vcd_property_frame_size *framesize =
				(struct vcd_property_frame_size *)
				property_value;

			if (sizeof(struct vcd_property_frame_size)
				== property_hdr->sz &&
				DDL_ALLOW_ENC_FRAMESIZE(framesize->width,
				framesize->height) &&
				(encoder->codec.codec == VCD_CODEC_H264 ||
				 DDL_VALIDATE_ENC_FRAMESIZE(framesize->width,
				 framesize->height))
				) {
				encoder->frame_size = *framesize;
				ddl_calculate_stride(&encoder->frame_size,
					false, encoder->codec.codec);
				ddl_set_default_encoder_buffer_req(encoder);
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_CODEC:
		{
			struct vcd_property_codec *codec =
				(struct vcd_property_codec *)
				property_value;
			if (sizeof(struct vcd_property_codec) ==
				property_hdr->sz) {
				u32 status;

				vcd_fw_transact(false, false,
					encoder->codec.codec);

				status = vcd_fw_transact(true, false,
					codec->codec);
				if (status) {
					encoder->codec = *codec;
					ddl_set_default_enc_property(ddl);
					vcd_status = VCD_S_SUCCESS;
				} else {
					status = vcd_fw_transact(true, false,
						encoder->codec.codec);
					vcd_status = VCD_ERR_NOT_SUPPORTED;
				}
			}
			break;
		}
	case VCD_I_PROFILE:
		{
			struct vcd_property_profile *profile =
				(struct vcd_property_profile *)
				property_value;
			if ((sizeof(struct vcd_property_profile) ==
				property_hdr->sz) &&
				((encoder->codec.codec ==
					VCD_CODEC_MPEG4 &&
				  (profile->profile ==
					VCD_PROFILE_MPEG4_SP ||
					profile->profile ==
					VCD_PROFILE_MPEG4_ASP)) ||
				 (encoder->codec.codec ==
					VCD_CODEC_H264 &&
				 (profile->profile >=
					VCD_PROFILE_H264_BASELINE ||
				  profile->profile <=
					VCD_PROFILE_H264_HIGH)) ||
				 (encoder->codec.codec ==
					VCD_CODEC_H263 &&
				  profile->profile ==
					VCD_PROFILE_H263_BASELINE))
				) {
				encoder->profile = *profile;
				vcd_status = VCD_S_SUCCESS;

				if (profile->profile ==
					VCD_PROFILE_H264_BASELINE)
					encoder->entropy_control.entropy_sel
						= VCD_ENTROPY_SEL_CAVLC;
				else
					encoder->entropy_control.entropy_sel
						= VCD_ENTROPY_SEL_CABAC;
			}
			break;
		}
	case VCD_I_LEVEL:
		{
			struct vcd_property_level *level =
				(struct vcd_property_level *)
				property_value;
			if (
				(sizeof(struct vcd_property_level) ==
				 property_hdr->sz
				) &&
				(
				(
				(encoder->codec.
				 codec == VCD_CODEC_MPEG4) &&
				(level->level >= VCD_LEVEL_MPEG4_0) &&
				(level->level <= VCD_LEVEL_MPEG4_6)
				) ||
				(
				(encoder->codec.
				 codec == VCD_CODEC_H264) &&
				(level->level >= VCD_LEVEL_H264_1) &&
				(level->level <= VCD_LEVEL_H264_3p1)
				) ||
				(
				(encoder->codec.
				 codec == VCD_CODEC_H263) &&
				(level->level >= VCD_LEVEL_H263_10) &&
				(level->level <= VCD_LEVEL_H263_70)
				)
				)
				) {
				encoder->level = *level;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_MULTI_SLICE:
		{
			struct vcd_property_multi_slice *multislice =
				(struct vcd_property_multi_slice *)
				property_value;
			switch (multislice->m_slice_sel) {
			case VCD_MSLICE_OFF:
				{
					vcd_status = VCD_S_SUCCESS;
					break;
				}
			case VCD_MSLICE_BY_GOB:
				{
					if (encoder->codec.codec ==
						VCD_CODEC_H263)
						vcd_status = VCD_S_SUCCESS;
					 break;
				}
			case VCD_MSLICE_BY_MB_COUNT:
				{
					if (multislice->m_slice_size
						>= 1 && (multislice->
						m_slice_size <=
						(encoder->frame_size.height
						* encoder->frame_size.width
						/ 16 / 16))
						) {
						vcd_status = VCD_S_SUCCESS;
					}
					break;
				  }
			case VCD_MSLICE_BY_BYTE_COUNT:
				{
					if (multislice->m_slice_size > 0)
						vcd_status = VCD_S_SUCCESS;
					break;
				}
			default:
				{
					break;
				}
			}
			if (sizeof(struct vcd_property_multi_slice) ==
				property_hdr->sz &&
				!vcd_status) {
				encoder->multi_slice = *multislice;
				if (multislice->m_slice_sel ==
						VCD_MSLICE_OFF)
					encoder->multi_slice.m_slice_size = 0;
			}
			break;
		}
	case VCD_I_RATE_CONTROL:
		{
			struct vcd_property_rate_control
				*ratecontrol =
				(struct vcd_property_rate_control *)
				property_value;
			if (sizeof(struct vcd_property_rate_control) ==
				property_hdr->sz &&
				ratecontrol->
				rate_control >= VCD_RATE_CONTROL_OFF &&
				ratecontrol->
				rate_control <= VCD_RATE_CONTROL_CBR_CFR
				) {
				encoder->rc = *ratecontrol;
				ddl_set_default_enc_rc_params(encoder);
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_SHORT_HEADER:
		{

		if (sizeof(struct vcd_property_short_header) ==
			property_hdr->sz &&
			encoder->codec.codec == VCD_CODEC_MPEG4) {
			encoder->short_header =
				*(struct vcd_property_short_header *)
				property_value;
			vcd_status = VCD_S_SUCCESS;
		}

			break;
		}
	case VCD_I_VOP_TIMING:
		{
			struct vcd_property_vop_timing *voptime =
				(struct vcd_property_vop_timing *)
				property_value;
			if (
				(sizeof(struct vcd_property_vop_timing) ==
					  property_hdr->sz
				) &&
				(encoder->frame_rate.fps_numerator <=
					voptime->vop_time_resolution)
				) {
				encoder->vop_timing = *voptime;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_HEADER_EXTENSION:
		{
			if (sizeof(u32) == property_hdr->sz &&
				encoder->codec.codec == VCD_CODEC_MPEG4
				) {
				encoder->hdr_ext_control = *(u32 *)
					property_value;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_ENTROPY_CTRL:
		{
			struct vcd_property_entropy_control
				*entropy_control =
				(struct vcd_property_entropy_control *)
				property_value;
			if (sizeof(struct vcd_property_entropy_control) ==
				property_hdr->sz &&
				encoder->codec.codec == VCD_CODEC_H264
				&& entropy_control->
				entropy_sel >= VCD_ENTROPY_SEL_CAVLC &&
				entropy_control->entropy_sel <=
				VCD_ENTROPY_SEL_CABAC) {
				encoder->entropy_control = *entropy_control;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_DEBLOCKING:
		{
			struct vcd_property_db_config *dbconfig =
				(struct vcd_property_db_config *)
				property_value;
			if (sizeof(struct vcd_property_db_config) ==
				property_hdr->sz &&
				encoder->codec.codec == VCD_CODEC_H264
				&& dbconfig->db_config >=
				VCD_DB_ALL_BLOCKING_BOUNDARY
				&& dbconfig->db_config <=
				VCD_DB_SKIP_SLICE_BOUNDARY
				) {
				encoder->db_control = *dbconfig;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_QP_RANGE:
		{
			struct vcd_property_qp_range *qp =
				(struct vcd_property_qp_range *)
				property_value;
			if ((sizeof(struct vcd_property_qp_range) ==
				property_hdr->sz) &&
				(qp->min_qp <= qp->max_qp) &&
				(
				(encoder->codec.codec == VCD_CODEC_H264
				&& qp->max_qp <= DDL_MAX_H264_QP) ||
				(qp->max_qp <= DDL_MAX_MPEG4_QP)
				)
				) {
				encoder->qp_range = *qp;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_SESSION_QP:
		{
			struct vcd_property_session_qp *qp =
				(struct vcd_property_session_qp *)
				property_value;

		if ((sizeof(struct vcd_property_session_qp) ==
			property_hdr->sz) &&
			(qp->i_frame_qp >= encoder->qp_range.min_qp) &&
			(qp->i_frame_qp <= encoder->qp_range.max_qp) &&
			(qp->p_frame_qp >= encoder->qp_range.min_qp) &&
			(qp->p_frame_qp <= encoder->qp_range.max_qp)
			) {
			encoder->session_qp = *qp;
			vcd_status = VCD_S_SUCCESS;
		}

			break;
		}
	case VCD_I_RC_LEVEL_CONFIG:
		{
			struct vcd_property_rc_level *rc_level =
				(struct vcd_property_rc_level *)
				property_value;
			if (sizeof(struct vcd_property_rc_level) ==
				property_hdr->sz &&
				(
				encoder->rc.
				rate_control >= VCD_RATE_CONTROL_VBR_VFR ||
				encoder->rc.
				rate_control <= VCD_RATE_CONTROL_CBR_VFR
				) &&
				(!rc_level->mb_level_rc ||
				encoder->codec.codec == VCD_CODEC_H264
				)
				) {
				encoder->rc_level = *rc_level;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_FRAME_LEVEL_RC:
		{

		struct vcd_property_frame_level_rc_params
			*frame_levelrc =
			(struct vcd_property_frame_level_rc_params *)
			property_value;

			if ((sizeof(struct
				vcd_property_frame_level_rc_params)
				== property_hdr->sz) &&
				(frame_levelrc->reaction_coeff) &&
				(encoder->rc_level.frame_level_rc)
				) {
				encoder->frame_level_rc = *frame_levelrc;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_ADAPTIVE_RC:
		{

		if ((sizeof(struct
			vcd_property_adaptive_rc_params)
			== property_hdr->sz) &&
			(encoder->codec.
			codec == VCD_CODEC_H264) &&
			(encoder->rc_level.mb_level_rc)) {

			encoder->adaptive_rc =
				*(struct vcd_property_adaptive_rc_params *)
				property_value;

			vcd_status = VCD_S_SUCCESS;
		}

			break;
		}
	case VCD_I_BUFFER_FORMAT:
		{
			struct vcd_property_buffer_format *tile =
				(struct vcd_property_buffer_format *)
				property_value;
			if (sizeof(struct vcd_property_buffer_format) ==
				property_hdr->sz &&
				tile->buffer_format ==
				VCD_BUFFER_FORMAT_NV12) {
				encoder->buf_format = *tile;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_INPUT_BUF_REQ:
		{
			struct vcd_buffer_requirement *buffer_req =
				(struct vcd_buffer_requirement *)
				property_value;
			if (sizeof(struct vcd_buffer_requirement) ==
				property_hdr->sz &&
				(ddl_valid_buffer_requirement(
				&encoder->input_buf_req, buffer_req))
				) {
				encoder->client_input_buf_req = *buffer_req;
				encoder->client_input_buf_req.min_count =
					encoder->input_buf_req.min_count;
				encoder->client_input_buf_req.max_count =
					encoder->input_buf_req.max_count;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_OUTPUT_BUF_REQ:
		{
			struct vcd_buffer_requirement *buffer_req =
				(struct vcd_buffer_requirement *)
				property_value;
			if (sizeof(struct vcd_buffer_requirement) ==
				property_hdr->sz &&
				(ddl_valid_buffer_requirement(
				&encoder->output_buf_req, buffer_req))
				) {
				encoder->client_output_buf_req =
					*buffer_req;
				encoder->client_output_buf_req.min_count =
					encoder->output_buf_req.min_count;
				encoder->client_output_buf_req.max_count =
					encoder->output_buf_req.max_count;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_METADATA_ENABLE:
	case VCD_I_METADATA_HEADER:
		{
			vcd_status = ddl_set_metadata_params(
				ddl, property_hdr, property_value);
			break;
		}
	case VCD_I_META_BUFFER_MODE:
		{
			vcd_status = VCD_S_SUCCESS;
			break;
		}
	default:
		{
			vcd_status = VCD_ERR_ILLEGAL_OP;
			break;
		}
	}
	return vcd_status;
}

static u32 ddl_get_dec_property
    (struct ddl_client_context *ddl,
     struct vcd_property_hdr *property_hdr, void *property_value) {
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	struct ddl_decoder_data *decoder = &ddl->codec_data.decoder;

	switch (property_hdr->prop_id) {
	case VCD_I_FRAME_SIZE:
		{
			struct vcd_property_frame_size *fz_size;
			if (sizeof(struct vcd_property_frame_size) ==
			    property_hdr->sz) {
					ddl_calculate_stride(
					&decoder->client_frame_size,
					!decoder->progressive_only,
					decoder->codec.codec);
					if (decoder->buf_format.buffer_format
						== VCD_BUFFER_FORMAT_TILE_4x2) {
						fz_size =
						&decoder->client_frame_size;
						fz_size->stride =
						DDL_TILE_ALIGN(fz_size->width,
							DDL_TILE_ALIGN_WIDTH);
						fz_size->scan_lines =
						DDL_TILE_ALIGN(fz_size->height,
							DDL_TILE_ALIGN_HEIGHT);
					}
					*(struct vcd_property_frame_size *)
						property_value =
						decoder->client_frame_size;
					vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_PROFILE:
		{
			if (sizeof(struct vcd_property_profile) ==
			    property_hdr->sz) {
				*(struct vcd_property_profile *)
				    property_value = decoder->profile;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_LEVEL:
		{
			if (sizeof(struct vcd_property_level) ==
			    property_hdr->sz) {
				*(struct vcd_property_level *)
				    property_value = decoder->level;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_PROGRESSIVE_ONLY:
		{
			if (sizeof(u32) == property_hdr->sz) {
				*(u32 *) property_value =
				    decoder->progressive_only;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_INPUT_BUF_REQ:
		{
			if (sizeof(struct vcd_buffer_requirement) ==
			    property_hdr->sz) {
				*(struct vcd_buffer_requirement *)
				    property_value =
						decoder->client_input_buf_req;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_OUTPUT_BUF_REQ:
		{
			if (sizeof(struct vcd_buffer_requirement) ==
			    property_hdr->sz) {
				*(struct vcd_buffer_requirement *)
				    property_value =
						decoder->client_output_buf_req;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_CODEC:
		{
			if (sizeof(struct vcd_property_codec) ==
			    property_hdr->sz) {
				*(struct vcd_property_codec *)
				    property_value = decoder->codec;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_BUFFER_FORMAT:
		{
			if (sizeof(struct vcd_property_buffer_format) ==
			    property_hdr->sz) {
				*(struct vcd_property_buffer_format *)
				    property_value = decoder->buf_format;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_POST_FILTER:
		{
			if (sizeof(struct vcd_property_post_filter) ==
			    property_hdr->sz) {
				*(struct vcd_property_post_filter *)
				    property_value = decoder->post_filter;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_SEQHDR_ALIGN_BYTES:
		{
			if (sizeof(u32) == property_hdr->sz) {
				*(u32 *) property_value =
				    DDL_LINEAR_BUFFER_ALIGN_BYTES;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_FRAME_PROC_UNITS:
		{
			if (sizeof(u32) == property_hdr->sz) {
				struct vcd_property_frame_size frame_sz =
					decoder->client_frame_size;
				ddl_calculate_stride(&frame_sz,
					!decoder->progressive_only,
					decoder->codec.codec);
				*(u32 *) property_value =
				    ((frame_sz.stride >> 4) *
				     (frame_sz.scan_lines >> 4));
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_DPB_RETRIEVE:
		{
			if (sizeof(struct ddl_frame_data_tag) ==
			    property_hdr->sz) {
				vcd_status =
				    ddl_decoder_dpb_transact(decoder,
					 (struct ddl_frame_data_tag *)
					     property_value,
					     DDL_DPB_OP_RETRIEVE);
			}
			break;
		}
	case VCD_I_OUTPUT_ORDER:
		{
			if (sizeof(u32) == property_hdr->sz) {
				*(u32 *)property_value = decoder->output_order;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_METADATA_ENABLE:
	case VCD_I_METADATA_HEADER:
		{
			vcd_status = ddl_get_metadata_params(
						   ddl,
						   property_hdr,
						   property_value);
			break;
		}
	default:
		{
			vcd_status = VCD_ERR_ILLEGAL_OP;
			break;
		}
	}
	return vcd_status;
}

static u32 ddl_get_enc_property
    (struct ddl_client_context *ddl,
     struct vcd_property_hdr *property_hdr, void *property_value) {
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM;
	struct ddl_encoder_data *encoder = &ddl->codec_data.encoder;

	struct vcd_property_entropy_control *entropy_control;
	struct vcd_property_intra_refresh_mb_number *intra_refresh;

	switch (property_hdr->prop_id) {
	case VCD_I_CODEC:
		{
			if (sizeof(struct vcd_property_codec) ==
			    property_hdr->sz) {
				*(struct vcd_property_codec *)
					property_value =
					encoder->codec;
		    vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_FRAME_SIZE:
		{
			if (sizeof(struct vcd_property_frame_size) ==
			    property_hdr->sz) {
				*(struct vcd_property_frame_size *)
					property_value =
					encoder->frame_size;

				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_FRAME_RATE:
		{
			if (sizeof(struct vcd_property_frame_rate) ==
				property_hdr->sz) {

				*(struct vcd_property_frame_rate *)
					property_value =
					encoder->frame_rate;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_TARGET_BITRATE:
		{

			if (sizeof(struct vcd_property_target_bitrate) ==
			    property_hdr->sz) {
				*(struct vcd_property_target_bitrate *)
					property_value =
					encoder->target_bit_rate;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_RATE_CONTROL:
		{
			if (sizeof(struct vcd_property_rate_control) ==
			    property_hdr->sz) {
				*(struct vcd_property_rate_control *)
				    property_value = encoder->rc;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_PROFILE:
		{
			if (sizeof(struct vcd_property_profile) ==
			    property_hdr->sz) {
				*(struct vcd_property_profile *)
				    property_value = encoder->profile;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_LEVEL:
		{
			if (sizeof(struct vcd_property_level) ==
			    property_hdr->sz) {
				*(struct vcd_property_level *)
				    property_value = encoder->level;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_MULTI_SLICE:
		{
			if (sizeof(struct vcd_property_multi_slice) ==
			    property_hdr->sz) {
				*(struct vcd_property_multi_slice *)
				    property_value = encoder->multi_slice;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_SEQ_HEADER:
		{
			struct vcd_sequence_hdr *seq_hdr =
			    (struct vcd_sequence_hdr *)property_value;
			if (encoder->seq_header.buffer_size &&
			    sizeof(struct vcd_sequence_hdr) ==
			    property_hdr->sz
			    && encoder->seq_header.buffer_size <=
			    seq_hdr->sequence_header_len) {
				DDL_MEMCPY(seq_hdr->sequence_header,
					   encoder->seq_header.
					   align_virtual_addr,
					   encoder->seq_header.buffer_size);
				seq_hdr->sequence_header_len =
				    encoder->seq_header.buffer_size;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_SEQHDR_PRESENT:
		{
			if (sizeof(u32) == property_hdr->sz) {
				if ((encoder->codec.
					codec == VCD_CODEC_MPEG4 &&
					!encoder->short_header.short_header)
					|| encoder->codec.codec ==
					VCD_CODEC_H264) {
					*(u32 *)property_value = 0x1;
				} else {
					*(u32 *)property_value = 0x0;
				}
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_VOP_TIMING:
		{
			if (sizeof(struct vcd_property_vop_timing) ==
			    property_hdr->sz) {
				*(struct vcd_property_vop_timing *)
				    property_value = encoder->vop_timing;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_SHORT_HEADER:
		{
			if (sizeof(struct vcd_property_short_header) ==
			    property_hdr->sz) {
				if (encoder->codec.codec ==
					VCD_CODEC_MPEG4) {
					*(struct vcd_property_short_header
					  *)property_value =
						encoder->short_header;
					vcd_status = VCD_S_SUCCESS;
				} else {
					vcd_status = VCD_ERR_ILLEGAL_OP;
				}
			}
			break;
		}
	case VCD_I_ENTROPY_CTRL:
		{
			entropy_control = property_value;
			if (sizeof(struct vcd_property_entropy_control) ==
			    property_hdr->sz) {
				if (encoder->codec.codec ==
					VCD_CODEC_H264) {
					*entropy_control =
				     encoder->entropy_control;
					vcd_status = VCD_S_SUCCESS;
				} else {
					vcd_status = VCD_ERR_ILLEGAL_OP;
				}
			}
			break;
		}
	case VCD_I_DEBLOCKING:
		{
			if (sizeof(struct vcd_property_db_config) ==
			    property_hdr->sz) {
				if (encoder->codec.codec ==
					VCD_CODEC_H264) {
					*(struct vcd_property_db_config *)
					    property_value =
					    encoder->db_control;
					vcd_status = VCD_S_SUCCESS;
				} else {
					vcd_status = VCD_ERR_ILLEGAL_OP;
				}
			}
			break;
		}
	case VCD_I_INTRA_PERIOD:
		{
			if (sizeof(struct vcd_property_i_period) ==
			    property_hdr->sz) {
				*(struct vcd_property_i_period *)
				    property_value = encoder->i_period;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_QP_RANGE:
		{
			if (sizeof(struct vcd_property_qp_range) ==
			    property_hdr->sz) {
				*(struct vcd_property_qp_range *)
				    property_value = encoder->qp_range;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_SESSION_QP:
		{
			if (sizeof(struct vcd_property_session_qp) ==
			    property_hdr->sz) {
				*(struct vcd_property_session_qp *)
				    property_value = encoder->session_qp;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_RC_LEVEL_CONFIG:
		{
			if (sizeof(struct vcd_property_rc_level) ==
			    property_hdr->sz) {
				*(struct vcd_property_rc_level *)
				    property_value = encoder->rc_level;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_FRAME_LEVEL_RC:
		{
			if (sizeof
			    (struct vcd_property_frame_level_rc_params) ==
			    property_hdr->sz) {
				*(struct vcd_property_frame_level_rc_params
				 *)property_value =
				 encoder->frame_level_rc;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_ADAPTIVE_RC:
		{
			if (sizeof(struct vcd_property_adaptive_rc_params)
			    == property_hdr->sz) {
				*(struct vcd_property_adaptive_rc_params *)
				    property_value = encoder->adaptive_rc;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_INTRA_REFRESH:
		{
			intra_refresh = property_value;
			if (sizeof
			    (struct vcd_property_intra_refresh_mb_number)
			    == property_hdr->sz) {
				*intra_refresh = encoder->intra_refresh;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_INPUT_BUF_REQ:
		{
			if (sizeof(struct vcd_buffer_requirement) ==
			    property_hdr->sz) {
				*(struct vcd_buffer_requirement *)
				    property_value =
						encoder->client_input_buf_req;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_OUTPUT_BUF_REQ:
		{
			if (sizeof(struct vcd_buffer_requirement) ==
			    property_hdr->sz) {
				*(struct vcd_buffer_requirement *)
				    property_value =
						encoder->client_output_buf_req;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_BUFFER_FORMAT:
		{
			if (sizeof(struct vcd_property_buffer_format) ==
			    property_hdr->sz) {
				*(struct vcd_property_buffer_format *)
				    property_value = encoder->buf_format;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case DDL_I_FRAME_PROC_UNITS:
		{
			if (sizeof(u32) == property_hdr->sz) {
				*(u32 *) property_value =
				    ((encoder->frame_size.width >> 4) *
				     (encoder->frame_size.height >> 4)
				    );
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_HEADER_EXTENSION:
		{
			if (sizeof(u32) == property_hdr->sz &&
			    encoder->codec.codec == VCD_CODEC_MPEG4) {
				*(u32 *) property_value =
				    encoder->hdr_ext_control;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_METADATA_ENABLE:
	case VCD_I_METADATA_HEADER:
		{
			vcd_status = ddl_get_metadata_params(
						   ddl,
						   property_hdr,
						   property_value);
			break;
		}
	default:
		{
			vcd_status = VCD_ERR_ILLEGAL_OP;
			break;
		}
	}
	return vcd_status;
}

static u32 ddl_set_enc_dynamic_property
    (struct ddl_client_context *ddl,
     struct vcd_property_hdr *property_hdr, void *property_value) {
	struct ddl_encoder_data *encoder = &(ddl->codec_data.encoder);
	u32 vcd_status = VCD_ERR_ILLEGAL_PARM, dynamic_prop_change = 0x0;
	switch (property_hdr->prop_id) {
	case VCD_I_REQ_IFRAME:
		{
			if (sizeof(struct vcd_property_req_i_frame) ==
			    property_hdr->sz) {
				dynamic_prop_change = DDL_ENC_REQ_IFRAME;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_TARGET_BITRATE:
		{
		    struct vcd_property_target_bitrate *bitrate =
				(struct vcd_property_target_bitrate *)
				property_value;
			if (sizeof(struct vcd_property_target_bitrate) ==
			 property_hdr->sz && bitrate->target_bitrate > 0
			 && bitrate->target_bitrate <= DDL_MAX_BIT_RATE) {
				encoder->target_bit_rate = *bitrate;
				dynamic_prop_change = DDL_ENC_CHANGE_BITRATE;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_INTRA_PERIOD:
		{
			struct vcd_property_i_period *iperiod =
				(struct vcd_property_i_period *)
				property_value;
			if (sizeof(struct vcd_property_i_period) ==
				property_hdr->sz &&
				!iperiod->b_frames) {
				encoder->i_period = *iperiod;
				dynamic_prop_change = DDL_ENC_CHANGE_IPERIOD;
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_FRAME_RATE:
		{
			struct vcd_property_frame_rate *frame_rate =
			    (struct vcd_property_frame_rate *)
			    property_value;
			if (sizeof(struct vcd_property_frame_rate)
			    == property_hdr->sz &&
			    frame_rate->fps_denominator &&
			    frame_rate->fps_numerator &&
			    frame_rate->fps_denominator <=
			    frame_rate->fps_numerator) {
				encoder->frame_rate = *frame_rate;
				dynamic_prop_change = DDL_ENC_CHANGE_FRAMERATE;
				if (DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_OPEN) &&
					(encoder->codec.codec != VCD_CODEC_MPEG4
					 || encoder->short_header.short_header))
					ddl_set_default_enc_vop_timing(encoder);
				vcd_status = VCD_S_SUCCESS;
			}
			break;
		}
	case VCD_I_INTRA_REFRESH:
		{
			struct vcd_property_intra_refresh_mb_number
				*intra_refresh_mbnum = (
				struct vcd_property_intra_refresh_mb_number *)
					property_value;
			u32 frame_mbnum =
				(encoder->frame_size.width >> 4) *
				(encoder->frame_size.height >> 4);
			if (sizeof(struct
				vcd_property_intra_refresh_mb_number)
				== property_hdr->sz &&
				intra_refresh_mbnum->cir_mb_number <=
				frame_mbnum) {
				encoder->intra_refresh =
					*intra_refresh_mbnum;
				dynamic_prop_change = DDL_ENC_CHANGE_CIR;
				vcd_status = VCD_S_SUCCESS;
			}

			break;
		}
	default:
		{
			vcd_status = VCD_ERR_ILLEGAL_OP;
			break;
		}
	}
	if (vcd_status == VCD_S_SUCCESS &&
	(DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_FRAME) ||
	DDLCLIENT_STATE_IS(ddl, DDL_CLIENT_WAIT_FOR_FRAME_DONE)))
		encoder->dynamic_prop_change |= dynamic_prop_change;
	return vcd_status;
}

void ddl_set_default_dec_property(struct ddl_client_context *ddl)
{
	struct ddl_decoder_data *decoder = &(ddl->codec_data.decoder);

	if (decoder->codec.codec >= VCD_CODEC_MPEG2 &&
		decoder->codec.codec <=  VCD_CODEC_XVID)
		decoder->post_filter.post_filter = true;
	else
		decoder->post_filter.post_filter = false;
	decoder->buf_format.buffer_format = VCD_BUFFER_FORMAT_NV12;
	decoder->client_frame_size.height = 144;
	decoder->client_frame_size.width = 176;
	decoder->client_frame_size.stride = 176;
	decoder->client_frame_size.scan_lines = 144;
	decoder->progressive_only = 1;
	decoder->idr_only_decoding = 0;
	decoder->profile.profile = VCD_PROFILE_UNKNOWN;
	decoder->level.level = VCD_LEVEL_UNKNOWN;
	decoder->output_order = VCD_DEC_ORDER_DISPLAY;
	ddl_set_default_metadata_flag(ddl);
	ddl_set_default_decoder_buffer_req(decoder, true);
}

static void ddl_set_default_enc_property(struct ddl_client_context *ddl)
{
	struct ddl_encoder_data *encoder = &(ddl->codec_data.encoder);

	ddl_set_default_enc_profile(encoder);
	ddl_set_default_enc_level(encoder);

	encoder->rc.rate_control = VCD_RATE_CONTROL_VBR_VFR;
	ddl_set_default_enc_rc_params(encoder);

	ddl_set_default_enc_intra_period(encoder);

	encoder->intra_refresh.cir_mb_number = 0;
	ddl_set_default_enc_vop_timing(encoder);

	encoder->multi_slice.m_slice_sel = VCD_MSLICE_OFF;
	encoder->multi_slice.m_slice_size = 0;
	encoder->short_header.short_header = false;

	encoder->entropy_control.entropy_sel = VCD_ENTROPY_SEL_CAVLC;
	encoder->entropy_control.cabac_model = VCD_CABAC_MODEL_NUMBER_0;
	encoder->db_control.db_config = VCD_DB_ALL_BLOCKING_BOUNDARY;
	encoder->db_control.slice_alpha_offset = 0;
	encoder->db_control.slice_beta_offset = 0;

	encoder->re_con_buf_format.buffer_format =
		VCD_BUFFER_FORMAT_TILE_4x2;

	encoder->buf_format.buffer_format = VCD_BUFFER_FORMAT_NV12;

	encoder->hdr_ext_control = 0;

	ddl_set_default_metadata_flag(ddl);

	ddl_set_default_encoder_buffer_req(encoder);
}

static void ddl_set_default_enc_profile(struct ddl_encoder_data *encoder)
{
	enum vcd_codec codec = encoder->codec.codec;
	if (codec == VCD_CODEC_MPEG4)
		encoder->profile.profile = VCD_PROFILE_MPEG4_SP;
	else if (codec == VCD_CODEC_H264)
		encoder->profile.profile = VCD_PROFILE_H264_BASELINE;
	else
		encoder->profile.profile = VCD_PROFILE_H263_BASELINE;
}

static void ddl_set_default_enc_level(struct ddl_encoder_data *encoder)
{
	enum vcd_codec codec = encoder->codec.codec;
	if (codec == VCD_CODEC_MPEG4)
		encoder->level.level = VCD_LEVEL_MPEG4_1;
	else if (codec == VCD_CODEC_H264)
		encoder->level.level = VCD_LEVEL_H264_1;
	else
		encoder->level.level = VCD_LEVEL_H263_10;
}

static void ddl_set_default_enc_vop_timing
    (struct ddl_encoder_data *encoder)
{
	if (encoder->codec.codec == VCD_CODEC_MPEG4)
		encoder->vop_timing.vop_time_resolution =
		    (2 * encoder->frame_rate.fps_numerator) /
		    encoder->frame_rate.fps_denominator;
	else
		encoder->vop_timing.vop_time_resolution = 0x7530;
}

static void ddl_set_default_enc_intra_period(
		struct ddl_encoder_data *encoder)
{
	switch (encoder->rc.rate_control) {
	default:
	case VCD_RATE_CONTROL_VBR_VFR:
	case VCD_RATE_CONTROL_VBR_CFR:
	case VCD_RATE_CONTROL_CBR_VFR:
	case VCD_RATE_CONTROL_OFF:
		{
			encoder->i_period.p_frames =
			    ((encoder->frame_rate.fps_numerator << 1) /
			     encoder->frame_rate.fps_denominator) - 1;
			break;
		}
	case VCD_RATE_CONTROL_CBR_CFR:
		{
			encoder->i_period.p_frames =
			    ((encoder->frame_rate.fps_numerator >> 1) /
			     encoder->frame_rate.fps_denominator) - 1;
			break;
		}
	}
	encoder->i_period.b_frames = 0;
}

static void ddl_set_default_enc_rc_params(
		struct ddl_encoder_data *encoder)
{
	enum vcd_codec codec = encoder->codec.codec;

	encoder->rc_level.frame_level_rc = true;
	encoder->qp_range.min_qp = 0x1;

	if (codec == VCD_CODEC_H264) {
		encoder->qp_range.max_qp = 0x33;
		encoder->session_qp.i_frame_qp = 0x14;
		encoder->session_qp.p_frame_qp = 0x14;

		encoder->rc_level.mb_level_rc = true;
		encoder->adaptive_rc.activity_region_flag = true;
		encoder->adaptive_rc.dark_region_as_flag = true;
		encoder->adaptive_rc.smooth_region_as_flag = true;
		encoder->adaptive_rc.static_region_as_flag = true;
	} else {
		encoder->qp_range.max_qp = 0x1f;
		encoder->session_qp.i_frame_qp = 0xd;
		encoder->session_qp.p_frame_qp = 0xd;
		encoder->rc_level.mb_level_rc = false;
	}

	switch (encoder->rc.rate_control) {
	default:
	case VCD_RATE_CONTROL_VBR_VFR:
		{
			encoder->r_cframe_skip = 1;
			encoder->frame_level_rc.reaction_coeff = 0x1f4;
			break;
		}
	case VCD_RATE_CONTROL_VBR_CFR:
		{
			encoder->r_cframe_skip = 0;
			encoder->frame_level_rc.reaction_coeff = 0x1f4;
			break;
		}
	case VCD_RATE_CONTROL_CBR_VFR:
		{
			encoder->r_cframe_skip = 1;
			if (codec != VCD_CODEC_H264) {
				encoder->session_qp.i_frame_qp = 0xf;
				encoder->session_qp.p_frame_qp = 0xf;
			}

			encoder->frame_level_rc.reaction_coeff = 0x14;
			break;
		}
	case VCD_RATE_CONTROL_CBR_CFR:
		{
			encoder->r_cframe_skip = 0;
			encoder->frame_level_rc.reaction_coeff = 0x6;
			break;
		}
	case VCD_RATE_CONTROL_OFF:
		{
			encoder->r_cframe_skip = 0;
			encoder->rc_level.frame_level_rc = false;
			encoder->rc_level.mb_level_rc = false;
			break;
		}
	}
}

void ddl_set_default_encoder_buffer_req(struct ddl_encoder_data *encoder)
{
	u32 y_cb_cr_size;

	y_cb_cr_size = ddl_get_yuv_buffer_size(&encoder->frame_size,
		&encoder->buf_format, false, encoder->codec.codec);

	memset(&encoder->input_buf_req, 0,
	       sizeof(struct vcd_buffer_requirement));

	encoder->input_buf_req.min_count = 1;
	encoder->input_buf_req.actual_count =
	    encoder->input_buf_req.min_count + 8;
	encoder->input_buf_req.max_count = DDL_MAX_BUFFER_COUNT;
	encoder->input_buf_req.sz = y_cb_cr_size;
	encoder->input_buf_req.align = DDL_LINEAR_BUFFER_ALIGN_BYTES;

	encoder->client_input_buf_req = encoder->input_buf_req;

	memset(&encoder->output_buf_req, 0,
	       sizeof(struct vcd_buffer_requirement));

	encoder->output_buf_req.min_count = 2;
	encoder->output_buf_req.actual_count =
	    encoder->output_buf_req.min_count + 3;
	encoder->output_buf_req.max_count = DDL_MAX_BUFFER_COUNT;
	encoder->output_buf_req.align = DDL_LINEAR_BUFFER_ALIGN_BYTES;
	encoder->output_buf_req.sz = y_cb_cr_size;
	ddl_set_default_encoder_metadata_buffer_size(encoder);
	encoder->client_output_buf_req = encoder->output_buf_req;
}

void ddl_set_default_decoder_buffer_req(struct ddl_decoder_data *decoder,
		u32 estimate)
{
	u32 y_cb_cr_size, min_dpb, num_mb;
	struct vcd_property_frame_size  *frame_size;
	struct vcd_buffer_requirement *output_buf_req, *input_buf_req;

	if (!decoder->codec.codec)
		return;

	if (estimate) {
		frame_size = &decoder->client_frame_size;
		output_buf_req = &decoder->client_output_buf_req;
		input_buf_req = &decoder->client_input_buf_req;
		min_dpb = ddl_decoder_min_num_dpb(decoder);
		 y_cb_cr_size = ddl_get_yuv_buffer_size(frame_size,
			&decoder->buf_format, (!decoder->progressive_only),
			decoder->codec.codec);
	} else {
		frame_size = &decoder->frame_size;
		output_buf_req = &decoder->actual_output_buf_req;
		input_buf_req = &decoder->actual_input_buf_req;
		y_cb_cr_size = decoder->y_cb_cr_size;
		min_dpb = decoder->min_dpb_num;
	}

	if (decoder->idr_only_decoding)
		min_dpb = 1;

	memset(output_buf_req, 0, sizeof(struct vcd_buffer_requirement));

	output_buf_req->min_count = min_dpb;

	num_mb = DDL_NO_OF_MB(frame_size->width, frame_size->height);
	if (decoder->idr_only_decoding) {
		output_buf_req->actual_count = output_buf_req->min_count;
	} else {
		if (num_mb >= DDL_WVGA_MBS) {
			output_buf_req->actual_count = min_dpb + 2;
			if (output_buf_req->actual_count < 10)
				output_buf_req->actual_count = 10;
		} else
			output_buf_req->actual_count = min_dpb + 5;
	}
	output_buf_req->max_count = DDL_MAX_BUFFER_COUNT;
	output_buf_req->sz = y_cb_cr_size;
	if (decoder->buf_format.buffer_format != VCD_BUFFER_FORMAT_NV12)
		output_buf_req->align = DDL_TILE_BUFFER_ALIGN_BYTES;
	else
		output_buf_req->align = DDL_LINEAR_BUFFER_ALIGN_BYTES;

	ddl_set_default_decoder_metadata_buffer_size(decoder,
		frame_size, output_buf_req);

	decoder->min_output_buf_req = *output_buf_req;

	memset(input_buf_req, 0, sizeof(struct vcd_buffer_requirement));

	input_buf_req->min_count = 1;
	input_buf_req->actual_count = input_buf_req->min_count + 3;
	input_buf_req->max_count = DDL_MAX_BUFFER_COUNT;
	input_buf_req->sz = (1280*720*3*3) >> 3;
	input_buf_req->align = DDL_LINEAR_BUFFER_ALIGN_BYTES;

	decoder->min_input_buf_req = *input_buf_req;

}

u32 ddl_get_yuv_buffer_size(struct vcd_property_frame_size *frame_size,
     struct vcd_property_buffer_format *buf_format, u32 inter_lace,
     enum vcd_codec codec)
{
	struct vcd_property_frame_size frame_sz = *frame_size;
	u32 total_memory_size;
	ddl_calculate_stride(&frame_sz, inter_lace, codec);

	if (buf_format->buffer_format != VCD_BUFFER_FORMAT_NV12) {
		u32 component_mem_size;
		u32 width_round_up;
		u32 height_round_up;
		u32 height_chroma = (frame_sz.scan_lines >> 1);

		width_round_up =
		    DDL_TILE_ALIGN(frame_sz.stride, DDL_TILE_ALIGN_WIDTH);
		height_round_up =
		    DDL_TILE_ALIGN(frame_sz.scan_lines, DDL_TILE_ALIGN_HEIGHT);

		component_mem_size = width_round_up * height_round_up;
		component_mem_size = DDL_TILE_ALIGN(component_mem_size,
						      DDL_TILE_MULTIPLY_FACTOR);

		total_memory_size = ((component_mem_size +
					 DDL_TILE_BUF_ALIGN_GUARD_BYTES) &
					DDL_TILE_BUF_ALIGN_MASK);

		height_round_up =
		    DDL_TILE_ALIGN(height_chroma, DDL_TILE_ALIGN_HEIGHT);
		component_mem_size = width_round_up * height_round_up;
		component_mem_size = DDL_TILE_ALIGN(component_mem_size,
						      DDL_TILE_MULTIPLY_FACTOR);
		total_memory_size += component_mem_size;
	} else {
		total_memory_size = frame_sz.scan_lines * frame_sz.stride;
		total_memory_size += (total_memory_size >> 1);
	}
	return total_memory_size;
}

void ddl_calculate_stride(struct vcd_property_frame_size *frame_size,
	u32 interlace, enum vcd_codec codec)
{
	frame_size->stride = ((frame_size->width + 15) >> 4) << 4;
	if (!interlace || codec == VCD_CODEC_MPEG4 ||
		codec == VCD_CODEC_DIVX_4 ||
		codec == VCD_CODEC_DIVX_5 ||
		codec == VCD_CODEC_DIVX_6 ||
		codec == VCD_CODEC_XVID) {
		frame_size->scan_lines =
			((frame_size->height + 15) >> 4) << 4;
	} else {
		frame_size->scan_lines =
			((frame_size->height + 31) >> 5) << 5;
	}

}

static u32 ddl_valid_buffer_requirement
	(struct vcd_buffer_requirement *original_buf_req,
	struct vcd_buffer_requirement *req_buf_req)
{
	u32 status = false;
	if (original_buf_req->max_count >= req_buf_req->actual_count &&
		original_buf_req->min_count <= req_buf_req->actual_count &&
		original_buf_req->align <= req_buf_req->align &&
		original_buf_req->sz <= req_buf_req->sz) {
		status = true;
	} else {
		VIDC_LOGERR_STRING("ddl_valid_buf_req:Failed");
	}
	return status;
}

static u32 ddl_decoder_min_num_dpb(struct ddl_decoder_data *decoder)
{
	u32 min_dpb = 0, yuv_size = 0;
	struct vcd_property_frame_size frame_sz = decoder->client_frame_size;
	switch (decoder->codec.codec) {
	default:
	case VCD_CODEC_MPEG4:
	case VCD_CODEC_MPEG2:
	case VCD_CODEC_DIVX_4:
	case VCD_CODEC_DIVX_5:
	case VCD_CODEC_DIVX_6:
	case VCD_CODEC_XVID:
		{
			min_dpb = 3;
			break;
		}
	case VCD_CODEC_H263:
		{
			min_dpb = 2;
			break;
		}
	case VCD_CODEC_VC1:
	case VCD_CODEC_VC1_RCV:
		{
			min_dpb = 4;
			break;
		}
	case VCD_CODEC_H264:
		{
			ddl_calculate_stride(&frame_sz,
				!decoder->progressive_only,
				decoder->codec.codec);
			yuv_size =
			    ((frame_sz.scan_lines *
			      frame_sz.stride * 3) >> 1);
			min_dpb = 6912000 / yuv_size;
			if (min_dpb > 16)
				min_dpb = 16;

			min_dpb += 2;
			break;
		}
	}
	return min_dpb;
}

static u32 ddl_set_dec_buffers
    (struct ddl_decoder_data *decoder,
     struct ddl_property_dec_pic_buffers *dpb) {
	u32 vcd_status = VCD_S_SUCCESS;
	u32 loopc;
	for (loopc = 0; !vcd_status &&
	     loopc < dpb->no_of_dec_pic_buf; ++loopc) {
		if ((!DDL_ADDR_IS_ALIGNED
		     (dpb->dec_pic_buffers[loopc].vcd_frm.physical,
		      decoder->client_output_buf_req.align)
		    )
		    || (dpb->dec_pic_buffers[loopc].vcd_frm.alloc_len <
			decoder->client_output_buf_req.sz)
		    ) {
			vcd_status = VCD_ERR_ILLEGAL_PARM;
		}
	}
	if (vcd_status) {
		VIDC_LOGERR_STRING
		    ("ddl_set_prop:Dpb_align_fail_or_alloc_size_small");
		return vcd_status;
	}
	if (decoder->dp_buf.no_of_dec_pic_buf) {
		DDL_FREE(decoder->dp_buf.dec_pic_buffers);
		decoder->dp_buf.no_of_dec_pic_buf = 0;
	}
	decoder->dp_buf.dec_pic_buffers =
	    DDL_MALLOC(dpb->no_of_dec_pic_buf *
		       sizeof(struct ddl_frame_data_tag));

	if (!decoder->dp_buf.dec_pic_buffers) {
		VIDC_LOGERR_STRING
		    ("ddl_dec_set_prop:Dpb_container_alloc_failed");
		return VCD_ERR_ALLOC_FAIL;
	}
	decoder->dp_buf.no_of_dec_pic_buf = dpb->no_of_dec_pic_buf;
	for (loopc = 0; loopc < dpb->no_of_dec_pic_buf; ++loopc) {
		decoder->dp_buf.dec_pic_buffers[loopc] =
		    dpb->dec_pic_buffers[loopc];
	}
	decoder->dpb_mask.client_mask = 0;
	decoder->dpb_mask.hw_mask = 0;
	decoder->dynamic_prop_change = 0;
	return VCD_S_SUCCESS;
}

void ddl_set_initial_default_values(struct ddl_client_context *ddl)
{
	if (ddl->decoding) {
		ddl->codec_data.decoder.codec.codec = VCD_CODEC_MPEG4;
		vcd_fw_transact(true, true,
			ddl->codec_data.decoder.codec.codec);
		ddl_set_default_dec_property(ddl);
	} else {
		struct ddl_encoder_data *encoder =
		    &(ddl->codec_data.encoder);
		encoder->codec.codec = VCD_CODEC_MPEG4;
		vcd_fw_transact(true, false,
			encoder->codec.codec);

		encoder->target_bit_rate.target_bitrate = 64000;
		encoder->frame_size.width = 176;
		encoder->frame_size.height = 144;
		encoder->frame_size.stride = 176;
		encoder->frame_size.scan_lines = 144;
		encoder->frame_rate.fps_numerator = 30;
		encoder->frame_rate.fps_denominator = 1;
		ddl_set_default_enc_property(ddl);
	}

	return;
}
