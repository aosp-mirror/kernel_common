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
 */


#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/math64.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <sound/q6asm-v2.h>
#include <sound/pcm_params.h>
#include <asm/dma.h>
#include <linux/dma-mapping.h>
#include <linux/msm_audio_ion.h>

#include <sound/timer.h>
#include <sound/tlv.h>

#include <sound/apr_audio-v2.h>
#include <sound/q6asm-v2.h>
#include <sound/compress_params.h>
#include <sound/compress_offload.h>
#include <sound/compress_driver.h>

#include "msm-pcm-routing-v2.h"
#include "audio_ocmem.h"
#include "msm-audio-effects-q6-v2.h"

#include <sound/q6adm-v2.h>
#include <linux/wakelock.h>
#include <mach/htc_acoustic_alsa.h>
#undef pr_info
#undef pr_err
#define pr_info(fmt, ...) pr_aud_info(fmt, ##__VA_ARGS__)
#define pr_err(fmt, ...) pr_aud_err(fmt, ##__VA_ARGS__)
#define DSP_PP_BUFFERING_IN_MSEC	25
#define PARTIAL_DRAIN_ACK_EARLY_BY_MSEC	150
#define MP3_OUTPUT_FRAME_SZ		1152
#define AAC_OUTPUT_FRAME_SZ		1024
#define AC3_OUTPUT_FRAME_SZ		1536
#define EAC3_OUTPUT_FRAME_SZ		1536
#define DSP_NUM_OUTPUT_FRAME_BUFFERED	2

#define DDP_DEC_MAX_NUM_PARAM		18

#define COMPR_PLAYBACK_MIN_FRAGMENT_SIZE (8 * 1024)

extern void store_asm_topology(uint32_t topology, uint32_t session);
#ifdef CONFIG_HD_AUDIO
#define COMPR_PLAYBACK_MAX_FRAGMENT_SIZE (256 * 1024)
#define COMPR_PLAYBACK_MAX_NUM_FRAGMENTS (16 * 8)
#else
#define COMPR_PLAYBACK_MAX_FRAGMENT_SIZE (128 * 1024)
#define COMPR_PLAYBACK_MAX_NUM_FRAGMENTS (16 * 4)
#endif
#define COMPR_PLAYBACK_MIN_NUM_FRAGMENTS (4)

#define COMPRESSED_LR_VOL_MAX_STEPS	0x2000
const DECLARE_TLV_DB_LINEAR(msm_compr_vol_gain, 0,
				COMPRESSED_LR_VOL_MAX_STEPS);

#define STREAM_ID_FROM_TOKEN(i) (i & 0xFF)

#define NEXT_STREAM_ID(stream_id) ((stream_id & 1) + 1)

#define STREAM_ARRAY_INDEX(stream_id) (stream_id - 1)

#define MAX_NUMBER_OF_STREAMS 2

#define ENABLE_OCMEM
struct wake_lock compr_lpa_q6_cb_wakelock;
#define APPI_SRS_TRUMEDIA_GEQ_MODULE_ID			0x10005100
#define APPI_SRS_TRUMEDIA_GEQ_SAMPLE_RATE		0x10005102

#define COMPRESSED_MASTER_VOL_MAX_STEPS	0x2000
struct msm_compr_gapless_state {
	bool set_next_stream_id;
	int32_t stream_opened[MAX_NUMBER_OF_STREAMS];
	uint32_t initial_samples_drop;
	uint32_t trailing_samples_drop;
	uint32_t gapless_transition;
	bool use_dsp_gapless_mode;
};

struct msm_compr_pdata {
	atomic_t audio_ocmem_req;
	struct snd_compr_stream *cstream[MSM_FRONTEND_DAI_MAX];
	uint32_t volume[MSM_FRONTEND_DAI_MAX][2]; 
	struct msm_compr_audio_effects *audio_effects[MSM_FRONTEND_DAI_MAX];
	bool use_dsp_gapless_mode;
	struct msm_compr_dec_params *dec_params[MSM_FRONTEND_DAI_MAX];
};

struct msm_compr_audio {
	struct snd_compr_stream *cstream;
	struct snd_compr_caps compr_cap;
	struct snd_compr_codec_caps codec_caps;
	struct snd_compr_params codec_param;
	struct audio_client *audio_client;

	uint32_t codec;
	void    *buffer; 
	uint32_t buffer_paddr; 
	uint32_t app_pointer;
	uint32_t buffer_size;
	uint32_t byte_offset;
	uint32_t copied_total; 
	uint32_t bytes_received; 
	uint32_t bytes_sent; 

	int32_t first_buffer;
	int32_t last_buffer;
	int32_t partial_drain_delay;

	uint16_t session_id;
	uint16_t bits_per_sample;
	int32_t write_last_buffer;
	atomic_t drain_done_pendding;
	uint64_t lasttimestamp;
	int32_t stream_end;
	uint32_t sample_rate;
	uint32_t num_channels;

	uint32_t cmd_ack;
	uint32_t cmd_interrupt;
	uint32_t drain_ready;
	uint32_t stream_available;
	uint32_t next_stream;

	uint64_t marker_timestamp;

	struct msm_compr_gapless_state gapless_state;

	atomic_t start;
	atomic_t eos;
	atomic_t drain;
	atomic_t xrun;
	atomic_t close;
	atomic_t wait_on_close;
	atomic_t error;

	wait_queue_head_t eos_wait;
	wait_queue_head_t drain_wait;
	wait_queue_head_t flush_wait;
	wait_queue_head_t close_wait;
	wait_queue_head_t wait_for_stream_avail;

	spinlock_t lock;
};

struct msm_compr_audio_effects {
	struct bass_boost_params bass_boost;
	struct virtualizer_params virtualizer;
	struct reverb_params reverb;
	struct eq_params equalizer;
};

struct msm_compr_dec_params {
	struct snd_dec_ddp ddp_params;
};

static int msm_compr_set_volume(struct snd_compr_stream *cstream,
				uint32_t volume_l, uint32_t volume_r)
{
	struct msm_compr_audio *prtd;
	int rc = 0;

	pr_debug("%s: volume_l %d volume_r %d\n",
		__func__, volume_l, volume_r);
	if (!cstream || !cstream->runtime) {
		pr_err("%s: session not active\n", __func__);
		return -EPERM;
	}
	prtd = cstream->runtime->private_data;
	if (prtd && prtd->audio_client) {
#if 0
		if (volume_l != volume_r) {
			pr_debug("%s: call q6asm_set_lrgain\n", __func__);
			rc = q6asm_set_lrgain(prtd->audio_client,

						volume_l, volume_r);
			if (rc < 0) {
				pr_err("%s: set lrgain command failed rc=%d\n",
				__func__, rc);
				return rc;
			}
			rc = q6asm_set_volume(prtd->audio_client,
						COMPRESSED_LR_VOL_MAX_STEPS);
		} else {
			pr_debug("%s: call q6asm_set_volume\n", __func__);
			rc = q6asm_set_lrgain(prtd->audio_client,
						COMPRESSED_LR_VOL_MAX_STEPS,
						COMPRESSED_LR_VOL_MAX_STEPS);
			if (rc < 0) {
				pr_err("%s: set lrgain command failed rc=%d\n",
				__func__, rc);
				return rc;
			}
			rc = q6asm_set_volume(prtd->audio_client, volume_l);
		}
#else
		pr_debug("%s: call q6asm_set_lrgain\n", __func__);
		rc = q6asm_set_lrgain(prtd->audio_client,
			volume_l, volume_r);
#endif
		if (rc < 0) {
			pr_err("%s: Send Volume command failed rc=%d\n",
				__func__, rc);
		}
	}

	return rc;
}

static int msm_compr_send_ddp_cfg(struct audio_client *ac,
				  struct snd_dec_ddp *ddp)
{
	int i, rc;
	pr_debug("%s\n", __func__);
	for (i = 0; i < ddp->params_length; i++) {
		rc = q6asm_ds1_set_endp_params(ac, ddp->params_id[i],
						ddp->params_value[i]);
		if (rc) {
			pr_err("sending params_id: %d failed\n",
				ddp->params_id[i]);
			return rc;
		}
	}
	return 0;
}

static int msm_compr_send_buffer(struct msm_compr_audio *prtd)
{
	int buffer_length;
	int bytes_available;
	struct audio_aio_write_param param;

	if (!prtd) {
		pr_err("%s: prtd is null \n", __func__);
		return -EINVAL;
        }

	if (!atomic_read(&prtd->start)) {
		pr_err("%s: stream is not in started state\n", __func__);
		return -EINVAL;
	}


	if (atomic_read(&prtd->xrun)) {
		WARN(1, "%s called while xrun is true", __func__);
		return -EPERM;
	}

	pr_info("%s: bytes_received = %d copied_total = %d\n",
		__func__, prtd->bytes_received, prtd->copied_total);
	if (prtd->first_buffer &&  prtd->gapless_state.use_dsp_gapless_mode)
		q6asm_stream_send_meta_data(prtd->audio_client,
				prtd->audio_client->stream_id,
				prtd->gapless_state.initial_samples_drop,
				prtd->gapless_state.trailing_samples_drop);

	buffer_length = prtd->codec_param.buffer.fragment_size;
	bytes_available = prtd->bytes_received - prtd->copied_total;
	if (bytes_available < prtd->codec_param.buffer.fragment_size)
		buffer_length = bytes_available;

	if (prtd->byte_offset + buffer_length > prtd->buffer_size) {
		buffer_length = (prtd->buffer_size - prtd->byte_offset);
		pr_debug("wrap around situation, send partial data %d now", buffer_length);
	}

	if (buffer_length)
		param.paddr	= prtd->buffer_paddr + prtd->byte_offset;
	else
		param.paddr	= prtd->buffer_paddr;
	WARN(param.paddr % 32 != 0, "param.paddr %lx not multiple of 32", param.paddr);

	param.len	= buffer_length;
	param.msw_ts	= 0;
	param.lsw_ts	= 0;
	param.flags	= NO_TIMESTAMP;
	param.uid	= buffer_length;
	param.metadata_len = 0;
	param.last_buffer = prtd->last_buffer;

	pr_info("%s: sending %d bytes to DSP byte_offset = %d\n",
		__func__, buffer_length, prtd->byte_offset);
	if (q6asm_async_write(prtd->audio_client, &param) < 0) {
		pr_err("%s:q6asm_async_write failed\n", __func__);
	} else {
		prtd->bytes_sent += buffer_length;
		if (prtd->first_buffer)
			prtd->first_buffer = 0;
	}

	return 0;
}

static void compr_event_handler(uint32_t opcode,
		uint32_t token, uint32_t *payload, void *priv)
{
	struct msm_compr_audio *prtd = priv;
	struct snd_compr_stream *cstream; 
	struct audio_client *ac; 
	uint32_t chan_mode = 0;
	uint32_t sample_rate = 0;
	int bytes_available, stream_id;
	uint32_t stream_index;

	if (!prtd) {
		pr_err("%s: cannot set from priv \n", __func__);
		return;
	}

	cstream = prtd->cstream;
	ac = prtd->audio_client;

	pr_debug("%s opcode =%08x\n", __func__, opcode);
	switch (opcode) {
	case ASM_DATA_EVENT_WRITE_DONE_V2:
		wake_lock_timeout(&compr_lpa_q6_cb_wakelock, 1.5 * HZ);
		pr_info("ASM_DATA_EVENT_WRITE_DONE_V2 hold wake_lock 1.5s\n");
		spin_lock(&prtd->lock);

		if (payload[3]) {
			pr_err("WRITE FAILED w/ err 0x%x !, paddr 0x%x"
			       " byte_offset = %d, copied_total = %d, token = %d\n",
			       payload[3],
			       payload[0],
			       prtd->byte_offset, prtd->copied_total, token);
			atomic_set(&prtd->start, 0);
		} else {
			pr_info("ASM_DATA_EVENT_WRITE_DONE_V2 offset %d, length %d\n",
				 prtd->byte_offset, token);
		}

		prtd->byte_offset += token;
		prtd->copied_total += token;
		if (prtd->byte_offset >= prtd->buffer_size)
			prtd->byte_offset -= prtd->buffer_size;

		snd_compr_fragment_elapsed(cstream);

		if (!atomic_read(&prtd->start)) {
			
			pr_info("write_done received while not started, treat as xrun");
			atomic_set(&prtd->xrun, 1);
			if (atomic_read(&prtd->drain)) {
				pr_info("write_done received while not started and wait drain buffer. Set drain_done_pending as 1\n");
				atomic_set(&prtd->drain_done_pendding, 1);
			}
			spin_unlock(&prtd->lock);
			break;
		}

		bytes_available = prtd->bytes_received - prtd->copied_total;
		if (((bytes_available == cstream->runtime->fragment_size)
			   && atomic_read(&prtd->drain)) || (prtd->write_last_buffer && bytes_available > 0)) {
			prtd->last_buffer = 1;
			msm_compr_send_buffer(prtd);
			prtd->last_buffer = 0;
		} else if (bytes_available < cstream->runtime->fragment_size) {
			pr_info("WRITE_DONE Insufficient data to send. break out\n");
			atomic_set(&prtd->xrun, 1);

			if (prtd->last_buffer)
				prtd->last_buffer = 0;
			if (atomic_read(&prtd->drain)) {
				pr_info("wake up on drain\n");
				prtd->drain_ready = 1;
				wake_up(&prtd->drain_wait);
				
			}
		} else
			msm_compr_send_buffer(prtd);

		spin_unlock(&prtd->lock);
		break;
	case ASM_DATA_EVENT_RENDERED_EOS:
		wake_lock_timeout(&compr_lpa_q6_cb_wakelock, 1.5 * HZ);
		pr_info("ASM_DATA_EVENT_RENDERED_EOS hold wake_lock 1.5s\n");
		spin_lock(&prtd->lock);
		pr_debug("%s: ASM_DATA_CMDRSP_EOS token 0x%x,stream id %d\n",
			  __func__, token, STREAM_ID_FROM_TOKEN(token));
		stream_id = STREAM_ID_FROM_TOKEN(token);
		if (atomic_read(&prtd->eos)) {
			if(!prtd->gapless_state.set_next_stream_id) {
			pr_debug("ASM_DATA_CMDRSP_EOS wake up\n");
			prtd->cmd_ack = 1;
			wake_up(&prtd->eos_wait);
			}
		}

		

		stream_index = STREAM_ARRAY_INDEX(stream_id);
		if (stream_index >= MAX_NUMBER_OF_STREAMS ||
		    stream_index < 0) {
			pr_err("%s: Invalid stream index %d", __func__,
				stream_index);
			spin_unlock(&prtd->lock);
			break;
		}

		if (prtd->gapless_state.set_next_stream_id &&
			prtd->gapless_state.stream_opened[stream_index]) {
			pr_debug("%s: CMD_CLOSE stream_id %d\n",
				  __func__, stream_id);
			q6asm_stream_cmd_nowait(ac, CMD_CLOSE, stream_id);
			atomic_set(&prtd->close, 1);
			prtd->gapless_state.stream_opened[stream_index] = 0;
			prtd->write_last_buffer = false;  
			prtd->gapless_state.set_next_stream_id = false;
		}
		if (prtd->gapless_state.gapless_transition)
			prtd->gapless_state.gapless_transition = 0;
		spin_unlock(&prtd->lock);
		break;
	case ASM_DATA_EVENT_SR_CM_CHANGE_NOTIFY:
	case ASM_DATA_EVENT_ENC_SR_CM_CHANGE_NOTIFY: {
		pr_info("ASM_DATA_EVENT_SR_CM_CHANGE_NOTIFY\n");
		chan_mode = payload[1] >> 16;
		sample_rate = payload[2] >> 16;
		if (prtd && (chan_mode != prtd->num_channels ||
				sample_rate != prtd->sample_rate)) {
			prtd->num_channels = chan_mode;
			prtd->sample_rate = sample_rate;
		}
	}
	case APR_BASIC_RSP_RESULT: {
		switch (payload[0]) {
		case ASM_SESSION_CMD_RUN_V2:
			
			pr_info("ASM_SESSION_CMD_RUN_V2\n");

			spin_lock(&prtd->lock);
			
			if (!prtd->bytes_sent) {
				bytes_available = prtd->bytes_received - prtd->copied_total;
				if (bytes_available > 0) {
					msm_compr_send_buffer(prtd);
				}
			}
			spin_unlock(&prtd->lock);
			break;
		case ASM_STREAM_CMD_FLUSH:
			pr_debug("%s: ASM_STREAM_CMD_FLUSH:", __func__);
			pr_debug("token 0x%x, stream id %d\n", token,
				  STREAM_ID_FROM_TOKEN(token));
			prtd->cmd_ack = 1;
			wake_up(&prtd->flush_wait);
			break;
		case ASM_DATA_CMD_REMOVE_INITIAL_SILENCE:
			pr_debug("%s: ASM_DATA_CMD_REMOVE_INITIAL_SILENCE:",
				   __func__);
			pr_debug("token 0x%x, stream id = %d\n", token,
				  STREAM_ID_FROM_TOKEN(token));
			break;
		case ASM_DATA_CMD_REMOVE_TRAILING_SILENCE:
			pr_debug("%s: ASM_DATA_CMD_REMOVE_TRAILING_SILENCE:",
				  __func__);
			pr_debug("token = 0x%x,	stream id = %d\n", token,
				  STREAM_ID_FROM_TOKEN(token));
			break;
		case ASM_STREAM_CMD_CLOSE:
			pr_debug("%s: ASM_DATA_CMD_CLOSE:", __func__);
			pr_debug("token 0x%x, stream id %d\n", token,
				  STREAM_ID_FROM_TOKEN(token));
			if (prtd->next_stream) {
				pr_debug("%s:CLOSE:wakeup wait for stream\n",
					  __func__);
				prtd->stream_available = 1;
				wake_up(&prtd->wait_for_stream_avail);
				prtd->next_stream = 0;
			}
			if (atomic_read(&prtd->close) &&
			    atomic_read(&prtd->wait_on_close)) {
				prtd->cmd_ack = 1;
				wake_up(&prtd->close_wait);
			}
			atomic_set(&prtd->close, 0);
			break;
		default:
			break;
		}
		break;
	}
	case ASM_SESSION_CMDRSP_GET_SESSIONTIME_V3:
		pr_debug("%s: ASM_SESSION_CMDRSP_GET_SESSIONTIME_V3\n",
			  __func__);
		break;
	case RESET_EVENTS:
		pr_err("%s: Received reset events CB, move to error state",
			__func__);
		spin_lock(&prtd->lock);
		snd_compr_fragment_elapsed(cstream);
		prtd->copied_total = prtd->bytes_received;
		atomic_set(&prtd->error, 1);
		spin_unlock(&prtd->lock);
		break;
	default:
		pr_debug("%s: Not Supported Event opcode[0x%x]\n",
			  __func__, opcode);
		break;
	}
}

static void populate_codec_list(struct msm_compr_audio *prtd)
{
	pr_debug("%s\n", __func__);
	prtd->compr_cap.direction = SND_COMPRESS_PLAYBACK;
	prtd->compr_cap.min_fragment_size =
			COMPR_PLAYBACK_MIN_FRAGMENT_SIZE;
	prtd->compr_cap.max_fragment_size =
			COMPR_PLAYBACK_MAX_FRAGMENT_SIZE;
	prtd->compr_cap.min_fragments =
			COMPR_PLAYBACK_MIN_NUM_FRAGMENTS;
	prtd->compr_cap.max_fragments =
			COMPR_PLAYBACK_MAX_NUM_FRAGMENTS;
	prtd->compr_cap.num_codecs = 5;
	prtd->compr_cap.codecs[0] = SND_AUDIOCODEC_MP3;
	prtd->compr_cap.codecs[1] = SND_AUDIOCODEC_AAC;
	prtd->compr_cap.codecs[2] = SND_AUDIOCODEC_AC3;
	prtd->compr_cap.codecs[3] = SND_AUDIOCODEC_EAC3;
	
	prtd->compr_cap.codecs[4] = SND_AUDIOCODEC_PCM;
}

static int msm_compr_send_media_format_block(struct snd_compr_stream *cstream,
					     int stream_id)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_compr_audio *prtd = runtime->private_data;
	struct asm_aac_cfg aac_cfg;
	int ret = 0;
	uint16_t bit_width = 16;

	switch (prtd->codec) {
	case FORMAT_LINEAR_PCM:
		pr_debug("SND_AUDIOCODEC_PCM\n");
		if (prtd->codec_param.codec.format == SNDRV_PCM_FORMAT_S24_LE)
			bit_width = 24;
		ret = q6asm_media_format_block_pcm_format_support(
							prtd->audio_client,
							prtd->sample_rate,
							prtd->num_channels,
							bit_width);
		if (ret < 0)
			pr_err("%s: CMD Format block failed\n", __func__);

		break;
	case FORMAT_MP3:
		
		break;
	case FORMAT_MPEG4_AAC:
		memset(&aac_cfg, 0x0, sizeof(struct asm_aac_cfg));
		aac_cfg.aot = AAC_ENC_MODE_EAAC_P;
		if (prtd->codec_param.codec.format ==
					SND_AUDIOSTREAMFORMAT_MP4ADTS)
			aac_cfg.format = 0x0;
		else
			aac_cfg.format = 0x03;
		aac_cfg.ch_cfg = prtd->num_channels;
		aac_cfg.sample_rate = prtd->sample_rate;
		ret = q6asm_stream_media_format_block_aac(prtd->audio_client,
							  &aac_cfg, stream_id);
		if (ret < 0)
			pr_err("%s: CMD Format block failed\n", __func__);
		
		ret = q6asm_stream_sample_rate_to_geq(prtd->audio_client, stream_id, APPI_SRS_TRUMEDIA_GEQ_MODULE_ID,
											APPI_SRS_TRUMEDIA_GEQ_SAMPLE_RATE, prtd->sample_rate);
		if (ret < 0)
			pr_err("%s: CMD samplerate to geq failed\n", __func__);
		
		break;
	case FORMAT_AC3:
		break;
	case FORMAT_EAC3:
		break;
	default:
		pr_debug("%s, unsupported format, skip", __func__);
		break;
	}
	return ret;
}

static int msm_compr_configure_dsp(struct snd_compr_stream *cstream)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_compr_audio *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *soc_prtd = cstream->private_data;
	uint16_t bits_per_sample = 16;
	int dir = IN, ret = 0;
	struct audio_client *ac = prtd->audio_client;
	uint32_t stream_index;
	struct asm_softvolume_params softvol = {
		.period = SOFT_VOLUME_PERIOD,
		.step = SOFT_VOLUME_STEP,
		.rampingcurve = SOFT_VOLUME_CURVE_LINEAR,
	};
	if (htc_acoustic_query_feature(HTC_AUD_24BIT)) {
		pr_info("%s: enable 24 bit Audio in POPP\n",
			    __func__);
		bits_per_sample = 24;
	}
	prtd->bits_per_sample = bits_per_sample;

	pr_debug("%s: stream_id %d\n", __func__, ac->stream_id);

#ifdef CONFIG_HD_AUDIO
	if ((prtd->codec == FORMAT_LINEAR_PCM) &&
	    (prtd->sample_rate > 48000)) {
		pr_info("ASM_STREAM_POSTPROC_TOPO_ID_DEFAULT");
		store_asm_topology(ASM_STREAM_POSTPROC_TOPO_ID_DEFAULT, prtd->audio_client->session);
	}
#endif

	ret = q6asm_stream_open_write_v2(ac,
				prtd->codec, bits_per_sample,
				ac->stream_id,
				prtd->gapless_state.use_dsp_gapless_mode);
	if (ret < 0) {
		pr_err("%s: Session out open failed\n", __func__);
		 return -ENOMEM;
	}

	stream_index = STREAM_ARRAY_INDEX(ac->stream_id);
	if (stream_index >= MAX_NUMBER_OF_STREAMS || stream_index < 0) {
		pr_err("%s: Invalid stream index:%d", __func__, stream_index);
		return -EINVAL;
	}

	prtd->gapless_state.stream_opened[stream_index] = 1;
	pr_debug("%s be_id %d\n", __func__, soc_prtd->dai_link->be_id);
	msm_pcm_routing_reg_phy_stream(soc_prtd->dai_link->be_id,
				ac->perf_mode,
				prtd->session_id,
				SNDRV_PCM_STREAM_PLAYBACK);


	
	ret = q6asm_set_volume(prtd->audio_client, COMPRESSED_MASTER_VOL_MAX_STEPS);

	if (ret < 0)
		pr_err("%s : Set Master Volume failed ret=%d\n", __func__, ret);
	ret = msm_compr_set_volume(cstream, 0, 0); 

	if (ret < 0)
		pr_err("%s : msm_compr_set_volume failed ret=%d\n", __func__, ret);

	ret = q6asm_set_softvolume(ac, &softvol);
	if (ret < 0)
		pr_err("%s: Send SoftVolume Param failed ret=%d\n",
			__func__, ret);

	ret = q6asm_set_io_mode(ac, (COMPRESSED_STREAM_IO | ASYNC_IO_MODE));
	if (ret < 0) {
		pr_err("%s: Set IO mode failed\n", __func__);
		return -EINVAL;
	}

	runtime->fragments = prtd->codec_param.buffer.fragments;
	runtime->fragment_size = prtd->codec_param.buffer.fragment_size;
	pr_debug("allocate %d buffers each of size %d\n",
			runtime->fragments,
			runtime->fragment_size);
	ret = q6asm_audio_client_buf_alloc_contiguous(dir, ac,
					runtime->fragment_size,
					runtime->fragments);
	if (ret < 0) {
		pr_err("Audio Start: Buffer Allocation failed rc = %d\n", ret);
		return -ENOMEM;
	}

	prtd->byte_offset  = 0;
	prtd->copied_total = 0;
	prtd->app_pointer  = 0;
	prtd->bytes_received = 0;
	prtd->bytes_sent = 0;
	prtd->buffer       = ac->port[dir].buf[0].data;
	prtd->buffer_paddr = ac->port[dir].buf[0].phys;
	prtd->buffer_size  = runtime->fragments * runtime->fragment_size;

	ret = msm_compr_send_media_format_block(cstream, ac->stream_id);
	if (ret < 0)
		pr_err("%s, failed to send media format block\n", __func__);

	return ret;
}

static int msm_compr_open(struct snd_compr_stream *cstream)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct msm_compr_audio *prtd;
	struct msm_compr_pdata *pdata =
			snd_soc_platform_get_drvdata(rtd->platform);

	pr_debug("%s\n", __func__);
	prtd = kzalloc(sizeof(struct msm_compr_audio), GFP_KERNEL);
	if (prtd == NULL) {
		pr_err("Failed to allocate memory for msm_compr_audio\n");
		return -ENOMEM;
	}

	prtd->cstream = cstream;
	pdata->cstream[rtd->dai_link->be_id] = cstream;
	pdata->audio_effects[rtd->dai_link->be_id] =
		 kzalloc(sizeof(struct msm_compr_audio_effects), GFP_KERNEL);
	if (!pdata->audio_effects[rtd->dai_link->be_id]) {
		pr_err("%s: Could not allocate memory for effects\n", __func__);
		pdata->cstream[rtd->dai_link->be_id] = NULL;
		kfree(prtd);
		return -ENOMEM;
	}
	pdata->dec_params[rtd->dai_link->be_id] =
		 kzalloc(sizeof(struct msm_compr_dec_params), GFP_KERNEL);
	if (!pdata->dec_params[rtd->dai_link->be_id]) {
		pr_err("%s: Could not allocate memory for dec params\n",
			__func__);
		kfree(pdata->audio_effects[rtd->dai_link->be_id]);
		pdata->cstream[rtd->dai_link->be_id] = NULL;
		kfree(prtd);
		return -ENOMEM;
	}
	prtd->audio_client = q6asm_audio_client_alloc(
				(app_cb)compr_event_handler, prtd);
	if (!prtd->audio_client) {
		pr_err("%s: Could not allocate memory for client\n", __func__);
		kfree(pdata->audio_effects[rtd->dai_link->be_id]);
		kfree(pdata->dec_params[rtd->dai_link->be_id]);
		pdata->cstream[rtd->dai_link->be_id] = NULL;
		kfree(prtd);
		return -ENOMEM;
	}

	pr_debug("%s: session ID %d\n", __func__, prtd->audio_client->session);
	prtd->audio_client->perf_mode = false;
	prtd->session_id = prtd->audio_client->session;
	prtd->codec = FORMAT_MP3;
	prtd->bytes_received = 0;
	prtd->bytes_sent = 0;
	prtd->copied_total = 0;
	prtd->byte_offset = 0;
	prtd->sample_rate = 44100;
	prtd->num_channels = 2;
#ifdef CONFIG_HD_AUDIO
	prtd->bits_per_sample = 16;
#endif
	prtd->drain_ready = 0;
	prtd->last_buffer = 0;
	prtd->first_buffer = 1;
	prtd->partial_drain_delay = 0;
	prtd->next_stream = 0;
	prtd->write_last_buffer = false;
	prtd->stream_end = false;
	prtd->lasttimestamp = 0;
	memset(&prtd->gapless_state, 0, sizeof(struct msm_compr_gapless_state));
	store_asm_topology(0, prtd->audio_client->session);
	if (q6asm_set_io_mode(prtd->audio_client, (COMPRESSED_IO | ASYNC_IO_MODE))) {
		pr_err("%s: Set IO mode failed\n", __func__);
		return -EINVAL;
	}
	prtd->gapless_state.use_dsp_gapless_mode = pdata->use_dsp_gapless_mode;

	pr_info("%s: gapless mode %d", __func__, pdata->use_dsp_gapless_mode);

	spin_lock_init(&prtd->lock);

	atomic_set(&prtd->eos, 0);
	atomic_set(&prtd->start, 0);
	atomic_set(&prtd->drain, 0);
	atomic_set(&prtd->xrun, 0);
	atomic_set(&prtd->close, 0);
	atomic_set(&prtd->wait_on_close, 0);
	atomic_set(&prtd->error, 0);
	atomic_set(&prtd->drain_done_pendding, 0); 

	init_waitqueue_head(&prtd->eos_wait);
	init_waitqueue_head(&prtd->drain_wait);
	init_waitqueue_head(&prtd->flush_wait);
	init_waitqueue_head(&prtd->close_wait);
	init_waitqueue_head(&prtd->wait_for_stream_avail);

	runtime->private_data = prtd;
	populate_codec_list(prtd);
#ifdef ENABLE_OCMEM
	if (cstream->direction == SND_COMPRESS_PLAYBACK) {
		if (!atomic_cmpxchg(&pdata->audio_ocmem_req, 0, 1))
			audio_ocmem_process_req(AUDIO, true);
		else
			atomic_inc(&pdata->audio_ocmem_req);
		pr_debug("%s: ocmem_req: %d\n", __func__,
				atomic_read(&pdata->audio_ocmem_req));
	} else {
		pr_err("%s: Unsupported stream type", __func__);
	}
#endif
	return 0;
}

static int msm_compr_free(struct snd_compr_stream *cstream)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_compr_audio *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *soc_prtd = cstream->private_data;
	struct msm_compr_pdata *pdata =
			snd_soc_platform_get_drvdata(soc_prtd->platform);
	struct audio_client *ac = prtd->audio_client;
	int dir = IN, ret = 0, stream_id;
	unsigned long flags;
	uint32_t stream_index;

	pr_debug("%s\n", __func__);

	if (atomic_read(&prtd->eos)) {
		ret = wait_event_timeout(prtd->eos_wait,
					 prtd->cmd_ack, 5 * HZ);
		if (!ret)
			pr_err("%s: CMD_EOS failed\n", __func__);
	}
	if (atomic_read(&prtd->close)) {
		prtd->cmd_ack = 0;
		atomic_set(&prtd->wait_on_close, 1);
		ret = wait_event_timeout(prtd->close_wait,
					prtd->cmd_ack, 5 * HZ);
		if (!ret)
			pr_err("%s: CMD_CLOSE failed\n", __func__);
	}

	spin_lock_irqsave(&prtd->lock, flags);
	stream_id = ac->stream_id;
	stream_index = STREAM_ARRAY_INDEX(NEXT_STREAM_ID(stream_id));

	if ((stream_index < MAX_NUMBER_OF_STREAMS && stream_index >= 0) &&
	    (prtd->gapless_state.stream_opened[stream_index])) {
		prtd->gapless_state.stream_opened[stream_index] = 0;
		spin_unlock_irqrestore(&prtd->lock, flags);
		pr_debug(" close stream %d", NEXT_STREAM_ID(stream_id));
		q6asm_stream_cmd(ac, CMD_CLOSE, NEXT_STREAM_ID(stream_id));
		spin_lock_irqsave(&prtd->lock, flags);
	}

	stream_index = STREAM_ARRAY_INDEX(stream_id);
	if ((stream_index < MAX_NUMBER_OF_STREAMS && stream_index >= 0) &&
	    (prtd->gapless_state.stream_opened[stream_index])) {
		prtd->gapless_state.stream_opened[stream_index] = 0;
		spin_unlock_irqrestore(&prtd->lock, flags);
		pr_debug("close stream %d", stream_id);
		q6asm_stream_cmd(ac, CMD_CLOSE, stream_id);
		spin_lock_irqsave(&prtd->lock, flags);
	}
	spin_unlock_irqrestore(&prtd->lock, flags);

	pdata->cstream[soc_prtd->dai_link->be_id] = NULL;
	if (cstream->direction == SND_COMPRESS_PLAYBACK) {
#ifdef ENABLE_OCMEM
		if (atomic_read(&pdata->audio_ocmem_req) > 1)
			atomic_dec(&pdata->audio_ocmem_req);
		else if (atomic_cmpxchg(&pdata->audio_ocmem_req, 1, 0))
			audio_ocmem_process_req(AUDIO, false);
#endif
		msm_pcm_routing_dereg_phy_stream(soc_prtd->dai_link->be_id,
						SNDRV_PCM_STREAM_PLAYBACK);
	}

	pr_debug("%s: ocmem_req: %d\n", __func__,
		atomic_read(&pdata->audio_ocmem_req));
	q6asm_audio_client_buf_free_contiguous(dir, ac);

	q6asm_audio_client_free(ac);

	kfree(pdata->audio_effects[soc_prtd->dai_link->be_id]);
	kfree(pdata->dec_params[soc_prtd->dai_link->be_id]);
	kfree(prtd);

	return 0;
}

static int msm_compr_set_params(struct snd_compr_stream *cstream,
				struct snd_compr_params *params)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_compr_audio *prtd = runtime->private_data;
	int ret = 0, frame_sz = 0, delay_time_ms = 0;

	pr_debug("%s\n", __func__);

	memcpy(&prtd->codec_param, params, sizeof(struct snd_compr_params));

	
	prtd->num_channels = prtd->codec_param.codec.ch_in;

	switch (prtd->codec_param.codec.sample_rate) {
	case SNDRV_PCM_RATE_8000:
		prtd->sample_rate = 8000;
		break;
	case SNDRV_PCM_RATE_11025:
		prtd->sample_rate = 11025;
		break;
	
	case SNDRV_PCM_RATE_16000:
		prtd->sample_rate = 16000;
		break;
	case SNDRV_PCM_RATE_22050:
		prtd->sample_rate = 22050;
		break;
	case SNDRV_PCM_RATE_32000:
		prtd->sample_rate = 32000;
		break;
	case SNDRV_PCM_RATE_44100:
		prtd->sample_rate = 44100;
		break;
	case SNDRV_PCM_RATE_48000:
		prtd->sample_rate = 48000;
		break;
#ifdef CONFIG_HD_AUDIO
	case SNDRV_PCM_RATE_64000:
		prtd->sample_rate = 64000;
		break;
	case SNDRV_PCM_RATE_88200:
		prtd->sample_rate = 88200;
		break;
	case SNDRV_PCM_RATE_96000:
		prtd->sample_rate = 96000;
		break;
	case SNDRV_PCM_RATE_176400:
		prtd->sample_rate = 176400;
		break;
	case SNDRV_PCM_RATE_192000:
		prtd->sample_rate = 192000;
		break;
#endif
	}

	pr_debug("%s: sample_rate %d\n", __func__, prtd->sample_rate);

	switch (params->codec.id) {
	case SND_AUDIOCODEC_PCM: {
		pr_debug("SND_AUDIOCODEC_PCM\n");
		prtd->codec = FORMAT_LINEAR_PCM;
		pr_info("SND_AUDIOCODEC_PCM format = %d, bits_per_sample=%d, prtd->bits_per_sample=%d \n",
				prtd->bits_per_sample,
				prtd->codec_param.codec.format,
				prtd->codec_param.codec.format);
		prtd->codec = FORMAT_LINEAR_PCM;
		prtd->bits_per_sample = prtd->codec_param.codec.format;
		break;
	}

	case SND_AUDIOCODEC_MP3: {
		pr_debug("SND_AUDIOCODEC_MP3\n");
		prtd->codec = FORMAT_MP3;
		frame_sz = MP3_OUTPUT_FRAME_SZ;
		break;
	}

	case SND_AUDIOCODEC_AAC: {
		pr_debug("SND_AUDIOCODEC_AAC\n");
		prtd->codec = FORMAT_MPEG4_AAC;
		frame_sz = AAC_OUTPUT_FRAME_SZ;
		break;
	}

	case SND_AUDIOCODEC_AC3: {
		prtd->codec = FORMAT_AC3;
		frame_sz = AC3_OUTPUT_FRAME_SZ;
		break;
	}

	case SND_AUDIOCODEC_EAC3: {
		prtd->codec = FORMAT_EAC3;
		frame_sz = EAC3_OUTPUT_FRAME_SZ;
		break;
	}

	default:
		pr_err("codec not supported, id =%d\n", params->codec.id);
		return -EINVAL;
	}

#ifdef CONFIG_HD_AUDIO
	if ((prtd->codec != FORMAT_LINEAR_PCM) && (prtd->sample_rate > 48000)) {
		pr_err("Out of bounds sample rate for codec %d", prtd->codec);
		return -EINVAL;
	}
#endif

	delay_time_ms = ((DSP_NUM_OUTPUT_FRAME_BUFFERED * frame_sz * 1000) /
			prtd->sample_rate) + DSP_PP_BUFFERING_IN_MSEC;
	delay_time_ms = delay_time_ms > PARTIAL_DRAIN_ACK_EARLY_BY_MSEC ?
			delay_time_ms - PARTIAL_DRAIN_ACK_EARLY_BY_MSEC : 0;
	prtd->partial_drain_delay = delay_time_ms;

	ret = msm_compr_configure_dsp(cstream);

	return ret;
}

static int msm_compr_drain_buffer(struct msm_compr_audio *prtd,
				  unsigned long *flags)
{
	int rc = 0;

	atomic_set(&prtd->drain, 1);
	prtd->drain_ready = 0;
	spin_unlock_irqrestore(&prtd->lock, *flags);
	pr_debug("%s: wait for buffer to be drained\n",  __func__);
	wait_event(prtd->drain_wait,
				prtd->drain_ready ||
				prtd->cmd_interrupt ||
				atomic_read(&prtd->xrun));
	atomic_set(&prtd->drain, 0);
	pr_debug("%s: out of buffer drain wait\n", __func__);
	spin_lock_irqsave(&prtd->lock, *flags);
	if (prtd->cmd_interrupt) {
		pr_debug("%s: buffer drain interrupted by flush)\n", __func__);
		rc = -EINTR;
		prtd->cmd_interrupt = 0;
	}
	return rc;
}

static int msm_compr_wait_for_stream_avail(struct msm_compr_audio *prtd,
				    unsigned long *flags)
{
	int rc = 0;
	pr_debug("next session is already in opened state\n");
	prtd->next_stream = 1;
	prtd->cmd_interrupt = 0;
	spin_unlock_irqrestore(&prtd->lock, *flags);
	rc = wait_event_timeout(prtd->wait_for_stream_avail,
		prtd->stream_available || prtd->cmd_interrupt, 1 * HZ);
	pr_err("%s:prtd->stream_available %d, prtd->cmd_interrupt %d rc %d\n",
		   __func__, prtd->stream_available, prtd->cmd_interrupt, rc);

	spin_lock_irqsave(&prtd->lock, *flags);
	if (rc == 0) {
		pr_err("%s: wait_for_stream_avail timed out\n",
						__func__);
		rc =  -ETIMEDOUT;
	} else if (prtd->cmd_interrupt == 1) {
		pr_debug("%s: wait_for_stream_avail interrupted\n", __func__);
		prtd->cmd_interrupt = 0;
		prtd->stream_available = 0;
		rc = -EINTR;
	} else {
		prtd->stream_available = 0;
		rc = 0;
	}
	pr_debug("%s : rc = %d",  __func__, rc);
	return rc;
}

static int msm_compr_trigger(struct snd_compr_stream *cstream, int cmd)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_compr_audio *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct msm_compr_pdata *pdata =
			snd_soc_platform_get_drvdata(rtd->platform);
	uint32_t *volume = pdata->volume[rtd->dai_link->be_id];
	struct audio_client *ac; 
	int rc = 0;
	int bytes_to_write;
	unsigned long flags;
	int stream_id;
	uint32_t stream_index;

	if (!prtd) {
		pr_err("%s: cannot set from private_data\n", __func__);
		return -EINVAL;
	}

        ac = prtd->audio_client;

	if (cstream->direction != SND_COMPRESS_PLAYBACK) {
		pr_err("%s: Unsupported stream type\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&prtd->lock, flags);
	if (atomic_read(&prtd->error)) {
		pr_err("%s Got RESET EVENTS notification, return immediately", __func__);
		spin_unlock_irqrestore(&prtd->lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&prtd->lock, flags);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		pr_info("%s: SNDRV_PCM_TRIGGER_START\n", __func__);
		atomic_set(&prtd->start, 1);
		q6asm_run_nowait(prtd->audio_client, 0, 0, 0);

		msm_compr_set_volume(cstream, volume[0], volume[1]);
		if (rc)
			pr_err("%s : Set Volume failed : %d\n",
				__func__, rc);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		spin_lock_irqsave(&prtd->lock, flags);

		pr_info("%s: SNDRV_PCM_TRIGGER_STOP transition %d\n", __func__,
					prtd->gapless_state.gapless_transition);

		stream_id = ac->stream_id;
		atomic_set(&prtd->start, 0);
		if (prtd->next_stream) {
			pr_debug("%s: interrupt next track wait queues\n",
								__func__);
			prtd->cmd_interrupt = 1;
			wake_up(&prtd->wait_for_stream_avail);
			prtd->next_stream = 0;
		}
		if (atomic_read(&prtd->eos)) {
			pr_info("%s: interrupt eos wait queues", __func__);
			prtd->cmd_interrupt = 1;
			wake_up(&prtd->eos_wait);
			
		}
		if (atomic_read(&prtd->drain)) {
			pr_debug("%s: interrupt drain wait queues", __func__);
			prtd->cmd_interrupt = 1;
			prtd->drain_ready = 1;
			wake_up(&prtd->drain_wait);
			
		}
		prtd->last_buffer = 0;
		prtd->cmd_ack = 0;
		pr_info("reset lasttimestamp at SNDRV_PCM_TRIGGER_STOP\n");
		prtd->stream_end = false;
		prtd->lasttimestamp = 0;
		if (!prtd->gapless_state.gapless_transition) {
			pr_debug("issue CMD_FLUSH stream_id %d\n", stream_id);
			spin_unlock_irqrestore(&prtd->lock, flags);
			rc = q6asm_stream_cmd(
				prtd->audio_client, CMD_FLUSH, stream_id);
			if (rc < 0) {
				pr_err("%s: flush cmd failed rc=%d\n",
							__func__, rc);
				return rc;
			}
			rc = wait_event_timeout(prtd->flush_wait,
					prtd->cmd_ack, 1 * HZ);
			if (!rc) {
				rc = -ETIMEDOUT;
				pr_err("Flush cmd timeout\n");
			} else {
				rc = 0; 
			}
			spin_lock_irqsave(&prtd->lock, flags);
		} else {
			prtd->first_buffer = 0;
		}
		
		prtd->byte_offset  = 0;
		prtd->copied_total = 0;
		prtd->app_pointer  = 0;
		prtd->bytes_received = 0;
		prtd->bytes_sent = 0;
		prtd->marker_timestamp = 0;

		atomic_set(&prtd->xrun, 0);
		atomic_set(&prtd->drain_done_pendding, 0); 
		spin_unlock_irqrestore(&prtd->lock, flags);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		pr_info("SNDRV_PCM_TRIGGER_PAUSE_PUSH transition %d\n",
				prtd->gapless_state.gapless_transition);
		if (!prtd->gapless_state.gapless_transition) {
			pr_debug("issue CMD_PAUSE stream_id %d\n",
				  ac->stream_id);
			q6asm_stream_cmd_nowait(ac, CMD_PAUSE, ac->stream_id);
			atomic_set(&prtd->start, 0);
		}
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		pr_info("SNDRV_PCM_TRIGGER_PAUSE_RELEASE transition %d\n",
				   prtd->gapless_state.gapless_transition);
		if (!prtd->gapless_state.gapless_transition) {
			atomic_set(&prtd->start, 1);
			q6asm_run_nowait(prtd->audio_client, 0, 0, 0);
			if (atomic_read(&prtd->drain_done_pendding)) {
				spin_lock_irqsave(&prtd->lock, flags);
				pr_info("SNDRV_PCM_TRIGGER_PAUSE_RELEASE: has the drain_done_pendding signal\n");
				if (prtd->last_buffer)
					prtd->last_buffer = 0;
				if (atomic_read(&prtd->drain)) {
					pr_info("SNDRV_PCM_TRIGGER_PAUSE_RELEASE: wake up on drain\n");
					prtd->drain_ready = 1;
					wake_up(&prtd->drain_wait);
					
				}
				atomic_set(&prtd->drain_done_pendding, 0);
				spin_unlock_irqrestore(&prtd->lock, flags);
			}
		}
		break;
	case SND_COMPR_TRIGGER_PARTIAL_DRAIN:
		pr_info("%s: SND_COMPR_TRIGGER_PARTIAL_DRAIN\n", __func__);
		if (!prtd->gapless_state.use_dsp_gapless_mode) {
			pr_info("%s: set partial drain as drain\n", __func__);
			cmd = SND_COMPR_TRIGGER_DRAIN;
		}
	case SND_COMPR_TRIGGER_DRAIN:
		pr_info("%s: SNDRV_COMPRESS_DRAIN\n", __func__);
		
		spin_lock_irqsave(&prtd->lock, flags);

		if (!atomic_read(&prtd->start)) {
			pr_err("%s: stream is not in started state\n",
				__func__);
			rc = -EPERM;
			spin_unlock_irqrestore(&prtd->lock, flags);
			break;
		}
		if (prtd->bytes_received > prtd->copied_total) {
			pr_debug("%s: wait till all the data is sent to dsp\n",
				__func__);
			rc = msm_compr_drain_buffer(prtd, &flags);
			if (rc || !atomic_read(&prtd->start)) {
				rc = -EINTR;
				spin_unlock_irqrestore(&prtd->lock, flags);
				break;
			}
			bytes_to_write = prtd->bytes_received - prtd->copied_total;
			WARN(bytes_to_write > runtime->fragment_size,
			     "last write %d cannot be > than fragment_size",
			     bytes_to_write);

			if (bytes_to_write > 0) {
				pr_debug("%s: send %d partial bytes at the end",
				       __func__, bytes_to_write);
				atomic_set(&prtd->xrun, 0);
				prtd->last_buffer = 1;
				msm_compr_send_buffer(prtd);
			}
		}

		if ((cmd == SND_COMPR_TRIGGER_PARTIAL_DRAIN) &&
		    (prtd->gapless_state.set_next_stream_id)) {
			

			if (prtd->last_buffer) {
				pr_info("%s: last buffer drain\n", __func__);
				rc = msm_compr_drain_buffer(prtd, &flags);
				if (rc) {
					spin_unlock_irqrestore(&prtd->lock, flags);
					break;
				}
			}

			
			prtd->cmd_ack = 0;
			pr_debug("issue CMD_EOS stream_id %d\n", ac->stream_id);
			q6asm_stream_cmd_nowait(ac, CMD_EOS, ac->stream_id);
			pr_info("PARTIAL DRAIN, do not wait for EOS ack\n");

			
			atomic_set(&prtd->xrun, 0);
			msm_compr_send_buffer(prtd);

			
			pr_info("%s: zero length buffer drain\n", __func__);
			rc = msm_compr_drain_buffer(prtd, &flags);
			if (rc) {
				spin_unlock_irqrestore(&prtd->lock, flags);
				break;
			}

			
			atomic_set(&prtd->drain, 1);
			prtd->drain_ready = 0;
			pr_info("%s, additional sleep: %d\n", __func__,
				 prtd->partial_drain_delay);
			spin_unlock_irqrestore(&prtd->lock, flags);
			rc = wait_event_timeout(prtd->drain_wait,
				prtd->drain_ready || prtd->cmd_interrupt,
				msecs_to_jiffies(prtd->partial_drain_delay));
			pr_info("%s: out of additional wait for low sample rate\n",
				 __func__);

			
			q6asm_get_session_time(prtd->audio_client, &prtd->lasttimestamp); 
			pr_info("gapless: record stream end timestamp:%llu\n", prtd->lasttimestamp); 
			spin_lock_irqsave(&prtd->lock, flags);
			if (prtd->cmd_interrupt) {
				pr_debug("%s: additional wait interrupted by flush)\n",
					 __func__);
				rc = -EINTR;
				prtd->cmd_interrupt = 0;
				spin_unlock_irqrestore(&prtd->lock, flags);
				break;
			}

			
			prtd->stream_end = true; 
			pr_debug("%s: Moving to next stream in gapless\n",
								__func__);
			ac->stream_id = NEXT_STREAM_ID(ac->stream_id);
			prtd->byte_offset = 0;
			prtd->app_pointer  = 0;
			prtd->first_buffer = 1;
			prtd->last_buffer = 0;
			prtd->gapless_state.gapless_transition = 1;
			prtd->marker_timestamp = 0;

			atomic_set(&prtd->drain, 0);
			pr_info("%s: issue CMD_RUN", __func__);
			q6asm_run_nowait(prtd->audio_client, 0, 0, 0);
			spin_unlock_irqrestore(&prtd->lock, flags);
			break;
		}
		prtd->write_last_buffer = false;
		prtd->gapless_state.set_next_stream_id = false;
		pr_debug("%s:CMD_EOS stream_id %d\n", __func__, ac->stream_id);

		prtd->cmd_ack = 0;
		atomic_set(&prtd->eos, 1);
		q6asm_stream_cmd_nowait(ac, CMD_EOS, ac->stream_id);

		spin_unlock_irqrestore(&prtd->lock, flags);


		
		wait_event(prtd->eos_wait,
					  (prtd->cmd_ack || prtd->cmd_interrupt));
		atomic_set(&prtd->eos, 0);
		if (rc < 0)
			pr_err("%s: EOS wait failed\n", __func__);

		pr_debug("%s: SNDRV_COMPRESS_DRAIN  out of wait for EOS\n",
			  __func__);

		if (prtd->cmd_interrupt)
			rc = -EINTR;

		
		if (rc == 0) {

			
			q6asm_get_session_time(prtd->audio_client, &prtd->lasttimestamp); 
			pr_info("record stream end timestamp:%llu\n", prtd->lasttimestamp); 
			spin_lock_irqsave(&prtd->lock, flags);
			prtd->stream_end = true; 
			pr_debug("%s:issue CMD_PAUSE stream_id %d",
					  __func__, ac->stream_id);
			q6asm_stream_cmd_nowait(ac, CMD_PAUSE, ac->stream_id);
			prtd->cmd_ack = 0;
			spin_unlock_irqrestore(&prtd->lock, flags);

			q6asm_get_session_time(prtd->audio_client,
					       &prtd->marker_timestamp);
			spin_lock_irqsave(&prtd->lock, flags);
			prtd->byte_offset = 0;

			prtd->app_pointer  = 0;
			prtd->first_buffer = 1;
			prtd->last_buffer = 0;
			atomic_set(&prtd->drain, 0);
			atomic_set(&prtd->xrun, 1);
			spin_unlock_irqrestore(&prtd->lock, flags);

			pr_debug("%s:issue CMD_FLUSH ac->stream_id %d",
					      __func__, ac->stream_id);
			q6asm_stream_cmd(ac, CMD_FLUSH, ac->stream_id);
			wait_event_timeout(prtd->flush_wait,
					   prtd->cmd_ack, 1 * HZ / 4);

			q6asm_run_nowait(prtd->audio_client, 0, 0, 0);
		}
		prtd->cmd_interrupt = 0;
		break;
	case SND_COMPR_TRIGGER_NEXT_TRACK:
		prtd->write_last_buffer = true;
		if (!prtd->gapless_state.use_dsp_gapless_mode) {
			pr_info("%s: ignore trigger next track\n", __func__);
			rc = 0;
			break;
		}
		pr_info("%s: SND_COMPR_TRIGGER_NEXT_TRACK\n", __func__);
		spin_lock_irqsave(&prtd->lock, flags);
		rc = 0;
		
		stream_id = NEXT_STREAM_ID(ac->stream_id);
		stream_index = STREAM_ARRAY_INDEX(stream_id);
		if (stream_index >= MAX_NUMBER_OF_STREAMS ||
		    stream_index < 0) {
			pr_err("%s: Invalid stream index: %d", __func__,
				stream_index);
			spin_unlock_irqrestore(&prtd->lock, flags);
			rc = -EINVAL;
			break;
		}

		if (prtd->gapless_state.stream_opened[stream_index]) {
			if (prtd->gapless_state.gapless_transition) {
				rc = msm_compr_wait_for_stream_avail(prtd,
								    &flags);
			} else {
				pr_debug("next session is in opened state\n");
				spin_unlock_irqrestore(&prtd->lock, flags);
				break;
			}
		}
		spin_unlock_irqrestore(&prtd->lock, flags);
		if (rc < 0) {
			if (rc == -EINTR) {
				pr_debug("%s: EINTR reset rc to 0\n", __func__);
				rc = 0;
			}
			break;
		}
		pr_debug("%s: open_write stream_id %d", __func__, stream_id);
		rc = q6asm_stream_open_write_v2(prtd->audio_client,
				prtd->codec, prtd->bits_per_sample,
				stream_id,
				prtd->gapless_state.use_dsp_gapless_mode);
		if (rc < 0) {
			pr_err("%s: Session out open failed for gapless\n",
				 __func__);
			break;
		}
		rc = msm_compr_send_media_format_block(cstream, stream_id);
		if (rc < 0) {
			 pr_err("%s, failed to send media format block\n",
				__func__);
			break;
		}
		spin_lock_irqsave(&prtd->lock, flags);
		prtd->gapless_state.stream_opened[stream_index] = 1;
		prtd->gapless_state.set_next_stream_id = true;
		spin_unlock_irqrestore(&prtd->lock, flags);
		break;
	}

	return rc;
}

static int msm_compr_pointer(struct snd_compr_stream *cstream,
				struct snd_compr_tstamp *arg)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_compr_audio *prtd = runtime->private_data;
	struct snd_compr_tstamp tstamp;
	uint64_t timestamp = 0;
	int rc = 0, first_buffer;
	uint64_t lasttimestamp;
	int32_t stream_end;
	unsigned long flags;

	pr_debug("%s\n", __func__);
	memset(&tstamp, 0x0, sizeof(struct snd_compr_tstamp));

	spin_lock_irqsave(&prtd->lock, flags);
	tstamp.sampling_rate = prtd->sample_rate;
	tstamp.byte_offset = prtd->byte_offset;
	tstamp.copied_total = prtd->copied_total;
	first_buffer = prtd->first_buffer;
	stream_end = prtd->stream_end;
	lasttimestamp = prtd->lasttimestamp;

	if (atomic_read(&prtd->error)) {
		pr_err("%s Got RESET EVENTS notification, return error", __func__);
		tstamp.pcm_io_frames = 0;
		memcpy(arg, &tstamp, sizeof(struct snd_compr_tstamp));
		spin_unlock_irqrestore(&prtd->lock, flags);
		return -ENETRESET;
	}

	spin_unlock_irqrestore(&prtd->lock, flags);

	if (stream_end) {
		timestamp = lasttimestamp;
	} else if (!first_buffer) {
		rc = q6asm_get_session_time(prtd->audio_client, &timestamp);
		if (rc < 0) {
			pr_err("%s: Get Session Time return value =%lld\n",
				__func__, timestamp);
			if (atomic_read(&prtd->error))
				return -ENETRESET;
			else
				return -EAGAIN;
		}
	} else {
		timestamp = prtd->marker_timestamp;
	}

	
	pr_debug("%s: timestamp = %lld usec\n", __func__, timestamp);
	timestamp *= prtd->sample_rate;
	tstamp.pcm_io_frames = (snd_pcm_uframes_t)div64_u64(timestamp, 1000000);
	memcpy(arg, &tstamp, sizeof(struct snd_compr_tstamp));

	return 0;
}

static int msm_compr_ack(struct snd_compr_stream *cstream,
			size_t count)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_compr_audio *prtd = runtime->private_data;
	void *src, *dstn;
	size_t copy;
	unsigned long flags;

	WARN(1, "This path is untested");
	return -EINVAL;

	pr_debug("%s: count = %d\n", __func__, count);
	if (!prtd->buffer) {
		pr_err("%s: Buffer is not allocated yet ??\n", __func__);
		return -EINVAL;
	}
	src = runtime->buffer + prtd->app_pointer;
	dstn = prtd->buffer + prtd->app_pointer;
	if (count < prtd->buffer_size - prtd->app_pointer) {
		memcpy(dstn, src, count);
		prtd->app_pointer += count;
	} else {
		copy = prtd->buffer_size - prtd->app_pointer;
		memcpy(dstn, src, copy);
		memcpy(prtd->buffer, runtime->buffer, count - copy);
		prtd->app_pointer = count - copy;
	}

	spin_lock_irqsave(&prtd->lock, flags);

	if (atomic_read(&prtd->start) &&
		prtd->bytes_received == prtd->copied_total) {
		prtd->bytes_received += count;
		msm_compr_send_buffer(prtd);
	} else
		prtd->bytes_received += count;

	spin_unlock_irqrestore(&prtd->lock, flags);

	return 0;
}

static int msm_compr_copy(struct snd_compr_stream *cstream,
			  char __user *buf, size_t count)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_compr_audio *prtd = runtime->private_data;
	void *dstn;
	size_t copy;
	size_t bytes_available = 0;
	unsigned long flags;

	pr_info("%s: count = %d\n", __func__, count);
	if (!prtd->buffer) {
		pr_err("%s: Buffer is not allocated yet ??", __func__);
		return 0;
	}

	spin_lock_irqsave(&prtd->lock, flags);
	if (atomic_read(&prtd->error)) {
		pr_err("%s Got RESET EVENTS notification", __func__);
		spin_unlock_irqrestore(&prtd->lock, flags);
		return -ENETRESET;
	}
	spin_unlock_irqrestore(&prtd->lock, flags);

	dstn = prtd->buffer + prtd->app_pointer;
	if (count < prtd->buffer_size - prtd->app_pointer) {
		if (copy_from_user(dstn, buf, count))
			return -EFAULT;
		prtd->app_pointer += count;
	} else {
		copy = prtd->buffer_size - prtd->app_pointer;
		if (copy_from_user(dstn, buf, copy))
			return -EFAULT;
		if (copy_from_user(prtd->buffer, buf + copy, count - copy))
			return -EFAULT;
		prtd->app_pointer = count - copy;
	}

	spin_lock_irqsave(&prtd->lock, flags);
	prtd->bytes_received += count;
	if (atomic_read(&prtd->start)) {
		if (atomic_read(&prtd->xrun)) {
			pr_debug("%s: in xrun, count = %d\n", __func__, count);
			bytes_available = prtd->bytes_received - prtd->copied_total;
			if (bytes_available >= runtime->fragment_size || prtd->first_buffer) { 
				
				if (prtd->stream_end){
					pr_info("reset lasttimestamp at msm_compr_send_buffer\n");
					prtd->stream_end = false;
				}
				
				pr_debug("%s: handle xrun, bytes_to_write = %d\n",
					 __func__,
					 bytes_available);
				atomic_set(&prtd->xrun, 0);
				msm_compr_send_buffer(prtd);
			} 
		} 
	}

	spin_unlock_irqrestore(&prtd->lock, flags);

	return count;
}

static int msm_compr_get_caps(struct snd_compr_stream *cstream,
				struct snd_compr_caps *arg)
{
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct msm_compr_audio *prtd = runtime->private_data;

	pr_debug("%s\n", __func__);
	memcpy(arg, &prtd->compr_cap, sizeof(struct snd_compr_caps));

	return 0;
}

static int msm_compr_get_codec_caps(struct snd_compr_stream *cstream,
				struct snd_compr_codec_caps *codec)
{
	pr_debug("%s\n", __func__);

	switch (codec->codec) {
	case SND_AUDIOCODEC_MP3:
#ifdef CONFIG_HD_AUDIO
		codec->num_descriptors = 3;
#else
		codec->num_descriptors = 2;
#endif
		codec->descriptor[0].max_ch = 2;
		codec->descriptor[0].sample_rates = SNDRV_PCM_RATE_8000_48000;
		codec->descriptor[0].bit_rate[0] = 320; 
		codec->descriptor[0].bit_rate[1] = 128;
		codec->descriptor[0].num_bitrates = 2;
		codec->descriptor[0].profiles = 0;
		codec->descriptor[0].modes = SND_AUDIOCHANMODE_MP3_STEREO;
		codec->descriptor[0].formats = 0;
		break;
	case SND_AUDIOCODEC_AAC:
#ifdef CONFIG_HD_AUDIO
		codec->num_descriptors = 3;
#else
		codec->num_descriptors = 2;
#endif
		codec->descriptor[1].max_ch = 2;
		codec->descriptor[1].sample_rates = SNDRV_PCM_RATE_8000_48000;
		codec->descriptor[1].bit_rate[0] = 320; 
		codec->descriptor[1].bit_rate[1] = 128;
		codec->descriptor[1].num_bitrates = 2;
		codec->descriptor[1].profiles = 0;
		codec->descriptor[1].modes = 0;
		codec->descriptor[1].formats =
			(SND_AUDIOSTREAMFORMAT_MP4ADTS |
				SND_AUDIOSTREAMFORMAT_RAW);
		break;
	case SND_AUDIOCODEC_AC3:
		break;
	case SND_AUDIOCODEC_EAC3:
		break;
#ifdef CONFIG_HD_AUDIO
	case SND_AUDIOCODEC_PCM:
		codec->num_descriptors = 3;
		codec->descriptor[2].max_ch = 2;
		codec->descriptor[2].sample_rates = SNDRV_PCM_RATE_8000_192000;
		codec->descriptor[2].bit_rate[0] = 192; 
		codec->descriptor[2].bit_rate[1] = 8;
		codec->descriptor[2].num_bitrates = 2;
		codec->descriptor[2].profiles = 0;
		codec->descriptor[2].modes = 0;
		codec->descriptor[2].formats = 0;
		break;
#endif
	default:
		pr_err("%s: Unsupported audio codec %d\n",
			__func__, codec->codec);
		return -EINVAL;
	}

	return 0;
}

static int msm_compr_config_effect(struct snd_compr_stream *cstream, void *data, void *payload)
{
   struct msm_compr_audio *prtd = cstream->runtime->private_data;
   struct dsp_effect_param *q6_param = (struct dsp_effect_param*)data;
   int rc = 0;
   if (!prtd->audio_client) {
       pr_err("%s: audio_client not found\n",
           __func__);
       return -EACCES;
   }
   rc = q6asm_enable_effect(prtd->audio_client,
               q6_param->module_id,
               q6_param->param_id,
               q6_param->payload_size,
               payload);
   pr_info("[%p] %s: call q6asm_enable_effect, rc %d\n",
       prtd, __func__, rc);
   return rc;
}

static int msm_compr_set_metadata(struct snd_compr_stream *cstream,
				struct snd_compr_metadata *metadata)
{
	struct msm_compr_audio *prtd;
	struct audio_client *ac;
	pr_debug("%s\n", __func__);

	if (!metadata || !cstream)
		return -EINVAL;

	prtd = cstream->runtime->private_data;
	if (!prtd && !prtd->audio_client)
		return -EINVAL;
	ac = prtd->audio_client;
	if (prtd->stream_end) {
		pr_info("reset lasttimestamp at setmetadata\n");
		prtd->stream_end = false;
		prtd->lasttimestamp = 0;
	}
	if (metadata->key == SNDRV_COMPRESS_ENCODER_PADDING) {
		pr_debug("%s, got encoder padding %u", __func__, metadata->value[0]);
		prtd->gapless_state.trailing_samples_drop = metadata->value[0];
	} else if (metadata->key == SNDRV_COMPRESS_ENCODER_DELAY) {
		pr_debug("%s, got encoder delay %u", __func__, metadata->value[0]);
		prtd->gapless_state.initial_samples_drop = metadata->value[0];
	}

	return 0;
}

static int msm_compr_volume_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_platform *platform = snd_kcontrol_chip(kcontrol);
	unsigned long fe_id = kcontrol->private_value;
	struct msm_compr_pdata *pdata = (struct msm_compr_pdata *)
			snd_soc_platform_get_drvdata(platform);
	struct snd_compr_stream *cstream = NULL;
	uint32_t *volume = NULL;

	if (fe_id >= MSM_FRONTEND_DAI_MAX) {
		pr_err("%s Received out of bounds fe_id %lu\n",
			__func__, fe_id);
		return -EINVAL;
	}

	cstream = pdata->cstream[fe_id];
	volume = pdata->volume[fe_id];

	volume[0] = ucontrol->value.integer.value[0];
	volume[1] = ucontrol->value.integer.value[1];
	pr_debug("%s: fe_id %lu left_vol %d right_vol %d\n",
		 __func__, fe_id, volume[0], volume[1]);
	if (cstream)
		msm_compr_set_volume(cstream, volume[0], volume[1]);
	return 0;
}

static int msm_compr_volume_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_platform *platform = snd_kcontrol_chip(kcontrol);
	unsigned long fe_id = kcontrol->private_value;

	struct msm_compr_pdata *pdata =
		snd_soc_platform_get_drvdata(platform);
	uint32_t *volume = NULL;

	if (fe_id >= MSM_FRONTEND_DAI_MAX) {
		pr_err("%s Received out of bound fe_id %lu\n", __func__, fe_id);
		return -EINVAL;
	}

	volume = pdata->volume[fe_id];
	pr_debug("%s: fe_id %lu\n", __func__, fe_id);
	ucontrol->value.integer.value[0] = volume[0];
	ucontrol->value.integer.value[1] = volume[1];

	return 0;
}

static int msm_compr_audio_effects_config_put(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_platform *platform = snd_kcontrol_chip(kcontrol);
	unsigned long fe_id = kcontrol->private_value;
	struct msm_compr_pdata *pdata = (struct msm_compr_pdata *)
			snd_soc_platform_get_drvdata(platform);
	struct msm_compr_audio_effects *audio_effects = NULL;
	struct snd_compr_stream *cstream = NULL;
	struct msm_compr_audio *prtd = NULL;
	long *values = &(ucontrol->value.integer.value[0]);
	int effects_module;

	pr_debug("%s\n", __func__);
	if (fe_id >= MSM_FRONTEND_DAI_MAX) {
		pr_err("%s Received out of bounds fe_id %lu\n",
			__func__, fe_id);
		return -EINVAL;
	}
	cstream = pdata->cstream[fe_id];
	audio_effects = pdata->audio_effects[fe_id];
	if (!cstream || !audio_effects) {
		pr_err("%s: stream or effects inactive\n", __func__);
		return -EINVAL;
	}
	prtd = cstream->runtime->private_data;
	if (!prtd) {
		pr_err("%s: cannot set audio effects\n", __func__);
		return -EINVAL;
	}
	effects_module = *values++;
	switch (effects_module) {
	case VIRTUALIZER_MODULE:
		pr_debug("%s: VIRTUALIZER_MODULE\n", __func__);
		msm_audio_effects_virtualizer_handler(prtd->audio_client,
						&(audio_effects->virtualizer),
						values);
		break;
	case REVERB_MODULE:
		pr_debug("%s: REVERB_MODULE\n", __func__);
		msm_audio_effects_reverb_handler(prtd->audio_client,
						 &(audio_effects->reverb),
						 values);
		break;
	case BASS_BOOST_MODULE:
		pr_debug("%s: BASS_BOOST_MODULE\n", __func__);
		msm_audio_effects_bass_boost_handler(prtd->audio_client,
						   &(audio_effects->bass_boost),
						     values);
		break;
	case EQ_MODULE:
		pr_debug("%s: EQ_MODULE\n", __func__);
		msm_audio_effects_popless_eq_handler(prtd->audio_client,
						    &(audio_effects->equalizer),
						     values);
		break;
	default:
		pr_err("%s Invalid effects config module\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int msm_compr_audio_effects_config_get(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_value *ucontrol)
{
	
	return 0;
}

static int msm_compr_dec_params_put(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_platform *platform = snd_kcontrol_chip(kcontrol);
	unsigned long fe_id = kcontrol->private_value;
	struct msm_compr_pdata *pdata = (struct msm_compr_pdata *)
			snd_soc_platform_get_drvdata(platform);
	struct msm_compr_dec_params *dec_params = NULL;
	struct snd_compr_stream *cstream = NULL;
	struct msm_compr_audio *prtd = NULL;
	long *values = &(ucontrol->value.integer.value[0]);

	pr_debug("%s\n", __func__);
	if (fe_id >= MSM_FRONTEND_DAI_MAX) {
		pr_err("%s Received out of bounds fe_id %lu\n",
			__func__, fe_id);
		return -EINVAL;
	}

	cstream = pdata->cstream[fe_id];
	dec_params = pdata->dec_params[fe_id];

	if (!cstream || !dec_params) {
		pr_err("%s: stream or dec_params inactive\n", __func__);
		return -EINVAL;
	}
	prtd = cstream->runtime->private_data;
	if (!prtd) {
		pr_err("%s: cannot set dec_params\n", __func__);
		return -EINVAL;
	}
	switch (prtd->codec) {
	case FORMAT_MP3:
	case FORMAT_MPEG4_AAC:
		pr_debug("%s: no runtime parameters for codec: %d\n", __func__,
			 prtd->codec);
		break;
	case FORMAT_AC3:
	case FORMAT_EAC3: {
		struct snd_dec_ddp *ddp = &dec_params->ddp_params;
		int cnt;
		ddp->params_length = (*values++);
		if (ddp->params_length > DDP_DEC_MAX_NUM_PARAM) {
			pr_err("%s: invalid num of params:: %d\n", __func__,
				ddp->params_length);
			return -EINVAL;
		}
		for (cnt = 0; cnt < ddp->params_length; cnt++) {
			ddp->params_id[cnt] = *values++;
			ddp->params_value[cnt] = *values++;
		}
		if (msm_compr_send_ddp_cfg(prtd->audio_client, ddp) < 0)
			pr_err("%s: DDP CMD CFG failed\n", __func__);
		break;
	}
	default:
		break;
	}
	return 0;
}

static int msm_compr_dec_params_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	
	return 0;
}

static int msm_compr_probe(struct snd_soc_platform *platform)
{
	struct msm_compr_pdata *pdata;
	int i;

	pr_debug("%s\n", __func__);
	pdata = (struct msm_compr_pdata *)
			kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	snd_soc_platform_set_drvdata(platform, pdata);

	atomic_set(&pdata->audio_ocmem_req, 0);

	for (i = 0; i < MSM_FRONTEND_DAI_MAX; i++) {
		pdata->volume[i][0] = COMPRESSED_LR_VOL_MAX_STEPS;
		pdata->volume[i][1] = COMPRESSED_LR_VOL_MAX_STEPS;
		pdata->audio_effects[i] = NULL;
		pdata->dec_params[i] = NULL;
		pdata->cstream[i] = NULL;
	}

	pdata->use_dsp_gapless_mode = false;
	return 0;
}

static int msm_compr_volume_info(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = COMPRESSED_LR_VOL_MAX_STEPS;
	return 0;
}

static int msm_compr_audio_effects_config_info(struct snd_kcontrol *kcontrol,
					       struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 128;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0xFFFFFFFF;
	return 0;
}

static int msm_compr_dec_params_info(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 128;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0xFFFFFFFF;
	return 0;
}

static int msm_compr_add_volume_control(struct snd_soc_pcm_runtime *rtd)
{
	const char *mixer_ctl_name = "Compress Playback";
	const char *deviceNo       = "NN";
	const char *suffix         = "Volume";
	char *mixer_str = NULL;
	int ctl_len;
	struct snd_kcontrol_new fe_volume_control[1] = {
		{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "?",
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |
			  SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = msm_compr_volume_info,
		.tlv.p = msm_compr_vol_gain,
		.get = msm_compr_volume_get,
		.put = msm_compr_volume_put,
		.private_value = 0,
		}
	};

	if (!rtd) {
		pr_err("%s NULL rtd\n", __func__);
		return 0;
	}
	pr_debug("%s: added new compr FE with name %s, id %d, cpu dai %s, device no %d\n",
		 __func__, rtd->dai_link->name, rtd->dai_link->be_id,
		 rtd->dai_link->cpu_dai_name, rtd->pcm->device);
	ctl_len = strlen(mixer_ctl_name) + 1 + strlen(deviceNo) + 1 +
		  strlen(suffix) + 1;
	mixer_str = kzalloc(ctl_len, GFP_KERNEL);
	if (!mixer_str) {
		pr_err("failed to allocate mixer ctrl str of len %d", ctl_len);
		return 0;
	}
	snprintf(mixer_str, ctl_len, "%s %d %s", mixer_ctl_name,
		 rtd->pcm->device, suffix);
	fe_volume_control[0].name = mixer_str;
	fe_volume_control[0].private_value = rtd->dai_link->be_id;
	pr_debug("Registering new mixer ctl %s", mixer_str);
	snd_soc_add_platform_controls(rtd->platform, fe_volume_control,
				      ARRAY_SIZE(fe_volume_control));
	kfree(mixer_str);
	return 0;
}

static int msm_compr_add_audio_effects_control(struct snd_soc_pcm_runtime *rtd)
{
	const char *mixer_ctl_name = "Audio Effects Config";
	const char *deviceNo       = "NN";
	char *mixer_str = NULL;
	int ctl_len;
	struct snd_kcontrol_new fe_audio_effects_config_control[1] = {
		{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "?",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = msm_compr_audio_effects_config_info,
		.get = msm_compr_audio_effects_config_get,
		.put = msm_compr_audio_effects_config_put,
		.private_value = 0,
		}
	};


	if (!rtd) {
		pr_err("%s NULL rtd\n", __func__);
		return 0;
	}

	pr_debug("%s: added new compr FE with name %s, id %d, cpu dai %s, device no %d\n",
		 __func__, rtd->dai_link->name, rtd->dai_link->be_id,
		 rtd->dai_link->cpu_dai_name, rtd->pcm->device);

	ctl_len = strlen(mixer_ctl_name) + 1 + strlen(deviceNo) + 1;
	mixer_str = kzalloc(ctl_len, GFP_KERNEL);

	if (!mixer_str) {
		pr_err("failed to allocate mixer ctrl str of len %d", ctl_len);
		return 0;
	}

	snprintf(mixer_str, ctl_len, "%s %d", mixer_ctl_name, rtd->pcm->device);

	fe_audio_effects_config_control[0].name = mixer_str;
	fe_audio_effects_config_control[0].private_value = rtd->dai_link->be_id;
	pr_debug("Registering new mixer ctl %s\n", mixer_str);
	snd_soc_add_platform_controls(rtd->platform,
				fe_audio_effects_config_control,
				ARRAY_SIZE(fe_audio_effects_config_control));
	kfree(mixer_str);
	return 0;
}

static int msm_compr_gapless_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_platform *platform = snd_kcontrol_chip(kcontrol);
	struct msm_compr_pdata *pdata = (struct msm_compr_pdata *)
		snd_soc_platform_get_drvdata(platform);
	pdata->use_dsp_gapless_mode =  ucontrol->value.integer.value[0];
	pr_debug("%s: value: %ld\n", __func__,
		ucontrol->value.integer.value[0]);

	return 0;
}

static int msm_compr_gapless_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_platform *platform = snd_kcontrol_chip(kcontrol);
	struct msm_compr_pdata *pdata =
		snd_soc_platform_get_drvdata(platform);
	pr_debug("%s:gapless mode %d\n", __func__, pdata->use_dsp_gapless_mode);
	ucontrol->value.integer.value[0] = pdata->use_dsp_gapless_mode;

	return 0;
}

static const struct snd_kcontrol_new msm_compr_gapless_controls[] = {
	SOC_SINGLE_EXT("Compress Gapless Playback",
			0, 0, 1, 0,
			msm_compr_gapless_get,
			msm_compr_gapless_put),
};

static int msm_compr_add_dec_runtime_params_control(
						struct snd_soc_pcm_runtime *rtd)
{
	const char *mixer_ctl_name	= "Audio Stream";
	const char *deviceNo		= "NN";
	const char *suffix		= "Dec Params";
	char *mixer_str = NULL;
	int ctl_len;
	struct snd_kcontrol_new fe_dec_params_control[1] = {
		{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "?",
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = msm_compr_dec_params_info,
		.get = msm_compr_dec_params_get,
		.put = msm_compr_dec_params_put,
		.private_value = 0,
		}
	};

	if (!rtd) {
		pr_err("%s NULL rtd\n", __func__);
		return 0;
	}

	pr_debug("%s: added new compr FE with name %s, id %d, cpu dai %s, device no %d\n",
		 __func__, rtd->dai_link->name, rtd->dai_link->be_id,
		 rtd->dai_link->cpu_dai_name, rtd->pcm->device);

	ctl_len = strlen(mixer_ctl_name) + 1 + strlen(deviceNo) + 1 +
		  strlen(suffix) + 1;
	mixer_str = kzalloc(ctl_len, GFP_KERNEL);

	if (!mixer_str) {
		pr_err("failed to allocate mixer ctrl str of len %d", ctl_len);
		return 0;
	}

	snprintf(mixer_str, ctl_len, "%s %d %s", mixer_ctl_name,
		 rtd->pcm->device, suffix);

	fe_dec_params_control[0].name = mixer_str;
	fe_dec_params_control[0].private_value = rtd->dai_link->be_id;
	pr_debug("Registering new mixer ctl %s", mixer_str);
	snd_soc_add_platform_controls(rtd->platform,
				      fe_dec_params_control,
				      ARRAY_SIZE(fe_dec_params_control));
	kfree(mixer_str);
	return 0;
}

static int msm_compr_new(struct snd_soc_pcm_runtime *rtd)
{
	int rc;

	rc = msm_compr_add_volume_control(rtd);
	if (rc)
		pr_err("%s: Could not add Compr Volume Control\n", __func__);
	rc = msm_compr_add_audio_effects_control(rtd);
	if (rc)
		pr_err("%s: Could not add Compr Audio Effects Control\n",
			__func__);
	rc = msm_compr_add_dec_runtime_params_control(rtd);
	if (rc)
		pr_err("%s: Could not add Compr Dec runtime params Control\n",
			__func__);
	return 0;
}

static struct snd_compr_ops msm_compr_ops = {
	.open		= msm_compr_open,
	.free		= msm_compr_free,
	.trigger	= msm_compr_trigger,
	.pointer	= msm_compr_pointer,
	.set_params	= msm_compr_set_params,
	.set_metadata	= msm_compr_set_metadata,
	.ack		= msm_compr_ack,
	.copy		= msm_compr_copy,
	.get_caps	= msm_compr_get_caps,
	.get_codec_caps = msm_compr_get_codec_caps,
	.config_effect = msm_compr_config_effect,
};

static struct snd_soc_platform_driver msm_soc_platform = {
	.probe		= msm_compr_probe,
	.compr_ops	= &msm_compr_ops,
	.pcm_new	= msm_compr_new,
	.controls       = msm_compr_gapless_controls,
	.num_controls   = ARRAY_SIZE(msm_compr_gapless_controls),

};

static __devinit int msm_compr_dev_probe(struct platform_device *pdev)
{
	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s", "msm-compress-dsp");

	pr_debug("%s: dev name %s\n", __func__, dev_name(&pdev->dev));
	return snd_soc_register_platform(&pdev->dev,
					&msm_soc_platform);
}

static int msm_compr_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static const struct of_device_id msm_compr_dt_match[] = {
	{.compatible = "qcom,msm-compress-dsp"},
	{}
};
MODULE_DEVICE_TABLE(of, msm_compr_dt_match);

static struct platform_driver msm_compr_driver = {
	.driver = {
		.name = "msm-compress-dsp",
		.owner = THIS_MODULE,
		.of_match_table = msm_compr_dt_match,
	},
	.probe = msm_compr_dev_probe,
	.remove = __devexit_p(msm_compr_remove),
};

static int __init msm_soc_platform_init(void)
{
	wake_lock_init(&compr_lpa_q6_cb_wakelock, WAKE_LOCK_SUSPEND, "compr_lpa_q6_cb");
	return platform_driver_register(&msm_compr_driver);
}
module_init(msm_soc_platform_init);

static void __exit msm_soc_platform_exit(void)
{
	platform_driver_unregister(&msm_compr_driver);
}
module_exit(msm_soc_platform_exit);

MODULE_DESCRIPTION("Compress Offload platform driver");
MODULE_LICENSE("GPL v2");
