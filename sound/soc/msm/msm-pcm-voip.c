/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <asm/dma.h>

#include "msm-pcm-q6.h"
#include "msm-pcm-routing.h"
#include "qdsp6/q6voice.h"

#define VOIP_MAX_Q_LEN 10
#define VOIP_MAX_VOC_PKT_SIZE 640
#define VOIP_MIN_VOC_PKT_SIZE 320

/* Length of the DSP frame info header added to the voc packet. */
#define DSP_FRAME_HDR_LEN 1

#define MODE_IS127		0x2
#define MODE_4GV_NB		0x3
#define MODE_4GV_WB		0x4
#define MODE_AMR		0x5
#define MODE_AMR_WB		0xD
#define MODE_PCM		0xC
#define MODE_G711		0xA
#define MODE_G711A		0xF

enum msm_audio_g711a_frame_type {
	MVS_G711A_SPEECH_GOOD,
	MVS_G711A_SID,
	MVS_G711A_NO_DATA,
	MVS_G711A_ERASURE
};

enum msm_audio_g711a_mode {
	MVS_G711A_MODE_MULAW,
	MVS_G711A_MODE_ALAW
};

enum msm_audio_g711_mode {
	MVS_G711_MODE_MULAW,
	MVS_G711_MODE_ALAW
};

enum format {
	FORMAT_S16_LE = 2,
	FORMAT_SPECIAL = 31,
};


enum amr_rate_type {
	AMR_RATE_4750, /* AMR 4.75 kbps */
	AMR_RATE_5150, /* AMR 5.15 kbps */
	AMR_RATE_5900, /* AMR 5.90 kbps */
	AMR_RATE_6700, /* AMR 6.70 kbps */
	AMR_RATE_7400, /* AMR 7.40 kbps */
	AMR_RATE_7950, /* AMR 7.95 kbps */
	AMR_RATE_10200, /* AMR 10.20 kbps */
	AMR_RATE_12200, /* AMR 12.20 kbps */
	AMR_RATE_6600, /* AMR-WB 6.60 kbps */
	AMR_RATE_8850, /* AMR-WB 8.85 kbps */
	AMR_RATE_12650, /* AMR-WB 12.65 kbps */
	AMR_RATE_14250, /* AMR-WB 14.25 kbps */
	AMR_RATE_15850, /* AMR-WB 15.85 kbps */
	AMR_RATE_18250, /* AMR-WB 18.25 kbps */
	AMR_RATE_19850, /* AMR-WB 19.85 kbps */
	AMR_RATE_23050, /* AMR-WB 23.05 kbps */
	AMR_RATE_23850, /* AMR-WB 23.85 kbps */
	AMR_RATE_UNDEF
};

enum voip_state {
	VOIP_STOPPED,
	VOIP_STARTED,
};

struct voip_frame {
	union {
	uint32_t frame_type;
	uint32_t packet_rate;
	} header;
	uint32_t len;
	uint8_t voc_pkt[VOIP_MAX_VOC_PKT_SIZE];
};

struct voip_buf_node {
	struct list_head list;
	struct voip_frame frame;
};

struct voip_drv_info {
	enum  voip_state state;

	struct snd_pcm_substream *playback_substream;
	struct snd_pcm_substream *capture_substream;

	struct list_head in_queue;
	struct list_head free_in_queue;

	struct list_head out_queue;
	struct list_head free_out_queue;

	wait_queue_head_t out_wait;
	wait_queue_head_t in_wait;

	struct mutex lock;

	spinlock_t dsp_lock;
	spinlock_t dsp_ul_lock;

	uint32_t mode;
	uint32_t rate_type;
	uint32_t rate;
	uint32_t dtx_mode;

	uint8_t capture_start;
	uint8_t playback_start;

	uint8_t playback_instance;
	uint8_t capture_instance;

	unsigned int play_samp_rate;
	unsigned int cap_samp_rate;

	unsigned int pcm_size;
	unsigned int pcm_count;
	unsigned int pcm_playback_irq_pos;      /* IRQ position */
	unsigned int pcm_playback_buf_pos;      /* position in buffer */

	unsigned int pcm_capture_size;
	unsigned int pcm_capture_count;
	unsigned int pcm_capture_irq_pos;       /* IRQ position */
	unsigned int pcm_capture_buf_pos;       /* position in buffer */
};

static int voip_get_media_type(uint32_t mode, uint32_t rate_type,
				unsigned int samp_rate,
				uint32_t *media_type);
static int voip_get_rate_type(uint32_t mode,
				uint32_t rate,
				uint32_t *rate_type);
static int msm_voip_mode_rate_config_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol);
static int msm_voip_mode_rate_config_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol);

static struct voip_drv_info voip_info;

static struct snd_pcm_hardware msm_pcm_hardware = {
	.info =                 (SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_BLOCK_TRANSFER |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_INTERLEAVED),
	.formats =              SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_SPECIAL,
	.rates =                SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
	.rate_min =             8000,
	.rate_max =             16000,
	.channels_min =         1,
	.channels_max =         1,
	.buffer_bytes_max =	sizeof(struct voip_buf_node) * VOIP_MAX_Q_LEN,
	.period_bytes_min =	VOIP_MIN_VOC_PKT_SIZE,
	.period_bytes_max =	VOIP_MAX_VOC_PKT_SIZE,
	.periods_min =		VOIP_MAX_Q_LEN,
	.periods_max =		VOIP_MAX_Q_LEN,
	.fifo_size =            0,
};


static int msm_voip_mute_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int mute = ucontrol->value.integer.value[0];

	pr_debug("%s: mute=%d\n", __func__, mute);

	voc_set_tx_mute(voc_get_session_id(VOIP_SESSION_NAME), TX_PATH, mute);

	return 0;
}

static int msm_voip_mute_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0;
	return 0;
}

static int msm_voip_volume_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int volume = ucontrol->value.integer.value[0];

	pr_debug("%s: volume: %d\n", __func__, volume);

	voc_set_rx_vol_index(voc_get_session_id(VOIP_SESSION_NAME),
			     RX_PATH,
			     volume);
	return 0;
}
static int msm_voip_volume_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0;
	return 0;
}

static int msm_voip_dtx_mode_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	mutex_lock(&voip_info.lock);

	voip_info.dtx_mode  = ucontrol->value.integer.value[0];

	pr_debug("%s: dtx: %d\n", __func__, voip_info.dtx_mode);

	mutex_unlock(&voip_info.lock);

	return 0;
}
static int msm_voip_dtx_mode_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	mutex_lock(&voip_info.lock);

	ucontrol->value.integer.value[0] = voip_info.dtx_mode;

	mutex_unlock(&voip_info.lock);

	return 0;
}

static struct snd_kcontrol_new msm_voip_controls[] = {
	SOC_SINGLE_EXT("Voip Tx Mute", SND_SOC_NOPM, 0, 1, 0,
				msm_voip_mute_get, msm_voip_mute_put),
	SOC_SINGLE_EXT("Voip Rx Volume", SND_SOC_NOPM, 0, 5, 0,
				msm_voip_volume_get, msm_voip_volume_put),
	SOC_SINGLE_MULTI_EXT("Voip Mode Rate Config", SND_SOC_NOPM, 0, 23850,
				0, 2, msm_voip_mode_rate_config_get,
				msm_voip_mode_rate_config_put),
	SOC_SINGLE_EXT("Voip Dtx Mode", SND_SOC_NOPM, 0, 1, 0,
				msm_voip_dtx_mode_get, msm_voip_dtx_mode_put),
};

static int msm_pcm_voip_probe(struct snd_soc_platform *platform)
{
	snd_soc_add_platform_controls(platform, msm_voip_controls,
					ARRAY_SIZE(msm_voip_controls));

	return 0;
}

/* sample rate supported */
static unsigned int supported_sample_rates[] = {8000, 16000};

/* capture path */
static void voip_process_ul_pkt(uint8_t *voc_pkt,
					uint32_t pkt_len,
					void *private_data)
{
	struct voip_buf_node *buf_node = NULL;
	struct voip_drv_info *prtd = private_data;
	unsigned long dsp_flags;

	if (prtd->capture_substream == NULL)
		return;

	/* Copy up-link packet into out_queue. */
	spin_lock_irqsave(&prtd->dsp_ul_lock, dsp_flags);

	/* discarding UL packets till start is received */
	if (!list_empty(&prtd->free_out_queue) && prtd->capture_start) {
		buf_node = list_first_entry(&prtd->free_out_queue,
					struct voip_buf_node, list);
		list_del(&buf_node->list);
		switch (prtd->mode) {
		case MODE_AMR_WB:
		case MODE_AMR: {
			/* Remove the DSP frame info header. Header format:
			 * Bits 0-3: Frame rate
			 * Bits 4-7: Frame type
			 */
			buf_node->frame.header.frame_type =
						((*voc_pkt) & 0xF0) >> 4;
			voc_pkt = voc_pkt + DSP_FRAME_HDR_LEN;
			buf_node->frame.len = pkt_len - DSP_FRAME_HDR_LEN;
			memcpy(&buf_node->frame.voc_pkt[0],
				voc_pkt,
				buf_node->frame.len);
			list_add_tail(&buf_node->list, &prtd->out_queue);
			break;
		}
		case MODE_IS127:
		case MODE_4GV_NB:
		case MODE_4GV_WB: {
			/* Remove the DSP frame info header.
			 * Header format:
			 * Bits 0-3: frame rate
			 */
			buf_node->frame.header.packet_rate = (*voc_pkt) & 0x0F;
			voc_pkt = voc_pkt + DSP_FRAME_HDR_LEN;
			buf_node->frame.len = pkt_len - DSP_FRAME_HDR_LEN;

			memcpy(&buf_node->frame.voc_pkt[0],
				voc_pkt,
				buf_node->frame.len);

			list_add_tail(&buf_node->list, &prtd->out_queue);
			break;
		}
		case MODE_G711:
		case MODE_G711A:{
			/* G711 frames are 10ms each, but the DSP works with
			 * 20ms frames and sends two 10ms frames per buffer.
			 * Extract the two frames and put them in separate
			 * buffers.
			 */
			/* Remove the first DSP frame info header.
			 * Header format: G711A
			 * Bits 0-1: Frame type
			 * Bits 2-3: Frame rate
			 *
			 * Header format: G711
			 * Bits 2-3: Frame rate
			 */
			if (prtd->mode == MODE_G711A)
				buf_node->frame.header.frame_type =
							(*voc_pkt) & 0x03;
			voc_pkt = voc_pkt + DSP_FRAME_HDR_LEN;

			/* There are two frames in the buffer. Length of the
			 * first frame:
			 */
			buf_node->frame.len = (pkt_len -
					       2 * DSP_FRAME_HDR_LEN) / 2;

			memcpy(&buf_node->frame.voc_pkt[0],
			       voc_pkt,
			       buf_node->frame.len);
			voc_pkt = voc_pkt + buf_node->frame.len;

			list_add_tail(&buf_node->list, &prtd->out_queue);

			/* Get another buffer from the free Q and fill in the
			 * second frame.
			 */
			if (!list_empty(&prtd->free_out_queue)) {
				buf_node =
					list_first_entry(&prtd->free_out_queue,
							 struct voip_buf_node,
							 list);
				list_del(&buf_node->list);

				/* Remove the second DSP frame info header.
				 * Header format:
				 * Bits 0-1: Frame type
				 * Bits 2-3: Frame rate
				 */

				if (prtd->mode == MODE_G711A)
					buf_node->frame.header.frame_type =
							(*voc_pkt) & 0x03;
				voc_pkt = voc_pkt + DSP_FRAME_HDR_LEN;

				/* There are two frames in the buffer. Length
				 * of the second frame:
				 */
				buf_node->frame.len = (pkt_len -
					2 * DSP_FRAME_HDR_LEN) / 2;

				memcpy(&buf_node->frame.voc_pkt[0],
				       voc_pkt,
				       buf_node->frame.len);

				list_add_tail(&buf_node->list,
					      &prtd->out_queue);
			} else {
				/* Drop the second frame */
				pr_err("%s: UL data dropped, read is slow\n",
				       __func__);
			}
			break;
		}
		default: {
			buf_node->frame.len = pkt_len;
			memcpy(&buf_node->frame.voc_pkt[0],
				voc_pkt,
				buf_node->frame.len);
			list_add_tail(&buf_node->list, &prtd->out_queue);
		}
		}
		pr_debug("ul_pkt: pkt_len =%d, frame.len=%d\n", pkt_len,
			buf_node->frame.len);
		prtd->pcm_capture_irq_pos += prtd->pcm_capture_count;
		spin_unlock_irqrestore(&prtd->dsp_ul_lock, dsp_flags);
		snd_pcm_period_elapsed(prtd->capture_substream);
	} else {
		spin_unlock_irqrestore(&prtd->dsp_ul_lock, dsp_flags);
		pr_err("UL data dropped\n");
	}

	wake_up(&prtd->out_wait);
}

/* playback path */
static void voip_process_dl_pkt(uint8_t *voc_pkt,
					uint32_t *pkt_len,
					void *private_data)
{
	struct voip_buf_node *buf_node = NULL;
	struct voip_drv_info *prtd = private_data;
	unsigned long dsp_flags;


	if (prtd->playback_substream == NULL)
		return;

	spin_lock_irqsave(&prtd->dsp_lock, dsp_flags);

	if (!list_empty(&prtd->in_queue) && prtd->playback_start) {
		buf_node = list_first_entry(&prtd->in_queue,
				struct voip_buf_node, list);
		list_del(&buf_node->list);
		switch (prtd->mode) {
		case MODE_AMR:
		case MODE_AMR_WB: {
			/* Add the DSP frame info header. Header format:
			 * Bits 0-3: Frame rate
			 * Bits 4-7: Frame type
			 */
			*voc_pkt = ((buf_node->frame.header.frame_type &
					0x0F) << 4) | (prtd->rate_type & 0x0F);
			voc_pkt = voc_pkt + DSP_FRAME_HDR_LEN;
			*pkt_len = buf_node->frame.len + DSP_FRAME_HDR_LEN;
			memcpy(voc_pkt,
				&buf_node->frame.voc_pkt[0],
				buf_node->frame.len);
			list_add_tail(&buf_node->list, &prtd->free_in_queue);
			break;
		}
		case MODE_IS127:
		case MODE_4GV_NB:
		case MODE_4GV_WB: {
			/* Add the DSP frame info header. Header format:
			 * Bits 0-3 : Frame rate
			*/
			*voc_pkt = buf_node->frame.header.packet_rate & 0x0F;
			voc_pkt = voc_pkt + DSP_FRAME_HDR_LEN;
			*pkt_len = buf_node->frame.len + DSP_FRAME_HDR_LEN;

			memcpy(voc_pkt,
				&buf_node->frame.voc_pkt[0],
				buf_node->frame.len);

			list_add_tail(&buf_node->list, &prtd->free_in_queue);
			break;
		}
		case MODE_G711:
		case MODE_G711A:{
			/* G711 frames are 10ms each but the DSP expects 20ms
			 * worth of data, so send two 10ms frames per buffer.
			 */
			/* Add the first DSP frame info header. Header format:
			 * Bits 0-1: Frame type
			 * Bits 2-3: Frame rate
			 */

			*voc_pkt = ((prtd->rate_type  & 0x0F) << 2) |
				    (buf_node->frame.header.frame_type & 0x03);
			voc_pkt = voc_pkt + DSP_FRAME_HDR_LEN;

			*pkt_len = buf_node->frame.len + DSP_FRAME_HDR_LEN;

			memcpy(voc_pkt,
			       &buf_node->frame.voc_pkt[0],
			       buf_node->frame.len);
			voc_pkt = voc_pkt + buf_node->frame.len;

			list_add_tail(&buf_node->list, &prtd->free_in_queue);

			if (!list_empty(&prtd->in_queue)) {
				/* Get the second buffer. */
				buf_node = list_first_entry(&prtd->in_queue,
							struct voip_buf_node,
							list);
				list_del(&buf_node->list);

				/* Add the second DSP frame info header.
				 * Header format:
				 * Bits 0-1: Frame type
				 * Bits 2-3: Frame rate
				 */
				*voc_pkt = ((prtd->rate_type & 0x0F) << 2) |
				(buf_node->frame.header.frame_type & 0x03);
				voc_pkt = voc_pkt + DSP_FRAME_HDR_LEN;

				*pkt_len = *pkt_len +
					buf_node->frame.len + DSP_FRAME_HDR_LEN;

				memcpy(voc_pkt,
				       &buf_node->frame.voc_pkt[0],
				       buf_node->frame.len);

				list_add_tail(&buf_node->list,
					      &prtd->free_in_queue);
			} else {
				/* Only 10ms worth of data is available, signal
				 * erasure frame.
				 */
				*voc_pkt = ((prtd->rate_type & 0x0F) << 2) |
					    (MVS_G711A_ERASURE & 0x03);

				*pkt_len = *pkt_len + DSP_FRAME_HDR_LEN;
				pr_debug("%s, Only 10ms read, erase 2nd frame\n",
					 __func__);
			}
			break;
		}
		default: {
			*pkt_len = buf_node->frame.len;

			memcpy(voc_pkt,
				&buf_node->frame.voc_pkt[0],
				buf_node->frame.len);

			list_add_tail(&buf_node->list, &prtd->free_in_queue);
		}
		}
		pr_debug("dl_pkt: pkt_len=%d, frame_len=%d\n", *pkt_len,
			buf_node->frame.len);
		prtd->pcm_playback_irq_pos += prtd->pcm_count;
		spin_unlock_irqrestore(&prtd->dsp_lock, dsp_flags);
		snd_pcm_period_elapsed(prtd->playback_substream);
	} else {
		*pkt_len = 0;
		spin_unlock_irqrestore(&prtd->dsp_lock, dsp_flags);
		pr_err("DL data not available\n");
	}
	wake_up(&prtd->in_wait);
}

static struct snd_pcm_hw_constraint_list constraints_sample_rates = {
	.count = ARRAY_SIZE(supported_sample_rates),
	.list = supported_sample_rates,
	.mask = 0,
};

static int msm_pcm_playback_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct voip_drv_info *prtd = runtime->private_data;

	prtd->play_samp_rate = runtime->rate;
	prtd->pcm_size = snd_pcm_lib_buffer_bytes(substream);
	prtd->pcm_count = snd_pcm_lib_period_bytes(substream);
	prtd->pcm_playback_irq_pos = 0;
	prtd->pcm_playback_buf_pos = 0;

	return 0;
}

static int msm_pcm_capture_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct voip_drv_info *prtd = runtime->private_data;
	int ret = 0;

	prtd->cap_samp_rate = runtime->rate;
	prtd->pcm_capture_size  = snd_pcm_lib_buffer_bytes(substream);
	prtd->pcm_capture_count = snd_pcm_lib_period_bytes(substream);
	prtd->pcm_capture_irq_pos = 0;
	prtd->pcm_capture_buf_pos = 0;
	return ret;
}

static int msm_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct voip_drv_info *prtd = runtime->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		pr_debug("%s: Trigger start\n", __func__);
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			prtd->capture_start = 1;
		else
			prtd->playback_start = 1;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		pr_debug("SNDRV_PCM_TRIGGER_STOP\n");
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			prtd->playback_start = 0;
		else
			prtd->capture_start = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int msm_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct voip_drv_info *prtd = &voip_info;
	int ret = 0;

	pr_debug("%s, VoIP\n", __func__);
	mutex_lock(&prtd->lock);

	runtime->hw = msm_pcm_hardware;

	ret = snd_pcm_hw_constraint_list(runtime, 0,
					SNDRV_PCM_HW_PARAM_RATE,
					&constraints_sample_rates);
	if (ret < 0)
		pr_debug("snd_pcm_hw_constraint_list failed\n");

	ret = snd_pcm_hw_constraint_integer(runtime,
					SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0) {
		pr_debug("snd_pcm_hw_constraint_integer failed\n");
		goto err;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		prtd->playback_substream = substream;
		prtd->playback_instance++;
	} else {
		prtd->capture_substream = substream;
		prtd->capture_instance++;
	}
	runtime->private_data = prtd;
err:
	mutex_unlock(&prtd->lock);

	return ret;
}

static int msm_pcm_playback_copy(struct snd_pcm_substream *substream, int a,
	snd_pcm_uframes_t hwoff, void __user *buf, snd_pcm_uframes_t frames)
{
	int ret = 0;
	struct voip_buf_node *buf_node = NULL;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct voip_drv_info *prtd = runtime->private_data;
	unsigned long dsp_flags;

	int count = frames_to_bytes(runtime, frames);
	pr_debug("%s: count = %d, frames=%d\n", __func__, count, (int)frames);

	ret = wait_event_interruptible_timeout(prtd->in_wait,
				(!list_empty(&prtd->free_in_queue) ||
				prtd->state == VOIP_STOPPED),
				1 * HZ);
	if (ret > 0) {
		if (count <= VOIP_MAX_VOC_PKT_SIZE) {
			spin_lock_irqsave(&prtd->dsp_lock, dsp_flags);
			buf_node =
				list_first_entry(&prtd->free_in_queue,
						struct voip_buf_node, list);
			list_del(&buf_node->list);
			spin_unlock_irqrestore(&prtd->dsp_lock, dsp_flags);
			if (prtd->mode == MODE_PCM) {
				ret = copy_from_user(&buf_node->frame.voc_pkt,
							buf, count);
				buf_node->frame.len = count;
			} else
				ret = copy_from_user(&buf_node->frame,
							buf, count);
			spin_lock_irqsave(&prtd->dsp_lock, dsp_flags);
			list_add_tail(&buf_node->list, &prtd->in_queue);
			spin_unlock_irqrestore(&prtd->dsp_lock, dsp_flags);
		} else {
			pr_err("%s: Write cnt %d is > VOIP_MAX_VOC_PKT_SIZE\n",
				__func__, count);
			ret = -ENOMEM;
		}

	} else if (ret == 0) {
		pr_err("%s: No free DL buffs\n", __func__);
		ret = -ETIMEDOUT;
	} else {
		pr_err("%s: playback copy  was interrupted\n", __func__);
	}

	return  ret;
}
static int msm_pcm_capture_copy(struct snd_pcm_substream *substream,
		int channel, snd_pcm_uframes_t hwoff, void __user *buf,
						snd_pcm_uframes_t frames)
{
	int ret = 0;
	int count = 0;
	struct voip_buf_node *buf_node = NULL;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct voip_drv_info *prtd = runtime->private_data;
	unsigned long dsp_flags;
	int size;

	count = frames_to_bytes(runtime, frames);

	pr_debug("%s: count = %d\n", __func__, count);

	ret = wait_event_interruptible_timeout(prtd->out_wait,
				(!list_empty(&prtd->out_queue) ||
				prtd->state == VOIP_STOPPED),
				1 * HZ);

	if (ret > 0) {

		if (count <= VOIP_MAX_VOC_PKT_SIZE) {
			spin_lock_irqsave(&prtd->dsp_ul_lock, dsp_flags);
			buf_node = list_first_entry(&prtd->out_queue,
					struct voip_buf_node, list);
			list_del(&buf_node->list);
			spin_unlock_irqrestore(&prtd->dsp_ul_lock, dsp_flags);
			if (prtd->mode == MODE_PCM) {
				ret = copy_to_user(buf,
						   &buf_node->frame.voc_pkt,
						   buf_node->frame.len);
			} else {
				size = sizeof(buf_node->frame.header) +
				       sizeof(buf_node->frame.len) +
				       buf_node->frame.len;

				ret = copy_to_user(buf,
						   &buf_node->frame,
						   size;
			}
			if (ret) {
				pr_err("%s: Copy to user retuned %d\n",
					__func__, ret);
				ret = -EFAULT;
			}
			spin_lock_irqsave(&prtd->dsp_ul_lock, dsp_flags);
			list_add_tail(&buf_node->list,
						&prtd->free_out_queue);
			spin_unlock_irqrestore(&prtd->dsp_ul_lock, dsp_flags);

		} else {
			pr_err("%s: Read count %d > VOIP_MAX_VOC_PKT_SIZE\n",
				__func__, count);
			ret = -ENOMEM;
		}


	} else if (ret == 0) {
		pr_err("%s: No UL data available\n", __func__);
		ret = -ETIMEDOUT;
	} else {
		pr_err("%s: Read was interrupted\n", __func__);
		ret = -ERESTARTSYS;
	}
	return ret;
}
static int msm_pcm_copy(struct snd_pcm_substream *substream, int a,
	 snd_pcm_uframes_t hwoff, void __user *buf, snd_pcm_uframes_t frames)
{
	int ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = msm_pcm_playback_copy(substream, a, hwoff, buf, frames);
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		ret = msm_pcm_capture_copy(substream, a, hwoff, buf, frames);

	return ret;
}

static int msm_pcm_close(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct list_head *ptr = NULL;
	struct list_head *next = NULL;
	struct voip_buf_node *buf_node = NULL;
	struct snd_dma_buffer *p_dma_buf, *c_dma_buf;
	struct snd_pcm_substream *p_substream, *c_substream;
	struct snd_pcm_runtime *runtime;
	struct voip_drv_info *prtd;
	unsigned long dsp_flags;

	if (substream == NULL) {
		pr_err("substream is NULL\n");
		return -EINVAL;
	}
	runtime = substream->runtime;
	prtd = runtime->private_data;

	wake_up(&prtd->out_wait);

	mutex_lock(&prtd->lock);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		prtd->playback_instance--;
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		prtd->capture_instance--;

	if (!prtd->playback_instance && !prtd->capture_instance) {
		if (prtd->state == VOIP_STARTED) {
			prtd->state = VOIP_STOPPED;
			voc_end_voice_call(
					voc_get_session_id(VOIP_SESSION_NAME));
			voc_register_mvs_cb(NULL, NULL, prtd);
		}
		/* release all buffer */
		/* release in_queue and free_in_queue */
		pr_debug("release all buffer\n");
		p_substream = prtd->playback_substream;
		if (p_substream == NULL) {
			pr_debug("p_substream is NULL\n");
			goto capt;
		}
		p_dma_buf = &p_substream->dma_buffer;
		if (p_dma_buf == NULL) {
			pr_debug("p_dma_buf is NULL\n");
			goto capt;
		}
		if (p_dma_buf->area != NULL) {
			spin_lock_irqsave(&prtd->dsp_lock, dsp_flags);
			list_for_each_safe(ptr, next, &prtd->in_queue) {
				buf_node = list_entry(ptr,
						struct voip_buf_node, list);
				list_del(&buf_node->list);
			}
			list_for_each_safe(ptr, next, &prtd->free_in_queue) {
				buf_node = list_entry(ptr,
						struct voip_buf_node, list);
				list_del(&buf_node->list);
			}
			spin_unlock_irqrestore(&prtd->dsp_lock, dsp_flags);
			dma_free_coherent(p_substream->pcm->card->dev,
				runtime->hw.buffer_bytes_max, p_dma_buf->area,
				p_dma_buf->addr);
			p_dma_buf->area = NULL;
		}
		/* release out_queue and free_out_queue */
capt:		c_substream = prtd->capture_substream;
		if (c_substream == NULL) {
			pr_debug("c_substream is NULL\n");
			goto done;
		}
		c_dma_buf = &c_substream->dma_buffer;
		if (c_substream == NULL) {
			pr_debug("c_dma_buf is NULL.\n");
			goto done;
		}
		if (c_dma_buf->area != NULL) {
			spin_lock_irqsave(&prtd->dsp_ul_lock, dsp_flags);
			list_for_each_safe(ptr, next, &prtd->out_queue) {
				buf_node = list_entry(ptr,
						struct voip_buf_node, list);
				list_del(&buf_node->list);
			}
			list_for_each_safe(ptr, next, &prtd->free_out_queue) {
				buf_node = list_entry(ptr,
						struct voip_buf_node, list);
				list_del(&buf_node->list);
			}
			spin_unlock_irqrestore(&prtd->dsp_ul_lock, dsp_flags);
			dma_free_coherent(c_substream->pcm->card->dev,
				runtime->hw.buffer_bytes_max, c_dma_buf->area,
				c_dma_buf->addr);
			c_dma_buf->area = NULL;
		}
done:
		prtd->capture_substream = NULL;
		prtd->playback_substream = NULL;
	}
	mutex_unlock(&prtd->lock);

	return ret;
}
static int msm_pcm_prepare(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct voip_drv_info *prtd = runtime->private_data;
	uint32_t media_type = 0;
	uint32_t rate_type = 0;

	mutex_lock(&prtd->lock);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = msm_pcm_playback_prepare(substream);
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		ret = msm_pcm_capture_prepare(substream);

	if ((runtime->format != FORMAT_SPECIAL) &&
		 ((prtd->mode == MODE_AMR) || (prtd->mode == MODE_AMR_WB) ||
		 (prtd->mode == MODE_IS127) || (prtd->mode == MODE_4GV_NB) ||
		 (prtd->mode == MODE_4GV_WB) || (prtd->mode == MODE_G711) ||
		 (prtd->mode == MODE_G711A))) {
		pr_err("mode:%d and format:%u are not mached\n",
			prtd->mode, (uint32_t)runtime->format);
		ret =  -EINVAL;
		goto done;
	}

	if ((runtime->format != FORMAT_S16_LE) &&
		(prtd->mode == MODE_PCM)) {
		pr_err("mode:%d and format:%u are not mached\n",
			prtd->mode, (uint32_t)runtime->format);
		ret = -EINVAL;
		goto done;
	}

	if (prtd->playback_instance && prtd->capture_instance
				&& (prtd->state != VOIP_STARTED)) {

		ret = voip_get_rate_type(prtd->mode,
					prtd->rate,
					&rate_type);
		if (ret < 0) {
			pr_err("fail at getting rate_type\n");
			ret = -EINVAL;
			goto done;
		}
		prtd->rate_type = rate_type;
		ret = voip_get_media_type(prtd->mode,
					  prtd->rate_type,
					  prtd->play_samp_rate,
					  &media_type);
		if (ret < 0) {
			pr_err("fail at getting media_type\n");
			goto done;
		}
		pr_debug(" media_type=%d, rate_type=%d\n", media_type,
			rate_type);
		if ((prtd->play_samp_rate == 8000) &&
					(prtd->cap_samp_rate == 8000))
			voc_config_vocoder(media_type, rate_type,
					VSS_NETWORK_ID_VOIP_NB,
					voip_info.dtx_mode);
		else if ((prtd->play_samp_rate == 16000) &&
					(prtd->cap_samp_rate == 16000))
			voc_config_vocoder(media_type, rate_type,
					VSS_NETWORK_ID_VOIP_WB,
					voip_info.dtx_mode);
		else {
			pr_debug("%s: Invalid rate playback %d, capture %d\n",
				 __func__, prtd->play_samp_rate,
				 prtd->cap_samp_rate);
			goto done;
		}
		voc_register_mvs_cb(voip_process_ul_pkt,
					voip_process_dl_pkt, prtd);
		voc_start_voice_call(voc_get_session_id(VOIP_SESSION_NAME));

		prtd->state = VOIP_STARTED;
	}
done:
	mutex_unlock(&prtd->lock);

	return ret;
}

static snd_pcm_uframes_t
msm_pcm_playback_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct voip_drv_info *prtd = runtime->private_data;

	pr_debug("%s\n", __func__);
	if (prtd->pcm_playback_irq_pos >= prtd->pcm_size)
		prtd->pcm_playback_irq_pos = 0;
	return bytes_to_frames(runtime, (prtd->pcm_playback_irq_pos));
}

static snd_pcm_uframes_t
msm_pcm_capture_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct voip_drv_info *prtd = runtime->private_data;

	if (prtd->pcm_capture_irq_pos >= prtd->pcm_capture_size)
		prtd->pcm_capture_irq_pos = 0;
	return bytes_to_frames(runtime, (prtd->pcm_capture_irq_pos));
}

static snd_pcm_uframes_t msm_pcm_pointer(struct snd_pcm_substream *substream)
{
	snd_pcm_uframes_t ret = 0;
	 pr_debug("%s\n", __func__);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = msm_pcm_playback_pointer(substream);
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		ret = msm_pcm_capture_pointer(substream);
	return ret;
}

static int msm_pcm_mmap(struct snd_pcm_substream *substream,
			struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	pr_debug("%s\n", __func__);
	dma_mmap_coherent(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr,
				     runtime->dma_bytes);
	return 0;
}

static int msm_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_dma_buffer *dma_buf = &substream->dma_buffer;
	struct voip_buf_node *buf_node = NULL;
	int i = 0, offset = 0;

	pr_debug("%s: voip\n", __func__);

	mutex_lock(&voip_info.lock);

	dma_buf->dev.type = SNDRV_DMA_TYPE_DEV;
	dma_buf->dev.dev = substream->pcm->card->dev;
	dma_buf->private_data = NULL;

	dma_buf->area = dma_alloc_coherent(substream->pcm->card->dev,
			runtime->hw.buffer_bytes_max,
			&dma_buf->addr, GFP_KERNEL);
	if (!dma_buf->area) {
		pr_err("%s:MSM VOIP dma_alloc failed\n", __func__);
		return -ENOMEM;
	}

	dma_buf->bytes = runtime->hw.buffer_bytes_max;
	memset(dma_buf->area, 0, runtime->hw.buffer_bytes_max);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		for (i = 0; i < VOIP_MAX_Q_LEN; i++) {
			buf_node = (void *)dma_buf->area + offset;

			list_add_tail(&buf_node->list,
					&voip_info.free_in_queue);
			offset = offset + sizeof(struct voip_buf_node);
		}
	} else {
		for (i = 0; i < VOIP_MAX_Q_LEN; i++) {
			buf_node = (void *) dma_buf->area + offset;
			list_add_tail(&buf_node->list,
					&voip_info.free_out_queue);
			offset = offset + sizeof(struct voip_buf_node);
		}
	}

	mutex_unlock(&voip_info.lock);

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

	return 0;
}

static int msm_voip_mode_rate_config_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	mutex_lock(&voip_info.lock);

	ucontrol->value.integer.value[0] = voip_info.mode;
	ucontrol->value.integer.value[1] = voip_info.rate;

	mutex_unlock(&voip_info.lock);

	return 0;
}

static int msm_voip_mode_rate_config_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	mutex_lock(&voip_info.lock);

	voip_info.mode = ucontrol->value.integer.value[0];
	voip_info.rate = ucontrol->value.integer.value[1];

	pr_debug("%s: mode=%d,rate=%d\n", __func__, voip_info.mode,
		voip_info.rate);

	mutex_unlock(&voip_info.lock);

	return 0;
}

static int voip_get_rate_type(uint32_t mode, uint32_t rate,
				 uint32_t *rate_type)
{
	int ret = 0;

	switch (mode) {
	case MODE_AMR: {
		switch (rate) {
		case 4750:
			*rate_type = AMR_RATE_4750;
			break;
		case 5150:
			*rate_type = AMR_RATE_5150;
			break;
		case 5900:
			*rate_type = AMR_RATE_5900;
			break;
		case 6700:
			*rate_type = AMR_RATE_6700;
			break;
		case 7400:
			*rate_type = AMR_RATE_7400;
			break;
		case 7950:
			*rate_type = AMR_RATE_7950;
			break;
		case 10200:
			*rate_type = AMR_RATE_10200;
			break;
		case 12200:
			*rate_type = AMR_RATE_12200;
			break;
		default:
			pr_err("wrong rate for AMR NB.\n");
			ret = -EINVAL;
			break;
		}
		break;
	}
	case MODE_AMR_WB: {
		switch (rate) {
		case 6600:
			*rate_type = AMR_RATE_6600 - AMR_RATE_6600;
			break;
		case 8850:
			*rate_type = AMR_RATE_8850 - AMR_RATE_6600;
			break;
		case 12650:
			*rate_type = AMR_RATE_12650 - AMR_RATE_6600;
			break;
		case 14250:
			*rate_type = AMR_RATE_14250 - AMR_RATE_6600;
			break;
		case 15850:
			*rate_type = AMR_RATE_15850 - AMR_RATE_6600;
			break;
		case 18250:
			*rate_type = AMR_RATE_18250 - AMR_RATE_6600;
			break;
		case 19850:
			*rate_type = AMR_RATE_19850 - AMR_RATE_6600;
			break;
		case 23050:
			*rate_type = AMR_RATE_23050 - AMR_RATE_6600;
			break;
		case 23850:
			*rate_type = AMR_RATE_23850 - AMR_RATE_6600;
			break;
		default:
			pr_err("wrong rate for AMR_WB.\n");
			ret = -EINVAL;
			break;
		}
		break;
	}
	case MODE_PCM: {
		*rate_type = 0;
		break;
	}
	case MODE_IS127:
	case MODE_4GV_NB:
	case MODE_4GV_WB: {
		switch (rate) {
		case VOC_0_RATE:
		case VOC_8_RATE:
		case VOC_4_RATE:
		case VOC_2_RATE:
		case VOC_1_RATE:
			*rate_type = rate;
			break;
		default:
			pr_err("wrong rate for IS127/4GV_NB/WB.\n");
			ret = -EINVAL;
			break;
		}
		break;
	}
	case MODE_G711:
	case MODE_G711A:
		*rate_type = rate;
		break;
	default:
		pr_err("wrong mode type.\n");
		ret = -EINVAL;
	}
	pr_debug("%s, mode=%d, rate=%u, rate_type=%d\n",
		__func__, mode, rate, *rate_type);
	return ret;
}

static int voip_get_media_type(uint32_t mode, uint32_t rate_type,
				unsigned int samp_rate,
				uint32_t *media_type)
{
	int ret = 0;

	pr_debug("%s: mode=%d, samp_rate=%d\n", __func__,
		mode, samp_rate);
	switch (mode) {
	case MODE_AMR:
		*media_type = VSS_MEDIA_ID_AMR_NB_MODEM;
		break;
	case MODE_AMR_WB:
		*media_type = VSS_MEDIA_ID_AMR_WB_MODEM;
		break;
	case MODE_PCM:
		if (samp_rate == 8000)
			*media_type = VSS_MEDIA_ID_PCM_NB;
		else
			*media_type = VSS_MEDIA_ID_PCM_WB;
		break;
	case MODE_IS127: /* EVRC-A */
		*media_type = VSS_MEDIA_ID_EVRC_MODEM;
		break;
	case MODE_4GV_NB: /* EVRC-B */
		*media_type = VSS_MEDIA_ID_4GV_NB_MODEM;
		break;
	case MODE_4GV_WB: /* EVRC-WB */
		*media_type = VSS_MEDIA_ID_4GV_WB_MODEM;
		break;
	case MODE_G711:
	case MODE_G711A:
		if (rate_type == MVS_G711A_MODE_MULAW)
			*media_type = VSS_MEDIA_ID_G711_MULAW;
		else
			*media_type = VSS_MEDIA_ID_G711_ALAW;
		break;
	default:
		pr_debug(" input mode is not supported\n");
		ret = -EINVAL;
	}

	pr_debug("%s: media_type is 0x%x\n", __func__, *media_type);

	return ret;
}


static struct snd_pcm_ops msm_pcm_ops = {
	.open           = msm_pcm_open,
	.copy		= msm_pcm_copy,
	.hw_params	= msm_pcm_hw_params,
	.close          = msm_pcm_close,
	.prepare        = msm_pcm_prepare,
	.trigger        = msm_pcm_trigger,
	.pointer        = msm_pcm_pointer,
	.mmap		= msm_pcm_mmap,
};

static int msm_asoc_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	int ret = 0;

	pr_debug("msm_asoc_pcm_new\n");
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);
	return ret;
}

static struct snd_soc_platform_driver msm_soc_platform = {
	.ops		= &msm_pcm_ops,
	.pcm_new	= msm_asoc_pcm_new,
	.probe		= msm_pcm_voip_probe,
};

static __devinit int msm_pcm_probe(struct platform_device *pdev)
{
	pr_info("%s: dev name %s\n", __func__, dev_name(&pdev->dev));
	return snd_soc_register_platform(&pdev->dev,
				   &msm_soc_platform);
}

static int msm_pcm_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_driver msm_pcm_driver = {
	.driver = {
		.name = "msm-voip-dsp",
		.owner = THIS_MODULE,
	},
	.probe = msm_pcm_probe,
	.remove = __devexit_p(msm_pcm_remove),
};

static int __init msm_soc_platform_init(void)
{
	memset(&voip_info, 0, sizeof(voip_info));
	voip_info.mode = MODE_PCM;
	mutex_init(&voip_info.lock);

	spin_lock_init(&voip_info.dsp_lock);
	spin_lock_init(&voip_info.dsp_ul_lock);

	init_waitqueue_head(&voip_info.out_wait);
	init_waitqueue_head(&voip_info.in_wait);

	INIT_LIST_HEAD(&voip_info.in_queue);
	INIT_LIST_HEAD(&voip_info.free_in_queue);
	INIT_LIST_HEAD(&voip_info.out_queue);
	INIT_LIST_HEAD(&voip_info.free_out_queue);

	return platform_driver_register(&msm_pcm_driver);
}
module_init(msm_soc_platform_init);

static void __exit msm_soc_platform_exit(void)
{
	platform_driver_unregister(&msm_pcm_driver);
}
module_exit(msm_soc_platform_exit);

MODULE_DESCRIPTION("PCM module platform driver");
MODULE_LICENSE("GPL v2");
