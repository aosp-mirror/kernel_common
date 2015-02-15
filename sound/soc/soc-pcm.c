/*
 * soc-pcm.c  --  ALSA SoC PCM
 *
 * Copyright 2005 Wolfson Microelectronics PLC.
 * Copyright 2005 Openedhand Ltd.
 * Copyright (C) 2010 Slimlogic Ltd.
 * Copyright (C) 2010 Texas Instruments Inc.
 *
 * Authors: Liam Girdwood <lrg@ti.com>
 *          Mark Brown <broonie@opensource.wolfsonmicro.com>       
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/export.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dpcm.h>
#include <sound/initval.h>

#undef pr_info
#undef pr_err
#define pr_info(fmt, ...) pr_aud_info(fmt, ##__VA_ARGS__)
#define pr_err(fmt, ...) pr_aud_err(fmt, ##__VA_ARGS__)

#define MAX_BE_USERS	8	

static const struct snd_pcm_hardware no_host_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min	= PAGE_SIZE >> 2,
	.period_bytes_max	= PAGE_SIZE >> 1,
	.periods_min		= 2,
	.periods_max		= 4,
	.buffer_bytes_max	= PAGE_SIZE,
};

int snd_soc_dpcm_can_be_free_stop(struct snd_soc_pcm_runtime *fe,
		struct snd_soc_pcm_runtime *be, int stream)
{
	struct snd_soc_dpcm_params *dpcm_params;

	list_for_each_entry(dpcm_params, &be->dpcm[stream].fe_clients, list_fe) {

		if (dpcm_params->fe == fe)
			continue;

		if (dpcm_params->fe->dpcm[stream].state == SND_SOC_DPCM_STATE_START ||
			dpcm_params->fe->dpcm[stream].state == SND_SOC_DPCM_STATE_PAUSED ||
			dpcm_params->fe->dpcm[stream].state == SND_SOC_DPCM_STATE_SUSPEND)
			return 0;
	}
	return 1;
}
EXPORT_SYMBOL_GPL(snd_soc_dpcm_can_be_free_stop);

static void dump_fe_state_by_be(struct snd_soc_pcm_runtime *fe,
		struct snd_soc_pcm_runtime *be, int stream)
{
	struct snd_soc_dpcm_params *dpcm_params;

	pr_info("%s: update be %s fail state %d\n",__func__,be->dai_link->name,be->dpcm[stream].state);
	pr_info("%s: current fe %s state %d\n",__func__,fe->dai_link->stream_name,fe->dpcm[stream].state);
	list_for_each_entry(dpcm_params, &be->dpcm[stream].fe_clients, list_fe) {

		if (dpcm_params->fe == fe)
			continue;

		pr_info("%s: connected fe %s state %d\n",__func__,dpcm_params->fe->dai_link->stream_name, \
			dpcm_params->fe->dpcm[stream].state);
	}
}
static int snd_soc_dpcm_can_be_params(struct snd_soc_pcm_runtime *fe,
		struct snd_soc_pcm_runtime *be, int stream)
{
	struct snd_soc_dpcm_params *dpcm_params;

	list_for_each_entry(dpcm_params, &be->dpcm[stream].fe_clients, list_fe) {

		if (dpcm_params->fe == fe)
			continue;

		if (dpcm_params->fe->dpcm[stream].state == SND_SOC_DPCM_STATE_START ||
			dpcm_params->fe->dpcm[stream].state == SND_SOC_DPCM_STATE_PAUSED ||
			dpcm_params->fe->dpcm[stream].state == SND_SOC_DPCM_STATE_SUSPEND ||
			dpcm_params->fe->dpcm[stream].state == SND_SOC_DPCM_STATE_PREPARE)
			return 0;
	}
	return 1;
}

static int soc_pcm_apply_symmetry(struct snd_pcm_substream *substream,
					struct snd_soc_dai *soc_dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int ret;

	if (!soc_dai->driver->symmetric_rates &&
	    !rtd->dai_link->symmetric_rates)
		return 0;

	if (!soc_dai->rate) {
		dev_warn(soc_dai->dev,
			 "Not enforcing symmetric_rates due to race\n");
		return 0;
	}

	dev_dbg(soc_dai->dev, "Symmetry forces %dHz rate\n", soc_dai->rate);

	ret = snd_pcm_hw_constraint_minmax(substream->runtime,
					   SNDRV_PCM_HW_PARAM_RATE,
					   soc_dai->rate, soc_dai->rate);
	if (ret < 0) {
		dev_err(soc_dai->dev,
			"Unable to apply rate symmetry constraint: %d\n", ret);
		return ret;
	}

	return 0;
}

static int sample_sizes[] = {
	8, 16, 24, 32,
};

static void soc_pcm_apply_msb(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
	int ret, i, bits;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		bits = dai->driver->playback.sig_bits;
	else
		bits = dai->driver->capture.sig_bits;

	if (!bits)
		return;

	for (i = 0; i < ARRAY_SIZE(sample_sizes); i++) {
		if (bits >= sample_sizes[i])
			continue;

		ret = snd_pcm_hw_constraint_msbits(substream->runtime, 0,
						   sample_sizes[i], bits);
		if (ret != 0)
			dev_warn(dai->dev,
				 "Failed to set MSB %d/%d: %d\n",
				 bits, sample_sizes[i], ret);
	}
}

int dpcm_dapm_stream_event(struct snd_soc_pcm_runtime *fe,
	int dir, const char *stream, int event)
{
	struct snd_soc_dpcm_params *dpcm_params;

	snd_soc_dapm_rtd_stream_event(fe, dir, event);

	list_for_each_entry(dpcm_params, &fe->dpcm[dir].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;

		dev_dbg(be->dev, "pm: BE %s stream %s event %d dir %d\n",
				be->dai_link->name, stream, event, dir);

		snd_soc_dapm_rtd_stream_event(be, dir, event);
	}

	return 0;
}

static int soc_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_platform *platform = rtd->platform;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai_driver *cpu_dai_drv = cpu_dai->driver;
	struct snd_soc_dai_driver *codec_dai_drv = codec_dai->driver;
	int ret = 0;

	pm_runtime_get_sync(cpu_dai->dev);
	pm_runtime_get_sync(codec_dai->dev);
	pm_runtime_get_sync(platform->dev);

	mutex_lock_nested(&rtd->pcm_mutex, rtd->pcm_subclass);

	if (rtd->dai_link->no_host_mode == SND_SOC_DAI_LINK_NO_HOST)
		snd_soc_set_runtime_hwparams(substream, &no_host_hardware);

	
	if (cpu_dai->driver->ops->startup) {
		ret = cpu_dai->driver->ops->startup(substream, cpu_dai);
		if (ret < 0) {
			printk(KERN_ERR "asoc: can't open interface %s\n",
				cpu_dai->name);
			goto out;
		}
	}

	if (platform->driver->ops && platform->driver->ops->open) {
		ret = platform->driver->ops->open(substream);
		if (ret < 0) {
			printk(KERN_ERR "asoc: can't open platform %s\n", platform->name);
			goto platform_err;
		}
	}

	if (codec_dai->driver->ops->startup) {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			ret = codec_dai->driver->ops->startup(substream,
								codec_dai);
			if (ret < 0) {
				printk(KERN_ERR "asoc: can't open codec %s\n",
					codec_dai->name);
				goto codec_dai_err;
			}
		} else {
			if (!codec_dai->capture_active) {
				ret = codec_dai->driver->ops->startup(substream,
								codec_dai);
				if (ret < 0) {
					printk(KERN_ERR "can't open codec %s\n",
						codec_dai->name);
					goto codec_dai_err;
				}
			}
		}
	}

	if (rtd->dai_link->ops && rtd->dai_link->ops->startup) {
		ret = rtd->dai_link->ops->startup(substream);
		if (ret < 0) {
			printk(KERN_ERR "asoc: %s startup failed\n", rtd->dai_link->name);
			goto machine_err;
		}
	}

	
	if (rtd->dai_link->dynamic || rtd->dai_link->no_pcm)
		goto dynamic;

	
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		runtime->hw.rate_min =
			max(codec_dai_drv->playback.rate_min,
			    cpu_dai_drv->playback.rate_min);
		runtime->hw.rate_max =
			min(codec_dai_drv->playback.rate_max,
			    cpu_dai_drv->playback.rate_max);
		runtime->hw.channels_min =
			max(codec_dai_drv->playback.channels_min,
				cpu_dai_drv->playback.channels_min);
		runtime->hw.channels_max =
			min(codec_dai_drv->playback.channels_max,
				cpu_dai_drv->playback.channels_max);
		runtime->hw.formats =
			codec_dai_drv->playback.formats & cpu_dai_drv->playback.formats;
		runtime->hw.rates =
			codec_dai_drv->playback.rates & cpu_dai_drv->playback.rates;
		if (codec_dai_drv->playback.rates
			   & (SNDRV_PCM_RATE_KNOT | SNDRV_PCM_RATE_CONTINUOUS))
			runtime->hw.rates |= cpu_dai_drv->playback.rates;
		if (cpu_dai_drv->playback.rates
			   & (SNDRV_PCM_RATE_KNOT | SNDRV_PCM_RATE_CONTINUOUS))
			runtime->hw.rates |= codec_dai_drv->playback.rates;
	} else {
		runtime->hw.rate_min =
			max(codec_dai_drv->capture.rate_min,
			    cpu_dai_drv->capture.rate_min);
		runtime->hw.rate_max =
			min(codec_dai_drv->capture.rate_max,
			    cpu_dai_drv->capture.rate_max);
		runtime->hw.channels_min =
			max(codec_dai_drv->capture.channels_min,
				cpu_dai_drv->capture.channels_min);
		runtime->hw.channels_max =
			min(codec_dai_drv->capture.channels_max,
				cpu_dai_drv->capture.channels_max);
		runtime->hw.formats =
			codec_dai_drv->capture.formats & cpu_dai_drv->capture.formats;
		runtime->hw.rates =
			codec_dai_drv->capture.rates & cpu_dai_drv->capture.rates;
		if (codec_dai_drv->capture.rates
			   & (SNDRV_PCM_RATE_KNOT | SNDRV_PCM_RATE_CONTINUOUS))
			runtime->hw.rates |= cpu_dai_drv->capture.rates;
		if (cpu_dai_drv->capture.rates
			   & (SNDRV_PCM_RATE_KNOT | SNDRV_PCM_RATE_CONTINUOUS))
			runtime->hw.rates |= codec_dai_drv->capture.rates;
	}

	ret = -EINVAL;
	snd_pcm_limit_hw_rates(runtime);
	if (!runtime->hw.rates) {
		printk(KERN_ERR "asoc: %s <-> %s No matching rates\n",
			codec_dai->name, cpu_dai->name);
		goto config_err;
	}
	if (!runtime->hw.formats) {
		printk(KERN_ERR "asoc: %s <-> %s No matching formats\n",
			codec_dai->name, cpu_dai->name);
		goto config_err;
	}
	if (!runtime->hw.channels_min || !runtime->hw.channels_max ||
	    runtime->hw.channels_min > runtime->hw.channels_max) {
		printk(KERN_ERR "asoc: %s <-> %s No matching channels\n",
				codec_dai->name, cpu_dai->name);
		goto config_err;
	}

	soc_pcm_apply_msb(substream, codec_dai);
	soc_pcm_apply_msb(substream, cpu_dai);

	
	if (cpu_dai->active) {
		ret = soc_pcm_apply_symmetry(substream, cpu_dai);
		if (ret != 0)
			goto config_err;
	}

	if (codec_dai->active) {
		ret = soc_pcm_apply_symmetry(substream, codec_dai);
		if (ret != 0)
			goto config_err;
	}

	pr_debug("asoc: %s <-> %s info:\n",
			codec_dai->name, cpu_dai->name);
	pr_debug("asoc: rate mask 0x%x\n", runtime->hw.rates);
	pr_debug("asoc: min ch %d max ch %d\n", runtime->hw.channels_min,
		 runtime->hw.channels_max);
	pr_debug("asoc: min rate %d max rate %d\n", runtime->hw.rate_min,
		 runtime->hw.rate_max);

dynamic:
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		cpu_dai->playback_active++;
		codec_dai->playback_active++;
	} else {
		cpu_dai->capture_active++;
		codec_dai->capture_active++;
	}
	cpu_dai->active++;
	codec_dai->active++;
	rtd->codec->active++;
	mutex_unlock(&rtd->pcm_mutex);
	return 0;

config_err:
	if (rtd->dai_link->ops && rtd->dai_link->ops->shutdown)
		rtd->dai_link->ops->shutdown(substream);

machine_err:
	if (codec_dai->driver->ops->shutdown)
		codec_dai->driver->ops->shutdown(substream, codec_dai);

codec_dai_err:
	if (platform->driver->ops && platform->driver->ops->close)
		platform->driver->ops->close(substream);

platform_err:
	if (cpu_dai->driver->ops->shutdown)
		cpu_dai->driver->ops->shutdown(substream, cpu_dai);
out:
	mutex_unlock(&rtd->pcm_mutex);

	pm_runtime_put(platform->dev);
	pm_runtime_put(codec_dai->dev);
	pm_runtime_put(cpu_dai->dev);

	return ret;
}

static void close_delayed_work(struct work_struct *work)
{
	struct snd_soc_pcm_runtime *rtd =
			container_of(work, struct snd_soc_pcm_runtime, delayed_work.work);
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	mutex_lock_nested(&rtd->pcm_mutex, rtd->pcm_subclass);

	pr_debug("pop wq checking: %s status: %s waiting: %s\n",
		 codec_dai->driver->playback.stream_name,
		 codec_dai->playback_active ? "active" : "inactive",
		 codec_dai->pop_wait ? "yes" : "no");

	
	if (codec_dai->pop_wait == 1) {
		codec_dai->pop_wait = 0;
		snd_soc_dapm_stream_event(rtd,
			codec_dai->driver->playback.stream_name,
			SND_SOC_DAPM_STREAM_STOP);
	}

	mutex_unlock(&rtd->pcm_mutex);
}

static int soc_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_platform *platform = rtd->platform;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = rtd->codec;

	mutex_lock_nested(&rtd->pcm_mutex, rtd->pcm_subclass);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		cpu_dai->playback_active--;
		codec_dai->playback_active--;
	} else {
		cpu_dai->capture_active--;
		codec_dai->capture_active--;
	}

	cpu_dai->active--;
	codec_dai->active--;
	codec->active--;

	
	if (!cpu_dai->active)
		cpu_dai->rate = 0;

	if (!codec_dai->active)
		codec_dai->rate = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		snd_soc_dai_digital_mute(codec_dai, 1);

	if (cpu_dai->driver->ops->shutdown)
		cpu_dai->driver->ops->shutdown(substream, cpu_dai);

	if (codec_dai->driver->ops->shutdown) {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			codec_dai->driver->ops->shutdown(substream, codec_dai);
		} else {
			if (!codec_dai->capture_active)
				codec_dai->driver->ops->shutdown(substream,
								codec_dai);
		}
	}

	if (rtd->dai_link->ops && rtd->dai_link->ops->shutdown)
		rtd->dai_link->ops->shutdown(substream);

	if (platform->driver->ops && platform->driver->ops->close)
		platform->driver->ops->close(substream);
	cpu_dai->runtime = NULL;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (codec->ignore_pmdown_time ||
		    rtd->dai_link->ignore_pmdown_time ||
		    !rtd->pmdown_time) {
			
			snd_soc_dapm_stream_event(rtd,
				codec_dai->driver->playback.stream_name,
				SND_SOC_DAPM_STREAM_STOP);
		} else {
			
			codec_dai->pop_wait = 1;
			schedule_delayed_work(&rtd->delayed_work,
				msecs_to_jiffies(rtd->pmdown_time));
		}
	} else {
		
		if (!codec_dai->capture_active)
			snd_soc_dapm_stream_event(rtd,
			codec_dai->driver->capture.stream_name,
			SND_SOC_DAPM_STREAM_STOP);
	}

	mutex_unlock(&rtd->pcm_mutex);

	pm_runtime_put(platform->dev);
	pm_runtime_put(codec_dai->dev);
	pm_runtime_put(cpu_dai->dev);

	return 0;
}

static int soc_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_platform *platform = rtd->platform;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret = 0;

	mutex_lock_nested(&rtd->pcm_mutex, rtd->pcm_subclass);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		snd_soc_dapm_stream_event(rtd,
		codec_dai->driver->playback.stream_name,
		SND_SOC_DAPM_STREAM_START);

	if (rtd->dai_link->ops && rtd->dai_link->ops->prepare) {
		ret = rtd->dai_link->ops->prepare(substream);
		if (ret < 0) {
			printk(KERN_ERR "asoc: machine prepare error\n");
			goto out;
		}
	}

	if (platform->driver->ops && platform->driver->ops->prepare) {
		ret = platform->driver->ops->prepare(substream);
		if (ret < 0) {
			printk(KERN_ERR "asoc: platform prepare error\n");
			goto out;
		}
	}

	if (codec_dai->driver->ops->prepare) {
		ret = codec_dai->driver->ops->prepare(substream, codec_dai);
		if (ret < 0) {
			printk(KERN_ERR "asoc: codec DAI prepare error\n");
			goto out;
		}
	}

	if (cpu_dai->driver->ops->prepare) {
		ret = cpu_dai->driver->ops->prepare(substream, cpu_dai);
		if (ret < 0) {
			printk(KERN_ERR "asoc: cpu DAI prepare error\n");
			goto out;
		}
	}

	
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK &&
	    codec_dai->pop_wait) {
		codec_dai->pop_wait = 0;
		cancel_delayed_work(&rtd->delayed_work);
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		if (codec_dai->capture_active == 1)
			snd_soc_dapm_stream_event(rtd,
			codec_dai->driver->capture.stream_name,
			SND_SOC_DAPM_STREAM_START);
	}
	snd_soc_dai_digital_mute(codec_dai, 0);

out:
	if (ret < 0 && substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		pr_err("%s: Issue stop stream for codec_dai due to op failure %d = ret\n",
		__func__, ret);
		snd_soc_dapm_stream_event(rtd,
		codec_dai->driver->playback.stream_name,
		SND_SOC_DAPM_STREAM_STOP);
	}
	mutex_unlock(&rtd->pcm_mutex);
	return ret;
}

static int soc_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_platform *platform = rtd->platform;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret = 0;

	mutex_lock_nested(&rtd->pcm_mutex, rtd->pcm_subclass);

	if (rtd->dai_link->ops && rtd->dai_link->ops->hw_params) {
		ret = rtd->dai_link->ops->hw_params(substream, params);
		if (ret < 0) {
			printk(KERN_ERR "asoc: machine hw_params failed\n");
			goto out;
		}
	}

	if (codec_dai->driver->ops->hw_params) {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			ret = codec_dai->driver->ops->hw_params(substream,
							params, codec_dai);
			if (ret < 0) {
				printk(KERN_ERR "not set codec %s hw params\n",
					codec_dai->name);
				goto codec_err;
			}
		} else {
			if (codec_dai->capture_active == 1) {
				ret = codec_dai->driver->ops->hw_params(
						substream, params, codec_dai);
				if (ret < 0) {
					printk(KERN_ERR "fail: %s hw params\n",
						codec_dai->name);
					goto codec_err;
				}
			}
		}
	}

	if (cpu_dai->driver->ops->hw_params) {
		ret = cpu_dai->driver->ops->hw_params(substream, params, cpu_dai);
		if (ret < 0) {
			printk(KERN_ERR "asoc: interface %s hw params failed\n",
				cpu_dai->name);
			goto interface_err;
		}
	}

	if (platform->driver->ops && platform->driver->ops->hw_params) {
		ret = platform->driver->ops->hw_params(substream, params);
		if (ret < 0) {
			printk(KERN_ERR "asoc: platform %s hw params failed\n",
				platform->name);
			goto platform_err;
		}
	}

	
	cpu_dai->rate = params_rate(params);
	codec_dai->rate = params_rate(params);

	if (rtd->dai_link->no_host_mode == SND_SOC_DAI_LINK_NO_HOST) {
		substream->dma_buffer.dev.type = SNDRV_DMA_TYPE_DEV;
		substream->dma_buffer.dev.dev = rtd->dev;
		substream->dma_buffer.dev.dev->coherent_dma_mask = DMA_BIT_MASK(32);
		substream->dma_buffer.private_data = NULL;

		ret = snd_pcm_lib_malloc_pages(substream, PAGE_SIZE);
		if (ret < 0)
			goto platform_err;
	}

out:
	mutex_unlock(&rtd->pcm_mutex);
	return ret;

platform_err:
	if (cpu_dai->driver->ops->hw_free)
		cpu_dai->driver->ops->hw_free(substream, cpu_dai);

interface_err:
	if (codec_dai->driver->ops->hw_free)
		codec_dai->driver->ops->hw_free(substream, codec_dai);

codec_err:
	if (rtd->dai_link->ops && rtd->dai_link->ops->hw_free)
		rtd->dai_link->ops->hw_free(substream);

	mutex_unlock(&rtd->pcm_mutex);
	return ret;
}

static int soc_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_platform *platform = rtd->platform;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = rtd->codec;

	mutex_lock_nested(&rtd->pcm_mutex, rtd->pcm_subclass);

	
	if (!codec->active)
		snd_soc_dai_digital_mute(codec_dai, 1);

	
	if (rtd->dai_link->ops && rtd->dai_link->ops->hw_free)
		rtd->dai_link->ops->hw_free(substream);

	
	if (platform->driver->ops && platform->driver->ops->hw_free)
		platform->driver->ops->hw_free(substream);

	
	if (codec_dai->driver->ops->hw_free)
		codec_dai->driver->ops->hw_free(substream, codec_dai);

	if (cpu_dai->driver->ops->hw_free)
		cpu_dai->driver->ops->hw_free(substream, cpu_dai);

	if (rtd->dai_link->no_host_mode == SND_SOC_DAI_LINK_NO_HOST)
		snd_pcm_lib_free_pages(substream);

	mutex_unlock(&rtd->pcm_mutex);
	return 0;
}

static int soc_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_platform *platform = rtd->platform;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	if (codec_dai->driver->ops->trigger) {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			ret = codec_dai->driver->ops->trigger(substream,
						cmd, codec_dai);
			if (ret < 0)
				return ret;
		} else {
			if (codec_dai->capture_active == 1) {
				ret = codec_dai->driver->ops->trigger(
						substream, cmd, codec_dai);
				if (ret < 0)
					return ret;
			}
		}
	}

	if (platform->driver->ops && platform->driver->ops->trigger) {
		ret = platform->driver->ops->trigger(substream, cmd);
		if (ret < 0)
			return ret;
	}

	if (cpu_dai->driver->ops->trigger) {
		ret = cpu_dai->driver->ops->trigger(substream, cmd, cpu_dai);
		if (ret < 0)
			return ret;
	}
	return 0;
}

int soc_pcm_bespoke_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_platform *platform = rtd->platform;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	if (codec_dai->driver->ops->bespoke_trigger) {
		ret = codec_dai->driver->ops->bespoke_trigger(substream, cmd, codec_dai);
		if (ret < 0)
			return ret;
	}

	if (platform->driver->bespoke_trigger) {
		ret = platform->driver->bespoke_trigger(substream, cmd);
		if (ret < 0)
			return ret;
	}

	if (cpu_dai->driver->ops->bespoke_trigger) {
		ret = cpu_dai->driver->ops->bespoke_trigger(substream, cmd, cpu_dai);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static snd_pcm_uframes_t soc_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_platform *platform = rtd->platform;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t offset = 0;
	snd_pcm_sframes_t delay = 0;

	if (platform->driver->ops && platform->driver->ops->pointer)
		offset = platform->driver->ops->pointer(substream);

	if (cpu_dai->driver->ops->delay)
		delay += cpu_dai->driver->ops->delay(substream, cpu_dai);

	if (codec_dai->driver->ops->delay)
		delay += codec_dai->driver->ops->delay(substream, codec_dai);

	if (platform->driver->delay)
		delay += platform->driver->delay(substream, codec_dai);

	runtime->delay = delay;

	return offset;
}

static inline int be_connect(struct snd_soc_pcm_runtime *fe,
		struct snd_soc_pcm_runtime *be, int stream)
{
	struct snd_soc_dpcm_params *dpcm_params;

	if (!fe->dpcm[stream].runtime && !fe->fe_compr) {
		dev_err(fe->dev, "%s no runtime\n", fe->dai_link->name);
		return -ENODEV;
	}

	
	list_for_each_entry(dpcm_params, &fe->dpcm[stream].be_clients, list_be) {
		if (dpcm_params->be == be && dpcm_params->fe == fe)
			return 0;
	}

	dpcm_params = kzalloc(sizeof(struct snd_soc_dpcm_params), GFP_KERNEL);
	if (!dpcm_params)
		return -ENOMEM;

	dpcm_params->be = be;
	dpcm_params->fe = fe;
	be->dpcm[stream].runtime = fe->dpcm[stream].runtime;
	dpcm_params->state = SND_SOC_DPCM_LINK_STATE_NEW;
	list_add(&dpcm_params->list_be, &fe->dpcm[stream].be_clients);
	list_add(&dpcm_params->list_fe, &be->dpcm[stream].fe_clients);

	dev_dbg(fe->dev, "  connected new DSP %s path %s %s %s\n",
			stream ? "capture" : "playback",  fe->dai_link->name,
			stream ? "<-" : "->", be->dai_link->name);

#ifdef CONFIG_DEBUG_FS
	dpcm_params->debugfs_state = debugfs_create_u32(be->dai_link->name, 0644,
			fe->debugfs_dpcm_root, &dpcm_params->state);
#endif

	return 1;
}

static inline void be_reparent(struct snd_soc_pcm_runtime *fe,
			struct snd_soc_pcm_runtime *be, int stream)
{
	struct snd_soc_dpcm_params *dpcm_params;
	struct snd_pcm_substream *fe_substream, *be_substream;

	
	if (!be->dpcm[stream].users)
		return;

	be_substream = snd_soc_dpcm_get_substream(be, stream);

	list_for_each_entry(dpcm_params, &be->dpcm[stream].fe_clients, list_fe) {
		if (dpcm_params->fe != fe) {

			dev_dbg(fe->dev, "  reparent %s path %s %s %s\n",
					stream ? "capture" : "playback",
					dpcm_params->fe->dai_link->name,
					stream ? "<-" : "->", dpcm_params->be->dai_link->name);

			fe_substream = snd_soc_dpcm_get_substream(dpcm_params->fe,
								stream);
			be_substream->runtime = fe_substream->runtime;
			break;
		}
	}
}

void dpcm_be_disconnect(struct snd_soc_pcm_runtime *fe, int stream)
{
	struct snd_soc_dpcm_params *dpcm_params, *d;

	list_for_each_entry_safe(dpcm_params, d, &fe->dpcm[stream].be_clients, list_be) {
		dev_dbg(fe->dev, "BE %s disconnect check for %s\n",
				stream ? "capture" : "playback",
				dpcm_params->be->dai_link->name);

		if (dpcm_params->state == SND_SOC_DPCM_LINK_STATE_FREE) {
			dev_dbg(fe->dev, "  freed DSP %s path %s %s %s\n",
					stream ? "capture" : "playback", fe->dai_link->name,
					stream ? "<-" : "->", dpcm_params->be->dai_link->name);

			
			be_reparent(fe, dpcm_params->be, stream);

#ifdef CONFIG_DEBUG_FS
			debugfs_remove(dpcm_params->debugfs_state);
#endif

			list_del(&dpcm_params->list_be);
			list_del(&dpcm_params->list_fe);
			kfree(dpcm_params);
		}
	}
}

static struct snd_soc_pcm_runtime *be_get_rtd(struct snd_soc_card *card,
		struct snd_soc_dapm_widget *widget)
{
	struct snd_soc_pcm_runtime *be;
	int i;

	if (!widget->sname) {
		dev_err(card->dev, "widget %s has no stream\n", widget->name);
		return NULL;
	}

	for (i = 0; i < card->num_links; i++) {
		be = &card->rtd[i];

		if (!strcmp(widget->sname, be->dai_link->stream_name))
			return be;
	}

	dev_err(card->dev, "can't get BE for %s\n", widget->name);
	return NULL;
}

static struct snd_soc_dapm_widget *be_get_widget(struct snd_soc_card *card,
		struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dapm_widget *widget;

	list_for_each_entry(widget, &card->widgets, list) {

		if (!widget->sname)
			continue;

		if (!strcmp(widget->sname, rtd->dai_link->stream_name))
			return widget;
	}

	dev_err(card->dev, "can't get widget for %s\n",
			rtd->dai_link->stream_name);
	return NULL;
}

static int widget_in_list(struct snd_soc_dapm_widget_list *list,
		struct snd_soc_dapm_widget *widget)
{
	int i;

	for (i = 0; i < list->num_widgets; i++) {
			if (widget == list->widgets[i])
				return 1;
	}

	return 0;
}

int dpcm_path_get(struct snd_soc_pcm_runtime *fe,
	int stream, struct snd_soc_dapm_widget_list **list_)
{
	struct snd_soc_dai *cpu_dai = fe->cpu_dai;
	struct snd_soc_dapm_widget_list *list;
	int paths;

	list = kzalloc(sizeof(struct snd_soc_dapm_widget_list) +
			sizeof(struct snd_soc_dapm_widget *), GFP_KERNEL);
	if (list == NULL)
		return -ENOMEM;

	
	paths = snd_soc_dapm_dai_get_connected_widgets(cpu_dai, stream, &list);

	dev_dbg(fe->dev, "found %d audio %s paths\n", paths,
			stream ? "capture" : "playback");

	*list_ = list;
	return paths;
}

static int be_prune_old(struct snd_soc_pcm_runtime *fe, int stream,
	struct snd_soc_dapm_widget_list **list_)
{
	struct snd_soc_card *card = fe->card;
	struct snd_soc_dpcm_params *dpcm_params;
	struct snd_soc_dapm_widget_list *list = *list_;
	struct snd_soc_dapm_widget *widget;
	int old = 0;

	
	list_for_each_entry(dpcm_params, &fe->dpcm[stream].be_clients, list_be) {

		
		widget = be_get_widget(card, dpcm_params->be);
		if (!widget) {
			dev_err(fe->dev, "no widget found for %s\n",
					dpcm_params->be->dai_link->name);
			continue;
		}

		
		if (widget_in_list(list, widget))
			continue;

		dev_dbg(fe->dev, "pruning %s BE %s for %s\n",
			stream ? "capture" : "playback", dpcm_params->be->dai_link->name,
			fe->dai_link->name);
		dpcm_params->state = SND_SOC_DPCM_LINK_STATE_FREE;
		dpcm_params->be->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_BE;
		old++;
	}

	dev_dbg(fe->dev, "found %d old BEs\n", old);
	return old;
}

static int be_add_new(struct snd_soc_pcm_runtime *fe, int stream,
	struct snd_soc_dapm_widget_list **list_)
{
	struct snd_soc_card *card = fe->card;
	struct snd_soc_dapm_widget_list *list = *list_;
	enum snd_soc_dapm_type be_type;
	int i, new = 0, err;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		be_type = snd_soc_dapm_aif_out;
	else
		be_type = snd_soc_dapm_aif_in;

	
	for (i = 0; i < list->num_widgets; i++) {

		if (list->widgets[i]->id == be_type) {
			struct snd_soc_pcm_runtime *be;

			
			be = be_get_rtd(card, list->widgets[i]);
			if (!be) {
				dev_err(fe->dev, "no BE found for %s\n",
						list->widgets[i]->name);
				continue;
			}

			
			if (!fe->dpcm[stream].runtime && !fe->fe_compr)
				continue;

			
			err = be_connect(fe, be, stream);
			if (err < 0) {
				dev_err(fe->dev, "can't connect %s\n", list->widgets[i]->name);
				break;
			} else if (err == 0) 
				continue;

			
			be->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_BE;
			new++;
		}
	}

	dev_dbg(fe->dev, "found %d new BEs\n", new);
	return new;
}

int dpcm_process_paths(struct snd_soc_pcm_runtime *fe,
	int stream, struct snd_soc_dapm_widget_list **list, int new)
{
	if (new)
		return be_add_new(fe, stream, list);
	else
		return be_prune_old(fe, stream, list);
	return 0;
}

void dpcm_clear_pending_state(struct snd_soc_pcm_runtime *fe, int stream)
{
	struct snd_soc_dpcm_params *dpcm_params;

	list_for_each_entry(dpcm_params, &fe->dpcm[stream].be_clients, list_be)
		dpcm_params->be->dpcm[stream].runtime_update =
						SND_SOC_DPCM_UPDATE_NO;
}

static void soc_dpcm_be_dai_startup_unwind(struct snd_soc_pcm_runtime *fe, int stream)
{
	struct snd_soc_dpcm_params *dpcm_params;

	
	list_for_each_entry(dpcm_params, &fe->dpcm[stream].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_pcm_substream *be_substream =
			snd_soc_dpcm_get_substream(be, stream);

		if (be->dpcm[stream].users == 0)
			dev_err(be->dev, "no users %s at close - state %d\n",
				stream ? "capture" : "playback", be->dpcm[stream].state);

		if (--be->dpcm[stream].users != 0)
			continue;

		if (be->dpcm[stream].state != SND_SOC_DPCM_STATE_OPEN)
			continue;

		soc_pcm_close(be_substream);
		be_substream->runtime = NULL;

		be->dpcm[stream].state = SND_SOC_DPCM_STATE_CLOSE;
	}
}

int dpcm_be_dai_startup(struct snd_soc_pcm_runtime *fe, int stream)
{
	struct snd_soc_dpcm_params *dpcm_params;
	int err, count = 0;

	
	list_for_each_entry(dpcm_params, &fe->dpcm[stream].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_pcm_substream *be_substream =
			snd_soc_dpcm_get_substream(be, stream);

		
		if (!snd_soc_dpcm_be_can_update(fe, be, stream))
			continue;

		
		if (be->dpcm[stream].users == MAX_BE_USERS)
			dev_err(be->dev, "too many users %s at open - state %d\n",
				stream ? "capture" : "playback", be->dpcm[stream].state);

		if (be->dpcm[stream].users++ != 0) {
			if(fe->dai_link->stream_name && be->dai_link->name)
				pr_info("fe %s ref be %s users %d \n",fe->dai_link->stream_name,\
					be->dai_link->name,be->dpcm[stream].users);
			continue;
		}

		if(fe->dai_link->stream_name && be->dai_link->name)
			pr_info("fe %s connec be %s users %d \n",fe->dai_link->stream_name,\
				be->dai_link->name,be->dpcm[stream].users);

		if ((be->dpcm[stream].state != SND_SOC_DPCM_STATE_NEW) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_CLOSE))
			continue;

		dev_dbg(be->dev, "dpcm: open BE %s\n", be->dai_link->name);

		pr_info("open path  %s %s %s \n", fe->dai_link->stream_name,(stream == SNDRV_PCM_STREAM_PLAYBACK)?"->":"<-",be->dai_link->name);

		be_substream->runtime = be->dpcm[stream].runtime;
		err = soc_pcm_open(be_substream);
		if (err < 0) {
			dev_err(be->dev, "BE open failed %d\n", err);
			be->dpcm[stream].users--;
			if (be->dpcm[stream].users < 0)
				dev_err(be->dev, "no users %s at unwind - state %d\n",
						stream ? "capture" : "playback",
						be->dpcm[stream].state);

			be->dpcm[stream].state = SND_SOC_DPCM_STATE_CLOSE;
			goto unwind;
		}

		be->dpcm[stream].state = SND_SOC_DPCM_STATE_OPEN;
		count++;
	}

	return count;

unwind:
	
	list_for_each_entry_continue_reverse(dpcm_params, &fe->dpcm[stream].be_clients, list_be) {
		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_pcm_substream *be_substream =
			snd_soc_dpcm_get_substream(be, stream);

		if (!snd_soc_dpcm_be_can_update(fe, be, stream))
			continue;

		if (be->dpcm[stream].users == 0)
			dev_err(be->dev, "no users %s at close - state %d\n",
				stream ? "capture" : "playback", be->dpcm[stream].state);

		if (--be->dpcm[stream].users != 0)
			continue;

		if (be->dpcm[stream].state != SND_SOC_DPCM_STATE_OPEN)
			continue;

		soc_pcm_close(be_substream);
		be_substream->runtime = NULL;

		be->dpcm[stream].state = SND_SOC_DPCM_STATE_CLOSE;
	}

	return err;
}

void soc_dpcm_set_dynamic_runtime(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai_driver *cpu_dai_drv = cpu_dai->driver;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		runtime->hw.rate_min = cpu_dai_drv->playback.rate_min;
		runtime->hw.rate_max = cpu_dai_drv->playback.rate_max;
		runtime->hw.channels_min = cpu_dai_drv->playback.channels_min;
		runtime->hw.channels_max = cpu_dai_drv->playback.channels_max;
		runtime->hw.formats &= cpu_dai_drv->playback.formats;
		runtime->hw.rates = cpu_dai_drv->playback.rates;
	} else {
		runtime->hw.rate_min = cpu_dai_drv->capture.rate_min;
		runtime->hw.rate_max = cpu_dai_drv->capture.rate_max;
		runtime->hw.channels_min = cpu_dai_drv->capture.channels_min;
		runtime->hw.channels_max = cpu_dai_drv->capture.channels_max;
		runtime->hw.formats &= cpu_dai_drv->capture.formats;
		runtime->hw.rates = cpu_dai_drv->capture.rates;
	}
}

static int soc_dpcm_fe_dai_startup(struct snd_pcm_substream *fe_substream)
{
	struct snd_soc_pcm_runtime *fe = fe_substream->private_data;
	struct snd_pcm_runtime *runtime = fe_substream->runtime;
	int stream = fe_substream->stream, ret = 0;

	fe->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_FE;

	ret = dpcm_be_dai_startup(fe, fe_substream->stream);
	if (ret < 0) {
		dev_err(fe->dev,"dpcm: failed to start some BEs %d\n", ret);
		goto be_err;
	}

	dev_dbg(fe->dev, "dpcm: open FE %s\n", fe->dai_link->name);

	
	ret = soc_pcm_open(fe_substream);
	if (ret < 0) {
		dev_err(fe->dev,"dpcm: failed to start FE %d\n", ret);
		goto unwind;
	}

	fe->dpcm[stream].state = SND_SOC_DPCM_STATE_OPEN;

	soc_dpcm_set_dynamic_runtime(fe_substream);
	snd_pcm_limit_hw_rates(runtime);

	fe->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_NO;
	return 0;

unwind:
	soc_dpcm_be_dai_startup_unwind(fe, fe_substream->stream);
be_err:
	fe->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_NO;
	return ret;
}

int dpcm_be_dai_shutdown(struct snd_soc_pcm_runtime *fe, int stream)
{
	struct snd_soc_dpcm_params *dpcm_params;

	
	list_for_each_entry(dpcm_params, &fe->dpcm[stream].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_pcm_substream *be_substream =
			snd_soc_dpcm_get_substream(be, stream);

		
		if (!snd_soc_dpcm_be_can_update(fe, be, stream))
			continue;

		if (be->dpcm[stream].users == 0)
			dev_err(be->dev, "no users %s at close - state %d\n",
				stream ? "capture" : "playback", be->dpcm[stream].state);

		if (--be->dpcm[stream].users != 0) {
			if(fe->dai_link->stream_name && be->dai_link->name)
				pr_info("fe %s discon be %s users %d\n", fe->dai_link->stream_name,\
					be->dai_link->name,be->dpcm[stream].users);
			continue;
		}

		if(fe->dai_link->stream_name && be->dai_link->name)
			pr_info("fe %s discon be %s users %d\n", fe->dai_link->stream_name,\
				be->dai_link->name,be->dpcm[stream].users);

		if ((be->dpcm[stream].state != SND_SOC_DPCM_STATE_HW_FREE) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_OPEN))
			continue;

		dev_dbg(be->dev, "dpcm: close BE %s\n",
			dpcm_params->fe->dai_link->name);
		pr_info("close path  %s %s %s \n", fe->dai_link->stream_name,(stream == SNDRV_PCM_STREAM_PLAYBACK)?"->":"<-",be->dai_link->name);
		soc_pcm_close(be_substream);
		be_substream->runtime = NULL;

		be->dpcm[stream].state = SND_SOC_DPCM_STATE_CLOSE;
	}
	return 0;
}

static int soc_dpcm_fe_dai_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *fe = substream->private_data;
	int stream = substream->stream;

	fe->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_FE;
	
	dev_dbg(fe->dev, "dpcm: close FE %s\n", fe->dai_link->name);

	
	soc_pcm_close(substream);

	
	dpcm_be_dai_shutdown(fe, substream->stream);
	
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		dpcm_dapm_stream_event(fe, stream,
				fe->cpu_dai->driver->playback.stream_name,
				SND_SOC_DAPM_STREAM_STOP);
	else
		dpcm_dapm_stream_event(fe, stream,
				fe->cpu_dai->driver->capture.stream_name,
				SND_SOC_DAPM_STREAM_STOP);

	fe->dpcm[stream].state = SND_SOC_DPCM_STATE_CLOSE;
	fe->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_NO;

	return 0;
}

int dpcm_be_dai_hw_params(struct snd_soc_pcm_runtime *fe, int stream)
{
	struct snd_soc_dpcm_params *dpcm_params;
	int ret;

	list_for_each_entry(dpcm_params, &fe->dpcm[stream].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_pcm_substream *be_substream =
			snd_soc_dpcm_get_substream(be, stream);

		
		if (!snd_soc_dpcm_be_can_update(fe, be, stream))
			continue;

		
		if (!snd_soc_dpcm_can_be_params(fe, be, stream))
			continue;

		if ((be->dpcm[stream].state != SND_SOC_DPCM_STATE_OPEN) &&
			(be->dpcm[stream].state != SND_SOC_DPCM_STATE_HW_PARAMS) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_HW_FREE))
			continue;

		dev_dbg(be->dev, "dpcm: hw_params BE %s\n",
			dpcm_params->fe->dai_link->name);

		
		memcpy(&dpcm_params->hw_params, &fe->dpcm[stream].hw_params,
				sizeof(struct snd_pcm_hw_params));

		
		if (be->dai_link->be_hw_params_fixup) {
			ret = be->dai_link->be_hw_params_fixup(be,
					&dpcm_params->hw_params);
			if (ret < 0) {
				dev_err(be->dev,
					"dpcm: hw_params BE fixup failed %d\n",
					ret);
				goto unwind;
			}
		}

		ret = soc_pcm_hw_params(be_substream, &dpcm_params->hw_params);
		if (ret < 0) {
			dev_err(dpcm_params->be->dev, "dpcm: hw_params BE failed %d\n", ret);
			dump_fe_state_by_be(fe, be, stream);
			goto unwind;
		}

		be->dpcm[stream].state = SND_SOC_DPCM_STATE_HW_PARAMS;
	}
	return 0;

unwind:
	
	list_for_each_entry_continue_reverse(dpcm_params, &fe->dpcm[stream].be_clients, list_be) {
		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_pcm_substream *be_substream =
			snd_soc_dpcm_get_substream(be, stream);

		if (!snd_soc_dpcm_be_can_update(fe, be, stream))
			continue;

		
		if (!snd_soc_dpcm_can_be_free_stop(fe, be, stream))
			continue;

		if ((be->dpcm[stream].state != SND_SOC_DPCM_STATE_OPEN) &&
			(be->dpcm[stream].state != SND_SOC_DPCM_STATE_HW_PARAMS) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_HW_FREE) &&
		     (be->dpcm[stream].state != SND_SOC_DPCM_STATE_STOP))
			continue;

		soc_pcm_hw_free(be_substream);
	}

	return ret;
}

int soc_dpcm_fe_dai_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *fe = substream->private_data;
	int ret, stream = substream->stream;

	mutex_lock(&fe->card->dpcm_mutex);
	fe->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_FE;

	memcpy(&fe->dpcm[substream->stream].hw_params, params,
			sizeof(struct snd_pcm_hw_params));
	ret = dpcm_be_dai_hw_params(fe, substream->stream);
	if (ret < 0) {
		dev_err(fe->dev,"dpcm: hw_params failed for some BEs %d\n", ret);
		goto out;
	}

	dev_dbg(fe->dev, "dpcm: hw_params FE %s rate %d chan %x fmt %d\n",
			fe->dai_link->name, params_rate(params), params_channels(params),
			params_format(params));

	
	ret = soc_pcm_hw_params(substream, params);
	if (ret < 0) {
		dev_err(fe->dev,"dpcm: hw_params FE failed %d\n", ret);
		dpcm_be_dai_hw_free(fe, stream);
	 } else
		fe->dpcm[stream].state = SND_SOC_DPCM_STATE_HW_PARAMS;

out:
	fe->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_NO;
	mutex_unlock(&fe->card->dpcm_mutex);
	return ret;
}

static int dpcm_do_trigger(struct snd_soc_dpcm_params *dpcm_params,
		struct snd_pcm_substream *substream, int cmd)
{
	int ret;

	dev_dbg(dpcm_params->be->dev, "dpcm: trigger BE %s cmd %d\n",
			dpcm_params->fe->dai_link->name, cmd);

	ret = soc_pcm_trigger(substream, cmd);
	if (ret < 0)
		dev_err(dpcm_params->be->dev,"dpcm: trigger BE failed %d\n", ret);

	return ret;
}

int dpcm_be_dai_trigger(struct snd_soc_pcm_runtime *fe, int stream, int cmd)
{
	struct snd_soc_dpcm_params *dpcm_params;
	int ret = 0;


	list_for_each_entry(dpcm_params, &fe->dpcm[stream].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_pcm_substream *be_substream =
			snd_soc_dpcm_get_substream(be, stream);

		
		if (!snd_soc_dpcm_be_can_update(fe, be, stream))
			continue;

		switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
			if ((be->dpcm[stream].state != SND_SOC_DPCM_STATE_PREPARE) &&
			    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_STOP))
				continue;

			ret = dpcm_do_trigger(dpcm_params, be_substream, cmd);
			if (ret)
				return ret;

			be->dpcm[stream].state = SND_SOC_DPCM_STATE_START;
			break;
		case SNDRV_PCM_TRIGGER_RESUME:
			if ((be->dpcm[stream].state != SND_SOC_DPCM_STATE_SUSPEND))
				continue;

			ret = dpcm_do_trigger(dpcm_params, be_substream, cmd);
			if (ret)
				return ret;

			be->dpcm[stream].state = SND_SOC_DPCM_STATE_START;
			break;
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			if ((be->dpcm[stream].state != SND_SOC_DPCM_STATE_PAUSED))
				continue;

			ret = dpcm_do_trigger(dpcm_params, be_substream, cmd);
			if (ret)
				return ret;

			be->dpcm[stream].state = SND_SOC_DPCM_STATE_START;
			break;
		case SNDRV_PCM_TRIGGER_STOP:
			if (be->dpcm[stream].state != SND_SOC_DPCM_STATE_START)
				continue;

			if (!snd_soc_dpcm_can_be_free_stop(fe, be, stream))
				continue;

			ret = dpcm_do_trigger(dpcm_params, be_substream, cmd);
			if (ret)
				return ret;

			be->dpcm[stream].state = SND_SOC_DPCM_STATE_STOP;
			break;
		case SNDRV_PCM_TRIGGER_SUSPEND:
			if (be->dpcm[stream].state != SND_SOC_DPCM_STATE_STOP)
				continue;

			if (!snd_soc_dpcm_can_be_free_stop(fe, be, stream))
				continue;

			ret = dpcm_do_trigger(dpcm_params, be_substream, cmd);
			if (ret)
				return ret;

			be->dpcm[stream].state = SND_SOC_DPCM_STATE_SUSPEND;
			break;
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			if (be->dpcm[stream].state != SND_SOC_DPCM_STATE_START)
				continue;

			if (!snd_soc_dpcm_can_be_free_stop(fe, be, stream))
				continue;

			ret = dpcm_do_trigger(dpcm_params, be_substream, cmd);
			if (ret)
				return ret;

			be->dpcm[stream].state = SND_SOC_DPCM_STATE_PAUSED;
			break;
		}
	}

	return ret;
}
EXPORT_SYMBOL_GPL(dpcm_be_dai_trigger);

int soc_dpcm_fe_dai_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *fe = substream->private_data;
	int stream = substream->stream, ret;
	enum snd_soc_dpcm_trigger trigger = fe->dai_link->trigger[stream];

	fe->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_FE;

	switch (trigger) {
	case SND_SOC_DPCM_TRIGGER_PRE:
		

		dev_dbg(fe->dev, "dpcm: pre trigger FE %s cmd %d\n",
				fe->dai_link->name, cmd);

		ret = soc_pcm_trigger(substream, cmd);
		if (ret < 0) {
			dev_err(fe->dev,"dpcm: trigger FE failed %d\n", ret);
			goto out;
		}

		ret = dpcm_be_dai_trigger(fe, substream->stream, cmd);
		break;
	case SND_SOC_DPCM_TRIGGER_POST:
		

		ret = dpcm_be_dai_trigger(fe, substream->stream, cmd);
		if (ret < 0) {
			dev_err(fe->dev,"dpcm: trigger FE failed %d\n", ret);
			goto out;
		}

		dev_dbg(fe->dev, "dpcm: post trigger FE %s cmd %d\n",
				fe->dai_link->name, cmd);

		ret = soc_pcm_trigger(substream, cmd);
		break;
	case SND_SOC_DPCM_TRIGGER_BESPOKE:
		

		dev_dbg(fe->dev, "dpcm: bespoke trigger FE %s cmd %d\n",
				fe->dai_link->name, cmd);

		ret = soc_pcm_bespoke_trigger(substream, cmd);
		if (ret < 0) {
			dev_err(fe->dev,"dpcm: trigger FE failed %d\n", ret);
			goto out;
		}
		break;
	default:
		dev_err(fe->dev, "dpcm: invalid trigger cmd %d for %s\n", cmd,
				fe->dai_link->name);
		ret = -EINVAL;
		goto out;
	}

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		fe->dpcm[stream].state = SND_SOC_DPCM_STATE_START;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		fe->dpcm[stream].state = SND_SOC_DPCM_STATE_STOP;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		fe->dpcm[stream].state = SND_SOC_DPCM_STATE_PAUSED;
		break;
	}

out:
	fe->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_NO;
	return ret;
}

int dpcm_be_dai_prepare(struct snd_soc_pcm_runtime *fe, int stream)
{
	struct snd_soc_dpcm_params *dpcm_params;
	int ret = 0;

	list_for_each_entry(dpcm_params, &fe->dpcm[stream].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_pcm_substream *be_substream =
			snd_soc_dpcm_get_substream(be, stream);

		
		if (!snd_soc_dpcm_be_can_update(fe, be, stream))
			continue;

		if ((be->dpcm[stream].state != SND_SOC_DPCM_STATE_HW_PARAMS) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_STOP))
			continue;

		dev_dbg(be->dev, "dpcm: prepare BE %s\n",
			dpcm_params->fe->dai_link->name);

		ret = soc_pcm_prepare(be_substream);
		if (ret < 0) {
			dev_err(be->dev, "dpcm: backend prepare failed %d\n",
				ret);
			break;
		}

		be->dpcm[stream].state = SND_SOC_DPCM_STATE_PREPARE;
	}
	return ret;
}

int soc_dpcm_fe_dai_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *fe = substream->private_data;
	int stream = substream->stream, ret = 0;

	mutex_lock(&fe->card->dpcm_mutex);

	dev_dbg(fe->dev, "dpcm: prepare FE %s\n", fe->dai_link->name);

	fe->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_FE;

	
	if (list_empty(&fe->dpcm[stream].be_clients)) {
		dev_err(fe->dev, "dpcm: no backend DAIs enabled for %s\n",
				fe->dai_link->name);
		ret = -EINVAL;
		goto out;
	}

	ret = dpcm_be_dai_prepare(fe, substream->stream);
	if (ret < 0) {
		dev_err(fe->dev, "ASoC: prepare FE %s failed\n",
						fe->dai_link->name);
		goto out;
	}

	
	if (!fe->fe_compr) {
		ret = soc_pcm_prepare(substream);
		if (ret < 0) {
			dev_err(fe->dev, "ASoC: prepare FE %s failed\n",
							fe->dai_link->name);
			goto out;
		}
	}

	
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		dpcm_dapm_stream_event(fe, stream,
				fe->cpu_dai->driver->playback.stream_name,
				SND_SOC_DAPM_STREAM_START);
	else
		dpcm_dapm_stream_event(fe, stream,
				fe->cpu_dai->driver->capture.stream_name,
				SND_SOC_DAPM_STREAM_START);

	fe->dpcm[stream].state = SND_SOC_DPCM_STATE_PREPARE;

out:
	fe->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_NO;
	mutex_unlock(&fe->card->dpcm_mutex);
	return ret;
}

int dpcm_be_dai_hw_free(struct snd_soc_pcm_runtime *fe, int stream)
{
	struct snd_soc_dpcm_params *dpcm_params;

	list_for_each_entry(dpcm_params, &fe->dpcm[stream].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_pcm_substream *be_substream =
			snd_soc_dpcm_get_substream(be, stream);

		
		if (!snd_soc_dpcm_be_can_update(fe, be, stream))
			continue;

		
		if (!snd_soc_dpcm_can_be_free_stop(fe, be, stream))
				continue;

		if ((be->dpcm[stream].state != SND_SOC_DPCM_STATE_HW_PARAMS) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_PREPARE) &&
			(be->dpcm[stream].state != SND_SOC_DPCM_STATE_HW_FREE) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_PAUSED) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_STOP) &&
		    !((be->dpcm[stream].state == SND_SOC_DPCM_STATE_START) &&
		      ((fe->dpcm[stream].state != SND_SOC_DPCM_STATE_START) &&
			(fe->dpcm[stream].state != SND_SOC_DPCM_STATE_PAUSED) &&
			(fe->dpcm[stream].state !=
						SND_SOC_DPCM_STATE_SUSPEND))))
			continue;

		dev_dbg(be->dev, "dpcm: hw_free BE %s\n",
			dpcm_params->fe->dai_link->name);

		soc_pcm_hw_free(be_substream);

		be->dpcm[stream].state = SND_SOC_DPCM_STATE_HW_FREE;
	}

	return 0;
}

int soc_dpcm_fe_dai_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *fe = substream->private_data;
	int err, stream = substream->stream;

	mutex_lock(&fe->card->dpcm_mutex);
	fe->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_FE;

	dev_dbg(fe->dev, "dpcm: hw_free FE %s\n", fe->dai_link->name);

	
	err = soc_pcm_hw_free(substream);
	if (err < 0)
		dev_err(fe->dev,"dpcm: hw_free FE %s failed\n", fe->dai_link->name);

	err = dpcm_be_dai_hw_free(fe, stream);

	fe->dpcm[stream].state = SND_SOC_DPCM_STATE_HW_FREE;
	fe->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_NO;

	mutex_unlock(&fe->card->dpcm_mutex);
	return 0;
}

static int soc_pcm_ioctl(struct snd_pcm_substream *substream,
		     unsigned int cmd, void *arg)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_platform *platform = rtd->platform;

	if (platform->driver->ops->ioctl)
		return platform->driver->ops->ioctl(substream, cmd, arg);
	return snd_pcm_lib_ioctl(substream, cmd, arg);
}

static int dpcm_run_update_shutdown(struct snd_soc_pcm_runtime *fe, int stream)
{
	struct snd_pcm_substream *substream = snd_soc_dpcm_get_substream(fe, stream);
	enum snd_soc_dpcm_trigger trigger = fe->dai_link->trigger[stream];
	int err;

	dev_dbg(fe->dev, "runtime %s close on FE %s\n",
			stream ? "capture" : "playback", fe->dai_link->name);

	if (trigger == SND_SOC_DPCM_TRIGGER_BESPOKE) {
		
		dev_dbg(fe->dev, "dpcm: bespoke trigger FE %s cmd stop\n",
				fe->dai_link->name);

		err = soc_pcm_bespoke_trigger(substream, SNDRV_PCM_TRIGGER_STOP);
		if (err < 0)
			dev_err(fe->dev,"dpcm: trigger FE failed %d\n", err);
	} else {
		dev_dbg(fe->dev, "dpcm: trigger FE %s cmd stop\n",
			fe->dai_link->name);

		err = dpcm_be_dai_trigger(fe, stream, SNDRV_PCM_TRIGGER_STOP);
		if (err < 0)
			dev_err(fe->dev,"dpcm: trigger FE failed %d\n", err);
	}

	err = dpcm_be_dai_hw_free(fe, stream);
	if (err < 0)
		dev_err(fe->dev,"dpcm: hw_free FE failed %d\n", err);

	err = dpcm_be_dai_shutdown(fe, stream);
	if (err < 0)
		dev_err(fe->dev,"dpcm: shutdown FE failed %d\n", err);

	
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		dpcm_dapm_stream_event(fe, stream,
				fe->cpu_dai->driver->playback.stream_name,
				SND_SOC_DAPM_STREAM_NOP);
	else
		dpcm_dapm_stream_event(fe, stream,
				fe->cpu_dai->driver->capture.stream_name,
				SND_SOC_DAPM_STREAM_NOP);

	return 0;
}

static int dpcm_run_update_startup(struct snd_soc_pcm_runtime *fe, int stream)
{
	struct snd_pcm_substream *substream = snd_soc_dpcm_get_substream(fe, stream);
	struct snd_soc_dpcm_params *dpcm_params;
	enum snd_soc_dpcm_trigger trigger = fe->dai_link->trigger[stream];
	int ret;

	dev_dbg(fe->dev, "runtime %s open on FE %s\n",
			stream ? "capture" : "playback", fe->dai_link->name);

	
	if (fe->dpcm[stream].state == SND_SOC_DPCM_STATE_HW_FREE ||
		fe->dpcm[stream].state == SND_SOC_DPCM_STATE_CLOSE)
		return -EINVAL;

	
	ret = dpcm_be_dai_startup(fe, stream);
	if (ret < 0) {
		goto disconnect;
		return ret;
	}

	
	if (fe->dpcm[stream].state == SND_SOC_DPCM_STATE_OPEN)
		return 0;

	ret = dpcm_be_dai_hw_params(fe, stream);
	if (ret < 0) {
		goto close;
		return ret;
	}

	
	if (fe->dpcm[stream].state == SND_SOC_DPCM_STATE_HW_PARAMS)
		return 0;


	ret = dpcm_be_dai_prepare(fe, stream);
	if (ret < 0) {
		goto hw_free;
		return ret;
	}

	
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		dpcm_dapm_stream_event(fe, stream,
				fe->cpu_dai->driver->playback.stream_name,
				SND_SOC_DAPM_STREAM_NOP);
	else
		dpcm_dapm_stream_event(fe, stream,
				fe->cpu_dai->driver->capture.stream_name,
				SND_SOC_DAPM_STREAM_NOP);

	
	if (fe->dpcm[stream].state == SND_SOC_DPCM_STATE_PREPARE ||
		fe->dpcm[stream].state == SND_SOC_DPCM_STATE_STOP)
		return 0;

	if (trigger == SND_SOC_DPCM_TRIGGER_BESPOKE) {
		
		dev_dbg(fe->dev, "dpcm: bespoke trigger FE %s cmd start\n",
				fe->dai_link->name);

		ret = soc_pcm_bespoke_trigger(substream, SNDRV_PCM_TRIGGER_START);
		if (ret < 0) {
			dev_err(fe->dev,"dpcm: bespoke trigger FE failed %d\n", ret);
			goto hw_free;
		}
	} else {
		dev_dbg(fe->dev, "dpcm: trigger FE %s cmd start\n",
			fe->dai_link->name);

		ret = dpcm_be_dai_trigger(fe, stream,
					SNDRV_PCM_TRIGGER_START);
		if (ret < 0) {
			dev_err(fe->dev,"dpcm: trigger FE failed %d\n", ret);
			goto hw_free;
		}
	}

	return 0;

hw_free:
	dpcm_be_dai_hw_free(fe, stream);
close:
	dpcm_be_dai_shutdown(fe, stream);
disconnect:
	
	list_for_each_entry(dpcm_params, &fe->dpcm[stream].be_clients, list_be) {
		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		if (be->dpcm[stream].state != SND_SOC_DPCM_STATE_START)
				dpcm_params->state = SND_SOC_DPCM_LINK_STATE_FREE;
	}

	return ret;
}

static int dpcm_run_new_update(struct snd_soc_pcm_runtime *fe, int stream)
{
	int ret;

	fe->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_BE;
	ret = dpcm_run_update_startup(fe, stream);
	if (ret < 0)
		dev_err(fe->dev, "failed to startup some BEs\n");
	fe->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_NO;

	return ret;
}

static int dpcm_run_old_update(struct snd_soc_pcm_runtime *fe, int stream)
{
	int ret;

	fe->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_BE;
	ret = dpcm_run_update_shutdown(fe, stream);
	if (ret < 0)
		dev_err(fe->dev, "failed to shutdown some BEs\n");
	fe->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_NO;

	return ret;
}

int soc_dpcm_runtime_update(struct snd_soc_dapm_widget *widget)
{
	struct snd_soc_card *card;
	int i, ret = 0, old, new, paths;

	if (widget->codec)
		card = widget->codec->card;
	else if (widget->platform)
		card = widget->platform->card;
	else
		return -EINVAL;

	mutex_lock(&card->dpcm_mutex);

	for (i = 0; i < card->num_rtd; i++) {
		struct snd_soc_dapm_widget_list *list;
		struct snd_soc_pcm_runtime *fe = &card->rtd[i];

		
		if (!fe->dai_link->dynamic)
			continue;

		
		if (!fe->cpu_dai->active)
			continue;

		
		dev_dbg(fe->dev, "DPCM runtime update for FE %s\n", fe->dai_link->name);

		
		if (!fe->cpu_dai->driver->playback.channels_min)
			goto capture;

		paths = dpcm_path_get(fe, SNDRV_PCM_STREAM_PLAYBACK, &list);
		if (paths < 0) {
			dev_warn(fe->dev, "%s no valid %s route from source to sink\n",
					fe->dai_link->name,  "playback");
			ret = paths;
			goto out;
		}

		
		new = dpcm_process_paths(fe, SNDRV_PCM_STREAM_PLAYBACK, &list, 1);
		if (new) {
			dpcm_run_new_update(fe, SNDRV_PCM_STREAM_PLAYBACK);
			dpcm_clear_pending_state(fe, SNDRV_PCM_STREAM_PLAYBACK);
			dpcm_be_disconnect(fe, SNDRV_PCM_STREAM_PLAYBACK);
		}

		
		old = dpcm_process_paths(fe, SNDRV_PCM_STREAM_PLAYBACK, &list, 0);
		if (old) {
			dpcm_run_old_update(fe, SNDRV_PCM_STREAM_PLAYBACK);
			dpcm_clear_pending_state(fe, SNDRV_PCM_STREAM_PLAYBACK);
			dpcm_be_disconnect(fe, SNDRV_PCM_STREAM_PLAYBACK);
		}

		dpcm_path_put(&list);

capture:
		
		if (!fe->cpu_dai->driver->capture.channels_min)
			continue;

		paths = dpcm_path_get(fe, SNDRV_PCM_STREAM_CAPTURE, &list);
		if (paths < 0) {
			dev_warn(fe->dev, "%s no valid %s route from source to sink\n",
					fe->dai_link->name,  "capture");
			ret = paths;
			goto out;
		}

		
		new = dpcm_process_paths(fe, SNDRV_PCM_STREAM_CAPTURE, &list, 1);
		if (new) {
			dpcm_run_new_update(fe, SNDRV_PCM_STREAM_CAPTURE);
			dpcm_clear_pending_state(fe, SNDRV_PCM_STREAM_CAPTURE);
			dpcm_be_disconnect(fe, SNDRV_PCM_STREAM_CAPTURE);
		}

		
		old = dpcm_process_paths(fe, SNDRV_PCM_STREAM_CAPTURE, &list, 0);
		if (old) {
			dpcm_run_old_update(fe, SNDRV_PCM_STREAM_CAPTURE);
			dpcm_clear_pending_state(fe, SNDRV_PCM_STREAM_CAPTURE);
			dpcm_be_disconnect(fe, SNDRV_PCM_STREAM_CAPTURE);
		}

		dpcm_path_put(&list);
	}

out:
	mutex_unlock(&card->dpcm_mutex);
	return ret;
}

int soc_dpcm_be_digital_mute(struct snd_soc_pcm_runtime *fe, int mute)
{
	struct snd_soc_dpcm_params *dpcm_params;

	list_for_each_entry(dpcm_params,
			&fe->dpcm[SNDRV_PCM_STREAM_PLAYBACK].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_soc_dai *dai = be->codec_dai;
		struct snd_soc_dai_driver *drv = dai->driver;

		if (be->dai_link->ignore_suspend)
			continue;

		dev_dbg(be->dev, "BE digital mute %s\n", be->dai_link->name);

		if (drv->ops->digital_mute && dai->playback_active)
				drv->ops->digital_mute(dai, mute);
	}

	return 0;
}

int soc_dpcm_be_cpu_dai_suspend(struct snd_soc_pcm_runtime *fe)
{
	struct snd_soc_dpcm_params *dpcm_params;

	
	list_for_each_entry(dpcm_params,
			&fe->dpcm[SNDRV_PCM_STREAM_PLAYBACK].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_soc_dai *dai = be->cpu_dai;
		struct snd_soc_dai_driver *drv = dai->driver;

		if (be->dai_link->ignore_suspend)
			continue;

		dev_dbg(be->dev, "pm: BE CPU DAI playback suspend %s\n",
				be->dai_link->name);

		if (drv->suspend && !drv->ac97_control)
				drv->suspend(dai);
	}

	
	list_for_each_entry(dpcm_params,
			&fe->dpcm[SNDRV_PCM_STREAM_CAPTURE].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_soc_dai *dai = be->cpu_dai;
		struct snd_soc_dai_driver *drv = dai->driver;

		if (be->dai_link->ignore_suspend)
			continue;

		dev_dbg(be->dev, "pm: BE CPU DAI capture suspend %s\n",
				be->dai_link->name);

		if (drv->suspend && !drv->ac97_control)
				drv->suspend(dai);
	}

	return 0;
}

int soc_dpcm_be_ac97_cpu_dai_suspend(struct snd_soc_pcm_runtime *fe)
{
	struct snd_soc_dpcm_params *dpcm_params;

	
	list_for_each_entry(dpcm_params,
			&fe->dpcm[SNDRV_PCM_STREAM_PLAYBACK].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_soc_dai *dai = be->cpu_dai;
		struct snd_soc_dai_driver *drv = dai->driver;

		if (be->dai_link->ignore_suspend)
			continue;

		dev_dbg(be->dev, "pm: BE CPU DAI playback suspend %s\n",
				be->dai_link->name);

		if (drv->suspend && drv->ac97_control)
				drv->suspend(dai);
	}

	
	list_for_each_entry(dpcm_params,
			&fe->dpcm[SNDRV_PCM_STREAM_CAPTURE].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_soc_dai *dai = be->cpu_dai;
		struct snd_soc_dai_driver *drv = dai->driver;

		if (be->dai_link->ignore_suspend)
			continue;

		dev_dbg(be->dev, "pm: BE CPU DAI capture suspend %s\n",
				be->dai_link->name);

		if (drv->suspend && drv->ac97_control)
				drv->suspend(dai);
	}

	return 0;
}

int soc_dpcm_be_platform_suspend(struct snd_soc_pcm_runtime *fe)
{
	struct snd_soc_dpcm_params *dpcm_params;

	
	list_for_each_entry(dpcm_params,
			&fe->dpcm[SNDRV_PCM_STREAM_PLAYBACK].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_soc_platform *platform = be->platform;
		struct snd_soc_platform_driver *drv = platform->driver;
		struct snd_soc_dai *dai = be->cpu_dai;

		if (be->dai_link->ignore_suspend)
			continue;

		dev_dbg(be->dev, "pm: BE platform playback suspend %s\n",
				be->dai_link->name);

		if (drv->suspend && !platform->suspended) {
			drv->suspend(dai);
			platform->suspended = 1;
		}
	}

	
	list_for_each_entry(dpcm_params,
			&fe->dpcm[SNDRV_PCM_STREAM_CAPTURE].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_soc_platform *platform = be->platform;
		struct snd_soc_platform_driver *drv = platform->driver;
		struct snd_soc_dai *dai = be->cpu_dai;

		if (be->dai_link->ignore_suspend)
			continue;

		dev_dbg(be->dev, "pm: BE platform capture suspend %s\n",
				be->dai_link->name);

		if (drv->suspend && !platform->suspended) {
			drv->suspend(dai);
			platform->suspended = 1;
		}
	}
	return 0;
}

int soc_dpcm_fe_suspend(struct snd_soc_pcm_runtime *fe)
{
	struct snd_soc_dai *dai = fe->cpu_dai;
	struct snd_soc_dai_driver *dai_drv = dai->driver;
	struct snd_soc_platform *platform = fe->platform;
	struct snd_soc_platform_driver *plat_drv = platform->driver;

	if (dai_drv->suspend && !dai_drv->ac97_control)
		dai_drv->suspend(dai);

	if (plat_drv->suspend && !platform->suspended) {
		plat_drv->suspend(dai);
		platform->suspended = 1;
	}

	soc_dpcm_be_cpu_dai_suspend(fe);
	soc_dpcm_be_platform_suspend(fe);

	return 0;
}

int soc_dpcm_be_cpu_dai_resume(struct snd_soc_pcm_runtime *fe)
{
	struct snd_soc_dpcm_params *dpcm_params;

	
	list_for_each_entry(dpcm_params,
			&fe->dpcm[SNDRV_PCM_STREAM_PLAYBACK].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_soc_dai *dai = be->cpu_dai;
		struct snd_soc_dai_driver *drv = dai->driver;

		if (be->dai_link->ignore_suspend)
			continue;

		dev_dbg(be->dev, "pm: BE CPU DAI playback resume %s\n",
				be->dai_link->name);

		if (drv->resume && !drv->ac97_control)
				drv->resume(dai);
	}

	
	list_for_each_entry(dpcm_params,
			&fe->dpcm[SNDRV_PCM_STREAM_CAPTURE].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_soc_dai *dai = be->cpu_dai;
		struct snd_soc_dai_driver *drv = dai->driver;

		if (be->dai_link->ignore_suspend)
			continue;

		dev_dbg(be->dev, "pm: BE CPU DAI capture resume %s\n",
				be->dai_link->name);

		if (drv->resume && !drv->ac97_control)
				drv->resume(dai);
	}

	return 0;
}

int soc_dpcm_be_ac97_cpu_dai_resume(struct snd_soc_pcm_runtime *fe)
{
	struct snd_soc_dpcm_params *dpcm_params;

	
	list_for_each_entry(dpcm_params,
			&fe->dpcm[SNDRV_PCM_STREAM_PLAYBACK].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_soc_dai *dai = be->cpu_dai;
		struct snd_soc_dai_driver *drv = dai->driver;

		if (be->dai_link->ignore_suspend)
			continue;

		dev_dbg(be->dev, "pm: BE CPU DAI playback resume %s\n",
				be->dai_link->name);

		if (drv->resume && drv->ac97_control)
				drv->resume(dai);
	}

	
	list_for_each_entry(dpcm_params,
			&fe->dpcm[SNDRV_PCM_STREAM_CAPTURE].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_soc_dai *dai = be->cpu_dai;
		struct snd_soc_dai_driver *drv = dai->driver;

		if (be->dai_link->ignore_suspend)
			continue;

		dev_dbg(be->dev, "pm: BE CPU DAI capture resume %s\n",
				be->dai_link->name);

		if (drv->resume && drv->ac97_control)
				drv->resume(dai);
	}

	return 0;
}

int soc_dpcm_be_platform_resume(struct snd_soc_pcm_runtime *fe)
{
	struct snd_soc_dpcm_params *dpcm_params;

	
	list_for_each_entry(dpcm_params,
			&fe->dpcm[SNDRV_PCM_STREAM_PLAYBACK].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_soc_platform *platform = be->platform;
		struct snd_soc_platform_driver *drv = platform->driver;
		struct snd_soc_dai *dai = be->cpu_dai;

		if (be->dai_link->ignore_suspend)
			continue;

		dev_dbg(be->dev, "pm: BE platform playback resume %s\n",
				be->dai_link->name);

		if (drv->resume && platform->suspended) {
			drv->resume(dai);
			platform->suspended = 0;
		}
	}

	
	list_for_each_entry(dpcm_params,
			&fe->dpcm[SNDRV_PCM_STREAM_CAPTURE].be_clients, list_be) {

		struct snd_soc_pcm_runtime *be = dpcm_params->be;
		struct snd_soc_platform *platform = be->platform;
		struct snd_soc_platform_driver *drv = platform->driver;
		struct snd_soc_dai *dai = be->cpu_dai;

		if (be->dai_link->ignore_suspend)
			continue;

		dev_dbg(be->dev, "pm: BE platform capture resume %s\n",
				be->dai_link->name);

		if (drv->resume && platform->suspended) {
			drv->resume(dai);
			platform->suspended = 0;
		}
	}

	return 0;
}

int soc_dpcm_fe_resume(struct snd_soc_pcm_runtime *fe)
{
	struct snd_soc_dai *dai = fe->cpu_dai;
	struct snd_soc_dai_driver *dai_drv = dai->driver;
	struct snd_soc_platform *platform = fe->platform;
	struct snd_soc_platform_driver *plat_drv = platform->driver;

	soc_dpcm_be_cpu_dai_resume(fe);
	soc_dpcm_be_platform_resume(fe);

	if (dai_drv->resume && !dai_drv->ac97_control)
		dai_drv->resume(dai);

	if (plat_drv->resume && platform->suspended) {
		plat_drv->resume(dai);
		platform->suspended = 0;
	}

	return 0;
}

int soc_dpcm_fe_dai_open(struct snd_pcm_substream *fe_substream)
{
	struct snd_soc_pcm_runtime *fe = fe_substream->private_data;
	struct snd_soc_dpcm_params *dpcm_params;
	struct snd_soc_dapm_widget_list *list;
	int ret;
	int stream = fe_substream->stream;

	mutex_lock(&fe->card->dpcm_mutex);
	fe->dpcm[stream].runtime = fe_substream->runtime;

	if ((ret = dpcm_path_get(fe, stream, &list)) <= 0) {
		dev_warn(fe->dev, "asoc: %s no valid %s route from source to sink\n",
			fe->dai_link->name, stream ? "capture" : "playback");
		if(ret == 0 && list)
			dpcm_path_put(&list);
		mutex_unlock(&fe->card->dpcm_mutex);
		return -EINVAL;
	}

	
	dpcm_process_paths(fe, stream, &list, 1);

	ret = soc_dpcm_fe_dai_startup(fe_substream);
	if (ret < 0) {
		
		list_for_each_entry(dpcm_params, &fe->dpcm[stream].be_clients, list_be)
				dpcm_params->state = SND_SOC_DPCM_LINK_STATE_FREE;

		dpcm_be_disconnect(fe, stream);
		fe->dpcm[stream].runtime = NULL;
	}

	dpcm_clear_pending_state(fe, stream);
	dpcm_path_put(&list);
	mutex_unlock(&fe->card->dpcm_mutex);
	return ret;
}

int soc_dpcm_fe_dai_close(struct snd_pcm_substream *fe_substream)
{
	struct snd_soc_pcm_runtime *fe = fe_substream->private_data;
	struct snd_soc_dpcm_params *dpcm_params;
	int stream = fe_substream->stream, ret;

	mutex_lock(&fe->card->dpcm_mutex);
	ret = soc_dpcm_fe_dai_shutdown(fe_substream);

	
	list_for_each_entry(dpcm_params, &fe->dpcm[stream].be_clients, list_be)
		dpcm_params->state = SND_SOC_DPCM_LINK_STATE_FREE;

	dpcm_be_disconnect(fe, stream);

	fe->dpcm[stream].runtime = NULL;
	mutex_unlock(&fe->card->dpcm_mutex);
	return ret;
}

int soc_new_pcm(struct snd_soc_pcm_runtime *rtd, int num)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_platform *platform = rtd->platform;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_pcm_substream *substream[2];
	struct snd_pcm *pcm;
	char new_name[64];
	int ret = 0, playback = 0, capture = 0;

	if (rtd->dai_link->dynamic || rtd->dai_link->no_pcm) {
		if (cpu_dai->driver->playback.channels_min)
			playback = 1;
		if (cpu_dai->driver->capture.channels_min)
			capture = 1;
	} else {
		if (codec_dai->driver->playback.channels_min)
			playback = 1;
		if (codec_dai->driver->capture.channels_min)
			capture = 1;
	}

	
	if (rtd->dai_link->no_pcm) {
		snprintf(new_name, sizeof(new_name), "(%s)",
			rtd->dai_link->stream_name);

		ret = snd_pcm_new_soc_be(rtd->card->snd_card, new_name, num,
				playback, capture, &pcm);
	} else {
		if (rtd->dai_link->dynamic)
			snprintf(new_name, sizeof(new_name), "%s (*)",
				rtd->dai_link->stream_name);
		else
			snprintf(new_name, sizeof(new_name), "%s %s-%d",
				rtd->dai_link->stream_name, codec_dai->name, num);

		ret = snd_pcm_new(rtd->card->snd_card, new_name, num, playback,
			capture, &pcm);
	}
	if (ret < 0) {
		printk(KERN_ERR "asoc: can't create pcm for codec %s\n", codec->name);
		return ret;
	}
	dev_dbg(rtd->card->dev, "registered pcm #%d %s\n",num, new_name);

	rtd->pcm = pcm;
	pcm->private_data = rtd;
	INIT_DELAYED_WORK(&rtd->delayed_work, close_delayed_work);

	substream[SNDRV_PCM_STREAM_PLAYBACK] =
			pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
	substream[SNDRV_PCM_STREAM_CAPTURE] =
			pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;

	if (rtd->dai_link->no_pcm) {
		if (playback)
			substream[SNDRV_PCM_STREAM_PLAYBACK]->private_data = rtd;
		if (capture)
			substream[SNDRV_PCM_STREAM_CAPTURE]->private_data = rtd;
		goto out;
	}

        
	if (rtd->dai_link->no_host_mode) {
		if (substream[SNDRV_PCM_STREAM_PLAYBACK]) {
			substream[SNDRV_PCM_STREAM_PLAYBACK]->hw_no_buffer = 1;
			snd_soc_set_runtime_hwparams(
				substream[SNDRV_PCM_STREAM_PLAYBACK],
				&no_host_hardware);
		}
		if (substream[SNDRV_PCM_STREAM_CAPTURE]) {
			substream[SNDRV_PCM_STREAM_CAPTURE]->hw_no_buffer = 1;
			snd_soc_set_runtime_hwparams(
				substream[SNDRV_PCM_STREAM_CAPTURE],
				&no_host_hardware);
		}
	}

	
	if (rtd->dai_link->dynamic) {
		rtd->ops.open		= soc_dpcm_fe_dai_open;
		rtd->ops.hw_params	= soc_dpcm_fe_dai_hw_params;
		rtd->ops.prepare	= soc_dpcm_fe_dai_prepare;
		rtd->ops.trigger	= soc_dpcm_fe_dai_trigger;
		rtd->ops.hw_free	= soc_dpcm_fe_dai_hw_free;
		rtd->ops.close		= soc_dpcm_fe_dai_close;
		rtd->ops.pointer	= soc_pcm_pointer;
		rtd->ops.ioctl		= soc_pcm_ioctl;
	} else {
		rtd->ops.open		= soc_pcm_open;
		rtd->ops.hw_params	= soc_pcm_hw_params;
		rtd->ops.prepare	= soc_pcm_prepare;
		rtd->ops.trigger	= soc_pcm_trigger;
		rtd->ops.hw_free	= soc_pcm_hw_free;
		rtd->ops.close		= soc_pcm_close;
		rtd->ops.pointer	= soc_pcm_pointer;
		rtd->ops.ioctl		= soc_pcm_ioctl;
	}

	if (platform->driver->ops) {
		rtd->ops.ack		= platform->driver->ops->ack;
		rtd->ops.copy		= platform->driver->ops->copy;
		rtd->ops.silence	= platform->driver->ops->silence;
		rtd->ops.page		= platform->driver->ops->page;
		rtd->ops.mmap		= platform->driver->ops->mmap;
		rtd->ops.restart	= platform->driver->ops->restart;
	}

	if (playback)
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &rtd->ops);

	if (capture)
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &rtd->ops);

	if (platform->driver->pcm_new) {
		ret = platform->driver->pcm_new(rtd);
		if (ret < 0) {
			pr_err("asoc: platform pcm constructor failed\n");
			return ret;
		}
	}

	pcm->private_free = platform->driver->pcm_free;
out:
	printk(KERN_INFO "asoc: %s <-> %s mapping ok\n", codec_dai->name,
		cpu_dai->name);
	return ret;
}

#ifdef CONFIG_DEBUG_FS
static char *dpcm_state_string(enum snd_soc_dpcm_state state)
{
	switch (state) {
	case SND_SOC_DPCM_STATE_NEW:
		return "new";
	case SND_SOC_DPCM_STATE_OPEN:
		return "open";
	case SND_SOC_DPCM_STATE_HW_PARAMS:
		return "hw_params";
	case SND_SOC_DPCM_STATE_PREPARE:
		return "prepare";
	case SND_SOC_DPCM_STATE_START:
		return "start";
	case SND_SOC_DPCM_STATE_STOP:
		return "stop";
	case SND_SOC_DPCM_STATE_SUSPEND:
		return "suspend";
	case SND_SOC_DPCM_STATE_PAUSED:
		return "paused";
	case SND_SOC_DPCM_STATE_HW_FREE:
		return "hw_free";
	case SND_SOC_DPCM_STATE_CLOSE:
		return "close";
	}

	return "unknown";
}

static int soc_dpcm_state_open_file(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t soc_dpcm_show_state(struct snd_soc_pcm_runtime *fe,
				int stream, char *buf, size_t size)
{
	struct snd_pcm_hw_params *params = &fe->dpcm[stream].hw_params;
	struct snd_soc_dpcm_params *dpcm_params;
	ssize_t offset = 0;

	
	offset += snprintf(buf + offset, size - offset,
			"[%s - %s]\n", fe->dai_link->name,
			stream ? "Capture" : "Playback");

	offset += snprintf(buf + offset, size - offset, "State: %s\n",
	                dpcm_state_string(fe->dpcm[stream].state));

	if ((fe->dpcm[stream].state >= SND_SOC_DPCM_STATE_HW_PARAMS) &&
	    (fe->dpcm[stream].state <= SND_SOC_DPCM_STATE_STOP))
		offset += snprintf(buf + offset, size - offset,
				"Hardware Params: "
				"Format = %s, Channels = %d, Rate = %d\n",
				snd_pcm_format_name(params_format(params)),
				params_channels(params),
				params_rate(params));

	
	offset += snprintf(buf + offset, size - offset, "Backends:\n");

	if (list_empty(&fe->dpcm[stream].be_clients)) {
		offset += snprintf(buf + offset, size - offset,
				" No active DSP links\n");
		goto out;
	}

	list_for_each_entry(dpcm_params, &fe->dpcm[stream].be_clients, list_be) {
		struct snd_soc_pcm_runtime *be = dpcm_params->be;

		offset += snprintf(buf + offset, size - offset,
				"- %s\n", be->dai_link->name);

		offset += snprintf(buf + offset, size - offset,
				"   State: %s\n",
				dpcm_state_string(fe->dpcm[stream].state));

		if ((be->dpcm[stream].state >= SND_SOC_DPCM_STATE_HW_PARAMS) &&
		    (be->dpcm[stream].state <= SND_SOC_DPCM_STATE_STOP))
			offset += snprintf(buf + offset, size - offset,
				"   Hardware Params: "
				"Format = %s, Channels = %d, Rate = %d\n",
				snd_pcm_format_name(params_format(params)),
				params_channels(params),
				params_rate(params));
	}

out:
	return offset;
}

static ssize_t soc_dpcm_state_read_file(struct file *file, char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct snd_soc_pcm_runtime *fe = file->private_data;
	ssize_t out_count = PAGE_SIZE, offset = 0, ret = 0;
	char *buf;

	buf = kmalloc(out_count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (fe->cpu_dai->driver->playback.channels_min)
		offset += soc_dpcm_show_state(fe, SNDRV_PCM_STREAM_PLAYBACK,
					buf + offset, out_count - offset);

	if (fe->cpu_dai->driver->capture.channels_min)
		offset += soc_dpcm_show_state(fe, SNDRV_PCM_STREAM_CAPTURE,
					buf + offset, out_count - offset);

        ret = simple_read_from_buffer(user_buf, count, ppos, buf, offset);

        kfree(buf);

        return ret;
}

static const struct file_operations soc_dpcm_state_fops = {
	.open = soc_dpcm_state_open_file,
	.read = soc_dpcm_state_read_file,
	.llseek = default_llseek,
};

int soc_dpcm_debugfs_add(struct snd_soc_pcm_runtime *rtd)
{
	rtd->debugfs_dpcm_root = debugfs_create_dir(rtd->dai_link->name,
			rtd->card->debugfs_card_root);
	if (!rtd->debugfs_dpcm_root) {
		dev_dbg(rtd->dev,
			 "ASoC: Failed to create dpcm debugfs directory %s\n",
			 rtd->dai_link->name);
		return -EINVAL;
	}

	rtd->debugfs_dpcm_state = debugfs_create_file("state", 0644,
						rtd->debugfs_dpcm_root,
						rtd, &soc_dpcm_state_fops);

	return 0;
}
#endif
