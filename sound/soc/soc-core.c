/*
 * soc-core.c  --  ALSA SoC Audio Layer
 *
 * Copyright 2005 Wolfson Microelectronics PLC.
 * Copyright 2005 Openedhand Ltd.
 * Copyright (C) 2010 Slimlogic Ltd.
 * Copyright (C) 2010 Texas Instruments Inc.
 *
 * Author: Liam Girdwood <lrg@slimlogic.co.uk>
 *         with code, comments and ideas from :-
 *         Richard Purdie <richard@openedhand.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  TODO:
 *   o Add hw rules to enforce rates, etc.
 *   o More testing with other codecs/machines.
 *   o Add more codecs and platforms to ensure good API coverage.
 *   o Support TDM on PCM and I2S
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <sound/ac97_codec.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dpcm.h>
#include <sound/initval.h>

#define CREATE_TRACE_POINTS
#include <trace/events/asoc.h>

#define NAME_SIZE	32

static DECLARE_WAIT_QUEUE_HEAD(soc_pm_waitq);

#ifdef CONFIG_DEBUG_FS
struct dentry *snd_soc_debugfs_root;
EXPORT_SYMBOL_GPL(snd_soc_debugfs_root);
#endif

static DEFINE_MUTEX(client_mutex);
static LIST_HEAD(dai_list);
static LIST_HEAD(platform_list);
static LIST_HEAD(codec_list);

int soc_new_pcm(struct snd_soc_pcm_runtime *rtd, int num);
int soc_dpcm_debugfs_add(struct snd_soc_pcm_runtime *rtd);
int soc_dpcm_be_digital_mute(struct snd_soc_pcm_runtime *fe, int mute);
int soc_dpcm_be_ac97_cpu_dai_suspend(struct snd_soc_pcm_runtime *fe);
int soc_dpcm_be_ac97_cpu_dai_resume(struct snd_soc_pcm_runtime *fe);
int soc_dpcm_be_cpu_dai_resume(struct snd_soc_pcm_runtime *fe);
int soc_dpcm_be_cpu_dai_suspend(struct snd_soc_pcm_runtime *fe);
int soc_dpcm_be_platform_suspend(struct snd_soc_pcm_runtime *fe);
int soc_dpcm_be_platform_resume(struct snd_soc_pcm_runtime *fe);

static int pmdown_time;
module_param(pmdown_time, int, 0);
MODULE_PARM_DESC(pmdown_time, "DAPM stream powerdown time (msecs)");

static int min_bytes_needed(unsigned long val)
{
	int c = 0;
	int i;

	for (i = (sizeof val * 8) - 1; i >= 0; --i, ++c)
		if (val & (1UL << i))
			break;
	c = (sizeof val * 8) - c;
	if (!c || (c % 8))
		c = (c + 8) / 8;
	else
		c /= 8;
	return c;
}

static int format_register_str(struct snd_soc_codec *codec,
			       unsigned int reg, char *buf, size_t len)
{
	int wordsize = min_bytes_needed(codec->driver->reg_cache_size) * 2;
	int regsize = codec->driver->reg_word_size * 2;
	int ret;
	char tmpbuf[len + 1];
	char regbuf[regsize + 1];

	WARN_ON(len > 63);

	
	if (wordsize + regsize + 2 + 1 != len)
		return -EINVAL;

	ret = snd_soc_read(codec, reg);
	if (ret < 0) {
		memset(regbuf, 'X', regsize);
		regbuf[regsize] = '\0';
	} else {
		snprintf(regbuf, regsize + 1, "%.*x", regsize, ret);
	}

	
	snprintf(tmpbuf, len + 1, "%.*x: %s\n", wordsize, reg, regbuf);
	
	memcpy(buf, tmpbuf, len);

	return 0;
}

static ssize_t soc_codec_reg_show(struct snd_soc_codec *codec, char *buf,
				  size_t count, loff_t pos)
{
	int i, step = 1;
	int wordsize, regsize;
	int len;
	size_t total = 0;
	loff_t p = 0;

	wordsize = min_bytes_needed(codec->driver->reg_cache_size) * 2;
	regsize = codec->driver->reg_word_size * 2;

	len = wordsize + regsize + 2 + 1;

	if (!codec->driver->reg_cache_size)
		return 0;

	if (codec->driver->reg_cache_step)
		step = codec->driver->reg_cache_step;

	for (i = 0; i < codec->driver->reg_cache_size; i += step) {
		if (!snd_soc_codec_readable_register(codec, i))
			continue;
		if (codec->driver->display_register) {
			count += codec->driver->display_register(codec, buf + count,
							 PAGE_SIZE - count, i);
		} else {
			if (p >= pos) {
				if (total + len >= count - 1)
					break;
				format_register_str(codec, i, buf + total, len);
				total += len;
			}
			p += len;
		}
	}

	total = min(total, count - 1);

	return total;
}

static ssize_t codec_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct snd_soc_pcm_runtime *rtd = dev_get_drvdata(dev);

	return soc_codec_reg_show(rtd->codec, buf, PAGE_SIZE, 0);
}

static DEVICE_ATTR(codec_reg, 0444, codec_reg_show, NULL);

static ssize_t pmdown_time_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct snd_soc_pcm_runtime *rtd = dev_get_drvdata(dev);

	return sprintf(buf, "%ld\n", rtd->pmdown_time);
}

static ssize_t pmdown_time_set(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct snd_soc_pcm_runtime *rtd = dev_get_drvdata(dev);
	int ret;

	ret = strict_strtol(buf, 10, &rtd->pmdown_time);
	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR(pmdown_time, 0644, pmdown_time_show, pmdown_time_set);

#ifdef CONFIG_DEBUG_FS
static int codec_reg_open_file(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t codec_reg_read_file(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	ssize_t ret;
	struct snd_soc_codec *codec = file->private_data;
	char *buf;

	if (*ppos < 0 || !count)
		return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = soc_codec_reg_show(codec, buf, count, *ppos);
	if (ret >= 0) {
		if (copy_to_user(user_buf, buf, ret)) {
			kfree(buf);
			return -EFAULT;
		}
		*ppos += ret;
	}

	kfree(buf);
	return ret;
}

static ssize_t codec_reg_write_file(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	char buf[32];
	size_t buf_size;
	char *start = buf;
	unsigned long reg, value;
	struct snd_soc_codec *codec = file->private_data;

	buf_size = min(count, (sizeof(buf)-1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	while (*start == ' ')
		start++;
	reg = simple_strtoul(start, &start, 16);
	while (*start == ' ')
		start++;
	if (strict_strtoul(start, 16, &value))
		return -EINVAL;

	
	add_taint(TAINT_USER);

	snd_soc_write(codec, reg, value);
	return buf_size;
}

static const struct file_operations codec_reg_fops = {
	.open = codec_reg_open_file,
	.read = codec_reg_read_file,
	.write = codec_reg_write_file,
	.llseek = default_llseek,
};

static void soc_init_codec_debugfs(struct snd_soc_codec *codec)
{
	struct dentry *debugfs_card_root = codec->card->debugfs_card_root;

	codec->debugfs_codec_root = debugfs_create_dir(codec->name,
						       debugfs_card_root);
	if (!codec->debugfs_codec_root) {
		printk(KERN_WARNING
		       "ASoC: Failed to create codec debugfs directory\n");
		return;
	}

	debugfs_create_bool("cache_sync", 0444, codec->debugfs_codec_root,
			    &codec->cache_sync);
	debugfs_create_bool("cache_only", 0444, codec->debugfs_codec_root,
			    &codec->cache_only);

	codec->debugfs_reg = debugfs_create_file("codec_reg", 0644,
						 codec->debugfs_codec_root,
						 codec, &codec_reg_fops);
	if (!codec->debugfs_reg)
		printk(KERN_WARNING
		       "ASoC: Failed to create codec register debugfs file\n");

	snd_soc_dapm_debugfs_init(&codec->dapm, codec->debugfs_codec_root);
}

static void soc_cleanup_codec_debugfs(struct snd_soc_codec *codec)
{
	debugfs_remove_recursive(codec->debugfs_codec_root);
}

static void soc_init_platform_debugfs(struct snd_soc_platform *platform)
{
	struct dentry *debugfs_card_root = platform->card->debugfs_card_root;

	platform->debugfs_platform_root = debugfs_create_dir(platform->name,
						       debugfs_card_root);
	if (!platform->debugfs_platform_root) {
		printk(KERN_WARNING
		       "ASoC: Failed to create platform debugfs directory\n");
		return;
	}

	snd_soc_dapm_debugfs_init(&platform->dapm, platform->debugfs_platform_root);
}

static void soc_cleanup_platform_debugfs(struct snd_soc_platform *platform)
{
	debugfs_remove_recursive(platform->debugfs_platform_root);
}

static ssize_t codec_list_read_file(struct file *file, char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	char *buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	ssize_t len, ret = 0;
	struct snd_soc_codec *codec;

	if (!buf)
		return -ENOMEM;

	list_for_each_entry(codec, &codec_list, list) {
		len = snprintf(buf + ret, PAGE_SIZE - ret, "%s\n",
			       codec->name);
		if (len >= 0)
			ret += len;
		if (ret > PAGE_SIZE) {
			ret = PAGE_SIZE;
			break;
		}
	}

	if (ret >= 0)
		ret = simple_read_from_buffer(user_buf, count, ppos, buf, ret);

	kfree(buf);

	return ret;
}

static const struct file_operations codec_list_fops = {
	.read = codec_list_read_file,
	.llseek = default_llseek,
};

static ssize_t dai_list_read_file(struct file *file, char __user *user_buf,
				  size_t count, loff_t *ppos)
{
	char *buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	ssize_t len, ret = 0;
	struct snd_soc_dai *dai;

	if (!buf)
		return -ENOMEM;

	list_for_each_entry(dai, &dai_list, list) {
		len = snprintf(buf + ret, PAGE_SIZE - ret, "%s\n", dai->name);
		if (len >= 0)
			ret += len;
		if (ret > PAGE_SIZE) {
			ret = PAGE_SIZE;
			break;
		}
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, ret);

	kfree(buf);

	return ret;
}

static const struct file_operations dai_list_fops = {
	.read = dai_list_read_file,
	.llseek = default_llseek,
};

static ssize_t platform_list_read_file(struct file *file,
				       char __user *user_buf,
				       size_t count, loff_t *ppos)
{
	char *buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	ssize_t len, ret = 0;
	struct snd_soc_platform *platform;

	if (!buf)
		return -ENOMEM;

	list_for_each_entry(platform, &platform_list, list) {
		len = snprintf(buf + ret, PAGE_SIZE - ret, "%s\n",
			       platform->name);
		if (len >= 0)
			ret += len;
		if (ret > PAGE_SIZE) {
			ret = PAGE_SIZE;
			break;
		}
	}

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, ret);

	kfree(buf);

	return ret;
}

static const struct file_operations platform_list_fops = {
	.read = platform_list_read_file,
	.llseek = default_llseek,
};

static void soc_init_card_debugfs(struct snd_soc_card *card)
{
	card->debugfs_card_root = debugfs_create_dir(card->name,
						     snd_soc_debugfs_root);
	if (!card->debugfs_card_root) {
		dev_warn(card->dev,
			 "ASoC: Failed to create card debugfs directory\n");
		return;
	}

	card->debugfs_pop_time = debugfs_create_u32("dapm_pop_time", 0644,
						    card->debugfs_card_root,
						    &card->pop_time);
	if (!card->debugfs_pop_time)
		dev_warn(card->dev,
		       "Failed to create pop time debugfs file\n");
}

static void soc_cleanup_card_debugfs(struct snd_soc_card *card)
{
	debugfs_remove_recursive(card->debugfs_card_root);
}

#else

static inline void soc_init_codec_debugfs(struct snd_soc_codec *codec)
{
}

static inline void soc_cleanup_codec_debugfs(struct snd_soc_codec *codec)
{
}

static inline void soc_init_platform_debugfs(struct snd_soc_platform *platform)
{
}

static inline void soc_cleanup_platform_debugfs(struct snd_soc_platform *platform)
{
}

static inline void soc_init_card_debugfs(struct snd_soc_card *card)
{
}

static inline void soc_cleanup_card_debugfs(struct snd_soc_card *card)
{
}
#endif

struct snd_pcm_substream *snd_soc_get_dai_substream(struct snd_soc_card *card,
		const char *dai_link, int stream)
{
	int i;

	for (i = 0; i < card->num_links; i++) {
		if (card->rtd[i].dai_link->no_pcm &&
			!strcmp(card->rtd[i].dai_link->name, dai_link))
			return card->rtd[i].pcm->streams[stream].substream;
	}
	dev_dbg(card->dev, "failed to find dai link %s\n", dai_link);
	return NULL;
}
EXPORT_SYMBOL_GPL(snd_soc_get_dai_substream);

struct snd_soc_pcm_runtime *snd_soc_get_pcm_runtime(struct snd_soc_card *card,
		const char *dai_link)
{
	int i;

	for (i = 0; i < card->num_links; i++) {
		if (!strcmp(card->rtd[i].dai_link->name, dai_link))
			return &card->rtd[i];
	}
	dev_dbg(card->dev, "failed to find rtd %s\n", dai_link);
	return NULL;
}
EXPORT_SYMBOL_GPL(snd_soc_get_pcm_runtime);

#ifdef CONFIG_SND_SOC_AC97_BUS
static int soc_ac97_dev_unregister(struct snd_soc_codec *codec)
{
	if (codec->ac97->dev.bus)
		device_unregister(&codec->ac97->dev);
	return 0;
}

static void soc_ac97_device_release(struct device *dev){}

static int soc_ac97_dev_register(struct snd_soc_codec *codec)
{
	int err;

	codec->ac97->dev.bus = &ac97_bus_type;
	codec->ac97->dev.parent = codec->card->dev;
	codec->ac97->dev.release = soc_ac97_device_release;

	dev_set_name(&codec->ac97->dev, "%d-%d:%s",
		     codec->card->snd_card->number, 0, codec->name);
	err = device_register(&codec->ac97->dev);
	if (err < 0) {
		snd_printk(KERN_ERR "Can't register ac97 bus\n");
		codec->ac97->dev.bus = NULL;
		return err;
	}
	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
int snd_soc_suspend(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct snd_soc_codec *codec;
	int i;

	if (list_empty(&card->codec_dev_list))
		return 0;

	snd_power_lock(card->snd_card);
	snd_power_wait(card->snd_card, SNDRV_CTL_POWER_D0);
	snd_power_unlock(card->snd_card);

	
	snd_power_change_state(card->snd_card, SNDRV_CTL_POWER_D3hot);

	
	for (i = 0; i < card->num_rtd; i++) {
		struct snd_soc_dai *dai = card->rtd[i].codec_dai;
		struct snd_soc_dai_driver *drv = dai->driver;

		if (card->rtd[i].dai_link->ignore_suspend ||
				card->rtd[i].dai_link->no_pcm)
			continue;

		if (card->rtd[i].dai_link->dynamic)
			soc_dpcm_be_digital_mute(&card->rtd[i], 1);
		else {
			if (drv->ops->digital_mute && dai->playback_active)
				drv->ops->digital_mute(dai, 1);
		}
	}

	
	for (i = 0; i < card->num_rtd; i++) {
		if (card->rtd[i].dai_link->ignore_suspend ||
				card->rtd[i].dai_link->no_pcm)
			continue;

		snd_pcm_suspend_all(card->rtd[i].pcm);
	}

	if (card->suspend_pre)
		card->suspend_pre(card);

	for (i = 0; i < card->num_rtd; i++) {
		struct snd_soc_dai *cpu_dai = card->rtd[i].cpu_dai;
		struct snd_soc_platform *platform = card->rtd[i].platform;

		if (card->rtd[i].dai_link->ignore_suspend ||
				card->rtd[i].dai_link->no_pcm)
			continue;

		if (card->rtd[i].dai_link->dynamic) {
			soc_dpcm_be_cpu_dai_suspend(&card->rtd[i]);
			soc_dpcm_be_platform_suspend(&card->rtd[i]);
		} else {
			if (cpu_dai->driver->suspend && !cpu_dai->driver->ac97_control)
				cpu_dai->driver->suspend(cpu_dai);
			if (platform->driver->suspend && !platform->suspended) {
				platform->driver->suspend(cpu_dai);
				platform->suspended = 1;
			}
		}
	}

	
	for (i = 0; i < card->num_rtd; i++) {
		flush_delayed_work_sync(&card->rtd[i].delayed_work);
		card->rtd[i].codec->dapm.suspend_bias_level = card->rtd[i].codec->dapm.bias_level;
	}

	for (i = 0; i < card->num_rtd; i++) {
		struct snd_soc_dai_driver *driver = card->rtd[i].codec_dai->driver;

		if (card->rtd[i].dai_link->ignore_suspend ||
				card->rtd[i].dai_link->no_pcm)
			continue;

		if (driver->playback.stream_name != NULL)
			snd_soc_dapm_stream_event(&card->rtd[i], driver->playback.stream_name,
				SND_SOC_DAPM_STREAM_SUSPEND);

		if (driver->capture.stream_name != NULL)
			snd_soc_dapm_stream_event(&card->rtd[i], driver->capture.stream_name,
				SND_SOC_DAPM_STREAM_SUSPEND);
	}

	
	list_for_each_entry(codec, &card->codec_dev_list, card_list) {
		if (!codec->suspended && codec->driver->suspend) {
			switch (codec->dapm.bias_level) {
			case SND_SOC_BIAS_STANDBY:
			case SND_SOC_BIAS_OFF:
				codec->driver->suspend(codec);
				codec->suspended = 1;
				codec->cache_sync = 1;
				break;
			default:
				dev_dbg(codec->dev, "CODEC is on over suspend\n");
				break;
			}
		}
	}

	for (i = 0; i < card->num_rtd; i++) {
		struct snd_soc_dai *cpu_dai = card->rtd[i].cpu_dai;

		if (card->rtd[i].dai_link->ignore_suspend ||
				card->rtd[i].dai_link->no_pcm)
			continue;

		if (card->rtd[i].dai_link->dynamic)
			soc_dpcm_be_ac97_cpu_dai_suspend(&card->rtd[i]);
		else
			if (cpu_dai->driver->suspend && cpu_dai->driver->ac97_control)
				cpu_dai->driver->suspend(cpu_dai);
	}

	if (card->suspend_post)
		card->suspend_post(card);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_suspend);

static void soc_resume_deferred(struct work_struct *work)
{
	struct snd_soc_card *card =
			container_of(work, struct snd_soc_card, deferred_resume_work);
	struct snd_soc_codec *codec;
	int i;


	dev_dbg(card->dev, "starting resume work\n");

	
	snd_power_change_state(card->snd_card, SNDRV_CTL_POWER_D2);

	if (card->resume_pre)
		card->resume_pre(card);

	
	for (i = 0; i < card->num_rtd; i++) {
		struct snd_soc_dai *cpu_dai = card->rtd[i].cpu_dai;

		if (card->rtd[i].dai_link->ignore_suspend ||
				card->rtd[i].dai_link->no_pcm)
			continue;

		if (card->rtd[i].dai_link->dynamic)
			soc_dpcm_be_ac97_cpu_dai_resume(&card->rtd[i]);
		else
			if (cpu_dai->driver->resume && cpu_dai->driver->ac97_control)
				cpu_dai->driver->resume(cpu_dai);
	}

	list_for_each_entry(codec, &card->codec_dev_list, card_list) {
		if (codec->driver->resume && codec->suspended) {
			switch (codec->dapm.bias_level) {
			case SND_SOC_BIAS_STANDBY:
			case SND_SOC_BIAS_OFF:
				codec->driver->resume(codec);
				codec->suspended = 0;
				break;
			default:
				dev_dbg(codec->dev, "CODEC was on over suspend\n");
				break;
			}
		}
	}

	for (i = 0; i < card->num_rtd; i++) {
		struct snd_soc_dai_driver *driver = card->rtd[i].codec_dai->driver;

		if (card->rtd[i].dai_link->ignore_suspend ||
				card->rtd[i].dai_link->no_pcm)
			continue;

		if (driver->playback.stream_name != NULL)
			snd_soc_dapm_stream_event(&card->rtd[i], driver->playback.stream_name,
				SND_SOC_DAPM_STREAM_RESUME);

		if (driver->capture.stream_name != NULL)
			snd_soc_dapm_stream_event(&card->rtd[i], driver->capture.stream_name,
				SND_SOC_DAPM_STREAM_RESUME);
	}

	
	for (i = 0; i < card->num_rtd; i++) {
		struct snd_soc_dai *dai = card->rtd[i].codec_dai;
		struct snd_soc_dai_driver *drv = dai->driver;

		if (card->rtd[i].dai_link->ignore_suspend ||
				card->rtd[i].dai_link->no_pcm)
			continue;

		if (card->rtd[i].dai_link->dynamic)
			soc_dpcm_be_digital_mute(&card->rtd[i], 0);
		else {
			if (drv->ops->digital_mute && dai->playback_active)
				drv->ops->digital_mute(dai, 0);
		}
	}

	for (i = 0; i < card->num_rtd; i++) {
		struct snd_soc_dai *cpu_dai = card->rtd[i].cpu_dai;
		struct snd_soc_platform *platform = card->rtd[i].platform;

		if (card->rtd[i].dai_link->ignore_suspend ||
				card->rtd[i].dai_link->no_pcm)
			continue;

		if (card->rtd[i].dai_link->dynamic) {
			soc_dpcm_be_cpu_dai_resume(&card->rtd[i]);
			soc_dpcm_be_platform_resume(&card->rtd[i]);
		} else {
			if (cpu_dai->driver->resume && !cpu_dai->driver->ac97_control)
				cpu_dai->driver->resume(cpu_dai);
			if (platform->driver->resume && platform->suspended) {
				platform->driver->resume(cpu_dai);
				platform->suspended = 0;
			}
		}
	}

	if (card->resume_post)
		card->resume_post(card);

	dev_dbg(card->dev, "resume work completed\n");

	
	snd_power_change_state(card->snd_card, SNDRV_CTL_POWER_D0);
}

int snd_soc_resume(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	int i, ac97_control = 0;

	if (list_empty(&card->codec_dev_list))
		return 0;

	for (i = 0; i < card->num_rtd; i++) {
		struct snd_soc_dai *cpu_dai = card->rtd[i].cpu_dai;
		ac97_control |= cpu_dai->driver->ac97_control;
	}
	if (ac97_control) {
		dev_dbg(dev, "Resuming AC97 immediately\n");
		soc_resume_deferred(&card->deferred_resume_work);
	} else {
		dev_dbg(dev, "Scheduling resume work\n");
		if (!schedule_work(&card->deferred_resume_work))
			dev_err(dev, "resume work item may be lost\n");
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_resume);
#else
#define snd_soc_suspend NULL
#define snd_soc_resume NULL
#endif

static const struct snd_soc_dai_ops null_dai_ops = {
};

static int soc_bind_dai_link(struct snd_soc_card *card, int num)
{
	struct snd_soc_dai_link *dai_link = &card->dai_link[num];
	struct snd_soc_pcm_runtime *rtd = &card->rtd[num];
	struct snd_soc_codec *codec;
	struct snd_soc_platform *platform;
	struct snd_soc_dai *codec_dai, *cpu_dai;
	const char *platform_name;

	dev_dbg(card->dev, "binding %s at idx %d\n", dai_link->name, num);

	
	list_for_each_entry(cpu_dai, &dai_list, list) {
		if (dai_link->cpu_dai_of_node) {
			if (cpu_dai->dev->of_node != dai_link->cpu_dai_of_node)
				continue;
		} else {
			if (strcmp(cpu_dai->name, dai_link->cpu_dai_name))
				continue;
		}

		rtd->cpu_dai = cpu_dai;
	}

	if (!rtd->cpu_dai) {
		dev_dbg(card->dev, "CPU DAI %s not registered\n",
			dai_link->cpu_dai_name);
		return -EPROBE_DEFER;

	}

	
	list_for_each_entry(codec, &codec_list, list) {
		if (dai_link->codec_of_node) {
			if (codec->dev->of_node != dai_link->codec_of_node)
				continue;
		} else {
			if (strcmp(codec->name, dai_link->codec_name))
				continue;
		}

		rtd->codec = codec;

		list_for_each_entry(codec_dai, &dai_list, list) {
			if (codec->dev == codec_dai->dev &&
				!strcmp(codec_dai->name,
					dai_link->codec_dai_name)) {

				rtd->codec_dai = codec_dai;
			}
		}
		if (!rtd->codec_dai) {
			dev_dbg(card->dev, "CODEC DAI %s not registered\n",
				dai_link->codec_dai_name);
			return -EPROBE_DEFER;
		}

	}

	if (!rtd->codec) {
		dev_dbg(card->dev, "CODEC %s not registered\n",
			dai_link->codec_name);
		return -EPROBE_DEFER;
	}

	
	platform_name = dai_link->platform_name;
	if (!platform_name && !dai_link->platform_of_node)
		platform_name = "snd-soc-dummy";

	
	list_for_each_entry(platform, &platform_list, list) {
		if (dai_link->platform_of_node) {
			if (platform->dev->of_node !=
			    dai_link->platform_of_node)
				continue;
		} else {
			if (strcmp(platform->name, platform_name))
				continue;
		}

		rtd->platform = platform;
	}
	if (!rtd->platform) {
		dev_dbg(card->dev, "platform %s not registered\n",
			dai_link->platform_name);

		return -EPROBE_DEFER;
	}
	card->num_rtd++;

	return 0;

}

static void soc_remove_codec(struct snd_soc_codec *codec)
{
	int err;

	if (codec->driver->remove) {
		err = codec->driver->remove(codec);
		if (err < 0)
			dev_err(codec->dev,
				"asoc: failed to remove %s: %d\n",
				codec->name, err);
	}

	
	snd_soc_dapm_free(&codec->dapm);

	soc_cleanup_codec_debugfs(codec);
	codec->probed = 0;
	list_del(&codec->card_list);
	module_put(codec->dev->driver->owner);
}

static void soc_remove_dai_link(struct snd_soc_card *card, int num, int order)
{
	struct snd_soc_pcm_runtime *rtd = &card->rtd[num];
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_platform *platform = rtd->platform;
	struct snd_soc_dai *codec_dai = rtd->codec_dai, *cpu_dai = rtd->cpu_dai;
	int err;

	
	if (rtd->dev_registered) {
		device_remove_file(rtd->dev, &dev_attr_pmdown_time);
		device_remove_file(rtd->dev, &dev_attr_codec_reg);
		device_unregister(rtd->dev);
		rtd->dev_registered = 0;
	}

	
	if (codec_dai && codec_dai->probed &&
			codec_dai->driver->remove_order == order) {
		if (codec_dai->driver->remove) {
			err = codec_dai->driver->remove(codec_dai);
			if (err < 0)
				printk(KERN_ERR "asoc: failed to remove %s\n", codec_dai->name);
		}
		codec_dai->probed = 0;
		list_del(&codec_dai->card_list);
	}

	
	if (platform && platform->probed &&
			platform->driver->remove_order == order) {
		if (platform->driver->remove) {
			err = platform->driver->remove(platform);
			if (err < 0)
				printk(KERN_ERR "asoc: failed to remove %s\n", platform->name);
		}
		
		snd_soc_dapm_free(&platform->dapm);

		soc_cleanup_platform_debugfs(platform);
		platform->probed = 0;
		list_del(&platform->card_list);
		module_put(platform->dev->driver->owner);
	}

	
	if (codec && codec->probed &&
			codec->driver->remove_order == order)
		soc_remove_codec(codec);

	
	if (cpu_dai && cpu_dai->probed &&
			cpu_dai->driver->remove_order == order) {
		if (cpu_dai->driver->remove) {
			err = cpu_dai->driver->remove(cpu_dai);
			if (err < 0)
				printk(KERN_ERR "asoc: failed to remove %s\n", cpu_dai->name);
		}
		cpu_dai->probed = 0;
		list_del(&cpu_dai->card_list);
		module_put(cpu_dai->dev->driver->owner);
	}
}

static void soc_remove_dai_links(struct snd_soc_card *card)
{
	int dai, order;

	for (order = SND_SOC_COMP_ORDER_FIRST; order <= SND_SOC_COMP_ORDER_LAST;
			order++) {
		for (dai = 0; dai < card->num_rtd; dai++)
			soc_remove_dai_link(card, dai, order);
	}
	card->num_rtd = 0;
}

static void soc_set_name_prefix(struct snd_soc_card *card,
				struct snd_soc_codec *codec)
{
	int i;

	if (card->codec_conf == NULL)
		return;

	for (i = 0; i < card->num_configs; i++) {
		struct snd_soc_codec_conf *map = &card->codec_conf[i];
		if (map->dev_name && !strcmp(codec->name, map->dev_name)) {
			codec->name_prefix = map->name_prefix;
			break;
		}
	}
}

static int soc_probe_codec(struct snd_soc_card *card,
			   struct snd_soc_codec *codec)
{
	int ret = 0;
	const struct snd_soc_codec_driver *driver = codec->driver;

	codec->card = card;
	codec->dapm.card = card;
	soc_set_name_prefix(card, codec);

	if (!try_module_get(codec->dev->driver->owner))
		return -ENODEV;

	soc_init_codec_debugfs(codec);

	if (driver->dapm_widgets)
		snd_soc_dapm_new_controls(&codec->dapm, driver->dapm_widgets,
					  driver->num_dapm_widgets);

	codec->dapm.idle_bias_off = driver->idle_bias_off;

	if (driver->probe) {
		ret = driver->probe(codec);
		if (ret < 0) {
			dev_err(codec->dev,
				"asoc: failed to probe CODEC %s: %d\n",
				codec->name, ret);
			goto err_probe;
		}
	}

	if (driver->controls)
		snd_soc_add_codec_controls(codec, driver->controls,
				     driver->num_controls);
	if (driver->dapm_routes)
		snd_soc_dapm_add_routes(&codec->dapm, driver->dapm_routes,
					driver->num_dapm_routes);

	
	codec->probed = 1;
	list_add(&codec->card_list, &card->codec_dev_list);
	list_add(&codec->dapm.list, &card->dapm_list);

	return 0;

err_probe:
	soc_cleanup_codec_debugfs(codec);
	module_put(codec->dev->driver->owner);

	return ret;
}

static int soc_probe_platform(struct snd_soc_card *card,
			   struct snd_soc_platform *platform)
{
	int ret = 0;
	const struct snd_soc_platform_driver *driver = platform->driver;

	platform->card = card;
	platform->dapm.card = card;

	if (!try_module_get(platform->dev->driver->owner))
		return -ENODEV;

	soc_init_platform_debugfs(platform);

	if (driver->dapm_widgets)
		snd_soc_dapm_new_controls(&platform->dapm,
			driver->dapm_widgets, driver->num_dapm_widgets);

	if (driver->probe) {
		ret = driver->probe(platform);
		if (ret < 0) {
			dev_err(platform->dev,
				"asoc: failed to probe platform %s: %d\n",
				platform->name, ret);
			goto err_probe;
		}
	}

	if (driver->controls)
		snd_soc_add_platform_controls(platform, driver->controls,
				     driver->num_controls);
	if (driver->dapm_routes)
		snd_soc_dapm_add_routes(&platform->dapm, driver->dapm_routes,
					driver->num_dapm_routes);

	
	platform->probed = 1;
	list_add(&platform->card_list, &card->platform_dev_list);
	list_add(&platform->dapm.list, &card->dapm_list);

	return 0;

err_probe:
	module_put(platform->dev->driver->owner);

	return ret;
}

static void rtd_release(struct device *dev)
{
	kfree(dev);
}

static int soc_post_component_init(struct snd_soc_card *card,
				   struct snd_soc_codec *codec,
				   int num, int dailess)
{
	struct snd_soc_dai_link *dai_link = NULL;
	struct snd_soc_aux_dev *aux_dev = NULL;
	struct snd_soc_pcm_runtime *rtd;
	const char *temp, *name;
	int ret = 0;

	if (!dailess) {
		dai_link = &card->dai_link[num];
		rtd = &card->rtd[num];
		name = dai_link->name;
	} else {
		aux_dev = &card->aux_dev[num];
		rtd = &card->rtd_aux[num];
		name = aux_dev->name;
	}
	rtd->card = card;

	
	snd_soc_dapm_new_widgets(&codec->dapm);

	
	temp = codec->name_prefix;
	codec->name_prefix = NULL;

	
	if (!dailess && dai_link->init)
		ret = dai_link->init(rtd);
	else if (dailess && aux_dev->init)
		ret = aux_dev->init(&codec->dapm);
	if (ret < 0) {
		dev_err(card->dev, "asoc: failed to init %s: %d\n", name, ret);
		return ret;
	}
	codec->name_prefix = temp;

	
	rtd->codec = codec;

	rtd->dev = kzalloc(sizeof(struct device), GFP_KERNEL);
	if (!rtd->dev)
		return -ENOMEM;
	device_initialize(rtd->dev);
	rtd->dev->parent = card->dev;
	rtd->dev->release = rtd_release;
	rtd->dev->init_name = name;
	dev_set_drvdata(rtd->dev, rtd);
	mutex_init(&rtd->pcm_mutex);
	INIT_LIST_HEAD(&rtd->dpcm[SNDRV_PCM_STREAM_PLAYBACK].be_clients);
	INIT_LIST_HEAD(&rtd->dpcm[SNDRV_PCM_STREAM_CAPTURE].be_clients);
	INIT_LIST_HEAD(&rtd->dpcm[SNDRV_PCM_STREAM_PLAYBACK].fe_clients);
	INIT_LIST_HEAD(&rtd->dpcm[SNDRV_PCM_STREAM_CAPTURE].fe_clients);
	ret = device_add(rtd->dev);
	if (ret < 0) {
		dev_err(card->dev,
			"asoc: failed to register runtime device: %d\n", ret);
		return ret;
	}
	rtd->dev_registered = 1;

	
	ret = snd_soc_dapm_sys_add(rtd->dev);
	if (ret < 0)
		dev_err(codec->dev,
			"asoc: failed to add codec dapm sysfs entries: %d\n",
			ret);

	
	ret = device_create_file(rtd->dev, &dev_attr_codec_reg);
	if (ret < 0)
		dev_err(codec->dev,
			"asoc: failed to add codec sysfs files: %d\n", ret);

#ifdef CONFIG_DEBUG_FS
	
	if (!dai_link->dynamic)
		goto out;

	ret = soc_dpcm_debugfs_add(rtd);
	if (ret < 0)
		dev_err(rtd->dev, "asoc: failed to add dpcm sysfs entries: %d\n", ret);

out:
#endif
	return 0;
}

static int soc_probe_dai_link(struct snd_soc_card *card, int num, int order)
{
	struct snd_soc_dai_link *dai_link = &card->dai_link[num];
	struct snd_soc_pcm_runtime *rtd = &card->rtd[num];
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_platform *platform = rtd->platform;
	struct snd_soc_dai *codec_dai = rtd->codec_dai, *cpu_dai = rtd->cpu_dai;
	int ret;

	dev_dbg(card->dev, "probe %s dai link %d late %d\n",
			card->name, num, order);

	
	codec_dai->codec = codec;
	cpu_dai->platform = platform;
	codec_dai->card = card;
	cpu_dai->card = card;

	
	rtd->pmdown_time = pmdown_time;

	
	if (!cpu_dai->probed &&
			cpu_dai->driver->probe_order == order) {
		if (!try_module_get(cpu_dai->dev->driver->owner))
			return -ENODEV;

		if (cpu_dai->driver->probe) {
			ret = cpu_dai->driver->probe(cpu_dai);
			if (ret < 0) {
				printk(KERN_ERR "asoc: failed to probe CPU DAI %s\n",
						cpu_dai->name);
				module_put(cpu_dai->dev->driver->owner);
				return ret;
			}
		}
		cpu_dai->probed = 1;
		
		list_add(&cpu_dai->card_list, &card->dai_dev_list);
	}

	
	if (!codec->probed &&
			codec->driver->probe_order == order) {
		ret = soc_probe_codec(card, codec);
		if (ret < 0)
			return ret;
	}

	
	if (!platform->probed &&
			platform->driver->probe_order == order) {
		ret = soc_probe_platform(card, platform);
		if (ret < 0)
			return ret;
	}

	
	if (!codec_dai->probed && codec_dai->driver->probe_order == order) {
		if (codec_dai->driver->probe) {
			ret = codec_dai->driver->probe(codec_dai);
			if (ret < 0) {
				printk(KERN_ERR "asoc: failed to probe CODEC DAI %s\n",
						codec_dai->name);
				return ret;
			}
		}

		
		codec_dai->probed = 1;
		list_add(&codec_dai->card_list, &card->dai_dev_list);
	}

	
	if (order != SND_SOC_COMP_ORDER_LAST)
		return 0;

	ret = soc_post_component_init(card, codec, num, 0);
	if (ret)
		return ret;

	ret = device_create_file(rtd->dev, &dev_attr_pmdown_time);
	if (ret < 0)
		printk(KERN_WARNING "asoc: failed to add pmdown_time sysfs\n");

	if (cpu_dai->driver->compress_dai) {
		
		ret = soc_new_compress(rtd, num);
		if (ret < 0) {
		  printk(KERN_ERR "asoc: can't create compress %s\n",
					 dai_link->stream_name);
			return ret;
		}
	} else {
		
		ret = soc_new_pcm(rtd, num);
		if (ret < 0) {
		  printk(KERN_ERR "asoc: can't create pcm %s :%d\n",
			       dai_link->stream_name, ret);
			return ret;
		}
	}

	
	if (rtd->codec_dai->driver->ac97_control)
		snd_ac97_dev_add_pdata(codec->ac97, rtd->cpu_dai->ac97_pdata);

	return 0;
}

#ifdef CONFIG_SND_SOC_AC97_BUS
static int soc_register_ac97_dai_link(struct snd_soc_pcm_runtime *rtd)
{
	int ret;

	if (rtd->codec_dai->driver->ac97_control && !rtd->codec->ac97_registered) {
		if (!rtd->codec->ac97_created)
			return 0;

		ret = soc_ac97_dev_register(rtd->codec);
		if (ret < 0) {
			printk(KERN_ERR "asoc: AC97 device register failed\n");
			return ret;
		}

		rtd->codec->ac97_registered = 1;
	}
	return 0;
}

static void soc_unregister_ac97_dai_link(struct snd_soc_codec *codec)
{
	if (codec->ac97_registered) {
		soc_ac97_dev_unregister(codec);
		codec->ac97_registered = 0;
	}
}
#endif

static int soc_check_aux_dev(struct snd_soc_card *card, int num)
{
	struct snd_soc_aux_dev *aux_dev = &card->aux_dev[num];
	struct snd_soc_codec *codec;

	
	list_for_each_entry(codec, &codec_list, list) {
		if (!strcmp(codec->name, aux_dev->codec_name))
			return 0;
	}

	return -EPROBE_DEFER;
}

static int soc_probe_aux_dev(struct snd_soc_card *card, int num)
{
	struct snd_soc_aux_dev *aux_dev = &card->aux_dev[num];
	struct snd_soc_codec *codec;
	int ret = -ENODEV;

	
	list_for_each_entry(codec, &codec_list, list) {
		if (!strcmp(codec->name, aux_dev->codec_name)) {
			if (codec->probed) {
				dev_err(codec->dev,
					"asoc: codec already probed");
				ret = -EBUSY;
				goto out;
			}
			goto found;
		}
	}
	
	dev_err(card->dev, "asoc: codec %s not found", aux_dev->codec_name);
	return -EPROBE_DEFER;

found:
	ret = soc_probe_codec(card, codec);
	if (ret < 0)
		return ret;

	ret = soc_post_component_init(card, codec, num, 1);

out:
	return ret;
}

static void soc_remove_aux_dev(struct snd_soc_card *card, int num)
{
	struct snd_soc_pcm_runtime *rtd = &card->rtd_aux[num];
	struct snd_soc_codec *codec = rtd->codec;

	
	if (rtd->dev_registered) {
		device_remove_file(rtd->dev, &dev_attr_codec_reg);
		device_del(rtd->dev);
		rtd->dev_registered = 0;
	}

	if (codec && codec->probed)
		soc_remove_codec(codec);
}

static int snd_soc_init_codec_cache(struct snd_soc_codec *codec,
				    enum snd_soc_compress_type compress_type)
{
	int ret;

	if (codec->cache_init)
		return 0;

	
	if (compress_type && codec->compress_type != compress_type)
		codec->compress_type = compress_type;
	ret = snd_soc_cache_init(codec);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache compression type: %d\n",
			ret);
		return ret;
	}
	codec->cache_init = 1;
	return 0;
}

static void soc_init_dai_aif_channel_map(struct snd_soc_card *card,
		struct snd_soc_dai *dai, int stream)
{
	struct snd_soc_dapm_widget *w = NULL;
	const char *aif_name;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		aif_name = dai->driver->playback.aif_name;
	else
		aif_name = dai->driver->capture.aif_name;

	if (dai->codec) {
		struct snd_soc_codec *codec;

		list_for_each_entry(codec, &card->codec_dev_list, card_list) {
			w = snd_soc_get_codec_widget(card, codec, aif_name);
			if (w)
				break;
		}
	} else if (dai->platform) {
		struct snd_soc_platform *platform;

		list_for_each_entry(platform, &card->platform_dev_list, card_list) {
			w = snd_soc_get_platform_widget(card, platform, aif_name);
			if (w)
				break;
		}
	}

	if (w) {
		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
			dai->playback_aif = w;
		else
			dai->capture_aif = w;
	} else
		dev_err(dai->dev, "unable to find %s DAI AIF %s\n",
			stream ? "capture" : "playback", aif_name);

	dai->channel_map_instanciated = 1;
}

static int soc_is_dai_pcm(struct snd_soc_card *card, struct snd_soc_dai *dai)
{
	int i;

	for (i = 0; i < card->num_rtd; i++) {
		if (card->rtd[i].cpu_dai == dai && !card->rtd[i].dai_link->no_pcm)
			return 1;
	}
	return 0;
}

static void soc_init_card_aif_channel_map(struct snd_soc_card *card)
{
	struct snd_soc_dai *dai;

	list_for_each_entry(dai, &card->dai_dev_list, card_list) {

		if (!dai->driver->playback.aif_name && !dai->driver->capture.aif_name)
			continue;

		
		if (!soc_is_dai_pcm(card, dai))
			continue;

		
		if (dai->channel_map_instanciated)
			continue;

		
		dai->playback_channel_map =
			((1 << dai->driver->playback.channels_max) - 1)
			<< card->num_playback_channels;
		card->num_playback_channels += dai->driver->playback.channels_max;

		dai->capture_channel_map =
			((1 << dai->driver->capture.channels_max) - 1)
			<< card->num_capture_channels;
		card->num_capture_channels += dai->driver->capture.channels_max;

		if (dai->driver->playback.channels_max)
			soc_init_dai_aif_channel_map(card, dai, SNDRV_PCM_STREAM_PLAYBACK);
		if (dai->driver->capture.channels_max)
			soc_init_dai_aif_channel_map(card, dai, SNDRV_PCM_STREAM_CAPTURE);
	}
}


static int snd_soc_instantiate_card(struct snd_soc_card *card)
{
	struct snd_soc_codec *codec;
	struct snd_soc_codec_conf *codec_conf;
	enum snd_soc_compress_type compress_type;
	struct snd_soc_dai_link *dai_link;
	int ret, i, order;

	mutex_lock(&card->mutex);


	
	for (i = 0; i < card->num_links; i++) {
		ret = soc_bind_dai_link(card, i);
		if (ret != 0)
			goto base_error;
	}

	
	for (i = 0; i < card->num_aux_devs; i++) {
		ret = soc_check_aux_dev(card, i);
		if (ret != 0)
			goto base_error;
	}

	
	list_for_each_entry(codec, &codec_list, list) {
		if (codec->cache_init)
			continue;
		
		compress_type = 0;
		
		for (i = 0; i < card->num_configs; ++i) {
			codec_conf = &card->codec_conf[i];
			if (!strcmp(codec->name, codec_conf->dev_name)) {
				compress_type = codec_conf->compress_type;
				if (compress_type && compress_type
				    != codec->compress_type)
					break;
			}
		}
		ret = snd_soc_init_codec_cache(codec, compress_type);
		if (ret < 0)
			goto base_error;
	}

	
	ret = snd_card_create(SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
			card->owner, 0, &card->snd_card);
	if (ret < 0) {
		printk(KERN_ERR "asoc: can't create sound card for card %s\n",
			card->name);
		goto base_error;
	}
	card->snd_card->dev = card->dev;

	card->dapm.bias_level = SND_SOC_BIAS_OFF;
	card->dapm.dev = card->dev;
	card->dapm.card = card;
	list_add(&card->dapm.list, &card->dapm_list);

#ifdef CONFIG_DEBUG_FS
	snd_soc_dapm_debugfs_init(&card->dapm, card->debugfs_card_root);
#endif

#ifdef CONFIG_PM_SLEEP
	
	INIT_WORK(&card->deferred_resume_work, soc_resume_deferred);
#endif

	if (card->dapm_widgets)
		snd_soc_dapm_new_controls(&card->dapm, card->dapm_widgets,
					  card->num_dapm_widgets);

	
	if (card->probe) {
		ret = card->probe(card);
		if (ret < 0)
			goto card_probe_error;
	}

	
	for (order = SND_SOC_COMP_ORDER_FIRST; order <= SND_SOC_COMP_ORDER_LAST;
			order++) {
		for (i = 0; i < card->num_links; i++) {
			ret = soc_probe_dai_link(card, i, order);
			if (ret < 0) {
				pr_err("asoc: failed to instantiate card %s: %d\n",
			       card->name, ret);
				goto probe_dai_err;
			}
		}
	}

	for (i = 0; i < card->num_aux_devs; i++) {
		ret = soc_probe_aux_dev(card, i);
		if (ret < 0) {
			pr_err("asoc: failed to add auxiliary devices %s: %d\n",
			       card->name, ret);
			goto probe_aux_dev_err;
		}
	}

	if (card->controls)
		snd_soc_add_card_controls(card, card->controls, card->num_controls);

	if (card->dapm_routes)
		snd_soc_dapm_add_routes(&card->dapm, card->dapm_routes,
					card->num_dapm_routes);

	snd_soc_dapm_new_widgets(&card->dapm);

	for (i = 0; i < card->num_links; i++) {
		dai_link = &card->dai_link[i];

		if (dai_link->dai_fmt) {
			ret = snd_soc_dai_set_fmt(card->rtd[i].codec_dai,
						  dai_link->dai_fmt);
			if (ret != 0)
				dev_warn(card->rtd[i].codec_dai->dev,
					 "Failed to set DAI format: %d\n",
					 ret);

			ret = snd_soc_dai_set_fmt(card->rtd[i].cpu_dai,
						  dai_link->dai_fmt);
			if (ret != 0)
				dev_warn(card->rtd[i].cpu_dai->dev,
					 "Failed to set DAI format: %d\n",
					 ret);
		}
	}

	snprintf(card->snd_card->shortname, sizeof(card->snd_card->shortname),
		 "%s", card->name);
	snprintf(card->snd_card->longname, sizeof(card->snd_card->longname),
		 "%s", card->long_name ? card->long_name : card->name);
	snprintf(card->snd_card->driver, sizeof(card->snd_card->driver),
		 "%s", card->driver_name ? card->driver_name : card->name);
	for (i = 0; i < ARRAY_SIZE(card->snd_card->driver); i++) {
		switch (card->snd_card->driver[i]) {
		case '_':
		case '-':
		case '\0':
			break;
		default:
			if (!isalnum(card->snd_card->driver[i]))
				card->snd_card->driver[i] = '_';
			break;
		}
	}

	if (card->late_probe) {
		ret = card->late_probe(card);
		if (ret < 0) {
			dev_err(card->dev, "%s late_probe() failed: %d\n",
				card->name, ret);
			goto probe_aux_dev_err;
		}
	}

	snd_soc_dapm_new_widgets(&card->dapm);
	soc_init_card_aif_channel_map(card);

	if (card->fully_routed)
		list_for_each_entry(codec, &card->codec_dev_list, card_list)
			snd_soc_dapm_auto_nc_codec_pins(codec);

	ret = snd_card_register(card->snd_card);
	if (ret < 0) {
		printk(KERN_ERR "asoc: failed to register soundcard for %s\n", card->name);
		goto probe_aux_dev_err;
	}

#ifdef CONFIG_SND_SOC_AC97_BUS
	
	for (i = 0; i < card->num_rtd; i++) {
		ret = soc_register_ac97_dai_link(&card->rtd[i]);
		if (ret < 0) {
			printk(KERN_ERR "asoc: failed to register AC97 %s\n", card->name);
			while (--i >= 0)
				soc_unregister_ac97_dai_link(card->rtd[i].codec);
			goto probe_aux_dev_err;
		}
	}
#endif

	card->instantiated = 1;
	snd_soc_dapm_sync(&card->dapm);
	mutex_unlock(&card->mutex);
	return 0;

probe_aux_dev_err:
	for (i = 0; i < card->num_aux_devs; i++)
		soc_remove_aux_dev(card, i);

probe_dai_err:
	soc_remove_dai_links(card);

card_probe_error:
	if (card->remove)
		card->remove(card);

	snd_card_free(card->snd_card);
base_error:
	mutex_unlock(&card->mutex);
	return ret;
}

static int soc_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	int ret = 0;

	if (!card)
		return -EINVAL;

	
	card->dev = &pdev->dev;

	ret = snd_soc_register_card(card);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to register card\n");
		return ret;
	}

	return 0;
}

static int soc_cleanup_card_resources(struct snd_soc_card *card)
{
	int i;

	
	for (i = 0; i < card->num_rtd; i++) {
		struct snd_soc_pcm_runtime *rtd = &card->rtd[i];
		flush_delayed_work_sync(&rtd->delayed_work);
	}

	
	for (i = 0; i < card->num_aux_devs; i++)
		soc_remove_aux_dev(card, i);

	
	soc_remove_dai_links(card);

	soc_cleanup_card_debugfs(card);

	
	if (card->remove)
		card->remove(card);

	snd_soc_dapm_free(&card->dapm);

	kfree(card->rtd);
	snd_card_free(card->snd_card);
	return 0;

}

static int soc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
	return 0;
}

int snd_soc_poweroff(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	int i;

	if (!card->instantiated)
		return 0;

	for (i = 0; i < card->num_rtd; i++) {
		struct snd_soc_pcm_runtime *rtd = &card->rtd[i];
		flush_delayed_work_sync(&rtd->delayed_work);
	}

	snd_soc_dapm_shutdown(card);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_poweroff);

const struct dev_pm_ops snd_soc_pm_ops = {
	.suspend = snd_soc_suspend,
	.resume = snd_soc_resume,
	.poweroff = snd_soc_poweroff,
};
EXPORT_SYMBOL_GPL(snd_soc_pm_ops);

static struct platform_driver soc_driver = {
	.driver		= {
		.name		= "soc-audio",
		.owner		= THIS_MODULE,
		.pm		= &snd_soc_pm_ops,
	},
	.probe		= soc_probe,
	.remove		= soc_remove,
};

int snd_soc_codec_volatile_register(struct snd_soc_codec *codec,
				    unsigned int reg)
{
	if (codec->volatile_register)
		return codec->volatile_register(codec, reg);
	else
		return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_codec_volatile_register);

int snd_soc_codec_readable_register(struct snd_soc_codec *codec,
				    unsigned int reg)
{
	if (codec->readable_register)
		return codec->readable_register(codec, reg);
	else
		return 1;
}
EXPORT_SYMBOL_GPL(snd_soc_codec_readable_register);

int snd_soc_codec_writable_register(struct snd_soc_codec *codec,
				    unsigned int reg)
{
	if (codec->writable_register)
		return codec->writable_register(codec, reg);
	else
		return 1;
}
EXPORT_SYMBOL_GPL(snd_soc_codec_writable_register);

int snd_soc_platform_read(struct snd_soc_platform *platform,
					unsigned int reg)
{
	unsigned int ret;

	if (!platform->driver->read) {
		dev_err(platform->dev, "platform has no read back\n");
		return -1;
	}

	ret = platform->driver->read(platform, reg);
	dev_dbg(platform->dev, "read %x => %x\n", reg, ret);
	trace_snd_soc_preg_read(platform, reg, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_platform_read);

int snd_soc_platform_write(struct snd_soc_platform *platform,
					 unsigned int reg, unsigned int val)
{
	if (!platform->driver->write) {
		dev_err(platform->dev, "platform has no write back\n");
		return -1;
	}

	dev_dbg(platform->dev, "write %x = %x\n", reg, val);
	trace_snd_soc_preg_write(platform, reg, val);
	return platform->driver->write(platform, reg, val);
}
EXPORT_SYMBOL_GPL(snd_soc_platform_write);

int snd_soc_new_ac97_codec(struct snd_soc_codec *codec,
	struct snd_ac97_bus_ops *ops, int num)
{
	mutex_lock(&codec->mutex);

	codec->ac97 = kzalloc(sizeof(struct snd_ac97), GFP_KERNEL);
	if (codec->ac97 == NULL) {
		mutex_unlock(&codec->mutex);
		return -ENOMEM;
	}

	codec->ac97->bus = kzalloc(sizeof(struct snd_ac97_bus), GFP_KERNEL);
	if (codec->ac97->bus == NULL) {
		kfree(codec->ac97);
		codec->ac97 = NULL;
		mutex_unlock(&codec->mutex);
		return -ENOMEM;
	}

	codec->ac97->bus->ops = ops;
	codec->ac97->num = num;

	codec->ac97_created = 1;

	mutex_unlock(&codec->mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_new_ac97_codec);

void snd_soc_free_ac97_codec(struct snd_soc_codec *codec)
{
	mutex_lock(&codec->mutex);
#ifdef CONFIG_SND_SOC_AC97_BUS
	soc_unregister_ac97_dai_link(codec);
#endif
	kfree(codec->ac97->bus);
	kfree(codec->ac97);
	codec->ac97 = NULL;
	codec->ac97_created = 0;
	mutex_unlock(&codec->mutex);
}
EXPORT_SYMBOL_GPL(snd_soc_free_ac97_codec);

unsigned int snd_soc_read(struct snd_soc_codec *codec, unsigned int reg)
{
	unsigned int ret;

	if (unlikely(!snd_card_is_online_state(codec->card->snd_card))) {
		dev_err(codec->dev, "read 0x%02x while offline\n", reg);
		return -ENODEV;
	}
	ret = codec->read(codec, reg);
	dev_dbg(codec->dev, "read %x => %x\n", reg, ret);
	trace_snd_soc_reg_read(codec, reg, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_read);

unsigned int snd_soc_write(struct snd_soc_codec *codec,
			   unsigned int reg, unsigned int val)
{
	if (unlikely(!snd_card_is_online_state(codec->card->snd_card))) {
		dev_err(codec->dev, "write 0x%02x while offline\n", reg);
		return -ENODEV;
	}
	dev_dbg(codec->dev, "write %x = %x\n", reg, val);
	trace_snd_soc_reg_write(codec, reg, val);
	return codec->write(codec, reg, val);
}
EXPORT_SYMBOL_GPL(snd_soc_write);

unsigned int snd_soc_bulk_write_raw(struct snd_soc_codec *codec,
				    unsigned int reg, const void *data, size_t len)
{
	return codec->bulk_write_raw(codec, reg, data, len);
}
EXPORT_SYMBOL_GPL(snd_soc_bulk_write_raw);

int snd_soc_update_bits(struct snd_soc_codec *codec, unsigned short reg,
				unsigned int mask, unsigned int value)
{
	bool change;
	unsigned int old, new;
	int ret;

	if (codec->using_regmap) {
		ret = regmap_update_bits_check(codec->control_data, reg,
					       mask, value, &change);
	} else {
		ret = snd_soc_read(codec, reg);
		if (ret < 0)
			return ret;

		old = ret;
		new = (old & ~mask) | (value & mask);
		change = old != new;
		if (change)
			ret = snd_soc_write(codec, reg, new);
	}

	if (ret < 0)
		return ret;

	return change;
}
EXPORT_SYMBOL_GPL(snd_soc_update_bits);

int snd_soc_update_bits_locked(struct snd_soc_codec *codec,
			       unsigned short reg, unsigned int mask,
			       unsigned int value)
{
	int change;

	mutex_lock(&codec->mutex);
	change = snd_soc_update_bits(codec, reg, mask, value);
	mutex_unlock(&codec->mutex);

	return change;
}
EXPORT_SYMBOL_GPL(snd_soc_update_bits_locked);

int snd_soc_test_bits(struct snd_soc_codec *codec, unsigned short reg,
				unsigned int mask, unsigned int value)
{
	int change;
	unsigned int old, new;

	old = snd_soc_read(codec, reg);
	new = (old & ~mask) | value;
	change = old != new;

	return change;
}
EXPORT_SYMBOL_GPL(snd_soc_test_bits);

int snd_soc_set_runtime_hwparams(struct snd_pcm_substream *substream,
	const struct snd_pcm_hardware *hw)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	if (!runtime)
		return 0;
	runtime->hw.info = hw->info;
	runtime->hw.formats = hw->formats;
	runtime->hw.period_bytes_min = hw->period_bytes_min;
	runtime->hw.period_bytes_max = hw->period_bytes_max;
	runtime->hw.periods_min = hw->periods_min;
	runtime->hw.periods_max = hw->periods_max;
	runtime->hw.buffer_bytes_max = hw->buffer_bytes_max;
	runtime->hw.fifo_size = hw->fifo_size;
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_set_runtime_hwparams);

struct snd_kcontrol *snd_soc_cnew(const struct snd_kcontrol_new *_template,
				  void *data, char *long_name,
				  const char *prefix)
{
	struct snd_kcontrol_new template;
	struct snd_kcontrol *kcontrol;
	char *name = NULL;
	int name_len;

	memcpy(&template, _template, sizeof(template));
	template.index = 0;

	if (!long_name)
		long_name = template.name;

	if (prefix) {
		name_len = strlen(long_name) + strlen(prefix) + 2;
		name = kmalloc(name_len, GFP_KERNEL);
		if (!name)
			return NULL;

		snprintf(name, name_len, "%s %s", prefix, long_name);

		template.name = name;
	} else {
		template.name = long_name;
	}

	kcontrol = snd_ctl_new1(&template, data);

	kfree(name);

	return kcontrol;
}
EXPORT_SYMBOL_GPL(snd_soc_cnew);

static int snd_soc_add_controls(struct snd_card *card, struct device *dev,
	const struct snd_kcontrol_new *controls, int num_controls,
	const char *prefix, void *data)
{
	int err, i;

	for (i = 0; i < num_controls; i++) {
		const struct snd_kcontrol_new *control = &controls[i];
		err = snd_ctl_add(card, snd_soc_cnew(control, data,
						     control->name, prefix));
		if (err < 0) {
			dev_err(dev, "Failed to add %s: %d\n", control->name, err);
			return err;
		}
	}

	return 0;
}

int snd_soc_add_codec_controls(struct snd_soc_codec *codec,
	const struct snd_kcontrol_new *controls, int num_controls)
{
	struct snd_card *card = codec->card->snd_card;

	return snd_soc_add_controls(card, codec->dev, controls, num_controls,
			codec->name_prefix, codec);
}
EXPORT_SYMBOL_GPL(snd_soc_add_codec_controls);

int snd_soc_add_platform_controls(struct snd_soc_platform *platform,
	const struct snd_kcontrol_new *controls, int num_controls)
{
	struct snd_card *card = platform->card->snd_card;

	return snd_soc_add_controls(card, platform->dev, controls, num_controls,
			NULL, platform);
}
EXPORT_SYMBOL_GPL(snd_soc_add_platform_controls);

int snd_soc_add_card_controls(struct snd_soc_card *soc_card,
	const struct snd_kcontrol_new *controls, int num_controls)
{
	struct snd_card *card = soc_card->snd_card;

	return snd_soc_add_controls(card, soc_card->dev, controls, num_controls,
			NULL, soc_card);
}
EXPORT_SYMBOL_GPL(snd_soc_add_card_controls);

int snd_soc_add_dai_controls(struct snd_soc_dai *dai,
	const struct snd_kcontrol_new *controls, int num_controls)
{
	struct snd_card *card = dai->card->snd_card;

	return snd_soc_add_controls(card, dai->dev, controls, num_controls,
			NULL, dai);
}
EXPORT_SYMBOL_GPL(snd_soc_add_dai_controls);

int snd_soc_info_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = e->shift_l == e->shift_r ? 1 : 2;
	uinfo->value.enumerated.items = e->max;

	if (uinfo->value.enumerated.item > e->max - 1)
		uinfo->value.enumerated.item = e->max - 1;
	memset(uinfo->value.enumerated.name, 0, sizeof(uinfo->value.enumerated.name));
	strncpy(uinfo->value.enumerated.name,
		snd_soc_get_enum_text(e, uinfo->value.enumerated.item), sizeof(uinfo->value.enumerated.name)-1);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_info_enum_double);

int snd_soc_get_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int val, bitmask;

	for (bitmask = 1; bitmask < e->max; bitmask <<= 1)
		;
	val = snd_soc_read(codec, e->reg);
	ucontrol->value.enumerated.item[0]
		= (val >> e->shift_l) & (bitmask - 1);
	if (e->shift_l != e->shift_r)
		ucontrol->value.enumerated.item[1] =
			(val >> e->shift_r) & (bitmask - 1);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_get_enum_double);

int snd_soc_put_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int val;
	unsigned int mask, bitmask;

	for (bitmask = 1; bitmask < e->max; bitmask <<= 1)
		;
	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;
	val = ucontrol->value.enumerated.item[0] << e->shift_l;
	mask = (bitmask - 1) << e->shift_l;
	if (e->shift_l != e->shift_r) {
		if (ucontrol->value.enumerated.item[1] > e->max - 1)
			return -EINVAL;
		val |= ucontrol->value.enumerated.item[1] << e->shift_r;
		mask |= (bitmask - 1) << e->shift_r;
	}

	return snd_soc_update_bits_locked(codec, e->reg, mask, val);
}
EXPORT_SYMBOL_GPL(snd_soc_put_enum_double);

int snd_soc_get_value_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int reg_val, val, mux;

	reg_val = snd_soc_read(codec, e->reg);
	val = (reg_val >> e->shift_l) & e->mask;
	for (mux = 0; mux < e->max; mux++) {
		if (val == e->values[mux])
			break;
	}
	ucontrol->value.enumerated.item[0] = mux;
	if (e->shift_l != e->shift_r) {
		val = (reg_val >> e->shift_r) & e->mask;
		for (mux = 0; mux < e->max; mux++) {
			if (val == e->values[mux])
				break;
		}
		ucontrol->value.enumerated.item[1] = mux;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_get_value_enum_double);

int snd_soc_put_value_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int val;
	unsigned int mask;

	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;
	val = e->values[ucontrol->value.enumerated.item[0]] << e->shift_l;
	mask = e->mask << e->shift_l;
	if (e->shift_l != e->shift_r) {
		if (ucontrol->value.enumerated.item[1] > e->max - 1)
			return -EINVAL;
		val |= e->values[ucontrol->value.enumerated.item[1]] << e->shift_r;
		mask |= e->mask << e->shift_r;
	}

	return snd_soc_update_bits_locked(codec, e->reg, mask, val);
}
EXPORT_SYMBOL_GPL(snd_soc_put_value_enum_double);

int snd_soc_info_enum_ext(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = e->max;

	if (uinfo->value.enumerated.item > e->max - 1)
		uinfo->value.enumerated.item = e->max - 1;
	memset(uinfo->value.enumerated.name, 0, sizeof(uinfo->value.enumerated.name));
	strncpy(uinfo->value.enumerated.name,
		snd_soc_get_enum_text(e, uinfo->value.enumerated.item), sizeof(uinfo->value.enumerated.name)-1);
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_info_enum_ext);

int snd_soc_info_volsw_ext(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	int max = kcontrol->private_value;

	if (max == 1 && !strstr(kcontrol->id.name, " Volume"))
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	else
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = max;
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_info_volsw_ext);

int snd_soc_info_multi_ext(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_multi_mixer_control *mc =
		(struct soc_multi_mixer_control *)kcontrol->private_value;
	int platform_max;

	if (!mc->platform_max)
		mc->platform_max = mc->max;
	platform_max = mc->platform_max;

	if (platform_max == 1 && !strnstr(kcontrol->id.name, " Volume", 30))
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	else
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = mc->count;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = platform_max;
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_info_multi_ext);

int snd_soc_info_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int platform_max;

	if (!mc->platform_max)
		mc->platform_max = mc->max;
	platform_max = mc->platform_max;

	if (platform_max == 1 && !strstr(kcontrol->id.name, " Volume"))
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	else
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = snd_soc_volsw_is_stereo(mc) ? 2 : 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = platform_max;
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_info_volsw);

int snd_soc_get_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;

	ucontrol->value.integer.value[0] =
		(snd_soc_read(codec, reg) >> shift) & mask;
	if (invert)
		ucontrol->value.integer.value[0] =
			max - ucontrol->value.integer.value[0];

	if (snd_soc_volsw_is_stereo(mc)) {
		if (reg == reg2)
			ucontrol->value.integer.value[1] =
				(snd_soc_read(codec, reg) >> rshift) & mask;
		else
			ucontrol->value.integer.value[1] =
				(snd_soc_read(codec, reg2) >> shift) & mask;
		if (invert)
			ucontrol->value.integer.value[1] =
				max - ucontrol->value.integer.value[1];
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_get_volsw);

int snd_soc_put_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int reg2 = mc->rreg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int max = mc->max;
	unsigned int mask = (1 << fls(max)) - 1;
	unsigned int invert = mc->invert;
	int err;
	bool type_2r = 0;
	unsigned int val2 = 0;
	unsigned int val, val_mask;

	val = (ucontrol->value.integer.value[0] & mask);
	if (invert)
		val = max - val;
	val_mask = mask << shift;
	val = val << shift;
	if (snd_soc_volsw_is_stereo(mc)) {
		val2 = (ucontrol->value.integer.value[1] & mask);
		if (invert)
			val2 = max - val2;
		if (reg == reg2) {
			val_mask |= mask << rshift;
			val |= val2 << rshift;
		} else {
			val2 = val2 << shift;
			type_2r = 1;
		}
	}
	err = snd_soc_update_bits_locked(codec, reg, val_mask, val);
	if (err < 0)
		return err;

	if (type_2r)
		err = snd_soc_update_bits_locked(codec, reg2, val_mask, val2);

	return err;
}
EXPORT_SYMBOL_GPL(snd_soc_put_volsw);

int snd_soc_info_volsw_s8(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int platform_max;
	int min = mc->min;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;

	if (!mc->platform_max)
		mc->platform_max = mc->max;
	platform_max = mc->platform_max;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = shift == rshift ? 1 : 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = platform_max - min;
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_info_volsw_s8);

int snd_soc_get_volsw_s8(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int min = mc->min;
	int val = snd_soc_read(codec, reg);

	ucontrol->value.integer.value[0] =
		((signed char)((val >> shift) & 0xff))-min;
	if (shift != rshift)
		ucontrol->value.integer.value[1] =
			((signed char)((val >> rshift) & 0xff))-min;
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_get_volsw_s8);

int snd_soc_put_volsw_s8(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int rshift = mc->rshift;
	int min = mc->min;
	unsigned int val, val2, val_mask;

	val = ((ucontrol->value.integer.value[0]+min) & 0xff) << shift;
	val_mask = 0xff << shift;
	if (shift != rshift) {
		val2 = (ucontrol->value.integer.value[1]+min) & 0xff;
		val |= val2 << rshift;
		val_mask |= 0xff << rshift;
	}

	return snd_soc_update_bits_locked(codec, reg, val_mask, val);
}
EXPORT_SYMBOL_GPL(snd_soc_put_volsw_s8);

int snd_soc_limit_volume(struct snd_soc_codec *codec,
	const char *name, int max)
{
	struct snd_card *card = codec->card->snd_card;
	struct snd_kcontrol *kctl;
	struct soc_mixer_control *mc;
	int found = 0;
	int ret = -EINVAL;

	
	if (unlikely(!name || max <= 0))
		return -EINVAL;

	list_for_each_entry(kctl, &card->controls, list) {
		if (!strncmp(kctl->id.name, name, sizeof(kctl->id.name))) {
			found = 1;
			break;
		}
	}
	if (found) {
		mc = (struct soc_mixer_control *)kctl->private_value;
		if (max <= mc->max) {
			mc->platform_max = max;
			ret = 0;
		}
	}
	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_limit_volume);

int snd_soc_info_volsw_2r_sx(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	int max = mc->max;
	int min = mc->min;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = max-min;

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_info_volsw_2r_sx);

int snd_soc_get_volsw_2r_sx(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int mask = (1<<mc->shift)-1;
	int min = mc->min;
	int val = snd_soc_read(codec, mc->reg) & mask;
	int valr = snd_soc_read(codec, mc->rreg) & mask;

	ucontrol->value.integer.value[0] = ((val & 0xff)-min) & mask;
	ucontrol->value.integer.value[1] = ((valr & 0xff)-min) & mask;
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_get_volsw_2r_sx);

int snd_soc_put_volsw_2r_sx(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int mask = (1<<mc->shift)-1;
	int min = mc->min;
	int ret;
	unsigned int val, valr, oval, ovalr;

	val = ((ucontrol->value.integer.value[0]+min) & 0xff);
	val &= mask;
	valr = ((ucontrol->value.integer.value[1]+min) & 0xff);
	valr &= mask;

	oval = snd_soc_read(codec, mc->reg) & mask;
	ovalr = snd_soc_read(codec, mc->rreg) & mask;

	ret = 0;
	if (oval != val) {
		ret = snd_soc_write(codec, mc->reg, val);
		if (ret < 0)
			return ret;
	}
	if (ovalr != valr) {
		ret = snd_soc_write(codec, mc->rreg, valr);
		if (ret < 0)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_put_volsw_2r_sx);

int snd_soc_dai_set_sysclk(struct snd_soc_dai *dai, int clk_id,
	unsigned int freq, int dir)
{
	if (dai->driver && dai->driver->ops->set_sysclk)
		return dai->driver->ops->set_sysclk(dai, clk_id, freq, dir);
	else if (dai->codec && dai->codec->driver->set_sysclk)
		return dai->codec->driver->set_sysclk(dai->codec, clk_id, 0,
						      freq, dir);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_set_sysclk);

int snd_soc_codec_set_sysclk(struct snd_soc_codec *codec, int clk_id,
			     int source, unsigned int freq, int dir)
{
	if (codec->driver->set_sysclk)
		return codec->driver->set_sysclk(codec, clk_id, source,
						 freq, dir);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_codec_set_sysclk);

int snd_soc_dai_set_clkdiv(struct snd_soc_dai *dai,
	int div_id, int div)
{
	if (dai->driver && dai->driver->ops->set_clkdiv)
		return dai->driver->ops->set_clkdiv(dai, div_id, div);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_set_clkdiv);

int snd_soc_dai_set_pll(struct snd_soc_dai *dai, int pll_id, int source,
	unsigned int freq_in, unsigned int freq_out)
{
	if (dai->driver && dai->driver->ops->set_pll)
		return dai->driver->ops->set_pll(dai, pll_id, source,
					 freq_in, freq_out);
	else if (dai->codec && dai->codec->driver->set_pll)
		return dai->codec->driver->set_pll(dai->codec, pll_id, source,
						   freq_in, freq_out);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_set_pll);

int snd_soc_codec_set_pll(struct snd_soc_codec *codec, int pll_id, int source,
			  unsigned int freq_in, unsigned int freq_out)
{
	if (codec->driver->set_pll)
		return codec->driver->set_pll(codec, pll_id, source,
					      freq_in, freq_out);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_codec_set_pll);

int snd_soc_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if (dai->driver && dai->driver->ops->set_fmt)
		return dai->driver->ops->set_fmt(dai, fmt);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_set_fmt);

int snd_soc_dai_set_tdm_slot(struct snd_soc_dai *dai,
	unsigned int tx_mask, unsigned int rx_mask, int slots, int slot_width)
{
	if (dai->driver && dai->driver->ops->set_tdm_slot)
		return dai->driver->ops->set_tdm_slot(dai, tx_mask, rx_mask,
				slots, slot_width);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_set_tdm_slot);

int snd_soc_dai_set_channel_map(struct snd_soc_dai *dai,
	unsigned int tx_num, unsigned int *tx_slot,
	unsigned int rx_num, unsigned int *rx_slot)
{
	if (dai->driver && dai->driver->ops->set_channel_map)
		return dai->driver->ops->set_channel_map(dai, tx_num, tx_slot,
			rx_num, rx_slot);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_set_channel_map);

int snd_soc_dai_get_channel_map(struct snd_soc_dai *dai,
	unsigned int *tx_num, unsigned int *tx_slot,
	unsigned int *rx_num, unsigned int *rx_slot)
{
	if (dai->driver && dai->driver->ops->get_channel_map)
		return dai->driver->ops->get_channel_map(dai, tx_num, tx_slot,
			rx_num, rx_slot);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_get_channel_map);
int snd_soc_dai_set_tristate(struct snd_soc_dai *dai, int tristate)
{
	if (dai->driver && dai->driver->ops->set_tristate)
		return dai->driver->ops->set_tristate(dai, tristate);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_set_tristate);

int snd_soc_dai_digital_mute(struct snd_soc_dai *dai, int mute)
{
	if (dai->driver && dai->driver->ops->digital_mute)
		return dai->driver->ops->digital_mute(dai, mute);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_digital_mute);

int snd_soc_register_card(struct snd_soc_card *card)
{
	int i;
	int ret = 0;

	if (!card->name || !card->dev)
		return -EINVAL;

	for (i = 0; i < card->num_links; i++) {
		struct snd_soc_dai_link *link = &card->dai_link[i];

		if (!!link->codec_name == !!link->codec_of_node) {
			dev_err(card->dev,
				"Neither/both codec name/of_node are set for %s\n",
				link->name);
			return -EINVAL;
		}

		if (link->platform_name && link->platform_of_node) {
			dev_err(card->dev,
				"Both platform name/of_node are set for %s\n", link->name);
			return -EINVAL;
		}

		if (!!link->cpu_dai_name == !!link->cpu_dai_of_node) {
			dev_err(card->dev,
				"Neither/both cpu_dai name/of_node are set for %s\n",
				link->name);
			return -EINVAL;
		}
	}

	dev_set_drvdata(card->dev, card);

	snd_soc_initialize_card_lists(card);

	soc_init_card_debugfs(card);

	card->rtd = kzalloc(sizeof(struct snd_soc_pcm_runtime) *
			    (card->num_links + card->num_aux_devs),
			    GFP_KERNEL);
	if (card->rtd == NULL)
		return -ENOMEM;
	card->num_rtd = 0;
	card->rtd_aux = &card->rtd[card->num_links];

	for (i = 0; i < card->num_links; i++)
		card->rtd[i].dai_link = &card->dai_link[i];

	INIT_LIST_HEAD(&card->list);
	INIT_LIST_HEAD(&card->dapm_dirty);
	card->instantiated = 0;
	mutex_init(&card->mutex);
	mutex_init(&card->dpcm_mutex);
	mutex_init(&card->dapm_power_mutex);
	mutex_init(&card->dapm_mutex);
	ret = snd_soc_instantiate_card(card);
	if (ret != 0) {
		soc_cleanup_card_debugfs(card);
		if (card->rtd)
			kfree(card->rtd);
	}


	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_register_card);

int snd_soc_unregister_card(struct snd_soc_card *card)
{
	if (card->instantiated)
		soc_cleanup_card_resources(card);
	dev_dbg(card->dev, "Unregistered card '%s'\n", card->name);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_unregister_card);

static char *fmt_single_name(struct device *dev, int *id)
{
	char *found, name[NAME_SIZE];
	int id1, id2;

	if (dev_name(dev) == NULL)
		return NULL;

	strlcpy(name, dev_name(dev), NAME_SIZE);

	
	found = strstr(name, dev->driver->name);
	if (found) {
		
		if (sscanf(&found[strlen(dev->driver->name)], ".%d", id) == 1) {

			
			if (*id == -1)
				found[strlen(dev->driver->name)] = '\0';
		}

	} else {
		
		if (sscanf(name, "%x-%x", &id1, &id2) == 2) {
			char tmp[NAME_SIZE];

			
			*id = ((id1 & 0xffff) << 16) + id2;

			
			snprintf(tmp, NAME_SIZE, "%s.%s", dev->driver->name, name);
			strlcpy(name, tmp, NAME_SIZE);
		} else
			*id = 0;
	}

	return kstrdup(name, GFP_KERNEL);
}

static inline char *fmt_multiple_name(struct device *dev,
		struct snd_soc_dai_driver *dai_drv)
{
	if (dai_drv->name == NULL) {
		printk(KERN_ERR "asoc: error - multiple DAI %s registered with no name\n",
				dev_name(dev));
		return NULL;
	}

	return kstrdup(dai_drv->name, GFP_KERNEL);
}

int snd_soc_register_dai(struct device *dev,
		struct snd_soc_dai_driver *dai_drv)
{
	struct snd_soc_dai *dai;

	dev_dbg(dev, "dai register %s\n", dev_name(dev));

	dai = kzalloc(sizeof(struct snd_soc_dai), GFP_KERNEL);
	if (dai == NULL)
		return -ENOMEM;

	
	dai->name = fmt_single_name(dev, &dai->id);
	if (dai->name == NULL) {
		kfree(dai);
		return -ENOMEM;
	}

	dai->dev = dev;
	dai->driver = dai_drv;
	if (!dai->driver->ops)
		dai->driver->ops = &null_dai_ops;

	mutex_lock(&client_mutex);
	list_add(&dai->list, &dai_list);
	mutex_unlock(&client_mutex);

	pr_debug("Registered DAI '%s'\n", dai->name);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_register_dai);

void snd_soc_unregister_dai(struct device *dev)
{
	struct snd_soc_dai *dai;

	list_for_each_entry(dai, &dai_list, list) {
		if (dev == dai->dev)
			goto found;
	}
	return;

found:
	mutex_lock(&client_mutex);
	list_del(&dai->list);
	mutex_unlock(&client_mutex);

	pr_debug("Unregistered DAI '%s'\n", dai->name);
	kfree(dai->name);
	kfree(dai);
}
EXPORT_SYMBOL_GPL(snd_soc_unregister_dai);

int snd_soc_register_dais(struct device *dev,
		struct snd_soc_dai_driver *dai_drv, size_t count)
{
	struct snd_soc_dai *dai;
	int i, ret = 0;

	dev_dbg(dev, "dai register %s #%Zu\n", dev_name(dev), count);

	for (i = 0; i < count; i++) {

		dai = kzalloc(sizeof(struct snd_soc_dai), GFP_KERNEL);
		if (dai == NULL) {
			ret = -ENOMEM;
			goto err;
		}

		
		dai->name = fmt_multiple_name(dev, &dai_drv[i]);
		if (dai->name == NULL) {
			kfree(dai);
			ret = -EINVAL;
			goto err;
		}

		dai->dev = dev;
		dai->driver = &dai_drv[i];

		if (dai->driver->id)
			dai->id = dai->driver->id;
		else
			dai->id = i;
		if (!dai->driver->ops)
			dai->driver->ops = &null_dai_ops;

		mutex_lock(&client_mutex);
		list_add(&dai->list, &dai_list);
		mutex_unlock(&client_mutex);

		pr_debug("Registered DAI '%s'\n", dai->name);
	}

	return 0;

err:
	for (i--; i >= 0; i--)
		snd_soc_unregister_dai(dev);

	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_register_dais);

void snd_soc_unregister_dais(struct device *dev, size_t count)
{
	int i;

	for (i = 0; i < count; i++)
		snd_soc_unregister_dai(dev);
}
EXPORT_SYMBOL_GPL(snd_soc_unregister_dais);

int snd_soc_register_platform(struct device *dev,
		struct snd_soc_platform_driver *platform_drv)
{
	struct snd_soc_platform *platform;

	dev_dbg(dev, "platform register %s\n", dev_name(dev));

	platform = kzalloc(sizeof(struct snd_soc_platform), GFP_KERNEL);
	if (platform == NULL)
		return -ENOMEM;

	
	platform->name = fmt_single_name(dev, &platform->id);
	if (platform->name == NULL) {
		kfree(platform);
		return -ENOMEM;
	}

	platform->dev = dev;
	platform->driver = platform_drv;
	platform->dapm.dev = dev;
	platform->dapm.platform = platform;
	platform->dapm.stream_event = platform_drv->stream_event;

	mutex_lock(&client_mutex);
	list_add(&platform->list, &platform_list);
	mutex_unlock(&client_mutex);

	pr_debug("Registered platform '%s'\n", platform->name);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_register_platform);

void snd_soc_unregister_platform(struct device *dev)
{
	struct snd_soc_platform *platform;

	list_for_each_entry(platform, &platform_list, list) {
		if (dev == platform->dev)
			goto found;
	}
	return;

found:
	mutex_lock(&client_mutex);
	list_del(&platform->list);
	mutex_unlock(&client_mutex);

	pr_debug("Unregistered platform '%s'\n", platform->name);
	kfree(platform->name);
	kfree(platform);
}
EXPORT_SYMBOL_GPL(snd_soc_unregister_platform);

static u64 codec_format_map[] = {
	SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE,
	SNDRV_PCM_FMTBIT_U16_LE | SNDRV_PCM_FMTBIT_U16_BE,
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_BE,
	SNDRV_PCM_FMTBIT_U24_LE | SNDRV_PCM_FMTBIT_U24_BE,
	SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S32_BE,
	SNDRV_PCM_FMTBIT_U32_LE | SNDRV_PCM_FMTBIT_U32_BE,
	SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_U24_3BE,
	SNDRV_PCM_FMTBIT_U24_3LE | SNDRV_PCM_FMTBIT_U24_3BE,
	SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE,
	SNDRV_PCM_FMTBIT_U20_3LE | SNDRV_PCM_FMTBIT_U20_3BE,
	SNDRV_PCM_FMTBIT_S18_3LE | SNDRV_PCM_FMTBIT_S18_3BE,
	SNDRV_PCM_FMTBIT_U18_3LE | SNDRV_PCM_FMTBIT_U18_3BE,
	SNDRV_PCM_FMTBIT_FLOAT_LE | SNDRV_PCM_FMTBIT_FLOAT_BE,
	SNDRV_PCM_FMTBIT_FLOAT64_LE | SNDRV_PCM_FMTBIT_FLOAT64_BE,
	SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE
	| SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_BE,
};

static void fixup_codec_formats(struct snd_soc_pcm_stream *stream)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(codec_format_map); i++)
		if (stream->formats & codec_format_map[i])
			stream->formats |= codec_format_map[i];
}

void snd_soc_card_change_online_state(struct snd_soc_card *soc_card, int online)
{
	snd_card_change_online_state(soc_card->snd_card, online);
}
EXPORT_SYMBOL(snd_soc_card_change_online_state);

int snd_soc_register_codec(struct device *dev,
			   const struct snd_soc_codec_driver *codec_drv,
			   struct snd_soc_dai_driver *dai_drv,
			   int num_dai)
{
	size_t reg_size;
	struct snd_soc_codec *codec;
	int ret, i;

	dev_dbg(dev, "codec register %s\n", dev_name(dev));

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	
	codec->name = fmt_single_name(dev, &codec->id);
	if (codec->name == NULL) {
		kfree(codec);
		return -ENOMEM;
	}

	if (codec_drv->compress_type)
		codec->compress_type = codec_drv->compress_type;
	else
		codec->compress_type = SND_SOC_FLAT_COMPRESSION;

	codec->write = codec_drv->write;
	codec->read = codec_drv->read;
	codec->volatile_register = codec_drv->volatile_register;
	codec->readable_register = codec_drv->readable_register;
	codec->writable_register = codec_drv->writable_register;
	codec->dapm.bias_level = SND_SOC_BIAS_OFF;
	codec->dapm.dev = dev;
	codec->dapm.codec = codec;
	codec->dapm.seq_notifier = codec_drv->seq_notifier;
	codec->dapm.stream_event = codec_drv->stream_event;
	codec->dev = dev;
	codec->driver = codec_drv;
	codec->num_dai = num_dai;
	mutex_init(&codec->mutex);

	
	if (codec_drv->reg_cache_size && codec_drv->reg_word_size) {
		reg_size = codec_drv->reg_cache_size * codec_drv->reg_word_size;
		codec->reg_size = reg_size;
		if (codec_drv->reg_cache_default) {
			codec->reg_def_copy = kmemdup(codec_drv->reg_cache_default,
						      reg_size, GFP_KERNEL);
			if (!codec->reg_def_copy) {
				ret = -ENOMEM;
				goto fail;
			}
		}
	}

	if (codec_drv->reg_access_size && codec_drv->reg_access_default) {
		if (!codec->volatile_register)
			codec->volatile_register = snd_soc_default_volatile_register;
		if (!codec->readable_register)
			codec->readable_register = snd_soc_default_readable_register;
		if (!codec->writable_register)
			codec->writable_register = snd_soc_default_writable_register;
	}

	for (i = 0; i < num_dai; i++) {
		fixup_codec_formats(&dai_drv[i].playback);
		fixup_codec_formats(&dai_drv[i].capture);
	}

	
	if (num_dai) {
		ret = snd_soc_register_dais(dev, dai_drv, num_dai);
		if (ret < 0)
			goto fail;
	}

	mutex_lock(&client_mutex);
	list_add(&codec->list, &codec_list);
	mutex_unlock(&client_mutex);

	pr_debug("Registered codec '%s'\n", codec->name);
	return 0;

fail:
	kfree(codec->reg_def_copy);
	codec->reg_def_copy = NULL;
	kfree(codec->name);
	kfree(codec);
	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_register_codec);

void snd_soc_unregister_codec(struct device *dev)
{
	struct snd_soc_codec *codec;
	int i;

	list_for_each_entry(codec, &codec_list, list) {
		if (dev == codec->dev)
			goto found;
	}
	return;

found:
	if (codec->num_dai)
		for (i = 0; i < codec->num_dai; i++)
			snd_soc_unregister_dai(dev);

	mutex_lock(&client_mutex);
	list_del(&codec->list);
	mutex_unlock(&client_mutex);

	pr_debug("Unregistered codec '%s'\n", codec->name);

	snd_soc_cache_exit(codec);
	kfree(codec->reg_def_copy);
	kfree(codec->name);
	kfree(codec);
}
EXPORT_SYMBOL_GPL(snd_soc_unregister_codec);

int snd_soc_of_parse_card_name(struct snd_soc_card *card,
			       const char *propname)
{
	struct device_node *np = card->dev->of_node;
	int ret;

	ret = of_property_read_string_index(np, propname, 0, &card->name);
	if (ret < 0 && ret != -EINVAL) {
		dev_err(card->dev,
			"Property '%s' could not be read: %d\n",
			propname, ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_of_parse_card_name);

int snd_soc_of_parse_audio_routing(struct snd_soc_card *card,
				   const char *propname)
{
	struct device_node *np = card->dev->of_node;
	int num_routes;
	struct snd_soc_dapm_route *routes;
	int i, ret;

	num_routes = of_property_count_strings(np, propname);
	if (num_routes < 0 || num_routes & 1) {
		dev_err(card->dev,
		     "Property '%s' does not exist or its length is not even\n",
		     propname);
		return -EINVAL;
	}
	num_routes /= 2;
	if (!num_routes) {
		dev_err(card->dev,
			"Property '%s's length is zero\n",
			propname);
		return -EINVAL;
	}

	routes = devm_kzalloc(card->dev, num_routes * sizeof(*routes),
			      GFP_KERNEL);
	if (!routes) {
		dev_err(card->dev,
			"Could not allocate DAPM route table\n");
		return -EINVAL;
	}

	for (i = 0; i < num_routes; i++) {
		ret = of_property_read_string_index(np, propname,
			2 * i, &routes[i].sink);
		if (ret) {
			dev_err(card->dev,
				"Property '%s' index %d could not be read: %d\n",
				propname, 2 * i, ret);
			return -EINVAL;
		}
		ret = of_property_read_string_index(np, propname,
			(2 * i) + 1, &routes[i].source);
		if (ret) {
			dev_err(card->dev,
				"Property '%s' index %d could not be read: %d\n",
				propname, (2 * i) + 1, ret);
			return -EINVAL;
		}
	}

	card->num_dapm_routes = num_routes;
	card->dapm_routes = routes;

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_of_parse_audio_routing);

static int __init snd_soc_init(void)
{
#ifdef CONFIG_DEBUG_FS
	snd_soc_debugfs_root = debugfs_create_dir("asoc", NULL);
	if (IS_ERR(snd_soc_debugfs_root) || !snd_soc_debugfs_root) {
		printk(KERN_WARNING
		       "ASoC: Failed to create debugfs directory\n");
		snd_soc_debugfs_root = NULL;
	}

	if (!debugfs_create_file("codecs", 0444, snd_soc_debugfs_root, NULL,
				 &codec_list_fops))
		pr_warn("ASoC: Failed to create CODEC list debugfs file\n");

	if (!debugfs_create_file("dais", 0444, snd_soc_debugfs_root, NULL,
				 &dai_list_fops))
		pr_warn("ASoC: Failed to create DAI list debugfs file\n");

	if (!debugfs_create_file("platforms", 0444, snd_soc_debugfs_root, NULL,
				 &platform_list_fops))
		pr_warn("ASoC: Failed to create platform list debugfs file\n");
#endif

	snd_soc_util_init();

	return platform_driver_register(&soc_driver);
}
module_init(snd_soc_init);

static void __exit snd_soc_exit(void)
{
	snd_soc_util_exit();

#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(snd_soc_debugfs_root);
#endif
	platform_driver_unregister(&soc_driver);
}
module_exit(snd_soc_exit);

MODULE_AUTHOR("Liam Girdwood, lrg@slimlogic.co.uk");
MODULE_DESCRIPTION("ALSA SoC Core");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:soc-audio");
