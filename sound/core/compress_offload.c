/*
 *  compress_core.c - compress offload core
 *
 *  Copyright (C) 2011 Intel Corporation
 *  Authors:	Vinod Koul <vinod.koul@linux.intel.com>
 *		Pierre-Louis Bossart <pierre-louis.bossart@linux.intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 */
#define FORMAT(fmt) "%s: %d: " fmt, __func__, __LINE__
#define pr_fmt(fmt) KBUILD_MODNAME ": " FORMAT(fmt)

#include <linux/file.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/math64.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/uio.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/compress_params.h>
#include <sound/compress_offload.h>
#include <sound/compress_driver.h>


static DEFINE_MUTEX(device_mutex);

struct snd_compr_file {
	unsigned long caps;
	struct snd_compr_stream stream;
};

static int snd_compr_open(struct inode *inode, struct file *f)
{
	struct snd_compr *compr;
	struct snd_compr_file *data;
	struct snd_compr_runtime *runtime;
	enum snd_compr_direction dirn;
	int maj = imajor(inode);
	int ret;

	if ((f->f_flags & O_ACCMODE) == O_WRONLY)
		dirn = SND_COMPRESS_PLAYBACK;
	else if ((f->f_flags & O_ACCMODE) == O_RDONLY)
		dirn = SND_COMPRESS_CAPTURE;
	else
		return -EINVAL;

	if (maj == snd_major)
		compr = snd_lookup_minor_data(iminor(inode),
					SNDRV_DEVICE_TYPE_COMPRESS);
	else
		return -EBADFD;

	if (compr == NULL) {
		pr_err("no device data!!!\n");
		return -ENODEV;
	}

	if (dirn != compr->direction) {
		pr_err("this device doesn't support this direction\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->stream.ops = compr->ops;
	data->stream.direction = dirn;
	data->stream.private_data = compr->private_data;
	data->stream.device = compr;
	runtime = kzalloc(sizeof(*runtime), GFP_KERNEL);
	if (!runtime) {
		kfree(data);
		return -ENOMEM;
	}
	runtime->state = SNDRV_PCM_STATE_OPEN;
	init_waitqueue_head(&runtime->sleep);
	data->stream.runtime = runtime;
	f->private_data = (void *)data;
	mutex_lock(&compr->lock);
	ret = compr->ops->open(&data->stream);
	mutex_unlock(&compr->lock);
	if (ret) {
		kfree(runtime);
		kfree(data);
	}
	return ret;
}

static int snd_compr_free(struct inode *inode, struct file *f)
{
	struct snd_compr_file *data = f->private_data;
	struct snd_compr_runtime *runtime = data->stream.runtime;

	switch (runtime->state) {
	case SNDRV_PCM_STATE_RUNNING:
	case SNDRV_PCM_STATE_DRAINING:
	case SNDRV_PCM_STATE_PAUSED:
		data->stream.ops->trigger(&data->stream, SNDRV_PCM_TRIGGER_STOP);
		break;
	default:
		break;
	}

	data->stream.ops->free(&data->stream);
	kfree(data->stream.runtime->buffer);
	kfree(data->stream.runtime);
	kfree(data);
	return 0;
}

static int snd_compr_update_tstamp(struct snd_compr_stream *stream,
		struct snd_compr_tstamp *tstamp)
{
	int err = 0;
	if (!stream->ops->pointer)
		return -ENOTSUPP;
	err = stream->ops->pointer(stream, tstamp);
	if (err)
		return err;
	pr_debug("dsp consumed till %d total %d bytes\n",
		tstamp->byte_offset, tstamp->copied_total);
	if (stream->direction == SND_COMPRESS_PLAYBACK)
		stream->runtime->total_bytes_transferred = tstamp->copied_total;
	else
		stream->runtime->total_bytes_available = tstamp->copied_total;
	return 0;
}

static size_t snd_compr_calc_avail(struct snd_compr_stream *stream,
		struct snd_compr_avail *avail)
{
	memset(avail, 0, sizeof(*avail));
	snd_compr_update_tstamp(stream, &avail->tstamp);
	

	if (stream->runtime->total_bytes_available == 0 &&
			stream->runtime->state == SNDRV_PCM_STATE_SETUP &&
			stream->direction == SND_COMPRESS_PLAYBACK) {
		pr_debug("detected init and someone forgot to do a write\n");
		return stream->runtime->buffer_size;
	}
	pr_debug("app wrote %lld, DSP consumed %lld\n",
			stream->runtime->total_bytes_available,
			stream->runtime->total_bytes_transferred);
	if (stream->runtime->total_bytes_available ==
				stream->runtime->total_bytes_transferred) {
		if (stream->direction == SND_COMPRESS_PLAYBACK) {
			pr_debug("both pointers are same, returning full avail\n");
			return stream->runtime->buffer_size;
		} else {
			pr_debug("both pointers are same, returning no avail\n");
			return 0;
		}
	}

	avail->avail = stream->runtime->total_bytes_available -
			stream->runtime->total_bytes_transferred;
	if (stream->direction == SND_COMPRESS_PLAYBACK)
		avail->avail = stream->runtime->buffer_size - avail->avail;

	pr_debug("ret avail as %lld\n", avail->avail);
	return avail->avail;
}

static inline size_t snd_compr_get_avail(struct snd_compr_stream *stream)
{
	struct snd_compr_avail avail;

	return snd_compr_calc_avail(stream, &avail);
}

static int
snd_compr_ioctl_avail(struct snd_compr_stream *stream, unsigned long arg)
{
	struct snd_compr_avail ioctl_avail;
	size_t avail;

	avail = snd_compr_calc_avail(stream, &ioctl_avail);
	ioctl_avail.avail = avail;

	if (copy_to_user((__u64 __user *)arg,
				&ioctl_avail, sizeof(ioctl_avail)))
		return -EFAULT;
	return 0;
}

static int snd_compr_write_data(struct snd_compr_stream *stream,
	       const char __user *buf, size_t count)
{
	void *dstn;
	size_t copy;
	struct snd_compr_runtime *runtime = stream->runtime;
	
	u64 app_pointer = div64_u64(runtime->total_bytes_available,
				    runtime->buffer_size);
	app_pointer = runtime->total_bytes_available -
		      (app_pointer * runtime->buffer_size);

	dstn = runtime->buffer + app_pointer;
	pr_debug("copying %ld at %lld\n",
			(unsigned long)count, app_pointer);
	if (count < runtime->buffer_size - app_pointer) {
		if (copy_from_user(dstn, buf, count))
			return -EFAULT;
	} else {
		copy = runtime->buffer_size - app_pointer;
		if (copy_from_user(dstn, buf, copy))
			return -EFAULT;
		if (copy_from_user(runtime->buffer, buf + copy, count - copy))
			return -EFAULT;
	}
	/* if DSP cares, let it know data has been written */
	if (stream->ops->ack)
		stream->ops->ack(stream, count);
	return count;
}

static ssize_t snd_compr_write(struct file *f, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct snd_compr_file *data = f->private_data;
	struct snd_compr_stream *stream;
	size_t avail;
	int retval;

	if (snd_BUG_ON(!data))
		return -EFAULT;

	stream = &data->stream;
	mutex_lock(&stream->device->lock);
	
	if (stream->runtime->state != SNDRV_PCM_STATE_SETUP &&
			stream->runtime->state != SNDRV_PCM_STATE_RUNNING) {
		mutex_unlock(&stream->device->lock);
		return -EBADFD;
	}

	avail = snd_compr_get_avail(stream);
	pr_debug("avail returned %ld\n", (unsigned long)avail);
	
	if (avail > count)
		avail = count;

	if (stream->ops->copy) {
		char __user* cbuf = (char __user*)buf;
		retval = stream->ops->copy(stream, cbuf, avail);
	} else {
		retval = snd_compr_write_data(stream, buf, avail);
	}
	if (retval > 0)
		stream->runtime->total_bytes_available += retval;

	if (stream->runtime->state == SNDRV_PCM_STATE_SETUP) {
		stream->runtime->state = SNDRV_PCM_STATE_PREPARED;
		pr_debug("stream prepared, Houston we are good to go\n");
	}

	mutex_unlock(&stream->device->lock);
	return retval;
}


static ssize_t snd_compr_read(struct file *f, char __user *buf,
		size_t count, loff_t *offset)
{
	struct snd_compr_file *data = f->private_data;
	struct snd_compr_stream *stream;
	size_t avail;
	int retval;

	if (snd_BUG_ON(!data))
		return -EFAULT;

	stream = &data->stream;
	mutex_lock(&stream->device->lock);

	
	if (stream->runtime->state != SNDRV_PCM_STATE_RUNNING) {
		retval = -EBADFD;
		goto out;
	}

	avail = snd_compr_get_avail(stream);
	pr_debug("avail returned %ld\n", (unsigned long)avail);
	
	if (avail > count)
		avail = count;

	if (stream->ops->copy) {
		retval = stream->ops->copy(stream, buf, avail);
	} else {
		retval = -ENXIO;
		goto out;
	}
	if (retval > 0)
		stream->runtime->total_bytes_transferred += retval;

out:
	mutex_unlock(&stream->device->lock);
	return retval;
}

static int snd_compr_mmap(struct file *f, struct vm_area_struct *vma)
{
	return -ENXIO;
}

static inline int snd_compr_get_poll(struct snd_compr_stream *stream)
{
	if (stream->direction == SND_COMPRESS_PLAYBACK)
		return POLLOUT | POLLWRNORM;
	else
		return POLLIN | POLLRDNORM;
}

static unsigned int snd_compr_poll(struct file *f, poll_table *wait)
{
	struct snd_compr_file *data = f->private_data;
	struct snd_compr_stream *stream;
	size_t avail;
	int retval = 0;

	if (snd_BUG_ON(!data))
		return -EFAULT;
	stream = &data->stream;
	if (snd_BUG_ON(!stream))
		return -EFAULT;

	mutex_lock(&stream->device->lock);
	if (stream->runtime->state == SNDRV_PCM_STATE_PAUSED ||
			stream->runtime->state == SNDRV_PCM_STATE_OPEN) {
		retval = -EBADFD;
		goto out;
	}
	poll_wait(f, &stream->runtime->sleep, wait);

	avail = snd_compr_get_avail(stream);
	pr_debug("avail is %ld\n", (unsigned long)avail);
	
	switch (stream->runtime->state) {
	case SNDRV_PCM_STATE_DRAINING:
		retval = snd_compr_get_poll(stream);
		stream->runtime->state = SNDRV_PCM_STATE_SETUP;
		break;
	case SNDRV_PCM_STATE_RUNNING:
	case SNDRV_PCM_STATE_PREPARED:
	case SNDRV_PCM_STATE_PAUSED:
		if (avail >= stream->runtime->fragment_size && (avail % stream->runtime->fragment_size == 0))
			retval = snd_compr_get_poll(stream);
		break;
	default:
		if (stream->direction == SND_COMPRESS_PLAYBACK)
			retval = POLLOUT | POLLWRNORM | POLLERR;
		else
			retval = POLLIN | POLLRDNORM | POLLERR;
		break;
	}
out:
	mutex_unlock(&stream->device->lock);
	return retval;
}

static int
snd_compr_get_caps(struct snd_compr_stream *stream, unsigned long arg)
{
	int retval;
	struct snd_compr_caps caps;

	if (!stream->ops->get_caps)
		return -ENXIO;

	memset(&caps, 0, sizeof(caps));
	retval = stream->ops->get_caps(stream, &caps);
	if (retval)
		goto out;
	if (copy_to_user((void __user *)arg, &caps, sizeof(caps)))
		retval = -EFAULT;
out:
	return retval;
}

static int
snd_compr_get_codec_caps(struct snd_compr_stream *stream, unsigned long arg)
{
	int retval;
	struct snd_compr_codec_caps *caps;

	if (!stream->ops->get_codec_caps)
		return -ENXIO;

	caps = kzalloc(sizeof(*caps), GFP_KERNEL);
	if (!caps)
		return -ENOMEM;

	retval = stream->ops->get_codec_caps(stream, caps);
	if (retval)
		goto out;
	if (copy_to_user((void __user *)arg, caps, sizeof(*caps)))
		retval = -EFAULT;

out:
	kfree(caps);
	return retval;
}

static int snd_compr_allocate_buffer(struct snd_compr_stream *stream,
		struct snd_compr_params *params)
{
	unsigned int buffer_size;
	void *buffer;

	buffer_size = params->buffer.fragment_size * params->buffer.fragments;
	if (stream->ops->copy) {
		buffer = NULL;
	} else {
		buffer = kmalloc(buffer_size, GFP_KERNEL);
		if (!buffer)
			return -ENOMEM;
	}
	stream->runtime->fragment_size = params->buffer.fragment_size;
	stream->runtime->fragments = params->buffer.fragments;
	stream->runtime->buffer = buffer;
	stream->runtime->buffer_size = buffer_size;
	return 0;
}

static int snd_compress_check_input(struct snd_compr_params *params)
{
	
	if (params->buffer.fragment_size == 0 ||
			params->buffer.fragments > UINT_MAX / params->buffer.fragment_size)
		return -EINVAL;

	
	if (params->codec.id == 0 || params->codec.id > SND_AUDIOCODEC_MAX)
		return -EINVAL;

	if (params->codec.ch_in == 0 || params->codec.ch_out == 0)
		return -EINVAL;

	if (!(params->codec.sample_rate & SNDRV_PCM_RATE_8000_192000))
		return -EINVAL;

	return 0;
}

static int
snd_compr_set_params(struct snd_compr_stream *stream, unsigned long arg)
{
	struct snd_compr_params *params;
	int retval;

	if (stream->runtime->state == SNDRV_PCM_STATE_OPEN) {
		params = kmalloc(sizeof(*params), GFP_KERNEL);
		if (!params)
			return -ENOMEM;
		if (copy_from_user(params, (void __user *)arg, sizeof(*params))) {
			retval = -EFAULT;
			goto out;
		}

		retval = snd_compress_check_input(params);
		if (retval)
			goto out;

		retval = snd_compr_allocate_buffer(stream, params);
		if (retval) {
			retval = -ENOMEM;
			goto out;
		}

		retval = stream->ops->set_params(stream, params);
		if (retval)
			goto out;

		stream->metadata_set = false;
		stream->next_track = false;

		if (stream->direction == SND_COMPRESS_PLAYBACK)
			stream->runtime->state = SNDRV_PCM_STATE_SETUP;
		else
			stream->runtime->state = SNDRV_PCM_STATE_PREPARED;
	} else {
		return -EPERM;
	}
out:
	kfree(params);
	return retval;
}

static int
snd_compr_get_params(struct snd_compr_stream *stream, unsigned long arg)
{
	struct snd_codec *params;
	int retval;

	if (!stream->ops->get_params)
		return -EBADFD;

	params = kzalloc(sizeof(*params), GFP_KERNEL);
	if (!params)
		return -ENOMEM;
	retval = stream->ops->get_params(stream, params);
	if (retval)
		goto out;
	if (copy_to_user((char __user *)arg, params, sizeof(*params)))
		retval = -EFAULT;

out:
	kfree(params);
	return retval;
}

static int
snd_compr_get_metadata(struct snd_compr_stream *stream, unsigned long arg)
{
	struct snd_compr_metadata metadata;
	int retval;

	if (!stream->ops->get_metadata)
		return -ENXIO;

	if (copy_from_user(&metadata, (void __user *)arg, sizeof(metadata)))
		return -EFAULT;

	retval = stream->ops->get_metadata(stream, &metadata);
	if (retval != 0)
		return retval;

	if (copy_to_user((void __user *)arg, &metadata, sizeof(metadata)))
		return -EFAULT;

	return 0;
}

static int
snd_compr_set_metadata(struct snd_compr_stream *stream, unsigned long arg)
{
	struct snd_compr_metadata metadata;
	int retval;

	if (!stream->ops->set_metadata)
		return -ENXIO;
	if (copy_from_user(&metadata, (void __user *)arg, sizeof(metadata)))
		return -EFAULT;

	retval = stream->ops->set_metadata(stream, &metadata);
	stream->metadata_set = true;

	return retval;
}

static inline int
snd_compr_tstamp(struct snd_compr_stream *stream, unsigned long arg)
{
	struct snd_compr_tstamp tstamp;
	int ret;

	memset(&tstamp, 0, sizeof(tstamp));
	ret = snd_compr_update_tstamp(stream, &tstamp);
	if (ret == 0)
		ret = copy_to_user((struct snd_compr_tstamp __user *)arg,
			&tstamp, sizeof(tstamp)) ? -EFAULT : 0;
	return ret;
}

static int snd_compr_pause(struct snd_compr_stream *stream)
{
	int retval;

	if (stream->runtime->state != SNDRV_PCM_STATE_RUNNING)
		return -EPERM;
	retval = stream->ops->trigger(stream, SNDRV_PCM_TRIGGER_PAUSE_PUSH);
	if (!retval)
		stream->runtime->state = SNDRV_PCM_STATE_PAUSED;
	return retval;
}

static int snd_compr_resume(struct snd_compr_stream *stream)
{
	int retval;

	if (stream->runtime->state != SNDRV_PCM_STATE_PAUSED)
		return -EPERM;
	retval = stream->ops->trigger(stream, SNDRV_PCM_TRIGGER_PAUSE_RELEASE);
	if (!retval)
		stream->runtime->state = SNDRV_PCM_STATE_RUNNING;
	return retval;
}

static int snd_compr_start(struct snd_compr_stream *stream)
{
	int retval;

	if (stream->runtime->state != SNDRV_PCM_STATE_PREPARED)
		return -EPERM;
	retval = stream->ops->trigger(stream, SNDRV_PCM_TRIGGER_START);
	if (!retval)
		stream->runtime->state = SNDRV_PCM_STATE_RUNNING;
	return retval;
}

static int snd_compr_stop(struct snd_compr_stream *stream)
{
	int retval;

	if (stream->runtime->state == SNDRV_PCM_STATE_PREPARED ||
			stream->runtime->state == SNDRV_PCM_STATE_SETUP)
		return -EPERM;
	retval = stream->ops->trigger(stream, SNDRV_PCM_TRIGGER_STOP);
	if (!retval) {
		stream->runtime->state = SNDRV_PCM_STATE_SETUP;
		wake_up(&stream->runtime->sleep);
		stream->runtime->total_bytes_available = 0;
		stream->runtime->total_bytes_transferred = 0;
	}
	return retval;
}

static int snd_compr_drain(struct snd_compr_stream *stream)
{
	int retval;

	mutex_lock(&stream->device->lock);
	if (stream->runtime->state == SNDRV_PCM_STATE_PREPARED ||
			stream->runtime->state == SNDRV_PCM_STATE_SETUP) {
		retval = -EPERM;
		goto ret;
	}
	mutex_unlock(&stream->device->lock);
	retval = stream->ops->trigger(stream, SND_COMPR_TRIGGER_DRAIN);
	mutex_lock(&stream->device->lock);
	if (!retval) {
		stream->runtime->state = SNDRV_PCM_STATE_DRAINING;
		wake_up(&stream->runtime->sleep);
	}
ret:
	mutex_unlock(&stream->device->lock);
	return retval;
}

static int snd_compr_next_track(struct snd_compr_stream *stream)
{
	int retval;

	
	if (stream->runtime->state != SNDRV_PCM_STATE_RUNNING)
		return -EPERM;

	if (stream->metadata_set == false)
		return -EPERM;

	retval = stream->ops->trigger(stream, SND_COMPR_TRIGGER_NEXT_TRACK);
	if (retval != 0)
		return retval;
	stream->metadata_set = false;
	stream->next_track = true;
	return 0;
}

static int snd_compr_partial_drain(struct snd_compr_stream *stream)
{
	int retval;

	mutex_lock(&stream->device->lock);
	if (stream->runtime->state == SNDRV_PCM_STATE_PREPARED ||
			stream->runtime->state == SNDRV_PCM_STATE_SETUP) {
		mutex_unlock(&stream->device->lock);
		return -EPERM;
	}
	mutex_unlock(&stream->device->lock);
	
	if (stream->next_track == false)
		return -EPERM;

	retval = stream->ops->trigger(stream, SND_COMPR_TRIGGER_PARTIAL_DRAIN);

	stream->next_track = false;
	return retval;
}

static int snd_compress_simple_ioctls(struct file *file,
				struct snd_compr_stream *stream,
				unsigned int cmd, unsigned long arg)
{
	int retval = -ENOTTY;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(SNDRV_COMPRESS_IOCTL_VERSION):
		retval = put_user(SNDRV_COMPRESS_VERSION,
				(int __user *)arg) ? -EFAULT : 0;
		break;

	case _IOC_NR(SNDRV_COMPRESS_GET_CAPS):
		retval = snd_compr_get_caps(stream, arg);
		break;

	case _IOC_NR(SNDRV_COMPRESS_GET_CODEC_CAPS):
		retval = snd_compr_get_codec_caps(stream, arg);
		break;


	case _IOC_NR(SNDRV_COMPRESS_TSTAMP):
		retval = snd_compr_tstamp(stream, arg);
		break;

	case _IOC_NR(SNDRV_COMPRESS_AVAIL):
		retval = snd_compr_ioctl_avail(stream, arg);
		break;

	case _IOC_NR(SNDRV_COMPRESS_DRAIN):
		retval = snd_compr_drain(stream);
		break;

	case _IOC_NR(SNDRV_COMPRESS_PARTIAL_DRAIN):
		retval = snd_compr_partial_drain(stream);
		break;
	}

	return retval;
}

static int snd_compr_effect(struct snd_compr_stream *stream, unsigned long arg)
{
   int rc = 0;
   struct snd_compr_runtime *runtime = stream->runtime;
   void *prtd = runtime->private_data;
   struct dsp_effect_param q6_param;
   void *payload;

   pr_info("compress_offload snd_compr_effect +++\n");
   pr_info("[%p] SNDRV_COMPRESS_ENABLE_EFFECT\n", __func__);
   if (copy_from_user(&q6_param, (void *) arg,
               sizeof(q6_param))) {
       pr_err("[%p] %s: copy param from user failed\n",
           prtd, __func__);
       return -EFAULT;
   }
   if (q6_param.payload_size <= 0 ||
       (q6_param.effect_type != 0 &&
        q6_param.effect_type != 1)) {
       pr_err("[%p] %s: unsupported param: %d, 0x%x, 0x%x, %d\n",
           prtd, __func__, q6_param.effect_type,
           q6_param.module_id, q6_param.param_id,
           q6_param.payload_size);
       return -EINVAL;
   }
   payload = kzalloc(q6_param.payload_size, GFP_KERNEL);
   if (!payload) {
       pr_err("[%p] %s: failed to allocate memory\n",
           prtd, __func__);
       return -ENOMEM;
   }
   if (copy_from_user(payload, (void *) (arg + sizeof(q6_param)),
       q6_param.payload_size)) {
       pr_err("[%p] %s: copy payload from user failed\n",
           prtd, __func__);
       kfree(payload);
       return -EFAULT;
   }
   if (q6_param.effect_type == 0) { 
       rc = stream->ops->config_effect(stream, (void *)&q6_param, payload);
       if (rc) {
           pr_err("[%p] %s: config_effect error %d\n", prtd, __func__, rc);
       }
   }
   pr_info("compress_offload snd_compr_effect ---\n");
   kfree(payload);
   return 0;
}


static long snd_compr_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct snd_compr_file *data = f->private_data;
	struct snd_compr_stream *stream;
	int retval = -ENOTTY;

	if (snd_BUG_ON(!data))
		return -EFAULT;
	stream = &data->stream;
	if (snd_BUG_ON(!stream))
		return -EFAULT;

	mutex_lock(&stream->device->lock);
	switch (_IOC_NR(cmd)) {
	case _IOC_NR(SNDRV_COMPRESS_SET_PARAMS):
		retval = snd_compr_set_params(stream, arg);
		break;

	case _IOC_NR(SNDRV_COMPRESS_GET_PARAMS):
		retval = snd_compr_get_params(stream, arg);
		break;

	case _IOC_NR(SNDRV_COMPRESS_SET_METADATA):
		retval = snd_compr_set_metadata(stream, arg);
		break;

	case _IOC_NR(SNDRV_COMPRESS_GET_METADATA):
		retval = snd_compr_get_metadata(stream, arg);
		break;

	case _IOC_NR(SNDRV_COMPRESS_PAUSE):
		retval = snd_compr_pause(stream);
		break;

	case _IOC_NR(SNDRV_COMPRESS_RESUME):
		retval = snd_compr_resume(stream);
		break;

	case _IOC_NR(SNDRV_COMPRESS_START):
		retval = snd_compr_start(stream);
		break;

	case _IOC_NR(SNDRV_COMPRESS_STOP):
		retval = snd_compr_stop(stream);
		break;

	case _IOC_NR(SNDRV_COMPRESS_NEXT_TRACK):
		retval = snd_compr_next_track(stream);
		break;

	case _IOC_NR(SNDRV_COMPRESS_ENABLE_EFFECT):
		retval = snd_compr_effect(stream, arg);
		break;

	default:
		mutex_unlock(&stream->device->lock);
		return snd_compress_simple_ioctls(f, stream, cmd, arg);

	}

	mutex_unlock(&stream->device->lock);
	return retval;
}

static const struct file_operations snd_compr_file_ops = {
		.owner =	THIS_MODULE,
		.open =		snd_compr_open,
		.release =	snd_compr_free,
		.write =	snd_compr_write,
		.read =		snd_compr_read,
		.unlocked_ioctl = snd_compr_ioctl,
		.mmap =		snd_compr_mmap,
		.poll =		snd_compr_poll,
};

static int snd_compress_dev_register(struct snd_device *device)
{
	int ret = -EINVAL;
	char str[16];
	struct snd_compr *compr;

	if (snd_BUG_ON(!device || !device->device_data))
		return -EBADFD;
	compr = device->device_data;

	snprintf(str, sizeof(str),"comprC%iD%i", compr->card->number, compr->device);
	pr_debug("reg %s for device %s, direction %d\n", str, compr->name,
			compr->direction);
	
	ret = snd_register_device(SNDRV_DEVICE_TYPE_COMPRESS, compr->card,
			compr->device, &snd_compr_file_ops, compr, str);
	if (ret < 0) {
		pr_err("snd_register_device failed\n %d", ret);
		return ret;
	}
	return ret;

}

static int snd_compress_dev_disconnect(struct snd_device *device)
{
	struct snd_compr *compr;

	compr = device->device_data;
	snd_unregister_device(compr->direction, compr->card, compr->device);
	return 0;
}

int snd_compress_new(struct snd_card *card, int device,
			int dirn, struct snd_compr *compr)
{
	static struct snd_device_ops ops = {
		.dev_free = NULL,
		.dev_register = snd_compress_dev_register,
		.dev_disconnect = snd_compress_dev_disconnect,
	};

	compr->card = card;
	compr->device = device;
	compr->direction = dirn;
	return snd_device_new(card, SNDRV_DEV_COMPRESS, compr, &ops);
}
EXPORT_SYMBOL_GPL(snd_compress_new);

void snd_compress_free(struct snd_card *card, struct snd_compr *compr)
{
	snd_device_free(card, compr);
}
EXPORT_SYMBOL_GPL(snd_compress_free);

static int snd_compress_add_device(struct snd_compr *device)
{
	int ret;

	if (!device->card)
		return -EINVAL;

	
	ret = snd_card_register(device->card);
	if (ret)
		goto out;
	return 0;

out:
	pr_err("failed with %d\n", ret);
	return ret;

}

static int snd_compress_remove_device(struct snd_compr *device)
{
	return snd_card_free(device->card);
}

int snd_compress_register(struct snd_compr *device)
{
	int retval;

	if (device->name == NULL || device->dev == NULL || device->ops == NULL)
		return -EINVAL;

	pr_debug("Registering compressed device %s\n", device->name);
	if (snd_BUG_ON(!device->ops->open))
		return -EINVAL;
	if (snd_BUG_ON(!device->ops->free))
		return -EINVAL;
	if (snd_BUG_ON(!device->ops->set_params))
		return -EINVAL;
	if (snd_BUG_ON(!device->ops->trigger))
		return -EINVAL;

	mutex_init(&device->lock);

	
	mutex_lock(&device_mutex);
	retval = snd_compress_add_device(device);
	mutex_unlock(&device_mutex);
	return retval;
}
EXPORT_SYMBOL_GPL(snd_compress_register);

int snd_compress_deregister(struct snd_compr *device)
{
	pr_debug("Removing compressed device %s\n", device->name);
	mutex_lock(&device_mutex);
	snd_compress_remove_device(device);
	mutex_unlock(&device_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(snd_compress_deregister);

static int __init snd_compress_init(void)
{
	return 0;
}

static void __exit snd_compress_exit(void)
{
}

module_init(snd_compress_init);
module_exit(snd_compress_exit);

MODULE_DESCRIPTION("ALSA Compressed offload framework");
MODULE_AUTHOR("Vinod Koul <vinod.koul@linux.intel.com>");
MODULE_LICENSE("GPL v2");
