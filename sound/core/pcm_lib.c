/*
 *  Digital Audio (PCM) abstract layer
 *  Copyright (c) by Jaroslav Kysela <perex@perex.cz>
 *                   Abramo Bagnara <abramo@alsa-project.org>
 *
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; only version 2 of the License.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/slab.h>
#include <linux/time.h>
#include <linux/math64.h>
#include <linux/export.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/tlv.h>
#include <sound/info.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/timer.h>

#define STRING_LENGTH_OF_INT 12

void snd_pcm_playback_silence(struct snd_pcm_substream *substream, snd_pcm_uframes_t new_hw_ptr)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t frames, ofs, transfer;

	if (runtime->silence_size < runtime->boundary) {
		snd_pcm_sframes_t noise_dist, n;
		if (runtime->silence_start != runtime->control->appl_ptr) {
			n = runtime->control->appl_ptr - runtime->silence_start;
			if (n < 0)
				n += runtime->boundary;
			if ((snd_pcm_uframes_t)n < runtime->silence_filled)
				runtime->silence_filled -= n;
			else
				runtime->silence_filled = 0;
			runtime->silence_start = runtime->control->appl_ptr;
		}
		if (runtime->silence_filled >= runtime->buffer_size)
			return;
		noise_dist = snd_pcm_playback_hw_avail(runtime) + runtime->silence_filled;
		if (noise_dist >= (snd_pcm_sframes_t) runtime->silence_threshold)
			return;
		frames = runtime->silence_threshold - noise_dist;
		if (frames > runtime->silence_size)
			frames = runtime->silence_size;
	} else {
		if (new_hw_ptr == ULONG_MAX) {	
			snd_pcm_sframes_t avail = snd_pcm_playback_hw_avail(runtime);
			if (avail > runtime->buffer_size)
				avail = runtime->buffer_size;
			runtime->silence_filled = avail > 0 ? avail : 0;
			runtime->silence_start = (runtime->status->hw_ptr +
						  runtime->silence_filled) %
						 runtime->boundary;
		} else {
			ofs = runtime->status->hw_ptr;
			frames = new_hw_ptr - ofs;
			if ((snd_pcm_sframes_t)frames < 0)
				frames += runtime->boundary;
			runtime->silence_filled -= frames;
			if ((snd_pcm_sframes_t)runtime->silence_filled < 0) {
				runtime->silence_filled = 0;
				runtime->silence_start = new_hw_ptr;
			} else {
				runtime->silence_start = ofs;
			}
		}
		frames = runtime->buffer_size - runtime->silence_filled;
	}
	if (snd_BUG_ON(frames > runtime->buffer_size))
		return;
	if (frames == 0)
		return;
	ofs = runtime->silence_start % runtime->buffer_size;
	while (frames > 0) {
		transfer = ofs + frames > runtime->buffer_size ? runtime->buffer_size - ofs : frames;
		if (runtime->access == SNDRV_PCM_ACCESS_RW_INTERLEAVED ||
		    runtime->access == SNDRV_PCM_ACCESS_MMAP_INTERLEAVED) {
			if (substream->ops->silence) {
				int err;
				err = substream->ops->silence(substream, -1, ofs, transfer);
				snd_BUG_ON(err < 0);
			} else {
				char *hwbuf = runtime->dma_area + frames_to_bytes(runtime, ofs);
				snd_pcm_format_set_silence(runtime->format, hwbuf, transfer * runtime->channels);
			}
		} else {
			unsigned int c;
			unsigned int channels = runtime->channels;
			if (substream->ops->silence) {
				for (c = 0; c < channels; ++c) {
					int err;
					err = substream->ops->silence(substream, c, ofs, transfer);
					snd_BUG_ON(err < 0);
				}
			} else {
				size_t dma_csize = runtime->dma_bytes / channels;
				for (c = 0; c < channels; ++c) {
					char *hwbuf = runtime->dma_area + (c * dma_csize) + samples_to_bytes(runtime, ofs);
					snd_pcm_format_set_silence(runtime->format, hwbuf, transfer);
				}
			}
		}
		runtime->silence_filled += transfer;
		frames -= transfer;
		ofs = 0;
	}
}

#ifdef CONFIG_SND_DEBUG
void snd_pcm_debug_name(struct snd_pcm_substream *substream,
			   char *name, size_t len)
{
	snprintf(name, len, "pcmC%dD%d%c:%d",
		 substream->pcm->card->number,
		 substream->pcm->device,
		 substream->stream ? 'c' : 'p',
		 substream->number);
}
EXPORT_SYMBOL(snd_pcm_debug_name);
#endif

#define XRUN_DEBUG_BASIC	(1<<0)
#define XRUN_DEBUG_STACK	(1<<1)	
#define XRUN_DEBUG_JIFFIESCHECK	(1<<2)	
#define XRUN_DEBUG_PERIODUPDATE	(1<<3)	
#define XRUN_DEBUG_HWPTRUPDATE	(1<<4)	
#define XRUN_DEBUG_LOG		(1<<5)	
#define XRUN_DEBUG_LOGONCE	(1<<6)	

#ifdef CONFIG_SND_PCM_XRUN_DEBUG

#define xrun_debug(substream, mask) \
			((substream)->pstr->xrun_debug & (mask))
#else
#define xrun_debug(substream, mask)	0
#endif

#define dump_stack_on_xrun(substream) do {			\
		if (xrun_debug(substream, XRUN_DEBUG_STACK))	\
			dump_stack();				\
	} while (0)

static void xrun(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (runtime->tstamp_mode == SNDRV_PCM_TSTAMP_ENABLE)
		snd_pcm_gettime(runtime, (struct timespec *)&runtime->status->tstamp);
	snd_pcm_stop(substream, SNDRV_PCM_STATE_XRUN);
	if (xrun_debug(substream, XRUN_DEBUG_BASIC)) {
		char name[16];
		snd_pcm_debug_name(substream, name, sizeof(name));
		snd_printd(KERN_DEBUG "XRUN: %s\n", name);
		dump_stack_on_xrun(substream);
	}
}

#ifdef CONFIG_SND_PCM_XRUN_DEBUG
#define hw_ptr_error(substream, fmt, args...)				\
	do {								\
		if (xrun_debug(substream, XRUN_DEBUG_BASIC)) {		\
			xrun_log_show(substream);			\
			if (printk_ratelimit()) {			\
				snd_printd("PCM: " fmt, ##args);	\
			}						\
			dump_stack_on_xrun(substream);			\
		}							\
	} while (0)

#define XRUN_LOG_CNT	10

struct hwptr_log_entry {
	unsigned int in_interrupt;
	unsigned long jiffies;
	snd_pcm_uframes_t pos;
	snd_pcm_uframes_t period_size;
	snd_pcm_uframes_t buffer_size;
	snd_pcm_uframes_t old_hw_ptr;
	snd_pcm_uframes_t hw_ptr_base;
};

struct snd_pcm_hwptr_log {
	unsigned int idx;
	unsigned int hit: 1;
	struct hwptr_log_entry entries[XRUN_LOG_CNT];
};

static void xrun_log(struct snd_pcm_substream *substream,
		     snd_pcm_uframes_t pos, int in_interrupt)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_pcm_hwptr_log *log = runtime->hwptr_log;
	struct hwptr_log_entry *entry;

	if (log == NULL) {
		log = kzalloc(sizeof(*log), GFP_ATOMIC);
		if (log == NULL)
			return;
		runtime->hwptr_log = log;
	} else {
		if (xrun_debug(substream, XRUN_DEBUG_LOGONCE) && log->hit)
			return;
	}
	entry = &log->entries[log->idx];
	entry->in_interrupt = in_interrupt;
	entry->jiffies = jiffies;
	entry->pos = pos;
	entry->period_size = runtime->period_size;
	entry->buffer_size = runtime->buffer_size;
	entry->old_hw_ptr = runtime->status->hw_ptr;
	entry->hw_ptr_base = runtime->hw_ptr_base;
	log->idx = (log->idx + 1) % XRUN_LOG_CNT;
}

static void xrun_log_show(struct snd_pcm_substream *substream)
{
	struct snd_pcm_hwptr_log *log = substream->runtime->hwptr_log;
	struct hwptr_log_entry *entry;
	char name[16];
	unsigned int idx;
	int cnt;

	if (log == NULL)
		return;
	if (xrun_debug(substream, XRUN_DEBUG_LOGONCE) && log->hit)
		return;
	snd_pcm_debug_name(substream, name, sizeof(name));
	for (cnt = 0, idx = log->idx; cnt < XRUN_LOG_CNT; cnt++) {
		entry = &log->entries[idx];
		if (entry->period_size == 0)
			break;
		snd_printd("hwptr log: %s: %sj=%lu, pos=%ld/%ld/%ld, "
			   "hwptr=%ld/%ld\n",
			   name, entry->in_interrupt ? "[Q] " : "",
			   entry->jiffies,
			   (unsigned long)entry->pos,
			   (unsigned long)entry->period_size,
			   (unsigned long)entry->buffer_size,
			   (unsigned long)entry->old_hw_ptr,
			   (unsigned long)entry->hw_ptr_base);
		idx++;
		idx %= XRUN_LOG_CNT;
	}
	log->hit = 1;
}

#else 

#define hw_ptr_error(substream, fmt, args...) do { } while (0)
#define xrun_log(substream, pos, in_interrupt)	do { } while (0)
#define xrun_log_show(substream)	do { } while (0)

#endif

int snd_pcm_update_state(struct snd_pcm_substream *substream,
			 struct snd_pcm_runtime *runtime)
{
	snd_pcm_uframes_t avail;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		avail = snd_pcm_playback_avail(runtime);
	else
		avail = snd_pcm_capture_avail(runtime);
	if (avail > runtime->avail_max)
		runtime->avail_max = avail;
	if (runtime->status->state == SNDRV_PCM_STATE_DRAINING) {
		if (avail >= runtime->buffer_size) {
			snd_pcm_drain_done(substream);
			return -EPIPE;
		}
	} else {
		if (avail >= runtime->stop_threshold) {
			xrun(substream);
			return -EPIPE;
		}
	}
	if (runtime->twake) {
		if (avail >= runtime->twake)
			wake_up(&runtime->tsleep);
	} else if (avail >= runtime->control->avail_min)
		wake_up(&runtime->sleep);
	return 0;
}

static int snd_pcm_update_hw_ptr0(struct snd_pcm_substream *substream,
				  unsigned int in_interrupt)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t pos;
	snd_pcm_uframes_t old_hw_ptr, new_hw_ptr, hw_base;
	snd_pcm_sframes_t hdelta, delta;
	unsigned long jdelta;

	old_hw_ptr = runtime->status->hw_ptr;
	pos = substream->ops->pointer(substream);
	if (pos == SNDRV_PCM_POS_XRUN) {
		xrun(substream);
		return -EPIPE;
	}
	if (pos >= runtime->buffer_size) {
		if (printk_ratelimit()) {
			char name[16];
			snd_pcm_debug_name(substream, name, sizeof(name));
			xrun_log_show(substream);
			snd_printd(KERN_ERR  "BUG: %s, pos = %ld, "
				   "buffer size = %ld, period size = %ld\n",
				   name, pos, runtime->buffer_size,
				   runtime->period_size);
		}
		pos = 0;
	}
	pos -= pos % runtime->min_align;
	if (xrun_debug(substream, XRUN_DEBUG_LOG))
		xrun_log(substream, pos, in_interrupt);
	hw_base = runtime->hw_ptr_base;
	new_hw_ptr = hw_base + pos;
	if (in_interrupt) {
		
		
		delta = runtime->hw_ptr_interrupt + runtime->period_size;
		if (delta > new_hw_ptr) {
			
			hdelta = jiffies - runtime->hw_ptr_jiffies;
			if (hdelta > runtime->hw_ptr_buffer_jiffies/2) {
				hw_base += runtime->buffer_size;
				if (hw_base >= runtime->boundary)
					hw_base = 0;
				new_hw_ptr = hw_base + pos;
				goto __delta;
			}
		}
	}
	
	
	if (new_hw_ptr < old_hw_ptr) {
		hw_base += runtime->buffer_size;
		if (hw_base >= runtime->boundary)
			hw_base = 0;
		new_hw_ptr = hw_base + pos;
	}
      __delta:
	delta = new_hw_ptr - old_hw_ptr;
	if (delta < 0)
		delta += runtime->boundary;
	if (xrun_debug(substream, in_interrupt ?
			XRUN_DEBUG_PERIODUPDATE : XRUN_DEBUG_HWPTRUPDATE)) {
		char name[16];
		snd_pcm_debug_name(substream, name, sizeof(name));
		snd_printd("%s_update: %s: pos=%u/%u/%u, "
			   "hwptr=%ld/%ld/%ld/%ld\n",
			   in_interrupt ? "period" : "hwptr",
			   name,
			   (unsigned int)pos,
			   (unsigned int)runtime->period_size,
			   (unsigned int)runtime->buffer_size,
			   (unsigned long)delta,
			   (unsigned long)old_hw_ptr,
			   (unsigned long)new_hw_ptr,
			   (unsigned long)runtime->hw_ptr_base);
	}

	if (runtime->no_period_wakeup) {
		snd_pcm_sframes_t xrun_threshold;
		jdelta = jiffies - runtime->hw_ptr_jiffies;
		if (jdelta < runtime->hw_ptr_buffer_jiffies / 2)
			goto no_delta_check;
		hdelta = jdelta - delta * HZ / runtime->rate;
		xrun_threshold = runtime->hw_ptr_buffer_jiffies / 2 + 1;
		while (hdelta > xrun_threshold) {
			delta += runtime->buffer_size;
			hw_base += runtime->buffer_size;
			if (hw_base >= runtime->boundary)
				hw_base = 0;
			new_hw_ptr = hw_base + pos;
			hdelta -= runtime->hw_ptr_buffer_jiffies;
		}
		goto no_delta_check;
	}

	
	if (delta >= runtime->buffer_size + runtime->period_size) {
		hw_ptr_error(substream,
			       "Unexpected hw_pointer value %s"
			       "(stream=%i, pos=%ld, new_hw_ptr=%ld, "
			       "old_hw_ptr=%ld)\n",
				     in_interrupt ? "[Q] " : "[P]",
				     substream->stream, (long)pos,
				     (long)new_hw_ptr, (long)old_hw_ptr);
		return 0;
	}

	
	if (!xrun_debug(substream, XRUN_DEBUG_JIFFIESCHECK))
		goto no_jiffies_check;

	if (runtime->hw.info & SNDRV_PCM_INFO_BATCH)
		goto no_jiffies_check;
	hdelta = delta;
	if (hdelta < runtime->delay)
		goto no_jiffies_check;
	hdelta -= runtime->delay;
	jdelta = jiffies - runtime->hw_ptr_jiffies;
	if (((hdelta * HZ) / runtime->rate) > jdelta + HZ/100) {
		delta = jdelta /
			(((runtime->period_size * HZ) / runtime->rate)
								+ HZ/100);
		
		new_hw_ptr = old_hw_ptr;
		hw_base = delta;
		
		
		while (delta > 0) {
			new_hw_ptr += runtime->period_size;
			if (new_hw_ptr >= runtime->boundary)
				new_hw_ptr -= runtime->boundary;
			delta--;
		}
		
		hw_ptr_error(substream,
			     "hw_ptr skipping! %s"
			     "(pos=%ld, delta=%ld, period=%ld, "
			     "jdelta=%lu/%lu/%lu, hw_ptr=%ld/%ld)\n",
			     in_interrupt ? "[Q] " : "",
			     (long)pos, (long)hdelta,
			     (long)runtime->period_size, jdelta,
			     ((hdelta * HZ) / runtime->rate), hw_base,
			     (unsigned long)old_hw_ptr,
			     (unsigned long)new_hw_ptr);
		
		delta = 0;
		hw_base = new_hw_ptr - (new_hw_ptr % runtime->buffer_size);
	}
 no_jiffies_check:
	if (delta > runtime->period_size + runtime->period_size / 2) {
		hw_ptr_error(substream,
			     "Lost interrupts? %s"
			     "(stream=%i, delta=%ld, new_hw_ptr=%ld, "
			     "old_hw_ptr=%ld)\n",
			     in_interrupt ? "[Q] " : "",
			     substream->stream, (long)delta,
			     (long)new_hw_ptr,
			     (long)old_hw_ptr);
	}

 no_delta_check:
	if (runtime->status->hw_ptr == new_hw_ptr)
		return 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK &&
	    runtime->silence_size > 0)
		snd_pcm_playback_silence(substream, new_hw_ptr);

	if (in_interrupt) {
		delta = new_hw_ptr - runtime->hw_ptr_interrupt;
		if (delta < 0)
			delta += runtime->boundary;
		delta -= (snd_pcm_uframes_t)delta % runtime->period_size;
		runtime->hw_ptr_interrupt += delta;
		if (runtime->hw_ptr_interrupt >= runtime->boundary)
			runtime->hw_ptr_interrupt -= runtime->boundary;
	}
	runtime->hw_ptr_base = hw_base;
	runtime->status->hw_ptr = new_hw_ptr;
	runtime->hw_ptr_jiffies = jiffies;
	if (runtime->tstamp_mode == SNDRV_PCM_TSTAMP_ENABLE)
		snd_pcm_gettime(runtime, (struct timespec *)&runtime->status->tstamp);

	return snd_pcm_update_state(substream, runtime);
}

int snd_pcm_update_hw_ptr(struct snd_pcm_substream *substream)
{
	return snd_pcm_update_hw_ptr0(substream, 0);
}

void snd_pcm_set_ops(struct snd_pcm *pcm, int direction, struct snd_pcm_ops *ops)
{
	struct snd_pcm_str *stream = &pcm->streams[direction];
	struct snd_pcm_substream *substream;
	
	for (substream = stream->substream; substream != NULL; substream = substream->next)
		substream->ops = ops;
}

EXPORT_SYMBOL(snd_pcm_set_ops);

void snd_pcm_set_sync(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	
	runtime->sync.id32[0] = substream->pcm->card->number;
	runtime->sync.id32[1] = -1;
	runtime->sync.id32[2] = -1;
	runtime->sync.id32[3] = -1;
}

EXPORT_SYMBOL(snd_pcm_set_sync);


static inline unsigned int div32(unsigned int a, unsigned int b, 
				 unsigned int *r)
{
	if (b == 0) {
		*r = 0;
		return UINT_MAX;
	}
	*r = a % b;
	return a / b;
}

static inline unsigned int div_down(unsigned int a, unsigned int b)
{
	if (b == 0)
		return UINT_MAX;
	return a / b;
}

static inline unsigned int div_up(unsigned int a, unsigned int b)
{
	unsigned int r;
	unsigned int q;
	if (b == 0)
		return UINT_MAX;
	q = div32(a, b, &r);
	if (r)
		++q;
	return q;
}

static inline unsigned int mul(unsigned int a, unsigned int b)
{
	if (a == 0)
		return 0;
	if (div_down(UINT_MAX, a) < b)
		return UINT_MAX;
	return a * b;
}

static inline unsigned int muldiv32(unsigned int a, unsigned int b,
				    unsigned int c, unsigned int *r)
{
	u_int64_t n = (u_int64_t) a * b;
	if (c == 0) {
		snd_BUG_ON(!n);
		*r = 0;
		return UINT_MAX;
	}
	n = div_u64_rem(n, c, r);
	if (n >= UINT_MAX) {
		*r = 0;
		return UINT_MAX;
	}
	return n;
}

int snd_interval_refine(struct snd_interval *i, const struct snd_interval *v)
{
	int changed = 0;
	if (snd_BUG_ON(snd_interval_empty(i)))
		return -EINVAL;
	if (i->min < v->min) {
		i->min = v->min;
		i->openmin = v->openmin;
		changed = 1;
	} else if (i->min == v->min && !i->openmin && v->openmin) {
		i->openmin = 1;
		changed = 1;
	}
	if (i->max > v->max) {
		i->max = v->max;
		i->openmax = v->openmax;
		changed = 1;
	} else if (i->max == v->max && !i->openmax && v->openmax) {
		i->openmax = 1;
		changed = 1;
	}
	if (!i->integer && v->integer) {
		i->integer = 1;
		changed = 1;
	}
	if (i->integer) {
		if (i->openmin) {
			i->min++;
			i->openmin = 0;
		}
		if (i->openmax) {
			i->max--;
			i->openmax = 0;
		}
	} else if (!i->openmin && !i->openmax && i->min == i->max)
		i->integer = 1;
	if (snd_interval_checkempty(i)) {
		snd_interval_none(i);
		return -EINVAL;
	}
	return changed;
}

EXPORT_SYMBOL(snd_interval_refine);

static int snd_interval_refine_first(struct snd_interval *i)
{
	if (snd_BUG_ON(snd_interval_empty(i)))
		return -EINVAL;
	if (snd_interval_single(i))
		return 0;
	i->max = i->min;
	i->openmax = i->openmin;
	if (i->openmax)
		i->max++;
	return 1;
}

static int snd_interval_refine_last(struct snd_interval *i)
{
	if (snd_BUG_ON(snd_interval_empty(i)))
		return -EINVAL;
	if (snd_interval_single(i))
		return 0;
	i->min = i->max;
	i->openmin = i->openmax;
	if (i->openmin)
		i->min--;
	return 1;
}

void snd_interval_mul(const struct snd_interval *a, const struct snd_interval *b, struct snd_interval *c)
{
	if (a->empty || b->empty) {
		snd_interval_none(c);
		return;
	}
	c->empty = 0;
	c->min = mul(a->min, b->min);
	c->openmin = (a->openmin || b->openmin);
	c->max = mul(a->max,  b->max);
	c->openmax = (a->openmax || b->openmax);
	c->integer = (a->integer && b->integer);
}

void snd_interval_div(const struct snd_interval *a, const struct snd_interval *b, struct snd_interval *c)
{
	unsigned int r;
	if (a->empty || b->empty) {
		snd_interval_none(c);
		return;
	}
	c->empty = 0;
	c->min = div32(a->min, b->max, &r);
	c->openmin = (r || a->openmin || b->openmax);
	if (b->min > 0) {
		c->max = div32(a->max, b->min, &r);
		if (r) {
			c->max++;
			c->openmax = 1;
		} else
			c->openmax = (a->openmax || b->openmin);
	} else {
		c->max = UINT_MAX;
		c->openmax = 0;
	}
	c->integer = 0;
}

void snd_interval_muldivk(const struct snd_interval *a, const struct snd_interval *b,
		      unsigned int k, struct snd_interval *c)
{
	unsigned int r;
	if (a->empty || b->empty) {
		snd_interval_none(c);
		return;
	}
	c->empty = 0;
	c->min = muldiv32(a->min, b->min, k, &r);
	c->openmin = (r || a->openmin || b->openmin);
	c->max = muldiv32(a->max, b->max, k, &r);
	if (r) {
		c->max++;
		c->openmax = 1;
	} else
		c->openmax = (a->openmax || b->openmax);
	c->integer = 0;
}

void snd_interval_mulkdiv(const struct snd_interval *a, unsigned int k,
		      const struct snd_interval *b, struct snd_interval *c)
{
	unsigned int r;
	if (a->empty || b->empty) {
		snd_interval_none(c);
		return;
	}
	c->empty = 0;
	c->min = muldiv32(a->min, k, b->max, &r);
	c->openmin = (r || a->openmin || b->openmax);
	if (b->min > 0) {
		c->max = muldiv32(a->max, k, b->min, &r);
		if (r) {
			c->max++;
			c->openmax = 1;
		} else
			c->openmax = (a->openmax || b->openmin);
	} else {
		c->max = UINT_MAX;
		c->openmax = 0;
	}
	c->integer = 0;
}



int snd_interval_ratnum(struct snd_interval *i,
			unsigned int rats_count, struct snd_ratnum *rats,
			unsigned int *nump, unsigned int *denp)
{
	unsigned int best_num, best_den;
	int best_diff;
	unsigned int k;
	struct snd_interval t;
	int err;
	unsigned int result_num, result_den;
	int result_diff;

	best_num = best_den = best_diff = 0;
	for (k = 0; k < rats_count; ++k) {
		unsigned int num = rats[k].num;
		unsigned int den;
		unsigned int q = i->min;
		int diff;
		if (q == 0)
			q = 1;
		den = div_up(num, q);
		if (den < rats[k].den_min)
			continue;
		if (den > rats[k].den_max)
			den = rats[k].den_max;
		else {
			unsigned int r;
			r = (den - rats[k].den_min) % rats[k].den_step;
			if (r != 0)
				den -= r;
		}
		diff = num - q * den;
		if (diff < 0)
			diff = -diff;
		if (best_num == 0 ||
		    diff * best_den < best_diff * den) {
			best_diff = diff;
			best_den = den;
			best_num = num;
		}
	}
	if (best_den == 0) {
		i->empty = 1;
		return -EINVAL;
	}
	t.min = div_down(best_num, best_den);
	t.openmin = !!(best_num % best_den);
	
	result_num = best_num;
	result_diff = best_diff;
	result_den = best_den;
	best_num = best_den = best_diff = 0;
	for (k = 0; k < rats_count; ++k) {
		unsigned int num = rats[k].num;
		unsigned int den;
		unsigned int q = i->max;
		int diff;
		if (q == 0) {
			i->empty = 1;
			return -EINVAL;
		}
		den = div_down(num, q);
		if (den > rats[k].den_max)
			continue;
		if (den < rats[k].den_min)
			den = rats[k].den_min;
		else {
			unsigned int r;
			r = (den - rats[k].den_min) % rats[k].den_step;
			if (r != 0)
				den += rats[k].den_step - r;
		}
		diff = q * den - num;
		if (diff < 0)
			diff = -diff;
		if (best_num == 0 ||
		    diff * best_den < best_diff * den) {
			best_diff = diff;
			best_den = den;
			best_num = num;
		}
	}
	if (best_den == 0) {
		i->empty = 1;
		return -EINVAL;
	}
	t.max = div_up(best_num, best_den);
	t.openmax = !!(best_num % best_den);
	t.integer = 0;
	err = snd_interval_refine(i, &t);
	if (err < 0)
		return err;

	if (snd_interval_single(i)) {
		if (best_diff * result_den < result_diff * best_den) {
			result_num = best_num;
			result_den = best_den;
		}
		if (nump)
			*nump = result_num;
		if (denp)
			*denp = result_den;
	}
	return err;
}

EXPORT_SYMBOL(snd_interval_ratnum);

static int snd_interval_ratden(struct snd_interval *i,
			       unsigned int rats_count, struct snd_ratden *rats,
			       unsigned int *nump, unsigned int *denp)
{
	unsigned int best_num, best_diff, best_den;
	unsigned int k;
	struct snd_interval t;
	int err;

	best_num = best_den = best_diff = 0;
	for (k = 0; k < rats_count; ++k) {
		unsigned int num;
		unsigned int den = rats[k].den;
		unsigned int q = i->min;
		int diff;
		num = mul(q, den);
		if (num > rats[k].num_max)
			continue;
		if (num < rats[k].num_min)
			num = rats[k].num_max;
		else {
			unsigned int r;
			r = (num - rats[k].num_min) % rats[k].num_step;
			if (r != 0)
				num += rats[k].num_step - r;
		}
		diff = num - q * den;
		if (best_num == 0 ||
		    diff * best_den < best_diff * den) {
			best_diff = diff;
			best_den = den;
			best_num = num;
		}
	}
	if (best_den == 0) {
		i->empty = 1;
		return -EINVAL;
	}
	t.min = div_down(best_num, best_den);
	t.openmin = !!(best_num % best_den);
	
	best_num = best_den = best_diff = 0;
	for (k = 0; k < rats_count; ++k) {
		unsigned int num;
		unsigned int den = rats[k].den;
		unsigned int q = i->max;
		int diff;
		num = mul(q, den);
		if (num < rats[k].num_min)
			continue;
		if (num > rats[k].num_max)
			num = rats[k].num_max;
		else {
			unsigned int r;
			r = (num - rats[k].num_min) % rats[k].num_step;
			if (r != 0)
				num -= r;
		}
		diff = q * den - num;
		if (best_num == 0 ||
		    diff * best_den < best_diff * den) {
			best_diff = diff;
			best_den = den;
			best_num = num;
		}
	}
	if (best_den == 0) {
		i->empty = 1;
		return -EINVAL;
	}
	t.max = div_up(best_num, best_den);
	t.openmax = !!(best_num % best_den);
	t.integer = 0;
	err = snd_interval_refine(i, &t);
	if (err < 0)
		return err;

	if (snd_interval_single(i)) {
		if (nump)
			*nump = best_num;
		if (denp)
			*denp = best_den;
	}
	return err;
}

int snd_interval_list(struct snd_interval *i, unsigned int count,
		      unsigned int *list, unsigned int mask)
{
        unsigned int k;
	struct snd_interval list_range;

	if (!count) {
		i->empty = 1;
		return -EINVAL;
	}
	snd_interval_any(&list_range);
	list_range.min = UINT_MAX;
	list_range.max = 0;
        for (k = 0; k < count; k++) {
		if (mask && !(mask & (1 << k)))
			continue;
		if (!snd_interval_test(i, list[k]))
			continue;
		list_range.min = min(list_range.min, list[k]);
		list_range.max = max(list_range.max, list[k]);
        }
	return snd_interval_refine(i, &list_range);
}

EXPORT_SYMBOL(snd_interval_list);

static int snd_interval_step(struct snd_interval *i, unsigned int min, unsigned int step)
{
	unsigned int n;
	int changed = 0;
	n = (i->min - min) % step;
	if (n != 0 || i->openmin) {
		i->min += step - n;
		changed = 1;
	}
	n = (i->max - min) % step;
	if (n != 0 || i->openmax) {
		i->max -= n;
		changed = 1;
	}
	if (snd_interval_checkempty(i)) {
		i->empty = 1;
		return -EINVAL;
	}
	return changed;
}


int snd_pcm_hw_rule_add(struct snd_pcm_runtime *runtime, unsigned int cond,
			int var,
			snd_pcm_hw_rule_func_t func, void *private,
			int dep, ...)
{
	struct snd_pcm_hw_constraints *constrs = &runtime->hw_constraints;
	struct snd_pcm_hw_rule *c;
	unsigned int k;
	va_list args;
	va_start(args, dep);
	if (constrs->rules_num >= constrs->rules_all) {
		struct snd_pcm_hw_rule *new;
		unsigned int new_rules = constrs->rules_all + 16;
		new = kcalloc(new_rules, sizeof(*c), GFP_KERNEL);
		if (!new) {
			va_end(args);
			return -ENOMEM;
		}
		if (constrs->rules) {
			memcpy(new, constrs->rules,
			       constrs->rules_num * sizeof(*c));
			kfree(constrs->rules);
		}
		constrs->rules = new;
		constrs->rules_all = new_rules;
	}
	c = &constrs->rules[constrs->rules_num];
	c->cond = cond;
	c->func = func;
	c->var = var;
	c->private = private;
	k = 0;
	while (1) {
		if (snd_BUG_ON(k >= ARRAY_SIZE(c->deps))) {
			va_end(args);
			return -EINVAL;
		}
		c->deps[k++] = dep;
		if (dep < 0)
			break;
		dep = va_arg(args, int);
	}
	constrs->rules_num++;
	va_end(args);
	return 0;
}

EXPORT_SYMBOL(snd_pcm_hw_rule_add);

int snd_pcm_hw_constraint_mask(struct snd_pcm_runtime *runtime, snd_pcm_hw_param_t var,
			       u_int32_t mask)
{
	struct snd_pcm_hw_constraints *constrs = &runtime->hw_constraints;
	struct snd_mask *maskp = constrs_mask(constrs, var);
	*maskp->bits &= mask;
	memset(maskp->bits + 1, 0, (SNDRV_MASK_MAX-32) / 8); 
	if (*maskp->bits == 0)
		return -EINVAL;
	return 0;
}

int snd_pcm_hw_constraint_mask64(struct snd_pcm_runtime *runtime, snd_pcm_hw_param_t var,
				 u_int64_t mask)
{
	struct snd_pcm_hw_constraints *constrs = &runtime->hw_constraints;
	struct snd_mask *maskp = constrs_mask(constrs, var);
	maskp->bits[0] &= (u_int32_t)mask;
	maskp->bits[1] &= (u_int32_t)(mask >> 32);
	memset(maskp->bits + 2, 0, (SNDRV_MASK_MAX-64) / 8); 
	if (! maskp->bits[0] && ! maskp->bits[1])
		return -EINVAL;
	return 0;
}

int snd_pcm_hw_constraint_integer(struct snd_pcm_runtime *runtime, snd_pcm_hw_param_t var)
{
	struct snd_pcm_hw_constraints *constrs = &runtime->hw_constraints;
	return snd_interval_setinteger(constrs_interval(constrs, var));
}

EXPORT_SYMBOL(snd_pcm_hw_constraint_integer);

int snd_pcm_hw_constraint_minmax(struct snd_pcm_runtime *runtime, snd_pcm_hw_param_t var,
				 unsigned int min, unsigned int max)
{
	struct snd_pcm_hw_constraints *constrs = &runtime->hw_constraints;
	struct snd_interval t;
	t.min = min;
	t.max = max;
	t.openmin = t.openmax = 0;
	t.integer = 0;
	return snd_interval_refine(constrs_interval(constrs, var), &t);
}

EXPORT_SYMBOL(snd_pcm_hw_constraint_minmax);

static int snd_pcm_hw_rule_list(struct snd_pcm_hw_params *params,
				struct snd_pcm_hw_rule *rule)
{
	struct snd_pcm_hw_constraint_list *list = rule->private;
	return snd_interval_list(hw_param_interval(params, rule->var), list->count, list->list, list->mask);
}		


int snd_pcm_hw_constraint_list(struct snd_pcm_runtime *runtime,
			       unsigned int cond,
			       snd_pcm_hw_param_t var,
			       struct snd_pcm_hw_constraint_list *l)
{
	return snd_pcm_hw_rule_add(runtime, cond, var,
				   snd_pcm_hw_rule_list, l,
				   var, -1);
}

EXPORT_SYMBOL(snd_pcm_hw_constraint_list);

static int snd_pcm_hw_rule_ratnums(struct snd_pcm_hw_params *params,
				   struct snd_pcm_hw_rule *rule)
{
	struct snd_pcm_hw_constraint_ratnums *r = rule->private;
	unsigned int num = 0, den = 0;
	int err;
	err = snd_interval_ratnum(hw_param_interval(params, rule->var),
				  r->nrats, r->rats, &num, &den);
	if (err >= 0 && den && rule->var == SNDRV_PCM_HW_PARAM_RATE) {
		params->rate_num = num;
		params->rate_den = den;
	}
	return err;
}

int snd_pcm_hw_constraint_ratnums(struct snd_pcm_runtime *runtime, 
				  unsigned int cond,
				  snd_pcm_hw_param_t var,
				  struct snd_pcm_hw_constraint_ratnums *r)
{
	return snd_pcm_hw_rule_add(runtime, cond, var,
				   snd_pcm_hw_rule_ratnums, r,
				   var, -1);
}

EXPORT_SYMBOL(snd_pcm_hw_constraint_ratnums);

static int snd_pcm_hw_rule_ratdens(struct snd_pcm_hw_params *params,
				   struct snd_pcm_hw_rule *rule)
{
	struct snd_pcm_hw_constraint_ratdens *r = rule->private;
	unsigned int num = 0, den = 0;
	int err = snd_interval_ratden(hw_param_interval(params, rule->var),
				  r->nrats, r->rats, &num, &den);
	if (err >= 0 && den && rule->var == SNDRV_PCM_HW_PARAM_RATE) {
		params->rate_num = num;
		params->rate_den = den;
	}
	return err;
}

int snd_pcm_hw_constraint_ratdens(struct snd_pcm_runtime *runtime, 
				  unsigned int cond,
				  snd_pcm_hw_param_t var,
				  struct snd_pcm_hw_constraint_ratdens *r)
{
	return snd_pcm_hw_rule_add(runtime, cond, var,
				   snd_pcm_hw_rule_ratdens, r,
				   var, -1);
}

EXPORT_SYMBOL(snd_pcm_hw_constraint_ratdens);

static int snd_pcm_hw_rule_msbits(struct snd_pcm_hw_params *params,
				  struct snd_pcm_hw_rule *rule)
{
	unsigned int l = (unsigned long) rule->private;
	int width = l & 0xffff;
	unsigned int msbits = l >> 16;
	struct snd_interval *i = hw_param_interval(params, SNDRV_PCM_HW_PARAM_SAMPLE_BITS);
	if (snd_interval_single(i) && snd_interval_value(i) == width)
		params->msbits = msbits;
	return 0;
}

int snd_pcm_hw_constraint_msbits(struct snd_pcm_runtime *runtime, 
				 unsigned int cond,
				 unsigned int width,
				 unsigned int msbits)
{
	unsigned long l = (msbits << 16) | width;
	return snd_pcm_hw_rule_add(runtime, cond, -1,
				    snd_pcm_hw_rule_msbits,
				    (void*) l,
				    SNDRV_PCM_HW_PARAM_SAMPLE_BITS, -1);
}

EXPORT_SYMBOL(snd_pcm_hw_constraint_msbits);

static int snd_pcm_hw_rule_step(struct snd_pcm_hw_params *params,
				struct snd_pcm_hw_rule *rule)
{
	unsigned long step = (unsigned long) rule->private;
	return snd_interval_step(hw_param_interval(params, rule->var), 0, step);
}

int snd_pcm_hw_constraint_step(struct snd_pcm_runtime *runtime,
			       unsigned int cond,
			       snd_pcm_hw_param_t var,
			       unsigned long step)
{
	return snd_pcm_hw_rule_add(runtime, cond, var, 
				   snd_pcm_hw_rule_step, (void *) step,
				   var, -1);
}

EXPORT_SYMBOL(snd_pcm_hw_constraint_step);

static int snd_pcm_hw_rule_pow2(struct snd_pcm_hw_params *params, struct snd_pcm_hw_rule *rule)
{
	static unsigned int pow2_sizes[] = {
		1<<0, 1<<1, 1<<2, 1<<3, 1<<4, 1<<5, 1<<6, 1<<7,
		1<<8, 1<<9, 1<<10, 1<<11, 1<<12, 1<<13, 1<<14, 1<<15,
		1<<16, 1<<17, 1<<18, 1<<19, 1<<20, 1<<21, 1<<22, 1<<23,
		1<<24, 1<<25, 1<<26, 1<<27, 1<<28, 1<<29, 1<<30
	};
	return snd_interval_list(hw_param_interval(params, rule->var),
				 ARRAY_SIZE(pow2_sizes), pow2_sizes, 0);
}		

int snd_pcm_hw_constraint_pow2(struct snd_pcm_runtime *runtime,
			       unsigned int cond,
			       snd_pcm_hw_param_t var)
{
	return snd_pcm_hw_rule_add(runtime, cond, var, 
				   snd_pcm_hw_rule_pow2, NULL,
				   var, -1);
}

EXPORT_SYMBOL(snd_pcm_hw_constraint_pow2);

static int snd_pcm_hw_rule_noresample_func(struct snd_pcm_hw_params *params,
					   struct snd_pcm_hw_rule *rule)
{
	unsigned int base_rate = (unsigned int)(uintptr_t)rule->private;
	struct snd_interval *rate;

	rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	return snd_interval_list(rate, 1, &base_rate, 0);
}

int snd_pcm_hw_rule_noresample(struct snd_pcm_runtime *runtime,
			       unsigned int base_rate)
{
	return snd_pcm_hw_rule_add(runtime, SNDRV_PCM_HW_PARAMS_NORESAMPLE,
				   SNDRV_PCM_HW_PARAM_RATE,
				   snd_pcm_hw_rule_noresample_func,
				   (void *)(uintptr_t)base_rate,
				   SNDRV_PCM_HW_PARAM_RATE, -1);
}
EXPORT_SYMBOL(snd_pcm_hw_rule_noresample);

static void _snd_pcm_hw_param_any(struct snd_pcm_hw_params *params,
				  snd_pcm_hw_param_t var)
{
	if (hw_is_mask(var)) {
		snd_mask_any(hw_param_mask(params, var));
		params->cmask |= 1 << var;
		params->rmask |= 1 << var;
		return;
	}
	if (hw_is_interval(var)) {
		snd_interval_any(hw_param_interval(params, var));
		params->cmask |= 1 << var;
		params->rmask |= 1 << var;
		return;
	}
	snd_BUG();
}

void _snd_pcm_hw_params_any(struct snd_pcm_hw_params *params)
{
	unsigned int k;
	memset(params, 0, sizeof(*params));
	for (k = SNDRV_PCM_HW_PARAM_FIRST_MASK; k <= SNDRV_PCM_HW_PARAM_LAST_MASK; k++)
		_snd_pcm_hw_param_any(params, k);
	for (k = SNDRV_PCM_HW_PARAM_FIRST_INTERVAL; k <= SNDRV_PCM_HW_PARAM_LAST_INTERVAL; k++)
		_snd_pcm_hw_param_any(params, k);
	params->info = ~0U;
}

EXPORT_SYMBOL(_snd_pcm_hw_params_any);

int snd_pcm_hw_param_value(const struct snd_pcm_hw_params *params,
			   snd_pcm_hw_param_t var, int *dir)
{
	if (hw_is_mask(var)) {
		const struct snd_mask *mask = hw_param_mask_c(params, var);
		if (!snd_mask_single(mask))
			return -EINVAL;
		if (dir)
			*dir = 0;
		return snd_mask_value(mask);
	}
	if (hw_is_interval(var)) {
		const struct snd_interval *i = hw_param_interval_c(params, var);
		if (!snd_interval_single(i))
			return -EINVAL;
		if (dir)
			*dir = i->openmin;
		return snd_interval_value(i);
	}
	return -EINVAL;
}

EXPORT_SYMBOL(snd_pcm_hw_param_value);

void _snd_pcm_hw_param_setempty(struct snd_pcm_hw_params *params,
				snd_pcm_hw_param_t var)
{
	if (hw_is_mask(var)) {
		snd_mask_none(hw_param_mask(params, var));
		params->cmask |= 1 << var;
		params->rmask |= 1 << var;
	} else if (hw_is_interval(var)) {
		snd_interval_none(hw_param_interval(params, var));
		params->cmask |= 1 << var;
		params->rmask |= 1 << var;
	} else {
		snd_BUG();
	}
}

EXPORT_SYMBOL(_snd_pcm_hw_param_setempty);

static int _snd_pcm_hw_param_first(struct snd_pcm_hw_params *params,
				   snd_pcm_hw_param_t var)
{
	int changed;
	if (hw_is_mask(var))
		changed = snd_mask_refine_first(hw_param_mask(params, var));
	else if (hw_is_interval(var))
		changed = snd_interval_refine_first(hw_param_interval(params, var));
	else
		return -EINVAL;
	if (changed) {
		params->cmask |= 1 << var;
		params->rmask |= 1 << var;
	}
	return changed;
}


int snd_pcm_hw_param_first(struct snd_pcm_substream *pcm, 
			   struct snd_pcm_hw_params *params, 
			   snd_pcm_hw_param_t var, int *dir)
{
	int changed = _snd_pcm_hw_param_first(params, var);
	if (changed < 0)
		return changed;
	if (params->rmask) {
		int err = snd_pcm_hw_refine(pcm, params);
		if (snd_BUG_ON(err < 0))
			return err;
	}
	return snd_pcm_hw_param_value(params, var, dir);
}

EXPORT_SYMBOL(snd_pcm_hw_param_first);

static int _snd_pcm_hw_param_last(struct snd_pcm_hw_params *params,
				  snd_pcm_hw_param_t var)
{
	int changed;
	if (hw_is_mask(var))
		changed = snd_mask_refine_last(hw_param_mask(params, var));
	else if (hw_is_interval(var))
		changed = snd_interval_refine_last(hw_param_interval(params, var));
	else
		return -EINVAL;
	if (changed) {
		params->cmask |= 1 << var;
		params->rmask |= 1 << var;
	}
	return changed;
}


int snd_pcm_hw_param_last(struct snd_pcm_substream *pcm, 
			  struct snd_pcm_hw_params *params,
			  snd_pcm_hw_param_t var, int *dir)
{
	int changed = _snd_pcm_hw_param_last(params, var);
	if (changed < 0)
		return changed;
	if (params->rmask) {
		int err = snd_pcm_hw_refine(pcm, params);
		if (snd_BUG_ON(err < 0))
			return err;
	}
	return snd_pcm_hw_param_value(params, var, dir);
}

EXPORT_SYMBOL(snd_pcm_hw_param_last);

int snd_pcm_hw_params_choose(struct snd_pcm_substream *pcm,
			     struct snd_pcm_hw_params *params)
{
	static int vars[] = {
		SNDRV_PCM_HW_PARAM_ACCESS,
		SNDRV_PCM_HW_PARAM_FORMAT,
		SNDRV_PCM_HW_PARAM_SUBFORMAT,
		SNDRV_PCM_HW_PARAM_CHANNELS,
		SNDRV_PCM_HW_PARAM_RATE,
		SNDRV_PCM_HW_PARAM_PERIOD_TIME,
		SNDRV_PCM_HW_PARAM_BUFFER_SIZE,
		SNDRV_PCM_HW_PARAM_TICK_TIME,
		-1
	};
	int err, *v;

	for (v = vars; *v != -1; v++) {
		if (*v != SNDRV_PCM_HW_PARAM_BUFFER_SIZE)
			err = snd_pcm_hw_param_first(pcm, params, *v, NULL);
		else
			err = snd_pcm_hw_param_last(pcm, params, *v, NULL);
		if (snd_BUG_ON(err < 0))
			return err;
	}
	return 0;
}

static int snd_pcm_lib_ioctl_reset(struct snd_pcm_substream *substream,
				   void *arg)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long flags;
	snd_pcm_stream_lock_irqsave(substream, flags);
	if (snd_pcm_running(substream) &&
	    snd_pcm_update_hw_ptr(substream) >= 0)
		runtime->status->hw_ptr %= runtime->buffer_size;
	else
		runtime->status->hw_ptr = 0;
	snd_pcm_stream_unlock_irqrestore(substream, flags);
	return 0;
}

static int snd_pcm_lib_ioctl_channel_info(struct snd_pcm_substream *substream,
					  void *arg)
{
	struct snd_pcm_channel_info *info = arg;
	struct snd_pcm_runtime *runtime = substream->runtime;
	int width;
	if (!(runtime->info & SNDRV_PCM_INFO_MMAP)) {
		info->offset = -1;
		return 0;
	}
	width = snd_pcm_format_physical_width(runtime->format);
	if (width < 0)
		return width;
	info->offset = 0;
	switch (runtime->access) {
	case SNDRV_PCM_ACCESS_MMAP_INTERLEAVED:
	case SNDRV_PCM_ACCESS_RW_INTERLEAVED:
		if ((UINT_MAX/width) < info->channel) {
			snd_printd("%s: integer overflow while multiply\n",
				   __func__);
			return -EINVAL;
		}
		info->first = info->channel * width;
		info->step = runtime->channels * width;
		break;
	case SNDRV_PCM_ACCESS_MMAP_NONINTERLEAVED:
	case SNDRV_PCM_ACCESS_RW_NONINTERLEAVED:
	{
		size_t size = runtime->dma_bytes / runtime->channels;

		if ((size > 0) && ((UINT_MAX/(size * 8)) < info->channel)) {
			snd_printd("%s: integer overflow while multiply\n",
				   __func__);
			return -EINVAL;
		}
		info->first = info->channel * size * 8;
		info->step = width;
		break;
	}
	default:
		snd_BUG();
		break;
	}
	return 0;
}

static int snd_pcm_lib_ioctl_fifo_size(struct snd_pcm_substream *substream,
				       void *arg)
{
	struct snd_pcm_hw_params *params = arg;
	snd_pcm_format_t format;
	int channels, width;

	params->fifo_size = substream->runtime->hw.fifo_size;
	if (!(substream->runtime->hw.info & SNDRV_PCM_INFO_FIFO_IN_FRAMES)) {
		format = params_format(params);
		channels = params_channels(params);
		width = snd_pcm_format_physical_width(format);
		params->fifo_size /= width * channels;
	}
	return 0;
}

int snd_pcm_lib_ioctl(struct snd_pcm_substream *substream,
		      unsigned int cmd, void *arg)
{
	switch (cmd) {
	case SNDRV_PCM_IOCTL1_INFO:
		return 0;
	case SNDRV_PCM_IOCTL1_RESET:
		return snd_pcm_lib_ioctl_reset(substream, arg);
	case SNDRV_PCM_IOCTL1_CHANNEL_INFO:
		return snd_pcm_lib_ioctl_channel_info(substream, arg);
	case SNDRV_PCM_IOCTL1_FIFO_SIZE:
		return snd_pcm_lib_ioctl_fifo_size(substream, arg);
	}
	return -ENXIO;
}

EXPORT_SYMBOL(snd_pcm_lib_ioctl);

void snd_pcm_period_elapsed(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime;
	unsigned long flags;

	if (PCM_RUNTIME_CHECK(substream))
		return;
	runtime = substream->runtime;

	if (runtime->transfer_ack_begin)
		runtime->transfer_ack_begin(substream);

	snd_pcm_stream_lock_irqsave(substream, flags);
	if (!snd_pcm_running(substream) ||
	    snd_pcm_update_hw_ptr0(substream, 1) < 0)
		goto _end;

	if (substream->timer_running)
		snd_timer_interrupt(substream->timer, 1);
 _end:
	snd_pcm_stream_unlock_irqrestore(substream, flags);
	if (runtime->transfer_ack_end)
		runtime->transfer_ack_end(substream);
	kill_fasync(&runtime->fasync, SIGIO, POLL_IN);
}

EXPORT_SYMBOL(snd_pcm_period_elapsed);

static int wait_for_avail(struct snd_pcm_substream *substream,
			      snd_pcm_uframes_t *availp)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int is_playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	wait_queue_t wait;
	int err = 0;
	snd_pcm_uframes_t avail = 0;
	long wait_time, tout;

	init_waitqueue_entry(&wait, current);
	set_current_state(TASK_INTERRUPTIBLE);
	add_wait_queue(&runtime->tsleep, &wait);

	if (runtime->no_period_wakeup)
		wait_time = MAX_SCHEDULE_TIMEOUT;
	else {
		wait_time = 3;
		if (runtime->rate) {
			long t = runtime->period_size * 2 / runtime->rate;
			wait_time = max(t, wait_time);
		}
		wait_time = msecs_to_jiffies(wait_time * 1000);
	}

	for (;;) {
		if (signal_pending(current)) {
			err = -ERESTARTSYS;
			break;
		}

		if (is_playback)
			avail = snd_pcm_playback_avail(runtime);
		else
			avail = snd_pcm_capture_avail(runtime);
		if (avail >= runtime->twake)
			break;
		snd_pcm_stream_unlock_irq(substream);

		tout = schedule_timeout(wait_time);

		snd_pcm_stream_lock_irq(substream);
		set_current_state(TASK_INTERRUPTIBLE);
		switch (runtime->status->state) {
		case SNDRV_PCM_STATE_SUSPENDED:
			err = -ESTRPIPE;
			goto _endloop;
		case SNDRV_PCM_STATE_XRUN:
			err = -EPIPE;
			goto _endloop;
		case SNDRV_PCM_STATE_DRAINING:
			if (is_playback)
				err = -EPIPE;
			else 
				avail = 0; 
			goto _endloop;
		case SNDRV_PCM_STATE_OPEN:
		case SNDRV_PCM_STATE_SETUP:
		case SNDRV_PCM_STATE_DISCONNECTED:
			err = -EBADFD;
			goto _endloop;
		}
		if (!tout) {
			snd_printd("%s write error (DMA or IRQ trouble?)\n",
				   is_playback ? "playback" : "capture");
			err = -EIO;
			break;
		}
	}
 _endloop:
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&runtime->tsleep, &wait);
	*availp = avail;
	return err;
}
	
static int snd_pcm_lib_write_transfer(struct snd_pcm_substream *substream,
				      unsigned int hwoff,
				      unsigned long data, unsigned int off,
				      snd_pcm_uframes_t frames)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int err;
	char __user *buf = (char __user *) data + frames_to_bytes(runtime, off);
	if (substream->ops->copy) {
		if ((err = substream->ops->copy(substream, -1, hwoff, buf, frames)) < 0)
			return err;
	} else {
		char *hwbuf = runtime->dma_area + frames_to_bytes(runtime, hwoff);
		if (copy_from_user(hwbuf, buf, frames_to_bytes(runtime, frames)))
			return -EFAULT;
	}
	return 0;
}
 
typedef int (*transfer_f)(struct snd_pcm_substream *substream, unsigned int hwoff,
			  unsigned long data, unsigned int off,
			  snd_pcm_uframes_t size);

static snd_pcm_sframes_t snd_pcm_lib_write1(struct snd_pcm_substream *substream, 
					    unsigned long data,
					    snd_pcm_uframes_t size,
					    int nonblock,
					    transfer_f transfer)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t xfer = 0;
	snd_pcm_uframes_t offset = 0;
	int err = 0;

	if (size == 0)
		return 0;

	snd_pcm_stream_lock_irq(substream);
	switch (runtime->status->state) {
	case SNDRV_PCM_STATE_PREPARED:
	case SNDRV_PCM_STATE_RUNNING:
	case SNDRV_PCM_STATE_PAUSED:
		break;
	case SNDRV_PCM_STATE_XRUN:
		err = -EPIPE;
		goto _end_unlock;
	case SNDRV_PCM_STATE_SUSPENDED:
		err = -ESTRPIPE;
		goto _end_unlock;
	default:
		err = -EBADFD;
		goto _end_unlock;
	}

	runtime->twake = runtime->control->avail_min ? : 1;
	while (size > 0) {
		snd_pcm_uframes_t frames, appl_ptr, appl_ofs;
		snd_pcm_uframes_t avail;
		snd_pcm_uframes_t cont;
		if (runtime->status->state == SNDRV_PCM_STATE_RUNNING)
			snd_pcm_update_hw_ptr(substream);
		avail = snd_pcm_playback_avail(runtime);
		if (!avail) {
			if (nonblock) {
				err = -EAGAIN;
				goto _end_unlock;
			}
			runtime->twake = min_t(snd_pcm_uframes_t, size,
					runtime->control->avail_min ? : 1);
			err = wait_for_avail(substream, &avail);
			if (err < 0)
				goto _end_unlock;
		}
		frames = size > avail ? avail : size;
		cont = runtime->buffer_size - runtime->control->appl_ptr % runtime->buffer_size;
		if (frames > cont)
			frames = cont;
		if (snd_BUG_ON(!frames)) {
			runtime->twake = 0;
			snd_pcm_stream_unlock_irq(substream);
			return -EINVAL;
		}
		appl_ptr = runtime->control->appl_ptr;
		appl_ofs = appl_ptr % runtime->buffer_size;
		snd_pcm_stream_unlock_irq(substream);
		err = transfer(substream, appl_ofs, data, offset, frames);
		snd_pcm_stream_lock_irq(substream);
		if (err < 0)
			goto _end_unlock;
		switch (runtime->status->state) {
		case SNDRV_PCM_STATE_XRUN:
			err = -EPIPE;
			goto _end_unlock;
		case SNDRV_PCM_STATE_SUSPENDED:
			err = -ESTRPIPE;
			goto _end_unlock;
		default:
			break;
		}
		appl_ptr += frames;
		if (appl_ptr >= runtime->boundary)
			appl_ptr -= runtime->boundary;
		runtime->control->appl_ptr = appl_ptr;
		if (substream->ops->ack)
			substream->ops->ack(substream);

		offset += frames;
		size -= frames;
		xfer += frames;
		if (runtime->status->state == SNDRV_PCM_STATE_PREPARED &&
		    snd_pcm_playback_hw_avail(runtime) >= (snd_pcm_sframes_t)runtime->start_threshold) {
			err = snd_pcm_start(substream);
			if (err < 0)
				goto _end_unlock;
		}
	}
 _end_unlock:
	runtime->twake = 0;
	if (xfer > 0 && err >= 0)
		snd_pcm_update_state(substream, runtime);
	snd_pcm_stream_unlock_irq(substream);
	return xfer > 0 ? (snd_pcm_sframes_t)xfer : err;
}

static int pcm_sanity_check(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime;
	if (PCM_RUNTIME_CHECK(substream))
		return -ENXIO;
	
	if (substream->hw_no_buffer)
		snd_printd("%s: warning this PCM is host less\n", __func__);
	runtime = substream->runtime;
	if (snd_BUG_ON(!substream->ops->copy && !runtime->dma_area))
		return -EINVAL;
	if (runtime->status->state == SNDRV_PCM_STATE_OPEN)
		return -EBADFD;
	return 0;
}

snd_pcm_sframes_t snd_pcm_lib_write(struct snd_pcm_substream *substream, const void __user *buf, snd_pcm_uframes_t size)
{
	struct snd_pcm_runtime *runtime;
	int nonblock;
	int err;

	err = pcm_sanity_check(substream);
	if (err < 0)
		return err;
	runtime = substream->runtime;
	nonblock = !!(substream->f_flags & O_NONBLOCK);

	if (runtime->access != SNDRV_PCM_ACCESS_RW_INTERLEAVED &&
	    runtime->channels > 1)
		return -EINVAL;
	return snd_pcm_lib_write1(substream, (unsigned long)buf, size, nonblock,
				  snd_pcm_lib_write_transfer);
}

EXPORT_SYMBOL(snd_pcm_lib_write);

static int snd_pcm_lib_writev_transfer(struct snd_pcm_substream *substream,
				       unsigned int hwoff,
				       unsigned long data, unsigned int off,
				       snd_pcm_uframes_t frames)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int err;
	void __user **bufs = (void __user **)data;
	int channels = runtime->channels;
	int c;
	if (substream->ops->copy) {
		if (snd_BUG_ON(!substream->ops->silence))
			return -EINVAL;
		for (c = 0; c < channels; ++c, ++bufs) {
			if (*bufs == NULL) {
				if ((err = substream->ops->silence(substream, c, hwoff, frames)) < 0)
					return err;
			} else {
				char __user *buf = *bufs + samples_to_bytes(runtime, off);
				if ((err = substream->ops->copy(substream, c, hwoff, buf, frames)) < 0)
					return err;
			}
		}
	} else {
		
		size_t dma_csize = runtime->dma_bytes / channels;
		for (c = 0; c < channels; ++c, ++bufs) {
			char *hwbuf = runtime->dma_area + (c * dma_csize) + samples_to_bytes(runtime, hwoff);
			if (*bufs == NULL) {
				snd_pcm_format_set_silence(runtime->format, hwbuf, frames);
			} else {
				char __user *buf = *bufs + samples_to_bytes(runtime, off);
				if (copy_from_user(hwbuf, buf, samples_to_bytes(runtime, frames)))
					return -EFAULT;
			}
		}
	}
	return 0;
}
 
snd_pcm_sframes_t snd_pcm_lib_writev(struct snd_pcm_substream *substream,
				     void __user **bufs,
				     snd_pcm_uframes_t frames)
{
	struct snd_pcm_runtime *runtime;
	int nonblock;
	int err;

	err = pcm_sanity_check(substream);
	if (err < 0)
		return err;
	runtime = substream->runtime;
	nonblock = !!(substream->f_flags & O_NONBLOCK);

	if (runtime->access != SNDRV_PCM_ACCESS_RW_NONINTERLEAVED)
		return -EINVAL;
	return snd_pcm_lib_write1(substream, (unsigned long)bufs, frames,
				  nonblock, snd_pcm_lib_writev_transfer);
}

EXPORT_SYMBOL(snd_pcm_lib_writev);

static int snd_pcm_lib_read_transfer(struct snd_pcm_substream *substream, 
				     unsigned int hwoff,
				     unsigned long data, unsigned int off,
				     snd_pcm_uframes_t frames)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int err;
	char __user *buf = (char __user *) data + frames_to_bytes(runtime, off);
	if (substream->ops->copy) {
		if ((err = substream->ops->copy(substream, -1, hwoff, buf, frames)) < 0)
			return err;
	} else {
		char *hwbuf = runtime->dma_area + frames_to_bytes(runtime, hwoff);
		if (copy_to_user(buf, hwbuf, frames_to_bytes(runtime, frames)))
			return -EFAULT;
	}
	return 0;
}

static snd_pcm_sframes_t snd_pcm_lib_read1(struct snd_pcm_substream *substream,
					   unsigned long data,
					   snd_pcm_uframes_t size,
					   int nonblock,
					   transfer_f transfer)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t xfer = 0;
	snd_pcm_uframes_t offset = 0;
	int err = 0;

	if (size == 0)
		return 0;

	snd_pcm_stream_lock_irq(substream);
	switch (runtime->status->state) {
	case SNDRV_PCM_STATE_PREPARED:
		if (size >= runtime->start_threshold) {
			err = snd_pcm_start(substream);
			if (err < 0)
				goto _end_unlock;
		}
		break;
	case SNDRV_PCM_STATE_DRAINING:
	case SNDRV_PCM_STATE_RUNNING:
	case SNDRV_PCM_STATE_PAUSED:
		break;
	case SNDRV_PCM_STATE_XRUN:
		err = -EPIPE;
		goto _end_unlock;
	case SNDRV_PCM_STATE_SUSPENDED:
		err = -ESTRPIPE;
		goto _end_unlock;
	default:
		err = -EBADFD;
		goto _end_unlock;
	}

	runtime->twake = runtime->control->avail_min ? : 1;
	while (size > 0) {
		snd_pcm_uframes_t frames, appl_ptr, appl_ofs;
		snd_pcm_uframes_t avail;
		snd_pcm_uframes_t cont;
		if (runtime->status->state == SNDRV_PCM_STATE_RUNNING)
			snd_pcm_update_hw_ptr(substream);
		avail = snd_pcm_capture_avail(runtime);
		if (!avail) {
			if (runtime->status->state ==
			    SNDRV_PCM_STATE_DRAINING) {
				snd_pcm_stop(substream, SNDRV_PCM_STATE_SETUP);
				goto _end_unlock;
			}
			if (nonblock) {
				err = -EAGAIN;
				goto _end_unlock;
			}
			runtime->twake = min_t(snd_pcm_uframes_t, size,
					runtime->control->avail_min ? : 1);
			err = wait_for_avail(substream, &avail);
			if (err < 0)
				goto _end_unlock;
			if (!avail)
				continue; 
		}
		frames = size > avail ? avail : size;
		cont = runtime->buffer_size - runtime->control->appl_ptr % runtime->buffer_size;
		if (frames > cont)
			frames = cont;
		if (snd_BUG_ON(!frames)) {
			runtime->twake = 0;
			snd_pcm_stream_unlock_irq(substream);
			return -EINVAL;
		}
		appl_ptr = runtime->control->appl_ptr;
		appl_ofs = appl_ptr % runtime->buffer_size;
		snd_pcm_stream_unlock_irq(substream);
		err = transfer(substream, appl_ofs, data, offset, frames);
		snd_pcm_stream_lock_irq(substream);
		if (err < 0)
			goto _end_unlock;
		switch (runtime->status->state) {
		case SNDRV_PCM_STATE_XRUN:
			err = -EPIPE;
			goto _end_unlock;
		case SNDRV_PCM_STATE_SUSPENDED:
			err = -ESTRPIPE;
			goto _end_unlock;
		default:
			break;
		}
		appl_ptr += frames;
		if (appl_ptr >= runtime->boundary)
			appl_ptr -= runtime->boundary;
		runtime->control->appl_ptr = appl_ptr;
		if (substream->ops->ack)
			substream->ops->ack(substream);

		offset += frames;
		size -= frames;
		xfer += frames;
	}
 _end_unlock:
	runtime->twake = 0;
	if (xfer > 0 && err >= 0)
		snd_pcm_update_state(substream, runtime);
	snd_pcm_stream_unlock_irq(substream);
	return xfer > 0 ? (snd_pcm_sframes_t)xfer : err;
}

snd_pcm_sframes_t snd_pcm_lib_read(struct snd_pcm_substream *substream, void __user *buf, snd_pcm_uframes_t size)
{
	struct snd_pcm_runtime *runtime;
	int nonblock;
	int err;
	
	err = pcm_sanity_check(substream);
	if (err < 0)
		return err;
	runtime = substream->runtime;
	nonblock = !!(substream->f_flags & O_NONBLOCK);
	if (runtime->access != SNDRV_PCM_ACCESS_RW_INTERLEAVED)
		return -EINVAL;
	return snd_pcm_lib_read1(substream, (unsigned long)buf, size, nonblock, snd_pcm_lib_read_transfer);
}

EXPORT_SYMBOL(snd_pcm_lib_read);

static int snd_pcm_lib_readv_transfer(struct snd_pcm_substream *substream,
				      unsigned int hwoff,
				      unsigned long data, unsigned int off,
				      snd_pcm_uframes_t frames)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int err;
	void __user **bufs = (void __user **)data;
	int channels = runtime->channels;
	int c;
	if (substream->ops->copy) {
		for (c = 0; c < channels; ++c, ++bufs) {
			char __user *buf;
			if (*bufs == NULL)
				continue;
			buf = *bufs + samples_to_bytes(runtime, off);
			if ((err = substream->ops->copy(substream, c, hwoff, buf, frames)) < 0)
				return err;
		}
	} else {
		snd_pcm_uframes_t dma_csize = runtime->dma_bytes / channels;
		for (c = 0; c < channels; ++c, ++bufs) {
			char *hwbuf;
			char __user *buf;
			if (*bufs == NULL)
				continue;

			hwbuf = runtime->dma_area + (c * dma_csize) + samples_to_bytes(runtime, hwoff);
			buf = *bufs + samples_to_bytes(runtime, off);
			if (copy_to_user(buf, hwbuf, samples_to_bytes(runtime, frames)))
				return -EFAULT;
		}
	}
	return 0;
}
 
snd_pcm_sframes_t snd_pcm_lib_readv(struct snd_pcm_substream *substream,
				    void __user **bufs,
				    snd_pcm_uframes_t frames)
{
	struct snd_pcm_runtime *runtime;
	int nonblock;
	int err;

	err = pcm_sanity_check(substream);
	if (err < 0)
		return err;
	runtime = substream->runtime;
	if (runtime->status->state == SNDRV_PCM_STATE_OPEN)
		return -EBADFD;

	nonblock = !!(substream->f_flags & O_NONBLOCK);
	if (runtime->access != SNDRV_PCM_ACCESS_RW_NONINTERLEAVED)
		return -EINVAL;
	return snd_pcm_lib_read1(substream, (unsigned long)bufs, frames, nonblock, snd_pcm_lib_readv_transfer);
}

EXPORT_SYMBOL(snd_pcm_lib_readv);


const struct snd_pcm_chmap_elem snd_pcm_std_chmaps[] = {
	{ .channels = 1,
	  .map = { SNDRV_CHMAP_FC } },
	{ .channels = 2,
	  .map = { SNDRV_CHMAP_FL, SNDRV_CHMAP_FR } },
	{ .channels = 4,
	  .map = { SNDRV_CHMAP_FL, SNDRV_CHMAP_FR,
		   SNDRV_CHMAP_RL, SNDRV_CHMAP_RR } },
	{ .channels = 6,
	  .map = { SNDRV_CHMAP_FL, SNDRV_CHMAP_FR,
		   SNDRV_CHMAP_RL, SNDRV_CHMAP_RR,
		   SNDRV_CHMAP_FC, SNDRV_CHMAP_LFE } },
	{ .channels = 8,
	  .map = { SNDRV_CHMAP_FL, SNDRV_CHMAP_FR,
		   SNDRV_CHMAP_RL, SNDRV_CHMAP_RR,
		   SNDRV_CHMAP_FC, SNDRV_CHMAP_LFE,
		   SNDRV_CHMAP_SL, SNDRV_CHMAP_SR } },
	{ }
};
EXPORT_SYMBOL_GPL(snd_pcm_std_chmaps);

const struct snd_pcm_chmap_elem snd_pcm_alt_chmaps[] = {
	{ .channels = 1,
	  .map = { SNDRV_CHMAP_FC } },
	{ .channels = 2,
	  .map = { SNDRV_CHMAP_FL, SNDRV_CHMAP_FR } },
	{ .channels = 4,
	  .map = { SNDRV_CHMAP_FL, SNDRV_CHMAP_FR,
		   SNDRV_CHMAP_RL, SNDRV_CHMAP_RR } },
	{ .channels = 6,
	  .map = { SNDRV_CHMAP_FL, SNDRV_CHMAP_FR,
		   SNDRV_CHMAP_FC, SNDRV_CHMAP_LFE,
		   SNDRV_CHMAP_RL, SNDRV_CHMAP_RR } },
	{ .channels = 8,
	  .map = { SNDRV_CHMAP_FL, SNDRV_CHMAP_FR,
		   SNDRV_CHMAP_FC, SNDRV_CHMAP_LFE,
		   SNDRV_CHMAP_RL, SNDRV_CHMAP_RR,
		   SNDRV_CHMAP_SL, SNDRV_CHMAP_SR } },
	{ }
};
EXPORT_SYMBOL_GPL(snd_pcm_alt_chmaps);

static bool valid_chmap_channels(const struct snd_pcm_chmap *info, int ch)
{
	if (ch > info->max_channels)
		return false;
	return !info->channel_mask || (info->channel_mask & (1U << ch));
}

static int pcm_chmap_ctl_info(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_info *uinfo)
{
	struct snd_pcm_chmap *info = snd_kcontrol_chip(kcontrol);

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 0;
	uinfo->count = info->max_channels;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = SNDRV_CHMAP_LAST;
	return 0;
}

static int pcm_chmap_ctl_get(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_pcm_chmap *info = snd_kcontrol_chip(kcontrol);
	unsigned int idx = snd_ctl_get_ioffidx(kcontrol, &ucontrol->id);
	struct snd_pcm_substream *substream;
	const struct snd_pcm_chmap_elem *map;

	if (snd_BUG_ON(!info->chmap))
		return -EINVAL;
	substream = snd_pcm_chmap_substream(info, idx);
	if (!substream)
		return -ENODEV;
	memset(ucontrol->value.integer.value, 0,
	       sizeof(ucontrol->value.integer.value));
	if (!substream->runtime)
		return 0; 
	for (map = info->chmap; map->channels; map++) {
		int i;
		if (map->channels == substream->runtime->channels &&
		    valid_chmap_channels(info, map->channels)) {
			for (i = 0; i < map->channels; i++)
				ucontrol->value.integer.value[i] = map->map[i];
			return 0;
		}
	}
	return -EINVAL;
}

static int pcm_chmap_ctl_tlv(struct snd_kcontrol *kcontrol, int op_flag,
			     unsigned int size, unsigned int __user *tlv)
{
	struct snd_pcm_chmap *info = snd_kcontrol_chip(kcontrol);
	const struct snd_pcm_chmap_elem *map;
	unsigned int __user *dst;
	int c, count = 0;

	if (snd_BUG_ON(!info->chmap))
		return -EINVAL;
	if (size < 8)
		return -ENOMEM;
	if (put_user(SNDRV_CTL_TLVT_CONTAINER, tlv))
		return -EFAULT;
	size -= 8;
	dst = tlv + 2;
	for (map = info->chmap; map->channels; map++) {
		int chs_bytes = map->channels * 4;
		if (!valid_chmap_channels(info, map->channels))
			continue;
		if (size < 8)
			return -ENOMEM;
		if (put_user(SNDRV_CTL_TLVT_CHMAP_FIXED, dst) ||
		    put_user(chs_bytes, dst + 1))
			return -EFAULT;
		dst += 2;
		size -= 8;
		count += 8;
		if (size < chs_bytes)
			return -ENOMEM;
		size -= chs_bytes;
		count += chs_bytes;
		for (c = 0; c < map->channels; c++) {
			if (put_user(map->map[c], dst))
				return -EFAULT;
			dst++;
		}
	}
	if (put_user(count, tlv + 1))
		return -EFAULT;
	return 0;
}

static void pcm_chmap_ctl_private_free(struct snd_kcontrol *kcontrol)
{
	struct snd_pcm_chmap *info = snd_kcontrol_chip(kcontrol);
	info->pcm->streams[info->stream].chmap_kctl = NULL;
	kfree(info);
}

static int pcm_volume_ctl_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0x2000;
	return 0;
}

static void pcm_volume_ctl_private_free(struct snd_kcontrol *kcontrol)
{
	struct snd_pcm_volume *info = snd_kcontrol_chip(kcontrol);
	info->pcm->streams[info->stream].vol_kctl = NULL;
	kfree(info);
}

int snd_pcm_add_chmap_ctls(struct snd_pcm *pcm, int stream,
			   const struct snd_pcm_chmap_elem *chmap,
			   int max_channels,
			   unsigned long private_value,
			   struct snd_pcm_chmap **info_ret)
{
	struct snd_pcm_chmap *info;
	struct snd_kcontrol_new knew = {
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.access = SNDRV_CTL_ELEM_ACCESS_READ |
			SNDRV_CTL_ELEM_ACCESS_TLV_READ |
			SNDRV_CTL_ELEM_ACCESS_TLV_CALLBACK,
		.info = pcm_chmap_ctl_info,
		.get = pcm_chmap_ctl_get,
		.tlv.c = pcm_chmap_ctl_tlv,
	};
	int err;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->pcm = pcm;
	info->stream = stream;
	info->chmap = chmap;
	info->max_channels = max_channels;
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		knew.name = "Playback Channel Map";
	else
		knew.name = "Capture Channel Map";
	knew.device = pcm->device;
	knew.count = pcm->streams[stream].substream_count;
	knew.private_value = private_value;
	info->kctl = snd_ctl_new1(&knew, info);
	if (!info->kctl) {
		kfree(info);
		return -ENOMEM;
	}
	info->kctl->private_free = pcm_chmap_ctl_private_free;
	err = snd_ctl_add(pcm->card, info->kctl);
	if (err < 0)
		return err;
	pcm->streams[stream].chmap_kctl = info->kctl;
	if (info_ret)
		*info_ret = info;
	return 0;
}
EXPORT_SYMBOL_GPL(snd_pcm_add_chmap_ctls);

int snd_pcm_add_volume_ctls(struct snd_pcm *pcm, int stream,
			   const struct snd_pcm_volume_elem *volume,
			   int max_length,
			   unsigned long private_value,
			   struct snd_pcm_volume **info_ret)
{
	struct snd_pcm_volume *info;
	struct snd_kcontrol_new knew = {
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |
			SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = pcm_volume_ctl_info,
	};
	int err;
	int size;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->pcm = pcm;
	info->stream = stream;
	info->volume = volume;
	info->max_length = max_length;
	size = sizeof("Playback ") + sizeof(" Volume") +
		STRING_LENGTH_OF_INT*sizeof(char) + 1;
	knew.name = kzalloc(size, GFP_KERNEL);
	if (!knew.name) {
		kfree(info);
		return -ENOMEM;
	}
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		snprintf(knew.name, size, "%s %d %s",
			"Playback", pcm->device, "Volume");
	else
		snprintf(knew.name, size, "%s %d %s",
			"Capture", pcm->device, "Volume");
	knew.device = pcm->device;
	knew.count = pcm->streams[stream].substream_count;
	knew.private_value = private_value;
	info->kctl = snd_ctl_new1(&knew, info);
	if (!info->kctl) {
		kfree(info);
		kfree(knew.name);
		return -ENOMEM;
	}
	info->kctl->private_free = pcm_volume_ctl_private_free;
	err = snd_ctl_add(pcm->card, info->kctl);
	if (err < 0) {
		kfree(info);
		kfree(knew.name);
		return -ENOMEM;
	}
	pcm->streams[stream].vol_kctl = info->kctl;
	if (info_ret)
		*info_ret = info;
	kfree(knew.name);
	return 0;
}
EXPORT_SYMBOL_GPL(snd_pcm_add_volume_ctls);
