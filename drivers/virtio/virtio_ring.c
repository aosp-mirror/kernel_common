/* Virtio ring implementation.
 *
 *  Copyright 2007 Rusty Russell IBM Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include <linux/virtio.h>
#include <linux/virtio_ring.h>
#include <linux/virtio_config.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/hrtimer.h>

/* virtio guest is communicating with a virtual "device" that actually runs on
 * a host processor.  Memory barriers are used to control SMP effects. */
#ifdef CONFIG_SMP
/* Where possible, use SMP barriers which are more lightweight than mandatory
 * barriers, because mandatory barriers control MMIO effects on accesses
 * through relaxed memory I/O windows (which virtio-pci does not use). */
#define virtio_mb(vq) \
	do { if ((vq)->weak_barriers) smp_mb(); else mb(); } while(0)
#define virtio_rmb(vq) \
	do { if ((vq)->weak_barriers) smp_rmb(); else rmb(); } while(0)
#define virtio_wmb(vq) \
	do { if ((vq)->weak_barriers) smp_wmb(); else wmb(); } while(0)
#else
/* We must force memory ordering even if guest is UP since host could be
 * running on another CPU, but SMP barriers are defined to barrier() in that
 * configuration. So fall back to mandatory barriers instead. */
#define virtio_mb(vq) mb()
#define virtio_rmb(vq) rmb()
#define virtio_wmb(vq) wmb()
#endif

#ifdef DEBUG
/* For development, we want to crash whenever the ring is screwed. */
#define BAD_RING(_vq, fmt, args...)				\
	do {							\
		dev_err(&(_vq)->vq.vdev->dev,			\
			"%s:"fmt, (_vq)->vq.name, ##args);	\
		BUG();						\
	} while (0)
/* Caller is supposed to guarantee no reentry. */
#define START_USE(_vq)						\
	do {							\
		if ((_vq)->in_use)				\
			panic("%s:in_use = %i\n",		\
			      (_vq)->vq.name, (_vq)->in_use);	\
		(_vq)->in_use = __LINE__;			\
	} while (0)
#define END_USE(_vq) \
	do { BUG_ON(!(_vq)->in_use); (_vq)->in_use = 0; } while(0)
#else
#define BAD_RING(_vq, fmt, args...)				\
	do {							\
		dev_err(&_vq->vq.vdev->dev,			\
			"%s:"fmt, (_vq)->vq.name, ##args);	\
		(_vq)->broken = true;				\
	} while (0)
#define START_USE(vq)
#define END_USE(vq)
#endif

struct vring_virtqueue
{
	struct virtqueue vq;

	/* Actual memory layout for this queue */
	struct vring vring;

	/* Can we use weak barriers? */
	bool weak_barriers;

	/* Other side has made a mess, don't try any more. */
	bool broken;

	/* Host supports indirect buffers */
	bool indirect;

	/* Host publishes avail event idx */
	bool event;

	/* Number of free buffers */
	unsigned int num_free;
	/* Head of free buffer list. */
	unsigned int free_head;
	/* Number we've added since last sync. */
	unsigned int num_added;

	/* Last used index we've seen. */
	u16 last_used_idx;

	/* How to notify other side. FIXME: commonalize hcalls! */
	void (*notify)(struct virtqueue *vq);

#ifdef DEBUG
	/* They're supposed to lock for us. */
	unsigned int in_use;

	/* Figure out if their kicks are too delayed. */
	bool last_add_time_valid;
	ktime_t last_add_time;
#endif

	/* Tokens for callbacks. */
	void *data[];
};

#define to_vvq(_vq) container_of(_vq, struct vring_virtqueue, vq)

/* Set up an indirect table of descriptors and add it to the queue. */
static int vring_add_indirect(struct vring_virtqueue *vq,
			      struct scatterlist sg[],
			      unsigned int out,
			      unsigned int in,
			      gfp_t gfp)
{
	struct vring_desc *desc;
	unsigned head;
	int i;

	desc = kmalloc((out + in) * sizeof(struct vring_desc), gfp);
	if (!desc)
		return -ENOMEM;

	/* Transfer entries from the sg list into the indirect page */
	for (i = 0; i < out; i++) {
		desc[i].flags = VRING_DESC_F_NEXT;
		desc[i].addr = sg_phys(sg);
		desc[i].len = sg->length;
		desc[i].next = i+1;
		sg++;
	}
	for (; i < (out + in); i++) {
		desc[i].flags = VRING_DESC_F_NEXT|VRING_DESC_F_WRITE;
		desc[i].addr = sg_phys(sg);
		desc[i].len = sg->length;
		desc[i].next = i+1;
		sg++;
	}

	/* Last one doesn't continue. */
	desc[i-1].flags &= ~VRING_DESC_F_NEXT;
	desc[i-1].next = 0;

	/* We're about to use a buffer */
	vq->num_free--;

	/* Use a single buffer which doesn't continue */
	head = vq->free_head;
	vq->vring.desc[head].flags = VRING_DESC_F_INDIRECT;
	vq->vring.desc[head].addr = virt_to_phys(desc);
	vq->vring.desc[head].len = i * sizeof(struct vring_desc);

	/* Update free pointer */
	vq->free_head = vq->vring.desc[head].next;

	return head;
}

/**
 * vring_add_buf - expose buffer to other end
 * @vq: the struct virtqueue we're talking about.
 * @sg: the description of the buffer(s).
 * @out_num: the number of sg readable by other side
 * @in_num: the number of sg which are writable (after readable ones)
 * @data: the token identifying the buffer.
 * @gfp: how to do memory allocations (if necessary).
 *
 * Caller must ensure we don't call this with other virtqueue operations
 * at the same time (except where noted).
 *
 * Returns remaining capacity of queue or a negative error
 * (ie. ENOSPC).  Note that it only really makes sense to treat all
 * positive return values as "available": indirect buffers mean that
 * we can put an entire sg[] array inside a single queue entry.
 */
static int vring_add_buf(struct virtqueue *_vq,
		      struct scatterlist sg[],
		      unsigned int out,
		      unsigned int in,
		      void *data,
		      gfp_t gfp)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	unsigned int i, avail, uninitialized_var(prev);
	int head;

	START_USE(vq);

	BUG_ON(data == NULL);

#ifdef DEBUG
	{
		ktime_t now = ktime_get();

		/* No kick or get, with .1 second between?  Warn. */
		if (vq->last_add_time_valid)
			WARN_ON(ktime_to_ms(ktime_sub(now, vq->last_add_time))
					    > 100);
		vq->last_add_time = now;
		vq->last_add_time_valid = true;
	}
#endif

	/* If the host supports indirect descriptor tables, and we have multiple
	 * buffers, then go indirect. FIXME: tune this threshold */
	if (vq->indirect && (out + in) > 1 && vq->num_free) {
		head = vring_add_indirect(vq, sg, out, in, gfp);
		if (likely(head >= 0))
			goto add_head;
	}

	BUG_ON(out + in > vq->vring.num);
	BUG_ON(out + in == 0);

	if (vq->num_free < out + in) {
		pr_debug("Can't add buf len %i - avail = %i\n",
			 out + in, vq->num_free);
		/* FIXME: for historical reasons, we force a notify here if
		 * there are outgoing parts to the buffer.  Presumably the
		 * host should service the ring ASAP. */
		if (out)
			vq->notify(&vq->vq);
		END_USE(vq);
		return -ENOSPC;
	}

	/* We're about to use some buffers from the free list. */
	vq->num_free -= out + in;

	head = vq->free_head;
	for (i = vq->free_head; out; i = vq->vring.desc[i].next, out--) {
		vq->vring.desc[i].flags = VRING_DESC_F_NEXT;
		vq->vring.desc[i].addr = sg_phys(sg);
		vq->vring.desc[i].len = sg->length;
		prev = i;
		sg++;
	}
	for (; in; i = vq->vring.desc[i].next, in--) {
		vq->vring.desc[i].flags = VRING_DESC_F_NEXT|VRING_DESC_F_WRITE;
		vq->vring.desc[i].addr = sg_phys(sg);
		vq->vring.desc[i].len = sg->length;
		prev = i;
		sg++;
	}
	/* Last one doesn't continue. */
	vq->vring.desc[prev].flags &= ~VRING_DESC_F_NEXT;

	/* Update free pointer */
	vq->free_head = i;

add_head:
	/* Set token. */
	vq->data[head] = data;

	/* Put entry in available array (but don't update avail->idx until they
	 * do sync). */
	avail = (vq->vring.avail->idx & (vq->vring.num-1));
	vq->vring.avail->ring[avail] = head;

	/* Descriptors and available array need to be set before we expose the
	 * new available array entries. */
	virtio_wmb(vq);
	vq->vring.avail->idx++;
	vq->num_added++;

	/* This is very unlikely, but theoretically possible.  Kick
	 * just in case. */
	if (unlikely(vq->num_added == (1 << 16) - 1))
		virtqueue_kick(_vq);

	pr_debug("Added buffer head %i to %p\n", head, vq);
	END_USE(vq);

	return vq->num_free;
}

/**
 * vring_kick_prepare - first half of split vring_kick call.
 * @vq: the struct virtqueue
 *
 * Instead of vring_kick(), you can do:
 *	if (vring_kick_prepare(vq))
 *		vring_kick_notify(vq);
 *
 * This is sometimes useful because the vring_kick_prepare() needs
 * to be serialized, but the actual vring_kick_notify() call does not.
 */
static bool vring_kick_prepare(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	u16 new, old;
	bool needs_kick;

	START_USE(vq);
	/* We need to expose available array entries before checking avail
	 * event. */
	virtio_mb(vq);

	old = vq->vring.avail->idx - vq->num_added;
	new = vq->vring.avail->idx;
	vq->num_added = 0;

#ifdef DEBUG
	if (vq->last_add_time_valid) {
		WARN_ON(ktime_to_ms(ktime_sub(ktime_get(),
					      vq->last_add_time)) > 100);
	}
	vq->last_add_time_valid = false;
#endif

	if (vq->event) {
		needs_kick = vring_need_event(vring_avail_event(&vq->vring),
					      new, old);
	} else {
		needs_kick = !(vq->vring.used->flags & VRING_USED_F_NO_NOTIFY);
	}
	END_USE(vq);
	return needs_kick;
}

/**
 * vring_kick_notify - second half of split virtqueue_kick call.
 * @vq: the struct virtqueue
 *
 * This does not need to be serialized.
 */
static void vring_kick_notify(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	/* Prod other side to tell it about changes. */
	vq->notify(_vq);
}

/**
 * vring_kick - update after add_buf
 * @vq: the struct virtqueue
 *
 * After one or more vring_add_buf calls, invoke this to kick
 * the other side.
 *
 * Caller must ensure we don't call this with other virtqueue
 * operations at the same time (except where noted).
 */
static void vring_kick(struct virtqueue *vq)
{
	if (vring_kick_prepare(vq))
		vring_kick_notify(vq);
}

static void detach_buf(struct vring_virtqueue *vq, unsigned int head)
{
	unsigned int i;

	/* Clear data ptr. */
	vq->data[head] = NULL;

	/* Put back on free list: find end */
	i = head;

	/* Free the indirect table */
	if (vq->vring.desc[i].flags & VRING_DESC_F_INDIRECT)
		kfree(phys_to_virt(vq->vring.desc[i].addr));

	while (vq->vring.desc[i].flags & VRING_DESC_F_NEXT) {
		i = vq->vring.desc[i].next;
		vq->num_free++;
	}

	vq->vring.desc[i].next = vq->free_head;
	vq->free_head = head;
	/* Plus final descriptor */
	vq->num_free++;
}

static inline bool more_used(const struct vring_virtqueue *vq)
{
	return vq->last_used_idx != vq->vring.used->idx;
}

/**
 * vring_get_buf - get the next used buffer
 * @vq: the struct virtqueue we're talking about.
 * @len: the length written into the buffer
 *
 * If the driver wrote data into the buffer, @len will be set to the
 * amount written.  This means you don't need to clear the buffer
 * beforehand to ensure there's no data leakage in the case of short
 * writes.
 *
 * Caller must ensure we don't call this with other virtqueue
 * operations at the same time (except where noted).
 *
 * Returns NULL if there are no used buffers, or the "data" token
 * handed to vring_add_buf().
 */
static void *vring_get_buf(struct virtqueue *_vq, unsigned int *len)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	void *ret;
	unsigned int i;
	u16 last_used;

	START_USE(vq);

	if (unlikely(vq->broken)) {
		END_USE(vq);
		return NULL;
	}

	if (!more_used(vq)) {
		pr_debug("No more buffers in queue\n");
		END_USE(vq);
		return NULL;
	}

	/* Only get used array entries after they have been exposed by host. */
	virtio_rmb(vq);

	last_used = (vq->last_used_idx & (vq->vring.num - 1));
	i = vq->vring.used->ring[last_used].id;
	*len = vq->vring.used->ring[last_used].len;

	if (unlikely(i >= vq->vring.num)) {
		BAD_RING(vq, "id %u out of range\n", i);
		return NULL;
	}
	if (unlikely(!vq->data[i])) {
		BAD_RING(vq, "id %u is not a head!\n", i);
		return NULL;
	}

	/* detach_buf clears data, so grab it now. */
	ret = vq->data[i];
	detach_buf(vq, i);
	vq->last_used_idx++;
	/* If we expect an interrupt for the next entry, tell host
	 * by writing event index and flush out the write before
	 * the read in the next get_buf call. */
	if (!(vq->vring.avail->flags & VRING_AVAIL_F_NO_INTERRUPT)) {
		vring_used_event(&vq->vring) = vq->last_used_idx;
		virtio_mb(vq);
	}

#ifdef DEBUG
	vq->last_add_time_valid = false;
#endif

	END_USE(vq);
	return ret;
}

/**
 * vring_disable_cb - disable callbacks
 * @vq: the struct virtqueue we're talking about.
 *
 * Note that this is not necessarily synchronous, hence unreliable and only
 * useful as an optimization.
 *
 * Unlike other operations, this need not be serialized.
 */
static void vring_disable_cb(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	vq->vring.avail->flags |= VRING_AVAIL_F_NO_INTERRUPT;
}

/**
 * vring_enable_cb - restart callbacks after disable_cb.
 * @vq: the struct virtqueue we're talking about.
 *
 * This re-enables callbacks; it returns "false" if there are pending
 * buffers in the queue, to detect a possible race between the driver
 * checking for more work, and enabling callbacks.
 *
 * Caller must ensure we don't call this with other virtqueue
 * operations at the same time (except where noted).
 */
static bool vring_enable_cb(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	START_USE(vq);

	/* We optimistically turn back on interrupts, then check if there was
	 * more to do. */
	/* Depending on the VIRTIO_RING_F_EVENT_IDX feature, we need to
	 * either clear the flags bit or point the event index at the next
	 * entry. Always do both to keep code simple. */
	vq->vring.avail->flags &= ~VRING_AVAIL_F_NO_INTERRUPT;
	vring_used_event(&vq->vring) = vq->last_used_idx;
	virtio_mb(vq);
	if (unlikely(more_used(vq))) {
		END_USE(vq);
		return false;
	}

	END_USE(vq);
	return true;
}

/**
 * vring_enable_cb_delayed - restart callbacks after disable_cb.
 * @vq: the struct virtqueue we're talking about.
 *
 * This re-enables callbacks but hints to the other side to delay
 * interrupts until most of the available buffers have been processed;
 * it returns "false" if there are many pending buffers in the queue,
 * to detect a possible race between the driver checking for more work,
 * and enabling callbacks.
 *
 * Caller must ensure we don't call this with other virtqueue
 * operations at the same time (except where noted).
 */
static bool vring_enable_cb_delayed(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	u16 bufs;

	START_USE(vq);

	/* We optimistically turn back on interrupts, then check if there was
	 * more to do. */
	/* Depending on the VIRTIO_RING_F_USED_EVENT_IDX feature, we need to
	 * either clear the flags bit or point the event index at the next
	 * entry. Always do both to keep code simple. */
	vq->vring.avail->flags &= ~VRING_AVAIL_F_NO_INTERRUPT;
	/* TODO: tune this threshold */
	bufs = (u16)(vq->vring.avail->idx - vq->last_used_idx) * 3 / 4;
	vring_used_event(&vq->vring) = vq->last_used_idx + bufs;
	virtio_mb(vq);
	if (unlikely((u16)(vq->vring.used->idx - vq->last_used_idx) > bufs)) {
		END_USE(vq);
		return false;
	}

	END_USE(vq);
	return true;
}

/**
 * vring_detach_unused_buf - detach first unused buffer
 * @vq: the struct virtqueue we're talking about.
 *
 * Returns NULL or the "data" token handed to vring_add_buf().
 * This is not valid on an active queue; it is useful only for device
 * shutdown.
 */
static void *vring_detach_unused_buf(struct virtqueue *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);
	unsigned int i;
	void *buf;

	START_USE(vq);

	for (i = 0; i < vq->vring.num; i++) {
		if (!vq->data[i])
			continue;
		/* detach_buf clears data, so grab it now. */
		buf = vq->data[i];
		detach_buf(vq, i);
		vq->vring.avail->idx--;
		END_USE(vq);
		return buf;
	}
	/* That should have freed everything. */
	BUG_ON(vq->num_free != vq->vring.num);

	END_USE(vq);
	return NULL;
}

irqreturn_t vring_interrupt(int irq, void *_vq)
{
	struct vring_virtqueue *vq = to_vvq(_vq);

	if (!more_used(vq)) {
		pr_debug("virtqueue interrupt with no work for %p\n", vq);
		return IRQ_NONE;
	}

	if (unlikely(vq->broken))
		return IRQ_HANDLED;

	pr_debug("virtqueue callback for %p (%p)\n", vq, vq->vq.callback);
	if (vq->vq.callback)
		vq->vq.callback(&vq->vq);

	return IRQ_HANDLED;
}
EXPORT_SYMBOL_GPL(vring_interrupt);

/**
 * get_vring_size - return the size of the virtqueue's vring
 * @vq: the struct virtqueue containing the vring of interest.
 *
 * Returns the size of the vring.  This is mainly used for boasting to
 * userspace.  Unlike other operations, this need not be serialized.
 */
static unsigned int get_vring_size(struct virtqueue *_vq)
{

	struct vring_virtqueue *vq = to_vvq(_vq);

	return vq->vring.num;
}

static struct virtqueue_ops vring_vq_ops = {
	.add_buf = vring_add_buf,
	.get_buf = vring_get_buf,
	.kick = vring_kick,
	.kick_prepare = vring_kick_prepare,
	.kick_notify = vring_kick_notify,
	.disable_cb = vring_disable_cb,
	.enable_cb = vring_enable_cb,
	.enable_cb_delayed = vring_enable_cb_delayed,
	.detach_unused_buf = vring_detach_unused_buf,
	.get_impl_size = get_vring_size,
};

struct virtqueue *vring_new_virtqueue(unsigned int num,
				      unsigned int vring_align,
				      struct virtio_device *vdev,
				      bool weak_barriers,
				      void *pages,
				      void (*notify)(struct virtqueue *),
				      void (*callback)(struct virtqueue *),
				      const char *name)
{
	struct vring_virtqueue *vq;
	unsigned int i;

	/* We assume num is a power of 2. */
	if (num & (num - 1)) {
		dev_warn(&vdev->dev, "Bad virtqueue length %u\n", num);
		return NULL;
	}

	vq = kmalloc(sizeof(*vq) + sizeof(void *)*num, GFP_KERNEL);
	if (!vq)
		return NULL;

	vring_init(&vq->vring, num, pages, vring_align);
	vq->vq.callback = callback;
	vq->vq.vdev = vdev;
	vq->vq.vq_ops = &vring_vq_ops;
	vq->vq.name = name;
	vq->notify = notify;
	vq->weak_barriers = weak_barriers;
	vq->broken = false;
	vq->last_used_idx = 0;
	vq->num_added = 0;
	list_add_tail(&vq->vq.list, &vdev->vqs);
#ifdef DEBUG
	vq->in_use = false;
	vq->last_add_time_valid = false;
#endif

	vq->indirect = virtio_has_feature(vdev, VIRTIO_RING_F_INDIRECT_DESC);
	vq->event = virtio_has_feature(vdev, VIRTIO_RING_F_EVENT_IDX);

	/* No callback?  Tell other side not to bother us. */
	if (!callback)
		vq->vring.avail->flags |= VRING_AVAIL_F_NO_INTERRUPT;

	/* Put everything in free lists. */
	vq->num_free = num;
	vq->free_head = 0;
	for (i = 0; i < num-1; i++) {
		vq->vring.desc[i].next = i+1;
		vq->data[i] = NULL;
	}
	vq->data[i] = NULL;

	return &vq->vq;
}
EXPORT_SYMBOL_GPL(vring_new_virtqueue);

void vring_del_virtqueue(struct virtqueue *vq)
{
	list_del(&vq->list);
	kfree(to_vvq(vq));
}
EXPORT_SYMBOL_GPL(vring_del_virtqueue);

/* Manipulates transport-specific feature bits. */
void vring_transport_features(struct virtio_device *vdev)
{
	unsigned int i;

	for (i = VIRTIO_TRANSPORT_F_START; i < VIRTIO_TRANSPORT_F_END; i++) {
		switch (i) {
		case VIRTIO_RING_F_INDIRECT_DESC:
			break;
		case VIRTIO_RING_F_EVENT_IDX:
			break;
		default:
			/* We don't understand this bit. */
			clear_bit(i, vdev->features);
		}
	}
}
EXPORT_SYMBOL_GPL(vring_transport_features);

MODULE_LICENSE("GPL");
