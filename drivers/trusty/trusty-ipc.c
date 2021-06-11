// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/aio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/idr.h>
#include <linux/completion.h>
#include <linux/dma-buf.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/compat.h>
#include <linux/uio.h>
#include <linux/file.h>

#include <linux/virtio.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_config.h>

#include <linux/trusty/trusty.h>
#include <linux/trusty/trusty_ipc.h>

#include <uapi/linux/trusty/ipc.h>

#define MAX_DEVICES			4

#define REPLY_TIMEOUT			5000
#define TXBUF_TIMEOUT			15000

#define MAX_SRV_NAME_LEN		256
#define MAX_DEV_NAME_LEN		32

#define DEFAULT_MSG_BUF_SIZE		PAGE_SIZE
#define DEFAULT_MSG_BUF_ALIGN		PAGE_SIZE

#define TIPC_CTRL_ADDR			53
#define TIPC_ANY_ADDR			0xFFFFFFFF

#define TIPC_MIN_LOCAL_ADDR		1024

#ifdef CONFIG_COMPAT
#define TIPC_IOC32_CONNECT	_IOW(TIPC_IOC_MAGIC, 0x80, compat_uptr_t)
#endif

struct tipc_virtio_dev;

struct tipc_dev_config {
	u32 msg_buf_max_size;
	u32 msg_buf_alignment;
	char dev_name[MAX_DEV_NAME_LEN];
} __packed;

struct tipc_shm {
	trusty_shared_mem_id_t obj_id;
	u64 size;
	u64 tag;
};

struct tipc_msg_hdr {
	u32 src;
	u32 dst;
	u16 reserved;
	u16 shm_cnt;
	u16 len;
	u16 flags;
	u8 data[];
} __packed;

enum tipc_ctrl_msg_types {
	TIPC_CTRL_MSGTYPE_GO_ONLINE = 1,
	TIPC_CTRL_MSGTYPE_GO_OFFLINE,
	TIPC_CTRL_MSGTYPE_CONN_REQ,
	TIPC_CTRL_MSGTYPE_CONN_RSP,
	TIPC_CTRL_MSGTYPE_DISC_REQ,
	TIPC_CTRL_MSGTYPE_RELEASE,
};

struct tipc_ctrl_msg {
	u32 type;
	u32 body_len;
	u8  body[];
} __packed;

struct tipc_conn_req_body {
	char name[MAX_SRV_NAME_LEN];
} __packed;

struct tipc_conn_rsp_body {
	u32 target;
	u32 status;
	u32 remote;
	u32 max_msg_size;
	u32 max_msg_cnt;
} __packed;

struct tipc_disc_req_body {
	u32 target;
} __packed;

struct tipc_release_body {
	trusty_shared_mem_id_t id;
} __packed;

struct tipc_cdev_node {
	struct cdev cdev;
	struct device *dev;
	unsigned int minor;
};

enum tipc_device_state {
	VDS_OFFLINE = 0,
	VDS_ONLINE,
	VDS_DEAD,
};

struct tipc_virtio_dev {
	struct kref refcount;
	struct mutex lock; /* protects access to this device */
	struct virtio_device *vdev;
	struct virtqueue *rxvq;
	struct virtqueue *txvq;
	unsigned int msg_buf_cnt;
	unsigned int msg_buf_max_cnt;
	size_t msg_buf_max_sz;
	unsigned int free_msg_buf_cnt;
	struct list_head free_buf_list;
	wait_queue_head_t sendq;
	struct idr addr_idr;
	enum tipc_device_state state;
	struct tipc_cdev_node cdev_node;
	/* protects shared_handles, dev lock never acquired while held */
	struct mutex shared_handles_lock;
	struct rb_root shared_handles;
	char   cdev_name[MAX_DEV_NAME_LEN];
};

enum tipc_chan_state {
	TIPC_DISCONNECTED = 0,
	TIPC_CONNECTING,
	TIPC_CONNECTED,
	TIPC_STALE,
};

struct tipc_chan {
	struct mutex lock; /* protects channel state  */
	struct kref refcount;
	enum tipc_chan_state state;
	struct tipc_virtio_dev *vds;
	const struct tipc_chan_ops *ops;
	void *ops_arg;
	u32 remote;
	u32 local;
	u32 max_msg_size;
	u32 max_msg_cnt;
	char srv_name[MAX_SRV_NAME_LEN];
};

struct tipc_shared_handle {
	struct rb_node node;
	struct tipc_shm tipc;
	struct tipc_virtio_dev *vds;
	struct sg_table *sgt;
	struct dma_buf_attachment *attach;
	struct dma_buf *dma_buf;
	bool shared;
};

static struct class *tipc_class;
static unsigned int tipc_major;

static struct virtio_device *default_vdev;

static DEFINE_IDR(tipc_devices);
static DEFINE_MUTEX(tipc_devices_lock);

static int _match_any(int id, void *p, void *data)
{
	return id;
}

static int _match_data(int id, void *p, void *data)
{
	return (p == data);
}

static void *_alloc_shareable_mem(size_t sz, gfp_t gfp)
{
	return alloc_pages_exact(sz, gfp);
}

static void _free_shareable_mem(size_t sz, void *va)
{
	free_pages_exact(va, sz);
}

static struct tipc_msg_buf *vds_alloc_msg_buf(struct tipc_virtio_dev *vds,
					      bool share_write)
{
	int ret;
	struct tipc_msg_buf *mb;
	size_t sz = vds->msg_buf_max_sz;
	pgprot_t pgprot = share_write ? PAGE_KERNEL : PAGE_KERNEL_RO;

	/* allocate tracking structure */
	mb = kzalloc(sizeof(struct tipc_msg_buf), GFP_KERNEL);
	if (!mb)
		return NULL;

	/* allocate buffer that can be shared with secure world */
	mb->buf_va = _alloc_shareable_mem(sz, GFP_KERNEL);
	if (!mb->buf_va)
		goto err_alloc;

	sg_init_one(&mb->sg, mb->buf_va, sz);
	ret = trusty_share_memory_compat(vds->vdev->dev.parent->parent,
					 &mb->buf_id, &mb->sg, 1, pgprot);
	if (ret) {
		dev_err(&vds->vdev->dev, "trusty_share_memory failed: %d\n",
			ret);
		goto err_share;
	}

	mb->buf_sz = sz;
	mb->shm_cnt = 0;

	return mb;

err_share:
	_free_shareable_mem(sz, mb->buf_va);
err_alloc:
	kfree(mb);
	return NULL;
}

static void vds_free_msg_buf(struct tipc_virtio_dev *vds,
			     struct tipc_msg_buf *mb)
{
	int ret;

	ret = trusty_reclaim_memory(vds->vdev->dev.parent->parent, mb->buf_id,
				    &mb->sg, 1);
	if (WARN_ON(ret)) {
		dev_err(&vds->vdev->dev,
			"trusty_revoke_memory failed: %d txbuf %lld\n",
			ret, mb->buf_id);

		/*
		 * It is not safe to free this memory if trusty_revoke_memory
		 * fails. Leak it in that case.
		 */
	} else {
		_free_shareable_mem(mb->buf_sz, mb->buf_va);
	}
	kfree(mb);
}

static void vds_free_msg_buf_list(struct tipc_virtio_dev *vds,
				  struct list_head *list)
{
	struct tipc_msg_buf *mb = NULL;

	mb = list_first_entry_or_null(list, struct tipc_msg_buf, node);
	while (mb) {
		list_del(&mb->node);
		vds_free_msg_buf(vds, mb);
		mb = list_first_entry_or_null(list, struct tipc_msg_buf, node);
	}
}

static inline void mb_reset(struct tipc_msg_buf *mb)
{
	mb->wpos = 0;
	mb->rpos = 0;
}

static inline void mb_reset_read(struct tipc_msg_buf *mb)
{
	mb->rpos = 0;
}

static void _free_vds(struct kref *kref)
{
	struct tipc_virtio_dev *vds =
		container_of(kref, struct tipc_virtio_dev, refcount);
	/* If this WARN triggers, we're leaking remote memory references. */
	WARN_ON(!RB_EMPTY_ROOT(&vds->shared_handles));
	kfree(vds);
}

static void _free_chan(struct kref *kref)
{
	struct tipc_chan *ch = container_of(kref, struct tipc_chan, refcount);

	if (ch->ops && ch->ops->handle_release)
		ch->ops->handle_release(ch->ops_arg);

	kref_put(&ch->vds->refcount, _free_vds);
	kfree(ch);
}

static bool _put_txbuf_locked(struct tipc_virtio_dev *vds,
			      struct tipc_msg_buf *mb)
{
	list_add_tail(&mb->node, &vds->free_buf_list);
	return vds->free_msg_buf_cnt++ == 0;
}

static struct tipc_msg_buf *_get_txbuf_locked(struct tipc_virtio_dev *vds)
{
	struct tipc_msg_buf *mb;

	if (vds->state != VDS_ONLINE)
		return  ERR_PTR(-ENODEV);

	if (vds->free_msg_buf_cnt) {
		/* take it out of free list */
		mb = list_first_entry(&vds->free_buf_list,
				      struct tipc_msg_buf, node);
		list_del(&mb->node);
		mb->shm_cnt = 0;
		vds->free_msg_buf_cnt--;
	} else {
		if (vds->msg_buf_cnt >= vds->msg_buf_max_cnt)
			return ERR_PTR(-EAGAIN);

		/* try to allocate it */
		mb = vds_alloc_msg_buf(vds, false);
		if (!mb)
			return ERR_PTR(-ENOMEM);

		vds->msg_buf_cnt++;
	}
	return mb;
}

static struct tipc_msg_buf *_vds_get_txbuf(struct tipc_virtio_dev *vds)
{
	struct tipc_msg_buf *mb;

	mutex_lock(&vds->lock);
	mb = _get_txbuf_locked(vds);
	mutex_unlock(&vds->lock);

	return mb;
}

static void vds_put_txbuf(struct tipc_virtio_dev *vds, struct tipc_msg_buf *mb)
{
	mutex_lock(&vds->lock);
	_put_txbuf_locked(vds, mb);
	wake_up_interruptible(&vds->sendq);
	mutex_unlock(&vds->lock);
}

static struct tipc_msg_buf *vds_get_txbuf(struct tipc_virtio_dev *vds,
					  long timeout)
{
	struct tipc_msg_buf *mb;

	mb = _vds_get_txbuf(vds);

	if ((PTR_ERR(mb) == -EAGAIN) && timeout) {
		DEFINE_WAIT_FUNC(wait, woken_wake_function);

		timeout = msecs_to_jiffies(timeout);
		add_wait_queue(&vds->sendq, &wait);
		for (;;) {
			timeout = wait_woken(&wait, TASK_INTERRUPTIBLE,
					     timeout);
			if (!timeout) {
				mb = ERR_PTR(-ETIMEDOUT);
				break;
			}

			if (signal_pending(current)) {
				mb = ERR_PTR(-ERESTARTSYS);
				break;
			}

			mb = _vds_get_txbuf(vds);
			if (PTR_ERR(mb) != -EAGAIN)
				break;
		}
		remove_wait_queue(&vds->sendq, &wait);
	}

	if (IS_ERR(mb))
		return mb;

	if (WARN_ON(!mb))
		return ERR_PTR(-EINVAL);

	/* reset and reserve space for message header */
	mb_reset(mb);
	mb_put_data(mb, sizeof(struct tipc_msg_hdr));

	return mb;
}

static int vds_queue_txbuf(struct tipc_virtio_dev *vds,
			   struct tipc_msg_buf *mb)
{
	int err;
	struct scatterlist sg;
	bool need_notify = false;

	mutex_lock(&vds->lock);
	if (vds->state == VDS_ONLINE) {
		sg_init_one(&sg, mb, mb->wpos);
		err = virtqueue_add_outbuf(vds->txvq, &sg, 1, mb, GFP_KERNEL);
		need_notify = virtqueue_kick_prepare(vds->txvq);
	} else {
		err = -ENODEV;
	}
	mutex_unlock(&vds->lock);

	if (need_notify)
		virtqueue_notify(vds->txvq);

	return err;
}

static int vds_add_channel(struct tipc_virtio_dev *vds,
			   struct tipc_chan *chan)
{
	int ret;

	mutex_lock(&vds->lock);
	if (vds->state == VDS_ONLINE) {
		ret = idr_alloc(&vds->addr_idr, chan,
				TIPC_MIN_LOCAL_ADDR, TIPC_ANY_ADDR - 1,
				GFP_KERNEL);
		if (ret > 0) {
			chan->local = ret;
			kref_get(&chan->refcount);
			ret = 0;
		}
	} else {
		ret = -EINVAL;
	}
	mutex_unlock(&vds->lock);

	return ret;
}

static void vds_del_channel(struct tipc_virtio_dev *vds,
			    struct tipc_chan *chan)
{
	mutex_lock(&vds->lock);
	if (chan->local) {
		idr_remove(&vds->addr_idr, chan->local);
		chan->local = 0;
		chan->remote = 0;
		kref_put(&chan->refcount, _free_chan);
	}
	mutex_unlock(&vds->lock);
}

static struct tipc_chan *vds_lookup_channel(struct tipc_virtio_dev *vds,
					    u32 addr)
{
	int id;
	struct tipc_chan *chan = NULL;

	mutex_lock(&vds->lock);
	if (addr == TIPC_ANY_ADDR) {
		id = idr_for_each(&vds->addr_idr, _match_any, NULL);
		if (id > 0)
			chan = idr_find(&vds->addr_idr, id);
	} else {
		chan = idr_find(&vds->addr_idr, addr);
	}
	if (chan)
		kref_get(&chan->refcount);
	mutex_unlock(&vds->lock);

	return chan;
}

static struct tipc_chan *vds_create_channel(struct tipc_virtio_dev *vds,
					    const struct tipc_chan_ops *ops,
					    void *ops_arg)
{
	int ret;
	struct tipc_chan *chan = NULL;

	if (!vds)
		return ERR_PTR(-ENOENT);

	if (!ops)
		return ERR_PTR(-EINVAL);

	chan = kzalloc(sizeof(*chan), GFP_KERNEL);
	if (!chan)
		return ERR_PTR(-ENOMEM);

	kref_get(&vds->refcount);
	chan->vds = vds;
	chan->ops = ops;
	chan->ops_arg = ops_arg;
	mutex_init(&chan->lock);
	kref_init(&chan->refcount);
	chan->state = TIPC_DISCONNECTED;

	ret = vds_add_channel(vds, chan);
	if (ret) {
		kfree(chan);
		kref_put(&vds->refcount, _free_vds);
		return ERR_PTR(ret);
	}

	return chan;
}

static void fill_msg_hdr(struct tipc_msg_buf *mb, u32 src, u32 dst)
{
	struct tipc_msg_hdr *hdr = mb_get_data(mb, sizeof(*hdr));

	hdr->src = src;
	hdr->dst = dst;
	hdr->len = mb_avail_data(mb);
	hdr->flags = 0;
	hdr->shm_cnt = mb->shm_cnt;
	hdr->reserved = 0;
}

static int tipc_shared_handle_new(struct tipc_shared_handle **shared_handle,
				  struct tipc_virtio_dev *vds)
{
	struct tipc_shared_handle *out = kzalloc(sizeof(*out), GFP_KERNEL);

	if (!out)
		return -ENOMEM;

	out->vds = vds;
	*shared_handle = out;

	return 0;
}

static struct device *tipc_shared_handle_dev(struct tipc_shared_handle
					     *shared_handle)
{
	return shared_handle->vds->vdev->dev.parent->parent;
}

static void tipc_shared_handle_register(struct tipc_shared_handle
					*new_handle)
{
	struct tipc_virtio_dev *vds = new_handle->vds;
	struct rb_node **new = &vds->shared_handles.rb_node;
	struct rb_node *parent = NULL;

	mutex_lock(&vds->shared_handles_lock);

	while (*new) {
		struct tipc_shared_handle *handle =
			rb_entry(*new, struct tipc_shared_handle, node);
		parent = *new;
		/* The handle is already registered? */
		if (WARN_ON(handle->tipc.obj_id == new_handle->tipc.obj_id))
			goto already_registered;
		if (handle->tipc.obj_id > new_handle->tipc.obj_id)
			new = &((*new)->rb_left);
		else
			new = &((*new)->rb_right);
	}

	rb_link_node(&new_handle->node, parent, new);
	rb_insert_color(&new_handle->node, &vds->shared_handles);

already_registered:
	mutex_unlock(&vds->shared_handles_lock);
}

static struct tipc_shared_handle *tipc_shared_handle_take(struct tipc_virtio_dev
							  *vds,
							  trusty_shared_mem_id_t
							  obj_id)
{
	struct rb_node *node = vds->shared_handles.rb_node;
	struct tipc_shared_handle *out = NULL;

	mutex_lock(&vds->shared_handles_lock);

	while (node) {
		struct tipc_shared_handle *handle =
			rb_entry(node, struct tipc_shared_handle, node);
		if (obj_id == handle->tipc.obj_id) {
			rb_erase(node, &vds->shared_handles);
			out = handle;
			break;
		} else if (obj_id < handle->tipc.obj_id) {
			node = node->rb_left;
		} else {
			node = node->rb_right;
		}
	}

	mutex_unlock(&vds->shared_handles_lock);

	return out;
}

static int tipc_shared_handle_drop(struct tipc_shared_handle *shared_handle)
{
	int ret;
	struct tipc_virtio_dev *vds = shared_handle->vds;
	struct device *dev = tipc_shared_handle_dev(shared_handle);

	/*
	 * If this warning fires, it means this shared handle was still in
	 * the set of active handles. This shouldn't happen (calling code
	 * should ensure it is out if the tree) but this serves as an extra
	 * check before it is released.
	 *
	 * However, the take itself should clean this incorrect state up by
	 * removing the handle from the tree.
	 */
	WARN_ON(tipc_shared_handle_take(vds, shared_handle->tipc.obj_id));

	if (shared_handle->shared) {
		ret = trusty_reclaim_memory(dev,
					    shared_handle->tipc.obj_id,
					    shared_handle->sgt->sgl,
					    shared_handle->sgt->orig_nents);
		if (ret) {
			/*
			 * We can't safely release this, it may still be in
			 * use outside Linux.
			 */
			dev_warn(dev, "Failed to drop handle, leaking...\n");
			return ret;
		}
	}

	if (shared_handle->sgt)
		dma_buf_unmap_attachment(shared_handle->attach,
					 shared_handle->sgt, DMA_BIDIRECTIONAL);
	if (shared_handle->attach)
		dma_buf_detach(shared_handle->dma_buf, shared_handle->attach);
	if (shared_handle->dma_buf)
		dma_buf_put(shared_handle->dma_buf);

	kfree(shared_handle);

	return 0;
}

/*****************************************************************************/

struct tipc_chan *tipc_create_channel(struct device *dev,
				      const struct tipc_chan_ops *ops,
				      void *ops_arg)
{
	struct virtio_device *vd;
	struct tipc_chan *chan;
	struct tipc_virtio_dev *vds;

	mutex_lock(&tipc_devices_lock);
	if (dev) {
		vd = container_of(dev, struct virtio_device, dev);
	} else {
		vd = default_vdev;
		if (!vd) {
			mutex_unlock(&tipc_devices_lock);
			return ERR_PTR(-ENOENT);
		}
	}
	vds = vd->priv;
	kref_get(&vds->refcount);
	mutex_unlock(&tipc_devices_lock);

	chan = vds_create_channel(vds, ops, ops_arg);
	kref_put(&vds->refcount, _free_vds);
	return chan;
}
EXPORT_SYMBOL(tipc_create_channel);

struct tipc_msg_buf *tipc_chan_get_rxbuf(struct tipc_chan *chan)
{
	return vds_alloc_msg_buf(chan->vds, true);
}
EXPORT_SYMBOL(tipc_chan_get_rxbuf);

void tipc_chan_put_rxbuf(struct tipc_chan *chan, struct tipc_msg_buf *mb)
{
	vds_free_msg_buf(chan->vds, mb);
}
EXPORT_SYMBOL(tipc_chan_put_rxbuf);

struct tipc_msg_buf *tipc_chan_get_txbuf_timeout(struct tipc_chan *chan,
						 long timeout)
{
	return vds_get_txbuf(chan->vds, timeout);
}
EXPORT_SYMBOL(tipc_chan_get_txbuf_timeout);

void tipc_chan_put_txbuf(struct tipc_chan *chan, struct tipc_msg_buf *mb)
{
	vds_put_txbuf(chan->vds, mb);
}
EXPORT_SYMBOL(tipc_chan_put_txbuf);

int tipc_chan_queue_msg(struct tipc_chan *chan, struct tipc_msg_buf *mb)
{
	int err;

	mutex_lock(&chan->lock);
	switch (chan->state) {
	case TIPC_CONNECTED:
		fill_msg_hdr(mb, chan->local, chan->remote);
		err = vds_queue_txbuf(chan->vds, mb);
		if (err) {
			/* this should never happen */
			dev_err(&chan->vds->vdev->dev,
				"%s: failed to queue tx buffer (%d)\n",
			       __func__, err);
		}
		break;
	case TIPC_DISCONNECTED:
	case TIPC_CONNECTING:
		err = -ENOTCONN;
		break;
	case TIPC_STALE:
		err = -ESHUTDOWN;
		break;
	default:
		err = -EBADFD;
		dev_err(&chan->vds->vdev->dev,
			"%s: unexpected channel state %d\n",
			__func__, chan->state);
	}
	mutex_unlock(&chan->lock);
	return err;
}
EXPORT_SYMBOL(tipc_chan_queue_msg);


int tipc_chan_connect(struct tipc_chan *chan, const char *name)
{
	int err;
	struct tipc_ctrl_msg *msg;
	struct tipc_conn_req_body *body;
	struct tipc_msg_buf *txbuf;

	txbuf = vds_get_txbuf(chan->vds, TXBUF_TIMEOUT);
	if (IS_ERR(txbuf))
		return PTR_ERR(txbuf);

	/* reserve space for connection request control message */
	msg = mb_put_data(txbuf, sizeof(*msg) + sizeof(*body));
	body = (struct tipc_conn_req_body *)msg->body;

	/* fill message */
	msg->type = TIPC_CTRL_MSGTYPE_CONN_REQ;
	msg->body_len  = sizeof(*body);

	strncpy(body->name, name, sizeof(body->name));
	body->name[sizeof(body->name)-1] = '\0';

	mutex_lock(&chan->lock);
	switch (chan->state) {
	case TIPC_DISCONNECTED:
		/* save service name we are connecting to */
		strcpy(chan->srv_name, body->name);

		fill_msg_hdr(txbuf, chan->local, TIPC_CTRL_ADDR);
		err = vds_queue_txbuf(chan->vds, txbuf);
		if (err) {
			/* this should never happen */
			dev_err(&chan->vds->vdev->dev,
				"%s: failed to queue tx buffer (%d)\n",
				__func__, err);
		} else {
			chan->state = TIPC_CONNECTING;
			txbuf = NULL; /* prevents discarding buffer */
		}
		break;
	case TIPC_CONNECTED:
	case TIPC_CONNECTING:
		/* check if we are trying to connect to the same service */
		if (strcmp(chan->srv_name, body->name) == 0)
			err = 0;
		else
			if (chan->state == TIPC_CONNECTING)
				err = -EALREADY; /* in progress */
			else
				err = -EISCONN;  /* already connected */
		break;

	case TIPC_STALE:
		err = -ESHUTDOWN;
		break;
	default:
		err = -EBADFD;
		dev_err(&chan->vds->vdev->dev,
			"%s: unexpected channel state %d\n",
			__func__, chan->state);
		break;
	}
	mutex_unlock(&chan->lock);

	if (txbuf)
		tipc_chan_put_txbuf(chan, txbuf); /* discard it */

	return err;
}
EXPORT_SYMBOL(tipc_chan_connect);

int tipc_chan_shutdown(struct tipc_chan *chan)
{
	int err;
	struct tipc_ctrl_msg *msg;
	struct tipc_disc_req_body *body;
	struct tipc_msg_buf *txbuf = NULL;

	/* get tx buffer */
	txbuf = vds_get_txbuf(chan->vds, TXBUF_TIMEOUT);
	if (IS_ERR(txbuf))
		return PTR_ERR(txbuf);

	mutex_lock(&chan->lock);
	if (chan->state == TIPC_CONNECTED || chan->state == TIPC_CONNECTING) {
		/* reserve space for disconnect request control message */
		msg = mb_put_data(txbuf, sizeof(*msg) + sizeof(*body));
		body = (struct tipc_disc_req_body *)msg->body;

		msg->type = TIPC_CTRL_MSGTYPE_DISC_REQ;
		msg->body_len = sizeof(*body);
		body->target = chan->remote;

		fill_msg_hdr(txbuf, chan->local, TIPC_CTRL_ADDR);
		err = vds_queue_txbuf(chan->vds, txbuf);
		if (err) {
			/* this should never happen */
			dev_err(&chan->vds->vdev->dev,
				"%s: failed to queue tx buffer (%d)\n",
				__func__, err);
		}
	} else {
		err = -ENOTCONN;
	}
	chan->state = TIPC_STALE;
	mutex_unlock(&chan->lock);

	if (err) {
		/* release buffer */
		tipc_chan_put_txbuf(chan, txbuf);
	}

	return err;
}
EXPORT_SYMBOL(tipc_chan_shutdown);

void tipc_chan_destroy(struct tipc_chan *chan)
{
	vds_del_channel(chan->vds, chan);
	kref_put(&chan->refcount, _free_chan);
}
EXPORT_SYMBOL(tipc_chan_destroy);

/***************************************************************************/

struct tipc_dn_chan {
	int state;
	struct mutex lock; /* protects rx_msg_queue list and channel state */
	struct tipc_chan *chan;
	wait_queue_head_t readq;
	struct completion reply_comp;
	struct list_head rx_msg_queue;
};

static int dn_wait_for_reply(struct tipc_dn_chan *dn, int timeout)
{
	int ret;

	ret = wait_for_completion_interruptible_timeout(&dn->reply_comp,
					msecs_to_jiffies(timeout));
	if (ret < 0)
		return ret;

	mutex_lock(&dn->lock);
	if (!ret) {
		/* no reply from remote */
		dn->state = TIPC_STALE;
		ret = -ETIMEDOUT;
	} else {
		/* got reply */
		if (dn->state == TIPC_CONNECTED)
			ret = 0;
		else if (dn->state == TIPC_DISCONNECTED)
			if (!list_empty(&dn->rx_msg_queue))
				ret = 0;
			else
				ret = -ENOTCONN;
		else
			ret = -EIO;
	}
	mutex_unlock(&dn->lock);

	return ret;
}

static struct tipc_msg_buf *dn_handle_msg(void *data,
					  struct tipc_msg_buf *rxbuf)
{
	struct tipc_dn_chan *dn = data;
	struct tipc_msg_buf *newbuf = rxbuf;

	mutex_lock(&dn->lock);
	if (dn->state == TIPC_CONNECTED) {
		/* get new buffer */
		newbuf = tipc_chan_get_rxbuf(dn->chan);
		if (newbuf) {
			/* queue an old buffer and return a new one */
			list_add_tail(&rxbuf->node, &dn->rx_msg_queue);
			wake_up_interruptible(&dn->readq);
		} else {
			/*
			 * return an old buffer effectively discarding
			 * incoming message
			 */
			dev_err(&dn->chan->vds->vdev->dev,
				"%s: discard incoming message\n", __func__);
			newbuf = rxbuf;
		}
	}
	mutex_unlock(&dn->lock);

	return newbuf;
}

static void dn_connected(struct tipc_dn_chan *dn)
{
	mutex_lock(&dn->lock);
	dn->state = TIPC_CONNECTED;

	/* complete all pending  */
	complete(&dn->reply_comp);

	mutex_unlock(&dn->lock);
}

static void dn_disconnected(struct tipc_dn_chan *dn)
{
	mutex_lock(&dn->lock);
	dn->state = TIPC_DISCONNECTED;

	/* complete all pending  */
	complete(&dn->reply_comp);

	/* wakeup all readers */
	wake_up_interruptible_all(&dn->readq);

	mutex_unlock(&dn->lock);
}

static void dn_shutdown(struct tipc_dn_chan *dn)
{
	mutex_lock(&dn->lock);

	/* set state to STALE */
	dn->state = TIPC_STALE;

	/* complete all pending  */
	complete(&dn->reply_comp);

	/* wakeup all readers */
	wake_up_interruptible_all(&dn->readq);

	mutex_unlock(&dn->lock);
}

static void dn_handle_event(void *data, int event)
{
	struct tipc_dn_chan *dn = data;

	switch (event) {
	case TIPC_CHANNEL_SHUTDOWN:
		dn_shutdown(dn);
		break;

	case TIPC_CHANNEL_DISCONNECTED:
		dn_disconnected(dn);
		break;

	case TIPC_CHANNEL_CONNECTED:
		dn_connected(dn);
		break;

	default:
		dev_err(&dn->chan->vds->vdev->dev,
			"%s: unhandled event %d\n", __func__, event);
		break;
	}
}

static void dn_handle_release(void *data)
{
	kfree(data);
}

static const struct tipc_chan_ops _dn_ops = {
	.handle_msg = dn_handle_msg,
	.handle_event = dn_handle_event,
	.handle_release = dn_handle_release,
};

#define cdev_to_cdn(c) container_of((c), struct tipc_cdev_node, cdev)
#define cdn_to_vds(cdn) container_of((cdn), struct tipc_virtio_dev, cdev_node)

static struct tipc_virtio_dev *_dn_lookup_vds(struct tipc_cdev_node *cdn)
{
	int ret;
	struct tipc_virtio_dev *vds = NULL;

	mutex_lock(&tipc_devices_lock);
	ret = idr_for_each(&tipc_devices, _match_data, cdn);
	if (ret) {
		vds = cdn_to_vds(cdn);
		kref_get(&vds->refcount);
	}
	mutex_unlock(&tipc_devices_lock);
	return vds;
}

static int tipc_open(struct inode *inode, struct file *filp)
{
	int ret;
	struct tipc_virtio_dev *vds;
	struct tipc_dn_chan *dn;
	struct tipc_cdev_node *cdn = cdev_to_cdn(inode->i_cdev);

	vds = _dn_lookup_vds(cdn);
	if (!vds) {
		ret = -ENOENT;
		goto err_vds_lookup;
	}

	dn = kzalloc(sizeof(*dn), GFP_KERNEL);
	if (!dn) {
		ret = -ENOMEM;
		goto err_alloc_chan;
	}

	mutex_init(&dn->lock);
	init_waitqueue_head(&dn->readq);
	init_completion(&dn->reply_comp);
	INIT_LIST_HEAD(&dn->rx_msg_queue);

	dn->state = TIPC_DISCONNECTED;

	dn->chan = vds_create_channel(vds, &_dn_ops, dn);
	if (IS_ERR(dn->chan)) {
		ret = PTR_ERR(dn->chan);
		goto err_create_chan;
	}

	filp->private_data = dn;
	kref_put(&vds->refcount, _free_vds);
	return 0;

err_create_chan:
	kfree(dn);
err_alloc_chan:
	kref_put(&vds->refcount, _free_vds);
err_vds_lookup:
	return ret;
}


static int dn_connect_ioctl(struct tipc_dn_chan *dn, char __user *usr_name)
{
	int ret;
	char name[MAX_SRV_NAME_LEN];

	/* copy in service name from user space */
	ret = strncpy_from_user(name, usr_name, sizeof(name));
	if (ret < 0)
		return ret;
	if (ret == sizeof(name))
		return -ENAMETOOLONG;

	/* send connect request */
	ret = tipc_chan_connect(dn->chan, name);
	if (ret)
		return ret;

	/* and wait for reply */
	return dn_wait_for_reply(dn, REPLY_TIMEOUT);
}

static int dn_share_fd(struct tipc_dn_chan *dn, int fd,
		       bool lend,
		       struct tipc_shared_handle **out)
{
	int ret = 0;
	struct tipc_shared_handle *shared_handle = NULL;
	struct file *file = NULL;
	struct device *dev = &dn->chan->vds->vdev->dev;
	bool writable = false;
	pgprot_t prot;
	u64 tag = 0;

	if (dn->state != TIPC_CONNECTED) {
		dev_dbg(dev, "Tried to share fd while not connected\n");
		return -ENOTCONN;
	}

	file = fget(fd);
	if (!file) {
		dev_dbg(dev, "Invalid fd (%d)\n", fd);
		return -EBADF;
	}

	if (!(file->f_mode & FMODE_READ)) {
		dev_dbg(dev, "Cannot create write-only mapping\n");
		fput(file);
		return -EACCES;
	}

	writable = file->f_mode & FMODE_WRITE;
	prot = writable ? PAGE_KERNEL : PAGE_KERNEL_RO;
	fput(file);
	file = NULL;

	ret = tipc_shared_handle_new(&shared_handle, dn->chan->vds);
	if (ret)
		return ret;

	shared_handle->dma_buf = dma_buf_get(fd);
	if (IS_ERR(shared_handle->dma_buf)) {
		ret = PTR_ERR(shared_handle->dma_buf);
		shared_handle->dma_buf = NULL;
		dev_dbg(dev, "Unable to get dma buf from fd (%d)\n", ret);
		goto cleanup_handle;
	}

	shared_handle->attach = dma_buf_attach(shared_handle->dma_buf, dev);
	if (IS_ERR(shared_handle->attach)) {
		ret = PTR_ERR(shared_handle->attach);
		shared_handle->attach = NULL;
		dev_dbg(dev, "Unable to attach to dma_buf (%d)\n", ret);
		goto cleanup_handle;
	}

	shared_handle->sgt = dma_buf_map_attachment(shared_handle->attach,
						    DMA_BIDIRECTIONAL);
	if (IS_ERR(shared_handle->sgt)) {
		ret = PTR_ERR(shared_handle->sgt);
		shared_handle->sgt = NULL;
		dev_dbg(dev, "Failed to match attachment (%d)\n", ret);
		goto cleanup_handle;
	}

	tag = trusty_dma_buf_get_ffa_tag(shared_handle->dma_buf);

	ret = trusty_transfer_memory(tipc_shared_handle_dev(shared_handle),
				     &shared_handle->tipc.obj_id,
				     shared_handle->sgt->sgl,
				     shared_handle->sgt->orig_nents, prot,
				     tag, lend);

	if (ret < 0) {
		dev_dbg(dev, "Transferring memory failed: %d\n", ret);
		/*
		 * The handle now has a sgt containing the pages, so we no
		 * longer need to clean up the pages directly.
		 */
		goto cleanup_handle;
	}
	shared_handle->shared = true;
	shared_handle->tipc.size = shared_handle->dma_buf->size;
	shared_handle->tipc.tag = tag;
	*out = shared_handle;
	return 0;

cleanup_handle:
	tipc_shared_handle_drop(shared_handle);
	return ret;
}

static ssize_t txbuf_write_iter(struct tipc_msg_buf *txbuf,
				struct iov_iter *iter)
{
	size_t len;
	/* message length */
	len = iov_iter_count(iter);

	/* check available space */
	if (len > mb_avail_space(txbuf))
		return -EMSGSIZE;

	/* copy in message data */
	if (copy_from_iter(mb_put_data(txbuf, len), len, iter) != len)
		return -EFAULT;

	return len;
}

static ssize_t txbuf_write_handles(struct tipc_msg_buf *txbuf,
				   struct tipc_shared_handle **shm_handles,
				   size_t shm_cnt)
{
	size_t idx;

	/* message length */
	size_t len = shm_cnt * sizeof(struct tipc_shm);

	/* check available space */
	if (len > mb_avail_space(txbuf))
		return -EMSGSIZE;

	/* copy over handles */
	for (idx = 0; idx < shm_cnt; idx++) {
		memcpy(mb_put_data(txbuf, sizeof(struct tipc_shm)),
		       &shm_handles[idx]->tipc,
		       sizeof(struct tipc_shm));
	}

	txbuf->shm_cnt += shm_cnt;

	return len;
}

static long filp_send_ioctl(struct file *filp,
			    const struct tipc_send_msg_req __user *arg)
{
	struct tipc_send_msg_req req;
	struct iovec fast_iovs[UIO_FASTIOV];
	struct iovec *iov = fast_iovs;
	struct iov_iter iter;
	struct trusty_shm *shm = NULL;
	struct tipc_shared_handle **shm_handles = NULL;
	int shm_idx = 0;
	int release_idx;
	struct tipc_dn_chan *dn = filp->private_data;
	struct tipc_virtio_dev *vds = dn->chan->vds;
	struct device *dev = &vds->vdev->dev;
	long timeout = TXBUF_TIMEOUT;
	struct tipc_msg_buf *txbuf = NULL;
	long ret = 0;
	ssize_t data_len = 0;
	ssize_t shm_len = 0;
	bool lend = false;

	if (copy_from_user(&req, arg, sizeof(req)))
		return -EFAULT;

	if (req.shm_cnt > U16_MAX)
		return -E2BIG;

	shm = kmalloc_array(req.shm_cnt, sizeof(*shm), GFP_KERNEL);
	if (!shm)
		return -ENOMEM;

	shm_handles = kmalloc_array(req.shm_cnt, sizeof(*shm_handles),
				    GFP_KERNEL);
	if (!shm_handles) {
		ret = -ENOMEM;
		goto shm_handles_alloc_failed;
	}

	if (copy_from_user(shm, u64_to_user_ptr(req.shm),
			   req.shm_cnt * sizeof(struct trusty_shm))) {
		ret = -EFAULT;
		goto load_shm_args_failed;
	}

	ret = import_iovec(READ, u64_to_user_ptr(req.iov), req.iov_cnt,
			   ARRAY_SIZE(fast_iovs), &iov, &iter);
	if (ret < 0) {
		dev_dbg(dev, "Failed to import iovec\n");
		goto iov_import_failed;
	}

	for (shm_idx = 0; shm_idx < req.shm_cnt; shm_idx++) {
		switch (shm[shm_idx].transfer) {
		case TRUSTY_SHARE:
			lend = false;
			break;
		case TRUSTY_LEND:
			lend = true;
			break;
		default:
			dev_err(dev, "Unknown transfer type: 0x%x\n",
				shm[shm_idx].transfer);
			goto shm_share_failed;
		}
		ret = dn_share_fd(dn, shm[shm_idx].fd,
				  lend,
				  &shm_handles[shm_idx]);
		if (ret) {
			dev_dbg(dev, "Forwarding memory failed\n"
				);
			goto shm_share_failed;
		}
	}

	if (filp->f_flags & O_NONBLOCK)
		timeout = 0;

	txbuf = tipc_chan_get_txbuf_timeout(dn->chan, timeout);
	if (IS_ERR(txbuf)) {
		dev_dbg(dev, "Failed to get txbuffer\n");
		ret = PTR_ERR(txbuf);
		goto get_txbuf_failed;
	}

	data_len = txbuf_write_iter(txbuf, &iter);
	if (data_len < 0) {
		ret = data_len;
		goto txbuf_write_failed;
	}

	shm_len = txbuf_write_handles(txbuf, shm_handles, req.shm_cnt);
	if (shm_len < 0) {
		ret = shm_len;
		goto txbuf_write_failed;
	}

	/*
	 * These need to be aded to the index before queueing the message.
	 * As soon as the message is sent, we may receive a message back from
	 * Trusty saying it's no longer in use, and the shared_handle needs
	 * to be there when that happens.
	 */
	for (shm_idx = 0; shm_idx < req.shm_cnt; shm_idx++)
		tipc_shared_handle_register(shm_handles[shm_idx]);

	ret = tipc_chan_queue_msg(dn->chan, txbuf);

	if (ret)
		goto queue_failed;

	ret = data_len;

common_cleanup:
	kfree(iov);
iov_import_failed:
load_shm_args_failed:
	kfree(shm_handles);
shm_handles_alloc_failed:
	kfree(shm);
	return ret;

queue_failed:
	for (release_idx = 0; release_idx < req.shm_cnt; release_idx++)
		tipc_shared_handle_take(vds,
					shm_handles[release_idx]->tipc.obj_id);
txbuf_write_failed:
	tipc_chan_put_txbuf(dn->chan, txbuf);
get_txbuf_failed:
shm_share_failed:
	for (shm_idx--; shm_idx >= 0; shm_idx--)
		tipc_shared_handle_drop(shm_handles[shm_idx]);
	goto common_cleanup;
}

static long tipc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct tipc_dn_chan *dn = filp->private_data;

	switch (cmd) {
	case TIPC_IOC_CONNECT:
		return dn_connect_ioctl(dn, (char __user *)arg);
	case TIPC_IOC_SEND_MSG:
		return filp_send_ioctl(filp,
				       (const struct tipc_send_msg_req __user *)
				       arg);
	default:
		dev_dbg(&dn->chan->vds->vdev->dev,
			"Unhandled ioctl cmd: 0x%x\n", cmd);
		return -ENOTTY;
	}
}

#ifdef CONFIG_COMPAT
static long tipc_compat_ioctl(struct file *filp,
			      unsigned int cmd, unsigned long arg)
{
	struct tipc_dn_chan *dn = filp->private_data;

	switch (cmd) {
	case TIPC_IOC32_CONNECT:
		cmd = TIPC_IOC_CONNECT;
		break;
	default:
		dev_dbg(&dn->chan->vds->vdev->dev,
			"Unhandled compat ioctl command: 0x%x\n", cmd);
		return -ENOTTY;
	}
	return tipc_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static inline bool _got_rx(struct tipc_dn_chan *dn)
{
	if (dn->state != TIPC_CONNECTED)
		return true;

	if (!list_empty(&dn->rx_msg_queue))
		return true;

	return false;
}

static ssize_t tipc_read_iter(struct kiocb *iocb, struct iov_iter *iter)
{
	ssize_t ret;
	size_t len;
	struct tipc_msg_buf *mb;
	struct file *filp = iocb->ki_filp;
	struct tipc_dn_chan *dn = filp->private_data;

	mutex_lock(&dn->lock);

	while (list_empty(&dn->rx_msg_queue)) {
		if (dn->state != TIPC_CONNECTED) {
			if (dn->state == TIPC_CONNECTING)
				ret = -ENOTCONN;
			else if (dn->state == TIPC_DISCONNECTED)
				ret = -ENOTCONN;
			else if (dn->state == TIPC_STALE)
				ret = -ESHUTDOWN;
			else
				ret = -EBADFD;
			goto out;
		}

		mutex_unlock(&dn->lock);

		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		if (wait_event_interruptible(dn->readq, _got_rx(dn)))
			return -ERESTARTSYS;

		mutex_lock(&dn->lock);
	}

	mb = list_first_entry(&dn->rx_msg_queue, struct tipc_msg_buf, node);

	len = mb_avail_data(mb);
	if (len > iov_iter_count(iter)) {
		ret = -EMSGSIZE;
		goto out;
	}

	if (copy_to_iter(mb_get_data(mb, len), len, iter) != len) {
		ret = -EFAULT;
		goto out;
	}

	ret = len;
	list_del(&mb->node);
	tipc_chan_put_rxbuf(dn->chan, mb);

out:
	mutex_unlock(&dn->lock);
	return ret;
}

static ssize_t tipc_write_iter(struct kiocb *iocb, struct iov_iter *iter)
{
	struct file *filp = iocb->ki_filp;
	struct tipc_dn_chan *dn = filp->private_data;
	long timeout = TXBUF_TIMEOUT;
	struct tipc_msg_buf *txbuf = NULL;
	ssize_t ret = 0;
	ssize_t len = 0;

	if (filp->f_flags & O_NONBLOCK)
		timeout = 0;

	txbuf = tipc_chan_get_txbuf_timeout(dn->chan, timeout);

	if (IS_ERR(txbuf))
		return PTR_ERR(txbuf);

	len = txbuf_write_iter(txbuf, iter);
	if (len < 0)
		goto err_out;

	/* queue message */
	ret = tipc_chan_queue_msg(dn->chan, txbuf);
	if (ret)
		goto err_out;

	return len;

err_out:
	tipc_chan_put_txbuf(dn->chan, txbuf);
	return ret;
}

static __poll_t tipc_poll(struct file *filp, poll_table *wait)
{
	__poll_t mask = 0;
	struct tipc_dn_chan *dn = filp->private_data;

	mutex_lock(&dn->lock);

	poll_wait(filp, &dn->readq, wait);

	/* Writes always succeed for now */
	mask |= EPOLLOUT | EPOLLWRNORM;

	if (!list_empty(&dn->rx_msg_queue))
		mask |= EPOLLIN | EPOLLRDNORM;

	if (dn->state != TIPC_CONNECTED)
		mask |= EPOLLERR;

	mutex_unlock(&dn->lock);
	return mask;
}


static int tipc_release(struct inode *inode, struct file *filp)
{
	struct tipc_dn_chan *dn = filp->private_data;

	dn_shutdown(dn);

	/* free all pending buffers */
	vds_free_msg_buf_list(dn->chan->vds, &dn->rx_msg_queue);

	/* shutdown channel  */
	tipc_chan_shutdown(dn->chan);

	/* and destroy it */
	tipc_chan_destroy(dn->chan);

	return 0;
}

static const struct file_operations tipc_fops = {
	.open		= tipc_open,
	.release	= tipc_release,
	.unlocked_ioctl	= tipc_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= tipc_compat_ioctl,
#endif
	.read_iter	= tipc_read_iter,
	.write_iter	= tipc_write_iter,
	.poll		= tipc_poll,
	.owner		= THIS_MODULE,
};

/*****************************************************************************/

static void chan_trigger_event(struct tipc_chan *chan, int event)
{
	if (!event)
		return;

	chan->ops->handle_event(chan->ops_arg, event);
}

static void _cleanup_vq(struct tipc_virtio_dev *vds, struct virtqueue *vq)
{
	struct tipc_msg_buf *mb;

	while ((mb = virtqueue_detach_unused_buf(vq)) != NULL)
		vds_free_msg_buf(vds, mb);
}

static int _create_cdev_node(struct device *parent,
			     struct tipc_cdev_node *cdn,
			     const char *name)
{
	int ret;
	dev_t devt;

	if (!name) {
		dev_dbg(parent, "%s: cdev name has to be provided\n",
			__func__);
		return -EINVAL;
	}

	/* allocate minor */
	ret = idr_alloc(&tipc_devices, cdn, 0, MAX_DEVICES, GFP_KERNEL);
	if (ret < 0) {
		dev_dbg(parent, "%s: failed (%d) to get id\n",
			__func__, ret);
		return ret;
	}

	cdn->minor = ret;
	cdev_init(&cdn->cdev, &tipc_fops);
	cdn->cdev.owner = THIS_MODULE;

	/* Add character device */
	devt = MKDEV(tipc_major, cdn->minor);
	ret = cdev_add(&cdn->cdev, devt, 1);
	if (ret) {
		dev_dbg(parent, "%s: cdev_add failed (%d)\n",
			__func__, ret);
		goto err_add_cdev;
	}

	/* Create a device node */
	cdn->dev = device_create(tipc_class, parent,
				 devt, NULL, "trusty-ipc-%s", name);
	if (IS_ERR(cdn->dev)) {
		ret = PTR_ERR(cdn->dev);
		dev_dbg(parent, "%s: device_create failed: %d\n",
			__func__, ret);
		goto err_device_create;
	}

	return 0;

err_device_create:
	cdn->dev = NULL;
	cdev_del(&cdn->cdev);
err_add_cdev:
	idr_remove(&tipc_devices, cdn->minor);
	return ret;
}

static void create_cdev_node(struct tipc_virtio_dev *vds,
			     struct tipc_cdev_node *cdn)
{
	int err;

	mutex_lock(&tipc_devices_lock);

	if (!default_vdev) {
		kref_get(&vds->refcount);
		default_vdev = vds->vdev;
	}

	if (vds->cdev_name[0] && !cdn->dev) {
		kref_get(&vds->refcount);
		err = _create_cdev_node(&vds->vdev->dev, cdn, vds->cdev_name);
		if (err) {
			dev_err(&vds->vdev->dev,
				"failed (%d) to create cdev node\n", err);
			kref_put(&vds->refcount, _free_vds);
		}
	}
	mutex_unlock(&tipc_devices_lock);
}

static void destroy_cdev_node(struct tipc_virtio_dev *vds,
			      struct tipc_cdev_node *cdn)
{
	mutex_lock(&tipc_devices_lock);
	if (cdn->dev) {
		device_destroy(tipc_class, MKDEV(tipc_major, cdn->minor));
		cdev_del(&cdn->cdev);
		idr_remove(&tipc_devices, cdn->minor);
		cdn->dev = NULL;
		kref_put(&vds->refcount, _free_vds);
	}

	if (default_vdev == vds->vdev) {
		default_vdev = NULL;
		kref_put(&vds->refcount, _free_vds);
	}

	mutex_unlock(&tipc_devices_lock);
}

static void _go_online(struct tipc_virtio_dev *vds)
{
	mutex_lock(&vds->lock);
	if (vds->state == VDS_OFFLINE)
		vds->state = VDS_ONLINE;
	mutex_unlock(&vds->lock);

	create_cdev_node(vds, &vds->cdev_node);

	dev_info(&vds->vdev->dev, "is online\n");
}

static void _go_offline(struct tipc_virtio_dev *vds)
{
	struct tipc_chan *chan;

	/* change state to OFFLINE */
	mutex_lock(&vds->lock);
	if (vds->state != VDS_ONLINE) {
		mutex_unlock(&vds->lock);
		return;
	}
	vds->state = VDS_OFFLINE;
	mutex_unlock(&vds->lock);

	/* wakeup all waiters */
	wake_up_interruptible_all(&vds->sendq);

	/* shutdown all channels */
	while ((chan = vds_lookup_channel(vds, TIPC_ANY_ADDR))) {
		mutex_lock(&chan->lock);
		chan->state = TIPC_STALE;
		chan->remote = 0;
		chan_trigger_event(chan, TIPC_CHANNEL_SHUTDOWN);
		mutex_unlock(&chan->lock);
		kref_put(&chan->refcount, _free_chan);
	}

	/* shutdown device node */
	destroy_cdev_node(vds, &vds->cdev_node);

	dev_info(&vds->vdev->dev, "is offline\n");
}

static void _handle_conn_rsp(struct tipc_virtio_dev *vds,
			     struct tipc_conn_rsp_body *rsp, size_t len)
{
	struct tipc_chan *chan;

	if (sizeof(*rsp) != len) {
		dev_err(&vds->vdev->dev, "%s: Invalid response length %zd\n",
			__func__, len);
		return;
	}

	dev_dbg(&vds->vdev->dev,
		"%s: connection response: for addr 0x%x: status %d remote addr 0x%x\n",
		__func__, rsp->target, rsp->status, rsp->remote);

	/* Lookup channel */
	chan = vds_lookup_channel(vds, rsp->target);
	if (chan) {
		mutex_lock(&chan->lock);
		if (chan->state == TIPC_CONNECTING) {
			if (!rsp->status) {
				chan->state = TIPC_CONNECTED;
				chan->remote = rsp->remote;
				chan->max_msg_cnt = rsp->max_msg_cnt;
				chan->max_msg_size = rsp->max_msg_size;
				chan_trigger_event(chan,
						   TIPC_CHANNEL_CONNECTED);
			} else {
				chan->state = TIPC_DISCONNECTED;
				chan->remote = 0;
				chan_trigger_event(chan,
						   TIPC_CHANNEL_DISCONNECTED);
			}
		}
		mutex_unlock(&chan->lock);
		kref_put(&chan->refcount, _free_chan);
	}
}

static void _handle_disc_req(struct tipc_virtio_dev *vds,
			     struct tipc_disc_req_body *req, size_t len)
{
	struct tipc_chan *chan;

	if (sizeof(*req) != len) {
		dev_err(&vds->vdev->dev, "%s: Invalid request length %zd\n",
			__func__, len);
		return;
	}

	dev_dbg(&vds->vdev->dev, "%s: disconnect request: for addr 0x%x\n",
		__func__, req->target);

	chan = vds_lookup_channel(vds, req->target);
	if (chan) {
		mutex_lock(&chan->lock);
		if (chan->state == TIPC_CONNECTED ||
			chan->state == TIPC_CONNECTING) {
			chan->state = TIPC_DISCONNECTED;
			chan->remote = 0;
			chan_trigger_event(chan, TIPC_CHANNEL_DISCONNECTED);
		}
		mutex_unlock(&chan->lock);
		kref_put(&chan->refcount, _free_chan);
	}
}

static void _handle_release(struct tipc_virtio_dev *vds,
			    struct tipc_release_body *req, size_t len)
{
	struct tipc_shared_handle *handle = NULL;
	struct device *dev = &vds->vdev->dev;
	int ret = 0;

	if (len < sizeof(*req)) {
		dev_err(dev, "Received undersized release control message\n");
		return;
	}

	handle = tipc_shared_handle_take(vds, req->id);
	if (!handle) {
		dev_err(dev,
			"Received release control message for untracked handle: 0x%llx\n",
			req->id);
		return;
	}

	ret = tipc_shared_handle_drop(handle);

	if (ret) {
		dev_err(dev,
			"Failed to release handle 0x%llx upon request: (%d)\n",
			req->id, ret);
		/*
		 * Put the handle back in case we got a spurious release now and
		 * get a real one later. This path should not happen, we're
		 * just trying to be robust.
		 */
		tipc_shared_handle_register(handle);
	}
}

static void _handle_ctrl_msg(struct tipc_virtio_dev *vds,
			     void *data, int len, u32 src)
{
	struct tipc_ctrl_msg *msg = data;

	if ((len < sizeof(*msg)) || (sizeof(*msg) + msg->body_len != len)) {
		dev_err(&vds->vdev->dev,
			"%s: Invalid message length ( %d vs. %d)\n",
			__func__, (int)(sizeof(*msg) + msg->body_len), len);
		return;
	}

	dev_dbg(&vds->vdev->dev,
		"%s: Incoming ctrl message: src 0x%x type %d len %d\n",
		__func__, src, msg->type, msg->body_len);

	switch (msg->type) {
	case TIPC_CTRL_MSGTYPE_GO_ONLINE:
		_go_online(vds);
		break;

	case TIPC_CTRL_MSGTYPE_GO_OFFLINE:
		_go_offline(vds);
		break;

	case TIPC_CTRL_MSGTYPE_CONN_RSP:
		_handle_conn_rsp(vds, (struct tipc_conn_rsp_body *)msg->body,
				 msg->body_len);
		break;

	case TIPC_CTRL_MSGTYPE_DISC_REQ:
		_handle_disc_req(vds, (struct tipc_disc_req_body *)msg->body,
				 msg->body_len);
		break;

	case TIPC_CTRL_MSGTYPE_RELEASE:
		_handle_release(vds, (struct tipc_release_body *)msg->body,
				msg->body_len);
	break;

	default:
		dev_warn(&vds->vdev->dev,
			 "%s: Unexpected message type: %d\n",
			 __func__, msg->type);
	}
}

static void handle_dropped_chan_msg(struct tipc_virtio_dev *vds,
				    struct tipc_msg_buf *mb,
				    struct tipc_msg_hdr *msg)
{
	int shm_idx;
	struct tipc_shm *shm;
	struct tipc_shared_handle *shared_handle;
	struct device *dev = &vds->vdev->dev;
	size_t len;

	if (msg->len < msg->shm_cnt * sizeof(*shm)) {
		dev_err(dev, "shm_cnt does not fit in dropped message");
		/* The message is corrupt, so we can't recover resources */
		return;
	}

	len = msg->len - msg->shm_cnt * sizeof(*shm);
	/* skip normal data */
	(void)mb_get_data(mb, len);

	for (shm_idx = 0; shm_idx < msg->shm_cnt; shm_idx++) {
		shm = mb_get_data(mb, sizeof(*shm));
		shared_handle = tipc_shared_handle_take(vds, shm->obj_id);
		if (shared_handle) {
			if (tipc_shared_handle_drop(shared_handle))
				dev_err(dev,
					"Failed to drop handle found in dropped buffer");
		} else {
			dev_err(dev,
				"Found handle in dropped buffer which was not registered to tipc device...");
		}
	}
}

static void handle_dropped_mb(struct tipc_virtio_dev *vds,
			      struct tipc_msg_buf *mb)
{
	struct tipc_msg_hdr *msg;

	mb_reset_read(mb);
	msg = mb_get_data(mb, sizeof(*msg));
	if (msg->dst != TIPC_CTRL_ADDR) {
		handle_dropped_chan_msg(vds, mb, msg);
	}
}

static int _handle_rxbuf(struct tipc_virtio_dev *vds,
			 struct tipc_msg_buf *rxbuf, size_t rxlen)
{
	int err;
	struct scatterlist sg;
	struct tipc_msg_hdr *msg;
	struct device *dev = &vds->vdev->dev;

	/* message sanity check */
	if (rxlen > rxbuf->buf_sz) {
		dev_warn(dev, "inbound msg is too big: %zd\n", rxlen);
		goto drop_it;
	}

	if (rxlen < sizeof(*msg)) {
		dev_warn(dev, "inbound msg is too short: %zd\n", rxlen);
		goto drop_it;
	}

	/* reset buffer and put data  */
	mb_reset(rxbuf);
	mb_put_data(rxbuf, rxlen);

	/* get message header */
	msg = mb_get_data(rxbuf, sizeof(*msg));
	if (mb_avail_data(rxbuf) != msg->len) {
		dev_warn(dev, "inbound msg length mismatch: (%zu vs. %d)\n",
			 mb_avail_data(rxbuf), msg->len);
		goto drop_it;
	}

	dev_dbg(dev, "From: %d, To: %d, Len: %d, Flags: 0x%x, Reserved: %d, shm_cnt: %d\n",
		msg->src, msg->dst, msg->len, msg->flags, msg->reserved,
		msg->shm_cnt);

	/* message directed to control endpoint is a special case */
	if (msg->dst == TIPC_CTRL_ADDR) {
		_handle_ctrl_msg(vds, msg->data, msg->len, msg->src);
	} else {
		struct tipc_chan *chan = NULL;
		/* Lookup channel */
		chan = vds_lookup_channel(vds, msg->dst);
		if (chan) {
			/* handle it */
			rxbuf = chan->ops->handle_msg(chan->ops_arg, rxbuf);
			kref_put(&chan->refcount, _free_chan);
			if (WARN_ON(!rxbuf))
				return -EINVAL;
		}
	}

drop_it:
	/* add the buffer back to the virtqueue */
	sg_init_one(&sg, rxbuf, rxbuf->buf_sz);
	err = virtqueue_add_inbuf(vds->rxvq, &sg, 1, rxbuf, GFP_KERNEL);
	if (err < 0) {
		dev_err(dev, "failed to add a virtqueue buffer: %d\n", err);
		return err;
	}

	return 0;
}

static void _rxvq_cb(struct virtqueue *rxvq)
{
	unsigned int len;
	struct tipc_msg_buf *mb;
	unsigned int msg_cnt = 0;
	struct tipc_virtio_dev *vds = rxvq->vdev->priv;

	while ((mb = virtqueue_get_buf(rxvq, &len)) != NULL) {
		if (_handle_rxbuf(vds, mb, len))
			break;
		msg_cnt++;
	}

	/* tell the other size that we added rx buffers */
	if (msg_cnt)
		virtqueue_kick(rxvq);
}

static void _txvq_cb(struct virtqueue *txvq)
{
	unsigned int len;
	struct tipc_msg_buf *mb;
	bool need_wakeup = false;
	struct tipc_virtio_dev *vds = txvq->vdev->priv;

	/* detach all buffers */
	mutex_lock(&vds->lock);
	while ((mb = virtqueue_get_buf(txvq, &len)) != NULL) {
		if ((int)len < 0)
			handle_dropped_mb(vds, mb);
		need_wakeup |= _put_txbuf_locked(vds, mb);
	}
	mutex_unlock(&vds->lock);

	if (need_wakeup) {
		/* wake up potential senders waiting for a tx buffer */
		wake_up_interruptible_all(&vds->sendq);
	}
}

static int tipc_virtio_probe(struct virtio_device *vdev)
{
	int err, i;
	struct tipc_virtio_dev *vds;
	struct tipc_dev_config config;
	struct virtqueue *vqs[2];
	vq_callback_t *vq_cbs[] = {_rxvq_cb, _txvq_cb};
	static const char * const vq_names[] = { "rx", "tx" };

	vds = kzalloc(sizeof(*vds), GFP_KERNEL);
	if (!vds)
		return -ENOMEM;

	vds->vdev = vdev;

	mutex_init(&vds->lock);
	mutex_init(&vds->shared_handles_lock);
	kref_init(&vds->refcount);
	init_waitqueue_head(&vds->sendq);
	INIT_LIST_HEAD(&vds->free_buf_list);
	idr_init(&vds->addr_idr);
	vds->shared_handles = RB_ROOT;
	dma_coerce_mask_and_coherent(&vds->vdev->dev,
				     *vds->vdev->dev.parent->parent->dma_mask);

	/* set default max message size and alignment */
	memset(&config, 0, sizeof(config));
	config.msg_buf_max_size  = DEFAULT_MSG_BUF_SIZE;
	config.msg_buf_alignment = DEFAULT_MSG_BUF_ALIGN;

	/* get configuration if present */
	vdev->config->get(vdev, 0, &config, sizeof(config));

	/* copy dev name */
	strncpy(vds->cdev_name, config.dev_name, sizeof(vds->cdev_name));
	vds->cdev_name[sizeof(vds->cdev_name)-1] = '\0';

	/* find tx virtqueues (rx and tx and in this order) */
	err = vdev->config->find_vqs(vdev, 2, vqs, vq_cbs, vq_names, NULL,
				     NULL);
	if (err)
		goto err_find_vqs;

	vds->rxvq = vqs[0];
	vds->txvq = vqs[1];

	/* save max buffer size and count */
	vds->msg_buf_max_sz = config.msg_buf_max_size;
	vds->msg_buf_max_cnt = virtqueue_get_vring_size(vds->txvq);

	/* set up the receive buffers */
	for (i = 0; i < virtqueue_get_vring_size(vds->rxvq); i++) {
		struct scatterlist sg;
		struct tipc_msg_buf *rxbuf;

		rxbuf = vds_alloc_msg_buf(vds, true);
		if (!rxbuf) {
			dev_err(&vdev->dev, "failed to allocate rx buffer\n");
			err = -ENOMEM;
			goto err_free_rx_buffers;
		}

		sg_init_one(&sg, rxbuf, rxbuf->buf_sz);
		err = virtqueue_add_inbuf(vds->rxvq, &sg, 1, rxbuf, GFP_KERNEL);
		WARN_ON(err); /* sanity check; this can't really happen */
	}

	vdev->priv = vds;
	vds->state = VDS_OFFLINE;

	dev_dbg(&vdev->dev, "%s: done\n", __func__);
	return 0;

err_free_rx_buffers:
	_cleanup_vq(vds, vds->rxvq);
err_find_vqs:
	kref_put(&vds->refcount, _free_vds);
	return err;
}

static void tipc_virtio_remove(struct virtio_device *vdev)
{
	struct tipc_virtio_dev *vds = vdev->priv;

	_go_offline(vds);

	mutex_lock(&vds->lock);
	vds->state = VDS_DEAD;
	vds->vdev = NULL;
	mutex_unlock(&vds->lock);

	vdev->config->reset(vdev);

	idr_destroy(&vds->addr_idr);

	_cleanup_vq(vds, vds->rxvq);
	_cleanup_vq(vds, vds->txvq);
	vds_free_msg_buf_list(vds, &vds->free_buf_list);

	vdev->config->del_vqs(vds->vdev);

	kref_put(&vds->refcount, _free_vds);
}

static const struct virtio_device_id tipc_virtio_id_table[] = {
	{ VIRTIO_ID_TRUSTY_IPC, VIRTIO_DEV_ANY_ID },
	{ 0 },
};

static const unsigned int features[] = {
	0,
};

static struct virtio_driver virtio_tipc_driver = {
	.feature_table	= features,
	.feature_table_size = ARRAY_SIZE(features),
	.driver.name	= KBUILD_MODNAME,
	.driver.owner	= THIS_MODULE,
	.id_table	= tipc_virtio_id_table,
	.probe		= tipc_virtio_probe,
	.remove		= tipc_virtio_remove,
};

static int __init tipc_init(void)
{
	int ret;
	dev_t dev;

	ret = alloc_chrdev_region(&dev, 0, MAX_DEVICES, KBUILD_MODNAME);
	if (ret) {
		pr_err("%s: alloc_chrdev_region failed: %d\n", __func__, ret);
		return ret;
	}

	tipc_major = MAJOR(dev);
	tipc_class = class_create(THIS_MODULE, KBUILD_MODNAME);
	if (IS_ERR(tipc_class)) {
		ret = PTR_ERR(tipc_class);
		pr_err("%s: class_create failed: %d\n", __func__, ret);
		goto err_class_create;
	}

	ret = register_virtio_driver(&virtio_tipc_driver);
	if (ret) {
		pr_err("failed to register virtio driver: %d\n", ret);
		goto err_register_virtio_drv;
	}

	return 0;

err_register_virtio_drv:
	class_destroy(tipc_class);

err_class_create:
	unregister_chrdev_region(dev, MAX_DEVICES);
	return ret;
}

static void __exit tipc_exit(void)
{
	unregister_virtio_driver(&virtio_tipc_driver);
	class_destroy(tipc_class);
	unregister_chrdev_region(MKDEV(tipc_major, 0), MAX_DEVICES);
}

/* We need to init this early */
subsys_initcall(tipc_init);
module_exit(tipc_exit);

MODULE_DEVICE_TABLE(tipc, tipc_virtio_id_table);
MODULE_DESCRIPTION("Trusty IPC driver");
MODULE_LICENSE("GPL v2");
