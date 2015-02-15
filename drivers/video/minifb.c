#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#include <linux/msm_ion.h>
#include <linux/minifb.h>

struct minifb_data
{
	struct minifb_req buf_info;
	struct list_head entry;
	struct ion_handle *ionhdl;
	int state;
	int info;
};

struct minifb_ctrl
{
	struct mutex lock;
	struct kref refcnt;

	struct list_head busy_queue;
	struct list_head free_queue;
	struct list_head ready_queue;
	struct minifb_data *retired;
	wait_queue_head_t wait_q; /* TBD */
	u32 state;
	uint32_t width;
	uint32_t height;
	uint32_t frame_cnt;
	uint32_t drop_cnt;
	uint32_t retry_cnt;
	uint32_t lock_cnt;
	unsigned long log_time;

	struct ion_client *iclient;
	struct dentry *debug_root;
};

static DEFINE_MUTEX(minifb_lock);
static struct minifb_ctrl *minifb_ctrl;

static struct minifb_ctrl *get_ctrl(void)
{
	struct minifb_ctrl *ctrl = NULL;

	mutex_lock(&minifb_lock);

	if (minifb_ctrl) {
		ctrl = minifb_ctrl;
		kref_get(&ctrl->refcnt);
	}

	mutex_unlock(&minifb_lock);

	return ctrl;
}

static void minifb_release_ctrl(struct kref *ref)
{
	struct minifb_data *node, *tmp;

	pr_info("%s\n", __func__);

	mutex_lock(&minifb_ctrl->lock);

	debugfs_remove_recursive(minifb_ctrl->debug_root);

	list_for_each_entry_safe(node, tmp, &minifb_ctrl->busy_queue, entry) {
		pr_warn("%s: Should not here! node#%d\n", __func__, node->info);
		ion_unmap_kernel(minifb_ctrl->iclient, node->ionhdl);
		list_del(&node->entry);
		ion_free(minifb_ctrl->iclient, node->ionhdl);
		kfree(node);
	}

	list_for_each_entry_safe(node, tmp, &minifb_ctrl->ready_queue, entry) {
		list_del(&node->entry);
		ion_free(minifb_ctrl->iclient, node->ionhdl);
		kfree(node);
	}

	list_for_each_entry_safe(node, tmp, &minifb_ctrl->free_queue, entry) {
		list_del(&node->entry);
		ion_free(minifb_ctrl->iclient, node->ionhdl);
		kfree(node);
	}

	ion_client_destroy(minifb_ctrl->iclient);

	mutex_unlock(&minifb_ctrl->lock);

	kfree(minifb_ctrl);
	minifb_ctrl = NULL;
}

static void put_ctrl(struct minifb_ctrl *ctrl)
{
	mutex_lock(&minifb_lock);

	kref_put(&ctrl->refcnt, minifb_release_ctrl);

	mutex_unlock(&minifb_lock);
}

static ssize_t minifb_frame_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int ret = 0, copy_len = 0;
	static void *start = NULL;
	static unsigned long size = 0;

	if (size && (*ppos >= size)) {
		return 0;
	}

	if (start == NULL) {
		char *ptr;
		if (minifb_lockbuf(&start, &size, MINIFB_NOREPEAT) < 0)
			return 0;
		ptr = start;
	}

	copy_len = size - *ppos;
	if (copy_len > count)
		copy_len = count;
	ret = copy_to_user(buff, start + *ppos, copy_len);
	pr_debug("%s: dump from %p with %lld/%lu bytes, count=%d, copylen=%d, ret = %d\n",
		__func__, start, *ppos, size, count, copy_len, ret);

	*ppos += copy_len;

	if (*ppos >= size) {
		minifb_unlockbuf();
		start = NULL;
	}

	return copy_len;
}

static const struct file_operations minifb_frame_fops = {
	.read = minifb_frame_read,
};

static ssize_t minifb_info_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int tot = 0;

	struct minifb_ctrl *fbctrl = get_ctrl();

	if (!fbctrl) {
		pr_err("%s: minifb was not initialized\n", __func__);
		return -EINVAL;
	}

	tot = snprintf(buff, count, "%dx%d, %d frame, %d drop, %d consume, %d again\n",
		fbctrl->width, fbctrl->height,
		fbctrl->frame_cnt, fbctrl->drop_cnt, fbctrl->lock_cnt, fbctrl->retry_cnt);
	if (*ppos >= tot)
		tot = 0;
	*ppos += tot;

	put_ctrl(fbctrl);
	return tot;
}

static const struct file_operations minifb_info_fops = {
	.read = minifb_info_read,
};

int minifb_init(struct minifb_session *sess)
{
	int ret = 0;

	pr_info("%s\n", __func__);
	mutex_lock(&minifb_lock);

	if (!minifb_ctrl) {
		minifb_ctrl = kzalloc(sizeof(struct minifb_ctrl), GFP_KERNEL);
	} else {
		pr_warn("minifb already initialized\n");
		/* TBD: EBUSY? */
	}

	mutex_init(&minifb_ctrl->lock);
	kref_init(&minifb_ctrl->refcnt);
	INIT_LIST_HEAD(&minifb_ctrl->free_queue);
	INIT_LIST_HEAD(&minifb_ctrl->busy_queue);
	INIT_LIST_HEAD(&minifb_ctrl->ready_queue);
	minifb_ctrl->state = 0;
	minifb_ctrl->width = sess->width;
	minifb_ctrl->height = sess->height;
	minifb_ctrl->frame_cnt = 0;
	minifb_ctrl->drop_cnt = 0;
	minifb_ctrl->retry_cnt = 0;
	minifb_ctrl->lock_cnt = 0;
	minifb_ctrl->log_time = 0;
	init_waitqueue_head(&minifb_ctrl->wait_q);

	/* FIXME: get rid of msm_ion_.. */
	minifb_ctrl->iclient = msm_ion_client_create(-1, "minifb");

	minifb_ctrl->debug_root = debugfs_create_dir("minifb", NULL);
	if (IS_ERR_OR_NULL(minifb_ctrl->debug_root)) {
		pr_err("debugfs_create_dir fail, error %ld\n",
		       PTR_ERR(minifb_ctrl->debug_root));
		return -ENODEV;
	}

	debugfs_create_file("framedump", 0440, minifb_ctrl->debug_root, NULL, &minifb_frame_fops);
	debugfs_create_file("info", 0440, minifb_ctrl->debug_root, NULL, &minifb_info_fops);

	mutex_unlock(&minifb_lock);
	pr_info("%s done\n", __func__ );

	return ret;
}

int minifb_terminate(struct minifb_session *sess)
{
	pr_info("%s\n", __func__);

	mutex_lock(&minifb_lock);

	if (!minifb_ctrl) {
		pr_err("%s: minifb was not initialized\n", __func__);
		return -EINVAL;
	}
	kref_put(&minifb_ctrl->refcnt, minifb_release_ctrl);

	mutex_unlock(&minifb_lock);

	return 0;
}

int minifb_queuebuf(struct minifb_req *data)
{
	static int sCount = 0;
	struct minifb_data *node, *tmp, *tmp2;
	struct minifb_ctrl *fbctrl = get_ctrl();

	if (!fbctrl) {
		pr_err("%s: minifb was not initialized\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s\n", __func__);
	/* empty ready list and put new buf into ready list */
	node = kzalloc(sizeof(struct minifb_data), GFP_KERNEL);
	if (!node) {
		pr_err("%s: kzalloc fail\n", __func__);
		return -ENOMEM;
	}

	memcpy(&node->buf_info, data, sizeof(node->buf_info));
	node->ionhdl = ion_import_dma_buf(fbctrl->iclient, node->buf_info.memory_id);
	if (IS_ERR_OR_NULL(node->ionhdl)) {
		pr_warn("%s: import ion buf fail, fd=%d\n", __func__, node->buf_info.memory_id);
	}
	node->info = sCount++;

	mutex_lock(&fbctrl->lock);
	if (!list_empty(&fbctrl->ready_queue)) {
		list_for_each_entry_safe(tmp, tmp2, &fbctrl->ready_queue, entry) {
			list_move(&tmp->entry, &fbctrl->free_queue);
			fbctrl->drop_cnt++;
			pr_debug("%s: drop frame#%d from ready queue\n", __func__, node->info);
		}
	}

	pr_debug("%s: queue frame#%d\n", __func__, node->info);
	list_add_tail(&node->entry, &fbctrl->ready_queue);
	fbctrl->frame_cnt++;

	if (time_after(jiffies, fbctrl->log_time + 5 * HZ) || !fbctrl->log_time) {
		pr_info("[MiniFB]: screen size=%dx%d, stat: %d frame, %d drop, "
			"%d consume, %d again\n",
			fbctrl->width, fbctrl->height,
			fbctrl->frame_cnt, fbctrl->drop_cnt,
			fbctrl->lock_cnt, fbctrl->retry_cnt);

		fbctrl->log_time = jiffies;
	}

	mutex_unlock(&fbctrl->lock);
	put_ctrl(fbctrl);

	return 0;
}

/* blocking/non-blocking operation */
int minifb_dequeuebuf(struct minifb_req *data)
{
	int ret = 0;
	struct minifb_ctrl *fbctrl = get_ctrl();

	if (!fbctrl) {
		pr_err("%s: minifb was not initialized\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s\n", __func__);
	mutex_lock(&fbctrl->lock);

	/* find a buf from free list and return it. */
	if (!list_empty(&fbctrl->free_queue)) {
		struct minifb_data *node;

		node = list_first_entry(&fbctrl->free_queue, struct minifb_data, entry);
		pr_debug("%s: dequeue frame#%d\n", __func__, node->info);
		list_del(&node->entry);
		memcpy(data, &node->buf_info, sizeof(*data));
		ion_free(fbctrl->iclient, node->ionhdl);
		if (node == fbctrl->retired)
			fbctrl->retired = NULL;
		kfree(node);
	} else {
		ret = -EBUSY;
	}

	mutex_unlock(&fbctrl->lock);
	put_ctrl(fbctrl);

	return ret;
}

int minifb_lockbuf(void **vaddr, unsigned long *ptr_size, int repeat)
{
	int ret = 0;
	struct minifb_data *node;
	struct minifb_ctrl *fbctrl = get_ctrl();

	/* move buf in ready list to busy state */
	if (!fbctrl) {
		pr_err("%s: minifb was not initialized\n", __func__);
		return -EINVAL;
	}

	if (!list_empty(&fbctrl->busy_queue)) {
		pr_err("%s: already lock buf\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s:\n", __func__);
	mutex_lock(&fbctrl->lock);

	*vaddr = NULL;
	*ptr_size = 0;
	if (!list_empty(&fbctrl->ready_queue)) {
		node = list_first_entry(&fbctrl->ready_queue, struct minifb_data, entry);
		list_move(&node->entry, &fbctrl->busy_queue);
		pr_debug("%s: lock frame#%d from fd%d\n", __func__, node->info, node->buf_info.memory_id);

		/* map ion memory */
		*vaddr = ion_map_kernel(fbctrl->iclient, node->ionhdl);
		ion_handle_get_size(fbctrl->iclient, node->ionhdl, ptr_size);
		fbctrl->lock_cnt++;
	} else if (repeat && fbctrl->retired) {
		node = fbctrl->retired;
		fbctrl->retired = NULL;
		list_move(&node->entry, &fbctrl->busy_queue);
		pr_debug("%s: lock frame#%d from retired fd%d\n", __func__, node->info, node->buf_info.memory_id);

		/* map ion memory */
		*vaddr = ion_map_kernel(fbctrl->iclient, node->ionhdl);
		ion_handle_get_size(fbctrl->iclient, node->ionhdl, ptr_size);
		fbctrl->lock_cnt++;
	} else {
		pr_debug("%s: no buffer\n", __func__);
		fbctrl->retry_cnt++;
		ret = -EAGAIN;
	}

	mutex_unlock(&fbctrl->lock);
	put_ctrl(fbctrl);

	return ret;
}

void minifb_unlockbuf(void)
{
	struct minifb_data *node;
	struct minifb_ctrl *fbctrl = get_ctrl();

	if (!fbctrl) {
		pr_err("%s: minifb was not initialized\n", __func__);
		return;
	}

	pr_debug("%s:\n", __func__);
	mutex_lock(&fbctrl->lock);

	/* move fb buf from busy state to free list */
	if (!list_empty(&fbctrl->busy_queue)) {
		node = list_first_entry(&fbctrl->busy_queue, struct minifb_data, entry);
		fbctrl->retired = node;
		pr_debug("%s: unlock node#%d\n", __func__, node->info);
		list_move_tail(&node->entry, &fbctrl->free_queue);
		ion_unmap_kernel(fbctrl->iclient, node->ionhdl);
	} else {
		pr_warn("%s: no buffer was in busy!\n", __func__);
	}

	mutex_unlock(&fbctrl->lock);
	put_ctrl(fbctrl);

	return;
}

int minifb_ioctl_handler(unsigned int cmd, void *argp)
{
	int ret = -ENOSYS;
	struct minifb_session sess;
	struct minifb_req data;

	switch (cmd) {
	case MINIFB_INIT:
		ret = copy_from_user(&sess, argp, sizeof(sess));
		if (ret)
			return ret;
		ret = minifb_init(&sess);
		if (!ret)
			ret = copy_to_user(argp, &sess, sizeof(sess));
		break;

	case MINIFB_TERMINATE:
		ret = copy_from_user(&sess, argp, sizeof(sess));
		if (ret)
			return ret;
		ret = minifb_terminate(&sess);
		break;

	case MINIFB_QUEUE_BUFFER:
		ret = copy_from_user(&data, argp, sizeof(data));
		if (ret)
			return ret;
		ret = minifb_queuebuf(&data);
		break;

	case MINIFB_DEQUEUE_BUFFER:
		ret = copy_from_user(&data, argp, sizeof(data));
		if (ret)
			return ret;
		ret = minifb_dequeuebuf(&data);

		if (!ret)
			ret = copy_to_user(argp, &data, sizeof(data));
		break;

	default:
		break;
	}

	return ret;
}
