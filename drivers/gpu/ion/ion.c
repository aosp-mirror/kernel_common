/*

 * drivers/gpu/ion/ion.c
 *
 * Copyright (C) 2011 Google, Inc.
 * Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/file.h>
#include <linux/freezer.h>
#include <linux/fs.h>
#include <linux/anon_inodes.h>
#include <linux/ion.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/list_sort.h>
#include <linux/memblock.h>
#include <linux/miscdevice.h>
#include <linux/export.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/rbtree.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/dma-buf.h>
#include <linux/idr.h>
#include <linux/msm_ion.h>
#include <trace/events/kmem.h>


#include "ion_priv.h"

struct ion_device {
	struct miscdevice dev;
	struct rb_root buffers;
	struct mutex buffer_lock;
	struct rw_semaphore lock;
	struct plist_head heaps;
	long (*custom_ioctl) (struct ion_client *client, unsigned int cmd,
			      unsigned long arg);
	struct rb_root clients;
	struct dentry *debug_root;
	struct dentry *heaps_debug_root;
	struct dentry *clients_debug_root;
};

struct ion_client {
	struct rb_node node;
	struct ion_device *dev;
	struct rb_root handles;
	struct idr idr;
	struct mutex lock;
	char *name;
	const char *debug_name;
	struct task_struct *task;
	pid_t pid;
	struct dentry *debug_root;
};

struct ion_handle {
	struct kref ref;
	struct ion_client *client;
	struct ion_buffer *buffer;
	struct rb_node node;
	unsigned int kmap_cnt;
	int id;
};

bool ion_buffer_fault_user_mappings(struct ion_buffer *buffer)
{
        return ((buffer->flags & ION_FLAG_CACHED) &&
                !(buffer->flags & ION_FLAG_CACHED_NEEDS_SYNC));
}

bool ion_buffer_cached(struct ion_buffer *buffer)
{
        return !!(buffer->flags & ION_FLAG_CACHED);
}

static void ion_buffer_add(struct ion_device *dev,
			   struct ion_buffer *buffer)
{
	struct rb_node **p = &dev->buffers.rb_node;
	struct rb_node *parent = NULL;
	struct ion_buffer *entry;

	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct ion_buffer, node);

		if (buffer < entry) {
			p = &(*p)->rb_left;
		} else if (buffer > entry) {
			p = &(*p)->rb_right;
		} else {
			pr_err("%s: buffer already found.", __func__);
			BUG();
		}
	}

	rb_link_node(&buffer->node, parent, p);
	rb_insert_color(&buffer->node, &dev->buffers);
}

static int ion_buffer_alloc_dirty(struct ion_buffer *buffer);

static struct ion_buffer *ion_buffer_create(struct ion_heap *heap,
				     struct ion_device *dev,
				     unsigned long len,
				     unsigned long align,
				     unsigned long flags)
{
	struct ion_buffer *buffer;
	struct sg_table *table;
	struct scatterlist *sg;
	int i, ret;

	buffer = kzalloc(sizeof(struct ion_buffer), GFP_KERNEL);
	if (!buffer)
		return ERR_PTR(-ENOMEM);

	buffer->heap = heap;
	buffer->flags = flags;
	kref_init(&buffer->ref);

	ret = heap->ops->allocate(heap, buffer, len, align, flags);

	if (ret) {
		if (!(heap->flags & ION_HEAP_FLAG_DEFER_FREE))
			goto err2;

		ion_heap_freelist_drain(heap, 0);
		ret = heap->ops->allocate(heap, buffer, len, align,
					  flags);
		if (ret)
			goto err2;
	}

	buffer->dev = dev;
	buffer->size = len;

	table = heap->ops->map_dma(heap, buffer);
	if (IS_ERR_OR_NULL(table)) {
		heap->ops->free(buffer);
		kfree(buffer);
		return ERR_PTR(PTR_ERR(table));
	}
	buffer->sg_table = table;
	if (ion_buffer_fault_user_mappings(buffer)) {
		for_each_sg(buffer->sg_table->sgl, sg, buffer->sg_table->nents,
			    i) {
			if (sg_dma_len(sg) == PAGE_SIZE)
				continue;
			pr_err("%s: cached mappings that will be faulted in "
			       "must have pagewise sg_lists\n", __func__);
			ret = -EINVAL;
			goto err;
		}

		ret = ion_buffer_alloc_dirty(buffer);
		if (ret)
			goto err;
	}

	buffer->dev = dev;
	buffer->size = len;
	INIT_LIST_HEAD(&buffer->vmas);
	mutex_init(&buffer->lock);
	for_each_sg(buffer->sg_table->sgl, sg, buffer->sg_table->nents, i) {
		if (sg_dma_address(sg) == 0)
			sg_dma_address(sg) = sg_phys(sg);
	}
	mutex_lock(&dev->buffer_lock);
	ion_buffer_add(dev, buffer);
	mutex_unlock(&dev->buffer_lock);
	return buffer;

err:
	heap->ops->unmap_dma(heap, buffer);
	heap->ops->free(buffer);
err2:
	kfree(buffer);
	return ERR_PTR(ret);
}

static void ion_delayed_unsecure(struct ion_buffer *buffer)
{
	if (buffer->heap->ops->unsecure_buffer)
		buffer->heap->ops->unsecure_buffer(buffer, 1);
}

void ion_buffer_destroy(struct ion_buffer *buffer)
{
	if (WARN_ON(buffer->kmap_cnt > 0))
		buffer->heap->ops->unmap_kernel(buffer->heap, buffer);
	buffer->heap->ops->unmap_dma(buffer->heap, buffer);

	ion_delayed_unsecure(buffer);
	buffer->heap->ops->free(buffer);
	if (buffer->flags & ION_FLAG_CACHED)
		kfree(buffer->dirty);
	kfree(buffer);
}

static void _ion_buffer_destroy(struct kref *kref)
{
	struct ion_buffer *buffer = container_of(kref, struct ion_buffer, ref);
	struct ion_heap *heap = buffer->heap;
	struct ion_device *dev = buffer->dev;

	mutex_lock(&dev->buffer_lock);
	rb_erase(&buffer->node, &dev->buffers);
	mutex_unlock(&dev->buffer_lock);

	if (heap->flags & ION_HEAP_FLAG_DEFER_FREE)
		ion_heap_freelist_add(heap, buffer);
	else
		ion_buffer_destroy(buffer);
}

static void ion_buffer_get(struct ion_buffer *buffer)
{
	kref_get(&buffer->ref);
}

static int ion_buffer_put(struct ion_buffer *buffer)
{
	return kref_put(&buffer->ref, _ion_buffer_destroy);
}

static void ion_buffer_add_to_handle(struct ion_buffer *buffer)
{
	mutex_lock(&buffer->lock);
	buffer->handle_count++;
	mutex_unlock(&buffer->lock);
}

static void ion_buffer_remove_from_handle(struct ion_buffer *buffer)
{
	mutex_lock(&buffer->lock);
	buffer->handle_count--;
	BUG_ON(buffer->handle_count < 0);
	if (!buffer->handle_count) {
		struct task_struct *task;

		task = current->group_leader;
		get_task_comm(buffer->task_comm, task);
		buffer->pid = task_pid_nr(task);
	}
	mutex_unlock(&buffer->lock);
}

static struct ion_handle *ion_handle_create(struct ion_client *client,
				     struct ion_buffer *buffer)
{
	struct ion_handle *handle;

	handle = kzalloc(sizeof(struct ion_handle), GFP_KERNEL);
	if (!handle)
		return ERR_PTR(-ENOMEM);
	kref_init(&handle->ref);
	rb_init_node(&handle->node);
	handle->client = client;
	ion_buffer_get(buffer);
	ion_buffer_add_to_handle(buffer);
	handle->buffer = buffer;

	return handle;
}

static void ion_handle_kmap_put(struct ion_handle *);

static void ion_handle_destroy(struct kref *kref)
{
	struct ion_handle *handle = container_of(kref, struct ion_handle, ref);
	struct ion_client *client = handle->client;
	struct ion_buffer *buffer = handle->buffer;

	mutex_lock(&buffer->lock);
	while (handle->kmap_cnt)
		ion_handle_kmap_put(handle);
	mutex_unlock(&buffer->lock);

	idr_remove(&client->idr, handle->id);
	if (!RB_EMPTY_NODE(&handle->node))
		rb_erase(&handle->node, &client->handles);

	ion_buffer_remove_from_handle(buffer);
	ion_buffer_put(buffer);

	kfree(handle);
}

struct ion_buffer *ion_handle_buffer(struct ion_handle *handle)
{
	return handle->buffer;
}

static void ion_handle_get(struct ion_handle *handle)
{
	kref_get(&handle->ref);
}

int ion_handle_put(struct ion_handle *handle)
{
	struct ion_client *client = handle->client;
	int ret;

	mutex_lock(&client->lock);
	ret = kref_put(&handle->ref, ion_handle_destroy);
	mutex_unlock(&client->lock);

	return ret;
}

static struct ion_handle *ion_handle_lookup(struct ion_client *client,
					    struct ion_buffer *buffer)
{
	struct rb_node *n;

	for (n = rb_first(&client->handles); n; n = rb_next(n)) {
		struct ion_handle *handle = rb_entry(n, struct ion_handle,
						   node);
		if (handle->buffer == buffer)
			return handle;
	}
	return NULL;
}

struct ion_handle *ion_handle_get_by_id(struct ion_client *client,
						int id)
{
	struct ion_handle *handle;

	mutex_lock(&client->lock);
	handle = idr_find(&client->idr, id);
	if (handle)
		ion_handle_get(handle);
	mutex_unlock(&client->lock);

	return handle ? handle : ERR_PTR(-EINVAL);
}

static bool ion_handle_validate(struct ion_client *client, struct ion_handle *handle)
{
	WARN_ON(!mutex_is_locked(&client->lock));
	return (idr_find(&client->idr, handle->id) == handle);
}

static int ion_handle_add(struct ion_client *client, struct ion_handle *handle)
{
	int rc;
	struct rb_node **p = &client->handles.rb_node;
	struct rb_node *parent = NULL;
	struct ion_handle *entry;

	do {
		int id;
		rc = idr_pre_get(&client->idr, GFP_KERNEL);
		if (!rc)
			return -ENOMEM;
		rc = idr_get_new_above(&client->idr, handle, 1, &id);
		handle->id = id;
	} while (rc == -EAGAIN);

	if (rc < 0)
		return rc;

	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct ion_handle, node);

		if (handle < entry)
			p = &(*p)->rb_left;
		else if (handle > entry)
			p = &(*p)->rb_right;
		else
			WARN(1, "%s: buffer already found.", __func__);
	}

	rb_link_node(&handle->node, parent, p);
	rb_insert_color(&handle->node, &client->handles);

	return 0;
}

struct ion_handle *ion_alloc(struct ion_client *client, size_t len,
			     size_t align, unsigned int heap_id_mask,
			     unsigned int flags)
{
	struct ion_handle *handle;
	struct ion_device *dev = client->dev;
	struct ion_buffer *buffer = NULL;
	struct ion_heap *heap;
	int ret;
	unsigned long secure_allocation = flags & ION_FLAG_SECURE;
	const unsigned int MAX_DBG_STR_LEN = 64;
	char dbg_str[MAX_DBG_STR_LEN];
	unsigned int dbg_str_idx = 0;

	dbg_str[0] = '\0';

	flags |= ION_FLAG_CACHED_NEEDS_SYNC;

	pr_debug("%s: len %d align %d heap_id_mask %u flags %x\n", __func__,
		 len, align, heap_id_mask, flags);
	if (WARN_ON(!len))
		return ERR_PTR(-EINVAL);

	len = PAGE_ALIGN(len);

	down_read(&dev->lock);
	plist_for_each_entry(heap, &dev->heaps, node) {
		
		if (!((1 << heap->id) & heap_id_mask))
			continue;
		
		if (secure_allocation &&
		    !ion_heap_allow_secure_allocation(heap->type))
			continue;
		trace_ion_alloc_buffer_start(client->name, heap->name, len,
					     heap_id_mask, flags);
		buffer = ion_buffer_create(heap, dev, len, align, flags);
		trace_ion_alloc_buffer_end(client->name, heap->name, len,
					   heap_id_mask, flags);
		if (!IS_ERR_OR_NULL(buffer))
			break;

		trace_ion_alloc_buffer_fallback(client->name, heap->name, len,
					    heap_id_mask, flags,
					    PTR_ERR(buffer));
		if (dbg_str_idx < MAX_DBG_STR_LEN) {
			unsigned int len_left = MAX_DBG_STR_LEN-dbg_str_idx-1;
			int ret_value = snprintf(&dbg_str[dbg_str_idx],
						len_left, "%s ", heap->name);
			if (ret_value >= len_left) {
				
				dbg_str[MAX_DBG_STR_LEN-1] = '\0';
				dbg_str_idx = MAX_DBG_STR_LEN;
			} else if (ret_value >= 0) {
				dbg_str_idx += ret_value;
			} else {
				
				dbg_str[MAX_DBG_STR_LEN-1] = '\0';
			}
		}
	}
	up_read(&dev->lock);

	if (buffer == NULL) {
		trace_ion_alloc_buffer_fail(client->name, dbg_str, len,
					    heap_id_mask, flags, -ENODEV);
		return ERR_PTR(-ENODEV);
	}

	if (IS_ERR(buffer)) {
		trace_ion_alloc_buffer_fail(client->name, dbg_str, len,
					    heap_id_mask, flags,
					    PTR_ERR(buffer));
		pr_debug("ION is unable to allocate 0x%x bytes (alignment: "
			 "0x%x) from heap(s) %sfor client %s\n",
			len, align, dbg_str, client->name);
		return ERR_PTR(PTR_ERR(buffer));
	}

	handle = ion_handle_create(client, buffer);

	ion_buffer_put(buffer);

	if (IS_ERR(handle))
		return handle;

	mutex_lock(&client->lock);
	ret = ion_handle_add(client, handle);
	mutex_unlock(&client->lock);
	if (ret) {
		ion_handle_put(handle);
		handle = ERR_PTR(ret);
	}

	return handle;
}
EXPORT_SYMBOL(ion_alloc);

void ion_free(struct ion_client *client, struct ion_handle *handle)
{
	bool valid_handle;

	BUG_ON(client != handle->client);

	mutex_lock(&client->lock);
	valid_handle = ion_handle_validate(client, handle);
	if (!valid_handle) {
		WARN(1, "%s: invalid handle passed to free.\n", __func__);
		mutex_unlock(&client->lock);
		return;
	}
	mutex_unlock(&client->lock);
	ion_handle_put(handle);
}
EXPORT_SYMBOL(ion_free);

int ion_phys(struct ion_client *client, struct ion_handle *handle,
	     ion_phys_addr_t *addr, size_t *len)
{
	struct ion_buffer *buffer;
	int ret;

	mutex_lock(&client->lock);
	if (!ion_handle_validate(client, handle)) {
		mutex_unlock(&client->lock);
		return -EINVAL;
	}

	buffer = handle->buffer;

	if (!buffer->heap->ops->phys) {
		pr_err("%s: ion_phys is not implemented by this heap.\n",
		       __func__);
		mutex_unlock(&client->lock);
		return -ENODEV;
	}
	mutex_unlock(&client->lock);
	ret = buffer->heap->ops->phys(buffer->heap, buffer, addr, len);
	return ret;
}
EXPORT_SYMBOL(ion_phys);

static void *ion_buffer_kmap_get(struct ion_buffer *buffer)
{
	void *vaddr;

	if (buffer->kmap_cnt) {
		buffer->kmap_cnt++;
		return buffer->vaddr;
	}
	vaddr = buffer->heap->ops->map_kernel(buffer->heap, buffer);
	if (IS_ERR_OR_NULL(vaddr))
		return vaddr;
	buffer->vaddr = vaddr;
	buffer->kmap_cnt++;
	return vaddr;
}

static void *ion_handle_kmap_get(struct ion_handle *handle)
{
	struct ion_buffer *buffer = handle->buffer;
	void *vaddr;

	if (handle->kmap_cnt) {
		handle->kmap_cnt++;
		return buffer->vaddr;
	}
	vaddr = ion_buffer_kmap_get(buffer);
	if (IS_ERR_OR_NULL(vaddr))
		return vaddr;
	handle->kmap_cnt++;
	return vaddr;
}

static void ion_buffer_kmap_put(struct ion_buffer *buffer)
{
	buffer->kmap_cnt--;
	if (!buffer->kmap_cnt) {
		buffer->heap->ops->unmap_kernel(buffer->heap, buffer);
		buffer->vaddr = NULL;
	}
}

static void ion_handle_kmap_put(struct ion_handle *handle)
{
	struct ion_buffer *buffer = handle->buffer;

	handle->kmap_cnt--;
	if (!handle->kmap_cnt)
		ion_buffer_kmap_put(buffer);
}

void *ion_map_kernel(struct ion_client *client, struct ion_handle *handle)
{
	struct ion_buffer *buffer;
	void *vaddr;

	mutex_lock(&client->lock);
	if (!ion_handle_validate(client, handle)) {
		pr_err("%s: invalid handle passed to map_kernel.\n",
		       __func__);
		mutex_unlock(&client->lock);
		return ERR_PTR(-EINVAL);
	}

	buffer = handle->buffer;

	if (!handle->buffer->heap->ops->map_kernel) {
		pr_err("%s: map_kernel is not implemented by this heap.\n",
		       __func__);
		mutex_unlock(&client->lock);
		return ERR_PTR(-ENODEV);
	}

	mutex_lock(&buffer->lock);
	vaddr = ion_handle_kmap_get(handle);
	mutex_unlock(&buffer->lock);
	mutex_unlock(&client->lock);
	return vaddr;
}
EXPORT_SYMBOL(ion_map_kernel);

void ion_unmap_kernel(struct ion_client *client, struct ion_handle *handle)
{
	struct ion_buffer *buffer;

	mutex_lock(&client->lock);
	buffer = handle->buffer;
	mutex_lock(&buffer->lock);
	ion_handle_kmap_put(handle);
	mutex_unlock(&buffer->lock);
	mutex_unlock(&client->lock);
}
EXPORT_SYMBOL(ion_unmap_kernel);

static int ion_debug_client_show(struct seq_file *s, void *unused)
{
	struct ion_client *client = s->private;
	struct rb_node *n;

	seq_printf(s, "%16.16s: %16.16s : %16.16s : %12.12s\n",
			"heap_name", "size_in_bytes", "handle refcount",
			"buffer");

	mutex_lock(&client->lock);
	for (n = rb_first(&client->handles); n; n = rb_next(n)) {
		struct ion_handle *handle = rb_entry(n, struct ion_handle,
						     node);

		seq_printf(s, "%16.16s: %16x : %16d : %12p",
				handle->buffer->heap->name,
				handle->buffer->size,
				atomic_read(&handle->ref.refcount),
				handle->buffer);

		seq_printf(s, "\n");
	}
	mutex_unlock(&client->lock);
	return 0;
}

static int ion_debug_client_open(struct inode *inode, struct file *file)
{
	return single_open(file, ion_debug_client_show, inode->i_private);
}

static const struct file_operations debug_client_fops = {
	.open = ion_debug_client_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static bool startswith(const char *string, const char *prefix)
{
	size_t l1 = strlen(string);
	size_t l2 = strlen(prefix);
	return strncmp(string, prefix, min(l1, l2)) == 0;
}

static int ion_get_client_serial(const struct rb_root *root,
					const unsigned char *name)
{
	int serial = -1;
	struct rb_node *node;
	for (node = rb_first(root); node; node = rb_next(node)) {
		int n;
		char *serial_string;
		struct ion_client *client = rb_entry(node, struct ion_client,
						node);
		if (!startswith(client->name, name))
			continue;
		serial_string = strrchr(client->name, '-');
		if (!serial_string)
			continue;
		serial_string++;
		sscanf(serial_string, "%d", &n);
		serial = max(serial, n);
	}
	return serial + 1;
}

struct ion_client *ion_client_create(struct ion_device *dev,
				     const char *name)
{
	struct ion_client *client;
	struct task_struct *task;
	struct rb_node **p;
	struct rb_node *parent = NULL;
	struct ion_client *entry;
	pid_t pid;
	int name_len;
	int client_serial;

	if (!name) {
		pr_err("%s: Name cannot be null\n", __func__);
		return ERR_PTR(-EINVAL);
	}
	name_len = strnlen(name, 64);
	
	name_len = min(64, name_len + 11);

	get_task_struct(current->group_leader);
	task_lock(current->group_leader);
	pid = task_pid_nr(current->group_leader);
	if (current->group_leader->flags & PF_KTHREAD) {
		put_task_struct(current->group_leader);
		task = NULL;
	} else {
		task = current->group_leader;
	}
	task_unlock(current->group_leader);

	client = kzalloc(sizeof(struct ion_client), GFP_KERNEL);
	if (!client) {
		if (task)
			put_task_struct(current->group_leader);
		return ERR_PTR(-ENOMEM);
	}

	client->dev = dev;
	client->handles = RB_ROOT;
	idr_init(&client->idr);
	mutex_init(&client->lock);

	client->name = kzalloc(name_len+1, GFP_KERNEL);
	if (!client->name) {
		put_task_struct(current->group_leader);
		kfree(client);
		return ERR_PTR(-ENOMEM);
	}

	client->task = task;
	client->pid = pid;

	down_write(&dev->lock);
	client_serial = ion_get_client_serial(&dev->clients, name);
	snprintf(client->name, name_len, "%s-%d", name, client_serial);
	p = &dev->clients.rb_node;
	while (*p) {
		parent = *p;
		entry = rb_entry(parent, struct ion_client, node);

		if (client < entry)
			p = &(*p)->rb_left;
		else if (client > entry)
			p = &(*p)->rb_right;
	}
	rb_link_node(&client->node, parent, p);
	rb_insert_color(&client->node, &dev->clients);


	client->debug_root = debugfs_create_file(client->name, 0664,
						dev->clients_debug_root,
						client, &debug_client_fops);
	if (!client->debug_root) {
		char buf[256], *path;
		path = dentry_path(dev->clients_debug_root, buf, 256);
		pr_err("Failed to created client debugfs at %s/%s\n",
			path, client->name);
	}

	up_write(&dev->lock);

	return client;
}
EXPORT_SYMBOL(ion_client_create);

int ion_client_set_debug_name(struct ion_client *client, const char *debug_name)
{
	int ret = 0;
	if (!debug_name) {
		pr_err("%s: debug name cannot be null\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&client->lock);
	if (client->debug_name) {
		kfree(client->debug_name);
	}

	client->debug_name = kstrndup(debug_name, 64, GFP_KERNEL);
	if (!client->debug_name) {
		pr_err("%s: nomem to kstrdup '%s'\n", __func__, debug_name);
		ret = -ENOMEM;
	}

	mutex_unlock(&client->lock);
	return ret;
}
EXPORT_SYMBOL(ion_client_set_debug_name);

void ion_client_destroy(struct ion_client *client)
{
	struct ion_device *dev = client->dev;
	struct rb_node *n;

	pr_debug("%s: %d\n", __func__, __LINE__);
	while ((n = rb_first(&client->handles))) {
		struct ion_handle *handle = rb_entry(n, struct ion_handle,
						     node);
		ion_handle_destroy(&handle->ref);
	}

	idr_remove_all(&client->idr);
	idr_destroy(&client->idr);

	down_write(&dev->lock);
	if (client->task)
		put_task_struct(client->task);
	rb_erase(&client->node, &dev->clients);
	debugfs_remove_recursive(client->debug_root);

	up_write(&dev->lock);

	if (client->debug_name)
		kfree(client->debug_name);
	kfree(client->name);
	kfree(client);
}
EXPORT_SYMBOL(ion_client_destroy);

int ion_handle_get_flags(struct ion_client *client, struct ion_handle *handle,
			unsigned long *flags)
{
	struct ion_buffer *buffer;

	mutex_lock(&client->lock);
	if (!ion_handle_validate(client, handle)) {
		pr_err("%s: invalid handle passed to %s.\n",
		       __func__, __func__);
		mutex_unlock(&client->lock);
		return -EINVAL;
	}
	buffer = handle->buffer;
	mutex_lock(&buffer->lock);
	*flags = buffer->flags;
	mutex_unlock(&buffer->lock);
	mutex_unlock(&client->lock);

	return 0;
}
EXPORT_SYMBOL(ion_handle_get_flags);

int ion_handle_get_size(struct ion_client *client, struct ion_handle *handle,
			unsigned long *size)
{
	struct ion_buffer *buffer;

	mutex_lock(&client->lock);
	if (!ion_handle_validate(client, handle)) {
		pr_err("%s: invalid handle passed to %s.\n",
		       __func__, __func__);
		mutex_unlock(&client->lock);
		return -EINVAL;
	}
	buffer = handle->buffer;
	mutex_lock(&buffer->lock);
	*size = buffer->size;
	mutex_unlock(&buffer->lock);
	mutex_unlock(&client->lock);

	return 0;
}
EXPORT_SYMBOL(ion_handle_get_size);

struct sg_table *ion_sg_table(struct ion_client *client,
			      struct ion_handle *handle)
{
	struct ion_buffer *buffer;
	struct sg_table *table;

	mutex_lock(&client->lock);
	if (!ion_handle_validate(client, handle)) {
		pr_err("%s: invalid handle passed to map_dma.\n",
		       __func__);
		mutex_unlock(&client->lock);
		return ERR_PTR(-EINVAL);
	}
	buffer = handle->buffer;
	table = buffer->sg_table;
	mutex_unlock(&client->lock);
	return table;
}
EXPORT_SYMBOL(ion_sg_table);

struct sg_table *ion_create_chunked_sg_table(phys_addr_t buffer_base,
					size_t chunk_size, size_t total_size)
{
	struct sg_table *table;
	int i, n_chunks, ret;
	struct scatterlist *sg;

	table = kzalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!table)
		return ERR_PTR(-ENOMEM);

	n_chunks = DIV_ROUND_UP(total_size, chunk_size);
	pr_debug("creating sg_table with %d chunks\n", n_chunks);

	ret = sg_alloc_table(table, n_chunks, GFP_KERNEL);
	if (ret)
		goto err0;

	for_each_sg(table->sgl, sg, table->nents, i) {
		dma_addr_t addr = buffer_base + i * chunk_size;
		sg_dma_address(sg) = addr;
		sg_dma_len(sg) = chunk_size;
	}

	return table;
err0:
	kfree(table);
	return ERR_PTR(ret);
}

static void ion_buffer_sync_for_device(struct ion_buffer *buffer,
				       struct device *dev,
				       enum dma_data_direction direction);

static struct sg_table *ion_map_dma_buf(struct dma_buf_attachment *attachment,
					enum dma_data_direction direction)
{
	struct dma_buf *dmabuf = attachment->dmabuf;
	struct ion_buffer *buffer = dmabuf->priv;

	ion_buffer_sync_for_device(buffer, attachment->dev, direction);
	return buffer->sg_table;
}

static void ion_unmap_dma_buf(struct dma_buf_attachment *attachment,
			      struct sg_table *table,
			      enum dma_data_direction direction)
{
}

static int ion_buffer_alloc_dirty(struct ion_buffer *buffer)
{
	unsigned long pages = buffer->sg_table->nents;
	unsigned long length = (pages + BITS_PER_LONG - 1)/BITS_PER_LONG;

	buffer->dirty = kzalloc(length * sizeof(unsigned long), GFP_KERNEL);
	if (!buffer->dirty)
		return -ENOMEM;
	return 0;
}

struct ion_vma_list {
	struct list_head list;
	struct vm_area_struct *vma;
};

static void ion_buffer_sync_for_device(struct ion_buffer *buffer,
				       struct device *dev,
				       enum dma_data_direction dir)
{
	struct scatterlist *sg;
	int i;
	struct ion_vma_list *vma_list;

	pr_debug("%s: syncing for device %s\n", __func__,
		 dev ? dev_name(dev) : "null");

	if (!ion_buffer_fault_user_mappings(buffer))
		return;

	mutex_lock(&buffer->lock);
	for_each_sg(buffer->sg_table->sgl, sg, buffer->sg_table->nents, i) {
		if (!test_bit(i, buffer->dirty))
			continue;
		dma_sync_sg_for_device(dev, sg, 1, dir);
		clear_bit(i, buffer->dirty);
	}
	list_for_each_entry(vma_list, &buffer->vmas, list) {
		struct vm_area_struct *vma = vma_list->vma;

		zap_page_range(vma, vma->vm_start, vma->vm_end - vma->vm_start,
			       NULL);
	}
	mutex_unlock(&buffer->lock);
}

int ion_vm_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct ion_buffer *buffer = vma->vm_private_data;
	struct scatterlist *sg;
	int i;

	mutex_lock(&buffer->lock);
	set_bit(vmf->pgoff, buffer->dirty);

	for_each_sg(buffer->sg_table->sgl, sg, buffer->sg_table->nents, i) {
		if (i != vmf->pgoff)
			continue;
		dma_sync_sg_for_cpu(NULL, sg, 1, DMA_BIDIRECTIONAL);
		vm_insert_page(vma, (unsigned long)vmf->virtual_address,
			       sg_page(sg));
		break;
	}
	mutex_unlock(&buffer->lock);
	return VM_FAULT_NOPAGE;
}

static void ion_vm_open(struct vm_area_struct *vma)
{
	struct ion_buffer *buffer = vma->vm_private_data;
	struct ion_vma_list *vma_list;

	vma_list = kmalloc(sizeof(struct ion_vma_list), GFP_KERNEL);
	if (!vma_list)
		return;
	vma_list->vma = vma;
	mutex_lock(&buffer->lock);
	list_add(&vma_list->list, &buffer->vmas);
	mutex_unlock(&buffer->lock);
	pr_debug("%s: adding %p\n", __func__, vma);
}

static void ion_vm_close(struct vm_area_struct *vma)
{
	struct ion_buffer *buffer = vma->vm_private_data;
	struct ion_vma_list *vma_list, *tmp;

	pr_debug("%s\n", __func__);
	mutex_lock(&buffer->lock);
	list_for_each_entry_safe(vma_list, tmp, &buffer->vmas, list) {
		if (vma_list->vma != vma)
			continue;
		list_del(&vma_list->list);
		kfree(vma_list);
		pr_debug("%s: deleting %p\n", __func__, vma);
		break;
	}
	mutex_unlock(&buffer->lock);

	if (buffer->heap->ops->unmap_user)
		buffer->heap->ops->unmap_user(buffer->heap, buffer);
}

struct vm_operations_struct ion_vma_ops = {
	.open = ion_vm_open,
	.close = ion_vm_close,
	.fault = ion_vm_fault,
};

static int ion_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct ion_buffer *buffer = dmabuf->priv;
	int ret = 0;

	if (!buffer->heap->ops->map_user) {
		pr_err("%s: this heap does not define a method for mapping "
		       "to userspace\n", __func__);
		return -EINVAL;
	}

	if (ion_buffer_fault_user_mappings(buffer)) {
		vma->vm_private_data = buffer;
		vma->vm_ops = &ion_vma_ops;
		vma->vm_flags |= VM_MIXEDMAP;
		ion_vm_open(vma);
		return 0;
	}

	if (!(buffer->flags & ION_FLAG_CACHED))
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	mutex_lock(&buffer->lock);
	
	ret = buffer->heap->ops->map_user(buffer->heap, buffer, vma);
	mutex_unlock(&buffer->lock);

	if (ret)
		pr_err("%s: failure mapping buffer to userspace\n",
		       __func__);

	return ret;
}

static void ion_dma_buf_release(struct dma_buf *dmabuf)
{
	struct ion_buffer *buffer = dmabuf->priv;
	ion_buffer_put(buffer);
}

static void *ion_dma_buf_kmap(struct dma_buf *dmabuf, unsigned long offset)
{
	struct ion_buffer *buffer = dmabuf->priv;
	return buffer->vaddr + offset * PAGE_SIZE;
}

static void ion_dma_buf_kunmap(struct dma_buf *dmabuf, unsigned long offset,
			       void *ptr)
{
	return;
}

static int ion_dma_buf_begin_cpu_access(struct dma_buf *dmabuf, size_t start,
					size_t len,
					enum dma_data_direction direction)
{
	struct ion_buffer *buffer = dmabuf->priv;
	void *vaddr;

	if (!buffer->heap->ops->map_kernel) {
		pr_err("%s: map kernel is not implemented by this heap.\n",
		       __func__);
		return -ENODEV;
	}

	mutex_lock(&buffer->lock);
	vaddr = ion_buffer_kmap_get(buffer);
	mutex_unlock(&buffer->lock);
	if (IS_ERR(vaddr))
		return PTR_ERR(vaddr);
	if (!vaddr)
		return -ENOMEM;
	return 0;
}

static void ion_dma_buf_end_cpu_access(struct dma_buf *dmabuf, size_t start,
				       size_t len,
				       enum dma_data_direction direction)
{
	struct ion_buffer *buffer = dmabuf->priv;

	mutex_lock(&buffer->lock);
	ion_buffer_kmap_put(buffer);
	mutex_unlock(&buffer->lock);
}

struct dma_buf_ops dma_buf_ops = {
	.map_dma_buf = ion_map_dma_buf,
	.unmap_dma_buf = ion_unmap_dma_buf,
	.mmap = ion_mmap,
	.release = ion_dma_buf_release,
	.begin_cpu_access = ion_dma_buf_begin_cpu_access,
	.end_cpu_access = ion_dma_buf_end_cpu_access,
	.kmap_atomic = ion_dma_buf_kmap,
	.kunmap_atomic = ion_dma_buf_kunmap,
	.kmap = ion_dma_buf_kmap,
	.kunmap = ion_dma_buf_kunmap,
};

struct dma_buf *ion_share_dma_buf(struct ion_client *client,
						struct ion_handle *handle)
{
	struct ion_buffer *buffer;
	struct dma_buf *dmabuf;
	bool valid_handle;

	mutex_lock(&client->lock);
	valid_handle = ion_handle_validate(client, handle);
	if (!valid_handle) {
		WARN(1, "%s: invalid handle passed to share.\n", __func__);
		mutex_unlock(&client->lock);
		return ERR_PTR(-EINVAL);
	}
	buffer = handle->buffer;
	ion_buffer_get(buffer);
	mutex_unlock(&client->lock);

	dmabuf = dma_buf_export(buffer, &dma_buf_ops, buffer->size, O_RDWR);
	if (IS_ERR(dmabuf)) {
		ion_buffer_put(buffer);
		return dmabuf;
	}

	return dmabuf;
}
EXPORT_SYMBOL(ion_share_dma_buf);

int ion_share_dma_buf_fd(struct ion_client *client, struct ion_handle *handle)
{
	struct dma_buf *dmabuf;
	int fd;

	dmabuf = ion_share_dma_buf(client, handle);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	fd = dma_buf_fd(dmabuf, O_CLOEXEC);
	if (fd < 0)
		dma_buf_put(dmabuf);

	return fd;
}
EXPORT_SYMBOL(ion_share_dma_buf_fd);

struct ion_handle *ion_import_dma_buf(struct ion_client *client, int fd)
{
	struct dma_buf *dmabuf;
	struct ion_buffer *buffer;
	struct ion_handle *handle;
	int ret;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR_OR_NULL(dmabuf))
		return ERR_PTR(PTR_ERR(dmabuf));
	

	if (dmabuf->ops != &dma_buf_ops) {
		pr_err("%s: can not import dmabuf from another exporter\n",
		       __func__);
		dma_buf_put(dmabuf);
		return ERR_PTR(-EINVAL);
	}
	buffer = dmabuf->priv;

	mutex_lock(&client->lock);
	
	handle = ion_handle_lookup(client, buffer);
	if (!IS_ERR_OR_NULL(handle)) {
		ion_handle_get(handle);
		mutex_unlock(&client->lock);
		goto end;
	}
	mutex_unlock(&client->lock);

	handle = ion_handle_create(client, buffer);
	if (IS_ERR_OR_NULL(handle))
		goto end;

	mutex_lock(&client->lock);
	ret = ion_handle_add(client, handle);
	mutex_unlock(&client->lock);
	if (ret) {
		ion_handle_put(handle);
		handle = ERR_PTR(ret);
	}

end:
	dma_buf_put(dmabuf);
	return handle;
}
EXPORT_SYMBOL(ion_import_dma_buf);

static int ion_sync_for_device(struct ion_client *client, int fd)
{
	struct dma_buf *dmabuf;
	struct ion_buffer *buffer;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR_OR_NULL(dmabuf))
		return PTR_ERR(dmabuf);

	
	if (dmabuf->ops != &dma_buf_ops) {
		pr_err("%s: can not sync dmabuf from another exporter\n",
		       __func__);
		dma_buf_put(dmabuf);
		return -EINVAL;
	}
	buffer = dmabuf->priv;

	dma_sync_sg_for_device(NULL, buffer->sg_table->sgl,
			       buffer->sg_table->nents, DMA_BIDIRECTIONAL);
	dma_buf_put(dmabuf);
	return 0;
}

static long ion_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct ion_client *client = filp->private_data;

	switch (cmd) {
	case ION_IOC_ALLOC:
	{
		struct ion_allocation_data data;
		struct ion_handle *handle;

		if (copy_from_user(&data, (void __user *)arg, sizeof(data)))
			return -EFAULT;
		handle = ion_alloc(client, data.len, data.align,
					     data.heap_mask, data.flags);

		if (IS_ERR(handle))
			return PTR_ERR(handle);

		data.handle = (ion_user_handle_t)handle->id;

		if (copy_to_user((void __user *)arg, &data, sizeof(data))) {
			ion_free(client, handle);
			return -EFAULT;
		}
		break;
	}
	case ION_IOC_FREE:
	{
		struct ion_handle_data data;
		struct ion_handle *handle;

		if (copy_from_user(&data, (void __user *)arg,
				   sizeof(struct ion_handle_data)))
			return -EFAULT;
		handle = ion_handle_get_by_id(client, (int)data.handle);
		if (IS_ERR(handle))
			return PTR_ERR(handle);
		ion_free(client, handle);
		ion_handle_put(handle);
		break;
	}
	case ION_IOC_SHARE:
	case ION_IOC_MAP:
	{
		struct ion_fd_data data;
		struct ion_handle *handle;
		if (copy_from_user(&data, (void __user *)arg, sizeof(data)))
			return -EFAULT;

		handle = ion_handle_get_by_id(client, (int)data.handle);
		if (IS_ERR(handle))
			return PTR_ERR(handle);
		data.fd = ion_share_dma_buf_fd(client, handle);
		ion_handle_put(handle);
		if (copy_to_user((void __user *)arg, &data, sizeof(data)))
			return -EFAULT;
		if (data.fd < 0)
			return data.fd;
		break;
	}
	case ION_IOC_IMPORT:
	{
		struct ion_fd_data data;
		struct ion_handle *handle;
		int ret = 0;
		if (copy_from_user(&data, (void __user *)arg,
				   sizeof(struct ion_fd_data)))
			return -EFAULT;
		handle = ion_import_dma_buf(client, data.fd);
		if (IS_ERR(handle))
			ret = PTR_ERR(handle);
		else
			data.handle = (ion_user_handle_t)handle->id;

		if (copy_to_user((void __user *)arg, &data,
				 sizeof(struct ion_fd_data)))
			return -EFAULT;
		if (ret < 0)
			return ret;
		break;
	}
	case ION_IOC_SYNC:
	{
		struct ion_fd_data data;
		if (copy_from_user(&data, (void __user *)arg,
				   sizeof(struct ion_fd_data)))
			return -EFAULT;
		ion_sync_for_device(client, data.fd);
		break;
	}
	case ION_IOC_CUSTOM:
	{
		struct ion_device *dev = client->dev;
		struct ion_custom_data data;

		if (!dev->custom_ioctl)
			return -ENOTTY;
		if (copy_from_user(&data, (void __user *)arg,
				sizeof(struct ion_custom_data)))
			return -EFAULT;
		return dev->custom_ioctl(client, data.cmd, data.arg);
	}
	case ION_IOC_CLEAN_CACHES:
		return client->dev->custom_ioctl(client,
						ION_IOC_CLEAN_CACHES, arg);
	case ION_IOC_INV_CACHES:
		return client->dev->custom_ioctl(client,
						ION_IOC_INV_CACHES, arg);
	case ION_IOC_CLEAN_INV_CACHES:
		return client->dev->custom_ioctl(client,
						ION_IOC_CLEAN_INV_CACHES, arg);
	default:
		return -ENOTTY;
	}
	return 0;
}

static int ion_release(struct inode *inode, struct file *file)
{
	struct ion_client *client = file->private_data;

	pr_debug("%s: %d\n", __func__, __LINE__);
	ion_client_destroy(client);
	return 0;
}

static int ion_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct ion_device *dev = container_of(miscdev, struct ion_device, dev);
	struct ion_client *client;
	char debug_name[64];

	pr_debug("%s: %d\n", __func__, __LINE__);
	snprintf(debug_name, 64, "%u", task_pid_nr(current->group_leader));
	client = ion_client_create(dev, debug_name);
	if (IS_ERR_OR_NULL(client))
		return PTR_ERR(client);
	file->private_data = client;

	return 0;
}

static const struct file_operations ion_fops = {
	.owner          = THIS_MODULE,
	.open           = ion_open,
	.release        = ion_release,
	.unlocked_ioctl = ion_ioctl,
};

static size_t ion_debug_heap_total(struct ion_client *client,
				   unsigned int id)
{
	size_t size = 0;
	struct rb_node *n;

	mutex_lock(&client->lock);
	for (n = rb_first(&client->handles); n; n = rb_next(n)) {
		struct ion_handle *handle = rb_entry(n,
						     struct ion_handle,
						     node);
		if (handle->buffer->heap->id == id)
			size += handle->buffer->size;
	}
	mutex_unlock(&client->lock);
	return size;
}

void ion_debug_mem_map_create(struct seq_file *s, struct ion_heap *heap,
			      struct list_head *mem_map)
{
	struct ion_device *dev = heap->dev;
	struct rb_node *cnode;
	size_t size;
	struct ion_client *client;

	if (!heap->ops->phys)
		return;

	down_read(&dev->lock);
	for (cnode = rb_first(&dev->clients); cnode; cnode = rb_next(cnode)) {
		struct rb_node *hnode;
		client = rb_entry(cnode, struct ion_client, node);

		mutex_lock(&client->lock);
		for (hnode = rb_first(&client->handles);
		     hnode;
		     hnode = rb_next(hnode)) {
			struct ion_handle *handle = rb_entry(
				hnode, struct ion_handle, node);
			if (handle->buffer->heap == heap) {
				struct mem_map_data *data =
					kzalloc(sizeof(*data), GFP_KERNEL);
				const char *name = client->debug_name ?
							client->debug_name : client->name;
				if (!data)
					goto inner_error;
				heap->ops->phys(heap, handle->buffer,
							&(data->addr), &size);
				data->size = (unsigned long) size;
				data->addr_end = data->addr + data->size - 1;
				data->client_name = kstrdup(name,
							GFP_KERNEL);
				if (!data->client_name) {
					kfree(data);
					goto inner_error;
				}
				list_add(&data->node, mem_map);
			}
		}
		mutex_unlock(&client->lock);
	}
	up_read(&dev->lock);
	return;

inner_error:
	seq_puts(s,
		"ERROR: out of memory. Part of memory map will not be logged\n");
	mutex_unlock(&client->lock);
	up_read(&dev->lock);
}

static void ion_debug_mem_map_destroy(struct list_head *mem_map)
{
	if (mem_map) {
		struct mem_map_data *data, *tmp;
		list_for_each_entry_safe(data, tmp, mem_map, node) {
			list_del(&data->node);
			kfree(data->client_name);
			kfree(data);
		}
	}
}

static int mem_map_cmp(void *priv, struct list_head *a, struct list_head *b)
{
	struct mem_map_data *d1, *d2;
	d1 = list_entry(a, struct mem_map_data, node);
	d2 = list_entry(b, struct mem_map_data, node);
	if (d1->addr == d2->addr)
		return d1->size - d2->size;
	return d1->addr - d2->addr;
}

static void ion_heap_print_debug(struct seq_file *s, struct ion_heap *heap)
{
	if (heap->ops->print_debug) {
		struct list_head mem_map = LIST_HEAD_INIT(mem_map);
		ion_debug_mem_map_create(s, heap, &mem_map);
		list_sort(NULL, &mem_map, mem_map_cmp);
		heap->ops->print_debug(heap, s, &mem_map);
		ion_debug_mem_map_destroy(&mem_map);
	}
}

static int ion_debug_heap_show(struct seq_file *s, void *unused)
{
	struct ion_heap *heap = s->private;
	struct ion_device *dev = heap->dev;
	struct rb_node *n;
	size_t total_size = 0;
	size_t total_orphaned_size = 0;

	seq_printf(s, "%16.s %16.s %16.s\n", "client", "pid", "size");
	seq_printf(s, "----------------------------------------------------\n");

	down_read(&dev->lock);
	for (n = rb_first(&dev->clients); n; n = rb_next(n)) {
		struct ion_client *client = rb_entry(n, struct ion_client,
						     node);
		size_t size = ion_debug_heap_total(client, heap->id);
		if (!size)
			continue;
		mutex_lock(&client->lock);
		if (client->debug_name) {
			seq_printf(s, "%16.s %16u %16u\n", client->debug_name,
				   client->pid, size);
			mutex_unlock(&client->lock);
			continue;
		}
		mutex_unlock(&client->lock);

		if (client->task) {
			char task_comm[TASK_COMM_LEN];

			get_task_comm(task_comm, client->task);
			seq_printf(s, "%16.s %16u %16u\n", task_comm,
				   client->pid, size);
		} else {
			seq_printf(s, "%16.s %16u %16u\n", client->name,
				   client->pid, size);
		}
	}
	up_read(&dev->lock);
	seq_printf(s, "----------------------------------------------------\n");
	seq_printf(s, "orphaned allocations (info is from last known client):"
		   "\n");
	mutex_lock(&dev->buffer_lock);
	for (n = rb_first(&dev->buffers); n; n = rb_next(n)) {
		struct ion_buffer *buffer = rb_entry(n, struct ion_buffer,
						     node);
		if (buffer->heap->id != heap->id)
			continue;
		total_size += buffer->size;
		if (!buffer->handle_count) {
			seq_printf(s, "%16.s %16u %16u %d %d\n", buffer->task_comm,
				   buffer->pid, buffer->size, buffer->kmap_cnt,
				   atomic_read(&buffer->ref.refcount));
			total_orphaned_size += buffer->size;
		}
	}
	mutex_unlock(&dev->buffer_lock);
	seq_printf(s, "----------------------------------------------------\n");
	seq_printf(s, "%16.s %16u\n", "total orphaned",
		   total_orphaned_size);
	seq_printf(s, "%16.s %16u\n", "total ", total_size);
	seq_printf(s, "----------------------------------------------------\n");

	if (heap->debug_show)
		heap->debug_show(heap, s, unused);

	ion_heap_print_debug(s, heap);
	return 0;
}

static int ion_debug_heap_open(struct inode *inode, struct file *file)
{
	return single_open(file, ion_debug_heap_show, inode->i_private);
}

static const struct file_operations debug_heap_fops = {
	.open = ion_debug_heap_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

#ifdef DEBUG_HEAP_SHRINKER
static int debug_shrink_set(void *data, u64 val)
{
        struct ion_heap *heap = data;
        struct shrink_control sc;
        int objs;

        sc.gfp_mask = -1;
        sc.nr_to_scan = 0;

        if (!val)
                return 0;

        objs = heap->shrinker.shrink(&heap->shrinker, &sc);
        sc.nr_to_scan = objs;

        heap->shrinker.shrink(&heap->shrinker, &sc);
        return 0;
}

static int debug_shrink_get(void *data, u64 *val)
{
        struct ion_heap *heap = data;
        struct shrink_control sc;
        int objs;

        sc.gfp_mask = -1;
        sc.nr_to_scan = 0;

        objs = heap->shrinker.shrink(&heap->shrinker, &sc);
        *val = objs;
        return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_shrink_fops, debug_shrink_get,
                        debug_shrink_set, "%llu\n");
#endif

void ion_device_add_heap(struct ion_device *dev, struct ion_heap *heap)
{
	struct dentry *debug_file;

	if (!heap->ops->allocate || !heap->ops->free || !heap->ops->map_dma ||
	    !heap->ops->unmap_dma)
		pr_err("%s: can not add heap with invalid ops struct.\n",
		       __func__);

	if (heap->flags & ION_HEAP_FLAG_DEFER_FREE)
		ion_heap_init_deferred_free(heap);

	heap->dev = dev;
	down_write(&dev->lock);
	plist_node_init(&heap->node, -heap->id);
	plist_add(&heap->node, &dev->heaps);
	debug_file = debugfs_create_file(heap->name, 0664,
					dev->heaps_debug_root, heap,
					&debug_heap_fops);

	if (!debug_file) {
		char buf[256], *path;
		path = dentry_path(dev->heaps_debug_root, buf, 256);
		pr_err("Failed to created heap debugfs at %s/%s\n",
			path, heap->name);
	}

#ifdef DEBUG_HEAP_SHRINKER
	if (heap->shrinker.shrink) {
		char debug_name[64];

		snprintf(debug_name, 64, "%s_shrink", heap->name);
		debug_file = debugfs_create_file(
			debug_name, 0644, dev->heaps_debug_root, heap,
			&debug_shrink_fops);
		if (!debug_file) {
			char buf[256], *path;
			path = dentry_path(dev->heaps_debug_root, buf, 256);
			pr_err("Failed to created heap shrinker debugfs at %s/%s\n",
				path, debug_name);
		}
	}
#endif
	up_write(&dev->lock);
}

int ion_secure_handle(struct ion_client *client, struct ion_handle *handle,
			int version, void *data, int flags)
{
	int ret = -EINVAL;
	struct ion_heap *heap;
	struct ion_buffer *buffer;

	mutex_lock(&client->lock);
	if (!ion_handle_validate(client, handle)) {
		WARN(1, "%s: invalid handle passed to secure.\n", __func__);
		goto out_unlock;
	}

	buffer = handle->buffer;
	heap = buffer->heap;

	if (!ion_heap_allow_handle_secure(heap->type)) {
		pr_err("%s: cannot secure buffer from non secure heap\n",
			__func__);
		goto out_unlock;
	}

	BUG_ON(!buffer->heap->ops->secure_buffer);
	ret = buffer->heap->ops->secure_buffer(buffer, version, data, flags);

out_unlock:
	mutex_unlock(&client->lock);
	return ret;
}

int ion_unsecure_handle(struct ion_client *client, struct ion_handle *handle)
{
	int ret = -EINVAL;
	struct ion_heap *heap;
	struct ion_buffer *buffer;

	mutex_lock(&client->lock);
	if (!ion_handle_validate(client, handle)) {
		WARN(1, "%s: invalid handle passed to secure.\n", __func__);
		goto out_unlock;
	}

	buffer = handle->buffer;
	heap = buffer->heap;

	if (!ion_heap_allow_handle_secure(heap->type)) {
		pr_err("%s: cannot secure buffer from non secure heap\n",
			__func__);
		goto out_unlock;
	}

	BUG_ON(!buffer->heap->ops->unsecure_buffer);
	ret = buffer->heap->ops->unsecure_buffer(buffer, 0);

out_unlock:
	mutex_unlock(&client->lock);
	return ret;
}

int ion_secure_heap(struct ion_device *dev, int heap_id, int version,
			void *data)
{
	int ret_val = 0;
	struct ion_heap *heap;

	down_write(&dev->lock);
	plist_for_each_entry(heap, &dev->heaps, node) {
		if (!ion_heap_allow_heap_secure(heap->type))
			continue;
		if (ION_HEAP(heap->id) != heap_id)
			continue;
		if (heap->ops->secure_heap)
			ret_val = heap->ops->secure_heap(heap, version, data);
		else
			ret_val = -EINVAL;
		break;
	}
	up_write(&dev->lock);
	return ret_val;
}
EXPORT_SYMBOL(ion_secure_heap);

int ion_walk_heaps(struct ion_client *client, int heap_id, void *data,
			int (*f)(struct ion_heap *heap, void *data))
{
	int ret_val = -EINVAL;
	struct ion_heap *heap;
	struct ion_device *dev = client->dev;
	down_write(&dev->lock);
	plist_for_each_entry(heap, &dev->heaps, node) {
		if (ION_HEAP(heap->id) != heap_id)
			continue;
		ret_val = f(heap, data);
		break;
	}
	up_write(&dev->lock);
	return ret_val;
}
EXPORT_SYMBOL(ion_walk_heaps);

int ion_unsecure_heap(struct ion_device *dev, int heap_id, int version,
			void *data)
{
	int ret_val = 0;
	struct ion_heap *heap;

	down_write(&dev->lock);
	plist_for_each_entry(heap, &dev->heaps, node) {
		if (!ion_heap_allow_heap_secure(heap->type))
			continue;
		if (ION_HEAP(heap->id) != heap_id)
			continue;
		if (heap->ops->secure_heap)
			ret_val = heap->ops->unsecure_heap(heap, version, data);
		else
			ret_val = -EINVAL;
		break;
	}
	up_write(&dev->lock);
	return ret_val;
}
EXPORT_SYMBOL(ion_unsecure_heap);

struct ion_device *ion_device_create(long (*custom_ioctl)
				     (struct ion_client *client,
				      unsigned int cmd,
				      unsigned long arg))
{
	struct ion_device *idev;
	int ret;

	idev = kzalloc(sizeof(struct ion_device), GFP_KERNEL);
	if (!idev)
		return ERR_PTR(-ENOMEM);

	idev->dev.minor = MISC_DYNAMIC_MINOR;
	idev->dev.name = "ion";
	idev->dev.fops = &ion_fops;
	idev->dev.parent = NULL;
	ret = misc_register(&idev->dev);
	if (ret) {
		pr_err("ion: failed to register misc device.\n");
		return ERR_PTR(ret);
	}

	idev->debug_root = debugfs_create_dir("ion", NULL);
	if (!idev->debug_root) {
		pr_err("ion: failed to create debugfs root directory.\n");
		goto debugfs_done;
	}
	idev->heaps_debug_root = debugfs_create_dir("heaps", idev->debug_root);
	if (!idev->heaps_debug_root) {
		pr_err("ion: failed to create debugfs heaps directory.\n");
		goto debugfs_done;
	}
	idev->clients_debug_root = debugfs_create_dir("clients",
						idev->debug_root);
	if (!idev->clients_debug_root)
		pr_err("ion: failed to create debugfs clients directory.\n");

debugfs_done:

	idev->custom_ioctl = custom_ioctl;
	idev->buffers = RB_ROOT;
	mutex_init(&idev->buffer_lock);
	init_rwsem(&idev->lock);
	plist_head_init(&idev->heaps);
	idev->clients = RB_ROOT;
	return idev;
}

void ion_device_destroy(struct ion_device *dev)
{
	misc_deregister(&dev->dev);
	debugfs_remove_recursive(dev->debug_root);
	
	kfree(dev);
}

void __init ion_reserve(struct ion_platform_data *data)
{
	int i;

	for (i = 0; i < data->nr; i++) {
		if (data->heaps[i].size == 0)
			continue;

		if (data->heaps[i].base == 0) {
			phys_addr_t paddr;
			paddr = memblock_alloc_base(data->heaps[i].size,
						    data->heaps[i].align,
						    MEMBLOCK_ALLOC_ANYWHERE);
			if (!paddr) {
				pr_err("%s: error allocating memblock for "
				       "heap %d\n",
					__func__, i);
				continue;
			}
			data->heaps[i].base = paddr;
		} else {
			int ret = memblock_reserve(data->heaps[i].base,
					       data->heaps[i].size);
			if (ret)
				pr_err("memblock reserve of %x@%pa failed\n",
				       data->heaps[i].size,
				       &data->heaps[i].base);
		}
		pr_info("%s: %s reserved base %pa size %d\n", __func__,
			data->heaps[i].name,
			&data->heaps[i].base,
			data->heaps[i].size);
	}
}
