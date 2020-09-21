// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/ctype.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/trusty/smcall.h>
#include <linux/trusty/trusty.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>

#include "trusty-test.h"

struct trusty_test_state {
	struct device *dev;
	struct device *trusty_dev;
};

struct trusty_test_shmem_obj {
	struct list_head node;
	size_t page_count;
	struct page **pages;
	void *buf;
	struct sg_table sgt;
	trusty_shared_mem_id_t mem_id;
};

/*
 * Allocate a test object with @page_count number of pages, map it and add it to
 * @list.
 * For multi-page allocations, order the pages so they are not contiguous.
 */
static int trusty_test_alloc_obj(struct trusty_test_state *s,
				 size_t page_count,
				 struct list_head *list)
{
	size_t i;
	int ret = -ENOMEM;
	struct trusty_test_shmem_obj *obj;

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj)
		goto err_alloc_obj;
	obj->page_count = page_count;

	obj->pages = kmalloc_array(page_count, sizeof(*obj->pages), GFP_KERNEL);
	if (!obj->pages) {
		ret = -ENOMEM;
		dev_err(s->dev, "failed to allocate page array, count %zd\n",
			page_count);
		goto err_alloc_pages;
	}

	for (i = 0; i < page_count; i++) {
		obj->pages[i] = alloc_page(GFP_KERNEL);
		if (!obj->pages[i]) {
			ret = -ENOMEM;
			dev_err(s->dev, "failed to allocate page %zd/%zd\n",
				i, page_count);
			goto err_alloc_page;
		}
		if (i > 0 && obj->pages[i - 1] + 1 == obj->pages[i]) {
			/* swap adacent pages to increase fragmentation */
			swap(obj->pages[i - 1], obj->pages[i]);
		}
	}

	obj->buf = vmap(obj->pages, page_count, VM_MAP, PAGE_KERNEL);
	if (!obj->buf) {
		ret = -ENOMEM;
		dev_err(s->dev, "failed to map test buffer page count %zd\n",
			page_count);
		goto err_map_pages;
	}

	ret = sg_alloc_table_from_pages(&obj->sgt, obj->pages, page_count,
					0, page_count * PAGE_SIZE, GFP_KERNEL);
	if (ret) {
		dev_err(s->dev, "sg_alloc_table_from_pages failed: %d\n", ret);
		goto err_alloc_sgt;
	}
	list_add_tail(&obj->node, list);
	dev_dbg(s->dev, "buffer has %d page runs\n", obj->sgt.nents);
	return 0;

err_alloc_sgt:
	vunmap(obj->buf);
err_map_pages:
	for (i = page_count; i > 0; i--) {
		__free_page(obj->pages[i - 1]);
err_alloc_page:
		;
	}
	kfree(obj->pages);
err_alloc_pages:
	kfree(obj);
err_alloc_obj:
	return ret;
}

/* Unlink, unmap and free a test object and its pages */
static void trusty_test_free_obj(struct trusty_test_state *s,
				 struct trusty_test_shmem_obj *obj)
{
	size_t i;

	list_del(&obj->node);
	sg_free_table(&obj->sgt);
	vunmap(obj->buf);
	for (i = obj->page_count; i > 0; i--)
		__free_page(obj->pages[i - 1]);
	kfree(obj->pages);
	kfree(obj);
}

/*
 * Share all the pages of all the test object in &obj_list.
 * If sharing a test object fails, free it so that every test object that
 * remains in @obj_list has been shared when this function returns.
 * Return a error if any test object failed to be shared.
 */
static int trusty_test_share_objs(struct trusty_test_state *s,
				  struct list_head *obj_list, size_t size)
{
	int ret = 0;
	int tmpret;
	struct trusty_test_shmem_obj *obj;
	struct trusty_test_shmem_obj *next_obj;
	ktime_t t1;
	ktime_t t2;

	list_for_each_entry_safe(obj, next_obj, obj_list, node) {
		t1 = ktime_get();
		tmpret = trusty_share_memory(s->trusty_dev, &obj->mem_id,
					     obj->sgt.sgl, obj->sgt.nents,
					     PAGE_KERNEL);
		t2 = ktime_get();
		if (tmpret) {
			ret = tmpret;
			dev_err(s->dev,
				"trusty_share_memory failed: %d, size=%zd\n",
				ret, size);

			/*
			 * Free obj and continue, so we can revoke the
			 * whole list in trusty_test_reclaim_objs.
			 */
			trusty_test_free_obj(s, obj);
		}
		dev_dbg(s->dev, "share id=0x%llx, size=%zu took %lld ns\n",
			obj->mem_id, size,
			ktime_to_ns(ktime_sub(t2, t1)));
	}

	return ret;
}

/* Reclaim memory shared with trusty for all test objects in @obj_list. */
static int trusty_test_reclaim_objs(struct trusty_test_state *s,
				    struct list_head *obj_list, size_t size)
{
	int ret = 0;
	int tmpret;
	struct trusty_test_shmem_obj *obj;
	struct trusty_test_shmem_obj *next_obj;
	ktime_t t1;
	ktime_t t2;

	list_for_each_entry_safe(obj, next_obj, obj_list, node) {
		t1 = ktime_get();
		tmpret = trusty_reclaim_memory(s->trusty_dev, obj->mem_id,
					       obj->sgt.sgl, obj->sgt.nents);
		t2 = ktime_get();
		if (tmpret) {
			ret = tmpret;
			dev_err(s->dev,
				"trusty_reclaim_memory failed: %d, id=0x%llx\n",
				ret, obj->mem_id);

			/*
			 * It is not safe to free this memory if
			 * trusty_reclaim_memory fails. Leak it in that
			 * case.
			 */
			list_del(&obj->node);
		}
		dev_dbg(s->dev, "revoke id=0x%llx, size=%zu took %lld ns\n",
			obj->mem_id, size,
			ktime_to_ns(ktime_sub(t2, t1)));
	}

	return ret;
}

/*
 * Test a test object. First, initialize the memory, then make a std call into
 * trusty which will read it and return an error if the initialized value does
 * not match what it expects. If trusty reads the correct values, it will modify
 * the memory and return 0. This function then checks that it can read the
 * correct modified value.
 */
static int trusty_test_rw(struct trusty_test_state *s,
			  struct trusty_test_shmem_obj *obj)
{
	size_t size = obj->page_count * PAGE_SIZE;
	int ret;
	size_t i;
	u64 *buf = obj->buf;
	ktime_t t1;
	ktime_t t2;

	for (i = 0; i < size / sizeof(*buf); i++)
		buf[i] = i;

	t1 = ktime_get();
	ret = trusty_std_call32(s->trusty_dev, SMC_SC_TEST_SHARED_MEM_RW,
				(u32)(obj->mem_id), (u32)(obj->mem_id >> 32),
				size);
	t2 = ktime_get();
	if (ret < 0) {
		dev_err(s->dev,
			"trusty std call (SMC_SC_TEST_SHARED_MEM_RW) failed: %d 0x%llx\n",
			ret, obj->mem_id);
		return ret;
	}

	for (i = 0; i < size / sizeof(*buf); i++) {
		if (buf[i] != size - i) {
			dev_err(s->dev,
				"input mismatch at %zd, got 0x%llx instead of 0x%zx\n",
				i, buf[i], size - i);
			return -EIO;
		}
	}

	dev_dbg(s->dev, "rw id=0x%llx, size=%zu took %lld ns\n", obj->mem_id,
		size, ktime_to_ns(ktime_sub(t2, t1)));

	return 0;
}

/*
 * Run test on every test object in @obj_list. Repeat @repeat_access times.
 */
static int trusty_test_rw_objs(struct trusty_test_state *s,
			       struct list_head *obj_list,
			       size_t repeat_access)
{
	int ret;
	size_t i;
	struct trusty_test_shmem_obj *obj;

	for (i = 0; i < repeat_access; i++) {
		/*
		 * Repeat test in case the memory attributes don't match
		 * and either side see old data.
		 */
		list_for_each_entry(obj, obj_list, node) {
			ret = trusty_test_rw(s, obj);
			if (ret)
				return ret;
		}
	}

	return 0;
}

/*
 * Allocate @obj_count test object that each have @page_count pages. Share each
 * object @repeat_share times, each time running tests on every object
 * @repeat_access times.
 */
static int trusty_test_run(struct trusty_test_state *s, size_t page_count,
			   size_t obj_count, size_t repeat_share,
			   size_t repeat_access)
{
	int ret = 0;
	int tmpret;
	size_t i;
	size_t size = page_count * PAGE_SIZE;
	LIST_HEAD(obj_list);
	struct trusty_test_shmem_obj *obj;
	struct trusty_test_shmem_obj *next_obj;

	for (i = 0; i < obj_count && !ret; i++)
		ret = trusty_test_alloc_obj(s, page_count, &obj_list);

	for (i = 0; i < repeat_share && !ret; i++) {
		ret = trusty_test_share_objs(s, &obj_list, size);
		if (ret) {
			dev_err(s->dev,
				"trusty_share_memory failed: %d, i=%zd/%zd, size=%zd\n",
				ret, i, repeat_share, size);
		} else {
			ret = trusty_test_rw_objs(s, &obj_list, repeat_access);
			if (ret)
				dev_err(s->dev,
					"test failed: %d, i=%zd/%zd, size=%zd\n",
					ret, i, repeat_share, size);
		}
		tmpret = trusty_test_reclaim_objs(s, &obj_list, size);
		if (tmpret) {
			ret = tmpret;
			dev_err(s->dev,
				"trusty_reclaim_memory failed: %d, i=%zd/%zd\n",
				ret, i, repeat_share);
		}
	}

	list_for_each_entry_safe(obj, next_obj, &obj_list, node)
		trusty_test_free_obj(s, obj);

	dev_info(s->dev, "[ %s ] size %zd, obj_count %zd, repeat_share %zd, repeat_access %zd\n",
		 ret ? "FAILED" : "PASSED", size, obj_count, repeat_share,
		 repeat_access);

	return ret;
}

/*
 * Get an optional numeric argument from @buf, update @buf and return the value.
 * If @buf does not start with ",", return @default_val instead.
 */
static size_t trusty_test_get_arg(const char **buf, size_t default_val)
{
	char *buf_next;
	size_t ret;

	if (**buf != ',')
		return default_val;

	(*buf)++;
	ret = simple_strtoul(*buf, &buf_next, 0);
	if (buf_next == *buf)
		return default_val;

	*buf = buf_next;

	return ret;
}

/*
 * Run tests described by a string in this format:
 * <obj_size>,<obj_count=1>,<repeat_share=1>,<repeat_access=3>
 */
static ssize_t trusty_test_run_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct trusty_test_state *s = platform_get_drvdata(pdev);
	size_t size;
	size_t obj_count;
	size_t repeat_share;
	size_t repeat_access;
	int ret;
	char *buf_next;

	while (true) {
		while (isspace(*buf))
			buf++;
		size = simple_strtoul(buf, &buf_next, 0);
		if (buf_next == buf)
			return count;
		buf = buf_next;
		obj_count = trusty_test_get_arg(&buf, 1);
		repeat_share = trusty_test_get_arg(&buf, 1);
		repeat_access = trusty_test_get_arg(&buf, 3);

		ret = trusty_test_run(s, DIV_ROUND_UP(size, PAGE_SIZE),
				      obj_count, repeat_share, repeat_access);
		if (ret)
			return ret;
	}
}

static DEVICE_ATTR_WO(trusty_test_run);

static struct attribute *trusty_test_attrs[] = {
	&dev_attr_trusty_test_run.attr,
	NULL,
};
ATTRIBUTE_GROUPS(trusty_test);

static int trusty_test_probe(struct platform_device *pdev)
{
	struct trusty_test_state *s;
	int ret;

	ret = trusty_std_call32(pdev->dev.parent, SMC_SC_TEST_VERSION,
				TRUSTY_STDCALLTEST_API_VERSION, 0, 0);
	if (ret != TRUSTY_STDCALLTEST_API_VERSION)
		return -ENOENT;

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->dev = &pdev->dev;
	s->trusty_dev = s->dev->parent;

	platform_set_drvdata(pdev, s);

	return 0;
}

static int trusty_test_remove(struct platform_device *pdev)
{
	struct trusty_log_state *s = platform_get_drvdata(pdev);

	kfree(s);
	return 0;
}

static const struct of_device_id trusty_test_of_match[] = {
	{ .compatible = "android,trusty-test-v1", },
	{},
};

MODULE_DEVICE_TABLE(trusty, trusty_test_of_match);

static struct platform_driver trusty_test_driver = {
	.probe = trusty_test_probe,
	.remove = trusty_test_remove,
	.driver = {
		.name = "trusty-test",
		.of_match_table = trusty_test_of_match,
		.dev_groups = trusty_test_groups,
	},
};

module_platform_driver(trusty_test_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Trusty test driver");
