// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2012 Red Hat
 * Copyright (c) 2015 - 2020 DisplayLink (UK) Ltd.
 *
 * Based on parts on udlfb.c:
 * Copyright (C) 2009 its respective authors
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <drm/drm_ioctl.h>
#include <drm/drm_file.h>
#include <drm/drm_drv.h>
#include <drm/drm_vblank.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_atomic_helper.h>
#include <uapi/drm/evdi_drm.h>
#include "evdi_drm_drv.h"
#include "evdi_platform_drv.h"
#include "evdi_cursor.h"
#include "evdi_debug.h"

#define EVDI_DRM_UNLOCKED DRM_UNLOCKED

static struct drm_driver driver;

struct drm_ioctl_desc evdi_painter_ioctls[] = {
	DRM_IOCTL_DEF_DRV(EVDI_CONNECT, evdi_painter_connect_ioctl, EVDI_DRM_UNLOCKED),
	DRM_IOCTL_DEF_DRV(EVDI_REQUEST_UPDATE, evdi_painter_request_update_ioctl, EVDI_DRM_UNLOCKED),
	DRM_IOCTL_DEF_DRV(EVDI_GRABPIX, evdi_painter_grabpix_ioctl, EVDI_DRM_UNLOCKED),
	DRM_IOCTL_DEF_DRV(EVDI_DDCCI_RESPONSE, evdi_painter_ddcci_response_ioctl, EVDI_DRM_UNLOCKED),
	DRM_IOCTL_DEF_DRV(EVDI_ENABLE_CURSOR_EVENTS, evdi_painter_enable_cursor_events_ioctl, EVDI_DRM_UNLOCKED),
};


static const struct file_operations evdi_driver_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.mmap = evdi_drm_gem_mmap,
	.poll = drm_poll,
	.read = drm_read,
	.unlocked_ioctl = drm_ioctl,
	.release = drm_release,

#ifdef CONFIG_COMPAT
	.compat_ioctl = evdi_compat_ioctl,
#endif

	.llseek = noop_llseek,
};


static struct drm_driver driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,

	.open = evdi_driver_open,
	.postclose = evdi_driver_postclose,

	/* gem hooks */


	.dumb_create = evdi_dumb_create,
	.dumb_map_offset = evdi_gem_mmap,

	.ioctls = evdi_painter_ioctls,
	.num_ioctls = ARRAY_SIZE(evdi_painter_ioctls),

	.fops = &evdi_driver_fops,

	.gem_prime_import = drm_gem_prime_import,
	.gem_prime_import_sg_table = evdi_prime_import_sg_table,

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCH,
};

static void evdi_drm_device_release_cb(__always_unused struct drm_device *dev,
				       __always_unused void *ptr)
{
	struct evdi_device *evdi = dev->dev_private;

	evdi_cursor_free(evdi->cursor);
	evdi_painter_cleanup(evdi->painter);
	kfree(evdi);
	dev->dev_private = NULL;
	EVDI_INFO("Evdi drm_device removed.\n");
}

static int evdi_drm_device_init(struct drm_device *dev)
{
	struct evdi_device *evdi;
	int ret;

	EVDI_CHECKPT();
	evdi = kzalloc(sizeof(struct evdi_device), GFP_KERNEL);
	if (!evdi)
		return -ENOMEM;

	evdi->ddev = dev;
	evdi->dev_index = dev->primary->index;
	evdi->cursor_events_enabled = false;
	dev->dev_private = evdi;
	ret = evdi_painter_init(evdi);
	if (ret)
		goto err_free;
	ret =  evdi_cursor_init(&evdi->cursor);
	if (ret)
		goto err_free;

	evdi_modeset_init(dev);
#ifdef CONFIG_FB
	ret = evdi_fbdev_init(dev);
	if (ret)
		goto err_init;
#endif /* CONFIG_FB */

	ret = drm_vblank_init(dev, 1);
	if (ret)
		goto err_init;
	drm_kms_helper_poll_init(dev);

	ret = drmm_add_action_or_reset(dev, evdi_drm_device_release_cb, NULL);
	if (ret)
		goto err_init;

	return 0;

err_init:
#ifdef CONFIG_FB
	evdi_fbdev_cleanup(dev);
#endif /* CONFIG_FB */
err_free:
	EVDI_ERROR("Failed to setup drm device %d\n", ret);
	evdi_cursor_free(evdi->cursor);
	kfree(evdi->painter);
	kfree(evdi);
	dev->dev_private = NULL;
	return ret;
}

int evdi_driver_open(struct drm_device *dev, __always_unused struct drm_file *file)
{
	char buf[100];

	evdi_log_process(buf, sizeof(buf));
	EVDI_INFO("(card%d) Opened by %s\n", dev->primary->index, buf);
	return 0;
}

static void evdi_driver_close(struct drm_device *drm_dev, struct drm_file *file)
{
	struct evdi_device *evdi = drm_dev->dev_private;

	EVDI_CHECKPT();
	if (evdi)
		evdi_painter_close(evdi, file);
}

void evdi_driver_preclose(struct drm_device *drm_dev, struct drm_file *file)
{
	evdi_driver_close(drm_dev, file);
}

void evdi_driver_postclose(struct drm_device *dev, struct drm_file *file)
{
	char buf[100];

	evdi_log_process(buf, sizeof(buf));
	evdi_driver_close(dev, file);
	EVDI_INFO("(card%d) Closed by %s\n", dev->primary->index, buf);
}

struct drm_device *evdi_drm_device_create(struct device *parent)
{
	struct drm_device *dev = NULL;
	int ret;

	dev = drm_dev_alloc(&driver, parent);
	if (IS_ERR(dev))
		return dev;

	ret = evdi_drm_device_init(dev);
	if (ret)
		goto err_free;

	ret = drm_dev_register(dev, 0);
	if (ret)
		goto err_free;

	return dev;

err_free:
	drm_dev_put(dev);
	return ERR_PTR(ret);
}

static void evdi_drm_device_deinit(struct drm_device *dev)
{
	drm_kms_helper_poll_fini(dev);
#ifdef CONFIG_FB
	evdi_fbdev_unplug(dev);
	evdi_fbdev_cleanup(dev);
#endif /* CONFIG_FB */
	evdi_modeset_cleanup(dev);
	drm_atomic_helper_shutdown(dev);
}

int evdi_drm_device_remove(struct drm_device *dev)
{
	drm_dev_unplug(dev);
	evdi_drm_device_deinit(dev);
	drm_dev_put(dev);
	return 0;
}

