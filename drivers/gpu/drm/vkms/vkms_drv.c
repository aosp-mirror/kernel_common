// SPDX-License-Identifier: GPL-2.0+

/**
 * DOC: vkms (Virtual Kernel Modesetting)
 *
 * VKMS is a software-only model of a KMS driver that is useful for testing
 * and for running X (or similar) on headless machines. VKMS aims to enable
 * a virtual display with no need of a hardware display capability, releasing
 * the GPU in DRM API tests.
 */

#include <linux/configfs.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <drm/drm_device.h>
#include <drm/drm_gem.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_generic.h>
#include <drm/drm_file.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_managed.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_vblank.h>

#include "vkms_drv.h"

#include <drm/drm_print.h>
#include <drm/drm_debugfs.h>

#define DRIVER_NAME	"vkms"
#define DRIVER_DESC	"Virtual Kernel Mode Setting"
#define DRIVER_DATE	"20180514"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

static bool enable_default_device = true;
module_param_named(enable_default_device, enable_default_device, bool, 0444);
MODULE_PARM_DESC(enable_default_device,
		 "Enable/Disable creating the default device");

static bool enable_cursor = true;
module_param_named(enable_cursor, enable_cursor, bool, 0444);
MODULE_PARM_DESC(enable_cursor,
		 "Enable/Disable cursor support for the default device");

static bool enable_writeback = true;
module_param_named(enable_writeback, enable_writeback, bool, 0444);
MODULE_PARM_DESC(
	enable_writeback,
	"Enable/Disable writeback connector support for the default device");

static bool enable_overlay;
module_param_named(enable_overlay, enable_overlay, bool, 0444);
MODULE_PARM_DESC(enable_overlay,
		 "Enable/Disable overlay support for the default device");

DEFINE_DRM_GEM_FOPS(vkms_driver_fops);

static void vkms_release(struct drm_device *dev)
{
	struct vkms_device *vkms = drm_device_to_vkms_device(dev);

	for (int i = 0; i < vkms->output.num_crtcs; i++)
		destroy_workqueue(vkms->output.crtcs[i].composer_workq);
}

static void vkms_atomic_commit_tail(struct drm_atomic_state *old_state)
{
	struct drm_device *dev = old_state->dev;
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state;
	int i;

	drm_atomic_helper_commit_modeset_disables(dev, old_state);

	drm_atomic_helper_commit_planes(dev, old_state, 0);

	drm_atomic_helper_commit_modeset_enables(dev, old_state);

	drm_atomic_helper_fake_vblank(old_state);

	drm_atomic_helper_commit_hw_done(old_state);

	drm_atomic_helper_wait_for_flip_done(dev, old_state);

	for_each_old_crtc_in_state(old_state, crtc, old_crtc_state, i) {
		struct vkms_crtc_state *vkms_state =
			to_vkms_crtc_state(old_crtc_state);

		flush_work(&vkms_state->composer_work);
	}

	drm_atomic_helper_cleanup_planes(dev, old_state);
}

static int vkms_config_show(struct seq_file *m, void *data)
{
	struct drm_debugfs_entry *entry = m->private;
	struct drm_device *dev = entry->dev;
	struct vkms_device *vkmsdev = drm_device_to_vkms_device(dev);

	seq_printf(m, "default_device=%d\n", enable_default_device);
	seq_printf(m, "writeback=%d\n", vkmsdev->config.writeback);
	seq_printf(m, "cursor=%d\n", vkmsdev->config.cursor);
	seq_printf(m, "overlay=%d\n", vkmsdev->config.overlay);

	return 0;
}

static const struct drm_debugfs_info vkms_config_debugfs_list[] = {
	{ "vkms_config", vkms_config_show, 0 },
};

static const struct drm_driver vkms_driver = {
	.driver_features	= DRIVER_MODESET | DRIVER_ATOMIC | DRIVER_GEM,
	.release		= vkms_release,
	.fops			= &vkms_driver_fops,
	DRM_GEM_SHMEM_DRIVER_OPS,

	.name			= DRIVER_NAME,
	.desc			= DRIVER_DESC,
	.date			= DRIVER_DATE,
	.major			= DRIVER_MAJOR,
	.minor			= DRIVER_MINOR,
};

static int vkms_atomic_check(struct drm_device *dev, struct drm_atomic_state *state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *new_crtc_state;
	int i;

	for_each_new_crtc_in_state(state, crtc, new_crtc_state, i) {
		if (!new_crtc_state->gamma_lut || !new_crtc_state->color_mgmt_changed)
			continue;

		if (new_crtc_state->gamma_lut->length / sizeof(struct drm_color_lut *)
		    > VKMS_LUT_SIZE)
			return -EINVAL;
	}

	return drm_atomic_helper_check(dev, state);
}

static const struct drm_mode_config_funcs vkms_mode_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = vkms_atomic_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static const struct drm_mode_config_helper_funcs vkms_mode_config_helpers = {
	.atomic_commit_tail = vkms_atomic_commit_tail,
};

static int vkms_modeset_init(struct vkms_device *vkmsdev)
{
	struct drm_device *dev = &vkmsdev->drm;
	int ret;

	ret = drmm_mode_config_init(dev);
	if (ret)
		return ret;

	dev->mode_config.funcs = &vkms_mode_funcs;
	dev->mode_config.min_width = XRES_MIN;
	dev->mode_config.min_height = YRES_MIN;
	dev->mode_config.max_width = XRES_MAX;
	dev->mode_config.max_height = YRES_MAX;
	dev->mode_config.cursor_width = 512;
	dev->mode_config.cursor_height = 512;
	/* FIXME: There's a confusion between bpp and depth between this and
	 * fbdev helpers. We have to go with 0, meaning "pick the default",
	 * which ix XRGB8888 in all cases.
	 */
	dev->mode_config.preferred_depth = 0;
	dev->mode_config.helper_private = &vkms_mode_config_helpers;

	return vkmsdev->configfs ? vkms_output_init(vkmsdev) :
				   vkms_output_init_default(vkmsdev);
}

static int vkms_platform_probe(struct platform_device *pdev)
{
	int ret;
	struct vkms_device_setup *vkms_device_setup = pdev->dev.platform_data;
	struct vkms_device *vkms_device;
	void *grp;

	grp = devres_open_group(&pdev->dev, NULL, GFP_KERNEL);
	if (!grp) {
		DRM_ERROR("Could not open devres group\n");
		return -ENOMEM;
	}

	vkms_device = devm_drm_dev_alloc(&pdev->dev, &vkms_driver,
					 struct vkms_device, drm);
	if (IS_ERR(vkms_device)) {
		ret = PTR_ERR(vkms_device);
		goto out_release_group;
	}

	vkms_device->platform = pdev;
	vkms_device->config.cursor = enable_cursor;
	vkms_device->config.writeback = enable_writeback;
	vkms_device->config.overlay = enable_overlay;
	vkms_device->configfs = vkms_device_setup->configfs;

	ret = dma_coerce_mask_and_coherent(vkms_device->drm.dev,
					   DMA_BIT_MASK(64));
	if (ret) {
		DRM_ERROR("Could not initialize DMA support\n");
		goto out_release_group;
	}

	ret = vkms_modeset_init(vkms_device);
	if (ret) {
		DRM_ERROR("Unable to initialize modesetting\n");
		goto out_release_group;
	}

	ret = drm_vblank_init(&vkms_device->drm, vkms_device->output.num_crtcs);
	if (ret) {
		DRM_ERROR("Failed to vblank\n");
		goto out_release_group;
	}

	drm_debugfs_add_files(&vkms_device->drm, vkms_config_debugfs_list,
			      ARRAY_SIZE(vkms_config_debugfs_list));

	ret = drm_dev_register(&vkms_device->drm, 0);
	if (ret) {
		DRM_ERROR("Unable to register device with id %d\n", pdev->id);
		goto out_release_group;
	}

	drm_fbdev_generic_setup(&vkms_device->drm, 0);
	platform_set_drvdata(pdev, vkms_device);
	devres_close_group(&pdev->dev, grp);

	return 0;

out_release_group:
	devres_release_group(&pdev->dev, grp);
	return ret;
}

static int vkms_platform_remove(struct platform_device *pdev)
{
	struct vkms_device *vkms_device;

	vkms_device = platform_get_drvdata(pdev);
	if (!vkms_device)
		return 0;

	drm_dev_unregister(&vkms_device->drm);
	drm_atomic_helper_shutdown(&vkms_device->drm);
	return 0;
}

static struct platform_driver vkms_platform_driver = {
	.probe = vkms_platform_probe,
	.remove = vkms_platform_remove,
	.driver.name = DRIVER_NAME,
};

struct vkms_device *vkms_add_device(struct vkms_configfs *configfs)
{
	struct device *dev = NULL;
	struct platform_device *pdev;
	int max_id = 1;
	struct vkms_device_setup vkms_device_setup = {
		.configfs = configfs,
	};

	while ((dev = platform_find_device_by_driver(
			dev, &vkms_platform_driver.driver))) {
		pdev = to_platform_device(dev);
		max_id = max(max_id, pdev->id);
		put_device(dev);
	}

	pdev = platform_device_register_data(NULL, DRIVER_NAME, max_id + 1,
					     &vkms_device_setup,
					     sizeof(vkms_device_setup));
	if (IS_ERR(pdev)) {
		DRM_ERROR("Unable to register vkms device'\n");
		return ERR_PTR(PTR_ERR(pdev));
	}

	return platform_get_drvdata(pdev);
}

void vkms_remove_device(struct vkms_device *vkms_device)
{
	platform_device_unregister(vkms_device->platform);
}

static int __init vkms_init(void)
{
	int ret;
	struct platform_device *default_pdev = NULL;

	ret = platform_driver_register(&vkms_platform_driver);
	if (ret) {
		DRM_ERROR("Unable to register platform driver\n");
		return ret;
	}

	if (enable_default_device) {
		struct vkms_device_setup vkms_device_setup = {
			.configfs = NULL,
		};

		default_pdev = platform_device_register_data(
			NULL, DRIVER_NAME, 0, &vkms_device_setup,
			sizeof(vkms_device_setup));
		if (IS_ERR(default_pdev)) {
			DRM_ERROR("Unable to register default vkms device\n");
			platform_driver_unregister(&vkms_platform_driver);
			return PTR_ERR(default_pdev);
		}
	}

	ret = vkms_init_configfs();
	if (ret) {
		DRM_ERROR("Unable to initialize configfs\n");
		if (default_pdev)
			platform_device_unregister(default_pdev);

		platform_driver_unregister(&vkms_platform_driver);
	}

	return 0;
}

static void __exit vkms_exit(void)
{
	struct device *dev;

	vkms_unregister_configfs();

	while ((dev = platform_find_device_by_driver(
			NULL, &vkms_platform_driver.driver))) {
		// platform_find_device_by_driver increments the refcount. Drop
		// it so we don't leak memory.
		put_device(dev);
		platform_device_unregister(to_platform_device(dev));
	}

	platform_driver_unregister(&vkms_platform_driver);
}

module_init(vkms_init);
module_exit(vkms_exit);

MODULE_AUTHOR("Haneen Mohammed <hamohammed.sa@gmail.com>");
MODULE_AUTHOR("Rodrigo Siqueira <rodrigosiqueiramelo@gmail.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
