// SPDX-License-Identifier: GPL-2.0+

#include <drm/drm_print.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder.h>
#include <drm/drm_plane.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <linux/printk.h>

#include "vkms_drv.h"

static const struct drm_connector_funcs vkms_connector_funcs = {
	.detect = vkms_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct vkms_config_connector *
find_config_for_connector(struct drm_connector *connector)
{
	struct vkms_device *vkms = drm_device_to_vkms_device(connector->dev);
	struct vkms_configfs *configfs = vkms->configfs;
	struct config_item *item;

	if (!configfs) {
		pr_info("Default connector has no configfs entry");
		return NULL;
	}

	list_for_each_entry(item, &configfs->connectors_group.cg_children,
			    ci_entry) {
		struct vkms_config_connector *config_connector =
			item_to_config_connector(item);
		if (config_connector->connector == connector)
			return config_connector;
	}

	pr_warn("Could not find config to match connector %s, but configfs was initialized",
		connector->name);

	return NULL;
}

enum drm_connector_status vkms_connector_detect(struct drm_connector *connector,
						bool force)
{
	enum drm_connector_status status = connector_status_connected;
	const struct vkms_config_connector *config_connector =
		find_config_for_connector(connector);

	if (!config_connector)
		return connector_status_connected;

	if (!config_connector->connected)
		status = connector_status_disconnected;

	return status;
}

static const struct drm_encoder_funcs vkms_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int vkms_conn_get_modes(struct drm_connector *connector)
{
	int count;

	count = drm_add_modes_noedid(connector, XRES_MAX, YRES_MAX);
	drm_set_preferred_mode(connector, XRES_DEF, YRES_DEF);

	return count;
}

static const struct drm_connector_helper_funcs vkms_conn_helper_funcs = {
	.get_modes = vkms_conn_get_modes,
};

static struct drm_connector *
vkms_connector_init(struct vkms_device *vkms_device)
{
	struct drm_connector *connector;
	int ret;

	if (vkms_device->output.num_connectors >= VKMS_MAX_OUTPUT_OBJECTS)
		return ERR_PTR(-ENOMEM);

	connector = &vkms_device->output
			     .connectors[vkms_device->output.num_connectors++];
	ret = drm_connector_init(&vkms_device->drm, connector,
				 &vkms_connector_funcs,
				 DRM_MODE_CONNECTOR_VIRTUAL);
	if (ret) {
		memset(connector, 0, sizeof(*connector));
		vkms_device->output.num_connectors -= 1;
		return ERR_PTR(ret);
	}

	drm_connector_helper_add(connector, &vkms_conn_helper_funcs);

	return connector;
}

static struct drm_encoder *vkms_encoder_init(struct vkms_device *vkms_device,
					     char *name)
{
	struct drm_encoder *encoder;
	int ret;

	if (vkms_device->output.num_encoders >= VKMS_MAX_OUTPUT_OBJECTS)
		return ERR_PTR(-ENOMEM);

	encoder = &vkms_device->output
			   .encoders[vkms_device->output.num_encoders++];
	ret = drm_encoder_init(&vkms_device->drm, encoder, &vkms_encoder_funcs,
			       DRM_MODE_ENCODER_VIRTUAL, name);
	if (ret) {
		memset(encoder, 0, sizeof(*encoder));
		vkms_device->output.num_encoders -= 1;
		return ERR_PTR(ret);
	}
	return encoder;
}

int vkms_output_init_default(struct vkms_device *vkmsdev)
{
	struct drm_device *dev = &vkmsdev->drm;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	struct vkms_crtc *vkms_crtc;
	struct vkms_plane *primary, *cursor = NULL;
	int ret;
	int writeback;
	unsigned int n;

	primary = vkms_plane_init(vkmsdev, DRM_PLANE_TYPE_PRIMARY,
				  "default-primary-plane");
	if (IS_ERR(primary))
		return PTR_ERR(primary);

	if (vkmsdev->config.overlay) {
		for (n = 0; n < NUM_OVERLAY_PLANES; n++) {
			struct vkms_plane *overlay =
				vkms_plane_init(vkmsdev, DRM_PLANE_TYPE_OVERLAY,
						"default-overlay-plane-%d", n);
			if (IS_ERR(overlay))
				return PTR_ERR(overlay);
		}
	}

	if (vkmsdev->config.cursor) {
		cursor = vkms_plane_init(vkmsdev, DRM_PLANE_TYPE_CURSOR,
					 "default-cursor-plane");
		if (IS_ERR(cursor))
			return PTR_ERR(cursor);
	}

	vkms_crtc = vkms_crtc_init(vkmsdev, &primary->base,
				   cursor ? &cursor->base : NULL,
				   "crtc-default");
	if (IS_ERR(vkms_crtc)) {
		DRM_ERROR("Failed to init crtc\n");
		return PTR_ERR(vkms_crtc);
	}

	for (int i = 0; i < vkmsdev->output.num_planes; i++) {
		vkmsdev->output.planes[i].base.possible_crtcs |=
			drm_crtc_mask(&vkms_crtc->base);
	}

	connector = vkms_connector_init(vkmsdev);
	if (IS_ERR(connector)) {
		DRM_ERROR("Failed to init connector\n");
		return PTR_ERR(connector);
	}

	encoder = vkms_encoder_init(vkmsdev, "encoder-default");
	if (IS_ERR(encoder)) {
		DRM_ERROR("Failed to init encoder\n");
		return PTR_ERR(encoder);
	}
	encoder->possible_crtcs |= drm_crtc_mask(&vkms_crtc->base);

	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret) {
		DRM_ERROR("Failed to attach connector to encoder\n");
		return ret;
	}

	if (vkmsdev->config.writeback) {
		writeback = vkms_enable_writeback_connector(vkmsdev, vkms_crtc);
		if (writeback)
			DRM_ERROR("Failed to init writeback connector\n");
	}

	drm_mode_config_reset(dev);

	return 0;
}

static bool is_object_linked(struct vkms_config_links *links, unsigned long idx)
{
	return links->linked_object_bitmap & (1 << idx);
}

/**
* validate_vkms_configfs_no_dangling_objects - warn on unused objects in vkms
* configfs.
* @vkmsdev: vkms device
*
* This gives slightly more visible warning messaging to the user before the drm
* system finds the configuration invalid and prints it's debug information.  In
* this case the user may have accidentally not included some links, or the user
* could be testing this faulty configuration.
*/
static void
validate_vkms_configfs_no_dangling_objects(struct vkms_device *vkmsdev)
{
	struct vkms_configfs *configfs = vkmsdev->configfs;
	struct config_item *item;

	// 1. Planes
	list_for_each_entry(item, &configfs->planes_group.cg_children,
			    ci_entry) {
		struct vkms_config_plane *config_plane =
			item_to_config_plane(item);
		if (config_plane->possible_crtcs.linked_object_bitmap == 0)
			DRM_WARN(
				"Vkms configfs created plane %s has no linked crtcs",
				item->ci_name);
	}

	// 2. connectors
	list_for_each_entry(item, &configfs->connectors_group.cg_children,
			    ci_entry) {
		struct vkms_config_connector *config_connector =
			item_to_config_connector(item);
		if (config_connector->possible_encoders.linked_object_bitmap ==
		    0) {
			DRM_WARN(
				"Vkms configfs created connector %s has no linked encoders",
				item->ci_name);
		}
	}

	// 3. encoders
	list_for_each_entry(item, &configfs->encoders_group.cg_children,
			    ci_entry) {
		struct vkms_config_encoder *config_encoder =
			item_to_config_encoder(item);
		if (config_encoder->possible_crtcs.linked_object_bitmap == 0) {
			DRM_WARN(
				"Vkms configfs created encoder %s has no linked crtcs",
				item->ci_name);
		}
	}

	// 4. crtcs only require a primary plane to function, this is checked during
	// output initialization and returns an error.
}

int vkms_output_init(struct vkms_device *vkmsdev)
{
	struct drm_device *dev = &vkmsdev->drm;
	struct vkms_configfs *configfs = vkmsdev->configfs;
	struct vkms_output *output = &vkmsdev->output;
	struct plane_map {
		struct vkms_config_plane *config_plane;
		struct vkms_plane *plane;
	} plane_map[VKMS_MAX_PLANES] = { 0 };
	struct encoder_map {
		struct vkms_config_encoder *config_encoder;
		struct drm_encoder *encoder;
	} encoder_map[VKMS_MAX_OUTPUT_OBJECTS] = { 0 };
	struct config_item *item;
	int map_idx = 0;

	// Ensure configfs has no unused objects, and warn if so.
	validate_vkms_configfs_no_dangling_objects(vkmsdev);

	list_for_each_entry(item, &configfs->planes_group.cg_children,
			    ci_entry) {
		struct vkms_config_plane *config_plane =
			item_to_config_plane(item);
		struct vkms_plane *plane = vkms_plane_init(
			vkmsdev, config_plane->type, item->ci_name);

		if (IS_ERR(plane)) {
			DRM_ERROR("Unable to init plane from config: %s",
				  item->ci_name);
			return PTR_ERR(plane);
		}

		plane_map[map_idx].config_plane = config_plane;
		plane_map[map_idx].plane = plane;
		map_idx += 1;
	}

	map_idx = 0;
	list_for_each_entry(item, &configfs->encoders_group.cg_children,
			    ci_entry) {
		struct vkms_config_encoder *config_encoder =
			item_to_config_encoder(item);
		struct drm_encoder *encoder =
			vkms_encoder_init(vkmsdev, item->ci_name);

		if (IS_ERR(encoder)) {
			DRM_ERROR("Failed to init config encoder: %s",
				  item->ci_name);
			return PTR_ERR(encoder);
		}
		encoder_map[map_idx].config_encoder = config_encoder;
		encoder_map[map_idx].encoder = encoder;
		map_idx += 1;
	}

	list_for_each_entry(item, &configfs->connectors_group.cg_children,
			    ci_entry) {
		struct vkms_config_connector *config_connector =
			item_to_config_connector(item);
		struct drm_connector *connector = vkms_connector_init(vkmsdev);
		if (IS_ERR(connector)) {
			DRM_ERROR("Failed to init connector from config: %s",
				  item->ci_name);
			return PTR_ERR(connector);
		}
		config_connector->connector = connector;

		for (int j = 0; j < output->num_encoders; j++) {
			struct encoder_map *encoder = &encoder_map[j];

			if (is_object_linked(
				    &config_connector->possible_encoders,
				    encoder->config_encoder
					    ->encoder_config_idx)) {
				drm_connector_attach_encoder(connector,
							     encoder->encoder);
			}
		}
	}

	list_for_each_entry(item, &configfs->crtcs_group.cg_children,
			    ci_entry) {
		struct vkms_config_crtc *config_crtc =
			item_to_config_crtc(item);
		struct vkms_crtc *vkms_crtc;
		struct drm_plane *primary = NULL, *cursor = NULL;

		for (int j = 0; j < output->num_planes; j++) {
			struct plane_map *plane_entry = &plane_map[j];
			struct drm_plane *plane = &plane_entry->plane->base;

			if (!is_object_linked(
				    &plane_entry->config_plane->possible_crtcs,
				    config_crtc->crtc_config_idx)) {
				continue;
			}

			if (plane->type == DRM_PLANE_TYPE_PRIMARY) {
				if (primary) {
					DRM_WARN(
						"Too many primary planes found for crtc %s.",
						item->ci_name);
					return -EINVAL;
				}
				primary = plane;
			} else if (plane->type == DRM_PLANE_TYPE_CURSOR) {
				if (cursor) {
					DRM_WARN(
						"Too many cursor planes found for crtc %s.",
						item->ci_name);
					return -EINVAL;
				}
				cursor = plane;
			}
		}

		if (!primary) {
			DRM_WARN("No primary plane configured for crtc %s",
				 item->ci_name);
			return -EINVAL;
		}

		vkms_crtc =
			vkms_crtc_init(vkmsdev, primary, cursor, item->ci_name);
		if (IS_ERR(vkms_crtc)) {
			DRM_WARN("Unable to init crtc from config: %s",
				 item->ci_name);
			return PTR_ERR(vkms_crtc);
		}

		for (int j = 0; j < output->num_planes; j++) {
			struct plane_map *plane_entry = &plane_map[j];

			if (!plane_entry->plane)
				break;

			if (is_object_linked(
				    &plane_entry->config_plane->possible_crtcs,
				    config_crtc->crtc_config_idx)) {
				plane_entry->plane->base.possible_crtcs |=
					drm_crtc_mask(&vkms_crtc->base);
			}
		}

		for (int j = 0; j < output->num_encoders; j++) {
			struct encoder_map *encoder_entry = &encoder_map[j];

			if (is_object_linked(&encoder_entry->config_encoder
						      ->possible_crtcs,
					     config_crtc->crtc_config_idx)) {
				encoder_entry->encoder->possible_crtcs |=
					drm_crtc_mask(&vkms_crtc->base);
			}
		}

		if (vkmsdev->config.writeback) {
			int ret = vkms_enable_writeback_connector(vkmsdev,
								  vkms_crtc);
			if (ret)
				DRM_WARN(
					"Failed to init writeback connector for config crtc: %s. Error code %d",
					item->ci_name, ret);
		}
	}

	drm_mode_config_reset(dev);

	return 0;
}
