// SPDX-License-Identifier: GPL-2.0+

#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include "vkms_drv.h"

static const struct drm_connector_funcs vkms_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

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

static struct drm_encoder *vkms_encoder_init(struct vkms_device *vkms_device)
{
	struct drm_encoder *encoder;
	int ret;

	if (vkms_device->output.num_encoders >= VKMS_MAX_OUTPUT_OBJECTS)
		return ERR_PTR(-ENOMEM);

	encoder = &vkms_device->output
			   .encoders[vkms_device->output.num_encoders++];
	ret = drm_encoder_init(&vkms_device->drm, encoder, &vkms_encoder_funcs,
			       DRM_MODE_ENCODER_VIRTUAL, NULL);
	if (ret) {
		memset(encoder, 0, sizeof(*encoder));
		vkms_device->output.num_encoders -= 1;
		return ERR_PTR(ret);
	}
	return encoder;
}

int vkms_output_init_default(struct vkms_device *vkmsdev)
{
	struct vkms_output *output = &vkmsdev->output;
	struct drm_device *dev = &vkmsdev->drm;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	struct vkms_crtc *vkms_crtc;
	struct vkms_plane *primary, *cursor = NULL;
	int ret;
	int writeback;
	unsigned int n;

	primary = vkms_plane_init(vkmsdev, DRM_PLANE_TYPE_PRIMARY);
	if (IS_ERR(primary))
		return PTR_ERR(primary);

	if (vkmsdev->config.overlay) {
		for (n = 0; n < NUM_OVERLAY_PLANES; n++) {
			struct vkms_plane *overlay = vkms_plane_init(
				vkmsdev, DRM_PLANE_TYPE_OVERLAY);
			if (IS_ERR(overlay)) {
				ret = PTR_ERR(overlay);
				goto err_planes;
			}
		}
	}

	if (vkmsdev->config.cursor) {
		cursor = vkms_plane_init(vkmsdev, DRM_PLANE_TYPE_CURSOR);
		if (IS_ERR(cursor)) {
			ret = PTR_ERR(cursor);
			goto err_planes;
		}
	}

	vkms_crtc = vkms_crtc_init(vkmsdev, &primary->base,
				   cursor ? &cursor->base : NULL);
	if (IS_ERR(vkms_crtc)) {
		DRM_ERROR("Failed to init crtc\n");
		ret = PTR_ERR(vkms_crtc);
		goto err_planes;
	}

	for (int i = 0; i < vkmsdev->output.num_planes; i++) {
		vkmsdev->output.planes[i].base.possible_crtcs |=
			drm_crtc_mask(&vkms_crtc->base);
	}

	connector = vkms_connector_init(vkmsdev);
	if (IS_ERR(connector)) {
		DRM_ERROR("Failed to init connector\n");
		ret = PTR_ERR(connector);
		goto err_connector;
	}

	encoder = vkms_encoder_init(vkmsdev);
	if (IS_ERR(encoder)) {
		DRM_ERROR("Failed to init encoder\n");
		ret = PTR_ERR(encoder);
		goto err_encoder;
	}
	encoder->possible_crtcs |= drm_crtc_mask(&vkms_crtc->base);

	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret) {
		DRM_ERROR("Failed to attach connector to encoder\n");
		goto err_attach;
	}

	if (vkmsdev->config.writeback) {
		writeback = vkms_enable_writeback_connector(vkmsdev, vkms_crtc);
		if (writeback)
			DRM_ERROR("Failed to init writeback connector\n");
	}

	drm_mode_config_reset(dev);

	return 0;

err_attach:
	drm_encoder_cleanup(encoder);

err_encoder:
	drm_connector_cleanup(connector);

err_connector:
	drm_crtc_cleanup(&vkms_crtc->base);

err_planes:
	for (int i = 0; i < output->num_planes; i++)
		drm_plane_cleanup(&output->planes[i].base);

	memset(output, 0, sizeof(*output));

	return ret;
}

int vkms_output_init(struct vkms_device *vkmsdev)
{
	return -EOPNOTSUPP;
}
