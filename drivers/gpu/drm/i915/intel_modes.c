/*
 * Copyright (c) 2007 Dave Airlie <airlied@linux.ie>
 * Copyright (c) 2007, 2010 Intel Corporation
 *   Jesse Barnes <jesse.barnes@intel.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/fb.h>
#include <drm/drm_edid.h>
#include "drmP.h"
#include "drm_edid.h"
#include "intel_drv.h"
#include "i915_drv.h"

/**
 * intel_ddc_probe
 *
 */
bool intel_ddc_probe(struct intel_encoder *intel_encoder, int ddc_bus)
{
	struct drm_i915_private *dev_priv = intel_encoder->base.dev->dev_private;
	u8 out_buf[] = { 0x0, 0x0};
	u8 buf[2];
	struct i2c_msg msgs[] = {
		{
			.addr = DDC_ADDR,
			.flags = 0,
			.len = 1,
			.buf = out_buf,
		},
		{
			.addr = DDC_ADDR,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = buf,
		}
	};

	return i2c_transfer(&dev_priv->gmbus[ddc_bus].adapter, msgs, 2) == 2;
}

/**
 * intel_ddc_get_modes - get modelist from monitor
 * @connector: DRM connector device to use
 * @adapter: i2c adapter
 *
 * Fetch the EDID information from @connector using the DDC bus.
 */
int intel_ddc_get_modes(struct drm_connector *connector,
			struct i2c_adapter *adapter)
{
	struct edid *edid;
	int ret = 0;

	edid = drm_get_edid(connector, adapter);
	if (edid) {
		drm_mode_connector_update_edid_property(connector, edid);
		ret = drm_add_edid_modes(connector, edid);
		drm_edid_to_eld(connector, edid);
		connector->display_info.raw_edid = NULL;
		kfree(edid);
	}

	return ret;
}

static const struct drm_prop_enum_list force_audio_names[] = {
	{ HDMI_AUDIO_OFF_DVI, "force-dvi" },
	{ HDMI_AUDIO_OFF, "off" },
	{ HDMI_AUDIO_AUTO, "auto" },
	{ HDMI_AUDIO_ON, "on" },
};

void
intel_attach_force_audio_property(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_property *prop;

	prop = dev_priv->force_audio_property;
	if (prop == NULL) {
		prop = drm_property_create_enum(dev, 0,
					   "audio",
					   force_audio_names,
					   ARRAY_SIZE(force_audio_names));
		if (prop == NULL)
			return;

		dev_priv->force_audio_property = prop;
	}
	drm_connector_attach_property(connector, prop, 0);
}

static const struct drm_prop_enum_list broadcast_rgb_names[] = {
	{ 0, "Full" },
	{ 1, "Limited 16:235" },
};

void
intel_attach_broadcast_rgb_property(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_property *prop;

	prop = dev_priv->broadcast_rgb_property;
	if (prop == NULL) {
		prop = drm_property_create_enum(dev, DRM_MODE_PROP_ENUM,
					   "Broadcast RGB",
					   broadcast_rgb_names,
					   ARRAY_SIZE(broadcast_rgb_names));
		if (prop == NULL)
			return;

		dev_priv->broadcast_rgb_property = prop;
	}

	drm_connector_attach_property(connector, prop, 0);
}
