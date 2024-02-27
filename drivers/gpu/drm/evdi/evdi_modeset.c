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

#include <drm/drm_vblank.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_atomic.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_atomic_helper.h>
#include <uapi/drm/evdi_drm.h>
#include "evdi_drm_drv.h"
#include "evdi_cursor.h"
#include "evdi_params.h"
#include <drm/drm_gem_atomic_helper.h>

static void evdi_crtc_dpms(__always_unused struct drm_crtc *crtc,
			   __always_unused int mode)
{
	EVDI_CHECKPT();
}

static void evdi_crtc_disable(__always_unused struct drm_crtc *crtc)
{
	EVDI_CHECKPT();
	drm_crtc_vblank_off(crtc);
}

static void evdi_crtc_destroy(struct drm_crtc *crtc)
{
	EVDI_CHECKPT();
	drm_crtc_cleanup(crtc);
	kfree(crtc);
}

static void evdi_crtc_commit(__always_unused struct drm_crtc *crtc)
{
	EVDI_CHECKPT();
}

static void evdi_crtc_set_nofb(__always_unused struct drm_crtc *crtc)
{
}

static void evdi_crtc_atomic_flush(
	struct drm_crtc *crtc
	, struct drm_atomic_state *state
	)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	struct evdi_device *evdi = crtc->dev->dev_private;
	bool notify_mode_changed = crtc_state->active &&
				   (crtc_state->mode_changed || evdi_painter_needs_full_modeset(evdi->painter));
	bool notify_dpms = crtc_state->active_changed || evdi_painter_needs_full_modeset(evdi->painter);

	if (notify_mode_changed)
		evdi_painter_mode_changed_notify(evdi, &crtc_state->adjusted_mode);

	if (notify_dpms)
		evdi_painter_dpms_notify(evdi,
			crtc_state->active ? DRM_MODE_DPMS_ON : DRM_MODE_DPMS_OFF);

	evdi_painter_set_vblank(evdi->painter, crtc, crtc_state->event);
	evdi_painter_send_update_ready_if_needed(evdi->painter);
	crtc_state->event = NULL;
}


static struct drm_crtc_helper_funcs evdi_helper_funcs = {
	.mode_set_nofb  = evdi_crtc_set_nofb,
	.atomic_flush   = evdi_crtc_atomic_flush,

	.dpms           = evdi_crtc_dpms,
	.commit         = evdi_crtc_commit,
	.disable        = evdi_crtc_disable
};

static int evdi_enable_vblank(__always_unused struct drm_crtc *crtc)
{
	return 1;
}

static void evdi_disable_vblank(__always_unused struct drm_crtc *crtc)
{
}

static const struct drm_crtc_funcs evdi_crtc_funcs = {
	.reset                  = drm_atomic_helper_crtc_reset,
	.destroy                = evdi_crtc_destroy,
	.set_config             = drm_atomic_helper_set_config,
	.page_flip              = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state   = drm_atomic_helper_crtc_destroy_state,

	.enable_vblank          = evdi_enable_vblank,
	.disable_vblank         = evdi_disable_vblank,
};

static void evdi_plane_atomic_update(struct drm_plane *plane,
				     struct drm_atomic_state *atom_state
		)
{
	struct drm_plane_state *old_state = drm_atomic_get_old_plane_state(atom_state, plane);
	struct drm_plane_state *state;
	struct evdi_device *evdi;
	struct evdi_painter *painter;
	struct drm_crtc *crtc;

	struct drm_atomic_helper_damage_iter iter;
	struct drm_rect rect;
	struct drm_clip_rect clip_rect;

	if (!plane || !plane->state) {
		EVDI_WARN("Plane state is null\n");
		return;
	}

	if (!plane->dev || !plane->dev->dev_private) {
		EVDI_WARN("Plane device is null\n");
		return;
	}

	state = plane->state;
	evdi = plane->dev->dev_private;
	painter = evdi->painter;
	crtc = state->crtc;

	if (!old_state->crtc && state->crtc)
		evdi_painter_dpms_notify(evdi, DRM_MODE_DPMS_ON);
	else if (old_state->crtc && !state->crtc)
		evdi_painter_dpms_notify(evdi, DRM_MODE_DPMS_OFF);

	if (state->fb) {
		struct drm_framebuffer *fb = state->fb;
		struct drm_framebuffer *old_fb = old_state->fb;
		struct evdi_framebuffer *efb = to_evdi_fb(fb);

		const struct drm_clip_rect fullscreen_rect = {
			0, 0, fb->width, fb->height
		};

		if (!old_fb && crtc)
			evdi_painter_force_full_modeset(painter);

		if (old_fb &&
		    fb->format && old_fb->format &&
		    fb->format->format != old_fb->format->format)
			evdi_painter_force_full_modeset(painter);

		if (fb != old_fb ||
		    evdi_painter_needs_full_modeset(painter)) {

			evdi_painter_set_scanout_buffer(painter, efb);

			state->visible = true;
			state->src.x1 = 0;
			state->src.y1 = 0;
			state->src.x2 = fb->width << 16;
			state->src.y2 = fb->height << 16;

			drm_atomic_helper_damage_iter_init(&iter, old_state, state);
			while (drm_atomic_helper_damage_iter_next(&iter, &rect)) {
				clip_rect.x1 = rect.x1;
				clip_rect.y1 = rect.y1;
				clip_rect.x2 = rect.x2;
				clip_rect.y2 = rect.y2;
				evdi_painter_mark_dirty(evdi, &clip_rect);
			}

		};

		if (evdi_painter_get_num_dirts(painter) == 0)
			evdi_painter_mark_dirty(evdi, &fullscreen_rect);
	}
}

static void evdi_cursor_atomic_get_rect(struct drm_clip_rect *rect,
					struct drm_plane_state *state)
{
	rect->x1 = (state->crtc_x < 0) ? 0 : state->crtc_x;
	rect->y1 = (state->crtc_y < 0) ? 0 : state->crtc_y;
	rect->x2 = state->crtc_x + state->crtc_w;
	rect->y2 = state->crtc_y + state->crtc_h;
}

static void evdi_cursor_atomic_update(struct drm_plane *plane,
				     struct drm_atomic_state *atom_state
		)
{
	struct drm_plane_state *old_state = drm_atomic_get_old_plane_state(atom_state, plane);

	if (plane && plane->state && plane->dev && plane->dev->dev_private) {
		struct drm_plane_state *state = plane->state;
		struct evdi_device *evdi = plane->dev->dev_private;
		struct drm_framebuffer *fb = state->fb;
		struct evdi_framebuffer *efb = to_evdi_fb(fb);

		struct drm_clip_rect old_rect;
		struct drm_clip_rect rect;
		bool cursor_changed = false;
		bool cursor_position_changed = false;
		int32_t cursor_position_x = 0;
		int32_t cursor_position_y = 0;

		mutex_lock(&plane->dev->struct_mutex);

		evdi_cursor_position(evdi->cursor, &cursor_position_x,
		&cursor_position_y);
		evdi_cursor_move(evdi->cursor, state->crtc_x, state->crtc_y);
		cursor_position_changed = cursor_position_x != state->crtc_x ||
					  cursor_position_y != state->crtc_y;

		if (fb != old_state->fb) {
			if (fb != NULL) {
				uint32_t stride = 4 * fb->width;

				evdi_cursor_set(evdi->cursor,
						efb->obj,
						fb->width,
						fb->height,
						0,
						0,
						fb->format->format,
						stride);
			}

			evdi_cursor_enable(evdi->cursor, fb != NULL);
			cursor_changed = true;
		}

		mutex_unlock(&plane->dev->struct_mutex);
		if (!evdi->cursor_events_enabled) {
			if (fb != NULL) {
				if (efb->obj->allow_sw_cursor_rect_updates) {
					evdi_cursor_atomic_get_rect(&old_rect, old_state);
					evdi_cursor_atomic_get_rect(&rect, state);

					evdi_painter_mark_dirty(evdi, &old_rect);
				} else {
					rect = evdi_painter_framebuffer_size(evdi->painter);
				}
				evdi_painter_mark_dirty(evdi, &rect);
			}
			return;
		}

		if (cursor_changed)
			evdi_painter_send_cursor_set(evdi->painter,
						     evdi->cursor);
		if (cursor_position_changed)
			evdi_painter_send_cursor_move(evdi->painter,
						      evdi->cursor);
	}
}

static const struct drm_plane_helper_funcs evdi_plane_helper_funcs = {
	.atomic_update = evdi_plane_atomic_update,
	.prepare_fb = drm_gem_plane_helper_prepare_fb
};

static const struct drm_plane_helper_funcs evdi_cursor_helper_funcs = {
	.atomic_update = evdi_cursor_atomic_update,
	.prepare_fb = drm_gem_plane_helper_prepare_fb
};

static const struct drm_plane_funcs evdi_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

static const uint32_t formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ABGR8888,
};

static struct drm_plane *evdi_create_plane(
		struct drm_device *dev,
		enum drm_plane_type type,
		const struct drm_plane_helper_funcs *helper_funcs)
{
	struct drm_plane *plane;
	int ret;
	char *plane_type = (type == DRM_PLANE_TYPE_CURSOR) ? "cursor" : "primary";

	plane = kzalloc(sizeof(*plane), GFP_KERNEL);
	if (plane == NULL) {
		EVDI_ERROR("Failed to allocate %s plane\n", plane_type);
		return NULL;
	}
	plane->format_default = true;

	ret = drm_universal_plane_init(dev,
				       plane,
				       0xFF,
				       &evdi_plane_funcs,
				       formats,
				       ARRAY_SIZE(formats),
				       NULL,
				       type,
				       NULL
				       );

	if (ret) {
		EVDI_ERROR("Failed to initialize %s plane\n", plane_type);
		kfree(plane);
		return NULL;
	}

	drm_plane_helper_add(plane, helper_funcs);

	return plane;
}

static int evdi_crtc_init(struct drm_device *dev)
{
	struct drm_crtc *crtc = NULL;
	struct drm_plane *primary_plane = NULL;
	struct drm_plane *cursor_plane = NULL;
	int status = 0;

	EVDI_CHECKPT();
	crtc = kzalloc(sizeof(struct drm_crtc), GFP_KERNEL);
	if (crtc == NULL)
		return -ENOMEM;

	primary_plane = evdi_create_plane(dev, DRM_PLANE_TYPE_PRIMARY,
					  &evdi_plane_helper_funcs);

	cursor_plane = evdi_create_plane(dev, DRM_PLANE_TYPE_CURSOR,
						&evdi_cursor_helper_funcs);

	drm_plane_enable_fb_damage_clips(primary_plane);

	status = drm_crtc_init_with_planes(dev, crtc,
					   primary_plane, cursor_plane,
					   &evdi_crtc_funcs,
					   NULL
					   );

	EVDI_DEBUG("drm_crtc_init: %d p%p\n", status, primary_plane);
	drm_crtc_helper_add(crtc, &evdi_helper_funcs);

	return 0;
}

static const struct drm_mode_config_funcs evdi_mode_funcs = {
	.fb_create = evdi_fb_user_fb_create,
	.output_poll_changed = NULL,
	.atomic_commit = drm_atomic_helper_commit,
	.atomic_check = drm_atomic_helper_check
};

void evdi_modeset_init(struct drm_device *dev)
{
	struct drm_encoder *encoder;

	EVDI_CHECKPT();

	drm_mode_config_init(dev);

	dev->mode_config.min_width = 64;
	dev->mode_config.min_height = 64;

	dev->mode_config.max_width = 7680;
	dev->mode_config.max_height = 4320;

	dev->mode_config.prefer_shadow = 0;
	dev->mode_config.preferred_depth = 24;

	dev->mode_config.funcs = &evdi_mode_funcs;

	evdi_crtc_init(dev);

	encoder = evdi_encoder_init(dev);

	evdi_connector_init(dev, encoder);

	drm_mode_config_reset(dev);
}

void evdi_modeset_cleanup(__maybe_unused struct drm_device *dev)
{
}
