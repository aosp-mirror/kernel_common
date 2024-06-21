/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef _VKMS_DRV_H_
#define _VKMS_DRV_H_

#include "drm/drm_connector.h"
#include <linux/configfs.h>
#include <linux/hrtimer.h>

#include <drm/drm.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_encoder.h>
#include <drm/drm_plane.h>
#include <drm/drm_writeback.h>

#define XRES_MIN    10
#define YRES_MIN    10

#define XRES_DEF  1024
#define YRES_DEF   768

#define XRES_MAX  8192
#define YRES_MAX  8192

#define NUM_OVERLAY_PLANES 8


#define VKMS_MAX_OUTPUT_OBJECTS 16
#define VKMS_MAX_PLANES (3 * VKMS_MAX_OUTPUT_OBJECTS)

#define VKMS_LUT_SIZE 256

struct vkms_frame_info {
	struct drm_framebuffer *fb;
	struct drm_rect src, dst;
	struct drm_rect rotated;
	struct iosys_map map[DRM_FORMAT_MAX_PLANES];
	unsigned int rotation;
	unsigned int offset;
	unsigned int pitch;
	unsigned int cpp;
};

struct pixel_argb_u16 {
	u16 a, r, g, b;
};

struct line_buffer {
	size_t n_pixels;
	struct pixel_argb_u16 *pixels;
};

struct vkms_writeback_job {
	struct iosys_map data[DRM_FORMAT_MAX_PLANES];
	struct vkms_frame_info wb_frame_info;
	void (*pixel_write)(u8 *dst_pixels, struct pixel_argb_u16 *in_pixel);
};

/**
 * vkms_plane_state - Driver specific plane state
 * @base: base plane state
 * @frame_info: data required for composing computation
 */
struct vkms_plane_state {
	struct drm_shadow_plane_state base;
	struct vkms_frame_info *frame_info;
	void (*pixel_read)(u8 *src_buffer, struct pixel_argb_u16 *out_pixel);
};

struct vkms_plane {
	struct drm_plane base;
};

struct vkms_crtc {
	struct drm_crtc base;

	struct drm_writeback_connector wb_connector;
	struct hrtimer vblank_hrtimer;
	ktime_t period_ns;
	struct drm_pending_vblank_event *event;
	/* ordered wq for composer_work */
	struct workqueue_struct *composer_workq;
	/* protects concurrent access to composer */
	spinlock_t lock;
	/* guarantees that if the composer is enabled, a job will be queued */
	struct mutex enabled_lock;

	/* protected by @enabled_lock */
	bool composer_enabled;
	struct vkms_crtc_state *composer_state;

	spinlock_t composer_lock;
};

struct vkms_color_lut {
	struct drm_color_lut *base;
	size_t lut_length;
	s64 channel_value2index_ratio;
};

/**
 * vkms_crtc_state - Driver specific CRTC state
 * @base: base CRTC state
 * @composer_work: work struct to compose and add CRC entries
 * @n_frame_start: start frame number for computed CRC
 * @n_frame_end: end frame number for computed CRC
 */
struct vkms_crtc_state {
	struct drm_crtc_state base;
	struct work_struct composer_work;

	int num_active_planes;
	/* stack of active planes for crc computation, should be in z order */
	struct vkms_plane_state **active_planes;
	struct vkms_writeback_job *active_writeback;
	struct vkms_color_lut gamma_lut;

	/* below four are protected by vkms_output.composer_lock */
	bool crc_pending;
	bool wb_pending;
	u64 frame_start;
	u64 frame_end;
};

struct vkms_output {
	int num_crtcs;
	struct vkms_crtc crtcs[VKMS_MAX_OUTPUT_OBJECTS];
	int num_encoders;
	struct drm_encoder encoders[VKMS_MAX_OUTPUT_OBJECTS];
	int num_connectors;
	struct drm_connector connectors[VKMS_MAX_OUTPUT_OBJECTS];
	int num_planes;
	struct vkms_plane planes[VKMS_MAX_PLANES];
};

struct vkms_config {
	bool writeback;
	bool cursor;
	bool overlay;
};

struct vkms_config_links {
	struct config_group group;
	unsigned long linked_object_bitmap;
};

struct vkms_config_connector {
	struct config_group config_group;
	struct drm_connector *connector;
	struct vkms_config_links possible_encoders;
	bool connected;
};

struct vkms_config_crtc {
	struct config_group config_group;
	unsigned long crtc_config_idx;
};

struct vkms_config_encoder {
	struct config_group config_group;
	struct vkms_config_links possible_crtcs;
	unsigned long encoder_config_idx;
};

struct vkms_config_plane {
	struct vkms_configfs *configfs;
	struct config_group config_group;
	struct vkms_config_links possible_crtcs;
	enum drm_plane_type type;
};

struct vkms_configfs {
	/* Directory group containing connector configs, e.g. /config/vkms/device/ */
	struct config_group device_group;
	/* Directory group containing connector configs, e.g. /config/vkms/device/connectors/ */
	struct config_group connectors_group;
	/* Directory group containing CRTC configs, e.g. /config/vkms/device/crtcs/ */
	struct config_group crtcs_group;
	/* Directory group containing encoder configs, e.g. /config/vkms/device/encoders/ */
	struct config_group encoders_group;
	/* Directory group containing plane configs, e.g. /config/vkms/device/planes/ */
	struct config_group planes_group;

	unsigned long allocated_crtcs;
	unsigned long allocated_encoders;

	struct mutex lock;

	/* The platform device if this is registered, otherwise NULL */
	struct vkms_device *vkms_device;
};

struct vkms_device_setup {
	// Is NULL in the case of the default card.
	struct vkms_configfs *configfs;
};

struct vkms_device {
	struct drm_device drm;
	struct platform_device *platform;
	// Is NULL in the case of the default card.
	struct vkms_configfs *configfs;
	struct vkms_output output;
	struct vkms_config config;
};

#define drm_crtc_to_vkms_crtc(crtc) container_of(crtc, struct vkms_crtc, base)

#define drm_device_to_vkms_device(target) \
	container_of(target, struct vkms_device, drm)

#define timer_to_vkms_crtc(timer) \
	container_of(timer, struct vkms_crtc, vblank_hrtimer)

#define to_vkms_crtc_state(target)\
	container_of(target, struct vkms_crtc_state, base)

#define to_vkms_plane_state(target)\
	container_of(target, struct vkms_plane_state, base.base)

#define item_to_configfs(item) \
	container_of(to_config_group(item), struct vkms_configfs, device_group)

#define connector_item_to_configfs(item)                                     \
	container_of(to_config_group(item->ci_parent), struct vkms_configfs, \
		     connectors_group)

#define item_to_config_connector(item)                                    \
	container_of(to_config_group(item), struct vkms_config_connector, \
		     config_group)

#define item_to_config_crtc(item)                                    \
	container_of(to_config_group(item), struct vkms_config_crtc, \
		     config_group)

#define item_to_config_encoder(item)                                    \
	container_of(to_config_group(item), struct vkms_config_encoder, \
		     config_group)

#define item_to_config_plane(item)                                    \
	container_of(to_config_group(item), struct vkms_config_plane, \
		     config_group)

#define item_to_config_links(item) \
	container_of(to_config_group(item), struct vkms_config_links, group)

#define plane_item_to_configfs(item)                                         \
	container_of(to_config_group(item->ci_parent), struct vkms_configfs, \
		     planes_group)

/* Devices */
struct vkms_device *vkms_add_device(struct vkms_configfs *configfs);
void vkms_remove_device(struct vkms_device *vkms_device);

/* CRTC */
struct vkms_crtc *vkms_crtc_init(struct vkms_device *vkmsdev,
				 struct drm_plane *primary,
				 struct drm_plane *cursor, const char *name);

int vkms_output_init(struct vkms_device *vkmsdev);
int vkms_output_init_default(struct vkms_device *vkmsdev);

struct vkms_plane *vkms_plane_init(struct vkms_device *vkmsdev,
				   enum drm_plane_type type, char* name, ...);

/* CRC Support */
const char *const *vkms_get_crc_sources(struct drm_crtc *crtc,
					size_t *count);
int vkms_set_crc_source(struct drm_crtc *crtc, const char *src_name);
int vkms_verify_crc_source(struct drm_crtc *crtc, const char *source_name,
			   size_t *values_cnt);

/* Composer Support */
void vkms_composer_worker(struct work_struct *work);
void vkms_set_composer(struct vkms_crtc *vkms_crtc, bool enabled);
void vkms_compose_row(struct line_buffer *stage_buffer, struct vkms_plane_state *plane, int y);
void vkms_writeback_row(struct vkms_writeback_job *wb, const struct line_buffer *src_buffer, int y);

/* Writeback */
int vkms_enable_writeback_connector(struct vkms_device *vkmsdev,
				    struct vkms_crtc *vkms_crtc);

/* ConfigFS Support */
int vkms_init_configfs(void);
void vkms_unregister_configfs(void);

/* Connector hotplugging */
enum drm_connector_status vkms_connector_detect(struct drm_connector *connector,
						bool force);

#endif /* _VKMS_DRV_H_ */
