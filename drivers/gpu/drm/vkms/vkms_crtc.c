// SPDX-License-Identifier: GPL-2.0+

#include "linux/mutex.h"
#include <linux/dma-fence.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "vkms_drv.h"

static enum hrtimer_restart vkms_vblank_simulate(struct hrtimer *timer)
{
	struct vkms_crtc *vkms_crtc = timer_to_vkms_crtc(timer);
	struct drm_crtc *crtc = &vkms_crtc->base;
	struct vkms_crtc_state *state;
	u64 ret_overrun;
	bool ret, fence_cookie, composer_enabled;

	fence_cookie = dma_fence_begin_signalling();

	ret_overrun = hrtimer_forward_now(&vkms_crtc->vblank_hrtimer,
					  vkms_crtc->period_ns);
	if (ret_overrun != 1)
		pr_warn("%s: vblank timer overrun\n", __func__);

	ret = drm_crtc_handle_vblank(crtc);
	if (!ret)
		DRM_ERROR("vkms failure on handling vblank");

	state = vkms_crtc->composer_state;
	composer_enabled = vkms_crtc->composer_enabled;
	mutex_unlock(&vkms_crtc->enabled_lock);

	if (state && composer_enabled) {
		u64 frame = drm_crtc_accurate_vblank_count(crtc);

		/* update frame_start only if a queued vkms_composer_worker()
		 * has read the data
		 */
		spin_lock(&vkms_crtc->composer_lock);
		if (!state->crc_pending)
			state->frame_start = frame;
		else
			DRM_DEBUG_DRIVER("crc worker falling behind, frame_start: %llu, frame_end: %llu\n",
					 state->frame_start, frame);
		state->frame_end = frame;
		state->crc_pending = true;
		spin_unlock(&vkms_crtc->composer_lock);

		ret = queue_work(vkms_crtc->composer_workq,
				 &state->composer_work);
		if (!ret)
			DRM_DEBUG_DRIVER("Composer worker already queued\n");
	}

	dma_fence_end_signalling(fence_cookie);

	return HRTIMER_RESTART;
}

static int vkms_enable_vblank(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct vkms_crtc *vkms_crtc = drm_crtc_to_vkms_crtc(crtc);
	unsigned int pipe = drm_crtc_index(crtc);
	struct drm_vblank_crtc *vblank = &dev->vblank[pipe];

	drm_calc_timestamping_constants(crtc, &crtc->mode);

	hrtimer_init(&vkms_crtc->vblank_hrtimer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	vkms_crtc->vblank_hrtimer.function = &vkms_vblank_simulate;
	vkms_crtc->period_ns = ktime_set(0, vblank->framedur_ns);
	hrtimer_start(&vkms_crtc->vblank_hrtimer, vkms_crtc->period_ns,
		      HRTIMER_MODE_REL);

	return 0;
}

static void vkms_disable_vblank(struct drm_crtc *crtc)
{
	struct vkms_crtc *vkms_crtc = drm_crtc_to_vkms_crtc(crtc);

	hrtimer_cancel(&vkms_crtc->vblank_hrtimer);
}

static bool vkms_get_vblank_timestamp(struct drm_crtc *crtc,
				      int *max_error, ktime_t *vblank_time,
				      bool in_vblank_irq)
{
	struct drm_device *dev = crtc->dev;
	struct vkms_crtc *vkms_crtc = drm_crtc_to_vkms_crtc(crtc);
	unsigned int pipe = crtc->index;
	struct drm_vblank_crtc *vblank = &dev->vblank[pipe];

	if (!READ_ONCE(vblank->enabled)) {
		*vblank_time = ktime_get();
		return true;
	}

	*vblank_time = READ_ONCE(vkms_crtc->vblank_hrtimer.node.expires);

	if (WARN_ON(*vblank_time == vblank->time))
		return true;

	/*
	 * To prevent races we roll the hrtimer forward before we do any
	 * interrupt processing - this is how real hw works (the interrupt is
	 * only generated after all the vblank registers are updated) and what
	 * the vblank core expects. Therefore we need to always correct the
	 * timestampe by one frame.
	 */
	*vblank_time -= vkms_crtc->period_ns;

	return true;
}

static struct drm_crtc_state *
vkms_atomic_crtc_duplicate_state(struct drm_crtc *crtc)
{
	struct vkms_crtc_state *vkms_state;

	if (WARN_ON(!crtc->state))
		return NULL;

	vkms_state = kzalloc(sizeof(*vkms_state), GFP_KERNEL);
	if (!vkms_state)
		return NULL;

	__drm_atomic_helper_crtc_duplicate_state(crtc, &vkms_state->base);

	INIT_WORK(&vkms_state->composer_work, vkms_composer_worker);

	return &vkms_state->base;
}

static void vkms_atomic_crtc_destroy_state(struct drm_crtc *crtc,
					   struct drm_crtc_state *state)
{
	struct vkms_crtc_state *vkms_state = to_vkms_crtc_state(state);

	__drm_atomic_helper_crtc_destroy_state(state);

	WARN_ON(work_pending(&vkms_state->composer_work));
	kfree(vkms_state->active_planes);
	kfree(vkms_state);
}

static void vkms_atomic_crtc_reset(struct drm_crtc *crtc)
{
	struct vkms_crtc_state *vkms_state =
		kzalloc(sizeof(*vkms_state), GFP_KERNEL);

	if (crtc->state)
		vkms_atomic_crtc_destroy_state(crtc, crtc->state);

	__drm_atomic_helper_crtc_reset(crtc, &vkms_state->base);
	if (vkms_state)
		INIT_WORK(&vkms_state->composer_work, vkms_composer_worker);
}

static const struct drm_crtc_funcs vkms_crtc_funcs = {
	.set_config             = drm_atomic_helper_set_config,
	.page_flip              = drm_atomic_helper_page_flip,
	.reset                  = vkms_atomic_crtc_reset,
	.atomic_duplicate_state = vkms_atomic_crtc_duplicate_state,
	.atomic_destroy_state   = vkms_atomic_crtc_destroy_state,
	.enable_vblank		= vkms_enable_vblank,
	.disable_vblank		= vkms_disable_vblank,
	.get_vblank_timestamp	= vkms_get_vblank_timestamp,
	.get_crc_sources	= vkms_get_crc_sources,
	.set_crc_source		= vkms_set_crc_source,
	.verify_crc_source	= vkms_verify_crc_source,
};

static int vkms_crtc_atomic_check(struct drm_crtc *crtc,
				  struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state,
									  crtc);
	struct vkms_crtc_state *vkms_state = to_vkms_crtc_state(crtc_state);
	struct drm_plane *plane;
	struct drm_plane_state *plane_state;
	int i = 0, ret;

	if (vkms_state->active_planes)
		return 0;

	ret = drm_atomic_add_affected_planes(crtc_state->state, crtc);
	if (ret < 0)
		return ret;

	drm_for_each_plane_mask(plane, crtc->dev, crtc_state->plane_mask) {
		plane_state = drm_atomic_get_existing_plane_state(crtc_state->state,
								  plane);
		WARN_ON(!plane_state);

		if (!plane_state->visible)
			continue;

		i++;
	}

	vkms_state->active_planes = kcalloc(i, sizeof(plane), GFP_KERNEL);
	if (!vkms_state->active_planes)
		return -ENOMEM;
	vkms_state->num_active_planes = i;

	i = 0;
	drm_for_each_plane_mask(plane, crtc->dev, crtc_state->plane_mask) {
		plane_state = drm_atomic_get_existing_plane_state(crtc_state->state,
								  plane);

		if (!plane_state->visible)
			continue;

		vkms_state->active_planes[i++] =
			to_vkms_plane_state(plane_state);
	}

	return 0;
}

static void vkms_crtc_atomic_enable(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	drm_crtc_vblank_on(crtc);
}

static void vkms_crtc_atomic_disable(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	drm_crtc_vblank_off(crtc);
}

static void vkms_crtc_atomic_begin(struct drm_crtc *crtc,
				   struct drm_atomic_state *state)
{
	struct vkms_crtc *vkms_crtc = drm_crtc_to_vkms_crtc(crtc);

	/* This lock is held across the atomic commit to block vblank timer
	 * from scheduling vkms_composer_worker until the composer is updated
	 */
	spin_lock_irq(&vkms_crtc->lock);
}

static void vkms_crtc_atomic_flush(struct drm_crtc *crtc,
				   struct drm_atomic_state *state)
{
	struct vkms_crtc *vkms_crtc = drm_crtc_to_vkms_crtc(crtc);

	if (crtc->state->event) {
		spin_lock(&crtc->dev->event_lock);

		if (drm_crtc_vblank_get(crtc) != 0)
			drm_crtc_send_vblank_event(crtc, crtc->state->event);
		else
			drm_crtc_arm_vblank_event(crtc, crtc->state->event);

		spin_unlock(&crtc->dev->event_lock);

		crtc->state->event = NULL;
	}

	vkms_crtc->composer_state = to_vkms_crtc_state(crtc->state);

	spin_unlock_irq(&vkms_crtc->lock);
}

static const struct drm_crtc_helper_funcs vkms_crtc_helper_funcs = {
	.atomic_check	= vkms_crtc_atomic_check,
	.atomic_begin	= vkms_crtc_atomic_begin,
	.atomic_flush	= vkms_crtc_atomic_flush,
	.atomic_enable	= vkms_crtc_atomic_enable,
	.atomic_disable	= vkms_crtc_atomic_disable,
};

struct vkms_crtc *vkms_crtc_init(struct vkms_device *vkmsdev,
				 struct drm_plane *primary,
				 struct drm_plane *cursor, const char *name)
{
	struct drm_device *dev = &vkmsdev->drm;
	struct vkms_crtc *vkms_crtc;
	int ret;

	if (vkmsdev->output.num_crtcs >= VKMS_MAX_OUTPUT_OBJECTS)
		return ERR_PTR(-ENOMEM);

	vkms_crtc = &vkmsdev->output.crtcs[vkmsdev->output.num_crtcs++];

	ret = drmm_crtc_init_with_planes(dev, &vkms_crtc->base, primary, cursor,
					 &vkms_crtc_funcs, name);
	if (ret) {
		DRM_ERROR("Failed to init CRTC\n");
		goto out_error;
	}

	drm_crtc_helper_add(&vkms_crtc->base, &vkms_crtc_helper_funcs);

	drm_mode_crtc_set_gamma_size(&vkms_crtc->base, VKMS_LUT_SIZE);
	drm_crtc_enable_color_mgmt(&vkms_crtc->base, 0, false, VKMS_LUT_SIZE);

	spin_lock_init(&vkms_crtc->lock);
	spin_lock_init(&vkms_crtc->composer_lock);
	mutex_init(&vkms_crtc->enabled_lock);

	vkms_crtc->composer_workq = alloc_ordered_workqueue("vkms_composer", 0);
	if (!vkms_crtc->composer_workq) {
		ret = -ENOMEM;
		goto out_error;
	}

	return vkms_crtc;

out_error:
	memset(vkms_crtc, 0, sizeof(*vkms_crtc));
	vkmsdev->output.num_crtcs -= 1;
	return ERR_PTR(ret);
}
