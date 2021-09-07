// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Virtio pvclock implementation.
 *
 *  Copyright (C) 2021 Google, Inc.
 */

#include <linux/clocksource.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/virtio.h>
#include <linux/virtio_pvclock.h>
#include <linux/workqueue.h>
#include <asm/pvclock.h>

enum virtio_pvclock_vq {
	VIRTIO_PVCLOCK_VQ_SET_PVCLOCK_PAGE,
	VIRTIO_PVCLOCK_VQ_MAX
};

struct virtio_pvclock {
	struct virtio_device *vdev;
	struct virtqueue *set_pvclock_page_vq;
	struct virtio_pvclock_set_pvclock_page_req set_page_request;

	/* Updating the suspend time happens via scheduled work. */
	struct work_struct update_suspend_time_work;
	/* Creating the clocksource happens via scheduled work. */
	struct work_struct create_clocksource_work;

	/* Synchronize access/update to injected_suspend_ns. */
	struct mutex inject_suspend_lock;
	/* Total ns injected as sleep time. */
	u64 injected_suspend_ns;

	/* DMA address of virtio_pvclock_page. */
	dma_addr_t pvclock_page_dma_addr;
};

/* CPU accessible pointer to pvclock page. */
static struct pvclock_vsyscall_time_info *virtio_pvclock_page;

static struct virtio_device_id id_table[] = {
	{ VIRTIO_ID_PVCLOCK, VIRTIO_DEV_ANY_ID },
	{ 0 },
};

void update_suspend_time(struct work_struct *work)
{
	u64 suspend_ns, suspend_time_delta = 0;
	struct timespec64 inject_time;
	struct virtio_pvclock *vp;

	vp = container_of(work, struct virtio_pvclock,
			  update_suspend_time_work);

	virtio_cread(vp->vdev, struct virtio_pvclock_config, suspend_time_ns,
		     &suspend_ns);

	mutex_lock(&vp->inject_suspend_lock);
	if (suspend_ns > vp->injected_suspend_ns) {
		suspend_time_delta = suspend_ns - vp->injected_suspend_ns;
		vp->injected_suspend_ns = suspend_ns;
	}
	mutex_unlock(&vp->inject_suspend_lock);

	if (suspend_time_delta == 0) {
		dev_err(&vp->vdev->dev,
			"%s: suspend_time_ns is less than injected_suspend_ns\n",
			__func__);
		return;
	}

	inject_time = ns_to_timespec64(suspend_time_delta);

	timekeeping_inject_sleeptime64(&inject_time);

	dev_info(&vp->vdev->dev, "injected sleeptime: %llu ns\n",
		 suspend_time_delta);
}

static u64 virtio_pvclock_clocksource_read(struct clocksource *cs)
{
	u64 ret;

	preempt_disable_notrace();
	ret = pvclock_clocksource_read(&virtio_pvclock_page->pvti);
	preempt_enable_notrace();
	return ret;
}

static struct clocksource virtio_pvclock_clocksource = {
	.name = "virtio-pvclock",
	.rating = 200, /* default rating, updated by virtpvclock_validate */
	.read = virtio_pvclock_clocksource_read,
	.mask = CLOCKSOURCE_MASK(64),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
};

static void set_pvclock_page_callback(struct virtqueue *vq)
{
	struct virtio_pvclock *vp = vq->vdev->priv;

	if (vp->set_page_request.status != VIRTIO_PVCLOCK_S_OK) {
		dev_err(&vq->vdev->dev,
			"%s: set_pvclock_page req status is %u\n", __func__,
			vp->set_page_request.status);
		return;
	}

	/*
	 * Create the actual clocksource via a work queue because we're in an
	 * interrupt handler right now.
	 */
	schedule_work(&vp->create_clocksource_work);
}

static void create_clocksource(struct work_struct *work)
{
	struct virtio_pvclock *vp;

	vp = container_of(work, struct virtio_pvclock, create_clocksource_work);

	clocksource_register_hz(&virtio_pvclock_clocksource, NSEC_PER_SEC);

	/*
	 * VDSO pvclock can only be used if the TSCs are stable. The device also
	 * must set PVCLOCK_TSC_STABLE_BIT in the pvclock flags field.
	 */
	if (virtio_has_feature(vp->vdev, VIRTIO_PVCLOCK_F_TSC_STABLE)) {
		pvclock_set_pvti_cpu0_va(virtio_pvclock_page);
		virtio_pvclock_clocksource.archdata.vclock_mode =
			VCLOCK_PVCLOCK;
	}

	dev_info(&vp->vdev->dev, "registered clocksource\n");
}

static void virtpvclock_changed(struct virtio_device *vdev)
{
	struct virtio_pvclock *vp = vdev->priv;

	schedule_work(&vp->update_suspend_time_work);
}

static int set_pvclock_page(struct virtio_pvclock *vp)
{
	struct scatterlist sg;
	int err;

	vp->set_page_request.pvclock_page_pa = vp->pvclock_page_dma_addr;
	vp->set_page_request.system_time = ktime_get();
	vp->set_page_request.tsc_timestamp = rdtsc_ordered();

	sg_init_one(&sg, &vp->set_page_request, sizeof(vp->set_page_request));
	err = virtqueue_add_outbuf(vp->set_pvclock_page_vq, &sg, 1, vp,
				   GFP_KERNEL);

	if (err) {
		dev_err(&vp->vdev->dev, "%s: failed to add output\n", __func__);
		return err;
	}
	virtqueue_kick(vp->set_pvclock_page_vq);

	return 0;
}

static int init_vqs(struct virtio_pvclock *vp)
{
	vq_callback_t *callbacks[VIRTIO_PVCLOCK_VQ_MAX];
	struct virtqueue *vqs[VIRTIO_PVCLOCK_VQ_MAX];
	const char *names[VIRTIO_PVCLOCK_VQ_MAX];
	int err;

	callbacks[VIRTIO_PVCLOCK_VQ_SET_PVCLOCK_PAGE] =
		set_pvclock_page_callback;
	names[VIRTIO_PVCLOCK_VQ_SET_PVCLOCK_PAGE] = "set_pvclock_page";

	err = vp->vdev->config->find_vqs(vp->vdev, VIRTIO_PVCLOCK_VQ_MAX, vqs,
					 callbacks, names, NULL, NULL);
	if (err)
		return err;

	vp->set_pvclock_page_vq = vqs[VIRTIO_PVCLOCK_VQ_SET_PVCLOCK_PAGE];

	return set_pvclock_page(vp);
}

static int virtpvclock_probe(struct virtio_device *vdev)
{
	struct virtio_pvclock *vp;
	int err;

	if (!vdev->config->get) {
		dev_err(&vdev->dev, "%s: config access disabled\n", __func__);
		return -EINVAL;
	}

	vp = kzalloc(sizeof(*vp), GFP_KERNEL);
	if (!vp) {
		err = -ENOMEM;
		goto out;
	}

	virtio_pvclock_page =
		dma_alloc_coherent(vdev->dev.parent,
				   sizeof(*virtio_pvclock_page),
				   &vp->pvclock_page_dma_addr, GFP_KERNEL);

	if (!virtio_pvclock_page) {
		err = -ENOMEM;
		goto out_free_vp;
	}

	INIT_WORK(&vp->update_suspend_time_work, update_suspend_time);
	INIT_WORK(&vp->create_clocksource_work, create_clocksource);
	mutex_init(&vp->inject_suspend_lock);

	vp->vdev = vdev;
	vdev->priv = vp;

	err = init_vqs(vp);
	if (err)
		goto out_free_pvclock_page;

	virtio_device_ready(vdev);

	return 0;

out_free_pvclock_page:
	dma_free_coherent(vdev->dev.parent, sizeof(*virtio_pvclock_page),
			  virtio_pvclock_page, vp->pvclock_page_dma_addr);

out_free_vp:
	kfree(vp);
out:
	return err;
}

static void remove_common(struct virtio_pvclock *vp)
{
	/* Now we reset the device so we can clean up the queues. */
	vp->vdev->config->reset(vp->vdev);

	vp->vdev->config->del_vqs(vp->vdev);
}

static void virtpvclock_remove(struct virtio_device *vdev)
{
	struct virtio_pvclock *vp = vdev->priv;

	remove_common(vp);

	dma_free_coherent(vdev->dev.parent, sizeof(*virtio_pvclock_page),
			  virtio_pvclock_page, vp->pvclock_page_dma_addr);

	kfree(vp);
}

#ifdef CONFIG_PM_SLEEP
static int virtpvclock_freeze(struct virtio_device *vdev)
{
	struct virtio_pvclock *vp = vdev->priv;

	/*
	 * The workqueue is already frozen by the PM core before this
	 * function is called.
	 */
	remove_common(vp);
	return 0;
}

static int virtpvclock_restore(struct virtio_device *vdev)
{
	int ret;

	ret = init_vqs(vdev->priv);
	if (ret)
		return ret;

	virtio_device_ready(vdev);

	return 0;
}
#endif

#define MAX_CLOCKSOURCE_RATING 450

static int virtpvclock_validate(struct virtio_device *vdev)
{
	if (!virtio_has_feature(vdev, VIRTIO_PVCLOCK_F_CLOCKSOURCE_RATING))
		return 0;

	uint32_t rating =
		virtio_cread32(vdev, offsetof(struct virtio_pvclock_config,
					      clocksource_rating));
	if (rating > MAX_CLOCKSOURCE_RATING) {
		dev_warn(
			&vdev->dev,
			"device clocksource rating too high: %u, using max rating: %u\n",
			rating, MAX_CLOCKSOURCE_RATING);
		__virtio_clear_bit(vdev, VIRTIO_PVCLOCK_F_CLOCKSOURCE_RATING);
		virtio_pvclock_clocksource.rating = (int)MAX_CLOCKSOURCE_RATING;
	} else {
		dev_info(&vdev->dev, "clocksource rating set to %u\n", rating);
		virtio_pvclock_clocksource.rating = (int)rating;
	}

	return 0;
}

static unsigned int features[] = { VIRTIO_PVCLOCK_F_TSC_STABLE,
				   VIRTIO_PVCLOCK_F_INJECT_SLEEP,
				   VIRTIO_PVCLOCK_F_CLOCKSOURCE_RATING };

static struct virtio_driver virtio_pvclock_driver = {
	.feature_table = features,
	.feature_table_size = ARRAY_SIZE(features),
	.driver.name = KBUILD_MODNAME,
	.driver.owner = THIS_MODULE,
	.id_table = id_table,
	.validate = virtpvclock_validate,
	.probe = virtpvclock_probe,
	.remove = virtpvclock_remove,
	.config_changed = virtpvclock_changed,
#ifdef CONFIG_PM_SLEEP
	.freeze = virtpvclock_freeze,
	.restore = virtpvclock_restore,
#endif
};

module_virtio_driver(virtio_pvclock_driver);
MODULE_DEVICE_TABLE(virtio, id_table);
MODULE_DESCRIPTION("Virtio pvclock driver");
MODULE_LICENSE("GPL");
