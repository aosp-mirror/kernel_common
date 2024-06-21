// SPDX-License-Identifier: GPL-2.0+

#include "drm/drm_probe_helper.h"
#include <linux/configfs.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>

#include <drm/drm_plane.h>
#include <drm/drm_print.h>

#include "vkms_drv.h"

/**
 * DOC: ConfigFS Support for VKMS
 *
 * VKMS is instrumented with support for configuration via :doc:`ConfigFS
 * <../filesystems/configfs>`.
 *
 * With VKMS installed, you can mount ConfigFS at ``/config/`` like so::
 *
 *   mkdir -p /config/
 *   sudo mount -t configfs none /config
 *
 * This allows you to configure multiple virtual devices. Note
 * that the default device which can be enabled in the module params with::
 *
 *  modprobe vkms default_device=1
 *
 * is immutable because we cannot pre-populate ConfigFS directories with normal
 * files.
 *
 * To set up a new device, create a new directory under the VKMS configfs
 * directory::
 *
 *   mkdir /config/vkms/test
 *
 * With your device created you'll find an new directory ready to be
 * configured::
 *
 *   /config
 *   `-- vkms
 *       `-- test
 *           |-- connectors
 *                `-- connected
 *           |-- crtcs
 *           |-- encoders
 *           |-- planes
 *           `-- enabled
 *
 * Each directory you add within the connectors, crtcs, encoders, and planes
 * directories will let you configure a new object of that type. Adding new
 * objects will automatically create a set of default files and folders you can
 * use to configure that object.
 *
 * For instance, we can set up a two-output device like so::
 *
 *   DRM_PLANE_TYPE_PRIMARY=1
 *   DRM_PLANE_TYPE_CURSOR=2
 *   DRM_PLANE_TYPE_OVERLAY=0
 *
 *   mkdir /config/vkms/test/planes/primary
 *   echo $DRM_PLANE_TYPE_PRIMARY > /config/vkms/test/planes/primary/type
 *
 *   mkdir /config/vkms/test/planes/other_primary
 *   echo $DRM_PLANE_TYPE_PRIMARY > /config/vkms/test/planes/other_primary/type
 *
 *   mkdir /config/vkms/test/crtcs/crtc
 *   mkdir /config/vkms/test/crtcs/crtc_other
 *
 *   mkdir /config/vkms/test/encoders/encoder
 *   mkdir /config/vkms/test/encoders/encoder_other
 *
 *   mkdir /config/vkms/test/connectors/connector
 *   mkdir /config/vkms/test/connectors/connector_other
 *
 * You can see that specific attributes, such as ``.../<plane>/type``, can be
 * configured by writing into them. Associating objects together can be done via
 * symlinks::
 *
 *   ln -s /config/vkms/test/encoders/encoder       /config/vkms/test/connectors/connector/possible_encoders
 *   ln -s /config/vkms/test/encoders/encoder_other /config/vkms/test/connectors/connector_other/possible_encoders
 *
 *   ln -s /config/vkms/test/crtcs/crtc             /config/vkms/test/planes/primary/possible_crtcs/
 *   ln -s /config/vkms/test/crtcs/crtc_other       /config/vkms/test/planes/other_primary/possible_crtcs/
 *
 *   ln -s /config/vkms/test/crtcs/crtc             /config/vkms/test/encoders/encoder/possible_crtcs/
 *   ln -s /config/vkms/test/crtcs/crtc_other       /config/vkms/test/encoders/encoder_other/possible_crtcs/
 *
 * Finally, to enable your configured device, just write 1 to the ``enabled``
 * file::
 *
 *   echo 1 > /config/vkms/test/enabled
 *
 * By default no display is "connected" so to connect a connector you'll also
 * have to write 1 to a connectors "connected" attribute::
 *
 *   echo 1 > /config/vkms/test/connectors/connector/connected
 *
 * One can verify that this is worked using the `modetest` utility or the
 * equivalent for your platform.
 *
 * When you're done with the virtual device, you can clean up the device like
 * so::
 *
 *   echo 0 > /config/vkms/test/enabled
 *
 *   rm /config/vkms/test/connectors/connector/possible_encoders/encoder
 *   rm /config/vkms/test/encoders/encoder/possible_crtcs/crtc
 *   rm /config/vkms/test/planes/primary/possible_crtcs/crtc
 *   rm /config/vkms/test/planes/cursor/possible_crtcs/crtc
 *   rm /config/vkms/test/planes/overlay/possible_crtcs/crtc
 *   rm /config/vkms/test/planes/overlay/possible_crtcs/crtc_other
 *   rm /config/vkms/test/planes/other_primary/possible_crtcs/crtc_other
 *
 *   rmdir /config/vkms/test/planes/primary
 *   rmdir /config/vkms/test/planes/other_primary
 *   rmdir /config/vkms/test/planes/cursor
 *   rmdir /config/vkms/test/planes/overlay
 *   rmdir /config/vkms/test/crtcs/crtc
 *   rmdir /config/vkms/test/crtcs/crtc_other
 *   rmdir /config/vkms/test/encoders/encoder
 *   rmdir /config/vkms/test/encoders/encoder_other
 *   rmdir /config/vkms/test/connectors/connector
 *   rmdir /config/vkms/test/connectors/connector_other
 *
 *   rmdir /config/vkms/test
 */

/*
 * Common helpers (i.e. common sub-groups)
 */

/* Possible CRTCs, e.g. /config/vkms/device/<object>/possible_crtcs/<symlink> */

static struct config_item_type crtc_type;

static int possible_crtcs_allow_link(struct config_item *src,
				     struct config_item *target)
{
	struct vkms_config_links *links = item_to_config_links(src);
	struct vkms_config_crtc *crtc;

	if (target->ci_type != &crtc_type) {
		DRM_ERROR("Unable to link non-CRTCs.\n");
		return -EINVAL;
	}

	crtc = item_to_config_crtc(target);

	if (links->linked_object_bitmap & BIT(crtc->crtc_config_idx)) {
		DRM_ERROR(
			"Tried to add two symlinks to the same CRTC from the same object\n");
		return -EINVAL;
	}

	links->linked_object_bitmap |= BIT(crtc->crtc_config_idx);

	return 0;
}

static void possible_crtcs_drop_link(struct config_item *src,
				     struct config_item *target)
{
	struct vkms_config_links *links = item_to_config_links(src);
	struct vkms_config_crtc *crtc = item_to_config_crtc(target);

	links->linked_object_bitmap &= ~BIT(crtc->crtc_config_idx);
}

static struct configfs_item_operations possible_crtcs_item_ops = {
	.allow_link = &possible_crtcs_allow_link,
	.drop_link = &possible_crtcs_drop_link,
};

static struct config_item_type possible_crtcs_group_type = {
	.ct_item_ops = &possible_crtcs_item_ops,
	.ct_owner = THIS_MODULE,
};

static void add_possible_crtcs(struct config_group *parent,
			       struct config_group *possible_crtcs)
{
	config_group_init_type_name(possible_crtcs, "possible_crtcs",
				    &possible_crtcs_group_type);
	configfs_add_default_group(possible_crtcs, parent);
}

/* Possible encoders, e.g. /config/vkms/device/connector/possible_encoders/<symlink> */

static struct config_item_type encoder_type;

static int possible_encoders_allow_link(struct config_item *src,
					struct config_item *target)
{
	struct vkms_config_links *links = item_to_config_links(src);
	struct vkms_config_encoder *encoder;

	if (target->ci_type != &encoder_type) {
		DRM_ERROR("Unable to link non-encoders.\n");
		return -EINVAL;
	}

	encoder = item_to_config_encoder(target);

	if (links->linked_object_bitmap & BIT(encoder->encoder_config_idx)) {
		DRM_ERROR(
			"Tried to add two symlinks to the same encoder from the same object\n");
		return -EINVAL;
	}

	links->linked_object_bitmap |= BIT(encoder->encoder_config_idx);

	return 0;
}

static void possible_encoders_drop_link(struct config_item *src,
					struct config_item *target)
{
	struct vkms_config_links *links = item_to_config_links(src);
	struct vkms_config_encoder *encoder = item_to_config_encoder(target);

	links->linked_object_bitmap &= ~BIT(encoder->encoder_config_idx);
}

static struct configfs_item_operations possible_encoders_item_ops = {
	.allow_link = &possible_encoders_allow_link,
	.drop_link = &possible_encoders_drop_link,
};

static struct config_item_type possible_encoders_group_type = {
	.ct_item_ops = &possible_encoders_item_ops,
	.ct_owner = THIS_MODULE,
};

static void add_possible_encoders(struct config_group *parent,
				  struct config_group *possible_encoders)
{
	config_group_init_type_name(possible_encoders, "possible_encoders",
				    &possible_encoders_group_type);
	configfs_add_default_group(possible_encoders, parent);
}

/*
 * Individual objects (connectors, crtcs, encoders, planes):
 */

/*  Connector item, e.g. /config/vkms/device/connectors/ID */

static ssize_t connector_connected_show(struct config_item *item, char *buf)
{
	struct vkms_config_connector *connector =
		item_to_config_connector(item);
	struct vkms_configfs *configfs = connector_item_to_configfs(item);
	bool connected = false;

	mutex_lock(&configfs->lock);
	connected = connector->connected;
	mutex_unlock(&configfs->lock);

	return sprintf(buf, "%d\n", connected);
}

static ssize_t connector_connected_store(struct config_item *item,
					 const char *buf, size_t len)
{
	struct vkms_config_connector *connector =
		item_to_config_connector(item);
	struct vkms_configfs *configfs = connector_item_to_configfs(item);
	int val, ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	if (val != 1 && val != 0)
		return -EINVAL;

	mutex_lock(&configfs->lock);
	connector->connected = val;
	if (!connector->connector) {
		pr_info("VKMS Device %s is not yet enabled, connector will be enabled on start",
			configfs->device_group.cg_item.ci_name);
	}
	mutex_unlock(&configfs->lock);

	if (connector->connector)
		drm_kms_helper_hotplug_event(connector->connector->dev);

	return len;
}

CONFIGFS_ATTR(connector_, connected);

static struct configfs_attribute *connector_attrs[] = {
	&connector_attr_connected,
	NULL,
};

static struct config_item_type connector_type = {
	.ct_attrs = connector_attrs,
	.ct_owner = THIS_MODULE,
};

/*  Crtc item, e.g. /config/vkms/device/crtcs/ID */

static struct config_item_type crtc_type = {
	.ct_owner = THIS_MODULE,
};

/*  Encoder item, e.g. /config/vkms/device/encoder/ID */

static struct config_item_type encoder_type = {
	.ct_owner = THIS_MODULE,
};

/*  Plane item, e.g. /config/vkms/device/planes/ID */

static ssize_t plane_type_show(struct config_item *item, char *buf)
{
	struct vkms_config_plane *plane = item_to_config_plane(item);
	struct vkms_configfs *configfs = plane_item_to_configfs(item);
	enum drm_plane_type plane_type;

	mutex_lock(&configfs->lock);
	plane_type = plane->type;
	mutex_unlock(&configfs->lock);

	return sprintf(buf, "%u\n", plane_type);
}

static ssize_t plane_type_store(struct config_item *item, const char *buf,
				size_t len)
{
	struct vkms_config_plane *plane = item_to_config_plane(item);
	struct vkms_configfs *configfs = plane_item_to_configfs(item);
	int val, ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	if (val != DRM_PLANE_TYPE_PRIMARY && val != DRM_PLANE_TYPE_CURSOR &&
	    val != DRM_PLANE_TYPE_OVERLAY)
		return -EINVAL;

	mutex_lock(&configfs->lock);
	plane->type = val;
	mutex_unlock(&configfs->lock);

	return len;
}

CONFIGFS_ATTR(plane_, type);

static struct configfs_attribute *plane_attrs[] = {
	&plane_attr_type,
	NULL,
};

static struct config_item_type plane_type = {
	.ct_attrs = plane_attrs,
	.ct_owner = THIS_MODULE,
};

/*
 * Directory groups, e.g. /config/vkms/device/{planes, crtcs, ...}
 */

/* Connectors group: /config/vkms/device/connectors/ */

static struct config_group *connectors_group_make(struct config_group *group,
						  const char *name)
{
	struct vkms_config_connector *connector =
		kzalloc(sizeof(*connector), GFP_KERNEL);
	if (!connector)
		return ERR_PTR(-ENOMEM);

	config_group_init_type_name(&connector->config_group, name,
				    &connector_type);
	add_possible_encoders(&connector->config_group,
			      &connector->possible_encoders.group);
	connector->connected = false;

	return &connector->config_group;
}

static void connectors_group_drop(struct config_group *group,
				  struct config_item *item)
{
	struct vkms_config_connector *connector =
		item_to_config_connector(item);
	kfree(connector);
}

static struct configfs_group_operations connectors_group_ops = {
	.make_group = &connectors_group_make,
	.drop_item = &connectors_group_drop,
};

static struct config_item_type connectors_group_type = {
	.ct_group_ops = &connectors_group_ops,
	.ct_owner = THIS_MODULE,
};

/* CRTCs group: /config/vkms/device/crtcs/ */

static struct config_group *crtcs_group_make(struct config_group *group,
					     const char *name)
{
	struct vkms_configfs *configfs =
		container_of(group, struct vkms_configfs, crtcs_group);
	unsigned long next_idx;
	struct vkms_config_crtc *crtc;

	mutex_lock(&configfs->lock);

	next_idx = find_first_zero_bit(&configfs->allocated_crtcs,
				       VKMS_MAX_OUTPUT_OBJECTS);

	if (next_idx == VKMS_MAX_OUTPUT_OBJECTS) {
		DRM_ERROR("Unable to allocate another CRTC.\n");
		mutex_unlock(&configfs->lock);
		return ERR_PTR(-ENOMEM);
	}

	crtc = kzalloc(sizeof(*crtc), GFP_KERNEL);
	if (!crtc) {
		DRM_ERROR("Unable to allocate CRTC.\n");
		mutex_unlock(&configfs->lock);
		return ERR_PTR(-ENOMEM);
	}

	config_group_init_type_name(&crtc->config_group, name, &crtc_type);
	crtc->crtc_config_idx = next_idx;

	set_bit(next_idx, &configfs->allocated_crtcs);

	mutex_unlock(&configfs->lock);

	return &crtc->config_group;
}

static void crtcs_group_drop(struct config_group *group,
			     struct config_item *item)
{
	struct vkms_config_crtc *crtc = item_to_config_crtc(item);

	kfree(crtc);
}

static struct configfs_group_operations crtcs_group_ops = {
	.make_group = &crtcs_group_make,
	.drop_item = &crtcs_group_drop,
};

static struct config_item_type crtcs_group_type = {
	.ct_group_ops = &crtcs_group_ops,
	.ct_owner = THIS_MODULE,
};

/* Encoders group: /config/vkms/device/encoders/ */

static struct config_group *encoders_group_make(struct config_group *group,
						const char *name)
{
	struct vkms_configfs *configfs =
		container_of(group, struct vkms_configfs, encoders_group);
	unsigned long next_idx;
	struct vkms_config_encoder *encoder;

	mutex_lock(&configfs->lock);

	next_idx = find_first_zero_bit(&configfs->allocated_encoders,
				       VKMS_MAX_OUTPUT_OBJECTS);

	if (next_idx == VKMS_MAX_OUTPUT_OBJECTS) {
		DRM_ERROR("Unable to allocate another encoder.\n");
		mutex_unlock(&configfs->lock);
		return ERR_PTR(-ENOMEM);
	}

	encoder = kzalloc(sizeof(*encoder), GFP_KERNEL);
	if (!encoder) {
		DRM_ERROR("Unable to allocate encoder.\n");
		mutex_unlock(&configfs->lock);
		return ERR_PTR(-ENOMEM);
	}

	config_group_init_type_name(&encoder->config_group, name,
				    &encoder_type);
	add_possible_crtcs(&encoder->config_group,
			   &encoder->possible_crtcs.group);
	encoder->encoder_config_idx = next_idx;
	set_bit(next_idx, &configfs->allocated_encoders);

	mutex_unlock(&configfs->lock);

	return &encoder->config_group;
}

static void encoders_group_drop(struct config_group *group,
				struct config_item *item)
{
	struct vkms_config_encoder *encoder = item_to_config_encoder(item);

	kfree(encoder);
}

static struct configfs_group_operations encoders_group_ops = {
	.make_group = &encoders_group_make,
	.drop_item = &encoders_group_drop,
};

static struct config_item_type encoders_group_type = {
	.ct_group_ops = &encoders_group_ops,
	.ct_owner = THIS_MODULE,
};

/* Planes group: /config/vkms/device/planes/ */

static struct config_group *make_plane_group(struct config_group *group,
					     const char *name)
{
	struct vkms_config_plane *plane = kzalloc(sizeof(*plane), GFP_KERNEL);

	if (!plane)
		return ERR_PTR(-ENOMEM);

	config_group_init_type_name(&plane->config_group, name, &plane_type);
	add_possible_crtcs(&plane->config_group, &plane->possible_crtcs.group);

	return &plane->config_group;
}

static void drop_plane_group(struct config_group *group,
			     struct config_item *item)
{
	struct vkms_config_plane *plane = item_to_config_plane(item);

	kfree(plane);
}

static struct configfs_group_operations plane_group_ops = {
	.make_group = &make_plane_group,
	.drop_item = &drop_plane_group,
};

static struct config_item_type planes_group_type = {
	.ct_group_ops = &plane_group_ops,
	.ct_owner = THIS_MODULE,
};

/* Root directory group, e.g. /config/vkms/device */

static ssize_t device_enabled_show(struct config_item *item, char *buf)
{
	struct vkms_configfs *configfs = item_to_configfs(item);
	bool is_enabled;

	mutex_lock(&configfs->lock);
	is_enabled = configfs->vkms_device != NULL;
	mutex_unlock(&configfs->lock);

	return sprintf(buf, "%d\n", is_enabled);
}

static ssize_t device_enabled_store(struct config_item *item, const char *buf,
				    size_t len)
{
	struct vkms_configfs *configfs = item_to_configfs(item);
	struct vkms_device *device;
	int enabled, ret;

	ret = kstrtoint(buf, 0, &enabled);
	if (ret)
		return ret;

	if (enabled == 0) {
		mutex_lock(&configfs->lock);
		if (configfs->vkms_device) {
			vkms_remove_device(configfs->vkms_device);
			configfs->vkms_device = NULL;
		}
		mutex_unlock(&configfs->lock);

		return len;
	}

	if (enabled == 1) {
		mutex_lock(&configfs->lock);
		if (!configfs->vkms_device) {
			device = vkms_add_device(configfs);
			if (IS_ERR(device)) {
				mutex_unlock(&configfs->lock);
				return -PTR_ERR(device);
			}

			configfs->vkms_device = device;
		}
		mutex_unlock(&configfs->lock);

		return len;
	}

	return -EINVAL;
}

CONFIGFS_ATTR(device_, enabled);

static ssize_t device_id_show(struct config_item *item, char *buf)
{
	struct vkms_configfs *configfs = item_to_configfs(item);
	int id = -1;

	mutex_lock(&configfs->lock);
	if (configfs->vkms_device)
		id = configfs->vkms_device->platform->id;

	mutex_unlock(&configfs->lock);

	return sprintf(buf, "%d\n", id);
}

CONFIGFS_ATTR_RO(device_, id);

static struct configfs_attribute *device_group_attrs[] = {
	&device_attr_id,
	&device_attr_enabled,
	NULL,
};

static struct config_item_type device_group_type = {
	.ct_attrs = device_group_attrs,
	.ct_owner = THIS_MODULE,
};

static void vkms_configfs_setup_default_groups(struct vkms_configfs *configfs,
					       const char *name)
{
	config_group_init_type_name(&configfs->device_group, name,
				    &device_group_type);

	config_group_init_type_name(&configfs->connectors_group, "connectors",
				    &connectors_group_type);
	configfs_add_default_group(&configfs->connectors_group,
				   &configfs->device_group);

	config_group_init_type_name(&configfs->crtcs_group, "crtcs",
				    &crtcs_group_type);
	configfs_add_default_group(&configfs->crtcs_group,
				   &configfs->device_group);

	config_group_init_type_name(&configfs->encoders_group, "encoders",
				    &encoders_group_type);
	configfs_add_default_group(&configfs->encoders_group,
				   &configfs->device_group);

	config_group_init_type_name(&configfs->planes_group, "planes",
				    &planes_group_type);
	configfs_add_default_group(&configfs->planes_group,
				   &configfs->device_group);
}

/* Root directory group and subsystem, e.g. /config/vkms/ */

static struct config_group *make_root_group(struct config_group *group,
					    const char *name)
{
	struct vkms_configfs *configfs = kzalloc(sizeof(*configfs), GFP_KERNEL);

	if (!configfs)
		return ERR_PTR(-ENOMEM);

	vkms_configfs_setup_default_groups(configfs, name);
	mutex_init(&configfs->lock);

	return &configfs->device_group;
}

static void drop_root_group(struct config_group *group,
			    struct config_item *item)
{
	struct vkms_configfs *configfs = item_to_configfs(item);

	mutex_lock(&configfs->lock);
	if (configfs->vkms_device)
		vkms_remove_device(configfs->vkms_device);
	mutex_unlock(&configfs->lock);

	kfree(configfs);
}

static struct configfs_group_operations root_group_ops = {
	.make_group = &make_root_group,
	.drop_item = &drop_root_group,
};

static struct config_item_type vkms_type = {
	.ct_group_ops = &root_group_ops,
	.ct_owner = THIS_MODULE,
};

static struct configfs_subsystem vkms_subsys = {
	.su_group = {
		.cg_item = {
			.ci_name = "vkms",
			.ci_type = &vkms_type,
		},
	},
	.su_mutex = __MUTEX_INITIALIZER(vkms_subsys.su_mutex),
};

int vkms_init_configfs(void)
{
	config_group_init(&vkms_subsys.su_group);
	return configfs_register_subsystem(&vkms_subsys);
}

void vkms_unregister_configfs(void)
{
	configfs_unregister_subsystem(&vkms_subsys);
}
