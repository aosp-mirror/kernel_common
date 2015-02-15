/*
 * Media device
 *
 * Copyright (C) 2010 Nokia Corporation
 *
 * Contacts: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 *	     Sakari Ailus <sakari.ailus@iki.fi>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/media.h>
#include <linux/export.h>

#include <media/media-device.h>
#include <media/media-devnode.h>
#include <media/media-entity.h>


static int media_device_open(struct file *filp)
{
	return 0;
}

static int media_device_close(struct file *filp)
{
	return 0;
}

static int media_device_get_info(struct media_device *dev,
				 struct media_device_info __user *__info)
{
	struct media_device_info info;

	memset(&info, 0, sizeof(info));

	strlcpy(info.driver, dev->dev->driver->name, sizeof(info.driver));
	strlcpy(info.model, dev->model, sizeof(info.model));
	strlcpy(info.serial, dev->serial, sizeof(info.serial));
	strlcpy(info.bus_info, dev->bus_info, sizeof(info.bus_info));

	info.media_version = MEDIA_API_VERSION;
	info.hw_revision = dev->hw_revision;
	info.driver_version = dev->driver_version;

	return copy_to_user(__info, &info, sizeof(*__info));
}

static struct media_entity *find_entity(struct media_device *mdev, u32 id)
{
	struct media_entity *entity;
	int next = id & MEDIA_ENT_ID_FLAG_NEXT;

	id &= ~MEDIA_ENT_ID_FLAG_NEXT;

	spin_lock(&mdev->lock);

	media_device_for_each_entity(entity, mdev) {
		if ((entity->id == id && !next) ||
		    (entity->id > id && next)) {
			spin_unlock(&mdev->lock);
			return entity;
		}
	}

	spin_unlock(&mdev->lock);

	return NULL;
}

static long media_device_enum_entities(struct media_device *mdev,
				       struct media_entity_desc __user *uent)
{
	struct media_entity *ent;
	struct media_entity_desc u_ent;

	if (copy_from_user(&u_ent.id, &uent->id, sizeof(u_ent.id)))
		return -EFAULT;

	ent = find_entity(mdev, u_ent.id);

	if (ent == NULL)
		return -EINVAL;

	u_ent.id = ent->id;
	u_ent.name[0] = '\0';
	if (ent->name)
		strlcpy(u_ent.name, ent->name, sizeof(u_ent.name));
	u_ent.type = ent->type;
	u_ent.revision = ent->revision;
	u_ent.flags = ent->flags;
	u_ent.group_id = ent->group_id;
	u_ent.pads = ent->num_pads;
	u_ent.links = ent->num_links - ent->num_backlinks;
	memcpy(&u_ent.raw, &ent->info, sizeof(ent->info));
	if (copy_to_user(uent, &u_ent, sizeof(u_ent)))
		return -EFAULT;
	return 0;
}

static void media_device_kpad_to_upad(const struct media_pad *kpad,
				      struct media_pad_desc *upad)
{
	upad->entity = kpad->entity->id;
	upad->index = kpad->index;
	upad->flags = kpad->flags;
}

static long media_device_enum_links(struct media_device *mdev,
				    struct media_links_enum __user *ulinks)
{
	struct media_entity *entity;
	struct media_links_enum links;

	if (copy_from_user(&links, ulinks, sizeof(links)))
		return -EFAULT;

	entity = find_entity(mdev, links.entity);
	if (entity == NULL)
		return -EINVAL;

	if (links.pads) {
		unsigned int p;

		for (p = 0; p < entity->num_pads; p++) {
			struct media_pad_desc pad;

			memset(&pad, 0, sizeof(pad));
			media_device_kpad_to_upad(&entity->pads[p], &pad);
			if (copy_to_user(&links.pads[p], &pad, sizeof(pad)))
				return -EFAULT;
		}
	}

	if (links.links) {
		struct media_link_desc __user *ulink;
		unsigned int l;

		for (l = 0, ulink = links.links; l < entity->num_links; l++) {
			struct media_link_desc link;

			
			if (entity->links[l].source->entity != entity)
				continue;

			memset(&link, 0, sizeof(link));
			media_device_kpad_to_upad(entity->links[l].source,
						  &link.source);
			media_device_kpad_to_upad(entity->links[l].sink,
						  &link.sink);
			link.flags = entity->links[l].flags;
			if (copy_to_user(ulink, &link, sizeof(*ulink)))
				return -EFAULT;
			ulink++;
		}
	}
	if (copy_to_user(ulinks, &links, sizeof(*ulinks)))
		return -EFAULT;
	return 0;
}

static long media_device_setup_link(struct media_device *mdev,
				    struct media_link_desc __user *_ulink)
{
	struct media_link *link = NULL;
	struct media_link_desc ulink;
	struct media_entity *source;
	struct media_entity *sink;
	int ret;

	if (copy_from_user(&ulink, _ulink, sizeof(ulink)))
		return -EFAULT;

	source = find_entity(mdev, ulink.source.entity);
	sink = find_entity(mdev, ulink.sink.entity);

	if (source == NULL || sink == NULL)
		return -EINVAL;

	if (ulink.source.index >= source->num_pads ||
	    ulink.sink.index >= sink->num_pads)
		return -EINVAL;

	link = media_entity_find_link(&source->pads[ulink.source.index],
				      &sink->pads[ulink.sink.index]);
	if (link == NULL)
		return -EINVAL;

	
	ret = __media_entity_setup_link(link, ulink.flags);

	if (copy_to_user(_ulink, &ulink, sizeof(ulink)))
		return -EFAULT;

	return ret;
}

static long media_device_ioctl(struct file *filp, unsigned int cmd,
			       unsigned long arg)
{
	struct media_devnode *devnode = media_devnode_data(filp);
	struct media_device *dev = to_media_device(devnode);
	long ret;

	switch (cmd) {
	case MEDIA_IOC_DEVICE_INFO:
		ret = media_device_get_info(dev,
				(struct media_device_info __user *)arg);
		break;

	case MEDIA_IOC_ENUM_ENTITIES:
		ret = media_device_enum_entities(dev,
				(struct media_entity_desc __user *)arg);
		break;

	case MEDIA_IOC_ENUM_LINKS:
		mutex_lock(&dev->graph_mutex);
		ret = media_device_enum_links(dev,
				(struct media_links_enum __user *)arg);
		mutex_unlock(&dev->graph_mutex);
		break;

	case MEDIA_IOC_SETUP_LINK:
		mutex_lock(&dev->graph_mutex);
		ret = media_device_setup_link(dev,
				(struct media_link_desc __user *)arg);
		mutex_unlock(&dev->graph_mutex);
		break;

	default:
		ret = -ENOIOCTLCMD;
	}

	return ret;
}

static const struct media_file_operations media_device_fops = {
	.owner = THIS_MODULE,
	.open = media_device_open,
	.ioctl = media_device_ioctl,
	.release = media_device_close,
};


static ssize_t show_model(struct device *cd,
			  struct device_attribute *attr, char *buf)
{
	struct media_device *mdev = to_media_device(to_media_devnode(cd));

	return sprintf(buf, "%.*s\n", (int)sizeof(mdev->model), mdev->model);
}

static DEVICE_ATTR(model, S_IRUGO, show_model, NULL);


static void media_device_release(struct media_devnode *mdev)
{
}

int __must_check media_device_register(struct media_device *mdev)
{
	int ret;

	if (WARN_ON(mdev->dev == NULL || mdev->model[0] == 0))
		return -EINVAL;

	mdev->entity_id = 1;
	INIT_LIST_HEAD(&mdev->entities);
	spin_lock_init(&mdev->lock);
	mutex_init(&mdev->graph_mutex);

	
	mdev->devnode.fops = &media_device_fops;
	mdev->devnode.parent = mdev->dev;
	mdev->devnode.release = media_device_release;
	ret = media_devnode_register(&mdev->devnode);
	if (ret < 0)
		return ret;

	ret = device_create_file(&mdev->devnode.dev, &dev_attr_model);
	if (ret < 0) {
		media_devnode_unregister(&mdev->devnode);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(media_device_register);

void media_device_unregister(struct media_device *mdev)
{
	struct media_entity *entity;
	struct media_entity *next;

	list_for_each_entry_safe(entity, next, &mdev->entities, list)
		media_device_unregister_entity(entity);

	device_remove_file(&mdev->devnode.dev, &dev_attr_model);
	media_devnode_unregister(&mdev->devnode);
}
EXPORT_SYMBOL_GPL(media_device_unregister);

int __must_check media_device_register_entity(struct media_device *mdev,
					      struct media_entity *entity)
{
	
	WARN_ON(entity->parent != NULL);
	entity->parent = mdev;

	spin_lock(&mdev->lock);
	if (entity->id == 0)
		entity->id = mdev->entity_id++;
	else
		mdev->entity_id = max(entity->id + 1, mdev->entity_id);
	list_add_tail(&entity->list, &mdev->entities);
	spin_unlock(&mdev->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(media_device_register_entity);

void media_device_unregister_entity(struct media_entity *entity)
{
	struct media_device *mdev = entity->parent;

	if (mdev == NULL)
		return;

	spin_lock(&mdev->lock);
	list_del(&entity->list);
	spin_unlock(&mdev->lock);
	entity->parent = NULL;
}
EXPORT_SYMBOL_GPL(media_device_unregister_entity);
