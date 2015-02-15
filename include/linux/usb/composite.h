/*
 * composite.h -- framework for usb gadgets which are composite devices
 *
 * Copyright (C) 2006-2008 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef	__LINUX_USB_COMPOSITE_H
#define	__LINUX_USB_COMPOSITE_H


#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/switch.h>

#define USB_GADGET_DELAYED_STATUS       0x7fff	

struct usb_configuration;

struct usb_function {
	const char			*name;
	struct usb_gadget_strings	**strings;
	struct usb_descriptor_header	**descriptors;
	struct usb_descriptor_header	**hs_descriptors;
	struct usb_descriptor_header	**ss_descriptors;

	struct usb_configuration	*config;


	
	int			(*bind)(struct usb_configuration *,
					struct usb_function *);
	void			(*unbind)(struct usb_configuration *,
					struct usb_function *);

	
	int			(*set_alt)(struct usb_function *,
					unsigned interface, unsigned alt);
	int			(*get_alt)(struct usb_function *,
					unsigned interface);
	void			(*disable)(struct usb_function *);
	int			(*setup)(struct usb_function *,
					const struct usb_ctrlrequest *);
	void			(*suspend)(struct usb_function *);
	void			(*resume)(struct usb_function *);

	
	int			(*get_status)(struct usb_function *);
	int			(*func_suspend)(struct usb_function *,
						u8 suspend_opt);
	
	
	struct list_head		list;
	DECLARE_BITMAP(endpoints, 32);
};

int usb_add_function(struct usb_configuration *, struct usb_function *);

int usb_function_deactivate(struct usb_function *);
int usb_function_activate(struct usb_function *);

int usb_interface_id(struct usb_configuration *, struct usb_function *);

int config_ep_by_speed(struct usb_gadget *g, struct usb_function *f,
			struct usb_ep *_ep);

#define	MAX_CONFIG_INTERFACES		16	

struct usb_configuration {
	const char			*label;
	struct usb_gadget_strings	**strings;
	const struct usb_descriptor_header **descriptors;


	
	void			(*unbind)(struct usb_configuration *);
	int			(*setup)(struct usb_configuration *,
					const struct usb_ctrlrequest *);

	
	u8			bConfigurationValue;
	u8			iConfiguration;
	u8			bmAttributes;
	u8			bMaxPower;

	struct usb_composite_dev	*cdev;

	
	
	struct list_head	list;
	struct list_head	functions;
	u8			next_interface_id;
	unsigned		superspeed:1;
	unsigned		highspeed:1;
	unsigned		fullspeed:1;
	struct usb_function	*interface[MAX_CONFIG_INTERFACES];
};

int usb_add_config(struct usb_composite_dev *,
		struct usb_configuration *,
		int (*)(struct usb_configuration *));

int usb_remove_config(struct usb_composite_dev *,
		struct usb_configuration *);

struct usb_composite_driver {
	const char				*name;
	const char				*iProduct;
	const char				*iManufacturer;
	const struct usb_device_descriptor	*dev;
	struct usb_gadget_strings		**strings;
	enum usb_device_speed			max_speed;
	unsigned		needs_serial:1;

	int			(*unbind)(struct usb_composite_dev *);

	void			(*disconnect)(struct usb_composite_dev *);

	
	void			(*suspend)(struct usb_composite_dev *);
	void			(*resume)(struct usb_composite_dev *);
};

extern int usb_composite_probe(struct usb_composite_driver *driver,
			       int (*bind)(struct usb_composite_dev *cdev));
extern void usb_composite_unregister(struct usb_composite_driver *driver);
extern void usb_composite_setup_continue(struct usb_composite_dev *cdev);


struct usb_composite_dev {
	struct usb_gadget		*gadget;
	struct usb_request		*req;
	unsigned			bufsiz;

	struct usb_configuration	*config;

	
	
	unsigned int			suspended:1;
	struct usb_device_descriptor	desc;
	struct list_head		configs;
	struct usb_composite_driver	*driver;
	u8				next_string_id;
	u8				manufacturer_override;
	u8				product_override;
	u8				serial_override;

	unsigned			deactivations;

	int				delayed_status;

	
	spinlock_t			lock;

	int vbus_draw_units;

	
	struct switch_dev		sw_connect2pc;
	struct delayed_work request_reset;
	struct work_struct cdusbcmdwork;
	struct delayed_work cdusbcmd_vzw_unmount_work;
	struct switch_dev compositesdev;
	int unmount_cdrom_mask;
};

extern int usb_string_id(struct usb_composite_dev *c);
extern int usb_string_ids_tab(struct usb_composite_dev *c,
			      struct usb_string *str);
extern int usb_string_ids_n(struct usb_composite_dev *c, unsigned n);


#define DBG(d, fmt, args...) \
	dev_dbg(&(d)->gadget->dev , fmt , ## args)
#define VDBG(d, fmt, args...) \
	dev_vdbg(&(d)->gadget->dev , fmt , ## args)
#define ERROR(d, fmt, args...) \
	dev_err(&(d)->gadget->dev , fmt , ## args)
#define WARNING(d, fmt, args...) \
	dev_warn(&(d)->gadget->dev , fmt , ## args)
#define INFO(d, fmt, args...) \
	dev_info(&(d)->gadget->dev , fmt , ## args)

#endif	
