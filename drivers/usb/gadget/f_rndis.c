/*
 * f_rndis.c -- RNDIS link function driver
 *
 * Copyright (C) 2003-2005,2008 David Brownell
 * Copyright (C) 2003-2004 Robert Schwebel, Benedikt Spranger
 * Copyright (C) 2008 Nokia Corporation
 * Copyright (C) 2009 Samsung Electronics
 *                    Author: Michal Nazarewicz (mina86@mina86.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */


#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/etherdevice.h>

#include <linux/atomic.h>

#include "u_ether.h"
#include "rndis.h"


static unsigned int rndis_dl_max_pkt_per_xfer = 3;
module_param(rndis_dl_max_pkt_per_xfer, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(rndis_dl_max_pkt_per_xfer,
	"Maximum packets per transfer for DL aggregation");

static unsigned int rndis_ul_max_pkt_per_xfer = 3;
module_param(rndis_ul_max_pkt_per_xfer, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(rndis_ul_max_pkt_per_xfer,
	"Maximum packets per transfer for UL aggregation");

struct f_rndis {
	struct gether			port;
	u8				ctrl_id, data_id;
	u8				ethaddr[ETH_ALEN];
	u32				vendorID;
	const char			*manufacturer;
	int				config;

	struct usb_ep			*notify;
	struct usb_request		*notify_req;
	atomic_t			notify_count;
	atomic_t			online;
};

static inline struct f_rndis *func_to_rndis(struct usb_function *f)
{
	return container_of(f, struct f_rndis, port.func);
}

static unsigned int bitrate(struct usb_gadget *g)
{
	if (gadget_is_superspeed(g) && g->speed == USB_SPEED_SUPER)
		return 13 * 1024 * 8 * 1000 * 8;
	else if (gadget_is_dualspeed(g) && g->speed == USB_SPEED_HIGH)
		return 13 * 512 * 8 * 1000 * 8;
	else
		return 19 * 64 * 1 * 1000 * 8;
}



#define LOG2_STATUS_INTERVAL_MSEC	5	
#define STATUS_BYTECOUNT		8	



static struct usb_interface_descriptor rndis_control_intf = {
	.bLength =		sizeof rndis_control_intf,
	.bDescriptorType =	USB_DT_INTERFACE,

	
	
	.bNumEndpoints =	1,
	.bInterfaceClass =	USB_CLASS_COMM,
	.bInterfaceSubClass =   USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol =   USB_CDC_ACM_PROTO_VENDOR,
	
};

static struct usb_cdc_header_desc header_desc = {
	.bLength =		sizeof header_desc,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_HEADER_TYPE,

	.bcdCDC =		cpu_to_le16(0x0110),
};

static struct usb_cdc_call_mgmt_descriptor call_mgmt_descriptor = {
	.bLength =		sizeof call_mgmt_descriptor,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_CALL_MANAGEMENT_TYPE,

	.bmCapabilities =	0x00,
	.bDataInterface =	0x01,
};

static struct usb_cdc_acm_descriptor rndis_acm_descriptor = {
	.bLength =		sizeof rndis_acm_descriptor,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_ACM_TYPE,

	.bmCapabilities =	0x00,
};

static struct usb_cdc_union_desc rndis_union_desc = {
	.bLength =		sizeof(rndis_union_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_UNION_TYPE,
	
	
};


static struct usb_interface_descriptor rndis_data_intf = {
	.bLength =		sizeof rndis_data_intf,
	.bDescriptorType =	USB_DT_INTERFACE,

	
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	0,
	
};


static struct usb_interface_assoc_descriptor
rndis_iad_descriptor = {
	.bLength =		sizeof rndis_iad_descriptor,
	.bDescriptorType =	USB_DT_INTERFACE_ASSOCIATION,

	.bFirstInterface =	0, 
	.bInterfaceCount = 	2,	
	.bFunctionClass =	USB_CLASS_COMM,
	.bFunctionSubClass =	USB_CDC_SUBCLASS_ETHERNET,
	.bFunctionProtocol =	USB_CDC_PROTO_NONE,
	
};


static struct usb_endpoint_descriptor fs_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(STATUS_BYTECOUNT),
	.bInterval =		1 << LOG2_STATUS_INTERVAL_MSEC,
};

static struct usb_endpoint_descriptor fs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *eth_fs_function[] = {
	(struct usb_descriptor_header *) &rndis_iad_descriptor,

	
	(struct usb_descriptor_header *) &rndis_control_intf,
	(struct usb_descriptor_header *) &header_desc,
	(struct usb_descriptor_header *) &call_mgmt_descriptor,
	(struct usb_descriptor_header *) &rndis_acm_descriptor,
	(struct usb_descriptor_header *) &rndis_union_desc,
	(struct usb_descriptor_header *) &fs_notify_desc,

	
	(struct usb_descriptor_header *) &rndis_data_intf,
	(struct usb_descriptor_header *) &fs_in_desc,
	(struct usb_descriptor_header *) &fs_out_desc,
	NULL,
};


static struct usb_endpoint_descriptor hs_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(STATUS_BYTECOUNT),
	.bInterval =		LOG2_STATUS_INTERVAL_MSEC + 4,
};

static struct usb_endpoint_descriptor hs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_descriptor_header *eth_hs_function[] = {
	(struct usb_descriptor_header *) &rndis_iad_descriptor,

	
	(struct usb_descriptor_header *) &rndis_control_intf,
	(struct usb_descriptor_header *) &header_desc,
	(struct usb_descriptor_header *) &call_mgmt_descriptor,
	(struct usb_descriptor_header *) &rndis_acm_descriptor,
	(struct usb_descriptor_header *) &rndis_union_desc,
	(struct usb_descriptor_header *) &hs_notify_desc,

	
	(struct usb_descriptor_header *) &rndis_data_intf,
	(struct usb_descriptor_header *) &hs_in_desc,
	(struct usb_descriptor_header *) &hs_out_desc,
	NULL,
};


static struct usb_endpoint_descriptor ss_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	cpu_to_le16(STATUS_BYTECOUNT),
	.bInterval =		LOG2_STATUS_INTERVAL_MSEC + 4,
};

static struct usb_ss_ep_comp_descriptor ss_intr_comp_desc = {
	.bLength =		sizeof ss_intr_comp_desc,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	
	
	
	.wBytesPerInterval =	cpu_to_le16(STATUS_BYTECOUNT),
};

static struct usb_endpoint_descriptor ss_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_endpoint_descriptor ss_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor ss_bulk_comp_desc = {
	.bLength =		sizeof ss_bulk_comp_desc,
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,

	
	
	
};

static struct usb_descriptor_header *eth_ss_function[] = {
	(struct usb_descriptor_header *) &rndis_iad_descriptor,

	
	(struct usb_descriptor_header *) &rndis_control_intf,
	(struct usb_descriptor_header *) &header_desc,
	(struct usb_descriptor_header *) &call_mgmt_descriptor,
	(struct usb_descriptor_header *) &rndis_acm_descriptor,
	(struct usb_descriptor_header *) &rndis_union_desc,
	(struct usb_descriptor_header *) &ss_notify_desc,
	(struct usb_descriptor_header *) &ss_intr_comp_desc,

	
	(struct usb_descriptor_header *) &rndis_data_intf,
	(struct usb_descriptor_header *) &ss_in_desc,
	(struct usb_descriptor_header *) &ss_bulk_comp_desc,
	(struct usb_descriptor_header *) &ss_out_desc,
	(struct usb_descriptor_header *) &ss_bulk_comp_desc,
	NULL,
};


static struct usb_string rndis_string_defs[] = {
	[0].s = "RNDIS Communications Control",
	[1].s = "RNDIS Ethernet Data",
	[2].s = "RNDIS",
	{  } 
};

static struct usb_gadget_strings rndis_string_table = {
	.language =		0x0409,	
	.strings =		rndis_string_defs,
};

static struct usb_gadget_strings *rndis_strings[] = {
	&rndis_string_table,
	NULL,
};


static struct sk_buff *rndis_add_header(struct gether *port,
					struct sk_buff *skb)
{
	struct sk_buff *skb2;
	struct rndis_packet_msg_type *header = NULL;
	struct f_rndis *rndis = func_to_rndis(&port->func);

	if (rndis->port.multi_pkt_xfer) {
		if (port->header) {
			header = port->header;
			memset(header, 0, sizeof(*header));
			header->MessageType = cpu_to_le32(REMOTE_NDIS_PACKET_MSG);
			header->MessageLength = cpu_to_le32(skb->len +
							sizeof(*header));
			header->DataOffset = cpu_to_le32(36);
			header->DataLength = cpu_to_le32(skb->len);
			pr_debug("MessageLength:%d DataLength:%d\n",
						header->MessageLength,
						header->DataLength);
			return skb;
		} else {
			pr_err("RNDIS header is NULL.\n");
			return NULL;
		}
	} else {
		skb2 = skb_realloc_headroom(skb,
				sizeof(struct rndis_packet_msg_type));
		if (skb2)
			rndis_add_hdr(skb2);

		dev_kfree_skb_any(skb);
		return skb2;
	}
}

static void rndis_response_available(void *_rndis)
{
	struct f_rndis			*rndis = _rndis;
	struct usb_request		*req = rndis->notify_req;
	struct usb_composite_dev	*cdev = rndis->port.func.config->cdev;
	__le32				*data = req->buf;
	int				status;

	if (atomic_inc_return(&rndis->notify_count) != 1)
		return;

	if (!rndis->notify->driver_data)
		return;
	data[0] = cpu_to_le32(1);
	data[1] = cpu_to_le32(0);

	status = usb_ep_queue(rndis->notify, req, GFP_ATOMIC);
	if (status) {
		atomic_dec(&rndis->notify_count);
		DBG(cdev, "notify/0 --> %d\n", status);
	}
}

static void rndis_response_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_rndis			*rndis = req->context;
	struct usb_composite_dev	*cdev;
	int				status = req->status;

	if (!rndis->port.func.config || !rndis->port.func.config->cdev)
		return;
	else
		cdev = rndis->port.func.config->cdev;

	switch (status) {
	case -ECONNRESET:
	case -ESHUTDOWN:
		
		atomic_set(&rndis->notify_count, 0);
		break;
	default:
		DBG(cdev, "RNDIS %s response error %d, %d/%d\n",
			ep->name, status,
			req->actual, req->length);
		
	case 0:
		if (ep != rndis->notify)
			break;

		if (atomic_dec_and_test(&rndis->notify_count))
			break;
		status = usb_ep_queue(rndis->notify, req, GFP_ATOMIC);
		if (status) {
			atomic_dec(&rndis->notify_count);
			DBG(cdev, "notify/1 --> %d\n", status);
		}
		break;
	}
}

static void rndis_command_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_rndis			*rndis = req->context;
	struct usb_composite_dev	*cdev;
	int				status;
	rndis_init_msg_type		*buf;

	if (req->status < 0) {
		pr_err("%s: staus error: %d\n", __func__, req->status);
		return;
	}

	if (!atomic_read(&rndis->online)) {
		pr_warning("%s: usb rndis is not online\n", __func__);
		return;
	}

	if (!rndis->port.func.config || !rndis->port.func.config->cdev)
		return;
	else
		cdev = rndis->port.func.config->cdev;

	
	status = rndis_msg_parser(rndis->config, (u8 *) req->buf);
	if (status < 0)
		ERROR(cdev, "RNDIS command error %d, %d/%d\n",
			status, req->actual, req->length);

	buf = (rndis_init_msg_type *)req->buf;

	if (buf->MessageType == REMOTE_NDIS_INITIALIZE_MSG) {
		if (buf->MaxTransferSize > 2048)
			rndis->port.multi_pkt_xfer = 1;
		else
			rndis->port.multi_pkt_xfer = 0;
		DBG(cdev, "%s: MaxTransferSize: %d : Multi_pkt_txr: %s\n",
				__func__, buf->MaxTransferSize,
				rndis->port.multi_pkt_xfer ? "enabled" :
							    "disabled");
		if (rndis_dl_max_pkt_per_xfer <= 1)
			rndis->port.multi_pkt_xfer = 0;
	}
}

static int
rndis_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct f_rndis		*rndis = func_to_rndis(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	*req = cdev->req;
	int			value = -EOPNOTSUPP;
	u16			w_index = le16_to_cpu(ctrl->wIndex);
	u16			w_value = le16_to_cpu(ctrl->wValue);
	u16			w_length = le16_to_cpu(ctrl->wLength);

	if (!atomic_read(&rndis->online)) {
		pr_warning("%s: usb rndis is not online\n", __func__);
		return -ENOTCONN;
	}

	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {

	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_SEND_ENCAPSULATED_COMMAND:
		if (w_value || w_index != rndis->ctrl_id)
			goto invalid;
		
		value = w_length;
		req->complete = rndis_command_complete;
		req->context = rndis;
		
		break;

	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_GET_ENCAPSULATED_RESPONSE:
		if (w_value || w_index != rndis->ctrl_id)
			goto invalid;
		else {
			u8 *buf;
			u32 n;

			
			buf = rndis_get_next_response(rndis->config, &n);
			if (buf) {
				memcpy(req->buf, buf, n);
				req->complete = rndis_response_complete;
				req->context = rndis;
				rndis_free_response(rndis->config, buf);
				value = n;
			}
			
		}
		break;

	default:
invalid:
		VDBG(cdev, "invalid control req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
	}

	
	if (value >= 0) {
		DBG(cdev, "rndis req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
		req->zero = (value < w_length);
		req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0)
			ERROR(cdev, "rndis response on err %d\n", value);
	}

	
	return value;
}

extern int bam_adaptive_timer_enabled;
extern int POLLING_MIN_SLEEP;
extern int POLLING_MAX_SLEEP;
static int rndis_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_rndis		*rndis = func_to_rndis(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	

	if (intf == rndis->ctrl_id) {
		if (rndis->notify->driver_data) {
			VDBG(cdev, "reset rndis control %d\n", intf);
			usb_ep_disable(rndis->notify);
		}
		if (!rndis->notify->desc) {
			VDBG(cdev, "init rndis ctrl %d\n", intf);
			if (config_ep_by_speed(cdev->gadget, f, rndis->notify))
				goto fail;
		}
		usb_ep_enable(rndis->notify);
		rndis->notify->driver_data = rndis;

	} else if (intf == rndis->data_id) {
		struct net_device	*net;

		if (rndis->port.in_ep->driver_data) {
			DBG(cdev, "reset rndis\n");
			gether_disconnect(&rndis->port);
		}

		if (!rndis->port.in_ep->desc || !rndis->port.out_ep->desc) {
			DBG(cdev, "init rndis\n");
			if (config_ep_by_speed(cdev->gadget, f,
					       rndis->port.in_ep) ||
			    config_ep_by_speed(cdev->gadget, f,
					       rndis->port.out_ep)) {
				rndis->port.in_ep->desc = NULL;
				rndis->port.out_ep->desc = NULL;
				goto fail;
			}
		}

		
		rndis->port.is_zlp_ok = false;

		rndis->port.cdc_filter = 0;

		DBG(cdev, "RNDIS RX/TX early activation ... \n");
		bam_adaptive_timer_enabled = 0;
		POLLING_MIN_SLEEP = 950;
		POLLING_MAX_SLEEP = 1050;
		pr_info("%s: bam_adaptive_timer_enabled = %d POLLING_MIN_SLEEP = %d POLLING_MAX_SLEEP = %d\n", __func__,bam_adaptive_timer_enabled, POLLING_MIN_SLEEP, POLLING_MAX_SLEEP);
		net = gether_connect(&rndis->port);
		if (IS_ERR(net))
			return PTR_ERR(net);

		rndis_set_param_dev(rndis->config, net,
				&rndis->port.cdc_filter);
	} else
		goto fail;

	atomic_set(&rndis->online, 1);
	return 0;
fail:
	return -EINVAL;
}

static void rndis_suspend(struct usb_function *f)
{
	bam_adaptive_timer_enabled = 1;
	POLLING_MIN_SLEEP = 2950;
	POLLING_MAX_SLEEP = 3050;
	pr_info("%s: bam_adaptive_timer_enabled = %d POLLING_MIN_SLEEP = %d POLLING_MAX_SLEEP = %d\n", __func__,bam_adaptive_timer_enabled, POLLING_MIN_SLEEP, POLLING_MAX_SLEEP);
	pr_info("%s: need suspend function\n", __func__);
}

static void rndis_disable(struct usb_function *f)
{
	struct f_rndis		*rndis = func_to_rndis(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	atomic_set(&rndis->online, 0);

	if (!rndis->notify->driver_data)
		return;

	DBG(cdev, "rndis deactivated\n");

	rndis_uninit(rndis->config);
	gether_disconnect(&rndis->port);

	usb_ep_disable(rndis->notify);
	rndis->notify->driver_data = NULL;
	bam_adaptive_timer_enabled = 1;
	POLLING_MIN_SLEEP = 2950;
	POLLING_MAX_SLEEP = 3050;
	pr_info("%s: bam_adaptive_timer_enabled = %d POLLING_MIN_SLEEP = %d POLLING_MAX_SLEEP = %d\n", __func__,bam_adaptive_timer_enabled, POLLING_MIN_SLEEP, POLLING_MAX_SLEEP);
}



static void rndis_open(struct gether *geth)
{
	struct f_rndis		*rndis = func_to_rndis(&geth->func);
	struct usb_composite_dev *cdev = geth->func.config->cdev;

	DBG(cdev, "%s\n", __func__);

	rndis_set_param_medium(rndis->config, NDIS_MEDIUM_802_3,
				bitrate(cdev->gadget) / 100);
	rndis_signal_connect(rndis->config);
}

static void rndis_close(struct gether *geth)
{
	struct f_rndis		*rndis = func_to_rndis(&geth->func);

	DBG(geth->func.config->cdev, "%s\n", __func__);

	rndis_set_param_medium(rndis->config, NDIS_MEDIUM_802_3, 0);
	rndis_signal_disconnect(rndis->config);
}



static int
rndis_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_rndis		*rndis = func_to_rndis(f);
	int			status;
	struct usb_ep		*ep;

	
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	rndis->ctrl_id = status;
	rndis_iad_descriptor.bFirstInterface = status;

	rndis_control_intf.bInterfaceNumber = status;
	rndis_union_desc.bMasterInterface0 = status;

	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	rndis->data_id = status;

	rndis_data_intf.bInterfaceNumber = status;
	rndis_union_desc.bSlaveInterface0 = status;

	status = -ENODEV;

	
	ep = usb_ep_autoconfig(cdev->gadget, &fs_in_desc);
	if (!ep)
		goto fail;
	rndis->port.in_ep = ep;
	ep->driver_data = cdev;	

	ep = usb_ep_autoconfig(cdev->gadget, &fs_out_desc);
	if (!ep)
		goto fail;
	rndis->port.out_ep = ep;
	ep->driver_data = cdev;	

	ep = usb_ep_autoconfig(cdev->gadget, &fs_notify_desc);
	if (!ep)
		goto fail;
	rndis->notify = ep;
	ep->driver_data = cdev;	

	status = -ENOMEM;

	
	rndis->notify_req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!rndis->notify_req)
		goto fail;
	rndis->notify_req->buf = kmalloc(STATUS_BYTECOUNT, GFP_KERNEL);
	if (!rndis->notify_req->buf)
		goto fail;
	rndis->notify_req->length = STATUS_BYTECOUNT;
	rndis->notify_req->context = rndis;
	rndis->notify_req->complete = rndis_response_complete;

	
	f->descriptors = usb_copy_descriptors(eth_fs_function);
	if (!f->descriptors)
		goto fail;

	if (gadget_is_dualspeed(c->cdev->gadget)) {
		hs_in_desc.bEndpointAddress =
				fs_in_desc.bEndpointAddress;
		hs_out_desc.bEndpointAddress =
				fs_out_desc.bEndpointAddress;
		hs_notify_desc.bEndpointAddress =
				fs_notify_desc.bEndpointAddress;

		
		f->hs_descriptors = usb_copy_descriptors(eth_hs_function);
		if (!f->hs_descriptors)
			goto fail;
	}

	if (gadget_is_superspeed(c->cdev->gadget)) {
		ss_in_desc.bEndpointAddress =
				fs_in_desc.bEndpointAddress;
		ss_out_desc.bEndpointAddress =
				fs_out_desc.bEndpointAddress;
		ss_notify_desc.bEndpointAddress =
				fs_notify_desc.bEndpointAddress;

		
		f->ss_descriptors = usb_copy_descriptors(eth_ss_function);
		if (!f->ss_descriptors)
			goto fail;
	}

	rndis->port.open = rndis_open;
	rndis->port.close = rndis_close;

	status = rndis_register(rndis_response_available, rndis);
	if (status < 0)
		goto fail;
	rndis->config = status;

	rndis_set_param_medium(rndis->config, NDIS_MEDIUM_802_3, 0);
	rndis_set_host_mac(rndis->config, rndis->ethaddr);
	rndis_set_max_pkt_xfer(rndis->config, rndis_ul_max_pkt_per_xfer);

	if (rndis->manufacturer && rndis->vendorID &&
			rndis_set_param_vendor(rndis->config, rndis->vendorID,
					       rndis->manufacturer))
		goto fail;


	DBG(cdev, "RNDIS: %s speed IN/%s OUT/%s NOTIFY/%s\n",
			gadget_is_superspeed(c->cdev->gadget) ? "super" :
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			rndis->port.in_ep->name, rndis->port.out_ep->name,
			rndis->notify->name);
	return 0;

fail:
	if (gadget_is_superspeed(c->cdev->gadget) && f->ss_descriptors)
		usb_free_descriptors(f->ss_descriptors);
	if (gadget_is_dualspeed(c->cdev->gadget) && f->hs_descriptors)
		usb_free_descriptors(f->hs_descriptors);
	if (f->descriptors)
		usb_free_descriptors(f->descriptors);

	if (rndis->notify_req) {
		kfree(rndis->notify_req->buf);
		usb_ep_free_request(rndis->notify, rndis->notify_req);
	}

	
	if (rndis->notify)
		rndis->notify->driver_data = NULL;
	if (rndis->port.out_ep->desc)
		rndis->port.out_ep->driver_data = NULL;
	if (rndis->port.in_ep->desc)
		rndis->port.in_ep->driver_data = NULL;

	ERROR(cdev, "%s: can't bind, err %d\n", f->name, status);

	return status;
}

static void
rndis_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_rndis		*rndis = func_to_rndis(f);

	rndis_deregister(rndis->config);
	rndis_exit();

	if (gadget_is_superspeed(c->cdev->gadget))
		usb_free_descriptors(f->ss_descriptors);
	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->descriptors);

	kfree(rndis->notify_req->buf);
	usb_ep_free_request(rndis->notify, rndis->notify_req);

	kfree(rndis);
}

static inline bool can_support_rndis(struct usb_configuration *c)
{
	
	return true;
}

int
rndis_bind_config(struct usb_configuration *c, u8 ethaddr[ETH_ALEN])
{
	return rndis_bind_config_vendor(c, ethaddr, 0, NULL);
}

int
rndis_bind_config_vendor(struct usb_configuration *c, u8 ethaddr[ETH_ALEN],
				u32 vendorID, const char *manufacturer)
{
	struct f_rndis	*rndis;
	int		status;

	if (!can_support_rndis(c) || !ethaddr)
		return -EINVAL;

	
	status = rndis_init();
	if (status < 0)
		return status;

	
	if (rndis_string_defs[0].id == 0) {

		
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		rndis_string_defs[0].id = status;
		rndis_control_intf.iInterface = status;

		
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		rndis_string_defs[1].id = status;
		rndis_data_intf.iInterface = status;

		
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		rndis_string_defs[2].id = status;
		rndis_iad_descriptor.iFunction = status;
	}

	
	status = -ENOMEM;
	rndis = kzalloc(sizeof *rndis, GFP_KERNEL);
	if (!rndis)
		goto fail;

	memcpy(rndis->ethaddr, ethaddr, ETH_ALEN);
	rndis->vendorID = vendorID;
	rndis->manufacturer = manufacturer;

	
	rndis->port.cdc_filter = 0;

	
	rndis->port.header_len = sizeof(struct rndis_packet_msg_type);
	rndis->port.wrap = rndis_add_header;
	rndis->port.unwrap = rndis_rm_hdr;
	rndis->port.ul_max_pkts_per_xfer = rndis_ul_max_pkt_per_xfer;
	rndis->port.dl_max_pkts_per_xfer = rndis_dl_max_pkt_per_xfer;

	rndis->port.func.name = "rndis";
	rndis->port.func.strings = rndis_strings;
	
	rndis->port.func.bind = rndis_bind;
	rndis->port.func.unbind = rndis_unbind;
	rndis->port.func.set_alt = rndis_set_alt;
	rndis->port.func.setup = rndis_setup;
	rndis->port.func.suspend = rndis_suspend;
	rndis->port.func.disable = rndis_disable;

	status = usb_add_function(c, &rndis->port.func);
	if (status) {
		kfree(rndis);
fail:
		rndis_exit();
	}
	return status;
}
