/*
 * Projector function driver
 *
 * Copyright (C) 2010 HTC Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>

#include <linux/types.h>
#include <linux/device.h>
#include <linux/fb.h>
#include <linux/minifb.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/htc_mode_server.h>
#include <linux/random.h>
#include <linux/switch.h>

unsigned short *test_frame;

#ifdef DBG
#undef DBG
#endif

#if 1
#define DBG(x...) do {} while (0)
#else
#define DBG(x...) printk(KERN_INFO x)
#endif

#ifdef VDBG
#undef VDBG
#endif

#if 1
#define VDBG(x...) do {} while (0)
#else
#define VDBG(x...) printk(KERN_INFO x)
#endif

static int touch_init_p2 = 0;

#define TXN_MAX 16384
#define RXN_MAX 4096

#define PROJ_RX_REQ_MAX 4

#define DEFAULT_PROJ2_WIDTH			480
#define DEFAULT_PROJ2_HEIGHT		800
#define PROJ2_TOUCH_WIDTH					2048
#define PROJ2_TOUCH_HEIGHT				2048

#define PROJECTOR2_FUNCTION_NAME "projector2"
#define FRAME_INTERVAL_TIME 200
#define CONTEXT_INFO_SIZE			28
#define MAX_NUM_CONTEXT_INFO		15
#define cHSML_UUID_SIZE             18

#define cHSML_12_CAP_ENDIAN         (1 << HSML_12_CAP_ENDIAN)

#define htc_mode_info2(fmt, args...) \
	printk(KERN_INFO "[projector2] " pr_fmt(fmt), ## args)

static DEFINE_MUTEX(hsml_header_lock);

enum {
	PRJ2_OFFLINE,
	PRJ2_ONLINE,
	PRJ2_PROJECTING,
};

enum {
    cHSML_STREAM_OFF = 0,
    cHSML_STREAM_ON,
    cHSML_ON_DEMAND,
};

static const char prj2_shortname[] = "prj2_status";

struct hsml_header07 {
	u8 signature[8];
	u16 seq;
	u32 timestamp;
	u32 frameBufferDataSize;
	u16 num_context_info;
	u8 context_info[492];
} __attribute__ ((__packed__));

#if HSML_VERSION_12
struct hsml_header12 {
	u8 signature[8];
	u32 seq;
	u32 timestamp;
	u32 frameBufferDataSize;
	u16 wWidth;
	u16 wHeight;
	u8 bPixelFormat;
	u8 bEncoding;
	u8 aucReserved[486];
} __attribute__ ((__packed__));
#endif

static struct switch_dev ml_switch = {
	.name = "mirror_link",
};

struct projector2_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;
	spinlock_t lock;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;

	struct usb_endpoint_descriptor	*in_desc;
	struct usb_endpoint_descriptor	*out_desc;

	int online;
	int error;

	struct list_head tx_idle;
	struct list_head rx_idle;

	int rx_done;

	u32 bitsPixel;
	u32 framesize;
	u32 width;
	u32 height;
	u8	init_done;
	u8 enabled;
	u16 frame_count;
	u32 rx_req_count;
	u32 tx_req_count;
#if HSML_VERSION_12
    u16	hsml_ver;
    u8 aucUUID[cHSML_UUID_SIZE];
#endif
	struct input_dev *keypad_input;
	struct input_dev *touch_input;
	char *fbaddr;

	atomic_t prj2_status;
	atomic_t prj2_enable_HSML;
	struct switch_dev prj2_status_sdev;
	struct work_struct notifier_display_work;
	struct work_struct notifier_setting_work;

	struct workqueue_struct *wq_display;
	struct work_struct send_fb_work;
	int start_send_fb;

	
	struct hsml_protocol *hsml_proto;

	u8 is_htcmode;
	struct hsml_header07 header;
#if HSML_VERSION_12
	struct hsml_header12 header12;
#endif
	u8 notify_authenticator;
};

static struct usb_interface_descriptor projector2_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 2,
	.bInterfaceClass        = 0xFF,
	.bInterfaceSubClass     = 0xFF,
	.bInterfaceProtocol     = 0xFF,
};

static struct usb_endpoint_descriptor projector2_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor projector2_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor projector2_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor projector2_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_projector2_descs[] = {
	(struct usb_descriptor_header *) &projector2_interface_desc,
	(struct usb_descriptor_header *) &projector2_fullspeed_in_desc,
	(struct usb_descriptor_header *) &projector2_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_projector2_descs[] = {
	(struct usb_descriptor_header *) &projector2_interface_desc,
	(struct usb_descriptor_header *) &projector2_highspeed_in_desc,
	(struct usb_descriptor_header *) &projector2_highspeed_out_desc,
	NULL,
};

static struct usb_string projector2_string_defs[] = {
	[0].s = "HSML Server",
	{  } 
};

static struct usb_gadget_strings projector2_string_table = {
	.language =		0x0409,	
	.strings =		projector2_string_defs,
};

static struct usb_gadget_strings *projector2_strings[] = {
	&projector2_string_table,
	NULL,
};

static struct projector2_dev *prj2_dev = NULL;

static inline struct projector2_dev *projector2_func_to_dev(struct usb_function *f)
{
	return container_of(f, struct projector2_dev, function);
}

static struct usb_request *projector2_request_new(struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
		return NULL;

	
	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

static void projector2_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static void projector2_req_put(struct projector2_dev *dev, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}

static struct usb_request *projector2_req_get(struct projector2_dev *dev, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&dev->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return req;
}

static int send_hsml_header07(struct projector2_dev *dev)
{
	struct usb_request *req;
	int err;
	static u32 seq;

	
	seq &= 0x0000ffff;
	dev->header.seq = htons((u16)seq++);
	dev->header.timestamp = htonl((u32)ktime_to_ms(ktime_get()));
	dev->header.frameBufferDataSize = htonl(dev->hsml_proto->set_display_info.wHeight *
			dev->hsml_proto->set_display_info.wWidth * (dev->bitsPixel / 8));

	while (!(req = projector2_req_get(dev, &dev->tx_idle))) {
		msleep(1);
		if (!dev->online)
			break;
	}

	if (req) {
		req->length = sizeof(struct hsml_header);
		mutex_lock(&hsml_header_lock);
		memcpy(req->buf, &dev->header, req->length);
		err = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
		if (err < 0) {
			projector2_req_put(dev, &dev->tx_idle, req);
			printk(KERN_WARNING "%s: failed to queue req"
				    " %p\n", __func__, req);
		}
		if (dev->header.num_context_info != 0) {
			dev->header.num_context_info = 0;
			memset(dev->header.context_info, 0, sizeof(dev->header.context_info));
		}
		mutex_unlock(&hsml_header_lock);
	} else {
		err = -ENODEV;
	}
	return err;
}

#if HSML_VERSION_12
static int send_hsml_header12(struct projector2_dev *dev)
{
struct usb_request *req;
int err;
static u32 seq;

	dev->header12.seq = htonl(seq++);
	dev->header12.timestamp = htonl((u32)ktime_to_ms(ktime_get()));
	dev->header12.frameBufferDataSize = htonl(dev->hsml_proto->set_display_info.wHeight *
			dev->hsml_proto->set_display_info.wWidth * (dev->bitsPixel / 8));
	dev->header12.wWidth = htons(dev->hsml_proto->set_display_info.wWidth);
	dev->header12.wHeight = htons(dev->hsml_proto->set_display_info.wHeight);
	dev->header12.bPixelFormat = dev->hsml_proto->set_display_info.bPixelFormat;
	dev->header12.bEncoding = 0;

	while (!(req = projector2_req_get(dev, &dev->tx_idle))) {
		msleep(1);
		if (!dev->online)
			break;
	}

	if (req) {
		req->length = sizeof(struct hsml_header12);
		mutex_lock(&hsml_header_lock);
		memcpy(req->buf, &dev->header12, req->length);
		err = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
		if (err < 0) {
			projector2_req_put(dev, &dev->tx_idle, req);
			printk(KERN_WARNING "%s: failed to queue req"
				    " %p\n", __func__, req);
		}
		mutex_unlock(&hsml_header_lock);
	} else {
		err = -ENODEV;
	}
	return err;
}
#endif

static void projector2_queue_out(struct projector2_dev *dev)
{
	int ret;
	struct usb_request *req;

	
	while ((req = projector2_req_get(dev, &dev->rx_idle))) {
		req->length = RXN_MAX;
		VDBG("%s: queue %p\n", __func__, req);
		ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
		if (ret < 0) {
			VDBG("projector: failed to queue out req (%d)\n", ret);
			dev->error = 1;
			projector2_req_put(dev, &dev->rx_idle, req);
			break;
		}
	}
}

static void touch_event_rotate(struct projector2_dev *dev,
		unsigned short *x, unsigned short *y)
{
	int tmp = ntohs(*x) * PROJ2_TOUCH_WIDTH
		/ dev->hsml_proto->set_display_info.wWidth;
	*x = (dev->hsml_proto->set_display_info.wHeight - ntohs(*y))
		* PROJ2_TOUCH_HEIGHT / dev->hsml_proto->set_display_info.wHeight;
	*y = tmp;
}

static void touch2_event_func(struct projector2_dev *dev,
		struct touch_content *data, int num_touch)
{
	if (num_touch > 0) {
		int i = 0;
		for (i = 0; i < num_touch; i++) {
			touch_event_rotate(dev, &data->x, &data->y);
			input_report_abs(dev->touch_input, ABS_MT_PRESSURE,
							data->pressure);
			input_report_abs(dev->touch_input, ABS_MT_POSITION_X,
							data->x);
			input_report_abs(dev->touch_input, ABS_MT_POSITION_Y,
							data->y);
			input_mt_sync(dev->touch_input);
			data++;
		}
	} else
		input_mt_sync(dev->touch_input);

	input_sync(dev->touch_input);
}


static void projector2_send_multitouch_event(struct projector2_dev *dev,
		char *data)
{
	struct touch_event *event;
	struct touch_content *content;

	event = (struct touch_event *)data;
	if (event->num_touch == 0)
		content = NULL;
	else {
		
		content = (struct touch_content *)(data + sizeof(struct touch_event));
	}
	touch2_event_func(dev, content, event->num_touch);
}

static void projector2_send_fb(struct projector2_dev *dev)
{

	struct usb_request *req;
	int xfer;
	int count = dev->hsml_proto->set_display_info.wHeight *
		dev->hsml_proto->set_display_info.wWidth * (dev->bitsPixel / 8);

	char *frame = NULL;
	unsigned long frameSize = 0;
	int last_pkt = 0;

	if (dev->hsml_proto->debug_mode) {
		frame = (char *)test_frame;
	} else {
		if (minifb_lockbuf((void **)&frame, &frameSize, MINIFB_REPEAT) < 0) {
			pr_warn("%s: no frame\n", __func__);
			return;
		}
	}

	if (frame == NULL) {
		printk(KERN_WARNING "send_fb: frame == NULL\n");
		return;
	}
	while (count > 0 || last_pkt == 1) {
		req = projector2_req_get(dev, &dev->tx_idle);
		if (req) {
			if (last_pkt == 1) {
				last_pkt = 0;
				req->length = 0;
				if (usb_ep_queue(dev->ep_in, req, GFP_ATOMIC) < 0) {
					projector2_req_put(dev, &dev->tx_idle, req);
					printk(KERN_WARNING "%s: failed to queue req %p\n",
						__func__, req);
					break;
				}
				continue;
			}

			xfer = count > TXN_MAX? TXN_MAX : count;
			req->length = xfer;
			memcpy(req->buf, frame, xfer);

			count -= xfer;
			frame += xfer;

			if (count <= 0)
				req->zero = 1;

			if (count  <= 0 && (xfer % 512) == 0)
				last_pkt = 1;

			if (usb_ep_queue(dev->ep_in, req, GFP_ATOMIC) < 0) {
				projector2_req_put(dev, &dev->tx_idle, req);
				printk(KERN_WARNING "%s: failed to queue req %p\n",
					__func__, req);
				break;
			}
		} else {
			printk(KERN_ERR "send_fb: no req to send\n");
			break;
		}
	}
	if (!dev->hsml_proto->debug_mode)
		minifb_unlockbuf();
}

static uint projector2_send_fb2(struct projector2_dev *dev)
{
struct usb_request *req;
int xfer;

char *frame;
unsigned long frameSize = 0;
int last_pkt = 0;
int count = dev->hsml_proto->set_display_info.wHeight *
	dev->hsml_proto->set_display_info.wWidth * (dev->bitsPixel / 8);
uint uXferCnt = 0;

	if (dev->hsml_proto->debug_mode) {
		frame = (char *)test_frame;
	} else {
		if (minifb_lockbuf((void**)&frame, &frameSize, MINIFB_REPEAT) < 0)
			return 0;
	}

	if (frame == NULL)
		return 0;

    if (count > frameSize)
    {
        printk(KERN_WARNING "[HSML] frameSize mismatch: %d/%lu\n", count, frameSize);
        count = frameSize;
    }

	if (dev->online
	    && (
#if HSML_VERSION_12
	        (prj2_dev->hsml_ver == cHSML_VER_12) ?
	            send_hsml_header12(dev)	:
#endif
	            send_hsml_header07(dev)) < 0) {
		printk(KERN_WARNING "%s: failed to send hsml header\n", __func__);
		goto unlock;
	}

	uXferCnt ++;
	while ((count > 0 || last_pkt == 1) && dev->online) {
		while (!(req = projector2_req_get(dev, &dev->tx_idle))) {
			msleep(1);

			if (!dev->online)
				break;
		}

		if (req) {
			if (last_pkt == 1) {
				last_pkt = 0;
				req->length = 0;
				if (usb_ep_queue(dev->ep_in, req, GFP_ATOMIC) < 0) {
					projector2_req_put(dev, &dev->tx_idle, req);
					printk(KERN_WARNING "%s: failed to queue req %p\n",
						__func__, req);
					break;
				}
				continue;
			}

			xfer = count > TXN_MAX ? TXN_MAX : count;
			req->length = xfer;
			memcpy(req->buf, frame, xfer);
			if (usb_ep_queue(dev->ep_in, req, GFP_ATOMIC) < 0) {
				projector2_req_put(dev, &dev->tx_idle, req);
				printk(KERN_WARNING "%s: failed to queue req %p\n",
					__func__, req);
				break;
			}
			count -= xfer;
			frame += xfer;
		} else {
			printk(KERN_ERR "send_fb: no req to send\n");
			break;
		}
	}

unlock:
	if (!dev->hsml_proto->debug_mode)
		minifb_unlockbuf();
	return uXferCnt;
}

void projector2_send_fb_do_work(struct work_struct *work)
{
struct projector2_dev *dev = prj2_dev;
unsigned int uXferCnt;
	while (dev->start_send_fb) {
        uXferCnt = projector2_send_fb2(dev);
        if (uXferCnt && (dev->start_send_fb == cHSML_ON_DEMAND))
            dev->start_send_fb = cHSML_STREAM_OFF;
		msleep(1);
	}
}

static void projector2_enable_fb_work(struct projector2_dev *dev, int enabled)
{
    if ((dev->start_send_fb == enabled) && (enabled == cHSML_ON_DEMAND))
    {
        queue_work(dev->wq_display, &dev->send_fb_work);
        return;
    }
	dev->start_send_fb = enabled;
	if (enabled) {
		if (atomic_read(&dev->prj2_status) != PRJ2_PROJECTING)
			atomic_set(&dev->prj2_status, PRJ2_PROJECTING);

		queue_work(dev->wq_display, &dev->notifier_display_work);
		queue_work(dev->wq_display, &dev->send_fb_work);
	} else {
		if (atomic_read(&dev->prj2_status) != PRJ2_ONLINE)
			atomic_set(&dev->prj2_status, PRJ2_ONLINE);

		queue_work(dev->wq_display, &dev->notifier_display_work);
	}
}

static void projector2_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct projector2_dev *dev = prj2_dev;
	projector2_req_put(dev, &dev->tx_idle, req);
}


static void projector2_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct projector2_dev *dev = prj2_dev;
	unsigned char *data = req->buf;

	if (req->status != 0) {
		pr_err("%s: error, status=%d\n", __func__, req->status);
		dev->error = 1;
		projector2_req_put(dev, &dev->rx_idle, req);
		return ;
	}

	switch (data[0]) {
	case HSML_MSG_TOUCH:
		projector2_send_multitouch_event(dev, data);
		break;

	default:
		pr_err("%s: Unknown message identifier %d\n", __func__, data[0]);
		break;
	}

	projector2_req_put(dev, &dev->rx_idle, req);
	projector2_queue_out(dev);
}


static int projector2_create_bulk_endpoints(struct projector2_dev *dev,
				struct usb_endpoint_descriptor *in_desc,
				struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;

	DBG("projector_create_bulk_endpoints dev: %p\n", dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if (!ep) {
		DBG("usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}
	DBG("usb_ep_autoconfig for ep_in got %s\n", ep->name);
	ep->driver_data = dev;		
	dev->ep_in = ep;

#if HSML_VERSION_12
    if (out_desc)
#endif
    {
        ep = usb_ep_autoconfig(cdev->gadget, out_desc);
        if (!ep) {
            DBG("usb_ep_autoconfig for ep_out failed\n");
            return -ENODEV;
        }
        DBG("usb_ep_autoconfig for projector ep_out got %s\n", ep->name);
        ep->driver_data = dev;		
        dev->ep_out = ep;
    }
#if HSML_VERSION_12
    else
        dev->ep_out = NULL;
#endif

	
#if HSML_VERSION_12
    if (out_desc)
#endif
    {
        for (i = 0; i < dev->rx_req_count; i++) {
            req = projector2_request_new(dev->ep_out, RXN_MAX);
            if (!req)
                goto fail;
            req->complete = projector2_complete_out;
            projector2_req_put(dev, &dev->rx_idle, req);
        }
    }

	for (i = 0; i < dev->tx_req_count; i++) {
		req = projector2_request_new(dev->ep_in, TXN_MAX);
		if (!req)
			goto fail;
		req->complete = projector2_complete_in;
		projector2_req_put(dev, &dev->tx_idle, req);
	}

	return 0;

fail:
	while ((req = projector2_req_get(dev, &dev->tx_idle)))
		projector2_request_free(req, dev->ep_in);
#if HSML_VERSION_12
    if (out_desc)
#endif
    {
        while ((req = projector2_req_get(dev, &dev->rx_idle)))
            projector2_request_free(req, dev->ep_out);
    }
	printk(KERN_ERR "projector: could not allocate requests\n");
	return -1;
}

static int
projector2_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct projector2_dev	*dev = projector2_func_to_dev(f);
	int			id;
	int			ret;

	dev->cdev = cdev;
	DBG("%s\n", __func__);

	
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;

	projector2_interface_desc.bInterfaceNumber = id;

	
#if HSML_VERSION_12
    if (dev->hsml_ver != cHSML_VER_12)
#endif
        ret = projector2_create_bulk_endpoints(dev, &projector2_fullspeed_in_desc,
                &projector2_fullspeed_out_desc);
#if HSML_VERSION_12
    else
        ret = projector2_create_bulk_endpoints(dev, &projector2_fullspeed_in_desc,
                NULL);
#endif
	if (ret)
		return ret;

	
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		projector2_highspeed_in_desc.bEndpointAddress =
			projector2_fullspeed_in_desc.bEndpointAddress;
#if HSML_VERSION_12
        if (dev->hsml_ver != cHSML_VER_12)
#endif
            projector2_highspeed_out_desc.bEndpointAddress =
                projector2_fullspeed_out_desc.bEndpointAddress;
	}

	DBG("%s speed %s: IN/%s, OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_in->name,
			(dev->ep_out ? dev->ep_out->name : "NULL")
			);
	return 0;
}


static int projector2_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct projector2_dev *dev = projector2_func_to_dev(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int ret;

	DBG("%s intf: %d alt: %d\n", __func__, intf, alt);

	ret = config_ep_by_speed(cdev->gadget, f, dev->ep_in);
	if (ret) {
		dev->ep_in->desc = NULL;
		printk(KERN_ERR "config_ep_by_speed failes for ep %s, result %d\n",
				dev->ep_in->name, ret);
		return ret;
	}
	ret = usb_ep_enable(dev->ep_in);
	if (ret) {
		printk(KERN_ERR "failed to enable ep %s, result %d\n",
			dev->ep_in->name, ret);
		return ret;
	}

#if HSML_VERSION_12
    if (dev->ep_out)
#endif
    {
        ret = config_ep_by_speed(cdev->gadget, f, dev->ep_out);
        if (ret) {
            dev->ep_out->desc = NULL;
            printk(KERN_ERR "config_ep_by_speed failes for ep %s, result %d\n",
                dev->ep_out->name, ret);
            usb_ep_disable(dev->ep_in);
            return ret;
        }
        ret = usb_ep_enable(dev->ep_out);
        if (ret) {
            printk(KERN_ERR "failed to enable ep %s, result %d\n",
                    dev->ep_out->name, ret);
            usb_ep_disable(dev->ep_in);
            return ret;
        }
    }

	dev->online = 1;

#if HSML_VERSION_12
    if (dev->ep_out)
#endif
        projector2_queue_out(dev);

	return 0;
}


static int projector2_touch_init(struct projector2_dev *dev)
{
	int x = PROJ2_TOUCH_WIDTH;
	int y = PROJ2_TOUCH_HEIGHT;
	int ret = 0;
	struct input_dev *tdev = dev->touch_input;
	if (touch_init_p2){
		pr_info("%s already initial\n", __func__);
		return 0;
	}
	dev->touch_input  = input_allocate_device();
	if (dev->touch_input == NULL) {
		printk(KERN_ERR "%s: Failed to allocate input device\n",
			__func__);
		return -1;
	}
	tdev = dev->touch_input;
	tdev->name = "hsml_touchscreen";
	set_bit(EV_SYN,    tdev->evbit);
	set_bit(EV_KEY,    tdev->evbit);
	set_bit(EV_ABS,    tdev->evbit);

	
	input_set_abs_params(tdev, ABS_MT_POSITION_X, 0, x, 0, 0);
	input_set_abs_params(tdev, ABS_MT_POSITION_Y, 0, y, 0, 0);
	input_set_abs_params(tdev, ABS_MT_PRESSURE, 0, 1, 0, 0);
	ret = input_register_device(tdev);
	if (ret) {
		printk(KERN_ERR "%s: Unable to register %s input device\n",
			__func__, tdev->name);
		input_free_device(tdev);
		return -1;
	}
	touch_init_p2 = 1;
	printk(KERN_INFO "%s OK \n", __func__);
	return 0;
}


static void prj2_notify_display(struct work_struct *w)
{
	struct projector2_dev *dev = container_of(w,
					struct projector2_dev, notifier_display_work);
	DBG("%s\n", __func__);
	switch_set_state(&dev->prj2_status_sdev, atomic_read(&dev->prj2_status));
}

static void prj2_notify_setting(struct work_struct *w)
{
	struct projector2_dev *dev = container_of(w,
					struct projector2_dev, notifier_setting_work);
	DBG("%s\n", __func__);
	switch_set_state(&ml_switch, atomic_read(&dev->prj2_enable_HSML));
}


static void projector2_function_disable(struct usb_function *f)
{
	struct projector2_dev *dev = projector2_func_to_dev(f);

	DBG("%s\n", __func__);

	dev->start_send_fb = cHSML_STREAM_OFF;
	dev->online = 0;
	dev->error = 1;
	usb_ep_disable(dev->ep_in);

#if HSML_VERSION_12
    if (dev->ep_out)
#endif
        usb_ep_disable(dev->ep_out);

	if (atomic_read(&dev->prj2_status) != PRJ2_OFFLINE) {
		atomic_set(&dev->prj2_status, PRJ2_OFFLINE);
		schedule_work(&dev->notifier_display_work);
	}

	if (atomic_read(&dev->prj2_enable_HSML) != 0) {
		atomic_set(&dev->prj2_enable_HSML, 0);
		schedule_work(&dev->notifier_setting_work);
	}

	VDBG(dev->cdev, "%s disabled\n", dev->function.name);
}


static void
projector2_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct projector2_dev *dev = projector2_func_to_dev(f);
	struct usb_request *req;

	DBG("%s\n", __func__);

	destroy_workqueue(dev->wq_display);

	while ((req = projector2_req_get(dev, &dev->tx_idle)))
		projector2_request_free(req, dev->ep_in);

#if HSML_VERSION_12
    if (dev->ep_out)
#endif
    {
        while ((req = projector2_req_get(dev, &dev->rx_idle)))
            projector2_request_free(req, dev->ep_out);
    }

	dev->online = 0;
	dev->error = 1;
	dev->is_htcmode = 0;
#if HSML_VERSION_12
    dev->in_desc = NULL;
    dev->out_desc = NULL;
#endif

	if (atomic_read(&dev->prj2_status) != PRJ2_OFFLINE) {
		atomic_set(&dev->prj2_status, PRJ2_OFFLINE);
		schedule_work(&dev->notifier_display_work);
	}

	if (atomic_read(&dev->prj2_enable_HSML) != 0) {
		atomic_set(&dev->prj2_enable_HSML, 0);
		schedule_work(&dev->notifier_setting_work);
	}

}

static void projector2_complete_req(struct usb_ep *ep, struct usb_request *req)
{
	struct projector2_dev *dev = ep->driver_data;
	int length = req->actual;
	char *dst = NULL;

	printk(KERN_INFO "%s: status=%d, request=0x%02X\n", __func__,
			req->status, dev->hsml_proto->request);

	if (req->status != 0) {
		pr_warn("projector_complete_req, err %d\n", req->status);
		return;
	}
#if HSML_VERSION_12
	if (dev->hsml_ver == cHSML_VER_12)
	{
        switch (dev->hsml_proto->request) {
            case HSML_12_REQ_SET_PARAMETERS:
                dst = (char *) &dev->hsml_proto->set_parameters_info;
                memcpy(dst, req->buf, length);
                dev->hsml_proto->set_display_info.bPixelFormat = dev->hsml_proto->set_parameters_info.pixelFormat;

                switch (dev->hsml_proto->set_display_info.bPixelFormat) {
                    case HSML_12_PIXEL_FORMAT_ARGB888:
                        dev->bitsPixel = 32;
                        break;
                    default:
                        dev->bitsPixel = 16;
                        break;
                }
                dev->framesize = dev->hsml_proto->set_display_info.wHeight *
                                 dev->hsml_proto->set_display_info.wWidth *
                                 (dev->bitsPixel / 8);
                break;

            default:
                break;
        }
	}
	else
#endif
	{
        switch (dev->hsml_proto->request) {
            case HSML_08_REQ_SET_SERVER_CONFIGURATION:
                dst = (char *)&dev->hsml_proto->set_server_configuation_info;
                memcpy(dst, req->buf, length);
                break;

            case HSML_08_REQ_SET_SERVER_DISPLAY:
                dst = (char *)&dev->hsml_proto->set_display_info;
                memcpy(dst, req->buf, length);
                switch (dev->hsml_proto->set_display_info.bPixelFormat) {
                case PIXEL_FORMAT_ARGB888:
                    dev->bitsPixel = 32;
                    break;
                case PIXEL_FORMAT_RGB565:
                case PIXEL_FORMAT_RGB555:
                    dev->bitsPixel = 16;
                    break;
                default:
                    dev->bitsPixel = 16;
                    break;
                }
                dev->framesize = dev->hsml_proto->set_display_info.wHeight *
                                 dev->hsml_proto->set_display_info.wWidth *
                                 (dev->bitsPixel / 8);
                break;

            default:
                break;
        }
	}
}

#if HSML_VERSION_12
void vAdjustDesc(u16 hsml_ver)
{
	DBG("[HSML] %s\n", __func__);
    if (hsml_ver != cHSML_VER_12)
    {
        projector2_interface_desc.bNumEndpoints = 2;
        projector2_interface_desc.bInterfaceSubClass = 0xFF,
        projector2_interface_desc.bInterfaceProtocol = 0xFF,

        fs_projector2_descs[2] = (struct usb_descriptor_header *) &projector2_fullspeed_out_desc;
        hs_projector2_descs[2] = (struct usb_descriptor_header *) &projector2_highspeed_out_desc;

        projector2_string_defs[0].s = "HSML Server";
    }
    else
    {
        projector2_interface_desc.bNumEndpoints = 1;
        projector2_interface_desc.bInterfaceSubClass = 0xCC,
        projector2_interface_desc.bInterfaceProtocol = 0x01,

        fs_projector2_descs[2] = NULL;
        hs_projector2_descs[2] = NULL;

        projector2_string_defs[0].s = "HSML Source";
    }
}
#endif

static int projector2_ctrlrequest(struct usb_composite_dev *cdev,
											const struct usb_ctrlrequest *ctrl)
{
	int value = -EOPNOTSUPP;
	u16 w_length = le16_to_cpu(ctrl->wLength);

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR) {
		printk(KERN_INFO "%s: request(req=0x%02x, wValue=%d, "
						"wIndex=%d, wLength=%d)\n", __func__,
						ctrl->bRequest, ctrl->wValue, ctrl->wIndex, ctrl->wLength);
        switch (ctrl->bRequest) {
            case HSML_08_REQ_MIRROR_LINK:
                if (prj2_dev)
                {
#if HSML_VERSION_12
                    if (le16_to_cpu(ctrl->wValue) >= 0x0201)
                        prj2_dev->hsml_ver = cHSML_VER_12;
                    else
                        prj2_dev->hsml_ver = cHSML_VER_08;
                    vAdjustDesc(prj2_dev->hsml_ver);

                    printk(KERN_INFO "[MIRROR_LINK] %s, set state: 1 (ver=%04X,%d)\n", __func__,
                                    le16_to_cpu(ctrl->wValue), prj2_dev->hsml_ver);
#else
                    printk(KERN_INFO "[MIRROR_LINK]%s, set state: 1\n",__func__);
#endif
                    atomic_set(&prj2_dev->prj2_enable_HSML, 1);
                    schedule_work(&prj2_dev->notifier_setting_work);
                }
                value = w_length;
                break;
            default:
                printk(KERN_INFO "%s: unrecognized request(req=0x%02x, wValue=%d, "
                                    "wIndex=%d, wLength=%d)\n", __func__,
                                    ctrl->bRequest, ctrl->wValue, ctrl->wIndex, ctrl->wLength);
                break;
        }
	}
	if (value >= 0) {
		cdev->req->zero = 0;
		cdev->req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, cdev->req, GFP_ATOMIC);
		if (value < 0)
			printk(KERN_ERR "%s setup response queue error\n",__func__);
	}
	return value;
}

static int projector2_function_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
struct projector2_dev *dev = projector2_func_to_dev(f);
struct usb_composite_dev *cdev = f->config->cdev;
int value = -EOPNOTSUPP;
u16	w_length = le16_to_cpu(ctrl->wLength);
u16	w_value = le16_to_cpu(ctrl->wValue);
	u32 ret = 0;

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR) {
		char *ptr = (char *)cdev->req->buf;
		int i = 0;

		printk(KERN_INFO "%s: request(req=0x%02x, wValue=%d, "
						 "wIndex=%d, wLength=%d)\n", __func__,
						 ctrl->bRequest, ctrl->wValue, ctrl->wIndex, ctrl->wLength);
#if HSML_VERSION_12
		if (prj2_dev->hsml_ver == cHSML_VER_12)
		{
            switch (ctrl->bRequest) {
                case HSML_12_REQ_GET_VERSION:
                    ptr[0] = (char)(HSML_12_PROTOCOL_VERSION >> 8);
                    ptr[1] = (char)(HSML_12_PROTOCOL_VERSION & 0xFF);
                    value = sizeof(u16);
                    break;

                case HSML_12_REQ_GET_PARAMETERS:
                    value = sizeof(struct get_parameters);
                    memset(cdev->req->buf, 0, value);
                    dev->hsml_proto->get_parameters_info.height = dev->hsml_proto->set_display_info.wHeight;
                    dev->hsml_proto->get_parameters_info.width = dev->hsml_proto->set_display_info.wWidth;

                    ptr = (char *) &prj2_dev->hsml_proto->get_parameters_info;
                    memcpy(cdev->req->buf, ptr, value);
                    break;

                case HSML_12_REQ_SET_PARAMETERS:
                    cdev->gadget->ep0->driver_data = dev;
                    cdev->req->complete = projector2_complete_req;
                    dev->hsml_proto->request = ctrl->bRequest;
                    value = w_length;
                    break;

                case HSML_12_REQ_START_FB_TRANS:
                    if (!w_value)
                        projector2_enable_fb_work(prj2_dev, cHSML_STREAM_ON);
                    else
                    
                        projector2_enable_fb_work(prj2_dev, cHSML_ON_DEMAND);
                    value = 0;
                    break;

                case HSML_12_REQ_PAUSE_FB_TRANS:
                case HSML_12_REQ_STOP_FB_TRANS:
                    projector2_enable_fb_work(prj2_dev, cHSML_STREAM_OFF);
                    value = 0;
                    break;

                case HSML_12_REQ_SET_MAX_FRAME_RATE:
                    prj2_dev->hsml_proto->MaxFPS = w_value;
                    queue_work(dev->wq_display, &dev->notifier_display_work);
                    value = 0;
                    break;

                case HSML_12_REQ_GET_ID:
                    memset(cdev->req->buf, 0, w_length);
                        ptr = (char *) dev->aucUUID;
                        value = w_length;
                        if (value > sizeof(dev->aucUUID))
                            value = sizeof(dev->aucUUID);
                        memcpy(cdev->req->buf, ptr, value);
                    break;

                default:
                    printk(KERN_INFO "%s: unrecognized request (HSML_12) (req=0x%02x, wValue=%d, "
                                     "wIndex=%d, wLength=%d)\n", __func__,
                            ctrl->bRequest, ctrl->wValue, ctrl->wIndex, ctrl->wLength);
                    break;
            }
		}
		else
#endif
		{
            switch (ctrl->bRequest) {
                case HSML_08_REQ_GET_SERVER_VERSION:
                    ptr[0] = (char)(HSML_07_PROTOCOL_VERSION >> 8);
                    ptr[1] = (char)(HSML_07_PROTOCOL_VERSION & 0xFF);
                    value = sizeof(u16);
                    break;

                case HSML_08_REQ_NUM_COMPRESSION_SETTINGS:
                    memset(cdev->req->buf,0, w_length);
                    value = sizeof(u16);
                    break;

                case HSML_08_REQ_GET_SERVER_CONFIGURATION:
                    value = sizeof(struct get_server_configuation);
                    memset(cdev->req->buf, 0, value);
                    ptr = (char *)&prj2_dev->hsml_proto->get_server_configuation_info;
                    memcpy(cdev->req->buf, ptr, value);
                    break;

                case HSML_08_REQ_GET_FB:
                    if (!w_value)
                        projector2_enable_fb_work(prj2_dev, 1);
                    else
                        projector2_send_fb(prj2_dev);
                    value = 0;
                    break;

                case HSML_08_REQ_STOP:
                    projector2_enable_fb_work(prj2_dev, 0);
                    value = 0;
                    break;

                case HSML_08_REQ_GET_SERVER_DISPLAY: {
                int maxSize = 0;
                struct fb_info *info;

                    info = registered_fb[0];
                    if (!info) {
                        pr_warn("%s: Can not access framebuffer\n", __func__);
                    } else {
                        pr_info("device(%d, %d)\n", info->var.xres, info->var.yres);
                        maxSize = info->var.xres * info->var.yres;
                    }

        

			for (i = 0; i <= 26; i++) {
				if ((display_setting[i][0] * display_setting[i][1]) <= maxSize)
					ret |= (1 << i);
			}

                    prj2_dev->hsml_proto->get_display_capabilities_info.dwResolutionSupported = ret;
                    prj2_dev->hsml_proto->get_display_capabilities_info.dwPixelFormatSupported =
                            (1 << PIXEL_FORMAT_RGB565) | (1 << PIXEL_FORMAT_ARGB888) | (1 << PIXEL_FORMAT_RGB555);
                    ptr = (char *)&prj2_dev->hsml_proto->get_display_capabilities_info;
                    memcpy(cdev->req->buf, ptr, w_length);

                    value = w_length;
                    break;
                }
                case HSML_08_REQ_SET_SERVER_CONFIGURATION:
                case HSML_08_REQ_SET_SERVER_DISPLAY:
                    cdev->gadget->ep0->driver_data = dev;
                    cdev->req->complete = projector2_complete_req;
                    dev->hsml_proto->request = ctrl->bRequest;
                    value = w_length;
                    break;

                case HSML_08_REQ_SET_MAX_FRAME_RATE:
                    prj2_dev->hsml_proto->MaxFPS = w_value;
                    queue_work(dev->wq_display, &dev->notifier_display_work);
                    value = 0;
                    break;

                default:
                    printk(KERN_INFO "%s: unrecognized request (HSML_08) (req=0x%02x, wValue=%d, "
                                     "wIndex=%d, wLength=%d)\n", __func__,
                            ctrl->bRequest, ctrl->wValue, ctrl->wIndex, ctrl->wLength);
                    break;
            }
		}
	}

	if (value >= 0) {
		cdev->req->zero = 0;
		cdev->req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, cdev->req, GFP_ATOMIC);
		if (value < 0)
			printk(KERN_ERR "%s setup response queue error\n",
				__func__);
	}

	return value;
}

static ssize_t print_prj2_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", prj2_shortname);
}

static ssize_t print_prj2_switch_state(struct switch_dev *prj2_status_sdev, char *buf)
{
	struct projector2_dev *dev = container_of(prj2_status_sdev,
					struct projector2_dev, prj2_status_sdev);
	return sprintf(buf, "%s\n", (atomic_read(&dev->prj2_status)==PRJ2_PROJECTING?
		    "projecting" : (atomic_read(&dev->prj2_status)==PRJ2_ONLINE ? "online" : "offline")));
}

static int projector2_bind_config(struct usb_configuration *c)
{
	struct projector2_dev *dev;
	int ret = 0;

	DBG("%s\n", __func__);
	dev = prj2_dev;

	if (projector2_string_defs[0].id == 0) {
		ret = usb_string_id(c->cdev);
		if (ret < 0)
			return ret;
		projector2_string_defs[0].id = ret;
		projector2_interface_desc.iInterface = ret;
	}

	dev->cdev = c->cdev;
	dev->function.name = "projector2";
	dev->function.strings = projector2_strings;
	dev->function.descriptors = fs_projector2_descs;
	dev->function.hs_descriptors = hs_projector2_descs;
	dev->function.bind = projector2_function_bind;
	dev->function.unbind = projector2_function_unbind;
	dev->function.set_alt = projector2_function_set_alt;
	dev->function.disable = projector2_function_disable;
	dev->function.setup = projector2_function_setup;

	dev->bitsPixel = 0;
	dev->width = DEFAULT_PROJ2_WIDTH;
	dev->height = DEFAULT_PROJ2_HEIGHT;
	dev->rx_req_count = PROJ_RX_REQ_MAX;
	dev->tx_req_count = (dev->width * dev->height * 2 / TXN_MAX) + 3;
	printk(KERN_INFO "[USB] resolution: %u*%u"
		", rx_cnt: %u, tx_cnt:%u\n", dev->width, dev->height,
		dev->rx_req_count, dev->tx_req_count);

	if (projector2_touch_init(dev) < 0)
		goto err_free;

	spin_lock_init(&dev->lock);
	INIT_LIST_HEAD(&dev->rx_idle);
	INIT_LIST_HEAD(&dev->tx_idle);
	ret = usb_add_function(c, &dev->function);
	if (ret)
		goto err_free;

	dev->wq_display = create_singlethread_workqueue("projector2_mode");
	if (!dev->wq_display)
		goto err_free;

	workqueue_set_max_active(dev->wq_display,1);

	INIT_WORK(&dev->send_fb_work, projector2_send_fb_do_work);

	atomic_set(&prj2_dev->prj2_enable_HSML, 1);

	return 0;

err_free:
	printk(KERN_ERR "projector gadget driver failed to initialize, err=%d\n", ret);
	return ret;
}

static int projector2_setup(struct hsml_protocol *config)
{
struct projector2_dev *dev;
int ret = 0;
const char sig[] = {
	0xFF, 0xFF, 0x48, 0x53,
	0x4D, 0x4C, 0xFF, 0xFF
};
#if HSML_VERSION_12
u8 ucTmpUUID[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
};
#endif

	DBG("%s\n", __func__);
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	prj2_dev = dev;

	dev->hsml_proto = config;

#if HSML_VERSION_12
    dev->hsml_ver = cHSML_VER_08;
    dev->in_desc = NULL;
    dev->out_desc = NULL;

    dev->hsml_proto->get_parameters_info.capabilities = 0;
    dev->hsml_proto->get_parameters_info.pixelFormatSupported =
        (1 << HSML_12_PIXEL_FORMAT_RGB565) | (1 << HSML_12_PIXEL_FORMAT_ARGB888) | (1 << HSML_12_PIXEL_FORMAT_RGB555);
    dev->hsml_proto->get_parameters_info.height = DEFAULT_PROJ2_WIDTH;
    dev->hsml_proto->get_parameters_info.width = DEFAULT_PROJ2_HEIGHT;
    dev->hsml_proto->get_parameters_info.encodingSupported = 1;

    memset(dev->aucUUID, 0, sizeof(dev->aucUUID));
    memcpy(dev->aucUUID, ucTmpUUID, cHSML_UUID_SIZE);
#endif
	dev->hsml_proto->get_server_configuation_info.dwCapabilities = (1 << HSML_SERVER_CAP_TOUCH);
	dev->hsml_proto->get_server_configuation_info.dwTouchConfiguration = (1 << 8) | (3 << 0);

	dev->hsml_proto->set_display_info.wHeight = DEFAULT_PROJ2_HEIGHT;
	dev->hsml_proto->set_display_info.wWidth = DEFAULT_PROJ2_WIDTH;

	memcpy(&dev->header.signature, sig, sizeof(dev->header.signature));
	dev->header.seq = 0;
	dev->header.timestamp = 0;

#if HSML_VERSION_12
	memcpy(&dev->header12.signature, sig, sizeof(dev->header12.signature));
	dev->header12.seq = 0;
	dev->header12.timestamp = 0;
	memset(dev->header12.aucReserved, 0, sizeof(dev->header12.aucReserved));
#endif

	INIT_WORK(&dev->notifier_display_work, prj2_notify_display);
	INIT_WORK(&dev->notifier_setting_work, prj2_notify_setting);

	ret = switch_dev_register(&ml_switch);
	if (ret < 0) {
		pr_err("[MIRROR_LINK]fail to register mirror_link switch!\n");
		goto err_free;
	}

	dev->prj2_status_sdev.name = prj2_shortname;
	dev->prj2_status_sdev.print_name = print_prj2_switch_name;
	dev->prj2_status_sdev.print_state = print_prj2_switch_state;
	ret = switch_dev_register(&dev->prj2_status_sdev);
	if (ret < 0) {
		printk(KERN_ERR "usb prj2_status_sdev switch_dev_register register fail\n");
		goto err_free;
	}

	return 0;

err_free:
	kfree(dev);
	printk(KERN_ERR "projector gadget driver failed to initialize, err=%d\n", ret);
	return ret;

}

static void projector2_cleanup(void)
{
	struct projector2_dev *dev = prj2_dev;

	if (dev->touch_input) {
		input_unregister_device(dev->touch_input);
		input_free_device(dev->touch_input);
	}
	touch_init_p2 = 0;
	kfree(prj2_dev);
}

static ssize_t context_info_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct projector2_dev *projector2_dev = prj2_dev;
	if (size % CONTEXT_INFO_SIZE) {
		printk(KERN_ERR "%s: Array size invalid, array size should be N*28, size=%d\n",__func__,size);
			return -EINVAL;
	} else {
			if ((size / CONTEXT_INFO_SIZE) <= MAX_NUM_CONTEXT_INFO) {
			mutex_lock(&hsml_header_lock);
			memset(projector2_dev->header.context_info,0,sizeof(projector2_dev->header.context_info));
			memcpy(projector2_dev->header.context_info, buf, size);
			projector2_dev->header.num_context_info = htons(size / CONTEXT_INFO_SIZE);
			mutex_unlock(&hsml_header_lock);
		} else {
			printk(KERN_ERR "%s: N is invalid value, N=%d\n",__func__,
					size / CONTEXT_INFO_SIZE);
			return -EINVAL;
		}
	}
	return size;
}

static ssize_t projector2_ver_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
struct projector2_dev *projector2_dev = prj2_dev;

    return snprintf(buf, PAGE_SIZE, "%d\n", projector2_dev->hsml_ver);
}

static ssize_t projector2_cap_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
struct projector2_dev *projector2_dev = prj2_dev;
struct hsml_protocol *config = projector2_dev->hsml_proto;

	return snprintf(buf, PAGE_SIZE, "%d\n", config->set_parameters_info.capabilities & cHSML_12_CAP_ENDIAN);
}

static ssize_t projector2_uuid_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
struct projector2_dev *projector2_dev = prj2_dev;

    if (size <= cHSML_UUID_SIZE) {
        memset(projector2_dev->aucUUID, 0, sizeof(projector2_dev->aucUUID));
        memcpy(projector2_dev->aucUUID, buf, size);
    } else {
        printk(KERN_ERR "%s: size is invalid %d/%d\n", __func__, size, cHSML_UUID_SIZE);
        return -EINVAL;
    }
    return size;
}

