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
#include <linux/minifb.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/htc_mode_server.h>
#include <linux/random.h>

unsigned short *test_frame;
static void setup_usb_denied(int htc_mode);

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


#define TXN_MAX 16384
#define RXN_MAX 4096

#define PROJ_RX_REQ_MAX 4

#ifdef DUMMY_DISPLAY_MODE_320_480
#define DEFAULT_PROJ_WIDTH			320
#define DEFAULT_PROJ_HEIGHT			480
#define TOUCH_WIDTH					320
#define TOUCH_HEIGHT				480
#else
#define DEFAULT_PROJ_WIDTH			480
#define DEFAULT_PROJ_HEIGHT			800
#define TOUCH_WIDTH					480
#define TOUCH_HEIGHT				800
#endif

#define BITSPIXEL 16
#define PROJECTOR_FUNCTION_NAME "projector"
#define FRAME_INTERVAL_TIME 200

#define htc_mode_info(fmt, args...) \
	printk(KERN_INFO "[htc_mode] " pr_fmt(fmt), ## args)

static int keypad_code[] = {KEY_WAKEUP, 0, 0, 0, KEY_HOME, 0, KEY_BACK};
static const char cand_shortname[] = "htc_cand";
static const char htcmode_shortname[] = "htcmode";
static ktime_t start;
static int touch_init = 0;
static int keypad_init = 0;

extern int htc_battery_set_max_input_current(int target_ma);

struct projector_dev {
	struct usb_function function;
	struct usb_composite_dev *cdev;
	spinlock_t lock;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;

	struct usb_endpoint_descriptor	*in;
	struct usb_endpoint_descriptor	*out;

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
	struct input_dev *keypad_input;
	struct input_dev *touch_input;
	char *fbaddr;

	atomic_t cand_online;
	struct switch_dev cand_sdev;
	struct switch_dev htcmode_sdev;
	struct work_struct notifier_work;
	struct work_struct htcmode_notifier_work;

	struct workqueue_struct *wq_display;
	struct work_struct send_fb_work;
	struct work_struct send_fb_work_legacy;
	int start_send_fb;

	
	struct htcmode_protocol *htcmode_proto;
	u8 is_htcmode;
	struct hsml_header header;
	u8 notify_authenticator;
};

static struct usb_interface_descriptor projector_interface_desc = {
	.bLength                = USB_DT_INTERFACE_SIZE,
	.bDescriptorType        = USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 2,
	.bInterfaceClass        = 0xFF,
	.bInterfaceSubClass     = 0xFF,
	.bInterfaceProtocol     = 0xFF,
};

static struct usb_endpoint_descriptor projector_highspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor projector_highspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor projector_fullspeed_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor projector_fullspeed_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_projector_descs[] = {
	(struct usb_descriptor_header *) &projector_interface_desc,
	(struct usb_descriptor_header *) &projector_fullspeed_in_desc,
	(struct usb_descriptor_header *) &projector_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_projector_descs[] = {
	(struct usb_descriptor_header *) &projector_interface_desc,
	(struct usb_descriptor_header *) &projector_highspeed_in_desc,
	(struct usb_descriptor_header *) &projector_highspeed_out_desc,
	NULL,
};


static struct usb_string projector_string_defs[] = {
	[0].s = "HTC PROJECTOR",
	{  } 
};

static struct usb_gadget_strings projector_string_table = {
	.language =		0x0409,	
	.strings =		projector_string_defs,
};

static struct usb_gadget_strings *projector_strings[] = {
	&projector_string_table,
	NULL,
};

static struct projector_dev *projector_dev;

struct size {
	int w;
	int h;
};

enum {
    NOT_ON_AUTOBOT,
    DOCK_ON_AUTOBOT,
    HTC_MODE_RUNNING
};
static atomic_t htc_mode_status = ATOMIC_INIT(0);

static void usb_setup_android_projector(struct work_struct *work);
static DECLARE_WORK(conf_usb_work, usb_setup_android_projector);


static void usb_setup_android_projector(struct work_struct *work)
{
    msleep(100);
    android_switch_htc_mode();
	htc_mode_enable(1);

	if (projector_dev) {
		atomic_set(&projector_dev->cand_online, 1);
		htc_mode_info("startcand %d\n",
				atomic_read(&projector_dev->cand_online));
		schedule_work(&projector_dev->notifier_work);
	}
}

static inline struct projector_dev *proj_func_to_dev(struct usb_function *f)
{
	return container_of(f, struct projector_dev, function);
}


static struct usb_request *projector_request_new(struct usb_ep *ep, int buffer_size)
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

static void projector_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static void proj_req_put(struct projector_dev *dev, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}

static struct usb_request *proj_req_get(struct projector_dev *dev, struct list_head *head)
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

static void projector_queue_out(struct projector_dev *dev)
{
	int ret;
	struct usb_request *req;

	
	while ((req = proj_req_get(dev, &dev->rx_idle))) {
		req->length = RXN_MAX;
		VDBG("%s: queue %p\n", __func__, req);
		ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
		if (ret < 0) {
			VDBG("projector: failed to queue out req (%d)\n", ret);
			dev->error = 1;
			proj_req_put(dev, &dev->rx_idle, req);
			break;
		}
	}
}

static void touch_event_func(struct projector_dev *dev,
		struct touch_content *data, int num_touch)
{
	if (num_touch > 0) {
		int i = 0;
		for (i = 0; i < num_touch; i++) {
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


static void projector_send_multitouch_event(struct projector_dev *dev,
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
	touch_event_func(dev, content, event->num_touch);
}


static void projector_send_touch_event(struct projector_dev *dev,
	int iPenType, int iX, int iY)
{
	struct input_dev *tdev = dev->touch_input;
	static int b_prePenDown = false;
	static int b_firstPenDown = true;
	static int iCal_LastX;
	static int iCal_LastY;
	static int iReportCount;

	if (iPenType != 3) {
		if (b_firstPenDown) {
			input_report_abs(tdev, ABS_X, iX);
			input_report_abs(tdev, ABS_Y, iY);
			input_report_abs(tdev, ABS_PRESSURE, 100);
			input_report_abs(tdev, ABS_TOOL_WIDTH, 1);
			input_report_key(tdev, BTN_TOUCH, 1);
			input_report_key(tdev, BTN_2, 0);
			input_sync(tdev);
			b_firstPenDown = false;
			b_prePenDown = true; 
			printk(KERN_INFO "projector: Pen down %d, %d\n", iX, iY);
		} else {
			
			if (iX != iCal_LastX || iY != iCal_LastY) {
				input_report_abs(tdev, ABS_X, iX);
				input_report_abs(tdev, ABS_Y, iY);
				input_report_abs(tdev, ABS_PRESSURE, 100);
				input_report_abs(tdev, ABS_TOOL_WIDTH, 1);
				input_report_key(tdev, BTN_TOUCH, 1);
				input_report_key(tdev, BTN_2, 0);
				input_sync(tdev);
				iReportCount++;
				if (iReportCount < 10)
					printk(KERN_INFO "projector: Pen move %d, %d\n", iX, iY);
			}
		}
	} else if (b_prePenDown) {
		input_report_abs(tdev, ABS_X, iX);
		input_report_abs(tdev, ABS_Y, iY);
		input_report_abs(tdev, ABS_PRESSURE, 0);
		input_report_abs(tdev, ABS_TOOL_WIDTH, 0);
		input_report_key(tdev, BTN_TOUCH, 0);
		input_report_key(tdev, BTN_2, 0);
		input_sync(tdev);
		printk(KERN_INFO "projector: Pen up %d, %d\n", iX, iY);
		b_prePenDown = false;
		b_firstPenDown = true;
		iReportCount = 0;
	}
	iCal_LastX = iX;
	iCal_LastY = iY;
}

static void projector_send_Key_event(struct projector_dev *dev,
	int iKeycode)
{
	struct input_dev *kdev = dev->keypad_input;
	printk(KERN_INFO "%s keycode %d\n", __func__, iKeycode);

	if (iKeycode <= 0 || iKeycode >= sizeof(keypad_code)/sizeof(keypad_code[0]))
		return;

	input_report_key(kdev, keypad_code[iKeycode], 1);
	input_sync(kdev);
	input_report_key(kdev, keypad_code[iKeycode], 0);
	input_sync(kdev);
}


static void projector_report_key_event(struct projector_dev *dev,
	struct key_event *event)
{
	struct input_dev *kdev = dev->keypad_input;
	printk(KERN_INFO "%s keycode %d, down=%d\n", __func__, event->code, event->down);

	input_report_key(kdev, event->code, event->down);
	input_sync(kdev);
}

static void send_fb(struct projector_dev *dev)
{

	struct usb_request *req;
	int xfer;
	int count = dev->framesize;
	char *frame = NULL;
	unsigned long frameSize = 0;
	int last_pkt = 0;

	if (projector_dev->htcmode_proto->debug_mode) {
		frame = (char *)test_frame;
	} else {
		if (minifb_lockbuf((void**)&frame, &frameSize, MINIFB_REPEAT) < 0) {
			pr_warn("%s: no frame\n", __func__);
			return;
		}
	}

	if (frame == NULL) {
		printk(KERN_WARNING "send_fb: frame == NULL\n");
		return;
	}
	while (count > 0 || last_pkt == 1) {
		req = proj_req_get(dev, &dev->tx_idle);
		if (req) {
			if (last_pkt == 1) {
				last_pkt = 0;
				req->length = 0;
				if (usb_ep_queue(dev->ep_in, req, GFP_ATOMIC) < 0) {
					proj_req_put(dev, &dev->tx_idle, req);
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
				proj_req_put(dev, &dev->tx_idle, req);
				printk(KERN_WARNING "%s: failed to queue req %p\n",
					__func__, req);
				break;
			}
		} else {
			printk(KERN_ERR "send_fb: no req to send\n");
			break;
		}
	}

	if (!projector_dev->htcmode_proto->debug_mode)
		minifb_unlockbuf();
}

static void send_server_info(struct projector_dev *dev);

static int send_hsml_header(struct projector_dev *dev)
{
	struct usb_request *req;
	static u16 prev_x;
	static u16 prev_y;
	static u16 prev_w;
	static u16 prev_h;
	int err;

	dev->header.x = 0;
	dev->header.y = 0;
	dev->header.w = dev->htcmode_proto->server_info.width;
	dev->header.h = dev->htcmode_proto->server_info.height;
	if ((prev_x != dev->header.x) || (prev_y != dev->header.y) ||
		(prev_w != dev->header.w) || (prev_h != dev->header.h)) {
		int i = 0;
		u8 *ptr = (u8 *)&dev->header;

		dev->header.msg_id = FB_HEADER_MSGID;
		dev->header.checksum = 0;
		for (i = 0; i < sizeof(struct hsml_header) - 1; i++)
			dev->header.checksum ^= *ptr++;

		prev_x = dev->header.x;
		prev_y = dev->header.y;
		prev_w = dev->header.w;
		prev_h = dev->header.h;
	}

	while (!(req = proj_req_get(dev, &dev->tx_idle))) {
		msleep(1);
		if (!dev->online)
			break;
	}

	if (req) {
		req->length = sizeof(struct hsml_header);
		memcpy(req->buf, &dev->header, req->length);
		err = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
		if (err < 0) {
			proj_req_put(dev, &dev->tx_idle, req);
			printk(KERN_WARNING "%s: failed to queue req"
				    " %p\n", __func__, req);
		}
	} else {
		err = -ENODEV;
	}

	return err;
}


static void send_fb2(struct projector_dev *dev)
{
	struct usb_request *req;
	int xfer;
	static char *frame,*pre_frame;
	unsigned long frameSize;
	int last_pkt = 0;
	int count = dev->htcmode_proto->server_info.width *
				dev->htcmode_proto->server_info.height * (BITSPIXEL / 8);
	ktime_t diff;

	if (projector_dev->htcmode_proto->debug_mode) {
		frame = (char *)test_frame;
	} else {
		if (minifb_lockbuf((void**)&frame, &frameSize, MINIFB_REPEAT) < 0){
			return;
		}
	}

	if (frame == NULL)
		return;



	if (frame == pre_frame && frame != (char *)test_frame) {
		diff = ktime_sub(ktime_get(), start);
		if (ktime_to_ms(diff) < FRAME_INTERVAL_TIME)
			goto unlock;
		else
			start = ktime_get();
	}

	if (dev->htcmode_proto->version >= 0x0006 &&
		send_hsml_header(dev) < 0) {
			printk(KERN_WARNING "%s: failed to send hsml header\n", __func__);
			goto unlock;
	}

	pre_frame = frame;

	while ((count > 0 || last_pkt == 1) && dev->online) {

		while (!(req = proj_req_get(dev, &dev->tx_idle))) {
			msleep(1);

			if (!dev->online)
				break;
		}

		if (req) {
			if (last_pkt == 1) {
				last_pkt = 0;
				req->length = 0;
				if (usb_ep_queue(dev->ep_in, req, GFP_ATOMIC) < 0) {
					proj_req_put(dev, &dev->tx_idle, req);
					printk(KERN_WARNING "%s: failed to queue req %p\n",
						__func__, req);
					break;
				}
				continue;
			}

			xfer = count > TXN_MAX? TXN_MAX : count;
			req->length = xfer;
			if (dev->htcmode_proto->version >= 0x0006 &&
				!dev->htcmode_proto->auth_result) {
				memset(req->buf, 0xFF, xfer);
				printk(KERN_ERR "%s: failed to authenticate\n",
									     __func__);
			} else
				memcpy(req->buf, frame, xfer);
			if (usb_ep_queue(dev->ep_in, req, GFP_ATOMIC) < 0) {
				proj_req_put(dev, &dev->tx_idle, req);
				printk(KERN_WARNING "%s: failed to queue req"
					    " %p\n", __func__, req);
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
	if (!projector_dev->htcmode_proto->debug_mode)
		minifb_unlockbuf();
}

void send_fb_do_work(struct work_struct *work)
{
	struct projector_dev *dev = projector_dev;
	start = ktime_get();
	while (dev->start_send_fb) {
		send_fb2(dev);
		msleep(1);
	}
}

void send_fb_do_work_legacy(struct work_struct *work)
{
	struct projector_dev *dev = projector_dev;

	if (!dev->online)
		return;

	send_fb(dev);
	dev->frame_count++;
	
	if (dev->frame_count == 30 * 30) {
		projector_send_Key_event(dev, 0);
		dev->frame_count = 0;
	}
}

static void send_info(struct projector_dev *dev)
{
	struct usb_request *req;

	req = proj_req_get(dev, &dev->tx_idle);
	if (req) {
		req->length = 20;
		memcpy(req->buf, "okay", 4);
		memcpy(req->buf + 4, &dev->bitsPixel, 4);
		memcpy(req->buf + 8, &dev->framesize, 4);
		memcpy(req->buf + 12, &dev->width, 4);
		memcpy(req->buf + 16, &dev->height, 4);
		if (usb_ep_queue(dev->ep_in, req, GFP_ATOMIC) < 0) {
			proj_req_put(dev, &dev->tx_idle, req);
			printk(KERN_WARNING "%s: failed to queue req %p\n",
				__func__, req);
		}
	} else
		printk(KERN_INFO "%s: no req to send\n", __func__);
}


static void send_server_info(struct projector_dev *dev)
{
	struct usb_request *req;

	req = proj_req_get(dev, &dev->tx_idle);
	if (req) {
		req->length = sizeof(struct msm_server_info);
		memcpy(req->buf, &dev->htcmode_proto->server_info, req->length);
		if (usb_ep_queue(dev->ep_in, req, GFP_ATOMIC) < 0) {
			proj_req_put(dev, &dev->tx_idle, req);
			printk(KERN_WARNING "%s: failed to queue req %p\n",
				__func__, req);
		}
	} else {
		printk(KERN_INFO "%s: no req to send\n", __func__);
	}
}

struct size rotate(struct size v)
{
	struct size r;
	r.w = v.h;
	r.h = v.w;
	return r;
}

static struct size get_projection_size(struct projector_dev *dev, struct msm_client_info *client_info)
{
	int server_width = 0;
	int server_height = 0;
	struct size client;
	struct size server;
	struct size ret;
	int perserve_aspect_ratio = client_info->display_conf & (1 << 0);
	int server_orientation = 0;
	int client_orientation = (client_info->width > client_info->height);
	int align_w = 0;

	server_width = dev->width;
	server_height = dev->height;

	server_orientation = (server_width > server_height);

	printk(KERN_INFO "%s(): perserve_aspect_ratio= %d\n", __func__, perserve_aspect_ratio);

	client.w = client_info->width;
	client.h = client_info->height;
	server.w = server_width;
	server.h = server_height;

	if (server_orientation != client_orientation)
		client = rotate(client);

	align_w = client.h * server.w > server.h * client.w;

	if (perserve_aspect_ratio) {
		if (align_w) {
			ret.w = client.w;
			ret.h = (client.w * server.h) / server.w;
		} else {
			ret.w = (client.h * server.w) / server.h;
			ret.h = client.h;
		}

		ret.w = round_down(ret.w, 32);
	} else {
		ret = client;
	}

	printk(KERN_INFO "projector size(w=%d, h=%d)\n", ret.w, ret.h);

	return ret;
}

#if 0
static void projector_get_msmfb(struct projector_dev *dev)
{
    struct msm_fb_info fb_info;

	msmfb_get_var(&fb_info);

	dev->bitsPixel = BITSPIXEL;
	dev->width = fb_info.xres;
	dev->height = fb_info.yres;
	dev->fbaddr = get_fb_addr();
	dev->framesize = dev->width * dev->height * (dev->bitsPixel / 8);
	printk(KERN_INFO "projector: width %d, height %d framesize %d, %p\n",
		   fb_info.xres, fb_info.yres, dev->framesize, dev->fbaddr);
}
#endif

static void projector_enable_fb_work(struct projector_dev *dev, int enabled)
{
	dev->start_send_fb = enabled;
	if (enabled) {
		queue_work(dev->wq_display, &dev->send_fb_work);

		if (atomic_inc_return(&htc_mode_status) != HTC_MODE_RUNNING)
			atomic_dec(&htc_mode_status);
		htc_mode_info("startfb current htc_mode_status = %d\n",
			    atomic_read(&htc_mode_status));
	} else {
		if (atomic_dec_return(&htc_mode_status) != DOCK_ON_AUTOBOT)
			atomic_inc(&htc_mode_status);
		htc_mode_info("endfb current htc_mode_status = %d\n",
			    atomic_read(&htc_mode_status));
	}
	schedule_work(&dev->htcmode_notifier_work);
}

static int projector_handle_common_msg(struct projector_dev *dev, struct usb_request *req)
{
	unsigned char *data = req->buf;
	int handled = 1;

	if (!strncmp("startfb", data, 7)) {
		projector_enable_fb_work(dev, 1);
	} else if (!strncmp("endfb", data, 5)) {
		projector_enable_fb_work(dev, 0);
	} else {
		handled = 0;
	}

	return handled;
}
static int projector_handle_htcmode_msg(struct projector_dev *dev, struct usb_request *req)
{
	unsigned char *data = req->buf;
	int handled = 1;
	struct size projector_size;

	if ((data[0] == CLIENT_INFO_MESGID) && (req->actual == sizeof(struct msm_client_info))) {
		memcpy(&dev->htcmode_proto->client_info, req->buf, sizeof(struct msm_client_info));

		projector_size = get_projection_size(dev, &dev->htcmode_proto->client_info);

		dev->htcmode_proto->server_info.mesg_id = SERVER_INFO_MESGID;
		dev->htcmode_proto->server_info.width = projector_size.w;
		dev->htcmode_proto->server_info.height = projector_size.h;
		dev->htcmode_proto->server_info.pixel_format = PIXEL_FORMAT_RGB565;
		dev->htcmode_proto->server_info.ctrl_conf = CTRL_CONF_TOUCH_EVENT_SUPPORTED |
									  CTRL_CONF_NUM_SIMULTANEOUS_TOUCH;
		send_server_info(dev);
	} else if (dev->htcmode_proto->version >= 0x0006 &&
			data[0] == HSML_TOUCH_EVENT_ID) {
		projector_send_multitouch_event(dev, data);
	} else if (dev->htcmode_proto->version >= 0x0006 &&
			data[0] == HSML_KEY_EVENT_ID) {
		projector_report_key_event(dev, (struct key_event *)data);
	} else if (!strncmp("startcand", data, 9)) {
	} else if (!strncmp("endcand", data, 7)) {
		atomic_set(&dev->cand_online, 0);
		htc_mode_info("endcand %d\n", atomic_read(&dev->cand_online));

		schedule_work(&dev->notifier_work);
	} else {
		handled = projector_handle_common_msg(dev, req);
	}

	return handled;
}

static void projector_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct projector_dev *dev = projector_dev;
	proj_req_put(dev, &dev->tx_idle, req);
}

static void projector_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct projector_dev *dev = projector_dev;
	unsigned char *data = req->buf;
	int mouse_data[3];
	int i;
	int handled = 0;
	VDBG("%s: status %d, %d bytes\n", __func__,
		req->status, req->actual);

	if (req->status != 0) {
		dev->error = 1;
		proj_req_put(dev, &dev->rx_idle, req);
		return ;
	}

	if (dev->is_htcmode)
		handled = projector_handle_htcmode_msg(dev, req);

	if (!handled)
		handled = projector_handle_common_msg(dev, req);

	if (!handled) {
		
		mouse_data[0] = *((int *)(req->buf));

		if (!strncmp("init", data, 4)) {

			dev->init_done = 1;
			dev->bitsPixel = BITSPIXEL;
			dev->width = DEFAULT_PROJ_WIDTH;
			dev->height = DEFAULT_PROJ_HEIGHT;
			dev->framesize = dev->width * dev->height * (BITSPIXEL / 8);

			send_info(dev);
			
			projector_send_Key_event(dev, 0);

			atomic_set(&htc_mode_status, HTC_MODE_RUNNING);
			htc_mode_info("init current htc_mode_status = %d\n",
			    atomic_read(&htc_mode_status));
			schedule_work(&dev->htcmode_notifier_work);
		} else if (*data == ' ') {
			queue_work(dev->wq_display, &dev->send_fb_work_legacy);
		} else if (mouse_data[0] > 0) {
			 if (mouse_data[0] < 4) {
				for (i = 0; i < 3; i++)
					mouse_data[i] = *(((int *)(req->buf))+i);
				projector_send_touch_event(dev,
					mouse_data[0], mouse_data[1], mouse_data[2]);
			} else {
				projector_send_Key_event(dev, mouse_data[0]);
				printk(KERN_INFO "projector: Key command data %02x, keycode %d\n",
					*((char *)(req->buf)), mouse_data[0]);
			}
		} else if (mouse_data[0] != 0)
			printk(KERN_ERR "projector: Unknow command data %02x, mouse %d,%d,%d\n",
				*((char *)(req->buf)), mouse_data[0], mouse_data[1], mouse_data[2]);
	}
	proj_req_put(dev, &dev->rx_idle, req);
	projector_queue_out(dev);
}

static int projector_create_bulk_endpoints(struct projector_dev *dev,
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

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if (!ep) {
		DBG("usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}
	DBG("usb_ep_autoconfig for projector ep_out got %s\n", ep->name);
	ep->driver_data = dev;		
	dev->ep_out = ep;

	
	for (i = 0; i < dev->rx_req_count; i++) {
		req = projector_request_new(dev->ep_out, RXN_MAX);
		if (!req)
			goto fail;
		req->complete = projector_complete_out;
		proj_req_put(dev, &dev->rx_idle, req);
	}
	for (i = 0; i < dev->tx_req_count; i++) {
		req = projector_request_new(dev->ep_in, TXN_MAX);
		if (!req)
			goto fail;
		req->complete = projector_complete_in;
		proj_req_put(dev, &dev->tx_idle, req);
	}

	return 0;

fail:
	while ((req = proj_req_get(dev, &dev->tx_idle)))
		projector_request_free(req, dev->ep_in);
	while ((req = proj_req_get(dev, &dev->rx_idle)))
		projector_request_free(req, dev->ep_out);
	printk(KERN_ERR "projector: could not allocate requests\n");
	return -1;
}

static int
projector_function_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct projector_dev	*dev = proj_func_to_dev(f);
	int			id;
	int			ret;

	dev->cdev = cdev;
	DBG("%s\n", __func__);

	
	id = usb_interface_id(c, f);
	if (id < 0)
		return id;

	projector_interface_desc.bInterfaceNumber = id;

	
	ret = projector_create_bulk_endpoints(dev, &projector_fullspeed_in_desc,
			&projector_fullspeed_out_desc);
	if (ret)
		return ret;

	
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		projector_highspeed_in_desc.bEndpointAddress =
			projector_fullspeed_in_desc.bEndpointAddress;
		projector_highspeed_out_desc.bEndpointAddress =
			projector_fullspeed_out_desc.bEndpointAddress;
	}

	DBG("%s speed %s: IN/%s, OUT/%s\n",
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			f->name, dev->ep_in->name, dev->ep_out->name);
	return 0;
}


static int projector_function_set_alt(struct usb_function *f,
		unsigned intf, unsigned alt)
{
	struct projector_dev *dev = proj_func_to_dev(f);
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
	dev->online = 1;

	projector_queue_out(dev);

	return 0;
}


static int projector_touch_init(struct projector_dev *dev)
{
	int x = TOUCH_WIDTH;
	int y = TOUCH_HEIGHT;
	int ret = 0;
	struct input_dev *tdev = dev->touch_input;
	if (touch_init) {
		pr_info("%s already initial\n", __func__);
		return 0;
	}

	printk(KERN_INFO "%s: x=%d y=%d\n", __func__, x, y);
	dev->touch_input  = input_allocate_device();
	if (dev->touch_input == NULL) {
		printk(KERN_ERR "%s: Failed to allocate input device\n",
			__func__);
		return -1;
	}
	tdev = dev->touch_input;
	tdev->name = "projector_input";
	set_bit(EV_SYN,    tdev->evbit);
	set_bit(EV_KEY,    tdev->evbit);
	set_bit(EV_ABS,    tdev->evbit);


	
	if (dev->htcmode_proto->version < 0x0006) {
		pr_info("%s: single-touch support\n", __func__);

		set_bit(BTN_TOUCH, tdev->keybit);
		set_bit(BTN_2,     tdev->keybit);

		input_set_abs_params(tdev, ABS_X, 0, x, 0, 0);
		input_set_abs_params(tdev, ABS_Y, 0, y, 0, 0);
		input_set_abs_params(tdev, ABS_PRESSURE, 0, 255, 0, 0);
		input_set_abs_params(tdev, ABS_TOOL_WIDTH, 0, 15, 0, 0);
		input_set_abs_params(tdev, ABS_HAT0X, 0, x, 0, 0);
		input_set_abs_params(tdev, ABS_HAT0Y, 0, y, 0, 0);
	} else {
		pr_info("%s: multi-touch support\n", __func__);
		input_set_abs_params(tdev, ABS_MT_POSITION_X, 0, x, 0, 0);
		input_set_abs_params(tdev, ABS_MT_POSITION_Y, 0, y, 0, 0);
		input_set_abs_params(tdev, ABS_MT_PRESSURE, 0, 1, 0, 0);
	}

	ret = input_register_device(tdev);
	if (ret) {
		printk(KERN_ERR "%s: Unable to register %s input device\n",
			__func__, tdev->name);
		input_free_device(tdev);
		return -1;
	}
	touch_init = 1;
	printk(KERN_INFO "%s OK \n", __func__);
	return 0;
}

static int projector_keypad_init(struct projector_dev *dev)
{
	struct input_dev *kdev;
	if (keypad_init) {
		pr_info("%s already initial\n", __func__);
		return 0;
	}
	
	dev->keypad_input = input_allocate_device();
	if (dev->keypad_input == NULL) {
		printk(KERN_ERR "%s: Failed to allocate input device\n",
			__func__);
		return -1;
	}
	kdev = dev->keypad_input;
	set_bit(EV_KEY, kdev->evbit);
	set_bit(KEY_SEND, kdev->keybit);
	set_bit(KEY_END, kdev->keybit);
	set_bit(KEY_MUTE, kdev->keybit);
	set_bit(KEY_PLAY, kdev->keybit);
	set_bit(KEY_PAUSE, kdev->keybit);
	set_bit(KEY_STOP, kdev->keybit);
	set_bit(KEY_NEXTSONG, kdev->keybit);
	set_bit(KEY_PLAYPAUSE, kdev->keybit);
	set_bit(KEY_PREVIOUSSONG, kdev->keybit);
	set_bit(KEY_FORWARD, kdev->keybit);
	set_bit(KEY_REWIND, kdev->keybit);
	set_bit(KEY_VOLUMEDOWN, kdev->keybit);
	set_bit(KEY_VOLUMEUP, kdev->keybit);
	set_bit(KEY_HOME, kdev->keybit);
	set_bit(KEY_BACK, kdev->keybit);
	set_bit(KEY_SEARCH, kdev->keybit);
	set_bit(KEY_ENTER, kdev->keybit);
	set_bit(KEY_DELETE, kdev->keybit);
	set_bit(KEY_ZOOMIN, kdev->keybit);
	set_bit(KEY_ZOOMOUT, kdev->keybit);

	set_bit(KEY_WAKEUP, kdev->keybit);

	kdev->name = "projector-Keypad";
	kdev->phys = "input2";
	kdev->id.bustype = BUS_HOST;
	kdev->id.vendor = 0x0123;
	kdev->id.product = 0x5220 ;
	kdev->id.version = 0x0100;
	kdev->keycodesize = sizeof(unsigned int);

	
	if (input_register_device(kdev) < 0) {
		printk(KERN_ERR "%s: Unable to register %s input device\n",
			__func__, kdev->name);
		input_free_device(kdev);
		return -1;
	}
	printk(KERN_INFO "%s OK \n", __func__);
	keypad_init = 1;
	return 0;
}

#if 0
static ssize_t store_enable(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int _enabled, ret;
	ret = strict_strtol(buf, 10, (unsigned long *)&_enabled);
	if (ret < 0) {
		printk(KERN_INFO "%s: %d\n", __func__, ret);
		return 0;
	}
	printk(KERN_INFO "projector: %d\n", _enabled);

	android_enable_function(&_projector_dev.function, _enabled);
	_projector_dev.enabled = _enabled;
	return count;
}

static ssize_t show_enable(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	buf[0] = '0' + _projector_dev.enabled;
	buf[1] = '\n';
	return 2;

}
static DEVICE_ATTR(enable, 0664, show_enable, store_enable);
#endif


static void cand_online_notify(struct work_struct *w)
{
	struct projector_dev *dev = container_of(w,
					struct projector_dev, notifier_work);
	DBG("%s\n", __func__);
	switch_set_state(&dev->cand_sdev, atomic_read(&dev->cand_online));
}

static void htcmode_status_notify(struct work_struct *w)
{
	struct projector_dev *dev = container_of(w,
					struct projector_dev, htcmode_notifier_work);
	DBG("%s\n", __func__);

	if (dev->htcmode_proto->notify_authenticator) {
		char *envp[] = {
				"SWITCH_NAME=htcmode",
				"SWITCH_STATE=check_client_sig", 0
		};
		kobject_uevent_env(&dev->htcmode_sdev.dev->kobj, KOBJ_CHANGE, envp);
		dev->htcmode_proto->notify_authenticator = 0;
	} else
		switch_set_state(&dev->htcmode_sdev, atomic_read(&htc_mode_status));
}

void htc_mode_enable(int enable)
{
	htc_mode_info("%s = %d, current htc_mode_status = %d\n",
			__func__, enable, atomic_read(&htc_mode_status));

	if (enable)
		atomic_set(&htc_mode_status, DOCK_ON_AUTOBOT);
	else
		atomic_set(&htc_mode_status, NOT_ON_AUTOBOT);

	htcmode_status_notify(&projector_dev->htcmode_notifier_work);
}

int check_htc_mode_status(void)
{
	return atomic_read(&htc_mode_status);
}

static ssize_t print_cand_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", cand_shortname);
}

static ssize_t print_cand_switch_state(struct switch_dev *cand_sdev, char *buf)
{
	struct projector_dev *dev = container_of(cand_sdev,
					struct projector_dev, cand_sdev);
	return sprintf(buf, "%s\n", (atomic_read(&dev->cand_online) ?
		    "online" : "offline"));
}

static ssize_t print_htcmode_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", htcmode_shortname);
}

static ssize_t print_htcmode_switch_state(struct switch_dev *htcmode_sdev, char *buf)
{
	return sprintf(buf, "%s\n", (atomic_read(&htc_mode_status) == HTC_MODE_RUNNING ?
		    "projecting" : (atomic_read(&htc_mode_status) == DOCK_ON_AUTOBOT ? "online" : "offline")));
}


static void projector_function_disable(struct usb_function *f)
{
	struct projector_dev *dev = proj_func_to_dev(f);

	DBG("%s\n", __func__);

	dev->start_send_fb = false;
	dev->online = 0;
	dev->error = 1;
	usb_ep_disable(dev->ep_in);
	usb_ep_disable(dev->ep_out);

	atomic_set(&dev->cand_online, 0);
	schedule_work(&dev->notifier_work);

	VDBG(dev->cdev, "%s disabled\n", dev->function.name);
}


static void
projector_function_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct projector_dev *dev = proj_func_to_dev(f);
	struct usb_request *req;

	DBG("%s\n", __func__);

	destroy_workqueue(dev->wq_display);

	while ((req = proj_req_get(dev, &dev->tx_idle)))
		projector_request_free(req, dev->ep_in);
	while ((req = proj_req_get(dev, &dev->rx_idle)))
		projector_request_free(req, dev->ep_out);

	dev->online = 0;
	dev->error = 1;
	dev->is_htcmode = 0;
	if (dev->htcmode_proto)
		dev->htcmode_proto->auth_in_progress = 0;

}


static int projector_bind_config(struct usb_configuration *c)
{
	struct projector_dev *dev;
	int ret = 0;

	DBG("%s\n", __func__);
	dev = projector_dev;

	if (projector_string_defs[0].id == 0) {
		ret = usb_string_id(c->cdev);
		if (ret < 0)
			return ret;
		projector_string_defs[0].id = ret;
		projector_interface_desc.iInterface = ret;
	}

	dev->cdev = c->cdev;
	dev->function.name = "projector";
	dev->function.strings = projector_strings;
	dev->function.descriptors = fs_projector_descs;
	dev->function.hs_descriptors = hs_projector_descs;
	dev->function.bind = projector_function_bind;
	dev->function.unbind = projector_function_unbind;
	dev->function.set_alt = projector_function_set_alt;
	dev->function.disable = projector_function_disable;

	dev->bitsPixel = BITSPIXEL;
	dev->width = DEFAULT_PROJ_WIDTH;
	dev->height = DEFAULT_PROJ_HEIGHT;
	dev->rx_req_count = PROJ_RX_REQ_MAX;
	dev->tx_req_count = (dev->width * dev->height * 2 / TXN_MAX) + 5;
	printk(KERN_INFO "[USB] resolution: %u*%u"
		", rx_cnt: %u, tx_cnt:%u\n", dev->width, dev->height,
		dev->rx_req_count, dev->tx_req_count);
	if (projector_touch_init(dev) < 0)
		goto err_free;
	if (projector_keypad_init(dev) < 0)
		goto err_free;

	spin_lock_init(&dev->lock);
	INIT_LIST_HEAD(&dev->rx_idle);
	INIT_LIST_HEAD(&dev->tx_idle);
	ret = usb_add_function(c, &dev->function);
	if (ret)
		goto err_free;

	dev->wq_display = create_singlethread_workqueue("projector_mode");
	if (!dev->wq_display)
		goto err_free;

	workqueue_set_max_active(dev->wq_display, 1);

	INIT_WORK(&dev->send_fb_work, send_fb_do_work);
	INIT_WORK(&dev->send_fb_work_legacy, send_fb_do_work_legacy);

	return 0;

err_free:
	printk(KERN_ERR "projector gadget driver failed to initialize, err=%d\n", ret);
	return ret;
}

static int projector_setup(struct htcmode_protocol *config)
{
	struct projector_dev *dev;
	int ret = 0;

	const char sig[] = {
		0xFF, 0xFF, 0x48, 0x53,
		0x4D, 0x4C, 0xFF, 0xFF
	};

	DBG("%s\n", __func__);
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	projector_dev = dev;

	dev->htcmode_proto = config;
	dev->htcmode_proto->server_info.height = DEFAULT_PROJ_HEIGHT;
	dev->htcmode_proto->server_info.width = DEFAULT_PROJ_WIDTH;
	dev->htcmode_proto->client_info.display_conf = 0;
	memcpy(&dev->header.signature, sig, sizeof(dev->header.signature));
	dev->htcmode_proto->notify_authenticator = 0;

	INIT_WORK(&dev->notifier_work, cand_online_notify);
	INIT_WORK(&dev->htcmode_notifier_work, htcmode_status_notify);

	dev->cand_sdev.name = cand_shortname;
	dev->cand_sdev.print_name = print_cand_switch_name;
	dev->cand_sdev.print_state = print_cand_switch_state;
	ret = switch_dev_register(&dev->cand_sdev);
	if (ret < 0) {
		printk(KERN_ERR "usb cand_sdev switch_dev_register register fail\n");
		goto err_free;
	}

	dev->htcmode_sdev.name = htcmode_shortname;
	dev->htcmode_sdev.print_name = print_htcmode_switch_name;
	dev->htcmode_sdev.print_state = print_htcmode_switch_state;
	ret = switch_dev_register(&dev->htcmode_sdev);
	if (ret < 0) {
		printk(KERN_ERR "usb htcmode_sdev switch_dev_register register fail\n");
		goto err_unregister_cand;
	}

	return 0;

err_unregister_cand:
	switch_dev_unregister(&dev->cand_sdev);
err_free:
	kfree(dev);
	printk(KERN_ERR "projector gadget driver failed to initialize, err=%d\n", ret);
	return ret;

}

static void projector_cleanup(void)
{
	struct projector_dev *dev;

	dev = projector_dev;
	touch_init = 0;
	keypad_init = 0;

	switch_dev_unregister(&dev->cand_sdev);
	switch_dev_unregister(&dev->htcmode_sdev);

	if (dev->touch_input) {
		input_unregister_device(dev->touch_input);
		input_free_device(dev->touch_input);
	}
	if (dev->keypad_input) {
		input_unregister_device(dev->keypad_input);
		input_free_device(dev->keypad_input);
	}

	kfree(dev);
}



static void projector_complete_req(struct usb_ep *ep, struct usb_request *req)
{
	struct projector_dev *dev = ep->driver_data;
	int length = req->actual;
	char *dst = NULL;

	printk(KERN_INFO "%s: status=%d, request=0x%02X\n", __func__,
			req->status, dev->htcmode_proto->request);

	if (req->status != 0) {
		pr_warn("projector_complete_req, err %d\n", req->status);
		return;
	}

	switch (dev->htcmode_proto->request) {
	case HSML_06_REQ_SEND_CLIENT_INFO:
		dst = (char *)&dev->htcmode_proto->client_info;
		memcpy(dst + 1, req->buf, length);
		break;

	case HSML_06_REQ_SEND_EXTENDED_CLIENT_INFO:
		break;

	case HSML_06_REQ_SET_CLIENT_AUTH:
		dst = (char *)&dev->htcmode_proto->client_sig;
		memcpy(dst, req->buf, length);
		dev->htcmode_proto->notify_authenticator = 1;
		schedule_work(&dev->htcmode_notifier_work);
		break;

	default:
		break;
	}
}

static int projector_ctrlrequest(struct usb_composite_dev *cdev,
				const struct usb_ctrlrequest *ctrl)
{
	int value = -EOPNOTSUPP;
	u16	w_length = le16_to_cpu(ctrl->wLength);
	u16	w_value = le16_to_cpu(ctrl->wValue);
	u16	w_index = le16_to_cpu(ctrl->wIndex);

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR) {
		char *ptr = (char *)cdev->req->buf;
		struct size projector_size;
		int i = 0;

		printk(KERN_INFO "%s: request(req=0x%02x, wValue=%d, "
						 "wIndex=%d, wLength=%d)\n", __func__,
						 ctrl->bRequest, ctrl->wValue, ctrl->wIndex, ctrl->wLength);
		if (ctrl->bRequest == HTC_MODE_CONTROL_REQ) {
			if (check_htc_mode_status() == NOT_ON_AUTOBOT) {
				if (projector_dev) {
					projector_dev->htcmode_proto->vendor = w_index;
					projector_dev->htcmode_proto->version = w_value;
					if (projector_dev->htcmode_proto->version == 0x0034)
						projector_dev->htcmode_proto->version = 0x0003;
					projector_dev->is_htcmode = 1;
					projector_dev->htcmode_proto->notify_authenticator = 0;
					printk(KERN_INFO "HTC Mode version = 0x%04X\n", projector_dev->htcmode_proto->version);
					printk(KERN_INFO "HSML Client vendor = 0x%04X\n", projector_dev->htcmode_proto->vendor);
					projector_dev->htcmode_proto->auth_in_progress = 0;
					projector_dev->htcmode_proto->auth_result = 1;
					setup_usb_denied(1);
					schedule_work(&conf_usb_work);
				} else {
					printk(KERN_ERR "%s: projector_dev is NULL!!", __func__);
				}
				value = 0;
			}
		}
		else if (check_htc_mode_status() != NOT_ON_AUTOBOT) {
			switch (ctrl->bRequest) {
			case HSML_06_REQ_GET_SERVER_VERSION:
				ptr[0] = (char)(HSML_PROTOCOL_VERSION >> 8);
				ptr[1] = (char)(HSML_PROTOCOL_VERSION & 0xFF);
				value = sizeof(u16);
				break;

			case HSML_06_REQ_GET_SERVER_INFO:
				projector_size = get_projection_size(projector_dev, &projector_dev->htcmode_proto->client_info);

				projector_dev->htcmode_proto->server_info.mesg_id = SERVER_INFO_MESGID;
				projector_dev->htcmode_proto->server_info.width = projector_size.w;
				projector_dev->htcmode_proto->server_info.height = projector_size.h;
				projector_dev->htcmode_proto->server_info.pixel_format = PIXEL_FORMAT_RGB565;
				projector_dev->htcmode_proto->server_info.ctrl_conf = CTRL_CONF_TOUCH_EVENT_SUPPORTED |
											  CTRL_CONF_NUM_SIMULTANEOUS_TOUCH;
				value = w_length;
				ptr = (char *)&projector_dev->htcmode_proto->server_info;
				memcpy(cdev->req->buf, ptr + 1, w_length);
				break;

			case HSML_06_REQ_GET_FB:
				if (!w_value)
					projector_enable_fb_work(projector_dev, 1);
				else
					send_fb(projector_dev);
				value = 0;
				break;

			case HSML_06_REQ_STOP:
				projector_enable_fb_work(projector_dev, 0);
				value = 0;
				break;

			case HSML_06_REQ_GET_SERVER_NONCE:
				if (w_length == HSML_SERVER_NONCE_SIZE) {
					for (i = 0; i < w_length / sizeof(int); i++)
						*((int *)(projector_dev->htcmode_proto->nonce + sizeof(int) * i)) = get_random_int();
					memcpy(cdev->req->buf, projector_dev->htcmode_proto->nonce, w_length);
				} else
					printk(KERN_ERR "%s: The request size for server nonce is incorrect. (w_length=%d)",
						__func__, w_length);
				value = w_length;
				projector_dev->htcmode_proto->auth_result = 0;
				projector_dev->htcmode_proto->auth_in_progress = 1;
				break;

			case HSML_06_REQ_SET_CLIENT_AUTH:
			case HSML_06_REQ_SEND_CLIENT_INFO:
			case HSML_06_REQ_SEND_EXTENDED_CLIENT_INFO:
				cdev->gadget->ep0->driver_data = projector_dev;
				cdev->req->complete = projector_complete_req;
				projector_dev->htcmode_proto->request = ctrl->bRequest;
				value = w_length;
				break;

			case HSML_06_REQ_GET_SERVER_AUTH:
				if (!projector_dev->htcmode_proto->auth_in_progress) {
					memcpy(cdev->req->buf,
							projector_dev->htcmode_proto->server_sig, w_length);
					value = w_length;
				}
				break;
			case HSML_06_REQ_SET_MAX_CHARGING_CURRENT:
				
				htc_battery_set_max_input_current((int)w_value);
				value = 0;
				break;

			default:
				printk(KERN_INFO "%s: unrecognized request(req=0x%02x, wValue=%d, "
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
