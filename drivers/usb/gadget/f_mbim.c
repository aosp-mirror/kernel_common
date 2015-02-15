/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/device.h>

#include <linux/usb/cdc.h>

#include <linux/usb/composite.h>
#include <linux/platform_device.h>

#include <linux/spinlock.h>

/*
 * This function is a "Mobile Broadband Interface Model" (MBIM) link.
 * MBIM is intended to be used with high-speed network attachments.
 *
 * Note that MBIM requires the use of "alternate settings" for its data
 * interface.  This means that the set_alt() method has real work to do,
 * and also means that a get_alt() method is required.
 */

#define MBIM_BULK_BUFFER_SIZE		4096

#define MBIM_IOCTL_MAGIC		'o'
#define MBIM_GET_NTB_SIZE		_IOR(MBIM_IOCTL_MAGIC, 2, u32)
#define MBIM_GET_DATAGRAM_COUNT		_IOR(MBIM_IOCTL_MAGIC, 3, u16)

#define NR_MBIM_PORTS			1

/* ID for Microsoft OS String */
#define MBIM_OS_STRING_ID   0xEE

struct ctrl_pkt {
	void			*buf;
	int			len;
	struct list_head	list;
};

struct mbim_ep_descs {
	struct usb_endpoint_descriptor	*in;
	struct usb_endpoint_descriptor	*out;
	struct usb_endpoint_descriptor	*notify;
};

struct mbim_notify_port {
	struct usb_ep			*notify;
	struct usb_request		*notify_req;
	u8				notify_state;
	atomic_t			notify_count;
};

enum mbim_notify_state {
	MBIM_NOTIFY_NONE,
	MBIM_NOTIFY_CONNECT,
	MBIM_NOTIFY_SPEED,
	MBIM_NOTIFY_RESPONSE_AVAILABLE,
};

struct f_mbim {
	struct usb_function		function;
	struct usb_composite_dev	*cdev;

	atomic_t	online;

	atomic_t	open_excl;
	atomic_t	ioctl_excl;
	atomic_t	read_excl;
	atomic_t	write_excl;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;

	enum transport_type		xport;
	u8				port_num;
	struct data_port		bam_port;
	struct mbim_notify_port		not_port;

	struct mbim_ep_descs		fs;
	struct mbim_ep_descs		hs;

	u8				ctrl_id, data_id;
	u8				data_alt_int;

	struct mbim_ndp_parser_opts	*parser_opts;

	spinlock_t			lock;

	struct list_head	cpkt_req_q;
	struct list_head	cpkt_resp_q;

	u32			ntb_input_size;
	u16			ntb_max_datagrams;

	atomic_t		error;
};

struct mbim_ntb_input_size {
	u32	ntb_input_size;
	u16	ntb_max_datagrams;
	u16	reserved;
};

/* temporary variable used between mbim_open() and mbim_gadget_bind() */
static struct f_mbim *_mbim_dev;

static unsigned int nr_mbim_ports;

static struct mbim_ports {
	struct f_mbim	*port;
	unsigned	port_num;
} mbim_ports[NR_MBIM_PORTS];

static inline struct f_mbim *func_to_mbim(struct usb_function *f)
{
	return container_of(f, struct f_mbim, function);
}

/* peak (theoretical) bulk transfer rate in bits-per-second */
static inline unsigned mbim_bitrate(struct usb_gadget *g)
{
	if (gadget_is_dualspeed(g) && g->speed == USB_SPEED_HIGH)
		return 13 * 512 * 8 * 1000 * 8;
	else
		return 19 *  64 * 1 * 1000 * 8;
}

/*-------------------------------------------------------------------------*/

#define MBIM_NTB_DEFAULT_IN_SIZE	(0x4000)
#define MBIM_NTB_OUT_SIZE		(0x1000)
#define MBIM_NDP_IN_DIVISOR		(0x4)

#define NTB_DEFAULT_IN_SIZE_IPA	(0x2000)
#define MBIM_NTB_OUT_SIZE_IPA		(0x2000)

#define MBIM_FORMATS_SUPPORTED	USB_CDC_NCM_NTB16_SUPPORTED

static struct usb_cdc_ncm_ntb_parameters mbim_ntb_parameters = {
	.wLength = sizeof mbim_ntb_parameters,
	.bmNtbFormatsSupported = cpu_to_le16(MBIM_FORMATS_SUPPORTED),
	.dwNtbInMaxSize = cpu_to_le32(MBIM_NTB_DEFAULT_IN_SIZE),
	.wNdpInDivisor = cpu_to_le16(MBIM_NDP_IN_DIVISOR),
	.wNdpInPayloadRemainder = cpu_to_le16(0),
	.wNdpInAlignment = cpu_to_le16(4),

	.dwNtbOutMaxSize = cpu_to_le32(MBIM_NTB_OUT_SIZE),
	.wNdpOutDivisor = cpu_to_le16(4),
	.wNdpOutPayloadRemainder = cpu_to_le16(0),
	.wNdpOutAlignment = cpu_to_le16(4),
	.wNtbOutMaxDatagrams = 0,
};

/*
 * Use wMaxPacketSize big enough to fit CDC_NOTIFY_SPEED_CHANGE in one
 * packet, to simplify cancellation; and a big transfer interval, to
 * waste less bandwidth.
 */

#define LOG2_STATUS_INTERVAL_MSEC	5	/* 1 << 5 == 32 msec */
#define NCM_STATUS_BYTECOUNT		16	/* 8 byte header + data */

static struct usb_interface_assoc_descriptor mbim_iad_desc = {
	.bLength =		sizeof mbim_iad_desc,
	.bDescriptorType =	USB_DT_INTERFACE_ASSOCIATION,

	/* .bFirstInterface =	DYNAMIC, */
	.bInterfaceCount =	2,	/* control + data */
	.bFunctionClass =	2,
	.bFunctionSubClass =	0x0e,
	.bFunctionProtocol =	0,
	/* .iFunction =		DYNAMIC */
};

/* interface descriptor: */
static struct usb_interface_descriptor mbim_control_intf = {
	.bLength =		sizeof mbim_control_intf,
	.bDescriptorType =	USB_DT_INTERFACE,

	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	1,
	.bInterfaceClass =	0x02,
	.bInterfaceSubClass =	0x0e,
	.bInterfaceProtocol =	0,
	/* .iInterface = DYNAMIC */
};

static struct usb_cdc_header_desc mbim_header_desc = {
	.bLength =		sizeof mbim_header_desc,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_HEADER_TYPE,

	.bcdCDC =		cpu_to_le16(0x0110),
};

static struct usb_cdc_union_desc mbim_union_desc = {
	.bLength =		sizeof(mbim_union_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_UNION_TYPE,
	/* .bMasterInterface0 =	DYNAMIC */
	/* .bSlaveInterface0 =	DYNAMIC */
};

static struct usb_cdc_mbb_desc mbb_desc = {
	.bLength =		sizeof mbb_desc,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_MBB_TYPE,

	.bcdMbbVersion =	cpu_to_le16(0x0100),

	.wMaxControlMessage =	cpu_to_le16(0x1000),
	.bNumberFilters =	0x20,
	.bMaxFilterSize =	0x80,
	.wMaxSegmentSize =	cpu_to_le16(0xfe0),
	.bmNetworkCapabilities = 0x20,
};

static struct usb_cdc_ext_mbb_desc ext_mbb_desc = {
	.bLength =	sizeof ext_mbb_desc,
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_EXT_MBB_TYPE,

	.bcdMbbExtendedVersion =	cpu_to_le16(0x0100),
	.bMaxOutstandingCmdMsges =	64,
	.wMTU =	1500,
};

/* the default data interface has no endpoints ... */
static struct usb_interface_descriptor mbim_data_nop_intf = {
	.bLength =		sizeof mbim_data_nop_intf,
	.bDescriptorType =	USB_DT_INTERFACE,

	/* .bInterfaceNumber = DYNAMIC */
	.bAlternateSetting =	0,
	.bNumEndpoints =	0,
	.bInterfaceClass =	0x0a,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	0x02,
	/* .iInterface = DYNAMIC */
};

/* ... but the "real" data interface has two bulk endpoints */
static struct usb_interface_descriptor mbim_data_intf = {
	.bLength =		sizeof mbim_data_intf,
	.bDescriptorType =	USB_DT_INTERFACE,

	/* .bInterfaceNumber = DYNAMIC */
	.bAlternateSetting =	1,
	.bNumEndpoints =	2,
	.bInterfaceClass =	0x0a,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	0x02,
	/* .iInterface = DYNAMIC */
};

/* full speed support: */

static struct usb_endpoint_descriptor fs_mbim_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	4*cpu_to_le16(NCM_STATUS_BYTECOUNT),
	.bInterval =		1 << LOG2_STATUS_INTERVAL_MSEC,
};

static struct usb_endpoint_descriptor fs_mbim_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fs_mbim_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *mbim_fs_function[] = {
	(struct usb_descriptor_header *) &mbim_iad_desc,
	/* MBIM control descriptors */
	(struct usb_descriptor_header *) &mbim_control_intf,
	(struct usb_descriptor_header *) &mbim_header_desc,
	(struct usb_descriptor_header *) &mbim_union_desc,
	(struct usb_descriptor_header *) &mbb_desc,
	(struct usb_descriptor_header *) &ext_mbb_desc,
	(struct usb_descriptor_header *) &fs_mbim_notify_desc,
	/* data interface, altsettings 0 and 1 */
	(struct usb_descriptor_header *) &mbim_data_nop_intf,
	(struct usb_descriptor_header *) &mbim_data_intf,
	(struct usb_descriptor_header *) &fs_mbim_in_desc,
	(struct usb_descriptor_header *) &fs_mbim_out_desc,
	NULL,
};

/* high speed support: */

static struct usb_endpoint_descriptor hs_mbim_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	4*cpu_to_le16(NCM_STATUS_BYTECOUNT),
	.bInterval =		LOG2_STATUS_INTERVAL_MSEC + 4,
};
static struct usb_endpoint_descriptor hs_mbim_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hs_mbim_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,

	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_descriptor_header *mbim_hs_function[] = {
	(struct usb_descriptor_header *) &mbim_iad_desc,
	/* MBIM control descriptors */
	(struct usb_descriptor_header *) &mbim_control_intf,
	(struct usb_descriptor_header *) &mbim_header_desc,
	(struct usb_descriptor_header *) &mbim_union_desc,
	(struct usb_descriptor_header *) &mbb_desc,
	(struct usb_descriptor_header *) &ext_mbb_desc,
	(struct usb_descriptor_header *) &hs_mbim_notify_desc,
	/* data interface, altsettings 0 and 1 */
	(struct usb_descriptor_header *) &mbim_data_nop_intf,
	(struct usb_descriptor_header *) &mbim_data_intf,
	(struct usb_descriptor_header *) &hs_mbim_in_desc,
	(struct usb_descriptor_header *) &hs_mbim_out_desc,
	NULL,
};

/* string descriptors: */

#define STRING_CTRL_IDX	0
#define STRING_DATA_IDX	1

static struct usb_string mbim_string_defs[] = {
	[STRING_CTRL_IDX].s = "MBIM Control",
	[STRING_DATA_IDX].s = "MBIM Data",
	{  } /* end of list */
};

static struct usb_gadget_strings mbim_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		mbim_string_defs,
};

static struct usb_gadget_strings *mbim_strings[] = {
	&mbim_string_table,
	NULL,
};

/* Microsoft OS Descriptors */

/*
 * We specify our own bMS_VendorCode byte which Windows will use
 * as the bRequest value in subsequent device get requests.
 */
#define MBIM_VENDOR_CODE	0xA5

/* Microsoft OS String */
static u8 mbim_os_string[] = {
	18, /* sizeof(mtp_os_string) */
	USB_DT_STRING,
	/* Signature field: "MSFT100" */
	'M', 0, 'S', 0, 'F', 0, 'T', 0, '1', 0, '0', 0, '0', 0,
	/* vendor code */
	MBIM_VENDOR_CODE,
	/* padding */
	0
};

/* Microsoft Extended Configuration Descriptor Header Section */
struct mbim_ext_config_desc_header {
	__le32	dwLength;
	__u16	bcdVersion;
	__le16	wIndex;
	__u8	bCount;
	__u8	reserved[7];
};

/* Microsoft Extended Configuration Descriptor Function Section */
struct mbim_ext_config_desc_function {
	__u8	bFirstInterfaceNumber;
	__u8	bInterfaceCount;
	__u8	compatibleID[8];
	__u8	subCompatibleID[8];
	__u8	reserved[6];
};

/* Microsoft Extended Configuration Descriptor */
static struct {
	struct mbim_ext_config_desc_header	header;
	struct mbim_ext_config_desc_function    function;
} mbim_ext_config_desc = {
	.header = {
		.dwLength = __constant_cpu_to_le32(sizeof mbim_ext_config_desc),
		.bcdVersion = __constant_cpu_to_le16(0x0100),
		.wIndex = __constant_cpu_to_le16(4),
		.bCount = 1,
	},
	.function = {
		.bFirstInterfaceNumber = 0,
		.bInterfaceCount = 1,
		.compatibleID = { 'A', 'L', 'T', 'R', 'C', 'F', 'G' },
		/* .subCompatibleID = DYNAMIC */
	},
};

/*
 * Here are options for the Datagram Pointer table (NDP) parser.
 * There are 2 different formats: NDP16 and NDP32 in the spec (ch. 3),
 * in NDP16 offsets and sizes fields are 1 16bit word wide,
 * in NDP32 -- 2 16bit words wide. Also signatures are different.
 * To make the parser code the same, put the differences in the structure,
 * and switch pointers to the structures when the format is changed.
 */

struct mbim_ndp_parser_opts {
	u32		nth_sign;
	u32		ndp_sign;
	unsigned	nth_size;
	unsigned	ndp_size;
	unsigned	ndplen_align;
	/* sizes in u16 units */
	unsigned	dgram_item_len; /* index or length */
	unsigned	block_length;
	unsigned	fp_index;
	unsigned	reserved1;
	unsigned	reserved2;
	unsigned	next_fp_index;
};

#define INIT_NDP16_OPTS {				\
	.nth_sign = USB_CDC_NCM_NTH16_SIGN,		\
	.ndp_sign = USB_CDC_NCM_NDP16_NOCRC_SIGN,	\
	.nth_size = sizeof(struct usb_cdc_ncm_nth16),	\
	.ndp_size = sizeof(struct usb_cdc_ncm_ndp16),	\
	.ndplen_align = 4,				\
	.dgram_item_len = 1,				\
	.block_length = 1,				\
	.fp_index = 1,					\
	.reserved1 = 0,					\
	.reserved2 = 0,					\
	.next_fp_index = 1,				\
}

#define INIT_NDP32_OPTS {				\
	.nth_sign = USB_CDC_NCM_NTH32_SIGN,		\
	.ndp_sign = USB_CDC_NCM_NDP32_NOCRC_SIGN,	\
	.nth_size = sizeof(struct usb_cdc_ncm_nth32),	\
	.ndp_size = sizeof(struct usb_cdc_ncm_ndp32),	\
	.ndplen_align = 8,				\
	.dgram_item_len = 2,				\
	.block_length = 2,				\
	.fp_index = 2,					\
	.reserved1 = 1,					\
	.reserved2 = 2,					\
	.next_fp_index = 2,				\
}

static struct mbim_ndp_parser_opts mbim_ndp16_opts = INIT_NDP16_OPTS;
static struct mbim_ndp_parser_opts mbim_ndp32_opts = INIT_NDP32_OPTS;

static inline int mbim_lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -EBUSY;
	}
}

static inline void mbim_unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

static struct ctrl_pkt *mbim_alloc_ctrl_pkt(unsigned len, gfp_t flags)
{
	struct ctrl_pkt *pkt;

	pkt = kzalloc(sizeof(struct ctrl_pkt), flags);
	if (!pkt)
		return ERR_PTR(-ENOMEM);

	pkt->buf = kmalloc(len, flags);
	if (!pkt->buf) {
		kfree(pkt);
		return ERR_PTR(-ENOMEM);
	}
	pkt->len = len;

	return pkt;
}

static void mbim_free_ctrl_pkt(struct ctrl_pkt *pkt)
{
	if (pkt) {
		kfree(pkt->buf);
		kfree(pkt);
	}
}

static struct usb_request *mbim_alloc_req(struct usb_ep *ep, int buffer_size)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
		return NULL;

	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}
	req->length = buffer_size;
	return req;
}

void fmbim_free_req(struct usb_ep *ep, struct usb_request *req)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static void fmbim_ctrl_response_available(struct f_mbim *dev)
{
	struct usb_request		*req = dev->not_port.notify_req;
	struct usb_cdc_notification	*event = NULL;
	unsigned long			flags;
	int				ret;

	pr_debug("dev:%p portno#%d\n", dev, dev->port_num);

	spin_lock_irqsave(&dev->lock, flags);

	if (!atomic_read(&dev->online)) {
		pr_err("dev:%p is not online\n", dev);
		spin_unlock_irqrestore(&dev->lock, flags);
		return;
	}

	if (!req) {
		pr_err("dev:%p req is NULL\n", dev);
		spin_unlock_irqrestore(&dev->lock, flags);
		return;
	}

	if (!req->buf) {
		pr_err("dev:%p req->buf is NULL\n", dev);
		spin_unlock_irqrestore(&dev->lock, flags);
		return;
	}

	if (atomic_inc_return(&dev->not_port.notify_count) != 1) {
		pr_debug("delay ep_queue: notifications queue is busy[%d]",
			atomic_read(&dev->not_port.notify_count));
		spin_unlock_irqrestore(&dev->lock, flags);
		return;
	}

	req->length = sizeof *event;
	event = req->buf;
	event->bmRequestType = USB_DIR_IN | USB_TYPE_CLASS
			| USB_RECIP_INTERFACE;
	event->bNotificationType = USB_CDC_NOTIFY_RESPONSE_AVAILABLE;
	event->wValue = cpu_to_le16(0);
	event->wIndex = cpu_to_le16(dev->ctrl_id);
	event->wLength = cpu_to_le16(0);
	spin_unlock_irqrestore(&dev->lock, flags);

	ret = usb_ep_queue(dev->not_port.notify,
			   req, GFP_ATOMIC);
	if (ret) {
		atomic_dec(&dev->not_port.notify_count);
		pr_err("ep enqueue error %d\n", ret);
	}

	pr_debug("Successful Exit");
}

static int
fmbim_send_cpkt_response(struct f_mbim *gr, struct ctrl_pkt *cpkt)
{
	struct f_mbim	*dev = gr;
	unsigned long	flags;

	if (!gr || !cpkt) {
		pr_err("Invalid cpkt, dev:%p cpkt:%p\n",
				gr, cpkt);
		return -ENODEV;
	}

	pr_debug("dev:%p port_num#%d\n", dev, dev->port_num);

	if (!atomic_read(&dev->online)) {
		pr_err("dev:%p is not connected\n", dev);
		mbim_free_ctrl_pkt(cpkt);
		return 0;
	}

	if (dev->not_port.notify_state != MBIM_NOTIFY_RESPONSE_AVAILABLE) {
		pr_err("dev:%p state=%d, recover!!\n", dev,
			dev->not_port.notify_state);
		mbim_free_ctrl_pkt(cpkt);
		return 0;
	}

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&cpkt->list, &dev->cpkt_resp_q);
	spin_unlock_irqrestore(&dev->lock, flags);

	fmbim_ctrl_response_available(dev);

	return 0;
}

/* ---------------------------- BAM INTERFACE ----------------------------- */

static int mbim_bam_setup(int no_ports)
{
	int ret;

	pr_info("no_ports:%d\n", no_ports);

	ret = bam_data_setup(no_ports);
	if (ret) {
		pr_err("bam_data_setup failed err: %d\n", ret);
		return ret;
	}

	pr_info("Initialized %d ports\n", no_ports);
	return 0;
}

int mbim_configure_params(void)
{
	struct teth_aggr_params aggr_params;
	int ret = 0;

	aggr_params.dl.aggr_prot = TETH_AGGR_PROTOCOL_MBIM;
	aggr_params.dl.max_datagrams = mbim_ntb_parameters.wNtbOutMaxDatagrams;
	aggr_params.dl.max_transfer_size_byte =
			mbim_ntb_parameters.dwNtbInMaxSize;

	aggr_params.ul.aggr_prot = TETH_AGGR_PROTOCOL_MBIM;
	aggr_params.ul.max_datagrams = mbim_ntb_parameters.wNtbOutMaxDatagrams;
	aggr_params.ul.max_transfer_size_byte =
			mbim_ntb_parameters.dwNtbOutMaxSize;

	ret = teth_bridge_set_aggr_params(&aggr_params);
	if (ret)
		pr_err("%s: teth_bridge_set_aggr_params failed\n", __func__);

	return ret;
}

static int mbim_bam_connect(struct f_mbim *dev)
{
	int ret;
	u8 src_connection_idx, dst_connection_idx;
	struct usb_gadget *gadget = dev->cdev->gadget;
	enum peer_bam bam_name = (dev->xport == USB_GADGET_XPORT_BAM2BAM_IPA) ?
							IPA_P_BAM : A2_P_BAM;

	pr_info("dev:%p portno:%d\n", dev, dev->port_num);

	src_connection_idx = usb_bam_get_connection_idx(gadget->name, bam_name,
					USB_TO_PEER_PERIPHERAL, dev->port_num);
	dst_connection_idx = usb_bam_get_connection_idx(gadget->name, bam_name,
					PEER_PERIPHERAL_TO_USB, dev->port_num);
	if (src_connection_idx < 0 || dst_connection_idx < 0) {
		pr_err("%s: usb_bam_get_connection_idx failed\n", __func__);
		return ret;
	}

	ret = bam_data_connect(&dev->bam_port, dev->port_num,
		dev->xport, src_connection_idx, dst_connection_idx,
		USB_FUNC_MBIM);

	if (ret) {
		pr_err("bam_data_setup failed: err:%d\n",
				ret);
		return ret;
	}

	pr_info("mbim bam connected\n");
	return 0;
}

static int mbim_bam_disconnect(struct f_mbim *dev)
{
	pr_info("dev:%p port:%d. Do nothing.\n",
			dev, dev->port_num);

	bam_data_disconnect(&dev->bam_port, dev->port_num);

	return 0;
}

/* -------------------------------------------------------------------------*/

static inline void mbim_reset_values(struct f_mbim *mbim)
{
	mbim->parser_opts = &mbim_ndp16_opts;

	mbim->ntb_input_size = MBIM_NTB_DEFAULT_IN_SIZE;

	atomic_set(&mbim->online, 0);
}

static void mbim_reset_function_queue(struct f_mbim *dev)
{
	struct ctrl_pkt	*cpkt = NULL;

	pr_debug("Queue empty packet for QBI");

	spin_lock(&dev->lock);

	cpkt = mbim_alloc_ctrl_pkt(0, GFP_ATOMIC);
	if (!cpkt) {
		pr_err("%s: Unable to allocate reset function pkt\n", __func__);
		spin_unlock(&dev->lock);
		return;
	}

	list_add_tail(&cpkt->list, &dev->cpkt_req_q);
	spin_unlock(&dev->lock);

	pr_debug("%s: Wake up read queue", __func__);
	wake_up(&dev->read_wq);
}

static void fmbim_reset_cmd_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_mbim		*dev = req->context;

	mbim_reset_function_queue(dev);
}

static void mbim_clear_queues(struct f_mbim *mbim)
{
	struct ctrl_pkt	*cpkt = NULL;
	struct list_head *act, *tmp;

	spin_lock(&mbim->lock);
	list_for_each_safe(act, tmp, &mbim->cpkt_req_q) {
		cpkt = list_entry(act, struct ctrl_pkt, list);
		list_del(&cpkt->list);
		mbim_free_ctrl_pkt(cpkt);
	}
	list_for_each_safe(act, tmp, &mbim->cpkt_resp_q) {
		cpkt = list_entry(act, struct ctrl_pkt, list);
		list_del(&cpkt->list);
		mbim_free_ctrl_pkt(cpkt);
	}
	spin_unlock(&mbim->lock);
}

/*
 * Context: mbim->lock held
 */
static void mbim_do_notify(struct f_mbim *mbim)
{
	struct usb_request		*req = mbim->not_port.notify_req;
	struct usb_cdc_notification	*event;
	int				status;

	pr_debug("notify_state: %d", mbim->not_port.notify_state);

	if (!req)
		return;

	event = req->buf;

	switch (mbim->not_port.notify_state) {

	case MBIM_NOTIFY_NONE:
		if (atomic_read(&mbim->not_port.notify_count) > 0)
			pr_err("Pending notifications in MBIM_NOTIFY_NONE\n");
		else
			pr_debug("No pending notifications\n");

		return;

	case MBIM_NOTIFY_RESPONSE_AVAILABLE:
		pr_debug("Notification %02x sent\n", event->bNotificationType);

		if (atomic_read(&mbim->not_port.notify_count) <= 0) {
			pr_debug("notify_response_avaliable: done");
			return;
		}

		spin_unlock(&mbim->lock);
		status = usb_ep_queue(mbim->not_port.notify, req, GFP_ATOMIC);
		spin_lock(&mbim->lock);
		if (status) {
			atomic_dec(&mbim->not_port.notify_count);
			pr_err("Queue notify request failed, err: %d", status);
		}

		return;
	}

	event->bmRequestType = 0xA1;
	event->wIndex = cpu_to_le16(mbim->ctrl_id);

	/*
	 * In double buffering if there is a space in FIFO,
	 * completion callback can be called right after the call,
	 * so unlocking
	 */
	atomic_inc(&mbim->not_port.notify_count);
	pr_debug("queue request: notify_count = %d",
		atomic_read(&mbim->not_port.notify_count));
	spin_unlock(&mbim->lock);
	status = usb_ep_queue(mbim->not_port.notify, req, GFP_ATOMIC);
	spin_lock(&mbim->lock);
	if (status) {
		atomic_dec(&mbim->not_port.notify_count);
		pr_err("usb_ep_queue failed, err: %d", status);
	}
}

static void mbim_notify_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_mbim			*mbim = req->context;
	struct usb_cdc_notification	*event = req->buf;

	pr_debug("dev:%p\n", mbim);

	spin_lock(&mbim->lock);
	switch (req->status) {
	case 0:
		atomic_dec(&mbim->not_port.notify_count);
		pr_debug("notify_count = %d",
			atomic_read(&mbim->not_port.notify_count));
		break;

	case -ECONNRESET:
	case -ESHUTDOWN:
		/* connection gone */
		mbim->not_port.notify_state = MBIM_NOTIFY_NONE;
		atomic_set(&mbim->not_port.notify_count, 0);
		pr_info("ESHUTDOWN/ECONNRESET, connection gone");
		spin_unlock(&mbim->lock);
		mbim_clear_queues(mbim);
		mbim_reset_function_queue(mbim);
		spin_lock(&mbim->lock);
		break;
	default:
		pr_err("Unknown event %02x --> %d\n",
			event->bNotificationType, req->status);
		break;
	}

	mbim_do_notify(mbim);
	spin_unlock(&mbim->lock);

	pr_debug("dev:%p Exit\n", mbim);
}

static void mbim_ep0out_complete(struct usb_ep *ep, struct usb_request *req)
{
	/* now for SET_NTB_INPUT_SIZE only */
	unsigned		in_size = 0;
	struct usb_function	*f = req->context;
	struct f_mbim		*mbim = func_to_mbim(f);
	struct mbim_ntb_input_size *ntb = NULL;

	pr_debug("dev:%p\n", mbim);

	req->context = NULL;
	if (req->status || req->actual != req->length) {
		pr_err("Bad control-OUT transfer\n");
		goto invalid;
	}

	if (req->length == 4) {
		in_size = get_unaligned_le32(req->buf);
		if (in_size < USB_CDC_NCM_NTB_MIN_IN_SIZE ||
		    in_size > le32_to_cpu(mbim_ntb_parameters.dwNtbInMaxSize)) {
			pr_err("Illegal INPUT SIZE (%d) from host\n", in_size);
			goto invalid;
		}
	} else if (req->length == 8) {
		ntb = (struct mbim_ntb_input_size *)req->buf;
		in_size = get_unaligned_le32(&(ntb->ntb_input_size));
		if (in_size < USB_CDC_NCM_NTB_MIN_IN_SIZE ||
		    in_size > le32_to_cpu(mbim_ntb_parameters.dwNtbInMaxSize)) {
			pr_err("Illegal INPUT SIZE (%d) from host\n", in_size);
			goto invalid;
		}
		mbim->ntb_max_datagrams =
			get_unaligned_le16(&(ntb->ntb_max_datagrams));
	} else {
		pr_err("Illegal NTB length %d\n", in_size);
		goto invalid;
	}

	pr_debug("Set NTB INPUT SIZE %d\n", in_size);

	mbim->ntb_input_size = in_size;
	return;

invalid:
	usb_ep_set_halt(ep);

	pr_err("dev:%p Failed\n", mbim);

	return;
}

static void
fmbim_cmd_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_mbim		*dev = req->context;
	struct ctrl_pkt		*cpkt = NULL;
	int			len = req->actual;

	if (!dev) {
		pr_err("mbim dev is null\n");
		return;
	}

	if (req->status < 0) {
		pr_err("mbim command error %d\n", req->status);
		return;
	}

	pr_debug("dev:%p port#%d\n", dev, dev->port_num);

	cpkt = mbim_alloc_ctrl_pkt(len, GFP_ATOMIC);
	if (!cpkt) {
		pr_err("Unable to allocate ctrl pkt\n");
		return;
	}

	pr_debug("Add to cpkt_req_q packet with len = %d\n", len);
	memcpy(cpkt->buf, req->buf, len);

	spin_lock(&dev->lock);

	list_add_tail(&cpkt->list, &dev->cpkt_req_q);
	spin_unlock(&dev->lock);

	/* wakeup read thread */
	pr_debug("Wake up read queue");
	wake_up(&dev->read_wq);

	return;
}

static int
mbim_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct f_mbim			*mbim = func_to_mbim(f);
	struct usb_composite_dev	*cdev = mbim->cdev;
	struct usb_request		*req = cdev->req;
	struct ctrl_pkt		*cpkt = NULL;
	int	value = -EOPNOTSUPP;
	u16	w_index = le16_to_cpu(ctrl->wIndex);
	u16	w_value = le16_to_cpu(ctrl->wValue);
	u16	w_length = le16_to_cpu(ctrl->wLength);

	/*
	 * composite driver infrastructure handles everything except
	 * CDC class messages; interface activation uses set_alt().
	 */

	if (!atomic_read(&mbim->online)) {
		pr_info("usb cable is not connected\n");
		return -ENOTCONN;
	}

	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
		| USB_CDC_RESET_FUNCTION:

		pr_debug("USB_CDC_RESET_FUNCTION");
		value = 0;
		req->complete = fmbim_reset_cmd_complete;
		req->context = mbim;
		break;

	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
		| USB_CDC_SEND_ENCAPSULATED_COMMAND:

		pr_debug("USB_CDC_SEND_ENCAPSULATED_COMMAND");

		if (w_length > req->length) {
			pr_debug("w_length > req->length: %d > %d",
			w_length, req->length);
		}
		value = w_length;
		req->complete = fmbim_cmd_complete;
		req->context = mbim;
		break;

	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
		| USB_CDC_GET_ENCAPSULATED_RESPONSE:

		pr_debug("USB_CDC_GET_ENCAPSULATED_RESPONSE");

		if (w_value) {
			pr_err("w_length > 0: %d", w_length);
			break;
		}

		pr_debug("req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);

		spin_lock(&mbim->lock);
		if (list_empty(&mbim->cpkt_resp_q)) {
			pr_err("ctrl resp queue empty\n");
			spin_unlock(&mbim->lock);
			break;
		}

		cpkt = list_first_entry(&mbim->cpkt_resp_q,
					struct ctrl_pkt, list);
		list_del(&cpkt->list);
		spin_unlock(&mbim->lock);

		value = min_t(unsigned, w_length, cpkt->len);
		memcpy(req->buf, cpkt->buf, value);
		mbim_free_ctrl_pkt(cpkt);

		pr_debug("copied encapsulated_response %d bytes",
			value);

		break;

	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
		| USB_CDC_GET_NTB_PARAMETERS:

		pr_debug("USB_CDC_GET_NTB_PARAMETERS");

		if (w_length == 0 || w_value != 0 || w_index != mbim->ctrl_id)
			break;

		value = w_length > sizeof mbim_ntb_parameters ?
			sizeof mbim_ntb_parameters : w_length;
		memcpy(req->buf, &mbim_ntb_parameters, value);
		break;

	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
		| USB_CDC_GET_NTB_INPUT_SIZE:

		pr_debug("USB_CDC_GET_NTB_INPUT_SIZE");

		if (w_length < 4 || w_value != 0 || w_index != mbim->ctrl_id)
			break;

		put_unaligned_le32(mbim->ntb_input_size, req->buf);
		value = 4;
		pr_debug("Reply to host INPUT SIZE %d\n",
		     mbim->ntb_input_size);
		break;

	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
		| USB_CDC_SET_NTB_INPUT_SIZE:

		pr_debug("USB_CDC_SET_NTB_INPUT_SIZE");

		if (w_length != 4 && w_length != 8) {
			pr_err("wrong NTB length %d", w_length);
			break;
		}

		if (w_value != 0 || w_index != mbim->ctrl_id)
			break;

		req->complete = mbim_ep0out_complete;
		req->length = w_length;
		req->context = f;

		value = req->length;
		break;

	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
		| USB_CDC_GET_NTB_FORMAT:
	{
		uint16_t format;

		pr_debug("USB_CDC_GET_NTB_FORMAT");

		if (w_length < 2 || w_value != 0 || w_index != mbim->ctrl_id)
			break;

		format = (mbim->parser_opts == &mbim_ndp16_opts) ? 0 : 1;
		put_unaligned_le16(format, req->buf);
		value = 2;
		pr_debug("NTB FORMAT: sending %d\n", format);
		break;
	}

	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
		| USB_CDC_SET_NTB_FORMAT:
	{
		pr_debug("USB_CDC_SET_NTB_FORMAT");

		if (w_length != 0 || w_index != mbim->ctrl_id)
			break;
		switch (w_value) {
		case 0x0000:
			mbim->parser_opts = &mbim_ndp16_opts;
			pr_debug("NCM16 selected\n");
			break;
		case 0x0001:
			mbim->parser_opts = &mbim_ndp32_opts;
			pr_debug("NCM32 selected\n");
			break;
		default:
			break;
		}
		value = 0;
		break;
	}

	/* optional in mbim descriptor: */
	/* case USB_CDC_GET_MAX_DATAGRAM_SIZE: */
	/* case USB_CDC_SET_MAX_DATAGRAM_SIZE: */

	default:
	pr_err("invalid control req: %02x.%02x v%04x i%04x l%d\n",
		ctrl->bRequestType, ctrl->bRequest,
		w_value, w_index, w_length);
	}

	 /* respond with data transfer or status phase? */
	if (value >= 0) {
		pr_debug("control request: %02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
		req->zero = (value < w_length);
		req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0) {
			pr_err("queueing req failed: %02x.%02x, err %d\n",
				ctrl->bRequestType,
			       ctrl->bRequest, value);
		}
	} else {
		pr_err("ctrl req err %d: %02x.%02x v%04x i%04x l%d\n",
			value, ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
	}

	/* device either stalls (value < 0) or reports success */
	return value;
}

/*
 * This function handles the Microsoft-specific OS descriptor control
 * requests that are issued by Windows host drivers to determine the
 * configuration containing the MBIM function.
 *
 * Unlike mbim_setup() this function handles two specific device requests,
 * and only when a configuration has not yet been selected.
 */
static int mbim_ctrlrequest(struct usb_composite_dev *cdev,
			    const struct usb_ctrlrequest *ctrl)
{
	int	value = -EOPNOTSUPP;
	u16	w_index = le16_to_cpu(ctrl->wIndex);
	u16	w_value = le16_to_cpu(ctrl->wValue);
	u16	w_length = le16_to_cpu(ctrl->wLength);

	/* only respond to OS desciptors when no configuration selected */
	if (cdev->config || !mbim_ext_config_desc.function.subCompatibleID[0])
		return value;

	pr_debug("%02x.%02x v%04x i%04x l%u",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);

	/* Handle MSFT OS string */
	if (ctrl->bRequestType ==
			(USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE)
			&& ctrl->bRequest == USB_REQ_GET_DESCRIPTOR
			&& (w_value >> 8) == USB_DT_STRING
			&& (w_value & 0xFF) == MBIM_OS_STRING_ID) {

		value = (w_length < sizeof(mbim_os_string) ?
				w_length : sizeof(mbim_os_string));
		memcpy(cdev->req->buf, mbim_os_string, value);

	} else if (ctrl->bRequestType ==
			(USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE)
			&& ctrl->bRequest == MBIM_VENDOR_CODE && w_index == 4) {

		/* Handle Extended OS descriptor */
		value = (w_length < sizeof(mbim_ext_config_desc) ?
				w_length : sizeof(mbim_ext_config_desc));
		memcpy(cdev->req->buf, &mbim_ext_config_desc, value);
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		int rc;
		cdev->req->zero = value < w_length;
		cdev->req->length = value;
		rc = usb_ep_queue(cdev->gadget->ep0, cdev->req, GFP_ATOMIC);
		if (rc < 0)
			pr_err("response queue error: %d", rc);
	}
	return value;
}

static int mbim_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_mbim		*mbim = func_to_mbim(f);
	struct usb_composite_dev *cdev = mbim->cdev;
	int ret = 0;

	/* Control interface has only altsetting 0 */
	if (intf == mbim->ctrl_id) {

		pr_info("CONTROL_INTERFACE");

		if (alt != 0)
			goto fail;

		if (mbim->not_port.notify->driver_data) {
			pr_info("reset mbim control %d\n", intf);
			usb_ep_disable(mbim->not_port.notify);
		}

		ret = config_ep_by_speed(cdev->gadget, f,
					mbim->not_port.notify);
		if (ret) {
			mbim->not_port.notify->desc = NULL;
			pr_err("Failed configuring notify ep %s: err %d\n",
				mbim->not_port.notify->name, ret);
			return ret;
		}

		ret = usb_ep_enable(mbim->not_port.notify);
		if (ret) {
			pr_err("usb ep#%s enable failed, err#%d\n",
				mbim->not_port.notify->name, ret);
			return ret;
		}
		mbim->not_port.notify->driver_data = mbim;

	/* Data interface has two altsettings, 0 and 1 */
	} else if (intf == mbim->data_id) {

		pr_info("DATA_INTERFACE");

		if (alt > 1)
			goto fail;

		if (mbim->bam_port.in->driver_data) {
			pr_info("reset mbim\n");
			mbim_reset_values(mbim);
		}

		/*
		 * CDC Network only sends data in non-default altsettings.
		 * Changing altsettings resets filters, statistics, etc.
		 */
		if (alt == 1) {
			pr_info("Alt set 1, initialize ports");

			if (!mbim->bam_port.in->desc) {

				pr_info("Choose endpoints");

				ret = config_ep_by_speed(cdev->gadget, f,
							mbim->bam_port.in);
				if (ret) {
					mbim->bam_port.in->desc = NULL;
					pr_err("IN ep %s failed: %d\n",
					mbim->bam_port.in->name, ret);
					return ret;
				}

				pr_info("Set mbim port in_desc = 0x%p",
					mbim->bam_port.in->desc);

				ret = config_ep_by_speed(cdev->gadget, f,
							mbim->bam_port.out);
				if (ret) {
					mbim->bam_port.out->desc = NULL;
					pr_err("OUT ep %s failed: %d\n",
					mbim->bam_port.out->name, ret);
					return ret;
				}

				pr_info("Set mbim port out_desc = 0x%p",
					mbim->bam_port.out->desc);

				pr_debug("Activate mbim\n");
				mbim_bam_connect(mbim);

			} else {
				pr_info("PORTS already SET");
			}
		}

		mbim->data_alt_int = alt;
		spin_lock(&mbim->lock);
		mbim->not_port.notify_state = MBIM_NOTIFY_RESPONSE_AVAILABLE;
		spin_unlock(&mbim->lock);
	} else {
		goto fail;
	}

	atomic_set(&mbim->online, 1);

	pr_info("SET DEVICE ONLINE");

	/* wakeup file threads */
	wake_up(&mbim->read_wq);
	wake_up(&mbim->write_wq);

	return 0;

fail:
	pr_err("ERROR: Illegal Interface");
	return -EINVAL;
}

/*
 * Because the data interface supports multiple altsettings,
 * this MBIM function *MUST* implement a get_alt() method.
 */
static int mbim_get_alt(struct usb_function *f, unsigned intf)
{
	struct f_mbim	*mbim = func_to_mbim(f);

	if (intf == mbim->ctrl_id)
		return 0;
	else if (intf == mbim->data_id)
		return mbim->data_alt_int;

	return -EINVAL;
}

static void mbim_disable(struct usb_function *f)
{
	struct f_mbim	*mbim = func_to_mbim(f);

	pr_info("SET DEVICE OFFLINE");
	atomic_set(&mbim->online, 0);

	mbim->not_port.notify_state = MBIM_NOTIFY_NONE;

	mbim_clear_queues(mbim);
	mbim_reset_function_queue(mbim);

	mbim_bam_disconnect(mbim);

	if (mbim->not_port.notify->driver_data) {
		usb_ep_disable(mbim->not_port.notify);
		mbim->not_port.notify->driver_data = NULL;
	}

	atomic_set(&mbim->not_port.notify_count, 0);

	pr_info("mbim deactivated\n");
}

#define MBIM_ACTIVE_PORT	0

static void mbim_suspend(struct usb_function *f)
{
	pr_info("mbim suspended\n");
	bam_data_suspend(MBIM_ACTIVE_PORT);
}

static void mbim_resume(struct usb_function *f)
{
	pr_info("mbim resumed\n");
	bam_data_resume(MBIM_ACTIVE_PORT);
}

/*---------------------- function driver setup/binding ---------------------*/

static int
mbim_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_mbim		*mbim = func_to_mbim(f);
	int			status;
	struct usb_ep		*ep;

	pr_info("Enter");

	mbim->cdev = cdev;

	/* allocate instance-specific interface IDs */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	mbim->ctrl_id = status;
	mbim_iad_desc.bFirstInterface = status;

	mbim_control_intf.bInterfaceNumber = status;
	mbim_union_desc.bMasterInterface0 = status;

	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	mbim->data_id = status;
	mbim->data_alt_int = 0;

	mbim_data_nop_intf.bInterfaceNumber = status;
	mbim_data_intf.bInterfaceNumber = status;
	mbim_union_desc.bSlaveInterface0 = status;

	mbim->bam_port.cdev = cdev;
	mbim->bam_port.func = &mbim->function;

	status = -ENODEV;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &fs_mbim_in_desc);
	if (!ep) {
		pr_err("usb epin autoconfig failed\n");
		goto fail;
	}
	pr_info("usb epin autoconfig succeeded\n");
	ep->driver_data = cdev;	/* claim */
	mbim->bam_port.in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, &fs_mbim_out_desc);
	if (!ep) {
		pr_err("usb epout autoconfig failed\n");
		goto fail;
	}
	pr_info("usb epout autoconfig succeeded\n");
	ep->driver_data = cdev;	/* claim */
	mbim->bam_port.out = ep;

	ep = usb_ep_autoconfig(cdev->gadget, &fs_mbim_notify_desc);
	if (!ep) {
		pr_err("usb notify ep autoconfig failed\n");
		goto fail;
	}
	pr_info("usb notify ep autoconfig succeeded\n");
	mbim->not_port.notify = ep;
	ep->driver_data = cdev;	/* claim */

	status = -ENOMEM;

	/* allocate notification request and buffer */
	mbim->not_port.notify_req = mbim_alloc_req(ep, NCM_STATUS_BYTECOUNT);
	if (!mbim->not_port.notify_req) {
		pr_info("failed to allocate notify request\n");
		goto fail;
	}
	pr_info("allocated notify ep request & request buffer\n");

	mbim->not_port.notify_req->context = mbim;
	mbim->not_port.notify_req->complete = mbim_notify_complete;

	if (mbim->xport == USB_GADGET_XPORT_BAM2BAM_IPA)
		mbb_desc.wMaxSegmentSize = cpu_to_le16(0x800);
	else
		mbb_desc.wMaxSegmentSize = cpu_to_le16(0xfe0);

	/* copy descriptors, and track endpoint copies */
	f->descriptors = usb_copy_descriptors(mbim_fs_function);
	if (!f->descriptors)
		goto fail;

	/*
	 * support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		hs_mbim_in_desc.bEndpointAddress =
				fs_mbim_in_desc.bEndpointAddress;
		hs_mbim_out_desc.bEndpointAddress =
				fs_mbim_out_desc.bEndpointAddress;
		hs_mbim_notify_desc.bEndpointAddress =
				fs_mbim_notify_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(mbim_hs_function);
		if (!f->hs_descriptors)
			goto fail;
	}

	/*
	 * If MBIM is bound in a config other than the first, tell Windows
	 * about it by returning the num as a string in the OS descriptor's
	 * subCompatibleID field. Windows only supports up to config #4.
	 */
	if (c->bConfigurationValue >= 2 && c->bConfigurationValue <= 4) {
		pr_debug("MBIM in configuration %d", c->bConfigurationValue);
		mbim_ext_config_desc.function.subCompatibleID[0] =
			c->bConfigurationValue + '0';
	}

	pr_info("mbim(%d): %s speed IN/%s OUT/%s NOTIFY/%s\n",
			mbim->port_num,
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			mbim->bam_port.in->name, mbim->bam_port.out->name,
			mbim->not_port.notify->name);

	return 0;

fail:
	pr_err("%s failed to bind, err %d\n", f->name, status);

	if (f->descriptors)
		usb_free_descriptors(f->descriptors);

	if (mbim->not_port.notify_req) {
		kfree(mbim->not_port.notify_req->buf);
		usb_ep_free_request(mbim->not_port.notify,
				    mbim->not_port.notify_req);
	}

	/* we might as well release our claims on endpoints */
	if (mbim->not_port.notify)
		mbim->not_port.notify->driver_data = NULL;
	if (mbim->bam_port.out)
		mbim->bam_port.out->driver_data = NULL;
	if (mbim->bam_port.in)
		mbim->bam_port.in->driver_data = NULL;

	return status;
}

static void mbim_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_mbim	*mbim = func_to_mbim(f);

	bam_data_destroy(mbim->port_num);
	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	usb_free_descriptors(f->descriptors);

	kfree(mbim->not_port.notify_req->buf);
	usb_ep_free_request(mbim->not_port.notify, mbim->not_port.notify_req);

	mbim_ext_config_desc.function.subCompatibleID[0] = 0;
}

/**
 * mbim_bind_config - add MBIM link to a configuration
 * @c: the configuration to support the network link
 * Context: single threaded during gadget setup
 * Returns zero on success, else negative errno.
 */
int mbim_bind_config(struct usb_configuration *c, unsigned portno,
					 char *xport_name)
{
	struct f_mbim	*mbim = NULL;
	int status = 0;

	pr_info("port number %u", portno);

	if (portno >= nr_mbim_ports) {
		pr_err("Can not add port %u. Max ports = %d",
		       portno, nr_mbim_ports);
		return -ENODEV;
	}

	status = mbim_bam_setup(nr_mbim_ports);
	if (status) {
		pr_err("bam setup failed");
		return status;
	}

	/* maybe allocate device-global string IDs */
	if (mbim_string_defs[0].id == 0) {

		/* control interface label */
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		mbim_string_defs[STRING_CTRL_IDX].id = status;
		mbim_control_intf.iInterface = status;

		/* data interface label */
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		mbim_string_defs[STRING_DATA_IDX].id = status;
		mbim_data_nop_intf.iInterface = status;
		mbim_data_intf.iInterface = status;
	}

	/* allocate and initialize one new instance */
	mbim = mbim_ports[0].port;
	if (!mbim) {
		pr_info("mbim struct not allocated");
		return -ENOMEM;
	}

	mbim->cdev = c->cdev;

	mbim_reset_values(mbim);

	mbim->function.name = "usb_mbim";
	mbim->function.strings = mbim_strings;
	mbim->function.bind = mbim_bind;
	mbim->function.unbind = mbim_unbind;
	mbim->function.set_alt = mbim_set_alt;
	mbim->function.get_alt = mbim_get_alt;
	mbim->function.setup = mbim_setup;
	mbim->function.disable = mbim_disable;
	mbim->function.suspend = mbim_suspend;
	mbim->function.resume = mbim_resume;
	mbim->xport = str_to_xport(xport_name);

	if (mbim->xport != USB_GADGET_XPORT_BAM2BAM_IPA) {
		/* Use BAM2BAM by default if not IPA */
		mbim->xport = USB_GADGET_XPORT_BAM2BAM;
	} else  {
		/* For IPA we use limit of 16 */
		mbim_ntb_parameters.wNtbOutMaxDatagrams = 16;
		/* For IPA this is proven to give maximum throughput */
		mbim_ntb_parameters.dwNtbInMaxSize =
		cpu_to_le32(NTB_DEFAULT_IN_SIZE_IPA);
		mbim_ntb_parameters.dwNtbOutMaxSize =
				cpu_to_le32(MBIM_NTB_OUT_SIZE_IPA);
		mbim_ntb_parameters.wNdpInDivisor = 1;
	}

	INIT_LIST_HEAD(&mbim->cpkt_req_q);
	INIT_LIST_HEAD(&mbim->cpkt_resp_q);

	status = usb_add_function(c, &mbim->function);

	pr_info("Exit status %d", status);

	return status;
}

/* ------------ MBIM DRIVER File Operations API for USER SPACE ------------ */

static ssize_t
mbim_read(struct file *fp, char __user *buf, size_t count, loff_t *pos)
{
	struct f_mbim *dev = fp->private_data;
	struct ctrl_pkt *cpkt = NULL;
	unsigned long	flags;
	int ret = 0;

	pr_debug("Enter(%d)\n", count);

	if (!dev) {
		pr_err("Received NULL mbim pointer\n");
		return -ENODEV;
	}

	if (count > MBIM_BULK_BUFFER_SIZE) {
		pr_err("Buffer size is too big %d, should be at most %d\n",
			count, MBIM_BULK_BUFFER_SIZE);
		return -EINVAL;
	}

	if (mbim_lock(&dev->read_excl)) {
		pr_err("Previous reading is not finished yet\n");
		return -EBUSY;
	}

	/* block until mbim online */
	while (!(atomic_read(&dev->online) || atomic_read(&dev->error))) {
		pr_err("USB cable not connected. Wait.\n");
		ret = wait_event_interruptible(dev->read_wq,
			(atomic_read(&dev->online) ||
			atomic_read(&dev->error)));
		if (ret < 0) {
			mbim_unlock(&dev->read_excl);
			return -ERESTARTSYS;
		}
	}

	if (atomic_read(&dev->error)) {
		mbim_unlock(&dev->read_excl);
		return -EIO;
	}

	spin_lock_irqsave(&dev->lock, flags);
	while (list_empty(&dev->cpkt_req_q)) {
		pr_debug("Requests list is empty. Wait.\n");
		spin_unlock_irqrestore(&dev->lock, flags);
		ret = wait_event_interruptible(dev->read_wq,
			!list_empty(&dev->cpkt_req_q));
		if (ret < 0) {
			pr_err("Waiting failed\n");
			mbim_unlock(&dev->read_excl);
			return -ERESTARTSYS;
		}
		pr_debug("Received request packet\n");
		spin_lock_irqsave(&dev->lock, flags);
	}

	cpkt = list_first_entry(&dev->cpkt_req_q, struct ctrl_pkt,
							list);
	if (cpkt->len > count) {
		spin_unlock_irqrestore(&dev->lock, flags);
		mbim_unlock(&dev->read_excl);
		pr_err("cpkt size too big:%d > buf size:%d\n",
				cpkt->len, count);
		return -ENOMEM;
	}

	pr_debug("cpkt size:%d\n", cpkt->len);

	list_del(&cpkt->list);
	spin_unlock_irqrestore(&dev->lock, flags);
	mbim_unlock(&dev->read_excl);

	ret = copy_to_user(buf, cpkt->buf, cpkt->len);
	if (ret) {
		pr_err("copy_to_user failed: err %d\n", ret);
		ret = -ENOMEM;
	} else {
		pr_debug("copied %d bytes to user\n", cpkt->len);
		ret = cpkt->len;
	}

	mbim_free_ctrl_pkt(cpkt);

	return ret;
}

static ssize_t
mbim_write(struct file *fp, const char __user *buf, size_t count, loff_t *pos)
{
	struct f_mbim *dev = fp->private_data;
	struct ctrl_pkt *cpkt = NULL;
	int ret = 0;

	pr_debug("Enter(%d)", count);

	if (!dev) {
		pr_err("Received NULL mbim pointer\n");
		return -ENODEV;
	}

	if (!count) {
		pr_err("zero length ctrl pkt\n");
		return -ENODEV;
	}

	if (count > MAX_CTRL_PKT_SIZE) {
		pr_err("given pkt size too big:%d > max_pkt_size:%d\n",
				count, MAX_CTRL_PKT_SIZE);
		return -ENOMEM;
	}

	if (mbim_lock(&dev->write_excl)) {
		pr_err("Previous writing not finished yet\n");
		return -EBUSY;
	}

	if (!atomic_read(&dev->online)) {
		pr_err("USB cable not connected\n");
		mbim_unlock(&dev->write_excl);
		return -EPIPE;
	}

	cpkt = mbim_alloc_ctrl_pkt(count, GFP_KERNEL);
	if (!cpkt) {
		pr_err("failed to allocate ctrl pkt\n");
		mbim_unlock(&dev->write_excl);
		return -ENOMEM;
	}

	ret = copy_from_user(cpkt->buf, buf, count);
	if (ret) {
		pr_err("copy_from_user failed err:%d\n", ret);
		mbim_free_ctrl_pkt(cpkt);
		mbim_unlock(&dev->write_excl);
		return 0;
	}

	fmbim_send_cpkt_response(dev, cpkt);

	mbim_unlock(&dev->write_excl);

	pr_debug("Exit(%d)", count);

	return count;

}

static int mbim_open(struct inode *ip, struct file *fp)
{
	pr_info("Open mbim driver\n");

	while (!_mbim_dev) {
		pr_err("mbim_dev not created yet\n");
		return -ENODEV;
	}

	if (mbim_lock(&_mbim_dev->open_excl)) {
		pr_err("Already opened\n");
		return -EBUSY;
	}

	pr_info("Lock mbim_dev->open_excl for open\n");

	if (!atomic_read(&_mbim_dev->online))
		pr_err("USB cable not connected\n");

	fp->private_data = _mbim_dev;

	atomic_set(&_mbim_dev->error, 0);

	pr_info("Exit, mbim file opened\n");

	return 0;
}

static int mbim_release(struct inode *ip, struct file *fp)
{
	pr_info("Close mbim file");

	mbim_unlock(&_mbim_dev->open_excl);

	return 0;
}

static long mbim_ioctl(struct file *fp, unsigned cmd, unsigned long arg)
{
	struct f_mbim *mbim = fp->private_data;
	int ret = 0;

	pr_debug("Received command %d", cmd);

	if (mbim_lock(&mbim->ioctl_excl))
		return -EBUSY;

	switch (cmd) {
	case MBIM_GET_NTB_SIZE:
		ret = copy_to_user((void __user *)arg,
			&mbim->ntb_input_size, sizeof(mbim->ntb_input_size));
		if (ret) {
			pr_err("copying to user space failed");
			ret = -EFAULT;
		}
		pr_info("Sent NTB size %d", mbim->ntb_input_size);
		break;
	case MBIM_GET_DATAGRAM_COUNT:
		ret = copy_to_user((void __user *)arg,
			&mbim->ntb_max_datagrams,
			sizeof(mbim->ntb_max_datagrams));
		if (ret) {
			pr_err("copying to user space failed");
			ret = -EFAULT;
		}
		pr_info("Sent NTB datagrams count %d",
			mbim->ntb_max_datagrams);
		break;
	default:
		pr_err("wrong parameter");
		ret = -EINVAL;
	}

	mbim_unlock(&mbim->ioctl_excl);

	return ret;
}

/* file operations for MBIM device /dev/android_mbim */
static const struct file_operations mbim_fops = {
	.owner = THIS_MODULE,
	.open = mbim_open,
	.release = mbim_release,
	.read = mbim_read,
	.write = mbim_write,
	.unlocked_ioctl	= mbim_ioctl,
};

static struct miscdevice mbim_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_mbim",
	.fops = &mbim_fops,
};

static int mbim_init(int instances)
{
	int i;
	struct f_mbim *dev = NULL;
	int ret;

	pr_info("initialize %d instances\n", instances);

	if (instances > NR_MBIM_PORTS) {
		pr_err("Max-%d instances supported\n", NR_MBIM_PORTS);
		return -EINVAL;
	}

	for (i = 0; i < instances; i++) {
		dev = kzalloc(sizeof(struct f_mbim), GFP_KERNEL);
		if (!dev) {
			pr_err("Failed to allocate mbim dev\n");
			ret = -ENOMEM;
			goto fail_probe;
		}

		dev->port_num = i;
		spin_lock_init(&dev->lock);
		INIT_LIST_HEAD(&dev->cpkt_req_q);
		INIT_LIST_HEAD(&dev->cpkt_resp_q);

		mbim_ports[i].port = dev;
		mbim_ports[i].port_num = i;

		init_waitqueue_head(&dev->read_wq);
		init_waitqueue_head(&dev->write_wq);

		atomic_set(&dev->open_excl, 0);
		atomic_set(&dev->ioctl_excl, 0);
		atomic_set(&dev->read_excl, 0);
		atomic_set(&dev->write_excl, 0);

		nr_mbim_ports++;

	}

	_mbim_dev = dev;
	ret = misc_register(&mbim_device);
	if (ret) {
		pr_err("mbim driver failed to register");
		goto fail_probe;
	}

	pr_info("Initialized %d ports\n", nr_mbim_ports);

	return ret;

fail_probe:
	pr_err("Failed");
	for (i = 0; i < nr_mbim_ports; i++) {
		kfree(mbim_ports[i].port);
		mbim_ports[i].port = NULL;
	}

	return ret;
}

static void fmbim_cleanup(void)
{
	int i = 0;

	pr_info("Enter");

	for (i = 0; i < nr_mbim_ports; i++) {
		kfree(mbim_ports[i].port);
		mbim_ports[i].port = NULL;
	}
	nr_mbim_ports = 0;

	misc_deregister(&mbim_device);

	_mbim_dev = NULL;
}

