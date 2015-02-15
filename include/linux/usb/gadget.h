/*
 * <linux/usb/gadget.h>
 *
 * We call the USB code inside a Linux-based peripheral device a "gadget"
 * driver, except for the hardware-specific bus glue.  One USB host can
 * master many USB gadgets, but the gadgets are only slaved to one host.
 *
 *
 * (C) Copyright 2002-2004 by David Brownell
 * All Rights Reserved.
 *
 * This software is licensed under the GNU GPL version 2.
 */

#ifndef __LINUX_USB_GADGET_H
#define __LINUX_USB_GADGET_H

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/types.h>
#include <linux/usb/ch9.h>

struct usb_ep;

/**
 * struct usb_request - describes one i/o request
 * @buf: Buffer used for data.  Always provide this; some controllers
 *	only use PIO, or don't use DMA for some endpoints.
 * @dma: DMA address corresponding to 'buf'.  If you don't set this
 *	field, and the usb controller needs one, it is responsible
 *	for mapping and unmapping the buffer.
 * @sg: a scatterlist for SG-capable controllers.
 * @num_sgs: number of SG entries
 * @num_mapped_sgs: number of SG entries mapped to DMA (internal)
 * @length: Length of that data
 * @stream_id: The stream id, when USB3.0 bulk streams are being used
 * @no_interrupt: If true, hints that no completion irq is needed.
 *	Helpful sometimes with deep request queues that are handled
 *	directly by DMA controllers.
 * @zero: If true, when writing data, makes the last packet be "short"
 *     by adding a zero length packet as needed;
 * @short_not_ok: When reading data, makes short packets be
 *     treated as errors (queue stops advancing till cleanup).
 * @complete: Function called when request completes, so this request and
 *	its buffer may be re-used.  The function will always be called with
 *	interrupts disabled, and it must not sleep.
 *	Reads terminate with a short packet, or when the buffer fills,
 *	whichever comes first.  When writes terminate, some data bytes
 *	will usually still be in flight (often in a hardware fifo).
 *	Errors (for reads or writes) stop the queue from advancing
 *	until the completion function returns, so that any transfers
 *	invalidated by the error may first be dequeued.
 * @context: For use by the completion callback
 * @list: For use by the gadget driver.
 * @status: Reports completion code, zero or a negative errno.
 *	Normally, faults block the transfer queue from advancing until
 *	the completion callback returns.
 *	Code "-ESHUTDOWN" indicates completion caused by device disconnect,
 *	or when the driver disabled the endpoint.
 * @actual: Reports bytes transferred to/from the buffer.  For reads (OUT
 *	transfers) this may be less than the requested length.  If the
 *	short_not_ok flag is set, short reads are treated as errors
 *	even when status otherwise indicates successful completion.
 *	Note that for writes (IN transfers) some data bytes may still
 *	reside in a device-side FIFO when the request is reported as
 *	complete.
 *@udc_priv: Vendor private data in usage by the UDC.
 *
 * These are allocated/freed through the endpoint they're used with.  The
 * hardware's driver can add extra per-request data to the memory it returns,
 * which often avoids separate memory allocations (potential failures),
 * later when the request is queued.
 *
 * Request flags affect request handling, such as whether a zero length
 * packet is written (the "zero" flag), whether a short read should be
 * treated as an error (blocking request queue advance, the "short_not_ok"
 * flag), or hinting that an interrupt is not required (the "no_interrupt"
 * flag, for use with deep request queues).
 *
 * Bulk endpoints can use any size buffers, and can also be used for interrupt
 * transfers. interrupt-only endpoints can be much less functional.
 *
 * NOTE:  this is analogous to 'struct urb' on the host side, except that
 * it's thinner and promotes more pre-allocation.
 */

struct usb_request {
	void			*buf;
	unsigned		length;
	dma_addr_t		dma;

	struct scatterlist	*sg;
	unsigned		num_sgs;
	unsigned		num_mapped_sgs;

	unsigned		stream_id:16;
	unsigned		no_interrupt:1;
	unsigned		zero:1;
	unsigned		short_not_ok:1;

	void			(*complete)(struct usb_ep *ep,
					struct usb_request *req);
	void			*context;
	struct list_head	list;

	int			status;
	unsigned		actual;
	unsigned		udc_priv;
};


struct usb_ep_ops {
	int (*enable) (struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc);
	int (*disable) (struct usb_ep *ep);

	struct usb_request *(*alloc_request) (struct usb_ep *ep,
		gfp_t gfp_flags);
	void (*free_request) (struct usb_ep *ep, struct usb_request *req);
	int (*queue) (struct usb_ep *ep, struct usb_request *req,
		gfp_t gfp_flags);
	int (*dequeue) (struct usb_ep *ep, struct usb_request *req);

	int (*set_halt) (struct usb_ep *ep, int value);
	int (*set_wedge) (struct usb_ep *ep);

	int (*fifo_status) (struct usb_ep *ep);
	void (*fifo_flush) (struct usb_ep *ep);
	void (*nuke) (struct usb_ep *ep);
};

struct usb_ep {
	void			*driver_data;
	bool			is_ncm;
	const char		*name;
	const struct usb_ep_ops	*ops;
	struct list_head	ep_list;
	unsigned		maxpacket:16;
	unsigned		max_streams:16;
	unsigned		mult:2;
	unsigned		maxburst:5;
	u8			address;
	const struct usb_endpoint_descriptor	*desc;
	const struct usb_ss_ep_comp_descriptor	*comp_desc;
};


static inline int usb_ep_enable(struct usb_ep *ep)
{
	return ep->ops->enable(ep, ep->desc);
}

static inline int usb_ep_disable(struct usb_ep *ep)
{
	return ep->ops->disable(ep);
}

static inline struct usb_request *usb_ep_alloc_request(struct usb_ep *ep,
						       gfp_t gfp_flags)
{
	return ep->ops->alloc_request(ep, gfp_flags);
}

static inline void usb_ep_free_request(struct usb_ep *ep,
				       struct usb_request *req)
{
	ep->ops->free_request(ep, req);
}

/**
 * usb_ep_queue - queues (submits) an I/O request to an endpoint.
 * @ep:the endpoint associated with the request
 * @req:the request being submitted
 * @gfp_flags: GFP_* flags to use in case the lower level driver couldn't
 *	pre-allocate all necessary memory with the request.
 *
 * This tells the device controller to perform the specified request through
 * that endpoint (reading or writing a buffer).  When the request completes,
 * including being canceled by usb_ep_dequeue(), the request's completion
 * routine is called to return the request to the driver.  Any endpoint
 * (except control endpoints like ep0) may have more than one transfer
 * request queued; they complete in FIFO order.  Once a gadget driver
 * submits a request, that request may not be examined or modified until it
 * is given back to that driver through the completion callback.
 *
 * Each request is turned into one or more packets.  The controller driver
 * never merges adjacent requests into the same packet.  OUT transfers
 * will sometimes use data that's already buffered in the hardware.
 * Drivers can rely on the fact that the first byte of the request's buffer
 * always corresponds to the first byte of some USB packet, for both
 * IN and OUT transfers.
 *
 * Bulk endpoints can queue any amount of data; the transfer is packetized
 * automatically.  The last packet will be short if the request doesn't fill it
 * out completely.  Zero length packets (ZLPs) should be avoided in portable
 * protocols since not all usb hardware can successfully handle zero length
 * packets.  (ZLPs may be explicitly written, and may be implicitly written if
 * the request 'zero' flag is set.)  Bulk endpoints may also be used
 * for interrupt transfers; but the reverse is not true, and some endpoints
 * won't support every interrupt transfer.  (Such as 768 byte packets.)
 *
 * Interrupt-only endpoints are less functional than bulk endpoints, for
 * example by not supporting queueing or not handling buffers that are
 * larger than the endpoint's maxpacket size.  They may also treat data
 * toggle differently.
 *
 * Control endpoints ... after getting a setup() callback, the driver queues
 * one response (even if it would be zero length).  That enables the
 * status ack, after transferring data as specified in the response.  Setup
 * functions may return negative error codes to generate protocol stalls.
 * (Note that some USB device controllers disallow protocol stall responses
 * in some cases.)  When control responses are deferred (the response is
 * written after the setup callback returns), then usb_ep_set_halt() may be
 * used on ep0 to trigger protocol stalls.  Depending on the controller,
 * it may not be possible to trigger a status-stage protocol stall when the
 * data stage is over, that is, from within the response's completion
 * routine.
 *
 * For periodic endpoints, like interrupt or isochronous ones, the usb host
 * arranges to poll once per interval, and the gadget driver usually will
 * have queued some data to transfer at that time.
 *
 * Returns zero, or a negative error code.  Endpoints that are not enabled
 * report errors; errors will also be
 * reported when the usb peripheral is disconnected.
 */
static inline int usb_ep_queue(struct usb_ep *ep,
			       struct usb_request *req, gfp_t gfp_flags)
{
	return ep->ops->queue(ep, req, gfp_flags);
}

static inline int usb_ep_dequeue(struct usb_ep *ep, struct usb_request *req)
{
	return ep->ops->dequeue(ep, req);
}

static inline int usb_ep_set_halt(struct usb_ep *ep)
{
	return ep->ops->set_halt(ep, 1);
}

static inline int usb_ep_clear_halt(struct usb_ep *ep)
{
	return ep->ops->set_halt(ep, 0);
}

static inline int
usb_ep_set_wedge(struct usb_ep *ep)
{
	if (ep->ops->set_wedge)
		return ep->ops->set_wedge(ep);
	else
		return ep->ops->set_halt(ep, 1);
}

/**
 * usb_ep_fifo_status - returns number of bytes in fifo, or error
 * @ep: the endpoint whose fifo status is being checked.
 *
 * FIFO endpoints may have "unclaimed data" in them in certain cases,
 * such as after aborted transfers.  Hosts may not have collected all
 * the IN data written by the gadget driver (and reported by a request
 * completion).  The gadget driver may not have collected all the data
 * written OUT to it by the host.  Drivers that need precise handling for
 * fault reporting or recovery may need to use this call.
 *
 * This returns the number of such bytes in the fifo, or a negative
 * errno if the endpoint doesn't use a FIFO or doesn't support such
 * precise handling.
 */
static inline int usb_ep_fifo_status(struct usb_ep *ep)
{
	if (ep->ops->fifo_status)
		return ep->ops->fifo_status(ep);
	else
		return -EOPNOTSUPP;
}

static inline void usb_ep_fifo_flush(struct usb_ep *ep)
{
	if (ep->ops->fifo_flush)
		ep->ops->fifo_flush(ep);
}

static inline void usb_ep_nuke(struct usb_ep *ep)
{
	if (ep->ops->nuke)
		ep->ops->nuke(ep);
}


struct usb_dcd_config_params {
	__u8  bU1devExitLat;	
#define USB_DEFAULT_U1_DEV_EXIT_LAT	0x01	
	__le16 bU2DevExitLat;	
#define USB_DEFAULT_U2_DEV_EXIT_LAT	0x1F4	
};


struct usb_gadget;
struct usb_gadget_driver;

struct usb_gadget_ops {
	int	(*get_frame)(struct usb_gadget *);
	int	(*wakeup)(struct usb_gadget *);
	int	(*set_selfpowered) (struct usb_gadget *, int is_selfpowered);
	int	(*vbus_session) (struct usb_gadget *, int is_active);
	int	(*vbus_draw) (struct usb_gadget *, unsigned mA);
	int	(*pullup) (struct usb_gadget *, int is_on);
	int	(*ioctl)(struct usb_gadget *,
				unsigned code, unsigned long param);
	void	(*get_config_params)(struct usb_dcd_config_params *);
	int	(*udc_start)(struct usb_gadget *,
			struct usb_gadget_driver *);
	int	(*udc_stop)(struct usb_gadget *,
			struct usb_gadget_driver *);

	
	int	(*start)(struct usb_gadget_driver *,
			int (*bind)(struct usb_gadget *));
	int	(*stop)(struct usb_gadget_driver *);
};

struct usb_gadget {
	
	const struct usb_gadget_ops	*ops;
	struct usb_ep			*ep0;
	struct list_head		ep_list;	
	enum usb_device_speed		speed;
	enum usb_device_speed		max_speed;
	unsigned			sg_supported:1;
	unsigned			is_otg:1;
	unsigned			is_a_peripheral:1;
	unsigned			b_hnp_enable:1;
	unsigned			a_hnp_support:1;
	unsigned			a_alt_hnp_support:1;
	unsigned			host_request:1;
	unsigned			otg_srp_reqd:1;
	const char			*name;
	struct device			dev;
	u8				usb_core_id;
	int             miMaxMtu;
	bool				l1_supported;
	bool				streaming_enabled;
	unsigned		ats_reset_irq_count;
	u32				xfer_isr_count;
};

static inline void set_gadget_data(struct usb_gadget *gadget, void *data)
	{ dev_set_drvdata(&gadget->dev, data); }
static inline void *get_gadget_data(struct usb_gadget *gadget)
	{ return dev_get_drvdata(&gadget->dev); }
static inline struct usb_gadget *dev_to_usb_gadget(struct device *dev)
{
	return container_of(dev, struct usb_gadget, dev);
}

#define gadget_for_each_ep(tmp, gadget) \
	list_for_each_entry(tmp, &(gadget)->ep_list, ep_list)


static inline int gadget_is_dualspeed(struct usb_gadget *g)
{
	return g->max_speed >= USB_SPEED_HIGH;
}

static inline int gadget_is_superspeed(struct usb_gadget *g)
{
	return g->max_speed >= USB_SPEED_SUPER;
}

static inline int gadget_is_otg(struct usb_gadget *g)
{
#ifdef CONFIG_USB_OTG
	return g->is_otg;
#else
	return 0;
#endif
}

static inline int usb_gadget_frame_number(struct usb_gadget *gadget)
{
	return gadget->ops->get_frame(gadget);
}

static inline int usb_gadget_wakeup(struct usb_gadget *gadget)
{
	if (!gadget->ops->wakeup)
		return -EOPNOTSUPP;
	return gadget->ops->wakeup(gadget);
}

static inline int usb_gadget_set_selfpowered(struct usb_gadget *gadget)
{
	if (!gadget->ops->set_selfpowered)
		return -EOPNOTSUPP;
	return gadget->ops->set_selfpowered(gadget, 1);
}

static inline int usb_gadget_clear_selfpowered(struct usb_gadget *gadget)
{
	if (!gadget->ops->set_selfpowered)
		return -EOPNOTSUPP;
	return gadget->ops->set_selfpowered(gadget, 0);
}

static inline int usb_gadget_vbus_connect(struct usb_gadget *gadget)
{
	if (!gadget->ops->vbus_session)
		return -EOPNOTSUPP;
	return gadget->ops->vbus_session(gadget, 1);
}

static inline int usb_gadget_vbus_draw(struct usb_gadget *gadget, unsigned mA)
{
	if (!gadget->ops->vbus_draw)
		return -EOPNOTSUPP;
	return gadget->ops->vbus_draw(gadget, mA);
}

static inline int usb_gadget_vbus_disconnect(struct usb_gadget *gadget)
{
	if (!gadget->ops->vbus_session)
		return -EOPNOTSUPP;
	return gadget->ops->vbus_session(gadget, 0);
}

static inline int usb_gadget_connect(struct usb_gadget *gadget)
{
	if (!gadget->ops->pullup)
		return -EOPNOTSUPP;
	return gadget->ops->pullup(gadget, 1);
}

static inline int usb_gadget_disconnect(struct usb_gadget *gadget)
{
	if (!gadget->ops->pullup)
		return -EOPNOTSUPP;
	return gadget->ops->pullup(gadget, 0);
}



struct usb_gadget_driver {
	char			*function;
	enum usb_device_speed	max_speed;
	void			(*unbind)(struct usb_gadget *);
	int			(*setup)(struct usb_gadget *,
					const struct usb_ctrlrequest *);
	void			(*disconnect)(struct usb_gadget *);
	void			(*mute_disconnect)(struct usb_gadget *);
	void			(*suspend)(struct usb_gadget *);
	void			(*resume)(struct usb_gadget *);
	void			(*broadcast_abnormal_usb_reset)(void);
	
	struct device_driver	driver;

	u8			usb_core_id;
};





int usb_gadget_probe_driver(struct usb_gadget_driver *driver,
		int (*bind)(struct usb_gadget *));

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver);

extern int usb_add_gadget_udc(struct device *parent, struct usb_gadget *gadget);
extern void usb_del_gadget_udc(struct usb_gadget *gadget);



struct usb_string {
	u8			id;
	const char		*s;
};

struct usb_gadget_strings {
	u16			language;	
	struct usb_string	*strings;
};

int usb_gadget_get_string(struct usb_gadget_strings *table, int id, u8 *buf);



int
usb_find_descriptor_fillbuf(void *, unsigned,
		const struct usb_descriptor_header **, u8);

int usb_descriptor_fillbuf(void *, unsigned,
		const struct usb_descriptor_header **);

int usb_gadget_config_buf(const struct usb_config_descriptor *config,
	void *buf, unsigned buflen, const struct usb_descriptor_header **desc);

struct usb_descriptor_header **usb_copy_descriptors(
		struct usb_descriptor_header **);

static inline void usb_free_descriptors(struct usb_descriptor_header **v)
{
	kfree(v);
}



extern int usb_gadget_map_request(struct usb_gadget *gadget,
		struct usb_request *req, int is_in);

extern void usb_gadget_unmap_request(struct usb_gadget *gadget,
		struct usb_request *req, int is_in);



extern struct usb_ep *usb_ep_autoconfig(struct usb_gadget *,
			struct usb_endpoint_descriptor *);


extern struct usb_ep *usb_ep_autoconfig_ss(struct usb_gadget *,
			struct usb_endpoint_descriptor *,
			struct usb_ss_ep_comp_descriptor *);

extern void usb_ep_autoconfig_reset(struct usb_gadget *);

#endif 
