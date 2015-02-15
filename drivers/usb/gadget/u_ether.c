/*
 * u_ether.c -- Ethernet-over-USB link layer utilities for Gadget stack
 *
 * Copyright (C) 2003-2005,2008 David Brownell
 * Copyright (C) 2003-2004 Robert Schwebel, Benedikt Spranger
 * Copyright (C) 2008 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#define fcAUTO_PERF_LOCK        0

#include <linux/kernel.h>
#include <linux/gfp.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>

#include "u_ether.h"



#define UETH__VERSION	"29-May-2008"

static struct workqueue_struct	*uether_wq;

struct eth_dev {
	spinlock_t		lock;
	struct gether		*port_usb;

	struct net_device	*net;
	struct usb_gadget	*gadget;

	spinlock_t		req_lock;	
	struct list_head	tx_reqs, rx_reqs;
	unsigned		tx_qlen;
#define TX_REQ_THRESHOLD	5
	int			no_tx_req_used;
	int			tx_skb_hold_count;
	u32			tx_req_bufsize;

	struct sk_buff_head	rx_frames;

	unsigned		header_len;
	unsigned int		ul_max_pkts_per_xfer;
	unsigned int		dl_max_pkts_per_xfer;
	struct sk_buff		*(*wrap)(struct gether *, struct sk_buff *skb);
	int			(*unwrap)(struct gether *,
						struct sk_buff *skb,
						struct sk_buff_head *list);

	struct work_struct	work;
	struct work_struct	rx_work;

	unsigned long		todo;
#define	WORK_RX_MEMORY		0

	bool			zlp;
	u8			host_mac[ETH_ALEN];
	int             miMaxMtu;

#if fcAUTO_PERF_LOCK
    bool auto_perf_lock_on;
    struct work_struct release_perf_lock_work;
    struct timer_list perf_timer;
    unsigned long timer_expired;
    struct switch_dev EthPerf_sdev;
    struct switch_dev *pEthPerf_sdev;
#endif
};


#define RX_EXTRA	20	

#define DEFAULT_QLEN	2	

static unsigned qmult = 10;
module_param(qmult, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(qmult, "queue length multiplier at high/super speed");

static inline int qlen(struct usb_gadget *gadget)
{
	if (gadget_is_dualspeed(gadget) && (gadget->speed == USB_SPEED_HIGH ||
					    gadget->speed == USB_SPEED_SUPER))
		return qmult * DEFAULT_QLEN;
	else
		return DEFAULT_QLEN;
}



#ifdef DEBUG
#undef DBG
#undef VDBG
#undef ERROR
#undef INFO

#define xprintk(d, level, fmt, args...) \
	printk(level "%s: " fmt , (d)->net->name , ## args)

#ifdef DEBUG
#undef DEBUG
#define DBG(dev, fmt, args...) \
	xprintk(dev , KERN_DEBUG , fmt , ## args)
#else
#define DBG(dev, fmt, args...) \
	do { } while (0)
#endif 

#ifdef VERBOSE_DEBUG
#define VDBG	DBG
#else
#define VDBG(dev, fmt, args...) \
	do { } while (0)
#endif 

#define ERROR(dev, fmt, args...) \
	xprintk(dev , KERN_ERR , fmt , ## args)
#define INFO(dev, fmt, args...) \
	xprintk(dev , KERN_INFO , fmt , ## args)
#endif 

static struct eth_dev *the_dev;



static int ueth_change_mtu(struct net_device *net, int new_mtu)
{
	struct eth_dev	*dev = netdev_priv(net);
	unsigned long	flags;
	int		status = 0;
	int     iMaxMtuSet = ETH_FRAME_LEN;

	if (the_dev) {
        if (the_dev->miMaxMtu == ETH_FRAME_LEN_MAX - ETH_HLEN)
            iMaxMtuSet = ETH_FRAME_LEN_MAX - ETH_HLEN;
	}

	
	spin_lock_irqsave(&dev->lock, flags);
	if (dev->port_usb)
		status = -EBUSY;
	else if (new_mtu <= ETH_HLEN || new_mtu > iMaxMtuSet)
		status = -ERANGE;
	else
		net->mtu = new_mtu;
	spin_unlock_irqrestore(&dev->lock, flags);

	return status;
}

#if fcAUTO_PERF_LOCK

static const char iEthPerf_name[] = "USB_PERF";

static void eth_perf_send_event(struct eth_dev *sEthDev, int iNetOn)
{
struct eth_dev *dev = the_dev;

    printk(KERN_INFO "[PERF] %s, iNetOn=%d", __func__, iNetOn);

        if (dev->pEthPerf_sdev != NULL)
            switch_set_state(&dev->EthPerf_sdev, iNetOn);
}

#define AUTO_PERF_EXPIRED	    (10000)

static void auto_setup_perf_lock(bool auto_perf_lock_on)
{
struct eth_dev *dev = the_dev;

    
    del_timer(&dev->perf_timer);
    if (auto_perf_lock_on) {
        if (!dev->auto_perf_lock_on)
        {
            eth_perf_send_event(dev, auto_perf_lock_on);
        }
    } else {
        if (dev->auto_perf_lock_on)
        {
            eth_perf_send_event(dev, auto_perf_lock_on);
        }
    }
    dev->auto_perf_lock_on = auto_perf_lock_on;
}

static void release_perf_lock_work_func(struct work_struct *work)
{
    auto_setup_perf_lock(false);
}

static void auto_perf_lock_enable(int expire_time)
{
struct eth_dev *dev = the_dev;

    if (expire_time) {
        auto_setup_perf_lock(true);
        dev->timer_expired = AUTO_PERF_EXPIRED * expire_time;
        mod_timer(&dev->perf_timer,
            jiffies + msecs_to_jiffies(dev->timer_expired));
    }
    else
        auto_setup_perf_lock(false);
}

static void auto_perf_lock_disable(unsigned long data)
{
struct eth_dev *dev = the_dev;

    schedule_work(&dev->release_perf_lock_work);
}

static void auto_perf_timeout_handler(unsigned long data)
{
    auto_perf_lock_disable(data);
}

#endif

static void eth_get_drvinfo(struct net_device *net, struct ethtool_drvinfo *p)
{
	struct eth_dev	*dev = netdev_priv(net);

	strlcpy(p->driver, "g_ether", sizeof p->driver);
	strlcpy(p->version, UETH__VERSION, sizeof p->version);
	strlcpy(p->fw_version, dev->gadget->name, sizeof p->fw_version);
	strlcpy(p->bus_info, dev_name(&dev->gadget->dev), sizeof p->bus_info);
}


static const struct ethtool_ops ops = {
	.get_drvinfo = eth_get_drvinfo,
	.get_link = ethtool_op_get_link,
};

static void defer_kevent(struct eth_dev *dev, int flag)
{
	if (test_and_set_bit(flag, &dev->todo))
		return;
	if (!schedule_work(&dev->work))
		ERROR(dev, "kevent %d may have been dropped\n", flag);
	else
		DBG(dev, "kevent %d scheduled\n", flag);
}

static void rx_complete(struct usb_ep *ep, struct usb_request *req);
static void tx_complete(struct usb_ep *ep, struct usb_request *req);

static int
rx_submit(struct eth_dev *dev, struct usb_request *req, gfp_t gfp_flags)
{
	struct sk_buff	*skb;
	int		retval = -ENOMEM;
	size_t		size = 0;
	struct usb_ep	*out;
	unsigned long	flags;

	spin_lock_irqsave(&dev->lock, flags);
	if (dev->port_usb)
		out = dev->port_usb->out_ep;
	else
		out = NULL;
	spin_unlock_irqrestore(&dev->lock, flags);

	if (!out)
		return -ENOTCONN;


	size += sizeof(struct ethhdr) + dev->net->mtu + RX_EXTRA;
	size += dev->port_usb->header_len;
	size += out->maxpacket - 1;
	size -= size % out->maxpacket;

	if (dev->ul_max_pkts_per_xfer)
		size *= dev->ul_max_pkts_per_xfer;

	if (dev->port_usb->is_fixed)
		size = max_t(size_t, size, dev->port_usb->fixed_out_len);

	pr_debug("%s: size: %d", __func__, size);
	skb = alloc_skb(size + NET_IP_ALIGN, gfp_flags);
	if (skb == NULL) {
		DBG(dev, "no rx skb\n");
		goto enomem;
	}

	skb_reserve(skb, NET_IP_ALIGN);

	req->buf = skb->data;
	req->length = size;
	req->context = skb;

	retval = usb_ep_queue(out, req, gfp_flags);
	if (retval == -ENOMEM)
enomem:
		defer_kevent(dev, WORK_RX_MEMORY);
	if (retval) {
		DBG(dev, "rx submit --> %d\n", retval);
		if (skb)
			dev_kfree_skb_any(skb);
	}
	return retval;
}

static void rx_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct sk_buff	*skb = req->context;
	struct eth_dev	*dev = ep->driver_data;
	int		status = req->status;
	bool		queue = 0;

	switch (status) {

	
	case 0:
		skb_put(skb, req->actual);

		if (dev->unwrap) {
			unsigned long	flags;

			spin_lock_irqsave(&dev->lock, flags);
			if (dev->port_usb) {
				status = dev->unwrap(dev->port_usb,
							skb,
							&dev->rx_frames);
				if (status == -EINVAL)
					dev->net->stats.rx_errors++;
				else if (status == -EOVERFLOW)
					dev->net->stats.rx_over_errors++;
			} else {
				dev_kfree_skb_any(skb);
				status = -ENOTCONN;
			}
			spin_unlock_irqrestore(&dev->lock, flags);
		} else {
			skb_queue_tail(&dev->rx_frames, skb);
		}

		if (!status)
			queue = 1;
		break;

	
	case -ECONNRESET:		
	case -ESHUTDOWN:		
		VDBG(dev, "rx shutdown, code %d\n", status);
		goto quiesce;

	
	case -ECONNABORTED:		
		DBG(dev, "rx %s reset\n", ep->name);
		defer_kevent(dev, WORK_RX_MEMORY);
quiesce:
		if (skb)
			dev_kfree_skb_any(skb);
		goto clean;

	
	case -EOVERFLOW:
		dev->net->stats.rx_over_errors++;
		

	default:
		queue = 1;
		dev_kfree_skb_any(skb);
		dev->net->stats.rx_errors++;
		DBG(dev, "rx status %d\n", status);
		break;
	}

clean:
	spin_lock(&dev->req_lock);
	list_add(&req->list, &dev->rx_reqs);
	spin_unlock(&dev->req_lock);

	if (queue)
		queue_work(uether_wq, &dev->rx_work);
}

static int prealloc(struct list_head *list, struct usb_ep *ep, unsigned n)
{
	unsigned		i;
	struct usb_request	*req;
	bool			usb_in;

	if (!n)
		return -ENOMEM;

	
	i = n;
	list_for_each_entry(req, list, list) {
		if (i-- == 0)
			goto extra;
	}

	if (ep->desc->bEndpointAddress & USB_DIR_IN)
		usb_in = true;
	else
		usb_in = false;

	while (i--) {
		req = usb_ep_alloc_request(ep, GFP_ATOMIC);
		if (!req)
			return list_empty(list) ? -ENOMEM : 0;
		
		if (usb_in)
			req->complete = tx_complete;
		else
			req->complete = rx_complete;

		list_add(&req->list, list);
	}
	return 0;

extra:
	
	for (;;) {
		struct list_head	*next;

		next = req->list.next;
		list_del(&req->list);
		usb_ep_free_request(ep, req);

		if (next == list)
			break;

		req = container_of(next, struct usb_request, list);
	}
	return 0;
}

static int alloc_requests(struct eth_dev *dev, struct gether *link, unsigned n)
{
	int	status;

	spin_lock(&dev->req_lock);
	status = prealloc(&dev->tx_reqs, link->in_ep, n);
	if (status < 0)
		goto fail;
	status = prealloc(&dev->rx_reqs, link->out_ep, n);
	if (status < 0)
		goto fail;
	goto done;
fail:
	DBG(dev, "can't alloc requests\n");
done:
	spin_unlock(&dev->req_lock);
	return status;
}

static void rx_fill(struct eth_dev *dev, gfp_t gfp_flags)
{
	struct usb_request	*req;
	unsigned long		flags;
	int			req_cnt = 0;

	
	spin_lock_irqsave(&dev->req_lock, flags);
	while (!list_empty(&dev->rx_reqs)) {
		
		if (++req_cnt > qlen(dev->gadget))
			break;

		req = container_of(dev->rx_reqs.next,
				struct usb_request, list);
		list_del_init(&req->list);
		spin_unlock_irqrestore(&dev->req_lock, flags);

		if (rx_submit(dev, req, gfp_flags) < 0) {
			spin_lock_irqsave(&dev->req_lock, flags);
			list_add(&req->list, &dev->rx_reqs);
			spin_unlock_irqrestore(&dev->req_lock, flags);
			defer_kevent(dev, WORK_RX_MEMORY);
			return;
		}

		spin_lock_irqsave(&dev->req_lock, flags);
	}
	spin_unlock_irqrestore(&dev->req_lock, flags);
}

static void process_rx_w(struct work_struct *work)
{
	struct eth_dev	*dev = container_of(work, struct eth_dev, rx_work);
	struct sk_buff	*skb;
	int		status = 0;
	unsigned int uiCurMtu = 0;

	if (!dev->port_usb)
		return;

	uiCurMtu = dev->net->mtu + ETH_HLEN;
	if ((uiCurMtu <= ETH_HLEN) || (uiCurMtu > ETH_FRAME_LEN_MAX))
	    uiCurMtu = ETH_FRAME_LEN;

	while ((skb = skb_dequeue(&dev->rx_frames))) {
		if (status < 0
				|| ETH_HLEN > skb->len
			
			    || skb->len > uiCurMtu) {
			dev->net->stats.rx_errors++;
			dev->net->stats.rx_length_errors++;
			DBG(dev, "rx length %d\n", skb->len);
			dev_kfree_skb_any(skb);
			continue;
		}
		skb->protocol = eth_type_trans(skb, dev->net);
		dev->net->stats.rx_packets++;
		dev->net->stats.rx_bytes += skb->len;

#if fcAUTO_PERF_LOCK
        if (skb->len >= 1024)
            auto_perf_lock_enable(1);
#endif
		status = netif_rx_ni(skb);
	}

	if (netif_running(dev->net))
		rx_fill(dev, GFP_KERNEL);
}

static void eth_work(struct work_struct *work)
{
	struct eth_dev	*dev = container_of(work, struct eth_dev, work);

	if (test_and_clear_bit(WORK_RX_MEMORY, &dev->todo)) {
		if (netif_running(dev->net))
			rx_fill(dev, GFP_KERNEL);
	}

	if (dev->todo)
		DBG(dev, "work done, flags = 0x%lx\n", dev->todo);
}

static void tx_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct sk_buff	*skb;
	struct eth_dev	*dev;
	struct net_device *net;
	struct usb_request *new_req;
	struct usb_ep *in;
	int length;
	int retval;

	if (!ep->driver_data) {
		usb_ep_free_request(ep, req);
		return;
	}

	dev = ep->driver_data;
	net = dev->net;

	if (!dev->port_usb) {
		usb_ep_free_request(ep, req);
		return;
	}

	switch (req->status) {
	default:
		dev->net->stats.tx_errors++;
		VDBG(dev, "tx err %d\n", req->status);
		
	case -ECONNRESET:		
	case -ESHUTDOWN:		
		break;
	case 0:
		if (!req->zero)
			dev->net->stats.tx_bytes += req->length-1;
		else
			dev->net->stats.tx_bytes += req->length;
	}
	dev->net->stats.tx_packets++;

	spin_lock(&dev->req_lock);
	list_add_tail(&req->list, &dev->tx_reqs);

	if (dev->port_usb->multi_pkt_xfer && !req->context) {
		dev->no_tx_req_used--;
		req->length = 0;
		in = dev->port_usb->in_ep;

		if (!list_empty(&dev->tx_reqs)) {
			new_req = container_of(dev->tx_reqs.next,
					struct usb_request, list);
			list_del(&new_req->list);
			spin_unlock(&dev->req_lock);
			if (new_req->length > 0) {
				length = new_req->length;

				if (dev->port_usb->is_fixed &&
					length == dev->port_usb->fixed_in_len &&
					(length % in->maxpacket) == 0)
					new_req->zero = 0;
				else
					new_req->zero = 1;

				if (new_req->zero && !dev->zlp &&
						(length % in->maxpacket) == 0) {
					new_req->zero = 0;
					length++;
				}

				new_req->length = length;
				retval = usb_ep_queue(in, new_req, GFP_ATOMIC);
				switch (retval) {
				default:
					DBG(dev, "tx queue err %d\n", retval);
					new_req->length = 0;
					spin_lock(&dev->req_lock);
					list_add_tail(&new_req->list,
							&dev->tx_reqs);
					spin_unlock(&dev->req_lock);
					break;
				case 0:
					spin_lock(&dev->req_lock);
					dev->no_tx_req_used++;
					spin_unlock(&dev->req_lock);
					net->trans_start = jiffies;
				}
			} else {
				spin_lock(&dev->req_lock);
				list_add_tail(&new_req->list, &dev->tx_reqs);
				spin_unlock(&dev->req_lock);
			}
		} else {
			spin_unlock(&dev->req_lock);
		}
	} else {
		skb = req->context;
		
		if (dev->port_usb->multi_pkt_xfer && dev->tx_req_bufsize) {
			req->buf = kzalloc(dev->tx_req_bufsize, GFP_ATOMIC);
			req->context = NULL;
		} else {
			req->buf = NULL;
		}

		spin_unlock(&dev->req_lock);
		dev_kfree_skb_any(skb);
	}

	if (netif_carrier_ok(dev->net))
		netif_wake_queue(dev->net);
}

static inline int is_promisc(u16 cdc_filter)
{
	return cdc_filter & USB_CDC_PACKET_TYPE_PROMISCUOUS;
}

static int alloc_tx_buffer(struct eth_dev *dev)
{
	struct list_head	*act;
	struct usb_request	*req;

	dev->tx_req_bufsize = (dev->dl_max_pkts_per_xfer *
				(dev->net->mtu
				+ sizeof(struct ethhdr)
				
				+ 44
				+ 22));

	list_for_each(act, &dev->tx_reqs) {
		req = container_of(act, struct usb_request, list);
		if (!req->buf) {
			req->buf = kzalloc(dev->tx_req_bufsize,
						GFP_ATOMIC);
			if (!req->buf)
				goto free_buf;
		}
		
		req->context = NULL;
	}
	return 0;

free_buf:
	
	dev->tx_req_bufsize = 0;
	list_for_each(act, &dev->tx_reqs) {
		req = container_of(act, struct usb_request, list);
		kfree(req->buf);
		req->buf = NULL;
	}
	return -ENOMEM;
}

static netdev_tx_t eth_start_xmit(struct sk_buff *skb,
					struct net_device *net)
{
	struct eth_dev		*dev = netdev_priv(net);
	int			length;
	int			retval;
	struct usb_request	*req = NULL;
	unsigned long		flags;
	struct usb_ep		*in;
	u16			cdc_filter;
	bool			multi_pkt_xfer = false;

	if ((!skb) || (IS_ERR(skb)))
		return NETDEV_TX_OK;

	if ((!net) || (IS_ERR(net)))
		return NETDEV_TX_OK;

	length = skb->len;
	spin_lock_irqsave(&dev->lock, flags);
	if (dev->port_usb) {
		in = dev->port_usb->in_ep;
		cdc_filter = dev->port_usb->cdc_filter;
		multi_pkt_xfer = dev->port_usb->multi_pkt_xfer;
	} else {
		in = NULL;
		cdc_filter = 0;
	}
	spin_unlock_irqrestore(&dev->lock, flags);

	if (!in) {
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	
	spin_lock_irqsave(&dev->req_lock, flags);
	if (multi_pkt_xfer && !dev->tx_req_bufsize) {
		retval = alloc_tx_buffer(dev);
		if (retval < 0) {
			spin_unlock_irqrestore(&dev->req_lock, flags);
			return -ENOMEM;
		}
	}
	spin_unlock_irqrestore(&dev->req_lock, flags);

	
	if (!is_promisc(cdc_filter)) {
		u8		*dest = skb->data;

		if (is_multicast_ether_addr(dest)) {
			u16	type;

			if (is_broadcast_ether_addr(dest))
				type = USB_CDC_PACKET_TYPE_BROADCAST;
			else
				type = USB_CDC_PACKET_TYPE_ALL_MULTICAST;
			if (!(cdc_filter & type)) {
				dev_kfree_skb_any(skb);
				return NETDEV_TX_OK;
			}
		}
		
	}

	spin_lock_irqsave(&dev->req_lock, flags);
	if (list_empty(&dev->tx_reqs)) {
		spin_unlock_irqrestore(&dev->req_lock, flags);
		return NETDEV_TX_BUSY;
	}

	req = container_of(dev->tx_reqs.next, struct usb_request, list);
	list_del(&req->list);

	
	if (list_empty(&dev->tx_reqs))
		netif_stop_queue(net);
	spin_unlock_irqrestore(&dev->req_lock, flags);

	spin_lock_irqsave(&dev->lock, flags);
	if (dev->wrap) {
		if (dev->port_usb)
			skb = dev->wrap(dev->port_usb, skb);
		if (!skb) {
			spin_unlock_irqrestore(&dev->lock, flags);
			goto drop;
		}
	}

	if (multi_pkt_xfer) {

		pr_debug("req->length:%d header_len:%u\n"
				"skb->len:%d skb->data_len:%d\n",
				req->length, dev->header_len,
				skb->len, skb->data_len);
		
		memcpy(req->buf + req->length, dev->port_usb->header,
						dev->header_len);
		
		req->length += dev->header_len;
		spin_unlock_irqrestore(&dev->lock, flags);
		
		memcpy(req->buf + req->length, skb->data, skb->len);
		
		req->length += skb->len;
		length = req->length;
		dev_kfree_skb_any(skb);

		spin_lock_irqsave(&dev->req_lock, flags);
		dev->tx_skb_hold_count++;
		if (dev->tx_skb_hold_count < dev->dl_max_pkts_per_xfer) {
			if (dev->no_tx_req_used > TX_REQ_THRESHOLD) {
				list_add(&req->list, &dev->tx_reqs);
				spin_unlock_irqrestore(&dev->req_lock, flags);
				goto success;
			}
		}

		dev->no_tx_req_used++;
		dev->tx_skb_hold_count = 0;
		spin_unlock_irqrestore(&dev->req_lock, flags);
	} else {
		spin_unlock_irqrestore(&dev->lock, flags);
		length = skb->len;
		req->buf = skb->data;
		req->context = skb;
	}

	
	if (dev->port_usb->is_fixed &&
	    length == dev->port_usb->fixed_in_len &&
	    (length % in->maxpacket) == 0)
		req->zero = 0;
	else
		req->zero = 1;

	if (req->zero && !dev->zlp && (length % in->maxpacket) == 0) {
		req->zero = 0;
		length++;
	}

	req->length = length;

	
	if (gadget_is_dualspeed(dev->gadget) &&
			 (dev->gadget->speed == USB_SPEED_HIGH)) {
		dev->tx_qlen++;
		if (dev->tx_qlen == (qmult/2)) {
			req->no_interrupt = 0;
			dev->tx_qlen = 0;
		} else {
			req->no_interrupt = 1;
		}
	} else {
		req->no_interrupt = 0;
	}

	retval = usb_ep_queue(in, req, GFP_ATOMIC);
	switch (retval) {
	default:
		DBG(dev, "tx queue err %d\n", retval);
		break;
	case 0:
		net->trans_start = jiffies;
	}

#if fcAUTO_PERF_LOCK
    if (skb->len >= 1024)
        auto_perf_lock_enable(1);
#endif

	if (retval) {
		if (!multi_pkt_xfer)
			dev_kfree_skb_any(skb);
		else
			req->length = 0;
drop:
		dev->net->stats.tx_dropped++;
		spin_lock_irqsave(&dev->req_lock, flags);
		if (list_empty(&dev->tx_reqs))
			netif_start_queue(net);
		list_add_tail(&req->list, &dev->tx_reqs);
		spin_unlock_irqrestore(&dev->req_lock, flags);
	}
success:
	return NETDEV_TX_OK;
}


static void eth_start(struct eth_dev *dev, gfp_t gfp_flags)
{
	DBG(dev, "%s\n", __func__);

	
	rx_fill(dev, gfp_flags);

	
	dev->tx_qlen = 0;
	netif_wake_queue(dev->net);
}

static int eth_open(struct net_device *net)
{
	struct eth_dev	*dev = netdev_priv(net);
	struct gether	*link;

	DBG(dev, "%s\n", __func__);
	if (netif_carrier_ok(dev->net))
		eth_start(dev, GFP_KERNEL);

	spin_lock_irq(&dev->lock);
	link = dev->port_usb;
	if (link && link->open)
		link->open(link);
	spin_unlock_irq(&dev->lock);

	return 0;
}

static int eth_stop(struct net_device *net)
{
	struct eth_dev	*dev = netdev_priv(net);
	unsigned long	flags;

#if fcAUTO_PERF_LOCK
	auto_setup_perf_lock(false);
#endif

	VDBG(dev, "%s\n", __func__);
	netif_stop_queue(net);

	DBG(dev, "stop stats: rx/tx %ld/%ld, errs %ld/%ld\n",
		dev->net->stats.rx_packets, dev->net->stats.tx_packets,
		dev->net->stats.rx_errors, dev->net->stats.tx_errors
		);

	
	spin_lock_irqsave(&dev->lock, flags);
	if (dev->port_usb) {
		struct gether	*link = dev->port_usb;
		const struct usb_endpoint_descriptor *in;
		const struct usb_endpoint_descriptor *out;

		if (link->close)
			link->close(link);

		in = link->in_ep->desc;
		out = link->out_ep->desc;
		usb_ep_disable(link->in_ep);
		usb_ep_disable(link->out_ep);
		if (netif_carrier_ok(net)) {
			if (config_ep_by_speed(dev->gadget, &link->func,
					       link->in_ep) ||
			    config_ep_by_speed(dev->gadget, &link->func,
					       link->out_ep)) {
				link->in_ep->desc = NULL;
				link->out_ep->desc = NULL;
				return -EINVAL;
			}
			DBG(dev, "host still using in/out endpoints\n");
			link->in_ep->desc = in;
			link->out_ep->desc = out;
			usb_ep_enable(link->in_ep);
			usb_ep_enable(link->out_ep);
		}
	}
	spin_unlock_irqrestore(&dev->lock, flags);

	return 0;
}


static char *dev_addr;
module_param(dev_addr, charp, S_IRUGO);
MODULE_PARM_DESC(dev_addr, "Device Ethernet Address");

static char *host_addr;
module_param(host_addr, charp, S_IRUGO);
MODULE_PARM_DESC(host_addr, "Host Ethernet Address");

static int get_ether_addr(const char *str, u8 *dev_addr)
{
	if (str) {
		unsigned	i;

		for (i = 0; i < 6; i++) {
			unsigned char num;

			if ((*str == '.') || (*str == ':'))
				str++;
			num = hex_to_bin(*str++) << 4;
			num |= hex_to_bin(*str++);
			dev_addr [i] = num;
		}
		if (is_valid_ether_addr(dev_addr))
			return 0;
	}
	random_ether_addr(dev_addr);
	return 1;
}


static const struct net_device_ops eth_netdev_ops = {
	.ndo_open		= eth_open,
	.ndo_stop		= eth_stop,
	.ndo_start_xmit		= eth_start_xmit,
	.ndo_change_mtu		= ueth_change_mtu,
	.ndo_set_mac_address 	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
};

static struct device_type gadget_type = {
	.name	= "gadget",
};

int gether_setup(struct usb_gadget *g, u8 ethaddr[ETH_ALEN])
{
	return gether_setup_name(g, ethaddr, "usb");
}

int gether_setup_name(struct usb_gadget *g, u8 ethaddr[ETH_ALEN],
		const char *netname)
{
	struct eth_dev		*dev;
	struct net_device	*net;
	int			status;

	if (the_dev) {
		memcpy(ethaddr, the_dev->host_mac, ETH_ALEN);
        if (g) {
            the_dev->miMaxMtu = g->miMaxMtu;
            if (the_dev->miMaxMtu == ETH_FRAME_LEN_MAX - ETH_HLEN)
                the_dev->net->mtu = ETH_FRAME_LEN_MAX - ETH_HLEN;
        }
		return 0;
	}

	net = alloc_etherdev(sizeof *dev);
	if (!net)
		return -ENOMEM;

	dev = netdev_priv(net);
	spin_lock_init(&dev->lock);
	spin_lock_init(&dev->req_lock);
	INIT_WORK(&dev->work, eth_work);
	INIT_WORK(&dev->rx_work, process_rx_w);
	INIT_LIST_HEAD(&dev->tx_reqs);
	INIT_LIST_HEAD(&dev->rx_reqs);

	skb_queue_head_init(&dev->rx_frames);

	
	dev->net = net;
	snprintf(net->name, sizeof(net->name), "%s%%d", netname);

	if (get_ether_addr(dev_addr, net->dev_addr))
		dev_warn(&g->dev,
			"using random %s ethernet address\n", "self");
	if (get_ether_addr(host_addr, dev->host_mac))
		dev_warn(&g->dev,
			"using random %s ethernet address\n", "host");

	if (ethaddr)
		memcpy(ethaddr, dev->host_mac, ETH_ALEN);

	net->netdev_ops = &eth_netdev_ops;

	SET_ETHTOOL_OPS(net, &ops);

	dev->gadget = g;
	SET_NETDEV_DEV(net, &g->dev);
	SET_NETDEV_DEVTYPE(net, &gadget_type);

	status = register_netdev(net);
	if (status < 0) {
		dev_dbg(&g->dev, "register_netdev failed, %d\n", status);
		free_netdev(net);
	} else {
		INFO(dev, "MAC %pM\n", net->dev_addr);
		INFO(dev, "HOST MAC %pM\n", dev->host_mac);

		the_dev = dev;
        if (g) {
            the_dev->miMaxMtu = g->miMaxMtu;
            if (the_dev->miMaxMtu == ETH_FRAME_LEN_MAX - ETH_HLEN)
                the_dev->net->mtu = ETH_FRAME_LEN_MAX - ETH_HLEN;
        }
		netif_carrier_off(net);
	}

#if fcAUTO_PERF_LOCK
    INIT_WORK(&dev->release_perf_lock_work, release_perf_lock_work_func);
    setup_timer(&dev->perf_timer, auto_perf_timeout_handler, (unsigned long) dev);

    dev->pEthPerf_sdev = NULL;
    dev->EthPerf_sdev.name = iEthPerf_name;
    status = switch_dev_register(&dev->EthPerf_sdev);
    if (status < 0) {
        printk(KERN_ERR "USB EthPerf_sdev switch_dev_register register fail\n");
    }
    dev->pEthPerf_sdev = &dev->EthPerf_sdev;
#endif

	return status;
}

void gether_cleanup(void)
{
#if fcAUTO_PERF_LOCK
struct eth_dev *dev = the_dev;
#endif

	if (!the_dev)
		return;

#if fcAUTO_PERF_LOCK
    del_timer(&dev->perf_timer);
    switch_dev_unregister(&dev->EthPerf_sdev);
    dev->pEthPerf_sdev = NULL;
#endif

	unregister_netdev(the_dev->net);
	flush_work_sync(&the_dev->work);
	free_netdev(the_dev->net);

	the_dev = NULL;
}

int gether_change_mtu(int new_mtu)
{
	struct eth_dev *dev = the_dev;
	return ueth_change_mtu(dev->net, new_mtu);
}

/**
 * gether_connect - notify network layer that USB link is active
 * @link: the USB link, set up with endpoints, descriptors matching
 *	current device speed, and any framing wrapper(s) set up.
 * Context: irqs blocked
 *
 * This is called to activate endpoints and let the network layer know
 * the connection is active ("carrier detect").  It may cause the I/O
 * queues to open and start letting network packets flow, but will in
 * any case activate the endpoints so that they respond properly to the
 * USB host.
 *
 * Verify net_device pointer returned using IS_ERR().  If it doesn't
 * indicate some error code (negative errno), ep->driver_data values
 * have been overwritten.
 */
struct net_device *gether_connect(struct gether *link)
{
	struct eth_dev		*dev = the_dev;
	int			result = 0;

	if (!dev)
		return ERR_PTR(-EINVAL);

	link->header = kzalloc(sizeof(struct rndis_packet_msg_type),
							GFP_ATOMIC);
	if (!link->header) {
		pr_err("RNDIS header memory allocation failed.\n");
		result = -ENOMEM;
		goto fail;
	}

	link->in_ep->driver_data = dev;
	result = usb_ep_enable(link->in_ep);
	if (result != 0) {
		DBG(dev, "enable %s --> %d\n",
			link->in_ep->name, result);
		goto fail0;
	}

	link->out_ep->driver_data = dev;
	result = usb_ep_enable(link->out_ep);
	if (result != 0) {
		DBG(dev, "enable %s --> %d\n",
			link->out_ep->name, result);
		goto fail1;
	}

	if (result == 0)
		result = alloc_requests(dev, link, qlen(dev->gadget));

	if (result == 0) {

		dev->zlp = link->is_zlp_ok;
		DBG(dev, "qlen %d\n", qlen(dev->gadget));

		dev->header_len = link->header_len;
		dev->unwrap = link->unwrap;
		dev->wrap = link->wrap;
		dev->ul_max_pkts_per_xfer = link->ul_max_pkts_per_xfer;
		dev->dl_max_pkts_per_xfer = link->dl_max_pkts_per_xfer;

		spin_lock(&dev->lock);
		dev->tx_skb_hold_count = 0;
		dev->no_tx_req_used = 0;
		dev->tx_req_bufsize = 0;
		dev->port_usb = link;
		link->ioport = dev;
		if (netif_running(dev->net)) {
			if (link->open)
				link->open(link);
		} else {
			if (link->close)
				link->close(link);
		}
		spin_unlock(&dev->lock);

		netif_carrier_on(dev->net);
		if (netif_running(dev->net))
			eth_start(dev, GFP_ATOMIC);

	
	} else {
		(void) usb_ep_disable(link->out_ep);
fail1:
		(void) usb_ep_disable(link->in_ep);
	}

	
	if (result < 0) {
fail0:
		kfree(link->header);
fail:
		return ERR_PTR(result);
	}

	return dev->net;
}

void gether_disconnect(struct gether *link)
{
	struct eth_dev		*dev = link->ioport;
	struct usb_request	*req;
	struct sk_buff		*skb;

	if (!dev)
		return;

	DBG(dev, "%s\n", __func__);

	netif_stop_queue(dev->net);
	netif_carrier_off(dev->net);

	usb_ep_disable(link->in_ep);
	spin_lock(&dev->req_lock);
	while (!list_empty(&dev->tx_reqs)) {
		req = container_of(dev->tx_reqs.next,
					struct usb_request, list);
		list_del(&req->list);

		spin_unlock(&dev->req_lock);
		if (link->multi_pkt_xfer) {
			kfree(req->buf);
			req->buf = NULL;
		}
		usb_ep_free_request(link->in_ep, req);
		spin_lock(&dev->req_lock);
	}
	
	kfree(link->header);
	link->header = NULL;
	spin_unlock(&dev->req_lock);
	link->in_ep->driver_data = NULL;
	link->in_ep->desc = NULL;

	usb_ep_disable(link->out_ep);
	spin_lock(&dev->req_lock);
	while (!list_empty(&dev->rx_reqs)) {
		req = container_of(dev->rx_reqs.next,
					struct usb_request, list);
		list_del(&req->list);

		spin_unlock(&dev->req_lock);
		usb_ep_free_request(link->out_ep, req);
		spin_lock(&dev->req_lock);
	}
	spin_unlock(&dev->req_lock);

	spin_lock(&dev->rx_frames.lock);
	while ((skb = __skb_dequeue(&dev->rx_frames)))
		dev_kfree_skb_any(skb);
	spin_unlock(&dev->rx_frames.lock);

	link->out_ep->driver_data = NULL;
	link->out_ep->desc = NULL;

	
	dev->header_len = 0;
	dev->unwrap = NULL;
	dev->wrap = NULL;

	spin_lock(&dev->lock);
	dev->port_usb = NULL;
	link->ioport = NULL;
	spin_unlock(&dev->lock);
}

static int __init gether_init(void)
{
	uether_wq  = create_singlethread_workqueue("uether");
	if (!uether_wq) {
		pr_err("%s: Unable to create workqueue: uether\n", __func__);
		return -ENOMEM;
	}
	return 0;
}
module_init(gether_init);

static void __exit gether_exit(void)
{
	destroy_workqueue(uether_wq);

}
module_exit(gether_exit);
MODULE_DESCRIPTION("ethernet over USB driver");
MODULE_LICENSE("GPL v2");
