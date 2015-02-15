/*
 * xHCI host controller driver
 *
 * Copyright (C) 2008 Intel Corp.
 *
 * Author: Sarah Sharp
 * Some code borrowed from the Linux EHCI driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/gfp.h>
#include <linux/slab.h>
#include <asm/unaligned.h>

#include "xhci.h"

#define	PORT_WAKE_BITS	(PORT_WKOC_E | PORT_WKDISC_E | PORT_WKCONN_E)
#define	PORT_RWC_BITS	(PORT_CSC | PORT_PEC | PORT_WRC | PORT_OCC | \
			 PORT_RC | PORT_PLC | PORT_PE)

/* usb 1.1 root hub device descriptor */
static u8 usb_bos_descriptor [] = {
	USB_DT_BOS_SIZE,		/*  __u8 bLength, 5 bytes */
	USB_DT_BOS,			/*  __u8 bDescriptorType */
	0x0F, 0x00,			/*  __le16 wTotalLength, 15 bytes */
	0x1,				/*  __u8 bNumDeviceCaps */
	/* First device capability */
	USB_DT_USB_SS_CAP_SIZE,		/*  __u8 bLength, 10 bytes */
	USB_DT_DEVICE_CAPABILITY,	/* Device Capability */
	USB_SS_CAP_TYPE,		/* bDevCapabilityType, SUPERSPEED_USB */
	0x00,				/* bmAttributes, LTM off by default */
	USB_5GBPS_OPERATION, 0x00,	/* wSpeedsSupported, 5Gbps only */
	0x03,				/* bFunctionalitySupport,
					   USB 3.0 speed only */
	0x00,				/* bU1DevExitLat, set later. */
	0x00, 0x00			/* __le16 bU2DevExitLat, set later. */
};


static void xhci_common_hub_descriptor(struct xhci_hcd *xhci,
		struct usb_hub_descriptor *desc, int ports)
{
	u16 temp;

	desc->bPwrOn2PwrGood = 10;	/* xhci section 5.4.9 says 20ms max */
	desc->bHubContrCurrent = 0;

	desc->bNbrPorts = ports;
	temp = 0;
	/* Bits 1:0 - support per-port power switching, or power always on */
	if (HCC_PPC(xhci->hcc_params))
		temp |= HUB_CHAR_INDV_PORT_LPSM;
	else
		temp |= HUB_CHAR_NO_LPSM;
	/* Bit  2 - root hubs are not part of a compound device */
	/* Bits 4:3 - individual port over current protection */
	temp |= HUB_CHAR_INDV_PORT_OCPM;
	/* Bits 6:5 - no TTs in root ports */
	/* Bit  7 - no port indicators */
	desc->wHubCharacteristics = cpu_to_le16(temp);
}

/* Fill in the USB 2.0 roothub descriptor */
static void xhci_usb2_hub_descriptor(struct usb_hcd *hcd, struct xhci_hcd *xhci,
		struct usb_hub_descriptor *desc)
{
	int ports;
	u16 temp;
	__u8 port_removable[(USB_MAXCHILDREN + 1 + 7) / 8];
	u32 portsc;
	unsigned int i;

	ports = xhci->num_usb2_ports;

	xhci_common_hub_descriptor(xhci, desc, ports);
	desc->bDescriptorType = USB_DT_HUB;
	temp = 1 + (ports / 8);
	desc->bDescLength = USB_DT_HUB_NONVAR_SIZE + 2 * temp;

	/* The Device Removable bits are reported on a byte granularity.
	 * If the port doesn't exist within that byte, the bit is set to 0.
	 */
	memset(port_removable, 0, sizeof(port_removable));
	for (i = 0; i < ports; i++) {
		portsc = xhci_readl(xhci, xhci->usb2_ports[i]);
		/* If a device is removable, PORTSC reports a 0, same as in the
		 * hub descriptor DeviceRemovable bits.
		 */
		if (portsc & PORT_DEV_REMOVE)
			/* This math is hairy because bit 0 of DeviceRemovable
			 * is reserved, and bit 1 is for port 1, etc.
			 */
			port_removable[(i + 1) / 8] |= 1 << ((i + 1) % 8);
	}

	/* ch11.h defines a hub descriptor that has room for USB_MAXCHILDREN
	 * ports on it.  The USB 2.0 specification says that there are two
	 * variable length fields at the end of the hub descriptor:
	 * DeviceRemovable and PortPwrCtrlMask.  But since we can have less than
	 * USB_MAXCHILDREN ports, we may need to use the DeviceRemovable array
	 * to set PortPwrCtrlMask bits.  PortPwrCtrlMask must always be set to
	 * 0xFF, so we initialize the both arrays (DeviceRemovable and
	 * PortPwrCtrlMask) to 0xFF.  Then we set the DeviceRemovable for each
	 * set of ports that actually exist.
	 */
	memset(desc->u.hs.DeviceRemovable, 0xff,
			sizeof(desc->u.hs.DeviceRemovable));
	memset(desc->u.hs.PortPwrCtrlMask, 0xff,
			sizeof(desc->u.hs.PortPwrCtrlMask));

	for (i = 0; i < (ports + 1 + 7) / 8; i++)
		memset(&desc->u.hs.DeviceRemovable[i], port_removable[i],
				sizeof(__u8));
}

/* Fill in the USB 3.0 roothub descriptor */
static void xhci_usb3_hub_descriptor(struct usb_hcd *hcd, struct xhci_hcd *xhci,
		struct usb_hub_descriptor *desc)
{
	int ports;
	u16 port_removable;
	u32 portsc;
	unsigned int i;

	ports = xhci->num_usb3_ports;
	xhci_common_hub_descriptor(xhci, desc, ports);
	desc->bDescriptorType = USB_DT_SS_HUB;
	desc->bDescLength = USB_DT_SS_HUB_SIZE;

	/* header decode latency should be zero for roothubs,
	 * see section 4.23.5.2.
	 */
	desc->u.ss.bHubHdrDecLat = 0;
	desc->u.ss.wHubDelay = 0;

	port_removable = 0;
	/* bit 0 is reserved, bit 1 is for port 1, etc. */
	for (i = 0; i < ports; i++) {
		portsc = xhci_readl(xhci, xhci->usb3_ports[i]);
		if (portsc & PORT_DEV_REMOVE)
			port_removable |= 1 << (i + 1);
	}
	memset(&desc->u.ss.DeviceRemovable,
			(__force __u16) cpu_to_le16(port_removable),
			sizeof(__u16));
}

static void xhci_hub_descriptor(struct usb_hcd *hcd, struct xhci_hcd *xhci,
		struct usb_hub_descriptor *desc)
{

	if (hcd->speed == HCD_USB3)
		xhci_usb3_hub_descriptor(hcd, xhci, desc);
	else
		xhci_usb2_hub_descriptor(hcd, xhci, desc);

}

static unsigned int xhci_port_speed(unsigned int port_status)
{
	if (DEV_LOWSPEED(port_status))
		return USB_PORT_STAT_LOW_SPEED;
	if (DEV_HIGHSPEED(port_status))
		return USB_PORT_STAT_HIGH_SPEED;
	/*
	 * FIXME: Yes, we should check for full speed, but the core uses that as
	 * a default in portspeed() in usb/core/hub.c (which is the only place
	 * USB_PORT_STAT_*_SPEED is used).
	 */
	return 0;
}

/*
 * These bits are Read Only (RO) and should be saved and written to the
 * registers: 0, 3, 10:13, 30
 * connect status, over-current status, port speed, and device removable.
 * connect status and port speed are also sticky - meaning they're in
 * the AUX well and they aren't changed by a hot, warm, or cold reset.
 */
#define	XHCI_PORT_RO	((1<<0) | (1<<3) | (0xf<<10) | (1<<30))
/*
 * These bits are RW; writing a 0 clears the bit, writing a 1 sets the bit:
 * bits 5:8, 9, 14:15, 25:27
 * link state, port power, port indicator state, "wake on" enable state
 */
#define XHCI_PORT_RWS	((0xf<<5) | (1<<9) | (0x3<<14) | (0x7<<25))
/*
 * These bits are RW; writing a 1 sets the bit, writing a 0 has no effect:
 * bit 4 (port reset)
 */
#define	XHCI_PORT_RW1S	((1<<4))
/*
 * These bits are RW; writing a 1 clears the bit, writing a 0 has no effect:
 * bits 1, 17, 18, 19, 20, 21, 22, 23
 * port enable/disable, and
 * change bits: connect, PED, warm port reset changed (reserved zero for USB 2.0 ports),
 * over-current, reset, link state, and L1 change
 */
#define XHCI_PORT_RW1CS	((1<<1) | (0x7f<<17))
/*
 * Bit 16 is RW, and writing a '1' to it causes the link state control to be
 * latched in
 */
#define	XHCI_PORT_RW	((1<<16))
/*
 * These bits are Reserved Zero (RsvdZ) and zero should be written to them:
 * bits 2, 24, 28:31
 */
#define	XHCI_PORT_RZ	((1<<2) | (1<<24) | (0xf<<28))

/*
 * Given a port state, this function returns a value that would result in the
 * port being in the same state, if the value was written to the port status
 * control register.
 * Save Read Only (RO) bits and save read/write bits where
 * writing a 0 clears the bit and writing a 1 sets the bit (RWS).
 * For all other types (RW1S, RW1CS, RW, and RZ), writing a '0' has no effect.
 */
u32 xhci_port_state_to_neutral(u32 state)
{
	/* Save read-only status and port state */
	return (state & XHCI_PORT_RO) | (state & XHCI_PORT_RWS);
}

/*
 * find slot id based on port number.
 * @port: The one-based port number from one of the two split roothubs.
 */
int xhci_find_slot_id_by_port(struct usb_hcd *hcd, struct xhci_hcd *xhci,
		u16 port)
{
	int slot_id;
	int i;
	enum usb_device_speed speed;

	slot_id = 0;
	for (i = 0; i < MAX_HC_SLOTS; i++) {
		if (!xhci->devs[i])
			continue;
		speed = xhci->devs[i]->udev->speed;
		if (((speed == USB_SPEED_SUPER) == (hcd->speed == HCD_USB3))
				&& xhci->devs[i]->fake_port == port) {
			slot_id = i;
			break;
		}
	}

	return slot_id;
}

/*
 * Stop device
 * It issues stop endpoint command for EP 0 to 30. And wait the last command
 * to complete.
 * suspend will set to 1, if suspend bit need to set in command.
 */
static int xhci_stop_device(struct xhci_hcd *xhci, int slot_id, int suspend)
{
	struct xhci_virt_device *virt_dev;
	struct xhci_command *cmd;
	unsigned long flags;
	int timeleft;
	int ret;
	int i;

	ret = 0;
	virt_dev = xhci->devs[slot_id];
	cmd = xhci_alloc_command(xhci, false, true, GFP_NOIO);
	if (!cmd) {
		xhci_dbg(xhci, "Couldn't allocate command structure.\n");
		return -ENOMEM;
	}

	spin_lock_irqsave(&xhci->lock, flags);
	for (i = LAST_EP_INDEX; i > 0; i--) {
		if (virt_dev->eps[i].ring && virt_dev->eps[i].ring->dequeue)
			xhci_queue_stop_endpoint(xhci, slot_id, i, suspend);
	}
	cmd->command_trb = xhci->cmd_ring->enqueue;
	list_add_tail(&cmd->cmd_list, &virt_dev->cmd_list);
	xhci_queue_stop_endpoint(xhci, slot_id, 0, suspend);
	xhci_ring_cmd_db(xhci);
	spin_unlock_irqrestore(&xhci->lock, flags);

	/* Wait for last stop endpoint command to finish */
	timeleft = wait_for_completion_interruptible_timeout(
			cmd->completion,
			USB_CTRL_SET_TIMEOUT);
	if (timeleft <= 0) {
		xhci_warn(xhci, "%s while waiting for stop endpoint command\n",
				timeleft == 0 ? "Timeout" : "Signal");
		spin_lock_irqsave(&xhci->lock, flags);
		/* The timeout might have raced with the event ring handler, so
		 * only delete from the list if the item isn't poisoned.
		 */
		if (cmd->cmd_list.next != LIST_POISON1)
			list_del(&cmd->cmd_list);
		spin_unlock_irqrestore(&xhci->lock, flags);
		ret = -ETIME;
		goto command_cleanup;
	}

command_cleanup:
	xhci_free_command(xhci, cmd);
	return ret;
}

/*
 * Ring device, it rings the all doorbells unconditionally.
 */
void xhci_ring_device(struct xhci_hcd *xhci, int slot_id)
{
	int i;

	for (i = 0; i < LAST_EP_INDEX + 1; i++)
		if (xhci->devs[slot_id]->eps[i].ring &&
		    xhci->devs[slot_id]->eps[i].ring->dequeue)
			xhci_ring_ep_doorbell(xhci, slot_id, i, 0);

	return;
}

static void xhci_disable_port(struct usb_hcd *hcd, struct xhci_hcd *xhci,
		u16 wIndex, __le32 __iomem *addr, u32 port_status)
{
	/* Don't allow the USB core to disable SuperSpeed ports. */
	if (hcd->speed == HCD_USB3) {
		xhci_dbg(xhci, "Ignoring request to disable "
				"SuperSpeed port.\n");
		return;
	}

	/* Write 1 to disable the port */
	xhci_writel(xhci, port_status | PORT_PE, addr);
	if (xhci->quirks & XHCI_PORTSC_DELAY)
		ndelay(100);
	port_status = xhci_readl(xhci, addr);
	xhci_dbg(xhci, "disable port, actual port %d status  = 0x%x\n",
			wIndex, port_status);
}

static void xhci_clear_port_change_bit(struct xhci_hcd *xhci, u16 wValue,
		u16 wIndex, __le32 __iomem *addr, u32 port_status)
{
	char *port_change_bit;
	u32 status;

	switch (wValue) {
	case USB_PORT_FEAT_C_RESET:
		status = PORT_RC;
		port_change_bit = "reset";
		break;
	case USB_PORT_FEAT_C_BH_PORT_RESET:
		status = PORT_WRC;
		port_change_bit = "warm(BH) reset";
		break;
	case USB_PORT_FEAT_C_CONNECTION:
		status = PORT_CSC;
		port_change_bit = "connect";
		break;
	case USB_PORT_FEAT_C_OVER_CURRENT:
		status = PORT_OCC;
		port_change_bit = "over-current";
		break;
	case USB_PORT_FEAT_C_ENABLE:
		status = PORT_PEC;
		port_change_bit = "enable/disable";
		break;
	case USB_PORT_FEAT_C_SUSPEND:
		status = PORT_PLC;
		port_change_bit = "suspend/resume";
		break;
	case USB_PORT_FEAT_C_PORT_LINK_STATE:
		status = PORT_PLC;
		port_change_bit = "link state";
		break;
	default:
		/* Should never happen */
		return;
	}
	/* Change bits are all write 1 to clear */
	xhci_writel(xhci, port_status | status, addr);
	if (xhci->quirks & XHCI_PORTSC_DELAY)
		ndelay(100);
	port_status = xhci_readl(xhci, addr);
	xhci_dbg(xhci, "clear port %s change, actual port %d status  = 0x%x\n",
			port_change_bit, wIndex, port_status);
}

static int xhci_get_ports(struct usb_hcd *hcd, __le32 __iomem ***port_array)
{
	int max_ports;
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);

	if (hcd->speed == HCD_USB3) {
		max_ports = xhci->num_usb3_ports;
		*port_array = xhci->usb3_ports;
	} else {
		max_ports = xhci->num_usb2_ports;
		*port_array = xhci->usb2_ports;
	}

	return max_ports;
}

void xhci_set_link_state(struct xhci_hcd *xhci, __le32 __iomem **port_array,
				int port_id, u32 link_state)
{
	u32 temp;

	temp = xhci_readl(xhci, port_array[port_id]);
	temp = xhci_port_state_to_neutral(temp);
	temp &= ~PORT_PLS_MASK;
	temp |= PORT_LINK_STROBE | link_state;
	xhci_writel(xhci, temp, port_array[port_id]);
	if (xhci->quirks & XHCI_PORTSC_DELAY)
		ndelay(100);
}

void xhci_set_remote_wake_mask(struct xhci_hcd *xhci,
		__le32 __iomem **port_array, int port_id, u16 wake_mask)
{
	u32 temp;

	temp = xhci_readl(xhci, port_array[port_id]);
	temp = xhci_port_state_to_neutral(temp);

	if (wake_mask & USB_PORT_FEAT_REMOTE_WAKE_CONNECT)
		temp |= PORT_WKCONN_E;
	else
		temp &= ~PORT_WKCONN_E;

	if (wake_mask & USB_PORT_FEAT_REMOTE_WAKE_DISCONNECT)
		temp |= PORT_WKDISC_E;
	else
		temp &= ~PORT_WKDISC_E;

	if (wake_mask & USB_PORT_FEAT_REMOTE_WAKE_OVER_CURRENT)
		temp |= PORT_WKOC_E;
	else
		temp &= ~PORT_WKOC_E;

	xhci_writel(xhci, temp, port_array[port_id]);
	if (xhci->quirks & XHCI_PORTSC_DELAY)
		ndelay(100);
}

/* Test and clear port RWC bit */
void xhci_test_and_clear_bit(struct xhci_hcd *xhci, __le32 __iomem **port_array,
				int port_id, u32 port_bit)
{
	u32 temp;

	temp = xhci_readl(xhci, port_array[port_id]);
	if (temp & port_bit) {
		temp = xhci_port_state_to_neutral(temp);
		temp |= port_bit;
		xhci_writel(xhci, temp, port_array[port_id]);
		if (xhci->quirks & XHCI_PORTSC_DELAY)
			ndelay(100);
	}
}

static void xhci_single_step_completion(struct urb *urb)
{
	struct completion *done = urb->context;

	complete(done);
}

/*
 * Allocate a URB and initialize the various fields of it.
 * This API is used by the single_step_set_feature test of
 * EHSET where IN packet of the GetDescriptor request is
 * sent 15secs after the SETUP packet.
 * Return NULL if failed.
 */
static struct urb *xhci_request_single_step_set_feature_urb(
		struct usb_device *udev,
		void *dr,
		void *buf,
		struct completion *done)
{
	struct urb *urb;
	struct usb_hcd *hcd = bus_to_hcd(udev->bus);
	struct usb_host_endpoint *ep;

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb)
		return NULL;

	urb->pipe = usb_rcvctrlpipe(udev, 0);
	ep = udev->ep_in[usb_pipeendpoint(urb->pipe)];
	if (!ep) {
		usb_free_urb(urb);
		return NULL;
	}

	/*
	 * Initialize the various URB fields as these are used by the HCD
	 * driver to queue it and as well as when completion happens.
	 */
	urb->ep = ep;
	urb->dev = udev;
	urb->setup_packet = dr;
	urb->transfer_buffer = buf;
	urb->transfer_buffer_length = USB_DT_DEVICE_SIZE;
	urb->complete = xhci_single_step_completion;
	urb->status = -EINPROGRESS;
	urb->actual_length = 0;
	urb->transfer_flags = URB_DIR_IN;
	usb_get_urb(urb);
	atomic_inc(&urb->use_count);
	atomic_inc(&urb->dev->urbnum);
	usb_hcd_map_urb_for_dma(hcd, urb, GFP_KERNEL);
	urb->context = done;
	return urb;
}

/*
 * This function implements the USB_PORT_FEAT_TEST handling of the
 * SINGLE_STEP_SET_FEATURE test mode as defined in the Embedded
 * High-Speed Electrical Test (EHSET) specification. This simply
 * issues a GetDescriptor control transfer, with an inserted 15-second
 * delay after the end of the SETUP stage and before the IN token of
 * the DATA stage is set. The idea is that this gives the test operator
 * enough time to configure the oscilloscope to perform a measurement
 * of the response time between the DATA and ACK packets that follow.
 */
static int xhci_ehset_single_step_set_feature(struct usb_hcd *hcd, int port)
{
	int retval = -ENOMEM;
	struct usb_ctrlrequest *dr;
	struct urb *urb;
	struct usb_device *udev;
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	struct usb_device_descriptor *buf;
	unsigned long flags;
	DECLARE_COMPLETION_ONSTACK(done);

	/* Obtain udev of the rhub's child port */
	udev = hcd->self.root_hub->children[port];
	if (!udev) {
		xhci_err(xhci, "No device attached to the RootHub\n");
		return -ENODEV;
	}
	buf = kmalloc(USB_DT_DEVICE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	dr = kmalloc(sizeof(struct usb_ctrlrequest), GFP_KERNEL);
	if (!dr) {
		kfree(buf);
		return -ENOMEM;
	}

	/* Fill Setup packet for GetDescriptor */
	dr->bRequestType = USB_DIR_IN;
	dr->bRequest = USB_REQ_GET_DESCRIPTOR;
	dr->wValue = cpu_to_le16(USB_DT_DEVICE << 8);
	dr->wIndex = 0;
	dr->wLength = cpu_to_le16(USB_DT_DEVICE_SIZE);
	urb = xhci_request_single_step_set_feature_urb(udev, dr, buf, &done);
	if (!urb)
		goto cleanup;

	/* Now complete just the SETUP stage */
	spin_lock_irqsave(&xhci->lock, flags);
	retval = xhci_submit_single_step_set_feature(hcd, urb, 1);
	spin_unlock_irqrestore(&xhci->lock, flags);
	if (retval)
		goto out1;

	if (!wait_for_completion_timeout(&done, msecs_to_jiffies(2000))) {
		usb_kill_urb(urb);
		retval = -ETIMEDOUT;
		xhci_err(xhci, "%s SETUP stage timed out on ep0\n", __func__);
		goto out1;
	}

	/* Sleep for 15 seconds; HC will send SOFs during this period */
	msleep(15 * 1000);

	/* Complete remaining DATA and status stages. Re-use same URB */
	urb->status = -EINPROGRESS;
	usb_get_urb(urb);
	atomic_inc(&urb->use_count);
	atomic_inc(&urb->dev->urbnum);

	spin_lock_irqsave(&xhci->lock, flags);
	retval = xhci_submit_single_step_set_feature(hcd, urb, 0);
	spin_unlock_irqrestore(&xhci->lock, flags);
	if (!retval && !wait_for_completion_timeout(&done,
						msecs_to_jiffies(2000))) {
		usb_kill_urb(urb);
		retval = -ETIMEDOUT;
		xhci_err(xhci, "%s IN stage timed out on ep0\n", __func__);
	}
out1:
	usb_free_urb(urb);
cleanup:
	kfree(dr);
	kfree(buf);
	return retval;
}

int xhci_hub_control(struct usb_hcd *hcd, u16 typeReq, u16 wValue,
		u16 wIndex, char *buf, u16 wLength)
{
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	int max_ports;
	unsigned long flags;
	u32 temp, status;
	int retval = 0;
	__le32 __iomem **port_array;
	int slot_id;
	struct xhci_bus_state *bus_state;
	u16 link_state = 0;
	u16 wake_mask = 0;
	u16 test_mode = 0;

	max_ports = xhci_get_ports(hcd, &port_array);
	bus_state = &xhci->bus_state[hcd_index(hcd)];

	spin_lock_irqsave(&xhci->lock, flags);
	switch (typeReq) {
	case GetHubStatus:
		/* No power source, over-current reported per port */
		memset(buf, 0, 4);
		break;
	case GetHubDescriptor:
		/* Check to make sure userspace is asking for the USB 3.0 hub
		 * descriptor for the USB 3.0 roothub.  If not, we stall the
		 * endpoint, like external hubs do.
		 */
		if (hcd->speed == HCD_USB3 &&
				(wLength < USB_DT_SS_HUB_SIZE ||
				 wValue != (USB_DT_SS_HUB << 8))) {
			xhci_dbg(xhci, "Wrong hub descriptor type for "
					"USB 3.0 roothub.\n");
			goto error;
		}
		xhci_hub_descriptor(hcd, xhci,
				(struct usb_hub_descriptor *) buf);
		break;
	case DeviceRequest | USB_REQ_GET_DESCRIPTOR:
		if ((wValue & 0xff00) != (USB_DT_BOS << 8))
			goto error;

		if (hcd->speed != HCD_USB3)
			goto error;

		memcpy(buf, &usb_bos_descriptor,
				USB_DT_BOS_SIZE + USB_DT_USB_SS_CAP_SIZE);
		temp = xhci_readl(xhci, &xhci->cap_regs->hcs_params3);
		buf[12] = HCS_U1_LATENCY(temp);
		put_unaligned_le16(HCS_U2_LATENCY(temp), &buf[13]);

		spin_unlock_irqrestore(&xhci->lock, flags);
		return USB_DT_BOS_SIZE + USB_DT_USB_SS_CAP_SIZE;
	case GetPortStatus:
		if (!wIndex || wIndex > max_ports)
			goto error;
		wIndex--;
		status = 0;
		temp = xhci_readl(xhci, port_array[wIndex]);
		if (temp == 0xffffffff) {
			retval = -ENODEV;
			break;
		}
		xhci_dbg(xhci, "get port status, actual port %d status  = 0x%x\n", wIndex, temp);

		/* wPortChange bits */
		if (temp & PORT_CSC)
			status |= USB_PORT_STAT_C_CONNECTION << 16;
		if (temp & PORT_PEC)
			status |= USB_PORT_STAT_C_ENABLE << 16;
		if ((temp & PORT_OCC))
			status |= USB_PORT_STAT_C_OVERCURRENT << 16;
		if ((temp & PORT_RC))
			status |= USB_PORT_STAT_C_RESET << 16;
		/* USB3.0 only */
		if (hcd->speed == HCD_USB3) {
			if ((temp & PORT_PLC))
				status |= USB_PORT_STAT_C_LINK_STATE << 16;
			if ((temp & PORT_WRC))
				status |= USB_PORT_STAT_C_BH_RESET << 16;
		}

		if (hcd->speed != HCD_USB3) {
			if ((temp & PORT_PLS_MASK) == XDEV_U3
					&& (temp & PORT_POWER))
				status |= USB_PORT_STAT_SUSPEND;
		}
		if ((temp & PORT_PLS_MASK) == XDEV_RESUME &&
				!DEV_SUPERSPEED(temp)) {
			if ((temp & PORT_RESET) || !(temp & PORT_PE))
				goto error;
			if (time_after_eq(jiffies,
					bus_state->resume_done[wIndex])) {
				xhci_dbg(xhci, "Resume USB2 port %d\n",
					wIndex + 1);
				bus_state->resume_done[wIndex] = 0;
				xhci_set_link_state(xhci, port_array, wIndex,
							XDEV_U0);
				xhci_dbg(xhci, "set port %d resume\n",
					wIndex + 1);
				slot_id = xhci_find_slot_id_by_port(hcd, xhci,
								 wIndex + 1);
				if (!slot_id) {
					xhci_dbg(xhci, "slot_id is zero\n");
					goto error;
				}
				xhci_ring_device(xhci, slot_id);
				bus_state->port_c_suspend |= 1 << wIndex;
				bus_state->suspended_ports &= ~(1 << wIndex);
			} else {
				/*
				 * The resume has been signaling for less than
				 * 20ms. Report the port status as SUSPEND,
				 * let the usbcore check port status again
				 * and clear resume signaling later.
				 */
				status |= USB_PORT_STAT_SUSPEND;
			}
		}
		if ((temp & PORT_PLS_MASK) == XDEV_U0
			&& (temp & PORT_POWER)
			&& (bus_state->suspended_ports & (1 << wIndex))) {
			bus_state->suspended_ports &= ~(1 << wIndex);
			if (hcd->speed != HCD_USB3)
				bus_state->port_c_suspend |= 1 << wIndex;
		}
		if (temp & PORT_CONNECT) {
			status |= USB_PORT_STAT_CONNECTION;
			status |= xhci_port_speed(temp);
		}
		if (temp & PORT_PE)
			status |= USB_PORT_STAT_ENABLE;
		if (temp & PORT_OC)
			status |= USB_PORT_STAT_OVERCURRENT;
		if (temp & PORT_RESET)
			status |= USB_PORT_STAT_RESET;
		if (temp & PORT_POWER) {
			if (hcd->speed == HCD_USB3)
				status |= USB_SS_PORT_STAT_POWER;
			else
				status |= USB_PORT_STAT_POWER;
		}
		/* Port Link State */
		if (hcd->speed == HCD_USB3) {
			/* resume state is a xHCI internal state.
			 * Do not report it to usb core.
			 */
			if ((temp & PORT_PLS_MASK) != XDEV_RESUME)
				status |= (temp & PORT_PLS_MASK);
		}
		if (bus_state->port_c_suspend & (1 << wIndex))
			status |= 1 << USB_PORT_FEAT_C_SUSPEND;
		xhci_dbg(xhci, "Get port status returned 0x%x\n", status);
		put_unaligned(cpu_to_le32(status), (__le32 *) buf);
		break;
	case SetPortFeature:
		/* The MSB of wIndex is the TEST Mode */
		test_mode = (wIndex & 0xff00) >> 8;
		if (wValue == USB_PORT_FEAT_LINK_STATE)
			link_state = (wIndex & 0xff00) >> 3;
		if (wValue == USB_PORT_FEAT_REMOTE_WAKE_MASK)
			wake_mask = wIndex & 0xff00;
		wIndex &= 0xff;
		if (!wIndex || wIndex > max_ports)
			goto error;
		wIndex--;
		temp = xhci_readl(xhci, port_array[wIndex]);
		if (temp == 0xffffffff) {
			retval = -ENODEV;
			break;
		}
		temp = xhci_port_state_to_neutral(temp);
		/* FIXME: What new port features do we need to support? */
		switch (wValue) {
		case USB_PORT_FEAT_SUSPEND:
			temp = xhci_readl(xhci, port_array[wIndex]);
			if ((temp & PORT_PLS_MASK) != XDEV_U0) {
				/* Resume the port to U0 first */
				xhci_set_link_state(xhci, port_array, wIndex,
							XDEV_U0);
				spin_unlock_irqrestore(&xhci->lock, flags);
				msleep(10);
				spin_lock_irqsave(&xhci->lock, flags);
			}
			/* In spec software should not attempt to suspend
			 * a port unless the port reports that it is in the
			 * enabled (PED = ‘1’,PLS < ‘3’) state.
			 */
			temp = xhci_readl(xhci, port_array[wIndex]);
			if ((temp & PORT_PE) == 0 || (temp & PORT_RESET)
				|| (temp & PORT_PLS_MASK) >= XDEV_U3) {
				xhci_warn(xhci, "USB core suspending device "
					  "not in U0/U1/U2.\n");
				goto error;
			}

			slot_id = xhci_find_slot_id_by_port(hcd, xhci,
					wIndex + 1);
			if (!slot_id) {
				xhci_warn(xhci, "slot_id is zero\n");
				goto error;
			}
			/* unlock to execute stop endpoint commands */
			spin_unlock_irqrestore(&xhci->lock, flags);
			xhci_stop_device(xhci, slot_id, 1);
			spin_lock_irqsave(&xhci->lock, flags);

			xhci_set_link_state(xhci, port_array, wIndex, XDEV_U3);

			spin_unlock_irqrestore(&xhci->lock, flags);
			msleep(10); /* wait device to enter */
			spin_lock_irqsave(&xhci->lock, flags);

			temp = xhci_readl(xhci, port_array[wIndex]);
			bus_state->suspended_ports |= 1 << wIndex;
			break;
		case USB_PORT_FEAT_LINK_STATE:
			temp = xhci_readl(xhci, port_array[wIndex]);
			/* Software should not attempt to set
			 * port link state above '5' (Rx.Detect) and the port
			 * must be enabled.
			 */
			if ((temp & PORT_PE) == 0 ||
				(link_state > USB_SS_PORT_LS_RX_DETECT)) {
				xhci_warn(xhci, "Cannot set link state.\n");
				goto error;
			}

			if (link_state == USB_SS_PORT_LS_U3) {
				slot_id = xhci_find_slot_id_by_port(hcd, xhci,
						wIndex + 1);
				if (slot_id) {
					/* unlock to execute stop endpoint
					 * commands */
					spin_unlock_irqrestore(&xhci->lock,
								flags);
					xhci_stop_device(xhci, slot_id, 1);
					spin_lock_irqsave(&xhci->lock, flags);
				}
			}

			xhci_set_link_state(xhci, port_array, wIndex,
						link_state);

			spin_unlock_irqrestore(&xhci->lock, flags);
			msleep(20); /* wait device to enter */
			spin_lock_irqsave(&xhci->lock, flags);

			temp = xhci_readl(xhci, port_array[wIndex]);
			if (link_state == USB_SS_PORT_LS_U3)
				bus_state->suspended_ports |= 1 << wIndex;
			break;
		case USB_PORT_FEAT_POWER:
			/*
			 * Turn on ports, even if there isn't per-port switching.
			 * HC will report connect events even before this is set.
			 * However, khubd will ignore the roothub events until
			 * the roothub is registered.
			 */
			xhci_writel(xhci, temp | PORT_POWER,
					port_array[wIndex]);
			if (xhci->quirks & XHCI_PORTSC_DELAY)
				ndelay(100);

			temp = xhci_readl(xhci, port_array[wIndex]);
			xhci_dbg(xhci, "set port power, actual port %d status  = 0x%x\n", wIndex, temp);
			break;
		case USB_PORT_FEAT_RESET:
			temp = (temp | PORT_RESET);
			xhci_writel(xhci, temp, port_array[wIndex]);
			if (xhci->quirks & XHCI_PORTSC_DELAY)
				ndelay(100);

			temp = xhci_readl(xhci, port_array[wIndex]);
			xhci_dbg(xhci, "set port reset, actual port %d status  = 0x%x\n", wIndex, temp);
			break;
		case USB_PORT_FEAT_REMOTE_WAKE_MASK:
			xhci_set_remote_wake_mask(xhci, port_array,
					wIndex, wake_mask);
			temp = xhci_readl(xhci, port_array[wIndex]);
			xhci_dbg(xhci, "set port remote wake mask, "
					"actual port %d status  = 0x%x\n",
					wIndex, temp);
			break;
		case USB_PORT_FEAT_BH_PORT_RESET:
			temp |= PORT_WR;
			xhci_writel(xhci, temp, port_array[wIndex]);
			if (xhci->quirks & XHCI_PORTSC_DELAY)
				ndelay(100);

			temp = xhci_readl(xhci, port_array[wIndex]);
			break;
		case USB_PORT_FEAT_TEST:
			slot_id = xhci_find_slot_id_by_port(hcd, xhci,
					wIndex + 1);
			if (test_mode && test_mode <= 5) {
				/* unlock to execute stop endpoint commands */
				spin_unlock_irqrestore(&xhci->lock, flags);
				xhci_stop_device(xhci, slot_id, 1);
				spin_lock_irqsave(&xhci->lock, flags);
				xhci_halt(xhci);

				temp = xhci_readl(xhci, port_array[wIndex] + 1);
				temp |= test_mode << 28;
				xhci_writel(xhci, temp, port_array[wIndex] + 1);
			} else if (test_mode == 6) {
				spin_unlock_irqrestore(&xhci->lock, flags);
				retval = xhci_ehset_single_step_set_feature(hcd,
									wIndex);
				spin_lock_irqsave(&xhci->lock, flags);
			} else {
				goto error;
			}
			break;
		default:
			goto error;
		}
		/* unblock any posted writes */
		temp = xhci_readl(xhci, port_array[wIndex]);
		break;
	case ClearPortFeature:
		if (!wIndex || wIndex > max_ports)
			goto error;
		wIndex--;
		temp = xhci_readl(xhci, port_array[wIndex]);
		if (temp == 0xffffffff) {
			retval = -ENODEV;
			break;
		}
		/* FIXME: What new port features do we need to support? */
		temp = xhci_port_state_to_neutral(temp);
		switch (wValue) {
		case USB_PORT_FEAT_SUSPEND:
			temp = xhci_readl(xhci, port_array[wIndex]);
			xhci_dbg(xhci, "clear USB_PORT_FEAT_SUSPEND\n");
			xhci_dbg(xhci, "PORTSC %04x\n", temp);
			if (temp & PORT_RESET)
				goto error;
			if ((temp & PORT_PLS_MASK) == XDEV_U3) {
				if ((temp & PORT_PE) == 0)
					goto error;

				xhci_set_link_state(xhci, port_array, wIndex,
							XDEV_RESUME);
				spin_unlock_irqrestore(&xhci->lock, flags);
				msleep(20);
				spin_lock_irqsave(&xhci->lock, flags);
				xhci_set_link_state(xhci, port_array, wIndex,
							XDEV_U0);
			}
			bus_state->port_c_suspend |= 1 << wIndex;

			slot_id = xhci_find_slot_id_by_port(hcd, xhci,
					wIndex + 1);
			if (!slot_id) {
				xhci_dbg(xhci, "slot_id is zero\n");
				goto error;
			}
			xhci_ring_device(xhci, slot_id);
			break;
		case USB_PORT_FEAT_C_SUSPEND:
			bus_state->port_c_suspend &= ~(1 << wIndex);
		case USB_PORT_FEAT_C_RESET:
		case USB_PORT_FEAT_C_BH_PORT_RESET:
		case USB_PORT_FEAT_C_CONNECTION:
		case USB_PORT_FEAT_C_OVER_CURRENT:
		case USB_PORT_FEAT_C_ENABLE:
		case USB_PORT_FEAT_C_PORT_LINK_STATE:
			xhci_clear_port_change_bit(xhci, wValue, wIndex,
					port_array[wIndex], temp);
			break;
		case USB_PORT_FEAT_ENABLE:
			xhci_disable_port(hcd, xhci, wIndex,
					port_array[wIndex], temp);
			break;
		default:
			goto error;
		}
		break;
	default:
error:
		/* "stall" on error */
		retval = -EPIPE;
	}
	spin_unlock_irqrestore(&xhci->lock, flags);
	return retval;
}

/*
 * Returns 0 if the status hasn't changed, or the number of bytes in buf.
 * Ports are 0-indexed from the HCD point of view,
 * and 1-indexed from the USB core pointer of view.
 *
 * Note that the status change bits will be cleared as soon as a port status
 * change event is generated, so we use the saved status from that event.
 */
int xhci_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	unsigned long flags;
	u32 temp, status;
	u32 mask;
	int i, retval;
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	int max_ports;
	__le32 __iomem **port_array;
	struct xhci_bus_state *bus_state;

	max_ports = xhci_get_ports(hcd, &port_array);
	bus_state = &xhci->bus_state[hcd_index(hcd)];

	/* Initial status is no changes */
	retval = (max_ports + 8) / 8;
	memset(buf, 0, retval);
	status = 0;

	mask = PORT_CSC | PORT_PEC | PORT_OCC | PORT_PLC | PORT_WRC;

	spin_lock_irqsave(&xhci->lock, flags);
	/* For each port, did anything change?  If so, set that bit in buf. */
	for (i = 0; i < max_ports; i++) {
		temp = xhci_readl(xhci, port_array[i]);
		if (temp == 0xffffffff) {
			retval = -ENODEV;
			break;
		}
		if ((temp & mask) != 0 ||
			(bus_state->port_c_suspend & 1 << i) ||
			(bus_state->resume_done[i] && time_after_eq(
			    jiffies, bus_state->resume_done[i]))) {
			buf[(i + 1) / 8] |= 1 << (i + 1) % 8;
			status = 1;
		}
	}
	spin_unlock_irqrestore(&xhci->lock, flags);
	return status ? retval : 0;
}

#ifdef CONFIG_PM

int xhci_bus_suspend(struct usb_hcd *hcd)
{
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	int max_ports, port_index;
	__le32 __iomem **port_array;
	struct xhci_bus_state *bus_state;
	unsigned long flags;

	max_ports = xhci_get_ports(hcd, &port_array);
	bus_state = &xhci->bus_state[hcd_index(hcd)];

	spin_lock_irqsave(&xhci->lock, flags);

	if (hcd->self.root_hub->do_remote_wakeup) {
		port_index = max_ports;
		while (port_index--) {
			if (bus_state->resume_done[port_index] != 0) {
				spin_unlock_irqrestore(&xhci->lock, flags);
				xhci_dbg(xhci, "suspend failed because "
						"port %d is resuming\n",
						port_index + 1);
				return -EBUSY;
			}
		}
	}

	port_index = max_ports;
	bus_state->bus_suspended = 0;
	while (port_index--) {
		/* suspend the port if the port is not suspended */
		u32 t1, t2;
		int slot_id;

		t1 = xhci_readl(xhci, port_array[port_index]);
		t2 = xhci_port_state_to_neutral(t1);

		if ((t1 & PORT_PE) && !(t1 & PORT_PLS_MASK)) {
			xhci_dbg(xhci, "port %d not suspended\n", port_index);
			slot_id = xhci_find_slot_id_by_port(hcd, xhci,
					port_index + 1);
			if (slot_id) {
				spin_unlock_irqrestore(&xhci->lock, flags);
				xhci_stop_device(xhci, slot_id, 1);
				spin_lock_irqsave(&xhci->lock, flags);
			}
			t2 &= ~PORT_PLS_MASK;
			t2 |= PORT_LINK_STROBE | XDEV_U3;
			set_bit(port_index, &bus_state->bus_suspended);
		}
		/* USB core sets remote wake mask for USB 3.0 hubs,
		 * including the USB 3.0 roothub, but only if CONFIG_USB_SUSPEND
		 * is enabled, so also enable remote wake here.
		 */
		if (hcd->self.root_hub->do_remote_wakeup) {
			if (t1 & PORT_CONNECT) {
				t2 |= PORT_WKOC_E | PORT_WKDISC_E;
				t2 &= ~PORT_WKCONN_E;
			} else {
				t2 |= PORT_WKOC_E | PORT_WKCONN_E;
				t2 &= ~PORT_WKDISC_E;
			}
		} else
			t2 &= ~PORT_WAKE_BITS;

		t1 = xhci_port_state_to_neutral(t1);
		if (t1 != t2) {
			xhci_writel(xhci, t2, port_array[port_index]);
			if (xhci->quirks & XHCI_PORTSC_DELAY)
				ndelay(100);
		}

		if (hcd->speed != HCD_USB3) {
			/* enable remote wake up for USB 2.0 */
			__le32 __iomem *addr;
			u32 tmp;

			/* Add one to the port status register address to get
			 * the port power control register address.
			 */
			addr = port_array[port_index] + 1;
			tmp = xhci_readl(xhci, addr);
			tmp |= PORT_RWE;
			xhci_writel(xhci, tmp, addr);
			if (xhci->quirks & XHCI_PORTSC_DELAY)
				ndelay(100);
		}
	}
	hcd->state = HC_STATE_SUSPENDED;
	bus_state->next_statechange = jiffies + msecs_to_jiffies(10);
	spin_unlock_irqrestore(&xhci->lock, flags);
	return 0;
}

int xhci_bus_resume(struct usb_hcd *hcd)
{
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	int max_ports, port_index;
	__le32 __iomem **port_array;
	struct xhci_bus_state *bus_state;
	u32 temp;
	unsigned long flags;

	max_ports = xhci_get_ports(hcd, &port_array);
	bus_state = &xhci->bus_state[hcd_index(hcd)];

	if (time_before(jiffies, bus_state->next_statechange))
		msleep(5);

	spin_lock_irqsave(&xhci->lock, flags);
	if (!HCD_HW_ACCESSIBLE(hcd)) {
		spin_unlock_irqrestore(&xhci->lock, flags);
		return -ESHUTDOWN;
	}

	/* delay the irqs */
	temp = xhci_readl(xhci, &xhci->op_regs->command);
	temp &= ~CMD_EIE;
	xhci_writel(xhci, temp, &xhci->op_regs->command);

	port_index = max_ports;
	while (port_index--) {
		/* Check whether need resume ports. If needed
		   resume port and disable remote wakeup */
		u32 temp;
		int slot_id;

		temp = xhci_readl(xhci, port_array[port_index]);
		if (DEV_SUPERSPEED(temp))
			temp &= ~(PORT_RWC_BITS | PORT_CEC | PORT_WAKE_BITS);
		else
			temp &= ~(PORT_RWC_BITS | PORT_WAKE_BITS);
		if (test_bit(port_index, &bus_state->bus_suspended) &&
		    (temp & PORT_PLS_MASK)) {
			if (DEV_SUPERSPEED(temp)) {
				xhci_set_link_state(xhci, port_array,
							port_index, XDEV_U0);
			} else {
				xhci_set_link_state(xhci, port_array,
						port_index, XDEV_RESUME);

				spin_unlock_irqrestore(&xhci->lock, flags);
				msleep(20);
				spin_lock_irqsave(&xhci->lock, flags);

				xhci_set_link_state(xhci, port_array,
							port_index, XDEV_U0);
			}
			/* wait for the port to enter U0 and report port link
			 * state change.
			 */
			spin_unlock_irqrestore(&xhci->lock, flags);
			msleep(20);
			spin_lock_irqsave(&xhci->lock, flags);

			/* Clear PLC */
			xhci_test_and_clear_bit(xhci, port_array, port_index,
						PORT_PLC);

			slot_id = xhci_find_slot_id_by_port(hcd,
					xhci, port_index + 1);
			if (slot_id)
				xhci_ring_device(xhci, slot_id);
		} else {
			xhci_writel(xhci, temp, port_array[port_index]);
			if (xhci->quirks & XHCI_PORTSC_DELAY)
				ndelay(100);
		}

		if (hcd->speed != HCD_USB3) {
			/* disable remote wake up for USB 2.0 */
			__le32 __iomem *addr;
			u32 tmp;

			/* Add one to the port status register address to get
			 * the port power control register address.
			 */
			addr = port_array[port_index] + 1;
			tmp = xhci_readl(xhci, addr);
			tmp &= ~PORT_RWE;
			xhci_writel(xhci, tmp, addr);
			if (xhci->quirks & XHCI_PORTSC_DELAY)
				ndelay(100);
		}
	}

	(void) xhci_readl(xhci, &xhci->op_regs->command);

	bus_state->next_statechange = jiffies + msecs_to_jiffies(5);
	/* re-enable irqs */
	temp = xhci_readl(xhci, &xhci->op_regs->command);
	temp |= CMD_EIE;
	xhci_writel(xhci, temp, &xhci->op_regs->command);
	temp = xhci_readl(xhci, &xhci->op_regs->command);

	spin_unlock_irqrestore(&xhci->lock, flags);
	return 0;
}

#endif	/* CONFIG_PM */
