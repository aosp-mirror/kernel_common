/*
 * Qualcomm Serial USB driver
 *
 *	Copyright (c) 2008, 2012 The Linux Foundation. All rights reserved.
 *	Copyright (c) 2009 Greg Kroah-Hartman <gregkh@suse.de>
 *	Copyright (c) 2009 Novell Inc.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License version
 *	2 as published by the Free Software Foundation.
 *
 */

#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/slab.h>
#include "usb-wwan.h"

#define DRIVER_AUTHOR "Qualcomm Inc"
#define DRIVER_DESC "Qualcomm USB Serial driver"

static bool debug;

#define DEVICE_G1K(v, p) \
	USB_DEVICE(v, p), .driver_info = 1

static const struct usb_device_id id_table[] = {
	/* Gobi 1000 devices */
	{DEVICE_G1K(0x05c6, 0x9211)},	/* Acer Gobi QDL device */
	{DEVICE_G1K(0x05c6, 0x9212)},	/* Acer Gobi Modem Device */
	{DEVICE_G1K(0x03f0, 0x1f1d)},	/* HP un2400 Gobi Modem Device */
	{DEVICE_G1K(0x03f0, 0x201d)},	/* HP un2400 Gobi QDL Device */
	{DEVICE_G1K(0x04da, 0x250d)},	/* Panasonic Gobi Modem device */
	{DEVICE_G1K(0x04da, 0x250c)},	/* Panasonic Gobi QDL device */
	{DEVICE_G1K(0x413c, 0x8172)},	/* Dell Gobi Modem device */
	{DEVICE_G1K(0x413c, 0x8171)},	/* Dell Gobi QDL device */
	{DEVICE_G1K(0x1410, 0xa001)},	/* Novatel Gobi Modem device */
	{DEVICE_G1K(0x1410, 0xa008)},	/* Novatel Gobi QDL device */
	{DEVICE_G1K(0x0b05, 0x1776)},	/* Asus Gobi Modem device */
	{DEVICE_G1K(0x0b05, 0x1774)},	/* Asus Gobi QDL device */
	{DEVICE_G1K(0x19d2, 0xfff3)},	/* ONDA Gobi Modem device */
	{DEVICE_G1K(0x19d2, 0xfff2)},	/* ONDA Gobi QDL device */
	{DEVICE_G1K(0x1557, 0x0a80)},	/* OQO Gobi QDL device */
	{DEVICE_G1K(0x05c6, 0x9001)},   /* Generic Gobi Modem device */
	{DEVICE_G1K(0x05c6, 0x9002)},	/* Generic Gobi Modem device */
	{DEVICE_G1K(0x05c6, 0x9202)},	/* Generic Gobi Modem device */
	{DEVICE_G1K(0x05c6, 0x9203)},	/* Generic Gobi Modem device */
	{DEVICE_G1K(0x05c6, 0x9222)},	/* Generic Gobi Modem device */
	{DEVICE_G1K(0x05c6, 0x9008)},	/* Generic Gobi QDL device */
	{DEVICE_G1K(0x05c6, 0x9009)},	/* Generic Gobi Modem device */
	{DEVICE_G1K(0x05c6, 0x9201)},	/* Generic Gobi QDL device */
	{DEVICE_G1K(0x05c6, 0x9221)},	/* Generic Gobi QDL device */
	{DEVICE_G1K(0x05c6, 0x9231)},	/* Generic Gobi QDL device */
	{DEVICE_G1K(0x1f45, 0x0001)},	/* Unknown Gobi QDL device */

	/* Gobi 2000 devices */
	{USB_DEVICE(0x1410, 0xa010)},	/* Novatel Gobi 2000 QDL device */
	{USB_DEVICE(0x1410, 0xa011)},	/* Novatel Gobi 2000 QDL device */
	{USB_DEVICE(0x1410, 0xa012)},	/* Novatel Gobi 2000 QDL device */
	{USB_DEVICE(0x1410, 0xa013)},	/* Novatel Gobi 2000 QDL device */
	{USB_DEVICE(0x1410, 0xa014)},	/* Novatel Gobi 2000 QDL device */
	{USB_DEVICE(0x413c, 0x8185)},	/* Dell Gobi 2000 QDL device (N0218, VU936) */
	{USB_DEVICE(0x413c, 0x8186)},	/* Dell Gobi 2000 Modem device (N0218, VU936) */
	{USB_DEVICE(0x05c6, 0x9208)},	/* Generic Gobi 2000 QDL device */
	{USB_DEVICE(0x05c6, 0x920b)},	/* Generic Gobi 2000 Modem device */
	{USB_DEVICE(0x05c6, 0x9224)},	/* Sony Gobi 2000 QDL device (N0279, VU730) */
	{USB_DEVICE(0x05c6, 0x9225)},	/* Sony Gobi 2000 Modem device (N0279, VU730) */
	{USB_DEVICE(0x05c6, 0x9244)},	/* Samsung Gobi 2000 QDL device (VL176) */
	{USB_DEVICE(0x05c6, 0x9245)},	/* Samsung Gobi 2000 Modem device (VL176) */
	{USB_DEVICE(0x03f0, 0x241d)},	/* HP Gobi 2000 QDL device (VP412) */
	{USB_DEVICE(0x03f0, 0x251d)},	/* HP Gobi 2000 Modem device (VP412) */
	{USB_DEVICE(0x05c6, 0x9214)},	/* Acer Gobi 2000 QDL device (VP413) */
	{USB_DEVICE(0x05c6, 0x9215)},	/* Acer Gobi 2000 Modem device (VP413) */
	{USB_DEVICE(0x05c6, 0x9264)},	/* Asus Gobi 2000 QDL device (VR305) */
	{USB_DEVICE(0x05c6, 0x9265)},	/* Asus Gobi 2000 Modem device (VR305) */
	{USB_DEVICE(0x05c6, 0x9234)},	/* Top Global Gobi 2000 QDL device (VR306) */
	{USB_DEVICE(0x05c6, 0x9235)},	/* Top Global Gobi 2000 Modem device (VR306) */
	{USB_DEVICE(0x05c6, 0x9274)},	/* iRex Technologies Gobi 2000 QDL device (VR307) */
	{USB_DEVICE(0x05c6, 0x9275)},	/* iRex Technologies Gobi 2000 Modem device (VR307) */
	{USB_DEVICE(0x1199, 0x9000)},	/* Sierra Wireless Gobi 2000 QDL device (VT773) */
	{USB_DEVICE(0x1199, 0x9001)},	/* Sierra Wireless Gobi 2000 Modem device (VT773) */
	{USB_DEVICE(0x1199, 0x9002)},	/* Sierra Wireless Gobi 2000 Modem device (VT773) */
	{USB_DEVICE(0x1199, 0x9003)},	/* Sierra Wireless Gobi 2000 Modem device (VT773) */
	{USB_DEVICE(0x1199, 0x9004)},	/* Sierra Wireless Gobi 2000 Modem device (VT773) */
	{USB_DEVICE(0x1199, 0x9005)},	/* Sierra Wireless Gobi 2000 Modem device (VT773) */
	{USB_DEVICE(0x1199, 0x9006)},	/* Sierra Wireless Gobi 2000 Modem device (VT773) */
	{USB_DEVICE(0x1199, 0x9007)},	/* Sierra Wireless Gobi 2000 Modem device (VT773) */
	{USB_DEVICE(0x1199, 0x9008)},	/* Sierra Wireless Gobi 2000 Modem device (VT773) */
	{USB_DEVICE(0x1199, 0x9009)},	/* Sierra Wireless Gobi 2000 Modem device (VT773) */
	{USB_DEVICE(0x1199, 0x900a)},	/* Sierra Wireless Gobi 2000 Modem device (VT773) */
	{USB_DEVICE(0x1199, 0x9011)},   /* Sierra Wireless Gobi 2000 Modem device (MC8305) */
	{USB_DEVICE(0x16d8, 0x8001)},	/* CMDTech Gobi 2000 QDL device (VU922) */
	{USB_DEVICE(0x16d8, 0x8002)},	/* CMDTech Gobi 2000 Modem device (VU922) */
	{USB_DEVICE(0x05c6, 0x9204)},	/* Gobi 2000 QDL device */
	{USB_DEVICE(0x05c6, 0x9205)},	/* Gobi 2000 Modem device */

	/* Gobi 3000 devices */
	{USB_DEVICE(0x03f0, 0x371d)},	/* HP un2430 Gobi 3000 QDL */
	{USB_DEVICE(0x05c6, 0x920c)},	/* Gobi 3000 QDL */
	{USB_DEVICE(0x05c6, 0x920d)},	/* Gobi 3000 Composite */
	{USB_DEVICE(0x1410, 0xa020)},   /* Novatel Gobi 3000 QDL */
	{USB_DEVICE(0x1410, 0xa021)},	/* Novatel Gobi 3000 Composite */
	{USB_DEVICE(0x413c, 0x8193)},	/* Dell Gobi 3000 QDL */
	{USB_DEVICE(0x413c, 0x8194)},	/* Dell Gobi 3000 Composite */
	{USB_DEVICE(0x1199, 0x9013)},	/* Sierra Wireless Gobi 3000 Modem device (MC8355) */
	{USB_DEVICE(0x12D1, 0x14F0)},	/* Sony Gobi 3000 QDL */
	{USB_DEVICE(0x12D1, 0x14F1)},	/* Sony Gobi 3000 Composite */
	{USB_DEVICE(0x05c6, 0x9048)},	/* MDM9x15 device */
	{USB_DEVICE(0x05c6, 0x904C)},	/* MDM9x15 device */
	{ }				/* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, id_table);

#define EFS_SYNC_IFC_NUM	2

static struct usb_driver qcdriver = {
	.name			= "qcserial",
	.probe			= usb_serial_probe,
	.disconnect		= usb_serial_disconnect,
	.id_table		= id_table,
	.suspend		= usb_serial_suspend,
	.resume			= usb_serial_resume,
	.supports_autosuspend	= true,
};

static int qcprobe(struct usb_serial *serial, const struct usb_device_id *id)
{
	struct usb_wwan_intf_private *data;
	struct usb_host_interface *intf = serial->interface->cur_altsetting;
	int retval = -ENODEV;
	__u8 nintf;
	__u8 ifnum;
	bool is_gobi1k = id->driver_info ? true : false;

	dbg("%s", __func__);
	dbg("Is Gobi 1000 = %d", is_gobi1k);

	nintf = serial->dev->actconfig->desc.bNumInterfaces;
	dbg("Num Interfaces = %d", nintf);
	ifnum = intf->desc.bInterfaceNumber;
	dbg("This Interface = %d", ifnum);

	data = kzalloc(sizeof(struct usb_wwan_intf_private),
					 GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	spin_lock_init(&data->susp_lock);

	switch (nintf) {
	case 1:
		/* QDL mode */
		/* Gobi 2000 has a single altsetting, older ones have two */
		if (serial->interface->num_altsetting == 2)
			intf = &serial->interface->altsetting[1];
		else if (serial->interface->num_altsetting > 2)
			break;

		if (intf->desc.bNumEndpoints == 2 &&
		    usb_endpoint_is_bulk_in(&intf->endpoint[0].desc) &&
		    usb_endpoint_is_bulk_out(&intf->endpoint[1].desc)) {
			dbg("QDL port found");

			if (serial->interface->num_altsetting == 1) {
				retval = 0; /* Success */
				break;
			}

			retval = usb_set_interface(serial->dev, ifnum, 1);
			if (retval < 0) {
				dev_err(&serial->dev->dev,
					"Could not set interface, error %d\n",
					retval);
				retval = -ENODEV;
				kfree(data);
			}
		}
		break;

	case 3:
	case 4:
		/* Composite mode; don't bind to the QMI/net interface as that
		 * gets handled by other drivers.
		 */

		/* Gobi 1K USB layout:
		 * 0: serial port (doesn't respond)
		 * 1: serial port (doesn't respond)
		 * 2: AT-capable modem port
		 * 3: QMI/net
		 *
		 * Gobi 2K+ USB layout:
		 * 0: QMI/net
		 * 1: DM/DIAG (use libqcdm from ModemManager for communication)
		 * 2: AT-capable modem port
		 * 3: NMEA
		 */

		if (ifnum == 1 && !is_gobi1k) {
			dbg("Gobi 2K+ DM/DIAG interface found");
			retval = usb_set_interface(serial->dev, ifnum, 0);
			if (retval < 0) {
				dev_err(&serial->dev->dev,
					"Could not set interface, error %d\n",
					retval);
				retval = -ENODEV;
				kfree(data);
			}
		} else if (ifnum == 2) {
			dbg("Modem port found");
			retval = usb_set_interface(serial->dev, ifnum, 0);
			if (retval < 0) {
				dev_err(&serial->dev->dev,
					"Could not set interface, error %d\n",
					retval);
				retval = -ENODEV;
				kfree(data);
			}
		} else if (ifnum==3 && !is_gobi1k) {
			/*
			 * NMEA (serial line 9600 8N1)
			 * # echo "\$GPS_START" > /dev/ttyUSBx
			 * # echo "\$GPS_STOP"  > /dev/ttyUSBx
			 */
			dbg("Gobi 2K+ NMEA GPS interface found");
			retval = usb_set_interface(serial->dev, ifnum, 0);
			if (retval < 0) {
				dev_err(&serial->dev->dev,
					"Could not set interface, error %d\n",
					retval);
				retval = -ENODEV;
				kfree(data);
			}
		}
		break;

	case 9:
		if (ifnum != EFS_SYNC_IFC_NUM) {
			kfree(data);
			break;
		}

		retval = 0;
		break;
	default:
		dev_err(&serial->dev->dev,
			"unknown number of interfaces: %d\n", nintf);
		kfree(data);
		retval = -ENODEV;
	}

	/* Set serial->private if not returning -ENODEV */
	if (retval != -ENODEV)
		usb_set_serial_data(serial, data);
	return retval;
}

static void qc_release(struct usb_serial *serial)
{
	struct usb_wwan_intf_private *priv = usb_get_serial_data(serial);

	dbg("%s", __func__);

	/* Call usb_wwan release & free the private data allocated in qcprobe */
	usb_wwan_release(serial);
	usb_set_serial_data(serial, NULL);
	kfree(priv);
}

static struct usb_serial_driver qcdevice = {
	.driver = {
		.owner     = THIS_MODULE,
		.name      = "qcserial",
	},
	.description         = "Qualcomm USB modem",
	.id_table            = id_table,
	.num_ports           = 1,
	.probe               = qcprobe,
	.open		     = usb_wwan_open,
	.close		     = usb_wwan_close,
	.write		     = usb_wwan_write,
	.write_room	     = usb_wwan_write_room,
	.chars_in_buffer     = usb_wwan_chars_in_buffer,
	.throttle            = usb_wwan_throttle,
	.unthrottle          = usb_wwan_unthrottle,
	.attach		     = usb_wwan_startup,
	.disconnect	     = usb_wwan_disconnect,
	.release	     = qc_release,
#ifdef CONFIG_PM
	.suspend	     = usb_wwan_suspend,
	.resume		     = usb_wwan_resume,
#endif
};

static struct usb_serial_driver * const serial_drivers[] = {
	&qcdevice, NULL
};

module_usb_serial_driver(qcdriver, serial_drivers);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");

module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug enabled or not");
