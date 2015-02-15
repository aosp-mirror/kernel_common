/*
 * u_serial.h - interface to USB gadget "serial port"/TTY utilities
 *
 * Copyright (C) 2008 David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

#ifndef __U_SERIAL_H
#define __U_SERIAL_H

#include <linux/usb/composite.h>
#include <linux/usb/cdc.h>

enum fserial_func_type {
	USB_FSER_FUNC_NONE,
	USB_FSER_FUNC_SERIAL,
	USB_FSER_FUNC_MODEM,
	USB_FSER_FUNC_MODEM_MDM,
	USB_FSER_FUNC_ACM,
	USB_FSER_FUNC_AUTOBOT,
};

/*
 * One non-multiplexed "serial" I/O port ... there can be several of these
 * on any given USB peripheral device, if it provides enough endpoints.
 *
 * The "u_serial" utility component exists to do one thing:  manage TTY
 * style I/O using the USB peripheral endpoints listed here, including
 * hookups to sysfs and /dev for each logical "tty" device.
 *
 * REVISIT at least ACM could support tiocmget() if needed.
 *
 * REVISIT someday, allow multiplexing several TTYs over these endpoints.
 */
struct gserial {
	struct usb_function		func;

	/* port is managed by gserial_{connect,disconnect} */
	struct gs_port			*ioport;

	struct usb_ep			*in;
	struct usb_ep			*out;

	/* REVISIT avoid this CDC-ACM support harder ... */
	struct usb_cdc_line_coding port_line_coding;	/* 9600-8-N-1 etc */
	u16				serial_state;

	/* control signal callbacks*/
	unsigned int (*get_dtr)(struct gserial *p);
	unsigned int (*get_rts)(struct gserial *p);

	/* notification callbacks */
	void (*connect)(struct gserial *p);
	void (*disconnect)(struct gserial *p);
	int (*send_break)(struct gserial *p, int duration);
	unsigned int (*send_carrier_detect)(struct gserial *p, unsigned int);
	unsigned int (*send_ring_indicator)(struct gserial *p, unsigned int);
	int (*send_modem_ctrl_bits)(struct gserial *p, int ctrl_bits);

	/* notification changes to modem */
	void (*notify_modem)(void *gser, u8 portno, int ctrl_bits);
};

/* utilities to allocate/free request and buffer */
struct usb_request *gs_alloc_req(struct usb_ep *ep, unsigned len, gfp_t flags);
void gs_free_req(struct usb_ep *, struct usb_request *req);

/* port setup/teardown is handled by gadget driver */
int gserial_setup(struct usb_gadget *g, unsigned n_ports);
void gserial_cleanup(void);

/* connect/disconnect is handled by individual functions */
int gserial_connect(struct gserial *, u8 port_num);
void gserial_disconnect(struct gserial *);

/* sdio related functions */
int gsdio_setup(struct usb_gadget *g, unsigned n_ports);
int gsdio_connect(struct gserial *, u8 port_num);
void gsdio_disconnect(struct gserial *, u8 portno);

int gsmd_setup(struct usb_gadget *g, unsigned n_ports);
int gsmd_connect(struct gserial *, u8 port_num);
void gsmd_disconnect(struct gserial *, u8 portno);

/* functions are bound to configurations by a config or gadget driver */
int acm_bind_config(struct usb_configuration *c, u8 port_num);
int gser_bind_config(struct usb_configuration *c, u8 port_num);
int obex_bind_config(struct usb_configuration *c, u8 port_num);

#endif /* __U_SERIAL_H */
