/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __U_BAM_DATA_H
#define __U_BAM_DATA_H

#include <mach/usb_gadget_xport.h>

enum function_type {
	USB_FUNC_ECM,
	USB_FUNC_MBIM,
	USB_FUNC_RNDIS,
};

struct data_port {
	struct usb_composite_dev	*cdev;
	struct usb_function		*func;
	struct usb_ep			*in;
	struct usb_ep			*out;
};

void bam_data_disconnect(struct data_port *gr, u8 port_num);

int bam_data_connect(struct data_port *gr, u8 port_num,
	enum transport_type trans, u8 src_connection_idx,
	u8 dst_connection_idx, enum function_type func);

int bam_data_setup(unsigned int no_bam2bam_port);

int bam_data_destroy(unsigned int no_bam2bam_port);

void bam_data_suspend(u8 port_num);

void bam_data_resume(u8 port_num);

#endif /* __U_BAM_DATA_H */
