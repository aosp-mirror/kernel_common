/*

SiI8558 / SiI8240 Linux Driver

Copyright (C) 2013 Silicon Image, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation version 2.
This program is distributed AS-IS WITHOUT ANY WARRANTY of any
kind, whether express or implied; INCLUDING without the implied warranty
of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.  See
the GNU General Public License for more details at http://www.gnu.org/licenses/gpl-2.0.html.

*/

#ifndef _MHL_RCP_INPUTDEV_H_
#define _MHL_RCP_INPUTDEV_H_

struct mhl_dev_context;
typedef struct _rcp_keymap_t{
	unsigned	multicode		:1;
	unsigned	press_and_hold_key	:1;
	unsigned	reserved		:6;
	uint16_t	map[2];
	uint8_t		rcp_support;
}rcp_keymap_t;
#define	MHL_DEV_LD_DISPLAY			(0x01 << 0)
#define	MHL_DEV_LD_VIDEO			(0x01 << 1)
#define	MHL_DEV_LD_AUDIO			(0x01 << 2)
#define	MHL_DEV_LD_MEDIA			(0x01 << 3)
#define	MHL_DEV_LD_TUNER			(0x01 << 4)
#define	MHL_DEV_LD_RECORD			(0x01 << 5)
#define	MHL_DEV_LD_SPEAKER			(0x01 << 6)
#define	MHL_DEV_LD_GUI				(0x01 << 7)

#define	MHL_LOGICAL_DEVICE_MAP	(MHL_DEV_LD_GUI)

#define	MHL_NUM_RCP_KEY_CODES	0x80	// inclusive
extern rcp_keymap_t rcpSupportTable[MHL_NUM_RCP_KEY_CODES];


int generate_rcp_input_event(struct mhl_dev_context *dev_context, uint8_t rcp_keycode);

uint8_t init_rcp_input_dev(struct mhl_dev_context *dev_context);

void destroy_rcp_input_dev(struct mhl_dev_context *dev_context);
void rcp_input_dev_one_time_init(struct mhl_dev_context *dev_context);

#endif /* #ifndef _MHL_RCP_INPUTDEV_H_ */
