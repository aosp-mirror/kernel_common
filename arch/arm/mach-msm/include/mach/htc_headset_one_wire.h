/*
 *
 * /arch/arm/mach-msm/include/mach/htc_headset_one_wire.h
 *
 * HTC 1-wire headset driver.
 *
 * Copyright (C) 2012 HTC, Inc.
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

#define		INIT_CMD_INT_5MS			0x35
#define		INIT_CMD_INT_8MS			0x65
#define		INIT_CMD_INT_10MS			0x75
#define		INIT_CMD_INT_13MS			0xA5
#define		INIT_CMD_INT_15MS			0xB5 
#define		INIT_CMD_INT_18MS			0xE5
#define		INIT_CMD_INT_30MS			0xF5
#define 	QUERY_AID				0xD5
#define		QUERY_CONFIG				0xE5
#define		QUERY_KEYCODE				0xF5

 
struct htc_headset_1wire_platform_data {
	unsigned int tx_level_shift_en;
	unsigned int uart_sw;
	unsigned int uart_tx;
	unsigned int uart_rx;
	unsigned int remote_press;
	char one_wire_remote[6]; /* Key code for press and release */
	char onewire_tty_dev[15];

};

struct htc_35mm_1wire_info {
	struct htc_headset_1wire_platform_data pdata;
	char aid;
	struct wake_lock hs_wake_lock;
	struct mutex mutex_lock;
};
