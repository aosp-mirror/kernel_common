/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#ifndef __WCD9310_SLIMSLAVE_H_
#define __WCD9310_SLIMSLAVE_H_

#include <linux/slimbus/slimbus.h>
#include <linux/mfd/wcd9xxx/core.h>


/*
 *  client is expected to give port ids in the range of
 *  1-10 for pre Taiko Tx ports and 1-16 for Taiko
 *  1-7 for pre Taiko Rx ports and 1-16 for Tako,
 *  we need to add offset for getting the absolute slave
 *  port id before configuring the HW
 */
#define TABLA_SB_PGD_MAX_NUMBER_OF_TX_SLAVE_DEV_PORTS 10
#define TAIKO_SB_PGD_MAX_NUMBER_OF_TX_SLAVE_DEV_PORTS 16

#define SLIM_MAX_TX_PORTS TAIKO_SB_PGD_MAX_NUMBER_OF_TX_SLAVE_DEV_PORTS

#define TABLA_SB_PGD_OFFSET_OF_RX_SLAVE_DEV_PORTS \
	TABLA_SB_PGD_MAX_NUMBER_OF_TX_SLAVE_DEV_PORTS
#define TAIKO_SB_PGD_OFFSET_OF_RX_SLAVE_DEV_PORTS \
	TAIKO_SB_PGD_MAX_NUMBER_OF_TX_SLAVE_DEV_PORTS

#define TABLA_SB_PGD_MAX_NUMBER_OF_RX_SLAVE_DEV_PORTS 7
#define TAIKO_SB_PGD_MAX_NUMBER_OF_RX_SLAVE_DEV_PORTS 13

#define SLIM_MAX_RX_PORTS TAIKO_SB_PGD_MAX_NUMBER_OF_RX_SLAVE_DEV_PORTS

#define TABLA_SB_PGD_RX_PORT_MULTI_CHANNEL_0_START_PORT_ID \
	TABLA_SB_PGD_OFFSET_OF_RX_SLAVE_DEV_PORTS
#define TAIKO_SB_PGD_RX_PORT_MULTI_CHANNEL_0_START_PORT_ID \
	TAIKO_SB_PGD_OFFSET_OF_RX_SLAVE_DEV_PORTS

#define TABLA_SB_PGD_RX_PORT_MULTI_CHANNEL_0_END_PORT_ID 16
#define TAIKO_SB_PGD_RX_PORT_MULTI_CHANNEL_0_END_PORT_ID 31

#define TABLA_SB_PGD_TX_PORT_MULTI_CHANNEL_1_END_PORT_ID 9
#define TAIKO_SB_PGD_TX_PORT_MULTI_CHANNEL_1_END_PORT_ID 15

/* below details are taken from SLIMBUS slave SWI */
#define SB_PGD_PORT_BASE 0x000

#define SB_PGD_PORT_CFG_BYTE_ADDR(offset, port_num) \
		(SB_PGD_PORT_BASE + offset + (1 * port_num))

#define SB_PGD_TX_PORT_MULTI_CHANNEL_0(port_num) \
		(SB_PGD_PORT_BASE + 0x100 + 4*port_num)
#define SB_PGD_TX_PORT_MULTI_CHANNEL_0_START_PORT_ID   0
#define SB_PGD_TX_PORT_MULTI_CHANNEL_0_END_PORT_ID     7

#define SB_PGD_TX_PORT_MULTI_CHANNEL_1(port_num) \
		(SB_PGD_PORT_BASE + 0x101 + 4*port_num)
#define SB_PGD_TX_PORT_MULTI_CHANNEL_1_START_PORT_ID   8

#define SB_PGD_RX_PORT_MULTI_CHANNEL_0(offset, port_num) \
		(SB_PGD_PORT_BASE + offset + (4 * port_num))

/* slave port water mark level
 *   (0: 6bytes, 1: 9bytes, 2: 12 bytes, 3: 15 bytes)
 */
#define SLAVE_PORT_WATER_MARK_6BYTES  0
#define SLAVE_PORT_WATER_MARK_9BYTES  1
#define SLAVE_PORT_WATER_MARK_12BYTES 2
#define SLAVE_PORT_WATER_MARK_15BYTES 3
#define SLAVE_PORT_WATER_MARK_SHIFT 1
#define SLAVE_PORT_ENABLE           1
#define SLAVE_PORT_DISABLE          0
#define WATER_MARK_VAL \
	((SLAVE_PORT_WATER_MARK_12BYTES << SLAVE_PORT_WATER_MARK_SHIFT) | \
	 (SLAVE_PORT_ENABLE))
#define BASE_CH_NUM 128


int wcd9xxx_init_slimslave(struct wcd9xxx *wcd9xxx,
			   u8 wcd9xxx_pgd_la,
			   unsigned int tx_num, unsigned int *tx_slot,
			   unsigned int rx_num, unsigned int *rx_slot);

int wcd9xxx_deinit_slimslave(struct wcd9xxx *wcd9xxx);

int wcd9xxx_cfg_slim_sch_rx(struct wcd9xxx *wcd9xxx,
			    struct list_head *wcd9xxx_ch_list,
			    unsigned int rate, unsigned int bit_width,
			    u16 *grph);
int wcd9xxx_cfg_slim_sch_tx(struct wcd9xxx *wcd9xxx,
			    struct list_head *wcd9xxx_ch_list,
			    unsigned int rate, unsigned int bit_width,
				u16 *grph);
int wcd9xxx_close_slim_sch_rx(struct wcd9xxx *wcd9xxx,
			      struct list_head *wcd9xxx_ch_list, u16 grph);
int wcd9xxx_close_slim_sch_tx(struct wcd9xxx *wcd9xxx,
			      struct list_head *wcd9xxx_ch_list, u16 grph);
int wcd9xxx_get_channel(struct wcd9xxx *wcd9xxx,
			unsigned int *rx_ch,
			unsigned int *tx_ch);
int wcd9xxx_get_slave_port(unsigned int ch_num);
int wcd9xxx_disconnect_port(struct wcd9xxx *wcd9xxx,
			    struct list_head *wcd9xxx_ch_list, u16 grph);
int wcd9xxx_rx_vport_validation(u32 port_id,
				struct list_head *codec_dai_list);
int wcd9xxx_tx_vport_validation(u32 vtable, u32 port_id,
				struct wcd9xxx_codec_dai_data *codec_dai,
				u32 num_codec_dais);
#endif /* __WCD9310_SLIMSLAVE_H_ */
