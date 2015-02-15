#ifndef _TPI_H_
#define _TPI_H_
/***************************************************************************
 *
 *   SiI9234 - MHL Transmitter Driver
 *
 * Copyright (C) 2011 SiliconImage, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *****************************************************************************/
#include <mach/mhl.h>
#include "TypeDefs.h"
#include <mach/board.h>
bool TPI_Init(void);				/* Document purpose, usage */
void TPI_Poll(void);			/* Document purpose, usage, rename */
byte Status_Query(void);
void D2ToD3(void);
bool tpi_get_hpd_state(void);

#define	POWER_STATE_D3				3
#define	POWER_STATE_D0_NO_MHL		2
#define	POWER_STATE_D0_MHL			0
#define	POWER_STATE_FIRST_INIT		0xFF

typedef struct {
	uint8_t reqStatus;
	uint8_t retryCount;
	uint8_t command;
	uint8_t offsetData;
	uint8_t length;
	union {
		unsigned char msgData[16];
		unsigned char	*pdatabytes;
	} payload_u;
} cbus_req_t;

bool SiiMhlTxChipInitialize(void);
void SiiMhlTxGetEvents(uint8_t *event, uint8_t *eventParameter);
#define	APP_DEMO_RCP_SEND_KEY_CODE	0x41
void ProcessRcp(uint8_t event, uint8_t eventParameter);
void SiiMhlTxDeviceIsr(void);
bool	SiiMhlTxDrvSendCbusCommand(cbus_req_t *pReq);
void	SiiMhlTxDrvTmdsControl(bool enable);
void	SiiMhlTxDrvNotifyEdidChange(void);
bool SiiMhlTxReadDevcap(uint8_t offset);
extern void    SiiMhlTxInitialize(bool interruptDriven, uint8_t pollIntervalMs);
extern	void	SiiMhlTxNotifyDsHpdChange(uint8_t dsHpdStatus);
extern	void	SiiMhlTxNotifyConnection(bool mhlConnected);
extern	void	SiiMhlTxMscCommandDone(uint8_t data1);
extern	void	SiiMhlTxGotMhlIntr(uint8_t intr_0, uint8_t intr_1);
extern	void	SiiMhlTxGotMhlStatus(uint8_t status_0, uint8_t status_1);
extern	void	SiiMhlTxGotMhlMscMsg(uint8_t subCommand, uint8_t cmdData);
extern	void	SiiMhlTxGotMhlWriteBurst(uint8_t *spadArray);
extern	bool	IsMHLConnection(void);
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL_HDCP_SUPPORT
extern	void	hdcp_deauthenticate(void);
#endif
extern	void	fill_black_screen(void);
extern  void	update_mhl_status(bool isMHL, enum usb_connect_type statMHL);
extern  void	sii9234_disableIRQ(void);
extern  int	sii9234_power_vote(bool enable);
#endif
