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

void 	SiiMhlTxInitialize(bool interruptDriven, uint8_t pollIntervalMs);

void SiiMhlTxGetEvents(uint8_t *event, uint8_t *eventParameter);
#define		MHL_TX_EVENT_NONE		0x00
#define		MHL_TX_EVENT_DISCONNECTION	0x01
#define		MHL_TX_EVENT_CONNECTION		0x02
#define		MHL_TX_EVENT_RCP_READY		0x03
#define		MHL_TX_EVENT_RCP_RECEIVED	0x04
#define		MHL_TX_EVENT_RCPK_RECEIVED	0x05
#define		MHL_TX_EVENT_RCPE_RECEIVED	0x06
void 	SiiMhlTxDeviceIsr(void);
bool SiiMhlTxRcpSend(uint8_t rcpKeyCode);
bool SiiMhlTxRcpkSend(uint8_t rcpKeyCode);
bool SiiMhlTxRcpeSend(uint8_t rcpeErrorCode);
bool SiiMhlTxSetPathEn(void);
bool SiiMhlTxClrPathEn(void);

extern	void	AppMhlTxDisableInterrupts(void);
extern	void	AppMhlTxRestoreInterrupts(void);
extern	void	AppVbusControl(bool powerOn);

