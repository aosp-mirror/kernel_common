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


typedef struct {
	bool		interruptDriven;
	uint8_t	pollIntervalMs;
	uint8_t	mscState;

	uint8_t	status_0;
	uint8_t	status_1;
	uint8_t     connectedReady;
	uint8_t     linkMode;
	uint8_t     mhlHpdStatus;
	uint8_t     mhlRequestWritePending;

	bool		mhlConnectionEvent;
	uint8_t	mhlConnected;

	uint8_t     mhlHpdRSENflags;

	bool		mscMsgArrived;
	uint8_t	mscMsgSubCommand;
	uint8_t	mscMsgData;

	uint8_t	mscFeatureFlag;

	uint8_t	cbusReferenceCount;
	uint8_t	mscLastCommand;
	uint8_t	mscLastOffset;
	uint8_t	mscLastData;

	uint8_t	mscMsgLastCommand;
	uint8_t	mscMsgLastData;
	uint8_t	mscSaveRcpKeyCode;

	uint8_t     localScratchPad[16];
	uint8_t     miscFlags;
	uint8_t	rapFlags;
} mhlTx_config_t;

enum {
	MSC_STATE_IDLE		= 0x00,
	MSC_STATE_BEGIN		= 0x01,
	MSC_STATE_POW_DONE	= 0x02,
	MSC_STATE_RCP_READY	= 0x03
};

typedef enum {
     MHL_HPD            = 0x01
   , MHL_RSEN           = 0x02
} MhlHpdRSEN_e;

extern void	MhlTxDriveStates(void);
extern void sii9234_send_keyevent(uint32_t key, uint32_t type);
extern	uint8_t	rcpSupportTable[];
#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
extern void ProcessMhlStatus(bool, bool);
extern uint8_t mscCmdInProgress;
extern enum usb_connect_type gStatusMHL;
#endif

