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

#include <linux/input.h>
#include <mach/board.h>
#include	"defs.h"
#include	"sii_mhltx_api.h"
#include	"sii_mhltx.h"
#include	"mhl_defs.h"

#include "TPI_Access.h"
#include "TPI.h"
#include "i2c_master_sw.h"
#include "inc/si_datatypes.h"

static	mhlTx_config_t	mhlTxConfig;

static	bool 	SiiMhlTxRapkSend(void);
/* static	void		MhlTxDriveStates(void); */
static	void		MhlTxResetStates(void);
static	bool		MhlTxSendMscMsg(uint8_t command, uint8_t cmdData);


#define NUM_CBUS_EVENT_QUEUE_EVENTS 5
typedef struct _CBusQueue_t {
    uint8_t head;
    uint8_t tail;
    cbus_req_t queue[NUM_CBUS_EVENT_QUEUE_EVENTS];
} CBusQueue_t, *PCBusQueue_t;


#define QUEUE_SIZE(x) (sizeof(x.queue)/sizeof(x.queue[0]))
#define MAX_QUEUE_DEPTH(x) (QUEUE_SIZE(x) - 1)
#define QUEUE_DEPTH(x) ((x.head <= x.tail)?(x.tail-x.head):(QUEUE_SIZE(x)-x.head+x.tail))
#define QUEUE_FULL(x) (QUEUE_DEPTH(x) >= MAX_QUEUE_DEPTH(x))

#define ADVANCE_QUEUE_HEAD(x) { x.head = (x.head < MAX_QUEUE_DEPTH(x))?(x.head+1):0; }
#define ADVANCE_QUEUE_TAIL(x) { x.tail = (x.tail < MAX_QUEUE_DEPTH(x))?(x.tail+1):0; }

#define RETREAT_QUEUE_HEAD(x) { x.head = (x.head > 0)?(x.head-1):MAX_QUEUE_DEPTH(x); }
CBusQueue_t CBusQueue;


#define PutNextCBusTransaction(req) PutNextCBusTransactionImpl(req)

bool PutNextCBusTransactionImpl(cbus_req_t *pReq)
{
	if (QUEUE_FULL(CBusQueue))
		return false;

	CBusQueue.queue[CBusQueue.tail] = *pReq;
	ADVANCE_QUEUE_TAIL(CBusQueue);
	return true;
}

static void SiiMhlTxTmdsEnable(void)
{
	TPI_DEBUG_PRINT(("MhlTx:SiiMhlTxTmdsEnable\n"));
	/*if (MHL_RSEN & mhlTxConfig.mhlHpdRSENflags) {
		TPI_DEBUG_PRINT(("\tMHL_RSEN\n"));
		if (MHL_HPD & mhlTxConfig.mhlHpdRSENflags) {
			TPI_DEBUG_PRINT(("\t\tMHL_HPD\n"));
			if (MHL_STATUS_PATH_ENABLED & mhlTxConfig.status_1) {
				TPI_DEBUG_PRINT(("\t\t\tMHL_STATUS_PATH_ENABLED\n")); */
				SiiMhlTxDrvTmdsControl(true);
	/*		}
		}
	} */
}

static bool SiiMhlTxSetStatus(uint8_t regToWrite, uint8_t value)
{
	cbus_req_t	req;

	req.retryCount  = 2;
	req.command     = MHL_WRITE_STAT;
	req.offsetData  = regToWrite;
	req.payload_u.msgData[0]  = value;

	TPI_DEBUG_PRINT(("MhlTx:SiiMhlTxSetStatus\n"));
	return( SiiMhlTxDrvSendCbusCommand( &req  ));
}

static bool SiiMhlTxSendLinkMode(void)
{
	return SiiMhlTxSetStatus(MHL_STATUS_REG_LINK_MODE, mhlTxConfig.linkMode);
}



#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
static uint8_t Chk_Dongle_Step;
#endif

void SiiMhlTxInitialize(bool interruptDriven, uint8_t pollIntervalMs)
{
	TPI_DEBUG_PRINT(("MhlTx: SiiMhlTxInitialize\n"));
	mhlTxConfig.interruptDriven = interruptDriven;
	mhlTxConfig.pollIntervalMs  = pollIntervalMs;

	MhlTxResetStates();

#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
	Chk_Dongle_Step = 0;
#endif

}

void SiiMhlTxGetEvents(uint8_t *event, uint8_t *eventParameter)
{
	TPI_Poll();
	MhlTxDriveStates();

	*event = MHL_TX_EVENT_NONE;
	*eventParameter = 0;

	if (mhlTxConfig.mhlConnectionEvent) {
		TPI_DEBUG_PRINT(("MhlTx: SiiMhlTxGetEvents mhlConnectionEvent\n"));

		mhlTxConfig.mhlConnectionEvent = false;
		*event          = mhlTxConfig.mhlConnected;
		*eventParameter	= mhlTxConfig.mscFeatureFlag;

		if (MHL_TX_EVENT_DISCONNECTION == mhlTxConfig.mhlConnected) {
			MhlTxResetStates();
		} else if (MHL_TX_EVENT_CONNECTION == mhlTxConfig.mhlConnected) {
			WriteByteCBUS(0x13, 0x30) ;
			WriteByteCBUS(0x14, 0x01) ;
			WriteByteCBUS(0x12, 0x08) ;
			WriteByteCBUS(0x13, 0x20) ;
			WriteByteCBUS(0x14, 0x01) ;
			WriteByteCBUS(0x12, 0x08) ;
		}
	} else if (mhlTxConfig.mscMsgArrived) {
		TPI_DEBUG_PRINT(("MhlTx: SiiMhlTxGetEvents MSC MSG <%02X, %02X>\n",
							(int) (mhlTxConfig.mscMsgSubCommand),
							(int) (mhlTxConfig.mscMsgData)));

		mhlTxConfig.mscMsgArrived = false;

		switch (mhlTxConfig.mscMsgSubCommand) {
		case	MHL_MSC_MSG_RAP:
				if (0x10 == mhlTxConfig.mscMsgData)
					SiiMhlTxDrvTmdsControl(true);
				else if (0x11 == mhlTxConfig.mscMsgData)
					SiiMhlTxDrvTmdsControl(false);
				SiiMhlTxRapkSend();
				break;

		case	MHL_MSC_MSG_RCP:
				if (MHL_LOGICAL_DEVICE_MAP & rcpSupportTable[mhlTxConfig.mscMsgData & 0x7F]) {
					*event          = MHL_TX_EVENT_RCP_RECEIVED;
					*eventParameter = mhlTxConfig.mscMsgData;
				} else {
					mhlTxConfig.mscSaveRcpKeyCode = mhlTxConfig.mscMsgData;
					SiiMhlTxRcpeSend(RCPE_INEEFECTIVE_KEY_CODE);
				}
				break;

		case	MHL_MSC_MSG_RCPK:
				*event = MHL_TX_EVENT_RCPK_RECEIVED;
				*eventParameter = mhlTxConfig.mscMsgData;
				break;

		case	MHL_MSC_MSG_RCPE:
				*event = MHL_TX_EVENT_RCPE_RECEIVED;
				*eventParameter = mhlTxConfig.mscMsgData;
				break;

		case	MHL_MSC_MSG_RAPK:
				break;

		default:
				break;
		}
	}
}

void	MhlTxDriveStates(void)
{

	switch (mhlTxConfig.mscState) {
	case MSC_STATE_BEGIN:
			SiiMhlTxReadDevcap(0x02);
			break;
	case MSC_STATE_POW_DONE:
			SiiMhlTxReadDevcap(0x0A);
			break;
	case MSC_STATE_IDLE:
	case MSC_STATE_RCP_READY:
			break;
	default:
			break;

	}
}

#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
bool Tri_state_dongle_GPIO0(void)
{
	bool result = true;

	I2C_WriteByte(CBUS_SLAVE_ADDR,0x13, 0x33);       // enable backdoor access
	I2C_WriteByte(CBUS_SLAVE_ADDR,0x14, 0x80);
	I2C_WriteByte(CBUS_SLAVE_ADDR,0x12, 0x08);

	I2C_WriteByte(CBUS_SLAVE_ADDR, 0xc0,  0xff);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0xc1,  0x7F);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0xc2,  0xFF);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x20,  0x02);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x13,  0x48);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x12,  0x10);

	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x13, 0x33);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x14, 0x00);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x12, 0x08);

	return result;
}

void Low_dongle_GPIO0(void)
{

	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x13, 0x33);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x14, 0x80);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x12, 0x08);

	I2C_WriteByte(CBUS_SLAVE_ADDR, 0xc0, 0xff);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0xc1, 0x7F);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0xc2, 0xFC);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x20, 0x02);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x13, 0x48);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x12, 0x10);

	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x13, 0x33);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x14, 0x00);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x12, 0x08);
}

void SiiMhlTxMscDetectCharger(uint8_t data1)
{
	/* connected to TV; and TV has power output (default 0.5A min ) */
	if ((data1 & 0x13) == 0x11) {
		mscCmdInProgress = false;
		mhlTxConfig.mscState = MSC_STATE_POW_DONE;
		Chk_Dongle_Step = 0;

		if (data1 & 0x10) {
			/* [bit4] POW ==1=AC charger attached */
			TPI_DEBUG_PRINT(("1000mA charger!!\n"));
			if (gStatusMHL != CONNECT_TYPE_MHL_AC) {
				gStatusMHL = CONNECT_TYPE_MHL_AC;
				ProcessMhlStatus(true, false);
			}
		} else {
			TPI_DEBUG_PRINT(("500mA charger!!\n"));
			if (gStatusMHL != CONNECT_TYPE_USB) {
				gStatusMHL = CONNECT_TYPE_USB;
				ProcessMhlStatus(true, false);
			}
		}
	}

	/* 03=dongle */
	if ((data1 & 0x03) == 0x03) {

		if (Chk_Dongle_Step == 0) {

			if (!Tri_state_dongle_GPIO0()) {
				TPI_DEBUG_PRINT(("%s: faild to set as GPI\n", __func__));
				mscCmdInProgress = false;
				return;
			}

			/* GPIO0_state=3; */

			mscCmdInProgress = false;
			SiiMhlTxReadDevcap(0x02);
			mscCmdInProgress = false;
			Chk_Dongle_Step = 1;
			return;
		}

		if (Chk_Dongle_Step == 1) {

			mscCmdInProgress = false;

			if (data1 & 0x10) {
				/* Turn off phone Vbus output ; */
				Low_dongle_GPIO0();

				/* GPIO0_state = 0;  */
#if 0
				SiiMhlTxReadDevcap(0x02);
#endif
				mscCmdInProgress = false;
				Chk_Dongle_Step = 2;
				return;
			} else {

				Chk_Dongle_Step = 0;

				mhlTxConfig.mscState = MSC_STATE_POW_DONE;

				/* turn on phone VBUS output.; */
				TPI_DEBUG_PRINT(("No charger!!\n"));

				if (gStatusMHL != CONNECT_TYPE_INTERNAL) {
					gStatusMHL = CONNECT_TYPE_INTERNAL;
					ProcessMhlStatus(true, false);
				}
			}
		}

		if (Chk_Dongle_Step == 2) {

			mhlTxConfig.mscState = MSC_STATE_POW_DONE;
			mscCmdInProgress = false;

			Chk_Dongle_Step = 0;

			if (data1 & 0x10) {
				/* Set charge battery current=AC charger rating-100mA ; */

				/* Enable battery charger; &*/
				TPI_DEBUG_PRINT(("1000mA charger!!\n"));
				if (gStatusMHL != CONNECT_TYPE_MHL_AC) {
					gStatusMHL = CONNECT_TYPE_MHL_AC;
					ProcessMhlStatus(true, false);
				}

			} else {
				/* turn off phone VBUS output; */
				TPI_DEBUG_PRINT(("500mA charger!!\n"));
				if (gStatusMHL != CONNECT_TYPE_USB) {
					gStatusMHL = CONNECT_TYPE_USB;
					ProcessMhlStatus(true, false);
				}
			}
		}
	}
}
#endif
void	SiiMhlTxMscCommandDone(uint8_t data1)
{
	TPI_DEBUG_PRINT(("MhlTx: SiiMhlTxMscCommandDone. data1 =%02X\n", (int) data1));

	if ((MHL_READ_DEVCAP == mhlTxConfig.mscLastCommand) &&
		(0x02 == mhlTxConfig.mscLastOffset)) {

#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
		SiiMhlTxMscDetectCharger(data1);
#else
		mhlTxConfig.mscState	= MSC_STATE_POW_DONE;
#endif
	} else if ((MHL_READ_DEVCAP == mhlTxConfig.mscLastCommand) &&
			(0x0A == mhlTxConfig.mscLastOffset)) {
		mhlTxConfig.mscState	= MSC_STATE_RCP_READY;

		mhlTxConfig.mscFeatureFlag = data1;

		mhlTxConfig.mhlConnectionEvent = true;
		mhlTxConfig.mhlConnected = MHL_TX_EVENT_RCP_READY;

		mhlTxConfig.mscLastCommand = 0;
		mhlTxConfig.mscLastOffset  = 0;

		TPI_DEBUG_PRINT(("MhlTx: Peer's Feature Flag =%02X\n", (int) data1));
	} else if (MHL_MSC_MSG_RCPE == mhlTxConfig.mscMsgLastCommand) {
		if (SiiMhlTxRcpkSend(mhlTxConfig.mscSaveRcpKeyCode)) {
			mhlTxConfig.mscMsgLastCommand = 0;
			mhlTxConfig.mscMsgLastData = 0;
		}
	}
}

void	SiiMhlTxGotMhlMscMsg(uint8_t subCommand, uint8_t cmdData)
{
	mhlTxConfig.mscMsgArrived		= true;
	mhlTxConfig.mscMsgSubCommand	= subCommand;
	mhlTxConfig.mscMsgData			= cmdData;
}

void	SiiMhlTxGotMhlIntr(uint8_t intr_0, uint8_t intr_1)
{
	TPI_DEBUG_PRINT(("MhlTx: INTERRUPT Arrived. %02X, %02X\n", (int) intr_0, (int) intr_1));
	if (BIT_0 & intr_0)
		SiiMhlTxReadDevcap(0x02);
	else if (BIT_1 & intr_1)
		SiiMhlTxDrvNotifyEdidChange();
}
bool SiiMhlTxSetPathEn(void )
{
	TPI_DEBUG_PRINT(("MhlTx:SiiMhlTxSetPathEn\n"));
    SiiMhlTxTmdsEnable();
    mhlTxConfig.linkMode |= MHL_STATUS_PATH_ENABLED;     // update local copy
    return SiiMhlTxSetStatus( MHL_STATUS_REG_LINK_MODE, mhlTxConfig.linkMode);
}

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxClrPathEn
//
bool SiiMhlTxClrPathEn( void )
{
	TPI_DEBUG_PRINT(("MhlTx: SiiMhlTxClrPathEn\n"));
    SiiMhlTxDrvTmdsControl( false );
    mhlTxConfig.linkMode &= ~MHL_STATUS_PATH_ENABLED;    // update local copy
    return SiiMhlTxSetStatus( MHL_STATUS_REG_LINK_MODE, mhlTxConfig.linkMode);
}

void	SiiMhlTxGotMhlStatus(uint8_t status_0, uint8_t status_1)
{
	uint8_t StatusChangeBitMask0,StatusChangeBitMask1;
    StatusChangeBitMask0 = status_0 ^ mhlTxConfig.status_0;
    StatusChangeBitMask1 = status_1 ^ mhlTxConfig.status_1;
	// Remember the event.   (other code checks the saved values, so save the values early, but not before the XOR operations above)
	mhlTxConfig.status_0 = status_0;
	mhlTxConfig.status_1 = status_1;

	TPI_DEBUG_PRINT(("MhlTx: STATUS Arrived.%02X,%02X\n", (int) status_0, (int) status_1));

	if(MHL_STATUS_DCAP_RDY & StatusChangeBitMask0)
	{
		if(MHL_STATUS_DCAP_RDY & status_0)
		{
			mhlTxConfig.mscState	 = MSC_STATE_BEGIN;
		}
	}

	if(MHL_STATUS_PATH_ENABLED & StatusChangeBitMask1)
    {
        TPI_DEBUG_PRINT(("MhlTx: PATH_EN changed\n"));
		if(MHL_STATUS_PATH_ENABLED & status_1)
		{
			SiiMhlTxSetPathEn();
		}
		else
		{
			SiiMhlTxClrPathEn();
		}
	}
}

bool SiiMhlTxRcpSend(uint8_t rcpKeyCode)
{
	if ((0 == (BIT_0 & mhlTxConfig.mscFeatureFlag)) || (MSC_STATE_RCP_READY != mhlTxConfig.mscState))
		return	false;
	return	(MhlTxSendMscMsg(MHL_MSC_MSG_RCP, rcpKeyCode));
}

bool SiiMhlTxRcpkSend(uint8_t rcpKeyCode)
{
	return	(MhlTxSendMscMsg(MHL_MSC_MSG_RCPK, rcpKeyCode));
}

static	bool SiiMhlTxRapkSend(void)
{
	return	(MhlTxSendMscMsg(MHL_MSC_MSG_RAPK, 0));
}

bool SiiMhlTxRcpeSend(uint8_t rcpeErrorCode)
{
	return	(MhlTxSendMscMsg(MHL_MSC_MSG_RCPE, rcpeErrorCode));
}

bool SiiMhlTxReadDevcap(uint8_t offset)
{
	cbus_req_t	req;

	req.command     = mhlTxConfig.mscLastCommand = MHL_READ_DEVCAP;
	req.offsetData  = mhlTxConfig.mscLastOffset  = offset;
	return	(SiiMhlTxDrvSendCbusCommand(&req));
}

static bool MhlTxSendMscMsg(uint8_t command, uint8_t cmdData)
{
	cbus_req_t	req;
	uint8_t		ccode;

	req.command     = MHL_MSC_MSG;
	req.payload_u.msgData[0]  = mhlTxConfig.mscMsgLastCommand = command;
	req.payload_u.msgData[1]  = mhlTxConfig.mscMsgLastData    = cmdData;
	ccode = SiiMhlTxDrvSendCbusCommand(&req);
	return	((bool)ccode);
}

void	SiiMhlTxNotifyConnection(bool mhlConnected)
{
	mhlTxConfig.mhlConnectionEvent = true;

	mhlTxConfig.mscState	 = MSC_STATE_IDLE;
	if (mhlConnected) {
		mhlTxConfig.mhlConnected = MHL_TX_EVENT_CONNECTION;
		mhlTxConfig.mhlHpdRSENflags |= MHL_RSEN;
		SiiMhlTxTmdsEnable();
		SiiMhlTxSendLinkMode();
	} else {
		mhlTxConfig.mhlConnected = MHL_TX_EVENT_DISCONNECTION;
		mhlTxConfig.mhlHpdRSENflags &= ~MHL_RSEN;
	}
}

void	SiiMhlTxNotifyDsHpdChange(uint8_t dsHpdStatus)
{
	if (0 == dsHpdStatus) {
		TPI_DEBUG_PRINT(("MhlTx: Disable TMDS - fake\n"));
		/*mhlTxConfig.mhlHpdRSENflags &= ~MHL_HPD;*/
		/*SiiMhlTxDrvTmdsControl(false);*/
	} else {
		TPI_DEBUG_PRINT(("MhlTx: Enable TMDS\n"));
		TPI_DEBUG_PRINT(("MhlTx: DsHPD ON\n"));
		mhlTxConfig.mhlHpdRSENflags |= MHL_HPD;
		SiiMhlTxTmdsEnable();
	}
}

static void MhlTxResetStates(void)
{
#if 0
	mhlTxConfig.mhlConnectionEvent	= false;
	mhlTxConfig.mhlConnected		= MHL_TX_EVENT_DISCONNECTION;
	mhlTxConfig.mscMsgArrived		= false;
	mhlTxConfig.mscState			= MSC_STATE_IDLE;
#endif

	mhlTxConfig.mhlConnectionEvent	= false;
	mhlTxConfig.mhlConnected		= MHL_TX_EVENT_DISCONNECTION;
	mhlTxConfig.mhlHpdRSENflags    &= ~(MHL_RSEN | MHL_HPD);
	mhlTxConfig.mscMsgArrived		= false;

	mhlTxConfig.status_0            = 0;
	mhlTxConfig.status_1            = 0;
	mhlTxConfig.connectedReady      = 0;
	mhlTxConfig.linkMode            = 3;
	mhlTxConfig.cbusReferenceCount  = 0;
	mhlTxConfig.miscFlags           = 0;
	mhlTxConfig.mscLastCommand      = 0;
	mhlTxConfig.mscMsgLastCommand   = 0;
}
static uint8_t ProcessRcpKeyCode(uint8_t rcpKeyCode)
{
	uint8_t rcpkStatus = rcpKeyCode;
	TPI_DEBUG_PRINT(("RCP Key Code: 0x%02X\n", (int)rcpKeyCode));
	switch (rcpKeyCode) {
	case MHD_RCP_CMD_SELECT:
		TPI_DEBUG_PRINT(("\n MHD_RCP_CMD_SELECT received %d\n\n", (int)rcpKeyCode));
		sii9234_send_keyevent(KEY_ENTER, 0);
		break;
	case MHD_RCP_CMD_UP:
		TPI_DEBUG_PRINT(("\n MHD_RCP_CMD_UP received %d\n\n", (int)rcpKeyCode));
		sii9234_send_keyevent(KEY_UP, 0);
		break;
	case MHD_RCP_CMD_DOWN:
		TPI_DEBUG_PRINT(("\n MHD_RCP_CMD_DOWN received %d\n\n", (int)rcpKeyCode));
		sii9234_send_keyevent(KEY_DOWN, 0);
		break;
	case MHD_RCP_CMD_LEFT:
		TPI_DEBUG_PRINT(("\n MHD_RCP_CMD_LEFT received %d\n\n", (int)rcpKeyCode));
		sii9234_send_keyevent(KEY_LEFT, 0);
		break;
	case MHD_RCP_CMD_RIGHT:
		TPI_DEBUG_PRINT(("\n MHD_RCP_CMD_RIGHT received %d\n\n", (int)rcpKeyCode));
		sii9234_send_keyevent(KEY_RIGHT, 0);
		break;
	case MHD_RCP_CMD_ROOT_MENU:
		TPI_DEBUG_PRINT(("\n MHD_RCP_CMD_ROOT_MENU received %d\n\n", (int)rcpKeyCode));
		sii9234_send_keyevent(KEY_HOME, 0);
		break;
	case MHD_RCP_CMD_EXIT:
		TPI_DEBUG_PRINT(("\n MHD_RCP_CMD_EXIT received %d\n\n", (int)rcpKeyCode));
		sii9234_send_keyevent(KEY_BACK, 0);
		break;
	case MHD_RCP_CMD_PLAY:
		TPI_DEBUG_PRINT(("\n MHD_RCP_CMD_PLAY received %d\n\n", (int)rcpKeyCode));
		sii9234_send_keyevent(KEY_PLAY, 0);
		break;
	case MHD_RCP_CMD_STOP:
		TPI_DEBUG_PRINT(("\n MHD_RCP_CMD_STOP received %d\n\n", (int)rcpKeyCode));
		sii9234_send_keyevent(KEY_STOP, 0);
		break;
	case MHD_RCP_CMD_PAUSE:
		TPI_DEBUG_PRINT(("\n MHD_RCP_CMD_PAUSE received %d\n\n", (int)rcpKeyCode));
		sii9234_send_keyevent(KEY_PLAYPAUSE, 0);
		break;
	case MHD_RCP_CMD_REWIND:
		TPI_DEBUG_PRINT(("\n MHD_RCP_CMD_REWIND received %d\n\n", (int)rcpKeyCode));
		sii9234_send_keyevent(KEY_REWIND, 0);
		break;
	case MHD_RCP_CMD_FAST_FWD:
		TPI_DEBUG_PRINT(("\n MHD_RCP_CMD_FAST_FWD received %d\n\n", (int)rcpKeyCode));
		sii9234_send_keyevent(KEY_FASTFORWARD, 0);
		break;
	default:
		break;
    }

    return (rcpkStatus);
}

void    ProcessRcp(uint8_t event, uint8_t eventParameter)
{
	uint8_t         rcpKeyCode;

	switch (event) {
	case    MHL_TX_EVENT_DISCONNECTION:
		TPI_DEBUG_PRINT(("App: Got event = MHL_TX_EVENT_DISCONNECTION\n"));
		break;
	case    MHL_TX_EVENT_CONNECTION:
		TPI_DEBUG_PRINT(("App: Got event = MHL_TX_EVENT_CONNECTION\n"));
		break;
	case    MHL_TX_EVENT_RCP_READY:
#if 0
		rcpKeyCode = APP_DEMO_RCP_SEND_KEY_CODE;
		TPI_DEBUG_PRINT(("App: Got event = MHL_TX_EVENT_RCP_READY...Sending RCP (%02X)\n", (int) rcpKeyCode));
		if ((0 == (BIT_0 & eventParameter)))
			TPI_DEBUG_PRINT(("App: Peer does NOT support RCP\n"));
		if ((0 == (BIT_1 & eventParameter)))
			TPI_DEBUG_PRINT(("App: Peer does NOT support RAP\n"));
		if ((0 == (BIT_2 & eventParameter)))
			TPI_DEBUG_PRINT(("App: Peer does NOT support WRITE_BURST\n"));
		if (SiiMhlTxRcpSend(rcpKeyCode)) {
			TPI_DEBUG_PRINT(("App: SiiMhlTxRcpSend (%02X)\n", (int) rcpKeyCode));
			TPI_DEBUG_PRINT(("Stupid coding check\n"));
		} else
			TPI_DEBUG_PRINT(("App: SiiMhlTxRcpSend (%02X) Returned Failure.\n", (int) rcpKeyCode));
#endif
		break;
	case    MHL_TX_EVENT_RCP_RECEIVED:
		TPI_DEBUG_PRINT(("App: Received an RCP key code = %02X\n", eventParameter));
		rcpKeyCode = ProcessRcpKeyCode(eventParameter);
		SiiMhlTxRcpkSend((int) rcpKeyCode);
		break;
	case    MHL_TX_EVENT_RCPK_RECEIVED:
		TPI_DEBUG_PRINT(("App: Received an RCPK = %02X\n", (int)eventParameter));
		break;
	case    MHL_TX_EVENT_RCPE_RECEIVED:
		TPI_DEBUG_PRINT(("App: Received an RCPE = %02X\n", (int)eventParameter));
		break;
	case    MHL_TX_EVENT_NONE:
		break;
	default:
		TPI_DEBUG_PRINT(("App: Got event = %02X, eventParameter = %02X\n", (int)event, (int)eventParameter));
		break;
	}
}
