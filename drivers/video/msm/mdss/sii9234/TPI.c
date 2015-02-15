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

#include "defs.h"
#include "TypeDefs.h"
#include "mhl_defs.h"
#include "TPI_Access.h"
#include "TPI.h"
#include "Util.h"
#include "i2c_master_sw.h"
#include "inc/si_datatypes.h"
#include <linux/jiffies.h>
#include <mach/board.h>
#include "sii_mhltx.h"

static unsigned long rsenCheckTimeout = 0;
static unsigned long deglitchTimeout = 0;
static int rsenCount = 0;
static int cbusErrCount = 0;
static int WR_Dcap_Rdy_Int_Done = false;/* new in V100109 */
static bool IsEstablished = false;/* new in V100109 */
extern bool g_bProbe;
extern bool disable_interswitch;
extern u8 dbg_drv_str_a3, dbg_drv_str_a6, dbg_drv_str_on;


static	uint8_t	fwPowerState = POWER_STATE_FIRST_INIT;
#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT

/* To remember current MHL connection status */
enum usb_connect_type gStatusMHL = CONNECT_TYPE_UNKNOWN;
static bool gConnectMHL = false;
#endif

#ifdef	APPLY_SCDT_SAFETY
static	uint8_t	gotFifoUnderRunOverRun = 0;
#endif
static	bool	deglitchingRsenNow = false;

uint8_t		mscCmdInProgress;	/* false when it is okay to send a new command */
static	uint8_t	dsHpdStatus = 0;
static  uint8_t contentOn = 0;
/* HTC board parameters */

#define	I2C_READ_MODIFY_WRITE(saddr, offset, mask)	I2C_WriteByte(saddr, offset, I2C_ReadByte(saddr, offset) | (mask));

#define	SET_BIT(saddr, offset, bitnumber)		I2C_READ_MODIFY_WRITE(saddr, offset, (1<<bitnumber))
#define	CLR_BIT(saddr, offset, bitnumber)		I2C_WriteByte(saddr, offset, I2C_ReadByte(saddr, offset) & ~(1<<bitnumber))
#define	DISABLE_DISCOVERY				CLR_BIT(TPI_SLAVE_ADDR, 0x90, 0);
#define	ENABLE_DISCOVERY				SET_BIT(TPI_SLAVE_ADDR, 0x90, 0);
#define	INTR_2_DESIRED_MASK            (BIT_0)
#define	UNMASK_INTR_2_INTERRUPTS       I2C_WriteByte(TPI_SLAVE_ADDR, 0x76, INTR_2_DESIRED_MASK)
#define	MASK_INTR_2_INTERRUPTS         I2C_WriteByte(TPI_SLAVE_ADDR, 0x76, 0x00)
#define	INTR_4_DESIRED_MASK				(BIT_0 | BIT_2 | BIT_3 | BIT_4 | BIT_5 | BIT_6)
#define	UNMASK_INTR_4_INTERRUPTS		I2C_WriteByte(TPI_SLAVE_ADDR, 0x78, INTR_4_DESIRED_MASK)
#define	MASK_INTR_4_INTERRUPTS			I2C_WriteByte(TPI_SLAVE_ADDR, 0x78, 0x00)
#define	INTR_1_DESIRED_MASK				(BIT_6 | BIT_5)
#define	UNMASK_INTR_1_INTERRUPTS		I2C_WriteByte(TPI_SLAVE_ADDR, 0x75, INTR_1_DESIRED_MASK)
#define	MASK_INTR_1_INTERRUPTS			I2C_WriteByte(TPI_SLAVE_ADDR, 0x75, 0x00)
#define	INTR_CBUS1_DESIRED_MASK			(BIT_2 | BIT_3 | BIT_4 | BIT_5 | BIT_6)
#define	UNMASK_CBUS1_INTERRUPTS			I2C_WriteByte(CBUS_SLAVE_ADDR, 0x09, INTR_CBUS1_DESIRED_MASK)
#define	MASK_CBUS1_INTERRUPTS			I2C_WriteByte(CBUS_SLAVE_ADDR, 0x09, 0x00)

#define	INTR_CBUS2_DESIRED_MASK			(BIT_0 | BIT_2 | BIT_3)	/* mw20110922 enable write burst int */
#define	UNMASK_CBUS2_INTERRUPTS			I2C_WriteByte(CBUS_SLAVE_ADDR, 0x1F, INTR_CBUS2_DESIRED_MASK)
#define	MASK_CBUS2_INTERRUPTS			I2C_WriteByte(CBUS_SLAVE_ADDR, 0x1F, 0x00)
#define I2C_INACCESSIBLE -1		/* new in V100109 */
#define I2C_ACCESSIBLE 1			/* new in v100109 */

static	int	Int4Isr(void);
static	void	Int1RsenIsr(void);
static	void	MhlCbusIsr(void);
static	void	DeglitchRsenLow(void);

static	void	CbusReset(void);
static	void	SwitchToD0(void);
static	void	SwitchToD3(void);
static	void	WriteInitialRegisterValues(void);
static	void	InitCBusRegs(void);
static	void	ForceUsbIdSwitchOpen(void);
static	void	ReleaseUsbIdSwitchOpen(void);
static	void	MhlTxDrvProcessConnection(void);
static	void	MhlTxDrvProcessDisconnection(void);
static	void	ApplyDdcAbortSafety(void);

static  bool	HDCPSuccess;
#ifdef CONFIG_CABLE_DETECT_ACCESSORY
void    ProcessMhlStatus(bool, bool);
#endif

extern void sii9234_request_abort(void);

#define	APPLY_PLL_RECOVERY

#ifdef APPLY_PLL_RECOVERY
static  void    SiiMhlTxDrvRecovery(void);
#endif

byte Status_Query(void)
{
	return fwPowerState;
}

static bool ProductID_Read(void)
{
	byte devID = 0x00;
	word wID = 0x0000;
	int i;

	I2C_WriteByte(TPI_SLAVE_ADDR, 0xC7, 0x80);
	for (i = 0; i < 3; i++) {
		devID = I2C_ReadByte(TPI_SLAVE_ADDR, 0x03);
		wID = devID;
		devID = I2C_ReadByte(TPI_SLAVE_ADDR, 0x02);
		wID = ((wID << 8) & 0xFF00) | (devID & 0x00FF);
		TPI_DEBUG_PRINT(("SiI %04X\n", (int) wID));
		if (wID == SiI9234_PRODUCT_ID)
			return TRUE;
	}
	TPI_DEBUG_PRINT(("Unsupported TX\n"));
	return FALSE;
}

static void TxHW_Reset(void)
{
	sii9234_reset();
}

bool TPI_Init(void)
{
	fwPowerState = POWER_STATE_FIRST_INIT;
	WR_Dcap_Rdy_Int_Done = false;
	cbusErrCount = 0;
	IsEstablished = false;
	HDCPSuccess = false;
	if(!g_bProbe) {
		TPI_DEBUG_PRINT(("Drv: Sii9244 not ready, this is called from cable detection\n"));
		return false;
	}
	SiiMhlTxInitialize(true, 0);

	TxHW_Reset();
	TPI_DEBUG_PRINT(("Drv: SiiMhlTxChipInitialize: %02X44\n", (int)I2C_ReadByte(TPI_SLAVE_ADDR, 0x03)));
	if (!ProductID_Read())
		return FALSE;

	WriteInitialRegisterValues();

	I2C_WriteByte(TPI_SLAVE_ADDR, 0x71, INTR_1_DESIRED_MASK);
	UNMASK_INTR_1_INTERRUPTS;
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x76, 02);
	UNMASK_INTR_4_INTERRUPTS;
	SwitchToD3();

	return true;
}

void	TPI_Poll(void)
{
	if (POWER_STATE_D0_MHL != fwPowerState) {

		Int4Isr();
#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
		gStatusMHL = CONNECT_TYPE_UNKNOWN;
#endif
	} else if (POWER_STATE_D0_MHL == fwPowerState) {

		rsenCount++;
		Int1RsenIsr();
		if(deglitchingRsenNow)
		{
			TPI_DEBUG_PRINT(("Drv: deglitchingRsenNow.\n"));
			DelayMS(100);
			DeglitchRsenLow();
			if( POWER_STATE_D0_MHL != fwPowerState )
				return;
		}
#ifdef	APPLY_PLL_RECOVERY
		if ((I2C_ReadByte(TPI_SLAVE_ADDR, 0x09) & BIT_2)) {
			TPI_DEBUG_PRINT(("PowerState=D0_MHL, chk TxDrvRecovery\n"));
			/*if ((MHL_STATUS_PATH_ENABLED & linkMode) && (BIT_6 &dsHpdStatus) &&(contentOn))	{*/
				SiiMhlTxDrvRecovery();
			/*}*/
		}

#endif
		MhlCbusIsr();
	}

}

void SiiMhlTxDrvReleaseUpstreamHPDControl(void)
{
	/* Un-force HPD (it was kept low, now propagate to source
	   let HPD float by clearing reg_hpd_out_ovr_en */
	CLR_BIT(TPI_SLAVE_ADDR, 0x79, 4);
	TPI_DEBUG_PRINT(("Drv:%d Upstream HPD released.\n", (int)__LINE__));
}


void	SiiMhlTxDrvTmdsControl(bool enable)
{
	if (enable) {
		SET_BIT(TPI_SLAVE_ADDR, 0x80, 4);
		TPI_DEBUG_PRINT(("Drv: TMDS Output Enabled\n"));
		SiiMhlTxDrvReleaseUpstreamHPDControl();  /* this triggers an EDID read */
	} else {
		CLR_BIT(TPI_SLAVE_ADDR, 0x80, 4);
		TPI_DEBUG_PRINT(("Drv: TMDS Ouput Disabled\n"));
	}
}

void	SiiMhlTxDrvNotifyEdidChange(void)
{
	TPI_DEBUG_PRINT(("Drv: SiiMhlTxDrvNotifyEdidChange\n"));

	/*SET_BIT(TPI_SLAVE_ADDR, 0x79, 4);*/
	ReadModifyWriteTPI(0x79, BIT_5 | BIT_4, BIT_4);
	TPI_DEBUG_PRINT(("Drv: Upstream HPD Acquired - driven low.\n"));
	/*CLR_BIT(TPI_SLAVE_ADDR, 0x79, 5);*/
	SET_BIT(TPI_SLAVE_ADDR, 0x79, 5);


	DelayMS(110);

	/* release HPD back to high by reg_hpd_out_ovr_val = HIGH
	SET_BIT(PAGE_0_0X72, 0x79, 5);*/
	CLR_BIT(TPI_SLAVE_ADDR, 0x79, 4);
	TPI_DEBUG_PRINT(("Drv: Upstream HPD released.\n"));

}

bool SiiMhlTxDrvSendCbusCommand(cbus_req_t *pReq)
{
	bool  success = true;
	uint8_t i, startbit;


	if ((POWER_STATE_D0_MHL != fwPowerState) || (mscCmdInProgress)) {
		TPI_DEBUG_PRINT(("Error: Drv: fwPowerState: %02X, or mscCmdInProgress = %d\n",
				(int) fwPowerState,
				(int) mscCmdInProgress));

		return false;
	}
	mscCmdInProgress	= true;

	TPI_DEBUG_PRINT(("Drv: Sending MSC command %02X, %02X, %02X, %02X\n",
			(int)pReq->command,
			(int)(pReq->offsetData), (int)pReq->payload_u.msgData[0], (int)pReq->payload_u.msgData[1]));

	WriteByteCBUS(0x13, pReq->offsetData);
	WriteByteCBUS(0x14, pReq->payload_u.msgData[0]);
	startbit = 0x00;
	switch (pReq->command) {
	case MHL_SET_INT:
		WriteByteCBUS((0x13 & 0xFF), pReq->offsetData + 0x20);
		startbit = (0x01 << 3);
		break;

	case MHL_WRITE_STAT:
		//WriteByteCBUS((0x13 & 0xFF), pReq->offsetData + 0x30);
		startbit = (0x01 << 3);
		break;

	case MHL_READ_DEVCAP:
		startbit = (0x01 << 2);
		break;

	case MHL_GET_STATE:
	case MHL_GET_VENDOR_ID:
	case MHL_SET_HPD:
	case MHL_CLR_HPD:
	case MHL_GET_SC1_ERRORCODE:
	case MHL_GET_DDC_ERRORCODE:
	case MHL_GET_MSC_ERRORCODE:
	case MHL_GET_SC3_ERRORCODE:
		WriteByteCBUS((0x13 & 0xFF), pReq->command);
		startbit = (0x01 << 0);
		break;

	case MHL_MSC_MSG:
		WriteByteCBUS((0x15 & 0xFF), pReq->payload_u.msgData[1]);
		WriteByteCBUS((0x13 & 0xFF), pReq->command);
		startbit = (0x01 << 1);
		break;

	case MHL_WRITE_BURST:
		WriteByteCBUS((0x13 & 0xFF), pReq->offsetData + 0x40);
		WriteByteCBUS((0x20 & 0xFF), pReq->length - 1);

		for (i = 0; i < pReq->length; i++)
			WriteByteCBUS((0xC0 & 0xFF) + i, pReq->payload_u.msgData[i]);

		startbit = (0x01 << 4);
		break;

	default:
		success = false;
		break;
	}
	if (success)
		WriteByteCBUS(0x12 & 0xFF, startbit);
	return (success);
}

void Int1ProcessRsen(uint8_t rsen)
{
	if (rsen == 0x00) {
		TPI_DEBUG_PRINT(("Drv: Int1RsenIsr: Start T_SRC_RSEN_DEGLITCH (%d ms) before disconnection\n",
								 (int)(T_SRC_RSEN_DEGLITCH)));
		deglitchTimeout = jiffies + HZ/10;
		deglitchingRsenNow = true;
	} else if (deglitchingRsenNow) {
		TPI_DEBUG_PRINT(("Drv: Ignore now, RSEN is high. This was a glitch.\n"));
		deglitchingRsenNow = false;
	}
}


void	Int1RsenIsr(void)
{
	uint8_t		reg71 = I2C_ReadByte(TPI_SLAVE_ADDR, 0x71);
	uint8_t		rsen  = I2C_ReadByte(TPI_SLAVE_ADDR, 0x09) & BIT_2;

	if ((reg71 & BIT_5)/* ||
		((false == deglitchingRsenNow) && (rsen == 0x00))*/) {
		TPI_DEBUG_PRINT(("Drv: Got INTR_1: reg71 = %02X, rsen = %02X\n", (int) reg71, (int) rsen));
		Int1ProcessRsen(rsen);
		/* Clear MDI_RSEN interrupt */
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x71, BIT_5);
		UNMASK_INTR_1_INTERRUPTS;
	}
#if 0
	DelayMS(100);
	if ((I2C_ReadByte(TPI_SLAVE_ADDR, 0x09) & BIT_2) == 0)
		MhlTxDrvProcessDisconnection();
	else if ((false == deglitchingRsenNow) && (rsen == 0x00)) {
		TPI_DEBUG_PRINT(("Drv: Ignore now, RSEN is high. This was a glitch.\n"));
		deglitchingRsenNow = false;
	} else if (deglitchingRsenNow) {
		TPI_DEBUG_PRINT(("Drv: Ignore now coz (reg71 & BIT_5) has been cleared. This was a glitch.\n"));
		Int1ProcessRsen(rsen);
	}
#endif
}

static void DeglitchRsenLow(void)
{
	TPI_DEBUG_PRINT(("Drv: DeglitchRsenLow RSEN <72:09[2]> = %02X\n", (int) (I2C_ReadByte(TPI_SLAVE_ADDR, 0x09))));

	if ((I2C_ReadByte(TPI_SLAVE_ADDR, 0x09) & BIT_2) == 0x00) {
		TPI_DEBUG_PRINT(("Drv: RSEN is Low.\n"));

		DelayMS(100);
		if ((POWER_STATE_D0_MHL == fwPowerState)  && time_after(jiffies, deglitchTimeout)) {
			TPI_DEBUG_PRINT(("Drv: Disconnection due to RSEN Low\n"));
			deglitchingRsenNow = false;

			DISABLE_DISCOVERY;
			ENABLE_DISCOVERY;

			dsHpdStatus &= ~BIT_6;  /* cable disconnect implies downstream HPD low */

			WriteByteCBUS(0x0D, dsHpdStatus);
			SiiMhlTxNotifyDsHpdChange(0);
			MhlTxDrvProcessDisconnection();
			MhlTxDriveStates();
		}
	} else
		deglitchingRsenNow = false;
}

static void WriteInitialRegisterValues(void)
{
	TPI_DEBUG_PRINT(("Drv: WriteInitialRegisterValues\n"));

	if (sii9234_get_ci2ca())
		I2C_WriteByte(0x7E, 0x3D, 0x3F);
	else
		I2C_WriteByte(0x7A, 0x3D, 0x3F);
	I2C_WriteByte(HDMI_SLAVE_ADDR, 0x11, 0x01);
	I2C_WriteByte(HDMI_SLAVE_ADDR, 0x12, 0x15);
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x08, 0x35);
	CbusReset();								/* mw20110922 match v100108 ; to pass CTS 3.3.5.3 & 3.3.7.1 */

	I2C_WriteByte(HDMI_SLAVE_ADDR, 0x10, 0xC1);
	I2C_WriteByte(HDMI_SLAVE_ADDR, 0x17, 0x03);
	I2C_WriteByte(HDMI_SLAVE_ADDR, 0x1A, 0x20);
	I2C_WriteByte(HDMI_SLAVE_ADDR, 0x22, 0x8A);
	I2C_WriteByte(HDMI_SLAVE_ADDR, 0x23, 0x6A);
	I2C_WriteByte(HDMI_SLAVE_ADDR, 0x24, 0xAA);
	I2C_WriteByte(HDMI_SLAVE_ADDR, 0x25, 0xCA);
	I2C_WriteByte(HDMI_SLAVE_ADDR, 0x26, 0xEA);
	I2C_WriteByte(HDMI_SLAVE_ADDR, 0x4C, 0xA0);
	I2C_WriteByte(HDMI_SLAVE_ADDR, 0x4D, 0x00);

	I2C_WriteByte(HDMI_SLAVE_ADDR, 0x50, 0x11);	/* mw20110925 match w/ v100109 */
	I2C_WriteByte(HDMI_SLAVE_ADDR, 0x51, 0x09);	/* mw20110925 match w/ v100109 */
	I2C_WriteByte(HDMI_SLAVE_ADDR, 0x52, 0x11);	/* mw20110925 match w/ v100109 */

	/* I2C_WriteByte(TPI_SLAVE_ADDR, 0x80, 0x34);	mw20110925 match w/ v100109 */
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x80, 0x24);
	I2C_WriteByte(HDMI_SLAVE_ADDR, 0x45, 0x44);
	I2C_WriteByte(HDMI_SLAVE_ADDR, 0x31, 0x0A);
	I2C_WriteByte(TPI_SLAVE_ADDR, 0xA0, 0xD0);
	I2C_WriteByte(TPI_SLAVE_ADDR, 0xA1, 0xFC);

	/* no build flag so far */
	if(/* board_build_flag() != SHIP_BUILD && */dbg_drv_str_on){
		I2C_WriteByte(TPI_SLAVE_ADDR, 0xA3, dbg_drv_str_a3);
		I2C_WriteByte(TPI_SLAVE_ADDR, 0xA6, dbg_drv_str_a6);
	}else{
		I2C_WriteByte(TPI_SLAVE_ADDR, 0xA3, 0xEB);
		I2C_WriteByte(TPI_SLAVE_ADDR, 0xA6, 0x0C);
	}

	I2C_WriteByte(TPI_SLAVE_ADDR, 0x2B, 0x01);


	ReadModifyWriteTPI(0x90, BIT_3 | BIT_2, BIT_2);/* mw20110925 match w/ v100109 */
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x91, 0xA5);



	I2C_WriteByte(TPI_SLAVE_ADDR, 0x94, 0x77); /* mw20110925 if same as v100108 0x77, then MHL won't established */


	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x31, I2C_ReadByte(CBUS_SLAVE_ADDR, 0x31) | 0x0c);

	I2C_WriteByte(TPI_SLAVE_ADDR, 0xA5, 0xA0);
	TPI_DEBUG_PRINT(("Drv: MHL 1.0 Compliant Clock\n"));

	/*  mw20110925 match w/ v100109
	if (sii9234_get_ci2ca())
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x95, 0x35);
	else
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x95, 0x31); */
	if(!disable_interswitch)
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x95, 0x71); /*  mw20110925 match w/ v100109 */

	I2C_WriteByte(TPI_SLAVE_ADDR, 0x97, 0x00);

	/* ReadModifyWriteTPI(0x95, BIT_6, BIT_6);  mw20110922 match w v100108 ; Force USB ID switch to open */

	if(!disable_interswitch) {
		WriteByteTPI(0x92, 0x86);
		WriteByteTPI(0x93, 0x8C);
	}


	ReadModifyWriteTPI(0x79, BIT_5 | BIT_4, BIT_4);

	DelayMS(25);
	if(!disable_interswitch) {
		ReadModifyWriteTPI(0x95, BIT_6, 0x00);
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x90, 0x27);
	}
	/*  mw20110925 match w/ v100109
		mw20110922 match v100108 to pass CTS 3.3.5.3 & 3.3.7.1
	CbusReset();*/

	InitCBusRegs();



	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x3C, 0xB4) ; 	/* mw20110921 add to match w v100108 */
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x2E, 0x15) ; 	/* mw20110921 add to match w v100108 handle CEC abort items */

	/* Enable Auto soft reset on SCDT = 0 */
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x05, 0x04);
	/* HDMI Transcode mode enable */
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x0D, 0x1C);
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x75, 0x60);	/* mw20110921 match w/ v100108 to pass CTS xxxxx [6]=enable Rsen change int */
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x76, 0x02);	/* enable Tclk stable  change int mw20110921 match w/ v100108 to pass CTS xxxxx */
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x3C, 0x02);	/* mw20110616 enable TPI Rsen int */
}

static void InitCBusRegs(void)
{
	uint8_t		regval;

	TPI_DEBUG_PRINT(("Drv: InitCBusRegs\n"));
	/* I2C_WriteByte(CBUS_SLAVE_ADDR, 0x07, 0x36);  new default is for MHL mode ; mw20110921 match v100108 DDC transition max value */
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x07, 0xF2);

	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x40, 0x03);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x42, 0x06);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x36, 0x0C);

	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x3D, 0xFD);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x1C, 0x01);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x1D, 0x0F);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x44, 0x02);


	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x80, 0x00);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x81, MHL_VERSION);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x82, 0x02);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x83, 0x01);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x84, 0x6F);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x85, 0x01 | 0x10);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x86, 0x01);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x87, 0);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x88, MHL_LOGICAL_DEVICE_MAP);

	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x89, 0x0F);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x8A, (BIT_0 | BIT_1 | BIT_2));
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x8B, 0);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x8C, 0);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x8D, 0x10);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x8E, 0x33);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x8F, 0);

	regval = I2C_ReadByte(CBUS_SLAVE_ADDR, 0x31);
	regval = (regval | 0x0C);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x31, regval);

	regval = I2C_ReadByte(CBUS_SLAVE_ADDR, 0x22);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x22, (regval&0xF0)|0x0D);/* mw20110921 match v100108 */

	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x30, 0x01);

}

uint8_t ReadHPD(void)
{
	uint8_t RetVal = 1;
	char buffer[2];

	buffer[0] = 0x0D;
	if (fwPowerState == POWER_STATE_D0_MHL)
		RetVal = sii9234_I2C_RxData(CBUS_SLAVE_ADDR, buffer, 1);

	TPI_DEBUG_PRINT(("Drv: ReadHPD: %X\n", (int)buffer[0]));

	if (RetVal == 0) {
		if ((buffer[0]&0x40) == 0x40)
			return 1;
	}

    return 0;
}



void SetHDCPStatus(bool Status)
{
	HDCPSuccess = Status;
}


static void ForceUsbIdSwitchOpen(void)
{
	if(!disable_interswitch) {
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x90, 0x26);
		ReadModifyWriteTPI(0x95, BIT_6, BIT_6);
		WriteByteTPI(0x92, 0x86);
		ReadModifyWriteTPI(0x79, BIT_5 | BIT_4, BIT_4);
	}
}

static void ReleaseUsbIdSwitchOpen(void)
{
	if(!disable_interswitch) {
		DelayMS(50);
		ReadModifyWriteTPI(0x95, BIT_6, 0x00);
		ENABLE_DISCOVERY;
	}
}

void CbusWakeUpPulseGenerator(void)
{
	uint8_t		regval;

	TPI_DEBUG_PRINT(("Drv: CbusWakeUpPulseGenerator\n"));

	regval = I2C_ReadByte(TPI_SLAVE_ADDR, 0x96);
	/* I2C_WriteByte(TPI_SLAVE_ADDR, 0x96, (I2C_ReadByte(TPI_SLAVE_ADDR, 0x96) | 0xC0));*/
	regval |= 0xC0;
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x96, regval);
	DelayMS(T_SRC_WAKE_PULSE_WIDTH_1 - 2);

	/* I2C_WriteByte(TPI_SLAVE_ADDR, 0x96, (I2C_ReadByte(TPI_SLAVE_ADDR, 0x96) & 0x3F));*/
	regval &= 0x3F;
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x96, regval);
	DelayMS(T_SRC_WAKE_PULSE_WIDTH_1 - 2);

	/* I2C_WriteByte(TPI_SLAVE_ADDR, 0x96, (I2C_ReadByte(TPI_SLAVE_ADDR, 0x96) | 0xC0));*/
	regval |= 0xC0;
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x96, regval);
	DelayMS(T_SRC_WAKE_PULSE_WIDTH_1 - 2);

	/* I2C_WriteByte(TPI_SLAVE_ADDR, 0x96, (I2C_ReadByte(TPI_SLAVE_ADDR, 0x96) & 0x3F));*/
	regval &= 0x3F;
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x96, regval);
	DelayMS(T_SRC_WAKE_PULSE_WIDTH_2 - 2);

	/* I2C_WriteByte(TPI_SLAVE_ADDR, 0x96, (I2C_ReadByte(TPI_SLAVE_ADDR, 0x96) | 0xC0));*/
	regval |= 0xC0;
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x96, regval);
	DelayMS(T_SRC_WAKE_PULSE_WIDTH_1 - 2);

	/* I2C_WriteByte(TPI_SLAVE_ADDR, 0x96, (I2C_ReadByte(TPI_SLAVE_ADDR, 0x96) & 0x3F));*/
	regval &= 0x3F;
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x96, regval);
	DelayMS(T_SRC_WAKE_PULSE_WIDTH_1 - 2);

	/* I2C_WriteByte(TPI_SLAVE_ADDR, 0x96, (I2C_ReadByte(TPI_SLAVE_ADDR, 0x96) | 0xC0));*/
	regval |= 0xC0;
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x96, regval);
	DelayMS(T_SRC_WAKE_PULSE_WIDTH_1 - 2);

	/* I2C_WriteByte(TPI_SLAVE_ADDR, 0x96, (I2C_ReadByte(TPI_SLAVE_ADDR, 0x96) & 0x3F));*/
	regval &= 0x3F;
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x96, regval);

	DelayMS(T_SRC_WAKE_TO_DISCOVER);

	TPI_DEBUG_PRINT(("Drv: CbusWakeUpPulseGenerator - end\n"));
}

static	void	ApplyDdcAbortSafety()
{
	uint8_t		bTemp, bPost;


	WriteByteCBUS(0x29, 0xFF);
	bTemp = ReadByteCBUS(0x29);
	DelayMS(3);
	bPost = ReadByteCBUS(0x29);

	if ((bPost > (bTemp + 50))) {
		TPI_DEBUG_PRINT(("Drv: Applying DDC Abort Safety(SWWA 18958)\n"));

		SET_BIT(TPI_SLAVE_ADDR, 0x05, 3);
		CLR_BIT(TPI_SLAVE_ADDR, 0x05, 3);

		InitCBusRegs();

		ForceUsbIdSwitchOpen();
		ReleaseUsbIdSwitchOpen();

		MhlTxDrvProcessDisconnection();
	}
}

void	ProcessRgnd(void)
{
	uint8_t		reg99RGNDRange;
	reg99RGNDRange = I2C_ReadByte(TPI_SLAVE_ADDR, 0x99) & 0x03;
	TPI_DEBUG_PRINT(("Drv: RGND Reg 99 = %02X : ", (int)reg99RGNDRange));

	/*Remove Sii9244 Cable detection, since HTC cable detection had detected MHL dongle succesfully*/
	/*if (0x02 == reg99RGNDRange || (0x01 == reg99RGNDRange)) {*/
			SET_BIT(TPI_SLAVE_ADDR, 0x95, 5);

			TPI_DEBUG_PRINT(("Drv: Waiting T_SRC_VBUS_CBUS_TO_STABLE (%d ms)\n", (int)T_SRC_VBUS_CBUS_TO_STABLE));
			DelayMS(T_SRC_VBUS_CBUS_TO_STABLE);
			CbusWakeUpPulseGenerator();
	/*} else{
			TPI_DEBUG_PRINT(("Drv: USB impedance. Set for USB Established = %02X.\n", (int)reg99RGNDRange));

			CLR_BIT(TPI_SLAVE_ADDR, 0x95, 5);
	}*/
}
void change_driving_strength(byte reg_a3, byte reg_a6)
{
	/* no build flag so far */
	if(/*board_build_flag() != SHIP_BUILD && */ dbg_drv_str_on) {
		TPI_DEBUG_PRINT(("Drv: %s debuging driving str 0xA3 = %x\n", __func__, dbg_drv_str_a3));
		TPI_DEBUG_PRINT(("Drv: %s debuging driving str 0xA6 = %x\n", __func__, dbg_drv_str_a6));
		return;
	}
	TPI_DEBUG_PRINT(("Drv: %s 0xA3 = %x 0xA6 = %x\n",
		__func__, reg_a3,reg_a6 ));
	I2C_WriteByte(TPI_SLAVE_ADDR, 0xA3, reg_a3);
	I2C_WriteByte(TPI_SLAVE_ADDR, 0xA6, reg_a6);
}


bool	IsD0Mode(void)
{
	if (POWER_STATE_D0_MHL != fwPowerState)
		return false;
	else
		return true;
}

void	SwitchToD0(void)
{
	TPI_DEBUG_PRINT(("Drv: Switch To Full power mode (D0)\n"));


	WriteInitialRegisterValues();

	I2C_WriteByte(TPI_SLAVE_ADDR, 0x90, 0x25);

	fwPowerState = POWER_STATE_D0_NO_MHL;
}

static void SwitchToD3(void)
{
	if (POWER_STATE_D3 != fwPowerState) {

		TPI_DEBUG_PRINT(("Drv: Switch To D3: pinAllowD3 = %d\n", 1));

		ForceUsbIdSwitchOpen();

		ReadModifyWriteTPI(0x93, BIT_7 | BIT_6 | BIT_5 | BIT_4, 0);

		ReadModifyWriteTPI(0x94, BIT_1 | BIT_0, 0);

		ReleaseUsbIdSwitchOpen();

		ReadModifyWriteTPI(0x79, BIT_5 | BIT_4, BIT_4);

		I2C_WriteByte(HDMI_SLAVE_ADDR, 0x01, 0x03);

		CLR_BIT(0x7A, 0x3D, 0);

		fwPowerState = POWER_STATE_D3;
	}
}

#ifdef CONFIG_CABLE_DETECT_ACCESSORY
#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
void ProcessMhlStatus(bool connect, bool force)
{
	if (force) {

		gConnectMHL = connect;

		/* if connected, let DetectCharger to report the status */
		if (connect)
			return;
		else
			gStatusMHL = CONNECT_TYPE_UNKNOWN;

	} else {
		/* the connection has been closed, no need to report the charger status */
		if (!gConnectMHL && gStatusMHL) {
			TPI_DEBUG_PRINT(("DetectCharger: no need to report the charger status?\n"));
			return;
		}
	}

	update_mhl_status(gConnectMHL, gStatusMHL);
}
#else
void ProcessMhlStatus(bool connect, bool force)
{
	update_mhl_status(connect, CONNECT_TYPE_UNKNOWN);
}
#endif
#endif

static	int	Int4Isr(void)
{
	uint8_t		reg74, reg72;

	reg74 = I2C_ReadByte(TPI_SLAVE_ADDR, (0x74));
	reg72 = I2C_ReadByte(TPI_SLAVE_ADDR, (0x72)); /* mw20110729 for debug when has SCDT,PSTABLE change int only */
	if (0xFF == reg74)
		return I2C_INACCESSIBLE;

	if (reg74 & BIT_2) {
		/* WR_Dcap_Rdy_Int_Done = false; */
		MhlTxDrvProcessConnection();
		/*UNMASK_INTR_2_INTERRUPTS; */
		/*UNMASK_INTR_4_INTERRUPTS; */
#ifdef CONFIG_CABLE_DETECT_ACCESSORY
		/* ProcessMhlStatus(true, true); */
#endif
	} else if (reg74 & BIT_3) {
		I2C_WriteByte(TPI_SLAVE_ADDR, (0x74), reg74);
		MhlTxDrvProcessDisconnection();
#ifdef CONFIG_CABLE_DETECT_ACCESSORY
		/* ProcessMhlStatus(false, true); */
#endif
		return I2C_INACCESSIBLE; /* mw20110925 may related to MHL_Est 5K pull up failure issue */
	}

	if ((POWER_STATE_D3 == fwPowerState) && (reg74 & BIT_6)) {
		SwitchToD0();
		ProcessRgnd();
		/* UNMASK_INTR_1_INTERRUPTS;*/
	}

	if (reg74 & BIT_4) {
		TPI_DEBUG_PRINT(("Drv: CBus Lockout\n"));

		ForceUsbIdSwitchOpen();
		ReleaseUsbIdSwitchOpen();
#ifdef CONFIG_CABLE_DETECT_ACCESSORY
		ProcessMhlStatus(false, true);
#endif

	}
	I2C_WriteByte(TPI_SLAVE_ADDR, (0x74), reg74);
	return I2C_ACCESSIBLE;
}
#ifdef	APPLY_PLL_RECOVERY
static void ApplyPllRecovery(void)
{

	CLR_BIT(TPI_SLAVE_ADDR, 0x80, 4);

	SET_BIT(TPI_SLAVE_ADDR, 0x80, 4);

	DelayMS(10);

	SET_BIT(TPI_SLAVE_ADDR, 0x05, 4);

	CLR_BIT(TPI_SLAVE_ADDR, 0x05, 4);


	TPI_DEBUG_PRINT(("Drv: Applied PLL Recovery\n"));
}

void SiiMhlTxDrvRecovery(void)
{
	if ((I2C_ReadByte(TPI_SLAVE_ADDR, (0x74)) & BIT_0)) {
		SET_BIT(TPI_SLAVE_ADDR, (0x74), 0);
		TPI_DEBUG_PRINT(("Drv: SCDT Interrupt\n"));

		if ((((I2C_ReadByte(TPI_SLAVE_ADDR, 0x81)) & BIT_1) >> 1))
			ApplyPllRecovery();
	}

	if ((I2C_ReadByte(TPI_SLAVE_ADDR, (0x72)) & BIT_1)) {

		TPI_DEBUG_PRINT(("Drv: PSTABLE Interrupt\n"));

		ApplyPllRecovery();

		SET_BIT(TPI_SLAVE_ADDR, (0x72), 1);

	}
}
#endif
static void MhlTxDrvProcessConnection(void)
{
	bool	mhlConnected = true;

	TPI_DEBUG_PRINT(("Drv: MHL Cable Connected. CBUS:0x0A = %02X\n", (int) ReadByteCBUS(0x0a)));

	if (POWER_STATE_D0_MHL == fwPowerState)
		return;

	I2C_WriteByte(TPI_SLAVE_ADDR, 0xA0, 0x10);

	fwPowerState = POWER_STATE_D0_MHL;

	WriteByteCBUS(0x07, 0xF2); /* mw20110922 match w/ V100108 */

	SET_BIT(CBUS_SLAVE_ADDR, 0x44, 1);

	/* CLR_BIT(TPI_SLAVE_ADDR, 0x79, 4);*/

	/* SiiMhlTxDrvTmdsControl( true ); mw20110922 , v100108 doesn't enable TMDS here ; Page0Reg0x80[4]=1 enable TMDS Tx */

	SET_BIT(TPI_SLAVE_ADDR, 0x90, 0);
	ENABLE_DISCOVERY;

	TPI_DEBUG_PRINT(("Drv: Wait T_SRC_RXSENSE_CHK (%d ms) before checking RSEN\n",
							(int) T_SRC_RXSENSE_CHK));

	rsenCheckTimeout = jiffies + HZ/3;
	rsenCount = 0;

	TPI_DEBUG_PRINT(("Update Rx Dcap_Rdy Int \n"));

	DelayMS(T_SRC_RXSENSE_CHK-100);
	contentOn = 1;
	SiiMhlTxNotifyConnection(mhlConnected = true);
	//SiiMhlTxDrvTmdsControl(true);

	if (!(I2C_ReadByte(TPI_SLAVE_ADDR, 0x09) & BIT_2)) {
		TPI_DEBUG_PRINT(("400ms exp, Rsen=0,discnct\n"));
		DISABLE_DISCOVERY;
		ENABLE_DISCOVERY;
		MhlTxDrvProcessDisconnection();
		DelayMS(100);
		return ;
	}

#ifdef CONFIG_CABLE_DETECT_ACCESSORY
	ProcessMhlStatus(true, true);
#endif
	IsEstablished = true;
}

static void MhlTxDrvProcessDisconnection(void)
{
	bool	mhlConnected = false;

	TPI_DEBUG_PRINT(("Drv: MhlTxDrvProcessDisconnection\n"));

#ifdef CONFIG_CABLE_DETECT_ACCESSORY
#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
	/*cable detect recognise wrong device as MHL, do cable out in the following*/
	if(fwPowerState == POWER_STATE_D0_NO_MHL && (gConnectMHL == false))
		fwPowerState = POWER_STATE_D0_MHL;
#endif
#endif


	I2C_WriteByte(TPI_SLAVE_ADDR, 0xA0, 0xD0);

	SiiMhlTxDrvTmdsControl(false);

	if (POWER_STATE_D0_MHL == fwPowerState) {
		contentOn = 0;
		SiiMhlTxNotifyConnection(mhlConnected = false);
#ifdef CONFIG_CABLE_DETECT_ACCESSORY
		ProcessMhlStatus(false, true);
#endif
	}
	SwitchToD3();

	/* Make sure CBUS/ID pin switch to USB after MHL disconnect. */
	sii9234_reset();
	IsEstablished = false;
}

bool IsMHLConnection(void)
{
	if ((fwPowerState == POWER_STATE_D0_MHL) && IsEstablished)
		return true;
	else
		return false;
}

void	CbusReset()
{
	int idx;
	SET_BIT(TPI_SLAVE_ADDR, 0x05, 3);
	DelayMS(2);
	CLR_BIT(TPI_SLAVE_ADDR, 0x05, 3);

	mscCmdInProgress = false;

	UNMASK_CBUS1_INTERRUPTS;
	UNMASK_CBUS2_INTERRUPTS;
	/* mw20110922 , delete to match w v100108 ; DDC translation time out=3; Cbus burst length=128 bytes, enable Cbus DDC burst mode */
	for (idx = 0; idx < 4; idx++) {
		WriteByteCBUS(0xE0 + idx, 0xFF);
		WriteByteCBUS(0xF0 + idx, 0xFF);
	}
}

static uint8_t CBusProcessErrors(uint8_t intStatus)
{
	uint8_t result          = 0;
	uint8_t mscAbortReason  = 0;
	uint8_t ddcAbortReason  = 0;


	intStatus &=  (BIT_6 | BIT_5);

	if (intStatus) {

		if (intStatus & BIT_2) {
			result = ddcAbortReason = ReadByteCBUS(0x0C);
			TPI_DEBUG_PRINT(("CBUS DDC ABORT happened, reason:: %02X\n", (int)(ddcAbortReason)));
			ForceUsbIdSwitchOpen();
			ReleaseUsbIdSwitchOpen();
		}

		if (intStatus & BIT_5) {
			result = mscAbortReason = ReadByteCBUS(0x0D);

			TPI_DEBUG_PRINT(("CBUS:: MSC Transfer ABORTED. Clearing 0x0D\n"));
			WriteByteCBUS(0x0D, 0xFF);
			ForceUsbIdSwitchOpen();
			ReleaseUsbIdSwitchOpen();
		}
		if (intStatus & BIT_6) {
			TPI_DEBUG_PRINT(("CBUS:: MSC Peer sent an ABORT. Clearing 0x0E\n"));
			WriteByteCBUS(0x0E, 0xFF);
		}


		if (mscAbortReason != 0) {
			TPI_DEBUG_PRINT(("CBUS:: Reason for ABORT is ....0x%02X = ", (int)mscAbortReason));
			if (mscAbortReason & (0x01 << 0))
				TPI_DEBUG_PRINT(("Requestor MAXFAIL - retry threshold exceeded\n"));
			if (mscAbortReason & (0x01 << 1))
				TPI_DEBUG_PRINT(("Protocol Error\n"));
			if (mscAbortReason & (0x01 << 2))
				TPI_DEBUG_PRINT(("Requestor translation layer timeout\n"));
			if (mscAbortReason & (0x01 << 7))
				TPI_DEBUG_PRINT(("Peer sent an abort\n"));
			if (mscAbortReason & (0x01 << 3))
				TPI_DEBUG_PRINT(("Undefined opcode\n"));
		}
	}
	return(result);
}


static void MhlCbusIsr(void)
{
	uint8_t		cbusInt;
	uint8_t     gotData[4];
	uint8_t		i;
	uint8_t		reg71 = I2C_ReadByte(TPI_SLAVE_ADDR, 0x71);

	cbusInt = ReadByteCBUS(0x08);

	if (cbusInt == 0xFF)
		return;

	if (cbusInt) {
		/* Clear all interrupts that were raised even if we did not process */
		WriteByteCBUS(0x08, cbusInt);
		TPI_DEBUG_PRINT(("Drv: CBUS INTR_1: %02X\n", (int) cbusInt));
	}

	if (cbusInt & BIT_2)
		ApplyDdcAbortSafety();

	if ((cbusInt & BIT_3)) {
		TPI_DEBUG_PRINT(("Drv: MSC_MSG Received\n"));
		SiiMhlTxGotMhlMscMsg(ReadByteCBUS(0x18), ReadByteCBUS(0x19));
	}

	if ((cbusInt & BIT_5) || (cbusInt & BIT_6)) {
		if (!WR_Dcap_Rdy_Int_Done && (cbusInt&BIT_5)) {
			TPI_DEBUG_PRINT(("Drv:0x0A:%02X, 0x0D:%02X, 0x09:%02X\n",
					(int)ReadByteCBUS(0x0A),
					(int)ReadByteCBUS(0x0D),
					(int)ReadByteCBUS(0x09)));
			if (cbusErrCount++ >= 10) {
				cbusErrCount = 0;
				sii9234_request_abort();
			}
			return; /* don't clear pending int, until 400ms sw delay expired */
		}
		gotData[0] = CBusProcessErrors(cbusInt);
	}

	/* BIT_5 not set */
	cbusErrCount = 0;

	if (cbusInt & BIT_4) {
		TPI_DEBUG_PRINT(("Drv: MSC_REQ_DONE\n"));
		mscCmdInProgress = false;
		SiiMhlTxMscCommandDone(ReadByteCBUS(0x16));
	}

	if (BIT_7 & cbusInt) {
		TPI_DEBUG_PRINT(("Drv:%d Clearing CBUS_link_hard_err_count\n", (int)__LINE__));
		WriteByteCBUS(0x38, (uint8_t)(ReadByteCBUS(0x38) & 0xF0));
	}


	cbusInt = ReadByteCBUS(0x1E);
	if (cbusInt)
		TPI_DEBUG_PRINT(("Drv: CBUS INTR_2: %d\n", (int) cbusInt));

	if (cbusInt & BIT_2) {
		TPI_DEBUG_PRINT(("Drv: MHL INTR Received\n"));
		SiiMhlTxGotMhlIntr(ReadByteCBUS(0xA0), ReadByteCBUS(0xA1));

		for (i = 0; i < 4; i++)
			WriteByteCBUS((0xA0 + i), ReadByteCBUS(0xA0 + i));
	}
	if (cbusInt & BIT_3) {
		TPI_DEBUG_PRINT(("Drv: MHL STATUS Received\n"));
		SiiMhlTxGotMhlStatus(ReadByteCBUS(0xB0), ReadByteCBUS(0xB1));

		for (i = 0; i < 4; i++)
			WriteByteCBUS((0xB0 + i), ReadByteCBUS(0xB0 + i));
	}
	if (cbusInt) {
		WriteByteCBUS(0x1E, cbusInt);
		TPI_DEBUG_PRINT(("Drv: Clear CBUS INTR_2: %02X\n", (int) cbusInt));
	}
	if (reg71) {
		TPI_DEBUG_PRINT(("Drv: INTR_1 @72:71 = %02X\n", (int) reg71));
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x71, BIT_6);
	}

	cbusInt = ReadByteCBUS(0x0D);

	if (BIT_6 & (dsHpdStatus ^ cbusInt)) {
		uint8_t status = cbusInt & BIT_6;
		/* SiiMhlTxNotifyDsHpdChange( cbusInt ); */
		TPI_DEBUG_PRINT(("Drv: Downstream HPD changed to: %02X\n", (int) cbusInt));
		SiiMhlTxNotifyDsHpdChange(status);
		if (status)
			SiiMhlTxDrvReleaseUpstreamHPDControl();

		dsHpdStatus = cbusInt;
	}
}

void D2ToD3(void)
{
	TPI_DEBUG_PRINT(("D2 To D3 mode\n"));

	TPI_DEBUG_PRINT(("Drv: Switch To D3: pinAllowD3 = %d\n", 1));

	ReadModifyWriteTPI(0x93, BIT_7 | BIT_6 | BIT_5 | BIT_4, 0);

	ReadModifyWriteTPI(0x94, BIT_1 | BIT_0, 0);

	ReadModifyWriteTPI(0x79, BIT_5 | BIT_4, BIT_4);

	I2C_WriteByte(HDMI_SLAVE_ADDR, 0x01, 0x03);

	I2C_WriteByte(0x7A, 0x3D, 0x3E);

	fwPowerState = POWER_STATE_D3;
}
bool tpi_get_hpd_state(void)
{
	uint8_t cbusInt, status;
	cbusInt = ReadByteCBUS(0x0D);
	status = cbusInt & BIT_6;
	TPI_DEBUG_PRINT(("Drv: %s hpd status %d\n", __func__, status));
	return (status) ? true : false;
}
