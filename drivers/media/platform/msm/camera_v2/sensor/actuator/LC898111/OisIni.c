//********************************************************************************
//
//		<< LC898111 Evaluation Soft >>
//		Program Name	: OisIni.c
//		Design			: Y.Yamada
//		History			: LC898111 changes						2011.04.08 d.yamagata
//********************************************************************************
//**************************
//	Include Header File		
//**************************
#define		OISINI

//#include	"Main.h"
//#include	"Cmd.h"
#include	"Ois.h"
#include	"OisFil.h"
#include	"OisDef.h"

/* HTC_START 20130329 */
#include	"HtcActOisBinder.h"
/* HTC_END */


//**************************
//	Local Function Prottype	
//**************************
void	IniClk( void ) ;		// Clock Setting
void	IniIop( void ) ;		// I/O Port Initial Setting
void	IniMon( void ) ;		// Monitor & Other Initial Setting
void	IniSrv( void ) ;		// Servo Register Initial Setting
void	IniGyr( void ) ;		// Gyro Filter Register Initial Setting
void	IniHfl( void ) ;		// Hall Filter Initial Parameter Setting
void	IniGfl( void ) ;		// Gyro Filter Initial Parameter Setting
void	IniAdj( void ) ;		// Adjust Fix Value Setting
void	IniCmd( void ) ;		// Command Execute Process Initial
void	IniDgy( void ) ;		// Digital Gyro Initial Setting



//********************************************************************************
// Function Name 	: IniSet
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Initial Setting Function
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniSet( void )
{
	// Clock Setting
	IniClk() ;
	// I/O Port Initial Setting
	IniIop() ;
	// DigitalGyro Initial Setting
	IniDgy() ;
	// Monitor & Other Initial Setting
	IniMon() ;
	// Servo Initial Setting
	IniSrv() ;
	// Gyro Filter Initial Setting
	IniGyr() ;
	// Hall Filter Initial Setting
	IniHfl() ;
	// Gyro Filter Initial Setting
	IniGfl() ;
	// Adjust Fix Value Setting
	IniAdj() ;
	// Command Execute Process Initial
	IniCmd() ;
}



//********************************************************************************
// Function Name 	: IniClk
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Clock Setting
// History			: First edition 						2009.07.30 Y.Tashita
//					  LC898111 changes						2011.04.08 d.yamagata
//********************************************************************************
void	IniClk( void )
{
	RegWriteA( OSCSTOP,  0x00 ) ;		// 0x0263	OSC active
	RegWriteA( OSCSET,   0x65 ) ;		// 0x0264	OSC ini
	UcOscAdjFlg	= 0 ;					// Osc adj flag 
	RegWriteA( OSCCNTEN, 0x00 ) ;		// 0x0265	OSC Cnt disable
	
	/*Clock Enables*/
	RegWriteA( CLKTST,	0x00 ) ;		// 0x020A	 [ - | - | CmCalClkTst | CMGifClkTst | CmPezClkTst | CmEepClkTst | CmSrvClkTst | CmPwmClkTst ]

#ifdef I2CE2PROM
	RegWriteA( CLKON,	0x33 ) ;		// 0x020B	 [ - | - | CmCalClkOn  | CMGifClkOn  | CmPezClkOn  | CmEepClkOn  | CmSrvClkOn  | CmPwmClkOn  ]
#else
 #ifdef	SPIE2PROM
	RegWriteA( CLKON,	0x17 ) ;		// 0x020B	 [ - | - | CmCalClkOn  | CMGifClkOn  | CmPezClkOn  | CmEepClkOn  | CmSrvClkOn  | CmPwmClkOn  ]
 #else
	RegWriteA( CLKON,	0x13 ) ;		// 0x020B	 [ - | - | CmCalClkOn  | CMGifClkOn  | CmPezClkOn  | CmEepClkOn  | CmSrvClkOn  | CmPwmClkOn  ]
 #endif
#endif
	/*Clock Settings*/
	RegWriteA( EEPDIV,	0x02 ) ;		// 0x0210	 EEPROM Clock Default Setting

	RegWriteA( SRVDIV,  0x02 ) ;		// 0x0211    Servo Clock Default Setting( 48MHz / 2 = 24MHz )
										//				Fs = XTAL/(SRVDIV*1024)
	RegWriteA( PWMDIV,	0x00 ) ;		// 0x0212	 PWM Clock Default Setting( 48MHz / 1 = 48MHz )
	
	RegWriteA( TSTDIV,	0x04 ) ;		// 0x0213	 Test Clock Default Setting
	RegWriteA( GIFDIV,	0x03 ) ;		// 0x0214	 Digital Gyro I/F Clock Default Setting
	RegWriteA( CALDIV,	0x06 ) ;		// 0x0215
}



//********************************************************************************
// Function Name 	: IniIop
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: I/O Port Initial Setting
// History			: First edition 						2009.07.30 Y.Tashita
//					  LC898111 changes						2011.04.08 d.yamagata
//********************************************************************************
void	IniIop( void )
{
	/*set IOP direction*/
	RegWriteA( P0LEV0, 0x00 ) ;		// 0x0220	[ WLEV7 | WLEV6 | WLEV5 | WLEV4 ][ WLEV3 | WLEV2 | WLEV1 | WLEV0 ]
	RegWriteA( P0LEV1, 0x00 ) ;		// 0x0221	[ - 	| - 	| - 	| - 	][ -	 | -	 | -	 | WLEV8 ]
	RegWriteA( P0DIR0, 0x77 ) ;		// 0x0222	[ DIR7	| DIR6	| DIR5	| DIR4	][ DIR3  | DIR2  | DIR1  | DIR0  ]
	RegWriteA( P0DIR1, 0x01 ) ;		// 0x0223	[ - 	| - 	| - 	| - 	][ -	 | -	 | -	 | DIR8  ]

	/*set pull up/down*/
	RegWriteA( P0PON0, 0x8D ) ;		// 0x0224	[ PON7 | PON6 | PON5 | PON4 ][ PON3  | PON2  | PON1 | PON0 ]
	RegWriteA( P0PON1, 0x0D ) ;		// 0x0225	[ -    | -	  | -	 | -	][ PON11 | PON10 | PON9 | PON8 ]
	RegWriteA( P0PUD0, 0x85 ) ;		// 0x0226	[ PUD7 | PUD6 | PUD5 | PUD4 ][ PUD3  | PUD2  | PUD1 | PUD0 ]
	RegWriteA( P0PUD1, 0x09 ) ;		// 0x0227	[ -    | -	  | -	 | -	][ PUD11 | PUD10 | PUD8 | PUD8 ]

	/*select IOP signal*/
	RegWriteA( IOP0SEL, 0x00 ); 	// 0x0230	[1:0] 00: DGMOSI, 01: HPS_CTL0, 1x: IOP0
	RegWriteA( IOP1SEL, 0x00 ); 	// 0x0231	[1:0] 00: DGSCLK/DGI2CK, 01: HPS_CTL1, 1x: IOP1
	RegWriteA( IOP2SEL, 0x00 ); 	// 0x0232	[5:4] 00: MONA, 01: MONB, 10: MONC, 11: MOND
									//			[1:0] 00: DGSSB, 01: MON, 1x: IOP2
	RegWriteA( IOP3SEL, 0x00 ); 	// 0x0233	[5:4] 00: MONA, 01: MONB, 10: MONC, 11: MOND
									//			[1:0] 00: DGINT, 01: MON, 1x: IOP3
#ifdef I2CE2PROM
	RegWriteA( IOP4SEL, 0x21 ); 	// 0x0234	[5:4] 00: MONA, 01: MONB, 10: MONC, 11: MOND
									//			[1:0] 00: BUSY1/EPSIIF, 01: MON, 1x: IOP4  (00ÌêATSTCLK[qÉÄIð)
#else
 #ifdef SPIE2PROM
	RegWriteA( IOP4SEL, 0x11 ); 	// 0x0234	[5:4] 00: MONA, 01: MONB, 10: MONC, 11: MOND
									//			[1:0] 00: BUSY1/EPSIIF, 01: MON, 1x: IOP4  (00ÌêATSTCLK[qÉÄIð)
 #else
	RegWriteA( IOP4SEL, 0x00 ); 	// 0x0234	[5:4] 00: MONA, 01: MONB, 10: MONC, 11: MOND
									//			[1:0] 00: BUSY1/EPSIIF, 01: MON, 1x: IOP4  (00ÌêATSTCLK[qÉÄIð)
 #endif
#endif
	RegWriteA( IOP5SEL, 0x01 ); 	// 0x0235	[5:4] 00: MONA, 01: MONB, 10: MONC, 11: MOND
									//			[1:0] 00: BUSY2, 01: MON, 1x: IOP5
#ifdef SPIE2PROM
	RegWriteA( IOP6SEL, 0x00 ); 	// 0x0236	[5:4] 00: MONA, 01: MONB, 10: MONC, 11: MOND
									//			[1:0] 00: EPSIIF, 01: MON, 1x: IOP6
#else
	RegWriteA( IOP6SEL, 0x11 ); 	// 0x0236	[5:4] 00: MONA, 01: MONB, 10: MONC, 11: MOND
									//			[1:0] 00: EPSIIF, 01: MON, 1x: IOP6
#endif
	RegWriteA( IOP7SEL, 0x00 ); 	// 0x0237	[5:4] 00: MONA, 01: MONB, 10: MONC, 11: MOND
									//			[1:0] 00: EPSOIF, 01: MON, 1x: IOP7
	RegWriteA( IOP8SEL, 0x00 ); 	// 0x0238	[5:4] 00: MONA, 01: MONB, 10: MONC, 11: MOND
									//			[1:0] 00: EPSKIF/MISO, 01: MON, 1x: IOP8  (00ÌêATSTCLK[qÉÄIð)

	/*select busy signal*/
	RegWriteA( BSYSEL, 0x00 );		// 0x0240	[3:0] 0h: EEPROMANZXÌBUSYM
									//				  1h: ServoñHÌÝM
									//				  2h: ªèñHÌ®ìð¦·BUSYM
									//				  3h: ServoñHÌsingoÍÌBUSYM
									//				  4h: GyroZÌBUSYM
									//				  5h: Digital GyroANZXÌBUSYM
									//				  6h: Calibrationf[^pEEPROMANZXÌBUSYM
									//				  7h: EEPROM§äBUSYM
									//				  8h~9h: ÝèÖ~

	/*set spi mode*/
	RegWriteA( SPIMD3, 0x00 );		// 0x0248	[1:0] x1: SPI-mode3Î®ì, 00: SPI-mode0Î®ì, 11: SPI-mode0/3¼Î®ì
	RegWriteA( I2CSEL, 0x00 );		// 0x0250	[0]    0: I2C Noise reduction ON, 1: OFF
	RegWriteA( SRMODE, 0x02 );		// 0x0251	[1]    0: SRAM DL ON, 1: OFF
									//			[0]    0: USE SRAM OFF, 1: ON
#ifdef I2CE2PROM
	RegWriteA( EEPMODE, 0x01 );		// 0x0252	[0] Ex I2C eeprom Mode
#else
	RegWriteA( EEPMODE, 0x00 );		// 0x0252	[0] Ex SPI eeprom Mode
#endif
}



//********************************************************************************
// Function Name 	: IniDgy
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Digital Gyro Initial Setting
// History			: First edition 						2009.11.10 Y.Hayashi
//					  LC898111 changes						2011.04.08 d.yamagata
//********************************************************************************
void	IniDgy( void )
{
	unsigned char	UcGrini ;
	
	/*************/
	/*For ST gyro*/
	/*************/
	
	/*Set SPI Type*/
	RegWriteA( SPIM 	, 0x01 );							// 0x038F 	[ - | - | - | - ][ - | - | - | DGSPI4 ]
															//				DGSPI4	0: 3-wire SPI, 1: 4-wire SPI

	/*Set to Command Mode*/
	RegWriteA( GRSEL	, 0x01 );							// 0x0380	[ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]

	/*Digital Gyro Read settings*/
	RegWriteA( GRINI	, 0x80 );							// 0x0381	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ LSBF | SLOWMODE | I2CMODE | - ]
	RegWriteA( GRINT	, 0x00 );							// 0x03B0	[ - | - | - | - ][ - | - | INTB | INTEN ]


	RegReadA( GRINI	, &UcGrini );							// 0x0381	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ LSBF | SLOWMODE | I2CMODE | - ]
	RegWriteA( GRINI	, ( UcGrini | SLOWMODE) );			// 0x0381	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ LSBF | SLOWMODE | I2CMODE | - ]
	
	RegWriteA( GRADR0	, 0x6A ) ;					// 0x0383	Set USER CONTROL
	RegWriteA( GSETDT	, 0x10 ) ;					// 0x038A	Set Write Data
	RegWriteA( GRACC	, 0x10 );					// 0x0382	[ ADRPLUS(1:0) | - | WR1B ][ - | RD4B | RD2B | RD1B ]
	AccWit( 0x10 ) ;								/* Digital Gyro busy wait 				*/

	RegWriteA( GRADR0,	0x1B ) ;					// 0x0383	Set GYRO_CONFIG
	RegWriteA( GSETDT,	( FS_SEL << 3) ) ;			// 0x038A	Set Write Data
	RegWriteA( GRACC,	0x10 ) ;					/* 0x0382	Set Trigger ON				*/
	AccWit( 0x10 ) ;								/* Digital Gyro busy wait 				*/

	RegReadA( GRINI	, &UcGrini );					// 0x0381	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ LSBF | SLOWMODE | I2CMODE | - ]
	RegWriteA( GRINI, ( UcGrini & ~SLOWMODE) );		// 0x0381	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ LSBF | SLOWMODE | I2CMODE | - ]
	
	UcStbySt = STBYST_OFF ;		/* TEST */
	
	GyOutSignal() ;

}


//********************************************************************************
// Function Name 	: IniMon
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Monitor & Other Initial Setting
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniMon( void )
{
	RegWriteA( PWMMONFC, 0x80 ) ;				// 0x00F4	
	
	RegWriteA( MONSELA, 0x5C ) ;				// 0x0270	
	RegWriteA( MONSELB, 0x5D ) ;				// 0x0271	
#ifdef I2CE2PROM
	RegWriteA( MONSELC, 0x2D ) ;				// 0x0272	E2P Clk for I2C
#else
	RegWriteA( MONSELC, 0x62 ) ;				// 0x0272	
#endif
	RegWriteA( MONSELD, 0x63 ) ;				// 0x0273	
}



//********************************************************************************
// Function Name 	: IniSrv
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Servo Initial Setting
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniSrv( void )
{
	UcPwmMod = INIT_PWMMODE ;					// Driver output mode
	
	RegWriteA( VGA_SET, 0x30 ) ;				// 0x0267		X,Y connect
	RegWriteA( LSVFC1 , 0x00 ) ;				// 0x0082		
	if( UcPwmMod == PWMMOD_CVL ) {
		RegWriteA( LXEQFC2 , 0x01 ) ;				// 0x0083		Linearâ³OFF
		RegWriteA( LYEQFC2 , 0x01 ) ;				// 0x008D		
	}else{
		RegWriteA( LXEQFC2 , 0x00 ) ;				// 0x0083		Linearâ³OFF
		RegWriteA( LYEQFC2 , 0x00 ) ;				// 0x008D		
	}
	
	/* X axis */
	RegWriteA( LXEQEN , 0x45 );					// 0x0084		LXSW OFF
	RegWriteA( LXEQFC , 0x00 );					// 0x0085		LXDSWB/LXPSWB/LXISWB ON
	
	RamWriteA( ADHXOFF,   0x0000 ) ;			// 0x1102
	RamWriteA( ADSAD4OFF, 0x0000 ) ;			// 0x110E
	RamWriteA( HXINOD,    0x0000 ) ;			// 0x1127
	RamWriteA( HXDCIN,    0x0000 ) ;			// 0x1126
	RamWriteA( HXSEPT1,    0x0000 ) ;			// 0x1123
	RamWriteA( HXSEPT2,    0x0000 ) ;			// 0x1124
	RamWriteA( HXSEPT3,    0x0000 ) ;			// 0x1125
	RamWriteA( LXDOBZ,     0x0000 ) ;			// 0x114A
	RamWriteA( LXFZF,      0x0000 ) ;			// 0x114B
	RamWriteA( LXFZB,      0x0000 ) ;			// 0x1156
	RamWriteA( LXDX,      0x0000 ) ;			// 0x1148
	RamWriteA( LXLMT,     0x7FFF ) ;			// 0x1157
	RamWriteA( LXLMT2,    0x7FFF ) ;			// 0x1158
	RamWriteA( LXLMTSD,   0x0000 ) ;			// 0x1159
	RamWriteA( PLXOFF,    0x0000 ) ;			// 0x115B
	RamWriteA( LXDODAT,   0x0000 ) ;			// 0x115A

	/* Y axis */
	RegWriteA( LYEQEN , 0x45 );					// 0x008E		LYSW OFF
	RegWriteA( LYEQFC , 0x00 );					// 0x008F		LYDSWB/LYPSWB/LYISWB ON
	
	RamWriteA( ADHYOFF,   0x0000 ) ;			// 0x1105
	RamWriteA( HYINOD,    0x0000 ) ;			// 0x1167
	RamWriteA( HYDCIN,    0x0000 ) ;			// 0x1166
	RamWriteA( HYSEPT1,    0x0000 ) ;			// 0x1163
	RamWriteA( HYSEPT2,    0x0000 ) ;			// 0x1164
	RamWriteA( HYSEPT3,    0x0000 ) ;			// 0x1165
	RamWriteA( LYDOBZ,     0x0000 ) ;			// 0x118A
	RamWriteA( LYFZF,      0x0000 ) ;			// 0x118B
	RamWriteA( LYFZB,      0x0000 ) ;			// 0x1196
	RamWriteA( LYDX,      0x0000 ) ;			// 0x1188
	RamWriteA( LYLMT,     0x7FFF ) ;			// 0x1197
	RamWriteA( LYLMT2,    0x7FFF ) ;			// 0x1198
	RamWriteA( LYLMTSD,   0x0000 ) ;			// 0x1199
	RamWriteA( PLYOFF,    0x0000 ) ;			// 0x119B
	RamWriteA( LYDODAT,   0x0000 ) ;			// 0x119A

	/* General Equalizer */
	RegWriteA( GNEQEN, 0x00 ) ;				// 0x009E	General Equalizer OFF
	RegWriteA( GNEQFC, 0x00 ) ;				// 0x009F	General mode
	RegWriteA( GNINADD, 0x0F ) ;			// 0x00A2	Default Setting
	RegWriteA( GNOUTADD, 0x00 ) ;			// 0x00A3	General Equalizer Output Cut Off

	RamWriteA( GNLMT, 0x7FFF ) ;			// 0x11AC	Limmiter
	RamWriteA( GDX, 0x0000 ) ;				// 0x11AF
	RamWriteA( GDOFFSET, 0x0000 ) ;			// 0x11B0
	RamWriteA( GOFF, 0x0000 ) ;				// 0x11A0

	// General Data Pass
	RegWriteA( GDPXFC, 0x00 ) ;				// 0x00A4	General data pass

	/* Calculation flow   X : Y1->X1    Y : X2->Y2 */
	RegWriteA( LCXFC, (unsigned char)0x00 ) ;			// 0x0001	High-order function X function setting
	RegWriteA( LCYFC, (unsigned char)0x00 ) ;			// 0x0006	High-order function Y function setting

	RegWriteA( LCY1INADD, (unsigned char)LXDOIN ) ;		// 0x0007	High-order function Y1 input selection
	RegWriteA( LCY1OUTADD, (unsigned char)DLY00 ) ;		// 0x0008	High-order function Y1 output selection
	RegWriteA( LCX1INADD, (unsigned char)DLY00 ) ;		// 0x0002	High-order function X1 input selection
	RegWriteA( LCX1OUTADD, (unsigned char)LXADOIN ) ;	// 0x0003	High-order function X1 output selection

	RegWriteA( LCX2INADD, (unsigned char)LYDOIN ) ;		// 0x0004	High-order function X2 input selection
	RegWriteA( LCX2OUTADD, (unsigned char)DLY01 ) ;		// 0x0005	High-order function X2 output selection
	RegWriteA( LCY2INADD, (unsigned char)DLY01 ) ;		// 0x0009	High-order function Y2 input selection
	RegWriteA( LCY2OUTADD, (unsigned char)LYADOIN ) ;	// 0x000A	High-order function Y2 output selection

	/* (0.5468917X^3+0.3750114X)*(0.5468917X^3+0.3750114X) 6.5ohm*/
	RamWriteA( LCY1A0, 0x0000 ) ;			// 0x12F2	0
	RamWriteA( LCY1A1, 0x4600 ) ;			// 0x12F3	1
	RamWriteA( LCY1A2, 0x0000 ) ;			// 0x12F4	2
	RamWriteA( LCY1A3, 0x3000 ) ;			// 0x12F5	3
	RamWriteA( LCY1A4, 0x0000 ) ;			// 0x12F6	4
	RamWriteA( LCY1A5, 0x0000 ) ;			// 0x12F7	5
	RamWriteA( LCY1A6, 0x0000 ) ;			// 0x12F8	6

	RamWriteA( LCX1A0, 0x0000 ) ;			// 0x12D2	0
	RamWriteA( LCX1A1, 0x4600 ) ;			// 0x12D3	1
	RamWriteA( LCX1A2, 0x0000 ) ;			// 0x12D4	2
	RamWriteA( LCX1A3, 0x3000 ) ;			// 0x12D5	3
	RamWriteA( LCX1A4, 0x0000 ) ;			// 0x12D6	4
	RamWriteA( LCX1A5, 0x0000 ) ;			// 0x12D7	5
	RamWriteA( LCX1A6, 0x0000 ) ;			// 0x12D8	6

	RamWriteA( LCX2A0, 0x0000 ) ;			// 0x12D9	0
	RamWriteA( LCX2A1, 0x4600 ) ;			// 0x12DA	1
	RamWriteA( LCX2A2, 0x0000 ) ;			// 0x12DB	2
	RamWriteA( LCX2A3, 0x3000 ) ;			// 0x12DC	3
	RamWriteA( LCX2A4, 0x0000 ) ;			// 0x12DD	4
	RamWriteA( LCX2A5, 0x0000 ) ;			// 0x12DE	5
	RamWriteA( LCX2A6, 0x0000 ) ;			// 0x12DF	6
	
	RamWriteA( LCY2A0, 0x0000 ) ;			// 0x12F9	0
	RamWriteA( LCY2A1, 0x4600 ) ;			// 0x12FA	1
	RamWriteA( LCY2A2, 0x0000 ) ;			// 0x12FB	2
	RamWriteA( LCY2A3, 0x3000 ) ;			// 0x12FC	3
	RamWriteA( LCY2A4, 0x0000 ) ;			// 0x12FD	4
	RamWriteA( LCY2A5, 0x0000 ) ;			// 0x12FE	5
	RamWriteA( LCY2A6, 0x0000 ) ;			// 0x12FF	6
	
	RegWriteA( GDPX1INADD,  0x00 ) ;		// 0x00A5	Default Setting
	RegWriteA( GDPX1OUTADD, 0x00 ) ;		// 0x00A6	General Data Pass Output Cut Off
	RegWriteA( GDPX2INADD,  0x00 ) ;		// 0x00A7	Default Setting
	RegWriteA( GDPX2OUTADD, 0x00 ) ;		// 0x00A8	General Data Pass Output Cut Off
	RegWriteA( GDPX3INADD,  0x00 ) ;		// 0x00A9	Default Setting
	RegWriteA( GDPX3OUTADD, 0x00 ) ;		// 0x00AA	General Data Pass Output Cut Off

	RegWriteA( GDPYFC, 0x00 ) ;				// 0x00AB	General data pass
	RegWriteA( GDPY1INADD, 0x00 ) ;			// 0x00AC	Default Setting
	RegWriteA( GDPY1OUTADD, 0x00 ) ;		// 0x00AD	General Data Pass Output Cut Off
	RegWriteA( GDPY2INADD, 0x00 ) ;			// 0x00AE	Default Setting
	RegWriteA( GDPY2OUTADD, 0x00 ) ;		// 0x00AF	General Data Pass Output Cut Off
	RegWriteA( GDPY3INADD, 0x00 ) ;			// 0x00B0	Default Setting
	RegWriteA( GDPY3OUTADD, 0x00 ) ;		// 0x00B1	General Data Pass Output Cut Off
	
	// Feed Forward X Filter
	RegWriteA( FFXEN, 0x00 ) ;				// 0x00B2	Equalizer OFF
	RegWriteA( FFXFC, 0x00 ) ;				// 0x00B3	45Convert Circuit OFF
	RegWriteA( FFXDS, 0x00 ) ;				// 0x00B4	Down Sampling 1/1
	RegWriteA( FXINADD, 0x2C ) ;			// 0x00B7	LXGZF
	RegWriteA( FXOUTADD, 0x49 ) ;			// 0x00B8	LXGZB

	// Feed Forward Y Filter
	RegWriteA( FFYEN, 0x00 ) ;				// 0x00B9	Equalizer OFF
	RegWriteA( FFYFC, 0x00 ) ;				// 0x00BA	45Convert Circuit OFF
	RegWriteA( FFYDS, 0x00 ) ;				// 0x00BB	Down Sampling 1/1
	RegWriteA( FYINADD, 0x6C ) ;			// 0x00BE	LYGZF
	RegWriteA( FYOUTADD, 0x89 ) ;			// 0x00BF	LYGZB
	
	// Measure Filter
	RegWriteA( MSF1EN, 0x00 ) ;				// 0x00C0		Measure Equalizer1 OFF
	RegWriteA( MSF2EN, 0x00 ) ;				// 0x00C4		Measure Equalizer2 OFF
	RegWriteA( MSFDS,  0x00 ) ;				// 0x00C8		Down sampling 1/1
	RegWriteA( MS1INADD, 0x46 ) ;			// 0x00C2		LXC1
	RegWriteA( MS1OUTADD, 0x00 ) ;			// 0x00C3		Measure Filter1 Output Cut Off
	RegWriteA( MS2INADD, 0x47 ) ;			// 0x00C6		LXC2 Setting
	RegWriteA( MS2OUTADD, 0x00 ) ;			// 0x00C7		Measure Filter2 Output Cut Off

	// Gyro Filter Interface
	RegWriteA( GYINFC, 0x00 ) ;				// 0x00DA		LXGZB,LYGZB Input Cut Off, 0 Sampling Delay, Down Sampling 1/1

	// Sin Wave Generater
	RegWriteA( SWEN, 0x08 ) ;				// 0x00DB		Sin Wave Generate OFF, Sin Wave Setting
	RegWriteA( SWFC2, 0x08 ) ;				// 0x00DE		SWC = 0
	RegWriteA( SWSEL, 0x00 ) ;				// 0x00E2		No Operation
	RegWriteA( SINXADD, 0x00 ) ;			// 0x00E3	
	RegWriteA( SINYADD, 0x00 ) ;			// 0x00E4	

	// Delay RAM Monitor
	RegWriteA( DAMONFC, 0x00 ) ;			// 0x00F5		ExDAC OFF , Default Setting
	RegWriteA( MDLY1ADD, 0x10 ) ;			// 0x00E5		Delay Monitor1
	RegWriteA( MDLY2ADD, 0x11 ) ;			// 0x00E6		Delay Monitor2

	// Delay RAM Clear
	BsyWit( DLYCLR, 0xFF ) ;				// 0x00EE	Delay RAM All Clear
	BsyWit( DLYCLR2, 0xEC ) ;				// 0x00EF	Delay RAM All Clear
	RegWriteA( DLYCLR	, 0x00 );			// 0x00EE	CLR disable

	// Hall Amp...
	RegWriteA( RTXADD, 0x00 ) ;				// 0x00CE	Cal OFF
	RegWriteA( RTYADD, 0x00 ) ;				// 0x00E8	Cal OFF
	
	// PWM Signal Generate
	DrvSw( OFF ) ;							/* 0x0070	Drvier Block Ena=0 */
	RegWriteA( DRVFC2	, 0x40 );			// 0x0068	PriDriver:Slope, Driver:DeadTime 12.5ns
	RegWriteA( DRVSELX	, 0x00 );			// 0x0071	PWM X drv max current  DRVSELX[7:0]
	RegWriteA( DRVSELY	, 0x00 );			// 0x0072	PWM Y drv max current  DRVSELY[7:0]
 #ifdef LOWCURRENT
	RegWriteA( PWMFC,   0x4D ) ;			// 0x0075	VREF, PWMCLK/64 MODEB, 12Bit Accuracy
 #else
	RegWriteA( PWMFC,   0x11 ) ;			// 0x0075	VREF, PWMCLK/512, MODE1, 12Bit Accuracy
 #endif
	RegWriteA( PWMA,    0x00 ) ;			// 0x0074	PWM X/Y standby
	RegWriteA( PWMDLY1,  0x04 ) ;			// 0x0076	X Phase Delay Setting
	RegWriteA( PWMDLY2,  0x04 ) ;			// 0x0077	Y Phase Delay Setting

	RegWriteA( LNA		, 0xC0 );			// 0x0078	Low Noise mode enable
	RegWriteA( LNFC 	, 0x02 );			// 0x0079
	RegWriteA( LNSMTHX	, 0x80 );			// 0x007A
	RegWriteA( LNSMTHY	, 0x80 );			// 0x007B

	RegWriteA( GEPWMFC, 0x01 ) ;			// 0x007E	General PWM Output Stanby, 12Bit Accuracy
	RegWriteA( GEPWMDLY, 0x00 ) ;			// 0x007F	Default Setting

	// Measure Circuit
	RegWriteA( MSMA, 0x00 ) ;				// 0x00C9		Measure mode OFF

	// Flag Monitor
	RegWriteA( FLGM, 0xCC ) ;				// 0x00F8	BUSY2 Output ON
	RegWriteA( FLGIST, 0xCC ) ;				// 0x00F9	Interrupt Clear
	RegWriteA( FLGIM2, 0xF8 ) ;				// 0x00FA	BUSY2 Output ON
	RegWriteA( FLGIST2, 0xF8 ) ;			// 0x00FB	Interrupt Clear

	// Function Setting
	RegWriteA( FCSW, 0x00 ) ;				// 0x00F6	2Axis Input, PWM Mode, X,Y Axis Reverse OFF
	RegWriteA( FCSW2, 0x00 ) ;				// 0x00F7	X,Y Axis Invert OFF, PWM Synchronous, A/D Over Sampling ON
	
	/* Srv Smooth start */
	RamWriteA( HXSMSTP   , 0x0400 ) ;					/* 0x1120	*/
	RamWriteA( HYSMSTP   , 0x0400 ) ;					/* 0x1160	*/

	RegWriteA( SSSFC1, 0x43 ) ;				// 0x0098	0.68ms * 8times = 5.46ms
	RegWriteA( SSSFC2, 0x03 ) ;				// 0x0099	1.36ms * 3 = 4.08ms
	RegWriteA( SSSFC3, 0x50 ) ;				// 0x009A	1.36ms

	/* Srv Emargency */
	RegWriteA( SEOEN,  0x00 ) ;				// 0x009B	Emargency mode off
	RegWriteA( SEOFC1, 0x01 ) ;				// 0x009C	Emargency para 1
	RegWriteA( SEOFC2, 0x77 ) ;				// 0x009D	Emargency para 2 699ms
	RamWriteA( LXSEOLMT   , 0x7000 ) ;					/* 0x12D0	*/
	RamWriteA( LYSEOLMT   , 0x7000 ) ;					/* 0x12D1	*/

	RegWriteA( STBB, 0x00 ) ;				// 0x0260	All standby
	
}



//********************************************************************************
// Function Name 	: IniGyr
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Gyro Filter Setting Initialize Function
// History			: First edition 						2009.07.30 Y.Tashita
//					  LC898111 changes						2011.04.08 d.yamagata
//********************************************************************************
#ifdef GAIN_CONT
  #define	TRI_LEVEL		0x3A03126F		/* 0.0005 */
  #define	TRI_HIGH		0x3F800000		/* 1.0 */
  #define	TIMELOW			0x50			/* */
  #define	TIMEMID			0x05			/* */
  #define	TIMEHGH			0x05			/* */
  #define	TIMEBSE			0x5D			/* 3.96ms */
  #define	XMONADR			GXXFZ
  #define	YMONADR			GYXFZ
  #define	GANADR			gxadj
  #define	XMINGAIN		0x00000000
  #define	XMAXGAIN		0x3F800000
  #define	YMINGAIN		0x00000000
  #define	YMAXGAIN		0x3F800000
  #define	XSTEPUP			0x38D1B717		/* 0.0001	 */
  #define	XSTEPDN			0xBD4CCCCD		/* -0.05 	 */
  #define	YSTEPUP			0x38D1B717		/* 0.0001	 */
  #define	YSTEPDN			0xBD4CCCCD		/* -0.05 	 */
#endif

void	IniGyr( void )
{
	
	/*Initialize Gyro RAM*/
	ClrGyr( 0x00 , CLR_GYR_ALL_RAM );
	
	/*Gyro Filter Setting*/
	RegWriteA( GEQSW	, 0x11 );		// 0x0101		[ - | - | Sine_In | AD_In ][ - | - | - | T-Filter_Out ]
	RegWriteA( GSHAKEON , 0x01 );		// 0x0102	 	[ - | - | - | -  ][ - | - | - | CmShakeOn ]
	RegWriteA( GSHTON	, 0x00 );		// 0x0104		[ - | - | - | CmSht2PanOff ][ - | - | CmShtOpe(1:0) ]
										//				CmShtOpe[1:0] 00: Vb^[OFF, 01: Vb^[ON, 1x:O§ä
	RegWriteA( G2NDCEFON1,0x00 );		// 0x0107       [ -  | -  | -  | gxistp ][ gxlens | gxzoom | gxgain | gxgyro ]
	RegWriteA( G2NDCEFON0,0x00 );		// 0x0106		[ L4 | L3 | L2 | L1 	][ H2	  | H1	   | I2 	| I1	 ]
	RegWriteA( GADMEANON, 0x00 );		// 0x0113		[ - | - | - | - ][ - | - | - | CmAdMeanOn ]
	RegWriteA( GVREFADD , 0x14 );		// 0x0114	 	Z^[ßµðs¤xRAMÌAhXºÊ6rbg@(default 0x14 = GXH1Z2/GYH1Z2)
	RegWriteA( GSHTMOD , 0x0E );		// 0x0115	 	Shutter Hold mode
	RegWriteA( GLMT3MOD , 0x00 );		// 0x0116 	[ - | - | - | - ][ - | - | - | CmLmt3Mod ]
										//				CmLmt3Mod	0: Êí~b^[®ì, 1: ~Ì¼a~b^[®ì
	RegWriteA( GLMT3SEL , 0x00 );		// 0x0117 	[ - | - | - | - ][ - | - | - | CmLmt3Sel ]
										//				CmLmt3Sel	0: gxlmt3H0/gylmt3H0ðgp, 1: gxlmt3H1/gylmt3H1ðgp
	RegWriteA( GGADON	, 0x01 );		// 0x011C		[ - | - | - | CmSht2PanOff ][ - | - | CmGadOn(1:0) ]
										//				CmGadOn[1]	0: CmDwmSmpÌÝèÅTvO, 1: FsüúÅTvO
										//				CmGadOn[0]	0: Analog Gyrogp, 1: Digital Gyrogp
	RegWriteA( GGADSMP1 , 0x01 );		// 0x011E		Digital GyroÌADÏ·mèÔðÝè
	RegWriteA( GGADSMP0 , 0x00 );		// 0x011D
	RegWriteA( GGADSMPT , 0x0E);		// 0x011F		X²ÆY²ÌÝæ¾·éêA0x0EÉÝè
										//				1FsÅ4²æ¾·éêA0x2DÉÝè
										//				2FSÈãÅ4²æ¾·éêA0x1EÉÝè

	/*Gyro Filter Down Sampling*/
	RegWriteA( GDWNSMP1 , 0x00 );		// 0x0110 	For overall filter
	RegWriteA( GDWNSMP2 , 0x00 );		// 0x0111 	For H1 fitler
	RegWriteA( GDWNSMP3 , 0x00 );		// 0x0112 	For T filter
	
	/*Gyro Filter Floating Point Value Limits*/
	RegWriteA( GEXPLMTH , 0x80 );		// 0x019C
	RegWriteA( GEXPLMTL , 0x5A );		// 0x019D
	
	// Limiter
	RamWrite32A( gxlmt1L, 0x00000000 ) ;	// 0x18B0
	RamWrite32A( gxlmt1H, GYRO_LMT1H ) ;	// 0x18B1
	RamWrite32A( gylmt1L, 0x00000000 ) ;	// 0x19B0
	RamWrite32A( gylmt1H, GYRO_LMT1H ) ;	// 0x19B1
	RamWrite32A( gxlmt2L, 0x00000000 ) ;	// 0x18B2
	RamWrite32A( gxlmt2H, 0x3F800000 ) ;	// 0x18B3
	RamWrite32A( gylmt2L, 0x00000000 ) ;	// 0x19B2
	RamWrite32A( gylmt2H, 0x3F800000 ) ;	// 0x19B3
	RamWrite32A( gxlmt4SL, GYRO_LMT4L ) ;	// 0x1808
	RamWrite32A( gxlmt4SH, GYRO_LMT4H ) ;	// 0x1809
	RamWrite32A( gylmt4SL, GYRO_LMT4L ) ;	// 0x1908
	RamWrite32A( gylmt4SH, GYRO_LMT4H ) ;	// 0x1909

	// Limiter3
	RamWrite32A( gxlmt3H0, 0x3F000000 ) ;	// 0x18B4	0.8*0.5=0.4 ... --- > 0.5
	RamWrite32A( gylmt3H0, 0x3F000000 ) ;	// 0x19B4	0.8*0.5=0.4 ... --- > 0.5
	RamWrite32A( gxlmt3H1, 0x3F000000 ) ;	// 0x18B5	0.8*0.5=0.4 ... --- > 0.5
	RamWrite32A( gylmt3H1, 0x3F000000 ) ;	// 0x19B5	0.8*0.5=0.4 ... --- > 0.5

	// Monitor Circuit
	RegWriteA( GDLYMON10, 0xF5 ) ;			// 0x0184
	RegWriteA( GDLYMON11, 0x01 ) ;			// 0x0185
	RegWriteA( GDLYMON20, 0xF5 ) ;			// 0x0186
	RegWriteA( GDLYMON21, 0x00 ) ;			// 0x0187
	RamWrite32A( gdm1g, 0x3F800000 ) ;		// 0x18AC
	RamWrite32A( gdm2g, 0x3F800000 ) ;		// 0x19AC
	RegWriteA( GDLYMON30, 0xF5 ) ;			// 0x0188
	RegWriteA( GDLYMON31, 0x01 ) ;			// 0x0189
	RegWriteA( GDLYMON40, 0xF5 ) ;			// 0x018A
	RegWriteA( GDLYMON41, 0x00 ) ;			// 0x018B
	RamWrite32A( gdm3g, 0x3F800000 ) ;		// 0x18AD
	RamWrite32A( gdm4g, 0x3F800000 ) ;		// 0x19AD
	RegWriteA( GPINMON3, 0x3C ) ;			// 0x0182
	RegWriteA( GPINMON4, 0x38 ) ;			// 0x0183
	
	/*Data Pass Setting*/
#ifdef	LMT1MODE
	RegWriteA( GDPI1ADD1, 0x01 );		// 0x0171	Data Pass 1 Input
	RegWriteA( GDPI1ADD0, 0xC0 );		// 0x0170
	RegWriteA( GDPO1ADD1, 0x01 );		// 0x0173	Data Pass 1 Output
	RegWriteA( GDPO1ADD0, 0xCB );		// 0x0172
	RegWriteA( GDPI2ADD1, 0x00 );		// 0x0175	Data Pass 2 Input
	RegWriteA( GDPI2ADD0, 0xC0 );		// 0x0174
	RegWriteA( GDPO2ADD1, 0x00 );		// 0x0177	Data Pass 2 Output
	RegWriteA( GDPO2ADD0, 0xCB );		// 0x0176
#else
	RegWriteA( GDPI1ADD1, 0x01 );		// 0x0171	Data Pass 1 Input
	RegWriteA( GDPI1ADD0, 0xC0 );		// 0x0170
	RegWriteA( GDPO1ADD1, 0x01 );		// 0x0173	Data Pass 1 Output
	RegWriteA( GDPO1ADD0, 0xC0 );		// 0x0172
	RegWriteA( GDPI2ADD1, 0x00 );		// 0x0175	Data Pass 2 Input
	RegWriteA( GDPI2ADD0, 0xC0 );		// 0x0174
	RegWriteA( GDPO2ADD1, 0x00 );		// 0x0177	Data Pass 2 Output
	RegWriteA( GDPO2ADD0, 0xC0 );		// 0x0176
#endif
	
	/*Input Sine Wave or AD value*/
	RegWriteA( GSINTST	, 0x00 );		// 0x018F		[ - | - | - | CmSinTst_X ][ - | - | - | CmSinTst_Y ]
										//				CmSinTst_X/Y 0: ADlðgp, 1: Singðgp
	
	/* Pan/Tilt parameter */
	RegWriteA( GPANADDA, 		0x14 );		// 0x0130
	RegWriteA( GPANADDB, 		0x0E );		// 0x0131
	
	 //Threshold
	RamWrite32A( SttxHis, 	0x00000000 );			// 0x183F
	RamWrite32A( SttyHis, 	0x00000000 );			// 0x193F
	RamWrite32A( SttxaL, 	0x00000000 );			// 0x18AE
	RamWrite32A( SttxbL, 	0x00000000 );			// 0x18BE
	RamWrite32A( Sttx12aM, 	GYRA12_MID );	// 0x184F
	RamWrite32A( Sttx12aH, 	GYRA12_HGH );	// 0x185F
	RamWrite32A( Sttx12bM, 	GYRB12_MID );	// 0x186F
	RamWrite32A( Sttx12bH, 	GYRB12_HGH );	// 0x187F
	RamWrite32A( Sttx34aM, 	GYRA34_MID );	// 0x188F
	RamWrite32A( Sttx34aH, 	GYRA34_HGH );	// 0x189F
	RamWrite32A( Sttx34bM, 	GYRB34_MID );	// 0x18AF
	RamWrite32A( Sttx34bH, 	GYRB34_HGH );	// 0x18BF
	RamWrite32A( SttyaL, 	0x00000000 );			// 0x19AE
	RamWrite32A( SttybL, 	0x00000000 );			// 0x19BE
	RamWrite32A( Stty12aM, 	GYRA12_MID );	// 0x194F
	RamWrite32A( Stty12aH, 	GYRA12_HGH );	// 0x195F
	RamWrite32A( Stty12bM, 	GYRB12_MID );	// 0x196F
	RamWrite32A( Stty12bH, 	GYRB12_HGH );	// 0x197F
	RamWrite32A( Stty34aM, 	GYRA34_MID );	// 0x198F
	RamWrite32A( Stty34aH, 	GYRA34_HGH );	// 0x199F
	RamWrite32A( Stty34bM, 	GYRB34_MID );	// 0x19AF
	RamWrite32A( Stty34bH, 	GYRB34_HGH );	// 0x19BF
	
	// Pan level
	RegWriteA( GPANLEVABS, 		0x00 );		// 0x0164
	
	// Average
	RegWriteA( GPANSTT1DWNSMP0, 0x00 );		// 0x0134
	RegWriteA( GPANSTT1DWNSMP1, 0x00 );		// 0x0135
	RegWriteA( GPANSTT2DWNSMP0, 0x00 );		// 0x0136
	RegWriteA( GPANSTT2DWNSMP1, 0x00 );		// 0x0137
	RegWriteA( GPANSTT3DWNSMP0, 0x00 );		// 0x0138
	RegWriteA( GPANSTT3DWNSMP1, 0x00 );		// 0x0139
	RegWriteA( GPANSTT4DWNSMP0, 0x00 );		// 0x013A
	RegWriteA( GPANSTT4DWNSMP1, 0x00 );		// 0x013B
	RegWriteA( GMEANAUTO, 		0x01 );		// 0x015E Auto

	// Force State
	RegWriteA( GPANSTTFRCE, 	0x00 );		// 0x010A not use force state
	
	// Phase Transition Setting
	// State 2 -> 1
	RegWriteA( GPANSTT21JUG0, 	0x00 );		// 0x0140
	RegWriteA( GPANSTT21JUG1, 	0x00 );		// 0x0141
	// State 3 -> 1
	RegWriteA( GPANSTT31JUG0, 	0x00 );		// 0x0142
	RegWriteA( GPANSTT31JUG1, 	0x00 );		// 0x0143
	// State 4 -> 1
	RegWriteA( GPANSTT41JUG0, 	0x13 );		// 0x0144
	RegWriteA( GPANSTT41JUG1, 	0x00 );		// 0x0145
	// State 1 -> 2
	RegWriteA( GPANSTT12JUG0, 	0x00 );		// 0x0146
	RegWriteA( GPANSTT12JUG1, 	0x07 );		// 0x0147
	// State 1 -> 3
	RegWriteA( GPANSTT13JUG0, 	0x00 );		// 0x0148
	RegWriteA( GPANSTT13JUG1, 	0x00 );		// 0x0149
	// State 2 -> 3
	RegWriteA( GPANSTT23JUG0, 	0x11 );		// 0x014A
	RegWriteA( GPANSTT23JUG1, 	0x01 );		// 0x014B
	// State 4 -> 3
	RegWriteA( GPANSTT43JUG0, 	0x00 );		// 0x014C
	RegWriteA( GPANSTT43JUG1, 	0x00 );		// 0x014D
	// State 3 -> 4
	RegWriteA( GPANSTT34JUG0, 	0x11 );		// 0x014E
	RegWriteA( GPANSTT34JUG1, 	0x01 );		// 0x014F
	// State 2 -> 4
	RegWriteA( GPANSTT24JUG0, 	0x00 );		// 0x0150
	RegWriteA( GPANSTT24JUG1, 	0x00 );		// 0x0151
	// State 4 -> 2
	RegWriteA( GPANSTT42JUG0, 	0x00 );		// 0x0152
	RegWriteA( GPANSTT42JUG1, 	0x00 );		// 0x0153

	// State Timer
	RegWriteA( GPANSTT1LEVTMR, 	0x00 );		// 0x0160
	RegWriteA( GPANSTT2LEVTMR, 	0x00 );		// 0x0161
	RegWriteA( GPANSTT3LEVTMR, 	0x00 );		// 0x0162
	RegWriteA( GPANSTT4LEVTMR, 	0x03 );		// 0x0163
	
	// Control filter
#ifdef	LMT1MODE
	RegWriteA( GPANTRSON0, 		0x07 );		// 0x0132
	RegWriteA( GPANTRSON1, 		0x1E );		// 0x0133
#else
	RegWriteA( GPANTRSON0, 		0x01 );		// 0x0132
	RegWriteA( GPANTRSON1, 		0x1C );		// 0x0133
#endif
	
	// State Setting
	RegWriteA( GPANSTTSETGYRO, 	0x00 );		// 0x0154
#ifdef	LMT1MODE
	RegWriteA( GPANSTTSETGAIN, 	0x00 );		// 0x0155
	RegWriteA( GPANSTTSETISTP, 	0x00 );		// 0x0156
	RegWriteA( GPANSTTSETI1FTR,	0x58 );		// 0x0157
	RegWriteA( GPANSTTSETI2FTR,	0x00 );		// 0x0158
#else
	RegWriteA( GPANSTTSETGAIN, 	0x10 );		// 0x0155
	RegWriteA( GPANSTTSETISTP, 	0x00 );		// 0x0156
	RegWriteA( GPANSTTSETI1FTR,	0x90 );		// 0x0157
	RegWriteA( GPANSTTSETI2FTR,	0x90 );		// 0x0158
#endif
	RegWriteA( GPANSTTSETL2FTR, 0x00 );		// 0x0159
	RegWriteA( GPANSTTSETL3FTR,	0x00 );		// 0x015A
	RegWriteA( GPANSTTSETL4FTR,	0x00 );		// 0x015B
	
	// Hold
	RegWriteA( GPANSTTSETILHLD,	0x00 );		// 0x0168
	
	// HPS 
	RegWriteA( GPANSTTSETHPS,	0xF0 );		// 0x015C
	RegWriteA( GHPSMOD,			0x00 );		// 0x016F
	RegWriteA( GPANHPSTMR0,		0x5C );		// 0x016A
	RegWriteA( GPANHPSTMR1,		0x00 );		// 0x016B
	
	// State2,4 Step Time Setting
	RegWriteA( GPANSTT2TMR0,	0x01 );		// 0x013C
	RegWriteA( GPANSTT2TMR1,	0x00 );		// 0x013D
	RegWriteA( GPANSTT4TMR0,	0x02 );		// 0x013E
	RegWriteA( GPANSTT4TMR1,	0x00 );		// 0x013F
	
	RegWriteA( GPANSTTXXXTH,	0x00 );		// 0x015D

#ifdef GAIN_CONT
	RamWrite32A( gxlevmid, TRI_LEVEL );					// 0x182D	Low Th
	RamWrite32A( gxlevhgh, TRI_HIGH );					// 0x182E	Hgh Th
	RamWrite32A( gylevmid, TRI_LEVEL );					// 0x192D	Low Th
	RamWrite32A( gylevhgh, TRI_HIGH );					// 0x192E	Hgh Th
	RamWrite32A( gxadjmin, XMINGAIN );					// 0x18BA	Low gain
	RamWrite32A( gxadjmax, XMAXGAIN );					// 0x18BB	Hgh gain
	RamWrite32A( gxadjdn, XSTEPDN );					// 0x18BC	-step
	RamWrite32A( gxadjup, XSTEPUP );					// 0x18BD	+step
	RamWrite32A( gyadjmin, YMINGAIN );					// 0x19BA	Low gain
	RamWrite32A( gyadjmax, YMAXGAIN );					// 0x19BB	Hgh gain
	RamWrite32A( gyadjdn, YSTEPDN );					// 0x19BC	-step
	RamWrite32A( gyadjup, YSTEPUP );					// 0x19BD	+step
	
	RegWriteA( GLEVGXADD, (unsigned char)XMONADR );		// 0x0120	Input signal
	RegWriteA( GLEVGYADD, (unsigned char)YMONADR );		// 0x0124	Input signal
	RegWriteA( GLEVTMR, 		TIMEBSE );				// 0x0124	Base Time
	RegWriteA( GLEVTMRLOWGX, 	TIMELOW );				// 0x0121	X Low Time
	RegWriteA( GLEVTMRMIDGX, 	TIMEMID );				// 0x0122	X Mid Time
	RegWriteA( GLEVTMRHGHGX, 	TIMEHGH );				// 0x0123	X Hgh Time
	RegWriteA( GLEVTMRLOWGY, 	TIMELOW );				// 0x0125	Y Low Time
	RegWriteA( GLEVTMRMIDGY, 	TIMEMID );				// 0x0126	Y Mid Time
	RegWriteA( GLEVTMRHGHGY, 	TIMEHGH );				// 0x0127	Y Hgh Time
	RegWriteA( GLEVFILMOD, 		0x00 );					// 0x0129	select output signal
	RegWriteA( GADJGANADD, (unsigned char)GANADR );		// 0x012A	control address
	RegWriteA( GADJGANGO, 		0x00 );					// 0x0108	manual off

	/* exe function */
	AutoGainControlSw( OFF ) ;							/* Auto Gain Control Mode OFF */
#endif
	
	/*Gyro Filter On*/
	RegWriteA( GEQON	, 0x01 );		// 0x0100		[ - | - | - | - ][ - | - | - | CmEqOn ]


}



//********************************************************************************
// Function Name 	: IniHfl
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Hall Filter Initial Parameter Setting
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniHfl( void )
{
	unsigned short	UsAryId ;
	
	
	// Hall&Gyro Register Parameter Setting
	UsAryId	= 0 ;
	while( CsHalReg[ UsAryId ].UsRegAdd != 0xFFFF )
	{
		RegWriteA( CsHalReg[ UsAryId ].UsRegAdd, CsHalReg[ UsAryId ].UcRegDat ) ;
		UsAryId++ ;
	}

	// Hall Filter Parameter Setting
	UsAryId	= 0 ;
	while( CsHalFil[ UsAryId ].UsRamAdd != 0xFFFF )
	{
		RamWriteA( CsHalFil[ UsAryId ].UsRamAdd, CsHalFil[ UsAryId ].UsRamDat ) ;
		UsAryId++ ;
	}
	
}



//********************************************************************************
// Function Name 	: IniGfl
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Gyro Filter Initial Parameter Setting
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniGfl( void )
{
 	unsigned short	UsAryId ;
	
	// Gyro Filter Parameter Setting
	UsAryId	= 0 ;
	while( CsGyrFil[ UsAryId ].UsRamAdd != 0xFFFF )
	{
		RamWrite32A( CsGyrFil[ UsAryId ].UsRamAdd, CsGyrFil[ UsAryId ].UlRamDat ) ;
		UsAryId++ ;
	}
	
}



//********************************************************************************
// Function Name 	: IniAdj
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Adjust Value Setting
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniAdj( void )
{
	
	RegWriteA( CMSDAC, BAIS_CUR ) ;				// 0x0261	Hall Dacd¬
	RegWriteA( OPGSEL, AMP_GAIN ) ;				// 0x0262	Hall amp Gain
	
#ifdef USE_EXE2PROM
	unsigned short	UsAdjCompF ;
	unsigned short	UsAdjData ;
	unsigned long	UlAdjData ;
	
	E2pRed( (unsigned short)ADJ_COMP_FLAG, 2, ( unsigned char * )&UsAdjCompF ) ;	// Eeprom Read
	
	/* Hall Xaxis Bias,Offset */
	if( (UsAdjCompF == 0x0000 ) || (UsAdjCompF & ( EXE_HXADJ - EXE_END )) ){
		RamWriteA( DAHLXO, DAHLXO_INI ) ;	// 0x1114
		RamWriteA( DAHLXB, DAHLXB_INI ) ;	// 0x1115
		RamWriteA( ADHXOFF, 0x0000 ) ;		// 0x1102
	}else{
		E2pRed( (unsigned short)HALL_OFFSET_X, 2, ( unsigned char * )&UsAdjData ) ;
		RamWriteA( DAHLXO, UsAdjData ) ;	// 0x1114
		E2pRed( (unsigned short)HALL_BIAS_X, 2, ( unsigned char * )&UsAdjData ) ;
		RamWriteA( DAHLXB, UsAdjData ) ;	// 0x1115
		E2pRed( (unsigned short)HALL_AD_OFFSET_X, 2, ( unsigned char * )&UsAdjData ) ;
		RamWriteA( ADHXOFF, UsAdjData ) ;	// 0x1102
	}
			
	/* Hall Yaxis Bias,Offset */
	if( (UsAdjCompF == 0x0000 ) || (UsAdjCompF & ( EXE_HYADJ - EXE_END )) ){
		RamWriteA( DAHLYO, DAHLYO_INI ) ;	// 0x1116
		RamWriteA( DAHLYB, DAHLYB_INI ) ;	// 0x1117
		RamWriteA( ADHYOFF, 0x0000 ) ;		// 0x1105
	}else{
		E2pRed( (unsigned short)HALL_OFFSET_Y, 2, ( unsigned char * )&UsAdjData ) ;
		RamWriteA( DAHLYO, UsAdjData ) ;	// 0x1116
		E2pRed( (unsigned short)HALL_BIAS_Y, 2, ( unsigned char * )&UsAdjData ) ;
		RamWriteA( DAHLYB, UsAdjData ) ;	// 0x1117
		E2pRed( (unsigned short)HALL_AD_OFFSET_Y, 2, ( unsigned char * )&UsAdjData ) ;
		RamWriteA( ADHYOFF, UsAdjData ) ;	// 0x1105
	}
			
	/* Hall Xaxis Loop Gain */
	if( (UsAdjCompF == 0x0000 ) || (UsAdjCompF & ( EXE_LXADJ - EXE_END )) ){
		RamWriteA( lxgain, LXGAIN_INI ) ;	// 0x132A
	}else{
		E2pRed( (unsigned short)LOOP_GAIN_X, 2, ( unsigned char * )&UsAdjData ) ;
		RamWriteA( lxgain, UsAdjData ) ;	// 0x132A
	}
		
	/* Hall Yaxis Loop Gain */
	if( (UsAdjCompF == 0x0000 ) || (UsAdjCompF & ( EXE_LYADJ - EXE_END )) ){
		RamWriteA( lygain, LYGAIN_INI ) ;	// 0x136A
	}else{
		E2pRed( (unsigned short)LOOP_GAIN_Y, 2, ( unsigned char * )&UsAdjData ) ;
		RamWriteA( lygain, UsAdjData ) ;	// 0x136A
	}
		
	// X axis Optical Center Offset Read & Setting
	E2pRed( (unsigned short)OPT_CENTER_X, 2, ( unsigned char * )&UsAdjData ) ;
	if( ( UsAdjData != 0x0000 ) && ( UsAdjData != 0xffff )){
		UsCntXof = UsAdjData ;					/* Set Optical center X value */
	} else {
		UsCntXof = OPTCEN_X ;						/* Clear Optical center X value */
	}
	RamWriteA( HXINOD, UsCntXof ) ;				// 0x1127

	// Y axis Optical Center Offset Read & Setting
	E2pRed( (unsigned short)OPT_CENTER_Y, 2, ( unsigned char * )&UsAdjData ) ;
	if( ( UsAdjData != 0x0000 ) && ( UsAdjData != 0xffff )){
		UsCntYof = UsAdjData ;					/* Set Optical center Y value */
	} else {
		UsCntYof = OPTCEN_Y ;						/* Clear Optical center Y value */
	}
	RamWriteA( HYINOD, UsCntYof ) ;				// 0x1167
	
	/* Gyro Xaxis Offset */
	E2pRed( (unsigned short)GYRO_AD_OFFSET_X, 2, ( unsigned char * )&UsAdjData ) ;
	if( ( UsAdjData == 0x0000 ) || ( UsAdjData == 0xffff )){
		RamWriteA( ADGXOFF, 0x0000 ) ;							// 0x1108
		RegWriteA( IZAH, DGYRO_OFST_XH ) ;						// 0x03A0		Set Offset High byte
		RegWriteA( IZAL, DGYRO_OFST_XL ) ;						// 0x03A1		Set Offset Low byte
	}else{
		RamWriteA( ADGXOFF, 0x0000 ) ;							// 0x1108
		RegWriteA( IZAH, (unsigned char)(UsAdjData >> 8) ) ;	// 0x03A0		Set Offset High byte
		RegWriteA( IZAL, (unsigned char)(UsAdjData) ) ;			// 0x03A1		Set Offset Low byte
	}
	
	/* Gyro Yaxis Offset */
	E2pRed( (unsigned short)GYRO_AD_OFFSET_Y, 2, ( unsigned char * )&UsAdjData ) ;
	if( ( UsAdjData == 0x0000 ) || ( UsAdjData == 0xffff )){
		RamWriteA( ADGYOFF, 0x0000 ) ;							// 0x110B
		RegWriteA( IZBH, DGYRO_OFST_YH ) ;						// 0x03A2		Set Offset High byte
		RegWriteA( IZBL, DGYRO_OFST_YL ) ;						// 0x03A3		Set Offset Low byte
	}else{
		RamWriteA( ADGYOFF, 0x0000 ) ;							// 0x110B
		RegWriteA( IZBH, (unsigned char)(UsAdjData >> 8) ) ;	// 0x03A2		Set Offset High byte
		RegWriteA( IZBL, (unsigned char)(UsAdjData) ) ;			// 0x03A3		Set Offset Low byte
	}
		
	/* Gyro Xaxis Gain */
	E2pRed( (unsigned short)GYRO_GAIN_X, 4 , ( unsigned char * )&UlAdjData ) ;
	if( ( UlAdjData != 0x00000000 ) && ( UlAdjData != 0xffffffff )){
		RamWrite32A( gxzoom, UlAdjData ) ;		// 0x1828 Gyro X axis Gain adjusted value
	}else{
		RamWrite32A( gxzoom, GXGAIN_INI ) ;		// 0x1828 Gyro X axis Gain adjusted initial value
	}
	
	/* Gyro Yaxis Gain */
	E2pRed( (unsigned short)GYRO_GAIN_Y, 4 , ( unsigned char * )&UlAdjData ) ;
	if( ( UlAdjData != 0x00000000 ) && ( UlAdjData != 0xffffffff )){
		RamWrite32A( gyzoom, UlAdjData ) ;		// 0x1928 Gyro Y axis Gain adjusted value
	}else{
		RamWrite32A( gyzoom, GXGAIN_INI ) ;		// 0x1928 Gyro Y axis Gain adjusted initial value
	}
	
	/* OSC Clock value */
	E2pRed( (unsigned short)OSC_CLK_VAL, 2 , ( unsigned char * )&UsAdjData ) ;
	if((unsigned char)UsAdjData != 0xff ){
		UsCntYof = UsAdjData ;					/* Optical center X value */
		RegWriteA( OSCSET, (unsigned char)UsAdjData ) ;		// 0x0264
	}else{
		RegWriteA( OSCSET, OSC_INI ) ;						// 0x0264
	}
	
	
#else	
	/* adjusted value */
	RegWriteA( IZAH,	DGYRO_OFST_XH ) ;	// 0x03A0		Set Offset High byte
	RegWriteA( IZAL,	DGYRO_OFST_XL ) ;	// 0x03A1		Set Offset Low byte
	RegWriteA( IZBH,	DGYRO_OFST_YH ) ;	// 0x03A2		Set Offset High byte
	RegWriteA( IZBL,	DGYRO_OFST_YL ) ;	// 0x03A3		Set Offset Low byte
	
	RamWriteA( DAHLXO, DAHLXO_INI ) ;		// 0x1114
	RamWriteA( DAHLXB, DAHLXB_INI ) ;		// 0x1115
	RamWriteA( DAHLYO, DAHLYO_INI ) ;		// 0x1116
	RamWriteA( DAHLYB, DAHLYB_INI ) ;		// 0x1117
	RamWriteA( ADHXOFF, ADHXOFF_INI ) ;		// 0x1102
	RamWriteA( ADHYOFF, ADHYOFF_INI ) ;		// 0x1105
	RamWriteA( lxgain, LXGAIN_INI ) ;		// 0x132A
	RamWriteA( lygain, LYGAIN_INI ) ;		// 0x136A
	RamWriteA( ADGXOFF, 0x0000 ) ;			// 0x1108
	RamWriteA( ADGYOFF, 0x0000 ) ;			// 0x110B
	UsCntXof = OPTCEN_X ;					/* Clear Optical center X value */
	UsCntYof = OPTCEN_Y ;					/* Clear Optical center Y value */
	RamWriteA( HXINOD, UsCntXof ) ;			// 0x1127
	RamWriteA( HYINOD, UsCntYof ) ;			// 0x1167
	RamWrite32A( gxzoom, GXGAIN_INI ) ;		// 0x1828 Gyro X axis Gain adjusted value
	RamWrite32A( gyzoom, GYGAIN_INI ) ;		// 0x1928 Gyro Y axis Gain adjusted value

	RegWriteA( OSCSET, OSC_INI ) ;			// 0x0264	OSC adj
	
#endif	
	
	RamWriteA( pzgxp, PZGXP_INI ) ;			// 0x133C	X axis output direction initial value
	RamWriteA( pzgyp, PZGYP_INI ) ;			// 0x137C	Y axis output direction initial value
	
	RamWriteA( hxinog, 0x7fff ) ;			// 0x1128	back up initial value
	RamWriteA( hyinog, 0x7fff ) ;			// 0x1168	back up initial value
	
	SetZsp(0) ;								// Zoom coefficient Initial Setting
	
	RegWriteA( PWMA 	, 0xC0 );			// 0x0074		PWM enable

	RegWriteA( STBB 	, 0x0F );							// 0x0260 	[ - | - | - | - ][ STBOPAY | STBOPAX | STBDAC | STBADC ]

	RegWriteA( LXEQEN 	, 0x45 );			// 0x0084
	RegWriteA( LYEQEN 	, 0x45 );			// 0x008E
	
	SetPanTiltMode( OFF ) ;					/* Pan/Tilt OFF */
#ifdef H1COEF_CHANGER
	SetH1cMod( ACTMODE ) ;					/* Lvl Change Active mode */
#endif
	
	DrvSw( ON ) ;							/* 0x0070		Driver Mode setting */
}



//********************************************************************************
// Function Name 	: IniCmd
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Command Execute Process Initial
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	IniCmd( void )
{

	MemClr( ( unsigned char * )&StAdjPar, sizeof( stAdjPar ) ) ;	// Adjust Parameter Clear
	MemClr( ( unsigned char * )&StLbgCon, sizeof( stLbgCon ) ) ;	// Zoom Value Controler Clear
	
}


//********************************************************************************
// Function Name 	: BsyWit
// Retun Value		: NON
// Argment Value	: Trigger Register Address, Trigger Register Data
// Explanation		: Busy Wait Function
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	BsyWit( unsigned short	UsTrgAdr, unsigned char	UcTrgDat )
{
	unsigned char	UcFlgVal ;

	RegWriteA( UsTrgAdr, UcTrgDat ) ;	// Trigger Register Setting

	UcFlgVal	= 1 ;

	while( UcFlgVal ) {
		RegReadA( FLGM, &UcFlgVal ) ;		// 0x00F8
		UcFlgVal	&= 0x40 ;
	} ;

}


//********************************************************************************
// Function Name 	: Bsy2Wit
// Retun Value		: NON
// Argment Value	: void
// Explanation		: Busy2 Wait Function
// History			: First edition 						2011.05.17 Y.Shigeoka
//********************************************************************************
void	Bsy2Wit( void )
{
	unsigned char	UcFlgVal ;

	UcFlgVal	= 1 ;

	while( UcFlgVal ) {
		RegReadA( BSYSEL, &UcFlgVal ) ;
		UcFlgVal	&= 0x80 ;
	} ;

}


//********************************************************************************
// Function Name 	: MemClr
// Retun Value		: void
// Argment Value	: Clear Target Pointer, Clear Byte Number
// Explanation		: Memory Clear Function
// History			: First edition 						2009.07.30 Y.Tashita
//********************************************************************************
void	MemClr( unsigned char	*NcTgtPtr, unsigned short	UsClrSiz )
{
	unsigned short	UsClrIdx ;

	for ( UsClrIdx = 0 ; UsClrIdx < UsClrSiz ; UsClrIdx++ )
	{
		*NcTgtPtr	= 0 ;
		NcTgtPtr++ ;
	}
}


//HTC_START  20130329 removed
#if 0
//********************************************************************************
// Function Name 	: WitTim
// Retun Value		: NON
// Argment Value	: Wait Time(ms)
// Explanation		: Timer Wait Function
// History			: First edition 						2009.07.31 Y.Tashita
//********************************************************************************
void	WitTim( unsigned short	UsWitTim )
{
	unsigned long	UlLopIdx, UlWitCyc ;

	UlWitCyc	= ( unsigned long )( ( float )UsWitTim / NOP_TIME / ( float )12 ) ;

	for( UlLopIdx = 0 ; UlLopIdx < UlWitCyc ; UlLopIdx++ )
	{
		;
	}
}
#endif
//HTC_END

//********************************************************************************
// Function Name 	: GyOutSignal
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Select Gyro Signal Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
void	GyOutSignal( void )
{

	RegWriteA( GRADR0,	GYROX_INI ) ;			// 0x0383	Set Gyro signal to Gyro X filter
	RegWriteA( GRADR1,	GYROY_INI ) ;			// 0x0384	Set Gyro signal to Gyro Y filter
	
	/*Start OIS Reading*/
	RegWriteA( GRSEL	, 0x02 );							// 0x0380	[ - | - | - | - ][ - | SRDMOE | OISMODE | COMMODE ]

}

#ifdef STANDBY_MODE
//********************************************************************************
// Function Name 	: AccWit
// Retun Value		: NON
// Argment Value	: Trigger Register Data
// Explanation		: Acc Wait Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
void	AccWit( unsigned char UcTrgDat )
{
	unsigned char	UcFlgVal ;

	UcFlgVal	= 1 ;

	while( UcFlgVal ) {
		RegReadA( GRACC, &UcFlgVal ) ;
		UcFlgVal	&= UcTrgDat ;
	} ;

}

//********************************************************************************
// Function Name 	: SelectGySleep
// Retun Value		: NON
// Argment Value	: mode	
// Explanation		: Select Gyro mode Function
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
void	SelectGySleep( unsigned char UcSelMode )
{
	unsigned char	UcRamIni ;
	unsigned char	UcGrini ;

	if(UcSelMode == ON)
	{
		RegWriteA( GEQON, 0x00 ) ;			// 0x0100	GYRO Equalizer OFF
		RegWriteA( GRSEL,	0x01 ) ;		/* 0x0380	Set Command Mode			*/

		RegReadA( GRINI	, &UcGrini );					// 0x0381	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ LSBF | SLOWMODE | I2CMODE | - ]
		RegWriteA( GRINI, ( UcGrini | SLOWMODE) );		// 0x0381	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ LSBF | SLOWMODE | I2CMODE | - ]
		
		RegWriteA( GRADR0,	0x6B ) ;		/* 0x0383	Set Write Command			*/
		RegWriteA( GRACC,	0x02 ) ;		/* 0x0382	Set Read Trigger ON				*/
		AccWit( 0x10 ) ;					/* Digital Gyro busy wait 				*/
		RegReadA( GRADT0H, &UcRamIni ) ;	/* 0x0390 */
		
		UcRamIni |= 0x40 ;					/* Set Sleep bit */
		
		RegWriteA( GRADR0,	0x6B ) ;		/* 0x0383	Set Write Command			*/
		RegWriteA( GSETDT,	UcRamIni ) ;	/* 0x038A	Set Write Data(Sleep ON)	*/
		RegWriteA( GRACC,	0x10 ) ;		/* 0x0382	Set Trigger ON				*/
		AccWit( 0x10 ) ;					/* Digital Gyro busy wait 				*/

	}
	else
	{
		RegWriteA( GRADR0,	0x6B ) ;		/* 0x0383	Set Write Command			*/
		RegWriteA( GRACC,	0x02 ) ;		/* 0x0382	Set Read Trigger ON				*/
		AccWit( 0x10 ) ;					/* Digital Gyro busy wait 				*/
		RegReadA( GRADT0H, &UcRamIni ) ;	/* 0x0390 */
		
		UcRamIni &= ~0x40 ;					/* Clear Sleep bit */
		
		RegWriteA( GSETDT,	UcRamIni ) ;	// 0x038A	Set Write Data(Sleep OFF)
		RegWriteA( GRACC,	0x10 ) ;		/* 0x0382	Set Trigger ON				*/
		AccWit( 0x10 ) ;					/* Digital Gyro busy wait 				*/

		RegReadA( GRINI	, &UcGrini );					// 0x0381	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ LSBF | SLOWMODE | I2CMODE | - ]
		RegWriteA( GRINI, ( UcGrini & ~SLOWMODE) );		// 0x0381	[ PARA_REG | AXIS7EN | AXIS4EN | - ][ LSBF | SLOWMODE | I2CMODE | - ]
		
		GyOutSignal( ) ;					/* Select Gyro output signal 			*/
		
		WitTim( 50 ) ;						// 50ms wait
		
		RegWriteA( GEQON, 0x01 ) ;			// 0x0100	GYRO Equalizer ON
		ClrGyr( 0x06 , CLR_GYR_DLY_RAM );
	}
}
#endif

#ifdef	GAIN_CONT
//********************************************************************************
// Function Name 	: AutoGainControlSw
// Retun Value		: NON
// Argment Value	: 0 :OFF  1:ON
// Explanation		: Select Gyro Signal Function
// History			: First edition 						2010.11.30 Y.Shigeoka
//********************************************************************************
void	AutoGainControlSw( unsigned char UcModeSw )
{

	if( UcModeSw == OFF )
	{
		RegWriteA( GADJGANGXMOD, 	0xE0 );					// 0x012B	X exe off
		RegWriteA( GADJGANGYMOD, 	0xE0 );					// 0x012C	Y exe off
		RamWrite32A( GANADR			 , XMAXGAIN ) ;			// Gain Through
		RamWrite32A( GANADR | 0x0100 , YMAXGAIN ) ;			// Gain Through
	}
	else
	{
		RegWriteA( GADJGANGXMOD, 	0xE7 );					// 0x012B	X exe on
		RegWriteA( GADJGANGYMOD, 	0xE7 );					// 0x012C	Y exe on
	}

}
#endif


//********************************************************************************
// Function Name 	: ClrGyr
// Retun Value		: NON
// Argment Value	: UcClrFil - Select filter to clear.  If 0x00, clears entire filter
//					  UcClrMod - 0x01: Parameter RAM Clear, 0x02: Delay RAM Clear, 0x03: All RAM Clear
// Explanation		: Gyro RAM clear function
// History			: First edition 						2011.04.08 d.yamagata
//********************************************************************************
void	ClrGyr( unsigned char UcClrFil , unsigned char UcClrMod )
{
	unsigned char	UcRamClr;
	unsigned char	UcClrBit;

	while(1){
		if( UcClrMod == CLR_GYR_DLY_RAM )
		{
			if( UcClrFil & 0x10 ){
				UcClrBit = 0x10 ;
			}else if( UcClrFil & 0x08 ){
				UcClrBit = 0x08 ;
			}else if( UcClrFil & 0x04 ){
				UcClrBit = 0x04 ;
			}else if( UcClrFil & 0x02 ){
				UcClrBit = 0x02 ;
			}else if( UcClrFil & 0x01 ){
				UcClrBit = 0x01 ;
			}else{
				UcClrBit = 0x00 ;
			}
				
			UcClrFil &= ~UcClrBit ;

		}else{
			UcClrBit = 0x00 ;
		}
		
		/*Select Filter to clear*/
		RegWriteA( GRAMDLYMOD	, UcClrBit ) ;	// 0x011B	[ - | - | - | P ][ T | L | H | I ]
												//				wèµ½tB^[ðNA·éªA
												//				0x00ÌêÍtB^[SÌðNA·é

		/*Enable Clear*/
		RegWriteA( GRAMINITON	, UcClrMod ) ;	// 0x0103	[ - | - | - | - ][ - | - | xClr | WClr ]
		
		/*Check RAM Clear complete*/
		do{
			RegReadA( GRAMINITON, &UcRamClr );
			UcRamClr &= 0x03;
		}while( UcRamClr != 0x00 );

		if(( UcClrMod != CLR_GYR_DLY_RAM ) || ( UcClrFil == 0x00 )){
			break ;
		}
	}
}


//********************************************************************************
// Function Name 	: DrvSw
// Retun Value		: NON
// Argment Value	: 0:OFF  1:ON
// Explanation		: Driver Mode setting function
// History			: First edition 						2012.04.25 Y.Shigeoka
//********************************************************************************
void	DrvSw( unsigned char UcDrvSw )
{

	if( UcDrvSw == ON )
	{
		if( UcPwmMod == PWMMOD_CVL ) {
			RegWriteA( DRVFC	, 0xE3 );			// 0x0070	MODE=2,Drvier Block Ena=1,FullMode=1
		}else{
#ifdef	LOWCURRENT
		RegWriteA( DRVFC	, 0x03 );			// 0x0070	DMODE=0,DRMODE=0,DRMODESEL=0
#else
		RegWriteA( DRVFC	, 0xC3 );			// 0x0070	MODE=1,Drvier Block Ena=1,FullMode=1
#endif
		}
	}
	else
	{
		RegWriteA( DRVFC	, 0x00 );				// 0x0070	Drvier Block Ena=0
	}
}


