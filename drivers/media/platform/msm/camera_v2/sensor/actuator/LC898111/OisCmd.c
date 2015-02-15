//********************************************************************************
//
//		<< LC898109 Evaluation Soft >>
//		Program Name	: OisCmd.c
//		Design			: Y.Yamada
//		History			: First edition						2009.07.31 Y.Tashita
//********************************************************************************
//**************************
//	Include Header File		
//**************************
#define		OISCMD

//#include	"Main.h"
//#include	"Cmd.h"
#include	"Ois.h"
#include	"OisDef.h"

/* HTC_START 20130329 */
#include	"HtcActOisBinder.h"
/* HTC_END */


//**************************
//	Local Function Prottype	
//**************************
void			MesFil( unsigned char ) ;					// Measure Filter Setting
#ifdef		MODULE_CALIBRATION		/* calibration code for Module maker */
 #ifndef	HALLADJ_HW
  void			LopIni( unsigned char ) ;					// Loop Gain Initialize
 #endif
 void			LopPar( unsigned char ) ;					// Loop Gain Parameter initialize
 #ifndef	HALLADJ_HW
  void			LopSin( unsigned char, unsigned char ) ;	// Loop Gain Sin Wave Output
  unsigned char	LopAdj( unsigned char ) ;					// Loop Gain Adjust
  void			LopMes( void ) ;							// Loop Gain Measure
 #endif
 #ifndef	HALLADJ_HW
 unsigned long	GinMes( unsigned char ) ;					// Measure Result Getting
 #endif
#endif
void			GyrCon( unsigned char ) ;					// Gyro Filter Control
short			GenMes( unsigned short, unsigned char ) ;	// General Measure

#ifdef		MODULE_CALIBRATION		/* calibration code for Module maker */
 #ifndef	HALLADJ_HW
//  unsigned long	TnePtp( unsigned char, unsigned char ) ;	// Get Hall Peak to Peak Values
//  unsigned char	TneCen( unsigned char, UnDwdVal ) ;			// Tuning Hall Center
  unsigned long	TneOff( UnDwdVal, unsigned char ) ;			// Hall Offset Tuning
  unsigned long	TneBia( UnDwdVal, unsigned char ) ;			// Hall Bias Tuning
 #endif
#endif
#ifdef USE_EXE2PROM
void			AdjSav( unsigned char ) ;					// Adjust Result Save
#endif
void 			StbOnn( void ) ;							// Servo ON Slope mode

void			SetSineWave(   unsigned char , unsigned char );
void			StartSineWave( void );
void			StopSineWave(  void );

void			SetMeasFil(    unsigned char , unsigned char , unsigned char );
void			StartMeasFil( void );
void			StopMeasFil( void );



//**************************
//	define					
//**************************
#define		MES_XG1			0								// LXG1 Measure Mode
#define		MES_XG2			1								// LXG2 Measure Mode

#define		HALL_ADJ		0
#define		LOOPGAIN		1
#define		THROUGH			2
#define		NOISE			3

// Measure Mode

 #define		TNE 			80								// Waiting Time For Movement
#ifdef	HALLADJ_HW

 #define	 __MEASURE_LOOPGAIN 	 0x00
 #define	 __MEASURE_BIASOFFSET	 0x01

#else

 #define		MARJIN			0x0300							// Marjin
 #define		BIAS_ADJ_BORDER	0x1998							// HALL_MAX_GAP < BIAS_ADJ_BORDER < HALL_MIN_GAP  80%

 #define		HALL_MAX_GAP	BIAS_ADJ_BORDER - MARJIN
 #define		HALL_MIN_GAP	BIAS_ADJ_BORDER + MARJIN
 #define		BIAS_LIMIT		0xFFFF							// HALL BIAS LIMIT
 #define		OFFSET_DIV		2								// Divide Difference For Offset Step
 #define		TIME_OUT		40								// Time Out Count
// #define		PTP_BEFORE		0
// #define		PTP_AFTER		1

#endif


//**************************
//	Global Variable			
//**************************
#ifdef	HALLADJ_HW
 unsigned char UcAdjBsy;

#else
 unsigned short	UsStpSiz	= 0 ;							// Bias Step Size
 unsigned short	UsErrBia, UsErrOfs ;
#endif



//**************************
//	Const					
//**************************
// gxlens Setting Value
#define		ZOOMTBL	4
const unsigned long	ClGyxZom[ ZOOMTBL ]	= {
		0x3F800000,
		0x3F800000,		/* Infini */
		0x3FBD317A,		/* 10cm */
		0x3FF6D6ED		/*	6cm */
	} ;

// gylens Setting Value
const unsigned long	ClGyyZom[ ZOOMTBL ]	= {
		0x3F800000,
		0x3F800000,		/* Infini */
		0x3FBD317A,		/* 10cm */
		0x3FF6D6ED 		/*	6cm */
	} ;


// DI Coefficient Setting Value
#define		COEFTBL	7
const unsigned long	ClDiCof[ COEFTBL ]	= {
		0x3F7FF000,		/* 0 */
		0x3F7FF200,		/* 1 */
		0x3F7FF400,		/* 2 */
		0x3F7FF600,		/* 3 */
		0x3F7FF800,		/* 4 */
		0x3F7FFA00,		/* 5 */
		0x3F7FFC00		/* 6 */
	} ;


#ifdef		MODULE_CALIBRATION		/* calibration code for Module maker */
//********************************************************************************
// Function Name 	: TneRun
// Retun Value		: Hall Tuning SUCCESS or FAILURE
// Argment Value	: NON
// Explanation		: Hall System Auto Adjustment Function
// History			: First edition 						2009.12.1 YS.Kim
//********************************************************************************
 #ifdef	HALLADJ_HW
 #else
 #define	INITBIASVAL		0xA000
 #endif
unsigned short	TneRun( void )
{
	unsigned char	UcHlySts, UcHlxSts, UcAtxSts, UcAtySts ;
	unsigned short	UsFinSts , UsOscSts ; 								// Final Adjustment state
 #ifndef	HALLADJ_HW
	UnDwdVal		StTneVal ;
 #endif
 #ifdef USE_EXE2PROM
	unsigned short	UsDatBk ;
 #endif

	/* OSC adjustment */
	 UsOscSts	= OscAdj() ;
 #ifdef USE_EXE2PROM
	UsDatBk = (unsigned short)StAdjPar.UcOscVal ;
	E2pWrt( OSC_CLK_VAL,	2, ( unsigned char * )&UsDatBk ) ;
 #endif


 #ifdef	HALLADJ_HW
	UcHlySts = BiasOffsetAdj( Y_DIR , 0 ) ;
	WitTim( TNE ) ;
	UcHlxSts = BiasOffsetAdj( X_DIR , 0 ) ;
	WitTim( TNE ) ;
	UcHlySts = BiasOffsetAdj( Y_DIR , 1 ) ;
	WitTim( TNE ) ;
	UcHlxSts = BiasOffsetAdj( X_DIR , 1 ) ;
 #else
	SrvCon( X_DIR, ON ) ;
	SrvCon( Y_DIR, OFF ) ;
	WitTim( TNE ) ;

	StTneVal.UlDwdVal	= TnePtp( Y_DIR , PTP_BEFORE ) ;
	UcHlySts	= TneCen( Y_DIR, StTneVal ) ;
	
	SrvCon( Y_DIR, ON ) ;
	SrvCon( X_DIR, OFF ) ;
	WitTim( TNE ) ;

	StTneVal.UlDwdVal	= TnePtp( X_DIR , PTP_BEFORE ) ;
	UcHlxSts	= TneCen( X_DIR, StTneVal ) ;

	SrvCon( X_DIR, ON ) ;
	SrvCon( Y_DIR, OFF ) ;
	WitTim( TNE ) ;

	StTneVal.UlDwdVal	= TnePtp( Y_DIR , PTP_AFTER ) ;
	UcHlySts	= TneCen( Y_DIR, StTneVal ) ;

	SrvCon( Y_DIR, ON ) ;
	SrvCon( X_DIR, OFF ) ;
	WitTim( TNE ) ;

	StTneVal.UlDwdVal	= TnePtp( X_DIR , PTP_AFTER ) ;
	UcHlxSts	= TneCen( X_DIR, StTneVal ) ;

	/* TEST Sequence */
	if( UcHlxSts - EXE_END )
	{
		SrvCon( Y_DIR, ON ) ;
		SrvCon( X_DIR, OFF ) ;
		WitTim( TNE ) ;

		RamWriteA( DAHLXB, INITBIASVAL ) ;		// Hall X Bias Ram Write
		StTneVal.UlDwdVal	= TnePtp( X_DIR , PTP_AFTER ) ;
		UcHlxSts	= TneCen( X2_DIR, StTneVal ) ;

	}
	if( UcHlySts - EXE_END )
	{
		SrvCon( X_DIR, ON ) ;
		SrvCon( Y_DIR, OFF ) ;
		WitTim( TNE ) ;

		RamWriteA( DAHLYB, INITBIASVAL ) ;		// Hall Y Bias Ram Write
		StTneVal.UlDwdVal	= TnePtp( Y_DIR , PTP_AFTER ) ;
		UcHlySts	= TneCen( Y2_DIR, StTneVal ) ;

	}
	
  #ifdef	NEUTRAL_CENTER
	TneHvc();
  #else	//NEUTRAL_CENTER	

	/* TEST Sequence */
	
	SrvCon( X_DIR, ON ) ;
	SrvCon( Y_DIR, ON ) ;
  #endif	//NEUTRAL_CENTER
	
 #endif
	WitTim( TNE ) ;

	RamWriteA( ADHXOFF, StAdjPar.StHalAdj.UsHlxCna	) ;	 	// 0x1102	Center Value Write To ADXOFFSET
	RamWriteA( ADHYOFF, StAdjPar.StHalAdj.UsHlyCna	) ; 	// 0x1105	Center Value Write To ADYOFFSET

	RamReadA( DAHLXO, &StAdjPar.StHalAdj.UsHlxOff ) ;		// 0x1114
	RamReadA( DAHLXB, &StAdjPar.StHalAdj.UsHlxGan ) ;		// 0x1115
	RamReadA( DAHLYO, &StAdjPar.StHalAdj.UsHlyOff ) ;		// 0x1116
	RamReadA( DAHLYB, &StAdjPar.StHalAdj.UsHlyGan ) ;		// 0x1117
	RamReadA( ADHXOFF, &StAdjPar.StHalAdj.UsAdxOff ) ;		// 0x1102
	RamReadA( ADHYOFF, &StAdjPar.StHalAdj.UsAdyOff ) ;		// 0x1105

 #ifdef USE_EXE2PROM
	AdjSav( Y_DIR ) ;
	AdjSav( X_DIR ) ;
 #endif
	
	WitTim( TNE ) ;

	// X Loop Gain Adjust
	UcAtxSts	= LopGan( X_DIR ) ;
 #ifdef USE_EXE2PROM
	E2pWrt( LOOP_GAIN_STATUS_X,	2, ( unsigned char * )&StAdjPar.StLopGan.UsLxgSts ) ;
	E2pWrt( LOOP_GAIN_X,		2, ( unsigned char * )&StAdjPar.StLopGan.UsLxgVal ) ;
 #endif

	// Y Loop Gain Adjust
	UcAtySts	= LopGan( Y_DIR ) ;
 #ifdef USE_EXE2PROM
	E2pWrt( LOOP_GAIN_STATUS_Y,	2, ( unsigned char * )&StAdjPar.StLopGan.UsLygSts ) ;
	E2pWrt( LOOP_GAIN_Y,		2, ( unsigned char * )&StAdjPar.StLopGan.UsLygVal ) ;
 #endif

	TneGvc() ;
 #ifdef USE_EXE2PROM
	E2pWrt( GYRO_OFFSET_STATUS_X,	2, ( unsigned char * )&StAdjPar.StGvcOff.UsGxoSts ) ;
	E2pWrt( GYRO_AD_OFFSET_X,	2, ( unsigned char * )&StAdjPar.StGvcOff.UsGxoVal ) ;
	E2pWrt( GYRO_OFFSET_STATUS_Y,	2, ( unsigned char * )&StAdjPar.StGvcOff.UsGyoSts ) ;
	E2pWrt( GYRO_AD_OFFSET_Y,	2, ( unsigned char * )&StAdjPar.StGvcOff.UsGyoVal ) ;
 #endif


	UsFinSts	= (unsigned short)( UcHlxSts - EXE_END ) + (unsigned short)( UcHlySts - EXE_END ) + (unsigned short)( UcAtxSts - EXE_END ) + (unsigned short)( UcAtySts - EXE_END ) + ( UsOscSts - (unsigned short)EXE_END ) + (unsigned short)EXE_END ;
 #ifdef USE_EXE2PROM
	
	E2pWrt( ADJ_COMP_FLAG,	2, ( unsigned char * )&UsFinSts ) ;
 #endif

	return( UsFinSts ) ;
}


 #ifndef	HALLADJ_HW

//********************************************************************************
// Function Name 	: TnePtp
// Retun Value		: Hall Top & Bottom Gaps
// Argment Value	: X,Y Direction, Adjust Before After Parameter
// Explanation		: Measuring Hall Paek To Peak
// History			: First edition 						2009.12.1 YS.Kim
//********************************************************************************
 #define		HALL_H_VAL	0x7FFF
 
unsigned long	TnePtp ( unsigned char	UcDirSel, unsigned char	UcBfrAft )
{
	UnDwdVal		StTneVal ;
	unsigned char	UcRegVal ;

	MesFil( THROUGH ) ;					// ªèptB^[ðÝè·éB


	if ( !UcDirSel ) {
		RamWriteA( wavxg , HALL_H_VAL );												// 0x13C3
		SetSinWavePara( 0x0A , XHALWAVE ); 
		if( UcPwmMod == PWMMOD_PWM ) {
			RegReadA( LXX3, &UcRegVal ) ;
			RegWriteA( LXX3,  UcRegVal | 0x04 ) ;
		}
	}else{
		RamWriteA( wavyg , HALL_H_VAL );												// 0x13C4
		SetSinWavePara( 0x0A , YHALWAVE ); 
		if( UcPwmMod == PWMMOD_PWM ) {
			RegReadA( LYX3, &UcRegVal ) ;
			RegWriteA( LYX3,  UcRegVal | 0x04 ) ;
		}
	}

	if ( !UcDirSel ) {					// AXIS X
		RegWriteA( MS1INADD, (unsigned char)HXIDAT ) ;
	} else {							// AXIS Y
		RegWriteA( MS1INADD, (unsigned char)HYIDAT ) ;
	}

	BsyWit( MSMA, 0x02 ) ;				// MSMA		Sine wave Measure

	RegWriteA( MSMA, 0x00 ) ;			// MSMA		Continuous measure stop

	RamReadA( MSSMAXAV, &StTneVal.StDwdVal.UsHigVal ) ;
	RamReadA( MSSMINAV, &StTneVal.StDwdVal.UsLowVal ) ;

	if ( !UcDirSel ) {					// AXIS X
		SetSinWavePara( 0x00 , XHALWAVE ); 	/* STOP */
		if( UcPwmMod == PWMMOD_PWM ) {
			RegWriteA( LXX3,  UcRegVal ) ;
		}
	}else{
		SetSinWavePara( 0x00 , YHALWAVE ); 	/* STOP */
		if( UcPwmMod == PWMMOD_PWM ) {
			RegWriteA( LYX3,  UcRegVal ) ;
		}
	}

	if( UcBfrAft == 0 ) {
		if( UcDirSel == X_DIR ) {
			StAdjPar.StHalAdj.UsHlxCen	= ( ( signed short )StTneVal.StDwdVal.UsHigVal + ( signed short )StTneVal.StDwdVal.UsLowVal ) / 2 ;
			StAdjPar.StHalAdj.UsHlxMax	= StTneVal.StDwdVal.UsHigVal ;
			StAdjPar.StHalAdj.UsHlxMin	= StTneVal.StDwdVal.UsLowVal ;
		} else {
			StAdjPar.StHalAdj.UsHlyCen	= ( ( signed short )StTneVal.StDwdVal.UsHigVal + ( signed short )StTneVal.StDwdVal.UsLowVal ) / 2 ;
			StAdjPar.StHalAdj.UsHlyMax	= StTneVal.StDwdVal.UsHigVal ;
			StAdjPar.StHalAdj.UsHlyMin	= StTneVal.StDwdVal.UsLowVal ;
		}
	} else {
		if( UcDirSel == X_DIR ){
 #ifdef	NEUTRAL_CENTER
 #else	//NEUTRAL_CENTER
			StAdjPar.StHalAdj.UsHlxCna	= ( ( signed short )StTneVal.StDwdVal.UsHigVal + ( signed short )StTneVal.StDwdVal.UsLowVal ) / 2 ;
 #endif	//NEUTRAL_CENTER
			StAdjPar.StHalAdj.UsHlxMxa	= StTneVal.StDwdVal.UsHigVal ;
			StAdjPar.StHalAdj.UsHlxMna	= StTneVal.StDwdVal.UsLowVal ;
		} else {
 #ifdef	NEUTRAL_CENTER
 #else	//NEUTRAL_CENTER
			StAdjPar.StHalAdj.UsHlyCna	= ( ( signed short )StTneVal.StDwdVal.UsHigVal + ( signed short )StTneVal.StDwdVal.UsLowVal ) / 2 ;
 #endif	//NEUTRAL_CENTER
			StAdjPar.StHalAdj.UsHlyMxa	= StTneVal.StDwdVal.UsHigVal ;
			StAdjPar.StHalAdj.UsHlyMna	= StTneVal.StDwdVal.UsLowVal ;
		}
	}


	StTneVal.StDwdVal.UsHigVal	= 0x7fff - StTneVal.StDwdVal.UsHigVal ;		// Maximum Gap = Maximum - Hall Peak Top
	StTneVal.StDwdVal.UsLowVal	= StTneVal.StDwdVal.UsLowVal - 0x7fff ; 	// Minimum Gap = Hall Peak Bottom - Minimum

	
	return( StTneVal.UlDwdVal ) ;
}

//********************************************************************************
// Function Name 	: TneCen
// Retun Value		: Hall Center Tuning Result
// Argment Value	: X,Y Direction, Hall Top & Bottom Gaps
// Explanation		: Hall Center Tuning Function
// History			: First edition 						2009.12.1 YS.Kim
//********************************************************************************
#define		DIVVAL	2
unsigned char	TneCen( unsigned char	UcTneAxs, UnDwdVal	StTneVal )
{
	unsigned char 	UcTneRst, UcTmeOut, UcTofRst ;
	unsigned short	UsOffDif ;

	UsErrBia	= 0 ;
	UsErrOfs	= 0 ;
	UcTmeOut	= 1 ;
	UsStpSiz	= 1 ;
	UcTneRst	= FAILURE ;
	UcTofRst	= FAILURE ;

	while ( UcTneRst && UcTmeOut )
	{
		if( UcTofRst == FAILURE ) {
			StTneVal.UlDwdVal	= TneOff( StTneVal, UcTneAxs ) ;
		} else {
			StTneVal.UlDwdVal	= TneBia( StTneVal, UcTneAxs ) ;
			UcTofRst	= FAILURE ;
		}

		if ( StTneVal.StDwdVal.UsHigVal > StTneVal.StDwdVal.UsLowVal ) {									// Check Offset Tuning Result
			UsOffDif	= ( StTneVal.StDwdVal.UsHigVal - StTneVal.StDwdVal.UsLowVal ) / 2 ;
		} else {
			UsOffDif	= ( StTneVal.StDwdVal.UsLowVal - StTneVal.StDwdVal.UsHigVal ) / 2 ;
		}

		if( UsOffDif < MARJIN ) {
			UcTofRst	= SUCCESS ;
		} else {
			UcTofRst	= FAILURE ;
		}

		if ( ( StTneVal.StDwdVal.UsHigVal < HALL_MIN_GAP && StTneVal.StDwdVal.UsLowVal < HALL_MIN_GAP )		// Check Tuning Result 
		&& ( StTneVal.StDwdVal.UsHigVal > HALL_MAX_GAP && StTneVal.StDwdVal.UsLowVal > HALL_MAX_GAP ) ) {
			UcTneRst	= SUCCESS ;
			break ;
		} else if ( UsStpSiz == 0 ) {
			UcTneRst	= SUCCESS ;
			break ;
		} else {
			UcTneRst	= FAILURE ;
			UcTmeOut++ ;
		}

		if( UcTneAxs & 0xF0 )
		{
			if ( ( UcTmeOut / DIVVAL ) == TIME_OUT ) {
				UcTmeOut	= 0 ;
			}		 																							// Set Time Out Count
		}else{
			if ( UcTmeOut == TIME_OUT ) {
				UcTmeOut	= 0 ;
			}		 																							// Set Time Out Count
		}
	}

	if( UcTneRst == FAILURE ) {
//		if( !UcTneAxs ) {
		if( !( UcTneAxs & 0x0F ) ) {
			UcTneRst					= EXE_HXADJ ;
			StAdjPar.StHalAdj.UsHlxGan	= 0xFFFF ;
			StAdjPar.StHalAdj.UsHlxOff	= 0xFFFF ;
		} else {
			UcTneRst					= EXE_HYADJ ;
			StAdjPar.StHalAdj.UsHlyGan	= 0xFFFF ;
			StAdjPar.StHalAdj.UsHlyOff	= 0xFFFF ;
		}
	} else {
		UcTneRst	= EXE_END ;
	}

	return( UcTneRst ) ;
}



//********************************************************************************
// Function Name 	: TneBia
// Retun Value		: Hall Top & Bottom Gaps
// Argment Value	: Hall Top & Bottom Gaps , X,Y Direction
// Explanation		: Hall Bias Tuning Function
// History			: First edition 						2009.12.1 YS.Kim
//********************************************************************************
unsigned long	TneBia( UnDwdVal	StTneVal, unsigned char	UcTneAxs )
{
	long					SlSetBia ;
	unsigned short			UsSetBia ;
	unsigned char			UcChkFst ;
	static unsigned short	UsTneVax ;							// Variable For 1/2 Searching

	UcChkFst	= 1 ;

	if ( UsStpSiz == 1) {
		UsTneVax	= 2 ;

		if ( ( StTneVal.StDwdVal.UsHigVal + StTneVal.StDwdVal.UsLowVal ) / 2 < BIAS_ADJ_BORDER ) {
			UcChkFst	= 0 ;
		}

		if( ( UcTneAxs & 0xF0 ) &&	UcChkFst )
		{
			;
		}else{
		
			if ( !UcTneAxs ) {										// Initializing Hall Offset & Bias, Step Size
				RamWriteA( DAHLXB, 0x8001 ) ; 				// 0x1115	Hall X Bias 0x8001
				RamWriteA( DAHLXO, 0x0000 ) ;				// 0x1114	Hall X Offset 0x0000
				UsStpSiz	= BIAS_LIMIT / UsTneVax ;
			} else {
				RamWriteA( DAHLYB, 0x8001 ) ; 				// 0x1117	Hall Y Bias 0x8001
				RamWriteA( DAHLYO, 0x0000 ) ;				// 011116	 Y Offset 0x0000
				UsStpSiz	= BIAS_LIMIT / UsTneVax ;
			}
		}
	}

//	if ( !UcTneAxs ) {
	if ( !( UcTneAxs & 0x0F ) ) {
		RamReadA( DAHLXB, &UsSetBia ) ;					// 0x1115	Hall X Bias Read
		SlSetBia	= ( long )UsSetBia ;
	} else {
		RamReadA( DAHLYB, &UsSetBia ) ;					// 0x1117	Hall Y Bias Read
		SlSetBia	= ( long )UsSetBia ;
	}

	if( SlSetBia > 0x00008000 ) {
		SlSetBia	|= 0xFFFF0000 ;
	}

	if( UcChkFst ) {
		if( UcTneAxs & 0xF0 )
		{

			if ( ( StTneVal.StDwdVal.UsHigVal + StTneVal.StDwdVal.UsLowVal ) / 2 > BIAS_ADJ_BORDER ) {	// Calculatiton For Hall BIAS 1/2 Searching
				SlSetBia	+= 0x0100 ;
			} else {
				SlSetBia	-= 0x0100 ;
			}
			UsStpSiz	= 0x0200 ;
			
		}else{
		
			if ( ( StTneVal.StDwdVal.UsHigVal + StTneVal.StDwdVal.UsLowVal ) / 2 > BIAS_ADJ_BORDER ) {	// Calculatiton For Hall BIAS 1/2 Searching
				SlSetBia	+= UsStpSiz ;
			} else {
				SlSetBia	-= UsStpSiz ;
			}

			UsTneVax	= UsTneVax * 2 ;
			UsStpSiz	= BIAS_LIMIT / UsTneVax ;
			
		}
	}

	if( SlSetBia > ( long )0x00007FFF ) {
		SlSetBia	= 0x00007FFF ;
	} else if( SlSetBia < ( long )0xFFFF8001 ) {
		SlSetBia	= 0xFFFF8001 ;
	}

//	if ( !UcTneAxs ) {
	if ( !( UcTneAxs & 0x0F ) ) {
		RamWriteA( DAHLXB, SlSetBia ) ;		// 0x1115	Hall X Bias Ram Write
	} else {
		RamWriteA( DAHLYB, SlSetBia ) ;		// 0x1116	Hall Y Bias Ram Write
	}

//	StTneVal.UlDwdVal	= TnePtp( UcTneAxs, PTP_AFTER ) ;
	StTneVal.UlDwdVal	= TnePtp( UcTneAxs & 0x0F , PTP_AFTER ) ;

	return( StTneVal.UlDwdVal ) ;
}



//********************************************************************************
// Function Name 	: TneOff
// Retun Value		: Hall Top & Bottom Gaps
// Argment Value	: Hall Top & Bottom Gaps , X,Y Direction
// Explanation		: Hall Offset Tuning Function
// History			: First edition 						2009.12.1 YS.Kim
//********************************************************************************
unsigned long	TneOff( UnDwdVal	StTneVal, unsigned char	UcTneAxs )
{
	long			SlSetOff ;
	unsigned short	UsSetOff ;

	/* TEST */
	UcTneAxs &= 0x0F ;
	/****** */
	
	if ( !UcTneAxs ) {																			// Initializing Hall Offset & Bias
		RamReadA( DAHLXO, &UsSetOff ) ;															// 0x1114	Hall X Offset Read
		SlSetOff	= ( long )UsSetOff ;
	} else {
		RamReadA( DAHLYO, &UsSetOff ) ;															// 0x1116	Hall Y Offset Read
		SlSetOff	= ( long )UsSetOff ;
	}

	if( SlSetOff > 0x00008000 ) {
		SlSetOff	|= 0xFFFF0000 ;
	}

	if ( StTneVal.StDwdVal.UsHigVal > StTneVal.StDwdVal.UsLowVal ) {
		SlSetOff	+= ( StTneVal.StDwdVal.UsHigVal - StTneVal.StDwdVal.UsLowVal ) / OFFSET_DIV ;	// Calculating Value For Increase Step
	} else {
		SlSetOff	-= ( StTneVal.StDwdVal.UsLowVal - StTneVal.StDwdVal.UsHigVal ) / OFFSET_DIV ;	// Calculating Value For Decrease Step
	}

	if( SlSetOff > ( long )0x00007FFF ) {
		SlSetOff	= 0x00007FFF ;
	} else if( SlSetOff < ( long )0xFFFF8001 ) {
		SlSetOff	= 0xFFFF8001 ;
	}

	if ( !UcTneAxs ) {
		RamWriteA( DAHLXO, SlSetOff ) ;		// 0x1114	Hall X Offset Ram Write
	} else {
		RamWriteA( DAHLYO, SlSetOff ) ;		// 0x1116	Hall Y Offset Ram Write
	}

	StTneVal.UlDwdVal	= TnePtp( UcTneAxs, PTP_AFTER ) ;

	return( StTneVal.UlDwdVal ) ;
}

 #endif
#endif
//********************************************************************************
// Function Name 	: MesFil
// Retun Value		: NON
// Argment Value	: Measure Filter Mode
// Explanation		: Measure Filter Setting Function
// History			: First edition 						2009.07.31	Y.Tashita
//********************************************************************************
void	MesFil( unsigned char	UcMesMod )
{
	if( !UcMesMod ) {								// Hall Bias&Offset Adjust
		// Measure Filter1 Setting
		RamWriteA( ms1aa, 0x0285 ) ;		// 0x13AF	LPF150Hz
		RamWriteA( ms1ab, 0x0285 ) ;		// 0x13B0
		RamWriteA( ms1ac, 0x7AF5 ) ;		// 0x13B1
		RamWriteA( ms1ad, 0x0000 ) ;		// 0x13B2
		RamWriteA( ms1ae, 0x0000 ) ;		// 0x13B3
		RamWriteA( ms1ba, 0x7FFF ) ;		// 0x13B4	Through
		RamWriteA( ms1bb, 0x0000 ) ;		// 0x13B5
		RamWriteA( ms1bc, 0x0000 ) ;		// 0x13B6
		RamWriteA( ms1bd, 0x0000 ) ;		// 0x13B7
		RamWriteA( ms1be, 0x0000 ) ;		// 0x13B8

		RegWriteA( MSF1SOF, 0x00 ) ;		// 0x00C1
		
		// Measure Filter2 Setting
		RamWriteA( ms2aa, 0x0285 ) ;		// 0x13B9	LPF150Hz
		RamWriteA( ms2ab, 0x0285 ) ;		// 0x13BA
		RamWriteA( ms2ac, 0x7AF5 ) ;		// 0x13BB
		RamWriteA( ms2ad, 0x0000 ) ;		// 0x13BC
		RamWriteA( ms2ae, 0x0000 ) ;		// 0x13BD
		RamWriteA( ms2ba, 0x7FFF ) ;		// 0x13BE	Through
		RamWriteA( ms2bb, 0x0000 ) ;		// 0x13BF
		RamWriteA( ms2bc, 0x0000 ) ;		// 0x13C0
		RamWriteA( ms2bd, 0x0000 ) ;		// 0x13C1
		RamWriteA( ms2be, 0x0000 ) ;		// 0x13C2

		RegWriteA( MSF2SOF, 0x00 ) ;		// 0x00C5
		
	} else if( UcMesMod == LOOPGAIN ) {				// Loop Gain Adjust
		// Measure Filter1 Setting
		RamWriteA( ms1aa, 0x0F21 ) ;		// 0x13AF	LPF1000Hz
		RamWriteA( ms1ab, 0x0F21 ) ;		// 0x13B0
		RamWriteA( ms1ac, 0x61BD ) ;		// 0x13B1
		RamWriteA( ms1ad, 0x0000 ) ;		// 0x13B2
		RamWriteA( ms1ae, 0x0000 ) ;		// 0x13B3
		RamWriteA( ms1ba, 0x7F7D ) ;		// 0x13B4	HPF30Hz
		RamWriteA( ms1bb, 0x8083 ) ;		// 0x13B5
		RamWriteA( ms1bc, 0x7EF9 ) ;		// 0x13B6
		RamWriteA( ms1bd, 0x0000 ) ;		// 0x13B7
		RamWriteA( ms1be, 0x0000 ) ;		// 0x13B8

		RegWriteA( MSF1SOF, 0x00 ) ;		// 0x00C1
		
		// Measure Filter2 Setting
		RamWriteA( ms2aa, 0x0F21 ) ;		// 0x13B9	LPF1000Hz
		RamWriteA( ms2ab, 0x0F21 ) ;		// 0x13BA
		RamWriteA( ms2ac, 0x61BD ) ;		// 0x13BB
		RamWriteA( ms2ad, 0x0000 ) ;		// 0x13BC
		RamWriteA( ms2ae, 0x0000 ) ;		// 0x13BD
		RamWriteA( ms2ba, 0x7F7D ) ;		// 0x13BE	HPF30Hz
		RamWriteA( ms2bb, 0x8083 ) ;		// 0x13BF
		RamWriteA( ms2bc, 0x7EF9 ) ;		// 0x13C0
		RamWriteA( ms2bd, 0x0000 ) ;		// 0x13C1
		RamWriteA( ms2be, 0x0000 ) ;		// 0x13C2

		RegWriteA( MSF2SOF, 0x00 ) ;		// 0x00C5
		
	} else if( UcMesMod == THROUGH ) {				// for Through
		// Measure Filter1 Setting
		RamWriteA( ms1aa, 0x7FFF ) ;		// 0x13AF	Through
		RamWriteA( ms1ab, 0x0000 ) ;		// 0x13B0
		RamWriteA( ms1ac, 0x0000 ) ;		// 0x13B1
		RamWriteA( ms1ad, 0x0000 ) ;		// 0x13B2
		RamWriteA( ms1ae, 0x0000 ) ;		// 0x13B3
		RamWriteA( ms1ba, 0x7FFF ) ;		// 0x13B4	Through
		RamWriteA( ms1bb, 0x0000 ) ;		// 0x13B5
		RamWriteA( ms1bc, 0x0000 ) ;		// 0x13B6
		RamWriteA( ms1bd, 0x0000 ) ;		// 0x13B7
		RamWriteA( ms1be, 0x0000 ) ;		// 0x13B8

		RegWriteA( MSF1SOF, 0x00 ) ;		// 0x00C1
		
		// Measure Filter2 Setting
		RamWriteA( ms2aa, 0x7FFF ) ;		// 0x13B9	Through
		RamWriteA( ms2ab, 0x0000 ) ;		// 0x13BA
		RamWriteA( ms2ac, 0x0000 ) ;		// 0x13BB
		RamWriteA( ms2ad, 0x0000 ) ;		// 0x13BC
		RamWriteA( ms2ae, 0x0000 ) ;		// 0x13BD
		RamWriteA( ms2ba, 0x7FFF ) ;		// 0x13BE	Through
		RamWriteA( ms2bb, 0x0000 ) ;		// 0x13BF
		RamWriteA( ms2bc, 0x0000 ) ;		// 0x13C0
		RamWriteA( ms2bd, 0x0000 ) ;		// 0x13C1
		RamWriteA( ms2be, 0x0000 ) ;		// 0x13C2

		RegWriteA( MSF2SOF, 0x00 ) ;		// 0x00C5
		
	} else if( UcMesMod == NOISE ) {				// SINE WAVE TEST for NOISE
		// Measure Filter1 Setting		1/2 sampling
		RamWriteA( ms1aa, 0x0303 ) ;		// 0x13AF	LPF90Hz
		RamWriteA( ms1ab, 0x0303 ) ;		// 0x13B0
		RamWriteA( ms1ac, 0x79F8 ) ;		// 0x13B1
		RamWriteA( ms1ad, 0x0000 ) ;		// 0x13B2
		RamWriteA( ms1ae, 0x0000 ) ;		// 0x13B3
		RamWriteA( ms1ba, 0x0303 ) ;		// 0x13B4	LPF90Hz
		RamWriteA( ms1bb, 0x0303 ) ;		// 0x13B5
		RamWriteA( ms1bc, 0x79F8 ) ;		// 0x13B6
		RamWriteA( ms1bd, 0x0000 ) ;		// 0x13B7
		RamWriteA( ms1be, 0x0000 ) ;		// 0x13B8

		RegWriteA( MSF1SOF, 0x00 ) ;		// 0x00C1
	
		// Measure Filter2 Setting		1/2 sampling
		RamWriteA( ms2aa, 0x0303 ) ;		// 0x13B9	LPF90Hz
		RamWriteA( ms2ab, 0x0303 ) ;		// 0x13BA
		RamWriteA( ms2ac, 0x79F8 ) ;		// 0x13BB
		RamWriteA( ms2ad, 0x0000 ) ;		// 0x13BC
		RamWriteA( ms2ae, 0x0000 ) ;		// 0x13BD
		RamWriteA( ms2ba, 0x0303 ) ;		// 0x13BE	LPF90Hz
		RamWriteA( ms2bb, 0x0303 ) ;		// 0x13BF
		RamWriteA( ms2bc, 0x79F8 ) ;		// 0x13C0
		RamWriteA( ms2bd, 0x0000 ) ;		// 0x13C1
		RamWriteA( ms2be, 0x0000 ) ;		// 0x13C2

		RegWriteA( MSF2SOF, 0x00 ) ;		// 0x00C5
		
	}
}



//********************************************************************************
// Function Name 	: SrvCon
// Retun Value		: NON
// Argment Value	: X or Y Select, Servo ON/OFF
// Explanation		: Servo ON,OFF Function
// History			: First edition 						2009.07.31 Y.Tashita
//********************************************************************************
void	SrvCon( unsigned char	UcDirSel, unsigned char	UcSwcCon )
{
	if( UcSwcCon ) {
		if( !UcDirSel ) {									// X Direction
			RegWriteA( LXEQEN, 0xC5 ) ;				// 0x0084	LXSW = ON
			RamWriteA( lxggf, 0x0000 ) ;			// 0x1308
		} else {											// Y Direction
			RegWriteA( LYEQEN, 0xC5 ) ;				// 0x008E	LYSW = ON
			RamWriteA( lyggf, 0x0000 ) ;			// 0x1348
		}
	} else {
		if( !UcDirSel ) {									// X Direction
			RegWriteA( LXEQEN, 0x45 ) ;				// 0x0084	LXSW = OFF
			RamWriteA( LXDODAT, 0x0000 ) ;			// 0x115A
		} else {											// Y Direction
			RegWriteA( LYEQEN, 0x45 ) ;				// 0x008E	LYSW = OFF
			RamWriteA( LYDODAT, 0x0000 ) ;			// 0x119A
		}
	}
}



#ifdef		MODULE_CALIBRATION		/* calibration code for Module maker */
//********************************************************************************
// Function Name 	: LopGan
// Retun Value		: Execute Result
// Argment Value	: X,Y Direction
// Explanation		: Loop Gain Adjust Function
// History			: First edition 						2009.07.31 Y.Tashita
//********************************************************************************
unsigned char	LopGan( unsigned char	UcDirSel )
{
	unsigned char	UcLpAdjSts ;
	
 #ifdef	HALLADJ_HW
	UcLpAdjSts	= LoopGainAdj( UcDirSel ) ;
 #else
	MesFil( LOOPGAIN ) ;

	// Servo ON
	SrvCon( X_DIR, ON ) ;
	SrvCon( Y_DIR, ON ) ;

	// Wait 300ms
	WitTim( 300 ) ;

	// Loop Gain Adjust Initialize
	LopIni( UcDirSel ) ;

	// Loop Gain Adjust
	UcLpAdjSts	= LopAdj( UcDirSel ) ;
 #endif
	// Servo OFF
//	SrvCon( X_DIR, OFF ) ;
//	SrvCon( Y_DIR, OFF ) ;

	if( !UcLpAdjSts ) {
		return( EXE_END ) ;
	} else {
		if( !UcDirSel ) {
			return( EXE_LXADJ ) ;
		} else {
			return( EXE_LYADJ ) ;
		}
	}
}



 #ifndef	HALLADJ_HW
//********************************************************************************
// Function Name 	: LopIni
// Retun Value		: NON
// Argment Value	: X,Y Direction
// Explanation		: Loop Gain Adjust Initialize Function
// History			: First edition 						2009.07.31 Y.Tashita
//********************************************************************************
void	LopIni( unsigned char	UcDirSel )
{
	// Loop Gain Value Initialize
	LopPar( UcDirSel ) ;

	// Sign Wave Output Setting
	LopSin( UcDirSel, ON ) ;

}
 #endif


//********************************************************************************
// Function Name 	: LopPar
// Retun Value		: NON
// Argment Value	: X,Y Direction
// Explanation		: Loop Gain Adjust Parameter Initialize Function
// History			: First edition 						2009.07.31 Y.Tashita
//********************************************************************************
void	LopPar( unsigned char	UcDirSel )
{
	unsigned short	UsLopGan ;

	if( !UcDirSel ) {
		UsLopGan	= LXGAIN_LOP ;
		RamWriteA( lxgain, UsLopGan ) ;			/* 0x132A */
	} else {
		UsLopGan	= LYGAIN_LOP ;
		RamWriteA( lygain, UsLopGan ) ;			/* 0x136A */
	}
}



 #ifndef	HALLADJ_HW
//********************************************************************************
// Function Name 	: LopSin
// Retun Value		: NON
// Argment Value	: X,Y Direction, ON/OFF Switch
// Explanation		: Loop Gain Adjust Sign Wave Initialize Function
// History			: First edition 						2009.07.31 Y.Tashita
//********************************************************************************
void	LopSin( unsigned char	UcDirSel, unsigned char	UcSonOff )
{
	if( UcSonOff ) {
		RegWriteA( SWFC1, 0x36 ) ;								// 0x00DD		139.5Hz

		if( !UcDirSel ) {
			RegWriteA( SINXADD, 0x00 ) ;			/* 0x00E3	---- */
			RegWriteA( SINYADD, 0x00 ) ;			/* 0x00E4	---- */
			
			RegWriteA( SWSEL, 0x80 ) ;							// 0x00E2	Sin Wave LXDX Write
			RegWriteA( SWEN, 0xC0 ) ;							// 0x00DB	Sine wave , Reset output
			RegWriteA( SWFC2, 0x48 ) ;							// 0x00DE	Sine wave , Left round , SWCOM on
			if( UcPwmMod == PWMMOD_CVL ) {
				RamWriteA( lxxg, 0x198A ) ;							// 0x132B		-14dB
			}else{
				RamWriteA( lxxg, 0x32F5 ) ;							// 0x132B		-8dB
			}
		} else {
			RegWriteA( SINXADD, 0x00 ) ;			/* 0x00E3	---- */
			RegWriteA( SINYADD, 0x00 ) ;			/* 0x00E4	---- */
			
			RegWriteA( SWSEL, 0x40 ) ;							// 0x00E2	Sin Wave LYDX Write
			RegWriteA( SWEN, 0xC0 ) ;							// 0x00DB	Sine wave , Reset output
			RegWriteA( SWFC2, 0x48 ) ;							// 0x00DE	Sine wave , Left round , SWCOM on
			if( UcPwmMod == PWMMOD_CVL ) {
				RamWriteA( lyxg, 0x198A ) ;							// 0x136B		-14dB
			}else{
				RamWriteA( lyxg, 0x32F5 ) ;							// 0x136B		-8dB
			}
		}
	} else {
		RegWriteA( SWEN, 0x00 ) ;								// 0x00DB		Sin Wave Generate OFF
		if( !UcDirSel ) {
			RamWriteA( lxxg, 0x0000 ) ;							// 0x132B
		} else {
					RamWriteA( lyxg, 0x0000 ) ;					// 0x136B
		}
	}
}



//********************************************************************************
// Function Name 	: LopAdj
// Retun Value		: Command Status
// Argment Value	: X,Y Direction
// Explanation		: Loop Gain Adjust Function
// History			: First edition 						2009.07.31 Y.Tashita
//********************************************************************************
unsigned char	LopAdj( unsigned char	UcDirSel )
{
	unsigned char	UcAdjSts	= FAILURE ;
	unsigned long	UlAdcXg1 ;
	unsigned long	UlAdcXg2 ;
	unsigned short 	UsRtnVal ;
//HTC_START  20130329
#if 0
	float			SfCmpVal ;
#else
	long			SfCmpVal;
#endif
//HTC_END
	unsigned char	UcIdxCnt ;
	unsigned char	UcIdxCn1 ;
	unsigned char	UcIdxCn2 ;

	unsigned short	UsGanVal[ 5 ] ;
	unsigned short	UsTemVal ;

	if( !UcDirSel ) {
		RegWriteA( MS1INADD, 0x46 ) ;											// 0x00C2	LXG1 Input
		RegWriteA( MS2INADD, 0x47 ) ;											// 0x00C6	LXG2 Input
	} else {
		RegWriteA( MS1INADD, 0x86 ) ;											// 0x00C2	LYG1 Input
		RegWriteA( MS2INADD, 0x87 ) ;											// 0x00C6	LYG2 Input
	}

	// 5 Times Average Value Calculation
	for( UcIdxCnt = 0 ; UcIdxCnt < 5 ; UcIdxCnt++ )
	{
		LopMes( ) ;																// Loop Gain Mesurement Start

		UlAdcXg1	= GinMes( MES_XG1 ) ;										// LXG1 Measure
		UlAdcXg2	= GinMes( MES_XG2 ) ;										// LXG2 Measure

//HTC_START  20130329
#if 0
		SfCmpVal	= ( float )UlAdcXg2 / ( float )UlAdcXg1 ;					// Compare Coefficient Value
#else
		SfCmpVal	= (UlAdcXg2*1000) / UlAdcXg1 ;					// Compare Coefficient Value
#endif
//HTC_END

		if( !UcDirSel ) {
			RamReadA( lxgain, &UsRtnVal ) ;									// 0x132A
		} else {
			RamReadA( lygain, &UsRtnVal ) ;									// 0x136A
		}

//HTC_START  20130329
#if 0
		UsRtnVal	= ( unsigned short )( ( float )UsRtnVal * SfCmpVal ) ;
#else
		UsRtnVal	= ( unsigned short )( (UsRtnVal * SfCmpVal) / 1000) ;
#endif
//HTC_END

		UsGanVal[ UcIdxCnt ]	= UsRtnVal ;
	}

	for( UcIdxCn1 = 0 ; UcIdxCn1 < 4 ; UcIdxCn1++ )
	{
		for( UcIdxCn2 = UcIdxCn1+1 ; UcIdxCn2 < 5 ; UcIdxCn2++ )
		{
			if( UsGanVal[ UcIdxCn1 ] > UsGanVal[ UcIdxCn2 ] ) {
				UsTemVal				= UsGanVal[ UcIdxCn1 ] ;
				UsGanVal[ UcIdxCn1 ]	= UsGanVal[ UcIdxCn2 ] ;
				UsGanVal[ UcIdxCn2 ]	= UsTemVal ;
			}
		}
	}

	UsRtnVal	= ( unsigned short )( ( ( long )UsGanVal[ 1 ] + ( long )UsGanVal[ 2 ] + ( long )UsGanVal[ 3 ] ) / 3 ) ;

	LopSin( UcDirSel, OFF ) ;

	if( UsRtnVal < 0x8000 ) {												// Adjust Error
		UcAdjSts	= SUCCESS ;												// Status OK
	}

	if( UcAdjSts ) {
		if( !UcDirSel ) {
			RamWriteA( lxgain, 0x7FFF ) ;							// 0x132A
			StAdjPar.StLopGan.UsLxgVal	= 0x7FFF ;
			StAdjPar.StLopGan.UsLxgSts	= 0x0000 ;
		} else {
			RamWriteA( lygain, 0x7FFF ) ;							// 0x136A
			StAdjPar.StLopGan.UsLygVal	= 0x7FFF ;
			StAdjPar.StLopGan.UsLygSts	= 0x0000 ;
		}
	} else {
		if( !UcDirSel ) {
			RamWriteA( lxgain, UsRtnVal ) ;							// 0x132A
			StAdjPar.StLopGan.UsLxgVal	= UsRtnVal ;
			StAdjPar.StLopGan.UsLxgSts	= 0xFFFF ;
		} else {
			RamWriteA( lygain, UsRtnVal ) ;							// 0x136A
			StAdjPar.StLopGan.UsLygVal	= UsRtnVal ;
			StAdjPar.StLopGan.UsLygSts	= 0xFFFF ;
		}
	}
	return( UcAdjSts ) ;
}


//********************************************************************************
// Function Name 	: LopMes
// Retun Value		: void
// Argment Value	: void
// Explanation		: Loop Gain Adjust Measure Setting
// History			: First edition 						2009.07.31 Y.Tashita
//********************************************************************************
void	LopMes( void )
{
	BsyWit( DLYCLR2, 0xC0 ) ;							// 0x00EF	Measure Filter1,2 Delay RAM Clear
	RegWriteA( MSMPLNSH, 0x03 ) ;						// 0x00CB
	RegWriteA( MSMPLNSL, 0xFF ) ;						// 0x00CA	1024 Times Measure
	RegWriteA( MSF1EN, 0x01 ) ;							// 0x00C0	Measure Filter1 Equalizer ON
	RegWriteA( MSF2EN, 0x01 ) ;							// 0x00C4	Measure Filter2 Equalizer ON
	BsyWit( MSMA, 0xE1 ) ;								// 0x00C9	Cycle Wait, Sin Wave Measure

	RegWriteA( MSMA, 0x00 ) ;							// 0x00C9	Measure End
	RegWriteA( MSF1EN, 0x00 ) ;							// 0x00C0	Measure Filter1 Equalizer OFF
	RegWriteA( MSF2EN, 0x00 ) ;							// 0x00C4	Measure Filter2 Equalizer OFF
}


//********************************************************************************
// Function Name 	: GinMes
// Retun Value		: Measure Result
// Argment Value	: LXG1/LXG2 Select
// Explanation		: Measure Result Read
// History			: First edition 						2009.11.16 Y.Tashita
//********************************************************************************
unsigned long	GinMes( unsigned char	UcXg1Xg2 )
{
	unsigned short	UsMesVll, UsMesVlh ;
	unsigned long	UlMesVal ;

	if( !UcXg1Xg2 ) {
		RamReadA( MSAVAC1L, &UsMesVll ) ;			// 0x1203
		RamReadA( MSAVAC1H, &UsMesVlh ) ;			// 0x1204
	} else {
		RamReadA( MSAVAC2L, &UsMesVll ) ;			// 0x1200
		RamReadA( MSAVAC2H, &UsMesVlh ) ;			// 0x1201
	}

	UlMesVal	= ( unsigned long )( ( ( unsigned long )UsMesVlh << 16 ) | UsMesVll ) ;

	return( UlMesVal ) ;
}

 #endif


//********************************************************************************
// Function Name 	: TneGvc
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Tunes the Gyro VC offset
// History			: First edition 						2009.07.31	Y.Tashita
//********************************************************************************
#define	LIMITH		0x0FA0
#define	LIMITL		0xF060
#define	INITVAL		0x0000

unsigned char	TneGvc( void )
{
	unsigned char  UcRsltSts;
	
	
	// for InvenSense Digital Gyro	ex.IDG-2000
	// A/D Offset Clear
	RamWriteA( ADGXOFF, 0x0000 ) ;	// 0x1108
	RamWriteA( ADGYOFF, 0x0000 ) ;	// 0x110B
	RegWriteA( IZAH,	(unsigned char)(INITVAL >> 8) ) ;	// 0x03A0		Set Offset High byte
	RegWriteA( IZAL,	(unsigned char)INITVAL ) ;			// 0x03A1		Set Offset Low byte
	RegWriteA( IZBH,	(unsigned char)(INITVAL >> 8) ) ;	// 0x03A2		Set Offset High byte
	RegWriteA( IZBL,	(unsigned char)INITVAL ) ;			// 0x03A3		Set Offset Low byte
	
	RegWriteA( GDLYMON10, 0xF5 ) ;		// 0x0184 <- GXADZ(0x19F5)
	RegWriteA( GDLYMON11, 0x01 ) ;		// 0x0185 <- GXADZ(0x19F5)
	RegWriteA( GDLYMON20, 0xF5 ) ;		// 0x0186 <- GYADZ(0x18F5)
//	RegWriteA( GDLYMON21, 0x01 ) ;		// 0x0187 <- GYADZ(0x18F5)
	RegWriteA( GDLYMON21, 0x00 ) ;		// 0x0187 <- GYADZ(0x18F5)
	MesFil( THROUGH ) ;				// ªèptB^[ðÝè·éB
	RegWriteA( MSF1EN, 0x01 ) ;			// 0x00C0		Measure Filter1 Equalizer ON
	//////////
	// X
	//////////
	BsyWit( DLYCLR2, 0x80 ) ;			// 0x00EF	Measure Filter1 Delay RAM Clear
	StAdjPar.StGvcOff.UsGxoVal = (unsigned short)GenMes( GYRMON1, 0 );		// 64ñÌ½Ïlªè	GYRMON1(0x1110) <- GXADZ(0x19F5)
	RegWriteA( IZAH, (unsigned char)(StAdjPar.StGvcOff.UsGxoVal >> 8) ) ;	// 0x03A0		Set Offset High byte
	RegWriteA( IZAL, (unsigned char)(StAdjPar.StGvcOff.UsGxoVal) ) ;		// 0x03A1		Set Offset Low byte
	//////////
	// Y
	//////////
	BsyWit( DLYCLR2, 0x80 ) ;			// 0x00EF	Measure Filter1 Delay RAM Clear
	StAdjPar.StGvcOff.UsGyoVal = (unsigned short)GenMes( GYRMON2, 0 );		// 64ñÌ½Ïlªè	GYRMON2(0x1111) <- GYADZ(0x18F5)
	RegWriteA( IZBH, (unsigned char)(StAdjPar.StGvcOff.UsGyoVal >> 8) ) ;	// 0x03A2		Set Offset High byte
	RegWriteA( IZBL, (unsigned char)(StAdjPar.StGvcOff.UsGyoVal) ) ;		// 0x03A3		Set Offset Low byte
	
	RegWriteA( MSF1EN, 0x00 ) ;			// 0x00C0		Measure Filter1 Equalizer OFF
	
	UcRsltSts = EXE_END ;						/* Clear Status */

	StAdjPar.StGvcOff.UsGxoSts	= 0xFFFF ;
	if(( (short)StAdjPar.StGvcOff.UsGxoVal < (short)LIMITL ) || ( (short)StAdjPar.StGvcOff.UsGxoVal > (short)LIMITH ))
	{
		UcRsltSts |= EXE_GXADJ ;
		StAdjPar.StGvcOff.UsGxoSts	= 0x0000 ;
	}
	
	StAdjPar.StGvcOff.UsGyoSts	= 0xFFFF ;
	if(( (short)StAdjPar.StGvcOff.UsGyoVal < (short)LIMITL ) || ( (short)StAdjPar.StGvcOff.UsGyoVal > (short)LIMITH ))
	{
		UcRsltSts |= EXE_GYADJ ;
		StAdjPar.StGvcOff.UsGyoSts	= 0x0000 ;
	}
	return( UcRsltSts );
		
}

#endif

//********************************************************************************
// Function Name 	: RtnCen
// Retun Value		: Command Status
// Argment Value	: Command Parameter
// Explanation		: Return to center Command Function
// History			: First edition 						2009.08.04 Y.Tashita
//********************************************************************************
unsigned char	RtnCen( unsigned char	UcCmdPar )
{
	unsigned char	UcCmdSts ;

	UcCmdSts	= EXE_END ;

	GyrCon( OFF ) ;											// Gyro OFF

	if( !UcCmdPar ) {										// X,Y Centering

		StbOnn() ;											// Slope Mode
		
	} else if( UcCmdPar == 0x01 ) {							// X Centering Only

		SrvCon( X_DIR, ON ) ;								// X only Servo ON
		SrvCon( Y_DIR, OFF ) ;
	} else if( UcCmdPar == 0x02 ) {							// Y Centering Only

		SrvCon( X_DIR, OFF ) ;								// Y only Servo ON
		SrvCon( Y_DIR, ON ) ;
	}

	return( UcCmdSts ) ;
}



//********************************************************************************
// Function Name 	: GyrCon
// Retun Value		: NON
// Argment Value	: Gyro Filter ON or OFF
// Explanation		: Gyro Filter Control Function
// History			: First edition 						2009.08.04 Y.Tashita
//********************************************************************************
void	GyrCon( unsigned char	UcGyrCon )
{

	// Return HPF Setting
	RegWriteA( GSHTON, 0x00 ) ;									// 0x0104

	if( UcGyrCon ) {													// Gyro ON

		ClrGyr( 0x02 , CLR_GYR_DLY_RAM );			/* Gyro Dlay Ram Clear */
		
		RamWriteA( lxggf, 0x7fff ) ;	// 0x1308
		RamWriteA( lyggf, 0x7fff ) ;	// 0x1348
		
#ifdef	GAIN_CONT
		/* Gain3 Register */
		AutoGainControlSw( ON ) ;											/* Auto Gain Control Mode ON */
#endif
	} else {															// Gyro OFF

#ifdef	GAIN_CONT
		/* Gain3 Register */
		AutoGainControlSw( OFF ) ;											/* Auto Gain Control Mode OFF */
#endif
	}
}



//********************************************************************************
// Function Name 	: OisEna
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: OIS Enable Control Function
// History			: First edition 						2009.08.07 Y.Tashita
//********************************************************************************
void	OisEna( void )
{
	// Servo ON
	SrvCon( X_DIR, ON ) ;
	SrvCon( Y_DIR, ON ) ;

	GyrCon( ON ) ;
}



//********************************************************************************
// Function Name 	: TimPro
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Timer Interrupt Process Function
// History			: First edition 						2009.08.07 Y.Tashita
//********************************************************************************
void	TimPro( void )
{
	if( UcOscAdjFlg )
	{
		if( UcOscAdjFlg == MEASSTR )
		{
			RegWriteA( OSCCNTEN, 0x01 ) ;		// 0x0265	OSC Cnt enable
			UcOscAdjFlg = MEASCNT ;
		}
		else if( UcOscAdjFlg == MEASCNT )
		{
			RegWriteA( OSCCNTEN, 0x00 ) ;		// 0x0265	OSC Cnt disable
			UcOscAdjFlg = MEASFIX ;
		}
	}
}



//********************************************************************************
// Function Name 	: S2cPro
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: S2 Command Function
// History			: First edition 						2009.08.08 Y.Tashita
//********************************************************************************
void	S2cPro( unsigned char uc_mode )
{
	if( uc_mode == 1 )
	{
#ifdef H1COEF_CHANGER
		SetH1cMod( S2MODE ) ;							/* cancel Lvl change */
#endif
//		RegWriteA( GLMT3SEL, 0x01 ) ;											// 0x0117
		RegWriteA( G2NDCEFON0, 0x04 ) ;											// 0x0106 2ND H1 use
		// HPF¨Through Setting
		RegWriteA( GSHTON, 0x11 ) ;												// 0x0104

	}
	else
	{
		// HPF¨Through Setting
		RegWriteA( GSHTON, 0x00 ) ;												// 0x0104
		RegWriteA( G2NDCEFON0, 0x00 ) ;											// 0x0106 1st H1 use
//		RegWriteA( GLMT3SEL, 0x00 ) ;											// 0x0117
#ifdef H1COEF_CHANGER
		SetH1cMod( UcH1LvlMod ) ;							/* Re-setting */
#endif

	}
	
}


//********************************************************************************
// Function Name 	: GenMes
// Retun Value		: A/D Convert Result
// Argment Value	: Measure Filter Input Signal Ram Address
// Explanation		: General Measure Function
// History			: First edition 						2008.7.8 Y.Tashita
//********************************************************************************
short	GenMes( unsigned short	UsRamAdd, unsigned char	UcMesMod )
{
	short	SsMesRlt ;

	RegWriteA( MS1INADD, ( unsigned char )( UsRamAdd & 0x00ff ) ) ;	// 0x00C2	Input Signal Select

	if( !UcMesMod ) {
		RegWriteA( MSMPLNSH, 0x00 ) ;									// 0x00CB
		RegWriteA( MSMPLNSL, 0x3F ) ;									// 0x00CA	64 Times Measure
	} else {
		RegWriteA( MSMPLNSH, 0x00 ) ;									// 0x00CB
		RegWriteA( MSMPLNSL, 0x00 ) ;									// 0x00CA	1 Times Measure
	}

	BsyWit( MSMA, 0x01 ) ;											// 0x00C9		Average Measure

	RegWriteA( MSMA, 0x00 ) ;										// 0x00C9		Measure Stop

	RamReadA( MSAV1, ( unsigned short * )&SsMesRlt ) ;				// 0x1205

	return( SsMesRlt ) ;
}


//********************************************************************************
// Function Name 	: SetSinWavePara
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Sine wave Test Function
// History			: First edition 						2010.11.08 Y.Shigeoka
//********************************************************************************
	/********* Parameter Setting *********/
	/* Servo Sampling Clock		=	23.437kHz	*/
	/* Freq						=	23.427*(SWB+1)/(96*16*(SWA+1))	*/
	/* Servo Sampling Clock		=	11.719kHz	*/
	/* Freq						=	11.719*(SWB+1)/(96*8*(SWA+1))	*/
	/* 05 00 XX MM 				XX:Freq MM:Sin or Circle */
const unsigned char	CucFreqVal[ 17 ]	= {
		0xF0,				//	0:	Stop
		0x0E,				//	1:	1.02Hz
		0x1E,				//	2:	2.03Hz
		0x19,				//	3:	3.05Hz	
		0x3E,				//	4:	4.07Hz
		0x02,				//	5:	5.09Hz
		0x14,				//	6:	6.10Hz
		0x5C,				//	7:	7.04Hz
		0x7E,				//	8:	8.14Hz
		0x6B,				//	9:	8.90Hz
		0x12,				//	A: 10.17Hz
		0x46,				//	B: 10.90Hz
		0xAD,				//	C: 11.99Hz
		0x56,				//	D: 13.08Hz
		0xAB,				//	E: 13.99Hz
		0x00,				//	F: 15.26Hz
		0x3E				// 10:	4.07Hz
	} ;
	
/* UÍwavxg(13C3h),wavyg(13C4h)Å²® */
void	SetSinWavePara( unsigned char UcTableVal ,	unsigned char UcMethodVal )
{
	unsigned char	UcFreqDat ;
	unsigned char	UcMethod ;


	if(UcTableVal > 0x10 )
		UcTableVal = 0x10 ;			/* Limit */
	UcFreqDat = CucFreqVal[ UcTableVal ] ;	
	
	if( UcMethodVal == SINEWAVE) {
		UcMethod = 0x4C ;			/* Sine wave + SWC=4 */
	}else if( UcMethodVal == CIRCWAVE ){
		UcMethod = 0xCC ;			/* Circular movement + SWC=4 */
	}else{
		UcMethod = 0x4C ;			/* Sine wave + SWC=4 */
	}
	
	if(( UcMethodVal != XHALWAVE ) && ( UcMethodVal != YHALWAVE )) {
		MesFil( NOISE ) ;			/* LPF */
	}
	
	if( UcFreqDat == 0xF0 )			/* Sineg~ */
	{
		RegWriteA( SINXADD, 0x00 ) ;			/* 0x00E3	---- */
		RegWriteA( SINYADD, 0x00 ) ;			/* 0x00E4	---- */
		
		RegWriteA( MS1OUTADD, 0x00 ) ;			// 0x00C3	Measure Filter1 Output Cut Off
		RegWriteA( MS2OUTADD, 0x00 ) ;			// 0x00C7	Measure Filter2 Output Cut Off
		RegWriteA( SWFC1, 0x1D ) ;				/* 0x00DD	*Hz */
		RegWriteA( SWEN, 0x00 ) ;				/* 0x00DB	Sin wave off */
		RegWriteA( SWSEL, 0x00 ) ;				/* 0x00E2	SINX,SINY */
		RegWriteA( SWFC2, 0x08 ) ;				/* 0x00DE	SWCOM=1 */
		RamWriteA( LXDX, 0x0000 ) ;				/* 0x1148	*/
		RamWriteA( LYDX, 0x0000 ) ;				/* 0x1188	*/
		if(( UcMethodVal != XHALWAVE ) && ( UcMethodVal != YHALWAVE )) {
			RamWriteA( HXINOD, UsCntXof ) ;			/* 0x1127	*/
			RamWriteA( HYINOD, UsCntYof ) ;			/* 0x1167	*/
		}else{
			RamWriteA( LXDODAT, 0x0000 ) ;			/* 0x115A	*/
			RamWriteA( LYDODAT, 0x0000 ) ;			/* 0x119A	*/
		}
		RegWriteA( MSFDS, 0x00 ) ;				/* 0x00C8	1/1 down sampling */
		RegWriteA( MSF1EN, 0x00 ) ;				/* 0x00C0	ªèpFilter1ÌZOFF */
		RegWriteA( MSF2EN, 0x00 ) ;				/* 0x00C4	ªèpFilter2ÌZOFF */
	}
	else
	{
		RegWriteA( SINXADD, 0x00 ) ;			/* 0x00E3	---- */
		RegWriteA( SINYADD, 0x00 ) ;			/* 0x00E4	---- */
		
		RegWriteA( SWFC1, UcFreqDat ) ;			/* 0x00DD	*Hz */
		RegWriteA( SWFC2, UcMethod ) ;			/* 0x00DE	SWC=* */
		RegWriteA( SWSEL, 0xC0 ) ;				/* 0x00E2	SINX,SINY */
		RegWriteA( SWEN, 0x80 ) ;				/* 0x00DB	Sin wave on */
		//
		if(( UcMethodVal != XHALWAVE ) && ( UcMethodVal != YHALWAVE )) {
			RegWriteA( MS1INADD, 0x48 ) ;			/* 0x00C2	<- LXDX */
			RegWriteA( MS1OUTADD, 0x27 ) ;			/* 0x00C3	-> HXINOD */
			RegWriteA( MS2INADD, 0x88 ) ;			/* 0x00C6	<- LYDX */
			RegWriteA( MS2OUTADD, 0x67 ) ;			/* 0x00C7	-> HYINOD */
		}else{
			if( UcMethodVal == XHALWAVE ){
				RegWriteA( SINXADD , (unsigned char)LXDODAT );							// 0x00E3  [ SINXADD(7:0) ]
			}else{
				RegWriteA( SINYADD , (unsigned char)LYDODAT );							// 0x00E4  [ SINYADD(7:0) ]
			}
			RegWriteA( MSMPLNSL, 0x00 ) ;			/* 0x00CA	Measure count */
			RegWriteA( MSMPLNSH, 0x00 ) ;			/* 0x00CB	Measure count */
		}
		
		RegWriteA( MSFDS, 0x01 ) ;				/* 0x00C8	1/2 down sampling */
		RegWriteA( MSF1EN, 0x81 ) ;				/* 0x00C0	ªèpFilter1ÌZON */
		RegWriteA( MSF2EN, 0x81 ) ;				/* 0x00C4	ªèpFilter2ÌZON */
		
	}
	
	
}

#ifdef USE_EXE2PROM
//********************************************************************************
// Function Name 	: E2pRed
// Retun Value		: NON
// Argment Value	: E2PROM Address, Length, Pointer
// Explanation		: E2PROM Read Function
// History			: First edition 						2009.11.16 Y.Hayashi
//********************************************************************************
void	E2pRed( unsigned short UsAdr, unsigned char UcLen, unsigned char *UcPtr )
{
	unsigned char UcAdh = UsAdr >> 8 ;
	unsigned char UcAdl = UsAdr & 0xFF ;
	unsigned char UcCnt ;

#ifdef I2CE2PROM
	RegWriteA( BSYSEL,	0x06 ) ;		// 0x0240	Set Busy signal
	RegWriteA( E2L,		UcAdl ) ;		// 0x0281	Set E2P ROM Address low
	RegWriteA( E2H,		UcAdh ) ;		// 0x0282	Set E2P ROM Address high
	
	RegWriteA( E2ACC, UcLen );			// 0x0280	Read Start

	Bsy2Wit( ) ;						// busy2 wait

	for( UcCnt = 0; UcCnt < UcLen; UcCnt++ ) {
		RegReadA( E2DAT0 + UcCnt, &UcPtr[ UcCnt ] ) ;	// 0x0283	Start data
		
	}
#endif
#ifdef SPIE2PROM
	RegWriteA( BSYSEL,	0x07 ) ;		// 0x0240	Set Busy signal
	RegWriteA( E2L,		UcAdl ) ;		// 0x0302	Set E2P ROM Address low
	RegWriteA( E2H,		UcAdh ) ;		// 0x0303	Set E2P ROM Address high
	
	RegWriteA( E2ACCR, UcLen << 1 );	// 0x0300	Read Start

	Bsy2Wit( ) ;						// busy2 wait

	for( UcCnt = 0; UcCnt < UcLen; UcCnt++ ) {
		RegReadA( E2DAT0 + UcCnt, &UcPtr[ UcCnt ] ) ;	// 0x0304	Start data
	}
#endif

}



//********************************************************************************
// Function Name 	: E2pWrt
// Retun Value		: NON
// Argment Value	: E2PROM Address, Length, Pointer
// Explanation		: E2PROM Write Function
// History			: First edition 						2009.11.16 Y.Hayashi
//********************************************************************************
void	E2pWrt( unsigned short UsAdr, unsigned char UcLen, unsigned char *UcPtr )
{
	unsigned char UcAdh = UsAdr >> 8 ;
	unsigned char UcAdl = UsAdr & 0xFF ;
	unsigned char UcWrtFlg ;
	unsigned char UcCnt ;

#ifdef I2CE2PROM		/* page 16byte */
	RegWriteA( BSYSEL,	0x06 ) ;		// 0x0240	Set Busy signal
	RegWriteA( E2L,		UcAdl ) ;		// 0x0281	Set E2P ROM Address low
	RegWriteA( E2H,		UcAdh ) ;		// 0x0282	Set E2P ROM Address high

										// Set Data
	for( UcCnt = 0; UcCnt < UcLen; UcCnt++ ) {
		RegWriteA( E2DAT0 + UcCnt, UcPtr[ UcCnt ] ) ;	// 0x0283	Start data
	}

	UcWrtFlg = UcLen * 16 ;				// 1->16, 2->32, 4->64

	RegWriteA( E2ACC, UcWrtFlg ) ;		// 0x0280	Write Start

	Bsy2Wit( ) ;						// busy2 wait

#endif

#ifdef SPIE2PROM		/* page : 32byte */
	/* fail safe */
	if(( UsAdr & 0x1FFF) != 0x0000 )
	{
		RegWriteA( BSYSEL,	0x07 ) ;		// 0x0240	Set Busy signal
		RegWriteA( E2L,		UcAdl ) ;		// 0x0302	Set E2P ROM Address low
		RegWriteA( E2H,		UcAdh ) ;		// 0x0303	Set E2P ROM Address high

											// Set Data
		for( UcCnt = 0; UcCnt < UcLen; UcCnt++ ) {
			RegWriteA( E2DAT0 + UcCnt, UcPtr[ UcCnt ] ) ;	// 0x0304	Start data
		}

		UcWrtFlg = UcLen << 1 ;				// 1->2, 2->4, 4->8

		RegWriteA( E2ACCW, UcWrtFlg ) ;		// 0x0301	Write Start

		Bsy2Wit( ) ;						// busy2 wait
	}

#endif
}



//********************************************************************************
// Function Name 	: AdjSav
// Retun Value		: NON
// Argment Value	: X or Y Axis Select
// Explanation		: Hall Adjust Result Save Function
// History			: First edition 						2010.01.20 Y.Tashita
//********************************************************************************
void	AdjSav( unsigned char	UcAxsSel )
{
	unsigned char *	NuAdjPar ;
	unsigned short	UsEepAdd ;

//	NuAdjPar	= ( unsigned char * )&StAdjPar ;

	switch( UcAxsSel )
	{
		case X_DIR :																	// Hall X Adjust Result Write
			NuAdjPar	= ( unsigned char * )&StAdjPar.StHalAdj.UsHlxCna ;
			for( UsEepAdd = CENTER_HALL_AD_X ; UsEepAdd < CENTER_HALL_AD_Y ; UsEepAdd	+= 2 )
			{
				E2pWrt( UsEepAdd, 2, NuAdjPar ) ;
				NuAdjPar	+= 2 ;
			}
			break ;
		case Y_DIR :																	// Hall Y Adjust Result Write
			NuAdjPar	= ( unsigned char * )&StAdjPar.StHalAdj.UsHlyCna ;
			for( UsEepAdd = CENTER_HALL_AD_Y ; UsEepAdd < LOOP_GAIN_STATUS_X ; UsEepAdd	+= 2 )
			{
				E2pWrt( UsEepAdd, 2, NuAdjPar ) ;
				NuAdjPar	+= 2 ;
			}
			break ;
		default :
			break ;
	}
}
#endif


#ifdef STANDBY_MODE
//********************************************************************************
// Function Name 	: SetStandby
// Retun Value		: NON
// Argment Value	: 0:Standby ON 1:Standby OFF 2:Standby2 ON 3:Standby2 OFF 
//					: 4:Standby3 ON 5:Standby3 OFF
// Explanation		: Set Standby
// History			: First edition 						2010.12.27 Y.Shigeoka
//********************************************************************************
void	SetStandby( unsigned char UcContMode )
{
	switch(UcContMode)
	{
	case STB1_ON:
		RegWriteA( STBB, 0x00 ) ;			/* 0x0260	Y-amp,X-amp,DAC,ADC Standby				*/
		RegWriteA( PWMA, 0x00 ) ;			/* 0x0074	PWM Output Circuit OFF		*/
		RegWriteA( LNA,  0x00 ) ;			/* 0x0078	Low Noise mode standby	*/
		DrvSw( OFF ) ;						/* 0x0070	Drvier Block Ena=0 */
		RegWriteA( PWMMONFC, 0x00 ) ;		/* 0x00F4	Monitor OFF	*/
		RegWriteA( DAMONFC, 0x00 ) ;		/* 0x00F5	ExDAC OFF	*/
		SelectGySleep( ON ) ;				/* Gyro Sleep */
		break ;
	case STB1_OFF:
		SelectGySleep( OFF ) ;				/* Gyro Wake Up */
		RegWriteA( DAMONFC, 0x00 ) ;		/* 0x00F5	ExDAC ON	*/
		RegWriteA( PWMMONFC, 0x80 ) ;		/* 0x00F4	Monitor ON	*/
		DrvSw( ON ) ;						/* 0x0070		Driver Mode setting */
		RegWriteA( LNA,  0xC0 ) ;			/* 0x0078	Low Noise mode enable	*/
		RegWriteA( PWMA, 0xC0 ) ;			/* 0x0074	PWM Output Circuit OFF		*/
		RegWriteA( STBB, 0x0F ) ;			// 0x0260	ON
		break ;
	case STB2_ON:
		RegWriteA( STBB, 0x00 ) ;			/* 0x0260	Y-amp,X-amp,DAC,ADC Standby				*/
		RegWriteA( PWMA, 0x00 ) ;			/* 0x0074	PWM Output Circuit OFF		*/
		RegWriteA( LNA,  0x00 ) ;			/* 0x0078	Low Noise mode standby	*/
		DrvSw( OFF ) ;						/* 0x0070	Drvier Block Ena=0 */
		RegWriteA( PWMMONFC, 0x00 ) ;		/* 0x00F4	Monitor OFF	*/
		RegWriteA( DAMONFC, 0x00 ) ;		/* 0x00F5	ExDAC OFF	*/
		SelectGySleep( ON ) ;				/* Gyro Sleep */
		RegWriteA( CLKON, 0x00 ) ;			/* 0x020B	Servo & PWM Clock OFF + D-Gyro I/F OFF	*/
		break ;
	case STB2_OFF:
#ifdef I2CE2PROM
		RegWriteA( CLKON, 0x33 ) ;			/* 0x020B	Servo & PWM Clock ON + D-Gyro I/F ON	*/
#else
 #ifdef SPIE2PROM
		RegWriteA( CLKON, 0x17 ) ;			/* 0x020B	Servo & PWM Clock ON + D-Gyro I/F ON	*/
 #else
		RegWriteA( CLKON, 0x13 ) ;			/* 0x020B	Servo & PWM Clock ON + D-Gyro I/F ON	*/
 #endif
#endif
		SelectGySleep( OFF ) ;				/* Gyro Wake Up */
		RegWriteA( DAMONFC, 0x00 ) ;		/* 0x00F5	ExDAC ON	*/
		RegWriteA( PWMMONFC, 0x80 ) ;		/* 0x00F4	Monitor ON	*/
		DrvSw( ON ) ;						/* 0x0070		Driver Mode setting */
		RegWriteA( LNA,  0xC0 ) ;			/* 0x0078	Low Noise mode enable	*/
		RegWriteA( PWMA, 0xC0 ) ;			/* 0x0074	PWM Output Circuit OFF		*/
		RegWriteA( STBB, 0x0F ) ;			// 0x0260	ON
		break ;
	case STB3_ON:
		RegWriteA( STBB, 0x00 ) ;			/* 0x0260	Y-amp,X-amp,DAC,ADC Standby				*/
		RegWriteA( PWMA, 0x00 ) ;			/* 0x0074	PWM Output Circuit OFF		*/
		RegWriteA( LNA,  0x00 ) ;			/* 0x0078	Low Noise mode standby	*/
		DrvSw( OFF ) ;						/* 0x0070	Drvier Block Ena=0 */
		RegWriteA( PWMMONFC, 0x00 ) ;		/* 0x00F4	Monitor OFF	*/
		RegWriteA( DAMONFC, 0x00 ) ;		/* 0x00F5	ExDAC OFF	*/
		SelectGySleep( ON ) ;				/* Gyro Sleep */
		RegWriteA( CLKON, 0x00 ) ;			/* 0x020B	Servo & PWM Clock OFF + D-Gyro I/F OFF	*/
		RegWriteA( I2CSEL, 0x01 ) ;			/* 0x0250	I2C Noise Cancel circuit OFF	*/
		RegWriteA( OSCSTOP, 0x02 ) ;		// 0x0263	Source Clock Input OFF
		break ;
	case STB3_OFF:
		RegWriteA( OSCSTOP, 0x00 ) ;		// 0x0263	Source Clock Input ON
		RegWriteA( I2CSEL, 0x00 ) ;			/* 0x0250	I2C Noise Cancel circuit ON	*/
#ifdef I2CE2PROM
		RegWriteA( CLKON, 0x33 ) ;			/* 0x020B	Servo & PWM Clock ON + D-Gyro I/F ON	*/
#else
 #ifdef SPIE2PROM
		RegWriteA( CLKON, 0x17 ) ;			/* 0x020B	Servo & PWM Clock ON + D-Gyro I/F ON	*/
 #else
		RegWriteA( CLKON, 0x13 ) ;			/* 0x020B	Servo & PWM Clock ON + D-Gyro I/F ON	*/
 #endif
#endif
		SelectGySleep( OFF ) ;				/* Gyro Wake Up */
		RegWriteA( DAMONFC, 0x00 ) ;		/* 0x00F5	ExDAC ON	*/
		RegWriteA( PWMMONFC, 0x80 ) ;		/* 0x00F4	Monitor ON	*/
		DrvSw( ON ) ;						/* 0x0070		Driver Mode setting */
		RegWriteA( LNA,  0xC0 ) ;			/* 0x0078	Low Noise mode enable	*/
		RegWriteA( PWMA, 0xC0 ) ;			/* 0x0074	PWM Output Circuit OFF		*/
		RegWriteA( STBB, 0x0F ) ;			// 0x0260	ON
		break ;
		
	case STB4_ON:
		RegWriteA( STBB, 0x00 ) ;			/* 0x0260	Y-amp,X-amp,DAC,ADC Standby				*/
		RegWriteA( PWMA, 0x00 ) ;			/* 0x0074	PWM Output Circuit OFF		*/
		RegWriteA( LNA,  0x00 ) ;			/* 0x0078	Low Noise mode standby	*/
		DrvSw( OFF ) ;						/* 0x0070	Drvier Block Ena=0 */
		RegWriteA( PWMMONFC, 0x00 ) ;		/* 0x00F4	Monitor OFF	*/
		RegWriteA( DAMONFC, 0x00 ) ;		/* 0x00F5	ExDAC OFF	*/
		RegWriteA( CLKON, 0x12 ) ;			/* 0x020B	Servo & PWM Clock OFF + D-Gyro I/F OFF	*/
		break ;
	case STB4_OFF:
#ifdef I2CE2PROM
		RegWriteA( CLKON, 0x33 ) ;			/* 0x020B	Servo & PWM Clock ON + D-Gyro I/F ON	*/
#else
 #ifdef SPIE2PROM
		RegWriteA( CLKON, 0x17 ) ;			/* 0x020B	Servo & PWM Clock ON + D-Gyro I/F ON	*/
 #else
		RegWriteA( CLKON, 0x13 ) ;			/* 0x020B	Servo & PWM Clock ON + D-Gyro I/F ON	*/
 #endif
#endif
		RegWriteA( DAMONFC, 0x00 ) ;		/* 0x00F5	ExDAC ON	*/
		RegWriteA( PWMMONFC, 0x80 ) ;		/* 0x00F4	Monitor ON	*/
		DrvSw( ON ) ;						/* 0x0070		Driver Mode setting */
		RegWriteA( LNA,  0xC0 ) ;			/* 0x0078	Low Noise mode enable	*/
		RegWriteA( PWMA, 0xC0 ) ;			/* 0x0074	PWM Output Circuit OFF		*/
		RegWriteA( STBB, 0x0F ) ;			// 0x0260	ON
		break ;
	}
}
#endif

//********************************************************************************
// Function Name 	: SetZsp
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Set Zoom Step parameter Function
// History			: First edition 						2010.09.16 Y.Shigeoka
//********************************************************************************
void	SetZsp( unsigned char	UcZoomStepDat )
{
	unsigned long	UlGyrZmx, UlGyrZmy, UlGyrZrx, UlGyrZry ;

	
	/* Zoom Step */
	if(UcZoomStepDat > (ZOOMTBL - 1))
		UcZoomStepDat = (ZOOMTBL -1) ;										/* ãÀðZOOMTBL-1ÉÝè·é */

	if( UcZoomStepDat == 0 )				/* initial setting	*/
	{
		UlGyrZmx	= ClGyxZom[ 0 ] ;		// Same Wide Coefficient
		UlGyrZmy	= ClGyyZom[ 0 ] ;		// Same Wide Coefficient
		/* Initial Rate value = 1 */
	}
	else
	{
		UlGyrZmx	= ClGyxZom[ UcZoomStepDat ] ;
		UlGyrZmy	= ClGyyZom[ UcZoomStepDat ] ;
		
		
	}
	
	// Zoom Value Setting
#ifdef	TESTPNTL
	RamWrite32A( gxgain, UlGyrZmx ) ;		/* 0x1811 */
	RamWrite32A( gygain, UlGyrZmy ) ;		/* 0x1911 */

	RamRead32A( gxgain, &UlGyrZrx ) ;		/* 0x1811 */
	RamRead32A( gygain, &UlGyrZry ) ;		/* 0x1911 */

	// Zoom Value Setting Error Check
	if( UlGyrZmx != UlGyrZrx ) {
		RamWrite32A( gxgain, UlGyrZmx ) ;		/* 0x1811 */
	}

	if( UlGyrZmy != UlGyrZry ) {
		RamWrite32A( gygain, UlGyrZmy ) ;		/* 0x1911 */
	}
#else
	RamWrite32A( gxlens, UlGyrZmx ) ;		/* 0x182A */
	RamWrite32A( gylens, UlGyrZmy ) ;		/* 0x192A */

	RamRead32A( gxlens, &UlGyrZrx ) ;		/* 0x182A */
	RamRead32A( gylens, &UlGyrZry ) ;		/* 0x192A */

	// Zoom Value Setting Error Check
	if( UlGyrZmx != UlGyrZrx ) {
		RamWrite32A( gxlens, UlGyrZmx ) ;		/* 0x182A */
	}

	if( UlGyrZmy != UlGyrZry ) {
		RamWrite32A( gylens, UlGyrZmy ) ;		/* 0x192A */
	}
#endif

}

//********************************************************************************
// Function Name 	: StbOnn
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Stabilizer For Servo On Function
// History			: First edition 						2010.09.15 Y.Shigeoka
//********************************************************************************
//#define	 SETTIMER
 
void StbOnn( void )
{
	unsigned char	UcRegValx,UcRegValy;					// Registor value 
	unsigned char	UcRegIni ;
	unsigned char	UcRegVal1 ;
	unsigned short	UsRamVal1 , UsRamVal2 , UsRamVal3 ,UsRamVal4 , UsRamVal5 , UsRamVal6 ;
	
	RegReadA( LXEQEN, &UcRegValx ) ;				/* 0x0084 */
	RegReadA( LYEQEN, &UcRegValy ) ;				/* 0x008E */
	if( (( UcRegValx & 0x80 ) != 0x80 ) && (( UcRegValy & 0x80 ) != 0x80 ))
	{
		RegReadA( SSSEN, &UcRegVal1 ) ;				/* 0x0097 */
		RamReadA( HXSMTMP , &UsRamVal1 ) ;			/* 0x115E */
		RamReadA( HXTMP , &UsRamVal2 ) ;			/* 0x115D */
		RamReadA( HXIN , &UsRamVal3 ) ;				/* 0x1129 */
		RamReadA( HYSMTMP , &UsRamVal4 ) ;			/* 0x119E */
		RamReadA( HYTMP , &UsRamVal5 ) ;			/* 0x119D */
		RamReadA( HYIN , &UsRamVal6 ) ;				/* 0x1169 */
		
		RegWriteA( SSSEN,  0x88 ) ;				// 0x0097	Smooth Start enable
		
		
		SrvCon( X_DIR, ON ) ;
		SrvCon( Y_DIR, ON ) ;
		
		UcRegIni = 0x11;
		while( (UcRegIni & 0x77) != 0x66 )
		{
			RegReadA( SSSEN,  &UcRegIni ) ;			// 0x0097	Status read
		}
		RegReadA( SSSEN, &UcRegVal1 ) ;				/* 0x0097 */
		RamReadA( HXSMTMP , &UsRamVal1 ) ;			/* 0x115E */
		RamReadA( HXTMP , &UsRamVal2 ) ;			/* 0x115D */
		RamReadA( HXIN , &UsRamVal3 ) ;				/* 0x1129 */
		RamReadA( HYSMTMP , &UsRamVal4 ) ;			/* 0x119E */
		RamReadA( HYTMP , &UsRamVal5 ) ;			/* 0x119D */
		RamReadA( HYIN , &UsRamVal6 ) ;				/* 0x1169 */
		RegWriteA( SSSEN,  0x00 ) ;				// 0x0097	Smooth Start disable	
		
	}
	else
	{
		SrvCon( X_DIR, ON ) ;
		SrvCon( Y_DIR, ON ) ;
	}
}

//********************************************************************************
// Function Name 	: OptCen
// Retun Value		: NON
// Argment Value	: UcOptMode 0:Set 1:Save&Set
//					: UsOptXval Xaxis offset
//					: UsOptYval Yaxis offset
// Explanation		: Send Optical Center
// History			: First edition 						2010.09.16 Y.Shigeoka
//********************************************************************************
void	OptCen( unsigned char UcOptmode , unsigned short UsOptXval , unsigned short UsOptYval )
{
	switch ( UcOptmode ) {
		case VAL_SET :
			RamWriteA( HXINOD	, UsOptXval ) ;								/* 0x1127	Check Hall X optical center */
			RamWriteA( HYINOD	, UsOptYval ) ;								/* 0x1167	Check Hall Y optical center */
			break ;
		case VAL_FIX :
			UsCntXof = UsOptXval ;
			UsCntYof = UsOptYval ;
			RamWriteA( HXINOD	, UsCntXof ) ;								/* 0x1127	Check Hall X optical center */
			RamWriteA( HYINOD	, UsCntYof ) ;								/* 0x1167	Check Hall Y optical center */
#ifdef USE_EXE2PROM
			E2pWrt( OPT_CENTER_X,	2, ( unsigned char * )&UsCntXof ) ;
			E2pWrt( OPT_CENTER_Y,	2, ( unsigned char * )&UsCntYof ) ;
#endif

			break ;
		case VAL_SPC :
			RamReadA( HXINOD   , &UsOptXval ) ;								/* 0x1127	Check Hall X optical center */
			RamReadA( HYINOD   , &UsOptYval ) ;								/* 0x1167	Check Hall Y optical center */
			UsCntXof = UsOptXval ;
			UsCntYof = UsOptYval ;
#ifdef USE_EXE2PROM
			E2pWrt( OPT_CENTER_X,	2, ( unsigned char * )&UsCntXof ) ;
			E2pWrt( OPT_CENTER_Y,	2, ( unsigned char * )&UsCntYof ) ;
#endif

			break ;
	}

}


#ifdef		MODULE_CALIBRATION		/* calibration code for Module maker */
//********************************************************************************
// Function Name 	: OscAdj
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: OSC Clock adjustment
// History			: First edition 						2011.06.09 Y.Shigeoka
//********************************************************************************
#define	RRATETABLE	8
#define	CRATETABLE	16
const signed char	ScRselRate[ RRATETABLE ]	= {
		-10,			/* -10% */
		 -6,			/*	-6% */
		 -3,			/*	-3% */
		  0,			/*	 0% */
		  5,			/*	 5% */
		  9,			/*	 9% */
		 14,			/*	14% */
		 20				/*	20% */
	} ;
const signed char	ScCselRate[ CRATETABLE ]	= {
		-14,			/* -14% */
		-12,			/* -12% */
		-10,			/* -10% */
		 -8,			/*	-8% */
		 -5,			/*	-5% */
		 -3,			/*	-3% */
		 -1,			/*	-1% */
		  0,			/*	 0% */
		  0,			/*	 0% */
		  3,			/*	 3% */
		  5,			/*	 5% */
		  7,			/*	 7% */
		  9,			/*	 9% */
		 11,			/*	11% */
		 13,			/*	13% */
		 15				/*	15% */
	} ;
	

#define	TARGET_FREQ		48000.0F
#define	START_RSEL		0x03	/* Typ */
#define	START_CSEL		0x07	/* Typ bit4:OSCPMSEL */
#define	MEAS_MAX		32		/* ãÀ32ñ */
/* Measure Status (UcClkJdg) */
#define	UNDR_MEAS		0x00
#define	FIX_MEAS		0x01
#define	JST_FIX			0x03
#define	OVR_MEAS		0x80
/* Measure Check Flag (UcMeasFlg) */
#define	RSELFX			0x08
#define	RSEL1ST			0x01
#define	RSEL2ND			0x02
#define	CSELFX			0x80
#define	CSELPLS			0x10
#define	CSELMNS			0x20

unsigned short	OscAdj( void )
{
	unsigned char	UcMeasFlg ;									/* Measure check flag */
	UnWrdVal		StClkVal ;									/* Measure value */
	unsigned char	UcMeasCnt ;									/* Measure counter */
	unsigned char	UcOscrsel , UcOsccsel ;						/* Reg set value */
	unsigned char	UcPwmDivBk ;								/* back up value */
	unsigned char	UcClkJdg ;									/* State flag */
	float			FcalA,FcalB ;								/* calcurate value */
	signed char		ScTblRate_Val, ScTblRate_Now, ScTblRate_Tgt ;	/* rate */
	float			FlRatePbk,FlRateMbk ;							/* Rate bk	*/
	unsigned char	UcOsccselP , UcOsccselM ;					/* Reg set value */
	unsigned short	UsResult ;
//	unsigned char	UcOscsetBk ;								/* Reg set value */
	
	UcMeasFlg = 0 ;						/* Clear Measure check flag */
	UcMeasCnt = 0;						/* Clear Measure counter */
	UcClkJdg = UNDR_MEAS;				/* under Measure */
	UcOscrsel = START_RSEL ;
	UcOsccsel = START_CSEL ;
	/* check */
//	RegReadA( OSCSET, &UcOscsetBk ) ;	// 0x0264	
//	UcOscrsel = ( UcOscsetBk & 0xE0 ) >> 5 ;
//	UcOsccsel = ( UcOscsetBk & 0x1E ) >> 1 ;
	/**/
	
	RegReadA( PWMDIV, &UcPwmDivBk ) ;	/* 0x0212 */
	RegWriteA( PWMDIV,	0x00 ) ;		// 0x0212	 PWM Clock = Xtalck
	RegWriteA( OSCSET, ( UcOscrsel << 5 ) | (UcOsccsel << 1 ) | 0x01 ) ;	// 0x0264	 
	
	while( UcClkJdg == UNDR_MEAS )
	{
		UcMeasCnt++ ;						/* Measure count up */
		UcOscAdjFlg = MEASSTR ;				// Start trigger ON
		
		while( UcOscAdjFlg != MEASFIX )
		{
			;
		}
		
		UcOscAdjFlg = 0x00 ;				// Clear Flag
		RegReadA( OSCCK_CNTR0, &StClkVal.StWrdVal.UcLowVal ) ;		/* 0x026B */
		RegReadA( OSCCK_CNTR1, &StClkVal.StWrdVal.UcHigVal ) ;		/* 0x026C */
		
		FcalA = (float)StClkVal.UsWrdVal ;
		FcalB = TARGET_FREQ / FcalA ;
		FcalB =  FcalB - 1.0F ;
		FcalB *= 100.0F ;
		
		if( FcalB == 0.0F )
		{
			UcClkJdg = JST_FIX ;					/* Just 48MHz */
			UcMeasFlg |= ( CSELFX | RSELFX ) ;		/* Fix Flag */
			break ;
		}

		/* Rsel check */
		if( !(UcMeasFlg & RSELFX) )
		{
			if(UcMeasFlg & RSEL1ST)
			{
				UcMeasFlg |= ( RSELFX | RSEL2ND ) ;
			}
			else
			{
				UcMeasFlg |= RSEL1ST ;
			}
			ScTblRate_Now = ScRselRate[ UcOscrsel ] ;					/* ¡ÌRate */
			ScTblRate_Tgt = ScTblRate_Now + (short)FcalB ;
			if( ScTblRate_Now > ScTblRate_Tgt )
			{
				while(1)
				{
					if( UcOscrsel == 0 )
					{
						break;
					}
					UcOscrsel -= 1 ;
					ScTblRate_Val = ScRselRate[ UcOscrsel ] ;	
					if( ScTblRate_Tgt >= ScTblRate_Val )
					{
						break;
					}
				}
			}
			else if( ScTblRate_Now < ScTblRate_Tgt )
			{
				while(1)
				{
					if(UcOscrsel == (RRATETABLE - 1))
					{
						break;
					}
					UcOscrsel += 1 ;
					ScTblRate_Val = ScRselRate[ UcOscrsel ] ;	
					if( ScTblRate_Tgt <= ScTblRate_Val )
					{
						break;
					}
				}
			}
			else
			{
				;
			}
		}
		else
		{		
		/* Csel check */
			if( FcalB > 0 )			/* Plus */
			{
				UcMeasFlg |= CSELPLS ;
				FlRatePbk = FcalB ;
				UcOsccselP = UcOsccsel ;
				if( UcMeasFlg & CSELMNS)
				{
					UcMeasFlg |= CSELFX ;
					UcClkJdg = FIX_MEAS ;			/* OK */
				}
				else if(UcOsccsel == (CRATETABLE - 1))
				{
					if(UcOscrsel < ( RRATETABLE - 1 ))
					{
						UcOscrsel += 1 ;
						UcOsccsel = START_CSEL ;
						UcMeasFlg = 0 ;			/* Clear */
					}
					else
					{
						UcClkJdg = OVR_MEAS ;			/* Over */
					}
				}
				else
				{
					UcOsccsel += 1 ;
				}
			}
			else					/* Minus */
			{
				UcMeasFlg |= CSELMNS ;
				FlRateMbk = (-1)*FcalB ;
				UcOsccselM = UcOsccsel ;
				if( UcMeasFlg & CSELPLS)
				{
					UcMeasFlg |= CSELFX ;
					UcClkJdg = FIX_MEAS ;			/* OK */
				}
				else if(UcOsccsel == 0x00)
				{
					if(UcOscrsel > 0)
					{
						UcOscrsel -= 1 ;
						UcOsccsel = START_CSEL ;
						UcMeasFlg = 0 ;			/* Clear */
					}
					else
					{
					UcClkJdg = OVR_MEAS ;			/* Over */
					}
				}
				else
				{
					UcOsccsel -= 1 ;
				}
			}
			if(UcMeasCnt >= MEAS_MAX)
			{
				UcClkJdg = OVR_MEAS ;			/* Over */
			}
		}	
		RegWriteA( OSCSET, ( UcOscrsel << 5 ) | (UcOsccsel << 1 ) | 0x01 ) ;	// 0x0264	 
	}
	
	UsResult = EXE_END ;
	
	if(UcClkJdg == FIX_MEAS)
	{
		if( FlRatePbk < FlRateMbk )
		{
			UcOsccsel = UcOsccselP ; 
		}
		else
		{
			UcOsccsel = UcOsccselM ; 
		}
	
		RegWriteA( OSCSET, ( UcOscrsel << 5 ) | (UcOsccsel << 1 ) | 0x01 ) ;	// 0x0264	 
	}
	StAdjPar.UcOscVal = ( ( UcOscrsel << 5 ) | (UcOsccsel << 1 ) | 0x01 );
	
	if(UcClkJdg == OVR_MEAS)
	{
		UsResult = EXE_OCADJ ;
		StAdjPar.UcOscVal = 0x00 ;
	}
	RegWriteA( PWMDIV,	UcPwmDivBk ) ;		// 0x0212	 PWM Clock set

	return( UsResult );
}
#endif
	
#ifdef HALLADJ_HW
//==============================================================================
//	Function	:	SetSineWave()
//	inputs		:	UcJikuSel	0: X-Axis
//								1: Y-Axis
//					UcMeasMode	0: Loop Gain frequency setting
//								1: Bias/Offset frequency setting
//	outputs 	:	void
//	explanation :	Initializes sine wave settings:
//						Sine Table, Amplitue, Offset, Frequency
//	revisions	:	First Edition						   2011.04.13 d.yamagata
//==============================================================================
void SetSineWave( unsigned char UcJikuSel , unsigned char UcMeasMode )
{
	unsigned char	UcSWFC1[]	= { 0xBA/*133Hz*/ , 0x4F/*20Hz*/ } ,			// { Loop Gain setting , Bias/Offset setting}
					UcSWFC2[]	= { 0x01/*200Hz*/ , 0x02/*20Hz*/ } ;			// { Loop Gain setting , Bias/Offset setting}

	unsigned char	UcSwSel[2][2] = { { 0x80 , 0x40 } , 						// Loop gain setting
									  { 0x00 , 0x00 }							// Bias/Offset Setting
									};

	UcMeasMode &= 0x01;
	UcJikuSel  &= 0x01;

	/* Auto Set "Sine Table", "Amplitude", and "Offset" RAMs */
//	  RegWriteA( SWEN , 0x08 ); 												  // SWEN	  [ SINON | SINRST | - | - ][ SININISET | - | - | - ]

	/* Manually Set Offset */
	RamWriteA( WAVXO , 0x0000 );												// 0x11D5
	RamWriteA( WAVYO , 0x0000 );												// 0x11D6

	/* Manually Set Amplitude */
	RamWriteA( wavxg , 0x7FFF );												// 0x13C3
	RamWriteA( wavyg , 0x7FFF );												// 0x13C4

	/* Set Frequency */
	//****************************************************
	//	üg = (fs/96)*(SWB+1)/(SWA+1)*1/(2^SWC)	[ Hz ]
	//****************************************************
	RegWriteA( SWFC1 , UcSWFC1[UcMeasMode] );									// 0x00DD	 [ SWB(7:4) ][ SWA(3:0) ]
	RegWriteA( SWFC2 , UcSWFC2[UcMeasMode] );									// 0x00DE	 [ SWCIR | SWDIR | - | - ][ SWCOM | SWFC(2:0) ]
	RegWriteA( SWFC3 , 0x00 );													// 0x00DF	 [ SWINIP | SWBGNP(6:0) ]
	RegWriteA( SWFC4 , 0x00 );													// 0x00E0	 [ SWFINP | SWENDP(6:0) ]
	RegWriteA( SWFC5 , 0x00 );													// 0x00E1	 [ SWFT(7:0) ]

	/* Set Sine Wave Input RAM */
	RegWriteA( SWSEL , UcSwSel[UcMeasMode][UcJikuSel] );						// 0x00E2	 [ SINX | SINY | SING | SIN0END ][ SINGDPX1 | SINGDPX2 | SINGDPY1 | SINGDPY2 ]

	/* Clear Optional Sine wave input address */
	if( !UcMeasMode )		// Loop Gain mode
	{
		RegWriteA( SINXADD , 0x00 );											// 0x00E3  [ SINXADD(7:0) ]
		RegWriteA( SINYADD , 0x00 );											// 0x00E4  [ SINYADD(7:0) ]
	}
	else if( !UcJikuSel )	// Bias/Offset mode X-Axis
	{
		RegWriteA( SINXADD , (unsigned char)LXDODAT );							// 0x00E3  [ SINXADD(7:0) ]
		RegWriteA( SINYADD , 0x00 );											// 0x00E4  [ SINYADD(7:0) ]
	}
	else					// Bias/Offset mode Y-Axis
	{
		RegWriteA( SINXADD , 0x00 );											// 0x00E3  [ SINXADD(7:0) ]
		RegWriteA( SINYADD , (unsigned char)LYDODAT );							// 0x00E4  [ SINYADD(7:0) ]
	}
}

//==============================================================================
//	Function	:	StartSineWave()
//	inputs		:	none
//	outputs 	:	void
//	explanation :	Starts sine wave
//	revisions	:	First Edition						   2011.04.13 d.yamagata
//==============================================================================
void StartSineWave( void )
{
	/* Start Sine Wave */
	//****************************************************
	//	Ó@TCgðÀs·éOÉÈºÌRAM/REGðÝèµÄ­¾³¢@
	//				- RAM:1290h`12A8h (TCge[u)
	//				- RAM:13C3h`13C4h (U)
	//				- RAM:11D5h`11D6h (JnÀW)
	//				- REG:00DDh,00DEh  (üg)
	//****************************************************
	RegWriteA( SWEN , 0x80 );													// 0x00DB	  [ SINON | SINRST | - | - ][ SININISET | - | - | - ]
}

//==============================================================================
//	Function	:	StopSineWave()
//	inputs		:	void
//	outputs 	:	void
//	explanation :	Stops sine wave
//	revisions	:	First Edition						   2011.04.13 d.yamagata
//==============================================================================
void StopSineWave( void )
{
	/* Set Sine Wave Input RAM */
	RegWriteA( SWSEL   , 0x00 );												// 0x00E2	 [ SINX | SINY | SING | SIN0END ][ SINGDPX1 | SINGDPX2 | SINGDPY1 | SINGDPY2 ]
	RegWriteA( SINXADD , 0x00 );												// 0x00E3  [ SINXADD(7:0) ]
	RegWriteA( SINYADD , 0x00 );												// 0x00E4  [ SINYADD(7:0) ]

	/* Stop Sine Wave */
	RegWriteA( SWEN  , 0x00 );													// 0x00DB	  [ SINON | SINRST | - | - ][ SININISET | - | - | - ]
}

//==============================================================================
//	Function	:	SetMeaseFil_LoopGain()
//	inputs		:	UcJikuSel	0: X-Axis
//								1: Y-Axis
//					UcMeasMode	0: Loop Gain frequency setting
//								1: Bias/Offset frequency setting
//					UcFilSel
//	outputs 	:	void
//	explanation :
//	revisions	:	First Edition						   2011.04.13 d.yamagata
//==============================================================================
void SetMeasFil( unsigned char UcJikuSel , unsigned char UcMeasMode , unsigned char UcFilSel )
{
	unsigned short	UsIn1Add[2][2] = { { LXC1	, LYC1	 } ,					// Loop Gain Setting
									   { ADHXI0 , ADHYI0 }						// Bias/Offset Setting
									 } ,
					UsIn2Add[2][2] = { { LXC2	, LYC2	 } ,					// Loop Gain Setting
									   { 0x0000 , 0x0000 }						// Bias/Offset Setting
									 } ;

	/* Set Limits on Input Parameters */
	UcJikuSel  &= 0x01;
	UcMeasMode &= 0x01;
	if( UcFilSel > NOISE ) UcFilSel = THROUGH;
	
	MesFil( UcFilSel ) ;					/* Set Measure filter */

	RegWriteA( MS1INADD , (unsigned char)UsIn1Add[UcMeasMode][UcJikuSel] ); 	// 0x00C2
	RegWriteA( MS1OUTADD, 0x00 );												// 0x00C3


	RegWriteA( MS2INADD , (unsigned char)UsIn2Add[UcMeasMode][UcJikuSel] ); 	// 0x00C6
	RegWriteA( MS2OUTADD, 0x00 );												// 0x00C7

	/* Set Measure Filter Down Sampling */
	//****************************************************
	//	ªèFilterÌ_ETvO = Fs/(MSFDS+1)
	//****************************************************
	RegWriteA( MSFDS , 0x00 );													// 0x00C8
}

//==============================================================================
//	Function	:	StartMeasFil()
//	inputs		:	void
//	outputs 	:	void
//	explanation :
//	revisions	:	First Edition						   2011.04.13 d.yamagata
//==============================================================================
void StartMeasFil( void )
{
	/* Enable Measure Filters */
	RegWriteA( MSF1EN , 0x01 ); 												// 0x00C0		[ MSF1SW | - | - | - ][ - | - | - | MSF1EN ]
	RegWriteA( MSF2EN , 0x01 ); 												// 0x00C4		[ MSF2SW | - | - | - ][ - | - | - | MSF2EN ]
}

//==============================================================================
//	Function	:	StopMeasFil()
//	inputs		:	void
//	outputs 	:	void
//	explanation :
//	revisions	:	First Edition						   2011.04.13 d.yamagata
//==============================================================================
void StopMeasFil( void )
{
	/* Enable Measure Filters */
	RegWriteA( MSF1EN , 0x00 ); 												// 0x00C0		[ MSF1SW | - | - | - ][ - | - | - | MSF1EN ]
	RegWriteA( MSF2EN , 0x00 ); 												// 0x00C4		[ MSF2SW | - | - | - ][ - | - | - | MSF2EN ]
}

//==============================================================================
//	Function	:	LoopGainAdj()
//	inputs		:	UcJikuSel	0: X-Axis, 1: Y-Axis
//	outputs 	:	void
//	explanation :
//	revisions	:	First Edition						   2011.04.13 d.yamagata
//==============================================================================
unsigned char	 LoopGainAdj( unsigned char UcJikuSel)
{
	//****************************************************
	//	Ó@±Ì²®ÖðÀs·éOÉA
	//			  ÈºÌRAMðKØÈlÉÝèµÄ­¾³¢
	//				 - lxgain [ 132Ah ]
	//				 - lygain [ 136Ah ]
	//****************************************************

	unsigned short	UsSineAdd[] = { lxxg   , lyxg	} ;
	unsigned short	UsSineGanCVL[] = { 0x198A , 0x198A } ;
	unsigned short	UsSineGanPWM[] = { 0x32F5 , 0x32F5 } ;
	unsigned short	UsRltVal ;
	unsigned char	UcAdjSts	= FAILURE ;
	
	UcJikuSel &= 0x01;

	StbOnn() ;											// Slope Mode
	
	// Wait 200ms
	WitTim( 200 ) ;
	
	/* set start gain */
	LopPar( UcJikuSel ) ;
	
	/* set sine wave */
	SetSineWave( UcJikuSel , __MEASURE_LOOPGAIN );

	/* Set Servo Filter */
	if( UcPwmMod == PWMMOD_CVL ) {
		RamWriteA( UsSineAdd[UcJikuSel] , UsSineGanCVL[UcJikuSel] ); 					// Set Sine Wave input amplitude
	}else{
		RamWriteA( UsSineAdd[UcJikuSel] , UsSineGanPWM[UcJikuSel] ); 					// Set Sine Wave input amplitude
	}
	
	/* Set Measure Count */
	RegWriteA( MSMPLNSH , 0x03 );												// 0x00CB
	RegWriteA( MSMPLNSL , 0xFF );												// 0x00CA

	/* Set Adjustment Limits */
	RamWriteA( AJLPMAX	, 0x7FFF ); 											// 0x1222	Loop gain adjustment max limit +6dB
	RamWriteA( AJLPMIN	, 0x2000 ); 											// 0x1223	Loop gain adjustment min limit -6dB
	RegWriteA( AJWAIT	, 0x00 );												// 0x00D0	Wait Time = Sampling Rate * AJWAIT
	RegWriteA( AJTIMEOUT, 0xFF );												// 0x00D1	Time-Out time = Sampling Rate * AJTIMEOUT * 1024
	RegWriteA( AJNUM	, 0x00 );												// 0x00D3	Calculation number

	/* set Measure Filter */
	SetMeasFil( UcJikuSel , __MEASURE_LOOPGAIN , LOOPGAIN );

	/* Start Measure Filters */
	StartMeasFil();

	/* Start Sine Wave */
	StartSineWave();

	/* Enable Loop Gain Adjustment */
	RegWriteA( AJTA 	, (0x98 | UcJikuSel) ); 								// 0x00CF		  [ AJABSON | AJSIN0ON1 | AJSIN0ON2 | AJMODE(4:2) | AJSEL(1:0) ]

	/* Check Busy Flag */
	do{
		RegReadA( FLGM , &UcAdjBsy );											// 0x00F8
	}while(( UcAdjBsy & 0x40 ) != 0x00 );

	RegReadA( AJSTATUS , &UcAdjBsy );								// 0x00D2
	if( UcAdjBsy )
	{
		if( UcJikuSel == X_DIR )
		{
			RamReadA( lxgain, &UsRltVal ) ;							// 0x132A
			StAdjPar.StLopGan.UsLxgVal	= UsRltVal ;
			StAdjPar.StLopGan.UsLxgSts	= 0x0000 ;
		} else {
			RamReadA( lygain, &UsRltVal ) ;							// 0x132A
			StAdjPar.StLopGan.UsLygVal	= UsRltVal ;
			StAdjPar.StLopGan.UsLygSts	= 0x0000 ;
		}
		RegWriteA( AJSTATUS , 0x00 );								// 0x00D2 Clear Status

	}
	else
	{
		if( UcJikuSel == X_DIR )
		{
			RamReadA( lxgain, &UsRltVal ) ;							// 0x132A
			StAdjPar.StLopGan.UsLxgVal	= UsRltVal ;
			StAdjPar.StLopGan.UsLxgSts	= 0xFFFF ;
		} else {
			RamReadA( lygain, &UsRltVal ) ;							// 0x132A
			StAdjPar.StLopGan.UsLygVal	= UsRltVal ;
			StAdjPar.StLopGan.UsLygSts	= 0xFFFF ;
		}
		UcAdjSts	= SUCCESS ;												// Status OK
	}


	/* Cut Sine input */
	RamWriteA( UsSineAdd[UcJikuSel] , 0x0000 ); 								// Set Sine Wave input amplitude
	
	/* Stop Sine Wave */
	StopSineWave();

	/* Stop Measure Filter */
	StopMeasFil();

	return( UcAdjSts ) ;
}

//==============================================================================
//	Function	:	BiasOffsetAdj()
//	inputs		:	UcJikuSel	0: X-Axis, 1: Y-Axis	UcMeasCnt  :Measure Count
//	outputs 	:	Result status
//	explanation :
//	revisions	:	First Edition						   2011.04.13 d.yamagata
//==============================================================================
unsigned char  BiasOffsetAdj( unsigned char UcJikuSel , unsigned char UcMeasCnt )
{
	//****************************************************
	//	Ó@±Ì²®ÖðÀs·éOÉA
	//			  ÈºÌRAMðKØÈlÉÝèµÄ­¾³¢
	//				 - DAHLXO [ 1114h ]
	//				 - DAHLXB [ 1115h ]
	//				 - DAHLYO [ 1116h ]
	//				 - DAHLYB [ 1117h ]
	//****************************************************
	unsigned char 	UcHadjRst ;
	unsigned short	UsEqSwAddOff[]	 = { LXEQEN , LYEQEN } ;
	unsigned short	UsEqSwAddOn[]	 = { LYEQEN , LXEQEN } ;
									/*	   STEP 	OFSTH	 OFSTL	  AMPH	   AMPL  (80%)*/
	unsigned short	UsTgtVal[2][5]	  = {{ 0x4026 , 0x0400 , 0xFC00 , 0x6A65 , 0x6265 },	/* ROUGH */
										 { 0x2013 , 0x0200 , 0xFE00 , 0x67B5 , 0x6515 }} ;	/* FINE */
	

	if(UcMeasCnt > 1)		UcMeasCnt = 1 ;
	
	UcJikuSel &= 0x01;

	/* Set Sine Wave */
	SetSineWave( UcJikuSel , __MEASURE_BIASOFFSET );

	/* Set Servo Filter */
	RegWriteA( UsEqSwAddOff[UcJikuSel] ,	0x45 );
	RegWriteA( UsEqSwAddOn[UcJikuSel] ,		0xC5 );

	/* Set Measure Count */
	RegWriteA( MSMPLNSH , 0x00 );									// 0x00CB
	RegWriteA( MSMPLNSL , 0x03 );									// 0x00CA

	/* Set Adjustment Limits */
	RamWriteA( AJHOFFD	, UsTgtVal[UcMeasCnt][0] );					// 0x1224	Offset adjustment step size
	RamWriteA( AJCTMAX	, UsTgtVal[UcMeasCnt][1] );					// 0x121C	Offset adjustment max center limit
	RamWriteA( AJCTMIN	, UsTgtVal[UcMeasCnt][2] );					// 0x121D	Offset adjustment min center limit
	RamWriteA( AJHGAIND , UsTgtVal[UcMeasCnt][0] );					// 0x1225	Bias adjustment step size
	RamWriteA( AJAPMAX	, UsTgtVal[UcMeasCnt][3] );					// 0x121E	Bias adjustment max value limit
	RamWriteA( AJAPMIN	, UsTgtVal[UcMeasCnt][4] );					// 0x121F	Bias adjustment min value limit
	RegWriteA( AJWAIT	, 0x00 );									// 0x00D0	Wait Time = Sampling Rate * AJWAIT
	RegWriteA( AJTIMEOUT, 0xE5 );									// 0x00D1	Time-Out time = Sampling Rate * AJTIMEOUT * 1024

	/* Set Measure Filter */
	SetMeasFil( UcJikuSel , __MEASURE_BIASOFFSET , HALL_ADJ );

	/* Start Measure Filters */
	StartMeasFil();

	/* Start Sine Wave */
	StartSineWave();

	/*Enabel Bias/Offset Adjustment */
	RegWriteA( AJTA 	, (0x0E | UcJikuSel) );						// 0x00CF		  [ AJABSON | AJSIN0ON1 | AJSIN0ON2 | AJMODE(4:2) | AJSEL(1:0) ]

	/* Check Busy Flag */
	do{
		RegReadA( FLGM , &UcAdjBsy );								// 0x00F8
	}while(( UcAdjBsy & 0x40 ) != 0x00 );
	
	RegReadA( AJSTATUS , &UcAdjBsy );								// 0x00D2
	if( UcAdjBsy )
	{
		if( UcJikuSel == X_DIR )
		{
			UcHadjRst = EXE_HXADJ ;
		}
		else
		{
			UcHadjRst = EXE_HYADJ ;
		}
		RegWriteA( AJSTATUS , 0x00 );								// 0x00D2 Clear Status

	}
	else
	{
		UcHadjRst = EXE_END ;
	}

	/* Stop Sine Wave */
	StopSineWave();

	/* Stop Measure Filter */
	StopMeasFil();

	/* Set Servo Filter */
	RegWriteA( UsEqSwAddOff[UcJikuSel] , 0xC5 );
	
	if( UcJikuSel == X_DIR )
	{
		RamReadA( MSSAPAV, &StAdjPar.StHalAdj.UsHlxMxa	) ;	 	// 0x1212 Max width value
		RamReadA( MSSCTAV, &StAdjPar.StHalAdj.UsHlxCna	) ;	 	// 0x120F offset value
	}
	else
	{
		RamReadA( MSSAPAV, &StAdjPar.StHalAdj.UsHlyMxa	) ;	 	// 0x1212 Max width value
		RamReadA( MSSCTAV, &StAdjPar.StHalAdj.UsHlyCna	) ;	 	// 0x120F offset value
	}

	return( UcHadjRst ) ;
}

#endif


//********************************************************************************
// Function Name 	: GyrGan
// Retun Value		: NON
// Argment Value	: UcGygmode 0:Set 1:Save&Set
//					: UlGygXval Xaxis Gain
//					: UlGygYval Yaxis Gain
// Explanation		: Send Gyro Gain
// History			: First edition 						2011.02.09 Y.Shigeoka
//********************************************************************************
void	GyrGan( unsigned char UcGygmode , unsigned long UlGygXval , unsigned long UlGygYval )
{
	switch ( UcGygmode ) {
		case VAL_SET :
			RamWrite32A( gxzoom, UlGygXval ) ;		// 0x1828
			RamWrite32A( gyzoom, UlGygYval ) ;		// 0x1928
			break ;
		case VAL_FIX :
			RamWrite32A( gxzoom, UlGygXval ) ;		// 0x1828
			RamWrite32A( gyzoom, UlGygYval ) ;		// 0x1928
#ifdef USE_EXE2PROM
			E2pWrt( GYRO_GAIN_X,	4, ( unsigned char * )&UlGygXval ) ;
			E2pWrt( GYRO_GAIN_Y,	4, ( unsigned char * )&UlGygYval ) ;
#endif
			break ;
		case VAL_SPC :
			RamRead32A( gxzoom, &UlGygXval ) ;		// 0x1828
			RamRead32A( gyzoom, &UlGygYval ) ;		// 0x1928
#ifdef USE_EXE2PROM
			E2pWrt( GYRO_GAIN_X,	4, ( unsigned char * )&UlGygXval ) ;
			E2pWrt( GYRO_GAIN_Y,	4, ( unsigned char * )&UlGygYval ) ;
#endif
			break ;
	}

}

//********************************************************************************
// Function Name 	: SetPanTiltMode
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Pan-Tilt Enable/Disable
// History			: First edition 						2011.02.09 Y.Shigeoka
//********************************************************************************
void	SetPanTiltMode( unsigned char UcPnTmod )
{
	switch ( UcPnTmod ) {
		case OFF :
			RegWriteA( GPANON, 0x00 ) ;			// 0x0109	X,Y Pan/Tilt Function OFF
			break ;
		case ON :
			RegWriteA( GPANON, 0x11 ) ;			// 0x0109	X,Y Pan/Tilt Function ON
			break ;
	}

}

#ifdef GAIN_CONT
//********************************************************************************
// Function Name 	: TriSts
// Retun Value		: Tripod Status
//					: bit0( 1:Y Tripod ON / 0:OFF)
//					: bit4( 1:X Tripod ON / 0:OFF)
//					: bit7( 1:Tripod ENABLE  / 0:DISABLE)
// Argment Value	: NON
// Explanation		: Read Status of Tripod mode Function
// History			: First edition 						2011.08.11 Y.Hayashi
//********************************************************************************
unsigned char	TriSts( void )
{
	unsigned char UcRsltSts = 0;
	unsigned char UcVal ;

	RegReadA( GADJGANGXMOD, &UcVal ) ;	// 0x012B
	if( UcVal & 0x07 ){						// Gain control enable?
		RegReadA( GLEVJUGE, &UcVal ) ;	// 0x01FB
		UcRsltSts = UcVal & 0x11 ;		// bit0, bit4 set
		UcRsltSts |= 0x80 ;				// bit7 ON
	}
	return( UcRsltSts ) ;
}
#endif

//********************************************************************************
// Function Name 	: DrvPwmSw
// Retun Value		: Mode Status
//					: bit4( 1:PWM / 0:LinearPwm)
// Argment Value	: NON
// Explanation		: Select Driver mode Function
//					: attention >>> this is changed parameter by LSI version
//					:               Now , this value is only LC898111A
// History			: First edition 						2012.12.20 Y.Shigeoka
//********************************************************************************
unsigned char	DrvPwmSw( unsigned char UcSelPwmMod )
{

	switch ( UcSelPwmMod ) {
		case Mlnp :
			RegWriteA( DRVFC	, 0xE3 ) ;						// 0x0070	MODE=2,Drvier Block Ena=1,FullMode=1
			RegWriteA( LXEQFC2 	, 0x01 ) ;						// 0x0083		Linearâ³ON
			RegWriteA( LYEQFC2	, 0x01 ) ;						// 0x008D		
			UcPwmMod = PWMMOD_CVL ;
			break ;
		
		case Mpwm :
			RegWriteA( LXEQFC2	, 0x00 ) ;						// 0x0083		Linearâ³OFF
			RegWriteA( LYEQFC2	, 0x00 ) ;						// 0x008D		
 #ifdef	LOWCURRENT
			RegWriteA( DRVFC	, 0x03 ) ;						// 0x0070	DMODE=0,DRMODE=0,DRMODESEL=0
 #else
			RegWriteA( DRVFC	, 0xC3 ) ;						// 0x0070	MODE=1,Drvier Block Ena=1,FullMode=1
 #endif
			UcPwmMod = PWMMOD_PWM ;
 			break ;
	}
	
	return( UcSelPwmMod << 4 ) ;
}

#ifdef		MODULE_CALIBRATION		/* calibration code for Module maker */
 #ifdef	NEUTRAL_CENTER
//********************************************************************************
// Function Name 	: TneHvc
// Retun Value		: NON
// Argment Value	: NON
// Explanation		: Tunes the Hall VC offset
// History			: First edition 						2013.03.13	T.Tokoro
//********************************************************************************
unsigned char	TneHvc( void )
{
	unsigned char	UcRsltSts;
	unsigned short	UsMesRlt1 ;
	unsigned short	UsMesRlt2 ;
	
	SrvCon( X_DIR, OFF ) ;				// X Servo OFF
	SrvCon( Y_DIR, OFF ) ;				// Y Servo OFF
	
	//WitTim( 1000 ) ;
	//WitTim( 1000 ) ;
	//WitTim( 1000 ) ;
	WitTim( 500 ) ;
	
	//½Ïlªè
	
	MesFil( THROUGH ) ;					// Set Measure Filter
	
	RegWriteA( MSF1EN, 0x01 ) ;			// 0x00C0		Measure Filter1 Equalizer ON
	RegWriteA( MSF2EN, 0x01 ) ;			// 0x00C4		Measure Filter2 Equalizer ON
	
	RegWriteA( MSMPLNSH, 0x00 ) ;		// 0x00CB
	RegWriteA( MSMPLNSL, 0x3F ) ;		// 0x00CA		64 times
	
	RegWriteA( MS1INADD, 0x01 ) ;		// 0x00C2		HXIDATðªè·é
	RegWriteA( MS2INADD, 0x04 ) ;		// 0x00C6		HYIDATðªè·é
	
	//RegWriteA( MSMA, 0x01 ) ;			// 0x00C9		Measure start
	BsyWit( MSMA, 0x01 ) ;				// 0x00C9		Measure
	//RegWriteA( MSMA, 0x00 ) ;			// 0x00C9		Measure stop
	
	RamReadA( MSAV1, &UsMesRlt1 ) ;	// 0x1205		Measure Result
	RamReadA( MSAV2, &UsMesRlt2 ) ;	// 0x1202		Measure Result
	
	StAdjPar.StHalAdj.UsHlxCna = UsMesRlt1;			//Measure Result Store
	StAdjPar.StHalAdj.UsHlxCen = UsMesRlt1;			//Measure Result Store
	
	StAdjPar.StHalAdj.UsHlyCna = UsMesRlt2;			//Measure Result Store
	StAdjPar.StHalAdj.UsHlyCen = UsMesRlt2;			//Measure Result Store
	
	RegWriteA( MSF1EN, 0x00 ) ;			// 0x00C0		Measure Filter1 Equalizer OFF
	RegWriteA( MSF2EN, 0x00 ) ;			// 0x00C4		Measure Filter2 Equalizer OFF
	
	UcRsltSts = EXE_END ;				// Clear Status
	
	SrvCon( X_DIR, ON ) ;				// X Servo ON
	SrvCon( Y_DIR, ON ) ;				// Y Servo ON
	
	return( UcRsltSts );
}
 #endif	//NEUTRAL_CENTER
#endif
	
//********************************************************************************
// Function Name 	: SetGcf
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Set DI filter coefficient Function
// History			: First edition 						2013.03.22 Y.Shigeoka
//********************************************************************************
void	SetGcf( unsigned char	UcSetNum )
{
	unsigned long	UlGyrCof ;

	
	/* Zoom Step */
	if(UcSetNum > (COEFTBL - 1))
		UcSetNum = (COEFTBL -1) ;			/* ãÀðCOEFTBL-1ÉÝè·é */

	UlGyrCof	= ClDiCof[ UcSetNum ] ;
		
	// Zoom Value Setting
	RamWrite32A( gxh1c, UlGyrCof ) ;		/* 0x1814 */
	RamWrite32A( gyh1c, UlGyrCof ) ;		/* 0x1914 */

#ifdef H1COEF_CHANGER
		SetH1cMod( UcSetNum ) ;							/* Re-setting */
#endif

}

#ifdef H1COEF_CHANGER
//********************************************************************************
// Function Name 	: SetH1cMod
// Retun Value		: NON
// Argment Value	: Command Parameter
// Explanation		: Set H1C coefficient Level chang Function
// History			: First edition 						2013.04.18 Y.Shigeoka
//********************************************************************************
void	SetH1cMod( unsigned char	UcSetNum )
{
	unsigned long	UlGyrCof ;

	
	switch( UcSetNum ){
	case ( ACTMODE ):				// initial 
		/* enable setting */
		/* Zoom Step */
		UlGyrCof	= ClDiCof[ 0 ] ;
			
		UcH1LvlMod = 0 ;
		
		// Limit value Value Setting
		RamWrite32A( gxl2a_2, MINLMT ) ;		/* 0x1860 L-Limit */
		RamWrite32A( gxl2b_2, MAXLMT ) ;		/* 0x1861 H-Limit */

		RamWrite32A( gyl2a_2, MINLMT ) ;		/* 0x1960 L-Limit */
		RamWrite32A( gyl2b_2, MAXLMT ) ;		/* 0x1961 H-Limit */

		RamWrite32A( gxh1c_2, UlGyrCof ) ;		/* 0x18A2 Base Coef */
		RamWrite32A( gxl2c_2, CHGCOEF ) ;		/* 0x1862 Change coefficient gain */

		RamWrite32A( gyh1c_2, UlGyrCof ) ;		/* 0x19A2 Base Coef */
		RamWrite32A( gyl2c_2, CHGCOEF ) ;		/* 0x1962 Change coefficient gain */
		
		RegWriteA( WG_LSEL,	0x51 );				// 0x017B Select signal 
		RegWriteA( WG_HCHR,	0x04 );				// 0x017A Select signal 
		break ;
		
	case( S2MODE ):				// cancel lvl change mode 
		RegWriteA( WG_LSEL,	0x00 );				// 0x017B Select signal 
		RegWriteA( WG_HCHR,	0x00 );				// 0x017A Select signal 

		RamWrite32A( gxh1c_2, S2COEF ) ;		/* 0x18A2 Base Coef */
		RamWrite32A( gyh1c_2, S2COEF ) ;		/* 0x19A2 Base Coef */
		break ;
		
	default :
		/* Zoom Step */
		if(UcSetNum > (COEFTBL - 1))
			UcSetNum = (COEFTBL -1) ;			/* ãÀðCOEFTBL-1ÉÝè·é */

		UlGyrCof	= ClDiCof[ UcSetNum ] ;

		UcH1LvlMod = UcSetNum ;
			
		RamWrite32A( gxh1c_2, UlGyrCof ) ;		/* 0x18A2 Base Coef */
		RamWrite32A( gyh1c_2, UlGyrCof ) ;		/* 0x19A2 Base Coef */
		
		RegWriteA( WG_LSEL,	0x51 );				// 0x017B Select signal 
		RegWriteA( WG_HCHR,	0x04 );				// 0x017A Select signal 
		break ;
	}
}
#endif

//********************************************************************************
// Function Name 	: RdFwVr
// Retun Value		: Firmware version
// Argment Value	: NON
// Explanation		: Read Fw Version Function
// History			: First edition 						2013.05.07 Y.Shigeoka
//********************************************************************************
unsigned short	RdFwVr( void )
{
	return( FW_VER ) ;
}
