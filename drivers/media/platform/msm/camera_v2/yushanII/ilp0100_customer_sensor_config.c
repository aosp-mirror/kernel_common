/*******************************************************************************
################################################################################
#                             (C) STMicroelectronics 2012
#
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License version 2 and only version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
#------------------------------------------------------------------------------
#                             Imaging Division
################################################################################
File Name:		ilp0100_customer_sensor_config.c
Author:			author name
Description:	declaration of Ilp0100 structures that will be used for init and setup
				for current project
********************************************************************************/
/*!
 * \file	ilp0100_customer_sensor_config.c
 * \brief	declaration of Ilp0100 structures that will be used for init and setup
			for current project
 * \author	sheena jain
 */

#include "ilp0100_customer_sensor_config.h"

/*  Structure containing All parameters for Ilp0100 depending on setup, but not sensor  */
/*  Initialization phase. */
#ifdef ST_SPECIFIC
Ilp0100_structInit SystemInitStruct={
	.NumberOfLanes		 = 1,
	.uwPixelFormat		 = RAW_10,
	.BitRate 			 = 256000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_0,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

Ilp0100_structFrameFormat SystemFrameFormat={
	.ActiveLineLengthPixels		= 2592,
	.ActiveFrameLengthLines		= 1944,
	.LineLengthPixels			= 4000,
	.FrameLengthLines			= 2000,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2592,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11,
	.Hoffset					= 0,
	.Voffset					= 0
};

Ilp0100_structDefcorConfig DefcorConfig={
	.Mode						= SINGLET_AND_COUPLET
};

Ilp0100_structDefcorParams DefcorParams={
	.SingletThreshold			=	11,  //  1
	.CoupletThreshold			=	200, // 33
	.WhiteStrength				=	15, //  22
	.BlackStrength				=	15  //  21
};

Ilp0100_structChannelOffsetConfig ChannelOffsetConfig={
		.Enable 				= TRUE
};

Ilp0100_structChannelOffsetParams ChannelOffsetParams={
		.SensorPedestalGreenRed 				= 50,
		.SensorPedestalRed						=50,
		.SensorPedestalBlue						=50,
		.SensorPedestalGreenBlue				=50
};


Ilp0100_structGlaceConfig GlaceConfig={
		.Enable 						= TRUE,
		.RoiHStart						= 0,
		.RoiVStart						= 0,
		.RoiHBlockSize					= 0,
		.RoiVBlockSize					= 0,
		.RoiHNumberOfBlocks				=8,
		.RoiHNumberOfBlocks				=6,
		.SaturationLevelRed				=0,
		.SaturationLevelGreen			=0,
		.SaturationLevelBlue			=0
};

Ilp0100_structHistConfig HistConfig={
		.Mode 								= REDGREENBLUE,
		.RoiXOffset							= 0,
		.RoiYOffset							= 0,
		.RoiXSize							= 0,
		.RoiYSize							= 0,
		.YConversionFactorGreenRed			=0,
		.YConversionFactorRed				=0,
		.YConversionFactorBlue				=0,
		.YConversionFactorGreenBlue			=0,
};

Ilp0100_structHdrMergeConfig HdrMergeConfig={
		.Mode 								= OUTPUT_SHORT_ONLY,

};

Ilp0100_structHdrMergeParams HdrMergeParams={
		.Method 								= USE_KNEE_POINTS,
		.ImageCodes								=MAX_MACRO_PIXEL
};

Ilp0100_structClsConfig ClsConfig={
		.Enable 								= TRUE,

};

Ilp0100_structClsParams ClsParams={
		.BowlerCornerGain						= 30,
		.ColorTempKelvin						=15
};

Ilp0100_structToneMappingParams ToneMappingParams={
		.Strength								=50
};

Ilp0100_structToneMappingConfig ToneMappingConfig={
		.Enable 								= TRUE,
		.UserDefinedCurveEnable					=TRUE
};

Ilp0100_structFrameParams FrameParams={
		.ExposureTime 						= 3,
		.AnalogGainCodeGreen				= 1,
		.AnalogGainCodeRed					= 1,
		.AnalogGainCodeBlue					= 1,
		.DigitalGainCodeGreen				= 2,
		.DigitalGainCodeRed					= 2,
		.DigitalGainCodeBlue				= 2,
};

const Ilp0100_structFrameFormat Ilp0100HDRMode5MPFormat={
	.ActiveLineLengthPixels		= 2592,
	.ActiveFrameLengthLines		= 1944,
	.LineLengthPixels			= 4000,
	.FrameLengthLines			= 2000,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2592,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100HDRMode4MPFormat={
	.ActiveLineLengthPixels		= 2688,
	.ActiveFrameLengthLines		= 1520,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1600,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2688,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100HDRMode3MPFormat={
	.ActiveLineLengthPixels		= 2048,
	.ActiveFrameLengthLines		= 1536,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1600,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2048,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100HDRMode2MPFormat={
	.ActiveLineLengthPixels		= 1600,
	.ActiveFrameLengthLines		= 1200,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1300,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 1600,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100HDRMode1080pFormat={
	.ActiveLineLengthPixels		= 1920,
	.ActiveFrameLengthLines		= 1080,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1100,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 1920,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100HDRModeVGAFormat={
	.ActiveLineLengthPixels		= 640,
	.ActiveFrameLengthLines		= 480,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 600,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 640,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structInit PrimarySensor102Mhz1LaneInitStruct={
	.NumberOfLanes		 = 1,
	.uwPixelFormat		 = RAW_10,
	.BitRate 			 = 102000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_0,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

const Ilp0100_structInit PrimarySensor102Mhz2LaneInitStruct={
	.NumberOfLanes		 = 2,
	.uwPixelFormat		 = RAW_10,
	.BitRate 			 = 102000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_0,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

const Ilp0100_structInit PrimarySensor102Mhz4LaneInitStruct={
	.NumberOfLanes		 = 4,
	.uwPixelFormat		 = RAW_10,
	.BitRate 			 = 102000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_0,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

const Ilp0100_structInit SecondarySensor102Mhz1LaneInitStruct={
	.NumberOfLanes		 = 1,
	.uwPixelFormat		 = RAW_10,
	.BitRate 			 = 102000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_1,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

const Ilp0100_structInit SecondarySensor102Mhz2LaneInitStruct={
	.NumberOfLanes		 = 2,
	.uwPixelFormat		 = RAW_10,
	.BitRate 			 = 102000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_1,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

const Ilp0100_structInit SecondarySensor102Mhz4LaneInitStruct={
	.NumberOfLanes		 = 4,
	.uwPixelFormat		 = RAW_10,
	.BitRate 			 = 102000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_1,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

const Ilp0100_structInit PrimarySensor252Mhz2LaneInitStruct={
	.NumberOfLanes		 = 2,
	.uwPixelFormat		 = RAW_10,
	.BitRate 			 = 252000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_0,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

const Ilp0100_structInit SecondarySensor252Mhz2LaneInitStruct={
	.NumberOfLanes		 = 2,
	.uwPixelFormat		 = RAW_10,
	.BitRate 			 = 252000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_1,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

const Ilp0100_structInit PrimarySensor612MhzInitStruct={
	.NumberOfLanes		 = 4,
	.uwPixelFormat		 = RAW_10,
	.BitRate			 = 612000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_0,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

const Ilp0100_structInit SecondarySensor612MhzInitStruct={
	.NumberOfLanes		 = 1, // Only 1/2 MIPI lane(s) available for secondary camera interface
	.BitRate			 = 612000000,
	.uwPixelFormat		 = RAW_10,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_1,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};


/*  Structure containing sensor-dependant parameter. */
/*  but not streaming mode-dependant parameters. */
const Ilp0100_structSensorParams Ilp0100AlishanParams={
	/*  Max sensor active line length (in full frame) */
	.FullActivePixels	= 2688, /*  HSize */
	/*  Min sensor line length (Active line+Line blanking) */
	.MinLineLength		= 2888,
	/*  Max sensor active frame length (in full frame) */
	.FullActiveLines	= 1520,	/*  VSize; */
	/* First pixel color */
	.PixelOrder			= GR,
	.StatusNbLines		= 2
};

/* Sensor information for sensor conencted to secondary interface */
const Ilp0100_structSensorParams Ilp0100OVTParams={
	/*  Max sensor active line length (in full frame) */
	.FullActivePixels 	= 1928, /*  HSize */
	/*  Min sensor line length (Active line+Line blanking) */
	.MinLineLength 		= 2128,
	/*  Max sensor active frame length (in full frame) */
	.FullActiveLines 	= 1088,  /*  VSize; */
	/* First pixel color */
	.PixelOrder			= GR,
	.StatusNbLines		= 0
};


/*  Structure containing Frame formats modes */
const Ilp0100_structFrameFormat Ilp0100955Sensor5MPFormat={
	.ActiveLineLengthPixels		= 2600,
	.ActiveFrameLengthLines		= 1952,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1974,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 1600,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100953Sensor5MPFormat={
	.ActiveLineLengthPixels		= 2592,
	.ActiveFrameLengthLines		= 1944,
	.LineLengthPixels			= 4000,
	.FrameLengthLines			= 2000,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2592,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100953Sensor4MPFormat={
	.ActiveLineLengthPixels		= 2688,
	.ActiveFrameLengthLines		= 1520,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1600,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2688,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100953Sensor3MPFormat={
	.ActiveLineLengthPixels		= 2048,
	.ActiveFrameLengthLines		= 1536,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1600,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2048,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100953Sensor2MPFormat={
	.ActiveLineLengthPixels		= 1600,
	.ActiveFrameLengthLines		= 1200,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1300,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 1600,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100953Sensor1080pFormat={
	.ActiveLineLengthPixels		= 1920,
	.ActiveFrameLengthLines		= 1080,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1100,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 1920,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100953SensorVGAFormat={
	.ActiveLineLengthPixels		= 640,
	.ActiveFrameLengthLines		= 480,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 600,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 640,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100AlishanFullFrameFormat={
	.ActiveLineLengthPixels		= 2688,
	.ActiveFrameLengthLines		= 1520,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 1592,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2688,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100AlishanFullFrameFormat4x3={
	.ActiveLineLengthPixels		= 2032,
	.ActiveFrameLengthLines		= 1520,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 1592,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2032,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100AlishanQuadFrameFormat={
	.ActiveLineLengthPixels		= 1344,
	.ActiveFrameLengthLines		= 760,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 900,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 1344,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100AlishanQuadFrameFormat4x3={
	.ActiveLineLengthPixels		= 1024,
	.ActiveFrameLengthLines		= 760,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 900,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels		= 1024,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode 					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100Alishan1080pFrameFormat={
	.ActiveLineLengthPixels		= 1936,
	.ActiveFrameLengthLines		= 1088,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 1200,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels		= 1936,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 2,
	.VScaling					= 2,
	.Binning					= 0x22
};
/*  Structure containing Frame formats modes */
const Ilp0100_structFrameFormat Ilp0100Alishan720pFrameFormat={
	.ActiveLineLengthPixels		= 1296,
	.ActiveFrameLengthLines		= 728,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 900,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels		= 1296,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 2,
	.VScaling					= 2,
	.Binning					= 0x22
};

const Ilp0100_structFrameFormat Ilp0100XxxxFullFrameFormat={
	.ActiveLineLengthPixels		= 2688,
	.ActiveFrameLengthLines		= 1520,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1300,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels		= 2688,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_VFLIP_AND_MIRROR,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100AlishanHDRFullFrameFormat={
	.ActiveLineLengthPixels		= 2688,
	.ActiveFrameLengthLines		= 1520,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 1800,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2688,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100AlishanHDRFullFrameFormat4x3={
	.ActiveLineLengthPixels		= 2032,
	.ActiveFrameLengthLines		= 1520,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 1800,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2032,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100AlishanHDRQuadFrameFormat={
	.ActiveLineLengthPixels		= 1344,
	.ActiveFrameLengthLines		= 760,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 1000,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 1344,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100AlishanHDRQuadFrameFormat4x3={
	.ActiveLineLengthPixels		= 1024,
	.ActiveFrameLengthLines		= 760,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 1000,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels		= 1024,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode 					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100AlishanHDR1080pFrameFormat={
	.ActiveLineLengthPixels		= 1936,
	.ActiveFrameLengthLines		= 1088,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 1400,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels		= 1936,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 2,
	.VScaling					= 2,
	.Binning					= 0x22
};
/*  Structure containing Frame formats modes */
const Ilp0100_structFrameFormat Ilp0100AlishanHDR720pFrameFormat={
	.ActiveLineLengthPixels		= 1296,
	.ActiveFrameLengthLines		= 728,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 1200,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels		= 1296,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 2,
	.VScaling					= 2,
	.Binning					= 0x22
};

#else
Ilp0100_structInit SystemInitStruct={
	.NumberOfLanes		 = 1,
	.uwPixelFormat		 = RAW_10,
	.BitRate 			 = 256000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_0,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

Ilp0100_structFrameFormat SystemFrameFormat={
	.ActiveLineLengthPixels		= 2592,
	.ActiveFrameLengthLines		= 1944,
	.LineLengthPixels			= 4000,
	.FrameLengthLines			= 2000,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2592,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11,
	.Hoffset					= 0,
	.Voffset					= 0
};

Ilp0100_structDefcorConfig DefcorConfig={
	.Mode						= SINGLET_AND_COUPLET
};

Ilp0100_structDefcorParams DefcorParams={
	.SingletThreshold			=	11,  //  1
	.CoupletThreshold			=	200, // 33
	.WhiteStrength				=	15, //  22
	.BlackStrength				=	15  //  21
};

Ilp0100_structChannelOffsetConfig ChannelOffsetConfig={
		.Enable 				= TRUE
};

Ilp0100_structChannelOffsetParams ChannelOffsetParams={
		.SensorPedestalGreenRed 				= 50,
		.SensorPedestalRed						=50,
		.SensorPedestalBlue						=50,
		.SensorPedestalGreenBlue				=50
};


Ilp0100_structGlaceConfig GlaceConfig={
		.Enable 						= TRUE,
		.RoiHStart						= 0,
		.RoiVStart						= 0,
		.RoiHBlockSize					= 0,
		.RoiVBlockSize					= 0,
		.RoiHNumberOfBlocks				=8,
		.RoiHNumberOfBlocks				=6,
		.SaturationLevelRed				=0,
		.SaturationLevelGreen			=0,
		.SaturationLevelBlue			=0
};

Ilp0100_structHistConfig HistConfig={
		.Mode 								= REDGREENBLUE,
		.RoiXOffset							= 0,
		.RoiYOffset							= 0,
		.RoiXSize							= 0,
		.RoiYSize							= 0,
		.YConversionFactorGreenRed			=0,
		.YConversionFactorRed				=0,
		.YConversionFactorBlue				=0,
		.YConversionFactorGreenBlue			=0,
};

Ilp0100_structHdrMergeConfig HdrMergeConfig={
		.Mode 								= OUTPUT_SHORT_ONLY,

};

Ilp0100_structHdrMergeParams HdrMergeParams={
		.Method 								= USE_KNEE_POINTS,
		.ImageCodes								=MAX_MACRO_PIXEL
};


Ilp0100_structClsConfig ClsConfig={
		.Enable 								= TRUE,

};

Ilp0100_structClsParams ClsParams={
		.BowlerCornerGain						= 30,
		.ColorTempKelvin						=15
};

Ilp0100_structToneMappingParams ToneMappingParams={
		.Strength								=50
};


Ilp0100_structToneMappingConfig ToneMappingConfig={
		.Enable 								= TRUE,
		.UserDefinedCurveEnable					=TRUE
};

Ilp0100_structFrameParams FrameParams={
		.ExposureTime 						= 3,
		.AnalogGainCodeGreen				= 1,
		.AnalogGainCodeRed					= 1,
		.AnalogGainCodeBlue					= 1,
		.DigitalGainCodeGreen				= 2,
		.DigitalGainCodeRed					= 2,
		.DigitalGainCodeBlue				= 2,
};

const Ilp0100_structFrameFormat Ilp0100HDRMode5MPFormat={
	.ActiveLineLengthPixels		= 2592,
	.ActiveFrameLengthLines		= 1944,
	.LineLengthPixels			= 4000,
	.FrameLengthLines			= 2000,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2592,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100HDRMode4MPFormat={
	.ActiveLineLengthPixels		= 2688,
	.ActiveFrameLengthLines		= 1520,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1600,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2688,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100HDRMode3MPFormat={
	.ActiveLineLengthPixels		= 2048,
	.ActiveFrameLengthLines		= 1536,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1600,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2048,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100HDRMode2MPFormat={
	.ActiveLineLengthPixels		= 1600,
	.ActiveFrameLengthLines		= 1200,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1300,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 1600,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100HDRMode1080pFormat={
	.ActiveLineLengthPixels		= 1920,
	.ActiveFrameLengthLines		= 1080,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1100,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 1920,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100HDRModeVGAFormat={
	.ActiveLineLengthPixels		= 640,
	.ActiveFrameLengthLines		= 480,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 600,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 640,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structInit PrimarySensor102Mhz1LaneInitStruct={
	.NumberOfLanes		 = 1,
	.uwPixelFormat		 = RAW_10,
	.BitRate 			 = 102000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_0,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

const Ilp0100_structInit PrimarySensor102Mhz2LaneInitStruct={
	.NumberOfLanes		 = 2,
	.uwPixelFormat		 = RAW_10,
	.BitRate 			 = 102000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_0,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

const Ilp0100_structInit PrimarySensor102Mhz4LaneInitStruct={
	.NumberOfLanes		 = 4,
	.uwPixelFormat		 = RAW_10,
	.BitRate 			 = 102000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_0,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

const Ilp0100_structInit SecondarySensor102Mhz1LaneInitStruct={
	.NumberOfLanes		 = 1,
	.uwPixelFormat		 = RAW_10,
	.BitRate 			 = 102000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_1,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

const Ilp0100_structInit SecondarySensor102Mhz2LaneInitStruct={
	.NumberOfLanes		 = 2,
	.uwPixelFormat		 = RAW_10,
	.BitRate 			 = 102000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_1,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

const Ilp0100_structInit SecondarySensor102Mhz4LaneInitStruct={
	.NumberOfLanes		 = 4,
	.uwPixelFormat		 = RAW_10,
	.BitRate 			 = 102000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_1,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

const Ilp0100_structInit PrimarySensor252Mhz2LaneInitStruct={
	.NumberOfLanes		 = 2,
	.uwPixelFormat		 = RAW_10,
	.BitRate 			 = 252000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_0,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

const Ilp0100_structInit SecondarySensor252Mhz2LaneInitStruct={
	.NumberOfLanes		 = 2,
	.uwPixelFormat		 = RAW_10,
	.BitRate 			 = 252000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_1,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

const Ilp0100_structInit PrimarySensor612MhzInitStruct={
	.NumberOfLanes		 = 4,
	.uwPixelFormat		 = RAW_10,
	.BitRate			 = 612000000,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_0,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};

const Ilp0100_structInit SecondarySensor612MhzInitStruct={
	.NumberOfLanes		 = 1, // Only 1/2 MIPI lane(s) available for secondary camera interface
	.BitRate			 = 612000000,
	.uwPixelFormat		 = RAW_10,
	.ExternalClock		 = 24000000,
	.ClockUsed			 = ILP0100_CLOCK,
	.UsedSensorInterface = SENSOR_1,
	.IntrEnablePin1		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN0,
	.IntrEnablePin2		 = ENABLE_RECOMMENDED_DEBUG_INTR_PIN1
};


/*  Structure containing sensor-dependant parameter. */
/*  but not streaming mode-dependant parameters. */
const Ilp0100_structSensorParams Ilp0100AlishanParams={
	/*  Max sensor active line length (in full frame) */
	.FullActivePixels	= 2688, /*  HSize */
	/*  Min sensor line length (Active line+Line blanking) */
	.MinLineLength		= 2888,
	/*  Max sensor active frame length (in full frame) */
	.FullActiveLines	= 1520,	/*  VSize; */
	/* First pixel color */
	.PixelOrder			= GR,
	.StatusNbLines		= 2
};

/* Sensor information for sensor conencted to secondary interface */
const Ilp0100_structSensorParams Ilp0100OVTParams={
	/*  Max sensor active line length (in full frame) */
	.FullActivePixels 	= 1928, /*  HSize */
	/*  Min sensor line length (Active line+Line blanking) */
	.MinLineLength 		= 2128,
	/*  Max sensor active frame length (in full frame) */
	.FullActiveLines 	= 1088,  /*  VSize; */
	/* First pixel color */
	.PixelOrder			= GR,
	.StatusNbLines		= 0
};



/* Sensor information for sensor conencted to secondary interface */
const Ilp0100_structSensorParams Ilp0100XxxxParams={
	/*  Max sensor active line length (in full frame) */
	.FullActivePixels 	= 1248, /*  HSize */
	/*  Min sensor line length (Active line+Line blanking) */
	.MinLineLength 		= 1724,
	/*  Max sensor active frame length (in full frame) */
	.FullActiveLines 	= 728,  /*  VSize; */
	/* First pixel color */
	.PixelOrder			= GR
};


/*  Structure containing Frame formats modes */
const Ilp0100_structFrameFormat Ilp0100955Sensor5MPFormat={
	.ActiveLineLengthPixels		= 2600,
	.ActiveFrameLengthLines		= 1952,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1974,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 1600,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100953Sensor5MPFormat={
	.ActiveLineLengthPixels		= 2592,
	.ActiveFrameLengthLines		= 1944,
	.LineLengthPixels			= 4000,
	.FrameLengthLines			= 2000,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2592,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100953Sensor4MPFormat={
	.ActiveLineLengthPixels		= 2688,
	.ActiveFrameLengthLines		= 1520,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1600,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2688,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100953Sensor3MPFormat={
	.ActiveLineLengthPixels		= 2048,
	.ActiveFrameLengthLines		= 1536,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1600,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2048,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100953Sensor2MPFormat={
	.ActiveLineLengthPixels		= 1600,
	.ActiveFrameLengthLines		= 1200,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1300,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 1600,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100953Sensor1080pFormat={
	.ActiveLineLengthPixels		= 1920,
	.ActiveFrameLengthLines		= 1080,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1100,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 1920,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100953SensorVGAFormat={
	.ActiveLineLengthPixels		= 640,
	.ActiveFrameLengthLines		= 480,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 600,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 640,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100AlishanFullFrameFormat={
	.ActiveLineLengthPixels		= 2688,
	.ActiveFrameLengthLines		= 1520,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 1592,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2688,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100AlishanFullFrameFormat4x3={
	.ActiveLineLengthPixels		= 2032,
	.ActiveFrameLengthLines		= 1520,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 1592,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2032,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100AlishanQuadFrameFormat={
	.ActiveLineLengthPixels		= 1344,
	.ActiveFrameLengthLines		= 760,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 900,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 1344,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100AlishanQuadFrameFormat4x3={
	.ActiveLineLengthPixels		= 1024,
	.ActiveFrameLengthLines		= 760,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 900,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels		= 1024,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode 					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100Alishan1080pFrameFormat={
	.ActiveLineLengthPixels		= 1936,
	.ActiveFrameLengthLines		= 1088,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 1200,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels		= 1936,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 2,
	.VScaling					= 2,
	.Binning					= 0x22
};
/*  Structure containing Frame formats modes */
const Ilp0100_structFrameFormat Ilp0100Alishan720pFrameFormat={
	.ActiveLineLengthPixels		= 1296,
	.ActiveFrameLengthLines		= 728,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 900,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels		= 1296,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 2,
	.VScaling					= 2,
	.Binning					= 0x22
};

const Ilp0100_structFrameFormat Ilp0100XxxxFullFrameFormat={
	.ActiveLineLengthPixels		= 2688,
	.ActiveFrameLengthLines		= 1520,
	.LineLengthPixels			= 3488,
	.FrameLengthLines			= 1300,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels		= 2688,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= HDR_NONE,
	.uwOutputPixelFormat		= RAW_10,
	.ImageOrientation			= ORIENTATION_VFLIP_AND_MIRROR,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100AlishanHDRFullFrameFormat={
	.ActiveLineLengthPixels		= 2688,
	.ActiveFrameLengthLines		= 1520,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 1800,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2688,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100AlishanHDRFullFrameFormat4x3={
	.ActiveLineLengthPixels		= 2032,
	.ActiveFrameLengthLines		= 1520,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 1800,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 2032,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100AlishanHDRQuadFrameFormat={
	.ActiveLineLengthPixels		= 1344,
	.ActiveFrameLengthLines		= 760,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 1000,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels 	= 1344,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100AlishanHDRQuadFrameFormat4x3={
	.ActiveLineLengthPixels		= 1024,
	.ActiveFrameLengthLines		= 760,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 1000,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels		= 1024,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode 					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 1,
	.VScaling					= 1,
	.Binning					= 0x11
};

const Ilp0100_structFrameFormat Ilp0100AlishanHDR1080pFrameFormat={
	.ActiveLineLengthPixels		= 1936,
	.ActiveFrameLengthLines		= 1088,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 1400,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels		= 1936,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 2,
	.VScaling					= 2,
	.Binning					= 0x22
};
/*  Structure containing Frame formats modes */
const Ilp0100_structFrameFormat Ilp0100AlishanHDR720pFrameFormat={
	.ActiveLineLengthPixels		= 1296,
	.ActiveFrameLengthLines		= 728,
	.LineLengthPixels			= 2888,
	.FrameLengthLines			= 1200,
	.StatusLinesOutputted		= TRUE,
	.StatusLineLengthPixels		= 1296,
	.StatusNbLines				= 2,
	.MinInterframe				= 70,
	.AutomaticFrameParamsUpdate = TRUE,
	.HDRMode					= STAGGERED,
	.uwOutputPixelFormat		= RAW_12,
	.ImageOrientation			= ORIENTATION_NORMAL,
	.HScaling					= 2,
	.VScaling					= 2,
	.Binning					= 0x22
};


#endif /*ST_SPECIFIC*/
