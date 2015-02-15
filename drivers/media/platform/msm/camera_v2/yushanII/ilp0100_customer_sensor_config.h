/*
******************************************************************************
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
********************************************************************************/
/*!
 * \file	ilp0100_customer_sensor_config.h
 * \brief	declaration of Ilp0100 structures that will be used for init and setup
			for current project
 * \author	sheena jain
 */

#ifndef ILP0100_CUSTOMER_SENSOR_CONFIG_H_
#define ILP0100_CUSTOMER_SENSOR_CONFIG_H_

#include "ilp0100_ST_definitions.h"

/* Example file */

/*  Structure containing All parameters for Ilp0100 depending on setup, but not sensor  */
/*  Initialization phase. */
#ifdef ST_SPECIFIC
extern Ilp0100_structInit SystemInitStruct;
extern Ilp0100_structFrameFormat SystemFrameFormat;

extern Ilp0100_structDefcorConfig DefcorConfig;
extern Ilp0100_structDefcorParams DefcorParams;
extern Ilp0100_structChannelOffsetConfig ChannelOffsetConfig;
extern Ilp0100_structChannelOffsetParams ChannelOffsetParams;
extern Ilp0100_structGlaceConfig GlaceConfig;
extern Ilp0100_structHistConfig HistConfig;
extern Ilp0100_structHdrMergeConfig HdrMergeConfig;
extern Ilp0100_structHdrMergeParams HdrMergeParams;
extern Ilp0100_structClsConfig ClsConfig;
extern Ilp0100_structClsParams ClsParams;
extern Ilp0100_structToneMappingConfig ToneMappingConfig;
extern Ilp0100_structToneMappingParams ToneMappingParams;
extern Ilp0100_structFrameParams FrameParams;

extern const Ilp0100_structInit PrimarySensor102Mhz1LaneInitStruct;
extern const Ilp0100_structInit PrimarySensor102Mhz2LaneInitStruct;
extern const Ilp0100_structInit PrimarySensor63Mhz4LaneInitStruct;

extern const Ilp0100_structInit SecondarySensor102Mhz1LaneInitStruct;
extern const Ilp0100_structInit SecondarySensor102Mhz2LaneInitStruct;
extern const Ilp0100_structInit SecondarySensor63Mhz4LaneInitStruct;

extern const Ilp0100_structInit PrimarySensor252Mhz2LaneInitStruct;
extern const Ilp0100_structInit SecondarySensor252Mhz2LaneInitStruct;

extern const Ilp0100_structInit PrimarySensor612MhzInitStruct;
extern const Ilp0100_structInit SecondarySensor612MhzInitStruct;

extern const Ilp0100_structSensorParams Ilp0100AlishanParams;
extern const Ilp0100_structSensorParams Ilp0100OVTParams;

extern const Ilp0100_structFrameFormat Ilp0100955Sensor5MPFormat;
extern const Ilp0100_structFrameFormat Ilp0100953Sensor5MPFormat;
extern const Ilp0100_structFrameFormat Ilp0100953Sensor4MPFormat;
extern const Ilp0100_structFrameFormat Ilp0100953Sensor3MPFormat;
extern const Ilp0100_structFrameFormat Ilp0100953Sensor2MPFormat;
extern const Ilp0100_structFrameFormat Ilp0100953Sensor1080pFormat;
extern const Ilp0100_structFrameFormat Ilp0100953SensorVGAFormat;

extern const Ilp0100_structFrameFormat Ilp0100HDRMode5MPFormat;
extern const Ilp0100_structFrameFormat Ilp0100HDRMode4MPFormat;
extern const Ilp0100_structFrameFormat Ilp0100HDRMode3MPFormat;
extern const Ilp0100_structFrameFormat Ilp0100HDRMode2MPFormat;
extern const Ilp0100_structFrameFormat Ilp0100HDRMode1080pFormat;
extern const Ilp0100_structFrameFormat Ilp0100HDRModeVGAFormat;

extern const Ilp0100_structFrameFormat Ilp0100AlishanFullFrameFormat;
extern const Ilp0100_structFrameFormat Ilp0100AlishanFullFrameFormat4x3;
extern const Ilp0100_structFrameFormat Ilp0100AlishanQuadFrameFormat;
extern const Ilp0100_structFrameFormat Ilp0100AlishanQuadFrameFormat4x3;
extern const Ilp0100_structFrameFormat Ilp0100Alishan1080pFrameFormat;
extern const Ilp0100_structFrameFormat Ilp0100Alishan720pFrameFormat;

extern const Ilp0100_structFrameFormat Ilp0100AlishanHDRFullFrameFormat;
extern const Ilp0100_structFrameFormat Ilp0100AlishanHDRFullFrameFormat4x3;
extern const Ilp0100_structFrameFormat Ilp0100AlishanHDRQuadFrameFormat;
extern const Ilp0100_structFrameFormat Ilp0100AlishanHDRQuadFrameFormat4x3;
extern const Ilp0100_structFrameFormat Ilp0100AlishanHDR1080pFrameFormat;
extern const Ilp0100_structFrameFormat Ilp0100AlishanHDR720pFrameFormat;

extern const Ilp0100_structFrameFormat Ilp0100XxxxFullFrameFormat;

#endif /*ST_SPECIFIC*/


#endif /* ILP0100_CUSTOMER_SENSOR_CONFIG_H_ */
