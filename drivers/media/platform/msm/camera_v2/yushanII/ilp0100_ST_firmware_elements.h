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
********************************************************************************/
/*!
 * \file	ilp0100_ST_firmware_elements.h
 * \brief	This file lists all the firmware page elements
 * \author	sheena jain
 */
#ifndef ILP0100_ST_FWELEMENTS_H_

//ILP0100_ST_FWELEMENTS_H_
#define ILP0100_ST_FWELEMENTS_H_
typedef enum
{
  Bayer2Y_Flag_e_FALSE = 0,
  Bayer2Y_Flag_e_TRUE = 1,
} Bayer2Y_Flag_te;

typedef enum
{
  CRMTop_Flag_e_FALSE = 0,
  CRMTop_Flag_e_TRUE = 1,
} CRMTop_Flag_te;

typedef enum
{
  CSI2Rx_VirtualChannelMap_e_DATAPATH0 = 0,
  CSI2Rx_VirtualChannelMap_e_DATAPATH1 = 1,
  CSI2Rx_VirtualChannelMap_e_DATAPATH2 = 2,
  CSI2Rx_VirtualChannelMap_e_DATAPATH3 = 3,
} CSI2Rx_VirtualChannelMap_te;

typedef enum
{
  Duster_Flag_e_False = 0,
  Duster_Flag_e_True = 1,
} Duster_Flag_te;

typedef enum
{
  Duster_ScytheMode_e_ScytheMap = 0,
  Duster_ScytheMode_e_ScytheAndMap = 1,
  Duster_ScytheMode_e_BypassScythe = 2,
  Duster_ScytheMode_e_ForceScythe = 3,
} Duster_ScytheMode_te;

typedef enum
{
  Glace_Flag_e_FALSE = 0,
  Glace_Flag_e_TRUE = 1,
} Glace_Flag_te;

typedef enum
{
  HawkEyes_Flag_e_FALSE = 0,
  HawkEyes_Flag_e_TRUE = 1,
} HawkEyes_Flag_te;

typedef enum
{
  HawkEyes_MergeMode_e_KNEE_CONTROL = 0,
  HawkEyes_MergeMode_e_AVG_NormLG_NormST = 1,
  HawkEyes_MergeMode_e_MERGE_AND_LINEARIZE = 2,
} HawkEyes_MergeMode_te;

typedef enum
{
  HIF_Flag_e_FALSE = 0,
  HIF_Flag_e_TRUE = 1,
} HIF_Flag_te;

typedef enum
{
  HIF_ImageFormat_e_RAW8 = 8,
  HIF_ImageFormat_e_RAW10 = 10,
  HIF_ImageFormat_e_RAW12 = 12,
} HIF_ImageFormat_te;

typedef enum
{
  HIF_ImagePipe_e_NONE = 0,
  HIF_ImagePipe_e_LONG = 1,
  HIF_ImagePipe_e_SHORT = 2,
  HIF_ImagePipe_e_HDR = 3,
  HIF_ImagePipe_e_NONHDR = 4,
} HIF_ImagePipe_te;

typedef enum
{
  HIF_InputImageSource_e_SENSOR = 0,
  HIF_InputImageSource_e_RX_TP_COLOUR_BAR = 0x81,
  HIF_InputImageSource_e_RX_TP_GRAD_BAR = 0x82,
  HIF_InputImageSource_e_RX_TP_DIAG_GREY = 0x83,
  HIF_InputImageSource_e_RX_TP_PSEUDORANDOM = 0x84,
  HIF_InputImageSource_e_RX_TP_HOR_GREY = 0x85,
  HIF_InputImageSource_e_RX_TP_VERT_GREY = 0x86,
  HIF_InputImageSource_e_RX_TP_SOLID = 0x87,
} HIF_InputImageSource_te;

typedef enum
{
  HIF_ModeChange_e_IDLE = 0,
  HIF_ModeChange_e_REQUEST = 1,
  HIF_ModeChange_e_UPDATE_BUSY = 2,
  HIF_ModeChange_e_REQUEST_ACCEPTED = 3,
  HIF_ModeChange_e_LIMIT_ERROR = 4,
} HIF_ModeChange_te;

typedef enum
{
  HIF_ModeControl_e_IDLE = 0x01,
  HIF_ModeControl_e_STREAM = 0x02,
} HIF_ModeControl_te;

typedef enum
{
  HIF_ModeStatus_e_UNINIT = 0x00,
  HIF_ModeStatus_e_GOINGTOIDLE_FROMINIT = 0x30,
  HIF_ModeStatus_e_GOINGTOIDLE_FROMSTREAM = 0x31,
  HIF_ModeStatus_e_GOINGTOIDLE_FROMERROR = 0x34,
  HIF_ModeStatus_e_IDLE = 0x40,
  HIF_ModeStatus_e_GOINGTOSTREAM_FROMIDLE = 0x50,
  HIF_ModeStatus_e_STREAM = 0x60,
  HIF_ModeStatus_e_ERROR = 0xB0,
} HIF_ModeStatus_te;

typedef enum
{
  HIF_Orientation_e_NORMAL = 0,
  HIF_Orientation_e_HFLIP = 1,
  HIF_Orientation_e_VFLIP = 2,
  HIF_Orientation_e_HVFLIP = 3,
} HIF_Orientation_te;

typedef enum
{
  HIF_PixelOrder_e_GR = 0,
  HIF_PixelOrder_e_RG = 1,
  HIF_PixelOrder_e_BG = 2,
  HIF_PixelOrder_e_GB = 3,
  HIF_PixelOrder_e_UNINIT = 255,
} HIF_PixelOrder_te;

typedef enum
{
  HIF_SensorSelected_e_PRIMARYSENSOR = 0,
  HIF_SensorSelected_e_SECONDARYSENSOR = 1,
} HIF_SensorSelected_te;

typedef enum
{
  HIF_StatsHostAck_e_IDLE = 0,
  HIF_StatsHostAck_e_DONE = 1,
} HIF_StatsHostAck_te;

typedef enum
{
  HIF_SystemError_e_NONE = 0,
  HIF_SystemError_e_CSIRX_DATA_ERR = 0x10,
  HIF_SystemError_e_CSIRX_CHEKSUM_ERR = 0x11,
  HIF_SystemError_e_CSIRX_SYNC_PULSE_ERR = 0x13,
  HIF_SystemError_e_CSIRX_ECC_ERR = 0x14,
  HIF_SystemError_e_CSIRX_TIME_OUT_CH0_ERR = 0x15,
  HIF_SystemError_e_CSIRX_TIME_OUT_CH1_ERR = 0x16,
  HIF_SystemError_e_SMIA_UNPACK_0_OVERFLOW_ERR = 0x30,
  HIF_SystemError_e_SMIA_UNPACK_0_UNDERFLOW_ERR = 0x31,
  HIF_SystemError_e_SMIA_UNPACK_1_OVERFLOW_ERR = 0x32,
  HIF_SystemError_e_SMIA_UNPACK_1_UNDERFLOW_ERR = 0x33,
  HIF_SystemError_e_P2W_SYNC_FIFO_OVERFLOW_ERR = 0x34,
  HIF_SystemError_e_P2W_ASYNC_FIFO_OVERFLOW_ERR = 0x35,
  HIF_SystemError_e_P2W_LC_FIFO_OVERFLOW_ERR = 0x36,
  HIF_SystemError_e_HAWKEYE_STREAM_ERR = 0x39,
  HIF_SystemError_e_HAWKEYE_OVERFLOW_ERR = 0x3A,
  HIF_SystemError_e_HAWKEYE_UNDERFLOW_ERR = 0x3B,
  HIF_SystemError_e_TONEMAP_OVERFLOW_ERR = 0x3D,
  HIF_SystemError_e_TONEMAP_UNDERFLOW_ERR = 0x3E,
  HIF_SystemError_e_TONEMAP_RAM_NOT_EMPTY_ERR = 0x3F,
  HIF_SystemError_e_SHORT_FIRST_ERR = 0x53,
  HIF_SystemError_e_LONG_ONLY_ERR = 0x52,
  HIF_SystemError_e_P2W_LC_UFLOW_ERR = 0x65,
  HIF_SystemError_e_P2W_DATA_UFLOW_ERR = 0x64,
  HIF_SystemError_e_CSI2TX_DI_ERR = 0x62,
  HIF_SystemError_e_CSI2TX_LP_ERR = 0x61,
  HIF_SystemError_e_CSI2TX_SP_ERR = 0x60,
  HIF_SystemError_e_RX_COLOR_BAR_0_ERR = 0x40,
  HIF_SystemError_e_RX_COLOR_BAR_1_ERR = 0x41,
  HIF_SystemError_e_RX_COLOR_BAR_2_ERR = 0x42,
  HIF_SystemError_e_RX_COLOR_BAR_3_ERR = 0x43,
  HIF_SystemError_e_RX_COLOR_BAR_4_ERR = 0x44,
  HIF_SystemError_e_RX_COLOR_BAR_5_ERR = 0x45,
  HIF_SystemError_e_RX_COLOR_BAR_6_ERR = 0x46,
  HIF_SystemError_e_RX_COLOR_BAR_7_ERR = 0x47,
  HIF_SystemError_e_RXPHY_SOT_SOFT_DL1_ERR = 0x80,
  HIF_SystemError_e_RXPHY_SOT_HARD_DL1_ERR = 0x81,
  HIF_SystemError_e_RXPHY_EOT_DL1_ERR = 0x82,
  HIF_SystemError_e_RXPHY_CTRL_DL1_ERR = 0x84,
  HIF_SystemError_e_RXPHY_SOT_SOFT_DL2_ERR = 0x88,
  HIF_SystemError_e_RXPHY_SOT_HARD_DL2_ERR = 0x89,
  HIF_SystemError_e_RXPHY_EOT_DL2_ERR = 0x8A,
  HIF_SystemError_e_RXPHY_CTRL_DL2_ERR = 0x8C,
  HIF_SystemError_e_RXPHY_SOT_SOFT_DL3_ERR = 0x90,
  HIF_SystemError_e_RXPHY_SOT_HARD_DL3_ERR = 0x91,
  HIF_SystemError_e_RXPHY_EOT_DL3_ERR = 0x92,
  HIF_SystemError_e_RXPHY_CTRL_DL3_ERR = 0x94,
  HIF_SystemError_e_RXPHY_EOT_DL4_ERR = 0x9A,
  HIF_SystemError_e_RXPHY_CTRL_DL4_ERR = 0x9C,
  HIF_SystemError_e_RXPHY_SOT_HARD_DL4_ERR = 0x99,
  HIF_SystemError_e_RXPHY_SOT_SOFT_DL4_ERR = 0x98,
  HIF_SystemError_e_TXPHY_CTRL_DL4_ERR = 0x23,
  HIF_SystemError_e_TXPHY_CTRL_DL3_ERR = 0x22,
  HIF_SystemError_e_TXPHY_CTRL_DL2_ERR = 0x21,
  HIF_SystemError_e_TXPHY_CTRL_DL1_ERR = 0x20,
  HIF_SystemError_e_MODE_TRANSITION_ERR = 0x70,
  HIF_SystemError_e_HAWKEYE_RAM_NOT_EMPTY_ERR = 0x3C,
  HIF_SystemError_e_RXPHY_ESC_DL1_ERR = 0x83,
  HIF_SystemError_e_RXPHY_ESC_DL2_ERR = 0x8B,
  HIF_SystemError_e_RXPHY_ESC_DL3_ERR = 0x93,
  HIF_SystemError_e_RXPHY_ESC_DL4_ERR = 0x9B,
  HIF_SystemError_e_SENSOR_WORD_COUNT_MISMATCH = 0xA0,
  HIF_SystemError_e_WRONG_RUBIKCALIB_BUFFER = 0xB0,
} HIF_SystemError_te;

typedef enum
{
  Histogram_Flag_e_FALSE = 0,
  Histogram_Flag_e_TRUE = 1,
} Histogram_Flag_te;

typedef enum
{
  InputInterface_PixelOrder_e_GRBG = 0,
  InputInterface_PixelOrder_e_RGGB = 1,
  InputInterface_PixelOrder_e_BGGR = 2,
  InputInterface_PixelOrder_e_GBRG = 3,
  InputInterface_PixelOrder_e_UNINIT = 255,
} InputInterface_PixelOrder_te;

typedef enum
{
  InputInterface_StatusLine_e_DISABLED = 0,
  InputInterface_StatusLine_e_ENABLED = 1,
  InputInterface_StatusLine_e_ENABLED_BUT_OVERRIDEN = 2,
} InputInterface_StatusLine_te;

typedef enum
{
  Pedestal_Flag_e_FALSE = 0,
  Pedestal_Flag_e_TRUE = 1,
} Pedestal_Flag_te;

typedef enum
{
  Rubik_Flag_e_FALSE = 0,
  Rubik_Flag_e_TRUE = 1,
} Rubik_Flag_te;

typedef enum
{
  RubikTop_BufferState_e_NONE = 0,
  RubikTop_BufferState_e_DOWNLOADED = 1,
  RubikTop_BufferState_e_PARSED = 2,
} RubikTop_BufferState_te;

typedef enum
{
  RubikTop_Mode_e_MANUAL_GRID0 = 0,
  RubikTop_Mode_e_MANUAL_GRID1 = 1,
  RubikTop_Mode_e_MANUAL_GRID2 = 2,
  RubikTop_Mode_e_MANUAL_GRID3 = 3,
  RubikTop_Mode_e_AUTO = 4,
} RubikTop_Mode_te;

typedef enum
{
  ToneMap_Flag_e_FALSE = 0,
  ToneMap_Flag_e_TRUE = 1,
} ToneMap_Flag_te;


//Bayer2Y_Control_0_e_Flag_Bayer2YEnabled
#define  Bayer2Y_Control_0_e_Flag_Bayer2YEnabled   0x101e
#define  Bayer2Y_Control_0_e_Flag_Bayer2YEnabled_Byte_0   0x101e
#define  Bayer2Y_Control_0_e_Flag_Bayer2YEnabled_SIZE    1
#define  Bayer2Y_Control_0_e_Flag_Bayer2YEnabled_STRING     "Bayer2Y_Control_0_e_Flag_Bayer2YEnabled"

//Bayer2Y_Control_1_e_Flag_Bayer2YEnabled
#define  Bayer2Y_Control_1_e_Flag_Bayer2YEnabled   0x1032
#define  Bayer2Y_Control_1_e_Flag_Bayer2YEnabled_Byte_0   0x1032
#define  Bayer2Y_Control_1_e_Flag_Bayer2YEnabled_SIZE    1
#define  Bayer2Y_Control_1_e_Flag_Bayer2YEnabled_STRING     "Bayer2Y_Control_1_e_Flag_Bayer2YEnabled"

//Bayer2Y_Status_0_u8_FPKr
#define  Bayer2Y_Status_0_u8_FPKr   0x1048
#define  Bayer2Y_Status_0_u8_FPKr_Byte_0   0x1048
#define  Bayer2Y_Status_0_u8_FPKr_SIZE    1
#define  Bayer2Y_Status_0_u8_FPKr_STRING     "Bayer2Y_Status_0_u8_FPKr"

//Bayer2Y_Status_0_u8_FPKgr
#define  Bayer2Y_Status_0_u8_FPKgr   0x1049
#define  Bayer2Y_Status_0_u8_FPKgr_Byte_0   0x1049
#define  Bayer2Y_Status_0_u8_FPKgr_SIZE    1
#define  Bayer2Y_Status_0_u8_FPKgr_STRING     "Bayer2Y_Status_0_u8_FPKgr"

//Bayer2Y_Status_0_u8_FPKb
#define  Bayer2Y_Status_0_u8_FPKb   0x104a
#define  Bayer2Y_Status_0_u8_FPKb_Byte_0   0x104a
#define  Bayer2Y_Status_0_u8_FPKb_SIZE    1
#define  Bayer2Y_Status_0_u8_FPKb_STRING     "Bayer2Y_Status_0_u8_FPKb"

//Bayer2Y_Status_0_u8_FPKgb
#define  Bayer2Y_Status_0_u8_FPKgb   0x104b
#define  Bayer2Y_Status_0_u8_FPKgb_Byte_0   0x104b
#define  Bayer2Y_Status_0_u8_FPKgb_SIZE    1
#define  Bayer2Y_Status_0_u8_FPKgb_STRING     "Bayer2Y_Status_0_u8_FPKgb"

//Bayer2Y_Status_1_u8_FPKr
#define  Bayer2Y_Status_1_u8_FPKr   0x104c
#define  Bayer2Y_Status_1_u8_FPKr_Byte_0   0x104c
#define  Bayer2Y_Status_1_u8_FPKr_SIZE    1
#define  Bayer2Y_Status_1_u8_FPKr_STRING     "Bayer2Y_Status_1_u8_FPKr"

//Bayer2Y_Status_1_u8_FPKgr
#define  Bayer2Y_Status_1_u8_FPKgr   0x104d
#define  Bayer2Y_Status_1_u8_FPKgr_Byte_0   0x104d
#define  Bayer2Y_Status_1_u8_FPKgr_SIZE    1
#define  Bayer2Y_Status_1_u8_FPKgr_STRING     "Bayer2Y_Status_1_u8_FPKgr"

//Bayer2Y_Status_1_u8_FPKb
#define  Bayer2Y_Status_1_u8_FPKb   0x104e
#define  Bayer2Y_Status_1_u8_FPKb_Byte_0   0x104e
#define  Bayer2Y_Status_1_u8_FPKb_SIZE    1
#define  Bayer2Y_Status_1_u8_FPKb_STRING     "Bayer2Y_Status_1_u8_FPKb"

//Bayer2Y_Status_1_u8_FPKgb
#define  Bayer2Y_Status_1_u8_FPKgb   0x104f
#define  Bayer2Y_Status_1_u8_FPKgb_Byte_0   0x104f
#define  Bayer2Y_Status_1_u8_FPKgb_SIZE    1
#define  Bayer2Y_Status_1_u8_FPKgb_STRING     "Bayer2Y_Status_1_u8_FPKgb"

//Bowler_Input_u16_VCrop
#define  Bowler_Input_u16_VCrop_Byte_0  0x1382
#define  Bowler_Input_u16_VCrop_Byte_1  0x1383
#define  Bowler_Input_u16_VCrop_SIZE    2
#define  Bowler_Input_u16_VCrop_STRING     "Bowler_Input_u16_VCrop"

//Bowler_Input_u16_HCrop
#define  Bowler_Input_u16_HCrop_Byte_0  0x1384
#define  Bowler_Input_u16_HCrop_Byte_1  0x1385
#define  Bowler_Input_u16_HCrop_SIZE    2
#define  Bowler_Input_u16_HCrop_STRING     "Bowler_Input_u16_HCrop"

//CRMTop_Control_f_MinVCOFreq_Mhz
#define  CRMTop_Control_f_MinVCOFreq_Mhz_Byte_0  0x1074
#define  CRMTop_Control_f_MinVCOFreq_Mhz_Byte_1  0x1075
#define  CRMTop_Control_f_MinVCOFreq_Mhz_Byte_2  0x1076
#define  CRMTop_Control_f_MinVCOFreq_Mhz_Byte_3  0x1077
#define  CRMTop_Control_f_MinVCOFreq_Mhz_SIZE    4
#define  CRMTop_Control_f_MinVCOFreq_Mhz_STRING     "CRMTop_Control_f_MinVCOFreq_Mhz"

//CRMTop_Control_f_MaxVCOFreq_Mhz
#define  CRMTop_Control_f_MaxVCOFreq_Mhz_Byte_0  0x1078
#define  CRMTop_Control_f_MaxVCOFreq_Mhz_Byte_1  0x1079
#define  CRMTop_Control_f_MaxVCOFreq_Mhz_Byte_2  0x107a
#define  CRMTop_Control_f_MaxVCOFreq_Mhz_Byte_3  0x107b
#define  CRMTop_Control_f_MaxVCOFreq_Mhz_SIZE    4
#define  CRMTop_Control_f_MaxVCOFreq_Mhz_STRING     "CRMTop_Control_f_MaxVCOFreq_Mhz"

//CRMTop_Control_e_Flag_AutoPixClockMode
#define  CRMTop_Control_e_Flag_AutoPixClockMode   0x108a
#define  CRMTop_Control_e_Flag_AutoPixClockMode_Byte_0   0x108a
#define  CRMTop_Control_e_Flag_AutoPixClockMode_SIZE    1
#define  CRMTop_Control_e_Flag_AutoPixClockMode_STRING     "CRMTop_Control_e_Flag_AutoPixClockMode"

//CRMTop_Control_u8_MaxPLLInputDivFactor
#define  CRMTop_Control_u8_MaxPLLInputDivFactor   0x108d
#define  CRMTop_Control_u8_MaxPLLInputDivFactor_Byte_0   0x108d
#define  CRMTop_Control_u8_MaxPLLInputDivFactor_SIZE    1
#define  CRMTop_Control_u8_MaxPLLInputDivFactor_STRING     "CRMTop_Control_u8_MaxPLLInputDivFactor"

//CRMTop_Control_f_DesiredMCUClkFreq_Mhz
#define  CRMTop_Control_f_DesiredMCUClkFreq_Mhz_Byte_0  0x1190
#define  CRMTop_Control_f_DesiredMCUClkFreq_Mhz_Byte_1  0x1191
#define  CRMTop_Control_f_DesiredMCUClkFreq_Mhz_Byte_2  0x1192
#define  CRMTop_Control_f_DesiredMCUClkFreq_Mhz_Byte_3  0x1193
#define  CRMTop_Control_f_DesiredMCUClkFreq_Mhz_SIZE    4
#define  CRMTop_Control_f_DesiredMCUClkFreq_Mhz_STRING     "CRMTop_Control_f_DesiredMCUClkFreq_Mhz"

//CSI2Rx_Control_e_VirtualChannelMap_Ch0
#define  CSI2Rx_Control_e_VirtualChannelMap_Ch0   0x109d
#define  CSI2Rx_Control_e_VirtualChannelMap_Ch0_Byte_0   0x109d
#define  CSI2Rx_Control_e_VirtualChannelMap_Ch0_SIZE    1
#define  CSI2Rx_Control_e_VirtualChannelMap_Ch0_STRING     "CSI2Rx_Control_e_VirtualChannelMap_Ch0"

//CSI2Tx_Control_u8_NumberofLanes
#define  CSI2Tx_Control_u8_NumberofLanes   0x11a1
#define  CSI2Tx_Control_u8_NumberofLanes_Byte_0   0x11a1
#define  CSI2Tx_Control_u8_NumberofLanes_SIZE    1
#define  CSI2Tx_Control_u8_NumberofLanes_STRING     "CSI2Tx_Control_u8_NumberofLanes"

//DeviceInfo_Status_u8_SystemFirmwareVersionMajor
#define  DeviceInfo_Status_u8_SystemFirmwareVersionMajor   0x1002
#define  DeviceInfo_Status_u8_SystemFirmwareVersionMajor_Byte_0   0x1002
#define  DeviceInfo_Status_u8_SystemFirmwareVersionMajor_SIZE    1
#define  DeviceInfo_Status_u8_SystemFirmwareVersionMajor_STRING     "DeviceInfo_Status_u8_SystemFirmwareVersionMajor"

//DeviceInfo_Status_u8_SystemFirmwareVersionMinor
#define  DeviceInfo_Status_u8_SystemFirmwareVersionMinor   0x1003
#define  DeviceInfo_Status_u8_SystemFirmwareVersionMinor_Byte_0   0x1003
#define  DeviceInfo_Status_u8_SystemFirmwareVersionMinor_SIZE    1
#define  DeviceInfo_Status_u8_SystemFirmwareVersionMinor_STRING     "DeviceInfo_Status_u8_SystemFirmwareVersionMinor"

//Duster_Control_0_e_Flag_DusterEnable
#define  Duster_Control_0_e_Flag_DusterEnable   0x10bc
#define  Duster_Control_0_e_Flag_DusterEnable_Byte_0   0x10bc
#define  Duster_Control_0_e_Flag_DusterEnable_SIZE    1
#define  Duster_Control_0_e_Flag_DusterEnable_STRING     "Duster_Control_0_e_Flag_DusterEnable"

//Duster_Control_0_u8_LocalSigmaTh_CC
#define  Duster_Control_0_u8_LocalSigmaTh_CC   0x10c1
#define  Duster_Control_0_u8_LocalSigmaTh_CC_Byte_0   0x10c1
#define  Duster_Control_0_u8_LocalSigmaTh_CC_SIZE    1
#define  Duster_Control_0_u8_LocalSigmaTh_CC_STRING     "Duster_Control_0_u8_LocalSigmaTh_CC"

//Duster_Control_0_u8_NormalisedTh_RC
#define  Duster_Control_0_u8_NormalisedTh_RC   0x10c2
#define  Duster_Control_0_u8_NormalisedTh_RC_Byte_0   0x10c2
#define  Duster_Control_0_u8_NormalisedTh_RC_SIZE    1
#define  Duster_Control_0_u8_NormalisedTh_RC_STRING     "Duster_Control_0_u8_NormalisedTh_RC"

//Duster_Control_0_u8_WhiteDCStrength
#define  Duster_Control_0_u8_WhiteDCStrength   0x10c3
#define  Duster_Control_0_u8_WhiteDCStrength_Byte_0   0x10c3
#define  Duster_Control_0_u8_WhiteDCStrength_SIZE    1
#define  Duster_Control_0_u8_WhiteDCStrength_STRING     "Duster_Control_0_u8_WhiteDCStrength"

//Duster_Control_0_u8_BlackDCStrength
#define  Duster_Control_0_u8_BlackDCStrength   0x10c4
#define  Duster_Control_0_u8_BlackDCStrength_Byte_0   0x10c4
#define  Duster_Control_0_u8_BlackDCStrength_SIZE    1
#define  Duster_Control_0_u8_BlackDCStrength_STRING     "Duster_Control_0_u8_BlackDCStrength"

//Duster_Control_0_e_Flag_BypassRC
#define  Duster_Control_0_e_Flag_BypassRC   0x10c5
#define  Duster_Control_0_e_Flag_BypassRC_Byte_0   0x10c5
#define  Duster_Control_0_e_Flag_BypassRC_SIZE    1
#define  Duster_Control_0_e_Flag_BypassRC_STRING     "Duster_Control_0_e_Flag_BypassRC"

//Duster_Control_0_e_Flag_RC_DoubletsOnly
#define  Duster_Control_0_e_Flag_RC_DoubletsOnly   0x10c6
#define  Duster_Control_0_e_Flag_RC_DoubletsOnly_Byte_0   0x10c6
#define  Duster_Control_0_e_Flag_RC_DoubletsOnly_SIZE    1
#define  Duster_Control_0_e_Flag_RC_DoubletsOnly_STRING     "Duster_Control_0_e_Flag_RC_DoubletsOnly"

//Duster_Control_0_e_Flag_BypassCC
#define  Duster_Control_0_e_Flag_BypassCC   0x10c7
#define  Duster_Control_0_e_Flag_BypassCC_Byte_0   0x10c7
#define  Duster_Control_0_e_Flag_BypassCC_SIZE    1
#define  Duster_Control_0_e_Flag_BypassCC_STRING     "Duster_Control_0_e_Flag_BypassCC"

//Duster_Control_0_e_Flag_BypassDefcor
#define  Duster_Control_0_e_Flag_BypassDefcor   0x10c9
#define  Duster_Control_0_e_Flag_BypassDefcor_Byte_0   0x10c9
#define  Duster_Control_0_e_Flag_BypassDefcor_SIZE    1
#define  Duster_Control_0_e_Flag_BypassDefcor_STRING     "Duster_Control_0_e_Flag_BypassDefcor"

//Duster_Control_0_e_Flag_BypassGaussian
#define  Duster_Control_0_e_Flag_BypassGaussian   0x10ca
#define  Duster_Control_0_e_Flag_BypassGaussian_Byte_0   0x10ca
#define  Duster_Control_0_e_Flag_BypassGaussian_SIZE    1
#define  Duster_Control_0_e_Flag_BypassGaussian_STRING     "Duster_Control_0_e_Flag_BypassGaussian"

//Duster_Control_0_e_Flag_RC_UseSimplified
#define  Duster_Control_0_e_Flag_RC_UseSimplified   0x10cc
#define  Duster_Control_0_e_Flag_RC_UseSimplified_Byte_0   0x10cc
#define  Duster_Control_0_e_Flag_RC_UseSimplified_SIZE    1
#define  Duster_Control_0_e_Flag_RC_UseSimplified_STRING     "Duster_Control_0_e_Flag_RC_UseSimplified"

//Duster_Control_0_e_ScytheMode
#define  Duster_Control_0_e_ScytheMode   0x10ce
#define  Duster_Control_0_e_ScytheMode_Byte_0   0x10ce
#define  Duster_Control_0_e_ScytheMode_SIZE    1
#define  Duster_Control_0_e_ScytheMode_STRING     "Duster_Control_0_e_ScytheMode"

//Duster_Control_1_e_Flag_DusterEnable
#define  Duster_Control_1_e_Flag_DusterEnable   0x10d8
#define  Duster_Control_1_e_Flag_DusterEnable_Byte_0   0x10d8
#define  Duster_Control_1_e_Flag_DusterEnable_SIZE    1
#define  Duster_Control_1_e_Flag_DusterEnable_STRING     "Duster_Control_1_e_Flag_DusterEnable"

//Duster_Control_1_u8_LocalSigmaTh_CC
#define  Duster_Control_1_u8_LocalSigmaTh_CC   0x10dd
#define  Duster_Control_1_u8_LocalSigmaTh_CC_Byte_0   0x10dd
#define  Duster_Control_1_u8_LocalSigmaTh_CC_SIZE    1
#define  Duster_Control_1_u8_LocalSigmaTh_CC_STRING     "Duster_Control_1_u8_LocalSigmaTh_CC"

//Duster_Control_1_u8_NormalisedTh_RC
#define  Duster_Control_1_u8_NormalisedTh_RC   0x10de
#define  Duster_Control_1_u8_NormalisedTh_RC_Byte_0   0x10de
#define  Duster_Control_1_u8_NormalisedTh_RC_SIZE    1
#define  Duster_Control_1_u8_NormalisedTh_RC_STRING     "Duster_Control_1_u8_NormalisedTh_RC"

//Duster_Control_1_u8_WhiteDCStrength
#define  Duster_Control_1_u8_WhiteDCStrength   0x10df
#define  Duster_Control_1_u8_WhiteDCStrength_Byte_0   0x10df
#define  Duster_Control_1_u8_WhiteDCStrength_SIZE    1
#define  Duster_Control_1_u8_WhiteDCStrength_STRING     "Duster_Control_1_u8_WhiteDCStrength"

//Duster_Control_1_u8_BlackDCStrength
#define  Duster_Control_1_u8_BlackDCStrength   0x10e0
#define  Duster_Control_1_u8_BlackDCStrength_Byte_0   0x10e0
#define  Duster_Control_1_u8_BlackDCStrength_SIZE    1
#define  Duster_Control_1_u8_BlackDCStrength_STRING     "Duster_Control_1_u8_BlackDCStrength"

//Duster_Control_1_e_Flag_BypassRC
#define  Duster_Control_1_e_Flag_BypassRC   0x10e1
#define  Duster_Control_1_e_Flag_BypassRC_Byte_0   0x10e1
#define  Duster_Control_1_e_Flag_BypassRC_SIZE    1
#define  Duster_Control_1_e_Flag_BypassRC_STRING     "Duster_Control_1_e_Flag_BypassRC"

//Duster_Control_1_e_Flag_RC_DoubletsOnly
#define  Duster_Control_1_e_Flag_RC_DoubletsOnly   0x10e2
#define  Duster_Control_1_e_Flag_RC_DoubletsOnly_Byte_0   0x10e2
#define  Duster_Control_1_e_Flag_RC_DoubletsOnly_SIZE    1
#define  Duster_Control_1_e_Flag_RC_DoubletsOnly_STRING     "Duster_Control_1_e_Flag_RC_DoubletsOnly"

//Duster_Control_1_e_Flag_BypassCC
#define  Duster_Control_1_e_Flag_BypassCC   0x10e3
#define  Duster_Control_1_e_Flag_BypassCC_Byte_0   0x10e3
#define  Duster_Control_1_e_Flag_BypassCC_SIZE    1
#define  Duster_Control_1_e_Flag_BypassCC_STRING     "Duster_Control_1_e_Flag_BypassCC"

//Duster_Control_1_e_Flag_BypassDefcor
#define  Duster_Control_1_e_Flag_BypassDefcor   0x10e5
#define  Duster_Control_1_e_Flag_BypassDefcor_Byte_0   0x10e5
#define  Duster_Control_1_e_Flag_BypassDefcor_SIZE    1
#define  Duster_Control_1_e_Flag_BypassDefcor_STRING     "Duster_Control_1_e_Flag_BypassDefcor"

//Duster_Control_1_e_Flag_BypassGaussian
#define  Duster_Control_1_e_Flag_BypassGaussian   0x10e6
#define  Duster_Control_1_e_Flag_BypassGaussian_Byte_0   0x10e6
#define  Duster_Control_1_e_Flag_BypassGaussian_SIZE    1
#define  Duster_Control_1_e_Flag_BypassGaussian_STRING     "Duster_Control_1_e_Flag_BypassGaussian"

//Duster_Control_1_e_Flag_RC_UseSimplified
#define  Duster_Control_1_e_Flag_RC_UseSimplified   0x10e8
#define  Duster_Control_1_e_Flag_RC_UseSimplified_Byte_0   0x10e8
#define  Duster_Control_1_e_Flag_RC_UseSimplified_SIZE    1
#define  Duster_Control_1_e_Flag_RC_UseSimplified_STRING     "Duster_Control_1_e_Flag_RC_UseSimplified"

//Duster_Control_1_e_ScytheMode
#define  Duster_Control_1_e_ScytheMode   0x10ea
#define  Duster_Control_1_e_ScytheMode_Byte_0   0x10ea
#define  Duster_Control_1_e_ScytheMode_SIZE    1
#define  Duster_Control_1_e_ScytheMode_STRING     "Duster_Control_1_e_ScytheMode"

//Glace_Control_0_u16_hOffset
#define  Glace_Control_0_u16_hOffset_Byte_0  0x10f6
#define  Glace_Control_0_u16_hOffset_Byte_1  0x10f7
#define  Glace_Control_0_u16_hOffset_SIZE    2
#define  Glace_Control_0_u16_hOffset_STRING     "Glace_Control_0_u16_hOffset"

//Glace_Control_0_u16_vOffset
#define  Glace_Control_0_u16_vOffset_Byte_0  0x10f8
#define  Glace_Control_0_u16_vOffset_Byte_1  0x10f9
#define  Glace_Control_0_u16_vOffset_SIZE    2
#define  Glace_Control_0_u16_vOffset_STRING     "Glace_Control_0_u16_vOffset"

//Glace_Control_0_u16_vBlockSize
#define  Glace_Control_0_u16_vBlockSize_Byte_0  0x10fa
#define  Glace_Control_0_u16_vBlockSize_Byte_1  0x10fb
#define  Glace_Control_0_u16_vBlockSize_SIZE    2
#define  Glace_Control_0_u16_vBlockSize_STRING     "Glace_Control_0_u16_vBlockSize"

//Glace_Control_0_u16_hBlockSize
#define  Glace_Control_0_u16_hBlockSize_Byte_0  0x10fc
#define  Glace_Control_0_u16_hBlockSize_Byte_1  0x10fd
#define  Glace_Control_0_u16_hBlockSize_SIZE    2
#define  Glace_Control_0_u16_hBlockSize_STRING     "Glace_Control_0_u16_hBlockSize"

//Glace_Control_0_u8_SatLevelBlue
#define  Glace_Control_0_u8_SatLevelBlue   0x1100
#define  Glace_Control_0_u8_SatLevelBlue_Byte_0   0x1100
#define  Glace_Control_0_u8_SatLevelBlue_SIZE    1
#define  Glace_Control_0_u8_SatLevelBlue_STRING     "Glace_Control_0_u8_SatLevelBlue"

//Glace_Control_0_u8_SatLevelGreen
#define  Glace_Control_0_u8_SatLevelGreen   0x1101
#define  Glace_Control_0_u8_SatLevelGreen_Byte_0   0x1101
#define  Glace_Control_0_u8_SatLevelGreen_SIZE    1
#define  Glace_Control_0_u8_SatLevelGreen_STRING     "Glace_Control_0_u8_SatLevelGreen"

//Glace_Control_0_u8_SatLevelRed
#define  Glace_Control_0_u8_SatLevelRed   0x1102
#define  Glace_Control_0_u8_SatLevelRed_Byte_0   0x1102
#define  Glace_Control_0_u8_SatLevelRed_SIZE    1
#define  Glace_Control_0_u8_SatLevelRed_STRING     "Glace_Control_0_u8_SatLevelRed"

//Glace_Control_0_u8_vGridSize
#define  Glace_Control_0_u8_vGridSize   0x1103
#define  Glace_Control_0_u8_vGridSize_Byte_0   0x1103
#define  Glace_Control_0_u8_vGridSize_SIZE    1
#define  Glace_Control_0_u8_vGridSize_STRING     "Glace_Control_0_u8_vGridSize"

//Glace_Control_0_u8_hGridSize
#define  Glace_Control_0_u8_hGridSize   0x1104
#define  Glace_Control_0_u8_hGridSize_Byte_0   0x1104
#define  Glace_Control_0_u8_hGridSize_SIZE    1
#define  Glace_Control_0_u8_hGridSize_STRING     "Glace_Control_0_u8_hGridSize"

//Glace_Control_0_e_Flag_GlaceEnabled
#define  Glace_Control_0_e_Flag_GlaceEnabled   0x1109
#define  Glace_Control_0_e_Flag_GlaceEnabled_Byte_0   0x1109
#define  Glace_Control_0_e_Flag_GlaceEnabled_SIZE    1
#define  Glace_Control_0_e_Flag_GlaceEnabled_STRING     "Glace_Control_0_e_Flag_GlaceEnabled"

//Glace_Control_1_u16_hOffset
#define  Glace_Control_1_u16_hOffset_Byte_0  0x110a
#define  Glace_Control_1_u16_hOffset_Byte_1  0x110b
#define  Glace_Control_1_u16_hOffset_SIZE    2
#define  Glace_Control_1_u16_hOffset_STRING     "Glace_Control_1_u16_hOffset"

//Glace_Control_1_u16_vOffset
#define  Glace_Control_1_u16_vOffset_Byte_0  0x110c
#define  Glace_Control_1_u16_vOffset_Byte_1  0x110d
#define  Glace_Control_1_u16_vOffset_SIZE    2
#define  Glace_Control_1_u16_vOffset_STRING     "Glace_Control_1_u16_vOffset"

//Glace_Control_1_u16_vBlockSize
#define  Glace_Control_1_u16_vBlockSize_Byte_0  0x110e
#define  Glace_Control_1_u16_vBlockSize_Byte_1  0x110f
#define  Glace_Control_1_u16_vBlockSize_SIZE    2
#define  Glace_Control_1_u16_vBlockSize_STRING     "Glace_Control_1_u16_vBlockSize"

//Glace_Control_1_u16_hBlockSize
#define  Glace_Control_1_u16_hBlockSize_Byte_0  0x1110
#define  Glace_Control_1_u16_hBlockSize_Byte_1  0x1111
#define  Glace_Control_1_u16_hBlockSize_SIZE    2
#define  Glace_Control_1_u16_hBlockSize_STRING     "Glace_Control_1_u16_hBlockSize"

//Glace_Control_1_u8_SatLevelBlue
#define  Glace_Control_1_u8_SatLevelBlue   0x1114
#define  Glace_Control_1_u8_SatLevelBlue_Byte_0   0x1114
#define  Glace_Control_1_u8_SatLevelBlue_SIZE    1
#define  Glace_Control_1_u8_SatLevelBlue_STRING     "Glace_Control_1_u8_SatLevelBlue"

//Glace_Control_1_u8_SatLevelGreen
#define  Glace_Control_1_u8_SatLevelGreen   0x1115
#define  Glace_Control_1_u8_SatLevelGreen_Byte_0   0x1115
#define  Glace_Control_1_u8_SatLevelGreen_SIZE    1
#define  Glace_Control_1_u8_SatLevelGreen_STRING     "Glace_Control_1_u8_SatLevelGreen"

//Glace_Control_1_u8_SatLevelRed
#define  Glace_Control_1_u8_SatLevelRed   0x1116
#define  Glace_Control_1_u8_SatLevelRed_Byte_0   0x1116
#define  Glace_Control_1_u8_SatLevelRed_SIZE    1
#define  Glace_Control_1_u8_SatLevelRed_STRING     "Glace_Control_1_u8_SatLevelRed"

//Glace_Control_1_u8_vGridSize
#define  Glace_Control_1_u8_vGridSize   0x1117
#define  Glace_Control_1_u8_vGridSize_Byte_0   0x1117
#define  Glace_Control_1_u8_vGridSize_SIZE    1
#define  Glace_Control_1_u8_vGridSize_STRING     "Glace_Control_1_u8_vGridSize"

//Glace_Control_1_u8_hGridSize
#define  Glace_Control_1_u8_hGridSize   0x1118
#define  Glace_Control_1_u8_hGridSize_Byte_0   0x1118
#define  Glace_Control_1_u8_hGridSize_SIZE    1
#define  Glace_Control_1_u8_hGridSize_STRING     "Glace_Control_1_u8_hGridSize"

//Glace_Control_1_e_Flag_GlaceEnabled
#define  Glace_Control_1_e_Flag_GlaceEnabled   0x111d
#define  Glace_Control_1_e_Flag_GlaceEnabled_Byte_0   0x111d
#define  Glace_Control_1_e_Flag_GlaceEnabled_SIZE    1
#define  Glace_Control_1_e_Flag_GlaceEnabled_STRING     "Glace_Control_1_e_Flag_GlaceEnabled"

//HDRMerge_Control_u8_HDRFactor
#define  HDRMerge_Control_u8_HDRFactor   0x118a
#define  HDRMerge_Control_u8_HDRFactor_Byte_0   0x118a
#define  HDRMerge_Control_u8_HDRFactor_SIZE    1
#define  HDRMerge_Control_u8_HDRFactor_STRING     "HDRMerge_Control_u8_HDRFactor"

//HIF_FrameDimension_u32_VPreScaleFactor
#define  HIF_FrameDimension_u32_VPreScaleFactor_Byte_0  0x11b4
#define  HIF_FrameDimension_u32_VPreScaleFactor_Byte_1  0x11b5
#define  HIF_FrameDimension_u32_VPreScaleFactor_Byte_2  0x11b6
#define  HIF_FrameDimension_u32_VPreScaleFactor_Byte_3  0x11b7
#define  HIF_FrameDimension_u32_VPreScaleFactor_SIZE    4
#define  HIF_FrameDimension_u32_VPreScaleFactor_STRING     "HIF_FrameDimension_u32_VPreScaleFactor"

//HIF_FrameDimension_u32_HPreScaleFactor
#define  HIF_FrameDimension_u32_HPreScaleFactor_Byte_0  0x11b8
#define  HIF_FrameDimension_u32_HPreScaleFactor_Byte_1  0x11b9
#define  HIF_FrameDimension_u32_HPreScaleFactor_Byte_2  0x11ba
#define  HIF_FrameDimension_u32_HPreScaleFactor_Byte_3  0x11bb
#define  HIF_FrameDimension_u32_HPreScaleFactor_SIZE    4
#define  HIF_FrameDimension_u32_HPreScaleFactor_STRING     "HIF_FrameDimension_u32_HPreScaleFactor"

//HIF_FrameDimension_u16_HCropStart
#define  HIF_FrameDimension_u16_HCropStart_Byte_0  0x11bc
#define  HIF_FrameDimension_u16_HCropStart_Byte_1  0x11bd
#define  HIF_FrameDimension_u16_HCropStart_SIZE    2
#define  HIF_FrameDimension_u16_HCropStart_STRING     "HIF_FrameDimension_u16_HCropStart"

//HIF_FrameDimension_u16_VCropStart
#define  HIF_FrameDimension_u16_VCropStart_Byte_0  0x11be
#define  HIF_FrameDimension_u16_VCropStart_Byte_1  0x11bf
#define  HIF_FrameDimension_u16_VCropStart_SIZE    2
#define  HIF_FrameDimension_u16_VCropStart_STRING     "HIF_FrameDimension_u16_VCropStart"

//HIF_FrameDimension_u16_FrameLength
#define  HIF_FrameDimension_u16_FrameLength_Byte_0  0x1598
#define  HIF_FrameDimension_u16_FrameLength_Byte_1  0x1599
#define  HIF_FrameDimension_u16_FrameLength_SIZE    2
#define  HIF_FrameDimension_u16_FrameLength_STRING     "HIF_FrameDimension_u16_FrameLength"

//HIF_FrameDimension_u16_LineLength
#define  HIF_FrameDimension_u16_LineLength_Byte_0  0x159a
#define  HIF_FrameDimension_u16_LineLength_Byte_1  0x159b
#define  HIF_FrameDimension_u16_LineLength_SIZE    2
#define  HIF_FrameDimension_u16_LineLength_STRING     "HIF_FrameDimension_u16_LineLength"

//HIF_ImageCharacteristics_u16_StatusLineLengthPixels
#define  HIF_ImageCharacteristics_u16_StatusLineLengthPixels_Byte_0  0x11c2
#define  HIF_ImageCharacteristics_u16_StatusLineLengthPixels_Byte_1  0x11c3
#define  HIF_ImageCharacteristics_u16_StatusLineLengthPixels_SIZE    2
#define  HIF_ImageCharacteristics_u16_StatusLineLengthPixels_STRING     "HIF_ImageCharacteristics_u16_StatusLineLengthPixels"

//HIF_ImageCharacteristics_u16_MinInterframeAtOutput
#define  HIF_ImageCharacteristics_u16_MinInterframeAtOutput_Byte_0  0x11c4
#define  HIF_ImageCharacteristics_u16_MinInterframeAtOutput_Byte_1  0x11c5
#define  HIF_ImageCharacteristics_u16_MinInterframeAtOutput_SIZE    2
#define  HIF_ImageCharacteristics_u16_MinInterframeAtOutput_STRING     "HIF_ImageCharacteristics_u16_MinInterframeAtOutput"

//HIF_ImageCharacteristics_e_InputImageSource
#define  HIF_ImageCharacteristics_e_InputImageSource   0x11c6
#define  HIF_ImageCharacteristics_e_InputImageSource_Byte_0   0x11c6
#define  HIF_ImageCharacteristics_e_InputImageSource_SIZE    1
#define  HIF_ImageCharacteristics_e_InputImageSource_STRING     "HIF_ImageCharacteristics_e_InputImageSource"

//HIF_ImageCharacteristics_e_ImageFormat_Output
#define  HIF_ImageCharacteristics_e_ImageFormat_Output   0x11c7
#define  HIF_ImageCharacteristics_e_ImageFormat_Output_Byte_0   0x11c7
#define  HIF_ImageCharacteristics_e_ImageFormat_Output_SIZE    1
#define  HIF_ImageCharacteristics_e_ImageFormat_Output_STRING     "HIF_ImageCharacteristics_e_ImageFormat_Output"

//HIF_ImageCharacteristics_e_Orientation_FlipXY
#define  HIF_ImageCharacteristics_e_Orientation_FlipXY   0x11c8
#define  HIF_ImageCharacteristics_e_Orientation_FlipXY_Byte_0   0x11c8
#define  HIF_ImageCharacteristics_e_Orientation_FlipXY_SIZE    1
#define  HIF_ImageCharacteristics_e_Orientation_FlipXY_STRING     "HIF_ImageCharacteristics_e_Orientation_FlipXY"

//HIF_ImageCharacteristics_e_Flag_StatusLinesOutputted
#define  HIF_ImageCharacteristics_e_Flag_StatusLinesOutputted   0x11c9
#define  HIF_ImageCharacteristics_e_Flag_StatusLinesOutputted_Byte_0   0x11c9
#define  HIF_ImageCharacteristics_e_Flag_StatusLinesOutputted_SIZE    1
#define  HIF_ImageCharacteristics_e_Flag_StatusLinesOutputted_STRING     "HIF_ImageCharacteristics_e_Flag_StatusLinesOutputted"

//HIF_ImageCharacteristics_u8_NoStatusLinesOutputted
#define  HIF_ImageCharacteristics_u8_NoStatusLinesOutputted   0x11ca
#define  HIF_ImageCharacteristics_u8_NoStatusLinesOutputted_Byte_0   0x11ca
#define  HIF_ImageCharacteristics_u8_NoStatusLinesOutputted_SIZE    1
#define  HIF_ImageCharacteristics_u8_NoStatusLinesOutputted_STRING     "HIF_ImageCharacteristics_u8_NoStatusLinesOutputted"

//HIF_ImageCharacteristics_e_Flag_AutoFrameParamUpdate
#define  HIF_ImageCharacteristics_e_Flag_AutoFrameParamUpdate   0x11cb
#define  HIF_ImageCharacteristics_e_Flag_AutoFrameParamUpdate_Byte_0   0x11cb
#define  HIF_ImageCharacteristics_e_Flag_AutoFrameParamUpdate_SIZE    1
#define  HIF_ImageCharacteristics_e_Flag_AutoFrameParamUpdate_STRING     "HIF_ImageCharacteristics_e_Flag_AutoFrameParamUpdate"

//HIF_ImageCharacteristics_e_ImageFormat_Input
#define  HIF_ImageCharacteristics_e_ImageFormat_Input   0x11cc
#define  HIF_ImageCharacteristics_e_ImageFormat_Input_Byte_0   0x11cc
#define  HIF_ImageCharacteristics_e_ImageFormat_Input_SIZE    1
#define  HIF_ImageCharacteristics_e_ImageFormat_Input_STRING     "HIF_ImageCharacteristics_e_ImageFormat_Input"

//HIF_ImageCharacteristics_e_PixelOrder
#define  HIF_ImageCharacteristics_e_PixelOrder   0x11cf
#define  HIF_ImageCharacteristics_e_PixelOrder_Byte_0   0x11cf
#define  HIF_ImageCharacteristics_e_PixelOrder_SIZE    1
#define  HIF_ImageCharacteristics_e_PixelOrder_STRING     "HIF_ImageCharacteristics_e_PixelOrder"

//HIF_ImageCharacteristics_u8_InputNbStatusLines
#define  HIF_ImageCharacteristics_u8_InputNbStatusLines   0x11d0
#define  HIF_ImageCharacteristics_u8_InputNbStatusLines_Byte_0   0x11d0
#define  HIF_ImageCharacteristics_u8_InputNbStatusLines_SIZE    1
#define  HIF_ImageCharacteristics_u8_InputNbStatusLines_STRING     "HIF_ImageCharacteristics_u8_InputNbStatusLines"

//HIF_ImageCharacteristics_u16_CustomImageSizeX
#define  HIF_ImageCharacteristics_u16_CustomImageSizeX_Byte_0  0x159c
#define  HIF_ImageCharacteristics_u16_CustomImageSizeX_Byte_1  0x159d
#define  HIF_ImageCharacteristics_u16_CustomImageSizeX_SIZE    2
#define  HIF_ImageCharacteristics_u16_CustomImageSizeX_STRING     "HIF_ImageCharacteristics_u16_CustomImageSizeX"

//HIF_ImageCharacteristics_u16_CustomImageSizeY
#define  HIF_ImageCharacteristics_u16_CustomImageSizeY_Byte_0  0x159e
#define  HIF_ImageCharacteristics_u16_CustomImageSizeY_Byte_1  0x159f
#define  HIF_ImageCharacteristics_u16_CustomImageSizeY_SIZE    2
#define  HIF_ImageCharacteristics_u16_CustomImageSizeY_STRING     "HIF_ImageCharacteristics_u16_CustomImageSizeY"

//HIF_SystemandStatus_u32_MaxOutputDataRate_bps
#define  HIF_SystemandStatus_u32_MaxOutputDataRate_bps_Byte_0  0x1194
#define  HIF_SystemandStatus_u32_MaxOutputDataRate_bps_Byte_1  0x1195
#define  HIF_SystemandStatus_u32_MaxOutputDataRate_bps_Byte_2  0x1196
#define  HIF_SystemandStatus_u32_MaxOutputDataRate_bps_Byte_3  0x1197
#define  HIF_SystemandStatus_u32_MaxOutputDataRate_bps_SIZE    4
#define  HIF_SystemandStatus_u32_MaxOutputDataRate_bps_STRING     "HIF_SystemandStatus_u32_MaxOutputDataRate_bps"

//HIF_SystemandStatus_e_ImagePipe_StreamOutput
#define  HIF_SystemandStatus_e_ImagePipe_StreamOutput   0x1198
#define  HIF_SystemandStatus_e_ImagePipe_StreamOutput_Byte_0   0x1198
#define  HIF_SystemandStatus_e_ImagePipe_StreamOutput_SIZE    1
#define  HIF_SystemandStatus_e_ImagePipe_StreamOutput_STRING     "HIF_SystemandStatus_e_ImagePipe_StreamOutput"

//HIF_SystemandStatus_e_StatsHostAck_LongGlace
#define  HIF_SystemandStatus_e_StatsHostAck_LongGlace   0x1199
#define  HIF_SystemandStatus_e_StatsHostAck_LongGlace_Byte_0   0x1199
#define  HIF_SystemandStatus_e_StatsHostAck_LongGlace_SIZE    1
#define  HIF_SystemandStatus_e_StatsHostAck_LongGlace_STRING     "HIF_SystemandStatus_e_StatsHostAck_LongGlace"

//HIF_SystemandStatus_e_Flag_ISPBypass
#define  HIF_SystemandStatus_e_Flag_ISPBypass   0x119a
#define  HIF_SystemandStatus_e_Flag_ISPBypass_Byte_0   0x119a
#define  HIF_SystemandStatus_e_Flag_ISPBypass_SIZE    1
#define  HIF_SystemandStatus_e_Flag_ISPBypass_STRING     "HIF_SystemandStatus_e_Flag_ISPBypass"

//HIF_SystemandStatus_e_Flag_Binning
#define  HIF_SystemandStatus_e_Flag_Binning   0x119b
#define  HIF_SystemandStatus_e_Flag_Binning_Byte_0   0x119b
#define  HIF_SystemandStatus_e_Flag_Binning_SIZE    1
#define  HIF_SystemandStatus_e_Flag_Binning_STRING     "HIF_SystemandStatus_e_Flag_Binning"

//HIF_SystemandStatus_e_ModeControl
#define  HIF_SystemandStatus_e_ModeControl   0x119c
#define  HIF_SystemandStatus_e_ModeControl_Byte_0   0x119c
#define  HIF_SystemandStatus_e_ModeControl_SIZE    1
#define  HIF_SystemandStatus_e_ModeControl_STRING     "HIF_SystemandStatus_e_ModeControl"

//HIF_SystemandStatus_e_Flag_HDRMode
#define  HIF_SystemandStatus_e_Flag_HDRMode   0x119f
#define  HIF_SystemandStatus_e_Flag_HDRMode_Byte_0   0x119f
#define  HIF_SystemandStatus_e_Flag_HDRMode_SIZE    1
#define  HIF_SystemandStatus_e_Flag_HDRMode_STRING     "HIF_SystemandStatus_e_Flag_HDRMode"

//HIF_SystemandStatus_u8_CSIRxDataLaneCount
#define  HIF_SystemandStatus_u8_CSIRxDataLaneCount   0x11a0
#define  HIF_SystemandStatus_u8_CSIRxDataLaneCount_Byte_0   0x11a0
#define  HIF_SystemandStatus_u8_CSIRxDataLaneCount_SIZE    1
#define  HIF_SystemandStatus_u8_CSIRxDataLaneCount_STRING     "HIF_SystemandStatus_u8_CSIRxDataLaneCount"

//HIF_SystemandStatus_e_SensorSelected
#define  HIF_SystemandStatus_e_SensorSelected   0x11a3
#define  HIF_SystemandStatus_e_SensorSelected_Byte_0   0x11a3
#define  HIF_SystemandStatus_e_SensorSelected_SIZE    1
#define  HIF_SystemandStatus_e_SensorSelected_STRING     "HIF_SystemandStatus_e_SensorSelected"

//HIF_SystemandStatus_e_ModeChange
#define  HIF_SystemandStatus_e_ModeChange   0x11a4
#define  HIF_SystemandStatus_e_ModeChange_Byte_0   0x11a4
#define  HIF_SystemandStatus_e_ModeChange_SIZE    1
#define  HIF_SystemandStatus_e_ModeChange_STRING     "HIF_SystemandStatus_e_ModeChange"

//HIF_SystemandStatus_e_StatsHostAck_ShortGlace
#define  HIF_SystemandStatus_e_StatsHostAck_ShortGlace   0x11a5
#define  HIF_SystemandStatus_e_StatsHostAck_ShortGlace_Byte_0   0x11a5
#define  HIF_SystemandStatus_e_StatsHostAck_ShortGlace_SIZE    1
#define  HIF_SystemandStatus_e_StatsHostAck_ShortGlace_STRING     "HIF_SystemandStatus_e_StatsHostAck_ShortGlace"

//HIF_SystemandStatus_e_StatsHostAck_ShortHist
#define  HIF_SystemandStatus_e_StatsHostAck_ShortHist   0x11a6
#define  HIF_SystemandStatus_e_StatsHostAck_ShortHist_Byte_0   0x11a6
#define  HIF_SystemandStatus_e_StatsHostAck_ShortHist_SIZE    1
#define  HIF_SystemandStatus_e_StatsHostAck_ShortHist_STRING     "HIF_SystemandStatus_e_StatsHostAck_ShortHist"

//HIF_SystemandStatus_e_StatsHostAck_LongHist
#define  HIF_SystemandStatus_e_StatsHostAck_LongHist   0x11a7
#define  HIF_SystemandStatus_e_StatsHostAck_LongHist_Byte_0   0x11a7
#define  HIF_SystemandStatus_e_StatsHostAck_LongHist_SIZE    1
#define  HIF_SystemandStatus_e_StatsHostAck_LongHist_STRING     "HIF_SystemandStatus_e_StatsHostAck_LongHist"

//HIF_SystemandStatus_e_ModeStatus
#define  HIF_SystemandStatus_e_ModeStatus   0x11d9
#define  HIF_SystemandStatus_e_ModeStatus_Byte_0   0x11d9
#define  HIF_SystemandStatus_e_ModeStatus_SIZE    1
#define  HIF_SystemandStatus_e_ModeStatus_STRING     "HIF_SystemandStatus_e_ModeStatus"

//HIF_SystemandStatus_e_ModeStatus_Next
#define  HIF_SystemandStatus_e_ModeStatus_Next   0x11da
#define  HIF_SystemandStatus_e_ModeStatus_Next_Byte_0   0x11da
#define  HIF_SystemandStatus_e_ModeStatus_Next_SIZE    1
#define  HIF_SystemandStatus_e_ModeStatus_Next_STRING     "HIF_SystemandStatus_e_ModeStatus_Next"

//HIF_SystemandStatus_e_SystemError
#define  HIF_SystemandStatus_e_SystemError   0x11dd
#define  HIF_SystemandStatus_e_SystemError_Byte_0   0x11dd
#define  HIF_SystemandStatus_e_SystemError_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_STRING     "HIF_SystemandStatus_e_SystemError"

//HIF_SystemandStatus_u32_SensorDataRate_bps
#define  HIF_SystemandStatus_u32_SensorDataRate_bps_Byte_0  0x15a0
#define  HIF_SystemandStatus_u32_SensorDataRate_bps_Byte_1  0x15a1
#define  HIF_SystemandStatus_u32_SensorDataRate_bps_Byte_2  0x15a2
#define  HIF_SystemandStatus_u32_SensorDataRate_bps_Byte_3  0x15a3
#define  HIF_SystemandStatus_u32_SensorDataRate_bps_SIZE    4
#define  HIF_SystemandStatus_u32_SensorDataRate_bps_STRING     "HIF_SystemandStatus_u32_SensorDataRate_bps"

//HIF_SystemandStatus_u32_ExternalClockFrequency_hz
#define  HIF_SystemandStatus_u32_ExternalClockFrequency_hz_Byte_0  0x15a4
#define  HIF_SystemandStatus_u32_ExternalClockFrequency_hz_Byte_1  0x15a5
#define  HIF_SystemandStatus_u32_ExternalClockFrequency_hz_Byte_2  0x15a6
#define  HIF_SystemandStatus_u32_ExternalClockFrequency_hz_Byte_3  0x15a7
#define  HIF_SystemandStatus_u32_ExternalClockFrequency_hz_SIZE    4
#define  HIF_SystemandStatus_u32_ExternalClockFrequency_hz_STRING     "HIF_SystemandStatus_u32_ExternalClockFrequency_hz"

//HIF_SystemandStatus_e_SystemError_List_0
#define  HIF_SystemandStatus_e_SystemError_List_0   0x15c4
#define  HIF_SystemandStatus_e_SystemError_List_0_Byte_0   0x15c4
#define  HIF_SystemandStatus_e_SystemError_List_0_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_0_STRING     "HIF_SystemandStatus_e_SystemError_List_0"

//HIF_SystemandStatus_e_SystemError_List_1
#define  HIF_SystemandStatus_e_SystemError_List_1   0x15c5
#define  HIF_SystemandStatus_e_SystemError_List_1_Byte_0   0x15c5
#define  HIF_SystemandStatus_e_SystemError_List_1_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_1_STRING     "HIF_SystemandStatus_e_SystemError_List_1"

//HIF_SystemandStatus_e_SystemError_List_2
#define  HIF_SystemandStatus_e_SystemError_List_2   0x15c6
#define  HIF_SystemandStatus_e_SystemError_List_2_Byte_0   0x15c6
#define  HIF_SystemandStatus_e_SystemError_List_2_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_2_STRING     "HIF_SystemandStatus_e_SystemError_List_2"

//HIF_SystemandStatus_e_SystemError_List_3
#define  HIF_SystemandStatus_e_SystemError_List_3   0x15c7
#define  HIF_SystemandStatus_e_SystemError_List_3_Byte_0   0x15c7
#define  HIF_SystemandStatus_e_SystemError_List_3_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_3_STRING     "HIF_SystemandStatus_e_SystemError_List_3"

//HIF_SystemandStatus_e_SystemError_List_4
#define  HIF_SystemandStatus_e_SystemError_List_4   0x15c8
#define  HIF_SystemandStatus_e_SystemError_List_4_Byte_0   0x15c8
#define  HIF_SystemandStatus_e_SystemError_List_4_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_4_STRING     "HIF_SystemandStatus_e_SystemError_List_4"

//HIF_SystemandStatus_e_SystemError_List_5
#define  HIF_SystemandStatus_e_SystemError_List_5   0x15c9
#define  HIF_SystemandStatus_e_SystemError_List_5_Byte_0   0x15c9
#define  HIF_SystemandStatus_e_SystemError_List_5_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_5_STRING     "HIF_SystemandStatus_e_SystemError_List_5"

//HIF_SystemandStatus_e_SystemError_List_6
#define  HIF_SystemandStatus_e_SystemError_List_6   0x15ca
#define  HIF_SystemandStatus_e_SystemError_List_6_Byte_0   0x15ca
#define  HIF_SystemandStatus_e_SystemError_List_6_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_6_STRING     "HIF_SystemandStatus_e_SystemError_List_6"

//HIF_SystemandStatus_e_SystemError_List_7
#define  HIF_SystemandStatus_e_SystemError_List_7   0x15cb
#define  HIF_SystemandStatus_e_SystemError_List_7_Byte_0   0x15cb
#define  HIF_SystemandStatus_e_SystemError_List_7_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_7_STRING     "HIF_SystemandStatus_e_SystemError_List_7"

//HIF_SystemandStatus_e_SystemError_List_8
#define  HIF_SystemandStatus_e_SystemError_List_8   0x15cc
#define  HIF_SystemandStatus_e_SystemError_List_8_Byte_0   0x15cc
#define  HIF_SystemandStatus_e_SystemError_List_8_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_8_STRING     "HIF_SystemandStatus_e_SystemError_List_8"

//HIF_SystemandStatus_e_SystemError_List_9
#define  HIF_SystemandStatus_e_SystemError_List_9   0x15cd
#define  HIF_SystemandStatus_e_SystemError_List_9_Byte_0   0x15cd
#define  HIF_SystemandStatus_e_SystemError_List_9_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_9_STRING     "HIF_SystemandStatus_e_SystemError_List_9"

//HIF_SystemandStatus_e_SystemError_List_10
#define  HIF_SystemandStatus_e_SystemError_List_10   0x15ce
#define  HIF_SystemandStatus_e_SystemError_List_10_Byte_0   0x15ce
#define  HIF_SystemandStatus_e_SystemError_List_10_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_10_STRING     "HIF_SystemandStatus_e_SystemError_List_10"

//HIF_SystemandStatus_e_SystemError_List_11
#define  HIF_SystemandStatus_e_SystemError_List_11   0x15cf
#define  HIF_SystemandStatus_e_SystemError_List_11_Byte_0   0x15cf
#define  HIF_SystemandStatus_e_SystemError_List_11_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_11_STRING     "HIF_SystemandStatus_e_SystemError_List_11"

//HIF_SystemandStatus_e_SystemError_List_12
#define  HIF_SystemandStatus_e_SystemError_List_12   0x15d0
#define  HIF_SystemandStatus_e_SystemError_List_12_Byte_0   0x15d0
#define  HIF_SystemandStatus_e_SystemError_List_12_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_12_STRING     "HIF_SystemandStatus_e_SystemError_List_12"

//HIF_SystemandStatus_e_SystemError_List_13
#define  HIF_SystemandStatus_e_SystemError_List_13   0x15d1
#define  HIF_SystemandStatus_e_SystemError_List_13_Byte_0   0x15d1
#define  HIF_SystemandStatus_e_SystemError_List_13_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_13_STRING     "HIF_SystemandStatus_e_SystemError_List_13"

//HIF_SystemandStatus_e_SystemError_List_14
#define  HIF_SystemandStatus_e_SystemError_List_14   0x15d2
#define  HIF_SystemandStatus_e_SystemError_List_14_Byte_0   0x15d2
#define  HIF_SystemandStatus_e_SystemError_List_14_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_14_STRING     "HIF_SystemandStatus_e_SystemError_List_14"

//HIF_SystemandStatus_e_SystemError_List_15
#define  HIF_SystemandStatus_e_SystemError_List_15   0x15d3
#define  HIF_SystemandStatus_e_SystemError_List_15_Byte_0   0x15d3
#define  HIF_SystemandStatus_e_SystemError_List_15_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_15_STRING     "HIF_SystemandStatus_e_SystemError_List_15"

//HIF_SystemandStatus_e_SystemError_List_16
#define  HIF_SystemandStatus_e_SystemError_List_16   0x15d4
#define  HIF_SystemandStatus_e_SystemError_List_16_Byte_0   0x15d4
#define  HIF_SystemandStatus_e_SystemError_List_16_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_16_STRING     "HIF_SystemandStatus_e_SystemError_List_16"

//HIF_SystemandStatus_e_SystemError_List_17
#define  HIF_SystemandStatus_e_SystemError_List_17   0x15d5
#define  HIF_SystemandStatus_e_SystemError_List_17_Byte_0   0x15d5
#define  HIF_SystemandStatus_e_SystemError_List_17_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_17_STRING     "HIF_SystemandStatus_e_SystemError_List_17"

//HIF_SystemandStatus_e_SystemError_List_18
#define  HIF_SystemandStatus_e_SystemError_List_18   0x15d6
#define  HIF_SystemandStatus_e_SystemError_List_18_Byte_0   0x15d6
#define  HIF_SystemandStatus_e_SystemError_List_18_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_18_STRING     "HIF_SystemandStatus_e_SystemError_List_18"

//HIF_SystemandStatus_e_SystemError_List_19
#define  HIF_SystemandStatus_e_SystemError_List_19   0x15d7
#define  HIF_SystemandStatus_e_SystemError_List_19_Byte_0   0x15d7
#define  HIF_SystemandStatus_e_SystemError_List_19_SIZE    1
#define  HIF_SystemandStatus_e_SystemError_List_19_STRING     "HIF_SystemandStatus_e_SystemError_List_19"

//HawkEyes_Control_e_Flag_ByPassMacroPixel
#define  HawkEyes_Control_e_Flag_ByPassMacroPixel   0x115e
#define  HawkEyes_Control_e_Flag_ByPassMacroPixel_Byte_0   0x115e
#define  HawkEyes_Control_e_Flag_ByPassMacroPixel_SIZE    1
#define  HawkEyes_Control_e_Flag_ByPassMacroPixel_STRING     "HawkEyes_Control_e_Flag_ByPassMacroPixel"

//HawkEyes_Control_e_MergeMode
#define  HawkEyes_Control_e_MergeMode   0x1160
#define  HawkEyes_Control_e_MergeMode_Byte_0   0x1160
#define  HawkEyes_Control_e_MergeMode_SIZE    1
#define  HawkEyes_Control_e_MergeMode_STRING     "HawkEyes_Control_e_MergeMode"

//Histogram_Control_0_u16_HistSizeX
#define  Histogram_Control_0_u16_HistSizeX_Byte_0  0x11e0
#define  Histogram_Control_0_u16_HistSizeX_Byte_1  0x11e1
#define  Histogram_Control_0_u16_HistSizeX_SIZE    2
#define  Histogram_Control_0_u16_HistSizeX_STRING     "Histogram_Control_0_u16_HistSizeX"

//Histogram_Control_0_u16_HistSizeY
#define  Histogram_Control_0_u16_HistSizeY_Byte_0  0x11e2
#define  Histogram_Control_0_u16_HistSizeY_Byte_1  0x11e3
#define  Histogram_Control_0_u16_HistSizeY_SIZE    2
#define  Histogram_Control_0_u16_HistSizeY_STRING     "Histogram_Control_0_u16_HistSizeY"

//Histogram_Control_0_u16_FOVOffsetX
#define  Histogram_Control_0_u16_FOVOffsetX_Byte_0  0x11e4
#define  Histogram_Control_0_u16_FOVOffsetX_Byte_1  0x11e5
#define  Histogram_Control_0_u16_FOVOffsetX_SIZE    2
#define  Histogram_Control_0_u16_FOVOffsetX_STRING     "Histogram_Control_0_u16_FOVOffsetX"

//Histogram_Control_0_u16_FOVOffsetY
#define  Histogram_Control_0_u16_FOVOffsetY_Byte_0  0x11e6
#define  Histogram_Control_0_u16_FOVOffsetY_Byte_1  0x11e7
#define  Histogram_Control_0_u16_FOVOffsetY_SIZE    2
#define  Histogram_Control_0_u16_FOVOffsetY_STRING     "Histogram_Control_0_u16_FOVOffsetY"

//Histogram_Control_0_e_Flag_Enabled
#define  Histogram_Control_0_e_Flag_Enabled   0x11ec
#define  Histogram_Control_0_e_Flag_Enabled_Byte_0   0x11ec
#define  Histogram_Control_0_e_Flag_Enabled_SIZE    1
#define  Histogram_Control_0_e_Flag_Enabled_STRING     "Histogram_Control_0_e_Flag_Enabled"

//Histogram_Control_1_u16_HistSizeX
#define  Histogram_Control_1_u16_HistSizeX_Byte_0  0x11f8
#define  Histogram_Control_1_u16_HistSizeX_Byte_1  0x11f9
#define  Histogram_Control_1_u16_HistSizeX_SIZE    2
#define  Histogram_Control_1_u16_HistSizeX_STRING     "Histogram_Control_1_u16_HistSizeX"

//Histogram_Control_1_u16_HistSizeY
#define  Histogram_Control_1_u16_HistSizeY_Byte_0  0x11fa
#define  Histogram_Control_1_u16_HistSizeY_Byte_1  0x11fb
#define  Histogram_Control_1_u16_HistSizeY_SIZE    2
#define  Histogram_Control_1_u16_HistSizeY_STRING     "Histogram_Control_1_u16_HistSizeY"

//Histogram_Control_1_u16_FOVOffsetX
#define  Histogram_Control_1_u16_FOVOffsetX_Byte_0  0x11fc
#define  Histogram_Control_1_u16_FOVOffsetX_Byte_1  0x11fd
#define  Histogram_Control_1_u16_FOVOffsetX_SIZE    2
#define  Histogram_Control_1_u16_FOVOffsetX_STRING     "Histogram_Control_1_u16_FOVOffsetX"

//Histogram_Control_1_u16_FOVOffsetY
#define  Histogram_Control_1_u16_FOVOffsetY_Byte_0  0x11fe
#define  Histogram_Control_1_u16_FOVOffsetY_Byte_1  0x11ff
#define  Histogram_Control_1_u16_FOVOffsetY_SIZE    2
#define  Histogram_Control_1_u16_FOVOffsetY_STRING     "Histogram_Control_1_u16_FOVOffsetY"

//Histogram_Control_1_e_Flag_Enabled
#define  Histogram_Control_1_e_Flag_Enabled   0x1204
#define  Histogram_Control_1_e_Flag_Enabled_Byte_0   0x1204
#define  Histogram_Control_1_e_Flag_Enabled_SIZE    1
#define  Histogram_Control_1_e_Flag_Enabled_STRING     "Histogram_Control_1_e_Flag_Enabled"

//Histogram_Status_0_u16_DarkestBin_R
#define  Histogram_Status_0_u16_DarkestBin_R_Byte_0  0x1230
#define  Histogram_Status_0_u16_DarkestBin_R_Byte_1  0x1231
#define  Histogram_Status_0_u16_DarkestBin_R_SIZE    2
#define  Histogram_Status_0_u16_DarkestBin_R_STRING     "Histogram_Status_0_u16_DarkestBin_R"

//Histogram_Status_1_u16_DarkestBin_R
#define  Histogram_Status_1_u16_DarkestBin_R_Byte_0  0x124c
#define  Histogram_Status_1_u16_DarkestBin_R_Byte_1  0x124d
#define  Histogram_Status_1_u16_DarkestBin_R_SIZE    2
#define  Histogram_Status_1_u16_DarkestBin_R_STRING     "Histogram_Status_1_u16_DarkestBin_R"

//Histom_Control_u16_KneePoints_0
#define  Histom_Control_u16_KneePoints_0_Byte_0  0x59fc
#define  Histom_Control_u16_KneePoints_0_Byte_1  0x59fd
#define  Histom_Control_u16_KneePoints_0_SIZE    2
#define  Histom_Control_u16_KneePoints_0_STRING     "Histom_Control_u16_KneePoints_0"

//InputInterface_Exposure_0_u16_CurrentExposureTime
#define  InputInterface_Exposure_0_u16_CurrentExposureTime_Byte_0  0x1294
#define  InputInterface_Exposure_0_u16_CurrentExposureTime_Byte_1  0x1295
#define  InputInterface_Exposure_0_u16_CurrentExposureTime_SIZE    2
#define  InputInterface_Exposure_0_u16_CurrentExposureTime_STRING     "InputInterface_Exposure_0_u16_CurrentExposureTime"

//InputInterface_Exposure_0_u16_AnalogGainCodeGreen
#define  InputInterface_Exposure_0_u16_AnalogGainCodeGreen_Byte_0  0x1296
#define  InputInterface_Exposure_0_u16_AnalogGainCodeGreen_Byte_1  0x1297
#define  InputInterface_Exposure_0_u16_AnalogGainCodeGreen_SIZE    2
#define  InputInterface_Exposure_0_u16_AnalogGainCodeGreen_STRING     "InputInterface_Exposure_0_u16_AnalogGainCodeGreen"

//InputInterface_Exposure_0_u16_AnalogGainCodeRed
#define  InputInterface_Exposure_0_u16_AnalogGainCodeRed_Byte_0  0x1298
#define  InputInterface_Exposure_0_u16_AnalogGainCodeRed_Byte_1  0x1299
#define  InputInterface_Exposure_0_u16_AnalogGainCodeRed_SIZE    2
#define  InputInterface_Exposure_0_u16_AnalogGainCodeRed_STRING     "InputInterface_Exposure_0_u16_AnalogGainCodeRed"

//InputInterface_Exposure_0_u16_AnalogGainCodeBlue
#define  InputInterface_Exposure_0_u16_AnalogGainCodeBlue_Byte_0  0x129a
#define  InputInterface_Exposure_0_u16_AnalogGainCodeBlue_Byte_1  0x129b
#define  InputInterface_Exposure_0_u16_AnalogGainCodeBlue_SIZE    2
#define  InputInterface_Exposure_0_u16_AnalogGainCodeBlue_STRING     "InputInterface_Exposure_0_u16_AnalogGainCodeBlue"

//InputInterface_Exposure_0_u16_DigitalGainCodeBlue
#define  InputInterface_Exposure_0_u16_DigitalGainCodeBlue_Byte_0  0x129c
#define  InputInterface_Exposure_0_u16_DigitalGainCodeBlue_Byte_1  0x129d
#define  InputInterface_Exposure_0_u16_DigitalGainCodeBlue_SIZE    2
#define  InputInterface_Exposure_0_u16_DigitalGainCodeBlue_STRING     "InputInterface_Exposure_0_u16_DigitalGainCodeBlue"

//InputInterface_Exposure_0_u16_DigitalGainCodeGreen
#define  InputInterface_Exposure_0_u16_DigitalGainCodeGreen_Byte_0  0x129e
#define  InputInterface_Exposure_0_u16_DigitalGainCodeGreen_Byte_1  0x129f
#define  InputInterface_Exposure_0_u16_DigitalGainCodeGreen_SIZE    2
#define  InputInterface_Exposure_0_u16_DigitalGainCodeGreen_STRING     "InputInterface_Exposure_0_u16_DigitalGainCodeGreen"

//InputInterface_Exposure_0_u16_DigitalGainCodeRed
#define  InputInterface_Exposure_0_u16_DigitalGainCodeRed_Byte_0  0x12a0
#define  InputInterface_Exposure_0_u16_DigitalGainCodeRed_Byte_1  0x12a1
#define  InputInterface_Exposure_0_u16_DigitalGainCodeRed_SIZE    2
#define  InputInterface_Exposure_0_u16_DigitalGainCodeRed_STRING     "InputInterface_Exposure_0_u16_DigitalGainCodeRed"

//InputInterface_Exposure_1_u16_CurrentExposureTime
#define  InputInterface_Exposure_1_u16_CurrentExposureTime_Byte_0  0x12a6
#define  InputInterface_Exposure_1_u16_CurrentExposureTime_Byte_1  0x12a7
#define  InputInterface_Exposure_1_u16_CurrentExposureTime_SIZE    2
#define  InputInterface_Exposure_1_u16_CurrentExposureTime_STRING     "InputInterface_Exposure_1_u16_CurrentExposureTime"

//InputInterface_Exposure_1_u16_AnalogGainCodeGreen
#define  InputInterface_Exposure_1_u16_AnalogGainCodeGreen_Byte_0  0x12a8
#define  InputInterface_Exposure_1_u16_AnalogGainCodeGreen_Byte_1  0x12a9
#define  InputInterface_Exposure_1_u16_AnalogGainCodeGreen_SIZE    2
#define  InputInterface_Exposure_1_u16_AnalogGainCodeGreen_STRING     "InputInterface_Exposure_1_u16_AnalogGainCodeGreen"

//InputInterface_Exposure_1_u16_AnalogGainCodeRed
#define  InputInterface_Exposure_1_u16_AnalogGainCodeRed_Byte_0  0x12aa
#define  InputInterface_Exposure_1_u16_AnalogGainCodeRed_Byte_1  0x12ab
#define  InputInterface_Exposure_1_u16_AnalogGainCodeRed_SIZE    2
#define  InputInterface_Exposure_1_u16_AnalogGainCodeRed_STRING     "InputInterface_Exposure_1_u16_AnalogGainCodeRed"

//InputInterface_Exposure_1_u16_AnalogGainCodeBlue
#define  InputInterface_Exposure_1_u16_AnalogGainCodeBlue_Byte_0  0x12ac
#define  InputInterface_Exposure_1_u16_AnalogGainCodeBlue_Byte_1  0x12ad
#define  InputInterface_Exposure_1_u16_AnalogGainCodeBlue_SIZE    2
#define  InputInterface_Exposure_1_u16_AnalogGainCodeBlue_STRING     "InputInterface_Exposure_1_u16_AnalogGainCodeBlue"

//InputInterface_Exposure_1_u16_DigitalGainCodeBlue
#define  InputInterface_Exposure_1_u16_DigitalGainCodeBlue_Byte_0  0x12ae
#define  InputInterface_Exposure_1_u16_DigitalGainCodeBlue_Byte_1  0x12af
#define  InputInterface_Exposure_1_u16_DigitalGainCodeBlue_SIZE    2
#define  InputInterface_Exposure_1_u16_DigitalGainCodeBlue_STRING     "InputInterface_Exposure_1_u16_DigitalGainCodeBlue"

//InputInterface_Exposure_1_u16_DigitalGainCodeGreen
#define  InputInterface_Exposure_1_u16_DigitalGainCodeGreen_Byte_0  0x12b0
#define  InputInterface_Exposure_1_u16_DigitalGainCodeGreen_Byte_1  0x12b1
#define  InputInterface_Exposure_1_u16_DigitalGainCodeGreen_SIZE    2
#define  InputInterface_Exposure_1_u16_DigitalGainCodeGreen_STRING     "InputInterface_Exposure_1_u16_DigitalGainCodeGreen"

//InputInterface_Exposure_1_u16_DigitalGainCodeRed
#define  InputInterface_Exposure_1_u16_DigitalGainCodeRed_Byte_0  0x12b2
#define  InputInterface_Exposure_1_u16_DigitalGainCodeRed_Byte_1  0x12b3
#define  InputInterface_Exposure_1_u16_DigitalGainCodeRed_SIZE    2
#define  InputInterface_Exposure_1_u16_DigitalGainCodeRed_STRING     "InputInterface_Exposure_1_u16_DigitalGainCodeRed"

//InputInterface_FrameDimension_0_e_PixelOrder
#define  InputInterface_FrameDimension_0_e_PixelOrder   0x12bc
#define  InputInterface_FrameDimension_0_e_PixelOrder_Byte_0   0x12bc
#define  InputInterface_FrameDimension_0_e_PixelOrder_SIZE    1
#define  InputInterface_FrameDimension_0_e_PixelOrder_STRING     "InputInterface_FrameDimension_0_e_PixelOrder"

//InputInterface_FrameDimension_1_e_PixelOrder
#define  InputInterface_FrameDimension_1_e_PixelOrder   0x12c4
#define  InputInterface_FrameDimension_1_e_PixelOrder_Byte_0   0x12c4
#define  InputInterface_FrameDimension_1_e_PixelOrder_SIZE    1
#define  InputInterface_FrameDimension_1_e_PixelOrder_STRING     "InputInterface_FrameDimension_1_e_PixelOrder"

//InputInterface_StatusLine_0_e_StatusLine_Control
#define  InputInterface_StatusLine_0_e_StatusLine_Control   0x12a5
#define  InputInterface_StatusLine_0_e_StatusLine_Control_Byte_0   0x12a5
#define  InputInterface_StatusLine_0_e_StatusLine_Control_SIZE    1
#define  InputInterface_StatusLine_0_e_StatusLine_Control_STRING     "InputInterface_StatusLine_0_e_StatusLine_Control"

//InputInterface_StatusLine_1_e_StatusLine_Control
#define  InputInterface_StatusLine_1_e_StatusLine_Control   0x12b7
#define  InputInterface_StatusLine_1_e_StatusLine_Control_Byte_0   0x12b7
#define  InputInterface_StatusLine_1_e_StatusLine_Control_SIZE    1
#define  InputInterface_StatusLine_1_e_StatusLine_Control_STRING     "InputInterface_StatusLine_1_e_StatusLine_Control"

//OffsetCompensation_Control_0_u16_SensorDataPedestal
#define  OffsetCompensation_Control_0_u16_SensorDataPedestal_Byte_0  0x12dc
#define  OffsetCompensation_Control_0_u16_SensorDataPedestal_Byte_1  0x12dd
#define  OffsetCompensation_Control_0_u16_SensorDataPedestal_SIZE    2
#define  OffsetCompensation_Control_0_u16_SensorDataPedestal_STRING     "OffsetCompensation_Control_0_u16_SensorDataPedestal"

//OffsetCompensation_Control_1_u16_SensorDataPedestal
#define  OffsetCompensation_Control_1_u16_SensorDataPedestal_Byte_0  0x12e8
#define  OffsetCompensation_Control_1_u16_SensorDataPedestal_Byte_1  0x12e9
#define  OffsetCompensation_Control_1_u16_SensorDataPedestal_SIZE    2
#define  OffsetCompensation_Control_1_u16_SensorDataPedestal_STRING     "OffsetCompensation_Control_1_u16_SensorDataPedestal"

//OffsetCompensation_Control_2_u16_SensorDataPedestal
#define  OffsetCompensation_Control_2_u16_SensorDataPedestal_Byte_0  0x12f4
#define  OffsetCompensation_Control_2_u16_SensorDataPedestal_Byte_1  0x12f5
#define  OffsetCompensation_Control_2_u16_SensorDataPedestal_SIZE    2
#define  OffsetCompensation_Control_2_u16_SensorDataPedestal_STRING     "OffsetCompensation_Control_2_u16_SensorDataPedestal"

//OffsetCompensation_Control_3_u16_SensorDataPedestal
#define  OffsetCompensation_Control_3_u16_SensorDataPedestal_Byte_0  0x1300
#define  OffsetCompensation_Control_3_u16_SensorDataPedestal_Byte_1  0x1301
#define  OffsetCompensation_Control_3_u16_SensorDataPedestal_SIZE    2
#define  OffsetCompensation_Control_3_u16_SensorDataPedestal_STRING     "OffsetCompensation_Control_3_u16_SensorDataPedestal"

//OffsetCompensation_Control_4_u16_SensorDataPedestal
#define  OffsetCompensation_Control_4_u16_SensorDataPedestal_Byte_0  0x130c
#define  OffsetCompensation_Control_4_u16_SensorDataPedestal_Byte_1  0x130d
#define  OffsetCompensation_Control_4_u16_SensorDataPedestal_SIZE    2
#define  OffsetCompensation_Control_4_u16_SensorDataPedestal_STRING     "OffsetCompensation_Control_4_u16_SensorDataPedestal"

//OffsetCompensation_Control_5_u16_SensorDataPedestal
#define  OffsetCompensation_Control_5_u16_SensorDataPedestal_Byte_0  0x1318
#define  OffsetCompensation_Control_5_u16_SensorDataPedestal_Byte_1  0x1319
#define  OffsetCompensation_Control_5_u16_SensorDataPedestal_SIZE    2
#define  OffsetCompensation_Control_5_u16_SensorDataPedestal_STRING     "OffsetCompensation_Control_5_u16_SensorDataPedestal"

//OffsetCompensation_Control_6_u16_SensorDataPedestal
#define  OffsetCompensation_Control_6_u16_SensorDataPedestal_Byte_0  0x1324
#define  OffsetCompensation_Control_6_u16_SensorDataPedestal_Byte_1  0x1325
#define  OffsetCompensation_Control_6_u16_SensorDataPedestal_SIZE    2
#define  OffsetCompensation_Control_6_u16_SensorDataPedestal_STRING     "OffsetCompensation_Control_6_u16_SensorDataPedestal"

//OffsetCompensation_Control_7_u16_SensorDataPedestal
#define  OffsetCompensation_Control_7_u16_SensorDataPedestal_Byte_0  0x1330
#define  OffsetCompensation_Control_7_u16_SensorDataPedestal_Byte_1  0x1331
#define  OffsetCompensation_Control_7_u16_SensorDataPedestal_SIZE    2
#define  OffsetCompensation_Control_7_u16_SensorDataPedestal_STRING     "OffsetCompensation_Control_7_u16_SensorDataPedestal"

//Pedestal_Control_0_e_Flag_Enable
#define  Pedestal_Control_0_e_Flag_Enable   0x135a
#define  Pedestal_Control_0_e_Flag_Enable_Byte_0   0x135a
#define  Pedestal_Control_0_e_Flag_Enable_SIZE    1
#define  Pedestal_Control_0_e_Flag_Enable_STRING     "Pedestal_Control_0_e_Flag_Enable"

//Pedestal_Control_1_e_Flag_Enable
#define  Pedestal_Control_1_e_Flag_Enable   0x135b
#define  Pedestal_Control_1_e_Flag_Enable_Byte_0   0x135b
#define  Pedestal_Control_1_e_Flag_Enable_SIZE    1
#define  Pedestal_Control_1_e_Flag_Enable_STRING     "Pedestal_Control_1_e_Flag_Enable"

//RubikTop_Control_u16_ColourTemp
#define  RubikTop_Control_u16_ColourTemp_Byte_0  0x13d0
#define  RubikTop_Control_u16_ColourTemp_Byte_1  0x13d1
#define  RubikTop_Control_u16_ColourTemp_SIZE    2
#define  RubikTop_Control_u16_ColourTemp_STRING     "RubikTop_Control_u16_ColourTemp"

//RubikTop_Control_e_Mode_GridSettings
#define  RubikTop_Control_e_Mode_GridSettings   0x13d7
#define  RubikTop_Control_e_Mode_GridSettings_Byte_0   0x13d7
#define  RubikTop_Control_e_Mode_GridSettings_SIZE    1
#define  RubikTop_Control_e_Mode_GridSettings_STRING     "RubikTop_Control_e_Mode_GridSettings"

//RubikTop_Control_e_BufferState
#define  RubikTop_Control_e_BufferState   0x13d8
#define  RubikTop_Control_e_BufferState_Byte_0   0x13d8
#define  RubikTop_Control_e_BufferState_SIZE    1
#define  RubikTop_Control_e_BufferState_STRING     "RubikTop_Control_e_BufferState"

//RubikTop_Control_u8_CornerGain
#define  RubikTop_Control_u8_CornerGain   0x13da

//Rubik_Input_u16_HSensorSize
#define  Rubik_Input_u16_HSensorSize_Byte_0  0x137a
#define  Rubik_Input_u16_HSensorSize_Byte_1  0x137b
#define  Rubik_Input_u16_HSensorSize_SIZE    2
#define  Rubik_Input_u16_HSensorSize_STRING     "Rubik_Input_u16_HSensorSize"

//Rubik_Input_u16_VSensorSize
#define  Rubik_Input_u16_VSensorSize_Byte_0  0x137c
#define  Rubik_Input_u16_VSensorSize_Byte_1  0x137d
#define  Rubik_Input_u16_VSensorSize_SIZE    2
#define  Rubik_Input_u16_VSensorSize_STRING     "Rubik_Input_u16_VSensorSize"

//Rubik_Input_s16_VUserCrop
#define  Rubik_Input_s16_VUserCrop_Byte_0  0x1382
#define  Rubik_Input_s16_VUserCrop_Byte_1  0x1383
#define  Rubik_Input_s16_VUserCrop_SIZE    2
#define  Rubik_Input_s16_VUserCrop_STRING     "Rubik_Input_s16_VUserCrop"

//Rubik_Input_s16_HUserCrop
#define  Rubik_Input_s16_HUserCrop_Byte_0  0x1384
#define  Rubik_Input_s16_HUserCrop_Byte_1  0x1385
#define  Rubik_Input_s16_HUserCrop_SIZE    2
#define  Rubik_Input_s16_HUserCrop_STRING     "Rubik_Input_s16_HUserCrop"

//Rubik_Input_e_Flag_RubikEnable
#define  Rubik_Input_e_Flag_RubikEnable   0x138b
#define  Rubik_Input_e_Flag_RubikEnable_Byte_0   0x138b
#define  Rubik_Input_e_Flag_RubikEnable_SIZE    1
#define  Rubik_Input_e_Flag_RubikEnable_STRING     "Rubik_Input_e_Flag_RubikEnable"

//SMIARx_Control_0_u16_FFormatDescriptorOverride0
#define  SMIARx_Control_0_u16_FFormatDescriptorOverride0_Byte_0  0x1400
#define  SMIARx_Control_0_u16_FFormatDescriptorOverride0_Byte_1  0x1401
#define  SMIARx_Control_0_u16_FFormatDescriptorOverride0_SIZE    2
#define  SMIARx_Control_0_u16_FFormatDescriptorOverride0_STRING     "SMIARx_Control_0_u16_FFormatDescriptorOverride0"

//SMIARx_Control_0_u16_FFormatDescriptorOverride01
#define  SMIARx_Control_0_u16_FFormatDescriptorOverride01_Byte_0  0x1402
#define  SMIARx_Control_0_u16_FFormatDescriptorOverride01_Byte_1  0x1403
#define  SMIARx_Control_0_u16_FFormatDescriptorOverride01_SIZE    2
#define  SMIARx_Control_0_u16_FFormatDescriptorOverride01_STRING     "SMIARx_Control_0_u16_FFormatDescriptorOverride01"

//SMIARx_Control_0_u16_FFormatDescriptorOverride02
#define  SMIARx_Control_0_u16_FFormatDescriptorOverride02_Byte_0  0x1404
#define  SMIARx_Control_0_u16_FFormatDescriptorOverride02_Byte_1  0x1405
#define  SMIARx_Control_0_u16_FFormatDescriptorOverride02_SIZE    2
#define  SMIARx_Control_0_u16_FFormatDescriptorOverride02_STRING     "SMIARx_Control_0_u16_FFormatDescriptorOverride02"

//SMIARx_Control_0_u8_FFModelSubtypeOverride
#define  SMIARx_Control_0_u8_FFModelSubtypeOverride   0x1438
#define  SMIARx_Control_0_u8_FFModelSubtypeOverride_Byte_0   0x1438
#define  SMIARx_Control_0_u8_FFModelSubtypeOverride_SIZE    1
#define  SMIARx_Control_0_u8_FFModelSubtypeOverride_STRING     "SMIARx_Control_0_u8_FFModelSubtypeOverride"

//SMIARx_Control_1_u16_FFormatDescriptorOverride0
#define  SMIARx_Control_1_u16_FFormatDescriptorOverride0_Byte_0  0x1460
#define  SMIARx_Control_1_u16_FFormatDescriptorOverride0_Byte_1  0x1461
#define  SMIARx_Control_1_u16_FFormatDescriptorOverride0_SIZE    2
#define  SMIARx_Control_1_u16_FFormatDescriptorOverride0_STRING     "SMIARx_Control_1_u16_FFormatDescriptorOverride0"

//SMIARx_Control_1_u16_FFormatDescriptorOverride01
#define  SMIARx_Control_1_u16_FFormatDescriptorOverride01_Byte_0  0x1462
#define  SMIARx_Control_1_u16_FFormatDescriptorOverride01_Byte_1  0x1463
#define  SMIARx_Control_1_u16_FFormatDescriptorOverride01_SIZE    2
#define  SMIARx_Control_1_u16_FFormatDescriptorOverride01_STRING     "SMIARx_Control_1_u16_FFormatDescriptorOverride01"

//SMIARx_Control_1_u16_FFormatDescriptorOverride02
#define  SMIARx_Control_1_u16_FFormatDescriptorOverride02_Byte_0  0x1464
#define  SMIARx_Control_1_u16_FFormatDescriptorOverride02_Byte_1  0x1465
#define  SMIARx_Control_1_u16_FFormatDescriptorOverride02_SIZE    2
#define  SMIARx_Control_1_u16_FFormatDescriptorOverride02_STRING     "SMIARx_Control_1_u16_FFormatDescriptorOverride02"

//SMIARx_Control_1_u8_FFModelSubtypeOverride
#define  SMIARx_Control_1_u8_FFModelSubtypeOverride   0x1498
#define  SMIARx_Control_1_u8_FFModelSubtypeOverride_Byte_0   0x1498
#define  SMIARx_Control_1_u8_FFModelSubtypeOverride_SIZE    1
#define  SMIARx_Control_1_u8_FFModelSubtypeOverride_STRING     "SMIARx_Control_1_u8_FFModelSubtypeOverride"

//ToneMap_Control_u8_Strength
#define  ToneMap_Control_u8_Strength   0x14b2
#define  ToneMap_Control_u8_Strength_Byte_0   0x14b2
#define  ToneMap_Control_u8_Strength_SIZE    1
#define  ToneMap_Control_u8_Strength_STRING     "ToneMap_Control_u8_Strength"

//ToneMap_Control_e_Flag_ToneMapEnabled
#define  ToneMap_Control_e_Flag_ToneMapEnabled   0x14b3
#define  ToneMap_Control_e_Flag_ToneMapEnabled_Byte_0   0x14b3
#define  ToneMap_Control_e_Flag_ToneMapEnabled_SIZE    1
#define  ToneMap_Control_e_Flag_ToneMapEnabled_STRING     "ToneMap_Control_e_Flag_ToneMapEnabled"

//ToneMap_Control_e_Flag_UserDefinedCurve
#define  ToneMap_Control_e_Flag_UserDefinedCurve   0x14b4
#define  ToneMap_Control_e_Flag_UserDefinedCurve_Byte_0   0x14b4
#define  ToneMap_Control_e_Flag_UserDefinedCurve_SIZE    1
#define  ToneMap_Control_e_Flag_UserDefinedCurve_STRING     "ToneMap_Control_e_Flag_UserDefinedCurve"
#endif

