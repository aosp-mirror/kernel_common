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
 * \file ilp0100_ST_core.h
 * \brief Internal Ilp0100 functions used for correct driving of the device
 * \author sheena jain
 */

#ifndef ILP0100_ST_CORE_H_
#define ILP0100_ST_CORE_H_

#include "ilp0100_ST_api.h"
#include "ilp0100_ST_definitions.h"
#include "ilp0100_ST_firmware_elements.h"

//! Fixed addresses for SPI Window 0 and 1
#define 	SPI_BASE_ADD_0		0x8000
#define 	SPI_BASE_ADD_1 		0xC000


//! Header for Binary Files
typedef struct {
                uint32_t magic_id ; // 'STCO'or 'STDA'
                uint32_t load_add ;
                uint32_t code_size ;
                uint16_t crc ;
                uint16_t file_offset ;
                uint8_t  major_version;
                uint8_t  minor_version;
                uint16_t pad ;

} file_hdr_t;
/*!
 * \fn 			ilp0100_error Ilp0100_core_initClocks(const Ilp0100_structInit Init, const Ilp0100_structSensorParams SensorParams)
 * \brief 		Ilp0100 Core function to initialise Pll Clock
 * \ingroup		Core_Functions
 * \param[in]	Init : Structure containing Initialisation information
 * \param[in]	SensorParams : Structure containing sensor params
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_initClocks(const Ilp0100_structInit Init, const Ilp0100_structSensorParams SensorParams);

/*!
 * \fn 			ilp0100_error Ilp0100_core_uploadFirmware(const Ilp0100_structInitFirmware InitFirmware)
 * \brief		Core function to upload Firmware Code, Calib. Data and Part to part calib Data.
 * \ingroup		Core_Functions
 * \param[in]	InitFirmware : Structure containing pointers to binary firmware code and Data
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_uploadFirmware(const Ilp0100_structInitFirmware InitFirmware);

/*!
 * \fn			ilp0100_error Ilp0100_core_checkCrc(uint8_t* pData, uint32_t SizeInBytes, uint16_t CrcExpected)
 * \brief		Core function to check Crc of binary data
 * \ingroup		Core_Functions
 * \param[in]	pData
 * \param[in]	SizeInBytes
 * \param[in]	CrcExpected : Expected Crc of Binary Data
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_checkCrc(uint8_t* pData, uint32_t SizeInBytes, uint16_t CrcExpected);

/*!
 * \fn 			ilp0100_error Ilp0100_core_bootXp70()
 * \brief		Core function to boot xp70
 * \ingroup		Core_Functions
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_bootXp70(void);

/*!
 * \fn			ilp0100_error Ilp0100_core_defineMode(const Ilp0100_structFrameFormat FrameFormat)
 * \brief		Core function to define Video Mode/FrameFormat used.
 * \ingroup		Core_Functions
 * \param[in]	FrameFormat
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_defineMode(const Ilp0100_structFrameFormat FrameFormat);

/*!
 * \fn 			ilp0100_error Ilp0100_core_stop()
 * \brief		Core function to stop Sensor Streaming
 * \ingroup		Core_Functions
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_stop(void);

/*!
 * \fn 			ilp0100_error Ilp0100_core_configDefcor(const Ilp0100_structDefcorConfig DefcorConfig, uint8_t HDRMode)
 * \brief 		Core function to configure Defcor.
 * \ingroup		Core_Functions
 * \param[in] 	Ilp0100_structDefcorConfig
 * \param[in] 	HDRMode
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_configDefcor(const Ilp0100_structDefcorConfig DefcorConfig, uint8_t HDRMode);

/*!
 * \fn 			ilp0100_error Ilp0100_core_updateDefcor(const Ilp0100_structDefcorParams DefcorParams, uint8_t HDRMode)
 * \brief 		Core function to update Defcor parameters.
 * \ingroup		Core_Functions
 * \param[in] 	Ilp0100_structDefcorParams
 * \param[in] 	HDRMode
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_updateDefcor(const Ilp0100_structDefcorParams DefcorParams, uint8_t HDRMode);

/*!
 * \fn 			ilp0100_error Ilp0100_core_configChannelOffset(const Ilp0100_structChannelOffsetConfig ChannelOffsetConfig, uint8_t HDRMode)
 * \brief 		Core function to configure Channel Gain.
 * \ingroup		Core_Functions
 * \param[in] 	Ilp0100_structChannelOffsetConfig
 * \param[in] 	HDRMode
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_configChannelOffset(const Ilp0100_structChannelOffsetConfig ChannelOffsetConfig, uint8_t HDRMode);

/*!
 * \fn 			ilp0100_error Ilp0100_core_updateChannelOffset(const Ilp0100_structChannelOffsetParams ChannelOffsetParams, uint8_t HDRMode)
 * \brief 		Core function to update Channel Gain parameters.
 * \ingroup		Core_Functions
 * \param[in] 	Ilp0100_structChannelOffsetParams
 * \param[in] 	HDRMode
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_updateChannelOffset(const Ilp0100_structChannelOffsetParams ChannelOffsetParams, uint8_t HDRMode);

/*!
 * \fn 			ilp0100_error Ilp0100_core_configGlace(const Ilp0100_structGlaceConfig GlaceConfig, uint8_t HDRMode)
 * \brief 		Core function to configure Glace.
 * \ingroup		Core_Functions
 * \param[in] 	Ilp0100_structGlaceConfig
 * \param[in] 	HDRMode
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_configGlace(const Ilp0100_structGlaceConfig GlaceConfig, uint8_t HDRMode);

/*!
 * \fn 			ilp0100_error Ilp0100_core_configHist(const Ilp0100_structHistConfig HistConfig, uint8_t HDRMode)
 * \brief 		Core function to configure Histogram.
 * \ingroup		Core_Functions
 * \param[in] 	Ilp0100_structHistConfig
 * \param[in] 	HDRMode
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_configHist(const Ilp0100_structHistConfig HistConfig, uint8_t HDRMode);

/*!
 * \fn 			ilp0100_error Ilp0100_core_configHdrMerge(const Ilp0100_structHdrMergeConfig HdrMergeConfig)
 * \brief 		Core function to configure HDR Merge.
 * \ingroup		Core_Functions
 * \param[in] 	Ilp0100_structHdrMergeConfig
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_configHdrMerge(const Ilp0100_structHdrMergeConfig HdrMergeConfig);

/*!
 * \fn 			ilp0100_error Ilp0100_core_updateHdrMerge(const Ilp0100_structHdrMergeParams HdrMergeParams)
 * \brief 		Core function to update HDR Merge Params.
 * \ingroup		Core_Functions
 * \param[in] 	Ilp0100_structHdrMergeParams
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_updateHdrMerge(const Ilp0100_structHdrMergeParams HdrMergeParams);

/*!
 * \fn 			ilp0100_error Ilp0100_core_configCls(const Ilp0100_structClsConfig ClsConfig)
 * \brief 		Core function to configure CLS
 * \ingroup		Core_Functions
 * \param[in] 	Ilp0100_structClsConfig
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_configCls(const Ilp0100_structClsConfig ClsConfig);

/*!
 * \fn 			ilp0100_error Ilp0100_core_updateCls(const Ilp0100_structClsParams ClsParams)
 * \brief 		Ilp0100 function to update the parameters sent to CLS
 * \ingroup		Core_Functions
 * \param[in] 	Ilp0100_structClsParams
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_updateCls(const Ilp0100_structClsParams ClsParams);

/*!
 * \fn 			ilp0100_error Ilp0100_core_configToneMapping(const Ilp0100_structToneMappingConfig ToneMappingConfig)
 * \brief 		Core function to configure Tone Mapping
 * \ingroup		Core_Functions
 * \param[in] 	Ilp0100_structToneMappingConfig
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_configToneMapping(const Ilp0100_structToneMappingConfig ToneMappingConfig);

/*!
 * \fn 			ilp0100_error Ilp0100_core_updateToneMapping(const Ilp0100_structToneMappingParams ToneMappingParams)
 * \brief 		Core function to update the Tone Mapping Curve
 * \ingroup		Core_Functions
 * \param[in] 	Ilp0100_structToneMappingParams
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_updateToneMapping(const Ilp0100_structToneMappingParams ToneMappingParams);

/*!
 * \fn 			ilp0100_error Ilp0100_core_updateSensorParams(const Ilp0100_structFrameParams FrameParams, uint8_t HDRMode)
 * \brief 		Core function to update exposure/Gains parameters from sensor
 * \ingroup		Core_Functions
 * \param[in] 	FrameParams
 * \param[in]	HDRMode
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_updateSensorParams(const Ilp0100_structFrameParams FrameParams, uint8_t HDRMode);

/*!
 * \fn 			ilp0100_error Ilp0100_core_startTestMode(uint16_t IpsBypass, uint16_t TestMode)
 * \brief		Core function to start Test Mode.
 * \ingroup		Core_Functions
 * \param[in] 	IpsBypass
 * \param[in]	TestMode
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_startTestMode(uint16_t IpsBypass, uint16_t TestMode);

/*!
 * \fn 			Ilp0100_core_readBackGlaceStatistics(Ilp0100_structGlaceStatsData *pGlaceStatsData, uint8_t HDRMode);
 * \brief		Core function to read back Glace statistics
 * \ingroup		Core_Functions
 * \param[out] 	pGlaceStatsData
 * \param[in]	HDRMode
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_readBackGlaceStatistics(Ilp0100_structGlaceStatsData *pGlaceStatsData, uint8_t HDRMode);

/*!
 * \fn 			Ilp0100_core_readBackHistStatistics(Ilp0100_structHistStatsData *pHistStatsData, uint8_t HDRMode);
 * \brief		Core function to read back Hist statistics
 * \ingroup		Core_Functions
 * \param[out]	pHistStatsData
 * \param[in]	HDRMode
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_readBackHistStatistics(Ilp0100_structHistStatsData *pHistStatsData, uint8_t HDRMode);

/*!
 * \fn 			ilp0100_error Ilp0100_core_setVirtualChannel(uint8_t VirtualChannel, uint8_t HDRMode)
 * \brief		Core function to set Virtual Channel
 * \ingroup		Core_Functions
 * \param[in] 	VirtualChannel
 * \param[in]	HDRMode
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_setVirtualChannel(uint8_t VirtualChannel, uint8_t HDRMode);

/*!
 * \fn 			ilp0100_error Ilp0100_core_setHDRFactor(uint8_t HDRFactor)
 * \brief		Core function to set HDR Factor
 * \ingroup		Core_Functions
 * \param[in] 	HDRFactor [0:100]
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_setHDRFactor(uint8_t HDRFactor);

/*!
 * \fn 			ilp0100_error Ilp0100_core_reset()
 * \brief		Core function to reset Ilp0100 completely.
 * \ingroup		Core_Functions
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_reset(void);

/*!
 * \fn 			ilp0100_error Ilp0100_core_onTheFlyReset()
 * \brief		Core function to help Ilp0100 to recover from wrong data sent by sensor
 * 				and Resynchronizes automatically on following frame
 * 				(Must be checked if this function can be implemented ...)
 * \ingroup		Core_Functions
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_onTheFlyReset(void);

/*!
 * \fn 			ilp0100_error Ilp0100_core_interruptEnable(uint32_t InterruptSetMask, bool_t Pin)
 * \brief		Core function to enable Interrupts
 * \ingroup		Core_Functions
 * \param[in] 	InterruptSetMask	: Mask for Interrupts to be enabled
 * \param[in]	Pin	: GPIO PIn Value 0 or 1
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_interruptEnable(uint32_t InterruptSetMask, bool_t Pin);

/*!
 * \fn 			ilp0100_error Ilp0100_core_interruptDisable(uint32_t InterruptClrMask, bool_t Pin)
 * \brief		Core function to disable Interrupts
 * \ingroup		Core_Functions
 * \param[in] 	InterruptClrMask	: Mask for Interrupts to be disabled
 * \param[in]	Pin	: GPIO PIn Value 0 or 1
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_interruptDisable(uint32_t InterruptClrMask, bool_t Pin);

/*!
 * \fn 			Ilp0100_core_interruptReadStatus(uint32_t* pInterruptId, uint8_t Pin)
 * \brief		Core function to read Interrupt Status
 * \ingroup		Core_Functions
 * \param[out] 	pInterruptId : InterruptId of raised Interrupt
 * \param[in]	Pin	: GPIO PIn Value 0 or 1
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_interruptReadStatus(uint32_t* pInterruptId, uint8_t Pin);

/*!
 * \fn 			Ilp0100_core_interruptClearStatus(uint32_t InterruptId, uint8_t Pin)
 * \brief		Core function to clear Interrupt status
 * \ingroup		Core_Functions
 * \param[in] 	InterruptId
 * \param[in]	Pin	: GPIO PIn Value 0 or 1
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_interruptClearStatus(uint32_t InterruptId, uint8_t Pin);

/*!
 * \fn 			ilp0100_error Ilp0100_core_getApiVersionNumber(uint8_t* pMajorNumber, uint8_t* pMinorNumber)
 * \brief		Core function to read back Api Version Number
 * \ingroup		Core_Functions
 * \param[out] 	pMajorNumber
 * \param[out]	pMinorNumber
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_getApiVersionNumber(uint8_t* pMajorNumber, uint8_t* pMinorNumber);

/*!
 * \fn 			ilp0100_error Ilp0100_core_getFirmwareVersionumber(uint8_t* pMajorNumber, uint8_t* pMinorNumber)
 * \brief		Core function to read back Firmware Version Number
 * \ingroup		Core_Functions
 * \param[out] 	pMajorNumber
 * \param[out]	pMinorNumber
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_getFirmwareVersionumber(uint8_t* pMajorNumber, uint8_t* pMinorNumber);

/*!
 * \fn 			ilp0100_error Ilp0100_core_readRegister(uint16_t registerName, uint8_t *pData)
 * \brief		Core function to read Register.
 * \ingroup		Core_Functions
 * \param[in] 	RegisterName
 * \param[in]	Count
 * \param[out]	pData
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_readRegister(uint16_t RegisterName, uint16_t Count, uint8_t *pData);

/*!
 * \fn 			ilp0100_error Ilp0100_core_writeRegister(uint16_t registerName, uint8_t *pData)
 * \brief		Core function to write Register.
 * \ingroup		Core_Functions
 * \param[in] 	RegisterName
 * \param[in]	Count
 * \param[in]	pData
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_writeRegister(uint16_t RegisterName, uint16_t Count, uint8_t *pData);

/*!
 * \fn 			ilp0100_error Ilp0100_core_enableIlp0100SensorClock()
 * \brief		Core function to enable Sensor Clock
 * \ingroup		Core_Functions
 * \param[in]	SensorInterface
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_enableIlp0100SensorClock(bool_t SensorInterface);

/*!
 * \fn 			ilp0100_error Ilp0100_core_disableIlp0100SensorClock()
 * \brief		Core function to disable Sensor Clock
 * \ingroup		Core_Functions
 * \param[in]	SensorInterface
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_disableIlp0100SensorClock(bool_t SensorInterface);

/*!
 * \fn 			ilp0100_error Ilp0100_core_saveSpiWindow1Addr(uint32_t *pAddr)
 * \brief		Core function to save Spi Window 1 Address
 * \ingroup		Core_Functions
 * \param[out] 	pAddr	: Present SPI window Address retrieved and written on pData.
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_saveSpiWindow1Addr(uint32_t *pAddr);

/*!
 * \fn 			ilp0100_error Ilp0100_core_restoreSpiWindow1Addr(uint32_t Addr)
 * \brief		Core function to restore Spi Window 1 Address
 * \ingroup		Core_Functions
 * \param[in] 	Addr : Address to be reprogrammed to SPI Window 1.
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_restoreSpiWindow1Addr(uint32_t Addr);

/*!
 * \fn 			ilp0100_error Ilp0100_core_strCmpr(const char* Str1, const char* Str2, uint8_t *pEqual)
 * \brief		Core function to compare 2 strings
 * \ingroup		Core_Functions
 * \param[in] 	Str1
 * \param[in]	Str2
 * \param[out]	pEqual	: 1 if strings are equal, 0 otherwise
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_core_strCmpr(const char* Str1, const char* Str2, uint8_t *pEqual);

#endif /* ILP0100_ST_CORE_H_ */
