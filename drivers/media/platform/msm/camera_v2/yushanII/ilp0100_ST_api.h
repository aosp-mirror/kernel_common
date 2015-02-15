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
 * \file 	ilp0100_ST_api.h
 * \author	sheena jain
 * \brief 	User interface functions usable to control Ilp0100 device.
 */

#ifndef ILP0100_ST_API_H_
#define ILP0100_ST_API_H_

#include "ilp0100_ST_definitions.h"
#include "ilp0100_ST_debugging.h"
#include "ilp0100_customer_platform.h"
#include "ilp0100_customer_sensor_config.h"
#include "ilp0100_customer_iq_settings.h"
#include "ilp0100_ST_error_codes.h"
#include "ilp0100_ST_register_map.h"

/*  Help compiling in C++  */
#ifdef __cplusplus
extern "C"{
#endif   /*__cplusplus*/


/*!
 * \fn			ilp0100_error Ilp0100_init(const Ilp0100_structInit Init, const Ilp0100_structInitFirmware InitFirmware, const Ilp0100_structSensorParams SensorParams)
 * \brief		Ilp0100 Global initialization function.
 * \ingroup		User_Interface_Functions
 * \param[in]	Init
 * \param[in]	InitFirmware
 * \param[in]	SensorParams
 * \retval		ILP0100_ERROR_NONE : Success
 * \retval		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_init(const Ilp0100_structInit Init, Ilp0100_structInitFirmware InitFirmware, const Ilp0100_structSensorParams SensorParams);

/*!
 * \fn			ilp0100_error Ilp0100_defineMode(const Ilp0100_structFrameFormat FrameFormat)
 * \brief		Ilp0100 function to define all video modes used.
 * \ingroup		User_Interface_Functions
 * \param[in] 	FrameFormat
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_defineMode(const Ilp0100_structFrameFormat FrameFormat);

/*!
 * \fn 			ilp0100_error Ilp0100_stop()
 * \brief		Function to stop Sensor Streaming
 * \ingroup		User_Interface_Functions
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_stop(void);

/*!
 * \fn 			iilp0100_error Ilp0100_configDefcorShortOrNormal(const Ilp0100_structDefcorConfig ShortOrNormalDefcorConfig)
 * \brief 		Ilp0100 function to configure Defcor on Short/Normal pipe
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structDefcorConfig
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_configDefcorShortOrNormal(const Ilp0100_structDefcorConfig ShortOrNormalDefcorConfig);

/*!
 * \fn 			ilp0100_error Ilp0100_configDefcorLong(const Ilp0100_structDefcorConfig LongDefcorConfig)
 * \brief 		Ilp0100 function to configure Defcor on Long pipe
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structDefcorConfig
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_configDefcorLong(const Ilp0100_structDefcorConfig LongDefcorConfig);

/*!
 * \fn 			iilp0100_error Ilp0100_updateDefcorShortOrNormal(const Ilp0100_structDefcorParams ShortOrNormalDefcorParams)
 * \brief 		Ilp0100 function to update Defcor parameters on Short/Normal pipe
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structDefcorParams
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_updateDefcorShortOrNormal(const Ilp0100_structDefcorParams ShortOrNormalDefcorParams);

/*!
 * \fn 			ilp0100_error Ilp0100_updateDefcorLong(const Ilp0100_structDefcorParams LongDefcorParams)
 * \brief 		Ilp0100 function to update Defcor parameters on Long pipe
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structDefcorParams
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_updateDefcorLong(const Ilp0100_structDefcorParams LongDefcorParams);

/*!
 * \fn 			ilp0100_error Ilp0100_configChannelOffsetShortOrNormal(const Ilp0100_structChannelOffsetConfig ShortOrNormalChannelOffsetConfig)
 * \brief 		Ilp0100 function to configure Channel Gain on Short/Normal pipe
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structChannelOffsetConfig
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_configChannelOffsetShortOrNormal(const Ilp0100_structChannelOffsetConfig ShortOrNormalChannelOffsetConfig);

/*!
 * \fn 			ilp0100_error Ilp0100_configChannelOffsetLong(const Ilp0100_structChannelOffsetConfig LongChannelOffsetConfig)
 * \brief 		Ilp0100 function to configure Channel Gain on Long pipe
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structChannelOffsetConfig
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_configChannelOffsetLong(const Ilp0100_structChannelOffsetConfig LongChannelOffsetConfig);

/*!
 * \fn 			ilp0100_error Ilp0100_updateChannelOffsetShortOrNormal(const Ilp0100_structChannelOffsetParams ShortOrNormalChannelOffsetParams);
 * \brief 		Ilp0100 function to update Channel Gain parameters on Short/Normal pipe
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structChannelOffsetParams
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_updateChannelOffsetShortOrNormal(const Ilp0100_structChannelOffsetParams ShortOrNormalChannelOffsetParams);

/*!
 * \fn 			ilp0100_error Ilp0100_updateChannelOffsetLong(const Ilp0100_structChannelOffsetParams LongChannelOffsetParams);
 * \brief 		Ilp0100 function to update Channel Gain parameters on Long pipe
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structChannelOffsetParams
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_updateChannelOffsetLong(const Ilp0100_structChannelOffsetParams LongChannelOffsetParams);
/*!
 * \fn 			ilp0100_error Ilp0100_configGlaceShortOrNormal(const Ilp0100_structGlaceConfig ShortOrNormalGlaceConfig)
 * \brief 		Ilp0100 function to configure Glace on Short/Normal pipe
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structGlaceConfig
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_configGlaceShortOrNormal(const Ilp0100_structGlaceConfig ShortOrNormalGlaceConfig);

/*!
 * \fn 			ilp0100_error Ilp0100_configGlaceLong(const Ilp0100_structGlaceConfig LongGlaceConfig)
 * \brief 		Ilp0100 function to configure Glace on Long pipe
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structGlaceConfig
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_configGlaceLong(const Ilp0100_structGlaceConfig LongGlaceConfig);

/*!
 * \fn 			ilp0100_error Ilp0100_configHistShortOrNormal(const Ilp0100_structHistConfig ShortOrNormalHistConfig)
 * \brief 		Ilp0100 function to configure Histogram on Short/Normal pipe
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structHistConfig
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_configHistShortOrNormal(const Ilp0100_structHistConfig ShortOrNormalHistConfig);

/*!
 * \fn 			ilp0100_error Ilp0100_configHistLong(const Ilp0100_structHistConfig LongHistConfig)
 * \brief 		Ilp0100 function to configure Histogram on Long pipe
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structHistConfig
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_configHistLong(const Ilp0100_structHistConfig LongHistConfig);

/*!
 * \fn 			ilp0100_error Ilp0100_configHdrMerge(const Ilp0100_structHdrMergeConfig HdrMergeConfig)
 * \brief 		Ilp0100 function to configure HDR Merge
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structHdrMergeConfig
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_configHdrMerge(const Ilp0100_structHdrMergeConfig HdrMergeConfig);

/*!
 * \fn 			ilp0100_error Ilp0100_updateHdrMerge(const Ilp0100_structHdrMergeParams HdrMergeParam)
 * \brief 		Ilp0100 function to update HDR Merge Params
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structHdrMergeParam
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_updateHdrMerge(const Ilp0100_structHdrMergeParams HdrMergeParams);

/*!
 * \fn 			ilp0100_error Ilp0100_configCls(const Ilp0100_structClsConfig ClsConfig)
 * \brief 		Ilp0100 function to configure CLS
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structClsConfig
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_configCls(const Ilp0100_structClsConfig ClsConfig);

/*!
 * \fn 			ilp0100_error Ilp0100_updateCls(const Ilp0100_structClsParams ClsParams)
 * \brief 		Ilp0100 function to update the parameters sent to CLS
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structClsParams
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_updateCls(const Ilp0100_structClsParams ClsParams);

/*!
 * \fn 			ilp0100_error Ilp0100_configToneMapping(const Ilp0100_structToneMappingConfig ToneMappingConfig)
 * \brief 		Ilp0100 function to configure Tone Mapping
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structToneMappingConfig
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_configToneMapping(const Ilp0100_structToneMappingConfig ToneMappingConfig);

/*!
 * \fn 			ilp0100_error Ilp0100_updateToneMapping(const Ilp0100_structToneMappingParams ToneMappingParams)
 * \brief 		Ilp0100 function to update the Tone Mapping Curve
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structToneMappingParams
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_updateToneMapping(const Ilp0100_structToneMappingParams ToneMappingParams);

/*!
 * \fn 			ilp0100_error Ilp0100_updateSensorParamsShortOrNormal(const Ilp0100_structFrameParams ShortOrNormalFrameParams)
 * \brief 		Ilp0100 function to update exposure/Gains parameters from sensor
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structFrameParams
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_updateSensorParamsShortOrNormal(const Ilp0100_structFrameParams ShortOrNormalFrameParams);

/*!
 * \fn 			ilp0100_error Ilp0100_updateSensorParamsLong(const Ilp0100_structFrameParams LongFrameParams)
 * \brief 		Ilp0100 function to update Hdr exposure/Gains parameters from sensor
 * \ingroup		User_Interface_Functions
 * \param[in] 	Ilp0100_structFrameParams
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_updateSensorParamsLong(const Ilp0100_structFrameParams LongFrameParams);

/*!
 * \fn 			ilp0100_error Ilp0100_readBackGlaceStatisticsShortOrNormal(Ilp0100_structGlaceStatsData *pShortOrNormalGlaceStatsData);
 * \brief 		Read back Glace statistics
 * \ingroup		User_Interface_Functions
 * \param[out] 	pShortOrNormalGlaceStatsData
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_readBackGlaceStatisticsShortOrNormal(Ilp0100_structGlaceStatsData *pShortOrNormalGlaceStatsData);

/*!
 * \fn 			ilp0100_error Ilp0100_readBackGlaceStatisticsLong(Ilp0100_structGlaceStatsData *pLongGlaceStatsData);
 * \brief 		Read back Glace Long statistics
 * \ingroup		User_Interface_Functions
 * \param[out] 	pLongGlaceStatsData
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_readBackGlaceStatisticsLong(Ilp0100_structGlaceStatsData *pLongGlaceStatsData);

/*!
 * \fn 			ilp0100_error Ilp0100_readBackHistStatisticsShortOrNormal(Ilp0100_structHistStatsData *pShortOrNormalHistStatsData);
 * \brief 		Read back Hist statistics
 * \ingroup		User_Interface_Functions
 * \param[out]	pShortOrNormalHistStatsData
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_readBackHistStatisticsShortOrNormal(Ilp0100_structHistStatsData *pShortOrNormalHistStatsData);

/*!
 * \fn 			ilp0100_error Ilp0100_readBackHistStatisticsLong(Ilp0100_structHistStatsData *pLongHistStatsData);
 * \brief 		Read back Hist Long statistics
 * \ingroup		User_Interface_Functions
 * \param[out] 	pLongHistStatsData
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_readBackHistStatisticsLong(Ilp0100_structHistStatsData *pLongHistStatsData);

/*!
 * \fn 			ilp0100_error Ilp0100_setVirtualChannelShortOrNormal(uint8_t VirtualChannel);
 * \brief 		Set Virtual Channel Short Or Normal
 * \ingroup		User_Interface_Functions
 * \param[in]	VirtualChannel
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_setVirtualChannelShortOrNormal(uint8_t VirtualChannel);

/*!
 * \fn 			ilp0100_error Ilp0100_setVirtualChannelLong(uint8_t VirtualChannel);
 * \brief 		Set Virtual Channel Long
 * \ingroup		User_Interface_Functions
 * \param[in] 	VirtualChannel
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_setVirtualChannelLong(uint8_t VirtualChannel);

/*!
 * \fn 			ilp0100_error Ilp0100_setHDRFactor(uint8_t HDRFactor)
 * \brief		Set HDR Factor
 * \ingroup		User_Interface_Functions
 * \param[in] 	HDRFactor [0:100]
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_setHDRFactor(uint8_t HDRFactor);

/*!
 * \fn 			ilp0100_error Ilp0100_reset()
 * \brief 		Function to reset completely Ilp0100
 * \ingroup		User_Interface_Functions
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_reset(void);

/*!
 * \fn 			ilp0100_error Ilp0100_onTheFlyReset()
 * \brief 		Function to help Ilp0100 to recover from wrong data sent by sensor and
 * 		  		Resynchronizes automatically on following frame,
 *        		Must be checked if this function can be implemented.
 * \ingroup		User_Interface_Functions
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_onTheFlyReset(void);

/*!
 * \fn 			ilp0100_error Ilp0100_interruptManager()
 * \brief 		Interrupt functions
 * 		  		Ilp0100 API function to be called when an interrupt have been raised by Ilp0100 to host
 *        		Two behavior possible:
 *        		- Interrupts types (Status, error, details on interrupts
 *        		- Interrupts are directly served by Ilp0100_InterruptManager function.
 * \ingroup		User_Interface_Functions
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
//ilp0100_error Ilp0100_interruptManager();

/*!
* \fn 			ilp0100_error Ilp0100_interruptEnable(uint32_t InterruptSetMask, bool_t Pin)
 * \brief		Function to enable Interrupts
 * \ingroup		User_Interface_Functions
 * \param[in] 	InterruptSetMask	: Mask for Interrupts to be enabled
 * \param[in]	Pin	: GPIO PIn Value 0 or 1
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_interruptEnable(uint32_t InterruptSetMask, bool_t Pin);

/*!
 * \fn 			ilp0100_error Ilp0100_core_interruptDisable(uint32_t InterruptClrMask, bool_t Pin)
 * \brief		Function to disable Interrupts
 * \ingroup		User_Interface_Functions
 * \param[in] 	InterruptClrMask	: Mask for Interrupts to be disabled
 * \param[in]	Pin	: GPIO PIn Value 0 or 1
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_interruptDisable(uint32_t InterruptClrMask, bool_t Pin);

/*!
 * \fn 			ilp0100_error Ilp0100_interruptReadStatus(uint32_t* pInterruptId, uint8_t Pin)
 * \brief		Function to read Interrupt Status
 * \ingroup		User_Interface_Functions
 * \param[out] 	pInterruptId : InterruptId of raised Interrupts
 * \param[in]	Pin	: GPIO PIn Value 0 or 1
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_interruptReadStatus(uint32_t* pInterruptId, uint8_t Pin);

/*!
 * \fn 			ilp0100_error Ilp0100_interruptClearStatus(uint32_t InterruptId, uint8_t Pin)
 * \brief		Function to clear Interrupt status
 * \ingroup		User_Interface_Functions
 * \param[in] 	InterruptId
 * \param[in]	Pin	: GPIO PIn Value 0 or 1
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_interruptClearStatus(uint32_t InterruptId, uint8_t Pin);


/*!
 * \fn 			ilp0100_error Ilp0100_getApiVersionNumber(uint8_t* pMajorNumber, uint8_t* pMinorNumber);
 * \brief 		Function to retrieve API Version No.
 * \ingroup		User_Interface_Functions
 * \param[out] 	pMajorNumber
 * \param[out] 	pminorNumber
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_getApiVersionNumber(uint8_t* pMajorNumber, uint8_t* pMinorNumber);

/*!
 * \fn 			ilp0100_error Ilp0100_getFirmwareVersionumber(uint8_t* pMajorNumber, uint8_t* pMinorNumber)
 * \brief		Function to read back Firmware Version Number
 * \ingroup		User_Interface_Functions
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_getFirmwareVersionumber(uint8_t* pMajorNumber, uint8_t* pMinorNumber);

/*!
 * \fn 			ilp0100_error Ilp0100_startTestMode(uint16_t IpsBypass, uint16_t TestMode)
 * \brief 		Debug test mode.
 * \ingroup		User_Interface_Functions
 * \param[in] 	IpsBypass
 * \param[in] 	TestMode
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_startTestMode(uint16_t IpsBypass, uint16_t TestMode);

/*!
 * \fn 			ilp0100_error Ilp0100_readRegister(uint32_t RegisterName, uint8_t *pData)
 * \brief 		Device register read function
 * \ingroup		User_Interface_Functions
 * \param[in] 	RegisterName
 * \param[in] 	pData
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_readRegister(uint32_t RegisterName, uint8_t *pData);

/*!
 * \fn 			ilp0100_error Ilp0100_writeRegister(uint32_t RegisterName, uint8_t *pData)
 * \brief 		Device register write function
 * \ingroup		User_Interface_Functions
 * \param[in] 	RegisterName
 * \param[in] 	pData
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_writeRegister(uint32_t RegisterName, uint8_t *pData);

/*!
 * \fn 			ilp0100_error Ilp0100_enableIlp0100SensorClock(bool_t SensorInterface)
 * \brief 		Enable Sensor Clock Function
 * \ingroup		User_Interface_Functions
 * \param[in] 	SensorInterface :	SENSOR_0 or SENSOR_1, for primary/secondary Sensor
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_enableIlp0100SensorClock(bool_t SensorInterface);

/*!
 * \fn 			ilp0100_error Ilp0100_disableIlp0100SensorClock(bool_t SensorInterface)
 * \brief 		Disable Sensor Clock Function
 * \ingroup		User_Interface_Functions
 * \param[in] 	SensorInterface :	SENSOR_0 or SENSOR_1, for primary/secondary Sensor
 * \retval 		ILP0100_ERROR_NONE : Success
 * \retval 		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_disableIlp0100SensorClock(bool_t SensorInterface);

/* help compiling in C++  */
#ifdef __cplusplus
}
#endif   /*__cplusplus*/





#endif /* ILP0100_ST_API_H_ */
