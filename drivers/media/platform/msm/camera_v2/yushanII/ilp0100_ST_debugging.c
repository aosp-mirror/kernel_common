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
 * \file	ilp0100_ST_debugging.c
 * \brief	Internal Ilp0100 functions for device debugging
 * \author	sheena jain
 */

//! \defgroup	Debugging_Functions

#include "ilp0100_ST_debugging.h"

//! \defgroup	Debugging_Functions

// Defines for internal use

#define ILP0100_DEBUG_NONE		0
#define ILP0100_DEBUG_BYTE		1
#define ILP0100_DEBUG_BOOL		1
#define ILP0100_DEBUG_UINT8		1
#define ILP0100_DEBUG_UINT16	2
#define ILP0100_DEBUG_UINT32	3
#define ILP0100_DEBUG_FLOAT		4


#ifdef ILP0100_DEBUG

#define DUMP_PARAM_STRUCT(ParamType, Param) 											Ilp0100_dumpParameter(#ParamType, sizeof(ParamType), Param, ILP0100_DEBUG_NONE)
#define DUMP_PARAM(ParamName, Param, ParamType) 										Ilp0100_dumpParameter(#ParamName, 1, 	 			 Param, ParamType)
#define DUMP_PARAM_PTR(ParamName, Param, ParamSize, ParamType) 							Ilp0100_dumpParameter(#ParamName, ParamSize, 		 Param, ParamType)
#define DUMP_STRUCTPARAM(StructType, StructName, ParamName, ParamType) 			  		Ilp0100_dumpParameter(#ParamName, 1, (uint8_t *)&(((StructType*)StructName)->ParamName), ParamType);
#define DUMP_STRUCTPARAM_PTR(StructType, StructName, ParamName, ParamSize, ParamType) 	Ilp0100_dumpParameter(#ParamName, ParamSize, (uint8_t *)(((StructType*)StructName)->ParamName) , ParamType);
#define DUMP_STRUCTPARAM_ARRAY(StructType, StructName, ParamName, ParamSize, ParamType) 	Ilp0100_dumpParameter(#ParamName, ParamSize, (uint8_t *)(&(((StructType*)StructName)->ParamName[0])) , ParamType);


#define ILP0100_DEBUG_TEST_STR_EQUALITY(Str1, Str2, UINT8_VAR) Ilp0100_core_strCmpr(Str1, Str2, &UINT8_VAR);if(UINT8_VAR)



/* private functions for debugging */
ilp0100_error Ilp0100_dumpParameters(const char* pFunctionName, void **pFuncArguments);
ilp0100_error Ilp0100_dumpParameter(const char* pParamName, const uint16_t ParamLength, const uint8_t *pParamValues, const uint8_t ParamType);
ilp0100_error Ilp0100_isApiCoreFunction(const char* pFunctionName, bool_t* isCoreFunc);


/* Global variables used only by debug system */
uint8_t* pIlp0100DebugBuffer=0;
uint32_t Ilp0100DebugLogSize=0;
bool_t	 Ilp0100DebugStarted=FALSE;

int8_t	 CurrentLevel=0;
char	 FunctionsLevel[20][45];

/**************************************************************/
/*	Debugging Functions										  */
/**************************************************************/
/*!
 * \fn			Ilp0100_loggingOpen()
 * \brief		Initialize debug sequence
 * \retval		ILP0100_ERROR_NONE : Success
 * \retval		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_loggingOpen()
{
	ilp0100_error Status = ILP0100_ERROR_NONE;
	/* Creates the buffer */
	/* With maximum allowable size */
	pIlp0100DebugBuffer = (uint8_t *)ILP0100_BUFFERCREATE(ILP0100_MAX_DEBUG_BUFFER_SIZE);
	Ilp0100DebugLogSize = 0;
	CurrentLevel = 0;

	if(pIlp0100DebugBuffer==0) {
		Status = ILP_0100_DEBUG_BUFFER_CREATION_ERROR;
	}

	if(Status == ILP0100_ERROR_NONE){
		/* Write header file of debugging file */
		ILP0100_DEBUG_WRITE_IN_LOG("<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n");
		ILP0100_DEBUG_WRITE_IN_LOG("<Ilp0100_Logging>\n");
	} else {
		return ILP_0100_DEBUG_BUFFER_CREATION_ERROR;
	}
	return ILP0100_ERROR_NONE;
}

/*!
 * \fn			Ilp0100_loggingClose()
 * \brief		Initialize debug sequence
 * \ingroup		Debugging_Functions
 * \retval		ILP0100_ERROR_NONE : Success
 * \retval		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_loggingClose()
{
	ilp0100_error Status = ILP0100_ERROR_NONE;
	if(Ilp0100DebugStarted){
		Status = Ilp0100_loggingStop();
	}
	if(Status == ILP0100_ERROR_NONE){
		/* Free buffer */
		ILP0100_BUFFERFREE(pIlp0100DebugBuffer);
		pIlp0100DebugBuffer=0;
	}

	return Status;
}

/*!
 * \fn			Ilp0100_loggingStart(uint8_t DebugLevel)
 * \brief		Start logging all ILP activities
 * \ingroup		Debugging_Functions
 * \param[in]	DebugLevel
 * \retval		ILP0100_ERROR_NONE : Success
 * \retval		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_loggingStart(uint8_t DebugLevel)
{
	ilp0100_error Status = ILP0100_ERROR_NONE;
	uint32_t CurrentTime;

	if(pIlp0100DebugBuffer==0) {
		/* Log buffer not created yet, create it first */
		Status = Ilp0100_loggingOpen();
	}

	Ilp0100DebugStarted=TRUE;

	if(Status == ILP0100_ERROR_NONE){
		Status = Ilp0100_GetTimeStamp(&CurrentTime);
	}
	if(Status == ILP0100_ERROR_NONE){
		ILP0100_DEBUG_WRITE_IN_LOG("<Ilp0100_LoggingSession TimeStamp=\"%.8d\">\n", CurrentTime);
		ILP0100_DEBUG_WRITE_IN_LOG("<Ilp0100_DebugStart TimeStamp=\"%.8d\"/>\n", CurrentTime);
	}

	/* TODO: Add ILP0100 Status dump */

	CurrentLevel = 0;

	return ILP0100_ERROR_NONE;
}

/*!
 * \fn			Ilp0100_loggingStop()
 * \brief		Stop logging all ILP activities
 * \ingroup		Debugging_Functions
 * \retval		ILP0100_ERROR_NONE : Success
 * \retval		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_loggingStop()
{
	ilp0100_error Status = ILP0100_ERROR_NONE;
	uint32_t CurrentTime;

	Status = Ilp0100_GetTimeStamp(&CurrentTime);
	if(Status == ILP0100_ERROR_NONE){
		ILP0100_DEBUG_WRITE_IN_LOG("<Ilp0100_DebugStop TimeStamp=\"%.8d\"/>\n", CurrentTime);
		ILP0100_DEBUG_WRITE_IN_LOG("</Ilp0100_LoggingSession>\n");
	}

	Ilp0100DebugStarted=FALSE;

	return ILP0100_ERROR_NONE;
}

/*!
 * \fn			Ilp0100_logDebugMessageStar(const char* pFunctionName)
 * \brief		Write start section to log debug message
 * \ingroup		Debugging_Functions
 * \param[in]	pFunctionName
 * \retval		ILP0100_ERROR_NONE : Success
 * \retval		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_logDebugMessageStart(const char* pFunctionName)
{
	ilp0100_error Status = ILP0100_ERROR_NONE;
	uint32_t CurrentTime;

	Status = Ilp0100_GetTimeStamp(&CurrentTime);
	if(Status==ILP0100_ERROR_NONE){
		if(pIlp0100DebugBuffer!=0) {
			ILP0100_DEBUG_WRITE_IN_LOG("<Log_message TimeStamp=\"%.8d\" API_Function_Name=\"%s\">",CurrentTime, pFunctionName);
		}
	}

	return Status;
}

/*!
 * \fn			Ilp0100_logDebugMessageEnd(
 * \brief		Write end section to log debug message
 * \ingroup		Debugging_Functions
 * \retval		ILP0100_ERROR_NONE : Success
 * \retval		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_logDebugMessageEnd()
{
	ilp0100_error Status = ILP0100_ERROR_NONE;
	if(pIlp0100DebugBuffer!=0) {
		ILP0100_DEBUG_WRITE_IN_LOG("</Log_message>\n");
	}
	return Status;
}

/*!
 * \fn			Ilp0100_logErrorMessageStart(const char* pFunctionName)
 * \brief		Write start section to log error message
 * \ingroup		Debugging_Functions
 * \param[in]	pFunctionName
 * \retval		ILP0100_ERROR_NONE : Success
 * \retval		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_logErrorMessageStart(const char* pFunctionName)
{
	ilp0100_error Status = ILP0100_ERROR_NONE;
	uint32_t CurrentTime;

	Status = Ilp0100_GetTimeStamp(&CurrentTime);
	if(Status==ILP0100_ERROR_NONE){
		if(pIlp0100DebugBuffer!=0) {
			ILP0100_DEBUG_WRITE_IN_LOG("<ERROR_MESSAGE TimeStamp=\"%.8d\" API_Function_Name=\"%s\">",CurrentTime, pFunctionName);
		}
	}

	return Status;
}

/*!
 * \fn			Ilp0100_logErrorMessageEnd()
 * \brief		Write end section to log error message
 * \ingroup		Debugging_Functions
 * \param[in]	pMessage
 * \retval		ILP0100_ERROR_NONE : Success
 * \retval		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_logErrorMessageEnd()
{
	ilp0100_error Status = ILP0100_ERROR_NONE;

	if(pIlp0100DebugBuffer!=0) {
		ILP0100_DEBUG_WRITE_IN_LOG("</ERROR_MESSAGE>\n");
	}

	return Status;
}

/*!
 * \fn			Ilp0100_loggingGetSize(uint32_t* pLogSize)
 * \brief		return current size of the log sequence
 * \ingroup		Debugging_Functions
 * \param[in]	pLogSize
 * \retval		ILP0100_ERROR_NONE : Success
 * \retval		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_loggingGetSize(uint32_t* pLogSize)
{
	/* Return current log size plus size of file footer*/
	*pLogSize = Ilp0100DebugLogSize+20;
	return ILP0100_ERROR_NONE;
}

/*!
 * \fn			Ilp0100_loggingReadBack(uint8_t* pDebugLog, uint32_t* pLogSize)
 * \brief		return a copy of the buffer
 * \ingroup		Debugging_Functions
 * \param[in]	pDebugLog
 * \param[in]	pLogSize
 * \retval		ILP0100_ERROR_NONE : Success
 * \retval		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_loggingReadBack(uint8_t* pDebugLog, uint32_t* pLogSize)
{
	ilp0100_error Status = ILP0100_ERROR_NONE;
	uint32_t CurrentLogSize;
	uint32_t i;

	if(pDebugLog!=0)
	{
		if(pIlp0100DebugBuffer!=0)
		{
			/* Write end of xml file */
			if(Status == ILP0100_ERROR_NONE){
				CurrentLogSize=Ilp0100DebugLogSize;
				ILP0100_DEBUG_WRITE_IN_LOG("</Ilp0100_Logging>\n");
			 }


			/* Copy of the buffer */
			for(i=0; i<Ilp0100DebugLogSize; i++)
			{
				*(pDebugLog+i) = *(pIlp0100DebugBuffer+i);
			}

			/* Copy of buffer size */
			*pLogSize = Ilp0100DebugLogSize;

			/* Change log size to go back before footer */
			Ilp0100DebugLogSize = CurrentLogSize;
		}
		else
		{
			Status = ILP_0100_DEBUG_SESSION_NOT_OPENED;
		}
	}
	else
	{
		Status = ILP_0100_DEBUG_NOT_VALID_BUFFER_ERROR;
	}
	return Status;
}

/*!
 * \fn			Ilp0100_loggingFunctionStart(char* FunctionName, void *pFuncArguments)
 * \brief		log start of an API function
 * \ingroup		Debugging_Functions
 * \param[in]	FunctionName
 * \param[in]	pFuncArguments
 * \retval		ILP0100_ERROR_NONE : Success
 * \retval		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_loggingFunctionStart(const char* pFunctionName, void **pFuncArguments)
{
	ilp0100_error Status = ILP0100_ERROR_NONE;
	uint32_t CurrentTime;
	bool_t isCoreFunc;

	if(Ilp0100DebugStarted){
		Status = Ilp0100_GetTimeStamp(&CurrentTime);
		if(Status==ILP0100_ERROR_NONE){
			if(pIlp0100DebugBuffer!=0) {
				ILP0100_DEBUG_WRITE_IN_LOG("<API_Function>\n");
				ILP0100_DEBUG_WRITE_IN_LOG("<Exec_Start TimeStamp=\"%.8d\">\n", CurrentTime);
				ILP0100_DEBUG_WRITE_IN_LOG("<API_Function_Name>%s</API_Function_Name>\n", pFunctionName);

				/* Check if we are starting to log an API core function */
				Ilp0100_isApiCoreFunction(pFunctionName, &isCoreFunc);
				if(isCoreFunc && CurrentLevel==0)
				{
					ILP0100_DEBUG_WRITE_IN_LOG("<Warning>\nSeems that a core function is called by User application\n</Warning>\n");
				}
				if((!isCoreFunc) && (CurrentLevel!=0))
				{
					ILP0100_DEBUG_WRITE_IN_LOG("<Warning>\nSeems that an API user interface function is called in parrallel to antoher one.\n(or API function calls another API user interface function)\n</Warning>\n");
				}
				ILP0100_DEBUG_WRITE_IN_LOG("<API_Input_Arguments>\n");
				Ilp0100_dumpParameters(pFunctionName, pFuncArguments);
				ILP0100_DEBUG_WRITE_IN_LOG("</API_Input_Arguments>\n");

				ILP0100_DEBUG_WRITE_IN_LOG("</Exec_Start>\n");

				ILP0100_SPRINTF(FunctionsLevel[CurrentLevel], "%s", pFunctionName);
				CurrentLevel = CurrentLevel+1;

			} else {
				Status = ILP_0100_DEBUG_SESSION_NOT_OPENED;
				}
		}
	}
	return Status;
}

/*!
 * \fn			Ilp0100_loggingFunctionEnd(char* FunctionName, void *pFuncArguments)
 * \brief		log end of an API function
 * \ingroup		Debugging_Functions
 * \param[in]	FunctionName
 * \param[in]	ReturnedValue
 * \param[in]	pFuncArguments
 * \retval		ILP0100_ERROR_NONE : Success
 * \retval		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_loggingFunctionEnd(const char* pFunctionName, ilp0100_error ReturnedValue, void **pFuncArguments)
{
	ilp0100_error Status = ILP0100_ERROR_NONE;
	uint32_t 	  CurrentTime;
	uint8_t		  FuncOK;

	if(Ilp0100DebugStarted){
		Status = Ilp0100_GetTimeStamp(&CurrentTime);

		if(Status==ILP0100_ERROR_NONE){
			if(pIlp0100DebugBuffer!=0) {
				ILP0100_DEBUG_WRITE_IN_LOG("<Exec_End TimeStamp=\"%.8d\"/>\n", CurrentTime);
				ILP0100_DEBUG_WRITE_IN_LOG("<API_Function_Name>%s</API_Function_Name>\n", pFunctionName);
				ILP0100_DEBUG_WRITE_IN_LOG("<API_Output_Arguments>\n");
				Ilp0100_dumpParameters(pFunctionName, pFuncArguments);
				ILP0100_DEBUG_WRITE_IN_LOG("</API_Output_Arguments>\n");
				ILP0100_DEBUG_WRITE_IN_LOG("<Returned_Value>%x</Returned_Value>\n", ReturnedValue);
				if(CurrentLevel>0)
					CurrentLevel = CurrentLevel-1;
				Ilp0100_core_strCmpr(pFunctionName, FunctionsLevel[CurrentLevel], &FuncOK);
				if(!FuncOK){
					ILP0100_DEBUG_WRITE_IN_LOG("<WARNING>\nEND EXEC FUNCTION NAME DO NOT CORRESPOND TO START FUNCTION NAME.\nEXPECTED END OF: %s\nGOT END OF: %s\nPLEASE CHECK!\n</WARNING>\n",FunctionsLevel[CurrentLevel],pFunctionName);
				}
				ILP0100_DEBUG_WRITE_IN_LOG("</API_Function>\n");
			} else {
				Status = ILP_0100_DEBUG_SESSION_NOT_OPENED;
			}
		}
	}
	return Status;
}

/*!
 * \fn			Ilp0100_loggingFunctionIlpAccess(const char* pFunctionName, uint16_t RegisterName, uint16_t Count, uint8_t *pData)
 * \brief		log an SPI access (Read or Write) of the ILP
 * \ingroup		Debugging_Functions
 * \param[in]	FunctionName
 * \param[in]	RegisterName
 * \param[in]	Count
 * \param[in]	pData
 * \param[in]	ReturnedValue
 * \retval		ILP0100_ERROR_NONE : Success
 * \retval		"Other Error Code" : Failure
 */
ilp0100_error Ilp0100_loggingFunctionIlpAccess(const char* pFunctionName, uint16_t RegisterName, uint16_t Count, uint8_t *pData, ilp0100_error ReturnedValue)
{
	ilp0100_error Status = ILP0100_ERROR_NONE;
	uint32_t CurrentTime;

	void *pFuncArguments[]={(void*)&RegisterName, (void*)&Count, (void*)pData};

	if(Ilp0100DebugStarted){
		Status = Ilp0100_GetTimeStamp(&CurrentTime);
		if(Status==ILP0100_ERROR_NONE){
			if(pIlp0100DebugBuffer!=0) {
				ILP0100_DEBUG_WRITE_IN_LOG("<ILP_Access TimeStamp=\"%.8ud\" API_FuncName=\"%s\">\n", CurrentTime, pFunctionName);
				Ilp0100_dumpParameters(pFunctionName, pFuncArguments);
				ILP0100_DEBUG_WRITE_IN_LOG("<Returned_Value>%x</Returned_Value>\n", ReturnedValue);
				ILP0100_DEBUG_WRITE_IN_LOG("</ILP_Access>\n");
			} else {
				Status = ILP_0100_DEBUG_SESSION_NOT_OPENED;
			}
		}
	}
	return Status;
}


ilp0100_error Ilp0100_isApiCoreFunction(const char* pFunctionName, bool_t* isCoreFunc)
{
	ilp0100_error Status = ILP0100_ERROR_NONE;

	char corefunctionRoot[]="Ilp0100_core";
	char functionRoot[13];
	uint8_t	i;

	for(i=0; ((i<12)&&(pFunctionName[i]!=0)); i=i+1)
	{
		functionRoot[i]=pFunctionName[i];
	}
	functionRoot[i]=0;

	/* Test Equality */
	Status = Ilp0100_core_strCmpr(functionRoot, corefunctionRoot, isCoreFunc);

	return Status;
}


ilp0100_error Ilp0100_dumpParameters(const char* pFunctionName, void **pFuncArguments)
{
	ilp0100_error Status = ILP0100_ERROR_NONE;
	uint8_t	FuncStrEqual;

	/* API FUNCTINONS */
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_init", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structInit, 			*(pFuncArguments+0));
		DUMP_PARAM_STRUCT(Ilp0100_structInitFirmware, 	*(pFuncArguments+1));
		DUMP_PARAM_STRUCT(Ilp0100_structSensorParams, 	*(pFuncArguments+2));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_defineMode", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structFrameFormat,	*(pFuncArguments+0));
	}

	/* IQ functions */
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_configDefcorShortOrNormal", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structDefcorConfig,	*(pFuncArguments+0));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_configDefcorLong", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structDefcorConfig,	*(pFuncArguments+0));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_updateDefcorShortOrNormal", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structDefcorParams,	*(pFuncArguments+0));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_updateDefcorLong", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structDefcorParams,	*(pFuncArguments+0));
	}

	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_configChannelOffsetShortOrNormal", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structChannelOffsetConfig,	*(pFuncArguments+0));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_configChannelOffsetLong", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structChannelOffsetConfig,	*(pFuncArguments+0));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_updateChannelOffsetShortOrNormal", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structChannelOffsetParams,	*(pFuncArguments+0));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_updateChannelOffsetLong", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structChannelOffsetParams,	*(pFuncArguments+0));
	}

	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_configHdrMerge", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structHdrMergeConfig,	*(pFuncArguments+0));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_updateHdrMerge", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structHdrMergeParams,	*(pFuncArguments+0));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_configCls", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structClsConfig,	*(pFuncArguments+0));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_updateCls", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structClsParams,	*(pFuncArguments+0));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_configToneMapping", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structToneMappingConfig,	*(pFuncArguments+0));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_updateToneMapping", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structToneMappingParams,	*(pFuncArguments+0));
	}
	/* End IQ functions */

	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_configGlaceShortOrNormal", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structGlaceConfig,	*(pFuncArguments+0));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_configGlaceLong", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structGlaceConfig,	*(pFuncArguments+0));
	}

	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_configHistShortOrNormal", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structHistConfig,	*(pFuncArguments+0));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_configHistLong", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structHistConfig,	*(pFuncArguments+0));
	}

	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_updateSensorParamsShortOrNormal", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structFrameParams,	*(pFuncArguments+0));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_updateSensorParamsLong", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structFrameParams,	*(pFuncArguments+0));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_readBackGlaceStatisticsShortOrNormal", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structGlaceStatsData,		*(pFuncArguments+0));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_readBackGlaceStatisticsLong", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structGlaceStatsData,		*(pFuncArguments+0));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_readBackHistStatisticsShortOrNormal", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structHistStatsData,		*(pFuncArguments+0));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_readBackHistStatisticsLong", FuncStrEqual){
		DUMP_PARAM_STRUCT(Ilp0100_structHistStatsData,		*(pFuncArguments+0));
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_setVirtualChannelShortOrNormal", FuncStrEqual){
		DUMP_PARAM(VirtualChannel,  *(pFuncArguments+0), ILP0100_DEBUG_UINT8);
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_setVirtualChannelLong", FuncStrEqual){
		DUMP_PARAM(VirtualChannel,  *(pFuncArguments+0), ILP0100_DEBUG_UINT8);
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_setHDRFactor", FuncStrEqual){
		DUMP_PARAM(HDRFactor,	    *(pFuncArguments+0), ILP0100_DEBUG_UINT8);
	}

	/* Interrupt functions */
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_interruptEnable", FuncStrEqual){
		DUMP_PARAM(InterruptSetMask,*(pFuncArguments+0), ILP0100_DEBUG_UINT32);
		DUMP_PARAM(Pin,				*(pFuncArguments+1), ILP0100_DEBUG_UINT8);
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_interruptDisable", FuncStrEqual){
		DUMP_PARAM(InterruptClrMask,*(pFuncArguments+0), ILP0100_DEBUG_UINT32);
		DUMP_PARAM(Pin,				*(pFuncArguments+1), ILP0100_DEBUG_UINT8);
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_interruptReadStatus", FuncStrEqual){
		DUMP_PARAM(pInterruptId,	*(pFuncArguments+0), ILP0100_DEBUG_UINT32);
		DUMP_PARAM(Pin,				*(pFuncArguments+1), ILP0100_DEBUG_UINT8);
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_interruptClearStatus", FuncStrEqual){
		DUMP_PARAM(pInterruptId,	*(pFuncArguments+0), ILP0100_DEBUG_UINT32);
		DUMP_PARAM(Pin,				*(pFuncArguments+1), ILP0100_DEBUG_UINT8);
	}
	/* End Interrupt functions */

	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_getApiVersionNumber", FuncStrEqual){
		DUMP_PARAM_PTR(pMajorNumber, *(pFuncArguments+0), 1, ILP0100_DEBUG_UINT8);
		DUMP_PARAM_PTR(pMinorNumber, *(pFuncArguments+1), 1, ILP0100_DEBUG_UINT8);
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_getFirmwareVersionumber", FuncStrEqual){
		DUMP_PARAM_PTR(pMajorNumber, *(pFuncArguments+0), 1, ILP0100_DEBUG_UINT8);
		DUMP_PARAM_PTR(pMinorNumber, *(pFuncArguments+1), 1, ILP0100_DEBUG_UINT8);
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_startTestMode", FuncStrEqual){
		DUMP_PARAM(IpsBypass,	*(pFuncArguments+0), ILP0100_DEBUG_UINT16);
		DUMP_PARAM(TestMode, 	*(pFuncArguments+1), ILP0100_DEBUG_UINT16);
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_readRegister", FuncStrEqual){
		DUMP_PARAM(RegisterName,	*(pFuncArguments+0),    ILP0100_DEBUG_UINT16);
		DUMP_PARAM_PTR(pData, 		*(pFuncArguments+1), 4, ILP0100_DEBUG_UINT8);
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_writeRegister", FuncStrEqual){
		DUMP_PARAM(RegisterName,	*(pFuncArguments+0),    ILP0100_DEBUG_UINT16);
		DUMP_PARAM_PTR(pData, 		*(pFuncArguments+1), 4, ILP0100_DEBUG_UINT8);
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_enableIlp0100SensorClock", FuncStrEqual){
		DUMP_PARAM(SensorInterface,	*(pFuncArguments+0), ILP0100_DEBUG_BOOL);
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_disableIlp0100SensorClock", FuncStrEqual){
		DUMP_PARAM(SensorInterface,	*(pFuncArguments+0), ILP0100_DEBUG_BOOL);
	}
	/* END API FUNCTINONS */

	/* API CORE FUNCTIONS */
	/* Not needed as now API user interface function are defined
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_core_interruptEnable", FuncStrEqual){
		DUMP_PARAM(InterruptSetMask,*(pFuncArguments+0), ILP0100_DEBUG_UINT32);
		DUMP_PARAM(Pin,				*(pFuncArguments+1), ILP0100_DEBUG_UINT8);
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_core_interruptDisable", FuncStrEqual){
		DUMP_PARAM(InterruptClrMask,*(pFuncArguments+0), ILP0100_DEBUG_UINT32);
		DUMP_PARAM(Pin,				*(pFuncArguments+1), ILP0100_DEBUG_UINT8);
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "lp0100_core_interruptReadStatus", FuncStrEqual){
		DUMP_PARAM(pInterruptId,	*(pFuncArguments+0), ILP0100_DEBUG_UINT32);
		DUMP_PARAM(Pin,				*(pFuncArguments+1), ILP0100_DEBUG_UINT8);
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_core_interruptClearStatus", FuncStrEqual){
		DUMP_PARAM(pInterruptId,	*(pFuncArguments+0), ILP0100_DEBUG_UINT32);
		DUMP_PARAM(Pin,				*(pFuncArguments+1), ILP0100_DEBUG_UINT8);
	}
	*/
	/* END API CORE FUNCTIONS */

	/* ILP ACCESS FUNCTIONS */
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_core_readRegister", FuncStrEqual){
		DUMP_PARAM(RegisterName,	*(pFuncArguments+0), ILP0100_DEBUG_UINT16);
		DUMP_PARAM(Count, 	  		*(pFuncArguments+1), ILP0100_DEBUG_UINT16);
		DUMP_PARAM_PTR(pData, 		*(pFuncArguments+2), *((uint16_t*)*(pFuncArguments+1)), ILP0100_DEBUG_UINT8);
	}
	ILP0100_DEBUG_TEST_STR_EQUALITY(pFunctionName, "Ilp0100_core_writeRegister", FuncStrEqual){
		DUMP_PARAM(RegisterName,	*(pFuncArguments+0), ILP0100_DEBUG_UINT16);
		DUMP_PARAM(Count, 	  		*(pFuncArguments+1), ILP0100_DEBUG_UINT16);
		DUMP_PARAM_PTR(pData, 		*(pFuncArguments+2), *((uint16_t*)*(pFuncArguments+1)), ILP0100_DEBUG_UINT8);
	}
	/* END ILP ACCESS FUNCTIONS */
	return Status;
}

ilp0100_error Ilp0100_dumpParameter(const char* pParamName, const uint16_t ParamLength, const uint8_t *pParamValues, const uint8_t ParamType)
{
	ilp0100_error Status = ILP0100_ERROR_NONE;
	uint16_t I;
	uint8_t	ParamStrEqual;

	ILP0100_DEBUG_WRITE_IN_LOG("<%s>",pParamName);

	if(ParamType==ILP0100_DEBUG_NONE){
		ILP0100_DEBUG_TEST_STR_EQUALITY(pParamName, "Ilp0100_structInit", ParamStrEqual){
			ILP0100_DEBUG_WRITE_IN_LOG("\n");
			DUMP_STRUCTPARAM(Ilp0100_structInit, pParamValues, NumberOfLanes,		ILP0100_DEBUG_BYTE);
			DUMP_STRUCTPARAM(Ilp0100_structInit, pParamValues, uwPixelFormat, 		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structInit, pParamValues, BitRate, 		 	ILP0100_DEBUG_UINT32);
			DUMP_STRUCTPARAM(Ilp0100_structInit, pParamValues, ExternalClock, 		ILP0100_DEBUG_UINT32);
			DUMP_STRUCTPARAM(Ilp0100_structInit, pParamValues, ClockUsed, 			ILP0100_DEBUG_BYTE);
			DUMP_STRUCTPARAM(Ilp0100_structInit, pParamValues, UsedSensorInterface, ILP0100_DEBUG_BYTE);
			DUMP_STRUCTPARAM(Ilp0100_structInit, pParamValues, IntrEnablePin1, 		ILP0100_DEBUG_UINT32);
			DUMP_STRUCTPARAM(Ilp0100_structInit, pParamValues, IntrEnablePin2, 		ILP0100_DEBUG_UINT32);
		}
		ILP0100_DEBUG_TEST_STR_EQUALITY(pParamName, "Ilp0100_structInitFirmware", ParamStrEqual){
			ILP0100_DEBUG_WRITE_IN_LOG("\n");
			DUMP_STRUCTPARAM_PTR(Ilp0100_structInitFirmware, pParamValues, pIlp0100Firmware, 	((Ilp0100_structInitFirmware*)pParamValues)->Ilp0100FirmwareSize,							ILP0100_DEBUG_BYTE);
			DUMP_STRUCTPARAM(	 Ilp0100_structInitFirmware, pParamValues, Ilp0100FirmwareSize,																								ILP0100_DEBUG_UINT32);
			DUMP_STRUCTPARAM_PTR(Ilp0100_structInitFirmware, pParamValues, pIlp0100SensorGenericCalibData, 	((Ilp0100_structInitFirmware*)pParamValues)->Ilp0100SensorGenericCalibDataSize,	ILP0100_DEBUG_BYTE);
			DUMP_STRUCTPARAM(	 Ilp0100_structInitFirmware, pParamValues, Ilp0100SensorGenericCalibDataSize,																				ILP0100_DEBUG_UINT32);
			DUMP_STRUCTPARAM_PTR(Ilp0100_structInitFirmware, pParamValues, pIlp0100SensorRawPart2PartCalibData,((Ilp0100_structInitFirmware*)pParamValues)->Ilp0100SensorRawPart2PartCalibDataSize,	ILP0100_DEBUG_BYTE);
			DUMP_STRUCTPARAM(	 Ilp0100_structInitFirmware, pParamValues, Ilp0100SensorRawPart2PartCalibDataSize,																			ILP0100_DEBUG_UINT32);
		}
		ILP0100_DEBUG_TEST_STR_EQUALITY(pParamName, "Ilp0100_structSensorParams", ParamStrEqual){
			ILP0100_DEBUG_WRITE_IN_LOG("\n");
			DUMP_STRUCTPARAM(Ilp0100_structSensorParams, pParamValues, FullActivePixels, 	ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structSensorParams, pParamValues, MinLineLength, 		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structSensorParams, pParamValues, FullActiveLines, 	ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structSensorParams, pParamValues, PixelOrder, 			ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM(Ilp0100_structSensorParams, pParamValues, StatusNbLines, 		ILP0100_DEBUG_UINT8);
		}
		ILP0100_DEBUG_TEST_STR_EQUALITY(pParamName, "Ilp0100_structFrameFormat", ParamStrEqual){
			ILP0100_DEBUG_WRITE_IN_LOG("\n");
			DUMP_STRUCTPARAM(Ilp0100_structFrameFormat, pParamValues, ActiveLineLengthPixels, 		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structFrameFormat, pParamValues, ActiveFrameLengthLines, 		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structFrameFormat, pParamValues, LineLengthPixels, 			ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structFrameFormat, pParamValues, FrameLengthLines, 			ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structFrameFormat, pParamValues, StatusLinesOutputted, 		ILP0100_DEBUG_BOOL);
			DUMP_STRUCTPARAM(Ilp0100_structFrameFormat, pParamValues, StatusLineLengthPixels, 		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structFrameFormat, pParamValues, StatusNbLines, 				ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structFrameFormat, pParamValues, MinInterframe, 				ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structFrameFormat, pParamValues, AutomaticFrameParamsUpdate, 	ILP0100_DEBUG_BOOL);
			DUMP_STRUCTPARAM(Ilp0100_structFrameFormat, pParamValues, HDRMode, 						ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM(Ilp0100_structFrameFormat, pParamValues, uwOutputPixelFormat, 			ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM(Ilp0100_structFrameFormat, pParamValues, ImageOrientation, 			ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM(Ilp0100_structFrameFormat, pParamValues, HScaling, 					ILP0100_DEBUG_UINT32);
			DUMP_STRUCTPARAM(Ilp0100_structFrameFormat, pParamValues, VScaling, 					ILP0100_DEBUG_UINT32);
			DUMP_STRUCTPARAM(Ilp0100_structFrameFormat, pParamValues, Binning, 						ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM(Ilp0100_structFrameFormat, pParamValues, Hoffset, 						ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structFrameFormat, pParamValues, Voffset, 						ILP0100_DEBUG_UINT16);
		}
		ILP0100_DEBUG_TEST_STR_EQUALITY(pParamName, "Ilp0100_structFrameParams", ParamStrEqual){
			ILP0100_DEBUG_WRITE_IN_LOG("\n");
			DUMP_STRUCTPARAM(Ilp0100_structFrameParams, pParamValues, ExposureTime, 			ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structFrameParams, pParamValues, AnalogGainCodeGreen,		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structFrameParams, pParamValues, AnalogGainCodeRed, 		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structFrameParams, pParamValues, AnalogGainCodeBlue, 		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structFrameParams, pParamValues, DigitalGainCodeGreen,		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structFrameParams, pParamValues, DigitalGainCodeRed, 		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structFrameParams, pParamValues, DigitalGainCodeBlue, 		ILP0100_DEBUG_UINT16);
		}

		/* IQ Structures */
		/* DefCor */
		ILP0100_DEBUG_TEST_STR_EQUALITY(pParamName, "Ilp0100_structDefcorConfig", ParamStrEqual){
			ILP0100_DEBUG_WRITE_IN_LOG("\n");
			DUMP_STRUCTPARAM(Ilp0100_structDefcorConfig, pParamValues, Mode, 			ILP0100_DEBUG_UINT8);
		}
		ILP0100_DEBUG_TEST_STR_EQUALITY(pParamName, "Ilp0100_structDefcorParams", ParamStrEqual){
			ILP0100_DEBUG_WRITE_IN_LOG("\n");
			DUMP_STRUCTPARAM(Ilp0100_structDefcorParams, pParamValues, SingletThreshold, 	ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM(Ilp0100_structDefcorParams, pParamValues, CoupletThreshold, 	ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM(Ilp0100_structDefcorParams, pParamValues, BlackStrength, 		ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM(Ilp0100_structDefcorParams, pParamValues, WhiteStrength, 		ILP0100_DEBUG_UINT8);
		}
		/* Channel Offset */
		ILP0100_DEBUG_TEST_STR_EQUALITY(pParamName, "Ilp0100_structChannelOffsetConfig", ParamStrEqual){
			ILP0100_DEBUG_WRITE_IN_LOG("\n");
			DUMP_STRUCTPARAM(Ilp0100_structChannelOffsetConfig, pParamValues, Enable, 			ILP0100_DEBUG_BOOL);
		}
		ILP0100_DEBUG_TEST_STR_EQUALITY(pParamName, "Ilp0100_structChannelOffsetParams", ParamStrEqual){
			ILP0100_DEBUG_WRITE_IN_LOG("\n");
			DUMP_STRUCTPARAM(Ilp0100_structChannelOffsetParams, pParamValues, SensorPedestalGreenRed, 	ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structChannelOffsetParams, pParamValues, SensorPedestalRed, 		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structChannelOffsetParams, pParamValues, SensorPedestalBlue, 		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structChannelOffsetParams, pParamValues, SensorPedestalGreenBlue, 	ILP0100_DEBUG_UINT16);
		}
		/* HDR Merge */
		ILP0100_DEBUG_TEST_STR_EQUALITY(pParamName, "Ilp0100_structHdrMergeConfig", ParamStrEqual){
			ILP0100_DEBUG_WRITE_IN_LOG("\n");
			DUMP_STRUCTPARAM(Ilp0100_structHdrMergeConfig, pParamValues, Mode, 			ILP0100_DEBUG_UINT8);
		}
		ILP0100_DEBUG_TEST_STR_EQUALITY(pParamName, "Ilp0100_structHdrMergeParams", ParamStrEqual){
			ILP0100_DEBUG_WRITE_IN_LOG("\n");
			DUMP_STRUCTPARAM(Ilp0100_structHdrMergeParams, pParamValues, Method, 			ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM(Ilp0100_structHdrMergeParams, pParamValues, ImageCodes, 		ILP0100_DEBUG_UINT8);
		}
		/* Color Lens Shading */
		ILP0100_DEBUG_TEST_STR_EQUALITY(pParamName, "Ilp0100_structClsConfig", ParamStrEqual){
			ILP0100_DEBUG_WRITE_IN_LOG("\n");
			DUMP_STRUCTPARAM(Ilp0100_structClsConfig, pParamValues, Enable, 			ILP0100_DEBUG_BOOL);
		}
		ILP0100_DEBUG_TEST_STR_EQUALITY(pParamName, "Ilp0100_structClsParams", ParamStrEqual){
			ILP0100_DEBUG_WRITE_IN_LOG("\n");
			DUMP_STRUCTPARAM(Ilp0100_structClsParams, pParamValues, BowlerCornerGain, 	ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM(Ilp0100_structClsParams, pParamValues, ColorTempKelvin, 	ILP0100_DEBUG_UINT16);
		}
		/* Tone Mapping */
		ILP0100_DEBUG_TEST_STR_EQUALITY(pParamName, "Ilp0100_structToneMappingConfig", ParamStrEqual){
			ILP0100_DEBUG_WRITE_IN_LOG("\n");
			DUMP_STRUCTPARAM(Ilp0100_structToneMappingConfig, pParamValues, Enable, 				ILP0100_DEBUG_BOOL);
			DUMP_STRUCTPARAM(Ilp0100_structToneMappingConfig, pParamValues, UserDefinedCurveEnable,	ILP0100_DEBUG_BOOL);
		}
		ILP0100_DEBUG_TEST_STR_EQUALITY(pParamName, "Ilp0100_structToneMappingParams", ParamStrEqual){
			ILP0100_DEBUG_WRITE_IN_LOG("\n");
			DUMP_STRUCTPARAM(		Ilp0100_structToneMappingParams, pParamValues, Strength, 					ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM_ARRAY(	Ilp0100_structToneMappingParams, pParamValues, UserDefinedCurve,	256,	ILP0100_DEBUG_UINT16);
		}
		/* End IQ Structures */

		ILP0100_DEBUG_TEST_STR_EQUALITY(pParamName, "Ilp0100_structGlaceConfig", ParamStrEqual){
			ILP0100_DEBUG_WRITE_IN_LOG("\n");
			DUMP_STRUCTPARAM(Ilp0100_structGlaceConfig, pParamValues, Enable, 				ILP0100_DEBUG_BOOL);
			DUMP_STRUCTPARAM(Ilp0100_structGlaceConfig, pParamValues, RoiHStart, 			ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structGlaceConfig, pParamValues, RoiVStart, 			ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structGlaceConfig, pParamValues, RoiHBlockSize, 		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structGlaceConfig, pParamValues, RoiVBlockSize, 		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structGlaceConfig, pParamValues, RoiHNumberOfBlocks,	ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM(Ilp0100_structGlaceConfig, pParamValues, RoiVNumberOfBlocks,	ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM(Ilp0100_structGlaceConfig, pParamValues, SaturationLevelRed,	ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM(Ilp0100_structGlaceConfig, pParamValues, SaturationLevelGreen,	ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM(Ilp0100_structGlaceConfig, pParamValues, SaturationLevelBlue,	ILP0100_DEBUG_UINT8);
		}

		ILP0100_DEBUG_TEST_STR_EQUALITY(pParamName, "Ilp0100_structHistConfig", ParamStrEqual){
			ILP0100_DEBUG_WRITE_IN_LOG("\n");
			DUMP_STRUCTPARAM(Ilp0100_structHistConfig, pParamValues, Mode, 						ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structHistConfig, pParamValues, RoiXOffset, 				ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structHistConfig, pParamValues, RoiYOffset, 				ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structHistConfig, pParamValues, RoiXSize, 					ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structHistConfig, pParamValues, RoiYSize, 					ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(Ilp0100_structHistConfig, pParamValues, YConversionFactorGreenRed,	ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM(Ilp0100_structHistConfig, pParamValues, YConversionFactorRed,		ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM(Ilp0100_structHistConfig, pParamValues, YConversionFactorBlue,		ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM(Ilp0100_structHistConfig, pParamValues, YConversionFactorGreenBlue,ILP0100_DEBUG_UINT8);
		}

		ILP0100_DEBUG_TEST_STR_EQUALITY(pParamName, "Ilp0100_structGlaceStatsData", ParamStrEqual){
			ILP0100_DEBUG_WRITE_IN_LOG("\n");
			DUMP_STRUCTPARAM_ARRAY(Ilp0100_structGlaceStatsData, pParamValues, GlaceStatsRedMean, 				48,	ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM_ARRAY(Ilp0100_structGlaceStatsData, pParamValues, GlaceStatsGreenMean,				48,	ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM_ARRAY(Ilp0100_structGlaceStatsData, pParamValues, GlaceStatsBlueMean,				48,	ILP0100_DEBUG_UINT8);
			DUMP_STRUCTPARAM_ARRAY(Ilp0100_structGlaceStatsData, pParamValues, GlaceStatsNbOfSaturatedPixels,	48,	ILP0100_DEBUG_UINT16);
		}

		ILP0100_DEBUG_TEST_STR_EQUALITY(pParamName, "Ilp0100_structHistStatsData", ParamStrEqual){
			ILP0100_DEBUG_WRITE_IN_LOG("\n");
			DUMP_STRUCTPARAM_ARRAY(Ilp0100_structHistStatsData, pParamValues, HistStatsRedBin,				64,	ILP0100_DEBUG_UINT32);
			DUMP_STRUCTPARAM_ARRAY(Ilp0100_structHistStatsData, pParamValues, HistStatsGreenBin,			64,	ILP0100_DEBUG_UINT32);
			DUMP_STRUCTPARAM_ARRAY(Ilp0100_structHistStatsData, pParamValues, HistStatsBlueBin,				64,	ILP0100_DEBUG_UINT32);
			DUMP_STRUCTPARAM(	 Ilp0100_structHistStatsData, pParamValues, HistStatsRedDarkestBin, 		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(	 Ilp0100_structHistStatsData, pParamValues, HistStatsRedDarkestCount, 		ILP0100_DEBUG_UINT32);
			DUMP_STRUCTPARAM(	 Ilp0100_structHistStatsData, pParamValues, HistStatsRedBrightestBin,		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(	 Ilp0100_structHistStatsData, pParamValues, HistStatsRedBrightestCount,		ILP0100_DEBUG_UINT32);
			DUMP_STRUCTPARAM(	 Ilp0100_structHistStatsData, pParamValues, HistStatsRedHighestBin,			ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(	 Ilp0100_structHistStatsData, pParamValues, HistStatsRedHighestCount, 		ILP0100_DEBUG_UINT32);
			DUMP_STRUCTPARAM(	 Ilp0100_structHistStatsData, pParamValues, HistStatsGreenDarkestBin,		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(	 Ilp0100_structHistStatsData, pParamValues, HistStatsGreenDarkestCount, 	ILP0100_DEBUG_UINT32);
			DUMP_STRUCTPARAM(	 Ilp0100_structHistStatsData, pParamValues, HistStatsGreenBrightestBin,		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(	 Ilp0100_structHistStatsData, pParamValues, HistStatsGreenBrightestCount, 	ILP0100_DEBUG_UINT32);
			DUMP_STRUCTPARAM(	 Ilp0100_structHistStatsData, pParamValues, HistStatsGreenHighestBin,		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(	 Ilp0100_structHistStatsData, pParamValues, HistStatsGreenHighestCount,		ILP0100_DEBUG_UINT32);
			DUMP_STRUCTPARAM(	 Ilp0100_structHistStatsData, pParamValues, HistStatsBlueDarkestBin,		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(	 Ilp0100_structHistStatsData, pParamValues, HistStatsBlueDarkestCount, 		ILP0100_DEBUG_UINT32);
			DUMP_STRUCTPARAM(	 Ilp0100_structHistStatsData, pParamValues, HistStatsBlueBrightestBin,		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(	 Ilp0100_structHistStatsData, pParamValues, HistStatsBlueBrightestCount,	ILP0100_DEBUG_UINT32);
			DUMP_STRUCTPARAM(	 Ilp0100_structHistStatsData, pParamValues, HistStatsBlueHighestBin,		ILP0100_DEBUG_UINT16);
			DUMP_STRUCTPARAM(	 Ilp0100_structHistStatsData, pParamValues, HistStatsBlueHighestCount, 		ILP0100_DEBUG_UINT32);
		}
	} else  {
		for(I=0; I<ParamLength; I++){
			switch(ParamType)
			{
				case ILP0100_DEBUG_NONE:
					/* SHOULD NEVER ENTER HERE */
					break;
				case ILP0100_DEBUG_BYTE:
					ILP0100_DEBUG_WRITE_IN_LOG("0x%02x",(uint8_t)*(pParamValues+I));
					break;
				case ILP0100_DEBUG_UINT16:
					ILP0100_DEBUG_WRITE_IN_LOG("0x%04x",*(uint16_t*)(pParamValues+(I*2)));
					break;
				case ILP0100_DEBUG_UINT32:
					ILP0100_DEBUG_WRITE_IN_LOG("0x%08x",*(uint32_t*)(pParamValues+(I*4)));
					break;
				case ILP0100_DEBUG_FLOAT:
					ILP0100_DEBUG_WRITE_IN_LOG("0x%08x",*(uint32_t*)(pParamValues+(I*4)));
					break;
				default:
					break;
			}
			if(I<(ParamLength-1))
				ILP0100_DEBUG_WRITE_IN_LOG(", ");
		}

	}

	ILP0100_DEBUG_WRITE_IN_LOG("</%s>\n",pParamName);

	return Status;
}

#endif
