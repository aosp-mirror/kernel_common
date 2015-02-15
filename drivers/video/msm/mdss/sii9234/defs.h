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
#include "sii9234.h"
#include <mach/debug_display.h>

#define T_MONITORING_PERIOD		10
#define SiI9234_PRODUCT_ID      0x9234
#define SiI_DEVICE_ID			0xB0

#define SiI_TARGET_STRING       "SiI9234 Starter Kit"

#define TX_HW_RESET_PERIOD		10


#define DISABLE 0x00
#define ENABLE  0xFF

#define CONF__TPI_DEBUG_PRINT   (ENABLE)


#if (CONF__TPI_DEBUG_PRINT == ENABLE)
    #define TPI_DEBUG_PRINT(x...) PR_DISP_DEBUG x
#else
    #define TPI_DEBUG_PRINT(x)
#endif

#define RCP_ENABLE 	1
#define MSC_TESTER	0
