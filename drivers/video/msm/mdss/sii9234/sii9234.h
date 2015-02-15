/* include/linux/mhl.h
 *
 * Copyright (C) 2010 HTC, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _SII9234_H_
#define _SII9234_H_
//********************************************************************
//  Nested Include Files
//********************************************************************
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
//********************************************************************
//  Manifest Constants
//********************************************************************

//********************************************************************
//  Type Definitions
//********************************************************************

//********************************************************************
//  Define & Macro
//********************************************************************

//********************************************************************
//  Prototype & Extern variable, function
//********************************************************************
#define MAX_MHL_PCLK 75000

extern int sii9234_I2C_RxData(uint8_t deviceID, char *rxData, uint32_t length);
extern int sii9234_I2C_TxData(uint8_t deviceID, char *txData, uint32_t length);
extern int sii9234_get_intr_status(void);
extern void sii9234_reset(void);
extern int sii9234_get_ci2ca(void);

#endif/*_SII9234_H_*/

