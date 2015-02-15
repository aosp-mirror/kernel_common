#ifndef _TYPEDEFS_
#define _TYPEDEFS_
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
#ifndef _LINUX_TYPES_H
typedef unsigned int		bool;
#endif
typedef unsigned char		byte;
typedef unsigned short		word;

#define FALSE                   0
#define TRUE                    1

#define LOW                     0
#define HIGH                    1

#define _ZERO					0x00
#define BIT_0                   0x01
#define BIT_1                   0x02
#define BIT_2                   0x04
#define BIT_3                   0x08
#define BIT_4                   0x10
#define BIT_5                   0x20
#define BIT_6                   0x40
#define BIT_7                   0x80
#endif

