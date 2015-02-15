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

#include "defs.h"
#include "TypeDefs.h"

void I2C_WriteByte(byte deviceID, byte offset, byte value)
{
	char buffer[2];

	buffer[0] = offset;
	buffer[1] = value;
	sii9234_I2C_TxData(deviceID, buffer, 2);
}

byte I2C_ReadByte(byte deviceID, byte offset)
{
	char buffer[2];

	buffer[0] = offset;
	sii9234_I2C_RxData(deviceID, buffer, 1);
    return buffer[0];
}

