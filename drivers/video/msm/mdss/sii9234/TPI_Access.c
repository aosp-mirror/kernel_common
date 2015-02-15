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
#include "i2c_master_sw.h"
#include "TPI_Access.h"
#include "inc/si_datatypes.h"


byte ReadByteTPI(byte Offset)
{
	return I2C_ReadByte(TPI_SLAVE_ADDR, Offset);
}

void WriteByteTPI(byte Offset, byte Data)
{
	I2C_WriteByte(TPI_SLAVE_ADDR, Offset, Data);
}

void ReadModifyWriteTPI(byte Offset, byte Mask, byte Data)
{

	byte Temp;

	Temp = ReadByteTPI(Offset);
	Temp &= ~Mask;
	Temp |= (Data & Mask);
	WriteByteTPI(Offset, Temp);
}

byte ReadByteCBUS(byte Offset)
{
	return I2C_ReadByte(CBUS_SLAVE_ADDR, Offset);
}

void WriteByteCBUS(byte Offset, byte Data)
{
	I2C_WriteByte(CBUS_SLAVE_ADDR, Offset, Data);
}

void ReadModifyWriteCBUS(byte Offset, byte Mask, byte Value)
{

	byte Temp;

	Temp = ReadByteCBUS(Offset);
	Temp &= ~Mask;
	Temp |= (Value & Mask);
	WriteByteCBUS(Offset, Temp);
}

byte ReadIndexedRegister(byte PageNum, byte Offset)
{
	WriteByteTPI(0xBC, PageNum);
	WriteByteTPI(0xBD, Offset);
	return ReadByteTPI(0xBE);
}

void WriteIndexedRegister(byte PageNum, byte Offset, byte Data)
{
	WriteByteTPI(0xBC, PageNum);
	WriteByteTPI(0xBD, Offset);
	WriteByteTPI(0xBE, Data);
}

void ReadModifyWriteIndexedRegister(byte PageNum, byte Offset, byte Mask, byte Data)
{

	byte Temp;

	Temp = ReadIndexedRegister(PageNum, Offset);
	Temp &= ~Mask;
	Temp |= (Data & Mask);
	WriteByteTPI(0xBE, Temp);
}
