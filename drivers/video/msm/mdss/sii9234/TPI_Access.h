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

uint8_t ReadByteTPI(uint8_t Offset);
void WriteByteTPI(uint8_t Offset, uint8_t Data);
void ReadModifyWriteTPI(uint8_t Offset, uint8_t Mask, uint8_t Data);

uint8_t ReadByteCBUS(uint8_t Offset);
void WriteByteCBUS(uint8_t Offset, uint8_t Data);
void ReadModifyWriteCBUS(uint8_t Offset, uint8_t Mask, uint8_t Value);

#define	TPI_SLAVE_ADDR	0x72
#define	HDMI_SLAVE_ADDR	0x92


#define INDEXED_PAGE_0		0x01
#define INDEXED_PAGE_1		0x02
#define INDEXED_PAGE_2		0x03

uint8_t ReadIndexedRegister(uint8_t PageNum, uint8_t Offset);
void WriteIndexedRegister(uint8_t PageNum, uint8_t Offset, uint8_t Data);
void ReadModifyWriteIndexedRegister(uint8_t PageNum, uint8_t Offset, uint8_t Mask, uint8_t Data);
