/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef MSM_RAWCHIP_SPI_H
#define MSM_RAWCHIP_SPI_H

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#define TRUE								1
#define SUCCESS								1
#define FALSE								0
#define FAILURE								0


int rawchip_spi_write(unsigned char addr, unsigned char data);
int rawchip_spi_write_2B1B(uint16_t addr, unsigned char data);
int rawchip_spi_read_burst(uint16_t addr, unsigned char *data, int count);
int rawchip_spi_read_2B4B(uint16_t addr, uint32_t *data);
int rawchip_spi_read_2B1B(uint16_t addr, unsigned char *data);
int rawchip_spi_read_2B2B(uint16_t addr, uint16_t *data);
int yushan_spi_write(uint16_t reg, uint8_t/*uint16_t*/ val);

int SPI_Read32( uint16_t uwIndex , uint16_t uwCount , uint32_t * pData);
int	SPI_Read(uint16_t uwIndex , uint16_t uwCount , uint8_t *pData);
int	SPI_Write(uint16_t uwIndex , uint16_t uwCount , uint8_t *pData);

int	SPI_Write_4thByte(uint16_t uwIndex ,
		uint16_t uwCount , uint8_t *pData);

int spi_rawchip_probe(struct spi_device *rawchip);

int rawchip_spi_init(void);

#endif
