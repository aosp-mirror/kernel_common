/*****************************************************************************
 Copyright(c) 2010 NMI Inc. All Rights Reserved
 
 File name : nmi625-i2c.h
 
 Description :  Generic I2C driver for NM625
 
 History : 
 ----------------------------------------------------------------------
 2010/05/17 	ssw		initial
*******************************************************************************/


#ifndef __NM625_I2C__
#define __NM625_I2C__

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#define NMI_I2C_ADDR 0x61			
#define I2C_MAX_SEND_LENGTH 300

#ifndef I2C_M_RD
#define I2C_M_RD 1
#endif

#ifndef I2C_M_WR
#define I2C_M_WR 0
#endif

extern int nmi625_i2c_init(void );
extern int nmi625_i2c_deinit(void);
extern void nmi625_i2c_read_chip_id(void);

int nmi625_i2c_read(void *hDevice, unsigned short addr, unsigned char *data, unsigned short length);
int nmi625_i2c_write(void *hDevice, unsigned short addr, unsigned char *data, unsigned short length);
	
#ifdef __cplusplus
}
#endif

#endif // __NM625_I2C__
