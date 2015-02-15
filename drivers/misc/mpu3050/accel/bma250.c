/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
  $
 */

/**
 *  @defgroup   ACCELDL (Motion Library - Accelerometer Driver Layer)
 *  @brief      Provides the interface to setup and handle an accelerometers
 *              connected to the secondary I2C interface of the gyroscope.
 *
 *  @{
 *      @file   bma250.c
 *      @brief  Accelerometer setup and handling methods.
 */

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#ifdef __KERNEL__
#include <linux/module.h>
#endif
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include "mpu.h"
#include "mlos.h"
#include "mlsl.h"

#include <log.h>
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-acc"

/* full scale setting - register and mask */
#define BOSCH_CTRL_REG      (0x0F)
#define BOSCH_INT_REG       (0x16)
#define BOSCH_PWR_REG       (0x11)
#define BMA250_REG_SOFT_RESET (0x14)
#define BMA250_BW_REG        (0x10)    /* BMA250 : BW setting register */

#define ACCEL_BOSCH_CTRL_MASK              (0x0F)
#define ACCEL_BOSCH_CTRL_MASK_FSR          (0xF8)
#define ACCEL_BOSCH_INT_MASK_WUP           (0xF8)
#define ACCEL_BOSCH_INT_MASK_IRQ           (0xDF)
#define BMA250_BW_MASK      (0xE0)    /* BMA250 : BW setting mask */

#define D(x...) printk(KERN_DEBUG "[GSNR][BMA250] " x)
#define I(x...) printk(KERN_INFO "[GSNR][BMA250] " x)
#define E(x...) printk(KERN_ERR "[GSNR][BMA250 ERROR] " x)
#define DIF(x...) \
	if (debug_flag) \
		printk(KERN_DEBUG "[GSNR][BMA250 DEBUG] " x)

#ifdef CONFIG_CIR_ALWAYS_READY
#define BMA250_INT_CTRL_REG                     0x21
#define BMA250_INT_MODE_SEL__POS                0
#define BMA250_INT_MODE_SEL__LEN                4
#define BMA250_INT_MODE_SEL__MSK                0x0F
#define BMA250_INT_MODE_SEL__REG                BMA250_INT_CTRL_REG

#define BMA250_STATUS1_REG                      0x09

#define BMA250_LOWG_INT_S__POS          0
#define BMA250_LOWG_INT_S__LEN          1
#define BMA250_LOWG_INT_S__MSK          0x01
#define BMA250_LOWG_INT_S__REG          BMA250_STATUS1_REG

#define BMA250_HIGHG_INT_S__POS          1
#define BMA250_HIGHG_INT_S__LEN          1
#define BMA250_HIGHG_INT_S__MSK          0x02
#define BMA250_HIGHG_INT_S__REG          BMA250_STATUS1_REG

#define BMA250_SLOPE_INT_S__POS          2
#define BMA250_SLOPE_INT_S__LEN          1
#define BMA250_SLOPE_INT_S__MSK          0x04
#define BMA250_SLOPE_INT_S__REG          BMA250_STATUS1_REG

#define BMA250_DOUBLE_TAP_INT_S__POS     4
#define BMA250_DOUBLE_TAP_INT_S__LEN     1
#define BMA250_DOUBLE_TAP_INT_S__MSK     0x10
#define BMA250_DOUBLE_TAP_INT_S__REG     BMA250_STATUS1_REG

#define BMA250_SINGLE_TAP_INT_S__POS     5
#define BMA250_SINGLE_TAP_INT_S__LEN     1
#define BMA250_SINGLE_TAP_INT_S__MSK     0x20
#define BMA250_SINGLE_TAP_INT_S__REG     BMA250_STATUS1_REG

#define BMA250_ORIENT_INT_S__POS         6
#define BMA250_ORIENT_INT_S__LEN         1
#define BMA250_ORIENT_INT_S__MSK         0x40
#define BMA250_ORIENT_INT_S__REG         BMA250_STATUS1_REG

#define BMA250_FLAT_INT_S__POS           7
#define BMA250_FLAT_INT_S__LEN           1
#define BMA250_FLAT_INT_S__MSK           0x80
#define BMA250_FLAT_INT_S__REG           BMA250_STATUS1_REG

#define BMA250_DATA_INT_S__POS           7
#define BMA250_DATA_INT_S__LEN           1
#define BMA250_DATA_INT_S__MSK           0x80
#define BMA250_DATA_INT_S__REG           BMA250_STATUS2_REG


#define BMA250_INT1_PAD_SEL_REG                 0x19

#define BMA250_EN_INT1_PAD_LOWG__POS        0
#define BMA250_EN_INT1_PAD_LOWG__LEN        1
#define BMA250_EN_INT1_PAD_LOWG__MSK        0x01
#define BMA250_EN_INT1_PAD_LOWG__REG        BMA250_INT1_PAD_SEL_REG

#define BMA250_EN_INT1_PAD_HIGHG__POS       1
#define BMA250_EN_INT1_PAD_HIGHG__LEN       1
#define BMA250_EN_INT1_PAD_HIGHG__MSK       0x02
#define BMA250_EN_INT1_PAD_HIGHG__REG       BMA250_INT1_PAD_SEL_REG

#define BMA250_EN_INT1_PAD_SLOPE__POS       2
#define BMA250_EN_INT1_PAD_SLOPE__LEN       1
#define BMA250_EN_INT1_PAD_SLOPE__MSK       0x04
#define BMA250_EN_INT1_PAD_SLOPE__REG       BMA250_INT1_PAD_SEL_REG

#define BMA250_EN_INT1_PAD_DB_TAP__POS      4
#define BMA250_EN_INT1_PAD_DB_TAP__LEN      1
#define BMA250_EN_INT1_PAD_DB_TAP__MSK      0x10
#define BMA250_EN_INT1_PAD_DB_TAP__REG      BMA250_INT1_PAD_SEL_REG

#define BMA250_EN_INT1_PAD_SNG_TAP__POS     5
#define BMA250_EN_INT1_PAD_SNG_TAP__LEN     1
#define BMA250_EN_INT1_PAD_SNG_TAP__MSK     0x20
#define BMA250_EN_INT1_PAD_SNG_TAP__REG     BMA250_INT1_PAD_SEL_REG

#define BMA250_EN_INT1_PAD_ORIENT__POS      6
#define BMA250_EN_INT1_PAD_ORIENT__LEN      1
#define BMA250_EN_INT1_PAD_ORIENT__MSK      0x40
#define BMA250_EN_INT1_PAD_ORIENT__REG      BMA250_INT1_PAD_SEL_REG

#define BMA250_EN_INT1_PAD_FLAT__POS        7
#define BMA250_EN_INT1_PAD_FLAT__LEN        1
#define BMA250_EN_INT1_PAD_FLAT__MSK        0x80
#define BMA250_EN_INT1_PAD_FLAT__REG        BMA250_INT1_PAD_SEL_REG

#define BMA250_INT_ENABLE1_REG                  0x16
#define BMA250_INT_ENABLE2_REG                  0x17

#define BMA250_EN_SLOPE_X_INT__POS         0
#define BMA250_EN_SLOPE_X_INT__LEN         1
#define BMA250_EN_SLOPE_X_INT__MSK         0x01
#define BMA250_EN_SLOPE_X_INT__REG         BMA250_INT_ENABLE1_REG

#define BMA250_EN_SLOPE_Y_INT__POS         1
#define BMA250_EN_SLOPE_Y_INT__LEN         1
#define BMA250_EN_SLOPE_Y_INT__MSK         0x02
#define BMA250_EN_SLOPE_Y_INT__REG         BMA250_INT_ENABLE1_REG

#define BMA250_EN_SLOPE_Z_INT__POS         2
#define BMA250_EN_SLOPE_Z_INT__LEN         1
#define BMA250_EN_SLOPE_Z_INT__MSK         0x04
#define BMA250_EN_SLOPE_Z_INT__REG         BMA250_INT_ENABLE1_REG

#define BMA250_EN_SLOPE_XYZ_INT__POS         0
#define BMA250_EN_SLOPE_XYZ_INT__LEN         3
#define BMA250_EN_SLOPE_XYZ_INT__MSK         0x07
#define BMA250_EN_SLOPE_XYZ_INT__REG         BMA250_INT_ENABLE1_REG

#define BMA250_EN_DOUBLE_TAP_INT__POS      4
#define BMA250_EN_DOUBLE_TAP_INT__LEN      1
#define BMA250_EN_DOUBLE_TAP_INT__MSK      0x10
#define BMA250_EN_DOUBLE_TAP_INT__REG      BMA250_INT_ENABLE1_REG

#define BMA250_EN_SINGLE_TAP_INT__POS      5
#define BMA250_EN_SINGLE_TAP_INT__LEN      1
#define BMA250_EN_SINGLE_TAP_INT__MSK      0x20
#define BMA250_EN_SINGLE_TAP_INT__REG      BMA250_INT_ENABLE1_REG

#define BMA250_EN_ORIENT_INT__POS          6
#define BMA250_EN_ORIENT_INT__LEN          1
#define BMA250_EN_ORIENT_INT__MSK          0x40
#define BMA250_EN_ORIENT_INT__REG          BMA250_INT_ENABLE1_REG

#define BMA250_EN_FLAT_INT__POS            7
#define BMA250_EN_FLAT_INT__LEN            1
#define BMA250_EN_FLAT_INT__MSK            0x80
#define BMA250_EN_FLAT_INT__REG            BMA250_INT_ENABLE1_REG

#define BMA250_EN_HIGHG_X_INT__POS         0
#define BMA250_EN_HIGHG_X_INT__LEN         1
#define BMA250_EN_HIGHG_X_INT__MSK         0x01
#define BMA250_EN_HIGHG_X_INT__REG         BMA250_INT_ENABLE2_REG

#define BMA250_EN_HIGHG_Y_INT__POS         1
#define BMA250_EN_HIGHG_Y_INT__LEN         1
#define BMA250_EN_HIGHG_Y_INT__MSK         0x02
#define BMA250_EN_HIGHG_Y_INT__REG         BMA250_INT_ENABLE2_REG

#define BMA250_EN_HIGHG_Z_INT__POS         2
#define BMA250_EN_HIGHG_Z_INT__LEN         1
#define BMA250_EN_HIGHG_Z_INT__MSK         0x04
#define BMA250_EN_HIGHG_Z_INT__REG         BMA250_INT_ENABLE2_REG

#define BMA250_EN_HIGHG_XYZ_INT__POS         2
#define BMA250_EN_HIGHG_XYZ_INT__LEN         1
#define BMA250_EN_HIGHG_XYZ_INT__MSK         0x04
#define BMA250_EN_HIGHG_XYZ_INT__REG         BMA250_INT_ENABLE2_REG

#define BMA250_EN_LOWG_INT__POS            3
#define BMA250_EN_LOWG_INT__LEN            1
#define BMA250_EN_LOWG_INT__MSK            0x08
#define BMA250_EN_LOWG_INT__REG            BMA250_INT_ENABLE2_REG

#define BMA250_EN_NEW_DATA_INT__POS        4
#define BMA250_EN_NEW_DATA_INT__LEN        1
#define BMA250_EN_NEW_DATA_INT__MSK        0x10
#define BMA250_EN_NEW_DATA_INT__REG        BMA250_INT_ENABLE2_REG

#define BMA250_MODE_CTRL_REG                    0x11

#define BMA250_EN_LOW_POWER__POS          6
#define BMA250_EN_LOW_POWER__LEN          1
#define BMA250_EN_LOW_POWER__MSK          0x40
#define BMA250_EN_LOW_POWER__REG          BMA250_MODE_CTRL_REG

#define BMA250_EN_SUSPEND__POS            7
#define BMA250_EN_SUSPEND__LEN            1
#define BMA250_EN_SUSPEND__MSK            0x80
#define BMA250_EN_SUSPEND__REG            BMA250_MODE_CTRL_REG

#define BMA250_ORIENT_PARAM_REG                 0x2C

#define BMA250_ORIENT_MODE__POS                  0
#define BMA250_ORIENT_MODE__LEN                  2
#define BMA250_ORIENT_MODE__MSK                  0x03
#define BMA250_ORIENT_MODE__REG                  BMA250_ORIENT_PARAM_REG

#define BMA250_ORIENT_BLOCK__POS                 2
#define BMA250_ORIENT_BLOCK__LEN                 2
#define BMA250_ORIENT_BLOCK__MSK                 0x0C
#define BMA250_ORIENT_BLOCK__REG                 BMA250_ORIENT_PARAM_REG

#define BMA250_ORIENT_HYST__POS                  4
#define BMA250_ORIENT_HYST__LEN                  3
#define BMA250_ORIENT_HYST__MSK                  0x70
#define BMA250_ORIENT_HYST__REG                  BMA250_ORIENT_PARAM_REG

#define BMA250_SLOPE_DURN_REG                   0x27

#define BMA250_SLOPE_DUR__POS                    0
#define BMA250_SLOPE_DUR__LEN                    2
#define BMA250_SLOPE_DUR__MSK                    0x03
#define BMA250_SLOPE_DUR__REG                    BMA250_SLOPE_DURN_REG

#define BMA250_SLOPE_THRES_REG                  0x28

#define BMA250_SLOPE_THRES__POS                  0
#define BMA250_SLOPE_THRES__LEN                  8
#define BMA250_SLOPE_THRES__MSK                  0xFF
#define BMA250_SLOPE_THRES__REG                  BMA250_SLOPE_THRES_REG

#define BMA250_THETA_FLAT_REG                   0x2E

#define BMA250_THETA_FLAT__POS                  0
#define BMA250_THETA_FLAT__LEN                  6
#define BMA250_THETA_FLAT__MSK                  0x3F
#define BMA250_THETA_FLAT__REG                  BMA250_THETA_FLAT_REG

#define BMA250_FLAT_HOLD_TIME_REG               0x2F

#define BMA250_FLAT_HOLD_TIME__POS              4
#define BMA250_FLAT_HOLD_TIME__LEN              2
#define BMA250_FLAT_HOLD_TIME__MSK              0x30
#define BMA250_FLAT_HOLD_TIME__REG              BMA250_FLAT_HOLD_TIME_REG


#define BMA250_BW_SEL_REG                       0x10
#define BMA250_BANDWIDTH__POS             0
#define BMA250_BANDWIDTH__LEN             5
#define BMA250_BANDWIDTH__MSK             0x1F
#define BMA250_BANDWIDTH__REG             BMA250_BW_SEL_REG

#define BMA250_BW_7_81HZ        0x08
#define BMA250_BW_15_63HZ       0x09
#define BMA250_BW_31_25HZ       0x0A
#define BMA250_BW_62_50HZ       0x0B
#define BMA250_BW_125HZ         0x0C
#define BMA250_BW_250HZ         0x0D
#define BMA250_BW_500HZ         0x0E
#define BMA250_BW_1000HZ        0x0F
#define BMA250_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)


#define BMA250_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

#define PAD_LOWG                                        0
#define PAD_HIGHG                                       1
#define PAD_SLOP                                        2
#define PAD_DOUBLE_TAP                                  3
#define PAD_SINGLE_TAP                                  4
#define PAD_ORIENT                                      5
#define PAD_FLAT                                        6


#define BMA250_MODE_NORMAL      0
#define BMA250_MODE_LOWPOWER    1
#define BMA250_MODE_SUSPEND     2


static void *g_mlsl_handle;
static struct ext_slave_platform_data *g_pdata;
int cir_flag = 0;
static int power_key_pressed = 0;
static int (*gsensor_power_LPM)(int on) = NULL;


EXPORT_SYMBOL(cir_flag);
#endif
/* --------------------- */
/* -    Variables.     - */
/* --------------------- */

struct bma250_config {
	unsigned int odr; /* Output data rate mHz */
	unsigned int fsr; /* full scale range mg */
	unsigned int irq_type;
	unsigned int power_mode;
	unsigned char ctrl_reg; /* range */
	unsigned char bw_reg; /* bandwidth */
	unsigned char int_reg;
};

struct bma250_private_data {
	struct bma250_config suspend;
	struct bma250_config resume;
	unsigned char state;
};

#ifdef CONFIG_CIR_ALWAYS_READY
static int bma250_set_slope_threshold(void *mlsl_handle, struct ext_slave_platform_data *pdata,
		unsigned char threshold)
{
	int comres = 0;
	unsigned char data;

	comres = MLSLSerialRead(mlsl_handle, pdata->address,
		BMA250_SLOPE_THRES__REG, 1, &data);

	data = BMA250_SET_BITSLICE(data, BMA250_SLOPE_THRES, threshold);
	comres += MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				BMA250_SLOPE_THRES__REG, data);


	return comres;
}

static int bma250_set_slope_duration(void *mlsl_handle, struct ext_slave_platform_data *pdata, unsigned char
		duration)
{
	int comres = 0;
	unsigned char data;

	comres = MLSLSerialRead(mlsl_handle, pdata->address,
		BMA250_SLOPE_DUR__REG, 1, &data);

	data = BMA250_SET_BITSLICE(data, BMA250_SLOPE_DUR, duration);
	comres += MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				BMA250_SLOPE_DUR__REG, data);


	return comres;
}
static int bma250_set_mode(void *mlsl_handle, struct ext_slave_platform_data *pdata, unsigned char Mode)
{
	int comres = 0;
	unsigned char data1;


	if (Mode < 3) {
		comres = MLSLSerialRead(mlsl_handle, pdata->address,
				BMA250_EN_LOW_POWER__REG, 1, &data1);
		switch (Mode) {
		case BMA250_MODE_NORMAL:
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_LOW_POWER, 0);
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_SUSPEND, 0);
			break;
		case BMA250_MODE_LOWPOWER:
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_LOW_POWER, 1);
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_SUSPEND, 0);
			break;
		case BMA250_MODE_SUSPEND:
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_LOW_POWER, 0);
			data1  = BMA250_SET_BITSLICE(data1,
					BMA250_EN_SUSPEND, 1);
			break;
		default:
			break;
		}

		comres += MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				BMA250_EN_LOW_POWER__REG, data1);
	} else{
		comres = -1;
	}

	comres = MLSLSerialRead(mlsl_handle, pdata->address,
		BMA250_EN_LOW_POWER__REG, 1, &data1);

	printk(KERN_INFO "[BMA250] power mode = %x", data1);

	return comres;
}
static int bma250_set_int1_pad_sel(void *mlsl_handle, struct ext_slave_platform_data *pdata, unsigned char
	int1sel)
{
	int comres = 0;
	unsigned char data = 0;
	unsigned char state;
	state = 0x01;


	switch (int1sel) {
	case 0:
	        comres = MLSLSerialRead(mlsl_handle, pdata->address, BMA250_EN_INT1_PAD_LOWG__REG, 1,
				&data);
		printk(KERN_INFO "[BMA250] before set LOWG data = %x", data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT1_PAD_LOWG,
			state);
		comres = MLSLSerialWriteSingle(mlsl_handle, pdata->address, BMA250_EN_INT1_PAD_LOWG__REG, data);
	        comres = MLSLSerialRead(mlsl_handle, pdata->address, BMA250_EN_INT1_PAD_LOWG__REG, 1,
				&data);
		printk(KERN_INFO "[BMA250] after set LOWG data = %x", data);
		break;
	case 2:
		comres = MLSLSerialRead(mlsl_handle, pdata->address,
				BMA250_EN_INT1_PAD_SLOPE__REG, 1, &data);
		printk(KERN_INFO "[BMA250] before set Slop data = %x", data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT1_PAD_SLOPE,
				state);
		comres = MLSLSerialWriteSingle(mlsl_handle,pdata->address,
				BMA250_EN_INT1_PAD_SLOPE__REG, data);
		printk(KERN_INFO "[BMA250] after set Slop data = %x", data);
		break;
	case 4:
		comres = MLSLSerialRead(mlsl_handle, pdata->address,
				BMA250_EN_INT1_PAD_SNG_TAP__REG, 1, &data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT1_PAD_SNG_TAP,
				state);
		comres = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				BMA250_EN_INT1_PAD_SNG_TAP__REG, data);
		break;
	case 6:
		comres = MLSLSerialRead(mlsl_handle, pdata->address, BMA250_EN_INT1_PAD_FLAT__REG, 1,
			&data);
		printk(KERN_INFO "[BMA250] before set FLAT data = %x", data);
		data = BMA250_SET_BITSLICE(data, BMA250_EN_INT1_PAD_FLAT,
			state);
		comres = MLSLSerialWriteSingle(mlsl_handle, pdata->address, BMA250_EN_INT1_PAD_FLAT__REG, data);
		comres = MLSLSerialRead(mlsl_handle, pdata->address, BMA250_EN_INT1_PAD_FLAT__REG, 1,
			&data);
		printk(KERN_INFO "[BMA250] after set FLAT data = %x", data);
		break;
	default:
		break;
	}

	return comres;
}
static int bma250_set_Int_Mode(void *mlsl_handle, struct ext_slave_platform_data *pdata, unsigned char mode)
{
	int comres = 0;
	unsigned char data;


	comres = MLSLSerialRead(mlsl_handle, pdata->address, BMA250_INT_MODE_SEL__REG, 1,
		&data);
	printk(KERN_INFO "[BMA250] before set int mode = %x", data);
	data = BMA250_SET_BITSLICE(data, BMA250_INT_MODE_SEL,
		mode);
	comres = MLSLSerialWriteSingle(mlsl_handle, pdata->address, BMA250_INT_MODE_SEL__REG, data);

	comres = MLSLSerialRead(mlsl_handle, pdata->address, BMA250_INT_MODE_SEL__REG, 1,
		&data);
	printk(KERN_INFO "[BMA250] after set int mode = %x", data);
	return comres;
}
static int bma250_set_Int_Enable(void *mlsl_handle, struct ext_slave_platform_data *pdata, unsigned char
		InterruptType , unsigned char value)
{
	int comres = 0;
	unsigned char data1 = 0, data2 = 0;

	printk(KERN_INFO "[BMA250] before set int enable 1 = %x, int enable 2 = %x\n", data1, data2);

	comres = MLSLSerialRead(mlsl_handle, pdata->address, BMA250_INT_ENABLE1_REG, 1, &data1);
	comres = MLSLSerialRead(mlsl_handle, pdata->address, BMA250_INT_ENABLE2_REG, 1, &data2);

	value = value & 1;
	switch (InterruptType) {
	case 0:
		/* Low G Interrupt  */
		data2 = BMA250_SET_BITSLICE(data2, BMA250_EN_LOWG_INT, value);
		break;
	case 1:
		/* High G X Interrupt */

		data2 = BMA250_SET_BITSLICE(data2, BMA250_EN_HIGHG_X_INT,
				value);
		break;
	case 2:
		/* High G Y Interrupt */

		data2 = BMA250_SET_BITSLICE(data2, BMA250_EN_HIGHG_Y_INT,
				value);
		break;
	case 3:
		/* High G Z Interrupt */

		data2 = BMA250_SET_BITSLICE(data2, BMA250_EN_HIGHG_Z_INT,
				value);
		break;
	case 4:
		/* New Data Interrupt  */

		data2 = BMA250_SET_BITSLICE(data2, BMA250_EN_NEW_DATA_INT,
				value);
		break;
	case 5:
		/* Slope X Interrupt */

		data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_SLOPE_X_INT,
				value);
		break;
	case 6:
		/* Slope Y Interrupt */

		data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_SLOPE_Y_INT,
				value);
		break;
	case 7:
		/* Slope Z Interrupt */

		data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_SLOPE_Z_INT,
				value);
		break;
	case 8:
		/* Single Tap Interrupt */

		data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_SINGLE_TAP_INT,
				value);
		break;
	case 9:
		/* Double Tap Interrupt */

		data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_DOUBLE_TAP_INT,
				value);
		break;
	case 10:
		/* Orient Interrupt  */

		data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_ORIENT_INT, value);
		break;
	case 11:
		/* Flat Interrupt */

		data1 = BMA250_SET_BITSLICE(data1, BMA250_EN_FLAT_INT, value);
		break;
	default:
		break;
	}
	comres = MLSLSerialWriteSingle(mlsl_handle, pdata->address, BMA250_INT_ENABLE1_REG,
			data1);
	comres = MLSLSerialWriteSingle(mlsl_handle, pdata->address, BMA250_INT_ENABLE2_REG,
			data2);

	comres = MLSLSerialRead(mlsl_handle, pdata->address, BMA250_INT_ENABLE1_REG, 1, &data1);
	comres = MLSLSerialRead(mlsl_handle, pdata->address, BMA250_INT_ENABLE2_REG, 1, &data2);
	printk(KERN_INFO "[BMA250] after set int enable 1 = %x, int enable 2 = %x\n", data1, data2);
	return comres;
}
#endif
static int set_normal_mode(void *mlsl_handle,
			 struct ext_slave_platform_data *pdata)
{
	int result = 0;

	result = MLSLSerialWriteSingle(mlsl_handle,
		pdata->address,	BOSCH_PWR_REG, 0x00);
	ERROR_CHECK(result);

	usleep(2000);

	return 0;
}
/*********************************************
    Accelerometer Initialization Functions
**********************************************/

/**
 * Sets the IRQ to fire when one of the IRQ events occur.  Threshold and
 * duration will not be used uless the type is MOT or NMOT.
 *
 * @param config configuration to apply to, suspend or resume
 * @param irq_type The type of IRQ.  Valid values are
 * - MPU_SLAVE_IRQ_TYPE_NONE
 * - MPU_SLAVE_IRQ_TYPE_MOTION
 * - MPU_SLAVE_IRQ_TYPE_DATA_READY
 *
 */
static int bma250_set_irq(void *mlsl_handle,
			struct ext_slave_platform_data *pdata,
			struct bma250_config *config,
			int apply,
			long irq_type)
{
	unsigned char irq_bits = 0;
	int result = ML_SUCCESS;

	/* TODO Use irq when necessary */
	return ML_SUCCESS;

	if (irq_type == MPU_SLAVE_IRQ_TYPE_MOTION)
		return ML_ERROR_FEATURE_NOT_IMPLEMENTED;

	config->irq_type = (unsigned char)irq_type;

	if (irq_type == MPU_SLAVE_IRQ_TYPE_DATA_READY) {
		irq_bits = 0x20;
		config->int_reg &= ACCEL_BOSCH_INT_MASK_WUP;
	} else {
		irq_bits = 0x00;
		config->int_reg &= ACCEL_BOSCH_INT_MASK_WUP;
	}

	config->int_reg &= ACCEL_BOSCH_INT_MASK_IRQ;
	config->int_reg |= irq_bits;

	if (apply) {

#ifndef CONFIG_CIR_ALWAYS_READY
		if (!config->power_mode) {
			/* BMA250: Software reset */
			result = MLSLSerialWriteSingle(mlsl_handle,
					pdata->address, BMA250_REG_SOFT_RESET,
					0xB6);
			ERROR_CHECK(result);
			MLOSSleep(1);
		}

#endif

		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
			BOSCH_CTRL_REG, config->ctrl_reg);
		ERROR_CHECK(result);

		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
			BOSCH_INT_REG, config->int_reg);
		ERROR_CHECK(result);

#ifdef CONFIG_CIR_ALWAYS_READY
		if (!config->power_mode && !cir_flag) {
#else
		if (!config->power_mode) {
#endif
			result = MLSLSerialWriteSingle(mlsl_handle,
				pdata->address, BOSCH_PWR_REG, 0x80);
			ERROR_CHECK(result);
			MLOSSleep(1);
		} else {
			result = set_normal_mode(mlsl_handle, pdata);
			ERROR_CHECK(result);
		}
	}
	return result;
}

/**
 * Set the Output data rate for the particular configuration
 *
 * @param config Config to modify with new ODR
 * @param odr Output data rate in units of 1/1000Hz
 */
static int bma250_set_odr(void *mlsl_handle,
			struct ext_slave_platform_data *pdata,
			struct bma250_config *config,
			int apply,
			long odr)
{
	unsigned char odr_bits = 0;
	unsigned char wup_bits = 0;
	unsigned char read_from_chip_bw = 0;
	int result = ML_SUCCESS;

	/* TO DO use dynamic bandwidth when stability safe */
	/*if (odr > 100000) {
		config->odr = 125000;
		odr_bits = 0x0C;
		config->power_mode = 1;
	} else if (odr > 50000) {
		config->odr = 62500;
		odr_bits = 0x0B;
		config->power_mode = 1;
	} else if (odr > 20000) {
		config->odr = 31250;
		odr_bits = 0x0A;
		config->power_mode = 1;
	} else if (odr > 15000) {
		config->odr = 15630;
		odr_bits = 0x09;
		config->power_mode = 1;
	} else if (odr > 0) {
		config->odr = 7810;
		odr_bits = 0x08;
		config->power_mode = 1;
	} else {
		config->odr = 0;
		wup_bits = 0x00;
		config->power_mode = 0;
	}*/
	if (odr > 100000) {
		config->odr = 31250;
		odr_bits = 0x0A;
		config->power_mode = 1;
	} else if (odr > 50000) {
		config->odr = 31250;
		odr_bits = 0x0A;
		config->power_mode = 1;
	} else if (odr > 20000) {
		config->odr = 31250;
		odr_bits = 0x0A;
		config->power_mode = 1;
	} else if (odr > 15000) {
		config->odr = 31250;
		odr_bits = 0x0A;
		config->power_mode = 1;
	} else if (odr > 0) {
		config->odr = 31250;
		odr_bits = 0x0A;
		config->power_mode = 1;
	} else {
		config->odr = 0;
		wup_bits = 0x00;
		config->power_mode = 0;
	}

	switch (config->power_mode) {
	case 1:
		config->bw_reg &= BMA250_BW_MASK;
		config->bw_reg |= odr_bits;
		config->int_reg &= ACCEL_BOSCH_INT_MASK_WUP;
		break;
	case 0:
		config->int_reg &= ACCEL_BOSCH_INT_MASK_WUP;
		config->int_reg |= wup_bits;
		break;
	default:
		break;
	}

	MPL_LOGV("ODR: %d \n", config->odr);
	if (apply) {
#ifndef CONFIG_CIR_ALWAYS_READY
#if 0 /*remove for G-Sensor values error when set odr */
			/* BMA250: Software reset */
			result = MLSLSerialWriteSingle(mlsl_handle,
					pdata->address, BMA250_REG_SOFT_RESET,
					0xB6);
			ERROR_CHECK(result);
			MLOSSleep(1);
#endif
#endif
			result = MLSLSerialRead(mlsl_handle,
					pdata->address, BMA250_BW_REG, 1,
					&read_from_chip_bw);
			ERROR_CHECK(result);

			if (odr_bits != read_from_chip_bw) {
				D("%s: Really set ODR to %d\n", __func__,
					config->odr);
				result = MLSLSerialWriteSingle(mlsl_handle,
						pdata->address, BMA250_BW_REG,
						config->bw_reg);
				ERROR_CHECK(result);
				/* Make sure G-Sensor data is ready */
				MLOSSleep(25);
			}

			/* TODO Use irq when necessary */
			/*
			result = MLSLSerialWriteSingle(mlsl_handle,
					pdata->address,	BOSCH_INT_REG,
					config->int_reg);
			ERROR_CHECK(result);*/

			if (!config->power_mode) {
				result = MLSLSerialWriteSingle(mlsl_handle,
						pdata->address,	BOSCH_PWR_REG,
						0x80);
				ERROR_CHECK(result);
				MLOSSleep(1);
			} else {
#if 0 /* remove for G-Sensor values error when set odr */
				result = set_normal_mode(mlsl_handle, pdata);
				ERROR_CHECK(result);
#endif
			}
	}

	return result;
}

/**
 * Set the full scale range of the accels
 *
 * @param config pointer to configuration
 * @param fsr requested full scale range
 */
static int bma250_set_fsr(void *mlsl_handle,
			struct ext_slave_platform_data *pdata,
			struct bma250_config *config,
			int apply,
			long fsr)
{
	unsigned char fsr_bits;
	int result = ML_SUCCESS;

	/* TO DO use dynamic range when stability safe */
	/*if (fsr <= 2048) {
		fsr_bits = 0x03;
		config->fsr = 2048;
	} else if (fsr <= 4096) {
		fsr_bits = 0x05;
		config->fsr = 4096;
	} else if (fsr <= 8192) {
		fsr_bits = 0x08;
		config->fsr = 8192;
	} else if (fsr <= 16384) {
		fsr_bits = 0x0C;
		config->fsr = 16384;
	} else {
		fsr_bits = 0x03;
		config->fsr = 2048;
	}*/
	if (fsr <= 2048) {
		fsr_bits = 0x03;
		config->fsr = 2048;
	} else if (fsr <= 4096) {
		fsr_bits = 0x03;
		config->fsr = 2048;
	} else if (fsr <= 8192) {
		fsr_bits = 0x03;
		config->fsr = 2048;
	} else if (fsr <= 16384) {
		fsr_bits = 0x03;
		config->fsr = 2048;
	} else {
		fsr_bits = 0x03;
		config->fsr = 2048;
	}

	config->ctrl_reg &= ACCEL_BOSCH_CTRL_MASK_FSR;
	config->ctrl_reg |= fsr_bits;

	MPL_LOGV("FSR: %d \n", config->fsr);
	if (apply) {
#ifndef CONFIG_CIR_ALWAYS_READY
		if (!config->power_mode) {
			/* BMA250: Software reset */
			result = MLSLSerialWriteSingle(mlsl_handle,
				pdata->address, BMA250_REG_SOFT_RESET, 0xB6);
			ERROR_CHECK(result);
			MLOSSleep(1);
		}
#endif

		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
			BOSCH_CTRL_REG, config->ctrl_reg);
		ERROR_CHECK(result);

		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
			BOSCH_CTRL_REG, config->ctrl_reg);
		ERROR_CHECK(result);

		if (!config->power_mode) {
			result = MLSLSerialWriteSingle(mlsl_handle,
				pdata->address,	BOSCH_PWR_REG, 0x80);
			ERROR_CHECK(result);
			MLOSSleep(1);
		} else {
			result = set_normal_mode(mlsl_handle, pdata);
			ERROR_CHECK(result);
		}
	}
	return result;
}

static int bma250_suspend(void *mlsl_handle,
			  struct ext_slave_descr *slave,
			  struct ext_slave_platform_data *pdata)
{

	int result = 0;
	unsigned char ctrl_reg;
	unsigned char int_reg;

	struct bma250_private_data *private_data = pdata->private_data;
	ctrl_reg = private_data->suspend.ctrl_reg;
	int_reg = private_data->suspend.int_reg;

	private_data->state = 1;

	/* TO DO sync from bma150 of MPL3.3.0, comment follows */
	/* BMA250: Software reset */
	/*result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
		BMA250_REG_SOFT_RESET, 0xB6);
	ERROR_CHECK(result);
	MLOSSleep(1);

	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
		BOSCH_CTRL_REG, ctrl_reg);
	ERROR_CHECK(result);*/

	/* TODO Use irq when necessary */
	/*result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
		BOSCH_INT_REG, int_reg);
	ERROR_CHECK(result);*/

#ifdef CONFIG_CIR_ALWAYS_READY
	//Add CIR Flag for always ready feature
	if ((!private_data->suspend.power_mode && !cir_flag)) {
#else
	if ((!private_data->suspend.power_mode)) {
#endif

		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
			BOSCH_PWR_REG, 0x80);
		printk(KERN_INFO "[BMA250] GSensor Low Power Mode\n");
		ERROR_CHECK(result);
	} 
	return result;
}


static int bma250_resume(void *mlsl_handle,
			 struct ext_slave_descr *slave,
			 struct ext_slave_platform_data *pdata)
{

	int result;
	unsigned char ctrl_reg;
	unsigned char bw_reg;
	unsigned char int_reg;

	struct bma250_private_data *private_data = pdata->private_data;



	ctrl_reg = private_data->resume.ctrl_reg;
	bw_reg = private_data->resume.bw_reg;
	int_reg = private_data->resume.int_reg;

	private_data->state = 0;

#ifndef CONFIG_CIR_ALWAYS_READY
	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
		BMA250_REG_SOFT_RESET, 0xB6);  /* BMA250: Software reset */
	ERROR_CHECK(result);
	MLOSSleep(1);
#endif

	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
		BOSCH_CTRL_REG, ctrl_reg);
	ERROR_CHECK(result);

	result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
		BMA250_BW_REG, bw_reg);
	ERROR_CHECK(result);

	/* TODO Use irq when necessary */
	/*result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
		BOSCH_INT_REG, int_reg);
	ERROR_CHECK(result);*/

	if (!private_data->resume.power_mode) {
		result = MLSLSerialWriteSingle(mlsl_handle, pdata->address,
			BOSCH_PWR_REG, 0x80);
		ERROR_CHECK(result);
	} else {
		result = set_normal_mode(mlsl_handle, pdata);
		ERROR_CHECK(result);
	}

	return result;
}

static int bma250_read(void *mlsl_handle,
		       struct ext_slave_descr *slave,
		       struct ext_slave_platform_data *pdata,
		       unsigned char *data)
{
	int result;

	result = MLSLSerialRead(mlsl_handle, pdata->address,
		slave->reg, slave->len, data);

	return result;
}


#ifdef CONFIG_CIR_ALWAYS_READY


static ssize_t bma250_enable_interrupt(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long enable;
	int error;

	error = strict_strtoul(buf, 10, &enable);
		if (error)
		    return error;
	printk(KERN_INFO "[BMA250] bma250_enable_interrupt, power_key_pressed = %d\n", power_key_pressed);
	if(enable == 1 && !power_key_pressed){ // Slope interrupt
		

	    //Don't change to low power mode due to enabling interrupt mode
	    if(gsensor_power_LPM){
		I("Non Low Power Mode\n");
		gsensor_power_LPM(0);

	    }
	    /*Set the related parameters*/
	    error = bma250_set_Int_Mode(g_mlsl_handle, g_pdata, 1);/*latch interrupt 250ms*/

	    error += bma250_set_slope_duration(g_mlsl_handle, g_pdata, 0x01);//dur+1
	    error += bma250_set_slope_threshold(g_mlsl_handle, g_pdata, 0x07);//0x07 * 3.91  = 
	    /*Enable the interrupts*/
	    error += bma250_set_Int_Enable(g_mlsl_handle, g_pdata,5, 1);//Slope X
	    error += bma250_set_Int_Enable(g_mlsl_handle, g_pdata,6, 1);//Slope Y
	    error += bma250_set_Int_Enable(g_mlsl_handle, g_pdata,7, 0);//Slope Z
	    error += bma250_set_int1_pad_sel(g_mlsl_handle, g_pdata, PAD_SLOP);
	    error += bma250_set_mode(g_mlsl_handle, g_pdata, BMA250_MODE_NORMAL);

	    cir_flag = 1;
	    if (error)
		return error;
	    printk(KERN_INFO "[BMA250] enable = 1 \n");
		
	} else if(enable == 0){

	    error += bma250_set_Int_Enable(g_mlsl_handle, g_pdata,5, 0);//Slope X
	    error += bma250_set_Int_Enable(g_mlsl_handle, g_pdata,6, 0);//Slope Y
	    error += bma250_set_Int_Enable(g_mlsl_handle, g_pdata,7, 0);//Slope Z	    
	
	    power_key_pressed = 0;
	    cir_flag = 0;
	    if (error)
		return error;
	    printk(KERN_INFO "[BMA250] enable = 0 , power_key_pressed = %d\n", power_key_pressed);

	}
	return count;
}
static ssize_t bma250_clear_powerkey_pressed(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long powerkey_pressed;
	int error;
	error = strict_strtoul(buf, 10, &powerkey_pressed);
	if (error)
	    return error;

	if(powerkey_pressed == 1) {
	    power_key_pressed = 1;
	}
	else if(powerkey_pressed == 0) {
	    power_key_pressed = 0;
	}
	return count;
}
static ssize_t bma250_get_powerkry_pressed(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", power_key_pressed);
}
static DEVICE_ATTR(enable, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		NULL, bma250_enable_interrupt);
static DEVICE_ATTR(clear_powerkey_flag, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		bma250_get_powerkry_pressed, bma250_clear_powerkey_pressed);
#endif
static int bma250_init(void *mlsl_handle,
			  struct ext_slave_descr *slave,
#ifdef CONFIG_CIR_ALWAYS_READY
			  struct ext_slave_platform_data *pdata,
			  int (*power_LPM)(int on)
			  )
#else
			  struct ext_slave_platform_data *pdata
			  )
#endif
{
	tMLError result = 0;
	unsigned char reg = 0;
	unsigned char bw_reg = 0;
#ifdef CONFIG_CIR_ALWAYS_READY
	struct class *bma250_class = NULL;
	struct device *bma250_dev = NULL;
	struct class *bma250_powerkey_class = NULL;
	struct device *bma250_powerkey_dev = NULL;
	int res;
#endif
	struct bma250_private_data *private_data;
	private_data = (struct bma250_private_data *)
		MLOSMalloc(sizeof(struct bma250_private_data));

	printk(KERN_DEBUG "%s\n", __func__);

	if (!private_data)
		return ML_ERROR_MEMORY_EXAUSTED;


#ifdef CONFIG_CIR_ALWAYS_READY

	g_pdata = pdata;
	gsensor_power_LPM = power_LPM;
	g_mlsl_handle = mlsl_handle;

	bma250_class = class_create(THIS_MODULE, "bma250");

	if (IS_ERR(bma250_class)) {
		res = PTR_ERR(bma250_class);
		bma250_class = NULL;
		E("%s, create bma250_class fail!\n", __func__);
		goto err_create_bma250_class_failed;
	}

	bma250_powerkey_class = class_create(THIS_MODULE, "bma250_powerkey");

	if (IS_ERR(bma250_powerkey_class)) {
		res = PTR_ERR(bma250_powerkey_class);
		bma250_powerkey_class = NULL;
		E("%s, create bma250_class fail!\n", __func__);
		goto err_create_bma250_powerkey_class_failed;
	}
	bma250_dev = device_create(bma250_class,
				NULL, 0, "%s", "bma250");
	bma250_powerkey_dev = device_create(bma250_powerkey_class,
				NULL, 0, "%s", "bma250");

	if (unlikely(IS_ERR(bma250_dev))) {
		res = PTR_ERR(bma250_dev);
		bma250_dev = NULL;
		E("%s, create bma250_dev fail!\n", __func__);
		goto err_create_bma250_device;
	}
	if (unlikely(IS_ERR(bma250_powerkey_dev))) {
		res = PTR_ERR(bma250_powerkey_dev);
		bma250_powerkey_dev = NULL;
		E("%s, create bma250_dev fail!\n", __func__);
		goto err_create_bma250_powerkey_device;
	}
	res = device_create_file(bma250_dev, &dev_attr_enable);
	if (res) {
		E("%s, create bma250_device_create_file fail!\n", __func__);
		goto err_create_bma250_device_file;
	}
	res = device_create_file(bma250_powerkey_dev, &dev_attr_clear_powerkey_flag);
	if (res) {
		E("%s, create bma250_device_create_file fail!\n", __func__);
		goto err_create_bma250_powerkey_device_file;
	}

#endif
	pdata->private_data = private_data;

	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
		BMA250_REG_SOFT_RESET, 0xB6);  /* BMA250: Software reset */
	ERROR_CHECK(result);
	MLOSSleep(1);

	result =
	    MLSLSerialRead(mlsl_handle, pdata->address, BOSCH_CTRL_REG, 1,
				&reg);
	ERROR_CHECK(result);

	result =
	    MLSLSerialRead(mlsl_handle, pdata->address, BMA250_BW_REG, 1,
				&bw_reg);
	ERROR_CHECK(result);

	private_data->resume.ctrl_reg = reg;
	private_data->suspend.ctrl_reg = reg;

	private_data->resume.bw_reg = bw_reg;
	private_data->suspend.bw_reg = bw_reg;

	/* TODO Use irq when necessary */
	/*result =
	    MLSLSerialRead(mlsl_handle, pdata->address, BOSCH_INT_REG, 1, &reg);
	ERROR_CHECK(result);*/

	private_data->resume.int_reg = reg;
	private_data->suspend.int_reg = reg;

	private_data->resume.power_mode = 1;
	private_data->suspend.power_mode = 0;

	private_data->state = 0;

	bma250_set_odr(mlsl_handle, pdata, &private_data->suspend,
			FALSE, 0);
	bma250_set_odr(mlsl_handle, pdata, &private_data->resume,
			TRUE, 25000);
	bma250_set_fsr(mlsl_handle, pdata, &private_data->suspend,
			FALSE, 2048);
	bma250_set_fsr(mlsl_handle, pdata, &private_data->resume,
			FALSE, 2048);

	/* TODO Use irq when necessary */
	/*bma250_set_irq(mlsl_handle, pdata, &private_data->suspend,
			FALSE,
			MPU_SLAVE_IRQ_TYPE_NONE);
	bma250_set_irq(mlsl_handle, pdata, &private_data->resume,
			FALSE,
			MPU_SLAVE_IRQ_TYPE_NONE);*/

	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address, BOSCH_PWR_REG,
					0x80);
	ERROR_CHECK(result);

	return result;
#ifdef CONFIG_CIR_ALWAYS_READY
err_create_bma250_powerkey_device_file:
	device_remove_file(bma250_powerkey_dev, &dev_attr_clear_powerkey_flag);
err_create_bma250_device_file:
	device_remove_file(bma250_dev, &dev_attr_enable);
err_create_bma250_powerkey_device:
	device_unregister(bma250_powerkey_dev);
err_create_bma250_device:
	device_unregister(bma250_dev);
err_create_bma250_powerkey_class_failed:
	class_destroy(bma250_powerkey_class);
err_create_bma250_class_failed:
	class_destroy(bma250_class);
	return result;
#endif
}

static int bma250_exit(void *mlsl_handle,
			  struct ext_slave_descr *slave,
			  struct ext_slave_platform_data *pdata)
{
	if (pdata->private_data)
		return MLOSFree(pdata->private_data);
	else
		return ML_SUCCESS;
}

static int bma250_config(void *mlsl_handle,
			struct ext_slave_descr *slave,
			struct ext_slave_platform_data *pdata,
			struct ext_slave_config *data)
{
	struct bma250_private_data *private_data = pdata->private_data;
	if (!data->data)
		return ML_ERROR_INVALID_PARAMETER;

	switch (data->key) {
	case MPU_SLAVE_CONFIG_ODR_SUSPEND:
		return bma250_set_odr(mlsl_handle, pdata,
					&private_data->suspend,
					data->apply,
					*((long *)data->data));
	case MPU_SLAVE_CONFIG_ODR_RESUME:
		return bma250_set_odr(mlsl_handle, pdata,
					&private_data->resume,
					data->apply,
					*((long *)data->data));
	case MPU_SLAVE_CONFIG_FSR_SUSPEND:
		return bma250_set_fsr(mlsl_handle, pdata,
					&private_data->suspend,
					data->apply,
					*((long *)data->data));
	case MPU_SLAVE_CONFIG_FSR_RESUME:
		return bma250_set_fsr(mlsl_handle, pdata,
					&private_data->resume,
					data->apply,
					*((long *)data->data));
	case MPU_SLAVE_CONFIG_IRQ_SUSPEND:
		return bma250_set_irq(mlsl_handle, pdata,
					&private_data->suspend,
					data->apply,
					*((long *)data->data));
	case MPU_SLAVE_CONFIG_IRQ_RESUME:
		return bma250_set_irq(mlsl_handle, pdata,
					&private_data->resume,
					data->apply,
					*((long *)data->data));
	default:
		return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
	};
	return ML_SUCCESS;
}

static int bma250_get_config(void *mlsl_handle,
				struct ext_slave_descr *slave,
				struct ext_slave_platform_data *pdata,
				struct ext_slave_config *data)
{
	struct bma250_private_data *private_data = pdata->private_data;
	if (!data->data)
		return ML_ERROR_INVALID_PARAMETER;

	switch (data->key) {
	case MPU_SLAVE_CONFIG_ODR_SUSPEND:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->suspend.odr;
		break;
	case MPU_SLAVE_CONFIG_ODR_RESUME:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->resume.odr;
		break;
	case MPU_SLAVE_CONFIG_FSR_SUSPEND:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->suspend.fsr;
		break;
	case MPU_SLAVE_CONFIG_FSR_RESUME:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->resume.fsr;
		break;
	case MPU_SLAVE_CONFIG_IRQ_SUSPEND:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->suspend.irq_type;
		break;
	case MPU_SLAVE_CONFIG_IRQ_RESUME:
		(*(unsigned long *)data->data) =
			(unsigned long) private_data->resume.irq_type;
		break;
	default:
		return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
	};

	return ML_SUCCESS;
}

static struct ext_slave_descr bma250_descr = {
	/*.init             = */ bma250_init,
	/*.exit             = */ bma250_exit,
	/*.suspend          = */ bma250_suspend,
	/*.resume           = */ bma250_resume,
	/*.read             = */ bma250_read,
	/*.config           = */ bma250_config,
	/*.get_config       = */ bma250_get_config,
	/*.name             = */ "bma250",
	/*.type             = */ EXT_SLAVE_TYPE_ACCELEROMETER,
	/*.id               = */ ACCEL_ID_BMA250,
	/*.reg              = */ 0x02,
	/*.len              = */ 6,
	/*.endian           = */ EXT_SLAVE_LITTLE_ENDIAN,
	/*.range            = */ {2, 0},
};

struct ext_slave_descr *bma250_get_slave_descr(void)
{
	return &bma250_descr;
}
EXPORT_SYMBOL(bma250_get_slave_descr);

#ifdef __KERNEL__
MODULE_AUTHOR("Invensense");
MODULE_DESCRIPTION("User space IRQ handler for MPU3xxx devices");
MODULE_LICENSE("GPL");
MODULE_ALIAS("bma");
#endif

/**
 *  @}
 */
