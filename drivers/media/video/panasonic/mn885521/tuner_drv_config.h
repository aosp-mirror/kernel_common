/******************************************************************************
 *
 *  file name       : tuner_drv_config.h
 *  brief note      : Driver Config Header
 *
 *  creation data   : 2011.08.28
 *  author          : K.Kitamura(*)
 *  special affairs : none
 *
 *  $Rev:: 1923                       $ Revision of Last commit
 *  $Date:: 2013-11-06 20:09:04 +0900#$ Date of last commit
 *
 *              Copyright (C) 2011 by Panasonic Co., Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 ******************************************************************************
 * HISTORY      : 2011/08/25    K.Kitamura(*)
 *                001 new creation
 *                2012/10/18	K.Okawa(KXD14)
 *                002 modify for MN88552
 ******************************************************************************/

#ifndef _TUNER_DRV_CONFIG_H
#define _TUNER_DRV_CONFIG_H

#define TUNER_SET_ON                     1       
#define TUNER_SET_OFF                    0       

#define TUNER_CONFIG_DRIVER_NAME		"mmtuner_drv"



#define TUNER_CONFIG_INT              0x07       

#define TUNER_CONFIG_I2C_BUSNUM       0x03      

#define TUNER_CONFIG_KTH_PRI            95       


#define TUNER_CONFIG_IRQ_LEVEL  TUNER_SET_ON    

#define TUNER_SLAVE_ADR_S             0x6C      
#define TUNER_SLAVE_ADR_M1            0x6D      
#define TUNER_SLAVE_ADR_M2            0x6E      

#endif
/*******************************************************************************
 *              Copyright(c) 2011 Panasonic Co., Ltd.
 ******************************************************************************/
