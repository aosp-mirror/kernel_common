/******************************************************************************
 *
 *  file name       : tuner_drv.h
 *  brief note      : Driver Common Header
 *
 *  creation data   : 2011.07.25
 *  author          : K.Kitamura(*)
 *  special affairs : none
 *
 *  $Rev:: 1720                       $ Revision of Last commit
 *  $Date:: 2013-05-08 22:02:48 +0900#$ Date of last commit
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
 ******************************************************************************/
#ifndef _TUNER_DRV_H
#define _TUNER_DRV_H

#include <linux/interrupt.h>
#include "tuner_drv_config.h"
#include "tuner_drv_sys.h"

#if 0
#define IOCTL_VALGET _IOW(TUNER_IOC_MAGIC, 1, struct ioctl_cmd)
#define IOCTL_VALSET _IOR(TUNER_IOC_MAGIC, 2, struct ioctl_cmd)
#endif
#define TUNER_OFF                     0x00       
#define TUNER_ON                      0x01       

#define TUNER_CHARNUM_MAX             0x08       

#define REG_INTDEF1_F (0xDC)	
#define BANK_OF_INTDEF1_F (TUNER_SLAVE_ADR_M1)
#define SIG_ENE_INTDEF1_F (7)
#define SIG_ENS_INTDEF1_F (0)
#define SIG_ENA_INTDEF1_F (0xFF)

#define REG_INTDEF2_F (0xDD)	
#define BANK_OF_INTDEF2_F (TUNER_SLAVE_ADR_M1)
#define SIG_ENE_INTDEF2_F (3)
#define SIG_ENS_INTDEF2_F (0)
#define SIG_ENA_INTDEF2_F (0x0F)

#define REG_INTDEF1_S (0xC7)	
#define BANK_OF_INTDEF1_S (TUNER_SLAVE_ADR_M2)
#define SIG_ENE_INTDEF1_S (7)
#define SIG_ENS_INTDEF1_S (0)
#define SIG_ENA_INTDEF1_S (0xFF)

#define REG_INTDEF2_S (0xC8)	
#define BANK_OF_INTDEF2_S (TUNER_SLAVE_ADR_M2)
#define SIG_ENE_INTDEF2_S (3)
#define SIG_ENS_INTDEF2_S (0)
#define SIG_ENA_INTDEF2_S (0x0F)

#define TUNER_DRV_ADR_INTCND_F        0xE3
#define TUNER_DRV_ADR_INTCND_S        0xCE
#define TUNER_DRV_ENAS_ALL               0
#define TUNER_DRV_ENAE_ALL               7
#define TUNER_DRV_ENA_ALL             0xFF
#define TUNER_DRV_PARAM_RINIT         0x00
#define TUNER_DRV_PARAM_ALL           0xFF

#define TUNER_DRV_ADR_INTST_F			(0xE4)
#define TUNER_DRV_ADR_INTST_S			(0xCF)

#define TUNER_R_MSGLEN                0x01       
#define TUNER_W_MSGLEN                0x02       
#define TUNER_R_MSGNUM                0x02       
#define TUNER_W_MSGNUM                0x01       

#define TUNER_I2C_WRITE_ALL_NUM     (1000)       
#define TUNER_I2C_MSG_DATA_NUM         (2)       

#define TUNER_EVENT_REGNUM             (4)       

#define TUNER_KTH_NONE          0x00000000       
#define TUNER_KTH_IRQHANDLER    0x00000001       
#define TUNER_KTH_END           0x80000000       

#define TUNER_KTH_ENDFLG_OFF          0x00       
#define TUNER_KTH_ENDFLG_ON           0x01       

#define DLOG
#ifdef  DLOG
#define ERROR_PRINT( fmt, arg... )  { printk(KERN_ERR  "%s: " fmt "\n", "mmtuner driver", ##arg); }
#define INFO_PRINT( fmt, arg... )   { printk(KERN_INFO "%s: " fmt "\n", "mmtuner driver", ##arg); }
#define DEBUG_PRINT( fmt, arg... )  { printk(KERN_INFO "%s: " fmt "\n", "mmtuner driver", ##arg); }
#define TRACE() DEBUG_PRINT( "%s( %d )", __FUNCTION__, __LINE__ )
#else
#define ERROR_PRINT( fmt, arg... )
#define INFO_PRINT( fmt, arg... )  
#define DEBUG_PRINT( fmt, arg... )
#define TRACE() DEBUG_PRINT()
#endif 

extern int tuner_drv_hw_access( unsigned int uCommand,
                                TUNER_DATA_RW *data,
                                unsigned short param_len );
extern int tuner_drv_ctl_power( int data );
extern int tuner_drv_set_interrupt( void );
extern void tuner_drv_release_interrupt( void );
extern void tuner_drv_enable_interrupt( void );
extern void tuner_drv_disable_interrupt( void );
#ifndef TUNER_CONFIG_IRQ_PC_LINUX
extern irqreturn_t tuner_interrupt( int irq, void *dev_id );
#else  
extern int tuner_interrupt( void );
#endif 

#endif 
/*******************************************************************************
 *              Copyright(c) 2011 Panasonc Co., Ltd.
 ******************************************************************************/
