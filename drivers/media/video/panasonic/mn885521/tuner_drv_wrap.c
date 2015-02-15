/******************************************************************************
 *
 *  file name       : tuner_drv_wrap.c
 *  brief note      : The Wrapper Layer for Tmm Tuner Driver
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
 * HISTORY      : 2011/07/25    K.Kitamura(*)
 *                001 new creation
 ******************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include "tuner_drv.h"

#ifdef TUNER_CONFIG_IRQ_PC_LINUX
#include "../../../i2c-parport-x/i2c-parport.h"
#endif  

int tuner_drv_ctl_power( int data );
int tuner_drv_set_interrupt( void );
void tuner_drv_release_interrupt( void );

int tuner_drv_ctl_power( int data )
{
    
    if( data == TUNER_DRV_CTL_POWON )
    {
    }
    
    else
    {
    }

    
    return 0;
}

int tuner_drv_set_interrupt( void )
{
#ifndef TUNER_CONFIG_IRQ_PC_LINUX
    int ret;                                     

    
    ret = request_irq( TUNER_CONFIG_INT,         
                       tuner_interrupt,          
                       IRQF_DISABLED,            
                       "mm_tuner",               
                       NULL );                   

    if( ret != 0 )                               
    {
        return -1;
    }
#else  
    i2c_set_interrupt( tuner_interrupt );
#endif 
    return 0;
}

void tuner_drv_release_interrupt( void )
{
#ifndef TUNER_CONFIG_IRQ_PC_LINUX
    
    free_irq( TUNER_CONFIG_INT, NULL );

#else  
    i2c_release_interrupt( NULL );
#endif 
}

#ifdef TUNER_CONFIG_IRQ_LEVELTRIGGER
void tuner_drv_enable_interrupt( void )
{
#ifndef TUNER_CONFIG_IRQ_PC_LINUX
    
    enable_irq( TUNER_INT, NULL );

#else  
    i2c_set_interrupt( tuner_interrupt );
#endif 
}

void tuner_drv_disable_interrupt( void )
{
#ifndef TUNER_CONFIG_IRQ_PC_LINUX 
    
    disable_irq( TUNER_INT, NULL );

#else  
    i2c_release_interrupt( NULL );
#endif 
}
#endif 

/*******************************************************************************
 *              Copyright(c) 2011 Panasonc Co., Ltd.
 ******************************************************************************/
