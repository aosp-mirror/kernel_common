/******************************************************************************
 *
 *  file name       : tuner_drv_hw.c
 *  brief note      : HW Control Layer of Tmm Tuner Driver
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
#include <asm/uaccess.h>
#include "tuner_drv.h"

#include <linux/mm.h>
#include <linux/vmalloc.h>

int tuner_drv_hw_access( unsigned int uCommand, TUNER_DATA_RW *data,
                         unsigned short param_len );

int tuner_drv_hw_access( unsigned int uCommand, TUNER_DATA_RW *data,
                         unsigned short param_len )
{
    int                ret;
    struct i2c_adapter *adap;
    struct i2c_msg     msgs[2];
    unsigned short     addr;
    unsigned short     flags;
    unsigned char      buf[TUNER_I2C_MSG_DATA_NUM];
    
    unsigned char      read_data;
    
    unsigned char      ena_data;
    unsigned char      write_data;
    unsigned short     loop_cnt;


    
    if( data == NULL )
    {
        DEBUG_PRINT("tuner_drv_hw_access -1- ");

        TRACE();
        return -EINVAL;
    }

    
    adap = i2c_get_adapter( TUNER_CONFIG_I2C_BUSNUM );
    if( adap == NULL )
    {
	DEBUG_PRINT("tuner_drv_hw_access -2- ");
        TRACE();
        return -EINVAL;
    }

    
    memset( msgs, 0x00, sizeof(struct i2c_msg) * 2 );
    ena_data = 0x00;
    flags = 0;
#if 0
    
    if( param_len >= TUNER_I2C_WRITE_ALL_NUM )
    {
        
        buf_all  = ( unsigned char * )vmalloc( param_len + 1 );

        if( buf_all  == NULL )
        {
            TRACE();
            i2c_put_adapter( adap );
            return -EINVAL;
        }

        
        memset( buf_all,
                0x00,
                param_len + 1 );

        
        msgs[ 0 ].addr  = data[ 0 ].slave_adr;
        msgs[ 0 ].flags = 0;
        msgs[ 0 ].len   = ( unsigned short )( param_len + 1 );
        msgs[ 0 ].buf   = buf_all;

        *buf_all = ( unsigned char )data[ 0 ].adr;

        DEBUG_PRINT("//Write slaveAdr, adr                           || 0x%02x, 0x%2x",
                    data[ 0 ].slave_adr, buf_all[ 0 ]);
        for( loop_cnt = 0; loop_cnt < param_len; loop_cnt++ )
        {
            buf_all[ loop_cnt + 1 ] =  ( unsigned char )data[ loop_cnt ].param;
            DEBUG_PRINT("//Write                                  w_data || 0x%02x",
                        buf_all[ loop_cnt + 1 ]);
        }

        
        ret = i2c_transfer( adap, msgs, TUNER_W_MSGNUM );

        
        vfree( buf_all );

        if( ret < 0 )
        {
            TRACE();
            i2c_put_adapter( adap );
            return -EINVAL;
        }

        i2c_put_adapter( adap );
        return 0;
    }
#endif
    
    for( loop_cnt = 0; loop_cnt < param_len; loop_cnt++ ) {
        
        memset( msgs, 0x00, sizeof(struct i2c_msg) * 2 );
        ena_data = 0x00;

        
        switch ( uCommand ) {
             case TUNER_IOCTL_VALGET:	
                addr   = data[ loop_cnt ].slave_adr;
                flags  = I2C_M_RD;
                buf[ 0 ] = (unsigned char)data[ loop_cnt ].adr;
                buf[ 1 ] = 0;
                break;
            case TUNER_IOCTL_VALSET:	
                addr   = data[ loop_cnt ].slave_adr;
                flags  = 0;
                buf[ 0 ] = (unsigned char)data[ loop_cnt ].adr;
                buf[ 1 ] = (unsigned char)data[ loop_cnt ].param;
                break;
            default:
                i2c_put_adapter( adap );
                return -EINVAL;
        }

        if( flags != I2C_M_RD ) {
        	 
        	 if( !(( data[ loop_cnt ].sbit == 0 ) && ( data[ loop_cnt ].ebit == 7 ))) {
            	
                
                if(( data[ loop_cnt ].sbit == TUNER_SET_ENADATA )
                && ( data[ loop_cnt ].ebit == TUNER_SET_ENADATA ))
                {
                    ena_data = ( unsigned char )data[ loop_cnt ].enabit; 
                }
                else
                {
                    
                	ena_data = (unsigned char)(((1U << (1+data[loop_cnt].ebit-data[loop_cnt].sbit))-1) << data[loop_cnt].sbit);
#if 0
                    for( cnt = 0; cnt < TUNER_CHARNUM_MAX; cnt++ )
                    {
                        if(( cnt >= data[ loop_cnt ].sbit )
                            && ( cnt <= data[ loop_cnt ].ebit ))
                        {
                            ena_data |= ( unsigned char )( 1 << cnt );
                        }
                    }
#endif
                }
                
                if( ena_data != 0xFF )
                {
                    
                    msgs[ 0 ].addr  = addr;
                    msgs[ 0 ].flags = 0;
                    msgs[ 0 ].len   = TUNER_R_MSGLEN;
                    msgs[ 0 ].buf   = &buf[ 0 ];
                    msgs[ 1 ].addr  = addr;
                    msgs[ 1 ].flags = I2C_M_RD;
                    msgs[ 1 ].len   = TUNER_R_MSGLEN;
                    msgs[ 1 ].buf   = &read_data;

                    ret = i2c_transfer( adap, msgs, TUNER_R_MSGNUM );
                    if( ret < 0 )
                    {
			DEBUG_PRINT("tuner_drv_hw_access -3- ");
                        TRACE();
                        i2c_put_adapter( adap );
                        return -EINVAL;
                    }

                    
                    memset( msgs, 0x00, sizeof( struct i2c_msg ) * 2 );

                    
                    read_data  &= ( unsigned char )( ~ena_data );
                    
                    write_data = ( unsigned char )( ena_data & data[ loop_cnt ].param );
                    buf[ 1 ] = ( unsigned char )( write_data | read_data );
                }
            }

            
            

            msgs[ 0 ].addr  = addr;
            msgs[ 0 ].flags = flags;
            msgs[ 0 ].len   = TUNER_W_MSGLEN;
            msgs[ 0 ].buf   = buf;


            ret = i2c_transfer( adap, msgs, TUNER_W_MSGNUM );
            if( ret < 0 )
            {
		DEBUG_PRINT("tuner_drv_hw_access -4- ");
                TRACE();
                i2c_put_adapter( adap );
                return -EINVAL;
            }
        } else {
        	
            msgs[ 0 ].addr  = addr;
            msgs[ 0 ].flags = 0;
            msgs[ 0 ].len   = TUNER_R_MSGLEN;
            msgs[ 0 ].buf   = &buf[ 0 ];
            msgs[ 1 ].addr  = addr;
            msgs[ 1 ].flags = flags;
            msgs[ 1 ].len   = TUNER_R_MSGLEN;
            msgs[ 1 ].buf   = &buf[ 1 ];

            ret = i2c_transfer( adap, msgs, TUNER_R_MSGNUM );
            if( ret < 0 )
            {
		DEBUG_PRINT("tuner_drv_hw_access, ret = %x -5- ", ret);
                DEBUG_PRINT("ioctl(read(Pre)) slv:0x%02x adr:0x%02x (single)",
                                                              addr, buf[ 0 ]);

                TRACE();
                i2c_put_adapter( adap );
                return -EINVAL;
            }

            
            data[ loop_cnt ].param = buf[ 1 ];
            
            

        }
    }
    i2c_put_adapter( adap );
    return 0;
}

/*******************************************************************************
 *              Copyright(c) 2011 Panasonc Co., Ltd.
 ******************************************************************************/
