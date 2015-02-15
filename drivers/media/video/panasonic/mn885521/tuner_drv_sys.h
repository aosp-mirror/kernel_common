/******************************************************************************
 *
 *  file name       : tuner_drv_sys.h
 *  brief note      : The Header for Driver Public Presentation
 *
 *  creation data   : 2011.08.01
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
#ifndef _TUNER_DRV_SYS_H
#define _TUNER_DRV_SYS_H

#define TUNER_IOC_MAGIC 'd'
#define TUNER_IOCTL_VALGET        _IOW(TUNER_IOC_MAGIC, 1, struct _tuner_data_rw)
#define TUNER_IOCTL_VALSET        _IOR(TUNER_IOC_MAGIC, 2, struct _tuner_data_rw)
#define TUNER_IOCTL_VALGET_EVENT  _IOR(TUNER_IOC_MAGIC, 3, struct _tuner_data_rw)
#define TUNER_IOCTL_VALSET_POWER  _IOR(TUNER_IOC_MAGIC, 4, struct _tuner_data_rw)
#define TUNER_IOCTL_VALSET_EVENT  _IOW(TUNER_IOC_MAGIC, 5, struct _tuner_data_rw)
#define TUNER_IOCTL_VALREL_EVENT  _IOW(TUNER_IOC_MAGIC, 6, struct _tuner_data_rw)

#define TUNER_DRV_CTL_POWON              0       
#define TUNER_DRV_CTL_POWOFF             1       

#define TUNER_SET_ENADATA             0xFF       

typedef struct _tuner_data_rw {
    unsigned short slave_adr;                     
    unsigned short adr;                           
    unsigned short sbit;                          
    unsigned short ebit;                          
    unsigned short param;                         
    unsigned short enabit;                        
} TUNER_DATA_RW ;

#if 0
struct ioctl_cmd {
    unsigned int reg;                            
    unsigned int offset;                         
    unsigned int val;                            
};
#endif
#endif
/*******************************************************************************
 *              Copyright(c) 2011 Panasonc Co., Ltd.
 ******************************************************************************/
