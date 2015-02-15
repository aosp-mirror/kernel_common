/*****************************************************************************
 Copyright(c) 2010 NMI Inc. All Rights Reserved
 
 File name : nmi_hw.h
 
 Description : NM625 host interface
 
 History : 
 ----------------------------------------------------------------------
 2010/05/17 	ssw		initial
*******************************************************************************/

#ifndef __NMI_HW_H__
#define __NMI_HW_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/ioctl.h>

#define MAX_OPEN_NUM 		8

#define IOCTL_MAGIC	't'
#define NMI625_IOCTL_BUF_SIZE   256

typedef struct {
	unsigned char dev_addr;
	unsigned short size;
	unsigned char buf[NMI625_IOCTL_BUF_SIZE];
} ioctl_info;

#define IOCTL_MAXNR			27

#define IOCTL_DMB_RESET			_IOW( IOCTL_MAGIC, 0, ioctl_info)
#define IOCTL_DMB_PROBE			_IO( IOCTL_MAGIC, 1 )
#define IOCTL_DMB_INIT		 	_IO( IOCTL_MAGIC, 2 )
#define IOCTL_DMB_DEINIT	 	_IO( IOCTL_MAGIC, 3 )

#define IOCTL_DMB_BYTE_READ 		_IOWR( IOCTL_MAGIC, 4, ioctl_info )
#define IOCTL_DMB_WORD_READ 		_IOWR( IOCTL_MAGIC, 5, ioctl_info )
#define IOCTL_DMB_LONG_READ 		_IOWR( IOCTL_MAGIC, 6, ioctl_info )
#define IOCTL_DMB_BULK_READ 		_IOWR( IOCTL_MAGIC, 7, ioctl_info )

#define IOCTL_DMB_BYTE_WRITE 		_IOW( IOCTL_MAGIC, 8, ioctl_info )
#define IOCTL_DMB_WORD_WRITE 		_IOW( IOCTL_MAGIC, 9, ioctl_info )
#define IOCTL_DMB_LONG_WRITE 		_IOW( IOCTL_MAGIC, 10, ioctl_info )
#define IOCTL_DMB_BULK_WRITE 		_IOW( IOCTL_MAGIC, 11, ioctl_info )

#define IOCTL_DMB_TUNER_READ	 	_IOWR( IOCTL_MAGIC, 12, ioctl_info )
#define IOCTL_DMB_TUNER_WRITE	 	_IOW( IOCTL_MAGIC, 13, ioctl_info )

#define IOCTL_DMB_TUNER_SET_FREQ 	_IOW( IOCTL_MAGIC, 14, ioctl_info )
#define IOCTL_DMB_TUNER_SELECT	 	_IOW( IOCTL_MAGIC, 15, ioctl_info )
#define IOCTL_DMB_TUNER_DESELECT 	_IO( IOCTL_MAGIC, 16 )
#define IOCTL_DMB_TUNER_GET_RSSI 	_IOWR( IOCTL_MAGIC, 17, ioctl_info )

#define IOCTL_DMB_HOSTIF_SELECT 	_IOW( IOCTL_MAGIC, 18, ioctl_info )
#define IOCTL_DMB_HOSTIF_DESELECT 	_IO( IOCTL_MAGIC, 19 )

#define IOCTL_DMB_POWER_ON		_IO( IOCTL_MAGIC, 20 )
#define IOCTL_DMB_POWER_OFF		_IO( IOCTL_MAGIC, 21 )


///+
#define IOCTL_DMB_ANTENNA_SELECT 	_IOW( IOCTL_MAGIC, 26, ioctl_info )
///-

typedef struct {
	long		index;
	void		**hInit;
	void		*hI2C;
} DMB_OPEN_INFO_T;

#ifdef __cplusplus
}
#endif

extern int oneseg_power(int on);

#endif // __NMI_HW_H__

