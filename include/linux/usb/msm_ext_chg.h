#ifndef __LINUX_USB_MSM_EXT_CHG_H
#define __LINUX_USB_MSM_EXT_CHG_H

#include <linux/ioctl.h>

#define USB_CHG_BLOCK_ULPI	1
#define USB_CHG_BLOCK_QSCRATCH	2

#define USB_REQUEST_5V		1
#define USB_REQUEST_9V		2
struct msm_usb_chg_info {
	uint32_t chg_block_type;
	off_t page_offset;
	size_t length;
};

#define MSM_USB_EXT_CHG_INFO _IOW('M', 0, struct msm_usb_chg_info)

#define MSM_USB_EXT_CHG_BLOCK_LPM _IOW('M', 1, int)

#define MSM_USB_EXT_CHG_VOLTAGE_INFO _IOW('M', 2, int)

#define MSM_USB_EXT_CHG_RESULT _IOW('M', 3, int)

#define MSM_USB_EXT_CHG_TYPE _IOW('M', 4, int)

#endif 
