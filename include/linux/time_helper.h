#ifndef _TIME_HELPER_H_
#define _TIME_HELPER_H_

#include <linux/ioctl.h>

#define TIME_HELPER_DEVICE_CLASS_NAME   "time_helper"
#define TIME_HELPER_DEVICE_FILE_NAME    "time_helper"

#define TIME_HELPER_IOC_MAGIC               '*'
#define TIME_HELPER_IOCRESET                _IO(TIME_HELPER_IOC_MAGIC, 0)
#define TIME_HELPER_IOCXMONOTONIC2REALTIME   _IOWR(TIME_HELPER_IOC_MAGIC, 1, struct timespec)
#define TIME_HELPER_IOC_MAXNR               1

#endif // _TIME_HELPER_H_
