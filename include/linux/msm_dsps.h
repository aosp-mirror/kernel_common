#ifndef _DSPS_H_
#define _DSPS_H_

#include <linux/ioctl.h>

#define DSPS_IOCTL_MAGIC 'd'

#define DSPS_IOCTL_ON	_IO(DSPS_IOCTL_MAGIC, 1)
#define DSPS_IOCTL_OFF	_IO(DSPS_IOCTL_MAGIC, 2)

#define DSPS_IOCTL_READ_SLOW_TIMER _IOR(DSPS_IOCTL_MAGIC, 3, unsigned int*)
#define DSPS_IOCTL_READ_FAST_TIMER _IOR(DSPS_IOCTL_MAGIC, 4, unsigned int*)

#define DSPS_IOCTL_RESET _IO(DSPS_IOCTL_MAGIC, 5)

#endif	/* _DSPS_H_ */
