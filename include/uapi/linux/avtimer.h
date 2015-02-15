#ifndef _UAPI_AVTIMER_H
#define _UAPI_AVTIMER_H

#include <linux/ioctl.h>

#define MAJOR_NUM 100

#define IOCTL_GET_AVTIMER_TICK _IOR(MAJOR_NUM, 0, char *)
struct dev_avtimer_data {
	uint32_t avtimer_msw_phy_addr;
	uint32_t avtimer_lsw_phy_addr;
};

#endif
