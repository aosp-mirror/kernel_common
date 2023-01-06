/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2022-2023, Intel Corporation.
 */

#ifndef __T7XX_PORT_DEVLINK_H__
#define __T7XX_PORT_DEVLINK_H__

#include <net/devlink.h>
#include <linux/string.h>

#include "t7xx_pci.h"

#define T7XX_MAX_QUEUE_LENGTH 32
#define T7XX_FB_COMMAND_SIZE  64
#define T7XX_FB_RESPONSE_SIZE 512
#define T7XX_FB_MCMD_SIZE     64
#define T7XX_FB_MDATA_SIZE    1024
#define T7XX_FB_RESP_COUNT    30

#define T7XX_FB_CMD_RTS          "_RTS"
#define T7XX_FB_CMD_CTS          "_CTS"
#define T7XX_FB_CMD_FIN          "_FIN"
#define T7XX_FB_CMD_OEM_MRDUMP   "oem mrdump"
#define T7XX_FB_CMD_OEM_LKDUMP   "oem dump_pllk_log"
#define T7XX_FB_CMD_DOWNLOAD     "download"
#define T7XX_FB_CMD_FLASH        "flash"
#define T7XX_FB_CMD_REBOOT       "reboot"
#define T7XX_FB_RESP_MRDUMP_DONE "MRDUMP08_DONE"
#define T7XX_FB_RESP_OKAY        "OKAY"
#define T7XX_FB_RESP_FAIL        "FAIL"
#define T7XX_FB_RESP_DATA        "DATA"
#define T7XX_FB_RESP_INFO        "INFO"
#define T7XX_FB_CMD_GET_VER      "get_version"

#define T7XX_FB_EVENT_SIZE      50

#define T7XX_MAX_SNAPSHOTS  1
#define T7XX_MRDUMP_SIZE    (160 * 1024 * 1024)
#define T7XX_LKDUMP_SIZE    (256 * 1024)
#define T7XX_TOTAL_REGIONS  2

#define T7XX_FLASH_STATUS   0
#define T7XX_MRDUMP_STATUS  1
#define T7XX_LKDUMP_STATUS  2
#define T7XX_GET_INFO       3
#define T7XX_DEVLINK_IDLE   0

#define T7XX_NORMAL_MODE    0
#define T7XX_FB_DL_MODE     1
#define T7XX_FB_DUMP_MODE   2

/* Internal region indexes */
enum t7xx_regions {
	T7XX_MRDUMP_INDEX,
	T7XX_LKDUMP_INDEX,
};

struct t7xx_devlink_region_info {
	const char *name;
	size_t size;
};

struct t7xx_devlink_region {
	struct t7xx_devlink_region_info *info;
	struct devlink_region_ops ops;
	struct devlink_region *dlreg;
	void *buf;
};

struct t7xx_devlink {
	struct t7xx_devlink_region regions[T7XX_TOTAL_REGIONS];
	struct t7xx_pci_dev *t7xx_dev;
	struct workqueue_struct *wq;
	struct t7xx_port *port;
	struct work_struct ws;
	struct devlink *ctx;
	unsigned long status;
	u8 mode;
};

bool t7xx_devlink_param_get_fastboot(struct devlink *devlink);
int t7xx_devlink_register(struct t7xx_pci_dev *t7xx_dev);
void t7xx_devlink_unregister(struct t7xx_pci_dev *t7xx_dev);

#endif /*__T7XX_PORT_DEVLINK_H__*/
