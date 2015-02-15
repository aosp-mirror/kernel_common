/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef MSM_JPEG_COMMON_H
#define MSM_JPEG_COMMON_H

#ifdef MSM_JPEG_DEBUG
#define JPEG_DBG(fmt, args...) pr_info(fmt, ##args)
#else
#define JPEG_DBG(fmt, args...) do { } while (0)
#endif

#define JPEG_PR_ERR   pr_err
#define JPEG_DBG_HIGH   pr_debug

enum JPEG_MODE {
	JPEG_MODE_DISABLE,
	JPEG_MODE_OFFLINE,
	JPEG_MODE_REALTIME,
	JPEG_MODE_REALTIME_ROTATION
};

enum JPEG_ROTATION {
	JPEG_ROTATION_0,
	JPEG_ROTATION_90,
	JPEG_ROTATION_180,
	JPEG_ROTATION_270
};

#ifdef pr_err
#undef pr_err
#endif
#define pr_err(fmt, args...) \
         printk(KERN_ERR "[CAM] " pr_fmt(fmt), ## args)

#ifdef pr_info
#undef pr_info
#endif
#define pr_info(fmt, args...) \
        printk(KERN_INFO "[CAM] " pr_fmt(fmt), ## args)

#endif 
