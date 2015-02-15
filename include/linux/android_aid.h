/* include/linux/android_aid.h
 *
 * Copyright (C) 2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_ANDROID_AID_H
#define _LINUX_ANDROID_AID_H

/* AIDs that the kernel treats differently */
#define AID_NET_BT_ADMIN 3001
#define AID_NET_BT       3002
#define AID_INET         3003
#define AID_NET_RAW      3004
#define AID_NET_ADMIN    3005
#define AID_NET_BW_STATS 3006  /* read bandwidth statistics */
#define AID_NET_BW_ACCT  3007  /* change bandwidth statistics accounting */

#endif
