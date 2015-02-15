/* ncp6924.h
 *
 * Copyright (C) 2013 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _NCP6924_H_
#define _NCP6924_H_

#include <linux/types.h>

#define NCP6924_I2C_NAME		"ncp6924"
#define NCP6924_ENABLE			0x14
#define NCP6924_VPROGDCDC1		0x20
#define NCP6924_VPROGDCDC2		0x22
#define NCP6924_VPROGLDO1		0x24
#define NCP6924_VPROGLDO2		0x25
#define NCP6924_VPROGLDO3		0x26
#define NCP6924_VPROGLDO4		0x27

#define NCP6924_DCDC1_ENABLE_BIT	0x1
#define NCP6924_DCDC2_ENABLE_BIT	0x3
#define NCP6924_LDO1_ENABLE_BIT		0x4
#define NCP6924_LDO2_ENABLE_BIT		0x5
#define NCP6924_LDO3_ENABLE_BIT		0x6
#define NCP6924_LDO4_ENABLE_BIT		0x7

#define NCP6924_ID_DCDC1		1
#define NCP6924_ID_DCDC2		2
#define NCP6924_ID_LDO1			1
#define NCP6924_ID_LDO2			2
#define NCP6924_ID_LDO3			3
#define NCP6924_ID_LDO4			4

#endif /* _NCP6924_H_ */
