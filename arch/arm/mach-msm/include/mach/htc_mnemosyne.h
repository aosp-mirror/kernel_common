/* arch/arm/mach-msm/htc_mnemosyne.h
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
 *
 */
#ifndef __MACH_MNEMOSYNE_H
#define __MACH_MNEMOSYNE_H

/* XXX: assembly do not support sizeof bulletin command,
 * set element size to calculate offset in assembly.
 */
#define MNEMOSYNE_ELEMENT_TYPE			uint32_t	/* DO NOT FORGOT TO CHANGE ELEMENT SIZE. */
#define MNEMOSYNE_ELEMENT_SIZE			4		/* in bytes */
#define MNEMOSYNE_ELEMENT_SIZE_BIT_SHIFT	2		/* for asm use to get shift bit for size. */

#ifdef __ASSEMBLY__
#include <linux/linkage.h>
#include <linux/threads.h>
#include <mach/msm_iomap.h>

/* cannot include linux/kernel.h here, define it. */
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

#define DECLARE_MNEMOSYNE_START()
#define DECLARE_MNEMOSYNE(name)			ASM_ENUM	mnemosyne_##name
#define DECLARE_MNEMOSYNE_ARRAY(name, number)	ASM_ENUM_ARRAY	mnemosyne_##name, number

/* Emulate enum in assembly */
.SET LAST_ENUM_VALUE, 0

.MACRO ASM_ENUM_ARRAY name, number
/* Export the offset of an element, we can address by getting base and adding this offset */
.EQUIV \name, LAST_ENUM_VALUE * MNEMOSYNE_ELEMENT_SIZE
.SET LAST_ENUM_VALUE, LAST_ENUM_VALUE + \number
.ENDM

.MACRO ASM_ENUM name
ASM_ENUM_ARRAY \name, 1
.ENDM

#define DECLARE_MNEMOSYNE_END()

/* For physical memory space, we need to do virt_to_phys manually,
   MACRO cannot work. */
#else
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#define DECLARE_MNEMOSYNE_START()		struct mnemosyne_data {
#define DECLARE_MNEMOSYNE(name)			MNEMOSYNE_ELEMENT_TYPE name;
#define DECLARE_MNEMOSYNE_ARRAY(name, number)	MNEMOSYNE_ELEMENT_TYPE name[number];
#define DECLARE_MNEMOSYNE_END()			};
#endif /* __ASSEMBLY__ */

#define NORMAL_FOOTPRINT_GROUP_OFFSET		(0x100)

#include <mach/htc_mnemosyne_footprint.inc>

/* For c/c++ codes, export API to set and get footprints. */
#ifndef __ASSEMBLY__
#define MNEMOSYNE_SET(f, v)		do {								\
						if (mnemosyne_get_base()) {mnemosyne_get_base()->f = (MNEMOSYNE_ELEMENT_TYPE )(v);}	\
					} while(0);

#define MNEMOSYNE_SET_I(f, i, v)	do {									\
						if (mnemosyne_get_base()) {mnemosyne_get_base()->f[i] = (MNEMOSYNE_ELEMENT_TYPE)(v);}	\
					} while(0);

#define MNEMOSYNE_GET(f)		((mnemosyne_get_base())?mnemosyne_get_base()->f:0)

#define MNEMOSYNE_GET_ADDR(f)		((mnemosyne_get_base())?&mnemosyne_get_base()->f:NULL)

#define MNEMOSYNE_GET_I(f, i)		((mnemosyne_get_base())?mnemosyne_get_base()->f[i]:0)

#define MNEMOSYNE_GET_ADDR_I(f, i)	((mnemosyne_get_base())?&mnemosyne_get_base()->f[i]:NULL)

struct mnemosyne_platform_data {
	u32 phys;
	u32 base;
};

struct mnemosyne_data *mnemosyne_get_base(void);
int mnemosyne_early_init(unsigned int phys, unsigned base);
#endif /* __ASSEMBLY__ */
#endif
