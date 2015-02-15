/* arch/arm/mach-msm/include/mach/timex.h
 *
 * Copyright (C) 2007 Google, Inc.
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

#ifndef __ASM_ARCH_MSM_TIMEX_H
#define __ASM_ARCH_MSM_TIMEX_H

#define CLOCK_TICK_RATE		1000000

#ifdef CONFIG_HAVE_ARCH_HAS_CURRENT_TIMER
#define ARCH_HAS_READ_CURRENT_TIMER
#endif

#endif
