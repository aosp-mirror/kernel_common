/*
 * Copyright (c) 2008-2013, The Linux Foundation. All rights reserved.
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

#ifndef __ASM_ARCH_MSM_IOMAP_8974_H
#define __ASM_ARCH_MSM_IOMAP_8974_H

/* Physical base address and size of peripherals.
 * Ordered by the virtual base addresses they will be mapped at.
 *
 * If you add or remove entries here, you'll want to edit the
 * io desc array in arch/arm/mach-msm/io.c to reflect your
 * changes.
 *
 */

#define MSM8974_MSM_SHARED_RAM_PHYS	0x0FA00000

#define MSM8974_QGIC_DIST_PHYS	0xF9000000
#define MSM8974_QGIC_DIST_SIZE	SZ_4K

#define MSM8974_QGIC_CPU_PHYS	0xF9002000
#define MSM8974_QGIC_CPU_SIZE	SZ_4K

#define MSM8974_TLMM_PHYS	0xFD510000
#define MSM8974_TLMM_SIZE	SZ_16K

#define MSM8974_MPM2_PSHOLD_PHYS	0xFC4AB000
#define MSM8974_MPM2_PSHOLD_SIZE	SZ_4K

#ifdef CONFIG_DEBUG_MSM8974_UART
#define MSM_DEBUG_UART_BASE	IOMEM(0xFA71E000)
#define MSM_DEBUG_UART_PHYS	0xF991E000
#endif

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
#define HTC_DEBUG_FOOTPRINT_PHYS	0x05B00000 + (SZ_1M - SZ_64K + SZ_4K) // 0x05BF1000
#define HTC_DEBUG_FOOTPRINT_BASE	IOMEM(0xFE000000)
#define HTC_DEBUG_FOOTPRINT_SIZE	SZ_4K
#endif

#define MSM_MPM_SLEEPTICK_BASE		IOMEM(0xFE010000)
#define MSM8974_MPM_SLEEPTICK_PHYS	0xFC4A3000
#define MSM8974_MPM_SLEEPTICK_SIZE	SZ_4K

#endif
