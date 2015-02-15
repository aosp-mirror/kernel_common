/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2013, The Linux Foundation. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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

#ifndef __ASM_ARCH_MSM_IRQS_H
#define __ASM_ARCH_MSM_IRQS_H

/*
 * 0-15:  STI/SGI (software triggered/generated interrupts)
 * 16-31: PPI (private peripheral interrupts)
 * 32+:   SPI (shared peripheral interrupts)
 */
#define GIC_PPI_START 16
#define GIC_SPI_START 32

#define MSM_IRQ_BIT(irq)     (1 << ((irq) & 31))

#if defined(CONFIG_ARCH_MSM8974) || defined(CONFIG_ARCH_MPQ8092)

#ifdef CONFIG_ARCH_MSM8974
#include "irqs-8974.h"
#endif

#ifdef CONFIG_ARCH_MPQ8092
#include "irqs-8092.h"
#endif

#elif defined(CONFIG_ARCH_MSM8960) || defined(CONFIG_ARCH_APQ8064) || \
	defined(CONFIG_ARCH_MSM8930)

#ifdef CONFIG_ARCH_MSM8960
#include "irqs-8960.h"
#endif

#ifdef CONFIG_ARCH_MSM8930
#include "irqs-8930.h"
#endif

#ifdef CONFIG_ARCH_APQ8064
#include "irqs-8064.h"
#endif

/* For now, use the maximum number of interrupts until a pending GIC issue
 * is sorted out */
#define NR_MSM_IRQS 288
#define NR_GPIO_IRQS 152
#define NR_PM8921_IRQS 256
#define NR_PM8821_IRQS 112
#define NR_WCD9XXX_IRQS 49
#define NR_TABLA_IRQS NR_WCD9XXX_IRQS
#define NR_GPIO_EXPANDER_IRQS 64
#ifdef CONFIG_PCI_MSI
#define NR_PCIE_MSI_IRQS 256
#define NR_BOARD_IRQS (NR_PM8921_IRQS + NR_PM8821_IRQS + \
		NR_WCD9XXX_IRQS + NR_GPIO_EXPANDER_IRQS + NR_PCIE_MSI_IRQS)
#else
#define NR_BOARD_IRQS (NR_PM8921_IRQS + NR_PM8821_IRQS + \
		NR_WCD9XXX_IRQS + NR_GPIO_EXPANDER_IRQS)
#endif
#define NR_MSM_GPIOS NR_GPIO_IRQS

#else

#if defined(CONFIG_ARCH_MSM9615)
#include "irqs-9615.h"
#elif defined(CONFIG_ARCH_MSM9625)
#include "irqs-9625.h"
#elif defined(CONFIG_ARCH_MSM7X30)
#include "irqs-7x30.h"
#elif defined(CONFIG_ARCH_QSD8X50)
#include "irqs-8x50.h"
#include "sirc.h"
#elif defined(CONFIG_ARCH_MSM8X60)
#include "irqs-8x60.h"
#elif defined(CONFIG_ARCH_MSM7X01A) || defined(CONFIG_ARCH_MSM7X25) \
	|| defined(CONFIG_ARCH_MSM7X27) || defined(CONFIG_ARCH_MSM8625)
#include "irqs-8625.h"
#include "irqs-7xxx.h"

#define NR_GPIO_IRQS 133
#define NR_MSM_IRQS 256
#define NR_BOARD_IRQS 256
#define NR_MSM_GPIOS NR_GPIO_IRQS
#elif defined(CONFIG_ARCH_FSM9XXX)
#include "irqs-fsm9xxx.h"
#include "sirc.h"
#endif

#endif

#if !defined(CONFIG_SPARSE_IRQ)

#if defined(CONFIG_ARCH_MSM8974) || defined(CONFIG_ARCH_MPQ8092)
#define NR_MSM_IRQS 1020 /* Should be 256 - but higher due to bug in sim */
#define NR_GPIO_IRQS 146
#define NR_QPNP_IRQS 32768
#define NR_BOARD_IRQS NR_QPNP_IRQS

#elif defined(CONFIG_ARCH_MSM8610) || defined(CONFIG_ARCH_MSM8226)
#define NR_MSM_IRQS 256
#define NR_GPIO_IRQS 117
#define NR_QPNP_IRQS 32768
#define NR_BOARD_IRQS NR_QPNP_IRQS

#elif defined(CONFIG_ARCH_MSM9625)
#define NR_MSM_IRQS 288
#define NR_GPIO_IRQS 76
#define NR_BOARD_IRQS 0

#endif

#define NR_IRQS (NR_MSM_IRQS + NR_GPIO_IRQS + NR_BOARD_IRQS)
#define MSM_GPIO_TO_INT(n) (NR_MSM_IRQS + (n))
#define FIRST_GPIO_IRQ MSM_GPIO_TO_INT(0)
#define MSM_INT_TO_REG(base, irq) (base + irq / 32)

#endif

#if defined(CONFIG_PCI_MSI) && defined(CONFIG_MSM_PCIE)
#define MSM_PCIE_MSI_INT(n) (NR_MSM_IRQS + NR_GPIO_IRQS + NR_PM8921_IRQS +  \
		NR_PM8821_IRQS + NR_TABLA_IRQS + NR_GPIO_EXPANDER_IRQS + (n))
#endif

#ifdef CONFIG_HTC_POWER_DEBUG
#define EE0_KRAIT_HLOS_SPMI_PERIPH_IRQ (GIC_SPI_START + 190)
#define TLMM_MSM_SUMMARY_IRQ (GIC_SPI_START + 208)
#endif

#endif
