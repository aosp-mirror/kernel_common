/* Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
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

#ifndef __ARCH_ARM_MACH_MSM_MSM_WATCHDOG_H
#define __ARCH_ARM_MACH_MSM_MSM_WATCHDOG_H

#define WDT0_OFFSET	0x38
#define WDT1_OFFSET	0x60

struct msm_watchdog_pdata {
	
	unsigned int pet_time;
	
	unsigned int bark_time;
	bool has_secure;
	bool needs_expired_enable;
	bool has_vic;
	
	bool use_kernel_fiq;
	void __iomem *base;
};

struct msm_watchdog_dump {
	uint32_t magic;
	uint32_t curr_cpsr;
	uint32_t usr_r0;
	uint32_t usr_r1;
	uint32_t usr_r2;
	uint32_t usr_r3;
	uint32_t usr_r4;
	uint32_t usr_r5;
	uint32_t usr_r6;
	uint32_t usr_r7;
	uint32_t usr_r8;
	uint32_t usr_r9;
	uint32_t usr_r10;
	uint32_t usr_r11;
	uint32_t usr_r12;
	uint32_t usr_r13;
	uint32_t usr_r14;
	uint32_t irq_spsr;
	uint32_t irq_r13;
	uint32_t irq_r14;
	uint32_t svc_spsr;
	uint32_t svc_r13;
	uint32_t svc_r14;
	uint32_t abt_spsr;
	uint32_t abt_r13;
	uint32_t abt_r14;
	uint32_t und_spsr;
	uint32_t und_r13;
	uint32_t und_r14;
	uint32_t fiq_spsr;
	uint32_t fiq_r8;
	uint32_t fiq_r9;
	uint32_t fiq_r10;
	uint32_t fiq_r11;
	uint32_t fiq_r12;
	uint32_t fiq_r13;
	uint32_t fiq_r14;
};

void msm_wdog_fiq_setup(void *stack);
extern unsigned int msm_wdog_fiq_length, msm_wdog_fiq_start;
extern unsigned int msm7k_fiq_start, msm7k_fiq_length;

#ifdef CONFIG_MSM_WATCHDOG
void pet_watchdog(void);
#else
static inline void pet_watchdog(void) { }
#endif

#if defined(CONFIG_HTC_DEBUG_WATCHDOG)
void msm_watchdog_bark(void);
void msm_watchdog_reset(void);
int msm_watchdog_suspend_deferred(void);
int msm_watchdog_resume_deferred(void);
#else
static inline void msm_watchdog_bark(void) {}
static inline int msm_watchdog_suspend_deferred(void) { return 0; }
static inline int msm_watchdog_resume_deferred(void) { return 0; }
#endif

#endif
