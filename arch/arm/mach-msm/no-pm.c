/*
 * Copyright (c) 2010-2011, 2013 The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <asm/proc-fns.h>
#include <mach/cpuidle.h>
#include "idle.h"
#include "pm.h"

void arch_idle(void)
{
	cpu_do_idle();
}

void msm_pm_set_platform_data(struct msm_pm_platform_data *data, int count)
{ }

void msm_pm_cpu_enter_lowpower(unsigned cpu)
{
	asm("wfi"
		:
		:
		: "memory", "cc");
}

void msm_pm_set_max_sleep_time(int64_t max_sleep_time_ns) { }

void msm_pm_set_irq_extns(struct msm_pm_irq_calls *irq_calls) {}

enum msm_pm_sleep_mode msm_pm_idle_enter(struct cpuidle_device *dev,
			struct cpuidle_driver *drv, int index)
{
	return -ENOSYS;
}

void msm_pm_enable_retention(bool enable) {}

