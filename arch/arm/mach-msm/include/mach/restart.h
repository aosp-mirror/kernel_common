/* Copyright (c) 2011, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _ASM_ARCH_MSM_RESTART_H_
#define _ASM_ARCH_MSM_RESTART_H_

#define RESTART_NORMAL 0x0
#define RESTART_DLOAD  0x1

#if defined(CONFIG_MSM_NATIVE_RESTART)
/* if arch_reset is called from userspace,
   restart mode will be set to 'h' equal to 104.
   As a result, we need MAX to know the mode is valid. */
enum RESTART_MODE {
	/* for legacy cmd restart */
	RESTART_MODE_LEGACY = 0,

	/* all other restart rised by kernel.
	   these modes will all enter ramdump. */
	RESTART_MODE_Q6_WATCHDOG_BITE,

	RESTART_MODE_MODEM_CRASH,
	RESTART_MODE_MODEM_USER_INVOKED,
	RESTART_MODE_MODEM_UNWEDGE_TIMEOUT,
	RESTART_MODE_MODEM_WATCHDOG_BITE,
	RESTART_MODE_MODEM_ERROR_FATAL,

	RESTART_MODE_MDM_DOG_BITE,
	RESTART_MODE_MDM_FATAL,

	RESTART_MODE_APP_WATCHDOG_BARK,
	RESTART_MODE_ERASE_EFS,
	/* This is pseudo enum to indicate the maximum,
	   add new restart mode before this one. */
	RESTART_MODE_MAX
};
void msm_set_restart_mode(int mode);
void msm_restart(char mode, const char *cmd);
#elif defined(CONFIG_ARCH_FSM9XXX)
void fsm_restart(char mode, const char *cmd);
#else
#define msm_set_restart_mode(mode)
#endif

extern int pmic_reset_irq;
extern char *hashed_command_line;

#endif

