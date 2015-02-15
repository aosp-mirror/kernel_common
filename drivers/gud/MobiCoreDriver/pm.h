/*
 * Header file of MobiCore Driver Kernel Module.
 *
 * <-- Copyright Giesecke & Devrient GmbH 2009-2012 -->
 * <-- Copyright Trustonic Limited 2013 -->
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _MC_PM_H_
#define _MC_PM_H_

#include "main.h"

#define NO_SLEEP_REQ	0
#define REQ_TO_SLEEP	1

#define NORMAL_EXECUTION	0
#define READY_TO_SLEEP		1

/* How much time after resume the daemon should backoff */
#define DAEMON_BACKOFF_TIME	500

/* Initialize Power Management */
int mc_pm_initialize(struct mc_context *context);
/* Free all Power Management resources*/
int mc_pm_free(void);
/* Initialize secure crypto clocks */
int mc_pm_clock_initialize(void);
/* Free secure crypto clocks */
void mc_pm_clock_finalize(void);
/* Enable secure crypto clocks */
int mc_pm_clock_enable(void);
/* Disable secure crypto clocks */
void mc_pm_clock_disable(void);
/* Test if sleep is possible */
bool mc_pm_sleep_ready(void);

#endif /* _MC_PM_H_ */
