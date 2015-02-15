/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
  $
 */

#ifndef __SLAVEIRQ__
#define __SLAVEIRQ__

#ifdef __KERNEL__
#include <linux/i2c-dev.h>
#endif

#include "mpu.h"
#include "mpuirq.h"

#define SLAVEIRQ_ENABLE_DEBUG          (1)
#define SLAVEIRQ_GET_INTERRUPT_CNT     (2)
#define SLAVEIRQ_GET_IRQ_TIME          (3)
#define SLAVEIRQ_GET_LED_VALUE         (4)
#define SLAVEIRQ_SET_TIMEOUT           (5)
#define SLAVEIRQ_SET_SLAVE_INFO        (6)

#ifdef __KERNEL__

void slaveirq_exit(struct ext_slave_platform_data *pdata);
int slaveirq_init(struct i2c_adapter *slave_adapter,
#ifdef CONFIG_CIR_ALWAYS_READY
		  struct i2c_client  *client,
#endif
		struct ext_slave_platform_data *pdata,
		char *name);

#endif

#endif
