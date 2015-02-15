/*
 * Copyright (C) 2013 HTC, Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <mach/board.h>

#include <mach/rpm-smd.h>

#include "rpm_htc_cmd.h"

void htc_rpm_cmd_hold_vdd_dig(bool en)
{
	uint32_t value = en;
	struct msm_rpm_kvp kvp;

	kvp.key = VDD_DIG_HOLD_PARA_ENABLE;
	kvp.data = (void *)&value;
	kvp.length = sizeof(uint32_t);

	msm_rpm_send_message_noirq(MSM_RPM_CTX_ACTIVE_SET, RPM_HTC_CMD_REQ_TYPE, RHCF_VDD_DIG_HOLD, &kvp, 1);
}

