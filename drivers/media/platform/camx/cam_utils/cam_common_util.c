// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
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

#include <linux/string.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/completion.h>
#include <linux/module.h>
#include <linux/iopoll.h>
#include <linux/moduleparam.h>
#include "cam_common_util.h"
#include "cam_debug_util.h"
#include "cam_hw.h"

int cam_common_util_get_string_index(const char **strings,
	uint32_t num_strings, char *matching_string, uint32_t *index)
{
	int i;

	for (i = 0; i < num_strings; i++) {
		if (strnstr(strings[i], matching_string, strlen(strings[i]))) {
			CAM_DBG(CAM_UTIL, "matched %s : %d\n",
				matching_string, i);
			*index = i;
			return 0;
		}
	}

	return -EINVAL;
}

uint32_t cam_common_util_remove_duplicate_arr(int32_t *arr, uint32_t num)
{
	int i, j;
	uint32_t wr_idx = 1;

	if (!arr) {
		CAM_ERR(CAM_UTIL, "Null input array");
		return 0;
	}

	for (i = 1; i < num; i++) {
		for (j = 0; j < wr_idx ; j++) {
			if (arr[i] == arr[j])
				break;
		}
		if (j == wr_idx)
			arr[wr_idx++] = arr[i];
	}

	return wr_idx;
}

ktime_t cam_common_util_get_curr_timestamp(void)
{
	return ktime_get_boottime();
}

void *cam_common_mem_kdup(void *from, size_t len)
{
	void *to = kvzalloc(len, GFP_KERNEL);

	if (!to) {
		CAM_ERR(CAM_UTIL, "Failed to allocate header memory");
		return to;
	}

	CAM_DBG(CAM_UTIL, "Allocate and copy header with size: %zu", len);
	memcpy(to, from, len);

	return to;
}
EXPORT_SYMBOL(cam_common_mem_kdup);

void cam_common_mem_free(void *memory)
{
	kvfree(memory);
}
EXPORT_SYMBOL(cam_common_mem_free);
