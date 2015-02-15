/* Copyright (c) 2008-2013, The Linux Foundation. All rights reserved.
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

#ifndef DIAGMEM_H
#define DIAGMEM_H
#include "diagchar.h"

extern mempool_t *diag_pools_array[NUM_MEMORY_POOLS];

void *diagmem_alloc(struct diagchar_dev *driver, int size, int pool_type);
void diagmem_free(struct diagchar_dev *driver, void *buf, int pool_type);
void diagmem_init(struct diagchar_dev *driver);
void diagmem_exit(struct diagchar_dev *driver, int pool_type);
#ifdef CONFIG_DIAGFWD_BRIDGE_CODE
void diagmem_hsic_init(int index);
#endif
#endif
