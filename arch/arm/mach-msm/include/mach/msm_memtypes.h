/* Copyright (c) 2010-2011, 2013 The Linux Foundation. All rights reserved.
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

/* The MSM Hardware supports multiple flavors of physical memory.
 * This file captures hardware specific information of these types.
*/

#ifndef __ASM_ARCH_MSM_MEMTYPES_H
#define __ASM_ARCH_MSM_MEMTYPES_H

#include <mach/memory.h>
#include <linux/init.h>

int __init meminfo_init(unsigned int, unsigned int);
/* Redundant check to prevent this from being included outside of 7x30 */
#if defined(CONFIG_ARCH_MSM7X30)
unsigned int get_num_populated_chipselects(void);
#endif

unsigned int get_num_memory_banks(void);
unsigned int get_memory_bank_size(unsigned int);
unsigned int get_memory_bank_start(unsigned int);

enum {
	MEMTYPE_NONE = -1,
	MEMTYPE_SMI_KERNEL = 0,
	MEMTYPE_SMI,
	MEMTYPE_EBI0,
	MEMTYPE_EBI1,
	MEMTYPE_MAX,
};

enum {
	SYS_MEMORY = 1,        /* system memory*/
	BOOT_REGION_MEMORY1,   /* boot loader memory 1*/
	BOOT_REGION_MEMORY2,   /* boot loader memory 2,reserved*/
	APPSBL_MEMORY,         /* apps boot loader memory*/
	APPS_MEMORY,           /* apps  usage memory*/
};


void msm_reserve(void);

#define MEMTYPE_FLAGS_FIXED	0x1
#define MEMTYPE_FLAGS_1M_ALIGN	0x2

struct memtype_reserve {
	phys_addr_t start;
	phys_addr_t size;
	phys_addr_t limit;
	int flags;
};

struct reserve_info {
	struct memtype_reserve *memtype_reserve_table;
	void (*calculate_reserve_sizes)(void);
	void (*reserve_fixed_area)(unsigned long);
	int (*paddr_to_memtype)(phys_addr_t);
	unsigned long low_unstable_address;
	unsigned long max_unstable_size;
	unsigned long bank_size;
	unsigned long fixed_area_start;
	unsigned long fixed_area_size;
};

extern struct reserve_info *reserve_info;

int __init dt_scan_for_memory_reserve(unsigned long node, const char *uname,
					int depth, void *data);
int __init dt_scan_for_memory_hole(unsigned long node, const char *uname,
					int depth, void *data);
void adjust_meminfo(unsigned long start, unsigned long size);
#endif
