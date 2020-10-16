// SPDX-License-Identifier: GPL-2.0
/*
 * debug_kinfo.h - backup kernel information for bootloader usage
 *
 * Copyright 2020 Google LLC
 */

#ifndef DEBUG_KINFO_H
#define DEBUG_KINFO_H

#include <linux/utsname.h>

#define BUILD_INFO_LEN		256
#define DEBUG_KINFO_MAGIC	0xCCEEDDFF

/*
 * Header structure must be byte-packed, since the table is provided to
 * bootloader.
 */
struct kernel_info {
	/* For kallsyms */
	u8 enabled_all;
	u8 enabled_base_relative;
	u8 enabled_absolute_percpu;
	u8 enabled_cfi_clang;
	u32 num_syms;
	u16 name_len;
	u16 bit_per_long;
	u16 module_name_len;
	u16 symbol_len;
	u64 _addresses_pa;
	u64 _relative_pa;
	u64 _stext_pa;
	u64 _etext_pa;
	u64 _sinittext_pa;
	u64 _einittext_pa;
	u64 _end_pa;
	u64 _offsets_pa;
	u64 _names_pa;
	u64 _token_table_pa;
	u64 _token_index_pa;
	u64 _markers_pa;

	/* For frame pointer */
	u32 thread_size;

	/* For virt_to_phys */
	u64 swapper_pg_dir_pa;

	/* For linux banner */
	u8 last_uts_release[__NEW_UTS_LEN];

	/* Info of running build */
	char build_info[BUILD_INFO_LEN];
} __packed;

struct kernel_all_info {
	u32 magic_number;
	u32 combined_checksum;
	struct kernel_info info;
} __packed;

#endif // DEBUG_KINFO_H
