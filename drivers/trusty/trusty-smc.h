/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Google, Inc.
 */

#pragma once

#include <linux/types.h>

struct smc_ret8 {
	ulong r0;
	ulong r1;
	ulong r2;
	ulong r3;
	ulong r4;
	ulong r5;
	ulong r6;
	ulong r7;
};

struct smc_ret8 trusty_smc8(ulong r0, ulong r1, ulong r2, ulong r3,
			    ulong r4, ulong r5, ulong r6, ulong r7);
