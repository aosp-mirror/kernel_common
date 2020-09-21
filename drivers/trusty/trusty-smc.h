/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef _TRUSTY_SMC_H
#define _TRUSTY_SMC_H

#include <linux/types.h>

struct smc_ret8 {
	unsigned long r0;
	unsigned long r1;
	unsigned long r2;
	unsigned long r3;
	unsigned long r4;
	unsigned long r5;
	unsigned long r6;
	unsigned long r7;
};

struct smc_ret8 trusty_smc8(unsigned long r0, unsigned long r1,
			    unsigned long r2, unsigned long r3,
			    unsigned long r4, unsigned long r5,
			    unsigned long r6, unsigned long r7);

#endif /* _TRUSTY_SMC_H */
