/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020 Google, Inc.
 */

#pragma once

#define SMC_SC_TEST_VERSION SMC_STDCALL_NR(SMC_ENTITY_TEST, 0)
#define SMC_SC_TEST_SHARED_MEM_RW SMC_STDCALL_NR(SMC_ENTITY_TEST, 1)

#define TRUSTY_STDCALLTEST_API_VERSION 1
