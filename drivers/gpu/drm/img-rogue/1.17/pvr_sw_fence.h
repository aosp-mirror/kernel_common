/*
 * @File
 * @Codingstyle LinuxKernel
 * @Copyright   Copyright (c) Imagination Technologies Ltd. All Rights Reserved
 * @License     Strictly Confidential.
 */

#if !defined(__PVR_SW_FENCES_H__)
#define __PVR_SW_FENCES_H__

#include "pvr_linux_fence.h"

struct pvr_sw_fence_context;

struct pvr_sw_fence_context *pvr_sw_fence_context_create(const char *name,
				const char *driver_name);
void pvr_sw_fence_context_destroy(struct pvr_sw_fence_context *fence_context);
struct dma_fence *pvr_sw_fence_create(struct pvr_sw_fence_context *
				      fence_context);

const char *pvr_sw_fence_context_name(struct pvr_sw_fence_context *fctx);
void pvr_sw_fence_context_value_str(struct pvr_sw_fence_context *fctx,
				    char *str, int size);

#endif /* !defined(__PVR_SW_FENCES_H__) */
