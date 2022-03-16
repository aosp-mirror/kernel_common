/*
 * @File
 * @Codingstyle LinuxKernel
 * @Copyright   Copyright (c) Imagination Technologies Ltd. All Rights Reserved
 * @License     Strictly Confidential.
 */

#if !defined(__PVR_COUNTING_TIMELINE_H__)
#define __PVR_COUNTING_TIMELINE_H__

#include "pvr_linux_fence.h"

struct pvr_counting_fence_timeline;

void pvr_counting_fence_timeline_dump_timeline(
	void *data,
	DUMPDEBUG_PRINTF_FUNC *dump_debug_printf,
	void *dump_debug_file);

struct pvr_counting_fence_timeline *pvr_counting_fence_timeline_create(
	const char *name);
void pvr_counting_fence_timeline_put(
	struct pvr_counting_fence_timeline *fence_timeline);
struct pvr_counting_fence_timeline *pvr_counting_fence_timeline_get(
	struct pvr_counting_fence_timeline *fence_timeline);
struct dma_fence *pvr_counting_fence_create(
	struct pvr_counting_fence_timeline *fence_timeline, u64 *sync_pt_idx);
bool pvr_counting_fence_timeline_inc(
	struct pvr_counting_fence_timeline *fence_timeline, u64 *sync_pt_idx);
void pvr_counting_fence_timeline_force_complete(
	struct pvr_counting_fence_timeline *fence_timeline);

#endif /* !defined(__PVR_COUNTING_TIMELINE_H__) */
