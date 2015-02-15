/*
 * Linux 2.6.32 and later Kernel module for VMware MVP Hypervisor Support
 *
 * Copyright (C) 2010-2013 VMware, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; see the file COPYING.  If not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#line 5

/**
 * @file
 *
 * @brief Common guest/host balloon state machine.
 */
#ifndef _MVP_BALLOON_H
#define _MVP_BALLOON_H

#define INCLUDE_ALLOW_VMX
#define INCLUDE_ALLOW_PV
#define INCLUDE_ALLOW_GPL
#define INCLUDE_ALLOW_GUESTUSER
#define INCLUDE_ALLOW_MODULE
#include "include_check.h"

/**
 * @brief Balloon watchdog timeout (in seconds).
 *
 * If we don't hear back from the guest balloon driver in this amount of time,
 * we terminate the guest.
 *
 * This can sound arbitrary long but we need to deal with checkpointing.  The
 * watchdog goal is only to not let not-responding VM running for ages.
 */
#define BALLOON_WATCHDOG_TIMEOUT_SECS 90

/**
 * @brief Threshold to distinguish an oom_score_adj from an oom_adj value.
 */
#define USE_OOM_SCORE_ADJ_THRESHOLD 50

/**
 * @brief MVP_BALLOON_GET_DELTA return.
 */
typedef union {
	struct {
		/**< Number/direction balloon adjustment in pages. */
		int32 delta:21;
	};
	uint32 u;
} Balloon_GetDeltaRet;

/**
 * @name Guest settings for lowmemorykiller oom_adj and minfree thresholds, as reflected in
 *       the guest's /sys/module/lowmemorykiller/parameters/{minfree,adj}.
 *
 * @{
 */

/**
 * @brief Android low memory killer thresholds (in pages).
 */
typedef enum {
	BALLOON_ANDROID_GUEST_MIN_FREE_FOREGROUND_APP_PAGES = 1536,
	BALLOON_ANDROID_GUEST_MIN_FREE_VISIBLE_APP_PAGES = 2048,
	BALLOON_ANDROID_GUEST_MIN_FREE_SECONDARY_SERVER_PAGES = 4096,
	BALLOON_ANDROID_GUEST_MIN_FREE_BACKUP_APP_PAGES = 4096,
	BALLOON_ANDROID_GUEST_MIN_FREE_HOME_APP_PAGES = 4096,
	BALLOON_ANDROID_GUEST_MIN_FREE_HIDDEN_APP_PAGES = 5120,
	BALLOON_ANDROID_GUEST_MIN_FREE_CONTENT_PROVIDER_PAGES = 5632,
	BALLOON_ANDROID_GUEST_MIN_FREE_EMPTY_APP_MEM_PAGES = 6144
} Balloon_AndroidGuestMinFreePages;

/* @} */

/**
 * @brief Host Android levels for the various thresholds.
 *
 * From framework/base/services/java/com/android/server/am/ProcessList.java
 * mOomAdj[]
 */
typedef enum {
	HOST_FOREGROUND_APP = 0,
	HOST_VISIBLE_APP = 1,
	HOST_PERCEPTIBLE_APP = 2,
	HOST_BACKUP_APP = 3,
	HOST_HIDDEN_APP_MIN = 4,
	HOST_HIDDEN_APP_MAX = 5,
} Balloon_AndroidHostApp;

/**
 * @brief Calculate distance to the point at which Android will terminate
 *        processes.
 *
 * In the balloon policy we strive to maintain the low memory killer minfree
 * value (e.g. max(freePages, filePages)) above the threshold for terminating
 * empty apps (as per the Android low memory killer's logic). Here we measure
 * the number of pages we have buffering us from this point.
 *
 * We chose the empty app threshold instead instead of the home app threshold,
 * the threshold we ultimately want to avoid crossing for two reasons:
 * - We want to avoid any error introduced by the use of max(free, file) when
 *   between the two thresholds from interfering with the errorBackground term
 *   in the balloon policy. If we instead measure the distance to the home app
 *   threshold, we can get into the situation that even when both sides have
 *   balanced background pages and the same low memory distance, different
 *   free/file ratios in the two worlds introduces a further bias.
 * - It's helpful in avoiding extreme situations where the balloon won't be able
 *   to adapt quickly to leave a buffer. With empty app minfree as the target,
 *   when background pages drops to zero, and both worlds are below the empty
 *   app minfree target, the balloon will stop adjusting, leaving each world to
 *   fend for itself. At this point, the worlds have a maximum of 8192 pages
 *   (using the above logic) until they start killing services and foreground
 *   apps, which seems like a reasonable buffer to have in place. Another way of
 *   putting it is that at this point, we are unsure that rebalancing the
 *   balloon won't harm the side it balances against by eating into its buffer.
 *
 * We assume that normally filePages only decreases as a result of freePages
 * being close to zero, when vmscan reclaiming kicks in. Based on this,
 * there are two cases when computing the distance.
 *
 * - filePages >= emptyAppPages:
 *      freePages + filePages - emptyAppPages
 * - filePages < emptyAppPages:
 *      freePages - emptyAppPages
 *
 * @param freePages number of free pages.
 * @param filePages number of pages in the page cache.
 * @param emptyAppPages number of free/file pages at which the
 *                      lowmemorykiller will start killing empty apps.
 *
 * @return Low memory distance measure (in pages), negative if below threshold.
 */
static inline int32
Balloon_LowMemDistance(uint32 freePages,
		       uint32 filePages,
		       uint32 emptyAppPages)
{
	return filePages >= emptyAppPages ?
		(int32) (freePages + filePages) - (int32) emptyAppPages :
		(int32) freePages - (int32) emptyAppPages;
}

#ifdef __KERNEL__
/**
 * @brief Obtain approximation of # anonymous pages belonging to Android
 *        background processes.
 *
 * Used to inform balloon policy. Note that this is a coarse approximation only,
 * since we use RSS. More precise accounting is possible but potentially costly
 * as it's not available directly in the task struct.
 *
 * @param minHiddenAppOOMScoreAdj minimum oom_score_adj for hidden apps.
 *
 * @return sum of empty, content provider and hidden app anon resident pages.
 *
 * @note On Linux < 3.3.0, Android LMK was using oom_adj values, which are much
 *       lower than oom_score_adj. If we detect a value higher than the
 *       threshold below, we use the latter instead of the former.
 */
static uint32
Balloon_AndroidBackgroundPages(uint32 minHiddenAppOOMScoreAdj)
{
	uint32 backgroundPages = 0, nonBackgroundPages = 0;
	struct task_struct *t;

	/*
	 * Traverse the tasklist to replicate the behavior of the Android
	 * low memory killer.
	 */
	rcu_read_lock();

	for_each_process(t) {
		int oom_score_adj = 0;

		task_lock(t);

		if (t->signal == NULL) {
			task_unlock(t);
			continue;
		} else {
			/*
			 * Use 50 as a threshold. 15 is the highest value we've
			 * observed on Android phones, and new phones will all
			 * use the oom_score_adj and thus have much bigger
			 * numbers (529 for example).
			 */
			oom_score_adj =
			  minHiddenAppOOMScoreAdj > USE_OOM_SCORE_ADJ_THRESHOLD
				?  t->signal->oom_score_adj
				: t->signal->oom_adj;
		}

		if (t->mm != NULL) {
#ifdef BALLOON_DEBUG_PRINT_ANDROID_PAGES
			pr_info("Balloon_AndroidBackgroundPages: %d %d %s\n",
				oom_score_adj,
				(int)get_mm_counter(t->mm, MM_ANONPAGES),
				t->comm);
#endif

			if (oom_score_adj >= (int)minHiddenAppOOMScoreAdj)
				/*
				 * Unlike the Android low memory killer, we
				 * only consider anonymous memory here, since
				 * we already account for file pages in the
				 * balloon policy using
				 * global_page_state(NR_FILE_PAGES).
				 */
				backgroundPages +=
					get_mm_counter(t->mm, MM_ANONPAGES);
			else
				nonBackgroundPages +=
					get_mm_counter(t->mm, MM_ANONPAGES);
		}

		task_unlock(t);
	}

	rcu_read_unlock();

#ifdef BALLOON_DEBUG_PRINT_ANDROID_PAGES
	pr_info("Balloon_AndroidBackgroundPages: non-background pages: %d " \
		"background pages: %d\n",
		nonBackgroundPages,
		backgroundPages);
#endif

	return backgroundPages;
}
#endif

#endif
