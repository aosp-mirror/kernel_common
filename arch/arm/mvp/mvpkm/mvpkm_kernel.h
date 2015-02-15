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

#ifndef _MVPKM_KERNEL_H
#define _MVPKM_KERNEL_H

#define INCLUDE_ALLOW_MODULE
#define INCLUDE_ALLOW_GPL
#include "include_check.h"

#include <linux/rwsem.h>
#include <linux/kobject.h>
#include <linux/rbtree.h>

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif

#include "atomic.h"
#include "montimer_kernel.h"
#include "worldswitch.h"

/**
 * @file
 *
 * @brief The kernel level driver.
 */

struct MvpkmVM {
	struct kobject      kobj;        /**< used to hook into sysfs */
	struct kset        *devicesKSet; /**< kset for virtual device entries */
	struct kset        *miscKSet;    /**< kset for miscellaneous entries */
	_Bool               haveKObj;    /**< used for proper release */
	struct rb_root      lockedRoot;  /**< locked page RB tree root */
	struct rw_semaphore lockedSem;   /**< linked list rw semaphore */
	AtmUInt32           usedPages;   /**< nmb of MEMREGION_MAINMEM pages */
	_Bool               isMonitorInited; /**< Was SetupMonitor called? */
	WorldSwitchPage    *wsp;             /**< worldswitch page */
	wait_queue_head_t   wfiWaitQ;        /**< guest VCPU is WFI-ing */

	/**
	 * prevents entries the WFI wait Q from disappearing
	 * underneath us in MvpkmShrink.
	 */
	struct rw_semaphore wspSem;

	/** monitor timers, there should be one of these per VCPU */
	struct MonTimer     monTimer;

	/** stub page to be used for unmappable pages */
	MPN                 stubPageMPN;

	struct vm_struct   *wspHkvaArea;  /**< VM area struct for wspHkvaArea */
	HKVA                wspHKVADummyPage; /**< Dummy page for wspHkvaArea */
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock    wakeLock;        /**< guest running wake lock */
#endif

	/** prevents monThreadTask from disappearing underneath us */
	struct rw_semaphore monThreadTaskSem;

	struct task_struct *monThreadTask;
	struct timer_list   balloonWDTimer;  /** Balloon watchdog timer */
	_Bool               balloonWDEnabled;/** Balloon watchdog enabled? */
	_Bool               watchdogTriggered;/**< Did watchdog hit timeout? */
};

void Mvpkm_WakeGuest(struct MvpkmVM *vm, int why);
struct kset *Mvpkm_FindVMNamedKSet(int vmID, const char *name);

extern struct cpumask inMonitor;

#endif
