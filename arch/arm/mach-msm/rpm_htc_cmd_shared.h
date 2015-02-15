/*===========================================================================

  Copyright (c) 2013 HTC.
  All Rights Reserved.
  HTC Proprietary and Confidential.

  ===========================================================================*/

#ifndef RPM_HTC_CMD_SHARED_H
#define RPM_HTC_CMD_SHARED_H

/* This header should be exact the same as kernel's copy.
   If you modify this header, COPY IT TO KERNEL!
*/

/* FIXME: MUST be the same as core/api/power/rpm.h enumeration.
 * For kernel, we cannot include rpm.h.
 * rpm.h cannot include this file since it is in private folder.
 */
#define RPM_HTC_CMD_REQ_TYPE 0x63637468 // 'htcc' in little endian

/*
 * This index indicates what command it runs actually.
 * NOTICE! NEVER INSERT NEW ITEM TO OLD ITEMS,
 * ALWAYS PUT NEW ITEM BEFORE RHCH_NUM!
 */
typedef enum
{
	RHCF_VDD_DIG_HOLD = 0,
	RHCF_RPM_FATAL,
	RHCF_NUM,
} RPM_HTC_CMD_FUNC_T;

/* Command RHCF_VDD_DIG_HOLD parameters*/
#define VDD_DIG_HOLD_PARA_ENABLE	0x62616e65 // 'enab' in little endian

#endif
