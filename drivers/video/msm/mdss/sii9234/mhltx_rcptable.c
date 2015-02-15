/***********************************************************************************/
/*  Copyright (c) 2002-2010, Silicon Image, Inc.  All rights reserved.             */
/*  No part of this work may be reproduced, modified, distributed, transmitted,    */
/*  transcribed, or translated into any language or computer format, in any form   */
/*  or by any means without written permission of: Silicon Image, Inc.,            */
/*  1060 East Arques Avenue, Sunnyvale, California 94085                           */
/***********************************************************************************/

#include "TypeDefs.h"
#include "mhl_defs.h"
#define	MHL_MAX_RCP_KEY_CODE	(0x7F + 1)

uint8_t		rcpSupportTable [MHL_MAX_RCP_KEY_CODE] = {
	(MHL_DEV_LD_GUI),
	(MHL_DEV_LD_GUI),
	(MHL_DEV_LD_GUI),
	(MHL_DEV_LD_GUI),
	(MHL_DEV_LD_GUI),
	0, 0, 0, 0,
	(MHL_DEV_LD_GUI),
	0, 0, 0,
	(MHL_DEV_LD_GUI),
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	0,
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	0, 0, 0,
	(MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_AUDIO),
	0,
	0,
	0,
	0,
	0,
	0, 0, 0, 0, 0, 0, 0,
	0,

	(MHL_DEV_LD_SPEAKER),
	(MHL_DEV_LD_SPEAKER),
	(MHL_DEV_LD_SPEAKER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_RECORD),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_RECORD),
	(MHL_DEV_LD_RECORD),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),
	(MHL_DEV_LD_MEDIA),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA),
	0, 0, 0,
	0,
	0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),
	(MHL_DEV_LD_RECORD),
	(MHL_DEV_LD_RECORD),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_RECORD),

	(MHL_DEV_LD_SPEAKER),
	(MHL_DEV_LD_SPEAKER),
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
