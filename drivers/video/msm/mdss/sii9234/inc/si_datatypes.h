/****************************************************************************
*!file     si_datatypes.h
*!brief    Silicon Image data type header (conforms to C99).
*
* No part of this work may be reproduced, modified, distributed,
* transmitted, transcribed, or translated into any language or computer
* format, in any form or by any means without written permission of
* Silicon Image, Inc., 1060 East Arques Avenue, Sunnyvale, California 94085
*
* Copyright 2008-2009, Silicon Image, Inc.  All rights reserved.
****************************************************************************/

#ifndef __SI_DATATYPES_H__
#define __SI_DATATYPES_H__

#define ROM     static
#define XDATA



#define MSG_ALWAYS              0x00
#define MSG_STAT                0x01
#define MSG_DBG                 0x02
#if 0
#define CI2CA_HIGH	0
#if machine_is_shooter()
#define cbus_slave_addr 0xCC
#else
#define CBUS_SLAVE_ADDR 0xC8
#endif
#else
#define	CBUS_SLAVE_ADDR	0xC8
#endif

#define SET_BITS    0xFF
#define CLEAR_BITS  0x00

#endif  /* __SI_DATATYPES_H__*/

