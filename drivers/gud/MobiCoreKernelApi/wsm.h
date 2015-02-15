/*
 * World shared memory definitions.
 *
 * <-- Copyright Giesecke & Devrient GmbH 2009 - 2012 -->
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _MC_KAPI_WSM_H_
#define _MC_KAPI_WSM_H_

#include "common.h"
#include <linux/list.h>

struct wsm {
	void			*virt_addr;
	uint32_t		len;
	uint32_t		handle;
	struct list_head	list;
};

#endif /* _MC_KAPI_WSM_H_ */
