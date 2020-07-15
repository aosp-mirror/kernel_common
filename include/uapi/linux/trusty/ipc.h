/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */

#ifndef _UAPI_LINUX_TRUSTY_IPC_H_
#define _UAPI_LINUX_TRUSTY_IPC_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#define TIPC_IOC_MAGIC			'r'
#define TIPC_IOC_CONNECT		_IOW(TIPC_IOC_MAGIC, 0x80, char *)

#if defined(CONFIG_COMPAT)
#define TIPC_IOC_CONNECT_COMPAT		_IOW(TIPC_IOC_MAGIC, 0x80, \
					     compat_uptr_t)
#endif

#endif
