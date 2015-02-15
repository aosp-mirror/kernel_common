/**
 * debug.h - DesignWare USB3 DRD Controller Debug Header
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *	    Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2, as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "core.h"

#ifdef CONFIG_DEBUG_FS
extern void dbg_event(u8, const char*, int);
extern void dbg_print(u8, const char*, int, const char*);
extern void dbg_done(u8, const u32, int);
extern void dbg_queue(u8, const struct usb_request*, int);
extern void dbg_setup(u8, const struct usb_ctrlrequest*);
extern int dwc3_debugfs_init(struct dwc3 *);
extern void dwc3_debugfs_exit(struct dwc3 *);
extern void dbg_print_reg(const char *name, int reg);
#else
static inline void dbg_event(u8 ep_num, const char *name, int status)
{  }
static inline void dbg_print(u8 ep_num, const char *name, int status,
			     const char *extra)
{  }
static inline void dbg_done(u8 ep_num, const u32 count, int status)
{  }
static inline void dbg_queue(u8 ep_num, const struct usb_request *req,
			     int status)
{  }
static inline void dbg_setup(u8 ep_num, const struct usb_ctrlrequest *req)
{  }
static inline void dbg_print_reg(const char *name, int reg)
{  }
static inline int dwc3_debugfs_init(struct dwc3 *d)
{  return 0;  }
static inline void dwc3_debugfs_exit(struct dwc3 *d)
{  }
#endif

