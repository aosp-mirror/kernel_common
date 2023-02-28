/* SPDX-License-Identifier: (GPL-2.0 WITH Linux-syscall-note) OR BSD-3-Clause */
/* This header is BSD licensed so anyone can use the definitions to implement
 * compatible drivers/servers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of IBM nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL IBM OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _LINUX_VIRTIO_PVCLOCK_H
#define _LINUX_VIRTIO_PVCLOCK_H

#include <linux/types.h>
#include <linux/virtio_types.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_config.h>

/* The feature bitmap for virtio pvclock */
/* TSC is stable */
#define VIRTIO_PVCLOCK_F_TSC_STABLE 0
/* Inject sleep for suspend */
#define VIRTIO_PVCLOCK_F_INJECT_SLEEP 1
/* Use device clocksource rating */
#define VIRTIO_PVCLOCK_F_CLOCKSOURCE_RATING 2

struct virtio_pvclock_config {
	/* Number of ns the VM has been suspended without guest suspension. */
	__u64 suspend_time_ns;
	/* Device-suggested rating of the pvclock clocksource. */
	__u32 clocksource_rating;
	__u32 padding;
};

/* Status values for a virtio_pvclock request. */
#define VIRTIO_PVCLOCK_S_OK 0
#define VIRTIO_PVCLOCK_S_IOERR 1
#define VIRTIO_PVCLOCK_S_UNSUPP 2

/*
 * Virtio pvclock set pvclock page request. Sets up the shared memory
 * pvclock_vsyscall_time_info struct.
 */
struct virtio_pvclock_set_pvclock_page_req {
	/* Physical address of pvclock_vsyscall_time_info. */
	__u64 pvclock_page_pa;
	/* Current system time. */
	__u64 system_time;
	/* Current tsc value. */
	__u64 tsc_timestamp;
	/* Status of this request, one of VIRTIO_PVCLOCK_S_*. */
	__u8 status;
	__u8 padding[7];
};

#endif /* _LINUX_VIRTIO_PVCLOCK_H */
