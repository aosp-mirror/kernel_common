/* SPDX-License-Identifier: (GPL-2.0 WITH Linux-syscall-note) OR BSD-3-Clause */
/*
 * Definitions for virtio-pmem devices.
 *
 * Copyright (C) 2019 Red Hat, Inc.
 *
 * Author(s): Pankaj Gupta <pagupta@redhat.com>
 */

#ifndef _UAPI_LINUX_VIRTIO_PMEM_H
#define _UAPI_LINUX_VIRTIO_PMEM_H

#include <linux/types.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_config.h>

/* Feature bits */
#define VIRTIO_PMEM_F_DISCARD	63 /* CHROMIUM: DISCARD is supported */

struct virtio_pmem_config {
	__le64 start;
	__le64 size;
};

#define VIRTIO_PMEM_REQ_TYPE_FLUSH      0
#define VIRTIO_PMEM_REQ_TYPE_DISCARD    U32_MAX /* CHROMIUM: DISCARD command */

struct virtio_pmem_resp {
	/* Host return status corresponding to pmem request */
	__le32 ret;
};

/* Virtio pmem request with no range data */
struct virtio_pmem_req {
	/* command type */
	__le32 type;
};

/* Virtio pmem request on a memory range */
struct virtio_pmem_range_req {
	/* command type */
	__le32 type;
	__le32 unused;
	__le64 start;
	__le64 size;
} __attribute__((packed));

#endif
