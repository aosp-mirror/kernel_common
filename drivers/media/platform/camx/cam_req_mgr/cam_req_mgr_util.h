/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2016-2019, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _CAM_REQ_MGR_UTIL_API_H_
#define _CAM_REQ_MGR_UTIL_API_H_

#include <media/cam_req_mgr.h>
#include "cam_req_mgr_util_priv.h"

/**
 * state of a handle(session/device)
 * @HDL_FREE: free handle
 * @HDL_ACTIVE: active handles
 */
enum hdl_state {
	HDL_FREE,
	HDL_ACTIVE
};

/**
 * handle type
 * @HDL_TYPE_DEV_CTX: for device context
 * @HDL_TYPE_DEV_BRIDGE: for device bridge
 * @HDL_TYPE_SESSION: for session
 * @HDL_TYPE_LINK: for link
 */
enum hdl_type {
	HDL_TYPE_UNDERFLOW,	/* first valid type is below this line */
	HDL_TYPE_DEV_CTX,
	HDL_TYPE_DEV_BRIDGE,
	HDL_TYPE_SESSION,
	HDL_TYPE_LINK,
	HDL_TYPE_OVERFLOW,	/* last valid type is above this line */
};

/**
 * struct handle
 * @session_hdl: session handle
 * @hdl_value: Allocated handle
 * @type: session/device handle
 * @state: free/used
 * @ops: ops structure
 * @priv: private data of a handle
 */
struct handle {
	int32_t session_hdl;
	uint32_t hdl_value;
	enum hdl_type type;
	enum hdl_state state;
	uint64_t dev_id;
	void *ops;
	void *priv;
};

/**
 * struct cam_req_mgr_util_hdl_tbl
 * @hdl: row of handles
 * @bitmap: bit map to get free hdl row idx
 * @bits: size of bit map in bits
 */
struct cam_req_mgr_util_hdl_tbl {
	struct handle hdl[CAM_REQ_MGR_MAX_HANDLES];
	void *bitmap;
	size_t bits;
};

/**
 * cam_req_mgr_util APIs for KMD drivers and cam_req_mgr
 * @session_hdl: session_hdl info
 * @v4l2_sub_dev_flag: flag to create v4l2 sub device
 * @media_entity_flag: flag for media entity
 * @reserved: reserved field
 * @ops: ops pointer for a device handle
 * @priv: private data for a device handle
 */
struct cam_create_dev_hdl {
	int32_t session_hdl;
	int32_t v4l2_sub_dev_flag;
	int32_t media_entity_flag;
	int32_t reserved;
	uint64_t dev_id;
	void *ops;
	void *priv;
};

/**
 * cam_create_session_hdl() - create a session handle
 * @priv: private data for a session handle
 *
 * cam_req_mgr core calls this function to get
 * a unique session handle. Returns a unique session
 * handle
 */
int32_t cam_create_session_hdl(void *priv);

/**
 * cam_create_device_ctx_hdl() - create a device context handle
 * @hdl_data: session hdl, flags, ops and priv dara as input
 */
int32_t cam_create_device_ctx_hdl(struct cam_create_dev_hdl *hdl_data);

/**
 * cam_create_device_bridge_hdl() - create a device bridge handle
 * @hdl_data: session hdl, flags, ops and priv dara as input
 */
int32_t cam_create_device_bridge_hdl(struct cam_create_dev_hdl *hdl_data);

/**
 * cam_create_link_hdl() - create a link handle
 * @hdl_data: session hdl, flags, ops and priv dara as input
 *
 * cam_req_mgr_core calls this function to get
 * session and link handles
 * KMD drivers calls this function to create
 * a link handle. Returns a unique link handle
 */
int32_t cam_create_link_hdl(struct cam_create_dev_hdl *hdl_data);

/**
 * cam_get_device_ctx() - get private data of a device context handle
 * @dev_hdl: handle for a device
 */
void *cam_get_device_ctx(int32_t dev_hdl);

/**
 * cam_get_device_bridge() - get private data of a device bridge handle
 * @dev_hdl: handle for a device
 */
void *cam_get_device_bridge(int32_t dev_hdl);

/**
 * cam_get_session_priv() - get private data of a session handle
 * @dev_hdl: handle for a session
 *
 * cam_req_mgr_core and KMD drivers use this function to
 * get private data of a handle. Returns a private data
 * structure pointer.
 */
struct cam_req_mgr_core_session *cam_get_session_priv(int32_t dev_hdl);

/**
 * cam_get_link_priv() - get private data of a link handle
 * @dev_hdl: handle for a link
 *
 * cam_req_mgr_core and KMD drivers use this function to
 * get private data of a handle. Returns a private data
 * structure pointer.
 */
struct cam_req_mgr_core_link *cam_get_link_priv(int32_t dev_hdl);

/**
 * cam_get_device_ops() - get ops of a handle
 * @dev_hdl: handle for a session/link/device
 *
 * cam_req_mgr_core and KMD drivers use this function to
 * get ops of a handle. Returns a pointer to ops.
 */
void *cam_get_device_ops(int32_t dev_hdl);

/**
 * cam_destroy_device__ctx_hdl() - destroy device context handle
 * @dev_hdl: handle for a device.
 */
int32_t cam_destroy_device_ctx_hdl(int32_t dev_hdl);

/**
 * cam_destroy_device__ctx_hdl() - destroy device bridge handle
 * @dev_hdl: handle for a device.
 */
int32_t cam_destroy_device_bridge_hdl(int32_t dev_hdl);

/**
 * cam_destroy_link_hdl() - destroy link handle
 * @dev_hdl: handle for a link.
 *
 * Returns success/failure
 */
int32_t cam_destroy_link_hdl(int32_t dev_hdl);

/**
 * cam_destroy_session_hdl() - destroy device handle
 * @dev_hdl: handle for a session
 *
 * Returns success/failure
 */
int32_t cam_destroy_session_hdl(int32_t dev_hdl);


/* Internal functions */
/**
 * cam_req_mgr_util_init() - init function of cam_req_mgr_util
 *
 * This is called as part of probe function to initialize
 * handle table, bitmap, locks
 */
int cam_req_mgr_util_init(void);

/**
 * cam_req_mgr_util_deinit() - deinit function of cam_req_mgr_util
 *
 * This function is called in case of probe failure
 */
int32_t cam_req_mgr_util_deinit(void);

/**
 * cam_req_mgr_util_free_hdls() - free handles in case of crash
 *
 * Called from cam_req_mgr_dev release function to make sure
 * all data structures are cleaned to avoid leaks
 *
 * cam_req_mgr core can call this function at the end of
 * camera to make sure all stale entries are printed and
 * cleaned
 */
int32_t cam_req_mgr_util_free_hdls(void);

#endif /* _CAM_REQ_MGR_UTIL_API_H_ */
