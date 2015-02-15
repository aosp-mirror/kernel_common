/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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

#ifndef MSM_HDMI_HDCP_MGR_H
#define MSM_HDMI_HDCP_MGR_H

enum DS_TYPE {  /* type of downstream device */
	DS_UNKNOWN,
	DS_RECEIVER,
	DS_REPEATER,
};

enum {
	MSG_ID_IDX,
	RET_CODE_IDX,
	HEADER_LEN,
};

enum RET_CODE {
	HDCP_NOT_AUTHED,
	HDCP_AUTHED,
	HDCP_DISABLE,
};

enum MSG_ID { /* List of functions expected to be called after it */
	DOWN_CHECK_TOPOLOGY,
	UP_REQUEST_TOPOLOGY,
	UP_SEND_TOPOLOGY,
	DOWN_REQUEST_TOPOLOGY,
	MSG_NUM,
};

enum SOURCE_ID {
	HDCP_V1_TX,
	HDCP_V1_RX,
	HDCP_V2_RX,
	HDCP_V2_TX,
	SRC_NUM,
};

/*
 * how to parse sysfs params buffer
 * from hdcp_tx driver.
 */

struct HDCP_V2V1_MSG_TOPOLOGY {
	/* indicates downstream's type */
	uint32_t ds_type;
	uint8_t bksv[5];
	uint8_t dev_count;
	uint8_t depth;
	uint8_t ksv_list[5 * 127];
	uint32_t max_cascade_exceeded;
	uint32_t max_dev_exceeded;
};

#endif /* MSM_HDMI_HDCP_MGR_H */
