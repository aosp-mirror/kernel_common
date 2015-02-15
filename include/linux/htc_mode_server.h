/*
 * HTC mode Server Header
 *
 * Copyright (C) 2011 HTC Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _HTC_MODE_SERVER_H_
#define _HTC_MODE_SERVER_H_

#include <linux/completion.h>

#define HSML_VERSION_12             1

#define HSML_PROTOCOL_VERSION		0x0006
#if HSML_VERSION_12
#define HSML_12_PROTOCOL_VERSION	0x0100
#endif
#define HSML_07_PROTOCOL_VERSION	0x0008

#define HTC_MODE_CONTROL_REQ		0x12

#define CLIENT_INFO_SERVER_ROTATE_USED		(1 << 1)

#define CTRL_CONF_TOUCH_EVENT_SUPPORTED		(1 << 0)
#define CTRL_CONF_NUM_SIMULTANEOUS_TOUCH	(4 << 1)


#define HSML_SERVER_NONCE_SIZE 		20
#define HSML_CLIENT_SIG_SIZE		168
#define HSML_SERVER_SIG_SIZE		148

#if HSML_VERSION_12
enum {
    cHSML_VER_08 = 0,
    cHSML_VER_12,
};
#endif

enum {
	CLIENT_INFO_MESGID = 0,
	SERVER_INFO_MESGID,
	TOUCH_EVENT_MESGID,
	AUTH_SERVER_NONCE_MESGID,
	AUTH_CLIENT_NONCE_MESGID,
	AUTH_RESPONSE_MESGID,
	OBU_INFO_MESGID
};

enum {
	HSML_FB_HEADER_ID = 0,
	HSML_TOUCH_EVENT_ID,
	HSML_KEY_EVENT_ID
};

enum {
	FB_HEADER_MSGID = 0,

};

enum {
	HSML_06_REQ_GET_SERVER_VERSION = 0x40,
	HSML_06_REQ_SEND_CLIENT_INFO,
	HSML_06_REQ_GET_SERVER_INFO,
	HSML_06_REQ_GET_FB,
	HSML_06_REQ_STOP,
	HSML_06_REQ_SEND_EXTENDED_CLIENT_INFO,
	HSML_06_REQ_GET_SERVER_NONCE,
	HSML_06_REQ_SET_CLIENT_AUTH,
	HSML_06_REQ_GET_SERVER_AUTH,
	HSML_06_REQ_SET_MAX_CHARGING_CURRENT,

	HSML_06_REQ_COUNT
};

struct msm_client_info {
	u8 mesg_id;
	u16 width;
	u16 height;
	u32 display_conf;
	u32 pixel_format;
	u32 ctrl_conf;
} __attribute__ ((__packed__));

struct msm_server_info {
	u8 mesg_id;
	u16 width;
	u16 height;
	u32 pixel_format;
	u32 ctrl_conf;
} __attribute__ ((__packed__));


struct htcmode_protocol {
	u16 version;
	u16 vendor;
	u8 request;
	struct msm_client_info client_info;
	struct msm_server_info server_info;
	char nonce[HSML_SERVER_NONCE_SIZE];
	u8 client_sig[HSML_CLIENT_SIG_SIZE];
	u8 server_sig[HSML_SERVER_SIG_SIZE];
	u8 notify_authenticator;
	u8 auth_in_progress;
	u8 auth_result;
	u8 debug_mode;
};

struct hsml_header {
	u8 msg_id;
	u16 x;
	u16 y;
	u16 w;
	u16 h;
	u8 signature[8];
	u8 reserved[494];
	u8 checksum;
} __attribute__ ((__packed__));

struct touch_content {
	u16 x;
	u16 y;
	u8 event_id;
	u8 pressure;
} __attribute__ ((__packed__));

struct touch_event {
	u8 mesg_id;
    u8 num_touch;
} __attribute__ ((__packed__));

struct key_event {
	u8 mesg_id;
    u8 down;
    u32 code;
} __attribute__ ((__packed__));


enum {
	HSML_08_REQ_GET_SERVER_VERSION = 0x40,
	HSML_08_REQ_NUM_COMPRESSION_SETTINGS,
	HSML_08_REQ_GET_SERVER_CONFIGURATION,
	HSML_08_REQ_SET_SERVER_CONFIGURATION,
	HSML_08_REQ_GET_FB,
	HSML_08_REQ_STOP,
	HSML_08_REQ_GET_SERVER_DISPLAY,
	HSML_08_REQ_SET_SERVER_DISPLAY,
	HSML_08_REQ_SET_MAX_FRAME_RATE,

	HSML_08_REQ_COUNT,
	HSML_08_REQ_MIRROR_LINK = 0xf0
};

enum {
	HSML_MSG_TOUCH	= 1,
};

enum {
	HSML_SERVER_CAP_H264 = 0,
	HSML_SERVER_CAP_HDCP,
	HSML_SERVER_CAP_TOUCH
};

enum {
	PIXEL_FORMAT_ARGB888 = 0,
	PIXEL_FORMAT_RGB565,
	PIXEL_FORMAT_RGB555,
	PIXEL_FORMAT_RGB444,
	PIXEL_FORMAT_RGB343,
	PIXEL_FORMAT_16_GRAY,
	PIXEL_FORMAT_8_GRAY
};

struct get_server_configuation {
	u32 dwCapabilities;
	u32 dwTouchConfiguration;
} __attribute__ ((__packed__));

struct set_server_configuation {
	u32 dwCapabilities;
	u32 dwTouchConfiguration;
	u8  bProfile;
	u8  bLevel;
} __attribute__ ((__packed__));

struct get_display_capabilities {
	u32 dwResolutionSupported;
	u32 dwPixelFormatSupported;
} __attribute__ ((__packed__));

static u32 display_setting[32][2] = {
	{640,360},{640,480},{720,480},{720,576},
	{800,480},{800,600},{848,480},{854,480},
	{864,480},{960,540},{1024,600},{1024,768},
	{1152,864},{1280,720},{1280,768},{1280,800},
	{1280,1024},{1360,768},{1400,900},{1400,900},
	{1400,1050},{1600,900},{1600,1200},{1680,1024},
	{1680,1050},{1920,1080},{1920,1200},{0,0},
	{0,0},{0,0},{0,0},{0,0},
};

struct set_display_info {
	u16 wWidth;
	u16 wHeight;
	u8 bPixelFormat;
	u16 wMaxGrayLevel;
} __attribute__ ((__packed__));

#if HSML_VERSION_12
enum {
    HSML_12_REQ_GET_VERSION = 0x40,
    HSML_12_REQ_GET_PARAMETERS,
    HSML_12_REQ_SET_PARAMETERS,
    HSML_12_REQ_START_FB_TRANS,
    HSML_12_REQ_PAUSE_FB_TRANS,
    HSML_12_REQ_STOP_FB_TRANS,
    HSML_12_REQ_SET_MAX_FRAME_RATE,
    HSML_12_REQ_GET_ID,

    HSML_12_REQ_COUNT,
};

enum {
    HSML_12_PIXEL_FORMAT_ARGB888 = 0,
    HSML_12_PIXEL_FORMAT_RGB888,
    HSML_12_PIXEL_FORMAT_RGB565,
    HSML_12_PIXEL_FORMAT_RGB555,
    HSML_12_PIXEL_FORMAT_RGB444,
    HSML_12_PIXEL_FORMAT_RGB343,
};

enum {
    HSML_12_CAP_ENDIAN = 0,
    HSML_12_CAP_FB_UPDATE,
};

struct get_parameters {
	u32 capabilities;
	u16 width;
	u16 height;
	u32 pixelFormatSupported;
	u32 encodingSupported;
} __attribute__ ((__packed__));

struct set_parameters {
	u32 capabilities;
	u8 pixelFormat;
	u8 paddings[3];
	u32 encodingSupported;
} __attribute__ ((__packed__));
#endif

struct hsml_protocol {
	u16 version;
	u16 vendor;
	u8 request;
	u32 MaxFPS;
#if HSML_VERSION_12
	struct get_parameters get_parameters_info;
	struct set_parameters set_parameters_info;
#endif
	struct get_server_configuation get_server_configuation_info;
	struct set_server_configuation set_server_configuation_info;
	struct get_display_capabilities get_display_capabilities_info;
	struct set_display_info set_display_info;
	u8 debug_mode;
};

#endif 
