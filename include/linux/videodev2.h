/*
 *  Video for Linux Two header file
 *
 *  Copyright (C) 1999-2007 the contributors
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  Alternatively you can redistribute this file under the terms of the
 *  BSD license as stated below:
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *  3. The names of its contributors may not be used to endorse or promote
 *     products derived from this software without specific prior written
 *     permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *	Header file for v4l or V4L2 drivers and applications
 * with public API.
 * All kernel-specific stuff were moved to media/v4l2-dev.h, so
 * no #if __KERNEL tests are allowed here
 *
 *	See http://linuxtv.org for more info
 *
 *	Author: Bill Dirks <bill@thedirks.org>
 *		Justin Schoeman
 *              Hans Verkuil <hverkuil@xs4all.nl>
 *		et al.
 */
#ifndef __LINUX_VIDEODEV2_H
#define __LINUX_VIDEODEV2_H

#ifdef __KERNEL__
#include <linux/time.h>     
#else
#include <sys/time.h>
#endif
#include <linux/compiler.h>
#include <linux/ioctl.h>
#include <linux/types.h>

#define VIDEO_MAX_FRAME               32
#define VIDEO_MAX_PLANES               8

#ifndef __KERNEL__


#define VID_TYPE_CAPTURE	1	
#define VID_TYPE_TUNER		2	
#define VID_TYPE_TELETEXT	4	
#define VID_TYPE_OVERLAY	8	
#define VID_TYPE_CHROMAKEY	16	
#define VID_TYPE_CLIPPING	32	
#define VID_TYPE_FRAMERAM	64	
#define VID_TYPE_SCALES		128	
#define VID_TYPE_MONOCHROME	256	
#define VID_TYPE_SUBCAPTURE	512	
#define VID_TYPE_MPEG_DECODER	1024	
#define VID_TYPE_MPEG_ENCODER	2048	
#define VID_TYPE_MJPEG_DECODER	4096	
#define VID_TYPE_MJPEG_ENCODER	8192	
#endif


#define v4l2_fourcc(a, b, c, d)\
	((__u32)(a) | ((__u32)(b) << 8) | ((__u32)(c) << 16) | ((__u32)(d) << 24))

enum v4l2_field {
	V4L2_FIELD_ANY           = 0, 
	V4L2_FIELD_NONE          = 1, 
	V4L2_FIELD_TOP           = 2, 
	V4L2_FIELD_BOTTOM        = 3, 
	V4L2_FIELD_INTERLACED    = 4, 
	V4L2_FIELD_SEQ_TB        = 5, 
	V4L2_FIELD_SEQ_BT        = 6, 
	V4L2_FIELD_ALTERNATE     = 7, 
	V4L2_FIELD_INTERLACED_TB = 8, 
	V4L2_FIELD_INTERLACED_BT = 9, 
};
#define V4L2_FIELD_HAS_TOP(field)	\
	((field) == V4L2_FIELD_TOP 	||\
	 (field) == V4L2_FIELD_INTERLACED ||\
	 (field) == V4L2_FIELD_INTERLACED_TB ||\
	 (field) == V4L2_FIELD_INTERLACED_BT ||\
	 (field) == V4L2_FIELD_SEQ_TB	||\
	 (field) == V4L2_FIELD_SEQ_BT)
#define V4L2_FIELD_HAS_BOTTOM(field)	\
	((field) == V4L2_FIELD_BOTTOM 	||\
	 (field) == V4L2_FIELD_INTERLACED ||\
	 (field) == V4L2_FIELD_INTERLACED_TB ||\
	 (field) == V4L2_FIELD_INTERLACED_BT ||\
	 (field) == V4L2_FIELD_SEQ_TB	||\
	 (field) == V4L2_FIELD_SEQ_BT)
#define V4L2_FIELD_HAS_BOTH(field)	\
	((field) == V4L2_FIELD_INTERLACED ||\
	 (field) == V4L2_FIELD_INTERLACED_TB ||\
	 (field) == V4L2_FIELD_INTERLACED_BT ||\
	 (field) == V4L2_FIELD_SEQ_TB ||\
	 (field) == V4L2_FIELD_SEQ_BT)

enum v4l2_buf_type {
	V4L2_BUF_TYPE_VIDEO_CAPTURE        = 1,
	V4L2_BUF_TYPE_VIDEO_OUTPUT         = 2,
	V4L2_BUF_TYPE_VIDEO_OVERLAY        = 3,
	V4L2_BUF_TYPE_VBI_CAPTURE          = 4,
	V4L2_BUF_TYPE_VBI_OUTPUT           = 5,
	V4L2_BUF_TYPE_SLICED_VBI_CAPTURE   = 6,
	V4L2_BUF_TYPE_SLICED_VBI_OUTPUT    = 7,
#if 1
	
	V4L2_BUF_TYPE_VIDEO_OUTPUT_OVERLAY = 8,
#endif
	V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE = 9,
	V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE  = 10,
	V4L2_BUF_TYPE_PRIVATE              = 0x80,
};

#define V4L2_TYPE_IS_MULTIPLANAR(type)			\
	((type) == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE	\
	 || (type) == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)

#define V4L2_TYPE_IS_OUTPUT(type)				\
	((type) == V4L2_BUF_TYPE_VIDEO_OUTPUT			\
	 || (type) == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE		\
	 || (type) == V4L2_BUF_TYPE_VIDEO_OVERLAY		\
	 || (type) == V4L2_BUF_TYPE_VIDEO_OUTPUT_OVERLAY	\
	 || (type) == V4L2_BUF_TYPE_VBI_OUTPUT			\
	 || (type) == V4L2_BUF_TYPE_SLICED_VBI_OUTPUT)

enum v4l2_tuner_type {
	V4L2_TUNER_RADIO	     = 1,
	V4L2_TUNER_ANALOG_TV	     = 2,
	V4L2_TUNER_DIGITAL_TV	     = 3,
};

enum v4l2_memory {
	V4L2_MEMORY_MMAP             = 1,
	V4L2_MEMORY_USERPTR          = 2,
	V4L2_MEMORY_OVERLAY          = 3,
};

enum v4l2_colorspace {
	
	V4L2_COLORSPACE_SMPTE170M     = 1,

	
	V4L2_COLORSPACE_SMPTE240M     = 2,

	
	V4L2_COLORSPACE_REC709        = 3,

	
	V4L2_COLORSPACE_BT878         = 4,

	
	V4L2_COLORSPACE_470_SYSTEM_M  = 5,
	V4L2_COLORSPACE_470_SYSTEM_BG = 6,

	V4L2_COLORSPACE_JPEG          = 7,

	
	V4L2_COLORSPACE_SRGB          = 8,
};

enum v4l2_priority {
	V4L2_PRIORITY_UNSET       = 0,  
	V4L2_PRIORITY_BACKGROUND  = 1,
	V4L2_PRIORITY_INTERACTIVE = 2,
	V4L2_PRIORITY_RECORD      = 3,
	V4L2_PRIORITY_DEFAULT     = V4L2_PRIORITY_INTERACTIVE,
};

struct v4l2_rect {
	__s32   left;
	__s32   top;
	__s32   width;
	__s32   height;
};

struct v4l2_fract {
	__u32   numerator;
	__u32   denominator;
};

struct v4l2_capability {
	__u8	driver[16];
	__u8	card[32];
	__u8	bus_info[32];
	__u32   version;
	__u32	capabilities;
	__u32	device_caps;
	__u32	reserved[3];
};

struct htc_callingpid_data {
        pid_t call_pid;
        const char *process_name;
};

#define V4L2_CAP_VIDEO_CAPTURE		0x00000001  
#define V4L2_CAP_VIDEO_OUTPUT		0x00000002  
#define V4L2_CAP_VIDEO_OVERLAY		0x00000004  
#define V4L2_CAP_VBI_CAPTURE		0x00000010  
#define V4L2_CAP_VBI_OUTPUT		0x00000020  
#define V4L2_CAP_SLICED_VBI_CAPTURE	0x00000040  
#define V4L2_CAP_SLICED_VBI_OUTPUT	0x00000080  
#define V4L2_CAP_RDS_CAPTURE		0x00000100  
#define V4L2_CAP_VIDEO_OUTPUT_OVERLAY	0x00000200  
#define V4L2_CAP_HW_FREQ_SEEK		0x00000400  
#define V4L2_CAP_RDS_OUTPUT		0x00000800  

#define V4L2_CAP_VIDEO_CAPTURE_MPLANE	0x00001000
#define V4L2_CAP_VIDEO_OUTPUT_MPLANE	0x00002000

#define V4L2_CAP_TUNER			0x00010000  
#define V4L2_CAP_AUDIO			0x00020000  
#define V4L2_CAP_RADIO			0x00040000  
#define V4L2_CAP_MODULATOR		0x00080000  

#define V4L2_CAP_READWRITE              0x01000000  
#define V4L2_CAP_ASYNCIO                0x02000000  
#define V4L2_CAP_STREAMING              0x04000000  

#define V4L2_CAP_DEVICE_CAPS            0x80000000  

struct v4l2_pix_format {
	__u32         		width;
	__u32			height;
	__u32			pixelformat;
	enum v4l2_field  	field;
	__u32            	bytesperline;	
	__u32          		sizeimage;
	enum v4l2_colorspace	colorspace;
	__u32			priv;		
};


#define V4L2_PIX_FMT_RGB332  v4l2_fourcc('R', 'G', 'B', '1') 
#define V4L2_PIX_FMT_RGB444  v4l2_fourcc('R', '4', '4', '4') 
#define V4L2_PIX_FMT_RGB555  v4l2_fourcc('R', 'G', 'B', 'O') 
#define V4L2_PIX_FMT_RGB565  v4l2_fourcc('R', 'G', 'B', 'P') 
#define V4L2_PIX_FMT_RGB555X v4l2_fourcc('R', 'G', 'B', 'Q') 
#define V4L2_PIX_FMT_RGB565X v4l2_fourcc('R', 'G', 'B', 'R') 
#define V4L2_PIX_FMT_BGR666  v4l2_fourcc('B', 'G', 'R', 'H') 
#define V4L2_PIX_FMT_BGR24   v4l2_fourcc('B', 'G', 'R', '3') 
#define V4L2_PIX_FMT_RGB24   v4l2_fourcc('R', 'G', 'B', '3') 
#define V4L2_PIX_FMT_BGR32   v4l2_fourcc('B', 'G', 'R', '4') 
#define V4L2_PIX_FMT_RGB32   v4l2_fourcc('R', 'G', 'B', '4') 

#define V4L2_PIX_FMT_GREY    v4l2_fourcc('G', 'R', 'E', 'Y') 
#define V4L2_PIX_FMT_Y4      v4l2_fourcc('Y', '0', '4', ' ') 
#define V4L2_PIX_FMT_Y6      v4l2_fourcc('Y', '0', '6', ' ') 
#define V4L2_PIX_FMT_Y10     v4l2_fourcc('Y', '1', '0', ' ') 
#define V4L2_PIX_FMT_Y12     v4l2_fourcc('Y', '1', '2', ' ') 
#define V4L2_PIX_FMT_Y16     v4l2_fourcc('Y', '1', '6', ' ') 

#define V4L2_PIX_FMT_Y10BPACK    v4l2_fourcc('Y', '1', '0', 'B') 

#define V4L2_PIX_FMT_PAL8    v4l2_fourcc('P', 'A', 'L', '8') 

#define V4L2_PIX_FMT_YVU410  v4l2_fourcc('Y', 'V', 'U', '9') 
#define V4L2_PIX_FMT_YVU420  v4l2_fourcc('Y', 'V', '1', '2') 
#define V4L2_PIX_FMT_YUYV    v4l2_fourcc('Y', 'U', 'Y', 'V') 
#define V4L2_PIX_FMT_YYUV    v4l2_fourcc('Y', 'Y', 'U', 'V') 
#define V4L2_PIX_FMT_YVYU    v4l2_fourcc('Y', 'V', 'Y', 'U') 
#define V4L2_PIX_FMT_UYVY    v4l2_fourcc('U', 'Y', 'V', 'Y') 
#define V4L2_PIX_FMT_VYUY    v4l2_fourcc('V', 'Y', 'U', 'Y') 
#define V4L2_PIX_FMT_YUV422P v4l2_fourcc('4', '2', '2', 'P') 
#define V4L2_PIX_FMT_YUV411P v4l2_fourcc('4', '1', '1', 'P') 
#define V4L2_PIX_FMT_Y41P    v4l2_fourcc('Y', '4', '1', 'P') 
#define V4L2_PIX_FMT_YUV444  v4l2_fourcc('Y', '4', '4', '4') 
#define V4L2_PIX_FMT_YUV555  v4l2_fourcc('Y', 'U', 'V', 'O') 
#define V4L2_PIX_FMT_YUV565  v4l2_fourcc('Y', 'U', 'V', 'P') 
#define V4L2_PIX_FMT_YUV32   v4l2_fourcc('Y', 'U', 'V', '4') 
#define V4L2_PIX_FMT_YUV410  v4l2_fourcc('Y', 'U', 'V', '9') 
#define V4L2_PIX_FMT_YUV420  v4l2_fourcc('Y', 'U', '1', '2') 
#define V4L2_PIX_FMT_HI240   v4l2_fourcc('H', 'I', '2', '4') 
#define V4L2_PIX_FMT_HM12    v4l2_fourcc('H', 'M', '1', '2') 
#define V4L2_PIX_FMT_M420    v4l2_fourcc('M', '4', '2', '0') 

#define V4L2_PIX_FMT_NV12    v4l2_fourcc('N', 'V', '1', '2') 
#define V4L2_PIX_FMT_NV21    v4l2_fourcc('N', 'V', '2', '1') 
#define V4L2_PIX_FMT_NV16    v4l2_fourcc('N', 'V', '1', '6') 
#define V4L2_PIX_FMT_NV61    v4l2_fourcc('N', 'V', '6', '1') 
#define V4L2_PIX_FMT_NV24    v4l2_fourcc('N', 'V', '2', '4') 
#define V4L2_PIX_FMT_NV42    v4l2_fourcc('N', 'V', '4', '2') 

#define V4L2_PIX_FMT_NV12M   v4l2_fourcc('N', 'M', '1', '2') 
#define V4L2_PIX_FMT_NV12MT  v4l2_fourcc('T', 'M', '1', '2') 

#define V4L2_PIX_FMT_YUV420M v4l2_fourcc('Y', 'M', '1', '2') 

#define V4L2_PIX_FMT_SBGGR8  v4l2_fourcc('B', 'A', '8', '1') 
#define V4L2_PIX_FMT_SGBRG8  v4l2_fourcc('G', 'B', 'R', 'G') 
#define V4L2_PIX_FMT_SGRBG8  v4l2_fourcc('G', 'R', 'B', 'G') 
#define V4L2_PIX_FMT_SRGGB8  v4l2_fourcc('R', 'G', 'G', 'B') 
#define V4L2_PIX_FMT_SBGGR10 v4l2_fourcc('B', 'G', '1', '0') 
#define V4L2_PIX_FMT_SGBRG10 v4l2_fourcc('G', 'B', '1', '0') 
#define V4L2_PIX_FMT_SGRBG10 v4l2_fourcc('B', 'A', '1', '0') 
#define V4L2_PIX_FMT_SRGGB10 v4l2_fourcc('R', 'G', '1', '0') 
#define V4L2_PIX_FMT_SBGGR12 v4l2_fourcc('B', 'G', '1', '2') 
#define V4L2_PIX_FMT_SGBRG12 v4l2_fourcc('G', 'B', '1', '2') 
#define V4L2_PIX_FMT_SGRBG12 v4l2_fourcc('B', 'A', '1', '2') 
#define V4L2_PIX_FMT_SRGGB12 v4l2_fourcc('R', 'G', '1', '2') 
	
#define V4L2_PIX_FMT_SGRBG10DPCM8 v4l2_fourcc('B', 'D', '1', '0')
#define V4L2_PIX_FMT_SBGGR16 v4l2_fourcc('B', 'Y', 'R', '2') 

#define V4L2_PIX_FMT_MJPEG    v4l2_fourcc('M', 'J', 'P', 'G') 
#define V4L2_PIX_FMT_JPEG     v4l2_fourcc('J', 'P', 'E', 'G') 
#define V4L2_PIX_FMT_DV       v4l2_fourcc('d', 'v', 's', 'd') 
#define V4L2_PIX_FMT_MPEG     v4l2_fourcc('M', 'P', 'E', 'G') 
#define V4L2_PIX_FMT_H264     v4l2_fourcc('H', '2', '6', '4') 
#define V4L2_PIX_FMT_H264_NO_SC v4l2_fourcc('A', 'V', 'C', '1') 
#define V4L2_PIX_FMT_H263     v4l2_fourcc('H', '2', '6', '3') 
#define V4L2_PIX_FMT_MPEG1    v4l2_fourcc('M', 'P', 'G', '1') 
#define V4L2_PIX_FMT_MPEG2    v4l2_fourcc('M', 'P', 'G', '2') 
#define V4L2_PIX_FMT_MPEG4    v4l2_fourcc('M', 'P', 'G', '4') 
#define V4L2_PIX_FMT_XVID     v4l2_fourcc('X', 'V', 'I', 'D') 
#define V4L2_PIX_FMT_VC1_ANNEX_G v4l2_fourcc('V', 'C', '1', 'G') 
#define V4L2_PIX_FMT_VC1_ANNEX_L v4l2_fourcc('V', 'C', '1', 'L') 
#define V4L2_PIX_FMT_DIVX_311  v4l2_fourcc('D', 'I', 'V', '3') 
#define V4L2_PIX_FMT_DIVX      v4l2_fourcc('D', 'I', 'V', 'X') 
#define V4L2_PIX_FMT_VP8 v4l2_fourcc('V', 'P', '8', '0') 
#define V4L2_PIX_FMT_HEVC v4l2_fourcc('H', 'E', 'V', 'C') 
#define V4L2_PIX_FMT_HEVC_HYBRID v4l2_fourcc('H', 'V', 'C', 'H')

#define V4L2_PIX_FMT_CPIA1    v4l2_fourcc('C', 'P', 'I', 'A') 
#define V4L2_PIX_FMT_WNVA     v4l2_fourcc('W', 'N', 'V', 'A') 
#define V4L2_PIX_FMT_SN9C10X  v4l2_fourcc('S', '9', '1', '0') 
#define V4L2_PIX_FMT_SN9C20X_I420 v4l2_fourcc('S', '9', '2', '0') 
#define V4L2_PIX_FMT_PWC1     v4l2_fourcc('P', 'W', 'C', '1') 
#define V4L2_PIX_FMT_PWC2     v4l2_fourcc('P', 'W', 'C', '2') 
#define V4L2_PIX_FMT_ET61X251 v4l2_fourcc('E', '6', '2', '5') 
#define V4L2_PIX_FMT_SPCA501  v4l2_fourcc('S', '5', '0', '1') 
#define V4L2_PIX_FMT_SPCA505  v4l2_fourcc('S', '5', '0', '5') 
#define V4L2_PIX_FMT_SPCA508  v4l2_fourcc('S', '5', '0', '8') 
#define V4L2_PIX_FMT_SPCA561  v4l2_fourcc('S', '5', '6', '1') 
#define V4L2_PIX_FMT_PAC207   v4l2_fourcc('P', '2', '0', '7') 
#define V4L2_PIX_FMT_MR97310A v4l2_fourcc('M', '3', '1', '0') 
#define V4L2_PIX_FMT_JL2005BCD v4l2_fourcc('J', 'L', '2', '0') 
#define V4L2_PIX_FMT_SN9C2028 v4l2_fourcc('S', 'O', 'N', 'X') 
#define V4L2_PIX_FMT_SQ905C   v4l2_fourcc('9', '0', '5', 'C') 
#define V4L2_PIX_FMT_PJPG     v4l2_fourcc('P', 'J', 'P', 'G') 
#define V4L2_PIX_FMT_OV511    v4l2_fourcc('O', '5', '1', '1') 
#define V4L2_PIX_FMT_OV518    v4l2_fourcc('O', '5', '1', '8') 
#define V4L2_PIX_FMT_STV0680  v4l2_fourcc('S', '6', '8', '0') 
#define V4L2_PIX_FMT_TM6000   v4l2_fourcc('T', 'M', '6', '0') 
#define V4L2_PIX_FMT_CIT_YYVYUY v4l2_fourcc('C', 'I', 'T', 'V') 
#define V4L2_PIX_FMT_KONICA420  v4l2_fourcc('K', 'O', 'N', 'I') 
#define V4L2_PIX_FMT_JPGL	v4l2_fourcc('J', 'P', 'G', 'L') 
#define V4L2_PIX_FMT_SE401      v4l2_fourcc('S', '4', '0', '1')
#define V4L2_PIX_FMT_STATS_COMB v4l2_fourcc('S', 'T', 'C', 'M')
#define V4L2_PIX_FMT_STATS_AE   v4l2_fourcc('S', 'T', 'A', 'E')
#define V4L2_PIX_FMT_STATS_AF   v4l2_fourcc('S', 'T', 'A', 'F')
#define V4L2_PIX_FMT_STATS_AWB  v4l2_fourcc('S', 'T', 'W', 'B')
#define V4L2_PIX_FMT_STATS_IHST v4l2_fourcc('I', 'H', 'S', 'T')
#define V4L2_PIX_FMT_STATS_CS   v4l2_fourcc('S', 'T', 'C', 'S')
#define V4L2_PIX_FMT_STATS_RS   v4l2_fourcc('S', 'T', 'R', 'S')
#define V4L2_PIX_FMT_STATS_BG   v4l2_fourcc('S', 'T', 'B', 'G')
#define V4L2_PIX_FMT_STATS_BF   v4l2_fourcc('S', 'T', 'B', 'F')
#define V4L2_PIX_FMT_STATS_BHST v4l2_fourcc('B', 'H', 'S', 'T')

struct v4l2_fmtdesc {
	__u32		    index;             
	enum v4l2_buf_type  type;              
	__u32               flags;
	__u8		    description[32];   
	__u32		    pixelformat;       
	__u32		    reserved[4];
};

#define V4L2_FMT_FLAG_COMPRESSED 0x0001
#define V4L2_FMT_FLAG_EMULATED   0x0002

#if 1
	
enum v4l2_frmsizetypes {
	V4L2_FRMSIZE_TYPE_DISCRETE	= 1,
	V4L2_FRMSIZE_TYPE_CONTINUOUS	= 2,
	V4L2_FRMSIZE_TYPE_STEPWISE	= 3,
};

struct v4l2_frmsize_discrete {
	__u32			width;		
	__u32			height;		
};

struct v4l2_frmsize_stepwise {
	__u32			min_width;	
	__u32			max_width;	
	__u32			step_width;	
	__u32			min_height;	
	__u32			max_height;	
	__u32			step_height;	
};

struct v4l2_frmsizeenum {
	__u32			index;		
	__u32			pixel_format;	
	__u32			type;		

	union {					
		struct v4l2_frmsize_discrete	discrete;
		struct v4l2_frmsize_stepwise	stepwise;
	};

	__u32   reserved[2];			
};

enum v4l2_frmivaltypes {
	V4L2_FRMIVAL_TYPE_DISCRETE	= 1,
	V4L2_FRMIVAL_TYPE_CONTINUOUS	= 2,
	V4L2_FRMIVAL_TYPE_STEPWISE	= 3,
};

struct v4l2_frmival_stepwise {
	struct v4l2_fract	min;		
	struct v4l2_fract	max;		
	struct v4l2_fract	step;		
};

struct v4l2_frmivalenum {
	__u32			index;		
	__u32			pixel_format;	
	__u32			width;		
	__u32			height;		
	__u32			type;		

	union {					
		struct v4l2_fract		discrete;
		struct v4l2_frmival_stepwise	stepwise;
	};

	__u32	reserved[2];			
};
#endif

struct v4l2_timecode {
	__u32	type;
	__u32	flags;
	__u8	frames;
	__u8	seconds;
	__u8	minutes;
	__u8	hours;
	__u8	userbits[4];
};

#define V4L2_TC_TYPE_24FPS		1
#define V4L2_TC_TYPE_25FPS		2
#define V4L2_TC_TYPE_30FPS		3
#define V4L2_TC_TYPE_50FPS		4
#define V4L2_TC_TYPE_60FPS		5

#define V4L2_TC_FLAG_DROPFRAME		0x0001 
#define V4L2_TC_FLAG_COLORFRAME		0x0002
#define V4L2_TC_USERBITS_field		0x000C
#define V4L2_TC_USERBITS_USERDEFINED	0x0000
#define V4L2_TC_USERBITS_8BITCHARS	0x0008

struct v4l2_jpegcompression {
	int quality;

	int  APPn;              /* Number of APP segment to be written,
				 * must be 0..15 */
	int  APP_len;           
	char APP_data[60];      

	int  COM_len;           
	char COM_data[60];      

	__u32 jpeg_markers;     

#define V4L2_JPEG_MARKER_DHT (1<<3)    
#define V4L2_JPEG_MARKER_DQT (1<<4)    
#define V4L2_JPEG_MARKER_DRI (1<<5)    
#define V4L2_JPEG_MARKER_COM (1<<6)    
#define V4L2_JPEG_MARKER_APP (1<<7)    
};

struct v4l2_requestbuffers {
	__u32			count;
	enum v4l2_buf_type      type;
	enum v4l2_memory        memory;
	__u32			reserved[2];
};

struct v4l2_plane {
	__u32			bytesused;
	__u32			length;
	union {
		__u32		mem_offset;
		unsigned long	userptr;
	} m;
	__u32			data_offset;
	__u32			reserved[11];
};

struct v4l2_buffer {
	int			index;
	enum v4l2_buf_type      type;
	__u32			bytesused;
	__u32			flags;
	enum v4l2_field		field;
	struct timeval		timestamp;
	struct v4l2_timecode	timecode;
	__u32			sequence;

	
	enum v4l2_memory        memory;
	union {
		__u32           offset;
		unsigned long   userptr;
		struct v4l2_plane *planes;
	} m;
	__u32			length;
	__u32			input;
	__u32			reserved;
};

#define V4L2_BUF_FLAG_MAPPED	0x0001  
#define V4L2_BUF_FLAG_QUEUED	0x0002	
#define V4L2_BUF_FLAG_DONE	0x0004	
#define V4L2_BUF_FLAG_KEYFRAME	0x0008	
#define V4L2_BUF_FLAG_PFRAME	0x0010	
#define V4L2_BUF_FLAG_BFRAME	0x0020	
#define V4L2_BUF_FLAG_ERROR	0x0040
#define V4L2_BUF_FLAG_TIMECODE	0x0100	
#define V4L2_BUF_FLAG_INPUT     0x0200  
#define V4L2_BUF_FLAG_PREPARED	0x0400	
#define V4L2_BUF_FLAG_NO_CACHE_INVALIDATE	0x0800
#define V4L2_BUF_FLAG_NO_CACHE_CLEAN		0x1000
#define V4L2_BUF_FLAG_EOS		0x2000
#define V4L2_QCOM_BUF_FLAG_CODECCONFIG  0x4000
#define V4L2_QCOM_BUF_FLAG_EOSEQ  0x8000
#define V4L2_QCOM_BUF_TIMESTAMP_INVALID 0x10000
#define V4L2_QCOM_BUF_FLAG_IDRFRAME	0x20000	
#define V4L2_QCOM_BUF_FLAG_DECODEONLY 0x40000
#define V4L2_QCOM_BUF_DATA_CORRUPT 0x80000
#define V4L2_QCOM_BUF_DROP_FRAME 0x100000
#define V4L2_QCOM_BUF_INPUT_UNSUPPORTED 0x200000
#define V4L2_QCOM_BUF_FLAG_EOS          0x2000
#define V4L2_QCOM_BUF_FLAG_READONLY     0x400000
#define V4L2_MSM_BUF_FLAG_MBAFF         0x800000

struct v4l2_framebuffer {
	__u32			capability;
	__u32			flags;
	void                    *base;
	struct v4l2_pix_format	fmt;
};
#define V4L2_FBUF_CAP_EXTERNOVERLAY	0x0001
#define V4L2_FBUF_CAP_CHROMAKEY		0x0002
#define V4L2_FBUF_CAP_LIST_CLIPPING     0x0004
#define V4L2_FBUF_CAP_BITMAP_CLIPPING	0x0008
#define V4L2_FBUF_CAP_LOCAL_ALPHA	0x0010
#define V4L2_FBUF_CAP_GLOBAL_ALPHA	0x0020
#define V4L2_FBUF_CAP_LOCAL_INV_ALPHA	0x0040
#define V4L2_FBUF_CAP_SRC_CHROMAKEY	0x0080
#define V4L2_FBUF_FLAG_PRIMARY		0x0001
#define V4L2_FBUF_FLAG_OVERLAY		0x0002
#define V4L2_FBUF_FLAG_CHROMAKEY	0x0004
#define V4L2_FBUF_FLAG_LOCAL_ALPHA	0x0008
#define V4L2_FBUF_FLAG_GLOBAL_ALPHA	0x0010
#define V4L2_FBUF_FLAG_LOCAL_INV_ALPHA	0x0020
#define V4L2_FBUF_FLAG_SRC_CHROMAKEY	0x0040

struct v4l2_clip {
	struct v4l2_rect        c;
	struct v4l2_clip	__user *next;
};

struct v4l2_window {
	struct v4l2_rect        w;
	enum v4l2_field  	field;
	__u32			chromakey;
	struct v4l2_clip	__user *clips;
	__u32			clipcount;
	void			__user *bitmap;
	__u8                    global_alpha;
};

struct v4l2_captureparm {
	__u32		   capability;	  
	__u32		   capturemode;	  
	struct v4l2_fract  timeperframe;  
	__u32		   extendedmode;  
	__u32              readbuffers;   
	__u32		   reserved[4];
};

#define V4L2_MODE_HIGHQUALITY	0x0001	
#define V4L2_CAP_TIMEPERFRAME	0x1000	
#define V4L2_CAP_QCOM_FRAMESKIP	0x2000	

struct v4l2_qcom_frameskip {
	__u64		   maxframeinterval;
	__u8		   fpsvariance;
};

struct v4l2_outputparm {
	__u32		   capability;	 
	__u32		   outputmode;	 
	struct v4l2_fract  timeperframe; 
	__u32		   extendedmode; 
	__u32              writebuffers; 
	__u32		   reserved[4];
};

struct v4l2_cropcap {
	enum v4l2_buf_type      type;
	struct v4l2_rect        bounds;
	struct v4l2_rect        defrect;
	struct v4l2_fract       pixelaspect;
};

struct v4l2_crop {
	enum v4l2_buf_type      type;
	struct v4l2_rect        c;
};

#define V4L2_SEL_FLAG_GE	0x00000001
#define V4L2_SEL_FLAG_LE	0x00000002


#define V4L2_SEL_TGT_CROP_ACTIVE	0x0000
#define V4L2_SEL_TGT_CROP_DEFAULT	0x0001
#define V4L2_SEL_TGT_CROP_BOUNDS	0x0002
#define V4L2_SEL_TGT_COMPOSE_ACTIVE	0x0100
#define V4L2_SEL_TGT_COMPOSE_DEFAULT	0x0101
#define V4L2_SEL_TGT_COMPOSE_BOUNDS	0x0102
#define V4L2_SEL_TGT_COMPOSE_PADDED	0x0103

struct v4l2_selection {
	__u32			type;
	__u32			target;
	__u32                   flags;
	struct v4l2_rect        r;
	__u32                   reserved[9];
};



typedef __u64 v4l2_std_id;

#define V4L2_STD_PAL_B          ((v4l2_std_id)0x00000001)
#define V4L2_STD_PAL_B1         ((v4l2_std_id)0x00000002)
#define V4L2_STD_PAL_G          ((v4l2_std_id)0x00000004)
#define V4L2_STD_PAL_H          ((v4l2_std_id)0x00000008)
#define V4L2_STD_PAL_I          ((v4l2_std_id)0x00000010)
#define V4L2_STD_PAL_D          ((v4l2_std_id)0x00000020)
#define V4L2_STD_PAL_D1         ((v4l2_std_id)0x00000040)
#define V4L2_STD_PAL_K          ((v4l2_std_id)0x00000080)

#define V4L2_STD_PAL_M          ((v4l2_std_id)0x00000100)
#define V4L2_STD_PAL_N          ((v4l2_std_id)0x00000200)
#define V4L2_STD_PAL_Nc         ((v4l2_std_id)0x00000400)
#define V4L2_STD_PAL_60         ((v4l2_std_id)0x00000800)

#define V4L2_STD_NTSC_M         ((v4l2_std_id)0x00001000)	
#define V4L2_STD_NTSC_M_JP      ((v4l2_std_id)0x00002000)	
#define V4L2_STD_NTSC_443       ((v4l2_std_id)0x00004000)
#define V4L2_STD_NTSC_M_KR      ((v4l2_std_id)0x00008000)	

#define V4L2_STD_SECAM_B        ((v4l2_std_id)0x00010000)
#define V4L2_STD_SECAM_D        ((v4l2_std_id)0x00020000)
#define V4L2_STD_SECAM_G        ((v4l2_std_id)0x00040000)
#define V4L2_STD_SECAM_H        ((v4l2_std_id)0x00080000)
#define V4L2_STD_SECAM_K        ((v4l2_std_id)0x00100000)
#define V4L2_STD_SECAM_K1       ((v4l2_std_id)0x00200000)
#define V4L2_STD_SECAM_L        ((v4l2_std_id)0x00400000)
#define V4L2_STD_SECAM_LC       ((v4l2_std_id)0x00800000)

#define V4L2_STD_ATSC_8_VSB     ((v4l2_std_id)0x01000000)
#define V4L2_STD_ATSC_16_VSB    ((v4l2_std_id)0x02000000)



#define V4L2_STD_NTSC           (V4L2_STD_NTSC_M	|\
				 V4L2_STD_NTSC_M_JP     |\
				 V4L2_STD_NTSC_M_KR)
#define V4L2_STD_SECAM_DK      	(V4L2_STD_SECAM_D	|\
				 V4L2_STD_SECAM_K	|\
				 V4L2_STD_SECAM_K1)
#define V4L2_STD_SECAM		(V4L2_STD_SECAM_B	|\
				 V4L2_STD_SECAM_G	|\
				 V4L2_STD_SECAM_H	|\
				 V4L2_STD_SECAM_DK	|\
				 V4L2_STD_SECAM_L       |\
				 V4L2_STD_SECAM_LC)
#define V4L2_STD_PAL_BG		(V4L2_STD_PAL_B		|\
				 V4L2_STD_PAL_B1	|\
				 V4L2_STD_PAL_G)
#define V4L2_STD_PAL_DK		(V4L2_STD_PAL_D		|\
				 V4L2_STD_PAL_D1	|\
				 V4L2_STD_PAL_K)
#define V4L2_STD_PAL		(V4L2_STD_PAL_BG	|\
				 V4L2_STD_PAL_DK	|\
				 V4L2_STD_PAL_H		|\
				 V4L2_STD_PAL_I)
#define V4L2_STD_B		(V4L2_STD_PAL_B		|\
				 V4L2_STD_PAL_B1	|\
				 V4L2_STD_SECAM_B)
#define V4L2_STD_G		(V4L2_STD_PAL_G		|\
				 V4L2_STD_SECAM_G)
#define V4L2_STD_H		(V4L2_STD_PAL_H		|\
				 V4L2_STD_SECAM_H)
#define V4L2_STD_L		(V4L2_STD_SECAM_L	|\
				 V4L2_STD_SECAM_LC)
#define V4L2_STD_GH		(V4L2_STD_G		|\
				 V4L2_STD_H)
#define V4L2_STD_DK		(V4L2_STD_PAL_DK	|\
				 V4L2_STD_SECAM_DK)
#define V4L2_STD_BG		(V4L2_STD_B		|\
				 V4L2_STD_G)
#define V4L2_STD_MN		(V4L2_STD_PAL_M		|\
				 V4L2_STD_PAL_N		|\
				 V4L2_STD_PAL_Nc	|\
				 V4L2_STD_NTSC)

#define V4L2_STD_MTS		(V4L2_STD_NTSC_M	|\
				 V4L2_STD_PAL_M		|\
				 V4L2_STD_PAL_N		|\
				 V4L2_STD_PAL_Nc)

#define V4L2_STD_525_60		(V4L2_STD_PAL_M		|\
				 V4L2_STD_PAL_60	|\
				 V4L2_STD_NTSC		|\
				 V4L2_STD_NTSC_443)
#define V4L2_STD_625_50		(V4L2_STD_PAL		|\
				 V4L2_STD_PAL_N		|\
				 V4L2_STD_PAL_Nc	|\
				 V4L2_STD_SECAM)

#define V4L2_STD_ATSC           (V4L2_STD_ATSC_8_VSB    |\
				 V4L2_STD_ATSC_16_VSB)
#define V4L2_STD_UNKNOWN        0
#define V4L2_STD_ALL            (V4L2_STD_525_60	|\
				 V4L2_STD_625_50)

struct v4l2_standard {
	__u32		     index;
	v4l2_std_id          id;
	__u8		     name[24];
	struct v4l2_fract    frameperiod; 
	__u32		     framelines;
	__u32		     reserved[4];
};

struct v4l2_dv_preset {
	__u32	preset;
	__u32	reserved[4];
};

struct v4l2_dv_enum_preset {
	__u32	index;
	__u32	preset;
	__u8	name[32]; 
	__u32	width;
	__u32	height;
	__u32	reserved[4];
};

#define		V4L2_DV_INVALID		0
#define		V4L2_DV_480P59_94	1 
#define		V4L2_DV_576P50		2 
#define		V4L2_DV_720P24		3 
#define		V4L2_DV_720P25		4 
#define		V4L2_DV_720P30		5 
#define		V4L2_DV_720P50		6 
#define		V4L2_DV_720P59_94	7 
#define		V4L2_DV_720P60		8 
#define		V4L2_DV_1080I29_97	9 
#define		V4L2_DV_1080I30		10 
#define		V4L2_DV_1080I25		11 
#define		V4L2_DV_1080I50		12 
#define		V4L2_DV_1080I60		13 
#define		V4L2_DV_1080P24		14 
#define		V4L2_DV_1080P25		15 
#define		V4L2_DV_1080P30		16 
#define		V4L2_DV_1080P50		17 
#define		V4L2_DV_1080P60		18 


struct v4l2_bt_timings {
	__u32	width;		
	__u32	height;		
	__u32	interlaced;	
	__u32	polarities;	
	__u64	pixelclock;	
	__u32	hfrontporch;	
	__u32	hsync;		
	__u32	hbackporch;	
	__u32	vfrontporch;	
	__u32	vsync;		
	__u32	vbackporch;	
	__u32	il_vfrontporch;	
	__u32	il_vsync;	
	__u32	il_vbackporch;	
	__u32	reserved[16];
} __attribute__ ((packed));

#define	V4L2_DV_PROGRESSIVE	0
#define	V4L2_DV_INTERLACED	1

#define V4L2_DV_VSYNC_POS_POL	0x00000001
#define V4L2_DV_HSYNC_POS_POL	0x00000002


struct v4l2_dv_timings {
	__u32 type;
	union {
		struct v4l2_bt_timings	bt;
		__u32	reserved[32];
	};
} __attribute__ ((packed));

#define V4L2_DV_BT_656_1120	0	

struct v4l2_input {
	__u32	     index;		
	__u8	     name[32];		
	__u32	     type;		
	__u32	     audioset;		
	__u32        tuner;             
	v4l2_std_id  std;
	__u32	     status;
	__u32	     capabilities;
	__u32	     reserved[3];
};

#define V4L2_INPUT_TYPE_TUNER		1
#define V4L2_INPUT_TYPE_CAMERA		2

#define V4L2_IN_ST_NO_POWER    0x00000001  
#define V4L2_IN_ST_NO_SIGNAL   0x00000002
#define V4L2_IN_ST_NO_COLOR    0x00000004

#define V4L2_IN_ST_HFLIP       0x00000010 
#define V4L2_IN_ST_VFLIP       0x00000020 

#define V4L2_IN_ST_NO_H_LOCK   0x00000100  
#define V4L2_IN_ST_COLOR_KILL  0x00000200  

#define V4L2_IN_ST_NO_SYNC     0x00010000  
#define V4L2_IN_ST_NO_EQU      0x00020000  
#define V4L2_IN_ST_NO_CARRIER  0x00040000  

#define V4L2_IN_ST_MACROVISION 0x01000000  
#define V4L2_IN_ST_NO_ACCESS   0x02000000  
#define V4L2_IN_ST_VTR         0x04000000  

#define V4L2_IN_CAP_PRESETS		0x00000001 
#define V4L2_IN_CAP_CUSTOM_TIMINGS	0x00000002 
#define V4L2_IN_CAP_STD			0x00000004 

struct v4l2_output {
	__u32	     index;		
	__u8	     name[32];		
	__u32	     type;		
	__u32	     audioset;		
	__u32	     modulator;         
	v4l2_std_id  std;
	__u32	     capabilities;
	__u32	     reserved[3];
};
#define V4L2_OUTPUT_TYPE_MODULATOR		1
#define V4L2_OUTPUT_TYPE_ANALOG			2
#define V4L2_OUTPUT_TYPE_ANALOGVGAOVERLAY	3

#define V4L2_OUT_CAP_PRESETS		0x00000001 
#define V4L2_OUT_CAP_CUSTOM_TIMINGS	0x00000002 
#define V4L2_OUT_CAP_STD		0x00000004 

struct v4l2_control {
	__u32		     id;
	__s32		     value;
};

struct v4l2_ext_control {
	__u32 id;
	__u32 size;
	__u32 reserved2[1];
	union {
		__s32 value;
		__s64 value64;
		char *string;
	};
} __attribute__ ((packed));

struct v4l2_ext_controls {
	__u32 ctrl_class;
	__u32 count;
	__u32 error_idx;
	__u32 reserved[2];
	struct v4l2_ext_control *controls;
};

#define V4L2_CTRL_CLASS_USER 0x00980000	
#define V4L2_CTRL_CLASS_MPEG 0x00990000	
#define V4L2_CTRL_CLASS_CAMERA 0x009a0000	
#define V4L2_CTRL_CLASS_FM_TX 0x009b0000	
#define V4L2_CTRL_CLASS_FLASH 0x009c0000	
#define V4L2_CTRL_CLASS_JPEG 0x009d0000		

#define V4L2_CTRL_ID_MASK      	  (0x0fffffff)
#define V4L2_CTRL_ID2CLASS(id)    ((id) & 0x0fff0000UL)
#define V4L2_CTRL_DRIVER_PRIV(id) (((id) & 0xffff) >= 0x1000)

enum v4l2_ctrl_type {
	V4L2_CTRL_TYPE_INTEGER	     = 1,
	V4L2_CTRL_TYPE_BOOLEAN	     = 2,
	V4L2_CTRL_TYPE_MENU	     = 3,
	V4L2_CTRL_TYPE_BUTTON	     = 4,
	V4L2_CTRL_TYPE_INTEGER64     = 5,
	V4L2_CTRL_TYPE_CTRL_CLASS    = 6,
	V4L2_CTRL_TYPE_STRING        = 7,
	V4L2_CTRL_TYPE_BITMASK       = 8,
};

struct v4l2_queryctrl {
	__u32		     id;
	enum v4l2_ctrl_type  type;
	__u8		     name[32];	
	__s32		     minimum;	
	__s32		     maximum;
	__s32		     step;
	__s32		     default_value;
	__u32                flags;
	__u32		     reserved[2];
};

struct v4l2_querymenu {
	__u32		id;
	__u32		index;
	__u8		name[32];	
	__u32		reserved;
};

#define V4L2_CTRL_FLAG_DISABLED		0x0001
#define V4L2_CTRL_FLAG_GRABBED		0x0002
#define V4L2_CTRL_FLAG_READ_ONLY 	0x0004
#define V4L2_CTRL_FLAG_UPDATE 		0x0008
#define V4L2_CTRL_FLAG_INACTIVE 	0x0010
#define V4L2_CTRL_FLAG_SLIDER 		0x0020
#define V4L2_CTRL_FLAG_WRITE_ONLY 	0x0040
#define V4L2_CTRL_FLAG_VOLATILE		0x0080

#define V4L2_CTRL_FLAG_NEXT_CTRL	0x80000000

#define V4L2_CID_MAX_CTRLS		1024
#define V4L2_CID_BASE			(V4L2_CTRL_CLASS_USER | 0x900)
#define V4L2_CID_USER_BASE 		V4L2_CID_BASE
#define V4L2_CID_PRIVATE_BASE		0x08000000

#define V4L2_CID_USER_CLASS 		(V4L2_CTRL_CLASS_USER | 1)
#define V4L2_CID_BRIGHTNESS		(V4L2_CID_BASE+0)
#define V4L2_CID_CONTRAST		(V4L2_CID_BASE+1)
#define V4L2_CID_SATURATION		(V4L2_CID_BASE+2)
#define V4L2_CID_HUE			(V4L2_CID_BASE+3)
#define V4L2_CID_AUDIO_VOLUME		(V4L2_CID_BASE+5)
#define V4L2_CID_AUDIO_BALANCE		(V4L2_CID_BASE+6)
#define V4L2_CID_AUDIO_BASS		(V4L2_CID_BASE+7)
#define V4L2_CID_AUDIO_TREBLE		(V4L2_CID_BASE+8)
#define V4L2_CID_AUDIO_MUTE		(V4L2_CID_BASE+9)
#define V4L2_CID_AUDIO_LOUDNESS		(V4L2_CID_BASE+10)
#define V4L2_CID_BLACK_LEVEL		(V4L2_CID_BASE+11) 
#define V4L2_CID_AUTO_WHITE_BALANCE	(V4L2_CID_BASE+12)
#define V4L2_CID_DO_WHITE_BALANCE	(V4L2_CID_BASE+13)
#define V4L2_CID_RED_BALANCE		(V4L2_CID_BASE+14)
#define V4L2_CID_BLUE_BALANCE		(V4L2_CID_BASE+15)
#define V4L2_CID_GAMMA			(V4L2_CID_BASE+16)
#define V4L2_CID_WHITENESS		(V4L2_CID_GAMMA) 
#define V4L2_CID_EXPOSURE		(V4L2_CID_BASE+17)
#define V4L2_CID_AUTOGAIN		(V4L2_CID_BASE+18)
#define V4L2_CID_GAIN			(V4L2_CID_BASE+19)
#define V4L2_CID_HFLIP			(V4L2_CID_BASE+20)
#define V4L2_CID_VFLIP			(V4L2_CID_BASE+21)

#define V4L2_CID_HCENTER		(V4L2_CID_BASE+22)
#define V4L2_CID_VCENTER		(V4L2_CID_BASE+23)

#define V4L2_CID_POWER_LINE_FREQUENCY	(V4L2_CID_BASE+24)
enum v4l2_power_line_frequency {
	V4L2_CID_POWER_LINE_FREQUENCY_DISABLED	= 0,
	V4L2_CID_POWER_LINE_FREQUENCY_50HZ	= 1,
	V4L2_CID_POWER_LINE_FREQUENCY_60HZ	= 2,
	V4L2_CID_POWER_LINE_FREQUENCY_AUTO	= 3,
};
#define V4L2_CID_HUE_AUTO			(V4L2_CID_BASE+25)
#define V4L2_CID_WHITE_BALANCE_TEMPERATURE	(V4L2_CID_BASE+26)
#define V4L2_CID_SHARPNESS			(V4L2_CID_BASE+27)
#define V4L2_CID_BACKLIGHT_COMPENSATION 	(V4L2_CID_BASE+28)
#define V4L2_CID_CHROMA_AGC                     (V4L2_CID_BASE+29)
#define V4L2_CID_COLOR_KILLER                   (V4L2_CID_BASE+30)
#define V4L2_CID_COLORFX			(V4L2_CID_BASE+31)
enum v4l2_colorfx {
	V4L2_COLORFX_NONE	= 0,
	V4L2_COLORFX_BW		= 1,
	V4L2_COLORFX_SEPIA	= 2,
	V4L2_COLORFX_NEGATIVE = 3,
	V4L2_COLORFX_EMBOSS = 4,
	V4L2_COLORFX_SKETCH = 5,
	V4L2_COLORFX_SKY_BLUE = 6,
	V4L2_COLORFX_GRASS_GREEN = 7,
	V4L2_COLORFX_SKIN_WHITEN = 8,
	V4L2_COLORFX_VIVID = 9,
};
#define V4L2_CID_AUTOBRIGHTNESS			(V4L2_CID_BASE+32)
#define V4L2_CID_BAND_STOP_FILTER		(V4L2_CID_BASE+33)

#define V4L2_CID_ROTATE				(V4L2_CID_BASE+34)
#define V4L2_CID_BG_COLOR			(V4L2_CID_BASE+35)

#define V4L2_CID_CHROMA_GAIN                    (V4L2_CID_BASE+36)

#define V4L2_CID_ILLUMINATORS_1			(V4L2_CID_BASE+37)
#define V4L2_CID_ILLUMINATORS_2			(V4L2_CID_BASE+38)

#define V4L2_CID_MIN_BUFFERS_FOR_CAPTURE	(V4L2_CID_BASE+39)
#define V4L2_CID_MIN_BUFFERS_FOR_OUTPUT		(V4L2_CID_BASE+40)

#define V4L2_CID_ALPHA_COMPONENT		(V4L2_CID_BASE+41)

#define V4L2_CID_LASTP1                         (V4L2_CID_BASE+42)
#define V4L2_CID_SPECIAL_EFFECT			(V4L2_CID_BASE+43)

#define V4L2_CID_MPEG_BASE 			(V4L2_CTRL_CLASS_MPEG | 0x900)
#define V4L2_CID_MPEG_CLASS 			(V4L2_CTRL_CLASS_MPEG | 1)

#define V4L2_CID_MPEG_STREAM_TYPE 		(V4L2_CID_MPEG_BASE+0)
enum v4l2_mpeg_stream_type {
	V4L2_MPEG_STREAM_TYPE_MPEG2_PS   = 0, 
	V4L2_MPEG_STREAM_TYPE_MPEG2_TS   = 1, 
	V4L2_MPEG_STREAM_TYPE_MPEG1_SS   = 2, 
	V4L2_MPEG_STREAM_TYPE_MPEG2_DVD  = 3, 
	V4L2_MPEG_STREAM_TYPE_MPEG1_VCD  = 4, 
	V4L2_MPEG_STREAM_TYPE_MPEG2_SVCD = 5, 
};
#define V4L2_CID_MPEG_STREAM_PID_PMT 		(V4L2_CID_MPEG_BASE+1)
#define V4L2_CID_MPEG_STREAM_PID_AUDIO 		(V4L2_CID_MPEG_BASE+2)
#define V4L2_CID_MPEG_STREAM_PID_VIDEO 		(V4L2_CID_MPEG_BASE+3)
#define V4L2_CID_MPEG_STREAM_PID_PCR 		(V4L2_CID_MPEG_BASE+4)
#define V4L2_CID_MPEG_STREAM_PES_ID_AUDIO 	(V4L2_CID_MPEG_BASE+5)
#define V4L2_CID_MPEG_STREAM_PES_ID_VIDEO 	(V4L2_CID_MPEG_BASE+6)
#define V4L2_CID_MPEG_STREAM_VBI_FMT 		(V4L2_CID_MPEG_BASE+7)
enum v4l2_mpeg_stream_vbi_fmt {
	V4L2_MPEG_STREAM_VBI_FMT_NONE = 0,  
	V4L2_MPEG_STREAM_VBI_FMT_IVTV = 1,  
};

#define V4L2_CID_MPEG_AUDIO_SAMPLING_FREQ 	(V4L2_CID_MPEG_BASE+100)
enum v4l2_mpeg_audio_sampling_freq {
	V4L2_MPEG_AUDIO_SAMPLING_FREQ_44100 = 0,
	V4L2_MPEG_AUDIO_SAMPLING_FREQ_48000 = 1,
	V4L2_MPEG_AUDIO_SAMPLING_FREQ_32000 = 2,
};
#define V4L2_CID_MPEG_AUDIO_ENCODING 		(V4L2_CID_MPEG_BASE+101)
enum v4l2_mpeg_audio_encoding {
	V4L2_MPEG_AUDIO_ENCODING_LAYER_1 = 0,
	V4L2_MPEG_AUDIO_ENCODING_LAYER_2 = 1,
	V4L2_MPEG_AUDIO_ENCODING_LAYER_3 = 2,
	V4L2_MPEG_AUDIO_ENCODING_AAC     = 3,
	V4L2_MPEG_AUDIO_ENCODING_AC3     = 4,
};
#define V4L2_CID_MPEG_AUDIO_L1_BITRATE 		(V4L2_CID_MPEG_BASE+102)
enum v4l2_mpeg_audio_l1_bitrate {
	V4L2_MPEG_AUDIO_L1_BITRATE_32K  = 0,
	V4L2_MPEG_AUDIO_L1_BITRATE_64K  = 1,
	V4L2_MPEG_AUDIO_L1_BITRATE_96K  = 2,
	V4L2_MPEG_AUDIO_L1_BITRATE_128K = 3,
	V4L2_MPEG_AUDIO_L1_BITRATE_160K = 4,
	V4L2_MPEG_AUDIO_L1_BITRATE_192K = 5,
	V4L2_MPEG_AUDIO_L1_BITRATE_224K = 6,
	V4L2_MPEG_AUDIO_L1_BITRATE_256K = 7,
	V4L2_MPEG_AUDIO_L1_BITRATE_288K = 8,
	V4L2_MPEG_AUDIO_L1_BITRATE_320K = 9,
	V4L2_MPEG_AUDIO_L1_BITRATE_352K = 10,
	V4L2_MPEG_AUDIO_L1_BITRATE_384K = 11,
	V4L2_MPEG_AUDIO_L1_BITRATE_416K = 12,
	V4L2_MPEG_AUDIO_L1_BITRATE_448K = 13,
};
#define V4L2_CID_MPEG_AUDIO_L2_BITRATE 		(V4L2_CID_MPEG_BASE+103)
enum v4l2_mpeg_audio_l2_bitrate {
	V4L2_MPEG_AUDIO_L2_BITRATE_32K  = 0,
	V4L2_MPEG_AUDIO_L2_BITRATE_48K  = 1,
	V4L2_MPEG_AUDIO_L2_BITRATE_56K  = 2,
	V4L2_MPEG_AUDIO_L2_BITRATE_64K  = 3,
	V4L2_MPEG_AUDIO_L2_BITRATE_80K  = 4,
	V4L2_MPEG_AUDIO_L2_BITRATE_96K  = 5,
	V4L2_MPEG_AUDIO_L2_BITRATE_112K = 6,
	V4L2_MPEG_AUDIO_L2_BITRATE_128K = 7,
	V4L2_MPEG_AUDIO_L2_BITRATE_160K = 8,
	V4L2_MPEG_AUDIO_L2_BITRATE_192K = 9,
	V4L2_MPEG_AUDIO_L2_BITRATE_224K = 10,
	V4L2_MPEG_AUDIO_L2_BITRATE_256K = 11,
	V4L2_MPEG_AUDIO_L2_BITRATE_320K = 12,
	V4L2_MPEG_AUDIO_L2_BITRATE_384K = 13,
};
#define V4L2_CID_MPEG_AUDIO_L3_BITRATE 		(V4L2_CID_MPEG_BASE+104)
enum v4l2_mpeg_audio_l3_bitrate {
	V4L2_MPEG_AUDIO_L3_BITRATE_32K  = 0,
	V4L2_MPEG_AUDIO_L3_BITRATE_40K  = 1,
	V4L2_MPEG_AUDIO_L3_BITRATE_48K  = 2,
	V4L2_MPEG_AUDIO_L3_BITRATE_56K  = 3,
	V4L2_MPEG_AUDIO_L3_BITRATE_64K  = 4,
	V4L2_MPEG_AUDIO_L3_BITRATE_80K  = 5,
	V4L2_MPEG_AUDIO_L3_BITRATE_96K  = 6,
	V4L2_MPEG_AUDIO_L3_BITRATE_112K = 7,
	V4L2_MPEG_AUDIO_L3_BITRATE_128K = 8,
	V4L2_MPEG_AUDIO_L3_BITRATE_160K = 9,
	V4L2_MPEG_AUDIO_L3_BITRATE_192K = 10,
	V4L2_MPEG_AUDIO_L3_BITRATE_224K = 11,
	V4L2_MPEG_AUDIO_L3_BITRATE_256K = 12,
	V4L2_MPEG_AUDIO_L3_BITRATE_320K = 13,
};
#define V4L2_CID_MPEG_AUDIO_MODE 		(V4L2_CID_MPEG_BASE+105)
enum v4l2_mpeg_audio_mode {
	V4L2_MPEG_AUDIO_MODE_STEREO       = 0,
	V4L2_MPEG_AUDIO_MODE_JOINT_STEREO = 1,
	V4L2_MPEG_AUDIO_MODE_DUAL         = 2,
	V4L2_MPEG_AUDIO_MODE_MONO         = 3,
};
#define V4L2_CID_MPEG_AUDIO_MODE_EXTENSION 	(V4L2_CID_MPEG_BASE+106)
enum v4l2_mpeg_audio_mode_extension {
	V4L2_MPEG_AUDIO_MODE_EXTENSION_BOUND_4  = 0,
	V4L2_MPEG_AUDIO_MODE_EXTENSION_BOUND_8  = 1,
	V4L2_MPEG_AUDIO_MODE_EXTENSION_BOUND_12 = 2,
	V4L2_MPEG_AUDIO_MODE_EXTENSION_BOUND_16 = 3,
};
#define V4L2_CID_MPEG_AUDIO_EMPHASIS 		(V4L2_CID_MPEG_BASE+107)
enum v4l2_mpeg_audio_emphasis {
	V4L2_MPEG_AUDIO_EMPHASIS_NONE         = 0,
	V4L2_MPEG_AUDIO_EMPHASIS_50_DIV_15_uS = 1,
	V4L2_MPEG_AUDIO_EMPHASIS_CCITT_J17    = 2,
};
#define V4L2_CID_MPEG_AUDIO_CRC 		(V4L2_CID_MPEG_BASE+108)
enum v4l2_mpeg_audio_crc {
	V4L2_MPEG_AUDIO_CRC_NONE  = 0,
	V4L2_MPEG_AUDIO_CRC_CRC16 = 1,
};
#define V4L2_CID_MPEG_AUDIO_MUTE 		(V4L2_CID_MPEG_BASE+109)
#define V4L2_CID_MPEG_AUDIO_AAC_BITRATE		(V4L2_CID_MPEG_BASE+110)
#define V4L2_CID_MPEG_AUDIO_AC3_BITRATE		(V4L2_CID_MPEG_BASE+111)
enum v4l2_mpeg_audio_ac3_bitrate {
	V4L2_MPEG_AUDIO_AC3_BITRATE_32K  = 0,
	V4L2_MPEG_AUDIO_AC3_BITRATE_40K  = 1,
	V4L2_MPEG_AUDIO_AC3_BITRATE_48K  = 2,
	V4L2_MPEG_AUDIO_AC3_BITRATE_56K  = 3,
	V4L2_MPEG_AUDIO_AC3_BITRATE_64K  = 4,
	V4L2_MPEG_AUDIO_AC3_BITRATE_80K  = 5,
	V4L2_MPEG_AUDIO_AC3_BITRATE_96K  = 6,
	V4L2_MPEG_AUDIO_AC3_BITRATE_112K = 7,
	V4L2_MPEG_AUDIO_AC3_BITRATE_128K = 8,
	V4L2_MPEG_AUDIO_AC3_BITRATE_160K = 9,
	V4L2_MPEG_AUDIO_AC3_BITRATE_192K = 10,
	V4L2_MPEG_AUDIO_AC3_BITRATE_224K = 11,
	V4L2_MPEG_AUDIO_AC3_BITRATE_256K = 12,
	V4L2_MPEG_AUDIO_AC3_BITRATE_320K = 13,
	V4L2_MPEG_AUDIO_AC3_BITRATE_384K = 14,
	V4L2_MPEG_AUDIO_AC3_BITRATE_448K = 15,
	V4L2_MPEG_AUDIO_AC3_BITRATE_512K = 16,
	V4L2_MPEG_AUDIO_AC3_BITRATE_576K = 17,
	V4L2_MPEG_AUDIO_AC3_BITRATE_640K = 18,
};
#define V4L2_CID_MPEG_AUDIO_DEC_PLAYBACK	(V4L2_CID_MPEG_BASE+112)
enum v4l2_mpeg_audio_dec_playback {
	V4L2_MPEG_AUDIO_DEC_PLAYBACK_AUTO	    = 0,
	V4L2_MPEG_AUDIO_DEC_PLAYBACK_STEREO	    = 1,
	V4L2_MPEG_AUDIO_DEC_PLAYBACK_LEFT	    = 2,
	V4L2_MPEG_AUDIO_DEC_PLAYBACK_RIGHT	    = 3,
	V4L2_MPEG_AUDIO_DEC_PLAYBACK_MONO	    = 4,
	V4L2_MPEG_AUDIO_DEC_PLAYBACK_SWAPPED_STEREO = 5,
};
#define V4L2_CID_MPEG_AUDIO_DEC_MULTILINGUAL_PLAYBACK (V4L2_CID_MPEG_BASE+113)

#define V4L2_CID_MPEG_VIDEO_ENCODING 		(V4L2_CID_MPEG_BASE+200)
enum v4l2_mpeg_video_encoding {
	V4L2_MPEG_VIDEO_ENCODING_MPEG_1     = 0,
	V4L2_MPEG_VIDEO_ENCODING_MPEG_2     = 1,
	V4L2_MPEG_VIDEO_ENCODING_MPEG_4_AVC = 2,
};
#define V4L2_CID_MPEG_VIDEO_ASPECT 		(V4L2_CID_MPEG_BASE+201)
enum v4l2_mpeg_video_aspect {
	V4L2_MPEG_VIDEO_ASPECT_1x1     = 0,
	V4L2_MPEG_VIDEO_ASPECT_4x3     = 1,
	V4L2_MPEG_VIDEO_ASPECT_16x9    = 2,
	V4L2_MPEG_VIDEO_ASPECT_221x100 = 3,
};
#define V4L2_CID_MPEG_VIDEO_B_FRAMES 		(V4L2_CID_MPEG_BASE+202)
#define V4L2_CID_MPEG_VIDEO_GOP_SIZE 		(V4L2_CID_MPEG_BASE+203)
#define V4L2_CID_MPEG_VIDEO_GOP_CLOSURE 	(V4L2_CID_MPEG_BASE+204)
#define V4L2_CID_MPEG_VIDEO_PULLDOWN 		(V4L2_CID_MPEG_BASE+205)
#define V4L2_CID_MPEG_VIDEO_BITRATE_MODE 	(V4L2_CID_MPEG_BASE+206)
enum v4l2_mpeg_video_bitrate_mode {
	V4L2_MPEG_VIDEO_BITRATE_MODE_VBR = 0,
	V4L2_MPEG_VIDEO_BITRATE_MODE_CBR = 1,
};
#define V4L2_CID_MPEG_VIDEO_BITRATE 		(V4L2_CID_MPEG_BASE+207)
#define V4L2_CID_MPEG_VIDEO_BITRATE_PEAK 	(V4L2_CID_MPEG_BASE+208)
#define V4L2_CID_MPEG_VIDEO_TEMPORAL_DECIMATION (V4L2_CID_MPEG_BASE+209)
#define V4L2_CID_MPEG_VIDEO_MUTE 		(V4L2_CID_MPEG_BASE+210)
#define V4L2_CID_MPEG_VIDEO_MUTE_YUV 		(V4L2_CID_MPEG_BASE+211)
#define V4L2_CID_MPEG_VIDEO_DECODER_SLICE_INTERFACE		(V4L2_CID_MPEG_BASE+212)
#define V4L2_CID_MPEG_VIDEO_DECODER_MPEG4_DEBLOCK_FILTER	(V4L2_CID_MPEG_BASE+213)
#define V4L2_CID_MPEG_VIDEO_CYCLIC_INTRA_REFRESH_MB		(V4L2_CID_MPEG_BASE+214)
#define V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE			(V4L2_CID_MPEG_BASE+215)
#define V4L2_CID_MPEG_VIDEO_HEADER_MODE				(V4L2_CID_MPEG_BASE+216)
enum v4l2_mpeg_video_header_mode {
	V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE			= 0,
	V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME	= 1,
	V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_I_FRAME		= 2,

};
#define V4L2_CID_MPEG_VIDEO_MAX_REF_PIC			(V4L2_CID_MPEG_BASE+217)
#define V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE		(V4L2_CID_MPEG_BASE+218)
#define V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES	(V4L2_CID_MPEG_BASE+219)
#define V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB		(V4L2_CID_MPEG_BASE+220)
#define V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE		(V4L2_CID_MPEG_BASE+221)
enum v4l2_mpeg_video_multi_slice_mode {
	V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE		= 0,
	V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_MB		= 1,
	V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_BYTES	= 2,
	V4L2_MPEG_VIDEO_MULTI_SLICE_GOB			= 3,
};
#define V4L2_CID_MPEG_VIDEO_VBV_SIZE			(V4L2_CID_MPEG_BASE+222)
#define V4L2_CID_MPEG_VIDEO_DEC_PTS			(V4L2_CID_MPEG_BASE+223)
#define V4L2_CID_MPEG_VIDEO_DEC_FRAME			(V4L2_CID_MPEG_BASE+224)

#define V4L2_CID_MPEG_VIDEO_H263_I_FRAME_QP		(V4L2_CID_MPEG_BASE+300)
#define V4L2_CID_MPEG_VIDEO_H263_P_FRAME_QP		(V4L2_CID_MPEG_BASE+301)
#define V4L2_CID_MPEG_VIDEO_H263_B_FRAME_QP		(V4L2_CID_MPEG_BASE+302)
#define V4L2_CID_MPEG_VIDEO_H263_MIN_QP			(V4L2_CID_MPEG_BASE+303)
#define V4L2_CID_MPEG_VIDEO_H263_MAX_QP			(V4L2_CID_MPEG_BASE+304)
#define V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP		(V4L2_CID_MPEG_BASE+350)
#define V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP		(V4L2_CID_MPEG_BASE+351)
#define V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP		(V4L2_CID_MPEG_BASE+352)
#define V4L2_CID_MPEG_VIDEO_H264_MIN_QP			(V4L2_CID_MPEG_BASE+353)
#define V4L2_CID_MPEG_VIDEO_H264_MAX_QP			(V4L2_CID_MPEG_BASE+354)
#define V4L2_CID_MPEG_VIDEO_H264_8X8_TRANSFORM		(V4L2_CID_MPEG_BASE+355)
#define V4L2_CID_MPEG_VIDEO_H264_CPB_SIZE		(V4L2_CID_MPEG_BASE+356)
#define V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE		(V4L2_CID_MPEG_BASE+357)
enum v4l2_mpeg_video_h264_entropy_mode {
	V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC	= 0,
	V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC	= 1,
};
#define V4L2_CID_MPEG_VIDEO_H264_I_PERIOD		(V4L2_CID_MPEG_BASE+358)
#define V4L2_CID_MPEG_VIDEO_H264_LEVEL			(V4L2_CID_MPEG_BASE+359)
enum v4l2_mpeg_video_h264_level {
	V4L2_MPEG_VIDEO_H264_LEVEL_1_0	= 0,
	V4L2_MPEG_VIDEO_H264_LEVEL_1B	= 1,
	V4L2_MPEG_VIDEO_H264_LEVEL_1_1	= 2,
	V4L2_MPEG_VIDEO_H264_LEVEL_1_2	= 3,
	V4L2_MPEG_VIDEO_H264_LEVEL_1_3	= 4,
	V4L2_MPEG_VIDEO_H264_LEVEL_2_0	= 5,
	V4L2_MPEG_VIDEO_H264_LEVEL_2_1	= 6,
	V4L2_MPEG_VIDEO_H264_LEVEL_2_2	= 7,
	V4L2_MPEG_VIDEO_H264_LEVEL_3_0	= 8,
	V4L2_MPEG_VIDEO_H264_LEVEL_3_1	= 9,
	V4L2_MPEG_VIDEO_H264_LEVEL_3_2	= 10,
	V4L2_MPEG_VIDEO_H264_LEVEL_4_0	= 11,
	V4L2_MPEG_VIDEO_H264_LEVEL_4_1	= 12,
	V4L2_MPEG_VIDEO_H264_LEVEL_4_2	= 13,
	V4L2_MPEG_VIDEO_H264_LEVEL_5_0	= 14,
	V4L2_MPEG_VIDEO_H264_LEVEL_5_1	= 15,
	V4L2_MPEG_VIDEO_H264_LEVEL_5_2	= 16,
};
#define V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA	(V4L2_CID_MPEG_BASE+360)
#define V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA	(V4L2_CID_MPEG_BASE+361)
#define V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE	(V4L2_CID_MPEG_BASE+362)
enum v4l2_mpeg_video_h264_loop_filter_mode {
	V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_ENABLED				= 0,
	V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED				= 1,
	V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED_AT_SLICE_BOUNDARY	= 2,
};
#define V4L2_CID_MPEG_VIDEO_H264_PROFILE		(V4L2_CID_MPEG_BASE+363)
enum v4l2_mpeg_video_h264_profile {
	V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE			= 0,
	V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE	= 1,
	V4L2_MPEG_VIDEO_H264_PROFILE_MAIN			= 2,
	V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED			= 3,
	V4L2_MPEG_VIDEO_H264_PROFILE_HIGH			= 4,
	V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10			= 5,
	V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422			= 6,
	V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE	= 7,
	V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10_INTRA		= 8,
	V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422_INTRA		= 9,
	V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_INTRA		= 10,
	V4L2_MPEG_VIDEO_H264_PROFILE_CAVLC_444_INTRA		= 11,
	V4L2_MPEG_VIDEO_H264_PROFILE_SCALABLE_BASELINE		= 12,
	V4L2_MPEG_VIDEO_H264_PROFILE_SCALABLE_HIGH		= 13,
	V4L2_MPEG_VIDEO_H264_PROFILE_SCALABLE_HIGH_INTRA	= 14,
	V4L2_MPEG_VIDEO_H264_PROFILE_STEREO_HIGH		= 15,
	V4L2_MPEG_VIDEO_H264_PROFILE_MULTIVIEW_HIGH		= 16,
	V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_HIGH		= 17,
};
#define V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_HEIGHT	(V4L2_CID_MPEG_BASE+364)
#define V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_WIDTH	(V4L2_CID_MPEG_BASE+365)
#define V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_ENABLE		(V4L2_CID_MPEG_BASE+366)
#define V4L2_CID_MPEG_VIDEO_H264_VUI_SAR_IDC		(V4L2_CID_MPEG_BASE+367)
enum v4l2_mpeg_video_h264_vui_sar_idc {
	V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_UNSPECIFIED	= 0,
	V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_1x1		= 1,
	V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_12x11		= 2,
	V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_10x11		= 3,
	V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_16x11		= 4,
	V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_40x33		= 5,
	V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_24x11		= 6,
	V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_20x11		= 7,
	V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_32x11		= 8,
	V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_80x33		= 9,
	V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_18x11		= 10,
	V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_15x11		= 11,
	V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_64x33		= 12,
	V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_160x99		= 13,
	V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_4x3		= 14,
	V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_3x2		= 15,
	V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_2x1		= 16,
	V4L2_MPEG_VIDEO_H264_VUI_SAR_IDC_EXTENDED	= 17,
};
#define V4L2_CID_MPEG_VIDEO_MPEG4_I_FRAME_QP	(V4L2_CID_MPEG_BASE+400)
#define V4L2_CID_MPEG_VIDEO_MPEG4_P_FRAME_QP	(V4L2_CID_MPEG_BASE+401)
#define V4L2_CID_MPEG_VIDEO_MPEG4_B_FRAME_QP	(V4L2_CID_MPEG_BASE+402)
#define V4L2_CID_MPEG_VIDEO_MPEG4_MIN_QP	(V4L2_CID_MPEG_BASE+403)
#define V4L2_CID_MPEG_VIDEO_MPEG4_MAX_QP	(V4L2_CID_MPEG_BASE+404)
#define V4L2_CID_MPEG_VIDEO_MPEG4_LEVEL		(V4L2_CID_MPEG_BASE+405)
enum v4l2_mpeg_video_mpeg4_level {
	V4L2_MPEG_VIDEO_MPEG4_LEVEL_0	= 0,
	V4L2_MPEG_VIDEO_MPEG4_LEVEL_0B	= 1,
	V4L2_MPEG_VIDEO_MPEG4_LEVEL_1	= 2,
	V4L2_MPEG_VIDEO_MPEG4_LEVEL_2	= 3,
	V4L2_MPEG_VIDEO_MPEG4_LEVEL_3	= 4,
	V4L2_MPEG_VIDEO_MPEG4_LEVEL_3B	= 5,
	V4L2_MPEG_VIDEO_MPEG4_LEVEL_4	= 6,
	V4L2_MPEG_VIDEO_MPEG4_LEVEL_5	= 7,
};
#define V4L2_CID_MPEG_VIDEO_MPEG4_PROFILE	(V4L2_CID_MPEG_BASE+406)
enum v4l2_mpeg_video_mpeg4_profile {
	V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE				= 0,
	V4L2_MPEG_VIDEO_MPEG4_PROFILE_ADVANCED_SIMPLE			= 1,
	V4L2_MPEG_VIDEO_MPEG4_PROFILE_CORE				= 2,
	V4L2_MPEG_VIDEO_MPEG4_PROFILE_SIMPLE_SCALABLE			= 3,
	V4L2_MPEG_VIDEO_MPEG4_PROFILE_ADVANCED_CODING_EFFICIENCY	= 4,
};
#define V4L2_CID_MPEG_VIDEO_MPEG4_QPEL		(V4L2_CID_MPEG_BASE+407)
#define V4L2_CID_QCOM_VIDEO_SYNC_FRAME_SEQ_HDR		(V4L2_CID_MPEG_BASE+408)

#define V4L2_CID_MPEG_CX2341X_BASE 				(V4L2_CTRL_CLASS_MPEG | 0x1000)
#define V4L2_CID_MPEG_CX2341X_VIDEO_SPATIAL_FILTER_MODE 	(V4L2_CID_MPEG_CX2341X_BASE+0)
enum v4l2_mpeg_cx2341x_video_spatial_filter_mode {
	V4L2_MPEG_CX2341X_VIDEO_SPATIAL_FILTER_MODE_MANUAL = 0,
	V4L2_MPEG_CX2341X_VIDEO_SPATIAL_FILTER_MODE_AUTO   = 1,
};
#define V4L2_CID_MPEG_CX2341X_VIDEO_SPATIAL_FILTER 		(V4L2_CID_MPEG_CX2341X_BASE+1)
#define V4L2_CID_MPEG_CX2341X_VIDEO_LUMA_SPATIAL_FILTER_TYPE 	(V4L2_CID_MPEG_CX2341X_BASE+2)
enum v4l2_mpeg_cx2341x_video_luma_spatial_filter_type {
	V4L2_MPEG_CX2341X_VIDEO_LUMA_SPATIAL_FILTER_TYPE_OFF                  = 0,
	V4L2_MPEG_CX2341X_VIDEO_LUMA_SPATIAL_FILTER_TYPE_1D_HOR               = 1,
	V4L2_MPEG_CX2341X_VIDEO_LUMA_SPATIAL_FILTER_TYPE_1D_VERT              = 2,
	V4L2_MPEG_CX2341X_VIDEO_LUMA_SPATIAL_FILTER_TYPE_2D_HV_SEPARABLE      = 3,
	V4L2_MPEG_CX2341X_VIDEO_LUMA_SPATIAL_FILTER_TYPE_2D_SYM_NON_SEPARABLE = 4,
};
#define V4L2_CID_MPEG_CX2341X_VIDEO_CHROMA_SPATIAL_FILTER_TYPE 	(V4L2_CID_MPEG_CX2341X_BASE+3)
enum v4l2_mpeg_cx2341x_video_chroma_spatial_filter_type {
	V4L2_MPEG_CX2341X_VIDEO_CHROMA_SPATIAL_FILTER_TYPE_OFF    = 0,
	V4L2_MPEG_CX2341X_VIDEO_CHROMA_SPATIAL_FILTER_TYPE_1D_HOR = 1,
};
#define V4L2_CID_MPEG_CX2341X_VIDEO_TEMPORAL_FILTER_MODE 	(V4L2_CID_MPEG_CX2341X_BASE+4)
enum v4l2_mpeg_cx2341x_video_temporal_filter_mode {
	V4L2_MPEG_CX2341X_VIDEO_TEMPORAL_FILTER_MODE_MANUAL = 0,
	V4L2_MPEG_CX2341X_VIDEO_TEMPORAL_FILTER_MODE_AUTO   = 1,
};
#define V4L2_CID_MPEG_CX2341X_VIDEO_TEMPORAL_FILTER 		(V4L2_CID_MPEG_CX2341X_BASE+5)
#define V4L2_CID_MPEG_CX2341X_VIDEO_MEDIAN_FILTER_TYPE 		(V4L2_CID_MPEG_CX2341X_BASE+6)
enum v4l2_mpeg_cx2341x_video_median_filter_type {
	V4L2_MPEG_CX2341X_VIDEO_MEDIAN_FILTER_TYPE_OFF      = 0,
	V4L2_MPEG_CX2341X_VIDEO_MEDIAN_FILTER_TYPE_HOR      = 1,
	V4L2_MPEG_CX2341X_VIDEO_MEDIAN_FILTER_TYPE_VERT     = 2,
	V4L2_MPEG_CX2341X_VIDEO_MEDIAN_FILTER_TYPE_HOR_VERT = 3,
	V4L2_MPEG_CX2341X_VIDEO_MEDIAN_FILTER_TYPE_DIAG     = 4,
};
#define V4L2_CID_MPEG_CX2341X_VIDEO_LUMA_MEDIAN_FILTER_BOTTOM 	(V4L2_CID_MPEG_CX2341X_BASE+7)
#define V4L2_CID_MPEG_CX2341X_VIDEO_LUMA_MEDIAN_FILTER_TOP 	(V4L2_CID_MPEG_CX2341X_BASE+8)
#define V4L2_CID_MPEG_CX2341X_VIDEO_CHROMA_MEDIAN_FILTER_BOTTOM	(V4L2_CID_MPEG_CX2341X_BASE+9)
#define V4L2_CID_MPEG_CX2341X_VIDEO_CHROMA_MEDIAN_FILTER_TOP 	(V4L2_CID_MPEG_CX2341X_BASE+10)
#define V4L2_CID_MPEG_CX2341X_STREAM_INSERT_NAV_PACKETS 	(V4L2_CID_MPEG_CX2341X_BASE+11)

#define V4L2_CID_MPEG_MFC51_BASE				(V4L2_CTRL_CLASS_MPEG | 0x1100)

#define V4L2_CID_MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY		(V4L2_CID_MPEG_MFC51_BASE+0)
#define V4L2_CID_MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY_ENABLE	(V4L2_CID_MPEG_MFC51_BASE+1)
#define V4L2_CID_MPEG_MFC51_VIDEO_FRAME_SKIP_MODE			(V4L2_CID_MPEG_MFC51_BASE+2)
enum v4l2_mpeg_mfc51_video_frame_skip_mode {
	V4L2_MPEG_MFC51_VIDEO_FRAME_SKIP_MODE_DISABLED		= 0,
	V4L2_MPEG_MFC51_VIDEO_FRAME_SKIP_MODE_LEVEL_LIMIT	= 1,
	V4L2_MPEG_MFC51_VIDEO_FRAME_SKIP_MODE_BUF_LIMIT		= 2,
};
#define V4L2_CID_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE			(V4L2_CID_MPEG_MFC51_BASE+3)
enum v4l2_mpeg_mfc51_video_force_frame_type {
	V4L2_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE_DISABLED		= 0,
	V4L2_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE_I_FRAME		= 1,
	V4L2_MPEG_MFC51_VIDEO_FORCE_FRAME_TYPE_NOT_CODED	= 2,
};
#define V4L2_CID_MPEG_MFC51_VIDEO_PADDING				(V4L2_CID_MPEG_MFC51_BASE+4)
#define V4L2_CID_MPEG_MFC51_VIDEO_PADDING_YUV				(V4L2_CID_MPEG_MFC51_BASE+5)
#define V4L2_CID_MPEG_MFC51_VIDEO_RC_FIXED_TARGET_BIT			(V4L2_CID_MPEG_MFC51_BASE+6)
#define V4L2_CID_MPEG_MFC51_VIDEO_RC_REACTION_COEFF			(V4L2_CID_MPEG_MFC51_BASE+7)
#define V4L2_CID_MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_ACTIVITY		(V4L2_CID_MPEG_MFC51_BASE+50)
#define V4L2_CID_MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_DARK			(V4L2_CID_MPEG_MFC51_BASE+51)
#define V4L2_CID_MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_SMOOTH		(V4L2_CID_MPEG_MFC51_BASE+52)
#define V4L2_CID_MPEG_MFC51_VIDEO_H264_ADAPTIVE_RC_STATIC		(V4L2_CID_MPEG_MFC51_BASE+53)
#define V4L2_CID_MPEG_MFC51_VIDEO_H264_NUM_REF_PIC_FOR_P		(V4L2_CID_MPEG_MFC51_BASE+54)

#define V4L2_CID_MPEG_MSM_VIDC_BASE		(V4L2_CTRL_CLASS_MPEG | 0x2000)

#define V4L2_CID_MPEG_VIDC_VIDEO_ENABLE_PICTURE_TYPE \
			(V4L2_CID_MPEG_MSM_VIDC_BASE+0)
#define V4L2_CID_MPEG_VIDC_VIDEO_KEEP_ASPECT_RATIO \
			(V4L2_CID_MPEG_MSM_VIDC_BASE+1)
#define V4L2_CID_MPEG_VIDC_VIDEO_POST_LOOP_DEBLOCKER_MODE \
			(V4L2_CID_MPEG_MSM_VIDC_BASE+2)
#define V4L2_CID_MPEG_VIDC_VIDEO_DIVX_FORMAT \
			(V4L2_CID_MPEG_MSM_VIDC_BASE+3)
enum v4l2_mpeg_vidc_video_divx_format_type {
	V4L2_MPEG_VIDC_VIDEO_DIVX_FORMAT_4		= 0,
	V4L2_MPEG_VIDC_VIDEO_DIVX_FORMAT_5		= 1,
	V4L2_MPEG_VIDC_VIDEO_DIVX_FORMAT_6	    = 2,
};
#define V4L2_CID_MPEG_VIDC_VIDEO_MB_ERROR_MAP_REPORTING	\
			(V4L2_CID_MPEG_MSM_VIDC_BASE+4)
#define V4L2_CID_MPEG_VIDC_VIDEO_CONTINUE_DATA_TRANSFER \
			(V4L2_CID_MPEG_MSM_VIDC_BASE+5)

#define V4L2_CID_MPEG_VIDC_VIDEO_STREAM_FORMAT   (V4L2_CID_MPEG_MSM_VIDC_BASE+6)
enum v4l2_mpeg_vidc_video_stream_format {
	V4L2_MPEG_VIDC_VIDEO_NAL_FORMAT_STARTCODES         = 0,
	V4L2_MPEG_VIDC_VIDEO_NAL_FORMAT_ONE_NAL_PER_BUFFER = 1,
	V4L2_MPEG_VIDC_VIDEO_NAL_FORMAT_ONE_BYTE_LENGTH    = 2,
	V4L2_MPEG_VIDC_VIDEO_NAL_FORMAT_TWO_BYTE_LENGTH    = 3,
	V4L2_MPEG_VIDC_VIDEO_NAL_FORMAT_FOUR_BYTE_LENGTH   = 4,
};

#define V4L2_CID_MPEG_VIDC_VIDEO_OUTPUT_ORDER   (V4L2_CID_MPEG_MSM_VIDC_BASE+7)
enum v4l2_mpeg_vidc_video_output_order {
	V4L2_MPEG_VIDC_VIDEO_OUTPUT_ORDER_DISPLAY         = 0,
	V4L2_MPEG_VIDC_VIDEO_OUTPUT_ORDER_DECODE          = 1,
};

#define V4L2_CID_MPEG_VIDC_VIDEO_FRAME_RATE   (V4L2_CID_MPEG_MSM_VIDC_BASE+8)
#define V4L2_CID_MPEG_VIDC_VIDEO_IDR_PERIOD   (V4L2_CID_MPEG_MSM_VIDC_BASE+9)
#define V4L2_CID_MPEG_VIDC_VIDEO_NUM_P_FRAMES (V4L2_CID_MPEG_MSM_VIDC_BASE+10)
#define V4L2_CID_MPEG_VIDC_VIDEO_NUM_B_FRAMES (V4L2_CID_MPEG_MSM_VIDC_BASE+11)
#define V4L2_CID_MPEG_VIDC_VIDEO_REQUEST_IFRAME (V4L2_CID_MPEG_MSM_VIDC_BASE+12)

#define V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL (V4L2_CID_MPEG_MSM_VIDC_BASE+13)
enum v4l2_mpeg_vidc_video_rate_control {
	V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_OFF = 0,
	V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_VBR_VFR = 1,
	V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_VBR_CFR = 2,
	V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_CBR_VFR = 3,
	V4L2_CID_MPEG_VIDC_VIDEO_RATE_CONTROL_CBR_CFR = 4,
};

#define V4L2_CID_MPEG_VIDC_VIDEO_ROTATION (V4L2_CID_MPEG_MSM_VIDC_BASE+14)
enum v4l2_mpeg_vidc_video_rotation {
	V4L2_CID_MPEG_VIDC_VIDEO_ROTATION_NONE = 0,
	V4L2_CID_MPEG_VIDC_VIDEO_ROTATION_90 = 1,
	V4L2_CID_MPEG_VIDC_VIDEO_ROTATION_180 = 2,
	V4L2_CID_MPEG_VIDC_VIDEO_ROTATION_270 = 3,
};
#define MSM_VIDC_BASE V4L2_CID_MPEG_MSM_VIDC_BASE
#define V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL (MSM_VIDC_BASE+15)
enum v4l2_mpeg_vidc_h264_cabac_model {
	V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL_0 = 0,
	V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL_1 = 1,
	V4L2_CID_MPEG_VIDC_VIDEO_H264_CABAC_MODEL_2 = 2,
};

#define V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_MODE (MSM_VIDC_BASE+16)
enum v4l2_mpeg_vidc_video_intra_refresh_mode {
	V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_NONE = 0,
	V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_CYCLIC = 1,
	V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_ADAPTIVE = 2,
	V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_CYCLIC_ADAPTIVE = 3,
	V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_RANDOM = 4,
};
#define V4L2_CID_MPEG_VIDC_VIDEO_AIR_MBS (V4L2_CID_MPEG_MSM_VIDC_BASE+17)
#define V4L2_CID_MPEG_VIDC_VIDEO_AIR_REF (V4L2_CID_MPEG_MSM_VIDC_BASE+18)
#define V4L2_CID_MPEG_VIDC_VIDEO_CIR_MBS (V4L2_CID_MPEG_MSM_VIDC_BASE+19)

#define V4L2_CID_MPEG_VIDC_VIDEO_H263_PROFILE (V4L2_CID_MPEG_MSM_VIDC_BASE+20)
enum v4l2_mpeg_vidc_video_h263_profile {
	V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_BASELINE = 0,
	V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_H320CODING	= 1,
	V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_BACKWARDCOMPATIBLE = 2,
	V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_ISWV2 = 3,
	V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_ISWV3 = 4,
	V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_HIGHCOMPRESSION = 5,
	V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_INTERNET = 6,
	V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_INTERLACE = 7,
	V4L2_MPEG_VIDC_VIDEO_H263_PROFILE_HIGHLATENCY = 8,
};

#define V4L2_CID_MPEG_VIDC_VIDEO_H263_LEVEL (V4L2_CID_MPEG_MSM_VIDC_BASE+21)
enum v4l2_mpeg_vidc_video_h263_level {
	V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_1_0 = 0,
	V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_2_0 = 1,
	V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_3_0 = 2,
	V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_4_0 = 3,
	V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_4_5 = 4,
	V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_5_0 = 5,
	V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_6_0 = 6,
	V4L2_MPEG_VIDC_VIDEO_H263_LEVEL_7_0 = 7,
};

#define V4L2_CID_MPEG_VIDC_VIDEO_H264_AU_DELIMITER \
		(V4L2_CID_MPEG_MSM_VIDC_BASE + 22)
enum v4l2_mpeg_vidc_video_h264_au_delimiter {
	V4L2_MPEG_VIDC_VIDEO_H264_AU_DELIMITER_DISABLED = 0,
	V4L2_MPEG_VIDC_VIDEO_H264_AU_DELIMITER_ENABLED = 1
};
#define V4L2_CID_MPEG_VIDC_VIDEO_SYNC_FRAME_DECODE \
		(V4L2_CID_MPEG_MSM_VIDC_BASE + 23)
enum v4l2_mpeg_vidc_video_sync_frame_decode {
	V4L2_MPEG_VIDC_VIDEO_SYNC_FRAME_DECODE_DISABLE = 0,
	V4L2_MPEG_VIDC_VIDEO_SYNC_FRAME_DECODE_ENABLE = 1
};
#define V4L2_CID_MPEG_VIDC_VIDEO_SECURE (V4L2_CID_MPEG_MSM_VIDC_BASE+24)
#define V4L2_CID_MPEG_VIDC_VIDEO_EXTRADATA \
		(V4L2_CID_MPEG_MSM_VIDC_BASE + 25)
enum v4l2_mpeg_vidc_extradata {
	V4L2_MPEG_VIDC_EXTRADATA_NONE = 0,
	V4L2_MPEG_VIDC_EXTRADATA_MB_QUANTIZATION = 1,
	V4L2_MPEG_VIDC_EXTRADATA_INTERLACE_VIDEO = 2,
	V4L2_MPEG_VIDC_EXTRADATA_VC1_FRAMEDISP = 3,
	V4L2_MPEG_VIDC_EXTRADATA_VC1_SEQDISP = 4,
	V4L2_MPEG_VIDC_EXTRADATA_TIMESTAMP = 5,
	V4L2_MPEG_VIDC_EXTRADATA_S3D_FRAME_PACKING = 6,
	V4L2_MPEG_VIDC_EXTRADATA_FRAME_RATE = 7,
	V4L2_MPEG_VIDC_EXTRADATA_PANSCAN_WINDOW = 8,
	V4L2_MPEG_VIDC_EXTRADATA_RECOVERY_POINT_SEI = 9,
	V4L2_MPEG_VIDC_EXTRADATA_CLOSED_CAPTION_UD = 10,
	V4L2_MPEG_VIDC_EXTRADATA_AFD_UD = 11,
	V4L2_MPEG_VIDC_EXTRADATA_MULTISLICE_INFO = 12,
	V4L2_MPEG_VIDC_EXTRADATA_NUM_CONCEALED_MB = 13,
	V4L2_MPEG_VIDC_EXTRADATA_METADATA_FILLER = 14,
	V4L2_MPEG_VIDC_INDEX_EXTRADATA_INPUT_CROP = 15,
	V4L2_MPEG_VIDC_INDEX_EXTRADATA_DIGITAL_ZOOM = 16,
	V4L2_MPEG_VIDC_INDEX_EXTRADATA_ASPECT_RATIO = 17,
	V4L2_MPEG_VIDC_EXTRADATA_MPEG2_SEQDISP = 18,
	V4L2_MPEG_VIDC_EXTRADATA_FRAME_QP = 19,
	V4L2_MPEG_VIDC_EXTRADATA_FRAME_BITS_INFO = 20,
	V4L2_MPEG_VIDC_EXTRADATA_LTR = 21,
	V4L2_MPEG_VIDC_EXTRADATA_METADATA_MBI = 22,
	V4L2_MPEG_VIDC_EXTRADATA_STREAM_USERDATA = 23,
};

#define V4L2_CID_MPEG_VIDC_SET_PERF_LEVEL (V4L2_CID_MPEG_MSM_VIDC_BASE + 26)
enum v4l2_mpeg_vidc_perf_level {
	V4L2_CID_MPEG_VIDC_PERF_LEVEL_NOMINAL			= 0,
	V4L2_CID_MPEG_VIDC_PERF_LEVEL_PERFORMANCE		= 1,
	V4L2_CID_MPEG_VIDC_PERF_LEVEL_TURBO			= 2,
};

#define V4L2_CID_MPEG_VIDEO_MULTI_SLICE_GOB		\
		(V4L2_CID_MPEG_MSM_VIDC_BASE + 27)

#define V4L2_CID_MPEG_VIDEO_MULTI_SLICE_DELIVERY_MODE	\
	(V4L2_CID_MPEG_MSM_VIDC_BASE + 28)

#define V4L2_CID_MPEG_VIDC_VIDEO_H264_VUI_TIMING_INFO \
		(V4L2_CID_MPEG_MSM_VIDC_BASE + 29)
enum v4l2_mpeg_vidc_video_h264_vui_timing_info {
	V4L2_MPEG_VIDC_VIDEO_H264_VUI_TIMING_INFO_DISABLED = 0,
	V4L2_MPEG_VIDC_VIDEO_H264_VUI_TIMING_INFO_ENABLED = 1
};

#define V4L2_CID_MPEG_VIDC_VIDEO_ALLOC_MODE_INPUT	\
		(V4L2_CID_MPEG_MSM_VIDC_BASE+30)
#define V4L2_CID_MPEG_VIDC_VIDEO_ALLOC_MODE_OUTPUT	\
		 (V4L2_CID_MPEG_MSM_VIDC_BASE+31)
enum v4l2_mpeg_vidc_video_alloc_mode_type {
	V4L2_MPEG_VIDC_VIDEO_STATIC	= 0,
	V4L2_MPEG_VIDC_VIDEO_RING	= 1,
	V4L2_MPEG_VIDC_VIDEO_DYNAMIC	= 2,
};

#define V4L2_CID_MPEG_VIDC_VIDEO_FRAME_ASSEMBLY	\
		(V4L2_CID_MPEG_MSM_VIDC_BASE+32)
enum v4l2_mpeg_vidc_video_assembly {
	V4L2_MPEG_VIDC_FRAME_ASSEMBLY_DISABLE	= 0,
	V4L2_MPEG_VIDC_FRAME_ASSEMBLY_ENABLE	= 1,
};

#define V4L2_CID_MPEG_VIDC_VIDEO_VP8_PROFILE_LEVEL \
		(V4L2_CID_MPEG_MSM_VIDC_BASE+33)
enum v4l2_mpeg_vidc_video_vp8_profile_level {
	V4L2_MPEG_VIDC_VIDEO_VP8_UNUSED,
	V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_0,
	V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_1,
	V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_2,
	V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_3,
};

#define V4L2_CID_MPEG_VIDC_VIDEO_DEINTERLACE \
	(V4L2_CID_MPEG_MSM_VIDC_BASE + 34)
enum v4l2_mpeg_vidc_video_deinterlace {
	V4L2_CID_MPEG_VIDC_VIDEO_DEINTERLACE_DISABLED = 0,
	V4L2_CID_MPEG_VIDC_VIDEO_DEINTERLACE_ENABLED = 1
};
#define V4L2_CID_MPEG_VIDC_VIDEO_STREAM_OUTPUT_MODE \
		(V4L2_CID_MPEG_MSM_VIDC_BASE + 35)
enum v4l2_mpeg_vidc_video_decoder_multi_stream {
	V4L2_CID_MPEG_VIDC_VIDEO_STREAM_OUTPUT_PRIMARY = 0,
	V4L2_CID_MPEG_VIDC_VIDEO_STREAM_OUTPUT_SECONDARY = 1,
};

#define V4L2_CID_MPEG_VIDC_VIDEO_VP8_MIN_QP (V4L2_CID_MPEG_MSM_VIDC_BASE + 36)
#define V4L2_CID_MPEG_VIDC_VIDEO_VP8_MAX_QP (V4L2_CID_MPEG_MSM_VIDC_BASE + 37)
#define V4L2_CID_MPEG_VIDC_VIDEO_CONCEAL_COLOR \
		(V4L2_CID_MPEG_MSM_VIDC_BASE + 38)

#define V4L2_CID_MPEG_VIDC_VIDEO_LTRMODE \
		(V4L2_CID_MPEG_MSM_VIDC_BASE + 39)

enum v4l2_mpeg_vidc_video_ltrmode {
	V4L2_MPEG_VIDC_VIDEO_LTR_MODE_DISABLE = 0,
	V4L2_MPEG_VIDC_VIDEO_LTR_MODE_MANUAL = 1,
	V4L2_MPEG_VIDC_VIDEO_LTR_MODE_PERIODIC = 2
};

#define V4L2_CID_MPEG_VIDC_VIDEO_LTRCOUNT \
		(V4L2_CID_MPEG_MSM_VIDC_BASE + 40)

#define V4L2_CID_MPEG_VIDC_VIDEO_USELTRFRAME \
		(V4L2_CID_MPEG_MSM_VIDC_BASE + 41)

#define V4L2_CID_MPEG_VIDC_VIDEO_MARKLTRFRAME \
		(V4L2_CID_MPEG_MSM_VIDC_BASE + 42)

#define V4L2_CID_MPEG_VIDC_VIDEO_HIER_P_NUM_LAYERS \
		(V4L2_CID_MPEG_MSM_VIDC_BASE + 43)

#define V4L2_CID_MPEG_VIDC_VIDEO_ENABLE_INITIAL_QP \
		(V4L2_CID_MPEG_MSM_VIDC_BASE + 44)

#define V4L2_CID_MPEG_VIDC_VIDEO_I_FRAME_QP \
		(V4L2_CID_MPEG_MSM_VIDC_BASE + 45)

#define V4L2_CID_MPEG_VIDC_VIDEO_P_FRAME_QP \
		(V4L2_CID_MPEG_MSM_VIDC_BASE + 46)

#define V4L2_CID_MPEG_VIDC_VIDEO_B_FRAME_QP \
		(V4L2_CID_MPEG_MSM_VIDC_BASE + 47)

#define V4L2_CID_MPEG_VIDC_VIDEO_BUFFER_SIZE_LIMIT \
		(V4L2_CID_MPEG_MSM_VIDC_BASE + 48)

#define V4L2_CID_CAMERA_CLASS_BASE 	(V4L2_CTRL_CLASS_CAMERA | 0x900)
#define V4L2_CID_CAMERA_CLASS 		(V4L2_CTRL_CLASS_CAMERA | 1)

#define V4L2_CID_EXPOSURE_AUTO			(V4L2_CID_CAMERA_CLASS_BASE+1)
enum  v4l2_exposure_auto_type {
	V4L2_EXPOSURE_AUTO = 0,
	V4L2_EXPOSURE_MANUAL = 1,
	V4L2_EXPOSURE_SHUTTER_PRIORITY = 2,
	V4L2_EXPOSURE_APERTURE_PRIORITY = 3
};
#define V4L2_CID_EXPOSURE_ABSOLUTE		(V4L2_CID_CAMERA_CLASS_BASE+2)
#define V4L2_CID_EXPOSURE_AUTO_PRIORITY		(V4L2_CID_CAMERA_CLASS_BASE+3)

#define V4L2_CID_PAN_RELATIVE			(V4L2_CID_CAMERA_CLASS_BASE+4)
#define V4L2_CID_TILT_RELATIVE			(V4L2_CID_CAMERA_CLASS_BASE+5)
#define V4L2_CID_PAN_RESET			(V4L2_CID_CAMERA_CLASS_BASE+6)
#define V4L2_CID_TILT_RESET			(V4L2_CID_CAMERA_CLASS_BASE+7)

#define V4L2_CID_PAN_ABSOLUTE			(V4L2_CID_CAMERA_CLASS_BASE+8)
#define V4L2_CID_TILT_ABSOLUTE			(V4L2_CID_CAMERA_CLASS_BASE+9)

#define V4L2_CID_FOCUS_ABSOLUTE			(V4L2_CID_CAMERA_CLASS_BASE+10)
#define V4L2_CID_FOCUS_RELATIVE			(V4L2_CID_CAMERA_CLASS_BASE+11)
#define V4L2_CID_FOCUS_AUTO			(V4L2_CID_CAMERA_CLASS_BASE+12)

#define V4L2_CID_ZOOM_ABSOLUTE			(V4L2_CID_CAMERA_CLASS_BASE+13)
#define V4L2_CID_ZOOM_RELATIVE			(V4L2_CID_CAMERA_CLASS_BASE+14)
#define V4L2_CID_ZOOM_CONTINUOUS		(V4L2_CID_CAMERA_CLASS_BASE+15)

#define V4L2_CID_PRIVACY			(V4L2_CID_CAMERA_CLASS_BASE+16)

#define V4L2_CID_IRIS_ABSOLUTE			(V4L2_CID_CAMERA_CLASS_BASE+17)
#define V4L2_CID_IRIS_RELATIVE			(V4L2_CID_CAMERA_CLASS_BASE+18)

#define V4L2_CID_FM_TX_CLASS_BASE		(V4L2_CTRL_CLASS_FM_TX | 0x900)
#define V4L2_CID_FM_TX_CLASS			(V4L2_CTRL_CLASS_FM_TX | 1)

#define V4L2_CID_RDS_TX_DEVIATION		(V4L2_CID_FM_TX_CLASS_BASE + 1)
#define V4L2_CID_RDS_TX_PI			(V4L2_CID_FM_TX_CLASS_BASE + 2)
#define V4L2_CID_RDS_TX_PTY			(V4L2_CID_FM_TX_CLASS_BASE + 3)
#define V4L2_CID_RDS_TX_PS_NAME			(V4L2_CID_FM_TX_CLASS_BASE + 5)
#define V4L2_CID_RDS_TX_RADIO_TEXT		(V4L2_CID_FM_TX_CLASS_BASE + 6)

#define V4L2_CID_AUDIO_LIMITER_ENABLED		(V4L2_CID_FM_TX_CLASS_BASE + 64)
#define V4L2_CID_AUDIO_LIMITER_RELEASE_TIME	(V4L2_CID_FM_TX_CLASS_BASE + 65)
#define V4L2_CID_AUDIO_LIMITER_DEVIATION	(V4L2_CID_FM_TX_CLASS_BASE + 66)

#define V4L2_CID_AUDIO_COMPRESSION_ENABLED	(V4L2_CID_FM_TX_CLASS_BASE + 80)
#define V4L2_CID_AUDIO_COMPRESSION_GAIN		(V4L2_CID_FM_TX_CLASS_BASE + 81)
#define V4L2_CID_AUDIO_COMPRESSION_THRESHOLD	(V4L2_CID_FM_TX_CLASS_BASE + 82)
#define V4L2_CID_AUDIO_COMPRESSION_ATTACK_TIME	(V4L2_CID_FM_TX_CLASS_BASE + 83)
#define V4L2_CID_AUDIO_COMPRESSION_RELEASE_TIME	(V4L2_CID_FM_TX_CLASS_BASE + 84)

#define V4L2_CID_PILOT_TONE_ENABLED		(V4L2_CID_FM_TX_CLASS_BASE + 96)
#define V4L2_CID_PILOT_TONE_DEVIATION		(V4L2_CID_FM_TX_CLASS_BASE + 97)
#define V4L2_CID_PILOT_TONE_FREQUENCY		(V4L2_CID_FM_TX_CLASS_BASE + 98)

#define V4L2_CID_TUNE_PREEMPHASIS		(V4L2_CID_FM_TX_CLASS_BASE + 112)
enum v4l2_preemphasis {
	V4L2_PREEMPHASIS_DISABLED	= 0,
	V4L2_PREEMPHASIS_50_uS		= 1,
	V4L2_PREEMPHASIS_75_uS		= 2,
};
#define V4L2_CID_TUNE_POWER_LEVEL		(V4L2_CID_FM_TX_CLASS_BASE + 113)
#define V4L2_CID_TUNE_ANTENNA_CAPACITOR		(V4L2_CID_FM_TX_CLASS_BASE + 114)

#define V4L2_CID_FLASH_CLASS_BASE		(V4L2_CTRL_CLASS_FLASH | 0x900)
#define V4L2_CID_FLASH_CLASS			(V4L2_CTRL_CLASS_FLASH | 1)

#define V4L2_CID_FLASH_LED_MODE			(V4L2_CID_FLASH_CLASS_BASE + 1)
enum v4l2_flash_led_mode {
	V4L2_FLASH_LED_MODE_NONE,
	V4L2_FLASH_LED_MODE_FLASH,
	V4L2_FLASH_LED_MODE_TORCH,
};

#define V4L2_CID_FLASH_STROBE_SOURCE		(V4L2_CID_FLASH_CLASS_BASE + 2)
enum v4l2_flash_strobe_source {
	V4L2_FLASH_STROBE_SOURCE_SOFTWARE,
	V4L2_FLASH_STROBE_SOURCE_EXTERNAL,
};

#define V4L2_CID_FLASH_STROBE			(V4L2_CID_FLASH_CLASS_BASE + 3)
#define V4L2_CID_FLASH_STROBE_STOP		(V4L2_CID_FLASH_CLASS_BASE + 4)
#define V4L2_CID_FLASH_STROBE_STATUS		(V4L2_CID_FLASH_CLASS_BASE + 5)

#define V4L2_CID_FLASH_TIMEOUT			(V4L2_CID_FLASH_CLASS_BASE + 6)
#define V4L2_CID_FLASH_INTENSITY		(V4L2_CID_FLASH_CLASS_BASE + 7)
#define V4L2_CID_FLASH_TORCH_INTENSITY		(V4L2_CID_FLASH_CLASS_BASE + 8)
#define V4L2_CID_FLASH_INDICATOR_INTENSITY	(V4L2_CID_FLASH_CLASS_BASE + 9)

#define V4L2_CID_FLASH_FAULT			(V4L2_CID_FLASH_CLASS_BASE + 10)
#define V4L2_FLASH_FAULT_OVER_VOLTAGE		(1 << 0)
#define V4L2_FLASH_FAULT_TIMEOUT		(1 << 1)
#define V4L2_FLASH_FAULT_OVER_TEMPERATURE	(1 << 2)
#define V4L2_FLASH_FAULT_SHORT_CIRCUIT		(1 << 3)
#define V4L2_FLASH_FAULT_OVER_CURRENT		(1 << 4)
#define V4L2_FLASH_FAULT_INDICATOR		(1 << 5)

#define V4L2_CID_FLASH_CHARGE			(V4L2_CID_FLASH_CLASS_BASE + 11)
#define V4L2_CID_FLASH_READY			(V4L2_CID_FLASH_CLASS_BASE + 12)

#define V4L2_CID_JPEG_CLASS_BASE		(V4L2_CTRL_CLASS_JPEG | 0x900)
#define V4L2_CID_JPEG_CLASS			(V4L2_CTRL_CLASS_JPEG | 1)

#define	V4L2_CID_JPEG_CHROMA_SUBSAMPLING	(V4L2_CID_JPEG_CLASS_BASE + 1)
enum v4l2_jpeg_chroma_subsampling {
	V4L2_JPEG_CHROMA_SUBSAMPLING_444	= 0,
	V4L2_JPEG_CHROMA_SUBSAMPLING_422	= 1,
	V4L2_JPEG_CHROMA_SUBSAMPLING_420	= 2,
	V4L2_JPEG_CHROMA_SUBSAMPLING_411	= 3,
	V4L2_JPEG_CHROMA_SUBSAMPLING_410	= 4,
	V4L2_JPEG_CHROMA_SUBSAMPLING_GRAY	= 5,
};
#define	V4L2_CID_JPEG_RESTART_INTERVAL		(V4L2_CID_JPEG_CLASS_BASE + 2)
#define	V4L2_CID_JPEG_COMPRESSION_QUALITY	(V4L2_CID_JPEG_CLASS_BASE + 3)

#define	V4L2_CID_JPEG_ACTIVE_MARKER		(V4L2_CID_JPEG_CLASS_BASE + 4)
#define	V4L2_JPEG_ACTIVE_MARKER_APP0		(1 << 0)
#define	V4L2_JPEG_ACTIVE_MARKER_APP1		(1 << 1)
#define	V4L2_JPEG_ACTIVE_MARKER_COM		(1 << 16)
#define	V4L2_JPEG_ACTIVE_MARKER_DQT		(1 << 17)
#define	V4L2_JPEG_ACTIVE_MARKER_DHT		(1 << 18)

struct v4l2_tuner {
	__u32                   index;
	__u8			name[32];
	enum v4l2_tuner_type    type;
	__u32			capability;
	__u32			rangelow;
	__u32			rangehigh;
	__u32			rxsubchans;
	__u32			audmode;
	__s32			signal;
	__s32			afc;
	__u32			reserved[4];
};

struct v4l2_modulator {
	__u32			index;
	__u8			name[32];
	__u32			capability;
	__u32			rangelow;
	__u32			rangehigh;
	__u32			txsubchans;
	__u32			reserved[4];
};

#define V4L2_TUNER_CAP_LOW		0x0001
#define V4L2_TUNER_CAP_NORM		0x0002
#define V4L2_TUNER_CAP_STEREO		0x0010
#define V4L2_TUNER_CAP_LANG2		0x0020
#define V4L2_TUNER_CAP_SAP		0x0020
#define V4L2_TUNER_CAP_LANG1		0x0040
#define V4L2_TUNER_CAP_RDS		0x0080
#define V4L2_TUNER_CAP_RDS_BLOCK_IO	0x0100
#define V4L2_TUNER_CAP_RDS_CONTROLS	0x0200

#define V4L2_TUNER_SUB_MONO		0x0001
#define V4L2_TUNER_SUB_STEREO		0x0002
#define V4L2_TUNER_SUB_LANG2		0x0004
#define V4L2_TUNER_SUB_SAP		0x0004
#define V4L2_TUNER_SUB_LANG1		0x0008
#define V4L2_TUNER_SUB_RDS		0x0010

#define V4L2_TUNER_MODE_MONO		0x0000
#define V4L2_TUNER_MODE_STEREO		0x0001
#define V4L2_TUNER_MODE_LANG2		0x0002
#define V4L2_TUNER_MODE_SAP		0x0002
#define V4L2_TUNER_MODE_LANG1		0x0003
#define V4L2_TUNER_MODE_LANG1_LANG2	0x0004

struct v4l2_frequency {
	__u32		      tuner;
	enum v4l2_tuner_type  type;
	__u32		      frequency;
	__u32		      reserved[8];
};

struct v4l2_hw_freq_seek {
	__u32		      tuner;
	enum v4l2_tuner_type  type;
	__u32		      seek_upward;
	__u32		      wrap_around;
	__u32		      spacing;
	__u32		      reserved[7];
};


struct v4l2_rds_data {
	__u8 	lsb;
	__u8 	msb;
	__u8 	block;
} __attribute__ ((packed));

#define V4L2_RDS_BLOCK_MSK 	 0x7
#define V4L2_RDS_BLOCK_A 	 0
#define V4L2_RDS_BLOCK_B 	 1
#define V4L2_RDS_BLOCK_C 	 2
#define V4L2_RDS_BLOCK_D 	 3
#define V4L2_RDS_BLOCK_C_ALT 	 4
#define V4L2_RDS_BLOCK_INVALID 	 7

#define V4L2_RDS_BLOCK_CORRECTED 0x40
#define V4L2_RDS_BLOCK_ERROR 	 0x80

struct v4l2_audio {
	__u32	index;
	__u8	name[32];
	__u32	capability;
	__u32	mode;
	__u32	reserved[2];
};

#define V4L2_AUDCAP_STEREO		0x00001
#define V4L2_AUDCAP_AVL			0x00002

#define V4L2_AUDMODE_AVL		0x00001

struct v4l2_audioout {
	__u32	index;
	__u8	name[32];
	__u32	capability;
	__u32	mode;
	__u32	reserved[2];
};

#if 1
#define V4L2_ENC_IDX_FRAME_I    (0)
#define V4L2_ENC_IDX_FRAME_P    (1)
#define V4L2_ENC_IDX_FRAME_B    (2)
#define V4L2_ENC_IDX_FRAME_MASK (0xf)

struct v4l2_enc_idx_entry {
	__u64 offset;
	__u64 pts;
	__u32 length;
	__u32 flags;
	__u32 reserved[2];
};

#define V4L2_ENC_IDX_ENTRIES (64)
struct v4l2_enc_idx {
	__u32 entries;
	__u32 entries_cap;
	__u32 reserved[4];
	struct v4l2_enc_idx_entry entry[V4L2_ENC_IDX_ENTRIES];
};


#define V4L2_ENC_CMD_START      (0)
#define V4L2_ENC_CMD_STOP       (1)
#define V4L2_ENC_CMD_PAUSE      (2)
#define V4L2_ENC_CMD_RESUME     (3)
#define V4L2_ENC_QCOM_CMD_FLUSH  (4)

#define V4L2_ENC_CMD_STOP_AT_GOP_END    (1 << 0)

struct v4l2_encoder_cmd {
	__u32 cmd;
	__u32 flags;
	union {
		struct {
			__u32 data[8];
		} raw;
	};
};

#define V4L2_DEC_CMD_START       (0)
#define V4L2_DEC_CMD_STOP        (1)
#define V4L2_DEC_CMD_PAUSE       (2)
#define V4L2_DEC_CMD_RESUME      (3)
#define V4L2_DEC_QCOM_CMD_FLUSH  (4)

#define V4L2_DEC_CMD_START_MUTE_AUDIO	(1 << 0)

#define V4L2_DEC_CMD_PAUSE_TO_BLACK	(1 << 0)

#define V4L2_DEC_CMD_STOP_TO_BLACK	(1 << 0)
#define V4L2_DEC_CMD_STOP_IMMEDIATELY	(1 << 1)

#define V4L2_DEC_QCOM_CMD_FLUSH_OUTPUT  (1 << 0)
#define V4L2_DEC_QCOM_CMD_FLUSH_CAPTURE (1 << 1)

#define V4L2_QCOM_CMD_FLUSH_OUTPUT  (1 << 0)
#define V4L2_QCOM_CMD_FLUSH_CAPTURE (1 << 1)


#define V4L2_DEC_START_FMT_NONE		(0)
#define V4L2_DEC_START_FMT_GOP		(1)

struct v4l2_decoder_cmd {
	__u32 cmd;
	__u32 flags;
	union {
		struct {
			__u64 pts;
		} stop;

		struct {
			__s32 speed;
			__u32 format;
		} start;

		struct {
			__u32 data[16];
		} raw;
	};
};
#endif



struct v4l2_vbi_format {
	__u32	sampling_rate;		
	__u32	offset;
	__u32	samples_per_line;
	__u32	sample_format;		
	__s32	start[2];
	__u32	count[2];
	__u32	flags;			
	__u32	reserved[2];		
};

#define V4L2_VBI_UNSYNC		(1 << 0)
#define V4L2_VBI_INTERLACED	(1 << 1)


struct v4l2_sliced_vbi_format {
	__u16   service_set;
	__u16   service_lines[2][24];
	__u32   io_size;
	__u32   reserved[2];            
};

#define V4L2_SLICED_TELETEXT_B          (0x0001)
#define V4L2_SLICED_VPS                 (0x0400)
#define V4L2_SLICED_CAPTION_525         (0x1000)
#define V4L2_SLICED_WSS_625             (0x4000)

#define V4L2_SLICED_VBI_525             (V4L2_SLICED_CAPTION_525)
#define V4L2_SLICED_VBI_625             (V4L2_SLICED_TELETEXT_B | V4L2_SLICED_VPS | V4L2_SLICED_WSS_625)

struct v4l2_sliced_vbi_cap {
	__u16   service_set;
	__u16   service_lines[2][24];
	enum v4l2_buf_type type;
	__u32   reserved[3];    
};

struct v4l2_sliced_vbi_data {
	__u32   id;
	__u32   field;          
	__u32   line;           
	__u32   reserved;       
	__u8    data[48];
};



#define V4L2_MPEG_VBI_IVTV_TELETEXT_B     (1)
#define V4L2_MPEG_VBI_IVTV_CAPTION_525    (4)
#define V4L2_MPEG_VBI_IVTV_WSS_625        (5)
#define V4L2_MPEG_VBI_IVTV_VPS            (7)

struct v4l2_mpeg_vbi_itv0_line {
	__u8 id;	
	__u8 data[42];	
} __attribute__ ((packed));

struct v4l2_mpeg_vbi_itv0 {
	__le32 linemask[2]; 
	struct v4l2_mpeg_vbi_itv0_line line[35];
} __attribute__ ((packed));

struct v4l2_mpeg_vbi_ITV0 {
	struct v4l2_mpeg_vbi_itv0_line line[36];
} __attribute__ ((packed));

#define V4L2_MPEG_VBI_IVTV_MAGIC0	"itv0"
#define V4L2_MPEG_VBI_IVTV_MAGIC1	"ITV0"

struct v4l2_mpeg_vbi_fmt_ivtv {
	__u8 magic[4];
	union {
		struct v4l2_mpeg_vbi_itv0 itv0;
		struct v4l2_mpeg_vbi_ITV0 ITV0;
	};
} __attribute__ ((packed));


struct v4l2_plane_pix_format {
	__u32		sizeimage;
	__u16		bytesperline;
	__u16		reserved[7];
} __attribute__ ((packed));

struct v4l2_pix_format_mplane {
	__u32				width;
	__u32				height;
	__u32				pixelformat;
	enum v4l2_field			field;
	enum v4l2_colorspace		colorspace;

	struct v4l2_plane_pix_format	plane_fmt[VIDEO_MAX_PLANES];
	__u8				num_planes;
	__u8				reserved[11];
} __attribute__ ((packed));

struct v4l2_format {
	enum v4l2_buf_type type;
	union {
		struct v4l2_pix_format		pix;     
		struct v4l2_pix_format_mplane	pix_mp;  
		struct v4l2_window		win;     
		struct v4l2_vbi_format		vbi;     
		struct v4l2_sliced_vbi_format	sliced;  
		__u8	raw_data[200];                   
	} fmt;
};

struct v4l2_streamparm {
	enum v4l2_buf_type type;
	union {
		struct v4l2_captureparm	capture;
		struct v4l2_outputparm	output;
		__u8	raw_data[200];  
	} parm;
};


#define V4L2_EVENT_ALL				0
#define V4L2_EVENT_VSYNC			1
#define V4L2_EVENT_EOS				2
#define V4L2_EVENT_CTRL				3
#define V4L2_EVENT_FRAME_SYNC			4
#define V4L2_EVENT_PRIVATE_START		0x08000000

#define V4L2_EVENT_MSM_VIDC_START	(V4L2_EVENT_PRIVATE_START + 0x00001000)
#define V4L2_EVENT_MSM_VIDC_FLUSH_DONE	(V4L2_EVENT_MSM_VIDC_START + 1)
#define V4L2_EVENT_MSM_VIDC_PORT_SETTINGS_CHANGED_SUFFICIENT	\
		(V4L2_EVENT_MSM_VIDC_START + 2)
#define V4L2_EVENT_MSM_VIDC_PORT_SETTINGS_CHANGED_INSUFFICIENT	\
		(V4L2_EVENT_MSM_VIDC_START + 3)
#define V4L2_EVENT_MSM_VIDC_CLOSE_DONE	(V4L2_EVENT_MSM_VIDC_START + 4)
#define V4L2_EVENT_MSM_VIDC_SYS_ERROR	(V4L2_EVENT_MSM_VIDC_START + 5)
#define V4L2_EVENT_MSM_VIDC_RELEASE_BUFFER_REFERENCE \
		(V4L2_EVENT_MSM_VIDC_START + 6)
#define V4L2_EVENT_MSM_VIDC_RELEASE_UNQUEUED_BUFFER \
		(V4L2_EVENT_MSM_VIDC_START + 7)
#define V4L2_EVENT_MSM_VIDC_HW_OVERLOAD (V4L2_EVENT_MSM_VIDC_START + 8)
#define V4L2_EVENT_MSM_VIDC_MAX_CLIENTS (V4L2_EVENT_MSM_VIDC_START + 9)

struct v4l2_event_vsync {
	
	__u8 field;
} __attribute__ ((packed));

#define V4L2_EVENT_CTRL_CH_VALUE		(1 << 0)
#define V4L2_EVENT_CTRL_CH_FLAGS		(1 << 1)

struct v4l2_event_ctrl {
	__u32 changes;
	__u32 type;
	union {
		__s32 value;
		__s64 value64;
	};
	__u32 flags;
	__s32 minimum;
	__s32 maximum;
	__s32 step;
	__s32 default_value;
};

struct v4l2_event_frame_sync {
	__u32 frame_sequence;
};

struct v4l2_event {
	__u32				type;
	union {
		struct v4l2_event_vsync		vsync;
		struct v4l2_event_ctrl		ctrl;
		struct v4l2_event_frame_sync	frame_sync;
		__u8				data[64];
	} u;
	__u32				pending;
	__u32				sequence;
	struct timespec			timestamp;
	__u32				id;
	__u32				reserved[8];
};

#define V4L2_EVENT_SUB_FL_SEND_INITIAL		(1 << 0)
#define V4L2_EVENT_SUB_FL_ALLOW_FEEDBACK	(1 << 1)

struct v4l2_event_subscription {
	__u32				type;
	__u32				id;
	__u32				flags;
	__u32				reserved[5];
};



#define V4L2_CHIP_MATCH_HOST       0  
#define V4L2_CHIP_MATCH_I2C_DRIVER 1  
#define V4L2_CHIP_MATCH_I2C_ADDR   2  
#define V4L2_CHIP_MATCH_AC97       3  

struct v4l2_dbg_match {
	__u32 type; 
	union {     
		__u32 addr;
		char name[32];
	};
} __attribute__ ((packed));

struct v4l2_dbg_register {
	struct v4l2_dbg_match match;
	__u32 size;	
	__u64 reg;
	__u64 val;
} __attribute__ ((packed));

struct v4l2_dbg_chip_ident {
	struct v4l2_dbg_match match;
	__u32 ident;       
	__u32 revision;    
} __attribute__ ((packed));

struct v4l2_create_buffers {
	__u32			index;
	__u32			count;
	enum v4l2_memory        memory;
	struct v4l2_format	format;
	__u32			reserved[8];
};

#define VIDIOC_QUERYCAP		 _IOR('V',  0, struct v4l2_capability)
#define VIDIOC_RESERVED		  _IO('V',  1)
#define VIDIOC_ENUM_FMT         _IOWR('V',  2, struct v4l2_fmtdesc)
#define VIDIOC_G_FMT		_IOWR('V',  4, struct v4l2_format)
#define VIDIOC_S_FMT		_IOWR('V',  5, struct v4l2_format)
#define VIDIOC_REQBUFS		_IOWR('V',  8, struct v4l2_requestbuffers)
#define VIDIOC_QUERYBUF		_IOWR('V',  9, struct v4l2_buffer)
#define VIDIOC_G_FBUF		 _IOR('V', 10, struct v4l2_framebuffer)
#define VIDIOC_S_FBUF		 _IOW('V', 11, struct v4l2_framebuffer)
#define VIDIOC_OVERLAY		 _IOW('V', 14, int)
#define VIDIOC_QBUF		_IOWR('V', 15, struct v4l2_buffer)
#define VIDIOC_DQBUF		_IOWR('V', 17, struct v4l2_buffer)
#define VIDIOC_STREAMON		 _IOW('V', 18, int)
#define VIDIOC_STREAMOFF	 _IOW('V', 19, int)
#define VIDIOC_G_PARM		_IOWR('V', 21, struct v4l2_streamparm)
#define VIDIOC_S_PARM		_IOWR('V', 22, struct v4l2_streamparm)
#define VIDIOC_G_STD		 _IOR('V', 23, v4l2_std_id)
#define VIDIOC_S_STD		 _IOW('V', 24, v4l2_std_id)
#define VIDIOC_ENUMSTD		_IOWR('V', 25, struct v4l2_standard)
#define VIDIOC_ENUMINPUT	_IOWR('V', 26, struct v4l2_input)
#define VIDIOC_G_CTRL		_IOWR('V', 27, struct v4l2_control)
#define VIDIOC_S_CTRL		_IOWR('V', 28, struct v4l2_control)
#define VIDIOC_G_TUNER		_IOWR('V', 29, struct v4l2_tuner)
#define VIDIOC_S_TUNER		 _IOW('V', 30, struct v4l2_tuner)
#define VIDIOC_G_AUDIO		 _IOR('V', 33, struct v4l2_audio)
#define VIDIOC_S_AUDIO		 _IOW('V', 34, struct v4l2_audio)
#define VIDIOC_QUERYCTRL	_IOWR('V', 36, struct v4l2_queryctrl)
#define VIDIOC_QUERYMENU	_IOWR('V', 37, struct v4l2_querymenu)
#define VIDIOC_G_INPUT		 _IOR('V', 38, int)
#define VIDIOC_S_INPUT		_IOWR('V', 39, int)
#define VIDIOC_G_OUTPUT		 _IOR('V', 46, int)
#define VIDIOC_S_OUTPUT		_IOWR('V', 47, int)
#define VIDIOC_ENUMOUTPUT	_IOWR('V', 48, struct v4l2_output)
#define VIDIOC_G_AUDOUT		 _IOR('V', 49, struct v4l2_audioout)
#define VIDIOC_S_AUDOUT		 _IOW('V', 50, struct v4l2_audioout)
#define VIDIOC_G_MODULATOR	_IOWR('V', 54, struct v4l2_modulator)
#define VIDIOC_S_MODULATOR	 _IOW('V', 55, struct v4l2_modulator)
#define VIDIOC_G_FREQUENCY	_IOWR('V', 56, struct v4l2_frequency)
#define VIDIOC_S_FREQUENCY	 _IOW('V', 57, struct v4l2_frequency)
#define VIDIOC_CROPCAP		_IOWR('V', 58, struct v4l2_cropcap)
#define VIDIOC_G_CROP		_IOWR('V', 59, struct v4l2_crop)
#define VIDIOC_S_CROP		 _IOW('V', 60, struct v4l2_crop)
#define VIDIOC_G_JPEGCOMP	 _IOR('V', 61, struct v4l2_jpegcompression)
#define VIDIOC_S_JPEGCOMP	 _IOW('V', 62, struct v4l2_jpegcompression)
#define VIDIOC_QUERYSTD      	 _IOR('V', 63, v4l2_std_id)
#define VIDIOC_TRY_FMT      	_IOWR('V', 64, struct v4l2_format)
#define VIDIOC_ENUMAUDIO	_IOWR('V', 65, struct v4l2_audio)
#define VIDIOC_ENUMAUDOUT	_IOWR('V', 66, struct v4l2_audioout)
#define VIDIOC_G_PRIORITY        _IOR('V', 67, enum v4l2_priority)
#define VIDIOC_S_PRIORITY        _IOW('V', 68, enum v4l2_priority)
#define VIDIOC_G_SLICED_VBI_CAP _IOWR('V', 69, struct v4l2_sliced_vbi_cap)
#define VIDIOC_LOG_STATUS         _IO('V', 70)
#define VIDIOC_G_EXT_CTRLS	_IOWR('V', 71, struct v4l2_ext_controls)
#define VIDIOC_S_EXT_CTRLS	_IOWR('V', 72, struct v4l2_ext_controls)
#define VIDIOC_TRY_EXT_CTRLS	_IOWR('V', 73, struct v4l2_ext_controls)
#if 1
#define VIDIOC_ENUM_FRAMESIZES	_IOWR('V', 74, struct v4l2_frmsizeenum)
#define VIDIOC_ENUM_FRAMEINTERVALS _IOWR('V', 75, struct v4l2_frmivalenum)
#define VIDIOC_G_ENC_INDEX       _IOR('V', 76, struct v4l2_enc_idx)
#define VIDIOC_ENCODER_CMD      _IOWR('V', 77, struct v4l2_encoder_cmd)
#define VIDIOC_TRY_ENCODER_CMD  _IOWR('V', 78, struct v4l2_encoder_cmd)
#endif

#if 1
#define	VIDIOC_DBG_S_REGISTER 	 _IOW('V', 79, struct v4l2_dbg_register)
#define	VIDIOC_DBG_G_REGISTER 	_IOWR('V', 80, struct v4l2_dbg_register)

#define VIDIOC_DBG_G_CHIP_IDENT _IOWR('V', 81, struct v4l2_dbg_chip_ident)
#endif

#define VIDIOC_S_HW_FREQ_SEEK	 _IOW('V', 82, struct v4l2_hw_freq_seek)
#define	VIDIOC_ENUM_DV_PRESETS	_IOWR('V', 83, struct v4l2_dv_enum_preset)
#define	VIDIOC_S_DV_PRESET	_IOWR('V', 84, struct v4l2_dv_preset)
#define	VIDIOC_G_DV_PRESET	_IOWR('V', 85, struct v4l2_dv_preset)
#define	VIDIOC_QUERY_DV_PRESET	_IOR('V',  86, struct v4l2_dv_preset)
#define	VIDIOC_S_DV_TIMINGS	_IOWR('V', 87, struct v4l2_dv_timings)
#define	VIDIOC_G_DV_TIMINGS	_IOWR('V', 88, struct v4l2_dv_timings)
#define	VIDIOC_DQEVENT		 _IOR('V', 89, struct v4l2_event)
#define	VIDIOC_SUBSCRIBE_EVENT	 _IOW('V', 90, struct v4l2_event_subscription)
#define	VIDIOC_UNSUBSCRIBE_EVENT _IOW('V', 91, struct v4l2_event_subscription)

#define VIDIOC_CREATE_BUFS	_IOWR('V', 92, struct v4l2_create_buffers)
#define VIDIOC_PREPARE_BUF	_IOWR('V', 93, struct v4l2_buffer)

#define VIDIOC_G_SELECTION	_IOWR('V', 94, struct v4l2_selection)
#define VIDIOC_S_SELECTION	_IOWR('V', 95, struct v4l2_selection)

#define VIDIOC_DECODER_CMD	_IOWR('V', 96, struct v4l2_decoder_cmd)
#define VIDIOC_TRY_DECODER_CMD	_IOWR('V', 97, struct v4l2_decoder_cmd)

#define VIDIOC_HTC_SET_CALLPIDNAME _IOW('V', 99, struct htc_callingpid_data)


#define BASE_VIDIOC_PRIVATE	192		

#endif 
