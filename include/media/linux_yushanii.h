/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, and the entire permission notice in its entirety,
 *    including the disclaimer of warranties.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * ALTERNATIVELY, this product may be distributed under the terms of
 * the GNU General Public License, version 2, in which case the provisions
 * of the GPL version 2 are required INSTEAD OF the BSD license.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ALL OF
 * WHICH ARE HEREBY DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF NOT ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#ifndef __LINUX_YUSHANII_H
#define __LINUX_YUSHANII_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define YUSHANII_IOCTL_MAGIC 'y'

#define YUSHANII_GET_INT \
	_IOR(YUSHANII_IOCTL_MAGIC, 1, struct yushanii_stats_event_ctrl *)

#define YUSHANII_GET_LOGN_GLACE \
	_IOR(YUSHANII_IOCTL_MAGIC, 2, struct GlaceStatsData *)

#define YUSHANII_GET_LOGN_HIST \
	_IOR(YUSHANII_IOCTL_MAGIC, 3, struct HistStatsData *)

#define YUSHANII_GET_SHORT_GLACE \
	_IOR(YUSHANII_IOCTL_MAGIC, 4, struct GlaceStatsData *)

#define YUSHANII_GET_SHORT_HIST \
	_IOR(YUSHANII_IOCTL_MAGIC, 5, struct HistStatsData *)

#define YUSHANII_SET_CHANNEL_OFFSET \
	_IOR(YUSHANII_IOCTL_MAGIC, 6, int *)

#define YUSHANII_SET_TONE_MAPPING \
	_IOR(YUSHANII_IOCTL_MAGIC, 7, int *)

#define YUSHANII_SET_DISABLE_DEFCOR \
	_IOR(YUSHANII_IOCTL_MAGIC, 8, int *)

#define YUSHANII_SET_CLS \
	_IOR(YUSHANII_IOCTL_MAGIC, 9, int *)

#define YUSHANII_SET_EXP \
	_IOR(YUSHANII_IOCTL_MAGIC, 10, struct yushanii_exposure *)

#define YUSHANII_SET_HDR_MERGE \
	_IOR(YUSHANII_IOCTL_MAGIC, 11, struct yushanii_hdr_merge *)

#define YUSHANII_SET_GLACE_CONFIG \
	_IOR(YUSHANII_IOCTL_MAGIC, 12, struct yushanii_glace_config *)

#define YUSHANII_SET_HDR_MERGE_MODE \
	_IOR(YUSHANII_IOCTL_MAGIC, 13, struct yushanii_hdr_merge_mode *)

#define YUSHANII_SET_HDR_FACTOR \
       _IOR(YUSHANII_IOCTL_MAGIC, 14, uint8_t *)

#define YUSHANII_SET_DEFCOR_LEVEL \
       _IOR(YUSHANII_IOCTL_MAGIC, 15, uint8_t *)

/*TODO:interrupt define should only in Ilp0100_ST_definition.c*/
/*yushanII interrupt*/
/*INT0*/
#define SPI_COMMS_READY  				0x00000001
#define IDLE_COMPLETE  					0x00000002
#define ISP_STREAMING  					0x00000004
#define MODE_CHANGE_COMPLETE	 		0x00000008
#define START_OF_FRAME  				0x00000020
#define END_OF_FRAME  					0x00000040
#define LONGEXP_GLACE_STATS_READY 	0x00000100
#define LONGEXP_HISTOGRAM_STATS_READY  0x00000200
#define SHORTEXP_GLACE_STATS_READY  	0x00010000
#define SHORTEXP_HISTOGRAM_STATS_READY  0x00020000
#define ITPOINT_LONG  					0x00200000
#define ITPOINT_SHORT_OR_NORMAL 		0x00400000
#define ITPOINT_MERGED  				0x00800000

//#define RXPHY_ULPM_ENTER  	N/A
//#define RXPHY_ULPM_EXIT  	N/A
/*INT1 error status*/
#define RXPHY_ERROR  		0x00000001
#define CSI2RX_ERROR  		0x00000002
#define SMIA_UNPACK_ERROR  0x00000004
#define P2W_FIFO_ERROR  	0x00000008
#define LTYPE_RETAG_ERROR  0x00000010
#define HDR_MERGE_ERROR  	0x00000020
#define TONE_MAP_ERROR  	0x00000040
#define MISC_SYS_ERROR  	0x00000080
#define CSI2TX_ERROR  		0x00000100
#define TXPHY_ERROR  		0x00000200
#define INCORRECT_WORD_COUNT  0x00000400

typedef struct{
  unsigned char Enable;
  uint16_t RoiHStart;
  uint16_t RoiVStart;
  uint16_t RoiHBlockSize;      //Block horizontal size, even, max 128
  uint16_t RoiVBlockSize;      //Block vertical size, even, max 128
  uint8_t RoiHNumberOfBlocks; //Number of blocks, min=2, max=8
  uint8_t RoiVNumberOfBlocks; //Number of blocks, min=1, max=6
  uint8_t SaturationLevelRed;
  uint8_t SaturationLevelGreen;
  uint8_t SaturationLevelBlue;
} GlaceConfig_t;

struct yushanii_glace_config {
	GlaceConfig_t long_glace_config;
	GlaceConfig_t short_glace_config;
};

typedef struct{
	uint8_t 	GlaceStatsRedMean[48];
	uint8_t 	GlaceStatsGreenMean[48];
	uint8_t 	GlaceStatsBlueMean[48];
	uint32_t    GlaceStatsNbOfSaturatedPixels[48];
} GlaceStatsData;


typedef struct{
	uint32_t	HistStatsRedBin[64];
	uint32_t 	HistStatsGreenBin[64];
	uint32_t	HistStatsBlueBin[64];
	uint16_t	HistStatsRedDarkestBin;
	uint32_t	HistStatsRedDarkestCount;
	uint16_t 	HistStatsRedBrightestBin;
	uint32_t 	HistStatsRedBrightestCount;
	uint16_t	HistStatsRedHighestBin;
	uint32_t	HistStatsRedHighestCount;
	uint16_t	HistStatsGreenDarkestBin;
	uint32_t	HistStatsGreenDarkestCount;
	uint16_t 	HistStatsGreenBrightestBin;
	uint32_t 	HistStatsGreenBrightestCount;
	uint16_t	HistStatsGreenHighestBin;
	uint32_t	HistStatsGreenHighestCount;
	uint16_t	HistStatsBlueDarkestBin;
	uint32_t	HistStatsBlueDarkestCount;
	uint16_t 	HistStatsBlueBrightestBin;
	uint32_t 	HistStatsBlueBrightestCount;
	uint16_t	HistStatsBlueHighestBin;
	uint32_t	HistStatsBlueHighestCount;
} HistStatsData;

struct yushanii_hist {	
	GlaceStatsData hist_glace_long;
	GlaceStatsData hist_glace_short;
	HistStatsData hist_long;
	HistStatsData hist_short;
};


struct yushanii_stats_event_ctrl {
	uint32_t type;
	uint32_t timeout_ms;
	uint32_t length;
	void *data;
};


struct yushanii_exposure{
	uint16_t long_exposure;
	uint16_t short_exposure;
	uint16_t AnalogGain_G;
	uint16_t AnalogGain_R;
	uint16_t AnalogGain_B;
	uint16_t DigitalShortGain_G;
	uint16_t DigitalShortGain_R;
	uint16_t DigitalShortGain_B;
	uint16_t DigitalLongGain_G;
	uint16_t DigitalLongGain_R;
	uint16_t DigitalLongGain_B;
};

struct yushanii_cls{
	int cls_enable;
	uint32_t color_temp;
};

typedef enum{
	KNEE_POINTS = 0,
	HDR_AVERAGE  = 1,
	AVERAGE_AND_KNEE_POINTS = 2
} HdrMergeMethod;

typedef enum{
	HDR_MAX_MACRO_PIXEL = 0,
	HDR_LUMA = 1
} HdrMergeImageCodes;

struct yushanii_hdr_merge{
	HdrMergeMethod method;
	HdrMergeImageCodes code;
};

typedef enum{
  HDR_ON = 0,
  HDR_OUTPUT_LONG_ONLY  = 1,
  HDR_OUTPUT_SHORT_ONLY = 2
} HDRMergeMode;

struct yushanii_hdr_merge_mode{
	HDRMergeMode Mode;
};


struct msm_camera_rawchip_info {
	int rawchip_reset;
	int rawchip_intr0;
	int rawchip_intr1;
	int (*camera_rawchip_power_on)(void);
	int (*camera_rawchip_power_off)(void);
	uint32_t raw_1v8_enable;
	uint32_t raw_1v2_enable;
	uint32_t raw_mclk;
};

typedef enum{
	DEFCOR_LEVEL_INVALID = -1,
	DEFCOR_LEVEL_0 = 0,
	DEFCOR_LEVEL_1 = 1,
	DEFCOR_LEVEL_2 = 2
} defcor_level_t;


#endif /* __LINUX_YUSHANII_H */

