/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 and
* only version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <sound/q6adm-v2.h>
#include <sound/q6asm-v2.h>
#include <sound/q6afe-v2.h>

#include "msm-dolby-dap-config.h"

/* dolby endp based parameters */
struct dolby_dap_endp_params_s {
	int device;
	int device_ch_caps;
	int dap_device;
	int params_id[DOLBY_NUM_ENDP_DEPENDENT_PARAMS];
	int params_len[DOLBY_NUM_ENDP_DEPENDENT_PARAMS];
	int params_offset[DOLBY_NUM_ENDP_DEPENDENT_PARAMS];
	int params_val[DOLBY_ENDDEP_PARAM_LENGTH];
};

const struct dolby_dap_endp_params_s
			dolby_dap_endp_params[NUM_DOLBY_ENDP_DEVICE] = {
	{EARPIECE, 2, DOLBY_ENDP_EXT_SPEAKERS,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-320, -320, 144}
	},
	{SPEAKER, 2, DOLBY_ENDP_INT_SPEAKERS,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-320, -320, 144}
	},
	{WIRED_HEADSET,	2, DOLBY_ENDP_HEADPHONES,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-320, -320, 144}
	},
	{WIRED_HEADPHONE, 2, DOLBY_ENDP_HEADPHONES,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-320, -320, 144}
	},
	{BLUETOOTH_SCO,	2, DOLBY_ENDP_EXT_SPEAKERS,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-320, -320, 144}
	},
	{BLUETOOTH_SCO_HEADSET,	2, DOLBY_ENDP_EXT_SPEAKERS,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-320, -320, 144}
	},
	{BLUETOOTH_SCO_CARKIT, 2, DOLBY_ENDP_EXT_SPEAKERS,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-320, -320, 144}
	},
	{BLUETOOTH_A2DP, 2, DOLBY_ENDP_EXT_SPEAKERS,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-320, -320, 144}
	},
	{BLUETOOTH_A2DP_HEADPHONES, 2, DOLBY_ENDP_HEADPHONES,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-320, -320, 144}
	},
	{BLUETOOTH_A2DP_SPEAKER, 2, DOLBY_ENDP_EXT_SPEAKERS,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-320, -320, 144}
	},
	{AUX_DIGITAL, 2, DOLBY_ENDP_HDMI,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-496, -496, 0}
	},
	{AUX_DIGITAL, 6, DOLBY_ENDP_HDMI,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-496, -496, 0}
	},
	{AUX_DIGITAL, 8, DOLBY_ENDP_HDMI,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-496, -496, 0}
	},
	{ANLG_DOCK_HEADSET, 2, DOLBY_ENDP_HEADPHONES,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-320, -320, 144}
	},
	{DGTL_DOCK_HEADSET, 2, DOLBY_ENDP_HEADPHONES,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-320, -320, 144}
	},
	{USB_ACCESSORY,	2, DOLBY_ENDP_EXT_SPEAKERS,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-320, -320, 144}
	},
	{USB_DEVICE, 2, DOLBY_ENDP_EXT_SPEAKERS,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-320, -320, 144}
	},
	{REMOTE_SUBMIX,	2, DOLBY_ENDP_HDMI,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-496, -496, 0}
	},
	{ANC_HEADSET, 2, DOLBY_ENDP_HEADPHONES,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-320, -320, 144}
	},
	{ANC_HEADPHONE,	2, DOLBY_ENDP_HEADPHONES,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-320, -320, 144}
	},
	{PROXY,	2, DOLBY_ENDP_EXT_SPEAKERS,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-320, -320, 144}
	},
	{PROXY,	6, DOLBY_ENDP_EXT_SPEAKERS,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-320, -320, 144}
	},
	{FM, 2, DOLBY_ENDP_HDMI,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-496, -496, 0}
	},
	{FM_TX,	2, DOLBY_ENDP_EXT_SPEAKERS,
		{DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_VMB},
		{DOLBY_ENDDEP_PARAM_DVLO_LENGTH, DOLBY_ENDDEP_PARAM_DVLI_LENGTH,
		 DOLBY_ENDDEP_PARAM_VMB_LENGTH},
		{DOLBY_ENDDEP_PARAM_DVLO_OFFSET, DOLBY_ENDDEP_PARAM_DVLI_OFFSET,
		 DOLBY_ENDDEP_PARAM_VMB_OFFSET},
		{-320, -320, 144}
	},
};

/* dolby param ids to/from dsp */
static uint32_t	dolby_dap_params_id[ALL_DOLBY_PARAMS] = {
	DOLBY_PARAM_ID_VDHE, DOLBY_PARAM_ID_VSPE, DOLBY_PARAM_ID_DSSF,
	DOLBY_PARAM_ID_DVLI, DOLBY_PARAM_ID_DVLO, DOLBY_PARAM_ID_DVLE,
	DOLBY_PARAM_ID_DVMC, DOLBY_PARAM_ID_DVME, DOLBY_PARAM_ID_IENB,
	DOLBY_PARAM_ID_IEBF, DOLBY_PARAM_ID_IEON, DOLBY_PARAM_ID_DEON,
	DOLBY_PARAM_ID_NGON, DOLBY_PARAM_ID_GEON, DOLBY_PARAM_ID_GENB,
	DOLBY_PARAM_ID_GEBF, DOLBY_PARAM_ID_AONB, DOLBY_PARAM_ID_AOBF,
	DOLBY_PARAM_ID_AOBG, DOLBY_PARAM_ID_AOON, DOLBY_PARAM_ID_ARNB,
	DOLBY_PARAM_ID_ARBF, DOLBY_PARAM_ID_PLB,  DOLBY_PARAM_ID_PLMD,
	DOLBY_PARAM_ID_DHSB, DOLBY_PARAM_ID_DHRG, DOLBY_PARAM_ID_DSSB,
	DOLBY_PARAM_ID_DSSA, DOLBY_PARAM_ID_DVLA, DOLBY_PARAM_ID_IEBT,
	DOLBY_PARAM_ID_IEA,  DOLBY_PARAM_ID_DEA,  DOLBY_PARAM_ID_DED,
	DOLBY_PARAM_ID_GEBG, DOLBY_PARAM_ID_AOCC, DOLBY_PARAM_ID_ARBI,
	DOLBY_PARAM_ID_ARBL, DOLBY_PARAM_ID_ARBH, DOLBY_PARAM_ID_AROD,
	DOLBY_PARAM_ID_ARTP, DOLBY_PARAM_ID_VMON, DOLBY_PARAM_ID_VMB,
	DOLBY_PARAM_ID_VCNB, DOLBY_PARAM_ID_VCBF, DOLBY_PARAM_ID_PREG,
	DOLBY_PARAM_ID_VEN,  DOLBY_PARAM_ID_PSTG, DOLBY_COMMIT_ALL_TO_DSP,
	DOLBY_COMMIT_TO_DSP, DOLBY_USE_CACHE, DOLBY_AUTO_ENDP,
	DOLBY_AUTO_ENDDEP_PARAMS
};

/* modifed state:	0x00000000 - Not updated
*			> 0x00000000 && < 0x00010000
*				Updated and not commited to DSP
*			0x00010001 - Updated and commited to DSP
*			> 0x00010001 - Modified the commited value
*/
static int dolby_dap_params_modified[MAX_DOLBY_PARAMS] = { 0 };
/* param offset */
static uint32_t	dolby_dap_params_offset[MAX_DOLBY_PARAMS] = {
	DOLBY_PARAM_VDHE_OFFSET, DOLBY_PARAM_VSPE_OFFSET,
	DOLBY_PARAM_DSSF_OFFSET, DOLBY_PARAM_DVLI_OFFSET,
	DOLBY_PARAM_DVLO_OFFSET, DOLBY_PARAM_DVLE_OFFSET,
	DOLBY_PARAM_DVMC_OFFSET, DOLBY_PARAM_DVME_OFFSET,
	DOLBY_PARAM_IENB_OFFSET, DOLBY_PARAM_IEBF_OFFSET,
	DOLBY_PARAM_IEON_OFFSET, DOLBY_PARAM_DEON_OFFSET,
	DOLBY_PARAM_NGON_OFFSET, DOLBY_PARAM_GEON_OFFSET,
	DOLBY_PARAM_GENB_OFFSET, DOLBY_PARAM_GEBF_OFFSET,
	DOLBY_PARAM_AONB_OFFSET, DOLBY_PARAM_AOBF_OFFSET,
	DOLBY_PARAM_AOBG_OFFSET, DOLBY_PARAM_AOON_OFFSET,
	DOLBY_PARAM_ARNB_OFFSET, DOLBY_PARAM_ARBF_OFFSET,
	DOLBY_PARAM_PLB_OFFSET,  DOLBY_PARAM_PLMD_OFFSET,
	DOLBY_PARAM_DHSB_OFFSET, DOLBY_PARAM_DHRG_OFFSET,
	DOLBY_PARAM_DSSB_OFFSET, DOLBY_PARAM_DSSA_OFFSET,
	DOLBY_PARAM_DVLA_OFFSET, DOLBY_PARAM_IEBT_OFFSET,
	DOLBY_PARAM_IEA_OFFSET,  DOLBY_PARAM_DEA_OFFSET,
	DOLBY_PARAM_DED_OFFSET,  DOLBY_PARAM_GEBG_OFFSET,
	DOLBY_PARAM_AOCC_OFFSET, DOLBY_PARAM_ARBI_OFFSET,
	DOLBY_PARAM_ARBL_OFFSET, DOLBY_PARAM_ARBH_OFFSET,
	DOLBY_PARAM_AROD_OFFSET, DOLBY_PARAM_ARTP_OFFSET,
	DOLBY_PARAM_VMON_OFFSET, DOLBY_PARAM_VMB_OFFSET,
	DOLBY_PARAM_VCNB_OFFSET, DOLBY_PARAM_VCBF_OFFSET,
	DOLBY_PARAM_PREG_OFFSET, DOLBY_PARAM_VEN_OFFSET,
	DOLBY_PARAM_PSTG_OFFSET
};
/* param_length */
static uint32_t	dolby_dap_params_length[MAX_DOLBY_PARAMS] = {
	DOLBY_PARAM_VDHE_LENGTH, DOLBY_PARAM_VSPE_LENGTH,
	DOLBY_PARAM_DSSF_LENGTH, DOLBY_PARAM_DVLI_LENGTH,
	DOLBY_PARAM_DVLO_LENGTH, DOLBY_PARAM_DVLE_LENGTH,
	DOLBY_PARAM_DVMC_LENGTH, DOLBY_PARAM_DVME_LENGTH,
	DOLBY_PARAM_IENB_LENGTH, DOLBY_PARAM_IEBF_LENGTH,
	DOLBY_PARAM_IEON_LENGTH, DOLBY_PARAM_DEON_LENGTH,
	DOLBY_PARAM_NGON_LENGTH, DOLBY_PARAM_GEON_LENGTH,
	DOLBY_PARAM_GENB_LENGTH, DOLBY_PARAM_GEBF_LENGTH,
	DOLBY_PARAM_AONB_LENGTH, DOLBY_PARAM_AOBF_LENGTH,
	DOLBY_PARAM_AOBG_LENGTH, DOLBY_PARAM_AOON_LENGTH,
	DOLBY_PARAM_ARNB_LENGTH, DOLBY_PARAM_ARBF_LENGTH,
	DOLBY_PARAM_PLB_LENGTH,  DOLBY_PARAM_PLMD_LENGTH,
	DOLBY_PARAM_DHSB_LENGTH, DOLBY_PARAM_DHRG_LENGTH,
	DOLBY_PARAM_DSSB_LENGTH, DOLBY_PARAM_DSSA_LENGTH,
	DOLBY_PARAM_DVLA_LENGTH, DOLBY_PARAM_IEBT_LENGTH,
	DOLBY_PARAM_IEA_LENGTH,  DOLBY_PARAM_DEA_LENGTH,
	DOLBY_PARAM_DED_LENGTH,  DOLBY_PARAM_GEBG_LENGTH,
	DOLBY_PARAM_AOCC_LENGTH, DOLBY_PARAM_ARBI_LENGTH,
	DOLBY_PARAM_ARBL_LENGTH, DOLBY_PARAM_ARBH_LENGTH,
	DOLBY_PARAM_AROD_LENGTH, DOLBY_PARAM_ARTP_LENGTH,
	DOLBY_PARAM_VMON_LENGTH, DOLBY_PARAM_VMB_LENGTH,
	DOLBY_PARAM_VCNB_LENGTH, DOLBY_PARAM_VCBF_LENGTH,
	DOLBY_PARAM_PREG_LENGTH, DOLBY_PARAM_VEN_LENGTH,
	DOLBY_PARAM_PSTG_LENGTH
};

/* param_value */
static uint32_t	dolby_dap_params_value[TOTAL_LENGTH_DOLBY_PARAM] = {0};

struct dolby_dap_params_get_s {
	int32_t  port_id;
	uint32_t device_id;
	uint32_t param_id;
	uint32_t offset;
	uint32_t length;
};

struct dolby_dap_params_states_s {
	bool use_cache;
	bool auto_endp;
	bool enddep_params;
	int  port_id;
	int  port_open_count;
	int  port_ids_dolby_can_be_enabled;
	int  device;
};

static struct dolby_dap_params_get_s dolby_dap_params_get = {-1, DEVICE_OUT_ALL,
								 0, 0, 0};
static struct dolby_dap_params_states_s dolby_dap_params_states = { true, true,
						true, DOLBY_INVALID_PORT_ID,
						0, DEVICE_OUT_ALL, 0 };
/*
port_ids_dolby_can_be_enabled is set to 0x7FFFFFFF.
this needs to be removed after interface validation
*/

static int map_device_to_dolby_endpoint(int device)
{
	int i, dolby_dap_device = DOLBY_ENDP_EXT_SPEAKERS;
	for (i = 0; i < NUM_DOLBY_ENDP_DEVICE; i++) {
		if (dolby_dap_endp_params[i].device == device) {
			dolby_dap_device = dolby_dap_endp_params[i].dap_device;
			break;
		}
	}
	/* default the endpoint to speaker if corresponding device entry */
	/* not found */
	if (i >= NUM_DOLBY_ENDP_DEVICE)
		dolby_dap_params_states.device = SPEAKER;
	return dolby_dap_device;
}

static int dolby_dap_send_end_point(int port_id)
{
	int rc = 0;
	char *params_value;
	int *update_params_value;
	uint32_t params_length = (DOLBY_PARAM_INT_ENDP_LENGTH +
				DOLBY_PARAM_PAYLOAD_SIZE) * sizeof(uint32_t);

	pr_debug("%s\n", __func__);
	params_value = kzalloc(params_length, GFP_KERNEL);
	if (!params_value) {
		pr_err("%s, params memory alloc failed", __func__);
		return -ENOMEM;
	}
	update_params_value = (int *)params_value;
	*update_params_value++ = DOLBY_BUNDLE_MODULE_ID;
	*update_params_value++ = DOLBY_PARAM_ID_INIT_ENDP;
	*update_params_value++ = DOLBY_PARAM_INT_ENDP_LENGTH * sizeof(uint32_t);
	*update_params_value++ =
		 map_device_to_dolby_endpoint(dolby_dap_params_states.device);
	rc = adm_dolby_dap_send_params(port_id, params_value, params_length);
	if (rc) {
		pr_err("%s: send dolby params failed\n", __func__);
		rc = -EINVAL;
	}
	kfree(params_value);
	return rc;
}

static int dolby_dap_send_enddep_params(int port_id, int device_channels)
{
	int i, j, rc = 0, idx, offset;
	char *params_value;
	int *update_params_value;
	uint32_t params_length = (DOLBY_ENDDEP_PARAM_LENGTH +
					DOLBY_NUM_ENDP_DEPENDENT_PARAMS *
					DOLBY_PARAM_PAYLOAD_SIZE) *
				sizeof(uint32_t);

	pr_debug("%s\n", __func__);
	params_value = kzalloc(params_length, GFP_KERNEL);
	if (!params_value) {
		pr_err("%s, params memory alloc failed", __func__);
		return -ENOMEM;
	}
	update_params_value = (int *)params_value;
	for (idx = 0; idx < NUM_DOLBY_ENDP_DEVICE; idx++) {
		if (dolby_dap_endp_params[idx].device ==
			dolby_dap_params_states.device) {
			if (dolby_dap_params_states.device == AUX_DIGITAL ||
			    dolby_dap_params_states.device == PROXY) {
				if (dolby_dap_endp_params[idx].device_ch_caps ==
					device_channels)
					break;
			} else {
				break;
			}
		}
	}
	if (idx >= NUM_DOLBY_ENDP_DEVICE) {
		pr_err("%s: device is not set accordingly\n", __func__);
		kfree(params_value);
		return -EINVAL;
	}
	for (i = 0; i < DOLBY_ENDDEP_PARAM_LENGTH; i++) {
		*update_params_value++ = DOLBY_BUNDLE_MODULE_ID;
		*update_params_value++ =
				dolby_dap_endp_params[idx].params_id[i];
		*update_params_value++ =
			dolby_dap_endp_params[idx].params_len[i] *
				sizeof(uint32_t);
		offset = dolby_dap_endp_params[idx].params_offset[i];
		for (j = 0; j < dolby_dap_endp_params[idx].params_len[i]; j++)
			*update_params_value++ =
				dolby_dap_endp_params[idx].params_val[offset+j];
	}
	rc = adm_dolby_dap_send_params(port_id, params_value, params_length);
	if (rc) {
		pr_err("%s: send dolby params failed\n", __func__);
		rc = -EINVAL;
	}
	kfree(params_value);
	return rc;
}

static int dolby_dap_send_cached_params(int port_id, int commit)
{
	char *params_value;
	int *update_params_value, rc = 0;
	uint32_t index_offset, i, j;
	uint32_t params_length = (TOTAL_LENGTH_DOLBY_PARAM +
				MAX_DOLBY_PARAMS * DOLBY_PARAM_PAYLOAD_SIZE) *
				sizeof(uint32_t);

	params_value = kzalloc(params_length, GFP_KERNEL);
	if (!params_value) {
		pr_err("%s, params memory alloc failed\n", __func__);
		return -ENOMEM;
	}
	update_params_value = (int *)params_value;
	params_length = 0;
	for (i = 0; i < MAX_DOLBY_PARAMS; i++) {
		if ((dolby_dap_params_modified[i] == 0) ||
		    ((commit) &&
		     ((dolby_dap_params_modified[i] & 0x00010000) &&
		     ((dolby_dap_params_modified[i] & 0x0000FFFF) <= 1))))
			continue;
		*update_params_value++ = DOLBY_BUNDLE_MODULE_ID;
		*update_params_value++ = dolby_dap_params_id[i];
		*update_params_value++ = dolby_dap_params_length[i] *
						sizeof(uint32_t);
		index_offset = dolby_dap_params_offset[i];
		for (j = 0; j < dolby_dap_params_length[i]; j++)
			*update_params_value++ =
				dolby_dap_params_value[index_offset+j];
		params_length += (DOLBY_PARAM_PAYLOAD_SIZE +
				dolby_dap_params_length[i]) * sizeof(uint32_t);
	}
	pr_debug("%s, valid param length: %d", __func__, params_length);
	if (params_length) {
		rc = adm_dolby_dap_send_params(port_id, params_value,
						params_length);
		if (rc) {
			pr_err("%s: send dolby params failed\n", __func__);
			kfree(params_value);
			return -EINVAL;
		}
		for (i = 0; i < MAX_DOLBY_PARAMS; i++) {
			if ((dolby_dap_params_modified[i] == 0) ||
			    ((commit) &&
			     ((dolby_dap_params_modified[i] & 0x00010000) &&
			     ((dolby_dap_params_modified[i] & 0x0000FFFF) <= 1))
			    ))
				continue;
			dolby_dap_params_modified[i] = 0x00010001;
		}
	}
	kfree(params_value);
	return 0;
}

int dolby_dap_init(int port_id, int channels)
{
	int ret = 0;
	if ((port_id != DOLBY_INVALID_PORT_ID) &&
		(port_id &
		 dolby_dap_params_states.port_ids_dolby_can_be_enabled)) {
		dolby_dap_params_states.port_id = port_id;
		dolby_dap_params_states.port_open_count++;
		if (dolby_dap_params_states.auto_endp) {
			ret = dolby_dap_send_end_point(port_id);
			if (ret) {
				pr_err("%s: err sending endppoint\n", __func__);
				return ret;
			}
		}
		if (dolby_dap_params_states.use_cache) {
			ret = dolby_dap_send_cached_params(port_id, 0);
			if (ret) {
				pr_err("%s: err sending cached params\n",
					__func__);
				return ret;
			}
		}
		if (dolby_dap_params_states.enddep_params) {
			dolby_dap_send_enddep_params(port_id,
				channels);
			if (ret) {
				pr_err("%s: err sending endp dependent params\n",
					__func__);
				return ret;
			}
		}
	}
	return ret;
}

void dolby_dap_deinit(int port_id)
{
	dolby_dap_params_states.port_open_count--;
	if ((dolby_dap_params_states.port_id == port_id) &&
		(!dolby_dap_params_states.port_open_count))
		dolby_dap_params_states.port_id = DOLBY_INVALID_PORT_ID;
}

static int map_device_to_port_id(int device)
{
	int port_id = SLIMBUS_0_RX;
	device = DEVICE_OUT_ALL;
	/*update the device when single stream to multiple device is handled*/
	if (device == DEVICE_OUT_ALL) {
		port_id = PRIMARY_I2S_RX | SLIMBUS_0_RX | HDMI_RX |
				INT_BT_SCO_RX | INT_FM_RX |
				RT_PROXY_PORT_001_RX |
				AFE_PORT_ID_PRIMARY_PCM_RX |
				MI2S_RX | SECONDARY_I2S_RX |
				SLIMBUS_1_RX | SLIMBUS_4_RX | SLIMBUS_3_RX |
				AFE_PORT_ID_SECONDARY_MI2S_RX;
	} else {
		/* update port_id based on the device */
	}
	return port_id;
}

int msm_routing_get_dolby_dap_param_to_set_control(
		struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol) {
	/* not used while setting the parameters */
	return 0;
}

int msm_routing_put_dolby_dap_param_to_set_control(
		struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol) {
	int rc = 0;
	uint32_t idx, j;
	uint32_t device = ucontrol->value.integer.value[0];
	uint32_t param_id = ucontrol->value.integer.value[1];
	uint32_t offset = ucontrol->value.integer.value[2];
	uint32_t length = ucontrol->value.integer.value[3];

	int port_id = dolby_dap_params_states.port_id;

	dolby_dap_params_states.port_ids_dolby_can_be_enabled =
						map_device_to_port_id(device);
	for (idx = 0; idx < ALL_DOLBY_PARAMS; idx++) {
		/*paramid from user space*/
		if (param_id == dolby_dap_params_id[idx])
			break;
	}
	if (idx > ALL_DOLBY_PARAMS-1) {
		pr_err("%s: invalid param id 0x%x to set\n", __func__,
			param_id);
		return -EINVAL;
	}
	switch (idx) {
		case DOLBY_COMMIT_ALL_IDX: {
			/* COMIIT ALL: Send all parameters to DSP */
			pr_debug("%s: COMMIT_ALL recvd\n", __func__);
			if (port_id != DOLBY_INVALID_PORT_ID)
				rc = dolby_dap_send_cached_params(port_id, 0);
		}
		break;
		case DOLBY_COMMIT_IDX: {
			pr_debug("%s: COMMIT recvd\n", __func__);
			/* COMMIT: Send only modified paramters to DSP */
			if (port_id != DOLBY_INVALID_PORT_ID)
				rc = dolby_dap_send_cached_params(port_id, 1);
		}
		break;
		case DOLBY_USE_CACHE_IDX: {
			pr_debug("%s: USE CACHE recvd val: %ld\n", __func__,
				ucontrol->value.integer.value[4]);
			dolby_dap_params_states.use_cache =
				ucontrol->value.integer.value[4];
		}
		break;
		case DOLBY_AUTO_ENDP_IDX: {
			pr_debug("%s: AUTO_ENDP recvd val: %ld\n", __func__,
				ucontrol->value.integer.value[4]);
			dolby_dap_params_states.auto_endp =
				ucontrol->value.integer.value[4];
		}
		break;
		case DOLBY_AUTO_ENDDEP_IDX: {
			pr_debug("%s: USE_ENDDEP_PARAMS recvd val: %ld\n",
				__func__, ucontrol->value.integer.value[4]);
			dolby_dap_params_states.enddep_params =
				ucontrol->value.integer.value[4];
		}
		break;
		default: {
			/* cache the parameters */
			dolby_dap_params_modified[idx] += 1;
			dolby_dap_params_length[idx] = length;
			pr_debug("%s: param recvd deviceId=0x%x paramId=0x%x offset=%d length=%d\n",
				__func__, device, param_id, offset, length);
			for (j = 0; j < length; j++) {
				dolby_dap_params_value[
					dolby_dap_params_offset[idx] +
					offset + j]
				= ucontrol->value.integer.value[4+j];
				pr_debug("value[%d]: %ld\n", j,
					ucontrol->value.integer.value[4+j]);
			}
		}
	}

	return rc;
}

int msm_routing_get_dolby_dap_param_to_get_control(
		struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol) {
	int rc = 0, i;
	char *params_value;
	int *update_params_value;
	uint32_t params_length = DOLBY_MAX_LENGTH_INDIVIDUAL_PARAM *
					sizeof(uint32_t);
	uint32_t param_payload_len =
			DOLBY_PARAM_PAYLOAD_SIZE * sizeof(uint32_t);
	int port_id = dolby_dap_params_states.port_id;

	if (port_id == DOLBY_INVALID_PORT_ID) {
		pr_err("%s, port_id not set, returning error", __func__);
		return -EINVAL;
	}
	params_value = kzalloc(params_length, GFP_KERNEL);
	if (!params_value) {
		pr_err("%s, params memory alloc failed\n", __func__);
		return -ENOMEM;
	}
	if (DOLBY_PARAM_ID_VER == dolby_dap_params_get.param_id) {
		rc = adm_get_params(dolby_dap_params_get.port_id,
						DOLBY_BUNDLE_MODULE_ID,
						DOLBY_PARAM_ID_VER,
						params_length +
							param_payload_len,
						params_value);
	} else {
		for (i = 0; i < MAX_DOLBY_PARAMS; i++)
			if (dolby_dap_params_id[i] ==
				dolby_dap_params_get.param_id)
				break;
		if (i > MAX_DOLBY_PARAMS-1) {
			pr_err("%s: invalid param id to set", __func__);
			rc = -EINVAL;
		} else {
			params_length = (dolby_dap_params_length[i] +
						DOLBY_PARAM_PAYLOAD_SIZE) *
						sizeof(uint32_t);
			rc = adm_get_params(
						dolby_dap_params_get.port_id,
						DOLBY_BUNDLE_MODULE_ID,
						dolby_dap_params_id[i],
						params_length +
						 param_payload_len,
						params_value);
		}
	}
	if (rc) {
		pr_err("%s: get parameters failed\n", __func__);
		kfree(params_value);
		return -EINVAL;
	}
	update_params_value = (int *)params_value;
	ucontrol->value.integer.value[0] = dolby_dap_params_get.device_id;
	ucontrol->value.integer.value[1] = dolby_dap_params_get.param_id;
	ucontrol->value.integer.value[2] = dolby_dap_params_get.offset;
	ucontrol->value.integer.value[3] = dolby_dap_params_get.length;

	pr_debug("%s: FROM DSP value[0] 0x%x value[1] %d value[2] 0x%x\n",
			__func__, update_params_value[0],
			update_params_value[1], update_params_value[2]);
	for (i = 0; i < dolby_dap_params_get.length; i++) {
		ucontrol->value.integer.value[DOLBY_PARAM_PAYLOAD_SIZE+i] =
			update_params_value[i];
		pr_debug("value[%d]:%d\n", i, update_params_value[i]);
	}
	pr_debug("%s: Returning param_id=0x%x offset=%d length=%d\n",
			__func__, dolby_dap_params_get.param_id,
			dolby_dap_params_get.offset,
			dolby_dap_params_get.length);
	kfree(params_value);
	return 0;
}

int msm_routing_put_dolby_dap_param_to_get_control(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol) {
	dolby_dap_params_get.device_id = ucontrol->value.integer.value[0];
	dolby_dap_params_get.port_id =
			(dolby_dap_params_get.device_id == DEVICE_OUT_ALL) ?
			dolby_dap_params_states.port_id :
			map_device_to_port_id(dolby_dap_params_get.device_id);
	dolby_dap_params_get.param_id = ucontrol->value.integer.value[1];
	dolby_dap_params_get.offset = ucontrol->value.integer.value[2];
	dolby_dap_params_get.length = ucontrol->value.integer.value[3];
	pr_debug("%s: param_id=0x%x offset=%d length=%d\n", __func__,
		dolby_dap_params_get.param_id, dolby_dap_params_get.offset,
		dolby_dap_params_get.length);
	return 0;
}

int msm_routing_get_dolby_dap_param_visualizer_control(
		struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol) {
	uint32_t length = dolby_dap_params_value[DOLBY_PARAM_VCNB_OFFSET];
	char *visualizer_data;
	int i, rc;
	int *update_visualizer_data;
	uint32_t offset, params_length =
		(2*length + DOLBY_VIS_PARAM_HEADER_SIZE)*sizeof(uint32_t);
	uint32_t param_payload_len =
		DOLBY_PARAM_PAYLOAD_SIZE * sizeof(uint32_t);
	int port_id = dolby_dap_params_states.port_id;
	if (port_id == DOLBY_INVALID_PORT_ID) {
		pr_debug("%s, port_id not set, returning error", __func__);
		ucontrol->value.integer.value[0] = 0;
		return -EINVAL;
	}
	visualizer_data = kzalloc(params_length, GFP_KERNEL);
	if (!visualizer_data) {
		pr_err("%s, params memory alloc failed\n", __func__);
		return -ENOMEM;
	}
	offset = 0;
	params_length = length * sizeof(uint32_t);
	rc = adm_get_params(dolby_dap_params_states.port_id,
					DOLBY_BUNDLE_MODULE_ID,
					DOLBY_PARAM_ID_VCBG,
					params_length + param_payload_len,
					visualizer_data + offset);
	if (rc) {
		pr_err("%s: get parameters failed\n", __func__);
		kfree(visualizer_data);
		return -EINVAL;
	}

	offset = length * sizeof(uint32_t);
	rc = adm_get_params(dolby_dap_params_states.port_id,
					DOLBY_BUNDLE_MODULE_ID,
					DOLBY_PARAM_ID_VCBE,
					params_length + param_payload_len,
					visualizer_data + offset);
	if (rc) {
		pr_err("%s: get parameters failed\n", __func__);
		kfree(visualizer_data);
		return -EINVAL;
	}

	ucontrol->value.integer.value[0] = 2*length;
	pr_debug("%s: visualizer data length %ld\n", __func__,
			ucontrol->value.integer.value[0]);
	update_visualizer_data = (int *)visualizer_data;
	for (i = 0; i < 2*length; i++) {
		ucontrol->value.integer.value[1+i] = update_visualizer_data[i];
		pr_debug("value[%d] %d\n", i, update_visualizer_data[i]);
	}
	kfree(visualizer_data);
	return 0;
}

int msm_routing_put_dolby_dap_param_visualizer_control(
		struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol) {
	/* not used while getting the visualizer data */
	return 0;
}

int msm_routing_get_dolby_dap_endpoint_control(
		struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol) {
	/* not used while setting the endpoint */
	return 0;
}

int msm_routing_put_dolby_dap_endpoint_control(
		struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol) {
	int device = ucontrol->value.integer.value[0];
	dolby_dap_params_states.device = device;
	return 0;
}
