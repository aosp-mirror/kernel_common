/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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
#ifndef __QDSP6VOICE_H__
#define __QDSP6VOICE_H__

#include <mach/qdsp6v2/apr.h>
#include <mach/qdsp6v2/rtac.h>
#include <linux/msm_ion.h>
#include <sound/voice_params.h>

#define MAX_VOC_PKT_SIZE 642
#define SESSION_NAME_LEN 20
#define NUM_OF_MEMORY_BLOCKS 1
#define NUM_OF_BUFFERS 2
#define BUFFER_BLOCK_SIZE       4096

#define MAX_COL_INFO_SIZE	324

#define VOC_REC_UPLINK		0x00
#define VOC_REC_DOWNLINK	0x01
#define VOC_REC_BOTH		0x02

struct voice_header {
	uint32_t id;
	uint32_t data_len;
};

struct voice_init {
	struct voice_header hdr;
	void *cb_handle;
};

struct stream_data {
	uint32_t stream_mute;
	uint32_t stream_mute_ramp_duration_ms;
};

struct device_data {
	uint32_t dev_mute;
	uint32_t sample;
	uint32_t enabled;
	uint32_t dev_id;
	uint32_t port_id;
	uint32_t volume_step_value;
	uint32_t volume_ramp_duration_ms;
	uint32_t dev_mute_ramp_duration_ms;
};

struct voice_dev_route_state {
	u16 rx_route_flag;
	u16 tx_route_flag;
};

struct voice_rec_route_state {
	u16 ul_flag;
	u16 dl_flag;
};

enum {
	VOC_INIT = 0,
	VOC_RUN,
	VOC_CHANGE,
	VOC_RELEASE,
	VOC_ERROR,
	VOC_STANDBY,
};

struct mem_buffer {
	dma_addr_t		phys;
	void			*data;
	uint32_t		size; 
};

struct share_mem_buf {
	struct ion_handle	*handle;
	struct ion_client	*client;
	struct mem_buffer	buf[NUM_OF_BUFFERS];
};

struct mem_map_table {
	dma_addr_t		phys;
	void			*data;
	uint32_t		size; 
	struct ion_handle	*handle;
	struct ion_client	*client;
};

#define VSS_ICOMMON_CMD_SET_UI_PROPERTY 0x00011103
#define VSS_ICOMMON_CMD_MAP_MEMORY   0x00011025
#define VSS_ICOMMON_CMD_UNMAP_MEMORY 0x00011026
#define VSS_ICOMMON_MAP_MEMORY_SHMEM8_4K_POOL  3

struct vss_icommon_cmd_map_memory_t {
	uint32_t phys_addr;

	uint32_t mem_size;
	

	uint16_t mem_pool_id;
} __packed;

struct vss_icommon_cmd_unmap_memory_t {
	uint32_t phys_addr;
} __packed;

struct vss_map_memory_cmd {
	struct apr_hdr hdr;
	struct vss_icommon_cmd_map_memory_t vss_map_mem;
} __packed;

struct vss_unmap_memory_cmd {
	struct apr_hdr hdr;
	struct vss_icommon_cmd_unmap_memory_t vss_unmap_mem;
} __packed;

#define VSS_IMVM_CMD_CREATE_PASSIVE_CONTROL_SESSION	0x000110FF

#define VSS_IMVM_CMD_SET_POLICY_DUAL_CONTROL	0x00011327

#define VSS_IMVM_CMD_CREATE_FULL_CONTROL_SESSION	0x000110FE

#define APRV2_IBASIC_CMD_DESTROY_SESSION		0x0001003C

#define VSS_IMVM_CMD_ATTACH_STREAM			0x0001123C

#define VSS_IMVM_CMD_DETACH_STREAM			0x0001123D

#define VSS_IMVM_CMD_ATTACH_VOCPROC		       0x0001123E

#define VSS_IMVM_CMD_DETACH_VOCPROC			0x0001123F

#define VSS_IMVM_CMD_START_VOICE			0x00011190

#define VSS_IMVM_CMD_STANDBY_VOICE                       0x00011191

#define VSS_IMVM_CMD_STOP_VOICE				0x00011192

#define VSS_IMVM_CMD_PAUSE_VOICE			0x0001137D

#define VSS_ISTREAM_CMD_ATTACH_VOCPROC			0x000110F8

#define VSS_ISTREAM_CMD_DETACH_VOCPROC			0x000110F9


#define VSS_ISTREAM_CMD_SET_TTY_MODE			0x00011196

#define VSS_ICOMMON_CMD_SET_NETWORK			0x0001119C

#define VSS_ICOMMON_CMD_SET_VOICE_TIMING		0x000111E0

#define VSS_IMEMORY_CMD_MAP_PHYSICAL			0x00011334
#define VSS_IMEMORY_RSP_MAP				0x00011336
#define VSS_IMEMORY_CMD_UNMAP				0x00011337
#define VSS_IMVM_CMD_SET_CAL_NETWORK			0x0001137A
#define VSS_IMVM_CMD_SET_CAL_MEDIA_TYPE		0x0001137B

enum msm_audio_voc_rate {
		VOC_0_RATE, 
		VOC_8_RATE, 
		VOC_4_RATE, 
		VOC_2_RATE, 
		VOC_1_RATE,  
		VOC_8_RATE_NC  
};

struct vss_istream_cmd_set_tty_mode_t {
	uint32_t mode;
} __packed;

struct vss_istream_cmd_attach_vocproc_t {
	uint16_t handle;
	
} __packed;

struct vss_istream_cmd_detach_vocproc_t {
	uint16_t handle;
	
} __packed;

struct vss_imvm_cmd_attach_stream_t {
	uint16_t handle;
	
} __packed;

struct vss_imvm_cmd_detach_stream_t {
	uint16_t handle;
	
} __packed;

struct vss_icommon_cmd_set_network_t {
	uint32_t network_id;
	
} __packed;

struct vss_icommon_cmd_set_voice_timing_t {
	uint16_t mode;
	uint16_t enc_offset;
	uint16_t dec_req_offset;
	uint16_t dec_offset;
} __packed;

struct vss_imvm_cmd_create_control_session_t {
	char name[SESSION_NAME_LEN];
} __packed;


struct vss_imvm_cmd_set_policy_dual_control_t {
	bool enable_flag;
	
} __packed;

struct mvm_attach_vocproc_cmd {
	struct apr_hdr hdr;
	struct vss_istream_cmd_attach_vocproc_t mvm_attach_cvp_handle;
} __packed;

struct mvm_detach_vocproc_cmd {
	struct apr_hdr hdr;
	struct vss_istream_cmd_detach_vocproc_t mvm_detach_cvp_handle;
} __packed;

struct mvm_create_ctl_session_cmd {
	struct apr_hdr hdr;
	struct vss_imvm_cmd_create_control_session_t mvm_session;
} __packed;

struct mvm_modem_dual_control_session_cmd {
	struct apr_hdr hdr;
	struct vss_imvm_cmd_set_policy_dual_control_t voice_ctl;
} __packed;

struct mvm_set_tty_mode_cmd {
	struct apr_hdr hdr;
	struct vss_istream_cmd_set_tty_mode_t tty_mode;
} __packed;

struct mvm_attach_stream_cmd {
	struct apr_hdr hdr;
	struct vss_imvm_cmd_attach_stream_t attach_stream;
} __packed;

struct mvm_detach_stream_cmd {
	struct apr_hdr hdr;
	struct vss_imvm_cmd_detach_stream_t detach_stream;
} __packed;

struct mvm_set_network_cmd {
	struct apr_hdr hdr;
	struct vss_icommon_cmd_set_network_t network;
} __packed;

struct mvm_set_voice_timing_cmd {
	struct apr_hdr hdr;
	struct vss_icommon_cmd_set_voice_timing_t timing;
} __packed;

struct vss_imemory_table_descriptor_t {
	uint64_t mem_address;
	uint32_t mem_size;
	
} __packed;

struct vss_imemory_block_t {
	uint64_t mem_address;
	uint32_t mem_size;
} __packed;

struct vss_imemory_table_t {
	struct vss_imemory_table_descriptor_t next_table_descriptor;
	struct vss_imemory_block_t blocks[NUM_OF_MEMORY_BLOCKS];
	
} __packed;

struct vss_imemory_cmd_map_physical_t {
	struct apr_hdr hdr;
	struct vss_imemory_table_descriptor_t table_descriptor;
	bool is_cached;
	uint16_t cache_line_size;
	
	uint32_t access_mask;
	uint32_t page_align;
	
	uint8_t min_data_width;
	uint8_t max_data_width;
} __packed;

struct vss_imvm_cmd_set_cal_network_t {
	struct apr_hdr hdr;
	uint32_t network_id;
} __packed;

struct vss_imvm_cmd_set_cal_media_type_t {
	struct apr_hdr hdr;
	uint32_t media_id;
} __packed;

struct vss_imemory_cmd_unmap_t {
	struct apr_hdr hdr;
	uint32_t mem_handle;
} __packed;

#define VSS_ISTREAM_CMD_CREATE_PASSIVE_CONTROL_SESSION	0x00011140

#define VSS_ISTREAM_CMD_CREATE_FULL_CONTROL_SESSION	0x000110F7

#define APRV2_IBASIC_CMD_DESTROY_SESSION		0x0001003C

#define VSS_IVOLUME_CMD_MUTE_V2				0x0001138B

#define VSS_ISTREAM_CMD_REGISTER_CALIBRATION_DATA_V2    0x00011369

#define VSS_ISTREAM_CMD_DEREGISTER_CALIBRATION_DATA     0x0001127A

#define VSS_ISTREAM_CMD_SET_MEDIA_TYPE			0x00011186

#define VSS_ISTREAM_EVT_SEND_ENC_BUFFER			0x00011015

#define VSS_ISTREAM_EVT_REQUEST_DEC_BUFFER		0x00011017

#define VSS_ISTREAM_EVT_OOB_NOTIFY_DEC_BUFFER_REQUEST	0x0001136E

#define VSS_ISTREAM_EVT_SEND_DEC_BUFFER			0x00011016

#define VSS_ISTREAM_CMD_VOC_AMR_SET_ENC_RATE		0x0001113E

#define VSS_ISTREAM_CMD_VOC_AMRWB_SET_ENC_RATE		0x0001113F

#define VSS_ISTREAM_CMD_CDMA_SET_ENC_MINMAX_RATE	0x00011019

#define VSS_ISTREAM_CMD_SET_ENC_DTX_MODE		0x0001101D

#define MODULE_ID_VOICE_MODULE_ST			0x00010EE3
#define VOICE_PARAM_MOD_ENABLE				0x00010E00
#define MOD_ENABLE_PARAM_LEN				4

#define VSS_IPLAYBACK_CMD_START				0x000112BD

#define VSS_IPLAYBACK_CMD_STOP				0x00011239

#define VSS_IPLAYBACK_PORT_ID_DEFAULT			0x0000FFFF

#define VSS_IPLAYBACK_PORT_ID_VOICE			0x00008005

#define VSS_IPLAYBACK_PORT_ID_VOICE2			0x00008002

#define VSS_IRECORD_CMD_START				0x000112BE
#define VSS_IRECORD_CMD_STOP				0x00011237

#define VSS_IRECORD_PORT_ID_DEFAULT			0x0000FFFF

#define VSS_IRECORD_TAP_POINT_NONE			0x00010F78

#define VSS_IRECORD_TAP_POINT_STREAM_END		0x00010F79

#define VSS_IRECORD_MODE_TX_RX_STEREO			0x00010F7A

#define VSS_IRECORD_MODE_TX_RX_MIXING			0x00010F7B

#define VSS_ISTREAM_EVT_NOT_READY			0x000110FD

#define VSS_ISTREAM_EVT_READY				0x000110FC

#define VSS_ISTREAM_EVT_OOB_NOTIFY_DEC_BUFFER_READY	0x0001136F

#define VSS_ISTREAM_EVT_OOB_NOTIFY_ENC_BUFFER_READY	0x0001136C

#define VSS_ISTREAM_EVT_OOB_NOTIFY_ENC_BUFFER_CONSUMED	0x0001136D

#define VSS_ISTREAM_CMD_SET_OOB_PACKET_EXCHANGE_CONFIG	0x0001136B

#define VSS_ISTREAM_PACKET_EXCHANGE_MODE_INBAND	0

#define VSS_ISTREAM_PACKET_EXCHANGE_MODE_OUT_OF_BAND	1

#define VSS_ISTREAM_CMD_SET_PACKET_EXCHANGE_MODE	0x0001136A

struct vss_iplayback_cmd_start_t {
	uint16_t port_id;
}  __packed;

struct vss_irecord_cmd_start_t {
	uint32_t rx_tap_point;
	uint32_t tx_tap_point;
	uint16_t port_id;
	uint32_t mode;
} __packed;

struct vss_istream_cmd_create_passive_control_session_t {
	char name[SESSION_NAME_LEN];
} __packed;

#define VSS_IVOLUME_DIRECTION_TX	0
#define VSS_IVOLUME_DIRECTION_RX	1

#define VSS_IVOLUME_MUTE_OFF		0
#define VSS_IVOLUME_MUTE_ON		1

#define DEFAULT_MUTE_RAMP_DURATION	500
#define DEFAULT_VOLUME_RAMP_DURATION	20
#define MAX_RAMP_DURATION		5000

struct vss_ivolume_cmd_mute_v2_t {
	uint16_t direction;
	uint16_t mute_flag;
	uint16_t ramp_duration_ms;
} __packed;

struct vss_istream_cmd_create_full_control_session_t {
	uint16_t direction;
	uint32_t enc_media_type;
	
	uint32_t dec_media_type;
	
	uint32_t network_id;
	
	char name[SESSION_NAME_LEN];
} __packed;

struct vss_istream_cmd_set_media_type_t {
	uint32_t rx_media_id;
	
	uint32_t tx_media_id;
	
} __packed;

struct vss_istream_evt_send_enc_buffer_t {
	uint32_t media_id;
      
	uint8_t packet_data[MAX_VOC_PKT_SIZE];
      
} __packed;

struct vss_istream_evt_send_dec_buffer_t {
	uint32_t media_id;
      
	uint8_t packet_data[MAX_VOC_PKT_SIZE];
      
} __packed;

struct vss_istream_cmd_voc_amr_set_enc_rate_t {
	uint32_t mode;
} __packed;

struct vss_istream_cmd_voc_amrwb_set_enc_rate_t {
	uint32_t mode;
} __packed;

struct vss_istream_cmd_cdma_set_enc_minmax_rate_t {
	uint16_t min_rate;
	uint16_t max_rate;
} __packed;

struct vss_istream_cmd_set_enc_dtx_mode_t {
	uint32_t enable;
} __packed;

struct vss_istream_cmd_register_calibration_data_v2_t {
	uint32_t cal_mem_handle;
	
	uint64_t cal_mem_address;
	
	uint32_t cal_mem_size;
	
	uint8_t column_info[MAX_COL_INFO_SIZE];
} __packed;

struct vss_icommon_cmd_set_ui_property_enable_t {
	uint32_t module_id;
	
	uint32_t param_id;
	
	uint16_t param_size;
	
	uint16_t reserved;
	
	uint16_t enable;
	uint16_t reserved_field;
	
};


#define VSS_ISTREAM_EVT_RX_DTMF_DETECTED (0x0001101A)

struct vss_istream_cmd_set_rx_dtmf_detection {
	uint32_t enable;
};

#define VSS_ISTREAM_CMD_SET_RX_DTMF_DETECTION (0x00011027)

struct vss_istream_evt_rx_dtmf_detected {
	uint16_t low_freq;
	uint16_t high_freq;
};

struct cvs_set_rx_dtmf_detection_cmd {
	struct apr_hdr hdr;
	struct vss_istream_cmd_set_rx_dtmf_detection cvs_dtmf_det;
} __packed;


struct cvs_create_passive_ctl_session_cmd {
	struct apr_hdr hdr;
	struct vss_istream_cmd_create_passive_control_session_t cvs_session;
} __packed;

struct cvs_create_full_ctl_session_cmd {
	struct apr_hdr hdr;
	struct vss_istream_cmd_create_full_control_session_t cvs_session;
} __packed;

struct cvs_destroy_session_cmd {
	struct apr_hdr hdr;
} __packed;

struct cvs_set_mute_cmd {
	struct apr_hdr hdr;
	struct vss_ivolume_cmd_mute_v2_t cvs_set_mute;
} __packed;

struct cvs_set_media_type_cmd {
	struct apr_hdr hdr;
	struct vss_istream_cmd_set_media_type_t media_type;
} __packed;

struct cvs_send_dec_buf_cmd {
	struct apr_hdr hdr;
	struct vss_istream_evt_send_dec_buffer_t dec_buf;
} __packed;

struct cvs_set_amr_enc_rate_cmd {
	struct apr_hdr hdr;
	struct vss_istream_cmd_voc_amr_set_enc_rate_t amr_rate;
} __packed;

struct cvs_set_amrwb_enc_rate_cmd {
	struct apr_hdr hdr;
	struct vss_istream_cmd_voc_amrwb_set_enc_rate_t amrwb_rate;
} __packed;

struct cvs_set_cdma_enc_minmax_rate_cmd {
	struct apr_hdr hdr;
	struct vss_istream_cmd_cdma_set_enc_minmax_rate_t cdma_rate;
} __packed;

struct cvs_set_enc_dtx_mode_cmd {
	struct apr_hdr hdr;
	struct vss_istream_cmd_set_enc_dtx_mode_t dtx_mode;
} __packed;

struct cvs_register_cal_data_cmd {
	struct apr_hdr hdr;
	struct vss_istream_cmd_register_calibration_data_v2_t cvs_cal_data;
} __packed;

struct cvs_deregister_cal_data_cmd {
	struct apr_hdr hdr;
} __packed;

struct cvs_set_pp_enable_cmd {
	struct apr_hdr hdr;
	struct vss_icommon_cmd_set_ui_property_enable_t vss_set_pp;
} __packed;
struct cvs_start_record_cmd {
	struct apr_hdr hdr;
	struct vss_irecord_cmd_start_t rec_mode;
} __packed;

struct cvs_start_playback_cmd {
	struct apr_hdr hdr;
	struct vss_iplayback_cmd_start_t playback_mode;
} __packed;

struct cvs_dec_buffer_ready_cmd {
	struct apr_hdr hdr;
} __packed;

struct cvs_enc_buffer_consumed_cmd {
	struct apr_hdr hdr;
} __packed;

struct vss_istream_cmd_set_oob_packet_exchange_config_t {
	struct apr_hdr hdr;
	uint32_t mem_handle;
	uint64_t enc_buf_addr;
	uint32_t enc_buf_size;
	uint64_t dec_buf_addr;
	uint32_t dec_buf_size;
} __packed;

struct vss_istream_cmd_set_packet_exchange_mode_t {
	struct apr_hdr hdr;
	uint32_t mode;
} __packed;


#define VSS_IVOCPROC_CMD_CREATE_FULL_CONTROL_SESSION	0x000100C3

#define APRV2_IBASIC_CMD_DESTROY_SESSION		0x0001003C

#define VSS_IVOCPROC_CMD_SET_DEVICE_V2			0x000112C6

#define VSS_IVOCPROC_CMD_SET_VP3_DATA			0x000110EB

#define VSS_IVOLUME_CMD_SET_STEP			0x000112C2

#define VSS_IVOCPROC_CMD_ENABLE				0x000100C6

#define VSS_IVOCPROC_CMD_DISABLE			0x000110E1

#define VSS_IVOCPROC_CMD_REGISTER_DEVICE_CONFIG		0x00011371

#define VSS_IVOCPROC_CMD_DEREGISTER_DEVICE_CONFIG	0x00011372

#define VSS_IVOCPROC_CMD_REGISTER_CALIBRATION_DATA_V2	0x00011373
#define VSS_IVOCPROC_CMD_DEREGISTER_CALIBRATION_DATA	0x00011276

#define VSS_IVOCPROC_CMD_REGISTER_VOL_CALIBRATION_DATA	0x00011374
#define VSS_IVOCPROC_CMD_DEREGISTER_VOL_CALIBRATION_DATA	0x00011375

#define VSS_IVOCPROC_TOPOLOGY_ID_NONE			0x00010F70
#define VSS_IVOCPROC_TOPOLOGY_ID_TX_SM_ECNS		0x00010F71
#define VSS_IVOCPROC_TOPOLOGY_ID_TX_DM_FLUENCE		0x00010F72

#define VSS_IVOCPROC_TOPOLOGY_ID_RX_DEFAULT		0x00010F77

#define VSS_ICOMMON_CAL_NETWORK_ID_NONE		0x0001135E

#define VSS_IVOCPROC_VOCPROC_MODE_EC_INT_MIXING	0x00010F7C

#define VSS_IVOCPROC_VOCPROC_MODE_EC_EXT_MIXING	0x00010F7D

#define VSS_IVOCPROC_PORT_ID_NONE			0xFFFF

#define VSS_NETWORK_ID_DEFAULT				0x00010037
#define VSS_NETWORK_ID_VOIP_NB				0x00011240
#define VSS_NETWORK_ID_VOIP_WB				0x00011241
#define VSS_NETWORK_ID_VOIP_WV				0x00011242

#define VSS_MEDIA_ID_EVRC_MODEM		0x00010FC2
#define VSS_MEDIA_ID_AMR_NB_MODEM	0x00010FC6
#define VSS_MEDIA_ID_AMR_WB_MODEM	0x00010FC7
#define VSS_MEDIA_ID_PCM_NB		0x00010FCB
#define VSS_MEDIA_ID_PCM_WB		0x00010FCC
#define VSS_MEDIA_ID_G711_ALAW		0x00010FCD
#define VSS_MEDIA_ID_G711_MULAW		0x00010FCE
#define VSS_MEDIA_ID_G729		0x00010FD0
#define VSS_MEDIA_ID_4GV_NB_MODEM	0x00010FC3
#define VSS_MEDIA_ID_4GV_WB_MODEM	0x00010FC4
#define VSS_MEDIA_ID_4GV_NW_MODEM	0x00010FC5

#define VSS_IVOCPROC_CMD_CREATE_FULL_CONTROL_SESSION_V2	0x000112BF

struct vss_ivocproc_cmd_create_full_control_session_v2_t {
	uint16_t direction;
	uint16_t tx_port_id;
	uint32_t tx_topology_id;
	uint16_t rx_port_id;
	uint32_t rx_topology_id;
	uint32_t profile_id;
	
	uint32_t vocproc_mode;
	uint16_t ec_ref_port_id;
	char name[SESSION_NAME_LEN];
} __packed;

struct vss_ivocproc_cmd_set_volume_index_t {
	uint16_t vol_index;
} __packed;

struct vss_ivolume_cmd_set_step_t {
	uint16_t direction;
	uint32_t value;
	uint16_t ramp_duration_ms;
} __packed;

struct vss_ivocproc_cmd_set_device_v2_t {
	uint16_t tx_port_id;
	uint32_t tx_topology_id;
	uint16_t rx_port_id;
	uint32_t rx_topology_id;
	uint32_t vocproc_mode;
	uint16_t ec_ref_port_id;
} __packed;

struct vss_ivocproc_cmd_register_device_config_t {
	uint32_t mem_handle;
	uint64_t mem_address;
	
	uint32_t mem_size;
	
} __packed;

struct vss_ivocproc_cmd_register_calibration_data_v2_t {
	uint32_t cal_mem_handle;
	uint64_t cal_mem_address;
	
	uint32_t cal_mem_size;
	
	uint8_t column_info[MAX_COL_INFO_SIZE];
} __packed;

struct vss_ivocproc_cmd_register_volume_cal_data_t {
	uint32_t cal_mem_handle;
	uint64_t cal_mem_address;
	
	uint32_t cal_mem_size;
	
	uint8_t column_info[MAX_COL_INFO_SIZE];
} __packed;

struct cvp_create_full_ctl_session_cmd {
	struct apr_hdr hdr;
	struct vss_ivocproc_cmd_create_full_control_session_v2_t cvp_session;
} __packed;

struct cvp_command {
	struct apr_hdr hdr;
} __packed;

struct cvp_set_device_cmd {
	struct apr_hdr hdr;
	struct vss_ivocproc_cmd_set_device_v2_t cvp_set_device_v2;
} __packed;

struct cvp_set_device_cmd_v2 {
	struct apr_hdr hdr;
	struct vss_ivocproc_cmd_set_device_v2_t cvp_set_device_v2;
} __packed;

struct cvp_set_vp3_data_cmd {
	struct apr_hdr hdr;
} __packed;

struct cvp_set_rx_volume_index_cmd {
	struct apr_hdr hdr;
	struct vss_ivocproc_cmd_set_volume_index_t cvp_set_vol_idx;
} __packed;

struct cvp_set_rx_volume_step_cmd {
	struct apr_hdr hdr;
	struct vss_ivolume_cmd_set_step_t cvp_set_vol_step;
} __packed;

struct cvp_register_dev_cfg_cmd {
	struct apr_hdr hdr;
	struct vss_ivocproc_cmd_register_device_config_t cvp_dev_cfg_data;
} __packed;

struct cvp_deregister_dev_cfg_cmd {
	struct apr_hdr hdr;
} __packed;

struct cvp_register_cal_data_cmd {
	struct apr_hdr hdr;
	struct vss_ivocproc_cmd_register_calibration_data_v2_t cvp_cal_data;
} __packed;

struct cvp_deregister_cal_data_cmd {
	struct apr_hdr hdr;
} __packed;

struct cvp_register_vol_cal_data_cmd {
	struct apr_hdr hdr;
	struct vss_ivocproc_cmd_register_volume_cal_data_t cvp_vol_cal_data;
} __packed;

struct cvp_deregister_vol_cal_data_cmd {
	struct apr_hdr hdr;
} __packed;

struct cvp_set_mute_cmd {
	struct apr_hdr hdr;
	struct vss_ivolume_cmd_mute_v2_t cvp_set_mute;
} __packed;

typedef void (*ul_cb_fn)(uint8_t *voc_pkt,
			 uint32_t pkt_len,
			 uint32_t timestamp,
			 void *private_data);

typedef void (*dl_cb_fn)(uint8_t *voc_pkt,
			 void *private_data);

typedef void (*dtmf_rx_det_cb_fn)(uint8_t *pkt,
				  char *session,
				  void *private_data);

typedef void (*voip_ssr_cb) (uint32_t opcode,
				void *private_data);

struct mvs_driver_info {
	uint32_t media_type;
	uint32_t rate;
	uint32_t network_type;
	uint32_t dtx_mode;
	ul_cb_fn ul_cb;
	dl_cb_fn dl_cb;
	voip_ssr_cb ssr_cb;
	void *private_data;
	uint32_t evrc_min_rate;
	uint32_t evrc_max_rate;
};

struct dtmf_driver_info {
	dtmf_rx_det_cb_fn dtmf_rx_ul_cb;
	void *private_data;
};

struct incall_rec_info {
	uint32_t rec_enable;
	uint32_t rec_mode;
	uint32_t recording;
};

struct incall_music_info {
	uint32_t play_enable;
	uint32_t playing;
	int count;
	int force;
	uint16_t port_id;
};

struct share_memory_info {
	u32			mem_handle;
	struct share_mem_buf	sh_buf;
	struct mem_map_table	memtbl;
};

struct voice_data {
	int voc_state;

	
	struct share_memory_info	shmem_info;

	wait_queue_head_t mvm_wait;
	wait_queue_head_t cvs_wait;
	wait_queue_head_t cvp_wait;

	
	struct device_data dev_rx;
	struct device_data dev_tx;

	
	struct stream_data stream_rx;
	struct stream_data stream_tx;

	u32 mvm_state;
	u32 cvs_state;
	u32 cvp_state;

	
	u16 mvm_handle;
	
	u16 cvs_handle;
	
	u16 cvp_handle;

	struct mutex lock;

	uint16_t sidetone_gain;
	uint8_t tty_mode;
	
	uint32_t st_enable;
	uint32_t dtmf_rx_detect_en;
	
	uint8_t lch_mode;

	struct voice_dev_route_state voc_route_state;

	u32 session_id;

	struct incall_rec_info rec_info;

	struct incall_music_info music_info;

	struct voice_rec_route_state rec_route_state;
};

struct cal_mem {
	struct ion_handle *handle;
	uint32_t phy;
	void *buf;
};

#define MAX_VOC_SESSIONS 6

struct common_data {
	
	uint32_t default_mute_val;
	uint32_t default_sample_val;
	uint32_t default_vol_step_val;
	uint32_t default_vol_ramp_duration_ms;
	uint32_t default_mute_ramp_duration_ms;
	bool ec_ref_ext;
	uint16_t ec_port_id;

	
	void *apr_q6_mvm;
	
	void *apr_q6_cvs;
	
	void *apr_q6_cvp;

	struct mem_map_table cal_mem_map_table;
	uint32_t cal_mem_handle;

	struct mem_map_table rtac_mem_map_table;
	uint32_t rtac_mem_handle;

	struct cal_mem cvp_cal;
	struct cal_mem cvs_cal;

	struct mutex common_lock;

	struct mvs_driver_info mvs_info;

	struct dtmf_driver_info dtmf_info;

	struct voice_data voice[MAX_VOC_SESSIONS];

	bool srvcc_rec_flag;
};

struct voice_session_itr {
	int cur_idx;
	int session_idx;
};

void voc_register_mvs_cb(ul_cb_fn ul_cb,
			dl_cb_fn dl_cb,
			voip_ssr_cb ssr_cb,
			void *private_data);

void voc_register_dtmf_rx_detection_cb(dtmf_rx_det_cb_fn dtmf_rx_ul_cb,
				       void *private_data);

void voc_config_vocoder(uint32_t media_type,
			uint32_t rate,
			uint32_t network_type,
			uint32_t dtx_mode,
			uint32_t evrc_min_rate,
			uint32_t evrc_max_rate);

enum {
	DEV_RX = 0,
	DEV_TX,
};

enum {
	RX_PATH = 0,
	TX_PATH,
};


#define VOC_PATH_PASSIVE 0
#define VOC_PATH_FULL 1
#define VOC_PATH_VOLTE_PASSIVE 2
#define VOC_PATH_VOICE2_PASSIVE 3
#define VOC_PATH_QCHAT_PASSIVE 4
#define VOC_PATH_VOWLAN_PASSIVE 5

#define MAX_SESSION_NAME_LEN 32
#define VOICE_SESSION_NAME   "Voice session"
#define VOIP_SESSION_NAME    "VoIP session"
#define VOLTE_SESSION_NAME   "VoLTE session"
#define VOICE2_SESSION_NAME  "Voice2 session"
#define QCHAT_SESSION_NAME   "QCHAT session"
#define VOWLAN_SESSION_NAME  "VoWLAN session"

#define VOICE2_SESSION_VSID_STR "10DC1000"
#define QCHAT_SESSION_VSID_STR "10803000"
#define VOWLAN_SESSION_VSID_STR "10002000"
#define VOICE_SESSION_VSID  0x10C01000
#define VOICE2_SESSION_VSID 0x10DC1000
#define VOLTE_SESSION_VSID  0x10C02000
#define VOIP_SESSION_VSID   0x10004000
#define QCHAT_SESSION_VSID  0x10803000
#define VOWLAN_SESSION_VSID 0x10002000
#define ALL_SESSION_VSID    0xFFFFFFFF
#define VSID_MAX            ALL_SESSION_VSID

#define APP_ID_MASK         0x3F000
#define APP_ID_SHIFT		12
enum vsid_app_type {
	VSID_APP_NONE = 0,
	VSID_APP_CS_VOICE = 1,
	VSID_APP_IMS = 2, 
	VSID_APP_QCHAT = 3,
	VSID_APP_VOIP = 4, 
	VSID_APP_MAX,
};

int voc_set_pp_enable(uint32_t session_id, uint32_t module_id,
		      uint32_t enable);
int voc_get_pp_enable(uint32_t session_id, uint32_t module_id);
uint8_t voc_get_tty_mode(uint32_t session_id);
int voc_set_tty_mode(uint32_t session_id, uint8_t tty_mode);
int voc_start_voice_call(uint32_t session_id);
int voc_end_voice_call(uint32_t session_id);
int voc_standby_voice_call(uint32_t session_id);
int voc_resume_voice_call(uint32_t session_id);
int voc_set_lch(uint32_t session_id, enum voice_lch_mode lch_mode);
int voc_set_rxtx_port(uint32_t session_id,
		      uint32_t dev_port_id,
		      uint32_t dev_type);
int voc_set_rx_vol_step(uint32_t session_id, uint32_t dir, uint32_t vol_step,
			uint32_t ramp_duration);
int voc_set_tx_mute(uint32_t session_id, uint32_t dir, uint32_t mute,
		    uint32_t ramp_duration);
int voc_set_device_mute(uint32_t session_id, uint32_t dir, uint32_t mute,
			uint32_t ramp_duration);
int voc_get_rx_device_mute(uint32_t session_id);
int voc_set_route_flag(uint32_t session_id, uint8_t path_dir, uint8_t set);
uint8_t voc_get_route_flag(uint32_t session_id, uint8_t path_dir);
int voc_enable_dtmf_rx_detection(uint32_t session_id, uint32_t enable);
void voc_disable_dtmf_det_on_active_sessions(void);
int voc_alloc_cal_shared_memory(void);
int voc_alloc_voip_shared_memory(void);
int is_voc_initialized(void);
int voc_register_vocproc_vol_table(void);
int voc_deregister_vocproc_vol_table(void);

int voc_unmap_cal_blocks(void);
int voc_map_rtac_block(struct rtac_cal_block_data *cal_block);
int voc_unmap_rtac_block(uint32_t *mem_map_handle);

uint32_t voc_get_session_id(char *name);

int voc_start_playback(uint32_t set, uint16_t port_id);
int voc_start_record(uint32_t port_id, uint32_t set, uint32_t session_id);
int voice_get_idx_for_session(u32 session_id);
int voc_set_ext_ec_ref(uint16_t port_id, bool state);
int voc_update_amr_vocoder_rate(uint32_t session_id);

int voc_disable_device(uint32_t session_id);
int voc_enable_device(uint32_t session_id);
#endif
