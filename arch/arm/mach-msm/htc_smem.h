#ifndef HTC_SMEM_H
#define HTC_SMEM_H

typedef struct {

	u32		cmdseq;
	u32		rspseq;

	u32		opcode;
	u32		reserve;

	u32		parameter[4];
	u32		response[4];

} htc_modem_request_type;

typedef struct {
/* ========= belows are App write ==================== */
	u32      	version;
	u32      	struct_size;

	u32      	htc_smem_ce_radio_dbg_flag;
	u32      	htc_smem_app_run_mode;
	u32      	htc_smem_test_flag;
	u32		htc_smem_boot_reason;
	u8      	reserve1[8];


/* ========= belows are modem write ==================== */
	u32      	version_R;
	u32      	struct_size_R;

	u32      	htc_smem_erase_efs_flag;
	u32    		htc_smem_flight_mode_flag;
	u8      	htc_radio_version_addr[16];	//modem fill it
	u8      	htc_protocol_version_addr[16]; // modem fill it
	u8      	reserve2[16];

/* ========= belows are shared ==================== */
	htc_modem_request_type		htc_modem_request;		// for error handling only

/* for eMMC feature */
	u32      	htc_emmc_magic_flag;
	u32      	htc_emmc_buff_addr;
	u32      	htc_emmc_buff_size;
	u32      	htc_emmc_config_offset;
	u32      	htc_emmc_efs_sync_status;
	u32      	htc_emmc_nv_calibrate_status;
	u32      	htc_emmc_is_dev_inited;

	u32      	htc_smem_user_time_offset;


/* radio debug */
// Use 32 bytes to record the TCXO shutdown time statistics
	u32      	htc_tcxo_off_time_total;
	u32      	htc_tcxo_off_cnt_total;
	u32      	htc_tcxo_off_time_pwrc_suspend;
	u32      	htc_tcxo_off_cnt_pwrc_suspend;
	u32      	htc_global_garbage_cnt;
	u32      	htc_mssahb_reset_status;
	u32      	htc_watchdog_status;
	u32      	htc_cdlog_start_addr_for_apps;
	u32      	htc_cdlog_max_size_for_apps;

#if 1 /* HTC_INTEGRATE_CIQ_006 */
	u32		htc_ciq_flag;
#endif /*END HTC_INTEGRATE_CIQ_006 */
	u32		htc_modem_info_dch_time;
	u32		htc_modem_info_fach_time;
	u32		htc_modem_info_3g_cs_bar1_time;
	u32		htc_modem_info_3g_cs_bar2_time;
	u32		htc_modem_info_3g_cs_bar3_time;
	u32		htc_modem_info_3g_cs_bar4_time;
	u32		htc_modem_info_3g_ps_bar1_time;
	u32		htc_modem_info_3g_ps_bar2_time;
	u32		htc_modem_info_3g_ps_bar3_time;
	u32		htc_modem_info_3g_ps_bar4_time;
	u32		htc_modem_info_2g_cs_bar1_time;
	u32		htc_modem_info_2g_cs_bar2_time;
	u32		htc_modem_info_2g_cs_bar3_time;
	u32		htc_modem_info_2g_cs_bar4_time;
	u32		htc_modem_info_cs_bar1_time_1x;
	u32		htc_modem_info_cs_bar2_time_1x;
	u32		htc_modem_info_cs_bar3_time_1x;
	u32		htc_modem_info_cs_bar4_time_1x;
	u32		htc_modem_info_cs_bar5_time_1x;
	u32		htc_modem_info_ps_bar1_time_ev;
	u32		htc_modem_info_ps_bar2_time_ev;
	u32		htc_modem_info_ps_bar3_time_ev;
	u32		htc_modem_info_ps_bar4_time_ev;
	u32		htc_modem_info_ps_bar5_time_ev;
	u32		htc_modem_info_ps_bar1_time_lte;
	u32		htc_modem_info_ps_bar2_time_lte;
	u32		htc_modem_info_ps_bar3_time_lte;
	u32		htc_modem_info_ps_bar4_time_lte;
	u32		htc_modem_info_ps_bar5_time_lte;

	u32		htc_global_SYNCACK_garbage_cnt;	/*HTC_INTEGRATE_8960PCN_GF_003*/	//Offset:14C
	u32		htc_smlog_magic;
	u8 		htc_rom_sku[16];
	u32      	htc_smem_ce_radio_dbg_flag_ex1;
	u32      	htc_smem_ce_radio_dbg_flag_ex2;

	u32		htc_ril_fatal;
} htc_smem_type;
#endif  /* HTC_SMEM_H */
