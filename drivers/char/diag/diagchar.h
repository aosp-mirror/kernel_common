/* Copyright (c) 2008-2014, The Linux Foundation. All rights reserved.
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

#ifndef DIAGCHAR_H
#define DIAGCHAR_H

#include <linux/init.h>
#include <linux/module.h>
#include <linux/mempool.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <mach/msm_smd.h>
#include <asm/atomic.h>
#include <mach/usbdiag.h>
#include <asm/mach-types.h>

#define USB_MAX_OUT_BUF 4096
#define APPS_BUF_SIZE	4096
#define IN_BUF_SIZE		16384
#define MAX_IN_BUF_SIZE	32768
#define MAX_SYNC_OBJ_NAME_SIZE	32
#define HDLC_MAX 4096
#define HDLC_OUT_BUF_SIZE	8192
#define POOL_TYPE_COPY		1
#define POOL_TYPE_HDLC		2
#define POOL_TYPE_USER		3
#define POOL_TYPE_WRITE_STRUCT	4
#define POOL_TYPE_HSIC		5
#define POOL_TYPE_HSIC_2	6
#define POOL_TYPE_HSIC_WRITE	11
#define POOL_TYPE_HSIC_2_WRITE	12
#define POOL_TYPE_ALL		10
#define POOL_TYPE_DCI		20

#define POOL_COPY_IDX		0
#define POOL_HDLC_IDX		1
#define POOL_USER_IDX		2
#define POOL_WRITE_STRUCT_IDX	3
#define POOL_DCI_IDX		4
#define POOL_BRIDGE_BASE	POOL_DCI_IDX
#define POOL_HSIC_IDX		(POOL_BRIDGE_BASE + 1)
#define POOL_HSIC_2_IDX		(POOL_BRIDGE_BASE + 2)
#define POOL_HSIC_3_IDX		(POOL_BRIDGE_BASE + 3)
#define POOL_HSIC_4_IDX		(POOL_BRIDGE_BASE + 4)
#define POOL_HSIC_WRITE_IDX	(POOL_BRIDGE_BASE + 5)
#define POOL_HSIC_2_WRITE_IDX	(POOL_BRIDGE_BASE + 6)
#define POOL_HSIC_3_WRITE_IDX	(POOL_BRIDGE_BASE + 7)
#define POOL_HSIC_4_WRITE_IDX	(POOL_BRIDGE_BASE + 8)

#ifdef CONFIG_DIAGFWD_BRIDGE_CODE
#define NUM_MEMORY_POOLS	13
#else
#define NUM_MEMORY_POOLS	5
#endif

#define MAX_SSID_PER_RANGE	200

#define ALL_PROC		-1
#define MODEM_DATA		0
#define LPASS_DATA		1
#define WCNSS_DATA		2
#define APPS_DATA		3
#define SDIO_DATA		4
#define HSIC_DATA		5
#define HSIC_2_DATA		6
#define SMUX_DATA		10
#define MODEM_PROC		0
#define APPS_PROC		1
#define LPASS_PROC		2
#define WCNSS_PROC		3
#define MSG_MASK_SIZE		((MAX_SSID_PER_RANGE+3) * 4 * MSG_MASK_TBL_CNT)
#define MAX_EQUIP_ID		16
#define MAX_ITEMS_PER_EQUIP_ID	512
#define LOG_MASK_ITEM_SIZE	(5 + MAX_ITEMS_PER_EQUIP_ID)
#define LOG_MASK_SIZE		(MAX_EQUIP_ID * LOG_MASK_ITEM_SIZE)
#define EVENT_MASK_SIZE 1000
#define USER_SPACE_DATA 8192
#define PKT_SIZE 4096

#define DIAG_CTRL_MSG_LOG_MASK	9
#define DIAG_CTRL_MSG_EVENT_MASK	10
#define DIAG_CTRL_MSG_F3_MASK	11
#define CONTROL_CHAR	0x7E

#define DIAG_CON_APSS (0x0001)	
#define DIAG_CON_MPSS (0x0002)	
#define DIAG_CON_LPASS (0x0004)	
#define DIAG_CON_WCNSS (0x0008)	

#define NUM_STM_PROCESSORS	4

#define DIAG_STM_MODEM	0x01
#define DIAG_STM_LPASS	0x02
#define DIAG_STM_WCNSS	0x04
#define DIAG_STM_APPS	0x08

#define BAD_PARAM_RESPONSE_MESSAGE 20

#define DIAG_CMD_VERSION	0
#define DIAG_CMD_DOWNLOAD	0x3A
#define DIAG_CMD_DIAG_SUBSYS	0x4B
#define DIAG_CMD_LOG_ON_DMND	0x78
#define DIAG_CMD_EXT_BUILD	0x7c

#define DIAG_SS_DIAG		0x12
#define DIAG_SS_PARAMS		0x32

#define DIAG_DIAG_MAX_PKT_SZ	0x55
#define DIAG_DIAG_STM		0x20E
#define DIAG_DIAG_POLL		0x03
#define DIAG_DEL_RSP_WRAP	0x04
#define DIAG_DEL_RSP_WRAP_CNT	0x05

#define MODE_CMD	41
#define RESET_ID	2

#define DIAG_STATUS_OPEN (0x00010000)	
#define DIAG_STATUS_CLOSED (0x00020000)	

#define MODE_REALTIME 1
#define MODE_NONREALTIME 0

#define NUM_SMD_DATA_CHANNELS 3
#define NUM_SMD_CONTROL_CHANNELS NUM_SMD_DATA_CHANNELS
#define NUM_SMD_DCI_CHANNELS 1
#define NUM_SMD_CMD_CHANNELS 1
#define NUM_SMD_DCI_CMD_CHANNELS 1

#define NUM_DCI_PROC	(NUM_SMD_DATA_CHANNELS + 1)

#define SMD_DATA_TYPE 0
#define SMD_CNTL_TYPE 1
#define SMD_DCI_TYPE 2
#define SMD_CMD_TYPE 3
#define SMD_DCI_CMD_TYPE 4

#define DIAG_PROC_DCI			1
#define DIAG_PROC_MEMORY_DEVICE		2

#define VOTE_DOWN			0
#define VOTE_UP				1

#define DIAG_TS_SIZE	50

extern int diag_max_reg;
extern int diag_threshold_reg;

#define APPEND_DEBUG(ch) \
do {							\
	diag_debug_buf[diag_debug_buf_idx] = ch; \
	(diag_debug_buf_idx < 1023) ? \
	(diag_debug_buf_idx++) : (diag_debug_buf_idx = 0); \
} while (0)

enum remote_procs {
	MDM = 1,
	MDM2 = 2,
	MDM3 = 3,
	MDM4 = 4,
	QSC = 5,
};

struct diag_pkt_header_t {
	uint8_t cmd_code;
	uint8_t subsys_id;
	uint16_t subsys_cmd_code;
} __packed;

struct diag_master_table {
	uint16_t cmd_code;
	uint16_t subsys_id;
	uint32_t client_id;
	uint16_t cmd_code_lo;
	uint16_t cmd_code_hi;
	int process_id;
};

struct bindpkt_params_per_process {
	
	char sync_obj_name[MAX_SYNC_OBJ_NAME_SIZE];
	uint32_t count;	
	struct bindpkt_params *params; 
};

struct bindpkt_params {
	uint16_t cmd_code;
	uint16_t subsys_id;
	uint16_t cmd_code_lo;
	uint16_t cmd_code_hi;
	
	uint16_t proc_id;
	uint32_t event_id;
	uint32_t log_code;
	
	uint32_t client_id;
};

struct diag_write_device {
	void *buf;
	int length;
};

struct diag_client_map {
	char name[20];
	int pid;
	int timeout;
};

struct real_time_vote_t {
	uint16_t proc;
	uint8_t real_time_vote;
};

#ifndef CONFIG_DIAG_OVER_USB
struct diag_request {
	char *buf;
	int length;
	int actual;
	int status;
	void *context;
};
#endif

struct diag_smd_info {
	int peripheral;	
	int type;	
	uint16_t peripheral_mask;
	int encode_hdlc; 

	smd_channel_t *ch;
	smd_channel_t *ch_save;

	struct mutex smd_ch_mutex;

	int in_busy_1;
	int in_busy_2;
	spinlock_t in_busy_lock;

	unsigned char *buf_in_1;
	unsigned char *buf_in_2;

	unsigned char *buf_in_1_raw;
	unsigned char *buf_in_2_raw;

	unsigned int buf_in_1_size;
	unsigned int buf_in_2_size;

	unsigned int buf_in_1_raw_size;
	unsigned int buf_in_2_raw_size;

	unsigned int buf_full;
	unsigned int buf_release;

	struct diag_request *write_ptr_1;
	struct diag_request *write_ptr_2;

	struct workqueue_struct *wq;

	struct work_struct diag_read_smd_work;
	struct work_struct diag_notify_update_smd_work;
	int notify_context;
	struct work_struct diag_general_smd_work;
	int general_context;

	int (*process_smd_read_data)(struct diag_smd_info *smd_info,
						void *buf, int num_bytes);
};

struct diagchar_dev {

	
	unsigned int major;
	unsigned int minor_start;
	int num;
	struct cdev *cdev;
	char *name;
	int dropped_count;
	struct class *diagchar_class;
	struct device *diag_dev;
	int ref_count;
	struct mutex diagchar_mutex;
	wait_queue_head_t wait_q;
	wait_queue_head_t smd_wait_q;
	struct diag_client_map *client_map;
	int *data_ready;
	int num_clients;
	int polling_reg_flag;
	struct diag_write_device *buf_tbl;
	unsigned int buf_tbl_size;
	int use_device_tree;
	int supports_separate_cmdrsp;
	int supports_apps_hdlc_encoding;
	
	int stm_state_requested[NUM_STM_PROCESSORS];
	
	int stm_state[NUM_STM_PROCESSORS];
	
	int peripheral_supports_stm[NUM_SMD_CONTROL_CHANNELS];
	
	struct list_head dci_req_list;
	struct list_head dci_client_list;
	int dci_tag;
	int dci_client_id;
	struct mutex dci_mutex;
	int num_dci_client;
	unsigned char *apps_dci_buf;
	int dci_state;
	struct workqueue_struct *diag_dci_wq;
#if defined(CONFIG_DIAG_SDIO_PIPE) || defined(CONFIG_DIAGFWD_BRIDGE_CODE)
	struct cdev *cdev_mdm;
	int num_mdmclients;
	struct mutex diagcharmdm_mutex;
	wait_queue_head_t mdmwait_q;
	struct diag_client_map *mdmclient_map;
	int *mdmdata_ready;
	int mdm_logging_process_id;
	unsigned char *user_space_mdm_data;

	struct cdev *cdev_qsc;
	int num_qscclients;
	struct mutex diagcharqsc_mutex;
	wait_queue_head_t qscwait_q;
	struct diag_client_map *qscclient_map;
	int *qscdata_ready;
	int qsc_logging_process_id;
	unsigned char *user_space_qsc_data;
#endif

	
	unsigned int itemsize;
	unsigned int poolsize;
	unsigned int itemsize_hdlc;
	unsigned int poolsize_hdlc;
	unsigned int itemsize_user;
	unsigned int poolsize_user;
	unsigned int itemsize_write_struct;
	unsigned int poolsize_write_struct;
	unsigned int itemsize_dci;
	unsigned int poolsize_dci;
	unsigned int debug_flag;
	
	mempool_t *diagpool;
	mempool_t *diag_hdlc_pool;
	mempool_t *diag_user_pool;
	mempool_t *diag_write_struct_pool;
	mempool_t *diag_dci_pool;
	spinlock_t diag_mem_lock;
	int count;
	int count_hdlc_pool;
	int count_user_pool;
	int count_write_struct_pool;
	int count_dci_pool;
	int used;
	
	struct mutex diag_cntl_mutex;
	struct diag_ctrl_event_mask *event_mask;
	struct diag_ctrl_log_mask *log_mask;
	struct diag_ctrl_msg_mask *msg_mask;
	struct diag_ctrl_feature_mask *feature_mask;
	struct mutex log_mask_mutex;
	
	struct diag_smd_info smd_data[NUM_SMD_DATA_CHANNELS];
	struct diag_smd_info smd_cntl[NUM_SMD_CONTROL_CHANNELS];
	struct diag_smd_info smd_dci[NUM_SMD_DCI_CHANNELS];
	struct diag_smd_info smd_cmd[NUM_SMD_CMD_CHANNELS];
	struct diag_smd_info smd_dci_cmd[NUM_SMD_DCI_CMD_CHANNELS];
	int rcvd_feature_mask[NUM_SMD_CONTROL_CHANNELS];
	int separate_cmdrsp[NUM_SMD_CONTROL_CHANNELS];
	unsigned char *usb_buf_out;
	unsigned char *apps_rsp_buf;
	unsigned char *user_space_data_buf;
	
	unsigned char *buf_msg_mask_update;
	unsigned char *buf_log_mask_update;
	unsigned char *buf_event_mask_update;
	unsigned char *buf_feature_mask_update;
	int read_len_legacy;
	struct mutex diag_hdlc_mutex;
	unsigned char *hdlc_buf;
	unsigned hdlc_count;
	unsigned hdlc_escape;
	int in_busy_pktdata;
	
	int real_time_mode;
	int real_time_update_busy;
	uint16_t proc_active_mask;
	uint16_t proc_rt_vote_mask;
	struct mutex real_time_mutex;
	struct work_struct diag_real_time_work;
	struct workqueue_struct *diag_real_time_wq;
#if DIAG_XPST
	unsigned char nohdlc;
	unsigned char in_busy_dmrounter;
	struct mutex smd_lock;
	unsigned char init_done;
	unsigned char is2ARM11;
	int debug_dmbytes_recv;
#endif
#ifdef CONFIG_DIAG_OVER_USB
	int usb_connected;
	struct usb_diag_ch *legacy_ch;
	struct work_struct diag_proc_hdlc_work;
	struct work_struct diag_read_work;
	struct work_struct diag_usb_connect_work;
	struct work_struct diag_usb_disconnect_work;
#endif
	struct workqueue_struct *diag_wq;
	struct workqueue_struct *diag_usb_wq;
	struct wake_lock wake_lock;
	struct work_struct diag_drain_work;
	struct workqueue_struct *diag_cntl_wq;
	uint8_t *msg_masks;
	uint8_t msg_status;
	uint8_t *log_masks;
	uint8_t log_status;
	uint8_t *event_masks;
	uint8_t event_status;
	uint8_t log_on_demand_support;
	struct diag_master_table *table;
	uint8_t *pkt_buf;
	int pkt_length;
	uint8_t *dci_pkt_buf; 
	uint32_t dci_pkt_length;
	int in_busy_dcipktdata;
	struct diag_request *usb_read_ptr;
	struct diag_request *write_ptr_svc;
	int logging_mode;
	int mask_check;
	int logging_process_id;
	struct task_struct *socket_process;
	struct task_struct *callback_process;

#ifdef CONFIG_DIAG_SDIO_PIPE
	unsigned char *buf_in_sdio;
	unsigned char *usb_buf_mdm_out;
	struct sdio_channel *sdio_ch;
	int read_len_mdm;
	int in_busy_sdio;
	struct usb_diag_ch *mdm_ch;
	struct work_struct diag_read_mdm_work;
	struct workqueue_struct *diag_sdio_wq;
	struct work_struct diag_read_sdio_work;
	struct work_struct diag_close_sdio_work;
	struct diag_request *usb_read_mdm_ptr;
	struct diag_request *write_ptr_mdm;
#endif
#ifdef CONFIG_DIAGFWD_BRIDGE_CODE
	
	struct work_struct diag_connect_work;
	struct work_struct diag_disconnect_work;
	
	int lcid;
	unsigned char *buf_in_smux;
	int in_busy_smux;
	int diag_smux_enabled;
	int smux_connected;
	struct diag_request *write_ptr_mdm;
#endif
	int qxdm2sd_drop;
	int qxdmusb_drop;
	struct timeval st0;
	struct timeval st1;
	
	spinlock_t ws_lock;
	int ws_ref_count;
	int copy_count;
};

extern struct diag_bridge_dev *diag_bridge;
extern struct diag_hsic_dev *diag_hsic;
extern struct diagchar_dev *driver;

#define DIAG_DBG_READ	1
#define DIAG_DBG_WRITE	2
#define DIAG_DBG_DROP	3

extern unsigned diag7k_debug_mask;
extern unsigned diag9k_debug_mask;
#define DIAGFWD_7K_RAWDATA(buf, src, flag) \
	__diagfwd_dbg_raw_data(buf, src, flag, diag7k_debug_mask)
#define DIAGFWD_9K_RAWDATA(buf, src, flag) \
	__diagfwd_dbg_raw_data(buf, src, flag, diag9k_debug_mask)
void __diagfwd_dbg_raw_data(void *buf, const char *src, unsigned dbg_flag, unsigned mask);

extern int wrap_enabled;
extern uint16_t wrap_count;
#if defined(CONFIG_ARCH_MSM8X60) || defined(CONFIG_ARCH_MSM8960) \
   || defined(CONFIG_ARCH_MSM8974) || defined(CONFIG_ARCH_DUMMY)
#define    SMDDIAG_NAME "DIAG"
#else
#define    SMDDIAG_NAME "SMD_DIAG"
#endif
extern struct diagchar_dev *driver;
void diag_get_timestamp(char *time_str);
int diag_find_polling_reg(int i);
void check_drain_timer(void);

void check_drain_timer(void);
#endif
