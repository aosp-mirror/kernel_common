/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#define DEBUG

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/debugfs.h>
#include <linux/clk.h>
#include <linux/wakelock.h>
#include <linux/kfifo.h>
#include <linux/of.h>
#include <linux/srcu.h>
#include <mach/msm_ipc_logging.h>
#include <mach/sps.h>
#include <mach/bam_dmux.h>
#include <mach/msm_smsm.h>
#include <mach/subsystem_notif.h>
#include <mach/socinfo.h>
#include <mach/subsystem_restart.h>

#include "bam_dmux_private.h"

#ifdef CONFIG_HTC_DEBUG_RIL_PCN0006_HTC_DUMP_BAM_DMUX_LOG
void bam_dmux_dbg_log_event(const char * event, ...);

#define DBG_MSG_LEN   100UL

#define DBG_MAX_MSG   256UL

#define TIME_BUF_LEN  20

static int bam_dmux_htc_debug_enable = 1;
static int bam_dmux_htc_debug_dump = 1;
static int bam_dmux_htc_debug_dump_lines = DBG_MAX_MSG;
static int bam_dmux_htc_debug_print = 0;
module_param_named(bam_dmux_htc_debug_enable, bam_dmux_htc_debug_enable,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(bam_dmux_htc_debug_dump, bam_dmux_htc_debug_dump,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(bam_dmux_htc_debug_dump_lines, bam_dmux_htc_debug_dump_lines,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);
module_param_named(bam_dmux_htc_debug_print, bam_dmux_htc_debug_print,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);

static struct {
	char     (buf[DBG_MAX_MSG])[DBG_MSG_LEN];   
	unsigned idx;   
	rwlock_t lck;   
} dbg_bam_dmux = {
	.idx = 0,
	.lck = __RW_LOCK_UNLOCKED(lck)
};
#endif

#define BAM_CH_LOCAL_OPEN       0x1
#define BAM_CH_REMOTE_OPEN      0x2
#define BAM_CH_IN_RESET         0x4

#define LOW_WATERMARK		2
#define HIGH_WATERMARK		4
#define DEFAULT_POLLING_MIN_SLEEP (950)
#define MAX_POLLING_SLEEP (6050)
#define MIN_POLLING_SLEEP (950)

static int msm_bam_dmux_debug_enable;
module_param_named(debug_enable, msm_bam_dmux_debug_enable,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);
int POLLING_MIN_SLEEP = 2950;
module_param_named(min_sleep, POLLING_MIN_SLEEP,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);
int POLLING_MAX_SLEEP = 3050;
module_param_named(max_sleep, POLLING_MAX_SLEEP,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);
static int POLLING_INACTIVITY = 1;
module_param_named(inactivity, POLLING_INACTIVITY,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);
int bam_adaptive_timer_enabled;
module_param_named(adaptive_timer_enabled,
			bam_adaptive_timer_enabled,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);

static struct bam_ops_if bam_default_ops = {
	
	.smsm_change_state_ptr = &smsm_change_state,
	.smsm_get_state_ptr = &smsm_get_state,
	.smsm_state_cb_register_ptr = &smsm_state_cb_register,
	.smsm_state_cb_deregister_ptr = &smsm_state_cb_deregister,

	
	.sps_connect_ptr = &sps_connect,
	.sps_disconnect_ptr = &sps_disconnect,
	.sps_register_bam_device_ptr = &sps_register_bam_device,
	.sps_deregister_bam_device_ptr = &sps_deregister_bam_device,
	.sps_alloc_endpoint_ptr = &sps_alloc_endpoint,
	.sps_free_endpoint_ptr = &sps_free_endpoint,
	.sps_set_config_ptr = &sps_set_config,
	.sps_get_config_ptr = &sps_get_config,
	.sps_device_reset_ptr = &sps_device_reset,
	.sps_register_event_ptr = &sps_register_event,
	.sps_transfer_one_ptr = &sps_transfer_one,
	.sps_get_iovec_ptr = &sps_get_iovec,
	.sps_get_unused_desc_num_ptr = &sps_get_unused_desc_num,

	.dma_to = DMA_TO_DEVICE,
	.dma_from = DMA_FROM_DEVICE,
};
static struct bam_ops_if *bam_ops = &bam_default_ops;

#if defined(DEBUG)
static uint32_t bam_dmux_read_cnt;
static uint32_t bam_dmux_write_cnt;
static uint32_t bam_dmux_write_cpy_cnt;
static uint32_t bam_dmux_write_cpy_bytes;
static uint32_t bam_dmux_tx_sps_failure_cnt;
static uint32_t bam_dmux_tx_stall_cnt;
static atomic_t bam_dmux_ack_out_cnt = ATOMIC_INIT(0);
static atomic_t bam_dmux_ack_in_cnt = ATOMIC_INIT(0);
static atomic_t bam_dmux_a2_pwr_cntl_in_cnt = ATOMIC_INIT(0);

#define DBG(x...) do {		                 \
		if (msm_bam_dmux_debug_enable)  \
			pr_debug(x);	         \
	} while (0)

#define DBG_INC_READ_CNT(x) do {	                               \
		bam_dmux_read_cnt += (x);                             \
		if (msm_bam_dmux_debug_enable)                        \
			pr_debug("%s: total read bytes %u\n",          \
				 __func__, bam_dmux_read_cnt);        \
	} while (0)

#define DBG_INC_WRITE_CNT(x)  do {	                               \
		bam_dmux_write_cnt += (x);                            \
		if (msm_bam_dmux_debug_enable)                        \
			pr_debug("%s: total written bytes %u\n",       \
				 __func__, bam_dmux_write_cnt);       \
	} while (0)

#define DBG_INC_WRITE_CPY(x)  do {	                                     \
		bam_dmux_write_cpy_bytes += (x);                            \
		bam_dmux_write_cpy_cnt++;                                   \
		if (msm_bam_dmux_debug_enable)                              \
			pr_debug("%s: total write copy cnt %u, bytes %u\n",  \
				 __func__, bam_dmux_write_cpy_cnt,          \
				 bam_dmux_write_cpy_bytes);                 \
	} while (0)

#define DBG_INC_TX_SPS_FAILURE_CNT() do {	\
		bam_dmux_tx_sps_failure_cnt++;		\
} while (0)

#define DBG_INC_TX_STALL_CNT() do { \
	bam_dmux_tx_stall_cnt++; \
} while (0)

#define DBG_INC_ACK_OUT_CNT() \
	atomic_inc(&bam_dmux_ack_out_cnt)

#define DBG_INC_A2_POWER_CONTROL_IN_CNT() \
	atomic_inc(&bam_dmux_a2_pwr_cntl_in_cnt)

#define DBG_INC_ACK_IN_CNT() \
	atomic_inc(&bam_dmux_ack_in_cnt)
#else
#define DBG(x...) do { } while (0)
#define DBG_INC_READ_CNT(x...) do { } while (0)
#define DBG_INC_WRITE_CNT(x...) do { } while (0)
#define DBG_INC_WRITE_CPY(x...) do { } while (0)
#define DBG_INC_TX_SPS_FAILURE_CNT() do { } while (0)
#define DBG_INC_TX_STALL_CNT() do { } while (0)
#define DBG_INC_ACK_OUT_CNT() do { } while (0)
#define DBG_INC_A2_POWER_CONTROL_IN_CNT() \
	do { } while (0)
#define DBG_INC_ACK_IN_CNT() do { } while (0)
#endif

struct bam_ch_info {
	uint32_t status;
	void (*notify)(void *, int, unsigned long);
	void *priv;
	spinlock_t lock;
	struct platform_device *pdev;
	char name[BAM_DMUX_CH_NAME_MAX_LEN];
	int num_tx_pkts;
	int use_wm;
};

#define A2_NUM_PIPES		6
#define A2_SUMMING_THRESHOLD	4096
#define A2_PHYS_BASE		0x124C2000
#define A2_PHYS_SIZE		0x2000
#define DEFAULT_NUM_BUFFERS	32

#ifndef A2_BAM_IRQ
#define A2_BAM_IRQ -1
#endif

static void *a2_phys_base;
static uint32_t a2_phys_size;
static int a2_bam_irq;
static struct sps_bam_props a2_props;
static u32 a2_device_handle;
static struct sps_pipe *bam_tx_pipe;
static struct sps_pipe *bam_rx_pipe;
static struct sps_connect tx_connection;
static struct sps_connect rx_connection;
static struct sps_mem_buffer tx_desc_mem_buf;
static struct sps_mem_buffer rx_desc_mem_buf;
static struct sps_register_event tx_register_event;
static struct sps_register_event rx_register_event;
static bool satellite_mode;
static uint32_t num_buffers;
static unsigned long long last_rx_pkt_timestamp;

static struct bam_ch_info bam_ch[BAM_DMUX_NUM_CHANNELS];
static int bam_mux_initialized;

static int polling_mode;
static unsigned long rx_timer_interval;

static LIST_HEAD(bam_rx_pool);
static DEFINE_MUTEX(bam_rx_pool_mutexlock);
static int bam_rx_pool_len;
static LIST_HEAD(bam_tx_pool);
static DEFINE_SPINLOCK(bam_tx_pool_spinlock);
static DEFINE_MUTEX(bam_pdev_mutexlock);

static void notify_all(int event, unsigned long data);
static void bam_mux_write_done(struct work_struct *work);
static void handle_bam_mux_cmd(struct work_struct *work);
static void rx_timer_work_func(struct work_struct *work);
static void queue_rx_work_func(struct work_struct *work);

static DECLARE_WORK(rx_timer_work, rx_timer_work_func);
static DECLARE_WORK(queue_rx_work, queue_rx_work_func);

static struct workqueue_struct *bam_mux_rx_workqueue;
static struct workqueue_struct *bam_mux_tx_workqueue;

static struct srcu_struct bam_dmux_srcu;

#define UL_TIMEOUT_DELAY 1000	
#define ENABLE_DISCONNECT_ACK	0x1
#define SHUTDOWN_TIMEOUT_MS	1000
#define UL_WAKEUP_TIMEOUT_MS	2000
static void toggle_apps_ack(void);
static void reconnect_to_bam(void);
static void disconnect_to_bam(void);
static void ul_wakeup(void);
static void ul_timeout(struct work_struct *work);
static void vote_dfab(void);
static void unvote_dfab(void);
static void kickoff_ul_wakeup_func(struct work_struct *work);
static void grab_wakelock(void);
static void release_wakelock(void);

static int bam_is_connected;
static DEFINE_MUTEX(wakeup_lock);
static struct completion ul_wakeup_ack_completion;
static struct completion bam_connection_completion;
static struct delayed_work ul_timeout_work;
static int ul_packet_written;
static atomic_t ul_ondemand_vote = ATOMIC_INIT(0);
static struct clk *dfab_clk, *xo_clk;
static DEFINE_RWLOCK(ul_wakeup_lock);
static DECLARE_WORK(kickoff_ul_wakeup, kickoff_ul_wakeup_func);
static int bam_connection_is_active;
static int wait_for_ack;
static struct wake_lock bam_wakelock;
static int a2_pc_disabled;
static DEFINE_MUTEX(dfab_status_lock);
static int dfab_is_on;
static int wait_for_dfab;
static struct completion dfab_unvote_completion;
static DEFINE_SPINLOCK(wakelock_reference_lock);
static int wakelock_reference_count;
static int a2_pc_disabled_wakelock_skipped;
static int disconnect_ack = 1;
static LIST_HEAD(bam_other_notify_funcs);
static DEFINE_MUTEX(smsm_cb_lock);
static DEFINE_MUTEX(delayed_ul_vote_lock);
static int need_delayed_ul_vote;
static int power_management_only_mode;
static int in_ssr;
static int ssr_skipped_disconnect;
static struct completion shutdown_completion;

struct outside_notify_func {
	void (*notify)(void *, int, unsigned long);
	void *priv;
	struct list_head list_node;
};

static int restart_notifier_cb(struct notifier_block *this,
				unsigned long code,
				void *data);

static struct notifier_block restart_notifier = {
	.notifier_call = restart_notifier_cb,
};
static int in_global_reset;

#define bam_ch_is_open(x)						\
	(bam_ch[(x)].status == (BAM_CH_LOCAL_OPEN | BAM_CH_REMOTE_OPEN))

#define bam_ch_is_local_open(x)			\
	(bam_ch[(x)].status & BAM_CH_LOCAL_OPEN)

#define bam_ch_is_remote_open(x)			\
	(bam_ch[(x)].status & BAM_CH_REMOTE_OPEN)

#define bam_ch_is_in_reset(x)			\
	(bam_ch[(x)].status & BAM_CH_IN_RESET)

struct kfifo bam_dmux_state_log;
static int bam_dmux_uplink_vote;
static int bam_dmux_power_state;

static void *bam_ipc_log_txt;

#define BAM_IPC_LOG_PAGES 5


#ifdef CONFIG_HTC_DEBUG_RIL_PCN0006_HTC_DUMP_BAM_DMUX_LOG
#define BAM_DMUX_LOG(fmt, args...) \
do { \
	if (bam_ipc_log_txt) { \
		ipc_log_string(bam_ipc_log_txt, \
		"<DMUX> %c%c%c%c %c%c%c%c%d " fmt, \
		a2_pc_disabled ? 'D' : 'd', \
		in_global_reset ? 'R' : 'r', \
		bam_dmux_power_state ? 'P' : 'p', \
		bam_connection_is_active ? 'A' : 'a', \
		bam_dmux_uplink_vote ? 'V' : 'v', \
		bam_is_connected ?  'U' : 'u', \
		wait_for_ack ? 'W' : 'w', \
		ul_wakeup_ack_completion.done ? 'A' : 'a', \
		atomic_read(&ul_ondemand_vote), \
		args); \
	} \
	if (bam_dmux_htc_debug_enable) { \
		bam_dmux_dbg_log_event( \
		"<DMUX> %c%c%c%c %c%c%c%c%d " fmt, \
		a2_pc_disabled ? 'D' : 'd', \
		in_global_reset ? 'R' : 'r', \
		bam_dmux_power_state ? 'P' : 'p', \
		bam_connection_is_active ? 'A' : 'a', \
		bam_dmux_uplink_vote ? 'V' : 'v', \
		bam_is_connected ?  'U' : 'u', \
		wait_for_ack ? 'W' : 'w', \
		ul_wakeup_ack_completion.done ? 'A' : 'a', \
		atomic_read(&ul_ondemand_vote), \
		args); \
	} \
} while (0)
#else
#define BAM_DMUX_LOG(fmt, args...) \
do { \
	if (bam_ipc_log_txt) { \
		ipc_log_string(bam_ipc_log_txt, \
		"<DMUX> %c%c%c%c %c%c%c%c%d%c " fmt, \
		a2_pc_disabled ? 'D' : 'd', \
		in_global_reset ? 'R' : 'r', \
		bam_dmux_power_state ? 'P' : 'p', \
		bam_connection_is_active ? 'A' : 'a', \
		bam_dmux_uplink_vote ? 'V' : 'v', \
		bam_is_connected ?  'U' : 'u', \
		wait_for_ack ? 'W' : 'w', \
		ul_wakeup_ack_completion.done ? 'A' : 'a', \
		atomic_read(&ul_ondemand_vote), \
		disconnect_ack ? 'D' : 'd', \
		args); \
	} \
} while (0)
#endif

#define DMUX_LOG_KERR(fmt, args...) \
do { \
	BAM_DMUX_LOG(fmt, args); \
	pr_err(fmt, args); \
} while (0)

static inline void set_tx_timestamp(struct tx_pkt_info *pkt)
{
	unsigned long long t_now;

	t_now = sched_clock();
	pkt->ts_nsec = do_div(t_now, 1000000000U);
	pkt->ts_sec = (unsigned)t_now;
}

static inline void verify_tx_queue_is_empty(const char *func)
{
	unsigned long flags;
	struct tx_pkt_info *info;
	int reported = 0;

	spin_lock_irqsave(&bam_tx_pool_spinlock, flags);
	list_for_each_entry(info, &bam_tx_pool, list_node) {
		if (!reported) {
			BAM_DMUX_LOG("%s: tx pool not empty\n", func);
			if (!in_global_reset)
				pr_err("%s: tx pool not empty\n", func);
			reported = 1;
		}
		BAM_DMUX_LOG("%s: node=%p ts=%u.%09lu\n", __func__,
			&info->list_node, info->ts_sec, info->ts_nsec);
		if (!in_global_reset)
			pr_err("%s: node=%p ts=%u.%09lu\n", __func__,
			&info->list_node, info->ts_sec, info->ts_nsec);
	}
	spin_unlock_irqrestore(&bam_tx_pool_spinlock, flags);
}

static void __queue_rx(gfp_t alloc_flags)
{
	void *ptr;
	struct rx_pkt_info *info;
	int ret;
	int rx_len_cached;

	mutex_lock(&bam_rx_pool_mutexlock);
	rx_len_cached = bam_rx_pool_len;
	mutex_unlock(&bam_rx_pool_mutexlock);

	while (bam_connection_is_active && rx_len_cached < num_buffers) {
		if (in_global_reset)
			goto fail;

		info = kmalloc(sizeof(struct rx_pkt_info), alloc_flags);
		if (!info) {
			DMUX_LOG_KERR(
			"%s: unable to alloc rx_pkt_info w/ flags %x, will retry later\n",
								__func__,
								alloc_flags);
			goto fail;
		}

		INIT_WORK(&info->work, handle_bam_mux_cmd);

		info->skb = __dev_alloc_skb(BUFFER_SIZE, alloc_flags);
		if (info->skb == NULL) {
			DMUX_LOG_KERR(
				"%s: unable to alloc skb w/ flags %x, will retry later\n",
								__func__,
								alloc_flags);
			goto fail_info;
		}
		ptr = skb_put(info->skb, BUFFER_SIZE);

		info->dma_address = dma_map_single(NULL, ptr, BUFFER_SIZE,
							bam_ops->dma_from);
		if (info->dma_address == 0 || info->dma_address == ~0) {
			DMUX_LOG_KERR("%s: dma_map_single failure %p for %p\n",
				__func__, (void *)info->dma_address, ptr);
			goto fail_skb;
		}

		mutex_lock(&bam_rx_pool_mutexlock);
		list_add_tail(&info->list_node, &bam_rx_pool);
		rx_len_cached = ++bam_rx_pool_len;
		ret = bam_ops->sps_transfer_one_ptr(bam_rx_pipe,
				info->dma_address, BUFFER_SIZE, info, 0);
		if (ret) {
			list_del(&info->list_node);
			rx_len_cached = --bam_rx_pool_len;
			mutex_unlock(&bam_rx_pool_mutexlock);
			DMUX_LOG_KERR("%s: sps_transfer_one failed %d\n",
				__func__, ret);

			dma_unmap_single(NULL, info->dma_address, BUFFER_SIZE,
						bam_ops->dma_from);

			goto fail_skb;
		}
		mutex_unlock(&bam_rx_pool_mutexlock);

	}
	return;

fail_skb:
	dev_kfree_skb_any(info->skb);

fail_info:
	kfree(info);

fail:
	if (!in_global_reset) {
		DMUX_LOG_KERR("%s: rescheduling\n", __func__);
		schedule_work(&queue_rx_work);
	}
}

static void queue_rx(void)
{
	__queue_rx(GFP_NOWAIT | __GFP_NOWARN);
}

static void queue_rx_work_func(struct work_struct *work)
{
	__queue_rx(GFP_KERNEL);
}

static void bam_mux_process_data(struct sk_buff *rx_skb)
{
	unsigned long flags;
	struct bam_mux_hdr *rx_hdr;
	unsigned long event_data;

	rx_hdr = (struct bam_mux_hdr *)rx_skb->data;

	rx_skb->data = (unsigned char *)(rx_hdr + 1);
	rx_skb->tail = rx_skb->data + rx_hdr->pkt_len;
	rx_skb->len = rx_hdr->pkt_len;
	rx_skb->truesize = rx_hdr->pkt_len + sizeof(struct sk_buff);

	event_data = (unsigned long)(rx_skb);

	spin_lock_irqsave(&bam_ch[rx_hdr->ch_id].lock, flags);
	if (bam_ch[rx_hdr->ch_id].notify)
		bam_ch[rx_hdr->ch_id].notify(
			bam_ch[rx_hdr->ch_id].priv, BAM_DMUX_RECEIVE,
							event_data);
	else
		dev_kfree_skb_any(rx_skb);
	spin_unlock_irqrestore(&bam_ch[rx_hdr->ch_id].lock, flags);

	queue_rx();
}

static inline void handle_bam_mux_cmd_open(struct bam_mux_hdr *rx_hdr)
{
	unsigned long flags;
	int ret;

	mutex_lock(&bam_pdev_mutexlock);
	if (in_global_reset) {
		BAM_DMUX_LOG("%s: open cid %d aborted due to ssr\n",
				__func__, rx_hdr->ch_id);
		mutex_unlock(&bam_pdev_mutexlock);
		queue_rx();
		return;
	}
	spin_lock_irqsave(&bam_ch[rx_hdr->ch_id].lock, flags);
	bam_ch[rx_hdr->ch_id].status |= BAM_CH_REMOTE_OPEN;
	bam_ch[rx_hdr->ch_id].num_tx_pkts = 0;
	spin_unlock_irqrestore(&bam_ch[rx_hdr->ch_id].lock, flags);
	ret = platform_device_add(bam_ch[rx_hdr->ch_id].pdev);
	if (ret)
		pr_err("%s: platform_device_add() error: %d\n",
				__func__, ret);
	mutex_unlock(&bam_pdev_mutexlock);
	queue_rx();
}

static void handle_bam_mux_cmd(struct work_struct *work)
{
	unsigned long flags;
	struct bam_mux_hdr *rx_hdr;
	struct rx_pkt_info *info;
	struct sk_buff *rx_skb;

	info = container_of(work, struct rx_pkt_info, work);
	rx_skb = info->skb;
	dma_unmap_single(NULL, info->dma_address, BUFFER_SIZE,
			bam_ops->dma_from);
	kfree(info);

	rx_hdr = (struct bam_mux_hdr *)rx_skb->data;

	DBG_INC_READ_CNT(sizeof(struct bam_mux_hdr));
	DBG("%s: magic %x reserved %d cmd %d pad %d ch %d len %d\n", __func__,
			rx_hdr->magic_num, rx_hdr->reserved, rx_hdr->cmd,
			rx_hdr->pad_len, rx_hdr->ch_id, rx_hdr->pkt_len);
	if (rx_hdr->magic_num != BAM_MUX_HDR_MAGIC_NO) {
		DMUX_LOG_KERR("%s: dropping invalid hdr. magic %x"
			" reserved %d cmd %d"
			" pad %d ch %d len %d\n", __func__,
			rx_hdr->magic_num, rx_hdr->reserved, rx_hdr->cmd,
			rx_hdr->pad_len, rx_hdr->ch_id, rx_hdr->pkt_len);
		dev_kfree_skb_any(rx_skb);
		queue_rx();
		return;
	}

	if (rx_hdr->ch_id >= BAM_DMUX_NUM_CHANNELS) {
		DMUX_LOG_KERR("%s: dropping invalid LCID %d"
			" reserved %d cmd %d"
			" pad %d ch %d len %d\n", __func__,
			rx_hdr->ch_id, rx_hdr->reserved, rx_hdr->cmd,
			rx_hdr->pad_len, rx_hdr->ch_id, rx_hdr->pkt_len);
		dev_kfree_skb_any(rx_skb);
		queue_rx();
		return;
	}

	switch (rx_hdr->cmd) {
	case BAM_MUX_HDR_CMD_DATA:
		DBG_INC_READ_CNT(rx_hdr->pkt_len);
		bam_mux_process_data(rx_skb);
		break;
	case BAM_MUX_HDR_CMD_OPEN:
		BAM_DMUX_LOG("%s: opening cid %d PC enabled\n", __func__,
				rx_hdr->ch_id);
		handle_bam_mux_cmd_open(rx_hdr);
		if (!(rx_hdr->reserved & ENABLE_DISCONNECT_ACK)) {
			BAM_DMUX_LOG("%s: deactivating disconnect ack\n",
								__func__);
			disconnect_ack = 0;
		}
		dev_kfree_skb_any(rx_skb);
		break;
	case BAM_MUX_HDR_CMD_OPEN_NO_A2_PC:
		BAM_DMUX_LOG("%s: opening cid %d PC disabled\n", __func__,
				rx_hdr->ch_id);

		if (!a2_pc_disabled) {
			a2_pc_disabled = 1;
			ul_wakeup();
		}

		handle_bam_mux_cmd_open(rx_hdr);
		dev_kfree_skb_any(rx_skb);
		break;
	case BAM_MUX_HDR_CMD_CLOSE:
		
		BAM_DMUX_LOG("%s: closing cid %d\n", __func__,
				rx_hdr->ch_id);
		mutex_lock(&bam_pdev_mutexlock);
		if (in_global_reset) {
			BAM_DMUX_LOG("%s: close cid %d aborted due to ssr\n",
					__func__, rx_hdr->ch_id);
			mutex_unlock(&bam_pdev_mutexlock);
			break;
		}
		spin_lock_irqsave(&bam_ch[rx_hdr->ch_id].lock, flags);
		bam_ch[rx_hdr->ch_id].status &= ~BAM_CH_REMOTE_OPEN;
		spin_unlock_irqrestore(&bam_ch[rx_hdr->ch_id].lock, flags);
		platform_device_unregister(bam_ch[rx_hdr->ch_id].pdev);
		bam_ch[rx_hdr->ch_id].pdev =
			platform_device_alloc(bam_ch[rx_hdr->ch_id].name, 2);
		if (!bam_ch[rx_hdr->ch_id].pdev)
			pr_err("%s: platform_device_alloc failed\n", __func__);
		mutex_unlock(&bam_pdev_mutexlock);
		dev_kfree_skb_any(rx_skb);
		queue_rx();
		break;
	default:
		DMUX_LOG_KERR("%s: dropping invalid hdr. magic %x"
			   " reserved %d cmd %d pad %d ch %d len %d\n",
			__func__, rx_hdr->magic_num, rx_hdr->reserved,
			rx_hdr->cmd, rx_hdr->pad_len, rx_hdr->ch_id,
			rx_hdr->pkt_len);
		dev_kfree_skb_any(rx_skb);
		queue_rx();
		return;
	}
}

static int bam_mux_write_cmd(void *data, uint32_t len)
{
	int rc;
	struct tx_pkt_info *pkt;
	dma_addr_t dma_address;
	unsigned long flags;

	pkt = kmalloc(sizeof(struct tx_pkt_info), GFP_ATOMIC);
	if (pkt == NULL) {
		pr_err("%s: mem alloc for tx_pkt_info failed\n", __func__);
		rc = -ENOMEM;
		return rc;
	}

	dma_address = dma_map_single(NULL, data, len,
					bam_ops->dma_to);
	if (!dma_address) {
		pr_err("%s: dma_map_single() failed\n", __func__);
		kfree(pkt);
		rc = -ENOMEM;
		return rc;
	}
	pkt->skb = (struct sk_buff *)(data);
	pkt->len = len;
	pkt->dma_address = dma_address;
	pkt->is_cmd = 1;
	set_tx_timestamp(pkt);
	INIT_WORK(&pkt->work, bam_mux_write_done);
	spin_lock_irqsave(&bam_tx_pool_spinlock, flags);
	list_add_tail(&pkt->list_node, &bam_tx_pool);
	rc = bam_ops->sps_transfer_one_ptr(bam_tx_pipe, dma_address, len,
				pkt, SPS_IOVEC_FLAG_EOT);
	if (rc) {
		DMUX_LOG_KERR("%s sps_transfer_one failed rc=%d\n",
			__func__, rc);
		list_del(&pkt->list_node);
		DBG_INC_TX_SPS_FAILURE_CNT();
		spin_unlock_irqrestore(&bam_tx_pool_spinlock, flags);
		dma_unmap_single(NULL, pkt->dma_address,
					pkt->len,
					bam_ops->dma_to);
		kfree(pkt);
	} else {
		spin_unlock_irqrestore(&bam_tx_pool_spinlock, flags);
	}

	ul_packet_written = 1;
	return rc;
}

static void bam_mux_write_done(struct work_struct *work)
{
	struct sk_buff *skb;
	struct bam_mux_hdr *hdr;
	struct tx_pkt_info *info;
	struct tx_pkt_info *info_expected;
	unsigned long event_data;
	unsigned long flags;

	if (in_global_reset)
		return;

	info = container_of(work, struct tx_pkt_info, work);

	spin_lock_irqsave(&bam_tx_pool_spinlock, flags);
	info_expected = list_first_entry(&bam_tx_pool,
			struct tx_pkt_info, list_node);
	if (unlikely(info != info_expected)) {
		struct tx_pkt_info *errant_pkt;

		DMUX_LOG_KERR("%s: bam_tx_pool mismatch .next=%p,"
				" list_node=%p, ts=%u.%09lu\n",
				__func__, bam_tx_pool.next, &info->list_node,
				info->ts_sec, info->ts_nsec
				);

		list_for_each_entry(errant_pkt, &bam_tx_pool, list_node) {
			DMUX_LOG_KERR("%s: node=%p ts=%u.%09lu\n", __func__,
			&errant_pkt->list_node, errant_pkt->ts_sec,
			errant_pkt->ts_nsec);

		}
		spin_unlock_irqrestore(&bam_tx_pool_spinlock, flags);
		BUG();
	}
	list_del(&info->list_node);
	spin_unlock_irqrestore(&bam_tx_pool_spinlock, flags);

	if (info->is_cmd) {
		kfree(info->skb);
		kfree(info);
		return;
	}
	skb = info->skb;
	kfree(info);
	hdr = (struct bam_mux_hdr *)skb->data;
	DBG_INC_WRITE_CNT(skb->len);
	event_data = (unsigned long)(skb);
	spin_lock_irqsave(&bam_ch[hdr->ch_id].lock, flags);
	bam_ch[hdr->ch_id].num_tx_pkts--;
	spin_unlock_irqrestore(&bam_ch[hdr->ch_id].lock, flags);
	if (bam_ch[hdr->ch_id].notify)
		bam_ch[hdr->ch_id].notify(
			bam_ch[hdr->ch_id].priv, BAM_DMUX_WRITE_DONE,
							event_data);
	else
		dev_kfree_skb_any(skb);
}

int msm_bam_dmux_write(uint32_t id, struct sk_buff *skb)
{
	int rc = 0;
	struct bam_mux_hdr *hdr;
	unsigned long flags;
	struct sk_buff *new_skb = NULL;
	dma_addr_t dma_address;
	struct tx_pkt_info *pkt;
	int rcu_id;

	if (id >= BAM_DMUX_NUM_CHANNELS)
		return -EINVAL;
	if (!skb)
		return -EINVAL;
	if (!bam_mux_initialized)
		return -ENODEV;

	rcu_id = srcu_read_lock(&bam_dmux_srcu);
	if (in_global_reset) {
		BAM_DMUX_LOG("%s: In SSR... ch_id[%d]\n", __func__, id);
		srcu_read_unlock(&bam_dmux_srcu, rcu_id);
		return -EFAULT;
	}

	DBG("%s: writing to ch %d len %d\n", __func__, id, skb->len);
	spin_lock_irqsave(&bam_ch[id].lock, flags);
	if (!bam_ch_is_open(id)) {
		spin_unlock_irqrestore(&bam_ch[id].lock, flags);
		pr_err("%s: port not open: %d\n", __func__, bam_ch[id].status);
		srcu_read_unlock(&bam_dmux_srcu, rcu_id);
		return -ENODEV;
	}

	if (bam_ch[id].use_wm &&
	    (bam_ch[id].num_tx_pkts >= HIGH_WATERMARK)) {
		spin_unlock_irqrestore(&bam_ch[id].lock, flags);
		pr_err("%s: watermark exceeded: %d\n", __func__, id);
		srcu_read_unlock(&bam_dmux_srcu, rcu_id);
		return -EAGAIN;
	}
	spin_unlock_irqrestore(&bam_ch[id].lock, flags);

	read_lock(&ul_wakeup_lock);
	if (!bam_is_connected) {
		read_unlock(&ul_wakeup_lock);
		ul_wakeup();
		if (unlikely(in_global_reset == 1)) {
			srcu_read_unlock(&bam_dmux_srcu, rcu_id);
			return -EFAULT;
		}
		read_lock(&ul_wakeup_lock);
		notify_all(BAM_DMUX_UL_CONNECTED, (unsigned long)(NULL));
	}

	if ((skb->len & 0x3) && (skb_tailroom(skb) < (4 - (skb->len & 0x3)))) {
		
		new_skb = skb_copy_expand(skb, skb_headroom(skb),
					  4 - (skb->len & 0x3), GFP_ATOMIC);
		if (new_skb == NULL) {
			pr_err("%s: cannot allocate skb\n", __func__);
			goto write_fail;
		}
		dev_kfree_skb_any(skb);
		skb = new_skb;
		DBG_INC_WRITE_CPY(skb->len);
	}

	hdr = (struct bam_mux_hdr *)skb_push(skb, sizeof(struct bam_mux_hdr));

	hdr->magic_num = BAM_MUX_HDR_MAGIC_NO;
	hdr->cmd = BAM_MUX_HDR_CMD_DATA;
	hdr->reserved = 0;
	hdr->ch_id = id;
	hdr->pkt_len = skb->len - sizeof(struct bam_mux_hdr);
	if (skb->len & 0x3)
		skb_put(skb, 4 - (skb->len & 0x3));

	hdr->pad_len = skb->len - (sizeof(struct bam_mux_hdr) + hdr->pkt_len);

	DBG("%s: data %p, tail %p skb len %d pkt len %d pad len %d\n",
	    __func__, skb->data, skb->tail, skb->len,
	    hdr->pkt_len, hdr->pad_len);

	pkt = kmalloc(sizeof(struct tx_pkt_info), GFP_ATOMIC);
	if (pkt == NULL) {
		pr_err("%s: mem alloc for tx_pkt_info failed\n", __func__);
		goto write_fail2;
	}

	dma_address = dma_map_single(NULL, skb->data, skb->len,
					bam_ops->dma_to);
	if (!dma_address) {
		pr_err("%s: dma_map_single() failed\n", __func__);
		goto write_fail3;
	}
	pkt->skb = skb;
	pkt->dma_address = dma_address;
	pkt->is_cmd = 0;
	set_tx_timestamp(pkt);
	INIT_WORK(&pkt->work, bam_mux_write_done);
	spin_lock_irqsave(&bam_tx_pool_spinlock, flags);
	list_add_tail(&pkt->list_node, &bam_tx_pool);
	rc = bam_ops->sps_transfer_one_ptr(bam_tx_pipe, dma_address, skb->len,
				pkt, SPS_IOVEC_FLAG_EOT);
	if (rc) {
		DMUX_LOG_KERR("%s sps_transfer_one failed rc=%d\n",
			__func__, rc);
		list_del(&pkt->list_node);
		DBG_INC_TX_SPS_FAILURE_CNT();
		spin_unlock_irqrestore(&bam_tx_pool_spinlock, flags);
		dma_unmap_single(NULL, pkt->dma_address,
					pkt->skb->len,	bam_ops->dma_to);
		kfree(pkt);
		if (new_skb)
			dev_kfree_skb_any(new_skb);
	} else {
		spin_unlock_irqrestore(&bam_tx_pool_spinlock, flags);
		spin_lock_irqsave(&bam_ch[id].lock, flags);
		bam_ch[id].num_tx_pkts++;
		spin_unlock_irqrestore(&bam_ch[id].lock, flags);
	}
	ul_packet_written = 1;
	read_unlock(&ul_wakeup_lock);
	srcu_read_unlock(&bam_dmux_srcu, rcu_id);
	return rc;

write_fail3:
	kfree(pkt);
write_fail2:
	skb_pull(skb, sizeof(struct bam_mux_hdr));
	if (new_skb)
		dev_kfree_skb_any(new_skb);
write_fail:
	read_unlock(&ul_wakeup_lock);
	srcu_read_unlock(&bam_dmux_srcu, rcu_id);
	return -ENOMEM;
}

int msm_bam_dmux_open(uint32_t id, void *priv,
			void (*notify)(void *, int, unsigned long))
{
	struct bam_mux_hdr *hdr;
	unsigned long flags;
	int rc = 0;

	DBG("%s: opening ch %d\n", __func__, id);
	if (!bam_mux_initialized) {
		DBG("%s: not inititialized\n", __func__);
		return -ENODEV;
	}
	if (id >= BAM_DMUX_NUM_CHANNELS) {
		pr_err("%s: invalid channel id %d\n", __func__, id);
		return -EINVAL;
	}
	if (notify == NULL) {
		pr_err("%s: notify function is NULL\n", __func__);
		return -EINVAL;
	}

	hdr = kmalloc(sizeof(struct bam_mux_hdr), GFP_KERNEL);
	if (hdr == NULL) {
		pr_err("%s: hdr kmalloc failed. ch: %d\n", __func__, id);
		return -ENOMEM;
	}
	spin_lock_irqsave(&bam_ch[id].lock, flags);
	if (bam_ch_is_open(id)) {
		DBG("%s: Already opened %d\n", __func__, id);
		spin_unlock_irqrestore(&bam_ch[id].lock, flags);
		kfree(hdr);
		goto open_done;
	}
	if (!bam_ch_is_remote_open(id)) {
		DBG("%s: Remote not open; ch: %d\n", __func__, id);
		spin_unlock_irqrestore(&bam_ch[id].lock, flags);
		kfree(hdr);
		return -ENODEV;
	}

	bam_ch[id].notify = notify;
	bam_ch[id].priv = priv;
	bam_ch[id].status |= BAM_CH_LOCAL_OPEN;
	bam_ch[id].num_tx_pkts = 0;
	bam_ch[id].use_wm = 0;
	spin_unlock_irqrestore(&bam_ch[id].lock, flags);

	read_lock(&ul_wakeup_lock);
	if (!bam_is_connected) {
		read_unlock(&ul_wakeup_lock);
		ul_wakeup();
		if (unlikely(in_global_reset == 1)) {
			kfree(hdr);
			return -EFAULT;
		}
		read_lock(&ul_wakeup_lock);
		notify_all(BAM_DMUX_UL_CONNECTED, (unsigned long)(NULL));
	}

	hdr->magic_num = BAM_MUX_HDR_MAGIC_NO;
	hdr->cmd = BAM_MUX_HDR_CMD_OPEN;
	hdr->reserved = 0;
	hdr->ch_id = id;
	hdr->pkt_len = 0;
	hdr->pad_len = 0;

	rc = bam_mux_write_cmd((void *)hdr, sizeof(struct bam_mux_hdr));
	read_unlock(&ul_wakeup_lock);

open_done:
	DBG("%s: opened ch %d\n", __func__, id);
	return rc;
}

int msm_bam_dmux_close(uint32_t id)
{
	struct bam_mux_hdr *hdr;
	unsigned long flags;
	int rc;

	if (id >= BAM_DMUX_NUM_CHANNELS)
		return -EINVAL;
	DBG("%s: closing ch %d\n", __func__, id);
	if (!bam_mux_initialized)
		return -ENODEV;

	read_lock(&ul_wakeup_lock);
	if (!bam_is_connected && !bam_ch_is_in_reset(id)) {
		read_unlock(&ul_wakeup_lock);
		ul_wakeup();
		if (unlikely(in_global_reset == 1))
			return -EFAULT;
		read_lock(&ul_wakeup_lock);
		notify_all(BAM_DMUX_UL_CONNECTED, (unsigned long)(NULL));
	}

	spin_lock_irqsave(&bam_ch[id].lock, flags);
	bam_ch[id].notify = NULL;
	bam_ch[id].priv = NULL;
	bam_ch[id].status &= ~BAM_CH_LOCAL_OPEN;
	spin_unlock_irqrestore(&bam_ch[id].lock, flags);

	if (bam_ch_is_in_reset(id)) {
		read_unlock(&ul_wakeup_lock);
		bam_ch[id].status &= ~BAM_CH_IN_RESET;
		return 0;
	}

	hdr = kmalloc(sizeof(struct bam_mux_hdr), GFP_ATOMIC);
	if (hdr == NULL) {
		pr_err("%s: hdr kmalloc failed. ch: %d\n", __func__, id);
		read_unlock(&ul_wakeup_lock);
		return -ENOMEM;
	}
	hdr->magic_num = BAM_MUX_HDR_MAGIC_NO;
	hdr->cmd = BAM_MUX_HDR_CMD_CLOSE;
	hdr->reserved = 0;
	hdr->ch_id = id;
	hdr->pkt_len = 0;
	hdr->pad_len = 0;

	rc = bam_mux_write_cmd((void *)hdr, sizeof(struct bam_mux_hdr));
	read_unlock(&ul_wakeup_lock);

	DBG("%s: closed ch %d\n", __func__, id);
	return rc;
}

int msm_bam_dmux_is_ch_full(uint32_t id)
{
	unsigned long flags;
	int ret;

	if (id >= BAM_DMUX_NUM_CHANNELS)
		return -EINVAL;

	spin_lock_irqsave(&bam_ch[id].lock, flags);
	bam_ch[id].use_wm = 1;
	ret = bam_ch[id].num_tx_pkts >= HIGH_WATERMARK;
	DBG("%s: ch %d num tx pkts=%d, HWM=%d\n", __func__,
	     id, bam_ch[id].num_tx_pkts, ret);
	if (!bam_ch_is_local_open(id)) {
		ret = -ENODEV;
		pr_err("%s: port not open: %d\n", __func__, bam_ch[id].status);
	}
	spin_unlock_irqrestore(&bam_ch[id].lock, flags);

	return ret;
}

int msm_bam_dmux_is_ch_low(uint32_t id)
{
	unsigned long flags;
	int ret;

	if (id >= BAM_DMUX_NUM_CHANNELS)
		return -EINVAL;

	spin_lock_irqsave(&bam_ch[id].lock, flags);
	bam_ch[id].use_wm = 1;
	ret = bam_ch[id].num_tx_pkts <= LOW_WATERMARK;
	DBG("%s: ch %d num tx pkts=%d, LWM=%d\n", __func__,
	     id, bam_ch[id].num_tx_pkts, ret);
	if (!bam_ch_is_local_open(id)) {
		ret = -ENODEV;
		pr_err("%s: port not open: %d\n", __func__, bam_ch[id].status);
	}
	spin_unlock_irqrestore(&bam_ch[id].lock, flags);

	return ret;
}

static void rx_switch_to_interrupt_mode(void)
{
	struct sps_connect cur_rx_conn;
	struct sps_iovec iov;
	struct rx_pkt_info *info;
	int ret;

	ret = bam_ops->sps_get_config_ptr(bam_rx_pipe, &cur_rx_conn);
	if (ret) {
		pr_err("%s: sps_get_config() failed %d\n", __func__, ret);
		goto fail;
	}

	rx_register_event.options = SPS_O_EOT;
	ret = bam_ops->sps_register_event_ptr(bam_rx_pipe, &rx_register_event);
	if (ret) {
		pr_err("%s: sps_register_event() failed %d\n", __func__, ret);
		goto fail;
	}

	cur_rx_conn.options = SPS_O_AUTO_ENABLE |
		SPS_O_EOT | SPS_O_ACK_TRANSFERS;
	ret = bam_ops->sps_set_config_ptr(bam_rx_pipe, &cur_rx_conn);
	if (ret) {
		pr_err("%s: sps_set_config() failed %d\n", __func__, ret);
		goto fail;
	}
	polling_mode = 0;
	complete_all(&shutdown_completion);
	release_wakelock();

	
	while (bam_connection_is_active && !polling_mode) {
		ret = bam_ops->sps_get_iovec_ptr(bam_rx_pipe, &iov);
		if (ret) {
			pr_err("%s: sps_get_iovec failed %d\n",
					__func__, ret);
			break;
		}
		if (iov.addr == 0)
			break;

		mutex_lock(&bam_rx_pool_mutexlock);
		if (unlikely(list_empty(&bam_rx_pool))) {
			DMUX_LOG_KERR("%s: have iovec %p but rx pool empty\n",
				__func__, (void *)iov.addr);
			mutex_unlock(&bam_rx_pool_mutexlock);
			continue;
		}
		info = list_first_entry(&bam_rx_pool, struct rx_pkt_info,
							list_node);
		if (info->dma_address != iov.addr) {
			DMUX_LOG_KERR("%s: iovec %p != dma %p\n",
				__func__,
				(void *)iov.addr,
				(void *)info->dma_address);
			list_for_each_entry(info, &bam_rx_pool, list_node) {
				DMUX_LOG_KERR("%s: dma %p\n", __func__,
					(void *)info->dma_address);
				if (iov.addr == info->dma_address)
					break;
			}
		}
		BUG_ON(info->dma_address != iov.addr);
		list_del(&info->list_node);
		--bam_rx_pool_len;
		mutex_unlock(&bam_rx_pool_mutexlock);
		handle_bam_mux_cmd(&info->work);
	}
	return;

fail:
	pr_err("%s: reverting to polling\n", __func__);
	queue_work_on(0, bam_mux_rx_workqueue, &rx_timer_work);
}

static void store_rx_timestamp(void)
{
	last_rx_pkt_timestamp = sched_clock();
}

static void log_rx_timestamp(void)
{
	unsigned long long t = last_rx_pkt_timestamp;
	unsigned long nanosec_rem;

	nanosec_rem = do_div(t, 1000000000U);
	BAM_DMUX_LOG("Last rx pkt processed at [%6u.%09lu]\n", (unsigned)t,
								nanosec_rem);
}

static void rx_timer_work_func(struct work_struct *work)
{
	struct sps_iovec iov;
	struct rx_pkt_info *info;
	int inactive_cycles = 0;
	int ret;
	u32 buffs_unused, buffs_used;

	BAM_DMUX_LOG("%s: polling start\n", __func__);
	while (bam_connection_is_active) { 
		++inactive_cycles;
		while (bam_connection_is_active) { 
			if (in_global_reset) {
				BAM_DMUX_LOG(
						"%s: polling exit, global reset detected\n",
						__func__);
				return;
			}

			ret = bam_ops->sps_get_iovec_ptr(bam_rx_pipe, &iov);
			if (ret) {
				DMUX_LOG_KERR("%s: sps_get_iovec failed %d\n",
						__func__, ret);
				break;
			}
			if (iov.addr == 0)
				break;
			store_rx_timestamp();
			inactive_cycles = 0;
			mutex_lock(&bam_rx_pool_mutexlock);
			if (unlikely(list_empty(&bam_rx_pool))) {
				DMUX_LOG_KERR(
					"%s: have iovec %p but rx pool empty\n",
					__func__, (void *)iov.addr);
				mutex_unlock(&bam_rx_pool_mutexlock);
				continue;
			}
			info = list_first_entry(&bam_rx_pool,
					struct rx_pkt_info,	list_node);
			if (info->dma_address != iov.addr) {
				DMUX_LOG_KERR("%s: iovec %p != dma %p\n",
					__func__,
					(void *)iov.addr,
					(void *)info->dma_address);
				list_for_each_entry(info, &bam_rx_pool,
						list_node) {
					DMUX_LOG_KERR("%s: dma %p\n", __func__,
						(void *)info->dma_address);
					if (iov.addr == info->dma_address)
						break;
				}
			}
			BUG_ON(info->dma_address != iov.addr);
			list_del(&info->list_node);
			--bam_rx_pool_len;
			mutex_unlock(&bam_rx_pool_mutexlock);
			handle_bam_mux_cmd(&info->work);
		}

		if (inactive_cycles >= POLLING_INACTIVITY) {
			BAM_DMUX_LOG("%s: polling exit, no data\n", __func__);
			rx_switch_to_interrupt_mode();
			break;
		}

		if (bam_adaptive_timer_enabled) {
			usleep_range(rx_timer_interval, rx_timer_interval + 50);

			ret = bam_ops->sps_get_unused_desc_num_ptr(bam_rx_pipe,
						&buffs_unused);

			if (ret) {
				DMUX_LOG_KERR(
					"%s: error getting num buffers unused after sleep\n",
					__func__);

				break;
			}

			buffs_used = num_buffers - buffs_unused;

			if (buffs_unused == 0) {
				rx_timer_interval = MIN_POLLING_SLEEP;
			} else {
				if (buffs_used > 0) {
					rx_timer_interval =
						(2 * num_buffers *
							rx_timer_interval)/
						(3 * buffs_used);
				} else {
					rx_timer_interval =
						MAX_POLLING_SLEEP;
				}
			}

			if (rx_timer_interval > MAX_POLLING_SLEEP)
				rx_timer_interval = MAX_POLLING_SLEEP;
			else if (rx_timer_interval < MIN_POLLING_SLEEP)
				rx_timer_interval = MIN_POLLING_SLEEP;
		} else {
			usleep_range(POLLING_MIN_SLEEP, POLLING_MAX_SLEEP);
		}
	}
}

static void bam_mux_tx_notify(struct sps_event_notify *notify)
{
	struct tx_pkt_info *pkt;

	DBG("%s: event %d notified\n", __func__, notify->event_id);

	if (in_global_reset)
		return;

	switch (notify->event_id) {
	case SPS_EVENT_EOT:
		pkt = notify->data.transfer.user;
		if (!pkt->is_cmd)
			dma_unmap_single(NULL, pkt->dma_address,
						pkt->skb->len,
						bam_ops->dma_to);
		else
			dma_unmap_single(NULL, pkt->dma_address,
						pkt->len,
						bam_ops->dma_to);
		queue_work(bam_mux_tx_workqueue, &pkt->work);
		break;
	default:
		pr_err("%s: recieved unexpected event id %d\n", __func__,
			notify->event_id);
	}
}

static void bam_mux_rx_notify(struct sps_event_notify *notify)
{
	int ret;
	struct sps_connect cur_rx_conn;

	DBG("%s: event %d notified\n", __func__, notify->event_id);

	if (in_global_reset)
		return;

	switch (notify->event_id) {
	case SPS_EVENT_EOT:
		
		if (!polling_mode) {
			ret = bam_ops->sps_get_config_ptr(bam_rx_pipe,
					&cur_rx_conn);
			if (ret) {
				pr_err("%s: sps_get_config() failed %d, interrupts"
					" not disabled\n", __func__, ret);
				break;
			}
			cur_rx_conn.options = SPS_O_AUTO_ENABLE |
				SPS_O_ACK_TRANSFERS | SPS_O_POLL;
			ret = bam_ops->sps_set_config_ptr(bam_rx_pipe,
					&cur_rx_conn);
			if (ret) {
				pr_err("%s: sps_set_config() failed %d, interrupts"
					" not disabled\n", __func__, ret);
				break;
			}
			INIT_COMPLETION(shutdown_completion);
			grab_wakelock();
			polling_mode = 1;
			queue_work_on(0, bam_mux_rx_workqueue, &rx_timer_work);
		}
		break;
	default:
		pr_err("%s: recieved unexpected event id %d\n", __func__,
			notify->event_id);
	}
}

#ifdef CONFIG_DEBUG_FS

static int debug_tbl(char *buf, int max)
{
	int i = 0;
	int j;

	for (j = 0; j < BAM_DMUX_NUM_CHANNELS; ++j) {
		i += scnprintf(buf + i, max - i,
			"ch%02d  local open=%s  remote open=%s\n",
			j, bam_ch_is_local_open(j) ? "Y" : "N",
			bam_ch_is_remote_open(j) ? "Y" : "N");
	}

	return i;
}

static int debug_ul_pkt_cnt(char *buf, int max)
{
	struct list_head *p;
	unsigned long flags;
	int n = 0;

	spin_lock_irqsave(&bam_tx_pool_spinlock, flags);
	__list_for_each(p, &bam_tx_pool) {
		++n;
	}
	spin_unlock_irqrestore(&bam_tx_pool_spinlock, flags);

	return scnprintf(buf, max, "Number of UL packets in flight: %d\n", n);
}

static int debug_stats(char *buf, int max)
{
	int i = 0;

	i += scnprintf(buf + i, max - i,
			"skb read cnt:    %u\n"
			"skb write cnt:   %u\n"
			"skb copy cnt:    %u\n"
			"skb copy bytes:  %u\n"
			"sps tx failures: %u\n"
			"sps tx stalls:   %u\n"
			"rx queue len:    %d\n"
			"a2 ack out cnt:  %d\n"
			"a2 ack in cnt:   %d\n"
			"a2 pwr cntl in:  %d\n",
			bam_dmux_read_cnt,
			bam_dmux_write_cnt,
			bam_dmux_write_cpy_cnt,
			bam_dmux_write_cpy_bytes,
			bam_dmux_tx_sps_failure_cnt,
			bam_dmux_tx_stall_cnt,
			bam_rx_pool_len,
			atomic_read(&bam_dmux_ack_out_cnt),
			atomic_read(&bam_dmux_ack_in_cnt),
			atomic_read(&bam_dmux_a2_pwr_cntl_in_cnt)
			);

	return i;
}

#define DEBUG_BUFMAX 4096
static char debug_buffer[DEBUG_BUFMAX];

static ssize_t debug_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	int (*fill)(char *buf, int max) = file->private_data;
	int bsize = fill(debug_buffer, DEBUG_BUFMAX);
	return simple_read_from_buffer(buf, count, ppos, debug_buffer, bsize);
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}


static const struct file_operations debug_ops = {
	.read = debug_read,
	.open = debug_open,
};

static void debug_create(const char *name, mode_t mode,
				struct dentry *dent,
				int (*fill)(char *buf, int max))
{
	struct dentry *file;

	file = debugfs_create_file(name, mode, dent, fill, &debug_ops);
	if (IS_ERR(file))
		pr_err("%s: debugfs create failed %d\n", __func__,
				(int)PTR_ERR(file));
}

#endif

static void notify_all(int event, unsigned long data)
{
	int i;
	struct list_head *temp;
	struct outside_notify_func *func;

	BAM_DMUX_LOG("%s: event=%d, data=%lu\n", __func__, event, data);

	for (i = 0; i < BAM_DMUX_NUM_CHANNELS; ++i) {
		if (bam_ch_is_open(i))
			bam_ch[i].notify(bam_ch[i].priv, event, data);
	}

	__list_for_each(temp, &bam_other_notify_funcs) {
		func = container_of(temp, struct outside_notify_func,
								list_node);
		func->notify(func->priv, event, data);
	}
}

static void kickoff_ul_wakeup_func(struct work_struct *work)
{
	read_lock(&ul_wakeup_lock);
	if (!bam_is_connected) {
		read_unlock(&ul_wakeup_lock);
		ul_wakeup();
		if (unlikely(in_global_reset == 1))
			return;
		read_lock(&ul_wakeup_lock);
		ul_packet_written = 1;
		notify_all(BAM_DMUX_UL_CONNECTED, (unsigned long)(NULL));
	}
	read_unlock(&ul_wakeup_lock);
}

int msm_bam_dmux_kickoff_ul_wakeup(void)
{
	int is_connected;

	read_lock(&ul_wakeup_lock);
	ul_packet_written = 1;
	is_connected = bam_is_connected;
	if (!is_connected)
		queue_work(bam_mux_tx_workqueue, &kickoff_ul_wakeup);
	read_unlock(&ul_wakeup_lock);

	return is_connected;
}

static void power_vote(int vote)
{
	BAM_DMUX_LOG("%s: curr=%d, vote=%d\n", __func__,
			bam_dmux_uplink_vote, vote);

	if (bam_dmux_uplink_vote == vote)
		BAM_DMUX_LOG("%s: warning - duplicate power vote\n", __func__);

	bam_dmux_uplink_vote = vote;
	if (vote)
		bam_ops->smsm_change_state_ptr(SMSM_APPS_STATE,
			0, SMSM_A2_POWER_CONTROL);
	else
		bam_ops->smsm_change_state_ptr(SMSM_APPS_STATE,
			SMSM_A2_POWER_CONTROL, 0);
}

static inline void ul_powerdown(void)
{
	BAM_DMUX_LOG("%s: powerdown\n", __func__);
	verify_tx_queue_is_empty(__func__);

	if (a2_pc_disabled) {
		wait_for_dfab = 1;
		INIT_COMPLETION(dfab_unvote_completion);
		release_wakelock();
	} else {
		wait_for_ack = 1;
		INIT_COMPLETION(ul_wakeup_ack_completion);
		power_vote(0);
	}
	bam_is_connected = 0;
	notify_all(BAM_DMUX_UL_DISCONNECTED, (unsigned long)(NULL));
}

static inline void ul_powerdown_finish(void)
{
	if (a2_pc_disabled && wait_for_dfab) {
		unvote_dfab();
		complete_all(&dfab_unvote_completion);
		wait_for_dfab = 0;
	}
}

int msm_bam_dmux_ul_power_vote(void)
{
	int is_connected;

	read_lock(&ul_wakeup_lock);
	atomic_inc(&ul_ondemand_vote);
	is_connected = bam_is_connected;
	if (!is_connected)
		queue_work(bam_mux_tx_workqueue, &kickoff_ul_wakeup);
	read_unlock(&ul_wakeup_lock);

	return is_connected;
}

int msm_bam_dmux_ul_power_unvote(void)
{
	int vote;

	read_lock(&ul_wakeup_lock);
	vote = atomic_dec_return(&ul_ondemand_vote);
	if (unlikely(vote) < 0)
		DMUX_LOG_KERR("%s: invalid power vote %d\n", __func__, vote);
	read_unlock(&ul_wakeup_lock);

	return vote == 0;
}

int msm_bam_dmux_reg_notify(void *priv,
			void (*notify)(void *priv, int event_type,
						unsigned long data))
{
	struct outside_notify_func *func;

	if (!notify)
		return -EINVAL;

	func = kmalloc(sizeof(struct outside_notify_func), GFP_KERNEL);
	if (!func)
		return -ENOMEM;

	func->notify = notify;
	func->priv = priv;
	list_add(&func->list_node, &bam_other_notify_funcs);

	return 0;
}

static void ul_timeout(struct work_struct *work)
{
	unsigned long flags;
	int ret;

	if (in_global_reset)
		return;
	ret = write_trylock_irqsave(&ul_wakeup_lock, flags);
	if (!ret) { 
		schedule_delayed_work(&ul_timeout_work,
				msecs_to_jiffies(UL_TIMEOUT_DELAY));
		return;
	}
	if (bam_is_connected) {
		if (!ul_packet_written) {
			spin_lock(&bam_tx_pool_spinlock);
			if (!list_empty(&bam_tx_pool)) {
				struct tx_pkt_info *info;

				info = list_first_entry(&bam_tx_pool,
						struct tx_pkt_info, list_node);
				DMUX_LOG_KERR("%s: UL delayed ts=%u.%09lu\n",
					__func__, info->ts_sec, info->ts_nsec);
				DBG_INC_TX_STALL_CNT();
				ul_packet_written = 1;
			}
			spin_unlock(&bam_tx_pool_spinlock);
		}

		if (ul_packet_written || atomic_read(&ul_ondemand_vote)) {
			BAM_DMUX_LOG("%s: pkt written %d\n",
				__func__, ul_packet_written);
			ul_packet_written = 0;
			schedule_delayed_work(&ul_timeout_work,
					msecs_to_jiffies(UL_TIMEOUT_DELAY));
		} else {
			ul_powerdown();
		}
	}
	write_unlock_irqrestore(&ul_wakeup_lock, flags);
	ul_powerdown_finish();
}

static int ssrestart_check(void)
{
	int ret = 0;

	if (in_global_reset) {
		DMUX_LOG_KERR("%s: modem timeout: already in SSR\n",
			__func__);
		return 1;
	}

	DMUX_LOG_KERR("%s: modem timeout: BAM DMUX disabled for SSR\n",
								__func__);
	pr_err("<DMUX> %c%c%c%c %c%c%c%c%d%c  %s: Record BAM state before SSR\n" ,
		a2_pc_disabled ? 'D' : 'd',
		in_global_reset ? 'R' : 'r',
		bam_dmux_power_state ? 'P' : 'p',
		bam_connection_is_active ? 'A' : 'a',
		bam_dmux_uplink_vote ? 'V' : 'v',
		bam_is_connected ?  'U' : 'u',
		wait_for_ack ? 'W' : 'w',
		ul_wakeup_ack_completion.done ? 'A' : 'a',
		atomic_read(&ul_ondemand_vote),
		disconnect_ack ? 'D' : 'd',
		__func__);
	in_global_reset = 1;
	ret = subsystem_restart("modem");
	if (ret == -ENODEV)
		panic("modem subsystem restart failed\n");
	return 1;
}

static void ul_wakeup(void)
{
	int ret;
	int do_vote_dfab = 0;

	mutex_lock(&wakeup_lock);
	if (bam_is_connected) { 
		BAM_DMUX_LOG("%s Already awake\n", __func__);
		mutex_unlock(&wakeup_lock);
		return;
	}

	if (unlikely(in_global_reset == 1)) {
		mutex_unlock(&wakeup_lock);
		return;
	}

	mutex_lock(&delayed_ul_vote_lock);
	if (unlikely(!bam_mux_initialized)) {
		need_delayed_ul_vote = 1;
		mutex_unlock(&delayed_ul_vote_lock);
		mutex_unlock(&wakeup_lock);
		return;
	}
	mutex_unlock(&delayed_ul_vote_lock);

	if (a2_pc_disabled) {
		if (likely(a2_pc_disabled_wakelock_skipped)) {
			grab_wakelock();
			do_vote_dfab = 1; 
		} else {
			a2_pc_disabled_wakelock_skipped = 1;
		}
		if (wait_for_dfab) {
			ret = wait_for_completion_timeout(
					&dfab_unvote_completion, HZ);
			BUG_ON(ret == 0);
		}
		if (likely(do_vote_dfab))
			vote_dfab();
		schedule_delayed_work(&ul_timeout_work,
				msecs_to_jiffies(UL_TIMEOUT_DELAY));
		bam_is_connected = 1;
		mutex_unlock(&wakeup_lock);
		return;
	}

	if (wait_for_ack) {
		BAM_DMUX_LOG("%s waiting for previous ack\n", __func__);
		ret = wait_for_completion_timeout(
					&ul_wakeup_ack_completion,
					msecs_to_jiffies(UL_WAKEUP_TIMEOUT_MS));
		wait_for_ack = 0;
		if (unlikely(ret == 0) && ssrestart_check()) {
			mutex_unlock(&wakeup_lock);
			BAM_DMUX_LOG("%s timeout previous ack\n", __func__);
			return;
		}
	}
	INIT_COMPLETION(ul_wakeup_ack_completion);
	power_vote(1);
	BAM_DMUX_LOG("%s waiting for wakeup ack\n", __func__);
	ret = wait_for_completion_timeout(&ul_wakeup_ack_completion,
					msecs_to_jiffies(UL_WAKEUP_TIMEOUT_MS));
	if (unlikely(ret == 0) && ssrestart_check()) {
		mutex_unlock(&wakeup_lock);
		BAM_DMUX_LOG("%s timeout wakeup ack\n", __func__);
		return;
	}
	BAM_DMUX_LOG("%s waiting completion\n", __func__);
	ret = wait_for_completion_timeout(&bam_connection_completion,
					msecs_to_jiffies(UL_WAKEUP_TIMEOUT_MS));
	if (unlikely(ret == 0) && ssrestart_check()) {
		mutex_unlock(&wakeup_lock);
		BAM_DMUX_LOG("%s timeout power on\n", __func__);
		return;
	}

	bam_is_connected = 1;
	BAM_DMUX_LOG("%s complete\n", __func__);
	schedule_delayed_work(&ul_timeout_work,
				msecs_to_jiffies(UL_TIMEOUT_DELAY));
	mutex_unlock(&wakeup_lock);
}

static void reconnect_to_bam(void)
{
	int i;

	in_global_reset = 0;
	in_ssr = 0;
	vote_dfab();
	if (!power_management_only_mode) {
		if (ssr_skipped_disconnect) {
			
			bam_ops->sps_disconnect_ptr(bam_tx_pipe);
			bam_ops->sps_disconnect_ptr(bam_rx_pipe);
			__memzero(rx_desc_mem_buf.base, rx_desc_mem_buf.size);
			__memzero(tx_desc_mem_buf.base, tx_desc_mem_buf.size);
		}
		ssr_skipped_disconnect = 0;
		i = bam_ops->sps_device_reset_ptr(a2_device_handle);
		if (i)
			pr_err("%s: device reset failed rc = %d\n", __func__,
									i);
		i = bam_ops->sps_connect_ptr(bam_tx_pipe, &tx_connection);
		if (i)
			pr_err("%s: tx connection failed rc = %d\n", __func__,
									i);
		i = bam_ops->sps_connect_ptr(bam_rx_pipe, &rx_connection);
		if (i)
			pr_err("%s: rx connection failed rc = %d\n", __func__,
									i);
		i = bam_ops->sps_register_event_ptr(bam_tx_pipe,
				&tx_register_event);
		if (i)
			pr_err("%s: tx event reg failed rc = %d\n", __func__,
									i);
		i = bam_ops->sps_register_event_ptr(bam_rx_pipe,
				&rx_register_event);
		if (i)
			pr_err("%s: rx event reg failed rc = %d\n", __func__,
									i);
	}

	bam_connection_is_active = 1;

	if (polling_mode)
		rx_switch_to_interrupt_mode();

	toggle_apps_ack();
	complete_all(&bam_connection_completion);
	if (!power_management_only_mode)
		queue_rx();
}

static void disconnect_to_bam(void)
{
	struct list_head *node;
	struct rx_pkt_info *info;
	unsigned long flags;
	unsigned long time_remaining;

	if (!in_global_reset) {
		time_remaining = wait_for_completion_timeout(
				&shutdown_completion,
				msecs_to_jiffies(SHUTDOWN_TIMEOUT_MS));
		if (time_remaining == 0) {
			DMUX_LOG_KERR("%s: shutdown completion timed out\n",
					__func__);
			log_rx_timestamp();
			ssrestart_check();
		}
	}

	bam_connection_is_active = 0;

	
	write_lock_irqsave(&ul_wakeup_lock, flags);
	if (bam_is_connected) {
		BAM_DMUX_LOG("%s: UL active - forcing powerdown\n", __func__);
		ul_powerdown();
	}
	write_unlock_irqrestore(&ul_wakeup_lock, flags);
	ul_powerdown_finish();

	
	INIT_COMPLETION(bam_connection_completion);

	
	if (!power_management_only_mode) {
		if (likely(!in_ssr)) {
			BAM_DMUX_LOG("%s: disconnect tx\n", __func__);
			bam_ops->sps_disconnect_ptr(bam_tx_pipe);
			BAM_DMUX_LOG("%s: disconnect rx\n", __func__);
			bam_ops->sps_disconnect_ptr(bam_rx_pipe);
			__memzero(rx_desc_mem_buf.base, rx_desc_mem_buf.size);
			__memzero(tx_desc_mem_buf.base, tx_desc_mem_buf.size);
			BAM_DMUX_LOG("%s: device reset\n", __func__);
			sps_device_reset(a2_device_handle);
		} else {
			ssr_skipped_disconnect = 1;
		}
	}
	unvote_dfab();

	mutex_lock(&bam_rx_pool_mutexlock);
	while (!list_empty(&bam_rx_pool)) {
		node = bam_rx_pool.next;
		list_del(node);
		info = container_of(node, struct rx_pkt_info, list_node);
		dma_unmap_single(NULL, info->dma_address, BUFFER_SIZE,
							bam_ops->dma_from);
		dev_kfree_skb_any(info->skb);
		kfree(info);
	}
	bam_rx_pool_len = 0;
	mutex_unlock(&bam_rx_pool_mutexlock);

	if (disconnect_ack)
		toggle_apps_ack();

	verify_tx_queue_is_empty(__func__);
}

static void vote_dfab(void)
{
	int rc;

	BAM_DMUX_LOG("%s\n", __func__);
	mutex_lock(&dfab_status_lock);
	if (dfab_is_on) {
		BAM_DMUX_LOG("%s: dfab is already on\n", __func__);
		mutex_unlock(&dfab_status_lock);
		return;
	}
	if (dfab_clk) {
		rc = clk_prepare_enable(dfab_clk);
		if (rc)
			DMUX_LOG_KERR("bam_dmux vote for dfab failed rc = %d\n",
									rc);
	}
	if (xo_clk) {
		rc = clk_prepare_enable(xo_clk);
		if (rc)
			DMUX_LOG_KERR("bam_dmux vote for xo failed rc = %d\n",
									rc);
	}
	dfab_is_on = 1;
	mutex_unlock(&dfab_status_lock);
}

static void unvote_dfab(void)
{
	BAM_DMUX_LOG("%s\n", __func__);
	mutex_lock(&dfab_status_lock);
	if (!dfab_is_on) {
		DMUX_LOG_KERR("%s: dfab is already off\n", __func__);
		dump_stack();
		mutex_unlock(&dfab_status_lock);
		return;
	}
	if (dfab_clk)
		clk_disable_unprepare(dfab_clk);
	if (xo_clk)
		clk_disable_unprepare(xo_clk);
	dfab_is_on = 0;
	mutex_unlock(&dfab_status_lock);
}

static void grab_wakelock(void)
{
	unsigned long flags;

	spin_lock_irqsave(&wakelock_reference_lock, flags);
	BAM_DMUX_LOG("%s: ref count = %d\n", __func__,
						wakelock_reference_count);
	if (wakelock_reference_count == 0)
		wake_lock(&bam_wakelock);
	++wakelock_reference_count;
	spin_unlock_irqrestore(&wakelock_reference_lock, flags);
}

static void release_wakelock(void)
{
	unsigned long flags;

	spin_lock_irqsave(&wakelock_reference_lock, flags);
	if (wakelock_reference_count == 0) {
		DMUX_LOG_KERR("%s: bam_dmux wakelock not locked\n", __func__);
		dump_stack();
		spin_unlock_irqrestore(&wakelock_reference_lock, flags);
		return;
	}
	BAM_DMUX_LOG("%s: ref count = %d\n", __func__,
						wakelock_reference_count);
	--wakelock_reference_count;
	if (wakelock_reference_count == 0)
		wake_unlock(&bam_wakelock);
	spin_unlock_irqrestore(&wakelock_reference_lock, flags);
}

static int restart_notifier_cb(struct notifier_block *this,
				unsigned long code,
				void *data)
{
	int i;
	struct list_head *node;
	struct tx_pkt_info *info;
	int temp_remote_status;
	unsigned long flags;

	if (code == SUBSYS_BEFORE_SHUTDOWN) {
		BAM_DMUX_LOG("%s: begin\n", __func__);
		in_global_reset = 1;
		in_ssr = 1;
		
		synchronize_srcu(&bam_dmux_srcu);
		BAM_DMUX_LOG("%s: ssr signaling complete\n", __func__);
		flush_workqueue(bam_mux_rx_workqueue);
	}
	if (code != SUBSYS_AFTER_SHUTDOWN)
		return NOTIFY_DONE;

	
	write_lock_irqsave(&ul_wakeup_lock, flags);
	if (bam_is_connected) {
		ul_powerdown();
		wait_for_ack = 0;
	}
	power_vote(0);
	write_unlock_irqrestore(&ul_wakeup_lock, flags);
	ul_powerdown_finish();
	a2_pc_disabled = 0;
	a2_pc_disabled_wakelock_skipped = 0;
	disconnect_ack = 1;

	
	mutex_lock(&bam_pdev_mutexlock);
	for (i = 0; i < BAM_DMUX_NUM_CHANNELS; ++i) {
		temp_remote_status = bam_ch_is_remote_open(i);
		bam_ch[i].status &= ~BAM_CH_REMOTE_OPEN;
		bam_ch[i].num_tx_pkts = 0;
		if (bam_ch_is_local_open(i))
			bam_ch[i].status |= BAM_CH_IN_RESET;
		if (temp_remote_status) {
			platform_device_unregister(bam_ch[i].pdev);
			bam_ch[i].pdev = platform_device_alloc(
						bam_ch[i].name, 2);
		}
	}
	mutex_unlock(&bam_pdev_mutexlock);

	
	spin_lock_irqsave(&bam_tx_pool_spinlock, flags);
	while (!list_empty(&bam_tx_pool)) {
		node = bam_tx_pool.next;
		list_del(node);
		info = container_of(node, struct tx_pkt_info,
							list_node);
		if (!info->is_cmd) {
			dma_unmap_single(NULL, info->dma_address,
						info->skb->len,
						bam_ops->dma_to);
			dev_kfree_skb_any(info->skb);
		} else {
			dma_unmap_single(NULL, info->dma_address,
						info->len,
						bam_ops->dma_to);
			kfree(info->skb);
		}
		kfree(info);
	}
	spin_unlock_irqrestore(&bam_tx_pool_spinlock, flags);

	BAM_DMUX_LOG("%s: complete\n", __func__);
	return NOTIFY_DONE;
}

static int bam_init(void)
{
	u32 h;
	dma_addr_t dma_addr;
	int ret;
	void *a2_virt_addr;
	int skip_iounmap = 0;

	vote_dfab();
	
	a2_virt_addr = ioremap_nocache((unsigned long)(a2_phys_base),
							a2_phys_size);
	if (!a2_virt_addr) {
		pr_err("%s: ioremap failed\n", __func__);
		ret = -ENOMEM;
		goto ioremap_failed;
	}
	a2_props.phys_addr = (u32)(a2_phys_base);
	a2_props.virt_addr = a2_virt_addr;
	a2_props.virt_size = a2_phys_size;
	a2_props.irq = a2_bam_irq;
	a2_props.options = SPS_BAM_OPT_IRQ_WAKEUP;
	a2_props.num_pipes = A2_NUM_PIPES;
	a2_props.summing_threshold = A2_SUMMING_THRESHOLD;
	a2_props.constrained_logging = true;
	a2_props.logging_number = 1;
	if (cpu_is_msm9615() || satellite_mode)
		a2_props.manage = SPS_BAM_MGR_DEVICE_REMOTE;
	
	ret = bam_ops->sps_register_bam_device_ptr(&a2_props, &h);
	if (ret < 0) {
		pr_err("%s: register bam error %d\n", __func__, ret);
		goto register_bam_failed;
	}
	a2_device_handle = h;

	bam_tx_pipe = bam_ops->sps_alloc_endpoint_ptr();
	if (bam_tx_pipe == NULL) {
		pr_err("%s: tx alloc endpoint failed\n", __func__);
		ret = -ENOMEM;
		goto tx_alloc_endpoint_failed;
	}
	ret = bam_ops->sps_get_config_ptr(bam_tx_pipe, &tx_connection);
	if (ret) {
		pr_err("%s: tx get config failed %d\n", __func__, ret);
		goto tx_get_config_failed;
	}

	tx_connection.source = SPS_DEV_HANDLE_MEM;
	tx_connection.src_pipe_index = 0;
	tx_connection.destination = h;
	tx_connection.dest_pipe_index = 4;
	tx_connection.mode = SPS_MODE_DEST;
	tx_connection.options = SPS_O_AUTO_ENABLE | SPS_O_EOT;
	tx_desc_mem_buf.size = 0x800; 
	tx_desc_mem_buf.base = dma_alloc_coherent(NULL, tx_desc_mem_buf.size,
							&dma_addr, 0);
	if (tx_desc_mem_buf.base == NULL) {
		pr_err("%s: tx memory alloc failed\n", __func__);
		ret = -ENOMEM;
		goto tx_get_config_failed;
	}
	tx_desc_mem_buf.phys_base = dma_addr;
	memset(tx_desc_mem_buf.base, 0x0, tx_desc_mem_buf.size);
	tx_connection.desc = tx_desc_mem_buf;
	tx_connection.event_thresh = 0x10;

	ret = bam_ops->sps_connect_ptr(bam_tx_pipe, &tx_connection);
	if (ret < 0) {
		pr_err("%s: tx connect error %d\n", __func__, ret);
		goto tx_connect_failed;
	}

	bam_rx_pipe = bam_ops->sps_alloc_endpoint_ptr();
	if (bam_rx_pipe == NULL) {
		pr_err("%s: rx alloc endpoint failed\n", __func__);
		ret = -ENOMEM;
		goto rx_alloc_endpoint_failed;
	}
	ret = bam_ops->sps_get_config_ptr(bam_rx_pipe, &rx_connection);
	if (ret) {
		pr_err("%s: rx get config failed %d\n", __func__, ret);
		goto rx_get_config_failed;
	}

	rx_connection.source = h;
	rx_connection.src_pipe_index = 5;
	rx_connection.destination = SPS_DEV_HANDLE_MEM;
	rx_connection.dest_pipe_index = 1;
	rx_connection.mode = SPS_MODE_SRC;
	rx_connection.options = SPS_O_AUTO_ENABLE | SPS_O_EOT |
					SPS_O_ACK_TRANSFERS;
	rx_desc_mem_buf.size = 0x800; 
	rx_desc_mem_buf.base = dma_alloc_coherent(NULL, rx_desc_mem_buf.size,
							&dma_addr, 0);
	if (rx_desc_mem_buf.base == NULL) {
		pr_err("%s: rx memory alloc failed\n", __func__);
		ret = -ENOMEM;
		goto rx_mem_failed;
	}
	rx_desc_mem_buf.phys_base = dma_addr;
	memset(rx_desc_mem_buf.base, 0x0, rx_desc_mem_buf.size);
	rx_connection.desc = rx_desc_mem_buf;
	rx_connection.event_thresh = 0x10;

	ret = bam_ops->sps_connect_ptr(bam_rx_pipe, &rx_connection);
	if (ret < 0) {
		pr_err("%s: rx connect error %d\n", __func__, ret);
		goto rx_connect_failed;
	}

	tx_register_event.options = SPS_O_EOT;
	tx_register_event.mode = SPS_TRIGGER_CALLBACK;
	tx_register_event.xfer_done = NULL;
	tx_register_event.callback = bam_mux_tx_notify;
	tx_register_event.user = NULL;
	ret = bam_ops->sps_register_event_ptr(bam_tx_pipe, &tx_register_event);
	if (ret < 0) {
		pr_err("%s: tx register event error %d\n", __func__, ret);
		goto rx_event_reg_failed;
	}

	rx_register_event.options = SPS_O_EOT;
	rx_register_event.mode = SPS_TRIGGER_CALLBACK;
	rx_register_event.xfer_done = NULL;
	rx_register_event.callback = bam_mux_rx_notify;
	rx_register_event.user = NULL;
	ret = bam_ops->sps_register_event_ptr(bam_rx_pipe, &rx_register_event);
	if (ret < 0) {
		pr_err("%s: tx register event error %d\n", __func__, ret);
		goto rx_event_reg_failed;
	}

	mutex_lock(&delayed_ul_vote_lock);
	bam_mux_initialized = 1;
	if (need_delayed_ul_vote) {
		need_delayed_ul_vote = 0;
		msm_bam_dmux_kickoff_ul_wakeup();
	}
	mutex_unlock(&delayed_ul_vote_lock);
	toggle_apps_ack();
	bam_connection_is_active = 1;
	complete_all(&bam_connection_completion);
	queue_rx();
	return 0;

rx_event_reg_failed:
	bam_ops->sps_disconnect_ptr(bam_rx_pipe);
rx_connect_failed:
	dma_free_coherent(NULL, rx_desc_mem_buf.size, rx_desc_mem_buf.base,
				rx_desc_mem_buf.phys_base);
rx_mem_failed:
rx_get_config_failed:
	bam_ops->sps_free_endpoint_ptr(bam_rx_pipe);
rx_alloc_endpoint_failed:
	bam_ops->sps_disconnect_ptr(bam_tx_pipe);
tx_connect_failed:
	dma_free_coherent(NULL, tx_desc_mem_buf.size, tx_desc_mem_buf.base,
				tx_desc_mem_buf.phys_base);
tx_get_config_failed:
	bam_ops->sps_free_endpoint_ptr(bam_tx_pipe);
tx_alloc_endpoint_failed:
	bam_ops->sps_deregister_bam_device_ptr(h);
	skip_iounmap = 1;
register_bam_failed:
	if (!skip_iounmap)
		iounmap(a2_virt_addr);
ioremap_failed:
	
	return ret;
}

static int bam_init_fallback(void)
{
	u32 h;
	int ret;
	void *a2_virt_addr;

	
	a2_virt_addr = ioremap_nocache((unsigned long)(a2_phys_base),
							a2_phys_size);
	if (!a2_virt_addr) {
		pr_err("%s: ioremap failed\n", __func__);
		ret = -ENOMEM;
		goto ioremap_failed;
	}
	a2_props.phys_addr = (u32)(a2_phys_base);
	a2_props.virt_addr = a2_virt_addr;
	a2_props.virt_size = a2_phys_size;
	a2_props.irq = a2_bam_irq;
	a2_props.options = SPS_BAM_OPT_IRQ_WAKEUP;
	a2_props.num_pipes = A2_NUM_PIPES;
	a2_props.summing_threshold = A2_SUMMING_THRESHOLD;
	if (cpu_is_msm9615() || satellite_mode)
		a2_props.manage = SPS_BAM_MGR_DEVICE_REMOTE;
	ret = bam_ops->sps_register_bam_device_ptr(&a2_props, &h);
	if (ret < 0) {
		pr_err("%s: register bam error %d\n", __func__, ret);
		goto register_bam_failed;
	}
	a2_device_handle = h;

	mutex_lock(&delayed_ul_vote_lock);
	bam_mux_initialized = 1;
	if (need_delayed_ul_vote) {
		need_delayed_ul_vote = 0;
		msm_bam_dmux_kickoff_ul_wakeup();
	}
	mutex_unlock(&delayed_ul_vote_lock);
	toggle_apps_ack();

	power_management_only_mode = 1;
	bam_connection_is_active = 1;
	complete_all(&bam_connection_completion);

	return 0;

register_bam_failed:
	iounmap(a2_virt_addr);
ioremap_failed:
	return ret;
}

static void msm9615_bam_init(void)
{
	int ret = 0;

	ret = bam_init();
	if (ret) {
		ret = bam_init_fallback();
		if (ret)
			pr_err("%s: bam init fallback failed: %d",
					__func__, ret);
	}
}

static void toggle_apps_ack(void)
{
	static unsigned int clear_bit; 

	if (in_global_reset) {
		BAM_DMUX_LOG("%s: skipped due to SSR\n", __func__);
		return;
	}

	BAM_DMUX_LOG("%s: apps ack %d->%d\n", __func__,
			clear_bit & 0x1, ~clear_bit & 0x1);
	bam_ops->smsm_change_state_ptr(SMSM_APPS_STATE,
				clear_bit & SMSM_A2_POWER_CONTROL_ACK,
				~clear_bit & SMSM_A2_POWER_CONTROL_ACK);
	clear_bit = ~clear_bit;
	DBG_INC_ACK_OUT_CNT();
}

static void bam_dmux_smsm_cb(void *priv, uint32_t old_state, uint32_t new_state)
{
	static int last_processed_state;

	mutex_lock(&smsm_cb_lock);
	bam_dmux_power_state = new_state & SMSM_A2_POWER_CONTROL ? 1 : 0;
	DBG_INC_A2_POWER_CONTROL_IN_CNT();
	BAM_DMUX_LOG("%s: 0x%08x -> 0x%08x\n", __func__, old_state,
			new_state);
	if (last_processed_state == (new_state & SMSM_A2_POWER_CONTROL)) {
		BAM_DMUX_LOG("%s: already processed this state\n", __func__);
		mutex_unlock(&smsm_cb_lock);
		return;
	}

	last_processed_state = new_state & SMSM_A2_POWER_CONTROL;

	if (bam_mux_initialized && new_state & SMSM_A2_POWER_CONTROL) {
		BAM_DMUX_LOG("%s: reconnect\n", __func__);
		grab_wakelock();
		reconnect_to_bam();
	} else if (bam_mux_initialized &&
					!(new_state & SMSM_A2_POWER_CONTROL)) {
		BAM_DMUX_LOG("%s: disconnect\n", __func__);
		disconnect_to_bam();
		release_wakelock();
	} else if (new_state & SMSM_A2_POWER_CONTROL) {
		BAM_DMUX_LOG("%s: init\n", __func__);
		grab_wakelock();
		if (cpu_is_msm9615())
			msm9615_bam_init();
		else
			bam_init();
	} else {
		BAM_DMUX_LOG("%s: bad state change\n", __func__);
		pr_err("%s: unsupported state change\n", __func__);
	}
	mutex_unlock(&smsm_cb_lock);

}

static void bam_dmux_smsm_ack_cb(void *priv, uint32_t old_state,
						uint32_t new_state)
{
	DBG_INC_ACK_IN_CNT();
	BAM_DMUX_LOG("%s: 0x%08x -> 0x%08x\n", __func__, old_state,
			new_state);
	complete_all(&ul_wakeup_ack_completion);
}

void msm_bam_dmux_set_bam_ops(struct bam_ops_if *ops)
{
	if (ops != NULL)
		bam_ops = ops;
	else
		bam_ops = &bam_default_ops;
}
EXPORT_SYMBOL(msm_bam_dmux_set_bam_ops);

void msm_bam_dmux_deinit(void)
{
	restart_notifier_cb(NULL, SUBSYS_BEFORE_SHUTDOWN, NULL);
	restart_notifier_cb(NULL, SUBSYS_AFTER_SHUTDOWN, NULL);
}
EXPORT_SYMBOL(msm_bam_dmux_deinit);

void msm_bam_dmux_reinit(void)
{
	bam_ops->smsm_state_cb_register_ptr(SMSM_MODEM_STATE,
			SMSM_A2_POWER_CONTROL,
			bam_dmux_smsm_cb, NULL);
	bam_ops->smsm_state_cb_register_ptr(SMSM_MODEM_STATE,
			SMSM_A2_POWER_CONTROL_ACK,
			bam_dmux_smsm_ack_cb, NULL);
	bam_mux_initialized = 0;
	bam_init();
}
EXPORT_SYMBOL(msm_bam_dmux_reinit);

static int bam_dmux_probe(struct platform_device *pdev)
{
	int rc;
	struct resource *r;

	DBG("%s probe called\n", __func__);
	if (bam_mux_initialized)
		return 0;

	if (pdev->dev.of_node) {
		r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!r) {
			pr_err("%s: reg field missing\n", __func__);
			return -ENODEV;
		}
		a2_phys_base = (void *)(r->start);
		a2_phys_size = (uint32_t)(resource_size(r));
		a2_bam_irq = platform_get_irq(pdev, 0);
		if (a2_bam_irq == -ENXIO) {
			pr_err("%s: irq field missing\n", __func__);
			return -ENODEV;
		}
		satellite_mode = of_property_read_bool(pdev->dev.of_node,
						"qcom,satellite-mode");

		rc = of_property_read_u32(pdev->dev.of_node,
						"qcom,rx-ring-size",
						&num_buffers);
		if (rc) {
			DBG("%s: falling back to num_buffs default, rc:%d\n",
							__func__, rc);
			num_buffers = DEFAULT_NUM_BUFFERS;
		}

		DBG("%s: base:%p size:%x irq:%d satellite:%d num_buffs:%d\n",
							__func__,
							a2_phys_base,
							a2_phys_size,
							a2_bam_irq,
							satellite_mode,
							num_buffers);
	} else { 
		a2_phys_base = (void *)(A2_PHYS_BASE);
		a2_phys_size = A2_PHYS_SIZE;
		a2_bam_irq = A2_BAM_IRQ;
		num_buffers = DEFAULT_NUM_BUFFERS;
	}

	xo_clk = clk_get(&pdev->dev, "xo");
	if (IS_ERR(xo_clk)) {
		BAM_DMUX_LOG("%s: did not get xo clock\n", __func__);
		xo_clk = NULL;
	}
	dfab_clk = clk_get(&pdev->dev, "bus_clk");
	if (IS_ERR(dfab_clk)) {
		BAM_DMUX_LOG("%s: did not get dfab clock\n", __func__);
		dfab_clk = NULL;
	} else {
		rc = clk_set_rate(dfab_clk, 64000000);
		if (rc)
			pr_err("%s: unable to set dfab clock rate\n", __func__);
	}

	bam_mux_rx_workqueue = alloc_workqueue("bam_dmux_rx",
					WQ_MEM_RECLAIM | WQ_CPU_INTENSIVE, 1);
	if (!bam_mux_rx_workqueue)
		return -ENOMEM;

	bam_mux_tx_workqueue = create_singlethread_workqueue("bam_dmux_tx");
	if (!bam_mux_tx_workqueue) {
		destroy_workqueue(bam_mux_rx_workqueue);
		return -ENOMEM;
	}

	for (rc = 0; rc < BAM_DMUX_NUM_CHANNELS; ++rc) {
		spin_lock_init(&bam_ch[rc].lock);
		scnprintf(bam_ch[rc].name, BAM_DMUX_CH_NAME_MAX_LEN,
					"bam_dmux_ch_%d", rc);
		
		bam_ch[rc].pdev = platform_device_alloc(bam_ch[rc].name, 2);
		if (!bam_ch[rc].pdev) {
			pr_err("%s: platform device alloc failed\n", __func__);
			destroy_workqueue(bam_mux_rx_workqueue);
			destroy_workqueue(bam_mux_tx_workqueue);
			return -ENOMEM;
		}
	}

	init_completion(&ul_wakeup_ack_completion);
	init_completion(&bam_connection_completion);
	init_completion(&dfab_unvote_completion);
	init_completion(&shutdown_completion);
	complete_all(&shutdown_completion);
	INIT_DELAYED_WORK(&ul_timeout_work, ul_timeout);
	wake_lock_init(&bam_wakelock, WAKE_LOCK_SUSPEND, "bam_dmux_wakelock");
	init_srcu_struct(&bam_dmux_srcu);

	rc = bam_ops->smsm_state_cb_register_ptr(SMSM_MODEM_STATE,
			SMSM_A2_POWER_CONTROL,
			bam_dmux_smsm_cb, NULL);

	if (rc) {
		destroy_workqueue(bam_mux_rx_workqueue);
		destroy_workqueue(bam_mux_tx_workqueue);
		pr_err("%s: smsm cb register failed, rc: %d\n", __func__, rc);
		return -ENOMEM;
	}

	rc = bam_ops->smsm_state_cb_register_ptr(SMSM_MODEM_STATE,
			SMSM_A2_POWER_CONTROL_ACK,
			bam_dmux_smsm_ack_cb, NULL);

	if (rc) {
		destroy_workqueue(bam_mux_rx_workqueue);
		destroy_workqueue(bam_mux_tx_workqueue);
		bam_ops->smsm_state_cb_deregister_ptr(SMSM_MODEM_STATE,
					SMSM_A2_POWER_CONTROL,
					bam_dmux_smsm_cb, NULL);
		pr_err("%s: smsm ack cb register failed, rc: %d\n", __func__,
				rc);
		for (rc = 0; rc < BAM_DMUX_NUM_CHANNELS; ++rc)
			platform_device_put(bam_ch[rc].pdev);
		return -ENOMEM;
	}

	if (bam_ops->smsm_get_state_ptr(SMSM_MODEM_STATE) &
			SMSM_A2_POWER_CONTROL)
		bam_dmux_smsm_cb(NULL, 0,
			bam_ops->smsm_get_state_ptr(SMSM_MODEM_STATE));

	return 0;
}

void bam_change_adaptive_timer(int mode)
{
	bam_adaptive_timer_enabled = mode;
	printk("\nchange bam_adaptive_timer_enabled=%d\n", bam_adaptive_timer_enabled);
}
EXPORT_SYMBOL(bam_change_adaptive_timer);

static struct of_device_id msm_match_table[] = {
	{.compatible = "qcom,bam_dmux"},
	{},
};

static struct platform_driver bam_dmux_driver = {
	.probe		= bam_dmux_probe,
	.driver		= {
		.name	= "BAM_RMNT",
		.owner	= THIS_MODULE,
		.of_match_table = msm_match_table,
	},
};

#ifdef CONFIG_HTC_DEBUG_RIL_PCN0006_HTC_DUMP_BAM_DMUX_LOG

static char bam_dmux_klog[PAGE_SIZE];

static void bam_dmux_dbg_inc(unsigned *idx)
{
	*idx = (*idx + 1) & (DBG_MAX_MSG-1);
}

static char *bam_dmux_get_timestamp(char *tbuf)
{
	unsigned long long t;
	unsigned long nanosec_rem;

	t = cpu_clock(smp_processor_id());
	nanosec_rem = do_div(t, 1000000000)/1000;
	scnprintf(tbuf, TIME_BUF_LEN, "[%5lu.%06lu] ", (unsigned long)t,
		nanosec_rem);
	return tbuf;
}

void bam_dmux_events_print(void)
{
	unsigned long	flags;
	unsigned	i;
	unsigned lines = 0;

	pr_info("### Show BAM DMUX Log Start ###\n");

	read_lock_irqsave(&dbg_bam_dmux.lck, flags);

	i = dbg_bam_dmux.idx;
	for (bam_dmux_dbg_inc(&i); i != dbg_bam_dmux.idx; bam_dmux_dbg_inc(&i)) {
		if (!strnlen(dbg_bam_dmux.buf[i], DBG_MSG_LEN))
			continue;
		pr_info("%s", dbg_bam_dmux.buf[i]);
		lines++;
		if ( lines > bam_dmux_htc_debug_dump_lines )
			break;
	}

	read_unlock_irqrestore(&dbg_bam_dmux.lck, flags);

	pr_info("### Show BAM DMUX Log End ###\n");
}

void msm_bam_dmux_dumplog(void)
{
	int ret = 0;

	if ( !bam_dmux_htc_debug_enable ) {
		pr_info("%s: bam_dmux_htc_debug_enable=[%d]\n", __func__, bam_dmux_htc_debug_enable);
		return;
	}

	if ( !bam_dmux_htc_debug_dump ) {
		pr_info("%s: bam_dmux_htc_debug_dump=[%d]\n", __func__, bam_dmux_htc_debug_dump);
		return;
	}

	if ( !bam_ipc_log_txt ) {
		pr_info("%s: bam_ipc_log_txt = NULL\n", __func__);
		bam_dmux_events_print();
		return;
	}

	pr_info("### Show BAM DMUX Log Start ###\n");

	do {

		memset(bam_dmux_klog, 0x0, PAGE_SIZE);
		ret = ipc_log_extract( bam_ipc_log_txt, bam_dmux_klog, PAGE_SIZE);
		if ( ret >= 0 ) {
			pr_info("%s\n", bam_dmux_klog);
		}

	} while ( ret > 0 );

	pr_info("### Show BAM DMUX Log End ###\n");

}
EXPORT_SYMBOL(msm_bam_dmux_dumplog);

void bam_dmux_dbg_log_event(const char * event, ...)
{
	unsigned long flags;
	char tbuf[TIME_BUF_LEN];
	char dbg_buff[DBG_MSG_LEN];
	va_list arg_list;
	int data_size;

	if ( !bam_dmux_htc_debug_enable ) {
		return;
	}

	va_start(arg_list, event);
	data_size = vsnprintf(dbg_buff,
			      DBG_MSG_LEN, event, arg_list);
	va_end(arg_list);

	write_lock_irqsave(&dbg_bam_dmux.lck, flags);

	scnprintf(dbg_bam_dmux.buf[dbg_bam_dmux.idx], DBG_MSG_LEN,
		"%s %s", bam_dmux_get_timestamp(tbuf), dbg_buff);

	bam_dmux_dbg_inc(&dbg_bam_dmux.idx);

	if ( bam_dmux_htc_debug_print )
		pr_info("%s", dbg_buff);
	write_unlock_irqrestore(&dbg_bam_dmux.lck, flags);

	return;

}
EXPORT_SYMBOL(bam_dmux_dbg_log_event);

static int bam_dmux_events_show(struct seq_file *s, void *unused)
{
	unsigned long	flags;
	unsigned	i;

	read_lock_irqsave(&dbg_bam_dmux.lck, flags);

	i = dbg_bam_dmux.idx;
	for (bam_dmux_dbg_inc(&i); i != dbg_bam_dmux.idx; bam_dmux_dbg_inc(&i)) {
		if (!strnlen(dbg_bam_dmux.buf[i], DBG_MSG_LEN))
			continue;
		seq_printf(s, "%s", dbg_bam_dmux.buf[i]);
	}

	read_unlock_irqrestore(&dbg_bam_dmux.lck, flags);

	return 0;
}

static int bam_dmux_events_open(struct inode *inode, struct file *f)
{
	return single_open(f, bam_dmux_events_show, inode->i_private);
}

const struct file_operations bam_dmux_dbg_fops = {
	.open = bam_dmux_events_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

static int __init bam_dmux_init(void)
{
#ifdef CONFIG_DEBUG_FS
	struct dentry *dent;

	dent = debugfs_create_dir("bam_dmux", 0);
	if (!IS_ERR(dent)) {
		debug_create("tbl", 0444, dent, debug_tbl);
		debug_create("ul_pkt_cnt", 0444, dent, debug_ul_pkt_cnt);
		debug_create("stats", 0444, dent, debug_stats);
#ifdef CONFIG_HTC_DEBUG_RIL_PCN0006_HTC_DUMP_BAM_DMUX_LOG
		debugfs_create_file("dumplog", S_IRUGO, dent, NULL, &bam_dmux_dbg_fops);
#endif
	}
#endif

	bam_ipc_log_txt = ipc_log_context_create(BAM_IPC_LOG_PAGES, "bam_dmux",
			0);
	if (!bam_ipc_log_txt) {
		pr_err("%s : unable to create IPC Logging Context", __func__);
	}

	rx_timer_interval = DEFAULT_POLLING_MIN_SLEEP;

	subsys_notif_register_notifier("modem", &restart_notifier);
	return platform_driver_register(&bam_dmux_driver);
}

late_initcall(bam_dmux_init); 
MODULE_DESCRIPTION("MSM BAM DMUX");
MODULE_LICENSE("GPL v2");
