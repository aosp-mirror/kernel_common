/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/stat.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/usb/msm_hsusb.h>
#include <mach/usb_bam.h>
#include <mach/sps.h>
#include <mach/ipa.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <mach/msm_smsm.h>
#include <linux/pm_runtime.h>

#define USB_THRESHOLD 512
#define USB_BAM_MAX_STR_LEN 50
#define USB_BAM_TIMEOUT (10*HZ)

enum usb_bam_sm {
	USB_BAM_SM_INIT = 0,
	USB_BAM_SM_PLUG_NOTIFIED,
	USB_BAM_SM_PLUG_ACKED,
	USB_BAM_SM_UNPLUG_NOTIFIED,
};

struct usb_bam_peer_handshake_info {
	enum usb_bam_sm state;
	bool client_ready;
	bool ack_received;
	int pending_work;
	struct usb_bam_event_info reset_event;
};

struct usb_bam_sps_type {
	struct sps_bam_props usb_props;
	struct sps_pipe **sps_pipes;
	struct sps_connect *sps_connections;
};

/**
* struct usb_bam_ctx_type - represents the usb bam driver entity
* @usb_bam_sps: holds the sps pipes the usb bam driver holds
*	against the sps driver.
* @usb_bam_pdev: the platfrom device that represents the usb bam.
* @usb_bam_wq: Worqueue used for managing states of reset against
*	a peer bam.
* @qscratch_ram1_reg: The memory region mapped to the qscratch
*	registers.
* @max_connections: The maximum number of pipes that are configured
*	in the platform data.
* @mem_clk: Clock that controls the usb bam driver memory in
*	case the usb bam uses its private memory for the pipes.
* @mem_iface_clk: Clock that controls the usb bam private memory in
*	case the usb bam uses its private memory for the pipes.
* @qdss_core_name: Stores the name of the core ("ssusb", "hsusb" or "hsic")
*	that it used as a peer of the qdss in bam2bam mode.
* @h_bam: This array stores for each BAM ("ssusb", "hsusb" or "hsic") the
*	handle/device of the sps driver.
* @pipes_enabled_per_bam: This array stores for each BAM
*	("ssusb", "hsusb" or "hsic") the number of pipes currently enabled.
* @inactivity_timer_ms: The timeout configuration per each bam for inactivity
*	timer feature.
* @is_bam_inactivity: Is there no activity on all pipes belongs to a
*	specific bam. (no activity = no data is pulled or pushed
*	from/into ones of the pipes).
*/
struct usb_bam_ctx_type {
	struct usb_bam_sps_type usb_bam_sps;
	struct platform_device *usb_bam_pdev;
	struct workqueue_struct *usb_bam_wq;
	void __iomem *qscratch_ram1_reg;
	u8 max_connections;
	struct clk *mem_clk;
	struct clk *mem_iface_clk;
	char qdss_core_name[USB_BAM_MAX_STR_LEN];
	u32 h_bam[MAX_BAMS];
	u8 pipes_enabled_per_bam[MAX_BAMS];
	u32 inactivity_timer_ms[MAX_BAMS];
	bool is_bam_inactivity[MAX_BAMS];
	struct completion reset_done;
};

static char *bam_enable_strings[MAX_BAMS] = {
	[SSUSB_BAM] = "ssusb",
	[HSUSB_BAM] = "hsusb",
	[HSIC_BAM]  = "hsic",
};

static enum usb_bam ipa_rm_bams[] = {HSUSB_BAM, HSIC_BAM};

static enum ipa_client_type ipa_rm_resource_prod[MAX_BAMS] = {
	[HSUSB_BAM] = IPA_RM_RESOURCE_USB_PROD,
	[HSIC_BAM]  = IPA_RM_RESOURCE_HSIC_PROD,
};

static enum ipa_client_type ipa_rm_resource_cons[MAX_BAMS] = {
	[HSUSB_BAM] = IPA_RM_RESOURCE_USB_CONS,
	[HSIC_BAM]  = IPA_RM_RESOURCE_HSIC_CONS,
};

static int usb_cons_request_resource(void);
static int usb_cons_release_resource(void);
static int hsic_cons_request_resource(void);
static int hsic_cons_release_resource(void);

static int (*request_resource_cb[MAX_BAMS])(void) = {
	[HSUSB_BAM] = usb_cons_request_resource,
	[HSIC_BAM]  = hsic_cons_request_resource,
};

static int (*release_resource_cb[MAX_BAMS])(void)  = {
	[HSUSB_BAM] = usb_cons_release_resource,
	[HSIC_BAM]  = hsic_cons_release_resource,
};

struct usb_bam_ipa_handshake_info {
	enum ipa_rm_event cur_prod_state[MAX_BAMS];
	enum ipa_rm_event cur_cons_state[MAX_BAMS];

	bool lpm_wait_handshake[MAX_BAMS];
	int connect_complete;
	bool lpm_wait_pipes;
	int bus_suspend;
	bool disconnected;
	bool in_lpm[MAX_BAMS];
	bool pending_lpm;

	int (*wake_cb)(void *);
	void *wake_param;
	void (*start)(void *, enum usb_bam_pipe_dir);
	void (*stop)(void *, enum usb_bam_pipe_dir);
	void *start_stop_param;

	u32 src_idx;
	u32 dst_idx;
	bool cons_stopped;
	bool prod_stopped;

	struct completion prod_avail[MAX_BAMS];
	struct completion prod_released[MAX_BAMS];

	struct mutex suspend_resume_mutex;
	struct work_struct resume_work;
	struct work_struct suspend_work;
	struct work_struct finish_suspend_work;
};

struct usb_bam_hsic_host_info {
	struct device *dev;
	bool in_lpm;
};

static spinlock_t usb_bam_ipa_handshake_info_lock;
static struct usb_bam_ipa_handshake_info info;
static spinlock_t usb_bam_peer_handshake_info_lock;
static struct usb_bam_peer_handshake_info peer_handshake_info;
static spinlock_t usb_bam_lock; /* Protect ctx and usb_bam_connections */
static struct usb_bam_pipe_connect *usb_bam_connections;
static struct usb_bam_ctx_type ctx;
static struct usb_bam_hsic_host_info hsic_host_info;

static int __usb_bam_register_wake_cb(u8 idx, int (*callback)(void *user),
	void *param, bool trigger_cb_per_pipe);
static void wait_for_prod_release(enum usb_bam cur_bam);

void msm_bam_set_hsic_host_dev(struct device *dev)
{
	if (dev) {
		/* Hold the device until allowing lpm */
		info.in_lpm[HSIC_BAM] = false;
		pr_debug("%s: Getting hsic device %x\n", __func__,
			(int)dev);
		pm_runtime_get(dev);
	} else if (hsic_host_info.dev) {
		pr_debug("%s: Putting hsic device %x\n", __func__,
			(int)hsic_host_info.dev);
		/* Just free previous device*/
		info.in_lpm[HSIC_BAM] = true;
		pm_runtime_put(hsic_host_info.dev);
	}

	hsic_host_info.dev = dev;
	hsic_host_info.in_lpm = false;
}

static int get_bam_type_from_core_name(const char *name)
{
	if (strnstr(name, bam_enable_strings[SSUSB_BAM],
			USB_BAM_MAX_STR_LEN) ||
		strnstr(name, "dwc3", USB_BAM_MAX_STR_LEN))
		return SSUSB_BAM;
	else if (strnstr(name, bam_enable_strings[HSIC_BAM],
			USB_BAM_MAX_STR_LEN))
		return HSIC_BAM;
	else if (strnstr(name, bam_enable_strings[HSUSB_BAM],
			USB_BAM_MAX_STR_LEN) ||
		strnstr(name, "ci", USB_BAM_MAX_STR_LEN))
		return HSUSB_BAM;

	pr_err("%s: invalid BAM name(%s)\n", __func__, name);
	return -EINVAL;
}

static bool bam_use_private_mem(enum usb_bam bam)
{
	int i;

	for (i = 0; i < ctx.max_connections; i++)
		if (usb_bam_connections[i].bam_type == bam &&
			usb_bam_connections[i].mem_type == USB_PRIVATE_MEM)
				return true;

	return false;
}

static void usb_bam_set_inactivity_timer(enum usb_bam bam)
{
	struct sps_timer_ctrl timer_ctrl;
	struct usb_bam_pipe_connect *pipe_connect;
	struct sps_pipe *pipe = NULL;
	int i;

	pr_debug("%s: enter\n", __func__);

	/*
	 * Since we configure global incativity timer for all pipes
	 * and not per each pipe, it is enough to use some pipe
	 * handle associated with this bam, so just find the first one.
	 * This pipe handle is required due to SPS driver API we use below.
	 */
	for (i = 0; i < ctx.max_connections; i++) {
		pipe_connect = &usb_bam_connections[i];
		if (pipe_connect->bam_type == bam &&
		    pipe_connect->enabled) {
			pipe = ctx.usb_bam_sps.sps_pipes[i];
			break;
		}
	}

	if (!pipe) {
		pr_warning("%s: Bam %s has no connected pipes\n", __func__,
			bam_enable_strings[bam]);
		return;
	}

	timer_ctrl.op = SPS_TIMER_OP_CONFIG;
	timer_ctrl.mode = SPS_TIMER_MODE_ONESHOT;
	timer_ctrl.timeout_msec = ctx.inactivity_timer_ms[bam];
	sps_timer_ctrl(pipe, &timer_ctrl, NULL);

	timer_ctrl.op = SPS_TIMER_OP_RESET;
	sps_timer_ctrl(pipe, &timer_ctrl, NULL);
}

static int connect_pipe(u8 idx, u32 *usb_pipe_idx)
{
	int ret, ram1_value;
	enum usb_bam bam;
	struct usb_bam_sps_type usb_bam_sps = ctx.usb_bam_sps;
	struct sps_pipe **pipe = &(usb_bam_sps.sps_pipes[idx]);
	struct sps_connect *sps_connection = &usb_bam_sps.sps_connections[idx];
	struct msm_usb_bam_platform_data *pdata =
		ctx.usb_bam_pdev->dev.platform_data;
	struct usb_bam_pipe_connect *pipe_connect = &usb_bam_connections[idx];
	enum usb_bam_pipe_dir dir = pipe_connect->dir;
	struct sps_mem_buffer *data_buf = &(pipe_connect->data_mem_buf);
	struct sps_mem_buffer *desc_buf = &(pipe_connect->desc_mem_buf);

	*pipe = sps_alloc_endpoint();
	if (*pipe == NULL) {
		pr_err("%s: sps_alloc_endpoint failed\n", __func__);
		return -ENOMEM;
	}

	ret = sps_get_config(*pipe, sps_connection);
	if (ret) {
		pr_err("%s: tx get config failed %d\n", __func__, ret);
		goto free_sps_endpoint;
	}

	ret = sps_phy2h(pipe_connect->src_phy_addr, &(sps_connection->source));
	if (ret) {
		pr_err("%s: sps_phy2h failed (src BAM) %d\n", __func__, ret);
		goto free_sps_endpoint;
	}

	sps_connection->src_pipe_index = pipe_connect->src_pipe_index;
	ret = sps_phy2h(pipe_connect->dst_phy_addr,
		&(sps_connection->destination));
	if (ret) {
		pr_err("%s: sps_phy2h failed (dst BAM) %d\n", __func__, ret);
		goto free_sps_endpoint;
	}
	sps_connection->dest_pipe_index = pipe_connect->dst_pipe_index;

	if (dir == USB_TO_PEER_PERIPHERAL) {
		sps_connection->mode = SPS_MODE_SRC;
		*usb_pipe_idx = pipe_connect->src_pipe_index;
	} else {
		sps_connection->mode = SPS_MODE_DEST;
		*usb_pipe_idx = pipe_connect->dst_pipe_index;
	}

	switch (pipe_connect->mem_type) {
	case SPS_PIPE_MEM:
		pr_debug("%s: USB BAM using SPS pipe memory\n", __func__);
		ret = sps_setup_bam2bam_fifo(
			data_buf,
			pipe_connect->data_fifo_base_offset,
			pipe_connect->data_fifo_size, 1);
		if (ret) {
			pr_err("%s: data fifo setup failure %d\n", __func__,
				ret);
			goto free_sps_endpoint;
		}

		ret = sps_setup_bam2bam_fifo(
			desc_buf,
			pipe_connect->desc_fifo_base_offset,
			pipe_connect->desc_fifo_size, 1);
		if (ret) {
			pr_err("%s: desc. fifo setup failure %d\n", __func__,
				ret);
			goto free_sps_endpoint;
		}
		break;
	case USB_PRIVATE_MEM:
		pr_debug("%s: USB BAM using private memory\n", __func__);

		if (IS_ERR(ctx.mem_clk) || IS_ERR(ctx.mem_iface_clk)) {
			pr_err("%s: Failed to enable USB mem_clk\n", __func__);
			ret = IS_ERR(ctx.mem_clk);
			goto free_sps_endpoint;
		}

		clk_prepare_enable(ctx.mem_clk);
		clk_prepare_enable(ctx.mem_iface_clk);

		/*
		 * Enable USB PRIVATE RAM to be used for BAM FIFOs
		 * HSUSB: Only RAM13 is used for BAM FIFOs
		 * SSUSB: RAM11, 12, 13 are used for BAM FIFOs
		 */
		bam = pipe_connect->bam_type;
		if (bam < 0)
			goto free_sps_endpoint;

		if (bam == HSUSB_BAM)
			ram1_value = 0x4;
		else
			ram1_value = 0x7;

		pr_debug("Writing 0x%x to QSCRATCH_RAM1\n", ram1_value);
		writel_relaxed(ram1_value, ctx.qscratch_ram1_reg);
		/* fall through */
	case OCI_MEM:
		pr_debug("%s: USB BAM using oci memory\n", __func__);
		data_buf->phys_base =
			pipe_connect->data_fifo_base_offset +
				pdata->usb_bam_fifo_baseaddr;
		data_buf->size = pipe_connect->data_fifo_size;
		data_buf->base =
			ioremap(data_buf->phys_base, data_buf->size);
		memset(data_buf->base, 0, data_buf->size);

		desc_buf->phys_base =
			pipe_connect->desc_fifo_base_offset +
				pdata->usb_bam_fifo_baseaddr;
		desc_buf->size = pipe_connect->desc_fifo_size;
		desc_buf->base =
			ioremap(desc_buf->phys_base, desc_buf->size);
		memset(desc_buf->base, 0, desc_buf->size);
		break;
	case SYSTEM_MEM:
		pr_debug("%s: USB BAM using system memory\n", __func__);
		/* BAM would use system memory, allocate FIFOs */
		data_buf->size = pipe_connect->data_fifo_size;
		data_buf->base =
			dma_alloc_coherent(&ctx.usb_bam_pdev->dev,
			pipe_connect->data_fifo_size,
			&(data_buf->phys_base),
			0);
		memset(data_buf->base, 0, pipe_connect->data_fifo_size);

		desc_buf->size = pipe_connect->desc_fifo_size;
		desc_buf->base =
			dma_alloc_coherent(&ctx.usb_bam_pdev->dev,
			pipe_connect->desc_fifo_size,
			&(desc_buf->phys_base),
			0);
		memset(desc_buf->base, 0, pipe_connect->desc_fifo_size);
		break;
	default:
		pr_err("%s: invalid mem type\n", __func__);
		goto free_sps_endpoint;
	}

	sps_connection->data = *data_buf;
	sps_connection->desc = *desc_buf;
	sps_connection->event_thresh = 16;
	sps_connection->options = SPS_O_AUTO_ENABLE;

	ret = sps_connect(*pipe, sps_connection);
	if (ret < 0) {
		pr_err("%s: sps_connect failed %d\n", __func__, ret);
		goto error;
	}

	return 0;

error:
	sps_disconnect(*pipe);
free_sps_endpoint:
	sps_free_endpoint(*pipe);
	return ret;
}

static int connect_pipe_ipa(u8 idx,
			struct usb_bam_connect_ipa_params *ipa_params)
{
	int ret;
	struct usb_bam_sps_type usb_bam_sps = ctx.usb_bam_sps;
	enum usb_bam_pipe_dir dir = ipa_params->dir;
	struct sps_pipe **pipe = &(usb_bam_sps.sps_pipes[idx]);
	struct sps_connect *sps_connection = &usb_bam_sps.sps_connections[idx];
	struct usb_bam_pipe_connect *pipe_connect = &usb_bam_connections[idx];

	struct ipa_connect_params ipa_in_params;
	struct ipa_sps_params sps_out_params;
	u32 usb_handle, usb_phy_addr;
	u32 clnt_hdl = 0;

	memset(&ipa_in_params, 0, sizeof(ipa_in_params));
	memset(&sps_out_params, 0, sizeof(sps_out_params));

	if (dir == USB_TO_PEER_PERIPHERAL) {
		usb_phy_addr = pipe_connect->src_phy_addr;
		ipa_in_params.client_ep_idx = pipe_connect->src_pipe_index;
	} else {
		usb_phy_addr = pipe_connect->dst_phy_addr;
		ipa_in_params.client_ep_idx = pipe_connect->dst_pipe_index;
	}
	/* Get HSUSB / HSIC bam handle */
	ret = sps_phy2h(usb_phy_addr, &usb_handle);
	if (ret) {
		pr_err("%s: sps_phy2h failed (HSUSB/HSIC BAM) %d\n",
			__func__, ret);
		return ret;
	}

	pipe_connect->activity_notify = ipa_params->activity_notify;
	pipe_connect->inactivity_notify = ipa_params->inactivity_notify;
	pipe_connect->priv = ipa_params->priv;

	/* IPA input parameters */
	ipa_in_params.client_bam_hdl = usb_handle;
	ipa_in_params.desc_fifo_sz = pipe_connect->desc_fifo_size;
	ipa_in_params.data_fifo_sz = pipe_connect->data_fifo_size;
	ipa_in_params.notify = ipa_params->notify;
	ipa_in_params.priv = ipa_params->priv;
	ipa_in_params.client = ipa_params->client;

	/* If BAM is using dedicated SPS pipe memory, get it */

	if (pipe_connect->mem_type == SPS_PIPE_MEM) {
		pr_debug("%s: USB BAM using SPS pipe memory\n", __func__);
		ret = sps_setup_bam2bam_fifo(
			&pipe_connect->data_mem_buf,
			pipe_connect->data_fifo_base_offset,
			pipe_connect->data_fifo_size, 1);
		if (ret) {
			pr_err("%s: data fifo setup failure %d\n",
				__func__, ret);
			return ret;
		}

		ret = sps_setup_bam2bam_fifo(
			&pipe_connect->desc_mem_buf,
			pipe_connect->desc_fifo_base_offset,
			pipe_connect->desc_fifo_size, 1);
		if (ret) {
			pr_err("%s: desc. fifo setup failure %d\n",
				__func__, ret);
			return ret;
		}

		ipa_in_params.desc = pipe_connect->desc_mem_buf;
		ipa_in_params.data = pipe_connect->data_mem_buf;
	}

	memcpy(&ipa_in_params.ipa_ep_cfg, &ipa_params->ipa_ep_cfg,
		   sizeof(struct ipa_ep_cfg));

	ret = ipa_connect(&ipa_in_params, &sps_out_params, &clnt_hdl);
	if (ret) {
		pr_err("%s: ipa_connect failed\n", __func__);
		return ret;
	}
	pipe_connect->ipa_clnt_hdl = clnt_hdl;

	*pipe = sps_alloc_endpoint();
	if (*pipe == NULL) {
		pr_err("%s: sps_alloc_endpoint failed\n", __func__);
		ret = -ENOMEM;
		goto disconnect_ipa;
	}

	ret = sps_get_config(*pipe, sps_connection);
	if (ret) {
		pr_err("%s: tx get config failed %d\n", __func__, ret);
		goto free_sps_endpoints;
	}

	if (dir == USB_TO_PEER_PERIPHERAL) {
		/* USB src IPA dest */
		sps_connection->mode = SPS_MODE_SRC;
		ipa_params->cons_clnt_hdl = clnt_hdl;
		sps_connection->source = usb_handle;
		sps_connection->destination = sps_out_params.ipa_bam_hdl;
		sps_connection->src_pipe_index = pipe_connect->src_pipe_index;
		sps_connection->dest_pipe_index = sps_out_params.ipa_ep_idx;
		*(ipa_params->src_pipe) = sps_connection->src_pipe_index;
		pipe_connect->dst_pipe_index = sps_out_params.ipa_ep_idx;
		pr_debug("%s: BAM pipe usb[%x]->ipa[%x] connection\n",
			__func__,
			pipe_connect->src_pipe_index,
			pipe_connect->dst_pipe_index);
		sps_connection->options = SPS_O_NO_DISABLE;
	} else {
		/* IPA src, USB dest */
		sps_connection->mode = SPS_MODE_DEST;
		ipa_params->prod_clnt_hdl = clnt_hdl;
		sps_connection->source = sps_out_params.ipa_bam_hdl;
		sps_connection->destination = usb_handle;
		sps_connection->src_pipe_index = sps_out_params.ipa_ep_idx;
		sps_connection->dest_pipe_index = pipe_connect->dst_pipe_index;
		*(ipa_params->dst_pipe) = sps_connection->dest_pipe_index;
		pipe_connect->src_pipe_index = sps_out_params.ipa_ep_idx;
		pr_debug("%s: BAM pipe ipa[%x]->usb[%x] connection\n",
			__func__,
			pipe_connect->src_pipe_index,
			pipe_connect->dst_pipe_index);
		sps_connection->options = 0;
	}

	sps_connection->data = sps_out_params.data;
	sps_connection->desc = sps_out_params.desc;
	sps_connection->event_thresh = 16;
	sps_connection->options |= SPS_O_AUTO_ENABLE;

	ret = sps_connect(*pipe, sps_connection);
	if (ret < 0) {
		pr_err("%s: sps_connect failed %d\n", __func__, ret);
		goto error;
	}

	return 0;

error:
	sps_disconnect(*pipe);
free_sps_endpoints:
	sps_free_endpoint(*pipe);
disconnect_ipa:
	ipa_disconnect(clnt_hdl);
	return ret;
}

static int disconnect_pipe(u8 idx)
{
	struct usb_bam_pipe_connect *pipe_connect =
		&usb_bam_connections[idx];
	struct sps_pipe *pipe = ctx.usb_bam_sps.sps_pipes[idx];
	struct sps_connect *sps_connection =
		&ctx.usb_bam_sps.sps_connections[idx];

	sps_disconnect(pipe);
	sps_free_endpoint(pipe);

	switch (pipe_connect->mem_type) {
	case SYSTEM_MEM:
		pr_debug("%s: Freeing system memory used by PIPE\n", __func__);
		if (sps_connection->data.phys_base)
			dma_free_coherent(&ctx.usb_bam_pdev->dev,
					sps_connection->data.size,
					sps_connection->data.base,
					sps_connection->data.phys_base);
		if (sps_connection->desc.phys_base)
			dma_free_coherent(&ctx.usb_bam_pdev->dev,
					sps_connection->desc.size,
					sps_connection->desc.base,
					sps_connection->desc.phys_base);
		break;
	case USB_PRIVATE_MEM:
		pr_debug("Freeing private memory used by BAM PIPE\n");
		writel_relaxed(0x0, ctx.qscratch_ram1_reg);
		clk_disable_unprepare(ctx.mem_clk);
		clk_disable_unprepare(ctx.mem_iface_clk);
	case OCI_MEM:
		pr_debug("Freeing oci memory used by BAM PIPE\n");
		iounmap(sps_connection->data.base);
		iounmap(sps_connection->desc.base);
		break;
	case SPS_PIPE_MEM:
		pr_debug("%s: nothing to be be\n", __func__);
		break;
	}

	sps_connection->options &= ~SPS_O_AUTO_ENABLE;
	return 0;
}

static void usb_bam_resume_core(enum usb_bam cur_bam)
{
	struct usb_phy *trans = usb_get_transceiver();

	if (cur_bam != HSUSB_BAM)
		return;
	BUG_ON(trans == NULL);
	pr_debug("%s: resume core", __func__);
	pm_runtime_resume(trans->dev);
}

static void usb_bam_start_lpm(bool disconnect)
{
	struct usb_phy *trans = usb_get_transceiver();

	BUG_ON(trans == NULL);

	spin_lock(&usb_bam_ipa_handshake_info_lock);

	info.lpm_wait_handshake[HSUSB_BAM] = false;
	info.lpm_wait_pipes = 0;

	if (disconnect)
		pm_runtime_put_noidle(trans->dev);

	if (info.pending_lpm) {
		info.pending_lpm = 0;
		spin_unlock(&usb_bam_ipa_handshake_info_lock);
		pr_debug("%s: Going to LPM\n", __func__);
		pm_runtime_suspend(trans->dev);
	} else
		spin_unlock(&usb_bam_ipa_handshake_info_lock);

}

int usb_bam_connect(u8 idx, u32 *bam_pipe_idx)
{
	int ret;
	struct usb_bam_pipe_connect *pipe_connect = &usb_bam_connections[idx];
	struct msm_usb_bam_platform_data *pdata;

	if (!ctx.usb_bam_pdev) {
		pr_err("%s: usb_bam device not found\n", __func__);
		return -ENODEV;
	}

	pdata = ctx.usb_bam_pdev->dev.platform_data;

	if (pipe_connect->enabled) {
		pr_debug("%s: connection %d was already established\n",
			__func__, idx);
		return 0;
	}

	if (!bam_pipe_idx) {
		pr_err("%s: invalid bam_pipe_idx\n", __func__);
		return -EINVAL;
	}
	if (idx < 0 || idx > ctx.max_connections) {
		pr_err("idx is wrong %d", idx);
		return -EINVAL;
	}

	spin_lock(&usb_bam_lock);
	/* Check if BAM requires RESET before connect and reset of first pipe */
	if ((pdata->reset_on_connect[pipe_connect->bam_type] == true) &&
	    (ctx.pipes_enabled_per_bam[pipe_connect->bam_type] == 0))
		sps_device_reset(ctx.h_bam[pipe_connect->bam_type]);
	spin_unlock(&usb_bam_lock);

	ret = connect_pipe(idx, bam_pipe_idx);
	if (ret) {
		pr_err("%s: pipe connection[%d] failure\n", __func__, idx);
		return ret;
	}

	pipe_connect->enabled = 1;
	spin_lock(&usb_bam_lock);
	ctx.pipes_enabled_per_bam[pipe_connect->bam_type] += 1;
	spin_unlock(&usb_bam_lock);

	return 0;
}

static int ipa_suspend_pipes(void)
{
	struct usb_bam_pipe_connect *dst_pipe_connect, *src_pipe_connect;
	int ret1, ret2;

	dst_pipe_connect = &usb_bam_connections[info.dst_idx];
	src_pipe_connect = &usb_bam_connections[info.src_idx];

	if (dst_pipe_connect->ipa_clnt_hdl == -1 ||
		src_pipe_connect->ipa_clnt_hdl == -1) {
		pr_err("%s: One of handles is -1, not connected?", __func__);
	}

	ret1 = ipa_suspend(dst_pipe_connect->ipa_clnt_hdl);
	if (ret1)
		pr_err("%s: ipa_suspend on dst failed with %d", __func__, ret1);
	ret2 = ipa_suspend(src_pipe_connect->ipa_clnt_hdl);
	if (ret2)
		pr_err("%s: ipa_suspend on src failed with %d", __func__, ret2);

	return ret1 | ret2;
}

static int ipa_resume_pipes(void)
{
	struct usb_bam_pipe_connect *dst_pipe_connect, *src_pipe_connect;
	int ret1, ret2;

	src_pipe_connect = &usb_bam_connections[info.src_idx];
	dst_pipe_connect = &usb_bam_connections[info.dst_idx];

	if (dst_pipe_connect->ipa_clnt_hdl == -1 ||
		src_pipe_connect->ipa_clnt_hdl == -1) {
		pr_err("%s: One of handles is -1, not connected?", __func__);
	}

	ret1 = ipa_resume(dst_pipe_connect->ipa_clnt_hdl);
	if (ret1)
		pr_err("%s: ipa_resume on dst failed with %d", __func__, ret1);
	ret2 = ipa_resume(src_pipe_connect->ipa_clnt_hdl);
	if (ret2)
		pr_err("%s: ipa_resume on src failed with %d", __func__, ret2);

	return ret1 | ret2;
}

static void usb_bam_finish_suspend(void)
{
	int ret;
	u32 cons_empty;
	struct sps_pipe *cons_pipe = ctx.usb_bam_sps.sps_pipes[info.dst_idx];
	struct usb_bam_pipe_connect *pipe_connect = &
			usb_bam_connections[info.dst_idx];
	enum usb_bam cur_bam = pipe_connect->bam_type;

	if (cur_bam != HSUSB_BAM) {
		pr_err("%s: Wrong type of BAM=%s\n", __func__,
			bam_enable_strings[cur_bam]);
		return;
	}

	mutex_lock(&info.suspend_resume_mutex);

	spin_lock(&usb_bam_ipa_handshake_info_lock);
	/* If cable was disconnected, let disconnection seq do everything */
	if (info.disconnected || info.cons_stopped) {
		spin_unlock(&usb_bam_ipa_handshake_info_lock);
		mutex_unlock(&info.suspend_resume_mutex);
		pr_debug("%s: Cable disconnected\n", __func__);
		return;
	}

	/* If resume was called don't finish this work */
	if (!info.bus_suspend) {
		spin_unlock(&usb_bam_ipa_handshake_info_lock);
		pr_err("%s: Bus resume in progress\n", __func__);
		goto no_lpm;
	}
	spin_unlock(&usb_bam_ipa_handshake_info_lock);

	ret = sps_is_pipe_empty(cons_pipe, &cons_empty);
	if (ret) {
		pr_err("%s: sps_is_pipe_empty failed with %d\n", __func__, ret);
		goto no_lpm;
	}

	/* Stop CONS transfers and go to lpm if no more data in the pipe */
	if (cons_empty) {
		if (info.stop && !info.cons_stopped)
			info.stop(info.start_stop_param,
					PEER_PERIPHERAL_TO_USB);
		pr_err("%s: Starting LPM on Bus Suspend\n", __func__);
		info.cons_stopped = 1;
		if (info.cur_cons_state[cur_bam] == IPA_RM_RESOURCE_RELEASED) {
			ipa_rm_notify_completion(IPA_RM_RESOURCE_RELEASED,
				ipa_rm_resource_cons[cur_bam]);
		}
		ipa_suspend_pipes();
		usb_bam_start_lpm(0);
		mutex_unlock(&info.suspend_resume_mutex);
		return;
	}

no_lpm:

	/* Finish the handshake. Resume Sequence will start automatically
	   by the data in the pipes */
	if (info.cur_cons_state[cur_bam] == IPA_RM_RESOURCE_RELEASED)
		ipa_rm_notify_completion(IPA_RM_RESOURCE_RELEASED,
				ipa_rm_resource_cons[cur_bam]);
	mutex_unlock(&info.suspend_resume_mutex);
}

void usb_bam_finish_suspend_(struct work_struct *w)
{
	usb_bam_finish_suspend();
}

static void usb_prod_notify_cb(void *user_data, enum ipa_rm_event event,
	unsigned long data)
{
	enum usb_bam *cur_bam = (void *)user_data;

	switch (event) {
	case IPA_RM_RESOURCE_GRANTED:
		pr_debug("%s: %s_PROD resource granted\n",
			__func__, bam_enable_strings[*cur_bam]);
		info.cur_prod_state[*cur_bam] = IPA_RM_RESOURCE_GRANTED;
		complete_all(&info.prod_avail[*cur_bam]);
		break;
	case IPA_RM_RESOURCE_RELEASED:
		pr_debug("%s: %s_PROD resource released\n",
			__func__, bam_enable_strings[*cur_bam]);
		info.cur_prod_state[*cur_bam] = IPA_RM_RESOURCE_RELEASED;
		complete_all(&info.prod_released[*cur_bam]);
		break;
	default:
		break;
	}
	return;
}

/**
 * usb_bam_resume_hsic_host: vote for hsic host core resume.
 * In addition also resume all hsic pipes that are connected to
 * the ipa peer bam.
 *
 * NOTE: This function should be called in a context that hold
 *	 usb_bam_lock.
 */
static void usb_bam_resume_hsic_host(void)
{
	int i;
	struct usb_bam_pipe_connect *pipe_iter;

	/* Exit from "full suspend" in case of hsic host */
	if (hsic_host_info.dev && info.in_lpm[HSIC_BAM]) {
		pr_debug("%s: Getting hsic device %x\n", __func__,
			(int)hsic_host_info.dev);
		pm_runtime_get(hsic_host_info.dev);
		info.in_lpm[HSIC_BAM] = false;

		for (i = 0; i < ctx.max_connections; i++) {
			pipe_iter = &usb_bam_connections[i];
			if (pipe_iter->bam_type == HSIC_BAM &&
			    pipe_iter->enabled &&
			    pipe_iter->suspended) {
				spin_unlock(&usb_bam_lock);
				ipa_resume(pipe_iter->ipa_clnt_hdl);
				pipe_iter->suspended = false;
				spin_lock(&usb_bam_lock);
			}
		}
	}
}

static int cons_request_resource(enum usb_bam cur_bam)
{
	int ret = -EINPROGRESS;

	pr_debug("%s: Request %s_CONS resource\n",
			__func__, bam_enable_strings[cur_bam]);

	spin_lock(&usb_bam_ipa_handshake_info_lock);
	info.cur_cons_state[cur_bam] = IPA_RM_RESOURCE_GRANTED;

	spin_lock(&usb_bam_lock);

	switch (cur_bam) {
	case HSUSB_BAM:
		if (ctx.pipes_enabled_per_bam[HSUSB_BAM] &&
		    info.connect_complete &&
			!info.cons_stopped && !info.prod_stopped) {
			pr_debug("%s: ACK on cons_request", __func__);
			ret = 0;
		} else if (ctx.pipes_enabled_per_bam[HSUSB_BAM] &&
				   info.connect_complete && info.bus_suspend) {
			info.bus_suspend = 0;
			if (info.wake_cb)
				info.wake_cb(info.wake_param);
		}

		break;
	case HSIC_BAM:
		/*
		 * Vote for hsic resume, however the core
		 * resume may not be completed yet or on the other hand
		 * hsic core might already be resumed, due to a vote
		 * by other driver, in this case we will just renew our
		 * vote here.
		 */
		usb_bam_resume_hsic_host();

		/*
		 * Return sucess if there are pipes connected
		 * and hsic core is actually not in lpm.
		 * If in lpm, grant will occur on resume
		 * finish (see msm_bam_hsic_notify_on_resume)
		 */
		if (ctx.pipes_enabled_per_bam[cur_bam] &&
		    !hsic_host_info.in_lpm) {
			ret = 0;
		}

		break;
	case SSUSB_BAM:
	default:
		break;
	}

	spin_unlock(&usb_bam_ipa_handshake_info_lock);
	spin_unlock(&usb_bam_lock);

	if (ret == -EINPROGRESS)
		pr_debug("%s: EINPROGRESS on cons_request", __func__);

	return ret;
}

static int usb_cons_request_resource(void)
{
	return cons_request_resource(HSUSB_BAM);
}

static int hsic_cons_request_resource(void)
{
	return cons_request_resource(HSIC_BAM);
}

static int cons_release_resource(enum usb_bam cur_bam)
{
	pr_debug("%s: Release %s_CONS resource\n",
			__func__, bam_enable_strings[cur_bam]);

	info.cur_cons_state[cur_bam] = IPA_RM_RESOURCE_RELEASED;

	spin_lock(&usb_bam_lock);
	if (!ctx.pipes_enabled_per_bam[cur_bam]) {
		spin_unlock(&usb_bam_lock);
		pr_debug("%s: ACK on cons_release", __func__);
		return 0;
	}
	spin_unlock(&usb_bam_lock);

	if (cur_bam == HSUSB_BAM) {
		spin_lock(&usb_bam_ipa_handshake_info_lock);
		if (info.bus_suspend)
			queue_work(ctx.usb_bam_wq, &info.finish_suspend_work);
		spin_unlock(&usb_bam_ipa_handshake_info_lock);

		pr_debug("%s: EINPROGRESS cons_release", __func__);
		return -EINPROGRESS;
	} else if (cur_bam == HSIC_BAM) {

		/*
		 * Allow to go to lpm for now. Actual state will be checked
		 * in msm_bam_hsic_lpm_ok() just before going to lpm.
		 */
		if (hsic_host_info.dev && !info.in_lpm[HSIC_BAM]) {
			pr_debug("%s: Putting hsic device %x\n", __func__,
			(int)hsic_host_info.dev);
			pm_runtime_put(hsic_host_info.dev);
			info.in_lpm[HSIC_BAM] = true;
		}
	}

	return 0;
}

static int hsic_cons_release_resource(void)
{
	return cons_release_resource(HSIC_BAM);
}

static int usb_cons_release_resource(void)
{
	return cons_release_resource(HSUSB_BAM);
}

static void usb_bam_ipa_create_resources(void)
{
	struct ipa_rm_create_params usb_prod_create_params;
	struct ipa_rm_create_params usb_cons_create_params;
	enum usb_bam cur_bam;
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(ipa_rm_bams); i++) {
		/* Create USB/HSIC_PROD entity */
		cur_bam = ipa_rm_bams[i];

		memset(&usb_prod_create_params, 0,
					sizeof(usb_prod_create_params));
		usb_prod_create_params.name = ipa_rm_resource_prod[cur_bam];
		usb_prod_create_params.reg_params.notify_cb =
							usb_prod_notify_cb;
		usb_prod_create_params.reg_params.user_data = &ipa_rm_bams[i];
		ret = ipa_rm_create_resource(&usb_prod_create_params);
		if (ret) {
			pr_err("%s: Failed to create USB_PROD resource\n",
								__func__);
			return;
		}

		/* Create USB_CONS entity */
		memset(&usb_cons_create_params, 0,
					sizeof(usb_cons_create_params));
		usb_cons_create_params.name = ipa_rm_resource_cons[cur_bam];
		usb_cons_create_params.request_resource =
						request_resource_cb[cur_bam];
		usb_cons_create_params.release_resource =
						release_resource_cb[cur_bam];
		ret = ipa_rm_create_resource(&usb_cons_create_params);
		if (ret) {
			pr_err("%s: Failed to create USB_CONS resource\n",
								__func__);
			return ;
		}
	}
}

static void wait_for_prod_granted(enum usb_bam cur_bam)
{
	int ret;

	pr_debug("%s Request %s_PROD_RES\n", __func__,
		bam_enable_strings[cur_bam]);
	if (info.cur_cons_state[cur_bam] == IPA_RM_RESOURCE_GRANTED)
		pr_debug("%s: CONS already granted for some reason\n",
			__func__);
	if (info.cur_prod_state[cur_bam] == IPA_RM_RESOURCE_GRANTED)
		pr_debug("%s: PROD already granted for some reason\n",
			__func__);

	init_completion(&info.prod_avail[cur_bam]);

	ret = ipa_rm_request_resource(ipa_rm_resource_prod[cur_bam]);
	if (!ret) {
		info.cur_prod_state[cur_bam] = IPA_RM_RESOURCE_GRANTED;
		complete_all(&info.prod_avail[cur_bam]);
		pr_debug("%s: PROD_GRANTED without wait\n", __func__);
	} else if (ret == -EINPROGRESS) {
		pr_debug("%s: Waiting for PROD_GRANTED\n", __func__);
		if (!wait_for_completion_timeout(&info.prod_avail[cur_bam],
			USB_BAM_TIMEOUT))
			pr_err("%s: Timeout wainting for PROD_GRANTED\n",
				__func__);
	} else
		pr_err("%s: ipa_rm_request_resource ret =%d\n", __func__, ret);
}

void notify_usb_connected(enum usb_bam cur_bam)
{
	pr_debug("%s: enter\n", __func__);

	spin_lock(&usb_bam_ipa_handshake_info_lock);
	if (cur_bam == HSUSB_BAM)
		info.connect_complete = 1;
	spin_unlock(&usb_bam_ipa_handshake_info_lock);

	if (info.cur_cons_state[cur_bam] == IPA_RM_RESOURCE_GRANTED) {
		pr_debug("%s: Notify %s_CONS_GRANTED\n", __func__,
			bam_enable_strings[cur_bam]);
		ipa_rm_notify_completion(IPA_RM_RESOURCE_GRANTED,
				 ipa_rm_resource_cons[cur_bam]);
	}
}

static void wait_for_prod_release(enum usb_bam cur_bam)
{
	int ret;

	if (info.cur_cons_state[cur_bam] == IPA_RM_RESOURCE_RELEASED)
		pr_debug("%s consumer already released\n", __func__);
	 if (info.cur_prod_state[cur_bam] == IPA_RM_RESOURCE_RELEASED)
		pr_debug("%s producer already released\n", __func__);

	init_completion(&info.prod_released[cur_bam]);
	pr_debug("%s: Releasing %s_PROD\n", __func__,
				bam_enable_strings[cur_bam]);
	ret = ipa_rm_release_resource(ipa_rm_resource_prod[cur_bam]);
	if (!ret) {
		pr_debug("%s: Released without waiting\n", __func__);
		info.cur_prod_state[cur_bam] = IPA_RM_RESOURCE_RELEASED;
		complete_all(&info.prod_released[cur_bam]);
	} else if (ret == -EINPROGRESS) {
		pr_debug("%s: Waiting for PROD_RELEASED\n", __func__);
		if (!wait_for_completion_timeout(&info.prod_released[cur_bam],
						USB_BAM_TIMEOUT))
			pr_err("%s: Timeout waiting for PROD_RELEASED\n",
			__func__);
	} else
		pr_err("%s: ipa_rm_request_resource ret =%d", __func__, ret);
}

static int check_pipes_empty(u8 src_idx, u8 dst_idx)
{
	struct sps_pipe *prod_pipe, *cons_pipe;
	u32 prod_empty, cons_empty;

	/* If we have any remaints in the pipes we don't go to sleep */
	prod_pipe = ctx.usb_bam_sps.sps_pipes[src_idx];
	cons_pipe = ctx.usb_bam_sps.sps_pipes[dst_idx];
	if (sps_is_pipe_empty(prod_pipe, &prod_empty) ||
			sps_is_pipe_empty(cons_pipe, &cons_empty)) {
		pr_err("%s: sps_is_pipe_empty failed with\n", __func__);
		return 0;
	}
	if (!prod_empty || !cons_empty) {
		pr_err("%s: pipes not empty prod=%d cond=%d", __func__,
			prod_empty, cons_empty);
		return 0;
	}

	return 1;
}

void usb_bam_suspend(struct usb_bam_connect_ipa_params *ipa_params)
{
	struct usb_bam_pipe_connect *pipe_connect;
	enum usb_bam cur_bam;
	u8 src_idx, dst_idx;

	if (!ipa_params) {
		pr_err("%s: Invalid ipa params\n", __func__);
		return;
	}

	src_idx = ipa_params->src_idx;
	dst_idx = ipa_params->dst_idx;

	if (src_idx >= ctx.max_connections || dst_idx >= ctx.max_connections) {
		pr_err("%s: Invalid connection index src=%d dst=%d\n",
			__func__, src_idx, dst_idx);
	}

	pipe_connect = &usb_bam_connections[src_idx];
	cur_bam = pipe_connect->bam_type;
	if (cur_bam != HSUSB_BAM)
		return;

	info.src_idx = src_idx;
	info.dst_idx = dst_idx;

	pr_err("%s: Starting suspend sequence(BAM=%s)\n", __func__,
			bam_enable_strings[cur_bam]);

	spin_lock(&usb_bam_ipa_handshake_info_lock);
	info.bus_suspend = 1;
	spin_unlock(&usb_bam_ipa_handshake_info_lock);

	/* Stop PROD transfers */
	if (info.stop) {
		spin_lock(&usb_bam_ipa_handshake_info_lock);
		info.stop(info.start_stop_param, USB_TO_PEER_PERIPHERAL);
		info.prod_stopped = true;
		spin_unlock(&usb_bam_ipa_handshake_info_lock);
	}

	spin_lock(&usb_bam_ipa_handshake_info_lock);
	/* If cable was disconnected, let disconnection seq do everything */
	if (info.disconnected) {
		spin_unlock(&usb_bam_ipa_handshake_info_lock);
		pr_debug("%s: Cable disconnected\n", __func__);
		return;
	}

	/* Don't go to LPM if data in the pipes */
	if (!check_pipes_empty(src_idx, dst_idx)) {
		spin_unlock(&usb_bam_ipa_handshake_info_lock);
		pr_err("%s: pipes not empty, won't start suspend", __func__);
		return;
	}
	spin_unlock(&usb_bam_ipa_handshake_info_lock);

	queue_work(ctx.usb_bam_wq, &info.suspend_work);
}

static void usb_bam_start_suspend(struct work_struct *w)
{
	pr_debug("%s: enter", __func__);
	mutex_lock(&info.suspend_resume_mutex);

	spin_lock(&usb_bam_ipa_handshake_info_lock);
	/* If cable was disconnected, let disconnection seq do everything */
	if (info.disconnected) {
		spin_unlock(&usb_bam_ipa_handshake_info_lock);
		mutex_unlock(&info.suspend_resume_mutex);
		pr_debug("%s: Cable disconnected\n", __func__);
		return;
	}

	if (!info.bus_suspend) {
		spin_unlock(&usb_bam_ipa_handshake_info_lock);
		pr_err("%s: Resume started, not suspending", __func__);
		mutex_unlock(&info.suspend_resume_mutex);
		return;
	}

	/* Stop PROD transfers in case they were started */
	if (info.stop && !info.prod_stopped) {
		info.stop(info.start_stop_param, USB_TO_PEER_PERIPHERAL);
		info.prod_stopped = true;
	}
	spin_unlock(&usb_bam_ipa_handshake_info_lock);

	/* Don't start LPM seq if data in the pipes */
	if (!check_pipes_empty(info.src_idx, info.dst_idx)) {
		mutex_unlock(&info.suspend_resume_mutex);
		return;
	}

	spin_lock(&usb_bam_ipa_handshake_info_lock);
	info.lpm_wait_handshake[HSUSB_BAM] = true;
	spin_unlock(&usb_bam_ipa_handshake_info_lock);

	wait_for_prod_release(HSUSB_BAM);

	mutex_unlock(&info.suspend_resume_mutex);
	if (info.cur_cons_state[HSUSB_BAM] == IPA_RM_RESOURCE_RELEASED)
		usb_bam_finish_suspend();
	else
		pr_debug("Consumer not released yet\n");
}

static void usb_bam_finish_resume(struct work_struct *w)
{
	struct usb_phy *trans = usb_get_transceiver();

	BUG_ON(trans == NULL);
	pr_debug("%s: enter", __func__);
	mutex_lock(&info.suspend_resume_mutex);
	/* Suspend happened in the meantime */
	spin_lock(&usb_bam_ipa_handshake_info_lock);
	if (info.bus_suspend) {
		spin_unlock(&usb_bam_ipa_handshake_info_lock);
		pr_err("%s: Bus suspended, not resuming", __func__);
		mutex_unlock(&info.suspend_resume_mutex);
		return;
	}
	info.lpm_wait_handshake[HSUSB_BAM] = true;
	spin_unlock(&usb_bam_ipa_handshake_info_lock);

	wait_for_prod_granted(HSUSB_BAM);
	notify_usb_connected(HSUSB_BAM);
	if (info.cons_stopped) {
		ipa_resume_pipes();
		if (info.start) {
			pr_debug("%s: Enqueue CONS transfer", __func__);
			info.start(info.start_stop_param,
				PEER_PERIPHERAL_TO_USB);
			info.cons_stopped = 0;
		}
	}

	if (info.start) {
		spin_lock(&usb_bam_ipa_handshake_info_lock);
		info.start(info.start_stop_param, USB_TO_PEER_PERIPHERAL);
		info.prod_stopped = false;
		spin_unlock(&usb_bam_ipa_handshake_info_lock);
	}
	if (info.cur_cons_state[HSUSB_BAM] == IPA_RM_RESOURCE_GRANTED) {
		pr_debug("%s: Notify CONS_GRANTED\n", __func__);
		ipa_rm_notify_completion(IPA_RM_RESOURCE_GRANTED,
				 ipa_rm_resource_cons[HSUSB_BAM]);
	}
	mutex_unlock(&info.suspend_resume_mutex);
	pr_debug("%s: done", __func__);
}

void usb_bam_resume(struct usb_bam_connect_ipa_params *ipa_params)
{
	enum usb_bam cur_bam;
	u8 src_idx, dst_idx;
	struct usb_bam_pipe_connect *pipe_connect;

	pr_debug("%s: Resuming\n", __func__);

	if (!ipa_params) {
		pr_err("%s: Invalid ipa params\n", __func__);
		return;
	}

	src_idx = ipa_params->src_idx;
	dst_idx = ipa_params->dst_idx;

	if (src_idx >= ctx.max_connections || dst_idx >= ctx.max_connections) {
		pr_err("%s: Invalid connection index src=%d dst=%d\n",
			__func__, src_idx, dst_idx);
		return;
	}

	pipe_connect = &usb_bam_connections[src_idx];
	cur_bam = pipe_connect->bam_type;
	if (cur_bam != HSUSB_BAM)
		return;

	info.in_lpm[HSUSB_BAM] = false;
	spin_lock(&usb_bam_ipa_handshake_info_lock);
	info.bus_suspend = 0;
	spin_unlock(&usb_bam_ipa_handshake_info_lock);
	queue_work(ctx.usb_bam_wq, &info.resume_work);
}

void msm_bam_wait_for_hsic_prod_granted(void)
{
	spin_lock(&usb_bam_lock);

	ctx.is_bam_inactivity[HSIC_BAM] = false;

	/* Get back to resume state including wakeup ipa */
	usb_bam_resume_hsic_host();

	/* Ensure getting the producer resource */
	wait_for_prod_granted(HSIC_BAM);

	spin_unlock(&usb_bam_lock);
}

void msm_bam_hsic_notify_on_resume(void)
{
	spin_lock(&usb_bam_lock);

	hsic_host_info.in_lpm = false;

	/* HSIC resume completed. Notify CONS grant if CONS was requested */
	notify_usb_connected(HSIC_BAM);

	/*
	 * This function is called to notify the usb bam driver
	 * that the hsic core and hsic bam hw are fully resumed
	 * and clocked on. Therefore we can now set the inactivity
	 * timer to the hsic bam hw.
	 */
	if (ctx.inactivity_timer_ms[HSIC_BAM])
		usb_bam_set_inactivity_timer(HSIC_BAM);

	spin_unlock(&usb_bam_lock);
}

bool msm_bam_hsic_lpm_ok(void)
{
	int i;
	struct usb_bam_pipe_connect *pipe_iter;

	if (hsic_host_info.dev) {

		pr_debug("%s: Starting hsic full suspend sequence\n",
			__func__);

		/*
		 * Start low power mode by releasing the device
		 * only in case that indeed the resources were released
		 * and we are still in inactivity state (wake event
		 * have not been occured while we were waiting to the
		 * resources release)
		 */
		spin_lock(&usb_bam_lock);

		if (info.cur_cons_state[HSIC_BAM] ==
			IPA_RM_RESOURCE_RELEASED &&
		    info.cur_prod_state[HSIC_BAM] ==
			IPA_RM_RESOURCE_RELEASED &&
		    ctx.is_bam_inactivity[HSIC_BAM] && info.in_lpm[HSIC_BAM]) {

			/* HSIC host will go now to lpm */
			pr_debug("%s: vote for suspend hsic %x\n",
				__func__, (int)hsic_host_info.dev);

			for (i = 0; i < ctx.max_connections; i++) {
				pipe_iter =
					&usb_bam_connections[i];
				if (pipe_iter->bam_type == HSIC_BAM &&
				    pipe_iter->enabled &&
				    !pipe_iter->suspended) {
					spin_unlock(&usb_bam_lock);
					ipa_suspend(
					   pipe_iter->ipa_clnt_hdl);
					pipe_iter->suspended = true;
					spin_lock(&usb_bam_lock);
				}
			}

			hsic_host_info.in_lpm = true;

			spin_unlock(&usb_bam_lock);
			return true;
		}

		/* We don't allow lpm, therefore renew our vote here */
		if (info.in_lpm[HSIC_BAM]) {
			pr_err("%s: Not allow lpm while ref count=0\n",
				__func__);
			pr_err("%s: inactivity=%d, c_s=%d p_s=%d lpm=%d\n",
				__func__, ctx.is_bam_inactivity[HSIC_BAM],
				info.cur_cons_state[HSIC_BAM],
				info.cur_prod_state[HSIC_BAM],
				info.in_lpm[HSIC_BAM]);
			pm_runtime_get(hsic_host_info.dev);
			info.in_lpm[HSIC_BAM] = false;
			spin_unlock(&usb_bam_lock);
		} else
			spin_unlock(&usb_bam_lock);

		return false;
	}

	return true;
}

int usb_bam_connect_ipa(struct usb_bam_connect_ipa_params *ipa_params)
{
	u8 idx;
	enum usb_bam cur_bam;
	struct usb_bam_pipe_connect *pipe_connect;
	int ret;
	struct msm_usb_bam_platform_data *pdata =
					ctx.usb_bam_pdev->dev.platform_data;

	if (!ipa_params) {
		pr_err("%s: Invalid ipa params\n",
			__func__);
		return -EINVAL;
	}

	if (ipa_params->dir == USB_TO_PEER_PERIPHERAL)
		idx = ipa_params->src_idx;
	else
		idx = ipa_params->dst_idx;

	if (idx >= ctx.max_connections) {
		pr_err("%s: Invalid connection index\n",
			__func__);
		return -EINVAL;
	}
	pipe_connect = &usb_bam_connections[idx];
	cur_bam = pipe_connect->bam_type;

	if (pipe_connect->enabled) {
		pr_err("%s: connection %d was already established\n",
			__func__, idx);
		return 0;
	}

	pr_debug("%s: enter", __func__);

	if (cur_bam == HSUSB_BAM) {
		mutex_lock(&info.suspend_resume_mutex);

		spin_lock(&usb_bam_lock);
		if (ctx.pipes_enabled_per_bam[HSUSB_BAM] == 0) {
			spin_unlock(&usb_bam_lock);
			spin_lock(&usb_bam_ipa_handshake_info_lock);
			info.lpm_wait_handshake[HSUSB_BAM] = true;
			info.connect_complete = 0;
			info.disconnected = 0;
			info.pending_lpm = 0;
			info.lpm_wait_pipes = 1;
			info.bus_suspend = 0;
			info.cons_stopped = 0;
			info.prod_stopped = 0;
			spin_unlock(&usb_bam_ipa_handshake_info_lock);
			usb_bam_resume_core(cur_bam);
		} else
			spin_unlock(&usb_bam_lock);
	}

	 /* Check if BAM requires RESET before connect and reset first pipe */
	 spin_lock(&usb_bam_lock);
	 if ((pdata->reset_on_connect[cur_bam] == true) &&
	     (ctx.pipes_enabled_per_bam[cur_bam] == 0)) {
		spin_unlock(&usb_bam_lock);
		sps_device_reset(ctx.h_bam[cur_bam]);

		/* On re-connect assume out from lpm for HSIC BAM */
		if (cur_bam == HSIC_BAM && hsic_host_info.dev &&
		    info.in_lpm[HSIC_BAM]) {
			pr_debug("%s: Getting hsic device %x\n",
					__func__, (int)hsic_host_info.dev);
			pm_runtime_get(hsic_host_info.dev);
		}

		/* On re-connect assume out from lpm for all BAMs */
		info.in_lpm[cur_bam] = false;
	 } else
		spin_unlock(&usb_bam_lock);

	if (ipa_params->dir == USB_TO_PEER_PERIPHERAL) {
		pr_debug("%s: Starting connect sequence\n", __func__);
		wait_for_prod_granted(cur_bam);
	}

	ret = connect_pipe_ipa(idx, ipa_params);
	if (ret) {
		pr_err("%s: pipe connection failure\n", __func__);
		if (cur_bam == HSUSB_BAM)
			mutex_unlock(&info.suspend_resume_mutex);
		return ret;
	}

	spin_lock(&usb_bam_lock);
	pipe_connect->enabled = 1;
	pipe_connect->suspended = 0;

	/* Set global inactivity timer upon first pipe connection */
	if (ctx.pipes_enabled_per_bam[pipe_connect->bam_type] == 0 &&
		ctx.inactivity_timer_ms[pipe_connect->bam_type] &&
		pipe_connect->inactivity_notify)
		usb_bam_set_inactivity_timer(pipe_connect->bam_type);

	ctx.pipes_enabled_per_bam[cur_bam] += 1;
	spin_unlock(&usb_bam_lock);
	if (ipa_params->dir == PEER_PERIPHERAL_TO_USB)
		notify_usb_connected(cur_bam);

	if (cur_bam == HSUSB_BAM)
		mutex_unlock(&info.suspend_resume_mutex);

	pr_debug("%s: done", __func__);

	return 0;
}
EXPORT_SYMBOL(usb_bam_connect_ipa);

int usb_bam_client_ready(bool ready)
{
	spin_lock(&usb_bam_peer_handshake_info_lock);
	if (peer_handshake_info.client_ready == ready) {
		pr_debug("%s: client state is already %d\n",
			__func__, ready);
		spin_unlock(&usb_bam_peer_handshake_info_lock);
		return 0;
	}

	peer_handshake_info.client_ready = ready;
	if (peer_handshake_info.state == USB_BAM_SM_PLUG_ACKED && !ready) {
		pr_debug("Starting reset sequence");
		INIT_COMPLETION(ctx.reset_done);
	}

	spin_unlock(&usb_bam_peer_handshake_info_lock);
	if (!queue_work(ctx.usb_bam_wq,
			&peer_handshake_info.reset_event.event_w)) {
		spin_lock(&usb_bam_peer_handshake_info_lock);
		peer_handshake_info.pending_work++;
		spin_unlock(&usb_bam_peer_handshake_info_lock);
	}

	return 0;
}

static void usb_bam_work(struct work_struct *w)
{
	int i;
	struct usb_bam_event_info *event_info =
		container_of(w, struct usb_bam_event_info, event_w);
	struct usb_bam_pipe_connect *pipe_connect =
		container_of(event_info, struct usb_bam_pipe_connect, event);
	struct usb_bam_pipe_connect *pipe_iter;
	int (*callback)(void *priv);
	void *param = NULL;

	switch (event_info->type) {
	case USB_BAM_EVENT_WAKEUP:
	case USB_BAM_EVENT_WAKEUP_PIPE:

		pr_debug("%s recieved USB_BAM_EVENT_WAKEUP\n", __func__);

		/*
		 * Make sure the PROD resource is granted before
		 * wakeup hsic host class driver (done by the callback below)
		 */
		if (pipe_connect->peer_bam == IPA_P_BAM &&
		    pipe_connect->bam_type == HSIC_BAM &&
		    info.cur_prod_state[HSIC_BAM] != IPA_RM_RESOURCE_GRANTED) {
			wait_for_prod_granted(HSIC_BAM);
		}

		/*
		 * Check if need to resume the hsic host.
		 * On one hand, since we got the wakeup interrupt
		 * the hsic bam clocks are already enabled, so no need
		 * to actualluy resume the hardware... However, we still need
		 * to update the usb bam driver state (to set in_lpm=false),
		 * and to wake ipa (ipa_resume) and to hold again the hsic host
		 * device again to avoid it going to low poer mode next time
		 * until we complete releasing the hsic consumer and producer
		 * resources against the ipa resource manager.
		 */
		spin_lock(&usb_bam_lock);
		if (pipe_connect->bam_type == HSIC_BAM)
			usb_bam_resume_hsic_host();
		spin_unlock(&usb_bam_lock);

		/* Notify about wakeup / activity of the bam */
		if (event_info->callback)
			event_info->callback(event_info->param);

		/*
		 * Reset inactivity timer counter if this pipe's bam
		 * has inactivity timeout.
		 */
		spin_lock(&usb_bam_lock);
		if (ctx.inactivity_timer_ms[pipe_connect->bam_type])
			usb_bam_set_inactivity_timer(pipe_connect->bam_type);
		spin_unlock(&usb_bam_lock);

		if (pipe_connect->bam_type == HSUSB_BAM) {
			/* A2 wakeup not from LPM (CONS was up) */
			wait_for_prod_granted(pipe_connect->bam_type);
			if (info.start) {
				pr_debug("%s: Enqueue PROD transfer", __func__);
				info.start(info.start_stop_param,
						USB_TO_PEER_PERIPHERAL);
			}
		}

		break;

	case USB_BAM_EVENT_INACTIVITY:

		pr_debug("%s recieved USB_BAM_EVENT_INACTIVITY\n", __func__);

		/*
		 * Since event info is one structure per pipe, it might be
		 * overriden when we will register the wakeup events below,
		 * and still we want ot register the wakeup events before we
		 * notify on the inactivity in order to identify the next
		 * activity as soon as possible.
		 */
		callback = event_info->callback;
		param = event_info->param;

		/*
		 * Upon inactivity, configure wakeup irq for all pipes
		 * that are into the usb bam.
		 */
		spin_lock(&usb_bam_lock);
		for (i = 0; i < ctx.max_connections; i++) {
			pipe_iter = &usb_bam_connections[i];
			if (pipe_iter->bam_type ==
				pipe_connect->bam_type &&
			    pipe_iter->dir ==
				PEER_PERIPHERAL_TO_USB &&
				pipe_iter->enabled) {
				pr_debug("%s: Register wakeup on pipe %x\n",
					__func__, (int)pipe_iter);
				__usb_bam_register_wake_cb(i,
					pipe_iter->activity_notify,
					pipe_iter->priv,
					false);
			}
		}
		spin_unlock(&usb_bam_lock);

		/* Notify about the inactivity to the USB class driver */
		if (callback)
			callback(param);

		wait_for_prod_release(pipe_connect->bam_type);
		pr_debug("%s: complete wait on hsic producer s=%d\n",
			__func__, info.cur_prod_state[pipe_connect->bam_type]);

		/*
		 * Allow to go to lpm for now if also consumer is down.
		 * If consumer is up, we will wait to the release consumer
		 * notification.
		 */
		if (hsic_host_info.dev &&
		    info.cur_cons_state[HSIC_BAM] ==
		    IPA_RM_RESOURCE_RELEASED && !info.in_lpm[HSIC_BAM]) {
			pr_debug("%s: Putting hsic device %x\n", __func__,
			(int)hsic_host_info.dev);
			pm_runtime_put(hsic_host_info.dev);
			info.in_lpm[HSIC_BAM] = true;
		}

		break;
	default:
		pr_err("%s: unknown usb bam event type %d\n", __func__,
			event_info->type);
	}
}

static void usb_bam_wake_cb(struct sps_event_notify *notify)
{
	struct usb_bam_event_info *event_info =
		(struct usb_bam_event_info *)notify->user;
	struct usb_bam_pipe_connect *pipe_connect =
		container_of(event_info,
			     struct usb_bam_pipe_connect,
			     event);
	enum usb_bam bam = pipe_connect->bam_type;

	spin_lock(&usb_bam_lock);

	if (event_info->type == USB_BAM_EVENT_WAKEUP_PIPE)
		queue_work(ctx.usb_bam_wq, &event_info->event_w);
	else if (event_info->type == USB_BAM_EVENT_WAKEUP &&
			ctx.is_bam_inactivity[bam]) {

		/*
		 * Sps wake event is per pipe, so usb_bam_wake_cb is
		 * called per pipe. However, we want to filter the wake
		 * event to be wake event per all the pipes.
		 * Therefore, the first pipe that awaked will be considered
		 * as global bam wake event.
		 */
		ctx.is_bam_inactivity[bam] = false;

		queue_work(ctx.usb_bam_wq, &event_info->event_w);
	}

	spin_unlock(&usb_bam_lock);
}

static void usb_bam_sm_work(struct work_struct *w)
{
	pr_debug("%s: current state: %d\n", __func__,
		peer_handshake_info.state);

	spin_lock(&usb_bam_peer_handshake_info_lock);

	switch (peer_handshake_info.state) {
	case USB_BAM_SM_INIT:
		if (peer_handshake_info.client_ready) {
			spin_unlock(&usb_bam_peer_handshake_info_lock);
			smsm_change_state(SMSM_APPS_STATE, 0,
				SMSM_USB_PLUG_UNPLUG);
			spin_lock(&usb_bam_peer_handshake_info_lock);
			peer_handshake_info.state = USB_BAM_SM_PLUG_NOTIFIED;
		}
		break;
	case USB_BAM_SM_PLUG_NOTIFIED:
		if (peer_handshake_info.ack_received) {
			peer_handshake_info.state = USB_BAM_SM_PLUG_ACKED;
			peer_handshake_info.ack_received = 0;
		}
		break;
	case USB_BAM_SM_PLUG_ACKED:
		if (!peer_handshake_info.client_ready) {
			spin_unlock(&usb_bam_peer_handshake_info_lock);
			pr_debug("Starting A2 reset sequence");
			smsm_change_state(SMSM_APPS_STATE,
				SMSM_USB_PLUG_UNPLUG, 0);
			spin_lock(&usb_bam_peer_handshake_info_lock);
			peer_handshake_info.state = USB_BAM_SM_UNPLUG_NOTIFIED;
		}
		break;
	case USB_BAM_SM_UNPLUG_NOTIFIED:
		if (peer_handshake_info.ack_received) {
			spin_unlock(&usb_bam_peer_handshake_info_lock);
			peer_handshake_info.reset_event.
				callback(peer_handshake_info.reset_event.param);
			spin_lock(&usb_bam_peer_handshake_info_lock);
			complete_all(&ctx.reset_done);
			pr_debug("Finished reset sequence");
			peer_handshake_info.state = USB_BAM_SM_INIT;
			peer_handshake_info.ack_received = 0;
		}
		break;
	}

	if (peer_handshake_info.pending_work) {
		peer_handshake_info.pending_work--;
		spin_unlock(&usb_bam_peer_handshake_info_lock);
		queue_work(ctx.usb_bam_wq,
			&peer_handshake_info.reset_event.event_w);
		spin_lock(&usb_bam_peer_handshake_info_lock);
	}
	spin_unlock(&usb_bam_peer_handshake_info_lock);
}

static void usb_bam_ack_toggle_cb(void *priv,
	uint32_t old_state, uint32_t new_state)
{
	static int last_processed_state;
	int current_state;

	spin_lock(&usb_bam_peer_handshake_info_lock);

	current_state = new_state & SMSM_USB_PLUG_UNPLUG;

	if (current_state == last_processed_state) {
		spin_unlock(&usb_bam_peer_handshake_info_lock);
		return;
	}

	last_processed_state = current_state;
	peer_handshake_info.ack_received = true;

	spin_unlock(&usb_bam_peer_handshake_info_lock);
	if (!queue_work(ctx.usb_bam_wq,
			&peer_handshake_info.reset_event.event_w)) {
		spin_lock(&usb_bam_peer_handshake_info_lock);
		peer_handshake_info.pending_work++;
		spin_unlock(&usb_bam_peer_handshake_info_lock);
	}
}

static int __usb_bam_register_wake_cb(u8 idx, int (*callback)(void *user),
	void *param, bool trigger_cb_per_pipe)
{
	struct sps_pipe *pipe = ctx.usb_bam_sps.sps_pipes[idx];
	struct sps_connect *sps_connection;
	struct usb_bam_pipe_connect *pipe_connect;
	struct usb_bam_event_info *wake_event_info;
	int ret;

	if (idx < 0 || idx > ctx.max_connections) {
		pr_err("%s:idx is wrong %d", __func__, idx);
		return -EINVAL;
	}
	pipe = ctx.usb_bam_sps.sps_pipes[idx];
	sps_connection = &ctx.usb_bam_sps.sps_connections[idx];
	pipe_connect = &usb_bam_connections[idx];
	wake_event_info = &pipe_connect->event;

	wake_event_info->type = (trigger_cb_per_pipe ?
				USB_BAM_EVENT_WAKEUP_PIPE :
				USB_BAM_EVENT_WAKEUP);
	wake_event_info->param = param;
	wake_event_info->callback = callback;
	wake_event_info->event.mode = SPS_TRIGGER_CALLBACK;
	wake_event_info->event.xfer_done = NULL;
	wake_event_info->event.callback = callback ? usb_bam_wake_cb : NULL;
	wake_event_info->event.user = wake_event_info;
	wake_event_info->event.options = SPS_O_WAKEUP;
	ret = sps_register_event(pipe, &wake_event_info->event);
	if (ret) {
		pr_err("%s: sps_register_event() failed %d\n", __func__, ret);
		return ret;
	}

	sps_connection->options = callback ?
		(SPS_O_AUTO_ENABLE | SPS_O_WAKEUP | SPS_O_WAKEUP_IS_ONESHOT) :
		SPS_O_AUTO_ENABLE;
	ret = sps_set_config(pipe, sps_connection);
	if (ret) {
		pr_err("%s: sps_set_config() failed %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

int usb_bam_register_wake_cb(u8 idx, int (*callback)(void *user),
	void *param)
{
	info.wake_cb = callback;
	info.wake_param = param;
	return __usb_bam_register_wake_cb(idx, callback, param, true);
}

int usb_bam_register_start_stop_cbs(
	void (*start)(void *, enum usb_bam_pipe_dir),
	void (*stop)(void *, enum usb_bam_pipe_dir),
	void *param)
{
	info.start = start;
	info.stop = stop;
	info.start_stop_param = param;
	return 0;
}

int usb_bam_register_peer_reset_cb(int (*callback)(void *), void *param)
{
	u32 ret = 0;

	if (callback) {
		peer_handshake_info.reset_event.param = param;
		peer_handshake_info.reset_event.callback = callback;

		ret = smsm_state_cb_register(SMSM_MODEM_STATE,
			SMSM_USB_PLUG_UNPLUG, usb_bam_ack_toggle_cb, NULL);
		if (ret) {
			pr_err("%s: failed to register SMSM callback\n",
				__func__);
		} else {
			if (smsm_get_state(SMSM_MODEM_STATE) &
				SMSM_USB_PLUG_UNPLUG)
				usb_bam_ack_toggle_cb(NULL, 0,
					SMSM_USB_PLUG_UNPLUG);
		}
	} else {
		peer_handshake_info.reset_event.param = NULL;
		peer_handshake_info.reset_event.callback = NULL;
		smsm_state_cb_deregister(SMSM_MODEM_STATE,
			SMSM_USB_PLUG_UNPLUG, usb_bam_ack_toggle_cb, NULL);
	}

	return ret;
}

int usb_bam_disconnect_pipe(u8 idx)
{
	struct usb_bam_pipe_connect *pipe_connect;
	int ret;

	pipe_connect = &usb_bam_connections[idx];

	if (!pipe_connect->enabled) {
		pr_err("%s: connection %d isn't enabled\n",
			__func__, idx);
		return 0;
	}

	ret = disconnect_pipe(idx);
	if (ret) {
		pr_err("%s: src pipe disconnection failure\n", __func__);
		return ret;
	}

	pipe_connect->enabled = 0;
	spin_lock(&usb_bam_lock);
	if (ctx.pipes_enabled_per_bam[pipe_connect->bam_type] == 0)
		pr_err("%s: wrong pipes enabled counter for bam_type=%d\n",
			__func__, pipe_connect->bam_type);
	else
		ctx.pipes_enabled_per_bam[pipe_connect->bam_type] -= 1;
	spin_unlock(&usb_bam_lock);

	return 0;
}

int usb_bam_disconnect_ipa(struct usb_bam_connect_ipa_params *ipa_params)
{
	int ret;
	u8 idx;
	struct usb_bam_pipe_connect *pipe_connect;
	struct sps_connect *sps_connection;
	enum usb_bam cur_bam;

	if (!ipa_params->prod_clnt_hdl && !ipa_params->cons_clnt_hdl) {
		pr_err("%s: Both of the handles is missing\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: Starting disconnect sequence\n", __func__);

	mutex_lock(&info.suspend_resume_mutex);
	/* Delay USB core to go into lpm before we finish our handshake */
	if (ipa_params->prod_clnt_hdl) {
		idx = ipa_params->dst_idx;
		pipe_connect = &usb_bam_connections[idx];

		pipe_connect->activity_notify = NULL;
		pipe_connect->inactivity_notify = NULL;
		pipe_connect->priv = NULL;

		/* Do the release handshake with the A2 via RM */
		cur_bam = pipe_connect->bam_type;
		spin_lock(&usb_bam_ipa_handshake_info_lock);
		if (cur_bam == HSUSB_BAM) {
			info.connect_complete = 0;
			info.lpm_wait_pipes = 1;
			info.disconnected = 1;
		}
		spin_unlock(&usb_bam_ipa_handshake_info_lock);
		wait_for_prod_release(cur_bam);
		/* close USB -> IPA pipe */
		usb_bam_resume_core(cur_bam);
		ret = ipa_disconnect(ipa_params->prod_clnt_hdl);
		if (ret) {
			pr_err("%s: dst pipe disconnection failure\n",
				__func__);
			mutex_unlock(&info.suspend_resume_mutex);
			return ret;
		}
		sps_connection = &ctx.usb_bam_sps.sps_connections[idx];
		sps_connection->data.phys_base = 0;
		sps_connection->desc.phys_base = 0;

		ret = usb_bam_disconnect_pipe(idx);
		if (ret) {
			pr_err("%s: failure to disconnect pipe %d\n",
				__func__, idx);
			mutex_unlock(&info.suspend_resume_mutex);
			return ret;
		}
	}

	if (ipa_params->cons_clnt_hdl) {
		idx = ipa_params->src_idx;
		pipe_connect = &usb_bam_connections[idx];

		pipe_connect->activity_notify = NULL;
		pipe_connect->inactivity_notify = NULL;
		pipe_connect->priv = NULL;

		cur_bam = pipe_connect->bam_type;
		/* close IPA -> USB pipe */
		ret = ipa_disconnect(ipa_params->cons_clnt_hdl);
		if (ret) {
			pr_err("%s: src pipe disconnection failure\n",
				__func__);
			mutex_unlock(&info.suspend_resume_mutex);
			return ret;
		}

		sps_connection = &ctx.usb_bam_sps.sps_connections[idx];
		sps_connection->data.phys_base = 0;
		sps_connection->desc.phys_base = 0;

		ret = usb_bam_disconnect_pipe(idx);
		if (ret) {
			pr_err("%s: failure to disconnect pipe %d\n",
				__func__, idx);
			mutex_unlock(&info.suspend_resume_mutex);
			return ret;
		}

		pipe_connect->ipa_clnt_hdl = -1;

		if (info.cur_cons_state[cur_bam] == IPA_RM_RESOURCE_RELEASED) {
			pr_debug("%s Notify CONS _RELEASED\n", __func__);
			ipa_rm_notify_completion(IPA_RM_RESOURCE_RELEASED,
				ipa_rm_resource_cons[cur_bam]);
		}

		if (cur_bam == HSUSB_BAM) {
			/*
			 * Currently we have for HSUSB BAM only one consumer
			 * pipe. Therefore ending disconnect sequence and
			 * starting hsusb lpm. This limitation will be changed
			 * in future patch.
			 */
			pr_debug("%s Ended disconnect sequence\n", __func__);
			usb_bam_start_lpm(1);
		}

		mutex_unlock(&info.suspend_resume_mutex);
		return 0;
	}

	mutex_unlock(&info.suspend_resume_mutex);
	return 0;
}
EXPORT_SYMBOL(usb_bam_disconnect_ipa);

void usb_bam_reset_complete(void)
{
	pr_debug("Waiting for reset compelte");
	if (wait_for_completion_interruptible_timeout(&ctx.reset_done,
			10*HZ) <= 0)
		pr_warn("Timeout while waiting for reset");

	pr_debug("Finished Waiting for reset complete");
}

int usb_bam_a2_reset(bool to_reconnect)
{
	struct usb_bam_pipe_connect *pipe_connect;
	int i;
	int ret = 0, ret_int;
	u8 bam = -1;
	int reconnect_pipe_idx[ctx.max_connections];

	for (i = 0; i < ctx.max_connections; i++)
		reconnect_pipe_idx[i] = -1;

	/* Disconnect a2 pipes */
	for (i = 0; i < ctx.max_connections; i++) {
		pipe_connect = &usb_bam_connections[i];
		if (strnstr(pipe_connect->name, "a2", USB_BAM_MAX_STR_LEN) &&
				pipe_connect->enabled) {
			if (pipe_connect->dir == USB_TO_PEER_PERIPHERAL)
				reconnect_pipe_idx[i] =
					pipe_connect->src_pipe_index;
			else
				reconnect_pipe_idx[i] =
					pipe_connect->dst_pipe_index;

			bam = pipe_connect->bam_type;
			if (bam < 0) {
				ret = -EINVAL;
				continue;
			}
			ret_int = usb_bam_disconnect_pipe(i);
			if (ret_int) {
				pr_err("%s: failure to connect pipe %d\n",
					__func__, i);
				ret = ret_int;
				continue;
			}
		}
	}
	/* Reset A2 (USB/HSIC) BAM */
	if (bam != -1 && sps_device_reset(ctx.h_bam[bam]))
		pr_err("%s: BAM reset failed\n", __func__);

	if (!to_reconnect)
		return ret;

	/* Reconnect A2 pipes */
	for (i = 0; i < ctx.max_connections; i++) {
		pipe_connect = &usb_bam_connections[i];
		if (reconnect_pipe_idx[i] != -1) {
			ret_int = usb_bam_connect(i, &reconnect_pipe_idx[i]);
			if (ret_int) {
				pr_err("%s: failure to reconnect pipe %d\n",
					__func__, i);
				ret = ret_int;
				continue;
			}
		}
	}

	return ret;
}

static void usb_bam_sps_events(enum sps_callback_case sps_cb_case, void *user)
{
	int i;
	enum usb_bam bam;
	struct usb_bam_pipe_connect *pipe_connect;
	struct usb_bam_event_info *event_info;

	switch (sps_cb_case) {
	case SPS_CALLBACK_BAM_TIMER_IRQ:

		pr_debug("%s:recieved SPS_CALLBACK_BAM_TIMER_IRQ\n", __func__);

		spin_lock(&usb_bam_lock);

		bam = get_bam_type_from_core_name((char *)user);
		ctx.is_bam_inactivity[bam] = true;
		pr_debug("%s: Incativity happened on bam=%s,%d\n", __func__,
			(char *)user, bam);

		for (i = 0; i < ctx.max_connections; i++) {
			pipe_connect = &usb_bam_connections[i];

			/*
			 * Notify inactivity once, Since it is global
			 * for all pipes on bam. Notify only if we have
			 * connected pipes.
			 */
			if (pipe_connect->bam_type == bam &&
			    pipe_connect->enabled) {
				event_info = &pipe_connect->event;
				event_info->type = USB_BAM_EVENT_INACTIVITY;
				event_info->param = pipe_connect->priv;
				event_info->callback =
					pipe_connect->inactivity_notify;
				queue_work(ctx.usb_bam_wq,
						&event_info->event_w);
				break;
			}
		}

		spin_unlock(&usb_bam_lock);

		break;
	default:
		pr_debug("%s:received sps_cb_case=%d\n", __func__,
			(int)sps_cb_case);
	}
}

static struct msm_usb_bam_platform_data *usb_bam_dt_to_pdata(
	struct platform_device *pdev)
{
	struct msm_usb_bam_platform_data *pdata;
	struct device_node *node = pdev->dev.of_node;
	int rc = 0;
	u8 i = 0;
	bool reset_bam;
	enum usb_bam bam;
	u32 addr;

	ctx.max_connections = 0;
	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("unable to allocate platform data\n");
		return NULL;
	}

	rc = of_property_read_u32(node, "qcom,usb-bam-num-pipes",
		&pdata->usb_bam_num_pipes);
	if (rc) {
		pr_err("Invalid usb bam num pipes property\n");
		return NULL;
	}

	rc = of_property_read_u32(node, "qcom,usb-bam-fifo-baseaddr",
			&addr);
	if (rc)
		pr_debug("%s: Invalid usb base address property\n", __func__);
	else
		pdata->usb_bam_fifo_baseaddr = addr;

	pdata->ignore_core_reset_ack = of_property_read_bool(node,
		"qcom,ignore-core-reset-ack");

	pdata->disable_clk_gating = of_property_read_bool(node,
		"qcom,disable-clk-gating");

	for_each_child_of_node(pdev->dev.of_node, node)
		ctx.max_connections++;

	if (!ctx.max_connections) {
		pr_err("%s: error: max_connections is zero\n", __func__);
		goto err;
	}

	usb_bam_connections = devm_kzalloc(&pdev->dev, ctx.max_connections *
		sizeof(struct usb_bam_pipe_connect), GFP_KERNEL);

	if (!usb_bam_connections) {
		pr_err("%s: devm_kzalloc failed(%d)\n", __func__,  __LINE__);
		return NULL;
	}

	/* retrieve device tree parameters */
	for_each_child_of_node(pdev->dev.of_node, node) {
		rc = of_property_read_string(node, "label",
			&usb_bam_connections[i].name);
		if (rc)
			goto err;

		rc = of_property_read_u32(node, "qcom,usb-bam-mem-type",
			&usb_bam_connections[i].mem_type);
		if (rc)
			goto err;

		if (usb_bam_connections[i].mem_type == USB_PRIVATE_MEM ||
				usb_bam_connections[i].mem_type == OCI_MEM) {
			if (!pdata->usb_bam_fifo_baseaddr) {
				pr_err("%s: base address is missing\n",
					__func__);
				goto err;
			}
		}

		rc = of_property_read_u32(node, "qcom,bam-type",
			&usb_bam_connections[i].bam_type);
		if (rc) {
			pr_err("%s: bam type is missing in device tree\n",
				__func__);
			goto err;
		}
		bam = usb_bam_connections[i].bam_type;

		rc = of_property_read_u32(node, "qcom,peer-bam",
			&usb_bam_connections[i].peer_bam);
		if (rc) {
			pr_err("%s: peer bam is missing in device tree\n",
				__func__);
			goto err;
		}
		rc = of_property_read_u32(node, "qcom,dir",
			&usb_bam_connections[i].dir);
		if (rc) {
			pr_err("%s: direction is missing in device tree\n",
				__func__);
			goto err;
		}

		rc = of_property_read_u32(node, "qcom,pipe-num",
			&usb_bam_connections[i].pipe_num);
		if (rc) {
			pr_err("%s: pipe num is missing in device tree\n",
				__func__);
			goto err;
		}

		reset_bam = of_property_read_bool(node,
			"qcom,reset-bam-on-connect");
		if (reset_bam)
			pdata->reset_on_connect[bam] = true;

		of_property_read_u32(node, "qcom,src-bam-physical-address",
			&usb_bam_connections[i].src_phy_addr);

		of_property_read_u32(node, "qcom,src-bam-pipe-index",
			&usb_bam_connections[i].src_pipe_index);

		of_property_read_u32(node, "qcom,dst-bam-physical-address",
			&usb_bam_connections[i].dst_phy_addr);

		of_property_read_u32(node, "qcom,dst-bam-pipe-index",
			&usb_bam_connections[i].dst_pipe_index);

		of_property_read_u32(node, "qcom,data-fifo-offset",
			&usb_bam_connections[i].data_fifo_base_offset);

		rc = of_property_read_u32(node, "qcom,data-fifo-size",
			&usb_bam_connections[i].data_fifo_size);
		if (rc)
			goto err;

		of_property_read_u32(node, "qcom,descriptor-fifo-offset",
			&usb_bam_connections[i].desc_fifo_base_offset);

		rc = of_property_read_u32(node, "qcom,descriptor-fifo-size",
			&usb_bam_connections[i].desc_fifo_size);
		if (rc)
			goto err;
		i++;
	}

	pdata->connections = usb_bam_connections;

	return pdata;
err:
	pr_err("%s: failed\n", __func__);
	return NULL;
}

static int usb_bam_init(int bam_idx)
{
	int ret, irq;
	void *usb_virt_addr;
	struct msm_usb_bam_platform_data *pdata =
		ctx.usb_bam_pdev->dev.platform_data;
	struct resource *res, *ram_resource;
	struct sps_bam_props props = ctx.usb_bam_sps.usb_props;

	pr_debug("%s: usb_bam_init - %s\n", __func__,
		bam_enable_strings[bam_idx]);
	res = platform_get_resource_byname(ctx.usb_bam_pdev, IORESOURCE_MEM,
		bam_enable_strings[bam_idx]);
	if (!res) {
		dev_dbg(&ctx.usb_bam_pdev->dev, "bam not initialized\n");
		return 0;
	}

	irq = platform_get_irq_byname(ctx.usb_bam_pdev,
		bam_enable_strings[bam_idx]);
	if (irq < 0) {
		dev_err(&ctx.usb_bam_pdev->dev, "Unable to get IRQ resource\n");
		return irq;
	}

	usb_virt_addr = devm_ioremap(&ctx.usb_bam_pdev->dev, res->start,
		resource_size(res));
	if (!usb_virt_addr) {
		pr_err("%s: ioremap failed\n", __func__);
		return -ENOMEM;
	}

	/* Check if USB3 pipe memory needs to be enabled */
	if (bam_idx == SSUSB_BAM && bam_use_private_mem(bam_idx)) {
		pr_debug("%s: Enabling USB private memory for: %s\n", __func__,
			bam_enable_strings[bam_idx]);

		ram_resource = platform_get_resource_byname(ctx.usb_bam_pdev,
			IORESOURCE_MEM, "qscratch_ram1_reg");
		if (!res) {
			dev_err(&ctx.usb_bam_pdev->dev, "Unable to get qscratch\n");
			ret = -ENODEV;
			goto free_bam_regs;
		}

		ctx.qscratch_ram1_reg = devm_ioremap(&ctx.usb_bam_pdev->dev,
			ram_resource->start,
			resource_size(ram_resource));
		if (!ctx.qscratch_ram1_reg) {
			pr_err("%s: ioremap failed for qscratch\n", __func__);
			ret = -ENOMEM;
			goto free_bam_regs;
		}
	}

	props.phys_addr = res->start;
	props.virt_addr = usb_virt_addr;
	props.virt_size = resource_size(res);
	props.irq = irq;
	props.summing_threshold = USB_THRESHOLD;
	props.event_threshold = USB_THRESHOLD;
	props.num_pipes = pdata->usb_bam_num_pipes;
	props.callback = usb_bam_sps_events;
	props.user = bam_enable_strings[bam_idx];
	props.options = SPS_BAM_OPT_IRQ_WAKEUP;

	/*
	* HSUSB and HSIC Cores don't support RESET ACK signal to BAMs
	* Hence, let BAM to ignore acknowledge from USB while resetting PIPE
	*/
	if (pdata->ignore_core_reset_ack && bam_idx != SSUSB_BAM)
		props.options = SPS_BAM_NO_EXT_P_RST;

	if (pdata->disable_clk_gating)
		props.options |= SPS_BAM_NO_LOCAL_CLK_GATING;

	ret = sps_register_bam_device(&props, &(ctx.h_bam[bam_idx]));
	if (ret < 0) {
		pr_err("%s: register bam error %d\n", __func__, ret);
		ret = -EFAULT;
		goto free_qscratch_reg;
	}

	return 0;

free_qscratch_reg:
	iounmap(ctx.qscratch_ram1_reg);
free_bam_regs:
	iounmap(usb_virt_addr);

	return ret;
}

static int enable_usb_bams(struct platform_device *pdev)
{
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(bam_enable_strings); i++) {
		ret = usb_bam_init(i);
		if (ret) {
			pr_err("failed to init usb bam %s\n",
				bam_enable_strings[i]);
			return ret;
		}
	}

	ctx.usb_bam_sps.sps_pipes = devm_kzalloc(&pdev->dev,
		ctx.max_connections * sizeof(struct sps_pipe *),
		GFP_KERNEL);

	if (!ctx.usb_bam_sps.sps_pipes) {
		pr_err("%s: failed to allocate sps_pipes\n", __func__);
		return -ENOMEM;
	}

	ctx.usb_bam_sps.sps_connections = devm_kzalloc(&pdev->dev,
		ctx.max_connections * sizeof(struct sps_connect),
		GFP_KERNEL);
	if (!ctx.usb_bam_sps.sps_connections) {
		pr_err("%s: failed to allocate sps_connections\n", __func__);
		return -ENOMEM;
	}

	return 0;
}

static ssize_t
usb_bam_show_inactivity_timer(struct device *dev, struct device_attribute *attr,
		    char *buf)
{
	char *buff = buf;
	int i;

	spin_lock(&usb_bam_lock);

	for (i = 0; i < ARRAY_SIZE(bam_enable_strings); i++) {
		buff += snprintf(buff, PAGE_SIZE, "%s: %dms\n",
					bam_enable_strings[i],
					ctx.inactivity_timer_ms[i]);
	}

	spin_unlock(&usb_bam_lock);

	return buff - buf;
}

static ssize_t usb_bam_store_inactivity_timer(struct device *dev,
				     struct device_attribute *attr,
				     const char *buff, size_t count)
{
	char buf[USB_BAM_MAX_STR_LEN];
	char *trimmed_buf, *bam_str, *bam_name, *timer;
	int timer_d;
	enum usb_bam bam;

	if (strnstr(buff, "help", USB_BAM_MAX_STR_LEN)) {
		pr_info("Usage: <bam_name> <ms>,<bam_name> <ms>,...\n");
		pr_info("\tbam_name: [%s, %s, %s]\n",
			bam_enable_strings[SSUSB_BAM],
			bam_enable_strings[HSUSB_BAM],
			bam_enable_strings[HSIC_BAM]);
		pr_info("\tms: time in ms. Use 0 to disable timer\n");
		return count;
	}

	strlcpy(buf, buff, sizeof(buf));
	trimmed_buf = strim(buf);

	while (trimmed_buf) {
		bam_str = strsep(&trimmed_buf, ",");
		if (bam_str) {
			bam_name = strsep(&bam_str, " ");
			bam = get_bam_type_from_core_name(bam_name);

			timer = strsep(&bam_str, " ");
			sscanf(timer, "%d", &timer_d);

			spin_lock(&usb_bam_lock);

			/* Apply new timer setting if bam has running pipes */
			if (ctx.inactivity_timer_ms[bam] != timer_d) {
				ctx.inactivity_timer_ms[bam] = timer_d;
				if (ctx.pipes_enabled_per_bam[bam] > 0 &&
				    !info.in_lpm[bam])
					usb_bam_set_inactivity_timer(bam);
			}

			spin_unlock(&usb_bam_lock);
		}
	}

	return count;
}

static DEVICE_ATTR(inactivity_timer, S_IWUSR | S_IRUSR,
		   usb_bam_show_inactivity_timer,
		   usb_bam_store_inactivity_timer);


static int usb_bam_probe(struct platform_device *pdev)
{
	int ret, i;
	struct msm_usb_bam_platform_data *pdata;

	dev_dbg(&pdev->dev, "usb_bam_probe\n");

	ret = device_create_file(&pdev->dev, &dev_attr_inactivity_timer);
	if (ret) {
		dev_err(&pdev->dev, "failed to create fs node\n");
		return ret;
	}

	ctx.mem_clk = devm_clk_get(&pdev->dev, "mem_clk");
	if (IS_ERR(ctx.mem_clk))
		dev_dbg(&pdev->dev, "failed to get mem_clock\n");

	ctx.mem_iface_clk = devm_clk_get(&pdev->dev, "mem_iface_clk");
	if (IS_ERR(ctx.mem_iface_clk))
		dev_dbg(&pdev->dev, "failed to get mem_iface_clock\n");

	if (pdev->dev.of_node) {
		dev_dbg(&pdev->dev, "device tree enabled\n");
		pdata = usb_bam_dt_to_pdata(pdev);
		if (!pdata)
			return -EINVAL;
		pdev->dev.platform_data = pdata;
	} else if (!pdev->dev.platform_data) {
		dev_err(&pdev->dev, "missing platform_data\n");
		return -ENODEV;
	} else {
		pdata = pdev->dev.platform_data;
		usb_bam_connections = pdata->connections;
		ctx.max_connections = pdata->max_connections;
	}
	ctx.usb_bam_pdev = pdev;

	for (i = 0; i < ctx.max_connections; i++) {
		usb_bam_connections[i].enabled = 0;
		INIT_WORK(&usb_bam_connections[i].event.event_w,
			usb_bam_work);
	}

	for (i = 0; i < MAX_BAMS; i++) {
		ctx.pipes_enabled_per_bam[i] = 0;
		ctx.inactivity_timer_ms[i] = 0;
		ctx.is_bam_inactivity[i] = false;
		init_completion(&info.prod_avail[i]);
		complete(&info.prod_avail[i]);
		init_completion(&info.prod_released[i]);
		complete(&info.prod_released[i]);
		info.cur_prod_state[i] = IPA_RM_RESOURCE_RELEASED;
		info.cur_cons_state[i] = IPA_RM_RESOURCE_RELEASED;
		info.lpm_wait_handshake[i] = false;
	}

	INIT_WORK(&info.resume_work, usb_bam_finish_resume);
	INIT_WORK(&info.suspend_work, usb_bam_start_suspend);
	INIT_WORK(&info.finish_suspend_work, usb_bam_finish_suspend_);
	mutex_init(&info.suspend_resume_mutex);

	spin_lock_init(&usb_bam_peer_handshake_info_lock);
	INIT_WORK(&peer_handshake_info.reset_event.event_w, usb_bam_sm_work);
	init_completion(&ctx.reset_done);
	complete(&ctx.reset_done);

	ctx.usb_bam_wq = alloc_workqueue("usb_bam_wq",
		WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!ctx.usb_bam_wq) {
		pr_err("unable to create workqueue usb_bam_wq\n");
		return -ENOMEM;
	}

	ret = enable_usb_bams(pdev);
	if (ret) {
		destroy_workqueue(ctx.usb_bam_wq);
		return ret;
	}
	spin_lock_init(&usb_bam_ipa_handshake_info_lock);
	usb_bam_ipa_create_resources();
	spin_lock_init(&usb_bam_lock);

	return ret;
}

int usb_bam_get_qdss_idx(u8 num)
{
	return usb_bam_get_connection_idx(ctx.qdss_core_name, QDSS_P_BAM,
		PEER_PERIPHERAL_TO_USB, num);
}
EXPORT_SYMBOL(usb_bam_get_qdss_idx);

void usb_bam_set_qdss_core(const char *qdss_core)
{
	strlcpy(ctx.qdss_core_name, qdss_core, USB_BAM_MAX_STR_LEN);
}

int get_bam2bam_connection_info(u8 idx, u32 *usb_bam_handle,
	u32 *usb_bam_pipe_idx, u32 *peer_pipe_idx,
	struct sps_mem_buffer *desc_fifo, struct sps_mem_buffer *data_fifo)
{
	struct usb_bam_pipe_connect *pipe_connect = &usb_bam_connections[idx];
	enum usb_bam_pipe_dir dir = pipe_connect->dir;
	struct sps_connect *sps_connection =
		&ctx.usb_bam_sps.sps_connections[idx];

	if (dir == USB_TO_PEER_PERIPHERAL) {
		*usb_bam_handle = sps_connection->source;
		*usb_bam_pipe_idx = sps_connection->src_pipe_index;
		*peer_pipe_idx = sps_connection->dest_pipe_index;
	} else {
		*usb_bam_handle = sps_connection->destination;
		*usb_bam_pipe_idx = sps_connection->dest_pipe_index;
		*peer_pipe_idx = sps_connection->src_pipe_index;
	}
	if (data_fifo)
		memcpy(data_fifo, &pipe_connect->data_mem_buf,
		sizeof(struct sps_mem_buffer));
	if (desc_fifo)
		memcpy(desc_fifo, &pipe_connect->desc_mem_buf,
		sizeof(struct sps_mem_buffer));
	return 0;
}
EXPORT_SYMBOL(get_bam2bam_connection_info);


int usb_bam_get_connection_idx(const char *core_name, enum peer_bam client,
	enum usb_bam_pipe_dir dir, u32 num)
{
	u8 i;
	int bam_type;

	bam_type = get_bam_type_from_core_name(core_name);
	if (bam_type < 0)
		return -EINVAL;

	for (i = 0; i < ctx.max_connections; i++)
		if (usb_bam_connections[i].bam_type == bam_type &&
				usb_bam_connections[i].peer_bam == client &&
				usb_bam_connections[i].dir == dir &&
				usb_bam_connections[i].pipe_num == num) {
			pr_debug("%s: index %d was found\n", __func__, i);
			return i;
		}

	pr_err("%s: failed for %s\n", __func__, core_name);
	return -ENODEV;
}
EXPORT_SYMBOL(usb_bam_get_connection_idx);

bool msm_bam_lpm_ok(void)
{
	spin_lock(&usb_bam_ipa_handshake_info_lock);
	if (info.lpm_wait_handshake[HSUSB_BAM] || info.lpm_wait_pipes) {
		info.pending_lpm = 1;
		spin_unlock(&usb_bam_ipa_handshake_info_lock);
		pr_err("%s: Scheduling LPM for later\n", __func__);
		return 0;
	} else {
		info.pending_lpm = 0;
		info.in_lpm[HSUSB_BAM] = true;
		spin_unlock(&usb_bam_ipa_handshake_info_lock);
		pr_err("%s: Going to LPM now\n", __func__);
		return 1;
	}
}
EXPORT_SYMBOL(msm_bam_lpm_ok);

void msm_bam_notify_lpm_resume()
{
	/*
	 * If core was resumed from lpm, just clear the
	 * pending indication, in case it is set.
	*/
	info.pending_lpm = 0;
}
EXPORT_SYMBOL(msm_bam_notify_lpm_resume);

static int usb_bam_remove(struct platform_device *pdev)
{
	destroy_workqueue(ctx.usb_bam_wq);

	return 0;
}

static const struct of_device_id usb_bam_dt_match[] = {
	{ .compatible = "qcom,usb-bam-msm",
	},
	{}
};
MODULE_DEVICE_TABLE(of, usb_bam_dt_match);

static struct platform_driver usb_bam_driver = {
	.probe = usb_bam_probe,
	.remove = usb_bam_remove,
	.driver		= {
		.name	= "usb_bam",
		.of_match_table = usb_bam_dt_match,
	},
};

static int __init init(void)
{
	return platform_driver_register(&usb_bam_driver);
}
module_init(init);

static void __exit cleanup(void)
{
	platform_driver_unregister(&usb_bam_driver);
}
module_exit(cleanup);

MODULE_DESCRIPTION("MSM USB BAM DRIVER");
MODULE_LICENSE("GPL v2");
