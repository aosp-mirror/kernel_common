/* include/linux/usb/msm_hsusb.h
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 * Copyright (c) 2009-2014, The Linux Foundation. All rights reserved.
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

#ifndef __ASM_ARCH_MSM_HSUSB_H
#define __ASM_ARCH_MSM_HSUSB_H

#include <linux/types.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/wakelock.h>
#include <linux/pm_qos.h>
#include <linux/hrtimer.h>
#include <linux/power_supply.h>
#include <linux/cdev.h>
#include <mach/board.h>
#define MSM_PIPE_ID_MASK		(0x1F)
#define MSM_TX_PIPE_ID_OFS		(16)
#define MSM_SPS_MODE			BIT(5)
#define MSM_IS_FINITE_TRANSFER		BIT(6)
#define MSM_PRODUCER			BIT(7)
#define MSM_DISABLE_WB			BIT(8)
#define MSM_ETD_IOC			BIT(9)
#define MSM_INTERNAL_MEM		BIT(10)
#define MSM_VENDOR_ID			BIT(16)

enum usb_bus_vote {
	USB_NO_PERF_VOTE = 0,
	USB_MAX_PERF_VOTE,
	USB_MIN_PERF_VOTE,
};

enum usb_mode_type {
	USB_NONE = 0,
	USB_PERIPHERAL,
	USB_HOST,
	USB_OTG,
};

enum otg_control_type {
	OTG_NO_CONTROL = 0,
	OTG_PHY_CONTROL,
	OTG_PMIC_CONTROL,
	OTG_USER_CONTROL,
};

enum msm_usb_phy_type {
	INVALID_PHY = 0,
	CI_45NM_INTEGRATED_PHY,
	SNPS_28NM_INTEGRATED_PHY,
};

#define IDEV_CHG_MAX	1500
#define IDEV_CHG_MIN	500
#define IUNIT		100

#define IDEV_ACA_CHG_MAX	1500
#define IDEV_ACA_CHG_LIMIT	500

enum usb_chg_state {
	USB_CHG_STATE_UNDEFINED = 0,
	USB_CHG_STATE_WAIT_FOR_DCD,
	USB_CHG_STATE_DCD_DONE,
	USB_CHG_STATE_PRIMARY_DONE,
	USB_CHG_STATE_SECONDARY_DONE,
	USB_CHG_STATE_DETECTED,
};

enum usb_chg_type {
	USB_INVALID_CHARGER = 0,
	USB_SDP_CHARGER,
	USB_DCP_CHARGER,
	USB_CDP_CHARGER,
	USB_ACA_A_CHARGER,
	USB_ACA_B_CHARGER,
	USB_ACA_C_CHARGER,
	USB_ACA_DOCK_CHARGER,
	USB_PROPRIETARY_CHARGER,
	USB_FLOATED_CHARGER,
};

enum usb_vdd_type {
	VDDCX_CORNER = 0,
	VDDCX,
	VDD_TYPE_MAX,
};

enum usb_vdd_value {
	VDD_NONE = 0,
	VDD_MIN,
	VDD_MAX,
	VDD_VAL_MAX,
};


enum usb_ext_chg_status {
	DEFAULT = 1,
	ACTIVE,
	INACTIVE,
};


struct msm_otg_platform_data {
	int *phy_init_seq;
    int *phy_init_seq_no_uart_switch;
	int (*vbus_power)(bool on);
	unsigned power_budget;
	enum usb_mode_type mode;
	enum otg_control_type otg_control;
	enum usb_mode_type default_mode;
	enum msm_usb_phy_type phy_type;
	void (*setup_gpio)(enum usb_otg_state state);
	int pmic_id_irq;
	unsigned int mpm_otgsessvld_int;
	unsigned int mpm_dpshv_int;
	unsigned int mpm_dmshv_int;
	bool mhl_enable;
	bool disable_reset_on_disconnect;
	bool pnoc_errata_fix;
	bool enable_lpm_on_dev_suspend;
	bool core_clk_always_on_workaround;
	bool delay_lpm_on_disconnect;
	bool delay_lpm_hndshk_on_disconnect;
	bool dp_manual_pullup;
	bool enable_sec_phy;
	struct msm_bus_scale_pdata *bus_scale_table;
	const char *mhl_dev_name;
	int log2_itc;
	bool l1_supported;
	bool dpdm_pulldown_added;
	bool enable_ahb2ahb_bypass;
	bool disable_retention_with_vdd_min;
};

#define ENABLE_DP_MANUAL_PULLUP		BIT(0)
#define ENABLE_SECONDARY_PHY		BIT(1)


#define TA_WAIT_VRISE	100	
#define TA_WAIT_VFALL	500	

#ifdef CONFIG_USB_OTG
#define TA_WAIT_BCON	-1	
#else
#define TA_WAIT_BCON	-1
#endif

#define TA_AIDL_BDIS	500	
#define TA_BIDL_ADIS	155	
#define TB_SRP_FAIL	6000	
#define TB_ASE0_BRST	200	

#define TB_SRP_INIT	2000	

#define TA_TST_MAINT	10100	
#define TB_TST_SRP	3000	
#define TB_TST_CONFIG	300


#define A_WAIT_VRISE	0
#define A_WAIT_VFALL	1
#define A_WAIT_BCON	2
#define A_AIDL_BDIS	3
#define A_BIDL_ADIS	4
#define B_SRP_FAIL	5
#define B_ASE0_BRST	6
#define A_TST_MAINT	7
#define B_TST_SRP	8
#define B_TST_CONFIG	9

struct msm_otg {
	struct usb_phy phy;
	struct msm_otg_platform_data *pdata;
	int irq;
	int async_irq;
	struct clk *xo_clk;
	struct clk *clk;
	struct clk *pclk;
	struct clk *core_clk;
	struct clk *sleep_clk;
	long core_clk_rate;
	struct resource *io_res;
	void __iomem *regs;
#define ID		0
#define B_SESS_VLD	1
#define ID_A		2
#define ID_B		3
#define ID_C		4
#define A_BUS_DROP	5
#define A_BUS_REQ	6
#define A_SRP_DET	7
#define A_VBUS_VLD	8
#define B_CONN		9
#define ADP_CHANGE	10
#define POWER_UP	11
#define A_CLR_ERR	12
#define A_BUS_RESUME	13
#define A_BUS_SUSPEND	14
#define A_CONN		15
#define B_BUS_REQ	16
#define MHL	        17
#define B_FALSE_SDP	18
	unsigned long inputs;
	struct work_struct sm_work;
	bool sm_work_pending;
	atomic_t pm_suspended;
	struct notifier_block pm_notify;
	atomic_t in_lpm;
	int async_int;
	unsigned cur_power;
	struct delayed_work chg_work;
	struct delayed_work ac_detect_work;
	struct delayed_work pmic_id_status_work;
	struct delayed_work suspend_work;
	enum usb_chg_state chg_state;
	enum usb_chg_type chg_type;
	unsigned dcd_time;
	struct wake_lock wlock;
	struct wake_lock cable_detect_wlock;
	struct notifier_block usbdev_nb;
	unsigned mA_port;
	struct timer_list id_timer;
	unsigned long caps;
	struct msm_xo_voter *xo_handle;
	uint32_t bus_perf_client;
	bool mhl_enabled;
	bool host_bus_suspend;
	struct timer_list chg_check_timer;
#define ALLOW_PHY_POWER_COLLAPSE	BIT(0)
#define ALLOW_PHY_RETENTION		BIT(1)
#define ALLOW_LPM_ON_DEV_SUSPEND	BIT(2)
#define ALLOW_PHY_REGULATORS_LPM	BIT(3)
#define ALLOW_HOST_PHY_RETENTION	BIT(4)
#define ALLOW_VDD_MIN_WITH_RETENTION_DISABLED BIT(5)
	unsigned long lpm_flags;
#define PHY_PWR_COLLAPSED		BIT(0)
#define PHY_RETENTIONED			BIT(1)
#define XO_SHUTDOWN			BIT(2)
#define CLOCKS_DOWN			BIT(3)
#define PHY_REGULATORS_LPM	BIT(4)
	struct work_struct notifier_work;
	enum usb_connect_type connect_type;
	int connect_type_ready;
	struct workqueue_struct *usb_wq;
	int reset_counter;
	unsigned long b_last_se0_sess;
	unsigned long tmouts;
	u8 active_tmout;
	struct hrtimer timer;
	enum usb_vdd_type vdd_type;
	struct power_supply usb_psy;
	unsigned int online;
	unsigned int host_mode;
	unsigned int voltage_max;
	unsigned int current_max;
	unsigned int usbin_health;

	dev_t ext_chg_dev;
	struct cdev ext_chg_cdev;
	struct class *ext_chg_class;
	struct device *ext_chg_device;
	bool ext_chg_opened;
	enum usb_ext_chg_status ext_chg_active;
	struct completion ext_chg_wait;
	struct qpnp_vadc_chip *vadc_chip;
	struct qpnp_vadc_chip	*vadc_dev;
	int chg_check_count;
	int ui_enabled;
	bool pm_done;
};

struct ci13xxx_platform_data {
	u8 usb_core_id;
	int log2_itc;
	void *prv_data;
	bool l1_supported;
	bool enable_ahb2ahb_bypass;
};

struct msm_hsic_host_platform_data {
	unsigned strobe;
	unsigned data;
	bool ignore_cal_pad_config;
	bool phy_sof_workaround;
	bool phy_susp_sof_workaround;
	u32 reset_delay;
	int strobe_pad_offset;
	int data_pad_offset;

	struct msm_bus_scale_pdata *bus_scale_table;
	unsigned log2_irq_thresh;

	
	unsigned resume_gpio;

	
	u32 swfi_latency;

	
	u32 standalone_latency;
	bool pool_64_bit_align;
	bool enable_hbm;
	bool disable_park_mode;
	bool consider_ipa_handshake;
	bool ahb_async_bridge_bypass;
	bool disable_cerr;
};

struct msm_usb_host_platform_data {
	unsigned int power_budget;
	int pmic_gpio_dp_irq;
	unsigned int dock_connect_irq;
	bool use_sec_phy;
	bool no_selective_suspend;
	int resume_gpio;
	bool is_uicc;
};

struct msm_hsic_peripheral_platform_data {
	bool core_clk_always_on_workaround;
};

struct usb_ext_notification {
	int (*notify)(void *, int, void (*)(void *, int online), void *);
	void *ctxt;
};
#ifdef CONFIG_USB_BAM
bool msm_bam_lpm_ok(void);
void msm_bam_notify_lpm_resume(void);
void msm_bam_set_hsic_host_dev(struct device *dev);
void msm_bam_wait_for_hsic_prod_granted(void);
bool msm_bam_hsic_lpm_ok(void);
void msm_bam_hsic_notify_on_resume(void);
#else
static inline bool msm_bam_lpm_ok(void) { return true; }
static inline void msm_bam_notify_lpm_resume(void) {}
static inline void msm_bam_set_hsic_host_dev(struct device *dev) {}
static inline void msm_bam_wait_for_hsic_prod_granted(void) {}
static inline bool msm_bam_hsic_lpm_ok(void) { return true; }
static inline void msm_bam_hsic_notify_on_resume(void) {}
#endif
#ifdef CONFIG_USB_CI13XXX_MSM
void msm_hw_bam_disable(bool bam_disable);
#else
static inline void msm_hw_bam_disable(bool bam_disable)
{
}
#endif

#ifdef CONFIG_USB_DWC3_MSM
int msm_ep_config(struct usb_ep *ep);
int msm_ep_unconfig(struct usb_ep *ep);
void dwc3_tx_fifo_resize_request(struct usb_ep *ep, bool qdss_enable);
int msm_data_fifo_config(struct usb_ep *ep, u32 addr, u32 size,
	u8 dst_pipe_idx);

void msm_dwc3_restart_usb_session(struct usb_gadget *gadget);

int msm_register_usb_ext_notification(struct usb_ext_notification *info);
#else
static inline int msm_data_fifo_config(struct usb_ep *ep, u32 addr, u32 size,
	u8 dst_pipe_idx)
{
	return -ENODEV;
}

static inline int msm_ep_config(struct usb_ep *ep)
{
	return -ENODEV;
}

static inline int msm_ep_unconfig(struct usb_ep *ep)
{
	return -ENODEV;
}

static inline void dwc3_tx_fifo_resize_request(
					struct usb_ep *ep, bool qdss_enable)
{
	return;
}

static inline void msm_dwc3_restart_usb_session(struct usb_gadget *gadget)
{
	return;
}

static inline int msm_register_usb_ext_notification(
					struct usb_ext_notification *info)
{
	return -ENODEV;
}
#endif
#endif
