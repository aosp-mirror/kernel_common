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

#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/wcnss_wlan.h>
#include <linux/platform_data/qcom_wcnss_device.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/ratelimit.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/suspend.h>
#include <linux/rwsem.h>
#include <linux/mfd/pm8xxx/misc.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/pm_qos.h>

#include <mach/board.h>
#include <mach/msm_smd.h>
#include <mach/msm_iomap.h>
#include <mach/subsystem_restart.h>
#include <mach/subsystem_notif.h>

#include <mach/devices_dtb.h>
#include <mach/devices_cmdline.h>

#ifdef CONFIG_WCNSS_MEM_PRE_ALLOC
#include "wcnss_prealloc.h"
#endif

#define DEVICE "wcnss_wlan"
#define CTRL_DEVICE "wcnss_ctrl"
#define VERSION "1.01"
#define WCNSS_PIL_DEVICE "wcnss"

#define WCNSS_DISABLE_PC_LATENCY	100
#define WCNSS_ENABLE_PC_LATENCY	PM_QOS_DEFAULT_VALUE
#define WCNSS_PM_QOS_TIMEOUT	15000

#define WCNSS_CONFIG_UNSPECIFIED (-1)
#define UINT32_MAX (0xFFFFFFFFU)

static int has_48mhz_xo = WCNSS_CONFIG_UNSPECIFIED;
module_param(has_48mhz_xo, int, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(has_48mhz_xo, "Is an external 48 MHz XO present");

static int has_calibrated_data = WCNSS_CONFIG_UNSPECIFIED;
module_param(has_calibrated_data, int, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(has_calibrated_data, "whether calibrated data file available");

static int has_autodetect_xo = WCNSS_CONFIG_UNSPECIFIED;
module_param(has_autodetect_xo, int, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(has_autodetect_xo, "Perform auto detect to configure IRIS XO");

static int do_not_cancel_vote = WCNSS_CONFIG_UNSPECIFIED;
module_param(do_not_cancel_vote, int, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(do_not_cancel_vote, "Do not cancel votes for wcnss");

static DEFINE_SPINLOCK(reg_spinlock);


#define MSM_RIVA_PHYS			0x03204000
#define MSM_PRONTO_PHYS			0xfb21b000

#define RIVA_SPARE_OFFSET		0x0b4
#define RIVA_SUSPEND_BIT		BIT(24)

#define MSM_RIVA_CCU_BASE			0x03200800

#define CCU_RIVA_INVALID_ADDR_OFFSET		0x100
#define CCU_RIVA_LAST_ADDR0_OFFSET		0x104
#define CCU_RIVA_LAST_ADDR1_OFFSET		0x108
#define CCU_RIVA_LAST_ADDR2_OFFSET		0x10c

#define PRONTO_PMU_SPARE_OFFSET       0x1088

#define PRONTO_PMU_COM_GDSCR_OFFSET       0x0024
#define PRONTO_PMU_COM_GDSCR_SW_COLLAPSE  BIT(0)
#define PRONTO_PMU_COM_GDSCR_HW_CTRL      BIT(1)

#define PRONTO_PMU_WLAN_BCR_OFFSET         0x0050
#define PRONTO_PMU_WLAN_BCR_BLK_ARES       BIT(0)

#define PRONTO_PMU_WLAN_GDSCR_OFFSET       0x0054
#define PRONTO_PMU_WLAN_GDSCR_SW_COLLAPSE  BIT(0)


#define PRONTO_PMU_CBCR_OFFSET        0x0008
#define PRONTO_PMU_CBCR_CLK_EN        BIT(0)

#define PRONTO_PMU_COM_CPU_CBCR_OFFSET     0x0030
#define PRONTO_PMU_COM_AHB_CBCR_OFFSET     0x0034

#define PRONTO_PMU_WLAN_AHB_CBCR_OFFSET    0x0074
#define PRONTO_PMU_WLAN_AHB_CBCR_CLK_EN    BIT(0)
#define PRONTO_PMU_WLAN_AHB_CBCR_CLK_OFF   BIT(31)

#define PRONTO_PMU_CPU_AHB_CMD_RCGR_OFFSET  0x0120
#define PRONTO_PMU_CPU_AHB_CMD_RCGR_ROOT_EN BIT(1)

#define PRONTO_PMU_CFG_OFFSET              0x1004
#define PRONTO_PMU_COM_CSR_OFFSET          0x1040
#define PRONTO_PMU_SOFT_RESET_OFFSET       0x104C

#define MSM_PRONTO_A2XB_BASE		0xfb100400
#define A2XB_CFG_OFFSET				0x00
#define A2XB_INT_SRC_OFFSET			0x0c
#define A2XB_TSTBUS_CTRL_OFFSET		0x14
#define A2XB_TSTBUS_OFFSET			0x18
#define A2XB_ERR_INFO_OFFSET		0x1c
#define A2XB_FIFO_FILL_OFFSET		0x07
#define A2XB_READ_FIFO_FILL_MASK		0x3F
#define A2XB_CMD_FIFO_FILL_MASK			0x0F
#define A2XB_WRITE_FIFO_FILL_MASK		0x1F
#define A2XB_FIFO_EMPTY			0x2
#define A2XB_FIFO_COUNTER			0xA

#define WCNSS_TSTBUS_CTRL_EN		BIT(0)
#define WCNSS_TSTBUS_CTRL_AXIM		(0x02 << 1)
#define WCNSS_TSTBUS_CTRL_CMDFIFO	(0x03 << 1)
#define WCNSS_TSTBUS_CTRL_WRFIFO	(0x04 << 1)
#define WCNSS_TSTBUS_CTRL_RDFIFO	(0x05 << 1)
#define WCNSS_TSTBUS_CTRL_CTRL		(0x07 << 1)
#define WCNSS_TSTBUS_CTRL_AXIM_CFG0	(0x00 << 8)
#define WCNSS_TSTBUS_CTRL_AXIM_CFG1	(0x01 << 8)
#define WCNSS_TSTBUS_CTRL_CTRL_CFG0	(0x00 << 28)
#define WCNSS_TSTBUS_CTRL_CTRL_CFG1	(0x01 << 28)

#define MSM_PRONTO_CCPU_BASE			0xfb205050
#define CCU_PRONTO_INVALID_ADDR_OFFSET		0x08
#define CCU_PRONTO_LAST_ADDR0_OFFSET		0x0c
#define CCU_PRONTO_LAST_ADDR1_OFFSET		0x10
#define CCU_PRONTO_LAST_ADDR2_OFFSET		0x14

#define MSM_PRONTO_SAW2_BASE			0xfb219000
#define PRONTO_SAW2_SPM_STS_OFFSET		0x0c

#define MSM_PRONTO_PLL_BASE				0xfb21b1c0
#define PRONTO_PLL_STATUS_OFFSET		0x1c
#define MSM_PRONTO_TXP_STATUS           0xfb08040c

#define MSM_PRONTO_MCU_BASE			0xfb080c00
#define MCU_CBR_CCAHB_ERR_OFFSET		0x380
#define MCU_CBR_CAHB_ERR_OFFSET			0x384
#define MCU_CBR_CCAHB_TIMEOUT_OFFSET		0x388
#define MCU_CBR_CAHB_TIMEOUT_OFFSET		0x38c
#define MCU_DBR_CDAHB_ERR_OFFSET		0x390
#define MCU_DBR_DAHB_ERR_OFFSET			0x394
#define MCU_DBR_CDAHB_TIMEOUT_OFFSET		0x398
#define MCU_DBR_DAHB_TIMEOUT_OFFSET		0x39c
#define MCU_FDBR_CDAHB_ERR_OFFSET		0x3a0
#define MCU_FDBR_FDAHB_ERR_OFFSET		0x3a4
#define MCU_FDBR_CDAHB_TIMEOUT_OFFSET		0x3a8
#define MCU_FDBR_FDAHB_TIMEOUT_OFFSET		0x3ac

#define MSM_PRONTO_MCU_BASE			0xfb080c00
#define MCU_CBR_CCAHB_ERR_OFFSET		0x380
#define MCU_CBR_CAHB_ERR_OFFSET			0x384
#define MCU_CBR_CCAHB_TIMEOUT_OFFSET		0x388
#define MCU_CBR_CAHB_TIMEOUT_OFFSET		0x38c
#define MCU_DBR_CDAHB_ERR_OFFSET		0x390
#define MCU_DBR_DAHB_ERR_OFFSET			0x394
#define MCU_DBR_CDAHB_TIMEOUT_OFFSET		0x398
#define MCU_DBR_DAHB_TIMEOUT_OFFSET		0x39c
#define MCU_FDBR_CDAHB_ERR_OFFSET		0x3a0
#define MCU_FDBR_FDAHB_ERR_OFFSET		0x3a4
#define MCU_FDBR_CDAHB_TIMEOUT_OFFSET		0x3a8
#define MCU_FDBR_FDAHB_TIMEOUT_OFFSET		0x3ac

#define MSM_PRONTO_MCU_BASE			0xfb080c00
#define MCU_APB2PHY_STATUS_OFFSET		0xec
#define MCU_CBR_CCAHB_ERR_OFFSET		0x380
#define MCU_CBR_CAHB_ERR_OFFSET			0x384
#define MCU_CBR_CCAHB_TIMEOUT_OFFSET		0x388
#define MCU_CBR_CAHB_TIMEOUT_OFFSET		0x38c
#define MCU_DBR_CDAHB_ERR_OFFSET		0x390
#define MCU_DBR_DAHB_ERR_OFFSET			0x394
#define MCU_DBR_CDAHB_TIMEOUT_OFFSET		0x398
#define MCU_DBR_DAHB_TIMEOUT_OFFSET		0x39c
#define MCU_FDBR_CDAHB_ERR_OFFSET		0x3a0
#define MCU_FDBR_FDAHB_ERR_OFFSET		0x3a4
#define MCU_FDBR_CDAHB_TIMEOUT_OFFSET		0x3a8
#define MCU_FDBR_FDAHB_TIMEOUT_OFFSET		0x3ac

#define MSM_PRONTO_TXP_STATUS           0xfb08040c
#define MSM_PRONTO_TXP_PHY_ABORT        0xfb080488
#define MSM_PRONTO_BRDG_ERR_SRC         0xfb080fb0

#define MSM_PRONTO_ALARMS_TXCTL         0xfb0120a8
#define MSM_PRONTO_ALARMS_TACTL         0xfb012448

#define WCNSS_DEF_WLAN_RX_BUFF_COUNT		1024
#define WCNSS_VBATT_THRESHOLD		3500000
#define WCNSS_VBATT_GUARD		20000
#define WCNSS_VBATT_HIGH		3700000
#define WCNSS_VBATT_LOW			3300000

#define WCNSS_CTRL_CHANNEL			"WCNSS_CTRL"
#define WCNSS_MAX_FRAME_SIZE		(4*1024)
#define WCNSS_VERSION_LEN			30
#define WCNSS_MAX_BUILD_VER_LEN		256
#define WCNSS_MAX_CMD_LEN		(128)
#define WCNSS_MIN_CMD_LEN		(3)
#define WCNSS_MIN_SERIAL_LEN		(6)

#define WCNSS_USR_CTRL_MSG_START  0x00000000
#define WCNSS_USR_SERIAL_NUM      (WCNSS_USR_CTRL_MSG_START + 1)
#define WCNSS_USR_HAS_CAL_DATA    (WCNSS_USR_CTRL_MSG_START + 2)
#define WCNSS_USR_WLAN_MAC_ADDR   (WCNSS_USR_CTRL_MSG_START + 3)

#define MAC_ADDRESS_STR "%02x:%02x:%02x:%02x:%02x:%02x"

#define WCNSS_CTRL_MSG_START	0x01000000
#define	WCNSS_VERSION_REQ             (WCNSS_CTRL_MSG_START + 0)
#define	WCNSS_VERSION_RSP             (WCNSS_CTRL_MSG_START + 1)
#define	WCNSS_NVBIN_DNLD_REQ          (WCNSS_CTRL_MSG_START + 2)
#define	WCNSS_NVBIN_DNLD_RSP          (WCNSS_CTRL_MSG_START + 3)
#define	WCNSS_CALDATA_UPLD_REQ        (WCNSS_CTRL_MSG_START + 4)
#define	WCNSS_CALDATA_UPLD_RSP        (WCNSS_CTRL_MSG_START + 5)
#define	WCNSS_CALDATA_DNLD_REQ        (WCNSS_CTRL_MSG_START + 6)
#define	WCNSS_CALDATA_DNLD_RSP        (WCNSS_CTRL_MSG_START + 7)
#define	WCNSS_VBATT_LEVEL_IND         (WCNSS_CTRL_MSG_START + 8)
#define	WCNSS_BUILD_VER_REQ           (WCNSS_CTRL_MSG_START + 9)
#define	WCNSS_BUILD_VER_RSP           (WCNSS_CTRL_MSG_START + 10)
#define	WCNSS_PM_CONFIG_REQ           (WCNSS_CTRL_MSG_START + 11)

#define WCNSS_MAX_CH_NUM			45
#define WCNSS_MAX_PIL_RETRY			2

#define VALID_VERSION(version) \
	((strncmp(version, "INVALID", WCNSS_VERSION_LEN)) ? 1 : 0)

#define FW_CALDATA_CAPABLE() \
	((penv->fw_major >= 1) && (penv->fw_minor >= 5) ? 1 : 0)

struct smd_msg_hdr {
	unsigned int msg_type;
	unsigned int msg_len;
};

struct wcnss_version {
	struct smd_msg_hdr hdr;
	unsigned char  major;
	unsigned char  minor;
	unsigned char  version;
	unsigned char  revision;
};

struct wcnss_pmic_dump {
	char reg_name[10];
	u16 reg_addr;
};

static struct wcnss_pmic_dump wcnss_pmic_reg_dump[] = {
	{"S2", 0x1D8},
	{"L4", 0xB4},
	{"L10", 0xC0},
	{"LVS2", 0x62},
	{"S4", 0x1E8},
	{"LVS7", 0x06C},
	{"LVS1", 0x060},
};

static int wcnss_notif_cb(struct notifier_block *this, unsigned long code,
				void *ss_handle);

static struct notifier_block wnb = {
	.notifier_call = wcnss_notif_cb,
};

#define NVBIN_FILE "wlan/prima/WCNSS_qcom_wlan_nv.bin"
#define NVBIN_FILE_EPCOS "wlan/prima/WCNSS_qcom_wlan_nv_EPCOS.bin"

#define NV_FRAGMENT_SIZE  3072
#define MAX_CALIBRATED_DATA_SIZE  (64*1024)
#define LAST_FRAGMENT        (1 << 0)
#define MESSAGE_TO_FOLLOW    (1 << 1)
#define CAN_RECEIVE_CALDATA  (1 << 15)
#define WCNSS_RESP_SUCCESS   1
#define WCNSS_RESP_FAIL      0


#define TOTALFRAGMENTS(x) (((x % NV_FRAGMENT_SIZE) == 0) ? \
	(x / NV_FRAGMENT_SIZE) : ((x / NV_FRAGMENT_SIZE) + 1))

struct nvbin_dnld_req_params {
	unsigned short frag_number;

	unsigned short msg_flags;

	
	unsigned int nvbin_buffer_size;

};


struct nvbin_dnld_req_msg {
	struct smd_msg_hdr hdr;
	struct nvbin_dnld_req_params dnld_req_params;
};

struct cal_data_params {

	unsigned int total_size;
	unsigned short frag_number;
	unsigned short msg_flags;
	unsigned int frag_size;
};

struct cal_data_msg {
	struct smd_msg_hdr hdr;
	struct cal_data_params cal_params;
};

struct vbatt_level {
	u32 curr_volt;
	u32 threshold;
};

struct vbatt_message {
	struct smd_msg_hdr hdr;
	struct vbatt_level vbatt;
};

static struct {
	struct platform_device *pdev;
	void		*pil;
	struct resource	*mmio_res;
	struct resource	*tx_irq_res;
	struct resource	*rx_irq_res;
	struct resource	*gpios_5wire;
	const struct dev_pm_ops *pm_ops;
	int		triggered;
	int		smd_channel_ready;
	u32		wlan_rx_buff_count;
	smd_channel_t	*smd_ch;
	unsigned char	wcnss_version[WCNSS_VERSION_LEN];
	unsigned char   fw_major;
	unsigned char   fw_minor;
	unsigned int	serial_number;
	int		thermal_mitigation;
	enum wcnss_hw_type	wcnss_hw_type;
	void		(*tm_notify)(struct device *, int);
	struct wcnss_wlan_config wlan_config;
	struct delayed_work wcnss_work;
	struct delayed_work vbatt_work;
	struct work_struct wcnssctrl_version_work;
	struct work_struct wcnss_pm_config_work;
	struct work_struct wcnssctrl_nvbin_dnld_work;
	struct work_struct wcnssctrl_rx_work;
	struct wake_lock wcnss_wake_lock;
	void __iomem *msm_wcnss_base;
	void __iomem *riva_ccu_base;
	void __iomem *pronto_a2xb_base;
	void __iomem *pronto_ccpu_base;
	void __iomem *pronto_saw2_base;
	void __iomem *pronto_pll_base;
	void __iomem *pronto_mcu_base;
	void __iomem *wlan_tx_status;
	void __iomem *wlan_tx_phy_aborts;
	void __iomem *wlan_brdg_err_source;
	void __iomem *alarms_txctl;
	void __iomem *alarms_tactl;
	void __iomem *fiq_reg;
	int	nv_downloaded;
	unsigned char *fw_cal_data;
	unsigned char *user_cal_data;
	int	fw_cal_rcvd;
	int	fw_cal_exp_frag;
	int	fw_cal_available;
	int	user_cal_read;
	int	user_cal_available;
	u32	user_cal_rcvd;
	int	user_cal_exp_size;
	int	device_opened;
	int	iris_xo_mode_set;
	int	fw_vbatt_state;
	int	ctrl_device_opened;
	char	wlan_nv_macAddr[WLAN_MAC_ADDR_SIZE];
	struct mutex dev_lock;
	struct mutex ctrl_lock;
	wait_queue_head_t read_wait;
	struct qpnp_adc_tm_btm_param vbat_monitor_params;
	struct qpnp_adc_tm_chip *adc_tm_dev;
	struct mutex vbat_monitor_mutex;
	u16 unsafe_ch_count;
	u16 unsafe_ch_list[WCNSS_MAX_CH_NUM];
	void *wcnss_notif_hdle;
	u8 is_shutdown;
	struct pm_qos_request wcnss_pm_qos_request;
	int pc_disabled;
	struct delayed_work wcnss_pm_qos_del_req;
} *penv = NULL;

static ssize_t wcnss_wlan_macaddr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char macAddr[WLAN_MAC_ADDR_SIZE];

	if (!penv)
		return -ENODEV;

	pr_debug("%s: Receive MAC Addr From user space: %s\n", __func__, buf);

	if (WLAN_MAC_ADDR_SIZE != sscanf(buf, MAC_ADDRESS_STR,
		 (int *)&macAddr[0], (int *)&macAddr[1],
		 (int *)&macAddr[2], (int *)&macAddr[3],
		 (int *)&macAddr[4], (int *)&macAddr[5])) {

		pr_err("%s: Failed to Copy MAC\n", __func__);
		return -EINVAL;
	}

	memcpy(penv->wlan_nv_macAddr, macAddr, sizeof(penv->wlan_nv_macAddr));

	pr_info("%s: Write MAC Addr:" MAC_ADDRESS_STR "\n", __func__,
		penv->wlan_nv_macAddr[0], penv->wlan_nv_macAddr[1],
		penv->wlan_nv_macAddr[2], penv->wlan_nv_macAddr[3],
		penv->wlan_nv_macAddr[4], penv->wlan_nv_macAddr[5]);

	return count;
}

static ssize_t wcnss_wlan_macaddr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (!penv)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, MAC_ADDRESS_STR,
		penv->wlan_nv_macAddr[0], penv->wlan_nv_macAddr[1],
		penv->wlan_nv_macAddr[2], penv->wlan_nv_macAddr[3],
		penv->wlan_nv_macAddr[4], penv->wlan_nv_macAddr[5]);
}

static DEVICE_ATTR(wcnss_mac_addr, S_IRUSR | S_IWUSR,
	wcnss_wlan_macaddr_show, wcnss_wlan_macaddr_store);

static ssize_t wcnss_serial_number_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (!penv)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%08X\n", penv->serial_number);
}

static ssize_t wcnss_serial_number_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int value;

	if (!penv)
		return -ENODEV;

	if (sscanf(buf, "%08X", &value) != 1)
		return -EINVAL;

	penv->serial_number = value;
	return count;
}

static DEVICE_ATTR(serial_number, S_IRUSR | S_IWUSR,
	wcnss_serial_number_show, wcnss_serial_number_store);


static ssize_t wcnss_thermal_mitigation_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (!penv)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%u\n", penv->thermal_mitigation);
}

static ssize_t wcnss_thermal_mitigation_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	if (!penv)
		return -ENODEV;

	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;
	penv->thermal_mitigation = value;
	if (penv->tm_notify)
		(penv->tm_notify)(dev, value);
	return count;
}

static DEVICE_ATTR(thermal_mitigation, S_IRUSR | S_IWUSR,
	wcnss_thermal_mitigation_show, wcnss_thermal_mitigation_store);


static ssize_t wcnss_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (!penv)
		return -ENODEV;

	return scnprintf(buf, PAGE_SIZE, "%s", penv->wcnss_version);
}

static DEVICE_ATTR(wcnss_version, S_IRUSR,
		wcnss_version_show, NULL);

void wcnss_riva_dump_pmic_regs(void)
{
	int i, rc;
	u8  val;

	for (i = 0; i < ARRAY_SIZE(wcnss_pmic_reg_dump); i++) {
		val = 0;
		rc = pm8xxx_read_register(wcnss_pmic_reg_dump[i].reg_addr,
				&val);
		if (rc)
			pr_err("PMIC READ: Failed to read addr = %d\n",
					wcnss_pmic_reg_dump[i].reg_addr);
		else
			pr_info_ratelimited("PMIC READ: %s addr = %x, value = %x\n",
				wcnss_pmic_reg_dump[i].reg_name,
				wcnss_pmic_reg_dump[i].reg_addr, val);
	}
}

void wcnss_riva_log_debug_regs(void)
{
	void __iomem *ccu_reg;
	u32 reg = 0;

	ccu_reg = penv->riva_ccu_base + CCU_RIVA_INVALID_ADDR_OFFSET;
	reg = readl_relaxed(ccu_reg);
	pr_info_ratelimited("%s: CCU_CCPU_INVALID_ADDR %08x\n", __func__, reg);

	ccu_reg = penv->riva_ccu_base + CCU_RIVA_LAST_ADDR0_OFFSET;
	reg = readl_relaxed(ccu_reg);
	pr_info_ratelimited("%s: CCU_CCPU_LAST_ADDR0 %08x\n", __func__, reg);

	ccu_reg = penv->riva_ccu_base + CCU_RIVA_LAST_ADDR1_OFFSET;
	reg = readl_relaxed(ccu_reg);
	pr_info_ratelimited("%s: CCU_CCPU_LAST_ADDR1 %08x\n", __func__, reg);

	ccu_reg = penv->riva_ccu_base + CCU_RIVA_LAST_ADDR2_OFFSET;
	reg = readl_relaxed(ccu_reg);
	pr_info_ratelimited("%s: CCU_CCPU_LAST_ADDR2 %08x\n", __func__, reg);
	wcnss_riva_dump_pmic_regs();

}
EXPORT_SYMBOL(wcnss_riva_log_debug_regs);

void wcnss_pronto_is_a2xb_bus_stall(void *tst_addr, u32 fifo_mask, char *type)
{
	u32 iter = 0, reg = 0;
	u32 axi_fifo_count = 0, axi_fifo_count_last = 0;

	reg = readl_relaxed(tst_addr);
	axi_fifo_count = (reg >> A2XB_FIFO_FILL_OFFSET) & fifo_mask;
	while ((++iter < A2XB_FIFO_COUNTER) && axi_fifo_count) {
		axi_fifo_count_last = axi_fifo_count;
		reg = readl_relaxed(tst_addr);
		axi_fifo_count = (reg >> A2XB_FIFO_FILL_OFFSET) & fifo_mask;
		if (axi_fifo_count < axi_fifo_count_last)
			break;
	}

	if (iter == A2XB_FIFO_COUNTER) {
		pr_err("%s data FIFO testbus possibly stalled reg%08x\n",
				type, reg);
	} else {
		pr_err("%s data FIFO tstbus not stalled reg%08x\n",
				type, reg);
	}
}

void wcnss_pronto_log_debug_regs(void)
{
	void __iomem *reg_addr, *tst_addr, *tst_ctrl_addr;
	u32 reg = 0, reg2 = 0, reg3 = 0, reg4 = 0;


	reg_addr = penv->msm_wcnss_base + PRONTO_PMU_SPARE_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("PRONTO_PMU_SPARE %08x\n", reg);

	reg_addr = penv->msm_wcnss_base + PRONTO_PMU_COM_CPU_CBCR_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("PRONTO_PMU_COM_CPU_CBCR %08x\n", reg);

	reg_addr = penv->msm_wcnss_base + PRONTO_PMU_COM_AHB_CBCR_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("PRONTO_PMU_COM_AHB_CBCR %08x\n", reg);

	reg_addr = penv->msm_wcnss_base + PRONTO_PMU_CFG_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("PRONTO_PMU_CFG %08x\n", reg);

	reg_addr = penv->msm_wcnss_base + PRONTO_PMU_COM_CSR_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("PRONTO_PMU_COM_CSR %08x\n", reg);

	reg_addr = penv->msm_wcnss_base + PRONTO_PMU_SOFT_RESET_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("PRONTO_PMU_SOFT_RESET %08x\n", reg);

	reg_addr = penv->pronto_saw2_base + PRONTO_SAW2_SPM_STS_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("PRONTO_SAW2_SPM_STS %08x\n", reg);

	reg_addr = penv->pronto_pll_base + PRONTO_PLL_STATUS_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("PRONTO_PLL_STATUS %08x\n", reg);

	reg_addr = penv->msm_wcnss_base + PRONTO_PMU_CPU_AHB_CMD_RCGR_OFFSET;
	reg4 = readl_relaxed(reg_addr);
	pr_err("PMU_CPU_CMD_RCGR %08x\n", reg4);

	reg_addr = penv->msm_wcnss_base + PRONTO_PMU_COM_GDSCR_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("PRONTO_PMU_COM_GDSCR %08x\n", reg);
	reg >>= 31;

	if (!reg) {
		pr_err("Cannot log, Pronto common SS is power collapsed\n");
		return;
	}
	reg &= ~(PRONTO_PMU_COM_GDSCR_SW_COLLAPSE
			| PRONTO_PMU_COM_GDSCR_HW_CTRL);
	writel_relaxed(reg, reg_addr);

	reg_addr = penv->msm_wcnss_base + PRONTO_PMU_CBCR_OFFSET;
	reg = readl_relaxed(reg_addr);
	reg |= PRONTO_PMU_CBCR_CLK_EN;
	writel_relaxed(reg, reg_addr);

	reg_addr = penv->pronto_a2xb_base + A2XB_CFG_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("A2XB_CFG_OFFSET %08x\n", reg);

	reg_addr = penv->pronto_a2xb_base + A2XB_INT_SRC_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("A2XB_INT_SRC_OFFSET %08x\n", reg);

	reg_addr = penv->pronto_a2xb_base + A2XB_ERR_INFO_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("A2XB_ERR_INFO_OFFSET %08x\n", reg);

	reg_addr = penv->pronto_ccpu_base + CCU_PRONTO_INVALID_ADDR_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("CCU_CCPU_INVALID_ADDR %08x\n", reg);

	reg_addr = penv->pronto_ccpu_base + CCU_PRONTO_LAST_ADDR0_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("CCU_CCPU_LAST_ADDR0 %08x\n", reg);

	reg_addr = penv->pronto_ccpu_base + CCU_PRONTO_LAST_ADDR1_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("CCU_CCPU_LAST_ADDR1 %08x\n", reg);

	reg_addr = penv->pronto_ccpu_base + CCU_PRONTO_LAST_ADDR2_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("CCU_CCPU_LAST_ADDR2 %08x\n", reg);

	tst_addr = penv->pronto_a2xb_base + A2XB_TSTBUS_OFFSET;
	tst_ctrl_addr = penv->pronto_a2xb_base + A2XB_TSTBUS_CTRL_OFFSET;

	
	reg = 0;
	reg = reg | WCNSS_TSTBUS_CTRL_EN | WCNSS_TSTBUS_CTRL_RDFIFO;
	writel_relaxed(reg, tst_ctrl_addr);
	reg = readl_relaxed(tst_addr);
	if (!(reg & A2XB_FIFO_EMPTY)) {
		wcnss_pronto_is_a2xb_bus_stall(tst_addr,
			       A2XB_READ_FIFO_FILL_MASK, "Read");
	} else {
		pr_err("Read data FIFO testbus %08x\n", reg);
	}
	
	reg = 0;
	reg = reg | WCNSS_TSTBUS_CTRL_EN | WCNSS_TSTBUS_CTRL_CMDFIFO;
	writel_relaxed(reg, tst_ctrl_addr);
	reg = readl_relaxed(tst_addr);

	if (!(reg & A2XB_FIFO_EMPTY)) {
		wcnss_pronto_is_a2xb_bus_stall(tst_addr,
			       A2XB_CMD_FIFO_FILL_MASK, "Cmd");
	} else {
		pr_err("Command FIFO testbus %08x\n", reg);
	}


	
	reg = 0;
	reg = reg | WCNSS_TSTBUS_CTRL_EN | WCNSS_TSTBUS_CTRL_WRFIFO;
	writel_relaxed(reg, tst_ctrl_addr);
	reg = readl_relaxed(tst_addr);
	if (!(reg & A2XB_FIFO_EMPTY)) {
		wcnss_pronto_is_a2xb_bus_stall(tst_addr,
				A2XB_WRITE_FIFO_FILL_MASK, "Write");
	} else {
		pr_err("Write data FIFO testbus %08x\n", reg);
	}

	
	reg = 0;
	reg = reg | WCNSS_TSTBUS_CTRL_EN | WCNSS_TSTBUS_CTRL_AXIM |
				WCNSS_TSTBUS_CTRL_AXIM_CFG0;
	writel_relaxed(reg, tst_ctrl_addr);
	reg = readl_relaxed(tst_addr);
	pr_err("AXIM SEL CFG0 testbus %08x\n", reg);

	
	reg = 0;
	reg = reg | WCNSS_TSTBUS_CTRL_EN | WCNSS_TSTBUS_CTRL_AXIM |
				WCNSS_TSTBUS_CTRL_AXIM_CFG1;
	writel_relaxed(reg, tst_ctrl_addr);
	reg = readl_relaxed(tst_addr);
	pr_err("AXIM SEL CFG1 testbus %08x\n", reg);

	
	reg = 0;
	reg = reg | WCNSS_TSTBUS_CTRL_EN | WCNSS_TSTBUS_CTRL_CTRL |
		WCNSS_TSTBUS_CTRL_CTRL_CFG0;
	writel_relaxed(reg, tst_ctrl_addr);
	reg = readl_relaxed(tst_addr);
	pr_err("CTRL SEL CFG0 testbus %08x\n", reg);

	
	reg = 0;
	reg = reg | WCNSS_TSTBUS_CTRL_EN | WCNSS_TSTBUS_CTRL_CTRL |
		WCNSS_TSTBUS_CTRL_CTRL_CFG1;
	writel_relaxed(reg, tst_ctrl_addr);
	reg = readl_relaxed(tst_addr);
	pr_err("CTRL SEL CFG1 testbus %08x\n", reg);


	reg_addr = penv->msm_wcnss_base + PRONTO_PMU_WLAN_BCR_OFFSET;
	reg = readl_relaxed(reg_addr);

	reg_addr = penv->msm_wcnss_base + PRONTO_PMU_WLAN_GDSCR_OFFSET;
	reg2 = readl_relaxed(reg_addr);

	reg_addr = penv->msm_wcnss_base + PRONTO_PMU_WLAN_AHB_CBCR_OFFSET;
	reg3 = readl_relaxed(reg_addr);
	pr_err("PMU_WLAN_AHB_CBCR %08x\n", reg3);

	if ((reg & PRONTO_PMU_WLAN_BCR_BLK_ARES) ||
		(reg2 & PRONTO_PMU_WLAN_GDSCR_SW_COLLAPSE) ||
		(!(reg4 & PRONTO_PMU_CPU_AHB_CMD_RCGR_ROOT_EN)) ||
		(reg3 & PRONTO_PMU_WLAN_AHB_CBCR_CLK_OFF) ||
		(!(reg3 & PRONTO_PMU_WLAN_AHB_CBCR_CLK_EN))) {
		pr_err("Cannot log, wlan domain is power collapsed\n");
		return;
	}

	msleep(50);

	reg = readl_relaxed(penv->wlan_tx_phy_aborts);
	pr_err("WLAN_TX_PHY_ABORTS %08x\n", reg);

	reg_addr = penv->pronto_mcu_base + MCU_APB2PHY_STATUS_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("MCU_APB2PHY_STATUS %08x\n", reg);

	reg_addr = penv->pronto_mcu_base + MCU_CBR_CCAHB_ERR_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("MCU_CBR_CCAHB_ERR %08x\n", reg);

	reg_addr = penv->pronto_mcu_base + MCU_CBR_CAHB_ERR_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("MCU_CBR_CAHB_ERR %08x\n", reg);

	reg_addr = penv->pronto_mcu_base + MCU_CBR_CCAHB_TIMEOUT_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("MCU_CBR_CCAHB_TIMEOUT %08x\n", reg);

	reg_addr = penv->pronto_mcu_base + MCU_CBR_CAHB_TIMEOUT_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("MCU_CBR_CAHB_TIMEOUT %08x\n", reg);

	reg_addr = penv->pronto_mcu_base + MCU_DBR_CDAHB_ERR_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("MCU_DBR_CDAHB_ERR %08x\n", reg);

	reg_addr = penv->pronto_mcu_base + MCU_DBR_DAHB_ERR_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("MCU_DBR_DAHB_ERR %08x\n", reg);

	reg_addr = penv->pronto_mcu_base + MCU_DBR_CDAHB_TIMEOUT_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("MCU_DBR_CDAHB_TIMEOUT %08x\n", reg);

	reg_addr = penv->pronto_mcu_base + MCU_DBR_DAHB_TIMEOUT_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("MCU_DBR_DAHB_TIMEOUT %08x\n", reg);

	reg_addr = penv->pronto_mcu_base + MCU_FDBR_CDAHB_ERR_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("MCU_FDBR_CDAHB_ERR %08x\n", reg);

	reg_addr = penv->pronto_mcu_base + MCU_FDBR_FDAHB_ERR_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("MCU_FDBR_FDAHB_ERR %08x\n", reg);

	reg_addr = penv->pronto_mcu_base + MCU_FDBR_CDAHB_TIMEOUT_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("MCU_FDBR_CDAHB_TIMEOUT %08x\n", reg);

	reg_addr = penv->pronto_mcu_base + MCU_FDBR_FDAHB_TIMEOUT_OFFSET;
	reg = readl_relaxed(reg_addr);
	pr_err("MCU_FDBR_FDAHB_TIMEOUT %08x\n", reg);

	reg = readl_relaxed(penv->wlan_brdg_err_source);
	pr_err("WLAN_BRDG_ERR_SOURCE %08x\n", reg);

	reg = readl_relaxed(penv->wlan_tx_status);
	pr_err("WLAN_TXP_STATUS %08x\n", reg);

	reg = readl_relaxed(penv->alarms_txctl);
	pr_err("ALARMS_TXCTL %08x\n", reg);

	reg = readl_relaxed(penv->alarms_tactl);
	pr_err("ALARMS_TACTL %08x\n", reg);
}
EXPORT_SYMBOL(wcnss_pronto_log_debug_regs);

#ifdef CONFIG_WCNSS_REGISTER_DUMP_ON_BITE
static void wcnss_log_iris_regs(void)
{
	int i;
	u32 reg_val;
	u32 regs_array[] = {
		0x04, 0x05, 0x11, 0x1e, 0x40, 0x48,
		0x49, 0x4b, 0x00, 0x01, 0x4d};

	pr_info("IRIS Registers [address] : value\n");

	for (i = 0; i < ARRAY_SIZE(regs_array); i++) {
		reg_val = wcnss_rf_read_reg(regs_array[i]);
		pr_info("[0x%08x] : 0x%08x\n", regs_array[i], reg_val);
	}
}

void wcnss_log_debug_regs_on_bite(void)
{
	struct platform_device *pdev = wcnss_get_platform_device();
	struct clk *measure;
	struct clk *wcnss_debug_mux;
	unsigned long clk_rate;

	if (wcnss_hardware_type() != WCNSS_PRONTO_HW)
		return;

	measure = clk_get(&pdev->dev, "measure");
	wcnss_debug_mux = clk_get(&pdev->dev, "wcnss_debug");

	if (!IS_ERR(measure) && !IS_ERR(wcnss_debug_mux)) {
		if (clk_set_parent(measure, wcnss_debug_mux))
			return;

		clk_rate = clk_get_rate(measure);
		pr_debug("wcnss: clock frequency is: %luHz\n", clk_rate);

		if (clk_rate) {
			wcnss_pronto_log_debug_regs();
		} else {
			pr_err("clock frequency is zero, cannot access PMU or other registers\n");
			wcnss_log_iris_regs();
		}
	}
    else{
        pr_err("Can't access measure or wcnss_debug\n");
    }
}
#endif

void wcnss_reset_intr(void)
{
	if (wcnss_hardware_type() == WCNSS_PRONTO_HW) {
		wcnss_pronto_log_debug_regs();
		wmb();
		__raw_writel(1 << 16, penv->fiq_reg);
	} else {
		wcnss_riva_log_debug_regs();
		wmb();
		__raw_writel(1 << 24, MSM_APCS_GCC_BASE + 0x8);
	}
}
EXPORT_SYMBOL(wcnss_reset_intr);

static int wcnss_create_sysfs(struct device *dev)
{
	int ret;

	if (!dev)
		return -ENODEV;

	ret = device_create_file(dev, &dev_attr_serial_number);
	if (ret)
		return ret;

	ret = device_create_file(dev, &dev_attr_thermal_mitigation);
	if (ret)
		goto remove_serial;

	ret = device_create_file(dev, &dev_attr_wcnss_version);
	if (ret)
		goto remove_thermal;

	ret = device_create_file(dev, &dev_attr_wcnss_mac_addr);
	if (ret)
		goto remove_version;

	return 0;

remove_version:
	device_remove_file(dev, &dev_attr_wcnss_version);
remove_thermal:
	device_remove_file(dev, &dev_attr_thermal_mitigation);
remove_serial:
	device_remove_file(dev, &dev_attr_serial_number);

	return ret;
}

static void wcnss_remove_sysfs(struct device *dev)
{
	if (dev) {
		device_remove_file(dev, &dev_attr_serial_number);
		device_remove_file(dev, &dev_attr_thermal_mitigation);
		device_remove_file(dev, &dev_attr_wcnss_version);
		device_remove_file(dev, &dev_attr_wcnss_mac_addr);
	}
}

static void wcnss_pm_qos_add_request(void)
{
	pr_info("%s: add request", __func__);
	pm_qos_add_request(&penv->wcnss_pm_qos_request, PM_QOS_CPU_DMA_LATENCY,
			PM_QOS_DEFAULT_VALUE);
}

static void wcnss_pm_qos_remove_request(void)
{
	pr_info("%s: remove request", __func__);
	pm_qos_remove_request(&penv->wcnss_pm_qos_request);
}

void wcnss_pm_qos_update_request(int val)
{
	pr_info("%s: update request %d", __func__, val);
	pm_qos_update_request(&penv->wcnss_pm_qos_request, val);
}

void wcnss_disable_pc_remove_req(void)
{
	if (penv->pc_disabled) {
		wcnss_pm_qos_update_request(WCNSS_ENABLE_PC_LATENCY);
		wcnss_pm_qos_remove_request();
		wcnss_allow_suspend();
		penv->pc_disabled = 0;
	}
}

void wcnss_disable_pc_add_req(void)
{
	if (!penv->pc_disabled) {
		wcnss_pm_qos_add_request();
		wcnss_prevent_suspend();
		wcnss_pm_qos_update_request(WCNSS_DISABLE_PC_LATENCY);
		penv->pc_disabled = 1;
	}
}

static void wcnss_smd_notify_event(void *data, unsigned int event)
{
	int len = 0;

	if (penv != data) {
		pr_err("wcnss: invalid env pointer in smd callback\n");
		return;
	}
	switch (event) {
	case SMD_EVENT_DATA:
		len = smd_read_avail(penv->smd_ch);
		if (len < 0) {
			pr_err("wcnss: failed to read from smd %d\n", len);
			return;
		}
		printk("wcnss_debug: wcnss_smd_notify_event() : Start to schedule wcnssctrl_rx_work to read smd packet \r\n");
		schedule_work(&penv->wcnssctrl_rx_work);
		break;

	case SMD_EVENT_OPEN:
		pr_debug("wcnss: opening WCNSS SMD channel :%s",
				WCNSS_CTRL_CHANNEL);
		printk("wcnss_debug: opening WCNSS SMD channel :%s", WCNSS_CTRL_CHANNEL);
		schedule_work(&penv->wcnssctrl_version_work);
		schedule_work(&penv->wcnss_pm_config_work);
		__cancel_delayed_work(&penv->wcnss_pm_qos_del_req);
		schedule_delayed_work(&penv->wcnss_pm_qos_del_req, 0);

		break;

	case SMD_EVENT_CLOSE:
		pr_debug("wcnss: closing WCNSS SMD channel :%s",
				WCNSS_CTRL_CHANNEL);
		penv->nv_downloaded = 0;
		break;

	default:
		break;
	}
}

static void wcnss_post_bootup(struct work_struct *work)
{
	if (do_not_cancel_vote == 1) {
		pr_info("%s: Keeping APPS vote for Iris & WCNSS\n", __func__);
		return;
	}

	pr_info("%s: Cancel APPS vote for Iris & WCNSS\n", __func__);

	
	wcnss_wlan_power(&penv->pdev->dev, &penv->wlan_config,
		WCNSS_WLAN_SWITCH_OFF, NULL);
	
	wcnss_allow_suspend();

}

static int
wcnss_pronto_gpios_config(struct device *dev, bool enable)
{
	int rc = 0;
	int i, j;
	int WCNSS_WLAN_NUM_GPIOS = 5;

	for (i = 0; i < WCNSS_WLAN_NUM_GPIOS; i++) {
		int gpio = of_get_gpio(dev->of_node, i);
		if (enable) {
			rc = gpio_request(gpio, "wcnss_wlan");
			if (rc) {
				pr_err("WCNSS gpio_request %d err %d\n",
					gpio, rc);
				goto fail;
			}
		} else
			gpio_free(gpio);
	}

	return rc;

fail:
	for (j = WCNSS_WLAN_NUM_GPIOS-1; j >= 0; j--) {
		int gpio = of_get_gpio(dev->of_node, i);
		gpio_free(gpio);
	}
	return rc;
}

static int
wcnss_gpios_config(struct resource *gpios_5wire, bool enable)
{
	int i, j;
	int rc = 0;

	for (i = gpios_5wire->start; i <= gpios_5wire->end; i++) {
		if (enable) {
			rc = gpio_request(i, gpios_5wire->name);
			if (rc) {
				pr_err("WCNSS gpio_request %d err %d\n", i, rc);
				goto fail;
			}
		} else
			gpio_free(i);
	}

	return rc;

fail:
	for (j = i-1; j >= gpios_5wire->start; j--)
		gpio_free(j);
	return rc;
}

static int __devinit
wcnss_wlan_ctrl_probe(struct platform_device *pdev)
{
	if (!penv || !penv->triggered)
		return -ENODEV;

	penv->smd_channel_ready = 1;

	pr_info("%s: SMD ctrl channel up\n", __func__);

	
	INIT_DELAYED_WORK(&penv->wcnss_work, wcnss_post_bootup);
	schedule_delayed_work(&penv->wcnss_work, msecs_to_jiffies(10000));

	return 0;
}

void wcnss_flush_delayed_boot_votes()
{
	flush_delayed_work(&penv->wcnss_work);
}
EXPORT_SYMBOL(wcnss_flush_delayed_boot_votes);

static int __devexit
wcnss_wlan_ctrl_remove(struct platform_device *pdev)
{
	if (penv)
		penv->smd_channel_ready = 0;

	pr_info("%s: SMD ctrl channel down\n", __func__);

	return 0;
}


static struct platform_driver wcnss_wlan_ctrl_driver = {
	.driver = {
		.name	= "WLAN_CTRL",
		.owner	= THIS_MODULE,
	},
	.probe	= wcnss_wlan_ctrl_probe,
	.remove	= __devexit_p(wcnss_wlan_ctrl_remove),
};

static int __devexit
wcnss_ctrl_remove(struct platform_device *pdev)
{
	if (penv && penv->smd_ch)
		smd_close(penv->smd_ch);

	return 0;
}

static int __devinit
wcnss_ctrl_probe(struct platform_device *pdev)
{
	int ret = 0;

	if (!penv || !penv->triggered)
		return -ENODEV;

	ret = smd_named_open_on_edge(WCNSS_CTRL_CHANNEL, SMD_APPS_WCNSS,
			&penv->smd_ch, penv, wcnss_smd_notify_event);
	if (ret < 0) {
		pr_err("wcnss: cannot open the smd command channel %s: %d\n",
				WCNSS_CTRL_CHANNEL, ret);
		return -ENODEV;
	}
	smd_disable_read_intr(penv->smd_ch);

	return 0;
}

static struct platform_driver wcnss_ctrl_driver = {
	.driver = {
		.name	= "WCNSS_CTRL",
		.owner	= THIS_MODULE,
	},
	.probe	= wcnss_ctrl_probe,
	.remove	= __devexit_p(wcnss_ctrl_remove),
};

void wcnss_get_monotonic_boottime(struct timespec *ts)
{
	get_monotonic_boottime(ts);
}
EXPORT_SYMBOL(wcnss_get_monotonic_boottime);


struct device *wcnss_wlan_get_device(void)
{
	if (penv && penv->pdev && penv->smd_channel_ready)
		return &penv->pdev->dev;
	return NULL;
}
EXPORT_SYMBOL(wcnss_wlan_get_device);

struct platform_device *wcnss_get_platform_device(void)
{
	if (penv && penv->pdev)
		return penv->pdev;
	return NULL;
}
EXPORT_SYMBOL(wcnss_get_platform_device);

struct wcnss_wlan_config *wcnss_get_wlan_config(void)
{
	if (penv && penv->pdev)
		return &penv->wlan_config;
	return NULL;
}
EXPORT_SYMBOL(wcnss_get_wlan_config);

int wcnss_is_hw_pronto_ver3(void)
{
	if (penv && penv->pdev)
		return penv->wlan_config.is_pronto_v3;
	return 0;
}
EXPORT_SYMBOL(wcnss_is_hw_pronto_ver3);

int wcnss_device_ready(void)
{
	if(penv == NULL)
	{
	    printk("wcnss_debug : wcnss_device is not ready\r\n");
	    return 0;
	}
	printk("wcnss_debug : wcnss_device_ready(): penv->nv_downloaded = 0x%x\r\n", penv->nv_downloaded);
	if (penv && penv->pdev && penv->nv_downloaded &&
	    !wcnss_device_is_shutdown())
		return 1;
	return 0;
}
EXPORT_SYMBOL(wcnss_device_ready);

int wcnss_device_is_shutdown(void)
{
	if (penv && penv->is_shutdown)
		return 1;
	return 0;
}
EXPORT_SYMBOL(wcnss_device_is_shutdown);

struct resource *wcnss_wlan_get_memory_map(struct device *dev)
{
	if (penv && dev && (dev == &penv->pdev->dev) && penv->smd_channel_ready)
		return penv->mmio_res;
	return NULL;
}
EXPORT_SYMBOL(wcnss_wlan_get_memory_map);

int wcnss_wlan_get_dxe_tx_irq(struct device *dev)
{
	if (penv && dev && (dev == &penv->pdev->dev) &&
				penv->tx_irq_res && penv->smd_channel_ready)
		return penv->tx_irq_res->start;
	return WCNSS_WLAN_IRQ_INVALID;
}
EXPORT_SYMBOL(wcnss_wlan_get_dxe_tx_irq);

int wcnss_wlan_get_dxe_rx_irq(struct device *dev)
{
	if (penv && dev && (dev == &penv->pdev->dev) &&
				penv->rx_irq_res && penv->smd_channel_ready)
		return penv->rx_irq_res->start;
	return WCNSS_WLAN_IRQ_INVALID;
}
EXPORT_SYMBOL(wcnss_wlan_get_dxe_rx_irq);

void wcnss_wlan_register_pm_ops(struct device *dev,
				const struct dev_pm_ops *pm_ops)
{
	if (penv && dev && (dev == &penv->pdev->dev) && pm_ops)
		penv->pm_ops = pm_ops;
}
EXPORT_SYMBOL(wcnss_wlan_register_pm_ops);

void wcnss_wlan_unregister_pm_ops(struct device *dev,
				const struct dev_pm_ops *pm_ops)
{
	if (penv && dev && (dev == &penv->pdev->dev) && pm_ops) {
		if (penv->pm_ops == NULL) {
			pr_err("%s: pm_ops is already unregistered.\n",
				 __func__);
			return;
		}

		if (pm_ops->suspend != penv->pm_ops->suspend ||
				pm_ops->resume != penv->pm_ops->resume)
			pr_err("PM APIs dont match with registered APIs\n");
		penv->pm_ops = NULL;
	}
}
EXPORT_SYMBOL(wcnss_wlan_unregister_pm_ops);

void wcnss_register_thermal_mitigation(struct device *dev,
				void (*tm_notify)(struct device *, int))
{
	if (penv && dev && tm_notify)
		penv->tm_notify = tm_notify;
}
EXPORT_SYMBOL(wcnss_register_thermal_mitigation);

void wcnss_unregister_thermal_mitigation(
				void (*tm_notify)(struct device *, int))
{
	if (penv && tm_notify) {
		if (tm_notify != penv->tm_notify)
			pr_err("tm_notify doesn't match registered\n");
		penv->tm_notify = NULL;
	}
}
EXPORT_SYMBOL(wcnss_unregister_thermal_mitigation);

unsigned int wcnss_get_serial_number(void)
{
	if (penv)
		return penv->serial_number;
	return 0;
}
EXPORT_SYMBOL(wcnss_get_serial_number);

int wcnss_get_wlan_mac_address(char mac_addr[WLAN_MAC_ADDR_SIZE])
{
	if (!penv)
		return -ENODEV;

	memcpy(mac_addr, penv->wlan_nv_macAddr, WLAN_MAC_ADDR_SIZE);
	pr_debug("%s: Get MAC Addr:" MAC_ADDRESS_STR "\n", __func__,
		penv->wlan_nv_macAddr[0], penv->wlan_nv_macAddr[1],
		penv->wlan_nv_macAddr[2], penv->wlan_nv_macAddr[3],
		penv->wlan_nv_macAddr[4], penv->wlan_nv_macAddr[5]);
	return 0;
}
EXPORT_SYMBOL(wcnss_get_wlan_mac_address);

static int enable_wcnss_suspend_notify;

static int enable_wcnss_suspend_notify_set(const char *val,
				struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret)
		return ret;

	if (enable_wcnss_suspend_notify)
		pr_debug("Suspend notification activated for wcnss\n");

	return 0;
}
module_param_call(enable_wcnss_suspend_notify, enable_wcnss_suspend_notify_set,
		param_get_int, &enable_wcnss_suspend_notify, S_IRUGO | S_IWUSR);

int wcnss_xo_auto_detect_enabled(void)
{
	return (has_autodetect_xo == 1 ? 1 : 0);
}

int wcnss_wlan_iris_xo_mode(void)
{
	if (penv && penv->pdev && penv->smd_channel_ready)
		return penv->iris_xo_mode_set;
	return -ENODEV;
}
EXPORT_SYMBOL(wcnss_wlan_iris_xo_mode);


void wcnss_suspend_notify(void)
{
	void __iomem *pmu_spare_reg;
	u32 reg = 0;
	unsigned long flags;

	if (!enable_wcnss_suspend_notify)
		return;

	if (wcnss_hardware_type() == WCNSS_PRONTO_HW)
		return;

	
	pmu_spare_reg = penv->msm_wcnss_base + RIVA_SPARE_OFFSET;
	spin_lock_irqsave(&reg_spinlock, flags);
	reg = readl_relaxed(pmu_spare_reg);
	reg |= RIVA_SUSPEND_BIT;
	writel_relaxed(reg, pmu_spare_reg);
	spin_unlock_irqrestore(&reg_spinlock, flags);
}
EXPORT_SYMBOL(wcnss_suspend_notify);

void wcnss_resume_notify(void)
{
	void __iomem *pmu_spare_reg;
	u32 reg = 0;
	unsigned long flags;

	if (!enable_wcnss_suspend_notify)
		return;

	if (wcnss_hardware_type() == WCNSS_PRONTO_HW)
		return;

	
	pmu_spare_reg = penv->msm_wcnss_base + RIVA_SPARE_OFFSET;

	spin_lock_irqsave(&reg_spinlock, flags);
	reg = readl_relaxed(pmu_spare_reg);
	reg &= ~RIVA_SUSPEND_BIT;
	writel_relaxed(reg, pmu_spare_reg);
	spin_unlock_irqrestore(&reg_spinlock, flags);
}
EXPORT_SYMBOL(wcnss_resume_notify);

static int wcnss_wlan_suspend(struct device *dev)
{
	if (penv && dev && (dev == &penv->pdev->dev) &&
	    penv->smd_channel_ready &&
	    penv->pm_ops && penv->pm_ops->suspend)
		return penv->pm_ops->suspend(dev);
	return 0;
}

static int wcnss_wlan_resume(struct device *dev)
{
	if (penv && dev && (dev == &penv->pdev->dev) &&
	    penv->smd_channel_ready &&
	    penv->pm_ops && penv->pm_ops->resume)
		return penv->pm_ops->resume(dev);
	return 0;
}

void wcnss_prevent_suspend()
{
	if (penv)
		wake_lock(&penv->wcnss_wake_lock);
}
EXPORT_SYMBOL(wcnss_prevent_suspend);

void wcnss_allow_suspend()
{
	if (penv)
		wake_unlock(&penv->wcnss_wake_lock);
}
EXPORT_SYMBOL(wcnss_allow_suspend);

int wcnss_hardware_type(void)
{
	if (penv)
		return penv->wcnss_hw_type;
	else
		return -ENODEV;
}
EXPORT_SYMBOL(wcnss_hardware_type);

int fw_cal_data_available(void)
{
	if (penv)
		return penv->fw_cal_available;
	else
		return -ENODEV;
}

u32 wcnss_get_wlan_rx_buff_count(void)
{
	if (penv)
		return penv->wlan_rx_buff_count;
	else
		return WCNSS_DEF_WLAN_RX_BUFF_COUNT;

}
EXPORT_SYMBOL(wcnss_get_wlan_rx_buff_count);

int wcnss_set_wlan_unsafe_channel(u16 *unsafe_ch_list, u16 ch_count)
{
	if (penv && unsafe_ch_list &&
		(ch_count <= WCNSS_MAX_CH_NUM)) {
		memcpy((char *)penv->unsafe_ch_list,
			(char *)unsafe_ch_list, ch_count * sizeof(u16));
		penv->unsafe_ch_count = ch_count;
		return 0;
	} else
		return -ENODEV;
}
EXPORT_SYMBOL(wcnss_set_wlan_unsafe_channel);

int wcnss_get_wlan_unsafe_channel(u16 *unsafe_ch_list, u16 buffer_size,
					u16 *ch_count)
{
	if (penv) {
		if (buffer_size < penv->unsafe_ch_count * sizeof(u16))
			return -ENODEV;
		memcpy((char *)unsafe_ch_list,
			(char *)penv->unsafe_ch_list,
			penv->unsafe_ch_count * sizeof(u16));
		*ch_count = penv->unsafe_ch_count;
		return 0;
	} else
		return -ENODEV;
}
EXPORT_SYMBOL(wcnss_get_wlan_unsafe_channel);

static int wcnss_smd_tx(void *data, int len)
{
	int ret = 0;

	ret = smd_write_avail(penv->smd_ch);
	if (ret < len) {
		pr_err("wcnss: no space available for smd frame\n");
		return -ENOSPC;
	}
	ret = smd_write(penv->smd_ch, data, len);
	if (ret < len) {
		pr_err("wcnss: failed to write Command %d", len);
		ret = -ENODEV;
	}
	printk("wcnss_debug: wcnss_smd_tx() finish and exit \r\n");
	return ret;
}

static void wcnss_notify_vbat(enum qpnp_tm_state state, void *ctx)
{
	mutex_lock(&penv->vbat_monitor_mutex);
	cancel_delayed_work_sync(&penv->vbatt_work);

	if (state == ADC_TM_LOW_STATE) {
		pr_debug("wcnss: low voltage notification triggered\n");
		penv->vbat_monitor_params.state_request =
			ADC_TM_HIGH_THR_ENABLE;
		penv->vbat_monitor_params.high_thr = WCNSS_VBATT_THRESHOLD +
		WCNSS_VBATT_GUARD;
		penv->vbat_monitor_params.low_thr = 0;
	} else if (state == ADC_TM_HIGH_STATE) {
		penv->vbat_monitor_params.state_request =
			ADC_TM_LOW_THR_ENABLE;
		penv->vbat_monitor_params.low_thr = WCNSS_VBATT_THRESHOLD -
		WCNSS_VBATT_GUARD;
		penv->vbat_monitor_params.high_thr = 0;
		pr_debug("wcnss: high voltage notification triggered\n");
	} else {
		pr_debug("wcnss: unknown voltage notification state: %d\n",
				state);
		mutex_unlock(&penv->vbat_monitor_mutex);
		return;
	}
	pr_debug("wcnss: set low thr to %d and high to %d\n",
			penv->vbat_monitor_params.low_thr,
			penv->vbat_monitor_params.high_thr);

	qpnp_adc_tm_channel_measure(penv->adc_tm_dev,
			&penv->vbat_monitor_params);
	schedule_delayed_work(&penv->vbatt_work, msecs_to_jiffies(2000));
	mutex_unlock(&penv->vbat_monitor_mutex);
}

static int wcnss_setup_vbat_monitoring(void)
{
	int rc = -1;

	if (!penv->adc_tm_dev) {
		pr_err("wcnss: not setting up vbatt\n");
		return rc;
	}
	penv->vbat_monitor_params.low_thr = WCNSS_VBATT_THRESHOLD;
	penv->vbat_monitor_params.high_thr = WCNSS_VBATT_THRESHOLD;
	penv->vbat_monitor_params.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
	penv->vbat_monitor_params.channel = VBAT_SNS;
	penv->vbat_monitor_params.btm_ctx = (void *)penv;
	penv->vbat_monitor_params.timer_interval = ADC_MEAS1_INTERVAL_1S;
	penv->vbat_monitor_params.threshold_notification = &wcnss_notify_vbat;
	pr_debug("wcnss: set low thr to %d and high to %d\n",
			penv->vbat_monitor_params.low_thr,
			penv->vbat_monitor_params.high_thr);

	rc = qpnp_adc_tm_channel_measure(penv->adc_tm_dev,
					&penv->vbat_monitor_params);
	if (rc)
		pr_err("wcnss: tm setup failed: %d\n", rc);

	return rc;
}

static void wcnss_update_vbatt(struct work_struct *work)
{
	struct vbatt_message vbatt_msg;
	int ret = 0;

	vbatt_msg.hdr.msg_type = WCNSS_VBATT_LEVEL_IND;
	vbatt_msg.hdr.msg_len = sizeof(struct vbatt_message);
	vbatt_msg.vbatt.threshold = WCNSS_VBATT_THRESHOLD;

	mutex_lock(&penv->vbat_monitor_mutex);
	if (penv->vbat_monitor_params.low_thr &&
		(penv->fw_vbatt_state == WCNSS_VBATT_LOW ||
			penv->fw_vbatt_state == WCNSS_CONFIG_UNSPECIFIED)) {
		vbatt_msg.vbatt.curr_volt = WCNSS_VBATT_HIGH;
		penv->fw_vbatt_state = WCNSS_VBATT_HIGH;
		pr_debug("wcnss: send HIGH BATT to FW\n");
	} else if (!penv->vbat_monitor_params.low_thr &&
		(penv->fw_vbatt_state == WCNSS_VBATT_HIGH ||
			penv->fw_vbatt_state == WCNSS_CONFIG_UNSPECIFIED)){
		vbatt_msg.vbatt.curr_volt = WCNSS_VBATT_LOW;
		penv->fw_vbatt_state = WCNSS_VBATT_LOW;
		pr_debug("wcnss: send LOW BATT to FW\n");
	} else {
		mutex_unlock(&penv->vbat_monitor_mutex);
		return;
	}
	mutex_unlock(&penv->vbat_monitor_mutex);
	ret = wcnss_smd_tx(&vbatt_msg, vbatt_msg.hdr.msg_len);
	if (ret < 0)
		pr_err("wcnss: smd tx failed\n");
	return;
}


static unsigned char wcnss_fw_status(void)
{
	int len = 0;
	int rc = 0;

	unsigned char fw_status = 0xFF;

	len = smd_read_avail(penv->smd_ch);
	if (len < 1) {
		pr_err("%s: invalid firmware status", __func__);
		return fw_status;
	}

	rc = smd_read(penv->smd_ch, &fw_status, 1);
	if (rc < 0) {
		pr_err("%s: incomplete data read from smd\n", __func__);
		return fw_status;
	}
	return fw_status;
}

static void wcnss_send_cal_rsp(unsigned char fw_status)
{
	struct smd_msg_hdr *rsphdr;
	unsigned char *msg = NULL;
	int rc;

	msg = kmalloc((sizeof(struct smd_msg_hdr) + 1), GFP_KERNEL);
	if (NULL == msg) {
		pr_err("wcnss: %s: failed to get memory\n", __func__);
		return;
	}

	rsphdr = (struct smd_msg_hdr *)msg;
	rsphdr->msg_type = WCNSS_CALDATA_UPLD_RSP;
	rsphdr->msg_len = sizeof(struct smd_msg_hdr) + 1;
	memcpy(msg+sizeof(struct smd_msg_hdr), &fw_status, 1);

	rc = wcnss_smd_tx(msg, rsphdr->msg_len);
	if (rc < 0)
		pr_err("wcnss: smd tx failed\n");

	kfree(msg);
}

void extract_cal_data(int len)
{
	int rc;
	struct cal_data_params calhdr;
	unsigned char fw_status = WCNSS_RESP_FAIL;

	if (len < sizeof(struct cal_data_params)) {
		pr_err("wcnss: incomplete cal header length\n");
		return;
	}

	rc = smd_read(penv->smd_ch, (unsigned char *)&calhdr,
			sizeof(struct cal_data_params));
	if (rc < sizeof(struct cal_data_params)) {
		pr_err("wcnss: incomplete cal header read from smd\n");
		return;
	}

	if (penv->fw_cal_exp_frag != calhdr.frag_number) {
		pr_err("wcnss: Invalid frgament");
		goto exit;
	}

	if (calhdr.frag_size > WCNSS_MAX_FRAME_SIZE) {
		pr_err("wcnss: Invalid fragment size");
		goto exit;
	}

	if (penv->fw_cal_available) {
		
		smd_read(penv->smd_ch, NULL, calhdr.frag_size);
		penv->fw_cal_exp_frag++;
		if (calhdr.msg_flags & LAST_FRAGMENT) {
			penv->fw_cal_exp_frag = 0;
			goto exit;
		}
		return;
	}

	if (0 == calhdr.frag_number) {
		if (calhdr.total_size > MAX_CALIBRATED_DATA_SIZE) {
			pr_err("wcnss: Invalid cal data size %d",
				calhdr.total_size);
			goto exit;
		}
		kfree(penv->fw_cal_data);
		penv->fw_cal_rcvd = 0;
		penv->fw_cal_data = kmalloc(calhdr.total_size,
				GFP_KERNEL);
		if (penv->fw_cal_data == NULL) {
			smd_read(penv->smd_ch, NULL, calhdr.frag_size);
			goto exit;
		}
	}

	mutex_lock(&penv->dev_lock);
	if (penv->fw_cal_rcvd + calhdr.frag_size >
			MAX_CALIBRATED_DATA_SIZE) {
		pr_err("calibrated data size is more than expected %d",
				penv->fw_cal_rcvd + calhdr.frag_size);
		penv->fw_cal_exp_frag = 0;
		penv->fw_cal_rcvd = 0;
		smd_read(penv->smd_ch, NULL, calhdr.frag_size);
		goto unlock_exit;
	}

	rc = smd_read(penv->smd_ch, penv->fw_cal_data + penv->fw_cal_rcvd,
			calhdr.frag_size);
	if (rc < calhdr.frag_size)
		goto unlock_exit;

	penv->fw_cal_exp_frag++;
	penv->fw_cal_rcvd += calhdr.frag_size;

	if (calhdr.msg_flags & LAST_FRAGMENT) {
		penv->fw_cal_exp_frag = 0;
		penv->fw_cal_available = true;
		pr_info("wcnss: cal data collection completed\n");
	}
	mutex_unlock(&penv->dev_lock);
	wake_up(&penv->read_wait);

	if (penv->fw_cal_available) {
		fw_status = WCNSS_RESP_SUCCESS;
		wcnss_send_cal_rsp(fw_status);
	}
	return;

unlock_exit:
	mutex_unlock(&penv->dev_lock);

exit:
	wcnss_send_cal_rsp(fw_status);
	return;
}


static void wcnssctrl_rx_handler(struct work_struct *worker)
{
	int len = 0;
	int rc = 0;
	unsigned char buf[sizeof(struct wcnss_version)];
	unsigned char build[WCNSS_MAX_BUILD_VER_LEN+1];
	struct smd_msg_hdr *phdr;
	struct smd_msg_hdr smd_msg;
	struct wcnss_version *pversion;
	int hw_type;
	unsigned char fw_status = 0;

	len = smd_read_avail(penv->smd_ch);
	if (len > WCNSS_MAX_FRAME_SIZE) {
		pr_err("wcnss: frame larger than the allowed size\n");
		smd_read(penv->smd_ch, NULL, len);
		return;
	}
	if (len <= 0)
		return;

	rc = smd_read(penv->smd_ch, buf, sizeof(struct smd_msg_hdr));
	if (rc < sizeof(struct smd_msg_hdr)) {
		pr_err("wcnss: incomplete header read from smd\n");
		return;
	}
	len -= sizeof(struct smd_msg_hdr);

	phdr = (struct smd_msg_hdr *)buf;

	switch (phdr->msg_type) {

	case WCNSS_VERSION_RSP:
		if (len != sizeof(struct wcnss_version)
				- sizeof(struct smd_msg_hdr)) {
			pr_err("wcnss: invalid version data from wcnss %d\n",
					len);
			return;
		}
		rc = smd_read(penv->smd_ch, buf+sizeof(struct smd_msg_hdr),
				len);
		if (rc < len) {
			pr_err("wcnss: incomplete data read from smd\n");
			return;
		}
		pversion = (struct wcnss_version *)buf;
		penv->fw_major = pversion->major;
		penv->fw_minor = pversion->minor;
		snprintf(penv->wcnss_version, WCNSS_VERSION_LEN,
			"%02x%02x%02x%02x", pversion->major, pversion->minor,
					pversion->version, pversion->revision);
		pr_info("wcnss: version %s\n", penv->wcnss_version);
		
		hw_type = wcnss_hardware_type();
		switch (hw_type) {
		case WCNSS_RIVA_HW:
			
			if ((pversion->major >= 1) && (pversion->minor >= 4)) {
				pr_info("wcnss: schedule dnld work for riva\n");
				schedule_work(&penv->wcnssctrl_nvbin_dnld_work);
			}
			break;

		case WCNSS_PRONTO_HW:
			smd_msg.msg_type = WCNSS_BUILD_VER_REQ;
			smd_msg.msg_len = sizeof(smd_msg);
			rc = wcnss_smd_tx(&smd_msg, smd_msg.msg_len);
			if (rc < 0)
				pr_err("wcnss: smd tx failed: %s\n", __func__);

			
			if ((pversion->major >= 1) && (pversion->minor >= 4)) {
				pr_info("wcnss: schedule dnld work for pronto\n");
				schedule_work(&penv->wcnssctrl_nvbin_dnld_work);
			}
			break;

		default:
			pr_info("wcnss: unknown hw type (%d), will not schedule dnld work\n",
				hw_type);
			break;
		}
		break;

	case WCNSS_BUILD_VER_RSP:
		if (len > WCNSS_MAX_BUILD_VER_LEN) {
			pr_err("wcnss: invalid build version data from wcnss %d\n",
					len);
			return;
		}
		rc = smd_read(penv->smd_ch, build, len);
		if (rc < len) {
			pr_err("wcnss: incomplete data read from smd\n");
			return;
		}
		build[len] = 0;
		pr_info("wcnss: build version %s\n", build);
		break;

	case WCNSS_NVBIN_DNLD_RSP:
		penv->nv_downloaded = true;
		fw_status = wcnss_fw_status();
		pr_debug("wcnss: received WCNSS_NVBIN_DNLD_RSP from ccpu %u\n",
			fw_status);
		wcnss_setup_vbat_monitoring();
		break;

	case WCNSS_CALDATA_DNLD_RSP:
		penv->nv_downloaded = true;
		fw_status = wcnss_fw_status();
		pr_debug("wcnss: received WCNSS_CALDATA_DNLD_RSP from ccpu %u\n",
			fw_status);
		break;

	case WCNSS_CALDATA_UPLD_REQ:
		extract_cal_data(len);
		break;

	default:
		pr_err("wcnss: invalid message type %d\n", phdr->msg_type);
	}
	return;
}

static void wcnss_send_version_req(struct work_struct *worker)
{
	struct smd_msg_hdr smd_msg;
	int ret = 0;

	smd_msg.msg_type = WCNSS_VERSION_REQ;
	smd_msg.msg_len = sizeof(smd_msg);
	ret = wcnss_smd_tx(&smd_msg, smd_msg.msg_len);
	if (ret < 0)
		pr_err("wcnss: smd tx failed\n");

	return;
}

static void wcnss_send_pm_config(struct work_struct *worker)
{
	struct smd_msg_hdr *hdr;
	unsigned char *msg = NULL;
	int rc, prop_len;
	u32 *payload;

	if (!of_find_property(penv->pdev->dev.of_node,
				"qcom,wcnss-pm", &prop_len))
		return;

	msg = kmalloc((sizeof(struct smd_msg_hdr) + prop_len), GFP_KERNEL);

	if (NULL == msg) {
		pr_err("wcnss: %s: failed to allocate memory\n", __func__);
		return;
	}

	payload = (u32 *)(msg + sizeof(struct smd_msg_hdr));

	prop_len /= sizeof(int);

	rc = of_property_read_u32_array(penv->pdev->dev.of_node,
			"qcom,wcnss-pm", payload, prop_len);
	if (rc < 0) {
		pr_err("wcnss: property read failed\n");
		kfree(msg);
		return;
	}

	pr_debug("%s:size=%d: <%d, %d, %d, %d, %d>\n", __func__,
			prop_len, *payload, *(payload+1), *(payload+2),
			*(payload+3), *(payload+4));

	hdr = (struct smd_msg_hdr *)msg;
	hdr->msg_type = WCNSS_PM_CONFIG_REQ;
	hdr->msg_len = sizeof(struct smd_msg_hdr) + prop_len;

	rc = wcnss_smd_tx(msg, hdr->msg_len);
	if (rc < 0)
		pr_err("wcnss: smd tx failed\n");

	kfree(msg);
	return;
}

static DECLARE_RWSEM(wcnss_pm_sem);

static int engid = 0;
static int engidprint(char *str){
    int ret;
    ret = get_option(&str, &engid);
    return ret;
}

int getengid(void)
{
    return engid;
}
EXPORT_SYMBOL(getengid);

__setup("androidboot.engid=",engidprint);

static void wcnss_nvbin_dnld(void)
{
	int ret = 0;
	struct nvbin_dnld_req_msg *dnld_req_msg;
	unsigned short total_fragments = 0;
	unsigned short count = 0;
	unsigned short retry_count = 0;
	unsigned short cur_frag_size = 0;
	unsigned char *outbuffer = NULL;
	const void *nv_blob_addr = NULL;
	unsigned int nv_blob_size = 0;
	const struct firmware *nv = NULL;
	struct device *dev = &penv->pdev->dev;

    down_read(&wcnss_pm_sem);

    
    
    pr_err("engid %d\n", engid);
    if (engid >= 8) {
        pr_err("wcnss: read WCNSS_qcom_wlan_nv_EPCOS.bin\n");
        ret = request_firmware(&nv, NVBIN_FILE_EPCOS, dev);

        if (ret || !nv || !nv->data || !nv->size) {
            pr_err("wcnss: %s: request_firmware failed for %s\n",
                      __func__, NVBIN_FILE_EPCOS);
            pr_err("wcnss: fail to read WCNSS_qcom_wlan_nv_EPCOS.bin -> read WCNSS_qcom_wlan_nv.bin \n");
            ret = request_firmware(&nv, NVBIN_FILE, dev);
            if (ret || !nv || !nv->data || !nv->size) {
                pr_err("wcnss: %s: request_firmware failed for %s(ret = %d)\n",
                    __func__, NVBIN_FILE, ret);
                goto out;
            }
        }
    } else {
            pr_err("wcnss: read WCNSS_qcom_wlan_nv.bin \n");
    

    	ret = request_firmware(&nv, NVBIN_FILE, dev);

    	if (ret || !nv || !nv->data || !nv->size) {
    		pr_err("wcnss: %s: request_firmware failed for %s(ret = %d)\n",
    			__func__, NVBIN_FILE, ret);
    		goto out;
    	}

    
    }
    

	nv_blob_addr = nv->data + 4;
	nv_blob_size = nv->size - 4;

	total_fragments = TOTALFRAGMENTS(nv_blob_size);

	pr_info("wcnss: NV bin size: %d, total_fragments: %d\n",
		nv_blob_size, total_fragments);

	
	outbuffer = kmalloc((sizeof(struct nvbin_dnld_req_msg) +
		NV_FRAGMENT_SIZE), GFP_KERNEL);

	if (NULL == outbuffer) {
		pr_err("wcnss: %s: failed to get buffer\n", __func__);
		goto err_free_nv;
	}

	dnld_req_msg = (struct nvbin_dnld_req_msg *)outbuffer;

	dnld_req_msg->hdr.msg_type = WCNSS_NVBIN_DNLD_REQ;
	dnld_req_msg->dnld_req_params.msg_flags = 0;

	for (count = 0; count < total_fragments; count++) {
		dnld_req_msg->dnld_req_params.frag_number = count;

		if (count == (total_fragments - 1)) {
			
			cur_frag_size = nv_blob_size % NV_FRAGMENT_SIZE;
			if (!cur_frag_size)
				cur_frag_size = NV_FRAGMENT_SIZE;

			dnld_req_msg->dnld_req_params.msg_flags |=
				LAST_FRAGMENT;
			dnld_req_msg->dnld_req_params.msg_flags |=
				CAN_RECEIVE_CALDATA;
		} else {
			cur_frag_size = NV_FRAGMENT_SIZE;
			dnld_req_msg->dnld_req_params.msg_flags &=
				~LAST_FRAGMENT;
		}

		dnld_req_msg->dnld_req_params.nvbin_buffer_size =
			cur_frag_size;

		dnld_req_msg->hdr.msg_len =
			sizeof(struct nvbin_dnld_req_msg) + cur_frag_size;

		
		memcpy((outbuffer + sizeof(struct nvbin_dnld_req_msg)),
			(nv_blob_addr + count * NV_FRAGMENT_SIZE),
			cur_frag_size);

		ret = wcnss_smd_tx(outbuffer, dnld_req_msg->hdr.msg_len);

		retry_count = 0;
		while ((ret == -ENOSPC) && (retry_count <= 3)) {
			pr_debug("wcnss: %s: smd tx failed, ENOSPC\n",
				__func__);
			pr_debug("fragment: %d, len: %d, TotFragments: %d, retry_count: %d\n",
				count, dnld_req_msg->hdr.msg_len,
				total_fragments, retry_count);

			
			msleep(20);
			retry_count++;
			ret = wcnss_smd_tx(outbuffer,
				dnld_req_msg->hdr.msg_len);
		}

		if (ret < 0) {
			pr_err("wcnss: %s: smd tx failed\n", __func__);
			pr_err("fragment %d, len: %d, TotFragments: %d, retry_count: %d\n",
				count, dnld_req_msg->hdr.msg_len,
				total_fragments, retry_count);
			goto err_dnld;
		}
	}

err_dnld:
	
	kfree(outbuffer);

err_free_nv:
	
	release_firmware(nv);

out:
	up_read(&wcnss_pm_sem);

	return;
}

static void wcnss_pm_qos_enable_pc(struct work_struct *worker)
{
	wcnss_disable_pc_remove_req();
	return;
}

static void wcnss_caldata_dnld(const void *cal_data,
		unsigned int cal_data_size, bool msg_to_follow)
{
	int ret = 0;
	struct cal_data_msg *cal_msg;
	unsigned short total_fragments = 0;
	unsigned short count = 0;
	unsigned short retry_count = 0;
	unsigned short cur_frag_size = 0;
	unsigned char *outbuffer = NULL;

	total_fragments = TOTALFRAGMENTS(cal_data_size);

	outbuffer = kmalloc((sizeof(struct cal_data_msg) +
		NV_FRAGMENT_SIZE), GFP_KERNEL);

	if (NULL == outbuffer) {
		pr_err("wcnss: %s: failed to get buffer\n", __func__);
		return;
	}

	cal_msg = (struct cal_data_msg *)outbuffer;

	cal_msg->hdr.msg_type = WCNSS_CALDATA_DNLD_REQ;
	cal_msg->cal_params.msg_flags = 0;

	for (count = 0; count < total_fragments; count++) {
		cal_msg->cal_params.frag_number = count;

		if (count == (total_fragments - 1)) {
			cur_frag_size = cal_data_size % NV_FRAGMENT_SIZE;
			if (!cur_frag_size)
				cur_frag_size = NV_FRAGMENT_SIZE;

			cal_msg->cal_params.msg_flags
			    |= LAST_FRAGMENT;
			if (msg_to_follow)
				cal_msg->cal_params.msg_flags |=
					MESSAGE_TO_FOLLOW;
		} else {
			cur_frag_size = NV_FRAGMENT_SIZE;
			cal_msg->cal_params.msg_flags &=
				~LAST_FRAGMENT;
		}

		cal_msg->cal_params.total_size = cal_data_size;
		cal_msg->cal_params.frag_size =
			cur_frag_size;

		cal_msg->hdr.msg_len =
			sizeof(struct cal_data_msg) + cur_frag_size;

		memcpy((outbuffer + sizeof(struct cal_data_msg)),
			(cal_data + count * NV_FRAGMENT_SIZE),
			cur_frag_size);

		ret = wcnss_smd_tx(outbuffer, cal_msg->hdr.msg_len);

		retry_count = 0;
		while ((ret == -ENOSPC) && (retry_count <= 3)) {
			pr_debug("wcnss: %s: smd tx failed, ENOSPC\n",
					__func__);
			pr_debug("fragment: %d, len: %d, TotFragments: %d, retry_count: %d\n",
				count, cal_msg->hdr.msg_len,
				total_fragments, retry_count);

			
			msleep(20);
			retry_count++;
			ret = wcnss_smd_tx(outbuffer,
				cal_msg->hdr.msg_len);
		}

		if (ret < 0) {
			pr_err("wcnss: %s: smd tx failed\n", __func__);
			pr_err("fragment %d, len: %d, TotFragments: %d, retry_count: %d\n",
				count, cal_msg->hdr.msg_len,
				total_fragments, retry_count);
			goto err_dnld;
		}
	}


err_dnld:
	
	kfree(outbuffer);

	return;
}


static void wcnss_nvbin_dnld_main(struct work_struct *worker)
{
	int retry = 0;

	if (!FW_CALDATA_CAPABLE())
		goto nv_download;

	if (!penv->fw_cal_available && WCNSS_CONFIG_UNSPECIFIED
		!= has_calibrated_data && !penv->user_cal_available) {
		while (!penv->user_cal_available && retry++ < 5)
			msleep(500);
	}
	if (penv->fw_cal_available) {
		pr_info_ratelimited("wcnss: cal download, using fw cal");
		wcnss_caldata_dnld(penv->fw_cal_data, penv->fw_cal_rcvd, true);

	} else if (penv->user_cal_available) {
		pr_info_ratelimited("wcnss: cal download, using user cal");
		wcnss_caldata_dnld(penv->user_cal_data,
		penv->user_cal_rcvd, true);
	}

nv_download:
	pr_info_ratelimited("wcnss: NV download");
	wcnss_nvbin_dnld();

	return;
}

static int wcnss_pm_notify(struct notifier_block *b,
			unsigned long event, void *p)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		down_write(&wcnss_pm_sem);
		break;

	case PM_POST_SUSPEND:
		up_write(&wcnss_pm_sem);
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block wcnss_pm_notifier = {
	.notifier_call = wcnss_pm_notify,
};

static int wcnss_ctrl_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	if (!penv || penv->ctrl_device_opened)
		return -EFAULT;

	penv->ctrl_device_opened = 1;

	return rc;
}


void process_usr_ctrl_cmd(u8 *buf, size_t len)
{
	u16 cmd = buf[0] << 8 | buf[1];

	switch (cmd) {

	case WCNSS_USR_SERIAL_NUM:
		if (WCNSS_MIN_SERIAL_LEN > len) {
			pr_err("%s: Invalid serial number\n", __func__);
			return;
		}
		penv->serial_number = buf[2] << 24 | buf[3] << 16
			| buf[4] << 8 | buf[5];
		break;

	case WCNSS_USR_HAS_CAL_DATA:
		if (1 < buf[2])
			pr_err("%s: Invalid data for cal %d\n", __func__,
				buf[2]);
		has_calibrated_data = buf[2];
		break;

	case WCNSS_USR_WLAN_MAC_ADDR:
		memcpy(&penv->wlan_nv_macAddr,  &buf[2],
				sizeof(penv->wlan_nv_macAddr));

		pr_debug("%s: MAC Addr:" MAC_ADDRESS_STR "\n", __func__,
			penv->wlan_nv_macAddr[0], penv->wlan_nv_macAddr[1],
			penv->wlan_nv_macAddr[2], penv->wlan_nv_macAddr[3],
			penv->wlan_nv_macAddr[4], penv->wlan_nv_macAddr[5]);
		break;

	default:
		pr_err("%s: Invalid command %d\n", __func__, cmd);
		break;
	}
}

static ssize_t wcnss_ctrl_write(struct file *fp, const char __user
			*user_buffer, size_t count, loff_t *position)
{
	int rc = 0;
	u8 buf[WCNSS_MAX_CMD_LEN];

	if (!penv || !penv->ctrl_device_opened || WCNSS_MAX_CMD_LEN < count
			|| WCNSS_MIN_CMD_LEN > count)
		return -EFAULT;

	mutex_lock(&penv->ctrl_lock);
	rc = copy_from_user(buf, user_buffer, count);
	if (0 == rc)
		process_usr_ctrl_cmd(buf, count);

	mutex_unlock(&penv->ctrl_lock);

	return rc;
}


static const struct file_operations wcnss_ctrl_fops = {
	.owner = THIS_MODULE,
	.open = wcnss_ctrl_open,
	.write = wcnss_ctrl_write,
};

static struct miscdevice wcnss_usr_ctrl = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = CTRL_DEVICE,
	.fops = &wcnss_ctrl_fops,
};

static int
wcnss_trigger_config(struct platform_device *pdev)
{
	int ret;
	struct qcom_wcnss_opts *pdata;
	unsigned long wcnss_phys_addr;
	int size = 0;
	struct resource *res;
	int is_pronto_v3;
	int pil_retry = 0;
	int has_pronto_hw = of_property_read_bool(pdev->dev.of_node,
									"qcom,has-pronto-hw");

	is_pronto_v3 = of_property_read_bool(pdev->dev.of_node,
							"qcom,is-pronto-v3");

	if (of_property_read_u32(pdev->dev.of_node,
			"qcom,wlan-rx-buff-count", &penv->wlan_rx_buff_count)) {
		penv->wlan_rx_buff_count = WCNSS_DEF_WLAN_RX_BUFF_COUNT;
	}

	
	if (penv->triggered)
		return 0;
	penv->triggered = 1;

	
	pdata = pdev->dev.platform_data;
	if (WCNSS_CONFIG_UNSPECIFIED == has_48mhz_xo) {
		if (has_pronto_hw) {
			has_48mhz_xo = of_property_read_bool(pdev->dev.of_node,
										"qcom,has-48mhz-xo");
		} else {
			has_48mhz_xo = pdata->has_48mhz_xo;
		}
	}
	penv->wcnss_hw_type = (has_pronto_hw) ? WCNSS_PRONTO_HW : WCNSS_RIVA_HW;
	penv->wlan_config.use_48mhz_xo = has_48mhz_xo;
	penv->wlan_config.is_pronto_v3 = is_pronto_v3;

	if (WCNSS_CONFIG_UNSPECIFIED == has_autodetect_xo && has_pronto_hw) {
		has_autodetect_xo = of_property_read_bool(pdev->dev.of_node,
									"qcom,has-autodetect-xo");
	}

	penv->thermal_mitigation = 0;
	strlcpy(penv->wcnss_version, "INVALID", WCNSS_VERSION_LEN);

	
	if (!has_pronto_hw) {
		penv->gpios_5wire = platform_get_resource_byname(pdev,
					IORESOURCE_IO, "wcnss_gpios_5wire");

		
		if (!penv->gpios_5wire) {
			dev_err(&pdev->dev, "insufficient IO resources\n");
			ret = -ENOENT;
			goto fail_gpio_res;
		}
		ret = wcnss_gpios_config(penv->gpios_5wire, true);
	} else
		ret = wcnss_pronto_gpios_config(&pdev->dev, true);

	if (ret) {
		dev_err(&pdev->dev, "WCNSS gpios config failed.\n");
		goto fail_gpio_res;
	}

	
	ret = wcnss_wlan_power(&pdev->dev, &penv->wlan_config,
					WCNSS_WLAN_SWITCH_ON,
					&penv->iris_xo_mode_set);
	if (ret) {
		dev_err(&pdev->dev, "WCNSS Power-up failed.\n");
		goto fail_power;
	}

	
	penv->mmio_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							"wcnss_mmio");
	penv->tx_irq_res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
							"wcnss_wlantx_irq");
	penv->rx_irq_res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
							"wcnss_wlanrx_irq");

	if (!(penv->mmio_res && penv->tx_irq_res && penv->rx_irq_res)) {
		dev_err(&pdev->dev, "insufficient resources\n");
		ret = -ENOENT;
		goto fail_res;
	}
	INIT_WORK(&penv->wcnssctrl_rx_work, wcnssctrl_rx_handler);
	INIT_WORK(&penv->wcnssctrl_version_work, wcnss_send_version_req);
	INIT_WORK(&penv->wcnss_pm_config_work, wcnss_send_pm_config);
	INIT_WORK(&penv->wcnssctrl_nvbin_dnld_work, wcnss_nvbin_dnld_main);
	INIT_DELAYED_WORK(&penv->wcnss_pm_qos_del_req, wcnss_pm_qos_enable_pc);

	wake_lock_init(&penv->wcnss_wake_lock, WAKE_LOCK_SUSPEND, "wcnss");
	
	wcnss_disable_pc_add_req();

	if (wcnss_hardware_type() == WCNSS_PRONTO_HW) {
		size = 0x3000;
		wcnss_phys_addr = MSM_PRONTO_PHYS;
	} else {
		wcnss_phys_addr = MSM_RIVA_PHYS;
		size = SZ_256;
	}

	penv->msm_wcnss_base = ioremap(wcnss_phys_addr, size);
	if (!penv->msm_wcnss_base) {
		ret = -ENOMEM;
		pr_err("%s: ioremap wcnss physical failed\n", __func__);
		goto fail_ioremap;
	}

	if (wcnss_hardware_type() == WCNSS_RIVA_HW) {
		penv->riva_ccu_base =  ioremap(MSM_RIVA_CCU_BASE, SZ_512);
		if (!penv->riva_ccu_base) {
			ret = -ENOMEM;
			pr_err("%s: ioremap wcnss physical failed\n", __func__);
			goto fail_ioremap2;
		}
	} else {
		penv->pronto_a2xb_base =  ioremap(MSM_PRONTO_A2XB_BASE, SZ_512);
		if (!penv->pronto_a2xb_base) {
			ret = -ENOMEM;
			pr_err("%s: ioremap wcnss physical failed\n", __func__);
			goto fail_ioremap2;
		}
		penv->pronto_ccpu_base =  ioremap(MSM_PRONTO_CCPU_BASE, SZ_512);
		if (!penv->pronto_ccpu_base) {
			ret = -ENOMEM;
			pr_err("%s: ioremap wcnss physical failed\n", __func__);
			goto fail_ioremap3;
		}
		
		res = platform_get_resource_byname(penv->pdev,
				IORESOURCE_MEM, "wcnss_fiq");
		if (!res) {
			dev_err(&pdev->dev, "insufficient irq mem resources\n");
			ret = -ENOENT;
			goto fail_ioremap4;
		}
		penv->fiq_reg = ioremap_nocache(res->start, resource_size(res));
		if (!penv->fiq_reg) {
			pr_err("wcnss: %s: ioremap_nocache() failed fiq_reg addr:%pr\n",
				__func__, &res->start);
			ret = -ENOMEM;
			goto fail_ioremap4;
		}
		penv->pronto_saw2_base = ioremap_nocache(MSM_PRONTO_SAW2_BASE,
				SZ_32);
		if (!penv->pronto_saw2_base) {
			pr_err("%s: ioremap wcnss physical(saw2) failed\n",
					__func__);
			ret = -ENOMEM;
			goto fail_ioremap5;
		}
		penv->pronto_pll_base = ioremap_nocache(MSM_PRONTO_PLL_BASE,
				SZ_64);
		if (!penv->pronto_pll_base) {
			pr_err("%s: ioremap wcnss physical(pll) failed\n",
					__func__);
			ret = -ENOMEM;
			goto fail_ioremap6;
		}

		penv->wlan_tx_phy_aborts =  ioremap(MSM_PRONTO_TXP_PHY_ABORT,
					SZ_8);
		if (!penv->wlan_tx_phy_aborts) {
			ret = -ENOMEM;
			pr_err("%s: ioremap wlan TX PHY failed\n", __func__);
			goto fail_ioremap7;
		}
		penv->wlan_brdg_err_source =  ioremap(MSM_PRONTO_BRDG_ERR_SRC,
							SZ_8);
		if (!penv->wlan_brdg_err_source) {
			ret = -ENOMEM;
			pr_err("%s: ioremap wlan BRDG ERR failed\n", __func__);
			goto fail_ioremap8;
		}
		penv->wlan_tx_status = ioremap(MSM_PRONTO_TXP_STATUS, SZ_8);
		if (!penv->wlan_tx_status) {
			ret = -ENOMEM;
			pr_err("%s: ioremap wlan TX STATUS failed\n", __func__);
			goto fail_ioremap9;
		}
		penv->alarms_txctl = ioremap(MSM_PRONTO_ALARMS_TXCTL, SZ_8);
		if (!penv->alarms_txctl) {
			ret = -ENOMEM;
			pr_err("%s: ioremap alarms TXCTL failed\n", __func__);
			goto fail_ioremap10;
		}
		penv->alarms_tactl = ioremap(MSM_PRONTO_ALARMS_TACTL, SZ_8);
		if (!penv->alarms_tactl) {
			ret = -ENOMEM;
			pr_err("%s: ioremap alarms TACTL failed\n", __func__);
			goto fail_ioremap11;
		}
		penv->pronto_mcu_base = ioremap(MSM_PRONTO_MCU_BASE, SZ_1K);
		if (!penv->pronto_mcu_base) {
			ret = -ENOMEM;
			pr_err("%s: ioremap wcnss physical(mcu) failed\n",
				__func__);
			goto fail_ioremap12;
		}
	}
	penv->adc_tm_dev = qpnp_get_adc_tm(&penv->pdev->dev, "wcnss");
	if (IS_ERR(penv->adc_tm_dev)) {
		pr_err("%s:  adc get failed\n", __func__);
		penv->adc_tm_dev = NULL;
	} else {
		INIT_DELAYED_WORK(&penv->vbatt_work, wcnss_update_vbatt);
		penv->fw_vbatt_state = WCNSS_CONFIG_UNSPECIFIED;
	}

	do {
		
		penv->pil = subsystem_get(WCNSS_PIL_DEVICE);
		if (IS_ERR(penv->pil)) {
			dev_err(&pdev->dev, "Peripheral Loader failed on WCNSS.\n");
			ret = PTR_ERR(penv->pil);
			wcnss_disable_pc_add_req();
			wcnss_pronto_log_debug_regs();
		}
	} while (pil_retry++ < WCNSS_MAX_PIL_RETRY && IS_ERR(penv->pil));

	if (pil_retry >= WCNSS_MAX_PIL_RETRY) {
		if (penv->wcnss_notif_hdle)
			subsys_notif_unregister_notifier(penv->wcnss_notif_hdle,
				&wnb);
		penv->pil = NULL;
		goto fail_pil;
	}
	
	wcnss_disable_pc_remove_req();

	return 0;

fail_pil:
	if (penv->riva_ccu_base)
		iounmap(penv->riva_ccu_base);
	if (penv->pronto_mcu_base)
		iounmap(penv->pronto_mcu_base);
fail_ioremap12:
	if (penv->alarms_tactl)
		iounmap(penv->alarms_tactl);
fail_ioremap11:
	if (penv->alarms_txctl)
		iounmap(penv->alarms_txctl);
fail_ioremap10:
	if (penv->wlan_tx_status)
		iounmap(penv->wlan_tx_status);
fail_ioremap9:
	if (penv->wlan_brdg_err_source)
		iounmap(penv->wlan_brdg_err_source);
fail_ioremap8:
	if (penv->wlan_tx_phy_aborts)
		iounmap(penv->wlan_tx_phy_aborts);
fail_ioremap7:
	if (penv->pronto_pll_base)
		iounmap(penv->pronto_pll_base);
fail_ioremap6:
	if (penv->pronto_saw2_base)
		iounmap(penv->pronto_saw2_base);
fail_ioremap5:
	if (penv->fiq_reg)
		iounmap(penv->fiq_reg);
fail_ioremap4:
	if (penv->pronto_ccpu_base)
		iounmap(penv->pronto_ccpu_base);
fail_ioremap3:
	if (penv->pronto_a2xb_base)
		iounmap(penv->pronto_a2xb_base);
fail_ioremap2:
	if (penv->msm_wcnss_base)
		iounmap(penv->msm_wcnss_base);
fail_ioremap:
	wcnss_allow_suspend();
	wake_lock_destroy(&penv->wcnss_wake_lock);
fail_res:
	wcnss_wlan_power(&pdev->dev, &penv->wlan_config,
				WCNSS_WLAN_SWITCH_OFF, NULL);
fail_power:
	if (has_pronto_hw)
		wcnss_pronto_gpios_config(&pdev->dev, false);
	else
		wcnss_gpios_config(penv->gpios_5wire, false);
fail_gpio_res:
	wcnss_disable_pc_remove_req();
	penv = NULL;
	return ret;
}

static int wcnss_node_open(struct inode *inode, struct file *file)
{
	struct platform_device *pdev;
	int rc = 0;

	if (!penv)
		return -EFAULT;

	if (!penv->triggered) {
		pr_info(DEVICE " triggered by userspace\n");
		pdev = penv->pdev;
		rc = wcnss_trigger_config(pdev);
		if (rc)
			return -EFAULT;
	}

	mutex_lock(&penv->dev_lock);
	penv->user_cal_rcvd = 0;
	penv->user_cal_read = 0;
	penv->user_cal_available = false;
	penv->user_cal_data = NULL;
	penv->device_opened = 1;
	mutex_unlock(&penv->dev_lock);

	return rc;
}

static ssize_t wcnss_wlan_read(struct file *fp, char __user
			*buffer, size_t count, loff_t *position)
{
	int rc = 0;

	if (!penv || !penv->device_opened)
		return -EFAULT;

	rc = wait_event_interruptible(penv->read_wait, penv->fw_cal_rcvd
			> penv->user_cal_read || penv->fw_cal_available);

	if (rc < 0)
		return rc;

	mutex_lock(&penv->dev_lock);

	if (penv->fw_cal_available && penv->fw_cal_rcvd
			== penv->user_cal_read) {
		rc = 0;
		goto exit;
	}

	if (count > penv->fw_cal_rcvd - penv->user_cal_read)
		count = penv->fw_cal_rcvd - penv->user_cal_read;

	rc = copy_to_user(buffer, penv->fw_cal_data +
			penv->user_cal_read, count);
	if (rc == 0) {
		penv->user_cal_read += count;
		rc = count;
	}

exit:
	mutex_unlock(&penv->dev_lock);
	return rc;
}

static ssize_t wcnss_wlan_write(struct file *fp, const char __user
			*user_buffer, size_t count, loff_t *position)
{
	int rc = 0;
	size_t size = 0;

	if (!penv || !penv->device_opened || penv->user_cal_available)
		return -EFAULT;

	if (penv->user_cal_rcvd == 0 && count >= 4
			&& !penv->user_cal_data) {
		rc = copy_from_user((void *)&size, user_buffer, 4);
		if (!size || size > MAX_CALIBRATED_DATA_SIZE) {
			pr_err(DEVICE " invalid size to write %d\n", size);
			return -EFAULT;
		}

		rc += count;
		count -= 4;
		penv->user_cal_exp_size =  size;
		penv->user_cal_data = kmalloc(size, GFP_KERNEL);
		if (penv->user_cal_data == NULL) {
			pr_err(DEVICE " no memory to write\n");
			return -ENOMEM;
		}
		if (0 == count)
			goto exit;

	} else if (penv->user_cal_rcvd == 0 && count < 4)
		return -EFAULT;

	if ((UINT32_MAX - count < penv->user_cal_rcvd) ||
	     MAX_CALIBRATED_DATA_SIZE < count + penv->user_cal_rcvd) {
		pr_err(DEVICE " invalid size to write %d\n", count +
				penv->user_cal_rcvd);
		rc = -ENOMEM;
		goto exit;
	}
	rc = copy_from_user((void *)penv->user_cal_data +
			penv->user_cal_rcvd, user_buffer, count);
	if (0 == rc) {
		penv->user_cal_rcvd += count;
		rc += count;
	}
	if (penv->user_cal_rcvd == penv->user_cal_exp_size) {
		penv->user_cal_available = true;
		pr_info_ratelimited("wcnss: user cal written");
	}

exit:
	return rc;
}


static int wcnss_notif_cb(struct notifier_block *this, unsigned long code,
				void *ss_handle)
{
	pr_info("%s: wcnss notification event: %lu\n", __func__, code);

         if (code == SUBSYS_BEFORE_SHUTDOWN) {
                 penv->is_shutdown = 1;
                 wcnss_disable_pc_add_req();
                 schedule_delayed_work(&penv->wcnss_pm_qos_del_req,
                                 msecs_to_jiffies(WCNSS_PM_QOS_TIMEOUT));
         } else if (code == SUBSYS_POWERUP_FAILURE) {
                 wcnss_pronto_log_debug_regs();
                 wcnss_disable_pc_remove_req();
         } else if (SUBSYS_AFTER_POWERUP == code)
                 penv->is_shutdown = 0;

	return NOTIFY_DONE;
}

static const struct file_operations wcnss_node_fops = {
	.owner = THIS_MODULE,
	.open = wcnss_node_open,
	.read = wcnss_wlan_read,
	.write = wcnss_wlan_write,
};

static struct miscdevice wcnss_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE,
	.fops = &wcnss_node_fops,
};

static int __devinit
wcnss_wlan_probe(struct platform_device *pdev)
{
	int ret = 0;

	
	if (penv) {
		dev_err(&pdev->dev, "cannot handle multiple devices.\n");
		return -ENODEV;
	}

	
	penv = devm_kzalloc(&pdev->dev, sizeof(*penv), GFP_KERNEL);
	if (!penv) {
		dev_err(&pdev->dev, "cannot allocate device memory.\n");
		return -ENOMEM;
	}
	penv->pdev = pdev;

	
	ret = wcnss_create_sysfs(&pdev->dev);
	if (ret) {
		penv = NULL;
		return -ENOENT;
	}

	
	penv->wcnss_notif_hdle = subsys_notif_register_notifier("wcnss", &wnb);
	if (IS_ERR(penv->wcnss_notif_hdle)) {
		pr_err("wcnss: register event notification failed!\n");
		return PTR_ERR(penv->wcnss_notif_hdle);
	}

	mutex_init(&penv->dev_lock);
	mutex_init(&penv->ctrl_lock);
	mutex_init(&penv->vbat_monitor_mutex);
	init_waitqueue_head(&penv->read_wait);

	pr_info(DEVICE " probed in built-in mode\n");

	misc_register(&wcnss_usr_ctrl);

	return misc_register(&wcnss_misc);

}

static int __devexit
wcnss_wlan_remove(struct platform_device *pdev)
{
	if (penv->wcnss_notif_hdle)
		subsys_notif_unregister_notifier(penv->wcnss_notif_hdle, &wnb);
	wcnss_remove_sysfs(&pdev->dev);
	penv = NULL;
	return 0;
}


static const struct dev_pm_ops wcnss_wlan_pm_ops = {
	.suspend	= wcnss_wlan_suspend,
	.resume		= wcnss_wlan_resume,
};

#ifdef CONFIG_WCNSS_CORE_PRONTO
static struct of_device_id msm_wcnss_pronto_match[] = {
	{.compatible = "qcom,wcnss_wlan"},
	{}
};
#endif

static struct platform_driver wcnss_wlan_driver = {
	.driver = {
		.name	= DEVICE,
		.owner	= THIS_MODULE,
		.pm	= &wcnss_wlan_pm_ops,
#ifdef CONFIG_WCNSS_CORE_PRONTO
		.of_match_table = msm_wcnss_pronto_match,
#endif
	},
	.probe	= wcnss_wlan_probe,
	.remove	= __devexit_p(wcnss_wlan_remove),
};

static int __init wcnss_wlan_init(void)
{
	int ret = 0;

	platform_driver_register(&wcnss_wlan_driver);
	platform_driver_register(&wcnss_wlan_ctrl_driver);
	platform_driver_register(&wcnss_ctrl_driver);
	register_pm_notifier(&wcnss_pm_notifier);
#ifdef CONFIG_WCNSS_MEM_PRE_ALLOC
	ret = wcnss_prealloc_init();
	if (ret < 0)
		pr_err("wcnss: pre-allocation failed\n");
#endif

	return ret;
}

static void __exit wcnss_wlan_exit(void)
{
	if (penv) {
		if (penv->pil)
			subsystem_put(penv->pil);
		penv = NULL;
	}

#ifdef CONFIG_WCNSS_MEM_PRE_ALLOC
	wcnss_prealloc_deinit();
#endif
	unregister_pm_notifier(&wcnss_pm_notifier);
	platform_driver_unregister(&wcnss_ctrl_driver);
	platform_driver_unregister(&wcnss_wlan_ctrl_driver);
	platform_driver_unregister(&wcnss_wlan_driver);
}

module_init(wcnss_wlan_init);
module_exit(wcnss_wlan_exit);

MODULE_LICENSE("GPL v2");
MODULE_VERSION(VERSION);
MODULE_DESCRIPTION(DEVICE "Driver");
