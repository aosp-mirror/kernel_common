/*
 * drivers/mmc/host/sdhci-msm.c - Qualcomm MSM SDHCI Platform
 * driver source file
 *
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/gfp.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/mmc/mmc.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/mmc/cd-gpio.h>
#include <linux/dma-mapping.h>
#include <mach/gpio.h>
#include <mach/msm_bus.h>
#include <mach/mpm.h>
#include <linux/iopoll.h>
#include <linux/android_alarm.h>
#include <linux/proc_fs.h>
#include <mach/board.h>

#include "sdhci-pltfm.h"

enum sdc_mpm_pin_state {
	SDC_DAT1_DISABLE,
	SDC_DAT1_ENABLE,
	SDC_DAT1_ENWAKE,
	SDC_DAT1_DISWAKE,
};

#define SDHCI_VER_100		0x2B
#define CORE_HC_MODE		0x78
#define HC_MODE_EN		0x1
#define FF_CLK_SW_RST_DIS	(1 << 13)

#define CORE_POWER		0x0
#define CORE_SW_RST		(1 << 7)

#define CORE_PWRCTL_STATUS	0xDC
#define CORE_PWRCTL_MASK	0xE0
#define CORE_PWRCTL_CLEAR	0xE4
#define CORE_PWRCTL_CTL		0xE8

#define CORE_PWRCTL_BUS_OFF	0x01
#define CORE_PWRCTL_BUS_ON	(1 << 1)
#define CORE_PWRCTL_IO_LOW	(1 << 2)
#define CORE_PWRCTL_IO_HIGH	(1 << 3)

#define CORE_PWRCTL_BUS_SUCCESS	0x01
#define CORE_PWRCTL_BUS_FAIL	(1 << 1)
#define CORE_PWRCTL_IO_SUCCESS	(1 << 2)
#define CORE_PWRCTL_IO_FAIL	(1 << 3)

#define INT_MASK		0xF
#define MAX_PHASES		16

#define CORE_DLL_CONFIG		0x100
#define CORE_CMD_DAT_TRACK_SEL	(1 << 0)
#define CORE_DLL_EN		(1 << 16)
#define CORE_CDR_EN		(1 << 17)
#define CORE_CK_OUT_EN		(1 << 18)
#define CORE_CDR_EXT_EN		(1 << 19)
#define CORE_DLL_PDN		(1 << 29)
#define CORE_DLL_RST		(1 << 30)

#define CORE_DLL_STATUS		0x108
#define CORE_DLL_LOCK		(1 << 7)

#define CORE_VENDOR_SPEC	0x10C
#define CORE_CLK_PWRSAVE	(1 << 1)
#define CORE_HC_MCLK_SEL_DFLT	(2 << 8)
#define CORE_HC_MCLK_SEL_HS400	(3 << 8)
#define CORE_HC_MCLK_SEL_MASK	(3 << 8)
#define CORE_IO_PAD_PWR_SWITCH	(1 << 16)
#define CORE_HC_SELECT_IN_EN	(1 << 18)
#define CORE_HC_SELECT_IN_HS400	(6 << 19)
#define CORE_HC_SELECT_IN_MASK	(7 << 19)

#define CORE_VENDOR_SPEC_ADMA_ERR_ADDR0	0x114
#define CORE_VENDOR_SPEC_ADMA_ERR_ADDR1	0x118

#define CORE_CSR_CDC_CTLR_CFG0		0x130
#define CORE_SW_TRIG_FULL_CALIB		(1 << 16)
#define CORE_HW_AUTOCAL_ENA		(1 << 17)

#define CORE_CSR_CDC_CTLR_CFG1		0x134
#define CORE_CSR_CDC_CAL_TIMER_CFG0	0x138
#define CORE_TIMER_ENA			(1 << 16)

#define CORE_CSR_CDC_CAL_TIMER_CFG1	0x13C
#define CORE_CSR_CDC_REFCOUNT_CFG	0x140
#define CORE_CSR_CDC_COARSE_CAL_CFG	0x144
#define CORE_CDC_OFFSET_CFG		0x14C
#define CORE_CSR_CDC_DELAY_CFG		0x150
#define CORE_CDC_SLAVE_DDA_CFG		0x160
#define CORE_CSR_CDC_STATUS0		0x164
#define CORE_CALIBRATION_DONE		(1 << 0)

#define CORE_CDC_ERROR_CODE_MASK	0x7000000

#define CORE_CSR_CDC_GEN_CFG		0x178
#define CORE_CDC_SWITCH_BYPASS_OFF	(1 << 0)
#define CORE_CDC_SWITCH_RC_EN		(1 << 1)

#define CORE_DDR_200_CFG		0x184
#define CORE_CDC_T4_DLY_SEL		(1 << 0)
#define CORE_START_CDC_TRAFFIC		(1 << 6)

#define CORE_MCI_DATA_CTRL	0x2C
#define CORE_MCI_DPSM_ENABLE	(1 << 0)

#define CORE_TESTBUS_CONFIG	0x0CC
#define CORE_TESTBUS_ENA	(1 << 3)
#define CORE_TESTBUS_SEL2	(1 << 4)

#define CORE_MCI_VERSION	0x050
#define CORE_VERSION_310	0x10000011

#define CORE_AHB_DATA_DELAY_US  23
#define CORE_AHB_DESC_DELAY_US  8

#define CORE_SDCC_DEBUG_REG	0x124
#define CORE_DEBUG_REG_AHB_HTRANS       (3 << 12)

#define SDHCI_MSM_MAX_SEGMENTS  (1 << 13)
#define SDHCI_MSM_MMC_CLK_GATE_DELAY	200 
#define SDHCI_MSM_MMC_CLK_GATE_DELAY_EMMC	20 

#define CORE_FREQ_100MHZ	(100 * 1000 * 1000)

#define INVALID_TUNING_PHASE	-1
atomic_t emmc_reboot = ATOMIC_INIT(0);

#define sdhci_is_valid_mpm_wakeup_int(_h) ((_h)->pdata->mpm_sdiowakeup_int >= 0)
#define sdhci_is_valid_gpio_wakeup_int(_h) ((_h)->pdata->sdiowakeup_irq >= 0)

#define NUM_TUNING_PHASES		16
#define MAX_DRV_TYPES_SUPPORTED_HS200	3

static const u32 tuning_block_64[] = {
	0x00FF0FFF, 0xCCC3CCFF, 0xFFCC3CC3, 0xEFFEFFFE,
	0xDDFFDFFF, 0xFBFFFBFF, 0xFF7FFFBF, 0xEFBDF777,
	0xF0FFF0FF, 0x3CCCFC0F, 0xCFCC33CC, 0xEEFFEFFF,
	0xFDFFFDFF, 0xFFBFFFDF, 0xFFF7FFBB, 0xDE7B7FF7
};

static const u32 tuning_block_128[] = {
	0xFF00FFFF, 0x0000FFFF, 0xCCCCFFFF, 0xCCCC33CC,
	0xCC3333CC, 0xFFFFCCCC, 0xFFFFEEFF, 0xFFEEEEFF,
	0xFFDDFFFF, 0xDDDDFFFF, 0xBBFFFFFF, 0xBBFFFFFF,
	0xFFFFFFBB, 0xFFFFFF77, 0x77FF7777, 0xFFEEDDBB,
	0x00FFFFFF, 0x00FFFFFF, 0xCCFFFF00, 0xCC33CCCC,
	0x3333CCCC, 0xFFCCCCCC, 0xFFEEFFFF, 0xEEEEFFFF,
	0xDDFFFFFF, 0xDDFFFFFF, 0xFFFFFFDD, 0xFFFFFFBB,
	0xFFFFBBBB, 0xFFFF77FF, 0xFF7777FF, 0xEEDDBB77
};

static int disable_slots;
module_param(disable_slots, int, S_IRUGO|S_IWUSR);

struct sdhci_msm_reg_data {
	
	struct regulator *reg;
	
	const char *name;
	
	u32 low_vol_level;
	u32 high_vol_level;
	
	u32 lpm_uA;
	u32 hpm_uA;

	
	bool is_enabled;
	
	bool is_always_on;
	
	bool lpm_sup;
	bool set_voltage_sup;
};

struct sdhci_msm_slot_reg_data {
	
	struct sdhci_msm_reg_data *vdd_data;
	 
	struct sdhci_msm_reg_data *vdd_io_data;
};

struct sdhci_msm_gpio {
	u32 no;
	const char *name;
	bool is_enabled;
};

struct sdhci_msm_gpio_data {
	struct sdhci_msm_gpio *gpio;
	u8 size;
};

struct sdhci_msm_pad_pull {
	enum msm_tlmm_pull_tgt no;
	u32 val;
};

struct sdhci_msm_pad_pull_data {
	struct sdhci_msm_pad_pull *on;
	struct sdhci_msm_pad_pull *off;
	u8 size;
};

struct sdhci_msm_pad_drv {
	enum msm_tlmm_hdrive_tgt no;
	u32 val;
};

struct sdhci_msm_pad_drv_data {
	struct sdhci_msm_pad_drv *on;
	struct sdhci_msm_pad_drv *on_uhs;
	struct sdhci_msm_pad_drv *on_sdr104;
	struct sdhci_msm_pad_drv *off;
	u8 size;
};

struct sdhci_msm_pad_data {
	struct sdhci_msm_pad_pull_data *pull;
	struct sdhci_msm_pad_drv_data *drv;
};


struct sdhci_msm_pin_data {
	u8 is_gpio;
	bool cfg_sts;
	struct sdhci_msm_gpio_data *gpio_data;
	struct sdhci_msm_pad_data *pad_data;
};

struct sdhci_msm_bus_voting_data {
	struct msm_bus_scale_pdata *bus_pdata;
	unsigned int *bw_vecs;
	unsigned int bw_vecs_size;
};

struct sdhci_msm_pltfm_data {
	
	u32 caps;

	
	u32 caps2;

	unsigned long mmc_bus_width;
	struct sdhci_msm_slot_reg_data *vreg_data;
	bool nonremovable;
	struct sdhci_msm_pin_data *pin_data;
	u32 cpu_dma_latency_us;
	int status_gpio; 
	struct sdhci_msm_bus_voting_data *voting_data;
	u32 *sup_clk_table;
	unsigned char sup_clk_cnt;
	int slot_type;
	int disable_sdcard_uhs;
	int mpm_sdiowakeup_int;
	int sdiowakeup_irq;
	int expand_debounce;
};

struct sdhci_msm_bus_vote {
	uint32_t client_handle;
	uint32_t curr_vote;
	int min_bw_vote;
	int max_bw_vote;
	bool is_max_bw_needed;
	struct delayed_work vote_work;
	struct device_attribute max_bus_bw;
};

struct sdhci_msm_host {
	struct platform_device	*pdev;
	void __iomem *core_mem;    
	int	pwr_irq;	
	struct clk	 *clk;     
	struct clk	 *pclk;    
	struct clk	 *bus_clk; 
	struct clk	 *ff_clk; 
	struct clk	 *sleep_clk; 
	atomic_t clks_on; 
	struct sdhci_msm_pltfm_data *pdata;
	struct mmc_host  *mmc;
	struct sdhci_pltfm_data sdhci_msm_pdata;
	u32 curr_pwr_state;
	u32 curr_io_level;
	struct completion pwr_irq_completion;
	struct sdhci_msm_bus_vote msm_bus_vote;
	struct device_attribute	polling;
	u32 clk_rate; 

	struct proc_dir_entry	*bkops_proc;
	struct proc_dir_entry	*speed_class;
	struct proc_dir_entry	*sd_tray_state;
	bool tuning_done;
	bool calibration_done;
	u8 saved_tuning_phase;
	atomic_t controller_clock;
	bool is_sdiowakeup_enabled;
};

enum vdd_io_level {
	
	VDD_IO_LOW,
	
	VDD_IO_HIGH,
	VDD_IO_SET_LEVEL,
};

static char *mmc_type_str(unsigned int slot_type)
{
	switch (slot_type) {
		case MMC_TYPE_MMC:		return "MMC";
		case MMC_TYPE_SD:		return "SD";
		case MMC_TYPE_SDIO:		return "SDIO";
		case MMC_TYPE_SDIO_WIFI:	return "SDIO(WIFI)";
		case MMC_TYPE_SDIO_WIMAX:	return "SDIO(WiMAX)";
		case MMC_TYPE_SDIO_SVLTE:	return "SDIO(SVLTE)";
		case MMC_TYPE_NA:
		default:			return "Unknown type";
	}
}

static int is_mmc_platform(struct sdhci_msm_pltfm_data *pdata)
{
	if (pdata && pdata->slot_type == MMC_TYPE_MMC)
		return 1;

	return 0;
}

static int is_sd_platform(struct sdhci_msm_pltfm_data *pdata)
{
	if (pdata && pdata->slot_type == MMC_TYPE_SD)
		return 1;

	return 0;
}

int mmc_is_sd_host(struct mmc_host *mmc)
{
	struct sdhci_host *host = mmc_priv(mmc);
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_msm_host *msm_host;

	pltfm_host = sdhci_priv(host);
	msm_host = pltfm_host->priv;

	return is_sd_platform(msm_host->pdata);
}

int mmc_is_mmc_host(struct mmc_host *mmc)
{
	struct sdhci_host *host = mmc_priv(mmc);
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_msm_host *msm_host;

	pltfm_host = sdhci_priv(host);
	msm_host = pltfm_host->priv;

	return is_mmc_platform(msm_host->pdata);

}
static inline int msm_dll_poll_ck_out_en(struct sdhci_host *host,
						u8 poll)
{
	int rc = 0;
	u32 wait_cnt = 50;
	u8 ck_out_en = 0;
	struct mmc_host *mmc = host->mmc;

	
	ck_out_en = !!(readl_relaxed(host->ioaddr + CORE_DLL_CONFIG) &
			CORE_CK_OUT_EN);

	while (ck_out_en != poll) {
		if (--wait_cnt == 0) {
			pr_err("%s: %s: CK_OUT_EN bit is not %d\n",
				mmc_hostname(mmc), __func__, poll);
			rc = -ETIMEDOUT;
			goto out;
		}
		udelay(1);

		ck_out_en = !!(readl_relaxed(host->ioaddr +
				CORE_DLL_CONFIG) & CORE_CK_OUT_EN);
	}
out:
	return rc;
}

static int msm_config_cm_dll_phase(struct sdhci_host *host, u8 phase)
{
	int rc = 0;
	u8 grey_coded_phase_table[] = {0x0, 0x1, 0x3, 0x2, 0x6, 0x7, 0x5, 0x4,
					0xC, 0xD, 0xF, 0xE, 0xA, 0xB, 0x9,
					0x8};
	unsigned long flags;
	u32 config;
	struct mmc_host *mmc = host->mmc;

	pr_debug("%s: Enter %s\n", mmc_hostname(mmc), __func__);
	spin_lock_irqsave(&host->lock, flags);

	config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
	config &= ~(CORE_CDR_EN | CORE_CK_OUT_EN);
	config |= (CORE_CDR_EXT_EN | CORE_DLL_EN);
	writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);

	
	rc = msm_dll_poll_ck_out_en(host, 0);
	if (rc)
		goto err_out;

	writel_relaxed(((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
			& ~(0xF << 20))
			| (grey_coded_phase_table[phase] << 20)),
			host->ioaddr + CORE_DLL_CONFIG);

	
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
			| CORE_CK_OUT_EN), host->ioaddr + CORE_DLL_CONFIG);

	
	rc = msm_dll_poll_ck_out_en(host, 1);
	if (rc)
		goto err_out;

	config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
	config |= CORE_CDR_EN;
	config &= ~CORE_CDR_EXT_EN;
	writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);
	goto out;

err_out:
	pr_err("%s: %s: Failed to set DLL phase: %d\n",
		mmc_hostname(mmc), __func__, phase);
out:
	spin_unlock_irqrestore(&host->lock, flags);
	pr_debug("%s: Exit %s\n", mmc_hostname(mmc), __func__);
	return rc;
}


static int msm_find_most_appropriate_phase(struct sdhci_host *host,
				u8 *phase_table, u8 total_phases)
{
	int ret;
	u8 ranges[MAX_PHASES][MAX_PHASES] = { {0}, {0} };
	u8 phases_per_row[MAX_PHASES] = {0};
	int row_index = 0, col_index = 0, selected_row_index = 0, curr_max = 0;
	int i, cnt, phase_0_raw_index = 0, phase_15_raw_index = 0;
	bool phase_0_found = false, phase_15_found = false;
	struct mmc_host *mmc = host->mmc;

	pr_debug("%s: Enter %s\n", mmc_hostname(mmc), __func__);
	if (!total_phases || (total_phases > MAX_PHASES)) {
		pr_err("%s: %s: invalid argument: total_phases=%d\n",
			mmc_hostname(mmc), __func__, total_phases);
		return -EINVAL;
	}

	for (cnt = 0; cnt < total_phases; cnt++) {
		ranges[row_index][col_index] = phase_table[cnt];
		phases_per_row[row_index] += 1;
		col_index++;

		if ((cnt + 1) == total_phases) {
			continue;
		
		} else if ((phase_table[cnt] + 1) != phase_table[cnt + 1]) {
			row_index++;
			col_index = 0;
		}
	}

	if (row_index >= MAX_PHASES)
		return -EINVAL;

	
	if (!ranges[0][0]) {
		phase_0_found = true;
		phase_0_raw_index = 0;
		
		for (cnt = 1; cnt <= row_index; cnt++) {
			if (phases_per_row[cnt]) {
				for (i = 0; i < phases_per_row[cnt]; i++) {
					if (ranges[cnt][i] == 15) {
						phase_15_found = true;
						phase_15_raw_index = cnt;
						break;
					}
				}
			}
		}
	}

	
	if (phase_0_found && phase_15_found) {
		
		u8 phases_0 = phases_per_row[phase_0_raw_index];
		
		u8 phases_15 = phases_per_row[phase_15_raw_index];

		if (phases_0 + phases_15 >= MAX_PHASES)
			return -EINVAL;

		
		i = phases_15;
		for (cnt = 0; cnt < phases_0; cnt++) {
			ranges[phase_15_raw_index][i] =
				ranges[phase_0_raw_index][cnt];
			if (++i >= MAX_PHASES)
				break;
		}

		phases_per_row[phase_0_raw_index] = 0;
		phases_per_row[phase_15_raw_index] = phases_15 + phases_0;
	}

	for (cnt = 0; cnt <= row_index; cnt++) {
		if (phases_per_row[cnt] > curr_max) {
			curr_max = phases_per_row[cnt];
			selected_row_index = cnt;
		}
	}

	i = ((curr_max * 3) / 4);
	if (i)
		i--;

	ret = (int)ranges[selected_row_index][i];

	if (ret >= MAX_PHASES) {
		ret = -EINVAL;
		pr_err("%s: %s: invalid phase selected=%d\n",
			mmc_hostname(mmc), __func__, ret);
	}

	pr_debug("%s: Exit %s\n", mmc_hostname(mmc), __func__);
	return ret;
}

static inline void msm_cm_dll_set_freq(struct sdhci_host *host)
{
	u32 mclk_freq = 0;

	
	if (host->clock <= 112000000)
		mclk_freq = 0;
	else if (host->clock <= 125000000)
		mclk_freq = 1;
	else if (host->clock <= 137000000)
		mclk_freq = 2;
	else if (host->clock <= 150000000)
		mclk_freq = 3;
	else if (host->clock <= 162000000)
		mclk_freq = 4;
	else if (host->clock <= 175000000)
		mclk_freq = 5;
	else if (host->clock <= 187000000)
		mclk_freq = 6;
	else if (host->clock <= 200000000)
		mclk_freq = 7;

	writel_relaxed(((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
			& ~(7 << 24)) | (mclk_freq << 24)),
			host->ioaddr + CORE_DLL_CONFIG);
}

static int msm_init_cm_dll(struct sdhci_host *host)
{
	struct mmc_host *mmc = host->mmc;
	int rc = 0;
	unsigned long flags;
	u32 wait_cnt;
	bool prev_pwrsave, curr_pwrsave;

	pr_debug("%s: Enter %s\n", mmc_hostname(mmc), __func__);
	spin_lock_irqsave(&host->lock, flags);
	prev_pwrsave = !!(readl_relaxed(host->ioaddr + CORE_VENDOR_SPEC) &
			  CORE_CLK_PWRSAVE);
	curr_pwrsave = prev_pwrsave;
	if (prev_pwrsave) {
		writel_relaxed((readl_relaxed(host->ioaddr + CORE_VENDOR_SPEC)
				& ~CORE_CLK_PWRSAVE),
				host->ioaddr + CORE_VENDOR_SPEC);
		curr_pwrsave = false;
	}

	
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
			| CORE_DLL_RST), host->ioaddr + CORE_DLL_CONFIG);

	
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
			| CORE_DLL_PDN), host->ioaddr + CORE_DLL_CONFIG);
	msm_cm_dll_set_freq(host);

	
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
			& ~CORE_DLL_RST), host->ioaddr + CORE_DLL_CONFIG);

	
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
			& ~CORE_DLL_PDN), host->ioaddr + CORE_DLL_CONFIG);

	
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
			| CORE_DLL_EN), host->ioaddr + CORE_DLL_CONFIG);

	
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
			| CORE_CK_OUT_EN), host->ioaddr + CORE_DLL_CONFIG);

	wait_cnt = 50;
	
	while (!(readl_relaxed(host->ioaddr + CORE_DLL_STATUS) &
		CORE_DLL_LOCK)) {
		
		if (--wait_cnt == 0) {
			pr_err("%s: %s: DLL failed to LOCK\n",
				mmc_hostname(mmc), __func__);
			rc = -ETIMEDOUT;
			goto out;
		}
		
		udelay(1);
	}

out:
	
	if (prev_pwrsave ^ curr_pwrsave) {
		u32 reg = readl_relaxed(host->ioaddr + CORE_VENDOR_SPEC);

		if (prev_pwrsave)
			reg |= CORE_CLK_PWRSAVE;
		else
			reg &= ~CORE_CLK_PWRSAVE;

		writel_relaxed(reg, host->ioaddr + CORE_VENDOR_SPEC);
	}

	spin_unlock_irqrestore(&host->lock, flags);
	pr_debug("%s: Exit %s\n", mmc_hostname(mmc), __func__);
	return rc;
}

static int sdhci_msm_cdclp533_calibration(struct sdhci_host *host)
{
	u32 wait_cnt;
	int ret = 0;
	int cdc_err = 0;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;

	pr_debug("%s: Enter %s\n", mmc_hostname(host->mmc), __func__);

	ret = msm_init_cm_dll(host);
	if (ret)
		goto out;

	
	ret = msm_config_cm_dll_phase(host, msm_host->saved_tuning_phase);
	if (ret)
		goto out;

	
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
			| CORE_CMD_DAT_TRACK_SEL),
			host->ioaddr + CORE_DLL_CONFIG);

	
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_DDR_200_CFG)
			& ~CORE_CDC_T4_DLY_SEL),
			host->ioaddr + CORE_DDR_200_CFG);

	
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_CSR_CDC_GEN_CFG)
			& ~CORE_CDC_SWITCH_BYPASS_OFF),
			host->ioaddr + CORE_CSR_CDC_GEN_CFG);

	
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_CSR_CDC_GEN_CFG)
			| CORE_CDC_SWITCH_RC_EN),
			host->ioaddr + CORE_CSR_CDC_GEN_CFG);

	
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_DDR_200_CFG)
			& ~CORE_START_CDC_TRAFFIC),
			host->ioaddr + CORE_DDR_200_CFG);


	writel_relaxed(0x11800EC, host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);
	writel_relaxed(0x3011111, host->ioaddr + CORE_CSR_CDC_CTLR_CFG1);
	writel_relaxed(0x1201000, host->ioaddr + CORE_CSR_CDC_CAL_TIMER_CFG0);
	writel_relaxed(0x4, host->ioaddr + CORE_CSR_CDC_CAL_TIMER_CFG1);
	writel_relaxed(0xCB732020, host->ioaddr + CORE_CSR_CDC_REFCOUNT_CFG);
	writel_relaxed(0xB19, host->ioaddr + CORE_CSR_CDC_COARSE_CAL_CFG);
	writel_relaxed(0x3AC, host->ioaddr + CORE_CSR_CDC_DELAY_CFG);
	writel_relaxed(0x0, host->ioaddr + CORE_CDC_OFFSET_CFG);
	writel_relaxed(0x16334, host->ioaddr + CORE_CDC_SLAVE_DDA_CFG);

	

	
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_CSR_CDC_CTLR_CFG0)
			| CORE_SW_TRIG_FULL_CALIB),
			host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);

	
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_CSR_CDC_CTLR_CFG0)
			& ~CORE_SW_TRIG_FULL_CALIB),
			host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);

	
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_CSR_CDC_CTLR_CFG0)
			| CORE_HW_AUTOCAL_ENA),
			host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);

	
	writel_relaxed((readl_relaxed(host->ioaddr +
			CORE_CSR_CDC_CAL_TIMER_CFG0) | CORE_TIMER_ENA),
			host->ioaddr + CORE_CSR_CDC_CAL_TIMER_CFG0);

	mb();

	
	wait_cnt = 50;
	while (!(readl_relaxed(host->ioaddr + CORE_CSR_CDC_STATUS0)
			& CORE_CALIBRATION_DONE)) {
		
		if (--wait_cnt == 0) {
			pr_err("%s: %s: CDC Calibration was not completed\n",
				mmc_hostname(host->mmc), __func__);
			ret = -ETIMEDOUT;
			goto out;
		}
		
		udelay(1);
	}

	
	cdc_err = readl_relaxed(host->ioaddr + CORE_CSR_CDC_STATUS0)
			& CORE_CDC_ERROR_CODE_MASK;
	if (cdc_err) {
		pr_err("%s: %s: CDC Error Code %d\n",
			mmc_hostname(host->mmc), __func__, cdc_err);
		ret = -EINVAL;
		goto out;
	}

	
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_DDR_200_CFG)
			| CORE_START_CDC_TRAFFIC),
			host->ioaddr + CORE_DDR_200_CFG);
out:
	pr_debug("%s: Exit %s, ret:%d\n", mmc_hostname(host->mmc),
			__func__, ret);
	return ret;
}

static void sdhci_msm_set_mmc_drv_type(struct sdhci_host *host, u32 opcode,
		u8 drv_type)
{
	struct mmc_command cmd = {0};
	struct mmc_request mrq = {NULL};
	struct mmc_host *mmc = host->mmc;
	u8 val = ((drv_type << 4) | 2);

	cmd.opcode = MMC_SWITCH;
	cmd.arg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) |
		(EXT_CSD_HS_TIMING << 16) |
		(val << 8) |
		EXT_CSD_CMD_SET_NORMAL;
	cmd.flags = MMC_CMD_AC | MMC_RSP_R1B;
	
	cmd.cmd_timeout_ms = 1000 * 1000;

	memset(cmd.resp, 0, sizeof(cmd.resp));
	cmd.retries = 3;

	mrq.cmd = &cmd;
	cmd.data = NULL;

	mmc_wait_for_req(mmc, &mrq);
	pr_debug("%s: %s: set card drive type to %d\n",
			mmc_hostname(mmc), __func__,
			drv_type);
}

int sdhci_msm_execute_tuning(struct sdhci_host *host, u32 opcode)
{
	unsigned long flags;
	int tuning_seq_cnt = 3;
	u8 phase, *data_buf, tuned_phases[NUM_TUNING_PHASES], tuned_phase_cnt;
	const u32 *tuning_block_pattern = tuning_block_64;
	int size = sizeof(tuning_block_64); 
	int rc;
	struct mmc_host *mmc = host->mmc;
	struct mmc_ios	ios = host->mmc->ios;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	u8 drv_type = 0;
	bool drv_type_changed = false;
	struct mmc_card *card = host->mmc->card;

	if (host->clock <= CORE_FREQ_100MHZ ||
		!((ios.timing == MMC_TIMING_MMC_HS400) ||
		(ios.timing == MMC_TIMING_MMC_HS200) ||
		(ios.timing == MMC_TIMING_UHS_SDR104)))
		return 0;

	pr_debug("%s: Enter %s\n", mmc_hostname(mmc), __func__);

	
	if (msm_host->tuning_done && !msm_host->calibration_done &&
		(mmc->ios.timing == MMC_TIMING_MMC_HS400)) {
		rc = sdhci_msm_cdclp533_calibration(host);
		spin_lock_irqsave(&host->lock, flags);
		if (!rc)
			msm_host->calibration_done = true;
		spin_unlock_irqrestore(&host->lock, flags);
		goto out;
	}

	spin_lock_irqsave(&host->lock, flags);

	if (((opcode == MMC_SEND_TUNING_BLOCK_HS400) ||
		(opcode == MMC_SEND_TUNING_BLOCK_HS200)) &&
		(mmc->ios.bus_width == MMC_BUS_WIDTH_8)) {
		tuning_block_pattern = tuning_block_128;
		size = sizeof(tuning_block_128);
	}
	spin_unlock_irqrestore(&host->lock, flags);

	data_buf = kmalloc(size, GFP_KERNEL);
	if (!data_buf) {
		rc = -ENOMEM;
		goto out;
	}

retry:
	tuned_phase_cnt = 0;

	
	rc = msm_init_cm_dll(host);
	if (rc)
		goto kfree;

	phase = 0;
	do {
		struct mmc_command cmd = {0};
		struct mmc_data data = {0};
		struct mmc_request mrq = {
			.cmd = &cmd,
			.data = &data
		};
		struct scatterlist sg;

		
		rc = msm_config_cm_dll_phase(host, phase);
		if (rc)
			goto kfree;

		cmd.opcode = opcode;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;

		data.blksz = size;
		data.blocks = 1;
		data.flags = MMC_DATA_READ;
		data.timeout_ns = 1000 * 1000 * 1000; 

		data.sg = &sg;
		data.sg_len = 1;
		sg_init_one(&sg, data_buf, size);
		memset(data_buf, 0, size);
		mmc_wait_for_req(mmc, &mrq);

		if (!cmd.error && !data.error &&
			!memcmp(data_buf, tuning_block_pattern, size)) {
			
			tuned_phases[tuned_phase_cnt++] = phase;
			pr_debug("%s: %s: found *** good *** phase = %d\n",
				mmc_hostname(mmc), __func__, phase);
		} else {
			pr_debug("%s: %s: found ## bad ## phase = %d\n",
				mmc_hostname(mmc), __func__, phase);
		}
	} while (++phase < 16);

	if ((tuned_phase_cnt == NUM_TUNING_PHASES) &&
			card && mmc_card_mmc(card)) {
		pr_debug("%s: tuned phases count: %d\n", mmc_hostname(mmc),
				tuned_phase_cnt);

		
		while (++drv_type <= MAX_DRV_TYPES_SUPPORTED_HS200) {
			if (card->ext_csd.raw_drive_strength &
					(1 << drv_type)) {
				sdhci_msm_set_mmc_drv_type(host, opcode,
						drv_type);
				if (!drv_type_changed)
					drv_type_changed = true;
				goto retry;
			}
		}
	}

	
	if (drv_type_changed)
		sdhci_msm_set_mmc_drv_type(host, opcode, 0);

	if (tuned_phase_cnt) {
		rc = msm_find_most_appropriate_phase(host, tuned_phases,
							tuned_phase_cnt);
		if (rc < 0)
			goto kfree;
		else
			phase = (u8)rc;

		rc = msm_config_cm_dll_phase(host, phase);
		if (rc)
			goto kfree;
		msm_host->saved_tuning_phase = phase;
		pr_debug("%s: %s: finally setting the tuning phase to %d\n",
				mmc_hostname(mmc), __func__, phase);
	} else {
		if (--tuning_seq_cnt)
			goto retry;
		
		pr_err("%s: %s: no tuning point found\n",
			mmc_hostname(mmc), __func__);
		rc = -EIO;
	}

kfree:
	kfree(data_buf);
out:
	spin_lock_irqsave(&host->lock, flags);
	if (!rc)
		msm_host->tuning_done = true;
	spin_unlock_irqrestore(&host->lock, flags);
	pr_debug("%s: Exit %s, err(%d)\n", mmc_hostname(mmc), __func__, rc);
	return rc;
}

static int sdhci_msm_setup_gpio(struct sdhci_msm_pltfm_data *pdata, bool enable)
{
	struct sdhci_msm_gpio_data *curr;
	int i, ret = 0;

	curr = pdata->pin_data->gpio_data;
	for (i = 0; i < curr->size; i++) {
		if (!gpio_is_valid(curr->gpio[i].no)) {
			ret = -EINVAL;
			pr_err("%s: Invalid gpio = %d\n", __func__,
					curr->gpio[i].no);
			goto free_gpios;
		}
		if (enable) {
			ret = gpio_request(curr->gpio[i].no,
						curr->gpio[i].name);
			if (ret) {
				pr_err("%s: gpio_request(%d, %s) failed %d\n",
					__func__, curr->gpio[i].no,
					curr->gpio[i].name, ret);
				goto free_gpios;
			}
			curr->gpio[i].is_enabled = true;
		} else {
			gpio_free(curr->gpio[i].no);
			curr->gpio[i].is_enabled = false;
		}
	}
	return ret;

free_gpios:
	for (i--; i >= 0; i--) {
		gpio_free(curr->gpio[i].no);
		curr->gpio[i].is_enabled = false;
	}
	return ret;
}

static int sdhci_msm_setup_pad(struct sdhci_msm_pltfm_data *pdata, bool enable)
{
	struct sdhci_msm_pad_data *curr;
	int i;

	curr = pdata->pin_data->pad_data;
	for (i = 0; i < curr->drv->size; i++) {
		if (enable)
			msm_tlmm_set_hdrive(curr->drv->on[i].no,
				curr->drv->on[i].val);
		else
			msm_tlmm_set_hdrive(curr->drv->off[i].no,
				curr->drv->off[i].val);
	}

	for (i = 0; i < curr->pull->size; i++) {
		if (enable)
			msm_tlmm_set_pull(curr->pull->on[i].no,
				curr->pull->on[i].val);
		else
			msm_tlmm_set_pull(curr->pull->off[i].no,
				curr->pull->off[i].val);
	}

	return 0;
}

static int sdhci_msm_setup_hifreq_pad(struct sdhci_msm_pltfm_data *pdata,
				struct sdhci_host *host)
{
	struct sdhci_msm_pad_data *curr;
	struct mmc_ios curr_ios = host->mmc->ios;
	int i;

	curr = pdata->pin_data->pad_data;
	if (curr_ios.timing == MMC_TIMING_UHS_SDR104) {
		for (i = 0; i < curr->drv->size; i++) {
			if (!curr->drv->on_sdr104[i].no || !curr->drv->on_sdr104[i].val)
				break;
			msm_tlmm_set_hdrive(curr->drv->on_sdr104[i].no,
				curr->drv->on_sdr104[i].val);
		}
	}
	else if (curr_ios.timing == MMC_TIMING_UHS_SDR12 ||
		curr_ios.timing == MMC_TIMING_UHS_SDR25 ||
		curr_ios.timing == MMC_TIMING_UHS_SDR50 ||
		curr_ios.timing == MMC_TIMING_UHS_DDR50) {
		for (i = 0; i < curr->drv->size; i++) {
			if (!curr->drv->on_uhs[i].no || !curr->drv->on_uhs[i].val)
				break;
			msm_tlmm_set_hdrive(curr->drv->on_uhs[i].no,
				curr->drv->on_uhs[i].val);
		}
	}
	return 0;
}

static int sdhci_msm_setup_pins(struct sdhci_msm_pltfm_data *pdata, bool enable)
{
	int ret = 0;

	if (!pdata->pin_data || (pdata->pin_data->cfg_sts == enable))
		return 0;
	if (pdata->pin_data->is_gpio)
		ret = sdhci_msm_setup_gpio(pdata, enable);
	else
		ret = sdhci_msm_setup_pad(pdata, enable);

	if (!ret)
		pdata->pin_data->cfg_sts = enable;
	return ret;
}

static int sdhci_msm_setup_hifreq_pins(struct sdhci_msm_pltfm_data *pdata,
				struct sdhci_host *host)
{
	int ret = 0;

	if (!pdata->pin_data)
		return 0;
	if (!pdata->pin_data->is_gpio)
		ret = sdhci_msm_setup_hifreq_pad(pdata, host);

	return ret;
}

static int sdhci_msm_dt_get_array(struct device *dev, const char *prop_name,
				 u32 **out, int *len, u32 size)
{
	int ret = 0;
	struct device_node *np = dev->of_node;
	size_t sz;
	u32 *arr = NULL;

	if (!of_get_property(np, prop_name, len)) {
		ret = -EINVAL;
		goto out;
	}
	sz = *len = *len / sizeof(*arr);
	if (sz <= 0 || (size > 0 && (sz > size))) {
		dev_err(dev, "%s invalid size\n", prop_name);
		ret = -EINVAL;
		goto out;
	}

	arr = devm_kzalloc(dev, sz * sizeof(*arr), GFP_KERNEL);
	if (!arr) {
		dev_err(dev, "%s failed allocating memory\n", prop_name);
		ret = -ENOMEM;
		goto out;
	}

	ret = of_property_read_u32_array(np, prop_name, arr, sz);
	if (ret < 0) {
		dev_err(dev, "%s failed reading array %d\n", prop_name, ret);
		goto out;
	}
	*out = arr;
out:
	if (ret)
		*len = 0;
	return ret;
}

#define MAX_PROP_SIZE 32
static int sdhci_msm_dt_parse_vreg_info(struct device *dev,
		struct sdhci_msm_reg_data **vreg_data, const char *vreg_name)
{
	int len, ret = 0;
	const __be32 *prop;
	char prop_name[MAX_PROP_SIZE];
	struct sdhci_msm_reg_data *vreg;
	struct device_node *np = dev->of_node;

	snprintf(prop_name, MAX_PROP_SIZE, "%s-supply", vreg_name);
	if (!of_parse_phandle(np, prop_name, 0)) {
		dev_info(dev, "No vreg data found for %s\n", vreg_name);
		return ret;
	}

	vreg = devm_kzalloc(dev, sizeof(*vreg), GFP_KERNEL);
	if (!vreg) {
		dev_err(dev, "No memory for vreg: %s\n", vreg_name);
		ret = -ENOMEM;
		return ret;
	}

	vreg->name = vreg_name;

	snprintf(prop_name, MAX_PROP_SIZE,
			"qcom,%s-always-on", vreg_name);
	if (of_get_property(np, prop_name, NULL))
		vreg->is_always_on = true;

	snprintf(prop_name, MAX_PROP_SIZE,
			"qcom,%s-lpm-sup", vreg_name);
	if (of_get_property(np, prop_name, NULL))
		vreg->lpm_sup = true;

	snprintf(prop_name, MAX_PROP_SIZE,
			"qcom,%s-voltage-level", vreg_name);
	prop = of_get_property(np, prop_name, &len);
	if (!prop || (len != (2 * sizeof(__be32)))) {
		dev_warn(dev, "%s %s property\n",
			prop ? "invalid format" : "no", prop_name);
	} else {
		vreg->low_vol_level = be32_to_cpup(&prop[0]);
		vreg->high_vol_level = be32_to_cpup(&prop[1]);
	}

	snprintf(prop_name, MAX_PROP_SIZE,
			"qcom,%s-current-level", vreg_name);
	prop = of_get_property(np, prop_name, &len);
	if (!prop || (len != (2 * sizeof(__be32)))) {
		dev_warn(dev, "%s %s property\n",
			prop ? "invalid format" : "no", prop_name);
	} else {
		vreg->lpm_uA = be32_to_cpup(&prop[0]);
		vreg->hpm_uA = be32_to_cpup(&prop[1]);
	}

	*vreg_data = vreg;
	dev_dbg(dev, "%s: %s %s vol=[%d %d]uV, curr=[%d %d]uA\n",
		vreg->name, vreg->is_always_on ? "always_on," : "",
		vreg->lpm_sup ? "lpm_sup," : "", vreg->low_vol_level,
		vreg->high_vol_level, vreg->lpm_uA, vreg->hpm_uA);

	return ret;
}

static int sdhci_msm_dt_get_pad_pull_info(struct device *dev, int id,
		struct sdhci_msm_pad_pull_data **pad_pull_data)
{
	int ret = 0, base = 0, len, i;
	u32 *tmp;
	struct sdhci_msm_pad_pull_data *pull_data;
	struct sdhci_msm_pad_pull *pull;

	switch (id) {
	case 1:
		base = TLMM_PULL_SDC1_CLK;
		break;
	case 2:
		base = TLMM_PULL_SDC2_CLK;
		break;
	case 3:
		base = TLMM_PULL_SDC3_CLK;
		break;
	case 4:
		base = TLMM_PULL_SDC4_CLK;
		break;
	default:
		dev_err(dev, "%s: Invalid slot id\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	pull_data = devm_kzalloc(dev, sizeof(struct sdhci_msm_pad_pull_data),
			GFP_KERNEL);
	if (!pull_data) {
		dev_err(dev, "No memory for msm_mmc_pad_pull_data\n");
		ret = -ENOMEM;
		goto out;
	}
	pull_data->size = 4; 

	
	pull = devm_kzalloc(dev, 2 * pull_data->size *\
			sizeof(struct sdhci_msm_pad_pull), GFP_KERNEL);
	if (!pull) {
		dev_err(dev, "No memory for msm_mmc_pad_pull\n");
		ret = -ENOMEM;
		goto out;
	}
	pull_data->on = pull;
	pull_data->off = pull + pull_data->size;

	ret = sdhci_msm_dt_get_array(dev, "qcom,pad-pull-on",
			&tmp, &len, pull_data->size);
	if (ret)
		goto out;

	for (i = 0; i < len; i++) {
		pull_data->on[i].no = base + i;
		pull_data->on[i].val = tmp[i];
		dev_dbg(dev, "%s: val[%d]=0x%x\n", __func__,
				i, pull_data->on[i].val);
	}

	ret = sdhci_msm_dt_get_array(dev, "qcom,pad-pull-off",
			&tmp, &len, pull_data->size);
	if (ret)
		goto out;

	for (i = 0; i < len; i++) {
		pull_data->off[i].no = base + i;
		pull_data->off[i].val = tmp[i];
		dev_dbg(dev, "%s: val[%d]=0x%x\n", __func__,
				i, pull_data->off[i].val);
	}

	*pad_pull_data = pull_data;
out:
	return ret;
}

static int sdhci_msm_dt_get_pad_drv_info(struct device *dev, int id,
		struct sdhci_msm_pad_drv_data **pad_drv_data)
{
	int ret = 0, base = 0, len, i;
	u32 *tmp;
	struct sdhci_msm_pad_drv_data *drv_data;
	struct sdhci_msm_pad_drv *drv;

	switch (id) {
	case 1:
		base = TLMM_HDRV_SDC1_CLK;
		break;
	case 2:
		base = TLMM_HDRV_SDC2_CLK;
		break;
	case 3:
		base = TLMM_HDRV_SDC3_CLK;
		break;
	case 4:
		base = TLMM_HDRV_SDC4_CLK;
		break;
	default:
		dev_err(dev, "%s: Invalid slot id\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	drv_data = devm_kzalloc(dev, sizeof(struct sdhci_msm_pad_drv_data),
			GFP_KERNEL);
	if (!drv_data) {
		dev_err(dev, "No memory for msm_mmc_pad_drv_data\n");
		ret = -ENOMEM;
		goto out;
	}
	drv_data->size = 3; 

	
	drv = devm_kzalloc(dev, 3 * drv_data->size *\
			sizeof(struct sdhci_msm_pad_drv), GFP_KERNEL);
	if (!drv) {
		dev_err(dev, "No memory msm_mmc_pad_drv\n");
		ret = -ENOMEM;
		goto out;
	}
	drv_data->on = drv;
	drv_data->on_uhs = drv + drv_data->size;
	drv_data->on_sdr104 = drv + drv_data->size * 2;
	drv_data->off = drv + drv_data->size * 3;

	ret = sdhci_msm_dt_get_array(dev, "qcom,pad-drv-on",
			&tmp, &len, drv_data->size);
	if (ret)
		goto out;

	for (i = 0; i < len; i++) {
		drv_data->on[i].no = base + i;
		drv_data->on[i].val = tmp[i];
		dev_dbg(dev, "%s: val[%d]=0x%x\n", __func__,
				i, drv_data->on[i].val);
	}

	ret = sdhci_msm_dt_get_array(dev, "qcom,pad-drv-on-uhs",
			&tmp, &len, drv_data->size);
	if (!ret) {
		for (i = 0; i < len; i++) {
			drv_data->on_uhs[i].no = base + i;
			drv_data->on_uhs[i].val = tmp[i];
			dev_dbg(dev, "%s: val[%d]=0x%x\n", __func__,
					i, drv_data->on_uhs[i].val);
		}
	} else {
		memset(drv_data->on_uhs, 0, drv_data->size * sizeof(struct sdhci_msm_pad_drv));
		dev_dbg(dev, "%s: can`t find UHS-I setting\n", __func__);
	}

	ret = sdhci_msm_dt_get_array(dev, "qcom,pad-drv-on-sdr104",
			&tmp, &len, drv_data->size);
	if (!ret) {
		for (i = 0; i < len; i++) {
			drv_data->on_sdr104[i].no = base + i;
			drv_data->on_sdr104[i].val = tmp[i];
			dev_dbg(dev, "%s: val[%d]=0x%x\n", __func__,
					i, drv_data->on_sdr104[i].val);
		}
	} else {
		memset(drv_data->on_sdr104, 0, drv_data->size * sizeof(struct sdhci_msm_pad_drv));
		dev_dbg(dev, "%s: can`t find sdr104 setting\n", __func__);
	}

	ret = sdhci_msm_dt_get_array(dev, "qcom,pad-drv-off",
			&tmp, &len, drv_data->size);
	if (ret)
		goto out;

	for (i = 0; i < len; i++) {
		drv_data->off[i].no = base + i;
		drv_data->off[i].val = tmp[i];
		dev_dbg(dev, "%s: val[%d]=0x%x\n", __func__,
				i, drv_data->off[i].val);
	}

	*pad_drv_data = drv_data;
out:
	return ret;
}

#define GPIO_NAME_MAX_LEN 32
static int sdhci_msm_dt_parse_gpio_info(struct device *dev,
		struct sdhci_msm_pltfm_data *pdata)
{
	int ret = 0, id = 0, cnt, i;
	struct sdhci_msm_pin_data *pin_data;
	struct device_node *np = dev->of_node;

	pin_data = devm_kzalloc(dev, sizeof(*pin_data), GFP_KERNEL);
	if (!pin_data) {
		dev_err(dev, "No memory for pin_data\n");
		ret = -ENOMEM;
		goto out;
	}

	cnt = of_gpio_count(np);
	if (cnt > 0) {
		pin_data->is_gpio = true;
		pin_data->gpio_data = devm_kzalloc(dev,
				sizeof(struct sdhci_msm_gpio_data), GFP_KERNEL);
		if (!pin_data->gpio_data) {
			dev_err(dev, "No memory for gpio_data\n");
			ret = -ENOMEM;
			goto out;
		}
		pin_data->gpio_data->size = cnt;
		pin_data->gpio_data->gpio = devm_kzalloc(dev, cnt *
				sizeof(struct sdhci_msm_gpio), GFP_KERNEL);

		if (!pin_data->gpio_data->gpio) {
			dev_err(dev, "No memory for gpio\n");
			ret = -ENOMEM;
			goto out;
		}
		for (i = 0; i < cnt; i++) {
			const char *name = NULL;
			char result[GPIO_NAME_MAX_LEN];
			pin_data->gpio_data->gpio[i].no = of_get_gpio(np, i);
			of_property_read_string_index(np,
					"qcom,gpio-names", i, &name);

			snprintf(result, GPIO_NAME_MAX_LEN, "%s-%s",
					dev_name(dev), name ? name : "?");
			pin_data->gpio_data->gpio[i].name = result;
			dev_dbg(dev, "%s: gpio[%s] = %d\n", __func__,
				pin_data->gpio_data->gpio[i].name,
				pin_data->gpio_data->gpio[i].no);
		}
	} else {
		pin_data->pad_data =
			devm_kzalloc(dev,
				     sizeof(struct sdhci_msm_pad_data),
				     GFP_KERNEL);
		if (!pin_data->pad_data) {
			dev_err(dev,
				"No memory for pin_data->pad_data\n");
			ret = -ENOMEM;
			goto out;
		}

		ret = of_alias_get_id(np, "sdhc");
		if (ret < 0) {
			dev_err(dev, "Failed to get slot index %d\n", ret);
			goto out;
		}
		id = ret;

		ret = sdhci_msm_dt_get_pad_pull_info(
			dev, id, &pin_data->pad_data->pull);
		if (ret)
			goto out;
		ret = sdhci_msm_dt_get_pad_drv_info(
			dev, id, &pin_data->pad_data->drv);
		if (ret)
			goto out;

	}
	pdata->pin_data = pin_data;
out:
	if (ret)
		dev_err(dev, "%s failed with err %d\n", __func__, ret);
	return ret;
}

static struct sdhci_msm_pltfm_data *sdhci_msm_populate_pdata(struct device *dev)
{
	struct sdhci_msm_pltfm_data *pdata = NULL;
	struct device_node *np = dev->of_node;
	u32 bus_width = 0;
	u32 cpu_dma_latency;
	int len, i, mpm_int;
	int clk_table_len;
	u32 *clk_table = NULL;
	enum of_gpio_flags flags = OF_GPIO_ACTIVE_LOW;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "failed to allocate memory for platform data\n");
		goto out;
	}

	pdata->status_gpio = of_get_named_gpio_flags(np, "cd-gpios", 0, &flags);
	if (gpio_is_valid(pdata->status_gpio) & !(flags & OF_GPIO_ACTIVE_LOW))
		pdata->caps2 |= MMC_CAP2_CD_ACTIVE_HIGH;

	if (of_property_read_u32(np, "htc,slot-type", &pdata->slot_type))
		pdata->slot_type = MMC_TYPE_NA;

	of_property_read_u32(np, "qcom,bus-width", &bus_width);
	if (bus_width == 8)
		pdata->mmc_bus_width = MMC_CAP_8_BIT_DATA;
	else if (bus_width == 4)
		pdata->mmc_bus_width = MMC_CAP_4_BIT_DATA;
	else {
		dev_notice(dev, "invalid bus-width, default to 1-bit mode\n");
		pdata->mmc_bus_width = 0;
	}

	if (!of_property_read_u32(np, "qcom,cpu-dma-latency-us",
				&cpu_dma_latency))
		pdata->cpu_dma_latency_us = cpu_dma_latency;

	if (sdhci_msm_dt_get_array(dev, "qcom,clk-rates",
			&clk_table, &clk_table_len, 0)) {
		dev_err(dev, "failed parsing supported clock rates\n");
		goto out;
	}
	if (!clk_table || !clk_table_len) {
		dev_err(dev, "Invalid clock table\n");
		goto out;
	}
	pdata->sup_clk_table = clk_table;
	pdata->sup_clk_cnt = clk_table_len;
	pdata->vreg_data = devm_kzalloc(dev, sizeof(struct
						    sdhci_msm_slot_reg_data),
					GFP_KERNEL);
	if (!pdata->vreg_data) {
		dev_err(dev, "failed to allocate memory for vreg data\n");
		goto out;
	}

	if (sdhci_msm_dt_parse_vreg_info(dev, &pdata->vreg_data->vdd_data,
					 "vdd")) {
		dev_err(dev, "failed parsing vdd data\n");
		goto out;
	}
	if (sdhci_msm_dt_parse_vreg_info(dev,
					 &pdata->vreg_data->vdd_io_data,
					 "vdd-io")) {
		dev_err(dev, "failed parsing vdd-io data\n");
		goto out;
	}

	if (sdhci_msm_dt_parse_gpio_info(dev, pdata)) {
		dev_err(dev, "failed parsing gpio data\n");
		goto out;
	}

	len = of_property_count_strings(np, "qcom,bus-speed-mode");

	for (i = 0; i < len; i++) {
		const char *name = NULL;

		of_property_read_string_index(np,
			"qcom,bus-speed-mode", i, &name);
		if (!name)
			continue;

		dev_notice(dev, "bus-speed-mode: %s\n", name);
		if (!strncmp(name, "HS400_1p8v", sizeof("HS400_1p8v")))
			pdata->caps2 |= MMC_CAP2_HS400_1_8V;
		else if (!strncmp(name, "HS400_1p2v", sizeof("HS400_1p2v")))
			pdata->caps2 |= MMC_CAP2_HS400_1_2V;
		else if (!strncmp(name, "HS200_1p8v", sizeof("HS200_1p8v")))
			pdata->caps2 |= MMC_CAP2_HS200_1_8V_SDR;
		else if (!strncmp(name, "HS200_1p2v", sizeof("HS200_1p2v")))
			pdata->caps2 |= MMC_CAP2_HS200_1_2V_SDR;
		else if (!strncmp(name, "DDR_1p8v", sizeof("DDR_1p8v")))
			pdata->caps |= MMC_CAP_1_8V_DDR
						| MMC_CAP_UHS_DDR50;
		else if (!strncmp(name, "DDR_1p2v", sizeof("DDR_1p2v")))
			pdata->caps |= MMC_CAP_1_2V_DDR
						| MMC_CAP_UHS_DDR50;
	}

	if (of_get_property(np, "qcom,nonremovable", NULL))
		pdata->nonremovable = true;

	if (of_get_property(np, "htc,bkops_support", NULL))
		pdata->caps2 |= MMC_CAP2_INIT_BKOPS;

	if (of_get_property(np, "htc,packed_cmd_support", NULL)) {
		dev_info(dev, " parsed htc,packed_cmd_support\n");
		pdata->caps2 |= MMC_CAP2_PACKED_WR;
		pdata->caps2 |= MMC_CAP2_PACKED_WR_CONTROL;
	}

	if (of_get_property(np, "htc,pon_support", NULL))
		pdata->caps2 |= MMC_CAP2_POWEROFF_NOTIFY;

	if (of_get_property(np, "htc,disable_sdcard_uhs", NULL))
		pdata->disable_sdcard_uhs = 1;

	if (!of_property_read_u32(np, "qcom,dat1-mpm-int",
				  &mpm_int))
		pdata->mpm_sdiowakeup_int = mpm_int;
	else
		pdata->mpm_sdiowakeup_int = -1;

	if (of_property_read_u32(np, "htc,expand-debounce", &pdata->expand_debounce))
		pdata->expand_debounce = 0;

	return pdata;
out:
	return NULL;
}

static unsigned int sdhci_get_bw_required(struct sdhci_host *host,
					struct mmc_ios *ios)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;

	unsigned int bw;

	bw = msm_host->clk_rate;
	if (ios->bus_width == MMC_BUS_WIDTH_4)
		bw /= 2;
	else if (ios->bus_width == MMC_BUS_WIDTH_1)
		bw /= 8;

	return bw;
}

static int sdhci_msm_bus_get_vote_for_bw(struct sdhci_msm_host *host,
					   unsigned int bw)
{
	unsigned int *table = host->pdata->voting_data->bw_vecs;
	unsigned int size = host->pdata->voting_data->bw_vecs_size;
	int i;

	if (host->msm_bus_vote.is_max_bw_needed && bw)
		return host->msm_bus_vote.max_bw_vote;

	for (i = 0; i < size; i++) {
		if (bw <= table[i])
			break;
	}

	if (i && (i == size))
		i--;

	return i;
}

static inline int sdhci_msm_bus_set_vote(struct sdhci_msm_host *msm_host,
					     int vote,
					     unsigned long flags)
{
	struct sdhci_host *host =  platform_get_drvdata(msm_host->pdev);
	int rc = 0;

	if (vote != msm_host->msm_bus_vote.curr_vote) {
		spin_unlock_irqrestore(&host->lock, flags);
		rc = msm_bus_scale_client_update_request(
				msm_host->msm_bus_vote.client_handle, vote);
		spin_lock_irqsave(&host->lock, flags);
		if (rc) {
			pr_err("%s: msm_bus_scale_client_update_request() failed: bus_client_handle=0x%x, vote=%d, err=%d\n",
				mmc_hostname(host->mmc),
				msm_host->msm_bus_vote.client_handle, vote, rc);
			goto out;
		}
		msm_host->msm_bus_vote.curr_vote = vote;
	}
out:
	return rc;
}

static void sdhci_msm_bus_work(struct work_struct *work)
{
	struct sdhci_msm_host *msm_host;
	struct sdhci_host *host;
	unsigned long flags;

	msm_host = container_of(work, struct sdhci_msm_host,
				msm_bus_vote.vote_work.work);
	host =  platform_get_drvdata(msm_host->pdev);

	if (!msm_host->msm_bus_vote.client_handle)
		return;

	spin_lock_irqsave(&host->lock, flags);
	
	if (!host->mrq) {
		sdhci_msm_bus_set_vote(msm_host,
			msm_host->msm_bus_vote.min_bw_vote, flags);
	} else
		pr_warning("%s: %s: Transfer in progress. skipping bus voting to 0 bandwidth\n",
			   mmc_hostname(host->mmc), __func__);
	spin_unlock_irqrestore(&host->lock, flags);
}

static void sdhci_msm_bus_cancel_work_and_set_vote(struct sdhci_host *host,
						unsigned int bw)
{
	int vote;
	unsigned long flags;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;

	cancel_delayed_work_sync(&msm_host->msm_bus_vote.vote_work);
	spin_lock_irqsave(&host->lock, flags);
	vote = sdhci_msm_bus_get_vote_for_bw(msm_host, bw);
	sdhci_msm_bus_set_vote(msm_host, vote, flags);
	spin_unlock_irqrestore(&host->lock, flags);
}

#define MSM_MMC_BUS_VOTING_DELAY	200 

static void sdhci_msm_bus_queue_work(struct sdhci_host *host)
{
	unsigned long flags;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;

	spin_lock_irqsave(&host->lock, flags);
	if (msm_host->msm_bus_vote.min_bw_vote !=
		msm_host->msm_bus_vote.curr_vote)
		queue_delayed_work(system_nrt_wq,
				   &msm_host->msm_bus_vote.vote_work,
				   msecs_to_jiffies(MSM_MMC_BUS_VOTING_DELAY));
	spin_unlock_irqrestore(&host->lock, flags);
}

static int sdhci_msm_bus_register(struct sdhci_msm_host *host,
				struct platform_device *pdev)
{
	int rc = 0;
	struct msm_bus_scale_pdata *bus_pdata;

	struct sdhci_msm_bus_voting_data *data;
	struct device *dev = &pdev->dev;

	data = devm_kzalloc(dev,
		sizeof(struct sdhci_msm_bus_voting_data), GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev,
			"%s: failed to allocate memory\n", __func__);
		rc = -ENOMEM;
		goto out;
	}
	data->bus_pdata = msm_bus_cl_get_pdata(pdev);
	if (data->bus_pdata) {
		rc = sdhci_msm_dt_get_array(dev, "qcom,bus-bw-vectors-bps",
				&data->bw_vecs, &data->bw_vecs_size, 0);
		if (rc) {
			dev_err(&pdev->dev,
				"%s: Failed to get bus-bw-vectors-bps\n",
				__func__);
			goto out;
		}
		host->pdata->voting_data = data;
	}
	if (host->pdata->voting_data &&
		host->pdata->voting_data->bus_pdata &&
		host->pdata->voting_data->bw_vecs &&
		host->pdata->voting_data->bw_vecs_size) {

		bus_pdata = host->pdata->voting_data->bus_pdata;
		host->msm_bus_vote.client_handle =
				msm_bus_scale_register_client(bus_pdata);
		if (!host->msm_bus_vote.client_handle) {
			dev_err(&pdev->dev, "msm_bus_scale_register_client()\n");
			rc = -EFAULT;
			goto out;
		}
		
		host->msm_bus_vote.min_bw_vote =
				sdhci_msm_bus_get_vote_for_bw(host, 0);
		host->msm_bus_vote.max_bw_vote =
				sdhci_msm_bus_get_vote_for_bw(host, UINT_MAX);
	} else {
		devm_kfree(dev, data);
	}

out:
	return rc;
}

static void sdhci_msm_bus_unregister(struct sdhci_msm_host *host)
{
	if (host->msm_bus_vote.client_handle)
		msm_bus_scale_unregister_client(
			host->msm_bus_vote.client_handle);
}

static void sdhci_msm_bus_voting(struct sdhci_host *host, u32 enable)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	struct mmc_ios *ios = &host->mmc->ios;
	unsigned int bw;

	if (!msm_host->msm_bus_vote.client_handle)
		return;

	bw = sdhci_get_bw_required(host, ios);
	if (enable) {
		sdhci_msm_bus_cancel_work_and_set_vote(host, bw);
	} else {
		if (host->mmc->clkgate_delay)
			sdhci_msm_bus_cancel_work_and_set_vote(host, 0);
		else
			sdhci_msm_bus_queue_work(host);
	}
}

static int sdhci_msm_vreg_init_reg(struct device *dev,
					struct sdhci_msm_reg_data *vreg)
{
	int ret = 0;

	
	if (vreg->reg)
		goto out;

	
	vreg->reg = devm_regulator_get(dev, vreg->name);
	if (IS_ERR(vreg->reg)) {
		ret = PTR_ERR(vreg->reg);
		pr_err("%s: devm_regulator_get(%s) failed. ret=%d\n",
			__func__, vreg->name, ret);
		goto out;
	}

	if (regulator_count_voltages(vreg->reg) > 0) {
		vreg->set_voltage_sup = true;
		
		if (!vreg->high_vol_level || !vreg->hpm_uA) {
			pr_err("%s: %s invalid constraints specified\n",
			       __func__, vreg->name);
			ret = -EINVAL;
		}
	}

out:
	return ret;
}

static void sdhci_msm_vreg_deinit_reg(struct sdhci_msm_reg_data *vreg)
{
	if (vreg->reg)
		devm_regulator_put(vreg->reg);
}

static int sdhci_msm_vreg_set_optimum_mode(struct sdhci_msm_reg_data
						  *vreg, int uA_load)
{
	int ret = 0;

	if (vreg->set_voltage_sup) {
		ret = regulator_set_optimum_mode(vreg->reg, uA_load);
		if (ret < 0)
			pr_err("%s: regulator_set_optimum_mode(reg=%s,uA_load=%d) failed. ret=%d\n",
			       __func__, vreg->name, uA_load, ret);
		else
			ret = 0;
	}
	return ret;
}

static int sdhci_msm_vreg_set_voltage(struct sdhci_msm_reg_data *vreg,
					int min_uV, int max_uV)
{
	int ret = 0;
	if (vreg->set_voltage_sup) {
		ret = regulator_set_voltage(vreg->reg, min_uV, max_uV);
		if (ret) {
			pr_err("%s: regulator_set_voltage(%s)failed. min_uV=%d,max_uV=%d,ret=%d\n",
			       __func__, vreg->name, min_uV, max_uV, ret);
		}
	}

	return ret;
}

static int sdhci_msm_vreg_enable(struct sdhci_msm_reg_data *vreg)
{
	int ret = 0;

	
	ret = sdhci_msm_vreg_set_optimum_mode(vreg, vreg->hpm_uA);
	if (ret < 0)
		return ret;

	if (!vreg->is_enabled) {
		
		ret = sdhci_msm_vreg_set_voltage(vreg, vreg->high_vol_level,
						vreg->high_vol_level);
		if (ret)
			return ret;
	}
	ret = regulator_enable(vreg->reg);
	if (ret) {
		pr_err("%s: regulator_enable(%s) failed. ret=%d\n",
				__func__, vreg->name, ret);
		return ret;
	}
	vreg->is_enabled = true;
	return ret;
}

static int sdhci_msm_vreg_disable(struct sdhci_msm_reg_data *vreg)
{
	int ret = 0;

	
	if (vreg->is_enabled && !vreg->is_always_on) {
		ret = regulator_disable(vreg->reg);
		if (ret) {
			pr_err("%s: regulator_disable(%s) failed. ret=%d\n",
				__func__, vreg->name, ret);
			goto out;
		}
		vreg->is_enabled = false;

		ret = sdhci_msm_vreg_set_optimum_mode(vreg, 0);
		if (ret < 0)
			goto out;

		
		ret = sdhci_msm_vreg_set_voltage(vreg, 0, vreg->high_vol_level);
		if (ret)
			goto out;
	} else if (vreg->is_enabled && vreg->is_always_on) {
		if (vreg->lpm_sup) {
			
			ret = sdhci_msm_vreg_set_optimum_mode(vreg,
							      vreg->lpm_uA);
			if (ret < 0)
				goto out;
		}
	}
out:
	return ret;
}

static int sdhci_msm_setup_vreg(struct sdhci_msm_pltfm_data *pdata,
			bool enable, bool is_init)
{
	int ret = 0, i;
	struct sdhci_msm_slot_reg_data *curr_slot;
	struct sdhci_msm_reg_data *vreg_table[2];

	curr_slot = pdata->vreg_data;
	if (!curr_slot) {
		pr_debug("%s: vreg info unavailable,assuming the slot is powered by always on domain\n",
			 __func__);
		goto out;
	}

	vreg_table[0] = curr_slot->vdd_data;
	vreg_table[1] = curr_slot->vdd_io_data;

	if(is_sd_platform(pdata) && (enable ^ vreg_table[0]->is_enabled)) {
		pr_info("%s : %s slot power\n", mmc_type_str(pdata->slot_type),
			enable ? "Enabling" : "Disabling");
	}

	for (i = 0; i < ARRAY_SIZE(vreg_table); i++) {
		if (vreg_table[i]) {
			if (enable)
				ret = sdhci_msm_vreg_enable(vreg_table[i]);
			else
				ret = sdhci_msm_vreg_disable(vreg_table[i]);
			if (ret)
				goto out;
		}
	}
out:
	return ret;
}

static int sdhci_msm_vreg_reset(struct sdhci_msm_pltfm_data *pdata)
{
	int ret;

	ret = sdhci_msm_setup_vreg(pdata, 1, true);
	if (ret)
		return ret;
	ret = sdhci_msm_setup_vreg(pdata, 0, true);
	return ret;
}

static int sdhci_msm_vreg_init(struct device *dev,
				struct sdhci_msm_pltfm_data *pdata,
				bool is_init)
{
	int ret = 0;
	struct sdhci_msm_slot_reg_data *curr_slot;
	struct sdhci_msm_reg_data *curr_vdd_reg, *curr_vdd_io_reg;

	curr_slot = pdata->vreg_data;
	if (!curr_slot)
		goto out;

	curr_vdd_reg = curr_slot->vdd_data;
	curr_vdd_io_reg = curr_slot->vdd_io_data;

	if (!is_init)
		
		goto vdd_io_reg_deinit;

	if (curr_vdd_reg) {
		ret = sdhci_msm_vreg_init_reg(dev, curr_vdd_reg);
		if (ret)
			goto out;
	}
	if (curr_vdd_io_reg) {
		ret = sdhci_msm_vreg_init_reg(dev, curr_vdd_io_reg);
		if (ret)
			goto vdd_reg_deinit;
	}
	ret = sdhci_msm_vreg_reset(pdata);
	if (ret)
		dev_err(dev, "vreg reset failed (%d)\n", ret);
	goto out;

vdd_io_reg_deinit:
	if (curr_vdd_io_reg)
		sdhci_msm_vreg_deinit_reg(curr_vdd_io_reg);
vdd_reg_deinit:
	if (curr_vdd_reg)
		sdhci_msm_vreg_deinit_reg(curr_vdd_reg);
out:
	return ret;
}


static int sdhci_msm_set_vdd_io_vol(struct sdhci_msm_pltfm_data *pdata,
			enum vdd_io_level level,
			unsigned int voltage_level)
{
	int ret = 0;
	int set_level;
	struct sdhci_msm_reg_data *vdd_io_reg;

	if (!pdata->vreg_data)
		return ret;

	vdd_io_reg = pdata->vreg_data->vdd_io_data;
	if (vdd_io_reg && vdd_io_reg->is_enabled) {
		switch (level) {
		case VDD_IO_LOW:
			set_level = vdd_io_reg->low_vol_level;
			break;
		case VDD_IO_HIGH:
			set_level = vdd_io_reg->high_vol_level;
			break;
		case VDD_IO_SET_LEVEL:
			set_level = voltage_level;
			break;
		default:
			pr_err("%s: invalid argument level = %d",
					__func__, level);
			ret = -EINVAL;
			return ret;
		}
		ret = sdhci_msm_vreg_set_voltage(vdd_io_reg, set_level,
				set_level);
	}
	return ret;
}

static void sdhci_msm_cfg_sdiowakeup_gpio_irq(struct sdhci_host *host,
					      bool enable)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;

	if (enable && !msm_host->is_sdiowakeup_enabled)
		enable_irq(msm_host->pdata->sdiowakeup_irq);
	else if (!enable && msm_host->is_sdiowakeup_enabled)
		disable_irq_nosync(msm_host->pdata->sdiowakeup_irq);
	else
		dev_warn(&msm_host->pdev->dev, "%s: wakeup to config: %d curr: %d\n",
			__func__, enable, msm_host->is_sdiowakeup_enabled);
	msm_host->is_sdiowakeup_enabled = enable;
}

static irqreturn_t sdhci_msm_sdiowakeup_irq(int irq, void *data)
{
	struct sdhci_host *host = (struct sdhci_host *)data;
	unsigned long flags;

	pr_debug("%s: irq (%d) received\n", __func__, irq);

	spin_lock_irqsave(&host->lock, flags);
	sdhci_msm_cfg_sdiowakeup_gpio_irq(host, false);
	spin_unlock_irqrestore(&host->lock, flags);

	return IRQ_HANDLED;
}

static irqreturn_t sdhci_msm_pwr_irq(int irq, void *data)
{
	struct sdhci_host *host = (struct sdhci_host *)data;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	u8 irq_status = 0;
	u8 irq_ack = 0;
	int ret = 0;
	int pwr_state = 0, io_level = 0;
	unsigned long flags;

	irq_status = readb_relaxed(msm_host->core_mem + CORE_PWRCTL_STATUS);
	pr_debug("%s: Received IRQ(%d), status=0x%x\n",
		mmc_hostname(msm_host->mmc), irq, irq_status);

	
	writeb_relaxed(irq_status, (msm_host->core_mem + CORE_PWRCTL_CLEAR));
	mb();

	
	if (irq_status & CORE_PWRCTL_BUS_ON) {
		ret = sdhci_msm_setup_vreg(msm_host->pdata, true, false);
		if (!ret) {
			ret = sdhci_msm_setup_pins(msm_host->pdata, true);
			ret |= sdhci_msm_set_vdd_io_vol(msm_host->pdata,
					VDD_IO_HIGH, 0);
		}
		if (ret)
			irq_ack |= CORE_PWRCTL_BUS_FAIL;
		else
			irq_ack |= CORE_PWRCTL_BUS_SUCCESS;

		pwr_state = REQ_BUS_ON;
		io_level = REQ_IO_HIGH;
	}
	if (irq_status & CORE_PWRCTL_BUS_OFF) {
		ret = sdhci_msm_setup_vreg(msm_host->pdata, false, false);
		if (!ret) {
			ret = sdhci_msm_setup_pins(msm_host->pdata, false);
			ret |= sdhci_msm_set_vdd_io_vol(msm_host->pdata,
					VDD_IO_LOW, 0);
		}
		if (ret)
			irq_ack |= CORE_PWRCTL_BUS_FAIL;
		else
			irq_ack |= CORE_PWRCTL_BUS_SUCCESS;

		pwr_state = REQ_BUS_OFF;
		io_level = REQ_IO_LOW;
	}
	
	if (irq_status & CORE_PWRCTL_IO_LOW) {
		
		ret = sdhci_msm_set_vdd_io_vol(msm_host->pdata, VDD_IO_LOW, 0);
		if (ret)
			irq_ack |= CORE_PWRCTL_IO_FAIL;
		else
			irq_ack |= CORE_PWRCTL_IO_SUCCESS;

		io_level = REQ_IO_LOW;
	}
	if (irq_status & CORE_PWRCTL_IO_HIGH) {
		
		ret = sdhci_msm_set_vdd_io_vol(msm_host->pdata, VDD_IO_HIGH, 0);
		if (ret)
			irq_ack |= CORE_PWRCTL_IO_FAIL;
		else
			irq_ack |= CORE_PWRCTL_IO_SUCCESS;

		io_level = REQ_IO_HIGH;
	}

	
	writeb_relaxed(irq_ack, (msm_host->core_mem + CORE_PWRCTL_CTL));
	mb();

	if (io_level & REQ_IO_HIGH)
		writel_relaxed((readl_relaxed(host->ioaddr + CORE_VENDOR_SPEC) &
				~CORE_IO_PAD_PWR_SWITCH),
				host->ioaddr + CORE_VENDOR_SPEC);
	else if (io_level & REQ_IO_LOW)
		writel_relaxed((readl_relaxed(host->ioaddr + CORE_VENDOR_SPEC) |
				CORE_IO_PAD_PWR_SWITCH),
				host->ioaddr + CORE_VENDOR_SPEC);
	mb();

	pr_debug("%s: Handled IRQ(%d), ret=%d, ack=0x%x\n",
		mmc_hostname(msm_host->mmc), irq, ret, irq_ack);
	spin_lock_irqsave(&host->lock, flags);
	if (pwr_state)
		msm_host->curr_pwr_state = pwr_state;
	if (io_level)
		msm_host->curr_io_level = io_level;
	complete(&msm_host->pwr_irq_completion);
	spin_unlock_irqrestore(&host->lock, flags);

	return IRQ_HANDLED;
}

static unsigned int sdhci_msm_get_vreg_vdd_max_current(struct sdhci_msm_host
						       *host)
{
	struct sdhci_msm_slot_reg_data *curr_slot = host->pdata->vreg_data;
	if (!curr_slot)
		return 0;
	if (curr_slot->vdd_data)
		return curr_slot->vdd_data->hpm_uA / 1000;
	else
		return 0;
}

static ssize_t
show_polling(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	int poll;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	poll = !!(host->mmc->caps & MMC_CAP_NEEDS_POLL);
	spin_unlock_irqrestore(&host->lock, flags);

	return snprintf(buf, PAGE_SIZE, "%d\n", poll);
}

static ssize_t
store_polling(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	int value;
	unsigned long flags;

	if (!kstrtou32(buf, 0, &value)) {
		spin_lock_irqsave(&host->lock, flags);
		if (value) {
			host->mmc->caps |= MMC_CAP_NEEDS_POLL;
			mmc_detect_change(host->mmc, 0);
		} else {
			host->mmc->caps &= ~MMC_CAP_NEEDS_POLL;
		}
		spin_unlock_irqrestore(&host->lock, flags);
	}
	return count;
}

static ssize_t
show_sdhci_max_bus_bw(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			msm_host->msm_bus_vote.is_max_bw_needed);
}

static ssize_t
store_sdhci_max_bus_bw(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	uint32_t value;
	unsigned long flags;

	if (!kstrtou32(buf, 0, &value)) {
		spin_lock_irqsave(&host->lock, flags);
		msm_host->msm_bus_vote.is_max_bw_needed = !!value;
		spin_unlock_irqrestore(&host->lock, flags);
	}
	return count;
}

static void sdhci_msm_check_power_status(struct sdhci_host *host, u32 req_type)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	unsigned long flags;
	bool done = false;

	spin_lock_irqsave(&host->lock, flags);
	pr_debug("%s: %s: request %d curr_pwr_state %x curr_io_level %x\n",
			mmc_hostname(host->mmc), __func__, req_type,
			msm_host->curr_pwr_state, msm_host->curr_io_level);
	if ((req_type & msm_host->curr_pwr_state) ||
			(req_type & msm_host->curr_io_level))
		done = true;
	spin_unlock_irqrestore(&host->lock, flags);

	if (done)
		init_completion(&msm_host->pwr_irq_completion);
	else
		wait_for_completion(&msm_host->pwr_irq_completion);

	pr_debug("%s: %s: request %d done\n", mmc_hostname(host->mmc),
			__func__, req_type);
}

static void sdhci_msm_toggle_cdr(struct sdhci_host *host, bool enable)
{
	if (enable)
		writel_relaxed((readl_relaxed(host->ioaddr +
					      CORE_DLL_CONFIG) | CORE_CDR_EN),
			       host->ioaddr + CORE_DLL_CONFIG);
	else
		writel_relaxed((readl_relaxed(host->ioaddr +
					      CORE_DLL_CONFIG) & ~CORE_CDR_EN),
			       host->ioaddr + CORE_DLL_CONFIG);
}

static unsigned int sdhci_msm_max_segs(void)
{
	return SDHCI_MSM_MAX_SEGMENTS;
}

static unsigned int sdhci_msm_get_min_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;

	return msm_host->pdata->sup_clk_table[0];
}

static unsigned int sdhci_msm_get_max_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	int max_clk_index = msm_host->pdata->sup_clk_cnt;

	return msm_host->pdata->sup_clk_table[max_clk_index - 1];
}

static unsigned int sdhci_msm_get_sup_clk_rate(struct sdhci_host *host,
						u32 req_clk)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	unsigned int sel_clk = -1;
	unsigned char cnt;

	if (req_clk < sdhci_msm_get_min_clock(host)) {
		sel_clk = sdhci_msm_get_min_clock(host);
		return sel_clk;
	}

	for (cnt = 0; cnt < msm_host->pdata->sup_clk_cnt; cnt++) {
		if (msm_host->pdata->sup_clk_table[cnt] > req_clk) {
			break;
		} else if (msm_host->pdata->sup_clk_table[cnt] == req_clk) {
			sel_clk = msm_host->pdata->sup_clk_table[cnt];
			break;
		} else {
			sel_clk = msm_host->pdata->sup_clk_table[cnt];
		}
	}
	return sel_clk;
}

static int sdhci_msm_enable_controller_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	int rc = 0;

	if (atomic_read(&msm_host->controller_clock))
		return 0;

	sdhci_msm_bus_voting(host, 1);

	if (!IS_ERR(msm_host->pclk)) {
		rc = clk_prepare_enable(msm_host->pclk);
		if (rc) {
			pr_err("%s: %s: failed to enable the pclk with error %d\n",
			       mmc_hostname(host->mmc), __func__, rc);
			goto remove_vote;
		}
	}

	rc = clk_prepare_enable(msm_host->clk);
	if (rc) {
		pr_err("%s: %s: failed to enable the host-clk with error %d\n",
		       mmc_hostname(host->mmc), __func__, rc);
		goto disable_pclk;
	}

	atomic_set(&msm_host->controller_clock, 1);
	pr_debug("%s: %s: enabled controller clock\n",
			mmc_hostname(host->mmc), __func__);
	goto out;

disable_pclk:
	if (!IS_ERR(msm_host->pclk))
		clk_disable_unprepare(msm_host->pclk);
remove_vote:
	if (msm_host->msm_bus_vote.client_handle)
		sdhci_msm_bus_cancel_work_and_set_vote(host, 0);
out:
	return rc;
}



static int sdhci_msm_prepare_clocks(struct sdhci_host *host, bool enable)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	int rc = 0;

	if (enable && !atomic_read(&msm_host->clks_on)) {
		pr_debug("%s: request to enable clocks\n",
				mmc_hostname(host->mmc));

		if (atomic_read(&msm_host->controller_clock))
			sdhci_msm_bus_voting(host, 1);

		rc = sdhci_msm_enable_controller_clock(host);
		if (rc)
			goto remove_vote;

		if (!IS_ERR_OR_NULL(msm_host->bus_clk)) {
			rc = clk_prepare_enable(msm_host->bus_clk);
			if (rc) {
				pr_err("%s: %s: failed to enable the bus-clock with error %d\n",
					mmc_hostname(host->mmc), __func__, rc);
				goto disable_controller_clk;
			}
		}
		if (!IS_ERR(msm_host->ff_clk)) {
			rc = clk_prepare_enable(msm_host->ff_clk);
			if (rc) {
				pr_err("%s: %s: failed to enable the ff_clk with error %d\n",
					mmc_hostname(host->mmc), __func__, rc);
				goto disable_bus_clk;
			}
		}
		if (!IS_ERR(msm_host->sleep_clk)) {
			rc = clk_prepare_enable(msm_host->sleep_clk);
			if (rc) {
				pr_err("%s: %s: failed to enable the sleep_clk with error %d\n",
					mmc_hostname(host->mmc), __func__, rc);
				goto disable_ff_clk;
			}
		}
		mb();

	} else if (!enable && atomic_read(&msm_host->clks_on)) {
		pr_debug("%s: request to disable clocks\n",
				mmc_hostname(host->mmc));
		sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);
		mb();
		if (!IS_ERR_OR_NULL(msm_host->sleep_clk))
			clk_disable_unprepare(msm_host->sleep_clk);
		if (!IS_ERR_OR_NULL(msm_host->ff_clk))
			clk_disable_unprepare(msm_host->ff_clk);
		clk_disable_unprepare(msm_host->clk);
		if (!IS_ERR(msm_host->pclk))
			clk_disable_unprepare(msm_host->pclk);
		if (!IS_ERR_OR_NULL(msm_host->bus_clk))
			clk_disable_unprepare(msm_host->bus_clk);

		atomic_set(&msm_host->controller_clock, 0);
		sdhci_msm_bus_voting(host, 0);
	}
	atomic_set(&msm_host->clks_on, enable);
	goto out;
disable_ff_clk:
	if (!IS_ERR_OR_NULL(msm_host->ff_clk))
		clk_disable_unprepare(msm_host->ff_clk);
disable_bus_clk:
	if (!IS_ERR_OR_NULL(msm_host->bus_clk))
		clk_disable_unprepare(msm_host->bus_clk);
disable_controller_clk:
	if (!IS_ERR_OR_NULL(msm_host->clk))
		clk_disable_unprepare(msm_host->clk);
	if (!IS_ERR_OR_NULL(msm_host->pclk))
		clk_disable_unprepare(msm_host->pclk);
	atomic_set(&msm_host->controller_clock, 0);
remove_vote:
	if (msm_host->msm_bus_vote.client_handle)
		sdhci_msm_bus_cancel_work_and_set_vote(host, 0);
out:
	return rc;
}

static void sdhci_msm_set_clock(struct sdhci_host *host, unsigned int clock)
{
	int rc;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	struct mmc_ios	curr_ios = host->mmc->ios;
	u32 sup_clock, ddr_clock;
	bool curr_pwrsave;

	if (!clock) {
		writel_relaxed(readl_relaxed(host->ioaddr + CORE_VENDOR_SPEC) &
			~CORE_CLK_PWRSAVE, host->ioaddr + CORE_VENDOR_SPEC);
		sdhci_msm_prepare_clocks(host, false);
		host->clock = clock;
		return;
	}

	rc = sdhci_msm_prepare_clocks(host, true);
	if (rc)
		return;

	curr_pwrsave = !!(readl_relaxed(host->ioaddr + CORE_VENDOR_SPEC) &
			  CORE_CLK_PWRSAVE);
	if ((clock > 400000) &&
	    !curr_pwrsave && mmc_host_may_gate_card(host->mmc->card))
		writel_relaxed(readl_relaxed(host->ioaddr + CORE_VENDOR_SPEC)
				| CORE_CLK_PWRSAVE,
				host->ioaddr + CORE_VENDOR_SPEC);
	else if (curr_pwrsave && !mmc_host_may_gate_card(host->mmc->card))
		writel_relaxed(readl_relaxed(host->ioaddr + CORE_VENDOR_SPEC)
				& ~CORE_CLK_PWRSAVE,
				host->ioaddr + CORE_VENDOR_SPEC);

	sup_clock = sdhci_msm_get_sup_clk_rate(host, clock);
	if ((curr_ios.timing == MMC_TIMING_UHS_DDR50) ||
		(curr_ios.timing == MMC_TIMING_MMC_HS400)) {
		ddr_clock = clock * 2;
		sup_clock = sdhci_msm_get_sup_clk_rate(host,
				ddr_clock);
	}

	if (curr_ios.timing == MMC_TIMING_MMC_HS400) {
		
		writel_relaxed(((readl_relaxed(host->ioaddr + CORE_VENDOR_SPEC)
					& ~CORE_HC_MCLK_SEL_MASK)
					| CORE_HC_MCLK_SEL_HS400),
					host->ioaddr + CORE_VENDOR_SPEC);
		if (msm_host->tuning_done && !msm_host->calibration_done) {
			writel_relaxed((readl_relaxed(host->ioaddr + \
					CORE_VENDOR_SPEC)
					| CORE_HC_SELECT_IN_HS400
					| CORE_HC_SELECT_IN_EN),
					host->ioaddr + CORE_VENDOR_SPEC);
		}
	} else {
		
		writel_relaxed(((readl_relaxed(host->ioaddr + CORE_VENDOR_SPEC)
					& ~CORE_HC_MCLK_SEL_MASK)
					| CORE_HC_MCLK_SEL_DFLT),
					host->ioaddr + CORE_VENDOR_SPEC);

		writel_relaxed((readl_relaxed(host->ioaddr + CORE_VENDOR_SPEC)
				& ~CORE_HC_SELECT_IN_EN
				& ~CORE_HC_SELECT_IN_MASK),
				host->ioaddr + CORE_VENDOR_SPEC);
	}
	mb();

	if (sup_clock != msm_host->clk_rate) {
		if (is_sd_platform(msm_host->pdata) &&
			curr_ios.signal_voltage == MMC_SIGNAL_VOLTAGE_180)
			sdhci_msm_setup_hifreq_pins(msm_host->pdata, host);

		pr_debug("%s: %s: setting clk rate to %u\n",
				mmc_hostname(host->mmc), __func__, sup_clock);
		rc = clk_set_rate(msm_host->clk, sup_clock);
		if (rc) {
			pr_err("%s: %s: Failed to set rate %u for host-clk : %d\n",
					mmc_hostname(host->mmc), __func__,
					sup_clock, rc);
			return;
		}
		msm_host->clk_rate = sup_clock;
		host->clock = clock;
		sdhci_msm_bus_voting(host, 1);
	}
}

static int sdhci_msm_set_uhs_signaling(struct sdhci_host *host,
					unsigned int uhs)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	u16 ctrl_2;

	ctrl_2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);
	
	ctrl_2 &= ~SDHCI_CTRL_UHS_MASK;
	if (uhs == MMC_TIMING_MMC_HS400)
		ctrl_2 |= SDHCI_CTRL_UHS_SDR104;
	else if (uhs == MMC_TIMING_MMC_HS200)
		ctrl_2 |= SDHCI_CTRL_UHS_SDR104;
	else if (uhs == MMC_TIMING_UHS_SDR12)
		ctrl_2 |= SDHCI_CTRL_UHS_SDR12;
	else if (uhs == MMC_TIMING_UHS_SDR25)
		ctrl_2 |= SDHCI_CTRL_UHS_SDR25;
	else if (uhs == MMC_TIMING_UHS_SDR50)
		ctrl_2 |= SDHCI_CTRL_UHS_SDR50;
	else if (uhs == MMC_TIMING_UHS_SDR104)
		ctrl_2 |= SDHCI_CTRL_UHS_SDR104;
	else if (uhs == MMC_TIMING_UHS_DDR50)
		ctrl_2 |= SDHCI_CTRL_UHS_DDR50;
	if (host->clock <= CORE_FREQ_100MHZ) {
		if ((uhs == MMC_TIMING_MMC_HS400) ||
		    (uhs == MMC_TIMING_MMC_HS200) ||
		    (uhs == MMC_TIMING_UHS_SDR104))
			ctrl_2 &= ~SDHCI_CTRL_UHS_MASK;

		writel_relaxed((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
				| CORE_DLL_RST),
				host->ioaddr + CORE_DLL_CONFIG);

		
		writel_relaxed((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
				| CORE_DLL_PDN),
				host->ioaddr + CORE_DLL_CONFIG);
		mb();

		msm_host->calibration_done = false;
	}

	pr_debug("%s: %s-clock:%u uhs mode:%u ctrl_2:0x%x\n",
		mmc_hostname(host->mmc), __func__, host->clock, uhs, ctrl_2);
	sdhci_writew(host, ctrl_2, SDHCI_HOST_CONTROL2);

	return 0;
}

static void sdhci_msm_disable_data_xfer(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	u32 value;
	int ret;
	u32 version;

	version = readl_relaxed(msm_host->core_mem + CORE_MCI_VERSION);
	
	if (version == CORE_VERSION_310)
		return;

	value = readl_relaxed(msm_host->core_mem + CORE_MCI_DATA_CTRL);
	value &= ~(u32)CORE_MCI_DPSM_ENABLE;
	writel_relaxed(value, msm_host->core_mem + CORE_MCI_DATA_CTRL);

	
	writel_relaxed(CORE_TESTBUS_ENA | CORE_TESTBUS_SEL2,
			msm_host->core_mem + CORE_TESTBUS_CONFIG);

	ret = readl_poll_timeout_noirq(msm_host->core_mem
			+ CORE_SDCC_DEBUG_REG, value,
			!(value & CORE_DEBUG_REG_AHB_HTRANS),
			CORE_AHB_DATA_DELAY_US, 1);
	if (ret) {
		pr_err("%s: %s: can't stop ongoing AHB bus access by ADMA\n",
				mmc_hostname(host->mmc), __func__);
		BUG();
	}
	
	value = readl_relaxed(msm_host->core_mem + CORE_TESTBUS_CONFIG);
	value &= ~CORE_TESTBUS_ENA;
	writel_relaxed(value, msm_host->core_mem + CORE_TESTBUS_CONFIG);

	udelay(CORE_AHB_DESC_DELAY_US);
}

bool ac_status = false;
static void ac_cable_status_notifier_func(enum usb_connect_type online)
{
	if (online == CONNECT_TYPE_AC)
		ac_status = true;
	else
		ac_status = false;
}

struct t_cable_status_notifier ac_cable_status_notifier = {
	.name = "htc_battery",
	.func = ac_cable_status_notifier_func,
};

static int sdhci_proc_bkops_show(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	struct sdhci_host *host = (struct sdhci_host*) data;
	struct mmc_host *mmc = host->mmc;

	if (!mmc || !mmc->card)
		return 0;

	return sprintf(page, "%d", mmc_card_need_bkops_in_suspend(mmc->card) ? 1 : 0);
}

static int sdhci_proc_bkops_set(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
#ifdef CONFIG_MMC_NEED_BKOPS_IN_SUSPEND
	struct sdhci_host *host = (struct sdhci_host*) data;
	struct mmc_host *mmc = host->mmc;
	unsigned long flags;
	int value;

	if (!mmc || !mmc->card)
		return count;

	sscanf(buffer, "%d", &value);
	spin_lock_irqsave(&mmc->lock, flags);
	if (value) {
		pr_info("%s: force bkops\n", __func__);
		mmc_card_set_need_bkops_in_suspend(mmc->card);
	}
	spin_unlock_irqrestore(&mmc->lock, flags);
#endif

	return count;
}

static int sdhci_proc_speed_class(char *page, char **start, off_t off,
               int count, int *eof, void *data)
{
       struct mmc_host *host = (struct mmc_host*) data;
       if (!host || !host->card)
               return 0;
       return sprintf(page, "%d", host->card->speed_class);
}

static int sdhci_proc_tray_state(char *page, char **start, off_t off,
               int count, int *eof, void *data)
{
       struct mmc_host *host = (struct mmc_host*) data;

       return sprintf(page, "%d", mmc_cd_get_status(host));
}
static int sdhci_msm_get_cd(struct sdhci_host *host)
{
       return mmc_cd_get_status(host->mmc);
}

static struct sdhci_ops sdhci_msm_ops = {
	.set_uhs_signaling = sdhci_msm_set_uhs_signaling,
	.check_power_status = sdhci_msm_check_power_status,
	.execute_tuning = sdhci_msm_execute_tuning,
	.toggle_cdr = sdhci_msm_toggle_cdr,
	.get_max_segments = sdhci_msm_max_segs,
	.set_clock = sdhci_msm_set_clock,
	.get_min_clock = sdhci_msm_get_min_clock,
	.get_max_clock = sdhci_msm_get_max_clock,
	.disable_data_xfer = sdhci_msm_disable_data_xfer,
	.get_cd = sdhci_msm_get_cd,
	.enable_controller_clock = sdhci_msm_enable_controller_clock,
};

static int sdhci_msm_cfg_mpm_pin_wakeup(struct sdhci_host *host, unsigned mode)
{
	int ret = 0;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	unsigned int pin = msm_host->pdata->mpm_sdiowakeup_int;

	if (!pin)
		return 0;

	switch (mode) {
	case SDC_DAT1_DISABLE:
		ret = msm_mpm_enable_pin(pin, 0);
		break;
	case SDC_DAT1_ENABLE:
		ret = msm_mpm_set_pin_type(pin, IRQ_TYPE_LEVEL_LOW);
		if (!ret)
			ret = msm_mpm_enable_pin(pin, 1);
		break;
	case SDC_DAT1_ENWAKE:
		ret = msm_mpm_set_pin_wake(pin, 1);
		break;
	case SDC_DAT1_DISWAKE:
		ret = msm_mpm_set_pin_wake(pin, 0);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int __devinit sdhci_msm_probe(struct platform_device *pdev)
{
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_msm_host *msm_host;
	struct resource *core_memres = NULL;
	int ret = 0, dead = 0;
	u32 vdd_max_current;
	u16 host_version;
	u32 pwr, irq_status, irq_ctl;
	unsigned long flags;

	pr_debug("%s: Enter %s\n", dev_name(&pdev->dev), __func__);
	msm_host = devm_kzalloc(&pdev->dev, sizeof(struct sdhci_msm_host),
				GFP_KERNEL);
	if (!msm_host) {
		ret = -ENOMEM;
		goto out;
	}

	msm_host->sdhci_msm_pdata.ops = &sdhci_msm_ops;
	host = sdhci_pltfm_init(pdev, &msm_host->sdhci_msm_pdata);
	if (IS_ERR(host)) {
		ret = PTR_ERR(host);
		goto out;
	}

	pltfm_host = sdhci_priv(host);
	pltfm_host->priv = msm_host;
	msm_host->mmc = host->mmc;
	msm_host->pdev = pdev;

	
	if (pdev->dev.of_node) {
		ret = of_alias_get_id(pdev->dev.of_node, "sdhc");
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to get slot index %d\n",
				ret);
			goto pltfm_free;
		}
		if (disable_slots & (1 << (ret - 1))) {
			dev_info(&pdev->dev, "%s: Slot %d disabled\n", __func__,
				ret);
			ret = -ENODEV;
			goto pltfm_free;
		}

		msm_host->pdata = sdhci_msm_populate_pdata(&pdev->dev);
		if (!msm_host->pdata) {
			dev_err(&pdev->dev, "DT parsing error\n");
			goto pltfm_free;
		} else {
			if (msm_host->pdata->disable_sdcard_uhs)
				host->disable_sdcard_uhs = 1;

			host->mmc->expand_debounce = msm_host->pdata->expand_debounce;
		}
	} else {
		dev_err(&pdev->dev, "No device tree node\n");
		goto pltfm_free;
	}

	

	
	msm_host->bus_clk = devm_clk_get(&pdev->dev, "bus_clk");
	if (!IS_ERR_OR_NULL(msm_host->bus_clk)) {
		
		ret = clk_set_rate(msm_host->bus_clk, INT_MAX);
		if (ret)
			goto pltfm_free;
		ret = clk_prepare_enable(msm_host->bus_clk);
		if (ret)
			goto pltfm_free;
	}

	
	msm_host->pclk = devm_clk_get(&pdev->dev, "iface_clk");
	if (!IS_ERR(msm_host->pclk)) {
		ret = clk_prepare_enable(msm_host->pclk);
		if (ret)
			goto bus_clk_disable;
	}
	atomic_set(&msm_host->controller_clock, 1);

	
	msm_host->clk = devm_clk_get(&pdev->dev, "core_clk");
	if (IS_ERR(msm_host->clk)) {
		ret = PTR_ERR(msm_host->clk);
		goto pclk_disable;
	}

	
	ret = clk_set_rate(msm_host->clk, sdhci_msm_get_min_clock(host));
	if (ret) {
		dev_err(&pdev->dev, "MClk rate set failed (%d)\n", ret);
		goto pclk_disable;
	}
	ret = clk_prepare_enable(msm_host->clk);
	if (ret)
		goto pclk_disable;

	msm_host->clk_rate = sdhci_msm_get_min_clock(host);
	atomic_set(&msm_host->clks_on, 1);

	
	msm_host->ff_clk = devm_clk_get(&pdev->dev, "cal_clk");
	if (!IS_ERR(msm_host->ff_clk)) {
		ret = clk_prepare_enable(msm_host->ff_clk);
		if (ret)
			goto clk_disable;
	}

	
	msm_host->sleep_clk = devm_clk_get(&pdev->dev, "sleep_clk");
	if (!IS_ERR(msm_host->sleep_clk)) {
		ret = clk_prepare_enable(msm_host->sleep_clk);
		if (ret)
			goto ff_clk_disable;
	}

	msm_host->saved_tuning_phase = INVALID_TUNING_PHASE;

	ret = sdhci_msm_bus_register(msm_host, pdev);
	if (ret)
		goto sleep_clk_disable;

	if (msm_host->msm_bus_vote.client_handle)
		INIT_DELAYED_WORK(&msm_host->msm_bus_vote.vote_work,
				  sdhci_msm_bus_work);
	sdhci_msm_bus_voting(host, 1);

	
	ret = sdhci_msm_vreg_init(&pdev->dev, msm_host->pdata, true);
	if (ret) {
		dev_err(&pdev->dev, "Regulator setup failed (%d)\n", ret);
		goto bus_unregister;
	}

	
	core_memres = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, "core_mem");
	msm_host->core_mem = devm_ioremap(&pdev->dev, core_memres->start,
					resource_size(core_memres));

	if (!msm_host->core_mem) {
		dev_err(&pdev->dev, "Failed to remap registers\n");
		ret = -ENOMEM;
		goto vreg_deinit;
	}

	
	writel_relaxed(0, (msm_host->core_mem + CORE_HC_MODE));

	
	writel_relaxed(readl_relaxed(msm_host->core_mem + CORE_POWER) |
			CORE_SW_RST, msm_host->core_mem + CORE_POWER);
	ret = readl_poll_timeout(msm_host->core_mem + CORE_POWER,
			pwr, !(pwr & CORE_SW_RST), 10, 1000);

	if (ret) {
		dev_err(&pdev->dev, "reset failed (%d)\n", ret);
		goto vreg_deinit;
	}
	
	writel_relaxed(HC_MODE_EN, (msm_host->core_mem + CORE_HC_MODE));

	
	writel_relaxed(readl_relaxed(msm_host->core_mem + CORE_HC_MODE) |
			FF_CLK_SW_RST_DIS, msm_host->core_mem + CORE_HC_MODE);

	irq_status = readl_relaxed(msm_host->core_mem + CORE_PWRCTL_STATUS);
	writel_relaxed(irq_status, (msm_host->core_mem + CORE_PWRCTL_CLEAR));
	irq_ctl = readl_relaxed(msm_host->core_mem + CORE_PWRCTL_CTL);
	if (irq_status & (CORE_PWRCTL_BUS_ON | CORE_PWRCTL_BUS_OFF))
		irq_ctl |= CORE_PWRCTL_BUS_SUCCESS;
	if (irq_status & (CORE_PWRCTL_IO_HIGH | CORE_PWRCTL_IO_LOW))
		irq_ctl |= CORE_PWRCTL_IO_SUCCESS;
	writel_relaxed(irq_ctl, (msm_host->core_mem + CORE_PWRCTL_CTL));
	mb();

	host->quirks |= SDHCI_QUIRK_BROKEN_CARD_DETECTION;
	host->quirks |= SDHCI_QUIRK_SINGLE_POWER_WRITE;
	host->quirks |= SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN;
	host->quirks2 |= SDHCI_QUIRK2_ALWAYS_USE_BASE_CLOCK;
	host->quirks2 |= SDHCI_QUIRK2_IGNORE_CMDCRC_FOR_TUNING;
	host->quirks2 |= SDHCI_QUIRK2_USE_MAX_DISCARD_SIZE;
	host->quirks2 |= SDHCI_QUIRK2_IGNORE_DATATOUT_FOR_R1BCMD;
	host->quirks2 |= SDHCI_QUIRK2_BROKEN_PRESET_VALUE;
	if (!is_mmc_platform(msm_host->pdata))
		host->quirks2 |= SDHCI_QUIRK2_USE_RESERVED_MAX_TIMEOUT;

	if (host->quirks2 & SDHCI_QUIRK2_ALWAYS_USE_BASE_CLOCK)
		host->quirks2 |= SDHCI_QUIRK2_DIVIDE_TOUT_BY_4;

	host_version = readw_relaxed((host->ioaddr + SDHCI_HOST_VERSION));
	dev_dbg(&pdev->dev, "Host Version: 0x%x Vendor Version 0x%x\n",
		host_version, ((host_version & SDHCI_VENDOR_VER_MASK) >>
		  SDHCI_VENDOR_VER_SHIFT));
	if (((host_version & SDHCI_VENDOR_VER_MASK) >>
		SDHCI_VENDOR_VER_SHIFT) == SDHCI_VER_100) {
		host->quirks2 |= SDHCI_QUIRK2_SLOW_INT_CLR;
		host->quirks2 |= SDHCI_QUIRK2_RDWR_TX_ACTIVE_EOT;
	}

	host->quirks2 |= SDHCI_QUIRK2_IGN_DATA_END_BIT_ERROR;

	
	msm_host->pwr_irq = platform_get_irq_byname(pdev, "pwr_irq");
	if (msm_host->pwr_irq < 0) {
		dev_err(&pdev->dev, "Failed to get pwr_irq by name (%d)\n",
				msm_host->pwr_irq);
		goto vreg_deinit;
	}
	ret = devm_request_threaded_irq(&pdev->dev, msm_host->pwr_irq, NULL,
					sdhci_msm_pwr_irq, IRQF_ONESHOT,
					dev_name(&pdev->dev), host);
	if (ret) {
		dev_err(&pdev->dev, "Request threaded irq(%d) failed (%d)\n",
				msm_host->pwr_irq, ret);
		goto vreg_deinit;
	}

	
	writel_relaxed(INT_MASK, (msm_host->core_mem + CORE_PWRCTL_MASK));

	
	if (is_mmc_platform(msm_host->pdata))
		msm_host->mmc->clkgate_delay = SDHCI_MSM_MMC_CLK_GATE_DELAY_EMMC;
	else
		msm_host->mmc->clkgate_delay = SDHCI_MSM_MMC_CLK_GATE_DELAY;

	
	msm_host->mmc->caps |= msm_host->pdata->mmc_bus_width;
	msm_host->mmc->caps |= msm_host->pdata->caps;

	vdd_max_current = sdhci_msm_get_vreg_vdd_max_current(msm_host);
	if (vdd_max_current >= 800)
		msm_host->mmc->caps |= MMC_CAP_MAX_CURRENT_800;
	else if (vdd_max_current >= 600)
		msm_host->mmc->caps |= MMC_CAP_MAX_CURRENT_600;
	else if (vdd_max_current >= 400)
		msm_host->mmc->caps |= MMC_CAP_MAX_CURRENT_400;
	else
		msm_host->mmc->caps |= MMC_CAP_MAX_CURRENT_200;

	if (vdd_max_current > 150)
		msm_host->mmc->caps |= MMC_CAP_SET_XPC_180 |
					MMC_CAP_SET_XPC_300|
					MMC_CAP_SET_XPC_330;

	msm_host->mmc->caps2 |= msm_host->pdata->caps2;
	msm_host->mmc->caps2 |= MMC_CAP2_CORE_RUNTIME_PM;
	msm_host->mmc->caps2 |= (MMC_CAP2_BOOTPART_NOACC |
				MMC_CAP2_DETECT_ON_ERR);
	
	msm_host->mmc->caps2 |= MMC_CAP2_CACHE_CTRL;
	
	
	msm_host->mmc->caps2 |= MMC_CAP2_STOP_REQUEST;
	msm_host->mmc->caps2 |= MMC_CAP2_ASYNC_SDIO_IRQ_4BIT_MODE;
	msm_host->mmc->caps2 |= MMC_CAP2_CORE_PM;
	msm_host->mmc->pm_caps |= MMC_PM_KEEP_POWER | MMC_PM_WAKE_SDIO_IRQ;

	if (msm_host->pdata->nonremovable)
		msm_host->mmc->caps |= MMC_CAP_NONREMOVABLE;

	host->cpu_dma_latency_us = msm_host->pdata->cpu_dma_latency_us;

	init_completion(&msm_host->pwr_irq_completion);

	if (gpio_is_valid(msm_host->pdata->status_gpio)) {
		ret = mmc_cd_gpio_request(msm_host->mmc,
				msm_host->pdata->status_gpio);
		if (ret) {
			dev_err(&pdev->dev, "%s: Failed to request card detection IRQ %d\n",
					__func__, ret);
			goto vreg_deinit;
		}
	}

	if (dma_supported(mmc_dev(host->mmc), DMA_BIT_MASK(32))) {
		host->dma_mask = DMA_BIT_MASK(32);
		mmc_dev(host->mmc)->dma_mask = &host->dma_mask;
	} else {
		dev_err(&pdev->dev, "%s: Failed to set dma mask\n", __func__);
	}

	msm_host->pdata->sdiowakeup_irq = platform_get_irq_byname(pdev,
							  "sdiowakeup_irq");
	if (msm_host->pdata->sdiowakeup_irq >= 0) {
		msm_host->is_sdiowakeup_enabled = true;
		ret = request_irq(msm_host->pdata->sdiowakeup_irq,
				  sdhci_msm_sdiowakeup_irq,
				  IRQF_SHARED | IRQF_TRIGGER_LOW,
				  "sdhci-msm sdiowakeup", host);
		if (ret) {
			dev_err(&pdev->dev, "%s: request sdiowakeup IRQ %d: failed: %d\n",
				__func__, msm_host->pdata->sdiowakeup_irq, ret);
			msm_host->pdata->sdiowakeup_irq = -1;
			msm_host->is_sdiowakeup_enabled = false;
			goto free_cd_gpio;
		} else {
			spin_lock_irqsave(&host->lock, flags);
			sdhci_msm_cfg_sdiowakeup_gpio_irq(host, false);
			spin_unlock_irqrestore(&host->lock, flags);
		}
	}

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(&pdev->dev, "Add host failed (%d)\n", ret);
		goto free_cd_gpio;
	}

	pr_info("%s: Platform slot type: %s\n", mmc_hostname(host->mmc),
		mmc_type_str(msm_host->pdata->slot_type));

	msm_host->msm_bus_vote.max_bus_bw.show = show_sdhci_max_bus_bw;
	msm_host->msm_bus_vote.max_bus_bw.store = store_sdhci_max_bus_bw;
	sysfs_attr_init(&msm_host->msm_bus_vote.max_bus_bw.attr);
	msm_host->msm_bus_vote.max_bus_bw.attr.name = "max_bus_bw";
	msm_host->msm_bus_vote.max_bus_bw.attr.mode = S_IRUGO | S_IWUSR;
	ret = device_create_file(&pdev->dev,
			&msm_host->msm_bus_vote.max_bus_bw);
	if (ret)
		goto remove_host;

	if (!gpio_is_valid(msm_host->pdata->status_gpio)) {
		msm_host->polling.show = show_polling;
		msm_host->polling.store = store_polling;
		sysfs_attr_init(&msm_host->polling.attr);
		msm_host->polling.attr.name = "polling";
		msm_host->polling.attr.mode = S_IRUGO | S_IWUSR;
		ret = device_create_file(&pdev->dev, &msm_host->polling);
		if (ret)
			goto remove_max_bus_bw_file;
	}
	ret = pm_runtime_set_active(&pdev->dev);
	if (ret)
		pr_err("%s: %s: pm_runtime_set_active failed: err: %d\n",
		       mmc_hostname(host->mmc), __func__, ret);
	else if (mmc_use_core_runtime_pm(host->mmc))
		pm_runtime_enable(&pdev->dev);

	if (is_mmc_platform(msm_host->pdata) &&
			(msm_host->mmc->caps2 & MMC_CAP2_INIT_BKOPS)) {
		msm_host->bkops_proc = create_proc_entry("emmc_bkops", 0664, NULL);
		if (msm_host->bkops_proc) {
			msm_host->bkops_proc->read_proc = sdhci_proc_bkops_show;
			msm_host->bkops_proc->write_proc = sdhci_proc_bkops_set;
			msm_host->bkops_proc->data = (void *) host;
		} else
			pr_warning("%s: Failed to create emmc_bkops entry\n",
					mmc_hostname(host->mmc));

	}
	if(is_sd_platform(msm_host->pdata)) {
			msm_host->speed_class = create_proc_entry("sd_speed_class", 0444, NULL);
			if (msm_host->speed_class) {
				   msm_host->speed_class->read_proc = sdhci_proc_speed_class;
				   msm_host->speed_class->data = (void *) host->mmc;
			} else
				   pr_warning("%s: Failed to create sd_speed_class entry\n",
						   mmc_hostname(host->mmc));
			msm_host->speed_class = create_proc_entry("sd_tray_state", 0444, NULL);
			if (msm_host->speed_class) {
				   msm_host->speed_class->read_proc = sdhci_proc_tray_state;
				   msm_host->speed_class->data = (void *) host->mmc;
			} else
				   pr_warning("%s: Failed to create sd_tray_state entry\n",
						   mmc_hostname(host->mmc));
	}

	if (msm_host->pdata->mpm_sdiowakeup_int != -1) {
		ret = sdhci_msm_cfg_mpm_pin_wakeup(host, SDC_DAT1_ENABLE);
		if (ret) {
			pr_err("%s: enabling wakeup: failed: ret: %d\n",
			       mmc_hostname(host->mmc), ret);
			ret = 0;
			msm_host->pdata->mpm_sdiowakeup_int = -1;
		}
	}

	device_enable_async_suspend(&pdev->dev);

	
	if (is_mmc_platform(msm_host->pdata))
		cable_detect_register_notifier(&ac_cable_status_notifier);

	goto out;

remove_max_bus_bw_file:
	device_remove_file(&pdev->dev, &msm_host->msm_bus_vote.max_bus_bw);
remove_host:
	dead = (readl_relaxed(host->ioaddr + SDHCI_INT_STATUS) == 0xffffffff);
	sdhci_remove_host(host, dead);
free_cd_gpio:
	if (gpio_is_valid(msm_host->pdata->status_gpio))
		mmc_cd_gpio_free(msm_host->mmc);
	if (sdhci_is_valid_gpio_wakeup_int(msm_host))
		free_irq(msm_host->pdata->sdiowakeup_irq, host);
vreg_deinit:
	sdhci_msm_vreg_init(&pdev->dev, msm_host->pdata, false);
bus_unregister:
	if (msm_host->msm_bus_vote.client_handle)
		sdhci_msm_bus_cancel_work_and_set_vote(host, 0);
	sdhci_msm_bus_unregister(msm_host);
sleep_clk_disable:
	if (!IS_ERR(msm_host->sleep_clk))
		clk_disable_unprepare(msm_host->sleep_clk);
ff_clk_disable:
	if (!IS_ERR(msm_host->ff_clk))
		clk_disable_unprepare(msm_host->ff_clk);
clk_disable:
	if (!IS_ERR(msm_host->clk))
		clk_disable_unprepare(msm_host->clk);
pclk_disable:
	if (!IS_ERR(msm_host->pclk))
		clk_disable_unprepare(msm_host->pclk);
bus_clk_disable:
	if (!IS_ERR_OR_NULL(msm_host->bus_clk))
		clk_disable_unprepare(msm_host->bus_clk);
pltfm_free:
	sdhci_pltfm_free(pdev);
out:
	pr_debug("%s: Exit %s\n", dev_name(&pdev->dev), __func__);
	return ret;
}

static void sdhci_msm_shutdown(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	struct mmc_host *mmc = host->mmc;

	if (!mmc || !host || !msm_host)
		return;

	if (is_mmc_platform(msm_host->pdata)) {
		pr_info("%s: %s enter\n", mmc_hostname(mmc), __func__);
		printk(KERN_INFO "%s: Lock eMMC\n", __func__);
		atomic_set(&emmc_reboot, 1);
		pr_info("%s: %s leave\n", mmc_hostname(mmc), __func__);
	}
}

static int __devexit sdhci_msm_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	struct sdhci_msm_pltfm_data *pdata = msm_host->pdata;
	int dead = (readl_relaxed(host->ioaddr + SDHCI_INT_STATUS) ==
			0xffffffff);

	pr_debug("%s: %s\n", dev_name(&pdev->dev), __func__);
	if (!gpio_is_valid(msm_host->pdata->status_gpio))
		device_remove_file(&pdev->dev, &msm_host->polling);
	device_remove_file(&pdev->dev, &msm_host->msm_bus_vote.max_bus_bw);
	sdhci_remove_host(host, dead);
	pm_runtime_disable(&pdev->dev);
	sdhci_pltfm_free(pdev);

	if (sdhci_is_valid_mpm_wakeup_int(msm_host))
		sdhci_msm_cfg_mpm_pin_wakeup(host, SDC_DAT1_DISABLE);

	if (sdhci_is_valid_gpio_wakeup_int(msm_host))
		free_irq(msm_host->pdata->sdiowakeup_irq, host);

	if (gpio_is_valid(msm_host->pdata->status_gpio))
		mmc_cd_gpio_free(msm_host->mmc);

	sdhci_msm_vreg_init(&pdev->dev, msm_host->pdata, false);

	if (pdata->pin_data)
		sdhci_msm_setup_pins(pdata, false);

	if (msm_host->msm_bus_vote.client_handle) {
		sdhci_msm_bus_cancel_work_and_set_vote(host, 0);
		sdhci_msm_bus_unregister(msm_host);
	}
	if(msm_host->speed_class)
		remove_proc_entry("sd_speed_class", NULL);
	return 0;
}

static int sdhci_msm_cfg_sdio_wakeup(struct sdhci_host *host, bool enable)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	unsigned long flags;
	int ret = 0;

	if (!(host->mmc->card && mmc_card_sdio(host->mmc->card) &&
	      (sdhci_is_valid_mpm_wakeup_int(msm_host) ||
	      sdhci_is_valid_gpio_wakeup_int(msm_host)) &&
	      mmc_card_wake_sdio_irq(host->mmc))) {
		return 1;
	}

	spin_lock_irqsave(&host->lock, flags);
	if (enable) {
		
		if (sdhci_is_valid_gpio_wakeup_int(msm_host)) {
			ret = enable_irq_wake(msm_host->pdata->sdiowakeup_irq);
			if (!ret)
				sdhci_msm_cfg_sdiowakeup_gpio_irq(host, true);
			goto out;
		} else {
			ret = sdhci_msm_cfg_mpm_pin_wakeup(host,
							   SDC_DAT1_ENWAKE);
			if (ret)
				goto out;
			ret = enable_irq_wake(host->irq);
			if (ret)
				sdhci_msm_cfg_mpm_pin_wakeup(host,
							     SDC_DAT1_DISWAKE);
		}
	} else {
		if (sdhci_is_valid_gpio_wakeup_int(msm_host)) {
			ret = disable_irq_wake(msm_host->pdata->sdiowakeup_irq);
			sdhci_msm_cfg_sdiowakeup_gpio_irq(host, false);
		} else {
			ret = sdhci_msm_cfg_mpm_pin_wakeup(host,
							   SDC_DAT1_DISWAKE);
			if (ret)
				goto out;
			ret = disable_irq_wake(host->irq);
		}
	}
out:
	if (ret)
		pr_err("%s: %s: %sable wakeup: failed: %d gpio: %d mpm: %d\n",
		       mmc_hostname(host->mmc), __func__, enable ? "en" : "dis",
		       ret, msm_host->pdata->sdiowakeup_irq,
		       msm_host->pdata->mpm_sdiowakeup_int);
	spin_unlock_irqrestore(&host->lock, flags);
	return ret;
}

static int sdhci_msm_runtime_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	int ret;

	ret = sdhci_msm_cfg_sdio_wakeup(host, true);
	
	if (!ret)
		goto skip_disable_host_irq;

	disable_irq(host->irq);

skip_disable_host_irq:
	disable_irq(msm_host->pwr_irq);

	if (!atomic_read(&msm_host->clks_on)) {
		if (msm_host->msm_bus_vote.client_handle)
			sdhci_msm_bus_cancel_work_and_set_vote(host, 0);
	}

	return 0;
}

static int sdhci_msm_runtime_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	int ret;

	ret = sdhci_msm_cfg_sdio_wakeup(host, false);
	if (!ret)
		goto skip_enable_host_irq;

	enable_irq(host->irq);

skip_enable_host_irq:
	enable_irq(msm_host->pwr_irq);

	return 0;
}

#ifdef CONFIG_PM_SLEEP

#ifdef CONFIG_MMC_NEED_BKOPS_IN_SUSPEND
static int sdhci_msm_prepare(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct mmc_host *mmc = host->mmc;

	if (!mmc->card)
		return 0;

	if (mmc_card_need_bkops_in_suspend(mmc->card) &&
			!mmc_card_doing_bkops(mmc->card))
		mmc_card_start_bkops(mmc->card);

	return 0;
}

static void sdhci_msm_complete(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct mmc_host *mmc = host->mmc;
	unsigned long flags;

	if (!mmc->card)
		return;

	if (mmc_card_need_bkops_in_suspend(mmc->card)) {
		mmc_card_stop_bkops(mmc->card);
		spin_lock_irqsave(&mmc->lock, flags);
		mmc_card_clr_need_bkops_in_suspend(mmc->card);
		spin_unlock_irqrestore(&mmc->lock, flags);
	}

	return;
}
#endif

static int sdhci_msm_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	int ret = 0;

	if (gpio_is_valid(msm_host->pdata->status_gpio)
		&& !is_sd_platform(msm_host->pdata))
		mmc_cd_gpio_free(msm_host->mmc);

	if (pm_runtime_suspended(dev)) {
		pr_debug("%s: %s: already runtime suspended\n",
		mmc_hostname(host->mmc), __func__);
		goto out;
	}

	return sdhci_msm_runtime_suspend(dev);
out:
	return ret;
}

static int sdhci_msm_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	int ret = 0;

	if (gpio_is_valid(msm_host->pdata->status_gpio)
		&& !is_sd_platform(msm_host->pdata)) {
		ret = mmc_cd_gpio_request(msm_host->mmc,
				msm_host->pdata->status_gpio);
		if (ret)
			pr_err("%s: %s: Failed to request card detection IRQ %d\n",
					mmc_hostname(host->mmc), __func__, ret);
	}

	if (pm_runtime_suspended(dev)) {
		pr_debug("%s: %s: runtime suspended, defer system resume\n",
		mmc_hostname(host->mmc), __func__);
		goto out;
	}

	return sdhci_msm_runtime_resume(dev);
out:
	return ret;
}

static int sdhci_msm_suspend_noirq(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	int ret = 0;

	if (atomic_read(&msm_host->clks_on)) {
		pr_warn("%s: %s: clock ON after suspend, aborting suspend\n",
			mmc_hostname(host->mmc), __func__);
		ret = -EAGAIN;
	}

	return ret;
}
#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops sdhci_msm_pmops = {
	SET_SYSTEM_SLEEP_PM_OPS(sdhci_msm_suspend, sdhci_msm_resume)
	SET_RUNTIME_PM_OPS(sdhci_msm_runtime_suspend, sdhci_msm_runtime_resume,
			   NULL)
#ifdef CONFIG_MMC_NEED_BKOPS_IN_SUSPEND
	.prepare = sdhci_msm_prepare,
	.complete = sdhci_msm_complete,
#endif
	.suspend_noirq = sdhci_msm_suspend_noirq,
};

#define SDHCI_MSM_PMOPS (&sdhci_msm_pmops)

#else
#define SDHCI_PM_OPS NULL
#endif
static const struct of_device_id sdhci_msm_dt_match[] = {
	{.compatible = "qcom,sdhci-msm"},
	{  }
};
MODULE_DEVICE_TABLE(of, sdhci_msm_dt_match);

static struct platform_driver sdhci_msm_driver = {
	.probe		= sdhci_msm_probe,
	.remove		= __devexit_p(sdhci_msm_remove),
	.shutdown	= sdhci_msm_shutdown,
	.driver		= {
		.name	= "sdhci_msm",
		.owner	= THIS_MODULE,
		.of_match_table = sdhci_msm_dt_match,
		.pm = SDHCI_MSM_PMOPS,
	},
};

module_platform_driver(sdhci_msm_driver);

MODULE_DESCRIPTION("Qualcomm Secure Digital Host Controller Interface driver");
MODULE_LICENSE("GPL v2");
