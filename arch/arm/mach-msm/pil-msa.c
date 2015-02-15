/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/dma-mapping.h>

#include "peripheral-loader.h"
#include "pil-q6v5.h"
#include "pil-msa.h"

#define QDSP6SS_RST_EVB			0x010

#define MSS_Q6_HALT_BASE		0x180
#define MSS_MODEM_HALT_BASE		0x200
#define MSS_NC_HALT_BASE		0x280

#define STATUS_PBL_SUCCESS		0x1
#define STATUS_XPU_UNLOCKED		0x1
#define STATUS_XPU_UNLOCKED_SCRIBBLED	0x2

#define RMB_MBA_IMAGE			0x00
#define RMB_PBL_STATUS			0x04
#define RMB_MBA_COMMAND			0x08
#define RMB_MBA_STATUS			0x0C
#define RMB_PMI_META_DATA		0x10
#define RMB_PMI_CODE_START		0x14
#define RMB_PMI_CODE_LENGTH		0x18

#define MAX_VDD_MX_UV			1150000

#define POLL_INTERVAL_US		50

#define CMD_META_DATA_READY		0x1
#define CMD_LOAD_READY			0x2

#define STATUS_META_DATA_AUTH_SUCCESS	0x3
#define STATUS_AUTH_COMPLETE		0x4

#define EXTERNAL_BHS_ON			BIT(0)
#define EXTERNAL_BHS_STATUS		BIT(4)
#define BHS_TIMEOUT_US			50

static int pbl_mba_boot_timeout_ms = 1000;
module_param(pbl_mba_boot_timeout_ms, int, S_IRUGO | S_IWUSR);

static int modem_auth_timeout_ms = 10000;
module_param(modem_auth_timeout_ms, int, S_IRUGO | S_IWUSR);

static void pil_msa_dump_rmb_regs(struct q6v5_data *drv) {
#define PIL_MSA_DUMP_RMB_REG(REG) \
	val = readl (drv->rmb_base + REG);\
	dev_err(dev, "%s: [0x%08X] %s: 0x%08X\n", __func__, (u32)drv->rmb_base_phys + REG, #REG, val);

	struct device *dev = drv->desc.dev;
	u32 val;

	PIL_MSA_DUMP_RMB_REG(RMB_MBA_IMAGE);
	PIL_MSA_DUMP_RMB_REG(RMB_PBL_STATUS);
	PIL_MSA_DUMP_RMB_REG(RMB_MBA_COMMAND);
	PIL_MSA_DUMP_RMB_REG(RMB_MBA_STATUS);
	PIL_MSA_DUMP_RMB_REG(RMB_PMI_META_DATA);
	PIL_MSA_DUMP_RMB_REG(RMB_PMI_CODE_START);
	PIL_MSA_DUMP_RMB_REG(RMB_PMI_CODE_LENGTH);
#undef PIL_MSA_DUMP_RMB_REG
}

static int pil_msa_pbl_power_up(struct q6v5_data *drv)
{
	int ret = 0;
	struct device *dev = drv->desc.dev;
	u32 regval;

	if (drv->vreg) {
		ret = regulator_enable(drv->vreg);
		if (ret)
			dev_err(dev, "Failed to enable modem regulator.\n");
	}

	if (drv->cxrail_bhs) {
		regval = readl_relaxed(drv->cxrail_bhs);
		regval |= EXTERNAL_BHS_ON;
		writel_relaxed(regval, drv->cxrail_bhs);

		ret = readl_poll_timeout(drv->cxrail_bhs, regval,
			regval & EXTERNAL_BHS_STATUS, 1, BHS_TIMEOUT_US);
	}

	return ret;
}

static int pil_msa_pbl_power_down(struct q6v5_data *drv)
{
	u32 regval;

	if (drv->cxrail_bhs) {
		regval = readl_relaxed(drv->cxrail_bhs);
		regval &= ~EXTERNAL_BHS_ON;
		writel_relaxed(regval, drv->cxrail_bhs);
	}

	if (drv->vreg)
		return regulator_disable(drv->vreg);

	return 0;
}

static int pil_msa_pbl_enable_clks(struct q6v5_data *drv)
{
	int ret;

	ret = clk_prepare_enable(drv->ahb_clk);
	if (ret)
		goto err_ahb_clk;
	ret = clk_prepare_enable(drv->axi_clk);
	if (ret)
		goto err_axi_clk;
	ret = clk_prepare_enable(drv->rom_clk);
	if (ret)
		goto err_rom_clk;

	return 0;

err_rom_clk:
	clk_disable_unprepare(drv->axi_clk);
err_axi_clk:
	clk_disable_unprepare(drv->ahb_clk);
err_ahb_clk:
	return ret;
}

static void pil_msa_pbl_disable_clks(struct q6v5_data *drv)
{
	clk_disable_unprepare(drv->rom_clk);
	clk_disable_unprepare(drv->axi_clk);
	clk_disable_unprepare(drv->ahb_clk);
}

static int pil_msa_wait_for_mba_ready(struct q6v5_data *drv)
{
	struct device *dev = drv->desc.dev;
	int ret;
	u32 status;

	
	ret = readl_poll_timeout(drv->rmb_base + RMB_PBL_STATUS, status,
		status != 0, POLL_INTERVAL_US, pbl_mba_boot_timeout_ms * 1000);
	if (ret) {
		dev_err(dev, "PBL boot timed out\n");
		pil_msa_dump_rmb_regs(drv);
		return ret;
	}
	if (status != STATUS_PBL_SUCCESS) {
		dev_err(dev, "PBL returned unexpected status %d\n", status);
		return -EINVAL;
	}

	
	ret = readl_poll_timeout(drv->rmb_base + RMB_MBA_STATUS, status,
		status != 0, POLL_INTERVAL_US, pbl_mba_boot_timeout_ms * 1000);
	if (ret) {
		dev_err(dev, "MBA boot timed out\n");
		pil_msa_dump_rmb_regs(drv);
		return ret;
	}
	if (status != STATUS_XPU_UNLOCKED &&
	    status != STATUS_XPU_UNLOCKED_SCRIBBLED) {
		dev_err(dev, "MBA returned unexpected status %d\n", status);
		pil_msa_dump_rmb_regs(drv);
		return -EINVAL;
	}

	return 0;
}

static int pil_msa_pbl_shutdown(struct pil_desc *pil)
{
	struct q6v5_data *drv = container_of(pil, struct q6v5_data, desc);

	pil_q6v5_halt_axi_port(pil, drv->axi_halt_base + MSS_Q6_HALT_BASE);
	pil_q6v5_halt_axi_port(pil, drv->axi_halt_base + MSS_MODEM_HALT_BASE);
	pil_q6v5_halt_axi_port(pil, drv->axi_halt_base + MSS_NC_HALT_BASE);

	writel_relaxed(1, drv->restart_reg);

	if (drv->is_booted) {
		pil_msa_pbl_disable_clks(drv);
		pil_msa_pbl_power_down(drv);
		drv->is_booted = false;
	}

	return 0;
}

static int pil_msa_pbl_reset(struct pil_desc *pil)
{
	struct q6v5_data *drv = container_of(pil, struct q6v5_data, desc);
	phys_addr_t start_addr = pil_get_entry_addr(pil);
	int ret;

	ret = pil_msa_pbl_power_up(drv);
	if (ret)
		goto err_power;

	
	writel_relaxed(0, drv->restart_reg);
	mb();
	udelay(2);

	ret = pil_msa_pbl_enable_clks(drv);
	if (ret)
		goto err_clks;

	
	if (drv->self_auth) {
		writel_relaxed(start_addr, drv->rmb_base + RMB_MBA_IMAGE);
		
		mb();
	} else {
		writel_relaxed((start_addr >> 4) & 0x0FFFFFF0,
				drv->reg_base + QDSP6SS_RST_EVB);
	}

	ret = pil_q6v5_reset(pil);
	if (ret)
		goto err_q6v5_reset;

	
	if (drv->self_auth) {
		ret = pil_msa_wait_for_mba_ready(drv);
		if (ret)
			goto err_q6v5_reset;
	}

	drv->is_booted = true;

	return 0;

err_q6v5_reset:
	pil_msa_pbl_disable_clks(drv);
err_clks:
	writel_relaxed(1, drv->restart_reg);
	pil_msa_pbl_power_down(drv);
err_power:
	return ret;
}

static int pil_msa_pbl_make_proxy_votes(struct pil_desc *pil)
{
	int ret;
	struct q6v5_data *drv = container_of(pil, struct q6v5_data, desc);

	ret = regulator_set_voltage(drv->vreg_mx, VDD_MSS_UV, MAX_VDD_MX_UV);
	if (ret) {
		dev_err(pil->dev, "Failed to request vreg_mx voltage\n");
		return ret;
	}

	ret = regulator_enable(drv->vreg_mx);
	if (ret) {
		dev_err(pil->dev, "Failed to enable vreg_mx\n");
		regulator_set_voltage(drv->vreg_mx, 0, MAX_VDD_MX_UV);
		return ret;
	}

	ret = pil_q6v5_make_proxy_votes(pil);
	if (ret) {
		regulator_disable(drv->vreg_mx);
		regulator_set_voltage(drv->vreg_mx, 0, MAX_VDD_MX_UV);
	}

	return ret;
}

static void pil_msa_pbl_remove_proxy_votes(struct pil_desc *pil)
{
	struct q6v5_data *drv = container_of(pil, struct q6v5_data, desc);
	pil_q6v5_remove_proxy_votes(pil);
	regulator_disable(drv->vreg_mx);
	regulator_set_voltage(drv->vreg_mx, 0, MAX_VDD_MX_UV);
}

struct pil_reset_ops pil_msa_pbl_ops = {
	.proxy_vote = pil_msa_pbl_make_proxy_votes,
	.proxy_unvote = pil_msa_pbl_remove_proxy_votes,
	.auth_and_reset = pil_msa_pbl_reset,
	.shutdown = pil_msa_pbl_shutdown,
};

static int pil_msa_mba_init_image(struct pil_desc *pil,
				  const u8 *metadata, size_t size)
{
	struct mba_data *drv = container_of(pil, struct mba_data, desc);
	void *mdata_virt;
	dma_addr_t mdata_phys;
	s32 status;
	int ret;

	
	mdata_virt = dma_alloc_coherent(pil->dev, size, &mdata_phys,
					GFP_KERNEL);
	if (!mdata_virt) {
		dev_err(pil->dev, "MBA metadata buffer allocation failed\n");
		return -ENOMEM;
	}
	memcpy(mdata_virt, metadata, size);
	
	wmb();

	
	writel_relaxed(0, drv->rmb_base + RMB_PMI_CODE_LENGTH);

	
	writel_relaxed(mdata_phys, drv->rmb_base + RMB_PMI_META_DATA);
	writel_relaxed(CMD_META_DATA_READY, drv->rmb_base + RMB_MBA_COMMAND);
	ret = readl_poll_timeout(drv->rmb_base + RMB_MBA_STATUS, status,
		status == STATUS_META_DATA_AUTH_SUCCESS || status < 0,
		POLL_INTERVAL_US, modem_auth_timeout_ms * 1000);
	if (ret) {
		dev_err(pil->dev, "MBA authentication of headers timed out\n");
	} else if (status < 0) {
		dev_err(pil->dev, "MBA returned error %d for headers\n",
				status);
		ret = -EINVAL;
	}

	dma_free_coherent(pil->dev, size, mdata_virt, mdata_phys);

	return ret;
}

static int pil_msa_mba_verify_blob(struct pil_desc *pil, phys_addr_t phy_addr,
				   size_t size)
{
	struct mba_data *drv = container_of(pil, struct mba_data, desc);
	s32 status;
	u32 img_length = readl_relaxed(drv->rmb_base + RMB_PMI_CODE_LENGTH);

	
	if (img_length == 0) {
		writel_relaxed(phy_addr, drv->rmb_base + RMB_PMI_CODE_START);
		writel_relaxed(CMD_LOAD_READY, drv->rmb_base + RMB_MBA_COMMAND);
	}
	
	img_length += size;
	writel_relaxed(img_length, drv->rmb_base + RMB_PMI_CODE_LENGTH);

	status = readl_relaxed(drv->rmb_base + RMB_MBA_STATUS);
	if (status < 0) {
		dev_err(pil->dev, "MBA returned error %d\n", status);
		return -EINVAL;
	}

	return 0;
}

static int pil_msa_mba_auth(struct pil_desc *pil)
{
	struct mba_data *drv = container_of(pil, struct mba_data, desc);
	int ret;
	s32 status;

	
	ret = readl_poll_timeout(drv->rmb_base + RMB_MBA_STATUS, status,
			status == STATUS_AUTH_COMPLETE || status < 0,
			50, modem_auth_timeout_ms * 1000);
	if (ret) {
		dev_err(pil->dev, "MBA authentication of image timed out\n");
	} else if (status < 0) {
		dev_err(pil->dev, "MBA returned error %d for image\n", status);
		ret = -EINVAL;
	}

	return ret;
}

struct pil_reset_ops pil_msa_mba_ops = {
	.init_image = pil_msa_mba_init_image,
	.verify_blob = pil_msa_mba_verify_blob,
	.auth_and_reset = pil_msa_mba_auth,
};
