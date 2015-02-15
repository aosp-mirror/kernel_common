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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_coresight.h>
#include <linux/coresight.h>
#include <linux/coresight-cti.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/usb/usb_qdss.h>
#include <linux/dma-mapping.h>
#include <mach/sps.h>
#include <mach/usb_bam.h>
#include <mach/msm_memory_dump.h>

#include "coresight-priv.h"

#define tmc_writel(drvdata, val, off)	__raw_writel((val), drvdata->base + off)
#define tmc_readl(drvdata, off)		__raw_readl(drvdata->base + off)

#define tmc_readl_no_log(drvdata, off)	__raw_readl_no_log(drvdata->base + off)

#define TMC_LOCK(drvdata)						\
do {									\
	mb();								\
	tmc_writel(drvdata, 0x0, CORESIGHT_LAR);			\
} while (0)
#define TMC_UNLOCK(drvdata)						\
do {									\
	tmc_writel(drvdata, CORESIGHT_UNLOCK, CORESIGHT_LAR);		\
	mb();								\
} while (0)

#define TMC_RSZ				(0x004)
#define TMC_STS				(0x00C)
#define TMC_RRD				(0x010)
#define TMC_RRP				(0x014)
#define TMC_RWP				(0x018)
#define TMC_TRG				(0x01C)
#define TMC_CTL				(0x020)
#define TMC_RWD				(0x024)
#define TMC_MODE			(0x028)
#define TMC_LBUFLEVEL			(0x02C)
#define TMC_CBUFLEVEL			(0x030)
#define TMC_BUFWM			(0x034)
#define TMC_RRPHI			(0x038)
#define TMC_RWPHI			(0x03C)
#define TMC_AXICTL			(0x110)
#define TMC_DBALO			(0x118)
#define TMC_DBAHI			(0x11C)
#define TMC_FFSR			(0x300)
#define TMC_FFCR			(0x304)
#define TMC_PSCR			(0x308)
#define TMC_ITMISCOP0			(0xEE0)
#define TMC_ITTRFLIN			(0xEE8)
#define TMC_ITATBDATA0			(0xEEC)
#define TMC_ITATBCTR2			(0xEF0)
#define TMC_ITATBCTR1			(0xEF4)
#define TMC_ITATBCTR0			(0xEF8)

#define BYTES_PER_WORD			4
#define TMC_ETR_BAM_PIPE_INDEX		0
#define TMC_ETR_BAM_NR_PIPES		2

#define TMC_ETFETB_DUMP_MAGIC_OFF	(0)
#define TMC_ETFETB_DUMP_MAGIC		(0x5D1DB1BF)
#define TMC_ETFETB_DUMP_VER_OFF		(4)
#define TMC_ETFETB_DUMP_VER		(1)
#define TMC_REG_DUMP_MAGIC_OFF		(0)
#define TMC_REG_DUMP_MAGIC		(0x5D1DB1BF)
#define TMC_REG_DUMP_VER_OFF		(4)
#define TMC_REG_DUMP_VER		(1)

enum tmc_config_type {
	TMC_CONFIG_TYPE_ETB,
	TMC_CONFIG_TYPE_ETR,
	TMC_CONFIG_TYPE_ETF,
};

enum tmc_mode {
	TMC_MODE_CIRCULAR_BUFFER,
	TMC_MODE_SOFTWARE_FIFO,
	TMC_MODE_HARDWARE_FIFO,
};

enum tmc_etr_out_mode {
	TMC_ETR_OUT_MODE_NONE,
	TMC_ETR_OUT_MODE_MEM,
	TMC_ETR_OUT_MODE_USB,
};

enum tmc_mem_intf_width {
	TMC_MEM_INTF_WIDTH_32BITS	= 0x2,
	TMC_MEM_INTF_WIDTH_64BITS	= 0x3,
	TMC_MEM_INTF_WIDTH_128BITS	= 0x4,
	TMC_MEM_INTF_WIDTH_256BITS	= 0x5,
};

struct tmc_etr_bam_data {
	struct sps_bam_props	props;
	uint32_t		handle;
	struct sps_pipe		*pipe;
	struct sps_connect	connect;
	uint32_t		src_pipe_idx;
	uint32_t		dest;
	uint32_t		dest_pipe_idx;
	struct sps_mem_buffer	desc_fifo;
	struct sps_mem_buffer	data_fifo;
	bool			enable;
};

struct tmc_drvdata {
	void __iomem		*base;
	struct device		*dev;
	struct coresight_device	*csdev;
	struct miscdevice	miscdev;
	struct cdev		byte_cntr_dev;
	struct class		*byte_cntr_class;
	struct clk		*clk;
	spinlock_t		spinlock;
	bool			reset_flush_race;
	struct coresight_cti	*cti_flush;
	struct coresight_cti	*cti_reset;
	struct mutex		read_lock;
	int			read_count;
	bool			reading;
	bool			aborting;
	char			*reg_buf;
	char			*buf;
	dma_addr_t		paddr;
	void __iomem		*vaddr;
	uint32_t		size;
	struct mutex		usb_lock;
	struct usb_qdss_ch	*usbch;
	struct tmc_etr_bam_data	*bamdata;
	enum tmc_etr_out_mode	out_mode;
	bool			enable_to_bam;
	bool			enable;
	enum tmc_config_type	config_type;
	uint32_t		trigger_cntr;
	int			byte_cntr_irq;
	atomic_t		byte_cntr_irq_cnt;
	uint32_t		byte_cntr_value;
	struct mutex		byte_cntr_read_lock;
	struct mutex		byte_cntr_lock;
	uint32_t		byte_cntr_block_size;
	bool			byte_cntr_overflow;
	bool			byte_cntr_present;
	bool			byte_cntr_enable;
	uint32_t		byte_cntr_overflow_cnt;
	bool			byte_cntr_read_active;
	wait_queue_head_t	wq;
	char			*byte_cntr_node;
	uint32_t		mem_size;
};

static void tmc_wait_for_flush(struct tmc_drvdata *drvdata)
{
	int count;

	/* Ensure no flush is in progress */
	for (count = TIMEOUT_US; BVAL(tmc_readl(drvdata, TMC_FFSR), 0) != 0
				&& count > 0; count--)
		udelay(1);
	WARN(count == 0, "timeout while waiting for TMC flush, TMC_FFSR: %#x\n",
	     tmc_readl(drvdata, TMC_FFSR));
}

static void tmc_wait_for_ready(struct tmc_drvdata *drvdata)
{
	int count;

	/* Ensure formatter, unformatter and hardware fifo are empty */
	for (count = TIMEOUT_US; BVAL(tmc_readl(drvdata, TMC_STS), 2) != 1
				&& count > 0; count--)
		udelay(1);
	WARN(count == 0, "timeout while waiting for TMC ready, TMC_STS: %#x\n",
	     tmc_readl(drvdata, TMC_STS));
}

static void tmc_flush_and_stop(struct tmc_drvdata *drvdata)
{
	int count;
	uint32_t ffcr;

	ffcr = tmc_readl(drvdata, TMC_FFCR);
	ffcr |= BIT(12);
	tmc_writel(drvdata, ffcr, TMC_FFCR);
	ffcr |= BIT(6);
	tmc_writel(drvdata, ffcr, TMC_FFCR);
	/* Ensure flush completes */
	for (count = TIMEOUT_US; BVAL(tmc_readl(drvdata, TMC_FFCR), 6) != 0
				&& count > 0; count--)
		udelay(1);
	WARN(count == 0, "timeout while flushing TMC, TMC_FFCR: %#x\n",
	     tmc_readl(drvdata, TMC_FFCR));

	tmc_wait_for_ready(drvdata);
}

static void __tmc_enable(struct tmc_drvdata *drvdata)
{
	tmc_writel(drvdata, 0x1, TMC_CTL);
}

static void __tmc_disable(struct tmc_drvdata *drvdata)
{
	tmc_writel(drvdata, 0x0, TMC_CTL);
}

static void tmc_etr_fill_usb_bam_data(struct tmc_drvdata *drvdata)
{
	struct tmc_etr_bam_data *bamdata = drvdata->bamdata;

	get_bam2bam_connection_info(usb_bam_get_qdss_idx(0),
				    &bamdata->dest,
				    &bamdata->dest_pipe_idx,
				    &bamdata->src_pipe_idx,
				    &bamdata->desc_fifo,
				    &bamdata->data_fifo);
}

static void __tmc_etr_enable_to_bam(struct tmc_drvdata *drvdata)
{
	struct tmc_etr_bam_data *bamdata = drvdata->bamdata;
	uint32_t axictl;

	if (drvdata->enable_to_bam)
		return;

	/* Configure and enable required CSR registers */
	msm_qdss_csr_enable_bam_to_usb();

	/* Configure and enable ETR for usb bam output */

	TMC_UNLOCK(drvdata);

	tmc_writel(drvdata, bamdata->data_fifo.size / BYTES_PER_WORD,
		   TMC_RSZ);
	tmc_writel(drvdata, TMC_MODE_CIRCULAR_BUFFER, TMC_MODE);

	axictl = tmc_readl(drvdata, TMC_AXICTL);
	axictl |= (0xF << 8);
	tmc_writel(drvdata, axictl, TMC_AXICTL);
	axictl &= ~(0x1 << 7);
	tmc_writel(drvdata, axictl, TMC_AXICTL);
	axictl = (axictl & ~0x3) | 0x2;
	tmc_writel(drvdata, axictl, TMC_AXICTL);

	tmc_writel(drvdata, (uint32_t)bamdata->data_fifo.phys_base, TMC_DBALO);
	tmc_writel(drvdata, (((uint64_t)bamdata->data_fifo.phys_base) >> 32)
		   & 0xFF, TMC_DBAHI);
	/* Set FOnFlIn for periodic flush */
	tmc_writel(drvdata, 0x133, TMC_FFCR);
	tmc_writel(drvdata, drvdata->trigger_cntr, TMC_TRG);
	__tmc_enable(drvdata);

	TMC_LOCK(drvdata);

	drvdata->enable_to_bam = true;
}

static int tmc_etr_bam_enable(struct tmc_drvdata *drvdata)
{
	struct tmc_etr_bam_data *bamdata = drvdata->bamdata;
	int ret;

	if (bamdata->enable)
		return 0;

	/* Reset bam to start with */
	ret = sps_device_reset(bamdata->handle);
	if (ret)
		goto err0;

	/* Now configure and enable bam */

	bamdata->pipe = sps_alloc_endpoint();
	if (!bamdata->pipe)
		return -ENOMEM;

	ret = sps_get_config(bamdata->pipe, &bamdata->connect);
	if (ret)
		goto err1;

	bamdata->connect.mode = SPS_MODE_SRC;
	bamdata->connect.source = bamdata->handle;
	bamdata->connect.event_thresh = 0x4;
	bamdata->connect.src_pipe_index = TMC_ETR_BAM_PIPE_INDEX;
	bamdata->connect.options = SPS_O_AUTO_ENABLE;

	bamdata->connect.destination = bamdata->dest;
	bamdata->connect.dest_pipe_index = bamdata->dest_pipe_idx;
	bamdata->connect.desc = bamdata->desc_fifo;
	bamdata->connect.data = bamdata->data_fifo;

	ret = sps_connect(bamdata->pipe, &bamdata->connect);
	if (ret)
		goto err1;

	bamdata->enable = true;
	return 0;
err1:
	sps_free_endpoint(bamdata->pipe);
err0:
	return ret;
}

static void __tmc_etr_disable_to_bam(struct tmc_drvdata *drvdata)
{
	if (!drvdata->enable_to_bam)
		return;

	/* Ensure periodic flush is disabled in CSR block */
	msm_qdss_csr_disable_flush();

	TMC_UNLOCK(drvdata);

	tmc_wait_for_flush(drvdata);
	__tmc_disable(drvdata);

	TMC_LOCK(drvdata);

	/* Disable CSR configuration */
	msm_qdss_csr_disable_bam_to_usb();
	drvdata->enable_to_bam = false;
}

static void tmc_etr_bam_disable(struct tmc_drvdata *drvdata)
{
	struct tmc_etr_bam_data *bamdata = drvdata->bamdata;

	if (!bamdata->enable)
		return;

	sps_disconnect(bamdata->pipe);
	sps_free_endpoint(bamdata->pipe);
	bamdata->enable = false;
}

static void usb_notifier(void *priv, unsigned int event,
			struct qdss_request *d_req, struct usb_qdss_ch *ch)
{
	struct tmc_drvdata *drvdata = priv;
	unsigned long flags;
	int ret = 0;

	mutex_lock(&drvdata->usb_lock);
	if (event == USB_QDSS_CONNECT) {
		tmc_etr_fill_usb_bam_data(drvdata);
		ret = tmc_etr_bam_enable(drvdata);
		if (ret)
			dev_err(drvdata->dev, "ETR BAM enable failed\n");

		spin_lock_irqsave(&drvdata->spinlock, flags);
		__tmc_etr_enable_to_bam(drvdata);
		spin_unlock_irqrestore(&drvdata->spinlock, flags);
	} else if (event == USB_QDSS_DISCONNECT) {
		spin_lock_irqsave(&drvdata->spinlock, flags);
		__tmc_etr_disable_to_bam(drvdata);
		spin_unlock_irqrestore(&drvdata->spinlock, flags);
		tmc_etr_bam_disable(drvdata);
	}
	mutex_unlock(&drvdata->usb_lock);
}

static uint32_t tmc_etr_get_write_ptr(struct tmc_drvdata *drvdata)
{
	uint32_t rwp = 0;

	TMC_UNLOCK(drvdata);

	rwp = tmc_readl(drvdata, TMC_RWP);

	TMC_LOCK(drvdata);

	return rwp;
}

static void tmc_etr_byte_cntr_start(struct tmc_drvdata *drvdata)
{
	if (!drvdata->byte_cntr_present)
		return;

	mutex_lock(&drvdata->byte_cntr_lock);
	atomic_set(&drvdata->byte_cntr_irq_cnt, 0);
	drvdata->byte_cntr_overflow = false;
	drvdata->byte_cntr_read_active = false;
	drvdata->byte_cntr_enable = true;
	if (drvdata->byte_cntr_value != 0)
		drvdata->byte_cntr_overflow_cnt = drvdata->size /
						 (drvdata->byte_cntr_value * 8);
	else
		drvdata->byte_cntr_overflow_cnt = 0;
	coresight_csr_set_byte_cntr(drvdata->byte_cntr_value);
	mutex_unlock(&drvdata->byte_cntr_lock);
}

static void tmc_etr_byte_cntr_stop(struct tmc_drvdata *drvdata)
{
	if (!drvdata->byte_cntr_present)
		return;

	mutex_lock(&drvdata->byte_cntr_lock);
	coresight_csr_set_byte_cntr(0);
	drvdata->byte_cntr_value = 0;
	drvdata->byte_cntr_enable = false;
	mutex_unlock(&drvdata->byte_cntr_lock);

	wake_up(&drvdata->wq);
}

static void __tmc_etb_enable(struct tmc_drvdata *drvdata)
{
	/* Zero out the memory to help with debug */
	memset(drvdata->buf, 0, drvdata->size);

	TMC_UNLOCK(drvdata);

	tmc_writel(drvdata, TMC_MODE_CIRCULAR_BUFFER, TMC_MODE);
	tmc_writel(drvdata, 0x1133, TMC_FFCR);
	tmc_writel(drvdata, drvdata->trigger_cntr, TMC_TRG);
	__tmc_enable(drvdata);

	TMC_LOCK(drvdata);
}

static void __tmc_etr_enable_to_mem(struct tmc_drvdata *drvdata)
{
	uint32_t axictl;

	/* Zero out the memory to help with debug */
	memset(drvdata->vaddr, 0, drvdata->size);

	TMC_UNLOCK(drvdata);

	tmc_writel(drvdata, drvdata->size / BYTES_PER_WORD, TMC_RSZ);
	tmc_writel(drvdata, TMC_MODE_CIRCULAR_BUFFER, TMC_MODE);

	axictl = tmc_readl(drvdata, TMC_AXICTL);
	axictl |= (0xF << 8);
	tmc_writel(drvdata, axictl, TMC_AXICTL);
	axictl &= ~(0x1 << 7);
	tmc_writel(drvdata, axictl, TMC_AXICTL);
	axictl = (axictl & ~0x3) | 0x2;
	tmc_writel(drvdata, axictl, TMC_AXICTL);

	tmc_writel(drvdata, (uint32_t)drvdata->paddr, TMC_DBALO);
	tmc_writel(drvdata, (((uint64_t)drvdata->paddr) >> 32) & 0xFF,
		   TMC_DBAHI);
	tmc_writel(drvdata, 0x1133, TMC_FFCR);
	tmc_writel(drvdata, drvdata->trigger_cntr, TMC_TRG);
	__tmc_enable(drvdata);

	TMC_LOCK(drvdata);
}

static void __tmc_etf_enable(struct tmc_drvdata *drvdata)
{
	TMC_UNLOCK(drvdata);

	tmc_writel(drvdata, TMC_MODE_HARDWARE_FIFO, TMC_MODE);
	tmc_writel(drvdata, 0x3, TMC_FFCR);
	tmc_writel(drvdata, 0x0, TMC_BUFWM);
	__tmc_enable(drvdata);

	TMC_LOCK(drvdata);
}

static int tmc_enable(struct tmc_drvdata *drvdata, enum tmc_mode mode)
{
	int ret;
	unsigned long flags;

	ret = clk_prepare_enable(drvdata->clk);
	if (ret)
		return ret;

	mutex_lock(&drvdata->usb_lock);
	if (drvdata->config_type == TMC_CONFIG_TYPE_ETB) {
		coresight_cti_map_trigout(drvdata->cti_flush, 1, 0);
		coresight_cti_map_trigin(drvdata->cti_reset, 0, 0);
	} else if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		if (drvdata->out_mode == TMC_ETR_OUT_MODE_MEM) {
			tmc_etr_byte_cntr_start(drvdata);
			if (!drvdata->reset_flush_race) {
				coresight_cti_map_trigout(drvdata->cti_flush,
							  3, 0);
				coresight_cti_map_trigin(drvdata->cti_reset,
							 2, 0);
			}
		} else if (drvdata->out_mode == TMC_ETR_OUT_MODE_USB) {
			drvdata->usbch = usb_qdss_open("qdss", drvdata,
						       usb_notifier);
			if (IS_ERR(drvdata->usbch)) {
				dev_err(drvdata->dev, "usb_qdss_open failed\n");
				ret = PTR_ERR(drvdata->usbch);
				goto err0;
			}
		}
	} else {
		if (mode == TMC_MODE_CIRCULAR_BUFFER) {
			coresight_cti_map_trigout(drvdata->cti_flush, 1, 0);
			coresight_cti_map_trigin(drvdata->cti_reset, 0, 0);
		}
	}

	spin_lock_irqsave(&drvdata->spinlock, flags);
	if (drvdata->reading) {
		ret = -EBUSY;
		goto err1;
	}

	if (drvdata->config_type == TMC_CONFIG_TYPE_ETB) {
		__tmc_etb_enable(drvdata);
	} else if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		if (drvdata->out_mode == TMC_ETR_OUT_MODE_MEM)
			__tmc_etr_enable_to_mem(drvdata);
	} else {
		if (mode == TMC_MODE_CIRCULAR_BUFFER)
			__tmc_etb_enable(drvdata);
		else
			__tmc_etf_enable(drvdata);
	}
	drvdata->enable = true;
	spin_unlock_irqrestore(&drvdata->spinlock, flags);
	mutex_unlock(&drvdata->usb_lock);

	dev_info(drvdata->dev, "TMC enabled\n");
	return 0;
err1:
	spin_unlock_irqrestore(&drvdata->spinlock, flags);
	if (drvdata->config_type == TMC_CONFIG_TYPE_ETR)
		if (drvdata->out_mode == TMC_ETR_OUT_MODE_USB)
			usb_qdss_close(drvdata->usbch);
err0:
	mutex_unlock(&drvdata->usb_lock);
	clk_disable_unprepare(drvdata->clk);
	return ret;
}

static int tmc_enable_sink(struct coresight_device *csdev)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(csdev->dev.parent);

	return tmc_enable(drvdata, TMC_MODE_CIRCULAR_BUFFER);
}

static int tmc_enable_link(struct coresight_device *csdev, int inport,
			   int outport)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(csdev->dev.parent);

	return tmc_enable(drvdata, TMC_MODE_HARDWARE_FIFO);
}

static void __tmc_reg_dump(struct tmc_drvdata *drvdata)
{
	char *reg_hdr;
	uint32_t *reg_buf;

	if (!drvdata->reg_buf || !drvdata->aborting)
		return;

	reg_hdr = drvdata->reg_buf - PAGE_SIZE;
	reg_buf = (uint32_t *)drvdata->reg_buf;

	reg_buf[1] = tmc_readl(drvdata, TMC_RSZ);
	reg_buf[3] = tmc_readl(drvdata, TMC_STS);
	reg_buf[5] = tmc_readl(drvdata, TMC_RRP);
	reg_buf[6] = tmc_readl(drvdata, TMC_RWP);
	reg_buf[7] = tmc_readl(drvdata, TMC_TRG);
	reg_buf[8] = tmc_readl(drvdata, TMC_CTL);
	reg_buf[10] = tmc_readl(drvdata, TMC_MODE);
	reg_buf[11] = tmc_readl(drvdata, TMC_LBUFLEVEL);
	reg_buf[12] = tmc_readl(drvdata, TMC_CBUFLEVEL);
	reg_buf[13] = tmc_readl(drvdata, TMC_BUFWM);
	if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		reg_buf[14] = tmc_readl(drvdata, TMC_RRPHI);
		reg_buf[15] = tmc_readl(drvdata, TMC_RWPHI);
		reg_buf[68] = tmc_readl(drvdata, TMC_AXICTL);
		reg_buf[70] = tmc_readl(drvdata, TMC_DBALO);
		reg_buf[71] = tmc_readl(drvdata, TMC_DBAHI);
	}
	reg_buf[192] = tmc_readl(drvdata, TMC_FFSR);
	reg_buf[193] = tmc_readl(drvdata, TMC_FFCR);
	reg_buf[194] = tmc_readl(drvdata, TMC_PSCR);
	reg_buf[1000] = tmc_readl(drvdata, CORESIGHT_CLAIMSET);
	reg_buf[1001] = tmc_readl(drvdata, CORESIGHT_CLAIMCLR);
	reg_buf[1005] = tmc_readl(drvdata, CORESIGHT_LSR);
	reg_buf[1006] = tmc_readl(drvdata, CORESIGHT_AUTHSTATUS);
	reg_buf[1010] = tmc_readl(drvdata, CORESIGHT_DEVID);
	reg_buf[1011] = tmc_readl(drvdata, CORESIGHT_DEVTYPE);
	reg_buf[1012] = tmc_readl(drvdata, CORESIGHT_PERIPHIDR4);
	reg_buf[1013] = tmc_readl(drvdata, CORESIGHT_PERIPHIDR5);
	reg_buf[1014] = tmc_readl(drvdata, CORESIGHT_PERIPHIDR6);
	reg_buf[1015] = tmc_readl(drvdata, CORESIGHT_PERIPHIDR7);
	reg_buf[1016] = tmc_readl(drvdata, CORESIGHT_PERIPHIDR0);
	reg_buf[1017] = tmc_readl(drvdata, CORESIGHT_PERIPHIDR1);
	reg_buf[1018] = tmc_readl(drvdata, CORESIGHT_PERIPHIDR2);
	reg_buf[1019] = tmc_readl(drvdata, CORESIGHT_PERIPHIDR3);
	reg_buf[1020] = tmc_readl(drvdata, CORESIGHT_COMPIDR0);
	reg_buf[1021] = tmc_readl(drvdata, CORESIGHT_COMPIDR1);
	reg_buf[1022] = tmc_readl(drvdata, CORESIGHT_COMPIDR2);
	reg_buf[1023] = tmc_readl(drvdata, CORESIGHT_COMPIDR3);

	*(uint32_t *)(reg_hdr + TMC_REG_DUMP_MAGIC_OFF) = TMC_REG_DUMP_MAGIC;
}

static void __tmc_etb_dump(struct tmc_drvdata *drvdata)
{
	enum tmc_mem_intf_width memwidth;
	uint8_t memwords;
	char *hdr;
	char *bufp;
	uint32_t read_data;

	memwidth = BMVAL(tmc_readl(drvdata, CORESIGHT_DEVID), 8, 10);
	if (memwidth == TMC_MEM_INTF_WIDTH_32BITS)
		memwords = 1;
	else if (memwidth == TMC_MEM_INTF_WIDTH_64BITS)
		memwords = 2;
	else if (memwidth == TMC_MEM_INTF_WIDTH_128BITS)
		memwords = 4;
	else
		memwords = 8;

	bufp = drvdata->buf;
	while (1) {
		read_data = tmc_readl_no_log(drvdata, TMC_RRD);
		if (read_data == 0xFFFFFFFF)
			goto out;
		if ((bufp - drvdata->buf) >= drvdata->size) {
			dev_err(drvdata->dev, "ETF-ETB end marker missing\n");
			goto out;
		}
		memcpy(bufp, &read_data, BYTES_PER_WORD);
		bufp += BYTES_PER_WORD;
	}

out:
	if ((bufp - drvdata->buf) % (memwords * BYTES_PER_WORD))
		dev_dbg(drvdata->dev, "ETF-ETB data is not %lx bytes aligned\n",
			(unsigned long) memwords * BYTES_PER_WORD);

	if (drvdata->aborting) {
		hdr = drvdata->buf - PAGE_SIZE;
		*(uint32_t *)(hdr + TMC_ETFETB_DUMP_MAGIC_OFF) =
							TMC_ETFETB_DUMP_MAGIC;
	}
}

static void __tmc_etb_disable(struct tmc_drvdata *drvdata)
{
	TMC_UNLOCK(drvdata);

	tmc_flush_and_stop(drvdata);
	__tmc_etb_dump(drvdata);
	__tmc_reg_dump(drvdata);
	__tmc_disable(drvdata);

	TMC_LOCK(drvdata);
}

static void __tmc_etr_dump(struct tmc_drvdata *drvdata)
{
	uint32_t rwp, rwphi;

	rwp = tmc_readl(drvdata, TMC_RWP);
	rwphi = tmc_readl(drvdata, TMC_RWPHI);

	if (BVAL(tmc_readl(drvdata, TMC_STS), 0))
		drvdata->buf = drvdata->vaddr + rwp - drvdata->paddr;
	else
		drvdata->buf = drvdata->vaddr;
}

static void __tmc_etr_disable_to_mem(struct tmc_drvdata *drvdata)
{
	TMC_UNLOCK(drvdata);

	tmc_flush_and_stop(drvdata);
	__tmc_etr_dump(drvdata);
	__tmc_reg_dump(drvdata);
	__tmc_disable(drvdata);

	TMC_LOCK(drvdata);
}

static void __tmc_etf_disable(struct tmc_drvdata *drvdata)
{
	TMC_UNLOCK(drvdata);

	tmc_flush_and_stop(drvdata);
	__tmc_disable(drvdata);

	TMC_LOCK(drvdata);
}

static void tmc_disable(struct tmc_drvdata *drvdata, enum tmc_mode mode)
{
	unsigned long flags;

	mutex_lock(&drvdata->usb_lock);
	spin_lock_irqsave(&drvdata->spinlock, flags);
	if (drvdata->reading)
		goto out;

	if (drvdata->config_type == TMC_CONFIG_TYPE_ETB) {
		__tmc_etb_disable(drvdata);
	} else if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		if (drvdata->out_mode == TMC_ETR_OUT_MODE_MEM)
			__tmc_etr_disable_to_mem(drvdata);
		else if (drvdata->out_mode == TMC_ETR_OUT_MODE_USB)
			__tmc_etr_disable_to_bam(drvdata);
	} else {
		if (mode == TMC_MODE_CIRCULAR_BUFFER)
			__tmc_etb_disable(drvdata);
		else
			__tmc_etf_disable(drvdata);
	}
	drvdata->enable = false;
	spin_unlock_irqrestore(&drvdata->spinlock, flags);

	if (drvdata->config_type == TMC_CONFIG_TYPE_ETB) {
		coresight_cti_unmap_trigin(drvdata->cti_reset, 0, 0);
		coresight_cti_unmap_trigout(drvdata->cti_flush, 1, 0);
	} else if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		if (drvdata->out_mode == TMC_ETR_OUT_MODE_MEM) {
			tmc_etr_byte_cntr_stop(drvdata);
			if (!drvdata->reset_flush_race) {
				coresight_cti_unmap_trigin(drvdata->cti_reset,
							   2, 0);
				coresight_cti_unmap_trigout(drvdata->cti_flush,
							    3, 0);
			}
		} else if (drvdata->out_mode == TMC_ETR_OUT_MODE_USB) {
			tmc_etr_bam_disable(drvdata);
			usb_qdss_close(drvdata->usbch);
		}
	} else {
		if (mode == TMC_MODE_CIRCULAR_BUFFER) {
			coresight_cti_unmap_trigin(drvdata->cti_reset, 0, 0);
			coresight_cti_unmap_trigout(drvdata->cti_flush, 1, 0);
		}
	}
	mutex_unlock(&drvdata->usb_lock);

	clk_disable_unprepare(drvdata->clk);

	dev_info(drvdata->dev, "TMC disabled\n");
	return;
out:
	drvdata->enable = false;
	spin_unlock_irqrestore(&drvdata->spinlock, flags);
	mutex_unlock(&drvdata->usb_lock);

	clk_disable_unprepare(drvdata->clk);

	dev_info(drvdata->dev, "TMC disabled\n");
}

static void tmc_disable_sink(struct coresight_device *csdev)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(csdev->dev.parent);

	tmc_disable(drvdata, TMC_MODE_CIRCULAR_BUFFER);
}

static void tmc_disable_link(struct coresight_device *csdev, int inport,
			     int outport)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(csdev->dev.parent);

	tmc_disable(drvdata, TMC_MODE_HARDWARE_FIFO);
}

static void tmc_abort(struct coresight_device *csdev)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(csdev->dev.parent);
	unsigned long flags;
	enum tmc_mode mode;

	drvdata->aborting = true;

	spin_lock_irqsave(&drvdata->spinlock, flags);
	if (drvdata->reading)
		goto out0;

	if (drvdata->config_type == TMC_CONFIG_TYPE_ETB) {
		__tmc_etb_disable(drvdata);
	} else if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		if (drvdata->out_mode == TMC_ETR_OUT_MODE_MEM)
			__tmc_etr_disable_to_mem(drvdata);
		else if (drvdata->out_mode == TMC_ETR_OUT_MODE_USB)
			__tmc_etr_disable_to_bam(drvdata);
	} else {
		mode = tmc_readl(drvdata, TMC_MODE);
		if (mode == TMC_MODE_CIRCULAR_BUFFER)
			__tmc_etb_disable(drvdata);
		else
			goto out1;
	}
out0:
	drvdata->enable = false;
	spin_unlock_irqrestore(&drvdata->spinlock, flags);

	dev_info(drvdata->dev, "TMC aborted\n");
	return;
out1:
	spin_unlock_irqrestore(&drvdata->spinlock, flags);
}

static const struct coresight_ops_sink tmc_sink_ops = {
	.enable		= tmc_enable_sink,
	.disable	= tmc_disable_sink,
	.abort		= tmc_abort,
};

static const struct coresight_ops_link tmc_link_ops = {
	.enable		= tmc_enable_link,
	.disable	= tmc_disable_link,
};

static const struct coresight_ops tmc_etb_cs_ops = {
	.sink_ops	= &tmc_sink_ops,
};

static const struct coresight_ops tmc_etr_cs_ops = {
	.sink_ops	= &tmc_sink_ops,
};

static const struct coresight_ops tmc_etf_cs_ops = {
	.sink_ops	= &tmc_sink_ops,
	.link_ops	= &tmc_link_ops,
};

static int tmc_read_prepare(struct tmc_drvdata *drvdata)
{
	int ret;
	unsigned long flags;
	enum tmc_mode mode;

	spin_lock_irqsave(&drvdata->spinlock, flags);
	if (!drvdata->enable)
		goto out;

	if (drvdata->config_type == TMC_CONFIG_TYPE_ETB) {
		__tmc_etb_disable(drvdata);
	} else if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		if (drvdata->out_mode == TMC_ETR_OUT_MODE_MEM) {
			__tmc_etr_disable_to_mem(drvdata);
		} else {
			ret = -ENODEV;
			goto err;
		}
	} else {
		mode = tmc_readl(drvdata, TMC_MODE);
		if (mode == TMC_MODE_CIRCULAR_BUFFER) {
			__tmc_etb_disable(drvdata);
		} else {
			ret = -ENODEV;
			goto err;
		}
	}
out:
	drvdata->reading = true;
	spin_unlock_irqrestore(&drvdata->spinlock, flags);

	dev_info(drvdata->dev, "TMC read start\n");
	return 0;
err:
	spin_unlock_irqrestore(&drvdata->spinlock, flags);
	return ret;
}

static void tmc_read_unprepare(struct tmc_drvdata *drvdata)
{
	unsigned long flags;
	enum tmc_mode mode;

	spin_lock_irqsave(&drvdata->spinlock, flags);
	if (!drvdata->enable)
		goto out;

	if (drvdata->config_type == TMC_CONFIG_TYPE_ETB) {
		__tmc_etb_enable(drvdata);
	} else if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		if (drvdata->out_mode == TMC_ETR_OUT_MODE_MEM)
			__tmc_etr_enable_to_mem(drvdata);
	} else {
		mode = tmc_readl(drvdata, TMC_MODE);
		if (mode == TMC_MODE_CIRCULAR_BUFFER)
			__tmc_etb_enable(drvdata);
	}
out:
	drvdata->reading = false;
	spin_unlock_irqrestore(&drvdata->spinlock, flags);

	dev_info(drvdata->dev, "TMC read end\n");
}

static int tmc_open(struct inode *inode, struct file *file)
{
	struct tmc_drvdata *drvdata = container_of(file->private_data,
						   struct tmc_drvdata, miscdev);
	int ret = 0;

	mutex_lock(&drvdata->read_lock);
	if (drvdata->read_count++)
		goto out;

	ret = tmc_read_prepare(drvdata);
	if (ret)
		goto err;
out:
	mutex_unlock(&drvdata->read_lock);
	nonseekable_open(inode, file);

	dev_dbg(drvdata->dev, "%s: successfully opened\n", __func__);
	return 0;
err:
	drvdata->read_count--;
	mutex_unlock(&drvdata->read_lock);
	return ret;
}

static ssize_t tmc_read(struct file *file, char __user *data, size_t len,
			loff_t *ppos)
{
	struct tmc_drvdata *drvdata = container_of(file->private_data,
						   struct tmc_drvdata, miscdev);
	char *bufp = drvdata->buf + *ppos;

	if (*ppos + len > drvdata->size)
		len = drvdata->size - *ppos;

	if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		if (bufp == (char *)(drvdata->vaddr + drvdata->size))
			bufp = drvdata->vaddr;
		else if (bufp > (char *)(drvdata->vaddr + drvdata->size))
			bufp -= drvdata->size;
		if ((bufp + len) > (char *)(drvdata->vaddr + drvdata->size))
			len = (char *)(drvdata->vaddr + drvdata->size) - bufp;
	}

	if (copy_to_user(data, bufp, len)) {
		dev_dbg(drvdata->dev, "%s: copy_to_user failed\n", __func__);
		return -EFAULT;
	}

	*ppos += len;

	dev_dbg(drvdata->dev, "%s: %d bytes copied, %d bytes left\n",
		__func__, len, (int) (drvdata->size - *ppos));
	return len;
}

static int tmc_release(struct inode *inode, struct file *file)
{
	struct tmc_drvdata *drvdata = container_of(file->private_data,
						   struct tmc_drvdata, miscdev);

	mutex_lock(&drvdata->read_lock);
	if (--drvdata->read_count) {
		if (drvdata->read_count < 0) {
			WARN_ONCE(1, "mismatched close\n");
			drvdata->read_count = 0;
		}
		goto out;
	}

	tmc_read_unprepare(drvdata);
out:
	mutex_unlock(&drvdata->read_lock);
	dev_dbg(drvdata->dev, "%s: released\n", __func__);
	return 0;
}

static const struct file_operations tmc_fops = {
	.owner		= THIS_MODULE,
	.open		= tmc_open,
	.read		= tmc_read,
	.release	= tmc_release,
	.llseek		= no_llseek,
};

static int tmc_etr_byte_cntr_open(struct inode *inode, struct file *file)
{
	struct tmc_drvdata *drvdata = container_of(inode->i_cdev,
						   struct tmc_drvdata,
						   byte_cntr_dev);

	if (drvdata->out_mode != TMC_ETR_OUT_MODE_MEM ||
	    !drvdata->byte_cntr_enable)
		return -EPERM;

	if (!mutex_trylock(&drvdata->byte_cntr_read_lock))
		return -EPERM;

	file->private_data = drvdata;
	nonseekable_open(inode, file);
	drvdata->byte_cntr_block_size = drvdata->byte_cntr_value * 8;
	drvdata->byte_cntr_read_active = true;
	dev_dbg(drvdata->dev, "%s: successfully opened\n", __func__);
	return 0;
}

static void tmc_etr_read_bytes(struct tmc_drvdata *drvdata, loff_t *ppos,
			       size_t bytes, size_t *len)
{
	if (*len >= bytes) {
		atomic_dec(&drvdata->byte_cntr_irq_cnt);
		*len = bytes;
	} else {
		if (((uint32_t)*ppos % bytes) + *len > bytes)
				*len = bytes - ((uint32_t)*ppos % bytes);
		if ((*len + (uint32_t)*ppos) % bytes == 0)
			atomic_dec(&drvdata->byte_cntr_irq_cnt);
	}
}

static size_t tmc_etr_flush_bytes(struct tmc_drvdata *drvdata, loff_t *ppos,
				  size_t bytes)
{
	uint32_t rwp = 0;
	size_t len = bytes;

	rwp = tmc_etr_get_write_ptr(drvdata);
	if (rwp >= (drvdata->paddr + *ppos)) {
		if (len > (rwp - drvdata->paddr - *ppos))
			len = rwp - drvdata->paddr - *ppos;
	}
	return len;
}

static ssize_t tmc_etr_byte_cntr_read(struct file *file, char __user *data,
				  size_t len, loff_t *ppos)
{
	struct tmc_drvdata *drvdata = file->private_data;
	char *bufp = drvdata->vaddr + *ppos;
	size_t bytes = drvdata->byte_cntr_block_size;
	int ret = 0;

	if (!data)
		return -EINVAL;
	if (drvdata->byte_cntr_overflow)
		return -EIO;

	mutex_lock(&drvdata->byte_cntr_lock);
	/* In case the byte counter is enabled and disabled multiple times
	 * prevent unexpected data from being given to the user
	 */
	if (!drvdata->byte_cntr_read_active)
		goto read_err0;

	if (!drvdata->byte_cntr_enable) {
		if (!atomic_read(&drvdata->byte_cntr_irq_cnt)) {
			/* Read the last 'block' of data which might be needed
			 * to be read partially. If already read, return 0
			 */
			len = tmc_etr_flush_bytes(drvdata, ppos, bytes);
			if (!len)
				goto read_err0;
		} else {
			/* Keep reading until you reach the last block of data
			 */
			tmc_etr_read_bytes(drvdata, ppos, bytes, &len);
		}
	} else {
		if (!atomic_read(&drvdata->byte_cntr_irq_cnt)) {
			mutex_unlock(&drvdata->byte_cntr_lock);
			if (wait_event_interruptible(drvdata->wq,
			    (atomic_read(&drvdata->byte_cntr_irq_cnt) > 0) ||
			    !drvdata->byte_cntr_enable)) {
				ret = -ERESTARTSYS;
				goto read_err1;
			}
			mutex_lock(&drvdata->byte_cntr_lock);
			if (!drvdata->byte_cntr_read_active) {
				ret = 0;
				goto read_err0;
			}
		}
		if (drvdata->byte_cntr_overflow) {
			ret = -EIO;
			goto read_err0;
		}
		if (!drvdata->byte_cntr_enable &&
		    !atomic_read(&drvdata->byte_cntr_irq_cnt)) {
			len = tmc_etr_flush_bytes(drvdata, ppos, bytes);
			if (!len) {
				ret = 0;
				goto read_err0;
			}
		} else {
			tmc_etr_read_bytes(drvdata, ppos, bytes, &len);
		}
	}
	if (copy_to_user(data, bufp, len)) {
		mutex_unlock(&drvdata->byte_cntr_lock);
		dev_dbg(drvdata->dev, "%s: copy_to_user failed\n", __func__);
		ret = -EFAULT;
		goto read_err1;
	}
	mutex_unlock(&drvdata->byte_cntr_lock);

	if (*ppos + len >= drvdata->size)
		*ppos = 0;
	else
		*ppos += len;

	dev_dbg(drvdata->dev, "%s: %d bytes copied, %d bytes left\n",
		__func__, len, (int) (drvdata->size - *ppos));
	return len;

read_err0:
	mutex_unlock(&drvdata->byte_cntr_lock);
read_err1:
	return ret;
}

static int tmc_etr_byte_cntr_release(struct inode *inode, struct file *file)
{
	struct tmc_drvdata *drvdata = file->private_data;

	mutex_lock(&drvdata->byte_cntr_lock);
	drvdata->byte_cntr_read_active = false;
	mutex_unlock(&drvdata->byte_cntr_lock);
	mutex_unlock(&drvdata->byte_cntr_read_lock);
	dev_dbg(drvdata->dev, "%s: released\n", __func__);
	return 0;
}

static const struct file_operations byte_cntr_fops = {
	.owner		= THIS_MODULE,
	.open		= tmc_etr_byte_cntr_open,
	.read		= tmc_etr_byte_cntr_read,
	.release	= tmc_etr_byte_cntr_release,
	.llseek		= no_llseek,
};

static ssize_t tmc_show_trigger_cntr(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val = drvdata->trigger_cntr;

	return scnprintf(buf, PAGE_SIZE, "%#lx\n", val);
}

static ssize_t tmc_store_trigger_cntr(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val;

	if (sscanf(buf, "%lx", &val) != 1)
		return -EINVAL;

	drvdata->trigger_cntr = val;
	return size;
}
static DEVICE_ATTR(trigger_cntr, S_IRUGO | S_IWUSR, tmc_show_trigger_cntr,
		   tmc_store_trigger_cntr);

static ssize_t tmc_etr_show_out_mode(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(dev->parent);

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 drvdata->out_mode == TMC_ETR_OUT_MODE_MEM ?
			 "mem" : "usb");
}

static ssize_t tmc_etr_store_out_mode(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(dev->parent);
	char str[10] = "";
	unsigned long flags;
	int ret;

	if (strlen(buf) >= 10)
		return -EINVAL;
	if (sscanf(buf, "%10s", str) != 1)
		return -EINVAL;

	mutex_lock(&drvdata->usb_lock);
	if (!strcmp(str, "mem")) {
		if (drvdata->out_mode == TMC_ETR_OUT_MODE_MEM)
			goto out;

		spin_lock_irqsave(&drvdata->spinlock, flags);
		if (!drvdata->enable) {
			drvdata->out_mode = TMC_ETR_OUT_MODE_MEM;
			spin_unlock_irqrestore(&drvdata->spinlock, flags);
			goto out;
		}
		__tmc_etr_disable_to_bam(drvdata);
		__tmc_etr_enable_to_mem(drvdata);
		drvdata->out_mode = TMC_ETR_OUT_MODE_MEM;
		spin_unlock_irqrestore(&drvdata->spinlock, flags);

		if (!drvdata->reset_flush_race) {
			coresight_cti_map_trigout(drvdata->cti_flush, 3, 0);
			coresight_cti_map_trigin(drvdata->cti_reset, 2, 0);
		}

		tmc_etr_bam_disable(drvdata);
		usb_qdss_close(drvdata->usbch);
	} else if (!strcmp(str, "usb")) {
		if (drvdata->out_mode == TMC_ETR_OUT_MODE_USB)
			goto out;

		spin_lock_irqsave(&drvdata->spinlock, flags);
		if (!drvdata->enable) {
			drvdata->out_mode = TMC_ETR_OUT_MODE_USB;
			spin_unlock_irqrestore(&drvdata->spinlock, flags);
			goto out;
		}
		if (drvdata->reading) {
			ret = -EBUSY;
			goto err1;
		}
		__tmc_etr_disable_to_mem(drvdata);
		drvdata->out_mode = TMC_ETR_OUT_MODE_USB;
		spin_unlock_irqrestore(&drvdata->spinlock, flags);

		if (!drvdata->reset_flush_race) {
			coresight_cti_unmap_trigin(drvdata->cti_reset, 2, 0);
			coresight_cti_unmap_trigout(drvdata->cti_flush, 3, 0);
		}

		drvdata->usbch = usb_qdss_open("qdss", drvdata,
					       usb_notifier);
		if (IS_ERR(drvdata->usbch)) {
			dev_err(drvdata->dev, "usb_qdss_open failed\n");
			ret = PTR_ERR(drvdata->usbch);
			goto err0;
		}
	}
out:
	mutex_unlock(&drvdata->usb_lock);
	return size;
err1:
	spin_unlock_irqrestore(&drvdata->spinlock, flags);
err0:
	mutex_unlock(&drvdata->usb_lock);
	return ret;
}
static DEVICE_ATTR(out_mode, S_IRUGO | S_IWUSR, tmc_etr_show_out_mode,
		   tmc_etr_store_out_mode);

static ssize_t tmc_etr_show_byte_cntr_value(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val = drvdata->byte_cntr_value;

	if (!drvdata->byte_cntr_present)
		return -EPERM;

	return scnprintf(buf, PAGE_SIZE, "%#lx\n", val);
}

static ssize_t tmc_etr_store_byte_cntr_value(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val;

	if (!drvdata->byte_cntr_present || drvdata->byte_cntr_enable)
		return -EPERM;
	if (sscanf(buf, "%lx", &val) != 1)
		return -EINVAL;
	if ((drvdata->size / 8) < val)
		return -EINVAL;
	if (val && drvdata->size % (val * 8) != 0)
		return -EINVAL;

	drvdata->byte_cntr_value = val;
	return size;
}
static DEVICE_ATTR(byte_cntr_value, S_IRUGO | S_IWUSR,
		   tmc_etr_show_byte_cntr_value, tmc_etr_store_byte_cntr_value);

static ssize_t tmc_etr_show_mem_size(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val = drvdata->mem_size;

	return scnprintf(buf, PAGE_SIZE, "%#lx\n", val);
}

static ssize_t tmc_etr_store_mem_size(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct tmc_drvdata *drvdata = dev_get_drvdata(dev->parent);
	unsigned long val;

	if (sscanf(buf, "%lx", &val) != 1)
		return -EINVAL;

	drvdata->mem_size = val;
	return size;
}
static DEVICE_ATTR(mem_size, S_IRUGO | S_IWUSR,
		   tmc_etr_show_mem_size, tmc_etr_store_mem_size);

static struct attribute *tmc_attrs[] = {
	&dev_attr_trigger_cntr.attr,
	NULL,
};

static struct attribute_group tmc_attr_grp = {
	.attrs = tmc_attrs,
};

static struct attribute *tmc_etr_attrs[] = {
	&dev_attr_out_mode.attr,
	&dev_attr_byte_cntr_value.attr,
	&dev_attr_mem_size.attr,
	NULL,
};

static struct attribute_group tmc_etr_attr_grp = {
	.attrs = tmc_etr_attrs,
};

static const struct attribute_group *tmc_etb_attr_grps[] = {
	&tmc_attr_grp,
	NULL,
};

static const struct attribute_group *tmc_etr_attr_grps[] = {
	&tmc_attr_grp,
	&tmc_etr_attr_grp,
	NULL,
};

static const struct attribute_group *tmc_etf_attr_grps[] = {
	&tmc_attr_grp,
	NULL,
};

static int __devinit tmc_etr_bam_init(struct platform_device *pdev,
				      struct tmc_drvdata *drvdata)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct tmc_etr_bam_data *bamdata;

	bamdata = devm_kzalloc(dev, sizeof(*bamdata), GFP_KERNEL);
	if (!bamdata)
		return -ENOMEM;
	drvdata->bamdata = bamdata;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "bam-base");
	if (!res)
		return -ENODEV;

	bamdata->props.phys_addr = res->start;
	bamdata->props.virt_addr = devm_ioremap(dev, res->start,
						resource_size(res));
	if (!bamdata->props.virt_addr)
		return -ENOMEM;
	bamdata->props.virt_size = resource_size(res);

	bamdata->props.event_threshold = 0x4; /* Pipe event threshold */
	bamdata->props.summing_threshold = 0x10; /* BAM event threshold */
	bamdata->props.irq = 0;
	bamdata->props.num_pipes = TMC_ETR_BAM_NR_PIPES;

	return sps_register_bam_device(&bamdata->props, &bamdata->handle);
}

static void tmc_etr_bam_exit(struct tmc_drvdata *drvdata)
{
	struct tmc_etr_bam_data *bamdata = drvdata->bamdata;

	if (!bamdata->handle)
		return;
	sps_deregister_bam_device(bamdata->handle);
}

static irqreturn_t tmc_etr_byte_cntr_irq(int irq, void *data)
{
	struct tmc_drvdata *drvdata = data;

	atomic_inc(&drvdata->byte_cntr_irq_cnt);
	if (atomic_read(&drvdata->byte_cntr_irq_cnt) >
			drvdata->byte_cntr_overflow_cnt) {
		dev_err(drvdata->dev, "Byte counter overflow\n");
		drvdata->byte_cntr_overflow = true;
	}
	wake_up(&drvdata->wq);
	return IRQ_HANDLED;
}

static int tmc_etr_byte_cntr_dev_register(struct tmc_drvdata *drvdata)
{
	int ret;
	struct device *device;
	dev_t dev;

	ret = alloc_chrdev_region(&dev, 0, 1, drvdata->byte_cntr_node);
	if (ret)
		goto err0;

	cdev_init(&drvdata->byte_cntr_dev, &byte_cntr_fops);

	drvdata->byte_cntr_dev.owner = THIS_MODULE;
	drvdata->byte_cntr_dev.ops = &byte_cntr_fops;
	ret = cdev_add(&drvdata->byte_cntr_dev, dev, 1);
	if (ret)
		goto err1;

	drvdata->byte_cntr_class = class_create(THIS_MODULE,
						drvdata->byte_cntr_node);
	if (IS_ERR(drvdata->byte_cntr_class)) {
		ret = PTR_ERR(drvdata->byte_cntr_class);
		goto err2;
	}

	device = device_create(drvdata->byte_cntr_class, NULL,
			       drvdata->byte_cntr_dev.dev, drvdata,
			       drvdata->byte_cntr_node);
	if (IS_ERR(device)) {
		ret = PTR_ERR(device);
		goto err3;
	}

	return 0;
err3:
	class_destroy(drvdata->byte_cntr_class);
err2:
	cdev_del(&drvdata->byte_cntr_dev);
err1:
	unregister_chrdev_region(drvdata->byte_cntr_dev.dev, 1);
err0:
	return ret;
}

static void tmc_etr_byte_cntr_dev_deregister(struct tmc_drvdata *drvdata)
{
	device_destroy(drvdata->byte_cntr_class, drvdata->byte_cntr_dev.dev);
	class_destroy(drvdata->byte_cntr_class);
	cdev_del(&drvdata->byte_cntr_dev);
	unregister_chrdev_region(drvdata->byte_cntr_dev.dev, 1);
}

static int tmc_etr_byte_cntr_init(struct platform_device *pdev,
				  struct tmc_drvdata *drvdata)
{
	int ret = 0;
	size_t node_size = strlen("-stream") + 1;
	char *node_name = (char *)((struct coresight_platform_data *)
			(pdev->dev.platform_data))->name;

	if (!drvdata->byte_cntr_present) {
		dev_info(&pdev->dev, "Byte Counter feature absent\n");
		goto out;
	}

	drvdata->byte_cntr_irq = platform_get_irq_byname(pdev,
							"byte-cntr-irq");
	if (drvdata->byte_cntr_irq < 0) {
		/* Even though this is an error condition, we do not fail
		 * the probe as the byte counter feature is optional
		 */
		dev_err(&pdev->dev, "Byte-cntr-irq not specified\n");
		goto err;
	}

	ret = devm_request_irq(&pdev->dev, drvdata->byte_cntr_irq,
			tmc_etr_byte_cntr_irq,
			IRQF_TRIGGER_RISING | IRQF_SHARED,
			node_name, drvdata);
	if (ret) {
		dev_err(&pdev->dev, "Request irq failed\n");
		goto err;
	}

	init_waitqueue_head(&drvdata->wq);
	node_size += strlen(node_name);

	drvdata->byte_cntr_node = devm_kzalloc(&pdev->dev,
					       node_size, GFP_KERNEL);
	if (!drvdata->byte_cntr_node) {
		dev_err(&pdev->dev, "Byte cntr node name allocation failed\n");
		ret = -ENOMEM;
		goto err;
	}

	strlcpy(drvdata->byte_cntr_node, node_name, node_size);
	strlcat(drvdata->byte_cntr_node, "-stream", node_size);

	ret = tmc_etr_byte_cntr_dev_register(drvdata);
	if (ret) {
		dev_err(&pdev->dev, "Byte cntr node not registered\n");
		goto err;
	}

	dev_info(&pdev->dev, "Byte Counter feature enabled\n");
	return 0;
err:
	drvdata->byte_cntr_present = false;
out:
	return ret;
}

static void tmc_etr_byte_cntr_exit(struct tmc_drvdata *drvdata)
{
	if (drvdata->byte_cntr_present)
		tmc_etr_byte_cntr_dev_deregister(drvdata);
}

static int __devinit tmc_probe(struct platform_device *pdev)
{
	int ret;
	uint32_t devid;
	struct device *dev = &pdev->dev;
	struct coresight_platform_data *pdata;
	struct tmc_drvdata *drvdata;
	struct resource *res;
	uint32_t reg_size;
	static int etfetb_count;
	static int count;
	void *baddr;
	struct msm_client_dump dump;
	struct coresight_cti_data *ctidata;
	struct coresight_desc *desc;

	if (coresight_fuse_access_disabled())
		return -EPERM;

	if (pdev->dev.of_node) {
		pdata = of_get_coresight_platform_data(dev, pdev->dev.of_node);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
		pdev->dev.platform_data = pdata;
	}

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;
	drvdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, drvdata);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "tmc-base");
	if (!res)
		return -ENODEV;
	reg_size = resource_size(res);

	drvdata->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!drvdata->base)
		return -ENOMEM;

	spin_lock_init(&drvdata->spinlock);
	mutex_init(&drvdata->read_lock);
	mutex_init(&drvdata->usb_lock);
	mutex_init(&drvdata->byte_cntr_lock);
	mutex_init(&drvdata->byte_cntr_read_lock);
	atomic_set(&drvdata->byte_cntr_irq_cnt, 0);

	drvdata->clk = devm_clk_get(dev, "core_clk");
	if (IS_ERR(drvdata->clk))
		return PTR_ERR(drvdata->clk);

	ret = clk_set_rate(drvdata->clk, CORESIGHT_CLK_RATE_TRACE);
	if (ret)
		return ret;

	ret = clk_prepare_enable(drvdata->clk);
	if (ret)
		return ret;

	devid = tmc_readl(drvdata, CORESIGHT_DEVID);
	drvdata->config_type = BMVAL(devid, 6, 7);

	if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		if (pdev->dev.of_node) {
			ret = of_property_read_u32(pdev->dev.of_node,
						   "qcom,memory-size",
						   &drvdata->size);
			if (ret) {
				clk_disable_unprepare(drvdata->clk);
				return ret;
			}
		}
	} else {
		drvdata->size = tmc_readl(drvdata, TMC_RSZ) * BYTES_PER_WORD;
	}

	clk_disable_unprepare(drvdata->clk);

	if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		drvdata->vaddr = dma_zalloc_coherent(&pdev->dev, drvdata->size,
						     &drvdata->paddr,
						     GFP_KERNEL);
		if (!drvdata->vaddr)
			return -ENOMEM;
		drvdata->buf = drvdata->vaddr;
		drvdata->out_mode = TMC_ETR_OUT_MODE_MEM;
		if (pdev->dev.of_node)
			drvdata->byte_cntr_present = !of_property_read_bool
						     (pdev->dev.of_node,
						     "qcom,byte-cntr-absent");
		ret = tmc_etr_byte_cntr_init(pdev, drvdata);
		if (ret)
			goto err0;
		ret = tmc_etr_bam_init(pdev, drvdata);
		if (ret)
			goto err1;
	} else {
		baddr = devm_kzalloc(dev, PAGE_SIZE + drvdata->size,
				     GFP_KERNEL);
		if (!baddr)
			return -ENOMEM;
		drvdata->buf = baddr + PAGE_SIZE;
		*(uint32_t *)(baddr + TMC_ETFETB_DUMP_VER_OFF) =
							TMC_ETFETB_DUMP_VER;
		dump.id = MSM_TMC_ETFETB + etfetb_count;
		dump.start_addr = virt_to_phys(baddr);
		dump.end_addr = dump.start_addr + PAGE_SIZE + drvdata->size;
		ret = msm_dump_table_register(&dump);
		/*
		 * Don't free the buffer in case of error since it can still
		 * be used to provide dump collection via the device node or
		 * as part of abort.
		 */
		if (ret)
			dev_info(dev, "TMC ETF-ETB dump setup failed\n");
		etfetb_count++;
	}

	baddr = devm_kzalloc(dev, PAGE_SIZE + reg_size, GFP_KERNEL);
	if (baddr) {
		drvdata->reg_buf = baddr + PAGE_SIZE;
		*(uint32_t *)(baddr + TMC_REG_DUMP_VER_OFF) = TMC_REG_DUMP_VER;
		dump.id = MSM_TMC0_REG + count;
		dump.start_addr = virt_to_phys(baddr);
		dump.end_addr = dump.start_addr + PAGE_SIZE + reg_size;
		ret = msm_dump_table_register(&dump);
		/*
		 * Don't free the buffer in case of error since it can still
		 * be used to dump registers as part of abort to aid post crash
		 * parsing.
		 */
		if (ret)
			dev_info(dev, "TMC REG dump setup failed\n");
	} else {
		dev_info(dev, "TMC REG dump space allocation failed\n");
	}
	count++;

	if (pdev->dev.of_node) {
		drvdata->reset_flush_race = of_property_read_bool(
						pdev->dev.of_node,
						"qcom,reset-flush-race");

		ctidata = of_get_coresight_cti_data(dev, pdev->dev.of_node);
		if (IS_ERR(ctidata)) {
			dev_err(dev, "invalid cti data\n");
		} else if (ctidata && ctidata->nr_ctis == 2) {
			drvdata->cti_flush = coresight_cti_get(
							ctidata->names[0]);
			if (IS_ERR(drvdata->cti_flush))
				dev_err(dev, "failed to get flush cti\n");

			drvdata->cti_reset = coresight_cti_get(
							ctidata->names[1]);
			if (IS_ERR(drvdata->cti_reset))
				dev_err(dev, "failed to get reset cti\n");
		}
	}

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc) {
		ret = -ENOMEM;
		goto err2;
	}
	if (drvdata->config_type == TMC_CONFIG_TYPE_ETB) {
		desc->type = CORESIGHT_DEV_TYPE_SINK;
		desc->subtype.sink_subtype = CORESIGHT_DEV_SUBTYPE_SINK_BUFFER;
		desc->ops = &tmc_etb_cs_ops;
		desc->pdata = pdev->dev.platform_data;
		desc->dev = &pdev->dev;
		desc->groups = tmc_etb_attr_grps;
		desc->owner = THIS_MODULE;
		drvdata->csdev = coresight_register(desc);
		if (IS_ERR(drvdata->csdev)) {
			ret = PTR_ERR(drvdata->csdev);
			goto err2;
		}
	} else if (drvdata->config_type == TMC_CONFIG_TYPE_ETR) {
		desc->type = CORESIGHT_DEV_TYPE_SINK;
		desc->subtype.sink_subtype = CORESIGHT_DEV_SUBTYPE_SINK_BUFFER;
		desc->ops = &tmc_etr_cs_ops;
		desc->pdata = pdev->dev.platform_data;
		desc->dev = &pdev->dev;
		desc->groups = tmc_etr_attr_grps;
		desc->owner = THIS_MODULE;
		drvdata->csdev = coresight_register(desc);
		if (IS_ERR(drvdata->csdev)) {
			ret = PTR_ERR(drvdata->csdev);
			goto err2;
		}
	} else {
		desc->type = CORESIGHT_DEV_TYPE_LINKSINK;
		desc->subtype.sink_subtype = CORESIGHT_DEV_SUBTYPE_SINK_BUFFER;
		desc->subtype.link_subtype = CORESIGHT_DEV_SUBTYPE_LINK_FIFO;
		desc->ops = &tmc_etf_cs_ops;
		desc->pdata = pdev->dev.platform_data;
		desc->dev = &pdev->dev;
		desc->groups = tmc_etf_attr_grps;
		desc->owner = THIS_MODULE;
		drvdata->csdev = coresight_register(desc);
		if (IS_ERR(drvdata->csdev)) {
			ret = PTR_ERR(drvdata->csdev);
			goto err2;
		}
	}

	drvdata->miscdev.name = ((struct coresight_platform_data *)
				 (pdev->dev.platform_data))->name;
	drvdata->miscdev.minor = MISC_DYNAMIC_MINOR;
	drvdata->miscdev.fops = &tmc_fops;
	ret = misc_register(&drvdata->miscdev);
	if (ret)
		goto err3;

	dev_info(dev, "TMC initialized\n");
	return 0;
err3:
	coresight_unregister(drvdata->csdev);
err2:
	tmc_etr_bam_exit(drvdata);
err1:
	tmc_etr_byte_cntr_exit(drvdata);
err0:
	if (drvdata->vaddr)
		dma_free_coherent(&pdev->dev, drvdata->size,
				  drvdata->vaddr,
				  drvdata->paddr);
	return ret;
}

static int __devexit tmc_remove(struct platform_device *pdev)
{
	struct tmc_drvdata *drvdata = platform_get_drvdata(pdev);

	tmc_etr_byte_cntr_exit(drvdata);
	misc_deregister(&drvdata->miscdev);
	coresight_unregister(drvdata->csdev);
	tmc_etr_bam_exit(drvdata);
	if (drvdata->vaddr)
		dma_free_coherent(&pdev->dev, drvdata->size, drvdata->vaddr,
				  drvdata->paddr);
	return 0;
}

static struct of_device_id tmc_match[] = {
	{.compatible = "arm,coresight-tmc"},
	{}
};
EXPORT_COMPAT("arm,coresight-tmc");

static struct platform_driver tmc_driver = {
	.probe          = tmc_probe,
	.remove         = __devexit_p(tmc_remove),
	.driver         = {
		.name   = "coresight-tmc",
		.owner	= THIS_MODULE,
		.of_match_table = tmc_match,
	},
};

static int __init tmc_init(void)
{
	return platform_driver_register(&tmc_driver);
}
module_init(tmc_init);

static void __exit tmc_exit(void)
{
	platform_driver_unregister(&tmc_driver);
}
module_exit(tmc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CoreSight Trace Memory Controller driver");
