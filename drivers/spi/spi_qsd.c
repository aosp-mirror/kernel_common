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
 *
 */
/*
 * SPI driver for Qualcomm MSM platforms
 *
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/remote_spinlock.h>
#include <linux/pm_qos.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/pm_runtime.h>
#include <mach/msm_spi.h>
#include <mach/sps.h>
#include <mach/dma.h>
#include <mach/msm_bus.h>
#include <mach/msm_bus_board.h>
#include "spi_qsd.h"

static int msm_spi_pm_resume_runtime(struct device *device);
static int msm_spi_pm_suspend_runtime(struct device *device);
static inline void msm_spi_dma_unmap_buffers(struct msm_spi *dd);

static inline int msm_spi_configure_gsbi(struct msm_spi *dd,
					struct platform_device *pdev)
{
	struct resource *resource;
	unsigned long   gsbi_mem_phys_addr;
	size_t          gsbi_mem_size;
	void __iomem    *gsbi_base;

	resource  = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!resource)
		return 0;

	gsbi_mem_phys_addr = resource->start;
	gsbi_mem_size = resource_size(resource);
	if (!devm_request_mem_region(&pdev->dev, gsbi_mem_phys_addr,
					gsbi_mem_size, SPI_DRV_NAME))
		return -ENXIO;

	gsbi_base = devm_ioremap(&pdev->dev, gsbi_mem_phys_addr,
					gsbi_mem_size);
	if (!gsbi_base)
		return -ENXIO;

	/* Set GSBI to SPI mode */
	writel_relaxed(GSBI_SPI_CONFIG, gsbi_base + GSBI_CTRL_REG);

	return 0;
}

static inline void msm_spi_register_init(struct msm_spi *dd)
{
	writel_relaxed(0x00000001, dd->base + SPI_SW_RESET);
	msm_spi_set_state(dd, SPI_OP_STATE_RESET);
	writel_relaxed(0x00000000, dd->base + SPI_OPERATIONAL);
	writel_relaxed(0x00000000, dd->base + SPI_CONFIG);
	writel_relaxed(0x00000000, dd->base + SPI_IO_MODES);
	if (dd->qup_ver)
		writel_relaxed(0x00000000, dd->base + QUP_OPERATIONAL_MASK);
}

static inline int msm_spi_request_gpios(struct msm_spi *dd)
{
	int i;
	int result = 0;

	for (i = 0; i < ARRAY_SIZE(spi_rsrcs); ++i) {
		if (dd->spi_gpios[i] >= 0) {
			result = gpio_request(dd->spi_gpios[i], spi_rsrcs[i]);
			if (result) {
				dev_err(dd->dev, "%s: gpio_request for pin %d "
					"failed with error %d\n", __func__,
					dd->spi_gpios[i], result);
				goto error;
			}
		}
	}
	return 0;

error:
	for (; --i >= 0;) {
		if (dd->spi_gpios[i] >= 0)
			gpio_free(dd->spi_gpios[i]);
	}
	return result;
}

static inline void msm_spi_free_gpios(struct msm_spi *dd)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(spi_rsrcs); ++i) {
		if (dd->spi_gpios[i] >= 0)
			gpio_free(dd->spi_gpios[i]);
	}

	for (i = 0; i < ARRAY_SIZE(spi_cs_rsrcs); ++i) {
		if (dd->cs_gpios[i].valid) {
			gpio_free(dd->cs_gpios[i].gpio_num);
			dd->cs_gpios[i].valid = 0;
		}
	}
}

static inline int msm_spi_request_cs_gpio(struct msm_spi *dd)
{
	int cs_num;
	int rc;

	cs_num = dd->cur_msg->spi->chip_select;
	if ((!(dd->cur_msg->spi->mode & SPI_LOOP)) &&
		(!(dd->cs_gpios[cs_num].valid)) &&
		(dd->cs_gpios[cs_num].gpio_num >= 0)) {
		rc = gpio_request(dd->cs_gpios[cs_num].gpio_num,
				spi_cs_rsrcs[cs_num]);
		if (rc) {
			dev_err(dd->dev,
				"gpio_request for pin %d failed,error %d\n",
				dd->cs_gpios[cs_num].gpio_num, rc);
			return rc;
		}
		dd->cs_gpios[cs_num].valid = 1;
	}
	return 0;
}

static inline void msm_spi_free_cs_gpio(struct msm_spi *dd)
{
	int cs_num;

	cs_num = dd->cur_msg->spi->chip_select;
	if (dd->cs_gpios[cs_num].valid) {
		gpio_free(dd->cs_gpios[cs_num].gpio_num);
		dd->cs_gpios[cs_num].valid = 0;
	}
}


/**
 * msm_spi_clk_max_rate: finds the nearest lower rate for a clk
 * @clk the clock for which to find nearest lower rate
 * @rate clock frequency in Hz
 * @return nearest lower rate or negative error value
 *
 * Public clock API extends clk_round_rate which is a ceiling function. This
 * function is a floor function implemented as a binary search using the
 * ceiling function.
 */
static long msm_spi_clk_max_rate(struct clk *clk, unsigned long rate)
{
	long lowest_available, nearest_low, step_size, cur;
	long step_direction = -1;
	long guess = rate;
	int  max_steps = 10;

	cur =  clk_round_rate(clk, rate);
	if (cur == rate)
		return rate;

	/* if we got here then: cur > rate */
	lowest_available =  clk_round_rate(clk, 0);
	if (lowest_available > rate)
		return -EINVAL;

	step_size = (rate - lowest_available) >> 1;
	nearest_low = lowest_available;

	while (max_steps-- && step_size) {
		guess += step_size * step_direction;

		cur =  clk_round_rate(clk, guess);

		if ((cur < rate) && (cur > nearest_low))
			nearest_low = cur;

		/*
		 * if we stepped too far, then start stepping in the other
		 * direction with half the step size
		 */
		if (((cur > rate) && (step_direction > 0))
		 || ((cur < rate) && (step_direction < 0))) {
			step_direction = -step_direction;
			step_size >>= 1;
		 }
	}
	return nearest_low;
}

static void msm_spi_clock_set(struct msm_spi *dd, int speed)
{
	long rate;
	int rc;

	rate = msm_spi_clk_max_rate(dd->clk, speed);
	if (rate < 0) {
		dev_err(dd->dev,
		"%s: no match found for requested clock frequency:%d",
			__func__, speed);
		return;
	}

	rc = clk_set_rate(dd->clk, rate);
	if (!rc)
		dd->clock_speed = rate;
}

static void msm_spi_clk_path_vote(struct msm_spi *dd)
{
	if (dd->clk_path_vote.client_hdl)
		msm_bus_scale_client_update_request(
						dd->clk_path_vote.client_hdl,
						MSM_SPI_CLK_PATH_RESUME_VEC);
}

static void msm_spi_clk_path_unvote(struct msm_spi *dd)
{
	if (dd->clk_path_vote.client_hdl)
		msm_bus_scale_client_update_request(
						dd->clk_path_vote.client_hdl,
						MSM_SPI_CLK_PATH_SUSPEND_VEC);
}

static void msm_spi_clk_path_teardown(struct msm_spi *dd)
{
	if (dd->pdata->active_only)
		msm_spi_clk_path_unvote(dd);

	if (dd->clk_path_vote.client_hdl) {
		msm_bus_scale_unregister_client(dd->clk_path_vote.client_hdl);
		dd->clk_path_vote.client_hdl = 0;
	}
}

/**
 * msm_spi_clk_path_init_structs: internal impl detail of msm_spi_clk_path_init
 *
 * allocates and initilizes the bus scaling vectors.
 */
static int msm_spi_clk_path_init_structs(struct msm_spi *dd)
{
	struct msm_bus_vectors *paths    = NULL;
	struct msm_bus_paths   *usecases = NULL;

	dev_dbg(dd->dev, "initialises path clock voting structs");

	paths = devm_kzalloc(dd->dev, sizeof(*paths) * 2, GFP_KERNEL);
	if (!paths) {
		dev_err(dd->dev,
		"msm_bus_paths.paths memory allocation failed");
		return -ENOMEM;
	}

	usecases = devm_kzalloc(dd->dev, sizeof(*usecases) * 2, GFP_KERNEL);
	if (!usecases) {
		dev_err(dd->dev,
		"msm_bus_scale_pdata.usecases memory allocation failed");
		goto path_init_err;
	}

	dd->clk_path_vote.pdata = devm_kzalloc(dd->dev,
					    sizeof(*dd->clk_path_vote.pdata),
					    GFP_KERNEL);
	if (!dd->clk_path_vote.pdata) {
		dev_err(dd->dev,
		"msm_bus_scale_pdata memory allocation failed");
		goto path_init_err;
	}

	paths[MSM_SPI_CLK_PATH_SUSPEND_VEC] = (struct msm_bus_vectors) {
		.src = dd->pdata->master_id,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	};

	paths[MSM_SPI_CLK_PATH_RESUME_VEC]  = (struct msm_bus_vectors) {
		.src = dd->pdata->master_id,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = MSM_SPI_CLK_PATH_AVRG_BW(dd),
		.ib  = MSM_SPI_CLK_PATH_BRST_BW(dd),
	};

	usecases[MSM_SPI_CLK_PATH_SUSPEND_VEC] = (struct msm_bus_paths) {
		.num_paths = 1,
		.vectors   = &paths[MSM_SPI_CLK_PATH_SUSPEND_VEC],
	};

	usecases[MSM_SPI_CLK_PATH_RESUME_VEC] = (struct msm_bus_paths) {
		.num_paths = 1,
		.vectors   = &paths[MSM_SPI_CLK_PATH_RESUME_VEC],
	};

	*dd->clk_path_vote.pdata = (struct msm_bus_scale_pdata) {
		.active_only  = dd->pdata->active_only,
		.name         = dev_name(dd->dev),
		.num_usecases = 2,
		.usecase      = usecases,
	};

	return 0;

path_init_err:
	devm_kfree(dd->dev, paths);
	devm_kfree(dd->dev, usecases);
	devm_kfree(dd->dev, dd->clk_path_vote.pdata);
	dd->clk_path_vote.pdata = NULL;
	return -ENOMEM;
}

/**
 * msm_spi_clk_path_postponed_register: reg with bus-scaling after it is probed
 *
 * @return zero on success
 *
 * Workaround: SPI driver may be probed before the bus scaling driver. Calling
 * msm_bus_scale_register_client() will fail if the bus scaling driver is not
 * ready yet. Thus, this function should be called not from probe but from a
 * later context. Also, this function may be called more then once before
 * register succeed. At this case only one error message will be logged. At boot
 * time all clocks are on, so earlier SPI transactions should succeed.
 */
static int msm_spi_clk_path_postponed_register(struct msm_spi *dd)
{
	dd->clk_path_vote.client_hdl = msm_bus_scale_register_client(
						dd->clk_path_vote.pdata);

	if (dd->clk_path_vote.client_hdl) {
		if (dd->clk_path_vote.reg_err) {
			/* log a success message if an error msg was logged */
			dd->clk_path_vote.reg_err = false;
			dev_info(dd->dev,
				"msm_bus_scale_register_client(mstr-id:%d "
				"actv-only:%d):0x%x",
				dd->pdata->master_id, dd->pdata->active_only,
				dd->clk_path_vote.client_hdl);
		}

		if (dd->pdata->active_only)
			msm_spi_clk_path_vote(dd);
	} else {
		/* guard to log only one error on multiple failure */
		if (!dd->clk_path_vote.reg_err) {
			dd->clk_path_vote.reg_err = true;

			dev_info(dd->dev,
				"msm_bus_scale_register_client(mstr-id:%d "
				"actv-only:%d):0",
				dd->pdata->master_id, dd->pdata->active_only);
		}
	}

	return dd->clk_path_vote.client_hdl ? 0 : -EAGAIN;
}

static void msm_spi_clk_path_init(struct msm_spi *dd)
{
	/*
	 * bail out if path voting is diabled (master_id == 0) or if it is
	 * already registered (client_hdl != 0)
	 */
	if (!dd->pdata->master_id || dd->clk_path_vote.client_hdl)
		return;

	/* if fail once then try no more */
	if (!dd->clk_path_vote.pdata && msm_spi_clk_path_init_structs(dd)) {
		dd->pdata->master_id = 0;
		return;
	};

	/* on failure try again later */
	if (msm_spi_clk_path_postponed_register(dd))
		return;

	if (dd->pdata->active_only)
		msm_spi_clk_path_vote(dd);
}

static int msm_spi_calculate_size(int *fifo_size,
				  int *block_size,
				  int block,
				  int mult)
{
	int words;

	switch (block) {
	case 0:
		words = 1; /* 4 bytes */
		break;
	case 1:
		words = 4; /* 16 bytes */
		break;
	case 2:
		words = 8; /* 32 bytes */
		break;
	default:
		return -EINVAL;
	}

	switch (mult) {
	case 0:
		*fifo_size = words * 2;
		break;
	case 1:
		*fifo_size = words * 4;
		break;
	case 2:
		*fifo_size = words * 8;
		break;
	case 3:
		*fifo_size = words * 16;
		break;
	default:
		return -EINVAL;
	}

	*block_size = words * sizeof(u32); /* in bytes */
	return 0;
}

static void get_next_transfer(struct msm_spi *dd)
{
	struct spi_transfer *t = dd->cur_transfer;

	if (t->transfer_list.next != &dd->cur_msg->transfers) {
		dd->cur_transfer = list_entry(t->transfer_list.next,
					      struct spi_transfer,
					      transfer_list);
		dd->write_buf          = dd->cur_transfer->tx_buf;
		dd->read_buf           = dd->cur_transfer->rx_buf;
	}
}

static void __init msm_spi_calculate_fifo_size(struct msm_spi *dd)
{
	u32 spi_iom;
	int block;
	int mult;

	spi_iom = readl_relaxed(dd->base + SPI_IO_MODES);

	block = (spi_iom & SPI_IO_M_INPUT_BLOCK_SIZE) >> INPUT_BLOCK_SZ_SHIFT;
	mult = (spi_iom & SPI_IO_M_INPUT_FIFO_SIZE) >> INPUT_FIFO_SZ_SHIFT;
	if (msm_spi_calculate_size(&dd->input_fifo_size, &dd->input_block_size,
				   block, mult)) {
		goto fifo_size_err;
	}

	block = (spi_iom & SPI_IO_M_OUTPUT_BLOCK_SIZE) >> OUTPUT_BLOCK_SZ_SHIFT;
	mult = (spi_iom & SPI_IO_M_OUTPUT_FIFO_SIZE) >> OUTPUT_FIFO_SZ_SHIFT;
	if (msm_spi_calculate_size(&dd->output_fifo_size,
				   &dd->output_block_size, block, mult)) {
		goto fifo_size_err;
	}
	if (dd->qup_ver == SPI_QUP_VERSION_NONE) {
		/* DM mode is not available for this block size */
		if (dd->input_block_size == 4 || dd->output_block_size == 4)
			dd->use_dma = 0;

		if (dd->use_dma) {
			dd->input_burst_size = max(dd->input_block_size,
						DM_BURST_SIZE);
			dd->output_burst_size = max(dd->output_block_size,
						DM_BURST_SIZE);
		}
	}

	return;

fifo_size_err:
	dd->use_dma = 0;
	pr_err("%s: invalid FIFO size, SPI_IO_MODES=0x%x\n", __func__, spi_iom);
	return;
}

static void msm_spi_read_word_from_fifo(struct msm_spi *dd)
{
	u32   data_in;
	int   i;
	int   shift;

	data_in = readl_relaxed(dd->base + SPI_INPUT_FIFO);
	if (dd->read_buf) {
		for (i = 0; (i < dd->bytes_per_word) &&
			     dd->rx_bytes_remaining; i++) {
			/* The data format depends on bytes_per_word:
			   4 bytes: 0x12345678
			   3 bytes: 0x00123456
			   2 bytes: 0x00001234
			   1 byte : 0x00000012
			*/
			shift = 8 * (dd->bytes_per_word - i - 1);
			*dd->read_buf++ = (data_in & (0xFF << shift)) >> shift;
			dd->rx_bytes_remaining--;
		}
	} else {
		if (dd->rx_bytes_remaining >= dd->bytes_per_word)
			dd->rx_bytes_remaining -= dd->bytes_per_word;
		else
			dd->rx_bytes_remaining = 0;
	}

	dd->read_xfr_cnt++;
	if (dd->multi_xfr) {
		if (!dd->rx_bytes_remaining)
			dd->read_xfr_cnt = 0;
		else if ((dd->read_xfr_cnt * dd->bytes_per_word) ==
						dd->read_len) {
			struct spi_transfer *t = dd->cur_rx_transfer;
			if (t->transfer_list.next != &dd->cur_msg->transfers) {
				t = list_entry(t->transfer_list.next,
						struct spi_transfer,
						transfer_list);
				dd->read_buf = t->rx_buf;
				dd->read_len = t->len;
				dd->read_xfr_cnt = 0;
				dd->cur_rx_transfer = t;
			}
		}
	}
}

static inline bool msm_spi_is_valid_state(struct msm_spi *dd)
{
	u32 spi_op = readl_relaxed(dd->base + SPI_STATE);

	return spi_op & SPI_OP_STATE_VALID;
}

static inline void msm_spi_udelay(unsigned long delay_usecs)
{
	/*
	 * For smaller values of delay, context switch time
	 * would negate the usage of usleep
	 */
	if (delay_usecs > 20)
		usleep_range(delay_usecs, delay_usecs);
	else if (delay_usecs)
		udelay(delay_usecs);
}

static inline int msm_spi_wait_valid(struct msm_spi *dd)
{
	unsigned long delay = 0;
	unsigned long timeout = 0;

	if (dd->clock_speed == 0)
		return -EINVAL;
	/*
	 * Based on the SPI clock speed, sufficient time
	 * should be given for the SPI state transition
	 * to occur
	 */
	delay = (10 * USEC_PER_SEC) / dd->clock_speed;
	/*
	 * For small delay values, the default timeout would
	 * be one jiffy
	 */
	if (delay < SPI_DELAY_THRESHOLD)
		delay = SPI_DELAY_THRESHOLD;

	/* Adding one to round off to the nearest jiffy */
	timeout = jiffies + msecs_to_jiffies(delay * SPI_DEFAULT_TIMEOUT) + 1;
	while (!msm_spi_is_valid_state(dd)) {
		if (time_after(jiffies, timeout)) {
			if (!msm_spi_is_valid_state(dd)) {
				if (dd->cur_msg)
					dd->cur_msg->status = -EIO;
				dev_err(dd->dev, "%s: SPI operational state"
					"not valid\n", __func__);
				return -ETIMEDOUT;
			} else
				return 0;
		}
		msm_spi_udelay(delay);
	}
	return 0;
}

static inline int msm_spi_set_state(struct msm_spi *dd,
				    enum msm_spi_state state)
{
	enum msm_spi_state cur_state;
	if (msm_spi_wait_valid(dd))
		return -EIO;
	cur_state = readl_relaxed(dd->base + SPI_STATE);
	/* Per spec:
	   For PAUSE_STATE to RESET_STATE, two writes of (10) are required */
	if (((cur_state & SPI_OP_STATE) == SPI_OP_STATE_PAUSE) &&
			(state == SPI_OP_STATE_RESET)) {
		writel_relaxed(SPI_OP_STATE_CLEAR_BITS, dd->base + SPI_STATE);
		writel_relaxed(SPI_OP_STATE_CLEAR_BITS, dd->base + SPI_STATE);
	} else {
		writel_relaxed((cur_state & ~SPI_OP_STATE) | state,
		       dd->base + SPI_STATE);
	}
	if (msm_spi_wait_valid(dd))
		return -EIO;

	return 0;
}

/**
 * msm_spi_set_bpw_and_no_io_flags: configure N, and no-input/no-output flags
 */
static inline void
msm_spi_set_bpw_and_no_io_flags(struct msm_spi *dd, u32 *config, int n)
{
	*config &= ~(SPI_NO_INPUT|SPI_NO_OUTPUT);

	if (n != (*config & SPI_CFG_N))
		*config = (*config & ~SPI_CFG_N) | n;

	if (((dd->mode == SPI_DMOV_MODE) && (!dd->read_len))
					|| (dd->mode == SPI_BAM_MODE)) {
		if (dd->read_buf == NULL)
			*config |= SPI_NO_INPUT;
		if (dd->write_buf == NULL)
			*config |= SPI_NO_OUTPUT;
	}
}

/**
 * msm_spi_calc_spi_config_loopback_and_input_first: Calculate the values that
 * should be updated into SPI_CONFIG's LOOPBACK and INPUT_FIRST flags
 * @return calculatd value for SPI_CONFIG
 */
static u32
msm_spi_calc_spi_config_loopback_and_input_first(u32 spi_config, u8 mode)
{
	if (mode & SPI_LOOP)
		spi_config |= SPI_CFG_LOOPBACK;
	else
		spi_config &= ~SPI_CFG_LOOPBACK;

	if (mode & SPI_CPHA)
		spi_config &= ~SPI_CFG_INPUT_FIRST;
	else
		spi_config |= SPI_CFG_INPUT_FIRST;

	return spi_config;
}

/**
 * msm_spi_set_spi_config: prepares register SPI_CONFIG to process the
 * next transfer
 */
static void msm_spi_set_spi_config(struct msm_spi *dd, int bpw)
{
	u32 spi_config = readl_relaxed(dd->base + SPI_CONFIG);
	spi_config = msm_spi_calc_spi_config_loopback_and_input_first(
					spi_config, dd->cur_msg->spi->mode);

	if (dd->qup_ver == SPI_QUP_VERSION_NONE)
		/* flags removed from SPI_CONFIG in QUP version-2 */
		msm_spi_set_bpw_and_no_io_flags(dd, &spi_config, bpw-1);

	/*
	 * HS_MODE improves signal stability for spi-clk high rates
	 * but is invalid in LOOPBACK mode.
	 */
	if ((dd->clock_speed >= SPI_HS_MIN_RATE) &&
	   !(dd->cur_msg->spi->mode & SPI_LOOP))
		spi_config |= SPI_CFG_HS_MODE;
	else
		spi_config &= ~SPI_CFG_HS_MODE;

	writel_relaxed(spi_config, dd->base + SPI_CONFIG);
}

/**
 * msm_spi_set_mx_counts: set SPI_MX_INPUT_COUNT and SPI_MX_INPUT_COUNT
 * for FIFO-mode. set SPI_MX_INPUT_COUNT and SPI_MX_OUTPUT_COUNT for
 * BAM and DMOV modes.
 * @n_words The number of reads/writes of size N.
 */
static void msm_spi_set_mx_counts(struct msm_spi *dd, u32 n_words)
{
	/*
	 * n_words cannot exceed fifo_size, and only one READ COUNT
	 * interrupt is generated per transaction, so for transactions
	 * larger than fifo size READ COUNT must be disabled.
	 * For those transactions we usually move to Data Mover mode.
	 */
	if (dd->mode == SPI_FIFO_MODE) {
		if (n_words <= dd->input_fifo_size) {
			writel_relaxed(n_words,
				       dd->base + SPI_MX_READ_COUNT);
			msm_spi_set_write_count(dd, n_words);
		} else {
			writel_relaxed(0, dd->base + SPI_MX_READ_COUNT);
			msm_spi_set_write_count(dd, 0);
		}
		if (dd->qup_ver == SPI_QUP_VERSION_BFAM) {
			/* must be zero for FIFO */
			writel_relaxed(0, dd->base + SPI_MX_INPUT_COUNT);
			writel_relaxed(0, dd->base + SPI_MX_OUTPUT_COUNT);
		}
	} else {
		/* must be zero for BAM and DMOV */
		writel_relaxed(0, dd->base + SPI_MX_READ_COUNT);
		msm_spi_set_write_count(dd, 0);

		/*
		 * for DMA transfers, both QUP_MX_INPUT_COUNT and
		 * QUP_MX_OUTPUT_COUNT must be zero to all cases but one.
		 * That case is a non-balanced transfer when there is
		 * only a read_buf.
		 */
		if (dd->qup_ver == SPI_QUP_VERSION_BFAM) {
			if (dd->write_buf)
				writel_relaxed(0,
						dd->base + SPI_MX_INPUT_COUNT);
			else
				writel_relaxed(n_words,
						dd->base + SPI_MX_INPUT_COUNT);

			writel_relaxed(0, dd->base + SPI_MX_OUTPUT_COUNT);
		}
	}
}

static int msm_spi_bam_pipe_disconnect(struct msm_spi *dd,
						struct msm_spi_bam_pipe  *pipe)
{
	int ret = sps_disconnect(pipe->handle);
	if (ret) {
		dev_dbg(dd->dev, "%s disconnect bam %s pipe failed\n",
							__func__, pipe->name);
		return ret;
	}
	return 0;
}

static int msm_spi_bam_pipe_connect(struct msm_spi *dd,
		struct msm_spi_bam_pipe  *pipe, struct sps_connect *config)
{
	int ret;
	struct sps_register_event event  = {
		.mode      = SPS_TRIGGER_WAIT,
		.options   = SPS_O_EOT,
		.xfer_done = &dd->transfer_complete,
	};

	ret = sps_connect(pipe->handle, config);
	if (ret) {
		dev_err(dd->dev, "%s: sps_connect(%s:0x%p):%d",
				__func__, pipe->name, pipe->handle, ret);
		return ret;
	}

	ret = sps_register_event(pipe->handle, &event);
	if (ret) {
		dev_err(dd->dev, "%s sps_register_event(hndl:0x%p %s):%d",
				__func__, pipe->handle, pipe->name, ret);
		msm_spi_bam_pipe_disconnect(dd, pipe);
		return ret;
	}

	pipe->teardown_required = true;
	return 0;
}


static void msm_spi_bam_pipe_flush(struct msm_spi *dd,
					enum msm_spi_pipe_direction pipe_dir)
{
	struct msm_spi_bam_pipe *pipe = (pipe_dir == SPI_BAM_CONSUMER_PIPE) ?
					(&dd->bam.prod) : (&dd->bam.cons);
	struct sps_connect           config  = pipe->config;
	int    ret;

	ret = msm_spi_bam_pipe_disconnect(dd, pipe);
	if (ret)
		return;

	ret = msm_spi_bam_pipe_connect(dd, pipe, &config);
	if (ret)
		return;
}

static void msm_spi_bam_flush(struct msm_spi *dd)
{
	dev_dbg(dd->dev, "%s flushing bam for recovery\n" , __func__);

	msm_spi_bam_pipe_flush(dd, SPI_BAM_CONSUMER_PIPE);
	msm_spi_bam_pipe_flush(dd, SPI_BAM_PRODUCER_PIPE);
}

static int
msm_spi_bam_process_rx(struct msm_spi *dd, u32 *bytes_to_send, u32 desc_cnt)
{
	int ret = 0;
	u32 data_xfr_size = 0, rem_bc = 0;
	u32 prod_flags = 0;

	rem_bc = dd->cur_rx_transfer->len - dd->bam.curr_rx_bytes_recvd;
	data_xfr_size = (rem_bc < *bytes_to_send) ? rem_bc : *bytes_to_send;

	/*
	 * set flags for last descriptor only
	 */
	if ((desc_cnt == 1)
		|| (*bytes_to_send == data_xfr_size))
		prod_flags = (dd->write_buf)
			? 0 : (SPS_IOVEC_FLAG_EOT | SPS_IOVEC_FLAG_NWD);

	/*
	 * enqueue read buffer in BAM
	 */
	ret = sps_transfer_one(dd->bam.prod.handle,
			dd->cur_rx_transfer->rx_dma
				+ dd->bam.curr_rx_bytes_recvd,
			data_xfr_size, dd, prod_flags);
	if (ret < 0) {
		dev_err(dd->dev,
		"%s: Failed to queue producer BAM transfer",
		__func__);
		return ret;
	}

	dd->bam.curr_rx_bytes_recvd += data_xfr_size;
	*bytes_to_send -= data_xfr_size;
	dd->bam.bam_rx_len -= data_xfr_size;
	return data_xfr_size;
}

static int
msm_spi_bam_process_tx(struct msm_spi *dd, u32 *bytes_to_send, u32 desc_cnt)
{
	int ret = 0;
	u32 data_xfr_size = 0, rem_bc = 0;
	u32 cons_flags = 0;

	rem_bc = dd->cur_tx_transfer->len - dd->bam.curr_tx_bytes_sent;
	data_xfr_size = (rem_bc < *bytes_to_send) ? rem_bc : *bytes_to_send;

	/*
	 * set flags for last descriptor only
	*/
	if ((desc_cnt == 1)
		|| (*bytes_to_send == data_xfr_size))
		cons_flags = SPS_IOVEC_FLAG_EOT | SPS_IOVEC_FLAG_NWD;

	/*
	 * enqueue write buffer in BAM
	 */
	ret = sps_transfer_one(dd->bam.cons.handle,
			dd->cur_tx_transfer->tx_dma
				+ dd->bam.curr_tx_bytes_sent,
			data_xfr_size, dd, cons_flags);
	if (ret < 0) {
		dev_err(dd->dev,
		"%s: Failed to queue consumer BAM transfer",
		__func__);
		return ret;
	}

	dd->bam.curr_tx_bytes_sent	+= data_xfr_size;
	*bytes_to_send	-= data_xfr_size;
	dd->bam.bam_tx_len -= data_xfr_size;
	return data_xfr_size;
}


/**
 * msm_spi_bam_begin_transfer: transfer dd->tx_bytes_remaining bytes
 * using BAM.
 * @brief BAM can transfer SPI_MAX_TRFR_BTWN_RESETS byte at a single
 * transfer. Between transfer QUP must change to reset state. A loop is
 * issuing a single BAM transfer at a time.
 * @return zero on success
 */
static int
msm_spi_bam_begin_transfer(struct msm_spi *dd)
{
	u32 tx_bytes_to_send = 0, rx_bytes_to_recv = 0;
	u32 n_words_xfr;
	s32 ret = 0;
	u32 prod_desc_cnt = SPI_BAM_MAX_DESC_NUM - 1;
	u32 cons_desc_cnt = SPI_BAM_MAX_DESC_NUM - 1;
	u32 byte_count = 0;

	rx_bytes_to_recv = min_t(u32, dd->bam.bam_rx_len,
				SPI_MAX_TRFR_BTWN_RESETS);
	tx_bytes_to_send = min_t(u32, dd->bam.bam_tx_len,
				SPI_MAX_TRFR_BTWN_RESETS);
	n_words_xfr = DIV_ROUND_UP(rx_bytes_to_recv,
				dd->bytes_per_word);

	msm_spi_set_mx_counts(dd, n_words_xfr);
	ret = msm_spi_set_state(dd, SPI_OP_STATE_RUN);
	if (ret < 0) {
		dev_err(dd->dev,
			"%s: Failed to set QUP state to run",
			__func__);
		goto xfr_err;
	}

	while ((rx_bytes_to_recv + tx_bytes_to_send) &&
		((cons_desc_cnt + prod_desc_cnt) > 0)) {
		struct spi_transfer *t = NULL, *next;

		if (dd->read_buf && (prod_desc_cnt > 0)) {
			ret = msm_spi_bam_process_rx(dd, &rx_bytes_to_recv,
							prod_desc_cnt);
			if (ret < 0)
				goto xfr_err;

			if (!(dd->cur_rx_transfer->len
				- dd->bam.curr_rx_bytes_recvd))
				t = dd->cur_rx_transfer;
			prod_desc_cnt--;
		}

		if (dd->write_buf && (cons_desc_cnt > 0)) {
			ret = msm_spi_bam_process_tx(dd, &tx_bytes_to_send,
							cons_desc_cnt);
			if (ret < 0)
				goto xfr_err;

			if (!(dd->cur_tx_transfer->len
				- dd->bam.curr_tx_bytes_sent))
				t = dd->cur_tx_transfer;
			cons_desc_cnt--;
		}

		if (t && (t->transfer_list.next != &dd->cur_msg->transfers)) {
			next = list_entry(t->transfer_list.next,
					struct spi_transfer,
					transfer_list);
			dd->read_buf  = next->rx_buf;
			dd->write_buf = next->tx_buf;
			dd->cur_rx_transfer = next;
			dd->cur_tx_transfer = next;
			dd->bam.curr_rx_bytes_recvd = 0;
			dd->bam.curr_tx_bytes_sent = 0;
		}

		byte_count += ret;
	}

	dd->tx_bytes_remaining -= min_t(u32, byte_count,
						SPI_MAX_TRFR_BTWN_RESETS);
	return 0;
xfr_err:
	return ret;
}

static int
msm_spi_bam_next_transfer(struct msm_spi *dd)
{
	if (dd->mode != SPI_BAM_MODE)
		return 0;

	if (dd->tx_bytes_remaining > 0) {
		init_completion(&dd->transfer_complete);
		if (msm_spi_set_state(dd, SPI_OP_STATE_RESET))
			return 0;
		if ((msm_spi_bam_begin_transfer(dd)) < 0) {
			dev_err(dd->dev, "%s: BAM transfer setup failed\n",
				__func__);
			return 0;
		}
		return 1;
	}
	return 0;
}

static void msm_spi_setup_dm_transfer(struct msm_spi *dd)
{
	dmov_box *box;
	int bytes_to_send, bytes_sent;
	int tx_num_rows, rx_num_rows;
	u32 num_transfers;

	atomic_set(&dd->rx_irq_called, 0);
	atomic_set(&dd->tx_irq_called, 0);
	if (dd->write_len && !dd->read_len) {
		/* WR-WR transfer */
		bytes_sent = dd->cur_msg_len - dd->tx_bytes_remaining;
		dd->write_buf = dd->temp_buf;
	} else {
		bytes_sent = dd->cur_transfer->len - dd->tx_bytes_remaining;
		/* For WR-RD transfer, bytes_sent can be negative */
		if (bytes_sent < 0)
			bytes_sent = 0;
	}
	/* We'll send in chunks of SPI_MAX_LEN if larger than
	 * 4K bytes for targets that have only 12 bits in
	 * QUP_MAX_OUTPUT_CNT register. If the target supports
	 * more than 12bits then we send the data in chunks of
	 * the infinite_mode value that is defined in the
	 * corresponding board file.
	 */
	if (!dd->pdata->infinite_mode)
		dd->max_trfr_len = SPI_MAX_LEN;
	else
		dd->max_trfr_len = (dd->pdata->infinite_mode) *
			   (dd->bytes_per_word);

	bytes_to_send = min_t(u32, dd->tx_bytes_remaining,
			      dd->max_trfr_len);

	num_transfers = DIV_ROUND_UP(bytes_to_send, dd->bytes_per_word);
	dd->tx_unaligned_len = bytes_to_send % dd->output_burst_size;
	dd->rx_unaligned_len = bytes_to_send % dd->input_burst_size;
	tx_num_rows = bytes_to_send / dd->output_burst_size;
	rx_num_rows = bytes_to_send / dd->input_burst_size;

	dd->mode = SPI_DMOV_MODE;

	if (tx_num_rows) {
		/* src in 16 MSB, dst in 16 LSB */
		box = &dd->tx_dmov_cmd->box;
		box->src_row_addr = dd->cur_transfer->tx_dma + bytes_sent;
		box->src_dst_len
			= (dd->output_burst_size << 16) | dd->output_burst_size;
		box->num_rows = (tx_num_rows << 16) | tx_num_rows;
		box->row_offset = (dd->output_burst_size << 16) | 0;

		dd->tx_dmov_cmd->cmd_ptr = CMD_PTR_LP |
				   DMOV_CMD_ADDR(dd->tx_dmov_cmd_dma +
				   offsetof(struct spi_dmov_cmd, box));
	} else {
		dd->tx_dmov_cmd->cmd_ptr = CMD_PTR_LP |
				   DMOV_CMD_ADDR(dd->tx_dmov_cmd_dma +
				   offsetof(struct spi_dmov_cmd, single_pad));
	}

	if (rx_num_rows) {
		/* src in 16 MSB, dst in 16 LSB */
		box = &dd->rx_dmov_cmd->box;
		box->dst_row_addr = dd->cur_transfer->rx_dma + bytes_sent;
		box->src_dst_len
			= (dd->input_burst_size << 16) | dd->input_burst_size;
		box->num_rows = (rx_num_rows << 16) | rx_num_rows;
		box->row_offset = (0 << 16) | dd->input_burst_size;

		dd->rx_dmov_cmd->cmd_ptr = CMD_PTR_LP |
				   DMOV_CMD_ADDR(dd->rx_dmov_cmd_dma +
				   offsetof(struct spi_dmov_cmd, box));
	} else {
		dd->rx_dmov_cmd->cmd_ptr = CMD_PTR_LP |
				   DMOV_CMD_ADDR(dd->rx_dmov_cmd_dma +
				   offsetof(struct spi_dmov_cmd, single_pad));
	}

	if (!dd->tx_unaligned_len) {
		dd->tx_dmov_cmd->box.cmd |= CMD_LC;
	} else {
		dmov_s *tx_cmd = &(dd->tx_dmov_cmd->single_pad);
		u32 tx_offset = dd->cur_transfer->len - dd->tx_unaligned_len;

		if ((dd->multi_xfr) && (dd->read_len <= 0))
			tx_offset = dd->cur_msg_len - dd->tx_unaligned_len;

		dd->tx_dmov_cmd->box.cmd &= ~CMD_LC;

		memset(dd->tx_padding, 0, dd->output_burst_size);
		if (dd->write_buf)
			memcpy(dd->tx_padding, dd->write_buf + tx_offset,
			       dd->tx_unaligned_len);

		tx_cmd->src = dd->tx_padding_dma;
		tx_cmd->len = dd->output_burst_size;
	}

	if (!dd->rx_unaligned_len) {
		dd->rx_dmov_cmd->box.cmd |= CMD_LC;
	} else {
		dmov_s *rx_cmd = &(dd->rx_dmov_cmd->single_pad);
		dd->rx_dmov_cmd->box.cmd &= ~CMD_LC;

		memset(dd->rx_padding, 0, dd->input_burst_size);
		rx_cmd->dst = dd->rx_padding_dma;
		rx_cmd->len = dd->input_burst_size;
	}

	/* This also takes care of the padding dummy buf
	   Since this is set to the correct length, the
	   dummy bytes won't be actually sent */
	if (dd->multi_xfr) {
		u32 write_transfers = 0;
		u32 read_transfers = 0;

		if (dd->write_len > 0) {
			write_transfers = DIV_ROUND_UP(dd->write_len,
						       dd->bytes_per_word);
			writel_relaxed(write_transfers,
				       dd->base + SPI_MX_OUTPUT_COUNT);
		}
		if (dd->read_len > 0) {
			/*
			 *  The read following a write transfer must take
			 *  into account, that the bytes pertaining to
			 *  the write transfer needs to be discarded,
			 *  before the actual read begins.
			 */
			read_transfers = DIV_ROUND_UP(dd->read_len +
						      dd->write_len,
						      dd->bytes_per_word);
			writel_relaxed(read_transfers,
				       dd->base + SPI_MX_INPUT_COUNT);
		}
	} else {
		if (dd->write_buf)
			writel_relaxed(num_transfers,
				       dd->base + SPI_MX_OUTPUT_COUNT);
		if (dd->read_buf)
			writel_relaxed(num_transfers,
				       dd->base + SPI_MX_INPUT_COUNT);
	}
}

static void msm_spi_enqueue_dm_commands(struct msm_spi *dd)
{
	dma_coherent_pre_ops();
	if (dd->write_buf)
		msm_dmov_enqueue_cmd(dd->tx_dma_chan, &dd->tx_hdr);
	if (dd->read_buf)
		msm_dmov_enqueue_cmd(dd->rx_dma_chan, &dd->rx_hdr);
}

/* SPI core on targets that does not support infinite mode can send
   maximum of 4K transfers or 64K transfers depending up on size of
   MAX_OUTPUT_COUNT register, Therefore, we are sending in several
   chunks. Upon completion we send the next chunk, or complete the
   transfer if everything is finished. On targets that support
   infinite mode, we send all the bytes in as single chunk.
*/
static int msm_spi_dm_send_next(struct msm_spi *dd)
{
	/* By now we should have sent all the bytes in FIFO mode,
	 * However to make things right, we'll check anyway.
	 */
	if (dd->mode != SPI_DMOV_MODE)
		return 0;

	/* On targets which does not support infinite mode,
	   We need to send more chunks, if we sent max last time  */
	if (dd->tx_bytes_remaining > dd->max_trfr_len) {
		dd->tx_bytes_remaining -= dd->max_trfr_len;
		if (msm_spi_set_state(dd, SPI_OP_STATE_RESET))
			return 0;
		dd->read_len = dd->write_len = 0;
		msm_spi_setup_dm_transfer(dd);
		msm_spi_enqueue_dm_commands(dd);
		if (msm_spi_set_state(dd, SPI_OP_STATE_RUN))
			return 0;
		return 1;
	} else if (dd->read_len && dd->write_len) {
		dd->tx_bytes_remaining -= dd->cur_transfer->len;
		if (list_is_last(&dd->cur_transfer->transfer_list,
					    &dd->cur_msg->transfers))
			return 0;
		get_next_transfer(dd);
		if (msm_spi_set_state(dd, SPI_OP_STATE_PAUSE))
			return 0;
		dd->tx_bytes_remaining = dd->read_len + dd->write_len;
		dd->read_buf = dd->temp_buf;
		dd->read_len = dd->write_len = -1;
		msm_spi_setup_dm_transfer(dd);
		msm_spi_enqueue_dm_commands(dd);
		if (msm_spi_set_state(dd, SPI_OP_STATE_RUN))
			return 0;
		return 1;
	}
	return 0;
}

static int msm_spi_dma_send_next(struct msm_spi *dd)
{
	int ret = 0;
	if (dd->mode == SPI_DMOV_MODE)
		ret = msm_spi_dm_send_next(dd);
	if (dd->mode == SPI_BAM_MODE)
		ret = msm_spi_bam_next_transfer(dd);
	return ret;
}

static inline void msm_spi_ack_transfer(struct msm_spi *dd)
{
	writel_relaxed(SPI_OP_MAX_INPUT_DONE_FLAG |
		       SPI_OP_MAX_OUTPUT_DONE_FLAG,
		       dd->base + SPI_OPERATIONAL);
	/* Ensure done flag was cleared before proceeding further */
	mb();
}

/* Figure which irq occured and call the relevant functions */
static inline irqreturn_t msm_spi_qup_irq(int irq, void *dev_id)
{
	u32 op, ret = IRQ_NONE;
	struct msm_spi *dd = dev_id;

	if (pm_runtime_suspended(dd->dev)) {
		dev_warn(dd->dev, "QUP: pm runtime suspend, irq:%d\n", irq);
		return ret;
	}
	if (readl_relaxed(dd->base + SPI_ERROR_FLAGS) ||
	    readl_relaxed(dd->base + QUP_ERROR_FLAGS)) {
		struct spi_master *master = dev_get_drvdata(dd->dev);
		ret |= msm_spi_error_irq(irq, master);
	}

	op = readl_relaxed(dd->base + SPI_OPERATIONAL);
	if (op & SPI_OP_INPUT_SERVICE_FLAG) {
		writel_relaxed(SPI_OP_INPUT_SERVICE_FLAG,
			       dd->base + SPI_OPERATIONAL);
		/*
		 * Ensure service flag was cleared before further
		 * processing of interrupt.
		 */
		mb();
		ret |= msm_spi_input_irq(irq, dev_id);
	}

	if (op & SPI_OP_OUTPUT_SERVICE_FLAG) {
		writel_relaxed(SPI_OP_OUTPUT_SERVICE_FLAG,
			       dd->base + SPI_OPERATIONAL);
		/*
		 * Ensure service flag was cleared before further
		 * processing of interrupt.
		 */
		mb();
		ret |= msm_spi_output_irq(irq, dev_id);
	}

	if (dd->done) {
		complete(&dd->transfer_complete);
		dd->done = 0;
	}
	return ret;
}

static irqreturn_t msm_spi_input_irq(int irq, void *dev_id)
{
	struct msm_spi	       *dd = dev_id;

	dd->stat_rx++;

	if (dd->mode == SPI_MODE_NONE)
		return IRQ_HANDLED;

	if (dd->mode == SPI_DMOV_MODE) {
		u32 op = readl_relaxed(dd->base + SPI_OPERATIONAL);
		if ((!dd->read_buf || op & SPI_OP_MAX_INPUT_DONE_FLAG) &&
		    (!dd->write_buf || op & SPI_OP_MAX_OUTPUT_DONE_FLAG)) {
			msm_spi_ack_transfer(dd);
				if (atomic_inc_return(&dd->rx_irq_called) == 1)
					return IRQ_HANDLED;
			msm_spi_complete(dd);
			return IRQ_HANDLED;
		}
		return IRQ_NONE;
	}

	if (dd->mode == SPI_FIFO_MODE) {
		while ((readl_relaxed(dd->base + SPI_OPERATIONAL) &
			SPI_OP_IP_FIFO_NOT_EMPTY) &&
			(dd->rx_bytes_remaining > 0)) {
			msm_spi_read_word_from_fifo(dd);
		}
		if (dd->rx_bytes_remaining == 0)
			msm_spi_complete(dd);
	}

	return IRQ_HANDLED;
}

static void msm_spi_write_word_to_fifo(struct msm_spi *dd)
{
	u32    word;
	u8     byte;
	int    i;

	word = 0;
	if (dd->write_buf) {
		for (i = 0; (i < dd->bytes_per_word) &&
			     dd->tx_bytes_remaining; i++) {
			dd->tx_bytes_remaining--;
			byte = *dd->write_buf++;
			word |= (byte << (BITS_PER_BYTE * (3 - i)));
		}
	} else
		if (dd->tx_bytes_remaining > dd->bytes_per_word)
			dd->tx_bytes_remaining -= dd->bytes_per_word;
		else
			dd->tx_bytes_remaining = 0;
	dd->write_xfr_cnt++;
	if (dd->multi_xfr) {
		if (!dd->tx_bytes_remaining)
			dd->write_xfr_cnt = 0;
		else if ((dd->write_xfr_cnt * dd->bytes_per_word) ==
						dd->write_len) {
			struct spi_transfer *t = dd->cur_tx_transfer;
			if (t->transfer_list.next != &dd->cur_msg->transfers) {
				t = list_entry(t->transfer_list.next,
						struct spi_transfer,
						transfer_list);
				dd->write_buf = t->tx_buf;
				dd->write_len = t->len;
				dd->write_xfr_cnt = 0;
				dd->cur_tx_transfer = t;
			}
		}
	}
	writel_relaxed(word, dd->base + SPI_OUTPUT_FIFO);
}

static inline void msm_spi_write_rmn_to_fifo(struct msm_spi *dd)
{
	int count = 0;

	while ((dd->tx_bytes_remaining > 0) && (count < dd->input_fifo_size) &&
	       !(readl_relaxed(dd->base + SPI_OPERATIONAL) &
		SPI_OP_OUTPUT_FIFO_FULL)) {
		msm_spi_write_word_to_fifo(dd);
		count++;
	}
}

static irqreturn_t msm_spi_output_irq(int irq, void *dev_id)
{
	struct msm_spi	       *dd = dev_id;

	dd->stat_tx++;

	if (dd->mode == SPI_MODE_NONE)
		return IRQ_HANDLED;

	if (dd->mode == SPI_DMOV_MODE) {
		/* TX_ONLY transaction is handled here
		   This is the only place we send complete at tx and not rx */
		if (dd->read_buf == NULL &&
		    readl_relaxed(dd->base + SPI_OPERATIONAL) &
		    SPI_OP_MAX_OUTPUT_DONE_FLAG) {
			msm_spi_ack_transfer(dd);
			if (atomic_inc_return(&dd->tx_irq_called) == 1)
				return IRQ_HANDLED;
			msm_spi_complete(dd);
			return IRQ_HANDLED;
		}
		return IRQ_NONE;
	}

	/* Output FIFO is empty. Transmit any outstanding write data. */
	if (dd->mode == SPI_FIFO_MODE)
		msm_spi_write_rmn_to_fifo(dd);

	return IRQ_HANDLED;
}

static irqreturn_t msm_spi_error_irq(int irq, void *dev_id)
{
	struct spi_master	*master = dev_id;
	struct msm_spi          *dd = spi_master_get_devdata(master);
	u32                      spi_err;

	spi_err = readl_relaxed(dd->base + SPI_ERROR_FLAGS);
	if (spi_err & SPI_ERR_OUTPUT_OVER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI output overrun error\n");
	if (spi_err & SPI_ERR_INPUT_UNDER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI input underrun error\n");
	if (spi_err & SPI_ERR_OUTPUT_UNDER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI output underrun error\n");
	msm_spi_get_clk_err(dd, &spi_err);
	if (spi_err & SPI_ERR_CLK_OVER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI clock overrun error\n");
	if (spi_err & SPI_ERR_CLK_UNDER_RUN_ERR)
		dev_warn(master->dev.parent, "SPI clock underrun error\n");
	msm_spi_clear_error_flags(dd);
	msm_spi_ack_clk_err(dd);
	/* Ensure clearing of QUP_ERROR_FLAGS was completed */
	mb();
	return IRQ_HANDLED;
}

/**
 * msm_spi_dmov_map_buffers: prepares buffer for DMA transfer
 * @return zero on success or negative error code
 *
 * calls dma_map_single() on the read/write buffers, effectively invalidating
 * their cash entries. for For WR-WR and WR-RD transfers, allocates temporary
 * buffer and copy the data to/from the client buffers
 */
static int msm_spi_dmov_map_buffers(struct msm_spi *dd)
{
	struct device *dev;
	struct spi_transfer *first_xfr;
	struct spi_transfer *nxt_xfr = NULL;
	void *tx_buf, *rx_buf;
	unsigned tx_len, rx_len;
	int ret = -EINVAL;

	dev = &dd->cur_msg->spi->dev;
	first_xfr = dd->cur_transfer;
	tx_buf = (void *)first_xfr->tx_buf;
	rx_buf = first_xfr->rx_buf;
	tx_len = rx_len = first_xfr->len;

	/*
	 * For WR-WR and WR-RD transfers, we allocate our own temporary
	 * buffer and copy the data to/from the client buffers.
	 */
	if (!dd->qup_ver && dd->multi_xfr) {
		dd->temp_buf = kzalloc(dd->cur_msg_len,
				       GFP_KERNEL | __GFP_DMA);
		if (!dd->temp_buf)
			return -ENOMEM;
		nxt_xfr = list_entry(first_xfr->transfer_list.next,
				     struct spi_transfer, transfer_list);

		if (dd->write_len && !dd->read_len) {
			if (!first_xfr->tx_buf || !nxt_xfr->tx_buf)
				goto error;

			memcpy(dd->temp_buf, first_xfr->tx_buf, first_xfr->len);
			memcpy(dd->temp_buf + first_xfr->len, nxt_xfr->tx_buf,
			       nxt_xfr->len);
			tx_buf = dd->temp_buf;
			tx_len = dd->cur_msg_len;
		} else {
			if (!first_xfr->tx_buf || !nxt_xfr->rx_buf)
				goto error;

			rx_buf = dd->temp_buf;
			rx_len = dd->cur_msg_len;
		}
	}
	if (tx_buf != NULL) {
		first_xfr->tx_dma = dma_map_single(dev, tx_buf,
						   tx_len, DMA_TO_DEVICE);
		if (dma_mapping_error(NULL, first_xfr->tx_dma)) {
			dev_err(dev, "dma %cX %d bytes error\n",
				'T', tx_len);
			ret = -ENOMEM;
			goto error;
		}
	}
	if (rx_buf != NULL) {
		dma_addr_t dma_handle;
		dma_handle = dma_map_single(dev, rx_buf,
					    rx_len, DMA_FROM_DEVICE);
		if (dma_mapping_error(NULL, dma_handle)) {
			dev_err(dev, "dma %cX %d bytes error\n",
				'R', rx_len);
			if (tx_buf != NULL)
				dma_unmap_single(NULL, first_xfr->tx_dma,
						 tx_len, DMA_TO_DEVICE);
			ret = -ENOMEM;
			goto error;
		}
		if (dd->multi_xfr)
			nxt_xfr->rx_dma = dma_handle;
		else
			first_xfr->rx_dma = dma_handle;
	}
	return 0;

error:
	kfree(dd->temp_buf);
	dd->temp_buf = NULL;
	return ret;
}

static int msm_spi_bam_map_buffers(struct msm_spi *dd)
{
	int ret = -EINVAL;
	struct device *dev;
	struct spi_transfer *first_xfr;
	struct spi_transfer *nxt_xfr;
	void *tx_buf, *rx_buf;
	u32 tx_len, rx_len;
	int num_xfrs_grped = dd->num_xfrs_grped;

	dev = &dd->cur_msg->spi->dev;
	first_xfr = dd->cur_transfer;

	do {
		tx_buf = (void *)first_xfr->tx_buf;
		rx_buf = first_xfr->rx_buf;
		tx_len = rx_len = first_xfr->len;
		if (tx_buf != NULL) {
			first_xfr->tx_dma = dma_map_single(dev, tx_buf,
							tx_len, DMA_TO_DEVICE);
			if (dma_mapping_error(NULL, first_xfr->tx_dma)) {
				ret = -ENOMEM;
				goto error;
			}
		}

		if (rx_buf != NULL) {
			first_xfr->rx_dma = dma_map_single(dev, rx_buf,	rx_len,
							DMA_FROM_DEVICE);
			if (dma_mapping_error(NULL, first_xfr->rx_dma)) {
				if (tx_buf != NULL)
					dma_unmap_single(NULL,
							first_xfr->tx_dma,
							tx_len, DMA_TO_DEVICE);
				ret = -ENOMEM;
				goto error;
			}
		}

		nxt_xfr = list_entry(first_xfr->transfer_list.next,
				struct spi_transfer, transfer_list);

		if (nxt_xfr == NULL)
			break;
		num_xfrs_grped--;
		first_xfr = nxt_xfr;
	} while (num_xfrs_grped > 0);

	return 0;
error:
	msm_spi_dma_unmap_buffers(dd);
	return ret;
}

static int msm_spi_dma_map_buffers(struct msm_spi *dd)
{
	int ret = 0;
	if (dd->mode == SPI_DMOV_MODE)
		ret = msm_spi_dmov_map_buffers(dd);
	else if (dd->mode == SPI_BAM_MODE)
		ret = msm_spi_bam_map_buffers(dd);
	return ret;
}

static void msm_spi_dmov_unmap_buffers(struct msm_spi *dd)
{
	struct device *dev;
	u32 offset;

	dev = &dd->cur_msg->spi->dev;
	if (dd->cur_msg->is_dma_mapped)
		goto unmap_end;

	if (dd->multi_xfr) {
		if (dd->write_len && !dd->read_len) {
			dma_unmap_single(dev,
					 dd->cur_transfer->tx_dma,
					 dd->cur_msg_len,
					 DMA_TO_DEVICE);
		} else {
			struct spi_transfer *prev_xfr;
			prev_xfr = list_entry(
				   dd->cur_transfer->transfer_list.prev,
				   struct spi_transfer,
				   transfer_list);
			if (dd->cur_transfer->rx_buf) {
				dma_unmap_single(dev,
						 dd->cur_transfer->rx_dma,
						 dd->cur_msg_len,
						 DMA_FROM_DEVICE);
			}
			if (prev_xfr->tx_buf) {
				dma_unmap_single(dev,
						 prev_xfr->tx_dma,
						 prev_xfr->len,
						 DMA_TO_DEVICE);
			}
			if (dd->rx_unaligned_len && dd->read_buf) {
				offset = dd->cur_msg_len - dd->rx_unaligned_len;
				dma_coherent_post_ops();
				memcpy(dd->read_buf + offset, dd->rx_padding,
				       dd->rx_unaligned_len);
				if (dd->cur_transfer->rx_buf)
					memcpy(dd->cur_transfer->rx_buf,
					       dd->read_buf + prev_xfr->len,
					       dd->cur_transfer->len);
			}
		}
		kfree(dd->temp_buf);
		dd->temp_buf = NULL;
		return;
	} else {
		if (dd->cur_transfer->rx_buf)
			dma_unmap_single(dev, dd->cur_transfer->rx_dma,
					 dd->cur_transfer->len,
					 DMA_FROM_DEVICE);
		if (dd->cur_transfer->tx_buf)
			dma_unmap_single(dev, dd->cur_transfer->tx_dma,
					 dd->cur_transfer->len,
					 DMA_TO_DEVICE);
	}

unmap_end:
	/* If we padded the transfer, we copy it from the padding buf */
	if (dd->rx_unaligned_len && dd->read_buf) {
		offset = dd->cur_transfer->len - dd->rx_unaligned_len;
		dma_coherent_post_ops();
		memcpy(dd->read_buf + offset, dd->rx_padding,
		       dd->rx_unaligned_len);
	}
}

static void msm_spi_bam_unmap_buffers(struct msm_spi *dd)
{
	struct device *dev;
	int num_xfrs_grped = dd->num_xfrs_grped;
	struct spi_transfer *first_xfr;
	struct spi_transfer *nxt_xfr;
	void *tx_buf, *rx_buf;
	u32  tx_len, rx_len;

	dev = &dd->cur_msg->spi->dev;
	first_xfr = dd->cur_transfer;

	 /* mapped by client */
	if (dd->cur_msg->is_dma_mapped)
		return;

	do {
		tx_buf = (void *)first_xfr->tx_buf;
		rx_buf = first_xfr->rx_buf;
		tx_len = rx_len = first_xfr->len;
		if (tx_buf != NULL)
			dma_unmap_single(dev, first_xfr->tx_dma,
					tx_len, DMA_TO_DEVICE);

		if (rx_buf != NULL)
			dma_unmap_single(dev, first_xfr->rx_dma,
					rx_len, DMA_FROM_DEVICE);

		nxt_xfr = list_entry(first_xfr->transfer_list.next,
				struct spi_transfer, transfer_list);

		if (nxt_xfr == NULL)
			break;
		num_xfrs_grped--;
		first_xfr = nxt_xfr;
	} while (num_xfrs_grped > 0);
}

static inline void msm_spi_dma_unmap_buffers(struct msm_spi *dd)
{
	if (dd->mode == SPI_DMOV_MODE)
		msm_spi_dmov_unmap_buffers(dd);
	else if (dd->mode == SPI_BAM_MODE)
		msm_spi_bam_unmap_buffers(dd);
}

/**
 * msm_spi_use_dma - decides whether to use Data-Mover or BAM for
 * the given transfer
 * @dd:       device
 * @tr:       transfer
 *
 * Start using DMA if:
 * 1. Is supported by HW
 * 2. Is not diabled by platfrom data
 * 3. Transfer size is greater than 3*block size.
 * 4. Buffers are aligned to cache line.
 * 5. Bytes-per-word is 8,16 or 32.
  */
static inline bool
msm_spi_use_dma(struct msm_spi *dd, struct spi_transfer *tr, u8 bpw)
{
	if (!dd->use_dma)
		return false;

	/* check constraints from platform data */
	if ((dd->qup_ver == SPI_QUP_VERSION_BFAM) && !dd->pdata->use_bam)
		return false;

	if (dd->cur_msg_len < 3*dd->input_block_size)
		return false;

	if ((dd->qup_ver != SPI_QUP_VERSION_BFAM) &&
		dd->multi_xfr && !dd->read_len && !dd->write_len)
		return false;

	if (dd->qup_ver == SPI_QUP_VERSION_NONE) {
		u32 cache_line = dma_get_cache_alignment();

		if (tr->tx_buf) {
			if (!IS_ALIGNED((size_t)tr->tx_buf, cache_line))
				return 0;
		}
		if (tr->rx_buf) {
			if (!IS_ALIGNED((size_t)tr->rx_buf, cache_line))
				return false;
		}

		if (tr->cs_change &&
		   ((bpw != 8) && (bpw != 16) && (bpw != 32)))
			return false;
	}

	return true;
}

/**
 * msm_spi_set_transfer_mode: Chooses optimal transfer mode. Sets dd->mode and
 * prepares to process a transfer.
 */
static void
msm_spi_set_transfer_mode(struct msm_spi *dd, u8 bpw, u32 read_count)
{
	if (msm_spi_use_dma(dd, dd->cur_transfer, bpw)) {
		if (dd->qup_ver) {
			dd->mode = SPI_BAM_MODE;
		} else {
			dd->mode = SPI_DMOV_MODE;
			if (dd->write_len && dd->read_len) {
				dd->tx_bytes_remaining = dd->write_len;
				dd->rx_bytes_remaining = dd->read_len;
			}
		}
	} else {
		dd->mode = SPI_FIFO_MODE;
		if (dd->multi_xfr) {
			dd->read_len = dd->cur_transfer->len;
			dd->write_len = dd->cur_transfer->len;
		}
	}
}

/**
 * msm_spi_set_qup_io_modes: prepares register QUP_IO_MODES to process a
 * transfer
 */
static void msm_spi_set_qup_io_modes(struct msm_spi *dd)
{
	u32 spi_iom;
	spi_iom = readl_relaxed(dd->base + SPI_IO_MODES);
	/* Set input and output transfer mode: FIFO, DMOV, or BAM */
	spi_iom &= ~(SPI_IO_M_INPUT_MODE | SPI_IO_M_OUTPUT_MODE);
	spi_iom = (spi_iom | (dd->mode << OUTPUT_MODE_SHIFT));
	spi_iom = (spi_iom | (dd->mode << INPUT_MODE_SHIFT));
	/* Turn on packing for data mover */
	if ((dd->mode == SPI_DMOV_MODE) || (dd->mode == SPI_BAM_MODE))
		spi_iom |= SPI_IO_M_PACK_EN | SPI_IO_M_UNPACK_EN;
	else
		spi_iom &= ~(SPI_IO_M_PACK_EN | SPI_IO_M_UNPACK_EN);

	/*if (dd->mode == SPI_BAM_MODE) {
		spi_iom |= SPI_IO_C_NO_TRI_STATE;
		spi_iom &= ~(SPI_IO_C_CS_SELECT | SPI_IO_C_CS_N_POLARITY);
	}*/
	writel_relaxed(spi_iom, dd->base + SPI_IO_MODES);
}

static u32 msm_spi_calc_spi_ioc_clk_polarity(u32 spi_ioc, u8 mode)
{
	if (mode & SPI_CPOL)
		spi_ioc |= SPI_IO_C_CLK_IDLE_HIGH;
	else
		spi_ioc &= ~SPI_IO_C_CLK_IDLE_HIGH;
	return spi_ioc;
}

/**
 * msm_spi_set_spi_io_control: prepares register SPI_IO_CONTROL to process the
 * next transfer
 * @return the new set value of SPI_IO_CONTROL
 */
static u32 msm_spi_set_spi_io_control(struct msm_spi *dd)
{
	u32 spi_ioc, spi_ioc_orig, chip_select;

	spi_ioc = readl_relaxed(dd->base + SPI_IO_CONTROL);
	spi_ioc_orig = spi_ioc;
	spi_ioc = msm_spi_calc_spi_ioc_clk_polarity(spi_ioc
						, dd->cur_msg->spi->mode);
	/* Set chip-select */
	chip_select = dd->cur_msg->spi->chip_select << 2;
	if ((spi_ioc & SPI_IO_C_CS_SELECT) != chip_select)
		spi_ioc = (spi_ioc & ~SPI_IO_C_CS_SELECT) | chip_select;
	if (!dd->cur_transfer->cs_change)
		spi_ioc |= SPI_IO_C_MX_CS_MODE;

	if (spi_ioc != spi_ioc_orig)
		writel_relaxed(spi_ioc, dd->base + SPI_IO_CONTROL);

	return spi_ioc;
}

/**
 * msm_spi_set_qup_op_mask: prepares register QUP_OPERATIONAL_MASK to process
 * the next transfer
 */
static void msm_spi_set_qup_op_mask(struct msm_spi *dd)
{
	/* mask INPUT and OUTPUT service flags in to prevent IRQs on FIFO status
	 * change in BAM mode */
	u32 mask = (dd->mode == SPI_BAM_MODE) ?
		QUP_OP_MASK_OUTPUT_SERVICE_FLAG | QUP_OP_MASK_INPUT_SERVICE_FLAG
		: 0;
	writel_relaxed(mask, dd->base + QUP_OPERATIONAL_MASK);
}

static void msm_spi_process_transfer(struct msm_spi *dd)
{
	u8  bpw;
	u32 max_speed;
	u32 read_count;
	u32 timeout;
	u32 spi_ioc;
	u32 int_loopback = 0;

	dd->tx_bytes_remaining = dd->cur_msg_len;
	dd->rx_bytes_remaining = dd->cur_msg_len;
	dd->read_buf           = dd->cur_transfer->rx_buf;
	dd->write_buf          = dd->cur_transfer->tx_buf;
	init_completion(&dd->transfer_complete);
	if (dd->cur_transfer->bits_per_word)
		bpw = dd->cur_transfer->bits_per_word;
	else
		if (dd->cur_msg->spi->bits_per_word)
			bpw = dd->cur_msg->spi->bits_per_word;
		else
			bpw = 8;
	dd->bytes_per_word = (bpw + 7) / 8;

	if (dd->cur_transfer->speed_hz)
		max_speed = dd->cur_transfer->speed_hz;
	else
		max_speed = dd->cur_msg->spi->max_speed_hz;
	if (!dd->clock_speed || max_speed != dd->clock_speed)
		msm_spi_clock_set(dd, max_speed);

	timeout = 100 * msecs_to_jiffies(
			DIV_ROUND_UP(dd->cur_msg_len * 8,
			DIV_ROUND_UP(max_speed, MSEC_PER_SEC)));

	read_count = DIV_ROUND_UP(dd->cur_msg_len, dd->bytes_per_word);
	if (dd->cur_msg->spi->mode & SPI_LOOP)
		int_loopback = 1;
	if (int_loopback && dd->multi_xfr &&
			(read_count > dd->input_fifo_size)) {
		if (dd->read_len && dd->write_len)
			pr_err(
			"%s:Internal Loopback does not support > fifo size"
			"for write-then-read transactions\n",
			__func__);
		else if (dd->write_len && !dd->read_len)
			pr_err(
			"%s:Internal Loopback does not support > fifo size"
			"for write-then-write transactions\n",
			__func__);
		return;
	}

	if (msm_spi_set_state(dd, SPI_OP_STATE_RESET))
		dev_err(dd->dev,
			"%s: Error setting QUP to reset-state",
			__func__);

	msm_spi_set_transfer_mode(dd, bpw, read_count);
	msm_spi_set_mx_counts(dd, read_count);
	if (dd->mode == SPI_DMOV_MODE) {
		if (msm_spi_dma_map_buffers(dd) < 0) {
			pr_err("Mapping DMA buffers\n");
			return;
			}
	} else if (dd->mode == SPI_BAM_MODE) {
			if (msm_spi_dma_map_buffers(dd) < 0) {
				pr_err("Mapping DMA buffers\n");
				return;
		}
	}
	msm_spi_set_qup_io_modes(dd);
	msm_spi_set_spi_config(dd, bpw);
	msm_spi_set_qup_config(dd, bpw);
	spi_ioc = msm_spi_set_spi_io_control(dd);
	msm_spi_set_qup_op_mask(dd);

	if (dd->mode == SPI_DMOV_MODE) {
		msm_spi_setup_dm_transfer(dd);
		msm_spi_enqueue_dm_commands(dd);
	}
	/* The output fifo interrupt handler will handle all writes after
	   the first. Restricting this to one write avoids contention
	   issues and race conditions between this thread and the int handler
	*/
	else if (dd->mode == SPI_FIFO_MODE) {
		if (msm_spi_prepare_for_write(dd))
			goto transfer_end;
		msm_spi_start_write(dd, read_count);
	} else if (dd->mode == SPI_BAM_MODE) {
		if ((msm_spi_bam_begin_transfer(dd)) < 0) {
			dev_err(dd->dev, "%s: BAM transfer setup failed\n",
				__func__);
			dd->cur_msg->status = -EIO;
			goto transfer_end;
		}
	}

	/*
	 * On BAM mode, current state here is run.
	 * Only enter the RUN state after the first word is written into
	 * the output FIFO. Otherwise, the output FIFO EMPTY interrupt
	 * might fire before the first word is written resulting in a
	 * possible race condition.
	 */
	if (dd->mode != SPI_BAM_MODE)
		if (msm_spi_set_state(dd, SPI_OP_STATE_RUN)) {
			dev_warn(dd->dev,
				"%s: Failed to set QUP to run-state. Mode:%d",
				__func__, dd->mode);
			goto transfer_end;
		}

	/* Assume success, this might change later upon transaction result */
	dd->cur_msg->status = 0;
	do {
		if (!wait_for_completion_timeout(&dd->transfer_complete,
						 timeout)) {
				dev_err(dd->dev,
					"%s: SPI transaction timeout\n",
					__func__);
				dd->cur_msg->status = -EIO;
				if (dd->mode == SPI_DMOV_MODE) {
					msm_dmov_flush(dd->tx_dma_chan, 1);
					msm_dmov_flush(dd->rx_dma_chan, 1);
				}
				if (dd->mode == SPI_BAM_MODE)
					msm_spi_bam_flush(dd);
				break;
		}
	} while (msm_spi_dma_send_next(dd));

	msm_spi_udelay(dd->xfrs_delay_usec);

transfer_end:
	if (dd->mode == SPI_BAM_MODE)
		msm_spi_bam_flush(dd);
	msm_spi_dma_unmap_buffers(dd);
	dd->mode = SPI_MODE_NONE;

	msm_spi_set_state(dd, SPI_OP_STATE_RESET);
	writel_relaxed(spi_ioc & ~SPI_IO_C_MX_CS_MODE,
		       dd->base + SPI_IO_CONTROL);
}

static void get_transfer_length(struct msm_spi *dd)
{
	struct spi_transfer *tr;
	int num_xfrs = 0;
	int readlen = 0;
	int writelen = 0;

	dd->cur_msg_len = 0;
	dd->multi_xfr = 0;
	dd->read_len = dd->write_len = 0;

	list_for_each_entry(tr, &dd->cur_msg->transfers, transfer_list) {
		if (tr->tx_buf)
			writelen += tr->len;
		if (tr->rx_buf)
			readlen += tr->len;
		dd->cur_msg_len += tr->len;
		num_xfrs++;
	}

	if (num_xfrs == 2) {
		struct spi_transfer *first_xfr = dd->cur_transfer;

		dd->multi_xfr = 1;
		tr = list_entry(first_xfr->transfer_list.next,
				struct spi_transfer,
				transfer_list);
		/*
		 * We update dd->read_len and dd->write_len only
		 * for WR-WR and WR-RD transfers.
		 */
		if ((first_xfr->tx_buf) && (!first_xfr->rx_buf)) {
			if (((tr->tx_buf) && (!tr->rx_buf)) ||
			    ((!tr->tx_buf) && (tr->rx_buf))) {
				dd->read_len = readlen;
				dd->write_len = writelen;
			}
		}
	} else if (num_xfrs > 1)
		dd->multi_xfr = 1;
}

static inline void write_force_cs(struct msm_spi *dd, bool set_flag)
{
	u32 spi_ioc;
	u32 spi_ioc_orig;

	spi_ioc = readl_relaxed(dd->base + SPI_IO_CONTROL);
	spi_ioc_orig = spi_ioc;
	if (set_flag)
		spi_ioc |= SPI_IO_C_FORCE_CS;
	else
		spi_ioc &= ~SPI_IO_C_FORCE_CS;

	if (spi_ioc != spi_ioc_orig)
		writel_relaxed(spi_ioc, dd->base + SPI_IO_CONTROL);
}

static inline int combine_transfers(struct msm_spi *dd)
{
	struct spi_transfer *t = dd->cur_transfer;
	struct spi_transfer *nxt;
	int xfrs_grped = 1;
	dd->xfrs_delay_usec = 0;

	dd->bam.bam_rx_len = dd->bam.bam_tx_len = 0;

	dd->cur_msg_len = dd->cur_transfer->len;

	if (dd->cur_transfer->tx_buf)
		dd->bam.bam_tx_len += dd->cur_transfer->len;
	if (dd->cur_transfer->rx_buf)
		dd->bam.bam_rx_len += dd->cur_transfer->len;

	while (t->transfer_list.next != &dd->cur_msg->transfers) {
		nxt = list_entry(t->transfer_list.next,
				 struct spi_transfer,
				 transfer_list);
		if (t->cs_change != nxt->cs_change)
			return xfrs_grped;
		if (t->delay_usecs) {
			dd->xfrs_delay_usec = t->delay_usecs;
			dev_info(dd->dev, "SPI slave requests delay per txn :%d usecs",
					t->delay_usecs);
			return xfrs_grped;
		}
		if (nxt->tx_buf)
			dd->bam.bam_tx_len += nxt->len;
		if (nxt->rx_buf)
			dd->bam.bam_rx_len += nxt->len;

		dd->cur_msg_len += nxt->len;
		xfrs_grped++;
		t = nxt;
	}

	if (1 == xfrs_grped)
		dd->xfrs_delay_usec = dd->cur_transfer->delay_usecs;

	return xfrs_grped;
}

static void msm_spi_process_message(struct msm_spi *dd)
{
	int xfrs_grped = 0;
	int rc;

	dd->num_xfrs_grped = 0;
	dd->bam.curr_rx_bytes_recvd = dd->bam.curr_tx_bytes_sent = 0;
	dd->write_xfr_cnt = dd->read_xfr_cnt = 0;
	rc = msm_spi_request_cs_gpio(dd);
	if (rc)
		return;

	dd->cur_transfer = list_first_entry(&dd->cur_msg->transfers,
						struct spi_transfer,
						transfer_list);

	get_transfer_length(dd);
	if (dd->qup_ver || (dd->multi_xfr && !dd->read_len && !dd->write_len)) {

		if (dd->qup_ver)
			write_force_cs(dd, 0);

		/*
		 * Handling of multi-transfers.
		 * FIFO mode is used by default
		 */
		list_for_each_entry(dd->cur_transfer,
					&dd->cur_msg->transfers,
					transfer_list) {
			if (!dd->cur_transfer->len)
				goto error;
			if (xfrs_grped) {
				xfrs_grped--;
				continue;
			} else {
				dd->read_len = dd->write_len = 0;
				xfrs_grped = combine_transfers(dd);
				dd->num_xfrs_grped = xfrs_grped;
				if (dd->qup_ver)
					write_force_cs(dd, 1);
			}

			dd->cur_tx_transfer = dd->cur_transfer;
			dd->cur_rx_transfer = dd->cur_transfer;
			msm_spi_process_transfer(dd);
			if (dd->qup_ver && !dd->xfrs_delay_usec)
				write_force_cs(dd, 0);
			xfrs_grped--;
		}
	} else {
		/* Handling of a single transfer or
		 * WR-WR or WR-RD transfers
		 */
		if ((!dd->cur_msg->is_dma_mapped) &&
			(msm_spi_use_dma(dd, dd->cur_transfer,
				dd->cur_transfer->bits_per_word))) {
			/* Mapping of DMA buffers */
			int ret = msm_spi_dma_map_buffers(dd);
			if (ret < 0) {
				dd->cur_msg->status = ret;
				goto error;
			}
		}

		dd->cur_tx_transfer = dd->cur_transfer;
		dd->cur_rx_transfer = dd->cur_transfer;
		dd->num_xfrs_grped = 1;
		msm_spi_process_transfer(dd);
	}

	return;

error:
	msm_spi_free_cs_gpio(dd);
}

/**
 * msm_spi_transfer_one_message: To process one spi message at a time
 * @master: spi master controller reference
 * @msg: one multi-segment SPI transaction
 * @return zero on success or negative error value
 *
 */
static int msm_spi_transfer_one_message(struct spi_master *master,
					  struct spi_message *msg)
{
	struct msm_spi	*dd;
	struct spi_transfer *tr;
	unsigned long        flags;
	u32	status_error = 0;

	dd = spi_master_get_devdata(master);

	if (list_empty(&msg->transfers) || !msg->complete)
		return -EINVAL;

	list_for_each_entry(tr, &msg->transfers, transfer_list) {
		/* Check message parameters */
		if (tr->speed_hz > dd->pdata->max_clock_speed ||
		    (tr->bits_per_word &&
		     (tr->bits_per_word < 4 || tr->bits_per_word > 32)) ||
		    (tr->tx_buf == NULL && tr->rx_buf == NULL)) {
			dev_err(dd->dev,
				"Invalid transfer: %d Hz, %d bpw tx=%p, rx=%p\n",
				tr->speed_hz, tr->bits_per_word,
				tr->tx_buf, tr->rx_buf);
			status_error = -EINVAL;
			goto out;
		}
	}

	mutex_lock(&dd->core_lock);

	/*
	 * Counter-part of system-suspend when runtime-pm is not enabled.
	 * This way, resume can be left empty and device will be put in
	 * active mode only if client requests anything on the bus
	 */
	if (!pm_runtime_enabled(dd->dev))
		msm_spi_pm_resume_runtime(dd->dev);

	if (dd->use_rlock)
		remote_mutex_lock(&dd->r_lock);

	spin_lock_irqsave(&dd->queue_lock, flags);
	dd->transfer_pending = 1;
	spin_unlock_irqrestore(&dd->queue_lock, flags);

	if (dd->suspended || !msm_spi_is_valid_state(dd)) {
		dev_err(dd->dev, "%s: SPI operational state not valid\n",
			__func__);
		status_error = 1;
	}
	spin_lock_irqsave(&dd->queue_lock, flags);
	dd->transfer_pending = 1;
	dd->cur_msg = msg;
	spin_unlock_irqrestore(&dd->queue_lock, flags);

	if (status_error)
			dd->cur_msg->status = -EIO;
	else
		msm_spi_process_message(dd);

	spin_lock_irqsave(&dd->queue_lock, flags);
	dd->transfer_pending = 0;
	spin_unlock_irqrestore(&dd->queue_lock, flags);

	if (dd->use_rlock)
		remote_mutex_unlock(&dd->r_lock);

	mutex_unlock(&dd->core_lock);

	/*
	 * If needed, this can be done after the current message is complete,
	 * and work can be continued upon resume. No motivation for now.
	 */
	if (dd->suspended)
		wake_up_interruptible(&dd->continue_suspend);

out:
	dd->cur_msg->status = status_error;
	spi_finalize_current_message(master);
	return 0;
}

static int msm_spi_prepare_transfer_hardware(struct spi_master *master)
{
	struct msm_spi	*dd = spi_master_get_devdata(master);

	pm_runtime_get_sync(dd->dev);
	return 0;
}

static int msm_spi_unprepare_transfer_hardware(struct spi_master *master)
{
	struct msm_spi	*dd = spi_master_get_devdata(master);

	pm_runtime_mark_last_busy(dd->dev);
	pm_runtime_put_autosuspend(dd->dev);
	return 0;
}

static int msm_spi_setup(struct spi_device *spi)
{
	struct msm_spi	*dd;
	int              rc = 0;
	u32              spi_ioc;
	u32              spi_config;
	u32              mask;

	if (spi->bits_per_word < 4 || spi->bits_per_word > 32) {
		dev_err(&spi->dev, "%s: invalid bits_per_word %d\n",
			__func__, spi->bits_per_word);
		rc = -EINVAL;
	}
	if (spi->chip_select > SPI_NUM_CHIPSELECTS-1) {
		dev_err(&spi->dev, "%s, chip select %d exceeds max value %d\n",
			__func__, spi->chip_select, SPI_NUM_CHIPSELECTS - 1);
		rc = -EINVAL;
	}

	if (rc)
		goto err_setup_exit;

	dd = spi_master_get_devdata(spi->master);

	pm_runtime_get_sync(dd->dev);

	mutex_lock(&dd->core_lock);

	/* Counter-part of system-suspend when runtime-pm is not enabled. */
	if (!pm_runtime_enabled(dd->dev))
		msm_spi_pm_resume_runtime(dd->dev);

	if (dd->suspended) {
		mutex_unlock(&dd->core_lock);
		return -EBUSY;
	}

	if (dd->use_rlock)
		remote_mutex_lock(&dd->r_lock);

	spi_ioc = readl_relaxed(dd->base + SPI_IO_CONTROL);
	mask = SPI_IO_C_CS_N_POLARITY_0 << spi->chip_select;
	if (spi->mode & SPI_CS_HIGH)
		spi_ioc |= mask;
	else
		spi_ioc &= ~mask;
	spi_ioc = msm_spi_calc_spi_ioc_clk_polarity(spi_ioc, spi->mode);

	writel_relaxed(spi_ioc, dd->base + SPI_IO_CONTROL);

	spi_config = readl_relaxed(dd->base + SPI_CONFIG);
	spi_config = msm_spi_calc_spi_config_loopback_and_input_first(
							spi_config, spi->mode);
	writel_relaxed(spi_config, dd->base + SPI_CONFIG);

	/* Ensure previous write completed before disabling the clocks */
	mb();

	if (dd->use_rlock)
		remote_mutex_unlock(&dd->r_lock);

	/* Counter-part of system-resume when runtime-pm is not enabled. */
	if (!pm_runtime_enabled(dd->dev))
		msm_spi_pm_suspend_runtime(dd->dev);

	mutex_unlock(&dd->core_lock);

	pm_runtime_mark_last_busy(dd->dev);
	pm_runtime_put_autosuspend(dd->dev);

err_setup_exit:
	return rc;
}

#ifdef CONFIG_DEBUG_FS
static int debugfs_iomem_x32_set(void *data, u64 val)
{
	writel_relaxed(val, data);
	/* Ensure the previous write completed. */
	mb();
	return 0;
}

static int debugfs_iomem_x32_get(void *data, u64 *val)
{
	*val = readl_relaxed(data);
	/* Ensure the previous read completed. */
	mb();
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_iomem_x32, debugfs_iomem_x32_get,
			debugfs_iomem_x32_set, "0x%08llx\n");

static void spi_debugfs_init(struct msm_spi *dd)
{
	dd->dent_spi = debugfs_create_dir(dev_name(dd->dev), NULL);
	if (dd->dent_spi) {
		int i;

		for (i = 0; i < ARRAY_SIZE(debugfs_spi_regs); i++) {
			dd->debugfs_spi_regs[i] =
			   debugfs_create_file(
			       debugfs_spi_regs[i].name,
			       debugfs_spi_regs[i].mode,
			       dd->dent_spi,
			       dd->base + debugfs_spi_regs[i].offset,
			       &fops_iomem_x32);
		}
	}
}

static void spi_debugfs_exit(struct msm_spi *dd)
{
	if (dd->dent_spi) {
		int i;

		debugfs_remove_recursive(dd->dent_spi);
		dd->dent_spi = NULL;
		for (i = 0; i < ARRAY_SIZE(debugfs_spi_regs); i++)
			dd->debugfs_spi_regs[i] = NULL;
	}
}
#else
static void spi_debugfs_init(struct msm_spi *dd) {}
static void spi_debugfs_exit(struct msm_spi *dd) {}
#endif

/* ===Device attributes begin=== */
static ssize_t show_stats(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct msm_spi *dd =  spi_master_get_devdata(master);

	return snprintf(buf, PAGE_SIZE,
			"Device       %s\n"
			"rx fifo_size = %d spi words\n"
			"tx fifo_size = %d spi words\n"
			"use_dma ?    %s\n"
			"rx block size = %d bytes\n"
			"tx block size = %d bytes\n"
			"input burst size = %d bytes\n"
			"output burst size = %d bytes\n"
			"DMA configuration:\n"
			"tx_ch=%d, rx_ch=%d, tx_crci= %d, rx_crci=%d\n"
			"--statistics--\n"
			"Rx isrs  = %d\n"
			"Tx isrs  = %d\n"
			"DMA error  = %d\n"
			"--debug--\n"
			"NA yet\n",
			dev_name(dev),
			dd->input_fifo_size,
			dd->output_fifo_size,
			dd->use_dma ? "yes" : "no",
			dd->input_block_size,
			dd->output_block_size,
			dd->input_burst_size,
			dd->output_burst_size,
			dd->tx_dma_chan,
			dd->rx_dma_chan,
			dd->tx_dma_crci,
			dd->rx_dma_crci,
			dd->stat_rx + dd->stat_dmov_rx,
			dd->stat_tx + dd->stat_dmov_tx,
			dd->stat_dmov_tx_err + dd->stat_dmov_rx_err
			);
}

/* Reset statistics on write */
static ssize_t set_stats(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct msm_spi *dd = dev_get_drvdata(dev);
	dd->stat_rx = 0;
	dd->stat_tx = 0;
	dd->stat_dmov_rx = 0;
	dd->stat_dmov_tx = 0;
	dd->stat_dmov_rx_err = 0;
	dd->stat_dmov_tx_err = 0;
	return count;
}

static DEVICE_ATTR(stats, S_IRUGO | S_IWUSR, show_stats, set_stats);

static struct attribute *dev_attrs[] = {
	&dev_attr_stats.attr,
	NULL,
};

static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};
/* ===Device attributes end=== */

/**
 * spi_dmov_tx_complete_func - DataMover tx completion callback
 *
 * Executed in IRQ context (Data Mover's IRQ) DataMover's
 * spinlock @msm_dmov_lock held.
 */
static void spi_dmov_tx_complete_func(struct msm_dmov_cmd *cmd,
				      unsigned int result,
				      struct msm_dmov_errdata *err)
{
	struct msm_spi *dd;

	if (!(result & DMOV_RSLT_VALID)) {
		pr_err("Invalid DMOV result: rc=0x%08x, cmd = %p", result, cmd);
		return;
	}
	/* restore original context */
	dd = container_of(cmd, struct msm_spi, tx_hdr);
	if (result & DMOV_RSLT_DONE) {
		dd->stat_dmov_tx++;
		if ((atomic_inc_return(&dd->tx_irq_called) == 1))
			return;
		complete(&dd->transfer_complete);
	} else {
		/* Error or flush */
		if (result & DMOV_RSLT_ERROR) {
			dev_err(dd->dev, "DMA error (0x%08x)\n", result);
			dd->stat_dmov_tx_err++;
		}
		if (result & DMOV_RSLT_FLUSH) {
			/*
			 * Flushing normally happens in process of
			 * removing, when we are waiting for outstanding
			 * DMA commands to be flushed.
			 */
			dev_info(dd->dev,
				 "DMA channel flushed (0x%08x)\n", result);
		}
		if (err)
			dev_err(dd->dev,
				"Flush data(%08x %08x %08x %08x %08x %08x)\n",
				err->flush[0], err->flush[1], err->flush[2],
				err->flush[3], err->flush[4], err->flush[5]);
		dd->cur_msg->status = -EIO;
		complete(&dd->transfer_complete);
	}
}

/**
 * spi_dmov_rx_complete_func - DataMover rx completion callback
 *
 * Executed in IRQ context (Data Mover's IRQ)
 * DataMover's spinlock @msm_dmov_lock held.
 */
static void spi_dmov_rx_complete_func(struct msm_dmov_cmd *cmd,
				      unsigned int result,
				      struct msm_dmov_errdata *err)
{
	struct msm_spi *dd;

	if (!(result & DMOV_RSLT_VALID)) {
		pr_err("Invalid DMOV result(rc = 0x%08x, cmd = %p)",
		       result, cmd);
		return;
	}
	/* restore original context */
	dd = container_of(cmd, struct msm_spi, rx_hdr);
	if (result & DMOV_RSLT_DONE) {
		dd->stat_dmov_rx++;
		if (atomic_inc_return(&dd->rx_irq_called) == 1)
			return;
		complete(&dd->transfer_complete);
	} else {
		/** Error or flush  */
		if (result & DMOV_RSLT_ERROR) {
			dev_err(dd->dev, "DMA error(0x%08x)\n", result);
			dd->stat_dmov_rx_err++;
		}
		if (result & DMOV_RSLT_FLUSH) {
			dev_info(dd->dev,
				"DMA channel flushed(0x%08x)\n", result);
		}
		if (err)
			dev_err(dd->dev,
				"Flush data(%08x %08x %08x %08x %08x %08x)\n",
				err->flush[0], err->flush[1], err->flush[2],
				err->flush[3], err->flush[4], err->flush[5]);
		dd->cur_msg->status = -EIO;
		complete(&dd->transfer_complete);
	}
}

static inline u32 get_chunk_size(struct msm_spi *dd, int input_burst_size,
			int output_burst_size)
{
	u32 cache_line = dma_get_cache_alignment();
	int burst_size = (input_burst_size > output_burst_size) ?
		input_burst_size : output_burst_size;

	return (roundup(sizeof(struct spi_dmov_cmd), DM_BYTE_ALIGN) +
			  roundup(burst_size, cache_line))*2;
}

static void msm_spi_dmov_teardown(struct msm_spi *dd)
{
	int limit = 0;

	if (!dd->use_dma)
		return;

	while (dd->mode == SPI_DMOV_MODE && limit++ < 50) {
		msm_dmov_flush(dd->tx_dma_chan, 1);
		msm_dmov_flush(dd->rx_dma_chan, 1);
		msleep(10);
	}

	dma_free_coherent(NULL,
		get_chunk_size(dd, dd->input_burst_size, dd->output_burst_size),
		dd->tx_dmov_cmd,
		dd->tx_dmov_cmd_dma);
	dd->tx_dmov_cmd = dd->rx_dmov_cmd = NULL;
	dd->tx_padding = dd->rx_padding = NULL;
}

static void msm_spi_bam_pipe_teardown(struct msm_spi *dd,
					enum msm_spi_pipe_direction pipe_dir)
{
	struct msm_spi_bam_pipe *pipe = (pipe_dir == SPI_BAM_CONSUMER_PIPE) ?
					(&dd->bam.prod) : (&dd->bam.cons);
	if (!pipe->teardown_required)
		return;

	msm_spi_bam_pipe_disconnect(dd, pipe);
	dma_free_coherent(dd->dev, pipe->config.desc.size,
		pipe->config.desc.base, pipe->config.desc.phys_base);
	sps_free_endpoint(pipe->handle);
	pipe->handle = 0;
	pipe->teardown_required = false;
}

static int msm_spi_bam_pipe_init(struct msm_spi *dd,
					enum msm_spi_pipe_direction pipe_dir)
{
	int rc = 0;
	struct sps_pipe *pipe_handle;
	struct msm_spi_bam_pipe *pipe = (pipe_dir == SPI_BAM_CONSUMER_PIPE) ?
					(&dd->bam.prod) : (&dd->bam.cons);
	struct sps_connect *pipe_conf = &pipe->config;

	pipe->name   = (pipe_dir == SPI_BAM_CONSUMER_PIPE) ? "cons" : "prod";
	pipe->handle = 0;
	pipe_handle  = sps_alloc_endpoint();
	if (!pipe_handle) {
		dev_err(dd->dev, "%s: Failed to allocate BAM endpoint\n"
								, __func__);
		return -ENOMEM;
	}

	memset(pipe_conf, 0, sizeof(*pipe_conf));
	rc = sps_get_config(pipe_handle, pipe_conf);
	if (rc) {
		dev_err(dd->dev, "%s: Failed to get BAM pipe config\n"
			, __func__);
		goto config_err;
	}

	if (pipe_dir == SPI_BAM_CONSUMER_PIPE) {
		pipe_conf->source          = dd->bam.handle;
		pipe_conf->destination     = SPS_DEV_HANDLE_MEM;
		pipe_conf->mode            = SPS_MODE_SRC;
		pipe_conf->src_pipe_index  =
					dd->pdata->bam_producer_pipe_index;
		pipe_conf->dest_pipe_index = 0;
	} else {
		pipe_conf->source          = SPS_DEV_HANDLE_MEM;
		pipe_conf->destination     = dd->bam.handle;
		pipe_conf->mode            = SPS_MODE_DEST;
		pipe_conf->src_pipe_index  = 0;
		pipe_conf->dest_pipe_index =
					dd->pdata->bam_consumer_pipe_index;
	}
	pipe_conf->options = SPS_O_EOT | SPS_O_AUTO_ENABLE;
	pipe_conf->desc.size = SPI_BAM_MAX_DESC_NUM * sizeof(struct sps_iovec);
	pipe_conf->desc.base = dma_alloc_coherent(dd->dev,
				pipe_conf->desc.size,
				&pipe_conf->desc.phys_base,
				GFP_KERNEL);
	if (!pipe_conf->desc.base) {
		dev_err(dd->dev, "%s: Failed allocate BAM pipe memory"
			, __func__);
		rc = -ENOMEM;
		goto config_err;
	}
	/* zero descriptor FIFO for convenient debugging of first descs */
	memset(pipe_conf->desc.base, 0x00, pipe_conf->desc.size);

	pipe->handle = pipe_handle;
	rc = msm_spi_bam_pipe_connect(dd, pipe, pipe_conf);
	if (rc)
		goto connect_err;

	return 0;

connect_err:
	dma_free_coherent(dd->dev, pipe_conf->desc.size,
		pipe_conf->desc.base, pipe_conf->desc.phys_base);
config_err:
	sps_free_endpoint(pipe_handle);

	return rc;
}

static void msm_spi_bam_teardown(struct msm_spi *dd)
{
	msm_spi_bam_pipe_teardown(dd, SPI_BAM_PRODUCER_PIPE);
	msm_spi_bam_pipe_teardown(dd, SPI_BAM_CONSUMER_PIPE);

	if (dd->bam.deregister_required) {
		sps_deregister_bam_device(dd->bam.handle);
		dd->bam.deregister_required = false;
	}
}

static int msm_spi_bam_init(struct msm_spi *dd)
{
	struct sps_bam_props bam_props = {0};
	u32 bam_handle;
	int rc = 0;

	rc = sps_phy2h(dd->bam.phys_addr, &bam_handle);
	if (rc || !bam_handle) {
		bam_props.phys_addr = dd->bam.phys_addr;
		bam_props.virt_addr = dd->bam.base;
		bam_props.irq       = dd->bam.irq;
		bam_props.manage    = SPS_BAM_MGR_DEVICE_REMOTE;
		bam_props.summing_threshold = 0x10;

		rc = sps_register_bam_device(&bam_props, &bam_handle);
		if (rc) {
			dev_err(dd->dev,
				"%s: Failed to register BAM device",
				__func__);
			return rc;
		}
		dd->bam.deregister_required = true;
	}

	dd->bam.handle = bam_handle;

	rc = msm_spi_bam_pipe_init(dd, SPI_BAM_PRODUCER_PIPE);
	if (rc) {
		dev_err(dd->dev,
			"%s: Failed to init producer BAM-pipe",
			__func__);
		goto bam_init_error;
	}

	rc = msm_spi_bam_pipe_init(dd, SPI_BAM_CONSUMER_PIPE);
	if (rc) {
		dev_err(dd->dev,
			"%s: Failed to init consumer BAM-pipe",
			__func__);
		goto bam_init_error;
	}

	return 0;

bam_init_error:
	msm_spi_bam_teardown(dd);
	return rc;
}

static __init int msm_spi_dmov_init(struct msm_spi *dd)
{
	dmov_box *box;
	u32 cache_line = dma_get_cache_alignment();

	/* Allocate all as one chunk, since all is smaller than page size */

	/* We send NULL device, since it requires coherent_dma_mask id
	   device definition, we're okay with using system pool */
	dd->tx_dmov_cmd
		= dma_alloc_coherent(NULL,
			get_chunk_size(dd, dd->input_burst_size,
				dd->output_burst_size),
			&dd->tx_dmov_cmd_dma, GFP_KERNEL);
	if (dd->tx_dmov_cmd == NULL)
		return -ENOMEM;

	/* DMA addresses should be 64 bit aligned aligned */
	dd->rx_dmov_cmd = (struct spi_dmov_cmd *)
			  ALIGN((size_t)&dd->tx_dmov_cmd[1], DM_BYTE_ALIGN);
	dd->rx_dmov_cmd_dma = ALIGN(dd->tx_dmov_cmd_dma +
			      sizeof(struct spi_dmov_cmd), DM_BYTE_ALIGN);

	/* Buffers should be aligned to cache line */
	dd->tx_padding = (u8 *)ALIGN((size_t)&dd->rx_dmov_cmd[1], cache_line);
	dd->tx_padding_dma = ALIGN(dd->rx_dmov_cmd_dma +
			      sizeof(struct spi_dmov_cmd), cache_line);
	dd->rx_padding = (u8 *)ALIGN((size_t)(dd->tx_padding +
		dd->output_burst_size), cache_line);
	dd->rx_padding_dma = ALIGN(dd->tx_padding_dma + dd->output_burst_size,
				      cache_line);

	/* Setup DM commands */
	box = &(dd->rx_dmov_cmd->box);
	box->cmd = CMD_MODE_BOX | CMD_SRC_CRCI(dd->rx_dma_crci);
	box->src_row_addr = (uint32_t)dd->mem_phys_addr + SPI_INPUT_FIFO;
	dd->rx_hdr.cmdptr = DMOV_CMD_PTR_LIST |
				   DMOV_CMD_ADDR(dd->rx_dmov_cmd_dma +
				   offsetof(struct spi_dmov_cmd, cmd_ptr));
	dd->rx_hdr.complete_func = spi_dmov_rx_complete_func;

	box = &(dd->tx_dmov_cmd->box);
	box->cmd = CMD_MODE_BOX | CMD_DST_CRCI(dd->tx_dma_crci);
	box->dst_row_addr = (uint32_t)dd->mem_phys_addr + SPI_OUTPUT_FIFO;
	dd->tx_hdr.cmdptr = DMOV_CMD_PTR_LIST |
			    DMOV_CMD_ADDR(dd->tx_dmov_cmd_dma +
			    offsetof(struct spi_dmov_cmd, cmd_ptr));
	dd->tx_hdr.complete_func = spi_dmov_tx_complete_func;

	dd->tx_dmov_cmd->single_pad.cmd = CMD_MODE_SINGLE | CMD_LC |
					  CMD_DST_CRCI(dd->tx_dma_crci);
	dd->tx_dmov_cmd->single_pad.dst = (uint32_t)dd->mem_phys_addr +
					   SPI_OUTPUT_FIFO;
	dd->rx_dmov_cmd->single_pad.cmd = CMD_MODE_SINGLE | CMD_LC |
					  CMD_SRC_CRCI(dd->rx_dma_crci);
	dd->rx_dmov_cmd->single_pad.src = (uint32_t)dd->mem_phys_addr +
					  SPI_INPUT_FIFO;

	/* Clear remaining activities on channel */
	msm_dmov_flush(dd->tx_dma_chan, 1);
	msm_dmov_flush(dd->rx_dma_chan, 1);

	return 0;
}

enum msm_spi_dt_entry_status {
	DT_REQ,  /* Required:  fail if missing */
	DT_SGST, /* Suggested: warn if missing */
	DT_OPT,  /* Optional:  don't warn if missing */
};

enum msm_spi_dt_entry_type {
	DT_U32,
	DT_GPIO,
	DT_BOOL,
};

struct msm_spi_dt_to_pdata_map {
	const char                  *dt_name;
	void                        *ptr_data;
	enum msm_spi_dt_entry_status status;
	enum msm_spi_dt_entry_type   type;
	int                          default_val;
};

static int __init msm_spi_dt_to_pdata_populate(struct platform_device *pdev,
					struct msm_spi_platform_data *pdata,
					struct msm_spi_dt_to_pdata_map  *itr)
{
	int  ret, err = 0;
	struct device_node *node = pdev->dev.of_node;

	for (; itr->dt_name ; ++itr) {
		switch (itr->type) {
		case DT_GPIO:
			ret = of_get_named_gpio(node, itr->dt_name, 0);
			if (ret >= 0) {
				*((int *) itr->ptr_data) = ret;
				ret = 0;
			}
			break;
		case DT_U32:
			ret = of_property_read_u32(node, itr->dt_name,
							 (u32 *) itr->ptr_data);
			break;
		case DT_BOOL:
			*((bool *) itr->ptr_data) =
				of_property_read_bool(node, itr->dt_name);
			ret = 0;
			break;
		default:
			dev_err(&pdev->dev, "%d is an unknown DT entry type\n",
								itr->type);
			ret = -EBADE;
		}

		dev_dbg(&pdev->dev, "DT entry ret:%d name:%s val:%d\n",
				ret, itr->dt_name, *((int *)itr->ptr_data));

		if (ret) {
			*((int *)itr->ptr_data) = itr->default_val;

			if (itr->status < DT_OPT) {
				dev_err(&pdev->dev, "Missing '%s' DT entry\n",
								itr->dt_name);

				/* cont on err to dump all missing entries */
				if (itr->status == DT_REQ && !err)
					err = ret;
			}
		}
	}

	return err;
}

/**
 * msm_spi_dt_to_pdata: create pdata and read gpio config from device tree
 */
struct msm_spi_platform_data * __init msm_spi_dt_to_pdata(
			struct platform_device *pdev, struct msm_spi *dd)
{
	struct msm_spi_platform_data *pdata;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("Unable to allocate platform data\n");
		return NULL;
	} else {
		struct msm_spi_dt_to_pdata_map map[] = {
		{"spi-max-frequency",
			&pdata->max_clock_speed,         DT_SGST, DT_U32,   0},
		{"qcom,infinite-mode",
			&pdata->infinite_mode,           DT_OPT,  DT_U32,   0},
		{"qcom,active-only",
			&pdata->active_only,             DT_OPT,  DT_BOOL,  0},
		{"qcom,master-id",
			&pdata->master_id,               DT_SGST, DT_U32,   0},
		{"qcom,ver-reg-exists",
			&pdata->ver_reg_exists,          DT_OPT,  DT_BOOL,  0},
		{"qcom,use-bam",
			&pdata->use_bam,                 DT_OPT,  DT_BOOL,  0},
		{"qcom,bam-consumer-pipe-index",
			&pdata->bam_consumer_pipe_index, DT_OPT,  DT_U32,   0},
		{"qcom,bam-producer-pipe-index",
			&pdata->bam_producer_pipe_index, DT_OPT,  DT_U32,   0},
		{"qcom,gpio-clk",
			&dd->spi_gpios[0],               DT_OPT,  DT_GPIO, -1},
		{"qcom,gpio-miso",
			&dd->spi_gpios[1],               DT_OPT,  DT_GPIO, -1},
		{"qcom,gpio-mosi",
			&dd->spi_gpios[2],               DT_OPT,  DT_GPIO, -1},
		{"qcom,gpio-cs0",
			&dd->cs_gpios[0].gpio_num,       DT_OPT,  DT_GPIO, -1},
		{"qcom,gpio-cs1",
			&dd->cs_gpios[1].gpio_num,       DT_OPT,  DT_GPIO, -1},
		{"qcom,gpio-cs2",
			&dd->cs_gpios[2].gpio_num,       DT_OPT,  DT_GPIO, -1},
		{"qcom,gpio-cs3",
			&dd->cs_gpios[3].gpio_num,       DT_OPT,  DT_GPIO, -1},
		{"qcom,rt-priority",
			&pdata->rt_priority,		 DT_OPT,  DT_BOOL,  0},
		{NULL,  NULL,                            0,       0,        0},
		};

		if (msm_spi_dt_to_pdata_populate(pdev, pdata, map)) {
			devm_kfree(&pdev->dev, pdata);
			return NULL;
		}
	}

	if (pdata->use_bam) {
		if (!pdata->bam_consumer_pipe_index) {
			dev_warn(&pdev->dev,
			"missing qcom,bam-consumer-pipe-index entry in device-tree\n");
			pdata->use_bam = false;
		}

		if (!pdata->bam_producer_pipe_index) {
			dev_warn(&pdev->dev,
			"missing qcom,bam-producer-pipe-index entry in device-tree\n");
			pdata->use_bam = false;
		}
	}
	return pdata;
}

static int __init msm_spi_get_qup_hw_ver(struct device *dev, struct msm_spi *dd)
{
	u32 data = readl_relaxed(dd->base + QUP_HARDWARE_VER);
	return (data >= QUP_HARDWARE_VER_2_1_1) ? SPI_QUP_VERSION_BFAM
						: SPI_QUP_VERSION_NONE;
}

static int __init msm_spi_bam_get_resources(struct msm_spi *dd,
	struct platform_device *pdev, struct spi_master *master)
{
	struct resource *resource;
	size_t bam_mem_size;

	resource = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"spi_bam_physical");
	if (!resource) {
		dev_warn(&pdev->dev,
			"%s: Missing spi_bam_physical entry in DT",
			__func__);
		return -ENXIO;
	}

	dd->bam.phys_addr = resource->start;
	bam_mem_size = resource_size(resource);
	dd->bam.base = devm_ioremap(&pdev->dev, dd->bam.phys_addr,
					bam_mem_size);
	if (!dd->bam.base) {
		dev_warn(&pdev->dev,
			"%s: Failed to ioremap(spi_bam_physical)",
			__func__);
		return -ENXIO;
	}

	dd->bam.irq = platform_get_irq_byname(pdev, "spi_bam_irq");
	if (dd->bam.irq < 0) {
		dev_warn(&pdev->dev, "%s: Missing spi_bam_irq entry in DT",
			__func__);
		return -EINVAL;
	}

	dd->dma_init = msm_spi_bam_init;
	dd->dma_teardown = msm_spi_bam_teardown;
	return 0;
}

static int __init msm_spi_probe(struct platform_device *pdev)
{
	struct spi_master      *master;
	struct msm_spi	       *dd;
	struct resource	       *resource;
	int                     rc = -ENXIO;
	int                     locked = 0;
	int                     i = 0;
	int                     clk_enabled = 0;
	int                     pclk_enabled = 0;
	struct msm_spi_platform_data *pdata;

	master = spi_alloc_master(&pdev->dev, sizeof(struct msm_spi));
	if (!master) {
		rc = -ENOMEM;
		dev_err(&pdev->dev, "master allocation failed\n");
		goto err_probe_exit;
	}

	master->bus_num        = pdev->id;
	master->mode_bits      = SPI_SUPPORTED_MODES;
	master->num_chipselect = SPI_NUM_CHIPSELECTS;
	master->setup          = msm_spi_setup;
	master->prepare_transfer_hardware = msm_spi_prepare_transfer_hardware;
	master->transfer_one_message = msm_spi_transfer_one_message;
	master->unprepare_transfer_hardware
			= msm_spi_unprepare_transfer_hardware;

	platform_set_drvdata(pdev, master);
	dd = spi_master_get_devdata(master);

	if (pdev->dev.of_node) {
		dd->qup_ver = SPI_QUP_VERSION_BFAM;
		master->dev.of_node = pdev->dev.of_node;
		pdata = msm_spi_dt_to_pdata(pdev, dd);
		if (!pdata) {
			rc = -ENOMEM;
			goto err_probe_exit;
		}

		rc = of_alias_get_id(pdev->dev.of_node, "spi");
		if (rc < 0)
			dev_warn(&pdev->dev,
				"using default bus_num %d\n", pdev->id);
		else
			master->bus_num = pdev->id = rc;
	} else {
		pdata = pdev->dev.platform_data;
		dd->qup_ver = SPI_QUP_VERSION_NONE;

		for (i = 0; i < ARRAY_SIZE(spi_rsrcs); ++i) {
			resource = platform_get_resource(pdev, IORESOURCE_IO,
							i);
			dd->spi_gpios[i] = resource ? resource->start : -1;
		}

		for (i = 0; i < ARRAY_SIZE(spi_cs_rsrcs); ++i) {
			resource = platform_get_resource(pdev, IORESOURCE_IO,
						i + ARRAY_SIZE(spi_rsrcs));
			dd->cs_gpios[i].gpio_num = resource ?
							resource->start : -1;
		}
	}

	for (i = 0; i < ARRAY_SIZE(spi_cs_rsrcs); ++i)
		dd->cs_gpios[i].valid = 0;

	master->rt = pdata->rt_priority;
	dd->pdata = pdata;
	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resource) {
		rc = -ENXIO;
		goto err_probe_res;
	}

	dd->mem_phys_addr = resource->start;
	dd->mem_size = resource_size(resource);

	if (pdata) {
		if (pdata->dma_config) {
			rc = pdata->dma_config();
			if (rc) {
				dev_warn(&pdev->dev,
					"%s: DM mode not supported\n",
					__func__);
				dd->use_dma = 0;
				goto skip_dma_resources;
			}
		}
		if (dd->qup_ver == SPI_QUP_VERSION_NONE) {
			resource = platform_get_resource(pdev,
							IORESOURCE_DMA, 0);
			if (resource) {
				dd->rx_dma_chan = resource->start;
				dd->tx_dma_chan = resource->end;
				resource = platform_get_resource(pdev,
							IORESOURCE_DMA, 1);
				if (!resource) {
					rc = -ENXIO;
					goto err_probe_res;
				}

				dd->rx_dma_crci = resource->start;
				dd->tx_dma_crci = resource->end;
				dd->use_dma = 1;
				master->dma_alignment =
						dma_get_cache_alignment();
				dd->dma_init = msm_spi_dmov_init ;
				dd->dma_teardown = msm_spi_dmov_teardown;
			}
		} else {
			if (!dd->pdata->use_bam)
				goto skip_dma_resources;

			rc = msm_spi_bam_get_resources(dd, pdev, master);
			if (rc) {
				dev_warn(dd->dev,
					"%s: Faild to get BAM resources",
					__func__);
				goto skip_dma_resources;
			}
			dd->use_dma = 1;
		}
	}

skip_dma_resources:

	spin_lock_init(&dd->queue_lock);
	mutex_init(&dd->core_lock);
	init_waitqueue_head(&dd->continue_suspend);

	if (!devm_request_mem_region(&pdev->dev, dd->mem_phys_addr,
					dd->mem_size, SPI_DRV_NAME)) {
		rc = -ENXIO;
		goto err_probe_reqmem;
	}

	dd->base = devm_ioremap(&pdev->dev, dd->mem_phys_addr, dd->mem_size);
	if (!dd->base) {
		rc = -ENOMEM;
		goto err_probe_reqmem;
	}

	if (pdata && pdata->rsl_id) {
		struct remote_mutex_id rmid;
		rmid.r_spinlock_id = pdata->rsl_id;
		rmid.delay_us = SPI_TRYLOCK_DELAY;

		rc = remote_mutex_init(&dd->r_lock, &rmid);
		if (rc) {
			dev_err(&pdev->dev, "%s: unable to init remote_mutex "
				"(%s), (rc=%d)\n", rmid.r_spinlock_id,
				__func__, rc);
			goto err_probe_rlock_init;
		}

		dd->use_rlock = 1;
		dd->pm_lat = pdata->pm_lat;
		pm_qos_add_request(&qos_req_list, PM_QOS_CPU_DMA_LATENCY,
						PM_QOS_DEFAULT_VALUE);
	}

	mutex_lock(&dd->core_lock);
	if (dd->use_rlock)
		remote_mutex_lock(&dd->r_lock);

	locked = 1;
	dd->dev = &pdev->dev;
	dd->clk = clk_get(&pdev->dev, "core_clk");
	if (IS_ERR(dd->clk)) {
		dev_err(&pdev->dev, "%s: unable to get core_clk\n", __func__);
		rc = PTR_ERR(dd->clk);
		goto err_probe_clk_get;
	}

	dd->pclk = clk_get(&pdev->dev, "iface_clk");
	if (IS_ERR(dd->pclk)) {
		dev_err(&pdev->dev, "%s: unable to get iface_clk\n", __func__);
		rc = PTR_ERR(dd->pclk);
		goto err_probe_pclk_get;
	}

	if (pdata && pdata->max_clock_speed)
		msm_spi_clock_set(dd, dd->pdata->max_clock_speed);

	rc = clk_prepare_enable(dd->clk);
	if (rc) {
		dev_err(&pdev->dev, "%s: unable to enable core_clk\n",
			__func__);
		goto err_probe_clk_enable;
	}

	clk_enabled = 1;
	rc = clk_prepare_enable(dd->pclk);
	if (rc) {
		dev_err(&pdev->dev, "%s: unable to enable iface_clk\n",
		__func__);
		goto err_probe_pclk_enable;
	}

	pclk_enabled = 1;

	if (pdata && pdata->ver_reg_exists) {
		enum msm_spi_qup_version ver =
					msm_spi_get_qup_hw_ver(&pdev->dev, dd);
		if (dd->qup_ver != ver)
			dev_warn(&pdev->dev,
			"%s: HW version different then initially assumed by probe",
			__func__);
	}

	/* GSBI dose not exists on B-family MSM-chips */
	if (dd->qup_ver != SPI_QUP_VERSION_BFAM) {
		rc = msm_spi_configure_gsbi(dd, pdev);
		if (rc)
			goto err_probe_gsbi;
	}

	msm_spi_calculate_fifo_size(dd);
	if (dd->use_dma) {
		rc = dd->dma_init(dd);
		if (rc)
			goto err_probe_dma;
	}

	msm_spi_register_init(dd);
	/*
	 * The SPI core generates a bogus input overrun error on some targets,
	 * when a transition from run to reset state occurs and if the FIFO has
	 * an odd number of entries. Hence we disable the INPUT_OVER_RUN_ERR_EN
	 * bit.
	 */
	msm_spi_enable_error_flags(dd);

	writel_relaxed(SPI_IO_C_NO_TRI_STATE, dd->base + SPI_IO_CONTROL);
	rc = msm_spi_set_state(dd, SPI_OP_STATE_RESET);
	if (rc)
		goto err_probe_state;

	clk_disable_unprepare(dd->clk);
	clk_disable_unprepare(dd->pclk);
	clk_enabled = 0;
	pclk_enabled = 0;

	dd->suspended = 1;
	dd->transfer_pending = 0;
	dd->multi_xfr = 0;
	dd->mode = SPI_MODE_NONE;

	rc = msm_spi_request_irq(dd, pdev, master);
	if (rc)
		goto err_probe_irq;

	msm_spi_disable_irqs(dd);
	if (dd->use_rlock)
		remote_mutex_unlock(&dd->r_lock);

	mutex_unlock(&dd->core_lock);
	locked = 0;

	pm_runtime_set_autosuspend_delay(&pdev->dev, MSEC_PER_SEC);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	rc = spi_register_master(master);
	if (rc)
		goto err_probe_reg_master;

	rc = sysfs_create_group(&(dd->dev->kobj), &dev_attr_grp);
	if (rc) {
		dev_err(&pdev->dev, "failed to create dev. attrs : %d\n", rc);
		goto err_attrs;
	}

	spi_debugfs_init(dd);

	return 0;

err_attrs:
	spi_unregister_master(master);
err_probe_reg_master:
	pm_runtime_disable(&pdev->dev);
err_probe_irq:
err_probe_state:
	if (dd->dma_teardown)
		dd->dma_teardown(dd);
err_probe_dma:
err_probe_gsbi:
	if (pclk_enabled)
		clk_disable_unprepare(dd->pclk);
err_probe_pclk_enable:
	if (clk_enabled)
		clk_disable_unprepare(dd->clk);
err_probe_clk_enable:
	clk_put(dd->pclk);
err_probe_pclk_get:
	clk_put(dd->clk);
err_probe_clk_get:
	if (locked) {
		if (dd->use_rlock)
			remote_mutex_unlock(&dd->r_lock);

		mutex_unlock(&dd->core_lock);
	}
err_probe_rlock_init:
err_probe_reqmem:
err_probe_res:
	spi_master_put(master);
err_probe_exit:
	return rc;
}

#ifdef CONFIG_PM
static int msm_spi_pm_suspend_runtime(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct spi_master *master = platform_get_drvdata(pdev);
	struct msm_spi	  *dd;
	unsigned long	   flags;

	dev_dbg(device, "pm_runtime: suspending...\n");
	if (!master)
		goto suspend_exit;
	dd = spi_master_get_devdata(master);
	if (!dd)
		goto suspend_exit;

	if (dd->suspended)
		return 0;

	/*
	 * Make sure nothing is added to the queue while we're
	 * suspending
	 */
	spin_lock_irqsave(&dd->queue_lock, flags);
	dd->suspended = 1;
	spin_unlock_irqrestore(&dd->queue_lock, flags);

	/* Wait for transactions to end, or time out */
	wait_event_interruptible(dd->continue_suspend,
		!dd->transfer_pending);

	msm_spi_disable_irqs(dd);
	clk_disable_unprepare(dd->clk);
	clk_disable_unprepare(dd->pclk);
	if (dd->pdata && !dd->pdata->active_only)
		msm_spi_clk_path_unvote(dd);

	/* Free  the spi clk, miso, mosi, cs gpio */
	if (dd->pdata && dd->pdata->gpio_release)
		dd->pdata->gpio_release();

	msm_spi_free_gpios(dd);

	if (pm_qos_request_active(&qos_req_list))
		pm_qos_update_request(&qos_req_list,
				PM_QOS_DEFAULT_VALUE);
suspend_exit:
	return 0;
}

static int msm_spi_pm_resume_runtime(struct device *device)
{
	struct platform_device *pdev = to_platform_device(device);
	struct spi_master *master = platform_get_drvdata(pdev);
	struct msm_spi	  *dd;
	int ret = 0;

	dev_dbg(device, "pm_runtime: resuming...\n");
	if (!master)
		goto resume_exit;
	dd = spi_master_get_devdata(master);
	if (!dd)
		goto resume_exit;

	if (!dd->suspended)
		return 0;

	if (pm_qos_request_active(&qos_req_list))
		pm_qos_update_request(&qos_req_list,
				  dd->pm_lat);

	/* Configure the spi clk, miso, mosi and cs gpio */
	if (dd->pdata->gpio_config) {
		ret = dd->pdata->gpio_config();
		if (ret) {
			dev_err(dd->dev,
					"%s: error configuring GPIOs\n",
					__func__);
			return ret;
		}
	}

	ret = msm_spi_request_gpios(dd);
	if (ret)
		return ret;

	msm_spi_clk_path_init(dd);
	if (!dd->pdata->active_only)
		msm_spi_clk_path_vote(dd);
	clk_prepare_enable(dd->clk);
	clk_prepare_enable(dd->pclk);
	msm_spi_enable_irqs(dd);
	dd->suspended = 0;

resume_exit:
	return 0;
}

static int msm_spi_suspend(struct device *device)
{
	if (!pm_runtime_enabled(device) || !pm_runtime_suspended(device)) {
		struct platform_device *pdev = to_platform_device(device);
		struct spi_master *master = platform_get_drvdata(pdev);
		struct msm_spi   *dd;

		dev_dbg(device, "system suspend");
		if (!master)
			goto suspend_exit;
		dd = spi_master_get_devdata(master);
		if (!dd)
			goto suspend_exit;
		msm_spi_pm_suspend_runtime(device);

		/*
		 * set the device's runtime PM status to 'suspended'
		 */
		pm_runtime_disable(device);
		pm_runtime_set_suspended(device);
		pm_runtime_enable(device);
	}
suspend_exit:
	return 0;
}

static int msm_spi_resume(struct device *device)
{
	/*
	 * Rely on runtime-PM to call resume in case it is enabled
	 * Even if it's not enabled, rely on 1st client transaction to do
	 * clock ON and gpio configuration
	 */
	dev_dbg(device, "system resume");
	return 0;
}
#else
#define msm_spi_suspend NULL
#define msm_spi_resume NULL
#define msm_spi_pm_suspend_runtime NULL
#define msm_spi_pm_resume_runtime NULL
#endif /* CONFIG_PM */

static int __devexit msm_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct msm_spi    *dd = spi_master_get_devdata(master);

	pm_qos_remove_request(&qos_req_list);
	spi_debugfs_exit(dd);
	sysfs_remove_group(&pdev->dev.kobj, &dev_attr_grp);

	if (dd->dma_teardown)
		dd->dma_teardown(dd);
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	clk_put(dd->clk);
	clk_put(dd->pclk);
	msm_spi_clk_path_teardown(dd);
	platform_set_drvdata(pdev, 0);
	spi_unregister_master(master);
	spi_master_put(master);

	return 0;
}

static struct of_device_id msm_spi_dt_match[] = {
	{
		.compatible = "qcom,spi-qup-v2",
	},
	{}
};

static const struct dev_pm_ops msm_spi_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(msm_spi_suspend, msm_spi_resume)
	SET_RUNTIME_PM_OPS(msm_spi_pm_suspend_runtime,
			msm_spi_pm_resume_runtime, NULL)
};

static struct platform_driver msm_spi_driver = {
	.driver		= {
		.name	= SPI_DRV_NAME,
		.owner	= THIS_MODULE,
		.pm		= &msm_spi_dev_pm_ops,
		.of_match_table = msm_spi_dt_match,
	},
	.remove		= __exit_p(msm_spi_remove),
};

static int __init msm_spi_init(void)
{
	return platform_driver_probe(&msm_spi_driver, msm_spi_probe);
}
module_init(msm_spi_init);

static void __exit msm_spi_exit(void)
{
	platform_driver_unregister(&msm_spi_driver);
}
module_exit(msm_spi_exit);

MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.4");
MODULE_ALIAS("platform:"SPI_DRV_NAME);
