/*
 * drivers/serial/msm_serial.c - driver for msm7k serial device and console
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* Acknowledgements:
 * This file is based on msm_serial.c, originally
 * Written by Robert Love <rlove@google.com>  */

#if defined(CONFIG_SERIAL_MSM_HSL_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/atomic.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/nmi.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <mach/board.h>
#include <mach/msm_serial_hs_lite.h>
#include <mach/msm_bus.h>
#include <asm/mach-types.h>
#include "msm_serial_hs_hwreg.h"
#include <mach/board_htc.h>
#include <linux/htc_cir.h>
#include <mach/devices_cmdline.h>
#include <mach/devices_dtb.h>

#define D(x...) pr_info("[CIR] " x)
#define E(x...) pr_err("[CIR][err] " x)

#define HTC_IRDA_FUNC 1
enum uart_core_type {
	LEGACY_HSUART,
	GSBI_HSUART,
	BLSP_HSUART,
};

enum uart_func_mode {
	UART_TWO_WIRE, 
	UART_FOUR_WIRE,
};

struct msm_hsl_port {
	struct uart_port	uart;
	char			name[16];
	struct clk		*clk;
	struct clk		*pclk;
	struct dentry		*loopback_dir;
#ifdef HTC_IRDA_FUNC
	struct dentry		*irda_config; 
#endif
	unsigned int		imr;
	unsigned int		*uart_csr_code;
	unsigned int            *gsbi_mapbase;
	unsigned int            *mapped_gsbi;
	int			is_uartdm;
	unsigned int            old_snap_state;
	unsigned int		ver_id;
	int			tx_timeout;
	struct mutex		clk_mutex;
	enum uart_core_type	uart_type;
	enum uart_func_mode	func_mode;
	struct wake_lock	port_open_wake_lock;
	int			clk_enable_count;
	u32			bus_perf_client;
	
	struct msm_bus_scale_pdata *bus_scale_table;
	int (*cir_set_path)(int);
	int (*cir_reset)(void);
	int (*cir_power)(int); 
	uint32_t rst_pin;
	uint32_t cir_sir_switch;
	uint32_t cir_learn_en;
	struct class *irda_class;
	struct device *irda_dev;
	struct class *cir_class;
	struct device *cir_dev;
};
static struct msm_hsl_port *htc_cir_port;
static int msm_serial_hsl_enable;
static int cir_enable_flg;
#define UARTDM_VERSION_11_13	0
#define UARTDM_VERSION_14	1

#define UART_TO_MSM(uart_port)	((struct msm_hsl_port *) uart_port)
#define is_console(port)	((port)->cons && \
				(port)->cons->index == (port)->line)

static const unsigned int regmap[][UARTDM_LAST] = {
	[UARTDM_VERSION_11_13] = {
		[UARTDM_MR1] = UARTDM_MR1_ADDR,
		[UARTDM_MR2] = UARTDM_MR2_ADDR,
		[UARTDM_IMR] = UARTDM_IMR_ADDR,
		[UARTDM_SR] = UARTDM_SR_ADDR,
		[UARTDM_CR] = UARTDM_CR_ADDR,
		[UARTDM_CSR] = UARTDM_CSR_ADDR,
		[UARTDM_IPR] = UARTDM_IPR_ADDR,
		[UARTDM_ISR] = UARTDM_ISR_ADDR,
		[UARTDM_RX_TOTAL_SNAP] = UARTDM_RX_TOTAL_SNAP_ADDR,
		[UARTDM_TFWR] = UARTDM_TFWR_ADDR,
		[UARTDM_RFWR] = UARTDM_RFWR_ADDR,
		[UARTDM_RF] = UARTDM_RF_ADDR,
		[UARTDM_TF] = UARTDM_TF_ADDR,
		[UARTDM_MISR] = UARTDM_MISR_ADDR,
		[UARTDM_DMRX] = UARTDM_DMRX_ADDR,
		[UARTDM_NCF_TX] = UARTDM_NCF_TX_ADDR,
		[UARTDM_DMEN] = UARTDM_DMEN_ADDR,
	},
	[UARTDM_VERSION_14] = {
		[UARTDM_MR1] = 0x0,
		[UARTDM_MR2] = 0x4,
		[UARTDM_IMR] = 0xb0,
		[UARTDM_SR] = 0xa4,
		[UARTDM_CR] = 0xa8,
		[UARTDM_CSR] = 0xa0,
		[UARTDM_IPR] = 0x18,
		[UARTDM_ISR] = 0xb4,
		[UARTDM_RX_TOTAL_SNAP] = 0xbc,
		[UARTDM_TFWR] = 0x1c,
		[UARTDM_RFWR] = 0x20,
		[UARTDM_RF] = 0x140,
		[UARTDM_TF] = 0x100,
		[UARTDM_MISR] = 0xac,
		[UARTDM_DMRX] = 0x34,
		[UARTDM_NCF_TX] = 0x40,
		[UARTDM_DMEN] = 0x3c,
	},
};

static struct of_device_id msm_hsl_match_table[] = {
	{	
		.compatible = "CIR"
		
	},
	{}
};
static struct dentry *debug_base;
static inline void wait_for_xmitr(struct uart_port *port, int bits);
static inline void msm_hsl_write(struct uart_port *port,
				 unsigned int val, unsigned int off)
{
	iowrite32(val, port->membase + off);
}
static inline unsigned int msm_hsl_read(struct uart_port *port,
		     unsigned int off)
{
	return ioread32(port->membase + off);
}

static unsigned int msm_serial_hsl_has_gsbi(struct uart_port *port)
{
	D("%s: cir uart port[%d] is_uartdm=%d\n", __func__, (port)->line, UART_TO_MSM(port)->is_uartdm);
	return UART_TO_MSM(port)->is_uartdm;
}

static int clk_en(struct uart_port *port, int enable)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	int ret = 0;

	if (enable) {

		ret = clk_prepare_enable(msm_hsl_port->clk);
		if (ret)
			goto err;
		if (msm_hsl_port->pclk) {
			ret = clk_prepare_enable(msm_hsl_port->pclk);
			if (ret) {
				clk_disable_unprepare(msm_hsl_port->clk);
				goto err;
			}
		}
	} else {

		clk_disable_unprepare(msm_hsl_port->clk);
		if (msm_hsl_port->pclk)
			clk_disable_unprepare(msm_hsl_port->pclk);
	}
err:
	return ret;
}
static int msm_hsl_loopback_enable_set(void *data, u64 val)
{
	struct msm_hsl_port *msm_hsl_port = data;
	struct uart_port *port = &(msm_hsl_port->uart);
	unsigned int vid;
	unsigned long flags;
	int ret = 0;

	ret = clk_set_rate(msm_hsl_port->clk, 7372800);
	if (!ret)
		clk_en(port, 1);
	else {
		E("%s(): Error: Setting the clock rate\n", __func__);
		return -EINVAL;
	}

	vid = msm_hsl_port->ver_id;
	if (val) {
		spin_lock_irqsave(&port->lock, flags);
		ret = msm_hsl_read(port, regmap[vid][UARTDM_MR2]);
		ret |= UARTDM_MR2_LOOP_MODE_BMSK;
		msm_hsl_write(port, ret, regmap[vid][UARTDM_MR2]);
		spin_unlock_irqrestore(&port->lock, flags);
		E("%s(): irda loopback enabled for line(%d)\n", __func__, port->line);
	} else {
		spin_lock_irqsave(&port->lock, flags);
		ret = msm_hsl_read(port, regmap[vid][UARTDM_MR2]);
		ret &= ~UARTDM_MR2_LOOP_MODE_BMSK;
		msm_hsl_write(port, ret, regmap[vid][UARTDM_MR2]);
		spin_unlock_irqrestore(&port->lock, flags);
	}

	clk_en(port, 0);
	return 0;
}
static int msm_hsl_loopback_enable_get(void *data, u64 *val)
{
	struct msm_hsl_port *msm_hsl_port = data;
	struct uart_port *port = &(msm_hsl_port->uart);
	unsigned long flags;
	int ret = 0;

	ret = clk_set_rate(msm_hsl_port->clk, 7372800);
	if (!ret)
		clk_en(port, 1);
	else {
		E("%s(): Error setting clk rate\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&port->lock, flags);
	ret = msm_hsl_read(port, regmap[msm_hsl_port->ver_id][UARTDM_MR2]);
	spin_unlock_irqrestore(&port->lock, flags);
	clk_en(port, 0);

	*val = (ret & UARTDM_MR2_LOOP_MODE_BMSK) ? 1 : 0;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(loopback_enable_fops, msm_hsl_loopback_enable_get,
			msm_hsl_loopback_enable_set, "%llu\n");
unsigned int force_baud_1;
#ifdef HTC_IRDA_FUNC
static int msm_hsl_irda_enable_set(void *data, u64 val)
{
	struct msm_hsl_port *msm_hsl_port = data;
	struct msm_hsl_port *msm_cir_port = htc_cir_port;
	struct uart_port *port = &(msm_hsl_port->uart);
	unsigned long flags;
	int ret = 0;

	if (val == 96) {
		force_baud_1 = 96;
		E("%s(): change baud = 9600\n", __func__);
		return 0;
	} else if (val == 1152) {
		E("%s(): change baud = 115200\n", __func__);
		force_baud_1 = 1152;
		return 0;
	} else if (val == 1153) {
		E("%s():no  change baud \n", __func__);
		force_baud_1 = 0;
		return 0;
	}

	ret = clk_set_rate(msm_hsl_port->clk, 7372800);
	if (!ret) {
		clk_en(port, 1);
		E("%s(): irda Clock enabled for line(%d)\n", __func__, port->line);
	} else {
		E("%s(): Error: Setting the clock rate\n", __func__);
		return -EINVAL;
	}

	if (val) {
		E("%s(): irda turn on IRDA\n", __func__);
		spin_lock_irqsave(&port->lock, flags);
		if (msm_cir_port->cir_learn_en)
			gpio_direction_output(msm_cir_port->cir_learn_en, 1);
		gpio_direction_output(msm_cir_port->rst_pin, 0);
		msleep(10);

		if (msm_cir_port->cir_sir_switch)
			gpio_direction_output(msm_cir_port->cir_sir_switch, 1);
		ret = 3;
		ret |= (int)val;
		msm_hsl_write(port, ret, UARTDM_IRDA_ADDR);
		spin_unlock_irqrestore(&port->lock, flags);

		if (msm_hsl_port->cir_set_path)
			msm_hsl_port->cir_set_path(PATH_IRDA);
		else
			E("no irda enable callback function");

	} else {
		E("%s(): irda turn off IRDA \n", __func__);
		spin_lock_irqsave(&port->lock, flags);
		if (msm_cir_port->cir_sir_switch)
			gpio_direction_output(msm_cir_port->cir_sir_switch, 0);
		if (msm_cir_port->cir_learn_en)
			gpio_direction_output(msm_cir_port->cir_learn_en, 0);
		gpio_direction_input(msm_cir_port->rst_pin);
		ret = 0;
		msm_hsl_write(port, ret, UARTDM_IRDA_ADDR);
		spin_unlock_irqrestore(&port->lock, flags);

		if (msm_hsl_port->cir_set_path)
			msm_hsl_port->cir_set_path(PATH_NONE);
		else
			E("no irda enable callback function");
	}


	clk_en(port, 0);
	E("%s(): irda Clock enabled for line(%d)\n", __func__, port->line);
	return 0;
}
static int msm_hsl_irda_enable_get(void *data, u64 *val)
{
	E("%s: warning! disallow read register UARTDM_IRDA\n", __func__);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(msm_serial_irda_fops, msm_hsl_irda_enable_get,
			msm_hsl_irda_enable_set, "%llu\n");
#endif

static void msm_hsl_debugfs_init(struct msm_hsl_port *msm_uport,
								int id)
{
	char node_name[15];

	snprintf(node_name, sizeof(node_name), "loopback.%d", id);
	msm_uport->loopback_dir = debugfs_create_file(node_name,
					S_IRUGO | S_IWUSR,
					debug_base,
					msm_uport,
					&loopback_enable_fops);

	if (IS_ERR_OR_NULL(msm_uport->loopback_dir))
		E("%s(): Cannot create loopback.%d debug entry",
							__func__, id);
#ifdef HTC_IRDA_FUNC
	snprintf(node_name, sizeof(node_name), "irda.%d", id);
	msm_uport->irda_config = debugfs_create_file(node_name,
					S_IRUGO | S_IWUSR,
					debug_base,
					msm_uport,
					&msm_serial_irda_fops);

	if (IS_ERR_OR_NULL(msm_uport->irda_config))
		E("%s(): Cannot create irda.%d debug entry",
							__func__, id);
#endif
}
static void msm_hsl_stop_tx_cir(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	struct msm_hsl_port *msm_cir_port = htc_cir_port;
	unsigned int vid = UART_TO_MSM(port)->ver_id;
	int count = 0;

	msm_hsl_port->imr &= ~UARTDM_ISR_TXLEV_BMSK;
	msm_hsl_write(port, msm_hsl_port->imr,
		regmap[msm_hsl_port->ver_id][UARTDM_IMR]);
	if (msm_cir_port->cir_learn_en && cir_enable_flg == PATH_IRDA) {
		if (!(msm_hsl_read(port, regmap[vid][UARTDM_SR]) &
				UARTDM_SR_TXEMT_BMSK)) {
			while (!(msm_hsl_read(port, regmap[vid][UARTDM_SR]) &
				UARTDM_SR_TXEMT_BMSK)) {
				udelay(1);
				touch_nmi_watchdog();
				cpu_relax();
				
				if (++count > (1000000000 / 115200) * 6) {
					E("Wait too long for Tx end\n");
					break;
				}
			}
			gpio_direction_output(msm_cir_port->cir_learn_en, 1);
		} else {
			gpio_direction_output(msm_cir_port->cir_learn_en, 1);
		}
	}
}

static void msm_hsl_start_tx_cir(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	struct msm_hsl_port *msm_cir_port = htc_cir_port;

	msm_hsl_port->imr |= UARTDM_ISR_TXLEV_BMSK;
	if (msm_cir_port->cir_learn_en && cir_enable_flg == PATH_IRDA) {
		gpio_direction_output(msm_cir_port->cir_learn_en, 0);
	}
	msm_hsl_write(port, msm_hsl_port->imr,
		regmap[msm_hsl_port->ver_id][UARTDM_IMR]);
}

static void msm_hsl_stop_rx_cir(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	msm_hsl_port->imr &= ~(UARTDM_ISR_RXLEV_BMSK |
			       UARTDM_ISR_RXSTALE_BMSK);
	msm_hsl_write(port, msm_hsl_port->imr,
		regmap[msm_hsl_port->ver_id][UARTDM_IMR]);
}

static void msm_hsl_enable_ms_cir(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	msm_hsl_port->imr |= UARTDM_ISR_DELTA_CTS_BMSK;
	msm_hsl_write(port, msm_hsl_port->imr,
		regmap[msm_hsl_port->ver_id][UARTDM_IMR]);
}

static void handle_rx(struct uart_port *port, unsigned int misr)
{
	struct tty_struct *tty = port->state->port.tty;
	unsigned int vid;
	unsigned int sr;
	int count = 0;
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	vid = msm_hsl_port->ver_id;
	if ((msm_hsl_read(port, regmap[vid][UARTDM_SR]) &
				UARTDM_SR_OVERRUN_BMSK)) {
		port->icount.overrun++;
		tty_insert_flip_char(tty, 0, TTY_OVERRUN);
		msm_hsl_write(port, RESET_ERROR_STATUS,
			regmap[vid][UARTDM_CR]);
	}

	if (misr & UARTDM_ISR_RXSTALE_BMSK) {
		count = msm_hsl_read(port,
			regmap[vid][UARTDM_RX_TOTAL_SNAP]) -
			msm_hsl_port->old_snap_state;
		msm_hsl_port->old_snap_state = 0;
	} else {
		count = 4 * (msm_hsl_read(port, regmap[vid][UARTDM_RFWR]));
		msm_hsl_port->old_snap_state += count;
	}

	
	while (count > 0) {
		unsigned int c;
		char flag = TTY_NORMAL;

		sr = msm_hsl_read(port, regmap[vid][UARTDM_SR]);
		if ((sr & UARTDM_SR_RXRDY_BMSK) == 0) {
			msm_hsl_port->old_snap_state -= count;
			break;
		}
		c = msm_hsl_read(port, regmap[vid][UARTDM_RF]);
		if (sr & UARTDM_SR_RX_BREAK_BMSK) {
			port->icount.brk++;
			if (uart_handle_break(port))
				continue;
		} else if (sr & UARTDM_SR_PAR_FRAME_BMSK) {
			port->icount.frame++;
		} else {
			port->icount.rx++;
		}

		
		sr &= port->read_status_mask;
		if (sr & UARTDM_SR_RX_BREAK_BMSK)
			flag = TTY_BREAK;
		else if (sr & UARTDM_SR_PAR_FRAME_BMSK)
			flag = TTY_FRAME;

		
		
		tty_insert_flip_string(tty, (char *) &c,
				       (count > 4) ? 4 : count);
		count -= 4;
	}

	tty_flip_buffer_push(tty);
}

static void handle_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	int sent_tx;
	int tx_count;
	int x;
	unsigned int tf_pointer = 0;
	unsigned int vid;

	vid = UART_TO_MSM(port)->ver_id;
	tx_count = uart_circ_chars_pending(xmit);

	if (tx_count > (UART_XMIT_SIZE - xmit->tail))
		tx_count = UART_XMIT_SIZE - xmit->tail;
	if (tx_count >= port->fifosize)
		tx_count = port->fifosize;

	
	if (port->x_char) {
		wait_for_xmitr(port, UARTDM_ISR_TX_READY_BMSK);
		msm_hsl_write(port, tx_count + 1,
			regmap[vid][UARTDM_NCF_TX]);
		msm_hsl_write(port, port->x_char, regmap[vid][UARTDM_TF]);
		port->icount.tx++;
		port->x_char = 0;
	} else if (tx_count) {
		wait_for_xmitr(port, UARTDM_ISR_TX_READY_BMSK);
		msm_hsl_write(port, tx_count, regmap[vid][UARTDM_NCF_TX]);
	}
	if (!tx_count) {
		msm_hsl_stop_tx_cir(port);
		return;
	}

	while (tf_pointer < tx_count)  {
		if (unlikely(!(msm_hsl_read(port, regmap[vid][UARTDM_SR]) &
			       UARTDM_SR_TXRDY_BMSK)))
			continue;
		switch (tx_count - tf_pointer) {
		case 1: {
			x = xmit->buf[xmit->tail];
			port->icount.tx++;
			break;
		}
		case 2: {
			x = xmit->buf[xmit->tail]
				| xmit->buf[xmit->tail+1] << 8;
			port->icount.tx += 2;
			break;
		}
		case 3: {
			x = xmit->buf[xmit->tail]
				| xmit->buf[xmit->tail+1] << 8
				| xmit->buf[xmit->tail + 2] << 16;
			port->icount.tx += 3;
			break;
		}
		default: {
			x = *((int *)&(xmit->buf[xmit->tail]));
			port->icount.tx += 4;
			break;
		}
		}
		msm_hsl_write(port, x, regmap[vid][UARTDM_TF]);
		xmit->tail = ((tx_count - tf_pointer < 4) ?
			      (tx_count - tf_pointer + xmit->tail) :
			      (xmit->tail + 4)) & (UART_XMIT_SIZE - 1);
		tf_pointer += 4;
		sent_tx = 1;
	}

	if (uart_circ_empty(xmit))
		msm_hsl_stop_tx_cir(port);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

static void handle_delta_cts(struct uart_port *port)
{
	unsigned int vid = UART_TO_MSM(port)->ver_id;

	msm_hsl_write(port, RESET_CTS, regmap[vid][UARTDM_CR]);
	port->icount.cts++;
	wake_up_interruptible(&port->state->port.delta_msr_wait);
}

static irqreturn_t msm_hsl_irq(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	unsigned int vid;
	unsigned int misr;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	vid = msm_hsl_port->ver_id;
	misr = msm_hsl_read(port, regmap[vid][UARTDM_MISR]);
	
	msm_hsl_write(port, 0, regmap[vid][UARTDM_IMR]);

	if (misr & (UARTDM_ISR_RXSTALE_BMSK | UARTDM_ISR_RXLEV_BMSK)) {
		handle_rx(port, misr);
		if (misr & (UARTDM_ISR_RXSTALE_BMSK))
			msm_hsl_write(port, RESET_STALE_INT,
					regmap[vid][UARTDM_CR]);
		msm_hsl_write(port, 6500, regmap[vid][UARTDM_DMRX]);
		msm_hsl_write(port, STALE_EVENT_ENABLE, regmap[vid][UARTDM_CR]);
	}
	if (misr & UARTDM_ISR_TXLEV_BMSK)
		handle_tx(port);

	if (misr & UARTDM_ISR_DELTA_CTS_BMSK)
		handle_delta_cts(port);

	
	msm_hsl_write(port, msm_hsl_port->imr, regmap[vid][UARTDM_IMR]);
	spin_unlock_irqrestore(&port->lock, flags);

	return IRQ_HANDLED;
}

static unsigned int msm_hsl_tx_empty_cir(struct uart_port *port)
{
	unsigned int ret;
	unsigned int vid = UART_TO_MSM(port)->ver_id;

	ret = (msm_hsl_read(port, regmap[vid][UARTDM_SR]) &
	       UARTDM_SR_TXEMT_BMSK) ? TIOCSER_TEMT : 0;
	return ret;
}

static void msm_hsl_reset(struct uart_port *port)
{
	unsigned int vid = UART_TO_MSM(port)->ver_id;

	
	msm_hsl_write(port, RESET_RX, regmap[vid][UARTDM_CR]);
	msm_hsl_write(port, RESET_TX, regmap[vid][UARTDM_CR]);
	msm_hsl_write(port, RESET_ERROR_STATUS, regmap[vid][UARTDM_CR]);
	msm_hsl_write(port, RESET_BREAK_INT, regmap[vid][UARTDM_CR]);
	msm_hsl_write(port, RESET_CTS, regmap[vid][UARTDM_CR]);
	msm_hsl_write(port, RFR_LOW, regmap[vid][UARTDM_CR]);
}

static unsigned int msm_hsl_get_mctrl_cir(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_CTS | TIOCM_DSR | TIOCM_RTS;
}

static void msm_hsl_set_mctrl_cir(struct uart_port *port, unsigned int mctrl)
{
	unsigned int vid = UART_TO_MSM(port)->ver_id;
	unsigned int mr;
	unsigned int loop_mode;

	mr = msm_hsl_read(port, regmap[vid][UARTDM_MR1]);

	if (!(mctrl & TIOCM_RTS)) {
		mr &= ~UARTDM_MR1_RX_RDY_CTL_BMSK;
		msm_hsl_write(port, mr, regmap[vid][UARTDM_MR1]);
		msm_hsl_write(port, RFR_HIGH, regmap[vid][UARTDM_CR]);
	} else {
		mr |= UARTDM_MR1_RX_RDY_CTL_BMSK;
		msm_hsl_write(port, mr, regmap[vid][UARTDM_MR1]);
	}

	loop_mode = TIOCM_LOOP & mctrl;
	if (loop_mode) {
		mr = msm_hsl_read(port, regmap[vid][UARTDM_MR2]);
		mr |= UARTDM_MR2_LOOP_MODE_BMSK;
		msm_hsl_write(port, mr, regmap[vid][UARTDM_MR2]);

		
		msm_hsl_reset(port);

		
		msm_hsl_write(port, UARTDM_CR_RX_EN_BMSK
		      | UARTDM_CR_TX_EN_BMSK, regmap[vid][UARTDM_CR]);
	}
}

static void msm_hsl_break_ctl_cir(struct uart_port *port, int break_ctl)
{
	unsigned int vid = UART_TO_MSM(port)->ver_id;

	if (break_ctl)
		msm_hsl_write(port, START_BREAK, regmap[vid][UARTDM_CR]);
	else
		msm_hsl_write(port, STOP_BREAK, regmap[vid][UARTDM_CR]);
}

static void msm_hsl_set_baud_rate(struct uart_port *port, unsigned int baud)
{
	unsigned int baud_code, rxstale, watermark;
	unsigned int data;
	unsigned int vid;
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	if (port->line == 2) {
		
		if (force_baud_1 == 96)
			baud = 9600;
		else if (force_baud_1 == 1152)
			baud = 115200;
		
	}
	D("%s ()baud %d:port->line %d, ir\n", __func__, baud, port->line);
	switch (baud) {
	case 300:
		baud_code = UARTDM_CSR_75;
		rxstale = 1;
		break;
	case 600:
		baud_code = UARTDM_CSR_150;
		rxstale = 1;
		break;
	case 1200:
		baud_code = UARTDM_CSR_300;
		rxstale = 1;
		break;
	case 2400:
		baud_code = UARTDM_CSR_600;
		rxstale = 1;
		break;
	case 4800:
		baud_code = UARTDM_CSR_1200;
		rxstale = 1;
		break;
	case 9600:
		baud_code = UARTDM_CSR_2400;
		rxstale = 2;
		break;
	case 14400:
		baud_code = UARTDM_CSR_3600;
		rxstale = 3;
		break;
	case 19200:
		baud_code = UARTDM_CSR_4800;
		rxstale = 4;
		break;
	case 28800:
		baud_code = UARTDM_CSR_7200;
		rxstale = 6;
		break;
	case 38400:
		baud_code = UARTDM_CSR_9600;
		rxstale = 8;
		break;
	case 57600:
		baud_code = UARTDM_CSR_14400;
		rxstale = 16;
		break;
	case 115200:
		baud_code = UARTDM_CSR_28800;
		rxstale = 31;
		break;
	case 230400:
		baud_code = UARTDM_CSR_57600;
		rxstale = 31;
		break;
	case 460800:
		baud_code = UARTDM_CSR_115200;
		rxstale = 31;
		break;
	default: 
		baud_code = UARTDM_CSR_28800;
		rxstale = 31;
		break;
	}

	vid = msm_hsl_port->ver_id;
	msm_hsl_write(port, baud_code, regmap[vid][UARTDM_CSR]);

	
		

	
	watermark = UARTDM_IPR_STALE_LSB_BMSK & rxstale;
	watermark |= UARTDM_IPR_STALE_TIMEOUT_MSB_BMSK & (rxstale << 2);
	msm_hsl_write(port, watermark, regmap[vid][UARTDM_IPR]);

	watermark = (port->fifosize * 3) / (4*4);
	msm_hsl_write(port, watermark, regmap[vid][UARTDM_RFWR]);

	
	msm_hsl_write(port, 0, regmap[vid][UARTDM_TFWR]);

	msm_hsl_write(port, CR_PROTECTION_EN, regmap[vid][UARTDM_CR]);
	msm_hsl_reset(port);

	data = UARTDM_CR_TX_EN_BMSK;
	data |= UARTDM_CR_RX_EN_BMSK;
	
	msm_hsl_write(port, data, regmap[vid][UARTDM_CR]);

	msm_hsl_write(port, RESET_STALE_INT, regmap[vid][UARTDM_CR]);
	
	msm_hsl_port->imr = UARTDM_ISR_RXSTALE_BMSK
		| UARTDM_ISR_DELTA_CTS_BMSK | UARTDM_ISR_RXLEV_BMSK;
	msm_hsl_write(port, msm_hsl_port->imr, regmap[vid][UARTDM_IMR]);
	msm_hsl_write(port, 6500, regmap[vid][UARTDM_DMRX]);
	msm_hsl_write(port, STALE_EVENT_ENABLE, regmap[vid][UARTDM_CR]);
}

static void msm_hsl_init_clock(struct uart_port *port)
{
	D("%s ()ok:port->line %d, ir\n", __func__, port->line);
	clk_en(port, 1);
}

static void msm_hsl_deinit_clock(struct uart_port *port)
{
	D("%s ()ok:port->line %d, ir\n", __func__, port->line);
	clk_en(port, 0);
}

static int msm_hsl_startup_cir(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	unsigned int data, rfr_level;
	unsigned int vid;
	int ret;
	unsigned long flags;

	snprintf(msm_hsl_port->name, sizeof(msm_hsl_port->name),
		 "msm_serial_hsl%d", port->line);
	D("%s () :port->line %d, ir\n", __func__, port->line);
	if (!(is_console(port)) || (!port->cons) ||
		(port->cons && (!(port->cons->flags & CON_ENABLED)))) {

		if (msm_serial_hsl_has_gsbi(port)) {
			D("%s () serial_hsl_has_gsbi:port->line %d, ir\n", __func__, port->line);
			if ((ioread32(msm_hsl_port->mapped_gsbi +
				GSBI_CONTROL_ADDR) & GSBI_PROTOCOL_I2C_UART)
					!= GSBI_PROTOCOL_I2C_UART){
				D("%s () iowrite32i:port->line %d, ir\n", __func__, port->line);
				iowrite32(GSBI_PROTOCOL_I2C_UART,
					msm_hsl_port->mapped_gsbi +
						GSBI_CONTROL_ADDR);
			}
		}
	}
#ifndef CONFIG_PM_RUNTIME
	msm_hsl_init_clock(port);
#endif
	pm_runtime_get_sync(port->dev);

	
	if (likely(port->fifosize > 48))
		rfr_level = port->fifosize - 16;
	else
		rfr_level = port->fifosize;

	rfr_level = (rfr_level / 4);

	spin_lock_irqsave(&port->lock, flags);

	vid = msm_hsl_port->ver_id;
	
	data = msm_hsl_read(port, regmap[vid][UARTDM_MR1]);
	data &= ~UARTDM_MR1_AUTO_RFR_LEVEL1_BMSK;
	data &= ~UARTDM_MR1_AUTO_RFR_LEVEL0_BMSK;
	data |= UARTDM_MR1_AUTO_RFR_LEVEL1_BMSK & (rfr_level << 2);
	data |= UARTDM_MR1_AUTO_RFR_LEVEL0_BMSK & rfr_level;
	msm_hsl_write(port, data, regmap[vid][UARTDM_MR1]);
	spin_unlock_irqrestore(&port->lock, flags);

	ret = request_irq(port->irq, msm_hsl_irq, IRQF_TRIGGER_HIGH,
			  msm_hsl_port->name, port);
	if (unlikely(ret)) {
		printk(KERN_ERR "%s: failed to request_irq\n", __func__);
		return ret;
	}
	return 0;
}

static void msm_hsl_shutdown_cir(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	msm_hsl_port->imr = 0;
	
	msm_hsl_write(port, 0, regmap[msm_hsl_port->ver_id][UARTDM_IMR]);

	free_irq(port->irq, port);

#ifndef CONFIG_PM_RUNTIME
	msm_hsl_deinit_clock(port);
#endif
	pm_runtime_put_sync(port->dev);
	if (!(is_console(port)) || (!port->cons) ||
		(port->cons && (!(port->cons->flags & CON_ENABLED)))) {
	}
}

static void msm_hsl_set_termios_cir(struct uart_port *port,
				struct ktermios *termios,
				struct ktermios *old)
{
	unsigned long flags;
	unsigned int baud, mr;
	unsigned int vid;

	spin_lock_irqsave(&port->lock, flags);

	
	baud = uart_get_baud_rate(port, termios, old, 300, 460800);

	msm_hsl_set_baud_rate(port, baud);

	vid = UART_TO_MSM(port)->ver_id;
	
	mr = msm_hsl_read(port, regmap[vid][UARTDM_MR2]);
	mr &= ~UARTDM_MR2_PARITY_MODE_BMSK;
	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & PARODD)
			mr |= ODD_PARITY;
		else if (termios->c_cflag & CMSPAR)
			mr |= SPACE_PARITY;
		else
			mr |= EVEN_PARITY;
	}

	
	mr &= ~UARTDM_MR2_BITS_PER_CHAR_BMSK;
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		mr |= FIVE_BPC;
		break;
	case CS6:
		mr |= SIX_BPC;
		break;
	case CS7:
		mr |= SEVEN_BPC;
		break;
	case CS8:
	default:
		mr |= EIGHT_BPC;
		break;
	}

	
	mr &= ~(STOP_BIT_ONE | STOP_BIT_TWO);
	if (termios->c_cflag & CSTOPB)
		mr |= STOP_BIT_TWO;
	else
		mr |= STOP_BIT_ONE;

	
	msm_hsl_write(port, mr, regmap[vid][UARTDM_MR2]);

	
	mr = msm_hsl_read(port, regmap[vid][UARTDM_MR1]);
	mr &= ~(UARTDM_MR1_CTS_CTL_BMSK | UARTDM_MR1_RX_RDY_CTL_BMSK);
	if (termios->c_cflag & CRTSCTS) {
		mr |= UARTDM_MR1_CTS_CTL_BMSK;
		mr |= UARTDM_MR1_RX_RDY_CTL_BMSK;
	}
	msm_hsl_write(port, mr, regmap[vid][UARTDM_MR1]);

	
	port->read_status_mask = 0;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UARTDM_SR_PAR_FRAME_BMSK;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UARTDM_SR_RX_BREAK_BMSK;

	uart_update_timeout(port, termios->c_cflag, baud);

	spin_unlock_irqrestore(&port->lock, flags);

	D("%s: cir MR is 0x%x\n", __func__, mr);
	D("%s: cir baud is %d\n", __func__, baud);
}

static const char *msm_hsl_type_cir(struct uart_port *port)
{
	return "MSM";
}

static void msm_hsl_release_port_cir(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *uart_resource;
	resource_size_t size;

	D("%s () :port->line %d, ir\n", __func__, port->line);
	uart_resource = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						     "uartdm_resource");
	if (!uart_resource) {
		D("%s ()  uart_resource:port->line %d, ir\n", __func__, port->line);
		uart_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	}
	if (unlikely(!uart_resource)||(!uart_resource))
		return;
	size = uart_resource->end - uart_resource->start + 1;

	release_mem_region(port->mapbase, size);
	iounmap(port->membase);
	port->membase = NULL;

	if (msm_serial_hsl_has_gsbi(port)) {
		D("%s () msm_serial_hsl_has_gsbi :port->line %d, ir\n", __func__, port->line);
		iowrite32(GSBI_PROTOCOL_IDLE, msm_hsl_port->mapped_gsbi +
			  GSBI_CONTROL_ADDR);
		iounmap(msm_hsl_port->mapped_gsbi);
		msm_hsl_port->mapped_gsbi = NULL;
	}
	D("%s () ok :port->line %d, ir\n", __func__, port->line);
}

static int msm_hsl_request_port_cir(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *uart_resource;
	struct resource *gsbi_resource;
	resource_size_t size;

	uart_resource = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						     "uartdm_resource");
	if (!uart_resource) {
		D("%s ():uart_resource :port->line %d, ir\n", __func__, port->line);
		uart_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	}
	if (unlikely(!uart_resource)||(!uart_resource)) {
		E("%s: can't get uartdm resource\n", __func__);
		return -ENXIO;
	}
	size = uart_resource->end - uart_resource->start + 1;

	if (unlikely(!request_mem_region(port->mapbase, size,
					 "msm_serial_cir"))) {
		E("%s: can't get mem region for uartdm\n", __func__);
		return -EBUSY;
	}

	port->membase = ioremap(port->mapbase, size);
	if (!port->membase) {
		release_mem_region(port->mapbase, size);
		return -EBUSY;
	}

	D("%s: memory map for base 0x%x\n", __func__, port->mapbase);
	D("%s ():uart_resource :port->line %d, memory map for base 0x%x ir\n", __func__, port->line, port->mapbase);

	if (msm_serial_hsl_has_gsbi(port)) {
		gsbi_resource = platform_get_resource_byname(pdev,
							     IORESOURCE_MEM,
							     "gsbi_resource");
		if (!gsbi_resource) {
			gsbi_resource = platform_get_resource(pdev,
						IORESOURCE_MEM, 1);
			D("%s ():gsbi_resource :port->line %d, ir\n", __func__, port->line);
		}
		if (unlikely(!gsbi_resource)||(!gsbi_resource)) {
			E("%s: can't get gsbi resource\n", __func__);
			return -ENXIO;
		}
		D("%s: get gsbi_resource for port[%d]\n", __func__, port->line);

		size = gsbi_resource->end - gsbi_resource->start + 1;
		msm_hsl_port->mapped_gsbi = ioremap(gsbi_resource->start,
						    size);
		if (!msm_hsl_port->mapped_gsbi)
			return -EBUSY;
	}

	D("%s () ok:port->line %d, ir\n", __func__, port->line);

	return 0;
}

static void msm_hsl_config_port_cir(struct uart_port *port, int flags)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	D("%s () :port->line %d, ir\n", __func__, port->line);
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_MSM;
		if (msm_hsl_request_port_cir(port)) {
			D("%s () request_port_cir return:port->line %d, ir\n", __func__, port->line);
			return;
		}
	}
	if (msm_serial_hsl_has_gsbi(port)) {
		if (msm_hsl_port->pclk)
			clk_prepare_enable(msm_hsl_port->pclk);
		if ((ioread32(msm_hsl_port->mapped_gsbi + GSBI_CONTROL_ADDR) &
			GSBI_PROTOCOL_I2C_UART) != GSBI_PROTOCOL_I2C_UART)
			iowrite32(GSBI_PROTOCOL_I2C_UART,
				msm_hsl_port->mapped_gsbi + GSBI_CONTROL_ADDR);
		D("%s: cir line[%d] GSBI_CONTROL_ADDR-%x\n", __func__,
			port->line, ioread32(msm_hsl_port->mapped_gsbi + GSBI_CONTROL_ADDR));
		D("%s () GSBI_CONTROL_ADDR:port->line %d, ir\n", __func__, port->line);
		if (msm_hsl_port->pclk)
			clk_disable_unprepare(msm_hsl_port->pclk);
	}
}

static int msm_hsl_verify_port_cir(struct uart_port *port,
			       struct serial_struct *ser)
{
	if (unlikely(ser->type != PORT_UNKNOWN && ser->type != PORT_MSM))
		return -EINVAL;
	if (unlikely(port->irq != ser->irq))
		return -EINVAL;
	return 0;
}

static void msm_hsl_power_cir(struct uart_port *port, unsigned int state,
			  unsigned int oldstate)
{
	int ret;
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	struct msm_hsl_port *msm_cir_port = htc_cir_port;
	unsigned long flags;

	D("%s (): ir, state %d\n", __func__, state);
	switch (state) {
	case 0:
		ret = clk_set_rate(msm_hsl_port->clk, 7372800);
		if (ret)
			E("%s(): Error setting UART clock rate\n",
								__func__);
		clk_en(port, 1);
		break;
	case 3:
		if (cir_enable_flg != PATH_CIR) {
			D("%s path is not CIR. flg = %d\n",
						__func__, cir_enable_flg);
			D("%s(): Clear IRDA mode \n", __func__);
			spin_lock_irqsave(&port->lock, flags);
			if (msm_cir_port->cir_sir_switch)
				gpio_direction_output(msm_cir_port->cir_sir_switch, 0);
			if (msm_cir_port->cir_learn_en)
				gpio_direction_output(msm_cir_port->cir_learn_en, 0);
			gpio_direction_input(msm_cir_port->rst_pin);
			ret = 0;
			msm_hsl_write(port, ret, UARTDM_IRDA_ADDR);
			spin_unlock_irqrestore(&port->lock, flags);

			cir_enable_flg = PATH_CIR;
			if (msm_hsl_port->cir_set_path)
				msm_hsl_port->cir_set_path(PATH_CIR);
		}
		clk_en(port, 0);
		break;
	default:
		E("%s(): msm_serial_hsl: Unknown PM state %d\n",
							__func__, state);
	}
}

static struct uart_ops msm_hsl_uart_pops = {
	.tx_empty = msm_hsl_tx_empty_cir,
	.set_mctrl = msm_hsl_set_mctrl_cir,
	.get_mctrl = msm_hsl_get_mctrl_cir,
	.stop_tx = msm_hsl_stop_tx_cir,
	.start_tx = msm_hsl_start_tx_cir,
	.stop_rx = msm_hsl_stop_rx_cir,
	.enable_ms = msm_hsl_enable_ms_cir,
	.break_ctl = msm_hsl_break_ctl_cir,
	.startup = msm_hsl_startup_cir,
	.shutdown = msm_hsl_shutdown_cir,
	.set_termios = msm_hsl_set_termios_cir,
	.type = msm_hsl_type_cir,
	.release_port = msm_hsl_release_port_cir,
	.request_port = msm_hsl_request_port_cir,
	.config_port = msm_hsl_config_port_cir,
	.verify_port = msm_hsl_verify_port_cir,
	.pm = msm_hsl_power_cir,
};

static struct msm_hsl_port msm_hsl_uart_ports[] = {
	{
		.uart = {
			.iotype = UPIO_MEM,
			.ops = &msm_hsl_uart_pops,
			.flags = UPF_BOOT_AUTOCONF,
			.fifosize = 64,
			.line = 0,
		},
	},
	{
		.uart = {
			.iotype = UPIO_MEM,
			.ops = &msm_hsl_uart_pops,
			.flags = UPF_BOOT_AUTOCONF,
			.fifosize = 64,
			.line = 1,
		},
	},
	{
		.uart = {
			.iotype = UPIO_MEM,
			.ops = &msm_hsl_uart_pops,
			.flags = UPF_BOOT_AUTOCONF,
			.fifosize = 64,
			.line = 2,
		},
	},
#if 0
	{
		.uart = {
			.iotype = UPIO_MEM,
			.ops = &msm_hsl_uart_pops,
			.flags = UPF_BOOT_AUTOCONF,
			.fifosize = 64,
			.line = 3,
		},
	},
	{
		.uart = {
			.iotype = UPIO_MEM,
			.ops = &msm_hsl_uart_pops,
			.flags = UPF_BOOT_AUTOCONF,
			.fifosize = 64,
			.line = 4,
		},
	},
	{
		.uart = {
			.iotype = UPIO_MEM,
			.ops = &msm_hsl_uart_pops,
			.flags = UPF_BOOT_AUTOCONF,
			.fifosize = 64,
			.line = 5,
		},
	},
#endif
};

#define UART_NR	ARRAY_SIZE(msm_hsl_uart_ports)

static inline struct uart_port *get_port_from_line(unsigned int line)
{
	return &msm_hsl_uart_ports[line].uart;
}

void wait_for_xmitr(struct uart_port *port, int bits)
{
	unsigned int vid = UART_TO_MSM(port)->ver_id;

	if (!(msm_hsl_read(port, regmap[vid][UARTDM_SR]) &
			UARTDM_SR_TXEMT_BMSK)) {
		while ((msm_hsl_read(port, regmap[vid][UARTDM_ISR]) &
					bits) != bits) {
			udelay(1);
			touch_nmi_watchdog();
			cpu_relax();
		}
		msm_hsl_write(port, CLEAR_TX_READY, regmap[vid][UARTDM_CR]);
	}
	D("%s ():port->line %d, ir\n", __func__, port->line);
}

#ifdef CONFIG_SERIAL_MSM_HSL_CONSOLE
static void msm_hsl_console_putchar(struct uart_port *port, int ch)
{
	unsigned int vid = UART_TO_MSM(port)->ver_id;

	wait_for_xmitr(port, UARTDM_ISR_TX_READY_BMSK);
	msm_hsl_write(port, 1, regmap[vid][UARTDM_NCF_TX]);

	while (!(msm_hsl_read(port, regmap[vid][UARTDM_SR]) &
				UARTDM_SR_TXRDY_BMSK)) {
		udelay(1);
		touch_nmi_watchdog();
	}

	msm_hsl_write(port, ch, regmap[vid][UARTDM_TF]);
	D("%s ():port->line %d, ir\n", __func__, port->line);
}

static void msm_hsl_console_write(struct console *co, const char *s,
				  unsigned int count)
{
	struct uart_port *port;
	struct msm_hsl_port *msm_hsl_port;
	unsigned int vid;
	int locked;

	BUG_ON(co->index < 0 || co->index >= UART_NR);

	port = get_port_from_line(co->index);
	msm_hsl_port = UART_TO_MSM(port);
	vid = msm_hsl_port->ver_id;

	
	if (port->sysrq || oops_in_progress)
		locked = spin_trylock(&port->lock);
	else {
		locked = 1;
		spin_lock(&port->lock);
	}
	msm_hsl_write(port, 0, regmap[vid][UARTDM_IMR]);
	uart_console_write(port, s, count, msm_hsl_console_putchar);
	msm_hsl_write(port, msm_hsl_port->imr, regmap[vid][UARTDM_IMR]);
	if (locked == 1)
		spin_unlock(&port->lock);

	D("%s ():port->line %d, ir\n", __func__, port->line);
}

static int msm_hsl_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	unsigned int vid;
	int baud = 0, flow, bits, parity;
	int ret;

	D("%s: ir\n", __func__);

	if (unlikely(co->index >= UART_NR || co->index < 0))
		return -ENXIO;

	port = get_port_from_line(co->index);
	vid = UART_TO_MSM(port)->ver_id;
	D("%s ():port->line %d, ir\n", __func__, port->line);
	if (unlikely(!port->membase))
		return -ENXIO;

	port->cons = co;

	pm_runtime_get_noresume(port->dev);

#ifndef CONFIG_PM_RUNTIME
	msm_hsl_init_clock(port);
#endif
	pm_runtime_resume(port->dev);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	bits = 8;
	parity = 'n';
	flow = 'n';
	msm_hsl_write(port, UARTDM_MR2_BITS_PER_CHAR_8 | STOP_BIT_ONE,
		      regmap[vid][UARTDM_MR2]);	

	if (baud < 300 || baud > 115200)
		baud = 115200;
	msm_hsl_set_baud_rate(port, baud);
	D("%s: cir port[%d] baud=%d\n", __func__, port->line, baud);

	ret = uart_set_options(port, co, baud, parity, bits, flow);
	msm_hsl_reset(port);
	
	msm_hsl_write(port, CR_PROTECTION_EN, regmap[vid][UARTDM_CR]);
	msm_hsl_write(port, UARTDM_CR_TX_EN_BMSK, regmap[vid][UARTDM_CR]);

	printk(KERN_INFO "msm_serial_hsl: console setup on port #%d\n",
	       port->line);
	D("%s ():port->line %d, ok, ir\n", __func__, port->line);
	return ret;
}

static struct uart_driver msm_hsl_uart_driver;

static struct console msm_hsl_console = {
	.name = "ttyHSL",
	.write = msm_hsl_console_write,
	.device = uart_console_device,
	.setup = msm_hsl_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &msm_hsl_uart_driver,
};

#define MSM_HSL_CONSOLE	(&msm_hsl_console)
static int get_console_state(struct uart_port *port)
{
	if (is_console(port) && (port->cons->flags & CON_ENABLED)) {
		D("%s ()return 1 :port->line %d, ir\n", __func__, port->line);
		return 1;
	} else {
		D("%s ()return 0 :port->line %d, ir\n", __func__, port->line);
		return 0;
	}
}

static ssize_t show_msm_console(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int enable;
	struct uart_port *port;

	struct platform_device *pdev = to_platform_device(dev);
	port = get_port_from_line(pdev->id);

	enable = get_console_state(port);
	D("%s () :port->line %d, ir\n", __func__, port->line);
	return snprintf(buf, sizeof(enable), "%d\n", enable);
}

static ssize_t set_msm_console(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int enable, cur_state;
	struct uart_port *port;

	struct platform_device *pdev = to_platform_device(dev);
	port = get_port_from_line(pdev->id);

	cur_state = get_console_state(port);
	enable = buf[0] - '0';
	D("%s ():port->line %d,enable %d, cur_state %d ir\n", __func__, port->line, enable, cur_state);
	if (enable == cur_state)
		return count;

	switch (enable) {
	case 0:
		D("%s(): Calling stop_console\n", __func__);
		console_stop(port->cons);
		D("%s(): Calling unregister_console\n", __func__);
		unregister_console(port->cons);
		pm_runtime_put_sync(&pdev->dev);
		pm_runtime_disable(&pdev->dev);
		msm_hsl_power_cir(port, 3, 1);
		break;
	case 1:
		D("%s(): Calling register_console\n", __func__);
		msm_hsl_power_cir(port, 0, 1);
		pm_runtime_enable(&pdev->dev);
		register_console(port->cons);
		break;
	default:
		return -EINVAL;
	}

	return count;
}

static DEVICE_ATTR(console, S_IWUSR | S_IRUGO, show_msm_console,
						set_msm_console);
#else
#define MSM_HSL_CONSOLE	NULL
#endif

static struct uart_driver msm_hsl_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = "msm_serial_cir",
	.dev_name = "ttyHSL",
	.nr = UART_NR,
	.cons = MSM_HSL_CONSOLE,
};

static ssize_t enable_irda_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	return ret;
}
static ssize_t enable_irda_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int irda_en;

	sscanf(buf, "%d", &irda_en);
	if (irda_en == 96) {
		force_baud_1 = 96;
		E("%s(): change baud = 9600\n", __func__);
		return count;
	} else if (irda_en == 1152) {
		E("%s(): change baud = 115200\n", __func__);
		force_baud_1 = 1152;
		return count;
	} else if (irda_en == 1153) {
		E("%s():no  change baud \n", __func__);
		force_baud_1 = 0;
		return count;
	}

	return count;
}

static DEVICE_ATTR(enable_irda, 0664, enable_irda_show, enable_irda_store);

static ssize_t enable_cir_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	return ret;
}
static ssize_t enable_cir_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct msm_hsl_port *msm_cir_port = htc_cir_port;
	struct uart_port *port = &(msm_cir_port->uart);
	unsigned long flags;
	int cir_en, ret = 0;

	sscanf(buf, "%d", &cir_en);
	if (cir_en != 1 && cir_en != 3 && cir_en != 0)
		D("%s: parameter invalid. cir_en = %d", __func__, cir_en);

	D("%s: (cir_enable_flg, cir_en) = (%d, %d)\n",
				__func__, cir_enable_flg, cir_en);

	ret = clk_set_rate(msm_cir_port->clk, 7372800);
	if (!ret) {
		clk_en(port, 1);
		D("%s(): irda Clock enabled for line(%d)\n", __func__, port->line);
	} else {
		D("%s(): Error: Setting the clock rate\n", __func__);
		return -EINVAL;
	}

	if (cir_en > 1) {
		D("%s(): Set IRDA mode\n", __func__);
		spin_lock_irqsave(&port->lock, flags);
		if (msm_cir_port->cir_learn_en)
			gpio_direction_output(msm_cir_port->cir_learn_en, 1);
		gpio_direction_output(msm_cir_port->rst_pin, 0);
		msleep(10);

		if (msm_cir_port->cir_sir_switch)
			gpio_direction_output(msm_cir_port->cir_sir_switch, 1);
		ret = 3;
		ret |= (int)cir_en;
		msm_hsl_write(port, ret, UARTDM_IRDA_ADDR);
		spin_unlock_irqrestore(&port->lock, flags);

		cir_enable_flg = PATH_IRDA;
		if (msm_cir_port->cir_set_path)
			msm_cir_port->cir_set_path(PATH_IRDA);
	}

	clk_en(port, 0);
	return count;
}
static DEVICE_ATTR(enable_cir, 0664, enable_cir_show, enable_cir_store);

static ssize_t enable_learn_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	return ret;
}

static ssize_t enable_learn_store(struct device *dev,
			struct device_attribute *attr, const char *buf,size_t count)
{
	int enable = 0;
	struct msm_hsl_port *msm_cir_port = htc_cir_port;

	sscanf(buf, "%d", &enable);
	D("%s trigger cir learn, input = %d.\n",__func__, enable);
	if ((enable == 1) && (msm_cir_port->cir_learn_en)){
		gpio_direction_output(msm_cir_port->cir_learn_en, 1);
	} else if ((enable == 0) && (msm_cir_port->cir_learn_en)) {
		gpio_direction_output(msm_cir_port->cir_learn_en,0);
	}
	return count;
}
static DEVICE_ATTR(enable_learn, 0666, enable_learn_show, enable_learn_store);

static ssize_t reset_cir_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	return ret;
}
static ssize_t reset_cir_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct msm_hsl_port *msm_cir_port = htc_cir_port;
	int reset;

	sscanf(buf, "%d", &reset);
	D("%s trigger cir reset, input = %d.\n",
					__func__, reset);

	if((reset==1) && (msm_cir_port->cir_reset) ) {
		msm_cir_port->cir_reset();
	}
	if (reset == 1) {
		if (msm_cir_port->rst_pin) {
			
			gpio_direction_output(msm_cir_port->rst_pin, 0);
			msleep(2);
			
			gpio_direction_input(msm_cir_port->rst_pin);
			
		}
	}

	D("%s count = %d.\n", __func__, count);
	return count;
}
static DEVICE_ATTR(reset_cir, 0600, reset_cir_show, reset_cir_store);

static struct cir_platform_data
		*msm_hsl_dt_to_pdata(struct platform_device *pdev)
{
	int ret;
	struct property *prop;
	struct device_node *node = pdev->dev.of_node;
	struct cir_platform_data *pdata;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("unable to allocate memory for platform data\n");
		return ERR_PTR(-ENOMEM);
	}

	ret = of_property_read_u32(node, "id",
				&pdata->id);
	if (ret && ret != -EINVAL) {
		pr_err("Error with config_gpio property.\n");
		return ERR_PTR(ret);
	}
	prop = of_find_property(node, "rst", NULL);
	if (prop) {
		pdata->rst_pin = of_get_named_gpio(node, "rst", 0);
	}
	prop = of_find_property(node, "CIR_SIR", NULL);
	if (prop) {
		pdata->cir_sir_switch = of_get_named_gpio(node, "CIR_SIR", 0);
	}
	prop = of_find_property(node, "CIR_LEARN_EN", NULL);
	if (prop) {
		pdata->cir_learn_en = of_get_named_gpio(node, "CIR_LEARN_EN", 0);
	}

	return pdata;
}

static atomic_t msm_serial_hsl_next_id = ATOMIC_INIT(0);

static int __devinit msm_serial_hsl_probe_cir(struct platform_device *pdev)
{
	struct msm_hsl_port *msm_hsl_port;
	struct resource *uart_resource;
	struct resource *gsbi_resource;
	struct uart_port *port;
	const struct of_device_id *match;
	struct cir_platform_data *pdata;
	int ret;
	u32 line;
	printk("[CIR]%s\n",__func__);
	if (pdev->id == -1)
		pdev->id = atomic_inc_return(&msm_serial_hsl_next_id) - 1;

	pdata = pdev->dev.platform_data;
	line = pdev->id;

	if(pdev->dev.of_node) {
		printk(KERN_INFO "device tree enabled\n");
		ret = of_alias_get_id(pdev->dev.of_node, "serial");
		if (ret >= 0)
			line = ret;

		pdata = msm_hsl_dt_to_pdata(pdev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
		pdev->dev.platform_data = pdata;
	}
	pdev->id = pdata->id;
	line = pdev->id;

	if (unlikely(pdev->id < 0 || pdev->id >= UART_NR))
		return -ENXIO;

	printk(KERN_INFO "msm_serial_cir: detected port #%d\n", pdev->id);

	port = get_port_from_line(pdev->id);
	port->dev = &pdev->dev;
	port->uartclk = 115200;
	msm_hsl_port = UART_TO_MSM(port);
	msm_hsl_port->rst_pin = pdata->rst_pin;
	msm_hsl_port->cir_sir_switch = pdata->cir_sir_switch;
	msm_hsl_port->cir_learn_en = pdata->cir_learn_en;
	htc_cir_port = msm_hsl_port;
	msm_hsl_port->clk = clk_get(&pdev->dev, "core_clk");
	if (unlikely(IS_ERR(msm_hsl_port->clk))) {
		ret = PTR_ERR(msm_hsl_port->clk);
		if (ret != -EPROBE_DEFER)
			pr_err("Error getting clk\n");
		return ret;
	}

	msm_hsl_port->pclk = clk_get(&pdev->dev, "iface_clk");
	if (unlikely(IS_ERR(msm_hsl_port->pclk))) {
		ret = PTR_ERR(msm_hsl_port->pclk);
		if (ret == -EPROBE_DEFER) {
			clk_put(msm_hsl_port->clk);
			return ret;
		} else {
			msm_hsl_port->pclk = NULL;
		}
	}

	msm_hsl_port->func_mode = UART_TWO_WIRE;

	cir_enable_flg = PATH_CIR;
	match = of_match_device(msm_hsl_match_table, &pdev->dev);
	if (!match) {
		msm_hsl_port->ver_id = UARTDM_VERSION_11_13;
	}
	else {
		D("%s () match:port->line %d, ir\n", __func__, port->line);
		msm_hsl_port->ver_id = (unsigned int)match->data;
		msm_hsl_port->uart_type = BLSP_HSUART;

		msm_hsl_port->bus_scale_table = msm_bus_cl_get_pdata(pdev);
		if (!msm_hsl_port->bus_scale_table) {
			pr_err("Bus scaling is disabled\n");
		} else {
			msm_hsl_port->bus_perf_client =
				msm_bus_scale_register_client(
					msm_hsl_port->bus_scale_table);
			if (IS_ERR(&msm_hsl_port->bus_perf_client)) {
				pr_err("Bus client register failed.\n");
				ret = -EINVAL;
				goto err;
			}
		}
	}

	gsbi_resource =	platform_get_resource_byname(pdev,
						     IORESOURCE_MEM,
						     "gsbi_resource");
	if (!gsbi_resource) {
		gsbi_resource = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		D("%s () gsbi_resourc:port->line %d, ir\n", __func__, port->line);
	}
	if (gsbi_resource) {
printk(KERN_INFO "msm_serial_cir: get gsbi_uart_clk and gsbi_pclk\n");
		msm_hsl_port->uart_type = GSBI_HSUART;
	} else {
printk(KERN_INFO "msm_serial_cir: get uartdm_clk\n");
		msm_hsl_port->uart_type = LEGACY_HSUART;
	}

	uart_resource = platform_get_resource_byname(pdev,
						     IORESOURCE_MEM,
						     "uartdm_resource");
	if (!uart_resource)
		uart_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!uart_resource)||(!uart_resource)) {
		printk(KERN_ERR "getting uartdm_resource failed\n");
		return -ENXIO;
	}
	port->mapbase = uart_resource->start;
printk(KERN_INFO "msm_serial_hsl: port[%d] mapbase:%x\n", port->line, port->mapbase);

	port->irq = platform_get_irq(pdev, 0);
	if (unlikely((int)port->irq < 0)) {
		printk(KERN_ERR "%s: getting irq failed\n", __func__);
		return -ENXIO;
	}

	device_set_wakeup_capable(&pdev->dev, 1);
	platform_set_drvdata(pdev, port);
	pm_runtime_enable(port->dev);
#ifdef CONFIG_SERIAL_MSM_HSL_CONSOLE
	ret = device_create_file(&pdev->dev, &dev_attr_console);
	D("%s () device_create_file, port->line %d, ir\n", __func__, port->line);
	if (unlikely(ret))
		E("%s():Can't create console attribute\n", __func__);
#endif
	msm_hsl_debugfs_init(msm_hsl_port, pdev->id);
	mutex_init(&msm_hsl_port->clk_mutex);

	if (msm_hsl_port->pclk) {
		clk_prepare_enable(msm_hsl_port->pclk);
		D("%s () clk_enable, port->line %d, ir\n", __func__, port->line);
	}
	ret = uart_add_one_port(&msm_hsl_uart_driver, port);
	if (msm_hsl_port->pclk) {
		D("%s () clk_disabl, port->line %d, ir\n", __func__, port->line);
		clk_disable_unprepare(msm_hsl_port->pclk);
	}

	D("%s ():port->line %d, ir\n", __func__, port->line);
		msm_hsl_port->irda_class = class_create(THIS_MODULE, "htc_irda");
	if (IS_ERR(msm_hsl_port->irda_class)) {
		ret = PTR_ERR(msm_hsl_port->irda_class);
		msm_hsl_port->irda_class = NULL;
		return -ENXIO;
	}
	msm_hsl_port->irda_dev = device_create(msm_hsl_port->irda_class,
				NULL, 0, "%s", "irda");
	if (unlikely(IS_ERR(msm_hsl_port->irda_dev))) {
		ret = PTR_ERR(msm_hsl_port->irda_dev);
		msm_hsl_port->irda_dev = NULL;
		goto err_create_ls_device;
	}
		
	ret = device_create_file(msm_hsl_port->irda_dev, &dev_attr_enable_irda);
	if (ret)
		goto err_create_ls_device_file;

	msm_hsl_port->cir_class = class_create(THIS_MODULE, "htc_cir");
	if (IS_ERR(msm_hsl_port->cir_class)) {
		ret = PTR_ERR(msm_hsl_port->cir_class);
		msm_hsl_port->cir_class = NULL;
		return -ENXIO;
	}
	msm_hsl_port->cir_dev = device_create(msm_hsl_port->cir_class,
				NULL, 0, "%s", "cir");
	if (unlikely(IS_ERR(msm_hsl_port->cir_dev))) {
		ret = PTR_ERR(msm_hsl_port->cir_dev);
		msm_hsl_port->cir_dev = NULL;
		goto err_create_ls_device;
	}
		
	ret = device_create_file(msm_hsl_port->cir_dev, &dev_attr_enable_cir);
	if (ret)
		goto err_create_ls_device_file;

	ret = device_create_file(msm_hsl_port->cir_dev, &dev_attr_reset_cir);
	if (ret)
		goto err_create_ls_device_file;
	ret = device_create_file(msm_hsl_port->cir_dev, &dev_attr_enable_learn);
	if (ret)
		goto err_create_ls_device_file;
err:
	return ret;

err_create_ls_device_file:
	device_unregister(msm_hsl_port->cir_dev);
err_create_ls_device:
	class_destroy(msm_hsl_port->cir_class);
	return ret;
	return ret;
}

static int __devexit msm_serial_hsl_remove(struct platform_device *pdev)
{
	struct msm_hsl_port *msm_hsl_port = platform_get_drvdata(pdev);
	struct uart_port *port;

	D("%s (): ir\n", __func__);
	port = get_port_from_line(pdev->id);
#ifdef CONFIG_SERIAL_MSM_HSL_CONSOLE
	device_remove_file(&pdev->dev, &dev_attr_console);
#endif
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	device_set_wakeup_capable(&pdev->dev, 0);
	platform_set_drvdata(pdev, NULL);
	uart_remove_one_port(&msm_hsl_uart_driver, port);

	clk_put(msm_hsl_port->pclk);
	clk_put(msm_hsl_port->clk);
	debugfs_remove(msm_hsl_port->loopback_dir);

	return 0;
}

#ifdef CONFIG_PM
static int msm_serial_hsl_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct msm_hsl_port *msm_cir_port = htc_cir_port;
	struct uart_port *port;
	port = get_port_from_line(pdev->id);
	D("%s ():port->line %d, cir_enable_flg = %d\n",
				__func__, port->line, cir_enable_flg);
	if (port) {
		D("%s ():is_console:port->line %d, ir\n", __func__, port->line);
		if (is_console(port))
			msm_hsl_deinit_clock(port);

		uart_suspend_port(&msm_hsl_uart_driver, port);
		if (device_may_wakeup(dev))
			enable_irq_wake(port->irq);

		if (msm_cir_port->cir_set_path)
			msm_cir_port->cir_set_path(PATH_NONE);
	}

	if (msm_cir_port->cir_power)
		msm_cir_port->cir_power(0);

	return 0;
}

static int msm_serial_hsl_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct msm_hsl_port *msm_cir_port = htc_cir_port;
	struct uart_port *port;
	port = get_port_from_line(pdev->id);

	D("%s ():port->line %d, cir_enable_flg = %d\n",
				__func__, port->line, cir_enable_flg);

	if (msm_cir_port->cir_power)
		msm_cir_port->cir_power(1);

	if (port) {
		if (msm_cir_port->cir_set_path)
			msm_cir_port->cir_set_path(cir_enable_flg);

		uart_resume_port(&msm_hsl_uart_driver, port);
		if (device_may_wakeup(dev))
			disable_irq_wake(port->irq);

		if (is_console(port))
			msm_hsl_init_clock(port);
	}

	return 0;
}
#else
#define msm_serial_hsl_suspend NULL
#define msm_serial_hsl_resume NULL
#endif

static int msm_hsl_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct uart_port *port;
	port = get_port_from_line(pdev->id);

	dev_dbg(dev, "pm_runtime: suspending\n");
	msm_hsl_deinit_clock(port);
	D("%s ():port->line %d, ir\n", __func__, port->line);
	return 0;
}

static int msm_hsl_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct uart_port *port;
	port = get_port_from_line(pdev->id);

	dev_dbg(dev, "pm_runtime: resuming\n");
	msm_hsl_init_clock(port);
	D("%s ():port->line %d, ir\n", __func__, port->line);
	return 0;
}

static struct dev_pm_ops msm_hsl_dev_pm_ops = {
	.suspend = msm_serial_hsl_suspend,
	.resume = msm_serial_hsl_resume,
	.runtime_suspend = msm_hsl_runtime_suspend,
	.runtime_resume = msm_hsl_runtime_resume,
};

static struct platform_driver msm_hsl_platform_driver = {
	.probe = msm_serial_hsl_probe_cir,
	.remove = __devexit_p(msm_serial_hsl_remove),
	.driver = {
		.name = "msm_serial_cir",
		.owner = THIS_MODULE,
		.pm = &msm_hsl_dev_pm_ops,
		.of_match_table = msm_hsl_match_table,
	},
};
module_platform_driver(msm_hsl_platform_driver);

static int __init msm_serial_hsl_init_cir(void)
{
	int ret;

	
	if (get_kernel_flag() & KERNEL_FLAG_SERIAL_HSL_ENABLE)
		msm_serial_hsl_enable = 1;

	if (!msm_serial_hsl_enable)
		msm_hsl_uart_driver.cons = NULL;

	ret = uart_register_driver(&msm_hsl_uart_driver);
	if (unlikely(ret))
		return ret;

	debug_base = debugfs_create_dir("msm_serial_cir", NULL);

	if (IS_ERR_OR_NULL(debug_base))
		E("%s():Cannot create debugfs dir\n", __func__);

	ret = platform_driver_register(&msm_hsl_platform_driver);
	if (unlikely(ret))
		uart_unregister_driver(&msm_hsl_uart_driver);

	D("%s(): driver initialized, msm_serial_hsl_enable %d\n", __func__, msm_serial_hsl_enable);
	return ret;
}

static void __exit msm_serial_hsl_exit_cir(void)
{
	debugfs_remove_recursive(debug_base);
#ifdef CONFIG_SERIAL_MSM_HSL_CONSOLE
	if (msm_serial_hsl_enable)
		unregister_console(&msm_hsl_console);
#endif
	platform_driver_unregister(&msm_hsl_platform_driver);
	uart_unregister_driver(&msm_hsl_uart_driver);
	D("%s(): \n", __func__);
}

module_init(msm_serial_hsl_init_cir);
module_exit(msm_serial_hsl_exit_cir);

MODULE_DESCRIPTION("Driver for msm HSUART serial device");
MODULE_LICENSE("GPL v2");
