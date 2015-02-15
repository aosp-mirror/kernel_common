/*
 * ci13xxx_udc.c - MIPS USB IP core family device controller
 *
 * Copyright (C) 2008 Chipidea - MIPS Technologies, Inc. All rights reserved.
 *
 * Author: David Lopo
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/ratelimit.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/usb/msm_hsusb.h>
#include <linux/tracepoint.h>
#include <mach/usb_trace.h>
#include "ci13xxx_udc.h"
#include <mach/htc_battery_common.h>
#include <mach/devices_cmdline.h>


#define DMA_ADDR_INVALID	(~(dma_addr_t)0)
#define USB_MAX_TIMEOUT		25 
#define EP_PRIME_CHECK_DELAY	(jiffies + msecs_to_jiffies(1000))
#define MAX_PRIME_CHECK_RETRY	3 

static DEFINE_SPINLOCK(udc_lock);
extern int msm_otg_usb_disable;

static const struct usb_endpoint_descriptor
ctrl_endpt_out_desc = {
	.bLength         = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = USB_DIR_OUT,
	.bmAttributes    = USB_ENDPOINT_XFER_CONTROL,
	.wMaxPacketSize  = cpu_to_le16(CTRL_PAYLOAD_MAX),
};

static const struct usb_endpoint_descriptor
ctrl_endpt_in_desc = {
	.bLength         = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes    = USB_ENDPOINT_XFER_CONTROL,
	.wMaxPacketSize  = cpu_to_le16(CTRL_PAYLOAD_MAX),
};

static struct ci13xxx *_udc;

#define ISR_MASK   0x1F
static struct {
	u32 test;
	u32 ui;
	u32 uei;
	u32 pci;
	u32 uri;
	u32 sli;
	u32 none;
	struct {
		u32 cnt;
		u32 buf[ISR_MASK+1];
		u32 idx;
	} hndl;
} isr_statistics;

static int ffs_nr(u32 x)
{
	int n = ffs(x);

	return n ? n-1 : 32;
}

struct ci13xxx_ebi_err_entry {
	u32 *usb_req_buf;
	u32 usb_req_length;
	u32 ep_info;
	struct ci13xxx_ebi_err_entry *next;
};

struct ci13xxx_ebi_err_data {
	u32 ebi_err_addr;
	u32 apkt0;
	u32 apkt1;
	struct ci13xxx_ebi_err_entry *ebi_err_entry;
};
static struct ci13xxx_ebi_err_data *ebi_err_data;

static struct {
	unsigned      lpm;    
	void __iomem *abs;    
	void __iomem *cap;    
	size_t        size;   
} hw_bank;

#define ABS_AHBBURST        (0x0090UL)
#define ABS_AHBMODE         (0x0098UL)
#define ABS_CAPLENGTH       (0x100UL)
#define ABS_HCCPARAMS       (0x108UL)
#define ABS_DCCPARAMS       (0x124UL)
#define ABS_TESTMODE        (hw_bank.lpm ? 0x0FCUL : 0x138UL)
#define CAP_USBCMD          (0x000UL)
#define CAP_USBSTS          (0x004UL)
#define CAP_USBINTR         (0x008UL)
#define CAP_DEVICEADDR      (0x014UL)
#define CAP_ENDPTLISTADDR   (0x018UL)
#define CAP_PORTSC          (0x044UL)
#define CAP_DEVLC           (0x084UL)
#define CAP_ENDPTPIPEID     (0x0BCUL)
#define CAP_USBMODE         (hw_bank.lpm ? 0x0C8UL : 0x068UL)
#define CAP_ENDPTSETUPSTAT  (hw_bank.lpm ? 0x0D8UL : 0x06CUL)
#define CAP_ENDPTPRIME      (hw_bank.lpm ? 0x0DCUL : 0x070UL)
#define CAP_ENDPTFLUSH      (hw_bank.lpm ? 0x0E0UL : 0x074UL)
#define CAP_ENDPTSTAT       (hw_bank.lpm ? 0x0E4UL : 0x078UL)
#define CAP_ENDPTCOMPLETE   (hw_bank.lpm ? 0x0E8UL : 0x07CUL)
#define CAP_ENDPTCTRL       (hw_bank.lpm ? 0x0ECUL : 0x080UL)
#define CAP_LAST            (hw_bank.lpm ? 0x12CUL : 0x0C0UL)

#define REMOTE_WAKEUP_DELAY	msecs_to_jiffies(200)

static unsigned hw_ep_max;
static void dbg_usb_op_fail(u8 addr, const char *name,
				const struct ci13xxx_ep *mep);
static inline int hw_ep_bit(int num, int dir)
{
	return num + (dir ? 16 : 0);
}

static int ep_to_bit(int n)
{
	int fill = 16 - hw_ep_max / 2;

	if (n >= hw_ep_max / 2)
		n += fill;

	return n;
}

static u32 hw_aread(u32 addr, u32 mask)
{
	return ioread32(addr + hw_bank.abs) & mask;
}

static void hw_awrite(u32 addr, u32 mask, u32 data)
{
	iowrite32(hw_aread(addr, ~mask) | (data & mask),
		  addr + hw_bank.abs);
}

static u32 hw_cread(u32 addr, u32 mask)
{
	return ioread32(addr + hw_bank.cap) & mask;
}

static void hw_cwrite(u32 addr, u32 mask, u32 data)
{
	iowrite32(hw_cread(addr, ~mask) | (data & mask),
		  addr + hw_bank.cap);
}

static u32 hw_ctest_and_clear(u32 addr, u32 mask)
{
	u32 reg = hw_cread(addr, mask);

	iowrite32(reg, addr + hw_bank.cap);
	return reg;
}

static u32 hw_ctest_and_write(u32 addr, u32 mask, u32 data)
{
	u32 reg = hw_cread(addr, ~0);

	iowrite32((reg & ~mask) | (data & mask), addr + hw_bank.cap);
	return (reg & mask) >> ffs_nr(mask);
}

static int hw_device_init(void __iomem *base)
{
	u32 reg;

	
	hw_bank.abs = base;

	hw_bank.cap = hw_bank.abs;
	hw_bank.cap += ABS_CAPLENGTH;
	hw_bank.cap += ioread8(hw_bank.cap);

	reg = hw_aread(ABS_HCCPARAMS, HCCPARAMS_LEN) >> ffs_nr(HCCPARAMS_LEN);
	hw_bank.lpm  = reg;
	hw_bank.size = hw_bank.cap - hw_bank.abs;
	hw_bank.size += CAP_LAST;
	hw_bank.size /= sizeof(u32);

	reg = hw_aread(ABS_DCCPARAMS, DCCPARAMS_DEN) >> ffs_nr(DCCPARAMS_DEN);
	hw_ep_max = reg * 2;   

	if (hw_ep_max == 0 || hw_ep_max > ENDPT_MAX)
		return -ENODEV;

	

	

	

	return 0;
}
static int hw_device_reset(struct ci13xxx *udc)
{
	int delay_count = 25; 

	
	hw_cwrite(CAP_ENDPTFLUSH, ~0, ~0);
	hw_cwrite(CAP_USBCMD, USBCMD_RS, 0);

	hw_cwrite(CAP_USBCMD, USBCMD_RST, USBCMD_RST);
	while (delay_count--  && hw_cread(CAP_USBCMD, USBCMD_RST))
		udelay(10);
	if (delay_count < 0)
		pr_err("USB controller reset failed\n");

	if (udc->udc_driver->notify_event)
		udc->udc_driver->notify_event(udc,
			CI13XXX_CONTROLLER_RESET_EVENT);

	
	hw_cwrite(CAP_USBMODE, USBMODE_CM, USBMODE_CM_IDLE);
	hw_cwrite(CAP_USBMODE, USBMODE_CM, USBMODE_CM_DEVICE);
	hw_cwrite(CAP_USBMODE, USBMODE_SLOM, USBMODE_SLOM);  

	if (udc->udc_driver->nz_itc)
		hw_cwrite(CAP_USBCMD, USBCMD_ITC_MASK,
			USBCMD_ITC(udc->udc_driver->nz_itc));
	else if (udc->udc_driver->flags & CI13XXX_ZERO_ITC)
		hw_cwrite(CAP_USBCMD, USBCMD_ITC_MASK, USBCMD_ITC(0));

	if (hw_cread(CAP_USBMODE, USBMODE_CM) != USBMODE_CM_DEVICE) {
		pr_err("cannot enter in device mode");
		pr_err("lpm = %i", hw_bank.lpm);
		return -ENODEV;
	}

	return 0;
}

static int hw_device_state(u32 dma)
{
	struct ci13xxx *udc = _udc;
	struct usb_gadget *gadget = &udc->gadget;

	if (dma) {
		if (gadget->streaming_enabled || !(udc->udc_driver->flags &
				CI13XXX_DISABLE_STREAMING)) {
			hw_cwrite(CAP_USBMODE, USBMODE_SDIS, 0);
			pr_debug("%s(): streaming mode is enabled. USBMODE:%x\n",
				__func__, hw_cread(CAP_USBMODE, ~0));
		} else {
			hw_cwrite(CAP_USBMODE, USBMODE_SDIS, USBMODE_SDIS);
			pr_debug("%s(): streaming mode is disabled. USBMODE:%x\n",
				__func__, hw_cread(CAP_USBMODE, ~0));
		}
		hw_cwrite(CAP_ENDPTLISTADDR, ~0, dma);

		if (udc->udc_driver->notify_event)
			udc->udc_driver->notify_event(udc,
				CI13XXX_CONTROLLER_CONNECT_EVENT);

		
		if (udc->udc_driver->flags & CI13XXX_ENABLE_AHB2AHB_BYPASS) {
			hw_awrite(ABS_AHBMODE, AHB2AHB_BYPASS, AHB2AHB_BYPASS);
			pr_debug("%s(): ByPass Mode is enabled. AHBMODE:%x\n",
					__func__, hw_aread(ABS_AHBMODE, ~0));
		}

		
		hw_cwrite(CAP_USBINTR, ~0,
			     USBi_UI|USBi_UEI|USBi_PCI|USBi_URI|USBi_SLI);
		hw_cwrite(CAP_USBCMD, USBCMD_RS, USBCMD_RS);
	} else {
		hw_cwrite(CAP_USBCMD, USBCMD_RS, 0);
		hw_cwrite(CAP_USBINTR, ~0, 0);
		
		if (udc->udc_driver->flags & CI13XXX_ENABLE_AHB2AHB_BYPASS) {
			hw_awrite(ABS_AHBMODE, AHB2AHB_BYPASS, 0);
			pr_debug("%s(): ByPass Mode is disabled. AHBMODE:%x\n",
					__func__, hw_aread(ABS_AHBMODE, ~0));
		}
	}
	return 0;
}

static void debug_ept_flush_info(int ep_num, int dir)
{
	struct ci13xxx *udc = _udc;
	struct ci13xxx_ep *mep;

	if (dir)
		mep = &udc->ci13xxx_ep[ep_num + hw_ep_max/2];
	else
		mep = &udc->ci13xxx_ep[ep_num];

	pr_err_ratelimited("USB Registers\n");
	pr_err_ratelimited("USBCMD:%x\n", hw_cread(CAP_USBCMD, ~0));
	pr_err_ratelimited("USBSTS:%x\n", hw_cread(CAP_USBSTS, ~0));
	pr_err_ratelimited("ENDPTLISTADDR:%x\n",
			hw_cread(CAP_ENDPTLISTADDR, ~0));
	pr_err_ratelimited("PORTSC:%x\n", hw_cread(CAP_PORTSC, ~0));
	pr_err_ratelimited("USBMODE:%x\n", hw_cread(CAP_USBMODE, ~0));
	pr_err_ratelimited("ENDPTSTAT:%x\n", hw_cread(CAP_ENDPTSTAT, ~0));

	dbg_usb_op_fail(0xFF, "FLUSHF", mep);
}
static int hw_ep_flush(int num, int dir)
{
	ktime_t start, diff;
	int n = hw_ep_bit(num, dir);
	struct ci13xxx_ep *mEp = &_udc->ci13xxx_ep[n];

	
	if (_udc->skip_flush || (num && list_empty(&mEp->qh.queue)))
		return 0;

	start = ktime_get();
	do {
		
		hw_cwrite(CAP_ENDPTFLUSH, BIT(n), BIT(n));
		while (hw_cread(CAP_ENDPTFLUSH, BIT(n))) {
			cpu_relax();
			diff = ktime_sub(ktime_get(), start);
			if (ktime_to_ms(diff) > USB_MAX_TIMEOUT) {
				printk_ratelimited(KERN_ERR
					"%s: Failed to flush ep#%d %s\n",
					__func__, num,
					dir ? "IN" : "OUT");
				debug_ept_flush_info(num, dir);
				_udc->skip_flush = true;
				return 0;
			}
		}
	} while (hw_cread(CAP_ENDPTSTAT, BIT(n)));

	return 0;
}

static int hw_ep_disable(int num, int dir)
{
	hw_cwrite(CAP_ENDPTCTRL + num * sizeof(u32),
		  dir ? ENDPTCTRL_TXE : ENDPTCTRL_RXE, 0);
	return 0;
}

static int hw_ep_enable(int num, int dir, int type)
{
	u32 mask, data;

	if (dir) {
		mask  = ENDPTCTRL_TXT;  
		data  = type << ffs_nr(mask);

		mask |= ENDPTCTRL_TXS;  
		mask |= ENDPTCTRL_TXR;  
		data |= ENDPTCTRL_TXR;
		mask |= ENDPTCTRL_TXE;  
		data |= ENDPTCTRL_TXE;
	} else {
		mask  = ENDPTCTRL_RXT;  
		data  = type << ffs_nr(mask);

		mask |= ENDPTCTRL_RXS;  
		mask |= ENDPTCTRL_RXR;  
		data |= ENDPTCTRL_RXR;
		mask |= ENDPTCTRL_RXE;  
		data |= ENDPTCTRL_RXE;
	}
	hw_cwrite(CAP_ENDPTCTRL + num * sizeof(u32), mask, data);

	
	mb();

	return 0;
}

static int hw_ep_get_halt(int num, int dir)
{
	u32 mask = dir ? ENDPTCTRL_TXS : ENDPTCTRL_RXS;

	return hw_cread(CAP_ENDPTCTRL + num * sizeof(u32), mask) ? 1 : 0;
}

static int hw_test_and_clear_setup_status(int n)
{
	n = ep_to_bit(n);
	return hw_ctest_and_clear(CAP_ENDPTSETUPSTAT, BIT(n));
}

static int hw_ep_prime(int num, int dir, int is_ctrl)
{
	int n = hw_ep_bit(num, dir);

	if (is_ctrl && dir == RX && hw_cread(CAP_ENDPTSETUPSTAT, BIT(num)))
		return -EAGAIN;

	hw_cwrite(CAP_ENDPTPRIME, BIT(n), BIT(n));

	if (is_ctrl && dir == RX  && hw_cread(CAP_ENDPTSETUPSTAT, BIT(num)))
		return -EAGAIN;

	
	return 0;
}

static int hw_ep_set_halt(int num, int dir, int value)
{
	u32 addr, mask_xs, mask_xr;

	if (value != 0 && value != 1)
		return -EINVAL;

	do {
		if (hw_cread(CAP_ENDPTSETUPSTAT, BIT(num)))
			return 0;

		addr = CAP_ENDPTCTRL + num * sizeof(u32);
		mask_xs = dir ? ENDPTCTRL_TXS : ENDPTCTRL_RXS;
		mask_xr = dir ? ENDPTCTRL_TXR : ENDPTCTRL_RXR;

		
		hw_cwrite(addr, mask_xs|mask_xr, value ? mask_xs : mask_xr);

	} while (value != hw_ep_get_halt(num, dir));

	return 0;
}

static int hw_intr_clear(int n)
{
	if (n >= REG_BITS)
		return -EINVAL;

	hw_cwrite(CAP_USBINTR, BIT(n), 0);
	hw_cwrite(CAP_USBSTS,  BIT(n), BIT(n));
	return 0;
}

static int hw_intr_force(int n)
{
	if (n >= REG_BITS)
		return -EINVAL;

	hw_awrite(ABS_TESTMODE, TESTMODE_FORCE, TESTMODE_FORCE);
	hw_cwrite(CAP_USBINTR,  BIT(n), BIT(n));
	hw_cwrite(CAP_USBSTS,   BIT(n), BIT(n));
	hw_awrite(ABS_TESTMODE, TESTMODE_FORCE, 0);
	return 0;
}

static int hw_port_is_high_speed(void)
{
	return hw_bank.lpm ? hw_cread(CAP_DEVLC, DEVLC_PSPD) :
		hw_cread(CAP_PORTSC, PORTSC_HSP);
}

static u8 hw_port_test_get(void)
{
	return hw_cread(CAP_PORTSC, PORTSC_PTC) >> ffs_nr(PORTSC_PTC);
}

static int hw_port_test_set(u8 mode)
{
	const u8 TEST_MODE_MAX = 7;

	if (mode > TEST_MODE_MAX)
		return -EINVAL;

	hw_cwrite(CAP_PORTSC, PORTSC_PTC, mode << ffs_nr(PORTSC_PTC));
	return 0;
}

static u32 hw_read_intr_enable(void)
{
	return hw_cread(CAP_USBINTR, ~0);
}

static u32 hw_read_intr_status(void)
{
	return hw_cread(CAP_USBSTS, ~0);
}

static size_t hw_register_read(u32 *buf, size_t size)
{
	unsigned i;

	if (size > hw_bank.size)
		size = hw_bank.size;

	for (i = 0; i < size; i++)
		buf[i] = hw_aread(i * sizeof(u32), ~0);

	return size;
}

static int hw_register_write(u16 addr, u32 data)
{
	
	addr /= sizeof(u32);

	if (addr >= hw_bank.size)
		return -EINVAL;

	
	addr *= sizeof(u32);

	hw_awrite(addr, ~0, data);
	return 0;
}

static int hw_test_and_clear_complete(int n)
{
	n = ep_to_bit(n);
	return hw_ctest_and_clear(CAP_ENDPTCOMPLETE, BIT(n));
}

static u32 hw_test_and_clear_intr_active(void)
{
	u32 reg = hw_read_intr_status() & hw_read_intr_enable();

	hw_cwrite(CAP_USBSTS, ~0, reg);
	return reg;
}

static int hw_test_and_clear_setup_guard(void)
{
	return hw_ctest_and_write(CAP_USBCMD, USBCMD_SUTW, 0);
}

static int hw_test_and_set_setup_guard(void)
{
	return hw_ctest_and_write(CAP_USBCMD, USBCMD_SUTW, USBCMD_SUTW);
}

static int hw_usb_set_address(u8 value)
{
	
	hw_cwrite(CAP_DEVICEADDR, DEVICEADDR_USBADR | DEVICEADDR_USBADRA,
		  value << ffs_nr(DEVICEADDR_USBADR) | DEVICEADDR_USBADRA);
	return 0;
}

static int hw_usb_reset(void)
{
	int delay_count = 10; 

	hw_usb_set_address(0);

	
	hw_cwrite(CAP_ENDPTFLUSH,    ~0, ~0);   

	
	hw_cwrite(CAP_ENDPTCOMPLETE,  0,  0);   

	
	while (delay_count-- && hw_cread(CAP_ENDPTPRIME, ~0))
		udelay(10);
	if (delay_count < 0)
		pr_err("ENDPTPRIME is not cleared during bus reset\n");

	


	return 0;
}

static ssize_t show_device(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct ci13xxx *udc = container_of(dev, struct ci13xxx, gadget.dev);
	struct usb_gadget *gadget = &udc->gadget;
	int n = 0;

	dbg_trace("[%s] %p\n", __func__, buf);
	if (attr == NULL || buf == NULL) {
		dev_err(dev, "[%s] EINVAL\n", __func__);
		return 0;
	}

	n += scnprintf(buf + n, PAGE_SIZE - n, "speed             = %d\n",
		       gadget->speed);
	n += scnprintf(buf + n, PAGE_SIZE - n, "max_speed         = %d\n",
		       gadget->max_speed);
	
	n += scnprintf(buf + n, PAGE_SIZE - n, "is_dualspeed      = %d\n",
		       gadget_is_dualspeed(gadget));
	n += scnprintf(buf + n, PAGE_SIZE - n, "is_otg            = %d\n",
		       gadget->is_otg);
	n += scnprintf(buf + n, PAGE_SIZE - n, "is_a_peripheral   = %d\n",
		       gadget->is_a_peripheral);
	n += scnprintf(buf + n, PAGE_SIZE - n, "b_hnp_enable      = %d\n",
		       gadget->b_hnp_enable);
	n += scnprintf(buf + n, PAGE_SIZE - n, "a_hnp_support     = %d\n",
		       gadget->a_hnp_support);
	n += scnprintf(buf + n, PAGE_SIZE - n, "a_alt_hnp_support = %d\n",
		       gadget->a_alt_hnp_support);
	n += scnprintf(buf + n, PAGE_SIZE - n, "name              = %s\n",
		       (gadget->name ? gadget->name : ""));

	return n;
}
static DEVICE_ATTR(device, S_IRUSR, show_device, NULL);

static ssize_t show_driver(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct ci13xxx *udc = container_of(dev, struct ci13xxx, gadget.dev);
	struct usb_gadget_driver *driver = udc->driver;
	int n = 0;

	dbg_trace("[%s] %p\n", __func__, buf);
	if (attr == NULL || buf == NULL) {
		dev_err(dev, "[%s] EINVAL\n", __func__);
		return 0;
	}

	if (driver == NULL)
		return scnprintf(buf, PAGE_SIZE,
				 "There is no gadget attached!\n");

	n += scnprintf(buf + n, PAGE_SIZE - n, "function  = %s\n",
		       (driver->function ? driver->function : ""));
	n += scnprintf(buf + n, PAGE_SIZE - n, "max speed = %d\n",
		       driver->max_speed);

	return n;
}
static DEVICE_ATTR(driver, S_IRUSR, show_driver, NULL);

#define DBG_DATA_MSG   64UL

#define DBG_DATA_MAX   128UL

static struct {
	char     (buf[DBG_DATA_MAX])[DBG_DATA_MSG];   
	unsigned idx;   
	unsigned tty;   
	rwlock_t lck;   
} dbg_data = {
	.idx = 0,
	.tty = 0,
	.lck = __RW_LOCK_UNLOCKED(lck)
};

static void dbg_dec(unsigned *idx)
{
	*idx = (*idx - 1) & (DBG_DATA_MAX-1);
}

static void dbg_inc(unsigned *idx)
{
	*idx = (*idx + 1) & (DBG_DATA_MAX-1);
}


static unsigned int ep_addr_txdbg_mask;
module_param(ep_addr_txdbg_mask, uint, S_IRUGO | S_IWUSR);
static unsigned int ep_addr_rxdbg_mask;
module_param(ep_addr_rxdbg_mask, uint, S_IRUGO | S_IWUSR);

static int allow_dbg_print(u8 addr)
{
	int dir, num;

	
	if (addr == 0xff)
		return 1;

	dir = addr & USB_ENDPOINT_DIR_MASK ? TX : RX;
	num = addr & ~USB_ENDPOINT_DIR_MASK;
	num = 1 << num;

	if ((dir == TX) && (num & ep_addr_txdbg_mask))
		return 1;
	if ((dir == RX) && (num & ep_addr_rxdbg_mask))
		return 1;

	return 0;
}

static void dbg_print(u8 addr, const char *name, int status, const char *extra)
{
	struct timeval tval;
	unsigned int stamp;
	unsigned long flags;

	if (!allow_dbg_print(addr))
		return;

	write_lock_irqsave(&dbg_data.lck, flags);

	do_gettimeofday(&tval);
	stamp = tval.tv_sec & 0xFFFF;	
	stamp = stamp * 1000000 + tval.tv_usec;

	scnprintf(dbg_data.buf[dbg_data.idx], DBG_DATA_MSG,
		  "%04X\t? %02X %-7.7s %4i ?\t%s\n",
		  stamp, addr, name, status, extra);

	dbg_inc(&dbg_data.idx);

	write_unlock_irqrestore(&dbg_data.lck, flags);

	if (dbg_data.tty != 0)
		pr_notice("%04X\t? %02X %-7.7s %4i ?\t%s\n",
			  stamp, addr, name, status, extra);
}

static void dbg_done(u8 addr, const u32 token, int status)
{
	char msg[DBG_DATA_MSG];

	scnprintf(msg, sizeof(msg), "%d %02X",
		  (int)(token & TD_TOTAL_BYTES) >> ffs_nr(TD_TOTAL_BYTES),
		  (int)(token & TD_STATUS)      >> ffs_nr(TD_STATUS));
	dbg_print(addr, "DONE", status, msg);
}

static void dbg_event(u8 addr, const char *name, int status)
{
	if (name != NULL)
		dbg_print(addr, name, status, "");
}

static void dbg_queue(u8 addr, const struct usb_request *req, int status)
{
	char msg[DBG_DATA_MSG];

	if (req != NULL) {
		scnprintf(msg, sizeof(msg),
			  "%d %d", !req->no_interrupt, req->length);
		dbg_print(addr, "QUEUE", status, msg);
	}
}

static void dbg_setup(u8 addr, const struct usb_ctrlrequest *req)
{
	char msg[DBG_DATA_MSG];

	if (req != NULL) {
		scnprintf(msg, sizeof(msg),
			  "%02X %02X %04X %04X %d", req->bRequestType,
			  req->bRequest, le16_to_cpu(req->wValue),
			  le16_to_cpu(req->wIndex), le16_to_cpu(req->wLength));
		dbg_print(addr, "SETUP", 0, msg);
	}
}

static void dbg_usb_op_fail(u8 addr, const char *name,
				const struct ci13xxx_ep *mep)
{
	char msg[DBG_DATA_MSG];
	struct ci13xxx_req *req;
	struct list_head *ptr = NULL;

	if (mep != NULL) {
		scnprintf(msg, sizeof(msg),
			"%s Fail EP%d%s QH:%08X",
			name, mep->num,
			mep->dir ? "IN" : "OUT", mep->qh.ptr->cap);
		dbg_print(addr, name, 0, msg);
		scnprintf(msg, sizeof(msg),
				"cap:%08X %08X %08X\n",
				mep->qh.ptr->curr, mep->qh.ptr->td.next,
				mep->qh.ptr->td.token);
		dbg_print(addr, "QHEAD", 0, msg);

		list_for_each(ptr, &mep->qh.queue) {
			req = list_entry(ptr, struct ci13xxx_req, queue);
			scnprintf(msg, sizeof(msg),
					"%08X:%08X:%08X\n",
					req->dma, req->ptr->next,
					req->ptr->token);
			dbg_print(addr, "REQ", 0, msg);
			scnprintf(msg, sizeof(msg), "%08X:%d\n",
					req->ptr->page[0],
					req->req.status);
			dbg_print(addr, "REQPAGE", 0, msg);
		}
	}
}

static ssize_t show_events(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	unsigned long flags;
	unsigned i, j, n = 0;

	dbg_trace("[%s] %p\n", __func__, buf);
	if (attr == NULL || buf == NULL) {
		dev_err(dev, "[%s] EINVAL\n", __func__);
		return 0;
	}

	read_lock_irqsave(&dbg_data.lck, flags);

	i = dbg_data.idx;
	for (dbg_dec(&i); i != dbg_data.idx; dbg_dec(&i)) {
		n += strlen(dbg_data.buf[i]);
		if (n >= PAGE_SIZE) {
			n -= strlen(dbg_data.buf[i]);
			break;
		}
	}
	for (j = 0, dbg_inc(&i); j < n; dbg_inc(&i))
		j += scnprintf(buf + j, PAGE_SIZE - j,
			       "%s", dbg_data.buf[i]);

	read_unlock_irqrestore(&dbg_data.lck, flags);

	return n;
}

static ssize_t store_events(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	unsigned tty;

	dbg_trace("[%s] %p, %d\n", __func__, buf, count);
	if (attr == NULL || buf == NULL) {
		dev_err(dev, "[%s] EINVAL\n", __func__);
		goto done;
	}

	if (sscanf(buf, "%u", &tty) != 1 || tty > 1) {
		dev_err(dev, "<1|0>: enable|disable console log\n");
		goto done;
	}

	dbg_data.tty = tty;
	dev_info(dev, "tty = %u", dbg_data.tty);

 done:
	return count;
}
static DEVICE_ATTR(events, S_IRUSR | S_IWUSR, show_events, store_events);

static ssize_t show_inters(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct ci13xxx *udc = container_of(dev, struct ci13xxx, gadget.dev);
	unsigned long flags;
	u32 intr;
	unsigned i, j, n = 0;

	dbg_trace("[%s] %p\n", __func__, buf);
	if (attr == NULL || buf == NULL) {
		dev_err(dev, "[%s] EINVAL\n", __func__);
		return 0;
	}

	spin_lock_irqsave(udc->lock, flags);

	n += scnprintf(buf + n, PAGE_SIZE - n,
		       "status = %08x\n", hw_read_intr_status());
	n += scnprintf(buf + n, PAGE_SIZE - n,
		       "enable = %08x\n", hw_read_intr_enable());

	n += scnprintf(buf + n, PAGE_SIZE - n, "*test = %d\n",
		       isr_statistics.test);
	n += scnprintf(buf + n, PAGE_SIZE - n, "? ui  = %d\n",
		       isr_statistics.ui);
	n += scnprintf(buf + n, PAGE_SIZE - n, "? uei = %d\n",
		       isr_statistics.uei);
	n += scnprintf(buf + n, PAGE_SIZE - n, "? pci = %d\n",
		       isr_statistics.pci);
	n += scnprintf(buf + n, PAGE_SIZE - n, "? uri = %d\n",
		       isr_statistics.uri);
	n += scnprintf(buf + n, PAGE_SIZE - n, "? sli = %d\n",
		       isr_statistics.sli);
	n += scnprintf(buf + n, PAGE_SIZE - n, "*none = %d\n",
		       isr_statistics.none);
	n += scnprintf(buf + n, PAGE_SIZE - n, "*hndl = %d\n",
		       isr_statistics.hndl.cnt);

	for (i = isr_statistics.hndl.idx, j = 0; j <= ISR_MASK; j++, i++) {
		i   &= ISR_MASK;
		intr = isr_statistics.hndl.buf[i];

		if (USBi_UI  & intr)
			n += scnprintf(buf + n, PAGE_SIZE - n, "ui  ");
		intr &= ~USBi_UI;
		if (USBi_UEI & intr)
			n += scnprintf(buf + n, PAGE_SIZE - n, "uei ");
		intr &= ~USBi_UEI;
		if (USBi_PCI & intr)
			n += scnprintf(buf + n, PAGE_SIZE - n, "pci ");
		intr &= ~USBi_PCI;
		if (USBi_URI & intr)
			n += scnprintf(buf + n, PAGE_SIZE - n, "uri ");
		intr &= ~USBi_URI;
		if (USBi_SLI & intr)
			n += scnprintf(buf + n, PAGE_SIZE - n, "sli ");
		intr &= ~USBi_SLI;
		if (intr)
			n += scnprintf(buf + n, PAGE_SIZE - n, "??? ");
		if (isr_statistics.hndl.buf[i])
			n += scnprintf(buf + n, PAGE_SIZE - n, "\n");
	}

	spin_unlock_irqrestore(udc->lock, flags);

	return n;
}

static ssize_t store_inters(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct ci13xxx *udc = container_of(dev, struct ci13xxx, gadget.dev);
	unsigned long flags;
	unsigned en, bit;

	dbg_trace("[%s] %p, %d\n", __func__, buf, count);
	if (attr == NULL || buf == NULL) {
		dev_err(dev, "[%s] EINVAL\n", __func__);
		goto done;
	}

	if (sscanf(buf, "%u %u", &en, &bit) != 2 || en > 1) {
		dev_err(dev, "<1|0> <bit>: enable|disable interrupt");
		goto done;
	}

	spin_lock_irqsave(udc->lock, flags);
	if (en) {
		if (hw_intr_force(bit))
			dev_err(dev, "invalid bit number\n");
		else
			isr_statistics.test++;
	} else {
		if (hw_intr_clear(bit))
			dev_err(dev, "invalid bit number\n");
	}
	spin_unlock_irqrestore(udc->lock, flags);

 done:
	return count;
}
static DEVICE_ATTR(inters, S_IRUSR | S_IWUSR, show_inters, store_inters);

static ssize_t show_port_test(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct ci13xxx *udc = container_of(dev, struct ci13xxx, gadget.dev);
	unsigned long flags;
	unsigned mode;

	dbg_trace("[%s] %p\n", __func__, buf);
	if (attr == NULL || buf == NULL) {
		dev_err(dev, "[%s] EINVAL\n", __func__);
		return 0;
	}

	spin_lock_irqsave(udc->lock, flags);
	mode = hw_port_test_get();
	spin_unlock_irqrestore(udc->lock, flags);

	return scnprintf(buf, PAGE_SIZE, "mode = %u\n", mode);
}

static ssize_t store_port_test(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct ci13xxx *udc = container_of(dev, struct ci13xxx, gadget.dev);
	unsigned long flags;
	unsigned mode;

	dbg_trace("[%s] %p, %d\n", __func__, buf, count);
	if (attr == NULL || buf == NULL) {
		dev_err(dev, "[%s] EINVAL\n", __func__);
		goto done;
	}

	if (sscanf(buf, "%u", &mode) != 1) {
		dev_err(dev, "<mode>: set port test mode");
		goto done;
	}

	spin_lock_irqsave(udc->lock, flags);
	if (hw_port_test_set(mode))
		dev_err(dev, "invalid mode\n");
	spin_unlock_irqrestore(udc->lock, flags);

 done:
	return count;
}
static DEVICE_ATTR(port_test, S_IRUSR | S_IWUSR,
		   show_port_test, store_port_test);

static ssize_t show_qheads(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct ci13xxx *udc = container_of(dev, struct ci13xxx, gadget.dev);
	unsigned long flags;
	unsigned i, j, n = 0;

	dbg_trace("[%s] %p\n", __func__, buf);
	if (attr == NULL || buf == NULL) {
		dev_err(dev, "[%s] EINVAL\n", __func__);
		return 0;
	}

	spin_lock_irqsave(udc->lock, flags);
	for (i = 0; i < hw_ep_max/2; i++) {
		struct ci13xxx_ep *mEpRx = &udc->ci13xxx_ep[i];
		struct ci13xxx_ep *mEpTx = &udc->ci13xxx_ep[i + hw_ep_max/2];
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "EP=%02i: RX=%08X TX=%08X\n",
			       i, (u32)mEpRx->qh.dma, (u32)mEpTx->qh.dma);
		for (j = 0; j < (sizeof(struct ci13xxx_qh)/sizeof(u32)); j++) {
			n += scnprintf(buf + n, PAGE_SIZE - n,
				       " %04X:    %08X    %08X\n", j,
				       *((u32 *)mEpRx->qh.ptr + j),
				       *((u32 *)mEpTx->qh.ptr + j));
		}
	}
	spin_unlock_irqrestore(udc->lock, flags);

	return n;
}
static DEVICE_ATTR(qheads, S_IRUSR, show_qheads, NULL);

#define DUMP_ENTRIES	512
static ssize_t show_registers(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct ci13xxx *udc = container_of(dev, struct ci13xxx, gadget.dev);
	unsigned long flags;
	u32 *dump;
	unsigned i, k, n = 0;

	dbg_trace("[%s] %p\n", __func__, buf);
	if (attr == NULL || buf == NULL) {
		dev_err(dev, "[%s] EINVAL\n", __func__);
		return 0;
	}

	dump = kmalloc(sizeof(u32) * DUMP_ENTRIES, GFP_KERNEL);
	if (!dump) {
		dev_err(dev, "%s: out of memory\n", __func__);
		return 0;
	}

	spin_lock_irqsave(udc->lock, flags);
	k = hw_register_read(dump, DUMP_ENTRIES);
	spin_unlock_irqrestore(udc->lock, flags);

	for (i = 0; i < k; i++) {
		n += scnprintf(buf + n, PAGE_SIZE - n,
			       "reg[0x%04X] = 0x%08X\n",
			       i * (unsigned)sizeof(u32), dump[i]);
	}
	kfree(dump);

	return n;
}

static ssize_t store_registers(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct ci13xxx *udc = container_of(dev, struct ci13xxx, gadget.dev);
	unsigned long addr, data, flags;

	dbg_trace("[%s] %p, %d\n", __func__, buf, count);
	if (attr == NULL || buf == NULL) {
		dev_err(dev, "[%s] EINVAL\n", __func__);
		goto done;
	}

	if (sscanf(buf, "%li %li", &addr, &data) != 2) {
		dev_err(dev, "<addr> <data>: write data to register address");
		goto done;
	}

	spin_lock_irqsave(udc->lock, flags);
	if (hw_register_write(addr, data))
		dev_err(dev, "invalid address range\n");
	spin_unlock_irqrestore(udc->lock, flags);

 done:
	return count;
}
static DEVICE_ATTR(registers, S_IRUSR | S_IWUSR,
		   show_registers, store_registers);

static ssize_t show_requests(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct ci13xxx *udc = container_of(dev, struct ci13xxx, gadget.dev);
	unsigned long flags;
	struct list_head   *ptr = NULL;
	struct ci13xxx_req *req = NULL;
	unsigned i, j, n = 0, qSize = sizeof(struct ci13xxx_td)/sizeof(u32);

	dbg_trace("[%s] %p\n", __func__, buf);
	if (attr == NULL || buf == NULL) {
		dev_err(dev, "[%s] EINVAL\n", __func__);
		return 0;
	}

	spin_lock_irqsave(udc->lock, flags);
	for (i = 0; i < hw_ep_max; i++)
		list_for_each(ptr, &udc->ci13xxx_ep[i].qh.queue)
		{
			req = list_entry(ptr, struct ci13xxx_req, queue);

			n += scnprintf(buf + n, PAGE_SIZE - n,
					"EP=%02i: TD=%08X %s\n",
					i % hw_ep_max/2, (u32)req->dma,
					((i < hw_ep_max/2) ? "RX" : "TX"));

			for (j = 0; j < qSize; j++)
				n += scnprintf(buf + n, PAGE_SIZE - n,
						" %04X:    %08X\n", j,
						*((u32 *)req->ptr + j));
		}
	spin_unlock_irqrestore(udc->lock, flags);

	return n;
}
static DEVICE_ATTR(requests, S_IRUSR, show_requests, NULL);

static ssize_t prime_ept(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct ci13xxx *udc = container_of(dev, struct ci13xxx, gadget.dev);
	struct ci13xxx_ep *mEp;
	unsigned int ep_num, dir;
	int n;
	struct ci13xxx_req *mReq = NULL;

	if (sscanf(buf, "%u %u", &ep_num, &dir) != 2) {
		dev_err(dev, "<ep_num> <dir>: prime the ep");
		goto done;
	}

	if (dir)
		mEp = &udc->ci13xxx_ep[ep_num + hw_ep_max/2];
	else
		mEp = &udc->ci13xxx_ep[ep_num];

	n = hw_ep_bit(mEp->num, mEp->dir);
	mReq =  list_entry(mEp->qh.queue.next, struct ci13xxx_req, queue);
	mEp->qh.ptr->td.next   = mReq->dma;
	mEp->qh.ptr->td.token &= ~TD_STATUS;

	wmb();

	hw_cwrite(CAP_ENDPTPRIME, BIT(n), BIT(n));
	while (hw_cread(CAP_ENDPTPRIME, BIT(n)))
		cpu_relax();

	pr_info("%s: prime:%08x stat:%08x ep#%d dir:%s\n", __func__,
			hw_cread(CAP_ENDPTPRIME, ~0),
			hw_cread(CAP_ENDPTSTAT, ~0),
			mEp->num, mEp->dir ? "IN" : "OUT");
done:
	return count;

}
static DEVICE_ATTR(prime, S_IWUSR, NULL, prime_ept);

static ssize_t print_dtds(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct ci13xxx *udc = container_of(dev, struct ci13xxx, gadget.dev);
	struct ci13xxx_ep *mEp;
	unsigned int ep_num, dir;
	int n;
	struct list_head   *ptr = NULL;
	struct ci13xxx_req *req = NULL;

	if (sscanf(buf, "%u %u", &ep_num, &dir) != 2) {
		dev_err(dev, "<ep_num> <dir>: to print dtds");
		goto done;
	}

	if (dir)
		mEp = &udc->ci13xxx_ep[ep_num + hw_ep_max/2];
	else
		mEp = &udc->ci13xxx_ep[ep_num];

	n = hw_ep_bit(mEp->num, mEp->dir);
	pr_info("%s: prime:%08x stat:%08x ep#%d dir:%s"
			"dTD_update_fail_count: %lu "
			"mEp->dTD_update_fail_count: %lu"
			"mEp->prime_fail_count: %lu\n", __func__,
			hw_cread(CAP_ENDPTPRIME, ~0),
			hw_cread(CAP_ENDPTSTAT, ~0),
			mEp->num, mEp->dir ? "IN" : "OUT",
			udc->dTD_update_fail_count,
			mEp->dTD_update_fail_count,
			mEp->prime_fail_count);

	pr_info("QH: cap:%08x cur:%08x next:%08x token:%08x\n",
			mEp->qh.ptr->cap, mEp->qh.ptr->curr,
			mEp->qh.ptr->td.next, mEp->qh.ptr->td.token);

	list_for_each(ptr, &mEp->qh.queue) {
		req = list_entry(ptr, struct ci13xxx_req, queue);

		pr_info("\treq:%08x next:%08x token:%08x page0:%08x status:%d\n",
				req->dma, req->ptr->next, req->ptr->token,
				req->ptr->page[0], req->req.status);
	}
done:
	return count;

}
static DEVICE_ATTR(dtds, S_IWUSR, NULL, print_dtds);

static int ci13xxx_wakeup(struct usb_gadget *_gadget)
{
	struct ci13xxx *udc = container_of(_gadget, struct ci13xxx, gadget);
	unsigned long flags;
	int ret = 0;

	trace();

	spin_lock_irqsave(udc->lock, flags);
	if (!udc->remote_wakeup) {
		ret = -EOPNOTSUPP;
		dbg_trace("remote wakeup feature is not enabled\n");
		goto out;
	}
	spin_unlock_irqrestore(udc->lock, flags);

	udc->udc_driver->notify_event(udc,
		CI13XXX_CONTROLLER_REMOTE_WAKEUP_EVENT);

	if (udc->transceiver)
		usb_phy_set_suspend(udc->transceiver, 0);

	spin_lock_irqsave(udc->lock, flags);
	if (!hw_cread(CAP_PORTSC, PORTSC_SUSP)) {
		ret = -EINVAL;
		dbg_trace("port is not suspended\n");
		goto out;
	}
	hw_cwrite(CAP_PORTSC, PORTSC_FPR, PORTSC_FPR);
out:
	spin_unlock_irqrestore(udc->lock, flags);
	return ret;
}

static void usb_do_remote_wakeup(struct work_struct *w)
{
	struct ci13xxx *udc = _udc;
	unsigned long flags;
	bool do_wake;

	spin_lock_irqsave(udc->lock, flags);
	do_wake = udc->suspended && udc->remote_wakeup;
	spin_unlock_irqrestore(udc->lock, flags);

	if (do_wake)
		ci13xxx_wakeup(&udc->gadget);
}

static ssize_t usb_remote_wakeup(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ci13xxx *udc = container_of(dev, struct ci13xxx, gadget.dev);

	ci13xxx_wakeup(&udc->gadget);

	return count;
}
static DEVICE_ATTR(wakeup, S_IWUSR, 0, usb_remote_wakeup);

__maybe_unused static int dbg_create_files(struct device *dev)
{
	int retval = 0;

	if (dev == NULL)
		return -EINVAL;
	retval = device_create_file(dev, &dev_attr_device);
	if (retval)
		goto done;
	retval = device_create_file(dev, &dev_attr_driver);
	if (retval)
		goto rm_device;
	retval = device_create_file(dev, &dev_attr_events);
	if (retval)
		goto rm_driver;
	retval = device_create_file(dev, &dev_attr_inters);
	if (retval)
		goto rm_events;
	retval = device_create_file(dev, &dev_attr_port_test);
	if (retval)
		goto rm_inters;
	retval = device_create_file(dev, &dev_attr_qheads);
	if (retval)
		goto rm_port_test;
	retval = device_create_file(dev, &dev_attr_registers);
	if (retval)
		goto rm_qheads;
	retval = device_create_file(dev, &dev_attr_requests);
	if (retval)
		goto rm_registers;
	retval = device_create_file(dev, &dev_attr_wakeup);
	if (retval)
		goto rm_remote_wakeup;
	retval = device_create_file(dev, &dev_attr_prime);
	if (retval)
		goto rm_prime;
	retval = device_create_file(dev, &dev_attr_dtds);
	if (retval)
		goto rm_dtds;

	return 0;

rm_dtds:
	device_remove_file(dev, &dev_attr_dtds);
rm_prime:
	device_remove_file(dev, &dev_attr_prime);
rm_remote_wakeup:
	device_remove_file(dev, &dev_attr_wakeup);
 rm_registers:
	device_remove_file(dev, &dev_attr_registers);
 rm_qheads:
	device_remove_file(dev, &dev_attr_qheads);
 rm_port_test:
	device_remove_file(dev, &dev_attr_port_test);
 rm_inters:
	device_remove_file(dev, &dev_attr_inters);
 rm_events:
	device_remove_file(dev, &dev_attr_events);
 rm_driver:
	device_remove_file(dev, &dev_attr_driver);
 rm_device:
	device_remove_file(dev, &dev_attr_device);
 done:
	return retval;
}

__maybe_unused static int dbg_remove_files(struct device *dev)
{
	if (dev == NULL)
		return -EINVAL;
	device_remove_file(dev, &dev_attr_requests);
	device_remove_file(dev, &dev_attr_registers);
	device_remove_file(dev, &dev_attr_qheads);
	device_remove_file(dev, &dev_attr_port_test);
	device_remove_file(dev, &dev_attr_inters);
	device_remove_file(dev, &dev_attr_events);
	device_remove_file(dev, &dev_attr_driver);
	device_remove_file(dev, &dev_attr_device);
	device_remove_file(dev, &dev_attr_wakeup);
	return 0;
}

static void dump_usb_info(void *ignore, unsigned int ebi_addr,
	unsigned int ebi_apacket0, unsigned int ebi_apacket1)
{
	struct ci13xxx *udc = _udc;
	unsigned long flags;
	struct list_head   *ptr = NULL;
	struct ci13xxx_req *req = NULL;
	struct ci13xxx_ep *mEp;
	unsigned i;
	struct ci13xxx_ebi_err_entry *temp_dump;
	static int count;
	u32 epdir = 0;

	if (count)
		return;
	count++;

	pr_info("%s: USB EBI error detected\n", __func__);

	ebi_err_data = kmalloc(sizeof(struct ci13xxx_ebi_err_data),
				 GFP_ATOMIC);
	if (!ebi_err_data) {
		pr_err("%s: memory alloc failed for ebi_err_data\n", __func__);
		return;
	}

	ebi_err_data->ebi_err_entry = kmalloc(
					sizeof(struct ci13xxx_ebi_err_entry),
					GFP_ATOMIC);
	if (!ebi_err_data->ebi_err_entry) {
		kfree(ebi_err_data);
		pr_err("%s: memory alloc failed for ebi_err_entry\n", __func__);
		return;
	}

	ebi_err_data->ebi_err_addr = ebi_addr;
	ebi_err_data->apkt0 = ebi_apacket0;
	ebi_err_data->apkt1 = ebi_apacket1;

	temp_dump = ebi_err_data->ebi_err_entry;
	pr_info("\n DUMPING USB Requests Information\n");
	spin_lock_irqsave(udc->lock, flags);
	for (i = 0; i < hw_ep_max; i++) {
		list_for_each(ptr, &udc->ci13xxx_ep[i].qh.queue) {
			mEp = &udc->ci13xxx_ep[i];
			req = list_entry(ptr, struct ci13xxx_req, queue);

			temp_dump->usb_req_buf = req->req.buf;
			temp_dump->usb_req_length = req->req.length;
			epdir = mEp->dir;
			temp_dump->ep_info = mEp->num | (epdir << 15);

			temp_dump->next = kmalloc(
					  sizeof(struct ci13xxx_ebi_err_entry),
					  GFP_ATOMIC);
			if (!temp_dump->next) {
				pr_err("%s: memory alloc failed\n", __func__);
				spin_unlock_irqrestore(udc->lock, flags);
				return;
			}
			temp_dump = temp_dump->next;
		}
	}
	spin_unlock_irqrestore(udc->lock, flags);
}

static inline u8 _usb_addr(struct ci13xxx_ep *ep)
{
	return ((ep->dir == TX) ? USB_ENDPOINT_DIR_MASK : 0) | ep->num;
}

static void usb_chg_stop(struct work_struct *w)
{
	USB_INFO("disable charger\n");
	htc_battery_pwrsrc_disable();
}

static void ep_prime_timer_func(unsigned long data)
{
	struct ci13xxx_ep *mep = (struct ci13xxx_ep *)data;
	struct ci13xxx_req *req;
	struct list_head *ptr = NULL;
	int n = hw_ep_bit(mep->num, mep->dir);
	unsigned long flags;


	spin_lock_irqsave(mep->lock, flags);

	if (_udc && (!_udc->vbus_active || _udc->suspended)) {
		pr_debug("ep%d%s prime timer when vbus_active=%d,suspend=%d\n",
			mep->num, mep->dir ? "IN" : "OUT",
			_udc->vbus_active, _udc->suspended);
		goto out;
	}

	if (!hw_cread(CAP_ENDPTPRIME, BIT(n)))
		goto out;

	if (list_empty(&mep->qh.queue))
		goto out;

	req = list_entry(mep->qh.queue.next, struct ci13xxx_req, queue);

	mb();
	if (!(TD_STATUS_ACTIVE & req->ptr->token))
		goto out;

	mep->prime_timer_count++;
	if (mep->prime_timer_count == MAX_PRIME_CHECK_RETRY) {
		mep->prime_timer_count = 0;
		pr_info("ep%d dir:%s QH:cap:%08x cur:%08x next:%08x tkn:%08x\n",
				mep->num, mep->dir ? "IN" : "OUT",
				mep->qh.ptr->cap, mep->qh.ptr->curr,
				mep->qh.ptr->td.next, mep->qh.ptr->td.token);
		list_for_each(ptr, &mep->qh.queue) {
			req = list_entry(ptr, struct ci13xxx_req, queue);
			pr_info("\treq:%08xnext:%08xtkn:%08xpage0:%08xsts:%d\n",
					req->dma, req->ptr->next,
					req->ptr->token, req->ptr->page[0],
					req->req.status);
		}
		dbg_usb_op_fail(0xFF, "PRIMEF", mep);
		mep->prime_fail_count++;
	} else {
		mod_timer(&mep->prime_timer, EP_PRIME_CHECK_DELAY);
	}

	spin_unlock_irqrestore(mep->lock, flags);
	return;

out:
	mep->prime_timer_count = 0;
	spin_unlock_irqrestore(mep->lock, flags);

}

static int _hardware_enqueue(struct ci13xxx_ep *mEp, struct ci13xxx_req *mReq)
{
	unsigned i;
	int ret = 0;
	unsigned length = mReq->req.length;
	struct ci13xxx *udc = _udc;

	trace("%p, %p", mEp, mReq);

	
	if (mReq->req.status == -EALREADY)
		return -EALREADY;

	mReq->req.status = -EALREADY;
	if (length && mReq->req.dma == DMA_ADDR_INVALID) {
		mReq->req.dma = \
			dma_map_single(mEp->device, mReq->req.buf,
				       length, mEp->dir ? DMA_TO_DEVICE :
				       DMA_FROM_DEVICE);
		if (mReq->req.dma == 0)
			return -ENOMEM;

		mReq->map = 1;
	}

	if (mReq->req.zero && length && (length % mEp->ep.maxpacket == 0)) {
		mReq->zptr = dma_pool_alloc(mEp->td_pool, GFP_ATOMIC,
					   &mReq->zdma);
		if (mReq->zptr == NULL) {
			if (mReq->map) {
				dma_unmap_single(mEp->device, mReq->req.dma,
					length, mEp->dir ? DMA_TO_DEVICE :
					DMA_FROM_DEVICE);
				mReq->req.dma = DMA_ADDR_INVALID;
				mReq->map     = 0;
			}
			return -ENOMEM;
		}
		memset(mReq->zptr, 0, sizeof(*mReq->zptr));
		mReq->zptr->next    = TD_TERMINATE;
		mReq->zptr->token   = TD_STATUS_ACTIVE;
		if (!mReq->req.no_interrupt)
			mReq->zptr->token   |= TD_IOC;
	}
	memset(mReq->ptr, 0, sizeof(*mReq->ptr));
	mReq->ptr->token    = length << ffs_nr(TD_TOTAL_BYTES);
	mReq->ptr->token   &= TD_TOTAL_BYTES;
	mReq->ptr->token   |= TD_STATUS_ACTIVE;
	if (mReq->zptr) {
		mReq->ptr->next    = mReq->zdma;
	} else {
		mReq->ptr->next    = TD_TERMINATE;
		if (!mReq->req.no_interrupt)
			mReq->ptr->token  |= TD_IOC;
	}

	if (CI13XX_REQ_VENDOR_ID(mReq->req.udc_priv) == MSM_VENDOR_ID) {
		if (mReq->req.udc_priv & MSM_SPS_MODE) {
			mReq->ptr->token = TD_STATUS_ACTIVE;
			if (mReq->req.udc_priv & MSM_IS_FINITE_TRANSFER)
				mReq->ptr->next = TD_TERMINATE;
			else
				mReq->ptr->next = MSM_ETD_TYPE | mReq->dma;
			if (!mReq->req.no_interrupt)
				mReq->ptr->token |= MSM_ETD_IOC;
		}
		mReq->req.dma = 0;
	}

	mReq->ptr->page[0]  = mReq->req.dma;
	for (i = 1; i < 5; i++)
		mReq->ptr->page[i] = (mReq->req.dma + i * CI13XXX_PAGE_SIZE) &
							~TD_RESERVED_MASK;
	wmb();

	
	if (udc->suspended) {
		if (!udc->remote_wakeup) {
			mReq->req.status = -EAGAIN;
			dev_dbg(mEp->device, "%s: queue failed (suspend) ept #%d\n",
				__func__, mEp->num);
			return -EAGAIN;
		}
		usb_phy_set_suspend(udc->transceiver, 0);
		schedule_delayed_work(&udc->rw_work, REMOTE_WAKEUP_DELAY);
	}

	if (!list_empty(&mEp->qh.queue)) {
		struct ci13xxx_req *mReqPrev;
		int n = hw_ep_bit(mEp->num, mEp->dir);
		int tmp_stat;
		ktime_t start, diff;

		mReqPrev = list_entry(mEp->qh.queue.prev,
				struct ci13xxx_req, queue);
		if (mReqPrev->zptr)
			mReqPrev->zptr->next = mReq->dma & TD_ADDR_MASK;
		else
			mReqPrev->ptr->next = mReq->dma & TD_ADDR_MASK;
		wmb();
		if (hw_cread(CAP_ENDPTPRIME, BIT(n)))
			goto done;
		start = ktime_get();
		do {
			hw_cwrite(CAP_USBCMD, USBCMD_ATDTW, USBCMD_ATDTW);
			tmp_stat = hw_cread(CAP_ENDPTSTAT, BIT(n));
			diff = ktime_sub(ktime_get(), start);
			
			if (ktime_to_ms(diff) > USB_MAX_TIMEOUT) {
				if (hw_cread(CAP_USBCMD, USBCMD_ATDTW))
					break;
				printk_ratelimited(KERN_ERR
				"%s:queue failed ep#%d %s\n",
				 __func__, mEp->num, mEp->dir ? "IN" : "OUT");
				return -EAGAIN;
			}
		} while (!hw_cread(CAP_USBCMD, USBCMD_ATDTW));
		hw_cwrite(CAP_USBCMD, USBCMD_ATDTW, 0);
		if (tmp_stat)
			goto done;
	}

	
	if (!list_empty(&mEp->qh.queue)) {
		struct ci13xxx_req *mReq = \
			list_entry(mEp->qh.queue.next,
				   struct ci13xxx_req, queue);

		if (TD_STATUS_ACTIVE & mReq->ptr->token) {
			mEp->qh.ptr->td.next   = mReq->dma;
			mEp->qh.ptr->td.token &= ~TD_STATUS;
			goto prime;
		}
	}

	mEp->qh.ptr->td.next   = mReq->dma;    

	if (CI13XX_REQ_VENDOR_ID(mReq->req.udc_priv) == MSM_VENDOR_ID) {
		if (mReq->req.udc_priv & MSM_SPS_MODE) {
			mEp->qh.ptr->td.next   |= MSM_ETD_TYPE;
			i = hw_cread(CAP_ENDPTPIPEID +
						 mEp->num * sizeof(u32), ~0);
			
			i = (mEp->dir == TX) ?
				((i >> MSM_TX_PIPE_ID_OFS) & MSM_PIPE_ID_MASK) :
					(i & MSM_PIPE_ID_MASK);
			if (i != (mReq->req.udc_priv & MSM_PIPE_ID_MASK)) {
				if (mEp->dir == TX)
					hw_cwrite(
						CAP_ENDPTPIPEID +
							mEp->num * sizeof(u32),
						MSM_PIPE_ID_MASK <<
							MSM_TX_PIPE_ID_OFS,
						(mReq->req.udc_priv &
						 MSM_PIPE_ID_MASK)
							<< MSM_TX_PIPE_ID_OFS);
				else
					hw_cwrite(
						CAP_ENDPTPIPEID +
							mEp->num * sizeof(u32),
						MSM_PIPE_ID_MASK,
						mReq->req.udc_priv &
							MSM_PIPE_ID_MASK);
			}
		}
	}

	mEp->qh.ptr->td.token &= ~TD_STATUS;   
	mEp->qh.ptr->cap |=  QH_ZLT;

prime:
	wmb();   

	ret = hw_ep_prime(mEp->num, mEp->dir,
			   mEp->type == USB_ENDPOINT_XFER_CONTROL);
	if (!ret)
		mod_timer(&mEp->prime_timer, EP_PRIME_CHECK_DELAY);
done:
	return ret;
}

static int _hardware_dequeue(struct ci13xxx_ep *mEp, struct ci13xxx_req *mReq)
{
	trace("%p, %p", mEp, mReq);

	if (mReq->req.status != -EALREADY)
		return -EINVAL;

	
	mb();

	if ((TD_STATUS_ACTIVE & mReq->ptr->token) != 0)
		return -EBUSY;

	if (CI13XX_REQ_VENDOR_ID(mReq->req.udc_priv) == MSM_VENDOR_ID)
		if ((mReq->req.udc_priv & MSM_SPS_MODE) &&
			(mReq->req.udc_priv & MSM_IS_FINITE_TRANSFER))
			return -EBUSY;
	if (mReq->zptr) {
		if ((TD_STATUS_ACTIVE & mReq->zptr->token) != 0)
			return -EBUSY;

		if (mEp->last_zptr)
			dma_pool_free(mEp->td_pool, mEp->last_zptr,
					mEp->last_zdma);
		mEp->last_zptr = mReq->zptr;
		mEp->last_zdma = mReq->zdma;

		mReq->zptr = NULL;
	}

	mReq->req.status = 0;

	if (mReq->map) {
		dma_unmap_single(mEp->device, mReq->req.dma, mReq->req.length,
				 mEp->dir ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		mReq->req.dma = DMA_ADDR_INVALID;
		mReq->map     = 0;
	}

	mReq->req.status = mReq->ptr->token & TD_STATUS;
	if ((TD_STATUS_HALTED & mReq->req.status) != 0) {
		USB_WARNING("%s: HALTED EP%d %s %6d\n", __func__, mEp->num,
			((mEp->dir == TX)? "I":"O"), mReq->req.length);
		mReq->req.status = -1;
	} else if ((TD_STATUS_DT_ERR & mReq->req.status) != 0) {
		USB_WARNING("%s: DT_ERR EP%d %s %6d\n", __func__, mEp->num,
			((mEp->dir == TX)? "I":"O"), mReq->req.length);
		mReq->req.status = -1;
	} else if ((TD_STATUS_TR_ERR & mReq->req.status) != 0) {
		USB_WARNING("%s: TR_ERR EP%d %s %6d\n", __func__, mEp->num,
			((mEp->dir == TX)? "I":"O"), mReq->req.length);
		mReq->req.status = -1;
	}

	mReq->req.actual   = mReq->ptr->token & TD_TOTAL_BYTES;
	mReq->req.actual >>= ffs_nr(TD_TOTAL_BYTES);
	mReq->req.actual   = mReq->req.length - mReq->req.actual;
	mReq->req.actual   = mReq->req.status ? 0 : mReq->req.actual;

	return mReq->req.actual;
}

static void restore_original_req(struct ci13xxx_req *mReq)
{
	mReq->req.buf = mReq->multi.buf;
	mReq->req.length = mReq->multi.len;
	if (!mReq->req.status)
		mReq->req.actual = mReq->multi.actual;

	mReq->multi.len = 0;
	mReq->multi.actual = 0;
	mReq->multi.buf = NULL;
}

static int _ep_nuke(struct ci13xxx_ep *mEp)
__releases(mEp->lock)
__acquires(mEp->lock)
{
	struct ci13xxx_ep *mEpTemp = mEp;
	unsigned val;

	trace("%p", mEp);

	if (mEp == NULL)
		return -EINVAL;

	del_timer(&mEp->prime_timer);
	mEp->prime_timer_count = 0;

	hw_ep_flush(mEp->num, mEp->dir);

	while (!list_empty(&mEp->qh.queue)) {

		
		struct ci13xxx_req *mReq = \
			list_entry(mEp->qh.queue.next,
				   struct ci13xxx_req, queue);
		list_del_init(&mReq->queue);

		
		if (CI13XX_REQ_VENDOR_ID(mReq->req.udc_priv) == MSM_VENDOR_ID) {
			if (mReq->req.udc_priv & MSM_SPS_MODE) {
				val = hw_cread(CAP_ENDPTPIPEID +
					mEp->num * sizeof(u32),
					~0);

				if (val != MSM_EP_PIPE_ID_RESET_VAL)
					hw_cwrite(
						CAP_ENDPTPIPEID +
						 mEp->num * sizeof(u32),
						~0, MSM_EP_PIPE_ID_RESET_VAL);
			}
		}
		mReq->req.status = -ESHUTDOWN;

		if (mReq->map) {
			dma_unmap_single(mEp->device, mReq->req.dma,
				mReq->req.length,
				mEp->dir ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
			mReq->req.dma = DMA_ADDR_INVALID;
			mReq->map     = 0;
		}

		if (mEp->multi_req) {
			restore_original_req(mReq);
			mEp->multi_req = false;
		}

		if (mReq->req.complete != NULL) {
			spin_unlock(mEp->lock);
			if ((mEp->type == USB_ENDPOINT_XFER_CONTROL) &&
				mReq->req.length)
				mEpTemp = &_udc->ep0in;
			mReq->req.complete(&mEpTemp->ep, &mReq->req);
			if (mEp->type == USB_ENDPOINT_XFER_CONTROL)
				mReq->req.complete = NULL;
			spin_lock(mEp->lock);
		}
	}
	return 0;
}

static int _gadget_stop_activity(struct usb_gadget *gadget, int mute)
{
	struct ci13xxx    *udc = container_of(gadget, struct ci13xxx, gadget);
	unsigned long flags;

	trace("%p", gadget);

	if (gadget == NULL)
		return -EINVAL;

	spin_lock_irqsave(udc->lock, flags);
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->remote_wakeup = 0;
	udc->suspended = 0;
	udc->configured = 0;
	spin_unlock_irqrestore(udc->lock, flags);

	gadget->xfer_isr_count = 0;
	gadget->b_hnp_enable = 0;
	gadget->a_hnp_support = 0;
	gadget->host_request = 0;
	gadget->otg_srp_reqd = 0;

	if (mute)
		udc->driver->mute_disconnect(gadget);
	else
		udc->driver->disconnect(gadget);

	spin_lock_irqsave(udc->lock, flags);
	_ep_nuke(&udc->ep0out);
	_ep_nuke(&udc->ep0in);
	spin_unlock_irqrestore(udc->lock, flags);

	if (udc->ep0in.last_zptr) {
		dma_pool_free(udc->ep0in.td_pool, udc->ep0in.last_zptr,
				udc->ep0in.last_zdma);
		udc->ep0in.last_zptr = NULL;
	}

	return 0;
}

static void isr_reset_handler(struct ci13xxx *udc)
__releases(udc->lock)
__acquires(udc->lock)
{
	int retval;

	trace("%p", udc);

	if (udc == NULL) {
		err("EINVAL");
		return;
	}

	dbg_event(0xFF, "BUS RST", 0);

	spin_unlock(udc->lock);

	if (udc->suspended) {
		if (udc->udc_driver->notify_event)
			udc->udc_driver->notify_event(udc,
			CI13XXX_CONTROLLER_RESUME_EVENT);
		if (udc->transceiver)
			usb_phy_set_suspend(udc->transceiver, 0);
		udc->driver->resume(&udc->gadget);
		udc->suspended = 0;
	}

	
	if (udc->transceiver)
		usb_phy_set_power(udc->transceiver, 100);

	retval = _gadget_stop_activity(&udc->gadget, 1);
	if (retval)
		goto done;

	_udc->skip_flush = false;
	retval = hw_usb_reset();
	if (retval)
		goto done;

	spin_lock(udc->lock);

 done:
	if (retval)
		err("error: %i", retval);
}

static void isr_resume_handler(struct ci13xxx *udc)
{
	udc->gadget.speed = hw_port_is_high_speed() ?
		USB_SPEED_HIGH : USB_SPEED_FULL;

	if (udc->gadget.speed == USB_SPEED_HIGH)
		USB_INFO("portchange USB_SPEED_HIGH\n");
	else
		USB_INFO("portchange USB_SPEED_FULL\n");

	if (udc->suspended) {
		spin_unlock(udc->lock);
		if (udc->udc_driver->notify_event)
			udc->udc_driver->notify_event(udc,
			  CI13XXX_CONTROLLER_RESUME_EVENT);
		if (udc->transceiver)
			usb_phy_set_suspend(udc->transceiver, 0);
		udc->driver->resume(&udc->gadget);
		spin_lock(udc->lock);
		udc->suspended = 0;
	}
}

static void isr_suspend_handler(struct ci13xxx *udc)
{
	if (udc->gadget.speed != USB_SPEED_UNKNOWN &&
		udc->vbus_active) {
		if (udc->suspended == 0) {
			spin_unlock(udc->lock);
			udc->driver->suspend(&udc->gadget);
			if (udc->udc_driver->notify_event)
				udc->udc_driver->notify_event(udc,
				CI13XXX_CONTROLLER_SUSPEND_EVENT);
			if (udc->transceiver)
				usb_phy_set_suspend(udc->transceiver, 1);
			spin_lock(udc->lock);
			udc->suspended = 1;
		}
	}
}

static void isr_get_status_complete(struct usb_ep *ep, struct usb_request *req)
{
	trace("%p, %p", ep, req);

	if (ep == NULL || req == NULL) {
		err("EINVAL");
		return;
	}

	if (req->status)
		err("GET_STATUS failed");
}

static int isr_get_status_response(struct ci13xxx *udc,
				   struct usb_ctrlrequest *setup)
__releases(mEp->lock)
__acquires(mEp->lock)
{
	struct ci13xxx_ep *mEp = &udc->ep0in;
	struct usb_request *req = udc->status;
	int dir, num, retval;

	trace("%p, %p", mEp, setup);

	if (mEp == NULL || setup == NULL)
		return -EINVAL;

	req->complete = isr_get_status_complete;
	req->length   = 2;
	req->buf      = udc->status_buf;

	if ((setup->bRequestType & USB_RECIP_MASK) == USB_RECIP_DEVICE) {
		if (setup->wIndex == OTG_STATUS_SELECTOR) {
			*((u8 *)req->buf) = _udc->gadget.host_request <<
						HOST_REQUEST_FLAG;
			req->length = 1;
		} else {
			
			*((u16 *)req->buf) = _udc->remote_wakeup << 1;
		}
		
		retval = 0;
	} else if ((setup->bRequestType & USB_RECIP_MASK) \
		   == USB_RECIP_ENDPOINT) {
		dir = (le16_to_cpu(setup->wIndex) & USB_ENDPOINT_DIR_MASK) ?
			TX : RX;
		num =  le16_to_cpu(setup->wIndex) & USB_ENDPOINT_NUMBER_MASK;
		*((u16 *)req->buf) = hw_ep_get_halt(num, dir);
	}
	

	spin_unlock(mEp->lock);
	retval = usb_ep_queue(&mEp->ep, req, GFP_ATOMIC);
	spin_lock(mEp->lock);
	return retval;
}

static void
isr_setup_status_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct ci13xxx *udc = req->context;
	unsigned long flags;

	trace("%p, %p", ep, req);

	spin_lock_irqsave(udc->lock, flags);
	if (udc->test_mode)
		hw_port_test_set(udc->test_mode);
	spin_unlock_irqrestore(udc->lock, flags);
}

static int isr_setup_status_phase(struct ci13xxx *udc)
__releases(mEp->lock)
__acquires(mEp->lock)
{
	int retval;
	struct ci13xxx_ep *mEp;

	trace("%p", udc);

	mEp = (udc->ep0_dir == TX) ? &udc->ep0out : &udc->ep0in;
	udc->status->context = udc;
	udc->status->complete = isr_setup_status_complete;
	udc->status->length = 0;

	spin_unlock(mEp->lock);
	retval = usb_ep_queue(&mEp->ep, udc->status, GFP_ATOMIC);
	spin_lock(mEp->lock);

	return retval;
}

static int isr_tr_complete_low(struct ci13xxx_ep *mEp)
__releases(mEp->lock)
__acquires(mEp->lock)
{
	struct ci13xxx_req *mReq, *mReqTemp;
	struct ci13xxx_ep *mEpTemp = mEp;
	int uninitialized_var(retval);
	int req_dequeue = 1;
	struct ci13xxx *udc = _udc;

	trace("%p", mEp);

	if (list_empty(&mEp->qh.queue))
		return 0;

	del_timer(&mEp->prime_timer);
	mEp->prime_timer_count = 0;
	list_for_each_entry_safe(mReq, mReqTemp, &mEp->qh.queue,
			queue) {
dequeue:
		retval = _hardware_dequeue(mEp, mReq);
		if (retval < 0) {
			if (retval == -EBUSY && req_dequeue &&
				(mEp->dir == 0 || mEp->num == 0)) {
				req_dequeue = 0;
				udc->dTD_update_fail_count++;
				mEp->dTD_update_fail_count++;
				udelay(10);
				goto dequeue;
			}
			break;
		}
		req_dequeue = 0;

		if (mEp->multi_req) { 
			unsigned remain_len;

			mReq->multi.actual += mReq->req.actual;
			remain_len = mReq->multi.len - mReq->multi.actual;
			if (mReq->req.status || !remain_len ||
				(mReq->req.actual != mReq->req.length)) {
				restore_original_req(mReq);
				mEp->multi_req = false;
			} else {
				mReq->req.buf = mReq->multi.buf +
						mReq->multi.actual;
				mReq->req.length = min_t(unsigned, remain_len,
						(4 * CI13XXX_PAGE_SIZE));

				mReq->req.status = -EINPROGRESS;
				mReq->req.actual = 0;
				list_del_init(&mReq->queue);
				retval = _hardware_enqueue(mEp, mReq);
				if (retval) {
					err("Large req failed in middle");
					mReq->req.status = retval;
					restore_original_req(mReq);
					mEp->multi_req = false;
					goto done;
				} else {
					list_add_tail(&mReq->queue,
						&mEp->qh.queue);
					return 0;
				}
			}
		}
		list_del_init(&mReq->queue);
done:

		dbg_done(_usb_addr(mEp), mReq->ptr->token, retval);

		if (mReq->req.complete != NULL) {
			spin_unlock(mEp->lock);
			if ((mEp->type == USB_ENDPOINT_XFER_CONTROL) &&
					mReq->req.length)
				mEpTemp = &_udc->ep0in;
			mReq->req.complete(&mEpTemp->ep, &mReq->req);
			spin_lock(mEp->lock);
		}
	}

	if (retval == -EBUSY)
		retval = 0;
	if (retval < 0)
		dbg_event(_usb_addr(mEp), "DONE", retval);

	return retval;
}

static void isr_tr_complete_handler(struct ci13xxx *udc)
__releases(udc->lock)
__acquires(udc->lock)
{
	unsigned i;
	u8 tmode = 0;

	trace("%p", udc);

	if (udc == NULL) {
		err("EINVAL");
		return;
	}

	for (i = 0; i < hw_ep_max; i++) {
		struct ci13xxx_ep *mEp  = &udc->ci13xxx_ep[i];
		int type, num, dir, err = -EINVAL;
		struct usb_ctrlrequest req;

		if (mEp->desc == NULL)
			continue;   

		if (hw_test_and_clear_complete(i)) {
			err = isr_tr_complete_low(mEp);
			if (mEp->type == USB_ENDPOINT_XFER_CONTROL) {
				if (err > 0)   
					err = isr_setup_status_phase(udc);
				if (err < 0) {
					dbg_event(_usb_addr(mEp),
						  "ERROR", err);
					spin_unlock(udc->lock);
					if (usb_ep_set_halt(&mEp->ep))
						err("error: ep_set_halt");
					spin_lock(udc->lock);
				}
			}
		}

		if (mEp->type != USB_ENDPOINT_XFER_CONTROL ||
		    !hw_test_and_clear_setup_status(i))
			continue;

		if (i != 0) {
			warn("ctrl traffic received at endpoint");
			continue;
		}

		_ep_nuke(&udc->ep0out);
		_ep_nuke(&udc->ep0in);

		
		do {
			hw_test_and_set_setup_guard();
			memcpy(&req, &mEp->qh.ptr->setup, sizeof(req));
			
			mb();
		} while (!hw_test_and_clear_setup_guard());

		type = req.bRequestType;

		udc->ep0_dir = (type & USB_DIR_IN) ? TX : RX;

		dbg_setup(_usb_addr(mEp), &req);

		switch (req.bRequest) {
		case USB_REQ_CLEAR_FEATURE:
			if (type == (USB_DIR_OUT|USB_RECIP_ENDPOINT) &&
					le16_to_cpu(req.wValue) ==
					USB_ENDPOINT_HALT) {
				if (req.wLength != 0)
					break;
				num  = le16_to_cpu(req.wIndex);
				dir = num & USB_ENDPOINT_DIR_MASK;
				num &= USB_ENDPOINT_NUMBER_MASK;
				if (dir) 
					num += hw_ep_max/2;
				if (!udc->ci13xxx_ep[num].wedge) {
					spin_unlock(udc->lock);
					err = usb_ep_clear_halt(
						&udc->ci13xxx_ep[num].ep);
					spin_lock(udc->lock);
					if (err)
						break;
				}
				err = isr_setup_status_phase(udc);
			} else if (type == (USB_DIR_OUT|USB_RECIP_DEVICE) &&
					le16_to_cpu(req.wValue) ==
					USB_DEVICE_REMOTE_WAKEUP) {
				if (req.wLength != 0)
					break;
				udc->remote_wakeup = 0;
				err = isr_setup_status_phase(udc);
			} else {
				goto delegate;
			}
			break;
		case USB_REQ_GET_STATUS:
			if (type != (USB_DIR_IN|USB_RECIP_DEVICE)   &&
			    type != (USB_DIR_IN|USB_RECIP_ENDPOINT) &&
			    type != (USB_DIR_IN|USB_RECIP_INTERFACE))
				goto delegate;
			if (le16_to_cpu(req.wValue)  != 0)
				break;
			err = isr_get_status_response(udc, &req);
			break;
		case USB_REQ_SET_ADDRESS:
			if (type != (USB_DIR_OUT|USB_RECIP_DEVICE))
				goto delegate;
			if (le16_to_cpu(req.wLength) != 0 ||
			    le16_to_cpu(req.wIndex)  != 0)
				break;
			err = hw_usb_set_address((u8)le16_to_cpu(req.wValue));
			if (err)
				break;
			err = isr_setup_status_phase(udc);
			break;
		case USB_REQ_SET_CONFIGURATION:
			if (type == (USB_DIR_OUT|USB_TYPE_STANDARD))
				udc->configured = !!req.wValue;
			goto delegate;
		case USB_REQ_SET_FEATURE:
			if (type == (USB_DIR_OUT|USB_RECIP_ENDPOINT) &&
					le16_to_cpu(req.wValue) ==
					USB_ENDPOINT_HALT) {
				if (req.wLength != 0)
					break;
				num  = le16_to_cpu(req.wIndex);
				dir = num & USB_ENDPOINT_DIR_MASK;
				num &= USB_ENDPOINT_NUMBER_MASK;
				if (dir) 
					num += hw_ep_max/2;

				spin_unlock(udc->lock);
				err = usb_ep_set_halt(&udc->ci13xxx_ep[num].ep);
				spin_lock(udc->lock);
				if (!err)
					isr_setup_status_phase(udc);
			} else if (type == (USB_DIR_OUT|USB_RECIP_DEVICE)) {
				if (req.wLength != 0)
					break;
				switch (le16_to_cpu(req.wValue)) {
				case USB_DEVICE_REMOTE_WAKEUP:
					udc->remote_wakeup = 1;
					err = isr_setup_status_phase(udc);
					break;
				case USB_DEVICE_B_HNP_ENABLE:
					udc->gadget.b_hnp_enable = 1;
					err = isr_setup_status_phase(udc);
					break;
				case USB_DEVICE_A_HNP_SUPPORT:
					udc->gadget.a_hnp_support = 1;
					err = isr_setup_status_phase(udc);
					break;
				case USB_DEVICE_A_ALT_HNP_SUPPORT:
					break;
				case USB_DEVICE_TEST_MODE:
					tmode = le16_to_cpu(req.wIndex) >> 8;
					switch (tmode) {
					case TEST_J:
					case TEST_K:
					case TEST_SE0_NAK:
					case TEST_PACKET:
						if (!udc->test_mode)
							schedule_delayed_work(&udc->chg_stop, 0);
					case TEST_FORCE_EN:
						udc->test_mode = tmode;
						err = isr_setup_status_phase(
								udc);
						break;
					case TEST_OTG_SRP_REQD:
						udc->gadget.otg_srp_reqd = 1;
						err = isr_setup_status_phase(
								udc);
						break;
					case TEST_OTG_HNP_REQD:
						udc->gadget.host_request = 1;
						err = isr_setup_status_phase(
								udc);
						break;
					default:
						break;
					}
				default:
					break;
				}
			} else {
				goto delegate;
			}
			break;
		default:
delegate:
			if (req.wLength == 0)   
				udc->ep0_dir = TX;

			spin_unlock(udc->lock);
			err = udc->driver->setup(&udc->gadget, &req);
			spin_lock(udc->lock);
			break;
		}

		if (err < 0) {
			dbg_event(_usb_addr(mEp), "ERROR", err);

			spin_unlock(udc->lock);
			if (usb_ep_set_halt(&mEp->ep))
				err("error: ep_set_halt");
			spin_lock(udc->lock);
		}
	}
}

static int ep_enable(struct usb_ep *ep,
		     const struct usb_endpoint_descriptor *desc)
{
	struct ci13xxx_ep *mEp = container_of(ep, struct ci13xxx_ep, ep);
	int retval = 0;
	unsigned long flags;
	unsigned mult = 0;

	trace("ep = %p, desc = %p", ep, desc);

	if (ep == NULL || desc == NULL)
		return -EINVAL;

	spin_lock_irqsave(mEp->lock, flags);

	

	mEp->desc = desc;

	if (!list_empty(&mEp->qh.queue))
		warn("enabling a non-empty endpoint!");

	mEp->dir  = usb_endpoint_dir_in(desc) ? TX : RX;
	mEp->num  = usb_endpoint_num(desc);
	mEp->type = usb_endpoint_type(desc);

	mEp->ep.maxpacket = usb_endpoint_maxp(desc);

	dbg_event(_usb_addr(mEp), "ENABLE", 0);

	mEp->qh.ptr->cap = 0;

	if (mEp->type == USB_ENDPOINT_XFER_CONTROL) {
		mEp->qh.ptr->cap |=  QH_IOS;
	} else if (mEp->type == USB_ENDPOINT_XFER_ISOC) {
		mEp->qh.ptr->cap &= ~QH_MULT;
		mult = ((mEp->ep.maxpacket >> QH_MULT_SHIFT) + 1) & 0x03;
		mEp->qh.ptr->cap |= (mult << ffs_nr(QH_MULT));
	} else {
		mEp->qh.ptr->cap |= QH_ZLT;
	}

	mEp->qh.ptr->cap |=
		(mEp->ep.maxpacket << ffs_nr(QH_MAX_PKT)) & QH_MAX_PKT;
	mEp->qh.ptr->td.next |= TD_TERMINATE;   

	
	mb();

	if (mEp->num)
		retval |= hw_ep_enable(mEp->num, mEp->dir, mEp->type);

	spin_unlock_irqrestore(mEp->lock, flags);
	return retval;
}

static int ep_disable(struct usb_ep *ep)
{
	struct ci13xxx_ep *mEp = container_of(ep, struct ci13xxx_ep, ep);
	int direction, retval = 0;
	unsigned long flags;

	trace("%p", ep);

	if (ep == NULL)
		return -EINVAL;
	else if (mEp->desc == NULL)
		return -EBUSY;

	spin_lock_irqsave(mEp->lock, flags);

	

	direction = mEp->dir;
	do {
		dbg_event(_usb_addr(mEp), "DISABLE", 0);

		retval |= _ep_nuke(mEp);
		retval |= hw_ep_disable(mEp->num, mEp->dir);

		if (mEp->type == USB_ENDPOINT_XFER_CONTROL)
			mEp->dir = (mEp->dir == TX) ? RX : TX;

	} while (mEp->dir != direction);

	if (mEp->last_zptr) {
		dma_pool_free(mEp->td_pool, mEp->last_zptr,
				mEp->last_zdma);
		mEp->last_zptr = NULL;
	}

	mEp->desc = NULL;
	mEp->ep.desc = NULL;
	mEp->ep.maxpacket = USHRT_MAX;

	spin_unlock_irqrestore(mEp->lock, flags);
	return retval;
}

static struct usb_request *ep_alloc_request(struct usb_ep *ep, gfp_t gfp_flags)
{
	struct ci13xxx_ep  *mEp  = container_of(ep, struct ci13xxx_ep, ep);
	struct ci13xxx_req *mReq = NULL;

	trace("%p, %i", ep, gfp_flags);

	if (ep == NULL) {
		err("EINVAL");
		return NULL;
	}

	mReq = kzalloc(sizeof(struct ci13xxx_req), gfp_flags);
	if (mReq != NULL) {
		INIT_LIST_HEAD(&mReq->queue);
		mReq->req.dma = DMA_ADDR_INVALID;

		mReq->ptr = dma_pool_alloc(mEp->td_pool, gfp_flags,
					   &mReq->dma);
		if (mReq->ptr == NULL) {
			kfree(mReq);
			mReq = NULL;
		}
	}

	dbg_event(_usb_addr(mEp), "ALLOC", mReq == NULL);

	return (mReq == NULL) ? NULL : &mReq->req;
}

static void ep_free_request(struct usb_ep *ep, struct usb_request *req)
{
	struct ci13xxx_ep  *mEp  = container_of(ep,  struct ci13xxx_ep, ep);
	struct ci13xxx_req *mReq = container_of(req, struct ci13xxx_req, req);
	unsigned long flags;

	trace("%p, %p", ep, req);

	if (ep == NULL || req == NULL) {
		err("EINVAL");
		return;
	} else if (!list_empty(&mReq->queue)) {
		err("EBUSY");
		return;
	}

	spin_lock_irqsave(mEp->lock, flags);

	if (mReq->ptr)
		dma_pool_free(mEp->td_pool, mReq->ptr, mReq->dma);
	kfree(mReq);

	dbg_event(_usb_addr(mEp), "FREE", 0);

	spin_unlock_irqrestore(mEp->lock, flags);
}

static int ep_queue(struct usb_ep *ep, struct usb_request *req,
		    gfp_t __maybe_unused gfp_flags)
{
	struct ci13xxx_ep  *mEp  = container_of(ep,  struct ci13xxx_ep, ep);
	struct ci13xxx_req *mReq = container_of(req, struct ci13xxx_req, req);
	int retval = 0;
	unsigned long flags;
	struct ci13xxx *udc = _udc;

	trace("%p, %p, %X", ep, req, gfp_flags);

	spin_lock_irqsave(mEp->lock, flags);
	if (ep == NULL || req == NULL || mEp->desc == NULL) {
		retval = -EINVAL;
		goto done;
	}

	if (!udc->softconnect) {
		retval = -ENODEV;
		goto done;
	}

	if (!udc->configured && mEp->type !=
		USB_ENDPOINT_XFER_CONTROL) {
		trace("usb is not configured"
			"ept #%d, ept name#%s\n",
			mEp->num, mEp->ep.name);
		retval = -ESHUTDOWN;
		goto done;
	}

	if (mEp->type == USB_ENDPOINT_XFER_CONTROL) {
		if (req->length)
			mEp = (_udc->ep0_dir == RX) ?
				&_udc->ep0out : &_udc->ep0in;
		if (!list_empty(&mEp->qh.queue)) {
			_ep_nuke(mEp);
			retval = -EOVERFLOW;
			warn("endpoint ctrl %X nuked", _usb_addr(mEp));
		}
	}

	
	if (!list_empty(&mReq->queue)) {
		retval = -EBUSY;
		err("request already in queue");
		goto done;
	}
	if (mEp->multi_req) {
		retval = -EAGAIN;
		err("Large request is in progress. come again");
		goto done;
	}

	if (req->length > (4 * CI13XXX_PAGE_SIZE)) {
		if (!list_empty(&mEp->qh.queue)) {
			retval = -EAGAIN;
			err("Queue is busy. Large req is not allowed");
			goto done;
		}
		if ((mEp->type != USB_ENDPOINT_XFER_BULK) ||
				(mEp->dir != RX)) {
			retval = -EINVAL;
			err("Larger req is supported only for Bulk OUT");
			goto done;
		}
		mEp->multi_req = true;
		mReq->multi.len = req->length;
		mReq->multi.buf = req->buf;
		req->length = (4 * CI13XXX_PAGE_SIZE);
	}

	dbg_queue(_usb_addr(mEp), req, retval);

	
	mReq->req.status = -EINPROGRESS;
	mReq->req.actual = 0;

	retval = _hardware_enqueue(mEp, mReq);

	if (retval == -EALREADY) {
		dbg_event(_usb_addr(mEp), "QUEUE", retval);
		retval = 0;
	}
	if (!retval)
		list_add_tail(&mReq->queue, &mEp->qh.queue);
	else if (mEp->multi_req)
		mEp->multi_req = false;

 done:
	spin_unlock_irqrestore(mEp->lock, flags);
	return retval;
}

static int ep_dequeue(struct usb_ep *ep, struct usb_request *req)
{
	struct ci13xxx_ep  *mEp  = container_of(ep,  struct ci13xxx_ep, ep);
	struct ci13xxx_ep *mEpTemp = mEp;
	struct ci13xxx_req *mReq = container_of(req, struct ci13xxx_req, req);
	unsigned long flags;

	trace("%p, %p", ep, req);

	spin_lock_irqsave(mEp->lock, flags);
	if (ep == NULL || req == NULL || mReq->req.status != -EALREADY ||
		mEp->desc == NULL || list_empty(&mReq->queue) ||
		(list_empty(&mEp->qh.queue) && ((mEp->type !=
			USB_ENDPOINT_XFER_CONTROL) ||
			list_empty(&_udc->ep0out.qh.queue)))) {
		spin_unlock_irqrestore(mEp->lock, flags);
		return -EINVAL;
	}

	dbg_event(_usb_addr(mEp), "DEQUEUE", 0);

	if ((mEp->type == USB_ENDPOINT_XFER_CONTROL)) {
		hw_ep_flush(_udc->ep0out.num, RX);
		hw_ep_flush(_udc->ep0in.num, TX);
	} else {
		hw_ep_flush(mEp->num, mEp->dir);
	}

	
	list_del_init(&mReq->queue);
	if (mReq->map) {
		dma_unmap_single(mEp->device, mReq->req.dma, mReq->req.length,
				 mEp->dir ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		mReq->req.dma = DMA_ADDR_INVALID;
		mReq->map     = 0;
	}
	req->status = -ECONNRESET;
	if (mEp->multi_req) {
		restore_original_req(mReq);
		mEp->multi_req = false;
	}

	if (mReq->req.complete != NULL) {
		spin_unlock(mEp->lock);
		if ((mEp->type == USB_ENDPOINT_XFER_CONTROL) &&
				mReq->req.length)
			mEpTemp = &_udc->ep0in;
		mReq->req.complete(&mEpTemp->ep, &mReq->req);
		if (mEp->type == USB_ENDPOINT_XFER_CONTROL)
			mReq->req.complete = NULL;
		spin_lock(mEp->lock);
	}

	spin_unlock_irqrestore(mEp->lock, flags);
	return 0;
}

static int is_sps_req(struct ci13xxx_req *mReq)
{
	return (CI13XX_REQ_VENDOR_ID(mReq->req.udc_priv) == MSM_VENDOR_ID &&
			mReq->req.udc_priv & MSM_SPS_MODE);
}

static int ep_set_halt(struct usb_ep *ep, int value)
{
	struct ci13xxx_ep *mEp = container_of(ep, struct ci13xxx_ep, ep);
	int direction, retval = 0;
	unsigned long flags;

	trace("%p, %i", ep, value);

	if (ep == NULL || mEp->desc == NULL)
		return -EINVAL;

	spin_lock_irqsave(mEp->lock, flags);

#ifndef STALL_IN
	
	if (value && mEp->type == USB_ENDPOINT_XFER_BULK && mEp->dir == TX &&
		!list_empty(&mEp->qh.queue) &&
		!is_sps_req(list_entry(mEp->qh.queue.next, struct ci13xxx_req,
							   queue))){
		spin_unlock_irqrestore(mEp->lock, flags);
		return -EAGAIN;
	}
#endif

	direction = mEp->dir;
	do {
		dbg_event(_usb_addr(mEp), "HALT", value);
		retval |= hw_ep_set_halt(mEp->num, mEp->dir, value);

		if (!value)
			mEp->wedge = 0;

		if (mEp->type == USB_ENDPOINT_XFER_CONTROL)
			mEp->dir = (mEp->dir == TX) ? RX : TX;

	} while (mEp->dir != direction);

	spin_unlock_irqrestore(mEp->lock, flags);
	return retval;
}

static int ep_set_wedge(struct usb_ep *ep)
{
	struct ci13xxx_ep *mEp = container_of(ep, struct ci13xxx_ep, ep);
	unsigned long flags;

	trace("%p", ep);

	if (ep == NULL || mEp->desc == NULL)
		return -EINVAL;

	spin_lock_irqsave(mEp->lock, flags);

	dbg_event(_usb_addr(mEp), "WEDGE", 0);
	mEp->wedge = 1;

	spin_unlock_irqrestore(mEp->lock, flags);

	return usb_ep_set_halt(ep);
}

static void ep_fifo_flush(struct usb_ep *ep)
{
	struct ci13xxx_ep *mEp = container_of(ep, struct ci13xxx_ep, ep);
	unsigned long flags;

	trace("%p", ep);

	if (ep == NULL) {
		err("%02X: -EINVAL", _usb_addr(mEp));
		return;
	}

	spin_lock_irqsave(mEp->lock, flags);

	dbg_event(_usb_addr(mEp), "FFLUSH", 0);
	_ep_nuke(mEp);

	spin_unlock_irqrestore(mEp->lock, flags);
}

static const struct usb_ep_ops usb_ep_ops = {
	.enable	       = ep_enable,
	.disable       = ep_disable,
	.alloc_request = ep_alloc_request,
	.free_request  = ep_free_request,
	.queue	       = ep_queue,
	.dequeue       = ep_dequeue,
	.set_halt      = ep_set_halt,
	.set_wedge     = ep_set_wedge,
	.fifo_flush    = ep_fifo_flush,
};

static int ci13xxx_vbus_session(struct usb_gadget *_gadget, int is_active)
{
	struct ci13xxx *udc = container_of(_gadget, struct ci13xxx, gadget);
	unsigned long flags;
	int gadget_ready = 0;

	if (!(udc->udc_driver->flags & CI13XXX_PULLUP_ON_VBUS))
		return -EOPNOTSUPP;

	spin_lock_irqsave(udc->lock, flags);
	udc->vbus_active = is_active;
	 
	_gadget->ats_reset_irq_count = 0;
	if (udc->driver)
		gadget_ready = 1;
	spin_unlock_irqrestore(udc->lock, flags);

	if (gadget_ready) {
		if (is_active) {
			pm_runtime_get_sync(&_gadget->dev);
			hw_device_reset(udc);
			if (udc->softconnect)
				hw_device_state(udc->ep0out.qh.dma);
		} else {
			hw_device_state(0);
			_gadget_stop_activity(&udc->gadget, 0);
			if (udc->udc_driver->notify_event)
				udc->udc_driver->notify_event(udc,
					CI13XXX_CONTROLLER_DISCONNECT_EVENT);
			pm_runtime_put_sync(&_gadget->dev);
		}
	}

	return 0;
}

static int ci13xxx_vbus_draw(struct usb_gadget *_gadget, unsigned mA)
{
	struct ci13xxx *udc = container_of(_gadget, struct ci13xxx, gadget);

	if (udc->transceiver)
		return usb_phy_set_power(udc->transceiver, mA);
	return -ENOTSUPP;
}

static int ci13xxx_pullup(struct usb_gadget *_gadget, int is_active)
{
	struct ci13xxx *udc = container_of(_gadget, struct ci13xxx, gadget);
	unsigned long flags;

	spin_lock_irqsave(udc->lock, flags);
	 
	_gadget->ats_reset_irq_count = 0;
	udc->softconnect = is_active;
	if (((udc->udc_driver->flags & CI13XXX_PULLUP_ON_VBUS) &&
			!udc->vbus_active) || !udc->driver) {
		spin_unlock_irqrestore(udc->lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(udc->lock, flags);

	if (is_active)
		hw_device_state(udc->ep0out.qh.dma);
	else {
		hw_device_state(0);
		_gadget_stop_activity(&udc->gadget, 1);
	}

	return 0;
}

static int ci13xxx_start(struct usb_gadget_driver *driver,
		int (*bind)(struct usb_gadget *));
static int ci13xxx_stop(struct usb_gadget_driver *driver);

static const struct usb_gadget_ops usb_gadget_ops = {
	.vbus_session	= ci13xxx_vbus_session,
	.wakeup		= ci13xxx_wakeup,
	.vbus_draw	= ci13xxx_vbus_draw,
	.pullup		= ci13xxx_pullup,
	.start		= ci13xxx_start,
	.stop		= ci13xxx_stop,
};

static int ci13xxx_start(struct usb_gadget_driver *driver,
		int (*bind)(struct usb_gadget *))
{
	struct ci13xxx *udc = _udc;
	unsigned long flags;
	int i, j;
	int retval = -ENOMEM;
	bool put = false;

	trace("%p", driver);

	if (driver             == NULL ||
	    bind               == NULL ||
	    driver->setup      == NULL ||
	    driver->disconnect == NULL)
		return -EINVAL;
	else if (udc         == NULL)
		return -ENODEV;
	else if (udc->driver != NULL)
		return -EBUSY;

	
	udc->qh_pool = dma_pool_create("ci13xxx_qh", &udc->gadget.dev,
				       sizeof(struct ci13xxx_qh),
				       64, CI13XXX_PAGE_SIZE);
	if (udc->qh_pool == NULL)
		return -ENOMEM;

	udc->td_pool = dma_pool_create("ci13xxx_td", &udc->gadget.dev,
				       sizeof(struct ci13xxx_td),
				       64, CI13XXX_PAGE_SIZE);
	if (udc->td_pool == NULL) {
		dma_pool_destroy(udc->qh_pool);
		udc->qh_pool = NULL;
		return -ENOMEM;
	}

	spin_lock_irqsave(udc->lock, flags);

	info("hw_ep_max = %d", hw_ep_max);

	udc->gadget.dev.driver = NULL;

	retval = 0;
	for (i = 0; i < hw_ep_max/2; i++) {
		for (j = RX; j <= TX; j++) {
			int k = i + j * hw_ep_max/2;
			struct ci13xxx_ep *mEp = &udc->ci13xxx_ep[k];

			scnprintf(mEp->name, sizeof(mEp->name), "ep%i%s", i,
					(j == TX)  ? "in" : "out");

			mEp->lock         = udc->lock;
			mEp->device       = &udc->gadget.dev;
			mEp->td_pool      = udc->td_pool;

			mEp->ep.name      = mEp->name;
			mEp->ep.ops       = &usb_ep_ops;
			mEp->ep.maxpacket =
				k ? USHRT_MAX : CTRL_PAYLOAD_MAX;

			INIT_LIST_HEAD(&mEp->qh.queue);
			spin_unlock_irqrestore(udc->lock, flags);
			mEp->qh.ptr = dma_pool_alloc(udc->qh_pool, GFP_KERNEL,
					&mEp->qh.dma);
			spin_lock_irqsave(udc->lock, flags);
			if (mEp->qh.ptr == NULL)
				retval = -ENOMEM;
			else
				memset(mEp->qh.ptr, 0, sizeof(*mEp->qh.ptr));

			
			if (i == 0)
				continue;

			list_add_tail(&mEp->ep.ep_list, &udc->gadget.ep_list);
		}
	}
	if (retval)
		goto done;
	spin_unlock_irqrestore(udc->lock, flags);
	udc->ep0out.ep.desc = &ctrl_endpt_out_desc;
	retval = usb_ep_enable(&udc->ep0out.ep);
	if (retval)
		return retval;

	udc->ep0in.ep.desc = &ctrl_endpt_in_desc;
	retval = usb_ep_enable(&udc->ep0in.ep);
	if (retval)
		return retval;
	udc->status = usb_ep_alloc_request(&udc->ep0in.ep, GFP_KERNEL);
	if (!udc->status)
		return -ENOMEM;
	udc->status_buf = kzalloc(2, GFP_KERNEL); 
	if (!udc->status_buf) {
		usb_ep_free_request(&udc->ep0in.ep, udc->status);
		return -ENOMEM;
	}
	spin_lock_irqsave(udc->lock, flags);

	udc->gadget.ep0 = &udc->ep0in.ep;
	
	driver->driver.bus     = NULL;
	udc->gadget.dev.driver = &driver->driver;
	udc->softconnect = 1;

	spin_unlock_irqrestore(udc->lock, flags);
	pm_runtime_get_sync(&udc->gadget.dev);
	retval = bind(&udc->gadget);                
	spin_lock_irqsave(udc->lock, flags);

	if (retval) {
		udc->gadget.dev.driver = NULL;
		goto done;
	}

	udc->driver = driver;
	if (udc->udc_driver->flags & CI13XXX_PULLUP_ON_VBUS) {
		if (udc->vbus_active) {
			if (udc->udc_driver->flags & CI13XXX_REGS_SHARED)
				hw_device_reset(udc);
		} else {
			put = true;
			goto done;
		}
	}

	if (!udc->softconnect) {
		put = true;
		goto done;
	}

	retval = hw_device_state(udc->ep0out.qh.dma);

 done:
	spin_unlock_irqrestore(udc->lock, flags);
	if (retval || put)
		pm_runtime_put_sync(&udc->gadget.dev);

	if (udc->udc_driver->notify_event)
			udc->udc_driver->notify_event(udc,
				CI13XXX_CONTROLLER_UDC_STARTED_EVENT);

	return retval;
}

static int ci13xxx_stop(struct usb_gadget_driver *driver)
{
	struct ci13xxx *udc = _udc;
	unsigned long i, flags;

	trace("%p", driver);

	if (driver             == NULL ||
	    driver->unbind     == NULL ||
	    driver->setup      == NULL ||
	    driver->disconnect == NULL ||
	    driver             != udc->driver)
		return -EINVAL;

	spin_lock_irqsave(udc->lock, flags);

	if (!(udc->udc_driver->flags & CI13XXX_PULLUP_ON_VBUS) ||
			udc->vbus_active) {
		hw_device_state(0);
		spin_unlock_irqrestore(udc->lock, flags);
		_gadget_stop_activity(&udc->gadget, 0);
		spin_lock_irqsave(udc->lock, flags);
		pm_runtime_put(&udc->gadget.dev);
	}

	
	spin_unlock_irqrestore(udc->lock, flags);
	driver->unbind(&udc->gadget);               
	spin_lock_irqsave(udc->lock, flags);

	usb_ep_free_request(&udc->ep0in.ep, udc->status);
	kfree(udc->status_buf);

	udc->gadget.dev.driver = NULL;

	
	for (i = 0; i < hw_ep_max; i++) {
		struct ci13xxx_ep *mEp = &udc->ci13xxx_ep[i];

		if (!list_empty(&mEp->ep.ep_list))
			list_del_init(&mEp->ep.ep_list);

		if (mEp->qh.ptr != NULL)
			dma_pool_free(udc->qh_pool, mEp->qh.ptr, mEp->qh.dma);
	}

	udc->gadget.ep0 = NULL;
	udc->driver = NULL;

	spin_unlock_irqrestore(udc->lock, flags);

	if (udc->td_pool != NULL) {
		dma_pool_destroy(udc->td_pool);
		udc->td_pool = NULL;
	}
	if (udc->qh_pool != NULL) {
		dma_pool_destroy(udc->qh_pool);
		udc->qh_pool = NULL;
	}

	return 0;
}

static irqreturn_t udc_irq(void)
{
	struct ci13xxx *udc = _udc;
	irqreturn_t retval;
	u32 intr;

	trace();

	if (udc == NULL) {
		err("ENODEV");
		return IRQ_HANDLED;
	}

	spin_lock(udc->lock);

	if (udc->udc_driver->flags & CI13XXX_REGS_SHARED) {
		if (hw_cread(CAP_USBMODE, USBMODE_CM) !=
				USBMODE_CM_DEVICE) {
			spin_unlock(udc->lock);
			return IRQ_NONE;
		}
	}
	intr = hw_test_and_clear_intr_active();
	if (intr) {
		isr_statistics.hndl.buf[isr_statistics.hndl.idx++] = intr;
		isr_statistics.hndl.idx &= ISR_MASK;
		isr_statistics.hndl.cnt++;

		
		if (USBi_URI & intr) {
			USB_INFO("reset, count %d\n",udc->gadget.ats_reset_irq_count);
			isr_statistics.uri++;

			if (udc->gadget.ats_reset_irq_count == 50) {
				udc->gadget.ats_reset_irq_count++;
				if (udc->driver->broadcast_abnormal_usb_reset) {
							printk(KERN_INFO "[USB] gadget irq :abnormal the amount of reset irq!\n");
							udc->driver->broadcast_abnormal_usb_reset();
				}
			} else if (udc->gadget.ats_reset_irq_count < 50)
				udc->gadget.ats_reset_irq_count++;

			
			if (board_mfg_mode() == 5 || msm_otg_usb_disable) {
				USB_INFO("Offmode / QuickBootMode\n");
				spin_unlock(udc->lock);
				if (udc->transceiver)
					udc->transceiver->notify_usb_disabled();
				spin_lock(udc->lock);
			}
			
			isr_reset_handler(udc);
			if (udc->transceiver)
				udc->transceiver->notify_usb_attached(NULL);
		}
		if (USBi_PCI & intr) {
			isr_statistics.pci++;
			isr_resume_handler(udc);
		}
		if (USBi_UEI & intr)
			isr_statistics.uei++;
		if (USBi_UI  & intr) {
			isr_statistics.ui++;
			udc->gadget.xfer_isr_count++;
			isr_tr_complete_handler(udc);
		}
		if (USBi_SLI & intr) {
			USB_INFO("suspend\n");
			isr_suspend_handler(udc);
			isr_statistics.sli++;
		}
		retval = IRQ_HANDLED;
	} else {
		isr_statistics.none++;
		retval = IRQ_NONE;
	}
	spin_unlock(udc->lock);

	return retval;
}

static void udc_release(struct device *dev)
{
	trace("%p", dev);

	if (dev == NULL)
		err("EINVAL");
}

static int udc_probe(struct ci13xxx_udc_driver *driver, struct device *dev,
		void __iomem *regs)
{
	struct ci13xxx *udc;
	struct ci13xxx_platform_data *pdata;
	int retval = 0, i;

	trace("%p, %p, %p", dev, regs, driver->name);

	if (dev == NULL || regs == NULL || driver == NULL ||
			driver->name == NULL)
		return -EINVAL;

	udc = kzalloc(sizeof(struct ci13xxx), GFP_KERNEL);
	if (udc == NULL)
		return -ENOMEM;

	udc->lock = &udc_lock;
	udc->regs = regs;
	udc->udc_driver = driver;

	udc->gadget.ops          = &usb_gadget_ops;
	udc->gadget.speed        = USB_SPEED_UNKNOWN;
	udc->gadget.max_speed    = USB_SPEED_HIGH;
	if (udc->udc_driver->flags & CI13XXX_IS_OTG)
		udc->gadget.is_otg       = 1;
	else
		udc->gadget.is_otg       = 0;
	udc->gadget.name         = driver->name;

	INIT_LIST_HEAD(&udc->gadget.ep_list);
	udc->gadget.ep0 = NULL;

	pdata = dev->platform_data;
	if (pdata)
		udc->gadget.usb_core_id = pdata->usb_core_id;

	dev_set_name(&udc->gadget.dev, "gadget");
	udc->gadget.dev.dma_mask = dev->dma_mask;
	udc->gadget.dev.coherent_dma_mask = dev->coherent_dma_mask;
	udc->gadget.dev.parent   = dev;
	udc->gadget.dev.release  = udc_release;

	if (udc->udc_driver->flags & CI13XXX_REQUIRE_TRANSCEIVER) {
		udc->transceiver = usb_get_transceiver();
		if (udc->transceiver == NULL) {
			retval = -ENODEV;
			goto free_udc;
		}
	}

	INIT_DELAYED_WORK(&udc->rw_work, usb_do_remote_wakeup);
	INIT_DELAYED_WORK(&udc->chg_stop, usb_chg_stop);

	retval = hw_device_init(regs);
	if (retval < 0)
		goto put_transceiver;

	for (i = 0; i < hw_ep_max; i++) {
		struct ci13xxx_ep *mEp = &udc->ci13xxx_ep[i];
		INIT_LIST_HEAD(&mEp->ep.ep_list);
		setup_timer(&mEp->prime_timer, ep_prime_timer_func,
			(unsigned long) mEp);
	}

	if (!(udc->udc_driver->flags & CI13XXX_REGS_SHARED)) {
		retval = hw_device_reset(udc);
		if (retval)
			goto put_transceiver;
	}

	retval = device_register(&udc->gadget.dev);
	if (retval) {
		put_device(&udc->gadget.dev);
		goto put_transceiver;
	}

#ifdef CONFIG_USB_GADGET_DEBUG_FILES
	retval = dbg_create_files(&udc->gadget.dev);
#endif
	if (retval)
		goto unreg_device;

	if (udc->transceiver) {
		retval = otg_set_peripheral(udc->transceiver->otg,
						&udc->gadget);
		if (retval)
			goto remove_dbg;
	}

	retval = usb_add_gadget_udc(dev, &udc->gadget);
	if (retval)
		goto remove_trans;

	pm_runtime_no_callbacks(&udc->gadget.dev);
	pm_runtime_enable(&udc->gadget.dev);

	if (register_trace_usb_daytona_invalid_access(dump_usb_info, NULL))
		pr_err("Registering trace failed\n");

	_udc = udc;
	return retval;

remove_trans:
	if (udc->transceiver) {
		otg_set_peripheral(udc->transceiver->otg, &udc->gadget);
		usb_put_transceiver(udc->transceiver);
	}

	err("error = %i", retval);
remove_dbg:
#ifdef CONFIG_USB_GADGET_DEBUG_FILES
	dbg_remove_files(&udc->gadget.dev);
#endif
unreg_device:
	device_unregister(&udc->gadget.dev);
put_transceiver:
	if (udc->transceiver)
		usb_put_transceiver(udc->transceiver);
free_udc:
	kfree(udc);
	_udc = NULL;
	return retval;
}

static void udc_remove(void)
{
	struct ci13xxx *udc = _udc;
	int retval;

	if (udc == NULL) {
		err("EINVAL");
		return;
	}
	retval = unregister_trace_usb_daytona_invalid_access(dump_usb_info,
									NULL);
	if (retval)
		pr_err("Unregistering trace failed\n");

	usb_del_gadget_udc(&udc->gadget);

	if (udc->transceiver) {
		otg_set_peripheral(udc->transceiver->otg, &udc->gadget);
		usb_put_transceiver(udc->transceiver);
	}
#ifdef CONFIG_USB_GADGET_DEBUG_FILES
	dbg_remove_files(&udc->gadget.dev);
#endif
	device_unregister(&udc->gadget.dev);

	kfree(udc);
	_udc = NULL;
}
