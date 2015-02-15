/* Copyright (c) 2009-2013, The Linux Foundation. All rights reserved.
 * Copyright (c) 2010, Google Inc.
 *
 * Original authors: Code Aurora Forum
 *
 * Author: Dima Zavin <dima@android.com>
 *  - Largely rewritten from original to not be an i2c driver.
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

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/msm_ssbi.h>
#include <linux/remote_spinlock.h>

/* SSBI 2.0 controller registers */
#define SSBI2_CMD			0x0008
#define SSBI2_RD			0x0010
#define SSBI2_STATUS			0x0014
#define SSBI2_MODE2			0x001C

/* SSBI_CMD fields */
#define SSBI_CMD_RDWRN			(1 << 24)

/* SSBI_STATUS fields */
#define SSBI_STATUS_RD_READY		(1 << 2)
#define SSBI_STATUS_READY		(1 << 1)
#define SSBI_STATUS_MCHN_BUSY		(1 << 0)

/* SSBI_MODE2 fields */
#define SSBI_MODE2_REG_ADDR_15_8_SHFT	0x04
#define SSBI_MODE2_REG_ADDR_15_8_MASK	(0x7f << SSBI_MODE2_REG_ADDR_15_8_SHFT)

#define SET_SSBI_MODE2_REG_ADDR_15_8(MD, AD) \
	(((MD) & 0x0F) | ((((AD) >> 8) << SSBI_MODE2_REG_ADDR_15_8_SHFT) & \
	SSBI_MODE2_REG_ADDR_15_8_MASK))

/* SSBI PMIC Arbiter command registers */
#define SSBI_PA_CMD			0x0000
#define SSBI_PA_RD_STATUS		0x0004

/* SSBI_PA_CMD fields */
#define SSBI_PA_CMD_RDWRN		(1 << 24)
#define SSBI_PA_CMD_ADDR_MASK		0x7fff /* REG_ADDR_7_0, REG_ADDR_8_14*/

/* SSBI_PA_RD_STATUS fields */
#define SSBI_PA_RD_STATUS_TRANS_DONE	(1 << 27)
#define SSBI_PA_RD_STATUS_TRANS_DENIED	(1 << 26)

#define SSBI_TIMEOUT_US			100

/* SSBI_FSM Read and Write commands for the FSM9xxx SSBI implementation */
#define SSBI_FSM_CMD_REG_ADDR_SHFT  (0x08)

#define SSBI_FSM_CMD_READ(AD) \
	(SSBI_CMD_RDWRN | (((AD) & 0xFFFF) << SSBI_FSM_CMD_REG_ADDR_SHFT))

#define SSBI_FSM_CMD_WRITE(AD, DT) \
	((((AD) & 0xFFFF) << SSBI_FSM_CMD_REG_ADDR_SHFT) | ((DT) & 0xFF))

struct msm_ssbi {
	struct device		*dev;
	struct device		*slave;
	void __iomem		*base;
	bool			 use_rlock;
	spinlock_t		lock;
	remote_spinlock_t	 rspin_lock;
	enum msm_ssbi_controller_type controller_type;
	int (*read)(struct msm_ssbi *, u16 addr, u8 *buf, int len);
	int (*write)(struct msm_ssbi *, u16 addr, u8 *buf, int len);
};

#define to_msm_ssbi(dev)	platform_get_drvdata(to_platform_device(dev))

static inline u32 ssbi_readl(struct msm_ssbi *ssbi, u32 reg)
{
	return readl(ssbi->base + reg);
}

static inline void ssbi_writel(struct msm_ssbi *ssbi, u32 val, u32 reg)
{
	writel(val, ssbi->base + reg);
}

static int ssbi_wait_mask(struct msm_ssbi *ssbi, u32 set_mask, u32 clr_mask)
{
	u32 timeout = SSBI_TIMEOUT_US;
	u32 val;

	while (timeout--) {
		val = ssbi_readl(ssbi, SSBI2_STATUS);
		if (((val & set_mask) == set_mask) && ((val & clr_mask) == 0))
			return 0;
		udelay(1);
	}

	dev_err(ssbi->dev, "%s: timeout (status %x set_mask %x clr_mask %x)\n",
		__func__, ssbi_readl(ssbi, SSBI2_STATUS), set_mask, clr_mask);
	return -ETIMEDOUT;
}

static int
msm_ssbi_read_bytes(struct msm_ssbi *ssbi, u16 addr, u8 *buf, int len)
{
	u32 cmd = SSBI_CMD_RDWRN | ((addr & 0xff) << 16);
	int ret = 0;

	if (ssbi->controller_type == MSM_SBI_CTRL_SSBI2) {
		u32 mode2 = ssbi_readl(ssbi, SSBI2_MODE2);
		mode2 = SET_SSBI_MODE2_REG_ADDR_15_8(mode2, addr);
		ssbi_writel(ssbi, mode2, SSBI2_MODE2);
	}

	if (ssbi->controller_type == FSM_SBI_CTRL_SSBI)
		cmd = SSBI_FSM_CMD_READ(addr);
	else
		cmd = SSBI_CMD_RDWRN | ((addr & 0xff) << 16);

	while (len) {
		ret = ssbi_wait_mask(ssbi, SSBI_STATUS_READY, 0);
		if (ret)
			goto err;

		ssbi_writel(ssbi, cmd, SSBI2_CMD);
		ret = ssbi_wait_mask(ssbi, SSBI_STATUS_RD_READY, 0);
		if (ret)
			goto err;
		*buf++ = ssbi_readl(ssbi, SSBI2_RD) & 0xff;
		len--;
	}

err:
	return ret;
}

static int
msm_ssbi_write_bytes(struct msm_ssbi *ssbi, u16 addr, u8 *buf, int len)
{
	int ret = 0;

	if (ssbi->controller_type == MSM_SBI_CTRL_SSBI2) {
		u32 mode2 = ssbi_readl(ssbi, SSBI2_MODE2);
		mode2 = SET_SSBI_MODE2_REG_ADDR_15_8(mode2, addr);
		ssbi_writel(ssbi, mode2, SSBI2_MODE2);
	}

	while (len) {
		ret = ssbi_wait_mask(ssbi, SSBI_STATUS_READY, 0);
		if (ret)
			goto err;

		if (ssbi->controller_type == FSM_SBI_CTRL_SSBI)
			ssbi_writel(ssbi, SSBI_FSM_CMD_WRITE(addr, *buf),
				SSBI2_CMD);
		else
			ssbi_writel(ssbi, ((addr & 0xff) << 16) | *buf,
				SSBI2_CMD);

		ret = ssbi_wait_mask(ssbi, 0, SSBI_STATUS_MCHN_BUSY);
		if (ret)
			goto err;
		buf++;
		len--;
	}

err:
	return ret;
}

static inline int
msm_ssbi_pa_transfer(struct msm_ssbi *ssbi, u32 cmd, u8 *data)
{
	u32 timeout = SSBI_TIMEOUT_US;
	u32 rd_status = 0;

	ssbi_writel(ssbi, cmd, SSBI_PA_CMD);

	while (timeout--) {
		rd_status = ssbi_readl(ssbi, SSBI_PA_RD_STATUS);

		if (rd_status & SSBI_PA_RD_STATUS_TRANS_DENIED) {
			dev_err(ssbi->dev, "%s: transaction denied (0x%x)\n",
					__func__, rd_status);
			return -EPERM;
		}

		if (rd_status & SSBI_PA_RD_STATUS_TRANS_DONE) {
			if (data)
				*data = rd_status & 0xff;
			return 0;
		}
		udelay(1);
	}

	dev_err(ssbi->dev, "%s: timeout, status 0x%x\n", __func__, rd_status);
	return -ETIMEDOUT;
}

static int
msm_ssbi_pa_read_bytes(struct msm_ssbi *ssbi, u16 addr, u8 *buf, int len)
{
	u32 cmd;
	int ret = 0;

	cmd = SSBI_PA_CMD_RDWRN | (addr & SSBI_PA_CMD_ADDR_MASK) << 8;

	while (len) {
		ret = msm_ssbi_pa_transfer(ssbi, cmd, buf);
		if (ret)
			goto err;
		buf++;
		len--;
	}

err:
	return ret;
}

static int
msm_ssbi_pa_write_bytes(struct msm_ssbi *ssbi, u16 addr, u8 *buf, int len)
{
	u32 cmd;
	int ret = 0;

	while (len) {
		cmd = (addr & SSBI_PA_CMD_ADDR_MASK) << 8 | *buf;
		ret = msm_ssbi_pa_transfer(ssbi, cmd, NULL);
		if (ret)
			goto err;
		buf++;
		len--;
	}

err:
	return ret;
}

int msm_ssbi_read(struct device *dev, u16 addr, u8 *buf, int len)
{
	struct msm_ssbi *ssbi = to_msm_ssbi(dev);
	unsigned long flags;
	int ret;

	if (ssbi->dev != dev)
		return -ENXIO;

	if (ssbi->use_rlock) {
		remote_spin_lock_irqsave(&ssbi->rspin_lock, flags);
		ret = ssbi->read(ssbi, addr, buf, len);
		remote_spin_unlock_irqrestore(&ssbi->rspin_lock, flags);
	} else {
		spin_lock_irqsave(&ssbi->lock, flags);
		ret = ssbi->read(ssbi, addr, buf, len);
		spin_unlock_irqrestore(&ssbi->lock, flags);
	}

	return ret;
}
EXPORT_SYMBOL(msm_ssbi_read);

int msm_ssbi_write(struct device *dev, u16 addr, u8 *buf, int len)
{
	struct msm_ssbi *ssbi = to_msm_ssbi(dev);
	unsigned long flags;
	int ret;

	if (ssbi->dev != dev)
		return -ENXIO;

	if (ssbi->use_rlock) {
		remote_spin_lock_irqsave(&ssbi->rspin_lock, flags);
		ret = ssbi->write(ssbi, addr, buf, len);
		remote_spin_unlock_irqrestore(&ssbi->rspin_lock, flags);
	} else {
		spin_lock_irqsave(&ssbi->lock, flags);
		ret = ssbi->write(ssbi, addr, buf, len);
		spin_unlock_irqrestore(&ssbi->lock, flags);
	}

	return ret;
}
EXPORT_SYMBOL(msm_ssbi_write);

static int __devinit msm_ssbi_add_slave(struct msm_ssbi *ssbi,
				const struct msm_ssbi_slave_info *slave)
{
	struct platform_device *slave_pdev;
	int ret;

	if (ssbi->slave) {
		pr_err("slave already attached??\n");
		return -EBUSY;
	}

	slave_pdev = platform_device_alloc(slave->name, -1);
	if (!slave_pdev) {
		pr_err("cannot allocate pdev for slave '%s'", slave->name);
		ret = -ENOMEM;
		goto err;
	}

	slave_pdev->dev.parent = ssbi->dev;
	slave_pdev->dev.platform_data = slave->platform_data;

	ret = platform_device_add(slave_pdev);
	if (ret) {
		pr_err("cannot add slave platform device for '%s'\n",
				slave->name);
		goto err;
	}

	ssbi->slave = &slave_pdev->dev;
	return 0;

err:
	if (slave_pdev)
		platform_device_put(slave_pdev);
	return ret;
}

static int __devinit msm_ssbi_probe(struct platform_device *pdev)
{
	const struct msm_ssbi_platform_data *pdata = pdev->dev.platform_data;
	struct resource *mem_res;
	struct msm_ssbi *ssbi;
	int ret = 0;

	if (!pdata) {
		pr_err("missing platform data\n");
		return -EINVAL;
	}

	pr_debug("%s\n", pdata->slave.name);

	ssbi = kzalloc(sizeof(struct msm_ssbi), GFP_KERNEL);
	if (!ssbi) {
		pr_err("can not allocate ssbi_data\n");
		return -ENOMEM;
	}

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_res) {
		pr_err("missing mem resource\n");
		ret = -EINVAL;
		goto err_get_mem_res;
	}

	ssbi->base = ioremap(mem_res->start, resource_size(mem_res));
	if (!ssbi->base) {
		pr_err("ioremap failed: %pr\n", mem_res);
		ret = -EINVAL;
		goto err_ioremap;
	}
	ssbi->dev = &pdev->dev;
	platform_set_drvdata(pdev, ssbi);

	ssbi->controller_type = pdata->controller_type;
	if (ssbi->controller_type == MSM_SBI_CTRL_PMIC_ARBITER) {
		ssbi->read = msm_ssbi_pa_read_bytes;
		ssbi->write = msm_ssbi_pa_write_bytes;
	} else {
		ssbi->read = msm_ssbi_read_bytes;
		ssbi->write = msm_ssbi_write_bytes;
	}

	if (pdata->rsl_id) {
		ret = remote_spin_lock_init(&ssbi->rspin_lock, pdata->rsl_id);
		if (ret) {
			dev_err(&pdev->dev, "remote spinlock init failed\n");
			goto err_ssbi_add_slave;
		}
		ssbi->use_rlock = 1;
	}

	spin_lock_init(&ssbi->lock);

	ret = msm_ssbi_add_slave(ssbi, &pdata->slave);
	if (ret)
		goto err_ssbi_add_slave;

	return 0;

err_ssbi_add_slave:
	platform_set_drvdata(pdev, NULL);
	iounmap(ssbi->base);
err_ioremap:
err_get_mem_res:
	kfree(ssbi);
	return ret;
}

static int __devexit msm_ssbi_remove(struct platform_device *pdev)
{
	struct msm_ssbi *ssbi = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	iounmap(ssbi->base);
	kfree(ssbi);
	return 0;
}

static struct platform_driver msm_ssbi_driver = {
	.probe		= msm_ssbi_probe,
	.remove		= __exit_p(msm_ssbi_remove),
	.driver		= {
		.name	= "msm_ssbi",
		.owner	= THIS_MODULE,
	},
};

static int __init msm_ssbi_init(void)
{
	return platform_driver_register(&msm_ssbi_driver);
}
postcore_initcall(msm_ssbi_init);

static void __exit msm_ssbi_exit(void)
{
	platform_driver_unregister(&msm_ssbi_driver);
}
module_exit(msm_ssbi_exit)

MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:msm_ssbi");
MODULE_AUTHOR("Dima Zavin <dima@android.com>");
