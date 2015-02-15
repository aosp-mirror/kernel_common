/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
#include <linux/fs.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include "nfc-nci.h"
#include <mach/gpiomux.h>

struct qca199x_platform_data {
	unsigned int irq_gpio;
	unsigned int	irq_gpio_clk_req;
	unsigned int	clk_req_irq_num;
	unsigned int dis_gpio;
	unsigned int clkreq_gpio;
	unsigned int reg;
	const char *clk_src_name;
	unsigned int clk_src_gpio;
};

static struct of_device_id msm_match_table[] = {
	{.compatible = "qcom,nfc-nci"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_match_table);

#define MAX_BUFFER_SIZE			(780)
#define PACKET_MAX_LENGTH		(258)
/* Read data */
#define PACKET_HEADER_SIZE_NCI	(4)
#define PACKET_TYPE_NCI			(16)
#define MAX_PACKET_SIZE			(PACKET_HEADER_SIZE_NCI + 255)
#define MAX_QCA_REG				(116)
/* will timeout in approx. 100ms as 10us steps */
#define NFC_RF_CLK_FREQ			(19200000)
#define NTF_TIMEOUT				(25)
#define	CORE_RESET_RSP_GID		(0x60)
#define	CORE_RESET_OID			(0x00)
#define CORE_RST_NTF_LENGTH		(0x02)
#define WAKE_TIMEOUT			(10)
#define WAKE_REG			(0x10)

static void clk_req_update(struct work_struct *work);

struct qca199x_dev {
	wait_queue_head_t read_wq;
	struct	mutex		read_mutex;
	struct	i2c_client	*client;
	struct	miscdevice	qca199x_device;
	/* NFC_IRQ new NCI data available */
	unsigned int		irq_gpio;
	/* CLK_REQ IRQ to signal the state has changed */
	unsigned int		irq_gpio_clk_req;
	/* Actual IRQ no. assigned to CLK_REQ */
	unsigned int		clk_req_irq_num;
	unsigned int		dis_gpio;
	unsigned int		clkreq_gpio;
	/* NFC_IRQ state */
	bool			irq_enabled;
	bool			sent_first_nci_write;
	spinlock_t		irq_enabled_lock;
	unsigned int		count_irq;
	/* CLK_REQ IRQ state */
	bool			irq_enabled_clk_req;
	spinlock_t		irq_enabled_lock_clk_req;
	unsigned int		count_irq_clk_req;
	enum	nfcc_state	state;
	/* CLK control */
	unsigned int		clk_src_gpio;
	const	char		*clk_src_name;
	struct	clk		*s_clk;
	unsigned int 		core_reset_ntf;
	bool			clk_run;
	struct work_struct	msm_clock_controll_work;
	struct workqueue_struct *my_wq;
};

static int nfc_i2c_write(struct i2c_client *client, u8 *buf, int len);
static int nfcc_hw_check(struct i2c_client *client, unsigned short curr_addr);
static int nfcc_initialise(struct i2c_client *client, unsigned short curr_addr,
				struct qca199x_dev *qca199x_dev);
static int qca199x_clock_select(struct qca199x_dev *qca199x_dev);
static int qca199x_clock_deselect(struct qca199x_dev *qca199x_dev);



/*
 * To allow filtering of nfc logging from user. This is set via
 * IOCTL NFC_KERNEL_LOGGING_MODE.
 */
static int logging_level;
/*
 * FTM-RAW-I2C RD/WR MODE
 */
static struct devicemode	device_mode;
static int					ftm_raw_write_mode;
static int					ftm_werr_code;


unsigned int	disable_ctrl;
bool		region2_sent;

static void qca199x_init_stat(struct qca199x_dev *qca199x_dev)
{
	qca199x_dev->count_irq = 0;
}

static void qca199x_disable_irq(struct qca199x_dev *qca199x_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&qca199x_dev->irq_enabled_lock, flags);
	if (qca199x_dev->irq_enabled) {
		disable_irq_nosync(qca199x_dev->client->irq);
		qca199x_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&qca199x_dev->irq_enabled_lock, flags);
}

static void qca199x_enable_irq(struct qca199x_dev *qca199x_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&qca199x_dev->irq_enabled_lock, flags);
	if (!qca199x_dev->irq_enabled) {
		qca199x_dev->irq_enabled = true;
		enable_irq(qca199x_dev->client->irq);
	}
	spin_unlock_irqrestore(&qca199x_dev->irq_enabled_lock, flags);
}

static irqreturn_t qca199x_dev_irq_handler(int irq, void *dev_id)
{
	struct qca199x_dev *qca199x_dev = dev_id;
	unsigned long flags;

	spin_lock_irqsave(&qca199x_dev->irq_enabled_lock, flags);
	qca199x_dev->count_irq++;
	spin_unlock_irqrestore(&qca199x_dev->irq_enabled_lock, flags);
	wake_up(&qca199x_dev->read_wq);

	return IRQ_HANDLED;
}

static unsigned int nfc_poll(struct file *filp, poll_table *wait)
{
	struct qca199x_dev *qca199x_dev = filp->private_data;
	unsigned int mask = 0;
	unsigned long flags;

	poll_wait(filp, &qca199x_dev->read_wq, wait);

	spin_lock_irqsave(&qca199x_dev->irq_enabled_lock, flags);
	if (qca199x_dev->count_irq > 0) {
		qca199x_dev->count_irq--;
		mask |= POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&qca199x_dev->irq_enabled_lock, flags);

	return mask;
}

/* Handlers for CLK_REQ */
static void qca199x_disable_irq_clk_req(struct qca199x_dev *qca199x_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&qca199x_dev->irq_enabled_lock_clk_req, flags);
	if (qca199x_dev->irq_enabled_clk_req) {
		disable_irq_nosync(qca199x_dev->clk_req_irq_num);
		qca199x_dev->irq_enabled_clk_req = false;
	}
	spin_unlock_irqrestore(&qca199x_dev->irq_enabled_lock_clk_req, flags);
}


static void qca199x_enable_irq_clk_req(struct qca199x_dev *qca199x_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&qca199x_dev->irq_enabled_lock_clk_req, flags);
	if (!qca199x_dev->irq_enabled_clk_req) {
		qca199x_dev->irq_enabled_clk_req = true;
		enable_irq(qca199x_dev->clk_req_irq_num);
	}
	spin_unlock_irqrestore(&qca199x_dev->irq_enabled_lock_clk_req, flags);
}


static irqreturn_t qca199x_dev_irq_handler_clk_req(int irq, void *dev_id)
{
	struct qca199x_dev *qca199x_dev = dev_id;
	unsigned long flags;

	spin_lock_irqsave(&qca199x_dev->irq_enabled_lock_clk_req, flags);
	qca199x_dev->count_irq_clk_req++;
	spin_unlock_irqrestore(&qca199x_dev->irq_enabled_lock_clk_req, flags);

	queue_work(qca199x_dev->my_wq, &qca199x_dev->msm_clock_controll_work);

	return IRQ_HANDLED;
}


static struct gpiomux_setting nfc_clk_on = {
	.func = GPIOMUX_FUNC_2,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct gpiomux_setting nfc_clk_on_suspend = {
	.func = GPIOMUX_FUNC_2,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting nfc_clk_off = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static void clk_req_update(struct work_struct *work)
{
	struct	i2c_client *client;
	struct	qca199x_dev *qca199x_dev;
	int		gpio_clk_req_level = 0;

	qca199x_dev =	container_of(work, struct qca199x_dev,
			msm_clock_controll_work);
	client =	qca199x_dev->client;

	/* Read status level of CLK_REQ from NFC Controller, QCA199_x */
	gpio_clk_req_level = gpio_get_value(qca199x_dev->irq_gpio_clk_req);
	if (gpio_clk_req_level == 1) {
		if (qca199x_dev->clk_run == false) {
			msm_gpiomux_write(qca199x_dev->clk_src_gpio,
				GPIOMUX_ACTIVE, &nfc_clk_on, NULL);
			msm_gpiomux_write(qca199x_dev->clk_src_gpio,
				GPIOMUX_SUSPENDED, &nfc_clk_on_suspend, NULL);
			qca199x_dev->clk_run = true;
			}
	} else{
		if (qca199x_dev->clk_run == true) {
			msm_gpiomux_write(qca199x_dev->clk_src_gpio,
				GPIOMUX_ACTIVE, &nfc_clk_off, NULL);
			msm_gpiomux_write(qca199x_dev->clk_src_gpio,
				GPIOMUX_SUSPENDED, &nfc_clk_off, NULL);
			qca199x_dev->clk_run = false;
		}
	}
}

/*
 * ONLY for FTM-RAW-I2C Mode
 * Required to instigate a read, which comes from DT layer. This means we need
 * to spoof an interrupt and send a wake up event.
 */
void ftm_raw_trigger_read(struct qca199x_dev *qca199x_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&qca199x_dev->irq_enabled_lock, flags);
	qca199x_dev->count_irq++;
	spin_unlock_irqrestore(&qca199x_dev->irq_enabled_lock, flags);
	wake_up(&qca199x_dev->read_wq);
}

static ssize_t nfc_read(struct file *filp, char __user *buf,
					size_t count, loff_t *offset)
{
	struct qca199x_dev *qca199x_dev = filp->private_data;
	unsigned char tmp[MAX_BUFFER_SIZE], rd_byte;
	unsigned char len[PAYLOAD_HEADER_LENGTH];
	int total, length, ret;
	int ftm_rerr_code;
	enum ehandler_mode dmode;

	total = 0;
	length = 0;
	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	mutex_lock(&qca199x_dev->read_mutex);
	memset(tmp, 0, sizeof(tmp));
	memset(len, 0, sizeof(len));
	dmode = device_mode.handle_flavour;
	/* FTM-RAW-I2C RD/WR MODE - Special Case */
	if ((dmode == UNSOLICITED_FTM_RAW_MODE) ||
		(dmode == SOLICITED_FTM_RAW_MODE)) {
		/* READ */
		if ((ftm_raw_write_mode == 0) && (ftm_werr_code == 0)) {
			ftm_rerr_code = i2c_master_recv(qca199x_dev->client,
						&rd_byte, 1);
			if (ftm_rerr_code == 0x1)
				ftm_rerr_code = 0;
			tmp[0] = (unsigned char)ftm_rerr_code;
			tmp[1] = rd_byte;
			total  = 2;
			ret = copy_to_user(buf, tmp, total);
		}
		/* WRITE */
		else if ((ftm_raw_write_mode == 1) || (ftm_werr_code != 0)) {
			tmp[0] = (unsigned char)ftm_werr_code;
			total = 1;
			ret = copy_to_user(buf, tmp, total);
		} else {
			/* Invalid case */
			total = 0;
			ret = copy_to_user(buf, tmp, total);
		}
		mutex_unlock(&qca199x_dev->read_mutex);
		goto done;
	}

	/* NORMAL NCI Behaviour */
	/* Read the header */
	ret = i2c_master_recv(qca199x_dev->client, len, PAYLOAD_HEADER_LENGTH);
	/*
	 * We ignore all packets of length PAYLOAD_HEADER_LENGTH
	 * or less (i.e <=3). In this case return a total length
	 * of ZERO. So ALL PACKETS MUST HAVE A PAYLOAD.
	 * If ret < 0 then this is an error code.
	 */
	if (ret != PAYLOAD_HEADER_LENGTH) {
		if (ret < 0)
			total = ret;
		else
			total = 0;
		goto err;
	}
	length = len[PAYLOAD_HEADER_LENGTH - 1];
	if (length == 0) {
		ret = 0;
		total = ret;
		goto err;
	}
	/** make sure full packet fits in the buffer **/
	if ((length > 0) && ((length + PAYLOAD_HEADER_LENGTH) <= count)) {
		/* Read the packet */
		ret = i2c_master_recv(qca199x_dev->client, tmp, (length +
			PAYLOAD_HEADER_LENGTH));
		total = ret;
		if (ret < 0)
			goto err;
	}

	dev_dbg(&qca199x_dev->client->dev, "%s : NfcNciRx %x %x %x\n",
			__func__, tmp[0], tmp[1], tmp[2]);
	if (total > 0) {
		if ((total > count) || copy_to_user(buf, tmp, total)) {
			dev_err(&qca199x_dev->client->dev,
				"failed to copy to user space, total = %d\n",
					total);
			total = -EFAULT;
		}
	}
err:
		mutex_unlock(&qca199x_dev->read_mutex);
done:
	return total;
}

/*
 * Local routine to read from nfcc buffer. This is called to clear any
 * pending receive messages in the nfcc's read buffer, which may be there
 * following a POR. In this way, the upper layers (Device Transport) will
 * associate the next rsp/ntf nci message with the next nci command to the
 * nfcc. Otherwise, the DT may interpret a ntf from the nfcc as being from
 * the nci core reset command when in fact it was already present in the
 * nfcc read buffer following a POR.
 */

int nfcc_read_buff_svc(struct qca199x_dev *qca199x_dev)
{
	unsigned char tmp[PACKET_MAX_LENGTH];
	unsigned char len[PAYLOAD_HEADER_LENGTH];
	int total, length, ret;
	total = 0;
	length = 0;
	mutex_lock(&qca199x_dev->read_mutex);
	memset(tmp, 0, sizeof(tmp));
	memset(len, 0, sizeof(len));

	/* Read the header */
	ret = i2c_master_recv(qca199x_dev->client, len, PAYLOAD_HEADER_LENGTH);
	if (ret < PAYLOAD_HEADER_LENGTH) {
		total = ret;
		goto leave;
	}
	length = len[PAYLOAD_HEADER_LENGTH - 1];
	if (length == 0) {
		ret = PAYLOAD_HEADER_LENGTH;
		total = ret;
		goto leave;
	}
	/** make sure full packet fits in the buffer **/
	if ((length > 0) && ((length + PAYLOAD_HEADER_LENGTH) <= PACKET_MAX_LENGTH)) {
		/* Read the packet */
		ret = i2c_master_recv(qca199x_dev->client, tmp, (length +
			PAYLOAD_HEADER_LENGTH));
		total = ret;
	}
	dev_dbg(&qca199x_dev->client->dev, "%s : NfcNciRx %x %x %x\n",
			__func__, tmp[0], tmp[1], tmp[2]);
leave:
	mutex_unlock(&qca199x_dev->read_mutex);
	return total;
}

static ssize_t nfc_write(struct file *filp, const char __user *buf,
				size_t count, loff_t *offset)
{
	struct qca199x_dev *qca199x_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret = 0;
	enum ehandler_mode dmode;
	int nfcc_buffer = 0;

	if (count > MAX_BUFFER_SIZE) {
		dev_err(&qca199x_dev->client->dev, "out of memory\n");
		return -ENOMEM;
	}
	if (copy_from_user(tmp, buf, count)) {
		dev_err(&qca199x_dev->client->dev,
			"nfc-nci write: failed to copy from user space\n");
		return -EFAULT;
	}
	/*
	 * A catch for when the DT is sending the initial NCI write
	 * following a hardware POR. In this case we should clear any
	 * pending messages in nfcc buffer and open the interrupt gate
	 * for new messages coming from the nfcc.
	 */
	if ((qca199x_dev->sent_first_nci_write == false) &&
		 (qca199x_dev->irq_enabled == false)) {
		/* check rsp/ntf from nfcc read-side buffer */
		nfcc_buffer = nfcc_read_buff_svc(qca199x_dev);
		/* There has been an error while reading from nfcc */
		if (nfcc_buffer < 0) {
			dev_err(&qca199x_dev->client->dev,
				"nfc-nci write: error while servicing nfcc read buffer\n");
		}
		qca199x_dev->sent_first_nci_write = true;
		qca199x_enable_irq(qca199x_dev);
	}
	mutex_lock(&qca199x_dev->read_mutex);
	dmode = device_mode.handle_flavour;
	/* FTM-DIRECT-I2C RD/WR MODE */
	/* This is a special FTM-i2c mode case, where tester is not using NCI */
	if ((dmode == UNSOLICITED_FTM_RAW_MODE) ||
		(dmode == SOLICITED_FTM_RAW_MODE)) {
		/* Read From Register */
		if (count == 1) {
			ftm_raw_write_mode = 0;
			ret = i2c_master_send(qca199x_dev->client, tmp, count);
			if (ret == 1)
				ftm_werr_code = 0;
			else
				ftm_werr_code = ret;
			ftm_raw_trigger_read(qca199x_dev);
		}
		/* Write to Register */
		if (count == 2) {
			ftm_raw_write_mode = 1;
			ret = i2c_master_send(qca199x_dev->client, tmp, count);
			if (ret == 2)
				ftm_werr_code = 0;
			else
				ftm_werr_code = ret;
			ftm_raw_trigger_read(qca199x_dev);
		}
	} else {
		/* NORMAL NCI behaviour - NB :
		We can be in FTM mode here also */
		ret = i2c_master_send(qca199x_dev->client, tmp, count);
	}
	if (ret != count) {
		dev_err(&qca199x_dev->client->dev,
		"NFC: failed to write %d\n", ret);
		ret = -EIO;
	}
	mutex_unlock(&qca199x_dev->read_mutex);

	/* If we detect a Region2 command prior to power-down */
	if ((tmp[0] == 0x2F) && (tmp[1] == 0x01) && (tmp[2] == 0x02) &&
		(tmp[3] == 0x08) && (tmp[4] == 0x00)) {
		region2_sent = true;
	}
	dev_dbg(&qca199x_dev->client->dev, "%s : NfcNciTx %x %x %x\n",
			__func__, tmp[0], tmp[1], tmp[2]);
	return ret;
}

static int nfc_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	struct qca199x_dev *qca199x_dev = container_of(filp->private_data,
							struct qca199x_dev,
							qca199x_device);

	filp->private_data = qca199x_dev;
	qca199x_init_stat(qca199x_dev);
	/* Enable interrupts from NFCC NFC_INT new NCI data available */
	qca199x_enable_irq(qca199x_dev);

	if ((!strcmp(qca199x_dev->clk_src_name, "GPCLK")) ||
		(!strcmp(qca199x_dev->clk_src_name, "GPCLK2"))) {
		/* Enable interrupts from NFCC CLK_REQ */
		qca199x_enable_irq_clk_req(qca199x_dev);
	}
	dev_dbg(&qca199x_dev->client->dev,
			"%d,%d\n", imajor(inode), iminor(inode));
	return ret;
}

/*
 * Wake/Sleep Mode
 */
int nfcc_wake(int level, struct file *filp)
{
	int r = 0;
	int time_taken = 0;
	unsigned char raw_nci_sleep[] = {0x2F, 0x03, 0x00};
	/* Change slave address to 0xE */
	unsigned char raw_nci_wake[]  = {0x10, 0x0F};
	unsigned short	slave_addr	=	0xE;
	unsigned short	curr_addr;
	unsigned char wake_status	= WAKE_REG;
	struct qca199x_dev *qca199x_dev = filp->private_data;

	dev_dbg(&qca199x_dev->client->dev, "nfcc_wake: %s: info: %p\n",
			__func__, qca199x_dev);

	if (level == NFCC_SLEEP) {
		r = i2c_master_send(qca199x_dev->client, &raw_nci_sleep[0],
						sizeof(raw_nci_sleep));

		if (r != sizeof(raw_nci_sleep))
			return -EMSGSIZE;
		qca199x_dev->state = NFCC_STATE_NORMAL_SLEEP;
	} else {
		curr_addr = qca199x_dev->client->addr;
		qca199x_dev->client->addr = slave_addr;
		r = nfc_i2c_write(qca199x_dev->client, &raw_nci_wake[0],
						sizeof(raw_nci_wake));
		do {
			wake_status = WAKE_REG;
			r = nfc_i2c_write(qca199x_dev->client, &wake_status, 1);
			/*
			 * NFCC chip needs to be at least
			 * 10usec high before make it low
			 */
			usleep_range(10, 15);
			r = i2c_master_recv(qca199x_dev->client, &wake_status,
						sizeof(wake_status));

			time_taken++;
			if ((wake_status & NCI_WAKE) != 0)
				/* NFCC wakeup time is between 0.5 and .52 ms */
				usleep_range(500, 520);

		} while ((wake_status & NCI_WAKE)
				&& (time_taken < WAKE_TIMEOUT));
		/* Restore original NFCC slave I2C address */
		if (time_taken >= WAKE_TIMEOUT)
			dev_err(&qca199x_dev->client->dev,
			"nfc_ioctl_nfcc_version : TIMED OUT to get WAKEUP bit\n");

		qca199x_dev->client->addr = curr_addr;
		if (r != sizeof(wake_status))
			return -EMSGSIZE;
		qca199x_dev->state = NFCC_STATE_NORMAL_WAKE;
	}

	return r;
}

/*
 * Inside nfc_ioctl_power_states
 *
 * @brief   ioctl functions
 *
 *
 * Device control
 * remove control via ioctl
 * (arg = 0): NFC_DISABLE   GPIO = 0
 * (arg = 1): NFC_DISABLE   GPIO = 1
 *  NOT USED   (arg = 2): FW_DL GPIO = 0
 *  NOT USED   (arg = 3): FW_DL GPIO = 1
 * (arg = 4): NFCC_WAKE  = 1
 * (arg = 5): NFCC_WAKE  = 0
 *
 *
 */
int nfc_ioctl_power_states(struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	int r = 0;
	struct qca199x_dev *qca199x_dev = filp->private_data;

	if (arg == 0) {
		r = qca199x_clock_select(qca199x_dev);
		if (r < 0)
			goto err_req;
		dev_dbg(&qca199x_dev->client->dev, "gpio_set_value disable: %s: info: %p\n",
			__func__, qca199x_dev);
		gpio_set_value(qca199x_dev->dis_gpio, 0);
		usleep(1000);
	} else if (arg == 1) {
		/*
		 * We are attempting a hardware reset so let us disable
		 * interrupts to avoid spurious notifications to upper
		 * layers.
		 */
		qca199x_disable_irq(qca199x_dev);
		/* Deselection of clock */
		r = qca199x_clock_deselect(qca199x_dev);
		if (r < 0)
			goto err_req;
		/*
		 * Also, set flag for initial NCI write following resetas
		 * may wish to do some house keeping. Ensure no pending
		 * messages in NFCC buffers which may be wrongly
		 * construed as response to initial message
		 */
		qca199x_dev->sent_first_nci_write = false;
		dev_dbg(&qca199x_dev->client->dev, "gpio_set_value enable: %s: info: %p\n",
			__func__, qca199x_dev);
		gpio_set_value(qca199x_dev->dis_gpio, 1);
		/*nfcc needs atleast 100ms for the chip to power cycle*/
		msleep(100);
	} else if (arg == 2) {
		mutex_lock(&qca199x_dev->read_mutex);
		dev_dbg(&qca199x_dev->client->dev, "before nfcc_initialise: %s: info: %p\n",
			__func__, qca199x_dev);
		r = nfcc_initialise(qca199x_dev->client, 0xE, qca199x_dev);
		dev_dbg(&qca199x_dev->client->dev, "after nfcc_initialise: %s: info: %p\n",
			__func__, qca199x_dev);
		/* Also reset first NCI write */
		qca199x_dev->sent_first_nci_write = false;
		mutex_unlock(&qca199x_dev->read_mutex);
		if (r) {
			dev_err(&qca199x_dev->client->dev,
				"nfc_ioctl_power_states: request nfcc initialise failed\n");
			goto err_req;
		}
	} else if (arg == 3) {
		msleep(20);
	} else if (arg == 4) {
		mutex_lock(&qca199x_dev->read_mutex);
		nfcc_wake(NFCC_WAKE, filp);
		dev_dbg(&qca199x_dev->client->dev, "nfcc wake: %s: info: %p\n",
			__func__, qca199x_dev);
		mutex_unlock(&qca199x_dev->read_mutex);
	} else if (arg == 5) {
		nfcc_wake(NFCC_SLEEP, filp);
	} else {
		r = -ENOIOCTLCMD;
	}

err_req:
	return r;
}

/*
 * Inside nfc_ioctl_nfcc_mode
 *
 * @brief   nfc_ioctl_nfcc_mode
 *
 * (arg = 0) ; NORMAL_MODE - Standard mode, unsolicited read behaviour
 * (arg = 1) ; SOLICITED_MODE - As above but reads are solicited from User Land
 * (arg = 2) ; UNSOLICITED_FTM_RAW MODE - NORMAL_MODE but messages from FTM and
 *             not NCI Host.
 * (arg = 2) ; SOLICITED_FTM_RAW_MODE - As SOLICITED_MODE but messages from FTM
 *             and not NCI Host.
 *
 *
 *
 */
int nfc_ioctl_nfcc_mode(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;

	static unsigned short nci_addr;
	struct qca199x_dev *qca199x_dev = filp->private_data;
	struct qca199x_platform_data *platform_data;

	platform_data = qca199x_dev->client->dev.platform_data;

	if (arg == 0) {
		device_mode.handle_flavour = UNSOLICITED_MODE;
		qca199x_dev->client->addr = NCI_I2C_SLAVE;
		/* enable interrupts again */
		qca199x_enable_irq(qca199x_dev);
	} else if (arg == 1) {
		device_mode.handle_flavour = SOLICITED_MODE;
		qca199x_dev->client->addr = qca199x_dev->client->addr;
		/* enable interrupts again */
		qca199x_enable_irq(qca199x_dev);
	} else if (arg == 2) {
		device_mode.handle_flavour = UNSOLICITED_FTM_RAW_MODE;
		nci_addr = qca199x_dev->client->addr;
		/* replace with new client slave address*/
		qca199x_dev->client->addr = 0xE;
		/* We also need to disable interrupts */
		qca199x_disable_irq(qca199x_dev);
	} else if (arg == 3) {
		device_mode.handle_flavour = SOLICITED_FTM_RAW_MODE;
		nci_addr = qca199x_dev->client->addr;
		/* replace with new client slave address*/
		qca199x_dev->client->addr = 0xE;
		/* We also need to disable interrupts */
		qca199x_disable_irq(qca199x_dev);
	} else {
		device_mode.handle_flavour = UNSOLICITED_MODE;
		qca199x_dev->client->addr = NCI_I2C_SLAVE;
	}
	return retval;
}

/*
 * Inside nfc_ioctl_nfcc_version
 *
 * @brief   nfc_ioctl_nfcc_version
 *
 *
 */
int nfc_ioctl_nfcc_version(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	int r = 0;
	int	time_taken	= 0;
	unsigned short	slave_addr	=	0xE;
	unsigned short	curr_addr;
	unsigned char	raw_nci_wake[]			= {0x10, 0x0F};
	unsigned char	raw_nci_read;
	unsigned char raw_chip_version_addr		= 0x00;
	unsigned char raw_chip_rev_id_addr		= 0x9C;
	unsigned char raw_chip_version			= 0xFF;

	struct qca199x_dev *qca199x_dev = filp->private_data;
	struct qca199x_platform_data *platform_data;

	platform_data = qca199x_dev->client->dev.platform_data;

	/*
	 * Always wake up chip when reading 0x9C, otherwise this
	 * register is not updated
	 */
	curr_addr = qca199x_dev->client->addr;
	qca199x_dev->client->addr = slave_addr;
	r = nfc_i2c_write(qca199x_dev->client, &raw_nci_wake[0],
						sizeof(raw_nci_wake));

	if (r != sizeof(raw_nci_wake))
		dev_err(&qca199x_dev->client->dev,
		"nfc_ioctl_nfcc_version : failed to send wake command\n");

	/*
	 * After placing the NFCC to sleep by a PROP
	 * SLEEP NCI msg (2F 03) and we need to wake
	 * it back up to obtain some information (by
	 * setting the wake bit).We need to determine
	 * when it has in actual fact woken before we
	 * can read the required data. We do that by
	 * reading back & testing if that wake bit has
	 * been cleared.
	 */
	do {
		raw_nci_read = 0x10;
		r = nfc_i2c_write(qca199x_dev->client, &raw_nci_read, 1);
		/*
		 * NFCC chip needs to be at least
		 * 10usec high before make it low
		 */
		usleep_range(10, 15);

		r = i2c_master_recv(qca199x_dev->client, &raw_nci_read,
						sizeof(raw_nci_read));

		if ((raw_nci_read & NCI_WAKE) != 0)
			/* NFCC wakeup time is between 0.5 and .52 ms */
			usleep_range(500, 520);

		time_taken++;

	} while ((raw_nci_read & NCI_WAKE)
			&& (time_taken < WAKE_TIMEOUT));

	if (time_taken < WAKE_TIMEOUT)
		qca199x_dev->state = NFCC_STATE_NORMAL_WAKE;
	else
		dev_err(&qca199x_dev->client->dev,
		"nfc_ioctl_nfcc_version : TIMED OUT to get WAKEUP bit\n");


	if (r != 1) {
		/*
		 * r < 0 indicates an error, maybe chip isn't
		 * up yet.What should we do??? r = 0 indicates
		 * nothing read, maybe chip isn't up yet. (should
		 * not happen) r > 1 indicates too many bytes read,
		 * maybe ?(should not happen)
		 */
		dev_err(&qca199x_dev->client->dev,
		"nfc_ioctl_nfcc_version : i2c error %d\n", r);
	}

	if (arg == 0) {
		r = nfc_i2c_write(qca199x_dev->client,
				&raw_chip_version_addr, 1);
	} else if (arg == 1) {
		r = nfc_i2c_write(qca199x_dev->client,
				&raw_chip_rev_id_addr, 1);
	} else {
		/* Restore original NFCC slave I2C address */
		qca199x_dev->client->addr = curr_addr;
		return -EINVAL;
	}

	if (r < 0) {
		/* Restore original NFCC slave I2C address */
		qca199x_dev->client->addr = curr_addr;
		dev_err(&qca199x_dev->client->dev,
			"NFCC_INVALID_CHIP_VERSION : i2c write fail\n");
		return -EIO;
	}

	usleep(10);
	r = i2c_master_recv(qca199x_dev->client, &raw_chip_version, 1);

	/* Restore original NFCC slave I2C address */
	qca199x_dev->client->addr = curr_addr;
	return raw_chip_version;
}

/*
 * Inside nfc_ioctl_kernel_logging
 *
 * @brief   nfc_ioctl_kernel_logging
 *
 * (arg = 0) ; NO_LOGGING
 * (arg = 1) ; COMMS_LOGGING - BASIC LOGGING - Mainly just comms over I2C
 * (arg = 2) ; FULL_LOGGING - ENABLE ALL  - DBG messages for handlers etc.
 *           ; ! Be aware as amount of logging could impact behaviour !
 *
 *
 */
int nfc_ioctl_kernel_logging(unsigned long arg,  struct file *filp)
{
	int retval = 0;
	struct qca199x_dev *qca199x_dev = container_of(filp->private_data,
							   struct qca199x_dev,
							   qca199x_device);
	if (arg == 0) {
		dev_dbg(&qca199x_dev->client->dev,
		"nfc_ioctl_kernel_logging : level = NO_LOGGING\n");
		logging_level = 0;
	} else if (arg == 1) {
		dev_dbg(&qca199x_dev->client->dev,
		"nfc_ioctl_kernel_logging: level = COMMS_LOGGING only\n");
		logging_level = 1;
	} else if (arg == 2) {
		dev_dbg(&qca199x_dev->client->dev,
		"nfc_ioctl_kernel_logging: level = FULL_LOGGING\n");
		logging_level = 2;
	}
	return retval;
}

/*
 * Inside nfc_ioctl_core_reset_ntf
 *
 * @brief   nfc_ioctl_core_reset_ntf
 *
 * Allows callers to determine if a CORE_RESET_NTF has arrived
 *
 * Returns the value of variable core_reset_ntf
 *
 */
int nfc_ioctl_core_reset_ntf(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	struct qca199x_dev *qca199x_dev = filp->private_data;
	dev_dbg(&qca199x_dev->client->dev,
		"nfc_ioctl_core_reset_ntf: returning = %d\n",
		qca199x_dev->core_reset_ntf);
	return qca199x_dev->core_reset_ntf;
}

static long nfc_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg)
{
	int r = 0;

	switch (cmd) {

	case NFC_SET_PWR:
		nfc_ioctl_power_states(pfile, cmd, arg);
		break;
	case NFCC_MODE:
		nfc_ioctl_nfcc_mode(pfile, cmd, arg);
		break;
	case NFCC_VERSION:
		r = nfc_ioctl_nfcc_version(pfile, cmd, arg);
		break;
	case NFC_KERNEL_LOGGING_MODE:
		nfc_ioctl_kernel_logging(arg, pfile);
		break;
	case SET_RX_BLOCK:
		break;
	case SET_EMULATOR_TEST_POINT:
		break;
	case NFCC_INITIAL_CORE_RESET_NTF:
		r = nfc_ioctl_core_reset_ntf(pfile, cmd, arg);
		break;
	default:
		r = -ENOIOCTLCMD;
	}
	return r;
}

static const struct file_operations nfc_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.poll  = nfc_poll,
	.read  = nfc_read,
	.write = nfc_write,
	.open = nfc_open,
	.unlocked_ioctl = nfc_ioctl
};

void dumpqca1990(struct i2c_client *client)
{
	int r = 0;
	int i = 0;
	unsigned char raw_reg_rd = {0x0};
	unsigned short temp_addr;

	temp_addr = client->addr;
	client->addr = 0x0E;

	for (i = 0; i < MAX_QCA_REG; i++) {
		raw_reg_rd = i;
		if (((i >= 0x0) && (i < 0x4)) || ((i > 0x7) && (i < 0xA)) ||
		((i > 0xF) && (i < 0x12)) || ((i > 0x39) && (i < 0x4d)) ||
		((i > 0x69) && (i < 0x74)) || (i == 0x18) || (i == 0x30) ||
		(i == 0x58)) {
			r = nfc_i2c_write(client, &raw_reg_rd, 1);
			msleep(20);
			r = i2c_master_recv(client, &raw_reg_rd, 1);
		}
	}
	client->addr = temp_addr;
}

static int nfc_i2c_write(struct i2c_client *client, u8 *buf, int len)
{
	int r;

	r = i2c_master_send(client, buf, len);
	dev_dbg(&client->dev, "send: %d\n", r);
	if (r == -EREMOTEIO) { /* Retry, chip was in standby */
		usleep_range(6000, 10000);
		r = i2c_master_send(client, buf, len);
		dev_dbg(&client->dev, "send2: %d\n", r);
	}
	if (r != len)
		return -EREMOTEIO;

	return r;
}

/* Check for availability of qca199x_ NFC controller hardware */
static int nfcc_hw_check(struct i2c_client *client, unsigned short curr_addr)
{
	int r = 0;
	unsigned char buf = 0;

	client->addr = curr_addr;
	/* Set-up Addr 0. No data written */
	r = i2c_master_send(client, &buf, 1);
	if (r < 0)
		goto err_presence_check;
	buf = 0;
	/* Read back from Addr 0 */
	r = i2c_master_recv(client, &buf, 1);
	if (r < 0)
		goto err_presence_check;

	r = 0;
	return r;

err_presence_check:
	r = -ENXIO;
	dev_err(&client->dev,
		"nfc-nci nfcc_presence check - no NFCC available\n");
	return r;
}
/* Initialise qca199x_ NFC controller hardware */
static int nfcc_initialise(struct i2c_client *client, unsigned short curr_addr,
				struct qca199x_dev *qca199x_dev)
{
	int r = 0;
	unsigned char raw_1p8_CONTROL_011[]	= {0x11, XTAL_CLOCK};
	unsigned char raw_1P8_CONTROL_010[]	= {0x10, PWR_EN};
	unsigned char raw_1P8_X0_0B0[]		= {0xB0, (FREQ_SEL)};
	unsigned char raw_slave1[]		= {0x09, NCI_I2C_SLAVE};
	unsigned char raw_slave2[]		= {0x8, 0x10};
	unsigned char raw_s73[]			= {0x73, 0x02};
	unsigned char raw_slave1_rd		= {0x0};
	unsigned char raw_1P8_PAD_CFG_CLK_REQ[]	= {0xA5, 0x1};
	unsigned char raw_1P8_PAD_CFG_PWR_REQ[]	= {0xA7, 0x1};
	unsigned char buf = 0;
	bool core_reset_completed = false;
	unsigned char rsp[6];
	int time_taken = 0;
	int ret = 0;

	client->addr = curr_addr;
	qca199x_dev->core_reset_ntf = DEFAULT_INITIAL_CORE_RESET_NTF;
	r = i2c_master_send(client, &buf, 1);
	if (r < 0)
		goto err_init;

	buf = 0;
	r = i2c_master_recv(client, &buf, 1);
	if (r < 0)
		goto err_init;

	RAW(s73, 0x02);

	r = nfc_i2c_write(client, &raw_s73[0], sizeof(raw_s73));
	if (r < 0)
		goto err_init;

	usleep(1000);
	RAW(1p8_CONTROL_011, XTAL_CLOCK | 0x01);

	r = nfc_i2c_write(client, &raw_1p8_CONTROL_011[0],
					sizeof(raw_1p8_CONTROL_011));
	if (r < 0)
		goto err_init;

	usleep(1000);
	RAW(1P8_CONTROL_010, (0x8));
	r = nfc_i2c_write(client, &raw_1P8_CONTROL_010[0],
					sizeof(raw_1P8_CONTROL_010));
	if (r < 0)
		goto err_init;

	usleep(10000);  /* 10ms wait */
	RAW(1P8_CONTROL_010, (0xC));
	r = nfc_i2c_write(client, &raw_1P8_CONTROL_010[0],
				sizeof(raw_1P8_CONTROL_010));
	if (r < 0)
		goto err_init;

	usleep(100);  /* 100uS wait */
	RAW(1P8_X0_0B0, (FREQ_SEL_19));
	r = nfc_i2c_write(client, &raw_1P8_X0_0B0[0],
					sizeof(raw_1P8_X0_0B0));
	if (r < 0)
		goto err_init;

	usleep(1000);

	/* PWR_EN = 1 */
	RAW(1P8_CONTROL_010, (0xd));
	r = nfc_i2c_write(client, &raw_1P8_CONTROL_010[0],
					sizeof(raw_1P8_CONTROL_010));
	if (r < 0)
		goto err_init;


	usleep(20000);  /* 20ms wait */
	/* LS_EN = 1 */
	RAW(1P8_CONTROL_010, 0xF);
	r = nfc_i2c_write(client, &raw_1P8_CONTROL_010[0],
					sizeof(raw_1P8_CONTROL_010));
	if (r < 0)
		goto err_init;

	usleep(20000);  /* 20ms wait */

	/* Enable the PMIC clock */
	RAW(1P8_PAD_CFG_CLK_REQ, (0x1));
	r = nfc_i2c_write(client, &raw_1P8_PAD_CFG_CLK_REQ[0],
				  sizeof(raw_1P8_PAD_CFG_CLK_REQ));
	if (r < 0)
		goto err_init;

	usleep(1000);

	RAW(1P8_PAD_CFG_PWR_REQ, (0x1));
	r = nfc_i2c_write(client, &raw_1P8_PAD_CFG_PWR_REQ[0],
				  sizeof(raw_1P8_PAD_CFG_PWR_REQ));
	if (r < 0)
		goto err_init;

	usleep(1000);

	RAW(slave2, 0x10);
	r = nfc_i2c_write(client, &raw_slave2[0], sizeof(raw_slave2));
	if (r < 0)
		goto err_init;

	usleep(1000);

	RAW(slave1, NCI_I2C_SLAVE);
	r = nfc_i2c_write(client, &raw_slave1[0], sizeof(raw_slave1));
	if (r < 0)
		goto err_init;

	usleep(1000);

	/* QCA199x NFCC CPU should now boot... */
	r = i2c_master_recv(client, &raw_slave1_rd, 1);
	/* Talk on NCI slave address NCI_I2C_SLAVE 0x2C*/
	client->addr = NCI_I2C_SLAVE;

	/*
	 * Start with small delay and then we will poll until we
	 * get a core reset notification - This is time for chip
	 * & NFCC controller to come-up.
	 */
	usleep(15000); /* 15 ms */

	do {
		ret = i2c_master_recv(client, rsp, 5);
		/* Found core reset notification */
		if (((rsp[0] == CORE_RESET_RSP_GID) &&
			(rsp[1] == CORE_RESET_OID) &&
			(rsp[2] == CORE_RST_NTF_LENGTH))
				|| time_taken == NTF_TIMEOUT) {
			dev_dbg(&client->dev,
				"NFC core reset recevd: %s: info: %p\n",
				__func__, client);
			core_reset_completed = true;
		} else {
		  usleep(2000);  /* 2ms sleep before retry */
		}
		time_taken++;
	} while (!core_reset_completed);
	if (time_taken == NTF_TIMEOUT)
		qca199x_dev->core_reset_ntf = TIMEDOUT_INITIAL_CORE_RESET_NTF;
	else
		qca199x_dev->core_reset_ntf = ARRIVED_INITIAL_CORE_RESET_NTF;

	r = 0;
	return r;
err_init:
	r = 1;
	dev_err(&client->dev,
		"nfc-nci nfcc_initialise: failed. Check Hardware\n");
	return r;
}
/*
	Routine to Select clocks
*/
static int qca199x_clock_select(struct qca199x_dev *qca199x_dev)
{
	int r = 0;

	if (!strcmp(qca199x_dev->clk_src_name, "BBCLK2")) {
		qca199x_dev->s_clk  =
			clk_get(&qca199x_dev->client->dev, "ref_clk");
		if (qca199x_dev->s_clk == NULL)
			goto err_invalid_dis_gpio;
	} else if (!strcmp(qca199x_dev->clk_src_name, "RFCLK3")) {
		qca199x_dev->s_clk  =
			clk_get(&qca199x_dev->client->dev, "ref_clk_rf");
		if (qca199x_dev->s_clk == NULL)
			goto err_invalid_dis_gpio;
	} else if (!strcmp(qca199x_dev->clk_src_name, "GPCLK")) {
		if (gpio_is_valid(qca199x_dev->clk_src_gpio)) {
			qca199x_dev->s_clk  =
				clk_get(&qca199x_dev->client->dev, "core_clk");
			if (qca199x_dev->s_clk == NULL)
				goto err_invalid_dis_gpio;
		} else {
			goto err_invalid_dis_gpio;
		}
	} else if (!strcmp(qca199x_dev->clk_src_name, "GPCLK2")) {
		if (gpio_is_valid(qca199x_dev->clk_src_gpio)) {
			qca199x_dev->s_clk  =
				clk_get(&qca199x_dev->client->dev, "core_clk_pvt");
			if (qca199x_dev->s_clk == NULL)
				goto err_invalid_dis_gpio;
		} else {
			goto err_invalid_dis_gpio;
		}
	} else {
		qca199x_dev->s_clk = NULL;
		goto err_invalid_dis_gpio;
	}
	if (qca199x_dev->clk_run == false) {
		/* Set clock rate */
		if ((!strcmp(qca199x_dev->clk_src_name, "GPCLK")) ||
			(!strcmp(qca199x_dev->clk_src_name, "GPCLK2"))) {
			r = clk_set_rate(qca199x_dev->s_clk, NFC_RF_CLK_FREQ);
			if (r)
				goto err_invalid_clk;
		}

		r = clk_prepare_enable(qca199x_dev->s_clk);
		if (r)
			goto err_invalid_clk;
		qca199x_dev->clk_run = true;
	}
	r = 0;
	return r;

err_invalid_clk:
	r = -1;
	return r;
err_invalid_dis_gpio:
	r = -2;
	return r;
}
/*
	Routine to De-Select clocks
*/

static int qca199x_clock_deselect(struct qca199x_dev *qca199x_dev)
{
	int r = -1;
	if (qca199x_dev->s_clk != NULL) {
		if (qca199x_dev->clk_run == true) {
			clk_disable_unprepare(qca199x_dev->s_clk);
			qca199x_dev->clk_run = false;
		}
		return 0;
	}
	return r;
}

static int nfc_parse_dt(struct device *dev, struct qca199x_platform_data *pdata)
{
	int r = 0;
	struct device_node *np = dev->of_node;

	r = of_property_read_u32(np, "reg", &pdata->reg);
	if (r)
		return -EINVAL;

	pdata->dis_gpio = of_get_named_gpio(np, "qcom,dis-gpio", 0);
	if ((!gpio_is_valid(pdata->dis_gpio)))
		return -EINVAL;
	disable_ctrl = pdata->dis_gpio;

	pdata->irq_gpio = of_get_named_gpio(np, "qcom,irq-gpio", 0);
	if ((!gpio_is_valid(pdata->irq_gpio)))
		return -EINVAL;

	r = of_property_read_string(np, "qcom,clk-src", &pdata->clk_src_name);

	if (strcmp(pdata->clk_src_name, "GPCLK2")) {
		pdata->clkreq_gpio = of_get_named_gpio(np, "qcom,clk-gpio", 0);
	}

	if ((!strcmp(pdata->clk_src_name, "GPCLK")) ||
	    (!strcmp(pdata->clk_src_name, "GPCLK2"))) {
			pdata->clk_src_gpio = of_get_named_gpio(np,
					"qcom,clk-src-gpio", 0);
			if ((!gpio_is_valid(pdata->clk_src_gpio)))
				return -EINVAL;
			pdata->irq_gpio_clk_req = of_get_named_gpio(np,
					"qcom,clk-req-gpio", 0);
			if ((!gpio_is_valid(pdata->irq_gpio_clk_req)))
				return -EINVAL;
	}

	if (r)
		return -EINVAL;
	return r;
}

static int qca199x_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int r = 0;
	int irqn = 0;
	struct qca199x_platform_data *platform_data;
	struct qca199x_dev *qca199x_dev;

	if (client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev,
			sizeof(struct qca199x_platform_data), GFP_KERNEL);
		if (!platform_data) {
			dev_err(&client->dev,
			"nfc-nci probe: Failed to allocate memory\n");
			return -ENOMEM;
		}
		r = nfc_parse_dt(&client->dev, platform_data);
		if (r)
			return r;
	} else {
		platform_data = client->dev.platform_data;
	}
	if (!platform_data)
		return -EINVAL;
	dev_dbg(&client->dev,
		"nfc-nci probe: %s, inside nfc-nci flags = %x\n",
		__func__, client->flags);
	if (platform_data == NULL) {
		dev_err(&client->dev, "nfc-nci probe: failed\n");
		return -ENODEV;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "nfc-nci probe: need I2C_FUNC_I2C\n");
		return -ENODEV;
	}
	qca199x_dev = kzalloc(sizeof(*qca199x_dev), GFP_KERNEL);
	if (qca199x_dev == NULL) {
		dev_err(&client->dev,
		"nfc-nci probe: failed to allocate memory for module data\n");
		return -ENOMEM;
	}
	qca199x_dev->client = client;

	/*
	 * To be efficient we need to test whether nfcc hardware is physically
	 * present before attempting further hardware initialisation.
	 * For this we need to be sure the device is in ULPM state by
	 * setting disable line low early on.
	 *
	 */


	if (gpio_is_valid(platform_data->dis_gpio)) {
		r = gpio_request(platform_data->dis_gpio, "nfc_reset_gpio");
		if (r) {
			dev_err(&client->dev,
			"NFC: unable to request gpio [%d]\n",
				platform_data->dis_gpio);
			goto err_free_dev;
		}
		r = gpio_direction_output(platform_data->dis_gpio, 1);
		if (r) {
			dev_err(&client->dev,
				"NFC: unable to set direction for gpio [%d]\n",
					platform_data->dis_gpio);
			goto err_dis_gpio;
		}
	} else {
		dev_err(&client->dev, "dis gpio not provided\n");
		goto err_free_dev;
	}

	/* Guarantee that the NFCC starts in a clean state. */
	gpio_set_value(platform_data->dis_gpio, 1);/* HPD */
	usleep(200);
	gpio_set_value(platform_data->dis_gpio, 0);/* ULPM */
	usleep(200);

	r = nfcc_hw_check(client, platform_data->reg);
	if (r) {
		/* We don't think there is hardware but just in case HPD */
		gpio_set_value(platform_data->dis_gpio, 1);
		goto err_dis_gpio;
	}

	if (gpio_is_valid(platform_data->irq_gpio)) {
		r = gpio_request(platform_data->irq_gpio, "nfc_irq_gpio");
		if (r) {
			dev_err(&client->dev, "unable to request gpio [%d]\n",
				platform_data->irq_gpio);
			goto err_dis_gpio;
		}
		r = gpio_direction_input(platform_data->irq_gpio);
		if (r) {

			dev_err(&client->dev,
			"unable to set direction for gpio [%d]\n",
				platform_data->irq_gpio);
			goto err_irq;
		}
		irqn = gpio_to_irq(platform_data->irq_gpio);
		if (irqn < 0) {
			r = irqn;
			goto err_irq;
		}
		client->irq = irqn;

	} else {
		dev_err(&client->dev, "irq gpio not provided\n");
		goto err_dis_gpio;
	}
	/* Interrupt from NFCC CLK_REQ to handle REF_CLK
		o/p gating/selection */
	if ((!strcmp(platform_data->clk_src_name, "GPCLK")) ||
		(!strcmp(platform_data->clk_src_name, "GPCLK2"))) {
		if (gpio_is_valid(platform_data->irq_gpio_clk_req)) {
			r = gpio_request(platform_data->irq_gpio_clk_req,
				"nfc_irq_gpio_clk_en");
			if (r) {
				dev_err(&client->dev, "unable to request CLK_EN gpio [%d]\n",
					platform_data->irq_gpio_clk_req);
				goto err_irq;
			}
			r = gpio_direction_input(
					platform_data->irq_gpio_clk_req);
			if (r) {
				dev_err(&client->dev,
					"unable to set direction for CLK_EN gpio [%d]\n",
					platform_data->irq_gpio_clk_req);
				goto err_irq_clk;
			}
			gpio_to_irq(0);
			irqn = gpio_to_irq(platform_data->irq_gpio_clk_req);
			if (irqn < 0) {
				r = irqn;
				goto err_irq_clk;
			}
			platform_data->clk_req_irq_num = irqn;
		} else {
			dev_err(&client->dev, "irq CLK_EN gpio not provided\n");
			goto err_irq;
		}
	}
	/* Get the clock source name and gpio from from Device Tree */
	qca199x_dev->clk_src_name = platform_data->clk_src_name;
	qca199x_dev->clk_src_gpio = platform_data->clk_src_gpio;
	qca199x_dev->clk_run = false;
	r = qca199x_clock_select(qca199x_dev);
	if (r != 0) {
		if (r == -1)
			goto err_clk;
		else
			goto err_irq_clk;
	}

	if (strcmp(platform_data->clk_src_name, "GPCLK2")) {
		if (gpio_is_valid(platform_data->clkreq_gpio)) {
			r = gpio_request(platform_data->clkreq_gpio,
				"nfc_clkreq_gpio");
			if (r) {
				dev_err(&client->dev, "unable to request gpio [%d]\n",
						platform_data->clkreq_gpio);
				goto err_clkreq_gpio;
			}
			r = gpio_direction_input(platform_data->clkreq_gpio);
			if (r) {
				dev_err(&client->dev,
						"unable to set direction for gpio [%d]\n",
						platform_data->clkreq_gpio);
				goto err_clkreq_gpio;
			}
		} else {
			dev_err(&client->dev, "clkreq gpio not provided\n");
			goto err_clk;
		}
		qca199x_dev->clkreq_gpio = platform_data->clkreq_gpio;
	}
	qca199x_dev->dis_gpio = platform_data->dis_gpio;
	qca199x_dev->irq_gpio = platform_data->irq_gpio;
	if ((!strcmp(platform_data->clk_src_name, "GPCLK")) ||
		(!strcmp(platform_data->clk_src_name, "GPCLK2"))) {
			qca199x_dev->irq_gpio_clk_req	=
						platform_data->irq_gpio_clk_req;
			qca199x_dev->clk_req_irq_num		=
						platform_data->clk_req_irq_num;
	}

	/* init mutex and queues */
	init_waitqueue_head(&qca199x_dev->read_wq);
	mutex_init(&qca199x_dev->read_mutex);
	spin_lock_init(&qca199x_dev->irq_enabled_lock);
	spin_lock_init(&qca199x_dev->irq_enabled_lock_clk_req);

	qca199x_dev->qca199x_device.minor = MISC_DYNAMIC_MINOR;
	qca199x_dev->qca199x_device.name = "nfc-nci";
	qca199x_dev->qca199x_device.fops = &nfc_dev_fops;

	r = misc_register(&qca199x_dev->qca199x_device);
	if (r) {
		dev_err(&client->dev, "misc_register failed\n");
		goto err_misc_register;
	}


	/*
	 * Reboot the NFCC now that all resources are ready
	 *
	 * The NFCC takes time to transition between power states.
	 * We wait 20uS for the NFCC to shutdown. (HPD)
	 * We wait 100uS for the NFCC to boot into ULPM.
	 */
	gpio_set_value(platform_data->dis_gpio, 1);/* HPD */
	msleep(20);
	gpio_set_value(platform_data->dis_gpio, 0);/* ULPM */
	msleep(100);


	/* Here we perform a second presence check. */
	r = nfcc_hw_check(client, platform_data->reg);
	if (r) {
		/* We don't think there is hardware but just in case HPD */
		gpio_set_value(platform_data->dis_gpio, 1);
		goto err_nfcc_not_present;
	}

	logging_level = 0;
	/*
	 * request irq.  The irq is set whenever the chip has data available
	 * for reading.  It is cleared when all data has been read.
	 */
	device_mode.handle_flavour = UNSOLICITED_MODE;
	/* NFC_INT IRQ */
	qca199x_dev->irq_enabled = true;
	r = request_irq(client->irq, qca199x_dev_irq_handler,
			  IRQF_TRIGGER_RISING, client->name, qca199x_dev);
	if (r) {
		dev_err(&client->dev, "nfc-nci probe: request_irq failed\n");
		goto err_request_irq_failed;
	}
	qca199x_disable_irq(qca199x_dev);
	/* CLK_REQ IRQ */
	if ((!strcmp(platform_data->clk_src_name, "GPCLK")) ||
		(!strcmp(platform_data->clk_src_name, "GPCLK2"))) {
		r = request_irq(qca199x_dev->clk_req_irq_num,
				qca199x_dev_irq_handler_clk_req,
				(IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING),
						client->name, qca199x_dev);
		if (r) {
			dev_err(&client->dev,
			"nfc-nci probe: request_irq failed. irq no = %d\n, main irq =  %d",
				qca199x_dev->clk_req_irq_num, client->irq);
			goto err_request_irq_failed;
		}
		qca199x_dev->irq_enabled_clk_req = true;
		qca199x_disable_irq_clk_req(qca199x_dev);


		qca199x_dev->my_wq =
			create_singlethread_workqueue("qca1990x_CLK_REQ_queue");
		if (!qca199x_dev->my_wq)
			goto err_create_workq;

		INIT_WORK(&qca199x_dev->msm_clock_controll_work,
			clk_req_update);
	}
	i2c_set_clientdata(client, qca199x_dev);
	gpio_set_value(platform_data->dis_gpio, 1);

	/* To keep track if region2 command has been sent to controller */
	region2_sent = false;

	dev_dbg(&client->dev,
	"nfc-nci probe: %s, probing qca1990 exited successfully\n",
		 __func__);
	return 0;

err_create_workq:
	dev_err(&client->dev,
	"nfc-nci probe: %s, work_queue creation failure\n",
		 __func__);
	free_irq(client->irq, qca199x_dev);
err_nfcc_not_present:
err_request_irq_failed:
	misc_deregister(&qca199x_dev->qca199x_device);
err_misc_register:
	mutex_destroy(&qca199x_dev->read_mutex);
err_clkreq_gpio:
	if (strcmp(platform_data->clk_src_name, "GPCLK2"))
		gpio_free(platform_data->clkreq_gpio);
err_clk:
		qca199x_clock_deselect(qca199x_dev);
err_irq_clk:
	if ((!strcmp(platform_data->clk_src_name, "GPCLK")) ||
		(!strcmp(platform_data->clk_src_name, "GPCLK2"))) {
		r = gpio_direction_input(platform_data->irq_gpio_clk_req);
		if (r)
			dev_err(&client->dev, "nfc-nci probe: Unable to set direction\n");
		gpio_free(platform_data->irq_gpio_clk_req);
	}
err_irq:
	gpio_free(platform_data->irq_gpio);
err_dis_gpio:
	gpio_free(platform_data->dis_gpio);
err_free_dev:
	kfree(qca199x_dev);
	return r;
}

static int qca199x_remove(struct i2c_client *client)
{
	struct qca199x_dev *qca199x_dev;

	qca199x_dev = i2c_get_clientdata(client);
	free_irq(client->irq, qca199x_dev);
	misc_deregister(&qca199x_dev->qca199x_device);
	mutex_destroy(&qca199x_dev->read_mutex);
	gpio_free(qca199x_dev->irq_gpio);
	if ((!strcmp(qca199x_dev->clk_src_name, "GPCLK")) ||
		(!strcmp(qca199x_dev->clk_src_name, "GPCLK2"))) {
		gpio_free(qca199x_dev->irq_gpio_clk_req);
	}
	gpio_free(qca199x_dev->dis_gpio);
	if (strcmp(qca199x_dev->clk_src_name, "GPCLK2"))
		gpio_free(qca199x_dev->clkreq_gpio);
	kfree(qca199x_dev);

	return 0;
}

static const struct i2c_device_id qca199x_id[] = {
	{"qca199x-i2c", 0},
	{}
};

static struct i2c_driver qca199x = {
	.id_table = qca199x_id,
	.probe = qca199x_probe,
	.remove = qca199x_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "nfc-nci",
		.of_match_table = msm_match_table,
	},
};


static int nfcc_reboot(struct notifier_block *notifier, unsigned long val,
		      void *v)
{
	/*
	 * Set DISABLE=1 *ONLY* if the NFC service has been disabled.
	 * This will put NFCC into HPD(Hard Power Down) state for power
	 * saving when powering down(Low Batt. or Power off handset)
	 * If user requires NFC and CE mode when powered down(PD) the
	 * middleware puts NFCC into region2 prior to PD. In this case
	 * we DO NOT HPD chip as this will trash Region2 and CE support
	 * when handset is PD.
	 */
	if (region2_sent == false) {
		/* HPD the NFCC */
		gpio_set_value(disable_ctrl, 1);
	}
	return NOTIFY_OK;
}


static struct notifier_block nfcc_notifier = {
	.notifier_call	= nfcc_reboot,
	.next		= NULL,
	.priority	= 0
};

/*
 * module load/unload record keeping
 */
static int __init qca199x_dev_init(void)
{
	int ret;

	ret = register_reboot_notifier(&nfcc_notifier);
	if (ret) {
		pr_err("cannot register reboot notifier (err=%d)\n", ret);
		return ret;
	}
	return i2c_add_driver(&qca199x);
}
module_init(qca199x_dev_init);

static void __exit qca199x_dev_exit(void)
{
	unregister_reboot_notifier(&nfcc_notifier);
	i2c_del_driver(&qca199x);
}
module_exit(qca199x_dev_exit);

MODULE_DESCRIPTION("NFC QCA199x");
MODULE_LICENSE("GPL v2");

