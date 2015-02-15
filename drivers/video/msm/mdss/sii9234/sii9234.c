/* drivers/i2c/chips/sii9234.c - sii9234 optical sensors driver
 *
 * Copyright (C) 2010 HTC, Inc.
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
/*********************************************************************
*  Inculde
**********************************************************************/
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <mach/mhl.h>
#include <mach/debug_display.h>
#include <mach/board.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <mach/cable_detect.h>
#include <linux/async.h>
#include <linux/regulator/consumer.h>
#include <linux/notifier.h>
#include <linux/fb.h>

#include "sii9234.h"
#include "TPI.h"
#include "mhl_defs.h"
#include "../mdss_hdmi_mhl.h"

static DEFINE_MUTEX(mhl_lpm_lock);

/*********************************************************************
  Define & Macro
***********************************************************************/
#define MHL_RCP_KEYEVENT
#define MHL_ISR_TIMEOUT 5

#define MHL_DEBUGFS_TIMEOUT 10
#define MHL_DEBUGFS_AMP 5

#define SII9234_I2C_RETRY_COUNT 2

#define sii_gpio_set_value(pin, val)	\
	do {	\
		if (gpio_cansleep(pin))	\
			gpio_set_value_cansleep(pin, val);	\
		else \
			gpio_set_value(pin, val);	\
	} while(0)
#define sii_gpio_get_value(pin)	\
	gpio_cansleep(pin)?	\
		gpio_get_value_cansleep(pin):	\
		gpio_get_value(pin)
/*********************************************************************
  Type Definitions
***********************************************************************/
typedef struct {
	struct i2c_client *i2c_client;
	struct workqueue_struct *wq;
	struct wake_lock wake_lock;
	void (*mhl_usb_switch)(int);
	void (*mhl_1v2_power)(bool enable);
	int  (*mhl_power_vote)(bool enable);
	int (*enable_5v)(int on);
	int (*mhl_lpm_power)(bool on);
	struct delayed_work init_delay_work;
	struct delayed_work init_complete_work;
	struct delayed_work irq_timeout_work;
	struct delayed_work mhl_on_delay_work;
	struct delayed_work turn_off_5v;
#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
	struct delayed_work detect_charger_work;
#endif
	int reset_pin;
	int intr_pin;
	int ci2ca_pin;
	int irq;
	bool isMHL;
	enum usb_connect_type statMHL;
	struct work_struct mhl_notifier_work;
	mhl_board_params board_params;
	struct regulator *avcc_33_vreg;
	struct regulator *avcc_12_vreg;
	struct regulator *iovcc_18_vreg;
	struct platform_device *hdmi_pdev;
	struct msm_hdmi_mhl_ops *hdmi_mhl_ops;
	struct notifier_block mhl_notif;
} T_MHL_SII9234_INFO;

/*********************************************************************
   Variable & Extern variable
**********************************************************************/
static T_MHL_SII9234_INFO *sii9234_info_ptr;
/*********************************************************************
  Prototype & Extern function
**********************************************************************/
static void sii9234_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sii9234_irq_work, sii9234_irq_do_work);

static DEFINE_MUTEX(mhl_early_suspend_sem);
unsigned long suspend_jiffies;
unsigned long irq_jiffies;
bool g_bEnterEarlySuspend = false;
bool g_bProbe = false;
bool disable_interswitch = false;
bool mhl_wakeuped = false;
static bool g_bInitCompleted = false;
static bool sii9244_interruptable = false;
static bool need_simulate_cable_out = false;
#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
static bool g_bPollDetect = false;
#endif
static bool g_bLowPowerModeOn = false;

static struct dentry *dbg_entry_dir, *dbg_entry_a3, *dbg_entry_a6, *dbg_entry_dbg_on;
static struct dentry *dbg_entry_con_test_timeout, *dbg_entry_con_test_on,
	*dbg_entry_con_test_random;

u8 dbg_drv_str_a3 = 0xEB, dbg_drv_str_a6 = 0x0C, dbg_drv_str_on = 0;
static u8 dbg_con_test_timeout = MHL_DEBUGFS_TIMEOUT, dbg_con_test_on = 0,
	dbg_con_test_random = 1;
void hdmi_set_switch_state(bool enable);

#ifdef MHL_RCP_KEYEVENT
struct input_dev *input_dev;
#endif
static struct platform_device *mhl_dev; /* Device structure */

/*********************************************************************
	Functions
**********************************************************************/
static int dbg_con_get_timeout(void)
{
	int ret = dbg_con_test_timeout;

	if (dbg_con_test_random) {
		int t = MHL_DEBUGFS_TIMEOUT;
		int s = jiffies%MHL_DEBUGFS_AMP;
		ret = jiffies%2?t+s:t-s;
	}

	PR_DISP_INFO("%s: timeout = %d\n", __func__, ret);

	return ret;
}

#ifdef CONFIG_CABLE_DETECT_ACCESSORY
static DEFINE_MUTEX(mhl_notify_sem);

#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
static void detect_charger_handler(struct work_struct *w)
{
	T_MHL_SII9234_INFO *pInfo = container_of(
			w, T_MHL_SII9234_INFO, detect_charger_work.work);

	mutex_lock(&mhl_early_suspend_sem);

	PR_DISP_DEBUG("%s: query status every 2 second\n", __func__);
	SiiMhlTxReadDevcap(0x02);

	mutex_unlock(&mhl_early_suspend_sem);

	queue_delayed_work(pInfo->wq, &pInfo->detect_charger_work, HZ*2);
}
#endif
#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
void check_mhl_5v_status(void)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;
	if (!pInfo)
		return;
	/*
		for the case of (1)plug dongle +AC +HDMI  (2)remove HDMI (3) plug HDMI,
		we should turn off internal 5v in this case step3
	*/
	if(pInfo->isMHL && (pInfo->statMHL == CONNECT_TYPE_MHL_AC || pInfo->statMHL == CONNECT_TYPE_USB )){
		if(pInfo->enable_5v)
			pInfo->enable_5v(0);
	}
}
#endif
void update_mhl_status(bool isMHL, enum usb_connect_type statMHL)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;
	if (!pInfo)
		return;

	PR_DISP_INFO("%s: -+-+-+-+- MHL is %sconnected, status = %d -+-+-+-+-\n",
		__func__, isMHL?"":"NOT ", statMHL);
	pInfo->isMHL = isMHL;
	pInfo->statMHL = statMHL;

	if(!isMHL)
		sii9234_power_vote(false);

	queue_work(pInfo->wq, &pInfo->mhl_notifier_work);

#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
	if (isMHL && (statMHL == CONNECT_TYPE_INTERNAL)) {
		if (!g_bPollDetect) {
			g_bPollDetect = true;
			queue_delayed_work(pInfo->wq, &pInfo->detect_charger_work, HZ/2);
			cancel_delayed_work(&pInfo->turn_off_5v);
		}
	} else if (statMHL == CONNECT_TYPE_MHL_AC || statMHL == CONNECT_TYPE_USB) {
		cancel_delayed_work(&pInfo->detect_charger_work);
		g_bPollDetect = false;
		/*disable boost 5v after 1 second*/
		queue_delayed_work(pInfo->wq, &pInfo->turn_off_5v, HZ);
	}
	else {
		g_bPollDetect = false;
		cancel_delayed_work(&pInfo->detect_charger_work);
	}
#endif
}

static void send_mhl_connect_notify(struct work_struct *w)
{
	static struct t_mhl_status_notifier *mhl_notifier;
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;

	if (!pInfo)
		return;

	PR_DISP_DEBUG("%s: %d\n", __func__, pInfo->isMHL);
	mutex_lock(&mhl_notify_sem);
	list_for_each_entry(mhl_notifier,
		&g_lh_mhl_detect_notifier_list,
		mhl_notifier_link) {
			if (mhl_notifier->func != NULL)
#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
				mhl_notifier->func(pInfo->isMHL, pInfo->statMHL);
#else
				mhl_notifier->func(pInfo->isMHL, false);
#endif
		}
	pInfo->hdmi_mhl_ops->send_cable_notification(pInfo->hdmi_pdev, pInfo->isMHL);
	mutex_unlock(&mhl_notify_sem);
}

int mhl_detect_register_notifier(struct t_mhl_status_notifier *notifier)
{
	if (!notifier || !notifier->name || !notifier->func)
		return -EINVAL;

	mutex_lock(&mhl_notify_sem);
	list_add(&notifier->mhl_notifier_link,
		&g_lh_mhl_detect_notifier_list);
	mutex_unlock(&mhl_notify_sem);
	return 0;
}
#endif

int sii9234_I2C_RxData(uint8_t deviceID, char *rxData, uint32_t length)
{
	uint8_t loop_i;
	uint8_t slave_addr = deviceID >> 1;
	struct i2c_msg msgs[] = {
		{
		 .addr = slave_addr,
		 .flags = 0,
		 .len = 1,
		 .buf = rxData,
		 },
		{
		 .addr = slave_addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};
	if (!sii9234_info_ptr)
		return 0;
	sii9234_info_ptr->i2c_client->addr = slave_addr;
	for (loop_i = 0; loop_i < SII9234_I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(sii9234_info_ptr->i2c_client->adapter, msgs, 2) > 0)
			break;

		mdelay(10);
	}

	if (loop_i >= SII9234_I2C_RETRY_COUNT) {
		PR_DISP_DEBUG("%s retry over %d\n", __func__, SII9234_I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

int sii9234_I2C_TxData(uint8_t deviceID, char *txData, uint32_t length)
{
	uint8_t loop_i;
	uint8_t slave_addr = deviceID >> 1;
	struct i2c_msg msg[] = {
		{
		 .addr = slave_addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};
	if (!sii9234_info_ptr)
		return 0;
	sii9234_info_ptr->i2c_client->addr = slave_addr;
	for (loop_i = 0; loop_i < SII9234_I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(sii9234_info_ptr->i2c_client->adapter, msg, 1) > 0)
			break;

		mdelay(10);
	}

	if (loop_i >= SII9234_I2C_RETRY_COUNT) {
		PR_DISP_DEBUG("%s retry over %d\n", __func__, SII9234_I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

int sii9234_get_intr_status(void)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;
	if (!pInfo)
		return -1;

	return sii_gpio_get_value(pInfo->intr_pin);
}

void sii9234_reset(void)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;
	if (!pInfo)
		return;

	sii_gpio_set_value(pInfo->reset_pin, 0);
	mdelay(2);
	sii_gpio_set_value(pInfo->reset_pin, 1);
}

int sii9234_get_ci2ca(void)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;
	if (!pInfo)
		return -1;
	return pInfo->ci2ca_pin;
}

static void sii9234_irq_do_work(struct work_struct *work)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;
	if (!pInfo)
		return;

	mutex_lock(&mhl_early_suspend_sem);
	if(time_after(jiffies, irq_jiffies + HZ/20))
	{
		uint8_t		event;
		uint8_t		eventParameter;
		irq_jiffies = jiffies;
		/*PR_DISP_DEBUG("MHL ISR\n");*/
		need_simulate_cable_out = false;
		if(!dbg_con_test_on)
			cancel_delayed_work(&pInfo->irq_timeout_work);
		SiiMhlTxGetEvents(&event, &eventParameter);
		ProcessRcp(event, eventParameter);
	}
	mutex_unlock(&mhl_early_suspend_sem);

	enable_irq(pInfo->irq);
}

void sii9234_disableIRQ(void)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;
	if (!pInfo)
		return;
	cancel_work_sync(&sii9234_irq_work);
	if (sii9244_interruptable) {
		PR_DISP_DEBUG("%s\n", __func__);
		disable_irq_nosync(pInfo->irq);
		sii9244_interruptable = false;
	}
}

int sii9234_power_vote(bool enable)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;

	if (!pInfo)
		return 0;
	if (pInfo->mhl_power_vote) {
		if (enable)
			pInfo->mhl_power_vote(1);
		else
			pInfo->mhl_power_vote(0);
	}
	return 0;
}

static irqreturn_t sii9234_irq_handler(int irq, void *data)
{
	T_MHL_SII9234_INFO *pInfo = data;

	if (pInfo->wq) {
		disable_irq_nosync(pInfo->irq);
		queue_work(pInfo->wq, &sii9234_irq_work);
	} else
		PR_DISP_DEBUG("%s: workqueue is not ready yet.", __func__);

	return IRQ_HANDLED;
}

void sii9234_send_keyevent(uint32_t key, uint32_t type)
{
#ifdef MHL_RCP_KEYEVENT
	PR_DISP_DEBUG("CBUS key_event: %d\n", key);
	if (type == 0) {
		input_report_key(input_dev, key, 1);
		input_report_key(input_dev, key, 0);
		input_sync(input_dev);
	}
#endif
}

#ifdef MHL_RCP_KEYEVENT
/* Sysfs method to input simulated coordinates */
static ssize_t write_keyevent(struct device *dev,
				struct device_attribute *attr,
				const char *buffer, size_t count)
{
	int key;

	/* parsing input data */
	sscanf(buffer, "%d", &key);


	PR_DISP_DEBUG("key_event: %d\n", key);

	/* Report key event */
	switch (key) {
	case 0:
		input_report_key(input_dev, KEY_HOME, 1);
		input_report_key(input_dev, KEY_HOME, 0);
		break;
	case 1:
		input_report_key(input_dev, KEY_UP, 1);
		input_report_key(input_dev, KEY_UP, 0);
		break;
	case 2:
		input_report_key(input_dev, KEY_DOWN, 1);
		input_report_key(input_dev, KEY_DOWN, 0);
		break;
	case 3:
		input_report_key(input_dev, KEY_LEFT, 1);
		input_report_key(input_dev, KEY_LEFT, 0);
		break;
	case 4:
		input_report_key(input_dev, KEY_RIGHT, 1);
		input_report_key(input_dev, KEY_RIGHT, 0);
		break;
	case 5:
		input_report_key(input_dev, KEY_ENTER, 1);
		input_report_key(input_dev, KEY_ENTER, 0);
		break;
	case 6:
		input_report_key(input_dev, KEY_SELECT, 1);
		input_report_key(input_dev, KEY_SELECT, 0);
		break;
	default:
		input_report_key(input_dev, KEY_OK, 1);
		input_report_key(input_dev, KEY_OK, 0);
		break;
	}
	input_sync(input_dev);
	return count;
}

/* Attach the sysfs write method */
static DEVICE_ATTR(rcp_event, 0644, NULL, write_keyevent);
#endif

void sii9234_mhl_device_wakeup(void)
{
	int err;
	int ret = 0 ;

	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;
	if (!pInfo)
		return;

	PR_DISP_INFO("sii9234_mhl_device_wakeup()\n");

	mutex_lock(&mhl_early_suspend_sem);

	sii9234_power_vote(true);

	if (!g_bInitCompleted) {
		PR_DISP_INFO("MHL inserted before HDMI related function was ready! Wait more 5 sec...\n");
		queue_delayed_work(pInfo->wq, &pInfo->init_delay_work, HZ*5);
		mutex_unlock(&mhl_early_suspend_sem);
		return;
	}

	/* MHL_USB_SW for Verdi  0: switch to MHL;  others projects, 1 : switch to MHL */
	if (pInfo->mhl_usb_switch)
		pInfo->mhl_usb_switch(1);

	sii_gpio_set_value(pInfo->reset_pin, 1); /* Reset High */

	if (g_bLowPowerModeOn) {
		g_bLowPowerModeOn = false;
		if (pInfo->mhl_lpm_power)
			pInfo->mhl_lpm_power(0);
	}

	if (pInfo->mhl_1v2_power)
		pInfo->mhl_1v2_power(1);

	err = TPI_Init();
	if (err != 1)
		PR_DISP_INFO("TPI can't init\n");

	sii9244_interruptable = true;
	PR_DISP_INFO("Enable Sii9244 IRQ\n");

	/*request irq pin again, for solving MHL_INT is captured to INUT LOW*/
	if(mhl_wakeuped) {
		disable_irq_nosync(pInfo->irq);
		free_irq(pInfo->irq, pInfo);

		ret = request_irq(pInfo->irq, sii9234_irq_handler, IRQF_TRIGGER_LOW, "mhl_sii9234_evt", pInfo);
		if (ret < 0) {
			PR_DISP_DEBUG("%s: request_irq(%d) failed for gpio %d (%d)\n",
				__func__, pInfo->irq, pInfo->intr_pin, ret);
			ret = -EIO;
		}
	} else
		enable_irq(pInfo->irq);

	mhl_wakeuped = true;

	/*switch to D0, we now depends on Sii9244 to detect the connection by MHL interrupt*/
	/*if there is no IRQ in the following steps , the status of connect will be in-correct and cannot be recovered*/
	/*add a mechanism to simulate cable out to prevent this case.*/
	need_simulate_cable_out = true;
	if(!dbg_con_test_on)
		queue_delayed_work(pInfo->wq, &pInfo->irq_timeout_work, HZ * MHL_ISR_TIMEOUT);
	else
		queue_delayed_work(pInfo->wq, &pInfo->irq_timeout_work, HZ * dbg_con_get_timeout());

	mutex_unlock(&mhl_early_suspend_sem);
}

static void init_delay_handler(struct work_struct *w)
{
	PR_DISP_INFO("init_delay_handler()\n");

	TPI_Init();
	update_mhl_status(false, CONNECT_TYPE_UNKNOWN);
}

static void init_complete_handler(struct work_struct *w)
{
	PR_DISP_INFO("init_complete_handler()\n");
	/*make sure the MHL is in sleep *& usb_bypass mode*/
	TPI_Init();
	g_bInitCompleted = true;
}
static void irq_timeout_handler(struct work_struct *w)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;
	if(!dbg_con_test_on) {
		if(need_simulate_cable_out) {
			int ret = 0 ;
			if (!pInfo)
				return;
			/*need to request_irq again on 8960 VLE, or this prevention is not working*/
			PR_DISP_INFO("%s , There is no MHL ISR simulate cable out.\n", __func__);
			disable_irq_nosync(pInfo->irq);
			TPI_Init();
			free_irq(pInfo->irq, pInfo);
			ret = request_irq(pInfo->irq, sii9234_irq_handler, IRQF_TRIGGER_LOW, "mhl_sii9234_evt", pInfo);
			if (ret < 0) {
				PR_DISP_DEBUG("%s: request_irq(%d) failed for gpio %d (%d)\n",
					__func__, pInfo->irq, pInfo->intr_pin, ret);
				ret = -EIO;
			}
			enable_irq(pInfo->irq);
			update_mhl_status(false, CONNECT_TYPE_UNKNOWN);
		}
	} else {
		sii9234_disableIRQ();
		if (pInfo->mhl_1v2_power)
			pInfo->mhl_1v2_power(0);
		if (pInfo->mhl_usb_switch)
			pInfo->mhl_usb_switch(0);
		TPI_Init();
		update_mhl_status(false, CONNECT_TYPE_UNKNOWN);
	}
}

static void mhl_on_delay_handler(struct work_struct *w)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;
	if (!pInfo)
		return;

	mutex_lock(&mhl_early_suspend_sem);
	if (IsMHLConnection()) {
		/*have HDMI cable on dongle*/
/*
		fill_black_screen();
		sii9234_EnableTMDS();

		if (pInfo->mhl_1v2_power)
			pInfo->mhl_1v2_power(1);
		hdmi_set_switch_state(true);
*/
		PR_DISP_DEBUG("MHL has connected. No SimulateCableOut!!!\n");
		mutex_unlock(&mhl_early_suspend_sem);
		return;
	}
	else {
		if(pInfo->isMHL){
			/*MHL dongle plugged but no HDMI calbe*/
			PR_DISP_DEBUG("notify cable out, re-init cable & mhl\n");
			update_mhl_status(false, CONNECT_TYPE_UNKNOWN);
			TPI_Init();
		} else {
			if (g_bLowPowerModeOn) {
				g_bLowPowerModeOn = false;
				if (pInfo->mhl_lpm_power)
					pInfo->mhl_lpm_power(0);
			}
		}
	}
	mutex_unlock(&mhl_early_suspend_sem);
}

static int sii9234_suspend(void)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;
	if (!pInfo)
		return -ENODEV;

	PR_DISP_INFO("%s(isMHL=%d)\n", __func__, pInfo->isMHL);

	/* dongle attached with no MHL cable plugged in */
	if (pInfo->isMHL && !tpi_get_hpd_state())
		sii9234_disableIRQ();

	mutex_lock(&mhl_early_suspend_sem);
	/* Enter the early suspend state...*/
	g_bEnterEarlySuspend = true;
	suspend_jiffies = jiffies;

	/* Cancel the previous TMDS on delay work...*/
	cancel_delayed_work(&pInfo->mhl_on_delay_work);
	if (pInfo->isMHL) {
		/*For the case of dongle without HDMI cable*/
		if(!tpi_get_hpd_state()){
			/* Turn-off the TMDS output...*/
			if (Status_Query() != POWER_STATE_D3)
				SiiMhlTxDrvTmdsControl(false);

#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
			cancel_delayed_work(&pInfo->detect_charger_work);
#endif
			/*follow suspend GPIO state,  disable hdmi HPD*/
			if (pInfo->mhl_1v2_power)
				pInfo->mhl_1v2_power(0);
			if (pInfo->mhl_usb_switch)
				pInfo->mhl_usb_switch(0);
			/*D3 mode with internal switch in by-pass mode*/
			TPI_Init();
		} else {
			/*if (pInfo->enable_5v)
				pInfo->enable_5v(0);
			if (pInfo->mhl_1v2_power)
				pInfo->mhl_1v2_power(0);
			hdmi_set_switch_state(false);*/
		}
	} else {
		/*in case cable_detect call D2ToD3(), make sure MHL chip is go Sleep mode correctly*/
		/*for the case of plug non-MHL accessory, need to disable internal switch for avoiding toggle USB_ID pin*/
		if (cable_get_accessory_type() != DOCK_STATE_MHL )
			disable_interswitch = true;
		if (!g_bLowPowerModeOn) {
			g_bLowPowerModeOn = true;
			if (pInfo->mhl_lpm_power)
				pInfo->mhl_lpm_power(1);
		}
		disable_interswitch = false;
	}
	mutex_unlock(&mhl_early_suspend_sem);

	return 0;
}

static int sii9234_resume(void)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;
	if (!pInfo)
		return -ENODEV;

	PR_DISP_INFO("sii9234_late_resume()\n");

	mutex_lock(&mhl_early_suspend_sem);
	queue_delayed_work(pInfo->wq, &pInfo->mhl_on_delay_work, HZ);

	g_bEnterEarlySuspend = false;
	mutex_unlock(&mhl_early_suspend_sem);

	return 0;
}

static void mhl_turn_off_5v(struct work_struct *w)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;
	if (!pInfo)
		return;
	if(pInfo->enable_5v)
		pInfo->enable_5v(0);
}

/* FIXME: Remove auto plug function temporary */
//extern void fake_plug(bool plug);

static int mhl_con_event_open(struct inode *inode, struct file *file)
{
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	return 0;
}
static int mhl_con_event_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t mhl_con_event_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	int con_event = 0;
	char debug_buf[5];

	if (count >= sizeof(debug_buf))
		return -EFAULT;

	if (copy_from_user(debug_buf, buff, count))
		return -EFAULT;

	debug_buf[count] = 0;

	sscanf(debug_buf, "%d", &con_event);

/*	FIXME: Remove auto plug function temporary
	if(con_event == 1)
		fake_plug(true);
	else
		fake_plug(false);
*/
	return count;
}

static const struct file_operations mhl_con_event_fops = {
	.open = mhl_con_event_open,
	.release = mhl_con_event_release,
	.read = NULL,
	.write = mhl_con_event_write,
};



/*add debugfs for driving strength 0xA3*/
static int sii_debugfs_init(void)
{
	dbg_entry_dir = debugfs_create_dir("mhl", NULL);
	if (!dbg_entry_dir) {
		PR_DISP_DEBUG("Fail to create debugfs dir: mhl\n");
		return -1;
	}
	dbg_entry_a3 = debugfs_create_u8("strength_a3", 0644, dbg_entry_dir, &dbg_drv_str_a3);
	if (!dbg_entry_a3)
		PR_DISP_DEBUG("Fail to create debugfs: strength_a3\n");
	dbg_entry_a6 = debugfs_create_u8("strength_a6", 0644, dbg_entry_dir, &dbg_drv_str_a6);
	if (!dbg_entry_a6)
		PR_DISP_DEBUG("Fail to create debugfs: strength_a6\n");
	dbg_entry_dbg_on = debugfs_create_u8("dbg_on", 0644, dbg_entry_dir, &dbg_drv_str_on);
	if (!dbg_entry_dbg_on)
		PR_DISP_DEBUG("Fail to create debugfs: dbg_on\n");

	/*debugfs for connection test*/
	dbg_entry_con_test_timeout = debugfs_create_u8("con_test_timeout", 0644, dbg_entry_dir, &dbg_con_test_timeout);
	if (!dbg_entry_con_test_timeout)
		PR_DISP_DEBUG("Fail to create debugfs: con_test_timeout\n");
	dbg_entry_con_test_on = debugfs_create_u8("con_test_on", 0644, dbg_entry_dir, &dbg_con_test_on);
	if (!dbg_entry_dbg_on)
		PR_DISP_DEBUG("Fail to create debugfs: con_test_on\n");
	dbg_entry_con_test_random = debugfs_create_u8("con_test_random", 0644, dbg_entry_dir, &dbg_con_test_random);
        if (!dbg_entry_dbg_on)
                PR_DISP_DEBUG("Fail to create debugfs: con_test_random\n");

	/*debugfs for connection_event test*/
	if (debugfs_create_file("con_event", 0644, dbg_entry_dir, 0, &mhl_con_event_fops)
			== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_file: debug fail\n",
			__FILE__, __LINE__);
		return -1;
	}
	return 0;
}

void sii9234_request_abort(void)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;
	need_simulate_cable_out = true;
	PR_DISP_INFO("reuqest to abort connection");
	queue_delayed_work(pInfo->wq, &pInfo->irq_timeout_work, HZ);
}

static int mhl_get_dt_data(struct device *dev,
	T_MHL_SII9234_INFO *pInfo)
{
	int tmp, rc = 0;
	struct device_node *of_node = NULL;
	struct device_node *hdmi_tx_node = NULL;

	if (!dev || !pInfo) {
		PR_DISP_ERR("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	of_node = dev->of_node;
	if (!of_node) {
		PR_DISP_ERR("%s: invalid of_node\n", __func__);
		return -EINVAL;
	}

	pInfo->reset_pin = of_get_named_gpio(of_node, "mhl-rst-gpio", 0);
	if (pInfo->reset_pin < 0) {
		PR_DISP_ERR("%s: Can't get mhl-rst-gpio\n", __func__);
		return -EINVAL;
	}

	pInfo->intr_pin = of_get_named_gpio(of_node, "mhl-intr-gpio", 0);
	if (pInfo->intr_pin < 0) {
		PR_DISP_ERR("%s: Can't get mhl-intr-gpio\n", __func__);
		return -EINVAL;
	}

	rc = of_property_read_u32(of_node, "mhl-ci2ca", &tmp);
	if (rc)
		PR_DISP_WARN("%s: ci2ca_pin not specified\n",
						__func__);
	pInfo->ci2ca_pin = (!rc ? tmp : 0);

	pInfo->avcc_33_vreg = devm_regulator_get(dev, "avcc_33");
	if (IS_ERR(pInfo->avcc_33_vreg)) {
		PR_DISP_ERR("%s: could not get avcc_33 reg, rc=%ld\n",
			__func__, PTR_ERR(pInfo->avcc_33_vreg));
		return PTR_ERR(pInfo->avcc_33_vreg);
	}
	rc = regulator_set_voltage(pInfo->avcc_33_vreg, 3300000,
			3300000);
	if (rc) {
		PR_DISP_ERR("%s: set voltage failed on avcc_33 vreg, rc=%d\n",
			__func__, rc);
		return rc;
	}

	/* avcc_12 & cvcc_12 must be derived from the same power source */
	pInfo->avcc_12_vreg = devm_regulator_get(dev, "avcc_12");
	if (IS_ERR(pInfo->avcc_12_vreg)) {
		PR_DISP_ERR("%s: could not get avcc_12 reg, rc=%ld\n",
			__func__, PTR_ERR(pInfo->avcc_12_vreg));
		return PTR_ERR(pInfo->avcc_12_vreg);
	}
	rc = regulator_set_voltage(pInfo->avcc_12_vreg, 1200000,
			1200000);
	if (rc) {
		PR_DISP_ERR("%s: set voltage failed on avcc_12 vreg, rc=%d\n",
			__func__, rc);
		return rc;
	}

	pInfo->iovcc_18_vreg = devm_regulator_get(dev, "iovcc_18");

	/* Sometimes iovcc_18 may use system io power */
	if (IS_ERR(pInfo->iovcc_18_vreg)) {
		PR_DISP_WARN("%s: could not get iovcc_18, rc=%ld\n",
			__func__, PTR_ERR(pInfo->iovcc_18_vreg));
	} else {
		rc = regulator_set_voltage(pInfo->iovcc_18_vreg, 1800000,
				1800000);
		if (rc) {
			PR_DISP_ERR("%s: set voltage failed on iovcc_18 vreg, rc=%d\n",
				__func__, rc);
			return rc;
		}
	}

	/* parse phandle for hdmi tx */
	hdmi_tx_node = of_parse_phandle(of_node, "qcom,hdmi-tx-map", 0);
	if (!hdmi_tx_node) {
		pr_err("%s: can't find hdmi phandle\n", __func__);
		return -EINVAL;
	}

	pInfo->hdmi_pdev = of_find_device_by_node(hdmi_tx_node);
	if (!pInfo->hdmi_pdev) {
		pr_err("%s: can't find the device by node\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: hdmi_pdev [0X%x] to pdata->pdev\n",
	       __func__, (unsigned int)pInfo->hdmi_pdev);

	return 0;
} /* mhl_tx_get_dt_data */

static int mhl_sii9234_power_on(T_MHL_SII9234_INFO *pInfo)
{
	int rc = 0;

	if (!IS_ERR(pInfo->iovcc_18_vreg)) {
		rc = regulator_enable(pInfo->iovcc_18_vreg);
		if (rc) {
			PR_DISP_ERR("%s: Failed to enable iovcc_18_vreg regulator.\n",
				__func__);
			return rc;
		}
	}

	rc = regulator_enable(pInfo->avcc_33_vreg);
	if (rc) {
		PR_DISP_ERR("%s: Failed to enable avcc_33_vreg regulator.\n",
			__func__);
		return rc;
	}

	rc = regulator_enable(pInfo->avcc_12_vreg);
	if (rc) {
		PR_DISP_ERR("%s: Failed to enable avcc_12_vreg regulator.\n",
			__func__);
		return rc;
	}

	/* Initial MHL rst/intr pin */
	if (gpio_is_valid(pInfo->reset_pin)) {
		gpio_set_value(pInfo->reset_pin, 0);
	} else {
		PR_DISP_ERR("%s: Failed to control mhl reset_pin.\n",
			__func__);
		return EINVAL;
	}

	if (gpio_is_valid(pInfo->intr_pin)) {
		gpio_set_value(pInfo->intr_pin, 0);
	} else {
		PR_DISP_ERR("%s: Failed to control mhl intr_pin.\n",
			__func__);
		return EINVAL;
	}

	return rc;
}

void mhl_sii9234_1v2_power(bool enable)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;

	if(enable) {
		pInfo->enable_5v(1);
		pInfo->hdmi_mhl_ops->set_upstream_hpd(pInfo->hdmi_pdev, enable);
	} else {
		pInfo->hdmi_mhl_ops->set_upstream_hpd(pInfo->hdmi_pdev, enable);
		pInfo->enable_5v(0);
	}
}

static int mhl_sii9234_lpm_power(bool enable)
{
	int rc = 0;
	int lpm_on_value = 0;
	int lpm_off_value = 100000;
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;

	mutex_lock(&mhl_lpm_lock);

	pr_info("[DISP] %s (%s)\n", __func__, (enable) ? "on" : "off");

	rc = regulator_set_optimum_mode(pInfo->avcc_33_vreg,
		(enable)? lpm_on_value : lpm_off_value);

	if (rc < 0)
		pr_err("%s: set_lpm avcc_33_vreg failed rc=%d\n", __func__, rc);
	rc = regulator_enable(pInfo->avcc_33_vreg);
	if (rc) {
		pr_err("%s avcc_33_vreg enable failed, rc=%d\n", __func__, rc);
		mutex_unlock(&mhl_lpm_lock);
		return rc;
	}

	rc = regulator_set_optimum_mode(pInfo->avcc_12_vreg,
		(enable)? lpm_on_value : lpm_off_value);

	if (rc < 0)
		pr_err("%s: set_lpm avcc_12_vreg failed rc=%d\n", __func__, rc);
	rc = regulator_enable(pInfo->avcc_12_vreg);
	if (rc) {
		pr_err("%s avcc_12_vreg enable failed, rc=%d\n", __func__, rc);
		mutex_unlock(&mhl_lpm_lock);
		return rc;
	}

	mutex_unlock(&mhl_lpm_lock);
	return rc;
}

static int sii9234_enable_internal_charging_5v(int on)
{
	struct device *dev;
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;
	static struct regulator *reg_boost_5v = NULL;/* HDMI_5V */
	struct device_node *of_node = NULL;
	static int prev_on = 0;
	int rc;

	if (on == prev_on)
		return 0;

	if(!reg_boost_5v) {
		dev = &pInfo->i2c_client->dev;
		of_node = dev->of_node;
		if (!of_node) {
			PR_DISP_ERR("%s: invalid of_node\n", __func__);
			return -EINVAL;
		}

		reg_boost_5v = devm_regulator_get(dev, "hpd_5v");
		if (IS_ERR(reg_boost_5v)) {
			PR_DISP_ERR("%s: get reg_boost_5v fail\n", __func__);
			return -EINVAL;
		}
	}

	if (on) {
		rc = regulator_enable(reg_boost_5v);
		if (rc) {
			pr_err("'hpd_5v' regulator enable failed, rc=%d\n", rc);
			return rc;
		}
	} else {
		rc = regulator_disable(reg_boost_5v);
		if (rc)
			pr_warning("'hpd_5v' regulator disable failed, rc=%d\n", rc);
	}

	pr_info("%s(%s): success\n", __func__, on?"on":"off");

	prev_on = on;

	return 0;
}

static int sii9234_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = E_MHL_OK;
	bool rv = TRUE;
	T_MHL_SII9234_INFO *pInfo;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PR_DISP_DEBUG("%s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	pInfo = kzalloc(sizeof(T_MHL_SII9234_INFO), GFP_KERNEL);
	if (!pInfo) {
		PR_DISP_DEBUG("%s: alloc memory error!!\n", __func__);
		return -ENOMEM;
	}

	if (!client->dev.of_node) {
		PR_DISP_DEBUG("%s : no device node\n", __func__);
		return -ENOMEM;
	}

	pInfo->i2c_client = client;
	pInfo->irq = client->irq;
	i2c_set_clientdata(client, pInfo);

	ret = mhl_get_dt_data(&client->dev, pInfo);
	if(ret) {
		PR_DISP_ERR("%s: register with hdmi failed\n", __func__);
		ret = -ENODEV;
		goto err_failed_probe_mhl;
	}
	pInfo->hdmi_mhl_ops = devm_kzalloc(&client->dev,
				    sizeof(struct msm_hdmi_mhl_ops),
				    GFP_KERNEL);
	if (!pInfo->hdmi_mhl_ops) {
		PR_DISP_ERR("%s: alloc hdmi mhl ops failed\n", __func__);
		ret = -ENOMEM;
		goto err_failed_probe_mhl;
	}

	if (pInfo->hdmi_pdev) {
		ret = msm_hdmi_register_mhl(pInfo->hdmi_pdev,
					   pInfo->hdmi_mhl_ops, NULL, true);
		if (ret) {
			PR_DISP_ERR("%s: register with hdmi failed\n", __func__);
			ret = -EPROBE_DEFER;
			goto err_failed_probe_mhl;
		}
	}

	ret = pInfo->hdmi_mhl_ops->set_mhl_max_pclk(
		pInfo->hdmi_pdev, MAX_MHL_PCLK);
	if (ret) {
		PR_DISP_ERR("%s: can't set max mhl pclk\n", __func__);
		goto err_failed_probe_mhl;
	}

	pInfo->mhl_1v2_power = mhl_sii9234_1v2_power;
	pInfo->mhl_lpm_power = mhl_sii9234_lpm_power;
	pInfo->enable_5v = sii9234_enable_internal_charging_5v;
	sii9234_info_ptr = pInfo;
	/*making sure TPI_INIT() is working fine with pInfo*/
	g_bProbe = true;
	/* Power ON */
	mhl_sii9234_power_on(pInfo);

	/* build mhl debugfs */
	sii_debugfs_init();

	/* Pin Config */
	gpio_request(pInfo->reset_pin, "mhl_sii9234_gpio_reset");
	gpio_direction_output(pInfo->reset_pin, 0);
	gpio_request(pInfo->intr_pin, "mhl_sii9234_gpio_intr");
	gpio_direction_input(pInfo->intr_pin);
	rv = TPI_Init();
	if (rv != TRUE) {
		PR_DISP_DEBUG("%s: can't init\n", __func__);
		ret = -ENOMEM;
		goto err_init;
	}

	INIT_DELAYED_WORK(&pInfo->init_delay_work, init_delay_handler);
	INIT_DELAYED_WORK(&pInfo->init_complete_work, init_complete_handler);
	INIT_DELAYED_WORK(&pInfo->irq_timeout_work, irq_timeout_handler);
	INIT_DELAYED_WORK(&pInfo->mhl_on_delay_work, mhl_on_delay_handler);
	INIT_DELAYED_WORK(&pInfo->turn_off_5v, mhl_turn_off_5v);

#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
	INIT_DELAYED_WORK(&pInfo->detect_charger_work, detect_charger_handler);
#endif
	irq_jiffies = jiffies;

	ret = request_irq(pInfo->irq, sii9234_irq_handler, IRQF_TRIGGER_LOW, "mhl_sii9234_evt", pInfo);
	if (ret < 0) {
		PR_DISP_DEBUG("%s: request_irq(%d) failed for gpio %d (%d)\n",
			__func__, pInfo->irq, pInfo->intr_pin, ret);
		ret = -EIO;
		goto err_request_intr_pin;
	}
	disable_irq_nosync(pInfo->irq);
	sii9244_interruptable = false;

	pInfo->wq = create_workqueue("mhl_sii9234_wq");
	if (!pInfo->wq) {
		PR_DISP_DEBUG("%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_workqueue;
	}
#ifdef CONFIG_CABLE_DETECT_ACCESSORY
	INIT_WORK(&pInfo->mhl_notifier_work, send_mhl_connect_notify);
#endif
	/* Register a platform device */
	mhl_dev = platform_device_register_simple("mhl", -1, NULL, 0);
	if (IS_ERR(mhl_dev)) {
		PR_DISP_DEBUG("mhl init: error\n");
		return PTR_ERR(mhl_dev);
	}
	/* Create a sysfs node to read simulated coordinates */

#ifdef MHL_RCP_KEYEVENT
	ret = device_create_file(&mhl_dev->dev, &dev_attr_rcp_event);

	input_dev = input_allocate_device();
	if (!input_dev) {
		PR_DISP_DEBUG("%s: could not allocate input device\n", __func__);
		ret = -ENOMEM;
		goto err_init;
	}
	/* indicate that we generate key events */
	set_bit(EV_KEY, input_dev->evbit);
	/* indicate that we generate *any* key event */
	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_HOME, input_dev->keybit);
	set_bit(KEY_ENTER, input_dev->keybit);
	set_bit(KEY_LEFT, input_dev->keybit);
	set_bit(KEY_UP, input_dev->keybit);
	set_bit(KEY_DOWN, input_dev->keybit);
	set_bit(KEY_RIGHT, input_dev->keybit);
	set_bit(KEY_PLAY, input_dev->keybit);
	set_bit(KEY_STOP, input_dev->keybit);
	set_bit(KEY_PLAYPAUSE, input_dev->keybit);
	set_bit(KEY_REWIND, input_dev->keybit);
	set_bit(KEY_FASTFORWARD, input_dev->keybit);

	input_dev->name = "rcp_events";

	ret = input_register_device(input_dev);
	if (ret < 0)
		PR_DISP_DEBUG("MHL: can't register input devce\n");
#endif
	/* Initiate a 5 sec delay which will change the "g_bInitCompleted" be true after it...*/
	queue_delayed_work(pInfo->wq, &pInfo->init_complete_work, HZ*10);
	PR_DISP_DEBUG("%s: Probe success!\n", __func__);
	return ret;
err_create_workqueue:
	gpio_free(pInfo->reset_pin);
	gpio_free(pInfo->intr_pin);
err_init:
err_request_intr_pin:
err_failed_probe_mhl:
	kfree(pInfo);
	sii9234_info_ptr = NULL;
err_check_functionality_failed:
	return ret;
}

static int sii9234_remove(struct i2c_client *client)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;
	if (!pInfo)
		return -1;
	gpio_free(pInfo->reset_pin);
	gpio_free(pInfo->intr_pin);

	debugfs_remove(dbg_entry_dir);
	destroy_workqueue(pInfo->wq);
	kfree(pInfo);
	return E_MHL_OK;
}

static const struct i2c_device_id sii9234_i2c_id[] = {
	{MHL_SII9234_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sii9234_i2c_id);

static struct of_device_id mhl_match_table[] = {
	{.compatible = COMPATIBLE_NAME,},
	{ },
};

static struct i2c_driver sii9234_driver = {
	.id_table = sii9234_i2c_id,
	.probe = sii9234_probe,
	.remove = sii9234_remove,
	.driver = {
		.name = MHL_SII9234_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mhl_match_table,
	},
};

static int mhl_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	PR_DISP_INFO("%s\n", __func__);
	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		switch (*blank) {
		case FB_BLANK_UNBLANK:
			sii9234_resume();
			break;
		case FB_BLANK_POWERDOWN:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_NORMAL:
			sii9234_suspend();
			break;
		}
	}

	return 0;
}

static void mhl_usb_status_notifier_func(int cable_type)
{
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;

	if (!pInfo)
		return;
	/*in suspend mode, vbus change should be wakeup the system*/
	mutex_lock(&mhl_early_suspend_sem);

	if(cable_get_accessory_type() == DOCK_STATE_MHL && g_bEnterEarlySuspend){
		if(pInfo->statMHL == CONNECT_TYPE_INTERNAL) {
			if(time_after(jiffies, suspend_jiffies + HZ))
				update_mhl_status(true, CONNECT_TYPE_MHL_AC);
		} else {
			update_mhl_status(true, CONNECT_TYPE_INTERNAL);
			pInfo->hdmi_mhl_ops->send_cable_notification(pInfo->hdmi_pdev, false);
		}
	}
	mutex_unlock(&mhl_early_suspend_sem);
}

static struct t_usb_status_notifier usb_status_notifier = {
	.name = "mhl_vbus_detect",
	.func = mhl_usb_status_notifier_func,
};

static int __devinit mhl_ctrl_probe(struct platform_device *pdev)
{
	int rc = 0;
	T_MHL_PLATFORM_DATA *pdata;
	T_MHL_SII9234_INFO *pInfo = sii9234_info_ptr;

	PR_DISP_INFO("%s:%d, debug info id=%d", __func__, __LINE__, pdev->id);

	if (pdev->dev.platform_data && pInfo) {
		pdata = pdev->dev.platform_data;
		pInfo->mhl_usb_switch = pdata->mhl_usb_switch;
	}

	pInfo->mhl_notif.notifier_call = mhl_notifier_callback;
	rc = fb_register_client(&pInfo->mhl_notif);
	if (rc)
		PR_DISP_ERR("Unable to register mhl_notifier: %d\n", rc);

	PR_DISP_INFO("%s: %p\n", __func__, pdev->dev.platform_data);

	return rc;
}

static struct platform_driver mhl_ctrl_driver = {
	.probe = mhl_ctrl_probe,
	.driver = {
		.name = "sii9234_mhl_ctrl",
	},
};

static void __init sii9234_init_async(void *unused, async_cookie_t cookie)
{
	htc_usb_register_notifier(&usb_status_notifier);
	i2c_add_driver(&sii9234_driver);
	platform_driver_register(&mhl_ctrl_driver);
}

static int __init sii9234_init(void)
{
	async_schedule(sii9234_init_async, NULL);
	return 0;
}

static void __exit sii9234_exit(void)
{
	i2c_del_driver(&sii9234_driver);
}

module_init(sii9234_init);
module_exit(sii9234_exit);

MODULE_DESCRIPTION("SiI9234 Driver");
MODULE_LICENSE("GPL");
