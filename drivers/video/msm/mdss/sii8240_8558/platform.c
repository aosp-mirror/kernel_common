/*

SiI8558 / SiI8240 Linux Driver

Copyright (C) 2013 Silicon Image, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation version 2.
This program is distributed AS-IS WITHOUT ANY WARRANTY of any
kind, whether express or implied; INCLUDING without the implied warranty
of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.  See
the GNU General Public License for more details at http://www.gnu.org/licenses/gpl-2.0.html.

*/

#include <linux/module.h>
#include <linux/async.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/semaphore.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <mach/board.h>
#include <mach/debug_display.h>

#include "si_fw_macros.h"
#include "si_mhl_defs.h"
#include "si_infoframe.h"
#include "si_edid.h"
#include "si_mhl2_edid_3d_api.h"
#include "si_mhl_tx_hw_drv_api.h"
#ifdef MEDIA_DATA_TUNNEL_SUPPORT
#include <linux/input.h>
#include "si_mdt_inputdev.h"
#endif
#include "mhl_linux_tx.h"
#include "platform.h"
#include "si_8240_8558_drv.h"
#include "si_8240_8558_regs.h"
#include "../mdss_hdmi_mhl.h"
#include "mhl_platform.h"


#define GET_FROM_MODULE_PARAM 	-1
#define GPIO_NOT_PRESENT	-2
#define MHL_ISR_TIMEOUT 	(5 * HZ)

static struct i2c_adapter	*i2c_bus_adapter = NULL;

struct i2c_dev_info {
	uint8_t			dev_addr;
	struct i2c_client	*client;
};

#define I2C_DEV_INFO(addr) \
	{.dev_addr = addr >> 1, .client = NULL}

static struct i2c_dev_info device_addresses[] = {
	I2C_DEV_INFO(TX_PAGE_L0),
	I2C_DEV_INFO(TX_PAGE_L1),
	I2C_DEV_INFO(TX_PAGE_2),
	I2C_DEV_INFO(TX_PAGE_3),
	I2C_DEV_INFO(TX_PAGE_TPI),
	I2C_DEV_INFO(TX_PAGE_CBUS),
	I2C_DEV_INFO(TX_PAGE_DDC_SEGM),
	I2C_DEV_INFO(TX_PAGE_DDC_EDID)
};


bool	allow_d3        = 1;
bool	transcode_mode  = 0;
int	debug_level     = 0;
int	override_strap  = 0;
bool	special_charger = 0;
bool	input_dev_rap   = 1;
bool	input_dev_rcp   = 1;
bool	input_dev_ucp   = 1;
int	gpio_index      = -1;
bool	push_pull_8240  = 0;
static bool reset_on_exit = 1;  
uint8_t mhlDriverDone 	= 0;
bool 	ap_hdcp_success = false;
EXPORT_SYMBOL(ap_hdcp_success);

module_param(allow_d3,		bool, S_IRUGO);
module_param(transcode_mode,	bool, S_IRUGO);
module_param(debug_level,	int,  S_IRUGO);
module_param(override_strap,	int,  S_IRUGO);
module_param(special_charger,	bool, S_IRUGO);
module_param(reset_on_exit,	bool, S_IRUGO);
module_param(input_dev_rap,	bool, S_IRUGO);
module_param(input_dev_rcp,	bool, S_IRUGO);
module_param(input_dev_ucp,	bool, S_IRUGO);
module_param(push_pull_8240,	bool, S_IRUGO);

module_param_named(debug_msgs,debug_level, int, S_IRUGO);

struct platform_signals_list platform_signals[] = {
		{	.name			= "W_RST#",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "M2U_VBUS_CTRL_M",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "MCPC_MODE",
			.gpio_number	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "ALLOW_D3",
			.gpio_number 	= GET_FROM_MODULE_PARAM,
			.param			= &allow_d3
		},
		{	.name			= "TX2MHLRX_PWR_M",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "TRANSCODE_MODE",
			.gpio_number 	= GET_FROM_MODULE_PARAM,
			.param			= &transcode_mode
		},
		{	.name			= "FW_WAKE",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "MHL_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "USB_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "USB_ID_L_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "UART_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "AUDIO_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "MIC_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "SRC_VBUS_ON_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "SINK_VBUS_ON_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "SDP_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "CDP_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "DCP_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "PS2_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "D0_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "D2_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "D3_HOT_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "D3_COLD_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "3D_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "PACKED_PIXEL_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "HDCP_ON_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "TRANSCODE_MODE_LED",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "SPARE_LED1",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "SPARE_LED2",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "GNT_",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "COMM_MODE",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
		{	.name			= "12V_PS_SENSE",
			.gpio_number 	= GPIO_NOT_PRESENT,
			.param			= NULL
		},
};

static inline int platform_read_i2c_block(struct i2c_adapter *i2c_bus, u8 page, u8 offset,
						u8 count, u8 *values)
{
	struct i2c_msg msg[2];

	msg[0].flags = 0;
	msg[0].addr = page >> 1;
	msg[0].buf = &offset;
	msg[0].len = 1;

	msg[1].flags = I2C_M_RD;
	msg[1].addr = page >> 1;
	msg[1].buf = values;
	msg[1].len = count;

	return i2c_transfer(i2c_bus_adapter, msg, 2);
}

static inline int platform_write_i2c_block(struct i2c_adapter *i2c_bus, u8 page, u8 offset,
						u16 count, u8 *values)
{
	struct i2c_msg msg;
	u8 *buffer;
	int ret;

	buffer = kmalloc(count + 1, GFP_KERNEL);
	if (!buffer) {
		printk("%s:%d buffer allocation failed\n",__FUNCTION__,__LINE__);
		return -ENOMEM;
	}

	buffer[0] = offset;
	memmove(&buffer[1], values, count);

	msg.flags = 0;
	msg.addr = page >> 1;
	msg.buf = buffer;
	msg.len = count + 1;

	ret = i2c_transfer(i2c_bus, &msg, 1);

	kfree(buffer);

	if (ret != 1) {
		printk("%s:%d I2c write failed 0x%02x:0x%02x\n"
				,__FUNCTION__,__LINE__, page, offset);
		ret = -EIO;
	} else {
		ret = 0;
	}

	return ret;
}

int get_config(void *dev_context, int config_idx)
{
	int	pin_state = 0;

	if (config_idx < ARRAY_SIZE(platform_signals)) {
		if (platform_signals[config_idx].gpio_number != GET_FROM_MODULE_PARAM) {
			if (GPIO_NOT_PRESENT != platform_signals[config_idx].gpio_number){
				pin_state =  gpio_get_value(platform_signals[config_idx].gpio_number);
			}
		}
		else
			pin_state =  *(platform_signals[config_idx].param);

		MHL_TX_DBG_GPIO(NULL, "(%d,%s) pin_state:%d\n"
			, platform_signals[config_idx ].gpio_number
			, platform_signals[config_idx ].name
			, pin_state
			);
	}
	return pin_state;
}

void set_pin_impl(void *dev_context, int pin_idx, int value
				,const char *function_name,int line_num)
{
	if (pin_idx < ARRAY_SIZE(platform_signals)) {
		MHL_TX_DBG_GPIO(NULL, "(%d,%s,%d)\n"
			, platform_signals[pin_idx].gpio_number
			, platform_signals[pin_idx].name
			, value
			);

		if (platform_signals[pin_idx].gpio_number != GET_FROM_MODULE_PARAM) {
			if (GPIO_NOT_PRESENT != platform_signals[pin_idx].gpio_number) {
				gpio_set_value(platform_signals[pin_idx].gpio_number, value);
			}
		}
	}
}

#ifdef CONFIG_HTC_MHL_DETECTION
static DEFINE_MUTEX(mhl_notify_sem);
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

static void send_mhl_connect_notify(struct work_struct *w)
{
	static struct t_mhl_status_notifier *mhl_notifier;
	struct mhl_dev_context *dev_context;

	pr_info("[MHL]%s\n", __func__);
	mutex_lock(&mhl_notify_sem);
	dev_context = i2c_get_clientdata(device_addresses[0].client);
	list_for_each_entry(mhl_notifier,
		&g_lh_mhl_detect_notifier_list,
		mhl_notifier_link) {
#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
		mhl_notifier->func(dev_context->isMHL, dev_context->statMHL);
#else
		mhl_notifier->func(dev_context->isMHL, false);
#endif
	}

	mutex_unlock(&mhl_notify_sem);
}
#else
static void send_mhl_connect_notify(struct work_struct *w)
{

}
#endif
#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
void htc_charging_enable(int dcap)
{
	if (MHL_DEV_CATEGORY_POW_BIT & dcap) {
		uint8_t param = 0, plim = 0;
		struct mhl_dev_context *dev_context = i2c_get_clientdata(device_addresses[0].client);
		param = dcap & MHL_DEV_CATEGORY_POW_BIT;
		plim = dcap & MHL_DEV_CATEGORY_PLIM;

		if (param) {
			mhl_tx_vbus_control(VBUS_OFF);
			if (plim) {
				
				PR_DISP_INFO("[MHL] 1000mA charger!!\n");
				dev_context->statMHL = CONNECT_TYPE_MHL_AC;
			} else {
				
				PR_DISP_INFO("[MHL] 500mA charger!!\n");
				dev_context->statMHL = CONNECT_TYPE_USB;
			}
			if (MHL_DEV_CAT_DONGLE & dcap) {
				PR_DISP_INFO("[MHL] BACKDOOR charger!!\n");
				dev_context->statMHL = CONNECT_TYPE_MHL_AC;
			}
			queue_work(dev_context->wq, &dev_context->mhl_notifier_work);
		}

		
		mhl_event_notify(dev_context, MHL_TX_EVENT_POW_BIT_CHG, param, NULL);
	} else
		MHL_TX_DBG_ERR(NULL, "NOT Valid dev_cap, No POW_BIT");
}
#endif
void mhl_tx_vbus_control(enum vbus_power_state power_state)
{
#if 1
	struct device *dev;
	static struct regulator *reg_boost_5v = NULL;
	struct device_node *of_node = NULL;
	static int prev_on = 0;
	int rc;

	if (power_state == prev_on)
		return;

	if(!reg_boost_5v) {
		dev = &device_addresses[0].client->dev;
		of_node = dev->of_node;
		if (!of_node) {
			PR_DISP_ERR("%s: invalid of_node\n", __func__);
			return ;
		}

		reg_boost_5v = devm_regulator_get(dev, "hpd_5v");
		if (IS_ERR_OR_NULL(reg_boost_5v)) {
			PR_DISP_ERR("%s: get reg_boost_5v fail, ret:%ld\n", __func__,
						PTR_ERR(reg_boost_5v));
			reg_boost_5v = NULL;
			return;
		}
	}

	if (power_state == VBUS_ON) {
		rc = regulator_enable(reg_boost_5v);
		if (rc) {
			pr_err("'hpd_5v' regulator enable failed, rc=%d\n", rc);
			return;
		}
	} else {
		rc = regulator_disable(reg_boost_5v);
		if (rc)
			pr_warning("'hpd_5v' regulator disable failed, rc=%d\n", rc);
	}

	pr_info("%s(%s): success\n", __func__, power_state?"on":"off");

	prev_on = power_state;

	return;
#else
	struct mhl_dev_context *dev_context;
	dev_context = i2c_get_clientdata(device_addresses[0].client);

	switch (power_state) {
	case VBUS_OFF:
		set_pin(dev_context,TX2MHLRX_PWR_M,1);
		set_pin(dev_context,SRC_VBUS_ON_LED,GPIO_LED_OFF);
		break;

	case VBUS_ON:
		set_pin(dev_context,TX2MHLRX_PWR_M,0);
		set_pin(dev_context,SRC_VBUS_ON_LED,GPIO_LED_ON);
		break;

	default:
		dev_err(dev_context->mhl_dev,
				"%s: Invalid power state %d received!\n",
				__func__, power_state);
		break;
	}
#endif
}

int si_8240_device_dbg_i2c_reg_xfer(void *dev_context, u8 page, u8 offset,
									u8 count, bool rw_flag, u8 *buffer)
{
	if (rw_flag == DEBUG_I2C_WRITE)
		return mhl_tx_write_reg_block(dev_context, page, offset, count, buffer);
	else
		return mhl_tx_read_reg_block(dev_context, page, offset, count, buffer);
}


#define MAX_DEBUG_MSG_SIZE	1024

#if defined(DEBUG)

char *find_file_name(const char *path_spec)
{
	char *pc;

	for (pc = (char *)&path_spec[strlen(path_spec)]; pc != path_spec; --pc) {
		if ('\\' == *pc) {
			++pc;
			break;
		}
		if ('/' == *pc) {
			++pc;
			break;
		}
	}
	return pc;
}

void print_formatted_debug_msg(int level,
							   char *file_spec, const char *func_name,
							   int line_num,
							   char *fmt, ...)
{
	uint8_t		*msg = NULL;
	uint8_t		*msg_offset;
	char		*file_spec_sep = NULL;
	int			remaining_msg_len = MAX_DEBUG_MSG_SIZE;
	int			len;
	va_list		ap;

	if (level > debug_level){
			return;
	}

	if (fmt == NULL)
		return;

	if (file_spec != NULL)
		file_spec = find_file_name(file_spec);

	msg = kmalloc(remaining_msg_len, GFP_KERNEL);
	if(msg == NULL)
		return;

	msg_offset = msg;

	if (file_spec != NULL) {
		if (func_name != NULL)
			file_spec_sep = "->";
		else if (line_num != -1)
			file_spec_sep = ":";
	}

	len = scnprintf(msg_offset, remaining_msg_len, "[MHL]");
	msg_offset += len;
	remaining_msg_len -= len;

	if (file_spec) {
		len = scnprintf(msg_offset, remaining_msg_len, "%s", file_spec);
		msg_offset += len;
		remaining_msg_len -= len;
	}

	if (file_spec_sep) {
		len = scnprintf(msg_offset, remaining_msg_len, "%s", file_spec_sep);
		msg_offset += len;
		remaining_msg_len -= len;
	}

	if (func_name) {
		len = scnprintf(msg_offset, remaining_msg_len, "%s", func_name);
		msg_offset += len;
		remaining_msg_len -= len;
	}

	if (line_num != -1) {
		if ((file_spec != NULL) || (func_name != NULL))
			len = scnprintf(msg_offset, remaining_msg_len, ":%d ", line_num);
		else
			len = scnprintf(msg_offset, remaining_msg_len, "%d ", line_num);

		msg_offset += len;
		remaining_msg_len -= len;
	}

	va_start(ap, fmt);
	len = vscnprintf(msg_offset, remaining_msg_len, fmt, ap);
	va_end(ap);

	printk(msg);

	kfree(msg);
}

void dump_i2c_transfer(void *context, u8 page, u8 offset,
						u16 count, u8 *values, bool write)
{
	int		buf_size = 64;
	u16		idx;
	int		buf_offset;
	char	*buf;

	if (count > 1) {
		buf_size += count * 3; 				
		buf_size += ((count / 16) + 1) * 8;	
	}

	buf = kmalloc(buf_size, GFP_KERNEL);
	if (!buf)
		return;

	if (count == 1) {

		scnprintf(buf, buf_size, "   I2C_%s %02x:%02x %s %x\n",
				  write ? "W" : "R",
				  page, offset,
				  write ? "<-" : "=",
				  values[0]);
	} else {
		idx = 0;
		buf_offset = scnprintf(buf, buf_size, "I2C_%sB %02x:%02x - %d bytes:",
							   write ? "W" : "R", page, offset, count);

		for (idx = 0; idx < count; idx++) {
			if (0 == (idx & 0x0F))
				buf_offset += scnprintf(&buf[buf_offset], buf_size - buf_offset,
										"\n%04X: ", idx);

			buf_offset += scnprintf(&buf[buf_offset], buf_size - buf_offset,
									"%02x ", values[idx]);
		}
		buf_offset += scnprintf(&buf[buf_offset], buf_size - buf_offset, "\n");
	}

	print_formatted_debug_msg(DBG_MSG_LEVEL_INFO, NULL, NULL, -1, buf);

	kfree(buf);
}
#endif 

struct mhl_drv_info drv_info = {
	.drv_context_size = sizeof(struct drv_hw_context),
	.mhl_device_initialize = si_mhl_tx_chip_initialize,
	.mhl_device_isr = si_mhl_tx_drv_device_isr,
	.mhl_device_dbg_i2c_reg_xfer = si_8240_device_dbg_i2c_reg_xfer,
};

int is_interrupt_asserted(void)
{
	return (gpio_get_value(drv_info.intr_pin) ? 0 : 1);
}

bool is_reset_on_exit_requested(void)
{
	return reset_on_exit;
}

int mhl_tx_write_reg_block(void *drv_context, u8 page, u8 offset, u16 count,
						   u8 *values)
{
	DUMP_I2C_TRANSFER(drv_context, page, offset, count, values, true);

	return platform_write_i2c_block(i2c_bus_adapter,page, offset, count,values);
}


int mhl_tx_write_reg(void *drv_context, u8 page, u8 offset, u8 value)
{
	return mhl_tx_write_reg_block(drv_context, page, offset, 1, &value);
}


int mhl_tx_read_reg_block(void *drv_context, u8 page, u8 offset, u8 count,
						  u8 *values)
{
	int						ret;
	ret = platform_read_i2c_block(i2c_bus_adapter
								, page
								, offset
								, count
								, values
								);
	if (ret != 2) {
		MHL_TX_DBG_ERR(drv_context, "I2c read failed, 0x%02x:0x%02x\n", page, offset);
		ret = -EIO;
	} else {
		ret = 0;
		DUMP_I2C_TRANSFER(drv_context, page, offset, count, values, false);
	}

	return ret;
}

int mhl_tx_read_reg(void *drv_context, u8 page, u8 offset)
{
	u8		byte_read = 0;
	int		status;

	status = mhl_tx_read_reg_block(drv_context, page, offset, 1, &byte_read);

	return status ? status : byte_read;
}

int mhl_tx_modify_reg(void *drv_context, u8 page, u8 offset,
					  u8 mask, u8 value)
{
	int	reg_value;
	int	write_status;

	reg_value = mhl_tx_read_reg(drv_context, page, offset);
	if (reg_value < 0)
		return reg_value;

	reg_value &= ~mask;
	reg_value |= mask & value;

	write_status = mhl_tx_write_reg(drv_context, page, offset, reg_value);

	if (write_status < 0)
		return write_status;
	else
		return reg_value;
}

hpd_control_mode platform_get_hpd_control_mode(void)
{
	MHL_TX_DBG_ERR(, " push_pull_8240=%d\n", push_pull_8240);
	return (push_pull_8240 ?  HPD_CTRL_PUSH_PULL : HPD_CTRL_OPEN_DRAIN);
}

void si_d2_to_d3(void)
{
	struct mhl_dev_context *dev_context;
	struct drv_hw_context *hw_context;

	if (mhlDriverDone) {
		dev_context = i2c_get_clientdata(device_addresses[0].client);
		hw_context = (struct drv_hw_context *)&dev_context->drv_context;
		switch_to_d3(hw_context, true);
	} else
		MHL_TX_DBG_ERR(NULL,"Driver probe failed\n" );
}

void si_wakeup_mhl(void)
{
	struct mhl_dev_context *dev_context;
	struct drv_hw_context *hw_context;

	pr_info("[MHL] %s\n", __func__);
	if (mhlDriverDone) {
		dev_context = i2c_get_clientdata(device_addresses[0].client);
		hw_context = (struct drv_hw_context *)&dev_context->drv_context;
		drv_info.mhl_device_initialize(hw_context, true);

		
		dev_context->fake_cable_out = true;
		queue_delayed_work(dev_context->wq, &dev_context->irq_timeout_work,
				MHL_ISR_TIMEOUT);
	} else
		MHL_TX_DBG_ERR(NULL,"Driver probe failed\n" );
}

static void update_mhl_status(bool isMHL, enum usb_connect_type statMHL)
{
	struct mhl_dev_context *dev_context =
				i2c_get_clientdata(device_addresses[0].client);

	pr_info("[MHL] ++++ MHL is %sConnected,"
		" Trigger remove event ++++\n", isMHL ? "": "NOT ");

	dev_context->isMHL = isMHL;
	dev_context->statMHL = statMHL;

	queue_work(dev_context->wq, &dev_context->mhl_notifier_work);
	drv_info.hdmi_mhl_ops->send_cable_notification(drv_info.hdmi_pdev, 0);
}

static void irq_timeout_handler(struct work_struct *w)
{
	struct mhl_dev_context *dev_context;
	struct drv_hw_context *hw_context;

	dev_context = i2c_get_clientdata(device_addresses[0].client);
	hw_context = (struct drv_hw_context *)&dev_context->drv_context;

	if (!dev_context || !hw_context)
		return;

	pr_info("[MHL]%s:fake_cable_out:%d\n", __func__, dev_context->fake_cable_out);
	if (dev_context->fake_cable_out) {
		disable_irq_nosync(dev_context->client->irq);
		drv_info.mhl_device_initialize(hw_context, true);
		enable_irq(dev_context->client->irq);
		update_mhl_status(false, CONNECT_TYPE_UNKNOWN);
	}
}

static int si_8240_8558_add_i2c(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	int idx;

	
	i2c_bus_adapter = adapter;
	if (i2c_bus_adapter == NULL) {
		PR_DISP_ERR("%s() failed to get i2c adapter\n", __func__);
		return ENODEV;
	}

	for (idx = 0; idx < ARRAY_SIZE(device_addresses); idx++) {
		if (idx == 0)
			device_addresses[idx].client = client;
		else {
			device_addresses[idx].client = i2c_new_dummy(i2c_bus_adapter,
								device_addresses[idx].dev_addr);
			if (device_addresses[idx].client == NULL) {
				return ENODEV;
			}
		}
	}

	return 0;
}

static int mhl_get_power_dt_data(struct device *dev)
{
	int rc = 0;

	drv_info.avcc_33_vreg = devm_regulator_get(dev, "avcc_33");
	if (IS_ERR(drv_info.avcc_33_vreg)) {
		PR_DISP_ERR("%s: could not get avcc_33 reg, rc=%ld\n",
			__func__, PTR_ERR(drv_info.avcc_33_vreg));
		return PTR_ERR(drv_info.avcc_33_vreg);
	}
	rc = regulator_set_voltage(drv_info.avcc_33_vreg, 3300000,
			3300000);
	if (rc) {
		PR_DISP_ERR("%s: set voltage failed on avcc_33 vreg, rc=%d\n",
			__func__, rc);
		return rc;
	}

	
	drv_info.avcc_12_vreg = devm_regulator_get(dev, "avcc_12");
	if (IS_ERR(drv_info.avcc_12_vreg)) {
		PR_DISP_ERR("%s: could not get avcc_12 reg, rc=%ld\n",
			__func__, PTR_ERR(drv_info.avcc_12_vreg));
		return PTR_ERR(drv_info.avcc_12_vreg);
	}
	rc = regulator_set_voltage(drv_info.avcc_12_vreg, 1200000,
			1200000);
	if (rc) {
		PR_DISP_ERR("%s: set voltage failed on avcc_12 vreg, rc=%d\n",
			__func__, rc);
		return rc;
	}

	drv_info.iovcc_18_vreg = devm_regulator_get(dev, "iovcc_18");
	
	if (IS_ERR(drv_info.iovcc_18_vreg)) {
		PR_DISP_WARN("%s: could not get iovcc_18, rc=%ld\n",
			__func__, PTR_ERR(drv_info.iovcc_18_vreg));
	} else {
		rc = regulator_set_voltage(drv_info.iovcc_18_vreg, 1800000,
				1800000);
		if (rc) {
			PR_DISP_ERR("%s: set voltage failed on iovcc_18 vreg, rc=%d\n",
				__func__, rc);
			return rc;
		}
	}

	return 0;
}

static int mhl_power_on(void)
{
	int rc = 0;

	if (!IS_ERR(drv_info.iovcc_18_vreg)) {
		rc = regulator_enable(drv_info.iovcc_18_vreg);
		if (rc) {
			PR_DISP_ERR("%s: Failed to enable iovcc_18_vreg regulator.\n",
				__func__);
			return rc;
		}
	}

	rc = regulator_enable(drv_info.avcc_33_vreg);
	if (rc) {
		PR_DISP_ERR("%s: Failed to enable avcc_33_vreg regulator.\n",
			__func__);
		return rc;
	}

	rc = regulator_enable(drv_info.avcc_12_vreg);
	if (rc) {
		PR_DISP_ERR("%s: Failed to enable avcc_12_vreg regulator.\n",
			__func__);
		return rc;
	}

	return rc;
}

static int mhl_get_gpio_dt_data(struct device *dev)
{
	int rc = 0, tmp;
	uint32_t chip_id = 0;
	struct device_node *of_node = NULL;
	struct mhl_dev_context *dev_context;
	struct drv_hw_context *hw_context;

	dev_context = i2c_get_clientdata(device_addresses[0].client);
	hw_context = (struct drv_hw_context *)&dev_context->drv_context;

	of_node = dev->of_node;
	drv_info.reset_pin = of_get_named_gpio(of_node, "mhl-rst-gpio", 0);
	if (!gpio_is_valid(drv_info.reset_pin)) {
		PR_DISP_ERR("%s: Can't get mhl-rst-gpio\n", __func__);
		return -EINVAL;
	} else {
		pr_info("[MHL]rst_gpio[%d]=%d,PwrOn Pull_High", drv_info.reset_pin,
				gpio_get_value_cansleep(drv_info.reset_pin));

		
		usleep(1000);
		gpio_set_value_cansleep(drv_info.reset_pin, 1);
		usleep(1000);
		gpio_set_value_cansleep(drv_info.reset_pin, 0);
		
		usleep(1000);
		gpio_set_value_cansleep(drv_info.reset_pin, 1);

		msleep(100);	
		platform_signals[TX_HW_RESET].gpio_number = drv_info.reset_pin;
	}

	drv_info.intr_pin = of_get_named_gpio(of_node, "mhl-intr-gpio", 0);
	if (!gpio_is_valid(drv_info.intr_pin)) {
		PR_DISP_ERR("%s: Can't get mhl-intr-gpio\n", __func__);
		return -EINVAL;
	}

	chip_id = get_device_id();
	switch (chip_id) {
	case DEVICE_ID_8558:
		drv_info.fw_wake_pin = of_get_named_gpio(of_node, "mhl-fw-wake-gpio", 0);
		if (drv_info.fw_wake_pin < 0) {
			PR_DISP_DEBUG("%s: Can't get mhl-fw-wake-gpio\n", __func__);
			return -EINVAL;
		}
		gpio_index = drv_info.fw_wake_pin;
		break;
	case DEVICE_ID_8240:
		break;
	default:
		pr_err("[MHL]%s get_device_id exeception,%#x",__func__, chip_id);
		return -EINVAL;
	}

	drv_info.dpdn_pin = of_get_named_gpio(of_node, "mhl-dpdn-gpio", 0);
	if (!gpio_is_valid(drv_info.dpdn_pin)) {
		PR_DISP_ERR("%s: Can't get mhl-dpdn-gpio\n", __func__);
		return -EINVAL;
	}

	rc = of_property_read_u32(of_node, "mhl-ci2ca", &tmp);
	if (rc)
		PR_DISP_WARN("%s: ci2ca_pin not specified\n",
						__func__);
	drv_info.ci2ca_pin = (!rc ? tmp : 0);

	return 0;
}

static int mhl_get_swing_value(struct device *dev)
{
	struct mhl_dev_context *dev_context;
	const u8 *data;
	int len, i;

	dev_context = i2c_get_clientdata(device_addresses[0].client);
	data = of_get_property(dev->of_node, "mhl-swing", &len);
	if ((!data) || (len != 3)) {
		PR_DISP_WARN("%s:%d, Unable to read swing value, use default value\n",
			   __func__, __LINE__);
		for (i = 0; i < len; i++)
			dev_context->swing_value[i] = 0x03;

		return 0;
	}
	for (i = 0; i < len; i++)
		dev_context->swing_value[i] = data[i];

	return 0;
}

static int mhl_get_dt_data(struct device *dev)
{
	int rc = 0;
	struct device_node *of_node = NULL;
	struct device_node *hdmi_tx_node = NULL;

	if (!dev) {
		PR_DISP_ERR("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	of_node = dev->of_node;
	if (!of_node) {
		PR_DISP_ERR("%s: invalid of_node\n", __func__);
		return -EINVAL;
	}

	rc = mhl_get_power_dt_data(dev);
	if (rc < 0) {
		PR_DISP_ERR("%s: mhl_get_power_dt_data fail\n", __func__);
		return -EINVAL;
	}

	rc = mhl_power_on();
	if (rc < 0) {
		PR_DISP_ERR("%s: mhl_power_on fail\n", __func__);
		return -EINVAL;
	}

	rc = mhl_get_gpio_dt_data(dev);
	if (rc < 0) {
		PR_DISP_ERR("%s: mhl_get_gpio_dt_data fail\n", __func__);
		return -EINVAL;
	}

	
	hdmi_tx_node = of_parse_phandle(of_node, "qcom,hdmi-tx-map", 0);
	if (!hdmi_tx_node) {
		pr_err("%s: can't find hdmi phandle\n", __func__);
		return -EINVAL;
	}

	drv_info.hdmi_pdev = of_find_device_by_node(hdmi_tx_node);
	if (!drv_info.hdmi_pdev) {
		pr_err("%s: can't find the device by node\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: hdmi_pdev [0X%x] to pdata->pdev\n",
	       __func__, (unsigned int)drv_info.hdmi_pdev);

	return 0;
} 

void enable_hdmi(int enable)
{
	if (drv_info.hdmi_pdev && drv_info.hdmi_mhl_ops) {
		drv_info.hdmi_mhl_ops->set_upstream_hpd(drv_info.hdmi_pdev, enable);
		gpio_set_value(drv_info.dpdn_pin, enable);
	}
	ap_hdcp_success = false;
}

static int __devinit si_8240_8558_mhl_tx_i2c_probe(struct i2c_client *client,
					      const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct mhl_dev_context *dev_context;
	int ret;

	printk("[MHL]+++%s+++\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		PR_DISP_ERR("[ERROR] %s() i2c no function\n", __func__);
		return -EIO;
	}

	if (!client->dev.of_node) {
		PR_DISP_ERR("%s : no device node\n", __func__);
		return -ENOMEM;
	}

	ret = si_8240_8558_add_i2c(client);
	if (ret < 0) {
		PR_DISP_ERR("si_8240_8558_add_i2c failed, error code %d\n", ret);
		return ret;
	}

	ret = mhl_get_dt_data(&client->dev);
	if(ret < 0) {
		PR_DISP_ERR("mhl_get_dt_data failed, error code %d\n", ret);
		return ret;
	}

	drv_info.hdmi_mhl_ops = devm_kzalloc(&client->dev,
				    sizeof(struct msm_hdmi_mhl_ops),
				    GFP_KERNEL);
	if (!drv_info.hdmi_mhl_ops) {
		PR_DISP_ERR("%s: alloc hdmi mhl ops failed\n", __func__);
		return -ENOMEM;
	}

	if (drv_info.hdmi_pdev) {
		ret = msm_hdmi_register_mhl(drv_info.hdmi_pdev,
					   drv_info.hdmi_mhl_ops, NULL, false);
		if (ret) {
			PR_DISP_ERR("%s: register with hdmi failed\n", __func__);
			return -EPROBE_DEFER;
		}
	}

	ret = drv_info.hdmi_mhl_ops->set_mhl_max_pclk(
		drv_info.hdmi_pdev, MAX_MHL_PCLK);
	if (ret) {
		PR_DISP_ERR("%s: can't set max mhl pclk\n", __func__);
		return -EPERM;
	}

	ret = mhl_tx_init(&drv_info, client);
	if(ret < 0) {
		PR_DISP_ERR("mhl_tx_init failed, error code %d\n", ret);
		return ret;
	}

	dev_context = i2c_get_clientdata(device_addresses[0].client);
	dev_context->wq = create_workqueue("mhl_sii8240_8558_wq");
	INIT_WORK(&dev_context->mhl_notifier_work, send_mhl_connect_notify);

	
	dev_context->fake_cable_out = false;
	INIT_DELAYED_WORK(&dev_context->irq_timeout_work, irq_timeout_handler);
	
	mhl_get_swing_value(&client->dev);
	mhlDriverDone = 1;
	printk("[MHL]---%s successful---\n", __func__);
	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int mhl_pm_suspend(struct device *dev)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	if (!dev_context)
		return -ENODEV;

	dev_dbg(dev, "mhl pm suspend\n");

	disable_irq_nosync(dev_context->client->irq);
	return 0;
}

static int mhl_pm_resume(struct device *dev)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	if (!dev_context)
		return -ENODEV;

	dev_dbg(dev, "mhl pm resume\n");

	enable_irq(dev_context->client->irq);
	return 0;
}
#else
#define mhl_pm_suspend NULL
#define mhl_pm_resume NULL
#endif

static int __devexit si_8240_8558_mhl_tx_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id si_8240_8558_mhl_tx_id[] = {
	{MHL_DEVICE_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, si_8240_8558_mhl_tx_id);

static struct of_device_id mhl_match_table[] = {
	{.compatible = COMPATIBLE_NAME,},
	{ },
};

static const struct dev_pm_ops mhl_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mhl_pm_suspend, mhl_pm_resume)
};

static struct i2c_driver si_8240_8558_mhl_tx_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = MHL_DRIVER_NAME,
		   .of_match_table = mhl_match_table,
		   .pm = &mhl_pm_ops,
		   },
	.id_table = si_8240_8558_mhl_tx_id,
	.probe = si_8240_8558_mhl_tx_i2c_probe,
	.remove = __devexit_p(si_8240_8558_mhl_tx_remove),
	.command = NULL,
};
static void __init si_8240_8558_init_async(void *unused, async_cookie_t cookie)
{
	int ret = -EFAULT;
	ret = i2c_add_driver(&si_8240_8558_mhl_tx_i2c_driver);
	if (ret < 0) {
		MHL_TX_DBG_INFO(,"[ERROR] ret:%d failed !\n",ret);
	}
	MHL_TX_DBG_INFO(dev_context, "returning %d\n",ret);
}

static int __init si_8240_8558_init(void)
{
	async_schedule(si_8240_8558_init_async, NULL);
	return 0;
}

static void __exit si_8240_8558_exit(void)
{
	int	idx;

	mhl_tx_remove(device_addresses[0].client);
	MHL_TX_DBG_INFO(NULL, "client removed\n");
	i2c_del_driver(&si_8240_8558_mhl_tx_i2c_driver);
	MHL_TX_DBG_INFO(NULL, "i2c driver deleted from context\n");

	for (idx = 0; idx < ARRAY_SIZE(device_addresses); idx++) {
		MHL_TX_DBG_INFO(NULL, "\n");
		if (device_addresses[idx].client != NULL){
			MHL_TX_DBG_INFO(NULL, "unregistering device:%p\n",device_addresses[idx].client);
			i2c_unregister_device(device_addresses[idx].client);
		}
	}
}

#ifdef DDC_BYPASS_API 
int si_8240_8558_ddc_bypass_control(int bypass)
{
	int status;
	struct mhl_dev_context *dev_context = i2c_get_clientdata(device_addresses[0].client);
	struct drv_hw_context *hw_context = (struct drv_hw_context *)&dev_context->drv_context;
	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
	}else{
		if (bypass){
			debug_level =2;
			MHL_TX_DBG_INFO(NULL, "bypass enabled\n\n\n");
		}
		status = si_8240_8558_drv_ddc_bypass_control(hw_context,bypass);
		if (!bypass){
			MHL_TX_DBG_INFO(NULL, "bypass disabled\n\n\n");
			debug_level =0;
		}
	}

	up(&dev_context->isr_lock);
	return status;
}
EXPORT_SYMBOL(si_8240_8558_ddc_bypass_control);
#endif 

module_init(si_8240_8558_init);
module_exit(si_8240_8558_exit);

MODULE_DESCRIPTION("Silicon Image MHL Transmitter driver");
MODULE_AUTHOR("Silicon Image <http://www.siliconimage.com>");
MODULE_LICENSE("GPL");
