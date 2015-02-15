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
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/fs.h>
#include <linux/semaphore.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/stringify.h>
#include <asm/uaccess.h>

#include "si_fw_macros.h"
#include "si_mhl_defs.h"
#include "si_infoframe.h"
#include "si_edid.h"
#include "si_mhl2_edid_3d_api.h"
#include "si_mhl_tx_hw_drv_api.h"
#include "si_8240_8558_drv.h"
#ifdef MEDIA_DATA_TUNNEL_SUPPORT
#include "si_mdt_inputdev.h"
#endif
#include "mhl_rcp_inputdev.h"
#include "mhl_linux_tx.h"
#include "mhl_supp.h"
#include "platform.h"

#define MHL_DRIVER_MINOR_MAX 1

static char *white_space = "' ', '\t'";
static dev_t dev_num;

static struct class *mhl_class;

static void mhl_tx_destroy_timer_support(struct  mhl_dev_context *dev_context);

#define SYS_ATTR_NAME_CONN			connection_state
#ifndef DRIVER_SPEC_SYSFS 
#define SYS_ATTR_NAME_RCP			rcp_keycode
#define SYS_ATTR_NAME_RCPK			rcp_ack
#define SYS_ATTR_NAME_RAP_LEGACY		rap_legacy
#define SYS_ATTR_NAME_RAP_STATUS		rap_status
#define SYS_ATTR_NAME_DEVCAP			devcap_old
#define SYS_ATTR_NAME_UCP			ucp_keycode
#define SYS_ATTR_NAME_UCPK			ucp_ack
#endif 
#define SYS_ATTR_NAME_I2C_REGS			i2c_regs
#define SYS_ATTR_NAME_SPAD			spad
#define SYS_ATTR_NAME_I2C_REGISTERS		i2c_registers
#define SYS_ATTR_NAME_ACCESSORY			accessory
#define SYS_ATTR_NAME_ID_IMPEDANCE       	id_impedance
#define SYS_ATTR_NAME_RETENTION			retention
#define SYS_ATTR_NAME_DEBUG_LEVEL		debug_level
#define SYS_ATTR_NAME_GPIO_INDEX		gpio_index
#define SYS_ATTR_NAME_GPIO_VALUE		gpio_value

#define SYS_ATTR_NAME_SWING_VALUE		htc_dbg_swing
#define SYS_ATTR_NAME_HDCP_FUNC			htc_hdcp_func

#ifdef DRIVER_SPEC_SYSFS 

#define 	SYS_ATTR_NAME_IN		in
#define 	SYS_ATTR_NAME_IN_STATUS		in_status
#define		SYS_ATTR_NAME_OUT		out
#define		SYS_ATTR_NAME_OUT_STATUS	out_status
#define		SYS_ATTR_NAME_INPUT_DEV		input_dev

#define SYS_OBJECT_NAME_RAP				rap
#define 	SYS_ATTR_NAME_RAP_IN			SYS_ATTR_NAME_IN
#define 	SYS_ATTR_NAME_RAP_IN_STATUS		SYS_ATTR_NAME_IN_STATUS
#define		SYS_ATTR_NAME_RAP_OUT			SYS_ATTR_NAME_OUT
#define		SYS_ATTR_NAME_RAP_OUT_STATUS		SYS_ATTR_NAME_OUT_STATUS
#define		SYS_ATTR_NAME_RAP_INPUT_DEV		SYS_ATTR_NAME_INPUT_DEV

#define SYS_OBJECT_NAME_RCP				rcp
#define 	SYS_ATTR_NAME_RCP_IN			SYS_ATTR_NAME_IN
#define 	SYS_ATTR_NAME_RCP_IN_STATUS		SYS_ATTR_NAME_IN_STATUS
#define		SYS_ATTR_NAME_RCP_OUT			SYS_ATTR_NAME_OUT
#define		SYS_ATTR_NAME_RCP_OUT_STATUS		SYS_ATTR_NAME_OUT_STATUS
#define		SYS_ATTR_NAME_RCP_INPUT_DEV		SYS_ATTR_NAME_INPUT_DEV

#define SYS_OBJECT_NAME_UCP				ucp
#define 	SYS_ATTR_NAME_UCP_IN			SYS_ATTR_NAME_IN
#define 	SYS_ATTR_NAME_UCP_IN_STATUS		SYS_ATTR_NAME_IN_STATUS
#define		SYS_ATTR_NAME_UCP_OUT			SYS_ATTR_NAME_OUT
#define		SYS_ATTR_NAME_UCP_OUT_STATUS		SYS_ATTR_NAME_OUT_STATUS
#define		SYS_ATTR_NAME_UCP_INPUT_DEV		SYS_ATTR_NAME_INPUT_DEV

#define SYS_OBJECT_NAME_DEVCAP				devcap
#define 	SYS_ATTR_NAME_DEVCAP_LOCAL_OFFSET	local_offset
#define		SYS_ATTR_NAME_DEVCAP_LOCAL		local
#define		SYS_ATTR_NAME_DEVCAP_REMOTE_OFFSET	remote_offset
#define		SYS_ATTR_NAME_DEVCAP_REMOTE		remote

#endif 


ssize_t show_connection_state(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);

	if (dev_context->mhl_flags & MHL_STATE_FLAG_CONNECTED) {
		return scnprintf(buf, PAGE_SIZE, "connected");
	} else {
		return scnprintf(buf, PAGE_SIZE, "not connected");
	}
}

#ifndef DRIVER_SPEC_SYSFS 
ssize_t show_rcp(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	extern bool input_dev_rcp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status = 0;

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (input_dev_rcp){
		status = scnprintf(buf, PAGE_SIZE, "input_dev_rcp");
	}else{
		if (dev_context->mhl_flags &
			(MHL_STATE_FLAG_RCP_SENT | MHL_STATE_FLAG_RCP_RECEIVED)) {
			status = scnprintf(buf, PAGE_SIZE, "0x%02x %s",
					dev_context->rcp_in_key_code,
					dev_context->mhl_flags & MHL_STATE_FLAG_RCP_SENT? "sent" : "received");
		}
	}

	up(&dev_context->isr_lock);

	return status;
}


ssize_t send_rcp(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	unsigned long			key_code;
	int						status = -EINVAL;

	MHL_TX_DBG_INFO(dev_context, "send_rcp received string: ""%s""\n", buf);

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit;
	}

	if (!(dev_context->mhl_flags & MHL_STATE_FLAG_CONNECTED))
		goto err_exit;

	if (strict_strtoul(buf, 0, &key_code)) {
		MHL_TX_DBG_ERR(dev_context, "Unable to convert key code string\n");
		goto err_exit;
	}

	if (key_code >= 0xFE) {
		MHL_TX_DBG_ERR(dev_context, "key code (0x%lx) is too large "\
				 "to be valid\n", key_code);
		goto err_exit;
	}

	dev_context->mhl_flags &= ~(MHL_STATE_FLAG_RCP_RECEIVED |
								MHL_STATE_FLAG_RCP_ACK |
								MHL_STATE_FLAG_RCP_NAK);
	dev_context->mhl_flags |= MHL_STATE_FLAG_RCP_SENT;
	dev_context->rcp_send_status = 0;
	dev_context->rcp_out_key_code = (u8)key_code;
	if (!si_mhl_tx_rcp_send(dev_context, (u8)key_code))
		goto err_exit;

	status = count;

err_exit:
	up(&dev_context->isr_lock);

	return status;
}


ssize_t send_rcp_ack(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	unsigned long	key_code = 0x100;	
	unsigned long	err_code = 0x100;
	char			*pStr;
	int				status = -EINVAL;

	extern bool input_dev_rcp;
	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (input_dev_rcp){
		
	}else{
		
		pStr = strstr(buf, "keycode=");
		if (pStr != NULL) {
			key_code = simple_strtoul(pStr + 8, NULL, 0);
			if (key_code > 0xFF) {
				MHL_TX_DBG_ERR(dev_context, "Unable to convert keycode string\n");
				return status;
			}
		} else {
			MHL_TX_DBG_ERR(dev_context, "Invalid string format, can't "\
					"find ""keycode"" value\n");
			return status;
		}

		pStr = strstr(buf, "errorcode=");
		if (pStr != NULL) {
			if(strict_strtoul(pStr + 10, 0, &err_code)) {
				MHL_TX_DBG_ERR(dev_context, "Unable to convert errorcode string\n");
				return status;
			}
		} else {
			MHL_TX_DBG_ERR(dev_context, "Invalid string format, can't "\
					"find ""errorcode"" value\n");
			return status;
		}

		if ((key_code > 0xFF) || (err_code > 0xFF)) {
			MHL_TX_DBG_ERR(dev_context, "Invalid key code or error code "\
					"specified, key code: 0x%02lx  error code: 0x%02lx\n",
					key_code, err_code);
			return status;
		}

		if (down_interruptible(&dev_context->isr_lock))
			return -ERESTARTSYS;

		if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
			status = -ENODEV;
		}else if (dev_context->mhl_flags & MHL_STATE_FLAG_CONNECTED) {

			if((key_code != dev_context->rcp_in_key_code)
				|| !(dev_context->mhl_flags & MHL_STATE_FLAG_RCP_RECEIVED)) {

				MHL_TX_DBG_ERR(dev_context, "Attempting to ACK a key code "\
							   "that was not received! try:0x%02x(%d)\n"
								,dev_context->rcp_in_key_code
								,dev_context->rcp_in_key_code);
			}else{
				status = count;
				if (err_code == 0) {
					if (!si_mhl_tx_rcpk_send(dev_context, (u8)key_code)) {
						status = -ENOMEM;
					}
				} else if (!si_mhl_tx_rcpe_send(dev_context, (u8)err_code)){
						status = -EINVAL;
				}
			}
		}
		up(&dev_context->isr_lock);
	}
	return status;
}


ssize_t show_rcp_ack(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status = -EINVAL;

	MHL_TX_DBG_INFO(dev_context, "called\n");

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->mhl_flags & (MHL_STATE_FLAG_RCP_ACK | MHL_STATE_FLAG_RCP_NAK)) {

		status = scnprintf(buf, PAGE_SIZE, "keycode=0x%02x errorcode=0x%02x",
				dev_context->rcp_out_key_code, dev_context->rcp_err_code);
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_ucp(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status = 0;

	MHL_TX_DBG_INFO(dev_context, "called keycode:0x%02x,0x02x\n"
					,dev_context->ucp_in_key_code
					,dev_context->ucp_out_key_code);
	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->mhl_flags &
		(MHL_STATE_FLAG_UCP_SENT | MHL_STATE_FLAG_UCP_RECEIVED)) {
		status = scnprintf(buf, PAGE_SIZE, "0x%02x %s"
				,dev_context->mhl_flags & MHL_STATE_FLAG_UCP_SENT
					?dev_context->ucp_out_key_code:dev_context->ucp_in_key_code
				,dev_context->mhl_flags & MHL_STATE_FLAG_UCP_SENT
					? "sent" : "received");
	}

	up(&dev_context->isr_lock);

	return status;
}


ssize_t send_ucp(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	unsigned long			key_code;
	int						status = -EINVAL;

	MHL_TX_DBG_INFO(dev_context, "received string: ""%s""\n", buf);

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit;
	}

	if (!(dev_context->mhl_flags & MHL_STATE_FLAG_CONNECTED))
		goto err_exit;

	if (strict_strtoul(buf, 0, &key_code)) {
		MHL_TX_DBG_ERR(dev_context, "Unable to convert key code string\n");
		goto err_exit;
	}

	if (key_code > 0xFF) {
		MHL_TX_DBG_ERR(dev_context, "ucp key code (0x%lx) is too large "\
				 "to be valid\n", key_code);
		goto err_exit;
	}

	dev_context->mhl_flags &= ~(MHL_STATE_FLAG_UCP_RECEIVED |
								MHL_STATE_FLAG_UCP_ACK |
								MHL_STATE_FLAG_UCP_NAK);
	dev_context->mhl_flags |= MHL_STATE_FLAG_UCP_SENT;
	dev_context->ucp_out_key_code = (u8)key_code;
	if (!si_mhl_tx_ucp_send(dev_context, (u8)key_code))
		goto err_exit;

	status = count;

err_exit:
	up(&dev_context->isr_lock);

	return status;
}


ssize_t send_ucp_ack(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	unsigned long	key_code = 0x100;	
	unsigned long	err_code = 0x100;
	char			*pStr;
	int				status = -EINVAL;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	
	pStr = strstr(buf, "keycode=");
	if (pStr != NULL) {
		key_code = simple_strtoul(pStr + 8, NULL, 0);
		if (key_code > 0xFF) {
			MHL_TX_DBG_ERR(dev_context, "Unable to convert keycode string\n");
			goto err_exit_2;
		}
	} else {
		MHL_TX_DBG_ERR(dev_context, "Invalid string format, can't "\
				"find ""keycode"" value\n");
		goto err_exit_2;
	}

	pStr = strstr(buf, "errorcode=");
	if (pStr != NULL) {
		if(strict_strtoul(pStr + 10, 0, &err_code)) {
			MHL_TX_DBG_ERR(dev_context, "Unable to convert errorcode string\n");
			goto err_exit_2;
		}
	} else {
		MHL_TX_DBG_ERR(dev_context, "Invalid string format, can't "\
				"find ""errorcode"" value\n");
		goto err_exit_2;
	}

	if ((key_code > 0xFF) || (err_code > 0xFF)) {
		MHL_TX_DBG_ERR(dev_context, "Invalid key code or error code "\
				"specified, key code: 0x%02lx  error code: 0x%02lx\n",
				key_code, err_code);
		goto err_exit_2;
	}

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit_1;
	}

	if (dev_context->mhl_flags & MHL_STATE_FLAG_CONNECTED) {

		if((key_code != dev_context->ucp_in_key_code)
			|| !(dev_context->mhl_flags & MHL_STATE_FLAG_UCP_RECEIVED)) {

			MHL_TX_DBG_ERR(dev_context, "Attempting to ACK a key code that "\
						   "was not received!\n");
			goto err_exit_1;
		}

		if (err_code == 0) {
			if (!si_mhl_tx_ucpk_send(dev_context, (u8)key_code)) {
				status = -ENOMEM;
				goto err_exit_1;
			}
		} else {
			if (!si_mhl_tx_ucpe_send(dev_context, (u8)err_code)) {
				status = -ENOMEM;
				goto err_exit_1;
			}
		}

		status = count;
	}

err_exit_1:
	up(&dev_context->isr_lock);

err_exit_2:
	return status;
}


ssize_t show_ucp_ack(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status = -EINVAL;

	MHL_TX_DBG_INFO(dev_context, "called\n");

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->mhl_flags & (MHL_STATE_FLAG_UCP_ACK | MHL_STATE_FLAG_UCP_NAK)) {

		status = scnprintf(buf, PAGE_SIZE, "keycode=0x%02x errorcode=0x%02x",
				dev_context->ucp_out_key_code, dev_context->ucp_err_code);
	}

	up(&dev_context->isr_lock);

	return status;
}

/*
 * show_rap() - Handle read request to the rap attribute file.
 *
 * Reads from this file return a string value indicating the last
 * Request Action Protocol (RAP) request received.
 *
 * The return value is the number characters written to buf, or EAGAIN
 * if the driver is busy and cannot service the read request immediately.
 * If EAGAIN is returned the caller should wait a little and retry the
 * read.
 */
ssize_t show_rap(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status = -EINVAL;


	MHL_TX_DBG_INFO(dev_context, "called last sub-command:0x%02x\n",dev_context->rap_in_sub_command);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
		goto err_exit;
	}
	*buf = '\0';
	if (MHL_RAP_POLL == dev_context->rap_in_sub_command)
		status = scnprintf(buf, PAGE_SIZE, "poll");
	else if (MHL_RAP_CONTENT_ON ==  dev_context->rap_in_sub_command)
		status = scnprintf(buf, PAGE_SIZE, "content_on");
	else if (MHL_RAP_CONTENT_OFF ==  dev_context->rap_in_sub_command)
		status = scnprintf(buf, PAGE_SIZE, "content_off");
	MHL_TX_DBG_INFO(dev_context,"buf:%c%s%c\n",'"',buf,'"');

err_exit:
	up(&dev_context->isr_lock);

	return status;
}

ssize_t send_rap(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit;
	}

	if (strnicmp("poll", buf, count - 1) == 0) {
		if (!si_mhl_tx_rap_send(dev_context, MHL_RAP_POLL))
			status = -EPERM;

	} else if (strnicmp("content_on", buf, count - 1) == 0) {
		if (!si_mhl_tx_rap_send(dev_context, MHL_RAP_CONTENT_ON))
			status = -EPERM;

	} else if (strnicmp("content_off", buf, count - 1) == 0) {
		if (!si_mhl_tx_rap_send(dev_context, MHL_RAP_CONTENT_OFF))
			status = -EPERM;

	} else {
		MHL_TX_DBG_ERR(dev_context, "Invalid parameter %s received\n", buf);
		status = -EINVAL;
	}

err_exit:
	up(&dev_context->isr_lock);

	return status;
}
ssize_t show_rap_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status = -EINVAL;


	MHL_TX_DBG_INFO(dev_context, "called last sub-command:0x%02x\n",dev_context->rap_in_sub_command);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
		goto err_exit;
	}
	*buf = '\0';
	if (dev_context->mhl_flags & MHL_STATE_APPLICATION_RAP_BUSY){
		status = scnprintf(buf, PAGE_SIZE, "busy");
	}else{
		status = scnprintf(buf, PAGE_SIZE, "ready");
	}
	MHL_TX_DBG_INFO(dev_context,"buf:%c%s%c\n",'"',buf,'"');

err_exit:
	up(&dev_context->isr_lock);

	return status;
}

ssize_t set_rap_status(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit;
	}

	if (strnicmp("busy", buf, count - 1) == 0) {
		dev_context->mhl_flags |= MHL_STATE_APPLICATION_RAP_BUSY;
	} else if (strnicmp("ready", buf, count - 1) == 0) {
		dev_context->mhl_flags &= ~MHL_STATE_APPLICATION_RAP_BUSY;
	} else {
		MHL_TX_DBG_ERR(dev_context, "Invalid parameter %s received\n", buf);
		status = -EINVAL;
	}

err_exit:
	up(&dev_context->isr_lock);

	return status;
}


ssize_t select_dev_cap(struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	unsigned long			offset;
	int						status = -EINVAL;


	MHL_TX_DBG_INFO(dev_context, "received string: ""%s""\n", buf);

	if (strict_strtoul(buf, 0, &offset)) {
		MHL_TX_DBG_ERR(dev_context, "Unable to convert register "\
				"offset string\n");
		goto err_exit;
	}

	if (offset > 0x0F) {
		MHL_TX_DBG_INFO(dev_context,
				"dev cap offset (0x%lx) is too large to be valid\n", offset);
		goto err_exit;
	}

	dev_context->dev_cap_remote_offset = (u8)offset;
	status = count;

err_exit:
	return status;
}


/*
 * show_dev_cap() - Handle read request to the devcap attribute file.
 *
 * Reads from this file return the hexadecimal string value of the last
 * Device Capability register offset written to this file.
 *
 * The return value is the number characters written to buf, or EAGAIN
 * if the driver is busy and cannot service the read request immediately.
 * If EAGAIN is returned the caller should wait a little and retry the
 * read.
 *
 * The format of the string returned in buf is:
 * 	"offset:<offset>=<regvalue>
 * 	where:	<offset>	is the last Device Capability register offset
 * 						written to this file
 * 			<regvalue>	the currentl value of the Device Capability register
 * 						specified in offset
 */
ssize_t show_dev_cap(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	uint8_t					regValue;
	int						status = -EINVAL;


	MHL_TX_DBG_INFO(dev_context, "called\n");

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit;
	}

	if (dev_context->mhl_flags & MHL_STATE_FLAG_CONNECTED) {

		status = si_mhl_tx_get_peer_dev_cap_entry(dev_context,
											dev_context->dev_cap_remote_offset,
											&regValue);
		if (status != 0) {
			status = -EAGAIN;
			goto err_exit;
		}
		status = scnprintf(buf, PAGE_SIZE, "offset:0x%02x=0x%02x",
						   dev_context->dev_cap_remote_offset, regValue);
	}

err_exit:
	up(&dev_context->isr_lock);

	return status;
}
#endif 


/*
 * send_scratch_pad() - Handle write request to the spad attribute file.
 *
 * This file is used to either initiate a write to the scratch pad registers
 * of an attached device, or to set the offset and byte count for a subsequent
 * read from the local scratch pad registers.
 *
 * The format of the string in buf must be:
 * 	offset=<offset_value> length=<Length_value> \
 * 	data=data_byte_0 ... data_byte_length-1
 * 	where:	<offset_value>	specifies the starting register offset to begin
 * 							read/writing within the scratch pad register space
 * 			<length_value>	number of scratch pad registers to be written/read
 * 			data_byte		space separated list of <length_value> data bytes
 * 							to be written.  If no data bytes are present then
 * 							the write to this file will only be used to set
 * 							the offset and length for a subsequent read from
 * 							this file.
 */
ssize_t send_scratch_pad(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	unsigned long	offset = 0x100;		
	unsigned long	length = 0x100;
	unsigned long	value;
	u8				data[MAX_SCRATCH_PAD_TRANSFER_SIZE];
	u8				idx;
	char			*str;
	char			*endptr;
	enum scratch_pad_status	scratch_pad_status;
	int				status = -EINVAL;

	MHL_TX_DBG_ERR(dev_context, "received string: ""%s""\n", buf);

	str = strstr(buf, "offset=");
	if (str != NULL) {
		offset = simple_strtoul(str + 7, NULL, 0);
		if (offset > SCRATCH_PAD_SIZE) {
			MHL_TX_DBG_ERR(dev_context, "Invalid offset value entered\n");
			goto err_exit_2;
		}
	} else {
		MHL_TX_DBG_ERR(dev_context, "Invalid string format, can't "\
				"find ""offset"" value\n");
		goto err_exit_2;
	}

	str = strstr(buf, "length=");
	if (str != NULL) {
		length = simple_strtoul(str + 7, NULL, 0);
		if (length > MAX_SCRATCH_PAD_TRANSFER_SIZE) {
			MHL_TX_DBG_ERR(dev_context, "Transfer length too large\n");
			goto err_exit_2;
		}
	} else {
		MHL_TX_DBG_ERR(dev_context, "Invalid string format, can't "
				"find ""length"" value\n");
		goto err_exit_2;
	}

	str = strstr(buf, "data=");
	if (str != NULL) {

		str += 5;
		endptr = str;
		for(idx = 0; idx < length; idx++) {

			endptr += strspn(endptr, white_space);
			str = endptr;
			if (*str == 0) {
				MHL_TX_DBG_ERR(dev_context, "Too few data values provided\n");
				goto err_exit_2;
			}

			value = simple_strtoul(str, &endptr, 0);
			if (value > 0xFF) {
				MHL_TX_DBG_ERR(dev_context, "Invalid scratch pad data detected\n");
				goto err_exit_2;
			}

			data[idx] = value;
		}

	} else {
		idx = 0;
	}

	if ((offset + length) > SCRATCH_PAD_SIZE) {
		MHL_TX_DBG_ERR(dev_context, "Invalid offset/length combination" \
				"entered");
		goto err_exit_2;
	}

	dev_context->spad_offset = offset;
	dev_context->spad_xfer_length = length;

	if (idx == 0) {
		MHL_TX_DBG_INFO(dev_context, "No data specified, storing offset "\
				"and length for subsequent scratch pad read\n");

		goto err_exit_2;
	}

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit_1;
	}

	if (!(dev_context->mhl_flags & MHL_STATE_FLAG_CONNECTED) ||
		(length < ADOPTER_ID_SIZE) ||
		(offset > (SCRATCH_PAD_SIZE - ADOPTER_ID_SIZE)) ||
		(offset + length > SCRATCH_PAD_SIZE)) {
		status = -EINVAL;
		goto err_exit_1;
	}

	dev_context->mhl_flags |= MHL_STATE_FLAG_SPAD_SENT;
	dev_context->spad_send_status = 0;

	scratch_pad_status = si_mhl_tx_request_write_burst(dev_context,
											offset, length, data);

	switch (scratch_pad_status) {
		case SCRATCHPAD_SUCCESS:
			/* On success return the number of bytes written to this file */
			status = count;
			break;

		case SCRATCHPAD_BUSY:
			status = -EAGAIN;
			break;

		default:
			status = -EFAULT;
			break;
	}

err_exit_1:
	up(&dev_context->isr_lock);

err_exit_2:
	return status;
}


/*
 * show_scratch_pad() - Handle read request to the spad attribute file.
 *
 * Reads from this file return one or more scratch pad register values
 * in hexadecimal string format.  The registers returned are specified
 * by the offset and length values previously written to this file.
 *
 * The return value is the number characters written to buf, or EAGAIN
 * if the driver is busy and cannot service the read request immediately.
 * If EAGAIN is returned the caller should wait a little and retry the
 * read.
 *
 * The format of the string returned in buf is:
 * 	"offset:<offset> length:<lenvalue> data:<datavalues>
 * 	where:	<offset>	is the last scratch pad register offset
 * 						written to this file
 * 			<lenvalue>	is the last scratch pad register transfer length
 * 						written to this file
 * 			<datavalue>	space separated list of <lenvalue> scratch pad
 * 						register values in OxXX format
 */
ssize_t show_scratch_pad(struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	u8						data[MAX_SCRATCH_PAD_TRANSFER_SIZE];
	u8						idx;
	enum scratch_pad_status	scratch_pad_status;
	int						status = -EINVAL;


	MHL_TX_DBG_INFO(dev_context, "called\n");

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit;
	}

	if (dev_context->mhl_flags & MHL_STATE_FLAG_CONNECTED) {

		scratch_pad_status  = si_get_scratch_pad_vector(dev_context,
												dev_context->spad_offset,
												dev_context->spad_xfer_length,
												data);

		switch (scratch_pad_status) {
			case SCRATCHPAD_SUCCESS:
				status = scnprintf(buf, PAGE_SIZE, "offset:0x%02x " \
								   "length:0x%02x data:",
								   dev_context->spad_offset,
								   dev_context->spad_xfer_length);

				for (idx = 0; idx < dev_context->spad_xfer_length; idx++) {
					status += scnprintf(&buf[status], PAGE_SIZE, "0x%02x ",
										data[idx]);
				}
				break;

			case SCRATCHPAD_BUSY:
				status = -EAGAIN;
				break;

			default:
				status = -EFAULT;
				break;
		}
	}

err_exit:
	up(&dev_context->isr_lock);

	return status;
}

/*
 * set_i2c() - Handle write request to the debug attribute file.
 *
 * This file is used to either perform a write to registers of the transmitter
 * or to set the address, offset and byte count for a subsequent from the
 * register(s) of the transmitter.
 *
 * The format of the string in buf must be:
 * 	address=<pageaddr> offset=<offset_value> length=<Length_value> \
 * 	data=data_byte_0 ... data_byte_length-1
 * 	where: <pageaddr>		specifies the I2C register page of the register(s)
 * 							to be written/read
 * 			<offset_value>	specifies the starting register offset within the
 * 							register page to begin writing/reading
 * 			<length_value>	number registers to be written/read
 * 			data_byte		space separated list of <length_value> data bytes
 * 							to be written.  If no data bytes are present then
 * 							the write to this file will only be used to set
 * 							the  page address, offset and length for a
 * 							subsequent read from this file.
 */
ssize_t set_i2c(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	unsigned long	address = 0x100;		
	unsigned long	offset = 0x100;
	unsigned long	length = 0x100;
	unsigned long	value;
	u8				data[MAX_DEBUG_TRANSFER_SIZE];
	u8				idx;
	char			*str;
	char			*endptr;
	int				status = -EINVAL;

	MHL_TX_DBG_INFO(dev_context, "received string: ""%s""\n", buf);

	str = strstr(buf, "address=");
	if (str != NULL) {
		address = simple_strtoul(str + 8, NULL, 0);
		if (address > 0xFF) {
			MHL_TX_DBG_ERR(dev_context, "Invalid page address: 0x%02lx" \
					"specified\n", address);
			goto err_exit_2;
		}
	} else {
		MHL_TX_DBG_ERR(dev_context, "Invalid string format, can't "\
				"find ""address"" parameter\n");
		goto err_exit_2;
	}

	str = strstr(buf, "offset=");
	if (str != NULL) {
		offset = simple_strtoul(str + 7, NULL, 0);
		if (offset > 0xFF) {
			MHL_TX_DBG_ERR(dev_context, "Invalid page offset: 0x%02lx" \
					"specified\n", offset);
			goto err_exit_2;
		}
	} else {
		MHL_TX_DBG_ERR(dev_context, "Invalid string format, can't "\
				"find ""offset"" value\n");
		goto err_exit_2;
	}

	str = strstr(buf, "length=");
	if (str != NULL) {
		length = simple_strtoul(str + 7, NULL, 0);
		if (length > MAX_DEBUG_TRANSFER_SIZE) {
			MHL_TX_DBG_ERR(dev_context, "Transfer size 0x%02lx is too "\
					"large\n", length);
			goto err_exit_2;
		}
	} else {
		MHL_TX_DBG_ERR(dev_context, "Invalid string format, can't "\
				"find ""length"" value\n");
		goto err_exit_2;
	}

	str = strstr(buf, "data=");
	if (str != NULL) {

		str += 5;
		endptr = str;
		for(idx = 0; idx < length; idx++) {
			endptr += strspn(endptr, white_space);
			str = endptr;
			if (*str == 0) {
				MHL_TX_DBG_ERR(dev_context, "Too few data values provided\n");
				goto err_exit_2;
			}

			value = simple_strtoul(str, &endptr, 0);

			if (value > 0xFF) {
				MHL_TX_DBG_ERR(dev_context, "Invalid register data "\
						"value detected\n");
				goto err_exit_2;
			}

			data[idx] = value;
		}

	} else {
		idx = 0;
	}

	if ((offset + length) > 0x100) {
		MHL_TX_DBG_ERR(dev_context
			, "Invalid offset/length combination entered 0x%02x/0x%02x"
			, offset, length);
		goto err_exit_2;
	}

	dev_context->debug_i2c_address = address;
	dev_context->debug_i2c_offset = offset;
	dev_context->debug_i2c_xfer_length = length;

	if (idx == 0) {
		MHL_TX_DBG_INFO(dev_context, "No data specified, storing address "\
				 "offset and length for subsequent debug read\n");
		goto err_exit_2;
	}

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit_1;
	}

	status =  dev_context->drv_info->mhl_device_dbg_i2c_reg_xfer(
												&dev_context->drv_context,
												address, offset, length,
												DEBUG_I2C_WRITE, data);
	if (status == 0)
		status = count;

err_exit_1:
	up(&dev_context->isr_lock);

err_exit_2:
	return status;
}

/*
 * show_i2c()	- Handle read request to the debug attribute file.
 *
 * Reads from this file return one or more transmitter register values in
 * hexadecimal string format.  The registers returned are specified by the
 * address, offset and length values previously written to this file.
 *
 * The return value is the number characters written to buf, or an error
 * code if the I2C read fails.
 *
 * The format of the string returned in buf is:
 * 	"address:<pageaddr> offset:<offset> length:<lenvalue> data:<datavalues>
 * 	where:	<pageaddr>	is the last I2C register page address written
 * 						to this file
 * 			<offset>	is the last register offset written to this file
 * 			<lenvalue>	is the last register transfer length written
 * 						to this file
 * 			<datavalue>	space separated list of <lenvalue> register
 * 						values in OxXX format
 */
ssize_t show_i2c(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	u8						data[MAX_DEBUG_TRANSFER_SIZE];
	u8						idx;
	int						status = -EINVAL;

	MHL_TX_DBG_INFO(dev_context, "called\n");

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto no_dev;
	}

	status =  dev_context->drv_info->mhl_device_dbg_i2c_reg_xfer(
											&dev_context->drv_context,
											dev_context->debug_i2c_address,
											dev_context->debug_i2c_offset,
											dev_context->debug_i2c_xfer_length,
											DEBUG_I2C_READ, data);
no_dev:
	up(&dev_context->isr_lock);

	if (status == 0) {

		status = scnprintf(buf, PAGE_SIZE, "address:0x%02x offset:0x%02x " \
						   "length:0x%02x data:",
							dev_context->debug_i2c_address,
							dev_context->debug_i2c_offset,
							dev_context->debug_i2c_xfer_length);

		for (idx = 0; idx < dev_context->debug_i2c_xfer_length; idx++) {
			status += scnprintf(&buf[status], PAGE_SIZE, "0x%02x ",
								data[idx]);
		}
	}

	return status;
}
/*
 * show_accessory() - Handle read request to the accessory attribute file.
 *
 * The return value is the number characters written to buf, or EAGAIN
 * if the driver is busy and cannot service the read request immediately.
 * If EAGAIN is returned the caller should wait a little and retry the
 * read.
 */
ssize_t get_accessory(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;
	char *accessory_string="none";
	int	accessory_config=0;



	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
		goto err_exit;
	}
	if (!dev_context->drv_context) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
		goto err_exit;
	}
	accessory_config = si_mhl_tx_get_accessory_switch_config(
				(struct drv_hw_context *)&dev_context->drv_context
				,&accessory_string
				);
	status = scnprintf(buf, PAGE_SIZE, "config=0x%02x device=%c%s%c"
		, accessory_config
		, '"',accessory_string,'"'
		);
	MHL_TX_DBG_INFO(dev_context,"buf:%c%s%c\n",'"',buf,'"');

err_exit:
	up(&dev_context->isr_lock);

	return status;
}

ssize_t set_accessory(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;
	char *str;
	int config;
	const char *key="config=";

	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit;
	}
	str = strstr(buf, key);
	if (str != NULL) {
		config = simple_strtoul(str + strlen(key), NULL, 0);
		si_mhl_tx_set_accessory_switch_config((struct drv_hw_context *)&dev_context->drv_context,config);
	} else {
		MHL_TX_DBG_ERR(dev_context, "Invalid string format, can't "
				"find \"config=\" parameter\n");
	}

err_exit:
	up(&dev_context->isr_lock);

	return status;
}


/*
 * show_id_impedance_measurement() - Handle read request to the id_impedance_measurement attribute file.
 *
 * The return value is the number characters written to buf, or EAGAIN
 * if the driver is busy and cannot service the read request immediately.
 * If EAGAIN is returned the caller should wait a little and retry the
 * read.
 */
ssize_t show_id_impedance_measurement(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;
	char *id_impedance_measurement_string="write only";
	int	id_impedance_measurement_index=0;



	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
		goto err_exit;
	}
#if 0 
	id_impedance_measurement_index = si_mhl_tx_get_accessory_switch_config(
				(struct drv_hw_context *)&dev_context->drv_context
				,&id_impedance_measurement_string
				);
#endif 
	status = scnprintf(buf, PAGE_SIZE, "config=0x%02x device=%c%s%c"
		, id_impedance_measurement_index
		, '"',id_impedance_measurement_string,'"'
		);
	MHL_TX_DBG_INFO(dev_context,"buf:%c%s%c\n",'"',buf,'"');

err_exit:
	up(&dev_context->isr_lock);

	return status;
}
ssize_t trigger_id_impedance_measurement(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;
	char *str;
	int config;
	const char *key="go";

	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit;
	}
	str = strstr(buf, key);
	if (str != NULL) {
		config = simple_strtoul(str + strlen(key), NULL, 0);
		si_mhl_tx_id_impedance_measurement(
			(struct drv_hw_context *)&dev_context->drv_context);
	} else {
		MHL_TX_DBG_ERR(dev_context, "Invalid string format, can't "
				"find \"config=\" parameter\n");
	}

err_exit:
	up(&dev_context->isr_lock);

	return status;
}


static int retention_index=0;
static int retention_length=NUM_RETENTION_REGS;
/*
 * show_retention() - Handle read request to the retention attribute file.
 *
 * The return value is the number characters written to buf, or EAGAIN
 * if the driver is busy and cannot service the read request immediately.
 * If EAGAIN is returned the caller should wait a little and retry the
 * read.
 */
ssize_t get_retention(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;
	int	retention_index=0;
	retention_data_t data;
	int idx;

	memset(&data,0,sizeof(data));

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
		goto err_exit;
	}
	if ( 0 > si_mhl_tx_get_retention_range(
				(struct drv_hw_context *)&dev_context->drv_context
				,data
				)){
		status = -ENODEV;
	}else{
		MHL_TX_DBG_INFO(dev_context,"\n");
		status = scnprintf(buf, PAGE_SIZE, "index=0x%02x length=0x%02x data="
			, retention_index
			, retention_length);


		MHL_TX_DBG_INFO(dev_context,"%d\n",status);
		for (idx = 0; idx < retention_length;++idx){
			status += scnprintf(&buf[status], PAGE_SIZE, "0x%02x "
				, data[retention_index+idx]
				);
		}
	}

err_exit:
	up(&dev_context->isr_lock);

	return status;
}

ssize_t set_retention(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;
	char *str;
	const char *index_key="index=";

	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit;
	}
	str = strstr(buf, index_key);
	if (str == NULL) {
		MHL_TX_DBG_ERR(dev_context, "Invalid string format, can't "
				"find \"index=\" parameter\n");
	} else {
		int index;
		const char *length_key="length=";
		index = simple_strtoul(str + strlen(index_key), NULL, 0);
		str = strstr(buf, length_key);
		if (str == NULL) {
			MHL_TX_DBG_ERR(dev_context, "Invalid string format, can't "\
					"find ""length"" value\n");
		} else {
			int length;
			length = simple_strtoul(str + strlen(length_key), NULL, 0);
			if ((retention_index + retention_length) > NUM_RETENTION_REGS) {
				MHL_TX_DBG_ERR(dev_context
					, "Invalid index/length combination entered 0x%02x/0x%02x"
					, index, length);
			} else{
				const char *data_key="data=";
				
				retention_index = index;
				retention_length = length;
				str = strstr(buf, data_key);
				if (str != NULL) {

					int idx;
					unsigned long value;
					char *endptr;
					retention_data_t data;
					memset(&data,0,sizeof(data));
					str += strlen(data_key);
					endptr = str;
					si_mhl_tx_get_retention_range(
						(struct drv_hw_context *)&dev_context->drv_context
						,data);
					for(idx = 0; idx < length; idx++) {
						endptr += strspn(endptr, white_space);
						str = endptr;
						if (*str == 0) {
							MHL_TX_DBG_ERR(dev_context, "Too few data values provided\n");
							break;
						}

						value = simple_strtoul(str, &endptr, 0);

						if (value > 0xFF) {
							MHL_TX_DBG_ERR(dev_context, "Invalid register data "\
									"value detected\n");
							break;
						}

						data[retention_index+idx] = value;
					}
					si_mhl_tx_set_retention_range(
						(struct drv_hw_context *)&dev_context->drv_context
						,data);

				}
			}
		}
	}


err_exit:
	up(&dev_context->isr_lock);

	return status;
}


/*
 * show_debug_level() - Handle read request to the debug_level attribute file.
 *
 * The return value is the number characters written to buf, or EAGAIN
 * if the driver is busy and cannot service the read request immediately.
 * If EAGAIN is returned the caller should wait a little and retry the
 * read.
 */
ssize_t get_debug_level(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;
	extern int debug_level;


	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
		goto err_exit;
	}

	status = scnprintf(buf, PAGE_SIZE, "level=%d",debug_level );
	MHL_TX_DBG_INFO(dev_context,"buf:%c%s%c\n",'"',buf,'"');

err_exit:
	up(&dev_context->isr_lock);

	return status;
}

ssize_t set_debug_level(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;
	extern int debug_level;

	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit;
	}
	debug_level = simple_strtol(buf, NULL, 0);

err_exit:
	up(&dev_context->isr_lock);

	return status;
}


/*
 * show_gpio_index() - Handle read request to the gpio_index attribute file.
 *
 * The return value is the number characters written to buf, or EAGAIN
 * if the driver is busy and cannot service the read request immediately.
 * If EAGAIN is returned the caller should wait a little and retry the
 * read.
 */
ssize_t get_gpio_index(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;
	extern int gpio_index;


	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
		goto err_exit;
	}

	status = scnprintf(buf, PAGE_SIZE, "%d",gpio_index );
	MHL_TX_DBG_INFO(dev_context,"buf:%c%s%c\n",'"',buf,'"');

err_exit:
	up(&dev_context->isr_lock);

	return status;
}

ssize_t set_gpio_index(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;
	extern int gpio_index;
	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit;
	}
	gpio_index = simple_strtol(buf, NULL, 0);
	if (!gpio_is_valid(gpio_index)) {
		MHL_TX_DBG_ERR(NULL, "invalid gpio #\n");
		status = -ENODEV;
		goto err_exit;
	}

	MHL_TX_DBG_INFO(,"gpio: %d\n",gpio_index);

err_exit:
	up(&dev_context->isr_lock);

	return status;
}

/*
 * show_gpio_level() - Handle read request to the gpio_level attribute file.
 *
 * The return value is the number characters written to buf, or EAGAIN
 * if the driver is busy and cannot service the read request immediately.
 * If EAGAIN is returned the caller should wait a little and retry the
 * read.
 */
ssize_t get_gpio_level(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;
	extern int gpio_index;
	int gpio_level;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
		goto err_exit;
	}
	if (!gpio_is_valid(gpio_index)) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
		goto err_exit;
	}
	gpio_level = gpio_get_value(gpio_index);
	status = scnprintf(buf, PAGE_SIZE, "%d",gpio_level );
	MHL_TX_DBG_INFO(dev_context,"buf:%c%s%c\n",'"',buf,'"');

err_exit:
	up(&dev_context->isr_lock);

	return status;
}

ssize_t set_gpio_level(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;
	int gpio_level;
	extern int gpio_index;
	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
		goto err_exit;
	}
	gpio_level = simple_strtol(buf, NULL, 0);
	MHL_TX_DBG_INFO(,"gpio: %d<-%d\n",gpio_index,gpio_level);
	if (!gpio_is_valid(gpio_index)) {
		status = -ENODEV;
		goto err_exit;
	}
	gpio_set_value(gpio_index, gpio_level);

err_exit:
	up(&dev_context->isr_lock);

	return status;
}

/*
 * show_debug_swing_value() - Handle read request to the swing_value attribute file.
 *
 * The return value is the number characters written to buf, or EAGAIN
 * if the driver is busy and cannot service the read request immediately.
 * If EAGAIN is returned the caller should wait a little and retry the
 * read.
 */
ssize_t show_debug_swing_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if(dev_context->debug_swing_value == -1)
		status = scnprintf(buf, PAGE_SIZE, "debug_swing not set\n");
	else
		status = scnprintf(buf, PAGE_SIZE, "debug_swing:0x%x\n", dev_context->debug_swing_value);

	return status;
}

ssize_t set_debug_swing_value(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	unsigned long value = 0;

	if (strict_strtoul(buf, 10, &value))
		return -EINVAL;
	dev_context->debug_swing_value = value;
	dev_context->debug_swing_value &= 0x7;
	return count;
}

/*
 * show_hdcp_status() - Handle read request to the debug_hdcp_status attribute file.
 *
 * The return value is the number characters written to buf, or EAGAIN
 * if the driver is busy and cannot service the read request immediately.
 * If EAGAIN is returned the caller should wait a little and retry the
 * read.
 */
ssize_t show_hdcp_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	status = scnprintf(buf, PAGE_SIZE, "debug_hdcp_status:%#x\n", dev_context->hdcp_status);
	return status;
}

ssize_t set_hdcp_status(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int i;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);

	if (sscanf(buf, "%d", &i) == 1 || sscanf(buf, "%d", &i) == 0) {
		dev_context->hdcp_status = i;
	} else
		pr_info("[MHL] parameter error for HDCP, 0 or 1 only");
	return count;
}

#define MAX_EVENT_STRING_LEN 128
void mhl_event_notify(struct mhl_dev_context *dev_context, u32 event,
					  u32 event_param, void *data)
{
	char	event_string[MAX_EVENT_STRING_LEN];
	char	*envp[] = {event_string, NULL};
	char	*buf;
	u32		length;
	u32		count;
	int		idx;
	extern bool input_dev_rcp;

	MHL_TX_DBG_INFO(dev_context, "called, event: 0x%08x "\
			 "event_param: 0x%08x\n", event, event_param);

	dev_context->pending_event = event;
	dev_context->pending_event_data = event_param;

	switch(event) {

	case MHL_TX_EVENT_CONNECTION:
		dev_context->mhl_flags |= MHL_STATE_FLAG_CONNECTED;

#ifdef MEDIA_DATA_TUNNEL_SUPPORT
		#if 0	
		mdt_init(dev_context);
		#endif
#endif
		if (input_dev_rcp){
			init_rcp_input_dev(dev_context);
		}
		sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
					 __stringify(SYS_ATTR_NAME_CONN));

		strncpy(event_string, "MHLEVENT=connected", MAX_EVENT_STRING_LEN);
		kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		break;

	case MHL_TX_EVENT_DISCONNECTION:
		dev_context->mhl_flags = 0;
		dev_context->rcp_in_key_code = 0;
		dev_context->rcp_out_key_code = 0;
		dev_context->rcp_err_code = 0;
		dev_context->rcp_send_status = 0;
		dev_context->ucp_in_key_code = 0;
		dev_context->ucp_out_key_code = 0;
		dev_context->ucp_err_code = 0;
		dev_context->spad_send_status = 0;

#ifdef MEDIA_DATA_TUNNEL_SUPPORT
		mdt_destroy(dev_context);
#endif
		if (input_dev_rcp){
			destroy_rcp_input_dev(dev_context);
		}
		sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
					 __stringify(SYS_ATTR_NAME_CONN));

		strncpy(event_string, "MHLEVENT=disconnected", MAX_EVENT_STRING_LEN);
		kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		break;

	case MHL_TX_EVENT_RCP_RECEIVED:

		if (input_dev_rcp){
			int result;
			result =generate_rcp_input_event(dev_context, (uint8_t)event_param);
			if (0 == result)
				si_mhl_tx_rcpk_send(dev_context, (uint8_t)event_param);
			else
				si_mhl_tx_rcpe_send(dev_context,
									MHL_RCPE_STATUS_INEEFECTIVE_KEY_CODE);
		}else{
			dev_context->mhl_flags &= ~MHL_STATE_FLAG_RCP_SENT;
			dev_context->mhl_flags |= MHL_STATE_FLAG_RCP_RECEIVED;
			dev_context->rcp_in_key_code = event_param;

			sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
						 __stringify(SYS_ATTR_NAME_RCP));

			snprintf(event_string, MAX_EVENT_STRING_LEN,
					"MHLEVENT=received_RCP key code=0x%02x", event_param);
			kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		}
		break;

	case MHL_TX_EVENT_RCPK_RECEIVED:
		if ((dev_context->mhl_flags & MHL_STATE_FLAG_RCP_SENT)
			&& (dev_context->rcp_out_key_code == event_param)) {

			dev_context->mhl_flags |= MHL_STATE_FLAG_RCP_ACK;

			MHL_TX_DBG_INFO(dev_context, "Generating RCPK received event, "
					 "keycode: 0x%02x\n", event_param);

			sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
						 __stringify(SYS_ATTR_NAME_RCPK));

			snprintf(event_string, MAX_EVENT_STRING_LEN,
					"MHLEVENT=received_RCPK key code=0x%02x", event_param);
			kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		} else {
			MHL_TX_DBG_ERR(dev_context, "Ignoring unexpected RCPK received "
					"event, keycode: 0x%02x\n", event_param);
		}
		break;

	case MHL_TX_EVENT_RCPE_RECEIVED:
		if (input_dev_rcp){
			
		}else{
			if (dev_context->mhl_flags & MHL_STATE_FLAG_RCP_SENT) {

				dev_context->rcp_err_code = event_param;
				dev_context->mhl_flags |= MHL_STATE_FLAG_RCP_NAK;

				MHL_TX_DBG_INFO(dev_context, "Generating RCPE received event, "
						 "error code: 0x%02x\n", event_param);

				sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
							 __stringify(SYS_ATTR_NAME_RCPK));

				snprintf(event_string, MAX_EVENT_STRING_LEN,
						"MHLEVENT=received_RCPE error code=0x%02x", event_param);
				kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
			} else {
				MHL_TX_DBG_ERR(dev_context, "Ignoring unexpected RCPE received "
						"event, error code: 0x%02x\n", event_param);
			}
		}
		break;

	case MHL_TX_EVENT_UCP_RECEIVED:
		dev_context->mhl_flags &= ~MHL_STATE_FLAG_UCP_SENT;
		dev_context->mhl_flags |= MHL_STATE_FLAG_UCP_RECEIVED;
		dev_context->ucp_in_key_code = event_param;
		sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
					 __stringify(SYS_ATTR_NAME_UCP));

		snprintf(event_string, MAX_EVENT_STRING_LEN,
				"MHLEVENT=received_UCP key code=0x%02x", event_param);
		kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		break;

	case MHL_TX_EVENT_UCPK_RECEIVED:
		if ((dev_context->mhl_flags & MHL_STATE_FLAG_UCP_SENT)
			&& (dev_context->ucp_out_key_code == event_param)) {

			dev_context->mhl_flags |= MHL_STATE_FLAG_UCP_ACK;

			MHL_TX_DBG_INFO(dev_context, "Generating UCPK received event, "
					 "keycode: 0x%02x\n", event_param);

			sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
						 __stringify(SYS_ATTR_NAME_UCPK));

			snprintf(event_string, MAX_EVENT_STRING_LEN,
					"MHLEVENT=received_UCPK key code=0x%02x", event_param);
			kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		} else {
			MHL_TX_DBG_ERR(dev_context, "Ignoring unexpected UCPK received "
					"event, keycode: 0x%02x\n", event_param);
		}
		break;

	case MHL_TX_EVENT_UCPE_RECEIVED:
		if (dev_context->mhl_flags & MHL_STATE_FLAG_UCP_SENT) {

			dev_context->ucp_err_code = event_param;
			dev_context->mhl_flags |= MHL_STATE_FLAG_UCP_NAK;

			MHL_TX_DBG_INFO(dev_context, "Generating UCPE received event, "
					 "error code: 0x%02x\n", event_param);

			sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
						 __stringify(SYS_ATTR_NAME_UCPK));

			snprintf(event_string, MAX_EVENT_STRING_LEN,
					"MHLEVENT=received_UCPE error code=0x%02x", event_param);
			kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		} else {
			MHL_TX_DBG_ERR(dev_context, "Ignoring unexpected UCPE received "
					"event, error code: 0x%02x\n", event_param);
		}
		break;

	case MHL_TX_EVENT_SPAD_RECEIVED:
		length = event_param;
		buf = data;

		sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
					 __stringify(SYS_ATTR_NAME_SPAD));

		idx = snprintf(event_string, MAX_EVENT_STRING_LEN,
					   "MHLEVENT=SPAD_CHG length=0x%02x data=", length);

		count = 0;
		while (idx < MAX_EVENT_STRING_LEN) {
			if (count >= length)
				break;

			idx += snprintf(&event_string[idx], MAX_EVENT_STRING_LEN - idx,
							"0x%02x ", buf[count]);
			count++;
		}

		if (idx < MAX_EVENT_STRING_LEN) {
			kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		} else {
			MHL_TX_DBG_ERR(dev_context, "Buffer too small to contain "
					"scratch pad data!\n");

		}
		break;

	case MHL_TX_EVENT_POW_BIT_CHG:
		MHL_TX_DBG_INFO(dev_context, "Generating VBUS power bit change "\
						"event, POW bit is %s\n", event_param? "ON" : "OFF");
		snprintf(event_string, MAX_EVENT_STRING_LEN,
				"MHLEVENT=MHL VBUS power %s", event_param? "ON" : "OFF");
		kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		break;

	case MHL_TX_EVENT_RAP_RECEIVED:
		MHL_TX_DBG_INFO(dev_context, "Generating RAP received event, "
						"action code: 0x%02x\n", event_param);

		sysfs_notify(&dev_context->mhl_dev->kobj, NULL,
					 __stringify(SYS_ATTR_NAME_RAP_IN));

		snprintf(event_string, MAX_EVENT_STRING_LEN,
				"MHLEVENT=received_RAP action code=0x%02x", event_param);
		kobject_uevent_env(&dev_context->mhl_dev->kobj, KOBJ_CHANGE, envp);
		break;

	default:
		MHL_TX_DBG_ERR(dev_context, "called with unrecognized event code!\n");
	}
}

static const struct file_operations mhl_fops = {
    .owner			= THIS_MODULE
};

struct device_attribute driver_attribs[] = {
		__ATTR(SYS_ATTR_NAME_CONN        , 0444, show_connection_state        , NULL),
#ifndef DRIVER_SPEC_SYSFS 
		__ATTR(SYS_ATTR_NAME_RCP         , 0644, show_rcp                     , send_rcp),
		__ATTR(SYS_ATTR_NAME_RCPK        , 0644, show_rcp_ack                 , send_rcp_ack),
		__ATTR(SYS_ATTR_NAME_RAP_LEGACY  , 0644, show_rap                     , send_rap),
		__ATTR(SYS_ATTR_NAME_RAP_STATUS  , 0644, show_rap_status              , set_rap_status),
		__ATTR(SYS_ATTR_NAME_DEVCAP      , 0644, show_dev_cap                 , select_dev_cap),
		__ATTR(SYS_ATTR_NAME_UCP         , 0644, show_ucp                     , send_ucp),
		__ATTR(SYS_ATTR_NAME_UCPK        , 0644, show_ucp_ack                 , send_ucp_ack),
#endif 
		__ATTR(SYS_ATTR_NAME_SPAD        , 0644, show_scratch_pad             , send_scratch_pad),
		__ATTR(SYS_ATTR_NAME_I2C_REGS    , 0644, show_i2c                     , set_i2c),
		__ATTR(SYS_ATTR_NAME_ACCESSORY   , 0644, get_accessory                , set_accessory),
		__ATTR(SYS_ATTR_NAME_ID_IMPEDANCE, 0644, show_id_impedance_measurement, trigger_id_impedance_measurement),
		__ATTR(SYS_ATTR_NAME_RETENTION   , 0644, get_retention                , set_retention),
		__ATTR(SYS_ATTR_NAME_DEBUG_LEVEL , 0644, get_debug_level              , set_debug_level),
		__ATTR(SYS_ATTR_NAME_GPIO_INDEX  , 0644, get_gpio_index               , set_gpio_index),
		__ATTR(SYS_ATTR_NAME_GPIO_VALUE  , 0644, get_gpio_level               , set_gpio_level),
		__ATTR(SYS_ATTR_NAME_SWING_VALUE , 0600, show_debug_swing_value	      , set_debug_swing_value),
		__ATTR(SYS_ATTR_NAME_HDCP_FUNC   , 0600, show_hdcp_status	      , set_hdcp_status),
		__ATTR_NULL
};

#ifdef DRIVER_SPEC_SYSFS 
ssize_t show_rap_in( struct device *dev
					,struct device_attribute *attr
					, char *buf
					)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(,"could not acquire mutex!!!\n");
		return -ERESTARTSYS;
	}
	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	} else {
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->rap_in_sub_command);
	}
	up(&dev_context->isr_lock);

	return status;
}

ssize_t set_rap_in_status(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
	} else{
		unsigned long param;
		param = simple_strtoul(buf, NULL, 0);
		switch(param){
		case 0x00:
			dev_context->mhl_flags &= ~MHL_STATE_APPLICATION_RAP_BUSY;
			break;
		case 0x03:
			dev_context->mhl_flags |= MHL_STATE_APPLICATION_RAP_BUSY;
			break;
		default:
			MHL_TX_DBG_ERR(dev_context, "Invalid parameter %s received\n", buf);
			status = -EINVAL;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_rap_out(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	} else{
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->rap_out_sub_command);
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t send_rap_out(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	}else{
		unsigned long param;
		param = simple_strtoul(buf, NULL, 0);
		switch(param){
		case MHL_RAP_POLL:
		case MHL_RAP_CONTENT_ON:
		case MHL_RAP_CONTENT_OFF:
			if (!si_mhl_tx_rap_send(dev_context, param)){
				MHL_TX_DBG_ERR(dev_context,"-EPERM\n");
				status = -EPERM;
			}else{
				dev_context->rap_out_sub_command = (u8)param;
			}
		default:
			MHL_TX_DBG_ERR(dev_context, "Invalid parameter %s received\n", buf);
			status = -EINVAL;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_rap_out_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	} else{
		
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->rap_out_sub_command);
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_rap_input_dev(struct device *dev, struct device_attribute *attr, char *buf)
{
	extern bool input_dev_rap;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	} else{
		status = scnprintf(buf,PAGE_SIZE,"%d\n",input_dev_rap);
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t set_rap_input_dev(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	extern bool input_dev_rap;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	}else{
		unsigned char param;
		param = simple_strtoul(buf, NULL, 0);
		switch(param){
		case 0:
		case 1:
			input_dev_rap = param;
			break;
		default:
			MHL_TX_DBG_ERR(dev_context, "Invalid parameter %s received\n", buf);
			status = -EINVAL;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}
static struct	device_attribute rap_in_attr         = __ATTR(SYS_ATTR_NAME_RAP_IN	  , 0444, show_rap_in	     , NULL);
static struct	device_attribute rap_in_status_attr  = __ATTR(SYS_ATTR_NAME_RAP_IN_STATUS , 0200, NULL		     , set_rap_in_status);
static struct	device_attribute rap_out_attr        = __ATTR(SYS_ATTR_NAME_RAP_OUT	  , 0644, show_rap_out	     , send_rap_out);
static struct	device_attribute rap_out_status_attr = __ATTR(SYS_ATTR_NAME_RAP_OUT_STATUS, 0444, show_rap_out_status, NULL);
static struct	device_attribute rap_input_dev       = __ATTR(SYS_ATTR_NAME_RAP_INPUT_DEV , 0644, show_rap_input_dev , set_rap_input_dev);

static struct attribute *rap_attrs[]={
	 &rap_in_attr.attr
	,&rap_in_status_attr.attr
	,&rap_out_attr.attr
	,&rap_out_status_attr.attr
	,&rap_input_dev.attr
	,NULL
};
static struct attribute_group rap_attribute_group ={
	 .name = __stringify(rap)
	,.attrs = rap_attrs
};

ssize_t show_rcp_in( struct device *dev
					,struct device_attribute *attr
					, char *buf
					)
{
	extern bool input_dev_rcp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(,"could not acquire mutex!!!\n");
		return -ERESTARTSYS;
	}
	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	}else if (input_dev_rcp){
		MHL_TX_DBG_INFO(,"\n");
		status = scnprintf(buf,PAGE_SIZE,"rcp_input_dev\n");
	}else{
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->rcp_in_key_code);
	}
	up(&dev_context->isr_lock);

	return status;
}

ssize_t send_rcp_in_status(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	extern bool input_dev_rcp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
	} else{
		unsigned long err_code;
		err_code = simple_strtoul(buf, NULL, 0);
		status = count;
		if (err_code == 0) {
			if (!si_mhl_tx_rcpk_send(dev_context, dev_context->rcp_in_key_code)) {
				status = -ENOMEM;
			}
		} else if (!si_mhl_tx_rcpe_send(dev_context, (u8)err_code)){
				status = -EINVAL;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_rcp_out(struct device *dev, struct device_attribute *attr, char *buf)
{
	extern bool input_dev_rcp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	} else{
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->rcp_out_key_code);
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t send_rcp_out(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	}else{
		unsigned long param;
		param = simple_strtoul(buf, NULL, 0);
		dev_context->mhl_flags &= ~(MHL_STATE_FLAG_RCP_RECEIVED |
									MHL_STATE_FLAG_RCP_ACK |
									MHL_STATE_FLAG_RCP_NAK);
		dev_context->mhl_flags |= MHL_STATE_FLAG_RCP_SENT;
		dev_context->rcp_send_status = 0;
		if (!si_mhl_tx_rcp_send(dev_context, (u8)param)){
			MHL_TX_DBG_ERR(dev_context,"-EPERM\n");
			status = -EPERM;
		}else{
			dev_context->rcp_out_key_code = param;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_rcp_out_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	extern bool input_dev_rcp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	} else{
		
		if (dev_context->mhl_flags & (MHL_STATE_FLAG_RCP_ACK | MHL_STATE_FLAG_RCP_NAK)) {
			status = scnprintf(buf,PAGE_SIZE,"0x%02x\n" ,dev_context->rcp_err_code);
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_rcp_input_dev(struct device *dev, struct device_attribute *attr, char *buf)
{
	extern bool input_dev_rcp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	} else{
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",input_dev_rcp);
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t set_rcp_input_dev(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	extern bool input_dev_rcp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	}else{
		unsigned char param;
		param = simple_strtoul(buf, NULL, 0);
		switch(param){
		case 0:
		case 1:
			input_dev_rcp = param;
			break;
		default:
			MHL_TX_DBG_ERR(dev_context, "Invalid parameter %s received\n", buf);
			status = -EINVAL;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

static struct	device_attribute rcp_in_attr         = __ATTR(SYS_ATTR_NAME_RCP_IN        , 0444, show_rcp_in        , NULL);
static struct	device_attribute rcp_in_status_attr  = __ATTR(SYS_ATTR_NAME_RCP_IN_STATUS , 0200, NULL               , send_rcp_in_status);
static struct	device_attribute rcp_out_attr        = __ATTR(SYS_ATTR_NAME_RCP_OUT       , 0644, show_rcp_out       , send_rcp_out);
static struct	device_attribute rcp_out_status_attr = __ATTR(SYS_ATTR_NAME_RCP_OUT_STATUS, 0444, show_rcp_out_status, NULL);
static struct	device_attribute rcp_input_dev       = __ATTR(SYS_ATTR_NAME_RCP_INPUT_DEV , 0644, show_rcp_input_dev , set_rcp_input_dev);

static struct attribute *rcp_attrs[]={
	 &rcp_in_attr.attr
	,&rcp_in_status_attr.attr
	,&rcp_out_attr.attr
	,&rcp_out_status_attr.attr
	,&rcp_input_dev.attr
	,NULL
};
static struct attribute_group rcp_attribute_group ={
	 .name = __stringify(rcp)
	,.attrs = rcp_attrs
};


ssize_t show_ucp_in( struct device *dev
					,struct device_attribute *attr
					, char *buf
					)
{
	
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(,"could not acquire mutex!!!\n");
		return -ERESTARTSYS;
	}
	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	}else {
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->ucp_in_key_code);
	}
	up(&dev_context->isr_lock);

	return status;
}

ssize_t send_ucp_in_status(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock))
		return -ERESTARTSYS;

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		status = -ENODEV;
	} else{
		unsigned long err_code;
		err_code = simple_strtoul(buf, NULL, 0);
		status = count;
		if (err_code == 0) {
			if (!si_mhl_tx_ucpk_send(dev_context, dev_context->ucp_in_key_code)) {
				status = -ENOMEM;
			}
		} else if (!si_mhl_tx_ucpe_send(dev_context, (u8)err_code)){
				status = -EINVAL;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_ucp_out(struct device *dev, struct device_attribute *attr, char *buf)
{
	
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	} else{
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->ucp_out_key_code);
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t send_ucp_out(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	}else{
		unsigned char param;
		param = simple_strtoul(buf, NULL, 0);
		dev_context->mhl_flags &= ~(MHL_STATE_FLAG_UCP_RECEIVED |
									MHL_STATE_FLAG_UCP_ACK |
									MHL_STATE_FLAG_UCP_NAK);
		dev_context->mhl_flags |= MHL_STATE_FLAG_UCP_SENT;
		if (!si_mhl_tx_ucp_send(dev_context, param)){
			MHL_TX_DBG_ERR(dev_context,"-EPERM\n");
			status = -EPERM;
		}else{
			dev_context->ucp_out_key_code = param;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_ucp_out_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	} else{
		
		if (dev_context->mhl_flags & (MHL_STATE_FLAG_UCP_ACK | MHL_STATE_FLAG_UCP_NAK)) {
			status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->ucp_err_code);
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_ucp_input_dev(struct device *dev, struct device_attribute *attr, char *buf)
{
	extern bool input_dev_ucp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	} else{
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",input_dev_ucp);
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t set_ucp_input_dev(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	extern bool input_dev_ucp;
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	}else{
		unsigned char param;
		param = simple_strtoul(buf, NULL, 0);
		switch(param){
		case 0:
		case 1:
			input_dev_ucp = param;
			break;
		default:
			MHL_TX_DBG_ERR(dev_context, "Invalid parameter %s received\n", buf);
			status = -EINVAL;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

static struct	device_attribute ucp_in_attr         = __ATTR(SYS_ATTR_NAME_UCP_IN	  ,0444, show_ucp_in		, NULL);
static struct	device_attribute ucp_in_status_attr  = __ATTR(SYS_ATTR_NAME_UCP_IN_STATUS ,0200, NULL			, send_ucp_in_status);
static struct	device_attribute ucp_out_attr        = __ATTR(SYS_ATTR_NAME_UCP_OUT	  ,0644, show_ucp_out		, send_ucp_out);
static struct	device_attribute ucp_out_status_attr = __ATTR(SYS_ATTR_NAME_UCP_OUT_STATUS,0444, show_ucp_out_status	, NULL);
static struct	device_attribute ucp_input_dev       = __ATTR(SYS_ATTR_NAME_UCP_INPUT_DEV ,0644, show_ucp_input_dev	, set_ucp_input_dev);

static struct attribute *ucp_attrs[]={
	 &ucp_in_attr.attr
	,&ucp_in_status_attr.attr
	,&ucp_out_attr.attr
	,&ucp_out_status_attr.attr
	,&ucp_input_dev.attr
	,NULL
};
static struct attribute_group ucp_attribute_group ={
	 .name = __stringify(ucp)
	,.attrs = ucp_attrs
};


ssize_t show_devcap_local_offset(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	} else{
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->dev_cap_local_offset);
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t set_devcap_local_offset(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	}else{
		unsigned char param;
		param = simple_strtoul(buf, NULL, 0);
		if (param >= 16){
			MHL_TX_DBG_ERR(dev_context, "Invalid parameter %s received\n", buf);
		}else{
			dev_context->dev_cap_local_offset= param;
			status = -EINVAL;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_devcap_local(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	} else{
		extern uint8_t dev_cap_values[];
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_cap_values[dev_context->dev_cap_local_offset]);
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t set_devcap_local(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	}else{
		extern uint8_t dev_cap_values[];
		unsigned char param;
		param = simple_strtoul(buf, NULL, 0);
		switch(param){
		case 0:
		case 1:
			dev_cap_values[dev_context->dev_cap_local_offset]= param;
			break;
		default:
			MHL_TX_DBG_ERR(dev_context, "Invalid parameter %s received\n", buf);
			status = -EINVAL;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_devcap_remote_offset(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	} else{
		status = scnprintf(buf,PAGE_SIZE,"0x%02x\n",dev_context->dev_cap_remote_offset);
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t set_devcap_remote_offset(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int						status;

	
	status = count;

	MHL_TX_DBG_INFO(dev_context, "received string: %s\n", buf);

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	}else{
		extern uint8_t dev_cap_values[];
		unsigned char param;
		param = simple_strtoul(buf, NULL, 0);
		switch(param){
		case 0:
		case 1:
			dev_cap_values[dev_context->dev_cap_remote_offset]= param;
			break;
		default:
			MHL_TX_DBG_ERR(dev_context, "Invalid parameter %s received\n", buf);
			status = -EINVAL;
		}
	}

	up(&dev_context->isr_lock);

	return status;
}

ssize_t show_devcap_remote(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mhl_dev_context	*dev_context = dev_get_drvdata(dev);
	int	status = -EINVAL;

	if (down_interruptible(&dev_context->isr_lock)){
		MHL_TX_DBG_ERR(dev_context,"-ERESTARTSYS\n");
		return -ERESTARTSYS;
	}

	if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN) {
		MHL_TX_DBG_ERR(dev_context,"-ENODEV\n");
		status = -ENODEV;
	} else{
		uint8_t	regValue;
		status = si_mhl_tx_get_peer_dev_cap_entry(dev_context,
											dev_context->dev_cap_remote_offset,
											&regValue);
		if (status != 0) {
			status = -EAGAIN;
		}else{
			status = scnprintf(buf, PAGE_SIZE, "0x%02x",regValue);
		}
	}

	up(&dev_context->isr_lock);

	return status;
}
static struct	device_attribute attr_devcap_local_offset  = __ATTR(SYS_ATTR_NAME_DEVCAP_LOCAL_OFFSET	,0644, show_devcap_local_offset	,set_devcap_local_offset);
static struct	device_attribute attr_devcap_local         = __ATTR(SYS_ATTR_NAME_DEVCAP_LOCAL		,0644, show_devcap_local		,set_devcap_local);
static struct	device_attribute attr_devcap_remote_offset = __ATTR(SYS_ATTR_NAME_DEVCAP_REMOTE_OFFSET	,0644, show_devcap_remote_offset	,set_devcap_remote_offset);
static struct	device_attribute attr_devcap_remote        = __ATTR(SYS_ATTR_NAME_DEVCAP_REMOTE		,0444, show_devcap_remote	,NULL	);

static struct attribute *devcap_attrs[]={
	 &attr_devcap_local_offset.attr
	,&attr_devcap_local.attr
	,&attr_devcap_remote_offset.attr
	,&attr_devcap_remote.attr
	,NULL
};
static struct attribute_group devcap_attribute_group ={
	 .name = __stringify(devcap)
	,.attrs = devcap_attrs
};


#endif 

static irqreturn_t mhl_irq_handler(int irq, void *data)
{
	struct mhl_dev_context	*dev_context = (struct mhl_dev_context *)data;

	MHL_TX_DBG_INFO(dev_context, "called\n");

	if (!down_interruptible(&dev_context->isr_lock)) {
		if (dev_context->fake_cable_out)
			cancel_delayed_work(&dev_context->irq_timeout_work);

		if (dev_context->dev_flags & DEV_FLAG_SHUTDOWN)
			goto irq_done;
		if (dev_context->dev_flags & DEV_FLAG_COMM_MODE)
			goto irq_done;

		memset(&dev_context->intr_info, 0, sizeof(*(&dev_context->intr_info)));

		dev_context->intr_info.edid_parser_context =
							dev_context->edid_parser_context;

		dev_context->drv_info->mhl_device_isr((struct drv_hw_context *)
							(&dev_context->drv_context),
							&dev_context->intr_info);

		
		if(dev_context->intr_info.flags & DRV_INTR_FLAG_DISCONNECT) {
			dev_context->misc_flags.flags.rap_content_on = false;
			dev_context->misc_flags.flags.mhl_rsen = false;
			dev_context->mhl_connection_event = true;
			dev_context->mhl_connected = MHL_TX_EVENT_DISCONNECTION;
			si_mhl_tx_process_events(dev_context);
		} else {
			if (dev_context->intr_info.flags & DRV_INTR_FLAG_CONNECT) {
				dev_context->misc_flags.flags.rap_content_on = true;
				dev_context->rap_in_sub_command = MHL_RAP_CONTENT_ON;
				dev_context->misc_flags.flags.mhl_rsen = true;
				dev_context->mhl_connection_event = true;
				dev_context->mhl_connected = MHL_TX_EVENT_CONNECTION;
				si_mhl_tx_process_events(dev_context);
			}

			if (dev_context->intr_info.flags & DRV_INTR_FLAG_CBUS_ABORT)
				process_cbus_abort(dev_context);

			if (dev_context->intr_info.flags & DRV_INTR_FLAG_WRITE_BURST)
				si_mhl_tx_process_write_burst_data(dev_context);

			if (dev_context->intr_info.flags & DRV_INTR_FLAG_SET_INT)
				si_mhl_tx_got_mhl_intr(dev_context,
						dev_context->intr_info.int_msg[0],
						dev_context->intr_info.int_msg[1]);

			if (dev_context->intr_info.flags & DRV_INTR_FLAG_MSC_DONE)
#ifdef CONFIG_MHL_MONITOR_WORKAROUND
				si_mhl_tx_msc_command_done(
						(struct drv_hw_context *)(&dev_context->drv_context),
						dev_context,
						dev_context->intr_info.msc_done_data);
#else
				si_mhl_tx_msc_command_done(dev_context,
						dev_context->intr_info.msc_done_data);
#endif

			if (dev_context->intr_info.flags & DRV_INTR_FLAG_HPD_CHANGE)
				si_mhl_tx_notify_downstream_hpd_change(dev_context,
						dev_context->intr_info.hpd_status);

			if (dev_context->intr_info.flags & DRV_INTR_FLAG_WRITE_STAT)
				si_mhl_tx_got_mhl_status(dev_context,
						dev_context->intr_info.write_stat[0],
						dev_context->intr_info.write_stat[1]);

			if (dev_context->intr_info.flags & DRV_INTR_FLAG_MSC_RECVD) {
				dev_context->msc_msg_arrived		= true;
				dev_context->msc_msg_sub_command	=
					dev_context->intr_info.msc_msg[0];
				dev_context->msc_msg_data		=
					dev_context->intr_info.msc_msg[1];
				si_mhl_tx_process_events(dev_context);
			}
		}
		si_mhl_tx_drive_states(dev_context);
irq_done:
		up(&dev_context->isr_lock);
	}

	return IRQ_HANDLED;
}

static ssize_t mhl_chipid_show(struct class *class,
				struct class_attribute *attr,
				char *buffer)
{
	return scnprintf(buffer, PAGE_SIZE, "%x\n", get_device_id());
}

static struct class_attribute mhl_attribute[] = {
	__ATTR(chipid, S_IRUGO, mhl_chipid_show, NULL),
	__ATTR_NULL,
};


int mhl_tx_init(struct mhl_drv_info const *drv_info,
				struct i2c_client *client)
{
	struct mhl_dev_context *dev_context;
	int		ret,dummy;


	if (drv_info == NULL || client == NULL) {
		pr_err("Null parameter passed to %s\n",__FUNCTION__);
		return -EINVAL;
	}

	if (drv_info->mhl_device_isr == NULL || client->irq == 0) {
		dev_err(&client->dev, "No IRQ specified!\n");
		return -EINVAL;
	}

	dev_context = kzalloc(sizeof(*dev_context) + drv_info->drv_context_size,
								 GFP_KERNEL);
	if (!dev_context) {
		dev_err(&client->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	dev_context->signature = MHL_DEV_CONTEXT_SIGNATURE;
	dev_context->drv_info = drv_info;
	dev_context->client = client;

	sema_init(&dev_context->isr_lock, 1);
	INIT_LIST_HEAD(&dev_context->timer_list);
	dev_context->timer_work_queue = create_workqueue(MHL_DRIVER_NAME);
	if (dev_context->timer_work_queue == NULL) {
		ret = -ENOMEM;
		goto free_mem;
	}

	if (mhl_class == NULL) {
		mhl_class = class_create(THIS_MODULE, "mhl");
		if(IS_ERR(mhl_class)) {
			ret = PTR_ERR(mhl_class);
			pr_info("class_create failed %d\n", ret);
			goto err_exit;
		}

		mhl_class->dev_attrs = driver_attribs;
		ret = class_create_file(mhl_class, mhl_attribute);
		if (ret) {
			pr_info("class_create_file mhl_class failed %d\n",ret);
		}

		ret = alloc_chrdev_region(&dev_num, 0, MHL_DRIVER_MINOR_MAX, MHL_DRIVER_NAME);
		if (ret) {
			pr_info("register_chrdev %s failed, error code: %d\n",
					MHL_DRIVER_NAME, ret);
			goto free_class;
		}

		cdev_init(&dev_context->mhl_cdev, &mhl_fops);
		dev_context->mhl_cdev.owner = THIS_MODULE;
		ret = cdev_add(&dev_context->mhl_cdev, MINOR(dev_num), MHL_DRIVER_MINOR_MAX);
		if (ret) {
			pr_info("cdev_add %s failed %d\n", MHL_DRIVER_NAME, ret);
			goto free_chrdev;
		}
	}

	dev_context->mhl_dev = device_create(mhl_class, &dev_context->client->dev,
					     dev_num, dev_context, "%s", MHL_DEVICE_NAME);
	if (IS_ERR(dev_context->mhl_dev)) {
		ret = PTR_ERR(dev_context->mhl_dev);
		pr_info("device_create failed %s %d\n", MHL_DEVICE_NAME, ret);
		goto free_cdev;
	}
#ifdef DRIVER_SPEC_SYSFS 
	{
		int status = 0;
		status = sysfs_create_group(&dev_context->mhl_dev->kobj,&rap_attribute_group);
		if (status) {
			MHL_TX_DBG_ERR(,"sysfs_create_group failed:%d\n",status);
		}
		status = sysfs_create_group(&dev_context->mhl_dev->kobj,&rcp_attribute_group);
		if (status) {
			MHL_TX_DBG_ERR(,"sysfs_create_group failed:%d\n",status);
		}
		status = sysfs_create_group(&dev_context->mhl_dev->kobj,&ucp_attribute_group);
		if (status) {
			MHL_TX_DBG_ERR(,"sysfs_create_group failed:%d\n",status);
		}
		status = sysfs_create_group(&dev_context->mhl_dev->kobj,&devcap_attribute_group);
		if (status) {
			MHL_TX_DBG_ERR(,"sysfs_create_group failed:%d\n",status);
		}
		dev_context->debug_swing_value = -1;
		dev_context->hdcp_status = 1;
	}
#endif 


	ret = request_threaded_irq(dev_context->client->irq, NULL,
					mhl_irq_handler, IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					MHL_DEVICE_NAME, dev_context);
	if (ret < 0) {
		dev_err(&client->dev, "request_threaded_irq failed, status: %d\n", ret);
		goto free_device;
	}

    
	ret = down_interruptible(&dev_context->isr_lock);
	if (ret) {
		dev_err(&client->dev, "failed to acquire ISR semaphore, status: %d\n", ret);
		goto free_irq_handler;
	}

	i2c_set_clientdata(client, dev_context);

	
	dev_context->edid_parser_context = si_edid_create_context(dev_context,&dev_context->drv_context);

	
	rcp_input_dev_one_time_init(dev_context);

	ret = si_mhl_tx_initialize(dev_context);
	up(&dev_context->isr_lock);

	if (ret)
		goto free_irq_handler;

	MHL_TX_DBG_INFO(dev_context, "MHL transmitter successfully initialized\n");


	return ret;

free_irq_handler:
	i2c_set_clientdata(client, NULL);
	dummy = down_interruptible(&dev_context->isr_lock);
	if(dev_context->edid_parser_context)
		si_edid_destroy_context(dev_context->edid_parser_context);

	free_irq(dev_context->client->irq, dev_context);

free_device:
	device_destroy(mhl_class, dev_num);

free_cdev:
	cdev_del(&dev_context->mhl_cdev);

free_chrdev:
	unregister_chrdev_region(dev_num, MHL_DRIVER_MINOR_MAX);
	dev_num = 0;

free_class:
	class_destroy(mhl_class);

err_exit:
	destroy_workqueue(dev_context->timer_work_queue);

free_mem:
	kfree(dev_context);

	return ret;
}

int mhl_tx_remove(struct i2c_client *client)
{
	struct mhl_dev_context *dev_context;
	int		ret=0;

	dev_context = i2c_get_clientdata(client);

	if (dev_context != NULL){
	extern bool input_dev_rcp;
		MHL_TX_DBG_INFO(dev_context, "%x\n",dev_context);
		ret = down_interruptible(&dev_context->isr_lock);

		dev_context->dev_flags |= DEV_FLAG_SHUTDOWN;

		ret = si_mhl_tx_shutdown(dev_context);

		mhl_tx_destroy_timer_support(dev_context);

		up(&dev_context->isr_lock);

		free_irq(dev_context->client->irq, dev_context);

		sysfs_remove_group(&dev_context->mhl_dev->kobj,&rap_attribute_group);
		sysfs_remove_group(&dev_context->mhl_dev->kobj,&rcp_attribute_group);
		sysfs_remove_group(&dev_context->mhl_dev->kobj,&ucp_attribute_group);
		sysfs_remove_group(&dev_context->mhl_dev->kobj,&devcap_attribute_group);

		device_destroy(mhl_class, dev_num);

		cdev_del(&dev_context->mhl_cdev);

		unregister_chrdev_region(dev_num, MHL_DRIVER_MINOR_MAX);
		dev_num = 0;

		class_destroy(mhl_class);
		mhl_class = NULL;

	#ifdef MEDIA_DATA_TUNNEL_SUPPORT
		mdt_destroy(dev_context);
	#endif
		if (input_dev_rcp){
			destroy_rcp_input_dev(dev_context);
		}

		si_edid_destroy_context(dev_context->edid_parser_context);

		

		kfree(dev_context);
	}
	return ret;
}

static void mhl_tx_destroy_timer_support(struct  mhl_dev_context *dev_context)
{
	struct timer_obj	*mhl_timer;

	while(!list_empty(&dev_context->timer_list)) {
		mhl_timer = list_first_entry(&dev_context->timer_list,
									 struct timer_obj, list_link);
		hrtimer_cancel(&mhl_timer->hr_timer);
		list_del(&mhl_timer->list_link);
		kfree(mhl_timer);
	}

	destroy_workqueue(dev_context->timer_work_queue);
	dev_context->timer_work_queue = NULL;
}

static void mhl_tx_timer_work_handler(struct work_struct *work)
{
	struct timer_obj	*mhl_timer;

	mhl_timer = container_of(work, struct timer_obj, work_item);

	mhl_timer->flags |= TIMER_OBJ_FLAG_WORK_IP;
	if (!down_interruptible(&mhl_timer->dev_context->isr_lock)) {

		mhl_timer->timer_callback_handler(mhl_timer->callback_param);

		up(&mhl_timer->dev_context->isr_lock);
	}
	mhl_timer->flags &= ~TIMER_OBJ_FLAG_WORK_IP;

	if(mhl_timer->flags & TIMER_OBJ_FLAG_DEL_REQ) {
		kfree(mhl_timer);
	}
}

static enum hrtimer_restart mhl_tx_timer_handler(struct hrtimer *timer)
{
	struct timer_obj	*mhl_timer;

	mhl_timer = container_of(timer, struct timer_obj, hr_timer);

	queue_work(mhl_timer->dev_context->timer_work_queue,
			   &mhl_timer->work_item);

	return HRTIMER_NORESTART;
}

static int is_timer_handle_valid(struct mhl_dev_context *dev_context,
								 void *timer_handle)
{
	struct timer_obj	*timer;

	list_for_each_entry(timer, &dev_context->timer_list, list_link) {
		if (timer == timer_handle)
			break;
	}

	if(timer != timer_handle) {
		MHL_TX_DBG_ERR(dev_context, "Invalid timer handle %p received\n",
						timer_handle);
		return -EINVAL;
	}
	return 0;
}

int mhl_tx_create_timer(void *context,
						void (*callback_handler)(void *callback_param),
						void *callback_param,
						void **timer_handle)
{
	struct mhl_dev_context	*dev_context;
	struct timer_obj		*new_timer;

	dev_context = get_mhl_device_context(context);

	if (callback_handler == NULL)
		return -EINVAL;

	if (dev_context->timer_work_queue == NULL)
		return -ENOMEM;

	new_timer = kmalloc(sizeof(*new_timer), GFP_KERNEL);
	if (new_timer == NULL)
		return -ENOMEM;

	new_timer->timer_callback_handler = callback_handler;
	new_timer->callback_param = callback_param;
	new_timer->flags = 0;

	new_timer->dev_context = dev_context;
	INIT_WORK(&new_timer->work_item, mhl_tx_timer_work_handler);

	list_add(&new_timer->list_link, &dev_context->timer_list);

	hrtimer_init(&new_timer->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	new_timer->hr_timer.function = mhl_tx_timer_handler;
	*timer_handle = new_timer;
	return 0;
}

int mhl_tx_delete_timer(void *context, void **timer_handle)
{
	struct mhl_dev_context	*dev_context;
	struct timer_obj		*timer;
	int						status;

	dev_context = get_mhl_device_context(context);

	status = is_timer_handle_valid(dev_context, *timer_handle);
	if (status == 0) {
		timer = *timer_handle;

		list_del(&timer->list_link);

		hrtimer_cancel(&timer->hr_timer);

		if(timer->flags & TIMER_OBJ_FLAG_WORK_IP) {
			timer->flags |= TIMER_OBJ_FLAG_DEL_REQ;
		} else {
			cancel_work_sync(&timer->work_item);
			*timer_handle = NULL;
			kfree(timer);
		}
	}

	return status;
}

int mhl_tx_start_timer(void *context, void *timer_handle,
					   uint32_t time_msec)
{
	struct mhl_dev_context	*dev_context;
	struct timer_obj		*timer;
	ktime_t					timer_period;
	int						status;

	dev_context = get_mhl_device_context(context);

	status = is_timer_handle_valid(dev_context, timer_handle);
	if (status == 0) {
		long secs=0;
		timer = timer_handle;

		secs=time_msec/1000;
		time_msec %= 1000;
		timer_period = ktime_set(secs, MSEC_TO_NSEC(time_msec));
		hrtimer_start(&timer->hr_timer, timer_period, HRTIMER_MODE_REL);
	}

	return status;
}

int mhl_tx_stop_timer(void *context, void *timer_handle)
{
	struct mhl_dev_context	*dev_context;
	struct timer_obj		*timer;
	int						status;

	dev_context = get_mhl_device_context(context);

	status = is_timer_handle_valid(dev_context, timer_handle);
	if (status == 0) {
		timer = timer_handle;

		hrtimer_cancel(&timer->hr_timer);
	}
	return status;
}
