/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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

#include <mach/camera2.h>
#include "msm_camera_i2c.h"
#include "msm_cci.h"

#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#define S_I2C_DBG(fmt, args...) pr_debug(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#define S_I2C_DBG(fmt, args...) do { } while (0)
#endif

#define I2C_COMPARE_MATCH 0
#define I2C_COMPARE_MISMATCH 1
#define I2C_POLL_MAX_ITERATION 20

int32_t msm_camera_cci_i2c_read(struct msm_camera_i2c_client *client,
	uint32_t addr, uint16_t *data,
	enum msm_camera_i2c_data_type data_type)
{
	int32_t rc = -EFAULT;
	unsigned char buf[client->addr_type+data_type];
	struct msm_camera_cci_ctrl cci_ctrl;

	if ((client->addr_type != MSM_CAMERA_I2C_BYTE_ADDR
		&& client->addr_type != MSM_CAMERA_I2C_WORD_ADDR)
		|| (data_type != MSM_CAMERA_I2C_BYTE_DATA
		&& data_type != MSM_CAMERA_I2C_WORD_DATA))
		return rc;

	cci_ctrl.cmd = MSM_CCI_I2C_READ;
	cci_ctrl.cci_info = client->cci_client;
	cci_ctrl.cfg.cci_i2c_read_cfg.addr = addr;
	cci_ctrl.cfg.cci_i2c_read_cfg.addr_type = client->addr_type;
	cci_ctrl.cfg.cci_i2c_read_cfg.data = buf;
	cci_ctrl.cfg.cci_i2c_read_cfg.num_byte = data_type;
	rc = v4l2_subdev_call(client->cci_client->cci_subdev,
			core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	if (rc < 0) {
		pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
		return rc;
	}
	rc = cci_ctrl.status;
	if (data_type == MSM_CAMERA_I2C_BYTE_DATA)
		*data = buf[0];
	else
		*data = buf[0] << 8 | buf[1];

	S_I2C_DBG("%s addr = 0x%x data: 0x%x\n", __func__, addr, *data);
	return rc;
}

int32_t msm_camera_cci_i2c_read_seq(struct msm_camera_i2c_client *client,
	uint32_t addr, uint8_t *data, uint32_t num_byte)
{
	int32_t rc = -EFAULT;
	unsigned char *buf = NULL;
	int i;
	struct msm_camera_cci_ctrl cci_ctrl;
	
	cci_ctrl.status = 0;
	

	if ((client->addr_type != MSM_CAMERA_I2C_BYTE_ADDR
		&& client->addr_type != MSM_CAMERA_I2C_WORD_ADDR)
		|| num_byte == 0)
		return rc;

	buf = kzalloc(num_byte, GFP_KERNEL);
	if (!buf) {
		pr_err("%s:%d no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}
	cci_ctrl.cmd = MSM_CCI_I2C_READ;
	cci_ctrl.cci_info = client->cci_client;
	cci_ctrl.cfg.cci_i2c_read_cfg.addr = addr;
	cci_ctrl.cfg.cci_i2c_read_cfg.addr_type = client->addr_type;
	cci_ctrl.cfg.cci_i2c_read_cfg.data = buf;
	cci_ctrl.cfg.cci_i2c_read_cfg.num_byte = num_byte;
	rc = v4l2_subdev_call(client->cci_client->cci_subdev,
			core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	CDBG("%s line %d rc = %d\n", __func__, __LINE__, rc);
	rc = cci_ctrl.status;

	S_I2C_DBG("%s addr = 0x%x", __func__, addr);
	for (i = 0; i < num_byte; i++) {
		data[i] = buf[i];
		S_I2C_DBG("Byte %d: 0x%x\n", i, buf[i]);
		S_I2C_DBG("Data: 0x%x\n", data[i]);
	}
	kfree(buf);
	return rc;
}

int32_t msm_camera_cci_i2c_write(struct msm_camera_i2c_client *client,
	uint32_t addr, uint16_t data,
	enum msm_camera_i2c_data_type data_type)
{
	int32_t rc = -EFAULT;
	struct msm_camera_cci_ctrl cci_ctrl;
	struct msm_camera_i2c_reg_array reg_conf_tbl;

	if ((client->addr_type != MSM_CAMERA_I2C_BYTE_ADDR
		&& client->addr_type != MSM_CAMERA_I2C_WORD_ADDR)
		|| (data_type != MSM_CAMERA_I2C_BYTE_DATA
		&& data_type != MSM_CAMERA_I2C_WORD_DATA))
		return rc;

	CDBG("%s:%d reg addr = 0x%x data type: %d\n",
		__func__, __LINE__, addr, data_type);
	reg_conf_tbl.reg_addr = addr;
	reg_conf_tbl.reg_data = data;
	cci_ctrl.cmd = MSM_CCI_I2C_WRITE;
	cci_ctrl.cci_info = client->cci_client;
	cci_ctrl.cfg.cci_i2c_write_cfg.reg_setting = &reg_conf_tbl;
	cci_ctrl.cfg.cci_i2c_write_cfg.data_type = data_type;
	cci_ctrl.cfg.cci_i2c_write_cfg.addr_type = client->addr_type;
	cci_ctrl.cfg.cci_i2c_write_cfg.size = 1;
	rc = v4l2_subdev_call(client->cci_client->cci_subdev,
			core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	if (rc < 0) {
		pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
		return rc;
	}
	rc = cci_ctrl.status;
	return rc;
}

int32_t msm_camera_cci_i2c_write_seq(struct msm_camera_i2c_client *client,
	uint32_t addr, uint8_t *data, uint32_t num_byte)
{
	int32_t rc = -EFAULT;
	uint8_t i = 0;
	struct msm_camera_cci_ctrl cci_ctrl;
	struct msm_camera_i2c_reg_array reg_conf_tbl[num_byte];
	
	cci_ctrl.status = 0;
	

	if ((client->addr_type != MSM_CAMERA_I2C_BYTE_ADDR
		&& client->addr_type != MSM_CAMERA_I2C_WORD_ADDR)
		|| num_byte == 0)
		return rc;

	S_I2C_DBG("%s reg addr = 0x%x num bytes: %d\n",
			  __func__, addr, num_byte);
	memset(reg_conf_tbl, 0,
		num_byte * sizeof(struct msm_camera_i2c_reg_array));
	reg_conf_tbl[0].reg_addr = addr;
	for (i = 0; i < num_byte; i++) {
		reg_conf_tbl[i].reg_data = data[i];
		reg_conf_tbl[i].delay = 0;
	}
	cci_ctrl.cmd = MSM_CCI_I2C_WRITE_SEQ;
	cci_ctrl.cci_info = client->cci_client;
	cci_ctrl.cfg.cci_i2c_write_cfg.reg_setting = reg_conf_tbl;
	cci_ctrl.cfg.cci_i2c_write_cfg.data_type = MSM_CAMERA_I2C_BYTE_DATA;
	cci_ctrl.cfg.cci_i2c_write_cfg.addr_type = client->addr_type;
	cci_ctrl.cfg.cci_i2c_write_cfg.size = num_byte;
	rc = v4l2_subdev_call(client->cci_client->cci_subdev,
			core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	CDBG("%s line %d rc = %d\n", __func__, __LINE__, rc);
	rc = cci_ctrl.status;
	return rc;
}

static int32_t msm_camera_cci_i2c_set_mask(struct msm_camera_i2c_client *client,
	uint32_t addr, uint16_t mask,
	enum msm_camera_i2c_data_type data_type, uint16_t set_mask);
int32_t msm_camera_cci_i2c_write_table(
	struct msm_camera_i2c_client *client,
	struct msm_camera_i2c_reg_setting *write_setting)
{
	int32_t rc = -EFAULT;
	struct msm_camera_cci_ctrl cci_ctrl;
    

	int i;
	struct msm_camera_i2c_reg_array *reg_setting;
	uint16_t client_addr_type;
	
	
	int special_delay = 0;
	
	if (!client || !write_setting)
		return rc;

	
	reg_setting = write_setting->reg_setting;
	for (i = 0; i < write_setting->size; i++)
	{
		if (reg_setting->reg_addr == 0xffff)
		{
		    special_delay = 1;
		    break;
		}
		reg_setting++;
	}
	
	if ((write_setting->addr_type != MSM_CAMERA_I2C_BYTE_ADDR
		&& write_setting->addr_type != MSM_CAMERA_I2C_WORD_ADDR)
		|| (write_setting->data_type != MSM_CAMERA_I2C_BYTE_DATA
		&& write_setting->data_type != MSM_CAMERA_I2C_WORD_DATA
		&& write_setting->data_type != MSM_CAMERA_I2C_SET_BYTE_MASK
		&& write_setting->data_type != MSM_CAMERA_I2C_UNSET_BYTE_MASK))
		return rc;

	if (write_setting->size == 0)
		rc = 0;

	reg_setting = write_setting->reg_setting;
	client_addr_type = client->addr_type;
	client->addr_type = write_setting->addr_type;

	if (write_setting->data_type == MSM_CAMERA_I2C_SET_BYTE_MASK) {
		for (i = 0; i < write_setting->size; i++) {
			rc = msm_camera_cci_i2c_set_mask(client, reg_setting->reg_addr,
				reg_setting->reg_data, MSM_CAMERA_I2C_BYTE_DATA, 1);
			if (rc < 0) {
				pr_err("%s:%d failed, i = %d, rc = %d\n", __func__, __LINE__, i, rc);
				return rc;
			}
			reg_setting++;
		}
	} else if (write_setting->data_type == MSM_CAMERA_I2C_UNSET_BYTE_MASK) {
		for (i = 0; i < write_setting->size; i++) {
			rc = msm_camera_cci_i2c_set_mask(client, reg_setting->reg_addr,
				reg_setting->reg_data, MSM_CAMERA_I2C_BYTE_DATA, 0);
			if (rc < 0) {
				pr_err("%s:%d failed, i = %d, rc = %d\n", __func__, __LINE__, i, rc);
				return rc;
			}
			reg_setting++;
		}
	} else {
		
		if(special_delay == 0)
		{
		
		cci_ctrl.cmd = MSM_CCI_I2C_WRITE;
		cci_ctrl.cci_info = client->cci_client;
		cci_ctrl.cfg.cci_i2c_write_cfg.reg_setting =
			write_setting->reg_setting;
		cci_ctrl.cfg.cci_i2c_write_cfg.data_type = write_setting->data_type;
		cci_ctrl.cfg.cci_i2c_write_cfg.addr_type = client->addr_type;
		cci_ctrl.cfg.cci_i2c_write_cfg.size = write_setting->size;
		rc = v4l2_subdev_call(client->cci_client->cci_subdev,
				core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
		if (rc < 0) {
			pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
			return rc;
		}
		rc = cci_ctrl.status;
		
		}
		else
		{
			for (i = 0; i < write_setting->size; i++)
			{
			    if (reg_setting->reg_addr == 0xffff)
			    {
			        int delay = reg_setting->reg_data;
			        msleep(delay);
			        pr_info("%s: delay %d ms\n", __func__,delay);
			    }
			    else
			    {
				
			        rc = msm_camera_cci_i2c_write(client, reg_setting->reg_addr,
			        reg_setting->reg_data, write_setting->data_type);
			        if (rc < 0)
			        {
			            pr_err("%s: write addr(0x%x) data:0x%x fail\n", __func__,reg_setting->reg_addr, reg_setting->reg_data);
			            return rc;
			        }
			    }
			    reg_setting++;
			}
		}
		
	}
	if (write_setting->delay > 20)
		msleep(write_setting->delay);
	else if (write_setting->delay)
		usleep_range(write_setting->delay * 1000, (write_setting->delay
			* 1000) + 1000);

	return rc;
}

int32_t msm_camera_cci_i2c_write_seq_table(
	struct msm_camera_i2c_client *client,
	struct msm_camera_i2c_seq_reg_setting *write_setting)
{
	int i;
	int32_t rc = -EFAULT;
	struct msm_camera_i2c_seq_reg_array *reg_setting;
	uint16_t client_addr_type;

	if (!client || !write_setting)
		return rc;

	if ((write_setting->addr_type != MSM_CAMERA_I2C_BYTE_ADDR
		&& write_setting->addr_type != MSM_CAMERA_I2C_WORD_ADDR)) {
		pr_err("%s Invalide addr type %d\n", __func__,
			write_setting->addr_type);
		return rc;
	}

	reg_setting = write_setting->reg_setting;
	client_addr_type = client->addr_type;
	client->addr_type = write_setting->addr_type;

	for (i = 0; i < write_setting->size; i++) {
		rc = msm_camera_cci_i2c_write_seq(client, reg_setting->reg_addr,
			reg_setting->reg_data, reg_setting->reg_data_size);
		if (rc < 0)
			return rc;
		reg_setting++;
	}
	if (write_setting->delay > 20)
		msleep(write_setting->delay);
	else if (write_setting->delay)
		usleep_range(write_setting->delay * 1000, (write_setting->delay
			* 1000) + 1000);

	client->addr_type = client_addr_type;
	return rc;
}

int32_t msm_camera_cci_i2c_write_table_w_microdelay(
	struct msm_camera_i2c_client *client,
	struct msm_camera_i2c_reg_setting *write_setting)
{
	int32_t rc = -EFAULT;
	struct msm_camera_cci_ctrl cci_ctrl;

	if (!client || !write_setting)
		return rc;

	if ((client->addr_type != MSM_CAMERA_I2C_BYTE_ADDR
		&& client->addr_type != MSM_CAMERA_I2C_WORD_ADDR)
		|| (write_setting->data_type != MSM_CAMERA_I2C_BYTE_DATA
		&& write_setting->data_type != MSM_CAMERA_I2C_WORD_DATA))
		return rc;

	cci_ctrl.cmd = MSM_CCI_I2C_WRITE;
	cci_ctrl.cci_info = client->cci_client;
	cci_ctrl.cfg.cci_i2c_write_cfg.reg_setting =
		write_setting->reg_setting;
	cci_ctrl.cfg.cci_i2c_write_cfg.data_type = write_setting->data_type;
	cci_ctrl.cfg.cci_i2c_write_cfg.addr_type = client->addr_type;
	cci_ctrl.cfg.cci_i2c_write_cfg.size = write_setting->size;
	rc = v4l2_subdev_call(client->cci_client->cci_subdev,
			core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	if (rc < 0) {
		pr_err("%s: line %d rc = %d\n", __func__, __LINE__, rc);
		return rc;
	}
	rc = cci_ctrl.status;
	return rc;
}

static int32_t msm_camera_cci_i2c_compare(struct msm_camera_i2c_client *client,
	uint32_t addr, uint16_t data,
	enum msm_camera_i2c_data_type data_type)
{
	int32_t rc;
	uint16_t reg_data = 0;
	int data_len = 0;
	switch (data_type) {
	case MSM_CAMERA_I2C_BYTE_DATA:
	case MSM_CAMERA_I2C_WORD_DATA:
		data_len = data_type;
		break;
	case MSM_CAMERA_I2C_SET_BYTE_MASK:
	case MSM_CAMERA_I2C_UNSET_BYTE_MASK:
		data_len = MSM_CAMERA_I2C_BYTE_DATA;
		break;
	case MSM_CAMERA_I2C_SET_WORD_MASK:
	case MSM_CAMERA_I2C_UNSET_WORD_MASK:
		data_len = MSM_CAMERA_I2C_WORD_DATA;
		break;
	default:
		pr_err("%s: Unsupport data type: %d\n", __func__, data_type);
		break;
	}

	rc = msm_camera_cci_i2c_read(client, addr, &reg_data, data_len);
	if (rc < 0)
		return rc;

	rc = I2C_COMPARE_MISMATCH;
	switch (data_type) {
	case MSM_CAMERA_I2C_BYTE_DATA:
	case MSM_CAMERA_I2C_WORD_DATA:
		if (data == reg_data)
			rc = I2C_COMPARE_MATCH;
		break;
	case MSM_CAMERA_I2C_SET_BYTE_MASK:
	case MSM_CAMERA_I2C_SET_WORD_MASK:
		if ((reg_data & data) == data)
			rc = I2C_COMPARE_MATCH;
		break;
	case MSM_CAMERA_I2C_UNSET_BYTE_MASK:
	case MSM_CAMERA_I2C_UNSET_WORD_MASK:
		if (!(reg_data & data))
			rc = I2C_COMPARE_MATCH;
		break;
	default:
		pr_err("%s: Unsupport data type: %d\n", __func__, data_type);
		break;
	}

	S_I2C_DBG("%s: Register and data match result %d\n", __func__,
		rc);
	return rc;
}

int32_t msm_camera_cci_i2c_poll(struct msm_camera_i2c_client *client,
	uint32_t addr, uint16_t data,
	enum msm_camera_i2c_data_type data_type)
{
	int32_t rc = -EFAULT; 
	int i;
	S_I2C_DBG("%s: addr: 0x%x data: 0x%x dt: %d\n",
		__func__, addr, data, data_type);

	for (i = 0; i < I2C_POLL_MAX_ITERATION; i++) {
		rc = msm_camera_cci_i2c_compare(client,
			addr, data, data_type);
		if (rc == 0 || rc < 0)
			break;
		usleep_range(10000, 11000);
	}
	return rc;
}

int32_t msm_camera_cci_i2c_poll_table(
	struct msm_camera_i2c_client *client,
	struct msm_camera_i2c_reg_setting *poll_setting)
{
	int i;
	int32_t rc = -EFAULT;
	struct msm_camera_i2c_reg_array *reg_setting;
	uint16_t client_addr_type;

	if (!client || !poll_setting) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return rc;
	}

	reg_setting = poll_setting->reg_setting;
	client_addr_type = client->addr_type;
	client->addr_type = poll_setting->addr_type;

	for (i = 0; i < poll_setting->size; i++) {
		rc = msm_camera_cci_i2c_poll(client, reg_setting->reg_addr,
			reg_setting->reg_data, poll_setting->data_type);
		if (rc < 0) {
			pr_err("%s:%d failed, i = %d, rc = %d\n", __func__, __LINE__, i, rc);
			return rc;
		}
		reg_setting++;
	}

	client->addr_type = client_addr_type;
	return rc;
}

static int32_t msm_camera_cci_i2c_set_mask(struct msm_camera_i2c_client *client,
	uint32_t addr, uint16_t mask,
	enum msm_camera_i2c_data_type data_type, uint16_t set_mask)
{
	int32_t rc;
	uint16_t reg_data;

	rc = msm_camera_cci_i2c_read(client, addr, &reg_data, data_type);
	if (rc < 0) {
		S_I2C_DBG("%s read fail\n", __func__);
		return rc;
	}
	S_I2C_DBG("%s addr: 0x%x data: 0x%x setmask: 0x%x\n",
			__func__, addr, reg_data, mask);

	if (set_mask)
		reg_data |= mask;
	else
		reg_data &= ~mask;
	S_I2C_DBG("%s write: 0x%x\n", __func__, reg_data);

	rc = msm_camera_cci_i2c_write(client, addr, reg_data, data_type);
	if (rc < 0)
		S_I2C_DBG("%s write fail\n", __func__);

	return rc;
}

static int32_t msm_camera_cci_i2c_set_write_mask_data(
	struct msm_camera_i2c_client *client,
	uint32_t addr, uint16_t data, int16_t mask,
	enum msm_camera_i2c_data_type data_type)
{
	int32_t rc;
	uint16_t reg_data;
	CDBG("%s\n", __func__);
	if (mask == -1)
		return 0;
	if (mask == 0) {
		rc = msm_camera_cci_i2c_write(client, addr, data, data_type);
	} else {
		rc = msm_camera_cci_i2c_read(client, addr, &reg_data,
			data_type);
		if (rc < 0) {
			CDBG("%s read fail\n", __func__);
			return rc;
		}
		reg_data &= ~mask;
		reg_data |= (data & mask);
		rc = msm_camera_cci_i2c_write(client, addr, reg_data,
			data_type);
		if (rc < 0)
			CDBG("%s write fail\n", __func__);
	}
	return rc;
}

int32_t msm_camera_cci_i2c_write_conf_tbl(
	struct msm_camera_i2c_client *client,
	struct msm_camera_i2c_reg_conf *reg_conf_tbl, uint16_t size,
	enum msm_camera_i2c_data_type data_type)
{
	int i;
	int32_t rc = -EFAULT;
	for (i = 0; i < size; i++) {
		enum msm_camera_i2c_data_type dt;
		if (reg_conf_tbl->cmd_type == MSM_CAMERA_I2C_CMD_POLL) {
			rc = msm_camera_cci_i2c_poll(client,
				reg_conf_tbl->reg_addr,
				reg_conf_tbl->reg_data,
				reg_conf_tbl->dt);
		} else {
			if (reg_conf_tbl->dt == 0)
				dt = data_type;
			else
				dt = reg_conf_tbl->dt;
			switch (dt) {
			case MSM_CAMERA_I2C_BYTE_DATA:
			case MSM_CAMERA_I2C_WORD_DATA:
				rc = msm_camera_cci_i2c_write(
					client,
					reg_conf_tbl->reg_addr,
					reg_conf_tbl->reg_data, dt);
				break;
			case MSM_CAMERA_I2C_SET_BYTE_MASK:
				rc = msm_camera_cci_i2c_set_mask(client,
					reg_conf_tbl->reg_addr,
					reg_conf_tbl->reg_data,
					MSM_CAMERA_I2C_BYTE_DATA, 1);
				break;
			case MSM_CAMERA_I2C_UNSET_BYTE_MASK:
				rc = msm_camera_cci_i2c_set_mask(client,
					reg_conf_tbl->reg_addr,
					reg_conf_tbl->reg_data,
					MSM_CAMERA_I2C_BYTE_DATA, 0);
				break;
			case MSM_CAMERA_I2C_SET_WORD_MASK:
				rc = msm_camera_cci_i2c_set_mask(client,
					reg_conf_tbl->reg_addr,
					reg_conf_tbl->reg_data,
					MSM_CAMERA_I2C_WORD_DATA, 1);
				break;
			case MSM_CAMERA_I2C_UNSET_WORD_MASK:
				rc = msm_camera_cci_i2c_set_mask(client,
					reg_conf_tbl->reg_addr,
					reg_conf_tbl->reg_data,
					MSM_CAMERA_I2C_WORD_DATA, 0);
				break;
			case MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA:
				rc = msm_camera_cci_i2c_set_write_mask_data(
					client,
					reg_conf_tbl->reg_addr,
					reg_conf_tbl->reg_data,
					reg_conf_tbl->mask,
					MSM_CAMERA_I2C_BYTE_DATA);
				break;
			default:
				pr_err("%s: Unsupport data type: %d\n",
					__func__, dt);
				break;
			}
		}
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}
	return rc;
}

int32_t msm_sensor_cci_i2c_util(struct msm_camera_i2c_client *client,
	uint16_t cci_cmd)
{
	int32_t rc = 0;
	struct msm_camera_cci_ctrl cci_ctrl;

	CDBG("%s line %d\n", __func__, __LINE__);
	cci_ctrl.cmd = cci_cmd;
	cci_ctrl.cci_info = client->cci_client;
	rc = v4l2_subdev_call(client->cci_client->cci_subdev,
			core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	if (rc < 0) {
		pr_err("%s line %d rc = %d\n", __func__, __LINE__, rc);
		return rc;
	}
	return cci_ctrl.status;
}
