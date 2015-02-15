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
#define pr_fmt(fmt) "[CAM]ti201 %s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include "msm_sd.h"
#include "msm_actuator.h"
#include "msm_cci.h"

/*#define MSM_ACUTUATOR_DEBUG*/
#undef CDBG
#ifdef MSM_ACUTUATOR_DEBUG
#define CDBG(fmt, args...) pr_info(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

static struct msm_actuator msm_vcm_actuator_table;
static struct msm_actuator msm_piezo_actuator_table;

static struct msm_actuator *actuators[] = {
	&msm_vcm_actuator_table,
	&msm_piezo_actuator_table,
};

/*HTC_START, Get step position table from user space for actuator modulation*/
#if 0
static int ti201_sharp_kernel_step_table[] = {
	304, 309, 314, 319, 324, 329, 333, 339, 344, 350,
	355, 361, 366, 372, 378, 385, 391, 399, 406, 416,
	425, 436, 446, 485, 469, 482, 494, 508, 522, 537, 552
};

static int ti201_liteon_kernel_step_table[] = {
	225, 229, 233, 237, 241, 245, 249, 254, 258, 263,
	267, 272, 276, 281, 286, 291, 296, 303, 309, 317,
	324, 333, 341, 351, 360, 371, 381, 393, 404, 417, 429
};
#endif
/*HTC_END*/

struct msm_actuator_ext ti201_act_ext = {
	.is_ois_supported = 1,
	.ois_slave_id     = 0x48,
	.small_step_damping = 47,
	.medium_step_damping = 75,
	.big_step_damping = 100,
	.is_af_infinity_supported = 0,
	.actuators        = actuators,
};

static int32_t msm_actuator_init_step_table(struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_set_info_t *set_info)
{
/*HTC_START, Get step position table from user space for actuator modulation*/
#if 1
    int i = 0;
    CDBG("Enter\n");

    if (a_ctrl->step_position_table != NULL)
      kfree(a_ctrl->step_position_table);
    a_ctrl->step_position_table = NULL;

    /* Fill step position table */
    a_ctrl->step_position_table =
    kmalloc(sizeof(uint16_t) *(set_info->af_tuning_params.total_steps + 1), GFP_KERNEL);

    if (a_ctrl->step_position_table == NULL)
    {
        pr_err("a_ctrl->step_position_table create fail\n");
        return -ENOMEM;
    }

    if (set_info->step_position_table == NULL)
    {
        pr_err("set_info->step_position_table NULL\n");
        return -ENOMEM;
    }

    for (i = 0; i <= a_ctrl->total_steps; i++)
    {
        a_ctrl->step_position_table[i] = set_info->step_position_table[i];
        CDBG("step_position_table(%d):%d;",i, a_ctrl->step_position_table[i]);
    }
#else
	int16_t step_index = 0, region_index = 0;
	uint16_t step_boundary = 0;
	uint32_t max_code_size = 1;
	uint16_t data_size = set_info->actuator_params.data_size;
	int *kernel_step_table;
	uint16_t kernel_step_table_size;

	CDBG("Enter\n");

	for (; data_size > 0; data_size--)
		max_code_size *= 2;

	kfree(a_ctrl->step_position_table);
	a_ctrl->step_position_table = NULL;

	/* Fill step position table */
	a_ctrl->step_position_table =
		kmalloc(sizeof(uint16_t) *
		(set_info->af_tuning_params.total_steps + 1), GFP_KERNEL);

	if (a_ctrl->step_position_table == NULL)
		return -ENOMEM;

	step_boundary =	a_ctrl->region_params[region_index].step_bound[MOVE_NEAR];
	kernel_step_table = (a_ctrl->af_OTP_info.VCM_Vendor == 0x1)? ti201_sharp_kernel_step_table: ti201_liteon_kernel_step_table;
	kernel_step_table_size = (a_ctrl->af_OTP_info.VCM_Vendor == 0x1) ?
								(sizeof(ti201_sharp_kernel_step_table) / sizeof(int)) :
								(sizeof(ti201_liteon_kernel_step_table) / sizeof(int));

	if (a_ctrl->af_OTP_info.VCM_OTP_Read)
		a_ctrl->initial_code = a_ctrl->af_OTP_info.VCM_Infinity;
	else
		a_ctrl->initial_code = kernel_step_table[0];

	a_ctrl->step_position_table[0] = a_ctrl->initial_code;
	pr_info("step_position_table[%d]=%d", 0, a_ctrl->step_position_table[0]);
	if((set_info->af_tuning_params.total_steps + 1) == kernel_step_table_size) {
		for(step_index = 1; step_index <= step_boundary; step_index++) {
			if (a_ctrl->af_OTP_info.VCM_OTP_Read)
				a_ctrl->step_position_table[step_index] = a_ctrl->af_OTP_info.VCM_Infinity +
					(kernel_step_table[step_index] - kernel_step_table[0]) * (a_ctrl->af_OTP_info.VCM_Macro - a_ctrl->af_OTP_info.VCM_Infinity) /
					(kernel_step_table[set_info->af_tuning_params.total_steps] - kernel_step_table[0]);
			else
				a_ctrl->step_position_table[step_index] = kernel_step_table[step_index];
			pr_info("step_position_table[%d]=%d", step_index, a_ctrl->step_position_table[step_index]);
		}
	} else {
		pr_err("%s: Setp table size is unmatched!!", __func__);
	}
#endif
/*HTC_END*/
	CDBG("Exit\n");
	return 0;
}

static int32_t msm_actuator_iaf_move_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int i,rc;
	struct msm_camera_i2c_reg_setting reg_setting;

	for(i = 0;i<30;i++){
		if(a_ctrl->step_position_table[i] > move_params->num_steps){
			pr_info("find step step_position_table[%d]:%d move_params->num_steps:%d",i,
							a_ctrl->step_position_table[i],move_params->num_steps);
			break;
		}
	}
	a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
			a_ctrl->step_position_table[i],
			move_params->ringing_params[a_ctrl->curr_region_index].hw_params, 
			move_params->ringing_params[a_ctrl->curr_region_index].damping_delay);

	reg_setting.reg_setting = a_ctrl->i2c_reg_tbl;
	reg_setting.data_type = a_ctrl->i2c_data_type;
	reg_setting.size = a_ctrl->i2c_tbl_index;
	rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_table_w_microdelay(
		&a_ctrl->i2c_client,
		&reg_setting);
	return 1;
}

static int32_t msm_actuator_move_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int32_t rc = 0;
	int8_t sign_dir = move_params->sign_dir;
	uint16_t step_boundary = 0;
	uint16_t target_step_pos = 0;
	uint16_t target_lens_pos = 0;
	int16_t dest_step_pos = move_params->dest_step_pos;
	uint16_t curr_lens_pos = 0;
	int dir = move_params->dir;
	int32_t num_steps = move_params->num_steps;
	struct msm_camera_i2c_reg_setting reg_setting;

	CDBG("called, dir %d, num_steps %d\n", dir, num_steps);

	if (dest_step_pos == a_ctrl->curr_step_pos)
		return rc;

	curr_lens_pos = a_ctrl->step_position_table[a_ctrl->curr_step_pos];
	a_ctrl->i2c_tbl_index = 0;
	CDBG("curr_step_pos =%d dest_step_pos =%d curr_lens_pos=%d\n",
		a_ctrl->curr_step_pos, dest_step_pos, curr_lens_pos);

	while (a_ctrl->curr_step_pos != dest_step_pos) {
		step_boundary =
			a_ctrl->region_params[a_ctrl->curr_region_index].step_bound[dir];
		CDBG("curr_region_index %d, dest_step_pos %d, step_boundary %d, sign_dir %d\n", a_ctrl->curr_region_index, dest_step_pos, step_boundary, sign_dir);

		if ((dest_step_pos * sign_dir) <=
			(step_boundary * sign_dir)) {

			target_step_pos = dest_step_pos;
			target_lens_pos =
				a_ctrl->step_position_table[target_step_pos];
			CDBG("target_step_pos %d, target_lens_pos %d\n", target_step_pos, target_lens_pos);

			a_ctrl->func_tbl->actuator_write_focus(a_ctrl,
					curr_lens_pos,
					&(move_params->
						ringing_params[a_ctrl->
						curr_region_index]),
					sign_dir,
					target_lens_pos);
			curr_lens_pos = target_lens_pos;

		} else {
			target_step_pos = step_boundary;
			target_lens_pos =
				a_ctrl->step_position_table[target_step_pos];
			CDBG("target_step_pos %d, target_lens_pos %d\n", target_step_pos, target_lens_pos);

			a_ctrl->func_tbl->actuator_write_focus(a_ctrl,
					curr_lens_pos,
					&(move_params->ringing_params[a_ctrl->
						curr_region_index]),
					sign_dir,
					target_lens_pos);
			curr_lens_pos = target_lens_pos;

			a_ctrl->curr_region_index += sign_dir;
		}
		a_ctrl->curr_step_pos = target_step_pos;
		CDBG("curr_step_pos =%d, curr_lens_pos=%d\n",
			a_ctrl->curr_step_pos, curr_lens_pos);
	}

	CDBG("sid = 0x%x (0x%x)\n",
		 a_ctrl->i2c_client.cci_client->sid, a_ctrl->i2c_client.cci_client->sid<<1);

	reg_setting.reg_setting = a_ctrl->i2c_reg_tbl;
	reg_setting.data_type = a_ctrl->i2c_data_type;
	reg_setting.size = a_ctrl->i2c_tbl_index;
	rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_table_w_microdelay(
		&a_ctrl->i2c_client,
		&reg_setting);

	if (rc < 0) {
		pr_err("i2c write error:%d\n", rc);
		return rc;
	}
	a_ctrl->i2c_tbl_index = 0;
	CDBG("Exit\n");

	return rc;
}

static void msm_actuator_write_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	uint16_t curr_lens_pos,
	struct damping_params_t *damping_params,
	int8_t sign_direction,
	int16_t code_boundary)
{
	int16_t next_lens_pos = 0;
	uint16_t damping_code_step = 0;
	uint16_t wait_time = 0;
	int16_t tmp_curr_step_pos = a_ctrl->curr_step_pos;
	int16_t step_pos_step = 1;

	CDBG("Enter\n");

	damping_code_step = damping_params->damping_step;
	wait_time = damping_params->damping_delay;

	/* Write code based on damping_code_step in a loop */

	if(tmp_curr_step_pos > 0 && tmp_curr_step_pos < a_ctrl->total_steps) {
		tmp_curr_step_pos += (sign_direction * step_pos_step);
		if(tmp_curr_step_pos > 0 && tmp_curr_step_pos < a_ctrl->total_steps) {
			next_lens_pos = a_ctrl->step_position_table[tmp_curr_step_pos];
			for (; (sign_direction * next_lens_pos) <= (sign_direction * code_boundary) ;) {
				CDBG("next_lens_pos %d\n", next_lens_pos);

				a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
					next_lens_pos, damping_params->hw_params, wait_time);

				curr_lens_pos = next_lens_pos;
				tmp_curr_step_pos += (sign_direction * step_pos_step);
				if(tmp_curr_step_pos > 0 && tmp_curr_step_pos < a_ctrl->total_steps)
					next_lens_pos = a_ctrl->step_position_table[tmp_curr_step_pos];
				else
					break;
			}
		}
	}

	if (curr_lens_pos != code_boundary) {
		a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
			code_boundary, damping_params->hw_params, wait_time);
	}
	CDBG("Exit\n");
}

static void msm_actuator_parse_i2c_params(struct msm_actuator_ctrl_t *a_ctrl,
	int16_t next_lens_position, uint32_t hw_params, uint16_t delay)
{
	struct msm_actuator_reg_params_t *write_arr = a_ctrl->reg_tbl;
	uint32_t hw_dword = hw_params;
	uint16_t value = 0;
	uint32_t size = a_ctrl->reg_tbl_size, i = 0;
	uint16_t i2c_byte1 = 0, i2c_byte2 = 0;
	struct msm_camera_i2c_reg_array *i2c_tbl = a_ctrl->i2c_reg_tbl;

	CDBG("Enter\n");
	for (i = 0; i < size; i++) {
		if (write_arr[i].reg_write_type == MSM_ACTUATOR_WRITE_DAC) {
			value = (next_lens_position <<
				write_arr[i].data_shift) |
				((hw_dword & write_arr[i].hw_mask) >>
				write_arr[i].hw_shift);

			pr_info("value=(%d << %d) | ((0x%x & 0x%x) >> %d)=0x%x\n",
				next_lens_position, 
				write_arr[i].data_shift,
				hw_dword, write_arr[i].hw_mask,
				write_arr[i].hw_shift, value);

			CDBG("[%d]reg_addr=0x%x\n", i, write_arr[i].reg_addr);
			if (write_arr[i].reg_addr != 0xFFFF) {
				i2c_byte1 = write_arr[i].reg_addr;
				i2c_byte2 = value;
				if (size != (i+1)) {
					i2c_byte2 = (value & 0x0300) >> 8;
					i2c_tbl[a_ctrl->i2c_tbl_index].
						reg_addr = i2c_byte1;
					i2c_tbl[a_ctrl->i2c_tbl_index].
						reg_data = i2c_byte2;
					i2c_tbl[a_ctrl->i2c_tbl_index].
						delay = 0;
					pr_info("[%d] : i2c_byte1:0x%x, i2c_byte2:0x%x\n", a_ctrl->i2c_tbl_index, i2c_byte1, i2c_byte2);
					a_ctrl->i2c_tbl_index++;
					i++;
					i2c_byte1 = write_arr[i].reg_addr;
					i2c_byte2 = value & 0xFF;
				}
			} else {
				i2c_byte1 = (value & 0xFF00) >> 8;
				i2c_byte2 = value & 0xFF;
			}
		} else {
			i2c_byte1 = write_arr[i].reg_addr;
			i2c_byte2 = (hw_dword & write_arr[i].hw_mask) >>
				write_arr[i].hw_shift;
		}
		CDBG("[%d] : i2c_byte1:0x%x, i2c_byte2:0x%x\n", a_ctrl->i2c_tbl_index, i2c_byte1, i2c_byte2);
		i2c_tbl[a_ctrl->i2c_tbl_index].reg_addr = i2c_byte1;
		i2c_tbl[a_ctrl->i2c_tbl_index].reg_data = i2c_byte2;
		i2c_tbl[a_ctrl->i2c_tbl_index].delay = delay;
		a_ctrl->i2c_tbl_index++;
	}
	CDBG("Exit\n");
}

/*HTC_START Harvey 20130701 - Set otp af value*/
int32_t ti201_act_set_af_value(struct msm_actuator_ctrl_t *a_ctrl, af_value_t af_value)
{
	int32_t rc = 0;
	uint8_t OTP_data[8] = {0,0,0,0,0,0,0,0};
	int16_t VCM_Infinity = 0;
	int32_t otp_deviation = 0;

	CDBG("Enter\n");

	OTP_data[0] = af_value.VCM_START_MSB;
	OTP_data[1] = af_value.VCM_START_LSB;
	OTP_data[2] = af_value.AF_INF_MSB;
	OTP_data[3] = af_value.AF_INF_LSB;
	OTP_data[4] = af_value.AF_MACRO_MSB;
	OTP_data[5] = af_value.AF_MACRO_LSB;
	/*opt diviation depends on different trace wide*/

	if (OTP_data[2] || OTP_data[3] || OTP_data[4] || OTP_data[5]) {
		a_ctrl->af_OTP_info.VCM_OTP_Read = true;
		a_ctrl->af_OTP_info.VCM_Vendor = af_value.VCM_VENDOR;
		otp_deviation = (a_ctrl->af_OTP_info.VCM_Vendor == 0x1/*sharp*/) ? 160 : 60;
		a_ctrl->af_OTP_info.VCM_Start = 0;
		pr_info("");
		VCM_Infinity = (int16_t)(OTP_data[2]<<8 | OTP_data[3]) - otp_deviation;
		if (VCM_Infinity < 0){
			a_ctrl->af_OTP_info.VCM_Infinity = 0;
		}else{
			a_ctrl->af_OTP_info.VCM_Infinity = VCM_Infinity;
		}
		a_ctrl->af_OTP_info.VCM_Macro = (OTP_data[4]<<8 | OTP_data[5]);
	}
	pr_info("OTP_data[2] %d OTP_data[3] %d OTP_data[4] %d OTP_data[5] %d\n",
		OTP_data[2], OTP_data[3], OTP_data[4], OTP_data[5]);
	pr_info("VCM_Start = %d\n", a_ctrl->af_OTP_info.VCM_Start);
	pr_info("VCM_Infinity = %d\n", a_ctrl->af_OTP_info.VCM_Infinity);
	pr_info("VCM_Macro = %d\n", a_ctrl->af_OTP_info.VCM_Macro);
	pr_info("VCM Module vendor =  = %d\n", a_ctrl->af_OTP_info.VCM_Vendor);
	return rc;
}
/*HTC_END*/

int32_t ti201_act_set_ois_mode(struct msm_actuator_ctrl_t *a_ctrl, int ois_mode)
{
	int32_t rc = 0;

	CDBG("Enter\n");

#if defined(CONFIG_ACT_OIS_BINDER)
	if (a_ctrl->oisbinder_act_set_ois_mode)
		rc = a_ctrl->oisbinder_act_set_ois_mode(ois_mode);
#endif
	return rc;
}

int32_t ti201_act_update_ois_tbl(struct msm_actuator_ctrl_t *a_ctrl, struct sensor_actuator_info_t * sensor_actuator_info)
{
	int32_t rc = 0;

	CDBG("Enter\n");

#if defined(CONFIG_ACT_OIS_BINDER)
	if (a_ctrl->oisbinder_mappingTbl_i2c_write)
		rc = a_ctrl->oisbinder_mappingTbl_i2c_write(sensor_actuator_info->startup_mode, sensor_actuator_info);
#endif
	return rc;
}

static struct msm_actuator msm_vcm_actuator_table = {
	.act_type = ACTUATOR_VCM,
	.func_tbl = {
		.actuator_init_step_table = msm_actuator_init_step_table,
		.actuator_move_focus = msm_actuator_move_focus,
		.actuator_iaf_move_focus = msm_actuator_iaf_move_focus,
		.actuator_write_focus = msm_actuator_write_focus,
		.actuator_set_default_focus = msm_actuator_set_default_focus,
		.actuator_init_focus = msm_actuator_init_focus,
		.actuator_parse_i2c_params = msm_actuator_parse_i2c_params,
/*HTC_START Harvey 20130628 - Porting OIS*/
		.actuator_set_ois_mode = msm_actuator_set_ois_mode,
		.actuator_update_ois_tbl = msm_actuator_update_ois_tbl,
/*HTC_END*/
	},
};

static struct msm_actuator msm_piezo_actuator_table = {
	.act_type = ACTUATOR_PIEZO,
	.func_tbl = {
		.actuator_init_step_table = NULL,
		.actuator_move_focus = msm_actuator_piezo_move_focus,
		.actuator_write_focus = NULL,
		.actuator_set_default_focus =
			msm_actuator_piezo_set_default_focus,
		.actuator_init_focus = msm_actuator_init_focus,
		.actuator_parse_i2c_params = msm_actuator_parse_i2c_params,
/*HTC_START Harvey 20130628 - Porting OIS*/
		.actuator_set_ois_mode = msm_actuator_set_ois_mode,
		.actuator_update_ois_tbl = msm_actuator_update_ois_tbl,
/*HTC_END*/
	},
};

