#include <linux/i2c.h>
#include <mach/camera2.h>
#include <mach/devices_cmdline.h>
#include "msm_camera_i2c.h"
#include "msm_sd.h"
#include "msm_cci.h"
#include "../msm_actuator.h"
#include "Ois.h"



static struct msm_camera_i2c_client *binder_i2c_client = NULL;
static int g_ois_mode = 0; 
static uint8_t g_otp_data[100] = {0};
static uint8_t g_otp_size = 0;

#define ILLEGAL_CMD_INPUT_VALUE 10000
static int16_t g_ois_debug_enable = 0;
static int16_t g_ois_level = ILLEGAL_CMD_INPUT_VALUE;
static int16_t g_compensation_angle = ILLEGAL_CMD_INPUT_VALUE;

uint16_t get_ois_debug_enable(void);
uint16_t get_ois_level(void);
uint16_t get_compensation_angle(void);

static int HtcActOisBinder_sysfs_init(void);

int g_support_ois = 0;


void RegReadA_lc898111(uint16_t addr, uint8_t* data)
{
	int32_t rc = 0;

	if (binder_i2c_client == NULL ||
		binder_i2c_client->i2c_func_tbl == NULL ||
		binder_i2c_client->i2c_func_tbl->i2c_read_seq == NULL)
		return;

	rc = binder_i2c_client->i2c_func_tbl->i2c_read_seq(binder_i2c_client, addr, (uint8_t *)data, 1);

	
	if (rc < 0) {
		pr_err("%s i2c read failed (%d)\n", __func__, rc);
		return;
	}
}

void RegWriteA_lc898111(uint16_t addr, uint8_t data)
{
	int32_t rc = 0;

	if (binder_i2c_client == NULL ||
		binder_i2c_client->i2c_func_tbl == NULL ||
		binder_i2c_client->i2c_func_tbl->i2c_write_seq == NULL)
		return;

	rc = binder_i2c_client->i2c_func_tbl->i2c_write_seq(binder_i2c_client, addr, (uint8_t *)&data, 1);

	
	if (rc < 0) {
		pr_err("%s i2c write failed (%d)\n", __func__, rc);
		return;
	}
}


void RamReadA_lc898111(uint16_t addr, uint16_t* data)
{
	int32_t rc = 0;
	uint8_t *data_ptr;
	uint8_t dummy[2];

	if (binder_i2c_client == NULL ||
		binder_i2c_client->i2c_func_tbl == NULL ||
		binder_i2c_client->i2c_func_tbl->i2c_read_seq == NULL)
		return;

	rc = binder_i2c_client->i2c_func_tbl->i2c_read_seq(binder_i2c_client, addr, (uint8_t *)data, 2);
	if (rc < 0) {
		pr_err("%s i2c read failed (%d)\n", __func__, rc);
		return;
	}

	data_ptr = (uint8_t *)data;
	dummy[0] = *(data_ptr+1);
	dummy[1] = *(data_ptr);
	*(data_ptr) = dummy[0];
	*(data_ptr+1) = dummy[1];
	
}

void RamWriteA_lc898111(uint16_t addr, uint16_t data)
{
	int32_t rc = 0;
	uint8_t *data_ptr;
	uint8_t dummy[2];

	if (binder_i2c_client == NULL ||
		binder_i2c_client->i2c_func_tbl == NULL ||
		binder_i2c_client->i2c_func_tbl->i2c_write_seq == NULL)
		return;

	data_ptr = (uint8_t *)(&data);
	dummy[0] = *(data_ptr+1);
	dummy[1] = *(data_ptr);
	*(data_ptr) = dummy[0];
	*(data_ptr+1) = dummy[1];

	rc = binder_i2c_client->i2c_func_tbl->i2c_write_seq(binder_i2c_client, addr, (uint8_t *)&data, 2);

	
	if (rc < 0) {
		pr_err("%s i2c write failed (%d)\n", __func__, rc);
		return;
	}
}

void RamRead32A_lc898111(uint16_t addr, uint32_t* data)
{
	int32_t rc = 0;
	uint8_t *data_ptr;
	uint8_t dummy[4];

	if (binder_i2c_client == NULL ||
		binder_i2c_client->i2c_func_tbl == NULL ||
		binder_i2c_client->i2c_func_tbl->i2c_read_seq == NULL)
		return;

	rc = binder_i2c_client->i2c_func_tbl->i2c_read_seq(binder_i2c_client, addr, (uint8_t *)data, 4);
	if (rc < 0) {
		pr_err("%s i2c read failed (%d)\n", __func__, rc);
		return;
	}

	data_ptr = (uint8_t *)data;
	dummy[0] = *(data_ptr+3);
	dummy[1] = *(data_ptr+2);
	dummy[2] = *(data_ptr+1);
	dummy[3] = *(data_ptr);
	*(data_ptr) = dummy[0];
	*(data_ptr+1) = dummy[1];
	*(data_ptr+2) = dummy[2];
	*(data_ptr+3) = dummy[3];
	
}

void RamWrite32A_lc898111(uint16_t addr, uint32_t data)
{
	int32_t rc = 0;
	uint8_t *data_ptr;

	uint8_t dummy[4];

	if (binder_i2c_client == NULL ||
		binder_i2c_client->i2c_func_tbl == NULL ||
		binder_i2c_client->i2c_func_tbl->i2c_write_seq == NULL)
		return;

	data_ptr = (uint8_t *)(&data);
	dummy[0] = *(data_ptr+3);
	dummy[1] = *(data_ptr+2);
	dummy[2] = *(data_ptr+1);
	dummy[3] = *(data_ptr);
	*(data_ptr) = dummy[0];
	*(data_ptr+1) = dummy[1];
	*(data_ptr+2) = dummy[2];
	*(data_ptr+3) = dummy[3];

	rc = binder_i2c_client->i2c_func_tbl->i2c_write_seq(binder_i2c_client, addr, (uint8_t *)&data, 4);

	
	if (rc < 0) {
		pr_err("%s i2c write failed (%d)\n", __func__, rc);
		return;
	}
}


void WitTim_lc898111(unsigned short ms)
{
    usleep(ms*1000);
}


void HtcActOisBinder_i2c_add_driver(struct msm_camera_i2c_client* cam_i2c_client)
{
	pr_info("[OIS]  %s\n", __func__);
	if (binder_i2c_client == NULL) {
		binder_i2c_client = kzalloc(sizeof(struct msm_camera_i2c_client), GFP_KERNEL);
		if(cam_i2c_client->i2c_func_tbl) {
			binder_i2c_client->i2c_func_tbl = kzalloc(sizeof(struct msm_camera_i2c_fn_t), GFP_KERNEL);
			memcpy(binder_i2c_client->i2c_func_tbl, cam_i2c_client->i2c_func_tbl, sizeof(struct msm_camera_i2c_fn_t));
		}
		if(cam_i2c_client->client) {
			binder_i2c_client->client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
			memcpy(binder_i2c_client->client, cam_i2c_client->client, sizeof(struct i2c_client));
		}
		if(cam_i2c_client->cci_client) {
			binder_i2c_client->cci_client = kzalloc(sizeof(struct msm_camera_cci_client), GFP_KERNEL);
			memcpy(binder_i2c_client->cci_client, cam_i2c_client->cci_client, sizeof(struct msm_camera_cci_client));
		}
		
		binder_i2c_client->addr_type = cam_i2c_client->addr_type;

		if(binder_i2c_client->cci_client->sid) {
			pr_info("[OIS]  sid=0x%x\n", binder_i2c_client->cci_client->sid<<1);
        } else {
			pr_info("[OIS]	addr=0x%x\n", binder_i2c_client->client->addr);
        }
	}

	HtcActOisBinder_sysfs_init();
}


void HtcActOisBinder_open_init(void)
{
	uint8_t ois_data_8;
	uint16_t ois_data_16;
	unsigned long ois_data_32;

	if (binder_i2c_client == NULL)
		return;

	pr_info("[OIS]  %s  start\n", __func__);
	pr_info("[OIS]  %s  FW_Version=0x%x\n", __func__, RdFwVr());

#if 0
	RegReadA_lc898111(0x027F, &ois_data_8); 
	pr_info("[OIS]  0x027F read : 0x%x\n", ois_data_8);
#endif


	IniSet(); 

	
	if (g_otp_size > 0)
	{
		pr_info("[OIS]  %s  g_otp_size=%d\n", __func__, g_otp_size);

		
		ois_data_16 = (g_otp_data[0] << 8) + g_otp_data[1];
		RamWriteA_lc898111(0x1114, ois_data_16); 
		ois_data_16 = (g_otp_data[2] << 8) + g_otp_data[3];
		RamWriteA_lc898111(0x1116, ois_data_16); 
		ois_data_16 = (g_otp_data[4] << 8) + g_otp_data[5];
		RamWriteA_lc898111(0x1115, ois_data_16); 
		ois_data_16 = (g_otp_data[6] << 8) + g_otp_data[7];
		RamWriteA_lc898111(0x1117, ois_data_16); 
		ois_data_16 = (g_otp_data[8] << 8) + g_otp_data[9];
		RamWriteA_lc898111(0x1102, ois_data_16); 
		ois_data_16 = (g_otp_data[10] << 8) + g_otp_data[11];
		RamWriteA_lc898111(0x1105, ois_data_16); 
		ois_data_16 = (g_otp_data[12] << 8) + g_otp_data[13];
		RamWriteA_lc898111(0x132A, ois_data_16); 
		ois_data_16 = (g_otp_data[14] << 8) + g_otp_data[15];
		RamWriteA_lc898111(0x136A, ois_data_16); 
		ois_data_16 = (g_otp_data[16] << 8) + g_otp_data[17];
		RamWriteA_lc898111(0x1127, ois_data_16); 
		ois_data_16 = (g_otp_data[18] << 8) + g_otp_data[19];
		RamWriteA_lc898111(0x1167, ois_data_16); 

		
		ois_data_8 = g_otp_data[20];
		RegWriteA_lc898111(0x03A0, ois_data_8); 
		ois_data_8 = g_otp_data[21];
		RegWriteA_lc898111(0x03A1, ois_data_8); 
		ois_data_8 = g_otp_data[22];
		RegWriteA_lc898111(0x03A2, ois_data_8); 
		ois_data_8 = g_otp_data[23];
		RegWriteA_lc898111(0x03A3, ois_data_8); 

		
		
		
		pr_info("[OIS]  Gyro Gain X0  : 0x%x\n", g_otp_data[24]);
		g_otp_data[24] = g_otp_data[24] | 0x80;
		pr_info("[OIS]  Corrected Gyro Gain X0  : 0x%x\n", g_otp_data[24]);
		
		ois_data_32 = (g_otp_data[24] << 24) + (g_otp_data[25] << 16) + (g_otp_data[26] << 8) + g_otp_data[27];
		RamWrite32A(0x1828, ois_data_32); 
		ois_data_32 = (g_otp_data[28] << 24) + (g_otp_data[29] << 16) + (g_otp_data[30] << 8) + g_otp_data[31];
		RamWrite32A(0x1928, ois_data_32); 


		
		ois_data_8 = g_otp_data[32];
		RegWriteA_lc898111(0x0264, ois_data_8); 
	}


#if 0
	
	
	RamWrite32A( 0x1808, 0x3F99999A ) ;	
	RamWrite32A( 0x1809, 0x3F99999A ) ;	
	RamWrite32A( 0x1908, 0x3F99999A ) ;	
	RamWrite32A( 0x1909, 0x3F99999A ) ;	
#endif


	RtnCen(0); 
	
	SetPanTiltMode(ON);

	pr_info("[OIS]  %s  g_ois_mode=%d\n", __func__, g_ois_mode);
	if (g_ois_mode != 0) {
		ClrGyr(0x06, CLR_GYR_DLY_RAM);
		OisEna(); 
		SetGcf(5); 
	}
	

#if 0
	RegReadA_lc898111(0x0084, &ois_data_8);
	pr_info("[OIS]  0x0084 read : 0x%x\n", ois_data_8);

	RamReadA_lc898111(0x1308, &ois_data_16);
	pr_info("[OIS]  0x1308 read : 0x%x\n", ois_data_16);

	RegReadA_lc898111(0x0084, &ois_data_8);
	pr_info("[OIS]  0x0084 read : 0x%x\n", ois_data_8);

	RamReadA_lc898111(0x1348, &ois_data_16);
	pr_info("[OIS]  0x1348 read : 0x%x\n", ois_data_16);
#endif
	pr_info("[OIS]  %s  end\n", __func__);
}


void HtcActOisBinder_power_down(void)
{
	if (binder_i2c_client == NULL)
		return;

	pr_info("[OIS]  %s  start\n", __func__);

	
	RtnCen(0); 

	SrvCon(X_DIR, OFF); 
	SrvCon(Y_DIR, OFF); 

	pr_info("[OIS]  %s  end\n", __func__);
}


void HtcActOisBinder_set_OIS_OTP(uint8_t *otp_data, uint8_t otp_size)
{
	int i;

	pr_info("[OIS]  %s  start\n", __func__);

	g_otp_size = otp_size;
	for (i=0 ;i<g_otp_size; i++)
	{
		g_otp_data[i] = *(otp_data+i);
		pr_info("[OIS]  OTP[%d]=0x%x\n", i, *(otp_data+i));
	}
	g_support_ois = 1;
	pr_info("[OIS]  %s g_support_ois:%d end\n", __func__, g_support_ois);
}

static int32_t check_if_enable_OIS_MFG_debug(void)
{
	if (board_mfg_mode())
		return 1;
	else if (get_ois_debug_enable())
		return 1;
	else
		return 0;
}


static int32_t process_OIS_MFG_debug(void)
{
	int32_t rc = 0;
	uint16_t ois_level;
	uint16_t compensation_angle;
#if 0
	uint16_t ois_data_1 = 0, ois_data_2 = 0, ois_data_3 = 0, ois_data_4 = 0;
#endif

	pr_info("[OIS]  %s called\n", __func__);

#if 0
	
	RamReadA_lc898111(0x1103, &ois_data_1); 
	RamReadA_lc898111(0x1106, &ois_data_2); 
	pr_info("[OIS]  OIS_ENABLE_DEBUG  0x1103 read : 0x%04x , 0x1106 read : 0x%04x\n", ois_data_1 , ois_data_2);

	RamReadA_lc898111(0x1101, &ois_data_3); 
	RamReadA_lc898111(0x1104, &ois_data_4); 
	pr_info("[OIS]  OIS_ENABLE_DEBUG  0x1101 read : 0x%04x , 0x1104 read : 0x%04x\n", ois_data_3 , ois_data_4);
#endif

	
	ois_level = get_ois_level();
	if (ois_level <= 6)
	{
		pr_info("[OIS]  set OIS level : %d\n", ois_level);
		SetGcf(ois_level); 
	}


	
	compensation_angle = get_compensation_angle();
	if(compensation_angle < ILLEGAL_CMD_INPUT_VALUE) {
		pr_info("[OIS]  set Compensation angle : %d\n", compensation_angle);
		if (compensation_angle == 800) {
			pr_info("[OIS]  Apply Compensation angle as 0.8deg\n");
			
			RamWrite32A( 0x1808, 0x3FD1EB85 ) ;	
			RamWrite32A( 0x1809, 0x3FD1EB85 ) ;	
			RamWrite32A( 0x1908, 0x3FD1EB85 ) ;	
			RamWrite32A( 0x1909, 0x3FD1EB85 ) ;	
		} else if (compensation_angle == 700) {
			pr_info("[OIS]  Apply Compensation angle as 0.7deg\n");
			
			RamWrite32A( 0x1808, 0x3FB33333 ) ;	
			RamWrite32A( 0x1809, 0x3FB33333 ) ;	
			RamWrite32A( 0x1908, 0x3FB33333 ) ;	
			RamWrite32A( 0x1909, 0x3FB33333 ) ;	
		} else if (compensation_angle == 600) {
			pr_info("[OIS]  Apply Compensation angle as 0.6deg\n");
			
			RamWrite32A( 0x1808, 0x3F99999A ) ;	
			RamWrite32A( 0x1809, 0x3F99999A ) ;	
			RamWrite32A( 0x1908, 0x3F99999A ) ;	
			RamWrite32A( 0x1909, 0x3F99999A ) ;	
		} else {
			pr_info("[OIS]  value is not allowed, Apply default Compensation angle as 0.8deg\n");
			
			RamWrite32A( 0x1808, 0x3FD1EB85 ) ;	
			RamWrite32A( 0x1809, 0x3FD1EB85 ) ;	
			RamWrite32A( 0x1908, 0x3FD1EB85 ) ;	
			RamWrite32A( 0x1909, 0x3FD1EB85 ) ;	
		}
	}

	return rc;
}


int32_t HtcActOisBinder_act_set_ois_mode(int ois_mode)
{
	int32_t rc = 0;

	pr_info("[OIS]  %s  ois_mode=%d\n", __func__, ois_mode);

	g_ois_mode = ois_mode;

	RtnCen(0); 

	if (ois_mode != 0) {
		ClrGyr(0x06, CLR_GYR_DLY_RAM);
		OisEna(); 
		SetGcf(5); 
	}

	return rc;
}


int32_t HtcActOisBinder_mappingTbl_i2c_write(int startup_mode, struct sensor_actuator_info_t * sensor_actuator_info)
{
	int32_t rc = 0;
	camera_video_mode_type cur_cam_mode;
	uint32_t cur_line_cnt = 0;
	uint32_t cur_exp_time = 0;
	int32_t cur_zoom_level = 0;
	uint16_t cur_cmp_angle = 600;
	uint16_t cur_ois_level = 5;
	uint16_t ois_off = 0;

	if(g_ois_mode == 0) {
		pr_info("[OIS]  %s  OIS is OFF , g_ois_mode=%d\n", __func__, g_ois_mode);
		return rc;
	}

	if (check_if_enable_OIS_MFG_debug())
		process_OIS_MFG_debug();


	cur_cam_mode = sensor_actuator_info->cam_mode;
	cur_line_cnt = sensor_actuator_info->cur_line_cnt;
	cur_exp_time = sensor_actuator_info->cur_exp_time;
	cur_zoom_level = sensor_actuator_info->zoom_level;

	if (cur_cam_mode == CAM_MODE_CAMERA_PREVIEW) {

		cur_cmp_angle = 600;
		if (cur_exp_time >= (1000/24)) {
			cur_ois_level = 5;
		} else if (cur_exp_time >= (1000/48)) {
			cur_ois_level = 4;
		} else if (cur_exp_time >= (1000/83)) {
			cur_ois_level = 3;
		} else {
			ois_off = 1;
		}

		if (cur_zoom_level >= 45) {

			if (cur_exp_time >= (1000/24))
				cur_ois_level = 5;
			else
				cur_ois_level = 4;

			ois_off = 0;
		} else if (cur_zoom_level >= 30) {

			if (cur_exp_time >= (1000/24))
				cur_ois_level = 5;
			else
				cur_ois_level = 4;

			ois_off = 0;
		} else if (cur_zoom_level >= 15) {

			if (cur_exp_time >= (1000/24))
				cur_ois_level = 5;
			else
				cur_ois_level = 4;

			ois_off = 0;
		}

	} else if (cur_cam_mode == CAM_MODE_VIDEO_RECORDING) {

		if (cur_exp_time >= (1000/24)) {
			cur_ois_level= 5;
		} else if (cur_exp_time >= (1000/48)) {
			cur_ois_level= 4;
		} else if (cur_exp_time >= (1000/83)) {
			cur_ois_level= 4;
		} else {
			cur_ois_level= 4;
		}
		cur_cmp_angle = 600;

		if (cur_zoom_level >= 45) {
			if (cur_exp_time >= (1000/24)) {
				cur_ois_level = 5;
			} else if (cur_exp_time >= (1000/48)) {
				cur_ois_level = 4;
			} else if (cur_exp_time >= (1000/83)) {
				cur_ois_level = 4;
			} else {
				cur_ois_level = 4;
			}
		} else if (cur_zoom_level >= 30) {
			if (cur_exp_time >= (1000/24)) {
				cur_ois_level = 5;
			} else {
				cur_ois_level = 4;
			}
		} else if (cur_zoom_level >= 15) {

			if (cur_exp_time >= (1000/24)) {
				cur_ois_level = 5;
			} else {
				cur_ois_level = 4;
			}
		}

		if (board_mfg_mode()) {
			pr_info("[RUMBA_S] mfg mode\n");
			cur_ois_level= 5;
		}
	} else {
		cur_ois_level= 5;
	}

	pr_info("[OIS] ois_off=%d, cur_cam_mode=%d,  cur_line_cnt=%d, cur_exp_time=%d, cur_ois_level=%d, cur_zoom_level=%d\n",
		ois_off,  cur_cam_mode, cur_line_cnt, cur_exp_time, cur_ois_level, cur_zoom_level);

	if (ois_off) {
		
		cur_ois_level= 0;
	}


		if (cur_ois_level <= 6)
		{
			pr_info("[OIS]  set OIS level : %d\n", cur_ois_level);
			SetGcf(cur_ois_level); 
		}

#if 0
		if (cur_cmp_angle == 800) {
			pr_info("[OIS]  Apply Compensation angle as 0.8deg\n");
			
			RamWrite32A( 0x1808, 0x3FD1EB85 ) ;	
			RamWrite32A( 0x1809, 0x3FD1EB85 ) ;	
			RamWrite32A( 0x1908, 0x3FD1EB85 ) ;	
			RamWrite32A( 0x1909, 0x3FD1EB85 ) ;	
		} else if (cur_cmp_angle == 700) {
			pr_info("[OIS]  Apply Compensation angle as 0.7deg\n");
			
			RamWrite32A( 0x1808, 0x3FB33333 ) ;	
			RamWrite32A( 0x1809, 0x3FB33333 ) ;	
			RamWrite32A( 0x1908, 0x3FB33333 ) ;	
			RamWrite32A( 0x1909, 0x3FB33333 ) ;	
		} else if (cur_cmp_angle == 600) {
			pr_info("[OIS]  Apply Compensation angle as 0.6deg\n");
			
			RamWrite32A( 0x1808, 0x3F99999A ) ;	
			RamWrite32A( 0x1809, 0x3F99999A ) ;	
			RamWrite32A( 0x1908, 0x3F99999A ) ;	
			RamWrite32A( 0x1909, 0x3F99999A ) ;	
		} else {
			pr_info("[OIS]  Apply Compensation angle : value is not allowed\n");
		}
#endif

	return rc;
}


uint16_t get_ois_debug_enable(void)
{
	return g_ois_debug_enable;
}
static ssize_t ois_debug_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int input;
	pr_info("[OIS]  %s called\n", __func__);

	sscanf(buf, "%d ",&input);
	pr_info("%s  input: %d\n",__func__,input);
	g_ois_debug_enable = input;
	return size;
}
static ssize_t ois_debug_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	pr_info("[OIS]  %s called\n", __func__);

	sprintf(buf, "%d\n", g_ois_debug_enable);
	ret = strlen(buf) + 1;
	return ret;
}
static DEVICE_ATTR(ois_debug_enable, S_IRUGO | S_IWUSR, ois_debug_enable_show, ois_debug_enable_store);


uint16_t get_ois_level(void)
{
	return g_ois_level;
}
static ssize_t ois_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int input;
	pr_info("[OIS]  %s called\n", __func__);

	sscanf(buf, "%d ",&input);
	pr_info("%s  input: %d\n",__func__,input);
	g_ois_level = input;
	return size;
}
static ssize_t ois_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	pr_info("[OIS]  %s called\n", __func__);

	sprintf(buf, "%d\n", g_ois_level);
	ret = strlen(buf) + 1;
	return ret;
}
static DEVICE_ATTR(ois_level, S_IRUGO | S_IWUSR, ois_level_show, ois_level_store);


uint16_t get_compensation_angle(void)
{
	return g_compensation_angle;
}
static ssize_t compensation_angle_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int input;
	pr_info("[OIS]  %s called\n", __func__);

	sscanf(buf, "%d ",&input);
	pr_info("%s  input: %d\n",__func__,input);
	g_compensation_angle = input;
	return size;
}
static ssize_t compensation_angle_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	pr_info("[OIS]  %s called\n", __func__);

	sprintf(buf, "%d\n", g_compensation_angle);
	ret = strlen(buf) + 1;
	return ret;
}
static DEVICE_ATTR(compensation_angle, S_IRUGO | S_IWUSR, compensation_angle_show, compensation_angle_store);


static struct kobject *android_htcactoisbinder;

static int HtcActOisBinder_sysfs_init(void)
{
	int ret = 0;

	pr_info("%s: HtcActOisBinder:kobject creat and add\n", __func__);

	android_htcactoisbinder = kobject_create_and_add("android_cam_ois", NULL);
	if (android_htcactoisbinder == NULL) {
		pr_info("HtcActOisBinder_sysfs_init: subsystem_register failed\n");
		ret = -ENOMEM;
		return ret ;
	}
	pr_info("HtcActOisBinder:sysfs_create_file\n");
	ret = sysfs_create_file(android_htcactoisbinder, &dev_attr_ois_debug_enable.attr);
	if (ret) {
		pr_info("HtcActOisBinder_sysfs_init: sysfs_create_file failed\n");
		kobject_del(android_htcactoisbinder);
	}

	ret = sysfs_create_file(android_htcactoisbinder, &dev_attr_ois_level.attr);
	if (ret) {
		pr_info("HtcActOisBinder_sysfs_init: sysfs_create_file failed\n");
		kobject_del(android_htcactoisbinder);
	}

	ret = sysfs_create_file(android_htcactoisbinder, &dev_attr_compensation_angle.attr);
	if (ret) {
		pr_info("HtcActOisBinder_sysfs_init: sysfs_create_file failed\n");
		kobject_del(android_htcactoisbinder);
	}

	return ret;
}

