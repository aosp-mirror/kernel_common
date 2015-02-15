/* drivers/input/touchscreen/himax8528.c
 *
 * Copyright (C) 2013 HTC Corporation.
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

#include <linux/himax_852xD.h>
#include <linux/delay.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/async.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#if defined (CONFIG_ARCH_MSM)
#include <mach/msm_hsusb.h>
#include <mach/cable_detect.h>
#endif
#include <mach/board.h>
#include <asm/atomic.h>
#include <mach/board_htc.h>
#include <linux/firmware.h>
#if defined (CONFIG_UX500_SOC_DB8500)
#include <linux/mfd/abx500/ux500_chargalg.h>
#endif
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <mach/devices_cmdline.h>
#endif

#define HIMAX_I2C_RETRY_TIMES 10
#define FAKE_EVENT
#define SUPPORT_FINGER_DATA_CHECKSUM 0x0F


#define D(x...) printk(KERN_DEBUG "[TP] " x)
#define I(x...) printk(KERN_INFO "[TP] " x)
#define W(x...) printk(KERN_WARNING "[TP][WARNING] " x)
#define E(x...) printk(KERN_ERR "[TP][ERROR] " x)
#define DIF(x...) \
	if (debug_flag) \
	printk(KERN_DEBUG "[TP][DEBUG] " x) \
} while(0)

static int irq_enable_count = 0;
static unsigned char	IC_CHECKSUM               = 0;
static unsigned char	IC_TYPE                   = 0;

static int		HX_TOUCH_INFO_POINT_CNT   = 0;

static int		HX_RX_NUM                 = 0;
static int		HX_TX_NUM                 = 0;
static int		HX_BT_NUM                 = 0;
static int		HX_X_RES                  = 0;
static int		HX_Y_RES                  = 0;
static int		HX_MAX_PT                 = 0;
static bool		HX_XY_REVERSE             = false;
static bool		HX_INT_IS_EDGE            = false;

static unsigned int	FW_VER_MAJ_FLASH_ADDR;
static unsigned int 	FW_VER_MAJ_FLASH_LENG;
static unsigned int 	FW_VER_MIN_FLASH_ADDR;
static unsigned int 	FW_VER_MIN_FLASH_LENG;
static unsigned int 	CFG_VER_MAJ_FLASH_ADDR;
static unsigned int 	CFG_VER_MAJ_FLASH_LENG;
static unsigned int 	CFG_VER_MIN_FLASH_ADDR;
static unsigned int 	CFG_VER_MIN_FLASH_LENG;

static u16	FW_VER_MAJ_buff[12];	
static u16	FW_VER_MIN_buff[12];
static u16	CFG_VER_MAJ_buff[12];
static u16	CFG_VER_MIN_buff[12];

static bool	config_load		= false;

static uint8_t 	vk_press = 0x00;
static uint8_t 	AA_press = 0x00;
static uint8_t	IC_STATUS_CHECK	= 0xAA;
static uint8_t 	EN_NoiseFilter = 0x00;
static uint8_t	Last_EN_NoiseFilter = 0x00;
static int	hx_point_num	= 0;																	
static int	p_point_num	= 0xFFFF;
static int	tpd_key	   	= 0x00;
static int	tpd_key_old	= 0x00;
static int  touch_monitor_stop_flag = 0;
static int 	touch_monitor_stop_limit = 5;

static struct kobject *android_touch_kobj = NULL;										
static struct himax_i2c_platform_data_config_type28 *type28_selected = NULL;

	static unsigned char c1[]    = { 0x37, 0xFF, 0x08, 0xFF, 0x08};
	static unsigned char c2[]    = { 0x3F, 0x00};
	static unsigned char c3[]    = { 0x62, 0x01, 0x00, 0x01, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	static unsigned char c4[]    = { 0x63, 0x10, 0x00, 0x10, 0x30, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00};
	static unsigned char c5[]    = { 0x64, 0x01, 0x00, 0x01, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	static unsigned char c6[]    = { 0x65, 0x10, 0x00, 0x10, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	static unsigned char c7[]    = { 0x66, 0x01, 0x00, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};
	static unsigned char c8[]    = { 0x67, 0x10, 0x00, 0x10, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	static unsigned char c9[]    = { 0x68, 0x01, 0x00, 0x01, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	static unsigned char c10[]   = { 0x69, 0x10, 0x00, 0x10, 0x30, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
	static unsigned char c11[]   = { 0x6A, 0x01, 0x00, 0x01, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};
	static unsigned char c12[]   = { 0x6B, 0x10, 0x00, 0x10, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	static unsigned char c13[]   = { 0x6C, 0x01, 0x00, 0x01, 0x30, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
	static unsigned char c14[]   = { 0x6D, 0x10, 0x00, 0x10, 0x03, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
	static unsigned char c15[]   = { 0x6E, 0x01, 0x00, 0x01, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	static unsigned char c16[]   = { 0x6F, 0x10, 0x00, 0x10, 0x20, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};
	static unsigned char c17[]   = { 0x70, 0x01, 0x00, 0x01, 0x03, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
	static unsigned char c18[]   = { 0x7B, 0x03};
	static unsigned char c19[]   = { 0x7C, 0x00, 0xD8, 0x8C};
	static unsigned char c20[]   = { 0x7F, 0x00, 0x04, 0x0A, 0x0A, 0x04, 0x00, 0x00, 0x00};
	static unsigned char c21[]   = { 0xA4, 0x94, 0x62, 0x94, 0x86};
	static unsigned char c22[]   = { 0xB4, 0x04, 0x01, 0x01, 0x01, 0x01, 0x03, 0x0F, 0x04, 0x07, 0x04, 0x07, 0x04, 0x07, 0x00};
	static unsigned char c23[]   = { 0xB9, 0x01, 0x36};
	static unsigned char c24[]   = { 0xBA, 0x00};
	static unsigned char c25[]   = { 0xBB, 0x00};
	static unsigned char c26[]   = { 0xBC, 0x00, 0x00, 0x00, 0x00};
	static unsigned char c27[]   = { 0xBD, 0x04, 0x0C};
	static unsigned char c28[]   = { 0xC2, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	static unsigned char c29[]   = { 0xC5, 0x0A, 0x1D, 0x00, 0x10, 0x1A, 0x1E, 0x0B, 0x1D, 0x08, 0x16};
	static unsigned char c30[]   = { 0xC6, 0x1A, 0x10, 0x1F};
	static unsigned char c31[]   = { 0xC9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x15, 0x17, 0x17, 0x19, 0x19, 0x1F, 0x1F, 0x1B, 0x1B, 0x1D, 0x1D, 0x21, 0x21, 0x23, 0x23,
	0x25, 0x25, 0x27, 0x27, 0x29, 0x29, 0x2B, 0x2B, 0x2D, 0x2D, 0x2F, 0x2F, 0x16, 0x16, 0x18, 0x18, 0x1A, 0x1A, 0x20, 0x20, 0x1C, 0x1C,
	0x1E, 0x1E, 0x22, 0x22, 0x24, 0x24, 0x26, 0x26, 0x28, 0x28, 0x2A, 0x2A, 0x2C, 0x2C, 0x2E, 0x2E, 0x30, 0x30, 0x00, 0x00, 0x00};
	static unsigned char c32[]   = { 0xCB, 0x01, 0xF5, 0xFF, 0xFF, 0x01, 0x00, 0x05, 0x00, 0x9F, 0x00, 0x00, 0x00};
	static unsigned char c33[]   = { 0xD0, 0x06, 0x01};
	static unsigned char c34[]   = { 0xD3, 0x06, 0x01};
	static unsigned char c35[]   = { 0xD5, 0xA5, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	static unsigned char c36[]   = { 0x40,0x01, 0x5A
	, 0x77, 0x02, 0xF0, 0x13, 0x00, 0x00
	, 0x56, 0x10, 0x14, 0x18, 0x06, 0x10, 0x0C, 0x0F, 0x0F, 0x0F, 0x52, 0x34, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};                                                                                            

	static unsigned char c37[]   = { 0x40, 0xA5, 0x00, 0x80, 0x82, 0x85, 0x00
	, 0x35, 0x25, 0x0F, 0x0F, 0x83, 0x3C, 0x00, 0x00
	, 0x11, 0x11, 0x00, 0x00
	, 0x01, 0x01, 0x00, 0x0A, 0x00, 0x00
	, 0x10, 0x02, 0x10, 0x64, 0x00, 0x00};                                                                                                                                                                                            

	static unsigned char c38[]   = { 0x40, 0x40, 0x38, 0x38, 0x02, 0x14, 0x00, 0x00, 0x00
	, 0x04, 0x03, 0x12, 0x06, 0x06, 0x00, 0x00, 0x00};                                                                                                                                                                                

	static unsigned char c39[]   = { 0x40, 0x18, 0x18, 0x05, 0x00, 0x00, 0xD8, 0x8C, 0x00, 0x00, 0x42, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00
	, 0x10, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00, 0x0C};                                                                                                                                                                                

	static unsigned char c40[]   = { 0x40, 0x10, 0x12, 0x20, 0x32, 0x01, 0x04, 0x07, 0x09
	, 0xB4, 0x6E, 0x32, 0x00
	, 0x0F, 0x1C, 0xA0, 0x16
	, 0x00, 0x00, 0x04, 0x38, 0x07, 0x80};                                                                                                                                                                                            

	static unsigned char c41[]   = { 0x40, 0x03, 0x2F, 0x08, 0x5B, 0x56, 0x2D, 0x05, 0x00, 0x69, 0x02, 0x15, 0x4B, 0x6C, 0x05
	, 0x03, 0xCE, 0x09, 0xFD, 0x58, 0xCC, 0x00, 0x00, 0x7F, 0x02, 0x85, 0x4C, 0xC7, 0x00};                                                                                                                                            

	static unsigned char c42[]   = { 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};                                                                                                                                           

	static unsigned char c43_1[] = { 0x40, 0x00, 0xFF, 0x15, 0x28, 0x01, 0xFF, 0x16, 0x29, 0x02, 0xFF, 0x1B, 0x2A, 0x03, 0xFF, 0x1C, 0xFF, 0x04, 0xFF, 0x1D, 0xFF, 0x05, 0x0F, 0x1E, 0xFF, 0x06, 0x10, 0x1F, 0xFF, 0x07, 0x11, 0x20}; 
	static unsigned char c43_2[] = { 0x40, 0xFF, 0x08, 0x12, 0x21, 0xFF, 0x09, 0x13, 0x22, 0xFF, 0x0A, 0x14, 0x23, 0xFF, 0x0B, 0x17, 0x24, 0xFF, 0x0C, 0x18, 0x25, 0xFF, 0x0D, 0x19, 0x26, 0xFF, 0x0E, 0x1A, 0x27, 0xFF};             

	static unsigned char c44_1[] = { 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 
	static unsigned char c44_2[] = { 0x40, 0x00, 0x00, 0x00, 0x00, 0x00};                                                                                                                                                             
	static unsigned char c45[]   = { 0x40, 0x1D, 0x00};                                                                                                                                                                               

struct himax_ts_data {
	uint8_t x_channel;
	uint8_t y_channel;
	uint8_t diag_command;
	uint8_t usb_connected;
	uint16_t finger_pressed;
	uint8_t first_pressed;
	uint8_t just_resume;
	uint16_t last_slot;
	uint8_t protocol_type;
	uint16_t pre_finger_mask;
	uint8_t useScreenRes;
	uint8_t diag_self[50];
	uint8_t *cable_config;
	uint8_t *diag_mutual;
	uint32_t debug_log_level;
	uint32_t event_htc_enable_type;
	uint32_t irq;
	uint32_t widthFactor;
	uint32_t heightFactor;
	atomic_t in_flash;
	atomic_t suspend_mode;
	uint32_t suspend_flag_b4_flash;
	u8 *fw_data_start;
	uint32_t fw_size;
#ifdef FAKE_EVENT
	int fake_X_S;
	int fake_Y_S;
	int fake_X_E;
	int fake_Y_E;
#endif
	int use_irq;
	int vendor_fw_ver;
	int vendor_config_ver;
	int vendor_sensor_id;
	struct device *dev;
	const struct firmware *fw;
	struct workqueue_struct *himax_wq;
	struct work_struct work;
	struct input_dev *input_dev;
	struct input_dev *sr_input_dev;
	struct hrtimer timer;
	struct i2c_client *client;
	int (*power)(int on);
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
	struct workqueue_struct *himax_att_wq;
	struct delayed_work work_att;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	int pre_finger_data[10][2];
	struct himax_i2c_platform_data *pdata;
	struct himax_config_init_api i2c_api;
	struct himax_virtual_key *button;
	
	#ifdef HX_TP_SYS_FLASH_DUMP
	struct workqueue_struct 			*flash_wq;
	struct work_struct 					flash_work;
	#endif
	

	
	#ifdef HX_RST_PIN_FUNC
	int rst_gpio;
	#endif
	
	
	#ifdef ENABLE_CHIP_RESET_MACHINE
	int retry_time;
	struct delayed_work himax_chip_reset_work;
	#endif
	
	int intr_gpio;

	uint8_t coord_data_size;
	uint8_t area_data_size;
	uint8_t raw_data_frame_size;
	uint8_t raw_data_nframes;
	uint8_t nFinger_support;
	uint8_t irq_enabled;
	uint32_t tw_x_min;
	uint32_t tw_x_max;
	uint32_t tw_y_min;
	uint32_t tw_y_max;
	uint32_t pl_x_min;
	uint32_t pl_x_max;
	uint32_t pl_y_min;
	uint32_t pl_y_max;
	bool suspended;
};

static struct himax_ts_data *private_ts;

#define SWITCH_TO_HTC_EVENT_ONLY	1
#define INJECT_HTC_EVENT		2

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void himax_ts_early_suspend(struct early_suspend *h);
static void himax_ts_late_resume(struct early_suspend *h);
#endif

#if defined (CONFIG_UX500_SOC_DB8500)
extern int htc_charger_is_vbus_present(void);
extern int board_build_flag(void);
#endif

#if defined (CONFIG_TOUCH_GET_LCMID)
int htc_get_lcm_id(void);
#endif
#ifdef CONFIG_OF
static int himax_parse_config(struct himax_ts_data *ts, struct himax_i2c_platform_data_config_type28 *pdata);
#endif
extern unsigned int get_tamper_sf(void);
static irqreturn_t himax_ts_thread(int irq, void *ptr);
static int himax_loadSensorConfig(struct i2c_client *client, struct himax_i2c_platform_data *pdata,bool need_load_default);
static void doHWreset(void);
void himax_HW_reset(void);
int i2c_himax_write(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry);
int i2c_himax_read(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry);
int i2c_himax_master_write(struct i2c_client *client, uint8_t *data, uint8_t length, uint8_t toRetry);
int himax_ManualMode(int enter);
int himax_FlashMode(int enter);
static int himax_lock_flash(void);
static int himax_unlock_flash(void);
static uint8_t himax_calculateChecksum(char *ImageBuffer, int fullLength);
u8 himax_read_FW_ver(bool hw_reset);
int i2c_himax_write_command(struct i2c_client *client, uint8_t command, uint8_t toRetry);
int himax_hang_shaking(void);

static void do_nsleep(unsigned int msecs, struct hrtimer_sleeper *sleeper,
	int sigs)
{
	enum hrtimer_mode mode = HRTIMER_MODE_REL;
	int state = sigs ? TASK_INTERRUPTIBLE : TASK_UNINTERRUPTIBLE;

	hrtimer_init(&sleeper->timer, CLOCK_MONOTONIC, mode);
	sleeper->timer.node.expires = ktime_set(msecs / 1000,
						(msecs % 1000) * NSEC_PER_MSEC);
	hrtimer_init_sleeper(sleeper, current);

	do {
		set_current_state(state);
		hrtimer_start(&sleeper->timer, sleeper->timer.node.expires, mode);
		if (sleeper->task)
			schedule();
		hrtimer_cancel(&sleeper->timer);
		mode = HRTIMER_MODE_ABS;
	} while (sleeper->task && !(sigs && signal_pending(current)));
}

void hr_msleep(unsigned int msecs)
{
	struct hrtimer_sleeper sleeper;

	do_nsleep(msecs, &sleeper, 0);
}
static void himax_int_enable(int enable)
{
	if (enable == 1 && irq_enable_count == 0) {
		enable_irq(private_ts->client->irq);
		irq_enable_count++;
	} else if (enable == 0 && irq_enable_count == 1) {
		disable_irq_nosync(private_ts->client->irq);
		irq_enable_count--;
	}
	I("irq_enable_count = %d", irq_enable_count);
}

int himax_hang_shaking(void)    
{
	int ret, result;
	uint8_t hw_reset_check[1];
	uint8_t hw_reset_check_2[1];
	uint8_t buf0[2];

	memset(hw_reset_check, 0x00, sizeof(hw_reset_check));
	memset(hw_reset_check_2, 0x00, sizeof(hw_reset_check_2));

	
	buf0[0] = 0x92;
	if (IC_STATUS_CHECK == 0xAA) {
		buf0[1] = 0xAA;
		IC_STATUS_CHECK = 0x55;
	} else {
		buf0[1] = 0x55;
		IC_STATUS_CHECK = 0xAA;
	}

	ret = i2c_himax_master_write(private_ts->client, buf0, 2, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		E("[Himax]:write 0x92 failed line: %d \n",__LINE__);
		goto work_func_send_i2c_msg_fail;
	}
	hr_msleep(50); 

	buf0[0] = 0x92;
	buf0[1] = 0x00;
	ret = i2c_himax_master_write(private_ts->client, buf0, 2, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		E("[Himax]:write 0x92 failed line: %d \n",__LINE__);
		goto work_func_send_i2c_msg_fail;
	}
	hr_msleep(2);

	ret = i2c_himax_read(private_ts->client, 0xDA, hw_reset_check, 1, DEFAULT_RETRY_CNT);
	if (ret < 0) {
		E("[Himax]:i2c_himax_read 0xDA failed line: %d \n",__LINE__);
		goto work_func_send_i2c_msg_fail;
	}
	

	if ((IC_STATUS_CHECK != hw_reset_check[0])) {
		hr_msleep(2);
		ret = i2c_himax_read(private_ts->client, 0xDA, hw_reset_check_2, 1, DEFAULT_RETRY_CNT);
		if (ret < 0) {
			E("[Himax]:i2c_himax_read 0xDA failed line: %d \n",__LINE__);
			goto work_func_send_i2c_msg_fail;
		}
		

		if (hw_reset_check[0] == hw_reset_check_2[0]) {
			result = 1; 
		} else {
			result = 0; 
		}
	} else {
		result = 0; 
	}

	return result;

work_func_send_i2c_msg_fail:
	return 2;
}

int himax_ManualMode(int enter)
{
	uint8_t cmd[2];
	cmd[0] = enter;
	if ( i2c_himax_write(private_ts->client, 0x42 ,&cmd[0], 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	return 0;
}

int himax_FlashMode(int enter)
{
	uint8_t cmd[2];
	cmd[0] = enter;
	if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 1, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	return 0;
}

static int himax_lock_flash(void)
{
	uint8_t cmd[5];

	
	cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x06;
	if (i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 3, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x03;cmd[1] = 0x00;cmd[2] = 0x00;
	if (i2c_himax_write(private_ts->client, 0x44 ,&cmd[0], 3, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x00;cmd[1] = 0x00;cmd[2] = 0x7D;cmd[3] = 0x03;
	if (i2c_himax_write(private_ts->client, 0x45 ,&cmd[0], 4, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}

	if (i2c_himax_write_command(private_ts->client, 0x4A, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	hr_msleep(50);
	return 0;
	
}

static int himax_unlock_flash(void)
{
	uint8_t cmd[5];

	
	cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x06;
	if (i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 3, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x03;cmd[1] = 0x00;cmd[2] = 0x00;
	if (i2c_himax_write(private_ts->client, 0x44 ,&cmd[0], 3, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}

	cmd[0] = 0x00;cmd[1] = 0x00;cmd[2] = 0x3D;cmd[3] = 0x03;
	if (i2c_himax_write(private_ts->client, 0x45 ,&cmd[0], 4, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}

	if (i2c_himax_write_command(private_ts->client, 0x4A, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	hr_msleep(50);

	return 0;
	
}

static uint8_t himax_calculateChecksum(char *ImageBuffer, int fullLength)
{
	
	if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_SW) {
		u16 checksum = 0;
		uint8_t cmd[5], last_byte;
		int FileLength, i, readLen, k, lastLength;

		FileLength = fullLength - 2;
		memset(cmd, 0x00, sizeof(cmd));

		

		
		

		
		
		

		himax_FlashMode(1);

		FileLength = (FileLength + 3) / 4;
		for (i = 0; i < FileLength; i++) {
			last_byte = 0;
			readLen = 0;

			cmd[0] = i & 0x1F;
			if (cmd[0] == 0x1F || i == FileLength - 1)
			last_byte = 1;
			cmd[1] = (i >> 5) & 0x1F;cmd[2] = (i >> 10) & 0x1F;
			if (i2c_himax_write(private_ts->client, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			if (i2c_himax_write_command(private_ts->client, 0x46, DEFAULT_RETRY_CNT) < 0) {
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			if (i2c_himax_read(private_ts->client, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0) {
				E("%s: i2c access fail!\n", __func__);
				return -1;
			}

			if (i < (FileLength - 1)) {
				checksum += cmd[0] + cmd[1] + cmd[2] + cmd[3];
				if (i == 0)
				{
					E("%s: himax_marked cmd 0 to 3 (first 4 bytes): %d, %d, %d, %d\n", __func__, cmd[0], cmd[1], cmd[2], cmd[3]);
				}
			} else {
				E("%s: himax_marked cmd 0 to 3 (last 4 bytes): %d, %d, %d, %d\n", __func__, cmd[0], cmd[1], cmd[2], cmd[3]);
				E("%s: himax_marked, checksum (not last): %d\n", __func__, checksum);

				lastLength = (((fullLength - 2) % 4) > 0)?((fullLength - 2) % 4):4;

				for (k = 0; k < lastLength; k++)
				{
					checksum += cmd[k];
				}
				E("%s: himax_marked, checksum (final): %d\n", __func__, checksum);

				
				if (ImageBuffer[fullLength - 1] == (u8)(0xFF & (checksum >> 8)) && ImageBuffer[fullLength - 2] == (u8)(0xFF & checksum))
				{
					himax_FlashMode(0);
					return 1;
				}
				else 
				{
					himax_FlashMode(0);
					return 0;
				}
			}
		}
	} else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_HW) {
		u32 sw_checksum = 0;
		u32 hw_checksum = 0;
		uint8_t cmd[5], last_byte;
		int FileLength, i, readLen, k, lastLength;

		FileLength = fullLength;
		memset(cmd, 0x00, sizeof(cmd));

		

		
		

		
		
		

		himax_FlashMode(1);

		FileLength = (FileLength + 3) / 4;
		for (i = 0; i < FileLength; i++) {
			last_byte = 0;
			readLen = 0;

			cmd[0] = i & 0x1F;
			if (cmd[0] == 0x1F || i == FileLength - 1) {
				last_byte = 1;
			}
			cmd[1] = (i >> 5) & 0x1F;cmd[2] = (i >> 10) & 0x1F;
			if (i2c_himax_write(private_ts->client, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			if (i2c_himax_write_command(private_ts->client, 0x46, DEFAULT_RETRY_CNT) < 0) {
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			if (i2c_himax_read(private_ts->client, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0) {
				E("%s: i2c access fail!\n", __func__);
				return -1;
			}

			if (i < (FileLength - 1)) {
				sw_checksum += cmd[0] + cmd[1] + cmd[2] + cmd[3];
				if (i == 0)
				{
					E("%s: himax_marked cmd 0 to 3 (first 4 bytes): %d, %d, %d, %d\n", __func__, cmd[0], cmd[1], cmd[2], cmd[3]);
				}
			} else {
				E("%s: himax_marked cmd 0 to 3 (last 4 bytes): %d, %d, %d, %d\n", __func__, cmd[0], cmd[1], cmd[2], cmd[3]);
				E("%s: himax_marked, sw_checksum (not last): %d\n", __func__, sw_checksum);

				lastLength = ((fullLength % 4) > 0)?(fullLength % 4):4;

				for (k = 0; k < lastLength; k++)
				{
					sw_checksum += cmd[k];
				}
				E("%s: himax_marked, sw_checksum (final): %d\n", __func__, sw_checksum);

				
				cmd[0] = 0x01;
				if (i2c_himax_write(private_ts->client, 0xE5 ,&cmd[0], 1, DEFAULT_RETRY_CNT) < 0)
				{
					E("%s: i2c access fail!\n", __func__);
					return 0;
				}

				
				hr_msleep(30);

				
				if ( i2c_himax_read(private_ts->client, 0xAD, cmd, 4, DEFAULT_RETRY_CNT) < 0)
				{
					E("%s: i2c access fail!\n", __func__);
					return -1;
				}

				hw_checksum = cmd[0] + cmd[1]*0x100 + cmd[2]*0x10000 + cmd[3]*1000000;
				I("[Touch_FH] %s: himax_marked, sw_checksum (final): %d\n", __func__, sw_checksum);
				I("[Touch_FH] %s: himax_marked, hw_checkusm (final): %d\n", __func__, hw_checksum);

				
				if ( hw_checksum == sw_checksum )
				{
					himax_FlashMode(0);
					return 1;
				}
				else
				{
					himax_FlashMode(0);
					return 0;
				}
			}
		}
	} else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_CRC) {
		uint8_t cmd[5];

		
		if (i2c_himax_read(private_ts->client, 0x7F, cmd, 5, DEFAULT_RETRY_CNT) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return -1;
		}
		cmd[3] = 0x02;

		if (i2c_himax_write(private_ts->client, 0x7F ,&cmd[0], 5, DEFAULT_RETRY_CNT) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}

		
		himax_FlashMode(1);

		
		cmd[0] = 0x05;
		cmd[1] = 0x00;
		cmd[2] = 0x00;
		if (i2c_himax_write(private_ts->client, 0xD2 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}

		
		cmd[0] = 0x01;
		if (i2c_himax_write(private_ts->client, 0xE5 ,&cmd[0], 1, DEFAULT_RETRY_CNT) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}

		
		hr_msleep(30);

		
		if (i2c_himax_read(private_ts->client, 0xAD, cmd, 4, DEFAULT_RETRY_CNT) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return -1;
		}

		if (cmd[0] == 0 && cmd[1] == 0 && cmd[2] == 0 && cmd[3] == 0 ) {
			himax_FlashMode(0);
			return 1;
		} else {
			himax_FlashMode(0);
			return 0;
		}
	}
	return 0;
}

u8 himax_read_FW_ver(bool hw_reset)
{
	u16 fw_ver_maj_start_addr;
	u16 fw_ver_maj_end_addr;
	u16 fw_ver_maj_addr;
	u16 fw_ver_maj_length;

	u16 fw_ver_min_start_addr;
	u16 fw_ver_min_end_addr;
	u16 fw_ver_min_addr;
	u16 fw_ver_min_length;

	u16 cfg_ver_maj_start_addr;
	u16 cfg_ver_maj_end_addr;
	u16 cfg_ver_maj_addr;
	u16 cfg_ver_maj_length;

	u16 cfg_ver_min_start_addr;
	u16 cfg_ver_min_end_addr;
	u16 cfg_ver_min_addr;
	u16 cfg_ver_min_length;

	uint8_t cmd[3];
	u16 i = 0;
	u16 j = 0;
	u16 k = 0;

	fw_ver_maj_start_addr  = FW_VER_MAJ_FLASH_ADDR / 4;                              
	fw_ver_maj_length      = FW_VER_MAJ_FLASH_LENG;                                  
	fw_ver_maj_end_addr    = (FW_VER_MAJ_FLASH_ADDR + fw_ver_maj_length ) / 4 + 1;   
	fw_ver_maj_addr        = FW_VER_MAJ_FLASH_ADDR % 4;                              

	fw_ver_min_start_addr  = FW_VER_MIN_FLASH_ADDR / 4;                              
	fw_ver_min_length      = FW_VER_MIN_FLASH_LENG;                                  
	fw_ver_min_end_addr    = (FW_VER_MIN_FLASH_ADDR + fw_ver_min_length ) / 4 + 1;   
	fw_ver_min_addr        = FW_VER_MIN_FLASH_ADDR % 4;                              

	cfg_ver_maj_start_addr = CFG_VER_MAJ_FLASH_ADDR / 4;                             
	cfg_ver_maj_length     = CFG_VER_MAJ_FLASH_LENG;                                 
	cfg_ver_maj_end_addr   = (CFG_VER_MAJ_FLASH_ADDR + cfg_ver_maj_length ) / 4 + 1; 
	cfg_ver_maj_addr       = CFG_VER_MAJ_FLASH_ADDR % 4;                             

	cfg_ver_min_start_addr = CFG_VER_MIN_FLASH_ADDR / 4;                             
	cfg_ver_min_length     = CFG_VER_MIN_FLASH_LENG;                                 
	cfg_ver_min_end_addr   = (CFG_VER_MIN_FLASH_ADDR + cfg_ver_min_length ) / 4 + 1; 
	cfg_ver_min_addr       = CFG_VER_MIN_FLASH_ADDR % 4;                             

	himax_int_enable(0);

	#ifdef HX_RST_PIN_FUNC
	if (hw_reset) {
		himax_HW_reset();
	}
	#endif

	
	if (i2c_himax_write(private_ts->client, 0x81 ,&cmd[0], 0, 3) < 0) {
		E("%s: i2c access fail!\n", __func__);
		return 0;
	}
	hr_msleep(120);

	
	himax_FlashMode(1);

	
	
	i = fw_ver_maj_start_addr;
	do {
		cmd[0] = i & 0x1F;					
		cmd[1] = (i >> 5) & 0x1F;		
		cmd[2] = (i >> 10) & 0x1F;	

		if (i2c_himax_write(private_ts->client, 0x44 ,&cmd[0], 3, 3) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}
		if (i2c_himax_write(private_ts->client, 0x46 ,&cmd[0], 0, 3) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}
		if (i2c_himax_read(private_ts->client, 0x59, cmd, 4, 3) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}

		if (i == fw_ver_maj_start_addr) {
			j = 0;
			for( k = fw_ver_maj_addr; k < 4 && j < fw_ver_maj_length; k++) {
				FW_VER_MAJ_buff[j++] = cmd[k];
			}
		} else {
			for( k = 0; k < 4 && j < fw_ver_maj_length; k++) {
				FW_VER_MAJ_buff[j++] = cmd[k];
			}
		}
		i++;
	} while (i < fw_ver_maj_end_addr);

	
	i = fw_ver_min_start_addr;
	do {
		cmd[0] = i & 0x1F;					
		cmd[1] = (i >> 5) & 0x1F;		
		cmd[2] = (i >> 10) & 0x1F;	

		if (i2c_himax_write(private_ts->client, 0x44 ,&cmd[0], 3, 3) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}
		if (i2c_himax_write(private_ts->client, 0x46 ,&cmd[0], 0, 3) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}
		if (i2c_himax_read(private_ts->client, 0x59, cmd, 4, 3) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}

		if (i == fw_ver_min_start_addr) {
			j = 0;
			for(k = fw_ver_min_addr; k < 4 && j < fw_ver_min_length; k++) {
				FW_VER_MIN_buff[j++] = cmd[k];
			}
		} else {
			for(k = 0; k < 4 && j < fw_ver_min_length; k++) {
				FW_VER_MIN_buff[j++] = cmd[k];
			}
		}
		i++;
	} while(i < fw_ver_min_end_addr);


	
	i = cfg_ver_maj_start_addr;
	do {
		cmd[0] = i & 0x1F;					
		cmd[1] = (i >> 5) & 0x1F;		
		cmd[2] = (i >> 10) & 0x1F;	

		if (i2c_himax_write(private_ts->client, 0x44 ,&cmd[0], 3, 3) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}
		if (i2c_himax_write(private_ts->client, 0x46 ,&cmd[0], 0, 3) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}
		if (i2c_himax_read(private_ts->client, 0x59, cmd, 4, 3) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}

		if (i == cfg_ver_maj_start_addr) {
			j = 0;
			for( k = cfg_ver_maj_addr; k < 4 && j < cfg_ver_maj_length; k++) {
				CFG_VER_MAJ_buff[j++] = cmd[k];
			}
		} else {
			for(k = 0; k < 4 && j < cfg_ver_maj_length; k++) {
				CFG_VER_MAJ_buff[j++] = cmd[k];
			}
		}
		i++;
	} while (i < cfg_ver_maj_end_addr);

	
	i = cfg_ver_min_start_addr;
	do {
		cmd[0] = i & 0x1F;					
		cmd[1] = (i >> 5) & 0x1F;		
		cmd[2] = (i >> 10) & 0x1F;	

		if (i2c_himax_write(private_ts->client, 0x44 ,&cmd[0], 3, 3) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}
		if (i2c_himax_write(private_ts->client, 0x46 ,&cmd[0], 0, 3) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}
		if (i2c_himax_read(private_ts->client, 0x59, cmd, 4, 3) < 0) {
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}

		if (i == cfg_ver_min_start_addr) {
			j = 0;
			for (k = cfg_ver_min_addr; k < 4 && j < cfg_ver_min_length; k++) {
				CFG_VER_MIN_buff[j++] = cmd[k];
			}
		} else {
			for (k = 0; k < 4 && j < cfg_ver_min_length; k++) {
				CFG_VER_MIN_buff[j++] = cmd[k];
			}
		}
		i++;
	} while (i < cfg_ver_min_end_addr);

	
	himax_FlashMode(0);


	I("FW_VER_MAJ_buff : %d \n",FW_VER_MAJ_buff[0]);
	I("FW_VER_MIN_buff : %d \n",FW_VER_MIN_buff[0]);

	I("CFG_VER_MAJ_buff : ");
	for (i=0; i<12; i++)
		I("%d ,",CFG_VER_MAJ_buff[i]);
	I("\n");

	I("CFG_VER_MIN_buff : ");
	for(i=0; i<12; i++)
		I("%d ,",CFG_VER_MIN_buff[i]);
	I("\n");

	
		#ifdef HX_RST_PIN_FUNC
		himax_HW_reset();
		himax_loadSensorConfig(private_ts->client,private_ts->pdata,true);
		#endif
	

	himax_int_enable(1);
	return 0;
}


	#ifdef HX_TP_SYS_REGISTER
	static uint8_t register_command 			= 0;
	static uint8_t multi_register_command = 0;
	static uint8_t multi_register[8] 			= {0x00};
	static uint8_t multi_cfg_bank[8] 			= {0x00};
	static uint8_t multi_value[1024] 			= {0x00};
	static bool 	config_bank_reg 				= false;
	#endif

	#ifdef HX_ESD_WORKAROUND
	static u8 		ESD_RESET_ACTIVATE 	= 1;
	static u8 		ESD_COUNTER 		= 0;
	
	static u8		ESD_R36_FAIL		= 0;

	void ESD_HW_REST(void);
	#endif

	#ifdef HX_ESD_WORKAROUND
	void ESD_HW_REST(void)
	{
		ESD_RESET_ACTIVATE = 1;
		ESD_COUNTER = 0;
		ESD_R36_FAIL = 0;

		I("START_Himax TP: ESD - Reset\n");

		while(ESD_R36_FAIL <=3 )
		{
		#ifdef HX_RST_BY_POWER
			
			hr_msleep(100);

			
			hr_msleep(100);

		#else
			gpio_set_value(private_ts->rst_gpio, 0);
			hr_msleep(30);
			gpio_set_value(private_ts->rst_gpio, 1);
			hr_msleep(30);
		#endif
			if(himax_loadSensorConfig(private_ts->client, private_ts->pdata,false)<0)
				ESD_R36_FAIL++;
			else
				break;
		}
		I("END_Himax TP: ESD - Reset\n");
	}
	#endif
#ifdef ENABLE_CHIP_RESET_MACHINE
	static void himax_chip_reset_function(struct work_struct *dat)
	{
		printk("[Himax]:himax_chip_reset_function ++ \n");

		if(private_ts->retry_time <= 10)
		{

		#ifdef HX_ESD_WORKAROUND
			ESD_RESET_ACTIVATE = 1;
		#endif

		#ifdef HX_RST_BY_POWER
				
				hr_msleep(100);

				
				hr_msleep(100);
		#else
				gpio_set_value(private_ts->rst_gpio, 0);
				hr_msleep(30);
				gpio_set_value(private_ts->rst_gpio, 1);
				hr_msleep(30);
		#endif

			if(gpio_get_value(private_ts->intr_gpio) == 0)
			{
				printk("[Himax]%s: IRQ = 0, Enable IRQ\n", __func__);
				himax_int_enable(1);
			}
		}
		private_ts->retry_time ++;
		printk("[Himax]:himax_chip_reset_function retry_time =%d --\n",private_ts->retry_time);
	}
#endif
	


	#ifdef HX_TP_SYS_DIAG
	static uint8_t x_channel 		= 0;
	static uint8_t y_channel 		= 0;
	static uint8_t *diag_mutual = NULL;
	static int diag_command = 0;
	static uint8_t diag_coor[128];

	static uint8_t diag_self[100] = {0};

	static uint8_t *getMutualBuffer(void);
	static uint8_t *getSelfBuffer(void);
	static uint8_t 	getDiagCommand(void);
	static uint8_t 	getXChannel(void);
	static uint8_t 	getYChannel(void);

	static void 	setMutualBuffer(void);
	static void 	setXChannel(uint8_t x);
	static void 	setYChannel(uint8_t y);

	static uint8_t	coordinate_dump_enable = 0;
	struct file	*coordinate_fn;
	#endif

	#ifdef HX_TP_SYS_DEBUG
	static bool	fw_update_complete = false;
	static bool irq_enable = false;
	static int handshaking_result = 0;
	static unsigned char debug_level_cmd = 0;
	static unsigned char upgrade_fw[32*1024];
	#endif

	#ifdef HX_TP_SYS_FLASH_DUMP
	static uint8_t *flash_buffer 				= NULL;
	static uint8_t flash_command 				= 0;
	static uint8_t flash_read_step 			= 0;
	static uint8_t flash_progress 			= 0;
	static uint8_t flash_dump_complete	= 0;
	static uint8_t flash_dump_fail 			= 0;
	static uint8_t sys_operation				= 0;
	static uint8_t flash_dump_sector	 	= 0;
	static uint8_t flash_dump_page 			= 0;
	static bool    flash_dump_going			= false;

	static uint8_t getFlashCommand(void);
	static uint8_t getFlashDumpComplete(void);
	static uint8_t getFlashDumpFail(void);
	static uint8_t getFlashDumpProgress(void);
	static uint8_t getFlashReadStep(void);
	static uint8_t getSysOperation(void);
	static uint8_t getFlashDumpSector(void);
	static uint8_t getFlashDumpPage(void);
	static bool	   getFlashDumpGoing(void);

	static void setFlashBuffer(void);
	static void setFlashCommand(uint8_t command);
	static void setFlashReadStep(uint8_t step);
	static void setFlashDumpComplete(uint8_t complete);
	static void setFlashDumpFail(uint8_t fail);
	static void setFlashDumpProgress(uint8_t progress);
	static void setSysOperation(uint8_t operation);
	static void setFlashDumpSector(uint8_t sector);
	static void setFlashDumpPage(uint8_t page);
	static void setFlashDumpGoing(bool going);
	#endif

	#ifdef HX_TP_SYS_SELF_TEST
	static ssize_t himax_chip_self_test_function(struct device *dev, struct device_attribute *attr, char *buf);
	static int himax_chip_self_test(void);
	#endif

#ifdef HX_RST_PIN_FUNC
void himax_HW_reset(void)
{
	#ifdef HX_ESD_WORKAROUND
	ESD_RESET_ACTIVATE = 1;
	#endif

	#ifdef HX_RST_BY_POWER
		
		hr_msleep(100);

		
		hr_msleep(100);

	#else
		gpio_set_value(private_ts->rst_gpio, 0);
		hr_msleep(30);
		gpio_set_value(private_ts->rst_gpio, 1);
		hr_msleep(30);
	#endif
}
#endif

static void calcDataSize(uint8_t finger_num)
{
	struct himax_ts_data *ts_data = private_ts;
	ts_data->coord_data_size = 4 * finger_num;
	ts_data->area_data_size = ((finger_num / 4) + (finger_num % 4 ? 1 : 0)) * 4;
	ts_data->raw_data_frame_size = 128 - ts_data->coord_data_size - ts_data->area_data_size - 4 - 4 - 1;
	ts_data->raw_data_nframes  = ((uint32_t)ts_data->x_channel * ts_data->y_channel +
																					ts_data->x_channel + ts_data->y_channel) / ts_data->raw_data_frame_size +
															(((uint32_t)ts_data->x_channel * ts_data->y_channel +
		  																		ts_data->x_channel + ts_data->y_channel) % ts_data->raw_data_frame_size)? 1 : 0;
	I("%s: coord_data_size: %d, area_data_size:%d, raw_data_frame_size:%d, raw_data_nframes:%d", __func__, ts_data->coord_data_size, ts_data->area_data_size, ts_data->raw_data_frame_size, ts_data->raw_data_nframes);
}

int i2c_himax_read(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;
		hr_msleep(10);
	}
	if (retry == toRetry) {
		E("%s: i2c_read_block retry over %d\n",
			__func__, toRetry);
		return -EIO;
	}
	return 0;

}


int i2c_himax_write(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry, loop_i;
	uint8_t buf[length + 1];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	buf[0] = command;
	for (loop_i = 0; loop_i < length; loop_i++)
		buf[loop_i + 1] = data[loop_i];

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		hr_msleep(10);
	}

	if (retry == toRetry) {
		E("%s: i2c_write_block retry over %d\n",
			__func__, toRetry);
		return -EIO;
	}
	return 0;

}

int i2c_himax_write_command(struct i2c_client *client, uint8_t command, uint8_t toRetry)
{
	return i2c_himax_write(client, command, NULL, 0, toRetry);
}

int i2c_himax_master_write(struct i2c_client *client, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry, loop_i;
	uint8_t buf[length];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length,
			.buf = buf,
		}
	};

	for (loop_i = 0; loop_i < length; loop_i++)
		buf[loop_i] = data[loop_i];

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		hr_msleep(10);
	}

	if (retry == toRetry) {
		E("%s: i2c_write_block retry over %d\n",
		       __func__, toRetry);
		return -EIO;
	}
	return 0;
}

int i2c_himax_read_command(struct i2c_client *client, uint8_t length, uint8_t *data, uint8_t *readlength, uint8_t toRetry)
{
	int retry;
	struct i2c_msg msg[] = {
		{
		.addr = client->addr,
		.flags = I2C_M_RD,
		.len = length,
		.buf = data,
		}
	};

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		hr_msleep(10);
	}
	if (retry == toRetry) {
		E("%s: i2c_read_block retry over %d\n",
		       __func__, toRetry);
		return -EIO;
	}
	return 0;
}

#define CC(pa) do { \
	i2c_himax_master_write(client, pa , sizeof(pa), normalRetry); \
	for (i = 0; i < sizeof(pa); i++) { \
		myCheckSum += pa[i]; \
	} \
} while (0)


static int himax_loaddefaultSensorConfig(struct i2c_client *client)
{
	char data[4];
	const int normalRetry = 10;

	
	data[0] = 0x42; data[1] = 0x02;
	i2c_himax_master_write(client, &data[0],2,normalRetry);
	hr_msleep(1);

	data[0] = 0x36;	data[1] = 0x0F; data[2] = 0x53;
	i2c_himax_master_write(client, &data[0],3,normalRetry);
	hr_msleep(1);

	data[0] = 0xDD; data[1] = 0x04; data[2] = 0x03;
	i2c_himax_master_write(client, &data[0],3,normalRetry);
	hr_msleep(1);

	data[0] = 0xB9;	data[1] = 0x01; data[2] = 0x36;
	i2c_himax_master_write(client, &data[0],3,normalRetry);
	hr_msleep(1);

	data[0] = 0xCB; data[1] = 0x01; data[2] = 0xF5;
	i2c_himax_master_write(client, &data[0],3,normalRetry);
	hr_msleep(1);

	
	i2c_himax_master_write(client, c1,sizeof(c1),normalRetry);
	i2c_himax_master_write(client, c2,sizeof(c2),normalRetry);
	i2c_himax_master_write(client, c3,sizeof(c3),normalRetry);
	i2c_himax_master_write(client, c4,sizeof(c4),normalRetry);
	i2c_himax_master_write(client, c5,sizeof(c5),normalRetry);
	i2c_himax_master_write(client, c6,sizeof(c6),normalRetry);
	i2c_himax_master_write(client, c7,sizeof(c7),normalRetry);
	i2c_himax_master_write(client, c8,sizeof(c8),normalRetry);
	i2c_himax_master_write(client, c9,sizeof(c9),normalRetry);
	i2c_himax_master_write(client, c10,sizeof(c10),normalRetry);
	i2c_himax_master_write(client, c11,sizeof(c11),normalRetry);
	i2c_himax_master_write(client, c12,sizeof(c12),normalRetry);
	i2c_himax_master_write(client, c13,sizeof(c13),normalRetry);
	i2c_himax_master_write(client, c14,sizeof(c14),normalRetry);
	i2c_himax_master_write(client, c15,sizeof(c15),normalRetry);
	i2c_himax_master_write(client, c16,sizeof(c16),normalRetry);
	i2c_himax_master_write(client, c17,sizeof(c17),normalRetry);
	i2c_himax_master_write(client, c18,sizeof(c18),normalRetry);
	i2c_himax_master_write(client, c19,sizeof(c19),normalRetry);
	i2c_himax_master_write(client, c20,sizeof(c20),normalRetry);
	i2c_himax_master_write(client, c21,sizeof(c21),normalRetry);
	i2c_himax_master_write(client, c22,sizeof(c22),normalRetry);
	i2c_himax_master_write(client, c23,sizeof(c23),normalRetry);
	i2c_himax_master_write(client, c24,sizeof(c24),normalRetry);
	i2c_himax_master_write(client, c25,sizeof(c25),normalRetry);
	i2c_himax_master_write(client, c26,sizeof(c26),normalRetry);
	i2c_himax_master_write(client, c27,sizeof(c27),normalRetry);
	i2c_himax_master_write(client, c28,sizeof(c28),normalRetry);
	i2c_himax_master_write(client, c29,sizeof(c29),normalRetry);
	i2c_himax_master_write(client, c30,sizeof(c30),normalRetry);
	i2c_himax_master_write(client, c31,sizeof(c31),normalRetry);
	i2c_himax_master_write(client, c32,sizeof(c32),normalRetry);
	i2c_himax_master_write(client, c33,sizeof(c33),normalRetry);
	i2c_himax_master_write(client, c34,sizeof(c34),normalRetry);
	i2c_himax_master_write(client, c35,sizeof(c35),normalRetry);

	
	
	data[0] = 0xE1;data[1] = 0x15;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xD8;data[1] = 0x00;data[2] = 0x00;	
	if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
		goto HimaxErr;

	if ((i2c_himax_master_write(client, c36,sizeof(c36),normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xE1;data[1] = 0x00;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	
	data[0] = 0xE1;data[1] = 0x15;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xD8;data[1] = 0x00;data[2] = 0x1E;	
	if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
		goto HimaxErr;

	if ((i2c_himax_master_write(client, c37,sizeof(c37),normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xE1;data[1] = 0x00;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	
	data[0] = 0xE1;data[1] = 0x15;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xD8;data[1] = 0x00;data[2] = 0x3C;	
	if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
		goto HimaxErr;

	if ((i2c_himax_master_write(client, c38,sizeof(c38),normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xE1;data[1] = 0x00;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;
	
	data[0] = 0xE1;data[1] = 0x15;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xD8;data[1] = 0x00;data[2] = 0x4C;	
	if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
		goto HimaxErr;

	if ((i2c_himax_master_write(client, c39,sizeof(c39),normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xE1;data[1] = 0x00;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	
	data[0] = 0xE1;data[1] = 0x15;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xD8;data[1] = 0x00;data[2] = 0x64;	
	if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
		goto HimaxErr;

	if ((i2c_himax_master_write(client, c40,sizeof(c40),normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xE1;data[1] = 0x00;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	
	data[0] = 0xE1;data[1] = 0x15;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xD8;data[1] = 0x00;data[2] = 0x7A;	
	if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
		goto HimaxErr;

	if ((i2c_himax_master_write(client, c41,sizeof(c41),normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xE1;data[1] = 0x00;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	
	data[0] = 0xE1;data[1] = 0x15;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xD8;data[1] = 0x00;data[2] = 0x96;	
	if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
		goto HimaxErr;

	if ((i2c_himax_master_write(client, c42,sizeof(c42),normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xE1;data[1] = 0x00;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	
	data[0] = 0xE1;data[1] = 0x15;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xD8;data[1] = 0x00;data[2] = 0x9E;	
	if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
		goto HimaxErr;

	if ((i2c_himax_master_write(client, c43_1,sizeof(c43_1),normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xE1;data[1] = 0x00;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	
	data[0] = 0xE1;data[1] = 0x15;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xD8;data[1] = 0x00;data[2] = 0xBD;	
	if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
		goto HimaxErr;

	if ((i2c_himax_master_write(client, c43_2,sizeof(c43_2),normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xE1;data[1] = 0x00;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	
	data[0] = 0xE1;data[1] = 0x15;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xD8;data[1] = 0x00;data[2] = 0xDA;	
	if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
		goto HimaxErr;

	if ((i2c_himax_master_write(client, c44_1,sizeof(c44_1),normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xE1;data[1] = 0x00;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	
	data[0] = 0xE1;data[1] = 0x15;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xD8;data[1] = 0x00;data[2] = 0xF9;	
	if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
		goto HimaxErr;

	if ((i2c_himax_master_write(client, c44_2,sizeof(c44_2),normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xE1;data[1] = 0x00;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	
	data[0] = 0xE1;data[1] = 0x15;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xD8;data[1] = 0x00;data[2] = 0xFE;	
	if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
		goto HimaxErr;

	if ((i2c_himax_master_write(client, c45,sizeof(c45),normalRetry))<0)
		goto HimaxErr;

	data[0] = 0xE1;data[1] = 0x00;
	if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
		goto HimaxErr;

	hr_msleep(1);

	
	i2c_himax_write_command(client, 0x83, normalRetry);
	hr_msleep(30);

	i2c_himax_write_command(client, 0x81, normalRetry);
	hr_msleep(50);

HimaxErr:
	return 0;
}

static int himax_loadSensorConfig(struct i2c_client *client, struct himax_i2c_platform_data *pdata,bool need_load_default)
{
#ifndef CONFIG_OF
	int i = 0;
#endif
	int result = 1;
	char data[12] = {0};
	int  rc=0, fw_ver_info_retry = 0;
	
	const int normalRetry = 10;	

	if (!client) {
		E("%s: Necessary parameters client are null!\n", __func__);
		return -1;
	}
	#ifdef CONFIG_OF
	if(config_load == false)
		{
			type28_selected = kzalloc(sizeof(*type28_selected), GFP_KERNEL);
			if (type28_selected == NULL) {
				E("%s: alloc type28_selected fail!\n", __func__);
				return -1;
			}
		}
	#else
	pdata = client->dev.platform_data;
		if (!pdata) {
			E("%s: Necessary parameters pdata are null!\n", __func__);
			return -1;
		}

	#endif
	if (fw_ver_info_retry > 2) {
		E("%s: Try to read firmware"
			" version fail over %d times, stop loading!\n", __func__, fw_ver_info_retry);
		return -1;
	}

	#if defined(CONFIG_TOUCH_GET_LCMID)
	lcm_id=htc_get_lcm_id();
	#endif

	
#ifdef HX_LOADIN_CONFIG
		
		if(need_load_default)
		{
			himax_loaddefaultSensorConfig(client);

			
			i2c_himax_write_command(client, 0x82, normalRetry);
			hr_msleep(120);

			
			i2c_himax_read(client, 0x32, data, 2, normalRetry);
			private_ts->vendor_fw_ver = data[0];

			
			i2c_himax_read(client, 0x95, data, 2, normalRetry);
			private_ts->vendor_sensor_id = data[0];
			
			#ifdef HX_RST_PIN_FUNC
			himax_HW_reset();
			#endif
		}
		I("sensor_id=%x.\n",private_ts->vendor_sensor_id);
		I("fw_ver=%x.\n",private_ts->vendor_fw_ver);
#ifdef CONFIG_OF
		I("%s, type28_selected, %X\n", __func__ ,(uint32_t)type28_selected);
		if(config_load == false)
			{
				rc = himax_parse_config(private_ts, type28_selected);
				if (rc < 0) {
					E(" DT:cfg table parser FAIL. ret=%d\n", rc);
					return rc;
				} else if (rc == 0){
					if ((private_ts->tw_x_max)&&(private_ts->tw_y_max))
						{
							pdata->abs_x_min = private_ts->tw_x_min;
							pdata->abs_x_max = private_ts->tw_x_max;
							pdata->abs_y_min = private_ts->tw_y_min;
							pdata->abs_y_max = private_ts->tw_y_max;
							I(" DT-%s:config-panel-coords = %d, %d, %d, %d\n", __func__, pdata->abs_x_min,
							pdata->abs_x_max, pdata->abs_y_min, pdata->abs_y_max);
						}
					if ((private_ts->pl_x_max)&&(private_ts->pl_y_max))
						{
							pdata->screenWidth = private_ts->pl_x_max;
							pdata->screenHeight= private_ts->pl_y_max;
							I(" DT-%s:config-display-coords = (%d, %d)", __func__, pdata->screenWidth,
							pdata->screenHeight);
						}
					config_load = true;
					I(" DT parser Done\n");
					}
			}
#else
		I("pdata->type28_size=%x.\n",(pdata->type28_size+1));
		I("config_type28_size=%x.\n",sizeof(struct himax_i2c_platform_data_config_type28));

		if (pdata->type28)
		{
			for (i = 0; i < pdata->type28_size/sizeof(struct himax_i2c_platform_data_config_type28); ++i) {
			I("(pdata->type28)[%x].version=%x.\n",i,(pdata->type28)[i].version);
			I("(pdata->type28)[%x].tw_id=%x.\n",i,(pdata->type28)[i].tw_id);

				if (private_ts->vendor_fw_ver < (pdata->type28)[i].version) {
					continue;
				}else{
					if ((private_ts->vendor_sensor_id == (pdata->type28)[i].tw_id)) {
						type28_selected = &((pdata->type28)[i]);
						I("type28 selected, %X\n", (uint32_t)type28_selected);
						break;
					}
					else if ((pdata->type28)[i].common) {
						I("common configuration detected.\n");
						type28_selected = &((pdata->type28)[i]);
						I("type28 selected, %X\n", (uint32_t)type28_selected);
						break;
					}
				}
			}
		}
		else
		{
			E("[HimaxError] %s pdata->type28 is not exist \n",__func__);
			goto HimaxErr;
		}
#endif
#endif

	
	
	data[0] = 0x42; data[1] = 0x02;
	i2c_himax_master_write(client, &data[0],2,normalRetry);
	hr_msleep(1);

	data[0] = 0x36;	data[1] = 0x0F; data[2] = 0x53;
	i2c_himax_master_write(client, &data[0],3,normalRetry);
	hr_msleep(1);

	data[0] = 0xDD; data[1] = 0x04; data[2] = 0x03;
	i2c_himax_master_write(client, &data[0],3,normalRetry);
	hr_msleep(1);

	data[0] = 0xB9;	data[1] = 0x01; data[2] = 0x36;
	i2c_himax_master_write(client, &data[0],3,normalRetry);
	hr_msleep(1);

	data[0] = 0xCB; data[1] = 0x01; data[2] = 0xF5;
	i2c_himax_master_write(client, &data[0],3,normalRetry);
	hr_msleep(1);

	
	#ifdef HX_LOADIN_CONFIG
		if (type28_selected) {
			
			private_ts->vendor_config_ver = type28_selected->c36[1];

			
			i2c_himax_master_write(client, type28_selected->c1,sizeof(type28_selected->c1),normalRetry);
			i2c_himax_master_write(client, type28_selected->c2,sizeof(type28_selected->c2),normalRetry);
			i2c_himax_master_write(client, type28_selected->c3,sizeof(type28_selected->c3),normalRetry);
			i2c_himax_master_write(client, type28_selected->c4,sizeof(type28_selected->c4),normalRetry);
			i2c_himax_master_write(client, type28_selected->c5,sizeof(type28_selected->c5),normalRetry);
			i2c_himax_master_write(client, type28_selected->c6,sizeof(type28_selected->c6),normalRetry);
			i2c_himax_master_write(client, type28_selected->c7,sizeof(type28_selected->c7),normalRetry);
			i2c_himax_master_write(client, type28_selected->c8,sizeof(type28_selected->c8),normalRetry);
			i2c_himax_master_write(client, type28_selected->c9,sizeof(type28_selected->c9),normalRetry);
			i2c_himax_master_write(client, type28_selected->c10,sizeof(type28_selected->c10),normalRetry);
			i2c_himax_master_write(client, type28_selected->c11,sizeof(type28_selected->c11),normalRetry);
			i2c_himax_master_write(client, type28_selected->c12,sizeof(type28_selected->c12),normalRetry);
			i2c_himax_master_write(client, type28_selected->c13,sizeof(type28_selected->c13),normalRetry);
			i2c_himax_master_write(client, type28_selected->c14,sizeof(type28_selected->c14),normalRetry);
			i2c_himax_master_write(client, type28_selected->c15,sizeof(type28_selected->c15),normalRetry);
			i2c_himax_master_write(client, type28_selected->c16,sizeof(type28_selected->c16),normalRetry);
			i2c_himax_master_write(client, type28_selected->c17,sizeof(type28_selected->c17),normalRetry);
			i2c_himax_master_write(client, type28_selected->c18,sizeof(type28_selected->c18),normalRetry);
			i2c_himax_master_write(client, type28_selected->c19,sizeof(type28_selected->c19),normalRetry);
			i2c_himax_master_write(client, type28_selected->c20,sizeof(type28_selected->c20),normalRetry);
			i2c_himax_master_write(client, type28_selected->c21,sizeof(type28_selected->c21),normalRetry);
			i2c_himax_master_write(client, type28_selected->c22,sizeof(type28_selected->c22),normalRetry);
			i2c_himax_master_write(client, type28_selected->c23,sizeof(type28_selected->c23),normalRetry);
			i2c_himax_master_write(client, type28_selected->c24,sizeof(type28_selected->c24),normalRetry);
			i2c_himax_master_write(client, type28_selected->c25,sizeof(type28_selected->c25),normalRetry);
			i2c_himax_master_write(client, type28_selected->c26,sizeof(type28_selected->c26),normalRetry);
			i2c_himax_master_write(client, type28_selected->c27,sizeof(type28_selected->c27),normalRetry);
			i2c_himax_master_write(client, type28_selected->c28,sizeof(type28_selected->c28),normalRetry);
			i2c_himax_master_write(client, type28_selected->c29,sizeof(type28_selected->c29),normalRetry);
			i2c_himax_master_write(client, type28_selected->c30,sizeof(type28_selected->c30),normalRetry);
			i2c_himax_master_write(client, type28_selected->c31,sizeof(type28_selected->c31),normalRetry);
			i2c_himax_master_write(client, type28_selected->c32,sizeof(type28_selected->c32),normalRetry);
			i2c_himax_master_write(client, type28_selected->c33,sizeof(type28_selected->c33),normalRetry);
			i2c_himax_master_write(client, type28_selected->c34,sizeof(type28_selected->c34),normalRetry);
			i2c_himax_master_write(client, type28_selected->c35,sizeof(type28_selected->c35),normalRetry);

			
			
			data[0] = 0xE1;data[1] = 0x15;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xD8;data[1] = 0x00;data[2] = 0x00;	
			if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
				goto HimaxErr;

			if ((i2c_himax_master_write(client, type28_selected->c36,sizeof(type28_selected->c36),normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xE1;data[1] = 0x00;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			
			data[0] = 0xE1;data[1] = 0x15;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xD8;data[1] = 0x00;data[2] = 0x1E;	
			if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
				goto HimaxErr;

			if ((i2c_himax_master_write(client, type28_selected->c37,sizeof(type28_selected->c37),normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xE1;data[1] = 0x00;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			
			data[0] = 0xE1;data[1] = 0x15;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xD8;data[1] = 0x00;data[2] = 0x3C;	
			if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
				goto HimaxErr;

			if ((i2c_himax_master_write(client, type28_selected->c38,sizeof(type28_selected->c38),normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xE1;data[1] = 0x00;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			
			data[0] = 0xE1;data[1] = 0x15;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xD8;data[1] = 0x00;data[2] = 0x4C;	
			if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
				goto HimaxErr;

			if ((i2c_himax_master_write(client, type28_selected->c39,sizeof(type28_selected->c39),normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xE1;data[1] = 0x00;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			
			data[0] = 0xE1;data[1] = 0x15;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xD8;data[1] = 0x00;data[2] = 0x64;	
			if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
				goto HimaxErr;

			if ((i2c_himax_master_write(client, type28_selected->c40,sizeof(type28_selected->c40),normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xE1;data[1] = 0x00;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			
			data[0] = 0xE1;data[1] = 0x15;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xD8;data[1] = 0x00;data[2] = 0x7A;	
			if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
				goto HimaxErr;

			if ((i2c_himax_master_write(client, type28_selected->c41,
					sizeof(type28_selected->c41),normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xE1;data[1] = 0x00;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			
			data[0] = 0xE1;data[1] = 0x15;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xD8;data[1] = 0x00;data[2] = 0x96;	
			if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
				goto HimaxErr;

			if ((i2c_himax_master_write(client, type28_selected->c42,
					sizeof(type28_selected->c42),normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xE1;data[1] = 0x00;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			
			data[0] = 0xE1;data[1] = 0x15;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xD8;data[1] = 0x00;data[2] = 0x9E;	
			if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
				goto HimaxErr;

			if ((i2c_himax_master_write(client, type28_selected->c43_1,
					sizeof(type28_selected->c43_1),normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xE1;data[1] = 0x00;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			
			data[0] = 0xE1;data[1] = 0x15;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xD8;data[1] = 0x00;data[2] = 0xBD;	
			if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
				goto HimaxErr;

			if ((i2c_himax_master_write(client, type28_selected->c43_2,
					sizeof(type28_selected->c43_2),normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xE1;data[1] = 0x00;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			
			data[0] = 0xE1;data[1] = 0x15;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xD8;data[1] = 0x00;data[2] = 0xDA;	
			if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
				goto HimaxErr;

			if ((i2c_himax_master_write(client, type28_selected->c44_1,
					sizeof(type28_selected->c44_1),normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xE1;data[1] = 0x00;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			
			data[0] = 0xE1;data[1] = 0x15;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xD8;data[1] = 0x00;data[2] = 0xF9;	
			if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
				goto HimaxErr;

			if ((i2c_himax_master_write(client, type28_selected->c44_2,
					sizeof(type28_selected->c44_2),normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xE1;data[1] = 0x00;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			
			data[0] = 0xE1;data[1] = 0x15;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xD8;data[1] = 0x00;data[2] = 0xFE;	
			if ((i2c_himax_master_write(client, &data[0],3,normalRetry))<0)
				goto HimaxErr;

			if ((i2c_himax_master_write(client, type28_selected->c45,
					sizeof(type28_selected->c45),normalRetry))<0)
				goto HimaxErr;

			data[0] = 0xE1;data[1] = 0x00;
			if ((i2c_himax_master_write(client, &data[0],2,normalRetry))<0)
				goto HimaxErr;

			hr_msleep(1);
		} else {
			E("[HimaxError] %s type28_selected is null.\n",__func__);
			goto HimaxErr;
		}
	#endif
	
	#ifdef HX_ESD_WORKAROUND
	
	i2c_himax_read(client, 0x36, data, 2, normalRetry);
	if(data[0] != 0x0F || data[1] != 0x53)
	{
		
		E("[HimaxError] %s R36 Fail : R36[0]=%d,R36[1]=%d,R36 Counter=%d \n",__func__,data[0],data[1],ESD_R36_FAIL);
		return -1;
	}
	#endif
	
	i2c_himax_write_command(client, 0x83, normalRetry);
	hr_msleep(30);

	i2c_himax_write_command(client, 0x81, normalRetry);
	hr_msleep(50);

	i2c_himax_write_command(client, 0x82, normalRetry);
	hr_msleep(50);

	i2c_himax_write_command(client, 0x80, normalRetry);
	hr_msleep(50);

	
	
	data[0] = 0xE1;
	data[1] = 0x15;
	i2c_himax_master_write(client, &data[0],2,normalRetry);
	hr_msleep(10);

	data[0] = 0xD8;
	data[1] = 0x00;
	data[2] = 0x70;
	i2c_himax_master_write(client, &data[0],3,normalRetry);
	hr_msleep(10);

	i2c_himax_read(client, 0x5A, data, 12, normalRetry);

	HX_RX_NUM = data[1];               
	HX_TX_NUM = data[2];               
	HX_MAX_PT = (data[3] & 0xF0) >> 4; 

	#ifdef HX_EN_SEL_BUTTON
	HX_BT_NUM = (data[3] & 0x0F); 
	#endif

	if((data[5] & 0x04) == 0x04) {
		HX_XY_REVERSE = true;
		HX_Y_RES = data[7]*256 + data[8]; 
		HX_X_RES = data[9]*256 + data[10]; 
	} else {
		HX_XY_REVERSE = false;
		HX_X_RES = data[7]*256 + data[8]; 
		HX_Y_RES = data[9]*256 + data[10]; 
	}

	data[0] = 0xE1;
	data[1] = 0x00;
	i2c_himax_master_write(client, &data[0],2,normalRetry);
	hr_msleep(10);

	
	data[0] = 0xE1;
	data[1] = 0x15;
	i2c_himax_master_write(client, &data[0],2,normalRetry);
	hr_msleep(10);

	data[0] = 0xD8;
	data[1] = 0x00;
	data[2] = 0x02;
	i2c_himax_master_write(client, &data[0],3,normalRetry);
	hr_msleep(10);

	i2c_himax_read(client, 0x5A, data, 10, normalRetry);

	if((data[2] && 0x01) == 1) {
		HX_INT_IS_EDGE = true;
	} else {
		HX_INT_IS_EDGE = false;
	}

	data[0] = 0xE1;
	data[1] = 0x00;
	i2c_himax_master_write(client, &data[0],2,normalRetry);
	hr_msleep(10);

	i2c_himax_write_command(client, 0x83, normalRetry);
	hr_msleep(30);

	i2c_himax_write_command(client, 0x81, normalRetry);
	hr_msleep(50);

	I("%s: initialization complete\n", __func__);

	return result;
HimaxErr:
	return -1;
}


#ifdef HX_TP_SYS_REGISTER
static ssize_t himax_register_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int base = 0;
	uint16_t loop_i,loop_j;
	uint8_t data[128];
	uint8_t outData[5];

	memset(outData, 0x00, sizeof(outData));
	memset(data, 0x00, sizeof(data));

	I("Himax multi_register_command = %d \n",multi_register_command);

	if (multi_register_command == 1) {
		base = 0;

		for(loop_i = 0; loop_i < 6; loop_i++) {
			if (multi_register[loop_i] != 0x00) {
				if (multi_cfg_bank[loop_i] == 1) {
					outData[0] = 0x15;
					i2c_himax_write(private_ts->client, 0xE1 ,&outData[0], 1, DEFAULT_RETRY_CNT);
					hr_msleep(10);

					outData[0] = 0x00;
					outData[1] = multi_register[loop_i];
					i2c_himax_write(private_ts->client, 0xD8 ,&outData[0], 2, DEFAULT_RETRY_CNT);
					hr_msleep(10);

					i2c_himax_read(private_ts->client, 0x5A, data, 128, DEFAULT_RETRY_CNT);

					outData[0] = 0x00;
					i2c_himax_write(private_ts->client, 0xE1 ,&outData[0], 1, DEFAULT_RETRY_CNT);

					for(loop_j=0; loop_j<128; loop_j++)
						multi_value[base++] = data[loop_j];
				} else {
					i2c_himax_read(private_ts->client, multi_register[loop_i], data, 128, DEFAULT_RETRY_CNT);

					for(loop_j=0; loop_j<128; loop_j++)
						multi_value[base++] = data[loop_j];
				}
			}
		}

		base = 0;
		for(loop_i = 0; loop_i < 6; loop_i++) {
			if (multi_register[loop_i] != 0x00) {
				if (multi_cfg_bank[loop_i] == 1)
					ret += sprintf(buf + ret, "Register: FE(%x)\n", multi_register[loop_i]);
				else
					ret += sprintf(buf + ret, "Register: %x\n", multi_register[loop_i]);

				for (loop_j = 0; loop_j < 128; loop_j++) {
					ret += sprintf(buf + ret, "0x%2.2X ", multi_value[base++]);
					if ((loop_j % 16) == 15)
						ret += sprintf(buf + ret, "\n");
				}
			}
		}
		return ret;
	}

	if (config_bank_reg) {
		I("%s: register_command = FE(%x)\n", __func__, register_command);

		
		outData[0] = 0x15;
		i2c_himax_write(private_ts->client, 0xE1,&outData[0], 1, DEFAULT_RETRY_CNT);

		hr_msleep(10);

		outData[0] = 0x00;
		outData[1] = register_command;
		i2c_himax_write(private_ts->client, 0xD8,&outData[0], 2, DEFAULT_RETRY_CNT);

		hr_msleep(10);

		i2c_himax_read(private_ts->client, 0x5A, data, 128, DEFAULT_RETRY_CNT);

		hr_msleep(10);

		outData[0] = 0x00;
		i2c_himax_write(private_ts->client, 0xE1,&outData[0], 1, DEFAULT_RETRY_CNT);
	} else {
		if (i2c_himax_read(private_ts->client, register_command, data, 128, DEFAULT_RETRY_CNT) < 0)
			return ret;
	}

	if (config_bank_reg)
		ret += sprintf(buf, "command: FE(%x)\n", register_command);
	else
		ret += sprintf(buf, "command: %x\n", register_command);

	for (loop_i = 0; loop_i < 128; loop_i++) {
		ret += sprintf(buf + ret, "0x%2.2X ", data[loop_i]);
		if ((loop_i % 16) == 15)
			ret += sprintf(buf + ret, "\n");
	}
	ret += sprintf(buf + ret, "\n");
	return ret;
}

static ssize_t himax_register_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	char buf_tmp[6], length = 0;
	unsigned long result    = 0;
	uint8_t loop_i          = 0;
	uint16_t base           = 5;
	uint8_t write_da[128];
	uint8_t outData[5];

	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(write_da, 0x0, sizeof(write_da));
	memset(outData, 0x0, sizeof(outData));

	I("himax %s \n",buf);

	if (buf[0] == 'm' && buf[1] == 'r' && buf[2] == ':') {
		memset(multi_register, 0x00, sizeof(multi_register));
		memset(multi_cfg_bank, 0x00, sizeof(multi_cfg_bank));
		memset(multi_value, 0x00, sizeof(multi_value));

		I("himax multi register enter\n");

		multi_register_command = 1;

		base 		= 2;
		loop_i 	= 0;

		while(true) {
			if (buf[base] == '\n')
				break;

			if (loop_i >= 6 )
				break;

			if (buf[base] == ':' && buf[base+1] == 'x' && buf[base+2] == 'F' &&
					buf[base+3] == 'E' && buf[base+4] != ':') {
				memcpy(buf_tmp, buf + base + 4, 2);
				if (!strict_strtoul(buf_tmp, 16, &result)) {
					multi_register[loop_i] = result;
					multi_cfg_bank[loop_i++] = 1;
				}
				base += 6;
			} else {
				memcpy(buf_tmp, buf + base + 2, 2);
				if (!strict_strtoul(buf_tmp, 16, &result)) {
					multi_register[loop_i] = result;
					multi_cfg_bank[loop_i++] = 0;
				}
				base += 4;
			}
		}

		I("========================== \n");
		for(loop_i = 0; loop_i < 6; loop_i++)
			I("%d,%d:",multi_register[loop_i],multi_cfg_bank[loop_i]);
		I("\n");
	} else if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':') {
		multi_register_command = 0;

		if (buf[2] == 'x') {
			if (buf[3] == 'F' && buf[4] == 'E') {
				config_bank_reg = true;

				memcpy(buf_tmp, buf + 5, 2);
				if (!strict_strtoul(buf_tmp, 16, &result))
					register_command = result;
				base = 7;

				I("CMD: FE(%x)\n", register_command);
			} else {
				config_bank_reg = false;

				memcpy(buf_tmp, buf + 3, 2);
				if (!strict_strtoul(buf_tmp, 16, &result))
					register_command = result;
				base = 5;
				I("CMD: %x\n", register_command);
			}

			for (loop_i = 0; loop_i < 128; loop_i++) {
				if (buf[base] == '\n') {
					if (buf[0] == 'w') {
						if (config_bank_reg) {
							outData[0] = 0x15;
							i2c_himax_write(private_ts->client, 0xE1, &outData[0], 1, DEFAULT_RETRY_CNT);

							hr_msleep(10);

							outData[0] = 0x00;
							outData[1] = register_command;
							i2c_himax_write(private_ts->client, 0xD8, &outData[0], 2, DEFAULT_RETRY_CNT);

							hr_msleep(10);
							i2c_himax_write(private_ts->client, 0x40, &write_da[0], length, DEFAULT_RETRY_CNT);

							hr_msleep(10);

							outData[0] = 0x00;
							i2c_himax_write(private_ts->client, 0xE1, &outData[0], 1, DEFAULT_RETRY_CNT);

							I("CMD: FE(%x), %x, %d\n", register_command,write_da[0], length);
						} else {
							i2c_himax_write(private_ts->client, register_command, &write_da[0], length, DEFAULT_RETRY_CNT);
							I("CMD: %x, %x, %d\n", register_command,write_da[0], length);
						}
					}
					I("\n");
					return count;
				}
				if (buf[base + 1] == 'x') {
					buf_tmp[4] = '\n';
					buf_tmp[5] = '\0';
					memcpy(buf_tmp, buf + base + 2, 2);
					if (!strict_strtoul(buf_tmp, 16, &result)) {
						write_da[loop_i] = result;
					}
					length++;
				}
				base += 4;
			}
		}
	}
	return count;
}

static DEVICE_ATTR(register, (S_IWUSR|S_IRUGO),himax_register_show, himax_register_store);
#endif

	#ifdef HX_TP_SYS_DIAG
	static uint8_t *getMutualBuffer(void)
	{
		return diag_mutual;
	}

	static uint8_t *getSelfBuffer(void)
	{
		return &diag_self[0];
	}

	static uint8_t getXChannel(void)
	{
		return x_channel;
	}

	static uint8_t getYChannel(void)
	{
		return y_channel;
	}

	static uint8_t getDiagCommand(void)
	{
		return diag_command;
	}

	static void setXChannel(uint8_t x)
	{
		x_channel = x;
	}

	static void setYChannel(uint8_t y)
	{
		y_channel = y;
	}

	static void setMutualBuffer(void)
	{
		diag_mutual = kzalloc(x_channel * y_channel * sizeof(uint8_t), GFP_KERNEL);
	}

	static ssize_t himax_diag_show(struct device *dev,struct device_attribute *attr, char *buf)
	{
		size_t count = 0;
		uint32_t loop_i;
		uint16_t mutual_num, self_num, width;

		mutual_num 	= x_channel * y_channel;
		self_num 		= x_channel + y_channel; 

		width 			= x_channel;
		count += sprintf(buf + count, "ChannelStart: %4d, %4d\n\n", x_channel, y_channel);

		
		if (diag_command >= 1 && diag_command <= 6) {
			if (diag_command <= 3) {
				for (loop_i = 0; loop_i < mutual_num; loop_i++) {
					count += sprintf(buf + count, "%4d", diag_mutual[loop_i]);
					if ((loop_i % width) == (width - 1))
						count += sprintf(buf + count, " %3d\n", diag_self[width + loop_i/width]);
				}
				count += sprintf(buf + count, "\n");
				for (loop_i = 0; loop_i < width; loop_i++) {
					count += sprintf(buf + count, "%4d", diag_self[loop_i]);
					if (((loop_i) % width) == (width - 1))
						count += sprintf(buf + count, "\n");
				}

				#ifdef HX_EN_SEL_BUTTON
				count += sprintf(buf + count, "\n");
				for (loop_i = 0; loop_i < HX_BT_NUM; loop_i++)
					count += sprintf(buf + count, "%4d", diag_self[HX_RX_NUM + HX_TX_NUM + loop_i]);
				#endif
			} else if (diag_command > 4) {
				for (loop_i = 0; loop_i < self_num; loop_i++) {
					count += sprintf(buf + count, "%4d", diag_self[loop_i]);
					if (((loop_i - mutual_num) % width) == (width - 1))
						count += sprintf(buf + count, "\n");
				}
			} else {
				for (loop_i = 0; loop_i < mutual_num; loop_i++) {
					count += sprintf(buf + count, "%4d", diag_mutual[loop_i]);
					if ((loop_i % width) == (width - 1))
						count += sprintf(buf + count, "\n");
				}
			}
			count += sprintf(buf + count, "ChannelEnd");
			count += sprintf(buf + count, "\n");
		} else if (diag_command == 7) {
			for (loop_i = 0; loop_i < 128 ;loop_i++) {
				if ((loop_i % 16) == 0)
					count += sprintf(buf + count, "LineStart:");

				count += sprintf(buf + count, "%4d", diag_coor[loop_i]);
				if ((loop_i % 16) == 15)
					count += sprintf(buf + count, "\n");
			}
		}
		return count;
	}

	static ssize_t himax_diag_dump(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
	{
		const uint8_t command_ec_128_raw_flag 		= 0x01;
		const uint8_t command_ec_24_normal_flag 	= 0x00;

		uint8_t command_ec_128_raw_baseline_flag 	= 0x02;
		uint8_t command_ec_128_raw_bank_flag 			= 0x03;

		uint8_t command_91h[2] = {0x91, 0x00};
		uint8_t command_82h[1] = {0x82};
		uint8_t command_F3h[2] = {0xF3, 0x00};
		uint8_t command_83h[1] = {0x83};
		uint8_t command_80h[1] = {0x80};
		uint8_t command_81h[1] = {0x81};
		uint8_t receive[1];

		memset(receive, 0x00, sizeof(receive));

		if (IC_TYPE != HX_85XX_D_SERIES_PWON)
			command_ec_128_raw_baseline_flag = 0x02 | command_ec_128_raw_flag;
		else {
			command_ec_128_raw_baseline_flag = 0x02;
			command_ec_128_raw_bank_flag = 0x03;
		}

		
		if (buf[0] == '1') {
			if (diag_command == 0)
				diag_command = 1;
			else {
				diag_command --;
				if (diag_command == 0) diag_command = 3;
			}
		} else if (buf[0] == '2') {
			diag_command ++;
			if (diag_command > 3) diag_command = 1;
		} else {
			diag_command = 0;
			E("[Himax]diag error!diag_command=0x%x\n",buf[0]);
		}


		if (buf[0] == '1' || buf[0] == '2'|| buf[0] == '0') {
			if (diag_command == 2)	{
				command_91h[1] = command_ec_128_raw_baseline_flag; 
				i2c_himax_write(private_ts->client, command_91h[0] ,&command_91h[1], 1, DEFAULT_RETRY_CNT);
				I("[Himax]diag_command=0x%x\n",diag_command);
			} else if (diag_command == 1) {
				command_91h[1] = command_ec_128_raw_flag;	
				i2c_himax_write(private_ts->client, command_91h[0] ,&command_91h[1], 1, DEFAULT_RETRY_CNT);
				I("[Himax]diag_command=0x%x\n",diag_command);
			} else if (diag_command == 3) {	
				if (IC_TYPE != HX_85XX_D_SERIES_PWON)
				{
					i2c_himax_write(private_ts->client, command_82h[0] ,&command_82h[0], 0, DEFAULT_RETRY_CNT);
					hr_msleep(50);
					i2c_himax_write(private_ts->client, command_80h[0] ,&command_80h[0], 0, DEFAULT_RETRY_CNT);
					hr_msleep(50);

					i2c_himax_read(private_ts->client, command_F3h[0], receive, 1, DEFAULT_RETRY_CNT) ;
					command_F3h[1] = (receive[0] | 0x80);
					i2c_himax_write(private_ts->client, command_F3h[0] ,&command_F3h[1], 1, DEFAULT_RETRY_CNT);

					command_91h[1] = command_ec_128_raw_baseline_flag;
					i2c_himax_write(private_ts->client, command_91h[0] ,&command_91h[1], 1, DEFAULT_RETRY_CNT);

					i2c_himax_write(private_ts->client, command_83h[0] ,&command_83h[0], 0, DEFAULT_RETRY_CNT);
					hr_msleep(50);
					i2c_himax_write(private_ts->client, command_81h[0] ,&command_81h[0], 0, DEFAULT_RETRY_CNT);
					hr_msleep(50);
				}
				else
				{
					command_91h[1] = command_ec_128_raw_bank_flag;	
					i2c_himax_write(private_ts->client, command_91h[0] ,&command_91h[1], 1, DEFAULT_RETRY_CNT);
				}
				I("[Himax]diag_command=0x%x\n",diag_command);
			} else {
				if (IC_TYPE != HX_85XX_D_SERIES_PWON)
				{
					i2c_himax_write(private_ts->client, command_82h[0] ,&command_82h[0], 0, DEFAULT_RETRY_CNT);
					hr_msleep(50);
					i2c_himax_write(private_ts->client, command_80h[0] ,&command_80h[0], 0, DEFAULT_RETRY_CNT);
					hr_msleep(50);
					command_91h[1] = command_ec_24_normal_flag;
					i2c_himax_write(private_ts->client, command_91h[0] ,&command_91h[1], 1, DEFAULT_RETRY_CNT);
					i2c_himax_read(private_ts->client, command_F3h[0], receive, 1, DEFAULT_RETRY_CNT);
					command_F3h[1] = (receive[0] & 0x7F);
					i2c_himax_write(private_ts->client, command_F3h[0] ,&command_F3h[1], 1, DEFAULT_RETRY_CNT);
					i2c_himax_write(private_ts->client, command_83h[0] ,&command_83h[0], 0, DEFAULT_RETRY_CNT);
					hr_msleep(50);
					i2c_himax_write(private_ts->client, command_81h[0] ,&command_81h[0], 0, DEFAULT_RETRY_CNT);
					hr_msleep(50);
				}
				else
				{
					command_91h[1] = command_ec_24_normal_flag;
					i2c_himax_write(private_ts->client, command_91h[0] ,&command_91h[1], 1, DEFAULT_RETRY_CNT);
				}
				
				touch_monitor_stop_flag = touch_monitor_stop_limit;
				
				I("[Himax]diag_command=0x%x\n",diag_command);
			}
		}
		else if (buf[0] == '7')
		{
			diag_command = buf[0] - '0';
		}
		
		else if (buf[0] == '8')
		{
			diag_command = buf[0] - '0';

			coordinate_fn = filp_open(DIAG_COORDINATE_FILE,O_CREAT | O_WRONLY | O_APPEND | O_TRUNC,0666);
			if (IS_ERR(coordinate_fn))
			{
				E("%s: coordinate_dump_file_create error\n", __func__);
				coordinate_dump_enable = 0;
				filp_close(coordinate_fn,NULL);
			}
			coordinate_dump_enable = 1;
		}
		else if (buf[0] == '9')
		{
			coordinate_dump_enable = 0;
			diag_command = buf[0] - '0';

			if (!IS_ERR(coordinate_fn))
			{
				filp_close(coordinate_fn,NULL);
			}
		}
		
		else
		{
			E("[Himax]Diag command error!diag_command=0x%x\n",diag_command);
		}
		return count;
	}
	static DEVICE_ATTR(diag, (S_IWUSR|S_IRUGO),himax_diag_show, himax_diag_dump);
	#endif
	

	
	#ifdef HX_TP_SYS_RESET
	static ssize_t himax_reset_set(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
	{
		if (buf[0] == '1')
			doHWreset();
		return count;
	}

	static DEVICE_ATTR(reset, (S_IWUSR|S_IRUGO),NULL, himax_reset_set);
	#endif
	

	
	#ifdef HX_TP_SYS_DEBUG

	int fts_ctpm_fw_upgrade_with_sys_fs(unsigned char *fw, int len)
	{
		unsigned char* ImageBuffer = fw;
		int fullFileLength = len;
		int i, j;
		uint8_t cmd[5], last_byte, prePage;
		int FileLength;
		uint8_t checksumResult = 0;

		
		for (j = 0; j < 3; j++)
		{
			if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_CRC)
			{
				FileLength = fullFileLength;
			}
			else
			{
				FileLength = fullFileLength - 2;
			}

			#ifdef HX_RST_PIN_FUNC
				himax_HW_reset();
			#endif

			if ( i2c_himax_write(private_ts->client, 0x81 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
			{
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			hr_msleep(120);

			himax_unlock_flash();  

			cmd[0] = 0x05;cmd[1] = 0x00;cmd[2] = 0x02;
			if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
			{
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}

			if ( i2c_himax_write(private_ts->client, 0x4F ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
			{
				E("%s: i2c access fail!\n", __func__);
				return 0;
			}
			hr_msleep(50);

			himax_ManualMode(1);
			himax_FlashMode(1);

			FileLength = (FileLength + 3) / 4;
			for (i = 0, prePage = 0; i < FileLength; i++)
			{
				last_byte = 0;
				cmd[0] = i & 0x1F;
				if (cmd[0] == 0x1F || i == FileLength - 1)
				{
					last_byte = 1;
				}
				cmd[1] = (i >> 5) & 0x1F;
				cmd[2] = (i >> 10) & 0x1F;
				if ( i2c_himax_write(private_ts->client, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
				{
					E("%s: i2c access fail!\n", __func__);
					return 0;
				}

				if (prePage != cmd[1] || i == 0)
				{
					prePage = cmd[1];
					cmd[0] = 0x01;cmd[1] = 0x09;
					if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
					{
						E("%s: i2c access fail!\n", __func__);
						return 0;
					}

					cmd[0] = 0x01;cmd[1] = 0x0D;
					if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
					{
						E("%s: i2c access fail!\n", __func__);
						return 0;
					}

					cmd[0] = 0x01;cmd[1] = 0x09;
					if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
					{
						E("%s: i2c access fail!\n", __func__);
						return 0;
					}
				}

				memcpy(&cmd[0], &ImageBuffer[4*i], 4);
				if ( i2c_himax_write(private_ts->client, 0x45 ,&cmd[0], 4, DEFAULT_RETRY_CNT) < 0)
				{
					E("%s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;cmd[1] = 0x0D;
				if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					E("%s: i2c access fail!\n", __func__);
					return 0;
				}

				cmd[0] = 0x01;cmd[1] = 0x09;
				if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					E("%s: i2c access fail!\n", __func__);
					return 0;
				}

				if (last_byte == 1)
				{
					cmd[0] = 0x01;cmd[1] = 0x01;
					if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
					{
						E("%s: i2c access fail!\n", __func__);
						return 0;
					}

					cmd[0] = 0x01;cmd[1] = 0x05;
					if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
					{
						E("%s: i2c access fail!\n", __func__);
						return 0;
					}

					cmd[0] = 0x01;cmd[1] = 0x01;
					if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
					{
						E("%s: i2c access fail!\n", __func__);
						return 0;
					}

					cmd[0] = 0x01;cmd[1] = 0x00;
					if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
					{
						E("%s: i2c access fail!\n", __func__);
						return 0;
					}

					hr_msleep(10);
					if (i == (FileLength - 1))
					{
						himax_FlashMode(0);
						himax_ManualMode(0);
						checksumResult = himax_calculateChecksum(ImageBuffer, fullFileLength);
						
						himax_lock_flash();

						if (checksumResult) 
						{
							return 1;
						}
						else 
						{
							return 0;
						}
					}
				}
			}
		}
		return 0;
	}

	static ssize_t himax_debug_show(struct device *dev,struct device_attribute *attr, char *buf)
	{
		size_t count = 0;
		int i = 0;

		if (debug_level_cmd == 't')
		{
			if (fw_update_complete)
			{
				count += sprintf(buf, "FW Update Complete \n");
			}
			else
			{
				count += sprintf(buf, "FW Update Fail \n");
			}
		}
		else if (debug_level_cmd == 'i')
		{
			if (irq_enable)
			{
				count += sprintf(buf, "IRQ is enable\n");
			}
			else
			{
				count += sprintf(buf, "IRQ is disable\n");
			}
		}
		else if (debug_level_cmd == 'h')
		{
			if (handshaking_result == 0)
			{
				count += sprintf(buf, "Handshaking Result = %d (MCU Running)\n",handshaking_result);
			}
			else if (handshaking_result == 1)
			{
				count += sprintf(buf, "Handshaking Result = %d (MCU Stop)\n",handshaking_result);
			}
			else if (handshaking_result == 2)
			{
				count += sprintf(buf, "Handshaking Result = %d (I2C Error)\n",handshaking_result);
			}
			else
			{
				count += sprintf(buf, "Handshaking Result = error \n");
			}
		}
		else if (debug_level_cmd == 'v')
		{
			count += sprintf(buf + count, "FW_VER_MAJ_buff = ");
             count += sprintf(buf + count, "0x%2.2X \n",FW_VER_MAJ_buff[0]);

			count += sprintf(buf + count, "FW_VER_MIN_buff = ");
             count += sprintf(buf + count, "0x%2.2X \n",FW_VER_MIN_buff[0]);

			count += sprintf(buf + count, "CFG_VER_MAJ_buff = ");
			for( i=0 ; i<12 ; i++)
			{
				count += sprintf(buf + count, "0x%2.2X ",CFG_VER_MAJ_buff[i]);
			}
			count += sprintf(buf + count, "\n");

			count += sprintf(buf + count, "CFG_VER_MIN_buff = ");
             for( i=0 ; i<12 ; i++)
             {
                 count += sprintf(buf + count, "0x%2.2X ",CFG_VER_MIN_buff[i]);
             }
			count += sprintf(buf + count, "\n");
		}
		else if (debug_level_cmd == 'd')
		{
			count += sprintf(buf + count, "Himax Touch IC Information :\n");
			if (IC_TYPE == HX_85XX_A_SERIES_PWON)
			{
				count += sprintf(buf + count, "IC Type : A\n");
			}
			else if (IC_TYPE == HX_85XX_B_SERIES_PWON)
			{
				count += sprintf(buf + count, "IC Type : B\n");
			}
			else if (IC_TYPE == HX_85XX_C_SERIES_PWON)
			{
				count += sprintf(buf + count, "IC Type : C\n");
			}
			else if (IC_TYPE == HX_85XX_D_SERIES_PWON)
			{
				count += sprintf(buf + count, "IC Type : D\n");
			}
			else
			{
				count += sprintf(buf + count, "IC Type error.\n");
			}

			if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_SW)
			{
				count += sprintf(buf + count, "IC Checksum : SW\n");
			}
			else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_HW)
			{
				count += sprintf(buf + count, "IC Checksum : HW\n");
			}
			else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_CRC)
			{
				count += sprintf(buf + count, "IC Checksum : CRC\n");
			}
			else
			{
				count += sprintf(buf + count, "IC Checksum error.\n");
			}

			if (HX_INT_IS_EDGE)
			{
				count += sprintf(buf + count, "Interrupt : EDGE TIRGGER\n");
			}
			else
			{
				count += sprintf(buf + count, "Interrupt : LEVEL TRIGGER\n");
			}

			count += sprintf(buf + count, "RX Num : %d\n",HX_RX_NUM);
			count += sprintf(buf + count, "TX Num : %d\n",HX_TX_NUM);
			count += sprintf(buf + count, "BT Num : %d\n",HX_BT_NUM);
			count += sprintf(buf + count, "X Resolution : %d\n",HX_X_RES);
			count += sprintf(buf + count, "Y Resolution : %d\n",HX_Y_RES);
			count += sprintf(buf + count, "Max Point : %d\n",HX_MAX_PT);
		}
		return count;
	}

	static ssize_t himax_debug_dump(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
	{
		struct file* filp = NULL;
		mm_segment_t oldfs;
		int result = 0;
		char fileName[128];

		if (buf[0] == 'i') 
		{
			debug_level_cmd = buf[0];

			if (buf[2] == '1') 
			{
				himax_int_enable(1);
				irq_enable = true;
			}
			else if (buf[2] == '0') 
			{
				himax_int_enable(0);
				irq_enable = false;
			}
			else
			{
				E("%s: debug_level command = 'i' , parameter error.\n", __func__);
			}
			return count;
		}

		if ( buf[0] == 'h') 
		{
			debug_level_cmd = buf[0];

			himax_int_enable(0);

			handshaking_result = himax_hang_shaking(); 

			himax_int_enable(1);

			return count;
		}

		if ( buf[0] == 'v') 
		{
			debug_level_cmd = buf[0];
			himax_read_FW_ver(true);
			return count;
		}

		if ( buf[0] == 'd') 
		{
			debug_level_cmd = buf[0];
			return count;
		}
		
			#ifdef ENABLE_CHIP_STATUS_MONITOR
			cancel_delayed_work_sync(&private_ts->himax_chip_monitor);
			#endif
		

		if (buf[0] == 't')
		{
			debug_level_cmd 		= buf[0];
			fw_update_complete		= false;

			memset(fileName, 0, 128);
			
			snprintf(fileName, count-2, "%s", &buf[2]);
			I("%s: upgrade from file(%s) start!\n", __func__, fileName);
			
			filp = filp_open(fileName, O_RDONLY, 0);
			if (IS_ERR(filp))
			{
				E("%s: open firmware file failed\n", __func__);
				goto firmware_upgrade_done;
				
			}
			oldfs = get_fs();
			set_fs(get_ds());

			
			result=filp->f_op->read(filp,upgrade_fw,sizeof(upgrade_fw), &filp->f_pos);
			if (result < 0)
			{
				E("%s: read firmware file failed\n", __func__);
				goto firmware_upgrade_done;
				
			}

			set_fs(oldfs);
			filp_close(filp, NULL);

			I("%s: upgrade start,len %d: %02X, %02X, %02X, %02X\n", __func__, result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2], upgrade_fw[3]);

			if (result > 0)
			{
				
				himax_int_enable(0);
				if (fts_ctpm_fw_upgrade_with_sys_fs(upgrade_fw, result) == 0)
				{
					E("%s: TP upgrade error, line: %d\n", __func__, __LINE__);
					fw_update_complete = false;
				}
				else
				{
					I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
					fw_update_complete = true;
				}
				himax_int_enable(1);
				goto firmware_upgrade_done;
				
			}
		}

		#ifdef HX_FW_UPDATE_BY_I_FILE
			if (buf[0] == 'f')
			{
				I("%s: upgrade firmware from kernel image start!\n", __func__);
				if (i_isTP_Updated == 0)
				{
					I("himax touch isTP_Updated: %d\n", i_isTP_Updated);
					if (1)
					{
						himax_int_enable(0);
						I("himax touch firmware upgrade: %d\n", i_isTP_Updated);
						if (fts_ctpm_fw_upgrade_with_i_file() == 0)
						{
							E("himax_marked TP upgrade error, line: %d\n", __LINE__);
							fw_update_complete = false;
						}
						else
						{
							I("himax_marked TP upgrade OK, line: %d\n", __LINE__);
							fw_update_complete = true;
						}
						himax_int_enable(1);
						i_isTP_Updated = 1;
						goto firmware_upgrade_done;
					}
				}
			}
		#endif

		firmware_upgrade_done:

		
		#ifdef HX_RST_PIN_FUNC
		himax_HW_reset();
		himax_loadSensorConfig(private_ts->client,private_ts->pdata,true);
		#endif
		

		
		
		

		
			#ifdef ENABLE_CHIP_STATUS_MONITOR
			queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_monitor, 10*HZ);
			#endif
		

		return count;
	}

	static DEVICE_ATTR(debug, (S_IWUSR|S_IRUGO),himax_debug_show, himax_debug_dump);

	#endif
	

	
	#ifdef HX_TP_SYS_FLASH_DUMP

	static uint8_t getFlashCommand(void)
	{
		return flash_command;
	}

	static uint8_t getFlashDumpProgress(void)
	{
		return flash_progress;
	}

	static uint8_t getFlashDumpComplete(void)
	{
		return flash_dump_complete;
	}

	static uint8_t getFlashDumpFail(void)
	{
		return flash_dump_fail;
	}

	static uint8_t getSysOperation(void)
	{
		return sys_operation;
	}

	static uint8_t getFlashReadStep(void)
	{
		return flash_read_step;
	}

	static uint8_t getFlashDumpSector(void)
	{
		return flash_dump_sector;
	}

	static uint8_t getFlashDumpPage(void)
	{
		return flash_dump_page;
	}

	static bool getFlashDumpGoing(void)
	{
		return flash_dump_going;
	}

	static void setFlashBuffer(void)
	{
		int i=0;
		flash_buffer = kzalloc(32768*sizeof(uint8_t), GFP_KERNEL);
		for(i=0; i<32768; i++)
		{
			flash_buffer[i] = 0x00;
		}
	}

	static void setSysOperation(uint8_t operation)
	{
		sys_operation = operation;
	}

	static void setFlashDumpProgress(uint8_t progress)
	{
		flash_progress = progress;
		
	}

	static void setFlashDumpComplete(uint8_t status)
	{
		flash_dump_complete = status;
	}

	static void setFlashDumpFail(uint8_t fail)
	{
		flash_dump_fail = fail;
	}

	static void setFlashCommand(uint8_t command)
	{
		flash_command = command;
	}

	static void setFlashReadStep(uint8_t step)
	{
		flash_read_step = step;
	}

	static void setFlashDumpSector(uint8_t sector)
	{
		flash_dump_sector = sector;
	}

	static void setFlashDumpPage(uint8_t page)
	{
		flash_dump_page = page;
	}

	static void setFlashDumpGoing(bool going)
	{
		flash_dump_going = going;
	}

	static ssize_t himax_flash_show(struct device *dev,struct device_attribute *attr, char *buf)
	{
		int ret = 0;
		int loop_i;
		uint8_t local_flash_read_step=0;
		uint8_t local_flash_complete = 0;
		uint8_t local_flash_progress = 0;
		uint8_t local_flash_command = 0;
		uint8_t local_flash_fail = 0;

		local_flash_complete = getFlashDumpComplete();
		local_flash_progress = getFlashDumpProgress();
		local_flash_command = getFlashCommand();
		local_flash_fail = getFlashDumpFail();

		I("TPPPP flash_progress = %d \n",local_flash_progress);

		if (local_flash_fail)
		{
			ret += sprintf(buf+ret, "FlashStart:Fail \n");
			ret += sprintf(buf + ret, "FlashEnd");
			ret += sprintf(buf + ret, "\n");
			return ret;
		}

		if (!local_flash_complete)
		{
			ret += sprintf(buf+ret, "FlashStart:Ongoing:0x%2.2x \n",flash_progress);
			ret += sprintf(buf + ret, "FlashEnd");
			ret += sprintf(buf + ret, "\n");
			return ret;
		}

		if (local_flash_command == 1 && local_flash_complete)
		{
			ret += sprintf(buf+ret, "FlashStart:Complete \n");
			ret += sprintf(buf + ret, "FlashEnd");
			ret += sprintf(buf + ret, "\n");
			return ret;
		}

		if (local_flash_command == 3 && local_flash_complete)
		{
			ret += sprintf(buf+ret, "FlashStart: \n");
			for(loop_i = 0; loop_i < 128; loop_i++)
			{
				ret += sprintf(buf + ret, "x%2.2x", flash_buffer[loop_i]);
				if ((loop_i % 16) == 15)
				{
					ret += sprintf(buf + ret, "\n");
				}
			}
			ret += sprintf(buf + ret, "FlashEnd");
			ret += sprintf(buf + ret, "\n");
			return ret;
		}

		
		local_flash_read_step = getFlashReadStep();

		ret += sprintf(buf+ret, "FlashStart:%2.2x \n",local_flash_read_step);

		for (loop_i = 0; loop_i < 1024; loop_i++)
		{
			ret += sprintf(buf + ret, "x%2.2X", flash_buffer[local_flash_read_step*1024 + loop_i]);

			if ((loop_i % 16) == 15)
			{
				ret += sprintf(buf + ret, "\n");
			}
		}

		ret += sprintf(buf + ret, "FlashEnd");
		ret += sprintf(buf + ret, "\n");
		return ret;
	}

	static ssize_t himax_flash_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
	{
		char buf_tmp[6];
		unsigned long result = 0;
		uint8_t loop_i = 0;
		int base = 0;

		memset(buf_tmp, 0x0, sizeof(buf_tmp));

		I("%s: buf[0] = %s\n", __func__, buf);

		if (getSysOperation() == 1)
		{
			E("%s: SYS is busy , return!\n", __func__);
			return count;
		}

		if (buf[0] == '0')
		{
			setFlashCommand(0);
			if (buf[1] == ':' && buf[2] == 'x')
			{
				memcpy(buf_tmp, buf + 3, 2);
				I("%s: read_Step = %s\n", __func__, buf_tmp);
				if (!strict_strtoul(buf_tmp, 16, &result))
				{
					I("%s: read_Step = %lu \n", __func__, result);
					setFlashReadStep(result);
				}
			}
		}
		else if (buf[0] == '1')
		{
			setSysOperation(1);
			setFlashCommand(1);
			setFlashDumpProgress(0);
			setFlashDumpComplete(0);
			setFlashDumpFail(0);
			queue_work(private_ts->flash_wq, &private_ts->flash_work);
		}
		else if (buf[0] == '2')
		{
			setSysOperation(1);
			setFlashCommand(2);
			setFlashDumpProgress(0);
			setFlashDumpComplete(0);
			setFlashDumpFail(0);

			queue_work(private_ts->flash_wq, &private_ts->flash_work);
		}
		else if (buf[0] == '3')
		{
			setSysOperation(1);
			setFlashCommand(3);
			setFlashDumpProgress(0);
			setFlashDumpComplete(0);
			setFlashDumpFail(0);

			memcpy(buf_tmp, buf + 3, 2);
			if (!strict_strtoul(buf_tmp, 16, &result))
			{
				setFlashDumpSector(result);
			}

			memcpy(buf_tmp, buf + 7, 2);
			if (!strict_strtoul(buf_tmp, 16, &result))
			{
				setFlashDumpPage(result);
			}

			queue_work(private_ts->flash_wq, &private_ts->flash_work);
		}
		else if (buf[0] == '4')
		{
			I("%s: command 4 enter.\n", __func__);
			setSysOperation(1);
			setFlashCommand(4);
			setFlashDumpProgress(0);
			setFlashDumpComplete(0);
			setFlashDumpFail(0);

			memcpy(buf_tmp, buf + 3, 2);
			if (!strict_strtoul(buf_tmp, 16, &result))
			{
				setFlashDumpSector(result);
			}
			else
			{
				E("%s: command 4 , sector error.\n", __func__);
				return count;
			}

			memcpy(buf_tmp, buf + 7, 2);
			if (!strict_strtoul(buf_tmp, 16, &result))
			{
				setFlashDumpPage(result);
			}
			else
			{
				E("%s: command 4 , page error.\n", __func__);
				return count;
			}

			base = 11;

			I("=========Himax flash page buffer start=========\n");
			for(loop_i=0;loop_i<128;loop_i++)
			{
				memcpy(buf_tmp, buf + base, 2);
				if (!strict_strtoul(buf_tmp, 16, &result))
				{
					flash_buffer[loop_i] = result;
					I("%d ",flash_buffer[loop_i]);
					if (loop_i % 16 == 15)
					{
						I("\n");
					}
				}
				base += 3;
			}
			I("=========Himax flash page buffer end=========\n");

			queue_work(private_ts->flash_wq, &private_ts->flash_work);
		}
		return count;
	}
	static DEVICE_ATTR(flash_dump, (S_IWUSR|S_IRUGO),himax_flash_show, himax_flash_store);

	static void himax_ts_flash_work_func(struct work_struct *work)
	{
		struct himax_ts_data *ts = container_of(work, struct himax_ts_data, flash_work);

		uint8_t page_tmp[128];
		uint8_t x59_tmp[4] = {0,0,0,0};
		int i=0, j=0, k=0, l=0, buffer_ptr = 0, flash_end_count = 0;
		uint8_t local_flash_command = 0;
		uint8_t sector = 0;
		uint8_t page = 0;

		uint8_t x81_command[2] = {0x81,0x00};
		uint8_t x82_command[2] = {0x82,0x00};
		uint8_t x42_command[2] = {0x42,0x00};
		uint8_t x43_command[4] = {0x43,0x00,0x00,0x00};
		uint8_t x44_command[4] = {0x44,0x00,0x00,0x00};
		uint8_t x45_command[5] = {0x45,0x00,0x00,0x00,0x00};
		uint8_t x46_command[2] = {0x46,0x00};
		uint8_t x4A_command[2] = {0x4A,0x00};
		uint8_t x4D_command[2] = {0x4D,0x00};
		

		himax_int_enable(0);
		setFlashDumpGoing(true);

		#ifdef HX_RST_PIN_FUNC
			himax_HW_reset();
		#endif

		sector = getFlashDumpSector();
		page = getFlashDumpPage();

		local_flash_command = getFlashCommand();

		if ( i2c_himax_master_write(ts->client, x81_command, 1, 3) < 0 )
		{
			E("%s i2c write 81 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		hr_msleep(120);

		if ( i2c_himax_master_write(ts->client, x82_command, 1, 3) < 0 )
		{
			E("%s i2c write 82 fail.\n",__func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		hr_msleep(100);

		I("%s: local_flash_command = %d enter.\n", __func__,local_flash_command);

		if (local_flash_command == 1 || local_flash_command == 2)
		{
			x43_command[1] = 0x01;
			if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 1, DEFAULT_RETRY_CNT) < 0)
			{
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(100);

			for( i=0 ; i<8 ;i++)
			{
				for(j=0 ; j<32 ; j++)
				{
					
					
					for(k=0; k<128; k++)
					{
						page_tmp[k] = 0x00;
					}
					for(k=0; k<32; k++)
					{
						x44_command[1] = k;
						x44_command[2] = j;
						x44_command[3] = i;
						if ( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
						{
							E("%s i2c write 44 fail.\n",__func__);
							goto Flash_Dump_i2c_transfer_error;
						}

						if ( i2c_himax_write_command(ts->client, x46_command[0], DEFAULT_RETRY_CNT) < 0)
						{
							E("%s i2c write 46 fail.\n",__func__);
							goto Flash_Dump_i2c_transfer_error;
						}
						
						if ( i2c_himax_read(ts->client, 0x59, x59_tmp, 4, DEFAULT_RETRY_CNT) < 0)
						{
							E("%s i2c write 59 fail.\n",__func__);
							goto Flash_Dump_i2c_transfer_error;
						}
						
						for(l=0; l<4; l++)
						{
							page_tmp[k*4+l] = x59_tmp[l];
						}
						
					}
					

					for(k=0; k<128; k++)
					{
						flash_buffer[buffer_ptr++] = page_tmp[k];

						if (page_tmp[k] == 0xFF)
						{
							flash_end_count ++;
							if (flash_end_count == 32)
							{
								flash_end_count = 0;
								buffer_ptr = buffer_ptr -32;
								goto FLASH_END;
							}
						}
						else
						{
							flash_end_count = 0;
						}
					}
					setFlashDumpProgress(i*32 + j);
				}
			}
		}
		else if (local_flash_command == 3)
		{
			x43_command[1] = 0x01;
			if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 43 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(100);

			for(i=0; i<128; i++)
			{
				page_tmp[i] = 0x00;
			}

			for(i=0; i<32; i++)
			{
				x44_command[1] = i;
				x44_command[2] = page;
				x44_command[3] = sector;

				if ( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
				{
					E("%s i2c write 44 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}

				if ( i2c_himax_write_command(ts->client, x46_command[0], DEFAULT_RETRY_CNT) < 0 )
				{
					E("%s i2c write 46 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				
				if ( i2c_himax_read(ts->client, 0x59, x59_tmp, 4, DEFAULT_RETRY_CNT) < 0 )
				{
					E("%s i2c write 59 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				
				for(j=0; j<4; j++)
				{
					page_tmp[i*4+j] = x59_tmp[j];
				}
				
			}
			
			for(i=0; i<128; i++)
			{
				flash_buffer[buffer_ptr++] = page_tmp[i];
			}
		}
		else if (local_flash_command == 4)
		{
			
			

			
			
			
			x43_command[1] = 0x01;
			x43_command[2] = 0x00;
			x43_command[3] = 0x06;
			if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 43 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(10);

			x44_command[1] = 0x03;
			x44_command[2] = 0x00;
			x44_command[3] = 0x00;
			if ( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 44 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(10);

			x45_command[1] = 0x00;
			x45_command[2] = 0x00;
			x45_command[3] = 0x3D;
			x45_command[4] = 0x03;
			if ( i2c_himax_write(ts->client, x45_command[0],&x45_command[1], 4, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 45 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(10);

			if ( i2c_himax_write_command(ts->client, x4A_command[0], DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 4A fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(50);

			
			
			
			x43_command[1] = 0x01;
			x43_command[2] = 0x00;
			x43_command[3] = 0x02;
			if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 43 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(10);

			x44_command[1] = 0x00;
			x44_command[2] = page;
			x44_command[3] = sector;
			if ( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 44 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(10);

			if ( i2c_himax_write_command(ts->client, x4D_command[0], DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 4D fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(100);

			
			
			
			x42_command[1] = 0x01;
			if ( i2c_himax_write(ts->client, x42_command[0],&x42_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 42 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(100);

			
			
			
			x43_command[1] = 0x01;
			x43_command[2] = 0x00;
			if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 43 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(10);

			
			
			
			x44_command[1] = 0x00;
			x44_command[2] = page;
			x44_command[3] = sector;
			if ( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 44 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(10);

			
			
			
			x43_command[1] = 0x01;
			x43_command[2] = 0x09;
			if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 43 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(10);

			x43_command[1] = 0x01;
			x43_command[2] = 0x0D;
			if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 43 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(10);

			x43_command[1] = 0x01;
			x43_command[2] = 0x09;
			if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 43 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(10);

			for(i=0; i<32; i++)
			{
				I("himax :i=%d \n",i);
				x44_command[1] = i;
				x44_command[2] = page;
				x44_command[3] = sector;
				if ( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
				{
					E("%s i2c write 44 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				hr_msleep(10);

				x45_command[1] = flash_buffer[i*4 + 0];
				x45_command[2] = flash_buffer[i*4 + 1];
				x45_command[3] = flash_buffer[i*4 + 2];
				x45_command[4] = flash_buffer[i*4 + 3];
				if ( i2c_himax_write(ts->client, x45_command[0],&x45_command[1], 4, DEFAULT_RETRY_CNT) < 0 )
				{
					E("%s i2c write 45 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				hr_msleep(10);

				
				// manual mode command : 48 ,data will be written into flash buffer
				
				x43_command[1] = 0x01;
				x43_command[2] = 0x0D;
				if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
				{
					E("%s i2c write 43 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				hr_msleep(10);

				x43_command[1] = 0x01;
				x43_command[2] = 0x09;
				if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
				{
					E("%s i2c write 43 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				hr_msleep(10);
			}

			
			
			
			x43_command[1] = 0x01;
			x43_command[2] = 0x01;
			if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 43 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(10);

			x43_command[1] = 0x01;
			x43_command[2] = 0x05;
			if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 43 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(10);

			x43_command[1] = 0x01;
			x43_command[2] = 0x01;
			if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 43 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(10);

			x43_command[1] = 0x01;
			x43_command[2] = 0x00;
			if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 43 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(10);

			
			
			
			x43_command[1] = 0x00;
			if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 43 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(10);

			
			
			
			x42_command[1] = 0x00;
			if ( i2c_himax_write(ts->client, x42_command[0],&x42_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 43 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(10);

			
			
			
			x43_command[1] = 0x01;
			x43_command[2] = 0x00;
			x43_command[3] = 0x06;
			if ( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 43 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(10);

			x44_command[1] = 0x03;
			x44_command[2] = 0x00;
			x44_command[3] = 0x00;
			if (i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 44 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(10);

			x45_command[1] = 0x00;
			x45_command[2] = 0x00;
			x45_command[3] = 0x7D;
			x45_command[4] = 0x03;
			if (i2c_himax_write(ts->client, x45_command[0],&x45_command[1], 4, DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 45 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			hr_msleep(10);

			if ( i2c_himax_write_command(ts->client, x4A_command[0], DEFAULT_RETRY_CNT) < 0 )
			{
				E("%s i2c write 4D fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}

			hr_msleep(50);

			buffer_ptr = 128;
			I("Himax: Flash page write Complete~~~~~~~~~~~~~~~~~~~~~~~\n");
		}

		FLASH_END:

		I("Complete~~~~~~~~~~~~~~~~~~~~~~~\n");

		i2c_himax_master_write(ts->client, x43_command, 1, 3);
		hr_msleep(50);

		if (local_flash_command == 2)
		{
			struct file *fn;

			fn = filp_open(FLASH_DUMP_FILE,O_CREAT | O_WRONLY ,0);
			if (!IS_ERR(fn))
			{
				fn->f_op->write(fn,flash_buffer,buffer_ptr*sizeof(uint8_t),&fn->f_pos);
				filp_close(fn,NULL);
			}
		}

		
		#ifdef HX_RST_PIN_FUNC
		himax_HW_reset();
		himax_loadSensorConfig(private_ts->client,private_ts->pdata,true);
		#endif
		

		himax_int_enable(1);
		setFlashDumpGoing(false);

		setFlashDumpComplete(1);
		setSysOperation(0);
		return;

		Flash_Dump_i2c_transfer_error:

		
		#ifdef HX_RST_PIN_FUNC
		himax_HW_reset();
		himax_loadSensorConfig(private_ts->client,private_ts->pdata,true);
		#endif
		

		himax_int_enable(1);
		setFlashDumpGoing(false);
		setFlashDumpComplete(0);
		setFlashDumpFail(1);
		setSysOperation(0);
		return;
	}
	#endif
	

	
	#ifdef HX_TP_SYS_SELF_TEST
	static ssize_t himax_chip_self_test_function(struct device *dev, struct device_attribute *attr, char *buf)
	{
		int val=0x00;
		val = himax_chip_self_test();

		if (val == 0x00) {
			return sprintf(buf, "Self_Test Pass\n");
		} else {
			return sprintf(buf, "Self_Test Fail\n");
		}
	}

	static int himax_chip_self_test(void)
	{
		uint8_t cmdbuf[11];
		int ret = 0;
		uint8_t valuebuf[16];
		int i=0, pf_value=0x00;

		memset(valuebuf, 0x00, sizeof(valuebuf));

		
		
		
		
		

		

		if (IC_TYPE == HX_85XX_C_SERIES_PWON) {
			
			cmdbuf[0] = HX_CMD_TSSOFF;
			ret = i2c_himax_master_write(private_ts->client, cmdbuf, 1, DEFAULT_RETRY_CNT);
			if (ret < 0) {
				E("[Himax]:write TSSOFF failed line: %d \n",__LINE__);
			}
			hr_msleep(120); 

			cmdbuf[0] = HX_CMD_SELFTEST_BUFFER;
			cmdbuf[1] = 0xB4; 
			cmdbuf[2] = 0x64; 
			cmdbuf[3] = 0x36;
			cmdbuf[4] = 0x09;
			cmdbuf[5] = 0x2D;
			cmdbuf[6] = 0x09;
			cmdbuf[7] = 0x32;
			cmdbuf[8] = 0x08;
			ret = i2c_himax_master_write(private_ts->client, cmdbuf, 9, DEFAULT_RETRY_CNT);
			if (ret < 0) {
				E("[Himax]:write HX_CMD_SELFTEST_BUFFER failed line: %d \n",__LINE__);
			}

			udelay(100);

			ret = i2c_himax_read(private_ts->client, HX_CMD_SELFTEST_BUFFER, valuebuf, 9, DEFAULT_RETRY_CNT);
			if (ret < 0) {
				E("[Himax]:read HX_CMD_SELFTEST_BUFFER failed line: %d \n",__LINE__);
			}
			I("[Himax]:0x8D[0] = 0x%x\n",valuebuf[0]);
			I("[Himax]:0x8D[1] = 0x%x\n",valuebuf[1]);
			I("[Himax]:0x8D[2] = 0x%x\n",valuebuf[2]);
			I("[Himax]:0x8D[3] = 0x%x\n",valuebuf[3]);
			I("[Himax]:0x8D[4] = 0x%x\n",valuebuf[4]);
			I("[Himax]:0x8D[5] = 0x%x\n",valuebuf[5]);
			I("[Himax]:0x8D[6] = 0x%x\n",valuebuf[6]);
			I("[Himax]:0x8D[7] = 0x%x\n",valuebuf[7]);
			I("[Himax]:0x8D[8] = 0x%x\n",valuebuf[8]);

			cmdbuf[0] = 0xE9;
			cmdbuf[1] = 0x00;
			cmdbuf[2] = 0x06;
			ret = i2c_himax_master_write(private_ts->client, cmdbuf, 3, DEFAULT_RETRY_CNT);
			if (ret < 0) {
				E("[Himax]:write sernsor to self-test mode failed line: %d \n",__LINE__);
			}
			udelay(100);

			ret = i2c_himax_read(private_ts->client, 0xE9, valuebuf, 3, DEFAULT_RETRY_CNT);
			if (ret < 0) {
				E("[Himax]:read 0xE9 failed line: %d \n",__LINE__);
			}
			I("[Himax]:0xE9[0] = 0x%x\n",valuebuf[0]);
			I("[Himax]:0xE9[1] = 0x%x\n",valuebuf[1]);
			I("[Himax]:0xE9[2] = 0x%x\n",valuebuf[2]);

			cmdbuf[0] = HX_CMD_TSSON;
			ret = i2c_himax_master_write(private_ts->client, cmdbuf, 1, DEFAULT_RETRY_CNT);
			if (ret < 0) {
				E("[Himax]:write HX_CMD_TSSON failed line: %d \n",__LINE__);
			}
			hr_msleep(1500); 

			cmdbuf[0] = HX_CMD_TSSOFF;
			ret = i2c_himax_master_write(private_ts->client, cmdbuf, 1, DEFAULT_RETRY_CNT);
			if (ret < 0) {
				E("[Himax]:write TSSOFF failed line: %d \n",__LINE__);
			}
			hr_msleep(120); 

			memset(valuebuf, 0x00 , 16);
			ret = i2c_himax_read(private_ts->client, HX_CMD_SELFTEST_BUFFER, valuebuf, 16, DEFAULT_RETRY_CNT);

			if (ret < 0) {
				E("[Himax]:read HX_CMD_FW_VERSION_ID failed line: %d \n",__LINE__);
			} else {
				if (valuebuf[0]==0xAA)
				{
					I("[Himax]: self-test pass\n");
					pf_value = 0x0;
				}
				else
				{
					E("[Himax]: self-test fail\n");
					pf_value = 0x1;
					for(i=0;i<16;i++)
					{
						I("[Himax]:0x8D buff[%d] = 0x%x\n",i,valuebuf[i]);
					}
				}
			}
			hr_msleep(120); 

			cmdbuf[0] = 0xE9;
			cmdbuf[1] = 0x00;
			cmdbuf[2] = 0x00;
			ret = i2c_himax_master_write(private_ts->client, cmdbuf, 3, DEFAULT_RETRY_CNT);
			if (ret < 0) {
				E("[Himax]:write sensor to normal mode failed line: %d \n",__LINE__);
			}
			hr_msleep(120); 

			cmdbuf[0] = HX_CMD_TSSON;
			ret = i2c_himax_master_write(private_ts->client, cmdbuf, 1, DEFAULT_RETRY_CNT);
			if (ret < 0) {
				E("[Himax]:write HX_CMD_TSSON failed line: %d \n",__LINE__);
			}
			hr_msleep(120); 
		} else if (IC_TYPE == HX_85XX_D_SERIES_PWON) {
			cmdbuf[0] = 0x06;
			i2c_himax_write(private_ts->client, 0x91,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);
			hr_msleep(120);

			cmdbuf[0] = 0x00;
			i2c_himax_write(private_ts->client, 0xD7,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);
			hr_msleep(120);

			i2c_himax_write(private_ts->client, 0x83,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
			hr_msleep(120);

			i2c_himax_write(private_ts->client, 0x81,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
			hr_msleep(2000);

			i2c_himax_write(private_ts->client, 0x82,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
			hr_msleep(120);

			i2c_himax_write(private_ts->client, 0x80,&cmdbuf[0], 0, DEFAULT_RETRY_CNT);
			hr_msleep(120);

			cmdbuf[0] = 0x01;
			i2c_himax_write(private_ts->client, 0xD7,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);
			hr_msleep(120);

			cmdbuf[0] = 0x00;
			i2c_himax_write(private_ts->client, 0x91,&cmdbuf[0], 1, DEFAULT_RETRY_CNT);
			hr_msleep(120);

			i2c_himax_read(private_ts->client, 0xB2, valuebuf, 8, DEFAULT_RETRY_CNT);
			hr_msleep(10);

			for(i=0;i<8;i++) {
				I("[Himax]: After slf test 0xB2 buff_back[%d] = 0x%x\n",i,valuebuf[i]);
			}

			hr_msleep(30);

			if (valuebuf[0]==0xAA) {
				I("[Himax]: self-test pass\n");
				pf_value = 0x0;
			} else {
				E("[Himax]: self-test fail\n");
				pf_value = 0x1;
			}
		}
		return pf_value;
	}

	static DEVICE_ATTR(tp_self_test, (S_IWUSR|S_IRUGO), himax_chip_self_test_function, NULL);
	#endif
	

static ssize_t touch_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct himax_ts_data *ts_data;
	ts_data = private_ts;

	ret += sprintf(buf, "%s_FW:%#x_CFG:%#x_SensorId:%#x\n", HIMAX8528_NAME,
			ts_data->vendor_fw_ver, ts_data->vendor_config_ver, ts_data->vendor_sensor_id);

	return ret;
}

static DEVICE_ATTR(vendor, (S_IRUGO), touch_vendor_show, NULL);

static ssize_t touch_attn_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct himax_ts_data *ts_data;
	ts_data = private_ts;

	sprintf(buf, "attn = %x\n", gpio_get_value(ts_data->irq));
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(attn, (S_IRUGO), touch_attn_show, NULL);
static ssize_t himax_int_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *ts = private_ts;
	size_t count = 0;

	count += sprintf(buf + count, "%d ", ts->irq_enabled);
	count += sprintf(buf + count, "\n");

	return count;
}

static ssize_t himax_int_status_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct himax_ts_data *ts = private_ts;
	int value, ret=0;

	if (sysfs_streq(buf, "0"))
		value = false;
	else if (sysfs_streq(buf, "1"))
		value = true;
	else
		return -EINVAL;

	if (value) {
		ret = request_threaded_irq(ts->client->irq, NULL, himax_ts_thread,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, ts->client->name, ts);
		if (ret == 0) {
			ts->irq_enabled = 1;
			irq_enable_count = 1;
		}
	} else {
		himax_int_enable(0);
		free_irq(ts->client->irq, ts);
		ts->irq_enabled = 0;
	}

	return count;
}

static DEVICE_ATTR(enabled, (S_IWUSR|S_IRUGO),
	himax_int_status_show, himax_int_status_store);
static int himax_input_register(struct himax_ts_data *ts)
{
	int ret;
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		E("%s: Failed to allocate input device\n", __func__);
		return ret;
	}
	ts->input_dev->name = "himax-touchscreen";

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);

	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(KEY_APP_SWITCH, ts->input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	if (ts->protocol_type == PROTOCOL_TYPE_A) {
		ts->input_dev->mtsize = ts->nFinger_support;
		input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID,
		0, 3, 0, 0);
	} else {
		set_bit(MT_TOOL_FINGER, ts->input_dev->keybit);
		input_mt_init_slots(ts->input_dev, ts->nFinger_support);
	}

	I("input_set_abs_params: mix_x %d, max_x %d, min_y %d, max_y %d\n",
		ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_x_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,ts->pdata->abs_y_min, ts->pdata->abs_y_max, ts->pdata->abs_y_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE,ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,ts->pdata->abs_width_min, ts->pdata->abs_width_max, ts->pdata->abs_pressure_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE, 0, ((ts->pdata->abs_pressure_max << 16) | ts->pdata->abs_width_max), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION, 0, (BIT(31) | (ts->pdata->abs_x_max << 16) | ts->pdata->abs_y_max), 0, 0);

	return input_register_device(ts->input_dev);
}
static ssize_t himax_layout_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *ts = private_ts;
	size_t count = 0;

	count += sprintf(buf + count, "%d ", ts->pdata->abs_x_min);
	count += sprintf(buf + count, "%d ", ts->pdata->abs_x_max);
	count += sprintf(buf + count, "%d ", ts->pdata->abs_y_min);
	count += sprintf(buf + count, "%d ", ts->pdata->abs_y_max);
	count += sprintf(buf + count, "\n");

	return count;
}

static ssize_t himax_layout_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct himax_ts_data *ts = private_ts;
	char buf_tmp[5];
	int i = 0, j = 0, k = 0, ret;
	unsigned long value;
	int layout[4] = {0};

	for (i = 0; i < 20; i++) {
		if (buf[i] == ',' || buf[i] == '\n') {
			memset(buf_tmp, 0x0, sizeof(buf_tmp));
			if (i - j <= 5)
				memcpy(buf_tmp, buf + j, i - j);
			else {
				I("buffer size is over 5 char\n");
				return count;
			}
			j = i + 1;
			if (k < 4) {
				ret = strict_strtol(buf_tmp, 10, &value);
				layout[k++] = value;
			}
		}
	}
	if (k == 4) {
		ts->pdata->abs_x_min=layout[0];
		ts->pdata->abs_x_max=layout[1];
		ts->pdata->abs_y_min=layout[2];
		ts->pdata->abs_y_max=layout[3];
		I("%d, %d, %d, %d\n",
			ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);
		input_unregister_device(ts->input_dev);
		himax_input_register(ts);
	} else
		I("ERR@%d, %d, %d, %d\n",
			ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);
	return count;
}

static DEVICE_ATTR(layout, (S_IWUSR|S_IRUGO),
	himax_layout_show, himax_layout_store);
static ssize_t himax_debug_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *ts_data;
	size_t count = 0;
	ts_data = private_ts;

	count += sprintf(buf, "%d\n", ts_data->debug_log_level);

	return count;
}
#define SHIFTBITS 5
static ssize_t himax_debug_level_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct himax_ts_data *ts;
	char buf_tmp[11];
	int i;
	ts = private_ts;
	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memcpy(buf_tmp, buf, count);

	ts->debug_log_level = 0;
	for(i=0; i<count-1; i++)
	{
		if( buf_tmp[i]>='0' && buf_tmp[i]<='9' )
			ts->debug_log_level |= (buf_tmp[i]-'0');
		else if( buf_tmp[i]>='A' && buf_tmp[i]<='F' )
			ts->debug_log_level |= (buf_tmp[i]-'A'+10);
		else if( buf_tmp[i]>='a' && buf_tmp[i]<='f' )
			ts->debug_log_level |= (buf_tmp[i]-'a'+10);

		if(i!=count-2)
			ts->debug_log_level <<= 4;
	}

	if (ts->debug_log_level & BIT(3)) {
		if (ts->pdata->screenWidth > 0 && ts->pdata->screenHeight > 0 &&
		 (ts->pdata->abs_x_max - ts->pdata->abs_x_min) > 0 &&
		 (ts->pdata->abs_y_max - ts->pdata->abs_y_min) > 0) {
			ts->widthFactor = (ts->pdata->screenWidth << SHIFTBITS)/(ts->pdata->abs_x_max - ts->pdata->abs_x_min);
			ts->heightFactor = (ts->pdata->screenHeight << SHIFTBITS)/(ts->pdata->abs_y_max - ts->pdata->abs_y_min);
			if (ts->widthFactor > 0 && ts->heightFactor > 0)
				ts->useScreenRes = 1;
			else {
				ts->heightFactor = 0;
				ts->widthFactor = 0;
				ts->useScreenRes = 0;
			}
		} else
			I("Enable finger debug with raw position mode!\n");
	} else {
		ts->useScreenRes = 0;
		ts->widthFactor = 0;
		ts->heightFactor = 0;
	}

	return count;
}

static DEVICE_ATTR(debug_level, (S_IWUSR|S_IRUGO),
	himax_debug_level_show, himax_debug_level_dump);

static ssize_t himax_set_event_htc(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct himax_ts_data *ts_data;
	unsigned long result = 0;
	ts_data = private_ts;
	if (!strict_strtoul(buf, 10, &result)) {
		ts_data->event_htc_enable_type = result;
		I("htc event enable = %d\n", ts_data->event_htc_enable_type);
	}
	return count;
}

static ssize_t himax_show_event_htc(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *ts_data;
	ts_data = private_ts;
	return sprintf(buf, "htc event enable = %d\n", ts_data->event_htc_enable_type);
}
static DEVICE_ATTR(htc_event, (S_IWUSR|S_IRUGO), himax_show_event_htc, himax_set_event_htc);

static void doHWreset(void)
{
	struct himax_ts_data *ts = private_ts;
	int ret = 0;
	if (ts->rst_gpio) {
		if (ts->use_irq)
			himax_int_enable(0);
		else {
			hrtimer_cancel(&ts->timer);
			ret = cancel_work_sync(&ts->work);
		}

		I("%s: Now reset the Touch chip.\n", __func__);

		gpio_direction_output(ts->rst_gpio, 0);
		hr_msleep(30);
		gpio_direction_output(ts->rst_gpio, 1);
		hr_msleep(30);

		himax_loadSensorConfig(private_ts->client,private_ts->pdata,true);

		if (ts->use_irq)
			himax_int_enable(1);
		else
			hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
}

#ifdef FAKE_EVENT

static int X_fake_S = 50;
static int Y_fake_S = 600;
static int X_fake_E = 800;
static int Y_fake_E = 600;
static int dx_fake = 2;
static int dy_fake;
static unsigned long report_time = 10000000;

static enum hrtimer_restart himax_ts_timer_fake_event_func(struct hrtimer *timer)
{
	struct himax_ts_data *ts = container_of(timer, struct himax_ts_data, timer);

	static int i;
	static int X_tmp;
	static int Y_tmp;

	if (!i) {
		X_tmp = X_fake_S;
		Y_tmp = Y_fake_S;
		i++;
	}
	if ((dx_fake > 0 ? X_tmp <= X_fake_E : dx_fake ? X_tmp >= X_fake_E : 0) ||
		(dy_fake > 0 ? Y_tmp <= Y_fake_E : dy_fake ? Y_tmp >= Y_fake_E : 0)) {
		if (ts->protocol_type == PROTOCOL_TYPE_A) {
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 10);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 10);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 5);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, X_tmp);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, Y_tmp);
			input_mt_sync(ts->input_dev);
		} else if (ts->protocol_type == PROTOCOL_TYPE_B) {
			input_mt_slot(ts->input_dev, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 10);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 10);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 5);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, X_tmp);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, Y_tmp);
		}
		input_report_key(ts->input_dev, BTN_TOUCH, 1);
		input_sync(ts->input_dev);
		X_tmp += dx_fake;
		Y_tmp += dy_fake;
		hrtimer_start(&ts->timer, ktime_set(0, report_time), HRTIMER_MODE_REL);
	} else {
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		if (ts->protocol_type == PROTOCOL_TYPE_A) {
			input_mt_sync(ts->input_dev);
		} else if (ts->protocol_type == PROTOCOL_TYPE_B) {
			input_mt_slot(ts->input_dev, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		}
		input_sync(ts->input_dev);
		i = 0;
		I("End of fake event\n");
	}

	return HRTIMER_NORESTART;
}

static ssize_t himax_fake_event_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *ts = private_ts;
	static uint8_t i;
	size_t count = 0;
	int Xres = 0, Yres = 0, Fx = 0, Fy = 0;

	if (!i) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = himax_ts_timer_fake_event_func;
		I("hrtimer_init\n");
		i++;
	}
	count += sprintf(buf + count, "%d,%d,%d,%d,%d,%d,%lu\n",
		ts->fake_X_S, ts->fake_Y_S, ts->fake_X_E,
		ts->fake_Y_E, dx_fake, dy_fake, report_time/1000000);
	if (ts->pdata->screenWidth != 0 && ts->pdata->screenWidth != 0) {
		Xres = ts->pdata->abs_x_max - ts->pdata->abs_x_min;
		Yres = ts->pdata->abs_y_max - ts->pdata->abs_y_min;
		Fx = Xres / ts->pdata->screenWidth;
		Fy = Yres / ts->pdata->screenHeight;
		X_fake_S = ts->fake_X_S * Fx + ts->pdata->abs_x_min;
		Y_fake_S = ts->fake_Y_S * Fy + ts->pdata->abs_y_min;
		X_fake_E = ts->fake_X_E * Fx + ts->pdata->abs_x_min;
		Y_fake_E = ts->fake_Y_E * Fy + ts->pdata->abs_y_min;
	}

	if (dx_fake & dy_fake)
		count += sprintf(buf + count, "dx_fake or dy_fake should one value need to be zero\n");
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	return count;
}

static ssize_t himax_fake_event_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct himax_ts_data *ts_data;
	long value;
	char buf_tmp[5];
	int i = 0, j = 0, k = 0, ret;
	ts_data = private_ts;

	while (1) {
		if (buf[i] == ',' || buf[i] == '\n') {
			memset(buf_tmp, 0x0, sizeof(buf_tmp));
			if (i - j <= 5)
				memcpy(buf_tmp, buf + j, i - j);
			else {
				E("buffer size is over 5 char\n");
				return count;
			}
			j = i + 1;

			ret = strict_strtol(buf_tmp, 10, &value);

			switch (k) {
			case 0:
				ts_data->fake_X_S = value;
				break;
			case 1:
				ts_data->fake_Y_S = value;
				break;
			case 2:
				ts_data->fake_X_E = value;
				break;
			case 3:
				ts_data->fake_Y_E = value;
				break;
			case 4:
				dx_fake = value;
				break;
			case 5:
				dy_fake = value;
				break;
			case 6:
				report_time = value*1000000;
			default:
				break;
			}
			k++;
		}
		if (buf[i] == '\n')
			break;
		i++;
	}

	return count;

}

static DEVICE_ATTR(fake_event, (S_IWUSR|S_IRUGO),
	himax_fake_event_show, himax_fake_event_store);

#endif

enum SR_REG_STATE{
	ALLOCATE_DEV_FAIL = -2,
	REGISTER_DEV_FAIL,
	SUCCESS,
};


static int register_sr_touch_device(void)
{
	struct himax_ts_data *ts = private_ts;

	ts->sr_input_dev = input_allocate_device();

	if (ts->sr_input_dev == NULL) {
		E("[SR]%s: Failed to allocate SR input device\n", __func__);
		return ALLOCATE_DEV_FAIL;
	}


	ts->sr_input_dev->name = "sr_touchscreen";
	set_bit(EV_SYN, ts->sr_input_dev->evbit);
	set_bit(EV_ABS, ts->sr_input_dev->evbit);
	set_bit(EV_KEY, ts->sr_input_dev->evbit);

	set_bit(KEY_BACK, ts->sr_input_dev->keybit);
	set_bit(KEY_HOME, ts->sr_input_dev->keybit);
	set_bit(KEY_MENU, ts->sr_input_dev->keybit);
	set_bit(KEY_SEARCH, ts->sr_input_dev->keybit);
	set_bit(BTN_TOUCH, ts->sr_input_dev->keybit);
	set_bit(KEY_APP_SWITCH, ts->sr_input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->sr_input_dev->propbit);
	ts->sr_input_dev->mtsize = ts->nFinger_support;
	input_set_abs_params(ts->sr_input_dev, ABS_MT_TRACKING_ID,
		0, 3, 0, 0);
	I("[SR]input_set_abs_params: mix_x %d, max_x %d,"
		" min_y %d, max_y %d\n", ts->pdata->abs_x_min,
		 ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);

	input_set_abs_params(ts->sr_input_dev, ABS_MT_POSITION_X,ts->pdata->abs_x_min, ts->pdata->abs_x_max, 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_POSITION_Y,ts->pdata->abs_y_min, ts->pdata->abs_y_max, 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_TOUCH_MAJOR,ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_PRESSURE,ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_WIDTH_MAJOR,ts->pdata->abs_width_min, ts->pdata->abs_width_max, 0, 0);

	if (input_register_device(ts->sr_input_dev)) {
		input_free_device(ts->sr_input_dev);
		E("[SR]%s: Unable to register %s input device\n",
			__func__, ts->sr_input_dev->name);
		return REGISTER_DEV_FAIL;
	}
	return SUCCESS;
}

static ssize_t himax_get_en_sr(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *ts = private_ts;
	size_t count = 0;

	if (ts->sr_input_dev)
		{
			count += sprintf(buf + count, "%s ", ts->sr_input_dev->name);
			count += sprintf(buf + count, "\n");
		}
	else
		count += sprintf(buf + count, "0\n");


	return count;
}

static ssize_t himax_set_en_sr(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct himax_ts_data *ts_data;
	ts_data = private_ts;
	if (buf[0]) {
		if (ts_data->sr_input_dev)
			I("[SR]%s: SR device already exist!\n", __func__);
		else
			I("[SR]%s: SR touch device enable result:%X\n", __func__, register_sr_touch_device());
	}
	return count;
}

static DEVICE_ATTR(sr_en, (S_IWUSR|S_IRUGO), himax_get_en_sr, himax_set_en_sr);
#if 0
#define HMX_FW_NAME "tp_HMX.img"
#include <linux/async.h>
#include <linux/wakelock.h>
static struct wake_lock flash_wake_lock;
#define FLASH_RETRY_TIMES 3
static void doFirmwareUpdate(void *unused, async_cookie_t cookie)
{
	struct himax_ts_data *ts = private_ts;
	int FileLength = ts->fw_size;
	int ret;
	uint8_t buf[2] = {0};
	uint32_t suspend_status = 0;

	if (fts_ctpm_fw_upgrade_with_sys_fs(ts->fw_data_start, FileLength) == 0)
		{
			E("%s: TP upgrade error, line: %d\n", __func__, __LINE__);
			fw_update_complete = false;
		}
	else
		{
			I("%s: TP upgrade OK, line: %d\n", __func__, __LINE__);
			fw_update_complete = true;
		}

		himax_HW_reset();
		himax_loadSensorConfig(ts->client,ts->pdata,true);

	suspend_status = atomic_read(&ts->suspend_mode);

	if (ts->suspend_flag_b4_flash ^ suspend_status) {
		if (suspend_status) {
			
			
			buf[0] = HX_CMD_TSSOFF;
			ret = i2c_himax_master_write(ts->client, buf, 1, HIMAX_I2C_RETRY_TIMES);
			if (ret < 0)
			{
				E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts->client->addr);
			}
			hr_msleep(30);
			buf[0] = HX_CMD_TSSLPIN;
			ret = i2c_himax_master_write(ts->client, buf, 1, HIMAX_I2C_RETRY_TIMES);
			if (ret < 0)
			{
				E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts->client->addr);
			}
			hr_msleep(30);
			buf[0] = HX_CMD_SETDEEPSTB;
			buf[1] = 0x01;
			ret = i2c_himax_master_write(ts->client, buf, 2, HIMAX_I2C_RETRY_TIMES);
			if (ret < 0)
			{
				E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts->client->addr);
			}
			if (ts->pdata->powerOff3V3 && ts->pdata->power)
				ts->pdata->power(0);
		} else {
			
			
			buf[0] = HX_CMD_SETDEEPSTB; 
			buf[1] = 0x00;
			ret = i2c_himax_master_write(ts->client, buf, 2, HIMAX_I2C_RETRY_TIMES);
			if (ret < 0)
			{
				E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts->client->addr);
			}
			hr_msleep(5);
			
			i2c_himax_write_command(ts->client, 0x83, HIMAX_I2C_RETRY_TIMES);
			hr_msleep(30);
			i2c_himax_write_command(ts->client, 0x81, HIMAX_I2C_RETRY_TIMES);
			ts->just_resume = 1;
			if (ts->use_irq)
				himax_int_enable(1);
			else
				hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		}
	} else {
		if (suspend_status) {
			if (ts->pdata->powerOff3V3 && ts->pdata->power)
				ts->pdata->power(0);
		} else {
			if (ts->use_irq)
				himax_int_enable(1);
			else
				hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		}
	}
	wake_unlock(&flash_wake_lock);
	wake_lock_destroy(&flash_wake_lock);
	atomic_set(&ts->in_flash, 0);
	I("[FW]firmware flash process complete!\n");
}
#include <linux/string.h>
#include <linux/kthread.h>
static int updateFirmware(void *arg)
{
	int ret = 0;
	u8 *ver_string = 0;
	char version[10] = { 0 };
	uint32_t fw_version = 0;
	struct himax_ts_data *ts = (struct himax_ts_data *)arg;

	
	ret = request_firmware(&ts->fw, HMX_FW_NAME, &ts->client->dev);
	if (ts->fw == NULL) {
		E("[FW] No firmware file, ignored firmware update\n");
		goto HMX_FW_REQUEST_FAILURE;
	} else if (ret) {
		E("[FW] Request_firmware failed, ret = %d\n", ret);
		goto HMX_FW_REQUEST_FAILURE;
	}

	
	if (!strncmp(ts->fw->data, "TP_", 3)) {
		
		ver_string = strchr(ts->fw->data + 3, '_');
		ts->fw_data_start = strchr(ts->fw->data + 3, '\n');
		memcpy(version, ver_string + 1, ts->fw_data_start++ - ver_string - 1);
		ret = strict_strtoul(version, 16, (unsigned long *)&fw_version);

		if (ret < 0) {
			E("[FW] TP tag parse failed %s, %d\n", version, fw_version);
			goto HMX_FW_HEADER_ILLEGAL;
		}

		if (fw_version == ts->pdata->fw_version) {
			E("[FW] firmware version is the same in chip %X, in file %X, ignore\n",
				ts->pdata->fw_version, fw_version);
			goto HMX_FW_SAME_VER_IGNORE;
		}
		ts->fw_size = ts->fw->size - (ts->fw_data_start - ts->fw->data);
	} else {
		I("[FW] no touch firmware header, force write\n");
		ts->fw_data_start = (u8 *)ts->fw->data;
		ts->fw_size = ts->fw->size;
	}

	ts->suspend_flag_b4_flash = atomic_read(&ts->suspend_mode);
	atomic_set(&ts->in_flash, 1);
	if (!ts->suspend_flag_b4_flash) {
		if (ts->use_irq) {
				himax_int_enable(0);
		} else {
			hrtimer_cancel(&ts->timer);
			ret = cancel_work_sync(&ts->work);
		}
	} else { 
		if (ts->pdata->powerOff3V3 && ts->pdata->power)
				ts->pdata->power(1);
	}
	wake_lock_init(&flash_wake_lock, WAKE_LOCK_SUSPEND, ts->client->name);
	wake_lock(&flash_wake_lock);
	if (wake_lock_active(&flash_wake_lock))
		I("[FW] wake lock successfully acquired!\n");
	else {
		E("[FW] failed to hold wake lock, give up.....\n");
		goto WAKE_LOCK_ACQUIRE_FAILED;
	}

	
	doHWreset();

	async_schedule(doFirmwareUpdate, NULL);

	return 0;

WAKE_LOCK_ACQUIRE_FAILED:
	wake_lock_destroy(&flash_wake_lock);
	if (ts->use_irq)
		himax_int_enable(1);
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	atomic_set(&ts->in_flash, 0);
HMX_FW_SAME_VER_IGNORE:
HMX_FW_HEADER_ILLEGAL:
	release_firmware(ts->fw);
HMX_FW_REQUEST_FAILURE:
	return -1;
}
#endif

static struct kobject *android_touch_kobj;

static int himax_touch_sysfs_init(void)
{
	int ret;
	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		E("%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr);
	if (ret) {
		E("%s: create_file debug_level failed\n", __func__);
		return ret;
	}

	#ifdef HX_TP_SYS_REGISTER
	register_command = 0;
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_register.attr);
	if (ret)
	{
		E("create_file register failed\n");
		return ret;
	}
	#endif

	#ifdef HX_TP_SYS_DIAG
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_diag.attr);
	if (ret)
	{
		E("sysfs_create_file failed\n");
		return ret;
	}
	#endif

	#ifdef HX_TP_SYS_DEBUG
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug.attr);
	if (ret)
	{
		E("create_file debug_level failed\n");
		return ret;
	}
	#endif

	#ifdef HX_TP_SYS_FLASH_DUMP
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_flash_dump.attr);
	if (ret)
	{
		E("sysfs_create_file failed\n");
		return ret;
	}
	#endif

	#ifdef HX_TP_SYS_SELF_TEST
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_tp_self_test.attr);
	if (ret)
	{
		E("[Himax]: sysfs_create_file dev_attr_tp_self_test failed\n");
		return ret;
	}
	#endif

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		E("%s: sysfs_create_file failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_htc_event.attr);
	if (ret) {
		E("%s: sysfs_create_file failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_reset.attr);
	if (ret) {
		E("%s: sysfs_create_file failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_attn.attr);
	if (ret) {
		E("%s: sysfs_create_file failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_enabled.attr);
	if (ret) {
		E("%s: sysfs_create_file failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_layout.attr);
	if (ret) {
		E("%s: sysfs_create_file failed\n", __func__);
		return ret;
	}

#ifdef FAKE_EVENT
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_fake_event.attr);
	if (ret) {
		E("%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
#endif

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_sr_en.attr);
	if (ret) {
		E("[SR]%s: sysfs_create_file failed\n", __func__);
		return ret;
	}

	return 0 ;
}

static void himax_touch_sysfs_deinit(void)
{
	#ifdef HX_TP_SYS_DIAG
	sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);
	#endif

	#ifdef HX_TP_SYS_FLASH_DUMP
	sysfs_remove_file(android_touch_kobj, &dev_attr_flash_dump.attr);
	#endif

	sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);

	#ifdef HX_TP_SYS_REGISTER
	sysfs_remove_file(android_touch_kobj, &dev_attr_register.attr);
	#endif

	#ifdef HX_TP_SYS_DEBUG
	sysfs_remove_file(android_touch_kobj, &dev_attr_debug.attr);
	#endif

	#ifdef HX_TP_SYS_SELF_TEST
	sysfs_remove_file(android_touch_kobj, &dev_attr_tp_self_test.attr);
	#endif

	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_htc_event.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_reset.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_attn.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_enabled.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_layout.attr);
#ifdef FAKE_EVENT
	sysfs_remove_file(android_touch_kobj, &dev_attr_fake_event.attr);
#endif
	sysfs_remove_file(android_touch_kobj, &dev_attr_sr_en.attr);
	kobject_del(android_touch_kobj);
}

#ifdef CONFIG_OF
static int himax_parse_config(struct himax_ts_data *ts, struct himax_i2c_platform_data_config_type28 *pdata)
{
	struct himax_config *cfg_table;
	struct device_node *node, *pp = NULL;
	struct property *prop;
	uint8_t cnt = 0, i = 0;
	u32 data = 0;
	uint32_t coords[4] = {0};
	int len = 0;
	char str[6]={0};

	node = ts->client->dev.of_node;
	if (node == NULL) {
		E(" %s, can't find device_node", __func__);
		return -ENODEV;
	}

	while ((pp = of_get_next_child(node, pp)))
		cnt++;

	if (!cnt)
		return -ENODEV;

	cfg_table = kzalloc(cnt * (sizeof *cfg_table), GFP_KERNEL);
	if (!cfg_table)
		return -ENOMEM;

	pp = NULL;
	while ((pp = of_get_next_child(node, pp))) {
		if (of_property_read_u32(pp, "default_cfg", &data) == 0)
			cfg_table[i].default_cfg = data;

		if (of_property_read_u32(pp, "sensor_id", &data) == 0)
			cfg_table[i].sensor_id = (data);

		if (of_property_read_u32(pp, "fw_ver", &data) == 0)
			cfg_table[i].fw_ver = data;

		if (of_property_read_u32_array(pp, "himax,tw-coords", coords, 4) == 0) {
			cfg_table[i].tw_x_min = coords[0], cfg_table[i].tw_x_max = coords[1];	
			cfg_table[i].tw_y_min = coords[2], cfg_table[i].tw_y_max = coords[3];	
		}

		if (of_property_read_u32_array(pp, "himax,pl-coords", coords, 4) == 0) {
			cfg_table[i].pl_x_min = coords[0], cfg_table[i].pl_x_max = coords[1];	
			cfg_table[i].pl_y_min = coords[2], cfg_table[i].pl_y_max = coords[3];	
		}

		prop = of_find_property(pp, "c1", &len);
		if ((!prop)||(!len)) {
			strcpy(str,"c1");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c1, prop->value, len);
		prop = of_find_property(pp, "c2", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c2");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c2, prop->value, len);
		prop = of_find_property(pp, "c3", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c3");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c3, prop->value, len);
		prop = of_find_property(pp, "c4", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c4");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c4, prop->value, len);
		prop = of_find_property(pp, "c5", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c5");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c5, prop->value, len);
		prop = of_find_property(pp, "c6", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c6");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c6, prop->value, len);
		prop = of_find_property(pp, "c7", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c7");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c7, prop->value, len);
		prop = of_find_property(pp, "c8", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c8");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c8, prop->value, len);
		prop = of_find_property(pp, "c9", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c9");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c9, prop->value, len);
		prop = of_find_property(pp, "c10", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c10");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c10, prop->value, len);
		prop = of_find_property(pp, "c11", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c11");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c11, prop->value, len);
		prop = of_find_property(pp, "c12", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c12");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c12, prop->value, len);
		prop = of_find_property(pp, "c13", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c13");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c13, prop->value, len);
		prop = of_find_property(pp, "c14", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c14");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c14, prop->value, len);
		prop = of_find_property(pp, "c15", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c15");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c15, prop->value, len);
		prop = of_find_property(pp, "c16", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c16");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c16, prop->value, len);
		prop = of_find_property(pp, "c17", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c17");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c17, prop->value, len);
		prop = of_find_property(pp, "c18", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c18");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c18, prop->value, len);
		prop = of_find_property(pp, "c19", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c19");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c19, prop->value, len);
		prop = of_find_property(pp, "c20", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c20");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c20, prop->value, len);
		prop = of_find_property(pp, "c21", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c21");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c21, prop->value, len);
		prop = of_find_property(pp, "c22", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c22");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c22, prop->value, len);
		prop = of_find_property(pp, "c23", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c23");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c23, prop->value, len);
		prop = of_find_property(pp, "c24", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c24");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c24, prop->value, len);
		prop = of_find_property(pp, "c25", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c25");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c25, prop->value, len);
		prop = of_find_property(pp, "c26", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c26");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c26, prop->value, len);
		prop = of_find_property(pp, "c27", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c27");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c27, prop->value, len);
		prop = of_find_property(pp, "c28", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c28");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c28, prop->value, len);
		prop = of_find_property(pp, "c29", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c29");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c29, prop->value, len);
		prop = of_find_property(pp, "c30", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c30");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c30, prop->value, len);
		prop = of_find_property(pp, "c31", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c31");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c31, prop->value, len);
		prop = of_find_property(pp, "c32", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c32");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c32, prop->value, len);
		prop = of_find_property(pp, "c33", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c33");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c33, prop->value, len);
		prop = of_find_property(pp, "c34", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c34");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c34, prop->value, len);
		prop = of_find_property(pp, "c35", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c35");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c35, prop->value, len);
		prop = of_find_property(pp, "c36", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c36");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c36, prop->value, len);
		#if 1
		I(" config version=[%02x]", cfg_table[i].c36[1]);
		#endif
		prop = of_find_property(pp, "c37", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c37");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c37, prop->value, len);
		prop = of_find_property(pp, "c38", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c38");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c38, prop->value, len);
		prop = of_find_property(pp, "c39", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c39");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c39, prop->value, len);
		prop = of_find_property(pp, "c40", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c40");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c40, prop->value, len);
		prop = of_find_property(pp, "c41", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c41");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c41, prop->value, len);
		prop = of_find_property(pp, "c42", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c42");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c42, prop->value, len);
		prop = of_find_property(pp, "c43_1", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c43_1");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c43_1, prop->value, len);
		prop = of_find_property(pp, "c43_2", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c43_2");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c43_2, prop->value, len);
		prop = of_find_property(pp, "c44_1", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c44_1");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c44_1, prop->value, len);
		prop = of_find_property(pp, "c44_2", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c44_2");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c44_2, prop->value, len);
		prop = of_find_property(pp, "c45", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c45");
			goto of_find_property_error;
			}		
		memcpy(cfg_table[i].c45, prop->value, len);
				
	#if 1
		I(" DT#%d-def_cfg:%d,id:%05x, FW:%d, len:%d,", i,
			cfg_table[i].default_cfg, cfg_table[i].sensor_id,
			cfg_table[i].fw_ver, cfg_table[i].length);
		
	#endif
		i++;
	of_find_property_error:
	if (!prop) {
		D(" %s:Looking up %s property in node %s failed",
			__func__, str, pp->full_name);
		return -ENODEV;
	} else if (!len) {
		D(" %s:Invalid length of configuration data in %s\n",
			__func__, str);
		return -EINVAL;
		}	
	}
	
	i = 0;	
	while (cfg_table[i].fw_ver> ts->vendor_fw_ver) {
		i++;
	}
	if(cfg_table[i].default_cfg!=0)
		goto startloadconf;
	while (cfg_table[i].sensor_id > 0 && (cfg_table[i].sensor_id !=  ts->vendor_sensor_id)) {
		I(" id:%#x!=%#x, (i++)",cfg_table[i].sensor_id, ts->vendor_sensor_id);
		i++;
	}
	startloadconf:
	if (i <= cnt) {
		I(" DT-%s cfg idx(%d) in cnt(%d)", __func__, i, cnt);
		pdata->version 		= cfg_table[i].fw_ver;
		pdata->tw_id      	= cfg_table[i].sensor_id;
		
		memcpy(pdata->c1, cfg_table[i].c1,sizeof(pdata->c1));
		memcpy(pdata->c2, cfg_table[i].c2,sizeof(pdata->c2));
		memcpy(pdata->c3, cfg_table[i].c3,sizeof(pdata->c3));
		memcpy(pdata->c4, cfg_table[i].c4,sizeof(pdata->c4));
		memcpy(pdata->c5, cfg_table[i].c5,sizeof(pdata->c5));
		memcpy(pdata->c6, cfg_table[i].c6,sizeof(pdata->c6));
		memcpy(pdata->c7, cfg_table[i].c7,sizeof(pdata->c7));
		memcpy(pdata->c8, cfg_table[i].c8,sizeof(pdata->c8));
		memcpy(pdata->c9, cfg_table[i].c9,sizeof(pdata->c9));
		memcpy(pdata->c10, cfg_table[i].c10,sizeof(pdata->c10));
		memcpy(pdata->c11, cfg_table[i].c11,sizeof(pdata->c11));
		memcpy(pdata->c12, cfg_table[i].c12,sizeof(pdata->c12));
		memcpy(pdata->c13, cfg_table[i].c13,sizeof(pdata->c13));
		memcpy(pdata->c14, cfg_table[i].c14,sizeof(pdata->c14));
		memcpy(pdata->c15, cfg_table[i].c15,sizeof(pdata->c15));
		memcpy(pdata->c16, cfg_table[i].c16,sizeof(pdata->c16));
		memcpy(pdata->c17, cfg_table[i].c17,sizeof(pdata->c17));
		memcpy(pdata->c18, cfg_table[i].c18,sizeof(pdata->c18));
		memcpy(pdata->c19, cfg_table[i].c19,sizeof(pdata->c19));
		memcpy(pdata->c20, cfg_table[i].c20,sizeof(pdata->c20));
		memcpy(pdata->c21, cfg_table[i].c21,sizeof(pdata->c21));
		memcpy(pdata->c22, cfg_table[i].c22,sizeof(pdata->c22));
		memcpy(pdata->c23, cfg_table[i].c23,sizeof(pdata->c23));
		memcpy(pdata->c24, cfg_table[i].c24,sizeof(pdata->c24));
		memcpy(pdata->c25, cfg_table[i].c25,sizeof(pdata->c25));
		memcpy(pdata->c26, cfg_table[i].c26,sizeof(pdata->c26));
		memcpy(pdata->c27, cfg_table[i].c27,sizeof(pdata->c27));
		memcpy(pdata->c28, cfg_table[i].c28,sizeof(pdata->c28));
		memcpy(pdata->c29, cfg_table[i].c29,sizeof(pdata->c29));
		memcpy(pdata->c30, cfg_table[i].c30,sizeof(pdata->c30));
		memcpy(pdata->c31, cfg_table[i].c31,sizeof(pdata->c31));
		memcpy(pdata->c32, cfg_table[i].c32,sizeof(pdata->c32));
		memcpy(pdata->c33, cfg_table[i].c33,sizeof(pdata->c33));
		memcpy(pdata->c34, cfg_table[i].c34,sizeof(pdata->c34));
		memcpy(pdata->c35, cfg_table[i].c35,sizeof(pdata->c35));
		memcpy(pdata->c36, cfg_table[i].c36,sizeof(pdata->c36));
		memcpy(pdata->c37, cfg_table[i].c37,sizeof(pdata->c37));
		memcpy(pdata->c38, cfg_table[i].c38,sizeof(pdata->c38));
		memcpy(pdata->c39, cfg_table[i].c39,sizeof(pdata->c39));
		memcpy(pdata->c40, cfg_table[i].c40,sizeof(pdata->c40));
		memcpy(pdata->c41, cfg_table[i].c41,sizeof(pdata->c41));
		memcpy(pdata->c42, cfg_table[i].c42,sizeof(pdata->c42));
		memcpy(pdata->c43_1, cfg_table[i].c43_1,sizeof(pdata->c43_1));
		memcpy(pdata->c43_2, cfg_table[i].c43_2,sizeof(pdata->c43_2));
		memcpy(pdata->c44_1, cfg_table[i].c44_1,sizeof(pdata->c44_1));
		memcpy(pdata->c44_2, cfg_table[i].c44_2,sizeof(pdata->c44_2));
		memcpy(pdata->c45, cfg_table[i].c45,sizeof(pdata->c45));

		ts->tw_x_min = cfg_table[i].tw_x_min, ts->tw_x_max = cfg_table[i].tw_x_max;	
		ts->tw_y_min = cfg_table[i].tw_y_min, ts->tw_y_max = cfg_table[i].tw_y_max;	

		ts->pl_x_min = cfg_table[i].pl_x_min, ts->pl_x_max = cfg_table[i].pl_x_max;	
		ts->pl_y_min = cfg_table[i].pl_y_min, ts->pl_y_max = cfg_table[i].pl_y_max;	
	
		I(" DT#%d-def_cfg:%d,id:%05x, FW:%d, len:%d,", i,
			cfg_table[i].default_cfg, cfg_table[i].sensor_id,
			cfg_table[i].fw_ver, cfg_table[i].length);
		I(" DT-%s:tw-coords = %d, %d, %d, %d\n", __func__, ts->tw_x_min,
				ts->tw_x_max, ts->tw_y_min, ts->tw_y_max);
		I(" DT-%s:pl-coords = %d, %d, %d, %d\n", __func__, ts->pl_x_min,
				ts->pl_x_max, ts->pl_y_min, ts->pl_y_max);
		I(" config version=[%02x]", pdata->c36[1]);
	} else {
		E(" DT-%s cfg idx(%d) > cnt(%d)", __func__, i, cnt);
		return -EINVAL;
	}
	return 0;	
}

static void himax_vk_parser(struct device_node *dt,
				struct himax_i2c_platform_data *pdata)
{
	u32 data = 0;
	uint8_t cnt = 0, i = 0;
	uint32_t coords[4] = {0};
	struct device_node *node, *pp = NULL;
	struct himax_virtual_key *vk;

	node = of_parse_phandle(dt, "virtualkey", 0);
	if (node == NULL) {
		I(" DT-No vk info in DT");
		return;
	} else {
		while ((pp = of_get_next_child(node, pp)))
			cnt++;
		if (!cnt)
			return;

		vk = kzalloc(cnt * (sizeof *vk), GFP_KERNEL);
		pp = NULL;
		while ((pp = of_get_next_child(node, pp))) {
			if (of_property_read_u32(pp, "idx", &data) == 0)
				vk[i].index = data;
			if (of_property_read_u32_array(pp, "range", coords, 4) == 0) {
				vk[i].x_range_min = coords[0], vk[i].x_range_max = coords[1];
				vk[i].y_range_min = coords[2], vk[i].y_range_max = coords[3];
			} else
				I(" range faile");
			i++;
		}
		pdata->virtual_key = vk;
		for (i = 0; i < cnt; i++)
			I(" vk[%d] idx:%d x_min:%d, y_max:%d", i,pdata->virtual_key[i].index,
				pdata->virtual_key[i].x_range_min, pdata->virtual_key[i].y_range_max);
	}
}

static int himax_parse_dt(struct himax_ts_data *ts,
				struct himax_i2c_platform_data *pdata)
{
	int rc, coords_size = 0;
	uint32_t coords[4] = {0};
	struct property *prop;
	struct device_node *dt = ts->client->dev.of_node;
	u32 data = 0;

	prop = of_find_property(dt, "himax,panel-coords", NULL);
	if (prop) {
		coords_size = prop->length / sizeof(u32);
		if (coords_size != 4)
			D(" %s:Invalid panel coords size %d", __func__, coords_size);
	}

	if (of_property_read_u32_array(dt, "himax,panel-coords", coords, coords_size) == 0) {
		pdata->abs_x_min = coords[0], pdata->abs_x_max = coords[1];
		pdata->abs_y_min = coords[2], pdata->abs_y_max = coords[3];
		I(" DT-%s:panel-coords = %d, %d, %d, %d\n", __func__, pdata->abs_x_min,
				pdata->abs_x_max, pdata->abs_y_min, pdata->abs_y_max);
	}

	prop = of_find_property(dt, "himax,display-coords", NULL);
	if (prop) {
		coords_size = prop->length / sizeof(u32);
		if (coords_size != 4)
			D(" %s:Invalid display coords size %d", __func__, coords_size);
	}
	rc = of_property_read_u32_array(dt, "himax,display-coords", coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		D(" %s:Fail to read display-coords %d\n", __func__, rc);
		return rc;
	}
	pdata->screenWidth  = coords[1];
	pdata->screenHeight = coords[3];
	I(" DT-%s:display-coords = (%d, %d)", __func__, pdata->screenWidth,
		pdata->screenHeight);

	pdata->gpio_irq = of_get_named_gpio(dt, "himax,irq-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_irq)) {
		I(" DT:gpio_irq value is not valid\n");
	}

	pdata->gpio_reset = of_get_named_gpio(dt, "himax,rst-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_reset)) {
		I(" DT:gpio_rst value is not valid\n");
	}
	I(" DT:gpio_irq=%d, gpio_rst=%d", pdata->gpio_irq, pdata->gpio_reset);

	if (of_property_read_u32(dt, "report_type", &data) == 0) {
		pdata->protocol_type = data;
		I(" DT:protocol_type=%d", pdata->protocol_type);
	}

	if (of_property_read_u32(dt, "support_htc_event", &data) == 0) {
		pdata->support_htc_event = data;
		I(" DT:support_htc_event=%d", pdata->support_htc_event);
	}
	
	if (of_property_read_u32(dt, "event_htc_enable", &data) == 0) {
		pdata->event_htc_enable = data;
		I(" DT:event_htc_enable=%d", pdata->event_htc_enable);
	}
	himax_vk_parser(dt, pdata);

	return 0;
}
#endif

static void himax_ts_button_func(int tp_key_index,struct himax_ts_data *ts)
{
	uint16_t x_position = 0, y_position = 0;
if ( tp_key_index != 0x00)
	{
		I("virtual key index =%x\n",tp_key_index);
		if ( tp_key_index == 1) {
			vk_press = 1;
			I("back key pressed\n");
			if (ts->button[0].index) {
				x_position = (ts->button[0].x_range_min + ts->button[0].x_range_max) / 2;
				y_position = (ts->button[0].y_range_min + ts->button[0].y_range_max) / 2;
			}
			if (ts->protocol_type == PROTOCOL_TYPE_A) {
				if (ts->event_htc_enable_type == INJECT_HTC_EVENT) {
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
						100 << 16 | 100);
					input_report_abs(ts->input_dev, ABS_MT_POSITION,
						x_position << 16 | y_position);
				}
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
					100);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
					100);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
					100);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					y_position);
				input_mt_sync(ts->input_dev);
			} else if (ts->protocol_type == PROTOCOL_TYPE_B) {
				if (ts->event_htc_enable_type == INJECT_HTC_EVENT) {
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
						100 << 16 | 100);
					input_report_abs(ts->input_dev, ABS_MT_POSITION,
						x_position << 16 | y_position);
				}
				input_mt_slot(ts->input_dev, 0);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,
				1);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
					100);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
					100);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
					100);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					y_position);
			}
		}
		else if ( tp_key_index == 2) {
			vk_press = 1;
			I("home key pressed\n");
			if (ts->button[1].index) {
				x_position = (ts->button[1].x_range_min + ts->button[1].x_range_max) / 2;
				y_position = (ts->button[1].y_range_min + ts->button[1].y_range_max) / 2;
			}
				if (ts->protocol_type == PROTOCOL_TYPE_A) {
				if (ts->event_htc_enable_type == INJECT_HTC_EVENT) {
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
						100 << 16 | 100);
					input_report_abs(ts->input_dev, ABS_MT_POSITION,
						x_position << 16 | y_position);
				}
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
					100);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
					100);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
					100);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					y_position);
				input_mt_sync(ts->input_dev);
			} else if (ts->protocol_type == PROTOCOL_TYPE_B) {
					if (ts->event_htc_enable_type == INJECT_HTC_EVENT) {
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
						100 << 16 | 100);
					input_report_abs(ts->input_dev, ABS_MT_POSITION,
						x_position << 16 | y_position);
				}
				input_mt_slot(ts->input_dev, 0);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,
				1);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
					100);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
					100);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
					100);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					x_position);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					y_position);
			}
		}
		input_sync(ts->input_dev);
	}
else
	{
		I("virtual key released\n");
		vk_press = 0;
		if (ts->protocol_type == PROTOCOL_TYPE_A) {
			if (ts->event_htc_enable_type == INJECT_HTC_EVENT) {
				input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
				input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
			}
			input_mt_sync(ts->input_dev);
		}
		else if (ts->protocol_type == PROTOCOL_TYPE_B) {
			if (ts->event_htc_enable_type == INJECT_HTC_EVENT) {
				input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
				input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
			}
			input_mt_slot(ts->input_dev, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		}
	input_sync(ts->input_dev);
	}
}

inline void himax_ts_work(struct himax_ts_data *ts)
{
	int ret = 0;
	uint8_t buf[128], finger_num, hw_reset_check[2];
	uint16_t finger_pressed;
	uint8_t finger_on = 0;
	int32_t loop_i;
	uint8_t coordInfoSize = ts->coord_data_size + ts->area_data_size + 4;

	int  i, temp1, temp2;
	
	
	
	unsigned char check_sum_cal = 0;
	int RawDataLen = 0;
	

	
	#ifdef HX_TP_SYS_DIAG
	uint8_t *mutual_data;
	uint8_t *self_data;
	uint8_t diag_cmd;
	int 		mul_num;
	int 		self_num;
	int 		index = 0;
	int raw_cnt_max ;
	int raw_cnt_rmd ;
	int hx_touch_info_size;

	
	char coordinate_char[15+(HX_MAX_PT+5)*2*5+2];
	struct timeval t;
	struct tm broken;
	
	#endif
	

	memset(buf, 0x00, sizeof(buf));
	memset(hw_reset_check, 0x00, sizeof(hw_reset_check));

	raw_cnt_max = HX_MAX_PT/4;
	raw_cnt_rmd = HX_MAX_PT%4;

	if (raw_cnt_rmd != 0x00) 
	{
		if (IC_TYPE == HX_85XX_D_SERIES_PWON)
		{
			RawDataLen = 128 - ((HX_MAX_PT+raw_cnt_max+3)*4) - 1;
		}
		else
		{
			RawDataLen = 128 - ((HX_MAX_PT+raw_cnt_max+3)*4);
		}

		hx_touch_info_size = (HX_MAX_PT+raw_cnt_max+2)*4;
	}
	else 
	{
		if (IC_TYPE == HX_85XX_D_SERIES_PWON)
		{
			RawDataLen = 128 - ((HX_MAX_PT+raw_cnt_max+2)*4) - 1;
		}
		else
		{
			RawDataLen = 128 - ((HX_MAX_PT+raw_cnt_max+2)*4);
		}

		hx_touch_info_size = (HX_MAX_PT+raw_cnt_max+1)*4;
	}

	
		#ifdef ENABLE_CHIP_STATUS_MONITOR
		ts->running_status = 1;
		cancel_delayed_work_sync(&ts->himax_chip_monitor);
		#endif
	

	#ifdef HX_TP_SYS_DIAG
	diag_cmd = getDiagCommand();
	if( diag_cmd ){
		ret = i2c_himax_read(ts->client, 0x86, buf, 128,HIMAX_I2C_RETRY_TIMES);
	}
	else{
		if(touch_monitor_stop_flag != 0){
			ret = i2c_himax_read(ts->client, 0x86, buf, 128,HIMAX_I2C_RETRY_TIMES);
			touch_monitor_stop_flag-- ;
		}
		else{
			ret = i2c_himax_read(ts->client, 0x86, buf, hx_touch_info_size,HIMAX_I2C_RETRY_TIMES);
		}
	}
	if (ret)
	#else
	if (i2c_himax_read(ts->client, 0x86, buf, hx_touch_info_size,HIMAX_I2C_RETRY_TIMES))
	#endif
	{
		E("%s: can't read data from chip!\n", __func__);
		goto err_workqueue_out;
	}
	else
	{
		
			#ifdef HX_ESD_WORKAROUND
			for(i = 0; i < hx_touch_info_size; i++)
			{
				if (buf[i] == 0x00) 
				{
					check_sum_cal = 1;
				}
				else if(buf[i] == 0xED)
				{
					check_sum_cal = 2;
				}
				else
				{
					check_sum_cal = 0;
					i = hx_touch_info_size;
				}
			}

			
			
			#ifdef HX_TP_SYS_DIAG
			diag_cmd = getDiagCommand();
				#ifdef HX_ESD_WORKAROUND
					if (check_sum_cal != 0 && ESD_RESET_ACTIVATE == 0 && diag_cmd == 0)  
				#else
					if (check_sum_cal != 0 && diag_cmd == 0)
				#endif
			#else
				#ifdef HX_ESD_WORKAROUND
					if (check_sum_cal != 0 && ESD_RESET_ACTIVATE == 0 )  
				#else
					if (check_sum_cal !=0)
				#endif
			#endif
			
			{
				ret = himax_hang_shaking(); 
				if (ret == 2)
				{
					goto err_workqueue_out;
				}

				if ((ret == 1) && (check_sum_cal == 1))
				{
					I("[HIMAX TP MSG]: ESD event checked - ALL Zero.\n");
					ESD_HW_REST();
				}
				else if (check_sum_cal == 2)
				{
					I("[HIMAX TP MSG]: ESD event checked - ALL 0xED.\n");
					ESD_HW_REST();
				}
				
					#ifdef ENABLE_CHIP_STATUS_MONITOR
					ts->running_status = 0;
					queue_delayed_work(ts->himax_wq, &ts->himax_chip_monitor, 10*HZ);
					#endif
				
				himax_int_enable(1);
				return;
			}
			else if (ESD_RESET_ACTIVATE)
			{
				ESD_RESET_ACTIVATE = 0;
				I("[HIMAX TP MSG]:%s: Back from reset, ready to serve.\n", __func__);

				
					#ifdef ENABLE_CHIP_STATUS_MONITOR
					ts->running_status = 0;
					queue_delayed_work(ts->himax_wq, &ts->himax_chip_monitor, 10*HZ);
					#endif
				
				return;
			}
			#endif
		

		for (loop_i = 0, check_sum_cal = 0; loop_i < hx_touch_info_size; loop_i++)
			check_sum_cal += buf[loop_i];

		if ((check_sum_cal != 0x00) )
		{
			I("[HIMAX TP MSG] checksum fail : check_sum_cal: 0x%02X\n", check_sum_cal);

			
			
			
			
			
			
			
			
			
			

			
				#ifdef ENABLE_CHIP_STATUS_MONITOR
				ts->running_status = 0;
				queue_delayed_work(ts->himax_wq, &ts->himax_chip_monitor, 10*HZ);
				#endif
			

			return;
		}
	}

	if (ts->debug_log_level & BIT(0)) {
		I("%s: raw data:\n", __func__);
		for (loop_i = 0; loop_i < hx_touch_info_size; loop_i++) {
			I("0x%2.2X ", buf[loop_i]);
			if (loop_i % 8 == 7)
				I("\n");
		}
	}

	
	
	#ifdef HX_TP_SYS_DIAG
	diag_cmd = getDiagCommand();
	if (diag_cmd >= 1 && diag_cmd <= 6)
	{
		if (IC_TYPE == HX_85XX_D_SERIES_PWON)
		{
			
			for (i = hx_touch_info_size, check_sum_cal = 0; i < 128; i++)
			{
				check_sum_cal += buf[i];
			}

			if (check_sum_cal % 0x100 != 0)
			{
				goto bypass_checksum_failed_packet;
			}
		}

		mutual_data = getMutualBuffer();
		self_data 	= getSelfBuffer();

		
		mul_num = getXChannel() * getYChannel();

		#ifdef HX_EN_SEL_BUTTON
		self_num = getXChannel() + getYChannel() + HX_BT_NUM;
		#else
		self_num = getXChannel() + getYChannel();
		#endif

		
		if (buf[hx_touch_info_size] == buf[hx_touch_info_size+1] && buf[hx_touch_info_size+1] == buf[hx_touch_info_size+2]
		&& buf[hx_touch_info_size+2] == buf[hx_touch_info_size+3] && buf[hx_touch_info_size] > 0)
		{
			index = (buf[hx_touch_info_size] - 1) * RawDataLen;
			
			for (i = 0; i < RawDataLen; i++)
			{
				if (IC_TYPE == HX_85XX_D_SERIES_PWON)
				{
					temp1 = index + i;
				}
				else
				{
					temp1 = index;
				}

				if (temp1 < mul_num)
				{ 
					mutual_data[index + i] = buf[i + hx_touch_info_size+4];	
				}
				else
				{
					if (IC_TYPE == HX_85XX_D_SERIES_PWON)
					{
						temp1 = i + index;
						temp2 = self_num + mul_num;
					}
					else
					{
						temp1 = i;
						temp2 = self_num;
					}
					if (temp1 >= temp2)
					{
						break;
					}

					if (IC_TYPE == HX_85XX_D_SERIES_PWON)
					{
						self_data[i+index-mul_num] = buf[i + hx_touch_info_size+4];	
					}
					else
					{
						self_data[i] = buf[i + hx_touch_info_size+4];	
					}
				}
			}
		}
		else
		{
			I("[HIMAX TP MSG]%s: header format is wrong!\n", __func__);
		}
	}
	else if (diag_cmd == 7)
	{
		memcpy(&(diag_coor[0]), &buf[0], 128);
	}
	
	if (coordinate_dump_enable == 1)
	{
		for(i=0; i<(15 + (HX_MAX_PT+5)*2*5); i++)
		{
			coordinate_char[i] = 0x20;
		}
		coordinate_char[15 + (HX_MAX_PT+5)*2*5] = 0xD;
		coordinate_char[15 + (HX_MAX_PT+5)*2*5 + 1] = 0xA;
	}
	
	#endif
	

bypass_checksum_failed_packet:
		EN_NoiseFilter = (buf[HX_TOUCH_INFO_POINT_CNT+2]>>3);
		
		EN_NoiseFilter = EN_NoiseFilter & 0x01;
		
	#if defined(HX_EN_SEL_BUTTON) || defined(HX_EN_MUT_BUTTON)
		tpd_key = (buf[HX_TOUCH_INFO_POINT_CNT+2]>>4);
		if (tpd_key == 0x0F)
		{
			tpd_key = 0x00;
		}
		
		#else
		tpd_key = 0x00;
		#endif

		p_point_num = hx_point_num;

		if (buf[HX_TOUCH_INFO_POINT_CNT] == 0xff)
			hx_point_num = 0;
		else
			hx_point_num= buf[HX_TOUCH_INFO_POINT_CNT] & 0x0f;

		
		if (hx_point_num != 0 ) {
			if(vk_press == 0x00)
				{
					uint16_t old_finger = ts->pre_finger_mask;
					finger_num = buf[coordInfoSize - 4] & 0x0F;
					finger_pressed = buf[coordInfoSize - 2] << 8 | buf[coordInfoSize - 3];
					finger_on = 1;
					AA_press = 1;
					for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
						if (((finger_pressed >> loop_i) & 1) == 1) {
							int base = loop_i * 4;
							int x = buf[base] << 8 | buf[base + 1];
							int y = (buf[base + 2] << 8 | buf[base + 3]);
							int w = buf[(ts->nFinger_support * 4) + loop_i];
							finger_num--;

							if (ts->event_htc_enable_type)
							{
								input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, w << 16 | w);
								input_report_abs(ts->input_dev, ABS_MT_POSITION,
									((finger_num ==  0) ? BIT(31) : 0) | x << 16 | y);
							}
							if ((ts->debug_log_level & BIT(3)) > 0)
							{
								if ((((old_finger >> loop_i) ^ (finger_pressed >> loop_i)) & 1) == 1)
								{
									if (ts->useScreenRes)
									{
										I("status:%X, Screen:F:%02d Down, X:%d, Y:%d, W:%d, N:%d\n",
										finger_pressed, loop_i+1, x * ts->widthFactor >> SHIFTBITS,
										y * ts->heightFactor >> SHIFTBITS, w, EN_NoiseFilter);
									}
									else
									{
										I("status:%X, Raw:F:%02d Down, X:%d, Y:%d, W:%d, N:%d\n",
										finger_pressed, loop_i+1, x, y, w, EN_NoiseFilter);
									}
								}
							}

							if (ts->protocol_type == PROTOCOL_TYPE_B)
							{
								input_mt_slot(ts->input_dev, loop_i);
							}

							if (ts->event_htc_enable_type != SWITCH_TO_HTC_EVENT_ONLY)
							{
								input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
								input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
								input_report_abs(ts->input_dev, ABS_MT_PRESSURE, w);
								input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
								input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
							}

							if (ts->protocol_type == PROTOCOL_TYPE_A)
							{
								input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, loop_i);
								input_mt_sync(ts->input_dev);
							}
							else
							{
								ts->last_slot = loop_i;
								input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
							}

							if (!ts->first_pressed)
							{
								ts->first_pressed = 1;
								ts->just_resume = 0;
								I("S1@%d, %d\n", x, y);
							}

							ts->pre_finger_data[loop_i][0] = x;
							ts->pre_finger_data[loop_i][1] = y;


							if (ts->debug_log_level & BIT(1))
								I("Finger %d=> X:%d, Y:%d W:%d, Z:%d, F:%d, N:%d\n",
									loop_i + 1, x, y, w, w, loop_i + 1, EN_NoiseFilter);

						} else {
							if (ts->protocol_type == PROTOCOL_TYPE_B)
							{
								input_mt_slot(ts->input_dev, loop_i);
								input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
							}

							if (loop_i == 0 && ts->first_pressed == 1)
							{
								ts->first_pressed = 2;
								I("E1@%d, %d\n",
								ts->pre_finger_data[0][0] , ts->pre_finger_data[0][1]);
							}
							if ((ts->debug_log_level & BIT(3)) > 0)
							{
								if ((((old_finger >> loop_i) ^ (finger_pressed >> loop_i)) & 1) == 1)
								{
									if (ts->useScreenRes)
									{
										I("status:%X, Screen:F:%02d Up, X:%d, Y:%d, N:%d\n",
										finger_pressed, loop_i+1, ts->pre_finger_data[loop_i][0] * ts->widthFactor >> SHIFTBITS,
										ts->pre_finger_data[loop_i][1] * ts->heightFactor >> SHIFTBITS, Last_EN_NoiseFilter);
									}
									else
									{
										I("status:%X, Raw:F:%02d Up, X:%d, Y:%d, N:%d\n",
										finger_pressed, loop_i+1, ts->pre_finger_data[loop_i][0],
										ts->pre_finger_data[loop_i][1], Last_EN_NoiseFilter);
									}
								}
							}
						}
					}
					ts->pre_finger_mask = finger_pressed;
				}else if ((tpd_key_old != 0x00)&&(tpd_key == 0x00)) {
					
					
					
					
					himax_ts_button_func(tpd_key,ts);
					finger_on = 0;
				}
			
				#ifdef HX_ESD_WORKAROUND
				ESD_COUNTER = 0;
				#endif
			
			if (ts->event_htc_enable_type != SWITCH_TO_HTC_EVENT_ONLY) {
			input_report_key(ts->input_dev, BTN_TOUCH, finger_on);
			input_sync(ts->input_dev);
			}
		} else if (hx_point_num == 0){
			if(AA_press){
				
				finger_on = 0;
				AA_press = 0;
				if (ts->event_htc_enable_type) {
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
					input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
				}

				if (ts->event_htc_enable_type != SWITCH_TO_HTC_EVENT_ONLY && ts->protocol_type == PROTOCOL_TYPE_A)
					input_mt_sync(ts->input_dev);

				for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++) {
						if (((ts->pre_finger_mask >> loop_i) & 1) == 1) {
							if (ts->event_htc_enable_type != SWITCH_TO_HTC_EVENT_ONLY && ts->protocol_type == PROTOCOL_TYPE_B) {
								input_mt_slot(ts->input_dev, loop_i);
								input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
							}
						}
					}
				if (ts->pre_finger_mask > 0) {
					for (loop_i = 0; loop_i < ts->nFinger_support && (ts->debug_log_level & BIT(3)) > 0; loop_i++) {
						if (((ts->pre_finger_mask >> loop_i) & 1) == 1) {
							if (ts->useScreenRes) {
								I("status:%X, Screen:F:%02d Up, X:%d, Y:%d, N:%d\n", 0, loop_i+1, ts->pre_finger_data[loop_i][0] * ts->widthFactor >> SHIFTBITS,
									ts->pre_finger_data[loop_i][1] * ts->heightFactor >> SHIFTBITS, Last_EN_NoiseFilter);
							} else {
								I("status:%X, Raw:F:%02d Up, X:%d, Y:%d, N:%d\n",0, loop_i+1, ts->pre_finger_data[loop_i][0],ts->pre_finger_data[loop_i][1], Last_EN_NoiseFilter);
							}
						}
					}
					ts->pre_finger_mask = 0;
				}

				if (ts->first_pressed == 1) {
					ts->first_pressed = 2;
					I("E1@%d, %d\n",ts->pre_finger_data[0][0] , ts->pre_finger_data[0][1]);
				}

				if (ts->debug_log_level & BIT(1))
					I("All Finger leave\n");

				#ifdef HX_PORTING_DEB_MSG
				I("[HIMAX PORTING MSG]%s Touch UP.\n",__func__);
				#endif

				
					#ifdef HX_TP_SYS_DIAG
					
					if (coordinate_dump_enable == 1)
					{
						do_gettimeofday(&t);
						time_to_tm(t.tv_sec, 0, &broken);

						sprintf(&coordinate_char[0], "%2d:%2d:%2d:%lu,", broken.tm_hour, broken.tm_min, broken.tm_sec, t.tv_usec/1000);
						sprintf(&coordinate_char[15], "Touch up!");
						coordinate_fn->f_op->write(coordinate_fn,&coordinate_char[0],15 + (HX_MAX_PT+5)*2*sizeof(char)*5 + 2,&coordinate_fn->f_pos);
					}
					
					#endif
				
			}
			else if (tpd_key != 0x00) {
				
				
				
				
				
				himax_ts_button_func(tpd_key,ts);
				finger_on = 1;
			}
			else if ((tpd_key_old != 0x00)&&(tpd_key == 0x00)) {
				
				
				
				
				himax_ts_button_func(tpd_key,ts);
				finger_on = 0;
			}
			
				#ifdef HX_ESD_WORKAROUND
				ESD_COUNTER = 0;
				#endif
			
			if (ts->event_htc_enable_type != SWITCH_TO_HTC_EVENT_ONLY) {
			input_report_key(ts->input_dev, BTN_TOUCH, finger_on);
			input_sync(ts->input_dev);
			}
		}
		tpd_key_old = tpd_key;
		Last_EN_NoiseFilter = EN_NoiseFilter;

		
			#ifdef ENABLE_CHIP_STATUS_MONITOR
			ts->running_status = 0;
			queue_delayed_work(ts->himax_wq, &ts->himax_chip_monitor, 10*HZ);
			#endif
		

workqueue_out:
	return;

err_workqueue_out:
	I("%s: Now reset the Touch chip.\n", __func__);
	if(ts->rst_gpio)
		{
			gpio_direction_output(ts->rst_gpio, 0);
			hr_msleep(20);
			gpio_direction_output(ts->rst_gpio, 1);
			hr_msleep(50);
		}

	
		#ifdef HX_RST_PIN_FUNC
		himax_HW_reset();
		himax_loadSensorConfig(private_ts->client,private_ts->pdata,true);
		#endif
	

	
		#ifdef ENABLE_CHIP_STATUS_MONITOR
		ts->running_status = 0;
		queue_delayed_work(ts->himax_wq, &ts->himax_chip_monitor, 10*HZ);
		#endif
	

	goto workqueue_out;
}

static irqreturn_t himax_ts_thread(int irq, void *ptr)
{
	struct himax_ts_data *ts = ptr;
	struct timespec timeStart, timeEnd, timeDelta;

	if (ts->debug_log_level & BIT(2)) {
			getnstimeofday(&timeStart);
	}
	himax_ts_work((struct himax_ts_data *)ptr);
	if(ts->debug_log_level & BIT(2)) {
			getnstimeofday(&timeEnd);
				timeDelta.tv_nsec = (timeEnd.tv_sec*1000000000+timeEnd.tv_nsec)
				-(timeStart.tv_sec*1000000000+timeStart.tv_nsec);
			I("Touch latency = %ld us\n", timeDelta.tv_nsec/1000);
	}
	return IRQ_HANDLED;
}

static void himax_ts_work_func(struct work_struct *work)
{
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data, work);
	himax_ts_work(ts);	
}

static enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer)
{
	struct himax_ts_data *ts;

	ts = container_of(timer, struct himax_ts_data, timer);
	queue_work(ts->himax_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static void himax_cable_tp_status_handler_func(int connect_status)
{
	struct himax_ts_data *ts;
	I("Touch: cable change to %d\n", connect_status);
	ts = private_ts;
	if (ts->cable_config) {
		if (!atomic_read(&ts->suspend_mode) || !atomic_read(&ts->in_flash)) {
			if ((!!connect_status) != ts->usb_connected) {
				if (!!connect_status) {
					ts->cable_config[1] = 0x01;
					ts->usb_connected = 0x01;
				} else {
					ts->cable_config[1] = 0x00;
					ts->usb_connected = 0x00;
				}

				i2c_himax_master_write(ts->client, ts->cable_config,
					sizeof(ts->cable_config), HIMAX_I2C_RETRY_TIMES);

				I("%s: Cable status change: 0x%2.2X\n", __func__, ts->cable_config[1]);
			} else
				I("%s: Cable status is the same as previous one, ignore.\n", __func__);
		} else {
			if (connect_status)
				ts->usb_connected = 0x01;
			else
				ts->usb_connected = 0x00;
			I("%s: Cable status remembered: 0x%2.2X\n", __func__, ts->usb_connected);
		}
	}
}

#if defined (CONFIG_ARCH_MSM)
static struct t_cable_status_notifier himax_cable_status_handler = {
	.name = "usb_tp_connected",
	.func = himax_cable_tp_status_handler_func,
};
#endif

#if defined (CONFIG_UX500_SOC_DB8500)
static struct battery_usb_status_notifier himax_cable_status_handler = {
	.name = "usb_tp_connected",
	.func = himax_cable_tp_status_handler_func,
};
#endif

static int himax_read_flash(unsigned char *buf, unsigned int addr_start, unsigned int length)
{
	u16 i = 0;
	u16 j = 0;
	u16 k = 0;
	uint8_t cmd[4];
	u16 local_start_addr   	= addr_start / 4;
	u16 local_length       	= length;
	u16 local_end_addr	   	= (addr_start + length ) / 4 + 1;
	u16 local_addr		   		= addr_start % 4;

	memset(cmd, 0x00, sizeof(cmd));

	I("Himax %s addr_start = %d , local_start_addr = %d , local_length = %d , local_end_addr = %d , local_addr = %d \n",__func__,addr_start,local_start_addr,local_length,local_end_addr,local_addr);
	if ( i2c_himax_write_command(private_ts->client, 0x81, DEFAULT_RETRY_CNT) < 0)
	{
		E("%s i2c write 81 fail.\n",__func__);
		return 0;
	}
	hr_msleep(120);
	if ( i2c_himax_write_command(private_ts->client, 0x82, DEFAULT_RETRY_CNT) < 0)
	{
		E("%s i2c write 82 fail.\n",__func__);
		return 0;
	}
	hr_msleep(100);
	cmd[0] = 0x01;
	if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 1, DEFAULT_RETRY_CNT) < 0)
	{
		E("%s i2c write 43 fail.\n",__func__);
		return 0;
	}
	hr_msleep(100);
	i = local_start_addr;
	do
	{
		cmd[0] = i & 0x1F;
		cmd[1] = (i >> 5) & 0x1F;
		cmd[2] = (i >> 10) & 0x1F;
		if ( i2c_himax_write(private_ts->client, 0x44 ,&cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
		{
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}
		if ( i2c_himax_write(private_ts->client, 0x46 ,&cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
		{
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}
		if ( i2c_himax_read(private_ts->client, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0)
		{
			E("%s: i2c access fail!\n", __func__);
			return 0;
		}
		I("Himax cmd[0]=%d,cmd[1]=%d,cmd[2]=%d,cmd[3]=%d\n",cmd[0],cmd[1],cmd[2],cmd[3]);
		if (i == local_start_addr) 
		{
			j = 0;
			for(k = local_addr; k < 4 && j < local_length; k++)
			{
				buf[j++] = cmd[k];
			}
		}
		else 
		{
			for(k = 0; k < 4 && j < local_length; k++)
			{
				buf[j++] = cmd[k];
			}
		}
		i++;
	}
	while(i < local_end_addr);
	cmd[0] = 0;
	if ( i2c_himax_write(private_ts->client, 0x43 ,&cmd[0], 1, DEFAULT_RETRY_CNT) < 0)
	{
		return 0;
	}
	return 1;
}

void himax_touch_information(void)
{
	static unsigned char temp_buffer[6];
	if (IC_TYPE == HX_85XX_A_SERIES_PWON) {
		HX_RX_NUM      = 0;
		HX_TX_NUM      = 0;
		HX_BT_NUM      = 0;
		HX_X_RES       = 0;
		HX_Y_RES       = 0;
		HX_MAX_PT      = 0;
		HX_XY_REVERSE  = false;
		HX_INT_IS_EDGE = false;
	} else if (IC_TYPE == HX_85XX_B_SERIES_PWON) {
		HX_RX_NUM      = 0;
		HX_TX_NUM      = 0;
		HX_BT_NUM      = 0;
		HX_X_RES       = 0;
		HX_Y_RES       = 0;
		HX_MAX_PT      = 0;
		HX_XY_REVERSE  = false;
		HX_INT_IS_EDGE = false;
	} else if (IC_TYPE == HX_85XX_C_SERIES_PWON) {
		
		himax_read_flash( temp_buffer, 0x3D5, 3);
		HX_RX_NUM = temp_buffer[0];
		HX_TX_NUM = temp_buffer[1];
		HX_BT_NUM = (temp_buffer[2]) & 0x1F;

		
		himax_read_flash( temp_buffer, 0x3EE, 2);

		if ((temp_buffer[0] && 0x04) == 0x04) {
			HX_XY_REVERSE = true;
		} else {
			HX_XY_REVERSE = false;
		}

		if ((temp_buffer[1] && 0x01) == 1 ) {
			HX_INT_IS_EDGE = true;
		} else {
			HX_INT_IS_EDGE = false;
		}

		
		himax_read_flash( temp_buffer, 0x345, 4);
		if (HX_XY_REVERSE) {
			HX_X_RES = temp_buffer[2]*256 + temp_buffer[3];
			HX_Y_RES = temp_buffer[0]*256 + temp_buffer[1];
		} else {
			HX_X_RES = temp_buffer[0]*256 + temp_buffer[1];
			HX_Y_RES = temp_buffer[2]*256 + temp_buffer[3];
		}

		
		himax_read_flash( temp_buffer, 0x3ED, 1);
		HX_MAX_PT = temp_buffer[0] >> 4;


	} else if (IC_TYPE == HX_85XX_D_SERIES_PWON) {
		himax_read_flash( temp_buffer, 0x26E, 3);
		HX_RX_NUM = temp_buffer[0];
		HX_TX_NUM = temp_buffer[1];
		HX_MAX_PT = (temp_buffer[2] & 0xF0) >> 4;

		#ifdef HX_EN_SEL_BUTTON
		HX_BT_NUM = (temp_buffer[2] & 0x0F);
		#endif

		#ifdef HX_EN_MUT_BUTTON
		himax_read_flash( temp_buffer, 0x262, 1);
		HX_BT_NUM = (temp_budder[0] & 0x07);
		#endif

		himax_read_flash( temp_buffer, 0x272, 6);
		if ((temp_buffer[0] & 0x04) == 0x04) {
			HX_XY_REVERSE = true;

			HX_X_RES = temp_buffer[4]*256 + temp_buffer[5];
			HX_Y_RES = temp_buffer[2]*256 + temp_buffer[3];
		} else {
			HX_XY_REVERSE = false;

			HX_X_RES = temp_buffer[2]*256 + temp_buffer[3];
			HX_Y_RES = temp_buffer[4]*256 + temp_buffer[5];
		}

		himax_read_flash( temp_buffer, 0x200, 6);
		if ((temp_buffer[1] && 0x01) == 1 ) {
			HX_INT_IS_EDGE = true;
		} else {
			HX_INT_IS_EDGE = false;
		}
	} else {
		HX_RX_NUM      = 0;
		HX_TX_NUM      = 0;
		HX_BT_NUM      = 0;
		HX_X_RES       = 0;
		HX_Y_RES       = 0;
		HX_MAX_PT      = 0;
		HX_XY_REVERSE  = false;
		HX_INT_IS_EDGE = false;
	}
}

static bool himax_ic_package_check(struct himax_ts_data *ts)
{
	uint8_t cmd[3];
	uint8_t data[3];

	memset(cmd, 0x00, sizeof(cmd));
	memset(data, 0x00, sizeof(data));

	if (i2c_himax_read(ts->client, 0xD1, cmd, 3, DEFAULT_RETRY_CNT) < 0)
		return false ;

	if (i2c_himax_read(ts->client, 0x31, data, 3, DEFAULT_RETRY_CNT) < 0)
		return false;

	if ((data[0] == 0x85 && data[1] == 0x28) || (cmd[0] == 0x04 && cmd[1] == 0x85 &&
		(cmd[2] == 0x26 || cmd[2] == 0x27 || cmd[2] == 0x28))) {
		IC_TYPE                = HX_85XX_D_SERIES_PWON;
		IC_CHECKSUM            = HX_TP_BIN_CHECKSUM_CRC;
		
		FW_VER_MAJ_FLASH_ADDR  = 133;                    
		FW_VER_MAJ_FLASH_LENG  = 1;;
		FW_VER_MIN_FLASH_ADDR  = 134;                    
		FW_VER_MIN_FLASH_LENG  = 1;
		CFG_VER_MAJ_FLASH_ADDR = 160;                    
		CFG_VER_MAJ_FLASH_LENG = 12;
		CFG_VER_MIN_FLASH_ADDR = 172;                    
		CFG_VER_MIN_FLASH_LENG = 12;

		I("Himax IC package 8528 D\n");
	} else if ((data[0] == 0x85 && data[1] == 0x23) || (cmd[0] == 0x03 && cmd[1] == 0x85 &&
			(cmd[2] == 0x26 || cmd[2] == 0x27 || cmd[2] == 0x28 || cmd[2] == 0x29))) {
		IC_TYPE                = HX_85XX_C_SERIES_PWON;
		IC_CHECKSUM            = HX_TP_BIN_CHECKSUM_SW;
		
		FW_VER_MAJ_FLASH_ADDR  = 133;                   
		FW_VER_MAJ_FLASH_LENG  = 1;
		FW_VER_MIN_FLASH_ADDR  = 134;                   
		FW_VER_MIN_FLASH_LENG  = 1;
		CFG_VER_MAJ_FLASH_ADDR = 135;                   
		CFG_VER_MAJ_FLASH_LENG = 12;
		CFG_VER_MIN_FLASH_ADDR = 147;                   
		CFG_VER_MIN_FLASH_LENG = 12;

		I("Himax IC package 8523 C\n");
	} else if ((data[0] == 0x85 && data[1] == 0x26) ||
		   (cmd[0] == 0x02 && cmd[1] == 0x85 &&
		   (cmd[2] == 0x19 || cmd[2] == 0x25 || cmd[2] == 0x26))) {
		IC_TYPE                = HX_85XX_B_SERIES_PWON;
		IC_CHECKSUM            = HX_TP_BIN_CHECKSUM_SW;
		
		FW_VER_MAJ_FLASH_ADDR  = 133;                   
		FW_VER_MAJ_FLASH_LENG  = 1;
		FW_VER_MIN_FLASH_ADDR  = 728;                   
		FW_VER_MIN_FLASH_LENG  = 1;
		CFG_VER_MAJ_FLASH_ADDR = 692;                   
		CFG_VER_MAJ_FLASH_LENG = 3;
		CFG_VER_MIN_FLASH_ADDR = 704;                   
		CFG_VER_MIN_FLASH_LENG = 3;

		I("Himax IC package 8526 B\n");
	} else if ((data[0] == 0x85 && data[1] == 0x20) || (cmd[0] == 0x01 &&
			cmd[1] == 0x85 && cmd[2] == 0x19)) {
		IC_TYPE     = HX_85XX_A_SERIES_PWON;
		IC_CHECKSUM = HX_TP_BIN_CHECKSUM_SW;
		I("Himax IC package 8520 A\n");
	} else {
		E("Himax IC package incorrect!!\n");
	}
	return true;
}

void calculate_point_number(void)
{
	HX_TOUCH_INFO_POINT_CNT = HX_MAX_PT * 4 ;

	if ( (HX_MAX_PT % 4) == 0)
		HX_TOUCH_INFO_POINT_CNT += (HX_MAX_PT / 4) * 4 ;
	else
		HX_TOUCH_INFO_POINT_CNT += ((HX_MAX_PT / 4) +1) * 4 ;
}
#ifdef CONFIG_FB
static void himax_fb_register(struct work_struct *work)
{
	int ret = 0;
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data,
							work_att.work);
	I(" %s in", __func__);

	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret)
		E(" Unable to register fb_notifier: %d\n", ret);
}
#endif

static int himax8528_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0, err = 0;
	uint8_t buf[2] = {0};
	struct himax_ts_data *ts;
	struct himax_i2c_platform_data *pdata;

	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		E("%s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct himax_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		E("%s: allocate himax_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->i2c_api.i2c_himax_master_write = i2c_himax_master_write;
	ts->i2c_api.i2c_himax_read_command = i2c_himax_read_command;
	ts->i2c_api.i2c_himax_write_command = i2c_himax_write_command;

	ts->client = client;
	ts->just_resume = 0;
	i2c_set_clientdata(client, ts);
	ts->client = client;
	ts->dev = &client->dev;

	#if 0
	dev_set_name(ts->dev, HIMAX8528_NAME);
	#endif
#ifdef CONFIG_OF
	if (client->dev.of_node) { 
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL) {
			err = -ENOMEM;
			goto err_alloc_dt_pdata_failed;
		}
		ret = himax_parse_dt(ts, pdata);
		if (ret < 0) {
			I(" pdata is NULL for DT\n");
			goto err_dt_platform_data_fail;
		}
	} else
#endif	
	{
		pdata = client->dev.platform_data;
		if (pdata == NULL) {
			I(" pdata is NULL(dev.platform_data)\n");
			goto err_get_platform_data_fail;
		}
	}
	#ifdef HX_RST_PIN_FUNC
		ts->rst_gpio = pdata->gpio_reset;
	#endif

	if (pdata->gpio_reset) {
		ret = gpio_request(pdata->gpio_reset, "himax-reset");
		if (ret < 0)
			E("%s: request reset pin failed\n", __func__);
	}
#ifndef CONFIG_OF
	if (pdata->power) {
		ret = pdata->power(1);
		if (ret < 0) {
			E("%s: power on failed\n", __func__);
			goto err_power_failed;
		}
	}

	if (pdata->init) {
		ret = pdata->init(ts->dev, pdata);
		if (ret) {
			E("failed initing regulator. err=%d\n", ret);
			goto err_power_failed;
		}
	}

	if (pdata->enable) {
		ret = pdata->enable(ts->dev, pdata);
		if (ret) {
			E("failed enable regulator. err=%d\n", ret);
			goto err_power_failed;
		}
	}
#endif
	private_ts = ts;
	
	I("%s, pdata, %X\n", __func__ ,(uint32_t)pdata);
	
	if (himax_ic_package_check(ts) == false) {
		E("Himax chip doesn NOT EXIST");
		return -1;
	}
	
	
	if (board_mfg_mode() == MFG_MODE_OFFMODE_CHARGING ||
	    board_mfg_mode() == MFG_MODE_POWER_TEST) {
		I(" %s: offmode charging. Set touch chip to sleep mode and skip touch driver probe\n", __func__);
		
		buf[0] = HX_CMD_TSSOFF;
		ret = i2c_himax_master_write(client, buf, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0)
		{
			E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, client->addr);
		}
		hr_msleep(30);

		buf[0] = HX_CMD_TSSLPIN;
		ret = i2c_himax_master_write(client, buf, 1, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0)
		{
			E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, client->addr);
		}
		hr_msleep(30);

		buf[0] = HX_CMD_SETDEEPSTB;
		buf[1] = 0x01;
		ret = i2c_himax_master_write(client, buf, 2, HIMAX_I2C_RETRY_TIMES);
		if (ret < 0)
		{
			E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, client->addr);
		}
		err = -ENODEV;
		goto err_check_offmode_charging;
	}
	
	
	#ifdef HX_TP_SYS_FLASH_DUMP
	setSysOperation(0);
	setFlashBuffer();
	#endif
	

	ts->usb_connected = 0x00;
	if (pdata->virtual_key)
		ts->button = pdata->virtual_key;
	
	if (pdata->loadSensorConfig) {
		if (pdata->loadSensorConfig(client, pdata, &(ts->i2c_api)) < 0) {
			E("%s: Load Sesnsor configuration failed, unload driver.\n", __func__);
			goto err_detect_failed;
		}
	} else {
		if (himax_loadSensorConfig(client, pdata,true) < 0) {
			E("%s: Load Sesnsor configuration failed, unload driver.\n", __func__);
			goto err_detect_failed;
		}
	}

	calculate_point_number();
	
	#ifdef HX_TP_SYS_DIAG
	setXChannel(HX_RX_NUM); 
	setYChannel(HX_TX_NUM); 

	setMutualBuffer();
	if (getMutualBuffer() == NULL) {
		E("%s: mutual buffer allocate fail failed\n", __func__);
		return -1;
	}
	#endif
	
#ifdef CONFIG_OF
	ts->power = pdata->power;
#endif
	ts->pdata = pdata;
	if (pdata->event_htc_enable)
		ts->event_htc_enable_type = SWITCH_TO_HTC_EVENT_ONLY;
	else if (pdata->support_htc_event)
		ts->event_htc_enable_type = INJECT_HTC_EVENT;
	else
		ts->event_htc_enable_type = 0;

	ts->x_channel = HX_RX_NUM;
	ts->y_channel = HX_TX_NUM;
	ts->nFinger_support = HX_MAX_PT;
	
	calcDataSize(ts->nFinger_support);
	I("%s: calcDataSize complete\n", __func__);
	#ifdef CONFIG_OF
	ts->pdata->abs_pressure_min = 0;
	ts->pdata->abs_pressure_max = 200;
	ts->pdata->abs_width_min = 0;
	ts->pdata->abs_width_max = 200;
	pdata->cable_config[0] = 0x90;
	pdata->cable_config[1] = 0x00;
	#endif
	ts->suspended          = false;
	
	ts->cable_config = pdata->cable_config;
	ts->protocol_type = pdata->protocol_type;
	I("%s: Use Protocol Type %c\n", __func__,
	ts->protocol_type == PROTOCOL_TYPE_A ? 'A' : 'B');

	ret = himax_input_register(ts);
	if (ret) {
		E("%s: Unable to register %s input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	if (get_tamper_sf() == 0) {
		ts->debug_log_level |= BIT(3);
		I("%s: Enable touch down/up debug log since not security-on device",
			__func__);
		if (pdata->screenWidth > 0 && pdata->screenHeight > 0 &&
		 (pdata->abs_x_max - pdata->abs_x_min) > 0 &&
		 (pdata->abs_y_max - pdata->abs_y_min) > 0) {
			ts->widthFactor = (pdata->screenWidth << SHIFTBITS)/(pdata->abs_x_max - pdata->abs_x_min);
			ts->heightFactor = (pdata->screenHeight << SHIFTBITS)/(pdata->abs_y_max - pdata->abs_y_min);
			if (ts->widthFactor > 0 && ts->heightFactor > 0)
				ts->useScreenRes = 1;
			else {
				ts->heightFactor = 0;
				ts->widthFactor = 0;
				ts->useScreenRes = 0;
			}
		} else
			I("Enable finger debug with raw position mode!\n");
	}

#ifdef CONFIG_FB
	ts->himax_att_wq = create_singlethread_workqueue("HMX_ATT_reuqest");
	if (!ts->himax_att_wq) {
		E(" allocate syn_att_wq failed\n");
		err = -ENOMEM;
		goto err_get_intr_bit_failed;
	}
	INIT_DELAYED_WORK(&ts->work_att, himax_fb_register);
	queue_delayed_work(ts->himax_att_wq, &ts->work_att, msecs_to_jiffies(15000));

#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING + 1;
	ts->early_suspend.suspend = himax_ts_early_suspend;
	ts->early_suspend.resume = himax_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	himax_touch_sysfs_init();

	
	#ifdef  HX_TP_SYS_FLASH_DUMP
	ts->flash_wq = create_singlethread_workqueue("himax_flash_wq");
	if (!ts->flash_wq)
	{
		E("%s: create flash workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_wq_failed;
	}

	INIT_WORK(&ts->flash_work, himax_ts_flash_work_func);

	setSysOperation(0);
	setFlashBuffer();
	#endif
	
	
	#ifdef ENABLE_CHIP_RESET_MACHINE
	INIT_DELAYED_WORK(&ts->himax_chip_reset_work, himax_chip_reset_function);
	#endif
	
	
	#ifdef ENABLE_CHIP_STATUS_MONITOR
	INIT_DELAYED_WORK(&ts->himax_chip_monitor, himax_chip_monitor_function); 
	#endif
	

	
	#ifdef HX_ESD_WORKAROUND
	ESD_RESET_ACTIVATE = 0;
	#endif
	

#ifdef FAKE_EVENT
	ts->fake_X_S = 10;
	ts->fake_Y_S = 400;
	ts->fake_X_E = 400;
	ts->fake_Y_E = 400;
#endif

	if (ts->cable_config)
#if defined (CONFIG_UX500_SOC_DB8500)
		battery_usb_register_notifier(&himax_cable_status_handler);
#else
		
		cable_detect_register_notifier(&himax_cable_status_handler);
#endif

#if defined (CONFIG_ARCH_U8500)
	himax_cable_tp_status_handler_func(htc_charger_is_vbus_present());
#endif

	ts->irq_enabled = 0;
	
	if (client->irq) {
		ts->use_irq = 1;
		#if defined (CONFIG_UX500_SOC_DB8500)
			I("%s defined CONFIG_UX500_SOC_DB8500\n ",__func__);
			ret = request_threaded_irq(client->irq, NULL, himax_ts_thread,IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->name, ts);
		#else
			I("%s don't defined CONFIG_UX500_SOC_DB8500\n ",__func__);
			ret = request_threaded_irq(client->irq, NULL, himax_ts_thread,IRQF_TRIGGER_LOW | IRQF_ONESHOT, client->name, ts);
		#endif
		if (ret == 0) {
			ts->irq_enabled = 1;
			irq_enable_count = 1;
			I("%s: irq enabled at qpio: %d\n", __func__, client->irq);
			ts->irq = pdata->gpio_irq;
		} else {
			ts->use_irq = 0;
			E("%s: request_irq failed\n", __func__);
		}
	} else {
		I("%s: client->irq is empty, use polling mode.\n", __func__);
	}

	if (!ts->use_irq) {
		ts->himax_wq = create_singlethread_workqueue("himax_touch");
		if (!ts->himax_wq)
			goto err_create_wq_failed;

		INIT_WORK(&ts->work, himax_ts_work_func);

		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = himax_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		I("%s: polling mode enabled\n", __func__);
	}
	return 0;

err_create_wq_failed:
err_get_intr_bit_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_detect_failed:
err_check_offmode_charging:
err_dt_platform_data_fail:
	kfree(pdata);
	
err_get_platform_data_fail:
err_alloc_dt_pdata_failed:
	kfree(ts);
	
err_alloc_data_failed:
err_check_functionality_failed:
	return err;

}

static int himax8528_remove(struct i2c_client *client)
{
	struct himax_ts_data *ts = i2c_get_clientdata(client);

	himax_touch_sysfs_deinit();
#ifdef CONFIG_FB
	if (fb_unregister_client(&ts->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif

	if (!ts->use_irq)
		hrtimer_cancel(&ts->timer);

	destroy_workqueue(ts->himax_wq);

	if (ts->protocol_type == PROTOCOL_TYPE_B)
		input_mt_destroy_slots(ts->input_dev);

	input_unregister_device(ts->sr_input_dev);
	input_unregister_device(ts->input_dev);
	kfree(ts->diag_mutual);
	kfree(ts);

	return 0;

}
#if defined (CONFIG_UX500_SOC_DB8500)
static int himax8528_hw_enable(struct himax_ts_data *data)
{
	int err=0;
	struct himax_i2c_platform_data *pdata = data->pdata;

	

	if (pdata->enable) {
		D("Calling hook enable()\n");
		err = pdata->enable(data->dev, pdata);
		if (err) {
			E("Error enabling hardware. err=%d\n",	err);
		goto out;
		}
	}

	
	
	if (err)
		goto out;
out:
	
	return err;
}
static int himax8528_hw_disable(struct himax_ts_data *data)
{
	int err = 0;
	struct himax_i2c_platform_data *pdata = data->pdata;

	

	
	if (pdata->disable) {
		D("Calling hook disable()\n");
		err = pdata->disable(data->dev, pdata);
		if (err) {
			E("Error disabling hardware. err=%d\n", err);
			goto out;
		}
	}

out:
	
	return err;
}
#endif

static int himax8528_suspend(struct device *dev)
{
	int ret;
	uint8_t buf[2] = {0};
	struct himax_ts_data *ts = dev_get_drvdata(dev);

	if(ts->suspended)
	{
		I("%s: Already suspended. Skipped.\n", __func__);
		return 0;
	}
	else
	{
		ts->suspended = true;
		I("%s: enter\n", __func__);
	}

	if (atomic_read(&ts->in_flash)) {
		I("[FW] flashing firmware, won't enter deep sleep now.\n");
		atomic_set(&ts->suspend_mode, 1);
		ts->first_pressed = 0;
		ts->pre_finger_mask = 0;
		return 0;
	}
	#ifdef HX_TP_SYS_FLASH_DUMP
	if (getFlashDumpGoing())
	{
		I("[himax] %s: Flash dump is going, reject suspend\n",__func__);
		return 0;
	}
	#endif

	himax_int_enable(0);

	
	buf[0] = HX_CMD_TSSOFF;
	ret = i2c_himax_master_write(ts->client, buf, 1, HIMAX_I2C_RETRY_TIMES);
	if (ret < 0)
	{
		E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts->client->addr);
	}
	hr_msleep(30);

	buf[0] = HX_CMD_TSSLPIN;
	ret = i2c_himax_master_write(ts->client, buf, 1, HIMAX_I2C_RETRY_TIMES);
	if (ret < 0)
	{
		E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts->client->addr);
	}
	hr_msleep(30);

	buf[0] = HX_CMD_SETDEEPSTB;
	buf[1] = 0x01;
	ret = i2c_himax_master_write(ts->client, buf, 2, HIMAX_I2C_RETRY_TIMES);
	if (ret < 0)
	{
		E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts->client->addr);
	}

	
	#ifdef ENABLE_CHIP_STATUS_MONITOR
	ts->running_status = 1;
	cancel_delayed_work_sync(&ts->himax_chip_monitor);
	#endif
	

	if (!ts->use_irq) {
		ret = cancel_work_sync(&ts->work);
		if (ret)
			himax_int_enable(1);
	}

	
	atomic_set(&ts->suspend_mode, 1);
	ts->pre_finger_mask = 0;
	if (ts->pdata->powerOff3V3 && ts->pdata->power)
		ts->pdata->power(0);

	#if defined (CONFIG_UX500_SOC_DB8500)
	himax8528_hw_disable(ts);
	#endif

	return 0;
}

static int himax8528_resume(struct device *dev)
{
	int ret = 0;
	uint8_t buf[5] = { 0 };

	struct himax_ts_data *ts = dev_get_drvdata(dev);

	I("%s: enter\n", __func__);

	if (ts->pdata->powerOff3V3 && ts->pdata->power)
		ts->pdata->power(1);

	#if defined (CONFIG_UX500_SOC_DB8500)
	himax8528_hw_enable(ts);
	#endif

	
	buf[0] = HX_CMD_SETDEEPSTB;	
	buf[1] = 0x00;
	ret = i2c_himax_master_write(ts->client, buf, 2, HIMAX_I2C_RETRY_TIMES);
	if (ret < 0)
	{
	    E("[himax] %s: I2C access failed addr = 0x%x\n", __func__, ts->client->addr);
	}
	hr_msleep(5);
	
	i2c_himax_write_command(ts->client, 0x83, HIMAX_I2C_RETRY_TIMES);
	hr_msleep(30);
	i2c_himax_write_command(ts->client, 0x81, HIMAX_I2C_RETRY_TIMES);
	atomic_set(&ts->suspend_mode, 0);
	ts->just_resume = 1;

	himax_int_enable(1);

	ts->suspended = false;
	return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct himax_ts_data *ts=
		container_of(self, struct himax_ts_data, fb_notif);

	I(" %s\n", __func__);
	if (evdata && evdata->data && event == FB_EVENT_BLANK && ts &&
			ts->client) {
		blank = evdata->data;
		switch (*blank) {
		case FB_BLANK_UNBLANK:
			himax8528_resume(&ts->client->dev);
		break;

		case FB_BLANK_POWERDOWN:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_NORMAL:
			himax8528_suspend(&ts->client->dev);
		break;
		}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void himax_ts_early_suspend(struct early_suspend *h)
{
	struct himax_ts_data *ts;
	ts = container_of(h, struct himax_ts_data, early_suspend);
	himax8528_suspend(ts->client, PMSG_SUSPEND);
}

static void himax_ts_late_resume(struct early_suspend *h)
{
	struct himax_ts_data *ts;
	ts = container_of(h, struct himax_ts_data, early_suspend);
	himax8528_resume(ts->client);

}
#endif

static const struct dev_pm_ops himax8528_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend = himax8528_suspend,
	.resume  = himax8528_resume,
#else
	.suspend = himax8528_suspend,
#endif
};

static const struct i2c_device_id himax8528_ts_id[] = {
	{HIMAX8528_NAME, 0 },
	{}
};

#ifdef CONFIG_OF
static struct of_device_id himax_match_table[] = {
	{.compatible = "himax,8528d" },
	{},
};
#else
#define himax_match_table NULL
#endif

static struct i2c_driver himax8528_driver = {
	.id_table	= himax8528_ts_id,
	.probe		= himax8528_probe,
	.remove		= himax8528_remove,
	.driver		= {
		.name = HIMAX8528_NAME,
		.owner = THIS_MODULE,
		.of_match_table = himax_match_table,
#ifdef CONFIG_PM
		.pm		= &himax8528_pm_ops,
#endif
	},
};
static void __devinit himax8528_init_async(void *unused, async_cookie_t cookie)
{
	i2c_add_driver(&himax8528_driver);
}

static int __init himax8528_init(void)
{
	async_schedule(himax8528_init_async, NULL);
	return 0;
}

static void __exit himax8528_exit(void)
{
	i2c_del_driver(&himax8528_driver);
}

module_init(himax8528_init);
module_exit(himax8528_exit);

MODULE_DESCRIPTION("Himax8528 driver");
MODULE_LICENSE("GPL");
