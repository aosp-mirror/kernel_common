/* drivers/input/touchscreen/synaptics_3200.c - Synaptics 3200 serious touch panel driver
 *
 * Copyright (C) 2011 HTC Corporation.
 *
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

#include <linux/module.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/slab.h>
#include <linux/rmi.h>
#include <mach/msm_hsusb.h>
#include <asm/gpio.h>
#include <linux/input/mt.h>
#include <linux/pl_sensor.h>
#include <linux/hall_sensor.h>
#include <mach/board.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <mach/devices_cmdline.h>
#endif
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#if defined(CONFIG_SYNC_TOUCH_STATUS)
#include <linux/CwMcuSensor.h>
#endif

#define MAX_BUF_SIZE	256
#define VKEY_VER_CODE	"0x01"
#define SYN_I2C_RETRY_TIMES (10)
#define SYN_UPDATE_RETRY_TIMES (5)
#define SHIFT_BITS (10)
#define SYN_WIRELESS_DEBUG
#define SYN_CALIBRATION_CONTROL

#ifdef SYN_CONFIG_LOG_ENABLE
#define syn_cfg_log(d) do { 	\
	uint8_t i, j; \
	printk("[TP] %02X %02X|", d[0], d[1]); \
	for (i = 0; i < 4; i++) { \
		for (j = 0; j < 4; j++) \
			printk("%02X ", d[2+j+4*i]); \
		if (i != 3) printk(","); \
	} \
	printk("|%02X\n", d[18]); \
} while (0)
#endif

struct synaptics_rmi4_f12_query_5 {
	union {
		struct {
			unsigned char size_of_query6;
			struct {
				unsigned char ctrl0_is_present:1;
				unsigned char ctrl1_is_present:1;
				unsigned char ctrl2_is_present:1;
				unsigned char ctrl3_is_present:1;
				unsigned char ctrl4_is_present:1;
				unsigned char ctrl5_is_present:1;
				unsigned char ctrl6_is_present:1;
				unsigned char ctrl7_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl8_is_present:1;
				unsigned char ctrl9_is_present:1;
				unsigned char ctrl10_is_present:1;
				unsigned char ctrl11_is_present:1;
				unsigned char ctrl12_is_present:1;
				unsigned char ctrl13_is_present:1;
				unsigned char ctrl14_is_present:1;
				unsigned char ctrl15_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl16_is_present:1;
				unsigned char ctrl17_is_present:1;
				unsigned char ctrl18_is_present:1;
				unsigned char ctrl19_is_present:1;
				unsigned char ctrl20_is_present:1;
				unsigned char ctrl21_is_present:1;
				unsigned char ctrl22_is_present:1;
				unsigned char ctrl23_is_present:1;
			} __packed;
			struct {
				unsigned char ctrl24_is_present:1;
				unsigned char ctrl25_is_present:1;
				unsigned char ctrl26_is_present:1;
				unsigned char ctrl27_is_present:1;
				unsigned char ctrl28_is_present:1;
				unsigned char ctrl29_is_present:1;
				unsigned char ctrl30_is_present:1;
				unsigned char ctrl31_is_present:1;
			} __packed;
		};
		unsigned char data[5];
	};
};

struct synaptics_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct input_dev *sr_input_dev;
	struct workqueue_struct *syn_wq;
	struct function_t *address_table;
	int use_irq;
	int gpio_irq;
	int gpio_reset;
	int gpio_i2c;
	struct hrtimer timer;
	struct work_struct  work;
	uint16_t max[2];
	uint32_t flags;
	uint8_t num_function;
	uint8_t finger_support;
	uint16_t finger_pressed;
	int (*power)(int on);
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	int pre_finger_data[11][4];
	uint32_t debug_log_level;
	uint32_t enable_noise_log;
	uint32_t raw_base;
	uint32_t raw_ref;
	uint64_t timestamp;
	uint16_t *filter_level;
#ifdef CONFIG_MOTION_FILTER
	uint8_t *reduce_report_level;
#endif
	unsigned long tap_timeout[10];
	int16_t *report_data;
	int32_t *report_data_32;
	uint8_t *temp_report_data;
	uint8_t grip_suppression;
	uint8_t grip_b_suppression;
	uint16_t tap_suppression;
	uint8_t ambiguous_state;
	uint8_t diag_command;
	uint8_t cable_support;
	uint8_t cable_config;
	uint8_t key_number;
	uint16_t key_postion_x[4];
	uint16_t key_postion_y;
	uint8_t intr_bit;
	uint8_t finger_count;
	uint8_t page_select;
	uint8_t config_table[SYN_CONFIG_SIZE_35XX];
	uint8_t x_channel;
	uint8_t y_channel;
	uint8_t config[SYN_CONFIG_SIZE_35XX];
	uint32_t config_version;
	uint16_t package_id;
	uint32_t packrat_number;
	int layout[4];
	uint8_t htc_event;
	atomic_t data_ready;
	uint8_t relaxation;
	uint8_t irq_enabled;
	uint8_t large_obj_check;
	uint8_t default_large_obj;
	uint16_t tw_vendor;
	uint16_t tw_pin_mask;
	uint8_t support_htc_event;
	uint8_t mfg_flag;
	uint8_t first_pressed;
	uint8_t segmentation_bef_unlock;
	uint8_t segmentation_aft_unlock;
	uint8_t psensor_status;
	uint8_t i2c_err_handler_en;
	uint8_t threshold_bef_unlock;
	uint8_t threshold_aft_unlock;
	uint16_t saturation_bef_unlock;
	uint16_t saturation_aft_unlock;
	uint8_t energy_ratio_relaxation;
	uint8_t multitouch_calibration;
	uint32_t width_factor;
	uint32_t height_factor;
	uint8_t finger_func_idx;
	uint8_t psensor_detection;
	uint8_t psensor_resume_enable;
	uint8_t psensor_phone_enable;
	uint8_t PixelTouchThreshold_bef_unlock;
	uint8_t PixelTouchThreshold_aft_unlock;
	struct work_struct  psensor_work;
	struct workqueue_struct *syn_psensor_wq;
	struct workqueue_struct *syn_att_wq;
	struct work_struct  cover_work;
	struct workqueue_struct *syn_cover_wq;
	struct delayed_work work_att;
	struct synaptics_virtual_key *button;
	char *vkey_buf;
	bool blmode;
	uint8_t ctrl_10_offset;
	uint8_t ctrl_15_offset;
	uint8_t ctrl_23_offset;
	uint8_t cover_setting[9];
	uint8_t uncover_setting[9];
	uint8_t cover_enable;
	uint8_t hover_mode;
	uint8_t i2c_to_mcu;
	uint8_t f12_ctrl10[2];
	uint8_t f12_ctrl15[2];
	uint8_t f54_ctrl89[8];
	uint8_t f54_ctrl91[5];
	uint16_t hall_block_touch_time;
	uint8_t hall_block_touch_event;
	bool suspended;
	int prev_NS;
	int prev_Freq;
#if defined(CONFIG_SECURE_TOUCH)
    atomic_t st_enabled;
    atomic_t st_pending_irqs;
    struct completion st_powerdown;
    struct completion st_irq_processed;
#endif
};

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif
#if defined(CONFIG_SYNC_TOUCH_STATUS)
static void switch_sensor_hub(struct synaptics_ts_data* ts, int mode);
#endif
static DECLARE_WAIT_QUEUE_HEAD(syn_data_ready_wq);
static DEFINE_MUTEX(syn_mutex);

static struct synaptics_ts_data *gl_ts;
static uint16_t syn_panel_version;
static uint8_t vk_press;

static int i2c_syn_write_byte_data(struct i2c_client *client, uint16_t addr, uint8_t value);
static int syn_pdt_scan(struct synaptics_ts_data *ts, int num_page);
static int synaptics_init_panel(struct synaptics_ts_data *ts);
static int i2c_syn_reset_handler(struct synaptics_ts_data *ts, uint8_t reset, char *reason, const char *fun_name);

static irqreturn_t synaptics_irq_thread(int irq, void *ptr);

extern unsigned int get_tamper_sf(void);

static DEFINE_MUTEX(syn_block_mutex);
static void syn_block_touch(struct synaptics_ts_data *ts, int enable)
{
       mutex_lock(&syn_block_mutex);
       ts->hall_block_touch_event = enable;
       mutex_unlock(&syn_block_mutex);
}

static void syn_block_touch_work_func(struct work_struct *dummy)
{
       struct synaptics_ts_data *ts = gl_ts;
       syn_block_touch(ts, 0);
}
static DECLARE_DELAYED_WORK(syn_block_touch_work, syn_block_touch_work_func);

static void syn_handle_block_touch(struct synaptics_ts_data *ts, int enable)
{
       int ret;
       if (ts->hall_block_touch_event) {
               ret = __cancel_delayed_work(&syn_block_touch_work);
               syn_block_touch(ts, 0);
       }
       if (enable) {
               pr_info("[TP][HL] %s: %d\n", __func__, ts->hall_block_touch_time);
               ret = schedule_delayed_work(&syn_block_touch_work, HZ*ts->hall_block_touch_time/1000);
               syn_block_touch(ts, 1);
       }
}

static void syn_page_select(struct i2c_client *client, uint8_t page)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	if (page ^ ts->page_select) {
		i2c_smbus_write_byte_data(client, 0xFF, page);
		ts->page_select = page;
	}
}

static int i2c_syn_read(struct i2c_client *client, uint16_t addr, uint8_t *data, uint16_t length)
{
	uint8_t retry, buf;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};
	buf = addr & 0xFF;

	mutex_lock(&syn_mutex);
	syn_page_select(client, addr >> 8);
	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;
		msleep(10);
	}
	mutex_unlock(&syn_mutex);

	if (retry == SYN_I2C_RETRY_TIMES) {
		pr_info("[TP] i2c_read retry over %d\n",
			SYN_I2C_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

static int i2c_syn_write(struct i2c_client *client, uint16_t addr, uint8_t *data, uint16_t length)
{
	uint8_t retry;
	uint8_t buf[length + 1];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	mutex_lock(&syn_mutex);
	syn_page_select(client, addr >> 8);

	buf[0] = addr & 0xFF;
	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		mdelay(10);
	}
	mutex_unlock(&syn_mutex);

	if (retry == SYN_I2C_RETRY_TIMES) {
		pr_info("[TP] i2c_write retry over %d\n",
			SYN_I2C_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

int i2c_rmi_read(uint16_t addr, uint8_t *data, uint16_t length)
{
	uint8_t retry, buf;
	struct synaptics_ts_data *ts = gl_ts;
	struct i2c_msg msg[] = {
		{
			.addr = ts->client->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = ts->client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};
	buf = addr & 0xFF;

	mutex_lock(&syn_mutex);
	syn_page_select(ts->client, addr >> 8);
	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(ts->client->adapter, msg, 2) == 2)
			break;
		msleep(10);
	}
	mutex_unlock(&syn_mutex);

	if (retry == SYN_I2C_RETRY_TIMES) {
		pr_info("[TP] i2c_read retry over %d\n",
			SYN_I2C_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(i2c_rmi_read);

int i2c_rmi_write(uint16_t addr, uint8_t *data, uint16_t length)
{
	uint8_t retry;
	uint8_t buf[length + 1];
	struct synaptics_ts_data *ts = gl_ts;
	struct i2c_msg msg[] = {
		{
			.addr = ts->client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	mutex_lock(&syn_mutex);
	syn_page_select(ts->client, addr >> 8);

	buf[0] = addr & 0xFF;
	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(ts->client->adapter, msg, 1) == 1)
			break;
		mdelay(10);
	}
	mutex_unlock(&syn_mutex);

	if (retry == SYN_I2C_RETRY_TIMES) {
		pr_info("[TP] i2c_write retry over %d\n",
			SYN_I2C_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(i2c_rmi_write);

static int i2c_syn_write_byte_data(struct i2c_client *client, uint16_t addr, uint8_t value)
{
	return i2c_syn_write(client, addr, &value, 1);
}

static int i2c_syn_error_handler(struct synaptics_ts_data *ts, uint8_t reset, char *reason, const char *fun_name)
{
	int ret;

	if (reason && fun_name)
		printk(KERN_ERR "[TP] TOUCH_ERR: I2C Error: %s:%s, reset = %d\n", fun_name, reason, reset);
	else
		pr_info("[TP] %s: rason and fun_name can't be null\n", __func__);

	if (reset) {
		if (ts->power) {
			ret = ts->power(0);
			if (ret < 0)
				printk(KERN_ERR "[TP] TOUCH_ERR: synaptics i2c error handler power off failed\n");
			msleep(10);
			ret = ts->power(1);
			if (ret < 0)
				printk(KERN_ERR "[TP] TOUCH_ERR: synaptics i2c error handler power on failed\n");
			ret = synaptics_init_panel(ts);
			if (ret < 0)
				printk(KERN_ERR "[TP] TOUCH_ERR: synaptics i2c error handler init panel failed\n");
		} else if (ts->gpio_reset) {
			gpio_direction_output(ts->gpio_reset, 0);
			msleep(1);
			gpio_direction_output(ts->gpio_reset, 1);
			pr_info("[TP] %s: synaptics touch chip reseted.\n", __func__);
		}

		if (!ts->use_irq) {
			hrtimer_cancel(&ts->timer);
			hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		}
	}

	return -EIO;
}

static int i2c_syn_reset_handler(struct synaptics_ts_data *ts, uint8_t reset, char *reason, const char *fun_name)
{
	int ret;

	if (reset) {
		if (ts->power) {
			ret = ts->power(0);
			if (ret < 0)
				printk(KERN_ERR "[TP] TOUCH_ERR: synaptics i2c reset handler power off failed\n");
			msleep(10);
			ret = ts->power(1);
			if (ret < 0)
				printk(KERN_ERR "[TP] TOUCH_ERR: synaptics i2c reset handler power on failed\n");
			ret = synaptics_init_panel(ts);
			if (ret < 0)
				printk(KERN_ERR "[TP] TOUCH_ERR: synaptics i2c reset handler init panel failed\n");
		} else if (ts->gpio_reset) {
			gpio_direction_output(ts->gpio_reset, 0);
			msleep(1);
			gpio_direction_output(ts->gpio_reset, 1);
			pr_info("[TP] %s: synaptics touch chip reseted.\n", __func__);
		}

		if (!ts->use_irq) {
			hrtimer_cancel(&ts->timer);
			hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		}
	}

	return -EIO;
}

static int get_address_base(struct synaptics_ts_data *ts, uint8_t command, uint8_t type)
{
	uint8_t i;
	for (i = 0; i < ts->num_function; i++) {
		if (ts->address_table[i].function_type == command) {
			switch (type) {
			case QUERY_BASE:
				return ts->address_table[i].query_base;
			case COMMAND_BASE:
				return ts->address_table[i].command_base;
			case CONTROL_BASE:
				return ts->address_table[i].control_base;
			case DATA_BASE:
				return ts->address_table[i].data_base;
			case INTR_SOURCE:
				return ts->address_table[i].interrupt_source;
			case FUNCTION:
				return 1;
			}
		}
	}
	if (type == FUNCTION)
		return 0;
	else
		return -1;
}
static int get_int_mask(uint8_t number, uint8_t offset)
{
	uint8_t i, mask = 0;
	for (i = 0; i < number; i++)
		mask |= BIT(i);
	return mask << offset;
}

static uint32_t syn_crc(uint16_t *data, uint16_t len)
{
	uint32_t sum1, sum2;
	sum1 = sum2 = 0xFFFF;
	if (data) {
		while (len--) {
			sum1 += *data++;
			sum2 += sum1;
			sum1 = (sum1 & 0xFFFF) + (sum1 >> 16);
			sum2 = (sum2 & 0xFFFF) + (sum2 >> 16);
			
		}
	} else {
		pr_err("[TP] %s: data incorrect", __func__);
		return (0xFFFF | 0xFFFF << 16);
	}
	return sum1 | (sum2 << 16);
}

static int wait_flash_interrupt(struct synaptics_ts_data *ts, int attr, int fw)
{
	uint16_t reg = 0;
	uint8_t data = 0;
	int i, ret;

	for (i = 0; i < 5000; i++) {
#ifdef SYN_FLASH_PROGRAMMING_LOG
		pr_info("[TP] ATT: %d\n", gpio_get_value(attr));
#endif
		if (!gpio_get_value(attr)) {
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x01, DATA_BASE) + 1, &data, 1);
			if (ret < 0)
				return i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "r:1", __func__);
			if ((data & 0x01) == 0x01) {
#ifdef SYN_FLASH_PROGRAMMING_LOG
				pr_info("[TP] ATT: %d, status: %x\n", gpio_get_value(attr), data);
#endif
				break;
			}
		}
		if (fw)
			mdelay(2);
		else
			msleep(20);
	}

	if (i == 5000 && syn_panel_version == 0) {
		ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, DATA_BASE) + 1, &data, 1);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:2", __func__);
	} else if (i == 5000) {
		pr_info("[TP] %s: interrupt over time!\n", __func__);
		return SYN_PROCESS_ERR;
	}

	reg = get_address_base(ts, 0x34, DATA_BASE) + ((ts->package_id < 3400) ? 0x12 : 0x03);
	ret = i2c_syn_read(ts->client, reg, &data, 1);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:3", __func__);
	
	if (!(data & 0x80)) {
		pr_info("[TP] Not in program enable mode, F34_data = %x\n", data);
		ret = i2c_syn_read(ts->client,
			get_address_base(ts, 0x01, DATA_BASE), &data, 1);
		pr_info("[TP] Not in program enable mode, 01_data = %x\n", data);
		return SYN_PROCESS_ERR;
	} else if (data != 0x80) {
		pr_info("[TP] Not in program enable mode, data = %x\n", data);
	}
	return 0;
}

static int enable_flash_programming(struct synaptics_ts_data *ts, int attr)
{
	int ret;
	uint8_t  data[2] = {0};
	uint16_t reg = 0;
	if (attr) {
		if (!gpio_get_value(attr)) {
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x01, DATA_BASE) + 1, data, 1);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:1", __func__);
			pr_info("[TP] %s: clear gpio_irq before enter bootloader mode, data = %x\n", __func__, data[0]);
		}
	}
	
	ret = i2c_syn_read(ts->client,
		get_address_base(ts, 0x34, QUERY_BASE), data, 2);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:1", __func__);

	reg = get_address_base(ts, 0x34, DATA_BASE) + ((ts->package_id < 3400) ? 2 : 1);
	ret = i2c_syn_write(ts->client, reg, data, 2);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:2", __func__);

	reg = get_address_base(ts, 0x34, DATA_BASE) + ((ts->package_id < 3400) ? 18 : 2);
	ret = i2c_syn_write_byte_data(ts->client, reg, 0x0F);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:3", __func__);

	ret = wait_flash_interrupt(ts, attr, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static int crc_comparison(struct synaptics_ts_data *ts, uint32_t config_crc, int attr)
{
	int ret;
	uint8_t data[17];
	uint32_t flash_crc, regaddr = 0;
#ifdef SYN_FLASH_PROGRAMMING_LOG
	uint8_t i, j;

	for (i = 0; i < (ts->package_id > 3500 ? 0x40 : 0x20) ; i++) {
		data[0] = i;
		data[1] = 0x00;
#else
		data[0] = (ts->package_id > 3500 ? 0x3F : 0x1F);
		data[1] = 0x00;
#endif
		ret = i2c_syn_write(ts->client,
			get_address_base(ts, 0x34, DATA_BASE), data, 2);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:1", __func__);

		regaddr = get_address_base(ts, 0x34, DATA_BASE) + ((ts->package_id < 3400) ? 18 : 2);
		ret = i2c_syn_write_byte_data(ts->client, regaddr, 0x05);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:2", __func__);

		ret = wait_flash_interrupt(ts, attr, 0);
		if (ret < 0)
			return ret;

		regaddr = get_address_base(ts, 0x34, DATA_BASE) + ((ts->package_id < 3400) ? 2 : 1);
		ret = i2c_syn_read(ts->client, regaddr , data, 17);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:3", __func__);

		memcpy(&flash_crc, &data[12], 4);

		pr_info("[TP] config_crc = %X, flash_crc = %X\n", config_crc, flash_crc);
#ifdef SYN_FLASH_PROGRAMMING_LOG
		for (j = 0; j < 0x11; j++)
			pr_info(" %d:%X ", j, data[j]);
		pr_info("\n");
	}
#endif
	if (flash_crc == config_crc)
		return 0;
	else
		return 1;
}

static int program_config(struct synaptics_ts_data *ts, uint8_t *config, int attr)
{
	int ret;
	uint8_t data[19] = {0};
	uint16_t i, reg = 0;

	ret = i2c_syn_read(ts->client,
		get_address_base(ts, 0x34, QUERY_BASE), data, 2);
	

	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:1", __func__);

	reg = get_address_base(ts, 0x34, DATA_BASE) + ((ts->package_id < 3400) ? 2 : 1);
	ret = i2c_syn_write(ts->client, reg, data, 2);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:2", __func__);

	

	reg = get_address_base(ts, 0x34, DATA_BASE) + ((ts->package_id < 3400) ? 18 : 2);
	ret = i2c_syn_write_byte_data(ts->client, reg, 0x07);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:3", __func__);

	ret = wait_flash_interrupt(ts, attr, 0);
	if (ret < 0)
		return ret;

	for (i = 0; i < (ts->package_id > 3500 ? 0x40 : 0x20); i++) {	
		data[0] = i & 0xFF;
		data[1] = (i & 0xFF00) >> 8;
		memcpy(&data[2], &config[SYN_CFG_BLK_UNIT * i], SYN_CFG_BLK_UNIT);
		data[18] = 0x06;
#ifdef SYN_CONFIG_LOG_ENABLE
		syn_cfg_log(data);
#endif
		ret = i2c_syn_write(ts->client, get_address_base(ts, 0x34, DATA_BASE), data, 19);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:4", __func__);

		ret = wait_flash_interrupt(ts, attr, 0);
		if (ret < 0)
			return ret;
	}
	return 0;
}

static int disable_flash_programming(struct synaptics_ts_data *ts, int status)
{
	int ret;
	uint8_t data = 0, i;

	ret = i2c_syn_write_byte_data(ts->client,
		get_address_base(ts, 0x01, COMMAND_BASE), 0x01);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:1", __func__);

	for (i = 0; i < 25; i++) {
		ret = i2c_syn_read(ts->client,
			get_address_base(ts, 0x01, DATA_BASE), &data, 1);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:2", __func__);

		if ((data & 0x40) == 0)
			break;
		else
			msleep(20);
	}

	if (i == 25) {
		pr_info("[TP] Disable flash programming fail! F01_data: %X\n", data);
		return SYN_PROCESS_ERR;
	} else {
		pr_info("[TP] Disable flash programming success! F01_data: %X\n", data);
		return status;
	}
}

static int syn_config_update(struct synaptics_ts_data *ts, int attr)
{
	uint8_t retry;
	uint32_t crc_checksum;
	int ret;

	crc_checksum = syn_crc((uint16_t *)ts->config,
			       (ts->package_id > 3500 ? SYN_CONFIG_SIZE_35XX : SYN_CONFIG_SIZE)/ 2 - 2);
	if (crc_checksum == 0xFFFFFFFF) {
		pr_err("[TP] %s: crc_checksum Error", __func__);
		return -1;
	}
	memcpy(&ts->config[(ts->package_id > 3500 ? SYN_CONFIG_SIZE_35XX : SYN_CONFIG_SIZE) - 4], &crc_checksum, 4);
	pr_info("[TP] CRC_cksum=%X ", crc_checksum);

	if (ts->tw_pin_mask == 0) {
		ret = enable_flash_programming(ts, attr);
		if (ret < 0) {
			pr_info("[TP] %s: Enable flash programming fail!\n", __func__);
			return disable_flash_programming(ts, ret);
		}

		ret = syn_pdt_scan(ts, SYN_BL_PAGE);
		if (ret < 0) {
			pr_info("[TP] %s: pdt scan failed\n", __func__);
			return disable_flash_programming(ts, ret);
		}
	}

	if ((ts->config != NULL && (ts->config[0] << 24 | ts->config[1] << 16 |
		ts->config[2] << 8 | ts->config[3]) == ts->config_version)) {
		ret = crc_comparison(ts, crc_checksum, attr);
		if (ret < 0) {
			pr_info("[TP] %s: CRC comparison fail!\n", __func__);
			return disable_flash_programming(ts, ret);
		} else if (ret == 0)
			return disable_flash_programming(ts, 1);
	}

	for (retry = 0; retry < 3; retry++) {
		ret = program_config(ts, ts->config, attr);
		if (ret < 0) {
#ifdef SYN_FLASH_PROGRAMMING_LOG
			pr_info("[TP] %s: Program config fail %d!\n", __func__, retry + 1);
#endif
			continue;
		}

		ret = disable_flash_programming(ts, 0);
		if (ret == 0)
			break;
		else
			pr_info("[TP] %s: Disable flash programming fail %d\n", __func__, retry + 1);
	}

	if (retry == 3) {
		pr_info("[TP] %s: Program config fail 3 times\n", __func__);
		return ret;
	}
	return 0;
}

static int syn_get_tw_vendor(struct synaptics_ts_data *ts, int attr)
{
	uint8_t data[6] = {0};
	int ret;

	ret = enable_flash_programming(ts, attr);
	if (ret < 0) {
		pr_info("[TP] Enable flash programming fail!\n");
		return disable_flash_programming(ts, -1);
	}

	ret = syn_pdt_scan(ts, SYN_BL_PAGE);
	if (ret < 0) {
		pr_info("[TP] %s: pdt scan failed\n", __func__);
		return disable_flash_programming(ts, ret);
	}

	memcpy(&data, &ts->tw_pin_mask, sizeof(ts->tw_pin_mask));
	pr_info("[TP] tw mask = %X %X , %X\n", data[0], data[1], ts->tw_pin_mask);
	data[2] = data[0];
	data[3] = data[1];
	if (ts->package_id < 3400) {
		i2c_syn_write(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 2, data, 2);
		i2c_syn_write(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 4, data, 2);
		i2c_syn_write_byte_data(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 18, 0x08);
	} else {
		i2c_syn_write(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 1, data, 4);
		i2c_syn_write_byte_data(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 2, 0x08);
	}

	if (wait_flash_interrupt(ts, attr, 0) < 0)
		return disable_flash_programming(ts, -1);

	if (ts->package_id < 3400) {
		i2c_syn_read(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 6, data, 2);
		ts->tw_vendor = (data[1] << 8) | data[0];
		pr_info("[TP] tw vendor= %x %x\n", data[1], data[0]);
	} else {
		i2c_syn_read(ts->client,
			get_address_base(ts, 0x34, DATA_BASE) + 1, data, 6);
		ts->tw_vendor = (data[5] << 8) | data[4];
		pr_info("[TP] tw vendor= %x %x\n", data[5], data[4]);
	}

	return 0;
}

static void syn_set_cover_func(struct work_struct *work)
{
	int ret;
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, cover_work);

	if (!ts)
	{
		pr_info("[TP] %s null\n", __func__);
		return;
	}
	else
	{
		pr_info("[TP] %s: %d\n", __func__, ts->cover_enable);
	}

	if (ts->cover_enable) {
		if (ts->package_id == 3528 )
		{
			if (ts->packrat_number >= SYNAPTICS_FW_35_COVER) {
				ts->f12_ctrl10[1] = ts->cover_setting[0];
				ts->f12_ctrl10[0] = ts->cover_setting[5];
				ret = i2c_syn_write(ts->client, get_address_base(ts, 0x12, CONTROL_BASE) + ts->ctrl_10_offset, ts->f12_ctrl10, 2);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:1", __func__);
					return;
				}

				ts->f12_ctrl15[1] = ts->cover_setting[7];
				ts->f12_ctrl15[0] = ts->cover_setting[1];
				ret = i2c_syn_write(ts->client, get_address_base(ts, 0x12, CONTROL_BASE) + ts->ctrl_15_offset, ts->f12_ctrl15, 2);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:2", __func__);
					return;
				}

				ret = i2c_syn_write_byte_data(ts->client, get_address_base(ts, 0x51, CONTROL_BASE) + 2, ts->cover_setting[2]);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:3", __func__);
					return;
				}

				ret = i2c_syn_write_byte_data(ts->client, get_address_base(ts, 0x51, CONTROL_BASE) + 3, ts->cover_setting[3]);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:4", __func__);
					return;
				}

				ret = i2c_syn_write_byte_data(ts->client, get_address_base(ts, 0x51, CONTROL_BASE) + 4, ts->cover_setting[4]);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:5", __func__);
					return;
				}

				ret = i2c_syn_write_byte_data(ts->client, get_address_base(ts, 0x54, CONTROL_BASE) + 28, ts->cover_setting[6]);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:6", __func__);
					return;
				}

				ret = i2c_syn_write_byte_data(ts->client, get_address_base(ts, 0x01, CONTROL_BASE) + 3, ts->cover_setting[8]);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:7", __func__);
					return;
				}
			}else {
				ret = i2c_syn_write(ts->client, get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ts->ctrl_10_offset, ts->cover_setting, 2);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:1", __func__);
					return;
				}

				ret = i2c_syn_write(ts->client, get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ts->ctrl_15_offset, &ts->cover_setting[2], 2);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:2", __func__);
					return;
				}

				ts->f54_ctrl89[4] = ts->cover_setting[4];
				ret = i2c_syn_write(ts->client, get_address_base(ts, 0x54, CONTROL_BASE) + 26, ts->f54_ctrl89, 5);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:3", __func__);
					return;
				}

				ret = i2c_syn_write_byte_data(ts->client, get_address_base(ts, 0x54, CONTROL_BASE) + 28, ts->cover_setting[5]);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:4", __func__);
					return;
				}

				ts->f54_ctrl91[4] = ts->cover_setting[6];
				ret = i2c_syn_write(ts->client, get_address_base(ts, 0x54, CONTROL_BASE) + 27, ts->f54_ctrl91, 5);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:5", __func__);
					return;
				}
			}
		}
		else if (ts->package_id == 3508)
		{
			if (ts->packrat_number >= SYNAPTICS_FW_3508_COVER) {
				ret = i2c_syn_write(ts->client, get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ts->ctrl_15_offset, ts->cover_setting, 1);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:1", __func__);
				}
			} else {
				ret = i2c_syn_write(ts->client, get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ts->ctrl_15_offset, ts->cover_setting, 2);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:1", __func__);
				}
			}
		}
	} else {
		if (ts->package_id == 3528 )
		{
			if (ts->packrat_number >= SYNAPTICS_FW_35_COVER) {
				ts->f12_ctrl10[1] = ts->uncover_setting[0];
				ts->f12_ctrl10[0] = ts->uncover_setting[5];
				ret = i2c_syn_write(ts->client, get_address_base(ts, 0x12, CONTROL_BASE) + ts->ctrl_10_offset, ts->f12_ctrl10, 2);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:1", __func__);
					return;
				}

				ts->f12_ctrl15[1] = ts->uncover_setting[7];
				ts->f12_ctrl15[0] = ts->uncover_setting[1];
				ret = i2c_syn_write(ts->client, get_address_base(ts, 0x12, CONTROL_BASE) + ts->ctrl_15_offset, ts->f12_ctrl15, 2);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:2", __func__);
					return;
				}

				ret = i2c_syn_write_byte_data(ts->client, get_address_base(ts, 0x51, CONTROL_BASE) + 2, ts->uncover_setting[2]);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:3", __func__);
					return;
				}

				ret = i2c_syn_write_byte_data(ts->client, get_address_base(ts, 0x51, CONTROL_BASE) + 3, ts->uncover_setting[3]);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:4", __func__);
					return;
				}

				ret = i2c_syn_write_byte_data(ts->client, get_address_base(ts, 0x51, CONTROL_BASE) + 4, ts->uncover_setting[4]);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:5", __func__);
					return;
				}

				ret = i2c_syn_write_byte_data(ts->client, get_address_base(ts, 0x54, CONTROL_BASE) + 28, ts->uncover_setting[6]);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:6", __func__);
					return;
				}

				ret = i2c_syn_write_byte_data(ts->client, get_address_base(ts, 0x01, CONTROL_BASE) + 3, ts->uncover_setting[8]);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:6", __func__);
					return;
				}
			}else {

				if (ts->hall_block_touch_time > 1)
					syn_handle_block_touch(ts, 0);

				ret = i2c_syn_write(ts->client, get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ts->ctrl_10_offset, ts->uncover_setting, 2);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:11", __func__);
					return;
				}

				ret = i2c_syn_write(ts->client, get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ts->ctrl_15_offset, &ts->uncover_setting[2], 2);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:12", __func__);
					return;
				}

				ts->f54_ctrl89[4] = ts->uncover_setting[4];
				ret = i2c_syn_write(ts->client, get_address_base(ts, 0x54, CONTROL_BASE) + 26, ts->f54_ctrl89, 5);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:13", __func__);
					return;
				}

				ret = i2c_syn_write_byte_data(ts->client, get_address_base(ts, 0x54, CONTROL_BASE) + 28, ts->uncover_setting[5]);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:14", __func__);
					return;
				}

				ts->f54_ctrl91[4] = ts->uncover_setting[6];
				ret = i2c_syn_write(ts->client, get_address_base(ts, 0x54, CONTROL_BASE) + 27, ts->f54_ctrl91, 5);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:15", __func__);
					return;
				}
			}
		}
		else if (ts->package_id == 3508)
		{
			if (ts->packrat_number >= SYNAPTICS_FW_3508_COVER) {
				ret = i2c_syn_write(ts->client, get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ts->ctrl_15_offset, ts->uncover_setting, 1);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:1", __func__);
				}
			} else {
				ret = i2c_syn_write(ts->client, get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ts->ctrl_15_offset, ts->uncover_setting, 2);
				if (ret < 0)
				{
					i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "w:2", __func__);
					return;
				}
			}
		}
	}
	return;
}


static int synaptics_input_register(struct synaptics_ts_data *ts)
{
	int ret;
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "[TP] TOUCH_ERR: %s: Failed to allocate input device\n", __func__);
		return ret;
	}
	ts->input_dev->name = "synaptics-rmi-touchscreen";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
	set_bit(KEY_APP_SWITCH, ts->input_dev->keybit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);

	pr_info("[TP] input_set_abs_params: mix_x %d, max_x %d, min_y %d, max_y %d\n",
		ts->layout[0], ts->layout[1], ts->layout[2], ts->layout[3]);

	if (ts->htc_event == SYN_AND_REPORT_TYPE_B) {
		input_mt_init_slots(ts->input_dev, ts->finger_support);
	} else {
		input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0,
			ts->finger_support - 1, 0, 0);
	}
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
		ts->layout[0], ts->layout[1], 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
		ts->layout[2], ts->layout[3], 0, 0);

	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 30, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 30, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_DISTANCE, 0, 255, 0, 0);

	input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE,
			0, ((255 << 16) | 15), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION,
		0, ((1 << 31) | (ts->layout[1] << 16) | ts->layout[3]), 0, 0);

#if defined(CONFIG_SECURE_TOUCH)
    ts->input_dev->id.bustype = BUS_I2C;
    ts->input_dev->dev.parent = &ts->client->dev;
    input_set_drvdata(ts->input_dev, ts);
#endif

	return input_register_device(ts->input_dev);
}

static ssize_t touch_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	char fw_version[2];
	struct synaptics_ts_data *ts;

	ts = gl_ts;
	memcpy(fw_version, &syn_panel_version, 2);
	ret = snprintf(buf, PAGE_SIZE, "synaptics-%d_%c.%c", ts->package_id, fw_version[1], fw_version[0]);
	if (ts->tw_pin_mask != 0)
		ret += snprintf(buf+ret, PAGE_SIZE, "_twID-%x", ts->tw_vendor);
	else
		ret += snprintf(buf+ret, PAGE_SIZE, "\n");

	ret += snprintf(buf+ret, PAGE_SIZE, "_PR: %d\n", ts->packrat_number);

	return ret;
}

static DEVICE_ATTR(vendor, S_IRUGO, touch_vendor_show, NULL);

static ssize_t gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct synaptics_ts_data *ts;

	ts = gl_ts;

	ret = gpio_get_value(ts->gpio_irq);
	printk(KERN_DEBUG "[TP] GPIO_TP_INT_N=%d\n", ret);
	snprintf(buf, PAGE_SIZE, "GPIO_TP_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(gpio, S_IRUGO, gpio_show, NULL);

static uint16_t syn_reg_addr = 0;
static uint8_t syn_reg_leng_data = 1;

static ssize_t register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0, i = 0;
	uint8_t *data;
	struct synaptics_ts_data *ts;
	ts = gl_ts;

	data = kzalloc(sizeof(uint8_t) * syn_reg_leng_data, GFP_KERNEL);

	ret = i2c_syn_read(ts->client, syn_reg_addr, data, syn_reg_leng_data);
	if (ret < 0) {
		i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r", __func__);
		ret += snprintf(buf, PAGE_SIZE, "addr: 0x , data: 0x \n");
	} else {
		ret += snprintf(buf, PAGE_SIZE, "addr: 0x%X, data: 0x\n", syn_reg_addr);
		for (i = 0; i < syn_reg_leng_data; i++) {
			ret += snprintf(buf + ret, PAGE_SIZE, "%2.2X ", data[i]);
			if ((i % 16) == (16 - 1))
				ret += snprintf(buf + ret, PAGE_SIZE, "\n");
		}
		ret += snprintf(buf + ret, PAGE_SIZE, "\n");
	}

	kfree(data);
	return ret;
}

static ssize_t register_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	struct synaptics_ts_data *ts;
	char buf_tmp[4];
	unsigned long first_field, second_field;

	ts = gl_ts;
	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':' &&
		(buf[5] == ':' && buf[9] == '\n')) {
		memcpy(buf_tmp, buf + 2, 3);
		ret = strict_strtoul(buf_tmp, 16, &first_field);
		if (ret < 0) {
			printk(KERN_DEBUG "[TP] 1st_field: 0x \n");
			return ret;
		}

		memcpy(buf_tmp, buf + 6, 3);
		ret = strict_strtoul(buf_tmp, 16, &second_field);
		if (ret < 0) {
			printk(KERN_DEBUG "[TP] 2st_field: 0x \n");
			return ret;
		}

		syn_reg_addr = first_field;
		syn_reg_leng_data = second_field;

		if (buf[0] == 'w') {
			printk(KERN_DEBUG "[TP] write addr: 0x%X, data: 0x%X\n",
						syn_reg_addr, syn_reg_leng_data);
			ret = i2c_syn_write_byte_data(ts->client,
					syn_reg_addr, syn_reg_leng_data);
			if (ret < 0) {
				i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w", __func__);
			}
			syn_reg_leng_data = 1;
		} else if (buf[0] == 'r') {
			if(syn_reg_leng_data < 1)
				syn_reg_leng_data = 1;

			printk(KERN_DEBUG "[TP] read addr: 0x%X, length: %u\n",
						syn_reg_addr, syn_reg_leng_data);
		}
	}else
		printk(KERN_DEBUG "[TP] Command Format Error!\n");

	return count;
}

static DEVICE_ATTR(register, (S_IWUSR|S_IRUGO),
	register_show, register_store);

static ssize_t debug_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = gl_ts;

	return snprintf(buf, PAGE_SIZE, "%d\n", ts->debug_log_level);
}

static ssize_t debug_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = gl_ts;
	int i;

	ts->debug_log_level = 0;
	for(i=0; i<count-1; i++)
	{
		if( buf[i]>='0' && buf[i]<='9' )
			ts->debug_log_level |= (buf[i]-'0');
		else if( buf[i]>='A' && buf[i]<='F' )
			ts->debug_log_level |= (buf[i]-'A'+10);
		else if( buf[i]>='a' && buf[i]<='f' )
			ts->debug_log_level |= (buf[i]-'a'+10);

		if(i!=count-2)
			ts->debug_log_level <<= 4;
	}

	return count;
}

static DEVICE_ATTR(debug_level, (S_IWUSR|S_IRUGO),
	debug_level_show, debug_level_store);

static ssize_t syn_diag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = gl_ts;
	size_t count = 0;
	uint16_t i, j;
	int ret;

	ret = i2c_syn_write_byte_data(ts->client,
		get_address_base(ts, 0x54, DATA_BASE), ts->diag_command);
	if (ret < 0) {
		i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:1", __func__);
		count += snprintf(buf, PAGE_SIZE, "[TP] TOUCH_ERR: %s: i2c write fail(%d)\n", __func__, ret);
		return count;
	}

	atomic_set(&ts->data_ready, 0);

	ret = i2c_syn_write_byte_data(ts->client,
		get_address_base(ts, 0x54, COMMAND_BASE), 0x01);
	if (ret < 0) {
		atomic_set(&ts->data_ready, 1);
		i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:2", __func__);
		count += snprintf(buf, PAGE_SIZE, "[TP] TOUCH_ERR: %s: i2c write fail(%d)\n", __func__, ret);
		return count;
	}

	wait_event_interruptible_timeout(syn_data_ready_wq,
					 atomic_read(&ts->data_ready), 50);

	if (ts->package_id == 3528) {
		if (ts->diag_command == 2)
			ts->diag_command = 40;
		if (ts->diag_command == 3)
			ts->diag_command = 38;

		ret = i2c_syn_write_byte_data(ts->client,
			get_address_base(ts, 0x54, DATA_BASE), ts->diag_command);
		if (ret < 0) {
			i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:1", __func__);
			count += snprintf(buf, PAGE_SIZE, "[TP] TOUCH_ERR: %s: i2c write fail(%d)\n", __func__, ret);
			return count;
		}

		atomic_set(&ts->data_ready, 0);

		ret = i2c_syn_write_byte_data(ts->client,
			get_address_base(ts, 0x54, COMMAND_BASE), 0x01);
		if (ret < 0) {
			atomic_set(&ts->data_ready, 1);
			i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:2", __func__);
			count += snprintf(buf, PAGE_SIZE, "[TP] TOUCH_ERR: %s: i2c write fail(%d)\n", __func__, ret);
			return count;
		}

		wait_event_interruptible_timeout(syn_data_ready_wq,
						 atomic_read(&ts->data_ready), 50);
		if (ts->diag_command == 40)
			ts->diag_command = 2;
		if (ts->diag_command == 38)
			ts->diag_command = 3;
	}

	for (i = 0; i < ts->y_channel; i++) {
		for (j = 0; j < ts->x_channel; j++) {
			switch (ts->package_id) {
			case 3201:
			case 3508:
			case 3528:
				count += snprintf(buf + count, PAGE_SIZE, "%5d", ts->report_data[i*ts->x_channel + j]);
				break;
			default:
				count += snprintf(buf + count, PAGE_SIZE, "%5d", ts->report_data[i + j*ts->y_channel]);
				break;
			}
		}
		if (ts->package_id == 3528)
			count += snprintf(buf + count, PAGE_SIZE, "\t  %5d\n", ts->report_data_32[ts->x_channel+i]);
		count += snprintf(buf + count, PAGE_SIZE, "\n");
	}
	if (ts->package_id == 3528) {
		count += snprintf(buf + count, PAGE_SIZE, "\n");
		for (i = 0; i < ts->x_channel; i+=2)
			count += snprintf(buf + count, PAGE_SIZE, "%5d     ", ts->report_data_32[i]);
		count += snprintf(buf + count, PAGE_SIZE, "\n");
		for (i = 1; i < ts->x_channel; i+=2)
			count += snprintf(buf + count, PAGE_SIZE, "     %5d", ts->report_data_32[i]);
		count += snprintf(buf + count, PAGE_SIZE, "\n");
	}

	return count;
}

static ssize_t syn_diag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts;
	ts = gl_ts;
	if (buf[0] == '1')
		ts->diag_command = 2;
	else if (buf[0] == '2')
		ts->diag_command = 3;

	return count;
}

static DEVICE_ATTR(diag, (S_IWUSR|S_IRUGO),
	syn_diag_show, syn_diag_store);

static ssize_t syn_unlock_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts;
	int unlock = -1;
	int ret;

	ts = gl_ts;

	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		unlock = buf[0] - '0';

	pr_info("[TP] Touch: unlock change to %d\n", unlock);

	if (unlock == 2 && ts->first_pressed && ts->pre_finger_data[0][0] < 2) {
		if (ts->packrat_number < SYNAPTICS_FW_NOCAL_PACKRAT) {
			ts->pre_finger_data[0][0] = 2;
			if (ts->psensor_detection) {
				if (ts->psensor_resume_enable == 1) {
					pr_info("[TP] %s: Disable P-sensor by Touch\n", __func__);
					ts->psensor_resume_enable = 0;
				} else if (ts->psensor_resume_enable == 2) {
					ts->psensor_resume_enable = 0;
				}
			}
#ifdef SYN_CALIBRATION_CONTROL
			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x10, 0x0);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:1", __func__);

			if (ts->energy_ratio_relaxation) {
				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE), 0x0);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:2", __func__);
			}

			if (ts->saturation_bef_unlock) {
				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x02, ts->saturation_aft_unlock & 0xFF);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:3", __func__);
				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x03, (ts->saturation_aft_unlock & 0xFF00) >> 8);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:4", __func__);
				pr_info("[TP] %s: unlock confirmed. set saturation: %x\n"
					, __func__, ts->saturation_aft_unlock);
			}

			if (ts->PixelTouchThreshold_bef_unlock) {
				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x04, ts->PixelTouchThreshold_aft_unlock);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "F54_ANALOG_CTRL03 Pixel Touch Threshold", __func__);
				pr_info("[TP] %s: set F54_ANALOG_CTRL03 Pixel Touch Threshold: %x\n", __func__, ts->PixelTouchThreshold_aft_unlock);
			}


			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, COMMAND_BASE), 0x04);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:5", __func__);

			if (ts->multitouch_calibration) {
				if (ts->package_id < 3400) {
					ret = i2c_syn_write_byte_data(ts->client,
							get_address_base(ts, ts->finger_func_idx, COMMAND_BASE), 0x01);
					if (ret < 0)
						return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:6", __func__);
					pr_info("[TP] %s: Touch Calibration Confirmed, rezero\n", __func__);
				}
			}
#endif
			if (ts->large_obj_check) {
				if (ts->package_id == 2200) {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x26, ts->default_large_obj);

				} else {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x29, ts->default_large_obj);
				}
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:7", __func__);
				pr_info("[TP] %s: unlock confirmed. set large obj suppression: %x\n"
					, __func__, ts->default_large_obj);
			}

			if (ts->segmentation_bef_unlock) {
				if (ts->package_id == 2200) {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x25, ts->segmentation_aft_unlock);

				} else {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x22, ts->segmentation_aft_unlock);
				}
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:8", __func__);
				pr_info("[TP] %s: unlock confirmed. set segmentation aggressiveness: %x\n"
					, __func__, ts->segmentation_aft_unlock);
			}

			if (ts->threshold_bef_unlock) {
				if (ts->package_id == 2200) {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x0A, ts->threshold_aft_unlock);
				} else {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x0C, ts->threshold_aft_unlock);
				}
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:9", __func__);
				pr_info("[TP] %s: unlock confirmed. set Z Touch threshold: %x\n"
					, __func__, ts->threshold_aft_unlock);
			}
		}
	}

	return count;
}

static DEVICE_ATTR(unlock, (S_IWUSR|S_IRUGO),
	NULL, syn_unlock_store);

static ssize_t syn_config_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = gl_ts;
	uint16_t i, length = 0;
	uint8_t j, temp_func_cmd = 0, temp_func_query = 0, size = 0;
	size_t count = 0;
	int ret;

	pr_info("[TP] ts->num_function: %d\n", ts->num_function);
	for (i = 0; i < SYN_MAX_PAGE; i++) {
		for (j = 0; j < ts->num_function; j++) {
			if (((ts->address_table[j].control_base >> 8) & 0xFF) == i) {
				temp_func_query = 0;
				for (temp_func_cmd = j; temp_func_cmd < ts->num_function; temp_func_cmd++) {
					uint16_t max_addr = (i << 8) | 0xFF;
					uint16_t min_addr = (i << 8) | 0;
					if ((ts->address_table[temp_func_cmd].command_base > min_addr) &&
						(ts->address_table[temp_func_cmd].command_base <= max_addr))
						break;
					if ((ts->address_table[temp_func_cmd].query_base > min_addr) &&
						(ts->address_table[temp_func_cmd].query_base <= max_addr)
						&& temp_func_query == 0)
						temp_func_query = temp_func_cmd;
				}

				if (temp_func_cmd != ts->num_function) {
					size = ts->address_table[temp_func_cmd].command_base -
						ts->address_table[j].control_base;
					pr_info("[TP] page%d has command function, function: %X\n"
						, i, ts->address_table[temp_func_cmd].function_type);
				} else {
					size = ts->address_table[temp_func_query].query_base -
						ts->address_table[j].control_base;
					pr_info("[TP] page%d has no command function, use query function, function: %X\n"
						, i, ts->address_table[temp_func_query].function_type);
				}

				ret = i2c_syn_read(ts->client, ts->address_table[j].control_base,
					&ts->config_table[length], size);
				if (ret < 0) {
					i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w", __func__);
					count += snprintf(buf, PAGE_SIZE, "[TP] TOUCH_ERR: %s: i2c write fail(%d)\n", __func__, ret);
					return count;
				}

				length += size;
				pr_info("[TP] Size: %x, Length: %x\n", size, length);
				break;
			}
		}
	}
	if (length > SYN_CONFIG_SIZE)
		length = SYN_CONFIG_SIZE;

	pr_info("");
	for (i = 0; i < length; i++) {
		pr_info("%2.2X ", ts->config_table[i]);
		if ((i % 16) == 15)
			pr_info("\n");
	}

	for (i = 0; i < length; i++) {
		count += snprintf(buf + count, PAGE_SIZE, "%2.2X ", ts->config_table[i]);
		if ((i % 16) == (16 - 1))
			count += snprintf(buf + count, PAGE_SIZE, "\n");
	}
	count += snprintf(buf + count, PAGE_SIZE, "\n");

	return count;
}

static ssize_t syn_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = gl_ts;
	uint8_t i, j, k = 0, length = 0;
	pr_info("[TP] ts->num_function: %d\n", ts->num_function);
	for (i = 0; i < SYN_MAX_PAGE; i++) {
		for (j = 0; j < ts->num_function; j++) {
			if (((ts->address_table[j].control_base >> 8) & 0xFF) == i) {
				for (k = j; k < ts->num_function; k++)
					if (ts->address_table[k].command_base != 0)
						break;
				length += ts->address_table[k].command_base -
					ts->address_table[j].control_base;
				pr_info("[%d]Length: %x\n", i, length);
				break;
			}
		}
	}

	return count;
}

static DEVICE_ATTR(config, (S_IWUSR|S_IRUGO),
	syn_config_show, syn_config_store);


static ssize_t syn_layout_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = gl_ts;
	uint8_t i;
	size_t count = 0;
	for (i = 0; i < 4; i++)
		count += snprintf(buf + count, PAGE_SIZE, "%d ", ts->layout[i]);
	count += snprintf(buf + count, PAGE_SIZE, "\n");

	return count;
}

static ssize_t syn_layout_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = gl_ts;
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
				pr_info("[TP] buffer size is over 5 char\n");
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
		memcpy(ts->layout, layout, sizeof(layout));
		pr_info("[TP] %d, %d, %d, %d\n",
			ts->layout[0], ts->layout[1], ts->layout[2], ts->layout[3]);
		input_unregister_device(ts->input_dev);
		synaptics_input_register(ts);
	} else
		pr_info("[TP] ERR@%d, %d, %d, %d\n",
			ts->layout[0], ts->layout[1], ts->layout[2], ts->layout[3]);
	return count;

}

static DEVICE_ATTR(layout, (S_IWUSR|S_IRUGO),
	syn_layout_show, syn_layout_store);


static ssize_t syn_pdt_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = gl_ts;
	uint8_t i;
	size_t count = 0;
	for (i = 0; i < ts->num_function; i++) {
		count += snprintf(buf + count, PAGE_SIZE,
			"Funtion: %2X, Query: %3X, Command: %3X, "
			"Control: %3X, Data: %3X, INTR: %2X\n",
			ts->address_table[i].function_type, ts->address_table[i].query_base ,
			ts->address_table[i].command_base, ts->address_table[i].control_base,
			ts->address_table[i].data_base, ts->address_table[i].interrupt_source);
	}
	return count;
}

static DEVICE_ATTR(pdt, S_IRUGO, syn_pdt_show, NULL);

static ssize_t syn_htc_event_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = gl_ts;

	return snprintf(buf, PAGE_SIZE, "%d\n", ts->htc_event);
}

static ssize_t syn_htc_event_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = gl_ts;

	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		ts->htc_event = buf[0] - '0';

	return count;
}

static DEVICE_ATTR(htc_event, (S_IWUSR|S_IRUGO),
	syn_htc_event_show, syn_htc_event_store);

#ifdef SYN_WIRELESS_DEBUG
static ssize_t syn_int_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = gl_ts;
	size_t count = 0;

	count += snprintf(buf + count, PAGE_SIZE, "%d ", ts->irq_enabled);
	count += snprintf(buf + count, PAGE_SIZE, "\n");

	return count;
}

static ssize_t syn_int_status_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = gl_ts;
	int value, ret=0;

	if (sysfs_streq(buf, "0"))
		value = false;
	else if (sysfs_streq(buf, "1"))
		value = true;
	else
		return -EINVAL;

	if (value) {
		ret = request_threaded_irq(ts->client->irq, NULL, synaptics_irq_thread,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, ts->client->name, ts);
		if (ret == 0) {
			ts->irq_enabled = 1;
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x01, CONTROL_BASE) + 1, &ts->intr_bit, 1);
			pr_info("[TP] %s: interrupt enable: %x\n", __func__, ts->intr_bit);
			if (ret)
				free_irq(ts->client->irq, ts);
		}
	} else {
		disable_irq(ts->client->irq);
		free_irq(ts->client->irq, ts);
		ts->irq_enabled = 0;
	}

	return count;
}

static DEVICE_ATTR(enabled, (S_IWUSR|S_IRUGO),
	syn_int_status_show, syn_int_status_store);

static ssize_t syn_reset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = gl_ts;

	if (buf[0] == '1' && ts->gpio_reset) {
		gpio_direction_output(ts->gpio_reset, 0);
		msleep(1);
		gpio_direction_output(ts->gpio_reset, 1);
		pr_info("[TP] %s: synaptics touch chip reseted.\n", __func__);
	}

	return count;
}

static DEVICE_ATTR(reset, (S_IWUSR),
	0, syn_reset);

#endif

static ssize_t syn_cover_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = gl_ts;
	size_t count = 0;
	count += snprintf(buf + count, PAGE_SIZE, "%d\n", ts->cover_enable);

	return count;
}

static ssize_t syn_cover_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{

	struct synaptics_ts_data *ts = gl_ts;

	if (( (ts->package_id == 3528) || (ts->package_id == 3508)) && ts->cover_setting[0]) {
		if (sysfs_streq(buf, "0"))
			ts->cover_enable = 0;
		else if (sysfs_streq(buf, "1"))
			ts->cover_enable = 1;
		else
			return -EINVAL;
		if (ts->syn_cover_wq)
			queue_work(ts->syn_cover_wq, &ts->cover_work);

		pr_info("[TP] %s: cover_enable = %d.\n", __func__, ts->cover_enable);
	}

	return count;
}

static DEVICE_ATTR(cover, (S_IWUSR|S_IRUGO),
	syn_cover_show, syn_cover_store);


enum SR_REG_STATE{
	ALLOCATE_DEV_FAIL = -2,
	REGISTER_DEV_FAIL,
	SUCCESS,
};


static int register_sr_touch_device(void)
{
	struct synaptics_ts_data *ts = gl_ts;

	ts->sr_input_dev = input_allocate_device();

	if (ts->sr_input_dev == NULL) {
		printk(KERN_ERR "[TP][TOUCH_ERR]%s: Failed to allocate SR input device\n", __func__);
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
	input_set_abs_params(ts->sr_input_dev, ABS_MT_TRACKING_ID,
		0, ts->finger_support - 1, 0, 0);
	pr_info("[TP][SR]input_set_abs_params: mix_x %d, max_x %d,"
		" min_y %d, max_y %d\n", ts->layout[0],
		 ts->layout[1], ts->layout[2], ts->layout[3]);

	input_set_abs_params(ts->sr_input_dev, ABS_MT_POSITION_X,
		ts->layout[0], ts->layout[1], 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_POSITION_Y,
		ts->layout[2], ts->layout[3], 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_TOUCH_MAJOR,
		0, 255, 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_PRESSURE,
		0, 30, 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_WIDTH_MAJOR,
		0, 30, 0, 0);

	if (input_register_device(ts->sr_input_dev)) {
		input_free_device(ts->sr_input_dev);
		printk(KERN_ERR "[TP][SR][TOUCH_ERR]%s: Unable to register %s input device\n",
			__func__, ts->sr_input_dev->name);
		return REGISTER_DEV_FAIL;
	}
	return SUCCESS;
}

static ssize_t syn_get_en_sr(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = gl_ts;
	size_t count = 0;

	if (ts->sr_input_dev)
	{
		count += snprintf(buf + count, PAGE_SIZE, "%s ", ts->sr_input_dev->name);
		count += snprintf(buf + count, PAGE_SIZE, "\n");
	}
	else
		count += snprintf(buf + count, PAGE_SIZE, "0\n");


	return count;
}

static ssize_t syn_set_en_sr(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = gl_ts;
	if (buf[0]) {
		if (ts->sr_input_dev)
			pr_info("[TP]%s: SR device already exist!\n", __func__);
		else
			pr_info("[TP]%s: SR touch device enable result:%X\n", __func__, register_sr_touch_device());
	}
	return count;
}

static DEVICE_ATTR(sr_en, (S_IWUSR|S_IRUGO), syn_get_en_sr, syn_set_en_sr);

static struct kobject *android_touch_kobj;

#if defined(CONFIG_SECURE_TOUCH)
static void syn_secure_touch_notify(struct synaptics_ts_data *data)
{
	sysfs_notify(android_touch_kobj, NULL, "secure_touch");
}

static irqreturn_t syn_filter_interrupt(struct synaptics_ts_data *data)
{
	if (atomic_read(&data->st_enabled)) {
		if (atomic_cmpxchg(&data->st_pending_irqs, 0, 1) == 0)
		{
			syn_secure_touch_notify(data);
			wait_for_completion_interruptible(&data->st_irq_processed);
		}
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

static ssize_t syn_secure_touch_enable_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *data = gl_ts;

	return scnprintf(buf, PAGE_SIZE, "%d", atomic_read(&data->st_enabled));
}
static ssize_t syn_secure_touch_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct synaptics_ts_data *data = gl_ts;
	unsigned long value;
	int err = 0;

	if (count > 2)
		return -EINVAL;

	err = kstrtoul(buf, 10, &value);
	if (err != 0)
		return err;

	err = count;

	switch (value) {
	case 0:
		if (atomic_read(&data->st_enabled) == 0)
			break;

		pm_runtime_put(data->client->adapter->dev.parent);
		atomic_set(&data->st_enabled, 0);
		syn_secure_touch_notify(data);
		complete(&data->st_irq_processed);
		synaptics_irq_thread(data->client->irq, data);
		complete(&data->st_powerdown);
		break;
	case 1:
		if (atomic_read(&data->st_enabled)) {
			err = -EBUSY;
			break;
		}

		if (pm_runtime_get(data->client->adapter->dev.parent) < 0) {
			dev_err(&data->client->dev, "pm_runtime_get failed\n");
			err = -EIO;
			break;
		}
		INIT_COMPLETION(data->st_powerdown);
		INIT_COMPLETION(data->st_irq_processed);
		atomic_set(&data->st_enabled, 1);
		synchronize_irq(data->client->irq);
		atomic_set(&data->st_pending_irqs, 0);
		break;
	default:
		dev_err(&data->client->dev, "unsupported value: %lu\n", value);
		err = -EINVAL;
		break;
	}

	return err;
}

static ssize_t syn_secure_touch_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *data = gl_ts;
	int val = 0;

	if (atomic_read(&data->st_enabled) == 0)
		return -EBADF;

	if (atomic_cmpxchg(&data->st_pending_irqs, -1, 0) == -1)
		return -EINVAL;

	if (atomic_cmpxchg(&data->st_pending_irqs, 1, 0) == 1)
		val = 1;
	else
		complete(&data->st_irq_processed);

	return scnprintf(buf, PAGE_SIZE, "%u", val);
}

static DEVICE_ATTR(secure_touch_enable, S_IRUGO | S_IWUSR | S_IWGRP ,
			 syn_secure_touch_enable_show,
			 syn_secure_touch_enable_store);
static DEVICE_ATTR(secure_touch, S_IRUGO, syn_secure_touch_show, NULL);
#endif


static int synaptics_touch_sysfs_init(void)
{
	int ret;
#ifdef SYN_WIRELESS_DEBUG
	struct synaptics_ts_data *ts = gl_ts;
#endif

	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		printk(KERN_ERR "[TP] TOUCH_ERR: %s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}

	if (sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr) ||
#if defined(CONFIG_SECURE_TOUCH)
        sysfs_create_file(android_touch_kobj, &dev_attr_secure_touch_enable.attr) ||
        sysfs_create_file(android_touch_kobj, &dev_attr_secure_touch.attr) ||
#endif
		sysfs_create_file(android_touch_kobj, &dev_attr_register.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_unlock.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_config.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_layout.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_pdt.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_htc_event.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_reset.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_sr_en.attr) ||
		sysfs_create_file(android_touch_kobj, &dev_attr_cover.attr)
#ifdef SYN_WIRELESS_DEBUG
		|| sysfs_create_file(android_touch_kobj, &dev_attr_enabled.attr)
#endif
		)
		return -ENOMEM;
	if (get_address_base(gl_ts, 0x54, FUNCTION))
		if (sysfs_create_file(android_touch_kobj, &dev_attr_diag.attr))
			return -ENOMEM;

#ifdef SYN_WIRELESS_DEBUG
	ret= gpio_request(ts->gpio_irq, "synaptics_attn");
	if (ret) {
		pr_info("[TP]%s: Failed to obtain touchpad IRQ %d. Code: %d.", __func__, ts->gpio_irq, ret);
		return ret;
	}
	if (ts->gpio_reset && !ts->i2c_err_handler_en) {
		ret = gpio_request(ts->gpio_reset, "synaptics_reset");
		if (ret)
			pr_info("[TP]%s: Failed to obtain reset pin: %d. Code: %d.", __func__, ts->gpio_reset, ret);
	}
	ret = gpio_export(ts->gpio_irq, true);
	if (ret) {
		pr_info("[TP]%s: Failed to "
			"export ATTN gpio!\n", __func__);
		ret = 0;
	} else {
		ret = gpio_export_link(&(ts->input_dev->dev), "attn",
			ts->gpio_irq);
		if (ret) {
			pr_info("[TP]%s: Failed to "
				"symlink ATTN gpio!\n", __func__);
			ret = 0;
		} else {
			pr_info("[TP]%s: Exported GPIO %d.", __func__, ts->gpio_irq);
		}
	}
#endif
	return 0;
}

static void synaptics_touch_sysfs_remove(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
#if defined(CONFIG_SECURE_TOUCH)
    sysfs_remove_file(android_touch_kobj, &dev_attr_secure_touch_enable.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_secure_touch.attr);
#endif
	if (get_address_base(gl_ts, 0x54, FUNCTION))
		sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_register.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_unlock.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_config.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_layout.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_pdt.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_htc_event.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_reset.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_sr_en.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_cover.attr);
#ifdef SYN_WIRELESS_DEBUG
	sysfs_remove_file(android_touch_kobj, &dev_attr_enabled.attr);
#endif
	kobject_del(android_touch_kobj);
}

static int synaptics_init_panel(struct synaptics_ts_data *ts)
{
	int ret = 0;
	uint16_t reg = 0;

	
	ret = i2c_syn_write_byte_data(ts->client,
			get_address_base(ts, 0x01, CONTROL_BASE), 0x80);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:1", __func__);
	if (ts->pre_finger_data[0][0] < 2) {
		if (ts->large_obj_check) {
			if (ts->package_id < 3400) {
				reg = get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ((ts->package_id == 2200) ? 0x26: 0x29);
				ret = i2c_syn_write_byte_data(ts->client, reg, ts->default_large_obj & 0x7F);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:2", __func__);
				pr_info("[TP] %s: set large obj suppression register to: %x\n", __func__, ts->default_large_obj & 0x7F);
			}
		}

		if (ts->segmentation_bef_unlock) {
			if (ts->package_id < 3400) {
				reg = get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ((ts->package_id == 2200) ? 0x25: 0x22);
				ret = i2c_syn_write_byte_data(ts->client, reg, ts->segmentation_bef_unlock);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:3", __func__);
				pr_info("[TP] %s: set segmentation aggressiveness to: %x\n", __func__, ts->segmentation_bef_unlock);
			}
		}

		if (ts->threshold_bef_unlock) {
			if (ts->package_id < 3400) {
				reg = get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ((ts->package_id == 2200) ? 0x0A: 0x0C);
				ret = i2c_syn_write_byte_data(ts->client, reg, ts->threshold_bef_unlock);
			} else {
				ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + 0x01, ts->threshold_bef_unlock);
			}
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:4", __func__);
			pr_info("[TP] %s: set Z Touch threshold to: %x\n", __func__, ts->threshold_bef_unlock);
		}

		if (ts->saturation_bef_unlock) {
			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x02, ts->saturation_bef_unlock & 0xFF);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "saturation capacitance", __func__);
			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x03, (ts->saturation_bef_unlock & 0xFF00) >> 8);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "saturation capacitance", __func__);
			pr_info("[TP] %s: set saturation to: %x\n", __func__, ts->saturation_bef_unlock);
			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, COMMAND_BASE), 0x04);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:5", __func__);
		}

		if (ts->PixelTouchThreshold_bef_unlock) {
			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x04, ts->PixelTouchThreshold_bef_unlock);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "F54_ANALOG_CTRL03 Pixel Touch Threshold", __func__);
			pr_info("[TP] %s: set F54_ANALOG_CTRL03 Pixel Touch Threshold: %x\n", __func__, ts->PixelTouchThreshold_bef_unlock);
			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, COMMAND_BASE), 0x04);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:6", __func__);
		}
	}

#ifdef SYN_CALIBRATION_CONTROL
	if (ts->packrat_number < SYNAPTICS_FW_NOCAL_PACKRAT) {
		if (ts->pre_finger_data[0][0] >= 2 || ts->mfg_flag == 1) {
			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x10, 0x0);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:6", __func__);

			ret = i2c_syn_write_byte_data(ts->client,
				get_address_base(ts, 0x54, COMMAND_BASE), 0x04);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:7", __func__);

			pr_info("[TP] %s: Touch init: set fast relaxation to 0x0\n", __func__);
		}
	}
#endif

	return ret;
}

static void synaptics_ts_hover_func(struct synaptics_ts_data *ts, uint8_t *buf, uint8_t *noise_index)
{
	uint8_t finger_state;
	static int x = 0, y = 0, z = 0;
	uint16_t temp_im = 0, temp_cidim = 0;

	temp_im = (noise_index[1] <<8) | noise_index[0];
	temp_cidim = (noise_index[6] <<8) | noise_index[5];

	finger_state = buf[0];
	if (!finger_state) {
		if (ts->debug_log_level & BIT(1))
			pr_info("[TP] Hover Leave\n");
		if ((ts->hover_mode ==1) && (ts->debug_log_level & BIT(3))) {
			if(ts->width_factor && ts->height_factor){
				pr_info("[TP] Screen: Hover:Up, X=%d, Y=%d, Z=%d, IM:%d, CIDIM:%d, Freq:%d, NS:%d\n",
					(x*ts->width_factor)>>SHIFT_BITS, (y*ts->height_factor)>>SHIFT_BITS, z,
					temp_im, temp_cidim, noise_index[9], noise_index[4]);
			} else {
				pr_info("[TP] Raw: Hover:Up, X=%d, Y=%d, Z=%d, IM:%d, CIDIM:%d, Freq:%d, NS:%d\n",
					x, y, z, temp_im, temp_cidim, noise_index[9], noise_index[4]);
			}
		}
		ts->hover_mode = 0;
		if (ts->support_htc_event) {
			input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
			input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
		}
		if (ts->htc_event == SYN_AND_REPORT_TYPE_A) {
			input_report_key(ts->input_dev, BTN_TOUCH, 0);
			input_mt_sync(ts->input_dev);
		}
		else if (ts->htc_event == SYN_AND_REPORT_TYPE_B) {
			input_mt_slot(ts->input_dev, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
			input_report_key(ts->input_dev, BTN_TOUCH, 0);
		}
	} else {
		x = (buf[2] << 8) + buf[1];
		y = (buf[4] << 8) + buf[3];
		z = 255 - buf[5];
		if (ts->debug_log_level & BIT(1))
			pr_info("[TP] Hover=> X:%d, Y:%d, Z:%d\n", x, y, z);
		if (ts->debug_log_level & BIT(17))
			pr_info("[TP] Hover=> X:%d, Y:%d, Z:%d, IM:%d, CIDIM:%d, Freq:%d, NS:%d\n",
				x, y, z, temp_im, temp_cidim, noise_index[9], noise_index[4]);
		if ((ts->hover_mode ==0) && (ts->debug_log_level & BIT(3))) {
			if(ts->width_factor && ts->height_factor){
				pr_info("[TP] Screen: Hover:Down, X=%d, Y=%d, Z=%d, IM:%d, CIDIM:%d, Freq:%d, NS:%d\n",
					(x*ts->width_factor)>>SHIFT_BITS, (y*ts->height_factor)>>SHIFT_BITS, z,
					temp_im, temp_cidim, noise_index[9], noise_index[4]);
			} else {
				pr_info("[TP] Raw: Hover:Down, X=%d, Y=%d, Z=%d, IM:%d, CIDIM:%d, Freq:%d, NS:%d\n",
					x, y, z, temp_im, temp_cidim, noise_index[9], noise_index[4]);
			}
		}
		ts->hover_mode = 1;
		if (ts->support_htc_event) {
			input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0 << 16 | 0);
			input_report_abs(ts->input_dev, ABS_MT_POSITION, x << 16 | y);
		}
		switch (ts->htc_event) {
		case SYN_AND_REPORT_TYPE_A:
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
			break;
		case SYN_AND_REPORT_TYPE_B:
			input_mt_slot(ts->input_dev, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
			break;
		}
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_report_abs(ts->input_dev, ABS_MT_DISTANCE, z);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
		switch (ts->htc_event) {
		case SYN_AND_REPORT_TYPE_A:
			input_mt_sync(ts->input_dev);
			break;
		case SYN_AND_REPORT_TYPE_B:
			break;
		}
	}
	input_sync(ts->input_dev);
}

static void synaptics_ts_finger_func(struct synaptics_ts_data *ts)
{
	int ret;
	uint8_t buf[ts->finger_support * 8 ], noise_index[10];
	uint16_t temp_im = 0, temp_cidim = 0;
	static int x_pos[10] = {0}, y_pos[10] = {0};

	memset(buf, 0x0, sizeof(buf));
	memset(noise_index, 0x0, sizeof(noise_index));
	if (ts->package_id < 3400)
		ret = i2c_syn_read(ts->client,
			get_address_base(ts, 0x01, DATA_BASE) + 2, buf, sizeof(buf));
	else {
		ret = i2c_syn_read(ts->client,
			get_address_base(ts, ts->finger_func_idx, DATA_BASE), buf, sizeof(buf));
		ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x54, DATA_BASE) + 4, noise_index, sizeof(noise_index));
		temp_im = (noise_index[1] <<8) | noise_index[0];
		temp_cidim = (noise_index[6] <<8) | noise_index[5];
	}

	if((noise_index[9] != ts->prev_Freq) || (noise_index[4] != ts->prev_NS))
	{
		pr_info("[TP][NS]Finger Pressed:%d, IM:%d, CIDIM:%d, Freq:%d, NS:%d\n",
		ts->finger_pressed, temp_im, temp_cidim, noise_index[9], noise_index[4]);
	}
	ts->prev_NS = noise_index[4];
	ts->prev_Freq = noise_index[9];

	if (ret < 0) {
		i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:1", __func__);
	} else {
		int finger_data[ts->finger_support][4];
		int base = 0;
		uint8_t i, j;
		uint16_t finger_press_changed = 0, finger_release_changed = 0, finger_pressed = 0;

		if (ts->package_id < 3400)
			base = (ts->finger_support + 3) / 4;
		ts->finger_count = 0;
		if (ts->debug_log_level & BIT(0)) {
			pr_info("[TP] Touch:");
			for (i = 0; i < sizeof(buf); i++)
				pr_info(" %2x", buf[i]);
			pr_info("\n");
		}
		if(ts->package_id >= 3528)
		{
			if ((buf[0]==0x05) || ((buf[0]==0x0) && ts->hover_mode==1)) {
				synaptics_ts_hover_func(ts, buf, noise_index);
				ts->finger_pressed = 0;
				return;
			}
			ts->hover_mode = 0;
		}

		for (i = 0; i < ts->finger_support; i++) {
			uint8_t finger_state;
			finger_state = ((ts->package_id < 3400) ? (buf[(i / 4)] >> ((i * 2) % 8)): buf[i * 8]);

			if (finger_state & 0x03) {
				finger_pressed |= BIT(i);
				ts->finger_count++;
				if (finger_state == 0x02)
					pr_info("[TP] Finger state[%d] = 0x02\n", i);
				else if (finger_state == 0x03)
					pr_info("[TP] Finger state[%d] = 0x03\n", i);
			}
#ifdef SYN_FILTER_CONTROL
			else if ((ts->grip_suppression | ts->grip_b_suppression) & BIT(i)) {
				ts->grip_suppression &= ~BIT(i);
				ts->grip_b_suppression &= ~BIT(i);
			}
#endif
		}
		if (ts->finger_pressed != finger_pressed
			) {
			finger_press_changed = ts->finger_pressed ^ finger_pressed;
			finger_release_changed = finger_press_changed & ts->finger_pressed;
			finger_press_changed &= finger_pressed;
			ts->finger_pressed = finger_pressed;
		}

		if (ts->debug_log_level & BIT(3)) {
			for (i = 0; i < ts->finger_support; i++) {
				if (finger_release_changed & BIT(i) ) {
					if (ts->package_id < 3400) {
						uint32_t flip_flag = SYNAPTICS_FLIP_X;
						uint8_t pos_mask = 0x0f;
						for (j = 0; j < 2; j++) {
							finger_data[i][j]
								= (buf[base+2] & pos_mask) >> (j * 4) |
								(uint16_t)buf[base + j] << 4;
							if (ts->flags & flip_flag)
								finger_data[i][j] = ts->max[j] - finger_data[i][j];
							flip_flag <<= 1;
							pos_mask <<= 4;
						}
						finger_data[i][2] = (buf[base+3] >> 4 & 0x0F) + (buf[base+3] & 0x0F);
						finger_data[i][3] = buf[base+4];
					} else {
						finger_data[i][0] = (buf[base+2] << 8) + buf[base+1];
						finger_data[i][1] = (buf[base+4] << 8) + buf[base+3];
						finger_data[i][2] = buf[base+6] + buf[base+7];
						finger_data[i][3] = buf[base+5];
					}

					if (ts->flags & SYNAPTICS_SWAP_XY)
						swap(finger_data[i][0], finger_data[i][1]);

					if (ts->layout[1] < finger_data[i][0])
						finger_data[i][0] = ts->layout[1];
					if (ts->width_factor && ts->height_factor) {
						pr_info("[TP] Screen:F[%02d]:Up, X=%d, Y=%d, W=%d, Z=%d, IM:%d, CIDIM:%d, Freq:%d, NS:%d\n",
							i+1, (x_pos[i]*ts->width_factor)>>SHIFT_BITS,
							(y_pos[i]*ts->height_factor)>>SHIFT_BITS,
							finger_data[i][2], finger_data[i][3],
							temp_im, temp_cidim, noise_index[9], noise_index[4]);
					} else {
						pr_info("[TP] Raw:F[%02d]:Up, X=%d, Y=%d, W=%d, Z=%d, IM:%d, CIDIM:%d, Freq:%d, NS:%d\n",
							i+1, x_pos[i], y_pos[i],
							finger_data[i][2], finger_data[i][3],
							temp_im, temp_cidim, noise_index[9], noise_index[4]);
					}
				}
				base += 5;
			}
		}

		if (ts->htc_event == SYN_AND_REPORT_TYPE_B && finger_release_changed) {
			for (i = 0; i < ts->finger_support; i++) {
				if (finger_release_changed & BIT(i)) {
					input_mt_slot(ts->input_dev, i);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
					input_report_key(ts->input_dev, BTN_TOUCH, 0);
					ts->tap_suppression &= ~BIT(i);
				}
			}
		}

		if (finger_pressed == 0 
) {
			if (ts->htc_event == SYN_AND_REPORT_TYPE_A) {
				
				if (ts->support_htc_event) {
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
					input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
				}
				input_report_key(ts->input_dev, BTN_TOUCH, 0);
				input_mt_sync(ts->input_dev);
			} else if (ts->htc_event == SYN_AND_REPORT_TYPE_B) {
				if (ts->support_htc_event) {
					input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
					input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
				}
			}
			else if (ts->htc_event == SYN_AND_REPORT_TYPE_HTC) {
				input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
				input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
			}

#ifdef SYN_FILTER_CONTROL
			if (ts->filter_level[0])
				ts->ambiguous_state = 0;
			ts->grip_b_suppression = 0;
#endif

#ifdef CONFIG_MOTION_FILTER
			if (ts->reduce_report_level[0])
				ts->tap_suppression = 0;
#endif
			if (ts->debug_log_level & BIT(1))
				pr_info("[TP] Finger leave\n");
		}

		if (ts->pre_finger_data[0][0] < 2 || finger_pressed) {
			base = ((ts->package_id < 3400) ? ((ts->finger_support + 3) / 4): 0);

			for (i = 0; i < ts->finger_support; i++) {
				uint32_t flip_flag = SYNAPTICS_FLIP_X;
				if ((finger_pressed | finger_release_changed) & BIT(i)) {
					if (ts->package_id < 3400) {
						uint8_t pos_mask = 0x0f;
						for (j = 0; j < 2; j++) {
							finger_data[i][j]
								= (buf[base+2] & pos_mask) >> (j * 4) |
								(uint16_t)buf[base + j] << 4;
							if (ts->flags & flip_flag)
								finger_data[i][j] = ts->max[j] - finger_data[i][j];
							flip_flag <<= 1;
							pos_mask <<= 4;
						}
						finger_data[i][2] = (buf[base+3] >> 4 & 0x0F) + (buf[base+3] & 0x0F);
						finger_data[i][3] = buf[base+4];
					} else {
						finger_data[i][0] = (buf[base+2] << 8) + buf[base+1];
						finger_data[i][1] = (buf[base+4] << 8) + buf[base+3];
						finger_data[i][2] = buf[base+6] + buf[base+7];
						finger_data[i][3] = buf[base+5];
					}

					if (ts->flags & SYNAPTICS_SWAP_XY)
						swap(finger_data[i][0], finger_data[i][1]);

					if (ts->layout[1] < finger_data[i][0])
						finger_data[i][0] = ts->layout[1];

					if ((finger_release_changed & BIT(i)) && ts->pre_finger_data[0][0] < 2) {
						if (!ts->first_pressed) {
							if (ts->finger_count == 0)
								ts->first_pressed = 1;
							pr_info("[TP] E%d@%d, %d\n", i + 1,
							x_pos[i], y_pos[i]);
						}
					}
#ifdef SYN_FILTER_CONTROL
					if (abs((buf[base+3] >> 4 & 0x0F) - (buf[base+3] & 0x0F)) >= 10)
						ts->grip_b_suppression |= BIT(i);

					if (ts->filter_level[0] &&
						((finger_press_changed | ts->grip_suppression) & BIT(i))) {
						if ((finger_data[i][0] < (ts->filter_level[0] + ts->ambiguous_state * 20) ||
							finger_data[i][0] > (ts->filter_level[3] - ts->ambiguous_state * 20)) &&
							!(ts->grip_suppression & BIT(i))) {
							ts->grip_suppression |= BIT(i);
						} else if ((finger_data[i][0] < (ts->filter_level[1] + ts->ambiguous_state * 20) ||
							finger_data[i][0] > (ts->filter_level[2] - ts->ambiguous_state * 20)) &&
							(ts->grip_suppression & BIT(i)))
							ts->grip_suppression |= BIT(i);
						else if (finger_data[i][0] > (ts->filter_level[1] + ts->ambiguous_state * 20) &&
							finger_data[i][0] < (ts->filter_level[2] - ts->ambiguous_state * 20)) {
							ts->grip_suppression &= ~BIT(i);
						}
					}
					if ((ts->grip_suppression | ts->grip_b_suppression) & BIT(i)) {
						finger_pressed &= ~BIT(i);
					} else
#endif

#ifdef CONFIG_MOTION_FILTER
					if (ts->htc_event == SYN_AND_REPORT_TYPE_B && ts->reduce_report_level[0]) {
						if (ts->tap_suppression & BIT(i) && finger_pressed & BIT(i)) {
							int dx, dy = 0;
							dx = abs(ts->pre_finger_data[i + 1][2] - finger_data[i][0]);
							dy = abs(ts->pre_finger_data[i + 1][3] - finger_data[i][1]);
							if (dx > ts->reduce_report_level[TAP_DX_OUTER] || dy > ts->reduce_report_level[TAP_DY_OUTER]) {
								ts->tap_suppression &= ~BIT(i);
							} else if (ts->reduce_report_level[TAP_TIMEOUT] && time_after(jiffies, ts->tap_timeout[i]) && (dx > ts->reduce_report_level[TAP_DX_INTER] || dy > ts->reduce_report_level[TAP_DY_INTER])) {
								ts->tap_suppression &= ~BIT(i);
							} else {
								finger_pressed &= ~BIT(i);
								if (ts->debug_log_level & BIT(1))
									pr_info("[TP] Filtered Finger %d=> X:%d, Y:%d w:%d, z:%d\n",
										i + 1, finger_data[i][0], finger_data[i][1],
										finger_data[i][2], finger_data[i][3]);
							}
						}
					}
#endif

					if ((finger_pressed & BIT(i)) == BIT(i)) {
					if (ts->hall_block_touch_event == 0) {
						if (ts->htc_event == SYN_AND_REPORT_TYPE_A) {
							if (ts->support_htc_event) {
								input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
									finger_data[i][3] << 16 | finger_data[i][2]);
								input_report_abs(ts->input_dev, ABS_MT_POSITION,
									(finger_pressed == 0) << 31 |
									finger_data[i][0] << 16 | finger_data[i][1]);
							}

							input_report_key(ts->input_dev, BTN_TOUCH, 1);
							input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
							input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
								finger_data[i][3]);
							input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
								finger_data[i][2]);
							input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
								finger_data[i][2]);
							input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
								finger_data[i][0]);
							input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
								finger_data[i][1]);
							input_mt_sync(ts->input_dev);
						} else if (ts->htc_event == SYN_AND_REPORT_TYPE_B) {
							if (ts->support_htc_event) {
								input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
									finger_data[i][3] << 16 | finger_data[i][2]);
								input_report_abs(ts->input_dev, ABS_MT_POSITION,
									(finger_pressed == 0) << 31 |
									finger_data[i][0] << 16 | finger_data[i][1]);
							}
							input_mt_slot(ts->input_dev, i);
							input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,
							1);
							input_report_key(ts->input_dev, BTN_TOUCH, 1);
							input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
								finger_data[i][3]);
							input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
								finger_data[i][2]);
							input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
								finger_data[i][2]);
							input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
								finger_data[i][0]);
							input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
								finger_data[i][1]);
						} else if (ts->htc_event == SYN_AND_REPORT_TYPE_HTC) {
							input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
							input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE,
								finger_data[i][3] << 16 | finger_data[i][2]);
							input_report_abs(ts->input_dev, ABS_MT_POSITION,
								(finger_pressed == 0) << 31 |
								finger_data[i][0] << 16 | finger_data[i][1]);
							}
						}
						x_pos[i] = finger_data[i][0];
						y_pos[i] = finger_data[i][1];
						finger_pressed &= ~BIT(i);

						if ((finger_press_changed & BIT(i)) && ts->debug_log_level & BIT(3)) {
							if(ts->width_factor && ts->height_factor){
								pr_info("[TP] Screen:F[%02d]:Down, X=%d, Y=%d, W=%d, Z=%d, IM:%d, CIDIM:%d, Freq:%d, NS:%d\n",
									i+1, (finger_data[i][0]*ts->width_factor)>>SHIFT_BITS,
									(finger_data[i][1]*ts->height_factor)>>SHIFT_BITS,
									finger_data[i][2], finger_data[i][3],
									temp_im, temp_cidim, noise_index[9], noise_index[4]);
							} else {
								pr_info("[TP] Raw:F[%02d]:Down, X=%d, Y=%d, W=%d, Z=%d, IM:%d, CIDIM:%d, Freq:%d, NS:%d\n",
									i+1, finger_data[i][0], finger_data[i][1],
									finger_data[i][2], finger_data[i][3],
									temp_im, temp_cidim, noise_index[9], noise_index[4]);
							}
						}

						if (ts->pre_finger_data[0][0] < 2) {
							if (finger_press_changed & BIT(i)) {
								ts->pre_finger_data[i + 1][0] = finger_data[i][0];
								ts->pre_finger_data[i + 1][1] = finger_data[i][1];

								if (!ts->first_pressed)
									pr_info("[TP] S%d@%d, %d\n", i + 1,
										finger_data[i][0], finger_data[i][1]);
								if (ts->packrat_number < SYNAPTICS_FW_NOCAL_PACKRAT) {
#ifdef SYN_CALIBRATION_CONTROL
									if (ts->multitouch_calibration) {
										if (ts->finger_count == ts->finger_support) {
											if (ts->package_id < 3400) {
												ret = i2c_syn_write_byte_data(ts->client,
													get_address_base(ts, ts->finger_func_idx, COMMAND_BASE), 0x01);
												if (ret < 0)
													i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:Rezero_1", __func__);
												pr_info("[TP] %s: Touch Calibration Confirmed, rezero\n", __func__);
											}
										} else if (!ts->pre_finger_data[0][0] && ts->finger_count > 1)
											ts->pre_finger_data[0][0] = 1;
									}
#endif
								}
							}
						}

#ifdef CONFIG_MOTION_FILTER
						if (ts->htc_event == SYN_AND_REPORT_TYPE_B && ts->reduce_report_level[TAP_DX_OUTER]) {
							if (finger_press_changed & BIT(i)) {
								ts->tap_suppression &= ~BIT(i);
								ts->tap_suppression |= BIT(i);
								ts->pre_finger_data[i + 1][2] = finger_data[i][0];
								ts->pre_finger_data[i + 1][3] = finger_data[i][1];
								if (ts->reduce_report_level[TAP_TIMEOUT] && (ts->tap_suppression))
									ts->tap_timeout[i] = jiffies + msecs_to_jiffies(ts->reduce_report_level[TAP_TIMEOUT]);
							}
						}
#endif

						if (ts->debug_log_level & BIT(1))
							pr_info("[TP] Finger %d=> X:%d, Y:%d W:%d, Z:%d\n",
								i + 1, finger_data[i][0], finger_data[i][1],
								finger_data[i][2], finger_data[i][3]);
						if (ts->debug_log_level & BIT(17))
							pr_info("[TP] Finger %d=> X:%d, Y:%d W:%d, Z:%d, IM:%d, CIDIM:%d, Freq:%d, NS:%d\n",
								i + 1, finger_data[i][0], finger_data[i][1], finger_data[i][2],
								finger_data[i][3], noise_index[1] | noise_index[0],
								noise_index[6] | noise_index[5], noise_index[9], noise_index[4]);
					}
					if (ts->packrat_number < SYNAPTICS_FW_NOCAL_PACKRAT) {
#ifdef SYN_CALIBRATION_CONTROL
						if (ts->multitouch_calibration) {
							if ((finger_release_changed & BIT(i)) && ts->pre_finger_data[0][0] == 1) {
								if (ts->package_id < 3400) {
									ret = i2c_syn_write_byte_data(ts->client,
										get_address_base(ts, ts->finger_func_idx, COMMAND_BASE), 0x01);
									if (ret < 0)
										i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:Rezero_2", __func__);
									pr_info("[TP] %s: Touch Calibration Confirmed, rezero\n", __func__);
								}
							}
						}
#endif
					}
					if (!ts->finger_count)
						ts->pre_finger_data[0][0] = 0;
				}
				base += ((ts->package_id < 3400) ? 5: 8);
			}
#ifdef SYN_FILTER_CONTROL
			if (ts->filter_level[0] && ts->grip_suppression) {
				ts->ambiguous_state = 0;
				for (i = 0; i < ts->finger_support; i++)
					if (ts->grip_suppression & BIT(i))
						ts->ambiguous_state++;
			}
			if (ts->debug_log_level & BIT(16))
				pr_info("[TP] ts->grip_suppression: %x, ts->ambiguous_state: %x\n",
					ts->grip_suppression, ts->ambiguous_state);
#endif
		}
	}

	input_sync(ts->input_dev);

}

static void synaptics_ts_report_func(struct synaptics_ts_data *ts)
{
	int ret, size;
	uint8_t data[2] = {0};

	ret = i2c_syn_write(ts->client,
		get_address_base(ts, 0x54, DATA_BASE) + 1, &data[0], 2);

	if (ret < 0)
		i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:1", __func__);
	else {
		if (ts->diag_command == 38 || ts->diag_command == 40) {
			size = (ts->x_channel + ts->y_channel) * 4;
		} else {
			size = ts->x_channel * ts->y_channel * 2;
		}
		ret = i2c_syn_read(ts->client,
			get_address_base(ts, 0x54, DATA_BASE) + 3, ts->temp_report_data, size);
		if (ret >= 0)
			if (ts->diag_command == 38 || ts->diag_command == 40)
				memcpy(&ts->report_data_32[0], &ts->temp_report_data[0], size);
			else
				memcpy(&ts->report_data[0], &ts->temp_report_data[0], size);
		else {
			memset(&ts->report_data[0], 0x0, sizeof(ts->report_data));
			i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:2", __func__);
		}
	}
	atomic_set(&ts->data_ready, 1);
	wake_up(&syn_data_ready_wq);

}

static void synaptics_ts_button_func(struct synaptics_ts_data *ts)
{
	int ret;
	uint8_t data = 0, idx = 0;
	uint16_t x_position = 0, y_position = 0;

	ret = i2c_syn_read(ts->client,
		get_address_base(ts, 0x1A, DATA_BASE), &data, 1);
	if (data) {
		vk_press = 1;
		if (ts->button) {
			idx = 	(data == 0x01 ? 0:
				 data == 0x02 ? 1:
				 data == 0x04 ? 2:
				 100);

			if (idx == 100) {
				vk_press = 0;
				pr_err("[TP] vk_data:%#x, idx=%d", data, idx);
				return;
			}
			x_position = (ts->button[idx].x_range_min + ts->button[idx].x_range_max) / 2;
			y_position = (ts->button[idx].y_range_min + ts->button[idx].y_range_max) / 2;
		}
		data == 0x01 ? pr_info("[TP] back key pressed, vk=%x\n", data) :
		data == 0x02 ? pr_info("[TP] home key pressed, vk=%x\n", data) :
		data == 0x04 ? pr_info("[TP] app key pressed , vk=%x\n", data) :
		pr_info("[TP] vk=%#x\n", data);

		if (ts->support_htc_event) {
			input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 100 << 16 | 100);
			input_report_abs(ts->input_dev, ABS_MT_POSITION, x_position << 16 | y_position);
		}
		switch (ts->htc_event) {
		case SYN_AND_REPORT_TYPE_A:
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
			break;
		case SYN_AND_REPORT_TYPE_B:
			input_mt_slot(ts->input_dev, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
			break;
		}
		input_report_key(ts->input_dev, BTN_TOUCH, 1);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 100);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 100);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 100);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x_position);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y_position);
		switch (ts->htc_event) {
		case SYN_AND_REPORT_TYPE_A:
			input_mt_sync(ts->input_dev);
			break;
		case SYN_AND_REPORT_TYPE_B:
			break;
		}
	} else {
		printk("[TP] virtual key released\n");
		vk_press = 0;
		if (ts->htc_event == SYN_AND_REPORT_TYPE_A) {
			if (ts->support_htc_event) {
				input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
				input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
			}
			input_report_key(ts->input_dev, BTN_TOUCH, 0);
			input_mt_sync(ts->input_dev);
		} else if (ts->htc_event == SYN_AND_REPORT_TYPE_B) {
			if (ts->support_htc_event) {
				input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
				input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
			}
			input_mt_slot(ts->input_dev, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
			input_report_key(ts->input_dev, BTN_TOUCH, 0);
		}
	}
	input_sync(ts->input_dev);
}

static void synaptics_ts_status_func(struct synaptics_ts_data *ts)
{
	int ret;
	uint8_t data = 0;

	ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, DATA_BASE), &data, 1);
	if (ret < 0) {
		i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r", __func__);
	} else {
		data &= 0x0F;
		pr_info("[TP] Device Status = %x\n", data);
		if (data == 1) {
			mutex_lock(&syn_mutex);
			ts->page_select = 0;
			mutex_unlock(&syn_mutex);
			pr_info("[TP] TOUCH: Page Select: %s: %d\n", __func__, ts->page_select);
			ret = synaptics_init_panel(ts);
			if (ret < 0)
				pr_info("[TP]%s: synaptics_init_panel fail\n", __func__);
			
			if (((ts->package_id == 3528 ) || (ts->package_id == 3508)) && ts->cover_setting[0]) {
				if (!ts->i2c_to_mcu && ts->cover_enable) {
					if (ts->syn_cover_wq)
						queue_work(ts->syn_cover_wq, &ts->cover_work);
					pr_info("[TP] %s: cover_enable = %d.\n", __func__, ts->cover_enable);
				}
			}

		}
	}

}

static void synaptics_ts_work_func(struct work_struct *work)
{
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);
	int ret;
	uint8_t buf = 0;
	ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, DATA_BASE) + 1, &buf, 1);

	if (ret < 0) {
		i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r", __func__);
	} else {
		if (buf & get_address_base(ts, ts->finger_func_idx, INTR_SOURCE))
			synaptics_ts_finger_func(ts);
		if (buf & get_address_base(ts, 0x01, INTR_SOURCE))
			synaptics_ts_status_func(ts);
		if (buf & get_address_base(ts, 0x54, INTR_SOURCE))
			synaptics_ts_report_func(ts);
	}

}

static irqreturn_t synaptics_irq_thread(int irq, void *ptr)
{
	struct synaptics_ts_data *ts = ptr;
	int ret;
	uint8_t buf = 0;
	struct timespec timeStart, timeEnd, timeDelta;

#if defined(CONFIG_SECURE_TOUCH)
    if(IRQ_HANDLED == syn_filter_interrupt(ts))
        return IRQ_HANDLED;
#endif

	if (ts->debug_log_level & BIT(2)) {
			getnstimeofday(&timeStart);
	}

	ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, DATA_BASE) + 1, &buf, 1);
	if (ret < 0) {
		i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r", __func__);
	} else {
		if (buf & get_address_base(ts, ts->finger_func_idx, INTR_SOURCE)) {
			if (!vk_press) {
				synaptics_ts_finger_func(ts);
				if (ts->debug_log_level & BIT(2)) {
					getnstimeofday(&timeEnd);
					timeDelta.tv_nsec = (timeEnd.tv_sec*1000000000+timeEnd.tv_nsec)
						-(timeStart.tv_sec*1000000000+timeStart.tv_nsec);
					pr_info("[TP] Touch latency = %ld us\n", timeDelta.tv_nsec/1000);
				}
			}
		}
		if (buf & get_address_base(ts, 0x1A, INTR_SOURCE)) {
			if (!ts->finger_count)
				synaptics_ts_button_func(ts);
			else
				printk("[TP] Ignore VK interrupt due to 2d points did not leave\n");
		}
		if (buf & get_address_base(ts, 0x01, INTR_SOURCE))
			synaptics_ts_status_func(ts);
		if (buf & get_address_base(ts, 0x54, INTR_SOURCE))
			synaptics_ts_report_func(ts);
	}

	return IRQ_HANDLED;
}

static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);
	queue_work(ts->syn_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

#ifdef SYN_CABLE_CONTROL
static void cable_tp_status_handler_func(int connect_status)
{
	struct synaptics_ts_data *ts = gl_ts;
	uint8_t data;
	int ret;

	pr_info("[TP] Touch: cable change to %d\n", connect_status);

	if (connect_status)
		connect_status = 1;
	ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, CONTROL_BASE), &data, 1);
	if (ret < 0) {
		i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:1", __func__);
	} else {
		ts->cable_config = (data & 0xDF) | (connect_status << 5);
		pr_info("[TP] %s: ts->cable_config: %x\n", __func__, ts->cable_config);
		ret = i2c_syn_write_byte_data(ts->client,
			get_address_base(ts, 0x01, CONTROL_BASE), ts->cable_config);
		if (ret < 0) {
			i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:2", __func__);
		}
	}
}
static struct t_usb_status_notifier cable_status_handler = {
	.name = "usb_tp_connected",
	.func = cable_tp_status_handler_func,
};
#endif

static void synaptics_ts_close_psensor_func(struct work_struct *work)
{
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, psensor_work);
	if(ts->psensor_resume_enable == 1) {
		pr_info("[TP] %s: Disable P-sensor by Touch\n", __func__);
		ts->psensor_resume_enable = 0;
	}
}

static int psensor_tp_status_handler_func(struct notifier_block *this,
	unsigned long status, void *unused)
{
	struct synaptics_ts_data *ts = gl_ts;
	int ret;

	pr_info("[TP] psensor status %d -> %lu\n",
		ts->psensor_status, status);

	if (ts->packrat_number < SYNAPTICS_FW_NOCAL_PACKRAT) {
		if (ts->psensor_detection) {
			if (status == 3 && ts->psensor_resume_enable >= 1) {
				if (!(ts->psensor_status == 1 && ts->psensor_resume_enable == 1)) {
					ret = i2c_syn_write_byte_data(ts->client, get_address_base(ts, ts->finger_func_idx, COMMAND_BASE), 0x01);
					if (ret < 0)
						i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "w:Rezero_1", __func__);
					pr_info("[TP] %s: Touch Calibration Confirmed, rezero\n", __func__);
				}

				if (ts->psensor_resume_enable == 1)
					queue_work(ts->syn_psensor_wq, &ts->psensor_work);
				else
					ts->psensor_resume_enable = 0;
			}
		}

		if (ts->psensor_status == 0) {
			if (status == 1)
				ts->psensor_status = status;
			else
				ts->psensor_status = 0;
		} else
			ts->psensor_status = status;

		if (ts->psensor_detection) {
			if (ts->psensor_status == 0) {
				ts->psensor_resume_enable = 0;
				ts->psensor_phone_enable = 0;
			}
		}
	}

	return NOTIFY_OK;
}

static struct notifier_block psensor_status_handler = {
	.notifier_call = psensor_tp_status_handler_func,
};

static int hallsensor_hover_status_handler_func(struct notifier_block *this,
	unsigned long status, void *unused)
{
	int pole = 0, pole_value = 0;
	struct synaptics_ts_data *ts = gl_ts;

	pole_value = 0x1 & status;
	pole = (0x2 & status) >> 1;
	pr_info("[TP][HL] %s[%s]", pole? "att_s" : "att_n", pole_value ? "Near" : "Far");

	if (pole == 1 && (ts->package_id == 3528 || ts->package_id == 3508 ) && ts->cover_setting[0]) {
		if (pole_value == 0)
			ts->cover_enable = 0;
		else
		{
			ts->cover_enable = 1;
			if (ts->hall_block_touch_time > 0)
				syn_handle_block_touch(ts, 1);
		}

		if (!ts->i2c_to_mcu) {
			if (ts->syn_cover_wq)
			{
				queue_work(ts->syn_cover_wq, &ts->cover_work);
			}
		}
		pr_info("[TP][HL] %s: cover_enable = %d.\n", __func__, ts->cover_enable);
	}

	return NOTIFY_OK;
}

static struct notifier_block hallsensor_status_handler = {
	.notifier_call = hallsensor_hover_status_handler_func,
};

static int syn_pdt_scan(struct synaptics_ts_data *ts, int num_page)
{
	uint8_t intr_count = 0, data[6] = {0}, num_function[SYN_MAX_PAGE] = {0};
	uint16_t i, j, k = 0;
	int ret = 0;
	ts->num_function = 0;

	for (i = 0; i < num_page; i++) {
		for (j = (0xEE | (i << 8)); j >= (0xBE | (i << 8)); j -= 6) {
			ret = i2c_syn_read(ts->client, j, data, 1);
			if (ret < 0)
			{
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r1", __func__);
			}
			if (data[0] == 0)
				break;
			else
				num_function[i]++;
		}
		ts->num_function += num_function[i];
	}

	if (ts->address_table != NULL) {
		kfree(ts->address_table);
		ts->address_table = NULL;
	}

	if (ts->address_table == NULL) {
		ts->address_table = kzalloc(sizeof(struct function_t) * ts->num_function, GFP_KERNEL);
		if (ts->address_table == NULL) {
			pr_info("[TP] syn_pdt_scan: memory allocate fail\n");
			return -ENOMEM;
		}
		pr_info("[TP] syn_pdt_scan: memory allocate success. ptr: %p\n", ts->address_table);
	}

	pr_info("[TP] synaptics: %d function supported\n", ts->num_function);
	for (i = 0; i < num_page; i++) {
		for (j = 0; j < num_function[i]; j++) {
			ret = i2c_syn_read(ts->client, i << 8 | (0xE9 - 6*j), data, 6);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:2", __func__);
			ts->address_table[j + k].query_base = i << 8 | data[0];
			ts->address_table[j + k].command_base = i << 8 | data[1];
			ts->address_table[j + k].control_base = i << 8 | data[2];
			ts->address_table[j + k].data_base = i << 8 | data[3];
			if (data[4] & 0x07) {
				ts->address_table[j + k].interrupt_source =
					get_int_mask(data[4] & 0x07, intr_count);
				intr_count += (data[4] & 0x07);
			}
			ts->address_table[j + k].function_type = data[5];
			pr_info("Query: %2.2X, Command: %4.4X, Control: %2X, Data: %2X, INTR: %2X, Funtion: %2X\n",
				ts->address_table[j + k].query_base , ts->address_table[j + k].command_base,
				ts->address_table[j + k].control_base, ts->address_table[j + k].data_base,
				ts->address_table[j + k].interrupt_source, ts->address_table[j + k].function_type);
		}
		k += num_function[i];
	}
	return ts->num_function;
}
static int syn_get_version(struct synaptics_ts_data *ts)
{
	uint8_t data[16] = {0};
	int ret = 0;
	ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, QUERY_BASE) + 17, data, 4);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:1", __func__);
	ts->package_id = data[1] << 8 | data[0];

	ts->finger_func_idx = ((ts->package_id >= 3400) ? 0x12: 0x11);

	ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, QUERY_BASE) + 18, data, 3);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:3", __func__);
	ts->packrat_number = data[2] << 16 | data[1] << 8 | data[0];

	if (ts->packrat_number < SYNAPTICS_FW_3_2_PACKRAT) {
		ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, QUERY_BASE) + 16, data, 3);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:2", __func__);
		syn_panel_version = data[0] << 8 | data[2];
	} else {
		ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, QUERY_BASE) + 28, data, 16);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:2", __func__);
		syn_panel_version = data[5] << 8 | data[7];
	}

	ret = i2c_syn_read(ts->client, get_address_base(ts, 0x34, CONTROL_BASE), data, 4);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:4", __func__);
	ts->config_version = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];

	pr_info("[TP] %s:pk_id=%d, pk_num=%d, pl_ver=%x, cfg_ver=%x", __func__, ts->package_id,
		ts->packrat_number, syn_panel_version, ts->config_version);

	return 0;
}

static int syn_get_information(struct synaptics_ts_data *ts)
{
	uint8_t data[8] = {0}, i, num_channel, *buf;
	uint16_t reg = 0;
	int ret = 0;

	if (ts->package_id < 3400) {
		ret = i2c_syn_read(ts->client, get_address_base(ts, ts->finger_func_idx, QUERY_BASE) + 1, data, 1);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:1", __func__);
		if ((data[0] & 0x07) == 5)
			ts->finger_support = 10;
		else if ((data[0] & 0x07) < 5)
			ts->finger_support = (data[0] & 0x07) + 1;
		else {
			pr_info("[TP] %s: number of fingers not define: %x\n",
				__func__, data[0] & 0x07);
			return SYN_PROCESS_ERR;
		}
	} else {
		struct synaptics_rmi4_f12_query_5 query_5;
		ret = i2c_syn_read(ts->client, get_address_base(ts, ts->finger_func_idx, QUERY_BASE) + 5, query_5.data, sizeof(query_5.data));
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:0", __func__);

		ts->ctrl_10_offset = query_5.ctrl0_is_present +
				query_5.ctrl1_is_present +
				query_5.ctrl2_is_present +
				query_5.ctrl3_is_present +
				query_5.ctrl4_is_present +
				query_5.ctrl5_is_present +
				query_5.ctrl6_is_present +
				query_5.ctrl7_is_present +
				query_5.ctrl8_is_present +
				query_5.ctrl9_is_present;
		ts->ctrl_15_offset = ts->ctrl_10_offset +
				query_5.ctrl10_is_present +
				query_5.ctrl11_is_present +
				query_5.ctrl12_is_present +
				query_5.ctrl13_is_present +
				query_5.ctrl14_is_present;

		ts->ctrl_23_offset = ts->ctrl_15_offset +
				query_5.ctrl15_is_present +
				query_5.ctrl16_is_present +
				query_5.ctrl17_is_present +
				query_5.ctrl18_is_present +
				query_5.ctrl19_is_present +
				query_5.ctrl20_is_present +
				query_5.ctrl21_is_present +
				query_5.ctrl22_is_present;

		ret = i2c_syn_read(ts->client, get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ts->ctrl_23_offset, data, 2);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:1", __func__);
		ts->finger_support = data[1];

		if ((ts->package_id == 3528) && ts->cover_setting[0]) {
			if (ts->packrat_number >= SYNAPTICS_FW_35_COVER) {
				ret = i2c_syn_read(ts->client, get_address_base(ts, 0x12, CONTROL_BASE) + ts->ctrl_10_offset, data, 2);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:4", __func__);
				memcpy(ts->f12_ctrl10, data, 2);
				ts->uncover_setting[0] = data[1];
				ts->uncover_setting[5] = data[0];

				ret = i2c_syn_read(ts->client, get_address_base(ts, 0x12, CONTROL_BASE) + ts->ctrl_15_offset, data, 2);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:5", __func__);
				memcpy(ts->f12_ctrl15, data, 2);
				ts->uncover_setting[1] = data[0];
				ts->uncover_setting[7] = data[1];

				ret = i2c_syn_read(ts->client, get_address_base(ts, 0x51, CONTROL_BASE) + 2, data, 1);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:6", __func__);
				ts->uncover_setting[2] = data[0];

				ret = i2c_syn_read(ts->client, get_address_base(ts, 0x51, CONTROL_BASE) + 3, data, 1);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:7", __func__);
				ts->uncover_setting[3] = data[0];

				ret = i2c_syn_read(ts->client, get_address_base(ts, 0x51, CONTROL_BASE) + 4, data, 1);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:8", __func__);
				ts->uncover_setting[4] = data[0];

				ret = i2c_syn_read(ts->client, get_address_base(ts, 0x54, CONTROL_BASE) + 28, data, 8);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:3", __func__);
				ts->uncover_setting[6] = data[0];

				ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, CONTROL_BASE) + 3, data, 1);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:9", __func__);
				ts->uncover_setting[8] = data[0];
			} else {
				ret = i2c_syn_read(ts->client, get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ts->ctrl_10_offset, data, 2);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:2", __func__);
				ts->uncover_setting[0] = data[0];
				ts->uncover_setting[1] = data[1];
				ret = i2c_syn_read(ts->client, get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ts->ctrl_15_offset, data, 2);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:3", __func__);
				ts->uncover_setting[2] = data[0];
				ts->uncover_setting[3] = data[1];

				ret = i2c_syn_read(ts->client, get_address_base(ts, 0x54, CONTROL_BASE) + 26, data, 8);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:4", __func__);
				memcpy(ts->f54_ctrl89, data, 8);
				ts->uncover_setting[4] = data[4];

				ret = i2c_syn_read(ts->client, get_address_base(ts, 0x54, CONTROL_BASE) + 28, data, 1);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:5", __func__);
				ts->uncover_setting[5] = data[0];

				ret = i2c_syn_read(ts->client, get_address_base(ts, 0x54, CONTROL_BASE) + 27, data, 5);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:6", __func__);
				memcpy(ts->f54_ctrl91, data, 5);
				ts->uncover_setting[6] = data[4];
			}
			
			ts->cover_enable = 0;
		}else if ((ts->package_id == 3508) && ts->cover_setting[0]) {
			if (ts->packrat_number >= SYNAPTICS_FW_3508_COVER) {
				ret = i2c_syn_read(ts->client, get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ts->ctrl_15_offset, data, 1);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:3", __func__);
				ts->uncover_setting[0] = data[0];

			} else {
				ret = i2c_syn_read(ts->client, get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ts->ctrl_15_offset, data, 2);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:3", __func__);
				ts->uncover_setting[0] = data[0];
				ts->uncover_setting[1] = data[1];
			}

			ts->cover_enable = 0;

		}

	}

	reg = get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ((ts->package_id < 3400) ? 6: 0);
	ret = i2c_syn_read(ts->client, reg, data, 4);
	if (ret < 0)
		return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:2", __func__);

	ts->max[0] = data[0] | data[1] << 8;
	ts->max[1] = data[2] | data[3] << 8;

	if (get_address_base(ts, 0x54, FUNCTION)) {
		ret = i2c_syn_read(ts->client, get_address_base(ts, 0x54, QUERY_BASE), data, 2);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:3", __func__);

		ts->y_channel = data[0];
		ts->x_channel = data[1];

		num_channel = ts->y_channel + ts->x_channel;
		buf = kzalloc(num_channel + 1, GFP_KERNEL);
		if (buf == NULL) {
			pr_info("[TP] %s: memory allocate fail\n", __func__);
			return -ENOMEM;
		}
		if (ts->packrat_number < SYNAPTICS_FW_3_2_PACKRAT)
			ret = i2c_syn_read(ts->client, get_address_base(ts, 0x54, CONTROL_BASE) + 17,
				buf, num_channel + 1);
		else
			ret = i2c_syn_read(ts->client, get_address_base(ts, 0x55, CONTROL_BASE),
				buf, num_channel + 1);
		if (ret < 0) {
			kfree(buf);
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:4", __func__);
		}

		for (i = 1; i < num_channel + 1; i++) {
			if (buf[i] == 0xFF) {
				if (i <= num_channel - ts->x_channel)
					ts->y_channel--;
				else
					ts->x_channel--;
			}
		}

		if (buf[0] & 0x01)
			swap(ts->y_channel, ts->x_channel);
		kfree(buf);

		ts->temp_report_data = kzalloc(4 * ts->x_channel * ts->y_channel, GFP_KERNEL);
		ts->report_data = kzalloc(4 * ts->x_channel * ts->y_channel, GFP_KERNEL);
		if(ts->temp_report_data == NULL || ts->report_data == NULL)
			return -ENOMEM;
		if (ts->package_id == 3528) {
			ts->report_data_32 = kzalloc(4 * (ts->x_channel + ts->y_channel), GFP_KERNEL);
			if(ts->report_data_32 == NULL)
				return -ENOMEM;
		}

		ret = i2c_syn_read(ts->client,
			get_address_base(ts, 0x54, CONTROL_BASE) + 0x10, &ts->relaxation, 1);
		if (ret < 0)
			return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:5", __func__);

	}
	pr_info("[TP] %s: finger_support:%d, max_x:%d, max_y:%d, X:%d, Y:%d, ts->relaxation:%d", __func__,
		ts->finger_support, ts->max[0], ts->max[1], ts->x_channel, ts->y_channel, ts->relaxation);

	if (ts->packrat_number < SYNAPTICS_FW_NOCAL_PACKRAT) {
		if (ts->large_obj_check) {
			if (ts->package_id < 3400) {
				reg = get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ((ts->package_id == 2200) ? 0x26: 0x29);
				ret = i2c_syn_read(ts->client, reg, &ts->default_large_obj, 1);
				if (ret < 0)
						return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:6", __func__);
				pr_info("[TP] %s: ts->default_large_obj: %x\n", __func__, ts->default_large_obj);
			}
		}

		if (ts->segmentation_bef_unlock) {
			if (ts->package_id < 3400) {
				reg = get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ((ts->package_id == 2200) ? 0x25: 0x22);
				ret = i2c_syn_read(ts->client, reg, &ts->segmentation_aft_unlock, 1);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:7", __func__);
				pr_info("[TP] %s: ts->segmentation_aft_unlock: %x\n", __func__, ts->segmentation_aft_unlock);
			}
		}

		if (ts->threshold_bef_unlock) {
			if (ts->package_id < 3400) {
				reg = get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ((ts->package_id == 2200) ? 0x0A: 0x0C);
				ret = i2c_syn_read(ts->client, reg, &ts->threshold_aft_unlock, 1);
				if (ret < 0)
					return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:8", __func__);
				pr_info("[TP] %s: ts->z_threshold_aft_unlock: %x\n", __func__, ts->threshold_aft_unlock);
			}
		}

		if (ts->saturation_bef_unlock) {
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x02, data, 2);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:8", __func__);
			ts->saturation_aft_unlock = (data[1] << 8) | data[0];
			pr_info("[TP] %s: ts->saturation_aft_unlock: %x\n", __func__, ts->saturation_aft_unlock);
		}

		if (ts->PixelTouchThreshold_bef_unlock) {
			ret = i2c_syn_read(ts->client,
				get_address_base(ts, 0x54, CONTROL_BASE) + 0x04, &ts->PixelTouchThreshold_aft_unlock, 1);
			if (ret < 0)
				return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "r:9", __func__);
			pr_info("[TP] %s: ts->PixelTouchThreshold_aft_unlock: %x\n", __func__, ts->PixelTouchThreshold_aft_unlock);
		}
	}

	return 0;
}

#ifdef CONFIG_OF
static ssize_t syn_vkeys_show(struct kobject  *obj,
		struct kobj_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = gl_ts;
	strlcpy(buf, ts->vkey_buf, MAX_BUF_SIZE);
	return strnlen(buf, MAX_BUF_SIZE);
}

static struct kobj_attribute syn_vkeys_attr = {
	.attr = {
		.name = "virtualkeys.synaptics-rmi-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &syn_vkeys_show,
};

static struct attribute *syn_properties_attrs[] = {
	&syn_vkeys_attr.attr,
	NULL
};

static struct attribute_group syn_properties_attr_group = {
	.attrs = syn_properties_attrs,
};

static void syn_init_vkeys(void)
{
	int rc = 0;
	static struct kobject *syn_properties_kobj;

	pr_info("[TP] init virtual key");
	syn_properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (syn_properties_kobj)
		rc = sysfs_create_group(syn_properties_kobj, &syn_properties_attr_group);
	if (!syn_properties_kobj || rc)
		pr_err("%s: failed to create board_properties\n", __func__);
	return;
}

static void synaptics_vk_parser(struct device_node *dt, const char *str,
				struct synaptics_i2c_rmi_platform_data *pdata)
{
	u32 data = 0;
	const char *range_str;
	uint8_t cnt = 0, i = 0;
	uint32_t coords[4] = {0};
	struct device_node *node, *pp = NULL;
	struct synaptics_virtual_key *vk;

	node = of_parse_phandle(dt, str, 0);
	if (node == NULL) {
		pr_info("[TP] DT-No vk info in DT");
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
			if (of_property_read_u32(pp, "keycode", &data) == 0)
				vk[i].keycode = data;
			if (of_property_read_u32_array(pp, "range", coords, 4) == 0) {
				vk[i].x_range_min = coords[0], vk[i].x_range_max = coords[1];
				vk[i].y_range_min = coords[2], vk[i].y_range_max = coords[3];
			}

			if (of_property_read_string(pp, "attr_range", &range_str) == 0)
				memcpy(vk[i].attr_range, range_str, SYN_VK_ATTR_STRING);
			else
				pr_info("[TP] attr range failed");
			i++;
		}
		pdata->virtual_key = vk;
	}
}

static int synaptics_parse_config(struct synaptics_ts_data *ts, struct synaptics_i2c_rmi_platform_data *pdata)
{
	struct synaptics_config *cfg_table;
	struct device_node *node, *pp = NULL;
	struct property *prop;
	uint8_t cnt = 0, i = 0;
	u32 data = 0;
	uint32_t coords[4] = {0}, cover[9] = {0};
	int len = 0, ret = 0;

	node = ts->client->dev.of_node;
	if (node == NULL) {
		pr_err("[TP] %s, can't find device_node", __func__);
		return -ENODEV;
	}

	while ((pp = of_get_next_child(node, pp)))
		cnt++;

	if (!cnt)
		return -ENODEV;

	cfg_table = kzalloc(cnt * (sizeof *cfg_table), GFP_KERNEL);
	if (!cfg_table)
	{
		kfree(cfg_table);
		return -ENOMEM;
	}

	pp = NULL;
	while ((pp = of_get_next_child(node, pp))) {
		if (of_property_read_u32(pp, "default_cfg", &data) == 0)
			cfg_table[i].default_cfg = data;

		if (of_property_read_u32(pp, "sensor_id", &data) == 0)
			cfg_table[i].sensor_id = (data | SENSOR_ID_CHECKING_EN);

		if (of_property_read_bool(pp, "mfgconfig")) {
			cfg_table[i].mfgconfig = 1;
		} else
			cfg_table[i].mfgconfig = 0;

		if (of_property_read_u32(pp, "pr_number", &data) == 0)
			cfg_table[i].pr_number = data;

		if (of_property_read_u32_array(pp, "syn,pl-coords", coords, 4) == 0) {
			cfg_table[i].pl_x_min = coords[0], cfg_table[i].pl_x_max = coords[1];	
			cfg_table[i].pl_y_min = coords[2], cfg_table[i].pl_y_max = coords[3];	
		}

		if (of_property_read_u32(pp, "vkey_setting", &data) == 0)
			cfg_table[i].vkey_setting = data;

		if (ts->package_id == 3528 )
		{
			if (ts->packrat_number >= SYNAPTICS_FW_35_COVER) {
				if (of_property_read_u32_array(pp, "cover_setting", cover, 9) == 0) {
					cfg_table[i].cover_setting[0] = cover[0];
					cfg_table[i].cover_setting[1] = cover[1];
					cfg_table[i].cover_setting[2] = cover[2];
					cfg_table[i].cover_setting[3] = cover[3];
					cfg_table[i].cover_setting[4] = cover[4];
					cfg_table[i].cover_setting[5] = cover[5];
					cfg_table[i].cover_setting[6] = cover[6];
					cfg_table[i].cover_setting[7] = cover[7];
					cfg_table[i].cover_setting[8] = cover[8];
				}
			} else {
				if (of_property_read_u32_array(pp, "cover_setting", cover, 7) == 0) {
					cfg_table[i].cover_setting[0] = cover[0];
					cfg_table[i].cover_setting[1] = cover[1];
					cfg_table[i].cover_setting[2] = cover[2];
					cfg_table[i].cover_setting[3] = cover[3];
					cfg_table[i].cover_setting[4] = cover[4];
					cfg_table[i].cover_setting[5] = cover[5];
					cfg_table[i].cover_setting[6] = cover[6];
				}
			}
		}
		else if (ts->package_id == 3508)
		{
			if (ts->packrat_number >= SYNAPTICS_FW_3508_COVER) {
				if (of_property_read_u32_array(pp, "cover_setting", cover, 1) == 0) {
					cfg_table[i].cover_setting[0] = cover[0];
				}
			} else {
				if (of_property_read_u32_array(pp, "cover_setting", cover, 2) == 0) {
					cfg_table[i].cover_setting[0] = cover[0];
					cfg_table[i].cover_setting[1] = cover[1];
				}
			}
		}

		prop = of_find_property(pp, "config", &len);
		if (!prop) {
			pr_debug( "[TP] %s:Looking up %s property in node %s failed",
					__func__, "config", pp->full_name);
			kfree(cfg_table);
			return -ENODEV;
		} else if (!len) {
			pr_debug("[TP] %s:Invalid length of configuration data\n",
					__func__);
			kfree(cfg_table);
			return -EINVAL;
		}

		cfg_table[i].length = len;
		memcpy(cfg_table[i].config, prop->value, cfg_table[i].length);
#ifdef SYN_CONFIG_LOG_ENABLE
		pr_info("[TP] DT#%d-def_cfg:%d,id:%05x, pr:%d, len:%d,", i,
			cfg_table[i].default_cfg, cfg_table[i].sensor_id,
			cfg_table[i].pr_number, cfg_table[i].length);
		printk(" cfg=[%02x,%02x,%02x,%02x]", cfg_table[i].config[0], cfg_table[i].config[1],
			cfg_table[i].config[2], cfg_table[i].config[3]);
		printk(" pl=[x_m%02d y_M%04d]\n", cfg_table[i].pl_x_min, cfg_table[i].pl_y_max);
#endif
		i++;
	}

	i = 0;
	if (ts->blmode == true) {	
		struct device_node *dt = ts->client->dev.of_node;
		pr_info("[TP] Stay BOOTLOADER mode\n");
		pdata->gpio_irq = of_get_named_gpio(dt, "synaptics,irq-gpio", 0);
		if (!gpio_is_valid(pdata->gpio_irq)) {
			pr_info("[TP] DT:gpio_irq value is not valid\n");
		}
		while (cfg_table[i].default_cfg != 1) {
			if (cfg_table[i].default_cfg == 0) {
				pr_err("[TP] TOUCH_ERR: NOT eanble reovery method\n");
			}
			i++;
		}
		memcpy(ts->config, cfg_table[i].config, sizeof(cfg_table[i].config));
		ret = syn_config_update(ts, pdata->gpio_irq);
		if (ret < 0) {
			pr_err("[TP] TOUCH_ERR: syn_config_update fail\n");
			kfree(cfg_table);
			return -EINVAL;
		} else if (ret == 0) {
			pr_info("[TP] BL reocver_config_update success\n");
			ts->blmode = false;
			if (syn_get_version(ts) < 0) {
				pr_err("[TP] BLmode re-read pr number err\n");
				kfree(cfg_table);
				return -EINVAL;
			}
		} else
			pr_info("[TP]%s:the same config version and CRC but touch controller\
				always stay in bootloader mode\n", __func__);
		kfree(cfg_table);
		return 0;
	}

	i = 0;	
	while (cfg_table[i].pr_number > ts->packrat_number) {
		i++;
	}

	if (board_build_flag() != BUILD_MODE_MFG) {
		while (cfg_table[i].mfgconfig == 1) {
			if (i < cnt)
				i++;
		}
		pr_info("[TP] not mfg config\n");
	}

	while (cfg_table[i].sensor_id > 0 && (cfg_table[i].sensor_id != (SENSOR_ID_CHECKING_EN | ts->tw_vendor))) {
		pr_info("[TP] id:%#x!=%#x, (i++)",cfg_table[i].sensor_id, (SENSOR_ID_CHECKING_EN | ts->tw_vendor));
		i++;
	}
	if (i <= cnt) {
		pr_info("[TP] DT-%s cfg idx(%d) in cnt(%d)", __func__, i, cnt);
		pdata->packrat_number = cfg_table[i].pr_number;
		pdata->sensor_id      = cfg_table[i].sensor_id;
		pdata->config_length  = cfg_table[i].length;
		if (cfg_table[i].pl_x_min > 0 && cfg_table[i].pl_y_max > 0) {
			pdata->abs_x_min = cfg_table[i].pl_x_min, pdata->abs_x_max = cfg_table[i].pl_x_max;
			pdata->abs_y_min = cfg_table[i].pl_y_min, pdata->abs_y_max = cfg_table[i].pl_y_max;
		}
		if (ts->packrat_number >= SYNAPTICS_FW_35_COVER) {
			memcpy(pdata->cover_setting, cfg_table[i].cover_setting, 9);
		} else {
			memcpy(pdata->cover_setting, cfg_table[i].cover_setting, 7);
		}
		memcpy(pdata->config, cfg_table[i].config,(ts->package_id > 3500 ? SYN_CONFIG_SIZE_35XX : SYN_CONFIG_SIZE));
		pr_info("[TP] DT#%d-def_cfg:%d,id:%05x, pr:%d, len:%d,", i,
			cfg_table[i].default_cfg, cfg_table[i].sensor_id,
			cfg_table[i].pr_number, cfg_table[i].length);
		printk(" cfg=[%02x,%02x,%02x,%02x]", cfg_table[i].config[0], cfg_table[i].config[1],
			cfg_table[i].config[2], cfg_table[i].config[3]);
		printk(" pl=[x_m%02d y_M%04d]\n", cfg_table[i].pl_x_min, cfg_table[i].pl_y_max);
		switch(cfg_table[i].vkey_setting) {
			case 0:
				synaptics_vk_parser(node, "virtualkey0", pdata);
				break;
			case 1:
				synaptics_vk_parser(node, "virtualkey1", pdata);
				break;
			case 2:
				synaptics_vk_parser(node, "virtualkey2", pdata);
				break;
			default:
				synaptics_vk_parser(node, "virtualkey0", pdata);
				break;
		}
	} else {
		pr_err("[TP] DT-%s cfg idx(%d) > cnt(%d)", __func__, i, cnt);
		kfree(cfg_table);
		return -EINVAL;
	}
	kfree(cfg_table);
	return 0;
}

static int synaptics_parse_dt(struct synaptics_ts_data *ts,
				struct synaptics_i2c_rmi_platform_data *pdata)
{
	int rc, coords_size = 0, i;
	uint32_t coords[4] = {0};
	struct property *prop;
	struct device_node *dt = ts->client->dev.of_node;
	u32 data = 0;

	prop = of_find_property(dt, "synaptics,panel-coords", NULL);
	if (prop) {
		coords_size = prop->length / sizeof(u32);
		if (coords_size != 4)
			pr_debug("[TP] %s:Invalid panel coords size %d", __func__, coords_size);
	}

	if (of_property_read_u32_array(dt, "synaptics,panel-coords", coords, coords_size) == 0) {
		pdata->abs_x_min = coords[0], pdata->abs_x_max = coords[1];
		pdata->abs_y_min = coords[2], pdata->abs_y_max = coords[3];
		pr_info("[TP] DT-%s:panel-coords = %d, %d, %d, %d\n", __func__, pdata->abs_x_min,
				pdata->abs_x_max, pdata->abs_y_min, pdata->abs_y_max);
	}

	prop = of_find_property(dt, "synaptics,display-coords", NULL);
	if (prop) {
		coords_size = prop->length / sizeof(u32);
		if (coords_size != 4)
			pr_debug("[TP] %s:Invalid display coords size %d", __func__, coords_size);
	}
	rc = of_property_read_u32_array(dt, "synaptics,display-coords", coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		pr_debug("[TP] %s:Fail to read display-coords %d\n", __func__, rc);
		return rc;
	}
	pdata->display_width  = coords[1];
	pdata->display_height = coords[3];
	pr_info("[TP] DT-%s:display-coords = (%d, %d)", __func__, pdata->display_width,
		pdata->display_height);

	pdata->gpio_irq = of_get_named_gpio(dt, "synaptics,irq-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_irq)) {
		pr_info("[TP] DT:gpio_irq value is not valid\n");
	}

	pdata->gpio_reset = of_get_named_gpio(dt, "synaptics,rst-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_reset)) {
		pr_info("[TP] DT:gpio_rst value is not valid\n");
	}

	pdata->gpio_i2c = of_get_named_gpio(dt, "synaptics,i2c-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_i2c)) {
		pr_info("[TP] DT:gpio_i2c value is not valid\n");
	}
	ts->gpio_i2c = pdata->gpio_i2c;
	if (ts->gpio_i2c >= 0) {
		rc = gpio_request(ts->gpio_i2c, "synaptics_i2c_gpio");
		if (rc)
			pr_info("[TP]%s: Failed to obtain i2c_gpio pin: %d. Code: %d.", __func__, ts->gpio_i2c, rc);
	}
	pr_info("[TP] DT:gpio_irq=%d, gpio_rst=%d, gpio_i2c=%d", pdata->gpio_irq, pdata->gpio_reset, pdata->gpio_i2c);

	if (of_property_read_u32(dt, "synaptics,i2c_err_hlr", &data) == 0)
	{
		pdata->i2c_err_handler_en = data;
		if (pdata->i2c_err_handler_en) {
			ts->i2c_err_handler_en = pdata->i2c_err_handler_en;
			ts->gpio_reset = pdata->gpio_reset;
			ts->use_irq = 1;
			if (ts->gpio_reset) {
				rc = gpio_request(ts->gpio_reset, "synaptics_reset");
				if (rc)
					pr_info("[TP]%s: Failed to obtain reset pin: %d. Code: %d.", __func__, ts->gpio_reset, rc);
			}
		}
	}

	if (of_property_read_u32(dt, "tw_pin_mask", &data) == 0) {
		ts->tw_pin_mask = data;
		pr_info("[TP] tw_pin_mask :%x",ts->tw_pin_mask);
	}

	if (of_property_read_u32(dt, "report_type", &data) == 0) {
		pdata->report_type = data;
		pr_info("[TP] DT:report_type=%d", pdata->report_type);
	}

	if (of_property_read_u32(dt, "support_htc_event", &data) == 0) {
		pdata->support_htc_event = data;
		pr_info("[TP] DT:support_htc_event=%d", pdata->support_htc_event);
	}

	if (of_property_read_u32(dt, "hall_block_touch_time", &data) == 0) {
		pdata->hall_block_touch_time = data;
		pr_info("[TP] DT:hall_block_touch_time=%d", pdata->hall_block_touch_time);
	}

	if (ts->tw_pin_mask) {
		for (i=0; i<SYN_UPDATE_RETRY_TIMES; i++) {
			rc = syn_get_tw_vendor(ts, pdata->gpio_irq);
			if (rc == 0)
				break;
		}
		if (rc < 0) {
			pr_err("[TP] TOUCH_ERR: syn_get_tw_vendor fail\n");
			return -EINVAL;
		}
	}

	rc = synaptics_parse_config(ts, pdata);
	if (rc < 0) {
		pr_err("[TP] DT:cfg table parser FAIL. ret=%d\n", rc);
		return rc;
	} else if (rc == 0)
		pr_info("[TP] DT parser Done\n");

	return 0;
}
#endif

static void syn_fb_register(struct work_struct *work)
{
	int ret = 0;
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data,
							work_att.work);
	pr_info("[TP] %s in", __func__);

	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret)
		pr_err("[TP] TOUCH_ERR:Unable to register fb_notifier: %d\n", ret);
}

static int __devinit synaptics_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	uint8_t i, buf = 0;
	int ret = 0;
	struct synaptics_i2c_rmi_platform_data *pdata;
	uint8_t data = 0;

	pr_info("[TP] %s: enter", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "[TP] TOUCH_ERR: synaptics_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	ts->client = client;
	i2c_set_clientdata(client, ts);

	ret = i2c_syn_read(ts->client, 0x00EE, &data, 1);
	if (ret < 0) {
		pr_info("[TP] No Synaptics chip\n");
		goto err_detect_failed;
	}

	if (syn_pdt_scan(ts, SYN_BL_PAGE) < 0) {
		printk(KERN_ERR "[TP] TOUCH_ERR: PDT scan fail\n");
		goto err_init_failed;
	}

	for (i = 0; i < 10; i++) {
		ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, DATA_BASE), &data, 1);
		if (ret < 0) {
			i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "read device status failed!", __func__);
			goto err_detect_failed;
		}
		if (data & 0x44) {
			msleep(20);
#ifdef SYN_FLASH_PROGRAMMING_LOG
			pr_info("[TP] synaptics probe: F01_data: %x touch controller stay in bootloader mode!\n", data);
#endif
		} else if (data & 0x40) {
			pr_debug("[TP] TOUCH_ERR: synaptics probe: F01_data: %x touch controller stay in bootloader mode!\n", data);
			goto err_detect_failed;
		} else
			break;
	}

	ts->blmode = (i == 10) ? true : false;
	if (ts->blmode) {
#ifdef CONFIG_OF
		uint8_t data1[3] = {0};
#endif
		pr_info("[TP] TOUCH_ERR: synaptics probe: touch controller doesn't enter UI mode! F01_data: %x\n", data);
		if (syn_pdt_scan(ts, SYN_BL_PAGE) < 0) {
			printk(KERN_ERR "[TP] TOUCH_ERR: PDT scan fail\n");
			goto err_init_failed;
		}
#ifndef CONFIG_OF
		pdata = ts->client->dev.platform_data;
		if (pdata) {
			uint8_t num = 0;
			while (pdata->default_config != 1) {
				if (pdata->default_config == 0) {
					printk(KERN_ERR "[TP] TOUCH_ERR: touch controller stays in bootloader mode "
						"and recovery method doesn't enable\n");
					goto err_init_failed;
				}
				pdata++;
				num++;
			}
			ts->config = pdata->config;

			ret = syn_config_update(ts, pdata->gpio_irq);
			if (ret < 0) {
				printk(KERN_ERR "[TP] TOUCH_ERR: syn_config_update fail\n");
				goto err_init_failed;
			} else if (ret == 0)
				pr_info("[TP] syn_config_update success\n");
			else
				pr_info("[TP] Warning: syn_config_update: the same "
					"config version and CRC but touch controller always stay in bootloader mode\n");
			pdata = pdata - num;
		}
#else
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL) {
			ret = -ENOMEM;
			goto err_alloc_dt_pdata_failed;
		}
		ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, QUERY_BASE) + 18, data1, 3);
		if (ret < 0) {
			printk(KERN_ERR "[TP] TOUCH_ERR: read packrat_number fail\n");
			goto err_init_failed;
		}
		ts->packrat_number = data1[2] << 16 | data1[1] << 8 | data1[0];
		if (ts->packrat_number < 1328272)
			ts->package_id = 3200;
		else if (ts->packrat_number < 1501819)
			ts->package_id = 3400;
		else if (ts->packrat_number >= 1501819)
			ts->package_id = 3508;
		ret = synaptics_parse_config(ts, pdata);
		if (ret < 0) {
			pr_err("[TP] TOUCH_ERR: config update fail\n. ret=%d\n", ret);
			goto err_init_failed;
		} else if (ret == 0)
			pr_info("[TP] config update Done\n");
		kfree(pdata);
#endif

		if (ts->address_table != NULL) {
			kfree(ts->address_table);
			ts->address_table = NULL;
		}
	}

	if (syn_pdt_scan(ts, SYN_MAX_PAGE) < 0) {
		printk(KERN_ERR "[TP] TOUCH_ERR: PDT scan fail\n");
		goto err_init_failed;
	}
	if (board_mfg_mode() == MFG_MODE_OFFMODE_CHARGING ||
	    board_mfg_mode() == MFG_MODE_POWER_TEST) {
		pr_info("[TP] %s: offmode charging. Set touch chip to sleep mode and skip touch driver probe\n", __func__);
		ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x01, CONTROL_BASE), 0x01); 
		if (ret < 0)
			i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "sleep: 0x01", __func__);
		ret = -ENODEV;
		goto err_check_offmode_charging;
	}

	ts->finger_func_idx = 0x11;
	if (syn_get_version(ts) < 0) {
		printk(KERN_ERR "[TP] TOUCH_ERR: syn_get_version fail\n");
		goto err_init_failed;
	}

	if (client->dev.of_node) { 
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL) {
			ret = -ENOMEM;
			goto err_alloc_dt_pdata_failed;
		}
		ret = synaptics_parse_dt(ts, pdata);
		if (ret < 0) {
			pr_info("[TP] pdata is NULL for DT\n");
			goto err_dt_platform_data_fail;
		}
	} else {
		pdata = client->dev.platform_data;
		if (pdata == NULL) {
			pr_info("[TP] pdata is NULL(dev.platform_data)\n");
			goto err_get_platform_data_fail;
		}
	}

	if (pdata) {
#ifndef CONFIG_OF
		while (pdata->packrat_number && pdata->packrat_number > ts->packrat_number) {
			pdata++;
		}
		if (pdata->tw_pin_mask) {
			ts->tw_pin_mask = pdata->tw_pin_mask;
			for (i=0; i<SYN_UPDATE_RETRY_TIMES; i++) {
				ret = syn_get_tw_vendor(ts, pdata->gpio_irq);
				if (ret == 0)
					break;
			}
			if (ret < 0) {
				printk(KERN_ERR "[TP] TOUCH_ERR: syn_get_tw_vendor fail\n");
				goto err_init_failed;
			}
		}
		while (pdata->sensor_id > 0 && pdata->sensor_id != (SENSOR_ID_CHECKING_EN | ts->tw_vendor)) {
			pdata++;
		}
#endif
		pr_info("[TP] %s: pdata->version = %x, pdata->packrat_number = %d, pdata->sensor_id = %x\n",
			__func__, pdata->version, pdata->packrat_number, pdata->sensor_id);

		if (!pdata->packrat_number) {
			printk(KERN_ERR "[TP] TOUCH_ERR: get null platform data\n");
			goto err_init_failed;
		}

		ts->power                          = pdata->power;
		ts->flags                          = pdata->flags;
		ts->htc_event                      = pdata->report_type;
		ts->filter_level                   = pdata->filter_level;
#ifdef CONFIG_MOTION_FILTER
		ts->reduce_report_level            = pdata->reduce_report_level;
#endif
		ts->gpio_irq                       = pdata->gpio_irq;
		ts->gpio_reset                     = pdata->gpio_reset;
		ts->gpio_i2c                       = pdata->gpio_i2c;
		ts->large_obj_check                = pdata->large_obj_check;
		ts->support_htc_event              = pdata->support_htc_event;
		ts->mfg_flag                       = pdata->mfg_flag;
		ts->segmentation_bef_unlock        = pdata->segmentation_bef_unlock;
		ts->i2c_err_handler_en             = pdata->i2c_err_handler_en;
		ts->threshold_bef_unlock           = pdata->threshold_bef_unlock;
		ts->saturation_bef_unlock          = pdata->saturation_bef_unlock;
		ts->energy_ratio_relaxation        = pdata->energy_ratio_relaxation;
		ts->multitouch_calibration         = pdata->multitouch_calibration;
		ts->psensor_detection              = pdata->psensor_detection;
		ts->PixelTouchThreshold_bef_unlock = pdata->PixelTouchThreshold_bef_unlock;
		#ifdef SYN_CABLE_CONTROL
		ts->cable_support                  = pdata->cable_support; 
		#endif
		memcpy(ts->config, pdata->config, sizeof(pdata->config));
		memcpy(ts->cover_setting, pdata->cover_setting, sizeof(pdata->cover_setting));
		ts->hover_mode                     = 0;
		ts->suspended                      = false;
		ts->hall_block_touch_event         = 0;
		ts->hall_block_touch_time          = pdata->hall_block_touch_time;
		ts->prev_NS                        = -1;
		ts->prev_Freq                      = -1;

		if (pdata->virtual_key) {
			uint8_t pos = 0;
 			ts->button = pdata->virtual_key;
			ts->vkey_buf = kzalloc(MAX_BUF_SIZE, GFP_KERNEL);
			for (i = 0; i < 2; i++) {
				pos += snprintf(ts->vkey_buf + pos, PAGE_SIZE, "%s:%d%s\n",
						VKEY_VER_CODE, ts->button[i].keycode,
						ts->button[i].attr_range);
			}
			pr_info("[TP] vk cnt:%d:%s\n%s", i, ts->vkey_buf, ts->vkey_buf+pos);
			syn_init_vkeys();
		}
	}
	pr_info("[TP] %s:config =[%02x,%02x,%02x,%02x] ", __func__,
		ts->config[0], ts->config[1], ts->config[2], ts->config[3]);

#ifndef SYN_DISABLE_CONFIG_UPDATE
	for (i=0; i < SYN_UPDATE_RETRY_TIMES; i++) {
		ret = syn_config_update(ts, pdata->gpio_irq);
		if (ret >= 0)
			break;
	}
	if (ret < 0) {
		pr_info("[TP] TOUCH_ERR: syn_config_update fail\n");
		goto err_init_failed;
	} else if (ret == 0)
		pr_info("[TP] syn_config_update success\n");
	else
		pr_info("[TP] syn_config_update: the same config version and CRC\n");
#else
	if (ts->tw_pin_mask) {
		ret = disable_flash_programming(ts, 0);
		if (ret < 0) {
			printk(KERN_ERR "[TP] TOUCH_ERR: disable_flash_programming fail\n");
			goto err_init_failed;
		}
	}
#endif
	if (syn_pdt_scan(ts, SYN_MAX_PAGE) < 0) {
		printk(KERN_ERR "[TP] TOUCH_ERR: PDT scan fail\n");
		goto err_init_failed;
	}

#ifndef SYN_DISABLE_CONFIG_UPDATE
	if (pdata->customer_register[CUS_REG_BASE]) {
		ret = i2c_syn_write(ts->client, pdata->customer_register[CUS_REG_BASE],
			&pdata->customer_register[CUS_BALLISTICS_CTRL], CUS_REG_SIZE - 1);
		pr_info("[TP] Loads customer register\n");
	}
#endif
	if (syn_get_information(ts) < 0) {
		printk(KERN_ERR "[TP] TOUCH_ERR: syn_get_information fail\n");
		goto err_syn_get_info_failed;
	}

	if (pdata->abs_x_max  == 0 && pdata->abs_y_max == 0) {
		ts->layout[0] = ts->layout[2] = 0;
		ts->layout[1] = ts->max[0];
		ts->layout[3] = ts->max[1];
	} else {
		ts->layout[0] = pdata->abs_x_min;
		ts->layout[1] = pdata->abs_x_max;
		ts->layout[2] = pdata->abs_y_min;
		ts->layout[3] = pdata->abs_y_max;
	}

	if (pdata->display_width && pdata->display_height) {
		pr_info("[TP] Load display resolution: %dx%d\n", pdata->display_width, pdata->display_height);
		ts->width_factor = (pdata->display_width<<SHIFT_BITS)/(ts->layout[1]-ts->layout[0]);
		ts->height_factor = (pdata->display_height<<SHIFT_BITS)/(ts->layout[3]-ts->layout[2]);
	}

	if (get_tamper_sf() == 0) {
		ts->debug_log_level |= BIT(3);
		pr_info("[TP] Debug log level=0x%02X\n", ts->debug_log_level);
	}

	if (get_address_base(ts, 0x19, FUNCTION)) {
		ret = i2c_syn_read(ts->client, get_address_base(ts, 0x19, QUERY_BASE) + 1,
			&ts->key_number, 1);
		if (ret < 0) {
			i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "F19 Query fail", __func__);
			goto err_F19_query_failed;
		}
		for (i = 0; i < ts->key_number; i++) {
			ts->key_postion_x[i] =
				(ts->layout[1] - ts->layout[0]) * (i * 2 + 1) / (ts->key_number * 2)
				+ ts->layout[0];
			pr_info("[TP] ts->key_postion_x[%d]: %d\n",
				i, ts->key_postion_x[i]);
		}
		ts->key_postion_y = ts->layout[2] +
			(21 * (ts->layout[3] - ts->layout[2]) / 20);
		pr_info("[TP] ts->key_postion_y: %d\n", ts->key_postion_y);
	}

	ret = synaptics_init_panel(ts);
	if (ret < 0) {
		printk(KERN_ERR "[TP] TOUCH_ERR: synaptics_init_panel fail\n");
		goto err_init_panel_failed;
	}

	init_waitqueue_head(&syn_data_ready_wq);

	if (ts->psensor_detection) {
		INIT_WORK(&ts->psensor_work, synaptics_ts_close_psensor_func);
		ts->syn_psensor_wq = create_singlethread_workqueue("synaptics_psensor_wq");
		if (!ts->syn_psensor_wq)
			goto err_create_wq_failed;
	}

	if (ts->cover_setting[0])
	{
		INIT_WORK(&ts->cover_work, syn_set_cover_func);
		ts->syn_cover_wq = create_singlethread_workqueue("synaptics_cover_wq");
		if (!ts->syn_cover_wq)
			goto err_create_wq_failed;
	}

	ret = synaptics_input_register(ts);
	if (ret) {
		printk(KERN_ERR "[TP] TOUCH_ERR: synaptics_ts_probe: "
				"Unable to register %s input device\n",
				ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	gl_ts = ts;

	ts->irq_enabled = 0;
	client->irq = gpio_to_irq(pdata->gpio_irq);
	pr_info("[TP] gpio = %d, irq = %d\n", pdata->gpio_irq, client->irq);
	if (!gpio_get_value(pdata->gpio_irq)) {
		ret = i2c_syn_read(ts->client, get_address_base(ts, 0x01, DATA_BASE) + 1, &buf, 1);
		pr_info("[TP] gpio status = %d, ret = %x\n", gpio_get_value(pdata->gpio_irq), buf);
	}
	if (client->irq) {
		ts->use_irq = 1;
		ret = request_threaded_irq(client->irq, NULL, synaptics_irq_thread,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT, client->name, ts);
		if (ret == 0) {
			ts->irq_enabled = 1;
			ret = i2c_syn_read(ts->client,
					get_address_base(ts, 0x01, CONTROL_BASE) + 1, &ts->intr_bit, 1);
			if (ret < 0) {
				free_irq(client->irq, ts);
				i2c_syn_error_handler(ts, ts->i2c_err_handler_en,
							"get interrupt bit failed", __func__);
				goto err_get_intr_bit_failed;
			}
			pr_info("[TP] %s: interrupt enable: %x\n", __func__, ts->intr_bit);

		} else {
			dev_err(&client->dev, "[TP] TOUCH_ERR: request_irq failed\n");
			ts->use_irq = 0;
		}
	}

	if (!ts->use_irq) {
		ts->syn_wq = create_singlethread_workqueue("synaptics_wq");
		if (!ts->syn_wq)
			goto err_create_wq_failed;

		INIT_WORK(&ts->work, synaptics_ts_work_func);

		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = synaptics_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

#ifdef CONFIG_FB
	ts->syn_att_wq = create_singlethread_workqueue("SYN_ATT_reuqest");
	if (!ts->syn_att_wq) {
		pr_err("[TP] allocate syn_att_wq failed\n");
		ret = -ENOMEM;
		goto err_get_intr_bit_failed;
	}
	INIT_DELAYED_WORK(&ts->work_att, syn_fb_register);
	queue_delayed_work(ts->syn_att_wq, &ts->work_att, msecs_to_jiffies(15000));

#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

#ifdef SYN_CABLE_CONTROL
	if (ts->cable_support) {
		usb_register_notifier(&cable_status_handler);
		
		ret = i2c_syn_read(ts->client,
			get_address_base(ts, ts->finger_func_idx, CONTROL_BASE), &ts->cable_config, 1);
		if (ret < 0) {
			printk(KERN_ERR "[TP] TOUCH_ERR: get cable config failed\n");
			goto err_get_cable_config_failed;
		}
		if (usb_get_connect_type())
			cable_tp_status_handler_func(1);
		pr_info("[TP] %s: ts->cable_config: %x\n", __func__, ts->cable_config);
	}
#endif
	register_notifier_by_psensor(&psensor_status_handler);
	register_notifier_by_hallsensor(&hallsensor_status_handler);
	synaptics_touch_sysfs_init();
#ifdef SYN_WIRELESS_DEBUG
	if (rmi_char_dev_register())
		pr_info("[TP] %s: error register char device", __func__);
#endif

	pr_info("[TP] synaptics_ts_probe: Start touchscreen %s in %s mode\n",
			ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");
#ifdef CONFIG_OF
	kfree(pdata);
#endif
#if defined(CONFIG_SECURE_TOUCH)
    init_completion(&ts->st_powerdown);
    init_completion(&ts->st_irq_processed);
#endif
	return 0;

#ifdef SYN_CABLE_CONTROL
err_get_cable_config_failed:
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		destroy_workqueue(ts->syn_wq);
#endif

err_create_wq_failed:

err_get_intr_bit_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_dt_platform_data_fail:
	kfree(pdata);

err_init_panel_failed:
err_F19_query_failed:
err_syn_get_info_failed:
err_alloc_dt_pdata_failed:
err_get_platform_data_fail:
	if (ts->report_data != NULL)
		kfree(ts->report_data);
	if (ts->temp_report_data != NULL)
		kfree(ts->temp_report_data);
err_init_failed:
err_check_offmode_charging:
	if (ts->address_table != NULL)
		kfree(ts->address_table);

err_detect_failed:
	kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int __devexit synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
#ifdef CONFIG_FB
	if (fb_unregister_client(&ts->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else {
		hrtimer_cancel(&ts->timer);
		if (ts->syn_wq)
			destroy_workqueue(ts->syn_wq);
	}
	if (ts->sr_input_dev != NULL)
		input_unregister_device(ts->sr_input_dev);
	input_unregister_device(ts->input_dev);

	synaptics_touch_sysfs_remove();

	if (ts->report_data != NULL)
		kfree(ts->report_data);
	if (ts->temp_report_data != NULL)
		kfree(ts->temp_report_data);
	if (ts->address_table != NULL)
		kfree(ts->address_table);
	kfree(ts);
	return 0;
}

static int synaptics_ts_suspend(struct device *dev)
{
	int ret;
	uint16_t reg = 0;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);

	if(ts->suspended)
	{
		pr_info("[TP] %s: Already suspended. Skipped. FW:%d;%d;%x;%x\n",
		__func__, ts->package_id, ts->packrat_number, syn_panel_version, ts->config_version);
		return 0;
	}
	else
	{
		ts->suspended = true;
		pr_info("[TP] %s: enter. FW:%d;%d;%x;%x\n",
		__func__, ts->package_id, ts->packrat_number, syn_panel_version, ts->config_version);
	}

	if (ts->use_irq) {
		if (ts->irq_enabled) {
			disable_irq(ts->client->irq);
			ts->irq_enabled = 0;
		}
	} else {
		hrtimer_cancel(&ts->timer);
		ret = cancel_work_sync(&ts->work);
	}

	if (ts->packrat_number < SYNAPTICS_FW_NOCAL_PACKRAT) {
		if (ts->psensor_detection) {
			if (ts->psensor_resume_enable == 1) {
				pr_info("[TP] %s: Disable P-sensor by Touch\n", __func__);
				ts->psensor_resume_enable = 0;
			}
		}

		if (ts->psensor_status == 0) {
			ts->pre_finger_data[0][0] = 0;
			ts->first_pressed = 0;

#ifdef SYN_CALIBRATION_CONTROL
			if (ts->mfg_flag != 1) {
				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, CONTROL_BASE) + 0x10, ts->relaxation);
				if (ret < 0)
					i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "fast relaxation", __func__);

				if (ts->energy_ratio_relaxation) {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, 0x54, CONTROL_BASE), 0x20);
					if (ret < 0)
						i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "energy ratio relaxation", __func__);
				}

				if (ts->saturation_bef_unlock) {
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, 0x54, CONTROL_BASE) + 0x02, ts->saturation_bef_unlock & 0xFF);
					if (ret < 0)
						return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "saturation capacitance", __func__);
					ret = i2c_syn_write_byte_data(ts->client,
						get_address_base(ts, 0x54, CONTROL_BASE) + 0x03, (ts->saturation_bef_unlock & 0xFF00) >> 8);
					if (ret < 0)
						return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "saturation capacitance", __func__);
					pr_info("[TP] touch suspend, saturation capacitance: %x\n", ts->saturation_bef_unlock);
				}

				if ( ts->PixelTouchThreshold_bef_unlock ) {
					if (ts->package_id <= 3400 ) {
						ret = i2c_syn_write_byte_data(ts->client,
							get_address_base(ts, 0x54, CONTROL_BASE) + 0x04, ts->PixelTouchThreshold_bef_unlock );
						if (ret < 0)
							return i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "F54_ANALOG_CTRL03 Pixel Touch Threshold", __func__);
						pr_info("[TP] touch suspend, set F54_ANALOG_CTRL03 Pixel Touch Threshold: %x\n", ts->PixelTouchThreshold_bef_unlock);
					}
				}

				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x54, COMMAND_BASE), 0x04);
				if (ret < 0)
					i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "force update", __func__);
				pr_info("[TP] touch suspend, fast relasxation: %x\n", ts->relaxation);
			}
#endif

			if (ts->large_obj_check) {
				if (ts->package_id < 3400) {
					reg = get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ((ts->package_id == 2200) ? 0x26: 0x29);
					ret = i2c_syn_write_byte_data(ts->client, reg, ts->default_large_obj & 0x7F);
					if (ret < 0)
						i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "large obj suppression", __func__);
					pr_info("[TP] touch suspend, set large obj suppression: %x\n", ts->default_large_obj & 0x7F);
				}
			}

			if (ts->segmentation_bef_unlock) {
				if (ts->package_id < 3400) {
					reg = get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ((ts->package_id == 2200) ? 0x25: 0x22);
					ret = i2c_syn_write_byte_data(ts->client, reg, ts->segmentation_bef_unlock);
					if (ret < 0)
						i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "segmentation aggressiveness", __func__);
					pr_info("[TP] touch suspend, set segmentation aggressiveness: %x\n", ts->segmentation_bef_unlock);
				}
			}

			if (ts->threshold_bef_unlock) {
				if (ts->package_id < 3400) {
					reg = get_address_base(ts, ts->finger_func_idx, CONTROL_BASE) + ((ts->package_id == 2200) ? 0x0A: 0x0C);
					ret = i2c_syn_write_byte_data(ts->client, reg, ts->threshold_bef_unlock);
					if (ret < 0)
						i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "Z Touch threshold", __func__);
					pr_info("[TP] touch suspend, set Z Touch threshold: %x\n", ts->threshold_bef_unlock);
				}
			}
		} else if (ts->psensor_detection)
			ts->psensor_phone_enable = 1;

	}
	if (ts->power)
		ts->power(0);
	else {
		if (ts->packrat_number >= SYNAPTICS_FW_NOCAL_PACKRAT) {
#if defined(CONFIG_SYNC_TOUCH_STATUS)
			if(ts->gpio_i2c < 0) {
				pr_info("[TP] %s: sleep: 0x01\n", __func__);
				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x01, CONTROL_BASE), 0x01); 
				if (ret < 0)
					i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "sleep: 0x01", __func__);
			}
#else
			ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x01, CONTROL_BASE), 0x01); 
			if (ret < 0)
				i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "sleep: 0x01", __func__);
#endif
		} else {
			if (ts->psensor_status > 0 ) {
				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x01, CONTROL_BASE), 0x02); 
				if (ret < 0)
					i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "sleep: 0x02", __func__);
			} else {
				ret = i2c_syn_write_byte_data(ts->client,
					get_address_base(ts, 0x01, CONTROL_BASE), 0x01); 
				if (ret < 0)
					i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "sleep: 0x01", __func__);
			}
		}
	}
#if defined(CONFIG_SYNC_TOUCH_STATUS)
	switch_sensor_hub(ts, 1);
#endif
	return 0;
}

static int synaptics_ts_resume(struct device *dev)
{
	int ret, i;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	pr_info("[TP] %s: enter. FW:%d;%d;%x;%x\n",
	__func__, ts->package_id, ts->packrat_number, syn_panel_version, ts->config_version);

#if defined(CONFIG_SYNC_TOUCH_STATUS)
	switch_sensor_hub(ts, 0);
#endif

	if (ts->power) {
		ts->power(1);
		msleep(100);
#ifdef SYN_CABLE_CONTROL
		if (ts->cable_support) {
			if (usb_get_connect_type())
				cable_tp_status_handler_func(1);
			pr_info("%s: ts->cable_config: %x\n", __func__, ts->cable_config);
		}
#endif
	} else {
		ret = i2c_syn_write_byte_data(ts->client,
			get_address_base(ts, 0x01, CONTROL_BASE), 0x00); 
		if (ret < 0)
			i2c_syn_error_handler(ts, ts->i2c_err_handler_en, "wake up", __func__);
	}

	if (ts->htc_event == SYN_AND_REPORT_TYPE_A) {
		if (ts->support_htc_event) {
			input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
			input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
			input_sync(ts->input_dev);
		}
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_sync(ts->input_dev);
	} else if (ts->htc_event == SYN_AND_REPORT_TYPE_B) {
		if (ts->package_id >= 3500) {
			for (i = 0; i < ts->finger_support; i++) {
				input_mt_slot(ts->input_dev, i);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
				input_sync(ts->input_dev);
				
			}
			ts->tap_suppression = 0;
			ts->finger_pressed = 0;
			vk_press = 0;
		}
	} else if (ts->htc_event == SYN_AND_REPORT_TYPE_HTC) {
		input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
		input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
	}
	if (ts->packrat_number < SYNAPTICS_FW_NOCAL_PACKRAT) {
		if (ts->psensor_detection) {
			if (ts->psensor_status == 0) {
				ts->psensor_resume_enable = 1;
				pr_info("[TP] %s: Enable P-sensor by Touch\n", __func__);
			} else if (ts->psensor_phone_enable == 0) {
				if (ts->psensor_status != 3)
					ts->psensor_resume_enable = 2;

				ts->psensor_phone_enable = 1;
			}
		}
	}

	if (ts->use_irq) {
		if (!ts->irq_enabled) {
			enable_irq(ts->client->irq);
			ts->irq_enabled = 1;
		}
	}
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	ts->suspended = false;
	return 0;
}

#if defined(CONFIG_SYNC_TOUCH_STATUS)
static void switch_sensor_hub(struct synaptics_ts_data* ts, int mode)
{
	if(ts->gpio_i2c >= 0) {
		switch(mode)
		{
		case 0:
			touch_status(0);
			gpio_direction_output(ts->gpio_i2c, 0);
			ts->i2c_to_mcu = 0;
			printk("[TP][SensorHub] Switch touch i2c to CPU\n");
			i2c_syn_reset_handler(ts, ts->i2c_err_handler_en, "resume from MCU", __func__); 
			break;
		case 1:
			gpio_direction_output(ts->gpio_i2c, 1);
			ts->i2c_to_mcu = 1;
			printk("[TP][SensorHub] Switch touch i2c to MCU\n");
			touch_status(1);
			break;
		}
	}
}
#endif

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct synaptics_ts_data *ts=
		container_of(self, struct synaptics_ts_data, fb_notif);

	pr_info("[TP] %s\n", __func__);
	if (evdata && evdata->data && event == FB_EVENT_BLANK && ts &&
			ts->client) {
		blank = evdata->data;
		switch (*blank) {
		case FB_BLANK_UNBLANK:
			synaptics_ts_resume(&ts->client->dev);
			break;
		case FB_BLANK_POWERDOWN:
		case FB_BLANK_HSYNC_SUSPEND:
		case FB_BLANK_VSYNC_SUSPEND:
		case FB_BLANK_NORMAL:
            if (ts->syn_cover_wq)
			{
				printk("[TP] Suspend , Flush cover workqueue");
				flush_workqueue( ts->syn_cover_wq );
			}
			synaptics_ts_suspend(&ts->client->dev);
			break;
		}
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_suspend(ts->client->dev);
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_resume(ts->client->dev);
}
#endif

static const struct dev_pm_ops synaptics_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend = synaptics_ts_suspend,
	.resume  = synaptics_ts_resume,
#else
	.suspend = synaptics_ts_suspend,
#endif
};

static const struct i2c_device_id synaptics_ts_id[] = {
	{ SYNAPTICS_3200_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, synaptics_ts_id);

#ifdef CONFIG_OF
static struct of_device_id synaptics_match_table[] = {
	{.compatible = "synaptics,3xxx" },
	{},
};
#else
#define synaptics_match_table NULL
#endif

static struct i2c_driver synaptics_ts_driver = {
	.driver = {
		.name           = "synaptics_3xxx",
		.owner          = THIS_MODULE,
		.of_match_table = synaptics_match_table,
#ifdef CONFIG_PM
		.pm             = &synaptics_pm_ops,
#endif
	},
	.probe    = synaptics_ts_probe,
	.remove   = synaptics_ts_remove,
	.id_table = synaptics_ts_id,
};

static void __devinit synaptics_ts_init_async(void *unused, async_cookie_t cookie)
{
	i2c_add_driver(&synaptics_ts_driver);
}

static int __devinit synaptics_ts_init(void)
{
	async_schedule(synaptics_ts_init_async, NULL);
	return 0;
}
static void __exit synaptics_ts_exit(void)
{
	i2c_del_driver(&synaptics_ts_driver);
}
module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");
