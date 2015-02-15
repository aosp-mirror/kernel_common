/* driver/leds/leds-lp5521_htc.c
 *
 * Copyright (C) 2010 HTC Corporation.
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

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/android_alarm.h>
#include <linux/earlysuspend.h>
#include <linux/leds.h>
#include <linux/leds-lp5521_htc.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>

#define LP5521_MAX_LEDS			9	
#define LED_DEBUG				1
#if LED_DEBUG
#define D(x...) printk(KERN_DEBUG "[LED]" x)
#define I(x...) printk(KERN_INFO "[LED]" x)
#else
#define D(x...)
#define I(x...)
#endif

static int led_rw_delay, chip_enable;
static int current_time;
static struct i2c_client *private_lp5521_client;
static struct mutex	led_mutex;
static struct workqueue_struct *g_led_work_queue;
static uint32_t ModeRGB;
static struct led_i2c_platform_data *plat_data;
#define Mode_Mask (0xff << 24)
#define Red_Mask (0xff << 16)
#define Green_Mask (0xff << 8)
#define Blue_Mask 0xff

static int gCurrent_param = 95;
module_param(gCurrent_param, int, 0600);


struct lp5521_led {
	int			id;
	u8			chan_nr;
	u8			led_current;
	u8			max_current;
	struct led_classdev	cdev;
	struct mutex 		led_data_mutex;
	struct alarm 		led_alarm;
	struct work_struct 	led_work;
	struct work_struct 	led_work_multicolor;
	uint8_t 		Mode;
	uint8_t			Red;
	uint8_t 		Green;
	uint8_t 		Blue;
	struct delayed_work	blink_delayed_work;
	struct wake_lock        led_wake_lock;
};

struct lp5521_chip {
	struct led_i2c_platform_data *pdata;
	struct mutex		led_i2c_rw_mutex; 
	struct i2c_client	*client;
	struct lp5521_led	leds[LP5521_MAX_LEDS];
	struct early_suspend early_suspend_led;
};
static int lp5521_parse_dt(struct device *, struct led_i2c_platform_data *);

static char *hex2string(uint8_t *data, int len)
{
	static char buf[LED_I2C_WRITE_BLOCK_SIZE*4];
	int i;

	i = LED_I2C_WRITE_BLOCK_SIZE -1;
	if (len > i)
		len = i;

	for (i = 0; i < len; i++)
		sprintf(buf + i * 4, "[%02X]", data[i]);

	return buf;
}

static int i2c_write_block(struct i2c_client *client, uint8_t addr,
		uint8_t *data, int length)
{
	int retry;
	uint8_t buf[LED_I2C_WRITE_BLOCK_SIZE];
	int i;
	struct lp5521_chip *cdata;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	dev_dbg(&client->dev, "W [%02X] = %s\n",
			addr, hex2string(data, length));

	cdata = i2c_get_clientdata(client);
	if (length + 1 > LED_I2C_WRITE_BLOCK_SIZE) {
		dev_err(&client->dev, "[LED] i2c_write_block length too long\n");
		return -E2BIG;
	}

	buf[0] = addr;
	for (i = 0; i < length; i++)
		buf[i+1] = data[i];

	mutex_lock(&cdata->led_i2c_rw_mutex);
	msleep(1);
	for (retry = 0; retry < I2C_WRITE_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		msleep(led_rw_delay);
	}
	if (retry >= I2C_WRITE_RETRY_TIMES) {
		dev_err(&client->dev, "[LED] i2c_write_block retry over %d times\n",
				I2C_WRITE_RETRY_TIMES);
		mutex_unlock(&cdata->led_i2c_rw_mutex);
		return -EIO;
	}
	mutex_unlock(&cdata->led_i2c_rw_mutex);

	return 0;
}


static int I2C_RxData_2(char *rxData, int length)
{
	uint8_t loop_i;

	struct i2c_msg msgs[] = {
		{
			.addr = private_lp5521_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = private_lp5521_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (loop_i = 0; loop_i < I2C_WRITE_RETRY_TIMES; loop_i++) {
		if (i2c_transfer(private_lp5521_client->adapter, msgs, 2) > 0)
			break;
		msleep(10);
	}

	if (loop_i >= I2C_WRITE_RETRY_TIMES) {
		printk(KERN_ERR "[LED] %s retry over %d times\n",
				__func__, I2C_WRITE_RETRY_TIMES);
		return -EIO;
	}

	return 0;
}

static int i2c_read_block(struct i2c_client *client,
		uint8_t cmd, uint8_t *pdata, int length)
{
	char buffer[3] = {0};
	int ret = 0, i;

	if (pdata == NULL)
		return -EFAULT;

	if (length > 2) {
		pr_err("[LED]%s: length %d> 2: \n", __func__, length);
		return ret;
	}
	buffer[0] = cmd;
	ret = I2C_RxData_2(buffer, length);
	if (ret < 0) {
		pr_err("[LED]%s: I2C_RxData fail \n", __func__);
		return ret;
	}

	for (i = 0; i < length; i++) {
		*(pdata+i) = buffer[i];
	}
	return ret;
}

static void lp5521_led_enable(struct i2c_client *client, int blink_enable)
{
	int ret = 0;
	uint8_t data;

	char data1[1] = {0};

	I(" %s +++\n" , __func__);

	
	if (plat_data->ena_gpio) {
		ret = gpio_direction_output(plat_data->ena_gpio, 1);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output high failed %d\n", __func__, ret);
			gpio_free(plat_data->ena_gpio);
		}
	} 

	msleep(1);
#if 1
	if (blink_enable) {
		
		data = 0xFF;
		ret = i2c_write_block(client, 0x0d, &data, 1);
		msleep(20);
		ret = i2c_read_block(client, 0x05, data1, 1);
		if (data1[0] != 0xaf) {
			I(" %s reset not ready %x\n" , __func__, data1[0]);
		}
	}
#endif
	chip_enable = 1;
	mutex_lock(&led_mutex);
	
	data = 0x40;
	ret = i2c_write_block(client, ENABLE_REGISTER, &data, 1);
	udelay(550);
	
	data = 0x29;
	ret = i2c_write_block(client, 0x08, &data, 1);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}
static void lp5521_red_long_blink(struct i2c_client *client)
{
	uint8_t data = 0x00;
	int ret;

	I(" %s +++\n" , __func__);
	mutex_lock(&led_mutex);
	data = 0x10;
	ret = i2c_write_block(client, OPRATION_REGISTER, &data, 1);
	udelay(200);

	
	data = 0x40;
	ret = i2c_write_block(client, 0x10, &data, 1);
	data = 0xc8;
	ret = i2c_write_block(client, 0x11, &data, 1);
	
	data = 0x7f;
	ret = i2c_write_block(client, 0x12, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x13, &data, 1);
	
	data = 0x40;
	ret = i2c_write_block(client, 0x14, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x15, &data, 1);
	
	data = 0x7f;
	ret = i2c_write_block(client, 0x16, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x17, &data, 1);
	
	data = 0x00;
	ret = i2c_write_block(client, 0x18, &data, 1);
	ret = i2c_write_block(client, 0x19, &data, 1);

	

	data = 0x20;
	ret = i2c_write_block(client, OPRATION_REGISTER, &data, 1);
	udelay(200);
	data = 0x60;
	ret = i2c_write_block(client, ENABLE_REGISTER, &data, 1);
	udelay(550);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

static void lp5521_color_blink(struct i2c_client *client, uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t data = 0x00;
	int ret;
	uint8_t mode = 0x00;
	I(" %s +++ red:%d, green:%d, blue:%d\n" , __func__, red, green, blue);
	mutex_lock(&led_mutex);

	if (red)
		mode |= (3 << 4);
	if (green)
		mode |= (3 << 2);
	if (blue)
		mode |= 3;
	data = mode & 0x15;
	ret = i2c_write_block(client, OPRATION_REGISTER, &data, 1);
	udelay(200);
	data = (u8)0;
	ret = i2c_write_block(client, 0x09, &data, 1);
	udelay(200);
	ret = i2c_write_block(client, 0x0a, &data, 1);
	udelay(200);
	ret = i2c_write_block(client, 0x0b, &data, 1);

	if (red) {
		
		data = 0x7f;
		ret = i2c_write_block(client, 0x10, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, 0x11, &data, 1);
		
		data = 0x40;
		ret = i2c_write_block(client, 0x12, &data, 1);

		ret = i2c_write_block(client, 0x13, &red, 1);
		
		data = 0x44;
		ret = i2c_write_block(client, 0x14, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, 0x15, &data, 1);
		
		data = 0x40;
		ret = i2c_write_block(client, 0x16, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, 0x17, &data, 1);
		
		data = 0x7c;
		ret = i2c_write_block(client, 0x18, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, 0x19, &data, 1);
		
		data = 0x00;
		ret = i2c_write_block(client, 0x1a, &data, 1);
		ret = i2c_write_block(client, 0x1b, &data, 1);
	}
	if (green) {
		
		data = 0x7f;
		ret = i2c_write_block(client, 0x30, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, 0x31, &data, 1);
		
		data = 0x40;
		ret = i2c_write_block(client, 0x32, &data, 1);

		ret = i2c_write_block(client, 0x33, &green, 1);
		
		data = 0x44;
		ret = i2c_write_block(client, 0x34, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, 0x35, &data, 1);
		
		data = 0x40;
		ret = i2c_write_block(client, 0x36, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, 0x37, &data, 1);
		
		data = 0x7c;
		ret = i2c_write_block(client, 0x38, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, 0x39, &data, 1);
		
		data = 0x00;
		ret = i2c_write_block(client, 0x3a, &data, 1);
		ret = i2c_write_block(client, 0x3b, &data, 1);
	}
	if (blue) {
		
		data = 0x7f;
		ret = i2c_write_block(client, 0x50, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, 0x51, &data, 1);
		
		data = 0x40;
		ret = i2c_write_block(client, 0x52, &data, 1);

		ret = i2c_write_block(client, 0x53, &blue, 1);
		
		data = 0x44;
		ret = i2c_write_block(client, 0x54, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, 0x55, &data, 1);
		
		data = 0x40;
		ret = i2c_write_block(client, 0x56, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, 0x57, &data, 1);
		
		data = 0x7c;
		ret = i2c_write_block(client, 0x58, &data, 1);
		data = 0x00;
		ret = i2c_write_block(client, 0x59, &data, 1);
		
		data = 0x00;
		ret = i2c_write_block(client, 0x5a, &data, 1);
		ret = i2c_write_block(client, 0x5b, &data, 1);
	}

	
	data = mode & 0x2a;
	ret = i2c_write_block(client, OPRATION_REGISTER, &data, 1);
	udelay(200);
	data = (mode & 0x2a)|0x40;
	ret = i2c_write_block(client, ENABLE_REGISTER, &data, 1);
	udelay(550);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}

static void lp5521_dual_color_blink(struct i2c_client *client)
{
	uint8_t data = 0x00;
	int ret;

	I(" %s +++\n" , __func__);
	mutex_lock(&led_mutex);
	data = 0x14;
	ret = i2c_write_block(client, OPRATION_REGISTER, &data, 1);
	udelay(200);


	
	data = 0x40;
	ret = i2c_write_block(client, 0x10, &data, 1);
	data = 0xc8;
	ret = i2c_write_block(client, 0x11, &data, 1);
	
	data = 0x44;
	ret = i2c_write_block(client, 0x12, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x13, &data, 1);
	
	data = 0x40;
	ret = i2c_write_block(client, 0x14, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x15, &data, 1);
	
	data = 0x50;
	ret = i2c_write_block(client, 0x16, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x17, &data, 1);
	
	data = 0xe1;
	ret = i2c_write_block(client, 0x18, &data, 1);
	data = 0x04;
	ret = i2c_write_block(client, 0x19, &data, 1);
	
	data = 0x00;
	ret = i2c_write_block(client, 0x1a, &data, 1);
	ret = i2c_write_block(client, 0x1b, &data, 1);
	udelay(550);

	
	data = 0xe0;
	ret = i2c_write_block(client, 0x30, &data, 1);
	data = 0x80;
	ret = i2c_write_block(client, 0x31, &data, 1);
	udelay(550);
	
	data = 0x40;
	ret = i2c_write_block(client, 0x32, &data, 1);
	data = 0xc8;
	ret = i2c_write_block(client, 0x33, &data, 1);
	
	data = 0x44;
	ret = i2c_write_block(client, 0x34, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x35, &data, 1);
	
	data = 0x40;
	ret = i2c_write_block(client, 0x36, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x37, &data, 1);
	
	data = 0x7f;
	ret = i2c_write_block(client, 0x38, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x39, &data, 1);
	
	data = 0x68;
	ret = i2c_write_block(client, 0x3a, &data, 1);
	data = 0x00;
	ret = i2c_write_block(client, 0x3b, &data, 1);
	
	data = 0xe0;
	ret = i2c_write_block(client, 0x3c, &data, 1);
	data = 0x02;
	ret = i2c_write_block(client, 0x3d, &data, 1);
	
	data = 0x00;
	ret = i2c_write_block(client, 0x3e, &data, 1);
	ret = i2c_write_block(client, 0x3f, &data, 1);
	udelay(550);

	

	data = 0x28;
	ret = i2c_write_block(client, OPRATION_REGISTER, &data, 1);
	udelay(200);

	data = 0x68;
	ret = i2c_write_block(client, ENABLE_REGISTER, &data, 1);
	udelay(550);
	mutex_unlock(&led_mutex);
	I(" %s ---\n" , __func__);
}
static void lp5521_led_off(struct i2c_client *client)
{
	uint8_t data = 0x00;
	int ret;
	char data1[1] = {0};

	I(" %s +++\n" , __func__);
	if (!chip_enable) {
		I(" %s return, chip already disable\n" , __func__);
		return;
	}
	ret = i2c_read_block(client, 0x00, data1, 1);
	if (!data1[0]) {
		I(" %s return, chip already disable\n" , __func__);
		return;
	}

	mutex_lock(&led_mutex);
	
	data = 0x00;
	ret = i2c_write_block(client, B_PWM_CONTROL, &data, 1);
	ret = i2c_write_block(client, G_PWM_CONTROL, &data, 1);
	ret = i2c_write_block(client, R_PWM_CONTROL, &data, 1);
	ret = i2c_write_block(client, OPRATION_REGISTER, &data, 1);
	ret = i2c_write_block(client, ENABLE_REGISTER, &data, 1);
	mutex_unlock(&led_mutex);
	if (plat_data->ena_gpio) {
		ret = gpio_direction_output(plat_data->ena_gpio, 0);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output high failed %d\n", __func__, ret);
			gpio_free(plat_data->ena_gpio);
		}
	} 
	chip_enable = 0;
	I(" %s ---\n" , __func__);
}


static void led_work_func(struct work_struct *work)
{
	struct i2c_client *client = private_lp5521_client;
	struct lp5521_led *ldata;

	I(" %s +++\n" , __func__);
	ldata = container_of(work, struct lp5521_led, led_work);
	lp5521_led_off(client);
	I(" %s ---\n" , __func__);
}

static void multicolor_work_func(struct work_struct *work)
{
	struct i2c_client *client = private_lp5521_client;
	struct lp5521_led *ldata;
	int ret;
	uint8_t data = 0x00;
        char data1[1] = {0};
        int i;

	ldata = container_of(work, struct lp5521_led, led_work_multicolor);
	I(" %s , Mode = %x\n" , __func__, ldata->Mode);

	if (ldata->Mode > 1 && ldata->Mode <= 5)
		lp5521_led_enable(client, 1);
	else if (ldata->Mode == 1)
		lp5521_led_enable(client, 0);
	if (ldata->Red) {
		data = (u8)gCurrent_param;
		ret = i2c_write_block(client, 0x05, &data, 1);
	} else {
		data = (u8)0;
		ret = i2c_write_block(client, 0x05, &data, 1);
	}
	if (ldata->Green) {
		data = (u8)gCurrent_param;
		ret = i2c_write_block(client, 0x06, &data, 1);
	} else {
		data = (u8)0;
		ret = i2c_write_block(client, 0x06, &data, 1);
	}
	if (ldata->Blue) {
		data = (u8)gCurrent_param;
		ret = i2c_write_block(client, 0x07, &data, 1);
	} else {
		data = (u8)0;
		ret = i2c_write_block(client, 0x07, &data, 1);
	}

	if (ldata->Mode == 0) {
		lp5521_led_off(client);
	} else if (ldata->Mode == 1) {  
		mutex_lock(&led_mutex);
		ret = i2c_write_block(client, R_PWM_CONTROL, &ldata->Red, 1);
		ret = i2c_write_block(client, G_PWM_CONTROL, &ldata->Green, 1);
		ret = i2c_write_block(client, B_PWM_CONTROL, &ldata->Blue, 1);
		data = 0x3f;
		ret = i2c_write_block(client, OPRATION_REGISTER, &data, 1);
		udelay(200);
		data = 0x40;
		ret = i2c_write_block(client, ENABLE_REGISTER, &data, 1);
		udelay(500);
		mutex_unlock(&led_mutex);
	} else if (ldata->Mode == 2) { 
		lp5521_color_blink(client, ldata->Red, ldata->Green, ldata->Blue);
	} else if (ldata->Mode == 3) { 
		msleep(1000);
		lp5521_color_blink(client, ldata->Red, ldata->Green, ldata->Blue);
	} else if (ldata->Mode == 4 && ldata->Red && !ldata->Green && !ldata->Blue) { 
		lp5521_red_long_blink(client);
	} else if (ldata->Mode ==5 && ldata->Red && ldata->Green && !ldata->Blue) { 
		lp5521_dual_color_blink(client);
	} else {
		for (i = 0; i <= 0x6f; i++) {
			ret = i2c_read_block(client, i, data1, 1);
			I(" %s i2c(%x) = 0x%x\n", __func__, i, data1[0]);
		}
	}
}

static void led_alarm_handler(struct alarm *alarm)
{
	struct lp5521_led *ldata;

	I(" %s +++\n" , __func__);
	ldata = container_of(alarm, struct lp5521_led, led_alarm);
	queue_work(g_led_work_queue, &ldata->led_work);
	I(" %s ---\n" , __func__);
}
static void led_blink_do_work(struct work_struct *work)
{
	struct i2c_client *client = private_lp5521_client;
	struct lp5521_led *ldata;

	I(" %s +++\n" , __func__);
	ldata = container_of(work, struct lp5521_led, blink_delayed_work.work);
	lp5521_color_blink(client, ldata->Red, ldata->Green, ldata->Blue);
	I(" %s ---\n" , __func__);
}

static ssize_t lp5521_led_off_timer_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", current_time);;
}

static ssize_t lp5521_led_off_timer_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct lp5521_led *ldata;
	int min, sec;
	uint16_t off_timer;
	ktime_t interval;
	ktime_t next_alarm;

	min = -1;
	sec = -1;
	sscanf(buf, "%d %d", &min, &sec);
	I(" %s , min = %d, sec = %d\n" , __func__, min, sec);
	if (min < 0 || min > 255)
		return -EINVAL;
	if (sec < 0 || sec > 255)
		return -EINVAL;

	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct lp5521_led, cdev);

	off_timer = min * 60 + sec;

	alarm_cancel(&ldata->led_alarm);
	cancel_work_sync(&ldata->led_work);
	if (off_timer) {
		interval = ktime_set(off_timer, 0);
		next_alarm = ktime_add(alarm_get_elapsed_realtime(), interval);
		alarm_start_range(&ldata->led_alarm, next_alarm, next_alarm);
	}

	return count;
}

static DEVICE_ATTR(off_timer, 0644, lp5521_led_off_timer_show,
		lp5521_led_off_timer_store);

static ssize_t lp5521_led_multi_color_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%x\n", ModeRGB);
}

static ssize_t lp5521_led_multi_color_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct led_classdev *led_cdev;
	struct lp5521_led *ldata;
	uint32_t val;
	sscanf(buf, "%x", &val);

	if (val < 0 || val > 0xFFFFFFFF)
		return -EINVAL;
	led_cdev = (struct led_classdev *)dev_get_drvdata(dev);
	ldata = container_of(led_cdev, struct lp5521_led, cdev);
	wake_lock_timeout(&(ldata->led_wake_lock), 2*HZ);
	ldata->Mode = (val & Mode_Mask) >> 24;
	ldata->Red = (val & Red_Mask) >> 16;
	ldata->Green = (val & Green_Mask) >> 8;
	ldata->Blue = val & Blue_Mask;
	ModeRGB = val;
	I(" %s , ModeRGB = %x\n" , __func__, val);
	queue_work(g_led_work_queue, &ldata->led_work_multicolor);
	return count;
}

static DEVICE_ATTR(ModeRGB, 0644, lp5521_led_multi_color_show,
		lp5521_led_multi_color_store);

static ssize_t lp5521_led_i2c_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	char data[1] = {0};
	int i;
	struct i2c_client *client = private_lp5521_client;

	for (i = 0; i <= 0x6f; i++) {
		ret = i2c_read_block(client, i, data, 1);
		I(" %s i2c(%x) = 0x%x\n", __func__, i, data[0]);
	}
	return ret;
}

static ssize_t lp5521_led_i2c_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = private_lp5521_client;
	int i, ret;
	char *token[10];
	unsigned long ul_reg, ul_data = 0;
	uint8_t reg = 0, data;
	char value[1] = {0};
	struct led_i2c_platform_data *pdata;
	pdata = client->dev.platform_data;

	for (i = 0; i < 2; i++) {
		token[i] = strsep((char **)&buf, " ");
		D("%s: token[%d] = %s\n", __func__, i, token[i]);
	}
	ret = strict_strtoul(token[0], 16, &ul_reg);
	ret = strict_strtoul(token[1], 16, &ul_data);

	reg = ul_reg;
	data = ul_data;

	if (reg < 0x6F) {
		ret = i2c_write_block(client, reg, &data, 1);
		ret = i2c_read_block(client, reg, value, 1);
		I(" %s , ret = %d, Set REG=0x%x, data=0x%x\n" , __func__, ret, reg, data);
		ret = i2c_read_block(client, reg, value, 1);
		I(" %s , ret = %d, Get REG=0x%x, data=0x%x\n" , __func__, ret, reg, value[0]);
	}
	if (reg == 0x99) {
		if (data == 1) {
			I("%s , pull up enable pin\n", __func__);
			if (pdata->ena_gpio) {
				ret = gpio_direction_output(pdata->ena_gpio, 1);
				if (ret < 0) {
					pr_err("[LED] %s: gpio_direction_output high failed %d\n", __func__, ret);
					gpio_free(pdata->ena_gpio);
				}
			}
		} else if (data == 0) {
			I("%s , pull down enable pin\n", __func__);
			if (pdata->ena_gpio) {
				ret = gpio_direction_output(pdata->ena_gpio, 1);
				if (ret < 0) {
					pr_err("[LED] %s: gpio_direction_output high failed %d\n", __func__, ret);
					gpio_free(pdata->ena_gpio);
				}
			}
		}
	}
	return count;
}

static DEVICE_ATTR(i2c, 0644, lp5521_led_i2c_show, lp5521_led_i2c_store);

static int lp5521_parse_dt(struct device *dev, struct led_i2c_platform_data *pdata)
{
	struct property *prop;
	struct device_node *dt = dev->of_node;
	prop = of_find_property(dt, "lp5521,lp5521_en", NULL);
	if (prop) {
		pdata->ena_gpio = of_get_named_gpio(dt, "lp5521,lp5521_en", 0);
	}
	prop = of_find_property(dt, "lp5521,num_leds", NULL);
	if (prop) {
		of_property_read_u32(dt, "lp5521,num_leds", &pdata->num_leds);
	}
	prop = of_find_property(dt, "lp5521,current_param", NULL);
	if (prop) {
		of_property_read_u32(dt, "lp5521,current_param", &gCurrent_param);
     	}
	return 0;
}

static int lp5521_led_probe(struct i2c_client *client
		, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct lp5521_chip		*cdata;
	struct led_i2c_platform_data *pdata;
	int ret =0;
	int i;

	printk("[LED][PROBE] led driver probe +++\n");

	
	cdata = kzalloc(sizeof(struct lp5521_chip), GFP_KERNEL);
	if (!cdata) {
		ret = -ENOMEM;
		dev_err(&client->dev, "[LED][PROBE_ERR] failed on allocat cdata\n");
		goto err_cdata;
	}

	i2c_set_clientdata(client, cdata);
	cdata->client = client;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (pdata == NULL)
		ret = -ENOMEM;
	ret = lp5521_parse_dt(&client->dev, pdata);
	led_rw_delay = 5;
	
	if (pdata->ena_gpio) {
		ret = gpio_request(pdata->ena_gpio, "led_enable");
		if (ret < 0) {
			pr_err("[LED] %s: gpio_request failed %d\n", __func__, ret);
			return ret;
		}
		ret = gpio_direction_output(pdata->ena_gpio, 1);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(pdata->ena_gpio);
			return ret;
		}
	} 
	
	if (pdata->tri_gpio) {
		ret = gpio_request(pdata->tri_gpio, "led_trigger");
		if (ret < 0) {
			pr_err("[LED] %s: gpio_request failed %d\n", __func__, ret);
			return ret;
		}
		ret = gpio_direction_output(pdata->tri_gpio, 0);
		if (ret < 0) {
			pr_err("[LED] %s: gpio_direction_output failed %d\n", __func__, ret);
			gpio_free(pdata->tri_gpio);
			return ret;
		}
	}
	private_lp5521_client = client;
	g_led_work_queue = create_workqueue("led");
	if (!g_led_work_queue) {
		ret = -10;
		pr_err("[LED] %s: create workqueue fail %d\n", __func__, ret);
		goto err_create_work_queue;
	}
	for (i = 0; i < pdata->num_leds; i++) {
		cdata->leds[i].cdev.name = "indicator";
		ret = led_classdev_register(dev, &cdata->leds[i].cdev);
		if (ret < 0) {
			dev_err(dev, "couldn't register led[%d]\n", i);
			return ret;
		}
		ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_ModeRGB);
		if (ret < 0) {
			pr_err("%s: failed on create attr ModeRGB [%d]\n", __func__, i);
			goto err_register_attr_ModeRGB;
		}

		ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_off_timer);
		if (ret < 0) {
			pr_err("%s: failed on create attr off_timer [%d]\n", __func__, i);
			goto err_register_attr_off_timer;
		}
		ret = device_create_file(cdata->leds[i].cdev.dev, &dev_attr_i2c);
		if (ret < 0) {
			pr_err("%s: failed on create attr off_timer [%d]\n", __func__, i);
		}
		wake_lock_init(&cdata->leds[i].led_wake_lock, WAKE_LOCK_SUSPEND, "lp5521");
		INIT_WORK(&cdata->leds[i].led_work, led_work_func);
		INIT_WORK(&cdata->leds[i].led_work_multicolor, multicolor_work_func);
		INIT_DELAYED_WORK(&cdata->leds[i].blink_delayed_work, led_blink_do_work);
		alarm_init(&cdata->leds[i].led_alarm,
				ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
				led_alarm_handler);
	}
	mutex_init(&cdata->led_i2c_rw_mutex);
	mutex_init(&led_mutex);
	plat_data = pdata;
	
#if 0
	
	data = 0x00;
	ret = i2c_write_block(client, ENABLE_REGISTER, &data, 1);
	udelay(550);
	if (pdata->ena_gpio) {
		gpio_direction_output(pdata->ena_gpio, 0);
	} 
#endif
	printk("[LED][PROBE] led driver probe ---\n");
	return 0;


err_register_attr_off_timer:
	kfree(cdata);
	for (i = 0; i < pdata->num_leds; i++) {
		device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_off_timer);
	}
err_register_attr_ModeRGB:
	for (i = 0; i < pdata->num_leds; i++) {
		if (!strcmp(cdata->leds[i].cdev.name, "multi_color"))
			device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_ModeRGB);
	}
err_create_work_queue:
	kfree(pdata);
err_cdata:
	return ret;
}

static int __devexit lp5521_led_remove(struct i2c_client *client)
{
	struct lp5521_chip *cdata;
	int i,ret;

	cdata = i2c_get_clientdata(client);
	cdata = kzalloc(sizeof(struct lp5521_chip), GFP_KERNEL);
	i2c_set_clientdata(client, cdata);
	cdata->client = client;

	ret = lp5521_parse_dt(&client->dev, plat_data);
	if (plat_data->ena_gpio) {
		gpio_direction_output(plat_data->ena_gpio, 0);
	} 
	for (i = 0; i < plat_data->num_leds; i++) {
		device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_off_timer);
		device_remove_file(cdata->leds[i].cdev.dev,&dev_attr_ModeRGB);
		led_classdev_unregister(&cdata->leds[i].cdev);
	}
	destroy_workqueue(g_led_work_queue);
	kfree(cdata);

	return 0;
}


static const struct i2c_device_id led_i2c_id[] = {
	{ "LP5521-LED", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, led_i2c_id);

static const struct of_device_id lp5521_mttable[] = {
	{ .compatible = "LP5521-LED"},
	{ },
};

static struct i2c_driver led_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "LP5521-LED",
		.of_match_table = lp5521_mttable,
	},
	.id_table = led_i2c_id,
	.probe = lp5521_led_probe,
	.remove = __devexit_p(lp5521_led_remove),
};
module_i2c_driver(led_i2c_driver);
MODULE_AUTHOR("<ShihHao_Shiung@htc.com>, <Dirk_Chang@htc.com>");
MODULE_DESCRIPTION("LP5521 LED driver");

