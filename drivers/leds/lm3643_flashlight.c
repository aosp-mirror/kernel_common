/* drivers/i2c/chips/lm3643_flashlight.c
 *
 * Copyright (C) 2008-2009 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <mach/msm_iomap.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/htc_flashlight.h>
#include <linux/module.h>
#include <mach/socinfo.h>
#include <linux/reboot.h>



#define FLT_DBG_LOG(fmt, ...) \
		printk(KERN_DEBUG "[FLT_FR]LM3643 " fmt, ##__VA_ARGS__)
#define FLT_INFO_LOG(fmt, ...) \
		printk(KERN_INFO "[FLT_FR]LM3643 " fmt, ##__VA_ARGS__)
#define FLT_ERR_LOG(fmt, ...) \
		printk(KERN_ERR "[FLT_FR][ERR]LM3643 " fmt, ##__VA_ARGS__)

#define LM3643_RETRY_COUNT 10

struct lm3643_data {
	struct led_classdev 		fl_lcdev;
	struct early_suspend		fl_early_suspend;
	enum flashlight_mode_flags 	mode_status;
	uint32_t			flash_sw_timeout;
	struct mutex 			lm3643_data_mutex;
	uint32_t			hwen;
	uint32_t			strobe;
	uint32_t			torch;
	uint32_t			reset;
	uint8_t 			led_count;
	uint8_t 			mode_pin_suspend_state_low;
	uint8_t				enable_FLT_1500mA;
	uint8_t				disable_tx_mask;
	uint32_t			power_save;
	uint32_t			power_save_2;
	struct lm3643_led_data	*led_array;
};

static struct i2c_client *this_client;
static struct lm3643_data *this_lm3643;
struct delayed_work lm3643_delayed_work;
static struct workqueue_struct *lm3643_work_queue;
static struct mutex lm3643_mutex;

static int switch_state = 1;
static int support_dual_flashlight = 1; 
static int vte_in_use = 0;

static int regaddr = 0x00;
static int regdata = 0x00;
static int reg_buffered[256] = {0x00};

static int lm3643_i2c_command(uint8_t, uint8_t);
static int flashlight_turn_off(void);

static int uncertain_support_dual_flashlight(void)
{
	int pid = of_machine_pid();
	int pcbid = of_machine_pcbid();
	FLT_INFO_LOG("pid=%d, pcbid=%d.\r\n", pid, pcbid);


	
	if ( pid == 271 || pid == 272 || pid == 280 || pid == 286 )
	{
		if ( pcbid>=4 || pcbid<0 ) return 1;
		else                       return 0;
	}

	
	if ( pid == 273 )
	{
		if ( pcbid>=4 || pcbid<0 ) return 1;
		else                       return 0;
	}

	
	if ( pid == 266 )
	{
		if ( pcbid>=3 || pcbid<0 ) return 1;
		else                       return 0;
	}

	
	if ( pid == 269 )
	{
		if ( pcbid>=1 || pcbid<0 ) return 1;
		else                       return 0;
	}

	
	if ( pid == 267 )
	{
		if ( pcbid>=3 || pcbid<0 ) return 1;
		else                       return 0;
	}

	
	if ( pid == 281 )
	{
		if ( pcbid>=1 || pcbid<0 ) return 1;
		else                       return 0;
	}

	
	return 1;
}

static ssize_t support_dual_flashlight_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return snprintf(buf, sizeof(support_dual_flashlight)+1, "%d\n", support_dual_flashlight);
}

static ssize_t support_dual_flashlight_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int input;
	input = simple_strtoul(buf, NULL, 10);

	if(input >= 0 && input < 2){
		support_dual_flashlight = input;
		FLT_INFO_LOG("%s: %d\n",__func__,support_dual_flashlight);
	}else
		FLT_INFO_LOG("%s: Input out of range\n",__func__);
	return size;
}

static DEVICE_ATTR(support_dual_flashlight, 0664,
		   support_dual_flashlight_show, support_dual_flashlight_store);

static ssize_t poweroff_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int input;
	input = simple_strtoul(buf, NULL, 10);
	FLT_INFO_LOG("%s\n", __func__);

	if(input == 1){
		flashlight_turn_off();
	}else
		FLT_INFO_LOG("%s: Input out of range\n",__func__);

	return size;
}

static DEVICE_ATTR(poweroff, 0220,
		   NULL, poweroff_store);

static ssize_t sw_timeout_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return snprintf(buf, sizeof(this_lm3643->flash_sw_timeout)+1, "%d\n", this_lm3643->flash_sw_timeout);
}
static ssize_t sw_timeout_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int input;
	input = simple_strtoul(buf, NULL, 10);

	if(input >= 0 && input < 1500){
		this_lm3643->flash_sw_timeout = input;
		FLT_INFO_LOG("%s: %d\n",__func__,this_lm3643->flash_sw_timeout);
	}else
		FLT_INFO_LOG("%s: Input out of range\n",__func__);
	return size;
}
static DEVICE_ATTR(sw_timeout, S_IRUGO | S_IWUSR, sw_timeout_show, sw_timeout_store);

static ssize_t regaddr_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return snprintf(buf, 6, "0x%02x\n", regaddr);
}
static ssize_t regaddr_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int input;
	input = simple_strtoul(buf, NULL, 16);

	if(input >= 0 && input < 256){
		regaddr = input;
		FLT_INFO_LOG("%s: %d\n",__func__,regaddr);
	}else
		FLT_INFO_LOG("%s: Input out of range\n",__func__);
	return size;
}
static DEVICE_ATTR(regaddr, S_IRUGO | S_IWUSR, regaddr_show, regaddr_store);

static ssize_t regdata_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return snprintf(buf, 6, "0x%02x\n", reg_buffered[regaddr]);
}
static ssize_t regdata_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int input;
	input = simple_strtoul(buf, NULL, 16);

	if(input >= 0 && input < 256){
		regdata = input;
		FLT_INFO_LOG("%s: %d\n",__func__,regdata);
		lm3643_i2c_command(regaddr, regdata);
	}else
		FLT_INFO_LOG("%s: Input out of range\n",__func__);

	return size;
}
static DEVICE_ATTR(regdata, S_IRUGO | S_IWUSR, regdata_show, regdata_store);

static ssize_t switch_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return snprintf(buf, sizeof("switch status:") + sizeof(switch_state) + 1, "switch status:%d\n", switch_state);
}

static ssize_t switch_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int switch_status;
	switch_status = -1;
	switch_status = simple_strtoul(buf, NULL, 10);

	if(switch_status >= 0 && switch_status < 2){
		switch_state = switch_status;
		FLT_INFO_LOG("%s: %d\n",__func__,switch_state);
	}else
		FLT_INFO_LOG("%s: Input out of range\n",__func__);
	return size;
}

static DEVICE_ATTR(function_switch, S_IRUGO | S_IWUSR, switch_show, switch_store);

static ssize_t max_current_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	if (this_lm3643->enable_FLT_1500mA)
		return snprintf(buf, 6, "1500\n");
	else
		return snprintf(buf, 5, "750\n");
}
static DEVICE_ATTR(max_current, S_IRUGO | S_IWUSR, max_current_show, NULL);
static ssize_t flash_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int val;
	val = simple_strtoul(buf, NULL, 10);

	if(val >= 0){
		FLT_INFO_LOG("%s: %d\n",__func__,val);
	}else
		FLT_INFO_LOG("%s: Input out of range\n",__func__);
	return size;
}
static DEVICE_ATTR(flash, S_IRUGO | S_IWUSR, NULL, flash_store);

static int LM3643_I2C_TxData(char *txData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < LM3643_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(this_client->adapter, msg, 1) > 0)
			break;

		mdelay(10);
	}

	if (loop_i >= LM3643_RETRY_COUNT) {
		FLT_ERR_LOG("%s retry over %d\n", __func__,
							LM3643_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int lm3643_i2c_command(uint8_t address, uint8_t data)
{
	uint8_t buffer[2];
	int ret;

	reg_buffered[address] = data;

	buffer[0] = address;
	buffer[1] = data;
	ret = LM3643_I2C_TxData(buffer, 2);
	if (ret < 0)
		FLT_ERR_LOG("%s error\n", __func__);
	return 0;
}

static int flashlight_turn_off(void)
{
	if (vte_in_use == 1)
		return 0;
	FLT_INFO_LOG("%s\n", __func__);
	gpio_set_value_cansleep(this_lm3643->strobe, 0);
	lm3643_i2c_command(0x01, 0x08);
	gpio_set_value_cansleep(this_lm3643->hwen, 0);
	FLT_INFO_LOG("%s %d\n", __func__,this_lm3643->mode_status);
	this_lm3643->mode_status = FL_MODE_OFF;
	return 0;
}

static int reboot_notify_sys(struct notifier_block *this,
			      unsigned long event,
			      void *unused)
{
	FLT_INFO_LOG("%s: %ld", __func__, event);
	switch (event) {
		case SYS_RESTART:
		case SYS_HALT:
		case SYS_POWER_OFF:
			flashlight_turn_off();
			return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block reboot_notifier = {
	.notifier_call  = reboot_notify_sys,
};

int lm3643_flashlight_flash(int led1, int led2)
{

	uint8_t current_hex = 0x0;
	FLT_INFO_LOG("%s \n", __func__);
	FLT_INFO_LOG("camera flash current %d+%d. ver: 0220\n", led1, led2);
	mutex_lock(&lm3643_mutex);

	if (led1 == 0 && led2== 0)
		flashlight_turn_off();
	else {
		gpio_set_value_cansleep(this_lm3643->hwen, 1);
		gpio_set_value_cansleep(this_lm3643->strobe, 0);
		gpio_set_value_cansleep(this_lm3643->torch, 0);
		lm3643_i2c_command(0x08, 0x1f);
		lm3643_i2c_command(0x01, 0x20);
		mdelay(10);
		if (led1 == 0)
			lm3643_i2c_command(0x01, 0x22); 
		else if (led2 ==0)
			lm3643_i2c_command(0x01, 0x21); 
		else
			lm3643_i2c_command(0x01, 0x23); 
		lm3643_i2c_command(0x02, 0x00);
		if (led1) {
			if (led1 > 1500)
				led1 =1500;
			current_hex = 100*led1/1172;
			lm3643_i2c_command(0x03, current_hex); 
			FLT_INFO_LOG("set led1 current to 0x%x.\r\n", current_hex);
		}
		if (led2) {
			if (led2 > 1500)
				led2 =1500;
			current_hex = 100*led2/1172;
			if (led1 == 0)
				lm3643_i2c_command(0x03, 0x0); 
			lm3643_i2c_command(0x04, current_hex); 
			FLT_INFO_LOG("set led2 current to 0x%x.\r\n", current_hex);
		}
		mdelay(10);
		gpio_set_value_cansleep(this_lm3643->strobe, 1);

		queue_delayed_work(lm3643_work_queue, &lm3643_delayed_work,
				   msecs_to_jiffies(this_lm3643->flash_sw_timeout));
	}
	mutex_unlock(&lm3643_mutex);
	return 0;
}

int lm3643_flashlight_torch (int led1, int led2)
{
	uint8_t current_hex = 0x0;
	FLT_INFO_LOG("%s \n", __func__);
	FLT_INFO_LOG("Torch current %d+%d. ver: 0220\n", led1, led2);
	mutex_lock(&lm3643_mutex);
	if (led1 == 0 && led2== 0)
		flashlight_turn_off();
	else {
		gpio_set_value_cansleep(this_lm3643->hwen, 1);
		gpio_set_value_cansleep(this_lm3643->strobe, 0);
		gpio_set_value_cansleep(this_lm3643->torch, 0);
		lm3643_i2c_command(0x02, 0x00);
		if (led1) {
			if (led1 > 186)
				led1 = 186;
			current_hex = 100*led1/146;
			lm3643_i2c_command(0x05, current_hex); 
			FLT_INFO_LOG("set led1 current to 0x%x.\r\n", current_hex);
		}
		if (led2) {
			if (led2 > 186)
				led2 = 186;
			current_hex = 100*led2/146;
			if (led1 == 0)
				lm3643_i2c_command(0x05, 0x0); 
			lm3643_i2c_command(0x06, current_hex); 
			FLT_INFO_LOG("set led2 current to 0x%x.\r\n", current_hex);
		}
		if (led1 == 0)
			lm3643_i2c_command(0x01, 0x0a); 
		else if (led2 ==0)
			lm3643_i2c_command(0x01, 0x9); 
		else
			lm3643_i2c_command(0x01, 0x0b); 
	}
	mutex_unlock(&lm3643_mutex);
	return 0;
}

static void fl_lcdev_brightness_set(struct led_classdev *led_cdev,
						enum led_brightness brightness)
{
	enum flashlight_mode_flags mode;

	vte_in_use = 1;
	if (brightness > 0 && brightness <= LED_HALF) {
		if (brightness == (LED_HALF - 3))
			mode = FL_MODE_TORCH_LEVEL_0;
		else if (brightness == (LED_HALF - 2))
			mode = FL_MODE_TORCH_LEVEL_1;
		else if (brightness == (LED_HALF - 1))
			mode = FL_MODE_TORCH_LEVEL_2;
		else if (brightness == 1 && this_lm3643->led_count ==2)
			mode = FL_MODE_TORCH_LED_A;
		else if (brightness == 2 && this_lm3643->led_count ==2)
			mode = FL_MODE_TORCH_LED_B;
		else
			mode = FL_MODE_TORCH;
	} else if (brightness > LED_HALF && brightness <= LED_FULL) {
		if (brightness == (LED_HALF + 1))
			mode = FL_MODE_PRE_FLASH; 
		else if (brightness == (LED_HALF + 3))
			mode = FL_MODE_FLASH_LEVEL1; 
		else if (brightness == (LED_HALF + 4))
			mode = FL_MODE_FLASH_LEVEL2; 
		else if (brightness == (LED_HALF + 5))
			mode = FL_MODE_FLASH_LEVEL3; 
		else if (brightness == (LED_HALF + 6))
			mode = FL_MODE_FLASH_LEVEL4; 
		else if (brightness == (LED_HALF + 7))
			mode = FL_MODE_FLASH_LEVEL5; 
		else if (brightness == (LED_HALF + 8))
			mode = FL_MODE_FLASH_LEVEL6; 
		else if (brightness == (LED_HALF + 9))
			mode = FL_MODE_FLASH_LEVEL7; 
		else
			mode = FL_MODE_FLASH; 
	} else {
		
		mode = FL_MODE_OFF;
		vte_in_use = 0;
	}
	switch (mode) {
		case FL_MODE_OFF:
			flashlight_turn_off();
		break;
		
		case FL_MODE_PRE_FLASH:
			lm3643_flashlight_flash(125,50);
		break;
		case FL_MODE_FLASH:
			lm3643_flashlight_flash(300,300);
		break;
		case FL_MODE_TORCH:
			lm3643_flashlight_torch(50,150);
		break;
		case FL_MODE_TORCH_LEVEL_1:
			lm3643_flashlight_torch(50,50);
		break;
		case FL_MODE_TORCH_LEVEL_2:
			lm3643_flashlight_torch(50,100);
		break;
		
		
		case FL_MODE_TORCH_LEVEL_0:
			lm3643_flashlight_torch(27,17);
		break;
		
		default:
		FLT_ERR_LOG("%s: unknown flash_light flags: %d\n",__func__, mode);
		break;
	}

	FLT_INFO_LOG("%s: mode: %d\n", __func__, mode);
	this_lm3643->mode_status = mode;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void flashlight_early_suspend(struct early_suspend *handler)
{
	FLT_INFO_LOG("%s\n", __func__);
	if (this_lm3643->mode_status)
		flashlight_turn_off();
	if (this_lm3643->power_save)
		gpio_set_value_cansleep(this_lm3643->power_save, 0);
	if (this_lm3643->power_save_2)
		gpio_set_value_cansleep(this_lm3643->power_save_2, 0);
}

static void flashlight_late_resume(struct early_suspend *handler)
{
}
#endif

static void flashlight_turn_off_work(struct work_struct *work)
{
	FLT_INFO_LOG("%s\n", __func__);
	flashlight_turn_off();
}

static int lm3643_parse_dt(struct device *dev, struct LM3643_flashlight_platform_data *pdata)
{
	struct property *prop;
	struct device_node *dt = dev->of_node;
	prop = of_find_property(dt, "lm3643,lm3643_hwen", NULL);
	if (prop) {
		pdata->lm3643_hwen= of_get_named_gpio(dt, "lm3643,lm3643_hwen", 0);
	}
	prop = of_find_property(dt, "lm3643,lm3643_strobe", NULL);
	if (prop) {
		pdata->lm3643_strobe= of_get_named_gpio(dt, "lm3643,lm3643_strobe", 0);
	}
	prop = of_find_property(dt, "lm3643,flash_duration_ms", NULL);
	if (prop) {
		of_property_read_u32(dt, "lm3643,flash_duration_ms", &pdata->flash_duration_ms);
	}
	prop = of_find_property(dt, "lm3643,enable_FLT_1500mA", NULL);
	if (prop) {
		of_property_read_u32(dt, "lm3643,enable_FLT_1500mA", &pdata->enable_FLT_1500mA);
	}
	prop = of_find_property(dt, "lm3643,led_count", NULL);
	if (prop) {
		of_property_read_u32(dt, "lm3643,led_count", &pdata->led_count);
	}
	prop = of_find_property(dt, "lm3643,disable_tx_mask", NULL);
	if (prop) {
		of_property_read_u32(dt, "lm3643,disable_tx_mask", &pdata->disable_tx_mask);
	}

	return 0;

}

enum led_status {
	OFF = 0,
	ON,
	BLINK,
};

enum led_id {
	LED_1   = 0,
	LED_2= 1,
};

struct lm3643_led_data {
	u8			num_leds;
	struct i2c_client	*client_dev;
	struct lm3643_data 	*lm3643;
	int status;
	struct led_classdev	cdev;
	int			max_current;
	int			id;
	u8			default_state;
	int                     torch_mode;
	struct mutex		lock;
	struct work_struct	work;
};

static int lm3643_get_common_configs(struct lm3643_led_data *led,
		struct device_node *node)
{
	int rc;
	const char *temp_string;

	led->cdev.default_trigger = "none";
	rc = of_property_read_string(node, "linux,default-trigger",
		&temp_string);
	if (!rc)
		led->cdev.default_trigger = temp_string;
	else if (rc != -EINVAL)
		return rc;

	led->default_state = LEDS_GPIO_DEFSTATE_OFF;
	rc = of_property_read_string(node, "htc,default-state",
		&temp_string);
	if (!rc) {
		if (!strcmp(temp_string, "keep"))
			led->default_state = LEDS_GPIO_DEFSTATE_KEEP;
		else if (!strcmp(temp_string, "on"))
			led->default_state = LEDS_GPIO_DEFSTATE_ON;
		else
			led->default_state = LEDS_GPIO_DEFSTATE_OFF;
	} else if (rc != -EINVAL)
		return rc;

	return 0;
}

static void __lm3643_led_work(struct lm3643_led_data *led,
				enum led_brightness value)
{
	mutex_lock(&led->lock);
	if ( led->torch_mode ) {
		switch (led->id) {
		case LED_1:
			lm3643_flashlight_torch(value, 0);
			break;
		case LED_2:
			lm3643_flashlight_torch(0, value);
			break;
		};
	} else {
		switch (led->id) {
		case LED_1:
			lm3643_flashlight_flash(value, 0);
			break;
		case LED_2:
			lm3643_flashlight_flash(0,  value);
			break;
		};
	}
	mutex_unlock(&led->lock);
}

static void lm3643_led_work(struct work_struct *work)
{
	struct lm3643_led_data *led = container_of(work,
					struct lm3643_led_data, work);

	__lm3643_led_work(led, led->cdev.brightness);

	return;
}

static void lm3643_led_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	struct lm3643_led_data *led;

	led = container_of(led_cdev, struct lm3643_led_data, cdev);
	if (value < LED_OFF || value > led->cdev.max_brightness) {
		printk(KERN_ERR "[FLT]"
				"Invalid brightness value\n");
		return;
	}
	led->cdev.brightness = value;
	schedule_work(&led->work);
}

static enum led_brightness lm3643_led_get(struct led_classdev *led_cdev)
{
	struct lm3643_led_data *led;

	led = container_of(led_cdev, struct lm3643_led_data, cdev);

	return led->cdev.brightness;
}


static int lm3643_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct lm3643_data *lm3643;
	struct LM3643_flashlight_platform_data *pdata;
	int i = 0, err = 0, ret = 0;

	struct lm3643_led_data *led, *led_array;
	struct device_node *node, *temp;
	int num_leds = 0, parsed_leds = 0;
	const char *led_label;
	int rc;

	FLT_INFO_LOG("%s +\n", __func__);

	pdata =  kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (pdata == NULL){
		err = -ENOMEM;
		return err;
	}
	err = lm3643_parse_dt(&client->dev, pdata);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto check_functionality_failed;
	}

	lm3643 = kzalloc(sizeof(struct lm3643_data), GFP_KERNEL);
	if (!lm3643) {
		FLT_ERR_LOG("%s: kzalloc fail !!!\n", __func__);
		kfree(pdata);
		return -ENOMEM;
	}

	i2c_set_clientdata(client, lm3643);
	this_client = client;

	INIT_DELAYED_WORK(&lm3643_delayed_work, flashlight_turn_off_work);
	lm3643_work_queue = create_singlethread_workqueue("lm3643_wq");
	if (!lm3643_work_queue)
		goto err_create_lm3643_work_queue;

	lm3643->fl_lcdev.name              = FLASHLIGHT_NAME_FRONT;
	lm3643->fl_lcdev.brightness_set    = fl_lcdev_brightness_set;
	lm3643->hwen                      = pdata->lm3643_hwen;
	lm3643->strobe                      = pdata->lm3643_strobe;
	lm3643->torch                      = pdata->lm3643_torch;
	lm3643->reset                      = pdata->lm3643_reset;
	lm3643->flash_sw_timeout           = pdata->flash_duration_ms;
	lm3643->led_count                  = (pdata->led_count) ? pdata->led_count : 1;
	lm3643->mode_pin_suspend_state_low = pdata->mode_pin_suspend_state_low;
	lm3643->enable_FLT_1500mA          = pdata->enable_FLT_1500mA;
	lm3643->disable_tx_mask            = pdata->disable_tx_mask;
	lm3643->power_save                 = pdata->power_save;
	lm3643->power_save_2               = pdata->power_save_2;
	htc_flashlight_flash		   = &lm3643_flashlight_flash;
	htc_flashlight_torch		   = &lm3643_flashlight_torch;

	if (lm3643->hwen) {
		ret = gpio_request(lm3643->hwen, "hwen");
		if (ret) {
			FLT_ERR_LOG("%s: unable to request gpio %d (%d)\n",
				__func__, lm3643->hwen, ret);
			return ret;
		}

		ret = gpio_direction_output(lm3643->hwen, 0);
		if (ret) {
			FLT_ERR_LOG("%s: Unable to set direction\n", __func__);
			return ret;
		}
	}
	if (lm3643->strobe) {
		ret = gpio_request(lm3643->strobe, "strobe");
		if (ret) {
			FLT_ERR_LOG("%s: unable to request gpio %d (%d)\n",
				__func__, lm3643->strobe, ret);
			return ret;
		}

		ret = gpio_direction_output(lm3643->strobe, 0);
		if (ret) {
			FLT_ERR_LOG("%s: Unable to set direction\n", __func__);
			return ret;
		}
	}
	if (lm3643->torch) {
		ret = gpio_request(lm3643->torch, "torch");
		if (ret) {
			FLT_ERR_LOG("%s: unable to request gpio %d (%d)\n",
				__func__, lm3643->torch, ret);
			return ret;
		}

		ret = gpio_direction_output(lm3643->torch, 0);
		if (ret) {
			FLT_ERR_LOG("%s: Unable to set direction\n", __func__);
			return ret;
		}
	}
	if (lm3643->flash_sw_timeout <= 0)
		lm3643->flash_sw_timeout = 400;

	node = client->dev.of_node;

	if (node == NULL)
		return -ENODEV;

	temp = NULL;
	while ((temp = of_get_next_child(node, temp)))
		num_leds++;

	if (!num_leds)
		return -ECHILD;

	led_array = devm_kzalloc(&client->dev,
		(sizeof(struct lm3643_led_data) * num_leds), GFP_KERNEL);
	if (!led_array) {
		dev_err(&client->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	lm3643->led_array = led_array;
	for_each_child_of_node(node, temp) {
		led = &led_array[parsed_leds];
		led->num_leds = num_leds;
		led->client_dev = client;
		led->lm3643 = lm3643;
		led->status = OFF;

		rc = of_property_read_string(temp, "label", &led_label);
		if (rc < 0) {
			FLT_ERR_LOG("Failure reading label, rc = %d\n", rc);
			goto fail_id_check;
		}

		rc = of_property_read_string(temp, "linux,name", &led->cdev.name);
		if (rc < 0) {
			FLT_ERR_LOG("Failure reading led name, rc = %d\n", rc);
			goto fail_id_check;
		}

		rc = of_property_read_u32(temp, "htc,max-current", &led->max_current);
		if (rc < 0) {
			FLT_ERR_LOG("Failure reading max_current, rc =  %d\n", rc);
			goto fail_id_check;
		}

		rc = of_property_read_u32(temp, "htc,id", &led->id);
		if (rc < 0) {
			FLT_ERR_LOG("Failure reading led id, rc =  %d\n", rc);
			goto fail_id_check;
		}

		rc = lm3643_get_common_configs(led, temp);
		if (rc) {
			FLT_ERR_LOG("Failure reading common led configuration," \
				" rc = %d\n", rc);
				goto fail_id_check;
		}

		led->cdev.brightness_set    = lm3643_led_set;
		led->cdev.brightness_get    = lm3643_led_get;

		if (strncmp(led_label, "flash", sizeof("flash")) == 0) {
			led->torch_mode = 0;
			if (rc < 0) {
				FLT_ERR_LOG("Unable to read flash config data\n");
				goto fail_id_check;
			}
		} else if (strncmp(led_label, "torch", sizeof("torch")) == 0) {
			led->torch_mode = 1;
			if (rc < 0) {
				FLT_ERR_LOG("Unable to read torch config data\n");
				goto fail_id_check;
			}
		} else {
			FLT_ERR_LOG("No LED matching label\n");
			rc = -EINVAL;
			goto fail_id_check;
		}

		mutex_init(&led->lock);
		INIT_WORK(&led->work, lm3643_led_work);

		led->cdev.max_brightness = led->max_current;

		rc = led_classdev_register(&client->dev, &led->cdev);
		if (rc) {
			FLT_ERR_LOG("unable to register led %d,rc=%d\n",
					led->id, rc);
			goto fail_id_check;
		}
		parsed_leds++;
	}

	mutex_init(&lm3643_mutex);
	err = led_classdev_register(&client->dev, &lm3643->fl_lcdev);
	if (err < 0) {
		FLT_ERR_LOG("%s: failed on led_classdev_register\n", __func__);
		goto platform_data_null;
	}

	this_lm3643 = lm3643;

	err = register_reboot_notifier(&reboot_notifier);
	if (err < 0) {
		FLT_ERR_LOG("%s: Register reboot notifier failed(err=%d)\n", __func__, err);
		goto platform_data_null;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	lm3643->fl_early_suspend.suspend = flashlight_early_suspend;
	lm3643->fl_early_suspend.resume  = flashlight_late_resume;
	register_early_suspend(&lm3643->fl_early_suspend);
#endif

	rc = of_property_read_u32(node, "htc,dualflash", &support_dual_flashlight);
	if (rc < 0)
		support_dual_flashlight = 0;
	else if (support_dual_flashlight == 2)
		
		support_dual_flashlight = uncertain_support_dual_flashlight();

	err = device_create_file(lm3643->fl_lcdev.dev, &dev_attr_support_dual_flashlight);
	if (err < 0) {
		FLT_ERR_LOG("%s, create support_dual_flashlight sysfs fail\n", __func__);
	}
	err = device_create_file(lm3643->fl_lcdev.dev, &dev_attr_poweroff);
	if (err < 0) {
		FLT_ERR_LOG("%s, create poweroff sysfs fail\n", __func__);
	}
	err = device_create_file(lm3643->fl_lcdev.dev, &dev_attr_sw_timeout);
	if (err < 0) {
		FLT_ERR_LOG("%s, create sw_timeout sysfs fail\n", __func__);
	}
	err = device_create_file(lm3643->fl_lcdev.dev, &dev_attr_regaddr);
	if (err < 0) {
		FLT_ERR_LOG("%s, create regaddr sysfs fail\n", __func__);
	}
	err = device_create_file(lm3643->fl_lcdev.dev, &dev_attr_regdata);
	if (err < 0) {
		FLT_ERR_LOG("%s, create regdata sysfs fail\n", __func__);
	}
	err = device_create_file(lm3643->fl_lcdev.dev, &dev_attr_function_switch);
	if (err < 0) {
		FLT_ERR_LOG("%s, create function_switch sysfs fail\n", __func__);
	}
	err = device_create_file(lm3643->fl_lcdev.dev, &dev_attr_max_current);
	if (err < 0) {
		FLT_ERR_LOG("%s, create max_current sysfs fail\n", __func__);
	}
	err = device_create_file(lm3643->fl_lcdev.dev, &dev_attr_flash);
	if (err < 0) {
		FLT_ERR_LOG("%s, create max_current sysfs fail\n", __func__);
	}

	if (this_lm3643->enable_FLT_1500mA)
		FLT_INFO_LOG("Flashlight with 1.5A\n");
	if (this_lm3643->reset)
		FLT_INFO_LOG("%s reset pin exist\n", __func__);
	else
		FLT_INFO_LOG("%s no reset pin\n", __func__);
	if (this_lm3643->power_save) {
		FLT_INFO_LOG("%s power save pin exist\n", __func__);
		gpio_set_value_cansleep(this_lm3643->power_save, 0);
	}
	else
		FLT_INFO_LOG("%s no power save pin\n", __func__);
	if (this_lm3643->power_save_2) {
		FLT_INFO_LOG("%s power save pin_2 exist\n", __func__);
		gpio_set_value_cansleep(this_lm3643->power_save_2, 0);
	}
	else
		FLT_INFO_LOG("%s no power save pin_2\n", __func__);
	FLT_INFO_LOG("%s -\n", __func__);
	return 0;


platform_data_null:
	destroy_workqueue(lm3643_work_queue);
	mutex_destroy(&lm3643_mutex);
err_create_lm3643_work_queue:
	kfree(lm3643);
check_functionality_failed:
	return err;
fail_id_check:
	for (i = 0; i < parsed_leds; i++) {
		mutex_destroy(&led_array[i].lock);
		led_classdev_unregister(&led_array[i].cdev);
	}

	return rc;
}

static int lm3643_remove(struct i2c_client *client)
{
	struct lm3643_data *lm3643 = i2c_get_clientdata(client);
	struct lm3643_led_data *led_array =lm3643->led_array;

	if (led_array) {
		int i, parsed_leds = led_array->num_leds;
		for (i=0; i<parsed_leds; i++) {
			cancel_work_sync(&led_array[i].work);
			mutex_destroy(&led_array[i].lock);
			led_classdev_unregister(&led_array[i].cdev);
		}
	}

	unregister_reboot_notifier(&reboot_notifier);
	led_classdev_unregister(&lm3643->fl_lcdev);
	destroy_workqueue(lm3643_work_queue);
	mutex_destroy(&lm3643_mutex);
	unregister_early_suspend(&lm3643->fl_early_suspend);
	kfree(lm3643);

	FLT_INFO_LOG("%s:\n", __func__);
	return 0;
}

static const struct i2c_device_id lm3643_id[] = {
	{ "LM3643_FLASHLIGHT", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm3643_id);
static int lm3643_resume(struct i2c_client *client)
{

		FLT_INFO_LOG("%s:\n", __func__);
		if (this_lm3643->mode_pin_suspend_state_low)
			gpio_set_value_cansleep(this_lm3643->hwen, 1);
		return 0;
}
static int lm3643_suspend(struct i2c_client *client, pm_message_t state)
{

		FLT_INFO_LOG("%s:\n", __func__);
		if (this_lm3643->mode_pin_suspend_state_low)
			gpio_set_value_cansleep(this_lm3643->hwen, 0);

		flashlight_turn_off();
		return 0;
}

static const struct of_device_id lm3643_mttable[] = {
	{ .compatible = "LM3643_FLASHLIGHT"},
	{ },
};

static struct i2c_driver lm3643_driver = {
	.driver		= {
		.name = "LM3643_FLASHLIGHT",
		.owner = THIS_MODULE,
		.of_match_table = lm3643_mttable,
	},
	.probe		= lm3643_probe,
	.remove		= lm3643_remove,
	.suspend	= lm3643_suspend,
	.resume		= lm3643_resume,
	.id_table = lm3643_id,
};
module_i2c_driver(lm3643_driver);

MODULE_DESCRIPTION("LM3643 Led Flash driver");
MODULE_LICENSE("GPL");
