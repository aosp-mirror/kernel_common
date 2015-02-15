/* drivers/input/misc/cm32181.c - cm32181 light sensor driver
 *
 * Copyright (C) 2013 Capella Microsystems, Inc.
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

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/lightsensor.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <linux/cm32181.h>
#include <linux/fs.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include <mach/board.h>
#include <mach/board_htc.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <mach/rpm-regulator.h>
#include <mach/devices_cmdline.h>

#define D(x...) pr_info(x)

#define I2C_RETRY_COUNT 10

#define NEAR_DELAY_TIME ((100 * HZ) / 1000)

#define CALIBRATION_DATA_PATH "/calibration_data"
#define LIGHT_SENSOR_FLASH_DATA "als_flash"

#define CONTROL_INT_ISR_REPORT        0x00
#define CONTROL_ALS                   0x01

static int record_init_fail = 0;
static void sensor_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sensor_irq_work, sensor_irq_do_work);

static uint32_t adctable[10] = {0};

struct cm32181_info {
	struct class *cm32181_class;
	struct device *ls_dev;

	struct input_dev *ls_input_dev;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;

	int intr_pin;
	int als_enable;

	uint32_t *adc_table;
	uint16_t cali_table[10];
	int irq;

	int ls_calibrate;
	int ws_calibrate;
	int (*power)(int, uint8_t); /* power to the chip */

	uint32_t als_kadc;
	uint32_t emmc_als_kadc;
	uint32_t als_gadc;
	uint16_t golden_adc;

	struct wake_lock ls_wake_lock;
	int lightsensor_opened;
	uint8_t ALS_cmd_address; // could be 0x48 or 0x10, specified in platform data and is determined by pin ADDR configuration

	int current_lux_level;
	uint16_t current_adc;

	unsigned long j_start;
	unsigned long j_end;

	uint16_t ls_cmd;
	uint8_t record_clear_int_fail;
};

struct cm32181_info *lp_info;
int enable_log = 0;
int fLevel = -1;
static struct mutex als_enable_mutex, als_disable_mutex, als_get_adc_mutex;
static struct mutex CM32181_control_mutex;
static int lightsensor_enable(struct cm32181_info *lpi);
static int lightsensor_disable(struct cm32181_info *lpi);
static int initial_cm32181(struct cm32181_info *lpi);

static int als_kadc = 0;

static int control_and_report(struct cm32181_info *lpi, uint8_t mode, uint8_t cmd_enable);

static int I2C_RxData(uint16_t slaveAddr, uint8_t cmd, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	int val;
	uint8_t subaddr[1];
  
	struct cm32181_info *lpi = lp_info;

	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = 1,
		 .buf = subaddr,
		 },
		{
		 .addr = slaveAddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	subaddr[0] = cmd;

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {

		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 2) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			D("[LS][CM32181 error] %s, i2c err, slaveAddr 0x%x ISR gpio %d  = %d, record_init_fail %d \n",
				__func__, slaveAddr, lpi->intr_pin, val, record_init_fail);

		msleep(10);
	}
	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[LS_ERR][CM32181 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int I2C_TxData(uint16_t slaveAddr, uint8_t *txData, int length)
{
	uint8_t loop_i;
	int val;
	struct cm32181_info *lpi = lp_info;
	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 1) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			D("[LS][CM32181 error] %s, i2c err, slaveAddr 0x%x, value 0x%x, ISR gpio%d  = %d, record_init_fail %d\n",
				__func__, slaveAddr, txData[0], lpi->intr_pin, val, record_init_fail);

		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[LS_ERR][CM32181 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int _cm32181_I2C_Read_Word(uint16_t slaveAddr, uint8_t cmd, uint16_t *pdata)
{
	uint8_t buffer[2] = {0};
	int ret = 0;

	if (pdata == NULL)
		return -EFAULT;

	ret = I2C_RxData(slaveAddr, cmd, buffer, 2);
	if (ret < 0) {
		pr_err(
			"[LS_ERR][CM32181 error]%s: I2C_RxData fail [0x%x, 0x%x]\n",
			__func__, slaveAddr, cmd);
		return ret;
	}

	*pdata = (buffer[1]<<8)|buffer[0];
#if 0
	/* Debug use */
	printk(KERN_DEBUG "[CM32181] %s: I2C_RxData[0x%x, 0x%x] = 0x%x\n",
		__func__, slaveAddr, cmd, *pdata);
#endif
	return ret;
}

static int _cm32181_I2C_Write_Word(uint16_t SlaveAddress, uint8_t cmd, uint16_t data)
{
	char buffer[3];
	int ret = 0;
#if 0
	/* Debug use */
	printk(KERN_DEBUG
	"[CM32181] %s: _cm32181_I2C_Write_Word[0x%x, 0x%x, 0x%x]\n",
		__func__, SlaveAddress, cmd, data);
#endif
	buffer[0] = cmd;
	buffer[1] = (uint8_t)(data&0xff);
	buffer[2] = (uint8_t)((data&0xff00)>>8);	
	
	ret = I2C_TxData(SlaveAddress, buffer, 3);
	if (ret < 0) {
		pr_err("[LS_ERR][CM32181 error]%s: I2C_TxData fail\n", __func__);
		return -EIO;
	}

	return ret;
}

static int get_ls_adc_value(uint16_t *als_step, bool resume)
{
	struct cm32181_info *lpi = lp_info;
	int ret = 0;

	if (als_step == NULL)
		return -EFAULT;

	/* Read ALS data: */
	ret = _cm32181_I2C_Read_Word(lpi->ALS_cmd_address, ALS_DATA, als_step);
	if (ret < 0) {
		pr_err(
			"[LS][CM32181 error]%s: _cm32181_I2C_Read_Word fail\n",
			__func__);
		return -EIO;
	}

	return ret;
}

static int set_lsensor_range(uint16_t low_thd, uint16_t high_thd)
{
	int ret = 0;
	struct cm32181_info *lpi = lp_info;

	_cm32181_I2C_Write_Word(lpi->ALS_cmd_address, ALS_HW, high_thd);
	_cm32181_I2C_Write_Word(lpi->ALS_cmd_address, ALS_LW, low_thd);

	return ret;
}

static void sensor_irq_do_work(struct work_struct *work)
{
	struct cm32181_info *lpi = lp_info;
	control_and_report(lpi, CONTROL_INT_ISR_REPORT, 0);

	enable_irq(lpi->irq);
}

static irqreturn_t cm32181_irq_handler(int irq, void *data)
{
	struct cm32181_info *lpi = data;

	disable_irq_nosync(lpi->irq);
	if (enable_log)
		D("[LS][CM32181] %s\n", __func__);

	queue_work(lpi->lp_wq, &sensor_irq_work);

	return IRQ_HANDLED;
}

static int als_power(int enable)
{
	struct cm32181_info *lpi = lp_info;

	if (lpi->power)
		lpi->power(LS_PWR_ON, 1);

	return 0;
}

void lightsensor_set_kvalue(struct cm32181_info *lpi)
{
	if (!lpi) {
		pr_err("[LS][CM32181 error]%s: ls_info is empty\n", __func__);
		return;
	}

	D("[LS][CM32181] %s: ALS calibrated als_kadc=0x%x\n",
			__func__, lpi->emmc_als_kadc);

	if (lpi->emmc_als_kadc >> 16 == ALS_CALIBRATED)
		lpi->als_kadc = lpi->emmc_als_kadc & 0xFFFF;
	else {
		lpi->als_kadc = 0;
		D("[LS][CM32181] %s: no ALS calibrated\n", __func__);
	}

	if (lpi->als_kadc && lpi->golden_adc > 0) {
		lpi->als_kadc = (lpi->als_kadc > 0) ?
				lpi->als_kadc : lpi->golden_adc;
		lpi->als_gadc = lpi->golden_adc;
	} else {
		lpi->als_kadc = 1;
		lpi->als_gadc = 1;
	}
	D("[LS][CM32181] %s: als_kadc=0x%x, als_gadc=0x%x\n",
		__func__, lpi->als_kadc, lpi->als_gadc);
}

static int lightsensor_update_table(struct cm32181_info *lpi)
{
	uint32_t tmpData[10];
	int i;
	for (i = 0; i < 10; i++) {
		tmpData[i] = (uint32_t)(*(lpi->adc_table + i))
				* lpi->als_kadc / lpi->als_gadc ;
		if( tmpData[i] <= 0xFFFF ){
      lpi->cali_table[i] = (uint16_t) tmpData[i];		
    } else {
      lpi->cali_table[i] = 0xFFFF;    
    }         
		D("[LS][CM32181] %s: Calibrated adc_table: data[%d], %x\n",
			__func__, i, lpi->cali_table[i]);
	}

	return 0;
}

static int lightsensor_enable(struct cm32181_info *lpi)
{
	int ret = -EIO;
	
	mutex_lock(&als_enable_mutex);
	D("[LS][CM32181] %s\n", __func__);

	if (lpi->als_enable) {
		D("[LS][CM32181] %s: already enabled\n", __func__);
		ret = 0;
	} else
  	ret = control_and_report(lpi, CONTROL_ALS, 1);
	
	mutex_unlock(&als_enable_mutex);
	return ret;
}

static int lightsensor_disable(struct cm32181_info *lpi)
{
	int ret = -EIO;
	mutex_lock(&als_disable_mutex);
	D("[LS][CM32181] %s\n", __func__);

	if ( lpi->als_enable == 0 ) {
		D("[LS][CM32181] %s: already disabled\n", __func__);
		ret = 0;
	} else
    ret = control_and_report(lpi, CONTROL_ALS, 0);
	
	mutex_unlock(&als_disable_mutex);
	return ret;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
	struct cm32181_info *lpi = lp_info;
	int rc = 0;

	D("[LS][CM32181] %s\n", __func__);
	if (lpi->lightsensor_opened) {
		pr_err("[LS][CM32181 error]%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lpi->lightsensor_opened = 1;
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	struct cm32181_info *lpi = lp_info;

	D("[LS][CM32181] %s\n", __func__);
	lpi->lightsensor_opened = 0;
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc, val;
	struct cm32181_info *lpi = lp_info;

	/*D("[CM32181] %s cmd %d\n", __func__, _IOC_NR(cmd));*/

	switch (cmd) {
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		D("[LS][CM32181] %s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",
			__func__, val);
		rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
		break;
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		val = lpi->als_enable;
		D("[LS][CM32181] %s LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
			__func__, val);
		rc = put_user(val, (unsigned long __user *)arg);
		break;
	default:
		pr_err("[LS][CM32181 error]%s: invalid cmd %d\n",
			__func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lightsensor_fops
};

static ssize_t ls_adc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret;
	struct cm32181_info *lpi = lp_info;

	control_and_report(lpi, CONTROL_ALS, 1);

	D("[LS][CM32181] %s: ADC = 0x%04X, Lux Level = %d \n",
		__func__, lpi->current_adc, lpi->current_lux_level);
	ret = snprintf(buf, PAGE_SIZE, "ADC[0x%04X] => lux level %d\n",
		lpi->current_adc, lpi->current_lux_level);

	return ret;
}

static ssize_t ls_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm32181_info *lpi = lp_info;

	ret = snprintf(buf, PAGE_SIZE, "Light sensor Auto Enable = %d\n",
			lpi->als_enable);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = 0;
	int ls_auto;
	struct cm32181_info *lpi = lp_info;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1 && ls_auto != 147 && ls_auto != 148)
		return -EINVAL;

	if (ls_auto) {
		lpi->ls_calibrate = (ls_auto == 147) ? 1 : 0;
		lpi->ws_calibrate = (ls_auto == 148) ? 1 : 0;
		ret = lightsensor_enable(lpi);
	} else {
		lpi->ls_calibrate = 0;
		lpi->ws_calibrate = 0;
		ret = lightsensor_disable(lpi);
	}

	D("[LS][CM32181] %s: lpi->als_enable = %d, lpi->ls_calibrate = %d, lpi->ws_calibrate = %d, ls_auto=%d\n",
		__func__, lpi->als_enable, lpi->ls_calibrate, lpi->ws_calibrate, ls_auto);

	if (ret < 0)
		pr_err(
		"[LS][CM32181 error]%s: set auto light sensor fail\n",
		__func__);

	return count;
}

static ssize_t ls_kadc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm32181_info *lpi = lp_info;
	int ret;

	ret = snprintf(buf, PAGE_SIZE, "kadc = 0x%x, gadc = 0x%x, kadc while this boot = 0x%x\n",
			lpi->als_kadc, lpi->als_gadc, lpi->emmc_als_kadc);

	return ret;
}

static ssize_t ls_kadc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cm32181_info *lpi = lp_info;
	int kadc_temp = 0;

if(lpi->ws_calibrate != 1) {
	sscanf(buf, "%d", &kadc_temp);

	mutex_lock(&als_get_adc_mutex);
	if(kadc_temp != 0) {
		lpi->als_kadc = kadc_temp;
		lpi->als_gadc = lpi->golden_adc;
		if(  lpi->als_gadc != 0){
  			if (lightsensor_update_table(lpi) < 0)
				printk(KERN_ERR "[LS][CM32181 error] %s: update ls table fail\n", __func__);
			else
				printk(KERN_INFO "[LS]%s: als_kadc=0x%x, als_gadc=0x%x\n", __func__, lpi->als_kadc, lpi->als_gadc);
  		} else {
			printk(KERN_INFO "[LS]%s: als_gadc =0x%x wait to be set\n",
					__func__, lpi->als_gadc);
		}		
	} else {
		printk(KERN_INFO "[LS]%s: als_kadc can't be set to zero\n",
				__func__);
	}
				
	mutex_unlock(&als_get_adc_mutex);
}
	return count;
}

static ssize_t ls_adc_table_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned length = 0;
	int i;

	for (i = 0; i < 10; i++) {
		length += snprintf(buf + length, PAGE_SIZE,
			"[CM32181]Get adc_table[%d] =  0x%x ; %d, Get cali_table[%d] =  0x%x ; %d, \n",
			i, *(lp_info->adc_table + i),
			*(lp_info->adc_table + i),
			i, *(lp_info->cali_table + i),
			*(lp_info->cali_table + i));
	}
	return length;
}

static ssize_t ls_adc_table_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cm32181_info *lpi = lp_info;
	char *token[10];
	uint16_t tempdata[10];
	int i;

	printk(KERN_INFO "[LS][CM32181]%s\n", buf);
	for (i = 0; i < 10; i++) {
		token[i] = strsep((char **)&buf, " ");
		tempdata[i] = simple_strtoul(token[i], NULL, 16);
		if (tempdata[i] < 1 || tempdata[i] > 0xffff) {
			printk(KERN_ERR
			"[LS][CM32181 error] adc_table[%d] =  0x%x Err\n",
			i, tempdata[i]);
			return count;
		}
	}
	mutex_lock(&als_get_adc_mutex);
	for (i = 0; i < 10; i++) {
		lpi->adc_table[i] = tempdata[i];
		printk(KERN_INFO
		"[LS][CM32181]Set lpi->adc_table[%d] =  0x%x\n",
		i, *(lp_info->adc_table + i));
	}
	if (lightsensor_update_table(lpi) < 0)
		printk(KERN_ERR "[LS][CM32181 error] %s: update ls table fail\n",
		__func__);
	mutex_unlock(&als_get_adc_mutex);
	D("[LS][CM32181] %s\n", __func__);

	return count;
}

static ssize_t ls_fLevel_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "fLevel = %d\n", fLevel);
}

static ssize_t ls_fLevel_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cm32181_info *lpi = lp_info;
	int value=0;
	sscanf(buf, "%d", &value);
	(value>=0)?(value=min(value,10)):(value=max(value,-1));
	fLevel=value;
	input_report_abs(lpi->ls_input_dev, ABS_MISC, fLevel);
	input_sync(lpi->ls_input_dev);
	printk(KERN_INFO "[LS]set fLevel = %d\n", fLevel);

	msleep(1000);
	fLevel=-1;
	return count;
}

static DEVICE_ATTR(ls_adc, 0664, ls_adc_show, NULL);
static DEVICE_ATTR(ls_auto, 0664, ls_enable_show, ls_enable_store);
static DEVICE_ATTR(ls_kadc, 0664, ls_kadc_show, ls_kadc_store);
static DEVICE_ATTR(ls_adc_table, 0664, ls_adc_table_show, ls_adc_table_store);
static DEVICE_ATTR(ls_flevel, 0664, ls_fLevel_show, ls_fLevel_store);

static int lightsensor_setup(struct cm32181_info *lpi)
{
	int ret;

	lpi->ls_input_dev = input_allocate_device();
	if (!lpi->ls_input_dev) {
		pr_err(
			"[LS][CM32181 error]%s: could not allocate ls input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ls_input_dev->name = "lightsensor-level";
	set_bit(EV_ABS, lpi->ls_input_dev->evbit);
	input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(lpi->ls_input_dev);
	if (ret < 0) {
		pr_err("[LS][CM32181 error]%s: can not register ls input device\n",
				__func__);
		goto err_free_ls_input_device;
	}

	ret = misc_register(&lightsensor_misc);
	if (ret < 0) {
		pr_err("[LS][CM32181 error]%s: can not register ls misc device\n",
				__func__);
		goto err_unregister_ls_input_device;
	}

	return ret;

err_unregister_ls_input_device:
	input_unregister_device(lpi->ls_input_dev);
err_free_ls_input_device:
	input_free_device(lpi->ls_input_dev);
	return ret;
}

static int initial_cm32181(struct cm32181_info *lpi)
{
	int val, ret, fail_counter = 0;
	uint16_t status = 0;

	val = gpio_get_value(lpi->intr_pin);
	D("[LS][CM32181] %s, INTERRUPT GPIO val = %d\n", __func__, val);

check_interrupt_gpio:
	if (fail_counter >= 10) {
		D("[LS][CM32181] %s, initial fail_counter = %d\n", __func__, fail_counter);
		if (record_init_fail == 0)
			record_init_fail = 1;
		return -ENOMEM;/*If devices without cm32181 chip and did not probe driver*/
	}
	lpi->ls_cmd = lpi->ls_cmd | CM32181_ALS_SD; 
	ret = _cm32181_I2C_Write_Word(lpi->ALS_cmd_address, ALS_CMD, lpi->ls_cmd );
	if ((ret < 0) && (fail_counter < 10)) {	
		fail_counter++;
		val = gpio_get_value(lpi->intr_pin);
		if( val == 0 ){
			D("[LS][CM32181] %s, interrupt GPIO val = %d, , inital fail_counter %d\n",
				__func__, val, fail_counter);
 			ret = _cm32181_I2C_Read_Word(lpi->ALS_cmd_address, ALS_STATUS, &status);
 			D("[LS][CM32181] %s, status register = 0x%x, ret %d\n",
				__func__, status, ret);
 		}
		goto	check_interrupt_gpio;
	}
	
	return 0;
}

static int cm32181_setup(struct cm32181_info *lpi)
{
	int ret = 0;

	als_power(1);
	msleep(5);
	ret = gpio_request(lpi->intr_pin, "gpio_cm32181_intr");
	if (ret < 0) {
		pr_err("[LS][CM32181 error]%s: gpio %d request failed (%d)\n",
			__func__, lpi->intr_pin, ret);
		return ret;
	}

	ret = gpio_direction_input(lpi->intr_pin);
	if (ret < 0) {
		pr_err(
			"[LS][CM32181 error]%s: fail to set gpio %d as input (%d)\n",
			__func__, lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}

	ret = initial_cm32181(lpi);
	if (ret < 0) {
		pr_err(
			"[LS_ERR][CM32181 error]%s: fail to initial cm32181 (%d)\n",
			__func__, ret);
		goto fail_free_intr_pin;
	}
	
	ret = request_any_context_irq(lpi->irq,
			cm32181_irq_handler,
			IRQF_TRIGGER_LOW,
			"cm32181",
			lpi);
	if (ret < 0) {
		pr_err(
			"[LS][CM32181 error]%s: req_irq(%d) fail for gpio %d (%d)\n",
			__func__, lpi->irq,
			lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}

	return ret;

fail_free_intr_pin:
	gpio_free(lpi->intr_pin);
	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cm32181_early_suspend(struct early_suspend *h)
{
	struct cm32181_info *lpi = lp_info;

	D("[LS][CM32181] %s\n", __func__);

	if (lpi->als_enable)
		lightsensor_disable(lpi);
}

static void cm32181_late_resume(struct early_suspend *h)
{
	struct cm32181_info *lpi = lp_info;

	D("[LS][CM32181] %s\n", __func__);

	if (!lpi->als_enable)
		lightsensor_enable(lpi);
}
#endif

static int cm32181_parse_dt(struct device *dev, struct cm32181_platform_data *pdata)
{
	struct property *prop;
	struct device_node *dt = dev->of_node;
	struct device_node *offset = NULL;
	int cali_size = 0;
	unsigned char *cali_data = NULL;
	int i = 0;
//	uint32_t temp = 0;

	D("[LS][cm32181] %s: +\n", __func__);

	prop = of_find_property(dt, "cm32181,model", NULL);
	if (prop) {
		of_property_read_u32(dt, "cm32181,model", &pdata->model);
	}
        prop = of_find_property(dt, "cm32181,levels", NULL);
        if (prop) {
                of_property_read_u32_array(dt, "cm32181,levels", adctable, 10);

		pdata->levels = &adctable[0];
        }
        prop = of_find_property(dt, "cm32181,golden_adc", NULL);
        if (prop) {
                of_property_read_u32(dt, "cm32181,golden_adc", &pdata->golden_adc);
        }
        prop = of_find_property(dt, "cm32181,cm32181_slave_address", NULL);
        if (prop) {
                of_property_read_u32(dt, "cm32181,cm32181_slave_address", &pdata->cm32181_slave_address);
        }

	pdata->emmc_als_kadc = 0;
	if ((offset = of_find_node_by_path(CALIBRATION_DATA_PATH))) {
		cali_data = (unsigned char*) of_get_property(offset, LIGHT_SENSOR_FLASH_DATA, &cali_size);

		D("%s: Light sensor cali_size = %d", __func__, cali_size);
		if (cali_data) {
			for (i = 0; (i < cali_size) && (i < 4); i++) {
				D("cali_data[%d] = %02x ", i, cali_data[i]);
				pdata->emmc_als_kadc |= (cali_data[i] << (i * 8));
			}
		}
	} else
		D("%s: Light sensor calibration data offset not found", __func__);


	pdata->intr = of_get_named_gpio_flags(dt, "cm32181,irq-gpio",
				0, &pdata->irq_gpio_flags);
//	pdata->lpm_power = cm3629_sr_lpm;
	return 0;
}

static int __devinit cm32181_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct cm32181_info *lpi;
	struct cm32181_platform_data *pdata;

	D("[LS][CM32181] %s\n", __func__);

	lpi = kzalloc(sizeof(struct cm32181_info), GFP_KERNEL);
	if (!lpi)
		return -ENOMEM;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (pdata == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	if (client->dev.of_node) {
		cm32181_parse_dt(&client->dev, pdata);

	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			pr_err("[LS][CM32181 error]%s: Assign platform_data error!!\n",
				__func__);
			ret = -EBUSY;
			goto err_platform_data_null;
		}
	}

	lpi->i2c_client = client;
	lpi->irq = client->irq;

	i2c_set_clientdata(client, lpi);
	
	lpi->intr_pin = pdata->intr;
	lpi->adc_table = pdata->levels;
	lpi->emmc_als_kadc = pdata->emmc_als_kadc;
	lpi->golden_adc = pdata->golden_adc;
	lpi->power = pdata->power;
	
        lpi->ALS_cmd_address = pdata->cm32181_slave_address;

	lpi->ls_cmd  = pdata->ls_cmd;
	
	lpi->j_start = 0;
	lpi->j_end = 0;
	lpi->record_clear_int_fail = 0;
	
	if (pdata->ls_cmd == 0) {
		lpi->ls_cmd  = (CM32181_ALS_SM_2 | CM32181_ALS_IT_200MS | CM32181_ALS_PERS_1);
	}

	lp_info = lpi;

	mutex_init(&CM32181_control_mutex);

	mutex_init(&als_enable_mutex);
	mutex_init(&als_disable_mutex);
	mutex_init(&als_get_adc_mutex);

	ret = lightsensor_setup(lpi);
	if (ret < 0) {
		pr_err("[LS][CM32181 error]%s: lightsensor_setup error!!\n",
			__func__);
		goto err_lightsensor_setup;
	}

	if (!(client->dev.of_node)) {
		lpi->emmc_als_kadc = als_kadc;
		D("[LS][cm32181] %s: Light sensor use ATAG Calibration data\n", __func__);
	}

	lightsensor_set_kvalue(lpi);
	ret = lightsensor_update_table(lpi);
	if (ret < 0) {
		pr_err("[LS][CM32181 error]%s: update ls table fail\n",
			__func__);
		goto err_lightsensor_update_table;
	}

	lpi->lp_wq = create_singlethread_workqueue("cm32181_wq");
	if (!lpi->lp_wq) {
		pr_err("[PS][CM32181 error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}
	wake_lock_init(&(lpi->ls_wake_lock), WAKE_LOCK_SUSPEND, "ls_wake_lock");

	ret = cm32181_setup(lpi);
	if (ret < 0) {
		pr_err("[PS_ERR][CM32181 error]%s: cm32181_setup error!\n", __func__);
		goto err_cm32181_setup;
	}

	lpi->cm32181_class = class_create(THIS_MODULE, "optical_sensors");
	if (IS_ERR(lpi->cm32181_class)) {
		ret = PTR_ERR(lpi->cm32181_class);
		lpi->cm32181_class = NULL;
		goto err_create_class;
	}

	lpi->ls_dev = device_create(lpi->cm32181_class,
				NULL, 0, "%s", "lightsensor");
	if (unlikely(IS_ERR(lpi->ls_dev))) {
		ret = PTR_ERR(lpi->ls_dev);
		lpi->ls_dev = NULL;
		goto err_create_ls_device;
	}

	/* register the attributes */

        ret = device_create_file(lpi->ls_dev, &dev_attr_ls_adc);
	if (ret)
		goto err_sysfs_create_group_light;

	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_auto);
	if (ret)
		goto err_sysfs_create_group_light;

	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_kadc);
	if (ret)
		goto err_sysfs_create_group_light;

	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_adc_table);
	if (ret)
		goto err_sysfs_create_group_light;

	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_flevel);
	if (ret)
		goto err_sysfs_create_group_light;

#ifdef CONFIG_HAS_EARLYSUSPEND
	lpi->early_suspend.level =
			EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	lpi->early_suspend.suspend = cm32181_early_suspend;
	lpi->early_suspend.resume = cm32181_late_resume;
	register_early_suspend(&lpi->early_suspend);
#endif

	D("[PS][CM32181] %s: Probe success!\n", __func__);
	lpi->als_enable = 0;
  
	return ret;

err_sysfs_create_group_light:
	device_unregister(lpi->ls_dev);
err_create_ls_device:
	class_destroy(lpi->cm32181_class);
err_create_class:
err_cm32181_setup:
	destroy_workqueue(lpi->lp_wq);
	wake_lock_destroy(&(lpi->ls_wake_lock));

	input_unregister_device(lpi->ls_input_dev);
	input_free_device(lpi->ls_input_dev);
err_create_singlethread_workqueue:
err_lightsensor_update_table:
	mutex_destroy(&CM32181_control_mutex);
	misc_deregister(&lightsensor_misc);
err_lightsensor_setup:
	mutex_destroy(&als_enable_mutex);
	mutex_destroy(&als_disable_mutex);
	mutex_destroy(&als_get_adc_mutex);
err_platform_data_null:
err_alloc_data_failed:
	kfree(lpi);
	return ret;
}
   
static int control_and_report( struct cm32181_info *lpi, uint8_t mode, uint8_t cmd_enable ) {
	int ret = 0;
	uint16_t adc_value = 0;	
	int val;
	int fail_counter = 0;
	uint16_t status = 0;	
	uint32_t lux_level;
	int i = 0;
	
	mutex_lock(&CM32181_control_mutex);

	while(1){
		gpio_get_value(lpi->intr_pin);
//		D("[LS][CM32181] %s, interrupt GPIO val = %d, fail_counter %d\n",
//			__func__, val, fail_counter);
      	
		val = gpio_get_value(lpi->intr_pin);
		if( val == 0){
			_cm32181_I2C_Read_Word(lpi->ALS_cmd_address, ALS_STATUS, &status);
//			D("[LS][CM32181] %s, interrupt GPIO val = %d, status register = 0x%x, ret %d\n",
//				__func__, val, status, ret);
		}

		lpi->ls_cmd &= ~CM32181_ALS_INT_EN;
		ret = _cm32181_I2C_Write_Word(lpi->ALS_cmd_address, ALS_CMD, lpi->ls_cmd);
		if( ret == 0 ){
			break;
		}
		else {
			fail_counter++;
			val = gpio_get_value(lpi->intr_pin);
			D("[LS][CM32181] %s, interrupt GPIO val = %d, , inital fail_counter %d\n",
				__func__, val, fail_counter);
		}
		if (fail_counter >= 10) {
			D("[LS][CM32181] %s, clear INT fail_counter = %d\n", __func__, fail_counter);
			if (lpi->record_clear_int_fail == 0)
			lpi->record_clear_int_fail = 1;
			ret=-ENOMEM;
			goto error_clear_interrupt;
		}
	}

  if( mode == CONTROL_ALS ){
    if(cmd_enable){
      lpi->ls_cmd &= ~CM32181_ALS_SD;      
    } else {
      lpi->ls_cmd |= CM32181_ALS_SD;
    }
    _cm32181_I2C_Write_Word(lpi->ALS_cmd_address, ALS_CMD, lpi->ls_cmd);
    lpi->als_enable=cmd_enable;
  }
  
  if((mode == CONTROL_ALS)&&(cmd_enable==1)){
		  input_report_abs(lpi->ls_input_dev, ABS_MISC, -1);
		  input_sync(lpi->ls_input_dev);
		  msleep(100);  
  }
  
	if (lpi->als_enable)
	{
		get_ls_adc_value(&adc_value, 0);
                for (i = 0; i < 10; i++) {
		    if (adc_value <= (*(lpi->adc_table + i))) {
		  	    lux_level = i;
			    if (*(lpi->adc_table + i))
				    break;
		    }
		    if (i == 9) {/*avoid  i = 10, because 'cali_table' of size is 10 */
			    lux_level = i;
			    break;
		    }
	        }
  
		D("[LS][CM32181] %s: raw adc = 0x%04X, ls_calibrate = %d\n",
			__func__, adc_value, lpi->ls_calibrate);

		lpi->ls_cmd |= CM32181_ALS_INT_EN;

		// set interrupt high/low threshold
                set_lsensor_range(((i == 0) || (adc_value == 0)) ? 0 :
				*(lpi->cali_table + (i - 1)) + 1,
				*(lpi->cali_table + i));

		if ((i == 0) || (adc_value == 0))
			D("[LS][cm32181] %s: ADC=0x%03X, Level=%d, l_thd equal 0, h_thd = 0x%x \n",
				__func__, adc_value, lux_level, *(lpi->cali_table + i));
		else
			D("[LS][cm32181] %s: ADC=0x%03X, Level=%d, l_thd = 0x%x, h_thd = 0x%x \n",
				__func__, adc_value, lux_level, *(lpi->cali_table + (i - 1)) + 1, *(lpi->cali_table + i));
		lpi->current_lux_level = lux_level;
		lpi->current_adc = adc_value;    

		input_report_abs(lpi->ls_input_dev, ABS_MISC, lux_level);
		input_sync(lpi->ls_input_dev);
	}
 
  ret = _cm32181_I2C_Write_Word(lpi->ALS_cmd_address, ALS_CMD, lpi->ls_cmd);
  if( ret != 0 )
    D("[LS][CM32181] %s, re-enable INT FAIL\n", __func__);

error_clear_interrupt:
  mutex_unlock(&CM32181_control_mutex);
  return ret;
}

static const struct i2c_device_id cm32181_i2c_id[] = {
	{"CM32181", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, cm32181_i2c_id);

static struct of_device_id cm3629_match_table[] = {
	{.compatible = "CM32181"},
	{},
};

static struct i2c_driver cm32181_driver = {
	.driver = {
		.name = "CM32181",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = cm3629_match_table,
#endif
	},
	.probe = cm32181_probe,
	.id_table = cm32181_i2c_id,
};
module_i2c_driver(cm32181_driver);
#ifndef CONFIG_OF
static int __init cm32181_init(void)
{
	return i2c_add_driver(&cm32181_driver);
}

static void __exit cm32181_exit(void)
{
	i2c_del_driver(&cm32181_driver);
}

module_init(cm32181_init);
module_exit(cm32181_exit);
#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CM32181 Light Sensor Driver");
MODULE_AUTHOR("Capella Microsystems");
