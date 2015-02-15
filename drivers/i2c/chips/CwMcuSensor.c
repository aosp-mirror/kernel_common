#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h> 
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/CwMcuSensor.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/vibtrig.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <mach/devices_cmdline.h>
#endif
#include <linux/workqueue.h>
#if HTC_ENABLE_SENSORHUB_DEBUG
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#endif
#include <mach/gpiomux.h>
#include <mach/board_htc.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#define D(x...) printk(KERN_DEBUG "[S_HUB][CW_MCU] " x)
#define I(x...) printk(KERN_INFO "[S_HUB][CW_MCU] " x)
#define E(x...) printk(KERN_ERR "[S_HUB][CW_MCU] " x)
#define HTC_ENABLE_SENSORHUB_UART_DEBUG
#define RETRY_TIMES 20
#define ACTIVE_RETRY_TIMES 10
#define ENABLE_LIST_RETRY_TIMES 5
#define DPS_MAX			(1 << (16 - 1))

#define CWMCU_POLL_INTERVAL	10
#define CWMCU_POLL_MAX		200
#define CWMCU_POLL_MIN		10
#define TOUCH_LOG_DELAY		5000

#define HTC_Gesture_Motion 0x07
#define REL_Significant_Motion REL_WHEEL
#define HTC_Any_Motion 0x09
#define HTC_Matrix_Gesture 0x0a
#define HTC_Gesture_Motion_HIDI 0x0b
#define HTC_Matrix_Gesture_HIDI 0x0c

#define USE_WAKE_MCU 1
#define CALIBRATION_DATA_PATH "/calibration_data"
#define G_SENSOR_FLASH_DATA "gs_flash"
#define GYRO_SENSOR_FLASH_DATA "gyro_flash"
#define LIGHT_SENSOR_FLASH_DATA "als_flash"
#define PROX_SENSOR_FLASH_DATA "ps_flash"
#define BARO_SENSOR_FLASH_DATA "bs_flash"
#define MFG_MODE 1

#if USE_WAKE_MCU
static int UseWakeMcu = 0;
#endif

#if HTC_ENABLE_SENSORHUB_DEBUG
#define MAX_DEBUG_BUF_SIZE 1536
#define MAX_BACKUP_CALL_STACK_SIZE 64
#define MAX_CALL_STACK_SIZE 512
#define MAX_I2C_BUF_SIZE 32
#define VIB_TIME 20
static int cwmcu_opened;
static int cwmcu_wdg_reset = 0;
static int cwmcu_i2c_error = 0;
u8 global_debug_buffer[MAX_DEBUG_BUF_SIZE];
u8 global_call_stack_buffer[MAX_CALL_STACK_SIZE];
#endif
static int probe_i2c_fail=0;
static int probe_success;
static int DEBUG_FLAG_GSENSOR = 0;
module_param(DEBUG_FLAG_GSENSOR,int,0600);
static int DEBUG_FLAG_COMPASS = 0;
module_param(DEBUG_FLAG_COMPASS,int,0600);
static int DEBUG_FLAG_GYRO = 0;
module_param(DEBUG_FLAG_GYRO,int,0600);
static int DEBUG_FLAG_PRESSURE = 0;
module_param(DEBUG_FLAG_PRESSURE,int,0600);
static int DEBUG_FLAG_FUSION = 0;
module_param(DEBUG_FLAG_FUSION,int,0600);
static int DEBUG_FLAG_REALTIME_TRANSPORT = 0;
module_param(DEBUG_FLAG_REALTIME_TRANSPORT, int, 0600);
static int DEBUG_FLAG_BUFFERED_TRANSPORT = 0;
module_param(DEBUG_FLAG_BUFFERED_TRANSPORT, int, 0600);
static int DEBUG_FLAG_MAGNETIC_UNCALIBRATED = 0;
module_param(DEBUG_FLAG_MAGNETIC_UNCALIBRATED, int, 0600);
static int DEBUG_FLAG_GEOMAGNETIC_ROTATION_VECTOR = 0;
module_param(DEBUG_FLAG_GEOMAGNETIC_ROTATION_VECTOR, int, 0600);
static int DEBUG_FLAG_TIME = 10;
module_param(DEBUG_FLAG_TIME, int, 0600);
static int p_status = 9;
static struct vib_trigger *vib_trigger = NULL;

static void polling_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(polling_work, polling_do_work);

static void exception_do_work_wdg(struct work_struct *w);
static DECLARE_DELAYED_WORK(exception_work_wdg, exception_do_work_wdg);

static void resume_do_work(struct work_struct *w);
static DECLARE_WORK(resume_work, resume_do_work);

static void touch_log_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(touch_log_work, touch_log_do_work);

struct workqueue_struct *mcu_wq;

struct wake_lock ges_wake_lock;
struct wake_lock significant_wake_lock;

static int power_key_pressed = 0;
struct CWMCU_data {
	struct i2c_client *client;
	atomic_t delay;
	int resume_done;
	struct input_polled_dev *input_polled;
	struct input_dev *input;
	struct timeval previous;
	struct timeval now;
	int time_diff;
        struct class *sensor_class;
        struct device *sensor_dev;
	u8	Acceleration_axes;
	u8	Magnetic_axes;
	u8	Gyro_axes;
	
	u32	enabled_list;
	
	int	sensors_time[numSensors];
	int	report_period[numSensors];
	int IRQ;
	struct delayed_work work;
	struct work_struct irq_work;
	struct input_dev *input_htc_gesture;
	
	uint32_t gpio_wake_mcu;
	uint32_t gpio_reset;
	uint32_t gpio_chip_mode;
	uint32_t gpio_mcu_irq;
	int GS_chip_layout;
	int gs_kvalue;
	int gs_kvalue_R1;
	int gs_kvalue_R2;
	int gs_kvalue_R3;
	int gs_kvalue_L1;
	int gs_kvalue_L2;
	int gs_kvalue_L3;
	int gy_kvalue;
	u8 ALS_goldh;
	u8 ALS_goldl;
	u8 ls_polling;
	u8 proximity_debu_info;
	int als_kvalue;
	int ps_kvalue;
	int ps_kheader;
	int bs_kvalue;
	int bs_kheader;
	u8  ps_calibrated;
	u8  gs_calibrated;
	u8  ls_calibrated;
	u8  bs_calibrated;
	u8  gy_calibrated;

	u8 filter_first_zeros[numSensors];
	int mfg_mode;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
	struct workqueue_struct *mcu_poll_wq;
	struct delayed_work work_poll;
#endif
	u32 gesture_motion_param;
};
static struct CWMCU_data *mcu_data;
static int CWMCU_i2c_read(struct CWMCU_data *sensor,
                         u8 reg_addr, u8 *data, u8 len);
static int CWMCU_i2c_write(struct CWMCU_data *sensor,
                          u8 reg_addr, u8 *data, u8 len);
static int mc_power_controller(int en);
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data);
#endif
#ifdef CONFIG_FB
static void mcu_fb_register(struct work_struct *work)
{
	int ret = 0;
	D("%s in", __func__);

	mcu_data->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&mcu_data->fb_notif);
	if (ret)
		D("MCU ERR:Unable to register fb_notifier: %d\n", ret);
}
#endif
#if HTC_ENABLE_SENSORHUB_DEBUG
static int cwmcu_read_debug_status(u8 *status) {

	int err = 0;
	u8 data[1]={0};
	D("[CWMCU] %s\n",__func__);
	err = CWMCU_i2c_read(mcu_data, CWSTM32_READ_Debug_Status, data, 1);
	if (err != 0) {
		E("[CWMCU] failed to enable dubug. num = %d", err);
		return err;
	} else {
		*status = data[0];
	}
	return err;
}

static void cwmcu_parse_call_stack(int length) {
	int m = 1, n = 0;
	D("[CWMCU] %s\n",__func__);
	for (n = length - 1; n >= 0; n--) {
		
		if ((n % 4 == 0) && n != length - 1) {
			printk("< == [CWMCU] dump call stack [%d]\n", m);
			m++;
		}
		printk("%x ", global_call_stack_buffer[n]);
	}
}

static int cwmcu_dump_call_stack(int length) {

	int err = 0;
	u8 data[MAX_I2C_BUF_SIZE]={0};
	int index = 0;
	int i;
	D("[CWMCU] %s, length = %d\n",__func__, length);

	if (length == MAX_CALL_STACK_SIZE) {
		for (i = 0; i < MAX_CALL_STACK_SIZE / MAX_I2C_BUF_SIZE; i++) {
			err = CWMCU_i2c_read(mcu_data, CWSTM32_READ_Dump_Call_Stack_Buffer, data, MAX_I2C_BUF_SIZE);
			if (err != 0) {
				E("[CWMCU] failed to dump call stack. err = %d", err);
				break;
			} else {
				memcpy((global_call_stack_buffer + index), &data[0] ,MAX_I2C_BUF_SIZE);
				index += MAX_I2C_BUF_SIZE;
			}
		}
		cwmcu_parse_call_stack(MAX_CALL_STACK_SIZE);

	} else if (length == MAX_BACKUP_CALL_STACK_SIZE) {
		for (i = 0; i < MAX_BACKUP_CALL_STACK_SIZE / MAX_I2C_BUF_SIZE; i++) {
			err = CWMCU_i2c_read(mcu_data, CWSTM32_READ_Dump_Backup_Call_Stack_Buffer, data, MAX_I2C_BUF_SIZE);
			if (err != 0) {
				E("[CWMCU] failed to dump call stack2. err = %d", err);
				break;
			} else {
				memcpy((global_call_stack_buffer + index), &data[0] ,MAX_I2C_BUF_SIZE);
				index += MAX_I2C_BUF_SIZE;
			}
		}
		cwmcu_parse_call_stack(MAX_BACKUP_CALL_STACK_SIZE);
	}
	return err;
}

static int cwmcu_dump_debug(void) {

	int err = 0;
	u8 data[MAX_I2C_BUF_SIZE]={0};
	int index = 0;
	int i;
	D("[CWMCU] %s\n",__func__);
	for (i = 0; i < MAX_DEBUG_BUF_SIZE / MAX_I2C_BUF_SIZE; i++) {
		err = CWMCU_i2c_read(mcu_data, CWSTM32_READ_Dump_Debug_Buffer, data, MAX_I2C_BUF_SIZE);
		if (err != 0) {
		    E("[CWMCU] failed to dump debug. err = %d", err);
		    break;
		} else {
		    memcpy((global_debug_buffer + index), &data[0] ,MAX_I2C_BUF_SIZE);
		    index += MAX_I2C_BUF_SIZE;
		}
	}
	return err;
}

static int cwmcu_write_reg(u8 reg, u8 val) {
	int err = 0;
	D("[CWMCU] %s\n",__func__);
	err = CWMCU_i2c_write(mcu_data, reg, &val, 1);
	return err;
}

static long cwmcu_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	unsigned char reg_value[2];
	long rc = 0;
	u8 write_data = 0;
	D("[CWMCU] %s\n",__func__);
	switch (cmd) {
	case CWSTM32_WRITE_Switch_Debug:

	       D("[CWMCU] CWSTM32_WRITE_Switch_Debug \n");
	       rc = cwmcu_read_debug_status(&write_data);
		if (rc != -1) {
			if (copy_from_user(reg_value, argp, sizeof(reg_value))) {
				pr_err("[CWMCU] CWSTM32_WRITE_Switch_Debug failed");
				goto err1;
			}
			pr_info("[CWMCU] %s: reg_value[1]=%2x, write_data=%2x\n", __func__, reg_value[1], write_data);
			if (reg_value[1] == 1) {
				write_data |= (1L << 0); 
			} else {
				write_data &= ~(1L << 0);
			}
			rc = cwmcu_write_reg(reg_value[0], write_data);
		}
	err1:
	       break;
	case CWSTM32_READ_Dump_Debug:
		D("[CWMCU] CWSTM32_READ_Dump_Debug\n");

		rc = cwmcu_dump_debug();
		if (rc == 0) {
			if (copy_to_user((void *)arg, &global_debug_buffer, sizeof(global_debug_buffer))) {
				pr_err("[CWMCU] CWSTM32_READ_Dump_Debug failed");
				goto err3;
			}
		}
	err3:
	       break;
	case CWSTM32_READ_Dump_Call_Stack:
		D("[CWMCU] CWSTM32_READ_Dump_Call_Stack\n");

		rc = cwmcu_dump_call_stack(MAX_CALL_STACK_SIZE);
		if (rc == 0) {
			if (copy_to_user((void *)arg, &global_call_stack_buffer, sizeof(global_call_stack_buffer))) {
				pr_err("[CWMCU] CWSTM32_READ_Dump_Call_Stack failed");
				goto err4;
			}
		}
		err4:
			break;
	default:
		pr_err("[CWMCU] %s: Invalid command\n", __func__);
		rc = -EINVAL;
		break;
	}
	return rc;
}


static int cwmcu_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	D("[CWMCU] %s\n",__func__);
	if (cwmcu_opened) {
	      pr_info("%s: busy\n", __func__);
	}
	cwmcu_opened = 1;
	return rc;
}

static int cwmcu_release(struct inode *inode, struct file *file)
{
	D("[CWMCU] %s\n",__func__);
	cwmcu_opened = 0;
	return 0;
}

static struct file_operations cwmcu_fops = {
	.owner = THIS_MODULE,
	.open = cwmcu_open,
	.release = cwmcu_release,
	.unlocked_ioctl = cwmcu_ioctl,
};

static struct miscdevice cwmcu_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cwmcu",
	.fops = &cwmcu_fops,
};
#endif

#ifdef HTC_ENABLE_SENSORHUB_UART_DEBUG
static int uart_debug_switch(struct device *dev, struct device_attribute *attr,const char *buf, size_t count){

	unsigned long switch_uart = 0, rc = 0;
	unsigned char reg_addr = 0xFF;
	unsigned char write_data = 0;

	D("[CWMCU] Uart Debug Switch \n");
	rc = strict_strtoul(buf, 10, &switch_uart);
	if (rc)
		return rc;
	rc = cwmcu_read_debug_status(&write_data);
	if (rc != -1) {
		pr_info("[CWMCU] %s: reg_addr=%2x, write_data=%2x\n", __func__, reg_addr, write_data);
		if (switch_uart == 1) {
			write_data |= (1L << 1); 
		} else {
			write_data &= ~(1L << 1);
		}
		cwmcu_write_reg(reg_addr, write_data);
	}
	return count;
}
#endif


static int mc_power_controller(int en){

	
	
	return 0;
}

static int CWMCU_i2c_write_block(struct CWMCU_data *sensor, u8 reg_addr, u8 *data, u8 len){
	int dummy;
	int retry = 0;

	for(retry = 0; retry <= RETRY_TIMES ; retry++) {
		dummy = i2c_smbus_write_block_data(sensor->client, reg_addr, len, data);
		if (dummy) {
			E("%s: i2c write error, retry = %d, dummy = %d\n",
				__func__, retry, dummy);
			continue;
		} else
			break;
	}

	if (retry > RETRY_TIMES) {
		E("%s: retry over %d\n", __func__, RETRY_TIMES);
		queue_delayed_work(mcu_wq, &exception_work_wdg, msecs_to_jiffies(atomic_read(&sensor->delay)));
		return -EIO;
	}

	return 0;
}
static int CWMCU_Get_Calibrator_Status(u8 sensor_id, u8 *data){
	int error_msg = 0;
	if(sensor_id == CW_ACCELERATION){
		error_msg = CWMCU_i2c_read(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_ACC, data,1);
	}else if(sensor_id == CW_MAGNETIC){
		error_msg = CWMCU_i2c_read(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_MAG, data,1);
	}else if(sensor_id == CW_GYRO){
		error_msg = CWMCU_i2c_read(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_GYRO, data,1);
	}
	return error_msg;
}

static int CWMCU_Set_Calibrator(u8 sensor_id, u8 *data){
	int error_msg = 0;
	
	if(sensor_id == CW_ACCELERATION){
		error_msg = CWMCU_i2c_write_block(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_ACC, data,3);
	}else if(sensor_id == CW_MAGNETIC){
		error_msg = CWMCU_i2c_write_block(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_MAG, data,26);
	}else if(sensor_id == CW_GYRO){
		error_msg = CWMCU_i2c_write_block(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_GYRO, data,3);
	}else if(sensor_id == CW_LIGHT){
		error_msg = CWMCU_i2c_write_block(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT, data,4);
	}else if(sensor_id == CW_PROXIMITY){
                error_msg = CWMCU_i2c_write_block(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY, data,2);
        }else if(sensor_id == CW_PRESSURE){
                error_msg = CWMCU_i2c_write_block(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE, data,4);
        }
	D("CYWEE sensors_id=%d calibrator data=%s\n",(int)sensor_id, data);
	return error_msg;
}

static int CWMCU_Get_Calibrator(u8 sensor_id, u8 *data){
	int error_msg = 0;
	
	if(sensor_id == CW_ACCELERATION){
		error_msg = CWMCU_i2c_read(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_ACC, data,3);
	}else if(sensor_id == CW_MAGNETIC){
		error_msg = CWMCU_i2c_read(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_MAG, data,26);
	}else if(sensor_id == CW_GYRO){
		error_msg = CWMCU_i2c_read(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_GYRO, data,3);
	}else if(sensor_id == CW_LIGHT){
		error_msg = CWMCU_i2c_read(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_LIGHT, data,4);
	}else if(sensor_id == CW_PROXIMITY){
		error_msg = CWMCU_i2c_read(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_PROXIMITY, data,4);
	}else if(sensor_id == CW_PRESSURE){
                error_msg = CWMCU_i2c_read(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_PRESSURE, data,4);
        }
	D("CYWEE sensors_id=%d calibrator data=%s\n",(int)sensor_id, data);
	return error_msg;
}

static int set_calibrator_en(struct device *dev,struct device_attribute *attr,const char *buf, size_t count){
	u8 data[3]={0};
	u8 data2[3]={0};
	
	int sensors_id = 0;
	
	sscanf(buf, "%d\n",&sensors_id);
#if 0
       data[0] = (u8)sensors_id;
       error_msg = CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_ENABLE, data,1);
#endif

	data[0] = (u8)sensors_id;
	D("set_calibrator_en sensor id is %x\n",data[0]);
	switch (data[0]) {
		case 1:
			CWMCU_i2c_read(mcu_data, G_SENSORS_STATUS, data2, 1);
			data[0]=16;
			data[0] = data2[0] | data[0];
			D("gs  data2 is %x data is %x",data2[0],data[0]);
			CWMCU_i2c_write(mcu_data, G_SENSORS_STATUS, data,1);

			break;
		case 2:
			CWMCU_i2c_read(mcu_data, ECOMPASS_SENSORS_STATUS, data2, 1);
			data[0]=16;
			data[0] = data2[0] | data[0];
			D("compass  data2 is %x data is %x",data2[0],data[0]);
			CWMCU_i2c_write(mcu_data, ECOMPASS_SENSORS_STATUS, data,1);
			break;
		case 4:
			CWMCU_i2c_read(mcu_data, GYRO_SENSORS_STATUS, data2, 1);
			data[0]=16;
			data[0] = data2[0] | data[0];
			D("gyro  data2 is %x data is %x",data2[0],data[0]);
			CWMCU_i2c_write(mcu_data, GYRO_SENSORS_STATUS, data,1);
			break;
		case 7:
			CWMCU_i2c_read(mcu_data, LIGHT_SENSORS_STATUS, data2, 1);
			data[0]=16;
			data[0] = data2[0] | data[0];
			D("light  data2 is %x data is %x",data2[0],data[0]);
			CWMCU_i2c_write(mcu_data, LIGHT_SENSORS_STATUS, data,1);
			break;
		case 8:
			CWMCU_i2c_read(mcu_data, PROXIMITY_SENSORS_STATUS, data2, 1);
			data[0]=16;
			data[0] = data2[0] | data[0];
			D("proximity  data2 is %x data is %x",data2[0],data[0]);
			CWMCU_i2c_write(mcu_data, PROXIMITY_SENSORS_STATUS, data,1);
			break;
		case 9:
			data[0]=2; 
			CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_TARGET_ACC, data,1);
			CWMCU_i2c_read(mcu_data, G_SENSORS_STATUS, data2, 1);
			data[0]=16;
			data[0] = data2[0] | data[0];
			D("gs  data2 is %x data is %x",data2[0],data[0]);
			CWMCU_i2c_write(mcu_data, G_SENSORS_STATUS, data,1);
			break;
		case 10:
			data[0]=1; 
			CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_TARGET_ACC, data,1);
			CWMCU_i2c_read(mcu_data, G_SENSORS_STATUS, data2, 1);
			data[0]=16;
			data[0] = data2[0] | data[0];
			D("gs  data2 is %x data is %x",data2[0],data[0]);
			CWMCU_i2c_write(mcu_data, G_SENSORS_STATUS, data,1);
			break;
		case 11:
			data[0]=0; 
			CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_TARGET_ACC, data,1);
			break;
		default:
			break;
	}

       return count;
}

static int show_calibrator_status_acc(struct device *dev, struct device_attribute *attr, char *buf){
       u8 data[6]={0};
       if(CWMCU_Get_Calibrator_Status(CW_ACCELERATION,data)>=0){
               return snprintf(buf, PAGE_SIZE, "%d\n",(int)data[0]);
       }
       return snprintf(buf, PAGE_SIZE, "%d\n",1);
}
static int show_calibrator_status_mag(struct device *dev, struct device_attribute *attr, char *buf){
       u8 data[6]={0};
       if(CWMCU_Get_Calibrator_Status(CW_MAGNETIC,data)>=0){
               return snprintf(buf, PAGE_SIZE, "%d\n",(int)data[0]);
       }
       return snprintf(buf, PAGE_SIZE, "%d\n",1);
}
static int show_calibrator_status_gyro(struct device *dev, struct device_attribute *attr, char *buf){
       u8 data[6]={0};
       if(CWMCU_Get_Calibrator_Status(CW_GYRO,data)>=0){
               return snprintf(buf, PAGE_SIZE, "%d\n",(int)data[0]);
       }
       return snprintf(buf, PAGE_SIZE, "%d\n",1);
}
static int set_k_value_acc_f(struct device *dev,struct device_attribute *attr,const char *buf, size_t count){
       int i = 0;
       u8 data[3]={0};
       int data_temp[3]={0};
       sscanf(buf, "%d %d %d\n", &data_temp[0], &data_temp[1], &data_temp[2]);
       D("G-sensor calibration by attr\n");
       for(i = 0;i<3;i++){
               data[i] = ((s8)data_temp[i]);
	       
	       CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_ACC,&data[i],1);
       }
#if 0
       CWMCU_Set_Calibrator(CW_ACCELERATION,data);
#endif
       return count;
}


static int set_k_value_mag_f(struct device *dev,struct device_attribute *attr,const char *buf, size_t count){
       int i = 0;
       u8 data_temp[26]={0};
       int data[26]={0};
       sscanf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n"
               , &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6], &data[7], &data[8], &data[9]
               , &data[10], &data[11], &data[12], &data[13], &data[14], &data[15], &data[16], &data[17], &data[18], &data[19]
               , &data[20], &data[21], &data[22], &data[23], &data[24], &data[25]
               );
       D("set compass calibration\n");
       for(i = 0;i<26;i++){
               data_temp[i] = (u8)((s8)data[i]);
               
               CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_MAG,&data_temp[i],1);
       }
#if 0
       CWMCU_Set_Calibrator(CW_MAGNETIC,data_temp);
#endif
       return count;
}
static int set_k_value_gyro_f(struct device *dev,struct device_attribute *attr,const char *buf, size_t count){
       int i = 0;
       u8 data[3]={0};
       int data_temp[3]={0};
       sscanf(buf, "%d %d %d\n", &data_temp[0], &data_temp[1], &data_temp[2]);
       for(i = 0;i<3;i++){
               data[i] = (u8)((s8)data_temp[i]);
       }
       CWMCU_Set_Calibrator(CW_GYRO,data);
       return count;
}

static int set_k_value_proximity_f(struct device *dev,struct device_attribute *attr,const char *buf, size_t count){
	int i = 0;
	u8 data[10]={0};
	int data_temp[10]={0};
	sscanf(buf, "%x %x\n", &data_temp[0], &data_temp[1]);
	D("set proximity calibration\n");
	for(i = 0;i<2;i++){
		data[i] = (u8)(s8) data_temp[i] & 0xFF;
		D("[proximity] data[%d] = %d, data_temp[%d] = %d\n", 2*i, data[i], 2*i, data_temp[i]);
		CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&data[i],1);
		data[i] = (u8)(s8) (data_temp[i] >> 8) & 0xFF;
		D("[proximity] data[%d] = %d, data_temp[%d] = %d\n", 2*i + 1,data[i] , 2*i + 1, data_temp[i]);
		CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&data[i],1);
	}
#if 0
	CWMCU_Set_Calibrator(CW_PROXIMITY,data);
#endif
	return count;
}
#if 0
static int CWMCU_Set_Calibrator_Pressure(u8 sensor_id, u8 *data){

	int error_msg = 0;
	
	u8 datatemp0,datatemp1,datatemp2,datatemp3;
#if 0
	datatemp3 =0x82;
	datatemp2 =0x86;
	datatemp1 =0x82;
	datatemp0 =0x82;

	for(i=0;i<=3;i++){
		
		printk("Set_Calibrator_Pressure data[%d] is %x datatemp[%d] is %x\n",i,data[i],i,datatemp[i]);
		error_msg = CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE, &datatemp[i],1);
	}
#endif
	error_msg = CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE, &datatemp0,1);
	printk("Set_Calibrator_Pressure data0 is %x\n",datatemp0);
	error_msg = CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE, &datatemp1,1);
	printk("Set_Calibrator_Pressure data1 is %x\n",datatemp1);
	error_msg = CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE, &datatemp2,1);
	printk("Set_Calibrator_Pressure data2 is %x\n",datatemp2);
	error_msg = CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE, &datatemp3,1);
	printk("Set_Calibrator_Pressure data3 is %x\n",datatemp3);
	return 0;
}
#endif

static int set_k_value_barometer_f(struct device *dev,struct device_attribute *attr,const char *buf, size_t count){
       int i = 0;
       u8 data[4]={0};
       int data_temp[4]={0};

       sscanf(buf, "%x %x %x %x\n", &data_temp[0], &data_temp[1], &data_temp[2], &data_temp[3]);

       for (i = 0; i <= 3; i++) {
               data[i] = (s8)(data_temp[i]);
               D("Set barometer calibration data[%d] is %x\n", i, data[i]);
               CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE, &data[i], 1);
       }

       return count;
}

static int set_ps_canc(struct device *dev,struct device_attribute *attr,const char *buf, size_t count){
	int code = 0;
	u8 PS_canc1, PS_canc2 = 0;
	u8 PS_th1, PS_th2 = 0;

	sscanf(buf, "%d\n", &code);

	if(code == 1) {
		if (((mcu_data->ps_kvalue >> 16) & 0xFFFF) == 0xFFFF) {
			PS_canc1 = 0x00;
			PS_canc2 = 0x00;
			CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc1,1);
			CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc2,1);
			PS_th1 = (mcu_data->ps_kvalue >>  8)& 0xFF;
			PS_th2 = 0x00;
			CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_PROXIMITY,&PS_th1,1);
			CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th2,1);
		} else {
			PS_canc1 = 0x00;
			PS_canc2 = 0x00;
			CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc1,1);
			CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc2,1);
			PS_th1 = (mcu_data->ps_kvalue & 0xFF0000) >>  16;
			PS_th2 = (mcu_data->ps_kvalue & 0xFF000000) >>  24;
			CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_PROXIMITY,&PS_th1,1);
			CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th2,1);
		}
		D("Reset PS_canc1 = %d, PS_canc2 = %d, PS_th1 = %d PS_th2 = %d\n", PS_canc1, PS_canc2, PS_th1, PS_th2);
	} else {
		if (((mcu_data->ps_kvalue >> 16) & 0xFFFF) == 0xFFFF) {
			PS_canc1 = (mcu_data->ps_kvalue) & 0xFF;
			PS_canc2 = 0x00;
			CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc1,1);
			CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc2,1);
			PS_th1 = (mcu_data->ps_kvalue >>  8) & 0xFF;
			PS_th2 = 0x00;
			CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_PROXIMITY,&PS_th1,1);
			CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th2,1);
		} else {
			PS_canc1 = mcu_data->ps_kvalue & 0xFF;
			PS_canc2 = ((mcu_data->ps_kvalue) & 0xFF00) >> 8;
			CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc1,1);
			CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc2,1);
			PS_th1 = (mcu_data->ps_kvalue & 0xFF0000) >>  16;
			PS_th2 = (mcu_data->ps_kvalue & 0xFF000000) >>  24;
			CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_PROXIMITY,&PS_th1,1);
			CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th2,1);
		}
		D("Recover PS_canc1 = %d, PS_canc2 = %d, PS_th1 = %d PS_th2 = %d\n", PS_canc1, PS_canc2, PS_th1, PS_th2);
	}
	return count;
}

static int get_k_value_acc_f(struct device *dev, struct device_attribute *attr, char *buf){
       int i = 0;
       u8 data[3]={0};
       int data_temp[3]={0};
       if(CWMCU_Get_Calibrator(CW_ACCELERATION,data)>=0){
               for(i = 0;i<3;i++){
                       data_temp[i] = (int)((s8)data[i]);
               }
               return snprintf(buf, PAGE_SIZE, "%d %d %d\n",data_temp[0],data_temp[1],data_temp[2]);
       }
       return snprintf(buf, PAGE_SIZE, "%d %d %d\n",0,0,0);
}

static int get_k_value_accRL_f(struct device *dev, struct device_attribute *attr, char *buf){
	int i = 0;
	u8 data[12]={0};
	int data_temp[12]={0};
	if(CWMCU_i2c_read(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_RESULT_RL_ACC, data,12)>=0){
		for(i = 0;i<12;i++){
			data_temp[i] = (int)((s8)data[i]);
			D("data_temp[%d]: %d", i, data_temp[i]);
		}
		mcu_data->gs_kvalue_L1 =  data_temp[1] << 8 | data_temp[0];
		mcu_data->gs_kvalue_L2 =  data_temp[3] << 8 | data_temp[2];
		mcu_data->gs_kvalue_L3 =  data_temp[5] << 8 | data_temp[4];
		mcu_data->gs_kvalue_R1 =  data_temp[7] << 8 | data_temp[6];
		mcu_data->gs_kvalue_R2 =  data_temp[9] << 8 | data_temp[8];
		mcu_data->gs_kvalue_R3 =  data_temp[11] << 8 | data_temp[10];
		return snprintf(buf, PAGE_SIZE, "%d %d %d %d %d %d %d %d %d %d %d %d\n",
			data_temp[0],data_temp[1],data_temp[2],data_temp[3],data_temp[4],data_temp[5],
			data_temp[6],data_temp[7],data_temp[8],data_temp[9],data_temp[10],data_temp[11]);
	}
	return snprintf(buf, PAGE_SIZE, "%d %d %d %d %d %d %d %d %d %d %d %d\n",0,0,0,0,0,0,0,0,0,0,0,0);
}

static int AP_get_k_value_accRL_f(struct device *dev, struct device_attribute *attr, char *buf){

	return snprintf(buf, PAGE_SIZE, "%d %d %d %d %d %d\n",(s16)mcu_data->gs_kvalue_L1, (s16)mcu_data->gs_kvalue_L2, (s16)mcu_data->gs_kvalue_L3,
			(s16)mcu_data->gs_kvalue_R1, (s16)mcu_data->gs_kvalue_R2, (s16)mcu_data->gs_kvalue_R3);
}

static int get_k_value_mag_f(struct device *dev, struct device_attribute *attr, char *buf){
       int i = 0;
       u8 data_temp[26]={0};
       int data[26]={0};
       if(CWMCU_Get_Calibrator(CW_MAGNETIC,data_temp)>=0){
               for(i = 0;i<26;i++){
                       data[i] = (int)((s8)data_temp[i]);
               }
               return snprintf(buf, PAGE_SIZE, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n"
                       ,data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9]
                       ,data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18], data[19]
                       ,data[20], data[21], data[22], data[23], data[24], data[25]
               );
       }
       return snprintf(buf, PAGE_SIZE, "%d %d %d\n",0,0,0);
}
static int get_k_value_gyro_f(struct device *dev, struct device_attribute *attr, char *buf){
       int i = 0;
       u8 data[3]={0};
       int data_temp[3]={0};
       if(CWMCU_Get_Calibrator(CW_GYRO,data)>=0){
               for(i = 0;i<3;i++){
                       data_temp[i] = (int)((s8)data[i]);
               }
               return snprintf(buf, PAGE_SIZE, "%d %d %d\n",data_temp[0],data_temp[1],data_temp[2]);
       }
       return snprintf(buf, PAGE_SIZE, "%d %d %d\n",0,0,0);
}

static int get_k_value_light_f(struct device *dev, struct device_attribute *attr, char *buf){
	int i = 0;
	u8 data[4]={0};
	int data_temp[4]={0};

	if(CWMCU_Get_Calibrator(CW_LIGHT,data)>=0){
		for(i = 0;i<4;i++){
			data_temp[i] = (int)((s8)data[i]);
		}
		return snprintf(buf, PAGE_SIZE, "%x %x %x %x\n",data[0],data[1],data[2],data[3]);
		
	}
	return snprintf(buf, PAGE_SIZE, "%d %d\n",0,0);
}

static int get_k_value_proximity_f(struct device *dev, struct device_attribute *attr, char *buf){
       int i = 0;
       u8 data[4]={0};
       int data_temp[2]={0};
       if(CWMCU_Get_Calibrator(CW_PROXIMITY,data)>=0){
               for(i = 0;i<2;i++){
                       data_temp[i] = (int)((data[2*i+1] << 8) | data[2*i]);
               }
               return snprintf(buf, PAGE_SIZE, "%d %d\n",data_temp[0],data_temp[1]);
       }
       return snprintf(buf, PAGE_SIZE, "%d %d\n",0,0);
}

static int get_k_value_barometer_f(struct device *dev, struct device_attribute *attr, char *buf){
       int i = 0;
       u8 data[4]={0};
       int data_temp[4]={0};
       if(CWMCU_Get_Calibrator(CW_PRESSURE,data)>=0){
               for(i = 0;i<4;i++){
                       data_temp[i] = (int)((s8)data[i]);
               }
               return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n",data_temp[0],data_temp[1],data_temp[2],data_temp[3]);
       }
       return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n",0,0,0,0);
}

static int get_light_kadc(struct device *dev, struct device_attribute *attr, char *buf){
	u8 data[4]={0};
	u16 light_gadc[1]={0};
	u16 light_kadc[1]={0};
	CWMCU_i2c_read(mcu_data, LIGHT_SENSORS_CALIBRATION_DATA, data, 4);
	memcpy(light_gadc,&data[0],2);
	memcpy(light_kadc,&data[2],2);
	return snprintf(buf, PAGE_SIZE, "gadc = 0x%x, kadc = 0x%x",light_gadc[0], light_kadc[0]);
}

static int get_firmware_version(struct device *dev, struct device_attribute *attr, char *buf){
        u8 firmware_version[5] = {0};
        CWMCU_i2c_read(mcu_data, FIRMWARE_VERSION, firmware_version, 5);
        return snprintf(buf, PAGE_SIZE, "Firmware Architecture version %d, Sense version %d, Cywee lib version %d, Water number %d, Active Engine %d\n",
					firmware_version[0], firmware_version[1], firmware_version[2], firmware_version[3], firmware_version[4]);
}
static int get_barometer(struct device *dev, struct device_attribute *attr, char *buf){
        
	u8 data[127]={0};

	CWMCU_i2c_read(mcu_data, CWSTM32_READ_Pressure, data, 6);
	return snprintf(buf, PAGE_SIZE, "%x %x %x %x\n",data[0],data[1],data[2],data[3]);
}
static int get_ps_canc(struct device *dev, struct device_attribute *attr, char *buf){
	u16 PS_canc, PS_th;
	u8 data[4] = {0};
	int ret = 0;

	ret = CWMCU_i2c_read(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_PROXIMITY, data,4);
	PS_canc = (data[1] << 8) | data[0];
	PS_th   = (data[3] << 8) | data[2];
	D("INTE_PS1_CANC = (0x%04X),  INTE_PS2_CANC = (0x%04X)\n", PS_canc, PS_th);
	if (((mcu_data->ps_kheader & (0x50 << 24)) == (0x50 << 24)) && ((mcu_data->ps_kheader & (0x53 << 16)) == (0x53 << 16)))
		ret = snprintf(buf, PAGE_SIZE, "P-sensor calibrated,"
				"INTE_PS1_CANC = (0x%04X), "
				"INTE_PS2_CANC = (0x%04X)\n",
				PS_canc, PS_th);
	else
		ret = snprintf(buf, PAGE_SIZE, "P-sensor NOT calibrated,"
				"INTE_PS1_CANC = (0x%04X), "
				"INTE_PS2_CANC = (0x%04X)\n",
				PS_canc, PS_th);
	return ret;
}

static int get_proximity(struct device *dev, struct device_attribute *attr, char *buf){
	u8 data[3]={0};
	uint16_t data2;

	CWMCU_i2c_read(mcu_data, CWSTM32_READ_Proximity, data, 3);
	data2 = (data[2] << 8) | data[1];
	return snprintf(buf, PAGE_SIZE, "%x %x \n",data[0],data2);
}

static int get_proximity_polling(struct device *dev, struct device_attribute *attr, char *buf){
	u8 data[3]={0};
	uint16_t data2;
	u8 data_polling_enable = 0;
	data_polling_enable = CW_MCU_BIT_PROXIMITY_POLLING;
	CWMCU_i2c_write(mcu_data, PROXIMITY_SENSORS_STATUS, &data_polling_enable,1);
	CWMCU_i2c_read(mcu_data, CWSTM32_READ_Proximity, data, 3);
	data2 = (data[2] << 8) | data[1];

	return snprintf(buf, PAGE_SIZE, "ADC[0x%02X] status is %d\n",data2,data[0]);
}

static int get_light_polling(struct device *dev, struct device_attribute *attr, char *buf){
	u8 data[5] = {0};
	u8 data_polling_enable = 0;
	u32 light_adc[1]={0};

	data_polling_enable = CW_MCU_BIT_LIGHT_POLLING;
	CWMCU_i2c_write(mcu_data, LIGHT_SENSORS_STATUS, &data_polling_enable,1);

	if (mcu_data->ls_polling == 0) {
		CWMCU_i2c_read(mcu_data, CWSTM32_READ_Light, data, 3);
		memcpy(light_adc, &data[1], 2);
		return snprintf(buf, PAGE_SIZE, "ADC[0x%04X] => level %d\n",light_adc[0], data[0]);
	} else {
		CWMCU_i2c_read(mcu_data, CWSTM32_READ_Light, data, 5);
		memcpy(light_adc, &data[1], 4);
		return snprintf(buf, PAGE_SIZE, "ADC[0x%08X] => level %d\n",
				light_adc[0], data[0]);
	}
}


static int get_ls_mechanism(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", mcu_data->ls_polling);
}


static int read_mcu_data(struct device *dev,struct device_attribute *attr,const char *buf, size_t count){
        
	int i = 0;
	u8 data[2]={0};
	int data_temp[2]={0};
	u8 mcu_rdata[128] = {0};
	sscanf(buf, "%x %d\n", &data_temp[0],&data_temp[1]);
	for(i = 0;i<2;i++){
		data[i] = (u8)((s8)data_temp[i]);
	}
	CWMCU_i2c_read(mcu_data, data[0], mcu_rdata, data_temp[1]);
	for(i=0 ; i<data_temp[1]; i++){
		D("read mcu data register is %x , Byte%d  value is %x\n",data[0],i,mcu_rdata[i]);
	}
	return count;
}


static int CWMCU_i2c_write(struct CWMCU_data *sensor,
			  u8 reg_addr, u8 *data, u8 len)
{
	int dummy;
	int retry = 0;
	int i;

	mc_power_controller(1);
	#if USE_WAKE_MCU
	if(gpio_get_value(mcu_data->gpio_wake_mcu)){
		gpio_set_value(mcu_data->gpio_wake_mcu,0);
		
	}
	gpio_set_value(mcu_data->gpio_wake_mcu, 1);
	#endif
	for (i = 0; i < len; i++) {
		for(retry = 0; retry <= RETRY_TIMES ; retry++) {
			dummy = i2c_smbus_write_byte_data(sensor->client,
						  reg_addr, data[i]);
			if (dummy) {
				E("%s: i2c write error, retry = %d, dummy = %d\n",
					__func__, retry, dummy);
				mdelay(5);
				continue;
			} else
				break;
		}

		if (retry > RETRY_TIMES) {
			E("%s: retry over %d\n", __func__, RETRY_TIMES);
			cwmcu_i2c_error = 1;
			queue_delayed_work(mcu_wq, &exception_work_wdg, msecs_to_jiffies(atomic_read(&sensor->delay)));
			#if USE_WAKE_MCU
			gpio_set_value(mcu_data->gpio_wake_mcu, 0);
			#endif
			return -EIO;
		}
	}
	#if USE_WAKE_MCU
	gpio_set_value(mcu_data->gpio_wake_mcu, 0);
	#endif
	return 0;
}

static int CWMCU_set_sensor_kvalue(struct CWMCU_data *sensor)
{
	
	int ret;
	u8 GS_datax,GS_datay,GS_dataz;
	u8 GY_datax,GY_datay,GY_dataz;
	u8 ALS_goldh = (sensor->ALS_goldh == 0) ? 0x17 : (sensor->ALS_goldh);
	u8 ALS_goldl = (sensor->ALS_goldl == 0) ? 0x4D : (sensor->ALS_goldl);
	u8 ALS_datal, ALS_datah;
	u8 PS_canc1, PS_canc2, PS_th1, PS_th2;
	u8 BS_dataa,BS_datab,BS_datac,BS_datad;
	u8 firmware_version[4] = {0};
	sensor->gs_calibrated = 0;
	sensor->gy_calibrated = 0;
	sensor->ls_calibrated = 0;
	sensor->ps_calibrated = 0;
	sensor->bs_calibrated = 0;
	ret = CWMCU_i2c_read(mcu_data, FIRMWARE_VERSION, firmware_version, 4);
	D("Firmware Architecture version %d, Sense version %d, Cywee lib version %d, Water number %d",firmware_version[0],firmware_version[1],firmware_version[2],firmware_version[3]);
	if((sensor->gs_kvalue & (0x67 << 24)) == (0x67 << 24)){
		GS_datax = (sensor->gs_kvalue >> 16)& 0xFF;
		ret = CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_ACC,&GS_datax,1);
		GS_datay = (sensor->gs_kvalue >>  8)& 0xFF;
		ret = CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_ACC,&GS_datay,1);
		GS_dataz = (sensor->gs_kvalue)&0xFF;
		ret = CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_ACC,&GS_dataz,1);
		sensor->gs_calibrated = 1;
		D("Set g-sensor kvalue x is %x y is %x z is %x\n",GS_datax,GS_datay,GS_dataz);
	}

        if((sensor->gy_kvalue & (0x67 << 24)) == (0x67 << 24)){
		GY_datax = (sensor->gy_kvalue >> 16)& 0xFF;
		CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_GYRO,&GY_datax,1);
		GY_datay = (sensor->gy_kvalue >>  8)& 0xFF;
		CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_GYRO,&GY_datay,1);
		GY_dataz = (sensor->gy_kvalue)&0xFF;
		CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_GYRO,&GY_dataz,1);
		sensor->gy_calibrated = 1;
		D("Set gyro-sensor kvalue x is %x y is %x z is %x\n",GY_datax,GY_datay,GY_dataz);
        }

	if(((sensor->als_kvalue & (0x6D << 24)) == (0x6D << 24)) && ((sensor->als_kvalue & (0xA5 << 16)) == (0xA5 << 16))){
		u8 data[4] = {0};
		u8 cmp_data[4] = {0};
		int i, j;

		CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,&ALS_goldl,1);
		CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,&ALS_goldh,1);
		ALS_datal = (sensor->als_kvalue)&0xFF;
		CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,&ALS_datal,1);
		ALS_datah = (sensor->als_kvalue >>  8)& 0xFF;
		CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,&ALS_datah,1);
		sensor->ls_calibrated = 1;
		I("Set light-sensor kvalue is (0x%x, 0x%x), gold = (0x%x, 0x%x)"
		  "\n", ALS_datah, ALS_datal, ALS_goldh, ALS_goldl);

		cmp_data[0] = ALS_goldl;
		cmp_data[1] = ALS_goldh;
		cmp_data[2] = ALS_datal;
		cmp_data[3] = ALS_datah;

		if (CWMCU_Get_Calibrator(CW_LIGHT, data) >= 0) {
			I("%s: Re-read lightsensor kvalues = (0x%x, 0x%x,"
			  " 0x%x, 0x%x)\n", __func__, data[0], data[1],
			  data[2], data[3]);
			for (i = 0; i < 4; i++) {
				if (data[i] != cmp_data[i]) {
					I(
					  "%s: Found diff, i = %d, data = 0x%x,"
					  " cmp_data = 0x%x\n", __func__,
					  i, data[i], cmp_data[i]);
					  break;
				}
			}
			if (i < 4) {
				u8 one_to_four[4] = {1, 2, 3, 4};

				for (j = 0; j < 4; j++) {
					CWMCU_i2c_write(sensor,
						CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
						&one_to_four[j], 1);
				}

				CWMCU_Get_Calibrator(CW_LIGHT, data);
				for (j = 0; j < 4; j++) {
					if (data[j] == 4) {
						I(
						  "%s: Found '4', j = %d\n",
						  __func__, j);
						j++;
						break;
					}
				}

				
				for (; j < 4; j++) {
					CWMCU_i2c_write(sensor,
						CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
						&cmp_data[j], 1);
				}

				
				for (j = 0; j < 4; j++) {
					CWMCU_i2c_write(sensor,
						CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
						&cmp_data[j], 1);
				}
			} else {
				I("%s: lightsensor kvalues matched\n",
				  __func__);
			}
		}
	}

	if(((sensor->ps_kheader & (0x50 << 24)) == (0x50 << 24)) && ((sensor->ps_kheader & (0x53 << 16)) == (0x53 << 16))){
		if (((sensor->ps_kvalue >> 16) & 0xFFFF) == 0xFFFF) {
			PS_canc1 = (sensor->ps_kvalue) & 0xFF;
			PS_canc2 = 0x00;
			CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc1,1);
			CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc2,1);
			PS_th1 = (sensor->ps_kvalue >>  8) & 0xFF;
			PS_th2 = 0x00;
			CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th1,1);
			CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th2,1);
			D("Set proximity-sensor kvalue is %x %x %x %x\n", PS_canc1, PS_canc2, PS_th1, PS_th2);
		} else {
			PS_canc1 = sensor->ps_kvalue & 0xFF;
			PS_canc2 = ((sensor->ps_kvalue) & 0xFF00) >> 8;
			CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc1,1);
			CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc2,1);
			PS_th1 = (sensor->ps_kvalue & 0xFF0000) >> 16;
			PS_th2 = (sensor->ps_kvalue & 0xFF000000) >> 24;
			CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th1,1);
			CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th2,1);
			D("Set proximity-sensor kvalue is %x %x %x %x\n", PS_canc1, PS_canc2, PS_th1, PS_th2);

		}
		sensor->ps_calibrated = 1;
	}

	if(sensor->bs_kheader == 0x67){
		BS_dataa = (sensor->bs_kvalue >> 24)& 0xFF;
		CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE,&BS_dataa,1);
		BS_datab = (sensor->bs_kvalue >> 16)& 0xFF;
		CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE,&BS_datab,1);
		BS_datac = (sensor->bs_kvalue >> 8)& 0xFF;
		CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE,&BS_datac,1);
		BS_datad = (sensor->bs_kvalue)& 0xFF;
		CWMCU_i2c_write(sensor, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE,&BS_datad,1);
		sensor->bs_calibrated = 1;
		D("Set baro-sensor kvalue a is %x b is %x c is %x d is %x\n",BS_dataa,BS_datab,BS_datac,BS_datad);
	}
	D("Sensor calibration matrix is (gs %d gy %d ls %d ps %d bs %d)\n",sensor->gs_calibrated,sensor->gy_calibrated,sensor->ls_calibrated,sensor->ps_calibrated,sensor->bs_calibrated);
	return 0 ;
}


static int CWMCU_sensor_placement(struct CWMCU_data *sensor)
{
	int ret;
	D("Set Sensor Placement\n");
	ret = CWMCU_i2c_write(sensor, GENSOR_POSITION, &sensor->Acceleration_axes,1);
	ret=CWMCU_i2c_write(sensor, COMPASS_POSITION, &sensor->Magnetic_axes,1);
	ret=CWMCU_i2c_write(sensor, GYRO_POSITION, &sensor->Gyro_axes,1);
	if(ret<0){
		probe_i2c_fail =1;
	}
	return 0 ;
}
static void polling_do_work(struct work_struct *w)
{
	CWMCU_sensor_placement(mcu_data);
	CWMCU_set_sensor_kvalue(mcu_data);
	cancel_delayed_work(&polling_work);
}


static int CWMCU_i2c_read(struct CWMCU_data *sensor,
			 u8 reg_addr, u8 *data, u8 len)
{
	int retry = 0;
	#if USE_WAKE_MCU
	if(gpio_get_value(mcu_data->gpio_wake_mcu)){
		gpio_set_value(mcu_data->gpio_wake_mcu,0);
		
	}
	gpio_set_value(mcu_data->gpio_wake_mcu, 1);
	#endif
	for(retry = 0; retry <= RETRY_TIMES ; retry++) {
		if(i2c_smbus_read_i2c_block_data(sensor->client, reg_addr, len, data) > 0)
			break;
		else
			mdelay(5);
	}
	if (retry > RETRY_TIMES) {
		E("%s: retry over %d\n", __func__, RETRY_TIMES);
		cwmcu_i2c_error = 1;
		queue_delayed_work(mcu_wq, &exception_work_wdg, msecs_to_jiffies(atomic_read(&sensor->delay)));
		#if USE_WAKE_MCU
		gpio_set_value(mcu_data->gpio_wake_mcu, 0);
		#endif
		return -EIO;
	} else {
		#if USE_WAKE_MCU
		gpio_set_value(mcu_data->gpio_wake_mcu, 0);
		#endif
		return 0;
	}
}

#if defined(CONFIG_SYNC_TOUCH_STATUS)
int touch_status(u8 status){
    int ret = -1;

    if (probe_success != 1)
        return ret;

    if(status == 1 || status == 0){
        ret = CWMCU_i2c_write(mcu_data, TOUCH_STATUS_REGISTER, &status, 1);
        D("[TP][SensorHub] touch_status = %d\n", status);
    }
    return ret;
}
#endif

static int active_set(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
#if 0
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct CWMCU_data *sensor = (struct CWMCU_data *)i2c_get_clientdata(client);
#endif
	int enabled = 0;
	int sensors_id = 0;
	
	u8 data;
	u8 i;
	int retry = 0;
	int rc = 0;
	u8 data8[34] = {0};

	for (retry = 0; retry < ACTIVE_RETRY_TIMES; retry++) {
		if (mcu_data->resume_done != 1)
			I("%s: resume not completed, retry = %d\n", __func__, retry);
		else
			break;
	}
	if (retry >= ACTIVE_RETRY_TIMES) {
		I("%s: resume not completed, retry = %d, retry fails!\n", __func__, retry);
		return 0;
	}

 	sscanf(buf, "%d %d\n",&sensors_id, &enabled);
	I("%s++: sensors_id = %d, enabled = %d\n", __func__, sensors_id, enabled);
	if (probe_i2c_fail) {
		I("%s++: probe_i2c_fail retrun 0\n", __func__);
		return 0;
	}
	if ((sensors_id == Any_Motion) && power_key_pressed) {
		I("%s: Any_Motion && power_key_pressed\n", __func__);
		return count;
	}
	if ((sensors_id == Proximity) && (enabled == 0)) {
		if (mcu_data->proximity_debu_info == 1) {
			uint8_t mcu_data_p[4];

			CWMCU_i2c_read(mcu_data,
			    CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_PROXIMITY,
			    mcu_data_p,
			    sizeof(mcu_data_p));
			CWMCU_i2c_read(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_DEBUG_PROXIMITY,
				data8,
				sizeof(data8));
			I("%s: AutoK: Threshold = %d, SADC = %d, "
			  "CompensationValue = %d\n", __func__,
			  *(uint16_t*)&mcu_data_p[2], *(uint16_t*)&data8[8],
			  data8[10]);
			I("%s: AutoK: QueueIsEmpty = %d, Queue = %d %d %d %d\n",
			  __func__, data8[11], *(uint16_t*)&data8[0],
			  *(uint16_t*)&data8[2], *(uint16_t*)&data8[4],
			  *(uint16_t*)&data8[6]);
			I("%s: AutoK: Last ADC values (from old to new) = %d, "
			  "%d, %d, %d, %d, %d, %d, %d, %d, %d\n", __func__,
			  *(uint16_t*)&data8[12], *(uint16_t*)&data8[14],
			  *(uint16_t*)&data8[16], *(uint16_t*)&data8[18],
			  *(uint16_t*)&data8[20], *(uint16_t*)&data8[22],
			  *(uint16_t*)&data8[24], *(uint16_t*)&data8[26],
			  *(uint16_t*)&data8[28], *(uint16_t*)&data8[30]);
		} else {
			CWMCU_i2c_read(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_DEBUG_PROXIMITY, data8, 14);
			I("%s: AutoK: Threshold = %d, SADC = %d, CompensationValue = %d\n", __func__, *(u16*)&data8[10], *(u16*)&data8[8], data8[12]);
			I("%s: AutoK: QueueIsEmpty = %d, Queue = %d %d %d %d\n", __func__, data8[13], *(u16*)&data8[0], *(u16*)&data8[2], *(u16*)&data8[4], *(u16*)&data8[6]);
		}

	}
	if (sensors_id == Proximity) {
		if (enabled)
			p_status = 1;
		else
			p_status = 9;
	}

	if (sensors_id == Light) {
		if (!enabled)
			input_report_abs(mcu_data->input, ABS_MISC, -1);
		input_sync(mcu_data->input);

	}


	if ((enabled == 1) &&
	    (sensors_id < CW_SENSORS_ID_END) &&
	    (sensors_id >= 0)
	   ) {
		I("%s: Filter first ZEROs, sensors_id = 0x%x\n", __func__, sensors_id);
		mcu_data->filter_first_zeros[sensors_id] = 1;
	}

	mcu_data->enabled_list &= ~(1<<sensors_id);
	mcu_data->enabled_list |= ((uint32_t)enabled)<<sensors_id;

#if 0	
	if(sensor->enabled_list){
		gpio_set_value(mcu_data->gpio_wake_mcu,1);
		usleep_range(1 * 10000, 1 * 10000);

	}
	else{
		
		
	}
#endif

	i = sensors_id /8;
	data = (u8)(mcu_data->enabled_list>>(i*8));
#if 0 
	gpio_set_value(mcu_data->gpio_wake_mcu,1);
#endif

	for (retry = 0; retry < ENABLE_LIST_RETRY_TIMES; retry++) {
		u8 confirm_data;
		u8 compare_data = (u8)(mcu_data->enabled_list>>(i*8));

		rc = CWMCU_i2c_write(mcu_data, CWSTM32_ENABLE_REG+i,
					    &data,1);
		if (rc)
			E("%s: CWMCU_i2c_write fails, rc = %d, retry = %d\n",
			  __func__, rc, retry);

		rc = CWMCU_i2c_read(mcu_data, CWSTM32_ENABLE_REG+i,
				    &confirm_data, 1);
		if (rc < 0)
			E("%s: CWMCU_i2c_read fails, rc = %d, retry = %d\n",
			  __func__, rc, retry);

		I(
		  "%s: confirm_data = 0x%x, compare_data = 0x%x, retry = %d\n",
		  __func__, confirm_data, compare_data, retry);

		if (confirm_data == compare_data)
			break;
		else {
			E(
			  "%s: read and write mis-match, confirm_data = 0x%x,"
			  " compare_data = 0x%x, retry = %d\n", __func__,
			  confirm_data, compare_data, retry);
		}
	}
#if 0 
	if (UseWakeMcu)
		gpio_set_value(mcu_data->gpio_wake_mcu,0);
	else
		gpio_set_value(mcu_data->gpio_wake_mcu,1);
#endif

	if ((mcu_data->input != NULL) && (sensors_id == Proximity) && (enabled == 1)) {
		input_report_abs(mcu_data->input, ABS_DISTANCE, -1);
		I("%s: Report dummy -1 proximity event\n", __func__);
	}

	I("%s--: sensors_id = %d, enable = %d, enable_list = 0x%x\n",
		__func__, sensors_id, enabled, mcu_data->enabled_list);

	return count;
}

static int active_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#if 0
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct CWMCU_data *sensor = (struct CWMCU_data *)i2c_get_clientdata(client);
#endif
	return snprintf(buf, PAGE_SIZE, "%u\n",mcu_data->enabled_list);
}

static int interval_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (mcu_data && mcu_data->input_polled)
		return snprintf(buf, PAGE_SIZE, "%d\n", mcu_data->input_polled->poll_interval);
	else
		return snprintf(buf, PAGE_SIZE, "%d\n", CWMCU_POLL_INTERVAL);
}

static int interval_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
#if 0
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct CWMCU_data *sensor = (struct CWMCU_data *)i2c_get_clientdata(client);
#endif
	int val = 0;
	int sensors_id = 0;
	int rc = 0;
	u8 reg_addr = 0;
	u8 reg_value = 0;

	sscanf(buf, "%d %d\n",&sensors_id , &val);

	if ((sensors_id < 0) || (sensors_id >= numSensors)) {
		I("%s: sensors_id = %d, out of bound!\n",
				__func__, sensors_id);
		return 0;
	}
  
	if (val < CWMCU_POLL_MIN)
		val = CWMCU_POLL_MIN;
#if 0
	sensor->sensors_time[sensors_id] = 0;
	sensor->report_period[sensors_id] = val*1000;

	printk("CYWEE sensors_id=%d delay_ms=%d\n",sensors_id, sensor->report_period[sensors_id]);
#endif
        mcu_data->sensors_time[sensors_id] = 0;
        mcu_data->report_period[sensors_id] = val*1000;

	if ((sensors_id = CW_LIGHT) && mcu_data->ls_polling)
		mcu_data->report_period[sensors_id] = 1000*1000;

	switch (sensors_id) {
	case CW_ACCELERATION:
		reg_addr = ACCE_UPDATE_RATE;
		break;
	case CW_MAGNETIC:
		reg_addr = MAGN_UPDATE_RATE;
		break;
	case CW_GYRO:
		reg_addr = GYRO_UPDATE_RATE;
		break;
	case CW_ORIENTATION:
		reg_addr = ORIE_UPDATE_RATE;
		break;
	case CW_ROTATIONVECTOR:
		reg_addr = ROTA_UPDATE_RATE;
		break;
	case CW_LINEARACCELERATION:
		reg_addr = LINE_UPDATE_RATE;
		break;
	case CW_GRAVITY:
		reg_addr = GRAV_UPDATE_RATE;
		break;
	case CW_MAGNETIC_UNCALIBRATED:
		reg_addr = MAGN_UNCA_UPDATE_RATE;
		break;
	case CW_GYROSCOPE_UNCALIBRATED:
		reg_addr = GYRO_UNCA_UPDATE_RATE;
		break;
	case CW_GAME_ROTATION_VECTOR:
		reg_addr = GAME_ROTA_UPDATE_RATE;
		break;
	case CW_GEOMAGNETIC_ROTATION_VECTOR:
		reg_addr = GEOM_ROTA_UPDATE_RATE;
		break;
	case CW_SIGNIFICANT_MOTION:
		reg_addr = SIGN_UPDATE_RATE;
		break;
	default:
		reg_addr = 0;
		I("%s: Only reoprt_period changed, sensors_id = %d,"
			" delay_us = %6d\n",
			__func__, sensors_id,
			mcu_data->report_period[sensors_id]);
		return count;
	}
	switch (val) {
	case 200:
		reg_value = UPDATE_RATE_NORMAL;
		break;
	case 66:
		reg_value = UPDATE_RATE_UI;
		break;
	case 20:
		reg_value = UPDATE_RATE_GAME;
		break;
	case 10:
	case 16:
		reg_value = UPDATE_RATE_FASTEST;
		break;
	default:
		I("%s: Only reoprt_period changed, "
		  "val = %3d\n", __func__, val);
		return count;
	}

	D("%s: reg_addr = 0x%x, reg_value = 0x%x\n",
			__func__, reg_addr, reg_value);
	rc = CWMCU_i2c_write(mcu_data, reg_addr, &reg_value, 1);
	if (rc)
		E("%s: CWMCU_i2c_write fails, rc = %d\n", __func__, rc);

        I("%s: sensors_id = %d, delay_us = %6d\n",
		__func__, sensors_id, mcu_data->report_period[sensors_id]);

	return count;
}

static int barometer_self_test_set(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int self_test = 0;
	int rc = 0;
	u8 reg_value = 0;

	sscanf(buf, "%d\n", &self_test);

	if (self_test == 1) {
		reg_value = 0x20;
		rc = CWMCU_i2c_write(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_PRESSURE, &reg_value, 1);
		if (rc)
			E("%s: CWMCU_i2c_write fails, rc = %d\n", __func__, rc);
	} else {
		I("%s: Do nothing, self_test = %d\n", __func__, self_test);
	}

	return count;
}

static int barometer_self_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[2] = {0};
	int rc = 0;
	int self_test_pass = -1;

	rc = CWMCU_i2c_read(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_PRESSURE, data, 1);
	if (rc)
		E("%s: CWMCU_i2c_read fails, rc = %d\n", __func__, rc);

	D("%s: data = 0x%x\n", __func__, data[0]);

	if (data[0] & 0xC0)
		self_test_pass = 0;
	else
		self_test_pass = 1;

	return snprintf(buf, PAGE_SIZE, "%d\n", self_test_pass);
}


static int boot_mode_set(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	I("%s: No function!\n", __func__);
	return count;
}

static int boot_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (mcu_data != NULL) {
		I("%s: Boot mode = %d\n", __func__, mcu_data->mfg_mode);
		return snprintf(buf, PAGE_SIZE, "%d\n", mcu_data->mfg_mode);
	} else
		return snprintf(buf, PAGE_SIZE, "%d\n", -1);
}

#if 0
static DEVICE_ATTR(enable, 0666, active_show,
		   active_set);
static DEVICE_ATTR(delay_ms, 0666, interval_show,
		   interval_set);

static DEVICE_ATTR(calibrator_en, 0666, NULL, set_calibrator_en);
static DEVICE_ATTR(calibrator_status_acc, 0666, show_calibrator_status_acc, NULL);
static DEVICE_ATTR(calibrator_status_mag, 0666, show_calibrator_status_mag, NULL);
static DEVICE_ATTR(calibrator_status_gyro, 0666, show_calibrator_status_gyro, NULL);
static DEVICE_ATTR(calibrator_data_acc, 0666, get_k_value_acc_f, set_k_value_acc_f);
static DEVICE_ATTR(calibrator_data_mag, 0666, get_k_value_mag_f, set_k_value_mag_f);
static DEVICE_ATTR(calibrator_data_gyro, 0666, get_k_value_gyro_f, set_k_value_gyro_f);

static struct attribute *sysfs_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay_ms.attr,
	&dev_attr_calibrator_en.attr,
	&dev_attr_calibrator_status_acc.attr,
	&dev_attr_calibrator_status_mag.attr,
	&dev_attr_calibrator_status_gyro.attr,
	&dev_attr_calibrator_data_acc.attr,
	&dev_attr_calibrator_data_mag.attr,
	&dev_attr_calibrator_data_gyro.attr,
	NULL
};

static struct attribute_group sysfs_attribute_group = {
	.attrs = sysfs_attributes
};
#endif
static void __devinit CWMCU_init_input_device(struct CWMCU_data *sensor,struct input_dev *idev)	
{
	idev->name = CWMCU_I2C_NAME;
	
	idev->id.bustype = BUS_I2C;
	idev->dev.parent = &sensor->client->dev;

	idev->evbit[0] = BIT_MASK(EV_ABS);
	set_bit(EV_ABS, idev->evbit);
	input_set_capability(idev, EV_REL, HTC_Gesture_Motion);
	input_set_capability(idev, EV_REL, REL_Significant_Motion);
	input_set_capability(idev, EV_REL, HTC_Any_Motion);
	input_set_capability(idev, EV_REL, HTC_Matrix_Gesture);
	input_set_capability(idev, EV_REL, HTC_Gesture_Motion_HIDI);
	input_set_capability(idev, EV_REL, HTC_Matrix_Gesture_HIDI);

        input_set_abs_params(idev, ABS_DISTANCE, 0, 1, 0, 0);

	input_set_abs_params(idev, ABS_ACC_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_ACC_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_ACC_Z, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_MAG_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_MAG_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_MAG_Z, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_MAG_ACCURACY, -3, 3, 0, 0);
	input_set_abs_params(idev, ABS_GYRO_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_GYRO_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_GYRO_Z, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_LIGHT_Z, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_PRESSURE_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_PRESSURE_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_PRESSURE_Z, -DPS_MAX, DPS_MAX, 0, 0);	
	input_set_abs_params(idev, ABS_ORI_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_ORI_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_ORI_Z, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_ORI_ACCURACY, -3, 3, 0, 0);
	input_set_abs_params(idev, ABS_ROT_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_ROT_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_ROT_Z, -DPS_MAX, DPS_MAX, 0, 0);	
	input_set_abs_params(idev, ABS_LIN_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_LIN_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_LIN_Z, -DPS_MAX, DPS_MAX, 0, 0);	
	input_set_abs_params(idev, ABS_GRA_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_GRA_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_GRA_Z, -DPS_MAX, DPS_MAX, 0, 0);	
	input_set_abs_params(idev, ABS_PEDOMETER_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_PEDOMETER_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_PEDOMETER_Z, -DPS_MAX, DPS_MAX, 0, 0);	
	input_set_abs_params(idev, ABS_AIR_MOUSE_Z, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_BUFFERED_TRANSPORT_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_BUFFERED_TRANSPORT_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_BUFFERED_TRANSPORT_Z, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_REALTIME_TRANSPORT_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_REALTIME_TRANSPORT_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_REALTIME_TRANSPORT_Z, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_TRANSPORT_BUFFER_FULL, -DPS_MAX, DPS_MAX, 0, 0);

	input_set_abs_params(idev, ABS_MAGNETIC_UNCALIBRATED_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_MAGNETIC_UNCALIBRATED_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_MAGNETIC_UNCALIBRATED_Z, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_MAGNETIC_UNCALIBRATED_BIAS_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_MAGNETIC_UNCALIBRATED_BIAS_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_MAGNETIC_UNCALIBRATED_BIAS_Z, -DPS_MAX, DPS_MAX, 0, 0);

	input_set_abs_params(idev, ABS_GYROSCOPE_UNCALIBRATED_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_GYROSCOPE_UNCALIBRATED_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_GYROSCOPE_UNCALIBRATED_Z, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_GYROSCOPE_UNCALIBRATED_BIAS_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_GYROSCOPE_UNCALIBRATED_BIAS_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_GYROSCOPE_UNCALIBRATED_BIAS_Z, -DPS_MAX, DPS_MAX, 0, 0);

	input_set_abs_params(idev, ABS_GAME_ROTATION_VECTOR_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_GAME_ROTATION_VECTOR_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_GAME_ROTATION_VECTOR_Z, -DPS_MAX, DPS_MAX, 0, 0);

	input_set_abs_params(idev, ABS_GEOMAGNETIC_ROTATION_VECTOR_X, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_GEOMAGNETIC_ROTATION_VECTOR_Y, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_GEOMAGNETIC_ROTATION_VECTOR_Z, -DPS_MAX, DPS_MAX, 0, 0);

	input_set_abs_params(idev, ABS_STEP_DETECTOR, -DPS_MAX, DPS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_STEP_COUNTER, -DPS_MAX, DPS_MAX, 0, 0);

	input_set_abs_params(idev, ABS_MISC, 0, 9, 0, 0);
}

static s32 data_buff_ps[3]={0};
static void CWMCU_read(struct CWMCU_data *sensor)
{
	s32 ret = 0;
	u8 data[127]={0};
	s16 data_buff[6]={0};
	
        if(probe_i2c_fail){
		return;
        }
#if 0
    gpio_set_value(sensor->gpio_wake_mcu,1);
#endif

	do_gettimeofday(&sensor->now);
	sensor->time_diff= (sensor->now.tv_sec - sensor->previous.tv_sec) * 1000000
		    + (sensor->now.tv_usec - sensor->previous.tv_usec);
	sensor->previous.tv_sec = sensor->now.tv_sec;
	sensor->previous.tv_usec = sensor->now.tv_usec;

	if (DEBUG_FLAG_GSENSOR == 1)
		D("%s: time_diff = %d, enabled_list = 0x%x\n", __func__, sensor->time_diff, sensor->enabled_list);

	if (sensor->enabled_list & (1<<Acceleration)) {
		if (DEBUG_FLAG_GSENSOR == 1) {
			D("%s: sensor->sensors_time[Acceleration] = %d, sensor->report_period[Acceleration] = %d \n",
				__func__, sensor->sensors_time[Acceleration], sensor->report_period[Acceleration]);
		}

		if (sensor->sensors_time[Acceleration] >= sensor->report_period[Acceleration]) {
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_Acceleration, data, 6);
			data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
			data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
			data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);

			sensor->sensors_time[Acceleration] = sensor->sensors_time[Acceleration] - sensor->report_period[Acceleration];

			if ((sensor->filter_first_zeros[Acceleration] == 1) &&
			    ((data_buff[0] == -1) && (data_buff[1] == -1) && (data_buff[2] == -1))
			   ) {
				D("%s: Accelerometer(x, y, z) = (%d, %d, %d), Filtered\n",
					__func__, data_buff[0], data_buff[1], data_buff[2]);
			} else {
				input_report_abs(sensor->input, ABS_ACC_X, 1);
				input_report_abs(sensor->input, ABS_ACC_X, data_buff[0]);
				input_report_abs(sensor->input, ABS_ACC_Y, data_buff[1]);
				input_report_abs(sensor->input, ABS_ACC_Z, data_buff[2]);

				if (DEBUG_FLAG_GSENSOR == 1)
					D("g-sensor datax is %d datay is %d dataz is %d\n",data_buff[0],data_buff[1],data_buff[2]);

				input_sync(sensor->input);
				sensor->filter_first_zeros[Acceleration] = 0;
			}
		}
		if ((sensor->sensors_time[Acceleration] >= 0) &&
		    (sensor->sensors_time[Acceleration] < (2 * sensor->report_period[Acceleration]))
		   )
			sensor->sensors_time[Acceleration] += sensor->time_diff;
		else
			sensor->sensors_time[Acceleration] = 0;
	}

	if (sensor->enabled_list & (1<<Magnetic)) {
		if (sensor->sensors_time[Magnetic] >= sensor->report_period[Magnetic]) {
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_Magnetic, data, 6);
			data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
			data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
			data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);

			ret = CWMCU_i2c_read(sensor, CW_I2C_REG_SENSORS_ACCURACY_MAG, data, 1);
			data_buff[3] = (s16)data[0];

			sensor->sensors_time[Magnetic] = sensor->sensors_time[Magnetic] - sensor->report_period[Magnetic];

			if ((sensor->filter_first_zeros[Magnetic] == 1) &&
			    ((data_buff[0] == -1) && (data_buff[1] == -1) && (data_buff[2] == -1))
			   ) {
				D("%s: Magnetic(x, y, z) = (%d, %d, %d), Filtered\n",
					__func__, data_buff[0], data_buff[1], data_buff[2]);
			} else {
				input_report_abs(sensor->input, ABS_MAG_X, 1);
				input_report_abs(sensor->input, ABS_MAG_X, data_buff[0]);
				input_report_abs(sensor->input, ABS_MAG_Y, data_buff[1]);
				input_report_abs(sensor->input, ABS_MAG_Z, data_buff[2]);
				input_report_abs(sensor->input, ABS_MAG_ACCURACY, data_buff[3]);

				if(DEBUG_FLAG_COMPASS == 1) {
					D("compass datax is %d datay is %d dataz is %d, Accuracy = %d\n",
						data_buff[0], data_buff[1], data_buff[2], data_buff[3]);
				}

				input_sync(sensor->input);
				sensor->filter_first_zeros[Magnetic] = 0;
			}
		}
		if ((sensor->sensors_time[Magnetic] >= 0) &&
		    (sensor->sensors_time[Magnetic] < (2 * sensor->report_period[Magnetic]))
		   )
			sensor->sensors_time[Magnetic] += sensor->time_diff;
		else
			sensor->sensors_time[Magnetic] = 0;
	}

	if (sensor->enabled_list & (1<<Gyro)) {
		if (sensor->sensors_time[Gyro] >= sensor->report_period[Gyro]) {
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_Gyro, data, 6);
			data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
			data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
			data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);

			sensor->sensors_time[Gyro] = sensor->sensors_time[Gyro] - sensor->report_period[Gyro];

			if ((sensor->filter_first_zeros[Gyro] == 1) &&
			    ((data_buff[0] == -1) && (data_buff[1] == -1) && (data_buff[2] == -1))
			   ) {
				D("%s: Gyro(x, y, z) = (%d, %d, %d), Filtered\n",
					__func__, data_buff[0], data_buff[1], data_buff[2]);
			} else {
				input_report_abs(sensor->input, ABS_GYRO_X, 1);
				input_report_abs(sensor->input, ABS_GYRO_X, data_buff[0]);
				input_report_abs(sensor->input, ABS_GYRO_Y, data_buff[1]);
				input_report_abs(sensor->input, ABS_GYRO_Z, data_buff[2]);

				if(DEBUG_FLAG_GYRO == 1) {
				        D("gyro datax is %d datay is %d dataz is %d\n",data_buff[0],data_buff[1],data_buff[2]);
				}

				input_sync(sensor->input);
				sensor->filter_first_zeros[Gyro] = 0;
			}
		}
		if ((sensor->sensors_time[Gyro] >= 0) &&
		    (sensor->sensors_time[Gyro] < (2 * sensor->report_period[Gyro]))
		   )
			sensor->sensors_time[Gyro] += sensor->time_diff;
		else
			sensor->sensors_time[Gyro] = 0;
	}

	if (sensor->ls_polling && (sensor->enabled_list & (1<<Light))) {
		if (sensor->sensors_time[Light] >= sensor->report_period[Light]) {
			u32 *ls_data_buf;

			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_Light, data, 5);
			ls_data_buf = (u32 *)(&data[1]);
			sensor->sensors_time[Light] = sensor->sensors_time[Light] - sensor->report_period[Light];

			I("%s: light = %u\n", __func__, *ls_data_buf);

			input_report_abs(sensor->input, ABS_MISC, *ls_data_buf);
			input_sync(sensor->input);
		}
		if ((sensor->sensors_time[Light] >= 0) &&
		    (sensor->sensors_time[Light] < (2 * sensor->report_period[Light]))
		   )
			sensor->sensors_time[Light] += sensor->time_diff;
		else
			sensor->sensors_time[Light] = 0;
	}

	if (sensor->enabled_list & (1<<Proximity)) {
		if (sensor->sensors_time[Proximity] >= sensor->report_period[Proximity]) {
#if 0
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_Proximity, data, 2);
			
			
			
			sensor->sensors_time[Proximity] = sensor->sensors_time[Proximity] - sensor->report_period[Proximity];

			if ((sensor->filter_first_zeros[Proximity] == 1) &&
			    ((data_buff[0] == -1) && (data_buff[1] == -1) && (data_buff[2] == -1))
			   ) {
				D("%s: Proximity(x, y, z) = (%d, %d, %d), Filtered\n",
					__func__, data_buff[0], data_buff[1], data_buff[2]);
			} else {
				input_report_abs(sensor->input, ABS_DISTANCE, data[0]);
				printk("Proximity sensor data is %x\n",data[0]);
				
				
				
				input_sync(sensor->input);
				sensor->filter_first_zeros[Proximity] = 0;
			}
#endif
		}
		if ((sensor->sensors_time[Proximity] >= 0) &&
		    (sensor->sensors_time[Proximity] < (2 * sensor->report_period[Proximity]))
		   )
			sensor->sensors_time[Proximity] += sensor->time_diff;
		else
			sensor->sensors_time[Proximity] = 0;
	}

	if (sensor->enabled_list & (1<<Pressure)) {
		if (sensor->sensors_time[Pressure] >= sensor->report_period[Pressure]) {
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_Pressure, data, 6);
			data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
			data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
			data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);
			
			
			data_buff_ps[1] = (s32)(((u32)data[3] << 24) | ((u32)data[2] << 16 ) | ((u32)data[1] << 8) | ((u32)data[0]));
			if(abs(data_buff_ps[1] - data_buff_ps[0]) > 10000) {
				ret = CWMCU_i2c_read(sensor, CWSTM32_READ_Pressure, data, 6);
				data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
				data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
				data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);
				data_buff_ps[0] = (s32)(((u32)data[3] << 24) | ((u32)data[2] << 16 ) | ((u32)data[1] << 8) | ((u32)data[0]));
				if(DEBUG_FLAG_PRESSURE == 1) {
					D("[S_HUB][CW_MCU] Wrong barometer value is found! Last = %d, Retry = %d", data_buff_ps[1], data_buff_ps[0]);
				}
			} else {
				data_buff_ps[0] = data_buff_ps[1];
			}
			sensor->sensors_time[Pressure] = sensor->sensors_time[Pressure] - sensor->report_period[Pressure];
			input_report_abs(sensor->input, ABS_PRESSURE_Z, 1);
			input_report_abs(sensor->input, ABS_PRESSURE_X, data_buff_ps[0]);
			input_report_abs(sensor->input, ABS_PRESSURE_Y, data_buff[1]);
			input_report_abs(sensor->input, ABS_PRESSURE_Z, data_buff[2]);

			if(DEBUG_FLAG_PRESSURE == 1) {
				D("pressure datax is %x datay is %x dataz is %x data_buff_ps is %x\n",data_buff[0],data_buff[1],data_buff[2],data_buff_ps[0]);
				
			}

			input_sync(sensor->input);
		}
		if ((sensor->sensors_time[Pressure] >= 0) &&
		    (sensor->sensors_time[Pressure] < (2 * sensor->report_period[Pressure]))
		   )
			sensor->sensors_time[Pressure] += sensor->time_diff;
		else
			sensor->sensors_time[Pressure] = 0;
	}

	if (sensor->enabled_list & (1<<Orientation)) {
		if (sensor->sensors_time[Orientation] >= sensor->report_period[Orientation]) {
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_Orientation, data, 6);
			data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
			data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
			data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);

			ret = CWMCU_i2c_read(sensor, CW_I2C_REG_SENSORS_ACCURACY_MAG, data, 1);
			data_buff[3] = (s16)data[0];

			sensor->sensors_time[Orientation] = sensor->sensors_time[Orientation] - sensor->report_period[Orientation];

			if ((sensor->filter_first_zeros[Orientation] == 1) &&
			    (
			     ((data_buff[0] == 0) && (data_buff[1] == 0) && (data_buff[2] == 0))
			     ||
			     ((data_buff[0] == -1) && (data_buff[1] == -1) && (data_buff[2] == -1))
			    )
			   ) {
				D("%s: Orientation(0, 1, 2) = (%d, %d, %d), Filtered\n",
						__func__, data_buff[0], data_buff[1], data_buff[2]);
			} else {
				input_report_abs(sensor->input, ABS_ORI_X, 1);
				input_report_abs(sensor->input, ABS_ORI_X, data_buff[0]);
				input_report_abs(sensor->input, ABS_ORI_Y, data_buff[1]);
				input_report_abs(sensor->input, ABS_ORI_Z, data_buff[2]);
				input_report_abs(sensor->input, ABS_ORI_ACCURACY, data_buff[3]);
				if(DEBUG_FLAG_FUSION == 1) {
				        D("orientation datax is %d datay is %d dataz is %d, Accuracy = %d\n",
						data_buff[0], data_buff[1], data_buff[2], data_buff[3]);
				}

				input_sync(sensor->input);
				sensor->filter_first_zeros[Orientation] = 0;
			}
		}
		if ((sensor->sensors_time[Orientation] >= 0) &&
		    (sensor->sensors_time[Orientation] < (2 * sensor->report_period[Orientation]))
		   )
			sensor->sensors_time[Orientation] += sensor->time_diff;
		else
			sensor->sensors_time[Orientation] = 0;
	}

	if (sensor->enabled_list & (1<<RotationVector)) {
		if (sensor->sensors_time[RotationVector]>=sensor->report_period[RotationVector]){
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_RotationVector, data, 6);
			data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
			data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
			data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);

			sensor->sensors_time[RotationVector] = sensor->sensors_time[RotationVector] - sensor->report_period[RotationVector];

			if ((sensor->filter_first_zeros[RotationVector] == 1) &&
			    (
			     ((data_buff[0] == 0) && (data_buff[1] == 0) && (data_buff[2] == 0))
			     ||
			     ((data_buff[0] == -1) && (data_buff[1] == -1) && (data_buff[2] == -1))
			    )
			   ) {
				D("%s: RotationVector(0, 1, 2) = (%d, %d, %d), Filtered\n",
						__func__, data_buff[0], data_buff[1], data_buff[2]);
			} else {
				input_report_abs(sensor->input, ABS_ROT_X, 1);
				input_report_abs(sensor->input, ABS_ROT_X, data_buff[0]);
				input_report_abs(sensor->input, ABS_ROT_Y, data_buff[1]);
				input_report_abs(sensor->input, ABS_ROT_Z, data_buff[2]);
				if(DEBUG_FLAG_FUSION == 1) {
					D("rotationvector datax is %d datay is %d dataz is %d\n",data_buff[0],data_buff[1],data_buff[2]);
				}

				input_sync(sensor->input);
				sensor->filter_first_zeros[RotationVector] = 0;
			}
		}
		if ((sensor->sensors_time[RotationVector] >= 0) &&
		    (sensor->sensors_time[RotationVector] < (2 * sensor->report_period[RotationVector]))
		   )
			sensor->sensors_time[RotationVector] += sensor->time_diff;
		else
			sensor->sensors_time[RotationVector] = 0;
	}

	if (sensor->enabled_list & (1<<LinearAcceleration)) {
		if (sensor->sensors_time[LinearAcceleration] >= sensor->report_period[LinearAcceleration]){
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_LinearAcceleration, data, 6);
			data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
			data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
			data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);

			sensor->sensors_time[LinearAcceleration] = sensor->sensors_time[LinearAcceleration] - sensor->report_period[LinearAcceleration];

			if ((sensor->filter_first_zeros[LinearAcceleration] == 1) &&
			    (
			     ((data_buff[0] == 0) && (data_buff[1] == 0) && (data_buff[2] == 0))
			     ||
			     ((data_buff[0] == -1) && (data_buff[1] == -1) && (data_buff[2] == -1))
			    )
			   ) {
				D("%s: LinearAcceleration(0, 1, 2) = (%d, %d, %d), Filtered\n",
						__func__, data_buff[0], data_buff[1], data_buff[2]);
			} else {
				input_report_abs(sensor->input, ABS_LIN_X, 1);
				input_report_abs(sensor->input, ABS_LIN_X, data_buff[0]);
				input_report_abs(sensor->input, ABS_LIN_Y, data_buff[1]);
				input_report_abs(sensor->input, ABS_LIN_Z, data_buff[2]);

				if(DEBUG_FLAG_FUSION == 1) {
				        D("linearacceleration datax is %d datay is %d dataz is %d\n",data_buff[0],data_buff[1],data_buff[2]);
				}

				input_sync(sensor->input);
				sensor->filter_first_zeros[LinearAcceleration] = 0;
			}
		}
		if ((sensor->sensors_time[LinearAcceleration] >= 0) &&
		    (sensor->sensors_time[LinearAcceleration] < (2 * sensor->report_period[LinearAcceleration]))
		   )
			sensor->sensors_time[LinearAcceleration] += sensor->time_diff;
		else
			sensor->sensors_time[LinearAcceleration] = 0;
	}

	if (sensor->enabled_list & (1<<Gravity)) {
		if (sensor->sensors_time[Gravity] >= sensor->report_period[Gravity]) {
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_Gravity, data, 6);
			data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
			data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
			data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);

			sensor->sensors_time[Gravity] = sensor->sensors_time[Gravity] - sensor->report_period[Gravity];

			if ((sensor->filter_first_zeros[Gravity] == 1) &&
			    (
			     ((data_buff[0] == 0) && (data_buff[1] == 0) && (data_buff[2] == 0))
			     ||
			     ((data_buff[0] == -1) && (data_buff[1] == -1) && (data_buff[2] == -1))
			    )
			   ) {
				D("%s: Gravity(0, 1, 2) = (%d, %d, %d), Filtered\n",
						__func__, data_buff[0], data_buff[1], data_buff[2]);
			} else {
				input_report_abs(sensor->input, ABS_GRA_X, 1);
				input_report_abs(sensor->input, ABS_GRA_X, data_buff[0]);
				input_report_abs(sensor->input, ABS_GRA_Y, data_buff[1]);
				input_report_abs(sensor->input, ABS_GRA_Z, data_buff[2]);

				if(DEBUG_FLAG_FUSION == 1) {
				        D("gravity datax is %d datay is %d dataz is %d\n",data_buff[0],data_buff[1],data_buff[2]);
				}

				input_sync(sensor->input);
				sensor->filter_first_zeros[Gravity] = 0;
			}
		}
		if ((sensor->sensors_time[Gravity] >= 0) &&
		    (sensor->sensors_time[Gravity] < (2 * sensor->report_period[Gravity]))
		   )
			sensor->sensors_time[Gravity] += sensor->time_diff;
		else
			sensor->sensors_time[Gravity] = 0;
	}

	if (sensor->enabled_list & (1<<Pedometer)) {
		if (sensor->sensors_time[Pedometer] >= sensor->report_period[Pedometer]) {
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_Pedometer, data, 6);
			data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
			data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
			data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);

			sensor->sensors_time[Pedometer] = sensor->sensors_time[Pedometer] - sensor->report_period[Pedometer];
			input_report_abs(sensor->input, ABS_PEDOMETER_X, data_buff[0]);
			input_report_abs(sensor->input, ABS_PEDOMETER_Y, data_buff[1]);
			input_report_abs(sensor->input, ABS_PEDOMETER_Z, data_buff[2]);
			input_sync(sensor->input);
		}
		if ((sensor->sensors_time[Pedometer] >= 0) &&
		    (sensor->sensors_time[Pedometer] < (2 * sensor->report_period[Pedometer]))
		   )
			sensor->sensors_time[Pedometer] += sensor->time_diff;
		else
			sensor->sensors_time[Pedometer] = 0;
	}

	if (sensor->enabled_list & (1<<Air_Mouse)) {
		if(sensor->sensors_time[Air_Mouse] >= sensor->report_period[Air_Mouse]) {
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_Air_Mouse, data, 6);
			data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
			data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
			data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);

			sensor->sensors_time[Air_Mouse] = sensor->sensors_time[Air_Mouse] -sensor->report_period[Air_Mouse];

			input_report_abs(sensor->input, ABS_AIR_MOUSE_Z, data_buff[2]);
			input_sync(sensor->input);
		}
		if ((sensor->sensors_time[Air_Mouse] >= 0) &&
		    (sensor->sensors_time[Air_Mouse] < (2 * sensor->report_period[Air_Mouse]))
		   )
			sensor->sensors_time[Air_Mouse] += sensor->time_diff;
		else
			sensor->sensors_time[Air_Mouse] = 0;
	}

#if 0
	if (sensor->enabled_list & (1<<Gesture_Motion)) {
		if(sensor->sensors_time[Gesture_Motion]>=sensor->report_period[Gesture_Motion]) {
			sensor->sensors_time[Gesture_Motion] = 0;
			input_report_rel(sensor->input, HTC_Gesture_Motion,1);
			input_report_rel(sensor->input, HTC_Gesture_Motion,0);
			input_sync(sensor->input);
		}
		if ((sensor->sensors_time[Gesture_Motion] >= 0) &&
		    (sensor->sensors_time[Gesture_Motion] < (2 * sensor->report_period[Gesture_Motion]))
		   )
			sensor->sensors_time[Gesture_Motion] += sensor->time_diff;
                else
			sensor->sensors_time[Gesture_Motion] = 0;
	}

#endif

	if (sensor->enabled_list & (1 << Buffered_Transport)) {
		if (sensor->sensors_time[Buffered_Transport]>=sensor->report_period[Buffered_Transport]) {
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_SENSOR_BUFFERED_TRANSPORT, data, 6);
			data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
			data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
			data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);
			sensor->sensors_time[Buffered_Transport] = sensor->sensors_time[Buffered_Transport] - sensor->report_period[Buffered_Transport];

			if (ret >= 0) {
				input_report_abs(sensor->input, ABS_BUFFERED_TRANSPORT_X, data_buff[0]);
				input_report_abs(sensor->input, ABS_BUFFERED_TRANSPORT_Y, data_buff[1]);
				input_report_abs(sensor->input, ABS_BUFFERED_TRANSPORT_Z, data_buff[2]);
				input_sync(sensor->input);

				if (DEBUG_FLAG_BUFFERED_TRANSPORT == 1) {
					D("%s: data_buff(0, 1, 2) = (%d, %d, %d), ret = %d, read address = 0x%X\n",
						__func__, data_buff[0], data_buff[1], data_buff[2], ret, CWSTM32_READ_SENSOR_BUFFERED_TRANSPORT);
				}
			} 
				
		}
		if ((sensor->sensors_time[Buffered_Transport] >= 0) &&
		    (sensor->sensors_time[Buffered_Transport] < (2 * sensor->report_period[Buffered_Transport]))
		   )
			sensor->sensors_time[Buffered_Transport] += sensor->time_diff;
		else
			sensor->sensors_time[Buffered_Transport] = 0;
	}

	if (sensor->enabled_list & (1 << Realtime_Transport)) {
		if (sensor->sensors_time[Realtime_Transport]>=sensor->report_period[Realtime_Transport]) {
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_SENSOR_REALTIME_TRANSPORT, data, 6);
			data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
			data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
			data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);
			sensor->sensors_time[Realtime_Transport] = sensor->sensors_time[Realtime_Transport] - sensor->report_period[Realtime_Transport];

			if (ret >= 0) {
				input_report_abs(sensor->input, ABS_REALTIME_TRANSPORT_X, data_buff[0]);
				input_report_abs(sensor->input, ABS_REALTIME_TRANSPORT_Y, data_buff[1]);
				input_report_abs(sensor->input, ABS_REALTIME_TRANSPORT_Z, data_buff[2]);
				input_sync(sensor->input);

				if (DEBUG_FLAG_REALTIME_TRANSPORT == 1) {
					D("%s: data_buff(0, 1, 2) = (%d, %d, %d), ret = %d, read address = 0x%X\n",
						__func__, data_buff[0], data_buff[1], data_buff[2], ret, CWSTM32_READ_SENSOR_REALTIME_TRANSPORT);
				}
			} 
				
		}
		if ((sensor->sensors_time[Realtime_Transport] >= 0) &&
		    (sensor->sensors_time[Realtime_Transport] < (2 * sensor->report_period[Realtime_Transport]))
		   )
			sensor->sensors_time[Realtime_Transport] += sensor->time_diff;
		else
			sensor->sensors_time[Realtime_Transport] = 0;
	}

	if (sensor->enabled_list & (1 << Magnetic_Uncalibrated)) {
		if (sensor->sensors_time[Magnetic_Uncalibrated]>=sensor->report_period[Magnetic_Uncalibrated]) {
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_MAGNETIC_UNCALIBRATED, data, 12);
			data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
			data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
			data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);
			data_buff[3] = (s16)(((u16)data[7] << 8) | (u16)data[6]);
			data_buff[4] = (s16)(((u16)data[9] << 8) | (u16)data[8]);
			data_buff[5] = (s16)(((u16)data[11] << 8) | (u16)data[10]);
			sensor->sensors_time[Magnetic_Uncalibrated] = sensor->sensors_time[Magnetic_Uncalibrated] - sensor->report_period[Magnetic_Uncalibrated];

			if (ret >= 0) {
				if ((sensor->filter_first_zeros[Magnetic_Uncalibrated] == 1) &&
				    (data_buff[0] == 0) &&
				    (data_buff[1] == 0) &&
				    (data_buff[2] == 0) &&
				    (data_buff[3] == 0) &&
				    (data_buff[4] == 0) &&
				    (data_buff[5] == 0)) {
					D("%s: Magn_Uncalib(0, 1, 2, 3, 4, 5) = (%d, %d, %d, %d, %d, %d), Filtered\n",
							__func__, data_buff[0], data_buff[1], data_buff[2], data_buff[3], data_buff[4], data_buff[5]);
				} else {
					input_report_abs(sensor->input, ABS_MAGNETIC_UNCALIBRATED_X, 1);
					input_report_abs(sensor->input, ABS_MAGNETIC_UNCALIBRATED_X, data_buff[0]);
					input_report_abs(sensor->input, ABS_MAGNETIC_UNCALIBRATED_Y, data_buff[1]);
					input_report_abs(sensor->input, ABS_MAGNETIC_UNCALIBRATED_Z, data_buff[2]);
					input_report_abs(sensor->input, ABS_MAGNETIC_UNCALIBRATED_BIAS_X, data_buff[3]);
					input_report_abs(sensor->input, ABS_MAGNETIC_UNCALIBRATED_BIAS_Y, data_buff[4]);
					input_report_abs(sensor->input, ABS_MAGNETIC_UNCALIBRATED_BIAS_Z, data_buff[5]);
					input_sync(sensor->input);
					sensor->filter_first_zeros[Magnetic_Uncalibrated] = 0;

					if (DEBUG_FLAG_MAGNETIC_UNCALIBRATED == 1) {
						D("%s: data_buff(0, 1, 2, 3, 4, 5) = (%d, %d, %d, %d, %d, %d), ret = %d, read address = 0x%X\n",
							__func__, data_buff[0], data_buff[1], data_buff[2], data_buff[3], data_buff[4], data_buff[5], ret, CWSTM32_READ_MAGNETIC_UNCALIBRATED);
					}
				}
			} 
				
		}
		if ((sensor->sensors_time[Magnetic_Uncalibrated] >= 0) &&
		    (sensor->sensors_time[Magnetic_Uncalibrated] < (2 * sensor->report_period[Magnetic_Uncalibrated]))
		   )
			sensor->sensors_time[Magnetic_Uncalibrated] += sensor->time_diff;
		else
			sensor->sensors_time[Magnetic_Uncalibrated] = 0;
	}

	if (sensor->enabled_list & (1 << Gyroscope_Uncalibrated)) {
		if (sensor->sensors_time[Gyroscope_Uncalibrated]>=sensor->report_period[Gyroscope_Uncalibrated]) {
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_GYROSCOPE_UNCALIBRATED, data, 12);
			data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
			data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
			data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);
			data_buff[3] = (s16)(((u16)data[7] << 8) | (u16)data[6]);
			data_buff[4] = (s16)(((u16)data[9] << 8) | (u16)data[8]);
			data_buff[5] = (s16)(((u16)data[11] << 8) | (u16)data[10]);
			sensor->sensors_time[Gyroscope_Uncalibrated] = sensor->sensors_time[Gyroscope_Uncalibrated] - sensor->report_period[Gyroscope_Uncalibrated];

			if (ret >= 0) {
				if ((sensor->filter_first_zeros[Gyroscope_Uncalibrated] == 1) &&
				    (data_buff[0] == 0) &&
				    (data_buff[1] == 0) &&
				    (data_buff[2] == 0) &&
				    (data_buff[3] == 0) &&
				    (data_buff[4] == 0) &&
				    (data_buff[5] == 0)) {
					D("%s: Gyro_Uncalib(0, 1, 2, 3, 4, 5) = (%d, %d, %d, %d, %d, %d), Filtered\n",
							__func__, data_buff[0], data_buff[1], data_buff[2], data_buff[3], data_buff[4], data_buff[5]);
				} else {
					input_report_abs(sensor->input, ABS_GYROSCOPE_UNCALIBRATED_X, 1);
					input_report_abs(sensor->input, ABS_GYROSCOPE_UNCALIBRATED_X, data_buff[0]);
					input_report_abs(sensor->input, ABS_GYROSCOPE_UNCALIBRATED_Y, data_buff[1]);
					input_report_abs(sensor->input, ABS_GYROSCOPE_UNCALIBRATED_Z, data_buff[2]);
					input_report_abs(sensor->input, ABS_GYROSCOPE_UNCALIBRATED_BIAS_X, data_buff[3]);
					input_report_abs(sensor->input, ABS_GYROSCOPE_UNCALIBRATED_BIAS_Y, data_buff[4]);
					input_report_abs(sensor->input, ABS_GYROSCOPE_UNCALIBRATED_BIAS_Z, data_buff[5]);
					input_sync(sensor->input);
					sensor->filter_first_zeros[Gyroscope_Uncalibrated] = 0;
				}
			} 
				
		}
		if ((sensor->sensors_time[Gyroscope_Uncalibrated] >= 0) &&
		    (sensor->sensors_time[Gyroscope_Uncalibrated] < (2 * sensor->report_period[Gyroscope_Uncalibrated]))
		   )
			sensor->sensors_time[Gyroscope_Uncalibrated] += sensor->time_diff;
		else
			sensor->sensors_time[Gyroscope_Uncalibrated] = 0;
	}

	if (sensor->enabled_list & (1<<Game_Rotation_Vector)) {
		if(sensor->sensors_time[Game_Rotation_Vector]>=sensor->report_period[Game_Rotation_Vector]){
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_GAME_ROTATION_VECTOR, data, 6);
			data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
			data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
			data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);
			sensor->sensors_time[Game_Rotation_Vector] = sensor->sensors_time[Game_Rotation_Vector] -sensor->report_period[Game_Rotation_Vector];

			if (ret >= 0) {
				if ((sensor->filter_first_zeros[Game_Rotation_Vector] == 1) &&
				    (
				     ((data_buff[0] == 0) && (data_buff[1] == 0) && (data_buff[2] == 0))
				     ||
				     ((data_buff[0] == -1) && (data_buff[1] == -1) && (data_buff[2] == -1))
				    )
				   ) {
					D("%s: Game_RV(0, 1, 2) = (%d, %d, %d), Filtered\n",
							__func__, data_buff[0], data_buff[1], data_buff[2]);
				} else {
					input_report_abs(sensor->input, ABS_GAME_ROTATION_VECTOR_X, 1);
					input_report_abs(sensor->input, ABS_GAME_ROTATION_VECTOR_X, data_buff[0]);
					input_report_abs(sensor->input, ABS_GAME_ROTATION_VECTOR_Y, data_buff[1]);
					input_report_abs(sensor->input, ABS_GAME_ROTATION_VECTOR_Z, data_buff[2]);
					input_sync(sensor->input);
					sensor->filter_first_zeros[Game_Rotation_Vector] = 0;
				}
			} 
				
		}
		if ((sensor->sensors_time[Game_Rotation_Vector] >= 0) &&
		    (sensor->sensors_time[Game_Rotation_Vector] < (2 * sensor->report_period[Game_Rotation_Vector]))
		   )
			sensor->sensors_time[Game_Rotation_Vector] += sensor->time_diff;
		else
			sensor->sensors_time[Game_Rotation_Vector] = 0;
	}

	if (sensor->enabled_list & (1<<Geomagnetic_Rotation_Vector)) {
		if(sensor->sensors_time[Geomagnetic_Rotation_Vector]>=sensor->report_period[Geomagnetic_Rotation_Vector]){
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_GEOMAGNETIC_ROTATION_VECTOR, data, 6);
			data_buff[0] = (s16)(((u16)data[1] << 8) | (u16)data[0]);
			data_buff[1] = (s16)(((u16)data[3] << 8) | (u16)data[2]);
			data_buff[2] = (s16)(((u16)data[5] << 8) | (u16)data[4]);
			sensor->sensors_time[Geomagnetic_Rotation_Vector] = sensor->sensors_time[Geomagnetic_Rotation_Vector] -sensor->report_period[Geomagnetic_Rotation_Vector];

			if (ret >= 0) {
				if ((sensor->filter_first_zeros[Geomagnetic_Rotation_Vector] == 1) &&
				    (
				     ((data_buff[0] == 0) && (data_buff[1] == 0) && (data_buff[2] == 0))
				     ||
				     ((data_buff[0] == -1) && (data_buff[1] == -1) && (data_buff[2] == -1))
				    )
				   ) {
					D("%s: Geo_RV(0, 1, 2) = (%d, %d, %d), Filtered\n",
							__func__, data_buff[0], data_buff[1], data_buff[2]);
				} else {
					input_report_abs(sensor->input, ABS_GEOMAGNETIC_ROTATION_VECTOR_X, 1);
					input_report_abs(sensor->input, ABS_GEOMAGNETIC_ROTATION_VECTOR_X, data_buff[0]);
					input_report_abs(sensor->input, ABS_GEOMAGNETIC_ROTATION_VECTOR_Y, data_buff[1]);
					input_report_abs(sensor->input, ABS_GEOMAGNETIC_ROTATION_VECTOR_Z, data_buff[2]);
					input_sync(sensor->input);
					sensor->filter_first_zeros[Geomagnetic_Rotation_Vector] = 0;
					if (DEBUG_FLAG_GEOMAGNETIC_ROTATION_VECTOR == 1) {
						D("%s: Geo_RV(0, 1, 2) = (%d, %d, %d), ret = %d, read address = 0x%X\n",
							__func__, data_buff[0], data_buff[1], data_buff[2], ret, CWSTM32_READ_GEOMAGNETIC_ROTATION_VECTOR);
					}
				}
			} 
				
		}
		if ((sensor->sensors_time[Geomagnetic_Rotation_Vector] >= 0) &&
		    (sensor->sensors_time[Geomagnetic_Rotation_Vector] < (2 * sensor->report_period[Geomagnetic_Rotation_Vector]))
		   )
			sensor->sensors_time[Geomagnetic_Rotation_Vector] += sensor->time_diff;
		else
			sensor->sensors_time[Geomagnetic_Rotation_Vector] = 0;
	}

#if 0
	if (UseWakeMcu)
		gpio_set_value(mcu_data->gpio_wake_mcu,0);
	else
		gpio_set_value(mcu_data->gpio_wake_mcu,1);
#endif
}

static void CWMCU_poll(struct input_polled_dev *dev)
{
	CWMCU_read(dev->private);
}

static int CWMCU_open(struct CWMCU_data *sensor)
{
	int error;

	error = pm_runtime_get_sync(&sensor->client->dev);
	if (error && error != -ENOSYS)
		return error;
	
	return 0;
}

static void CWMCU_close(struct CWMCU_data *sensor)
{
	pm_runtime_put_sync(&sensor->client->dev);
}

static void CWMCU_poll_open(struct input_polled_dev *ipoll_dev)
{
	struct CWMCU_data *sensor = ipoll_dev->private;
	
	CWMCU_open(sensor);
}

static void CWMCU_poll_close(struct input_polled_dev *ipoll_dev)
{
	struct CWMCU_data *sensor = ipoll_dev->private;

	CWMCU_close(sensor);
}

static int __devinit CWMCU_register_polled_device(struct CWMCU_data *sensor)
{
	struct input_polled_dev *ipoll_dev;
	int error;

	ipoll_dev = input_allocate_polled_device();
	if (!ipoll_dev)
		return -ENOMEM;
	mcu_data->input_polled = ipoll_dev;
	ipoll_dev->private = sensor;
	ipoll_dev->open = CWMCU_poll_open;
	ipoll_dev->close = CWMCU_poll_close;
	ipoll_dev->poll = CWMCU_poll;
	ipoll_dev->poll_interval = CWMCU_POLL_INTERVAL;
	ipoll_dev->poll_interval_min = CWMCU_POLL_MIN;
	ipoll_dev->poll_interval_max = CWMCU_POLL_MAX;

	CWMCU_init_input_device(sensor, ipoll_dev->input);

	error = input_register_polled_device(ipoll_dev);
	if (error) {
		input_free_polled_device(ipoll_dev);
		return error;
	}

	sensor->input_polled = ipoll_dev;
	sensor->input = ipoll_dev->input;

	return 0;
}


static int CWMCU_suspend(struct device *dev)
{
	uint8_t data = 0x00;
	D("[CWMCU] %s\n",__func__);
#if USE_WAKE_MCU
	UseWakeMcu = 1;
	CWMCU_i2c_write(mcu_data, HTC_SYSTEM_STATUS_REG, &data, 1);
	gpio_set_value(mcu_data->gpio_wake_mcu,0);
#endif
	cancel_delayed_work(&touch_log_work);
	mcu_data->resume_done = 0;

	return 0;
}

static void resume_do_work(struct work_struct *w)
{
	uint8_t data = 0x01;

	I("[CWMCU] %s++\n",__func__);

	
	CWMCU_i2c_write(mcu_data, HTC_SYSTEM_STATUS_REG, &data, 1);
	UseWakeMcu = 0;

	mcu_data->resume_done = 1;

	I("[CWMCU] %s--\n",__func__);
}

static int CWMCU_resume(struct device *dev)
{
	
	I("[CWMCU] %s++\n",__func__);

#if USE_WAKE_MCU
	queue_work(mcu_wq, &resume_work);
	queue_delayed_work(mcu_wq, &touch_log_work, msecs_to_jiffies(TOUCH_LOG_DELAY));
#endif
	I("[CWMCU] %s--\n",__func__);
	return 0;
}

static struct gpiomux_setting gpio_reset_output_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting gpio_reset_input_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static void exception_do_work_wdg(struct work_struct *w)
{
	struct CWMCU_data *sensor = mcu_data;
	D("[CWMCU] %s: %d\n",__func__, cwmcu_wdg_reset);

	

	if (cwmcu_wdg_reset == 1) {
		if (cwmcu_i2c_error == 1) {
			cwmcu_dump_call_stack(MAX_BACKUP_CALL_STACK_SIZE);
		}
		cwmcu_wdg_reset = 0;
		cwmcu_i2c_error = 0;
	} else {
		if (sensor->gpio_reset) {
			msm_gpiomux_write(sensor->gpio_reset, GPIOMUX_ACTIVE,
				&gpio_reset_output_config, NULL);

			gpio_direction_output(sensor->gpio_reset, 0);
			D("%s: set gpio_reset = [%d] to LOW, value = %d\n", __func__,
				sensor->gpio_reset, gpio_get_value_cansleep(sensor->gpio_reset));

			mdelay(10);

			msm_gpiomux_write(sensor->gpio_reset, GPIOMUX_ACTIVE,
				&gpio_reset_input_config, NULL);
			D("%s: set gpio_reset = [%d] to original config I(PU),"
			  " config.func = %d, config.dir"
			  " = %d, config.pull = %d, value = %d\n",
				__func__, sensor->gpio_reset,
				gpio_reset_input_config.func, gpio_reset_input_config.dir,
				gpio_reset_input_config.pull, gpio_get_value_cansleep(sensor->gpio_reset));

			msleep(5);
			CWMCU_set_sensor_kvalue(sensor);
			if (cwmcu_i2c_error == 1) {
				cwmcu_dump_call_stack(MAX_BACKUP_CALL_STACK_SIZE);
			}
			cwmcu_i2c_error = 0;
			
			
		} else {
			I("%s: Reset pin is not set by DT or Platform data,"
			  " sensor->gpio_reset = %d\n", __func__,
				sensor->gpio_reset);
		}
	}
}


static int reset_hub_set(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf,
			 size_t count)
{
	return count;
}

static int reset_hub_show(struct device *dev,
			  struct device_attribute *attr,
			  char *buf)
{
	int rc = 0;
	u8 data;

	if (mcu_data->gpio_reset) {
		msm_gpiomux_write(mcu_data->gpio_reset, GPIOMUX_ACTIVE,
			&gpio_reset_output_config, NULL);

		gpio_direction_output(mcu_data->gpio_reset, 0);
		D("%s: set gpio_reset = [%d] to LOW, value = %d\n", __func__,
			mcu_data->gpio_reset,
			gpio_get_value_cansleep(mcu_data->gpio_reset));

		mdelay(10);

		msm_gpiomux_write(mcu_data->gpio_reset, GPIOMUX_ACTIVE,
			&gpio_reset_input_config, NULL);
		D("%s: set gpio_reset = [%d] to original config I(PU),"
		  " config.func = %d, config.dir"
		  " = %d, config.pull = %d, value = %d\n",
			__func__, mcu_data->gpio_reset,
			gpio_reset_input_config.func,
			gpio_reset_input_config.dir,
			gpio_reset_input_config.pull,
			gpio_get_value_cansleep(mcu_data->gpio_reset));

		data = (u8)(mcu_data->enabled_list >> 8);
		rc = CWMCU_i2c_write(mcu_data, CWSTM32_ENABLE_REG, &data, 1);
		if (rc < 0) {
			E("%s: i2c_write fail, rc = %d\n", __func__, rc);
			return -EIO;
		}
	}

	return snprintf(buf, PAGE_SIZE, "rc = %d\n", rc);
}


static void touch_log_do_work(struct work_struct *w)
{
	struct CWMCU_data *sensor = mcu_data;
	int ret = 0;
	u8 data[32] = {0};
	u32 *TouchActiveCount = NULL;
	u32 *TouchActiveAndTouched = NULL;
	u32 *TotalTouchPointCount = NULL;
	u8 TouchActiveOF = 0;
	u8 TouchActiveAndTouchedOF = 0;
	u8 TotalTouchPointOF = 0;

	ret = CWMCU_i2c_read(sensor, CWSTM32_READ_Touch_Log, data, 15);
	TouchActiveCount = (u32*)(&data[0]);
	TouchActiveAndTouched = (u32*)(&data[4]);
	TotalTouchPointCount = (u32*)(&data[8]);
	TouchActiveOF = data[12];
	TouchActiveAndTouchedOF = data[13];
	TotalTouchPointOF = data[14];

	if (ret >= 0) {
		D("%s: TouchActiveCount = %d, TouchActiveAndTouched = %d, TotalTouchPointCount = %d\n"
		  "    TouchActiveOF = %d, TouchActiveAndTouchedOF = %d, TotalTouchPointOF = %d\n",
			__func__, *TouchActiveCount, *TouchActiveAndTouched, *TotalTouchPointCount,
			TouchActiveOF, TouchActiveAndTouchedOF, TotalTouchPointOF);
	} else
		D("%s: CWSTM32_READ_Touch_Log: ret = %d\n", __func__, ret);
}

static void cwmcu_irq_work_func(struct work_struct *work)
{
	struct CWMCU_data *sensor = container_of((struct work_struct *)work,struct CWMCU_data, irq_work);
	s32 ret = 0;
	u8 data[127]={0};
	s16 data_buff[3]={0};
	int retry;
	s32 data_event = 0;
	u8 INT_st1 = 0, INT_st2 = 0, INT_st3 = 0, INT_st4 = 0, ERR_st = 0;
	u8 clear_intr = 0xFF;
	u16 light_adc[1]={0};
	D("[CWMCU] %s\n",__func__);
	

	if(sensor->input == NULL ) {
		E("[CWMCU] sensor->input == NULL\n");
		return;
	}

	CWMCU_i2c_read(sensor, CWSTM32_INT_ST1, data, 1);
	INT_st1 = data[0];

	CWMCU_i2c_read(sensor, CWSTM32_INT_ST2, data, 1);
	INT_st2 = data[0];

	CWMCU_i2c_read(sensor, CWSTM32_INT_ST3, data, 1);
	INT_st3 = data[0];

	CWMCU_i2c_read(sensor, CWSTM32_INT_ST4, data, 1);
	INT_st4 = data[0];

	CWMCU_i2c_read(sensor, CWSTM32_ERR_ST, data, 1);
	ERR_st = data[0];

        D("%s: INT_st(1, 2, 3, 4, 5) = (0x%x, 0x%x, 0x%x, 0x%x, 0x%x)\n",
                __func__, INT_st1, INT_st2, INT_st3, INT_st4, ERR_st);
	
	if (INT_st1 & CW_MCU_INT_BIT_PROXIMITY) {
		if(sensor->enabled_list & (1<<Proximity)){
			if (sensor->proximity_debu_info == 1)
				ret = CWMCU_i2c_read(sensor, CWSTM32_READ_Proximity, data, 6);
			else
				ret = CWMCU_i2c_read(sensor, CWSTM32_READ_Proximity, data, 2);
			if(data[0] < 2){
				sensor->sensors_time[Proximity] = sensor->sensors_time[Proximity] -sensor->report_period[Proximity];
				p_status = data[0];
				input_report_abs(sensor->input, ABS_DISTANCE, data[0]);
				input_sync(sensor->input);
				if (sensor->proximity_debu_info == 1) {
					I("Proximity interrupt occur value is %d adc is"
					  " %d thre is %d comp is %d ps_calibration is "
					  "%d\n", data[0], *(uint16_t *)&data[1],
					  *(uint16_t *)&data[3], data[5],
					  sensor->ps_calibrated);
				} else {
					D("Proximity interrupt occur value is %d adc is %x ps_calibration is %d\n",data[0],data[1],sensor->ps_calibrated);
				}
			} else {
				D("Proximity interrupt occur value is %d adc is %x ps_calibration is %d (message only)\n",
				  data[0],data[1],sensor->ps_calibrated);
			}
		}
		if(data[0] < 2){
			clear_intr = CW_MCU_INT_BIT_PROXIMITY;
			ret = CWMCU_i2c_write(sensor, CWSTM32_INT_ST1, &clear_intr, 1);
			CWMCU_i2c_write(sensor, CWSTM32_READ_Proximity, &data[0], 1);
		}
	}

	
	if (INT_st1 & CW_MCU_INT_BIT_LIGHT) {
		if(sensor->enabled_list & (1<<Light)){
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_Light, data, 3);
			if(data[0] < 11){
				sensor->sensors_time[Light] = sensor->sensors_time[Light] -sensor->report_period[Light];
				memcpy(light_adc,&data[1],2);

				if (sensor->ls_polling )
					input_report_abs(sensor->input, ABS_MISC, light_adc[0]);
				else
					input_report_abs(sensor->input, ABS_MISC, data[0]);
				input_sync(sensor->input);

				D("Light interrupt occur value is %d, adc is %x ls_calibration is %d\n",data[0],light_adc[0],sensor->ls_calibrated);
			} else {
				D("Light interrupt occur value is %d, adc is %x ls_calibration is %d (message only)\n",
				  data[0],light_adc[0],sensor->ls_calibrated);
			}
		}
		if(data[0] < 11){
			clear_intr = CW_MCU_INT_BIT_LIGHT;
			ret = CWMCU_i2c_write(sensor, CWSTM32_INT_ST1, &clear_intr, 1);
		}
	}

	
	if (INT_st3 & CW_MCU_INT_BIT_SIGNIFICANT_MOTION) {
		if (sensor->enabled_list & (1<<Significant_Motion)) {
			sensor->sensors_time[Significant_Motion] = 0;

			wake_lock_timeout(&significant_wake_lock, 1 * HZ);
			input_report_rel(sensor->input,	REL_Significant_Motion, 1);
			input_sync(sensor->input);

			D("%s: Significant Motion interrupt occurs!!\n", __func__);
		}
		clear_intr = CW_MCU_INT_BIT_SIGNIFICANT_MOTION;
		ret = CWMCU_i2c_write(sensor, CWSTM32_INT_ST3, &clear_intr, 1);
	}

	
	if (INT_st3 & CW_MCU_INT_BIT_STEP_DETECTOR) {
		if (sensor->enabled_list & (1<<Step_Detector)) {
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_STEP_DETECTOR, data, 1);
			if (ret >= 0) {
				sensor->sensors_time[Step_Detector] = 0;

				input_report_abs(sensor->input,	ABS_STEP_DETECTOR, -1);
				input_report_abs(sensor->input,	ABS_STEP_DETECTOR, data[0]);
				input_sync(sensor->input);

				D("%s: Step Detector interrupt occurs, timestamp = %d\n", __func__, data[0]);
			} else {
				I("%s: Step Detector i2c read fails, ret = %d\n", __func__, ret);
			}
		}
		clear_intr = CW_MCU_INT_BIT_STEP_DETECTOR;
		ret = CWMCU_i2c_write(sensor, CWSTM32_INT_ST3, &clear_intr, 1);
	}

	
	if (INT_st3 & CW_MCU_INT_BIT_STEP_COUNTER) {
		if (sensor->enabled_list & (1<<Step_Counter)) {
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_STEP_COUNTER, data, 1);
			if (ret >= 0) {
				sensor->sensors_time[Step_Counter] = 0;

				input_report_abs(sensor->input,	ABS_STEP_COUNTER, -1);
				input_report_abs(sensor->input,	ABS_STEP_COUNTER, data[0]);
				input_sync(sensor->input);

				D("%s: Step Counter interrupt occurs, step = %d\n", __func__, data[0]);
			} else {
				I("%s: Step Counter i2c read fails, ret = %d\n", __func__, ret);
			}
		}
		clear_intr = CW_MCU_INT_BIT_STEP_COUNTER;
		ret = CWMCU_i2c_write(sensor, CWSTM32_INT_ST3, &clear_intr, 1);
	}

	
	if (INT_st4 & CW_MCU_INT_BIT_HTC_GESTURE_MOTION) {
		for(retry = 0; retry < 100 ; retry++){
			ret = i2c_smbus_read_i2c_block_data(sensor->client, CWSTM32_READ_Gesture_Motion, 6, data);
			if(ret == 6) {
				break;
			}
			mdelay(5);
		}
		D("[CWMCU] i2c bus read %d bytes\n", ret);
		data_event = (s32)((data[0] & 0x1F) | (((data[1] | (data[2] << 8)) & 0x3FF) << 5) | (data[3] << 15) | (data[4] << 23));
		if (vib_trigger) {
			if (data[0] == 14) {
				vib_trigger_event(vib_trigger, VIB_TIME);
				D("Gesture motion detected, vibrate for %d ms!\n", VIB_TIME);
			}
			else if(data[0] == 6 || data[0] == 15 || data[0] == 18 || data[0] == 19 || data[0] == 24 || data[0] == 25 || data[0] == 26 || data[0] == 27) {
				vib_trigger_event(vib_trigger, VIB_TIME);
				sensor->sensors_time[Gesture_Motion] = 0;
				input_report_rel(sensor->input, HTC_Gesture_Motion,data_event);
				input_sync(sensor->input);
				power_key_pressed = 0;
				D("[CWMCU][vib_trigger] Gesture_Motion: df0: %d, d0: %d, d1: %d\n", data_buff[0], data[0], data[1]);
				D("[CWMCU][vib_trigger] Gesture_Motion: data_buff: %d, data_event: %d\n", data_buff[1], data_event);
				D("[CWMCU][vib_trigger] Gesture_Motion input sync\n");
			}
			else {
                                sensor->sensors_time[Gesture_Motion] = 0;
                                input_report_rel(sensor->input, HTC_Gesture_Motion,data_event);
                                input_sync(sensor->input);
                                power_key_pressed = 0;
                                D("[CWMCU][disable vib_trigger] Gesture_Motion: df0: %d, d0: %d, d1: %d\n", data_buff[0], data[0], data[1]);
                                D("[CWMCU][disable vib_trigger] Gesture_Motion: data_buff: %d, data_event: %d\n", data_buff[1], data_event);
                                D("[CWMCU][disable vib_trigger] Gesture_Motion input sync\n");
			}
		} else {
			sensor->sensors_time[Gesture_Motion] = 0;
			input_report_rel(sensor->input, HTC_Gesture_Motion,data_event);
			input_sync(sensor->input);
			power_key_pressed = 0;
			D("[CWMCU] Gesture_Motion: df0: %d, d0: %d, d1: %d\n", data_buff[0], data[0], data[1]);
			D("[CWMCU] Gesture_Motion: data_buff: %d, data_event: %d\n", data_buff[1], data_event);
			D("[CWMCU] Gesture_Motion input sync\n");
		}
		clear_intr = CW_MCU_INT_BIT_HTC_GESTURE_MOTION;
		ret = CWMCU_i2c_write(sensor, CWSTM32_INT_ST4, &clear_intr, 1);
	}

	
	if (INT_st4 & CW_MCU_INT_BIT_TRANSPORT_BUFFER_FULL) {
		if (sensor->enabled_list & (1 << Transport_Buffer_Full)) {
			u32 active_engine_data = 0;
			u16 *time_stamp = NULL;
			ret = CWMCU_i2c_read(sensor, CWSTM32_READ_SENSOR_TRANSPORT_BUFFER_FULL, data, 4);
			if (ret != 0)
				E("%s: Active Engine read fails, ret = %d\n", __func__, ret);

			active_engine_data = (data[0] & 0xFF) | ((data[1] << 8) & 0xFF00) | ((data[2] << 16) & 0xFF0000) | ((data[3] << 24) & 0xFF000000);
			D("ACTIVE_ENGINE, active_engine_data = 0x%x\n", active_engine_data);
			time_stamp = (u16 *)(&data[2]);
			sensor->sensors_time[Transport_Buffer_Full] = sensor->sensors_time[Transport_Buffer_Full] - sensor->report_period[Transport_Buffer_Full];
			input_report_abs(sensor->input, SENSOR_TRANSPORT_BUFFER_FULL, active_engine_data);
			input_sync(sensor->input);
			D("ACTIVE_ENGINE interrupt occur, (Cmd, P1, P2, Timestamp) = (0x%x, %d, %d, %d)\n", (data[0] & 0xF), ((data[0] >> 4) & 0xF), data[1], *time_stamp);
		}
		clear_intr = CW_MCU_INT_BIT_TRANSPORT_BUFFER_FULL;
		ret = CWMCU_i2c_write(sensor, CWSTM32_INT_ST4, &clear_intr, 1);
	}

	
	if (INT_st4 & CW_MCU_INT_BIT_ANY_MOTION) {
	    if (sensor->enabled_list & (1<<Any_Motion)) {
			for(retry = 0; retry < 100 ; retry++){
				ret = i2c_smbus_read_i2c_block_data(sensor->client, CWSTM32_READ_Any_Motion, 1, data);
				if(ret == 1) {
				    break;
				}
				mdelay(5);
			}
			sensor->sensors_time[Any_Motion] = 0;
			input_report_rel(sensor->input,	HTC_Any_Motion,data[0]);
			input_sync(sensor->input);
			D("[CWMCU] Any_Motion: %d\n", data[0]);
	    }
		clear_intr = CW_MCU_INT_BIT_ANY_MOTION;
		ret = CWMCU_i2c_write(sensor, CWSTM32_INT_ST4, &clear_intr, 1);
	}

	
	if (INT_st4 & CW_MCU_INT_BIT_MATRIX_GESTURE) {
		for (retry = 0; retry < 100 ; retry++) {
			ret = i2c_smbus_read_i2c_block_data(sensor->client, CWSTM32_READ_Matrix_Gesture, 6, data);
			if (ret == 6)
				break;
			mdelay(5);
		}
		D("[CWMCU]CW_MCU_INT_BIT_MATRIX_GESTURE: i2c bus read %d bytes\n", ret);
		data_event = (s32)((data[0] & 0x1F) | (((data[1] | (data[2] << 8)) & 0x3FF) << 5) | (data[3] << 15) | (data[4] << 23));
		if (vib_trigger) {
			if (data[0] == 14) {
				vib_trigger_event(vib_trigger, VIB_TIME);
				D("Matrix Gesture detected, vibrate for %d ms!\n", VIB_TIME);
			} else if(data[0] == 6 || data[0] == 15 || data[0] == 18 || data[0] == 19 || data[0] == 24 || data[0] == 25 || data[0] == 26 || data[0] == 27) {
				vib_trigger_event(vib_trigger, VIB_TIME);
				sensor->sensors_time[Matrix_Gesture] = 0;
				input_report_rel(sensor->input, HTC_Matrix_Gesture, data_event);
				input_sync(sensor->input);
				power_key_pressed = 0;
				D("[CWMCU][vib_trigger] Matrix_Gesture: df0: %d, d0: %d, d1: %d\n", data_buff[0], data[0], data[1]);
				D("[CWMCU][vib_trigger] Matrix_Gesture: data_buff: %d, data_event: %d\n", data_buff[1], data_event);
				D("[CWMCU][vib_trigger] Matrix_Gesture input sync\n");
			} else {
                                sensor->sensors_time[Matrix_Gesture] = 0;
                                input_report_rel(sensor->input, HTC_Matrix_Gesture, data_event);
                                input_sync(sensor->input);
                                power_key_pressed = 0;
                                D("[CWMCU][disable vib_trigger] Matrix_Gesture: df0: %d, d0: %d, d1: %d\n", data_buff[0], data[0], data[1]);
                                D("[CWMCU][disable vib_trigger] Matrix_Gesture: data_buff: %d, data_event: %d\n", data_buff[1], data_event);
                                D("[CWMCU][disable vib_trigger] Matrix_Gesture input sync\n");
			}
		} else {
			sensor->sensors_time[Matrix_Gesture] = 0;
			input_report_rel(sensor->input, HTC_Matrix_Gesture, data_event);
			input_sync(sensor->input);
			power_key_pressed = 0;
			D("[CWMCU] Matrix_Gesture: df0: %d, d0: %d, d1: %d\n", data_buff[0], data[0], data[1]);
			D("[CWMCU] Matrix_Gesture: data_buff: %d, data_event: %d\n", data_buff[1], data_event);
			D("[CWMCU] Matrix_Gesture input sync\n");
		}
		clear_intr = CW_MCU_INT_BIT_MATRIX_GESTURE;
		ret = CWMCU_i2c_write(sensor, CWSTM32_INT_ST4, &clear_intr, 1);
	}

	
	if (INT_st4 & CW_MCU_INT_BIT_HTC_GESTURE_MOTION_HIDI) {
		for (retry = 0; retry < 100 ; retry++) {
			ret = i2c_smbus_read_i2c_block_data(sensor->client, CWSTM32_READ_Gesture_Motion_HIDI, 6, data);
			if (ret == 6)
				break;
			mdelay(5);
		}
		D("[CWMCU]CW_MCU_INT_BIT_HTC_GESTURE_MOTION_HIDI: i2c bus read %d bytes\n", ret);
		data_event = (s32)((data[0] & 0x1F) | (((data[1] | (data[2] << 8)) & 0x3FF) << 5) | (data[3] << 15) | (data[4] << 23));
		if (vib_trigger) {
			if (data[0] == 14) {
				vib_trigger_event(vib_trigger, VIB_TIME);
				D("Gesture motion HIDI detected, vibrate for %d ms!\n", VIB_TIME);
			} else if(data[0] == 6 || data[0] == 15 || data[0] == 18 || data[0] == 19 || data[0] == 24 || data[0] == 25 || data[0] == 26 || data[0] == 27) {
				vib_trigger_event(vib_trigger, VIB_TIME);
				sensor->sensors_time[Gesture_Motion_HIDI] = 0;
				input_report_rel(sensor->input, HTC_Gesture_Motion_HIDI, data_event);
				input_sync(sensor->input);
				power_key_pressed = 0;
				D("[CWMCU][vib_trigger] Gesture_Motion_HIDI: df0: %d, d0: %d, d1: %d\n", data_buff[0], data[0], data[1]);
				D("[CWMCU][vib_trigger] Gesture_Motion_HIDI: data_buff: %d, data_event: %d\n", data_buff[1], data_event);
				D("[CWMCU][vib_trigger] Gesture_Motion_HIDI input sync\n");
			} else {
                                sensor->sensors_time[Gesture_Motion_HIDI] = 0;
                                input_report_rel(sensor->input, HTC_Gesture_Motion_HIDI, data_event);
                                input_sync(sensor->input);
                                power_key_pressed = 0;
                                D("[CWMCU][disable vib_trigger] Gesture_Motion_HIDI: df0: %d, d0: %d, d1: %d\n", data_buff[0], data[0], data[1]);
                                D("[CWMCU][disable vib_trigger] Gesture_Motion_HIDI: data_buff: %d, data_event: %d\n", data_buff[1], data_event);
                                D("[CWMCU][disable vib_trigger] Gesture_Motion_HIDI input sync\n");
			}
		} else {
			sensor->sensors_time[Gesture_Motion_HIDI] = 0;
			input_report_rel(sensor->input, HTC_Gesture_Motion_HIDI, data_event);
			input_sync(sensor->input);
			power_key_pressed = 0;
			D("[CWMCU] Gesture_Motion_HIDI: df0: %d, d0: %d, d1: %d\n", data_buff[0], data[0], data[1]);
			D("[CWMCU] Gesture_Motion_HIDI: data_buff: %d, data_event: %d\n", data_buff[1], data_event);
			D("[CWMCU] Gesture_Motion_HIDI input sync\n");
		}
		clear_intr = CW_MCU_INT_BIT_HTC_GESTURE_MOTION_HIDI;
		ret = CWMCU_i2c_write(sensor, CWSTM32_INT_ST4, &clear_intr, 1);
	}

	
	if (INT_st4 & CW_MCU_INT_BIT_HTC_MATRIX_GESTURE_HIDI) {
		for (retry = 0; retry < 100 ; retry++) {
			ret = i2c_smbus_read_i2c_block_data(sensor->client, CWSTM32_READ_Matrix_Gesture_HIDI, 6, data);
			if (ret == 6)
				break;
			mdelay(5);
		}
		D("[CWMCU]CW_MCU_INT_BIT_HTC_MATRIX_GESTURE_HIDI: i2c bus read %d bytes\n", ret);
		data_event = (s32)((data[0] & 0x1F) | (((data[1] | (data[2] << 8)) & 0x3FF) << 5) | (data[3] << 15) | (data[4] << 23));
		if (vib_trigger) {
			if (data[0] == 14) {
				vib_trigger_event(vib_trigger, VIB_TIME);
				D("Matrix gesture HIDI detected, vibrate for %d ms!\n", VIB_TIME);
			} else if(data[0] == 6 || data[0] == 15 || data[0] == 18 || data[0] == 19 || data[0] == 24 || data[0] == 25 || data[0] == 26 || data[0] == 27) {
				vib_trigger_event(vib_trigger, VIB_TIME);
				sensor->sensors_time[Matrix_Gesture_HIDI] = 0;
				input_report_rel(sensor->input, HTC_Matrix_Gesture_HIDI, data_event);
				input_sync(sensor->input);
				power_key_pressed = 0;
				D("[CWMCU][vib_trigger] Matrix_Gesture_HIDI: df0: %d, d0: %d, d1: %d\n", data_buff[0], data[0], data[1]);
				D("[CWMCU][vib_trigger] Matrix_Gesture_HIDI: data_buff: %d, data_event: %d\n", data_buff[1], data_event);
				D("[CWMCU][vib_trigger] Matrix_Gesture_HIDI input sync\n");
			} else {
                                sensor->sensors_time[Matrix_Gesture_HIDI] = 0;
                                input_report_rel(sensor->input, HTC_Matrix_Gesture_HIDI, data_event);
                                input_sync(sensor->input);
                                power_key_pressed = 0;
                                D("[CWMCU][disable vib_trigger] Matrix_Gesture_HIDI: df0: %d, d0: %d, d1: %d\n", data_buff[0], data[0], data[1]);
                                D("[CWMCU][disable vib_trigger] Matrix_Gesture_HIDI: data_buff: %d, data_event: %d\n", data_buff[1], data_event);
                                D("[CWMCU][disable vib_trigger] Matrix_Gesture_HIDI input sync\n");
			}
		} else {
			sensor->sensors_time[Matrix_Gesture_HIDI] = 0;
			input_report_rel(sensor->input, HTC_Matrix_Gesture_HIDI, data_event);
			input_sync(sensor->input);
			power_key_pressed = 0;
			D("[CWMCU] Matrix_Gesture_HIDI: df0: %d, d0: %d, d1: %d\n", data_buff[0], data[0], data[1]);
			D("[CWMCU] Matrix_Gesture_HIDI: data_buff: %d, data_event: %d\n", data_buff[1], data_event);
			D("[CWMCU] Matrix_Gesture_HIDI input sync\n");
		}
		clear_intr = CW_MCU_INT_BIT_HTC_MATRIX_GESTURE_HIDI;
		ret = CWMCU_i2c_write(sensor, CWSTM32_INT_ST4, &clear_intr, 1);
	}

	
	if (ERR_st & CW_MCU_INT_BIT_ERROR_MCU_EXCEPTION) {
		E("[CWMCU] MCU Exception \n");
		cwmcu_wdg_reset = 0;
		cwmcu_i2c_error = 0;
		queue_delayed_work(mcu_wq, &exception_work_wdg, msecs_to_jiffies(atomic_read(&sensor->delay)));
		cwmcu_dump_call_stack(MAX_CALL_STACK_SIZE);
		clear_intr = CW_MCU_INT_BIT_ERROR_MCU_EXCEPTION;
		ret = CWMCU_i2c_write(sensor, CWSTM32_ERR_ST, &clear_intr, 1);
	}

	
	if (ERR_st & CW_MCU_INT_BIT_ERROR_WATCHDOG_RESET) {
		E("[CWMCU] Watch Dog Reset \n");
		msleep(5);
		clear_intr = CW_MCU_INT_BIT_ERROR_WATCHDOG_RESET;
		ret = CWMCU_i2c_write(sensor, CWSTM32_ERR_ST, &clear_intr, 1);
		cwmcu_wdg_reset = 1;
	}
	enable_irq(sensor->IRQ);
}

static irqreturn_t cwmcu_irq_handler(int irq, void *handle)
{
    struct CWMCU_data *data = handle;

    if (probe_success != 1) {
        I("%s: probe not completed\n", __func__);
        return IRQ_HANDLED;
    }

    if (data)
        disable_irq_nosync(data->IRQ);

    if (data == NULL)
	return IRQ_HANDLED;
    if (data->client == NULL)
	return IRQ_HANDLED;
    wake_lock_timeout(&ges_wake_lock, 2 * HZ);
    
    schedule_work(&data->irq_work);

    return IRQ_HANDLED;
}

static int mcu_parse_dt(struct device *dev, struct CWMCU_data *pdata)
{
        struct property *prop = NULL;
        struct device_node *dt = dev->of_node;
        u32 buf = 0;
        struct device_node *g_sensor_offset = NULL;
        int g_sensor_cali_size = 0;
        unsigned char *g_sensor_cali_data = NULL;
	struct device_node *gyro_sensor_offset = NULL;
	int gyro_sensor_cali_size = 0;
	unsigned char *gyro_sensor_cali_data = NULL;
	struct device_node *light_sensor_offset = NULL;
	int light_sensor_cali_size = 0;
	unsigned char *light_sensor_cali_data = NULL;
	struct device_node *p_sensor_offset = NULL;
	int p_sensor_cali_size = 0;
	unsigned char *p_sensor_cali_data = NULL;
	struct device_node *baro_sensor_offset = NULL;
	int baro_sensor_cali_size = 0;
	unsigned char *baro_sensor_cali_data = NULL;

        int i = 0;
        if (pdata == NULL) {
                D("%s: pdata is NULL\n", __func__);
                return -EINVAL;
        }
	pdata->gs_kvalue = 0;
	if ((g_sensor_offset = of_find_node_by_path(CALIBRATION_DATA_PATH))) {
		g_sensor_cali_data = (unsigned char*) of_get_property(g_sensor_offset, G_SENSOR_FLASH_DATA, &g_sensor_cali_size);
		D("%s: cali_size = %d\n", __func__, g_sensor_cali_size);
		if (g_sensor_cali_data) {
			for (i = 0; (i < g_sensor_cali_size) && (i < 4); i++) {
			printk("g sensor cali_data[%d] = %02x \n", i, g_sensor_cali_data[i]);
			pdata->gs_kvalue |= (g_sensor_cali_data[i] << (i * 8));
			}
		}

	} else
		D("%s: G-sensor Calibration data offset not found", __func__);

	pdata->gs_kvalue_L1 = 0;
	pdata->gs_kvalue_L2 = 0;
	pdata->gs_kvalue_L3 = 0;
	pdata->gs_kvalue_R1 = 0;
	pdata->gs_kvalue_R2 = 0;
	pdata->gs_kvalue_R3 = 0;
	pdata->gy_kvalue = 0;
	pdata->ALS_goldh = 0;
	pdata->ALS_goldl = 0;
	if ((gyro_sensor_offset = of_find_node_by_path(CALIBRATION_DATA_PATH))) {
		gyro_sensor_cali_data = (unsigned char*) of_get_property(gyro_sensor_offset, GYRO_SENSOR_FLASH_DATA, &gyro_sensor_cali_size);
		printk("%s:gyro cali_size = %d\n", __func__, gyro_sensor_cali_size);
		if (gyro_sensor_cali_data) {
			for (i = 0; (i < gyro_sensor_cali_size) && (i < 4); i++) {
				D("gyro sensor cali_data[%d] = %02x \n", i, gyro_sensor_cali_data[i]);
				pdata->gy_kvalue |= (gyro_sensor_cali_data[i] << (i * 8));
			}
			pdata->gs_kvalue_L1 = (gyro_sensor_cali_data[5] << 8) | gyro_sensor_cali_data[4];
			D("g sensor cali_data L1 = %x \n", pdata->gs_kvalue_L1);
			pdata->gs_kvalue_L2 = (gyro_sensor_cali_data[7] << 8) | gyro_sensor_cali_data[6];
			D("g sensor cali_data L2 = %x \n", pdata->gs_kvalue_L2);
			pdata->gs_kvalue_L3 = (gyro_sensor_cali_data[9] << 8) | gyro_sensor_cali_data[8];
			D("g sensor cali_data L3 = %x \n", pdata->gs_kvalue_L3);
			pdata->gs_kvalue_R1 = (gyro_sensor_cali_data[11] << 8) | gyro_sensor_cali_data[10];
			D("g sensor cali_data R1 = %x \n", pdata->gs_kvalue_R1);
			pdata->gs_kvalue_R2 = (gyro_sensor_cali_data[13] << 8) | gyro_sensor_cali_data[12];
			D("g sensor cali_data R2 = %x \n", pdata->gs_kvalue_R2);
			pdata->gs_kvalue_R3 = (gyro_sensor_cali_data[15] << 8) | gyro_sensor_cali_data[14];
			D("g sensor cali_data R3 = %x \n", pdata->gs_kvalue_R3);
		}

	} else
		D("%s: GYRO-sensor Calibration data offset not found", __func__);

	pdata->als_kvalue = 0;
	if ((light_sensor_offset = of_find_node_by_path(CALIBRATION_DATA_PATH))) {
		light_sensor_cali_data = (unsigned char*) of_get_property(light_sensor_offset, LIGHT_SENSOR_FLASH_DATA, &light_sensor_cali_size);
		D("%s:light cali_size = %d\n", __func__, light_sensor_cali_size);
		if (light_sensor_cali_data) {
			for (i = 0; (i < light_sensor_cali_size) && (i < 4); i++) {
				D("light sensor cali_data[%d] = %02x \n", i, light_sensor_cali_data[i]);
				pdata->als_kvalue |= (light_sensor_cali_data[i] << (i * 8));
			}
		}
	} else
		D("%s: LIGHT-sensor Calibration data offset not found", __func__);

	pdata->ps_kvalue = 0;
	if ((p_sensor_offset = of_find_node_by_path(CALIBRATION_DATA_PATH))) {
		p_sensor_cali_data = (unsigned char*) of_get_property(p_sensor_offset, PROX_SENSOR_FLASH_DATA, &p_sensor_cali_size);
		D("%s:proximity cali_size = %d\n", __func__, p_sensor_cali_size);
		if (p_sensor_cali_data) {
			for (i = 0; (i < p_sensor_cali_size) && (i < 8); i++) {
				D("proximity sensor cali_data[%d] = %02x \n", i, p_sensor_cali_data[i]);
				if(i < 4)
					pdata->ps_kheader |= (p_sensor_cali_data[i] << (i * 8));
				else
					pdata->ps_kvalue |= (p_sensor_cali_data[i] << ((i-4) * 8));
			}
		}
	} else
		D("%s: PROXIMITY-sensor Calibration data offset not found", __func__);

	pdata->bs_kvalue = 0;
	if ((baro_sensor_offset = of_find_node_by_path(CALIBRATION_DATA_PATH))) {
		baro_sensor_cali_data = (unsigned char*) of_get_property(baro_sensor_offset, BARO_SENSOR_FLASH_DATA, &baro_sensor_cali_size);
		D("%s: cali_size = %d\n", __func__, baro_sensor_cali_size);
		if (baro_sensor_cali_data) {
			for (i = 0; (i < baro_sensor_cali_size) && (i < 5); i++) {
				D("baro sensor cali_data[%d] = %02x \n", i, baro_sensor_cali_data[i]);
				if(i == baro_sensor_cali_size - 1)
					pdata->bs_kheader = baro_sensor_cali_data[i];
				else
					pdata->bs_kvalue |= (baro_sensor_cali_data[i] << (i * 8));
			}
		}
	} else
		D("%s: Barometer-sensor Calibration data offset not found", __func__);

        pdata->gpio_wake_mcu = of_get_named_gpio(dt, "mcu,Cpu_wake_mcu-gpio", 0);
        if (!gpio_is_valid(pdata->gpio_wake_mcu)) {
                pr_info("[MCU] DT:gpio_wake_mcu value is not valid\n");
        } else
                pr_info("[MCU] DT:gpio_wake_mcu=%d", pdata->gpio_wake_mcu);

        pdata->gpio_mcu_irq = of_get_named_gpio(dt, "mcu,intr-gpio", 0);
        if (!gpio_is_valid(pdata->gpio_mcu_irq)) {
                pr_info("[MCU] DT:gpio_mcu_irq value is not valid\n");
        } else
                pr_info("[MCU] DT:gpio_mcu_irq=%d", pdata->gpio_mcu_irq);

        pdata->gpio_reset = of_get_named_gpio(dt, "mcu,Reset-gpio", 0);
        if (!gpio_is_valid(pdata->gpio_reset)) {
                pr_info("[MCU] DT:gpio_reset value is not valid\n");
        } else
                pr_info("[MCU] DT:gpio_reset=%d", pdata->gpio_reset);

        pdata->gpio_chip_mode = of_get_named_gpio(dt, "mcu,Chip_mode-gpio", 0);
        if (!gpio_is_valid(pdata->gpio_chip_mode)) {
                pr_info("[MCU] DT:gpio_chip_mode value is not valid\n");
        } else
                pr_info("[MCU] DT:gpio_chip_mode=%d", pdata->gpio_chip_mode);

        prop = of_find_property(dt, "mcu,GS_chip_layout", NULL);
        if (prop) {
                of_property_read_u32(dt, "mcu,GS_chip_layout", &buf);
                pdata->GS_chip_layout = buf;
                I("%s: chip_layout = %d", __func__, pdata->GS_chip_layout);
        } else
                I("%s: g_sensor,chip_layout not found", __func__);

        prop = of_find_property(dt, "mcu,Acceleration_axes", NULL);
        if (prop) {
                of_property_read_u32(dt, "mcu,Acceleration_axes", &buf);
                pdata->Acceleration_axes = buf;
                I("%s: Acceleration axes = %d", __func__, pdata->Acceleration_axes);
        } else
                I("%s: g_sensor axes not found", __func__);

        prop = of_find_property(dt, "mcu,Magnetic_axes", NULL);
        if (prop) {
                of_property_read_u32(dt, "mcu,Magnetic_axes", &buf);
                pdata->Magnetic_axes = buf;
                I("%s: Compass axes = %d", __func__, pdata->Magnetic_axes);
        } else
                I("%s: Compass axes not found", __func__);

        prop = of_find_property(dt, "mcu,Gyro_axes", NULL);
        if (prop) {
                of_property_read_u32(dt, "mcu,Gyro_axes", &buf);
                pdata->Gyro_axes = buf;
                I("%s: Gyro axes = %d", __func__, pdata->Gyro_axes);
        } else
                I("%s: Gyro axes not found", __func__);

        prop = of_find_property(dt, "mcu,ALS_goldh", NULL);
        if (prop) {
                of_property_read_u32(dt, "mcu,ALS_goldh", &buf);
                pdata->ALS_goldh = buf;
                I("%s: ALS_goldh = 0x%x", __func__, pdata->ALS_goldh);
        } else
                I("%s: ALS_goldh not found", __func__);

        prop = of_find_property(dt, "mcu,ALS_goldl", NULL);
        if (prop) {
                of_property_read_u32(dt, "mcu,ALS_goldl", &buf);
                pdata->ALS_goldl = buf;
                I("%s: ALS_goldl = 0x%x", __func__, pdata->ALS_goldl);
        } else
                I("%s: ALS_goldl not found", __func__);

        prop = of_find_property(dt, "mcu,ls_polling", NULL);
        if (prop) {
                of_property_read_u32(dt, "mcu,ls_polling", &buf);
                pdata->ls_polling = buf;
                I("%s: ls_polling = 0x%x", __func__, pdata->ls_polling);
        } else {
                pdata->ls_polling = 0;
                I("%s: ls_polling not found", __func__);
        }

	prop = of_find_property(dt, "mcu,proximity_debu_info", NULL);
	if (prop) {
		buf = 0;
		of_property_read_u32(dt, "mcu,proximity_debu_info", &buf);
		pdata->proximity_debu_info = buf;
		I("%s: proximity_debu_info = 0x%x", __func__,
		  pdata->proximity_debu_info);
	} else {
		pdata->proximity_debu_info = 0;
		I("%s: proximity_debu_info not found", __func__);
	}

        return 0;
}

static ssize_t bma250_clear_powerkey_pressed(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long powerkey_pressed;
	int error;
	error = strict_strtoul(buf, 10, &powerkey_pressed);
	if (error)
	    return error;

	if(powerkey_pressed == 1) {
	    power_key_pressed = 1;
	}
	else if(powerkey_pressed == 0) {
	    power_key_pressed = 0;
	}
	return count;
}

static ssize_t bma250_get_powerkry_pressed(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", power_key_pressed);
}
static DEVICE_ATTR(clear_powerkey_flag, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		bma250_get_powerkry_pressed, bma250_clear_powerkey_pressed);

static ssize_t p_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%d\n",p_status);
}
static DEVICE_ATTR(p_status, 0444, p_status_show, NULL);

static int set_gesture_motion(struct device *dev,struct device_attribute *attr,const char *buf, size_t count){
	u8 *data;
	u32 val = 0;
	int i;

	sscanf(buf, "0x%x\n", &val);

	data = (u8 *)&val;

	I("%s: Gesture motion parameter = 0x%x\n", __func__, val);
	I("%s: data(0, 1, 2, 3) = (0x%x, 0x%x, 0x%x, 0x%x)\n",
	  __func__, data[0], data[1], data[2], data[3]);
	mcu_data->gesture_motion_param = val;

	for (i = 0; i < GESTURE_MOTION_UPDATE_ATTRIBUTE_LEN; i++) {
		I("%s: writing 0x%x to 0xC9\n", __func__, data[i]);
		CWMCU_i2c_write(mcu_data,
				  GESTURE_MOTION_UPDATE_ATTRIBUTE,
				  &data[i], 1);
	}

	return count;
}

static int get_gesture_motion(struct device *dev, struct device_attribute *attr, char *buf){
	u8 data[GESTURE_MOTION_UPDATE_ATTRIBUTE_LEN] = {0};

	if (CWMCU_i2c_read(mcu_data, GESTURE_MOTION_UPDATE_ATTRIBUTE, data, 4) >= 0) {
		I("%s: gesture_motion_param = 0x%08X, data(0, 1, 2, 3) = (0x%x, 0x%x, 0x%x, 0x%x), cpu_to_le32p((__u32 *)&data) = 0x%08X\n", __func__,
		  mcu_data->gesture_motion_param, data[0], data[1], data[2], data[3], cpu_to_le32p((__u32 *)&data));
		return snprintf(buf, PAGE_SIZE, "0x%08X\n",
				cpu_to_le32p((__u32 *)&data));
	}
	return snprintf(buf, PAGE_SIZE, "0x%08X\n", 0xFFFFFFFF);
}


static struct device_attribute attributes[] = {

	__ATTR(enable, 0666, active_show,
			active_set),
	__ATTR(delay_ms, 0666, interval_show,
			interval_set),
	__ATTR(barometer_self_test, 0660, barometer_self_test_show,
			barometer_self_test_set),
	__ATTR(reset_hub, 0660, reset_hub_show, reset_hub_set),
	__ATTR(calibrator_en, 0666, NULL, set_calibrator_en),
	__ATTR(calibrator_status_acc, 0666, show_calibrator_status_acc, NULL),
	__ATTR(calibrator_status_mag, 0666, show_calibrator_status_mag, NULL),
	__ATTR(calibrator_status_gyro, 0666, show_calibrator_status_gyro, NULL),
	__ATTR(calibrator_data_acc, 0666, get_k_value_acc_f, set_k_value_acc_f),
	__ATTR(calibrator_data_accRL, 0666, get_k_value_accRL_f, NULL),
	__ATTR(AP_calibrator_data_accRL, 0666, AP_get_k_value_accRL_f, NULL),
	__ATTR(calibrator_data_mag, 0666, get_k_value_mag_f, set_k_value_mag_f),
	__ATTR(calibrator_data_gyro, 0666, get_k_value_gyro_f, set_k_value_gyro_f),
	__ATTR(calibrator_data_light, 0666, get_k_value_light_f, NULL),
	__ATTR(calibrator_data_proximity, 0666, get_k_value_proximity_f, set_k_value_proximity_f),
	__ATTR(calibrator_data_barometer, 0666, get_k_value_barometer_f, set_k_value_barometer_f),
	__ATTR(gesture_motion, 0660, get_gesture_motion, set_gesture_motion),
	__ATTR(data_barometer, 0666, get_barometer, NULL),
	__ATTR(data_proximity, 0666, get_proximity, NULL),
	__ATTR(data_proximity_polling, 0666, get_proximity_polling, NULL),
	__ATTR(data_light_polling, 0666, get_light_polling, NULL),
	__ATTR(ls_mechanism, 0666, get_ls_mechanism, NULL),
	__ATTR(sensor_hub_rdata,0666,NULL,read_mcu_data),
	__ATTR(ps_canc, 0666, get_ps_canc, set_ps_canc),
	__ATTR(data_light_kadc, 0666, get_light_kadc, NULL),
	__ATTR(firmware_version, 0666, get_firmware_version, NULL),
	__ATTR(boot_mode, 0660, boot_mode_show, boot_mode_set),
#ifdef HTC_ENABLE_SENSORHUB_UART_DEBUG
	__ATTR(uart_debug, 0666, NULL, uart_debug_switch),
#endif
};


static int create_sysfs_interfaces(struct CWMCU_data *sensor)
{
	int i;
	struct class *bma250_powerkey_class = NULL;
	struct device *bma250_powerkey_dev = NULL;
	struct class *optical_class = NULL;
	struct device *proximity_dev = NULL;
	int res;

	bma250_powerkey_class = class_create(THIS_MODULE, "bma250_powerkey");
	if (IS_ERR(bma250_powerkey_class)) {
		res = PTR_ERR(bma250_powerkey_class);
		bma250_powerkey_class = NULL;
		E("%s: could not allocate bma250_powerkey_class, res = %d\n", __func__, res);
		goto error_powerkey_class;
	}

	bma250_powerkey_dev= device_create(bma250_powerkey_class,
			NULL, 0, "%s", "bma250");
	res = device_create_file(bma250_powerkey_dev, &dev_attr_clear_powerkey_flag);
	if (res) {
		E("%s, create bma250_device_create_file fail!\n", __func__);
		goto err_create_bma250_device_file;
	}

	optical_class = class_create(THIS_MODULE, "optical_sensors");
	if (IS_ERR(optical_class)) {
		res = PTR_ERR(optical_class);
		optical_class = NULL;
		E("%s: could not allocate optical_class, res = %d\n", __func__, res);
		goto error_optical_class_create;
	}

	proximity_dev= device_create(optical_class,
			NULL, 0, "%s", "proximity");
	res = device_create_file(proximity_dev, &dev_attr_p_status);
	if (res) {
		E("%s, create proximty_device_create_file fail!\n", __func__);
		goto err_create_proximty_device_file;
	}

	sensor->sensor_class = class_create(THIS_MODULE, "htc_sensorhub");
	if (sensor->sensor_class == NULL)
		goto custom_class_error;

	sensor->sensor_dev = device_create(sensor->sensor_class, NULL, 0, "%s", "sensor_hub");
	if (sensor->sensor_dev == NULL)
		goto custom_device_error;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(sensor->sensor_dev, attributes + i))
			goto error;

	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(sensor->sensor_dev, attributes + i);

custom_device_error:
	class_destroy(sensor->sensor_class);
custom_class_error:
	device_remove_file(proximity_dev, &dev_attr_p_status);
err_create_proximty_device_file:
	class_destroy(optical_class);
error_optical_class_create:
	device_remove_file(bma250_powerkey_dev, &dev_attr_clear_powerkey_flag);

err_create_bma250_device_file:
	class_destroy(bma250_powerkey_class);
error_powerkey_class:
	dev_err(&sensor->client->dev, "%s:Unable to create interface\n", __func__);
	return -1;
}
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
        struct fb_event *evdata = data;
        int *blank;

        D("%s\n", __func__);
        if (evdata && evdata->data && event == FB_EVENT_BLANK && mcu_data &&
                        mcu_data->client) {
                blank = evdata->data;
                switch (*blank) {
                case FB_BLANK_UNBLANK:
			D("MCU late_resume\n");
			mcu_data->input_polled->poll_interval = 10;
                        break;
                case FB_BLANK_POWERDOWN:
                case FB_BLANK_HSYNC_SUSPEND:
                case FB_BLANK_VSYNC_SUSPEND:
                case FB_BLANK_NORMAL:
			D("MCU early_suspend\n");
			mcu_data->input_polled->poll_interval = 200;
                        break;
                }
        }
        return 0;
}
#endif
static int __devinit CWMCU_i2c_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	struct CWMCU_data *sensor;
	int error;
	int i = 0, j = 0;

	probe_success = 0;
	if (board_mfg_mode() == MFG_MODE_OFFMODE_CHARGING) {
		E("%s: offmode_charging, do not probe CwMcuSensor\n", __func__);
		return -EACCES;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "castor: i2c_check_functionality error\n");
		return -EIO;
	}

	sensor = kzalloc(sizeof(struct CWMCU_data), GFP_KERNEL);
	if (!sensor) {
		E("%s: kzalloc error\n", __func__);
		return -ENOMEM;
	}
	mcu_data = sensor;

	sensor->client = client;
	i2c_set_clientdata(client, sensor);

	error = CWMCU_register_polled_device(sensor);
	if (error) {
		D("castor: CWMCU_register_polled_device error\n");
		goto err_free_mem;
	}
#if 0
	error =sysfs_create_group(&sensor->input->dev.kobj,
			       &sysfs_attribute_group);
#endif
        error = create_sysfs_interfaces(sensor);

	if (error)
		goto exit_free_input;

	for(i=0;i<numSensors; i++){
		sensor->sensors_time[i] = 0;
		sensor->report_period[i] = 20000;
	}
        if (client->dev.of_node) {
                D("Device Tree parsing.");

                error = mcu_parse_dt(&client->dev, sensor);
                if (error) {
                        dev_err(&client->dev, "%s: mcu_parse_dt "
                                        "for pdata failed. err = %d",
                                        __func__, error);
                        goto exit_mcu_parse_dt_fail;
                }
        } else {
                if (client->dev.platform_data != NULL){
			sensor->Acceleration_axes = ((struct CWMCU_platform_data *)sensor->client->dev.platform_data)->Acceleration_axes;
			sensor->Magnetic_axes = ((struct CWMCU_platform_data *)sensor->client->dev.platform_data)->Magnetic_axes;
			sensor->Gyro_axes = ((struct CWMCU_platform_data *)sensor->client->dev.platform_data)->Gyro_axes;
			sensor->gpio_wake_mcu = ((struct CWMCU_platform_data *)sensor->client->dev.platform_data)->gpio_wake_mcu;
			sensor->ALS_goldl = ((struct CWMCU_platform_data *)sensor->client->dev.platform_data)->ALS_goldl;
			sensor->ALS_goldh = ((struct CWMCU_platform_data *)sensor->client->dev.platform_data)->ALS_goldh;
			sensor->ls_polling = ((struct CWMCU_platform_data *)sensor->client->dev.platform_data)->ls_polling;
                }
        }
#if 0
	if (!(struct CWMCU_platform_data *)sensor->client->dev.platform_data) {
		sensor->Acceleration_axes = 0;
		sensor->Magnetic_axes = 0;
		sensor->Gyro_axes = 0;		
	}else{
		sensor->Acceleration_axes = ((struct CWMCU_platform_data *)sensor->client->dev.platform_data)->Acceleration_axes;
		sensor->Magnetic_axes = ((struct CWMCU_platform_data *)sensor->client->dev.platform_data)->Magnetic_axes;
		sensor->Gyro_axes = ((struct CWMCU_platform_data *)sensor->client->dev.platform_data)->Gyro_axes;
	}
	
	sensor->wake_mcu = ((struct CWMCU_platform_data *)sensor->client->dev.platform_data)->wake_mcu;
#endif

	wake_lock_init(&ges_wake_lock, WAKE_LOCK_SUSPEND, "ges_wake_lock");
	wake_lock_init(&significant_wake_lock, WAKE_LOCK_SUSPEND, "significant_wake_lock");

	INIT_WORK(&sensor->irq_work, cwmcu_irq_work_func);

	INIT_DELAYED_WORK(&sensor->work, exception_do_work_wdg);
	atomic_set(&sensor->delay, CWMCU_MAX_DELAY);

	mcu_wq = create_singlethread_workqueue("htc_mcu");
	i2c_set_clientdata(client, sensor);
	pm_runtime_enable(&client->dev);

	error = gpio_request(sensor->gpio_mcu_irq, "cwmcu_int");
	if (error) {
		D("%s : request irq gpio fail\n",
			__func__);
	}
	client->irq = gpio_to_irq(sensor->gpio_mcu_irq);
#if 1
	sensor->IRQ = client->irq;
	D("Requesting irq = %d\n", sensor->IRQ);
        error = request_irq(sensor->IRQ, cwmcu_irq_handler, IRQF_TRIGGER_RISING,
			  "cwmcu", sensor);
        if (error)
            D("[CWMCU] could not request irq %d\n", error);
        error = enable_irq_wake(sensor->IRQ); 
	if(error < 0)
            D("[CWMCU] could not enable irq as wakeup source %d\n", error);
#endif
	sensor->resume_done = 1;

	for (j = 0; j < numSensors; j++)
		sensor->filter_first_zeros[j] = 0;

	queue_delayed_work(mcu_wq, &polling_work,
                        msecs_to_jiffies(5000));
#if HTC_ENABLE_SENSORHUB_DEBUG
	error = misc_register(&cwmcu_device);
	if (error) {
		D("[CWMCU] cwmcu_device register failed\n");
	}
#endif
	vib_trigger_register_simple("vibrator", &vib_trigger);

	sensor->mfg_mode = board_mfg_mode();
	D("%s: Boot mode = %d\n", __func__, sensor->mfg_mode);

	D("castor: CWMCU_i2c_probe success!\n");
#ifdef CONFIG_FB
        sensor->mcu_poll_wq = create_singlethread_workqueue("mcu_poll_reuqest");
        if (!sensor->mcu_poll_wq) {
                pr_err("[MCU] allocate failed\n");
                goto err_get_mcu_poll_failed;
        }
        INIT_DELAYED_WORK(&sensor->work_poll, mcu_fb_register);
        queue_delayed_work(sensor->mcu_poll_wq, &sensor->work_poll, msecs_to_jiffies(15000));
#endif
	probe_success = 1;
	return 0;
#if 0
exit_free_input_htc_gesture:
	input_free_device(sensor->input_htc_gesture);
#endif
err_get_mcu_poll_failed:
exit_free_input:
	if (sensor && (sensor->input))
		input_free_device(sensor->input);
	if (sensor && (sensor->input_polled)) {
		input_unregister_polled_device(sensor->input_polled);
		input_free_polled_device(sensor->input_polled);
	}
exit_mcu_parse_dt_fail:
        if (client->dev.of_node && ((struct CWMCU_platform_data *)sensor->client->dev.platform_data))
                kfree(sensor->client->dev.platform_data);
err_free_mem:
	kfree(sensor);
	return error;	
}


static int CWMCU_i2c_remove(struct i2c_client *client)
{
	struct CWMCU_data *sensor = i2c_get_clientdata(client);

	if (sensor && (sensor->input_polled)) {
		input_unregister_polled_device(sensor->input_polled);
		input_free_polled_device(sensor->input_polled);
	}
#ifdef CONFIG_FB
        if (fb_unregister_client(&sensor->fb_notif))
                dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");

#endif
	wake_lock_destroy(&ges_wake_lock);
	wake_lock_destroy(&significant_wake_lock);
	kfree(sensor);
	return 0;
}

static const struct dev_pm_ops CWMCU_pm_ops = {
	.suspend = CWMCU_suspend,
	.resume = CWMCU_resume
};


static const struct i2c_device_id CWMCU_id[] = {
	{CWMCU_I2C_NAME, 0},
	{ }
};
#ifdef CONFIG_OF
static struct of_device_id mcu_match_table[] = {
        {.compatible = "htc_mcu" },
        {},
};
#else
#define mcu_match_table NULL
#endif

MODULE_DEVICE_TABLE(i2c, CWMCU_id);

static struct i2c_driver CWMCU_driver = {
	.driver = {
		.name = CWMCU_I2C_NAME,
		   .owner = THIS_MODULE,
		.pm = &CWMCU_pm_ops,
		.of_match_table = mcu_match_table,
	},
	.probe    = CWMCU_i2c_probe,
	.remove   = __devexit_p(CWMCU_i2c_remove),
	.id_table = CWMCU_id,
};

static int __init CWMCU_i2c_init(void)
{
	return i2c_add_driver(&CWMCU_driver);
}
module_init(CWMCU_i2c_init);

static void __exit CWMCU_i2c_exit(void)
{
	i2c_del_driver(&CWMCU_driver);
}
module_exit(CWMCU_i2c_exit);

MODULE_DESCRIPTION("CWMCU I2C Bus Driver V1.6");
MODULE_AUTHOR("CyWee Group Ltd.");
MODULE_LICENSE("GPL");
