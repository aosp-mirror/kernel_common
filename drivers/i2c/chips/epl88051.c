/* drivers/i2c/chips/epl88051.c - light and proxmity sensors driver
 * Copyright (C) 2011 ELAN Corporation.
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

#include <linux/lightsensor.h>
#include <linux/hrtimer.h>
#include <linux/timer.h>
#include <linux/delay.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
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
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include <linux/elan_interface.h>
#include <linux/epl8800.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <mach/board_htc.h>

#define PS_RAW_8BIT
#define PS_POLLING_MODE         	0				

#ifdef PS_RAW_8BIT 
#define PS_LOW_THRESHOLD		7
#define PS_HIGH_THRESHOLD		13
#else
#define PS_LOW_THRESHOLD		2000
#define PS_HIGH_THRESHOLD		5000
#endif 

#define LUX_PER_COUNT			700
#define PS_DELAY			35
#define ALS_DELAY			80
#define HS_BUFFER_SIZE			200
#define HS_CYCLE			1
#define CALIBRATION_DATA_PATH "/calibration_data"
#define LIGHT_SENSOR_FLASH_DATA "als_flash"
#define PSENSOR_FLASH_DATA "ps_flash"
#define POLLING_DELAY           200
#define TH_ADD                  10
#define MFG_MODE 1
#define NEAR_DELAY_TIME ((500 * HZ) / 1000)
static int hs_count = 0;
static int start_idx = 0;
static struct mutex sensor_mutex;
static int HS_INTT = EPL_INTT_PS_250;
static int PS_INTT = EPL_INTT_PS_150; 
int c_gain;
int dynamic_intt_idx;
int dynamic_intt_init_idx = 1;	
static int lightsensor_cali;
uint32_t dynamic_intt_min_unit = 1000;
uint8_t dynamic_intt_intt;
uint8_t dynamic_intt_gain;
uint16_t dynamic_intt_high_thr;
uint16_t dynamic_intt_low_thr;
static int als_dynamic_intt_intt[] = {EPL_INTT_ALS_3000, EPL_INTT_ALS_250, EPL_INTT_ALS_15};
static int als_dynamic_intt_intt_value[] = {3000, 250, 15};
static int als_dynamic_intt_gain[] = {EPL_M_GAIN, EPL_M_GAIN, EPL_M_GAIN};
static int als_dynamic_intt_high_thr[] = {900*64, 850*64, 580*64};
static int als_dynamic_intt_low_thr[] = {60*64, 60*64, 40*64};
static int als_dynamic_intt_intt_num =  sizeof(als_dynamic_intt_intt_value)/sizeof(int);
static uint32_t adctable[10] = {0};
static int p_status = 9;
static int psensor_enabled;

#define TXBYTES			2
#define RXBYTES			2
#define PACKAGE_SIZE 		2
#define I2C_RETRY_COUNT 	10

#define MAX_PS_CH0 30000 

typedef struct _epl_raw_data
{
	u8 raw_bytes[PACKAGE_SIZE];
	u16 ps_state;
    u8 con_sat; 
	u16 ps_raw;
	u16 ps_ch0_raw; 
	u16 als_ch0_raw;
	u16 als_ch1_raw;
	u16 als_ch0_raw_now;
	u16 als_ch1_raw_now;
	u16 als_lux;
	uint32_t ratio;
	u16 hs_data[HS_BUFFER_SIZE];
	int als_level;
	u16 als_raw;
} epl_raw_data;

struct epl88051_priv
{
	struct i2c_client *client;
	struct input_dev *als_input_dev;
	struct input_dev *ps_input_dev;
	struct input_dev *input_dev;
	struct workqueue_struct *epl_wq;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
	struct workqueue_struct *epl88051_fb_wq;
	struct delayed_work work_fb;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct class *epl88051_class;
	struct device *ls_dev;
	struct device *ps_dev;

	int intr_pin;
	uint32_t irq_gpio_flags;
	int (*power)(int on);
	int ps_pocket_mode;

	int ps_opened;
	int als_opened;

	int ps_threshold_high;
	int ps_threshold_low;
	int ps_threshold_diff;

	int als_threshold_high;
	int als_threshold_low;

	int polling_mode_als;
	int polling_mode_ps;
	int polling_mode_hs;

	u16 ps_max_ch0;

	int als_suspend;
	int ps_suspend;
	int hs_suspend;

	int lux_per_count;

	int enable_pflag;
	int enable_lflag;
	int enable_hflag;

	int read_flag;
	int irq;

	int c_gain_h; 
	int c_gain_l; 
	int c_gain_saturated;
	uint32_t lsource_thd_high; 
	uint32_t lsource_thd_low; 
	int ps_delay;

	uint32_t emmc_als_kadc;
	uint32_t emmc_ps_kadc1;
	uint32_t emmc_ps_kadc2;
	uint32_t golden_adc;
	uint32_t *adc_table;
	int ls_calibrate;
	int ws_calibrate;
	uint32_t als_gadc;
	uint32_t als_kadc;

	int mfg_mode;
	unsigned long j_start;
	unsigned long j_end;
} ;
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data);
#endif

static struct platform_device *sensor_dev;
static struct wake_lock g_ps_wlock;
struct epl88051_priv *epl88051_obj;
static epl_raw_data gRawData;
static int epl_ps_raw_data;
static int min_epl_ps_raw_data;
static const char ElanPsensorName[]="proximity";
static const char ElanALsensorName[]="lightsensor-level";

#define LOG_TAG                      "[EPL88051] "
#define LOG_FUN(f)               	 printk(KERN_INFO LOG_TAG"%s\n", __FUNCTION__)
#define LOG_FUN(f)                       printk(KERN_INFO LOG_TAG"%s\n", __FUNCTION__)
#define LOG_INFO(fmt, args...)    	 printk(KERN_INFO LOG_TAG fmt, ##args)
#define LOG_ERR(fmt, args...)   	 printk(KERN_ERR  LOG_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define D(x...) pr_info(x)

static int epl88051_set_ps_threshold(uint16_t low_thd, uint16_t high_thd);
static int epl88051_setup_interrupt(struct epl88051_priv *epld);

static void epl88051_eint_work(struct work_struct *work);
static DECLARE_WORK(epl_sensor_irq_work, epl88051_eint_work);

static void epl88051_polling_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(polling_work, epl88051_polling_work);

static void epl88051_dyna_thd_polling_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(dyna_thd_polling_work, epl88051_dyna_thd_polling_work);
/*
//====================I2C write operation===============//
//regaddr: ELAN Register Address.
//bytecount: How many bytes to be written to register via i2c bus.
//txbyte: I2C bus transmit byte(s). Single byte(0X01) transmit only slave address.
//data: setting value.
//
// Example: If you want to write single byte to 0x1D register address, show below
//	      epl88051_I2C_Write(client,0x1D,0x01,0X02,0xff);
//
 */
static int epl88051_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount, uint8_t txbyte, uint8_t data)
{
	uint8_t buffer[2];
	int ret = 0;
	int retry, val;
	struct epl88051_priv *epld = epl88051_obj;

	buffer[0] = (regaddr << 3) | bytecount ;
	buffer[1] = data;


	for (retry = 0; retry < I2C_RETRY_COUNT; retry++)
	{
		ret = i2c_master_send(client, buffer, txbyte);

		if (ret == txbyte)
		{
			break;
		}

		val = gpio_get_value(epld->intr_pin);

		LOG_INFO("%s INTERRUPT GPIO val = %d\n", __FUNCTION__, val);

		msleep(10);
	}


	if (retry >= I2C_RETRY_COUNT)
	{
		LOG_ERR(KERN_ERR "i2c write retry over %d\n", I2C_RETRY_COUNT);
		return -EINVAL;
	}

	return ret;
}

static int epl88051_I2C_Read(struct i2c_client *client)
{
	uint8_t buffer[RXBYTES];
	int ret = 0, i = 0;
	int retry, val;
	struct epl88051_priv *epld = epl88051_obj;

	for (retry = 0; retry < I2C_RETRY_COUNT; retry++) {

		ret = i2c_master_recv(client, buffer, RXBYTES);

		if (ret == RXBYTES)
			break;

		val = gpio_get_value(epld->intr_pin);

		LOG_INFO("%s INTERRUPT GPIO val = %d\n", __FUNCTION__, val);

		msleep(10);
	}

	if (retry >= I2C_RETRY_COUNT) {
		LOG_ERR("i2c read retry over %d\n", I2C_RETRY_COUNT);
		return -EINVAL;
	}

	for (i = 0; i < PACKAGE_SIZE; i++) {
		gRawData.raw_bytes[i] = buffer[i];
	}

	return ret;
}

static void epl88051_restart_work(void)
{
	struct epl88051_priv *epld = epl88051_obj;
	cancel_delayed_work(&polling_work);
	queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(50));
#if 1 
    if(epld->mfg_mode != MFG_MODE)
    {
        cancel_delayed_work(&dyna_thd_polling_work);
        queue_delayed_work(epld->epl_wq, &dyna_thd_polling_work,msecs_to_jiffies(POLLING_DELAY));
    }
#endif  
}



static void epl88051_hs_enable(struct epl88051_priv *epld, bool interrupt, bool full_enable)
{
	int ret;
	uint8_t regdata = 0;
	struct i2c_client *client = epld->client;

	if (full_enable) {
		regdata =  EPL_INT_CH1 | EPL_IR_INTERNAL | (interrupt? EPL_INT_FRAME_ENABLE : EPL_INT_DISABLE );
		ret = epl88051_I2C_Write(client, REG_6, W_SINGLE_BYTE, 0x02, regdata);

		regdata = EPL_IR_MODE_VOLTAGE;
		epl88051_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02, regdata);

		regdata =  EPL_PS_MODE |EPL_10BIT_ADC | EPL_L_GAIN |EPL_S_SENSING_MODE;
		ret = epl88051_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0X02, regdata);

		ret = epl88051_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02, 0xc0);
	}

	regdata = HS_INTT | (HS_CYCLE << 5);
	ret = epl88051_I2C_Write(client, REG_1, W_SINGLE_BYTE, 0X02, regdata);
	ret = epl88051_I2C_Write(client, REG_8, W_SINGLE_BYTE, 0X02, EPL_C_RESET);
	ret = epl88051_I2C_Write(client, REG_8, W_SINGLE_BYTE, 0x02, EPL_C_START_RUN);

}


static int epl88051_psensor_enable(struct epl88051_priv *epld)
{
	int ret;
	uint8_t regdata = 0;
	struct i2c_client *client = epld->client;
	int ps_state;

	LOG_INFO("[PS] %s --- Proximity sensor Enable ---\n", __FUNCTION__);

	ret = epl88051_I2C_Write(client, REG_6, W_SINGLE_BYTE, 0x02, EPL_INT_DISABLE | EPL_INT_CH1);

	regdata =  EPL_PS_MODE | EPL_10BIT_ADC | EPL_M_GAIN ;
	regdata = regdata | (epld->polling_mode_ps == 0 ? EPL_C_SENSING_MODE : EPL_S_SENSING_MODE);
	ret = epl88051_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0X02, regdata);

	regdata = PS_INTT | EPL_SENSING_8_TIME;
	ret = epl88051_I2C_Write(client, REG_1, W_SINGLE_BYTE, 0X02, regdata);

	
	ret = epl88051_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0X02, EPL_IR_MODE_VOLTAGE);

	
	epl88051_set_ps_threshold(epld->ps_threshold_low, epld->ps_threshold_high);

	ret = epl88051_I2C_Write(client, REG_8, W_SINGLE_BYTE, 0X02, EPL_C_RESET);
	ret = epl88051_I2C_Write(client, REG_8, W_SINGLE_BYTE, 0x02, EPL_C_START_RUN);

	msleep(epld->ps_delay);
	if (epld->polling_mode_ps == 0) {
		epl88051_I2C_Write(client, REG_13, R_SINGLE_BYTE, 0x01, 0);
		epl88051_I2C_Read(client);
		ps_state= !((gRawData.raw_bytes[0] & 0x04) >> 2);
		gRawData.con_sat = (gRawData.raw_bytes[0] >> 1) & 0x01; 
#if 1 
        
        epl88051_I2C_Write(client, REG_14, R_TWO_BYTE, 0x01, 0x00);
		epl88051_I2C_Read(client);
        gRawData.ps_ch0_raw = (gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0];
#endif

		epl88051_I2C_Write(client, REG_16, R_TWO_BYTE, 0x01, 0x00);
		epl88051_I2C_Read(client);
#ifdef PS_RAW_8BIT 
		if (gRawData.raw_bytes[1] >= (uint8_t) (epld->emmc_ps_kadc2 & 0xFF))
			gRawData.ps_raw = gRawData.raw_bytes[1] - (uint8_t) (epld->emmc_ps_kadc2 & 0xFF);  
		else
			gRawData.ps_raw = 0;
#else
		if (((gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0]) >= (uint8_t) epld->emmc_ps_kadc2)
			gRawData.ps_raw = ((gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0]) - (uint8_t) epld->emmc_ps_kadc2; 
		else
			gRawData.ps_raw = 0;
#endif  
		LOG_INFO("[PS] %s gRawData.ps_raw=%d\n", __FUNCTION__, gRawData.ps_raw);
		if (gRawData.ps_state != ps_state) {
			regdata =  EPL_INT_CH1 | EPL_INT_FRAME_ENABLE;                                              
			epl88051_I2C_Write(client, REG_6, W_SINGLE_BYTE, 0x02, regdata);
		} else {
			regdata =  EPL_INT_CH1 | EPL_INT_ACTIVE_LOW;
			epl88051_I2C_Write(client, REG_6, W_SINGLE_BYTE, 0x02, regdata);
		}
		ret = irq_set_irq_wake(epld->irq, 1);
		if (ret < 0) {
			LOG_ERR("%s: fail to enable irq %d as wake interrupt\n", __func__, epld->irq);
			return ret;
		}

	}

	psensor_enabled = 1;
	return ret;
}

static int epl88051_lsensor_enable(struct epl88051_priv *epld)
{

	int ret;
	uint8_t regdata = 0;
	struct i2c_client *client = epld->client;

	LOG_INFO("[LS] %s --- ALS sensor Enable --- \n", __FUNCTION__);
	regdata = EPL_INT_DISABLE;
	ret = epl88051_I2C_Write(client, REG_6, W_SINGLE_BYTE, 0x02, regdata);


	regdata = EPL_S_SENSING_MODE | EPL_ALS_MODE | dynamic_intt_gain | EPL_8BIT_ADC;
	ret = epl88051_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0X02, regdata);

	regdata = dynamic_intt_intt | EPL_SENSING_16_TIME;
	ret = epl88051_I2C_Write(client, REG_1, W_SINGLE_BYTE, 0X02, regdata);

	ret = epl88051_I2C_Write(client, REG_10, W_SINGLE_BYTE, 0x02, (EPL_GO_LOW << 4) | EPL_GO_MID);

	ret = epl88051_I2C_Write(client,REG_8, W_SINGLE_BYTE, 0X02, EPL_C_RESET);
	ret = epl88051_I2C_Write(client,REG_8, W_SINGLE_BYTE, 0x02, EPL_C_START_RUN);
	msleep(ALS_DELAY);

	return ret;
}

static void epl88051_read_ps(void)
{
	struct epl88051_priv *epld = epl88051_obj;
	struct i2c_client *client = epld->client;
	int is_ps_mode = 1;
	uint8_t setting;
	epld->j_end = jiffies;
	
	epl88051_I2C_Write(epld->client, REG_13, R_SINGLE_BYTE, 0x01, 0);
	epl88051_I2C_Read(epld->client);
	setting = gRawData.raw_bytes[0];
	if (((setting >> 3) & 7) != 0x01) {
		is_ps_mode = 0;
		LOG_ERR("read ps data in wrong mode\n");
	}
	if(is_ps_mode) {
		gRawData.ps_state= !((gRawData.raw_bytes[0] & 0x04) >> 2);
		
		

		
		epl88051_I2C_Write(client, REG_16, R_TWO_BYTE, 0x01, 0x00);
		epl88051_I2C_Read(client);
#ifdef PS_RAW_8BIT 
		if (gRawData.raw_bytes[1] >= (uint8_t) (epld->emmc_ps_kadc2 & 0xFF))
			gRawData.ps_raw = gRawData.raw_bytes[1] - (uint8_t) (epld->emmc_ps_kadc2 & 0xFF);  
		else
			gRawData.ps_raw = 0;
		epl_ps_raw_data = gRawData.raw_bytes[1];
#else
		if (((gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0]) >= (uint8_t) epld->emmc_ps_kadc2)
			gRawData.ps_raw = ((gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0]) - (uint8_t) epld->emmc_ps_kadc2; 
		else
			gRawData.ps_raw = 0;
		epl_ps_raw_data = (gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0];
#endif  
	}

	LOG_INFO("[PS] %s proximity %s ps_raw_data: %d\n", __FUNCTION__, gRawData.ps_state ? "FAR" : "NEAR", gRawData.ps_raw);

	if (gRawData.ps_state)
		p_status = 1;
	else
		p_status = 0;
	if (epld->mfg_mode != MFG_MODE && time_before(epld->j_end, (epld->j_start + NEAR_DELAY_TIME)) && gRawData.ps_state == 0)
		LOG_INFO("[PS] Ignore NEAR event");
	else {
		input_report_abs(epld->ps_input_dev, ABS_DISTANCE, gRawData.ps_state);
		input_sync(epld->ps_input_dev);
	}
}

static void epl88051_dyna_thd_read_ps(void)
{
	struct epl88051_priv *epld = epl88051_obj;
	struct i2c_client *client = epld->client;
	uint8_t setting;
#if 1 
    bool enable_ps = epld->enable_pflag == 1 && epld->ps_suspend == 0;
	bool enable_als = epld->enable_lflag == 1 && epld->als_suspend == 0;

    LOG_INFO("[%s]: enable_ps=%d, enable_als=%d \r\n", __func__, enable_ps, enable_als);
#endif 
	if(enable_ps == 1 && enable_als == 0) 
	{ 
		
		epl88051_I2C_Write(epld->client, REG_13, R_SINGLE_BYTE, 0x01, 0);
		epl88051_I2C_Read(epld->client);
		setting = gRawData.raw_bytes[0];
		if (((setting >> 3) & 7) != 0x01) {
			LOG_ERR("read ps data in wrong mode\n");
		}
		gRawData.ps_state= !((gRawData.raw_bytes[0] & 0x04) >> 2);
#if 1 
		gRawData.con_sat = (gRawData.raw_bytes[0] >> 1) & 0x01;
#endif 

	}

	
	
	if(enable_ps == 1 && enable_als == 0)
	{ 

#if 1 
		
		epl88051_I2C_Write(client, REG_14, R_TWO_BYTE, 0x01, 0x00);
		epl88051_I2C_Read(client);
		gRawData.ps_ch0_raw = (gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0];
#endif 

		
		epl88051_I2C_Write(client, REG_16, R_TWO_BYTE, 0x01, 0x00);
		epl88051_I2C_Read(client);
#ifdef PS_RAW_8BIT 
		if (gRawData.raw_bytes[1] >= (uint8_t) (epld->emmc_ps_kadc2 & 0xFF))
			gRawData.ps_raw = gRawData.raw_bytes[1] - (uint8_t) (epld->emmc_ps_kadc2 & 0xFF);  
		else
			gRawData.ps_raw = 0;
		epl_ps_raw_data = gRawData.raw_bytes[1];
#else
		if (((gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0]) >= (uint8_t) epld->emmc_ps_kadc2)
			gRawData.ps_raw = ((gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0]) - (uint8_t) epld->emmc_ps_kadc2; 
		else
			gRawData.ps_raw = 0;
		epl_ps_raw_data = (gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0];
#endif  
	}   
#if 1 
	else
	{
#ifdef PS_RAW_8BIT
		epl_ps_raw_data = gRawData.ps_raw + (uint8_t) (epld->emmc_ps_kadc2 & 0xFF);
#else
		epl_ps_raw_data = gRawData.ps_raw + (uint8_t) epld->emmc_ps_kadc2;
#endif
	}
#endif 

	LOG_INFO("[PS] %s proximity %s ps_raw_data: %d\n", __FUNCTION__, gRawData.ps_state ? "FAR" : "NEAR", gRawData.ps_raw);

	LOG_INFO("[%s] epl_ps_raw_data=%d, gRawData.con_sat=%d, gRawData.ps_ch0_raw=%d \n", __FUNCTION__, epl_ps_raw_data, gRawData.con_sat, gRawData.ps_ch0_raw);

	if (gRawData.ps_state)
		p_status = 1;
	else
		p_status = 0;
}

uint32_t raw_convert_to_lux(u16 raw_data)
{
	uint32_t lux = 0;

	
	
	if ((gRawData.als_ch1_raw * 42 / als_dynamic_intt_intt_value[dynamic_intt_idx]) > 65535)
		gRawData.als_raw = 65535;
	else
		gRawData.als_raw = gRawData.als_ch1_raw * 42 / als_dynamic_intt_intt_value[dynamic_intt_idx];

	lux = c_gain * raw_data * 15 / als_dynamic_intt_intt_value[dynamic_intt_idx];

	return lux;
}

static void epl88051_read_als(void)
{
	struct epl88051_priv *epld = epl88051_obj;
	struct i2c_client *client = epld->client;
	uint8_t now_gain;
	int ratio_flag = 0;
	uint32_t lux_temp;
	long luxratio = 0;
	int i;
	uint32_t als_step_temp = 0;
	u16 ch1, ch0;


	epl88051_I2C_Write(client, REG_13, R_SINGLE_BYTE, 0x01, 0);
	epl88051_I2C_Read(client);
	now_gain = gRawData.raw_bytes[0];
	if (((now_gain >> 3) & 7) != 0x00) {
		LOG_ERR("read als data in wrong mode %d\n", (now_gain >> 3) & 7);
	}
	
	epl88051_I2C_Write(client, REG_16, R_TWO_BYTE, 0x01, 0x00);
	epl88051_I2C_Read(client);
	ch1 = (gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0];
	
	epl88051_I2C_Write(client, REG_14, R_TWO_BYTE, 0x01, 0x00);
	epl88051_I2C_Read(client);
	ch0 = (gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0];
	if (ch0 == 0) {
		ch0 = 1;
		LOG_ERR("read ch0 data is 0 \r\n");
	}

	luxratio = (long)((ch1*dynamic_intt_min_unit) / (ch0)); 

	gRawData.ratio = luxratio;

	if (gRawData.ratio > epld->lsource_thd_high || gRawData.ratio == epld->lsource_thd_high) {
		c_gain = epld->c_gain_h;
		ratio_flag = 0;
	} else if (gRawData.ratio < epld->lsource_thd_low || gRawData.ratio == epld->lsource_thd_low) {
		c_gain = epld->c_gain_l;
		ratio_flag = 1;
	} else { 
		int a = 0, b = 0;
		a = (epld->c_gain_h - epld->c_gain_l) * dynamic_intt_min_unit / (epld->lsource_thd_high - epld->lsource_thd_low);
		b = (epld->c_gain_h) - ((a * epld->lsource_thd_high) / dynamic_intt_min_unit );
		c_gain = ((a * gRawData.ratio) / dynamic_intt_min_unit) + b;
	}

#if 0
	LOG_INFO("[LS] %s------------------- ch0 = %d, ch1 = %d, c_gain = %d \r\n\n", __FUNCTION__, ch0, ch1, c_gain);
	LOG_INFO("[LS] %s>>>>>>>>>>>>>>>>>>>>>>> luxratio=%ld, gRawData.ratio=%d", __FUNCTION__, luxratio, gRawData.ratio);
	LOG_INFO("[LS] %sdynamic_intt_idx=%d, als_dynamic_intt_intt_value=%d, dynamic_intt_gain=%d \r\n", __FUNCTION__,
			dynamic_intt_idx, als_dynamic_intt_intt_value[dynamic_intt_idx], dynamic_intt_gain);
#endif
	gRawData.als_ch0_raw = ch0;
	gRawData.als_ch1_raw = ch1;
	gRawData.als_ch0_raw_now = ch0;
	gRawData.als_ch1_raw_now = ch1;



	if (gRawData.als_ch1_raw > dynamic_intt_high_thr) {
		if (dynamic_intt_idx == (als_dynamic_intt_intt_num - 1)) {
			gRawData.als_ch1_raw = dynamic_intt_high_thr;
			if (ratio_flag == 0) {
				lux_temp = raw_convert_to_lux(gRawData.als_ch1_raw);
			} else {
				if (gRawData.als_ch0_raw > dynamic_intt_high_thr) {
					gRawData.als_ch0_raw = dynamic_intt_high_thr;
				}
				c_gain = epld->c_gain_saturated;
				lux_temp = raw_convert_to_lux(gRawData.als_ch0_raw);
			}

			
		} else {
			gRawData.als_ch1_raw = dynamic_intt_high_thr;
			lux_temp = raw_convert_to_lux(gRawData.als_ch1_raw);
			dynamic_intt_idx++;
			
		}
	} else if (gRawData.als_ch1_raw < dynamic_intt_low_thr) {
		if (dynamic_intt_idx == 0) {
			lux_temp = raw_convert_to_lux(gRawData.als_ch1_raw);
			
		} else {
			gRawData.als_ch1_raw = dynamic_intt_low_thr;
			lux_temp = raw_convert_to_lux(gRawData.als_ch1_raw);
			dynamic_intt_idx--;
			
		}
	} else {
		lux_temp = raw_convert_to_lux(gRawData.als_ch1_raw);
	}
	if (!epld->ls_calibrate) {
		als_step_temp = gRawData.als_raw;
		gRawData.als_raw = als_step_temp * epld->als_gadc / epld->als_kadc;
		if((gRawData.als_raw * epld->als_kadc) < (als_step_temp * epld->als_gadc)) {
			gRawData.als_raw++;
		}
		if (gRawData.als_raw > 0xFFFF)
			gRawData.als_raw = 0xFFFF;
	}


	for (i = 0; i < 10; i++) {
		if (gRawData.als_raw <= (*(epld->adc_table + i))) {
			gRawData.als_level = i;
			if (*(epld->adc_table + i))
				break;
		}
		if (i == 9) {
			gRawData.als_level = i;
			break;
		}
	}
	gRawData.als_lux = lux_temp  / dynamic_intt_min_unit;
	LOG_INFO("[LS] %s-------------------  ALS raw = %d, lux = %d\n\n", __FUNCTION__, gRawData.als_ch1_raw, gRawData.als_lux);
	dynamic_intt_intt = als_dynamic_intt_intt[dynamic_intt_idx];
	dynamic_intt_gain = als_dynamic_intt_gain[dynamic_intt_idx];
	dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
	dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
}

static void epl88051_read_hs(void)
{
	struct epl88051_priv *epld = epl88051_obj;
	struct i2c_client *client = epld->client;
	u16 data;
	
	int idx = start_idx+hs_count;
	mutex_lock(&sensor_mutex);
	epl88051_I2C_Write(client, REG_16, R_EIGHT_BYTE, 0x01, 0x00);
	epl88051_I2C_Read(client);
	data=((gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0]) >> 6;

	if (data > 950 && HS_INTT > 0)
		HS_INTT--;

	if (idx >= HS_BUFFER_SIZE)
		idx -= HS_BUFFER_SIZE;

	gRawData.hs_data[idx] = data;

	if (hs_count >= HS_BUFFER_SIZE) {
		start_idx++;
		if(start_idx >= HS_BUFFER_SIZE)
			start_idx = 0;
	}

	hs_count++;
	if (hs_count >= HS_BUFFER_SIZE)
		hs_count = HS_BUFFER_SIZE;
	mutex_unlock(&sensor_mutex);
}

static int epl88051_set_ps_threshold(uint16_t low_thd, uint16_t high_thd)

{
	int ret = 0;
	struct epl88051_priv *epld = epl88051_obj;
	struct i2c_client *client = epld->client;
	uint8_t high_thd_h = 0;
	uint8_t high_thd_l = 0;
	uint8_t low_thd_h = 0;
	uint8_t low_thd_l = 0;
	high_thd_h = (high_thd >> 8) & 0xFF;
	high_thd_l = high_thd & 0xFF;
	low_thd_h = (low_thd >> 8) & 0xFF;
	low_thd_l = low_thd & 0xFF;
#ifdef PS_RAW_8BIT 
	LOG_INFO("[PS] %s high_thd_l:%d, low_thd_l:%d\n", __FUNCTION__, high_thd_l, low_thd_l);
	epl88051_I2C_Write(client,REG_2, W_SINGLE_BYTE, 0x02, 0);
	epl88051_I2C_Write(client,REG_3, W_SINGLE_BYTE, 0x02, high_thd_l);
	epl88051_I2C_Write(client,REG_4, W_SINGLE_BYTE, 0x02, 0);
	epl88051_I2C_Write(client,REG_5, W_SINGLE_BYTE, 0x02, low_thd_l);
#else
	epl88051_I2C_Write(client,REG_2, W_SINGLE_BYTE, 0x02, high_thd_l);
	epl88051_I2C_Write(client,REG_3, W_SINGLE_BYTE, 0x02, high_thd_h);
	epl88051_I2C_Write(client,REG_4, W_SINGLE_BYTE, 0x02, low_thd_l);
	epl88051_I2C_Write(client,REG_5, W_SINGLE_BYTE, 0x02, low_thd_h);
#endif 
	return ret;
}
static void epl88051_dyna_thd_polling_work(struct work_struct *work)
{
	struct epl88051_priv *epld = epl88051_obj;
#if 1 
    bool enable_ps = epld->enable_pflag == 1 && epld->ps_suspend == 0;
	bool enable_als = epld->enable_lflag == 1 && epld->als_suspend == 0;
    LOG_INFO("[%s]: enable_ps=%d, enable_als=%d \r\n", __FUNCTION__, enable_ps, enable_als);
	if (psensor_enabled == 1 && enable_ps == 1) {
#else
    if (psensor_enabled == 1) {
#endif 
		epl88051_dyna_thd_read_ps();
		LOG_INFO("[PS] epl_ps_raw_data:%d, min_epl_ps_raw_data:%d\n", epl_ps_raw_data, min_epl_ps_raw_data);
		if (epl_ps_raw_data != 0) {
			if ((min_epl_ps_raw_data > epl_ps_raw_data) && gRawData.con_sat == 0 && gRawData.ps_ch0_raw < epld->ps_max_ch0) { 
				min_epl_ps_raw_data = epl_ps_raw_data;
				epld->ps_threshold_low = min_epl_ps_raw_data + TH_ADD;
				epld->ps_threshold_high = epld->ps_threshold_low + epld->ps_threshold_diff;
				if (epld->ps_threshold_low > 254)
					epld->ps_threshold_low = 254;
				if (epld->ps_threshold_high > 255)
					epld->ps_threshold_high = 255;
				LOG_INFO("[PS] set low thd:%d  .........................\n", epld->ps_threshold_low);

                if(enable_ps == 1 && enable_als == 0) 
                { 
				    epl88051_set_ps_threshold(epld->ps_threshold_low, epld->ps_threshold_high);
                } 
			}
		}
		queue_delayed_work(epld->epl_wq, &dyna_thd_polling_work,
				msecs_to_jiffies(POLLING_DELAY));
	}
}
static void epl88051_polling_work(struct work_struct *work)
{
	struct epl88051_priv *epld = epl88051_obj;
	struct i2c_client *client = epld->client;

	bool enable_ps = epld->enable_pflag == 1 && epld->ps_suspend == 0;
	bool enable_als = epld->enable_lflag == 1 && epld->als_suspend == 0;
	bool enable_hs = epld->enable_hflag == 1 && epld->hs_suspend == 0;

	

	cancel_delayed_work(&polling_work);
	if (enable_als == true) {
		queue_delayed_work(epld->epl_wq, &polling_work, msecs_to_jiffies(ALS_DELAY + 2 * PS_DELAY + 30));
	}

	if (enable_hs) {
		if (epld->polling_mode_hs == 0) {
			epl88051_hs_enable(epld, true, true);
		} else {
			epl88051_read_hs();
			epl88051_hs_enable(epld, false, true);
			queue_delayed_work(epld->epl_wq, &polling_work, msecs_to_jiffies(10));
		}
	} else {
		if (enable_als) {
			
			epl88051_lsensor_enable(epld);
			
			epl88051_read_als();
			
#ifdef outputLUX
			input_report_abs(epld->als_input_dev, ABS_MISC, gRawData.als_lux);
#else
			input_report_abs(epld->als_input_dev, ABS_MISC, gRawData.als_level);
#endif
			input_sync(epld->als_input_dev);
		}

		if (enable_ps) {
			
			epl88051_psensor_enable(epld);
			
			if (epld->polling_mode_ps == 1) {
				epl88051_read_ps();
			}
		}
	}


	if (enable_als == false && enable_ps == false && enable_hs == false) {
		cancel_delayed_work(&polling_work);
		LOG_INFO("%s disable sensor\n", __FUNCTION__);
		epl88051_I2C_Write(client,REG_6, W_SINGLE_BYTE, 0x02, EPL_INT_DISABLE);
		epl88051_I2C_Write(client,REG_8, W_SINGLE_BYTE, 0X02, EPL_C_P_DOWN);
	}

}

static irqreturn_t epl88051_eint_func(int irqNo, void *handle)
{
	struct epl88051_priv *epld = (struct epl88051_priv*) handle;

	disable_irq_nosync(epld->irq);
	queue_work(epld->epl_wq, &epl_sensor_irq_work);
	return IRQ_HANDLED;
}

static void epl88051_eint_work(struct work_struct *work)
{
	struct epl88051_priv *epld = epl88051_obj;
	struct i2c_client *client = epld->client;
	int mode = 0;

	if (epld->enable_hflag) {
		epl88051_read_hs();
		epl88051_hs_enable(epld, true, true);
	} else if (epld->enable_pflag) {
		
		epl88051_I2C_Write(client, REG_13, R_SINGLE_BYTE, 0x01, 0);
		epl88051_I2C_Read(client);
		mode = (gRawData.raw_bytes[0] >> 3) & 7;

		if (mode == 0x01 && epld->enable_pflag) {
			
			epl88051_read_ps();
		} else {
			LOG_ERR("error: interrupt in ps\n");
		}

		
		epl88051_I2C_Write(client, REG_6, W_SINGLE_BYTE, 0x02, EPL_INT_CH1 | EPL_INT_ACTIVE_LOW);

		
		epl88051_I2C_Write(client, REG_8, W_SINGLE_BYTE, 0x02, EPL_DATA_UNLOCK);
	}

	enable_irq(epld->irq);
}

static int epl88051_setup_interrupt(struct epl88051_priv *epld)
{
	struct i2c_client *client = epld->client;
	int err = 0;
	msleep(5);
	err = gpio_request(epld->intr_pin, "gpio_epl88051_intr");
	if (err < 0) {
		LOG_ERR("gpio %d request failed (%d)\n", epld->intr_pin, err);
		goto initial_fail;
	}

	err = gpio_direction_input(epld->intr_pin);
	if (err < 0) {
		LOG_ERR("fail to set gpio %d as input (%d)\n", epld->intr_pin, err);
		goto fail_free_intr_pin;
	}

	err = request_irq(epld->irq, epl88051_eint_func, IRQF_TRIGGER_FALLING,
			client->dev.driver->name, epld);
	if (err <0) {
		LOG_ERR("request irq pin %d fail for gpio\n",err);
		goto fail_free_intr_pin;
	}
	return err;
initial_fail:
fail_free_intr_pin:
	gpio_free(epld->intr_pin);
	return err;
}

static ssize_t ls_adc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct epl88051_priv *epld = epl88051_obj;
	

	if (epld->enable_lflag == 0 && epld->enable_pflag == 0) {
		epl88051_lsensor_enable(epld);
		
		epl88051_read_als();
	} else {
		epld->enable_lflag = 1;
		epl88051_restart_work();
	}
	LOG_INFO("[LS] %s: ADC = 0x%04X, Level = %d \n",
			__func__, gRawData.als_raw, gRawData.als_level);
	ret = sprintf(buf, "ADC[0x%04X] => level %d\n",
			gRawData.als_raw, gRawData.als_level);

	return ret;
}

static DEVICE_ATTR(ls_adc, 0664, ls_adc_show, NULL);

static ssize_t ls_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct epl88051_priv *epld = epl88051_obj;

	ret = sprintf(buf, "Light sensor Auto Enable = %d\n",
			epld->enable_lflag);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ls_auto;
	struct epl88051_priv *epld = epl88051_obj;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1 && ls_auto != 147 && ls_auto != 148) {
		return -EINVAL;
	}
	if (ls_auto) {
		epld->ls_calibrate = (ls_auto == 147) ? 1 : 0;
		epld->ws_calibrate = (ls_auto == 148) ? 1 : 0;
	} else {
		epld->ls_calibrate = 0;
		epld->ws_calibrate = 0;
	}

	LOG_INFO("[LS] %s lpi->ls_calibrate = %d, ls_auto=%d\n", __FUNCTION__, epld->ls_calibrate, ls_auto);

	return count;
}

static DEVICE_ATTR(ls_auto, 0664, ls_enable_show, ls_enable_store);

static ssize_t epl88051_show_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct i2c_client *client = epl88051_obj->client;


	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x00 value = %8x\n", i2c_smbus_read_byte_data(client, 0x00));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x01 value = %8x\n", i2c_smbus_read_byte_data(client, 0x08));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x02 value = %8x\n", i2c_smbus_read_byte_data(client, 0x10));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x03 value = %8x\n", i2c_smbus_read_byte_data(client, 0x18));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x04 value = %8x\n", i2c_smbus_read_byte_data(client, 0x20));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x05 value = %8x\n", i2c_smbus_read_byte_data(client, 0x28));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x06 value = %8x\n", i2c_smbus_read_byte_data(client, 0x30));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x07 value = %8x\n", i2c_smbus_read_byte_data(client, 0x38));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x09 value = %8x\n", i2c_smbus_read_byte_data(client, 0x48));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0D value = %8x\n", i2c_smbus_read_byte_data(client, 0x68));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0E value = %8x\n", i2c_smbus_read_byte_data(client, 0x70));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0F value = %8x\n", i2c_smbus_read_byte_data(client, 0x71));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x10 value = %8x\n", i2c_smbus_read_byte_data(client, 0x80));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x11 value = %8x\n", i2c_smbus_read_byte_data(client, 0x88));
	len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x17 value = %8x\n", i2c_smbus_read_byte_data(client, 0xB8));

	return len;
}
static DEVICE_ATTR(elan_reg, 0644, epl88051_show_reg, NULL);
static ssize_t epl88051_show_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	size_t len = 0;
	if (!epl88051_obj) {
		LOG_ERR("epl88051_obj is null!!\n");
		return 0;
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "als_lux=%d    \n", gRawData.als_lux);
	len += snprintf(buf+len, PAGE_SIZE-len, "als_level=%d    \n", gRawData.als_level);
	len += snprintf(buf+len, PAGE_SIZE-len, "als_raw=%d    \n", gRawData.als_raw);
	len += snprintf(buf+len, PAGE_SIZE-len, "als_ch0_raw_now=%d   \n", gRawData.als_ch0_raw_now);
	len += snprintf(buf+len, PAGE_SIZE-len, "als_ch0_raw=%d   \n", gRawData.als_ch0_raw);
	len += snprintf(buf+len, PAGE_SIZE-len, "als_ch1_raw_now=%d   \n", gRawData.als_ch1_raw_now);
	len += snprintf(buf+len, PAGE_SIZE-len, "als_ch1_raw=%d   \n", gRawData.als_ch1_raw);
	len += snprintf(buf+len, PAGE_SIZE-len, "ratio=%d    \n",gRawData.ratio);
	len += snprintf(buf+len, PAGE_SIZE-len, "c_gain=%d       \n", c_gain);
	len += snprintf(buf+len, PAGE_SIZE-len, "INTT=%d  \n", als_dynamic_intt_intt_value[dynamic_intt_idx]);
	len += snprintf(buf+len, PAGE_SIZE-len, "ps_state=%d  \n", gRawData.ps_state);
	len += snprintf(buf+len, PAGE_SIZE-len, "ps_raw=%d  \n", gRawData.ps_raw);

	return len;
}
static DEVICE_ATTR(elan_status, 0664, epl88051_show_status, NULL);

static ssize_t epl88051_store_als_int_time(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int input = 0;
	LOG_FUN();

	sscanf(buf, "%d", &input);
	dynamic_intt_intt = (uint8_t) input;

	return count;
}

static DEVICE_ATTR(als_int_time, 0664, NULL, epl88051_store_als_int_time);
static ssize_t epl88051_show_ps_cal_raw(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct epl88051_priv *obj = epl88051_obj;
	u16 *tmp = (u16*)buf;
	int ch1;
	int ch1_all = 0;
	int avg_count = 5;
	int i;


	obj->als_suspend = 1;
	for (i = 0; i < avg_count; i++) {
		msleep(obj->ps_delay);

		if (obj->polling_mode_ps) {
			ch1_all = ch1_all + gRawData.ps_raw;
		} else {
			epl88051_I2C_Write(obj->client, REG_8, W_SINGLE_BYTE, 0x02, EPL_DATA_LOCK);
			epl88051_I2C_Write(obj->client, REG_16, R_TWO_BYTE, 0x01, 0x00);
			epl88051_I2C_Read(obj->client);
#ifdef PS_RAW_8BIT 
			ch1_all = ch1_all + gRawData.raw_bytes[1];
#else
			ch1_all = ch1_all + ((gRawData.raw_bytes[1] << 8) | gRawData.raw_bytes[0]);
#endif
			epl88051_I2C_Write(obj->client, REG_8, W_SINGLE_BYTE, 0x02, EPL_DATA_UNLOCK);
		}
	}

	ch1 = ch1_all / avg_count;
	obj->als_suspend = 0;
	tmp[0] = ch1;

	return  2;
}
static DEVICE_ATTR(ps_cal_raw, 0664, epl88051_show_ps_cal_raw, NULL);

static ssize_t epl88051_store_ps_int_time(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	LOG_FUN();

	sscanf(buf, "%d", &PS_INTT);

	return count;
}
static DEVICE_ATTR(ps_int_time, 0664, NULL, epl88051_store_ps_int_time);

static ssize_t epl88051_store_ps_threshold(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct epl88051_priv *epld = epl88051_obj;
	LOG_FUN();

	sscanf(buf, "%d%d", &epld->ps_threshold_low, &epld->ps_threshold_high);

	return count;
}
static DEVICE_ATTR(ps_threshold, 0664, NULL, epl88051_store_ps_threshold);

static ssize_t epl88051_store_ps_polling_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct epl88051_priv *epld = epl88051_obj;
	LOG_FUN();

	sscanf(buf, "%d",&epld->polling_mode_ps);

	if (epld->polling_mode_ps == 0)
		epl88051_setup_interrupt(epld);

	return count;
}
static DEVICE_ATTR(ps_polling_mode, 0664, NULL, epl88051_store_ps_polling_mode);

#if 1 
static ssize_t epl88051_store_ps_max_ch0(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct epl88051_priv *epld = epl88051_obj;
	int ps_ch0=0;
	LOG_FUN();

	sscanf(buf, "%d",&ps_ch0);
    epld->ps_max_ch0 = ps_ch0;

	return count;
}
static DEVICE_ATTR(dyn_max_ps_ch0, 0664, NULL, epl88051_store_ps_max_ch0);
#endif 

static ssize_t epl88051_store_hs_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	uint16_t mode = 0;
	struct epl88051_priv *obj = epl88051_obj;
	LOG_FUN();

	sscanf(buf, "%hu", &mode);

	if (mode)
		obj->enable_hflag = 1;
	else
		obj->enable_hflag = 0;

	if (mode) {
		HS_INTT = 15;
		start_idx = 0;
		hs_count = 0;
	}

	epl88051_restart_work();
	return count;
}
static DEVICE_ATTR(hs_enable, 0644, NULL, epl88051_store_hs_enable);

static ssize_t epl88051_show_hs_raws(struct device *dev, struct device_attribute *attr, char *buf)
{
	u16 *tmp = (u16*)buf;
	u16 length = hs_count * 1;
	int byte_count = 2 + length * 2;
	int i = 0;
	int start = 0;
	mutex_lock(&sensor_mutex);
	tmp[0] = length;
	for (i = start; i < length; i++)
		tmp[i+1] = gRawData.hs_data[i];
	hs_count = 0;
	mutex_unlock(&sensor_mutex);
	return byte_count;
}
static DEVICE_ATTR(hs_raws, 0644, epl88051_show_hs_raws, NULL);
#if 0
static struct attribute *epl88051_attr_list[] =
{
	&dev_attr_hs_enable.attr,
	&dev_attr_hs_raws.attr,
};
static struct attribute_group epl88051_attr_group =
{
	.attrs = epl88051_attr_list,
};
#endif
static ssize_t ps_adc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	int ps_adc = 0;
	int ret;
	struct  epl88051_priv *epl88051 = epl88051_obj;
	int int_gpio;

	int_gpio = gpio_get_value_cansleep(epl88051->intr_pin);
	if (epl88051->enable_pflag == 0 && epl88051->enable_lflag == 0) {
		epl88051_psensor_enable(epl88051);
		epl88051_dyna_thd_read_ps();
	} else if (epl88051->enable_pflag == 1 && epl88051->enable_lflag == 0){
		
		epl88051_I2C_Write(epl88051->client, REG_16, R_TWO_BYTE, 0x01, 0x00);
		epl88051_I2C_Read(epl88051->client);
#ifdef PS_RAW_8BIT 
		if (gRawData.raw_bytes[1] >= (uint8_t) (epl88051->emmc_ps_kadc2 & 0xFF))
			gRawData.ps_raw = gRawData.raw_bytes[1] - (uint8_t) (epl88051->emmc_ps_kadc2 & 0xFF);  
		else
			gRawData.ps_raw = 0;
#else
		if (((gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0]) >= (uint8_t) epl88051->emmc_ps_kadc2)
			gRawData.ps_raw = ((gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0]) - (uint8_t) epl88051->emmc_ps_kadc2; 
		else
			gRawData.ps_raw = 0;
#endif  

	} else {
		epl88051->enable_pflag = 1;
	}
	ps_adc = gRawData.ps_raw;
	ret = sprintf(buf, "ADC[0x%02X], ENABLE = %d, intr_pin = %d, "
			"ps_pocket_mode = %d\n",
			ps_adc, epl88051->enable_pflag, int_gpio, epl88051->ps_pocket_mode);

	return ret;
}
static ssize_t ps_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ps_en;
	struct  epl88051_priv *epl88051 = epl88051_obj;

	ps_en = -1;
	sscanf(buf, "%d", &ps_en);

	if (ps_en != 0 && ps_en != 1
			&& ps_en != 10 && ps_en != 13 && ps_en != 16)
		return -EINVAL;

	LOG_INFO("[PS] %s: ps_en=%d\n",	__func__, ps_en);

	if (ps_en && !epl88051->enable_pflag) {
		epl88051_restart_work();
		epl88051->enable_pflag = 1;
	} else if (!ps_en && epl88051->enable_pflag) {
		epl88051->enable_pflag = 0;
	}


	return count;

}

static DEVICE_ATTR(ps_adc, 0664, ps_adc_show, ps_enable_store);

static ssize_t ps_canc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct epl88051_priv *epld = epl88051_obj;

	ret = sprintf(buf, "PS1_CANC = 0x%02X, PS2_CANC = 0x%02X\n",
			epld->emmc_ps_kadc2 & 0xff, (epld->emmc_ps_kadc2 >> 16) & 0xff);

	return ret;
}
static ssize_t ps_canc_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ps1_canc = 0;
	int ps2_canc = 0;
	struct epl88051_priv *epld = epl88051_obj;

	sscanf(buf, "0x%x 0x%x", &ps1_canc, &ps2_canc);
	epld->emmc_ps_kadc2 = (ps2_canc << 16) | ps1_canc;
	epld->ps_threshold_high = ps1_canc + ps2_canc + epld->ps_threshold_diff;
	epld->ps_threshold_low = ps1_canc + ps2_canc;
	epl88051_set_ps_threshold(epld->ps_threshold_low,epld->ps_threshold_high);

	LOG_INFO("[PS] %s PS1_CANC = 0x%02X, PS2_CANC = 0x%02X\n", __FUNCTION__, ps1_canc, ps2_canc);

	return count;
}
static DEVICE_ATTR(ps_canc, 0664, ps_canc_show, ps_canc_store);
static int kcalibrated;
static ssize_t ps_kadc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct epl88051_priv *epld = epl88051_obj;
	if ((epld->emmc_ps_kadc1 >> 16 == PS_CALIBRATED) || kcalibrated == 1)
		ret = sprintf(buf, "P-sensor calibrated,"
				"INTE_PS1_CANC = (0x%02X), "
				"INTE_PS2_CANC = (0x%02X)\n",
				(uint8_t) (epld->emmc_ps_kadc2 & 0xFF), (uint8_t) ((epld->emmc_ps_kadc2 >> 16) & 0xFF));
	else
		ret = sprintf(buf, "P-sensor NOT calibrated,"
				"INTE_PS1_CANC = (0x%02X), "
				"INTE_PS2_CANC = (0x%02X)\n",
				(uint8_t) (epld->emmc_ps_kadc2 & 0xFF), (uint8_t) ((epld->emmc_ps_kadc2 >> 16) & 0xFF));

	return ret;
}

static ssize_t ps_kadc_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int param1, param2;
	uint8_t ps1_canc = 0;
	uint8_t ps2_canc = 0;
	struct epl88051_priv *epld = epl88051_obj;
	sscanf(buf, "0x%x 0x%x", &param1, &param2);
	LOG_INFO("[PS] %s: store value = 0x%X, 0x%X\n", __func__, param1, param2);


	epld->emmc_ps_kadc2 = param2;
	ps1_canc = (uint8_t) (epld->emmc_ps_kadc2 & 0xFF);
	ps2_canc = (uint8_t) ((epld->emmc_ps_kadc2 >> 16) & 0xFF);
	epld->ps_threshold_low = ps1_canc + ps2_canc;
        epld->ps_threshold_high = epld->ps_threshold_low + epld->ps_threshold_diff;
	epl88051_set_ps_threshold(epld->ps_threshold_low,epld->ps_threshold_high);

	LOG_INFO("[PS] %s: ps1_canc = 0x%02X, ps2_canc = 0x%02X\n", __func__, ps1_canc, ps2_canc);
	kcalibrated = 1;
	return count;
}

static DEVICE_ATTR(ps_kadc, 0664, ps_kadc_show, ps_kadc_store);

static ssize_t p_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%d\n",p_status);
}
static DEVICE_ATTR(p_status, 0444, p_status_show, NULL);

static int epl88051_als_open(struct inode *inode, struct file *file)
{
	struct epl88051_priv *epld = epl88051_obj;

	LOG_FUN();

	if (epld->als_opened) {
		return -EBUSY;
	}
	epld->als_opened = 1;

	return 0;
}

static int epl88051_als_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	struct epl88051_priv *epld = epl88051_obj;
	int buf[1];
	if (epld->read_flag ==1) {
		buf[0] = gRawData.als_ch1_raw;
		if(copy_to_user(buffer, &buf , sizeof(buf)))
			return 0;
		epld->read_flag = 0;
		return 12;
	} else {
		return 0;
	}
}

static int epl88051_als_release(struct inode *inode, struct file *file)
{
	struct epl88051_priv *epld = epl88051_obj;

	LOG_FUN();

	epld->als_opened = 0;

	return 0;
}

static long epl88051_als_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int flag;
	unsigned long buf[1];
	struct epl88051_priv *epld = epl88051_obj;
	void __user *argp = (void __user *)arg;

	LOG_INFO("[LS] %s als io ctrl cmd %d\n", __FUNCTION__, _IOC_NR(cmd));

	switch(cmd) {
		
		case ELAN_EPL6800_IOCTL_GET_LFLAG:
		case LIGHTSENSOR_IOCTL_GET_ENABLED:

			LOG_INFO("[LS] %s elan ambient-light IOCTL Sensor get lflag \n", __FUNCTION__);
			flag = epld->enable_lflag;
			if (copy_to_user(argp, &flag, sizeof(flag)))
				return -EFAULT;

			LOG_INFO("[LS] %s elan ambient-light Sensor get lflag %d\n", __FUNCTION__, flag);
			break;
			
		case ELAN_EPL6800_IOCTL_ENABLE_LFLAG:
		case LIGHTSENSOR_IOCTL_ENABLE:
			LOG_INFO("[LS] %s elan ambient-light IOCTL Sensor set lflag \n", __FUNCTION__);
			if (copy_from_user(&flag, argp, sizeof(flag)))
				return -EFAULT;
			if (flag < 0 || flag > 1)
				return -EINVAL;

			dynamic_intt_idx = dynamic_intt_init_idx;
			dynamic_intt_intt = als_dynamic_intt_intt[dynamic_intt_idx];
			dynamic_intt_gain = als_dynamic_intt_gain[dynamic_intt_idx];
			dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
			dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
			gRawData.ratio = 0;
			epld->enable_lflag = flag;
			epl88051_restart_work();

			LOG_INFO("[LS] %s elan ambient-light Sensor set lflag %d\n", __FUNCTION__, flag);
			break;
			
		case ELAN_EPL6800_IOCTL_GETDATA:
			buf[0] = (unsigned long)gRawData.als_ch1_raw;
			if (copy_to_user(argp, &buf , sizeof(buf)))
				return -EFAULT;

			break;

		default:
			LOG_ERR("invalid cmd %d\n", _IOC_NR(cmd));
			return -EINVAL;
	}

	return 0;
}

static struct file_operations epl88051_als_fops =
{
	.owner = THIS_MODULE,
	.open = epl88051_als_open,
	.read = epl88051_als_read,
	.release = epl88051_als_release,
	.unlocked_ioctl = epl88051_als_ioctl
};

static struct miscdevice epl88051_als_device =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor", 
	.fops = &epl88051_als_fops
};

static int epl88051_ps_open(struct inode *inode, struct file *file)
{
	struct epl88051_priv *epld = epl88051_obj;

	LOG_FUN();

	if (epld->ps_opened)
		return -EBUSY;

	epld->ps_opened = 1;
	return 0;
}

static int epl88051_ps_release(struct inode *inode, struct file *file)
{
	struct epl88051_priv *epld = epl88051_obj;

	LOG_FUN();
	epld->ps_opened = 0;
	return 0;
}

static long epl88051_ps_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int value;
	int flag;
	struct epl88051_priv *epld = epl88051_obj;

	void __user *argp = (void __user *)arg;

	LOG_INFO("[PS] %s ps io ctrl cmd %d\n", __FUNCTION__, _IOC_NR(cmd));

	
	switch(cmd) {
		
		
		case CAPELLA_CM3602_IOCTL_GET_ENABLED:
			LOG_INFO("[PS] %s elan Proximity Sensor IOCTL get pflag \n", __FUNCTION__);
			flag = epld->enable_pflag;
			if (copy_to_user(argp, &flag, sizeof(flag)))
				return -EFAULT;

			LOG_INFO("[PS] %s elan Proximity Sensor get pflag %d\n", __FUNCTION__, flag);
			break;
		
		
		case CAPELLA_CM3602_IOCTL_ENABLE:
			LOG_INFO("[PS] %s elan Proximity IOCTL Sensor set pflag \n", __FUNCTION__);
			if (copy_from_user(&flag, argp, sizeof(flag)))
				return -EFAULT;
			if (flag < 0 || flag > 1)
				return -EINVAL;

			if (flag && epld->polling_mode_ps == 0)
				gRawData.ps_state = 2;

			epld->enable_pflag = flag;

			if (flag)
				p_status = 1;
			else
				p_status = 9;

			epl88051_restart_work();
			LOG_INFO("[PS] epld->mfg_mode:%d\n", epld->mfg_mode);
			if (flag && epld->mfg_mode != MFG_MODE) {
				min_epl_ps_raw_data = 255;
				epld->j_start = jiffies;
				LOG_INFO("[PS] default report FAR ");
				input_report_abs(epld->ps_input_dev, ABS_DISTANCE, 1);
				input_sync(epld->ps_input_dev);
				
			    
			}

			if (!flag) {
				psensor_enabled = 0;
				cancel_delayed_work(&dyna_thd_polling_work);
			}
			LOG_INFO("[PS] %s elan Proximity Sensor set pflag %d\n", __FUNCTION__, flag);
			break;
		
		case ELAN_EPL6800_IOCTL_GETDATA:

			value = gRawData.ps_raw;
			if (copy_to_user(argp, &value , sizeof(value)))
				return -EFAULT;

			LOG_INFO("[PS] %s elan proximity Sensor get data (%d) \n", __FUNCTION__, value);
			break;

		default:
			LOG_ERR("invalid cmd %d\n", _IOC_NR(cmd));
			return -EINVAL;
	}

	return 0;
}

static struct file_operations epl88051_ps_fops =
{
	.owner = THIS_MODULE,
	.open = epl88051_ps_open,
	.release = epl88051_ps_release,
	.unlocked_ioctl = epl88051_ps_ioctl
};

static struct miscdevice epl88051_ps_device =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cm3602", 
	.fops = &epl88051_ps_fops
};

static int epl88051_initial_sensor(struct epl88051_priv *epld)
{
	struct i2c_client *client = epld->client;

	int ret = 0;

	LOG_INFO("%s epl88051_initial_sensor enter!\n", __FUNCTION__);

	ret = epl88051_I2C_Read(client);

	if (ret < 0)
		return -EINVAL;
	
	epl88051_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0x02, EPL_S_SENSING_MODE);
	epl88051_I2C_Write(client, REG_6, W_SINGLE_BYTE, 0x02, EPL_INT_DISABLE);

	msleep(2);

	epld->enable_lflag = 0;
	epld->enable_pflag = 0;

	return ret;
}

static ssize_t light_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct epl88051_priv *epld  = epl88051_obj;
	printk("%s: ALS_status=%d\n", __func__, epld->enable_lflag);
	return sprintf(buf, "%d\n", epld->enable_lflag);
}

static ssize_t light_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct epl88051_priv *epld = epl88051_obj;

	LOG_INFO("[LS] %s light_enable_store: enable=%s \n", __FUNCTION__, buf);

	if (sysfs_streq(buf, "1"))
		epld->enable_lflag = 1;
	else if (sysfs_streq(buf, "0"))
		epld->enable_lflag = 0;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return 0;
	}

	epl88051_restart_work();
	return size;
}

static ssize_t ls_kadc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct epl88051_priv *epld = epl88051_obj;
	int ret;

	ret = sprintf(buf, "kadc = 0x%x, gadc = 0x%x, kadc while this boot = 0x%x\n",
			epld->als_kadc, epld->als_gadc, epld->emmc_als_kadc);
	return ret;
}

static ssize_t ls_kadc_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct epl88051_priv *epld = epl88051_obj;
	int kadc_temp = 0;

	sscanf(buf, "%d", &kadc_temp);
	LOG_INFO("[LS] %s KADC:%d\n", __FUNCTION__, kadc_temp);
	if (kadc_temp <= 0 || epld->golden_adc <= 0) {
		LOG_ERR("kadc_temp=0x%x, als_gadc=0x%x\n", kadc_temp, epld->golden_adc);
		return -EINVAL;
	}

	if (!epld->ws_calibrate) {
		epld->als_kadc = kadc_temp;
		epld->als_gadc = epld->golden_adc;
		LOG_ERR("als_kadc=0x%x, als_gadc=0x%x\n", epld->als_kadc, epld->als_gadc);
	}

	return count;
}

static DEVICE_ATTR(ls_kadc, 0664, ls_kadc_show, ls_kadc_store);

static ssize_t ls_adc_table_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned length = 0;
	int i;

	for (i = 0; i < 10; i++) {
		length += sprintf(buf + length,
				"Get adc_table[%d] =  0x%x ; %d\n",
				i, *(epl88051_obj->adc_table + i),
				*(epl88051_obj->adc_table + i));
	}
	return length;
}

static ssize_t ls_adc_table_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct epl88051_priv *epld = epl88051_obj;
	char *token[10];
	unsigned long tempdata[10];
	int i, ret;

	LOG_INFO("[LS] %s\n", __FUNCTION__);
	for (i = 0; i < 10; i++) {
		token[i] = strsep((char **)&buf, " ");
		ret = strict_strtoul(token[i], 16, &(tempdata[i]));
		if (tempdata[i] < 1 || tempdata[i] > 0xffff) {
			LOG_ERR(" adc_table[%d] =  0x%lx Err\n", i, tempdata[i]);
			return count;
		}
	}
	for (i = 0; i < 10; i++) {
		epld->adc_table[i] = tempdata[i];
		LOG_INFO("[LS] %s Set adc_table[%d] =  0x%x\n", __FUNCTION__, i, *(epl88051_obj->adc_table + i));
	}

	return count;
}

static DEVICE_ATTR(ls_adc_table, 0664,
		ls_adc_table_show, ls_adc_table_store);

static struct device_attribute dev_attr_light_enable =
	__ATTR(enable, S_IRWXUGO, light_enable_show, light_enable_store);

static struct attribute *light_sysfs_attrs[] =
{
	&dev_attr_light_enable.attr,
	NULL
};

static struct attribute_group light_attribute_group =
{
	.attrs = light_sysfs_attrs,
};

static void lightsensor_set_kvalue(struct epl88051_priv *epld)
{
	if (!epld) {
		LOG_ERR("ls_info is empty\n");
		return;
	}

	LOG_INFO("[LS] %s ALS calibrated als_kadc=0x%x\n", __FUNCTION__, epld->emmc_als_kadc);

	if (epld->emmc_als_kadc >> 16 == ALS_CALIBRATED) {
		epld->als_kadc = epld->emmc_als_kadc & 0xFFFF;
		lightsensor_cali = 1;
	} else {
		epld->als_kadc = 0;
		lightsensor_cali = 0;
		LOG_INFO("[LS] %s : no ALS calibrated\n", __FUNCTION__);
	}
	if (epld->als_kadc && epld->golden_adc > 0) {
		epld->als_kadc = (epld->als_kadc > 0) ?
			epld->als_kadc : epld->golden_adc;
		epld->als_gadc = epld->golden_adc;
	} else {
		epld->als_kadc = 1;
		epld->als_gadc = 1;
	}
	LOG_INFO("[LS] %s als_kadc=0x%x, als_gadc=0x%x\n", __FUNCTION__, epld->als_kadc, epld->als_gadc);
}
static int epl88051_setup_lsensor(struct epl88051_priv *epld)
{
	int err = 0;

	LOG_INFO("[LS] %s epl88051_setup_lsensor enter.\n", __FUNCTION__);

	epld->als_input_dev = input_allocate_device();
	if (!epld->als_input_dev) {
		LOG_ERR( "could not allocate ls input device\n");
		return -ENOMEM;
	}
	epld->als_input_dev->name = ElanALsensorName;
	set_bit(EV_ABS, epld->als_input_dev->evbit);
	input_set_abs_params(epld->als_input_dev, ABS_MISC, 0, 9, 0, 0);

	err = input_register_device(epld->als_input_dev);
	if (err < 0) {
		LOG_ERR("can not register ls input device\n");
		goto err_free_ls_input_device;
	}

	err = misc_register(&epl88051_als_device);
	if (err < 0) {
		LOG_ERR("can not register ls misc device\n");
		goto err_unregister_ls_input_device;
	}

	err = sysfs_create_group(&epld->als_input_dev->dev.kobj, &light_attribute_group);

	if (err) {
		pr_err("%s: could not create sysfs group\n", __func__);
		goto err_free_ls_input_device;
	}
	return err;


err_unregister_ls_input_device:
	input_unregister_device(epld->als_input_dev);
err_free_ls_input_device:
	input_free_device(epld->als_input_dev);
	return err;
}

static ssize_t proximity_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct epl88051_priv *epld  = epl88051_obj;
	printk("%s: PS status=%d\n", __func__, epld->enable_pflag);
	LOG_INFO("[PS] %s epl88051_setup_psensor enter.\n", __FUNCTION__);
	return sprintf(buf, "%d\n", epld->enable_pflag);
}

static ssize_t proximity_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct epl88051_priv *epld = epl88051_obj;

	LOG_INFO("[PS] %s proximity_enable_store: enable=%s \n", __FUNCTION__, buf);
	if (sysfs_streq(buf, "1"))
		epld->enable_pflag = 1;
	else if (sysfs_streq(buf, "0"))
		epld->enable_pflag = 0;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return 0;
	}

	epl88051_restart_work();
	return size;
}
static struct device_attribute dev_attr_ps_enable =
	__ATTR(enable, S_IRWXUGO, proximity_enable_show, proximity_enable_store);

static struct attribute *proximity_sysfs_attrs[] =
{
	&dev_attr_ps_enable.attr,
	NULL
};

static struct attribute_group proximity_attribute_group =
{
	.attrs = proximity_sysfs_attrs,
};

static int epl88051_setup_psensor(struct epl88051_priv *epld)
{
	int err = 0;
	LOG_INFO("[PS] %s epl88051_setup_psensor enter.\n", __FUNCTION__);

	epld->ps_input_dev = input_allocate_device();
	if (!epld->ps_input_dev) {
		LOG_ERR("could not allocate ps input device\n");
		return -ENOMEM;
	}
	epld->ps_input_dev->name = ElanPsensorName;
	set_bit(EV_ABS, epld->ps_input_dev->evbit);
	input_set_abs_params(epld->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	err = input_register_device(epld->ps_input_dev);
	if (err < 0) {
		LOG_ERR("could not register ps input device\n");
		goto err_free_ps_input_device;
	}

	err = misc_register(&epl88051_ps_device);
	if (err < 0) {
		LOG_ERR("could not register ps misc device\n");
		goto err_unregister_ps_input_device;
	}

	err = sysfs_create_group(&epld->ps_input_dev->dev.kobj, &proximity_attribute_group);

	if (err) {
		pr_err("%s: PS could not create sysfs group\n", __func__);
		goto err_free_ps_input_device;
	}

	return err;
err_unregister_ps_input_device:
	input_unregister_device(epld->ps_input_dev);
err_free_ps_input_device:
	input_free_device(epld->ps_input_dev);
	return err;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct epl88051_priv *epld = container_of(self, struct epl88051_priv, fb_notif);

	LOG_INFO("%s\n", __func__);
	if (evdata && evdata->data && event == FB_EVENT_BLANK && epld && epld->client) {
		blank = evdata->data;
		switch (*blank) {
			case FB_BLANK_UNBLANK:
				epld->als_suspend = 0;
				epld->ps_suspend = 0;
				epld->hs_suspend = 0;

				if(epld->enable_lflag || epld->enable_hflag) {
					epl88051_restart_work();
				}
				break;
			case FB_BLANK_POWERDOWN:
			case FB_BLANK_HSYNC_SUSPEND:
			case FB_BLANK_VSYNC_SUSPEND:
			case FB_BLANK_NORMAL:
				if (epld->enable_pflag == 0) {
					epld->als_suspend = 1;
					epld->hs_suspend = 1;
					epld->ps_suspend = 1;
				} else {
					LOG_INFO("%s: Psensor enable, so did not enter lpm\n", __func__);
					epld->als_suspend = 1;
					epld->hs_suspend = 1;
				}
				break;
		}
	}
	return 0;
}

#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void epl88051_early_suspend(struct early_suspend *h)
{
	struct epl88051_priv *epld = epl88051_obj;
	LOG_FUN();

	epld->als_suspend = 1;
	epld->hs_suspend = 1;
}

static void epl88051_late_resume(struct early_suspend *h)
{
	struct epl88051_priv *epld = epl88051_obj;
	LOG_FUN();

	epld->als_suspend = 0;
	epld->ps_suspend = 0;
	epld->hs_suspend = 0;

	if (epld->enable_lflag || epld->enable_hflag) {
		epl88051_restart_work();
	}
}
#endif

#ifdef CONFIG_SUSPEND
static int epl88051_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct epl88051_priv *epld = epl88051_obj;
	LOG_FUN();

	epld->als_suspend = 1;
	epld->ps_suspend = 1;
	if (epld->enable_pflag) {
		epl88051_psensor_enable(epld);
	} else {
		epl88051_I2C_Write(client, REG_8, W_SINGLE_BYTE, 0X02, EPL_C_P_DOWN);
	}
	return 0;
}

static int epl88051_resume(struct i2c_client *client)
{
	struct epl88051_priv *epld = epl88051_obj;
	LOG_FUN();

	epld->ps_suspend = 0;

	if (epld->enable_pflag) {
		epl88051_restart_work();
	}
	return 0;
}

#endif
static int epl88051_parse_dt(struct device *dev, struct epl88051_priv *pdata)
{
	struct property *prop;
	struct device_node *dt = dev->of_node;
	struct device_node *offset = NULL;
	int cali_size = 0;
	unsigned char *cali_data = NULL;
	int i = 0;

	LOG_INFO("%s: +\n", __func__);

	prop = of_find_property(dt, "epl88051,lsource_thd_high", NULL);
	if (prop) {
		of_property_read_u32(dt, "epl88051,lsource_thd_high", &pdata->lsource_thd_high);
	}
	prop = of_find_property(dt, "epl88051,lsource_thd_low", NULL);
	if (prop) {
		of_property_read_u32(dt, "epl88051,lsource_thd_high", &pdata->lsource_thd_low);
	}
	prop = of_find_property(dt, "epl88051,c_gain_h", NULL);
	if (prop) {
		of_property_read_u32(dt, "epl88051,c_gain_h", &pdata->c_gain_h);
	}
	prop = of_find_property(dt, "epl88051,c_gain_l", NULL);
	if (prop) {
		of_property_read_u32(dt, "epl88051,c_gain_l", &pdata->c_gain_l);
	}
	prop = of_find_property(dt, "epl88051,c_gain_saturated", NULL);
	if (prop) {
		of_property_read_u32(dt, "epl88051,c_gain_saturated", &pdata->c_gain_saturated);
	}

	prop = of_find_property(dt, "epl88051,golden_adc", NULL);
	if (prop) {
		of_property_read_u32(dt, "epl88051,golden_adc", &pdata->golden_adc);
	}

	prop = of_find_property(dt, "epl88051,ps_threshold_diff", NULL);
	if (prop) {
		of_property_read_u32(dt, "epl88051,ps_threshold_diff", &pdata->ps_threshold_diff);
	}

	prop = of_find_property(dt, "epl88051,lightsensor_table", NULL);
	if (prop) {
		of_property_read_u32_array(dt, "epl88051,lightsensor_table", adctable, 10);
		pdata->adc_table = &adctable[0];
	}

	pdata->emmc_als_kadc = 0;

	if ((offset = of_find_node_by_path(CALIBRATION_DATA_PATH))) {
		cali_data = (unsigned char*) of_get_property(offset, LIGHT_SENSOR_FLASH_DATA, &cali_size);

		LOG_INFO("%s: Light sensor cali_size = %d", __func__, cali_size);
		if (cali_data) {
			for (i = 0; (i < cali_size) && (i < 4); i++) {
				LOG_INFO("cali_data[%d] = %02x ", i, cali_data[i]);
				pdata->emmc_als_kadc |= (cali_data[i] << (i * 8));
			}
		}
	} else
		LOG_INFO("%s: Light sensor calibration data offset not found", __func__);

	pdata->emmc_ps_kadc1 = 0;
	pdata->emmc_ps_kadc2 = 0;

	if ((offset = of_find_node_by_path(CALIBRATION_DATA_PATH))) {
		cali_data = (unsigned char*) of_get_property(offset, PSENSOR_FLASH_DATA, &cali_size);

		LOG_INFO("%s: Psensor cali_size = %d", __func__, cali_size);
		if (cali_data) {
			for (i = 0; (i < cali_size) && (i < 4); i++) {
				LOG_INFO("cali_data[%d] = %02x ", i, cali_data[i]);
				pdata->emmc_ps_kadc1 |= (cali_data[i] << (i * 8));
			}
			for (i = 4; (i < cali_size) && (i < 8); i++) {
				LOG_INFO("cali_data[%d] = %02x ", i, cali_data[i]);
				pdata->emmc_ps_kadc2 |= (cali_data[i] << ((i-4) * 8));
			}
		}
	} else
		LOG_INFO("%s: Psensor calibration data offset not found", __func__);

	pdata->intr_pin = of_get_named_gpio_flags(dt, "epl88051,irq-gpio",
			0, &pdata->irq_gpio_flags);
	

	return 0;

}

static void epl88051_fb_register(struct work_struct *work)
{
	int ret = 0;
	struct  epl88051_priv *epld = container_of(work, struct epl88051_priv, work_fb.work);
	LOG_INFO("%s in", __func__);
	epld->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&epld->fb_notif);
	if (ret)
		LOG_ERR("[warning]:Unable to register fb_notifier: %d\n", ret);
}


static int epl88051_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	int err = 0;
	struct epl88051_priv *epld ;

	LOG_INFO("%s elan sensor probe enter.\n", __FUNCTION__);
	epld = kzalloc(sizeof(struct epl88051_priv), GFP_KERNEL);
	if (!epld)
		return -ENOMEM;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,"No supported i2c func what we need?!!\n");
		err = -ENOTSUPP;
		goto i2c_fail;
	}
	LOG_INFO("%s chip id REG 0x00 value = %8x\n", __FUNCTION__, i2c_smbus_read_byte_data(client, 0x00));
	LOG_INFO("%s chip id REG 0x01 value = %8x\n", __FUNCTION__, i2c_smbus_read_byte_data(client, 0x08));
	LOG_INFO("%s chip id REG 0x02 value = %8x\n", __FUNCTION__, i2c_smbus_read_byte_data(client, 0x10));
	LOG_INFO("%s chip id REG 0x03 value = %8x\n", __FUNCTION__, i2c_smbus_read_byte_data(client, 0x18));
	LOG_INFO("%s chip id REG 0x04 value = %8x\n", __FUNCTION__, i2c_smbus_read_byte_data(client, 0x20));
	LOG_INFO("%s chip id REG 0x05 value = %8x\n", __FUNCTION__, i2c_smbus_read_byte_data(client, 0x28));
	LOG_INFO("%s chip id REG 0x06 value = %8x\n", __FUNCTION__, i2c_smbus_read_byte_data(client, 0x30));
	LOG_INFO("%s chip id REG 0x07 value = %8x\n", __FUNCTION__, i2c_smbus_read_byte_data(client, 0x38));
	LOG_INFO("%s chip id REG 0x09 value = %8x\n", __FUNCTION__, i2c_smbus_read_byte_data(client, 0x48));
	LOG_INFO("%s chip id REG 0x0D value = %8x\n", __FUNCTION__, i2c_smbus_read_byte_data(client, 0x68));
	LOG_INFO("%s chip id REG 0x0E value = %8x\n", __FUNCTION__, i2c_smbus_read_byte_data(client, 0x70));
	LOG_INFO("%s chip id REG 0x0F value = %8x\n", __FUNCTION__, i2c_smbus_read_byte_data(client, 0x71));
	LOG_INFO("%s chip id REG 0x10 value = %8x\n", __FUNCTION__, i2c_smbus_read_byte_data(client, 0x80));
	LOG_INFO("%s chip id REG 0x11 value = %8x\n", __FUNCTION__, i2c_smbus_read_byte_data(client, 0x88));
	LOG_INFO("%s chip id REG 0x17 value = %8x\n", __FUNCTION__, i2c_smbus_read_byte_data(client, 0xB8));

	if ((i2c_smbus_read_byte_data(client, 0x00)) == 0xfffffffa) {
		LOG_INFO("%s elan ALS/PS sensor is failed. \n", __FUNCTION__);
		goto i2c_fail;
	}

	mutex_init(&sensor_mutex);
	err = epl88051_parse_dt(&client->dev, epld);
	lightsensor_set_kvalue(epld);
	epld->lux_per_count = LUX_PER_COUNT;
	epld->client = client;
	client->irq = gpio_to_irq(epld->intr_pin);
	epld->irq = client->irq;
	epld->mfg_mode = board_mfg_mode();
	i2c_set_clientdata(client, epld);

	epld->ps_delay = PS_DELAY;
	epld->ps_threshold_low = (uint8_t) ((epld->emmc_ps_kadc2 >> 16) & 0xFF) + (uint8_t) (epld->emmc_ps_kadc2 & 0xFF);
	epld->ps_threshold_high = epld->ps_threshold_low + epld->ps_threshold_diff;
	epld->polling_mode_ps = PS_POLLING_MODE;
    epld->ps_max_ch0 = MAX_PS_CH0;  

	epl88051_obj = epld;

	epld->epl_wq = create_singlethread_workqueue("elan_sensor_wq");
	if (!epld->epl_wq) {
		LOG_ERR("can't create workqueue\n");
		err = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}

	
	err = epl88051_setup_lsensor(epld);
	if (err < 0) {
		LOG_ERR("epl88051_setup_lsensor error!!\n");
		goto err_lightsensor_setup;
	}

	
	err = epl88051_setup_psensor(epld);
	if (err < 0) {
		LOG_ERR("epl88051_setup_psensor error!!\n");
		goto err_psensor_setup;
	}

	
	err = epl88051_initial_sensor(epld);
	if (err < 0) {
		LOG_ERR("fail to initial sensor (%d)\n", err);
		goto err_sensor_setup;
	}

	if (epld->polling_mode_ps == 0) {
		
		err = epl88051_setup_interrupt(epld);
		if (err < 0) {
			LOG_ERR("setup error!\n");
			goto err_sensor_setup;
		}
	}

#ifdef CONFIG_FB
	epld->epl88051_fb_wq = create_singlethread_workqueue("EPL88051_FB");
	if (!epld->epl88051_fb_wq) {
		LOG_ERR("allocate epl88051_fb_wq failed\n");
		err = -ENOMEM;
		goto err_create_epl88051_fb_workqueue_failed;
	}
	INIT_DELAYED_WORK(&epld->work_fb, epl88051_fb_register);
	queue_delayed_work(epld->epl88051_fb_wq, &epld->work_fb, msecs_to_jiffies(30000));

#elif defined(CONFIG_SUSPEND)
	epld->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	epld->early_suspend.suspend = epl88051_early_suspend;
	epld->early_suspend.resume = epl88051_late_resume;
	register_early_suspend(&epld->early_suspend);
#endif
	wake_lock_init(&g_ps_wlock, WAKE_LOCK_SUSPEND, "ps_wakelock");
	sensor_dev = platform_device_register_simple("elan_alsps", -1, NULL, 0);
	epld->epl88051_class = class_create(THIS_MODULE, "optical_sensors");
	if (IS_ERR(epld->epl88051_class)) {
		err = PTR_ERR(epld->epl88051_class);
		epld->epl88051_class = NULL;
		goto err_create_class;
	}
	epld->ls_dev = device_create(epld->epl88051_class,
			NULL, 0, "%s", "lightsensor");
	if (unlikely(IS_ERR(epld->ls_dev))) {
		err = PTR_ERR(epld->ls_dev);
		epld->ls_dev = NULL;
		goto err_create_ls_device;
	}

	err = device_create_file(epld->ls_dev, &dev_attr_als_int_time);
	if (err)
		goto err_create_ls_device_file;
	err = device_create_file(epld->ls_dev, &dev_attr_ls_adc);
	if (err)
		goto err_create_ls_device_file;

	err = device_create_file(epld->ls_dev, &dev_attr_ls_auto);
	if (err)
		goto err_create_ls_device_file;

	err = device_create_file(epld->ls_dev, &dev_attr_ls_kadc);
	if (err)
		goto err_create_ls_device_file;

	err = device_create_file(epld->ls_dev, &dev_attr_ls_adc_table);
	if (err)
		goto err_create_ls_device_file;

	epld->ps_dev = device_create(epld->epl88051_class,
			NULL, 0, "%s", "proximity");

	if (unlikely(IS_ERR(epld->ps_dev))) {
		err = PTR_ERR(epld->ps_dev);
		epld->ps_dev = NULL;
		goto err_create_ls_device_file;
	}

	err = device_create_file(epld->ps_dev, &dev_attr_ps_adc);
	if (err)
		goto err_create_ps_device;
	err = device_create_file(epld->ps_dev, &dev_attr_elan_status);
	if (err)
		goto err_create_ps_device;
	err = device_create_file(epld->ps_dev, &dev_attr_elan_reg);
	if (err)
		goto err_create_ps_device;
	err = device_create_file(epld->ps_dev, &dev_attr_ps_int_time);
	if (err)
		goto err_create_ps_device;
	err = device_create_file(epld->ps_dev, &dev_attr_ps_threshold);
	if (err)
		goto err_create_ps_device;
	err = device_create_file(epld->ps_dev, &dev_attr_ps_cal_raw);
	if (err)
		goto err_create_ps_device;
	err = device_create_file(epld->ps_dev, &dev_attr_ps_polling_mode);
	if (err)
		goto err_create_ps_device;
#if 1 
    err = device_create_file(epld->ps_dev, &dev_attr_dyn_max_ps_ch0);
	if (err)
		goto err_create_ps_device;
#endif

	err = device_create_file(epld->ps_dev, &dev_attr_hs_enable);
	if (err)
		goto err_create_ps_device;
	err = device_create_file(epld->ps_dev, &dev_attr_hs_raws);
	if (err)
		goto err_create_ps_device;
	err = device_create_file(epld->ps_dev, &dev_attr_ps_canc);
	if (err)
		goto err_create_ps_device;
	err = device_create_file(epld->ps_dev, &dev_attr_ps_kadc);
	if (err)
		goto err_create_ps_device;
	err = device_create_file(epld->ps_dev, &dev_attr_p_status);
	if (err)
		goto err_create_ps_device;
#if 0
	err = sysfs_create_group(&sensor_dev->dev.kobj, &epl88051_attr_group);
	if (err !=0)
	{
		dev_err(&client->dev,"%s:create sysfs group error", __func__);
		goto err_fail;
	}
#endif
	LOG_INFO(" %s sensor probe success.\n", __FUNCTION__);

	return err;
err_create_ps_device:
	device_unregister(epld->ps_dev);
err_create_ls_device_file:
	device_unregister(epld->ls_dev);
err_create_ls_device:
	class_destroy(epld->epl88051_class);
err_create_class:
	input_unregister_device(epld->als_input_dev);
	input_unregister_device(epld->ps_input_dev);
	input_free_device(epld->als_input_dev);
	input_free_device(epld->ps_input_dev);
err_create_epl88051_fb_workqueue_failed:
err_lightsensor_setup:
err_psensor_setup:
err_sensor_setup:
	destroy_workqueue(epld->epl_wq);
	misc_deregister(&epl88051_ps_device);
	misc_deregister(&epl88051_als_device);
err_create_singlethread_workqueue:
i2c_fail:
	
	kfree(epld);
	return err;
}

static int epl88051_remove(struct i2c_client *client)
{
	struct epl88051_priv *epld = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s: enter.\n", __func__);

	
	platform_device_unregister(sensor_dev);
	input_unregister_device(epld->als_input_dev);
	input_unregister_device(epld->ps_input_dev);
	input_free_device(epld->als_input_dev);
	input_free_device(epld->ps_input_dev);
	misc_deregister(&epl88051_ps_device);
	misc_deregister(&epl88051_als_device);
	free_irq(epld->irq,epld);
	destroy_workqueue(epld->epl_wq);
	kfree(epld);
	return 0;
}

static const struct i2c_device_id epl88051_id[] =
{
	{ "EPL88051", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, epl88051_i2c_id);
static struct of_device_id epl88051_match_table[] = {
	{.compatible = "EPL88051"},
	{},
};
static struct i2c_driver epl88051_driver =
{
	.probe	= epl88051_probe,
	.remove	= epl88051_remove,
	.id_table	= epl88051_id,
	.driver	= {
		.name = "EPL88051",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = epl88051_match_table,
#endif
	},
#ifdef CONFIG_SUSPEND
	.suspend = epl88051_suspend,
	.resume = epl88051_resume,
#endif
};

module_i2c_driver(epl88051_driver);

#ifndef CONFIG_OF
static int __init epl88051_init(void)
{
	return i2c_add_driver(&epl88051_driver);
}

static void __exit  epl88051_exit(void)
{
	i2c_del_driver(&epl88051_driver);
}

module_init(epl88051_init);
module_exit(epl88051_exit);
#endif
MODULE_AUTHOR("Renato Pan <renato.pan@eminent-tek.com>");
MODULE_DESCRIPTION("ELAN epl88051 driver");
MODULE_LICENSE("GPL");

