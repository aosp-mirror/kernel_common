#ifndef HIMAX8528_H
#define HIMAX8528_H
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>

#define HIMAX8528_NAME "Himax8528"
#define HIMAX8528_FINGER_SUPPORT_NUM 10

#define HX_TP_SYS_DIAG											
#define HX_TP_SYS_REGISTER									
#define HX_TP_SYS_DEBUG										
#define HX_TP_SYS_FLASH_DUMP								
#define HX_TP_SYS_SELF_TEST									
#define HX_TP_SYS_HITOUCH										
#define HX_RST_PIN_FUNC											
#define HX_LOADIN_CONFIG									

#ifdef HX_RST_PIN_FUNC
	
	#if defined(CONFIG_TOUCHSCREEN_HIMAX_ESD_EN)
	#define HX_ESD_WORKAROUND								
	#endif
	#define ENABLE_CHIP_RESET_MACHINE					
#endif	

#ifdef ENABLE_CHIP_RESET_MACHINE 
	#define HX_TP_SYS_RESET										
	
#endif

#define HX_85XX_A_SERIES_PWON		1
#define HX_85XX_B_SERIES_PWON		2
#define HX_85XX_C_SERIES_PWON		3
#define HX_85XX_D_SERIES_PWON		4


#define HX_TP_BIN_CHECKSUM_SW		1
#define HX_TP_BIN_CHECKSUM_HW		2
#define HX_TP_BIN_CHECKSUM_CRC	3
	
#define HX_KEY_MAX_COUNT             4			
#define DEFAULT_RETRY_CNT            3			

#define HIMAX_PWR_GPIO				59
#define HIMAX_INT_GPIO				60
#define HIMAX_RST_GPIO				62

#define HIMAX_I2C_ADDR				0x48
#define HIMAX_TS_NAME			"himax-ts"

#define INPUT_DEV_NAME	"himax-touchscreen"	

#define FLASH_DUMP_FILE "/sdcard/Flash_Dump.bin"

#define DIAG_COORDINATE_FILE "/sdcard/Coordinate_Dump.csv"

#define HX_VKEY_0   KEY_BACK
#define HX_VKEY_1   KEY_HOME
#define HX_VKEY_2   KEY_RESERVED
#define HX_VKEY_3   KEY_RESERVED
#define HX_KEY_ARRAY    {HX_VKEY_0, HX_VKEY_1, HX_VKEY_2, HX_VKEY_3}


struct himax_config_init_api {
	int (*i2c_himax_master_write)(struct i2c_client *client, uint8_t *data, uint8_t length, uint8_t retry);
	int (*i2c_himax_write_command)(struct i2c_client *client, uint8_t command, uint8_t retry);
	int (*i2c_himax_read_command)(struct i2c_client *client, uint8_t length, uint8_t *data, uint8_t *readlength, uint8_t retry);
};

struct himax_virtual_key {
	int index;
	int keycode;
	int x_range_min;
	int x_range_max;
	int y_range_min;
	int y_range_max;
};

struct himax_config {
	uint8_t default_cfg;
	uint32_t sensor_id;
	uint32_t fw_ver;
	uint16_t length;
	uint32_t tw_x_min;
	uint32_t tw_x_max;
	uint32_t tw_y_min;
	uint32_t tw_y_max;
	uint32_t pl_x_min;
	uint32_t pl_x_max;
	uint32_t pl_y_min;
	uint32_t pl_y_max;
	uint8_t c1[5];
	uint8_t c2[2];
	uint8_t c3[11];
	uint8_t c4[11];
	uint8_t c5[11];
	uint8_t c6[11];
	uint8_t c7[11];
	uint8_t c8[11];
	uint8_t c9[11];
	uint8_t c10[11];
	uint8_t c11[11];
	uint8_t c12[11];
	uint8_t c13[11];
	uint8_t c14[11];
	uint8_t c15[11];
	uint8_t c16[11];
	uint8_t c17[11];
	uint8_t c18[2];
	uint8_t c19[4];
	uint8_t c20[9];
	uint8_t c21[5];
	uint8_t c22[16];
	uint8_t c23[3];
	uint8_t c24[2];
	uint8_t c25[2];
	uint8_t c26[5];
	uint8_t c27[3];
	uint8_t c28[9];
	uint8_t c29[11];
	uint8_t c30[4];
	uint8_t c31[65];
	uint8_t c32[13];
	uint8_t c33[3];
	uint8_t c34[3];
	uint8_t c35[17];
	uint8_t c36[31];
	uint8_t c37[31];
	uint8_t c38[17];
	uint8_t c39[25];
	uint8_t c40[23];
	uint8_t c41[29];
	uint8_t c42[9];
	uint8_t c43_1[32];
	uint8_t c43_2[30];
	uint8_t c44_1[32];
	uint8_t c44_2[6];
	uint8_t c45[3];
};

struct himax_i2c_platform_data_config_type28
{
	uint8_t version;
	uint8_t tw_id;
	uint8_t common;
	uint8_t x_fuzz;
	uint8_t y_fuzz;
	uint8_t z_fuzz;

	uint8_t c1[5];
	uint8_t c2[2];
	uint8_t c3[11];
	uint8_t c4[11];
	uint8_t c5[11];
	uint8_t c6[11];
	uint8_t c7[11];
	uint8_t c8[11];
	uint8_t c9[11];
	uint8_t c10[11];
	uint8_t c11[11];
	uint8_t c12[11];
	uint8_t c13[11];
	uint8_t c14[11];
	uint8_t c15[11];
	uint8_t c16[11];
	uint8_t c17[11];
	uint8_t c18[2];
	uint8_t c19[4];
	uint8_t c20[9];
	uint8_t c21[5];
	uint8_t c22[16];
	uint8_t c23[3];
	uint8_t c24[2];
	uint8_t c25[2];
	uint8_t c26[5];
	uint8_t c27[3];
	uint8_t c28[9];
	uint8_t c29[11];
	uint8_t c30[4];
	uint8_t c31[65];
	uint8_t c32[13];
	uint8_t c33[3];
	uint8_t c34[3];
	uint8_t c35[17];
	uint8_t c36[31];
	uint8_t c37[31];
	uint8_t c38[17];
	uint8_t c39[25];
	uint8_t c40[23];
	uint8_t c41[29];
	uint8_t c42[9];
	uint8_t c43_1[32];
	uint8_t c43_2[30];
	uint8_t c44_1[32];
	uint8_t c44_2[6];
	uint8_t c45[3];
};

struct himax_i2c_platform_data {
	
	int abs_x_min;
	int abs_x_max;
	int abs_x_fuzz;
	int abs_y_min;
	int abs_y_max;
	int abs_y_fuzz;
	int abs_pressure_min;
	int abs_pressure_max;
	int abs_pressure_fuzz;
	int abs_width_min;
	int abs_width_max;
	uint8_t powerOff3V3;
	int (*power)(int on);
	struct himax_virtual_key *virtual_key;
	int gpio_irq;
	int gpio_reset;
	struct kobject *vk_obj;
	struct kobj_attribute *vk2Use;
	uint8_t slave_addr;
	uint32_t event_htc_enable;
	uint8_t cable_config[2];

	
	uint8_t support_htc_event;
	

	
	uint8_t protocol_type;

	
	int screenWidth;
	int screenHeight;
	

	
	char ID0[20];
	char ID1[20];
	char ID2[20];
	char ID3[20];
	

	void (*reset)(void);
	int (*loadSensorConfig)(struct i2c_client *client, struct himax_i2c_platform_data *pdata, struct himax_config_init_api *i2c_api);
	
	uint8_t version;
	uint8_t fw_version;
	uint8_t tw_id;
	
	uint8_t *regCD;
	
	struct himax_i2c_platform_data_config_type28 *type28;
	int type28_size;
	
	
	int (*init)(struct device *dev, struct himax_i2c_platform_data *pdata);
	int (*enable)(struct device *dev, struct himax_i2c_platform_data *pdata);
	int (*disable)(struct device *dev, struct himax_i2c_platform_data *pdata);
	void (*exit)(struct device *dev, struct himax_i2c_platform_data *pdata);
	void *extra;
	
	int power_keep_on;
};

#define HX_CMD_NOP					 0x00	
#define HX_CMD_SETMICROOFF			 0x35	
#define HX_CMD_SETROMRDY			 0x36	
#define HX_CMD_TSSLPIN				 0x80	
#define HX_CMD_TSSLPOUT 			 0x81	
#define HX_CMD_TSSOFF				 0x82	
#define HX_CMD_TSSON				 0x83	
#define HX_CMD_ROE					 0x85	
#define HX_CMD_RAE					 0x86	
#define HX_CMD_RLE					 0x87	
#define HX_CMD_CLRES				 0x88	
#define HX_CMD_TSSWRESET			 0x9E	
#define HX_CMD_SETDEEPSTB			 0xD7	
#define HX_CMD_SET_CACHE_FUN		 0xDD	
#define HX_CMD_SETIDLE				 0xF2	
#define HX_CMD_SETIDLEDELAY 		 0xF3	
#define HX_CMD_SELFTEST_BUFFER		 0x8D	
#define HX_CMD_MANUALMODE			 0x42
#define HX_CMD_FLASH_ENABLE 		 0x43
#define HX_CMD_FLASH_SET_ADDRESS	 0x44
#define HX_CMD_FLASH_WRITE_REGISTER  0x45
#define HX_CMD_FLASH_SET_COMMAND	 0x47
#define HX_CMD_FLASH_WRITE_BUFFER	 0x48
#define HX_CMD_FLASH_PAGE_ERASE 	 0x4D
#define HX_CMD_FLASH_SECTOR_ERASE	 0x4E
#define HX_CMD_CB					 0xCB
#define HX_CMD_EA					 0xEA
#define HX_CMD_4A					 0x4A
#define HX_CMD_4F					 0x4F
#define HX_CMD_B9					 0xB9
#define HX_CMD_76					 0x76

enum input_protocol_type {
	PROTOCOL_TYPE_A	= 0x00,
	PROTOCOL_TYPE_B	= 0x01,
};
extern int proximity_enable_from_ps(int on);
#endif
