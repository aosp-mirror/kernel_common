/*

SiI8558 / SiI8240 Linux Driver

Copyright (C) 2013 Silicon Image, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation version 2.
This program is distributed AS-IS WITHOUT ANY WARRANTY of any
kind, whether express or implied; INCLUDING without the implied warranty
of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.  See
the GNU General Public License for more details at http://www.gnu.org/licenses/gpl-2.0.html.

*/

#if !defined(PLATFORM_H)
#define PLATFORM_H

#define DEVICE_ID_8558 0x8558
#define DEVICE_ID_8240 0x8240

#define DEBUG_I2C_WRITE				1
#define DEBUG_I2C_READ				0
#define MAX_DEBUG_TRANSFER_SIZE		32
#define COMPATIBLE_NAME				"mhl-sii8240-sii8558"

#define MAX_MHL_PCLK 150000

#define HTC

enum dbg_msg_level {
	DBG_MSG_LEVEL_ERR = 0
	,DBG_MSG_LEVEL_WARN
	,DBG_MSG_LEVEL_INFO
	,DBG_MSG_LEVEL_GPIO
	,DBG_MSG_LEVEL_EDID_INFO
};


#if defined(DEBUG)

void print_formatted_debug_msg(int level,
							   char *file_spec, const char *func_name,
							   int line_num,
							   char *fmt, ...);

void dump_i2c_transfer(void *context, u8 page, u8 offset,
						u16 count, u8 *values, bool write);

#define MHL_TX_DBG_GPIO(driver_context, fmt, arg...)			\
	print_formatted_debug_msg(DBG_MSG_LEVEL_GPIO,				\
							  NULL, __func__, __LINE__,		\
							  fmt, ## arg);

#define MHL_TX_EDID_INFO(context,fmt,arg...) \
	print_formatted_debug_msg(DBG_MSG_LEVEL_EDID_INFO,\
							  NULL, __func__, __LINE__,		\
							  fmt, ## arg);

#define MHL_TX_DBG_INFO(driver_context, fmt, arg...)			\
	print_formatted_debug_msg(DBG_MSG_LEVEL_INFO,				\
							  NULL, __func__, __LINE__,		\
							  fmt, ## arg);

#define MHL_TX_DBG_WARN(driver_context, fmt, arg...) 			\
	print_formatted_debug_msg(DBG_MSG_LEVEL_WARN,				\
							  NULL, __func__, __LINE__,		\
							  fmt, ## arg);

#define MHL_TX_DBG_ERR(driver_context, fmt, arg...)				\
	print_formatted_debug_msg(DBG_MSG_LEVEL_ERR,				\
							  NULL, __func__, __LINE__,		\
							  fmt, ## arg);

#define DUMP_I2C_TRANSFER(context, page, offset, count, values, write_flag)	\
		dump_i2c_transfer(context, page, offset, count, values, write_flag);

#define DEBUG_PRINT_WRAPPER(...)						\
		print_formatted_debug_msg(DBG_MSG_LEVEL_INFO,	\
								  __FILE__, __func__,	\
								  __LINE__, __VA_ARGS__);

#define APP_DEBUG_PRINT(x) 			DEBUG_PRINT_WRAPPER x
#define PP_DEBUG_PRINT(x)			DEBUG_PRINT_WRAPPER x
#define PIXCLK_DEBUG_PRINT(x)		DEBUG_PRINT_WRAPPER x
#define ERROR_DEBUG_PRINT(x)		DEBUG_PRINT_WRAPPER x
#define TX_EDID_PRINT(x)			DEBUG_PRINT_WRAPPER x
#define TX_DEBUG_PRINT(x)			DEBUG_PRINT_WRAPPER x
#define TX_PRUNE_PRINT(x)			DEBUG_PRINT_WRAPPER x
#define EDID_DEBUG_PRINT(x)			DEBUG_PRINT_WRAPPER x
#define TX_EDID_PRINT_SIMPLE(x)		DEBUG_PRINT_WRAPPER x

#else
#define MHL_TX_DBG_GPIO(driver_context, fmt, arg...)
#define MHL_TX_EDID_INFO(context,fmt,arg...)
#define MHL_TX_DBG_INFO(driver_context, fmt, arg...)
#define MHL_TX_DBG_WARN(driver_context, fmt, ...)
#define MHL_TX_DBG_ERR(driver_context, fmt, ...)

#define DUMP_I2C_TRANSFER(context, page, offset, count, values, write_flag)

#define APP_DEBUG_PRINT(x)
#define PP_DEBUG_PRINT(x)
#define PIXCLK_DEBUG_PRINT(x)
#define ERROR_DEBUG_PRINT(x)
#define TX_EDID_PRINT(x)
#define TX_DEBUG_PRINT(x)
#define TX_PRUNE_PRINT(x)
#define EDID_DEBUG_PRINT(x)
#define TX_EDID_PRINT_SIMPLE(x)

#endif

enum vbus_power_state {
	VBUS_OFF,
	VBUS_ON
};
typedef enum {
	HPD_CTRL_MODE_ERROR=-1
	,HPD_CTRL_OPEN_DRAIN
	,HPD_CTRL_PUSH_PULL
}hpd_control_mode;

hpd_control_mode platform_get_hpd_control_mode(void);
void mhl_tx_vbus_control(enum vbus_power_state power_state);
#ifdef CONFIG_INTERNAL_CHARGING_SUPPORT
void htc_charging_enable(int dcap);
#endif

struct platform_reg_pair{
	uint8_t	slave_addr;
	uint8_t offset;
};
struct platform_signals_list {
	char		*name;
	int16_t		gpio_number;
	bool		*param;
};

int gpio_extender_init(struct mhl_dev_context *dev_context);

bool is_reset_on_exit_requested(void);

int mhl_tx_write_reg(void *drv_context, u8 page, u8 offset, u8 value);

int mhl_tx_read_reg(void *drv_context, u8 page, u8 offset);

int mhl_tx_write_reg_block(void *drv_context, u8 page, u8 offset, u16 count, u8 *values);

int mhl_tx_read_reg_block(void *drv_context, u8 page, u8 offset, u8 count, u8 *values);

int mhl_tx_modify_reg(void *drv_context, u8 page, u8 offset,
					  u8 mask, u8 value);

int is_interrupt_asserted(void);
int get_config(void *dev_context, int config_idx);
#define	GPIO_LED_ON		0
#define	GPIO_LED_OFF	1
void set_pin_impl(void *dev_context, int pin_idx, int value,const char *function_name,int line_num);
#define set_pin(dev_context,pin_idx,value) set_pin_impl(dev_context, pin_idx, value,__FUNCTION__,__LINE__)
void enable_hdmi(int enable);

extern	bool	allow_d3;
extern	bool	source_vbus_on;
extern	bool	transcode_mode;
extern int override_strap;

typedef enum {

	TX_HW_RESET	= 0
	,M2U_VBUS_CTRL_M
	,MCPC_MODE
	,ALLOW_D3
	,TX2MHLRX_PWR_M
	,TRANSCODE_MODE
	,TX_FW_WAKE
	,MHL_LED
	,USB_LED
	,USB_ID_L_LED
	,UART_LED
	,AUDIO_LED
	,MIC_LED
	,SRC_VBUS_ON_LED
	,SINK_VBUS_ON_LED
	,SDP_LED
	,CDP_LED
	,DCP_LED
	,PS2_LED
	,D0_LED
	,D2_LED
	,D3_HOT_LED
	,D3_COLD_LED
	,_3D_LED
	,PACKED_PIXEL_LED
	,HDCP_ON_LED
	,TRANSCODE_MODE_LED
	,SPARE_LED1
	,SPARE_LED2
	,GNT_
	,COMM_MODE
	,TWELVE_VOLT_PS_SENSE
}GPIO_INDICES;

#endif /* if !defined(PLATFORM_H) */
