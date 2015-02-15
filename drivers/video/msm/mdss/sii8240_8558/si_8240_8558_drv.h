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

#if !defined(SI_8240_8558_DRV_H)
#define SI_8240_8558_DRV_H

#define LOW		0
#define	HIGH	1


struct drv_hw_context {
	struct interrupt_info *intr_info;
	uint8_t 	chip_rev_id;
	uint16_t	chip_device_id;
	uint8_t		cbus_status;
	uint8_t		gen2_write_burst;
	uint8_t		ready_for_mdt;
	uint8_t		video_path;
	uint8_t		video_ready;
	uint8_t		ckdt_done;
	uint8_t		audio_poll_enabled;
	uint8_t		current_edid_request_block;
	uint8_t		edid_fifo_block_number;
	uint8_t		saved_reg_mhltx_ctl2;
	uint8_t		valid_vsif;
	uint8_t		valid_avif;
	uint8_t		rx_hdmi_ctrl2_defval;
	bool		suspend_strapped_mode;
	uint8_t		boot_usb_impedance_reference;
	uint8_t		boot_uart_impedance_reference;
	uint8_t		strapping_value;

	struct workqueue_struct 	*demo_loop_wq;
	struct delayed_work 		demo_loop_work;

	enum{
	  ACCESSORY_MCPC_SETUP
	, ACCESSORY_MCPC_ACTIVE
	, ACCESSORY_BOARD
	, ACCESSORY_OTG_HOST
	, ACCESSORY_BOOT_DEVICE /* this MUST come after ACCESSORY_OTG_HOST */
	, ACCESSORY_USB_DEVICE
	, ACCESSORY_DISCONNECTED
	, ACCESSORY_MHL
	}accessory_type;
	unsigned char 	mcpc_button_on;

	avi_info_frame_t				current_avi_info_frame;
	vendor_specific_info_frame_t	current_vs_info_frame;
	hw_avi_payload_t        		outgoingAviPayLoad;
	uint8_t		write_burst_data[MHL_SCRATCHPAD_SIZE];
};

bool si_mhl_tx_set_status(struct mhl_dev_context *dev_context,
								 uint8_t reg_to_write, uint8_t value);
int get_device_id(void);

#ifdef DDC_BYPASS_API //(
int __si_8240_8558_drv_ddc_bypass_control(char *file,int iLine,struct drv_hw_context *hw_context,int bypass);
#define si_8240_8558_drv_ddc_bypass_control(hw_context,bypass) __si_8240_8558_drv_ddc_bypass_control(__FILE__,__LINE__,hw_context,bypass)
#endif //)

#endif /* if !defined(SI_8240_8558_DRV_H) */
