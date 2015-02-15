/*
 * Copyright (C) 2010 HTC, Inc.
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
#ifndef __ASM_ARCH_MSM_HTC_USB_H
#define __ASM_ARCH_MSM_HTC_USB_H

#ifdef CONFIG_ARCH_QSD8X50
void msm_hsusb_8x50_phy_reset(void);
#endif

#ifdef ERROR
#undef ERROR
#endif

#include <linux/usb/f_accessory.h>

#define HTC_PRODUCT_STRING "Android Phone"
#define HTC_COMPANY_STRING "HTC"

struct android_usb_product {
  __u16 vendor_id;

  
  __u16 product_id;

  int num_functions;
  char **functions;
};


#ifdef CONFIG_USB_ANDROID_USBNET
static char *usb_functions_usbnet[] = {
	"usbnet",
};

static char *usb_functions_usbnet_adb[] = {
	"usbnet",
	"adb",
};
#endif

static char *usb_functions_ums[] = {
	"mass_storage",
};

static char *usb_functions_ptp[] = {
	"ptp",
};

static char *usb_functions_ptp_adb[] = {
	"ptp",
	"adb",
};

static char *usb_functions_adb[] = {
	"mass_storage",
	"adb",
};

static char *usb_functions_mtp_ums_adb[] = {
	"mtp",
	"adb",
	"mass_storage",
};

static char *usb_functions_mtp_ums[] = {
	"mtp",
	"mass_storage",
};

static char *usb_functions_ecm[] = {
	"cdc_ethernet",
};
static char *usb_functions_rndis[] = {
	"rndis",
};
static char *usb_functions_rndis_modem[] = {
	"rndis",
	"modem",
};
static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};
static char *usb_functions_rndis_adb_modem[] = {
	"rndis",
	"adb",
	"modem",
};
static char *usb_functions_rndis_mtp_ums[] = {
	"rndis",
	"mtp",
	"mass_storage",
};
static char *usb_functions_rndis_mtp_ums_modem[] = {
	"rndis",
	"mtp",
	"mass_storage",
	"modem",
};
static char *usb_functions_rndis_mtp_ums_diag[] = {
	"rndis",
	"mtp",
	"mass_storage",
	"diag",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_rndis_mtp_ums_diag_modem[] = {
	"rndis",
	"mtp",
	"mass_storage",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_rndis_mtp_ums_adb[] = {
	"rndis",
	"mtp",
	"adb",
	"mass_storage",
};
static char *usb_functions_rndis_mtp_ums_adb_modem[] = {
	"rndis",
	"mtp",
	"adb",
	"mass_storage",
	"modem",
};
static char *usb_functions_rndis_mtp_ums_adb_diag[] = {
	"rndis",
	"mtp",
	"adb",
	"mass_storage",
	"diag",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_rndis_mtp_ums_adb_diag_modem[] = {
	"rndis",
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_rndis_diag[] = {
	"rndis",
	"diag",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_rndis_diag_modem[] = {
	"rndis",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_rndis_adb_diag[] = {
	"rndis",
	"adb",
	"diag",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_rndis_adb_diag_modem[] = {
	"rndis",
	"adb",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_accessory[] = { "accessory" };
static char *usb_functions_accessory_adb[] = { "accessory", "adb" };

static char *usb_functions_audio[] = { "audio_source"};
static char *usb_functions_audio_adb[] = { "audio_source", "adb" };
static char *usb_functions_accessory_audio[] = { "accessory", "audio_source"};
static char *usb_functions_accessory_audio_adb[] = { "accessory", "audio_source", "adb" };

static char *usb_functions_projector[] = {
	"mass_storage",
	"projector",
};
static char *usb_functions_adb_projector[] = {
	"mass_storage",
	"adb",
	"projector",
};
static char *usb_functions_mtp_ums_projector[] = {
	"mtp",
	"mass_storage",
	"projector",
};
static char *usb_functions_mtp_ums_adb_projector[] = {
	"mtp",
	"adb",
	"mass_storage",
	"projector",
};
static char *usb_function_adb_diag_projector[] = {
	"mass_storage",
	"adb",
	"diag",
	"projector",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_function_mtp_ums_adb_diag_projector[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"projector",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_function_adb_diag_modem_projector[] = {
	"mass_storage",
	"adb",
	"diag",
	"modem",
	"projector",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_function_mtp_ums_adb_diag_modem_projector[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"modem",
	"projector",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};

static char *usb_functions_mtp[] = {
	"mtp",
};

static char *usb_functions_mtp_adb[] = {
	"mtp",
	"adb",
};

#if defined(CONFIG_USB_ANDROID_MTP36) && defined(CONFIG_USB_ANDROID_MTP)
static char *usb_functions_mtp36[] = {
	"mtp36",
};

static char *usb_functions_mtp36_adb[] = {
	"mtp36",
	"adb",
};
#endif

static char *usb_functions_diag[] = {
	"mass_storage",
	"diag",
};
static char *usb_functions_diag2[] = {
	"mass_storage",
	"diag",
	"diag_mdm",
};
static char *usb_functions_adb_diag[] = {
	"mass_storage",
	"adb",
	"diag",
};
static char *usb_functions_adb_diag2[] = {
	"mass_storage",
	"adb",
	"diag",
	"diag_mdm",
};
static char *usb_functions_mtp_ums_adb_diag[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
};
static char *usb_functions_mtp_ums_diag[] = {
	"mtp",
	"mass_storage",
	"diag",
};
static char *usb_functions_mtp_ums_adb_diag2[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"diag_mdm",
};
static char *usb_functions_mtp_ums_diag2[] = {
	"mtp",
	"mass_storage",
	"diag",
	"diag_mdm",
};

static char *usb_functions_adb_serial[] = {
	"mass_storage",
	"adb",
	"serial",
};

static char *usb_functions_modem[] = {
	"mass_storage",
	"modem",
};
static char *usb_functions_adb_modem[] = {
	"mass_storage",
	"adb",
	"modem",
};
static char *usb_functions_adb_serial_modem[] = {
	"mass_storage",
	"adb",
	"modem",
	"serial",
};
static char *usb_functions_serial_modem[] = {
	"mass_storage",
	"modem",
	"serial",
};
static char *usb_functions_mtp_ums_adb_modem[] = {
	"mtp",
	"adb",
	"mass_storage",
	"modem",
};
static char *usb_functions_mtp_ums_modem[] = {
	"mtp",
	"mass_storage",
	"modem",
};
static char *usb_functions_mtp_ums_serial_modem[] = {
	"mtp",
	"mass_storage",
	"modem",
	"serial",
};
static char *usb_functions_mtp_ums_adb_serial_modem[] = {
	"mtp",
	"adb",
	"mass_storage",
	"modem",
	"serial",
};
static char *usb_functions_mtp_ums_adb_serial[] = {
	"mtp",
	"adb",
	"mass_storage",
	"serial",
};
static char *usb_functions_diag_modem[] = {
	"mass_storage",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_adb_diag_modem[] = {
	"mass_storage",
	"adb",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_adb_diag_serial[] = {
	"mass_storage",
	"adb",
	"diag",
	"serial",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_diag_serial[] = {
	"mass_storage",
	"diag",
	"serial",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_adb_diag_serial_modem[] = {
	"mass_storage",
	"adb",
	"diag",
	"modem",
	"serial",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_diag_serial_modem[] = {
	"mass_storage",
	"diag",
	"modem",
	"serial",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_ums_adb_diag_modem[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_ums_diag_serial[] = {
	"mtp",
	"mass_storage",
	"diag",
	"serial",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_ums_adb_diag_serial[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"serial",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_ums_adb_diag_serial_modem[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"modem",
	"serial",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_ums_diag_serial_modem[] = {
	"mtp",
	"mass_storage",
	"diag",
	"modem",
	"serial",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_ums_diag_modem[] = {
	"mtp",
	"mass_storage",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};

static char *usb_functions_adb_acm[] = {
	"adb",
	"mass_storage",
	"acm",
};
static char *usb_functions_ums_acm[] = {
	"mass_storage",
	"acm",
};
static char *usb_functions_acm[] = {
	"acm",
};
static char *usb_functions_mtp_ums_acm[] = {
	"mtp",
	"mass_storage",
	"acm",
};
static char *usb_functions_mtp_adb_ums_acm[] = {
	"mtp",
	"adb",
	"mass_storage",
	"acm",
};
static char *usb_functions_rndis_ums_acm[] = {
	"rndis",
	"mass_storage",
	"acm",
};
static char *usb_functions_rndis_adb_ums_acm[] = {
	"rndis",
	"adb",
	"mass_storage",
	"acm",
};
static char *usb_functions_rndis_mtp_ums_acm[] = {
	"rndis",
	"mtp",
	"mass_storage",
	"acm",
};
static char *usb_functions_rndis_mtp_adb_ums_acm[] = {
	"rndis",
	"mtp",
	"adb",
	"mass_storage",
	"acm",
};
static char *usb_functions_rndis_mtp_adb_ums_diag_acm[] = {
	"rndis",
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_rndis_mtp_adb_ums_modem_acm[] = {
	"rndis",
	"mtp",
	"adb",
	"mass_storage",
	"modem",
	"acm",
};
static char *usb_functions_rndis_mtp_adb_ums_diag_modem_acm[] = {
	"rndis",
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"modem",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_rndis_mtp_ums_diag_acm[] = {
	"rndis",
	"mtp",
	"mass_storage",
	"diag",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_rndis_mtp_ums_modem_acm[] = {
	"rndis",
	"mtp",
	"mass_storage",
	"modem",
	"acm",
};
static char *usb_functions_rndis_mtp_ums_diag_modem_acm[] = {
	"rndis",
	"mtp",
	"mass_storage",
	"diag",
	"modem",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_adb_ums_diag_acm[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_adb_ums_modem_acm[] = {
	"mtp",
	"adb",
	"mass_storage",
	"modem",
	"acm",
};
static char *usb_functions_mtp_adb_ums_diag_modem_acm[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"modem",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_ums_diag_acm[] = {
	"mtp",
	"mass_storage",
	"diag",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_ums_modem_acm[] = {
	"mtp",
	"mass_storage",
	"modem",
	"acm",
};
static char *usb_functions_mtp_ums_diag_modem_acm[] = {
	"mtp",
	"mass_storage",
	"diag",
	"modem",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_rndis_adb_ums_diag_acm[] = {
	"rndis",
	"adb",
	"mass_storage",
	"diag",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_rndis_adb_ums_modem_acm[] = {
	"rndis",
	"adb",
	"mass_storage",
	"modem",
	"acm",
};
static char *usb_functions_rndis_adb_ums_diag_modem_acm[] = {
	"rndis",
	"adb",
	"mass_storage",
	"diag",
	"modem",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_rndis_ums_diag_acm[] = {
	"rndis",
	"mass_storage",
	"diag",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_rndis_ums_modem_acm[] = {
	"rndis",
	"mass_storage",
	"modem",
	"acm",
};
static char *usb_functions_rndis_ums_diag_modem_acm[] = {
	"rndis",
	"mass_storage",
	"diag",
	"modem",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_adb_ums_diag_acm[] = {
	"adb",
	"mass_storage",
	"diag",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_adb_ums_modem_acm[] = {
	"adb",
	"mass_storage",
	"modem",
	"acm",
};
static char *usb_functions_adb_ums_diag_modem_acm[] = {
	"adb",
	"mass_storage",
	"diag",
	"modem",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_ums_diag_acm[] = {
	"mass_storage",
	"diag",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_ums_modem_acm[] = {
	"mass_storage",
	"modem",
	"acm",
};
static char *usb_functions_ums_diag_modem_acm[] = {
	"mass_storage",
	"diag",
	"modem",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_adb_ums_diag_serial_acm[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"serial",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_adb_ums_modem_serial_acm[] = {
	"mtp",
	"adb",
	"mass_storage",
	"modem",
	"serial",
	"acm",
};
static char *usb_functions_mtp_adb_ums_diag_modem_serial_acm[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"modem",
	"serial",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_ums_diag_serial_acm[] = {
	"mtp",
	"mass_storage",
	"diag",
	"serial",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_ums_modem_serial_acm[] = {
	"mtp",
	"mass_storage",
	"modem",
	"serial",
	"acm",
};
static char *usb_functions_mtp_ums_diag_modem_serial_acm[] = {
	"mtp",
	"mass_storage",
	"diag",
	"modem",
	"serial",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
};
static char *usb_functions_mtp_ums_projector_acm[] = {
	"mtp",
	"mass_storage",
	"projector",
	"acm",
};
static char *usb_functions_mtp_ums_adb_projector_acm[] = {
	"mtp",
	"adb",
	"mass_storage",
	"projector",
	"acm",
};
static char *usb_functions_mtp_ums_adb_acm_rmnet[] = {
	"mtp",
	"adb",
	"mass_storage",
	"acm",
	"rmnet",
};
static char *usb_functions_mtp_ums_acm_rmnet[] = {
	"mtp",
	"mass_storage",
	"acm",
	"rmnet",
};

#ifdef CONFIG_USB_ANDROID_NCM
	#ifdef CONFIG_USB_ANDROID_PROJECTOR
static char *usb_functions_ncm_adb_projector[] = {
	"cdc_network",
	"adb",
	"projector",
};
	#else
static char *usb_functions_ncm_adb[] = {
	"cdc_network",
	"adb",
};
	#endif
	#ifdef CONFIG_USB_ANDROID_PROJECTOR
static char *usb_functions_ncm_projector[] = {
	"cdc_network",
	"projector",
};
	#else
static char *usb_functions_ncm[] = {
	"cdc_network",
};
	#endif
#endif

static char *usb_functions_adb_diag_modem_svlte2[] = {
	"mass_storage",
	"adb",
	"diag",
	"modem",
	"modem_mdm",
	"diag_mdm",
};
static char *usb_functions_diag_modem_svlte2[] = {
	"mass_storage",
	"diag",
	"modem",
	"modem_mdm",
	"diag_mdm",
};
static char *usb_functions_mtp_ums_adb_diag_modem_svlte2[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"modem",
	"modem_mdm",
	"diag_mdm",
};
static char *usb_functions_mtp_ums_diag_modem_svlte2[] = {
	"mtp",
	"mass_storage",
	"diag",
	"modem",
	"modem_mdm",
	"diag_mdm",
};
static char *usb_functions_adb_modem_svlte2[] = {
	"mass_storage",
	"adb",
	"modem",
	"modem_mdm",
};
static char *usb_functions_modem_svlte2[] = {
	"mass_storage",
	"modem",
	"modem_mdm",
};
static char *usb_functions_mtp_ums_adb_modem_svlte2[] = {
	"mtp",
	"adb",
	"mass_storage",
	"modem",
	"modem_mdm",
};
static char *usb_functions_mtp_ums_modem_svlte2[] = {
	"mtp",
	"mass_storage",
	"modem",
	"modem_mdm",
};
static char *usb_functions_adb_diag_modem_svlte2_rment[] = {
	"mass_storage",
	"adb",
	"diag",
	"modem",
	"modem_mdm",
	"diag_mdm",
	"rmnet",
};
static char *usb_functions_diag_modem_svlte2_rment[] = {
	"mass_storage",
	"diag",
	"modem",
	"modem_mdm",
	"diag_mdm",
	"rmnet",
};
static char *usb_functions_mtp_ums_adb_diag_modem_svlte2_rment[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"modem",
	"modem_mdm",
	"diag_mdm",
	"rmnet",
};
static char *usb_functions_mtp_ums_diag_modem_svlte2_rment[] = {
	"mtp",
	"mass_storage",
	"diag",
	"modem",
	"modem_mdm",
	"diag_mdm",
	"rmnet",
};

static char *usb_functions_adb_diag_modem_rment[] = {
	"mass_storage",
	"adb",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
	"rmnet",
};
static char *usb_functions_diag_modem_rment[] = {
	"mass_storage",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
	"rmnet",
};
static char *usb_functions_mtp_ums_adb_diag_modem_rment[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
	"rmnet",
};
static char *usb_functions_mtp_ums_diag_modem_rment[] = {
	"mtp",
	"mass_storage",
	"diag",
	"modem",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
	"rmnet",
};

static char *usb_functions_adb_diag_rment[] = {
	"mass_storage",
	"adb",
	"diag",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
	"rmnet",
};
static char *usb_functions_diag_rment[] = {
	"mass_storage",
	"diag",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
	"rmnet",
};
static char *usb_functions_mtp_ums_adb_diag_rment[] = {
	"mtp",
	"adb",
	"mass_storage",
	"diag",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
	"rmnet",
};
static char *usb_functions_mtp_ums_diag_rment[] = {
	"mtp",
	"mass_storage",
	"diag",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
	"rmnet",
};
static char *usb_functions_mtp_modem[] = {
	"mtp",
	"modem",
};
static char *usb_functions_mtp_adb_modem[] = {
	"mtp",
	"adb",
	"modem",
};
static char *usb_functions_mtp_diag_modem[] = {
	"mtp",
	"diag",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
	"modem",
};
static char *usb_functions_mtp_adb_diag_modem[] = {
	"mtp",
	"adb",
	"diag",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
	"modem",
};
static char *usb_functions_adb_rment[] = {
	"mass_storage",
	"adb",
	"rmnet",
};
static char *usb_functions_rment[] = {
	"mass_storage",
	"rmnet",
};
static char *usb_functions_mtp_ums_adb_rment[] = {
	"mtp",
	"adb",
	"mass_storage",
	"rmnet",
};
static char *usb_functions_mtp_ums_rment[] = {
	"mtp",
	"mass_storage",
	"rmnet",
};

static __maybe_unused char *usb_functions_all[] = {
	"rndis",
	"accessory",
	"mtp",
	"ptp",
#if defined(CONFIG_USB_ANDROID_MTP36) && defined(CONFIG_USB_ANDROID_MTP)
	"mtp36",
#endif
#ifdef CONFIG_USB_ANDROID_NCM
	"cdc_network",
#endif
	"adb",
	"mass_storage",
	"cdc_ethernet",
	"diag",
	"serial",
	"projector",
	"acm",
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	"diag_mdm",
#endif
	"rmnet",
#ifdef CONFIG_USB_ANDROID_USBNET
	"usbnet",
#endif
	 "audio_source",
};

static struct android_usb_product usb_products[] = {
	{
		.product_id = 0x0f90, 
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb),
		.functions	= usb_functions_mtp_ums_adb,
	},
	{
		.product_id = 0x0c02, 
		.num_functions	= ARRAY_SIZE(usb_functions_adb),
		.functions	= usb_functions_adb,
	},
	{
		.product_id	= 0x0f25,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums),
		.functions	= usb_functions_mtp_ums,
	},
	{
		.product_id	= 0x0ff9,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
#ifdef CONFIG_USB_ANDROID_USBNET
	{
		.product_id	= 0x0fcd,
		.num_functions	= ARRAY_SIZE(usb_functions_usbnet),
		.functions	= usb_functions_usbnet,
	},
	{
		.product_id	= 0x0fce,
		.num_functions	= ARRAY_SIZE(usb_functions_usbnet_adb),
		.functions	= usb_functions_usbnet_adb,
	},
#endif
	{
		.product_id	= 0x0ff4,
		.num_functions	= ARRAY_SIZE(usb_functions_acm),
		.functions	= usb_functions_acm,
	},
	{
		.product_id	= 0x0ff5,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_acm),
		.functions	= usb_functions_adb_acm,
	},
	{
		.product_id	= 0x0ff8,
		.num_functions	= ARRAY_SIZE(usb_functions_ecm),
		.functions	= usb_functions_ecm,
	},
#ifdef CONFIG_USB_ANDROID_NCM
	{
	#ifdef CONFIG_USB_ANDROID_PROJECTOR
		.product_id	= 0x0f8b,
		.num_functions	= ARRAY_SIZE(usb_functions_ncm_adb_projector),
		.functions	= usb_functions_ncm_adb_projector,
	#else
		.product_id	= 0x0f92,
		.num_functions	= ARRAY_SIZE(usb_functions_ncm_adb),
		.functions	= usb_functions_ncm_adb,
	#endif
	},
	{
	#ifdef CONFIG_USB_ANDROID_PROJECTOR
		.product_id	= 0x0f8a,
		.num_functions	= ARRAY_SIZE(usb_functions_ncm_projector),
		.functions	= usb_functions_ncm_projector,
	#else
		.product_id	= 0x0f93,
		.num_functions	= ARRAY_SIZE(usb_functions_ncm),
		.functions	= usb_functions_ncm,
	#endif
	},
#endif
	{
		.product_id	= 0x0fc5,
		.num_functions	= ARRAY_SIZE(usb_functions_serial_modem),
		.functions	= usb_functions_serial_modem,
	},
	{
		.product_id	= 0x0fc6,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_serial_modem),
		.functions	= usb_functions_adb_serial_modem,
	},
	{
		.product_id	= 0x0f9e,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_serial_modem),
		.functions	= usb_functions_mtp_ums_serial_modem,
	},
	{
		.product_id	= 0x0f9f,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_serial_modem),
		.functions	= usb_functions_mtp_ums_adb_serial_modem,
	},
	{
		.product_id	= 0x0fd1,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_serial),
		.functions	= usb_functions_adb_serial,
	},
	{
		.product_id	= 0x0fa0,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_serial),
		.functions	= usb_functions_mtp_ums_adb_serial,
	},

	{
		.product_id	= 0x0c03,
		.num_functions	= ARRAY_SIZE(usb_functions_modem),
		.functions	= usb_functions_modem,
	},
	{
		.product_id	= 0x0c04,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_modem),
		.functions	= usb_functions_adb_modem,
	},
	{
		.product_id	= 0x0f95,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_modem),
		.functions	= usb_functions_mtp_ums_adb_modem,
	},
	{
		.product_id	= 0x0f96,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_modem),
		.functions	= usb_functions_mtp_ums_modem,
	},
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	{
		.product_id	= 0x0fde,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_modem),
		.functions	= usb_functions_adb_diag_modem,
	},
	{
		.product_id	= 0x0fdf,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_serial),
		.functions	= usb_functions_diag_serial,
	},
	{
		.product_id	= 0x0fe0,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_serial),
		.functions	= usb_functions_adb_diag_serial,
	},
	{
		.product_id	= 0x0fe1,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_serial_modem),
		.functions	= usb_functions_adb_diag_serial_modem,
	},
	{
		.product_id	= 0x0fe2,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_serial_modem),
		.functions	= usb_functions_diag_serial_modem,
	},
	{
		.product_id	= 0x0fe7,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_modem),
		.functions	= usb_functions_diag_modem,
	},
	{
		.product_id	= 0x0fad,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_modem),
		.functions	= usb_functions_mtp_ums_adb_diag_modem,
	},
	{
		.product_id	= 0x0fae,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_serial),
		.functions	= usb_functions_mtp_ums_diag_serial,
	},
	{
		.product_id	= 0x0faf,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_serial),
		.functions	= usb_functions_mtp_ums_adb_diag_serial,
	},
	{
		.product_id	= 0x0fb0,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_serial_modem),
		.functions	= usb_functions_mtp_ums_adb_diag_serial_modem,
	},
	{
		.product_id	= 0x0fb1,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_serial_modem),
		.functions	= usb_functions_mtp_ums_diag_serial_modem,
	},
	{
		.product_id	= 0x0fb6,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_modem),
		.functions	= usb_functions_mtp_ums_diag_modem,
	},
#else
	{
		.product_id	= 0x0c88,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_modem),
		.functions	= usb_functions_adb_diag_modem,
	},
	{
		.product_id	= 0x0c89,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_serial),
		.functions	= usb_functions_diag_serial,
	},
	{
		.product_id	= 0x0c8a,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_serial),
		.functions	= usb_functions_adb_diag_serial,
	},
	{
		.product_id	= 0x0f9b,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_modem),
		.functions	= usb_functions_mtp_ums_adb_diag_modem,
	},
	{
		.product_id	= 0x0f9c,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_serial),
		.functions	= usb_functions_mtp_ums_diag_serial,
	},
	{
		.product_id	= 0x0f9d,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_serial),
		.functions	= usb_functions_mtp_ums_adb_diag_serial,
	},
	{
		.product_id	= 0x0fe8,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_serial_modem),
		.functions	= usb_functions_adb_diag_serial_modem,
	},
	{
		.product_id	= 0x0fe9,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_serial_modem),
		.functions	= usb_functions_diag_serial_modem,
	},
	{
		.product_id	= 0x0ffb,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_modem),
		.functions	= usb_functions_diag_modem,
	},
	{
		.product_id	= 0x0fa6,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_serial_modem),
		.functions	= usb_functions_mtp_ums_adb_diag_serial_modem,
	},
	{
		.product_id	= 0x0fa7,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_serial_modem),
		.functions	= usb_functions_mtp_ums_diag_serial_modem,
	},
	{
		.product_id	= 0x0faa,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_modem),
		.functions	= usb_functions_mtp_ums_diag_modem,
	},
#endif
	{
		.product_id	= 0x0c05,
		.num_functions	= ARRAY_SIZE(usb_functions_projector),
		.functions	= usb_functions_projector,
	},
	{
		.product_id	= 0x0c06,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_projector),
		.functions	= usb_functions_adb_projector,
	},
	{
		.product_id	= 0x0f97,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_projector),
		.functions	= usb_functions_mtp_ums_adb_projector,
	},
	{
		.product_id	= 0x0f98,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_projector),
		.functions	= usb_functions_mtp_ums_projector,
	},
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	{
		.product_id	= 0x0FE3,
		.num_functions	= ARRAY_SIZE(usb_function_adb_diag_projector),
		.functions	= usb_function_adb_diag_projector,
	},
	{
		.product_id	= 0x0FB2,
		.num_functions	= ARRAY_SIZE(usb_function_mtp_ums_adb_diag_projector),
		.functions	= usb_function_mtp_ums_adb_diag_projector,
	},
	{
		.product_id	= 0x0FE4,
		.num_functions	= ARRAY_SIZE(usb_function_adb_diag_modem_projector),
		.functions	= usb_function_adb_diag_modem_projector,
	},
	{
		.product_id	= 0x0FB3,
		.num_functions	= ARRAY_SIZE(usb_function_mtp_ums_adb_diag_modem_projector),
		.functions	= usb_function_mtp_ums_adb_diag_modem_projector,
	},
#else
	{
		.product_id	= 0x0FF1,
		.num_functions	= ARRAY_SIZE(usb_function_adb_diag_projector),
		.functions	= usb_function_adb_diag_projector,
	},
	{
		.product_id	= 0x0FA8,
		.num_functions	= ARRAY_SIZE(usb_function_mtp_ums_adb_diag_projector),
		.functions	= usb_function_mtp_ums_adb_diag_projector,
	},
	{
		.product_id	= 0x0FF2,
		.num_functions	= ARRAY_SIZE(usb_function_adb_diag_modem_projector),
		.functions	= usb_function_adb_diag_modem_projector,
	},
	{
		.product_id	= 0x0FA9,
		.num_functions	= ARRAY_SIZE(usb_function_mtp_ums_adb_diag_modem_projector),
		.functions	= usb_function_mtp_ums_adb_diag_modem_projector,
	},
#endif
	{
		.product_id	= 0x0FDC,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag2),
		.functions	= usb_functions_adb_diag2,
	},
	{
		.product_id	= 0x0FDD,
		.num_functions	= ARRAY_SIZE(usb_functions_diag2),
		.functions	= usb_functions_diag2,
	},
	{
		.product_id	= 0x0FAB,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag2),
		.functions	= usb_functions_mtp_ums_adb_diag2,
	},
	{
		.product_id	= 0x0FAC,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag2),
		.functions	= usb_functions_mtp_ums_diag2,
	},
	{
		.product_id	= 0x0c07,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag),
		.functions	= usb_functions_adb_diag,
	},
	{
		.product_id	= 0x0c08,
		.num_functions	= ARRAY_SIZE(usb_functions_diag),
		.functions	= usb_functions_diag,
	},
	{
		.product_id	= 0x0f99,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag),
		.functions	= usb_functions_mtp_ums_adb_diag,
	},
	{
		.product_id	= 0x0f9a,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag),
		.functions	= usb_functions_mtp_ums_diag,
	},
	{
		.product_id	= 0x0f87,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_adb),
		.functions	= usb_functions_mtp_adb,
	},
	{
		.product_id	= 0x0c93,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp),
		.functions	= usb_functions_mtp,
	},
#if defined(CONFIG_USB_ANDROID_MTP36) && defined(CONFIG_USB_ANDROID_MTP)
	{
		.product_id	= 0x0ca8,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp36_adb),
		.functions	= usb_functions_mtp36_adb,
	},
	{
		.product_id	= 0x0c93,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp36),
		.functions	= usb_functions_mtp36,
	},
#endif
	{
		.product_id	= 0x0ffe,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	
	{
		.product_id	= 0x0f80,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_modem),
		.functions	= usb_functions_rndis_modem,
	},
	{
		.product_id	= 0x0ffc,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
	
	{
		.product_id	= 0x0f81,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb_modem),
		.functions	= usb_functions_rndis_adb_modem,
	},
	{
		.product_id	= 0x0fb4,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_adb),
		.functions	= usb_functions_rndis_mtp_ums_adb,
	},
	
	{
		.product_id	= 0x0f7b,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_adb_modem),
		.functions	= usb_functions_rndis_mtp_ums_adb_modem,
	},
	{
		.product_id	= 0x0fb5,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums),
		.functions	= usb_functions_rndis_mtp_ums,
	},
	
	{
		.product_id	= 0x0f5f,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_adb_ums_acm),
		.functions	= usb_functions_rndis_mtp_adb_ums_acm,
	},
	
	{
		.product_id	= 0x0f60,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_acm),
		.functions	= usb_functions_rndis_mtp_ums_acm,
	},
	
	{
		.product_id	= 0x0f61,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb_ums_acm),
		.functions	= usb_functions_rndis_adb_ums_acm,
	},
	
	{
		.product_id	= 0x0f62,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_ums_acm),
		.functions	= usb_functions_rndis_ums_acm,
	},
	
	{
		.product_id	= 0x0f63,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_adb_ums_acm),
		.functions	= usb_functions_mtp_adb_ums_acm,
	},
	
	{
		.product_id	= 0x0f64,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_acm),
		.functions	= usb_functions_mtp_ums_acm,
	},
	
	{
		.product_id	= 0x0f65,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_acm),
		.functions	= usb_functions_ums_acm,
	},
	
	{
		.product_id	= 0x0f39,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_adb_ums_modem_acm),
		.functions	= usb_functions_rndis_mtp_adb_ums_modem_acm,
	},
	
	{
		.product_id	= 0x0f3e,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_modem_acm),
		.functions	= usb_functions_rndis_mtp_ums_modem_acm,
	},
	
	{
		.product_id	= 0x0f43,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_adb_ums_modem_acm),
		.functions	= usb_functions_mtp_adb_ums_modem_acm,
	},
	
	{
		.product_id	= 0x0f48,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_modem_acm),
		.functions	= usb_functions_mtp_ums_modem_acm,
	},
	
	{
		.product_id	= 0x0f4d,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb_ums_modem_acm),
		.functions	= usb_functions_rndis_adb_ums_modem_acm,
	},
	
	{
		.product_id	= 0x0f52,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_ums_modem_acm),
		.functions	= usb_functions_rndis_ums_modem_acm,
	},
	
	{
		.product_id	= 0x0f57,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_ums_modem_acm),
		.functions	= usb_functions_adb_ums_modem_acm,
	},
	
	{
		.product_id	= 0x0f5c,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_modem_acm),
		.functions	= usb_functions_ums_modem_acm,
	},
	
	{
		.product_id	= 0x0f2f,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_adb_ums_modem_serial_acm),
		.functions	= usb_functions_mtp_adb_ums_modem_serial_acm,
	},
	
	{
		.product_id	= 0x0f34,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_modem_serial_acm),
		.functions	= usb_functions_mtp_ums_modem_serial_acm,
	},
	
	{
		.product_id	= 0x0f2c,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_projector_acm),
		.functions	= usb_functions_mtp_ums_projector_acm,
	},
	
	{
		.product_id	= 0x0f2b,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_projector_acm),
		.functions	= usb_functions_mtp_ums_adb_projector_acm,
	},
	
	{
		.product_id	= 0x0f29,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_acm_rmnet),
		.functions	= usb_functions_mtp_ums_adb_acm_rmnet,
	},
	
	{
		.product_id	= 0x0f2a,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_acm_rmnet),
		.functions	= usb_functions_mtp_ums_acm_rmnet,
	},

#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	{
		.product_id	= 0x0f38,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_adb_ums_diag_acm),
		.functions	= usb_functions_rndis_mtp_adb_ums_diag_acm,
	},
	{
		.product_id	= 0x0f3b,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_adb_ums_diag_modem_acm),
		.functions	= usb_functions_rndis_mtp_adb_ums_diag_modem_acm,
	},
	{
		.product_id	= 0x0f3d,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_diag_acm),
		.functions	= usb_functions_rndis_mtp_ums_diag_acm,
	},
	{
		.product_id	= 0x0f40,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_diag_modem_acm),
		.functions	= usb_functions_rndis_mtp_ums_diag_modem_acm,
	},
	{
		.product_id	= 0x0f42,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_adb_ums_diag_acm),
		.functions	= usb_functions_mtp_adb_ums_diag_acm,
	},
	{
		.product_id	= 0x0f45,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_adb_ums_diag_modem_acm),
		.functions	= usb_functions_mtp_adb_ums_diag_modem_acm,
	},
	{
		.product_id	= 0x0f47,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_acm),
		.functions	= usb_functions_mtp_ums_diag_acm,
	},
	{
		.product_id	= 0x0f4a,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_modem_acm),
		.functions	= usb_functions_mtp_ums_diag_modem_acm,
	},
	{
		.product_id	= 0x0f4c,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb_ums_diag_acm),
		.functions	= usb_functions_rndis_adb_ums_diag_acm,
	},
	{
		.product_id	= 0x0f4f,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb_ums_diag_modem_acm),
		.functions	= usb_functions_rndis_adb_ums_diag_modem_acm,
	},
	{
		.product_id	= 0x0f51,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_ums_diag_acm),
		.functions	= usb_functions_rndis_ums_diag_acm,
	},
	{
		.product_id	= 0x0f54,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_ums_diag_modem_acm),
		.functions	= usb_functions_rndis_ums_diag_modem_acm,
	},
	{
		.product_id	= 0x0f56,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_ums_diag_acm),
		.functions	= usb_functions_adb_ums_diag_acm,
	},
	{
		.product_id	= 0x0f59,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_ums_diag_modem_acm),
		.functions	= usb_functions_adb_ums_diag_modem_acm,
	},
	{
		.product_id	= 0x0f5b,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_diag_acm),
		.functions	= usb_functions_ums_diag_acm,
	},
	{
		.product_id	= 0x0f5e,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_diag_modem_acm),
		.functions	= usb_functions_ums_diag_modem_acm,
	},
	{
		.product_id	= 0x0f2e,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_adb_ums_diag_serial_acm),
		.functions	= usb_functions_mtp_adb_ums_diag_serial_acm,
	},
	{
		.product_id	= 0x0f31,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_adb_ums_diag_modem_serial_acm),
		.functions	= usb_functions_mtp_adb_ums_diag_modem_serial_acm,
	},
	{
		.product_id	= 0x0f33,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_serial_acm),
		.functions	= usb_functions_mtp_ums_diag_serial_acm,
	},
	{
		.product_id	= 0x0f36,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_modem_serial_acm),
		.functions	= usb_functions_mtp_ums_diag_modem_serial_acm,
	},
#else
	{
		.product_id	= 0x0f37,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_adb_ums_diag_acm),
		.functions	= usb_functions_rndis_mtp_adb_ums_diag_acm,
	},
	{
		.product_id	= 0x0f3a,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_adb_ums_diag_modem_acm),
		.functions	= usb_functions_rndis_mtp_adb_ums_diag_modem_acm,
	},
	{
		.product_id	= 0x0f3c,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_diag_acm),
		.functions	= usb_functions_rndis_mtp_ums_diag_acm,
	},
	{
		.product_id	= 0x0f3f,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_diag_modem_acm),
		.functions	= usb_functions_rndis_mtp_ums_diag_modem_acm,
	},
	{
		.product_id	= 0x0f41,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_adb_ums_diag_acm),
		.functions	= usb_functions_mtp_adb_ums_diag_acm,
	},
	{
		.product_id	= 0x0f44,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_adb_ums_diag_modem_acm),
		.functions	= usb_functions_mtp_adb_ums_diag_modem_acm,
	},
	{
		.product_id	= 0x0f46,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_acm),
		.functions	= usb_functions_mtp_ums_diag_acm,
	},
	{
		.product_id	= 0x0f49,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_modem_acm),
		.functions	= usb_functions_mtp_ums_diag_modem_acm,
	},
	{
		.product_id	= 0x0f4b,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb_ums_diag_acm),
		.functions	= usb_functions_rndis_adb_ums_diag_acm,
	},
	{
		.product_id	= 0x0f4e,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb_ums_diag_modem_acm),
		.functions	= usb_functions_rndis_adb_ums_diag_modem_acm,
	},
	{
		.product_id	= 0x0f50,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_ums_diag_acm),
		.functions	= usb_functions_rndis_ums_diag_acm,
	},
	{
		.product_id	= 0x0f53,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_ums_diag_modem_acm),
		.functions	= usb_functions_rndis_ums_diag_modem_acm,
	},
	{
		.product_id	= 0x0f55,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_ums_diag_acm),
		.functions	= usb_functions_adb_ums_diag_acm,
	},
	{
		.product_id	= 0x0f58,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_ums_diag_modem_acm),
		.functions	= usb_functions_adb_ums_diag_modem_acm,
	},
	{
		.product_id	= 0x0f5a,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_diag_acm),
		.functions	= usb_functions_ums_diag_acm,
	},
	{
		.product_id	= 0x0f5d,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_diag_modem_acm),
		.functions	= usb_functions_ums_diag_modem_acm,
	},
	{
		.product_id	= 0x0f2d,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_adb_ums_diag_serial_acm),
		.functions	= usb_functions_mtp_adb_ums_diag_serial_acm,
	},
	{
		.product_id	= 0x0f30,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_adb_ums_diag_modem_serial_acm),
		.functions	= usb_functions_mtp_adb_ums_diag_modem_serial_acm,
	},
	{
		.product_id	= 0x0f32,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_serial_acm),
		.functions	= usb_functions_mtp_ums_diag_serial_acm,
	},
	{
		.product_id	= 0x0f35,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_modem_serial_acm),
		.functions	= usb_functions_mtp_ums_diag_modem_serial_acm,
	},
#endif
	
	{
		.product_id	= 0x0f66,
		.num_functions	= ARRAY_SIZE(usb_functions_ptp),
		.functions	= usb_functions_ptp,
	},
	
	{
		.product_id	= 0x0f67,
		.num_functions	= ARRAY_SIZE(usb_functions_ptp_adb),
		.functions	= usb_functions_ptp_adb,
	},
	
	{
		.product_id	= 0x0f74,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_modem),
		.functions	= usb_functions_mtp_modem,
	},
	
	{
		.product_id	= 0x0f75,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_adb_modem),
		.functions	= usb_functions_mtp_adb_modem,
	},
	
	{
		.product_id	= 0x0f7a,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_modem),
		.functions	= usb_functions_rndis_mtp_ums_modem,
	},
#if defined(CONFIG_USB_ANDROID_MDM9K_DIAG)
	{
		.product_id	= 0x0fe5,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb_diag),
		.functions	= usb_functions_rndis_adb_diag,
	},
	
	{
		.product_id	= 0x0f85,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb_diag_modem),
		.functions	= usb_functions_rndis_adb_diag_modem,
	},
	{
		.product_id	= 0x0fe6,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_diag),
		.functions	= usb_functions_rndis_diag,
	},
	
	{
		.product_id	= 0x0f84,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_diag_modem),
		.functions	= usb_functions_rndis_diag_modem,
	},
	{
		.product_id	= 0x0f8e,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_adb_diag),
		.functions	= usb_functions_rndis_mtp_ums_adb_diag,
	},
	
	{
		.product_id	= 0x0f7d,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_adb_diag_modem),
		.functions	= usb_functions_rndis_mtp_ums_adb_diag_modem,
	},
	{
		.product_id	= 0x0f8f,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_diag),
		.functions	= usb_functions_rndis_mtp_ums_diag,
	},
	
	{
		.product_id	= 0x0f7c,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_diag_modem),
		.functions	= usb_functions_rndis_mtp_ums_diag_modem,
	},
	
	{
		.product_id	= 0x0f78,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_diag_modem),
		.functions	= usb_functions_mtp_diag_modem,
	},
	
	{
		.product_id	= 0x0f79,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_adb_diag_modem),
		.functions	= usb_functions_mtp_adb_diag_modem,
	},
#else
	{
		.product_id	= 0x0f8d,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_diag),
		.functions	= usb_functions_rndis_mtp_ums_diag,
	},
	
	{
		.product_id	= 0x0f7e,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_diag_modem),
		.functions	= usb_functions_rndis_mtp_ums_diag_modem,
	},
	{
		.product_id	= 0x0f8c,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_adb_diag),
		.functions	= usb_functions_rndis_mtp_ums_adb_diag,
	},
	
	{
		.product_id	= 0x0f7f,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_mtp_ums_adb_diag_modem),
		.functions	= usb_functions_rndis_mtp_ums_adb_diag_modem,
	},
	{
		.product_id	= 0x0ff6,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb_diag),
		.functions	= usb_functions_rndis_adb_diag,
	},
	
	{
		.product_id	= 0x0f83,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb_diag_modem),
		.functions	= usb_functions_rndis_adb_diag_modem,
	},
	{
		.product_id	= 0x0ff7,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_diag),
		.functions	= usb_functions_rndis_diag,
	},
	
	{
		.product_id	= 0x0f82,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_diag_modem),
		.functions	= usb_functions_rndis_diag_modem,
	},
	
	{
		.product_id	= 0x0f76,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_diag_modem),
		.functions	= usb_functions_mtp_diag_modem,
	},
	
	{
		.product_id	= 0x0f77,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_adb_diag_modem),
		.functions	= usb_functions_mtp_adb_diag_modem,
	},
#endif
	{
		.vendor_id	= USB_ACCESSORY_VENDOR_ID,
		.product_id	= USB_ACCESSORY_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_accessory),
		.functions	= usb_functions_accessory,
	},
	{
		.vendor_id	= USB_ACCESSORY_VENDOR_ID,
		.product_id	= USB_ACCESSORY_ADB_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_accessory_adb),
		.functions	= usb_functions_accessory_adb,
	},
	{ 
		.vendor_id	= USB_ACCESSORY_VENDOR_ID,
		.product_id	= USB_AUDIO_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_audio),
		.functions	= usb_functions_audio,
	},
	{
		.vendor_id	= USB_ACCESSORY_VENDOR_ID,
		.product_id	= USB_AUDIO_ADB_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_audio_adb),
		.functions	= usb_functions_audio_adb,
	},
	{
		.vendor_id	= USB_ACCESSORY_VENDOR_ID,
		.product_id	= USB_ACCESSORY_AUDIO_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_accessory_audio),
		.functions	= usb_functions_accessory_audio,
	},
	{
		.vendor_id	= USB_ACCESSORY_VENDOR_ID,
		.product_id	= USB_ACCESSORY_AUDIO_ADB_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_accessory_audio_adb),
		.functions	= usb_functions_accessory_audio_adb,
	},
#ifdef CONFIG_USB_ANDROID_MDM9K_DIAG
	{
		.product_id	= 0x0fd2,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_modem_rment),
		.functions	= usb_functions_adb_diag_modem_rment,
	},
	{
		.product_id	= 0x0fd3,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_modem_rment),
		.functions	= usb_functions_diag_modem_rment,
	},
	{
		.product_id	= 0x0fd4,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_rment),
		.functions	= usb_functions_adb_diag_rment,
	},
	{
		.product_id	= 0x0fd5,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_rment),
		.functions	= usb_functions_diag_rment,
	},
	{
		.product_id	= 0x0fbd,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_modem_rment),
		.functions	= usb_functions_mtp_ums_adb_diag_modem_rment,
	},
	{
		.product_id	= 0x0fbe,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_modem_rment),
		.functions	= usb_functions_mtp_ums_diag_modem_rment,
	},
	{
		.product_id	= 0x0fbf,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_rment),
		.functions	= usb_functions_mtp_ums_adb_diag_rment,
	},
	{
		.product_id	= 0x0fc0,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_rment),
		.functions	= usb_functions_mtp_ums_diag_rment,
	},
#endif
	{
		.product_id	= 0x0fd6,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_rment),
		.functions	= usb_functions_adb_diag_rment,
	},
	{
		.product_id	= 0x0fd7,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_rment),
		.functions	= usb_functions_diag_rment,
	},

	{
		.product_id	= 0x0fd8,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_modem_rment),
		.functions	= usb_functions_adb_diag_modem_rment,
	},
	{
		.product_id	= 0x0fd9,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_modem_rment),
		.functions	= usb_functions_diag_modem_rment,
	},
	{
		.product_id	= 0x0fda,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_rment),
		.functions	= usb_functions_adb_rment,
	},
	{
		.product_id	= 0x0fdb,
		.num_functions	= ARRAY_SIZE(usb_functions_rment),
		.functions	= usb_functions_rment,
	},
	{
		.product_id	= 0x0fc1,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_rment),
		.functions	= usb_functions_mtp_ums_adb_diag_rment,
	},
	{
		.product_id	= 0x0fa1,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_rment),
		.functions	= usb_functions_mtp_ums_diag_rment,
	},

	{
		.product_id	= 0x0fa2,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_modem_rment),
		.functions	= usb_functions_mtp_ums_adb_diag_modem_rment,
	},
	{
		.product_id	= 0x0fa3,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_modem_rment),
		.functions	= usb_functions_mtp_ums_diag_modem_rment,
	},
	{
		.product_id	= 0x0fa4,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_rment),
		.functions	= usb_functions_mtp_ums_adb_rment,
	},
	{
		.product_id	= 0x0fa5,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_rment),
		.functions	= usb_functions_mtp_ums_rment,
	},
	{
		.product_id	= 0x0fcf,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_modem_svlte2_rment),
		.functions	= usb_functions_adb_diag_modem_svlte2_rment,
	},
	{
		.product_id	= 0x0fd0,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_modem_svlte2_rment),
		.functions	= usb_functions_diag_modem_svlte2_rment,
	},
	{
		.product_id	= 0x0fbb,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_modem_svlte2_rment),
		.functions	= usb_functions_mtp_ums_adb_diag_modem_svlte2_rment,
	},
	{
		.product_id	= 0x0fbc,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_modem_svlte2_rment),
		.functions	= usb_functions_mtp_ums_diag_modem_svlte2_rment,
	},
	{
		.product_id	= 0x0fc9,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag_modem_svlte2),
		.functions	= usb_functions_adb_diag_modem_svlte2,
	},
	{
		.product_id	= 0x0fca,
		.num_functions	= ARRAY_SIZE(usb_functions_diag_modem_svlte2),
		.functions	= usb_functions_diag_modem_svlte2,
	},
	{
		.product_id	= 0x0fcb,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_modem_svlte2),
		.functions	= usb_functions_adb_modem_svlte2,
	},
	{
		.product_id	= 0x0fcc,
		.num_functions	= ARRAY_SIZE(usb_functions_modem_svlte2),
		.functions	= usb_functions_modem_svlte2,
	},
	{
		.product_id	= 0x0fb7,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_diag_modem_svlte2),
		.functions	= usb_functions_mtp_ums_adb_diag_modem_svlte2,
	},
	{
		.product_id	= 0x0fb8,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_diag_modem_svlte2),
		.functions	= usb_functions_mtp_ums_diag_modem_svlte2,
	},
	{
		.product_id	= 0x0fb9,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_adb_modem_svlte2),
		.functions	= usb_functions_mtp_ums_adb_modem_svlte2,
	},
	{
		.product_id	= 0x0fba,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_ums_modem_svlte2),
		.functions	= usb_functions_mtp_ums_modem_svlte2,
	},
};

static __maybe_unused int qsc_modem_product_id_match_array[] = {
	0x0fad, 0x0f68, 
	0x0fb6, 0x0f69, 
	0x0f7d, 0x0f6a, 
	0x0f7c, 0x0f6b, 
	0x0fab, 0x0f6c, 
	0x0fac, 0x0f6d, 
	0x0faf, 0x0f6e, 
	0x0fae, 0x0f6f, 
	0x0fbd, 0x0f70, 
	0x0fbe, 0x0f71, 
	0x0f8e, 0x0f72, 
	0x0f8f, 0x0f73, 
	-1,
};

static __maybe_unused int qsc_modem_usb_id_match(int product_id, int intrsharing)
{
	int *pid_array = qsc_modem_product_id_match_array;

	if (!pid_array)
		return product_id;

	while (pid_array && pid_array[0] >= 0) {
		if (product_id == pid_array[0])
			return pid_array[1];
		pid_array += 2;
	}
	return product_id;
}

#ifdef CONFIG_USB_GADGET_VERIZON_PRODUCT_ID
#ifdef CONFIG_MACH_VIGOR
static int vigor_usb_product_id_match_array[] = {
	0x0c93, 0x0e05, 
	0x0ca8, 0x0e06, 
	0x0fda, 0x0e07, 
	0x0fdb, 0x0e08, 
	0x0c07, 0x0e0b, 
	0x0c08, 0x0e0c, 
	0x0ff8, 0x0e0d, 
	0x0fd5, 0x0e0e, 
	0x0fd4, 0x0e0f, 
	0x0ff9, 0x0e73, 
	0x0fa4, 0x0e8f, 
	0x0fa5, 0x0e90, 
	0x0f99, 0x0e91, 
	0x0f9a, 0x0e92, 
	0x0fc0, 0x0e93, 
	0x0fbf, 0x0e94, 
	-1,
};

static int vigor_usb_product_id_rndis[] = {
	0x0e09, 0x0e0a,
};
#endif 

#ifdef CONFIG_MACH_VIVOW
static int vivow_usb_product_id_match_array[] = {
	0x0c93, 0x0e10, 
	0x0ca8, 0x0e11, 
	0x0fda, 0x0e12, 
	0x0fdb, 0x0e13, 
	0x0c07, 0x0e16, 
	0x0c08, 0x0e17, 
	0x0ff8, 0x0e18, 
	0x0ff9, 0x0e74, 
	0x0fa4, 0x0e87, 
	0x0fa5, 0x0e88, 
	0x0f99, 0x0e89, 
	0x0f9a, 0x0e8a, 
	-1,
};

static int vivow_usb_product_id_rndis[] = {
	0x0e14, 0x0e15,
};
#endif 

#ifdef CONFIG_MACH_MECHA
static int mecha_usb_product_id_match_array[] = {
	0x0c93, 0x0e1b, 
	0x0ca8, 0x0e1c, 
	0x0fda, 0x0e1d, 
	0x0fdb, 0x0e1e, 
	0x0c07, 0x0e21, 
	0x0c08, 0x0e22, 
	0x0ff8, 0x0e23, 
	0x0fd5, 0x0e24, 
	0x0fd4, 0x0e25, 
	0x0ff9, 0x0e75, 
	0x0fa4, 0x0e95, 
	0x0fa5, 0x0e96, 
	0x0f99, 0x0e97, 
	0x0f9a, 0x0e98, 
	0x0fc0, 0x0e99, 
	0x0fbf, 0x0e9a, 
	-1,
};

static int mecha_usb_product_id_rndis[] = {
	0x0e1f, 0x0e20,
};
#endif 

#ifdef CONFIG_MACH_BLISSC
static int blissc_usb_product_id_match_array[] = {
	0x0c93, 0x0e26, 
	0x0ca8, 0x0e27, 
	0x0fda, 0x0e28, 
	0x0fdb, 0x0e29, 
	0x0c07, 0x0e2c, 
	0x0c08, 0x0e2d, 
	0x0ff8, 0x0e2e, 
	0x0ff9, 0x0ccb, 
	0x0ffc, 0x0e7d, 
	0x0fa4, 0x0e9b, 
	0x0fa5, 0x0e9c, 
	0x0f99, 0x0e9d, 
	0x0f9a, 0x0e9e, 
	-1,
};

static int blissc_usb_product_id_rndis[] = {
	0x0e2a, 0x0e2b,
};
#endif 
#endif 

#endif
