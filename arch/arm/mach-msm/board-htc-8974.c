/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/memory.h>
#include <linux/persistent_ram.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/krait-regulator.h>
#include <linux/msm_tsens.h>
#include <linux/msm_thermal.h>
#include <asm/mach/map.h>
#include <asm/hardware/gic.h>
#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include <mach/board.h>
#include <mach/gpiomux.h>
#include <linux/gpio.h>
#include <mach/msm_iomap.h>
#ifdef CONFIG_ION_MSM
#include <mach/ion.h>
#endif
#include <mach/msm_memtypes.h>
#include <mach/msm_smd.h>
#include <mach/restart.h>
#include <mach/rpm-smd.h>
#include <mach/rpm-regulator-smd.h>
#include <mach/socinfo.h>
#include <mach/msm_smem.h>
#include "board-dt.h"
#include "clock.h"
#include "devices.h"
#include "spm.h"
#include "pm.h"
#include "modem_notifier.h"
#include "platsmp.h"
#include <linux/usb/android.h>
#include <mach/cable_detect.h>
#include <mach/devices_cmdline.h>
#ifdef CONFIG_HTC_POWER_DEBUG
#include <mach/htc_util.h>
#include <mach/devices_dtb.h>
#endif
#ifdef CONFIG_HTC_BATT_8960
#include "linux/mfd/pm8xxx/pm8921-charger.h"
#include "linux/mfd/pm8xxx/pm8921-bms.h"
#include "linux/mfd/pm8xxx/batt-alarm.h"
#include "mach/htc_battery_8960.h"
#include "mach/htc_battery_cell.h"
#include <linux/qpnp/qpnp-charger.h>
#include <linux/qpnp/qpnp-bms.h>
#endif 

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
#include <mach/htc_mnemosyne.h>
#endif

#include <linux/qpnp/qpnp-adc.h>

#ifdef CONFIG_BT
#include <mach/htc_bdaddress.h>
#endif

#ifdef CONFIG_PERFLOCK
#include <mach/perflock.h>
#endif

#ifdef CONFIG_HTC_BUILD_EDIAG
#include <linux/android_ediagpmem.h>
#endif

#if defined(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8240_SII8558) && defined(CONFIG_HTC_MHL_DETECTION)
#include "../../../drivers/video/msm/mdss/sii8240_8558/mhl_platform.h"
#endif
#if defined(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII9234) && defined(CONFIG_HTC_MHL_DETECTION)
#include <mach/mhl.h>
#include "../../../drivers/video/msm/mdss/sii9234/TPI.h"
extern void mhl_sii9234_1v2_power(bool enable);
#endif
static int mhl_usb_sw_gpio;

#define HTC_8974_PERSISTENT_RAM_PHYS 0x05B00000
#ifdef CONFIG_HTC_BUILD_EDIAG
#define HTC_8974_PERSISTENT_RAM_SIZE (SZ_1M - SZ_128K - SZ_64K)
#else
#define HTC_8974_PERSISTENT_RAM_SIZE (SZ_1M - SZ_128K)
#endif
#define HTC_8974_RAM_CONSOLE_SIZE    HTC_8974_PERSISTENT_RAM_SIZE

static struct persistent_ram_descriptor htc_8974_persistent_ram_descs[] = {
	{
		.name = "ram_console",
		.size = HTC_8974_RAM_CONSOLE_SIZE,
        },
};

static struct persistent_ram htc_8974_persistent_ram = {
	.start     = HTC_8974_PERSISTENT_RAM_PHYS,
	.size      = HTC_8974_PERSISTENT_RAM_SIZE,
	.num_descs = ARRAY_SIZE(htc_8974_persistent_ram_descs),
	.descs     = htc_8974_persistent_ram_descs,
};

#ifdef CONFIG_HTC_BUILD_EDIAG
#define MSM_HTC_PMEM_EDIAG_BASE 0x05BD0000
#define MSM_HTC_PMEM_EDIAG_SIZE SZ_64K
#define MSM_HTC_PMEM_EDIAG1_BASE MSM_HTC_PMEM_EDIAG_BASE
#define MSM_HTC_PMEM_EDIAG1_SIZE MSM_HTC_PMEM_EDIAG_SIZE
#define MSM_HTC_PMEM_EDIAG2_BASE MSM_HTC_PMEM_EDIAG_BASE
#define MSM_HTC_PMEM_EDIAG2_SIZE MSM_HTC_PMEM_EDIAG_SIZE
#define MSM_HTC_PMEM_EDIAG3_BASE MSM_HTC_PMEM_EDIAG_BASE
#define MSM_HTC_PMEM_EDIAG3_SIZE MSM_HTC_PMEM_EDIAG_SIZE

static struct android_pmem_platform_data android_pmem_ediag_pdata = {
	.name = "pmem_ediag",
	.start = MSM_HTC_PMEM_EDIAG_BASE,
	.size = MSM_HTC_PMEM_EDIAG_SIZE,
	.no_allocator = 0,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_ediag1_pdata = {
	.name = "pmem_ediag1",
	.start = MSM_HTC_PMEM_EDIAG1_BASE,
	.size = MSM_HTC_PMEM_EDIAG1_SIZE,
	.no_allocator = 0,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_ediag2_pdata = {
	.name = "pmem_ediag2",
	.start = MSM_HTC_PMEM_EDIAG2_BASE,
	.size = MSM_HTC_PMEM_EDIAG2_SIZE,
	.no_allocator = 0,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_ediag3_pdata = {
	.name = "pmem_ediag3",
	.start = MSM_HTC_PMEM_EDIAG3_BASE,
	.size = MSM_HTC_PMEM_EDIAG3_SIZE,
	.no_allocator = 0,
	.cached = 0,
};

static struct platform_device android_pmem_ediag_device = {
	.name = "ediag_pmem",	.id = 1,
	.dev = { .platform_data = &android_pmem_ediag_pdata },
};

static struct platform_device android_pmem_ediag1_device = {
	.name = "ediag_pmem",	.id = 2,
	.dev = { .platform_data = &android_pmem_ediag1_pdata },
};

static struct platform_device android_pmem_ediag2_device = {
	.name = "ediag_pmem",	.id = 3,
	.dev = { .platform_data = &android_pmem_ediag2_pdata },
};

static struct platform_device android_pmem_ediag3_device = {
	.name = "ediag_pmem",	.id = 4,
	.dev = { .platform_data = &android_pmem_ediag3_pdata },
};
#endif

extern int __init htc_8974_dsi_panel_power_register(void);

#define HTC_8974_USB1_HS_ID_GPIO 102

static uint32_t htc_8974_usb_id_gpio_input[] = {
	GPIO_CFG(HTC_8974_USB1_HS_ID_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
static uint32_t htc_8974_usb_id_gpio_output[] = {
	GPIO_CFG(HTC_8974_USB1_HS_ID_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static void htc_8974_config_usb_id_gpios(bool output)
{
	if (output) {
		if (gpio_tlmm_config(htc_8974_usb_id_gpio_output[0], GPIO_CFG_ENABLE)) {
			printk(KERN_ERR "[CABLE] fail to config usb id, output = %d\n",output);
			return;
		}
		gpio_set_value(HTC_8974_USB1_HS_ID_GPIO, 1);
		pr_info("[CABLE] %s: %d output high\n",  __func__, HTC_8974_USB1_HS_ID_GPIO);
	} else {
		if (gpio_tlmm_config(htc_8974_usb_id_gpio_input[0], GPIO_CFG_ENABLE)) {
			printk(KERN_ERR "[CABLE] fail to config usb id, output = %d\n",output);
			return;
		}
	}
}

static int64_t htc_8974_get_usbid_adc(void)
{
#if 0
	struct qpnp_vadc_result result;
	int err = 0, adc = 0;
	err = qpnp_vadc_read(LR_MUX10_USB_ID_LV, &result);
	if (err < 0) {
		pr_info("[CABLE] %s: get adc fail, err %d\n", __func__, err);
		return err;
	}
	adc = result.physical;
	adc /= 1000;
	pr_info("[CABLE] chan=%d, adc_code=%d, measurement=%lld, \
			physical=%lld translate voltage %d\n", result.chan, result.adc_code,
			result.measurement, result.physical, adc);
	return adc;
#endif
	return htc_qpnp_adc_get_usbid_adc();
}


static void htc_8974_usb_dpdn_switch(int path)
{
	if (mhl_usb_sw_gpio < 0) {
		pr_info("%s: no define mhl_usb_sw_gpio\n", __func__);
		return;
	}

	pr_debug("%s path:%d\n", __func__, path);
	switch (path) {
	case PATH_USB:
		gpio_set_value(mhl_usb_sw_gpio, 0);
		break;
	case PATH_MHL:
		gpio_set_value(mhl_usb_sw_gpio, 1);
		break;
	}
}

#if defined(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII9234) && defined(CONFIG_HTC_MHL_DETECTION)
static T_MHL_PLATFORM_DATA mhl_sii9234_device_data = {
    .mhl_usb_switch         = htc_8974_usb_dpdn_switch,
};

static struct platform_device mhl_ctrl_device = {
    .name                   = "sii9234_mhl_ctrl",
    .id                     = -1,
    .dev.platform_data      = &mhl_sii9234_device_data,
};

int __init htc_8974_mhl_ctrl_register(void)
{
	platform_device_register(&mhl_ctrl_device);
	return 0;
}
#endif

static struct cable_detect_platform_data cable_detect_pdata = {
	.detect_type            = CABLE_TYPE_PMIC_ADC,
	.usb_id_pin_type        = CABLE_TYPE_APP_GPIO,
	.usb_id_pin_gpio        = HTC_8974_USB1_HS_ID_GPIO,
	.get_adc_cb             = htc_8974_get_usbid_adc,
	.config_usb_id_gpios    = htc_8974_config_usb_id_gpios,
	.usb_dpdn_switch        = htc_8974_usb_dpdn_switch,
#ifdef CONFIG_HTC_BATT_8960
	.is_pwr_src_plugged_in	= pm8941_is_pwr_src_plugged_in,
#endif
#if defined(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII9234) && defined(CONFIG_HTC_MHL_DETECTION)
	.mhl_1v2_power          = mhl_sii9234_1v2_power,
	.mhl_reset_gpio         = -1,
#endif
	.vbus_debounce_retry = 1,
};

static struct platform_device cable_detect_device = {
	.name   = "cable_detect",
	.id     = -1,
	.dev    = {
		.platform_data = &cable_detect_pdata,
	},
};

void htc_8974_cable_detect_register(void)
{
#ifdef CONFIG_HTC_MHL_DETECTION
	struct device_node *np;

	np = of_find_node_by_path("/soc/mhl_usb_detect");
	if (np) {
		mhl_usb_sw_gpio = of_get_named_gpio(np, "mhl_usb_sw_gpio", 0);
		if (!gpio_is_valid(mhl_usb_sw_gpio)) {
			pr_info("%s: No define mhl_usb_sw_gpio\n", __func__);
		} else {
			pr_info("%s: mhl_usb_sw_gpio =%d\n", __func__, mhl_usb_sw_gpio);
		}
#ifdef CONFIG_FB_MSM_MDSS_HDMI_MHL_SII9234
		{
			int mhl_usb_reset_gpio;
			mhl_usb_reset_gpio = of_get_named_gpio(np, "mhl_usb_reset_gpio", 0);
			if (!gpio_is_valid(mhl_usb_reset_gpio)) {
				pr_info("%s: No define mhl_usb_reset_gpio\n", __func__);
			} else {
				cable_detect_pdata.mhl_reset_gpio = mhl_usb_reset_gpio;
				pr_info("%s: mhl_usb_reset_gpio =%d\n", __func__, mhl_usb_reset_gpio);
			}

			cable_detect_pdata.mhl_disable_irq = sii9234_disableIRQ;
			cable_detect_pdata.switch_to_d3 = D2ToD3;
			cable_detect_pdata.mhl_wakeup = sii9234_mhl_device_wakeup;
			cable_detect_pdata.mhl_detect_register_notifier = mhl_detect_register_notifier;
		}
#endif
#ifdef CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8240_SII8558
		{
			int mhl_usb_reset_gpio;
			mhl_usb_reset_gpio = of_get_named_gpio(np, "mhl_usb_reset_gpio", 0);
			if (!gpio_is_valid(mhl_usb_reset_gpio)) {
				pr_info("%s: No define mhl_usb_reset_gpio\n", __func__);
			} else {
				cable_detect_pdata.mhl_reset_gpio = mhl_usb_reset_gpio;
				pr_info("%s: mhl_usb_reset_gpio =%d\n", __func__, mhl_usb_reset_gpio);
			}

			cable_detect_pdata.switch_to_d3 = si_d2_to_d3;
			cable_detect_pdata.mhl_wakeup = si_wakeup_mhl;
			cable_detect_pdata.mhl_detect_register_notifier = mhl_detect_register_notifier;
		}
#endif
	} else {
		pr_info("%s no mhl_usb_detect node\n", __func__);
	}
#endif
	platform_device_register(&cable_detect_device);
}
#ifdef CONFIG_HTC_POWER_DEBUG
static struct platform_device cpu_usage_stats_device = {
	.name = "cpu_usage_stats",
	.id = -1,
};

int __init htc_cpu_usage_register(void)
{
	platform_device_register(&cpu_usage_stats_device);
	return 0;
}
#endif

static int __maybe_unused m8wl_usb_product_id_match_array[] = {
		0x0ff8, 0x0e65, 
		0x0fa4, 0x0eab, 
		0x0fa5, 0x0eac, 
		0x0f91, 0x0ec3, 
		0x0f64, 0x07ca, 
		0x0f63, 0x07cb, 
		0x0f29, 0x07c8, 
		0x0f2a, 0x07c9, 
		0x0f9a, 0x0eae, 
		0x0f99, 0x0ead, 
		-1,
};

static int __maybe_unused m8wl_usb_product_id_rndis[] = {
	0x0762, 
	0x0768, 
	0x0763, 
	0x0769, 
	0x07be, 
	0x07c2, 
	0x07bf, 
	0x07c3, 
};
static int __maybe_unused m8wl_usb_product_id_match(int product_id, int intrsharing)
{
	int *pid_array = m8wl_usb_product_id_match_array;
	int *rndis_array = m8wl_usb_product_id_rndis;
	int category = 0;

	if (!pid_array)
		return product_id;

	
	if (board_mfg_mode())
		return product_id;

	while (pid_array[0] >= 0) {
		if (product_id == pid_array[0])
			return pid_array[1];
		pid_array += 2;
	}
	printk("%s(%d):product_id=%d, intrsharing=%d\n", __func__, __LINE__, product_id, intrsharing);

	switch (product_id) {
		case 0x0f8c: 
			category = 0;
			break;
		case 0x0f8d: 
			category = 1;
			break;
		case 0x0f5f: 
			category = 2;
			break;
		case 0x0f60: 
			category = 3;
			break;
		default:
			category = -1;
			break;
	}
	if (category != -1) {
		if (intrsharing)
			return rndis_array[category * 2];
		else
			return rndis_array[category * 2 + 1];
	}
	return product_id;
}


static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id      = 0x0bb4,
	.product_id     = 0x060e, 
	.product_name		= "Android Phone",
	.manufacturer_name	= "HTC",
	.serial_number = "123456789012",
	.usb_core_id = 0,
	.usb_rmnet_interface = "smd,bam",
	.usb_diag_interface = "diag",
	.fserial_init_string = "smd:modem,tty,tty:autobot,tty:serial,tty:autobot,tty:acm",
#ifdef CONFIG_MACH_DUMMY
	.match = m8wl_usb_product_id_match,
#endif
	.nluns = 1,
	.cdrom_lun = 0x1,
	.vzw_unmount_cdrom = 0,
};

static struct platform_device android_usb_device = {
	.name   = "android_usb",
	.id     = -1,
	.dev    = {
		.platform_data = &android_usb_pdata,
	},
};

static void htc_8974_add_usb_devices(void)
{
	char *mid;
	android_usb_pdata.serial_number = board_serialno();

	mid = board_mid();

	if (board_mfg_mode() == 0) {
#ifdef CONFIG_MACH_DUMMY
		android_usb_pdata.nluns = 2;
		android_usb_pdata.cdrom_lun = 0x2;
#elif defined(CONFIG_MACH_DUMMY)
		android_usb_pdata.nluns = 2;
		android_usb_pdata.cdrom_lun = 0x2;
#else
		android_usb_pdata.nluns = 1;
		android_usb_pdata.cdrom_lun = 0x1;
#endif

	}
#ifdef CONFIG_MACH_M8
	android_usb_pdata.product_id	= 0x061A;
#elif defined(CONFIG_MACH_DUMMY)
	android_usb_pdata.product_id	= 0x0616;
	android_usb_pdata.vzw_unmount_cdrom = 1;
#elif defined(CONFIG_MACH_DUMMY)
	android_usb_pdata.product_id	= 0x061A;
#elif defined(CONFIG_MACH_DUMMY)
	android_usb_pdata.product_id	= 0x0623;
#elif defined(CONFIG_MACH_DUMMY)
	android_usb_pdata.product_id	= 0x063B;
#elif defined(CONFIG_MACH_DUMMY)
	android_usb_pdata.product_id	= 0x0643;
#elif defined(CONFIG_MACH_DUMMY)
	android_usb_pdata.product_id	= 0x0634;
#elif defined(CONFIG_MACH_DUMMY)
	android_usb_pdata.product_id	= 0x0642;
#elif defined(CONFIG_MACH_DUMMY)
	android_usb_pdata.product_id	= 0x0642;
#else
	
#endif

	if (strcmp("0PFH20000", mid)==0)
		android_usb_pdata.product_id	= 0x064B;
	else if (strcmp("0P6B90000", mid)==0 || strcmp("0P6B91000", mid)==0)
		android_usb_pdata.product_id	= 0x064A;

	platform_device_register(&android_usb_device);
}
#if (defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY))
static ssize_t syn_vkeys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
	__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":241:2007:158:165"
	":" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":847:2007:158:165"
	"\n");
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

static void syn_init_vkeys_8974(void)
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
#endif
static struct memtype_reserve htc_8974_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static int htc_8974_paddr_to_memtype(phys_addr_t paddr)
{
	return MEMTYPE_EBI1;
}

static struct reserve_info htc_8974_reserve_info __initdata = {
	.memtype_reserve_table = htc_8974_reserve_table,
	.paddr_to_memtype = htc_8974_paddr_to_memtype,
};

void __init htc_8974_reserve(void)
{
	reserve_info = &htc_8974_reserve_info;
	of_scan_flat_dt(dt_scan_for_memory_reserve, htc_8974_reserve_table);
	msm_reserve();
}

static void __init htc_8974_early_memory(void)
{
	reserve_info = &htc_8974_reserve_info;
	of_scan_flat_dt(dt_scan_for_memory_hole, htc_8974_reserve_table);
}

#if defined(CONFIG_HTC_BATT_8960)
#ifdef CONFIG_HTC_PNPMGR
extern int pnpmgr_battery_charging_enabled(int charging_enabled);
#endif 
static int critical_alarm_voltage_mv[] = {3000, 3200, 3400};

static struct htc_battery_platform_data htc_battery_pdev_data = {
	.guage_driver = 0,
#if defined(CONFIG_MACH_DUMMY)
	.chg_limit_active_mask = HTC_BATT_CHG_LIMIT_BIT_TALK |
								HTC_BATT_CHG_LIMIT_BIT_NAVI |
								HTC_BATT_CHG_LIMIT_BIT_KDDI |
								HTC_BATT_CHG_LIMIT_BIT_THRML,
#else
	.chg_limit_active_mask = HTC_BATT_CHG_LIMIT_BIT_TALK |
								HTC_BATT_CHG_LIMIT_BIT_NAVI |
								HTC_BATT_CHG_LIMIT_BIT_THRML,
#endif
#ifdef CONFIG_DUTY_CYCLE_LIMIT
	.chg_limit_timer_sub_mask = HTC_BATT_CHG_LIMIT_BIT_THRML,
#endif
	.critical_low_voltage_mv = 3200,
	.critical_alarm_vol_ptr = critical_alarm_voltage_mv,
	.critical_alarm_vol_cols = sizeof(critical_alarm_voltage_mv) / sizeof(int),
	.overload_vol_thr_mv = 4000,
	.overload_curr_thr_ma = 0,
	.smooth_chg_full_delay_min = 3,
	.decreased_batt_level_check = 1,
	.force_shutdown_batt_vol = 3000,
#if defined(CONFIG_MACH_DUMMY)
	.usb_temp_monitor_enable = 1,
	.usb_temp_overheat_increase_threshold = 25, 
	.normal_usb_temp_threshold = 450, 
	.usb_temp_overheat_threshold = 650,
#endif
	
	.icharger.name = "pm8941",
	.icharger.get_charging_source = pm8941_get_charging_source,
	.icharger.get_charging_enabled = pm8941_get_charging_enabled,
	.icharger.set_charger_enable = pm8941_charger_enable,
	.icharger.set_pwrsrc_enable = pm8941_pwrsrc_enable,
	.icharger.set_pwrsrc_and_charger_enable =
						pm8941_set_pwrsrc_and_charger_enable,
	.icharger.set_limit_charge_enable = pm8941_limit_charge_enable,
	.icharger.set_limit_input_current = pm8941_limit_input_current,
	.icharger.set_chg_iusbmax = pm8941_set_chg_iusbmax,
	.icharger.set_chg_curr_settled = pm8941_set_chg_curr_settled,
	.icharger.set_chg_vin_min = pm8941_set_chg_vin_min,
	.icharger.is_ovp = pm8941_is_charger_ovp,
	.icharger.is_batt_temp_fault_disable_chg =
						pm8941_is_batt_temp_fault_disable_chg,
	.icharger.is_under_rating = pm8921_is_pwrsrc_under_rating,
	.icharger.charger_change_notifier_register =
						cable_detect_register_notifier,
	.icharger.dump_all = pm8941_dump_all,
	.icharger.is_safty_timer_timeout = pm8941_is_chg_safety_timer_timeout,
	.icharger.get_attr_text = pm8941_charger_get_attr_text,
	.icharger.max_input_current = pm8941_set_hsml_target_ma,
	.icharger.is_battery_full_eoc_stop = pm8941_is_batt_full_eoc_stop,
	.icharger.get_charge_type = pm8941_get_charge_type,
	.icharger.get_chg_usb_iusbmax = pm8941_get_chg_usb_iusbmax,
	.icharger.get_chg_curr_settled = pm8941_get_chg_curr_settled,
	.icharger.get_chg_vinmin = pm8941_get_chg_vinmin,
	.icharger.get_input_voltage_regulation =
						pm8941_get_input_voltage_regulation,
	.icharger.store_battery_charger_data = pm8941_store_battery_charger_data_emmc,
	.icharger.set_ftm_charge_enable_type = pm8941_set_ftm_charge_enable_type,
	
	.igauge.name = "pm8941",
	.igauge.get_battery_voltage = pm8941_get_batt_voltage,
	.igauge.get_battery_current = pm8941_bms_get_batt_current,
	.igauge.get_battery_temperature = pm8941_get_batt_temperature,
	.igauge.get_battery_id = pm8941_get_batt_id,
	.igauge.get_battery_id_mv = pm8941_get_batt_id_mv,
	.igauge.get_battery_soc = pm8941_bms_get_batt_soc,
	.igauge.get_battery_cc = pm8941_bms_get_batt_cc,
#if defined(CONFIG_MACH_DUMMY)
	.igauge.get_usb_temperature = pm8941_get_usb_temperature,
	.igauge.usb_overheat_otg_mode_check = pm8941_usb_overheat_otg_mode_check,
#endif
	.igauge.store_battery_gauge_data = pm8941_bms_store_battery_gauge_data_emmc,
	.igauge.store_battery_ui_soc = pm8941_bms_store_battery_ui_soc,
	.igauge.get_battery_ui_soc = pm8941_bms_get_battery_ui_soc,
	.igauge.enter_qb_mode = pm8941_bms_enter_qb_mode,
	.igauge.exit_qb_mode = pm8941_bms_exit_qb_mode,
	.igauge.qb_mode_pwr_consumption_check = pm8941_qb_mode_pwr_consumption_check,
	.igauge.is_battery_temp_fault = pm8941_is_batt_temperature_fault,
	.igauge.is_battery_full = pm8941_is_batt_full,
	.igauge.get_attr_text = pm8941_gauge_get_attr_text,
	.igauge.set_lower_voltage_alarm_threshold =
						pm8941_batt_lower_alarm_threshold_set,
	.igauge.check_soc_for_sw_ocv = pm8941_check_soc_for_sw_ocv,
	
#ifdef CONFIG_HTC_PNPMGR
	.notify_pnpmgr_charging_enabled = pnpmgr_battery_charging_enabled,
#endif 
};
static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id = -1,
	.dev    = {
		.platform_data = &htc_battery_pdev_data,
	},
};

static void msm8974_add_batt_devices(void)
{
	platform_device_register(&htc_battery_pdev);
}

static struct platform_device htc_battery_cell_pdev = {
	.name = "htc_battery_cell",
	.id = -1,
};

int __init htc_batt_cell_register(void)
{
	platform_device_register(&htc_battery_cell_pdev);
	return 0;
}
#endif 

void __init htc_8974_add_drivers(void)
{
	msm_smem_init();
	msm_init_modem_notifier_list();
	msm_smd_init();
	msm_rpm_driver_init();
	msm_pm_sleep_status_init();
	rpm_regulator_smd_driver_init();
	msm_spm_device_init();
	krait_power_init();
	msm_clock_init(&msm8974_clock_init_data);
	tsens_tm_init_driver();
	msm_thermal_device_init();
#if defined(CONFIG_HTC_BATT_8960)
	htc_batt_cell_register();
	msm8974_add_batt_devices();
#endif 
#if (defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_DUMMY))
	syn_init_vkeys_8974();
#endif
	htc_8974_cable_detect_register();
	
	if (board_mfg_mode() != 6 && board_mfg_mode() != 7)
		htc_8974_add_usb_devices();
	else
		pr_info("[USB] %s: Don't load USB since board_mfg_mode=%d\n", __func__, board_mfg_mode());

	htc_8974_dsi_panel_power_register();
#if defined(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII9234) && defined(CONFIG_HTC_MHL_DETECTION)
	htc_8974_mhl_ctrl_register();
#endif
#ifdef CONFIG_HTC_POWER_DEBUG
	htc_cpu_usage_register();
#endif
}

static struct of_dev_auxdata htc_8974_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("qcom,hsusb-otg", 0xF9A55000, \
			"msm_otg", NULL),
	OF_DEV_AUXDATA("qcom,ehci-host", 0xF9A55000, \
			"msm_ehci_host", NULL),
	OF_DEV_AUXDATA("qcom,dwc-usb3-msm", 0xF9200000, \
			"msm_dwc3", NULL),
	OF_DEV_AUXDATA("qcom,usb-bam-msm", 0xF9304000, \
			"usb_bam", NULL),
	OF_DEV_AUXDATA("qcom,spi-qup-v2", 0xF9924000, \
			"spi_qsd.1", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF9824000, \
			"msm_sdcc.1", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF98A4000, \
			"msm_sdcc.2", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF9864000, \
			"msm_sdcc.3", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF98E4000, \
			"msm_sdcc.4", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF9824900, \
			"msm_sdcc.1", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF98A4900, \
			"msm_sdcc.2", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF9864900, \
			"msm_sdcc.3", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF98E4900, \
			"msm_sdcc.4", NULL),
	OF_DEV_AUXDATA("qcom,msm-rng", 0xF9BFF000, \
			"msm_rng", NULL),
	OF_DEV_AUXDATA("qcom,qseecom", 0xFE806000, \
			"qseecom", NULL),
	OF_DEV_AUXDATA("qcom,mdss_mdp", 0xFD900000, "mdp.0", NULL),
	OF_DEV_AUXDATA("qcom,msm-tsens", 0xFC4A8000, \
			"msm-tsens", NULL),
	OF_DEV_AUXDATA("qcom,qcedev", 0xFD440000, \
			"qcedev.0", NULL),
	OF_DEV_AUXDATA("qcom,qcrypto", 0xFD440000, \
			"qcrypto.0", NULL),
	OF_DEV_AUXDATA("qcom,hsic-host", 0xF9A00000, \
			"msm_hsic_host", NULL),
	{}
};

static void __init htc_8974_map_io(void)
{
	msm_map_8974_io();
}

void __init htc_8974_init_early(void)
{
	
	persistent_ram_early_init(&htc_8974_persistent_ram);

#ifdef CONFIG_HTC_DEBUG_FOOTPRINT
	mnemosyne_early_init((unsigned int)HTC_DEBUG_FOOTPRINT_PHYS, (unsigned int)HTC_DEBUG_FOOTPRINT_BASE);
#endif
}

void __init htc_8974_init(void)
{
	struct of_dev_auxdata *adata = htc_8974_auxdata_lookup;

	if (socinfo_init() < 0)
		pr_err("%s: socinfo_init() failed\n", __func__);

	pr_info("%s: pid=%d, pcbid=%d, socver=0x%x\n", __func__
		, of_machine_pid(), of_machine_pcbid(), of_machine_socver());

	msm_htc_8974_init_gpiomux();
	regulator_has_full_constraints();
	board_dt_populate(adata);
	htc_8974_add_drivers();
#ifdef CONFIG_HTC_BUILD_EDIAG
	platform_device_register(&android_pmem_ediag_device);
	platform_device_register(&android_pmem_ediag1_device);
	platform_device_register(&android_pmem_ediag2_device);
	platform_device_register(&android_pmem_ediag3_device);
#endif
#ifdef CONFIG_BT
	bt_export_bd_address();
#endif
#ifdef CONFIG_PERFLOCK
	platform_device_register(&msm8974_device_perf_lock);
#endif
#ifdef CONFIG_HTC_POWER_DEBUG
	htc_monitor_init();
#endif
}

void __init htc_8974_init_very_early(void)
{
	htc_8974_early_memory();
}

static const char *htc_8974_dt_match[] __initconst = {
	"htc,msm8974",
	NULL
};


#define MACH_NAME "Qualcomm MSM8974 " CONFIG_PROJECT_NAME

DT_MACHINE_START(htc_8974_DT, MACH_NAME)
	.map_io = htc_8974_map_io,
	.init_early = htc_8974_init_early,
	.init_irq = msm_dt_init_irq,
	.init_machine = htc_8974_init,
	.handle_irq = gic_handle_irq,
	.timer = &msm_dt_timer,
	.dt_compat = htc_8974_dt_match,
	.reserve = htc_8974_reserve,
	.init_very_early = htc_8974_init_very_early,
	.restart = msm_restart,
	.smp = &msm8974_smp_ops,
MACHINE_END
