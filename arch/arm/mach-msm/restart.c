/* Copyright (c) 2010-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/pmic8901.h>
#include <linux/mfd/pm8xxx/misc.h>
#include <linux/gpio.h>
#include <linux/console.h>
#include <linux/qpnp/power-on.h>

#include <asm/mach-types.h>
#include <asm/cacheflush.h>

#include <mach/msm_iomap.h>
#include <mach/restart.h>
#include <mach/socinfo.h>
#include <mach/irqs.h>
#include <mach/scm.h>
#include <mach/htc_restart_handler.h>
#include "msm_watchdog.h"
#include "timer.h"
#include "wdog_debug.h"

#ifdef CONFIG_HTC_FEATURES_FINAL_EFS_SYNC
#include <mach/devices_cmdline.h>
#include "smd_private.h"
#include "htc_smem.h"

#include <linux/fs.h>
#include <asm/uaccess.h>

#define HTC_SMEM_PARAM_BASE_ADDR 0xFBF0000

#define HW_RST_ADDR 0x270
#define MISC_DEV_PREFIX "/dev/block/mmcblk0p"

extern int get_partition_num_by_name(char *name);

#endif

#if defined(CONFIG_MACH_EYE_UL)
#define PN547_I2C_POWEROFF_SEQUENCE_FOR_EYE
#elif defined(CONFIG_MACH_EYE_WHL)
#define PN547_I2C_POWEROFF_SEQUENCE_FOR_EYE
#elif defined(CONFIG_MACH_EYE_WL)
#define PN547_I2C_POWEROFF_SEQUENCE_FOR_EYE
#elif defined(CONFIG_MACH_MEC_TL)
#define PN547_I2C_POWEROFF_SEQUENCE_FOR_MEC
#elif defined(CONFIG_MACH_MEC_WHL)
#define PN547_I2C_POWEROFF_SEQUENCE_FOR_MEC
#elif defined(CONFIG_MACH_MEC_UL)
#define PN547_I2C_POWEROFF_SEQUENCE_FOR_MEC
#elif defined(CONFIG_MACH_MEC_DUG)
#define PN547_I2C_POWEROFF_SEQUENCE_FOR_MEC
#elif defined(CONFIG_MACH_MEC_DWG)
#define PN547_I2C_POWEROFF_SEQUENCE_FOR_MEC
#elif defined(CONFIG_MACH_DUMMY)
#define PN547_I2C_POWEROFF_SEQUENCE_FOR_B2
#else
#endif


#if defined(PN547_I2C_POWEROFF_SEQUENCE_FOR_EYE)
#define SR_I2C_SCL     11
#define SR_I2C_SDA     10
extern void force_disable_PM8941_VREG_ID_L22(void);
#endif
#if defined(PN547_I2C_POWEROFF_SEQUENCE_FOR_MEC)
#define SR_I2C_SCL     11
#define SR_I2C_SDA     10
#define TP_RST         23
#define SRIO_1V8_EN    95
extern void force_disable_PMICGPIO34(void);
#endif
#if defined(PN547_I2C_POWEROFF_SEQUENCE_FOR_B2)
#define SR_I2C_SCL     11
#define SR_I2C_SDA     10
#define TP_RST         23
extern void force_disable_PMICGPIO34(void);
extern void force_disable_PMICLVS1(void);
#endif

#define WDT0_RST	0x38
#define WDT0_EN		0x40
#define WDT0_BARK_TIME	0x4C
#define WDT0_BITE_TIME	0x5C

#define PSHOLD_CTL_SU (MSM_TLMM_BASE + 0x820)

#define RESTART_REASON_ADDR 0x65C
#define DLOAD_MODE_ADDR     0x0
#define EMERGENCY_DLOAD_MODE_ADDR    0xFE0
#define EMERGENCY_DLOAD_MAGIC1    0x322A4F99
#define EMERGENCY_DLOAD_MAGIC2    0xC67E4350
#define EMERGENCY_DLOAD_MAGIC3    0x77777777

#define SCM_IO_DISABLE_PMIC_ARBITER	1

#ifdef CONFIG_MSM_RESTART_V2
#define use_restart_v2()	1
#else
#define use_restart_v2()	0
#endif

static int restart_mode;

int pmic_reset_irq;
static void __iomem *msm_tmr0_base;

static int in_panic;

static int panic_prep_restart(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	in_panic = 1;
	return NOTIFY_DONE;
}

static struct notifier_block panic_blk = {
	.notifier_call	= panic_prep_restart,
};

#ifdef CONFIG_MSM_DLOAD_MODE
static void *dload_mode_addr;
static bool dload_mode_enabled;
static void *emergency_dload_mode_addr;

static int dload_set(const char *val, struct kernel_param *kp);
static int download_mode = 1;
module_param_call(download_mode, dload_set, param_get_int,
			&download_mode, 0644);

static void set_dload_mode(int on)
{
	if (dload_mode_addr) {
		__raw_writel(on ? 0xE47B337D : 0, dload_mode_addr);
		__raw_writel(on ? 0xCE14091A : 0,
		       dload_mode_addr + sizeof(unsigned int));
		mb();
		dload_mode_enabled = on;
	}
}

static bool get_dload_mode(void)
{
	return dload_mode_enabled;
}

static void enable_emergency_dload_mode(void)
{
	if (emergency_dload_mode_addr) {
		__raw_writel(EMERGENCY_DLOAD_MAGIC1,
				emergency_dload_mode_addr);
		__raw_writel(EMERGENCY_DLOAD_MAGIC2,
				emergency_dload_mode_addr +
				sizeof(unsigned int));
		__raw_writel(EMERGENCY_DLOAD_MAGIC3,
				emergency_dload_mode_addr +
				(2 * sizeof(unsigned int)));

		qpnp_pon_wd_config(0);
		mb();
	}
}

static int dload_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = download_mode;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	
	if (download_mode >> 1) {
		download_mode = old_val;
		return -EINVAL;
	}

	set_dload_mode(download_mode);

	return 0;
}
#else
#define set_dload_mode(x) do {} while (0)

static void enable_emergency_dload_mode(void)
{
	printk(KERN_ERR "dload mode is not enabled on target\n");
}
#endif

void msm_set_restart_mode(int mode)
{
	restart_mode = mode;
}
EXPORT_SYMBOL(msm_set_restart_mode);

static void msm_flush_console(void)
{
	unsigned long flags;

	printk("\n");
	printk(KERN_EMERG "[K] Restarting %s\n", linux_banner);
	if (console_trylock()) {
		console_unlock();
		return;
	}

	mdelay(50);

	local_irq_save(flags);

	if (console_trylock())
		printk(KERN_EMERG "[K] restart: Console was locked! Busting\n");
	else
		printk(KERN_EMERG "[K] restart: Console was locked!\n");
	console_unlock();

	local_irq_restore(flags);
}

#ifdef CONFIG_HTC_FEATURES_FINAL_EFS_SYNC
static void set_ril_fatal(unsigned long oem_code)
{
	htc_smem_type *htc_smem_ram_addr =
		(htc_smem_type *)ioremap(HTC_SMEM_PARAM_BASE_ADDR, sizeof(htc_smem_type));

	oem_code |= 0x6F656D00;
	memcpy(&htc_smem_ram_addr->htc_ril_fatal, &oem_code, 4);
}

static void set_modem_efs_sync(void)
{
	smsm_change_state(SMSM_APPS_STATE, SMSM_APPS_REBOOT, SMSM_APPS_REBOOT);
	printk(KERN_INFO "[K] %s: wait for modem efs_sync\n", __func__);
}

static int check_modem_efs_sync(void)
{
	return (smsm_get_state(SMSM_MODEM_STATE) & SMSM_SYSTEM_PWRDWN_USR);
}

static void check_modem_efs_sync_timeout(unsigned timeout)
{
	while (timeout > 0 && !check_modem_efs_sync()) {
		msleep(1000);
		timeout--;
	}
	if (timeout <= 0)
		pr_notice("%s: modem efs_sync timeout.\n", __func__);
	else
		pr_info("%s: modem efs_sync done.\n", __func__);
}

static ssize_t kernel_write(struct file *file, const char *buf, size_t count, loff_t pos)
{
	mm_segment_t old_fs;
	ssize_t res;

	old_fs = get_fs();
	set_fs(get_ds());
	res = vfs_write(file, (const char __user *)buf, count, &pos);
	set_fs(old_fs);
	return res;
}

static int set_reset_reason(unsigned int reason)
{
	char filename[32] = "";
	unsigned int hw_reason = reason;
	struct file *filp = NULL;
	ssize_t nread;
	int pnum = get_partition_num_by_name("misc");

	if (pnum < 0) {
		printk(KERN_ERR "Unknonwn partition number for misc partition\n");
		return -1;
	}
	sprintf(filename, "%s%d", MISC_DEV_PREFIX, pnum);

	filp = filp_open(filename, O_RDWR, 0);
	if (IS_ERR(filp) || (!filp)) {
		printk(KERN_ERR "Unable to open misc partition file: %s\n", filename);
		return PTR_ERR(filp);
	}
	filp->f_pos = HW_RST_ADDR;
	nread = kernel_write(filp, (char *)&hw_reason, sizeof(unsigned int), filp->f_pos);
	printk(KERN_ERR "[REBOOT] write: %X (%d), %s\n", hw_reason, nread, filename);

	if (filp)
		filp_close(filp, NULL);

	return 0;
}

static int notify_efs_sync_call
	(struct notifier_block *this, unsigned long code, void *_cmd)
{
	unsigned long oem_code = 0;

	switch (code) {
	case SYS_RESTART:
		if (_cmd && !strncmp(_cmd, "oem-", 4)) {
			oem_code = simple_strtoul(_cmd + 4, 0, 16) & 0xff;
			set_reset_reason(RESTART_REASON_OEM_BASE | oem_code);
			if (oem_code == 0x98 || oem_code == 0x99)
				set_ril_fatal(oem_code);
		}
	case SYS_POWER_OFF:
		
		pr_notice("%s: board_mfg_mode=%d\n", __func__, board_mfg_mode());
		if ((board_mfg_mode() <= MFG_MODE_MINI) && (board_mfg_mode() != MFG_MODE_OFFMODE_CHARGING)) {
			set_modem_efs_sync();
			check_modem_efs_sync_timeout(10);
		}

		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block notify_efs_sync_notifier = {
	.notifier_call = notify_efs_sync_call,
};

#endif

static bool scm_pmic_arbiter_disable_supported;
static void halt_spmi_pmic_arbiter(void)
{
	if (scm_pmic_arbiter_disable_supported) {
		pr_crit("Calling SCM to disable SPMI PMIC arbiter\n");
		scm_call_atomic1(SCM_SVC_PWR, SCM_IO_DISABLE_PMIC_ARBITER, 0);
	}
}

static void __msm_power_off(int lower_pshold)
{
	printk(KERN_CRIT "[K] Powering off the SoC\n");

#if defined(PN547_I2C_POWEROFF_SEQUENCE_FOR_EYE)
	
	
	gpio_tlmm_config(GPIO_CFG(SR_I2C_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	gpio_set_value(SR_I2C_SCL, 0); 
	mdelay(1);

	gpio_tlmm_config(GPIO_CFG(SR_I2C_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	gpio_set_value(SR_I2C_SDA, 0); 
	mdelay(1);

	force_disable_PM8941_VREG_ID_L22(); 

#elif defined(PN547_I2C_POWEROFF_SEQUENCE_FOR_MEC)
	gpio_tlmm_config(GPIO_CFG(SR_I2C_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	gpio_set_value(SR_I2C_SCL, 0);
	msleep(1);

	gpio_tlmm_config(GPIO_CFG(SR_I2C_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	gpio_set_value(SR_I2C_SDA, 0);
	msleep(1);

	gpio_tlmm_config(GPIO_CFG(TP_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	gpio_set_value(TP_RST, 0);
	msleep(10);

	force_disable_PMICGPIO34();
	msleep(10);

	gpio_tlmm_config(GPIO_CFG(SRIO_1V8_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(SRIO_1V8_EN, 0);
	msleep(100);
#elif defined(PN547_I2C_POWEROFF_SEQUENCE_FOR_B2)
	gpio_tlmm_config(GPIO_CFG(SR_I2C_SCL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	gpio_set_value(SR_I2C_SCL, 0);
	msleep(1);

	gpio_tlmm_config(GPIO_CFG(SR_I2C_SDA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	gpio_set_value(SR_I2C_SDA, 0);
	msleep(20);

	gpio_tlmm_config(GPIO_CFG(TP_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	gpio_set_value(TP_RST, 0);
	msleep(10);

	force_disable_PMICGPIO34();
	msleep(10);

	force_disable_PMICLVS1();
	msleep(100);
#else
#endif

#ifdef CONFIG_MSM_DLOAD_MODE
	set_dload_mode(0);
#endif
	pm8xxx_reset_pwr_off(0);
	qpnp_pon_system_pwr_off(PON_POWER_OFF_SHUTDOWN);

	if (lower_pshold) {
		if (!use_restart_v2()) {
			__raw_writel(0, PSHOLD_CTL_SU);
		} else {
			halt_spmi_pmic_arbiter();
			__raw_writel(0, MSM_MPM2_PSHOLD_BASE);
		}

		mdelay(10000);
		printk(KERN_ERR "[K] Powering off has failed\n");
	}
	return;
}

static void msm_power_off(void)
{
	
	__msm_power_off(1);
}

static void cpu_power_off(void *data)
{
	int rc;

	pr_err("PMIC Initiated shutdown %s cpu=%d\n", __func__,
						smp_processor_id());
	if (smp_processor_id() == 0) {
		__msm_power_off(0);

		pet_watchdog();
		pr_err("Calling scm to disable arbiter\n");
		
		rc = scm_call_atomic1(SCM_SVC_PWR,
						SCM_IO_DISABLE_PMIC_ARBITER, 1);

		pr_err("SCM returned even when asked to busy loop rc=%d\n", rc);
		pr_err("waiting on pmic to shut msm down\n");
	}

	preempt_disable();
	while (1)
		;
}

static irqreturn_t resout_irq_handler(int irq, void *dev_id)
{
	pr_warn("%s PMIC Initiated shutdown\n", __func__);
	oops_in_progress = 1;
	smp_call_function_many(cpu_online_mask, cpu_power_off, NULL, 0);
	if (smp_processor_id() == 0)
		cpu_power_off(NULL);
	preempt_disable();
	while (1)
		;
	return IRQ_HANDLED;
}

static void msm_restart_prepare(char mode, const char *cmd)
{
#ifdef CONFIG_MSM_DLOAD_MODE

	
	set_dload_mode(0);

	
	set_dload_mode(in_panic);

	
	if (restart_mode == RESTART_DLOAD)
		set_dload_mode(1);

	
	if (!download_mode)
		set_dload_mode(0);
#endif

	pm8xxx_reset_pwr_off(1);
	qpnp_pon_system_pwr_off(PON_POWER_OFF_WARM_RESET);

	pr_info("%s: restart by command: [%s]\r\n", __func__, (cmd) ? cmd : "");

	if (in_panic) {
		
	} else if (!cmd) {
		set_restart_action(RESTART_REASON_REBOOT, NULL);
	} else if (!strncmp(cmd, "bootloader", 10)) {
		set_restart_action(RESTART_REASON_BOOTLOADER, NULL);
	} else if (!strncmp(cmd, "recovery", 8)) {
		set_restart_action(RESTART_REASON_RECOVERY, NULL);
	} else if (!strncmp(cmd, "eraseflash", 10)) {
		set_restart_action(RESTART_REASON_ERASE_FLASH, NULL);
	} else if (!strncmp(cmd, "oem-", 4)) {
		unsigned long code;
		code = simple_strtoul(cmd + 4, NULL, 16) & 0xff;
		set_restart_to_oem(code, NULL);
	} else if (!strncmp(cmd, "ftm", 3)) {
		#define FTM_BASE_FLAG 0xf0
		unsigned long code;
		code = (simple_strtoul(cmd + 3, NULL, 16) & 0xff) | FTM_BASE_FLAG;
		set_restart_to_oem(code, NULL);
	} else if (!strncmp(cmd, "edl", 3)) {
		enable_emergency_dload_mode();
	} else if (!strncmp(cmd, "force-dog-bark", 14)) {
		set_restart_to_ramdump("force-dog-bark");
	} else if (!strncmp(cmd, "force-hard", 10) ||
			(RESTART_MODE_LEGACY < mode && mode < RESTART_MODE_MAX)) {
		
		if (mode == RESTART_MODE_MODEM_USER_INVOKED)
			set_restart_action(RESTART_REASON_REBOOT, NULL);
		else if (mode == RESTART_MODE_ERASE_EFS)
			set_restart_action(RESTART_REASON_ERASE_EFS, NULL);
		else
			set_restart_action(RESTART_REASON_RAMDUMP, cmd);
	} else {
		set_restart_action(RESTART_REASON_REBOOT, NULL);
	}

	msm_flush_console();
	flush_cache_all();
	outer_flush_all();

	if (cmd && !strncmp(cmd, "force-dog-bark", 14)) {
		pr_info("%s: Force dog bark!\r\n", __func__);

#if defined(CONFIG_HTC_DEBUG_WATCHDOG)
		msm_watchdog_bark();
#endif

		mdelay(10000);

		pr_info("%s: Force Watchdog bark does not work, falling back to normal process.\r\n", __func__);
	}
}

void msm_restart(char mode, const char *cmd)
{
	printk(KERN_NOTICE "[K] Going down for restart now\n");

	printk(KERN_NOTICE "%s: Kernel command line: %s\n", __func__, hashed_command_line);

	msm_restart_prepare(mode, cmd);

	if (!use_restart_v2()) {
		__raw_writel(0, msm_tmr0_base + WDT0_EN);
		if (!(machine_is_msm8x60_fusion() ||
		      machine_is_msm8x60_fusn_ffa())) {
			mb();
			 
			__raw_writel(0, PSHOLD_CTL_SU);
			mdelay(5000);
			pr_notice("PS_HOLD didn't work, falling back to watchdog\n");
		}

		__raw_writel(1, msm_tmr0_base + WDT0_RST);
		__raw_writel(5*0x31F3, msm_tmr0_base + WDT0_BARK_TIME);
		__raw_writel(0x31F3, msm_tmr0_base + WDT0_BITE_TIME);
		__raw_writel(1, msm_tmr0_base + WDT0_EN);
	} else {
		
		msm_disable_wdog_debug();
		halt_spmi_pmic_arbiter();
#if defined(CONFIG_ARCH_DUMMY) && defined(CONFIG_HTC_DEBUG_WATCHDOG)
		msm_watchdog_reset();
#else
		__raw_writel(0, MSM_MPM2_PSHOLD_BASE);
#endif
	}

	mdelay(10000);
	printk(KERN_ERR "Restarting has failed\n");
}

static int __init msm_pmic_restart_init(void)
{
	int rc;

	if (use_restart_v2())
		return 0;

	if (pmic_reset_irq != 0) {
		rc = request_any_context_irq(pmic_reset_irq,
					resout_irq_handler, IRQF_TRIGGER_HIGH,
					"restart_from_pmic", NULL);
		if (rc < 0)
			pr_err("pmic restart irq fail rc = %d\n", rc);
	} else {
		pr_warn("no pmic restart interrupt specified\n");
	}

	return 0;
}

late_initcall(msm_pmic_restart_init);

static int __init msm_restart_init(void)
{
	htc_restart_handler_init();

	atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
#ifdef CONFIG_HTC_FEATURES_FINAL_EFS_SYNC
	register_reboot_notifier(&notify_efs_sync_notifier);
#endif

#ifdef CONFIG_MSM_DLOAD_MODE
	dload_mode_addr = MSM_IMEM_BASE + DLOAD_MODE_ADDR;
	emergency_dload_mode_addr = MSM_IMEM_BASE +
		EMERGENCY_DLOAD_MODE_ADDR;
	set_dload_mode(download_mode);
#endif
	msm_tmr0_base = msm_timer_get_timer0_base();
	pm_power_off = msm_power_off;

	if (scm_is_call_available(SCM_SVC_PWR, SCM_IO_DISABLE_PMIC_ARBITER) > 0)
		scm_pmic_arbiter_disable_supported = true;

	return 0;
}
early_initcall(msm_restart_init);
