/* drivers/input/misc/gpio_input.c
 *
 * Copyright (C) 2007 Google, Inc.
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

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <mach/board.h>
#include <linux/irq.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <mach/devices_cmdline.h>
#if defined(CONFIG_PWRKEY_STATUS_API) || defined(CONFIG_PWRKEY_WAKESRC_LOG)
#include <linux/module.h>
#endif

#ifdef CONFIG_POWER_KEY_LED
#ifdef CONFIG_QPNP_PWM
#include <linux/qpnp/pwm.h>
#else
#include <linux/les-pm8921.h>
#endif

#define PWRKEYLEDON_DELAY 3*HZ
#define PWRKEYKP_DELAY (6*HZ)
#define PWRKEYLEDOFF_DELAY 0
#define HW_RESET_REASON 0x44332211
static int power_key_led_requested;
static int pre_power_key_status;
static int pre_power_key_led_status;
#endif 

#ifdef CONFIG_POWER_KEY_CLR_RESET
#include <linux/pl_sensor.h>
#define PWRKEYCHKRST_DELAY (3*HZ + HZ/2)
#define PWRKEYCLRCHK_DELAY 0
#define PWRKEYCHKRST_WAKELOCK_TIMEOUT (PWRKEYCHKRST_DELAY + 1 * HZ)
struct wake_lock key_reset_clr_wake_lock;
#ifdef CONFIG_KPDPWR_S2_DVDD_RESET
#include <linux/qpnp/power-on.h>
struct hrtimer clr_kpd_reset_timer;
struct hrtimer enable_kpd_s2_timer;
static int clear_kpdpwr_s2_rst_flag;
#define KPDPWR_CLR_RESET_TIMER (150 * NSEC_PER_MSEC) 
#endif 
#endif
#ifdef CONFIG_MFD_MAX8957
static struct workqueue_struct *ki_queue;
#endif
static int power_key_intr_flag;
static DEFINE_MUTEX(wakeup_mutex);
static unsigned char wakeup_bitmask;
static unsigned char set_wakeup;
static unsigned int vol_up_irq;
static unsigned int vol_down_irq;
static ssize_t vol_wakeup_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	unsigned char bitmask = 0;
	bitmask = simple_strtoull(buf, NULL, 10);
	mutex_lock(&wakeup_mutex);
	if (bitmask) {
		if (bitmask == 127)
			wakeup_bitmask &= bitmask;
		else if (bitmask > 128)
			wakeup_bitmask &= bitmask;
		else
			wakeup_bitmask |= bitmask;
	}

	if (wakeup_bitmask && (!set_wakeup)) {
		enable_irq_wake(vol_up_irq);
		enable_irq_wake(vol_down_irq);
		set_wakeup = 1;
		KEY_LOGI("%s:change to wake up function(%d, %d)\n", __func__, vol_up_irq, vol_down_irq);
	} else if ((!wakeup_bitmask) && set_wakeup){
		disable_irq_wake(vol_up_irq);
		disable_irq_wake(vol_down_irq);
		set_wakeup = 0;
		KEY_LOGI("%s:change to non-wake up function(%d, %d)\n", __func__, vol_up_irq, vol_down_irq);
	}
	mutex_unlock(&wakeup_mutex);
	return count;
}
static ssize_t vol_wakeup_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%x\n", wakeup_bitmask);
}

static DEVICE_ATTR(vol_wakeup, 0664, vol_wakeup_show, vol_wakeup_store);

#ifdef CONFIG_PWRKEY_WAKESRC_LOG
static uint16_t power_key_gpio;
uint16_t get_power_key_gpio(void)
{
	return power_key_gpio;
}
EXPORT_SYMBOL(get_power_key_gpio);
#endif

#ifdef CONFIG_PWRKEY_STATUS_API
static uint8_t power_key_state;
static spinlock_t power_key_state_lock;

#define PWRKEY_PRESS_DUE 1*HZ
#include <linux/module.h>
static void init_power_key_api(void)
{
	spin_lock_init(&power_key_state_lock);
	power_key_state = 0;
}

static void setPowerKeyState(uint8_t flag)
{
	spin_lock(&power_key_state_lock);
	power_key_state = flag;
	spin_unlock(&power_key_state_lock);
}

uint8_t getPowerKeyState(void)
{
	uint8_t value;

	spin_lock(&power_key_state_lock);
	value = power_key_state;
	spin_unlock(&power_key_state_lock);

	return value;
}
EXPORT_SYMBOL(getPowerKeyState);

static void power_key_state_disable_work_func(struct work_struct *dummy)
{
    setPowerKeyState(0);

    KEY_LOGI("[PWR][STATE]power key pressed outdated\n");
}
static DECLARE_DELAYED_WORK(power_key_state_disable_work, power_key_state_disable_work_func);

static void handle_power_key_state(unsigned int code, int value)
{
	int ret = 0;
	if (code == KEY_POWER && value == 1) {
		KEY_LOGI("[PWR][STATE]try to schedule power key pressed due\n");
		ret = schedule_delayed_work(&power_key_state_disable_work, PWRKEY_PRESS_DUE);
		if (!ret) {
			KEY_LOGI("[PWR][STATE]Schedule power key pressed due failed, seems already have one, try to cancel...\n");
			ret = __cancel_delayed_work(&power_key_state_disable_work);
			if (!ret) {
				setPowerKeyState(1);
				if (schedule_delayed_work(&power_key_state_disable_work, PWRKEY_PRESS_DUE)) {
					KEY_LOGI("[PWR][STATE]Re-schedule power key pressed due SCCUESS.\n");
					KEY_LOGI("[PWR][STATE] start count for power key pressed due\n");
					setPowerKeyState(1);
				} else
					KEY_LOGI("[PWR][STATE]Re-schedule power key pressed due FAILED, reason unknown, give up.\n");
			} else {
				KEY_LOGI("[PWR][STATE]Cancel scheduled power key due success, now re-schedule.\n");
				if (schedule_delayed_work(&power_key_state_disable_work, PWRKEY_PRESS_DUE)) {
					KEY_LOGI("[PWR][STATE]Re-schedule power key pressed due SCCUESS.\n");
					KEY_LOGI("[PWR][STATE] start count for power key pressed due\n");
					setPowerKeyState(1);
				} else
					KEY_LOGI("[PWR][STATE]Re-schedule power key pressed due FAILED, reason unknown, give up.\n");
			}
		} else {
			KEY_LOGI("[PWR][STATE] start count for power key pressed due\n");
			setPowerKeyState(1);
		}
	}
}

#endif

#ifdef CONFIG_MFD_MAX8957
static struct workqueue_struct *ki_queue;
#endif

enum {
	DEBOUNCE_UNSTABLE     = BIT(0),	
	DEBOUNCE_PRESSED      = BIT(1),
	DEBOUNCE_NOTPRESSED   = BIT(2),
	DEBOUNCE_WAIT_IRQ     = BIT(3),	
	DEBOUNCE_POLL         = BIT(4),	

	DEBOUNCE_UNKNOWN =
		DEBOUNCE_PRESSED | DEBOUNCE_NOTPRESSED,
};

struct gpio_key_state {
	struct gpio_input_state *ds;
	uint8_t debounce;
#ifdef CONFIG_MFD_MAX8957
	struct work_struct work;
#endif
};

struct gpio_input_state {
	int debug_log;
	struct gpio_event_input_devs *input_devs;
	const struct gpio_event_input_info *info;
#ifndef CONFIG_MFD_MAX8957
	struct hrtimer timer;
#endif
	int use_irq;
	int debounce_count;
	spinlock_t irq_lock;
	struct wake_lock wake_lock;
#ifdef CONFIG_MFD_MAX8957
	struct wake_lock key_pressed_wake_lock;
#endif
	struct gpio_key_state key_state[0];
};

#ifdef CONFIG_POWER_KEY_LED
static ssize_t kernel_write(struct file *file, const char *buf,
	size_t count, loff_t pos)
{
	mm_segment_t old_fs;
	ssize_t res;

	old_fs = get_fs();
	set_fs(get_ds());
	
	res = vfs_write(file, (const char __user *)buf, count, &pos);
	set_fs(old_fs);

	return res;
}

static int set_hw_reason(int reason)
{
	char filename[32] = "";
	int hw_reason = reason;
	struct file *filp = NULL;
	ssize_t nread;

	int pnum = get_partition_num_by_name("misc");

	if (pnum < 0) {
		pr_info("unknown partition number for misc partition\n");
		return 0;
	}

	snprintf(filename, 32, "/dev/block/mmcblk0p%d", pnum);

	filp = filp_open(filename, O_RDWR, 0);
	if (IS_ERR(filp)) {
		pr_info("unable to open file: %s\n", filename);
		return PTR_ERR(filp);
	}

	filp->f_pos = 624;
	nread = kernel_write(filp, (char *)&hw_reason, sizeof(int), filp->f_pos);
	pr_info("wrire: %X (%d)\n", hw_reason, nread);

	if (filp)
		filp_close(filp, NULL);

	return 1;
}

#if defined(CONFIG_PM8921_BMS) && (CONFIG_HTC_BATT_8960)
#include <linux/mfd/pm8xxx/pm8921-bms.h>
#endif

#ifdef CONFIG_POWER_KEY_CLR_RESET
#include <mach/restart.h>
static struct gpio_event_input_info *gis;
int set_restart_to_ramdump(const char *msg);

static int is_rrm1_mode(void)
{
	return gis->info.rrm1_mode;
}

static void clear_hw_reset(void)
{
	struct gpio_event_input_info *aa = gis;
	KEY_LOGI("[PWR] %s\n", __func__);
#ifdef CONFIG_OF
	if ((aa->dt_clear_hw_reset)) {
		printk(KERN_INFO "[KEY] clear hw reset\n");
		aa->dt_clear_hw_reset(aa->clr_gpio);
	}
#else
	if ((aa->clear_hw_reset)) {
		printk(KERN_INFO "[KEY] clear hw reset\n");
		aa->clear_hw_reset();
	}
#endif
	else {
		KEY_LOGI("[PWR] No reset  clear function\n");
	}
}

static void power_key_restart_work_func(struct work_struct *dummy)
{
#ifndef CONFIG_POWER_VOLUP_RESET
	int pocket_mode = (board_mfg_mode() == MFG_MODE_NORMAL) ? power_key_check_in_pocket() : 0;
	KEY_LOGI( "%s: power_key_check_in_pocket = %d\n", __func__, pocket_mode);

	if (!pocket_mode && pre_power_key_led_status == 1 && !is_rrm1_mode()) {
#else
	uint8_t gpio_val = 0, i, idx_pwr = 9, idx_vup = 9;
	struct gpio_event_input_info *local = gis;
	for (i = 0; i < local->keymap_size; i++) {
		if (local->keymap[i].code == KEY_POWER) {
			idx_pwr = i;
			continue;
		}
		if (local->keymap[i].code == KEY_VOLUMEUP)
			idx_vup = i;
	}
	gpio_val = !(gpio_get_value(local->keymap[idx_pwr].gpio) |
		     gpio_get_value(local->keymap[idx_vup].gpio));

	if (gpio_val && pre_power_key_led_status == 1 && !is_rrm1_mode()) {
		KEY_LOGI("%s, (PWR+VOL_UP) reset", __func__);
#endif
		
		set_hw_reason(0);
#if defined(CONFIG_PM8921_BMS) && (CONFIG_HTC_BATT_8960)
		pm8921_store_hw_reset_reason(1);
#endif
		clear_hw_reset();
		set_restart_to_ramdump("Powerkey Hard Reset - SW");
		msm_restart(0, NULL);
	}
}

static DECLARE_DELAYED_WORK(power_key_restart_work, power_key_restart_work_func);
#endif

static void power_key_led_on_work_func(struct work_struct *dummy)
{
	KEY_LOGI("[PWR] %s in (%x)\n", __func__, power_key_led_requested);
	if (power_key_led_requested == 1) {
		pre_power_key_led_status = 1;
#ifdef CONFIG_POWER_KEY_CLR_RESET
		schedule_delayed_work(&power_key_restart_work, PWRKEYKP_DELAY);
#endif
		KEY_LOGI("[PWR] change power key led on\n");
#ifdef CONFIG_QPNP_PWM
#ifdef CONFIG_VK_LED
		qpnp_led_set_for_key(1);
#endif
#else
		pm8xxx_led_current_set_for_key(1);
#endif
#ifdef CONFIG_POWER_KEY_CLR_RESET
		if(!is_rrm1_mode())
#endif
			set_hw_reason(HW_RESET_REASON);
		KEY_LOGI("[PWR] Show Blocked State -- long press power key\n");
		show_state_filter(TASK_UNINTERRUPTIBLE);
	}
}
static DECLARE_DELAYED_WORK(power_key_led_on_work, power_key_led_on_work_func);

static void power_key_led_off_work_func(struct work_struct *dummy)
{
	if (power_key_led_requested) {
		if (cancel_delayed_work_sync(&power_key_led_on_work)) {
			KEY_LOGI("[PWR] cancel power key led work successfully(%x)\n", power_key_led_requested);
		} else
			KEY_LOGI("[PWR] cancel power key led work unsuccessfully (%x)\n", power_key_led_requested);

		power_key_led_requested = 0;
	}
	if (pre_power_key_led_status == 1) {
#ifdef CONFIG_POWER_KEY_CLR_RESET
		cancel_delayed_work_sync(&power_key_restart_work);
#endif
		KEY_LOGI("[PWR] change power key led off\n");
#ifdef CONFIG_QPNP_PWM
#ifdef CONFIG_VK_LED
		qpnp_led_set_for_key(0);
#endif
#else
		pm8xxx_led_current_set_for_key(0);
#endif
		pre_power_key_led_status = 0;
#ifdef CONFIG_POWER_KEY_CLR_RESET
		if(!is_rrm1_mode())
#endif
			set_hw_reason(0);
	}
}
static DECLARE_DELAYED_WORK(power_key_led_off_work, power_key_led_off_work_func);

static void handle_power_key_led(unsigned int code, int value)
{
	if (code == KEY_POWER) {
		if (pre_power_key_status == value)
			return;
		pre_power_key_status = value;
		if (value) {
#ifndef CONFIG_POWER_VOLUP_RESET
			KEY_LOGI("[PWR] start count for power key led on\n");
			schedule_delayed_work(&power_key_led_on_work, PWRKEYLEDON_DELAY);
#endif
			power_key_led_requested = 1;
		} else {
			KEY_LOGI("[PWR] start count for power key led off\n");
			schedule_delayed_work(&power_key_led_off_work, PWRKEYLEDOFF_DELAY);
		}
	}
}
#endif

#ifdef CONFIG_POWER_KEY_CLR_RESET
static void handle_power_key_reset(unsigned int code, int value);
static void power_key_check_reset_work_func(struct work_struct *dummy)
{
	struct gpio_event_input_info *aa = gis;
#ifdef CONFIG_POWER_VOLUP_RESET
	uint8_t val = 0, i = 0;
#else
	int pocket_mode = 0;
#endif
	KEY_LOGI("[PWR] %s\n", __func__);

#ifdef CONFIG_OF
	if ((aa->dt_clear_hw_reset)) {
		if (aa->info.rrm1_mode) {
			KEY_LOGI(" Power key check in Lab Test RRM1 mode.\n");
			aa->dt_clear_hw_reset(aa->clr_gpio);
#else
	if ((aa->clear_hw_reset)) {
		if (aa->info.rrm1_mode) {
			printk(KERN_INFO "[KEY] Power key check in Lab Test RRM1 mode.\n");
			aa->clear_hw_reset();
#endif
#ifndef CONFIG_POWER_VOLUP_RESET
		} else if (board_mfg_mode() == MFG_MODE_NORMAL) {
			
			pocket_mode = power_key_check_in_pocket();
			if (pocket_mode) {
				printk(KERN_INFO "[KEY] power_key_check_in_pocket = %d\n", pocket_mode);
#ifdef CONFIG_OF
				aa->dt_clear_hw_reset(aa->clr_gpio);
#else
				aa->clear_hw_reset();
#endif
			}
		} else
			printk(KERN_INFO "[KEY] Not in normal OS mode, mode=%d\n", board_mfg_mode());
#else	
		} else {
			printk(KERN_INFO "[KEY] OS Mode=%d\n", board_mfg_mode());
			for (i = 0; i < aa->keymap_size; i++) {
				if (aa->keymap[i].code == KEY_VOLUMEUP) {
					val = gpio_get_value(aa->keymap[i].gpio);
					break;
				}
			}
			if (!val) { 	
				KEY_LOGI("HW RESET continue");
			} else {	
#ifdef CONFIG_OF
				aa->dt_clear_hw_reset(aa->clr_gpio);
#else
				aa->clear_hw_reset();
#endif
			}
		}
#endif	
	} else {
		KEY_LOGI("[PWR] No reset  clear function\n");
	}
	handle_power_key_reset(KEY_POWER, 1);
}
static DECLARE_DELAYED_WORK(power_key_check_reset_work, power_key_check_reset_work_func);

static void power_key_clr_check_work_func(struct work_struct *dummy)
{
#ifdef CONFIG_POWER_VOLUP_RESET
	uint8_t i = 0, val = 0, pwr_idx = 0;
	struct gpio_event_input_info *aa = gis;

	for (i = 0; i < aa->keymap_size; i++) {
		if (aa->keymap[i].code == KEY_VOLUMEUP) {
			val = gpio_get_value(aa->keymap[i].gpio);
			if (val) {
				KEY_LOGI("volUP clear");
#ifdef CONFIG_OF
				aa->dt_clear_hw_reset(aa->clr_gpio);
#else
				aa->clear_hw_reset();
#endif
			}
			break;
		}
		if (aa->keymap[i].code == KEY_POWER)
			pwr_idx = i;
	}

	val = gpio_get_value(aa->keymap[pwr_idx].gpio);

	if (val) {
#endif 
		if (cancel_delayed_work_sync(&power_key_check_reset_work))
			KEY_LOGI("[PWR] cancel power key check reset work successfully\n");
		else
			KEY_LOGI("[PWR] cancel power key check reset work unsuccessfully\n");
#ifdef CONFIG_KPDPWR_S2_DVDD_RESET
		clear_kpdpwr_s2_rst_flag = 0;
		KEY_LOGD("%s: Disable kpdpwr s2 reset clear up [%d]\n", __func__, clear_kpdpwr_s2_rst_flag);
		if (hrtimer_is_queued(&clr_kpd_reset_timer))
			hrtimer_cancel(&clr_kpd_reset_timer);
		if (hrtimer_is_queued(&enable_kpd_s2_timer))
			hrtimer_cancel(&enable_kpd_s2_timer);
#endif 
#ifdef CONFIG_POWER_VOLUP_RESET
	}
#endif
	wake_unlock(&key_reset_clr_wake_lock);
}
static DECLARE_DELAYED_WORK(power_key_clr_check_work, power_key_clr_check_work_func);

static void handle_power_key_reset(unsigned int code, int value)
{
#ifdef CONFIG_POWER_VOLUP_RESET
	uint8_t i = 0, read_val = 0;
	struct gpio_event_input_info *aa = gis;
	if (code == KEY_POWER || code == KEY_VOLUMEUP) {
		if (!value && code == KEY_VOLUMEUP) {
			KEY_LOGI("[VUP] start count for power key led off\n");
			schedule_delayed_work(&power_key_led_off_work, PWRKEYLEDOFF_DELAY);
		}
		for (i = 0; (i < aa->keymap_size && code != KEY_POWER); i++) {
			if (aa->keymap[i].code == KEY_POWER) {
				read_val = gpio_get_value(aa->keymap[i].gpio);
				KEY_LOGI("Idx[%d] GPIO_%d:PWR is %s\n",
						i, aa->keymap[i].gpio,
						(read_val ? "NOT pressed" : "PRESSED" ));
				if (read_val) {
#ifdef CONFIG_KPDPWR_S2_DVDD_RESET
					clear_kpdpwr_s2_rst_flag = 0;
					KEY_LOGD("%s: Disable kpdpwr s2 reset clear up [%d]\n",
								__func__, clear_kpdpwr_s2_rst_flag);
#endif 
					return;
				} else {
					KEY_LOGI("[PWR+VUP] start count for power key led on\n");
					schedule_delayed_work(&power_key_led_on_work, PWRKEYLEDON_DELAY);
					power_key_led_requested = 1;
#ifdef CONFIG_KPDPWR_S2_DVDD_RESET
					clear_kpdpwr_s2_rst_flag = 1;
					KEY_LOGD("%s: Enable kpdpwr s2 reset clear up [%d]\n",
								__func__, clear_kpdpwr_s2_rst_flag);
#endif 
					break;
				}
			}
		}
		for (i = aa->keymap_size; (i > 0  && code != KEY_VOLUMEUP); i--) {
			if (aa->keymap[i-1].code == KEY_VOLUMEUP) {
				read_val = gpio_get_value(aa->keymap[i-1].gpio);
				KEY_LOGI("Idx[%d] GPIO_%d:VOL_UP is %s\n",
						i-1, aa->keymap[i-1].gpio,
						(read_val ? "NOT pressed" : "PRESSED" ));
				if (read_val) {
					goto KEY_PWR;
				} else {
					KEY_LOGI("[VUP+PWR] start count for power key led on\n");
					schedule_delayed_work(&power_key_led_on_work, PWRKEYLEDON_DELAY);
					power_key_led_requested = 1;
					break;
				}
			}
		}
KEY_PWR:
#else
	if (code == KEY_POWER) {
#endif
		if (value) {
#ifdef CONFIG_KPDPWR_S2_DVDD_RESET
			clear_kpdpwr_s2_rst_flag = 1;
			KEY_LOGD("%s: Enable kpdpwr s2 reset clear up [%d]\n",
						__func__, clear_kpdpwr_s2_rst_flag);
			hrtimer_start(&clr_kpd_reset_timer,
				ktime_set(0, KPDPWR_CLR_RESET_TIMER), HRTIMER_MODE_REL);
#endif 
#ifdef CONFIG_POWER_VOLUP_RESET
			wake_lock_timeout(&key_reset_clr_wake_lock, msecs_to_jiffies(2000));
			KEY_LOGI("[PWR] start count for power key check reset\n");
			if (!schedule_delayed_work(&power_key_check_reset_work, msecs_to_jiffies(1000)))
#else
			wake_lock_timeout(&key_reset_clr_wake_lock, PWRKEYCHKRST_WAKELOCK_TIMEOUT);
			KEY_LOGI("[PWR] start count for power key check reset\n");
			if (!schedule_delayed_work(&power_key_check_reset_work, PWRKEYCHKRST_DELAY))
#endif
				KEY_LOGI("[PWR] the reset work in already in the queue\n");
		} else {
			KEY_LOGI("[PWR] start count for power key clear check\n");

			if (!schedule_delayed_work(&power_key_clr_check_work, PWRKEYCLRCHK_DELAY))
				KEY_LOGI("[PWR] the clear work in already in the queue\n");
#ifdef CONFIG_KPDPWR_S2_DVDD_RESET
			clear_kpdpwr_s2_rst_flag = 0;
			KEY_LOGD("%s: Disable kpdpwr s2 reset clear up [%d]\n",
						__func__, clear_kpdpwr_s2_rst_flag);
			if (hrtimer_is_queued(&clr_kpd_reset_timer))
				hrtimer_cancel(&clr_kpd_reset_timer);
			if (hrtimer_is_queued(&enable_kpd_s2_timer))
				hrtimer_cancel(&enable_kpd_s2_timer);
#endif 
		}
	}
}

#ifdef CONFIG_KPDPWR_S2_DVDD_RESET
static enum hrtimer_restart clr_kpd_rst_timer_func(struct hrtimer *timer)
{
	if (qpnp_get_reset_en(PON_KPDPWR) > 0) {
		qpnp_config_reset_enable(PON_KPDPWR, 0);
		qpnp_config_reset_enable(PON_KPDPWR, 1);
		hrtimer_start(&enable_kpd_s2_timer, ktime_set(0, 0), HRTIMER_MODE_REL);
	}

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart enable_kpd_s2_timer_func(struct hrtimer *timer)
{
	qpnp_config_reset_enable(PON_KPDPWR, 1);
	if (clear_kpdpwr_s2_rst_flag) {
		hrtimer_start(&clr_kpd_reset_timer,
			ktime_set(0, KPDPWR_CLR_RESET_TIMER), HRTIMER_MODE_REL);
	} else {
	        if (hrtimer_is_queued(&clr_kpd_reset_timer))
			hrtimer_cancel(&clr_kpd_reset_timer);
		if (hrtimer_is_queued(&enable_kpd_s2_timer))
			hrtimer_cancel(&enable_kpd_s2_timer);
	}

	return HRTIMER_NORESTART;
}
#endif 
#endif 

#ifndef CONFIG_MFD_MAX8957
static enum hrtimer_restart gpio_event_input_timer_func(struct hrtimer *timer)
{
	int i;
	int pressed;
	struct gpio_input_state *ds =
		container_of(timer, struct gpio_input_state, timer);
	unsigned gpio_flags = ds->info->flags;
	unsigned npolarity;
	int nkeys = ds->info->keymap_size;
	const struct gpio_event_direct_entry *key_entry;
	struct gpio_key_state *key_state;
	unsigned long irqflags;
	uint8_t debounce;
	bool sync_needed;

#if 0
	key_entry = kp->keys_info->keymap;
	key_state = kp->key_state;
	for (i = 0; i < nkeys; i++, key_entry++, key_state++)
		pr_info("gpio_read_detect_status %d %d\n", key_entry->gpio,
			gpio_read_detect_status(key_entry->gpio));
#endif
	key_entry = ds->info->keymap;
	key_state = ds->key_state;
	sync_needed = false;
	spin_lock_irqsave(&ds->irq_lock, irqflags);
	for (i = 0; i < nkeys; i++, key_entry++, key_state++) {
		debounce = key_state->debounce;
		if (debounce & DEBOUNCE_WAIT_IRQ)
			continue;
		if (key_state->debounce & DEBOUNCE_UNSTABLE) {
			debounce = key_state->debounce = DEBOUNCE_UNKNOWN;
#if 0
			enable_irq(gpio_to_irq(key_entry->gpio));
#endif
			if (gpio_flags & GPIOEDF_PRINT_KEY_UNSTABLE)
				KEY_LOGI("gpio_keys_scan_keys: key %x-%x, %d "
					"(%d) continue debounce\n",
					ds->info->type, key_entry->code,
					i, key_entry->gpio);
		}
		npolarity = !(gpio_flags & GPIOEDF_ACTIVE_HIGH);
		pressed = gpio_get_value(key_entry->gpio) ^ npolarity;
		if (debounce & DEBOUNCE_POLL) {
			if (pressed == !(debounce & DEBOUNCE_PRESSED)) {
				ds->debounce_count++;
				key_state->debounce = DEBOUNCE_UNKNOWN;
				if (gpio_flags & GPIOEDF_PRINT_KEY_DEBOUNCE)
					KEY_LOGI("gpio_keys_scan_keys: key %x-"
						"%x, %d (%d) start debounce\n",
						ds->info->type, key_entry->code,
						i, key_entry->gpio);
			}
			continue;
		}
		if (pressed && (debounce & DEBOUNCE_NOTPRESSED)) {
			if (gpio_flags & GPIOEDF_PRINT_KEY_DEBOUNCE)
				KEY_LOGI("gpio_keys_scan_keys: key %x-%x, %d "
					"(%d) debounce pressed 1\n",
					ds->info->type, key_entry->code,
					i, key_entry->gpio);
			key_state->debounce = DEBOUNCE_PRESSED;
			continue;
		}
		if (!pressed && (debounce & DEBOUNCE_PRESSED)) {
			if (gpio_flags & GPIOEDF_PRINT_KEY_DEBOUNCE)
				KEY_LOGI("gpio_keys_scan_keys: key %x-%x, %d "
					"(%d) debounce pressed 0\n",
					ds->info->type, key_entry->code,
					i, key_entry->gpio);
			key_state->debounce = DEBOUNCE_NOTPRESSED;
			continue;
		}
		
		ds->debounce_count--;
		if (ds->use_irq)
			key_state->debounce |= DEBOUNCE_WAIT_IRQ;
		else
			key_state->debounce |= DEBOUNCE_POLL;
		if (gpio_flags & GPIOEDF_PRINT_KEYS)
			KEY_LOGI("gpio_keys_scan_keys: key %x-%x, %d (%d) "
				"changed to %d\n", ds->info->type,
				key_entry->code, i, key_entry->gpio, pressed);
#ifdef CONFIG_POWER_KEY_LED
		handle_power_key_led(key_entry->code, pressed);
#endif
#ifdef CONFIG_POWER_KEY_CLR_RESET
		handle_power_key_reset(key_entry->code, pressed);
#endif
#ifdef CONFIG_PWRKEY_STATUS_API
		handle_power_key_state(key_entry->code, pressed);
#endif

		input_event(ds->input_devs->dev[key_entry->dev], ds->info->type,
			    key_entry->code, pressed);
		sync_needed = true;
	}
	if (sync_needed) {
		for (i = 0; i < ds->input_devs->count; i++)
			input_sync(ds->input_devs->dev[i]);
	}

#if 0
	key_entry = kp->keys_info->keymap;
	key_state = kp->key_state;
	for (i = 0; i < nkeys; i++, key_entry++, key_state++) {
		pr_info("gpio_read_detect_status %d %d\n", key_entry->gpio,
			gpio_read_detect_status(key_entry->gpio));
	}
#endif

	if (ds->debounce_count)
		hrtimer_start(timer, ds->info->debounce_time, HRTIMER_MODE_REL);
	else if (!ds->use_irq)
		hrtimer_start(timer, ds->info->poll_time, HRTIMER_MODE_REL);
	else
		wake_unlock(&ds->wake_lock);

	spin_unlock_irqrestore(&ds->irq_lock, irqflags);

	return HRTIMER_NORESTART;
}
#endif
#ifdef CONFIG_MFD_MAX8957
void keypad_report_keycode(struct gpio_key_state *ks)
{
	struct gpio_input_state *ds = ks->ds;
	int keymap_index;
	const struct gpio_event_direct_entry *key_entry;
	int pressed;

	if (ds == NULL) {
		KEY_LOGE("%s, (ds == NULL) failed\n", __func__);
		return;
	}
	keymap_index = ks - ds->key_state;

	key_entry = &ds->info->keymap[keymap_index];
	if (key_entry == NULL) {
		KEY_LOGE("%s, (key_entry == NULL) failed\n", __func__);
		return;
	}

	pressed = gpio_get_value(key_entry->gpio) ^
			!(ds->info->flags & GPIOEDF_ACTIVE_HIGH);

	if (key_entry->code == KEY_POWER) {
		if (pressed)
			wake_lock(&ds->key_pressed_wake_lock);
		else
			wake_unlock(&ds->key_pressed_wake_lock);
	}

	if (ds->info->flags & GPIOEDF_PRINT_KEYS)
		KEY_LOGD("%s: key %d-%d, %d "
			"(%d) changed to %d\n", __func__,
			ds->info->type, key_entry->code, keymap_index,
			key_entry->gpio, pressed);

#ifdef CONFIG_POWER_KEY_LED
	handle_power_key_led(key_entry->code, pressed);
#endif
#ifdef CONFIG_POWER_KEY_CLR_RESET
	handle_power_key_reset(key_entry->code, pressed);
#endif

	input_event(ds->input_devs->dev[key_entry->dev],
			ds->info->type, key_entry->code, pressed);
	input_sync(ds->input_devs->dev[key_entry->dev]);
}

static void keypad_do_work(struct work_struct *w)
{
	struct gpio_key_state *ks = container_of(w, struct gpio_key_state, work);
	keypad_report_keycode(ks);
}
#endif

static irqreturn_t gpio_event_input_irq_handler(int irq, void *dev_id)
{
	struct gpio_key_state *ks = dev_id;
	struct gpio_input_state *ds = ks->ds;
	int keymap_index = ks - ds->key_state;
	const struct gpio_event_direct_entry *key_entry;
	unsigned long irqflags;
#ifndef CONFIG_MFD_MAX8957
	int pressed;
#endif
	KEY_LOGD("%s, irq=%d, use_irq=%d\n", __func__, irq, ds->use_irq);

	if (!ds->use_irq)
		return IRQ_HANDLED;

	disable_irq_nosync(irq);	
	key_entry = &ds->info->keymap[keymap_index];

	if (key_entry->code == KEY_POWER && power_key_intr_flag == 0) {
		irq_set_irq_type(irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
		power_key_intr_flag = 1;
		KEY_LOGD("%s, keycode = %d, first intr", __func__, key_entry->code);
	}
	if (ds->info->debounce_time.tv64) {
		spin_lock_irqsave(&ds->irq_lock, irqflags);
		if (ks->debounce & DEBOUNCE_WAIT_IRQ) {
			ks->debounce = DEBOUNCE_UNKNOWN;
			if (ds->debounce_count++ == 0) {
				wake_lock(&ds->wake_lock);
#ifndef CONFIG_MFD_MAX8957
				hrtimer_start(
					&ds->timer, ds->info->debounce_time,
					HRTIMER_MODE_REL);
#endif
			}
			if (ds->info->flags & GPIOEDF_PRINT_KEY_DEBOUNCE)
				KEY_LOGD("gpio_event_input_irq_handler: "
					"key %x-%x, %d (%d) start debounce\n",
					ds->info->type, key_entry->code,
					keymap_index, key_entry->gpio);
		} else {
#if 0
			disable_irq_nosync(irq);
#endif
			ks->debounce = DEBOUNCE_UNSTABLE;
		}
		spin_unlock_irqrestore(&ds->irq_lock, irqflags);
	} else {
#ifdef CONFIG_MFD_MAX8957
		queue_work(ki_queue, &ks->work);
#else
		pressed = gpio_get_value(key_entry->gpio) ^
			!(ds->info->flags & GPIOEDF_ACTIVE_HIGH);
		if (ds->info->flags & GPIOEDF_PRINT_KEYS)
			KEY_LOGD("gpio_event_input_irq_handler: key %x-%x, %d "
				"(%d) changed to %d\n",
				ds->info->type, key_entry->code, keymap_index,
				key_entry->gpio, pressed);
		input_event(ds->input_devs->dev[key_entry->dev], ds->info->type,
			    key_entry->code, pressed);
		input_sync(ds->input_devs->dev[key_entry->dev]);
#endif
	}
	enable_irq(irq);		
	return IRQ_HANDLED;
}

static int gpio_event_input_request_irqs(struct gpio_input_state *ds)
{
	int i;
	int err;
	int value;
	unsigned int irq;
	unsigned long req_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

	for (i = 0; i < ds->info->keymap_size; i++) {
		err = irq = gpio_to_irq(ds->info->keymap[i].gpio);
		if (err < 0)
			goto err_gpio_get_irq_num_failed;
		if (ds->info->keymap[i].code == KEY_POWER) {
#ifdef CONFIG_PWRKEY_WAKESRC_LOG
			power_key_gpio = ds->info->keymap[i].gpio;
			KEY_LOGI("Power Key gpio = %d", power_key_gpio);
#endif
			power_key_intr_flag = 0;
			value = gpio_get_value(ds->info->keymap[i].gpio);
			req_flags = value ? IRQF_TRIGGER_FALLING: IRQF_TRIGGER_RISING;
			KEY_LOGI("keycode = %d, gpio = %d, type = %lx", ds->info->keymap[i].code, value, req_flags);
		}
		else
			req_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
#ifdef CONFIG_MFD_MAX8957
		INIT_WORK(&ds->key_state[i].work, keypad_do_work);
		queue_work(ki_queue, &ds->key_state[i].work);
#endif
		err = request_any_context_irq(irq, gpio_event_input_irq_handler,
				  req_flags, "gpio_keys", &ds->key_state[i]);
		if (err < 0) {
			KEY_LOGE("KEY_ERR: %s: request_irq "
				"failed for input %d, irq %d, err %d\n", __func__,
				ds->info->keymap[i].gpio, irq, err);
			goto err_request_irq_failed;
		}
		if (ds->info->keymap[i].code == KEY_VOLUMEUP ||
			ds->info->keymap[i].code == KEY_VOLUMEDOWN || ds->info->keymap[i].code == KEY_HP ) {
			KEY_LOGI("keycode = %d, gpio = %d, irq = %d", ds->info->keymap[i].code, ds->info->keymap[i].gpio, irq);
			if (ds->info->keymap[i].code == KEY_VOLUMEUP)
				vol_up_irq = irq;
			else if (ds->info->keymap[i].code == KEY_VOLUMEDOWN)
				vol_down_irq = irq;
		} else
			enable_irq_wake(irq);
	}
#ifdef CONFIG_PWRKEY_STATUS_API
	init_power_key_api();
#endif
	return 0;

	for (i = ds->info->keymap_size - 1; i >= 0; i--) {
		free_irq(gpio_to_irq(ds->info->keymap[i].gpio),
			 &ds->key_state[i]);
err_request_irq_failed:
err_gpio_get_irq_num_failed:
		;
	}
	return err;
}

int gpio_event_input_func(struct gpio_event_input_devs *input_devs,
			struct gpio_event_info *info, void **data, int func)
{
	int ret;
	int i;
	unsigned long irqflags;
	struct gpio_event_input_info *di;
	struct gpio_input_state *ds = *data;
	struct kobject *keyboard_kobj;

	di = container_of(info, struct gpio_event_input_info, info);

#ifdef CONFIG_POWER_KEY_CLR_RESET
	gis = di;
#endif

	if (func == GPIO_EVENT_FUNC_SUSPEND) {
		if (ds->use_irq)
			for (i = 0; i < di->keymap_size; i++)
				disable_irq(gpio_to_irq(di->keymap[i].gpio));
#ifndef CONFIG_MFD_MAX8957
		hrtimer_cancel(&ds->timer);
#endif
		return 0;
	}
	if (func == GPIO_EVENT_FUNC_RESUME) {
		spin_lock_irqsave(&ds->irq_lock, irqflags);
		if (ds->use_irq)
			for (i = 0; i < di->keymap_size; i++)
				enable_irq(gpio_to_irq(di->keymap[i].gpio));
#ifndef CONFIG_MFD_MAX8957
		hrtimer_start(&ds->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
#endif
		spin_unlock_irqrestore(&ds->irq_lock, irqflags);
		return 0;
	}

	if (func == GPIO_EVENT_FUNC_INIT) {
#ifdef CONFIG_POWER_KEY_CLR_RESET
#ifdef CONFIG_OF
		if (di->info.rrm1_mode && di->dt_clear_hw_reset) {
			KEY_LOGI("First clear reset in Lab Test RRM1 mode.\n");
			if (di->dt_clear_hw_reset)
				di->dt_clear_hw_reset(di->clr_gpio);
#else
		if (di->info.rrm1_mode && di->clear_hw_reset) {
			KEY_LOGI("First clear reset in Lab Test RRM1 mode.\n");
			if (di->clear_hw_reset)
				di->clear_hw_reset();
#endif
		}
#endif 

		if (ktime_to_ns(di->poll_time) <= 0)
			di->poll_time = ktime_set(0, 20 * NSEC_PER_MSEC);

		*data = ds = kzalloc(sizeof(*ds) + sizeof(ds->key_state[0]) *
					di->keymap_size, GFP_KERNEL);
		if (ds == NULL) {
			ret = -ENOMEM;
			KEY_LOGE("KEY_ERR: %s: "
				"Failed to allocate private data\n", __func__);
			goto err_ds_alloc_failed;
		}
		ds->debounce_count = di->keymap_size;
		ds->input_devs = input_devs;
		ds->info = di;
		wake_lock_init(&ds->wake_lock, WAKE_LOCK_SUSPEND, "gpio_input");
#ifdef CONFIG_MFD_MAX8957
		wake_lock_init(&ds->key_pressed_wake_lock, WAKE_LOCK_SUSPEND, "pwr_key_pressed");
#endif
#ifdef CONFIG_POWER_KEY_CLR_RESET
		wake_lock_init(&key_reset_clr_wake_lock, WAKE_LOCK_SUSPEND, "gpio_input_pwr_clear");
#endif
		spin_lock_init(&ds->irq_lock);
		if (board_build_flag() == BUILD_MODE_SHIP)
			ds->debug_log = 0;
		else
			ds->debug_log = 1;

		for (i = 0; i < di->keymap_size; i++) {
			int dev = di->keymap[i].dev;
			if (dev >= input_devs->count) {
				KEY_LOGE("KEY_ERR: %s: bad device "
					"index %d >= %d for key code %d\n",
					__func__, dev, input_devs->count,
					di->keymap[i].code);
				ret = -EINVAL;
				goto err_bad_keymap;
			}
			input_set_capability(input_devs->dev[dev], di->type,
					     di->keymap[i].code);
			ds->key_state[i].ds = ds;
			ds->key_state[i].debounce = DEBOUNCE_UNKNOWN;
		}

		for (i = 0; i < di->keymap_size; i++) {
			ret = gpio_request(di->keymap[i].gpio, "gpio_kp_in");
			if (ret) {
				KEY_LOGE("KEY_ERR: %s: gpio_request "
					"failed for %d\n", __func__, di->keymap[i].gpio);
				goto err_gpio_request_failed;
			}
			ret = gpio_direction_input(di->keymap[i].gpio);
			if (ret) {
				KEY_LOGE("KEY_ERR: %s: "
					"gpio_direction_input failed for %d\n",
					__func__, di->keymap[i].gpio);
				goto err_gpio_configure_failed;
			}
		}

#ifdef CONFIG_OF
		if (di->dt_setup_input_gpio)
			di->dt_setup_input_gpio(di->keymap, di->keymap_size);
#else
		if (di->setup_input_gpio)
			di->setup_input_gpio();
#endif
#ifdef CONFIG_MFD_MAX8957
		ki_queue = create_singlethread_workqueue("ki_queue");
#endif

		ret = gpio_event_input_request_irqs(ds);

		keyboard_kobj = kobject_create_and_add("keyboard", NULL);
		if (keyboard_kobj == NULL) {
			KEY_LOGE("KEY_ERR: %s: subsystem_register failed\n", __func__);
			ret = -ENOMEM;
			return ret;
		}
		if (sysfs_create_file(keyboard_kobj, &dev_attr_vol_wakeup.attr))
			KEY_LOGE("KEY_ERR: %s: sysfs_create_file "
					"return %d\n", __func__, ret);
		wakeup_bitmask = 0;
		set_wakeup = 0;
		spin_lock_irqsave(&ds->irq_lock, irqflags);
		ds->use_irq = ret == 0;

		KEY_LOGI("GPIO Input Driver: Start gpio inputs for %s%s in %s "
			"mode\n", input_devs->dev[0]->name,
			(input_devs->count > 1) ? "..." : "",
			ret == 0 ? "interrupt" : "polling");

#ifndef CONFIG_MFD_MAX8957
		hrtimer_init(&ds->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ds->timer.function = gpio_event_input_timer_func;
		hrtimer_start(&ds->timer, ktime_set(0, 0), HRTIMER_MODE_REL);
#endif
#if defined(CONFIG_KPDPWR_S2_DVDD_RESET) && defined(CONFIG_POWER_KEY_CLR_RESET)
		hrtimer_init(&clr_kpd_reset_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		clr_kpd_reset_timer.function = clr_kpd_rst_timer_func;
		hrtimer_init(&enable_kpd_s2_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		enable_kpd_s2_timer.function = enable_kpd_s2_timer_func;
#endif 
		spin_unlock_irqrestore(&ds->irq_lock, irqflags);
		return 0;
	}

	ret = 0;
	spin_lock_irqsave(&ds->irq_lock, irqflags);
#ifndef CONFIG_MFD_MAX8957
	hrtimer_cancel(&ds->timer);
#endif
	if (ds->use_irq) {
		for (i = di->keymap_size - 1; i >= 0; i--) {
			free_irq(gpio_to_irq(di->keymap[i].gpio),
				 &ds->key_state[i]);
		}
	}
	spin_unlock_irqrestore(&ds->irq_lock, irqflags);

	for (i = di->keymap_size - 1; i >= 0; i--) {
err_gpio_configure_failed:
		gpio_free(di->keymap[i].gpio);
err_gpio_request_failed:
		;
	}
err_bad_keymap:
	wake_lock_destroy(&ds->wake_lock);
#ifdef CONFIG_MFD_MAX8957
	wake_lock_destroy(&ds->key_pressed_wake_lock);
#endif
#ifdef CONFIG_POWER_KEY_CLR_RESET
	wake_lock_destroy(&key_reset_clr_wake_lock);
#endif
	kfree(ds);
err_ds_alloc_failed:
	return ret;
}
