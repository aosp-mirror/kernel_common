/* arch/arm/mach-msm/htc_footprint.c
 * Copyright (C) 2013 HTC Corporation.
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
#include <linux/io.h>
#include <mach/htc_mnemosyne.h>
#include <mach/htc_footprint.h>
#include <linux/sched.h>
#include "idle.h"

#define APPS_WDOG_FOOT_PRINT_MAGIC			0xACBDFE00

#define HOTPLUG_ON_MAGIC				0xACBDBB00

int read_backup_cc_uah(void)
{
	int cur_batt_magic, cur_cc_backup_uah;

	cur_batt_magic = MNEMOSYNE_GET(batt_magic);
	cur_cc_backup_uah = MNEMOSYNE_GET(cc_backup_uah);

	pr_info("%s: cc_backup_uah=%d, magic=%x\n", __func__,
			cur_cc_backup_uah, cur_batt_magic);
	if((cur_batt_magic & 0xFFFFFF00) == MAGIC_NUM_FOR_BATT_SAVE) {
		if ((cur_batt_magic & 0xFF) <= BATT_SAVE_MASK) {
			if ((cur_batt_magic & HTC_BATT_SAVE_CC)
					== HTC_BATT_SAVE_CC)
				return cur_cc_backup_uah;
		}
	}
	return 0;
}

void write_backup_cc_uah(int cc_reading)
{
	int cur_batt_magic, cur_cc_backup_uah;

	cur_batt_magic = MNEMOSYNE_GET(batt_magic);
	cur_cc_backup_uah = MNEMOSYNE_GET(cc_backup_uah);

	pr_info("%s: ori cc_backup_uah=%d, cc_reading=%d, magic_num=%x\n",
		__func__, cur_cc_backup_uah, cc_reading, cur_batt_magic);
	if ((cur_batt_magic & ~BATT_SAVE_MASK) != MAGIC_NUM_FOR_BATT_SAVE)
		cur_batt_magic = MAGIC_NUM_FOR_BATT_SAVE;

	cur_batt_magic |= HTC_BATT_SAVE_CC;

	MNEMOSYNE_SET(batt_magic, cur_batt_magic);
	MNEMOSYNE_SET(cc_backup_uah, cc_reading);
	mb();
}

int read_backup_ocv_uv(void)
{
	int cur_batt_magic, cur_ocv_backup_uv;

	cur_batt_magic = MNEMOSYNE_GET(batt_magic);
	cur_ocv_backup_uv = MNEMOSYNE_GET(ocv_backup_uv);

	pr_info("%s: ocv_backup_uv=%d, magic=%x\n", __func__,
			cur_ocv_backup_uv, cur_batt_magic);
	if((cur_batt_magic & 0xFFFFFF00) == MAGIC_NUM_FOR_BATT_SAVE) {
		if ((cur_batt_magic & 0xFF) <= BATT_SAVE_MASK) {
			if ((cur_batt_magic & HTC_BATT_SAVE_OCV_UV)
					== HTC_BATT_SAVE_OCV_UV)
				return cur_ocv_backup_uv;
		}
	}
	return 0;
}

void write_backup_ocv_uv(int ocv_backup)
{
	int cur_batt_magic, cur_ocv_backup_uv;

	cur_batt_magic = MNEMOSYNE_GET(batt_magic);
	cur_ocv_backup_uv = MNEMOSYNE_GET(ocv_backup_uv);
	pr_info("%s: ori ocv_backup_uv=%d, ocv_backup=%d, magic_num=%x\n",
		__func__, cur_ocv_backup_uv, ocv_backup, cur_batt_magic);
	if((cur_batt_magic & ~BATT_SAVE_MASK) != MAGIC_NUM_FOR_BATT_SAVE)
		cur_batt_magic = MAGIC_NUM_FOR_BATT_SAVE;

	cur_batt_magic |= HTC_BATT_SAVE_OCV_UV;

	MNEMOSYNE_SET(batt_magic, cur_batt_magic);
	MNEMOSYNE_SET(ocv_backup_uv, ocv_backup);
	mb();
}

void set_msm_watchdog_en_footprint(int enable)
{
	MNEMOSYNE_SET(apps_watchdog_en, (APPS_WDOG_FOOT_PRINT_MAGIC | enable));
	mb();
}

void set_msm_watchdog_pet_time_utc(void)
{
	struct timespec ts;

	getnstimeofday(&ts);

	MNEMOSYNE_SET(apps_watchdog_pet_utc, ts.tv_sec);

	mb();
}

void set_msm_watchdog_pet_footprint(unsigned int sleep_clk_base)
{
	uint32_t sleep_clk = __raw_readl(sleep_clk_base);
	unsigned long long timestamp_ms = sched_clock();

	do_div(timestamp_ms, NSEC_PER_MSEC);

	if (likely(!timekeeping_suspended)) {
		set_msm_watchdog_pet_time_utc();
	}

	MNEMOSYNE_SET(apps_watchdog_pet, sleep_clk);
	MNEMOSYNE_SET(apps_watchdog_pet_schedclk, (unsigned long)timestamp_ms);
	mb();
}

void set_acpuclk_footprint(unsigned cpu, unsigned state)
{
	if (unlikely(cpu >= NR_CPUS)) {
		WARN(1, "Only %d cores, but try to set footprint for core %d\n", NR_CPUS, cpu);
		return;
	}

	MNEMOSYNE_SET_I(acpuclk_set_rate_footprint_cpu, cpu, (CPU_FOOT_PRINT_MAGIC | state));
	mb();
}

void set_acpuclk_cpu_freq_footprint(enum FREQ_TYPE type, unsigned cpu, unsigned khz)
{
	if (unlikely(cpu >= NR_CPUS)) {
		WARN(1, "Only %d cores, but try to set footprint for core %d\n", NR_CPUS, cpu);
		return;
	}

	switch (type) {
		case FT_PREV_RATE:
			MNEMOSYNE_SET_I(cpu_prev_frequency, cpu, khz);
			break;
		case FT_CUR_RATE:
			MNEMOSYNE_SET_I(cpu_frequency, cpu, khz);
			break;
		case FT_NEW_RATE:
			MNEMOSYNE_SET_I(cpu_new_frequency, cpu, khz);
			break;
	}
	mb();
}

void set_acpuclk_l2_freq_footprint(enum FREQ_TYPE type, unsigned khz)
{
	switch (type) {
		case FT_PREV_RATE:
			MNEMOSYNE_SET(l2_prev_frequency, khz);
			break;
		case FT_CUR_RATE:
			MNEMOSYNE_SET(l2_frequency, khz);
			break;
		case FT_NEW_RATE:
			MNEMOSYNE_SET(l2_new_frequency, khz);
			break;
	}
	mb();
}

void init_cpu_debug_counter_for_cold_boot(void)
{
	int i = 0;
	for (i = 0; i < NR_CPUS; i++) {
		MNEMOSYNE_SET_I(kernel_exit_counter_from_cpu, i, 0x0);
	}
	mb();
}

void init_cpu_foot_print(unsigned cpu, bool from_idle, bool notify_rpm)
{
	unsigned state = CPU_FOOT_PRINT_MAGIC_HOTPLUG;
	bool is_spc = !notify_rpm;
	bool not_hotplug = !cpu || from_idle;

	if (unlikely(cpu >= NR_CPUS)) {
		WARN(1, "Only %d cores, but try to set footprint for core %d\n", NR_CPUS, cpu);
		return;
	}

	if (not_hotplug) {
		if (is_spc)
			state = (from_idle) ? CPU_FOOT_PRINT_MAGIC_SPC_FROM_IDLE : CPU_FOOT_PRINT_MAGIC_SPC;
		else
			state = (from_idle) ? CPU_FOOT_PRINT_MAGIC_FROM_IDLE : CPU_FOOT_PRINT_MAGIC;
	}

	MNEMOSYNE_SET_I(kernel_footprint_cpu, cpu, state);
	mb();
}

void set_cpu_foot_print(unsigned cpu, unsigned state)
{
	unsigned mask = 0xFF;
	unsigned new_state;

	if (unlikely(cpu >= NR_CPUS)) {
		WARN(1, "Only %d cores, but try to set footprint for core %d\n", NR_CPUS, cpu);
		return;
	}

	new_state = (MNEMOSYNE_GET_I(kernel_footprint_cpu, cpu) & ~mask) | (state & mask);

	MNEMOSYNE_SET_I(kernel_footprint_cpu, cpu, new_state)
	mb();
}

void clean_reset_vector_debug_info(unsigned cpu)
{
	if (unlikely(cpu >= NR_CPUS)) {
		WARN(1, "Only %d cores, but try to set footprint for core %d\n", NR_CPUS, cpu);
		return;
	}

	MNEMOSYNE_SET_I(reset_vector_for_cpu, cpu, RESET_VECTOR_CLEAN_MAGIC);
	mb();
}

void set_reset_vector(unsigned cpu)
{
	if (unlikely(cpu >= NR_CPUS)) {
		WARN(1, "Only %d cores, but try to set footprint for core %d\n", NR_CPUS, cpu);
		return;
	}

	MNEMOSYNE_SET_I(reset_vector_for_cpu, cpu, msm_pm_boot_vector[cpu]);
	mb();
}

void set_reset_vector_address_after_pc(unsigned cpu)
{
	if (unlikely(cpu >= NR_CPUS)) {
		WARN(1, "Only %d cores, but try to set footprint for core %d\n", NR_CPUS, cpu);
		return;
	}

	MNEMOSYNE_SET_I(cpu_reset_vector_address, cpu, virt_to_phys(&msm_pm_boot_vector[cpu]));
	mb();
}

void set_reset_vector_value_after_pc(unsigned cpu)
{
	if (unlikely(cpu >= NR_CPUS)) {
		WARN(1, "Only %d cores, but try to set footprint for core %d\n", NR_CPUS, cpu);
		return;
	}

	MNEMOSYNE_SET_I(cpu_reset_vector_address_value, cpu, msm_pm_boot_vector[cpu]);
	mb();
}

void store_pm_boot_entry_addr(void)
{
	MNEMOSYNE_SET(msm_pm_boot_entry, virt_to_phys(msm_pm_boot_entry));
	mb();
}

void store_pm_boot_vector_addr(unsigned value)
{
	MNEMOSYNE_SET(msm_pm_boot_vector, value);
	mb();
}

void set_hotplug_on_footprint(unsigned cpu, unsigned value)
{
	MNEMOSYNE_SET_I(cpu_hotplug_on, cpu, HOTPLUG_ON_MAGIC | (value & 0xFF));
	mb();
}
