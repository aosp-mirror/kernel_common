/* arch/arm/mach-msm/htc_mnemosyne_footprint.inc
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

/* Declare footprints here, use macro to be compatible with assembly and c/c++ */
DECLARE_MNEMOSYNE_START()

	/* Old footprints group */
	DECLARE_MNEMOSYNE_ARRAY(kernel_footprint_cpu, NR_CPUS)
	DECLARE_MNEMOSYNE_ARRAY(kernel_exit_counter_from_cpu, NR_CPUS)
	DECLARE_MNEMOSYNE(msm_pm_boot_entry)
	DECLARE_MNEMOSYNE(msm_pm_boot_vector)
	DECLARE_MNEMOSYNE_ARRAY(reset_vector_for_cpu, NR_CPUS)
	DECLARE_MNEMOSYNE_ARRAY(cpu_reset_vector_address, NR_CPUS)
	DECLARE_MNEMOSYNE_ARRAY(cpu_reset_vector_address_value, NR_CPUS)
	DECLARE_MNEMOSYNE_ARRAY(cpu_frequency, NR_CPUS)
	DECLARE_MNEMOSYNE(l2_frequency)
	DECLARE_MNEMOSYNE_ARRAY(acpuclk_set_rate_footprint_cpu, NR_CPUS)
	DECLARE_MNEMOSYNE_ARRAY(cpu_prev_frequency, NR_CPUS)
	DECLARE_MNEMOSYNE(l2_prev_frequency)
	DECLARE_MNEMOSYNE_ARRAY(cpu_new_frequency, NR_CPUS)
	DECLARE_MNEMOSYNE(l2_new_frequency)
	DECLARE_MNEMOSYNE_ARRAY(cpu_hotplug_on, NR_CPUS)
	/* fill the gap before next footprint group,
	   REMEMBER to update the size upon changing above footprints.
	 */
	DECLARE_MNEMOSYNE_ARRAY(backward_compatible_region,
		(NORMAL_FOOTPRINT_GROUP_OFFSET/MNEMOSYNE_ELEMENT_SIZE -
			(3 + 2 + NR_CPUS * (7 + 2 + 1))))

	/* Second footprint group */
	DECLARE_MNEMOSYNE(apps_watchdog_en)
	DECLARE_MNEMOSYNE(apps_watchdog_pet)
	DECLARE_MNEMOSYNE(apps_watchdog_pet_schedclk)
	DECLARE_MNEMOSYNE(apps_watchdog_pet_utc)
	DECLARE_MNEMOSYNE(batt_magic)
	DECLARE_MNEMOSYNE(cc_backup_uah)
	DECLARE_MNEMOSYNE(ocv_backup_uv)
DECLARE_MNEMOSYNE_END()
