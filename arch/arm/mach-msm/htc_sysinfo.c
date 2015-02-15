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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <mach/board.h>

#define MSM_MAX_PARTITIONS 52

static int emmc_partition_update;
struct htc_emmc_partition {
	unsigned int dev_num;
	unsigned int partition_size;
	char partition_name[16];
};

static struct htc_emmc_partition emmc_partitions[MSM_MAX_PARTITIONS];

int htc_emmc_partition_read(char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	struct htc_emmc_partition *ptn = emmc_partitions;
	char *p = page;
	int i;
	unsigned int dev_num;
	unsigned int size;

	p += sprintf(p, "dev:        size     erasesize name\n");

	for (i = 0; i < MSM_MAX_PARTITIONS && (ptn->partition_size != 0); i++, ptn++) {
		dev_num = ptn->dev_num;
		size = ptn->partition_size;
		p += sprintf(p, "mmcblk0p%u: %08x  %08x  \"%s\"\n",
				dev_num, size * 512, 512, ptn->partition_name);
	}

	return p - page;
}

int get_partition_num_by_name(char *name)
{
	struct htc_emmc_partition *ptn = emmc_partitions;
	int i;

	for (i = 0; i < MSM_MAX_PARTITIONS && ptn->partition_name; i++, ptn++) {
		if (strcmp(ptn->partition_name, name) == 0)
			return ptn->dev_num;
	}

	return -1;
}
EXPORT_SYMBOL(get_partition_num_by_name);

void add_emmc_part_entry(unsigned int dev_num, unsigned int part_size, char *name)
{
	struct htc_emmc_partition *ptn = emmc_partitions;
	int i;
	if (emmc_partition_update < 0 || emmc_partition_update >= MSM_MAX_PARTITIONS)
		return;

	for (i = 0; i < MSM_MAX_PARTITIONS; i++, ptn++) {
		if (ptn->dev_num == dev_num) {
			pr_debug("%s: partition exists.\n", __func__);
			return;
		}
	}
	strncpy(emmc_partitions[emmc_partition_update].partition_name, name, sizeof(emmc_partitions[emmc_partition_update].partition_name)-1);
	emmc_partitions[emmc_partition_update].dev_num = dev_num;
	emmc_partitions[emmc_partition_update].partition_size = part_size;

	emmc_partition_update ++;
}

int htc_emmc_partition_write(struct file *file, const char *buffer,
				unsigned long count, void *data)
{
	struct htc_emmc_partition *ptn = emmc_partitions;
	int i;
	char buf[64];
	unsigned int dev_num, partition_size;
	char partition_name[16];
	int ret;
	return count;

	if (emmc_partition_update < 0 || emmc_partition_update >= MSM_MAX_PARTITIONS)
		return 0;

	if (copy_from_user(buf, buffer, 64))
		return -EFAULT;

	if ((ret = sscanf(buf, "%d %d %16s", &dev_num, &partition_size, partition_name)) != 3) {
		pr_info("%s: partition information format error:\
			%d items input matched. \n", __func__, ret);
		return -EINVAL;
	}

	for (i = 0; i < MSM_MAX_PARTITIONS; i++, ptn++) {
		if (ptn->dev_num == dev_num) {
			pr_debug("%s: partition exists.\n", __func__);
			return count;
		}
	}

	strncpy(emmc_partitions[emmc_partition_update].partition_name, partition_name, 16);
	emmc_partitions[emmc_partition_update].dev_num = dev_num;
	emmc_partitions[emmc_partition_update].partition_size = partition_size;

	emmc_partition_update ++;

	return count;
}

extern int board_get_boot_powerkey_debounce_time(void);
int sys_boot_powerkey_debounce_ms(char *page, char **start, off_t off,
                           int count, int *eof, void *data)
{
        char *p = page;

        p += sprintf(p, "%d\n", board_get_boot_powerkey_debounce_time());

        return p - page;
}

static int __init sysinfo_proc_init(void)
{
	struct proc_dir_entry *entry = NULL;

	memset(emmc_partitions, 0, sizeof(struct htc_emmc_partition));

	emmc_partition_update = 0;
	pr_info("%s: Init HTC system info proc interface.\r\n", __func__);

	entry = create_proc_entry("emmc", 0444, NULL);
	if (entry == NULL) {
		pr_err("%s: unable to create /proc entry\n", __func__);
        } else {
		entry->read_proc = htc_emmc_partition_read;
		entry->write_proc = htc_emmc_partition_write;
	}

        entry = create_proc_read_entry("powerkey_debounce_ms", 0, NULL, sys_boot_powerkey_debounce_ms, NULL);
	if (entry == NULL) {
		pr_err("%s: unable to create /proc/powerkey_debounce_ms entry\n", __func__);
        }

	return 0;
}

module_init(sysinfo_proc_init);
MODULE_AUTHOR("Jimmy.CM Chen <jimmy.cm_chen@htc.com>");
MODULE_DESCRIPTION("HTC System Info Interface");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
