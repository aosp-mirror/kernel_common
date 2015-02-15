/* drivers/android/ram_console.c
 *
 * Copyright (C) 2007-2008 Google, Inc.
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

#include <linux/console.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/persistent_ram.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include "ram_console.h"

static struct persistent_ram_zone *ram_console_zone;
static const char *bootinfo;
static size_t bootinfo_size;
static char *rst_msg_buf;
static unsigned long rst_msg_buf_size = 0;

#if defined(CONFIG_HTC_DEBUG_RAMCONSOLE)
#include <mach/devices_cmdline.h>

static size_t bl_old_log_size;
static const char *bl_old_log;
static size_t bl_log_size;
static const char *bl_log;
static char *bl_old_log_buf;
static unsigned long bl_old_log_buf_size = 0;
static char *bl_log_buf;
static unsigned long bl_log_buf_size = 0;

void bldr_log_parser(const char *bldr_log, char *bldr_log_buf, unsigned long bldr_log_size, unsigned long *bldr_log_buf_size)
{
	int i,j,k;
	int last_index=0;
	int line_length=0;
	const char *bldr_log_ptr=bldr_log;
	char *bldr_log_buf_ptr=bldr_log_buf;
	const char *terminal_pattern="\r\n";
	const char *specific_pattern="[HBOOT]";
	int terminal_pattern_len=strlen(terminal_pattern);
	int specific_pattern_len=strlen(specific_pattern);

	if (!get_tamper_sf()) {
		memcpy(bldr_log_buf, bldr_log, bldr_log_size);
		*bldr_log_buf_size = bldr_log_size;
		printk(KERN_INFO "[K] bldr_log_parser: size %ld\n", *bldr_log_buf_size);
		return;
	}

	for(i=0; i<bldr_log_size; i++) 
	{
		bool terminal_match = true;

		if((i+terminal_pattern_len) > bldr_log_size)
			break;

		for(j=0; j < terminal_pattern_len; j++) 
		{
			if(bldr_log[i+j] != terminal_pattern[j])
			{
				terminal_match = false;
				break;
			}
		}

		if(terminal_match) 
		{
			bool specific_match = true;
			int specific_pattern_start = i-specific_pattern_len;
			line_length = i+terminal_pattern_len-last_index;

			for(k=0; k < specific_pattern_len; k++) 
			{
				if(bldr_log[specific_pattern_start+k] != specific_pattern[k])
				{
					specific_match = false;
					break;
				}
			}

			if(specific_match) 
			{
				
				memcpy(bldr_log_buf_ptr, bldr_log_ptr, line_length-terminal_pattern_len-specific_pattern_len);
				bldr_log_buf_ptr +=(line_length-terminal_pattern_len-specific_pattern_len);
				memcpy(bldr_log_buf_ptr, terminal_pattern, terminal_pattern_len);
				bldr_log_buf_ptr +=terminal_pattern_len;
			}

			bldr_log_ptr+=line_length;
			last_index=i+terminal_pattern_len;
		}
	}

	*bldr_log_buf_size = bldr_log_buf_ptr - bldr_log_buf;
	printk(KERN_INFO "[K] bldr_log_parser: size %ld\n", *bldr_log_buf_size);
}

static bool bldr_rst_msg_parser(char *bldr_log_buf, unsigned long bldr_log_buf_size, bool is_last_bldr)
{
	int i,j,k;
	const char *ramdump_pattern_rst="ramdump_show_rst_msg:";
	const char *ramdump_pattern_vib="###[ RAMDUMP Mode ]###";
	const char *ramdump_pattern_real="ramdump_real_rst_msg:";
	int ramdump_pattern_rst_len=strlen(ramdump_pattern_rst);
	int ramdump_pattern_vib_len=strlen(ramdump_pattern_vib);
	int ramdump_pattern_real_len=strlen(ramdump_pattern_real);
	char *bldr_ramdump_pattern_rst_buf_ptr = NULL;
	bool is_ramdump_mode = false;
	bool found_ramdump_pattern_rst = false;

	for(i=0; i<bldr_log_buf_size; i++) 
	{
		bool ramdump_pattern_rst_match = true;
		bool ramdump_pattern_vib_match = true;

		if(!found_ramdump_pattern_rst &&
			(i+ramdump_pattern_rst_len) <= bldr_log_buf_size)
		{
			for(j=0; j < ramdump_pattern_rst_len; j++) 
			{
				if(bldr_log_buf[i+j] != ramdump_pattern_rst[j])
				{
					ramdump_pattern_rst_match = false;
					break;
				}
			}

			if(ramdump_pattern_rst_match) 
			{
				if(is_last_bldr)
					bldr_ramdump_pattern_rst_buf_ptr = bldr_log_buf+i;
				else
					memcpy(bldr_log_buf+i, ramdump_pattern_real, ramdump_pattern_real_len);
				found_ramdump_pattern_rst = true;
			}
		}

		if(!is_ramdump_mode &&
			(i+ramdump_pattern_vib_len) <= bldr_log_buf_size &&
			is_last_bldr)
		{
			for(k=0; k < ramdump_pattern_vib_len; k++) 
			{
				if(bldr_log_buf[i+k] != ramdump_pattern_vib[k])
				{
					ramdump_pattern_vib_match = false;
					break;
				}
			}

			if(ramdump_pattern_vib_match) 
				is_ramdump_mode = true;
		}

		if(found_ramdump_pattern_rst &&
			is_ramdump_mode &&
			is_last_bldr)  
		{
			memcpy(bldr_ramdump_pattern_rst_buf_ptr, ramdump_pattern_real, ramdump_pattern_real_len);
			break;
		}
	}

	if(is_last_bldr)
		return is_ramdump_mode;
	else
		return found_ramdump_pattern_rst;
}
#endif

static void
ram_console_write(struct console *console, const char *s, unsigned int count)
{
	struct persistent_ram_zone *prz = console->data;
	persistent_ram_write(prz, s, count);
}

static struct console ram_console = {
	.name	= "ram",
	.write	= ram_console_write,
	.flags	= CON_PRINTBUFFER | CON_ENABLED | CON_ANYTIME,
	.index	= -1,
};

void ram_console_enable_console(int enabled)
{
	if (enabled)
		ram_console.flags |= CON_ENABLED;
	else
		ram_console.flags &= ~CON_ENABLED;
}

static int __devinit ram_console_probe(struct platform_device *pdev)
{
	struct ram_console_platform_data *pdata = pdev->dev.platform_data;
	struct persistent_ram_zone *prz;

#if defined(CONFIG_HTC_DEBUG_RAMCONSOLE)
	struct resource *bl_old_log_res;
	struct resource *bl_log_res;

	bl_old_log_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "bl_old_log");
	if (bl_old_log_res) {
		bl_old_log = ioremap(bl_old_log_res->start, resource_size(bl_old_log_res));
		if (bl_old_log == NULL) {
			pr_warn("[K] failed to map last bootloader log buffer\n");
		} else {
			bl_old_log_size = resource_size(bl_old_log_res);
			pr_info("[K] got last bootloader log buffer at %zx, size %zx\n", bl_old_log_res->start, bl_old_log_size);
		}
	}
	bl_log_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "bl_log");
	if (bl_log_res) {
		bl_log = ioremap(bl_log_res->start, resource_size(bl_log_res));
		if (bl_log == NULL) {
			pr_warn("[K] failed to map bootloader log buffer\n");
		} else {
			bl_log_size = resource_size(bl_log_res);
			pr_info("[K] got bootloader log buffer at %zx, size %zx\n", bl_log_res->start, bl_log_size);
		}
	}
#endif

	rst_msg_buf = board_get_google_boot_reason();
	rst_msg_buf_size = strlen(rst_msg_buf);

	prz = persistent_ram_init_ringbuffer_by_name("ram_console", false);
	if (IS_ERR(prz))
		return PTR_ERR(prz);


	if (pdata) {
		bootinfo = kstrdup(pdata->bootinfo, GFP_KERNEL);
		if (bootinfo)
			bootinfo_size = strlen(bootinfo);
	}

	ram_console_zone = prz;
	ram_console.data = prz;

	register_console(&ram_console);

	return 0;
}

static struct of_device_id ram_console_dt_match_table[] = {
	{
		.compatible = "ram_console"
	},
	{},
};

static struct platform_driver ram_console_driver = {
	.driver		= {
		.name	= "ram_console",
		.of_match_table = ram_console_dt_match_table,
	},
	.probe = ram_console_probe,
};

static int __init ram_console_module_init(void)
{
	return platform_driver_register(&ram_console_driver);
}

#ifndef CONFIG_PRINTK
#define dmesg_restrict	0
#endif

static ssize_t ram_console_read_old(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count;
	struct persistent_ram_zone *prz = ram_console_zone;
	size_t old_log_size = persistent_ram_old_size(prz);
	const char *old_log = persistent_ram_old(prz);
	char *str;
	int ret;

	if (dmesg_restrict && !capable(CAP_SYSLOG))
		return -EPERM;

#if defined(CONFIG_HTC_DEBUG_RAMCONSOLE)
	
	if (pos < bl_old_log_buf_size) {
		count = min(len, (size_t)(bl_old_log_buf_size - pos));
		if (copy_to_user(buf, bl_old_log_buf + pos, count))
			return -EFAULT;
		goto out;
	}

	pos -= bl_old_log_buf_size;
#endif

	
	if (pos < old_log_size) {
		count = min(len, (size_t)(old_log_size - pos));
		if (copy_to_user(buf, old_log + pos, count))
			return -EFAULT;
		goto out;
	}

	
	pos -= old_log_size;
	count = persistent_ram_ecc_string(prz, NULL, 0);
	if (pos < count) {
		str = kmalloc(count, GFP_KERNEL);
		if (!str)
			return -ENOMEM;
		persistent_ram_ecc_string(prz, str, count + 1);
		count = min(len, (size_t)(count - pos));
		ret = copy_to_user(buf, str + pos, count);
		kfree(str);
		if (ret)
			return -EFAULT;
		goto out;
	}

	
	pos -= count;
	if (pos < rst_msg_buf_size) {
		count = min(len, (size_t)(rst_msg_buf_size - pos));
		if (copy_to_user(buf, rst_msg_buf + pos, count))
			return -EFAULT;
		goto out;
	}

	
	pos -= rst_msg_buf_size;
	if (pos < bootinfo_size) {
		count = min(len, (size_t)(bootinfo_size - pos));
		if (copy_to_user(buf, bootinfo + pos, count))
			return -EFAULT;
		goto out;
	}

#if defined(CONFIG_HTC_DEBUG_RAMCONSOLE)
	
	pos -= bootinfo_size;
	if (pos < bl_log_buf_size) {
		count = min(len, (size_t)(bl_log_buf_size - pos));
		if (copy_to_user(buf, bl_log_buf + pos, count))
			return -EFAULT;
		goto out;
	}
#endif

	
	return 0;

out:
	*offset += count;
	return count;
}

static const struct file_operations ram_console_file_ops = {
	.owner = THIS_MODULE,
	.read = ram_console_read_old,
};

static int __init ram_console_late_init(void)
{
	struct proc_dir_entry *entry;
	struct persistent_ram_zone *prz = ram_console_zone;
#if defined(CONFIG_HTC_DEBUG_RAMCONSOLE)
	bool is_last_bldr_ramdump_mode = false;
#endif

	if (!prz)
		return 0;

	if (persistent_ram_old_size(prz) == 0)
		return 0;

	entry = create_proc_entry("last_kmsg", S_IFREG | S_IRUGO, NULL);
	if (!entry) {
		printk(KERN_ERR "ram_console: failed to create proc entry\n");
		persistent_ram_free_old(prz);
		return 0;
	}

#if defined(CONFIG_HTC_DEBUG_RAMCONSOLE)
	if (bl_old_log != NULL) {
		bl_old_log_buf = kmalloc(bl_old_log_size, GFP_KERNEL);
		if (bl_old_log_buf == NULL) {
			printk(KERN_ERR "[K] ram_console: failed to allocate buffer for last bootloader log, size: %zu\n", bl_old_log_size);
		} else {
			printk(KERN_INFO "[K] ram_console: allocate buffer for last bootloader log, size: %zu\n", bl_old_log_size);
			bldr_log_parser(bl_old_log, bl_old_log_buf, bl_old_log_size, &bl_old_log_buf_size);
			is_last_bldr_ramdump_mode = bldr_rst_msg_parser(bl_old_log_buf, bl_old_log_buf_size, true);
			if (is_last_bldr_ramdump_mode)
				printk(KERN_INFO "[K] ram_console: Found abnormal rst_msg pattern in last bootloader log\n");
		}
	}
	if (bl_log != NULL) {
		bl_log_buf = kmalloc(bl_log_size, GFP_KERNEL);
		if (bl_log_buf == NULL) {
			printk(KERN_ERR "[K] ram_console: failed to allocate buffer for bootloader log, size: %zu\n", bl_log_size);
		} else {
			printk(KERN_INFO "[K] ram_console: allocate buffer for bootloader log, size: %zu\n", bl_log_size);
			bldr_log_parser(bl_log, bl_log_buf, bl_log_size, &bl_log_buf_size);
			if (!is_last_bldr_ramdump_mode) {
				if (bldr_rst_msg_parser(bl_log_buf, bl_log_buf_size, false))
					printk(KERN_INFO "[K] ram_console: Found abnormal rst_msg pattern in bootloader log\n");
			}
		}
	}
#endif

	entry->proc_fops = &ram_console_file_ops;
	entry->size = persistent_ram_old_size(prz) +
		persistent_ram_ecc_string(prz, NULL, 0) +
		bootinfo_size;
#if defined(CONFIG_HTC_DEBUG_RAMCONSOLE)
	entry->size += bl_old_log_buf_size;
	entry->size += bl_log_buf_size;
#endif

	return 0;
}

late_initcall(ram_console_late_init);
postcore_initcall(ram_console_module_init);
