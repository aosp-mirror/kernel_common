/*
 * bootmsg.c - keep the first BOOTMSG_BUFFER_SZ bytes of kernel log to memory
 *
 * Copyright (C) HTC Corporation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define BOOTMSG_BUFFER_SZ (256 << 10)
#define RELEASE_RETRY_DELAY (msecs_to_jiffies(60 * MSEC_PER_SEC))

static long  bootmsg_buffer_phys = 0x0;
static char* bootmsg_buffer = NULL;
static int   bootmsg_len    = 0;

static bool should_release_console = false;

static void
bootmsg_console_write(struct console *console, const char *s, unsigned int count)
{
	int len = min(count, (unsigned int) (BOOTMSG_BUFFER_SZ - bootmsg_len));
	if (len) {
		memcpy(bootmsg_buffer + bootmsg_len, s, len);
		bootmsg_len += len;
	}

	if (bootmsg_len >= BOOTMSG_BUFFER_SZ) {
		console->flags &= ~CON_ENABLED;
		should_release_console = true;
	}
}

static struct console bootmsg_console = {
	.name	= "bootmsg",
	.write	= bootmsg_console_write,
	.flags	= CON_PRINTBUFFER | CON_ENABLED | CON_ANYTIME,
	.index	= -1,
};

static void release_console_work_fn(struct work_struct *work)
{
	if (should_release_console) {
		unregister_console(&bootmsg_console);
		pr_info("bootmsg is disabled (bootmsg_buffer_phys=%08lx)\n",
				bootmsg_buffer_phys);
	} else
		schedule_delayed_work(container_of(work, struct delayed_work, work),
				RELEASE_RETRY_DELAY);
}
static DECLARE_DELAYED_WORK(release_console_work, release_console_work_fn);

static int __init bootmsg_release_console_work_init(void)
{
	schedule_delayed_work(&release_console_work, RELEASE_RETRY_DELAY);
	return 0;
}
late_initcall(bootmsg_release_console_work_init);

static int __init bootmsg_console_init(void)
{
	WARN_ON(BOOTMSG_BUFFER_SZ % PAGE_SIZE);

	bootmsg_buffer = kzalloc(BOOTMSG_BUFFER_SZ, GFP_KERNEL);
	if (bootmsg_buffer) {
		bootmsg_buffer_phys = virt_to_phys(bootmsg_buffer);
		register_console(&bootmsg_console);
	} else
		pr_err("%s: fail to alloc buffer\n", __func__);

	return 0;
}
core_initcall(bootmsg_console_init);
