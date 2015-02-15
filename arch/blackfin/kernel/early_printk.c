/*
 * allow a console to be used for early printk
 * derived from arch/x86/kernel/early_printk.c
 *
 * Copyright 2007-2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/console.h>
#include <linux/string.h>
#include <linux/reboot.h>
#include <asm/blackfin.h>
#include <asm/irq_handler.h>
#include <asm/early_printk.h>

#ifdef CONFIG_SERIAL_BFIN
extern struct console *bfin_earlyserial_init(unsigned int port,
						unsigned int cflag);
#endif
#ifdef CONFIG_BFIN_JTAG_COMM
extern struct console *bfin_jc_early_init(void);
#endif

static struct console *early_console;

#define DEFAULT_PORT 0
#define DEFAULT_CFLAG CS8|B57600

#define DEFAULT_EARLY_PORT "serial,uart0,57600"

#ifdef CONFIG_SERIAL_CORE
static struct console * __init earlyserial_init(char *buf)
{
	int baud, bit;
	char parity;
	unsigned int serial_port = DEFAULT_PORT;
	unsigned int cflag = DEFAULT_CFLAG;

	serial_port = simple_strtoul(buf, &buf, 10);
	buf++;

	cflag = 0;
	baud = simple_strtoul(buf, &buf, 10);
	switch (baud) {
	case 1200:
		cflag |= B1200;
		break;
	case 2400:
		cflag |= B2400;
		break;
	case 4800:
		cflag |= B4800;
		break;
	case 9600:
		cflag |= B9600;
		break;
	case 19200:
		cflag |= B19200;
		break;
	case 38400:
		cflag |= B38400;
		break;
	case 115200:
		cflag |= B115200;
		break;
	default:
		cflag |= B57600;
	}

	parity = buf[0];
	buf++;
	switch (parity) {
	case 'e':
		cflag |= PARENB;
		break;
	case 'o':
		cflag |= PARODD;
		break;
	}

	bit = simple_strtoul(buf, &buf, 10);
	switch (bit) {
	case 5:
		cflag |= CS5;
		break;
	case 6:
		cflag |= CS6;
		break;
	case 7:
		cflag |= CS7;
		break;
	default:
		cflag |= CS8;
	}

#ifdef CONFIG_SERIAL_BFIN
	return bfin_earlyserial_init(serial_port, cflag);
#else
	return NULL;
#endif

}
#endif

int __init setup_early_printk(char *buf)
{

	if (!buf)
		return 0;

	if (!*buf)
		return 0;

	if (early_console != NULL)
		return 0;

#ifdef CONFIG_SERIAL_BFIN
	
	if (!strncmp(buf, "serial,uart", 11)) {
		buf += 11;
		early_console = earlyserial_init(buf);
	}
#endif

#ifdef CONFIG_BFIN_JTAG_COMM
	
	if (!strncmp(buf, "jtag", 4)) {
		buf += 4;
		early_console = bfin_jc_early_init();
	}
#endif

#ifdef CONFIG_FB
		
#endif

	if (likely(early_console)) {
		early_console->flags |= CON_BOOT;

		register_console(early_console);
		printk(KERN_INFO "early printk enabled on %s%d\n",
			early_console->name,
			early_console->index);
	}

	return 0;
}


asmlinkage void __init init_early_exception_vectors(void)
{
	u32 evt;
	SSYNC();

	mark_shadow_error();
	early_shadow_puts(linux_banner);
	early_shadow_stamp();

	if (CPUID != bfin_cpuid()) {
		early_shadow_puts("Running on wrong machine type, expected");
		early_shadow_reg(CPUID, 16);
		early_shadow_puts(", but running on");
		early_shadow_reg(bfin_cpuid(), 16);
		early_shadow_puts("\n");
	}

	for (evt = EVT2; evt <= EVT15; evt += 4)
		bfin_write32(evt, early_trap);
	CSYNC();

	asm("\tRETI = %0; RETX = %0; RETN = %0;\n"
		: : "p"(early_trap));

}

__attribute__((__noreturn__))
asmlinkage void __init early_trap_c(struct pt_regs *fp, void *retaddr)
{
	if (likely(early_console == NULL) && CPUID == bfin_cpuid())
		setup_early_printk(DEFAULT_EARLY_PORT);

	if (!shadow_console_enabled()) {
		
		early_shadow_puts("panic before setup_arch\n");
		early_shadow_puts("IPEND:");
		early_shadow_reg(fp->ipend, 16);
		if (fp->seqstat & SEQSTAT_EXCAUSE) {
			early_shadow_puts("\nEXCAUSE:");
			early_shadow_reg(fp->seqstat & SEQSTAT_EXCAUSE, 8);
		}
		if (fp->seqstat & SEQSTAT_HWERRCAUSE) {
			early_shadow_puts("\nHWERRCAUSE:");
			early_shadow_reg(
				(fp->seqstat & SEQSTAT_HWERRCAUSE) >> 14, 8);
		}
		early_shadow_puts("\nErr @");
		if (fp->ipend & EVT_EVX)
			early_shadow_reg(fp->retx, 32);
		else
			early_shadow_reg(fp->pc, 32);
#ifdef CONFIG_DEBUG_BFIN_HWTRACE_ON
		early_shadow_puts("\nTrace:");
		if (likely(bfin_read_TBUFSTAT() & TBUFCNT)) {
			while (bfin_read_TBUFSTAT() & TBUFCNT) {
				early_shadow_puts("\nT  :");
				early_shadow_reg(bfin_read_TBUF(), 32);
				early_shadow_puts("\n S :");
				early_shadow_reg(bfin_read_TBUF(), 32);
			}
		}
#endif
		early_shadow_puts("\nUse bfin-elf-addr2line to determine "
			"function names\n");
		if (CPUID == bfin_cpuid()) {
			early_shadow_puts("Trying to restart\n");
			machine_restart("");
		}

		early_shadow_puts("Halting, since it is not safe to restart\n");
		while (1)
			asm volatile ("EMUEXCPT; IDLE;\n");

	} else {
		printk(KERN_EMERG "Early panic\n");
		show_regs(fp);
		dump_bfin_trace_buffer();
	}

	panic("Died early");
}

early_param("earlyprintk", setup_early_printk);
