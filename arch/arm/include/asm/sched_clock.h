/*
 * sched_clock.h: support for extending counters to full 64-bit ns counter
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef ASM_SCHED_CLOCK
#define ASM_SCHED_CLOCK

extern void sched_clock_postinit(void);
extern void setup_sched_clock(u32 (*read)(void), int bits, unsigned long rate);

#endif
