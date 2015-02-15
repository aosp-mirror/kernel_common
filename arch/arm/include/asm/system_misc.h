#ifndef __ASM_ARM_SYSTEM_MISC_H
#define __ASM_ARM_SYSTEM_MISC_H

#ifndef __ASSEMBLY__

#include <linux/compiler.h>
#include <linux/linkage.h>
#include <linux/irqflags.h>

extern void cpu_init(void);

void soft_restart(unsigned long);
extern void (*arm_pm_restart)(char str, const char *cmd);
extern void (*arm_pm_idle)(void);

#define UDBG_UNDEFINED	(1 << 0)
#define UDBG_SYSCALL	(1 << 1)
#define UDBG_BADABORT	(1 << 2)
#define UDBG_SEGV	(1 << 3)
#define UDBG_BUS	(1 << 4)

extern unsigned int user_debug;

extern void disable_hlt(void);
extern void enable_hlt(void);
extern int get_hlt(void);
extern char* (*arch_read_hardware_id)(void);

#endif /* !__ASSEMBLY__ */

#endif /* __ASM_ARM_SYSTEM_MISC_H */
