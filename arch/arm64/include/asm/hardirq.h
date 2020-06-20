/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2012 ARM Ltd.
 */
#ifndef __ASM_HARDIRQ_H
#define __ASM_HARDIRQ_H

#include <linux/cache.h>
#include <linux/percpu.h>
#include <linux/threads.h>
#include <asm/barrier.h>
#include <asm/irq.h>
#include <asm/kvm_arm.h>
#include <asm/sysreg.h>

typedef struct {
	unsigned int __softirq_pending;
} ____cacheline_aligned irq_cpustat_t;

#include <linux/irq_cpustat.h>	/* Standard mappings for irq_cpustat_t above */

#define __ARCH_IRQ_EXIT_IRQS_DISABLED	1

struct nmi_ctx {
	u64 hcr;
};

DECLARE_PER_CPU(struct nmi_ctx, nmi_contexts);

#define arch_nmi_enter()							\
	do {									\
		if (is_kernel_in_hyp_mode()) {					\
			struct nmi_ctx *nmi_ctx = this_cpu_ptr(&nmi_contexts);	\
			nmi_ctx->hcr = read_sysreg(hcr_el2);			\
			if (!(nmi_ctx->hcr & HCR_TGE)) {			\
				write_sysreg(nmi_ctx->hcr | HCR_TGE, hcr_el2);	\
				isb();						\
			}							\
		}								\
	} while (0)

#define arch_nmi_exit()								\
	do {									\
		if (is_kernel_in_hyp_mode()) {					\
			struct nmi_ctx *nmi_ctx = this_cpu_ptr(&nmi_contexts);	\
			if (!(nmi_ctx->hcr & HCR_TGE))				\
				write_sysreg(nmi_ctx->hcr, hcr_el2);		\
		}								\
	} while (0)

static inline void ack_bad_irq(unsigned int irq)
{
	extern unsigned long irq_err_count;
	irq_err_count++;
}

#endif /* __ASM_HARDIRQ_H */
