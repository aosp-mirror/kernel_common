/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2023 Google, Inc.
 */

#if !defined(_TRUSTY_IRQ_H)
#define _TRUSTY_IRQ_H

#ifdef CONFIG_TRUSTY_IRQ
int __init trusty_irq_driver_init(void);
void trusty_irq_driver_exit(void);
#else
static inline int trusty_irq_driver_init(void) { return 0; }
static inline void trusty_irq_driver_exit(void) {}
#endif

#endif /* _TRUSTY_IRQ_H */
