/* Copyright (c) 2010-2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __MSM_PERIPHERAL_LOADER_H
#define __MSM_PERIPHERAL_LOADER_H

struct device;
struct module;
struct pil_priv;

struct pil_desc {
	const char *name;
	struct device *dev;
	const struct pil_reset_ops *ops;
	struct module *owner;
	unsigned long proxy_timeout;
	unsigned long flags;
#define PIL_SKIP_ENTRY_CHECK	BIT(0)
	struct pil_priv *priv;
	unsigned int proxy_unvote_irq;
};

struct pil_reset_ops {
	int (*init_image)(struct pil_desc *pil, const u8 *metadata,
			  size_t size);
	int (*mem_setup)(struct pil_desc *pil, phys_addr_t addr, size_t size);
	int (*verify_blob)(struct pil_desc *pil, phys_addr_t phy_addr,
			   size_t size);
	int (*proxy_vote)(struct pil_desc *pil);
	int (*auth_and_reset)(struct pil_desc *pil);
	void (*proxy_unvote)(struct pil_desc *pil);
	int (*shutdown)(struct pil_desc *pil);
};

#ifdef CONFIG_MSM_PIL
extern int pil_desc_init(struct pil_desc *desc);
extern int pil_boot(struct pil_desc *desc);
extern void pil_shutdown(struct pil_desc *desc);
extern void pil_desc_release(struct pil_desc *desc);
extern phys_addr_t pil_get_entry_addr(struct pil_desc *desc);
extern int pil_do_ramdump(struct pil_desc *desc, void *ramdump_dev);
#else
static inline int pil_desc_init(struct pil_desc *desc) { return 0; }
static inline int pil_boot(struct pil_desc *desc) { return 0; }
static inline void pil_shutdown(struct pil_desc *desc) { }
static inline void pil_desc_release(struct pil_desc *desc) { }
static inline phys_addr_t pil_get_entry_addr(struct pil_desc *desc)
{
	return 0;
}
static inline int pil_do_ramdump(struct pil_desc *desc, void *ramdump_dev)
{
	return 0;
}
#endif

#endif
