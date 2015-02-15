/*
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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
#ifndef __MSM_PIL_Q6V5_H
#define __MSM_PIL_Q6V5_H

#include "peripheral-loader.h"

struct regulator;
struct clk;
struct pil_device;
struct platform_device;

struct q6v5_data {
	void __iomem *reg_base;
	void __iomem *rmb_base;
	void __iomem *cxrail_bhs;  
	struct clk *xo;		   
	struct clk *ahb_clk;	   
	struct clk *axi_clk;	   
	struct clk *core_clk;	   
	struct clk *reg_clk;	   
	struct clk *rom_clk;	   
	void __iomem *axi_halt_base;
	void __iomem *restart_reg;
	struct regulator *vreg;
	struct regulator *vreg_cx;
	struct regulator *vreg_mx;
	struct regulator *vreg_pll;
	bool is_booted;
	struct pil_desc desc;
	bool self_auth;

	phys_addr_t *rmb_base_phys;
};

int pil_q6v5_make_proxy_votes(struct pil_desc *pil);
void pil_q6v5_remove_proxy_votes(struct pil_desc *pil);
void pil_q6v5_halt_axi_port(struct pil_desc *pil, void __iomem *halt_base);
void pil_q6v5_shutdown(struct pil_desc *pil);
int pil_q6v5_reset(struct pil_desc *pil);
struct q6v5_data *pil_q6v5_init(struct platform_device *pdev);

#endif
