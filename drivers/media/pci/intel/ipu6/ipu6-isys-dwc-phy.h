/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2013 - 2022 Intel Corporation
 */

#ifndef IPU6_ISYS_DWC_PHY_H
#define IPU6_ISYS_DWC_PHY_H

int ipu6_isys_dwc_phy_powerup_ack(struct ipu_isys *isys, u32 phy_id);
int ipu6_isys_dwc_phy_config(struct ipu_isys *isys, u32 phy_id, u32 mbps);
int ipu6_isys_dwc_phy_termcal_rext(struct ipu_isys *isys, u32 mbps);
void ipu6_isys_dwc_phy_reset(struct ipu_isys *isys, u32 phy_id);
void ipu6_isys_dwc_phy_aggr_setup(struct ipu_isys *isys, u32 master, u32 slave,
				  u32 mbps);
#endif
