/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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

#ifndef __MSM_ISPIF_HWREG_V2_H__
#define __MSM_ISPIF_HWREG_V2_H__

/* common registers */
#define ISPIF_RST_CMD_ADDR                       0x008
#define ISPIF_RST_CMD_1_ADDR                     0x00C
#define ISPIF_IRQ_GLOBAL_CLEAR_CMD_ADDR          0x01C
#define PIX0_LINE_BUF_EN_BIT                     6

#define ISPIF_VFE(m)                             ((m) * 0x200)

#define ISPIF_VFE_m_CTRL_0(m)                    (0x200 + ISPIF_VFE(m))
#define ISPIF_VFE_m_IRQ_MASK_0(m)                (0x208 + ISPIF_VFE(m))
#define ISPIF_VFE_m_IRQ_MASK_1(m)                (0x20C + ISPIF_VFE(m))
#define ISPIF_VFE_m_IRQ_MASK_2(m)                (0x210 + ISPIF_VFE(m))
#define ISPIF_VFE_m_IRQ_STATUS_0(m)              (0x21C + ISPIF_VFE(m))
#define ISPIF_VFE_m_IRQ_STATUS_1(m)              (0x220 + ISPIF_VFE(m))
#define ISPIF_VFE_m_IRQ_STATUS_2(m)              (0x224 + ISPIF_VFE(m))
#define ISPIF_VFE_m_IRQ_CLEAR_0(m)               (0x230 + ISPIF_VFE(m))
#define ISPIF_VFE_m_IRQ_CLEAR_1(m)               (0x234 + ISPIF_VFE(m))
#define ISPIF_VFE_m_IRQ_CLEAR_2(m)               (0x238 + ISPIF_VFE(m))
#define ISPIF_VFE_m_INPUT_SEL(m)                 (0x244 + ISPIF_VFE(m))
#define ISPIF_VFE_m_INTF_CMD_0(m)                (0x248 + ISPIF_VFE(m))
#define ISPIF_VFE_m_INTF_CMD_1(m)                (0x24C + ISPIF_VFE(m))
#define ISPIF_VFE_m_PIX_INTF_n_CID_MASK(m, n)    (0x254 + ISPIF_VFE(m) + 4*(n))
#define ISPIF_VFE_m_RDI_INTF_n_CID_MASK(m, n)    (0x264 + ISPIF_VFE(m) + 4*(n))
#define ISPIF_VFE_m_PIX_INTF_n_CROP(m, n)        (0x278 + ISPIF_VFE(m) + 4*(n))
#define ISPIF_VFE_m_3D_THRESHOLD(m)              (0x288 + ISPIF_VFE(m))
#define ISPIF_VFE_m_OUTPUT_SEL(m)                (0x28C + ISPIF_VFE(m))
#define ISPIF_VFE_m_PIX_OUTPUT_n_MISR(m, n)      (0x290 + ISPIF_VFE(m) + 4*(n))
#define ISPIF_VFE_m_RDI_OUTPUT_n_MISR_0(m, n)    (0x298 + ISPIF_VFE(m) + 8*(n))
#define ISPIF_VFE_m_RDI_OUTPUT_n_MISR_1(m, n)    (0x29C + ISPIF_VFE(m) + 8*(n))
#define ISPIF_VFE_m_PIX_INTF_n_STATUS(m, n)      (0x2C0 + ISPIF_VFE(m) + 4*(n))
#define ISPIF_VFE_m_RDI_INTF_n_STATUS(m, n)      (0x2D0 + ISPIF_VFE(m) + 4*(n))
#define ISPIF_VFE_m_3D_DESKEW_SIZE(m)            (0x2E4 + ISPIF_VFE(m))

/* CSID CLK MUX SEL REGISTERS */
#define ISPIF_RDI_CLK_MUX_SEL_ADDR               0x8

/*ISPIF RESET BITS*/
#define VFE_CLK_DOMAIN_RST                       BIT(31)
#define PIX_1_CLK_DOMAIN_RST                     BIT(30)
#define PIX_CLK_DOMAIN_RST                       BIT(29)
#define RDI_2_CLK_DOMAIN_RST                     BIT(28)
#define RDI_1_CLK_DOMAIN_RST                     BIT(27)
#define RDI_CLK_DOMAIN_RST                       BIT(26)
#define AHB_CLK_DOMAIN_RST                       BIT(25)
#define RDI_2_VFE_RST_STB                        BIT(12)
#define RDI_2_CSID_RST_STB                       BIT(11)
#define RDI_1_VFE_RST_STB                        BIT(10)
#define RDI_1_CSID_RST_STB                       BIT(9)
#define RDI_0_VFE_RST_STB                        BIT(8)
#define RDI_0_CSID_RST_STB                       BIT(7)
#define PIX_1_VFE_RST_STB                        BIT(6)
#define PIX_1_CSID_RST_STB                       BIT(5)
#define PIX_0_VFE_RST_STB                        BIT(4)
#define PIX_0_CSID_RST_STB                       BIT(3)
#define SW_REG_RST_STB                           BIT(2)
#define MISC_LOGIC_RST_STB                       BIT(1)
#define STROBED_RST_EN                           BIT(0)

#define ISPIF_RST_CMD_MASK                       0xFE0F1FFF
#define ISPIF_RST_CMD_1_MASK                     0xFC0F1FF9

#define PIX_INTF_0_OVERFLOW_IRQ                  BIT(12)
#define RAW_INTF_0_OVERFLOW_IRQ                  BIT(25)
#define RAW_INTF_1_OVERFLOW_IRQ                  BIT(25)
#define RAW_INTF_2_OVERFLOW_IRQ                  BIT(12)
#define RESET_DONE_IRQ                           BIT(27)

#define ISPIF_IRQ_STATUS_MASK                    0x0A493249
#define ISPIF_IRQ_STATUS_1_MASK                  0x02493249
#define ISPIF_IRQ_STATUS_2_MASK                  0x00001249

#define ISPIF_IRQ_STATUS_PIX_SOF_MASK            0x249
#define ISPIF_IRQ_STATUS_RDI0_SOF_MASK           0x492000
#define ISPIF_IRQ_STATUS_RDI1_SOF_MASK           0x492000
#define ISPIF_IRQ_STATUS_RDI2_SOF_MASK           0x249

#define ISPIF_IRQ_GLOBAL_CLEAR_CMD               0x1

#define ISPIF_STOP_INTF_IMMEDIATELY              0xAAAAAAAA

#endif /* __MSM_ISPIF_HWREG_V2_H__ */
