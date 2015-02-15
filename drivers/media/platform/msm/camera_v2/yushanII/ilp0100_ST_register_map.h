/*******************************************************************************
################################################################################
#                             (C) STMicroelectronics 2012
#
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License version 2 and only version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
#------------------------------------------------------------------------------
#                             Imaging Division
################################################################################
File Name:		ilp0100_ST_register_map.h
Author:			author name
Description:	List of Ilp0100 register
********************************************************************************/
/*!
 * \file	ilp0100_ST_register_map.h
 * \brief	List of Ilp0100 register
 * \author	sheena jain
 */

#ifndef ILP0100_ST_REGISTER_MAP_H_
#define ILP0100_ST_REGISTER_MAP_H_



#define ILP0100_ITM_ISP2HOST_0_STATUS					0x8001A8

#define ILP0100_ITM_ISP2HOST_1_STATUS					0x8001C0

#define ILP0100_ITM_ISP2HOST_0_EN_STATUS				0x8001AC

#define ILP0100_ITM_ISP2HOST_1_EN_STATUS				0x8001C4

#define ILP0100_ITM_ISP2HOST_0_STATUS_BCLR				0x8001B0

#define ILP0100_ITM_ISP2HOST_0_STATUS_BSET				0x8001B4

#define ILP0100_ITM_ISP2HOST_1_STATUS_BSET				0x8001CC

#define ILP0100_ITM_ISP2HOST_0_EN_STATUS_BCLR			0x8001B8

#define ILP0100_ITM_ISP2HOST_1_EN_STATUS_BCLR			0x8001D0

#define ILP0100_ITM_ISP2HOST_0_EN_STATUS_BSET			0x8001BC

#define ILP0100_ITM_ISP2HOST_1_EN_STATUS_BSET			0x8001D4

#define ILP0100_ITM_ISP2HOST_1_STATUS_BCLR				0x8001C8

#define	ILP0100_CLK_DIV_FACTOR							0x800320

#define	ILP0100_CLK_DIV_FACTOR_2						0x800324

#define ILP0100_CLK_CTRL           						0x800328

#define ILP0100_RESET_CTRL                 				0x80032C

#define ILP0100_PLL_CTRL_MAIN              				0x800330

#define ILP0100_PLL_LOOP_OUT_DF            				0x800334

#define ILP0100_HOST_IF_SPI_BASE_ADDRESS_0				0x800308

#define ILP0100_HOST_IF_SPI_BASE_ADDRESS_1				0x80030C

#define ILP0100_PERIPH_MCU_CTRL 						0x800000

#define ILP0100_LONG_EXP_HIST_DARKEST_G					0x8008A0

#define ILP0100_SHORT_EXP_HIST_DARKEST_G				0x800CA0

#define ILP0100_LONG_EXP_GLACE_STATS_PAGE				0x900000

#define ILP0100_LONG_EXP_HIST_STATS_PAGE				0x900200

#define ILP0100_LONG_EXP_GLACE_STATS_PIX_MEAN_SAT		0x900004

#define ILP0100_LONG_EXP_HIST_STATS_GREEN_BIN			0x900204

#define ILP0100_LONG_EXP_HIST_STATS_RED_BIN				0x900304

#define ILP0100_LONG_EXP_HIST_STATS_BLUE_BIN			0x900404

#define ILP0100_SHORT_EXP_GLACE_STATS_PAGE				0x901000

#define ILP0100_SHORT_EXP_HIST_STATS_PAGE				0x901200

#define ILP0100_SHORT_EXP_GLACE_STATS_PIX_MEAN_SAT		0x901004

#define ILP0100_SHORT_EXP_HIST_STATS_GREEN_BIN			0x901204

#define ILP0100_SHORT_EXP_HIST_STATS_RED_BIN			0x901304

#define ILP0100_SHORT_EXP_HIST_STATS_BLUE_BIN			0x901404

#define ILP0100_CSI2TX_FRAME_NO_0						0x801430

#endif /* ILP0100_ST_REGISTER_MAP_H_ */
