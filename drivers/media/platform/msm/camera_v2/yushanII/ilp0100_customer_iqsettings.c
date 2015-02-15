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
File Name:		ilp0100_customer_iq_settings.c
Author:			author name
Description: 	definition of structures defined for IQ parameters
********************************************************************************/
/*!
 * \file	ilp0100_customer_iq_settings.c
 * \brief	definition of structures defined for IQ parameters
 * \author	sheena jain
 */

#include "ilp0100_customer_iq_settings.h"


const Ilp0100_structHdrMergeParams HdrMergePresetLinearAndMacro={
		.Method 	= USE_AVERAGE,
		.ImageCodes	= MAX_MACRO_PIXEL
};

const Ilp0100_structHdrMergeParams HdrMergePresetLinearAndLuma={
		.Method 	= USE_AVERAGE,
		.ImageCodes	= LUMA
};

const Ilp0100_structHdrMergeParams HdrMergePresetLinearPlusMergeAndMacro={
		.Method 	= USE_AVERAGE_AND_KNEE_POINTS,
		.ImageCodes	= MAX_MACRO_PIXEL
};

const Ilp0100_structHdrMergeParams HdrMergePresetLinearPlusMergeAndLuma={
		.Method 	= USE_AVERAGE_AND_KNEE_POINTS,
		.ImageCodes	= LUMA
};
