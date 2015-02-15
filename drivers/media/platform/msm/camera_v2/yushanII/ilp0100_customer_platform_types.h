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
********************************************************************************/
/*!
 * \file ilp0100_customer_platform_types.h
 * \brief definition of types required by api but which are platform dependant
 * \author sheena jain
 */

#ifndef ILP0100_CUSTOMER_PLATFORM_TYPES_H_
#define ILP0100_CUSTOMER_PLATFORM_TYPES_H_

/* API requires that bool_t, uint8_t, int8_t, uint16_t, uint32_t are defined*/

/*  By example for linux kernel */
/* #include <linux/types.h> */
//! Define LINUX_TEST if to be tested on UNIX
//#define LINUX_TEST
/* for other platforms or platform which do not define them*/
#ifdef ST_SPECIFIC
typedef unsigned char bool_t;
typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef short	int16_t;
typedef int	int32_t;
typedef float float_t;
#else/*hTC*/
typedef unsigned char bool_t;
typedef float float_t;
#endif /*ST_SPECIFIC*/






#endif /* ILP0100_CUSTOMER_PLATFORM_TYPES_H_ */
