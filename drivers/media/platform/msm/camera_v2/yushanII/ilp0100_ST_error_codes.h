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
 * \file	ilp0100_ST_error_codes.h
 * \brief	This file lists all teh error codes
 * \author	sheena jain
 */

#ifndef ILP0100_ST_ERROR_CODES_H_
#define ILP0100_ST_ERROR_CODES_H_

//#define   ILP0100_ERROR_NONE                    ((Ilp0100_error) 0x0000) /*!< No error             .                                     */
#define		ILP0100_ERROR_NONE						0
#define 	ILP0100_ERROR							1

#define 	ILP_0100_DEBUGGING_ERROR_BASE			0x7000
#define		ILP_0100_DEBUG_SESSION_NOT_OPENED		ILP_0100_DEBUGGING_ERROR_BASE+0x000
#define		ILP_0100_DEBUG_BUFFER_CREATION_ERROR	ILP_0100_DEBUGGING_ERROR_BASE+0x001
#define		ILP_0100_DEBUG_NOT_VALID_BUFFER_ERROR	ILP_0100_DEBUGGING_ERROR_BASE+0x100

#endif /* ILP0100_ST_ERROR_CODES_H_ */
