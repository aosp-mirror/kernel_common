/* SPDX-License-Identifier: MIT */
/*
 * Copyright (C) 2020 Google, Inc.
 *
 * Trusty and TF-A also have a copy of this header.
 * Please keep the copies in sync.
 */
#ifndef __LINUX_TRUSTY_ARM_FFA_H
#define __LINUX_TRUSTY_ARM_FFA_H

/*
 * Subset of Arm PSA Firmware Framework for Arm v8-A 1.0 EAC 1_0
 * (https://developer.arm.com/docs/den0077/a) needed for shared memory.
 */

#include "smcall.h"

#ifndef STATIC_ASSERT
#define STATIC_ASSERT(e) _Static_assert(e, #e)
#endif

#define FFA_CURRENT_VERSION_MAJOR (1U)
#define FFA_CURRENT_VERSION_MINOR (0U)

#define FFA_VERSION_TO_MAJOR(version) ((version) >> 16)
#define FFA_VERSION_TO_MINOR(version) ((version) & (0xffff))
#define FFA_VERSION(major, minor) (((major) << 16) | (minor))
#define FFA_CURRENT_VERSION \
	FFA_VERSION(FFA_CURRENT_VERSION_MAJOR, FFA_CURRENT_VERSION_MINOR)

#define SMC_ENTITY_SHARED_MEMORY 4

#define SMC_FASTCALL_NR_SHARED_MEMORY(nr) \
	SMC_FASTCALL_NR(SMC_ENTITY_SHARED_MEMORY, nr)
#define SMC_FASTCALL64_NR_SHARED_MEMORY(nr) \
	SMC_FASTCALL64_NR(SMC_ENTITY_SHARED_MEMORY, nr)

/**
 * typedef ffa_endpoint_id16_t - Endpoint ID
 *
 * Current implementation only supports VMIDs. FFA spec also support stream
 * endpoint ids.
 */
typedef uint16_t ffa_endpoint_id16_t;

/**
 * struct ffa_cons_mrd - Constituent memory region descriptor
 * @address:
 *         Start address of contiguous memory region. Must be 4K page aligned.
 * @page_count:
 *         Number of 4K pages in region.
 * @reserved_12_15:
 *         Reserve bytes 12-15 to pad struct size to 16 bytes.
 */
struct ffa_cons_mrd {
	uint64_t address;
	uint32_t page_count;
	uint32_t reserved_12_15;
};
STATIC_ASSERT(sizeof(struct ffa_cons_mrd) == 16);

/**
 * struct ffa_comp_mrd - Composite memory region descriptor
 * @total_page_count:
 *         Number of 4k pages in memory region. Must match sum of
 *         @address_range_array[].page_count.
 * @address_range_count:
 *         Number of entries in @address_range_array.
 * @reserved_8_15:
 *         Reserve bytes 8-15 to pad struct size to 16 byte alignment and
 *         make @address_range_array 16 byte aligned.
 * @address_range_array:
 *         Array of &struct ffa_cons_mrd entries.
 */
struct ffa_comp_mrd {
	uint32_t total_page_count;
	uint32_t address_range_count;
	uint64_t reserved_8_15;
	struct ffa_cons_mrd address_range_array[];
};
STATIC_ASSERT(sizeof(struct ffa_comp_mrd) == 16);

/**
 * typedef ffa_mem_attr8_t - Memory region attributes
 *
 * * @FFA_MEM_ATTR_DEVICE_NGNRNE:
 *     Device-nGnRnE.
 * * @FFA_MEM_ATTR_DEVICE_NGNRE:
 *     Device-nGnRE.
 * * @FFA_MEM_ATTR_DEVICE_NGRE:
 *     Device-nGRE.
 * * @FFA_MEM_ATTR_DEVICE_GRE:
 *     Device-GRE.
 * * @FFA_MEM_ATTR_NORMAL_MEMORY_UNCACHED
 *     Normal memory. Non-cacheable.
 * * @FFA_MEM_ATTR_NORMAL_MEMORY_CACHED_WB
 *     Normal memory. Write-back cached.
 * * @FFA_MEM_ATTR_NON_SHAREABLE
 *     Non-shareable. Combine with FFA_MEM_ATTR_NORMAL_MEMORY_*.
 * * @FFA_MEM_ATTR_OUTER_SHAREABLE
 *     Outer Shareable. Combine with FFA_MEM_ATTR_NORMAL_MEMORY_*.
 * * @FFA_MEM_ATTR_INNER_SHAREABLE
 *     Inner Shareable. Combine with FFA_MEM_ATTR_NORMAL_MEMORY_*.
 */
typedef uint8_t ffa_mem_attr8_t;
#define FFA_MEM_ATTR_DEVICE_NGNRNE ((1U << 4) | (0x0U << 2))
#define FFA_MEM_ATTR_DEVICE_NGNRE ((1U << 4) | (0x1U << 2))
#define FFA_MEM_ATTR_DEVICE_NGRE ((1U << 4) | (0x2U << 2))
#define FFA_MEM_ATTR_DEVICE_GRE ((1U << 4) | (0x3U << 2))
#define FFA_MEM_ATTR_NORMAL_MEMORY_UNCACHED ((2U << 4) | (0x1U << 2))
#define FFA_MEM_ATTR_NORMAL_MEMORY_CACHED_WB ((2U << 4) | (0x3U << 2))
#define FFA_MEM_ATTR_NON_SHAREABLE (0x0U << 0)
#define FFA_MEM_ATTR_OUTER_SHAREABLE (0x2U << 0)
#define FFA_MEM_ATTR_INNER_SHAREABLE (0x3U << 0)

/**
 * typedef ffa_mem_perm8_t - Memory access permissions
 *
 * * @FFA_MEM_ATTR_RO
 *     Request or specify read-only mapping.
 * * @FFA_MEM_ATTR_RW
 *     Request or allow read-write mapping.
 * * @FFA_MEM_PERM_NX
 *     Deny executable mapping.
 * * @FFA_MEM_PERM_X
 *     Request executable mapping.
 */
typedef uint8_t ffa_mem_perm8_t;
#define FFA_MEM_PERM_RO (1U << 0)
#define FFA_MEM_PERM_RW (1U << 1)
#define FFA_MEM_PERM_NX (1U << 2)
#define FFA_MEM_PERM_X (1U << 3)

/**
 * typedef ffa_mem_flag8_t - Endpoint memory flags
 *
 * * @FFA_MEM_FLAG_OTHER
 *     Other borrower. Memory region must not be or was not retrieved on behalf
 *     of this endpoint.
 */
typedef uint8_t ffa_mem_flag8_t;
#define FFA_MEM_FLAG_OTHER (1U << 0)

/**
 * typedef ffa_mtd_flag32_t - Memory transaction descriptor flags
 *
 * * @FFA_MTD_FLAG_ZERO_MEMORY
 *     Zero memory after unmapping from sender (must be 0 for share).
 * * @FFA_MTD_FLAG_TIME_SLICING
 *     Not supported by this implementation.
 * * @FFA_MTD_FLAG_ZERO_MEMORY_AFTER_RELINQUISH
 *     Zero memory after unmapping from borrowers (must be 0 for share).
 * * @FFA_MTD_FLAG_TYPE_MASK
 *     Bit-mask to extract memory management transaction type from flags.
 * * @FFA_MTD_FLAG_TYPE_SHARE_MEMORY
 *     Share memory transaction flag.
 *     Used by @SMC_FC_FFA_MEM_RETRIEVE_RESP to indicate that memory came from
 *     @SMC_FC_FFA_MEM_SHARE and by @SMC_FC_FFA_MEM_RETRIEVE_REQ to specify that
 *     it must have.
 * * @FFA_MTD_FLAG_ADDRESS_RANGE_ALIGNMENT_HINT_MASK
 *     Not supported by this implementation.
 */
typedef uint32_t ffa_mtd_flag32_t;
#define FFA_MTD_FLAG_ZERO_MEMORY (1U << 0)
#define FFA_MTD_FLAG_TIME_SLICING (1U << 1)
#define FFA_MTD_FLAG_ZERO_MEMORY_AFTER_RELINQUISH (1U << 2)
#define FFA_MTD_FLAG_TYPE_MASK (3U << 3)
#define FFA_MTD_FLAG_TYPE_SHARE_MEMORY (1U << 3)
#define FFA_MTD_FLAG_ADDRESS_RANGE_ALIGNMENT_HINT_MASK (0x1FU << 5)

/**
 * struct ffa_mapd - Memory access permissions descriptor
 * @endpoint_id:
 *         Endpoint id that @memory_access_permissions and @flags apply to.
 *         (&typedef ffa_endpoint_id16_t).
 * @memory_access_permissions:
 *         FFA_MEM_PERM_* values or'ed together (&typedef ffa_mem_perm8_t).
 * @flags:
 *         FFA_MEM_FLAG_* values or'ed together (&typedef ffa_mem_flag8_t).
 */
struct ffa_mapd {
	ffa_endpoint_id16_t endpoint_id;
	ffa_mem_perm8_t memory_access_permissions;
	ffa_mem_flag8_t flags;
};
STATIC_ASSERT(sizeof(struct ffa_mapd) == 4);

/**
 * struct ffa_emad - Endpoint memory access descriptor.
 * @mapd:  &struct ffa_mapd.
 * @comp_mrd_offset:
 *         Offset of &struct ffa_comp_mrd form start of &struct ffa_mtd.
 * @reserved_8_15:
 *         Reserved bytes 8-15. Must be 0.
 */
struct ffa_emad {
	struct ffa_mapd mapd;
	uint32_t comp_mrd_offset;
	uint64_t reserved_8_15;
};
STATIC_ASSERT(sizeof(struct ffa_emad) == 16);

/**
 * struct ffa_mtd - Memory transaction descriptor.
 * @sender_id:
 *         Sender endpoint id.
 * @memory_region_attributes:
 *         FFA_MEM_ATTR_* values or'ed together (&typedef ffa_mem_attr8_t).
 * @reserved_3:
 *         Reserved bytes 3. Must be 0.
 * @flags:
 *         FFA_MTD_FLAG_* values or'ed together (&typedef ffa_mtd_flag32_t).
 * @handle:
 *         Id of shared memory object. Most be 0 for MEM_SHARE.
 * @tag:   Client allocated tag. Must match original value.
 * @reserved_24_27:
 *         Reserved bytes 24-27. Must be 0.
 * @emad_count:
 *         Number of entries in @emad. Must be 1 in current implementation.
 *         FFA spec allows more entries.
 * @emad:
 *         Endpoint memory access descriptor array (see @struct ffa_emad).
 */
struct ffa_mtd {
	ffa_endpoint_id16_t sender_id;
	ffa_mem_attr8_t memory_region_attributes;
	uint8_t reserved_3;
	ffa_mtd_flag32_t flags;
	uint64_t handle;
	uint64_t tag;
	uint32_t reserved_24_27;
	uint32_t emad_count;
	struct ffa_emad emad[];
};
STATIC_ASSERT(sizeof(struct ffa_mtd) == 32);

/**
 * struct ffa_mem_relinquish_descriptor - Relinquish request descriptor.
 * @handle:
 *         Id of shared memory object to relinquish.
 * @flags:
 *         If bit 0 is set clear memory after unmapping from borrower. Must be 0
 *         for share. Bit[1]: Time slicing. Not supported, must be 0. All other
 *         bits are reserved 0.
 * @endpoint_count:
 *         Number of entries in @endpoint_array.
 * @endpoint_array:
 *         Array of endpoint ids.
 */
struct ffa_mem_relinquish_descriptor {
	uint64_t handle;
	uint32_t flags;
	uint32_t endpoint_count;
	ffa_endpoint_id16_t endpoint_array[];
};
STATIC_ASSERT(sizeof(struct ffa_mem_relinquish_descriptor) == 16);

/**
 * enum ffa_error - FF-A error code
 * @FFA_ERROR_NOT_SUPPORTED:
 *         Operation contained possibly valid parameters not supported by the
 *         current implementation. Does not match FF-A 1.0 EAC 1_0 definition.
 * @FFA_ERROR_INVALID_PARAMETERS:
 *         Invalid parameters. Conditions function specific.
 * @FFA_ERROR_NO_MEMORY:
 *         Not enough memory.
 * @FFA_ERROR_DENIED:
 *         Operation not allowed. Conditions function specific.
 *
 * FF-A 1.0 EAC 1_0 defines other error codes as well but the current
 * implementation does not use them.
 */
enum ffa_error {
	FFA_ERROR_NOT_SUPPORTED = -1,
	FFA_ERROR_INVALID_PARAMETERS = -2,
	FFA_ERROR_NO_MEMORY = -3,
	FFA_ERROR_DENIED = -6,
};

/**
 * SMC_FC32_FFA_MIN - First 32 bit SMC opcode reserved for FFA
 */
#define SMC_FC32_FFA_MIN SMC_FASTCALL_NR_SHARED_MEMORY(0x60)

/**
 * SMC_FC32_FFA_MAX - Last 32 bit SMC opcode reserved for FFA
 */
#define SMC_FC32_FFA_MAX SMC_FASTCALL_NR_SHARED_MEMORY(0x7F)

/**
 * SMC_FC64_FFA_MIN - First 64 bit SMC opcode reserved for FFA
 */
#define SMC_FC64_FFA_MIN SMC_FASTCALL64_NR_SHARED_MEMORY(0x60)

/**
 * SMC_FC64_FFA_MAX - Last 64 bit SMC opcode reserved for FFA
 */
#define SMC_FC64_FFA_MAX SMC_FASTCALL64_NR_SHARED_MEMORY(0x7F)

/**
 * SMC_FC_FFA_ERROR - SMC error return opcode
 *
 * Register arguments:
 *
 * * w1:     VMID in [31:16], vCPU in [15:0]
 * * w2:     Error code (&enum ffa_error)
 */
#define SMC_FC_FFA_ERROR SMC_FASTCALL_NR_SHARED_MEMORY(0x60)

/**
 * SMC_FC_FFA_SUCCESS - 32 bit SMC success return opcode
 *
 * Register arguments:
 *
 * * w1:     VMID in [31:16], vCPU in [15:0]
 * * w2-w7:  Function specific
 */
#define SMC_FC_FFA_SUCCESS SMC_FASTCALL_NR_SHARED_MEMORY(0x61)

/**
 * SMC_FC64_FFA_SUCCESS - 64 bit SMC success return opcode
 *
 * Register arguments:
 *
 * * w1:             VMID in [31:16], vCPU in [15:0]
 * * w2/x2-w7/x7:    Function specific
 */
#define SMC_FC64_FFA_SUCCESS SMC_FASTCALL64_NR_SHARED_MEMORY(0x61)

/**
 * SMC_FC_FFA_VERSION - SMC opcode to return supported FF-A version
 *
 * Register arguments:
 *
 * * w1:     Major version bit[30:16] and minor version in bit[15:0] supported
 *           by caller. Bit[31] must be 0.
 *
 * Return:
 * * w0:     &SMC_FC_FFA_SUCCESS
 * * w2:     Major version bit[30:16], minor version in bit[15:0], bit[31] must
 *           be 0.
 *
 * or
 *
 * * w0:     SMC_FC_FFA_ERROR
 * * w2:     FFA_ERROR_NOT_SUPPORTED if major version passed in is less than the
 *           minimum major version supported.
 */
#define SMC_FC_FFA_VERSION SMC_FASTCALL_NR_SHARED_MEMORY(0x63)

/**
 * SMC_FC_FFA_FEATURES - SMC opcode to check optional feature support
 *
 * Register arguments:
 *
 * * w1:     FF-A function ID
 *
 * Return:
 * * w0:     &SMC_FC_FFA_SUCCESS
 * * w2:     Bit[0]: Supports custom buffers for memory transactions.
 *           Bit[1:0]: For RXTX_MAP min buffer size and alignment boundary.
 *           Other bits must be 0.
 * * w3:     For FFA_MEM_RETRIEVE_REQ, bit[7-0]: Number of times receiver can
 *           retrieve each memory region before relinquishing it specified as
 *           ((1U << (value + 1)) - 1 (or value = bits in reference count - 1).
 *           For all other bits and commands: must be 0.
 * or
 *
 * * w0:     SMC_FC_FFA_ERROR
 * * w2:     FFA_ERROR_NOT_SUPPORTED if function is not implemented, or
 *           FFA_ERROR_INVALID_PARAMETERS if function id is not valid.
 */
#define SMC_FC_FFA_FEATURES SMC_FASTCALL_NR_SHARED_MEMORY(0x64)

/**
 * SMC_FC_FFA_RXTX_MAP - 32 bit SMC opcode to map message buffers
 *
 * Register arguments:
 *
 * * w1:     TX address
 * * w2:     RX address
 * * w3:     RX/TX page count in bit[5:0]
 *
 * Return:
 * * w0:     &SMC_FC_FFA_SUCCESS
 */
#define SMC_FC_FFA_RXTX_MAP SMC_FASTCALL_NR_SHARED_MEMORY(0x66)

/**
 * SMC_FC64_FFA_RXTX_MAP - 64 bit SMC opcode to map message buffers
 *
 * Register arguments:
 *
 * * x1:     TX address
 * * x2:     RX address
 * * x3:     RX/TX page count in bit[5:0]
 *
 * Return:
 * * w0:     &SMC_FC_FFA_SUCCESS
 */
#define SMC_FC64_FFA_RXTX_MAP SMC_FASTCALL64_NR_SHARED_MEMORY(0x66)
#ifdef CONFIG_64BIT
#define SMC_FCZ_FFA_RXTX_MAP SMC_FC64_FFA_RXTX_MAP
#else
#define SMC_FCZ_FFA_RXTX_MAP SMC_FC_FFA_RXTX_MAP
#endif

/**
 * SMC_FC_FFA_RXTX_UNMAP - SMC opcode to unmap message buffers
 *
 * Register arguments:
 *
 * * w1:     ID in [31:16]
 *
 * Return:
 * * w0:     &SMC_FC_FFA_SUCCESS
 */
#define SMC_FC_FFA_RXTX_UNMAP SMC_FASTCALL_NR_SHARED_MEMORY(0x67)

/**
 * SMC_FC_FFA_ID_GET - SMC opcode to get endpoint id of caller
 *
 * Return:
 * * w0:     &SMC_FC_FFA_SUCCESS
 * * w2:     ID in bit[15:0], bit[31:16] must be 0.
 */
#define SMC_FC_FFA_ID_GET SMC_FASTCALL_NR_SHARED_MEMORY(0x69)

/**
 * SMC_FC_FFA_MEM_DONATE - 32 bit SMC opcode to donate memory
 *
 * Not supported.
 */
#define SMC_FC_FFA_MEM_DONATE SMC_FASTCALL_NR_SHARED_MEMORY(0x71)

/**
 * SMC_FC_FFA_MEM_LEND - 32 bit SMC opcode to lend memory
 *
 * Not currently supported.
 */
#define SMC_FC_FFA_MEM_LEND SMC_FASTCALL_NR_SHARED_MEMORY(0x72)

/**
 * SMC_FC_FFA_MEM_SHARE - 32 bit SMC opcode to share memory
 *
 * Register arguments:
 *
 * * w1:     Total length
 * * w2:     Fragment length
 * * w3:     Address
 * * w4:     Page count
 *
 * Return:
 * * w0:     &SMC_FC_FFA_SUCCESS
 * * w2/w3:  Handle
 *
 * or
 *
 * * w0:     &SMC_FC_FFA_MEM_FRAG_RX
 * * w1-:    See &SMC_FC_FFA_MEM_FRAG_RX
 *
 * or
 *
 * * w0:     SMC_FC_FFA_ERROR
 * * w2:     Error code (&enum ffa_error)
 */
#define SMC_FC_FFA_MEM_SHARE SMC_FASTCALL_NR_SHARED_MEMORY(0x73)

/**
 * SMC_FC64_FFA_MEM_SHARE - 64 bit SMC opcode to share memory
 *
 * Register arguments:
 *
 * * w1:     Total length
 * * w2:     Fragment length
 * * x3:     Address
 * * w4:     Page count
 *
 * Return:
 * * w0:     &SMC_FC_FFA_SUCCESS
 * * w2/w3:  Handle
 *
 * or
 *
 * * w0:     &SMC_FC_FFA_MEM_FRAG_RX
 * * w1-:    See &SMC_FC_FFA_MEM_FRAG_RX
 *
 * or
 *
 * * w0:     SMC_FC_FFA_ERROR
 * * w2:     Error code (&enum ffa_error)
 */
#define SMC_FC64_FFA_MEM_SHARE SMC_FASTCALL64_NR_SHARED_MEMORY(0x73)

/**
 * SMC_FC_FFA_MEM_RETRIEVE_REQ - 32 bit SMC opcode to retrieve shared memory
 *
 * Register arguments:
 *
 * * w1:     Total length
 * * w2:     Fragment length
 * * w3:     Address
 * * w4:     Page count
 *
 * Return:
 * * w0:             &SMC_FC_FFA_MEM_RETRIEVE_RESP
 * * w1/x1-w5/x5:    See &SMC_FC_FFA_MEM_RETRIEVE_RESP
 */
#define SMC_FC_FFA_MEM_RETRIEVE_REQ SMC_FASTCALL_NR_SHARED_MEMORY(0x74)

/**
 * SMC_FC64_FFA_MEM_RETRIEVE_REQ - 64 bit SMC opcode to retrieve shared memory
 *
 * Register arguments:
 *
 * * w1:     Total length
 * * w2:     Fragment length
 * * x3:     Address
 * * w4:     Page count
 *
 * Return:
 * * w0:             &SMC_FC_FFA_MEM_RETRIEVE_RESP
 * * w1/x1-w5/x5:    See &SMC_FC_FFA_MEM_RETRIEVE_RESP
 */
#define SMC_FC64_FFA_MEM_RETRIEVE_REQ SMC_FASTCALL64_NR_SHARED_MEMORY(0x74)

/**
 * SMC_FC_FFA_MEM_RETRIEVE_RESP - Retrieve 32 bit SMC return opcode
 *
 * Register arguments:
 *
 * * w1:     Total length
 * * w2:     Fragment length
 */
#define SMC_FC_FFA_MEM_RETRIEVE_RESP SMC_FASTCALL_NR_SHARED_MEMORY(0x75)

/**
 * SMC_FC_FFA_MEM_RELINQUISH - SMC opcode to relinquish shared memory
 *
 * Input in &struct ffa_mem_relinquish_descriptor format in message buffer.
 *
 * Return:
 * * w0:     &SMC_FC_FFA_SUCCESS
 */
#define SMC_FC_FFA_MEM_RELINQUISH SMC_FASTCALL_NR_SHARED_MEMORY(0x76)

/**
 * SMC_FC_FFA_MEM_RECLAIM - SMC opcode to reclaim shared memory
 *
 * Register arguments:
 *
 * * w1/w2:  Handle
 * * w3:     Flags
 *
 * Return:
 * * w0:     &SMC_FC_FFA_SUCCESS
 */
#define SMC_FC_FFA_MEM_RECLAIM SMC_FASTCALL_NR_SHARED_MEMORY(0x77)

/**
 * SMC_FC_FFA_MEM_FRAG_RX - SMC opcode to request next fragment.
 *
 * Register arguments:
 *
 * * w1/w2:  Cookie
 * * w3:     Fragment offset.
 * * w4:     Endpoint id ID in [31:16], if client is hypervisor.
 *
 * Return:
 * * w0:             &SMC_FC_FFA_MEM_FRAG_TX
 * * w1/x1-w5/x5:    See &SMC_FC_FFA_MEM_FRAG_TX
 */
#define SMC_FC_FFA_MEM_FRAG_RX SMC_FASTCALL_NR_SHARED_MEMORY(0x7A)

/**
 * SMC_FC_FFA_MEM_FRAG_TX - SMC opcode to transmit next fragment
 *
 * Register arguments:
 *
 * * w1/w2:  Cookie
 * * w3:     Fragment length.
 * * w4:     Sender endpoint id ID in [31:16], if client is hypervisor.
 *
 * Return:
 * * w0:             &SMC_FC_FFA_MEM_FRAG_RX or &SMC_FC_FFA_SUCCESS.
 * * w1/x1-w5/x5:    See opcode in w0.
 */
#define SMC_FC_FFA_MEM_FRAG_TX SMC_FASTCALL_NR_SHARED_MEMORY(0x7B)

#endif /* __LINUX_TRUSTY_ARM_FFA_H */
