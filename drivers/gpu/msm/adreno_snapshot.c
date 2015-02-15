/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#include "kgsl.h"
#include "kgsl_sharedmem.h"
#include "kgsl_snapshot.h"

#include "adreno.h"
#include "adreno_pm4types.h"
#include "a2xx_reg.h"
#include "a3xx_reg.h"

/* Number of dwords of ringbuffer history to record */
#define NUM_DWORDS_OF_RINGBUFFER_HISTORY 100

/* Maintain a list of the objects we see during parsing */

#define SNAPSHOT_OBJ_BUFSIZE 64

#define SNAPSHOT_OBJ_TYPE_IB 0

/* Keep track of how many bytes are frozen after a snapshot and tell the user */
static int snapshot_frozen_objsize;

static struct kgsl_snapshot_obj {
	int type;
	uint32_t gpuaddr;
	phys_addr_t ptbase;
	void *ptr;
	int dwords;
	struct kgsl_mem_entry *entry;
} objbuf[SNAPSHOT_OBJ_BUFSIZE];

/* Pointer to the next open entry in the object list */
static int objbufptr;

/* Push a new buffer object onto the list */
static void push_object(struct kgsl_device *device, int type,
	phys_addr_t ptbase,
	uint32_t gpuaddr, int dwords)
{
	int index;
	void *ptr;
	struct kgsl_mem_entry *entry = NULL;

	/*
	 * Sometimes IBs can be reused in the same dump.  Because we parse from
	 * oldest to newest, if we come across an IB that has already been used,
	 * assume that it has been reused and update the list with the newest
	 * size.
	 */

	for (index = 0; index < objbufptr; index++) {
		if (objbuf[index].gpuaddr == gpuaddr &&
			objbuf[index].ptbase == ptbase) {
				objbuf[index].dwords = dwords;
				return;
			}
	}

	if (objbufptr == SNAPSHOT_OBJ_BUFSIZE) {
		KGSL_DRV_ERR(device, "snapshot: too many snapshot objects\n");
		return;
	}

	/*
	 * adreno_convertaddr verifies that the IB size is valid - at least in
	 * the context of it being smaller then the allocated memory space
	 */
	ptr = adreno_convertaddr(device, ptbase, gpuaddr, dwords << 2, &entry);

	if (ptr == NULL) {
		KGSL_DRV_ERR(device,
			"snapshot: Can't find GPU address for %x\n", gpuaddr);
		return;
	}

	/* Put it on the list of things to parse */
	objbuf[objbufptr].type = type;
	objbuf[objbufptr].gpuaddr = gpuaddr;
	objbuf[objbufptr].ptbase = ptbase;
	objbuf[objbufptr].dwords = dwords;
	objbuf[objbufptr].entry = entry;
	objbuf[objbufptr++].ptr = ptr;
}

/*
 * Return a 1 if the specified object is already on the list of buffers
 * to be dumped
 */

static int find_object(int type, unsigned int gpuaddr, phys_addr_t ptbase)
{
	int index;

	for (index = 0; index < objbufptr; index++) {
		if (objbuf[index].gpuaddr == gpuaddr &&
			objbuf[index].ptbase == ptbase &&
			objbuf[index].type == type)
			return 1;
	}

	return 0;
}

/*
 * This structure keeps track of type0 writes to VSC_PIPE_DATA_ADDRESS_x and
 * VSC_PIPE_DATA_LENGTH_x. When a draw initator is called these registers
 * point to buffers that we need to freeze for a snapshot
 */

static struct {
	unsigned int base;
	unsigned int size;
} vsc_pipe[8];

/*
 * This is the cached value of type0 writes to the VSC_SIZE_ADDRESS which
 * contains the buffer address of the visiblity stream size buffer during a
 * binning pass
 */

static unsigned int vsc_size_address;

/*
 * This struct keeps track of type0 writes to VFD_FETCH_INSTR_0_X and
 * VFD_FETCH_INSTR_1_X registers. When a draw initator is called the addresses
 * and sizes in these registers point to VBOs that we need to freeze for a
 * snapshot
 */

static struct {
	unsigned int base;
	unsigned int stride;
} vbo[16];

/*
 * This is the cached value of type0 writes to VFD_INDEX_MAX.  This will be used
 * to calculate the size of the VBOs when the draw initator is called
 */

static unsigned int vfd_index_max;

/*
 * This is the cached value of type0 writes to VFD_CONTROL_0 which tells us how
 * many VBOs are active when the draw initator is called
 */

static unsigned int vfd_control_0;

/*
 * Cached value of type0 writes to SP_VS_PVT_MEM_ADDR and SP_FS_PVT_MEM_ADDR.
 * This is a buffer that contains private stack information for the shader
 */

static unsigned int sp_vs_pvt_mem_addr;
static unsigned int sp_fs_pvt_mem_addr;

/*
 * Cached value of SP_VS_OBJ_START_REG and SP_FS_OBJ_START_REG.
 */
static unsigned int sp_vs_obj_start_reg;
static unsigned int sp_fs_obj_start_reg;

/*
 * Each load state block has two possible types.  Each type has a different
 * number of dwords per unit.  Use this handy lookup table to make sure
 * we dump the right amount of data from the indirect buffer
 */

static int load_state_unit_sizes[7][2] = {
	{ 2, 4 },
	{ 0, 1 },
	{ 2, 4 },
	{ 0, 1 },
	{ 8, 2 },
	{ 8, 2 },
	{ 8, 2 },
};

static int ib_parse_load_state(struct kgsl_device *device, unsigned int *pkt,
	phys_addr_t ptbase)
{
	unsigned int block, source, type;
	int ret = 0;

	/*
	 * The object here is to find indirect shaders i.e - shaders loaded from
	 * GPU memory instead of directly in the command.  These should be added
	 * to the list of memory objects to dump. So look at the load state
	 * if the block is indirect (source = 4). If so then add the memory
	 * address to the list.  The size of the object differs depending on the
	 * type per the load_state_unit_sizes array above.
	 */

	if (type3_pkt_size(pkt[0]) < 2)
		return 0;

	/*
	 * pkt[1] 18:16 - source
	 * pkt[1] 21:19 - state block
	 * pkt[1] 31:22 - size in units
	 * pkt[2] 0:1 - type
	 * pkt[2] 31:2 - GPU memory address
	 */

	block = (pkt[1] >> 19) & 0x07;
	source = (pkt[1] >> 16) & 0x07;
	type = pkt[2] & 0x03;

	if (source == 4) {
		int unitsize, ret;

		if (type == 0)
			unitsize = load_state_unit_sizes[block][0];
		else
			unitsize = load_state_unit_sizes[block][1];

		/* Freeze the GPU buffer containing the shader */

		ret = kgsl_snapshot_get_object(device, ptbase,
				pkt[2] & 0xFFFFFFFC,
				(((pkt[1] >> 22) & 0x03FF) * unitsize) << 2,
				SNAPSHOT_GPU_OBJECT_SHADER);

		if (ret < 0)
			return -EINVAL;

		snapshot_frozen_objsize += ret;
	}

	return ret;
}

/*
 * This opcode sets the base addresses for the visibilty stream buffer and the
 * visiblity stream size buffer.
 */

static int ib_parse_set_bin_data(struct kgsl_device *device, unsigned int *pkt,
	phys_addr_t ptbase)
{
	int ret;

	if (type3_pkt_size(pkt[0]) < 2)
		return 0;

	/* Visiblity stream buffer */
	ret = kgsl_snapshot_get_object(device, ptbase, pkt[1], 0,
			SNAPSHOT_GPU_OBJECT_GENERIC);

	if (ret < 0)
		return -EINVAL;

	snapshot_frozen_objsize += ret;

	/* visiblity stream size buffer (fixed size 8 dwords) */
	ret = kgsl_snapshot_get_object(device, ptbase, pkt[2], 32,
			SNAPSHOT_GPU_OBJECT_GENERIC);

	if (ret >= 0)
		snapshot_frozen_objsize += ret;

	return ret;
}

/*
 * This opcode writes to GPU memory - if the buffer is written to, there is a
 * good chance that it would be valuable to capture in the snapshot, so mark all
 * buffers that are written to as frozen
 */

static int ib_parse_mem_write(struct kgsl_device *device, unsigned int *pkt,
	phys_addr_t ptbase)
{
	int ret;

	if (type3_pkt_size(pkt[0]) < 1)
		return 0;

	/*
	 * The address is where the data in the rest of this packet is written
	 * to, but since that might be an offset into the larger buffer we need
	 * to get the whole thing. Pass a size of 0 kgsl_snapshot_get_object to
	 * capture the entire buffer.
	 */

	ret = kgsl_snapshot_get_object(device, ptbase, pkt[1] & 0xFFFFFFFC, 0,
		SNAPSHOT_GPU_OBJECT_GENERIC);

	if (ret >= 0)
		snapshot_frozen_objsize += ret;

	return ret;
}

/*
 * The DRAW_INDX opcode sends a draw initator which starts a draw operation in
 * the GPU, so this is the point where all the registers and buffers become
 * "valid".  The DRAW_INDX may also have an index buffer pointer that should be
 * frozen with the others
 */

static int ib_parse_draw_indx(struct kgsl_device *device, unsigned int *pkt,
	phys_addr_t ptbase)
{
	int ret = 0, i;

	if (type3_pkt_size(pkt[0]) < 3)
		return 0;

	/*  DRAW_IDX may have a index buffer pointer */

	if (type3_pkt_size(pkt[0]) > 3) {
		ret = kgsl_snapshot_get_object(device, ptbase, pkt[4], pkt[5],
			SNAPSHOT_GPU_OBJECT_GENERIC);
		if (ret < 0)
			return -EINVAL;

		snapshot_frozen_objsize += ret;
	}

	/*
	 * All of the type0 writes are valid at a draw initiator, so freeze
	 * the various buffers that we are tracking
	 */

	/* First up the visiblity stream buffer */

	for (i = 0; i < ARRAY_SIZE(vsc_pipe); i++) {
		if (vsc_pipe[i].base != 0 && vsc_pipe[i].size != 0) {
			ret = kgsl_snapshot_get_object(device, ptbase,
				vsc_pipe[i].base, vsc_pipe[i].size,
				SNAPSHOT_GPU_OBJECT_GENERIC);
			if (ret < 0)
				return -EINVAL;

			snapshot_frozen_objsize += ret;
		}
	}

	/* Next the visibility stream size buffer */

	if (vsc_size_address) {
		ret = kgsl_snapshot_get_object(device, ptbase,
				vsc_size_address, 32,
				SNAPSHOT_GPU_OBJECT_GENERIC);
		if (ret < 0)
			return -EINVAL;

		snapshot_frozen_objsize += ret;
	}

	/* Next private shader buffer memory */
	if (sp_vs_pvt_mem_addr) {
		ret = kgsl_snapshot_get_object(device, ptbase,
				sp_vs_pvt_mem_addr, 8192,
				SNAPSHOT_GPU_OBJECT_GENERIC);
		if (ret < 0)
			return -EINVAL;

		snapshot_frozen_objsize += ret;
		sp_vs_pvt_mem_addr = 0;
	}

	if (sp_fs_pvt_mem_addr) {
		ret = kgsl_snapshot_get_object(device, ptbase,
				sp_fs_pvt_mem_addr, 8192,
				SNAPSHOT_GPU_OBJECT_GENERIC);
		if (ret < 0)
			return -EINVAL;

		snapshot_frozen_objsize += ret;
		sp_fs_pvt_mem_addr = 0;
	}

	if (sp_vs_obj_start_reg) {
		ret = kgsl_snapshot_get_object(device, ptbase,
			sp_vs_obj_start_reg & 0xFFFFFFE0, 0,
			SNAPSHOT_GPU_OBJECT_GENERIC);
		if (ret < 0)
			return -EINVAL;
		snapshot_frozen_objsize += ret;
		sp_vs_obj_start_reg = 0;
	}

	if (sp_fs_obj_start_reg) {
		ret = kgsl_snapshot_get_object(device, ptbase,
			sp_fs_obj_start_reg & 0xFFFFFFE0, 0,
			SNAPSHOT_GPU_OBJECT_GENERIC);
		if (ret < 0)
			return -EINVAL;
		snapshot_frozen_objsize += ret;
		sp_fs_obj_start_reg = 0;
	}

	/* Finally: VBOs */

	/* The number of active VBOs is stored in VFD_CONTROL_O[31:27] */
	for (i = 0; i < (vfd_control_0) >> 27; i++) {
		int size;

		/*
		 * The size of the VBO is the stride stored in
		 * VFD_FETCH_INSTR_0_X.BUFSTRIDE * VFD_INDEX_MAX. The base
		 * is stored in VFD_FETCH_INSTR_1_X
		 */

		if (vbo[i].base != 0) {
			size = vbo[i].stride * vfd_index_max;

			ret = kgsl_snapshot_get_object(device, ptbase,
				vbo[i].base,
				0, SNAPSHOT_GPU_OBJECT_GENERIC);
			if (ret < 0)
				return -EINVAL;

			snapshot_frozen_objsize += ret;
		}

		vbo[i].base = 0;
		vbo[i].stride = 0;
	}

	vfd_control_0 = 0;
	vfd_index_max = 0;

	return ret;
}

/*
 * Parse all the type3 opcode packets that may contain important information,
 * such as additional GPU buffers to grab or a draw initator
 */

static int ib_parse_type3(struct kgsl_device *device, unsigned int *ptr,
	phys_addr_t ptbase)
{
	int opcode = cp_type3_opcode(*ptr);

	if (opcode == CP_LOAD_STATE)
		return ib_parse_load_state(device, ptr, ptbase);
	else if (opcode == CP_SET_BIN_DATA)
		return ib_parse_set_bin_data(device, ptr, ptbase);
	else if (opcode == CP_MEM_WRITE)
		return ib_parse_mem_write(device, ptr, ptbase);
	else if (opcode == CP_DRAW_INDX)
		return ib_parse_draw_indx(device, ptr, ptbase);

	return 0;
}

/*
 * Parse type0 packets found in the stream.  Some of the registers that are
 * written are clues for GPU buffers that we need to freeze.  Register writes
 * are considred valid when a draw initator is called, so just cache the values
 * here and freeze them when a CP_DRAW_INDX is seen.  This protects against
 * needlessly caching buffers that won't be used during a draw call
 */

static void ib_parse_type0(struct kgsl_device *device, unsigned int *ptr,
	phys_addr_t ptbase)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int size = type0_pkt_size(*ptr);
	int offset = type0_pkt_offset(*ptr);
	int i;

	for (i = 0; i < size - 1; i++, offset++) {

		/* Visiblity stream buffer */

		if (offset >= adreno_getreg(adreno_dev,
				ADRENO_REG_VSC_PIPE_DATA_ADDRESS_0) &&
			offset <= adreno_getreg(adreno_dev,
					ADRENO_REG_VSC_PIPE_DATA_LENGTH_7)) {
			int index = offset - adreno_getreg(adreno_dev,
					ADRENO_REG_VSC_PIPE_DATA_ADDRESS_0);

			/* Each bank of address and length registers are
			 * interleaved with an empty register:
			 *
			 * address 0
			 * length 0
			 * empty
			 * address 1
			 * length 1
			 * empty
			 * ...
			 */

			if ((index % 3) == 0)
				vsc_pipe[index / 3].base = ptr[i + 1];
			else if ((index % 3) == 1)
				vsc_pipe[index / 3].size = ptr[i + 1];
		} else if ((offset >= adreno_getreg(adreno_dev,
					ADRENO_REG_VFD_FETCH_INSTR_0_0)) &&
			(offset <= adreno_getreg(adreno_dev,
					ADRENO_REG_VFD_FETCH_INSTR_1_F))) {
			int index = offset -
				adreno_getreg(adreno_dev,
					ADRENO_REG_VFD_FETCH_INSTR_0_0);

			/*
			 * FETCH_INSTR_0_X and FETCH_INSTR_1_X banks are
			 * interleaved as above but without the empty register
			 * in between
			 */

			if ((index % 2) == 0)
				vbo[index >> 1].stride =
					(ptr[i + 1] >> 7) & 0x1FF;
			else
				vbo[index >> 1].base = ptr[i + 1];
		} else {
			/*
			 * Cache various support registers for calculating
			 * buffer sizes
			 */

			if (offset ==
				adreno_getreg(adreno_dev,
						ADRENO_REG_VFD_CONTROL_0))
				vfd_control_0 = ptr[i + 1];
			else if (offset ==
				adreno_getreg(adreno_dev,
						ADRENO_REG_VFD_INDEX_MAX))
				vfd_index_max = ptr[i + 1];
			else if (offset ==
				adreno_getreg(adreno_dev,
						ADRENO_REG_VSC_SIZE_ADDRESS))
				vsc_size_address = ptr[i + 1];
			else if (offset == adreno_getreg(adreno_dev,
					ADRENO_REG_SP_VS_PVT_MEM_ADDR_REG))
				sp_vs_pvt_mem_addr = ptr[i + 1];
			else if (offset == adreno_getreg(adreno_dev,
					ADRENO_REG_SP_FS_PVT_MEM_ADDR_REG))
				sp_fs_pvt_mem_addr = ptr[i + 1];
			else if (offset == adreno_getreg(adreno_dev,
					ADRENO_REG_SP_VS_OBJ_START_REG))
				sp_vs_obj_start_reg = ptr[i + 1];
			else if (offset == adreno_getreg(adreno_dev,
					ADRENO_REG_SP_FS_OBJ_START_REG))
				sp_fs_obj_start_reg = ptr[i + 1];
		}
	}
}

static inline int parse_ib(struct kgsl_device *device, phys_addr_t ptbase,
		unsigned int gpuaddr, unsigned int dwords);

/* Add an IB as a GPU object, but first, parse it to find more goodies within */

static int ib_add_gpu_object(struct kgsl_device *device, phys_addr_t ptbase,
		unsigned int gpuaddr, unsigned int dwords)
{
	int i, ret, rem = dwords;
	unsigned int *src;
	struct kgsl_mem_entry *entry = NULL;

	/*
	 * If the object is already in the list, we don't need to parse it again
	 */

	if (kgsl_snapshot_have_object(device, ptbase, gpuaddr, dwords << 2))
		return 0;

	src = (unsigned int *) adreno_convertaddr(device, ptbase, gpuaddr,
		dwords << 2, &entry);

	if (src == NULL)
		return -EINVAL;

	for (i = 0; rem > 0; rem--, i++) {
		int pktsize;

		/* If the packet isn't a type 1 or a type 3, then don't bother
		 * parsing it - it is likely corrupted */

		if (!pkt_is_type0(src[i]) && !pkt_is_type3(src[i]))
			break;

		pktsize = type3_pkt_size(src[i]);

		if (!pktsize || (pktsize + 1) > rem)
			break;

		if (pkt_is_type3(src[i])) {
			if (adreno_cmd_is_ib(src[i])) {
				unsigned int gpuaddr = src[i + 1];
				unsigned int size = src[i + 2];

				ret = parse_ib(device, ptbase, gpuaddr, size);

				/* If adding the IB failed then stop parsing */
				if (ret < 0)
					goto done;
			} else {
				ret = ib_parse_type3(device, &src[i], ptbase);
				/*
				 * If the parse function failed (probably
				 * because of a bad decode) then bail out and
				 * just capture the binary IB data
				 */

				if (ret < 0)
					goto done;
			}
		} else if (pkt_is_type0(src[i])) {
			ib_parse_type0(device, &src[i], ptbase);
		}

		i += pktsize;
		rem -= pktsize;
	}

done:
	ret = kgsl_snapshot_get_object(device, ptbase, gpuaddr, dwords << 2,
		SNAPSHOT_GPU_OBJECT_IB);

	if (ret >= 0)
		snapshot_frozen_objsize += ret;

	if (entry) {
		kgsl_memdesc_unmap(&entry->memdesc);
		kgsl_mem_entry_put(entry);
	}

	return ret;
}

/*
 * We want to store the last executed IB1 and IB2 in the static region to ensure
 * that we get at least some information out of the snapshot even if we can't
 * access the dynamic data from the sysfs file.  Push all other IBs on the
 * dynamic list
 */
static inline int parse_ib(struct kgsl_device *device, phys_addr_t ptbase,
		unsigned int gpuaddr, unsigned int dwords)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned int ib1base, ib2base;
	int ret = 0;

	/*
	 * Check the IB address - if it is either the last executed IB1 or the
	 * last executed IB2 then push it into the static blob otherwise put
	 * it in the dynamic list
	 */

	adreno_readreg(adreno_dev, ADRENO_REG_CP_IB1_BASE, &ib1base);
	adreno_readreg(adreno_dev, ADRENO_REG_CP_IB2_BASE, &ib2base);

	if (gpuaddr == ib1base || gpuaddr == ib2base)
		push_object(device, SNAPSHOT_OBJ_TYPE_IB, ptbase,
			gpuaddr, dwords);
	else
		ret = ib_add_gpu_object(device, ptbase, gpuaddr, dwords);

	return ret;
}

/* Snapshot the ringbuffer memory */
static int snapshot_rb(struct kgsl_device *device, void *snapshot,
	int remain, void *priv)
{
	struct kgsl_snapshot_rb *header = snapshot;
	unsigned int *data = snapshot + sizeof(*header);
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_ringbuffer *rb = &adreno_dev->ringbuffer;
	unsigned int rptr, *rbptr, ibbase;
	phys_addr_t ptbase;
	int index, size, i;
	int parse_ibs = 0, ib_parse_start;

	/* Get the physical address of the MMU pagetable */
	ptbase = kgsl_mmu_get_current_ptbase(&device->mmu);

	/* Get the current read pointers for the RB */
	adreno_readreg(adreno_dev, ADRENO_REG_CP_RB_RPTR, &rptr);

	/* Address of the last processed IB */
	adreno_readreg(adreno_dev, ADRENO_REG_CP_IB1_BASE, &ibbase);

	/*
	 * Figure out the window of ringbuffer data to dump.  First we need to
	 * find where the last processed IB ws submitted.  Start walking back
	 * from the rptr
	 */

	index = rptr;
	rbptr = rb->buffer_desc.hostptr;

	do {
		index--;

		if (index < 0) {
			index = rb->sizedwords - 3;

			/* We wrapped without finding what we wanted */
			if (index < rb->wptr) {
				index = rb->wptr;
				break;
			}
		}

		if (adreno_cmd_is_ib(rbptr[index]) &&
			rbptr[index + 1] == ibbase)
			break;
	} while (index != rb->wptr);

	/*
	 * index points at the last submitted IB. We can only trust that the
	 * memory between the context switch and the hanging IB is valid, so
	 * the next step is to find the context switch before the submission
	 */

	while (index != rb->wptr) {
		index--;

		if (index < 0) {
			index = rb->sizedwords - 2;

			/*
			 * Wrapped without finding the context switch. This is
			 * harmless - we should still have enough data to dump a
			 * valid state
			 */

			if (index < rb->wptr) {
				index = rb->wptr;
				break;
			}
		}

		/* Break if the current packet is a context switch identifier */
		if ((rbptr[index] == cp_nop_packet(1)) &&
			(rbptr[index + 1] == KGSL_CONTEXT_TO_MEM_IDENTIFIER))
			break;
	}

	/*
	 * Index represents the start of the window of interest.  We will try
	 * to dump all buffers between here and the rptr
	 */

	ib_parse_start = index;

	/*
	 * Dump the entire ringbuffer - the parser can choose how much of it to
	 * process
	 */

	size = (rb->sizedwords << 2);

	if (remain < size + sizeof(*header)) {
		KGSL_DRV_ERR(device,
			"snapshot: Not enough memory for the rb section");
		return 0;
	}

	/* Write the sub-header for the section */
	header->start = rb->wptr;
	header->end = rb->wptr;
	header->wptr = rb->wptr;
	header->rbsize = rb->sizedwords;
	header->count = rb->sizedwords;

	/*
	 * Loop through the RB, copying the data and looking for indirect
	 * buffers and MMU pagetable changes
	 */

	index = rb->wptr;
	for (i = 0; i < rb->sizedwords; i++) {
		*data = rbptr[index];

		/*
		 * Only parse IBs between the start and the rptr or the next
		 * context switch, whichever comes first
		 */

		if (parse_ibs == 0 && index == ib_parse_start)
			parse_ibs = 1;
		else if (index == rptr || adreno_rb_ctxtswitch(&rbptr[index]))
			parse_ibs = 0;

		if (parse_ibs && adreno_cmd_is_ib(rbptr[index])) {
			unsigned int ibaddr = rbptr[index + 1];
			unsigned int ibsize = rbptr[index + 2];

			/*
			 * This will return non NULL if the IB happens to be
			 * part of the context memory (i.e - context switch
			 * command buffers)
			 */

			struct kgsl_memdesc *memdesc =
				adreno_find_ctxtmem(device, ptbase, ibaddr,
					ibsize << 2);

			/* IOMMU uses a NOP IB placed in setsate memory */
			if (NULL == memdesc)
				if (kgsl_gpuaddr_in_memdesc(
						&device->mmu.setstate_memory,
						ibaddr, ibsize << 2))
					memdesc = &device->mmu.setstate_memory;
			/*
			 * The IB from CP_IB1_BASE and the IBs for legacy
			 * context switch go into the snapshot all
			 * others get marked at GPU objects
			 */

			if (memdesc != NULL)
				push_object(device, SNAPSHOT_OBJ_TYPE_IB,
					ptbase, ibaddr, ibsize);
			else
				parse_ib(device, ptbase, ibaddr, ibsize);
		}

		index = index + 1;

		if (index == rb->sizedwords)
			index = 0;

		data++;
	}

	/* Return the size of the section */
	return size + sizeof(*header);
}

static int snapshot_capture_mem_list(struct kgsl_device *device, void *snapshot,
			int remain, void *priv)
{
	struct kgsl_snapshot_replay_mem_list *header = snapshot;
	struct kgsl_process_private *private = NULL;
	struct kgsl_process_private *tmp_private;
	phys_addr_t ptbase;
	struct rb_node *node;
	struct kgsl_mem_entry *entry = NULL;
	int num_mem;
	unsigned int *data = snapshot + sizeof(*header);

	ptbase = kgsl_mmu_get_current_ptbase(&device->mmu);
	mutex_lock(&kgsl_driver.process_mutex);
	list_for_each_entry(tmp_private, &kgsl_driver.process_list, list) {
		if (kgsl_mmu_pt_equal(&device->mmu, tmp_private->pagetable,
			ptbase)) {
			private = tmp_private;
			break;
		}
	}
	mutex_unlock(&kgsl_driver.process_mutex);
	if (!private) {
		KGSL_DRV_ERR(device,
		"Failed to get pointer to process private structure\n");
		return 0;
	}
	/* We need to know the number of memory objects that the process has */
	spin_lock(&private->mem_lock);
	for (node = rb_first(&private->mem_rb), num_mem = 0; node; ) {
		entry = rb_entry(node, struct kgsl_mem_entry, node);
		node = rb_next(&entry->node);
		num_mem++;
	}

	if (remain < ((num_mem * 3 * sizeof(unsigned int)) +
			sizeof(*header))) {
		KGSL_DRV_ERR(device,
			"snapshot: Not enough memory for the mem list section");
		spin_unlock(&private->mem_lock);
		return 0;
	}
	header->num_entries = num_mem;
	header->ptbase = (__u32)ptbase;
	/*
	 * Walk throught the memory list and store the
	 * tuples(gpuaddr, size, memtype) in snapshot
	 */
	for (node = rb_first(&private->mem_rb); node; ) {
		entry = rb_entry(node, struct kgsl_mem_entry, node);
		node = rb_next(&entry->node);

		*data++ = entry->memdesc.gpuaddr;
		*data++ = entry->memdesc.size;
		*data++ = (entry->memdesc.priv & KGSL_MEMTYPE_MASK) >>
							KGSL_MEMTYPE_SHIFT;
	}
	spin_unlock(&private->mem_lock);
	return sizeof(*header) + (num_mem * 3 * sizeof(unsigned int));
}

/* Snapshot the memory for an indirect buffer */
static int snapshot_ib(struct kgsl_device *device, void *snapshot,
	int remain, void *priv)
{
	struct kgsl_snapshot_ib *header = snapshot;
	struct kgsl_snapshot_obj *obj = priv;
	unsigned int *src = obj->ptr;
	unsigned int *dst = snapshot + sizeof(*header);
	int i, ret;

	if (remain < (obj->dwords << 2) + sizeof(*header)) {
		KGSL_DRV_ERR(device,
			"snapshot: Not enough memory for the ib section");
		return 0;
	}

	/* Write the sub-header for the section */
	header->gpuaddr = obj->gpuaddr;
	header->ptbase = (__u32)obj->ptbase;
	header->size = obj->dwords;

	/* Write the contents of the ib */
	for (i = 0; i < obj->dwords; i++, src++, dst++) {
		*dst = *src;

		if (pkt_is_type3(*src)) {
			if ((obj->dwords - i) < type3_pkt_size(*src) + 1)
				continue;

			if (adreno_cmd_is_ib(*src))
				ret = parse_ib(device, obj->ptbase, src[1],
					src[2]);
			else
				ret = ib_parse_type3(device, src, obj->ptbase);

			/* Stop parsing if the type3 decode fails */
			if (ret < 0)
				break;
		}
	}

	return (obj->dwords << 2) + sizeof(*header);
}

/* Dump another item on the current pending list */
static void *dump_object(struct kgsl_device *device, int obj, void *snapshot,
	int *remain)
{
	switch (objbuf[obj].type) {
	case SNAPSHOT_OBJ_TYPE_IB:
		snapshot = kgsl_snapshot_add_section(device,
			KGSL_SNAPSHOT_SECTION_IB, snapshot, remain,
			snapshot_ib, &objbuf[obj]);
		if (objbuf[obj].entry) {
			kgsl_memdesc_unmap(&(objbuf[obj].entry->memdesc));
			kgsl_mem_entry_put(objbuf[obj].entry);
		}
		break;
	default:
		KGSL_DRV_ERR(device,
			"snapshot: Invalid snapshot object type: %d\n",
			objbuf[obj].type);
		break;
	}

	return snapshot;
}

/* adreno_snapshot - Snapshot the Adreno GPU state
 * @device - KGSL device to snapshot
 * @snapshot - Pointer to the start of memory to write into
 * @remain - A pointer to how many bytes of memory are remaining in the snapshot
 * @hang - set if this snapshot was automatically triggered by a GPU hang
 * This is a hook function called by kgsl_snapshot to snapshot the
 * Adreno specific information for the GPU snapshot.  In turn, this function
 * calls the GPU specific snapshot function to get core specific information.
 */

void *adreno_snapshot(struct kgsl_device *device, void *snapshot, int *remain,
		int hang)
{
	int i;
	uint32_t ibbase, ibsize;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	phys_addr_t ptbase;

	/* Reset the list of objects */
	objbufptr = 0;

	snapshot_frozen_objsize = 0;

	/* Clear the caches for the visibilty stream and VBO parsing */

	vfd_control_0 = 0;
	vfd_index_max = 0;
	vsc_size_address = 0;

	memset(vsc_pipe, 0, sizeof(vsc_pipe));
	memset(vbo, 0, sizeof(vbo));

	/* Get the physical address of the MMU pagetable */
	ptbase = kgsl_mmu_get_current_ptbase(&device->mmu);

	/* Dump the ringbuffer */
	snapshot = kgsl_snapshot_add_section(device, KGSL_SNAPSHOT_SECTION_RB,
		snapshot, remain, snapshot_rb, NULL);

	/*
	 * Add a section that lists (gpuaddr, size, memtype) tuples of the
	 * hanging process
	 */
	snapshot = kgsl_snapshot_add_section(device,
			KGSL_SNAPSHOT_SECTION_MEMLIST, snapshot, remain,
			snapshot_capture_mem_list, NULL);
	/*
	 * Make sure that the last IB1 that was being executed is dumped.
	 * Since this was the last IB1 that was processed, we should have
	 * already added it to the list during the ringbuffer parse but we
	 * want to be double plus sure.
	 */

	adreno_readreg(adreno_dev, ADRENO_REG_CP_IB1_BASE, &ibbase);
	adreno_readreg(adreno_dev, ADRENO_REG_CP_IB1_BUFSZ, &ibsize);

	/*
	 * The problem is that IB size from the register is the unprocessed size
	 * of the buffer not the original size, so if we didn't catch this
	 * buffer being directly used in the RB, then we might not be able to
	 * dump the whle thing. Print a warning message so we can try to
	 * figure how often this really happens.
	 */

	if (!find_object(SNAPSHOT_OBJ_TYPE_IB, ibbase, ptbase) && ibsize) {
		push_object(device, SNAPSHOT_OBJ_TYPE_IB, ptbase,
			ibbase, ibsize);
		KGSL_DRV_ERR(device, "CP_IB1_BASE not found in the ringbuffer. "
			"Dumping %x dwords of the buffer.\n", ibsize);
	}

	adreno_readreg(adreno_dev, ADRENO_REG_CP_IB2_BASE, &ibbase);
	adreno_readreg(adreno_dev, ADRENO_REG_CP_IB2_BUFSZ, &ibsize);

	/*
	 * Add the last parsed IB2 to the list. The IB2 should be found as we
	 * parse the objects below, but we try to add it to the list first, so
	 * it too can be parsed.  Don't print an error message in this case - if
	 * the IB2 is found during parsing, the list will be updated with the
	 * correct size.
	 */

	if (!find_object(SNAPSHOT_OBJ_TYPE_IB, ibbase, ptbase) && ibsize) {
		push_object(device, SNAPSHOT_OBJ_TYPE_IB, ptbase,
			ibbase, ibsize);
	}

	/*
	 * Go through the list of found objects and dump each one.  As the IBs
	 * are parsed, more objects might be found, and objbufptr will increase
	 */
	for (i = 0; i < objbufptr; i++)
		snapshot = dump_object(device, i, snapshot, remain);

	/* Add GPU specific sections - registers mainly, but other stuff too */
	if (adreno_dev->gpudev->snapshot)
		snapshot = adreno_dev->gpudev->snapshot(adreno_dev, snapshot,
			remain, hang);

	if (snapshot_frozen_objsize)
		KGSL_DRV_ERR(device, "GPU snapshot froze %dKb of GPU buffers\n",
			snapshot_frozen_objsize / 1024);

	return snapshot;
}
