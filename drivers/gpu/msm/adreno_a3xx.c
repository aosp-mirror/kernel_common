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
 *
 */

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/ratelimit.h>
#include <mach/socinfo.h>

#include "kgsl.h"
#include "adreno.h"
#include "kgsl_sharedmem.h"
#include "kgsl_cffdump.h"
#include "a3xx_reg.h"
#include "adreno_a3xx_trace.h"


const unsigned int a3xx_registers[] = {
	0x0000, 0x0002, 0x0010, 0x0012, 0x0018, 0x0018, 0x0020, 0x0027,
	0x0029, 0x002b, 0x002e, 0x0033, 0x0040, 0x0042, 0x0050, 0x005c,
	0x0060, 0x006c, 0x0080, 0x0082, 0x0084, 0x0088, 0x0090, 0x00e5,
	0x00ea, 0x00ed, 0x0100, 0x0100, 0x0110, 0x0123, 0x01c0, 0x01c1,
	0x01c3, 0x01c5, 0x01c7, 0x01c7, 0x01d5, 0x01d9, 0x01dc, 0x01dd,
	0x01ea, 0x01ea, 0x01ee, 0x01f1, 0x01f5, 0x01f5, 0x01fc, 0x01ff,
	0x0440, 0x0440, 0x0443, 0x0443, 0x0445, 0x0445, 0x044d, 0x044f,
	0x0452, 0x0452, 0x0454, 0x046f, 0x047c, 0x047c, 0x047f, 0x047f,
	0x0578, 0x057f, 0x0600, 0x0602, 0x0605, 0x0607, 0x060a, 0x060e,
	0x0612, 0x0614, 0x0c01, 0x0c02, 0x0c06, 0x0c1d, 0x0c3d, 0x0c3f,
	0x0c48, 0x0c4b, 0x0c80, 0x0c80, 0x0c88, 0x0c8b, 0x0ca0, 0x0cb7,
	0x0cc0, 0x0cc1, 0x0cc6, 0x0cc7, 0x0ce4, 0x0ce5,
	0x0e41, 0x0e45, 0x0e64, 0x0e65,
	0x0e80, 0x0e82, 0x0e84, 0x0e89, 0x0ea0, 0x0ea1, 0x0ea4, 0x0ea7,
	0x0ec4, 0x0ecb, 0x0ee0, 0x0ee0, 0x0f00, 0x0f01, 0x0f03, 0x0f09,
	0x2040, 0x2040, 0x2044, 0x2044, 0x2048, 0x204d, 0x2068, 0x2069,
	0x206c, 0x206d, 0x2070, 0x2070, 0x2072, 0x2072, 0x2074, 0x2075,
	0x2079, 0x207a, 0x20c0, 0x20d3, 0x20e4, 0x20ef, 0x2100, 0x2109,
	0x210c, 0x210c, 0x210e, 0x210e, 0x2110, 0x2111, 0x2114, 0x2115,
	0x21e4, 0x21e4, 0x21ea, 0x21ea, 0x21ec, 0x21ed, 0x21f0, 0x21f0,
	0x2240, 0x227e,
	0x2280, 0x228b, 0x22c0, 0x22c0, 0x22c4, 0x22ce, 0x22d0, 0x22d8,
	0x22df, 0x22e6, 0x22e8, 0x22e9, 0x22ec, 0x22ec, 0x22f0, 0x22f7,
	0x22ff, 0x22ff, 0x2340, 0x2343,
	0x2440, 0x2440, 0x2444, 0x2444, 0x2448, 0x244d,
	0x2468, 0x2469, 0x246c, 0x246d, 0x2470, 0x2470, 0x2472, 0x2472,
	0x2474, 0x2475, 0x2479, 0x247a, 0x24c0, 0x24d3, 0x24e4, 0x24ef,
	0x2500, 0x2509, 0x250c, 0x250c, 0x250e, 0x250e, 0x2510, 0x2511,
	0x2514, 0x2515, 0x25e4, 0x25e4, 0x25ea, 0x25ea, 0x25ec, 0x25ed,
	0x25f0, 0x25f0,
	0x2640, 0x267e, 0x2680, 0x268b, 0x26c0, 0x26c0, 0x26c4, 0x26ce,
	0x26d0, 0x26d8, 0x26df, 0x26e6, 0x26e8, 0x26e9, 0x26ec, 0x26ec,
	0x26f0, 0x26f7, 0x26ff, 0x26ff, 0x2740, 0x2743,
	0x300C, 0x300E, 0x301C, 0x301D,
	0x302A, 0x302A, 0x302C, 0x302D, 0x3030, 0x3031, 0x3034, 0x3036,
	0x303C, 0x303C, 0x305E, 0x305F,
};

const unsigned int a3xx_registers_count = ARRAY_SIZE(a3xx_registers) / 2;

const unsigned int a3xx_hlsq_registers[] = {
	0x0e00, 0x0e05, 0x0e0c, 0x0e0c, 0x0e22, 0x0e23,
	0x2200, 0x2212, 0x2214, 0x2217, 0x221a, 0x221a,
	0x2600, 0x2612, 0x2614, 0x2617, 0x261a, 0x261a,
};

const unsigned int a3xx_hlsq_registers_count =
			ARRAY_SIZE(a3xx_hlsq_registers) / 2;


const unsigned int a330_registers[] = {
	0x1d0, 0x1d0, 0x1d4, 0x1d4, 0x453, 0x453,
};

const unsigned int a330_registers_count = ARRAY_SIZE(a330_registers) / 2;


#define _SET(_shift, _val) ((_val) << (_shift))

#define VBIF_PERF_MASK (VBIF_PERF_CNT_0 | VBIF_PERF_PWR_CNT_0)
#define RBBM_PERF_ENABLE_MASK (RBBM_RBBM_CTL_ENABLE_PWR_CTR1)
#define RBBM_PERF_RESET_MASK (RBBM_RBBM_CTL_RESET_PWR_CTR1)


#define ALU_SHADOW_SIZE      (8*1024) 
#define REG_SHADOW_SIZE      (4*1024) 
#define CMD_BUFFER_SIZE      (5*1024) 
#define TEX_SIZE_MEM_OBJECTS 896      
#define TEX_SIZE_MIPMAP      1936     
#define TEX_SIZE_SAMPLER_OBJ 256      
#define TEX_SHADOW_SIZE                            \
	((TEX_SIZE_MEM_OBJECTS + TEX_SIZE_MIPMAP + \
	TEX_SIZE_SAMPLER_OBJ)*2) 
#define SHADER_SHADOW_SIZE   (8*1024) 

#define CONTEXT_SIZE                         \
	(ALU_SHADOW_SIZE+REG_SHADOW_SIZE +   \
	CMD_BUFFER_SIZE+SHADER_SHADOW_SIZE + \
	TEX_SHADOW_SIZE)

#define REG_OFFSET ALU_SHADOW_SIZE
#define CMD_OFFSET (REG_OFFSET+REG_SHADOW_SIZE)
#define SHADER_OFFSET (CMD_OFFSET+CMD_BUFFER_SIZE)
#define TEX_OFFSET (SHADER_OFFSET+SHADER_SHADOW_SIZE)
#define VS_TEX_OFFSET_MEM_OBJECTS TEX_OFFSET
#define VS_TEX_OFFSET_MIPMAP (VS_TEX_OFFSET_MEM_OBJECTS+TEX_SIZE_MEM_OBJECTS)
#define VS_TEX_OFFSET_SAMPLER_OBJ (VS_TEX_OFFSET_MIPMAP+TEX_SIZE_MIPMAP)
#define FS_TEX_OFFSET_MEM_OBJECTS \
	(VS_TEX_OFFSET_SAMPLER_OBJ+TEX_SIZE_SAMPLER_OBJ)
#define FS_TEX_OFFSET_MIPMAP (FS_TEX_OFFSET_MEM_OBJECTS+TEX_SIZE_MEM_OBJECTS)
#define FS_TEX_OFFSET_SAMPLER_OBJ (FS_TEX_OFFSET_MIPMAP+TEX_SIZE_MIPMAP)

#define SSIZE (16*1024)

#define HLSQ_SAMPLER_OFFSET 0x000
#define HLSQ_MEMOBJ_OFFSET  0x400
#define HLSQ_MIPMAP_OFFSET  0x800

#define HLSQ_SHADOW_BASE		(0x10000+SSIZE*2)

#define REG_TO_MEM_LOOP_COUNT_SHIFT	18

#define BUILD_PC_DRAW_INITIATOR(prim_type, source_select, index_size, \
	vis_cull_mode) \
	(((prim_type)      << PC_DRAW_INITIATOR_PRIM_TYPE) | \
	((source_select)   << PC_DRAW_INITIATOR_SOURCE_SELECT) | \
	((index_size & 1)  << PC_DRAW_INITIATOR_INDEX_SIZE) | \
	((index_size >> 1) << PC_DRAW_INITIATOR_SMALL_INDEX) | \
	((vis_cull_mode)   << PC_DRAW_INITIATOR_VISIBILITY_CULLING_MODE) | \
	(1                 << PC_DRAW_INITIATOR_PRE_DRAW_INITIATOR_ENABLE))

static const unsigned int context_register_ranges[] = {
	A3XX_GRAS_CL_CLIP_CNTL, A3XX_GRAS_CL_CLIP_CNTL,
	A3XX_GRAS_CL_GB_CLIP_ADJ, A3XX_GRAS_CL_GB_CLIP_ADJ,
	A3XX_GRAS_CL_VPORT_XOFFSET, A3XX_GRAS_CL_VPORT_ZSCALE,
	A3XX_GRAS_SU_POINT_MINMAX, A3XX_GRAS_SU_POINT_SIZE,
	A3XX_GRAS_SU_POLY_OFFSET_SCALE, A3XX_GRAS_SU_POLY_OFFSET_OFFSET,
	A3XX_GRAS_SU_MODE_CONTROL, A3XX_GRAS_SU_MODE_CONTROL,
	A3XX_GRAS_SC_CONTROL, A3XX_GRAS_SC_CONTROL,
	A3XX_GRAS_SC_SCREEN_SCISSOR_TL, A3XX_GRAS_SC_SCREEN_SCISSOR_BR,
	A3XX_GRAS_SC_WINDOW_SCISSOR_TL, A3XX_GRAS_SC_WINDOW_SCISSOR_BR,
	A3XX_RB_MODE_CONTROL, A3XX_RB_MRT_BLEND_CONTROL3,
	A3XX_RB_BLEND_RED, A3XX_RB_COPY_DEST_INFO,
	A3XX_RB_DEPTH_CONTROL, A3XX_RB_DEPTH_CONTROL,
	A3XX_PC_VSTREAM_CONTROL, A3XX_PC_VSTREAM_CONTROL,
	A3XX_PC_VERTEX_REUSE_BLOCK_CNTL, A3XX_PC_VERTEX_REUSE_BLOCK_CNTL,
	A3XX_PC_PRIM_VTX_CNTL, A3XX_PC_RESTART_INDEX,
	A3XX_HLSQ_CONTROL_0_REG, A3XX_HLSQ_CONST_FSPRESV_RANGE_REG,
	A3XX_HLSQ_CL_NDRANGE_0_REG, A3XX_HLSQ_CL_NDRANGE_0_REG,
	A3XX_HLSQ_CL_NDRANGE_2_REG, A3XX_HLSQ_CL_CONTROL_1_REG,
	A3XX_HLSQ_CL_KERNEL_CONST_REG, A3XX_HLSQ_CL_KERNEL_GROUP_Z_REG,
	A3XX_HLSQ_CL_WG_OFFSET_REG, A3XX_HLSQ_CL_WG_OFFSET_REG,
	A3XX_VFD_CONTROL_0, A3XX_VFD_VS_THREADING_THRESHOLD,
	A3XX_SP_SP_CTRL_REG, A3XX_SP_SP_CTRL_REG,
	A3XX_SP_VS_CTRL_REG0, A3XX_SP_VS_OUT_REG_7,
	A3XX_SP_VS_VPC_DST_REG_0, A3XX_SP_VS_PVT_MEM_SIZE_REG,
	A3XX_SP_VS_LENGTH_REG, A3XX_SP_FS_PVT_MEM_SIZE_REG,
	A3XX_SP_FS_FLAT_SHAD_MODE_REG_0, A3XX_SP_FS_FLAT_SHAD_MODE_REG_1,
	A3XX_SP_FS_OUTPUT_REG, A3XX_SP_FS_OUTPUT_REG,
	A3XX_SP_FS_MRT_REG_0, A3XX_SP_FS_IMAGE_OUTPUT_REG_3,
	A3XX_SP_FS_LENGTH_REG, A3XX_SP_FS_LENGTH_REG,
	A3XX_TPL1_TP_VS_TEX_OFFSET, A3XX_TPL1_TP_FS_BORDER_COLOR_BASE_ADDR,
	A3XX_VPC_ATTR, A3XX_VPC_VARY_CYLWRAP_ENABLE_1,
};

static const unsigned int global_registers[] = {
	A3XX_GRAS_CL_USER_PLANE_X0, A3XX_GRAS_CL_USER_PLANE_Y0,
	A3XX_GRAS_CL_USER_PLANE_Z0, A3XX_GRAS_CL_USER_PLANE_W0,
	A3XX_GRAS_CL_USER_PLANE_X1, A3XX_GRAS_CL_USER_PLANE_Y1,
	A3XX_GRAS_CL_USER_PLANE_Z1, A3XX_GRAS_CL_USER_PLANE_W1,
	A3XX_GRAS_CL_USER_PLANE_X2, A3XX_GRAS_CL_USER_PLANE_Y2,
	A3XX_GRAS_CL_USER_PLANE_Z2, A3XX_GRAS_CL_USER_PLANE_W2,
	A3XX_GRAS_CL_USER_PLANE_X3, A3XX_GRAS_CL_USER_PLANE_Y3,
	A3XX_GRAS_CL_USER_PLANE_Z3, A3XX_GRAS_CL_USER_PLANE_W3,
	A3XX_GRAS_CL_USER_PLANE_X4, A3XX_GRAS_CL_USER_PLANE_Y4,
	A3XX_GRAS_CL_USER_PLANE_Z4, A3XX_GRAS_CL_USER_PLANE_W4,
	A3XX_GRAS_CL_USER_PLANE_X5, A3XX_GRAS_CL_USER_PLANE_Y5,
	A3XX_GRAS_CL_USER_PLANE_Z5, A3XX_GRAS_CL_USER_PLANE_W5,
	A3XX_VSC_BIN_SIZE,
	A3XX_VSC_PIPE_CONFIG_0, A3XX_VSC_PIPE_CONFIG_1,
	A3XX_VSC_PIPE_CONFIG_2, A3XX_VSC_PIPE_CONFIG_3,
	A3XX_VSC_PIPE_CONFIG_4, A3XX_VSC_PIPE_CONFIG_5,
	A3XX_VSC_PIPE_CONFIG_6, A3XX_VSC_PIPE_CONFIG_7,
	A3XX_VSC_PIPE_DATA_ADDRESS_0, A3XX_VSC_PIPE_DATA_ADDRESS_1,
	A3XX_VSC_PIPE_DATA_ADDRESS_2, A3XX_VSC_PIPE_DATA_ADDRESS_3,
	A3XX_VSC_PIPE_DATA_ADDRESS_4, A3XX_VSC_PIPE_DATA_ADDRESS_5,
	A3XX_VSC_PIPE_DATA_ADDRESS_6, A3XX_VSC_PIPE_DATA_ADDRESS_7,
	A3XX_VSC_PIPE_DATA_LENGTH_0, A3XX_VSC_PIPE_DATA_LENGTH_1,
	A3XX_VSC_PIPE_DATA_LENGTH_2, A3XX_VSC_PIPE_DATA_LENGTH_3,
	A3XX_VSC_PIPE_DATA_LENGTH_4, A3XX_VSC_PIPE_DATA_LENGTH_5,
	A3XX_VSC_PIPE_DATA_LENGTH_6, A3XX_VSC_PIPE_DATA_LENGTH_7,
	A3XX_VSC_SIZE_ADDRESS
};

#define GLOBAL_REGISTER_COUNT ARRAY_SIZE(global_registers)

static struct tmp_ctx {
	unsigned int *cmd; 

	
	uint32_t reg_values[GLOBAL_REGISTER_COUNT];
	uint32_t gmem_base; 
} tmp_ctx;

#ifndef GSL_CONTEXT_SWITCH_CPU_SYNC
static unsigned int *rmw_regtomem(unsigned int *cmd,
				  unsigned int reg, unsigned int and,
				  unsigned int rol, unsigned int or,
				  unsigned int dest)
{
	
	*cmd++ = cp_type3_packet(CP_REG_RMW, 3);
	*cmd++ = (1 << 30) | A3XX_CP_SCRATCH_REG2;
	*cmd++ = 0x00000000;	
	*cmd++ = reg;		

	
	*cmd++ = cp_type3_packet(CP_REG_RMW, 3);
	*cmd++ = (rol << 24) | A3XX_CP_SCRATCH_REG2;
	*cmd++ = and;		
	*cmd++ = or;		

	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ = A3XX_CP_SCRATCH_REG2;
	*cmd++ = dest;

	return cmd;
}
#endif

static void build_regconstantsave_cmds(struct adreno_device *adreno_dev,
				       struct adreno_context *drawctxt)
{
	unsigned int *cmd = tmp_ctx.cmd;
	unsigned int *start;
	unsigned int i;

	drawctxt->constant_save_commands[0].hostptr = cmd;
	drawctxt->constant_save_commands[0].gpuaddr =
	    virt2gpu(cmd, &drawctxt->gpustate);
	cmd++;

	start = cmd;

	*cmd++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
	*cmd++ = 0;

#ifndef CONFIG_MSM_KGSL_DISABLE_SHADOW_WRITES

	*cmd++ = cp_type3_packet(CP_LOAD_CONSTANT_CONTEXT, 3);
	*cmd++ = (drawctxt->gpustate.gpuaddr + REG_OFFSET) & 0xFFFFE000;
	*cmd++ = 4 << 16;	
	*cmd++ = 0x0;		

#else

	
	for (i = 0; i < ARRAY_SIZE(context_register_ranges) / 2; i++) {
		unsigned int start = context_register_ranges[i * 2];
		unsigned int end = context_register_ranges[i * 2 + 1];
		*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
		*cmd++ = ((end - start + 1) << REG_TO_MEM_LOOP_COUNT_SHIFT) |
			start;
		*cmd++ = ((drawctxt->gpustate.gpuaddr + REG_OFFSET)
			  & 0xFFFFE000) + (start - 0x2000) * 4;
	}
#endif

	
	for (i = 0; i < ARRAY_SIZE(global_registers); i++) {
		*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
		*cmd++ = global_registers[i];
		*cmd++ = tmp_ctx.reg_values[i];
	}

	
	*cmd++ = cp_type3_packet(CP_COND_EXEC, 4);
	*cmd++ = drawctxt->cond_execs[2].gpuaddr >> 2;
	*cmd++ = drawctxt->cond_execs[2].gpuaddr >> 2;
	*cmd++ = 0x0000FFFF;
	*cmd++ = 3; 
	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	drawctxt->constant_save_commands[1].hostptr = cmd;
	drawctxt->constant_save_commands[1].gpuaddr =
	    virt2gpu(cmd, &drawctxt->gpustate);
	*cmd++ = 0;	
	
	*cmd++ = drawctxt->gpustate.gpuaddr & 0xfffffffc;

	
	*cmd++ = cp_type3_packet(CP_COND_EXEC, 4);
	*cmd++ = drawctxt->cond_execs[3].gpuaddr >> 2;
	*cmd++ = drawctxt->cond_execs[3].gpuaddr >> 2;
	*cmd++ = 0x0000FFFF;
	*cmd++ = 3; 
	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	drawctxt->constant_save_commands[2].hostptr = cmd;
	drawctxt->constant_save_commands[2].gpuaddr =
	    virt2gpu(cmd, &drawctxt->gpustate);
	*cmd++ = 0;	

	*cmd++ = 0;		

	
	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ =
	    ((TEX_SIZE_MEM_OBJECTS / 4) << REG_TO_MEM_LOOP_COUNT_SHIFT) |
		((HLSQ_SHADOW_BASE + HLSQ_MEMOBJ_OFFSET) / 4);
	*cmd++ =
	    (drawctxt->gpustate.gpuaddr +
	     VS_TEX_OFFSET_MEM_OBJECTS) & 0xfffffffc;

	
	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ =
	    ((TEX_SIZE_MIPMAP / 4) << REG_TO_MEM_LOOP_COUNT_SHIFT) |
		((HLSQ_SHADOW_BASE + HLSQ_MIPMAP_OFFSET) / 4);
	*cmd++ =
	    (drawctxt->gpustate.gpuaddr + VS_TEX_OFFSET_MIPMAP) & 0xfffffffc;

	
	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ = ((TEX_SIZE_SAMPLER_OBJ / 4) << REG_TO_MEM_LOOP_COUNT_SHIFT) |
		((HLSQ_SHADOW_BASE + HLSQ_SAMPLER_OFFSET) / 4);
	*cmd++ =
	    (drawctxt->gpustate.gpuaddr +
	     VS_TEX_OFFSET_SAMPLER_OBJ) & 0xfffffffc;

	
	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ =
	    ((TEX_SIZE_MEM_OBJECTS / 4) << REG_TO_MEM_LOOP_COUNT_SHIFT) |
		((HLSQ_SHADOW_BASE + HLSQ_MEMOBJ_OFFSET + SSIZE) / 4);
	*cmd++ =
	    (drawctxt->gpustate.gpuaddr +
	     FS_TEX_OFFSET_MEM_OBJECTS) & 0xfffffffc;

	
	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ =
	    ((TEX_SIZE_MIPMAP / 4) << REG_TO_MEM_LOOP_COUNT_SHIFT) |
		((HLSQ_SHADOW_BASE + HLSQ_MIPMAP_OFFSET + SSIZE) / 4);
	*cmd++ =
	    (drawctxt->gpustate.gpuaddr + FS_TEX_OFFSET_MIPMAP) & 0xfffffffc;

	
	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ =
	    ((TEX_SIZE_SAMPLER_OBJ / 4) << REG_TO_MEM_LOOP_COUNT_SHIFT) |
		((HLSQ_SHADOW_BASE + HLSQ_SAMPLER_OFFSET + SSIZE) / 4);
	*cmd++ =
	    (drawctxt->gpustate.gpuaddr +
	     FS_TEX_OFFSET_SAMPLER_OBJ) & 0xfffffffc;

	
	create_ib1(drawctxt, drawctxt->regconstant_save, start, cmd);

	tmp_ctx.cmd = cmd;
}

unsigned int adreno_a3xx_rbbm_clock_ctl_default(struct adreno_device
							*adreno_dev)
{
	if (adreno_is_a305(adreno_dev))
		return A305_RBBM_CLOCK_CTL_DEFAULT;
	else if (adreno_is_a305c(adreno_dev))
		return A305C_RBBM_CLOCK_CTL_DEFAULT;
	else if (adreno_is_a320(adreno_dev))
		return A320_RBBM_CLOCK_CTL_DEFAULT;
	else if (adreno_is_a330v2(adreno_dev))
		return A330v2_RBBM_CLOCK_CTL_DEFAULT;
	else if (adreno_is_a330(adreno_dev))
		return A330_RBBM_CLOCK_CTL_DEFAULT;
	else if (adreno_is_a305b(adreno_dev))
		return A305B_RBBM_CLOCK_CTL_DEFAULT;

	BUG_ON(1);
}

static unsigned int *build_gmem2sys_cmds(struct adreno_device *adreno_dev,
					 struct adreno_context *drawctxt,
					 struct gmem_shadow_t *shadow)
{
	unsigned int *cmds = tmp_ctx.cmd;
	unsigned int *start = cmds;

	*cmds++ = cp_type0_packet(A3XX_RBBM_CLOCK_CTL, 1);
	*cmds++ = adreno_a3xx_rbbm_clock_ctl_default(adreno_dev);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_RB_MODE_CONTROL);

	
	*cmds++ = _SET(RB_MODECONTROL_RENDER_MODE, RB_RESOLVE_PASS) |
		_SET(RB_MODECONTROL_MARB_CACHE_SPLIT_MODE, 1) |
		_SET(RB_MODECONTROL_PACKER_TIMER_ENABLE, 1);
	
	*cmds++ = _SET(RB_RENDERCONTROL_BIN_WIDTH, shadow->width >> 5) |
		_SET(RB_RENDERCONTROL_DISABLE_COLOR_PIPE, 1);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 5);
	*cmds++ = CP_REG(A3XX_RB_COPY_CONTROL);
	
	*cmds++ = _SET(RB_COPYCONTROL_RESOLVE_CLEAR_MODE,
		RB_CLEAR_MODE_RESOLVE) |
		_SET(RB_COPYCONTROL_COPY_GMEM_BASE,
		tmp_ctx.gmem_base >> 14);
	
	*cmds++ = _SET(RB_COPYDESTBASE_COPY_DEST_BASE,
		shadow->gmemshadow.gpuaddr >> 5);
	
	*cmds++ = _SET(RB_COPYDESTPITCH_COPY_DEST_PITCH,
		(shadow->pitch * 4) / 32);
	
	*cmds++ = _SET(RB_COPYDESTINFO_COPY_DEST_TILE,
		RB_TILINGMODE_LINEAR) |
		_SET(RB_COPYDESTINFO_COPY_DEST_FORMAT, RB_R8G8B8A8_UNORM) |
		_SET(RB_COPYDESTINFO_COPY_COMPONENT_ENABLE, 0X0F) |
		_SET(RB_COPYDESTINFO_COPY_DEST_ENDIAN, RB_ENDIAN_NONE);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_GRAS_SC_CONTROL);
	
	*cmds++ = _SET(GRAS_SC_CONTROL_RENDER_MODE, 2);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_VFD_CONTROL_0);
	
	*cmds++ = _SET(VFD_CTRLREG0_TOTALATTRTOVS, 4) |
		_SET(VFD_CTRLREG0_PACKETSIZE, 2) |
		_SET(VFD_CTRLREG0_STRMDECINSTRCNT, 1) |
		_SET(VFD_CTRLREG0_STRMFETCHINSTRCNT, 1);
	
	*cmds++ = _SET(VFD_CTRLREG1_MAXSTORAGE, 1) |
		_SET(VFD_CTRLREG1_REGID4VTX,  252) |
		_SET(VFD_CTRLREG1_REGID4INST,  252);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_VFD_FETCH_INSTR_0_0);
	
	*cmds++ = _SET(VFD_FETCHINSTRUCTIONS_FETCHSIZE, 11) |
		_SET(VFD_FETCHINSTRUCTIONS_BUFSTRIDE, 12) |
		_SET(VFD_FETCHINSTRUCTIONS_STEPRATE, 1);
	
	*cmds++ = _SET(VFD_BASEADDR_BASEADDR,
		shadow->quad_vertices.gpuaddr);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_VFD_DECODE_INSTR_0);
	
	*cmds++ = _SET(VFD_DECODEINSTRUCTIONS_WRITEMASK, 0x0F) |
		_SET(VFD_DECODEINSTRUCTIONS_CONSTFILL, 1) |
		_SET(VFD_DECODEINSTRUCTIONS_FORMAT, 2) |
		_SET(VFD_DECODEINSTRUCTIONS_SHIFTCNT, 12) |
		_SET(VFD_DECODEINSTRUCTIONS_LASTCOMPVALID, 1);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 5);
	*cmds++ = CP_REG(A3XX_HLSQ_CONTROL_0_REG);
	
	*cmds++ = _SET(HLSQ_CTRL0REG_FSTHREADSIZE, HLSQ_FOUR_PIX_QUADS) |
		_SET(HLSQ_CTRL0REG_FSSUPERTHREADENABLE, 1) |
		_SET(HLSQ_CTRL0REG_RESERVED2, 1) |
		_SET(HLSQ_CTRL0REG_SPCONSTFULLUPDATE, 1);
	
	*cmds++ = _SET(HLSQ_CTRL1REG_VSTHREADSIZE, HLSQ_TWO_VTX_QUADS) |
		_SET(HLSQ_CTRL1REG_VSSUPERTHREADENABLE, 1);
	
	*cmds++ = _SET(HLSQ_CTRL2REG_PRIMALLOCTHRESHOLD, 31);
	
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 5);
	*cmds++ = CP_REG(A3XX_HLSQ_VS_CONTROL_REG);
	
	*cmds++ = _SET(HLSQ_VSCTRLREG_VSINSTRLENGTH, 1);
	
	*cmds++ = _SET(HLSQ_FSCTRLREG_FSCONSTLENGTH, 1) |
		_SET(HLSQ_FSCTRLREG_FSCONSTSTARTOFFSET, 128) |
		_SET(HLSQ_FSCTRLREG_FSINSTRLENGTH, 1);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = _SET(HLSQ_CONSTFSPRESERVEDRANGEREG_STARTENTRY, 32) |
		_SET(HLSQ_CONSTFSPRESERVEDRANGEREG_ENDENTRY, 32);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_SP_FS_LENGTH_REG);
	
	*cmds++ = _SET(SP_SHADERLENGTH_LEN, 1);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_SP_SP_CTRL_REG);
	
	*cmds++ = _SET(SP_SPCTRLREG_SLEEPMODE, 1) |
		_SET(SP_SPCTRLREG_LOMODE, 1);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 12);
	*cmds++ = CP_REG(A3XX_SP_VS_CTRL_REG0);
	
	*cmds++ = _SET(SP_VSCTRLREG0_VSTHREADMODE, SP_MULTI) |
		_SET(SP_VSCTRLREG0_VSINSTRBUFFERMODE, SP_BUFFER_MODE) |
		_SET(SP_VSCTRLREG0_VSICACHEINVALID, 1) |
		_SET(SP_VSCTRLREG0_VSFULLREGFOOTPRINT, 1) |
		_SET(SP_VSCTRLREG0_VSTHREADSIZE, SP_TWO_VTX_QUADS) |
		_SET(SP_VSCTRLREG0_VSSUPERTHREADMODE, 1) |
		_SET(SP_VSCTRLREG0_VSLENGTH, 1);
	
	*cmds++ = _SET(SP_VSCTRLREG1_VSINITIALOUTSTANDING, 4);
	
	*cmds++ = _SET(SP_VSPARAMREG_PSIZEREGID, 252);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 7);
	*cmds++ = CP_REG(A3XX_SP_VS_VPC_DST_REG_0);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 6);
	*cmds++ = CP_REG(A3XX_SP_VS_LENGTH_REG);
	
	*cmds++ = _SET(SP_SHADERLENGTH_LEN, 1);
	
	*cmds++ = _SET(SP_FSCTRLREG0_FSTHREADMODE, SP_MULTI) |
		_SET(SP_FSCTRLREG0_FSINSTRBUFFERMODE, SP_BUFFER_MODE) |
		_SET(SP_FSCTRLREG0_FSICACHEINVALID, 1) |
		_SET(SP_FSCTRLREG0_FSHALFREGFOOTPRINT, 1) |
		_SET(SP_FSCTRLREG0_FSINOUTREGOVERLAP, 1) |
		_SET(SP_FSCTRLREG0_FSTHREADSIZE, SP_FOUR_PIX_QUADS) |
		_SET(SP_FSCTRLREG0_FSSUPERTHREADMODE, 1) |
		_SET(SP_FSCTRLREG0_FSLENGTH, 1);
	
	*cmds++ = _SET(SP_FSCTRLREG1_FSCONSTLENGTH, 1) |
		_SET(SP_FSCTRLREG1_HALFPRECVAROFFSET, 63);
	
	*cmds++ = _SET(SP_OBJOFFSETREG_CONSTOBJECTSTARTOFFSET, 128) |
		_SET(SP_OBJOFFSETREG_SHADEROBJOFFSETINIC, 127);
	
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_SP_FS_FLAT_SHAD_MODE_REG_0);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_SP_FS_OUTPUT_REG);
	
	*cmds++ = _SET(SP_IMAGEOUTPUTREG_DEPTHOUTMODE, SP_PIXEL_BASED);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 5);
	*cmds++ = CP_REG(A3XX_SP_FS_MRT_REG_0);
	
	*cmds++ = _SET(SP_FSMRTREG_PRECISION, 1);

	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 11);
	*cmds++ = CP_REG(A3XX_VPC_ATTR);
	
	*cmds++ = _SET(VPC_VPCATTR_THRHDASSIGN, 1) |
		_SET(VPC_VPCATTR_LMSIZE, 1);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_LOAD_STATE, 10);
	*cmds++ = (0 << CP_LOADSTATE_DSTOFFSET_SHIFT)
		| (HLSQ_DIRECT << CP_LOADSTATE_STATESRC_SHIFT)
		| (HLSQ_BLOCK_ID_SP_VS << CP_LOADSTATE_STATEBLOCKID_SHIFT)
		| (1 << CP_LOADSTATE_NUMOFUNITS_SHIFT);
	*cmds++ = (HLSQ_SP_VS_INSTR << CP_LOADSTATE_STATETYPE_SHIFT)
		| (0 << CP_LOADSTATE_EXTSRCADDR_SHIFT);

	
	*cmds++ = 0x00000000; *cmds++ = 0x13001000;
	
	*cmds++ = 0x00000000; *cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000; *cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000; *cmds++ = 0x00000000;


	*cmds++ = cp_type0_packet(A3XX_VFD_PERFCOUNTER0_SELECT, 1);
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_LOAD_STATE, 10);
	*cmds++ = (0 << CP_LOADSTATE_DSTOFFSET_SHIFT)
		| (HLSQ_DIRECT << CP_LOADSTATE_STATESRC_SHIFT)
		| (HLSQ_BLOCK_ID_SP_FS << CP_LOADSTATE_STATEBLOCKID_SHIFT)
		| (1 << CP_LOADSTATE_NUMOFUNITS_SHIFT);
	*cmds++ = (HLSQ_SP_FS_INSTR << CP_LOADSTATE_STATETYPE_SHIFT)
		| (0 << CP_LOADSTATE_EXTSRCADDR_SHIFT);

	
	*cmds++ = 0x00000000; *cmds++ = 0x30201b00;
	
	*cmds++ = 0x00000000; *cmds++ = 0x03000000;
	
	*cmds++ = 0x00000000; *cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000; *cmds++ = 0x00000000;



	*cmds++ = cp_type0_packet(A3XX_VFD_PERFCOUNTER0_SELECT, 1);
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
	*cmds++ = 0x00000000;


	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_RB_MSAA_CONTROL);
	
	*cmds++ = _SET(RB_MSAACONTROL_MSAA_DISABLE, 1) |
		_SET(RB_MSAACONTROL_SAMPLE_MASK, 0xFFFF);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_RB_DEPTH_CONTROL);
	
	*cmds++ = _SET(RB_DEPTHCONTROL_Z_TEST_FUNC, RB_FRAG_NEVER);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_RB_STENCIL_CONTROL);
	
	*cmds++ = _SET(RB_STENCILCONTROL_STENCIL_FUNC, RB_REF_NEVER) |
		_SET(RB_STENCILCONTROL_STENCIL_FAIL, RB_STENCIL_KEEP) |
		_SET(RB_STENCILCONTROL_STENCIL_ZPASS, RB_STENCIL_KEEP) |
		_SET(RB_STENCILCONTROL_STENCIL_ZFAIL, RB_STENCIL_KEEP) |
		_SET(RB_STENCILCONTROL_STENCIL_FUNC_BF, RB_REF_NEVER) |
		_SET(RB_STENCILCONTROL_STENCIL_FAIL_BF, RB_STENCIL_KEEP) |
		_SET(RB_STENCILCONTROL_STENCIL_ZPASS_BF, RB_STENCIL_KEEP) |
		_SET(RB_STENCILCONTROL_STENCIL_ZFAIL_BF, RB_STENCIL_KEEP);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_GRAS_SU_MODE_CONTROL);
	
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_RB_MRT_CONTROL0);
	
	*cmds++ = _SET(RB_MRTCONTROL_READ_DEST_ENABLE, 1) |
		_SET(RB_MRTCONTROL_ROP_CODE, 12) |
		_SET(RB_MRTCONTROL_DITHER_MODE, RB_DITHER_ALWAYS) |
		_SET(RB_MRTCONTROL_COMPONENT_ENABLE, 0xF);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_RB_MRT_BLEND_CONTROL0);
	
	*cmds++ = _SET(RB_MRTBLENDCONTROL_RGB_SRC_FACTOR, RB_FACTOR_ONE) |
		_SET(RB_MRTBLENDCONTROL_RGB_BLEND_OPCODE, RB_BLEND_OP_ADD) |
		_SET(RB_MRTBLENDCONTROL_RGB_DEST_FACTOR, RB_FACTOR_ZERO) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_SRC_FACTOR, RB_FACTOR_ONE) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_DEST_FACTOR, RB_FACTOR_ZERO) |
		_SET(RB_MRTBLENDCONTROL_CLAMP_ENABLE, 1);
	
	*cmds++ = _SET(RB_MRTCONTROL_READ_DEST_ENABLE, 1) |
		_SET(RB_MRTCONTROL_DITHER_MODE, RB_DITHER_DISABLE) |
		_SET(RB_MRTCONTROL_COMPONENT_ENABLE, 0xF);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_RB_MRT_BLEND_CONTROL1);
	
	*cmds++ = _SET(RB_MRTBLENDCONTROL_RGB_SRC_FACTOR, RB_FACTOR_ONE) |
		_SET(RB_MRTBLENDCONTROL_RGB_BLEND_OPCODE, RB_BLEND_OP_ADD) |
		_SET(RB_MRTBLENDCONTROL_RGB_DEST_FACTOR, RB_FACTOR_ZERO) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_SRC_FACTOR, RB_FACTOR_ONE) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_DEST_FACTOR, RB_FACTOR_ZERO) |
		_SET(RB_MRTBLENDCONTROL_CLAMP_ENABLE, 1);
	
	*cmds++ = _SET(RB_MRTCONTROL_READ_DEST_ENABLE, 1) |
		_SET(RB_MRTCONTROL_DITHER_MODE, RB_DITHER_DISABLE) |
		_SET(RB_MRTCONTROL_COMPONENT_ENABLE, 0xF);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_RB_MRT_BLEND_CONTROL2);
	
	*cmds++ = _SET(RB_MRTBLENDCONTROL_RGB_SRC_FACTOR, RB_FACTOR_ONE) |
		_SET(RB_MRTBLENDCONTROL_RGB_BLEND_OPCODE, RB_BLEND_OP_ADD) |
		_SET(RB_MRTBLENDCONTROL_RGB_DEST_FACTOR, RB_FACTOR_ZERO) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_SRC_FACTOR, RB_FACTOR_ONE) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_DEST_FACTOR, RB_FACTOR_ZERO) |
		_SET(RB_MRTBLENDCONTROL_CLAMP_ENABLE, 1);
	
	*cmds++ = _SET(RB_MRTCONTROL_READ_DEST_ENABLE, 1) |
		_SET(RB_MRTCONTROL_DITHER_MODE, RB_DITHER_DISABLE) |
		_SET(RB_MRTCONTROL_COMPONENT_ENABLE, 0xF);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_RB_MRT_BLEND_CONTROL3);
	
	*cmds++ = _SET(RB_MRTBLENDCONTROL_RGB_SRC_FACTOR, RB_FACTOR_ONE) |
		_SET(RB_MRTBLENDCONTROL_RGB_BLEND_OPCODE, RB_BLEND_OP_ADD) |
		_SET(RB_MRTBLENDCONTROL_RGB_DEST_FACTOR, RB_FACTOR_ZERO) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_SRC_FACTOR, RB_FACTOR_ONE) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_DEST_FACTOR, RB_FACTOR_ZERO) |
		_SET(RB_MRTBLENDCONTROL_CLAMP_ENABLE, 1);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 5);
	*cmds++ = CP_REG(A3XX_VFD_INDEX_MIN);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x155;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_VFD_VS_THREADING_THRESHOLD);
	
	*cmds++ = _SET(VFD_THREADINGTHRESHOLD_REGID_THRESHOLD, 15) |
		_SET(VFD_THREADINGTHRESHOLD_REGID_VTXCNT, 252);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_TPL1_TP_VS_TEX_OFFSET);
	
	*cmds++ = 0;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_TPL1_TP_FS_TEX_OFFSET);
	
	*cmds++ = _SET(TPL1_TPTEXOFFSETREG_SAMPLEROFFSET, 16) |
		_SET(TPL1_TPTEXOFFSETREG_MEMOBJOFFSET, 16) |
		_SET(TPL1_TPTEXOFFSETREG_BASETABLEPTR, 224);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_PC_PRIM_VTX_CNTL);
	
	*cmds++ = _SET(PC_PRIM_VTX_CONTROL_POLYMODE_FRONT_PTYPE,
		PC_DRAW_TRIANGLES) |
		_SET(PC_PRIM_VTX_CONTROL_POLYMODE_BACK_PTYPE,
		PC_DRAW_TRIANGLES) |
		_SET(PC_PRIM_VTX_CONTROL_PROVOKING_VTX_LAST, 1);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_GRAS_SC_WINDOW_SCISSOR_TL);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = _SET(GRAS_SC_WINDOW_SCISSOR_BR_BR_X, shadow->width - 1) |
		_SET(GRAS_SC_WINDOW_SCISSOR_BR_BR_Y, shadow->height - 1);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_GRAS_SC_SCREEN_SCISSOR_TL);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = _SET(GRAS_SC_SCREEN_SCISSOR_BR_BR_X, shadow->width - 1) |
		_SET(GRAS_SC_SCREEN_SCISSOR_BR_BR_Y, shadow->height - 1);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 5);
	*cmds++ = CP_REG(A3XX_GRAS_CL_VPORT_XOFFSET);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = _SET(GRAS_CL_VPORT_XSCALE_VPORT_XSCALE, 0x3f800000);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = _SET(GRAS_CL_VPORT_YSCALE_VPORT_YSCALE, 0x3f800000);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_GRAS_CL_VPORT_ZOFFSET);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = _SET(GRAS_CL_VPORT_ZSCALE_VPORT_ZSCALE, 0x3f800000);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_GRAS_CL_CLIP_CNTL);
	
	*cmds++ = _SET(GRAS_CL_CLIP_CNTL_CLIP_DISABLE, 1) |
		_SET(GRAS_CL_CLIP_CNTL_ZFAR_CLIP_DISABLE, 1) |
		_SET(GRAS_CL_CLIP_CNTL_VP_CLIP_CODE_IGNORE, 1) |
		_SET(GRAS_CL_CLIP_CNTL_VP_XFORM_DISABLE, 1) |
		_SET(GRAS_CL_CLIP_CNTL_PERSP_DIVISION_DISABLE, 1);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_GRAS_CL_GB_CLIP_ADJ);
	
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
	*cmds++ = 0x00000000;


	
	*cmds++ = cp_type0_packet(A3XX_SP_VS_CTRL_REG0, 1);
	*cmds++ = 0x00000400;

	*cmds++ = cp_type0_packet(A3XX_SP_FS_CTRL_REG0, 1);
	*cmds++ = 0x00000400;

	*cmds++ = cp_type0_packet(A3XX_SP_VS_PVT_MEM_SIZE_REG, 1);
	*cmds++ = 0x00008000; 

	*cmds++ = cp_type0_packet(A3XX_SP_FS_PVT_MEM_SIZE_REG, 1);
	*cmds++ = 0x00008000; 

	
	*cmds++ = cp_type0_packet(A3XX_SP_VS_CTRL_REG0, 1);
	*cmds++ = _SET(SP_VSCTRLREG0_VSTHREADMODE, SP_MULTI) |
		_SET(SP_VSCTRLREG0_VSINSTRBUFFERMODE, SP_BUFFER_MODE) |
		_SET(SP_VSCTRLREG0_VSFULLREGFOOTPRINT, 1) |
		_SET(SP_VSCTRLREG0_VSTHREADSIZE, SP_TWO_VTX_QUADS) |
		_SET(SP_VSCTRLREG0_VSSUPERTHREADMODE, 1) |
		_SET(SP_VSCTRLREG0_VSLENGTH, 1);

	*cmds++ = cp_type0_packet(A3XX_SP_FS_CTRL_REG0, 1);
	*cmds++ = _SET(SP_FSCTRLREG0_FSTHREADMODE, SP_MULTI) |
		_SET(SP_FSCTRLREG0_FSINSTRBUFFERMODE, SP_BUFFER_MODE) |
		_SET(SP_FSCTRLREG0_FSHALFREGFOOTPRINT, 1) |
		_SET(SP_FSCTRLREG0_FSINOUTREGOVERLAP, 1) |
		_SET(SP_FSCTRLREG0_FSTHREADSIZE, SP_FOUR_PIX_QUADS) |
		_SET(SP_FSCTRLREG0_FSSUPERTHREADMODE, 1) |
		_SET(SP_FSCTRLREG0_FSLENGTH, 1);

	*cmds++ = cp_type0_packet(A3XX_SP_VS_PVT_MEM_SIZE_REG, 1);
	*cmds++ = 0x00000000;		 

	*cmds++ = cp_type0_packet(A3XX_SP_FS_PVT_MEM_SIZE_REG, 1);
	*cmds++ = 0x00000000;		 

	

	*cmds++ = cp_type3_packet(CP_DRAW_INDX_2, 6);
	*cmds++ = 0x00000000; 
	*cmds++ = BUILD_PC_DRAW_INITIATOR(PC_DI_PT_TRILIST,
					  PC_DI_SRC_SEL_IMMEDIATE,
					  PC_DI_INDEX_SIZE_32_BIT,
					  PC_DI_IGNORE_VISIBILITY);
	*cmds++ = 0x00000003; 
	*cmds++ = 0x00000000; 
	*cmds++ = 0x00000001; 
	*cmds++ = 0x00000002; 

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_HLSQ_CL_CONTROL_0_REG);
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_DRAW_INDX_2, 6);
	*cmds++ = 0x00000000; 
	*cmds++ = BUILD_PC_DRAW_INITIATOR(PC_DI_PT_TRILIST,
					  PC_DI_SRC_SEL_IMMEDIATE,
					  PC_DI_INDEX_SIZE_32_BIT,
					  PC_DI_IGNORE_VISIBILITY);
	*cmds++ = 0x00000003; 
	*cmds++ = 0x00000002; 
	*cmds++ = 0x00000001; 
	*cmds++ = 0x00000003; 

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_HLSQ_CL_CONTROL_0_REG);
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
	*cmds++ = 0x00000000;

	
	create_ib1(drawctxt, shadow->gmem_save, start, cmds);

	return cmds;
}
static void build_shader_save_cmds(struct adreno_device *adreno_dev,
				   struct adreno_context *drawctxt)
{
	unsigned int *cmd = tmp_ctx.cmd;
	unsigned int *start;

	
	drawctxt->cond_execs[0].hostptr = cmd;
	drawctxt->cond_execs[0].gpuaddr = virt2gpu(cmd, &drawctxt->gpustate);
	*cmd++ = 0;
	drawctxt->cond_execs[1].hostptr = cmd;
	drawctxt->cond_execs[1].gpuaddr = virt2gpu(cmd, &drawctxt->gpustate);
	*cmd++ = 0;

	drawctxt->shader_save_commands[0].hostptr = cmd;
	drawctxt->shader_save_commands[0].gpuaddr =
	    virt2gpu(cmd, &drawctxt->gpustate);
	*cmd++ = 0;
	drawctxt->shader_save_commands[1].hostptr = cmd;
	drawctxt->shader_save_commands[1].gpuaddr =
	    virt2gpu(cmd, &drawctxt->gpustate);
	*cmd++ = 0;

	start = cmd;

	

	*cmd++ = cp_type3_packet(CP_COND_EXEC, 4);
	*cmd++ = drawctxt->cond_execs[0].gpuaddr >> 2;
	*cmd++ = drawctxt->cond_execs[0].gpuaddr >> 2;
	*cmd++ = 0x0000FFFF;
	*cmd++ = 3;		

	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	drawctxt->shader_save_commands[2].hostptr = cmd;
	drawctxt->shader_save_commands[2].gpuaddr =
	    virt2gpu(cmd, &drawctxt->gpustate);
	*cmd++ = 0;	
	*cmd++ = (drawctxt->gpustate.gpuaddr + SHADER_OFFSET) & 0xfffffffc;

	
	*cmd++ = cp_type3_packet(CP_COND_EXEC, 4);
	*cmd++ = drawctxt->cond_execs[1].gpuaddr >> 2;
	*cmd++ = drawctxt->cond_execs[1].gpuaddr >> 2;
	*cmd++ = 0x0000FFFF;
	*cmd++ = 3;		

	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	drawctxt->shader_save_commands[3].hostptr = cmd;
	drawctxt->shader_save_commands[3].gpuaddr =
	    virt2gpu(cmd, &drawctxt->gpustate);
	*cmd++ = 0;	
	*cmd++ = (drawctxt->gpustate.gpuaddr + SHADER_OFFSET
		  + (SHADER_SHADOW_SIZE / 2)) & 0xfffffffc;

	
	create_ib1(drawctxt, drawctxt->shader_save, start, cmd);

	tmp_ctx.cmd = cmd;
}


static void build_save_fixup_cmds(struct adreno_device *adreno_dev,
				  struct adreno_context *drawctxt)
{
	unsigned int *cmd = tmp_ctx.cmd;
	unsigned int *start = cmd;

	
	*cmd++ = cp_type3_packet(CP_EVENT_WRITE, 1);
	*cmd++ = 0x7;		
	*cmd++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
	*cmd++ = 0;

	*cmd++ = cp_type0_packet(A3XX_UCHE_CACHE_INVALIDATE0_REG, 2);
	*cmd++ = 0x00000000; 
	*cmd++ = (unsigned int)
		UCHE_ENTIRE_CACHE << UCHE_INVALIDATE1REG_ALLORPORTION |
		UCHE_OP_INVALIDATE << UCHE_INVALIDATE1REG_OPCODE |
		0; 

	
	*cmd++ = cp_type3_packet(CP_CONTEXT_UPDATE, 1);
	*cmd++ = 0;

#ifdef GSL_CONTEXT_SWITCH_CPU_SYNC

	
	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ = A3XX_SP_VS_CTRL_REG0;
	*cmd++ = drawctxt->shader_save_commands[2].gpuaddr;

	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ = A3XX_SP_FS_CTRL_REG0;
	*cmd++ = drawctxt->shader_save_commands[3].gpuaddr;

	
	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ = A3XX_SP_FS_OBJ_OFFSET_REG;
	*cmd++ = drawctxt->shader_save_commands[1].gpuaddr;

	
	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ = A3XX_SP_VS_CTRL_REG1;
	*cmd++ = drawctxt->constant_save_commands[1].gpuaddr;
	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ = A3XX_SP_FS_CTRL_REG1;
	*cmd++ = drawctxt->constant_save_commands[2].gpuaddr;

	
	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ = A3XX_SP_FS_OBJ_OFFSET_REG;
	*cmd++ = drawctxt->constant_save_commands[0].gpuaddr;


	
	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ = A3XX_SP_VS_CTRL_REG0;
	*cmd++ = drawctxt->cond_execs[0].gpuaddr;

	
	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ = A3XX_SP_FS_CTRL_REG0;
	*cmd++ = drawctxt->cond_execs[1].gpuaddr;
#else

	
	cmd = rmw_regtomem(cmd, A3XX_SP_VS_CTRL_REG0, 0x7f000000,
			11+REG_TO_MEM_LOOP_COUNT_SHIFT,
			(HLSQ_SHADOW_BASE + 0x1000) / 4,
			drawctxt->shader_save_commands[2].gpuaddr);

	
	*cmd++ = cp_type3_packet(CP_REG_RMW, 3);
	*cmd++ = (1 << 30) | A3XX_CP_SCRATCH_REG2;
	*cmd++ = 0x00000000;	
	*cmd++ = A3XX_SP_FS_CTRL_REG0;	
	*cmd++ = cp_type3_packet(CP_REG_RMW, 3);
	*cmd++ = ((11 + REG_TO_MEM_LOOP_COUNT_SHIFT) << 24) |
		A3XX_CP_SCRATCH_REG2;
	*cmd++ = 0x7f000000;	
	*cmd++ = (HLSQ_SHADOW_BASE + 0x1000 + SSIZE) / 4;	


	*cmd++ = cp_type3_packet(CP_REG_RMW, 3);
	*cmd++ = (1 << 30) | A3XX_CP_SCRATCH_REG3;
	*cmd++ = 0x00000000;	
	*cmd++ = A3XX_SP_FS_OBJ_OFFSET_REG;	
	*cmd++ = cp_type3_packet(CP_REG_RMW, 3);
	*cmd++ = A3XX_CP_SCRATCH_REG3;
	*cmd++ = 0xfe000000;	
	*cmd++ = 0x00000000;	
	*cmd++ = cp_type3_packet(CP_REG_RMW, 3);
	*cmd++ = (1 << 30) | A3XX_CP_SCRATCH_REG2;
	*cmd++ = 0xffffffff;	
	*cmd++ = A3XX_CP_SCRATCH_REG3;	

	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ = A3XX_CP_SCRATCH_REG2;
	*cmd++ = drawctxt->shader_save_commands[3].gpuaddr;

	
	cmd = rmw_regtomem(cmd, A3XX_SP_VS_CTRL_REG1, 0x000003ff,
			   2 + REG_TO_MEM_LOOP_COUNT_SHIFT,
			   (HLSQ_SHADOW_BASE + 0x2000) / 4,
			   drawctxt->constant_save_commands[1].gpuaddr);

	cmd = rmw_regtomem(cmd, A3XX_SP_FS_CTRL_REG1, 0x000003ff,
			   2 + REG_TO_MEM_LOOP_COUNT_SHIFT,
			   (HLSQ_SHADOW_BASE + 0x2000 + SSIZE) / 4,
			   drawctxt->constant_save_commands[2].gpuaddr);

	cmd = rmw_regtomem(cmd, A3XX_SP_FS_OBJ_OFFSET_REG, 0x00ff0000,
			   18, drawctxt->gpustate.gpuaddr & 0xfffffe00,
			   drawctxt->constant_save_commands[2].gpuaddr
			   + sizeof(unsigned int));

	
	cmd = rmw_regtomem(cmd, A3XX_SP_VS_CTRL_REG1, 0x000003ff,
		0, 0, drawctxt->cond_execs[2].gpuaddr);

	cmd = rmw_regtomem(cmd, A3XX_SP_FS_CTRL_REG1, 0x000003ff,
		0, 0, drawctxt->cond_execs[3].gpuaddr);

	

	cmd = rmw_regtomem(cmd, A3XX_SP_VS_CTRL_REG0, 0x00000002,
			   31, 0, drawctxt->cond_execs[0].gpuaddr);

	
	cmd = rmw_regtomem(cmd, A3XX_SP_FS_CTRL_REG0, 0x00000002,
			   31, 0, drawctxt->cond_execs[1].gpuaddr);

#endif

	create_ib1(drawctxt, drawctxt->save_fixup, start, cmd);

	tmp_ctx.cmd = cmd;
}


static unsigned int *build_sys2gmem_cmds(struct adreno_device *adreno_dev,
					 struct adreno_context *drawctxt,
					 struct gmem_shadow_t *shadow)
{
	unsigned int *cmds = tmp_ctx.cmd;
	unsigned int *start = cmds;

	*cmds++ = cp_type0_packet(A3XX_RBBM_CLOCK_CTL, 1);
	*cmds++ = adreno_a3xx_rbbm_clock_ctl_default(adreno_dev);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 5);
	*cmds++ = CP_REG(A3XX_HLSQ_CONTROL_0_REG);
	
	*cmds++ = _SET(HLSQ_CTRL0REG_FSTHREADSIZE, HLSQ_FOUR_PIX_QUADS) |
		_SET(HLSQ_CTRL0REG_FSSUPERTHREADENABLE, 1) |
		_SET(HLSQ_CTRL0REG_SPSHADERRESTART, 1) |
		_SET(HLSQ_CTRL0REG_CHUNKDISABLE, 1) |
		_SET(HLSQ_CTRL0REG_SPCONSTFULLUPDATE, 1);
	
	*cmds++ = _SET(HLSQ_CTRL1REG_VSTHREADSIZE, HLSQ_TWO_VTX_QUADS) |
		_SET(HLSQ_CTRL1REG_VSSUPERTHREADENABLE, 1);
	
	*cmds++ = _SET(HLSQ_CTRL2REG_PRIMALLOCTHRESHOLD, 31);
	
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_RB_MRT_BUF_INFO0);
	
	*cmds++ = _SET(RB_MRTBUFINFO_COLOR_FORMAT, RB_R8G8B8A8_UNORM) |
		_SET(RB_MRTBUFINFO_COLOR_TILE_MODE, RB_TILINGMODE_32X32) |
		_SET(RB_MRTBUFINFO_COLOR_BUF_PITCH,
		(shadow->gmem_pitch * 4 * 8) / 256);
	
	*cmds++ = _SET(RB_MRTBUFBASE_COLOR_BUF_BASE, tmp_ctx.gmem_base >> 5);

	
	*cmds++ = cp_type3_packet(CP_LOAD_STATE, 4);
	*cmds++ = (16 << CP_LOADSTATE_DSTOFFSET_SHIFT)
		| (HLSQ_DIRECT << CP_LOADSTATE_STATESRC_SHIFT)
		| (HLSQ_BLOCK_ID_TP_TEX << CP_LOADSTATE_STATEBLOCKID_SHIFT)
		| (1 << CP_LOADSTATE_NUMOFUNITS_SHIFT);
	*cmds++ = (HLSQ_TP_TEX_SAMPLERS << CP_LOADSTATE_STATETYPE_SHIFT)
		| (0 << CP_LOADSTATE_EXTSRCADDR_SHIFT);
	*cmds++ = 0x00000240;
	*cmds++ = 0x00000000;

	*cmds++ = cp_type0_packet(A3XX_VFD_PERFCOUNTER0_SELECT, 1);
	*cmds++ = 0x00000000;

	
	*cmds++ = cp_type3_packet(CP_LOAD_STATE, 6);
	*cmds++ = (16 << CP_LOADSTATE_DSTOFFSET_SHIFT)
		| (HLSQ_DIRECT << CP_LOADSTATE_STATESRC_SHIFT)
		| (HLSQ_BLOCK_ID_TP_TEX << CP_LOADSTATE_STATEBLOCKID_SHIFT)
		| (1 << CP_LOADSTATE_NUMOFUNITS_SHIFT);
	*cmds++ = (HLSQ_TP_TEX_MEMOBJ << CP_LOADSTATE_STATETYPE_SHIFT)
		| (0 << CP_LOADSTATE_EXTSRCADDR_SHIFT);
	*cmds++ = 0x4cc06880;
	*cmds++ = shadow->height | (shadow->width << 14);
	*cmds++ = (shadow->pitch*4*8) << 9;
	*cmds++ = 0x00000000;

	*cmds++ = cp_type0_packet(A3XX_VFD_PERFCOUNTER0_SELECT, 1);
	*cmds++ = 0x00000000;

	
	*cmds++ = cp_type3_packet(CP_LOAD_STATE, 16);
	*cmds++ = (224 << CP_LOADSTATE_DSTOFFSET_SHIFT)
		| (HLSQ_DIRECT << CP_LOADSTATE_STATESRC_SHIFT)
		| (HLSQ_BLOCK_ID_TP_MIPMAP << CP_LOADSTATE_STATEBLOCKID_SHIFT)
		| (14 << CP_LOADSTATE_NUMOFUNITS_SHIFT);
	*cmds++ = (HLSQ_TP_MIPMAP_BASE << CP_LOADSTATE_STATETYPE_SHIFT)
		| (0 << CP_LOADSTATE_EXTSRCADDR_SHIFT);
	*cmds++ = shadow->gmemshadow.gpuaddr;
	*cmds++ = 0x00000000;
	*cmds++ = 0x00000000;
	*cmds++ = 0x00000000;
	*cmds++ = 0x00000000;
	*cmds++ = 0x00000000;
	*cmds++ = 0x00000000;
	*cmds++ = 0x00000000;
	*cmds++ = 0x00000000;
	*cmds++ = 0x00000000;
	*cmds++ = 0x00000000;
	*cmds++ = 0x00000000;
	*cmds++ = 0x00000000;
	*cmds++ = 0x00000000;

	*cmds++ = cp_type0_packet(A3XX_VFD_PERFCOUNTER0_SELECT, 1);
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 5);
	*cmds++ = CP_REG(A3XX_HLSQ_VS_CONTROL_REG);
	
	*cmds++ = _SET(HLSQ_VSCTRLREG_VSINSTRLENGTH, 1);
	
	*cmds++ = _SET(HLSQ_FSCTRLREG_FSCONSTLENGTH, 1) |
		_SET(HLSQ_FSCTRLREG_FSCONSTSTARTOFFSET, 128) |
		_SET(HLSQ_FSCTRLREG_FSINSTRLENGTH, 2);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_SP_FS_LENGTH_REG);
	
	*cmds++ = _SET(SP_SHADERLENGTH_LEN, 2);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 12);
	*cmds++ = CP_REG(A3XX_SP_VS_CTRL_REG0);
	
	*cmds++ = _SET(SP_VSCTRLREG0_VSTHREADMODE, SP_MULTI) |
		_SET(SP_VSCTRLREG0_VSINSTRBUFFERMODE, SP_BUFFER_MODE) |
		_SET(SP_VSCTRLREG0_VSICACHEINVALID, 1) |
		_SET(SP_VSCTRLREG0_VSFULLREGFOOTPRINT, 2) |
		_SET(SP_VSCTRLREG0_VSTHREADSIZE, SP_TWO_VTX_QUADS) |
		_SET(SP_VSCTRLREG0_VSLENGTH, 1);
	
	*cmds++ = _SET(SP_VSCTRLREG1_VSINITIALOUTSTANDING, 8);
	
	*cmds++ = _SET(SP_VSPARAMREG_POSREGID, 4) |
		_SET(SP_VSPARAMREG_PSIZEREGID, 252) |
		_SET(SP_VSPARAMREG_TOTALVSOUTVAR, 1);
	
	*cmds++ = _SET(SP_VSOUTREG_COMPMASK0, 3);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 7);
	*cmds++ = CP_REG(A3XX_SP_VS_VPC_DST_REG_0);
	
	*cmds++ = _SET(SP_VSVPCDSTREG_OUTLOC0, 8);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 6);
	*cmds++ = CP_REG(A3XX_SP_VS_LENGTH_REG);
	
	*cmds++ = _SET(SP_SHADERLENGTH_LEN, 1);
	
	*cmds++ = _SET(SP_FSCTRLREG0_FSTHREADMODE, SP_MULTI) |
		_SET(SP_FSCTRLREG0_FSINSTRBUFFERMODE, SP_BUFFER_MODE) |
		_SET(SP_FSCTRLREG0_FSICACHEINVALID, 1) |
		_SET(SP_FSCTRLREG0_FSHALFREGFOOTPRINT, 1) |
		_SET(SP_FSCTRLREG0_FSFULLREGFOOTPRINT, 1) |
		_SET(SP_FSCTRLREG0_FSINOUTREGOVERLAP, 1) |
		_SET(SP_FSCTRLREG0_FSTHREADSIZE, SP_FOUR_PIX_QUADS) |
		_SET(SP_FSCTRLREG0_FSSUPERTHREADMODE, 1) |
		_SET(SP_FSCTRLREG0_PIXLODENABLE, 1) |
		_SET(SP_FSCTRLREG0_FSLENGTH, 2);
	
	*cmds++ = _SET(SP_FSCTRLREG1_FSCONSTLENGTH, 1) |
		_SET(SP_FSCTRLREG1_FSINITIALOUTSTANDING, 2) |
		_SET(SP_FSCTRLREG1_HALFPRECVAROFFSET, 63);
	
	*cmds++ = _SET(SP_OBJOFFSETREG_CONSTOBJECTSTARTOFFSET, 128) |
		_SET(SP_OBJOFFSETREG_SHADEROBJOFFSETINIC, 126);
	
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_SP_FS_FLAT_SHAD_MODE_REG_0);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_SP_FS_OUTPUT_REG);
	
	*cmds++ = _SET(SP_FSOUTREG_PAD0, SP_PIXEL_BASED);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 5);
	*cmds++ = CP_REG(A3XX_SP_FS_MRT_REG_0);
	
	*cmds++ = _SET(SP_FSMRTREG_PRECISION, 1);
	
	*cmds++ = 0;
	
	*cmds++ = 0;
	
	*cmds++ = 0;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 11);
	*cmds++ = CP_REG(A3XX_VPC_ATTR);
	
	*cmds++ = _SET(VPC_VPCATTR_TOTALATTR, 2) |
		_SET(VPC_VPCATTR_THRHDASSIGN, 1) |
		_SET(VPC_VPCATTR_LMSIZE, 1);
	
	*cmds++ = _SET(VPC_VPCPACK_NUMFPNONPOSVAR, 2) |
		_SET(VPC_VPCPACK_NUMNONPOSVSVAR, 2);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = _SET(VPC_VPCVARPSREPLMODE_COMPONENT08, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT09, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0A,	1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0B, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0C, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0D, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0E, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0F, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT10, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT11, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT12, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT13, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT14, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT15, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT16, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT17, 2);
	
	*cmds++ = _SET(VPC_VPCVARPSREPLMODE_COMPONENT08, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT09, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0A,	1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0B, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0C, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0D, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0E, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0F, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT10, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT11, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT12, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT13, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT14, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT15, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT16, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT17, 2);
	
	*cmds++ = _SET(VPC_VPCVARPSREPLMODE_COMPONENT08, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT09, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0A,	1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0B, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0C, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0D, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0E, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0F, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT10, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT11, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT12, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT13, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT14, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT15, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT16, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT17, 2);
	
	*cmds++ = _SET(VPC_VPCVARPSREPLMODE_COMPONENT08, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT09, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0A,	1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0B, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0C, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0D, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0E, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT0F, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT10, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT11, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT12, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT13, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT14, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT15, 2) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT16, 1) |
		_SET(VPC_VPCVARPSREPLMODE_COMPONENT17, 2);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_SP_SP_CTRL_REG);
	
	*cmds++ = _SET(SP_SPCTRLREG_SLEEPMODE, 1) |
		_SET(SP_SPCTRLREG_LOMODE, 1);

	
	*cmds++ = cp_type3_packet(CP_LOAD_STATE, 10);
	*cmds++ = (0 << CP_LOADSTATE_DSTOFFSET_SHIFT)
		| (HLSQ_DIRECT << CP_LOADSTATE_STATESRC_SHIFT)
		| (HLSQ_BLOCK_ID_SP_VS << CP_LOADSTATE_STATEBLOCKID_SHIFT)
		| (1 << CP_LOADSTATE_NUMOFUNITS_SHIFT);
	*cmds++ = (HLSQ_SP_VS_INSTR << CP_LOADSTATE_STATETYPE_SHIFT)
		| (0 << CP_LOADSTATE_EXTSRCADDR_SHIFT);
	
	*cmds++ = 0x00000000; *cmds++ = 0x13001000;
	
	*cmds++ = 0x00000000; *cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000; *cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000; *cmds++ = 0x00000000;

	*cmds++ = cp_type0_packet(A3XX_VFD_PERFCOUNTER0_SELECT, 1);
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
	*cmds++ = 0x00000000;


	
	*cmds++ = cp_type3_packet(CP_LOAD_STATE, 18);
	*cmds++ = (0 << CP_LOADSTATE_DSTOFFSET_SHIFT)
		| (HLSQ_DIRECT << CP_LOADSTATE_STATESRC_SHIFT)
		| (HLSQ_BLOCK_ID_SP_FS << CP_LOADSTATE_STATEBLOCKID_SHIFT)
		| (2 << CP_LOADSTATE_NUMOFUNITS_SHIFT);
	*cmds++ = (HLSQ_SP_FS_INSTR << CP_LOADSTATE_STATETYPE_SHIFT)
		| (0 << CP_LOADSTATE_EXTSRCADDR_SHIFT);
	
	*cmds++ = 0x00002000; *cmds++ = 0x57309902;
	
	*cmds++ = 0x00000000; *cmds++ = 0x00000500;
	
	*cmds++ = 0x00000005; *cmds++ = 0xa0c01f00;
	
	*cmds++ = 0x00000000; *cmds++ = 0x30040b00;
	
	*cmds++ = 0x00000000; *cmds++ = 0x03000000;
	
	*cmds++ = 0x00000000; *cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000; *cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000; *cmds++ = 0x00000000;

	*cmds++ = cp_type0_packet(A3XX_VFD_PERFCOUNTER0_SELECT, 1);
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_VFD_CONTROL_0);
	
	*cmds++ = _SET(VFD_CTRLREG0_TOTALATTRTOVS, 8) |
		_SET(VFD_CTRLREG0_PACKETSIZE, 2) |
		_SET(VFD_CTRLREG0_STRMDECINSTRCNT, 2) |
		_SET(VFD_CTRLREG0_STRMFETCHINSTRCNT, 2);
	
	*cmds++ =  _SET(VFD_CTRLREG1_MAXSTORAGE, 2) |
		_SET(VFD_CTRLREG1_REGID4VTX, 252) |
		_SET(VFD_CTRLREG1_REGID4INST, 252);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 5);
	*cmds++ = CP_REG(A3XX_VFD_FETCH_INSTR_0_0);
	
	*cmds++ = _SET(VFD_FETCHINSTRUCTIONS_FETCHSIZE, 7) |
		_SET(VFD_FETCHINSTRUCTIONS_BUFSTRIDE, 8) |
		_SET(VFD_FETCHINSTRUCTIONS_SWITCHNEXT, 1) |
		_SET(VFD_FETCHINSTRUCTIONS_STEPRATE, 1);
	
	*cmds++ = _SET(VFD_BASEADDR_BASEADDR,
		shadow->quad_vertices_restore.gpuaddr);
	
	*cmds++ = _SET(VFD_FETCHINSTRUCTIONS_FETCHSIZE, 11) |
		_SET(VFD_FETCHINSTRUCTIONS_BUFSTRIDE, 12) |
		_SET(VFD_FETCHINSTRUCTIONS_INDEXDECODE, 1) |
		_SET(VFD_FETCHINSTRUCTIONS_STEPRATE, 1);
	
	*cmds++ = _SET(VFD_BASEADDR_BASEADDR,
		shadow->quad_vertices_restore.gpuaddr + 16);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_VFD_DECODE_INSTR_0);
	
	*cmds++ = _SET(VFD_DECODEINSTRUCTIONS_WRITEMASK, 0x0F) |
		_SET(VFD_DECODEINSTRUCTIONS_CONSTFILL, 1) |
		_SET(VFD_DECODEINSTRUCTIONS_FORMAT, 1) |
		_SET(VFD_DECODEINSTRUCTIONS_SHIFTCNT, 8) |
		_SET(VFD_DECODEINSTRUCTIONS_LASTCOMPVALID, 1) |
		_SET(VFD_DECODEINSTRUCTIONS_SWITCHNEXT, 1);
	
	*cmds++ = _SET(VFD_DECODEINSTRUCTIONS_WRITEMASK, 0x0F) |
		_SET(VFD_DECODEINSTRUCTIONS_CONSTFILL, 1) |
		_SET(VFD_DECODEINSTRUCTIONS_FORMAT, 2) |
		_SET(VFD_DECODEINSTRUCTIONS_REGID, 4) |
		_SET(VFD_DECODEINSTRUCTIONS_SHIFTCNT, 12) |
		_SET(VFD_DECODEINSTRUCTIONS_LASTCOMPVALID, 1);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_RB_DEPTH_CONTROL);
	
	*cmds++ = _SET(RB_DEPTHCONTROL_Z_TEST_FUNC, RB_FRAG_LESS);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_RB_STENCIL_CONTROL);
	
	*cmds++ = _SET(RB_STENCILCONTROL_STENCIL_FUNC, RB_REF_ALWAYS) |
		_SET(RB_STENCILCONTROL_STENCIL_FAIL, RB_STENCIL_KEEP) |
		_SET(RB_STENCILCONTROL_STENCIL_ZPASS, RB_STENCIL_KEEP) |
		_SET(RB_STENCILCONTROL_STENCIL_ZFAIL, RB_STENCIL_KEEP) |
		_SET(RB_STENCILCONTROL_STENCIL_FUNC_BF, RB_REF_ALWAYS) |
		_SET(RB_STENCILCONTROL_STENCIL_FAIL_BF, RB_STENCIL_KEEP) |
		_SET(RB_STENCILCONTROL_STENCIL_ZPASS_BF, RB_STENCIL_KEEP) |
		_SET(RB_STENCILCONTROL_STENCIL_ZFAIL_BF, RB_STENCIL_KEEP);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_RB_MODE_CONTROL);
	
	*cmds++ = _SET(RB_MODECONTROL_RENDER_MODE, RB_RENDERING_PASS) |
		_SET(RB_MODECONTROL_MARB_CACHE_SPLIT_MODE, 1);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_RB_RENDER_CONTROL);
	
	*cmds++ = _SET(RB_RENDERCONTROL_BIN_WIDTH, shadow->width >> 5) |
		_SET(RB_RENDERCONTROL_ALPHA_TEST_FUNC, 7);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_RB_MSAA_CONTROL);
	
	*cmds++ = _SET(RB_MSAACONTROL_MSAA_DISABLE, 1) |
		_SET(RB_MSAACONTROL_SAMPLE_MASK, 0xFFFF);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_RB_MRT_CONTROL0);
	
	*cmds++ = _SET(RB_MRTCONTROL_ROP_CODE, 12) |
		_SET(RB_MRTCONTROL_DITHER_MODE, RB_DITHER_DISABLE) |
		_SET(RB_MRTCONTROL_COMPONENT_ENABLE, 0xF);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_RB_MRT_BLEND_CONTROL0);
	
	*cmds++ = _SET(RB_MRTBLENDCONTROL_RGB_SRC_FACTOR, RB_FACTOR_ONE) |
		_SET(RB_MRTBLENDCONTROL_RGB_BLEND_OPCODE, RB_BLEND_OP_ADD) |
		_SET(RB_MRTBLENDCONTROL_RGB_DEST_FACTOR, RB_FACTOR_ZERO) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_SRC_FACTOR, RB_FACTOR_ONE) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_BLEND_OPCODE, RB_BLEND_OP_ADD) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_DEST_FACTOR, RB_FACTOR_ZERO) |
		_SET(RB_MRTBLENDCONTROL_CLAMP_ENABLE, 1);
	
	*cmds++ = _SET(RB_MRTCONTROL_READ_DEST_ENABLE, 1) |
		_SET(RB_MRTCONTROL_ROP_CODE, 12) |
		_SET(RB_MRTCONTROL_DITHER_MODE, RB_DITHER_ALWAYS) |
		_SET(RB_MRTCONTROL_COMPONENT_ENABLE, 0xF);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_RB_MRT_BLEND_CONTROL1);
	
	*cmds++ = _SET(RB_MRTBLENDCONTROL_RGB_SRC_FACTOR, RB_FACTOR_ONE) |
		_SET(RB_MRTBLENDCONTROL_RGB_BLEND_OPCODE, RB_BLEND_OP_ADD) |
		_SET(RB_MRTBLENDCONTROL_RGB_DEST_FACTOR, RB_FACTOR_ZERO) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_SRC_FACTOR, RB_FACTOR_ONE) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_BLEND_OPCODE, RB_BLEND_OP_ADD) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_DEST_FACTOR, RB_FACTOR_ZERO) |
		_SET(RB_MRTBLENDCONTROL_CLAMP_ENABLE, 1);
	
	*cmds++ = _SET(RB_MRTCONTROL_READ_DEST_ENABLE, 1) |
		_SET(RB_MRTCONTROL_ROP_CODE, 12) |
		_SET(RB_MRTCONTROL_DITHER_MODE, RB_DITHER_ALWAYS) |
		_SET(RB_MRTCONTROL_COMPONENT_ENABLE, 0xF);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_RB_MRT_BLEND_CONTROL2);
	
	*cmds++ = _SET(RB_MRTBLENDCONTROL_RGB_SRC_FACTOR, RB_FACTOR_ONE) |
		_SET(RB_MRTBLENDCONTROL_RGB_BLEND_OPCODE, RB_BLEND_OP_ADD) |
		_SET(RB_MRTBLENDCONTROL_RGB_DEST_FACTOR, RB_FACTOR_ZERO) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_SRC_FACTOR, RB_FACTOR_ONE) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_BLEND_OPCODE, RB_BLEND_OP_ADD) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_DEST_FACTOR, RB_FACTOR_ZERO) |
		_SET(RB_MRTBLENDCONTROL_CLAMP_ENABLE, 1);
	
	*cmds++ = _SET(RB_MRTCONTROL_READ_DEST_ENABLE, 1) |
		_SET(RB_MRTCONTROL_ROP_CODE, 12) |
		_SET(RB_MRTCONTROL_DITHER_MODE, RB_DITHER_ALWAYS) |
		_SET(RB_MRTCONTROL_COMPONENT_ENABLE, 0xF);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_RB_MRT_BLEND_CONTROL3);
	
	*cmds++ = _SET(RB_MRTBLENDCONTROL_RGB_SRC_FACTOR, RB_FACTOR_ONE) |
		_SET(RB_MRTBLENDCONTROL_RGB_BLEND_OPCODE, RB_BLEND_OP_ADD) |
		_SET(RB_MRTBLENDCONTROL_RGB_DEST_FACTOR, RB_FACTOR_ZERO) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_SRC_FACTOR, RB_FACTOR_ONE) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_BLEND_OPCODE, RB_BLEND_OP_ADD) |
		_SET(RB_MRTBLENDCONTROL_ALPHA_DEST_FACTOR, RB_FACTOR_ZERO) |
		_SET(RB_MRTBLENDCONTROL_CLAMP_ENABLE, 1);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 5);
	*cmds++ = CP_REG(A3XX_VFD_INDEX_MIN);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 340;
	
	*cmds++ = 0x00000000;
	
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_VFD_VS_THREADING_THRESHOLD);
	
	*cmds++ = _SET(VFD_THREADINGTHRESHOLD_REGID_THRESHOLD, 15) |
		_SET(VFD_THREADINGTHRESHOLD_REGID_VTXCNT, 252);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_TPL1_TP_VS_TEX_OFFSET);
	
	*cmds++ = 0x00000000;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_TPL1_TP_FS_TEX_OFFSET);
	
	*cmds++ = _SET(TPL1_TPTEXOFFSETREG_SAMPLEROFFSET, 16) |
		_SET(TPL1_TPTEXOFFSETREG_MEMOBJOFFSET, 16) |
		_SET(TPL1_TPTEXOFFSETREG_BASETABLEPTR, 224);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_GRAS_SC_CONTROL);
	
	*cmds++ = 0x04001000;

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_GRAS_SU_MODE_CONTROL);
	
	*cmds++ = _SET(GRAS_SU_CTRLMODE_LINEHALFWIDTH, 2);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_GRAS_SC_WINDOW_SCISSOR_TL);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = _SET(GRAS_SC_WINDOW_SCISSOR_BR_BR_X, shadow->width - 1) |
		_SET(GRAS_SC_WINDOW_SCISSOR_BR_BR_Y, shadow->height - 1);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_GRAS_SC_SCREEN_SCISSOR_TL);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = _SET(GRAS_SC_SCREEN_SCISSOR_BR_BR_X, shadow->width - 1) |
		_SET(GRAS_SC_SCREEN_SCISSOR_BR_BR_Y, shadow->height - 1);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 5);
	*cmds++ = CP_REG(A3XX_GRAS_CL_VPORT_XOFFSET);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = _SET(GRAS_CL_VPORT_XSCALE_VPORT_XSCALE, 0x3F800000);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = _SET(GRAS_CL_VPORT_YSCALE_VPORT_YSCALE, 0x3F800000);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 3);
	*cmds++ = CP_REG(A3XX_GRAS_CL_VPORT_ZOFFSET);
	
	*cmds++ = 0x00000000;
	
	*cmds++ = _SET(GRAS_CL_VPORT_ZSCALE_VPORT_ZSCALE, 0x3F800000);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_GRAS_CL_CLIP_CNTL);
	
	*cmds++ = _SET(GRAS_CL_CLIP_CNTL_IJ_PERSP_CENTER, 1);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_SP_FS_IMAGE_OUTPUT_REG_0);
	
	*cmds++ = _SET(SP_IMAGEOUTPUTREG_MRTFORMAT, SP_R8G8B8A8_UNORM);

	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_PC_PRIM_VTX_CNTL);
	
	*cmds++ = _SET(PC_PRIM_VTX_CONTROL_STRIDE_IN_VPC, 2) |
		_SET(PC_PRIM_VTX_CONTROL_POLYMODE_FRONT_PTYPE,
		PC_DRAW_TRIANGLES) |
		_SET(PC_PRIM_VTX_CONTROL_POLYMODE_BACK_PTYPE,
		PC_DRAW_TRIANGLES) |
		_SET(PC_PRIM_VTX_CONTROL_PROVOKING_VTX_LAST, 1);


	
	*cmds++ = cp_type0_packet(A3XX_SP_VS_CTRL_REG0, 1);
	*cmds++ = 0x00000400;

	*cmds++ = cp_type0_packet(A3XX_SP_FS_CTRL_REG0, 1);
	*cmds++ = 0x00000400;

	*cmds++ = cp_type0_packet(A3XX_SP_VS_PVT_MEM_SIZE_REG, 1);
	*cmds++ = 0x00008000; 

	*cmds++ = cp_type0_packet(A3XX_SP_FS_PVT_MEM_SIZE_REG, 1);
	*cmds++ = 0x00008000; 

	
	*cmds++ = cp_type0_packet(A3XX_SP_VS_CTRL_REG0, 1);
	*cmds++ = _SET(SP_VSCTRLREG0_VSTHREADMODE, SP_MULTI) |
		_SET(SP_VSCTRLREG0_VSINSTRBUFFERMODE, SP_BUFFER_MODE) |
		_SET(SP_VSCTRLREG0_VSFULLREGFOOTPRINT, 2) |
		_SET(SP_VSCTRLREG0_VSTHREADSIZE, SP_TWO_VTX_QUADS) |
		_SET(SP_VSCTRLREG0_VSLENGTH, 1);

	*cmds++ = cp_type0_packet(A3XX_SP_FS_CTRL_REG0, 1);
	*cmds++ = _SET(SP_FSCTRLREG0_FSTHREADMODE, SP_MULTI) |
		_SET(SP_FSCTRLREG0_FSINSTRBUFFERMODE, SP_BUFFER_MODE) |
		_SET(SP_FSCTRLREG0_FSHALFREGFOOTPRINT, 1) |
		_SET(SP_FSCTRLREG0_FSFULLREGFOOTPRINT, 1) |
		_SET(SP_FSCTRLREG0_FSINOUTREGOVERLAP, 1) |
		_SET(SP_FSCTRLREG0_FSTHREADSIZE, SP_FOUR_PIX_QUADS) |
		_SET(SP_FSCTRLREG0_FSSUPERTHREADMODE, 1) |
		_SET(SP_FSCTRLREG0_FSLENGTH, 2);

	*cmds++ = cp_type0_packet(A3XX_SP_VS_PVT_MEM_SIZE_REG, 1);
	*cmds++ = 0x00000000;		 

	*cmds++ = cp_type0_packet(A3XX_SP_FS_PVT_MEM_SIZE_REG, 1);
	*cmds++ = 0x00000000;		 

	

	*cmds++ = cp_type3_packet(CP_DRAW_INDX, 3);
	*cmds++ = 0x00000000; 
	*cmds++ = BUILD_PC_DRAW_INITIATOR(PC_DI_PT_RECTLIST,
					  PC_DI_SRC_SEL_AUTO_INDEX,
					  PC_DI_INDEX_SIZE_16_BIT,
					  PC_DI_IGNORE_VISIBILITY);
	*cmds++ = 0x00000002; 

	
	create_ib1(drawctxt, shadow->gmem_restore, start, cmds);

	return cmds;
}


static void build_regrestore_cmds(struct adreno_device *adreno_dev,
				  struct adreno_context *drawctxt)
{
	unsigned int *start = tmp_ctx.cmd;
	unsigned int *cmd = start;
	unsigned int *lcc_start;

	int i;

	
	*cmd++ = cp_type3_packet(CP_EVENT_WRITE, 1);
	*cmd++ = 0x7;		
	*cmd++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
	*cmd++ = 0;

	*cmd++ = cp_type0_packet(A3XX_UCHE_CACHE_INVALIDATE0_REG, 2);
	*cmd++ = 0x00000000;    
	*cmd++ = (unsigned int)
		UCHE_ENTIRE_CACHE << UCHE_INVALIDATE1REG_ALLORPORTION |
		UCHE_OP_INVALIDATE << UCHE_INVALIDATE1REG_OPCODE |
		0;  

	lcc_start = cmd;

	
	cmd++;

#ifdef CONFIG_MSM_KGSL_DISABLE_SHADOW_WRITES
	
	*cmd++ = ((drawctxt->gpustate.gpuaddr + REG_OFFSET) & 0xFFFFE000) | 1;
#else
	*cmd++ = (drawctxt->gpustate.gpuaddr + REG_OFFSET) & 0xFFFFE000;
#endif

	for (i = 0; i < ARRAY_SIZE(context_register_ranges) / 2; i++) {
		cmd = reg_range(cmd, context_register_ranges[i * 2],
				context_register_ranges[i * 2 + 1]);
	}

	lcc_start[0] = cp_type3_packet(CP_LOAD_CONSTANT_CONTEXT,
					(cmd - lcc_start) - 1);

#ifdef CONFIG_MSM_KGSL_DISABLE_SHADOW_WRITES
	lcc_start[2] |= (0 << 24) | (4 << 16);	
#else
	lcc_start[2] |= (1 << 24) | (4 << 16);
#endif

	for (i = 0; i < ARRAY_SIZE(global_registers); i++) {
		*cmd++ = cp_type0_packet(global_registers[i], 1);
		tmp_ctx.reg_values[i] = virt2gpu(cmd, &drawctxt->gpustate);
		*cmd++ = 0x00000000;
	}

	create_ib1(drawctxt, drawctxt->reg_restore, start, cmd);
	tmp_ctx.cmd = cmd;
}

static void build_constantrestore_cmds(struct adreno_device *adreno_dev,
				       struct adreno_context *drawctxt)
{
	unsigned int *cmd = tmp_ctx.cmd;
	unsigned int *start = cmd;
	unsigned int mode = 4;	
	unsigned int stateblock;
	unsigned int numunits;
	unsigned int statetype;

	drawctxt->cond_execs[2].hostptr = cmd;
	drawctxt->cond_execs[2].gpuaddr = virt2gpu(cmd, &drawctxt->gpustate);
	*cmd++ = 0;
	drawctxt->cond_execs[3].hostptr = cmd;
	drawctxt->cond_execs[3].gpuaddr = virt2gpu(cmd, &drawctxt->gpustate);
	*cmd++ = 0;

#ifndef CONFIG_MSM_KGSL_DISABLE_SHADOW_WRITES
	*cmd++ = cp_type3_packet(CP_LOAD_CONSTANT_CONTEXT, 3);
	*cmd++ = (drawctxt->gpustate.gpuaddr + REG_OFFSET) & 0xFFFFE000;
	*cmd++ = 4 << 16;
	*cmd++ = 0x0;
#endif
	
	*cmd++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmd++ = CP_REG(A3XX_HLSQ_CONTROL_0_REG);
	*cmd++ = 0x68000240;	

#ifndef CONFIG_MSM_KGSL_DISABLE_SHADOW_WRITES
	
	*cmd++ = cp_type3_packet(CP_LOAD_CONSTANT_CONTEXT, 3);
	*cmd++ = (drawctxt->gpustate.gpuaddr + REG_OFFSET) & 0xFFFFE000;
	*cmd++ = (4 << 16) | (1 << 24);
	*cmd++ = 0x0;
#endif

	
	*cmd++ = cp_type3_packet(CP_COND_EXEC, 4);
	*cmd++ = drawctxt->cond_execs[2].gpuaddr >> 2;
	*cmd++ = drawctxt->cond_execs[2].gpuaddr >> 2;
	*cmd++ = 0x0000ffff;
	*cmd++ = 3; 
	*cmd++ = cp_type3_packet(CP_LOAD_STATE, 2);
	drawctxt->constant_load_commands[0].hostptr = cmd;
	drawctxt->constant_load_commands[0].gpuaddr = virt2gpu(cmd,
		&drawctxt->gpustate);


	*cmd++ = 0;		
	*cmd++ = ((drawctxt->gpustate.gpuaddr) & 0xfffffffc) | 1;

	
	*cmd++ = cp_type3_packet(CP_COND_EXEC, 4);
	*cmd++ = drawctxt->cond_execs[3].gpuaddr >> 2;
	*cmd++ = drawctxt->cond_execs[3].gpuaddr >> 2;
	*cmd++ = 0x0000ffff;
	*cmd++ = 3; 
	*cmd++ = cp_type3_packet(CP_LOAD_STATE, 2);
	drawctxt->constant_load_commands[1].hostptr = cmd;
	drawctxt->constant_load_commands[1].gpuaddr =
	    virt2gpu(cmd, &drawctxt->gpustate);

	*cmd++ = 0;		
	drawctxt->constant_load_commands[2].hostptr = cmd;
	drawctxt->constant_load_commands[2].gpuaddr =
	    virt2gpu(cmd, &drawctxt->gpustate);
	*cmd++ = 0;		

	
	stateblock = 0;
	statetype = 1;
	numunits = (TEX_SIZE_MEM_OBJECTS / 7) / 4;

	*cmd++ = cp_type3_packet(CP_LOAD_STATE, 2);
	*cmd++ = (numunits << 22) | (stateblock << 19) | (mode << 16);
	*cmd++ = ((drawctxt->gpustate.gpuaddr + VS_TEX_OFFSET_MEM_OBJECTS)
	    & 0xfffffffc) | statetype;

	
	stateblock = 1;
	statetype = 1;
	numunits = TEX_SIZE_MIPMAP / 4;
	*cmd++ = cp_type3_packet(CP_LOAD_STATE, 2);
	*cmd++ = (numunits << 22) | (stateblock << 19) | (mode << 16);
	*cmd++ = ((drawctxt->gpustate.gpuaddr + VS_TEX_OFFSET_MIPMAP)
	    & 0xfffffffc) | statetype;

	
	stateblock = 0;
	statetype = 0;
	numunits = (TEX_SIZE_SAMPLER_OBJ / 2) / 4;
	*cmd++ = cp_type3_packet(CP_LOAD_STATE, 2);
	*cmd++ = (numunits << 22) | (stateblock << 19) | (mode << 16);
	*cmd++ = ((drawctxt->gpustate.gpuaddr + VS_TEX_OFFSET_SAMPLER_OBJ)
	    & 0xfffffffc) | statetype;

	
	stateblock = 2;
	statetype = 1;
	numunits = (TEX_SIZE_MEM_OBJECTS / 7) / 4;
	*cmd++ = cp_type3_packet(CP_LOAD_STATE, 2);
	*cmd++ = (numunits << 22) | (stateblock << 19) | (mode << 16);
	*cmd++ = ((drawctxt->gpustate.gpuaddr + FS_TEX_OFFSET_MEM_OBJECTS)
	    & 0xfffffffc) | statetype;

	
	stateblock = 3;
	statetype = 1;
	numunits = TEX_SIZE_MIPMAP / 4;
	*cmd++ = cp_type3_packet(CP_LOAD_STATE, 2);
	*cmd++ = (numunits << 22) | (stateblock << 19) | (mode << 16);
	*cmd++ = ((drawctxt->gpustate.gpuaddr + FS_TEX_OFFSET_MIPMAP)
	    & 0xfffffffc) | statetype;

	
	stateblock = 2;
	statetype = 0;
	numunits = (TEX_SIZE_SAMPLER_OBJ / 2) / 4;
	*cmd++ = cp_type3_packet(CP_LOAD_STATE, 2);
	*cmd++ = (numunits << 22) | (stateblock << 19) | (mode << 16);
	*cmd++ = ((drawctxt->gpustate.gpuaddr + FS_TEX_OFFSET_SAMPLER_OBJ)
	    & 0xfffffffc) | statetype;

	create_ib1(drawctxt, drawctxt->constant_restore, start, cmd);
	tmp_ctx.cmd = cmd;
}

static void build_shader_restore_cmds(struct adreno_device *adreno_dev,
				      struct adreno_context *drawctxt)
{
	unsigned int *cmd = tmp_ctx.cmd;
	unsigned int *start = cmd;

	
	*cmd++ = cp_type3_packet(CP_COND_EXEC, 4);
	*cmd++ = drawctxt->cond_execs[0].gpuaddr >> 2;
	*cmd++ = drawctxt->cond_execs[0].gpuaddr >> 2;
	*cmd++ = 1;
	*cmd++ = 3;		

	*cmd++ = cp_type3_packet(CP_LOAD_STATE, 2);
	drawctxt->shader_load_commands[0].hostptr = cmd;
	drawctxt->shader_load_commands[0].gpuaddr =
	    virt2gpu(cmd, &drawctxt->gpustate);
	*cmd++ = 0;		
	*cmd++ = (drawctxt->gpustate.gpuaddr + SHADER_OFFSET) & 0xfffffffc;

	
	*cmd++ = cp_type3_packet(CP_COND_EXEC, 4);
	*cmd++ = drawctxt->cond_execs[1].gpuaddr >> 2;
	*cmd++ = drawctxt->cond_execs[1].gpuaddr >> 2;
	*cmd++ = 1;
	*cmd++ = 3;		

	*cmd++ = cp_type3_packet(CP_LOAD_STATE, 2);
	drawctxt->shader_load_commands[1].hostptr = cmd;
	drawctxt->shader_load_commands[1].gpuaddr =
	    virt2gpu(cmd, &drawctxt->gpustate);
	*cmd++ = 0;		
	*cmd++ = (drawctxt->gpustate.gpuaddr + SHADER_OFFSET
		  + (SHADER_SHADOW_SIZE / 2)) & 0xfffffffc;

	create_ib1(drawctxt, drawctxt->shader_restore, start, cmd);
	tmp_ctx.cmd = cmd;
}

static void build_hlsqcontrol_restore_cmds(struct adreno_device *adreno_dev,
					   struct adreno_context *drawctxt)
{
	unsigned int *cmd = tmp_ctx.cmd;
	unsigned int *start = cmd;

	*cmd++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmd++ = CP_REG(A3XX_HLSQ_CONTROL_0_REG);
	drawctxt->hlsqcontrol_restore_commands[0].hostptr = cmd;
	drawctxt->hlsqcontrol_restore_commands[0].gpuaddr
	    = virt2gpu(cmd, &drawctxt->gpustate);
	*cmd++ = 0;

	
	create_ib1(drawctxt, drawctxt->hlsqcontrol_restore, start, cmd);

	tmp_ctx.cmd = cmd;
}

static void build_restore_fixup_cmds(struct adreno_device *adreno_dev,
				     struct adreno_context *drawctxt)
{
	unsigned int *cmd = tmp_ctx.cmd;
	unsigned int *start = cmd;

#ifdef GSL_CONTEXT_SWITCH_CPU_SYNC
	
	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ = A3XX_SP_VS_CTRL_REG0;
	*cmd++ = drawctxt->shader_load_commands[0].gpuaddr;

	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ = A3XX_SP_FS_CTRL_REG0;
	*cmd++ = drawctxt->shader_load_commands[1].gpuaddr;

	
	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ = A3XX_SP_VS_CTRL_REG1;
	*cmd++ = drawctxt->constant_load_commands[0].gpuaddr;

	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ = A3XX_SP_FS_CTRL_REG1;
	*cmd++ = drawctxt->constant_load_commands[1].gpuaddr;

	
	*cmd++ = cp_type3_packet(CP_REG_TO_MEM, 2);
	*cmd++ = A3XX_SP_FS_OBJ_OFFSET_REG;
	*cmd++ = drawctxt->constant_load_commands[2].gpuaddr;
#else
	
	cmd = rmw_regtomem(cmd, A3XX_SP_VS_CTRL_REG0, 0x7f000000,
			   30, (4 << 19) | (4 << 16),
			   drawctxt->shader_load_commands[0].gpuaddr);

	cmd = rmw_regtomem(cmd, A3XX_SP_FS_CTRL_REG0, 0x7f000000,
			   30, (6 << 19) | (4 << 16),
			   drawctxt->shader_load_commands[1].gpuaddr);

	
	cmd = rmw_regtomem(cmd, A3XX_SP_VS_CTRL_REG1, 0x000003ff,
			   23, (4 << 19) | (4 << 16),
			   drawctxt->constant_load_commands[0].gpuaddr);

	cmd = rmw_regtomem(cmd, A3XX_SP_FS_CTRL_REG1, 0x000003ff,
			   23, (6 << 19) | (4 << 16),
			   drawctxt->constant_load_commands[1].gpuaddr);

	
	cmd = rmw_regtomem(cmd, A3XX_SP_VS_CTRL_REG1, 0x000003ff,
			0, 0, drawctxt->cond_execs[2].gpuaddr);

	cmd = rmw_regtomem(cmd, A3XX_SP_FS_CTRL_REG1, 0x000003ff,
			0, 0, drawctxt->cond_execs[3].gpuaddr);

	
	cmd = rmw_regtomem(cmd, A3XX_SP_FS_OBJ_OFFSET_REG, 0x00ff0000,
			   18, (drawctxt->gpustate.gpuaddr & 0xfffffe00) | 1,
			   drawctxt->constant_load_commands[2].gpuaddr);
#endif


	cmd = rmw_regtomem(cmd,  A3XX_HLSQ_CONTROL_0_REG, 0x9ffffdff,
		0, 0, drawctxt->hlsqcontrol_restore_commands[0].gpuaddr);

	create_ib1(drawctxt, drawctxt->restore_fixup, start, cmd);

	tmp_ctx.cmd = cmd;
}

static int a3xx_create_gpustate_shadow(struct adreno_device *adreno_dev,
				     struct adreno_context *drawctxt)
{
	build_regrestore_cmds(adreno_dev, drawctxt);
	build_constantrestore_cmds(adreno_dev, drawctxt);
	build_hlsqcontrol_restore_cmds(adreno_dev, drawctxt);
	build_regconstantsave_cmds(adreno_dev, drawctxt);
	build_shader_save_cmds(adreno_dev, drawctxt);
	build_shader_restore_cmds(adreno_dev, drawctxt);
	build_restore_fixup_cmds(adreno_dev, drawctxt);
	build_save_fixup_cmds(adreno_dev, drawctxt);

	return 0;
}

static int a3xx_create_gmem_shadow(struct adreno_device *adreno_dev,
				 struct adreno_context *drawctxt)
{
	int result;

	calc_gmemsize(&drawctxt->context_gmem_shadow, adreno_dev->gmem_size);
	tmp_ctx.gmem_base = adreno_dev->gmem_base;

	result = kgsl_allocate(&drawctxt->context_gmem_shadow.gmemshadow,
		drawctxt->base.proc_priv->pagetable,
		drawctxt->context_gmem_shadow.size);

	if (result)
		return result;

	build_quad_vtxbuff(drawctxt, &drawctxt->context_gmem_shadow,
		&tmp_ctx.cmd);

	tmp_ctx.cmd = build_gmem2sys_cmds(adreno_dev, drawctxt,
		&drawctxt->context_gmem_shadow);
	tmp_ctx.cmd = build_sys2gmem_cmds(adreno_dev, drawctxt,
		&drawctxt->context_gmem_shadow);

	kgsl_cache_range_op(&drawctxt->context_gmem_shadow.gmemshadow,
		KGSL_CACHE_OP_FLUSH);

	return 0;
}

static void a3xx_drawctxt_detach(struct adreno_context *drawctxt)
{
	kgsl_sharedmem_free(&drawctxt->gpustate);
	kgsl_sharedmem_free(&drawctxt->context_gmem_shadow.gmemshadow);
}

static int a3xx_drawctxt_save(struct adreno_device *adreno_dev,
			   struct adreno_context *context);

static int a3xx_drawctxt_restore(struct adreno_device *adreno_dev,
			      struct adreno_context *context);

static const struct adreno_context_ops a3xx_legacy_ctx_ops = {
	.save = a3xx_drawctxt_save,
	.restore = a3xx_drawctxt_restore,
	.detach = a3xx_drawctxt_detach,
};


static int a3xx_drawctxt_create(struct adreno_device *adreno_dev,
	struct adreno_context *drawctxt)
{
	int ret;

	if ((drawctxt->base.flags & KGSL_CONTEXT_PREAMBLE) &&
		(drawctxt->base.flags & KGSL_CONTEXT_NO_GMEM_ALLOC)) {
		drawctxt->ops = &adreno_preamble_ctx_ops;
		return 0;
	}
	drawctxt->ops = &a3xx_legacy_ctx_ops;

	ret = kgsl_allocate(&drawctxt->gpustate,
		drawctxt->base.proc_priv->pagetable, CONTEXT_SIZE);

	if (ret)
		return ret;

	kgsl_sharedmem_set(&adreno_dev->dev, &drawctxt->gpustate, 0, 0,
			CONTEXT_SIZE);
	tmp_ctx.cmd = drawctxt->gpustate.hostptr + CMD_OFFSET;

	if (!(drawctxt->base.flags & KGSL_CONTEXT_PREAMBLE)) {
		ret = a3xx_create_gpustate_shadow(adreno_dev, drawctxt);
		if (ret)
			goto done;

		set_bit(ADRENO_CONTEXT_SHADER_SAVE, &drawctxt->priv);
	}

	if (!(drawctxt->base.flags & KGSL_CONTEXT_NO_GMEM_ALLOC))
		ret = a3xx_create_gmem_shadow(adreno_dev, drawctxt);

done:
	if (ret)
		kgsl_sharedmem_free(&drawctxt->gpustate);

	return ret;
}

static int a3xx_drawctxt_save(struct adreno_device *adreno_dev,
			   struct adreno_context *context)
{
	struct kgsl_device *device = &adreno_dev->dev;
	int ret;

	if (context->state == ADRENO_CONTEXT_STATE_INVALID)
		return 0;

	if (!(context->base.flags & KGSL_CONTEXT_PREAMBLE)) {
		
		ret = adreno_ringbuffer_issuecmds(device, context,
			KGSL_CMD_FLAGS_NONE, context->save_fixup, 3);
		if (ret)
			return ret;

		
		ret = adreno_ringbuffer_issuecmds(device, context,
			KGSL_CMD_FLAGS_NONE,
			context->regconstant_save, 3);
		if (ret)
			return ret;

		if (test_bit(ADRENO_CONTEXT_SHADER_SAVE, &context->priv)) {
			
			ret = adreno_ringbuffer_issuecmds(device, context,
				KGSL_CMD_FLAGS_PMODE, context->shader_save, 3);
			if (ret)
				return ret;
			set_bit(ADRENO_CONTEXT_SHADER_RESTORE, &context->priv);
		}
	}

	if (test_bit(ADRENO_CONTEXT_GMEM_SAVE, &context->priv)) {

		kgsl_cffdump_syncmem(context->base.device,
			&context->gpustate,
			context->context_gmem_shadow.gmem_save[1],
			context->context_gmem_shadow.gmem_save[2] << 2, true);

		ret = adreno_ringbuffer_issuecmds(device, context,
					KGSL_CMD_FLAGS_PMODE,
					    context->context_gmem_shadow.
					    gmem_save, 3);
		if (ret)
			return ret;

		set_bit(ADRENO_CONTEXT_GMEM_RESTORE, &context->priv);
	}

	return 0;
}

static int a3xx_drawctxt_restore(struct adreno_device *adreno_dev,
			      struct adreno_context *context)
{
	struct kgsl_device *device = &adreno_dev->dev;
	int ret;

	
	ret = adreno_context_restore(adreno_dev, context);
	if (ret)
		return ret;


	if (test_bit(ADRENO_CONTEXT_GMEM_RESTORE, &context->priv)) {
		kgsl_cffdump_syncmem(context->base.device,
			&context->gpustate,
			context->context_gmem_shadow.gmem_restore[1],
			context->context_gmem_shadow.gmem_restore[2] << 2,
			true);

		ret = adreno_ringbuffer_issuecmds(device, context,
					KGSL_CMD_FLAGS_PMODE,
					    context->context_gmem_shadow.
					    gmem_restore, 3);
		if (ret)
			return ret;
		clear_bit(ADRENO_CONTEXT_GMEM_RESTORE, &context->priv);
	}

	if (!(context->base.flags & KGSL_CONTEXT_PREAMBLE)) {
		ret = adreno_ringbuffer_issuecmds(device, context,
			KGSL_CMD_FLAGS_NONE, context->reg_restore, 3);
		if (ret)
			return ret;

		
		ret = adreno_ringbuffer_issuecmds(device, context,
			KGSL_CMD_FLAGS_NONE,
			context->restore_fixup, 3);
		if (ret)
			return ret;

		ret = adreno_ringbuffer_issuecmds(device, context,
			KGSL_CMD_FLAGS_NONE,
			context->constant_restore, 3);
		if (ret)
			return ret;

		if (test_bit(ADRENO_CONTEXT_SHADER_RESTORE, &context->priv)) {
			ret = adreno_ringbuffer_issuecmds(device, context,
				KGSL_CMD_FLAGS_NONE,
				context->shader_restore, 3);
			if (ret)
				return ret;
		}
		
		ret = adreno_ringbuffer_issuecmds(device, context,
			KGSL_CMD_FLAGS_NONE,
			context->hlsqcontrol_restore, 3);
	}

	return ret;
}

static const unsigned int _a3xx_pwron_fixup_fs_instructions[] = {
	0x00000000, 0x302CC300, 0x00000000, 0x302CC304,
	0x00000000, 0x302CC308, 0x00000000, 0x302CC30C,
	0x00000000, 0x302CC310, 0x00000000, 0x302CC314,
	0x00000000, 0x302CC318, 0x00000000, 0x302CC31C,
	0x00000000, 0x302CC320, 0x00000000, 0x302CC324,
	0x00000000, 0x302CC328, 0x00000000, 0x302CC32C,
	0x00000000, 0x302CC330, 0x00000000, 0x302CC334,
	0x00000000, 0x302CC338, 0x00000000, 0x302CC33C,
	0x00000000, 0x00000400, 0x00020000, 0x63808003,
	0x00060004, 0x63828007, 0x000A0008, 0x6384800B,
	0x000E000C, 0x6386800F, 0x00120010, 0x63888013,
	0x00160014, 0x638A8017, 0x001A0018, 0x638C801B,
	0x001E001C, 0x638E801F, 0x00220020, 0x63908023,
	0x00260024, 0x63928027, 0x002A0028, 0x6394802B,
	0x002E002C, 0x6396802F, 0x00320030, 0x63988033,
	0x00360034, 0x639A8037, 0x003A0038, 0x639C803B,
	0x003E003C, 0x639E803F, 0x00000000, 0x00000400,
	0x00000003, 0x80D60003, 0x00000007, 0x80D60007,
	0x0000000B, 0x80D6000B, 0x0000000F, 0x80D6000F,
	0x00000013, 0x80D60013, 0x00000017, 0x80D60017,
	0x0000001B, 0x80D6001B, 0x0000001F, 0x80D6001F,
	0x00000023, 0x80D60023, 0x00000027, 0x80D60027,
	0x0000002B, 0x80D6002B, 0x0000002F, 0x80D6002F,
	0x00000033, 0x80D60033, 0x00000037, 0x80D60037,
	0x0000003B, 0x80D6003B, 0x0000003F, 0x80D6003F,
	0x00000000, 0x03000000, 0x00000000, 0x00000000,
};

int adreno_a3xx_pwron_fixup_init(struct adreno_device *adreno_dev)
{
	unsigned int *cmds;
	int count = ARRAY_SIZE(_a3xx_pwron_fixup_fs_instructions);
	int ret;

	
	if (test_bit(ADRENO_DEVICE_PWRON_FIXUP, &adreno_dev->priv))
		return 0;

	ret = kgsl_allocate_contiguous(&adreno_dev->pwron_fixup, PAGE_SIZE);

	if (ret)
		return ret;

	adreno_dev->pwron_fixup.flags |= KGSL_MEMFLAGS_GPUREADONLY;

	cmds = adreno_dev->pwron_fixup.hostptr;

	*cmds++ = cp_type0_packet(A3XX_UCHE_CACHE_INVALIDATE0_REG, 2);
	*cmds++ = 0x00000000;
	*cmds++ = 0x90000000;
	*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type3_packet(CP_REG_RMW, 3);
	*cmds++ = A3XX_RBBM_CLOCK_CTL;
	*cmds++ = 0xFFFCFFFF;
	*cmds++ = 0x00010000;
	*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CONTROL_0_REG, 1);
	*cmds++ = 0x1E000150;
	*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_HLSQ_CONTROL_0_REG);
	*cmds++ = 0x1E000150;
	*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CONTROL_0_REG, 1);
	*cmds++ = 0x1E000150;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CONTROL_1_REG, 1);
	*cmds++ = 0x00000040;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CONTROL_2_REG, 1);
	*cmds++ = 0x80000000;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CONTROL_3_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_VS_CONTROL_REG, 1);
	*cmds++ = 0x00000001;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_FS_CONTROL_REG, 1);
	*cmds++ = 0x0D001002;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CONST_VSPRESV_RANGE_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CONST_FSPRESV_RANGE_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CL_NDRANGE_0_REG, 1);
	*cmds++ = 0x00401101;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CL_NDRANGE_1_REG, 1);
	*cmds++ = 0x00000400;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CL_NDRANGE_2_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CL_NDRANGE_3_REG, 1);
	*cmds++ = 0x00000001;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CL_NDRANGE_4_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CL_NDRANGE_5_REG, 1);
	*cmds++ = 0x00000001;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CL_NDRANGE_6_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CL_CONTROL_0_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CL_CONTROL_1_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CL_KERNEL_CONST_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CL_KERNEL_GROUP_X_REG, 1);
	*cmds++ = 0x00000010;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CL_KERNEL_GROUP_Y_REG, 1);
	*cmds++ = 0x00000001;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CL_KERNEL_GROUP_Z_REG, 1);
	*cmds++ = 0x00000001;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CL_WG_OFFSET_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_SP_CTRL_REG, 1);
	*cmds++ = 0x00040000;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_CTRL_REG0, 1);
	*cmds++ = 0x0000000A;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_CTRL_REG1, 1);
	*cmds++ = 0x00000001;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_PARAM_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_OUT_REG_0, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_OUT_REG_1, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_OUT_REG_2, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_OUT_REG_3, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_OUT_REG_4, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_OUT_REG_5, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_OUT_REG_6, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_OUT_REG_7, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_VPC_DST_REG_0, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_VPC_DST_REG_1, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_VPC_DST_REG_2, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_VPC_DST_REG_3, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_OBJ_OFFSET_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_OBJ_START_REG, 1);
	*cmds++ = 0x00000004;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_PVT_MEM_PARAM_REG, 1);
	*cmds++ = 0x04008001;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_PVT_MEM_ADDR_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_PVT_MEM_SIZE_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_VS_LENGTH_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_FS_CTRL_REG0, 1);
	*cmds++ = 0x0DB0400A;
	*cmds++ = cp_type0_packet(A3XX_SP_FS_CTRL_REG1, 1);
	*cmds++ = 0x00300402;
	*cmds++ = cp_type0_packet(A3XX_SP_FS_OBJ_OFFSET_REG, 1);
	*cmds++ = 0x00010000;
	*cmds++ = cp_type0_packet(A3XX_SP_FS_OBJ_START_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_FS_PVT_MEM_PARAM_REG, 1);
	*cmds++ = 0x04008001;
	*cmds++ = cp_type0_packet(A3XX_SP_FS_PVT_MEM_ADDR_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_FS_PVT_MEM_SIZE_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_FS_FLAT_SHAD_MODE_REG_0, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_FS_FLAT_SHAD_MODE_REG_1, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_FS_OUTPUT_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_FS_MRT_REG_0, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_FS_MRT_REG_1, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_FS_MRT_REG_2, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_FS_MRT_REG_3, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_FS_IMAGE_OUTPUT_REG_0, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_FS_IMAGE_OUTPUT_REG_1, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_FS_IMAGE_OUTPUT_REG_2, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_FS_IMAGE_OUTPUT_REG_3, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_SP_FS_LENGTH_REG, 1);
	*cmds++ = 0x0000000D;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_CLIP_CNTL, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_GB_CLIP_ADJ, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_VPORT_XOFFSET, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_VPORT_XSCALE, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_VPORT_YOFFSET, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_VPORT_YSCALE, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_VPORT_ZOFFSET, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_VPORT_ZSCALE, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_X0, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_Y0, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_Z0, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_W0, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_X1, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_Y1, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_Z1, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_W1, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_X2, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_Y2, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_Z2, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_W2, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_X3, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_Y3, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_Z3, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_W3, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_X4, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_Y4, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_Z4, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_W4, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_X5, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_Y5, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_Z5, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_CL_USER_PLANE_W5, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_SU_POINT_MINMAX, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_SU_POINT_SIZE, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_SU_POLY_OFFSET_OFFSET, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_SU_POLY_OFFSET_SCALE, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_SU_MODE_CONTROL, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_SC_CONTROL, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_SC_SCREEN_SCISSOR_TL, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_SC_SCREEN_SCISSOR_BR, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_SC_WINDOW_SCISSOR_BR, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_SC_WINDOW_SCISSOR_TL, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_TSE_DEBUG_ECO, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_PERFCOUNTER0_SELECT, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_PERFCOUNTER1_SELECT, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_PERFCOUNTER2_SELECT, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_GRAS_PERFCOUNTER3_SELECT, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_MODE_CONTROL, 1);
	*cmds++ = 0x00008000;
	*cmds++ = cp_type0_packet(A3XX_RB_RENDER_CONTROL, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_MSAA_CONTROL, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_ALPHA_REFERENCE, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_MRT_CONTROL0, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_MRT_CONTROL1, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_MRT_CONTROL2, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_MRT_CONTROL3, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_MRT_BUF_INFO0, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_MRT_BUF_INFO1, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_MRT_BUF_INFO2, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_MRT_BUF_INFO3, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_MRT_BUF_BASE0, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_MRT_BUF_BASE1, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_MRT_BUF_BASE2, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_MRT_BUF_BASE3, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_MRT_BLEND_CONTROL0, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_MRT_BLEND_CONTROL1, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_MRT_BLEND_CONTROL2, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_MRT_BLEND_CONTROL3, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_BLEND_RED, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_BLEND_GREEN, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_BLEND_BLUE, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_BLEND_ALPHA, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_CLEAR_COLOR_DW0, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_CLEAR_COLOR_DW1, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_CLEAR_COLOR_DW2, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_CLEAR_COLOR_DW3, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_COPY_CONTROL, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_COPY_DEST_BASE, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_COPY_DEST_PITCH, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_COPY_DEST_INFO, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_DEPTH_CONTROL, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_DEPTH_CLEAR, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_DEPTH_BUF_INFO, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_DEPTH_BUF_PITCH, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_STENCIL_CONTROL, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_STENCIL_CLEAR, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_STENCIL_BUF_INFO, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_STENCIL_BUF_PITCH, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_STENCIL_REF_MASK, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_STENCIL_REF_MASK_BF, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_LRZ_VSC_CONTROL, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_WINDOW_OFFSET, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_SAMPLE_COUNT_CONTROL, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_SAMPLE_COUNT_ADDR, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_Z_CLAMP_MIN, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_Z_CLAMP_MAX, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_GMEM_BASE_ADDR, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_DEBUG_ECO_CONTROLS_ADDR, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_PERFCOUNTER0_SELECT, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_PERFCOUNTER1_SELECT, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_RB_FRAME_BUFFER_DIMENSION, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type3_packet(CP_LOAD_STATE, 4);
	*cmds++ = (1 << CP_LOADSTATE_DSTOFFSET_SHIFT) |
		(0 << CP_LOADSTATE_STATESRC_SHIFT) |
		(6 << CP_LOADSTATE_STATEBLOCKID_SHIFT) |
		(1 << CP_LOADSTATE_NUMOFUNITS_SHIFT);
	*cmds++ = (1 << CP_LOADSTATE_STATETYPE_SHIFT) |
		(0 << CP_LOADSTATE_EXTSRCADDR_SHIFT);
	*cmds++ = 0x00400000;
	*cmds++ = 0x00000000;
	*cmds++ = cp_type3_packet(CP_LOAD_STATE, 4);
	*cmds++ = (2 << CP_LOADSTATE_DSTOFFSET_SHIFT) |
		(6 << CP_LOADSTATE_STATEBLOCKID_SHIFT) |
		(1 << CP_LOADSTATE_NUMOFUNITS_SHIFT);
	*cmds++ = (1 << CP_LOADSTATE_STATETYPE_SHIFT);
	*cmds++ = 0x00400220;
	*cmds++ = 0x00000000;
	*cmds++ = cp_type3_packet(CP_LOAD_STATE, 4);
	*cmds++ = (6 << CP_LOADSTATE_STATEBLOCKID_SHIFT) |
		(1 << CP_LOADSTATE_NUMOFUNITS_SHIFT);
	*cmds++ = (1 << CP_LOADSTATE_STATETYPE_SHIFT);
	*cmds++ = 0x00000000;
	*cmds++ = 0x00000000;
	*cmds++ = cp_type3_packet(CP_LOAD_STATE, 2 + count);
	*cmds++ = (6 << CP_LOADSTATE_STATEBLOCKID_SHIFT) |
		(13 << CP_LOADSTATE_NUMOFUNITS_SHIFT);
	*cmds++ = 0x00000000;

	memcpy(cmds, _a3xx_pwron_fixup_fs_instructions, count << 2);

	cmds += count;

	*cmds++ = cp_type3_packet(CP_EXEC_CL, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CL_CONTROL_0_REG, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type0_packet(A3XX_HLSQ_CONTROL_0_REG, 1);
	*cmds++ = 0x1E000150;
	*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
	*cmds++ = CP_REG(A3XX_HLSQ_CONTROL_0_REG);
	*cmds++ = 0x1E000050;
	*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
	*cmds++ = 0x00000000;
	*cmds++ = cp_type3_packet(CP_REG_RMW, 3);
	*cmds++ = A3XX_RBBM_CLOCK_CTL;
	*cmds++ = 0xFFFCFFFF;
	*cmds++ = 0x00000000;
	*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
	*cmds++ = 0x00000000;

	adreno_dev->pwron_fixup_dwords =
		(cmds - (unsigned int *) adreno_dev->pwron_fixup.hostptr);

	
	set_bit(ADRENO_DEVICE_PWRON_FIXUP, &adreno_dev->priv);
	return 0;
}

static int a3xx_rb_init(struct adreno_device *adreno_dev,
			 struct adreno_ringbuffer *rb)
{
	unsigned int *cmds, cmds_gpu;
	cmds = adreno_ringbuffer_allocspace(rb, NULL, 18);
	if (cmds == NULL)
		return -ENOMEM;

	cmds_gpu = rb->buffer_desc.gpuaddr + sizeof(uint) * (rb->wptr - 18);

	GSL_RB_WRITE(rb->device, cmds, cmds_gpu,
			cp_type3_packet(CP_ME_INIT, 17));
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, 0x000003f7);
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, 0x00000000);
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, 0x00000000);
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, 0x00000000);
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, 0x00000080);
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, 0x00000100);
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, 0x00000180);
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, 0x00006600);
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, 0x00000150);
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, 0x0000014e);
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, 0x00000154);
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, 0x00000001);
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, 0x00000000);
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, 0x00000000);
	
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, 0x20000000);
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, 0x00000000);
	GSL_RB_WRITE(rb->device, cmds, cmds_gpu, 0x00000000);

	adreno_ringbuffer_submit(rb);

	return 0;
}

static void a3xx_fatal_err_callback(struct adreno_device *adreno_dev, int bit)
{
	struct kgsl_device *device = &adreno_dev->dev;
	const char *err = "";

	switch (bit) {
	case A3XX_INT_MISC_HANG_DETECT:
		err = "MISC: GPU hang detected\n";
		break;
	default:
		return;
	}
	KGSL_DRV_CRIT(device, "%s\n", err);
	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_OFF);

	
	adreno_dispatcher_irq_fault(device);
}

static void a3xx_err_callback(struct adreno_device *adreno_dev, int bit)
{
	struct kgsl_device *device = &adreno_dev->dev;

	switch (bit) {
	case A3XX_INT_RBBM_AHB_ERROR: {
		unsigned int reg;

		kgsl_regread(device, A3XX_RBBM_AHB_ERROR_STATUS, &reg);


		KGSL_DRV_CRIT(device,
			"RBBM | AHB bus error | %s | addr=%x | ports=%x:%x\n",
			reg & (1 << 28) ? "WRITE" : "READ",
			(reg & 0xFFFFF) >> 2, (reg >> 20) & 0x3,
			(reg >> 24) & 0x3);

		
		kgsl_regwrite(device, A3XX_RBBM_AHB_CMD, (1 << 3));
		break;
	}
	case A3XX_INT_RBBM_REG_TIMEOUT:
		KGSL_DRV_CRIT_RATELIMIT(device, "RBBM: AHB register timeout\n");
		break;
	case A3XX_INT_RBBM_ME_MS_TIMEOUT:
		KGSL_DRV_CRIT_RATELIMIT(device,
			"RBBM: ME master split timeout\n");
		break;
	case A3XX_INT_RBBM_PFP_MS_TIMEOUT:
		KGSL_DRV_CRIT_RATELIMIT(device,
			"RBBM: PFP master split timeout\n");
		break;
	case A3XX_INT_RBBM_ATB_BUS_OVERFLOW:
		KGSL_DRV_CRIT_RATELIMIT(device,
			"RBBM: ATB bus oveflow\n");
		break;
	case A3XX_INT_VFD_ERROR:
		KGSL_DRV_CRIT_RATELIMIT(device,
			"VFD: Out of bounds access\n");
		break;
	case A3XX_INT_CP_T0_PACKET_IN_IB:
		KGSL_DRV_CRIT_RATELIMIT(device,
			"ringbuffer TO packet in IB interrupt\n");
		break;
	case A3XX_INT_CP_OPCODE_ERROR:
		KGSL_DRV_CRIT_RATELIMIT(device,
			"ringbuffer opcode error interrupt\n");
		break;
	case A3XX_INT_CP_RESERVED_BIT_ERROR:
		KGSL_DRV_CRIT_RATELIMIT(device,
				"ringbuffer reserved bit error interrupt\n");
		break;
	case A3XX_INT_CP_HW_FAULT: {
		unsigned int reg;
		adreno_readreg(adreno_dev, ADRENO_REG_CP_HW_FAULT, &reg);
		KGSL_DRV_CRIT_RATELIMIT(device,
			"CP | Ringbuffer HW fault | status=%x\n", reg);
		break;
	}
	case A3XX_INT_CP_REG_PROTECT_FAULT: {
		unsigned int reg;
		kgsl_regread(device, A3XX_CP_PROTECT_STATUS, &reg);

		KGSL_DRV_CRIT(device,
			"CP | Protected mode error| %s | addr=%x\n",
			reg & (1 << 24) ? "WRITE" : "READ",
			(reg & 0x1FFFF) >> 2);
		break;
	}
	case A3XX_INT_CP_AHB_ERROR_HALT:
		KGSL_DRV_CRIT(device, "ringbuffer AHB error interrupt\n");
		break;
	case A3XX_INT_UCHE_OOB_ACCESS:
		KGSL_DRV_CRIT_RATELIMIT(device,
			"UCHE:  Out of bounds access\n");
		break;
	}
}

static void a3xx_cp_callback(struct adreno_device *adreno_dev, int irq)
{
	struct kgsl_device *device = &adreno_dev->dev;

	device->pwrctrl.irq_last = 1;
	queue_work(device->work_queue, &device->ts_expired_ws);
	adreno_dispatcher_schedule(device);
}


static int a3xx_perfcounter_enable_pwr(struct kgsl_device *device,
	unsigned int counter)
{
	unsigned int in, out;

	if (counter > 1)
		return -EINVAL;

	kgsl_regread(device, A3XX_RBBM_RBBM_CTL, &in);

	if (counter == 0)
		out = in | RBBM_RBBM_CTL_RESET_PWR_CTR0;
	else
		out = in | RBBM_RBBM_CTL_RESET_PWR_CTR1;

	kgsl_regwrite(device, A3XX_RBBM_RBBM_CTL, out);

	if (counter == 0)
		out = in | RBBM_RBBM_CTL_ENABLE_PWR_CTR0;
	else
		out = in | RBBM_RBBM_CTL_ENABLE_PWR_CTR1;

	kgsl_regwrite(device, A3XX_RBBM_RBBM_CTL, out);

	return 0;
}

static int a3xx_perfcounter_enable_vbif(struct kgsl_device *device,
					 unsigned int counter,
					 unsigned int countable)
{
	unsigned int in, out, bit, sel;

	if (counter > 1 || countable > 0x7f)
		return -EINVAL;

	kgsl_regread(device, A3XX_VBIF_PERF_CNT_EN, &in);
	kgsl_regread(device, A3XX_VBIF_PERF_CNT_SEL, &sel);

	if (counter == 0) {
		bit = VBIF_PERF_CNT_0;
		sel = (sel & ~VBIF_PERF_CNT_0_SEL_MASK) | countable;
	} else {
		bit = VBIF_PERF_CNT_1;
		sel = (sel & ~VBIF_PERF_CNT_1_SEL_MASK)
			| (countable << VBIF_PERF_CNT_1_SEL);
	}

	out = in | bit;

	kgsl_regwrite(device, A3XX_VBIF_PERF_CNT_SEL, sel);

	kgsl_regwrite(device, A3XX_VBIF_PERF_CNT_CLR, bit);
	kgsl_regwrite(device, A3XX_VBIF_PERF_CNT_CLR, 0);

	kgsl_regwrite(device, A3XX_VBIF_PERF_CNT_EN, out);
	return 0;
}

static int a3xx_perfcounter_enable_vbif_pwr(struct kgsl_device *device,
					     unsigned int counter)
{
	unsigned int in, out, bit;

	if (counter > 2)
		return -EINVAL;

	kgsl_regread(device, A3XX_VBIF_PERF_CNT_EN, &in);
	if (counter == 0)
		bit = VBIF_PERF_PWR_CNT_0;
	else if (counter == 1)
		bit = VBIF_PERF_PWR_CNT_1;
	else
		bit = VBIF_PERF_PWR_CNT_2;

	out = in | bit;

	kgsl_regwrite(device, A3XX_VBIF_PERF_CNT_CLR, bit);
	kgsl_regwrite(device, A3XX_VBIF_PERF_CNT_CLR, 0);

	kgsl_regwrite(device, A3XX_VBIF_PERF_CNT_EN, out);
	return 0;
}


static int a3xx_perfcounter_enable(struct adreno_device *adreno_dev,
	unsigned int group, unsigned int counter, unsigned int countable)
{
	struct kgsl_device *device = &adreno_dev->dev;
	struct adreno_perfcount_register *reg;

	
	if (group == KGSL_PERFCOUNTER_GROUP_PWR)
		return a3xx_perfcounter_enable_pwr(device, counter);
	else if (group == KGSL_PERFCOUNTER_GROUP_VBIF)
		return a3xx_perfcounter_enable_vbif(device, counter,
							countable);
	else if (group == KGSL_PERFCOUNTER_GROUP_VBIF_PWR)
		return a3xx_perfcounter_enable_vbif_pwr(device, counter);

	if (group >= adreno_dev->gpudev->perfcounters->group_count)
		return -EINVAL;

	if ((0 == adreno_dev->gpudev->perfcounters->groups[group].reg_count) ||
		(counter >=
		adreno_dev->gpudev->perfcounters->groups[group].reg_count))
		return -EINVAL;

	reg = &(adreno_dev->gpudev->perfcounters->groups[group].regs[counter]);

	
	kgsl_regwrite(device, reg->select, countable);

	return 0;
}

static uint64_t a3xx_perfcounter_read_pwr(struct adreno_device *adreno_dev,
				unsigned int counter)
{
	struct kgsl_device *device = &adreno_dev->dev;
	struct adreno_perfcounters *counters = adreno_dev->gpudev->perfcounters;
	struct adreno_perfcount_register *reg;
	unsigned int in, out, lo = 0, hi = 0;
	unsigned int enable_bit;

	if (counter > 1)
		return 0;
	if (0 == counter)
		enable_bit = RBBM_RBBM_CTL_ENABLE_PWR_CTR0;
	else
		enable_bit = RBBM_RBBM_CTL_ENABLE_PWR_CTR1;
	
	adreno_readreg(adreno_dev, ADRENO_REG_RBBM_RBBM_CTL, &in);
	out = (in & ~enable_bit);
	adreno_writereg(adreno_dev, ADRENO_REG_RBBM_RBBM_CTL, out);

	reg = &counters->groups[KGSL_PERFCOUNTER_GROUP_PWR].regs[counter];
	kgsl_regread(device, reg->offset, &lo);
	kgsl_regread(device, reg->offset + 1, &hi);

	
	adreno_writereg(adreno_dev, ADRENO_REG_RBBM_RBBM_CTL, in);

	return (((uint64_t) hi) << 32) | lo;
}

static uint64_t a3xx_perfcounter_read_vbif(struct adreno_device *adreno_dev,
				unsigned int counter)
{
	struct adreno_perfcounters *counters = adreno_dev->gpudev->perfcounters;
	struct kgsl_device *device = &adreno_dev->dev;
	struct adreno_perfcount_register *reg;
	unsigned int in, out, lo = 0, hi = 0;

	if (counter > 1)
		return 0;

	
	kgsl_regread(device, A3XX_VBIF_PERF_CNT_EN, &in);
	if (counter == 0)
		out = (in & ~VBIF_PERF_CNT_0);
	else
		out = (in & ~VBIF_PERF_CNT_1);
	kgsl_regwrite(device, A3XX_VBIF_PERF_CNT_EN, out);

	reg = &counters->groups[KGSL_PERFCOUNTER_GROUP_VBIF].regs[counter];
	kgsl_regread(device, reg->offset, &lo);
	kgsl_regread(device, reg->offset + 1, &hi);

	
	kgsl_regwrite(device, A3XX_VBIF_PERF_CNT_EN, in);

	return (((uint64_t) hi) << 32) | lo;
}

static uint64_t a3xx_perfcounter_read_vbif_pwr(struct adreno_device *adreno_dev,
				unsigned int counter)
{
	struct adreno_perfcounters *counters = adreno_dev->gpudev->perfcounters;
	struct kgsl_device *device = &adreno_dev->dev;
	struct adreno_perfcount_register *reg;
	unsigned int in, out, lo = 0, hi = 0;

	if (counter > 2)
		return 0;

	
	kgsl_regread(device, A3XX_VBIF_PERF_CNT_EN, &in);
	if (0 == counter)
		out = (in & ~VBIF_PERF_PWR_CNT_0);
	else
		out = (in & ~VBIF_PERF_PWR_CNT_2);
	kgsl_regwrite(device, A3XX_VBIF_PERF_CNT_EN, out);

	reg = &counters->groups[KGSL_PERFCOUNTER_GROUP_VBIF_PWR].regs[counter];
	kgsl_regread(device, reg->offset, &lo);
	kgsl_regread(device, reg->offset + 1, &hi);
	
	kgsl_regwrite(device, A3XX_VBIF_PERF_CNT_EN, in);

	return (((uint64_t) hi) << 32) | lo;
}

static uint64_t a3xx_perfcounter_read(struct adreno_device *adreno_dev,
	unsigned int group, unsigned int counter)
{
	struct kgsl_device *device = &adreno_dev->dev;
	struct adreno_perfcount_register *reg;
	unsigned int lo = 0, hi = 0;
	unsigned int offset;
	unsigned int in, out;

	if (group == KGSL_PERFCOUNTER_GROUP_VBIF_PWR)
		return a3xx_perfcounter_read_vbif_pwr(adreno_dev, counter);

	if (group == KGSL_PERFCOUNTER_GROUP_VBIF)
		return a3xx_perfcounter_read_vbif(adreno_dev, counter);

	if (group == KGSL_PERFCOUNTER_GROUP_PWR)
		return a3xx_perfcounter_read_pwr(adreno_dev, counter);

	if (group >= adreno_dev->gpudev->perfcounters->group_count)
		return 0;

	if ((0 == adreno_dev->gpudev->perfcounters->groups[group].reg_count) ||
		(counter >=
		adreno_dev->gpudev->perfcounters->groups[group].reg_count))
		return 0;

	reg = &(adreno_dev->gpudev->perfcounters->groups[group].regs[counter]);

	
	kgsl_regread(device, A3XX_RBBM_PERFCTR_CTL, &in);
	out = in & ~RBBM_PERFCTR_CTL_ENABLE;
	kgsl_regwrite(device, A3XX_RBBM_PERFCTR_CTL, out);

	offset = reg->offset;
	
	kgsl_regread(device, offset, &lo);
	kgsl_regread(device, offset + 1, &hi);

	
	kgsl_regwrite(device, A3XX_RBBM_PERFCTR_CTL, in);
	return (((uint64_t) hi) << 32) | lo;
}

static inline int loadable_perfcounter_group(unsigned int groupid)
{
	return ((groupid == KGSL_PERFCOUNTER_GROUP_VBIF_PWR) ||
		(groupid == KGSL_PERFCOUNTER_GROUP_VBIF) ||
		(groupid == KGSL_PERFCOUNTER_GROUP_PWR)) ? 0 : 1;
}

static inline int active_countable(unsigned int countable)
{
	return ((countable != KGSL_PERFCOUNTER_NOT_USED) &&
		(countable != KGSL_PERFCOUNTER_BROKEN));
}

static void a3xx_perfcounter_save(struct adreno_device *adreno_dev)
{
	struct adreno_perfcounters *counters = adreno_dev->gpudev->perfcounters;
	struct adreno_perfcount_group *group;
	unsigned int regid, groupid;

	for (groupid = 0; groupid < counters->group_count; groupid++) {
		if (!loadable_perfcounter_group(groupid))
			continue;

		group = &(counters->groups[groupid]);

		
		for (regid = 0; regid < group->reg_count; regid++) {
			if (!active_countable(group->regs[regid].countable))
				continue;

			group->regs[regid].value =
				adreno_dev->gpudev->perfcounter_read(
				adreno_dev, groupid, regid);
		}
	}
}

/**
 * a3xx_perfcounter_write() - Write the physical performance counter values.
 * @adreno_dev -  Adreno device whose registers are to be written to.
 * @group - group to which the physical counter belongs to.
 * @counter - register id of the physical counter to which the value is
 *		written to.
 *
 * This function loads the 64 bit saved value into the particular physical
 * counter by enabling the corresponding bit in A3XX_RBBM_PERFCTR_LOAD_CMD*
 * register.
 */
static void a3xx_perfcounter_write(struct adreno_device *adreno_dev,
				unsigned int group, unsigned int counter)
{
	struct kgsl_device *device = &(adreno_dev->dev);
	struct adreno_perfcount_register *reg;
	unsigned int val;

	reg = &(adreno_dev->gpudev->perfcounters->groups[group].regs[counter]);

	
	kgsl_regwrite(device, A3XX_RBBM_PERFCTR_LOAD_CMD0, 0);
	kgsl_regwrite(device, A3XX_RBBM_PERFCTR_LOAD_CMD1, 0);

	
	kgsl_regwrite(device, A3XX_RBBM_PERFCTR_LOAD_VALUE_LO,
			(uint32_t)reg->value);
	kgsl_regwrite(device, A3XX_RBBM_PERFCTR_LOAD_VALUE_HI,
			(uint32_t)(reg->value >> 32));

	if (reg->load_bit < 32)	{
		val = 1 << reg->load_bit;
		kgsl_regwrite(device, A3XX_RBBM_PERFCTR_LOAD_CMD0, val);
	} else {
		val  = 1 << (reg->load_bit - 32);
		kgsl_regwrite(device, A3XX_RBBM_PERFCTR_LOAD_CMD1, val);
	}
}

static void a3xx_perfcounter_restore(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = &adreno_dev->dev;
	struct adreno_perfcounters *counters = adreno_dev->gpudev->perfcounters;
	struct adreno_perfcount_group *group;
	unsigned int regid, groupid;

	for (groupid = 0; groupid < counters->group_count; groupid++) {
		if (!loadable_perfcounter_group(groupid))
			continue;

		group = &(counters->groups[groupid]);

		
		for (regid = 0; regid < group->reg_count; regid++) {
			if (!active_countable(group->regs[regid].countable))
				continue;

			a3xx_perfcounter_write(adreno_dev, groupid, regid);
		}
	}

	
	kgsl_regwrite(device, A3XX_RBBM_PERFCTR_LOAD_CMD0, 0);
	kgsl_regwrite(device, A3XX_RBBM_PERFCTR_LOAD_CMD1, 0);

}

#define A3XX_IRQ_CALLBACK(_c) { .func = _c }

#define A3XX_INT_MASK \
	((1 << A3XX_INT_RBBM_AHB_ERROR) |        \
	 (1 << A3XX_INT_RBBM_ATB_BUS_OVERFLOW) | \
	 (1 << A3XX_INT_CP_T0_PACKET_IN_IB) |    \
	 (1 << A3XX_INT_CP_OPCODE_ERROR) |       \
	 (1 << A3XX_INT_CP_RESERVED_BIT_ERROR) | \
	 (1 << A3XX_INT_CP_HW_FAULT) |           \
	 (1 << A3XX_INT_CP_IB1_INT) |            \
	 (1 << A3XX_INT_CP_IB2_INT) |            \
	 (1 << A3XX_INT_CP_RB_INT) |             \
	 (1 << A3XX_INT_CP_REG_PROTECT_FAULT) |  \
	 (1 << A3XX_INT_CP_AHB_ERROR_HALT) |     \
	 (1 << A3XX_INT_UCHE_OOB_ACCESS))

static struct {
	void (*func)(struct adreno_device *, int);
} a3xx_irq_funcs[] = {
	A3XX_IRQ_CALLBACK(NULL),               
	A3XX_IRQ_CALLBACK(a3xx_err_callback),  
	A3XX_IRQ_CALLBACK(a3xx_err_callback),  
	A3XX_IRQ_CALLBACK(a3xx_err_callback),  
	A3XX_IRQ_CALLBACK(a3xx_err_callback),  
	A3XX_IRQ_CALLBACK(a3xx_err_callback),  
	A3XX_IRQ_CALLBACK(a3xx_err_callback),  
	A3XX_IRQ_CALLBACK(NULL),	       
	A3XX_IRQ_CALLBACK(a3xx_err_callback),  
	A3XX_IRQ_CALLBACK(a3xx_err_callback),  
	A3XX_IRQ_CALLBACK(a3xx_err_callback),  
	A3XX_IRQ_CALLBACK(a3xx_err_callback),  
	A3XX_IRQ_CALLBACK(NULL),	       
	A3XX_IRQ_CALLBACK(a3xx_cp_callback),   
	A3XX_IRQ_CALLBACK(a3xx_cp_callback),   
	A3XX_IRQ_CALLBACK(a3xx_cp_callback),   
	A3XX_IRQ_CALLBACK(a3xx_err_callback),  
	A3XX_IRQ_CALLBACK(NULL),	       
	A3XX_IRQ_CALLBACK(NULL),	       
	A3XX_IRQ_CALLBACK(NULL),	       
	A3XX_IRQ_CALLBACK(NULL),	       
	A3XX_IRQ_CALLBACK(a3xx_err_callback),  
	A3XX_IRQ_CALLBACK(NULL),	       
	A3XX_IRQ_CALLBACK(NULL),	       
	A3XX_IRQ_CALLBACK(a3xx_fatal_err_callback),
	A3XX_IRQ_CALLBACK(a3xx_err_callback),  
	
};

static irqreturn_t a3xx_irq_handler(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = &adreno_dev->dev;
	irqreturn_t ret = IRQ_NONE;
	unsigned int status, tmp;
	int i;

	kgsl_regread(&adreno_dev->dev, A3XX_RBBM_INT_0_STATUS, &status);

	for (tmp = status, i = 0; tmp && i < ARRAY_SIZE(a3xx_irq_funcs); i++) {
		if (tmp & 1) {
			if (a3xx_irq_funcs[i].func != NULL) {
				a3xx_irq_funcs[i].func(adreno_dev, i);
				ret = IRQ_HANDLED;
			} else {
				KGSL_DRV_CRIT(device,
					"Unhandled interrupt bit %x\n", i);
			}
		}

		tmp >>= 1;
	}

	trace_kgsl_a3xx_irq_status(device, status);

	if (status)
		kgsl_regwrite(&adreno_dev->dev, A3XX_RBBM_INT_CLEAR_CMD,
			status);
	return ret;
}

static void a3xx_irq_control(struct adreno_device *adreno_dev, int state)
{
	struct kgsl_device *device = &adreno_dev->dev;

	if (state)
		kgsl_regwrite(device, A3XX_RBBM_INT_0_MASK, A3XX_INT_MASK |
			(test_bit(ADRENO_DEVICE_HANG_INTR, &adreno_dev->priv) ?
				(1 << A3XX_INT_MISC_HANG_DETECT) : 0));
	else
		kgsl_regwrite(device, A3XX_RBBM_INT_0_MASK, 0);
}

static unsigned int a3xx_irq_pending(struct adreno_device *adreno_dev)
{
	unsigned int status;

	kgsl_regread(&adreno_dev->dev, A3XX_RBBM_INT_0_STATUS, &status);

	return (status & A3XX_INT_MASK) ? 1 : 0;
}

static unsigned int counter_delta(struct adreno_device *adreno_dev,
			unsigned int reg, unsigned int *counter)
{
	struct kgsl_device *device = &adreno_dev->dev;
	unsigned int val;
	unsigned int ret = 0;

	
	if (reg == ADRENO_REG_RBBM_PERFCTR_PWR_1_LO)
		adreno_readreg(adreno_dev, reg, &val);
	else
		kgsl_regread(device, reg, &val);

	
	if (*counter != 0) {
		if (val < *counter)
			ret = (0xFFFFFFFF - *counter) + val;
		else
			ret = val - *counter;
	}

	*counter = val;
	return ret;
}

void a3xx_busy_cycles(struct adreno_device *adreno_dev,
				struct adreno_busy_data *data)
{
	struct adreno_busy_data *busy = &adreno_dev->busy_data;
	struct kgsl_device *device = &adreno_dev->dev;

	memset(data, 0, sizeof(*data));

	data->gpu_busy = counter_delta(adreno_dev,
					ADRENO_REG_RBBM_PERFCTR_PWR_1_LO,
					&busy->gpu_busy);
	if (device->pwrctrl.bus_control) {
		data->vbif_ram_cycles = counter_delta(adreno_dev,
					adreno_dev->ram_cycles_lo,
					&busy->vbif_ram_cycles);
		data->vbif_starved_ram = counter_delta(adreno_dev,
					A3XX_VBIF_PERF_PWR_CNT0_LO,
					&busy->vbif_starved_ram);
	}
}

struct a3xx_vbif_data {
	unsigned int reg;
	unsigned int val;
};
static struct a3xx_vbif_data a305_vbif[] = {
	
	{ A3XX_VBIF_IN_RD_LIM_CONF0, 0x10101010 },
	{ A3XX_VBIF_IN_RD_LIM_CONF1, 0x10101010 },
	{ A3XX_VBIF_OUT_RD_LIM_CONF0, 0x10101010 },
	{ A3XX_VBIF_OUT_WR_LIM_CONF0, 0x10101010 },
	{ A3XX_VBIF_DDR_OUT_MAX_BURST, 0x0000303 },
	{ A3XX_VBIF_IN_WR_LIM_CONF0, 0x10101010 },
	{ A3XX_VBIF_IN_WR_LIM_CONF1, 0x10101010 },
	
	{ A3XX_VBIF_GATE_OFF_WRREQ_EN, 0x0000FF },
	
	{ A3XX_VBIF_ARB_CTL, 0x00000030 },
	
	{ A3XX_VBIF_OUT_AXI_AOOO_EN, 0x0000003C },
	{ A3XX_VBIF_OUT_AXI_AOOO, 0x003C003C },
	{0, 0},
};

static struct a3xx_vbif_data a305b_vbif[] = {
	{ A3XX_VBIF_IN_RD_LIM_CONF0, 0x00181818 },
	{ A3XX_VBIF_IN_WR_LIM_CONF0, 0x00181818 },
	{ A3XX_VBIF_OUT_RD_LIM_CONF0, 0x00000018 },
	{ A3XX_VBIF_OUT_WR_LIM_CONF0, 0x00000018 },
	{ A3XX_VBIF_DDR_OUT_MAX_BURST, 0x00000303 },
	{ A3XX_VBIF_ROUND_ROBIN_QOS_ARB, 0x0003 },
	{0, 0},
};

static struct a3xx_vbif_data a305c_vbif[] = {
	{ A3XX_VBIF_IN_RD_LIM_CONF0, 0x00101010 },
	{ A3XX_VBIF_IN_WR_LIM_CONF0, 0x00101010 },
	{ A3XX_VBIF_OUT_RD_LIM_CONF0, 0x00000010 },
	{ A3XX_VBIF_OUT_WR_LIM_CONF0, 0x00000010 },
	{ A3XX_VBIF_DDR_OUT_MAX_BURST, 0x00000101 },
	{ A3XX_VBIF_ARB_CTL, 0x00000010 },
	
	{ A3XX_VBIF_OUT_AXI_AOOO_EN, 0x00000007 },
	{ A3XX_VBIF_OUT_AXI_AOOO, 0x00070007 },
	{0, 0},
};

static struct a3xx_vbif_data a320_vbif[] = {
	
	{ A3XX_VBIF_IN_RD_LIM_CONF0, 0x10101010 },
	{ A3XX_VBIF_IN_RD_LIM_CONF1, 0x10101010 },
	{ A3XX_VBIF_OUT_RD_LIM_CONF0, 0x10101010 },
	{ A3XX_VBIF_OUT_WR_LIM_CONF0, 0x10101010 },
	{ A3XX_VBIF_DDR_OUT_MAX_BURST, 0x0000303 },
	{ A3XX_VBIF_IN_WR_LIM_CONF0, 0x10101010 },
	{ A3XX_VBIF_IN_WR_LIM_CONF1, 0x10101010 },
	
	{ A3XX_VBIF_GATE_OFF_WRREQ_EN, 0x0000FF },
	
	{ A3XX_VBIF_ARB_CTL, 0x00000030 },
	
	{ A3XX_VBIF_OUT_AXI_AOOO_EN, 0x0000003C },
	{ A3XX_VBIF_OUT_AXI_AOOO, 0x003C003C },
	
	{ A3XX_VBIF_ABIT_SORT, 0x000000FF },
	{ A3XX_VBIF_ABIT_SORT_CONF, 0x000000A4 },
	{0, 0},
};

static struct a3xx_vbif_data a330_vbif[] = {
	
	{ A3XX_VBIF_IN_RD_LIM_CONF0, 0x18181818 },
	{ A3XX_VBIF_IN_RD_LIM_CONF1, 0x00001818 },
	{ A3XX_VBIF_OUT_RD_LIM_CONF0, 0x00001818 },
	{ A3XX_VBIF_OUT_WR_LIM_CONF0, 0x00001818 },
	{ A3XX_VBIF_DDR_OUT_MAX_BURST, 0x0000303 },
	{ A3XX_VBIF_IN_WR_LIM_CONF0, 0x18181818 },
	{ A3XX_VBIF_IN_WR_LIM_CONF1, 0x00001818 },
	
	{ A3XX_VBIF_GATE_OFF_WRREQ_EN, 0x00003F },
	
	{ A3XX_VBIF_ARB_CTL, 0x00000030 },
	
	{ A3XX_VBIF_ROUND_ROBIN_QOS_ARB, 0x0001 },
	
	{ A3XX_VBIF_OUT_AXI_AOOO_EN, 0x0000003F },
	{ A3XX_VBIF_OUT_AXI_AOOO, 0x003F003F },
	
	{ A3XX_VBIF_ABIT_SORT, 0x0001003F },
	{ A3XX_VBIF_ABIT_SORT_CONF, 0x000000A4 },
	{ A3XX_VBIF_CLKON, 1 },
	{0, 0},
};

static struct a3xx_vbif_data a330v2_vbif[] = {
	
	{ A3XX_VBIF_ABIT_SORT, 0x0001003F },
	{ A3XX_VBIF_ABIT_SORT_CONF, 0x000000A4 },
	
	{ A3XX_VBIF_GATE_OFF_WRREQ_EN, 0x00003F },
	{ A3XX_VBIF_DDR_OUT_MAX_BURST, 0x0000303 },
	
	{ A3XX_VBIF_ROUND_ROBIN_QOS_ARB, 0x0003 },
	{0, 0},
};

static struct {
	int(*devfunc)(struct adreno_device *);
	struct a3xx_vbif_data *vbif;
} a3xx_vbif_platforms[] = {
	{ adreno_is_a305, a305_vbif },
	{ adreno_is_a305c, a305c_vbif },
	{ adreno_is_a320, a320_vbif },
	
	{ adreno_is_a330v2, a330v2_vbif },
	{ adreno_is_a330, a330_vbif },
	{ adreno_is_a305b, a305b_vbif },
};


static struct adreno_perfcount_register a3xx_perfcounters_cp[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_CP_0_LO,
		A3XX_RBBM_PERFCTR_CP_0_HI, 0, A3XX_CP_PERFCOUNTER_SELECT },
};

static struct adreno_perfcount_register a3xx_perfcounters_rbbm[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_RBBM_0_LO,
		A3XX_RBBM_PERFCTR_RBBM_0_HI, 1, A3XX_RBBM_PERFCOUNTER0_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_RBBM_1_LO,
		A3XX_RBBM_PERFCTR_RBBM_1_HI, 2, A3XX_RBBM_PERFCOUNTER1_SELECT },
};

static struct adreno_perfcount_register a3xx_perfcounters_pc[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_PC_0_LO,
		A3XX_RBBM_PERFCTR_PC_0_HI, 3, A3XX_PC_PERFCOUNTER0_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_PC_1_LO,
		A3XX_RBBM_PERFCTR_PC_1_HI, 4, A3XX_PC_PERFCOUNTER1_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_PC_2_LO,
		A3XX_RBBM_PERFCTR_PC_2_HI, 5, A3XX_PC_PERFCOUNTER2_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_PC_3_LO,
		A3XX_RBBM_PERFCTR_PC_3_HI, 6, A3XX_PC_PERFCOUNTER3_SELECT },
};

static struct adreno_perfcount_register a3xx_perfcounters_vfd[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_VFD_0_LO,
		A3XX_RBBM_PERFCTR_VFD_0_HI, 7, A3XX_VFD_PERFCOUNTER0_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_VFD_1_LO,
		A3XX_RBBM_PERFCTR_VFD_1_HI, 8, A3XX_VFD_PERFCOUNTER1_SELECT },
};

static struct adreno_perfcount_register a3xx_perfcounters_hlsq[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_HLSQ_0_LO,
		A3XX_RBBM_PERFCTR_HLSQ_0_HI, 9,
		A3XX_HLSQ_PERFCOUNTER0_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_HLSQ_1_LO,
		A3XX_RBBM_PERFCTR_HLSQ_1_HI, 10,
		A3XX_HLSQ_PERFCOUNTER1_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_HLSQ_2_LO,
		A3XX_RBBM_PERFCTR_HLSQ_2_HI, 11,
		A3XX_HLSQ_PERFCOUNTER2_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_HLSQ_3_LO,
		A3XX_RBBM_PERFCTR_HLSQ_3_HI, 12,
		A3XX_HLSQ_PERFCOUNTER3_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_HLSQ_4_LO,
		A3XX_RBBM_PERFCTR_HLSQ_4_HI, 13,
		A3XX_HLSQ_PERFCOUNTER4_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_HLSQ_5_LO,
		A3XX_RBBM_PERFCTR_HLSQ_5_HI, 14,
		A3XX_HLSQ_PERFCOUNTER5_SELECT },
};

static struct adreno_perfcount_register a3xx_perfcounters_vpc[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_VPC_0_LO,
		A3XX_RBBM_PERFCTR_VPC_0_HI, 15, A3XX_VPC_PERFCOUNTER0_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_VPC_1_LO,
		A3XX_RBBM_PERFCTR_VPC_1_HI, 16, A3XX_VPC_PERFCOUNTER1_SELECT },
};

static struct adreno_perfcount_register a3xx_perfcounters_tse[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_TSE_0_LO,
		A3XX_RBBM_PERFCTR_TSE_0_HI, 17, A3XX_GRAS_PERFCOUNTER0_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_TSE_1_LO,
		A3XX_RBBM_PERFCTR_TSE_1_HI, 18, A3XX_GRAS_PERFCOUNTER1_SELECT },
};

static struct adreno_perfcount_register a3xx_perfcounters_ras[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_RAS_0_LO,
		A3XX_RBBM_PERFCTR_RAS_0_HI, 19, A3XX_GRAS_PERFCOUNTER2_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_RAS_1_LO,
		A3XX_RBBM_PERFCTR_RAS_1_HI, 20, A3XX_GRAS_PERFCOUNTER3_SELECT },
};

static struct adreno_perfcount_register a3xx_perfcounters_uche[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_UCHE_0_LO,
		A3XX_RBBM_PERFCTR_UCHE_0_HI, 21,
		A3XX_UCHE_PERFCOUNTER0_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_UCHE_1_LO,
		A3XX_RBBM_PERFCTR_UCHE_1_HI, 22,
		A3XX_UCHE_PERFCOUNTER1_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_UCHE_2_LO,
		A3XX_RBBM_PERFCTR_UCHE_2_HI, 23,
		A3XX_UCHE_PERFCOUNTER2_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_UCHE_3_LO,
		A3XX_RBBM_PERFCTR_UCHE_3_HI, 24,
		A3XX_UCHE_PERFCOUNTER3_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_UCHE_4_LO,
		A3XX_RBBM_PERFCTR_UCHE_4_HI, 25,
		A3XX_UCHE_PERFCOUNTER4_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_UCHE_5_LO,
		A3XX_RBBM_PERFCTR_UCHE_5_HI, 26,
		A3XX_UCHE_PERFCOUNTER5_SELECT },
};

static struct adreno_perfcount_register a3xx_perfcounters_tp[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_TP_0_LO,
		A3XX_RBBM_PERFCTR_TP_0_HI, 27, A3XX_TP_PERFCOUNTER0_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_TP_1_LO,
		A3XX_RBBM_PERFCTR_TP_1_HI, 28, A3XX_TP_PERFCOUNTER1_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_TP_2_LO,
		A3XX_RBBM_PERFCTR_TP_2_HI, 29, A3XX_TP_PERFCOUNTER2_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_TP_3_LO,
		A3XX_RBBM_PERFCTR_TP_3_HI, 30, A3XX_TP_PERFCOUNTER3_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_TP_4_LO,
		A3XX_RBBM_PERFCTR_TP_4_HI, 31, A3XX_TP_PERFCOUNTER4_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_TP_5_LO,
		A3XX_RBBM_PERFCTR_TP_5_HI, 32, A3XX_TP_PERFCOUNTER5_SELECT },
};

static struct adreno_perfcount_register a3xx_perfcounters_sp[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_SP_0_LO,
		A3XX_RBBM_PERFCTR_SP_0_HI, 33, A3XX_SP_PERFCOUNTER0_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_SP_1_LO,
		A3XX_RBBM_PERFCTR_SP_1_HI, 34, A3XX_SP_PERFCOUNTER1_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_SP_2_LO,
		A3XX_RBBM_PERFCTR_SP_2_HI, 35, A3XX_SP_PERFCOUNTER2_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_SP_3_LO,
		A3XX_RBBM_PERFCTR_SP_3_HI, 36, A3XX_SP_PERFCOUNTER3_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_SP_4_LO,
		A3XX_RBBM_PERFCTR_SP_4_HI, 37, A3XX_SP_PERFCOUNTER4_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_SP_5_LO,
		A3XX_RBBM_PERFCTR_SP_5_HI, 38, A3XX_SP_PERFCOUNTER5_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_SP_6_LO,
		A3XX_RBBM_PERFCTR_SP_6_HI, 39, A3XX_SP_PERFCOUNTER6_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_SP_7_LO,
		A3XX_RBBM_PERFCTR_SP_7_HI, 40, A3XX_SP_PERFCOUNTER7_SELECT },
};

static struct adreno_perfcount_register a3xx_perfcounters_rb[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_RB_0_LO,
		A3XX_RBBM_PERFCTR_RB_0_HI, 41, A3XX_RB_PERFCOUNTER0_SELECT },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_RB_1_LO,
		A3XX_RBBM_PERFCTR_RB_1_HI, 42, A3XX_RB_PERFCOUNTER1_SELECT },
};

static struct adreno_perfcount_register a3xx_perfcounters_pwr[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_PWR_0_LO,
		A3XX_RBBM_PERFCTR_PWR_0_HI, -1, 0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_RBBM_PERFCTR_PWR_1_LO,
		A3XX_RBBM_PERFCTR_PWR_1_HI, -1, 0 },
};

static struct adreno_perfcount_register a3xx_perfcounters_vbif[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_VBIF_PERF_CNT0_LO,
		A3XX_VBIF_PERF_CNT0_HI, -1, 0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_VBIF_PERF_CNT1_LO,
		A3XX_VBIF_PERF_CNT1_HI, -1, 0 },
};
static struct adreno_perfcount_register a3xx_perfcounters_vbif_pwr[] = {
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_VBIF_PERF_PWR_CNT0_LO,
		A3XX_VBIF_PERF_PWR_CNT0_HI, -1, 0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_VBIF_PERF_PWR_CNT1_LO,
		A3XX_VBIF_PERF_PWR_CNT1_HI, -1, 0 },
	{ KGSL_PERFCOUNTER_NOT_USED, 0, 0, A3XX_VBIF_PERF_PWR_CNT2_LO,
		A3XX_VBIF_PERF_PWR_CNT2_HI, -1, 0 },
};

static struct adreno_perfcount_group a3xx_perfcounter_groups[] = {
	ADRENO_PERFCOUNTER_GROUP(a3xx, cp),
	ADRENO_PERFCOUNTER_GROUP(a3xx, rbbm),
	ADRENO_PERFCOUNTER_GROUP(a3xx, pc),
	ADRENO_PERFCOUNTER_GROUP(a3xx, vfd),
	ADRENO_PERFCOUNTER_GROUP(a3xx, hlsq),
	ADRENO_PERFCOUNTER_GROUP(a3xx, vpc),
	ADRENO_PERFCOUNTER_GROUP(a3xx, tse),
	ADRENO_PERFCOUNTER_GROUP(a3xx, ras),
	ADRENO_PERFCOUNTER_GROUP(a3xx, uche),
	ADRENO_PERFCOUNTER_GROUP(a3xx, tp),
	ADRENO_PERFCOUNTER_GROUP(a3xx, sp),
	ADRENO_PERFCOUNTER_GROUP(a3xx, rb),
	ADRENO_PERFCOUNTER_GROUP_FLAGS(a3xx, pwr,
		ADRENO_PERFCOUNTER_GROUP_FIXED),
	ADRENO_PERFCOUNTER_GROUP(a3xx, vbif),
	ADRENO_PERFCOUNTER_GROUP_FLAGS(a3xx, vbif_pwr,
		ADRENO_PERFCOUNTER_GROUP_FIXED),
};

static struct adreno_perfcounters a3xx_perfcounters = {
	a3xx_perfcounter_groups,
	ARRAY_SIZE(a3xx_perfcounter_groups),
};

static inline int _get_counter(struct adreno_device *adreno_dev,
		int group, int countable, unsigned int *lo,
		unsigned int *hi)
{
	int ret = 0;

	if (*lo == 0) {

		ret = adreno_perfcounter_get(adreno_dev, group, countable,
			lo, hi, PERFCOUNTER_FLAG_KERNEL);

		if (ret) {
			struct kgsl_device *device = &adreno_dev->dev;

			KGSL_DRV_ERR(device,
				"Unable to allocate fault detect performance counter %d/%d\n",
				group, countable);
			KGSL_DRV_ERR(device,
				"GPU fault detect will be less reliable\n");
		}
	}

	return ret;
}

static inline void _put_counter(struct adreno_device *adreno_dev,
		int group, int countable, unsigned int *lo,
		unsigned int *hi)
{
	if (*lo != 0) {
		adreno_perfcounter_put(adreno_dev, group, countable,
			PERFCOUNTER_FLAG_KERNEL);
	}

	*lo = 0;
	*hi = 0;
}

void a3xx_fault_detect_start(struct adreno_device *adreno_dev)
{
	_get_counter(adreno_dev, KGSL_PERFCOUNTER_GROUP_SP,
		SP_ALU_ACTIVE_CYCLES,
		&ft_detect_regs[6], &ft_detect_regs[7]);

	_get_counter(adreno_dev, KGSL_PERFCOUNTER_GROUP_SP,
		SP0_ICL1_MISSES,
		&ft_detect_regs[8], &ft_detect_regs[9]);

	_get_counter(adreno_dev, KGSL_PERFCOUNTER_GROUP_SP,
		SP_FS_CFLOW_INSTRUCTIONS,
		&ft_detect_regs[10], &ft_detect_regs[11]);

	_get_counter(adreno_dev, KGSL_PERFCOUNTER_GROUP_TSE,
		TSE_INPUT_PRIM_NUM,
		&ft_detect_regs[12], &ft_detect_regs[13]);
}
void a3xx_fault_detect_stop(struct adreno_device *adreno_dev)
{
	_put_counter(adreno_dev, KGSL_PERFCOUNTER_GROUP_SP,
		SP_ALU_ACTIVE_CYCLES,
		&ft_detect_regs[6], &ft_detect_regs[7]);

	_put_counter(adreno_dev, KGSL_PERFCOUNTER_GROUP_SP,
		SP0_ICL1_MISSES,
		&ft_detect_regs[8], &ft_detect_regs[9]);

	_put_counter(adreno_dev, KGSL_PERFCOUNTER_GROUP_SP,
		SP_FS_CFLOW_INSTRUCTIONS,
		&ft_detect_regs[10], &ft_detect_regs[11]);

	_put_counter(adreno_dev, KGSL_PERFCOUNTER_GROUP_TSE,
		TSE_INPUT_PRIM_NUM,
		&ft_detect_regs[12], &ft_detect_regs[13]);
}

static void a3xx_perfcounter_close(struct adreno_device *adreno_dev)
{
	adreno_perfcounter_put(adreno_dev, KGSL_PERFCOUNTER_GROUP_PWR, 1,
		PERFCOUNTER_FLAG_KERNEL);

	adreno_perfcounter_put(adreno_dev, KGSL_PERFCOUNTER_GROUP_VBIF_PWR, 0,
		PERFCOUNTER_FLAG_KERNEL);

	adreno_perfcounter_put(adreno_dev, KGSL_PERFCOUNTER_GROUP_VBIF,
		VBIF_AXI_TOTAL_BEATS, PERFCOUNTER_FLAG_KERNEL);

	if (adreno_dev->fast_hang_detect)
		a3xx_fault_detect_stop(adreno_dev);
}

static int a3xx_perfcounter_init(struct adreno_device *adreno_dev)
{
	int ret;
	struct kgsl_device *device = &adreno_dev->dev;
	
	if (adreno_is_a330(adreno_dev))
		a3xx_perfcounters_sp[3].countable = KGSL_PERFCOUNTER_BROKEN;

	if (adreno_dev->fast_hang_detect)
		a3xx_fault_detect_start(adreno_dev);

	
	ret = adreno_perfcounter_get(adreno_dev, KGSL_PERFCOUNTER_GROUP_PWR, 1,
			NULL, NULL, PERFCOUNTER_FLAG_KERNEL);

	if (device->pwrctrl.bus_control) {
		
		ret |= adreno_perfcounter_get(adreno_dev,
				KGSL_PERFCOUNTER_GROUP_VBIF_PWR, 0,
				NULL, NULL, PERFCOUNTER_FLAG_KERNEL);
		
		ret |= adreno_perfcounter_get(adreno_dev,
				KGSL_PERFCOUNTER_GROUP_VBIF,
				VBIF_AXI_TOTAL_BEATS,
				&adreno_dev->ram_cycles_lo, NULL,
				PERFCOUNTER_FLAG_KERNEL);
	}

	
	adreno_dev->profile.enabled = false;
	return ret;
}

static void a3xx_protect_init(struct kgsl_device *device)
{
	int index = 0;

	
	kgsl_regwrite(device, A3XX_CP_PROTECT_CTRL, 0x00000007);

	
	adreno_set_protected_registers(device, &index, 0x18, 0);
	adreno_set_protected_registers(device, &index, 0x20, 2);
	adreno_set_protected_registers(device, &index, 0x33, 0);
	adreno_set_protected_registers(device, &index, 0x42, 0);
	adreno_set_protected_registers(device, &index, 0x50, 4);
	adreno_set_protected_registers(device, &index, 0x63, 0);
	adreno_set_protected_registers(device, &index, 0x100, 4);

	
	adreno_set_protected_registers(device, &index, 0x1C0, 5);
	adreno_set_protected_registers(device, &index, 0x1EC, 1);
	adreno_set_protected_registers(device, &index, 0x1F6, 1);
	adreno_set_protected_registers(device, &index, 0x1F8, 2);
	adreno_set_protected_registers(device, &index, 0x45E, 2);
	adreno_set_protected_registers(device, &index, 0x460, 4);

	
	adreno_set_protected_registers(device, &index, 0xCC0, 0);

	
	adreno_set_protected_registers(device, &index, 0x3000, 6);

	
	adreno_set_protected_registers(device, &index, 0x4000, 14);
}

static void a3xx_start(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = &adreno_dev->dev;
	struct a3xx_vbif_data *vbif = NULL;
	int i;

	for (i = 0; i < ARRAY_SIZE(a3xx_vbif_platforms); i++) {
		if (a3xx_vbif_platforms[i].devfunc(adreno_dev)) {
			vbif = a3xx_vbif_platforms[i].vbif;
			break;
		}
	}

	BUG_ON(vbif == NULL);

	while (vbif->reg != 0) {
		kgsl_regwrite(device, vbif->reg, vbif->val);
		vbif++;
	}

	
	kgsl_regwrite(device, A3XX_RBBM_GPU_BUSY_MASKED, 0xFFFFFFFF);

	
	kgsl_regwrite(device, A3XX_RBBM_SP_HYST_CNT, 0x10);
	kgsl_regwrite(device, A3XX_RBBM_WAIT_IDLE_CLOCKS_CTL, 0x10);


	kgsl_regwrite(device, A3XX_RBBM_AHB_CTL0, 0x00000001);

	
	kgsl_regwrite(device, A3XX_RBBM_AHB_CTL1, 0xA6FFFFFF);

	
	kgsl_regwrite(device, A3XX_RBBM_RBBM_CTL, 0x00030000);

	if (adreno_is_a330v2(adreno_dev))
		kgsl_regwrite(device, A3XX_RBBM_INTERFACE_HANG_INT_CTL,
				(1 << 31) | 0xFFFF);
	else
		kgsl_regwrite(device, A3XX_RBBM_INTERFACE_HANG_INT_CTL,
				(1 << 16) | 0xFFF);

	
	kgsl_regwrite(device, A3XX_UCHE_CACHE_MODE_CONTROL_REG, 0x00000001);

	
	kgsl_regwrite(device, A3XX_RBBM_CLOCK_CTL,
		adreno_a3xx_rbbm_clock_ctl_default(adreno_dev));

	if (adreno_is_a330v2(adreno_dev))
		kgsl_regwrite(device, A3XX_RBBM_GPR0_CTL,
			A330v2_RBBM_GPR0_CTL_DEFAULT);
	else if (adreno_is_a330(adreno_dev))
		kgsl_regwrite(device, A3XX_RBBM_GPR0_CTL,
			A330_RBBM_GPR0_CTL_DEFAULT);

	
	if (adreno_is_a330(adreno_dev) ||
		adreno_is_a305b(adreno_dev)) {
		kgsl_regwrite(device, A3XX_RB_GMEM_BASE_ADDR,
			(unsigned int)(adreno_dev->ocmem_base >> 14));
	}
	
	a3xx_protect_init(device);

	
	kgsl_regwrite(device, A3XX_RBBM_PERFCTR_CTL, 0x01);

	
	kgsl_regwrite(device, REG_CP_DEBUG, A2XX_CP_DEBUG_DEFAULT);
	memset(&adreno_dev->busy_data, 0, sizeof(adreno_dev->busy_data));
}

int a3xx_coresight_enable(struct kgsl_device *device)
{
	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
	if (!kgsl_active_count_get(device)) {
		kgsl_regwrite(device, A3XX_RBBM_DEBUG_BUS_CTL, 0x0001093F);
		kgsl_regwrite(device, A3XX_RBBM_DEBUG_BUS_STB_CTL0,
				0x00000000);
		kgsl_regwrite(device, A3XX_RBBM_DEBUG_BUS_STB_CTL1,
				0xFFFFFFFE);
		kgsl_regwrite(device, A3XX_RBBM_INT_TRACE_BUS_CTL,
				0x00201111);
		kgsl_regwrite(device, A3XX_RBBM_EXT_TRACE_BUS_CTL,
				0x89100010);
		kgsl_regwrite(device, A3XX_RBBM_EXT_TRACE_STOP_CNT,
				0x00017fff);
		kgsl_regwrite(device, A3XX_RBBM_EXT_TRACE_START_CNT,
				0x0001000f);
		kgsl_regwrite(device, A3XX_RBBM_EXT_TRACE_PERIOD_CNT ,
				0x0001ffff);
		kgsl_regwrite(device, A3XX_RBBM_EXT_TRACE_CMD,
				0x00000001);
		kgsl_active_count_put(device);
	}
	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
	return 0;
}

void a3xx_coresight_disable(struct kgsl_device *device)
{
	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
	if (!kgsl_active_count_get(device)) {
		kgsl_regwrite(device, A3XX_RBBM_DEBUG_BUS_CTL, 0x0);
		kgsl_regwrite(device, A3XX_RBBM_DEBUG_BUS_STB_CTL0, 0x0);
		kgsl_regwrite(device, A3XX_RBBM_DEBUG_BUS_STB_CTL1, 0x0);
		kgsl_regwrite(device, A3XX_RBBM_INT_TRACE_BUS_CTL, 0x0);
		kgsl_regwrite(device, A3XX_RBBM_EXT_TRACE_BUS_CTL, 0x0);
		kgsl_regwrite(device, A3XX_RBBM_EXT_TRACE_STOP_CNT, 0x0);
		kgsl_regwrite(device, A3XX_RBBM_EXT_TRACE_START_CNT, 0x0);
		kgsl_regwrite(device, A3XX_RBBM_EXT_TRACE_PERIOD_CNT , 0x0);
		kgsl_regwrite(device, A3XX_RBBM_EXT_TRACE_CMD, 0x0);
		kgsl_active_count_put(device);
	}
	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
}

static void a3xx_coresight_write_reg(struct kgsl_device *device,
		unsigned int wordoffset, unsigned int val)
{
	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
	if (!kgsl_active_count_get(device)) {
		kgsl_regwrite(device, wordoffset, val);
		kgsl_active_count_put(device);
	}
	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
}

void a3xx_coresight_config_debug_reg(struct kgsl_device *device,
		int debug_reg, unsigned int val)
{
	switch (debug_reg) {

	case DEBUG_BUS_CTL:
		a3xx_coresight_write_reg(device, A3XX_RBBM_DEBUG_BUS_CTL, val);
		break;

	case TRACE_STOP_CNT:
		a3xx_coresight_write_reg(device, A3XX_RBBM_EXT_TRACE_STOP_CNT,
				val);
		break;

	case TRACE_START_CNT:
		a3xx_coresight_write_reg(device, A3XX_RBBM_EXT_TRACE_START_CNT,
				val);
		break;

	case TRACE_PERIOD_CNT:
		a3xx_coresight_write_reg(device, A3XX_RBBM_EXT_TRACE_PERIOD_CNT,
				val);
		break;

	case TRACE_CMD:
		a3xx_coresight_write_reg(device, A3XX_RBBM_EXT_TRACE_CMD, val);
		break;

	case TRACE_BUS_CTL:
		a3xx_coresight_write_reg(device, A3XX_RBBM_EXT_TRACE_BUS_CTL,
				val);
		break;
	}

}

static void a3xx_soft_reset(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = &adreno_dev->dev;
	unsigned int reg;

	kgsl_regwrite(device, A3XX_RBBM_SW_RESET_CMD, 1);
	kgsl_regread(device, A3XX_RBBM_SW_RESET_CMD, &reg);
	kgsl_regwrite(device, A3XX_RBBM_SW_RESET_CMD, 0);
}

void *a3xx_snapshot(struct adreno_device *adreno_dev, void *snapshot,
	int *remain, int hang);

static void a3xx_postmortem_dump(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = &adreno_dev->dev;
	unsigned int r1, r2, r3, rbbm_status;
	unsigned int cp_stat, rb_count;

	kgsl_regread(device, REG_RBBM_STATUS, &rbbm_status);
	KGSL_LOG_DUMP(device, "RBBM:   STATUS   = %08X\n", rbbm_status);

	{
		struct log_field lines[] = {
			{rbbm_status & BIT(0),  "HI busy     "},
			{rbbm_status & BIT(1),  "CP ME busy  "},
			{rbbm_status & BIT(2),  "CP PFP busy "},
			{rbbm_status & BIT(14), "CP NRT busy "},
			{rbbm_status & BIT(15), "VBIF busy   "},
			{rbbm_status & BIT(16), "TSE busy    "},
			{rbbm_status & BIT(17), "RAS busy    "},
			{rbbm_status & BIT(18), "RB busy     "},
			{rbbm_status & BIT(19), "PC DCALL bsy"},
			{rbbm_status & BIT(20), "PC VSD busy "},
			{rbbm_status & BIT(21), "VFD busy    "},
			{rbbm_status & BIT(22), "VPC busy    "},
			{rbbm_status & BIT(23), "UCHE busy   "},
			{rbbm_status & BIT(24), "SP busy     "},
			{rbbm_status & BIT(25), "TPL1 busy   "},
			{rbbm_status & BIT(26), "MARB busy   "},
			{rbbm_status & BIT(27), "VSC busy    "},
			{rbbm_status & BIT(28), "ARB busy    "},
			{rbbm_status & BIT(29), "HLSQ busy   "},
			{rbbm_status & BIT(30), "GPU bsy noHC"},
			{rbbm_status & BIT(31), "GPU busy    "},
			};
		adreno_dump_fields(device, " STATUS=", lines,
				ARRAY_SIZE(lines));
	}

	kgsl_regread(device, REG_CP_RB_BASE, &r1);
	kgsl_regread(device, REG_CP_RB_CNTL, &r2);
	rb_count = 2 << (r2 & (BIT(6) - 1));
	kgsl_regread(device, REG_CP_RB_RPTR_ADDR, &r3);
	KGSL_LOG_DUMP(device,
		"CP_RB:  BASE = %08X | CNTL   = %08X | RPTR_ADDR = %08X"
		"| rb_count = %08X\n", r1, r2, r3, rb_count);

	kgsl_regread(device, REG_CP_RB_RPTR, &r1);
	kgsl_regread(device, REG_CP_RB_WPTR, &r2);
	kgsl_regread(device, REG_CP_RB_RPTR_WR, &r3);
	KGSL_LOG_DUMP(device,
		"CP_RB:  BASE = %08X | CNTL   = %08X | RPTR_ADDR = %08X"
		"| rb_count = %08X\n", r1, r2, r3, rb_count);

	kgsl_regread(device, REG_CP_RB_RPTR, &r1);
	kgsl_regread(device, REG_CP_RB_WPTR, &r2);
	kgsl_regread(device, REG_CP_RB_RPTR_WR, &r3);
	KGSL_LOG_DUMP(device,
		"	RPTR = %08X | WPTR   = %08X | RPTR_WR   = %08X"
		"\n", r1, r2, r3);

	kgsl_regread(device, REG_CP_IB1_BASE, &r1);
	kgsl_regread(device, REG_CP_IB1_BUFSZ, &r2);
	KGSL_LOG_DUMP(device, "CP_IB1: BASE = %08X | BUFSZ  = %d\n", r1, r2);

	kgsl_regread(device, REG_CP_ME_CNTL, &r1);
	kgsl_regread(device, REG_CP_ME_STATUS, &r2);
	KGSL_LOG_DUMP(device, "CP_ME:  CNTL = %08X | STATUS = %08X\n", r1, r2);

	kgsl_regread(device, REG_CP_STAT, &cp_stat);
	KGSL_LOG_DUMP(device, "CP_STAT      = %08X\n", cp_stat);
#ifndef CONFIG_MSM_KGSL_PSTMRTMDMP_CP_STAT_NO_DETAIL
	{
		struct log_field lns[] = {
			{cp_stat & BIT(0), "WR_BSY     0"},
			{cp_stat & BIT(1), "RD_RQ_BSY  1"},
			{cp_stat & BIT(2), "RD_RTN_BSY 2"},
		};
		adreno_dump_fields(device, "    MIU=", lns, ARRAY_SIZE(lns));
	}
	{
		struct log_field lns[] = {
			{cp_stat & BIT(5), "RING_BUSY  5"},
			{cp_stat & BIT(6), "NDRCTS_BSY 6"},
			{cp_stat & BIT(7), "NDRCT2_BSY 7"},
			{cp_stat & BIT(9), "ST_BUSY    9"},
			{cp_stat & BIT(10), "BUSY      10"},
		};
		adreno_dump_fields(device, "    CSF=", lns, ARRAY_SIZE(lns));
	}
	{
		struct log_field lns[] = {
			{cp_stat & BIT(11), "RNG_Q_BSY 11"},
			{cp_stat & BIT(12), "NDRCTS_Q_B12"},
			{cp_stat & BIT(13), "NDRCT2_Q_B13"},
			{cp_stat & BIT(16), "ST_QUEUE_B16"},
			{cp_stat & BIT(17), "PFP_BUSY  17"},
		};
		adreno_dump_fields(device, "   RING=", lns, ARRAY_SIZE(lns));
	}
	{
		struct log_field lns[] = {
			{cp_stat & BIT(3), "RBIU_BUSY  3"},
			{cp_stat & BIT(4), "RCIU_BUSY  4"},
			{cp_stat & BIT(8), "EVENT_BUSY 8"},
			{cp_stat & BIT(18), "MQ_RG_BSY 18"},
			{cp_stat & BIT(19), "MQ_NDRS_BS19"},
			{cp_stat & BIT(20), "MQ_NDR2_BS20"},
			{cp_stat & BIT(21), "MIU_WC_STL21"},
			{cp_stat & BIT(22), "CP_NRT_BSY22"},
			{cp_stat & BIT(23), "3D_BUSY   23"},
			{cp_stat & BIT(26), "ME_BUSY   26"},
			{cp_stat & BIT(27), "RB_FFO_BSY27"},
			{cp_stat & BIT(28), "CF_FFO_BSY28"},
			{cp_stat & BIT(29), "PS_FFO_BSY29"},
			{cp_stat & BIT(30), "VS_FFO_BSY30"},
			{cp_stat & BIT(31), "CP_BUSY   31"},
		};
		adreno_dump_fields(device, " CP_STT=", lns, ARRAY_SIZE(lns));
	}
#endif

	kgsl_regread(device, A3XX_RBBM_INT_0_STATUS, &r1);
	KGSL_LOG_DUMP(device, "MSTR_INT_SGNL = %08X\n", r1);
	{
		struct log_field ints[] = {
			{r1 & BIT(0),  "RBBM_GPU_IDLE 0"},
			{r1 & BIT(1),  "RBBM_AHB_ERROR 1"},
			{r1 & BIT(2),  "RBBM_REG_TIMEOUT 2"},
			{r1 & BIT(3),  "RBBM_ME_MS_TIMEOUT 3"},
			{r1 & BIT(4),  "RBBM_PFP_MS_TIMEOUT 4"},
			{r1 & BIT(5),  "RBBM_ATB_BUS_OVERFLOW 5"},
			{r1 & BIT(6),  "VFD_ERROR 6"},
			{r1 & BIT(7),  "CP_SW_INT 7"},
			{r1 & BIT(8),  "CP_T0_PACKET_IN_IB 8"},
			{r1 & BIT(9),  "CP_OPCODE_ERROR 9"},
			{r1 & BIT(10), "CP_RESERVED_BIT_ERROR 10"},
			{r1 & BIT(11), "CP_HW_FAULT 11"},
			{r1 & BIT(12), "CP_DMA 12"},
			{r1 & BIT(13), "CP_IB2_INT 13"},
			{r1 & BIT(14), "CP_IB1_INT 14"},
			{r1 & BIT(15), "CP_RB_INT 15"},
			{r1 & BIT(16), "CP_REG_PROTECT_FAULT 16"},
			{r1 & BIT(17), "CP_RB_DONE_TS 17"},
			{r1 & BIT(18), "CP_VS_DONE_TS 18"},
			{r1 & BIT(19), "CP_PS_DONE_TS 19"},
			{r1 & BIT(20), "CACHE_FLUSH_TS 20"},
			{r1 & BIT(21), "CP_AHB_ERROR_HALT 21"},
			{r1 & BIT(24), "MISC_HANG_DETECT 24"},
			{r1 & BIT(25), "UCHE_OOB_ACCESS 25"},
		};
		adreno_dump_fields(device, "INT_SGNL=", ints, ARRAY_SIZE(ints));
	}
}

static unsigned int a3xx_register_offsets[ADRENO_REG_REGISTER_MAX] = {
	ADRENO_REG_DEFINE(ADRENO_REG_CP_DEBUG, REG_CP_DEBUG),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_ME_RAM_WADDR, REG_CP_ME_RAM_WADDR),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_ME_RAM_DATA, REG_CP_ME_RAM_DATA),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_PFP_UCODE_DATA, A3XX_CP_PFP_UCODE_DATA),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_PFP_UCODE_ADDR, A3XX_CP_PFP_UCODE_ADDR),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_WFI_PEND_CTR, A3XX_CP_WFI_PEND_CTR),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_RB_BASE, REG_CP_RB_BASE),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_RB_RPTR_ADDR, REG_CP_RB_RPTR_ADDR),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_RB_RPTR, REG_CP_RB_RPTR),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_RB_WPTR, REG_CP_RB_WPTR),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_PROTECT_CTRL, A3XX_CP_PROTECT_CTRL),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_ME_CNTL, REG_CP_ME_CNTL),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_RB_CNTL, REG_CP_RB_CNTL),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_IB1_BASE, REG_CP_IB1_BASE),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_IB1_BUFSZ, REG_CP_IB1_BUFSZ),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_IB2_BASE, REG_CP_IB2_BASE),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_IB2_BUFSZ, REG_CP_IB2_BUFSZ),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_TIMESTAMP, REG_CP_TIMESTAMP),
	ADRENO_REG_DEFINE(ADRENO_REG_SCRATCH_ADDR, REG_SCRATCH_ADDR),
	ADRENO_REG_DEFINE(ADRENO_REG_SCRATCH_UMSK, REG_SCRATCH_UMSK),
	ADRENO_REG_DEFINE(ADRENO_REG_CP_HW_FAULT, A3XX_CP_HW_FAULT),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_STATUS, A3XX_RBBM_STATUS),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_PERFCTR_CTL, A3XX_RBBM_PERFCTR_CTL),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_PERFCTR_LOAD_CMD0,
					A3XX_RBBM_PERFCTR_LOAD_CMD0),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_PERFCTR_LOAD_CMD1,
					A3XX_RBBM_PERFCTR_LOAD_CMD1),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_PERFCTR_PWR_1_LO,
					A3XX_RBBM_PERFCTR_PWR_1_LO),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_INT_0_MASK, A3XX_RBBM_INT_0_MASK),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_INT_0_STATUS, A3XX_RBBM_INT_0_STATUS),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_AHB_ERROR_STATUS,
					A3XX_RBBM_AHB_ERROR_STATUS),
	ADRENO_REG_DEFINE(ADRENO_REG_VPC_VPC_DEBUG_RAM_SEL,
					A3XX_VPC_VPC_DEBUG_RAM_SEL),
	ADRENO_REG_DEFINE(ADRENO_REG_VPC_VPC_DEBUG_RAM_READ,
					A3XX_VPC_VPC_DEBUG_RAM_READ),
	ADRENO_REG_DEFINE(ADRENO_REG_VSC_PIPE_DATA_ADDRESS_0,
				A3XX_VSC_PIPE_DATA_ADDRESS_0),
	ADRENO_REG_DEFINE(ADRENO_REG_VSC_PIPE_DATA_LENGTH_7,
					A3XX_VSC_PIPE_DATA_LENGTH_7),
	ADRENO_REG_DEFINE(ADRENO_REG_VSC_SIZE_ADDRESS, A3XX_VSC_SIZE_ADDRESS),
	ADRENO_REG_DEFINE(ADRENO_REG_VFD_CONTROL_0, A3XX_VFD_CONTROL_0),
	ADRENO_REG_DEFINE(ADRENO_REG_VFD_FETCH_INSTR_0_0,
					A3XX_VFD_FETCH_INSTR_0_0),
	ADRENO_REG_DEFINE(ADRENO_REG_VFD_FETCH_INSTR_1_F,
					A3XX_VFD_FETCH_INSTR_1_F),
	ADRENO_REG_DEFINE(ADRENO_REG_VFD_INDEX_MAX, A3XX_VFD_INDEX_MAX),
	ADRENO_REG_DEFINE(ADRENO_REG_SP_VS_PVT_MEM_ADDR_REG,
				A3XX_SP_VS_PVT_MEM_ADDR_REG),
	ADRENO_REG_DEFINE(ADRENO_REG_SP_FS_PVT_MEM_ADDR_REG,
				A3XX_SP_FS_PVT_MEM_ADDR_REG),
	ADRENO_REG_DEFINE(ADRENO_REG_SP_VS_OBJ_START_REG,
				A3XX_SP_VS_OBJ_START_REG),
	ADRENO_REG_DEFINE(ADRENO_REG_SP_FS_OBJ_START_REG,
				A3XX_SP_FS_OBJ_START_REG),
	ADRENO_REG_DEFINE(ADRENO_REG_PA_SC_AA_CONFIG, REG_PA_SC_AA_CONFIG),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_PM_OVERRIDE2, REG_RBBM_PM_OVERRIDE2),
	ADRENO_REG_DEFINE(ADRENO_REG_SCRATCH_REG2, REG_SCRATCH_REG2),
	ADRENO_REG_DEFINE(ADRENO_REG_SQ_GPR_MANAGEMENT, REG_SQ_GPR_MANAGEMENT),
	ADRENO_REG_DEFINE(ADRENO_REG_SQ_INST_STORE_MANAGMENT,
				REG_SQ_INST_STORE_MANAGMENT),
	ADRENO_REG_DEFINE(ADRENO_REG_TC_CNTL_STATUS, REG_TC_CNTL_STATUS),
	ADRENO_REG_DEFINE(ADRENO_REG_TP0_CHICKEN, REG_TP0_CHICKEN),
	ADRENO_REG_DEFINE(ADRENO_REG_RBBM_RBBM_CTL, A3XX_RBBM_RBBM_CTL),
	ADRENO_REG_DEFINE(ADRENO_REG_UCHE_INVALIDATE0,
			A3XX_UCHE_CACHE_INVALIDATE0_REG),
};

struct adreno_reg_offsets a3xx_reg_offsets = {
	.offsets = a3xx_register_offsets,
	.offset_0 = ADRENO_REG_REGISTER_MAX,
};

struct adreno_gpudev adreno_a3xx_gpudev = {
	.reg_offsets = &a3xx_reg_offsets,
	.perfcounters = &a3xx_perfcounters,

	.ctxt_create = a3xx_drawctxt_create,
	.rb_init = a3xx_rb_init,
	.perfcounter_init = a3xx_perfcounter_init,
	.perfcounter_close = a3xx_perfcounter_close,
	.perfcounter_save = a3xx_perfcounter_save,
	.perfcounter_restore = a3xx_perfcounter_restore,
	.irq_control = a3xx_irq_control,
	.irq_handler = a3xx_irq_handler,
	.irq_pending = a3xx_irq_pending,
	.busy_cycles = a3xx_busy_cycles,
	.start = a3xx_start,
	.snapshot = a3xx_snapshot,
	.perfcounter_enable = a3xx_perfcounter_enable,
	.perfcounter_read = a3xx_perfcounter_read,
	.perfcounter_write = a3xx_perfcounter_write,
	.coresight_enable = a3xx_coresight_enable,
	.coresight_disable = a3xx_coresight_disable,
	.coresight_config_debug_reg = a3xx_coresight_config_debug_reg,
	.fault_detect_start = a3xx_fault_detect_start,
	.fault_detect_stop = a3xx_fault_detect_stop,
	.soft_reset = a3xx_soft_reset,
	.postmortem_dump = a3xx_postmortem_dump,
};
