/*
 * Copyright 2010 Advanced Micro Devices, Inc.
 * Copyright 2008 Red Hat Inc.
 * Copyright 2009 Jerome Glisse.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors: Dave Airlie
 *          Alex Deucher
 *          Jerome Glisse
 */
#include "drmP.h"
#include "radeon.h"
#include "evergreend.h"
#include "evergreen_reg_safe.h"
#include "cayman_reg_safe.h"

#define MAX(a,b)                   (((a)>(b))?(a):(b))
#define MIN(a,b)                   (((a)<(b))?(a):(b))

static int evergreen_cs_packet_next_reloc(struct radeon_cs_parser *p,
					  struct radeon_cs_reloc **cs_reloc);

struct evergreen_cs_track {
	u32			group_size;
	u32			nbanks;
	u32			npipes;
	u32			row_size;
	/* value we track */
	u32			nsamples;		/* unused */
	struct radeon_bo	*cb_color_bo[12];
	u32			cb_color_bo_offset[12];
	struct radeon_bo	*cb_color_fmask_bo[8];	/* unused */
	struct radeon_bo	*cb_color_cmask_bo[8];	/* unused */
	u32			cb_color_info[12];
	u32			cb_color_view[12];
	u32			cb_color_pitch[12];
	u32			cb_color_slice[12];
	u32			cb_color_attrib[12];
	u32			cb_color_cmask_slice[8];/* unused */
	u32			cb_color_fmask_slice[8];/* unused */
	u32			cb_target_mask;
	u32			cb_shader_mask; /* unused */
	u32			vgt_strmout_config;
	u32			vgt_strmout_buffer_config;
	struct radeon_bo	*vgt_strmout_bo[4];
	u32			vgt_strmout_bo_offset[4];
	u32			vgt_strmout_size[4];
	u32			db_depth_control;
	u32			db_depth_view;
	u32			db_depth_slice;
	u32			db_depth_size;
	u32			db_z_info;
	u32			db_z_read_offset;
	u32			db_z_write_offset;
	struct radeon_bo	*db_z_read_bo;
	struct radeon_bo	*db_z_write_bo;
	u32			db_s_info;
	u32			db_s_read_offset;
	u32			db_s_write_offset;
	struct radeon_bo	*db_s_read_bo;
	struct radeon_bo	*db_s_write_bo;
	bool			sx_misc_kill_all_prims;
	bool			cb_dirty;
	bool			db_dirty;
	bool			streamout_dirty;
	u32			htile_offset;
	u32			htile_surface;
	struct radeon_bo	*htile_bo;
};

static u32 evergreen_cs_get_aray_mode(u32 tiling_flags)
{
	if (tiling_flags & RADEON_TILING_MACRO)
		return ARRAY_2D_TILED_THIN1;
	else if (tiling_flags & RADEON_TILING_MICRO)
		return ARRAY_1D_TILED_THIN1;
	else
		return ARRAY_LINEAR_GENERAL;
}

static u32 evergreen_cs_get_num_banks(u32 nbanks)
{
	switch (nbanks) {
	case 2:
		return ADDR_SURF_2_BANK;
	case 4:
		return ADDR_SURF_4_BANK;
	case 8:
	default:
		return ADDR_SURF_8_BANK;
	case 16:
		return ADDR_SURF_16_BANK;
	}
}

static void evergreen_cs_track_init(struct evergreen_cs_track *track)
{
	int i;

	for (i = 0; i < 8; i++) {
		track->cb_color_fmask_bo[i] = NULL;
		track->cb_color_cmask_bo[i] = NULL;
		track->cb_color_cmask_slice[i] = 0;
		track->cb_color_fmask_slice[i] = 0;
	}

	for (i = 0; i < 12; i++) {
		track->cb_color_bo[i] = NULL;
		track->cb_color_bo_offset[i] = 0xFFFFFFFF;
		track->cb_color_info[i] = 0;
		track->cb_color_view[i] = 0xFFFFFFFF;
		track->cb_color_pitch[i] = 0;
		track->cb_color_slice[i] = 0;
	}
	track->cb_target_mask = 0xFFFFFFFF;
	track->cb_shader_mask = 0xFFFFFFFF;
	track->cb_dirty = true;

	track->db_depth_view = 0xFFFFC000;
	track->db_depth_size = 0xFFFFFFFF;
	track->db_depth_control = 0xFFFFFFFF;
	track->db_z_info = 0xFFFFFFFF;
	track->db_z_read_offset = 0xFFFFFFFF;
	track->db_z_write_offset = 0xFFFFFFFF;
	track->db_z_read_bo = NULL;
	track->db_z_write_bo = NULL;
	track->db_s_info = 0xFFFFFFFF;
	track->db_s_read_offset = 0xFFFFFFFF;
	track->db_s_write_offset = 0xFFFFFFFF;
	track->db_s_read_bo = NULL;
	track->db_s_write_bo = NULL;
	track->db_dirty = true;
	track->htile_bo = NULL;
	track->htile_offset = 0xFFFFFFFF;
	track->htile_surface = 0;

	for (i = 0; i < 4; i++) {
		track->vgt_strmout_size[i] = 0;
		track->vgt_strmout_bo[i] = NULL;
		track->vgt_strmout_bo_offset[i] = 0xFFFFFFFF;
	}
	track->streamout_dirty = true;
	track->sx_misc_kill_all_prims = false;
}

struct eg_surface {
	/* value gathered from cs */
	unsigned	nbx;
	unsigned	nby;
	unsigned	format;
	unsigned	mode;
	unsigned	nbanks;
	unsigned	bankw;
	unsigned	bankh;
	unsigned	tsplit;
	unsigned	mtilea;
	unsigned	nsamples;
	/* output value */
	unsigned	bpe;
	unsigned	layer_size;
	unsigned	palign;
	unsigned	halign;
	unsigned long	base_align;
};

static int evergreen_surface_check_linear(struct radeon_cs_parser *p,
					  struct eg_surface *surf,
					  const char *prefix)
{
	surf->layer_size = surf->nbx * surf->nby * surf->bpe * surf->nsamples;
	surf->base_align = surf->bpe;
	surf->palign = 1;
	surf->halign = 1;
	return 0;
}

static int evergreen_surface_check_linear_aligned(struct radeon_cs_parser *p,
						  struct eg_surface *surf,
						  const char *prefix)
{
	struct evergreen_cs_track *track = p->track;
	unsigned palign;

	palign = MAX(64, track->group_size / surf->bpe);
	surf->layer_size = surf->nbx * surf->nby * surf->bpe * surf->nsamples;
	surf->base_align = track->group_size;
	surf->palign = palign;
	surf->halign = 1;
	if (surf->nbx & (palign - 1)) {
		if (prefix) {
			dev_warn(p->dev, "%s:%d %s pitch %d invalid must be aligned with %d\n",
				 __func__, __LINE__, prefix, surf->nbx, palign);
		}
		return -EINVAL;
	}
	return 0;
}

static int evergreen_surface_check_1d(struct radeon_cs_parser *p,
				      struct eg_surface *surf,
				      const char *prefix)
{
	struct evergreen_cs_track *track = p->track;
	unsigned palign;

	palign = track->group_size / (8 * surf->bpe * surf->nsamples);
	palign = MAX(8, palign);
	surf->layer_size = surf->nbx * surf->nby * surf->bpe;
	surf->base_align = track->group_size;
	surf->palign = palign;
	surf->halign = 8;
	if ((surf->nbx & (palign - 1))) {
		if (prefix) {
			dev_warn(p->dev, "%s:%d %s pitch %d invalid must be aligned with %d (%d %d %d)\n",
				 __func__, __LINE__, prefix, surf->nbx, palign,
				 track->group_size, surf->bpe, surf->nsamples);
		}
		return -EINVAL;
	}
	if ((surf->nby & (8 - 1))) {
		if (prefix) {
			dev_warn(p->dev, "%s:%d %s height %d invalid must be aligned with 8\n",
				 __func__, __LINE__, prefix, surf->nby);
		}
		return -EINVAL;
	}
	return 0;
}

static int evergreen_surface_check_2d(struct radeon_cs_parser *p,
				      struct eg_surface *surf,
				      const char *prefix)
{
	struct evergreen_cs_track *track = p->track;
	unsigned palign, halign, tileb, slice_pt;

	tileb = 64 * surf->bpe * surf->nsamples;
	palign = track->group_size / (8 * surf->bpe * surf->nsamples);
	palign = MAX(8, palign);
	slice_pt = 1;
	if (tileb > surf->tsplit) {
		slice_pt = tileb / surf->tsplit;
	}
	tileb = tileb / slice_pt;
	/* macro tile width & height */
	palign = (8 * surf->bankw * track->npipes) * surf->mtilea;
	halign = (8 * surf->bankh * surf->nbanks) / surf->mtilea;
	surf->layer_size = surf->nbx * surf->nby * surf->bpe * slice_pt;
	surf->base_align = (palign / 8) * (halign / 8) * tileb;
	surf->palign = palign;
	surf->halign = halign;

	if ((surf->nbx & (palign - 1))) {
		if (prefix) {
			dev_warn(p->dev, "%s:%d %s pitch %d invalid must be aligned with %d\n",
				 __func__, __LINE__, prefix, surf->nbx, palign);
		}
		return -EINVAL;
	}
	if ((surf->nby & (halign - 1))) {
		if (prefix) {
			dev_warn(p->dev, "%s:%d %s height %d invalid must be aligned with %d\n",
				 __func__, __LINE__, prefix, surf->nby, halign);
		}
		return -EINVAL;
	}

	return 0;
}

static int evergreen_surface_check(struct radeon_cs_parser *p,
				   struct eg_surface *surf,
				   const char *prefix)
{
	/* some common value computed here */
	surf->bpe = r600_fmt_get_blocksize(surf->format);

	switch (surf->mode) {
	case ARRAY_LINEAR_GENERAL:
		return evergreen_surface_check_linear(p, surf, prefix);
	case ARRAY_LINEAR_ALIGNED:
		return evergreen_surface_check_linear_aligned(p, surf, prefix);
	case ARRAY_1D_TILED_THIN1:
		return evergreen_surface_check_1d(p, surf, prefix);
	case ARRAY_2D_TILED_THIN1:
		return evergreen_surface_check_2d(p, surf, prefix);
	default:
		dev_warn(p->dev, "%s:%d %s invalid array mode %d\n",
				__func__, __LINE__, prefix, surf->mode);
		return -EINVAL;
	}
	return -EINVAL;
}

static int evergreen_surface_value_conv_check(struct radeon_cs_parser *p,
					      struct eg_surface *surf,
					      const char *prefix)
{
	switch (surf->mode) {
	case ARRAY_2D_TILED_THIN1:
		break;
	case ARRAY_LINEAR_GENERAL:
	case ARRAY_LINEAR_ALIGNED:
	case ARRAY_1D_TILED_THIN1:
		return 0;
	default:
		dev_warn(p->dev, "%s:%d %s invalid array mode %d\n",
				__func__, __LINE__, prefix, surf->mode);
		return -EINVAL;
	}

	switch (surf->nbanks) {
	case 0: surf->nbanks = 2; break;
	case 1: surf->nbanks = 4; break;
	case 2: surf->nbanks = 8; break;
	case 3: surf->nbanks = 16; break;
	default:
		dev_warn(p->dev, "%s:%d %s invalid number of banks %d\n",
			 __func__, __LINE__, prefix, surf->nbanks);
		return -EINVAL;
	}
	switch (surf->bankw) {
	case 0: surf->bankw = 1; break;
	case 1: surf->bankw = 2; break;
	case 2: surf->bankw = 4; break;
	case 3: surf->bankw = 8; break;
	default:
		dev_warn(p->dev, "%s:%d %s invalid bankw %d\n",
			 __func__, __LINE__, prefix, surf->bankw);
		return -EINVAL;
	}
	switch (surf->bankh) {
	case 0: surf->bankh = 1; break;
	case 1: surf->bankh = 2; break;
	case 2: surf->bankh = 4; break;
	case 3: surf->bankh = 8; break;
	default:
		dev_warn(p->dev, "%s:%d %s invalid bankh %d\n",
			 __func__, __LINE__, prefix, surf->bankh);
		return -EINVAL;
	}
	switch (surf->mtilea) {
	case 0: surf->mtilea = 1; break;
	case 1: surf->mtilea = 2; break;
	case 2: surf->mtilea = 4; break;
	case 3: surf->mtilea = 8; break;
	default:
		dev_warn(p->dev, "%s:%d %s invalid macro tile aspect %d\n",
			 __func__, __LINE__, prefix, surf->mtilea);
		return -EINVAL;
	}
	switch (surf->tsplit) {
	case 0: surf->tsplit = 64; break;
	case 1: surf->tsplit = 128; break;
	case 2: surf->tsplit = 256; break;
	case 3: surf->tsplit = 512; break;
	case 4: surf->tsplit = 1024; break;
	case 5: surf->tsplit = 2048; break;
	case 6: surf->tsplit = 4096; break;
	default:
		dev_warn(p->dev, "%s:%d %s invalid tile split %d\n",
			 __func__, __LINE__, prefix, surf->tsplit);
		return -EINVAL;
	}
	return 0;
}

static int evergreen_cs_track_validate_cb(struct radeon_cs_parser *p, unsigned id)
{
	struct evergreen_cs_track *track = p->track;
	struct eg_surface surf;
	unsigned pitch, slice, mslice;
	unsigned long offset;
	int r;

	mslice = G_028C6C_SLICE_MAX(track->cb_color_view[id]) + 1;
	pitch = track->cb_color_pitch[id];
	slice = track->cb_color_slice[id];
	surf.nbx = (pitch + 1) * 8;
	surf.nby = ((slice + 1) * 64) / surf.nbx;
	surf.mode = G_028C70_ARRAY_MODE(track->cb_color_info[id]);
	surf.format = G_028C70_FORMAT(track->cb_color_info[id]);
	surf.tsplit = G_028C74_TILE_SPLIT(track->cb_color_attrib[id]);
	surf.nbanks = G_028C74_NUM_BANKS(track->cb_color_attrib[id]);
	surf.bankw = G_028C74_BANK_WIDTH(track->cb_color_attrib[id]);
	surf.bankh = G_028C74_BANK_HEIGHT(track->cb_color_attrib[id]);
	surf.mtilea = G_028C74_MACRO_TILE_ASPECT(track->cb_color_attrib[id]);
	surf.nsamples = 1;

	if (!r600_fmt_is_valid_color(surf.format)) {
		dev_warn(p->dev, "%s:%d cb invalid format %d for %d (0x%08x)\n",
			 __func__, __LINE__, surf.format,
			id, track->cb_color_info[id]);
		return -EINVAL;
	}

	r = evergreen_surface_value_conv_check(p, &surf, "cb");
	if (r) {
		return r;
	}

	r = evergreen_surface_check(p, &surf, "cb");
	if (r) {
		dev_warn(p->dev, "%s:%d cb[%d] invalid (0x%08x 0x%08x 0x%08x 0x%08x)\n",
			 __func__, __LINE__, id, track->cb_color_pitch[id],
			 track->cb_color_slice[id], track->cb_color_attrib[id],
			 track->cb_color_info[id]);
		return r;
	}

	offset = track->cb_color_bo_offset[id] << 8;
	if (offset & (surf.base_align - 1)) {
		dev_warn(p->dev, "%s:%d cb[%d] bo base %ld not aligned with %ld\n",
			 __func__, __LINE__, id, offset, surf.base_align);
		return -EINVAL;
	}

	offset += surf.layer_size * mslice;
	if (offset > radeon_bo_size(track->cb_color_bo[id])) {
		dev_warn(p->dev, "%s:%d cb[%d] bo too small (layer size %d, "
			 "offset %d, max layer %d, bo size %ld, slice %d)\n",
			 __func__, __LINE__, id, surf.layer_size,
			track->cb_color_bo_offset[id] << 8, mslice,
			radeon_bo_size(track->cb_color_bo[id]), slice);
		dev_warn(p->dev, "%s:%d problematic surf: (%d %d) (%d %d %d %d %d %d %d)\n",
			 __func__, __LINE__, surf.nbx, surf.nby,
			surf.mode, surf.bpe, surf.nsamples,
			surf.bankw, surf.bankh,
			surf.tsplit, surf.mtilea);
		return -EINVAL;
	}

	return 0;
}

static int evergreen_cs_track_validate_htile(struct radeon_cs_parser *p,
						unsigned nbx, unsigned nby)
{
	struct evergreen_cs_track *track = p->track;
	unsigned long size;

	if (track->htile_bo == NULL) {
		dev_warn(p->dev, "%s:%d htile enabled without htile surface 0x%08x\n",
				__func__, __LINE__, track->db_z_info);
		return -EINVAL;
	}

	if (G_028ABC_LINEAR(track->htile_surface)) {
		/* pitch must be 16 htiles aligned == 16 * 8 pixel aligned */
		nbx = round_up(nbx, 16 * 8);
		/* height is npipes htiles aligned == npipes * 8 pixel aligned */
		nby = round_up(nby, track->npipes * 8);
	} else {
		switch (track->npipes) {
		case 8:
			nbx = round_up(nbx, 64 * 8);
			nby = round_up(nby, 64 * 8);
			break;
		case 4:
			nbx = round_up(nbx, 64 * 8);
			nby = round_up(nby, 32 * 8);
			break;
		case 2:
			nbx = round_up(nbx, 32 * 8);
			nby = round_up(nby, 32 * 8);
			break;
		case 1:
			nbx = round_up(nbx, 32 * 8);
			nby = round_up(nby, 16 * 8);
			break;
		default:
			dev_warn(p->dev, "%s:%d invalid num pipes %d\n",
					__func__, __LINE__, track->npipes);
			return -EINVAL;
		}
	}
	/* compute number of htile */
	nbx = nbx / 8;
	nby = nby / 8;
	size = nbx * nby * 4;
	size += track->htile_offset;

	if (size > radeon_bo_size(track->htile_bo)) {
		dev_warn(p->dev, "%s:%d htile surface too small %ld for %ld (%d %d)\n",
				__func__, __LINE__, radeon_bo_size(track->htile_bo),
				size, nbx, nby);
		return -EINVAL;
	}
	return 0;
}

static int evergreen_cs_track_validate_stencil(struct radeon_cs_parser *p)
{
	struct evergreen_cs_track *track = p->track;
	struct eg_surface surf;
	unsigned pitch, slice, mslice;
	unsigned long offset;
	int r;

	mslice = G_028008_SLICE_MAX(track->db_depth_view) + 1;
	pitch = G_028058_PITCH_TILE_MAX(track->db_depth_size);
	slice = track->db_depth_slice;
	surf.nbx = (pitch + 1) * 8;
	surf.nby = ((slice + 1) * 64) / surf.nbx;
	surf.mode = G_028040_ARRAY_MODE(track->db_z_info);
	surf.format = G_028044_FORMAT(track->db_s_info);
	surf.tsplit = G_028044_TILE_SPLIT(track->db_s_info);
	surf.nbanks = G_028040_NUM_BANKS(track->db_z_info);
	surf.bankw = G_028040_BANK_WIDTH(track->db_z_info);
	surf.bankh = G_028040_BANK_HEIGHT(track->db_z_info);
	surf.mtilea = G_028040_MACRO_TILE_ASPECT(track->db_z_info);
	surf.nsamples = 1;

	if (surf.format != 1) {
		dev_warn(p->dev, "%s:%d stencil invalid format %d\n",
			 __func__, __LINE__, surf.format);
		return -EINVAL;
	}
	/* replace by color format so we can use same code */
	surf.format = V_028C70_COLOR_8;

	r = evergreen_surface_value_conv_check(p, &surf, "stencil");
	if (r) {
		return r;
	}

	r = evergreen_surface_check(p, &surf, NULL);
	if (r) {
		/* old userspace doesn't compute proper depth/stencil alignment
		 * check that alignment against a bigger byte per elements and
		 * only report if that alignment is wrong too.
		 */
		surf.format = V_028C70_COLOR_8_8_8_8;
		r = evergreen_surface_check(p, &surf, "stencil");
		if (r) {
			dev_warn(p->dev, "%s:%d stencil invalid (0x%08x 0x%08x 0x%08x 0x%08x)\n",
				 __func__, __LINE__, track->db_depth_size,
				 track->db_depth_slice, track->db_s_info, track->db_z_info);
		}
		return r;
	}

	offset = track->db_s_read_offset << 8;
	if (offset & (surf.base_align - 1)) {
		dev_warn(p->dev, "%s:%d stencil read bo base %ld not aligned with %ld\n",
			 __func__, __LINE__, offset, surf.base_align);
		return -EINVAL;
	}
	offset += surf.layer_size * mslice;
	if (offset > radeon_bo_size(track->db_s_read_bo)) {
		dev_warn(p->dev, "%s:%d stencil read bo too small (layer size %d, "
			 "offset %ld, max layer %d, bo size %ld)\n",
			 __func__, __LINE__, surf.layer_size,
			(unsigned long)track->db_s_read_offset << 8, mslice,
			radeon_bo_size(track->db_s_read_bo));
		dev_warn(p->dev, "%s:%d stencil invalid (0x%08x 0x%08x 0x%08x 0x%08x)\n",
			 __func__, __LINE__, track->db_depth_size,
			 track->db_depth_slice, track->db_s_info, track->db_z_info);
		return -EINVAL;
	}

	offset = track->db_s_write_offset << 8;
	if (offset & (surf.base_align - 1)) {
		dev_warn(p->dev, "%s:%d stencil write bo base %ld not aligned with %ld\n",
			 __func__, __LINE__, offset, surf.base_align);
		return -EINVAL;
	}
	offset += surf.layer_size * mslice;
	if (offset > radeon_bo_size(track->db_s_write_bo)) {
		dev_warn(p->dev, "%s:%d stencil write bo too small (layer size %d, "
			 "offset %ld, max layer %d, bo size %ld)\n",
			 __func__, __LINE__, surf.layer_size,
			(unsigned long)track->db_s_write_offset << 8, mslice,
			radeon_bo_size(track->db_s_write_bo));
		return -EINVAL;
	}

	/* hyperz */
	if (G_028040_TILE_SURFACE_ENABLE(track->db_z_info)) {
		r = evergreen_cs_track_validate_htile(p, surf.nbx, surf.nby);
		if (r) {
			return r;
		}
	}

	return 0;
}

static int evergreen_cs_track_validate_depth(struct radeon_cs_parser *p)
{
	struct evergreen_cs_track *track = p->track;
	struct eg_surface surf;
	unsigned pitch, slice, mslice;
	unsigned long offset;
	int r;

	mslice = G_028008_SLICE_MAX(track->db_depth_view) + 1;
	pitch = G_028058_PITCH_TILE_MAX(track->db_depth_size);
	slice = track->db_depth_slice;
	surf.nbx = (pitch + 1) * 8;
	surf.nby = ((slice + 1) * 64) / surf.nbx;
	surf.mode = G_028040_ARRAY_MODE(track->db_z_info);
	surf.format = G_028040_FORMAT(track->db_z_info);
	surf.tsplit = G_028040_TILE_SPLIT(track->db_z_info);
	surf.nbanks = G_028040_NUM_BANKS(track->db_z_info);
	surf.bankw = G_028040_BANK_WIDTH(track->db_z_info);
	surf.bankh = G_028040_BANK_HEIGHT(track->db_z_info);
	surf.mtilea = G_028040_MACRO_TILE_ASPECT(track->db_z_info);
	surf.nsamples = 1;

	switch (surf.format) {
	case V_028040_Z_16:
		surf.format = V_028C70_COLOR_16;
		break;
	case V_028040_Z_24:
	case V_028040_Z_32_FLOAT:
		surf.format = V_028C70_COLOR_8_8_8_8;
		break;
	default:
		dev_warn(p->dev, "%s:%d depth invalid format %d\n",
			 __func__, __LINE__, surf.format);
		return -EINVAL;
	}

	r = evergreen_surface_value_conv_check(p, &surf, "depth");
	if (r) {
		dev_warn(p->dev, "%s:%d depth invalid (0x%08x 0x%08x 0x%08x)\n",
			 __func__, __LINE__, track->db_depth_size,
			 track->db_depth_slice, track->db_z_info);
		return r;
	}

	r = evergreen_surface_check(p, &surf, "depth");
	if (r) {
		dev_warn(p->dev, "%s:%d depth invalid (0x%08x 0x%08x 0x%08x)\n",
			 __func__, __LINE__, track->db_depth_size,
			 track->db_depth_slice, track->db_z_info);
		return r;
	}

	offset = track->db_z_read_offset << 8;
	if (offset & (surf.base_align - 1)) {
		dev_warn(p->dev, "%s:%d stencil read bo base %ld not aligned with %ld\n",
			 __func__, __LINE__, offset, surf.base_align);
		return -EINVAL;
	}
	offset += surf.layer_size * mslice;
	if (offset > radeon_bo_size(track->db_z_read_bo)) {
		dev_warn(p->dev, "%s:%d depth read bo too small (layer size %d, "
			 "offset %ld, max layer %d, bo size %ld)\n",
			 __func__, __LINE__, surf.layer_size,
			(unsigned long)track->db_z_read_offset << 8, mslice,
			radeon_bo_size(track->db_z_read_bo));
		return -EINVAL;
	}

	offset = track->db_z_write_offset << 8;
	if (offset & (surf.base_align - 1)) {
		dev_warn(p->dev, "%s:%d stencil write bo base %ld not aligned with %ld\n",
			 __func__, __LINE__, offset, surf.base_align);
		return -EINVAL;
	}
	offset += surf.layer_size * mslice;
	if (offset > radeon_bo_size(track->db_z_write_bo)) {
		dev_warn(p->dev, "%s:%d depth write bo too small (layer size %d, "
			 "offset %ld, max layer %d, bo size %ld)\n",
			 __func__, __LINE__, surf.layer_size,
			(unsigned long)track->db_z_write_offset << 8, mslice,
			radeon_bo_size(track->db_z_write_bo));
		return -EINVAL;
	}

	/* hyperz */
	if (G_028040_TILE_SURFACE_ENABLE(track->db_z_info)) {
		r = evergreen_cs_track_validate_htile(p, surf.nbx, surf.nby);
		if (r) {
			return r;
		}
	}

	return 0;
}

static int evergreen_cs_track_validate_texture(struct radeon_cs_parser *p,
					       struct radeon_bo *texture,
					       struct radeon_bo *mipmap,
					       unsigned idx)
{
	struct eg_surface surf;
	unsigned long toffset, moffset;
	unsigned dim, llevel, mslice, width, height, depth, i;
	u32 texdw[8];
	int r;

	texdw[0] = radeon_get_ib_value(p, idx + 0);
	texdw[1] = radeon_get_ib_value(p, idx + 1);
	texdw[2] = radeon_get_ib_value(p, idx + 2);
	texdw[3] = radeon_get_ib_value(p, idx + 3);
	texdw[4] = radeon_get_ib_value(p, idx + 4);
	texdw[5] = radeon_get_ib_value(p, idx + 5);
	texdw[6] = radeon_get_ib_value(p, idx + 6);
	texdw[7] = radeon_get_ib_value(p, idx + 7);
	dim = G_030000_DIM(texdw[0]);
	llevel = G_030014_LAST_LEVEL(texdw[5]);
	mslice = G_030014_LAST_ARRAY(texdw[5]) + 1;
	width = G_030000_TEX_WIDTH(texdw[0]) + 1;
	height =  G_030004_TEX_HEIGHT(texdw[1]) + 1;
	depth = G_030004_TEX_DEPTH(texdw[1]) + 1;
	surf.format = G_03001C_DATA_FORMAT(texdw[7]);
	surf.nbx = (G_030000_PITCH(texdw[0]) + 1) * 8;
	surf.nbx = r600_fmt_get_nblocksx(surf.format, surf.nbx);
	surf.nby = r600_fmt_get_nblocksy(surf.format, height);
	surf.mode = G_030004_ARRAY_MODE(texdw[1]);
	surf.tsplit = G_030018_TILE_SPLIT(texdw[6]);
	surf.nbanks = G_03001C_NUM_BANKS(texdw[7]);
	surf.bankw = G_03001C_BANK_WIDTH(texdw[7]);
	surf.bankh = G_03001C_BANK_HEIGHT(texdw[7]);
	surf.mtilea = G_03001C_MACRO_TILE_ASPECT(texdw[7]);
	surf.nsamples = 1;
	toffset = texdw[2] << 8;
	moffset = texdw[3] << 8;

	if (!r600_fmt_is_valid_texture(surf.format, p->family)) {
		dev_warn(p->dev, "%s:%d texture invalid format %d\n",
			 __func__, __LINE__, surf.format);
		return -EINVAL;
	}
	switch (dim) {
	case V_030000_SQ_TEX_DIM_1D:
	case V_030000_SQ_TEX_DIM_2D:
	case V_030000_SQ_TEX_DIM_CUBEMAP:
	case V_030000_SQ_TEX_DIM_1D_ARRAY:
	case V_030000_SQ_TEX_DIM_2D_ARRAY:
		depth = 1;
	case V_030000_SQ_TEX_DIM_3D:
		break;
	default:
		dev_warn(p->dev, "%s:%d texture invalid dimension %d\n",
			 __func__, __LINE__, dim);
		return -EINVAL;
	}

	r = evergreen_surface_value_conv_check(p, &surf, "texture");
	if (r) {
		return r;
	}

	/* align height */
	evergreen_surface_check(p, &surf, NULL);
	surf.nby = ALIGN(surf.nby, surf.halign);

	r = evergreen_surface_check(p, &surf, "texture");
	if (r) {
		dev_warn(p->dev, "%s:%d texture invalid 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
			 __func__, __LINE__, texdw[0], texdw[1], texdw[4],
			 texdw[5], texdw[6], texdw[7]);
		return r;
	}

	/* check texture size */
	if (toffset & (surf.base_align - 1)) {
		dev_warn(p->dev, "%s:%d texture bo base %ld not aligned with %ld\n",
			 __func__, __LINE__, toffset, surf.base_align);
		return -EINVAL;
	}
	if (moffset & (surf.base_align - 1)) {
		dev_warn(p->dev, "%s:%d mipmap bo base %ld not aligned with %ld\n",
			 __func__, __LINE__, moffset, surf.base_align);
		return -EINVAL;
	}
	if (dim == SQ_TEX_DIM_3D) {
		toffset += surf.layer_size * depth;
	} else {
		toffset += surf.layer_size * mslice;
	}
	if (toffset > radeon_bo_size(texture)) {
		dev_warn(p->dev, "%s:%d texture bo too small (layer size %d, "
			 "offset %ld, max layer %d, depth %d, bo size %ld) (%d %d)\n",
			 __func__, __LINE__, surf.layer_size,
			(unsigned long)texdw[2] << 8, mslice,
			depth, radeon_bo_size(texture),
			surf.nbx, surf.nby);
		return -EINVAL;
	}

	/* check mipmap size */
	for (i = 1; i <= llevel; i++) {
		unsigned w, h, d;

		w = r600_mip_minify(width, i);
		h = r600_mip_minify(height, i);
		d = r600_mip_minify(depth, i);
		surf.nbx = r600_fmt_get_nblocksx(surf.format, w);
		surf.nby = r600_fmt_get_nblocksy(surf.format, h);

		switch (surf.mode) {
		case ARRAY_2D_TILED_THIN1:
			if (surf.nbx < surf.palign || surf.nby < surf.halign) {
				surf.mode = ARRAY_1D_TILED_THIN1;
			}
			/* recompute alignment */
			evergreen_surface_check(p, &surf, NULL);
			break;
		case ARRAY_LINEAR_GENERAL:
		case ARRAY_LINEAR_ALIGNED:
		case ARRAY_1D_TILED_THIN1:
			break;
		default:
			dev_warn(p->dev, "%s:%d invalid array mode %d\n",
				 __func__, __LINE__, surf.mode);
			return -EINVAL;
		}
		surf.nbx = ALIGN(surf.nbx, surf.palign);
		surf.nby = ALIGN(surf.nby, surf.halign);

		r = evergreen_surface_check(p, &surf, "mipmap");
		if (r) {
			return r;
		}

		if (dim == SQ_TEX_DIM_3D) {
			moffset += surf.layer_size * d;
		} else {
			moffset += surf.layer_size * mslice;
		}
		if (moffset > radeon_bo_size(mipmap)) {
			dev_warn(p->dev, "%s:%d mipmap [%d] bo too small (layer size %d, "
					"offset %ld, coffset %ld, max layer %d, depth %d, "
					"bo size %ld) level0 (%d %d %d)\n",
					__func__, __LINE__, i, surf.layer_size,
					(unsigned long)texdw[3] << 8, moffset, mslice,
					d, radeon_bo_size(mipmap),
					width, height, depth);
			dev_warn(p->dev, "%s:%d problematic surf: (%d %d) (%d %d %d %d %d %d %d)\n",
				 __func__, __LINE__, surf.nbx, surf.nby,
				surf.mode, surf.bpe, surf.nsamples,
				surf.bankw, surf.bankh,
				surf.tsplit, surf.mtilea);
			return -EINVAL;
		}
	}

	return 0;
}

static int evergreen_cs_track_check(struct radeon_cs_parser *p)
{
	struct evergreen_cs_track *track = p->track;
	unsigned tmp, i;
	int r;
	unsigned buffer_mask = 0;

	/* check streamout */
	if (track->streamout_dirty && track->vgt_strmout_config) {
		for (i = 0; i < 4; i++) {
			if (track->vgt_strmout_config & (1 << i)) {
				buffer_mask |= (track->vgt_strmout_buffer_config >> (i * 4)) & 0xf;
			}
		}

		for (i = 0; i < 4; i++) {
			if (buffer_mask & (1 << i)) {
				if (track->vgt_strmout_bo[i]) {
					u64 offset = (u64)track->vgt_strmout_bo_offset[i] +
							(u64)track->vgt_strmout_size[i];
					if (offset > radeon_bo_size(track->vgt_strmout_bo[i])) {
						DRM_ERROR("streamout %d bo too small: 0x%llx, 0x%lx\n",
							  i, offset,
							  radeon_bo_size(track->vgt_strmout_bo[i]));
						return -EINVAL;
					}
				} else {
					dev_warn(p->dev, "No buffer for streamout %d\n", i);
					return -EINVAL;
				}
			}
		}
		track->streamout_dirty = false;
	}

	if (track->sx_misc_kill_all_prims)
		return 0;

	/* check that we have a cb for each enabled target
	 */
	if (track->cb_dirty) {
		tmp = track->cb_target_mask;
		for (i = 0; i < 8; i++) {
			if ((tmp >> (i * 4)) & 0xF) {
				/* at least one component is enabled */
				if (track->cb_color_bo[i] == NULL) {
					dev_warn(p->dev, "%s:%d mask 0x%08X | 0x%08X no cb for %d\n",
						__func__, __LINE__, track->cb_target_mask, track->cb_shader_mask, i);
					return -EINVAL;
				}
				/* check cb */
				r = evergreen_cs_track_validate_cb(p, i);
				if (r) {
					return r;
				}
			}
		}
		track->cb_dirty = false;
	}

	if (track->db_dirty) {
		/* Check stencil buffer */
		if (G_028800_STENCIL_ENABLE(track->db_depth_control)) {
			r = evergreen_cs_track_validate_stencil(p);
			if (r)
				return r;
		}
		/* Check depth buffer */
		if (G_028800_Z_ENABLE(track->db_depth_control)) {
			r = evergreen_cs_track_validate_depth(p);
			if (r)
				return r;
		}
		track->db_dirty = false;
	}

	return 0;
}

/**
 * evergreen_cs_packet_parse() - parse cp packet and point ib index to next packet
 * @parser:	parser structure holding parsing context.
 * @pkt:	where to store packet informations
 *
 * Assume that chunk_ib_index is properly set. Will return -EINVAL
 * if packet is bigger than remaining ib size. or if packets is unknown.
 **/
int evergreen_cs_packet_parse(struct radeon_cs_parser *p,
			      struct radeon_cs_packet *pkt,
			      unsigned idx)
{
	struct radeon_cs_chunk *ib_chunk = &p->chunks[p->chunk_ib_idx];
	uint32_t header;

	if (idx >= ib_chunk->length_dw) {
		DRM_ERROR("Can not parse packet at %d after CS end %d !\n",
			  idx, ib_chunk->length_dw);
		return -EINVAL;
	}
	header = radeon_get_ib_value(p, idx);
	pkt->idx = idx;
	pkt->type = CP_PACKET_GET_TYPE(header);
	pkt->count = CP_PACKET_GET_COUNT(header);
	pkt->one_reg_wr = 0;
	switch (pkt->type) {
	case PACKET_TYPE0:
		pkt->reg = CP_PACKET0_GET_REG(header);
		break;
	case PACKET_TYPE3:
		pkt->opcode = CP_PACKET3_GET_OPCODE(header);
		break;
	case PACKET_TYPE2:
		pkt->count = -1;
		break;
	default:
		DRM_ERROR("Unknown packet type %d at %d !\n", pkt->type, idx);
		return -EINVAL;
	}
	if ((pkt->count + 1 + pkt->idx) >= ib_chunk->length_dw) {
		DRM_ERROR("Packet (%d:%d:%d) end after CS buffer (%d) !\n",
			  pkt->idx, pkt->type, pkt->count, ib_chunk->length_dw);
		return -EINVAL;
	}
	return 0;
}

/**
 * evergreen_cs_packet_next_reloc() - parse next packet which should be reloc packet3
 * @parser:		parser structure holding parsing context.
 * @data:		pointer to relocation data
 * @offset_start:	starting offset
 * @offset_mask:	offset mask (to align start offset on)
 * @reloc:		reloc informations
 *
 * Check next packet is relocation packet3, do bo validation and compute
 * GPU offset using the provided start.
 **/
static int evergreen_cs_packet_next_reloc(struct radeon_cs_parser *p,
					  struct radeon_cs_reloc **cs_reloc)
{
	struct radeon_cs_chunk *relocs_chunk;
	struct radeon_cs_packet p3reloc;
	unsigned idx;
	int r;

	if (p->chunk_relocs_idx == -1) {
		DRM_ERROR("No relocation chunk !\n");
		return -EINVAL;
	}
	*cs_reloc = NULL;
	relocs_chunk = &p->chunks[p->chunk_relocs_idx];
	r = evergreen_cs_packet_parse(p, &p3reloc, p->idx);
	if (r) {
		return r;
	}
	p->idx += p3reloc.count + 2;
	if (p3reloc.type != PACKET_TYPE3 || p3reloc.opcode != PACKET3_NOP) {
		DRM_ERROR("No packet3 for relocation for packet at %d.\n",
			  p3reloc.idx);
		return -EINVAL;
	}
	idx = radeon_get_ib_value(p, p3reloc.idx + 1);
	if (idx >= relocs_chunk->length_dw) {
		DRM_ERROR("Relocs at %d after relocations chunk end %d !\n",
			  idx, relocs_chunk->length_dw);
		return -EINVAL;
	}
	/* FIXME: we assume reloc size is 4 dwords */
	*cs_reloc = p->relocs_ptr[(idx / 4)];
	return 0;
}

/**
 * evergreen_cs_packet_next_vline() - parse userspace VLINE packet
 * @parser:		parser structure holding parsing context.
 *
 * Userspace sends a special sequence for VLINE waits.
 * PACKET0 - VLINE_START_END + value
 * PACKET3 - WAIT_REG_MEM poll vline status reg
 * RELOC (P3) - crtc_id in reloc.
 *
 * This function parses this and relocates the VLINE START END
 * and WAIT_REG_MEM packets to the correct crtc.
 * It also detects a switched off crtc and nulls out the
 * wait in that case.
 */
static int evergreen_cs_packet_parse_vline(struct radeon_cs_parser *p)
{
	struct drm_mode_object *obj;
	struct drm_crtc *crtc;
	struct radeon_crtc *radeon_crtc;
	struct radeon_cs_packet p3reloc, wait_reg_mem;
	int crtc_id;
	int r;
	uint32_t header, h_idx, reg, wait_reg_mem_info;
	volatile uint32_t *ib;

	ib = p->ib->ptr;

	/* parse the WAIT_REG_MEM */
	r = evergreen_cs_packet_parse(p, &wait_reg_mem, p->idx);
	if (r)
		return r;

	/* check its a WAIT_REG_MEM */
	if (wait_reg_mem.type != PACKET_TYPE3 ||
	    wait_reg_mem.opcode != PACKET3_WAIT_REG_MEM) {
		DRM_ERROR("vline wait missing WAIT_REG_MEM segment\n");
		return -EINVAL;
	}

	wait_reg_mem_info = radeon_get_ib_value(p, wait_reg_mem.idx + 1);
	/* bit 4 is reg (0) or mem (1) */
	if (wait_reg_mem_info & 0x10) {
		DRM_ERROR("vline WAIT_REG_MEM waiting on MEM rather than REG\n");
		return -EINVAL;
	}
	/* waiting for value to be equal */
	if ((wait_reg_mem_info & 0x7) != 0x3) {
		DRM_ERROR("vline WAIT_REG_MEM function not equal\n");
		return -EINVAL;
	}
	if ((radeon_get_ib_value(p, wait_reg_mem.idx + 2) << 2) != EVERGREEN_VLINE_STATUS) {
		DRM_ERROR("vline WAIT_REG_MEM bad reg\n");
		return -EINVAL;
	}

	if (radeon_get_ib_value(p, wait_reg_mem.idx + 5) != EVERGREEN_VLINE_STAT) {
		DRM_ERROR("vline WAIT_REG_MEM bad bit mask\n");
		return -EINVAL;
	}

	/* jump over the NOP */
	r = evergreen_cs_packet_parse(p, &p3reloc, p->idx + wait_reg_mem.count + 2);
	if (r)
		return r;

	h_idx = p->idx - 2;
	p->idx += wait_reg_mem.count + 2;
	p->idx += p3reloc.count + 2;

	header = radeon_get_ib_value(p, h_idx);
	crtc_id = radeon_get_ib_value(p, h_idx + 2 + 7 + 1);
	reg = CP_PACKET0_GET_REG(header);
	obj = drm_mode_object_find(p->rdev->ddev, crtc_id, DRM_MODE_OBJECT_CRTC);
	if (!obj) {
		DRM_ERROR("cannot find crtc %d\n", crtc_id);
		return -EINVAL;
	}
	crtc = obj_to_crtc(obj);
	radeon_crtc = to_radeon_crtc(crtc);
	crtc_id = radeon_crtc->crtc_id;

	if (!crtc->enabled) {
		/* if the CRTC isn't enabled - we need to nop out the WAIT_REG_MEM */
		ib[h_idx + 2] = PACKET2(0);
		ib[h_idx + 3] = PACKET2(0);
		ib[h_idx + 4] = PACKET2(0);
		ib[h_idx + 5] = PACKET2(0);
		ib[h_idx + 6] = PACKET2(0);
		ib[h_idx + 7] = PACKET2(0);
		ib[h_idx + 8] = PACKET2(0);
	} else {
		switch (reg) {
		case EVERGREEN_VLINE_START_END:
			header &= ~R600_CP_PACKET0_REG_MASK;
			header |= (EVERGREEN_VLINE_START_END + radeon_crtc->crtc_offset) >> 2;
			ib[h_idx] = header;
			ib[h_idx + 4] = (EVERGREEN_VLINE_STATUS + radeon_crtc->crtc_offset) >> 2;
			break;
		default:
			DRM_ERROR("unknown crtc reloc\n");
			return -EINVAL;
		}
	}
	return 0;
}

static int evergreen_packet0_check(struct radeon_cs_parser *p,
				   struct radeon_cs_packet *pkt,
				   unsigned idx, unsigned reg)
{
	int r;

	switch (reg) {
	case EVERGREEN_VLINE_START_END:
		r = evergreen_cs_packet_parse_vline(p);
		if (r) {
			DRM_ERROR("No reloc for ib[%d]=0x%04X\n",
					idx, reg);
			return r;
		}
		break;
	default:
		printk(KERN_ERR "Forbidden register 0x%04X in cs at %d\n",
		       reg, idx);
		return -EINVAL;
	}
	return 0;
}

static int evergreen_cs_parse_packet0(struct radeon_cs_parser *p,
				      struct radeon_cs_packet *pkt)
{
	unsigned reg, i;
	unsigned idx;
	int r;

	idx = pkt->idx + 1;
	reg = pkt->reg;
	for (i = 0; i <= pkt->count; i++, idx++, reg += 4) {
		r = evergreen_packet0_check(p, pkt, idx, reg);
		if (r) {
			return r;
		}
	}
	return 0;
}

/**
 * evergreen_cs_check_reg() - check if register is authorized or not
 * @parser: parser structure holding parsing context
 * @reg: register we are testing
 * @idx: index into the cs buffer
 *
 * This function will test against evergreen_reg_safe_bm and return 0
 * if register is safe. If register is not flag as safe this function
 * will test it against a list of register needind special handling.
 */
static int evergreen_cs_check_reg(struct radeon_cs_parser *p, u32 reg, u32 idx)
{
	struct evergreen_cs_track *track = (struct evergreen_cs_track *)p->track;
	struct radeon_cs_reloc *reloc;
	u32 last_reg;
	u32 m, i, tmp, *ib;
	int r;

	if (p->rdev->family >= CHIP_CAYMAN)
		last_reg = ARRAY_SIZE(cayman_reg_safe_bm);
	else
		last_reg = ARRAY_SIZE(evergreen_reg_safe_bm);

	i = (reg >> 7);
	if (i >= last_reg) {
		dev_warn(p->dev, "forbidden register 0x%08x at %d\n", reg, idx);
		return -EINVAL;
	}
	m = 1 << ((reg >> 2) & 31);
	if (p->rdev->family >= CHIP_CAYMAN) {
		if (!(cayman_reg_safe_bm[i] & m))
			return 0;
	} else {
		if (!(evergreen_reg_safe_bm[i] & m))
			return 0;
	}
	ib = p->ib->ptr;
	switch (reg) {
	/* force following reg to 0 in an attempt to disable out buffer
	 * which will need us to better understand how it works to perform
	 * security check on it (Jerome)
	 */
	case SQ_ESGS_RING_SIZE:
	case SQ_GSVS_RING_SIZE:
	case SQ_ESTMP_RING_SIZE:
	case SQ_GSTMP_RING_SIZE:
	case SQ_HSTMP_RING_SIZE:
	case SQ_LSTMP_RING_SIZE:
	case SQ_PSTMP_RING_SIZE:
	case SQ_VSTMP_RING_SIZE:
	case SQ_ESGS_RING_ITEMSIZE:
	case SQ_ESTMP_RING_ITEMSIZE:
	case SQ_GSTMP_RING_ITEMSIZE:
	case SQ_GSVS_RING_ITEMSIZE:
	case SQ_GS_VERT_ITEMSIZE:
	case SQ_GS_VERT_ITEMSIZE_1:
	case SQ_GS_VERT_ITEMSIZE_2:
	case SQ_GS_VERT_ITEMSIZE_3:
	case SQ_GSVS_RING_OFFSET_1:
	case SQ_GSVS_RING_OFFSET_2:
	case SQ_GSVS_RING_OFFSET_3:
	case SQ_HSTMP_RING_ITEMSIZE:
	case SQ_LSTMP_RING_ITEMSIZE:
	case SQ_PSTMP_RING_ITEMSIZE:
	case SQ_VSTMP_RING_ITEMSIZE:
	case VGT_TF_RING_SIZE:
		/* get value to populate the IB don't remove */
		/*tmp =radeon_get_ib_value(p, idx);
		  ib[idx] = 0;*/
		break;
	case SQ_ESGS_RING_BASE:
	case SQ_GSVS_RING_BASE:
	case SQ_ESTMP_RING_BASE:
	case SQ_GSTMP_RING_BASE:
	case SQ_HSTMP_RING_BASE:
	case SQ_LSTMP_RING_BASE:
	case SQ_PSTMP_RING_BASE:
	case SQ_VSTMP_RING_BASE:
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			dev_warn(p->dev, "bad SET_CONTEXT_REG "
					"0x%04X\n", reg);
			return -EINVAL;
		}
		ib[idx] += (u32)((reloc->lobj.gpu_offset >> 8) & 0xffffffff);
		break;
	case DB_DEPTH_CONTROL:
		track->db_depth_control = radeon_get_ib_value(p, idx);
		track->db_dirty = true;
		break;
	case CAYMAN_DB_EQAA:
		if (p->rdev->family < CHIP_CAYMAN) {
			dev_warn(p->dev, "bad SET_CONTEXT_REG "
				 "0x%04X\n", reg);
			return -EINVAL;
		}
		break;
	case CAYMAN_DB_DEPTH_INFO:
		if (p->rdev->family < CHIP_CAYMAN) {
			dev_warn(p->dev, "bad SET_CONTEXT_REG "
				 "0x%04X\n", reg);
			return -EINVAL;
		}
		break;
	case DB_Z_INFO:
		track->db_z_info = radeon_get_ib_value(p, idx);
		if (!(p->cs_flags & RADEON_CS_KEEP_TILING_FLAGS)) {
			r = evergreen_cs_packet_next_reloc(p, &reloc);
			if (r) {
				dev_warn(p->dev, "bad SET_CONTEXT_REG "
						"0x%04X\n", reg);
				return -EINVAL;
			}
			ib[idx] &= ~Z_ARRAY_MODE(0xf);
			track->db_z_info &= ~Z_ARRAY_MODE(0xf);
			ib[idx] |= Z_ARRAY_MODE(evergreen_cs_get_aray_mode(reloc->lobj.tiling_flags));
			track->db_z_info |= Z_ARRAY_MODE(evergreen_cs_get_aray_mode(reloc->lobj.tiling_flags));
			if (reloc->lobj.tiling_flags & RADEON_TILING_MACRO) {
				unsigned bankw, bankh, mtaspect, tile_split;

				evergreen_tiling_fields(reloc->lobj.tiling_flags,
							&bankw, &bankh, &mtaspect,
							&tile_split);
				ib[idx] |= DB_NUM_BANKS(evergreen_cs_get_num_banks(track->nbanks));
				ib[idx] |= DB_TILE_SPLIT(tile_split) |
						DB_BANK_WIDTH(bankw) |
						DB_BANK_HEIGHT(bankh) |
						DB_MACRO_TILE_ASPECT(mtaspect);
			}
		}
		track->db_dirty = true;
		break;
	case DB_STENCIL_INFO:
		track->db_s_info = radeon_get_ib_value(p, idx);
		track->db_dirty = true;
		break;
	case DB_DEPTH_VIEW:
		track->db_depth_view = radeon_get_ib_value(p, idx);
		track->db_dirty = true;
		break;
	case DB_DEPTH_SIZE:
		track->db_depth_size = radeon_get_ib_value(p, idx);
		track->db_dirty = true;
		break;
	case R_02805C_DB_DEPTH_SLICE:
		track->db_depth_slice = radeon_get_ib_value(p, idx);
		track->db_dirty = true;
		break;
	case DB_Z_READ_BASE:
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			dev_warn(p->dev, "bad SET_CONTEXT_REG "
					"0x%04X\n", reg);
			return -EINVAL;
		}
		track->db_z_read_offset = radeon_get_ib_value(p, idx);
		ib[idx] += (u32)((reloc->lobj.gpu_offset >> 8) & 0xffffffff);
		track->db_z_read_bo = reloc->robj;
		track->db_dirty = true;
		break;
	case DB_Z_WRITE_BASE:
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			dev_warn(p->dev, "bad SET_CONTEXT_REG "
					"0x%04X\n", reg);
			return -EINVAL;
		}
		track->db_z_write_offset = radeon_get_ib_value(p, idx);
		ib[idx] += (u32)((reloc->lobj.gpu_offset >> 8) & 0xffffffff);
		track->db_z_write_bo = reloc->robj;
		track->db_dirty = true;
		break;
	case DB_STENCIL_READ_BASE:
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			dev_warn(p->dev, "bad SET_CONTEXT_REG "
					"0x%04X\n", reg);
			return -EINVAL;
		}
		track->db_s_read_offset = radeon_get_ib_value(p, idx);
		ib[idx] += (u32)((reloc->lobj.gpu_offset >> 8) & 0xffffffff);
		track->db_s_read_bo = reloc->robj;
		track->db_dirty = true;
		break;
	case DB_STENCIL_WRITE_BASE:
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			dev_warn(p->dev, "bad SET_CONTEXT_REG "
					"0x%04X\n", reg);
			return -EINVAL;
		}
		track->db_s_write_offset = radeon_get_ib_value(p, idx);
		ib[idx] += (u32)((reloc->lobj.gpu_offset >> 8) & 0xffffffff);
		track->db_s_write_bo = reloc->robj;
		track->db_dirty = true;
		break;
	case VGT_STRMOUT_CONFIG:
		track->vgt_strmout_config = radeon_get_ib_value(p, idx);
		track->streamout_dirty = true;
		break;
	case VGT_STRMOUT_BUFFER_CONFIG:
		track->vgt_strmout_buffer_config = radeon_get_ib_value(p, idx);
		track->streamout_dirty = true;
		break;
	case VGT_STRMOUT_BUFFER_BASE_0:
	case VGT_STRMOUT_BUFFER_BASE_1:
	case VGT_STRMOUT_BUFFER_BASE_2:
	case VGT_STRMOUT_BUFFER_BASE_3:
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			dev_warn(p->dev, "bad SET_CONTEXT_REG "
					"0x%04X\n", reg);
			return -EINVAL;
		}
		tmp = (reg - VGT_STRMOUT_BUFFER_BASE_0) / 16;
		track->vgt_strmout_bo_offset[tmp] = radeon_get_ib_value(p, idx) << 8;
		ib[idx] += (u32)((reloc->lobj.gpu_offset >> 8) & 0xffffffff);
		track->vgt_strmout_bo[tmp] = reloc->robj;
		track->streamout_dirty = true;
		break;
	case VGT_STRMOUT_BUFFER_SIZE_0:
	case VGT_STRMOUT_BUFFER_SIZE_1:
	case VGT_STRMOUT_BUFFER_SIZE_2:
	case VGT_STRMOUT_BUFFER_SIZE_3:
		tmp = (reg - VGT_STRMOUT_BUFFER_SIZE_0) / 16;
		/* size in register is DWs, convert to bytes */
		track->vgt_strmout_size[tmp] = radeon_get_ib_value(p, idx) * 4;
		track->streamout_dirty = true;
		break;
	case CP_COHER_BASE:
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			dev_warn(p->dev, "missing reloc for CP_COHER_BASE "
					"0x%04X\n", reg);
			return -EINVAL;
		}
		ib[idx] += (u32)((reloc->lobj.gpu_offset >> 8) & 0xffffffff);
	case CB_TARGET_MASK:
		track->cb_target_mask = radeon_get_ib_value(p, idx);
		track->cb_dirty = true;
		break;
	case CB_SHADER_MASK:
		track->cb_shader_mask = radeon_get_ib_value(p, idx);
		track->cb_dirty = true;
		break;
	case PA_SC_AA_CONFIG:
		if (p->rdev->family >= CHIP_CAYMAN) {
			dev_warn(p->dev, "bad SET_CONTEXT_REG "
				 "0x%04X\n", reg);
			return -EINVAL;
		}
		tmp = radeon_get_ib_value(p, idx) & MSAA_NUM_SAMPLES_MASK;
		track->nsamples = 1 << tmp;
		break;
	case CAYMAN_PA_SC_AA_CONFIG:
		if (p->rdev->family < CHIP_CAYMAN) {
			dev_warn(p->dev, "bad SET_CONTEXT_REG "
				 "0x%04X\n", reg);
			return -EINVAL;
		}
		tmp = radeon_get_ib_value(p, idx) & CAYMAN_MSAA_NUM_SAMPLES_MASK;
		track->nsamples = 1 << tmp;
		break;
	case CB_COLOR0_VIEW:
	case CB_COLOR1_VIEW:
	case CB_COLOR2_VIEW:
	case CB_COLOR3_VIEW:
	case CB_COLOR4_VIEW:
	case CB_COLOR5_VIEW:
	case CB_COLOR6_VIEW:
	case CB_COLOR7_VIEW:
		tmp = (reg - CB_COLOR0_VIEW) / 0x3c;
		track->cb_color_view[tmp] = radeon_get_ib_value(p, idx);
		track->cb_dirty = true;
		break;
	case CB_COLOR8_VIEW:
	case CB_COLOR9_VIEW:
	case CB_COLOR10_VIEW:
	case CB_COLOR11_VIEW:
		tmp = ((reg - CB_COLOR8_VIEW) / 0x1c) + 8;
		track->cb_color_view[tmp] = radeon_get_ib_value(p, idx);
		track->cb_dirty = true;
		break;
	case CB_COLOR0_INFO:
	case CB_COLOR1_INFO:
	case CB_COLOR2_INFO:
	case CB_COLOR3_INFO:
	case CB_COLOR4_INFO:
	case CB_COLOR5_INFO:
	case CB_COLOR6_INFO:
	case CB_COLOR7_INFO:
		tmp = (reg - CB_COLOR0_INFO) / 0x3c;
		track->cb_color_info[tmp] = radeon_get_ib_value(p, idx);
		if (!(p->cs_flags & RADEON_CS_KEEP_TILING_FLAGS)) {
			r = evergreen_cs_packet_next_reloc(p, &reloc);
			if (r) {
				dev_warn(p->dev, "bad SET_CONTEXT_REG "
						"0x%04X\n", reg);
				return -EINVAL;
			}
			ib[idx] |= CB_ARRAY_MODE(evergreen_cs_get_aray_mode(reloc->lobj.tiling_flags));
			track->cb_color_info[tmp] |= CB_ARRAY_MODE(evergreen_cs_get_aray_mode(reloc->lobj.tiling_flags));
		}
		track->cb_dirty = true;
		break;
	case CB_COLOR8_INFO:
	case CB_COLOR9_INFO:
	case CB_COLOR10_INFO:
	case CB_COLOR11_INFO:
		tmp = ((reg - CB_COLOR8_INFO) / 0x1c) + 8;
		track->cb_color_info[tmp] = radeon_get_ib_value(p, idx);
		if (!(p->cs_flags & RADEON_CS_KEEP_TILING_FLAGS)) {
			r = evergreen_cs_packet_next_reloc(p, &reloc);
			if (r) {
				dev_warn(p->dev, "bad SET_CONTEXT_REG "
						"0x%04X\n", reg);
				return -EINVAL;
			}
			ib[idx] |= CB_ARRAY_MODE(evergreen_cs_get_aray_mode(reloc->lobj.tiling_flags));
			track->cb_color_info[tmp] |= CB_ARRAY_MODE(evergreen_cs_get_aray_mode(reloc->lobj.tiling_flags));
		}
		track->cb_dirty = true;
		break;
	case CB_COLOR0_PITCH:
	case CB_COLOR1_PITCH:
	case CB_COLOR2_PITCH:
	case CB_COLOR3_PITCH:
	case CB_COLOR4_PITCH:
	case CB_COLOR5_PITCH:
	case CB_COLOR6_PITCH:
	case CB_COLOR7_PITCH:
		tmp = (reg - CB_COLOR0_PITCH) / 0x3c;
		track->cb_color_pitch[tmp] = radeon_get_ib_value(p, idx);
		track->cb_dirty = true;
		break;
	case CB_COLOR8_PITCH:
	case CB_COLOR9_PITCH:
	case CB_COLOR10_PITCH:
	case CB_COLOR11_PITCH:
		tmp = ((reg - CB_COLOR8_PITCH) / 0x1c) + 8;
		track->cb_color_pitch[tmp] = radeon_get_ib_value(p, idx);
		track->cb_dirty = true;
		break;
	case CB_COLOR0_SLICE:
	case CB_COLOR1_SLICE:
	case CB_COLOR2_SLICE:
	case CB_COLOR3_SLICE:
	case CB_COLOR4_SLICE:
	case CB_COLOR5_SLICE:
	case CB_COLOR6_SLICE:
	case CB_COLOR7_SLICE:
		tmp = (reg - CB_COLOR0_SLICE) / 0x3c;
		track->cb_color_slice[tmp] = radeon_get_ib_value(p, idx);
		track->cb_dirty = true;
		break;
	case CB_COLOR8_SLICE:
	case CB_COLOR9_SLICE:
	case CB_COLOR10_SLICE:
	case CB_COLOR11_SLICE:
		tmp = ((reg - CB_COLOR8_SLICE) / 0x1c) + 8;
		track->cb_color_slice[tmp] = radeon_get_ib_value(p, idx);
		track->cb_dirty = true;
		break;
	case CB_COLOR0_ATTRIB:
	case CB_COLOR1_ATTRIB:
	case CB_COLOR2_ATTRIB:
	case CB_COLOR3_ATTRIB:
	case CB_COLOR4_ATTRIB:
	case CB_COLOR5_ATTRIB:
	case CB_COLOR6_ATTRIB:
	case CB_COLOR7_ATTRIB:
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			dev_warn(p->dev, "bad SET_CONTEXT_REG "
					"0x%04X\n", reg);
			return -EINVAL;
		}
		if (!(p->cs_flags & RADEON_CS_KEEP_TILING_FLAGS)) {
			if (reloc->lobj.tiling_flags & RADEON_TILING_MACRO) {
				unsigned bankw, bankh, mtaspect, tile_split;

				evergreen_tiling_fields(reloc->lobj.tiling_flags,
							&bankw, &bankh, &mtaspect,
							&tile_split);
				ib[idx] |= CB_NUM_BANKS(evergreen_cs_get_num_banks(track->nbanks));
				ib[idx] |= CB_TILE_SPLIT(tile_split) |
					   CB_BANK_WIDTH(bankw) |
					   CB_BANK_HEIGHT(bankh) |
					   CB_MACRO_TILE_ASPECT(mtaspect);
			}
		}
		tmp = ((reg - CB_COLOR0_ATTRIB) / 0x3c);
		track->cb_color_attrib[tmp] = ib[idx];
		track->cb_dirty = true;
		break;
	case CB_COLOR8_ATTRIB:
	case CB_COLOR9_ATTRIB:
	case CB_COLOR10_ATTRIB:
	case CB_COLOR11_ATTRIB:
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			dev_warn(p->dev, "bad SET_CONTEXT_REG "
					"0x%04X\n", reg);
			return -EINVAL;
		}
		if (!(p->cs_flags & RADEON_CS_KEEP_TILING_FLAGS)) {
			if (reloc->lobj.tiling_flags & RADEON_TILING_MACRO) {
				unsigned bankw, bankh, mtaspect, tile_split;

				evergreen_tiling_fields(reloc->lobj.tiling_flags,
							&bankw, &bankh, &mtaspect,
							&tile_split);
				ib[idx] |= CB_NUM_BANKS(evergreen_cs_get_num_banks(track->nbanks));
				ib[idx] |= CB_TILE_SPLIT(tile_split) |
					   CB_BANK_WIDTH(bankw) |
					   CB_BANK_HEIGHT(bankh) |
					   CB_MACRO_TILE_ASPECT(mtaspect);
			}
		}
		tmp = ((reg - CB_COLOR8_ATTRIB) / 0x1c) + 8;
		track->cb_color_attrib[tmp] = ib[idx];
		track->cb_dirty = true;
		break;
	case CB_COLOR0_FMASK:
	case CB_COLOR1_FMASK:
	case CB_COLOR2_FMASK:
	case CB_COLOR3_FMASK:
	case CB_COLOR4_FMASK:
	case CB_COLOR5_FMASK:
	case CB_COLOR6_FMASK:
	case CB_COLOR7_FMASK:
		tmp = (reg - CB_COLOR0_FMASK) / 0x3c;
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			dev_err(p->dev, "bad SET_CONTEXT_REG 0x%04X\n", reg);
			return -EINVAL;
		}
		ib[idx] += (u32)((reloc->lobj.gpu_offset >> 8) & 0xffffffff);
		track->cb_color_fmask_bo[tmp] = reloc->robj;
		break;
	case CB_COLOR0_CMASK:
	case CB_COLOR1_CMASK:
	case CB_COLOR2_CMASK:
	case CB_COLOR3_CMASK:
	case CB_COLOR4_CMASK:
	case CB_COLOR5_CMASK:
	case CB_COLOR6_CMASK:
	case CB_COLOR7_CMASK:
		tmp = (reg - CB_COLOR0_CMASK) / 0x3c;
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			dev_err(p->dev, "bad SET_CONTEXT_REG 0x%04X\n", reg);
			return -EINVAL;
		}
		ib[idx] += (u32)((reloc->lobj.gpu_offset >> 8) & 0xffffffff);
		track->cb_color_cmask_bo[tmp] = reloc->robj;
		break;
	case CB_COLOR0_FMASK_SLICE:
	case CB_COLOR1_FMASK_SLICE:
	case CB_COLOR2_FMASK_SLICE:
	case CB_COLOR3_FMASK_SLICE:
	case CB_COLOR4_FMASK_SLICE:
	case CB_COLOR5_FMASK_SLICE:
	case CB_COLOR6_FMASK_SLICE:
	case CB_COLOR7_FMASK_SLICE:
		tmp = (reg - CB_COLOR0_FMASK_SLICE) / 0x3c;
		track->cb_color_fmask_slice[tmp] = radeon_get_ib_value(p, idx);
		break;
	case CB_COLOR0_CMASK_SLICE:
	case CB_COLOR1_CMASK_SLICE:
	case CB_COLOR2_CMASK_SLICE:
	case CB_COLOR3_CMASK_SLICE:
	case CB_COLOR4_CMASK_SLICE:
	case CB_COLOR5_CMASK_SLICE:
	case CB_COLOR6_CMASK_SLICE:
	case CB_COLOR7_CMASK_SLICE:
		tmp = (reg - CB_COLOR0_CMASK_SLICE) / 0x3c;
		track->cb_color_cmask_slice[tmp] = radeon_get_ib_value(p, idx);
		break;
	case CB_COLOR0_BASE:
	case CB_COLOR1_BASE:
	case CB_COLOR2_BASE:
	case CB_COLOR3_BASE:
	case CB_COLOR4_BASE:
	case CB_COLOR5_BASE:
	case CB_COLOR6_BASE:
	case CB_COLOR7_BASE:
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			dev_warn(p->dev, "bad SET_CONTEXT_REG "
					"0x%04X\n", reg);
			return -EINVAL;
		}
		tmp = (reg - CB_COLOR0_BASE) / 0x3c;
		track->cb_color_bo_offset[tmp] = radeon_get_ib_value(p, idx);
		ib[idx] += (u32)((reloc->lobj.gpu_offset >> 8) & 0xffffffff);
		track->cb_color_bo[tmp] = reloc->robj;
		track->cb_dirty = true;
		break;
	case CB_COLOR8_BASE:
	case CB_COLOR9_BASE:
	case CB_COLOR10_BASE:
	case CB_COLOR11_BASE:
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			dev_warn(p->dev, "bad SET_CONTEXT_REG "
					"0x%04X\n", reg);
			return -EINVAL;
		}
		tmp = ((reg - CB_COLOR8_BASE) / 0x1c) + 8;
		track->cb_color_bo_offset[tmp] = radeon_get_ib_value(p, idx);
		ib[idx] += (u32)((reloc->lobj.gpu_offset >> 8) & 0xffffffff);
		track->cb_color_bo[tmp] = reloc->robj;
		track->cb_dirty = true;
		break;
	case DB_HTILE_DATA_BASE:
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			dev_warn(p->dev, "bad SET_CONTEXT_REG "
					"0x%04X\n", reg);
			return -EINVAL;
		}
		track->htile_offset = radeon_get_ib_value(p, idx);
		ib[idx] += (u32)((reloc->lobj.gpu_offset >> 8) & 0xffffffff);
		track->htile_bo = reloc->robj;
		track->db_dirty = true;
		break;
	case DB_HTILE_SURFACE:
		/* 8x8 only */
		track->htile_surface = radeon_get_ib_value(p, idx);
		track->db_dirty = true;
		break;
	case CB_IMMED0_BASE:
	case CB_IMMED1_BASE:
	case CB_IMMED2_BASE:
	case CB_IMMED3_BASE:
	case CB_IMMED4_BASE:
	case CB_IMMED5_BASE:
	case CB_IMMED6_BASE:
	case CB_IMMED7_BASE:
	case CB_IMMED8_BASE:
	case CB_IMMED9_BASE:
	case CB_IMMED10_BASE:
	case CB_IMMED11_BASE:
	case SQ_PGM_START_FS:
	case SQ_PGM_START_ES:
	case SQ_PGM_START_VS:
	case SQ_PGM_START_GS:
	case SQ_PGM_START_PS:
	case SQ_PGM_START_HS:
	case SQ_PGM_START_LS:
	case SQ_CONST_MEM_BASE:
	case SQ_ALU_CONST_CACHE_GS_0:
	case SQ_ALU_CONST_CACHE_GS_1:
	case SQ_ALU_CONST_CACHE_GS_2:
	case SQ_ALU_CONST_CACHE_GS_3:
	case SQ_ALU_CONST_CACHE_GS_4:
	case SQ_ALU_CONST_CACHE_GS_5:
	case SQ_ALU_CONST_CACHE_GS_6:
	case SQ_ALU_CONST_CACHE_GS_7:
	case SQ_ALU_CONST_CACHE_GS_8:
	case SQ_ALU_CONST_CACHE_GS_9:
	case SQ_ALU_CONST_CACHE_GS_10:
	case SQ_ALU_CONST_CACHE_GS_11:
	case SQ_ALU_CONST_CACHE_GS_12:
	case SQ_ALU_CONST_CACHE_GS_13:
	case SQ_ALU_CONST_CACHE_GS_14:
	case SQ_ALU_CONST_CACHE_GS_15:
	case SQ_ALU_CONST_CACHE_PS_0:
	case SQ_ALU_CONST_CACHE_PS_1:
	case SQ_ALU_CONST_CACHE_PS_2:
	case SQ_ALU_CONST_CACHE_PS_3:
	case SQ_ALU_CONST_CACHE_PS_4:
	case SQ_ALU_CONST_CACHE_PS_5:
	case SQ_ALU_CONST_CACHE_PS_6:
	case SQ_ALU_CONST_CACHE_PS_7:
	case SQ_ALU_CONST_CACHE_PS_8:
	case SQ_ALU_CONST_CACHE_PS_9:
	case SQ_ALU_CONST_CACHE_PS_10:
	case SQ_ALU_CONST_CACHE_PS_11:
	case SQ_ALU_CONST_CACHE_PS_12:
	case SQ_ALU_CONST_CACHE_PS_13:
	case SQ_ALU_CONST_CACHE_PS_14:
	case SQ_ALU_CONST_CACHE_PS_15:
	case SQ_ALU_CONST_CACHE_VS_0:
	case SQ_ALU_CONST_CACHE_VS_1:
	case SQ_ALU_CONST_CACHE_VS_2:
	case SQ_ALU_CONST_CACHE_VS_3:
	case SQ_ALU_CONST_CACHE_VS_4:
	case SQ_ALU_CONST_CACHE_VS_5:
	case SQ_ALU_CONST_CACHE_VS_6:
	case SQ_ALU_CONST_CACHE_VS_7:
	case SQ_ALU_CONST_CACHE_VS_8:
	case SQ_ALU_CONST_CACHE_VS_9:
	case SQ_ALU_CONST_CACHE_VS_10:
	case SQ_ALU_CONST_CACHE_VS_11:
	case SQ_ALU_CONST_CACHE_VS_12:
	case SQ_ALU_CONST_CACHE_VS_13:
	case SQ_ALU_CONST_CACHE_VS_14:
	case SQ_ALU_CONST_CACHE_VS_15:
	case SQ_ALU_CONST_CACHE_HS_0:
	case SQ_ALU_CONST_CACHE_HS_1:
	case SQ_ALU_CONST_CACHE_HS_2:
	case SQ_ALU_CONST_CACHE_HS_3:
	case SQ_ALU_CONST_CACHE_HS_4:
	case SQ_ALU_CONST_CACHE_HS_5:
	case SQ_ALU_CONST_CACHE_HS_6:
	case SQ_ALU_CONST_CACHE_HS_7:
	case SQ_ALU_CONST_CACHE_HS_8:
	case SQ_ALU_CONST_CACHE_HS_9:
	case SQ_ALU_CONST_CACHE_HS_10:
	case SQ_ALU_CONST_CACHE_HS_11:
	case SQ_ALU_CONST_CACHE_HS_12:
	case SQ_ALU_CONST_CACHE_HS_13:
	case SQ_ALU_CONST_CACHE_HS_14:
	case SQ_ALU_CONST_CACHE_HS_15:
	case SQ_ALU_CONST_CACHE_LS_0:
	case SQ_ALU_CONST_CACHE_LS_1:
	case SQ_ALU_CONST_CACHE_LS_2:
	case SQ_ALU_CONST_CACHE_LS_3:
	case SQ_ALU_CONST_CACHE_LS_4:
	case SQ_ALU_CONST_CACHE_LS_5:
	case SQ_ALU_CONST_CACHE_LS_6:
	case SQ_ALU_CONST_CACHE_LS_7:
	case SQ_ALU_CONST_CACHE_LS_8:
	case SQ_ALU_CONST_CACHE_LS_9:
	case SQ_ALU_CONST_CACHE_LS_10:
	case SQ_ALU_CONST_CACHE_LS_11:
	case SQ_ALU_CONST_CACHE_LS_12:
	case SQ_ALU_CONST_CACHE_LS_13:
	case SQ_ALU_CONST_CACHE_LS_14:
	case SQ_ALU_CONST_CACHE_LS_15:
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			dev_warn(p->dev, "bad SET_CONTEXT_REG "
					"0x%04X\n", reg);
			return -EINVAL;
		}
		ib[idx] += (u32)((reloc->lobj.gpu_offset >> 8) & 0xffffffff);
		break;
	case SX_MEMORY_EXPORT_BASE:
		if (p->rdev->family >= CHIP_CAYMAN) {
			dev_warn(p->dev, "bad SET_CONFIG_REG "
				 "0x%04X\n", reg);
			return -EINVAL;
		}
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			dev_warn(p->dev, "bad SET_CONFIG_REG "
					"0x%04X\n", reg);
			return -EINVAL;
		}
		ib[idx] += (u32)((reloc->lobj.gpu_offset >> 8) & 0xffffffff);
		break;
	case CAYMAN_SX_SCATTER_EXPORT_BASE:
		if (p->rdev->family < CHIP_CAYMAN) {
			dev_warn(p->dev, "bad SET_CONTEXT_REG "
				 "0x%04X\n", reg);
			return -EINVAL;
		}
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			dev_warn(p->dev, "bad SET_CONTEXT_REG "
					"0x%04X\n", reg);
			return -EINVAL;
		}
		ib[idx] += (u32)((reloc->lobj.gpu_offset >> 8) & 0xffffffff);
		break;
	case SX_MISC:
		track->sx_misc_kill_all_prims = (radeon_get_ib_value(p, idx) & 0x1) != 0;
		break;
	default:
		dev_warn(p->dev, "forbidden register 0x%08x at %d\n", reg, idx);
		return -EINVAL;
	}
	return 0;
}

static bool evergreen_is_safe_reg(struct radeon_cs_parser *p, u32 reg, u32 idx)
{
	u32 last_reg, m, i;

	if (p->rdev->family >= CHIP_CAYMAN)
		last_reg = ARRAY_SIZE(cayman_reg_safe_bm);
	else
		last_reg = ARRAY_SIZE(evergreen_reg_safe_bm);

	i = (reg >> 7);
	if (i >= last_reg) {
		dev_warn(p->dev, "forbidden register 0x%08x at %d\n", reg, idx);
		return false;
	}
	m = 1 << ((reg >> 2) & 31);
	if (p->rdev->family >= CHIP_CAYMAN) {
		if (!(cayman_reg_safe_bm[i] & m))
			return true;
	} else {
		if (!(evergreen_reg_safe_bm[i] & m))
			return true;
	}
	dev_warn(p->dev, "forbidden register 0x%08x at %d\n", reg, idx);
	return false;
}

static int evergreen_packet3_check(struct radeon_cs_parser *p,
				   struct radeon_cs_packet *pkt)
{
	struct radeon_cs_reloc *reloc;
	struct evergreen_cs_track *track;
	volatile u32 *ib;
	unsigned idx;
	unsigned i;
	unsigned start_reg, end_reg, reg;
	int r;
	u32 idx_value;

	track = (struct evergreen_cs_track *)p->track;
	ib = p->ib->ptr;
	idx = pkt->idx + 1;
	idx_value = radeon_get_ib_value(p, idx);

	switch (pkt->opcode) {
	case PACKET3_SET_PREDICATION:
	{
		int pred_op;
		int tmp;
		uint64_t offset;

		if (pkt->count != 1) {
			DRM_ERROR("bad SET PREDICATION\n");
			return -EINVAL;
		}

		tmp = radeon_get_ib_value(p, idx + 1);
		pred_op = (tmp >> 16) & 0x7;

		/* for the clear predicate operation */
		if (pred_op == 0)
			return 0;

		if (pred_op > 2) {
			DRM_ERROR("bad SET PREDICATION operation %d\n", pred_op);
			return -EINVAL;
		}

		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			DRM_ERROR("bad SET PREDICATION\n");
			return -EINVAL;
		}

		offset = reloc->lobj.gpu_offset +
		         (idx_value & 0xfffffff0) +
		         ((u64)(tmp & 0xff) << 32);

		ib[idx + 0] = offset;
		ib[idx + 1] = (tmp & 0xffffff00) | (upper_32_bits(offset) & 0xff);
	}
	break;
	case PACKET3_CONTEXT_CONTROL:
		if (pkt->count != 1) {
			DRM_ERROR("bad CONTEXT_CONTROL\n");
			return -EINVAL;
		}
		break;
	case PACKET3_INDEX_TYPE:
	case PACKET3_NUM_INSTANCES:
	case PACKET3_CLEAR_STATE:
		if (pkt->count) {
			DRM_ERROR("bad INDEX_TYPE/NUM_INSTANCES/CLEAR_STATE\n");
			return -EINVAL;
		}
		break;
	case CAYMAN_PACKET3_DEALLOC_STATE:
		if (p->rdev->family < CHIP_CAYMAN) {
			DRM_ERROR("bad PACKET3_DEALLOC_STATE\n");
			return -EINVAL;
		}
		if (pkt->count) {
			DRM_ERROR("bad INDEX_TYPE/NUM_INSTANCES/CLEAR_STATE\n");
			return -EINVAL;
		}
		break;
	case PACKET3_INDEX_BASE:
	{
		uint64_t offset;

		if (pkt->count != 1) {
			DRM_ERROR("bad INDEX_BASE\n");
			return -EINVAL;
		}
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			DRM_ERROR("bad INDEX_BASE\n");
			return -EINVAL;
		}

		offset = reloc->lobj.gpu_offset +
		         idx_value +
		         ((u64)(radeon_get_ib_value(p, idx+1) & 0xff) << 32);

		ib[idx+0] = offset;
		ib[idx+1] = upper_32_bits(offset) & 0xff;

		r = evergreen_cs_track_check(p);
		if (r) {
			dev_warn(p->dev, "%s:%d invalid cmd stream\n", __func__, __LINE__);
			return r;
		}
		break;
	}
	case PACKET3_DRAW_INDEX:
	{
		uint64_t offset;
		if (pkt->count != 3) {
			DRM_ERROR("bad DRAW_INDEX\n");
			return -EINVAL;
		}
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			DRM_ERROR("bad DRAW_INDEX\n");
			return -EINVAL;
		}

		offset = reloc->lobj.gpu_offset +
		         idx_value +
		         ((u64)(radeon_get_ib_value(p, idx+1) & 0xff) << 32);

		ib[idx+0] = offset;
		ib[idx+1] = upper_32_bits(offset) & 0xff;

		r = evergreen_cs_track_check(p);
		if (r) {
			dev_warn(p->dev, "%s:%d invalid cmd stream\n", __func__, __LINE__);
			return r;
		}
		break;
	}
	case PACKET3_DRAW_INDEX_2:
	{
		uint64_t offset;

		if (pkt->count != 4) {
			DRM_ERROR("bad DRAW_INDEX_2\n");
			return -EINVAL;
		}
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			DRM_ERROR("bad DRAW_INDEX_2\n");
			return -EINVAL;
		}

		offset = reloc->lobj.gpu_offset +
		         radeon_get_ib_value(p, idx+1) +
		         ((u64)(radeon_get_ib_value(p, idx+2) & 0xff) << 32);

		ib[idx+1] = offset;
		ib[idx+2] = upper_32_bits(offset) & 0xff;

		r = evergreen_cs_track_check(p);
		if (r) {
			dev_warn(p->dev, "%s:%d invalid cmd stream\n", __func__, __LINE__);
			return r;
		}
		break;
	}
	case PACKET3_DRAW_INDEX_AUTO:
		if (pkt->count != 1) {
			DRM_ERROR("bad DRAW_INDEX_AUTO\n");
			return -EINVAL;
		}
		r = evergreen_cs_track_check(p);
		if (r) {
			dev_warn(p->dev, "%s:%d invalid cmd stream %d\n", __func__, __LINE__, idx);
			return r;
		}
		break;
	case PACKET3_DRAW_INDEX_MULTI_AUTO:
		if (pkt->count != 2) {
			DRM_ERROR("bad DRAW_INDEX_MULTI_AUTO\n");
			return -EINVAL;
		}
		r = evergreen_cs_track_check(p);
		if (r) {
			dev_warn(p->dev, "%s:%d invalid cmd stream %d\n", __func__, __LINE__, idx);
			return r;
		}
		break;
	case PACKET3_DRAW_INDEX_IMMD:
		if (pkt->count < 2) {
			DRM_ERROR("bad DRAW_INDEX_IMMD\n");
			return -EINVAL;
		}
		r = evergreen_cs_track_check(p);
		if (r) {
			dev_warn(p->dev, "%s:%d invalid cmd stream\n", __func__, __LINE__);
			return r;
		}
		break;
	case PACKET3_DRAW_INDEX_OFFSET:
		if (pkt->count != 2) {
			DRM_ERROR("bad DRAW_INDEX_OFFSET\n");
			return -EINVAL;
		}
		r = evergreen_cs_track_check(p);
		if (r) {
			dev_warn(p->dev, "%s:%d invalid cmd stream\n", __func__, __LINE__);
			return r;
		}
		break;
	case PACKET3_DRAW_INDEX_OFFSET_2:
		if (pkt->count != 3) {
			DRM_ERROR("bad DRAW_INDEX_OFFSET_2\n");
			return -EINVAL;
		}
		r = evergreen_cs_track_check(p);
		if (r) {
			dev_warn(p->dev, "%s:%d invalid cmd stream\n", __func__, __LINE__);
			return r;
		}
		break;
	case PACKET3_DISPATCH_DIRECT:
		if (pkt->count != 3) {
			DRM_ERROR("bad DISPATCH_DIRECT\n");
			return -EINVAL;
		}
		r = evergreen_cs_track_check(p);
		if (r) {
			dev_warn(p->dev, "%s:%d invalid cmd stream %d\n", __func__, __LINE__, idx);
			return r;
		}
		break;
	case PACKET3_DISPATCH_INDIRECT:
		if (pkt->count != 1) {
			DRM_ERROR("bad DISPATCH_INDIRECT\n");
			return -EINVAL;
		}
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			DRM_ERROR("bad DISPATCH_INDIRECT\n");
			return -EINVAL;
		}
		ib[idx+0] = idx_value + (u32)(reloc->lobj.gpu_offset & 0xffffffff);
		r = evergreen_cs_track_check(p);
		if (r) {
			dev_warn(p->dev, "%s:%d invalid cmd stream\n", __func__, __LINE__);
			return r;
		}
		break;
	case PACKET3_WAIT_REG_MEM:
		if (pkt->count != 5) {
			DRM_ERROR("bad WAIT_REG_MEM\n");
			return -EINVAL;
		}
		/* bit 4 is reg (0) or mem (1) */
		if (idx_value & 0x10) {
			uint64_t offset;

			r = evergreen_cs_packet_next_reloc(p, &reloc);
			if (r) {
				DRM_ERROR("bad WAIT_REG_MEM\n");
				return -EINVAL;
			}

			offset = reloc->lobj.gpu_offset +
			         (radeon_get_ib_value(p, idx+1) & 0xfffffffc) +
			         ((u64)(radeon_get_ib_value(p, idx+2) & 0xff) << 32);

			ib[idx+1] = (ib[idx+1] & 0x3) | (offset & 0xfffffffc);
			ib[idx+2] = upper_32_bits(offset) & 0xff;
		}
		break;
	case PACKET3_SURFACE_SYNC:
		if (pkt->count != 3) {
			DRM_ERROR("bad SURFACE_SYNC\n");
			return -EINVAL;
		}
		/* 0xffffffff/0x0 is flush all cache flag */
		if (radeon_get_ib_value(p, idx + 1) != 0xffffffff ||
		    radeon_get_ib_value(p, idx + 2) != 0) {
			r = evergreen_cs_packet_next_reloc(p, &reloc);
			if (r) {
				DRM_ERROR("bad SURFACE_SYNC\n");
				return -EINVAL;
			}
			ib[idx+2] += (u32)((reloc->lobj.gpu_offset >> 8) & 0xffffffff);
		}
		break;
	case PACKET3_EVENT_WRITE:
		if (pkt->count != 2 && pkt->count != 0) {
			DRM_ERROR("bad EVENT_WRITE\n");
			return -EINVAL;
		}
		if (pkt->count) {
			uint64_t offset;

			r = evergreen_cs_packet_next_reloc(p, &reloc);
			if (r) {
				DRM_ERROR("bad EVENT_WRITE\n");
				return -EINVAL;
			}
			offset = reloc->lobj.gpu_offset +
			         (radeon_get_ib_value(p, idx+1) & 0xfffffff8) +
			         ((u64)(radeon_get_ib_value(p, idx+2) & 0xff) << 32);

			ib[idx+1] = offset & 0xfffffff8;
			ib[idx+2] = upper_32_bits(offset) & 0xff;
		}
		break;
	case PACKET3_EVENT_WRITE_EOP:
	{
		uint64_t offset;

		if (pkt->count != 4) {
			DRM_ERROR("bad EVENT_WRITE_EOP\n");
			return -EINVAL;
		}
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			DRM_ERROR("bad EVENT_WRITE_EOP\n");
			return -EINVAL;
		}

		offset = reloc->lobj.gpu_offset +
		         (radeon_get_ib_value(p, idx+1) & 0xfffffffc) +
		         ((u64)(radeon_get_ib_value(p, idx+2) & 0xff) << 32);

		ib[idx+1] = offset & 0xfffffffc;
		ib[idx+2] = (ib[idx+2] & 0xffffff00) | (upper_32_bits(offset) & 0xff);
		break;
	}
	case PACKET3_EVENT_WRITE_EOS:
	{
		uint64_t offset;

		if (pkt->count != 3) {
			DRM_ERROR("bad EVENT_WRITE_EOS\n");
			return -EINVAL;
		}
		r = evergreen_cs_packet_next_reloc(p, &reloc);
		if (r) {
			DRM_ERROR("bad EVENT_WRITE_EOS\n");
			return -EINVAL;
		}

		offset = reloc->lobj.gpu_offset +
		         (radeon_get_ib_value(p, idx+1) & 0xfffffffc) +
		         ((u64)(radeon_get_ib_value(p, idx+2) & 0xff) << 32);

		ib[idx+1] = offset & 0xfffffffc;
		ib[idx+2] = (ib[idx+2] & 0xffffff00) | (upper_32_bits(offset) & 0xff);
		break;
	}
	case PACKET3_SET_CONFIG_REG:
		start_reg = (idx_value << 2) + PACKET3_SET_CONFIG_REG_START;
		end_reg = 4 * pkt->count + start_reg - 4;
		if ((start_reg < PACKET3_SET_CONFIG_REG_START) ||
		    (start_reg >= PACKET3_SET_CONFIG_REG_END) ||
		    (end_reg >= PACKET3_SET_CONFIG_REG_END)) {
			DRM_ERROR("bad PACKET3_SET_CONFIG_REG\n");
			return -EINVAL;
		}
		for (i = 0; i < pkt->count; i++) {
			reg = start_reg + (4 * i);
			r = evergreen_cs_check_reg(p, reg, idx+1+i);
			if (r)
				return r;
		}
		break;
	case PACKET3_SET_CONTEXT_REG:
		start_reg = (idx_value << 2) + PACKET3_SET_CONTEXT_REG_START;
		end_reg = 4 * pkt->count + start_reg - 4;
		if ((start_reg < PACKET3_SET_CONTEXT_REG_START) ||
		    (start_reg >= PACKET3_SET_CONTEXT_REG_END) ||
		    (end_reg >= PACKET3_SET_CONTEXT_REG_END)) {
			DRM_ERROR("bad PACKET3_SET_CONTEXT_REG\n");
			return -EINVAL;
		}
		for (i = 0; i < pkt->count; i++) {
			reg = start_reg + (4 * i);
			r = evergreen_cs_check_reg(p, reg, idx+1+i);
			if (r)
				return r;
		}
		break;
	case PACKET3_SET_RESOURCE:
		if (pkt->count % 8) {
			DRM_ERROR("bad SET_RESOURCE\n");
			return -EINVAL;
		}
		start_reg = (idx_value << 2) + PACKET3_SET_RESOURCE_START;
		end_reg = 4 * pkt->count + start_reg - 4;
		if ((start_reg < PACKET3_SET_RESOURCE_START) ||
		    (start_reg >= PACKET3_SET_RESOURCE_END) ||
		    (end_reg >= PACKET3_SET_RESOURCE_END)) {
			DRM_ERROR("bad SET_RESOURCE\n");
			return -EINVAL;
		}
		for (i = 0; i < (pkt->count / 8); i++) {
			struct radeon_bo *texture, *mipmap;
			u32 toffset, moffset;
			u32 size, offset;

			switch (G__SQ_CONSTANT_TYPE(radeon_get_ib_value(p, idx+1+(i*8)+7))) {
			case SQ_TEX_VTX_VALID_TEXTURE:
				/* tex base */
				r = evergreen_cs_packet_next_reloc(p, &reloc);
				if (r) {
					DRM_ERROR("bad SET_RESOURCE (tex)\n");
					return -EINVAL;
				}
				if (!(p->cs_flags & RADEON_CS_KEEP_TILING_FLAGS)) {
					ib[idx+1+(i*8)+1] |=
						TEX_ARRAY_MODE(evergreen_cs_get_aray_mode(reloc->lobj.tiling_flags));
					if (reloc->lobj.tiling_flags & RADEON_TILING_MACRO) {
						unsigned bankw, bankh, mtaspect, tile_split;

						evergreen_tiling_fields(reloc->lobj.tiling_flags,
									&bankw, &bankh, &mtaspect,
									&tile_split);
						ib[idx+1+(i*8)+6] |= TEX_TILE_SPLIT(tile_split);
						ib[idx+1+(i*8)+7] |=
							TEX_BANK_WIDTH(bankw) |
							TEX_BANK_HEIGHT(bankh) |
							MACRO_TILE_ASPECT(mtaspect) |
							TEX_NUM_BANKS(evergreen_cs_get_num_banks(track->nbanks));
					}
				}
				texture = reloc->robj;
				toffset = (u32)((reloc->lobj.gpu_offset >> 8) & 0xffffffff);
				/* tex mip base */
				r = evergreen_cs_packet_next_reloc(p, &reloc);
				if (r) {
					DRM_ERROR("bad SET_RESOURCE (tex)\n");
					return -EINVAL;
				}
				moffset = (u32)((reloc->lobj.gpu_offset >> 8) & 0xffffffff);
				mipmap = reloc->robj;
				r = evergreen_cs_track_validate_texture(p, texture, mipmap, idx+1+(i*8));
				if (r)
					return r;
				ib[idx+1+(i*8)+2] += toffset;
				ib[idx+1+(i*8)+3] += moffset;
				break;
			case SQ_TEX_VTX_VALID_BUFFER:
			{
				uint64_t offset64;
				/* vtx base */
				r = evergreen_cs_packet_next_reloc(p, &reloc);
				if (r) {
					DRM_ERROR("bad SET_RESOURCE (vtx)\n");
					return -EINVAL;
				}
				offset = radeon_get_ib_value(p, idx+1+(i*8)+0);
				size = radeon_get_ib_value(p, idx+1+(i*8)+1);
				if (p->rdev && (size + offset) > radeon_bo_size(reloc->robj)) {
					/* force size to size of the buffer */
					dev_warn(p->dev, "vbo resource seems too big for the bo\n");
					ib[idx+1+(i*8)+1] = radeon_bo_size(reloc->robj) - offset;
				}

				offset64 = reloc->lobj.gpu_offset + offset;
				ib[idx+1+(i*8)+0] = offset64;
				ib[idx+1+(i*8)+2] = (ib[idx+1+(i*8)+2] & 0xffffff00) |
						    (upper_32_bits(offset64) & 0xff);
				break;
			}
			case SQ_TEX_VTX_INVALID_TEXTURE:
			case SQ_TEX_VTX_INVALID_BUFFER:
			default:
				DRM_ERROR("bad SET_RESOURCE\n");
				return -EINVAL;
			}
		}
		break;
	case PACKET3_SET_ALU_CONST:
		/* XXX fix me ALU const buffers only */
		break;
	case PACKET3_SET_BOOL_CONST:
		start_reg = (idx_value << 2) + PACKET3_SET_BOOL_CONST_START;
		end_reg = 4 * pkt->count + start_reg - 4;
		if ((start_reg < PACKET3_SET_BOOL_CONST_START) ||
		    (start_reg >= PACKET3_SET_BOOL_CONST_END) ||
		    (end_reg >= PACKET3_SET_BOOL_CONST_END)) {
			DRM_ERROR("bad SET_BOOL_CONST\n");
			return -EINVAL;
		}
		break;
	case PACKET3_SET_LOOP_CONST:
		start_reg = (idx_value << 2) + PACKET3_SET_LOOP_CONST_START;
		end_reg = 4 * pkt->count + start_reg - 4;
		if ((start_reg < PACKET3_SET_LOOP_CONST_START) ||
		    (start_reg >= PACKET3_SET_LOOP_CONST_END) ||
		    (end_reg >= PACKET3_SET_LOOP_CONST_END)) {
			DRM_ERROR("bad SET_LOOP_CONST\n");
			return -EINVAL;
		}
		break;
	case PACKET3_SET_CTL_CONST:
		start_reg = (idx_value << 2) + PACKET3_SET_CTL_CONST_START;
		end_reg = 4 * pkt->count + start_reg - 4;
		if ((start_reg < PACKET3_SET_CTL_CONST_START) ||
		    (start_reg >= PACKET3_SET_CTL_CONST_END) ||
		    (end_reg >= PACKET3_SET_CTL_CONST_END)) {
			DRM_ERROR("bad SET_CTL_CONST\n");
			return -EINVAL;
		}
		break;
	case PACKET3_SET_SAMPLER:
		if (pkt->count % 3) {
			DRM_ERROR("bad SET_SAMPLER\n");
			return -EINVAL;
		}
		start_reg = (idx_value << 2) + PACKET3_SET_SAMPLER_START;
		end_reg = 4 * pkt->count + start_reg - 4;
		if ((start_reg < PACKET3_SET_SAMPLER_START) ||
		    (start_reg >= PACKET3_SET_SAMPLER_END) ||
		    (end_reg >= PACKET3_SET_SAMPLER_END)) {
			DRM_ERROR("bad SET_SAMPLER\n");
			return -EINVAL;
		}
		break;
	case PACKET3_STRMOUT_BUFFER_UPDATE:
		if (pkt->count != 4) {
			DRM_ERROR("bad STRMOUT_BUFFER_UPDATE (invalid count)\n");
			return -EINVAL;
		}
		/* Updating memory at DST_ADDRESS. */
		if (idx_value & 0x1) {
			u64 offset;
			r = evergreen_cs_packet_next_reloc(p, &reloc);
			if (r) {
				DRM_ERROR("bad STRMOUT_BUFFER_UPDATE (missing dst reloc)\n");
				return -EINVAL;
			}
			offset = radeon_get_ib_value(p, idx+1);
			offset += ((u64)(radeon_get_ib_value(p, idx+2) & 0xff)) << 32;
			if ((offset + 4) > radeon_bo_size(reloc->robj)) {
				DRM_ERROR("bad STRMOUT_BUFFER_UPDATE dst bo too small: 0x%llx, 0x%lx\n",
					  offset + 4, radeon_bo_size(reloc->robj));
				return -EINVAL;
			}
			offset += reloc->lobj.gpu_offset;
			ib[idx+1] = offset;
			ib[idx+2] = upper_32_bits(offset) & 0xff;
		}
		/* Reading data from SRC_ADDRESS. */
		if (((idx_value >> 1) & 0x3) == 2) {
			u64 offset;
			r = evergreen_cs_packet_next_reloc(p, &reloc);
			if (r) {
				DRM_ERROR("bad STRMOUT_BUFFER_UPDATE (missing src reloc)\n");
				return -EINVAL;
			}
			offset = radeon_get_ib_value(p, idx+3);
			offset += ((u64)(radeon_get_ib_value(p, idx+4) & 0xff)) << 32;
			if ((offset + 4) > radeon_bo_size(reloc->robj)) {
				DRM_ERROR("bad STRMOUT_BUFFER_UPDATE src bo too small: 0x%llx, 0x%lx\n",
					  offset + 4, radeon_bo_size(reloc->robj));
				return -EINVAL;
			}
			offset += reloc->lobj.gpu_offset;
			ib[idx+3] = offset;
			ib[idx+4] = upper_32_bits(offset) & 0xff;
		}
		break;
	case PACKET3_COPY_DW:
		if (pkt->count != 4) {
			DRM_ERROR("bad COPY_DW (invalid count)\n");
			return -EINVAL;
		}
		if (idx_value & 0x1) {
			u64 offset;
			/* SRC is memory. */
			r = evergreen_cs_packet_next_reloc(p, &reloc);
			if (r) {
				DRM_ERROR("bad COPY_DW (missing src reloc)\n");
				return -EINVAL;
			}
			offset = radeon_get_ib_value(p, idx+1);
			offset += ((u64)(radeon_get_ib_value(p, idx+2) & 0xff)) << 32;
			if ((offset + 4) > radeon_bo_size(reloc->robj)) {
				DRM_ERROR("bad COPY_DW src bo too small: 0x%llx, 0x%lx\n",
					  offset + 4, radeon_bo_size(reloc->robj));
				return -EINVAL;
			}
			offset += reloc->lobj.gpu_offset;
			ib[idx+1] = offset;
			ib[idx+2] = upper_32_bits(offset) & 0xff;
		} else {
			/* SRC is a reg. */
			reg = radeon_get_ib_value(p, idx+1) << 2;
			if (!evergreen_is_safe_reg(p, reg, idx+1))
				return -EINVAL;
		}
		if (idx_value & 0x2) {
			u64 offset;
			/* DST is memory. */
			r = evergreen_cs_packet_next_reloc(p, &reloc);
			if (r) {
				DRM_ERROR("bad COPY_DW (missing dst reloc)\n");
				return -EINVAL;
			}
			offset = radeon_get_ib_value(p, idx+3);
			offset += ((u64)(radeon_get_ib_value(p, idx+4) & 0xff)) << 32;
			if ((offset + 4) > radeon_bo_size(reloc->robj)) {
				DRM_ERROR("bad COPY_DW dst bo too small: 0x%llx, 0x%lx\n",
					  offset + 4, radeon_bo_size(reloc->robj));
				return -EINVAL;
			}
			offset += reloc->lobj.gpu_offset;
			ib[idx+3] = offset;
			ib[idx+4] = upper_32_bits(offset) & 0xff;
		} else {
			/* DST is a reg. */
			reg = radeon_get_ib_value(p, idx+3) << 2;
			if (!evergreen_is_safe_reg(p, reg, idx+3))
				return -EINVAL;
		}
		break;
	case PACKET3_NOP:
		break;
	default:
		DRM_ERROR("Packet3 opcode %x not supported\n", pkt->opcode);
		return -EINVAL;
	}
	return 0;
}

int evergreen_cs_parse(struct radeon_cs_parser *p)
{
	struct radeon_cs_packet pkt;
	struct evergreen_cs_track *track;
	u32 tmp;
	int r;

	if (p->track == NULL) {
		/* initialize tracker, we are in kms */
		track = kzalloc(sizeof(*track), GFP_KERNEL);
		if (track == NULL)
			return -ENOMEM;
		evergreen_cs_track_init(track);
		if (p->rdev->family >= CHIP_CAYMAN)
			tmp = p->rdev->config.cayman.tile_config;
		else
			tmp = p->rdev->config.evergreen.tile_config;

		switch (tmp & 0xf) {
		case 0:
			track->npipes = 1;
			break;
		case 1:
		default:
			track->npipes = 2;
			break;
		case 2:
			track->npipes = 4;
			break;
		case 3:
			track->npipes = 8;
			break;
		}

		switch ((tmp & 0xf0) >> 4) {
		case 0:
			track->nbanks = 4;
			break;
		case 1:
		default:
			track->nbanks = 8;
			break;
		case 2:
			track->nbanks = 16;
			break;
		}

		switch ((tmp & 0xf00) >> 8) {
		case 0:
			track->group_size = 256;
			break;
		case 1:
		default:
			track->group_size = 512;
			break;
		}

		switch ((tmp & 0xf000) >> 12) {
		case 0:
			track->row_size = 1;
			break;
		case 1:
		default:
			track->row_size = 2;
			break;
		case 2:
			track->row_size = 4;
			break;
		}

		p->track = track;
	}
	do {
		r = evergreen_cs_packet_parse(p, &pkt, p->idx);
		if (r) {
			kfree(p->track);
			p->track = NULL;
			return r;
		}
		p->idx += pkt.count + 2;
		switch (pkt.type) {
		case PACKET_TYPE0:
			r = evergreen_cs_parse_packet0(p, &pkt);
			break;
		case PACKET_TYPE2:
			break;
		case PACKET_TYPE3:
			r = evergreen_packet3_check(p, &pkt);
			break;
		default:
			DRM_ERROR("Unknown packet type %d !\n", pkt.type);
			kfree(p->track);
			p->track = NULL;
			return -EINVAL;
		}
		if (r) {
			kfree(p->track);
			p->track = NULL;
			return r;
		}
	} while (p->idx < p->chunks[p->chunk_ib_idx].length_dw);
#if 0
	for (r = 0; r < p->ib->length_dw; r++) {
		printk(KERN_INFO "%05d  0x%08X\n", r, p->ib->ptr[r]);
		mdelay(1);
	}
#endif
	kfree(p->track);
	p->track = NULL;
	return 0;
}

/* vm parser */
static bool evergreen_vm_reg_valid(u32 reg)
{
	/* context regs are fine */
	if (reg >= 0x28000)
		return true;

	/* check config regs */
	switch (reg) {
	case GRBM_GFX_INDEX:
	case VGT_VTX_VECT_EJECT_REG:
	case VGT_CACHE_INVALIDATION:
	case VGT_GS_VERTEX_REUSE:
	case VGT_PRIMITIVE_TYPE:
	case VGT_INDEX_TYPE:
	case VGT_NUM_INDICES:
	case VGT_NUM_INSTANCES:
	case VGT_COMPUTE_DIM_X:
	case VGT_COMPUTE_DIM_Y:
	case VGT_COMPUTE_DIM_Z:
	case VGT_COMPUTE_START_X:
	case VGT_COMPUTE_START_Y:
	case VGT_COMPUTE_START_Z:
	case VGT_COMPUTE_INDEX:
	case VGT_COMPUTE_THREAD_GROUP_SIZE:
	case VGT_HS_OFFCHIP_PARAM:
	case PA_CL_ENHANCE:
	case PA_SU_LINE_STIPPLE_VALUE:
	case PA_SC_LINE_STIPPLE_STATE:
	case PA_SC_ENHANCE:
	case SQ_DYN_GPR_CNTL_PS_FLUSH_REQ:
	case SQ_DYN_GPR_SIMD_LOCK_EN:
	case SQ_CONFIG:
	case SQ_GPR_RESOURCE_MGMT_1:
	case SQ_GLOBAL_GPR_RESOURCE_MGMT_1:
	case SQ_GLOBAL_GPR_RESOURCE_MGMT_2:
	case SQ_CONST_MEM_BASE:
	case SQ_STATIC_THREAD_MGMT_1:
	case SQ_STATIC_THREAD_MGMT_2:
	case SQ_STATIC_THREAD_MGMT_3:
	case SPI_CONFIG_CNTL:
	case SPI_CONFIG_CNTL_1:
	case TA_CNTL_AUX:
	case DB_DEBUG:
	case DB_DEBUG2:
	case DB_DEBUG3:
	case DB_DEBUG4:
	case DB_WATERMARKS:
	case TD_PS_BORDER_COLOR_INDEX:
	case TD_PS_BORDER_COLOR_RED:
	case TD_PS_BORDER_COLOR_GREEN:
	case TD_PS_BORDER_COLOR_BLUE:
	case TD_PS_BORDER_COLOR_ALPHA:
	case TD_VS_BORDER_COLOR_INDEX:
	case TD_VS_BORDER_COLOR_RED:
	case TD_VS_BORDER_COLOR_GREEN:
	case TD_VS_BORDER_COLOR_BLUE:
	case TD_VS_BORDER_COLOR_ALPHA:
	case TD_GS_BORDER_COLOR_INDEX:
	case TD_GS_BORDER_COLOR_RED:
	case TD_GS_BORDER_COLOR_GREEN:
	case TD_GS_BORDER_COLOR_BLUE:
	case TD_GS_BORDER_COLOR_ALPHA:
	case TD_HS_BORDER_COLOR_INDEX:
	case TD_HS_BORDER_COLOR_RED:
	case TD_HS_BORDER_COLOR_GREEN:
	case TD_HS_BORDER_COLOR_BLUE:
	case TD_HS_BORDER_COLOR_ALPHA:
	case TD_LS_BORDER_COLOR_INDEX:
	case TD_LS_BORDER_COLOR_RED:
	case TD_LS_BORDER_COLOR_GREEN:
	case TD_LS_BORDER_COLOR_BLUE:
	case TD_LS_BORDER_COLOR_ALPHA:
	case TD_CS_BORDER_COLOR_INDEX:
	case TD_CS_BORDER_COLOR_RED:
	case TD_CS_BORDER_COLOR_GREEN:
	case TD_CS_BORDER_COLOR_BLUE:
	case TD_CS_BORDER_COLOR_ALPHA:
	case SQ_ESGS_RING_SIZE:
	case SQ_GSVS_RING_SIZE:
	case SQ_ESTMP_RING_SIZE:
	case SQ_GSTMP_RING_SIZE:
	case SQ_HSTMP_RING_SIZE:
	case SQ_LSTMP_RING_SIZE:
	case SQ_PSTMP_RING_SIZE:
	case SQ_VSTMP_RING_SIZE:
	case SQ_ESGS_RING_ITEMSIZE:
	case SQ_ESTMP_RING_ITEMSIZE:
	case SQ_GSTMP_RING_ITEMSIZE:
	case SQ_GSVS_RING_ITEMSIZE:
	case SQ_GS_VERT_ITEMSIZE:
	case SQ_GS_VERT_ITEMSIZE_1:
	case SQ_GS_VERT_ITEMSIZE_2:
	case SQ_GS_VERT_ITEMSIZE_3:
	case SQ_GSVS_RING_OFFSET_1:
	case SQ_GSVS_RING_OFFSET_2:
	case SQ_GSVS_RING_OFFSET_3:
	case SQ_HSTMP_RING_ITEMSIZE:
	case SQ_LSTMP_RING_ITEMSIZE:
	case SQ_PSTMP_RING_ITEMSIZE:
	case SQ_VSTMP_RING_ITEMSIZE:
	case VGT_TF_RING_SIZE:
	case SQ_ESGS_RING_BASE:
	case SQ_GSVS_RING_BASE:
	case SQ_ESTMP_RING_BASE:
	case SQ_GSTMP_RING_BASE:
	case SQ_HSTMP_RING_BASE:
	case SQ_LSTMP_RING_BASE:
	case SQ_PSTMP_RING_BASE:
	case SQ_VSTMP_RING_BASE:
	case CAYMAN_VGT_OFFCHIP_LDS_BASE:
	case CAYMAN_SQ_EX_ALLOC_TABLE_SLOTS:
		return true;
	default:
		return false;
	}
}

static int evergreen_vm_packet3_check(struct radeon_device *rdev,
				      u32 *ib, struct radeon_cs_packet *pkt)
{
	u32 idx = pkt->idx + 1;
	u32 idx_value = ib[idx];
	u32 start_reg, end_reg, reg, i;

	switch (pkt->opcode) {
	case PACKET3_NOP:
	case PACKET3_SET_BASE:
	case PACKET3_CLEAR_STATE:
	case PACKET3_INDEX_BUFFER_SIZE:
	case PACKET3_DISPATCH_DIRECT:
	case PACKET3_DISPATCH_INDIRECT:
	case PACKET3_MODE_CONTROL:
	case PACKET3_SET_PREDICATION:
	case PACKET3_COND_EXEC:
	case PACKET3_PRED_EXEC:
	case PACKET3_DRAW_INDIRECT:
	case PACKET3_DRAW_INDEX_INDIRECT:
	case PACKET3_INDEX_BASE:
	case PACKET3_DRAW_INDEX_2:
	case PACKET3_CONTEXT_CONTROL:
	case PACKET3_DRAW_INDEX_OFFSET:
	case PACKET3_INDEX_TYPE:
	case PACKET3_DRAW_INDEX:
	case PACKET3_DRAW_INDEX_AUTO:
	case PACKET3_DRAW_INDEX_IMMD:
	case PACKET3_NUM_INSTANCES:
	case PACKET3_DRAW_INDEX_MULTI_AUTO:
	case PACKET3_STRMOUT_BUFFER_UPDATE:
	case PACKET3_DRAW_INDEX_OFFSET_2:
	case PACKET3_DRAW_INDEX_MULTI_ELEMENT:
	case PACKET3_MPEG_INDEX:
	case PACKET3_WAIT_REG_MEM:
	case PACKET3_MEM_WRITE:
	case PACKET3_SURFACE_SYNC:
	case PACKET3_EVENT_WRITE:
	case PACKET3_EVENT_WRITE_EOP:
	case PACKET3_EVENT_WRITE_EOS:
	case PACKET3_SET_CONTEXT_REG:
	case PACKET3_SET_BOOL_CONST:
	case PACKET3_SET_LOOP_CONST:
	case PACKET3_SET_RESOURCE:
	case PACKET3_SET_SAMPLER:
	case PACKET3_SET_CTL_CONST:
	case PACKET3_SET_RESOURCE_OFFSET:
	case PACKET3_SET_CONTEXT_REG_INDIRECT:
	case PACKET3_SET_RESOURCE_INDIRECT:
	case CAYMAN_PACKET3_DEALLOC_STATE:
		break;
	case PACKET3_COND_WRITE:
		if (idx_value & 0x100) {
			reg = ib[idx + 5] * 4;
			if (!evergreen_vm_reg_valid(reg))
				return -EINVAL;
		}
		break;
	case PACKET3_COPY_DW:
		if (idx_value & 0x2) {
			reg = ib[idx + 3] * 4;
			if (!evergreen_vm_reg_valid(reg))
				return -EINVAL;
		}
		break;
	case PACKET3_SET_CONFIG_REG:
		start_reg = (idx_value << 2) + PACKET3_SET_CONFIG_REG_START;
		end_reg = 4 * pkt->count + start_reg - 4;
		if ((start_reg < PACKET3_SET_CONFIG_REG_START) ||
		    (start_reg >= PACKET3_SET_CONFIG_REG_END) ||
		    (end_reg >= PACKET3_SET_CONFIG_REG_END)) {
			DRM_ERROR("bad PACKET3_SET_CONFIG_REG\n");
			return -EINVAL;
		}
		for (i = 0; i < pkt->count; i++) {
			reg = start_reg + (4 * i);
			if (!evergreen_vm_reg_valid(reg))
				return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

int evergreen_ib_parse(struct radeon_device *rdev, struct radeon_ib *ib)
{
	int ret = 0;
	u32 idx = 0;
	struct radeon_cs_packet pkt;

	do {
		pkt.idx = idx;
		pkt.type = CP_PACKET_GET_TYPE(ib->ptr[idx]);
		pkt.count = CP_PACKET_GET_COUNT(ib->ptr[idx]);
		pkt.one_reg_wr = 0;
		switch (pkt.type) {
		case PACKET_TYPE0:
			dev_err(rdev->dev, "Packet0 not allowed!\n");
			ret = -EINVAL;
			break;
		case PACKET_TYPE2:
			idx += 1;
			break;
		case PACKET_TYPE3:
			pkt.opcode = CP_PACKET3_GET_OPCODE(ib->ptr[idx]);
			ret = evergreen_vm_packet3_check(rdev, ib->ptr, &pkt);
			idx += pkt.count + 2;
			break;
		default:
			dev_err(rdev->dev, "Unknown packet type %d !\n", pkt.type);
			ret = -EINVAL;
			break;
		}
		if (ret)
			break;
	} while (idx < ib->length_dw);

	return ret;
}
