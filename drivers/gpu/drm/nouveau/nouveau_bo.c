/*
 * Copyright 2007 Dave Airlied
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * VA LINUX SYSTEMS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */
/*
 * Authors: Dave Airlied <airlied@linux.ie>
 *	    Ben Skeggs   <darktama@iinet.net.au>
 *	    Jeremy Kolb  <jkolb@brandeis.edu>
 */

#include "drmP.h"
#include "ttm/ttm_page_alloc.h"

#include "nouveau_drm.h"
#include "nouveau_drv.h"
#include "nouveau_dma.h"
#include "nouveau_mm.h"
#include "nouveau_vm.h"

#include <linux/log2.h>
#include <linux/slab.h>

static void
nouveau_bo_del_ttm(struct ttm_buffer_object *bo)
{
	struct drm_nouveau_private *dev_priv = nouveau_bdev(bo->bdev);
	struct drm_device *dev = dev_priv->dev;
	struct nouveau_bo *nvbo = nouveau_bo(bo);

	if (unlikely(nvbo->gem))
		DRM_ERROR("bo %p still attached to GEM object\n", bo);

	nv10_mem_put_tile_region(dev, nvbo->tile, NULL);
	kfree(nvbo);
}

static void
nouveau_bo_fixup_align(struct nouveau_bo *nvbo, u32 flags,
		       int *align, int *size)
{
	struct drm_nouveau_private *dev_priv = nouveau_bdev(nvbo->bo.bdev);

	if (dev_priv->card_type < NV_50) {
		if (nvbo->tile_mode) {
			if (dev_priv->chipset >= 0x40) {
				*align = 65536;
				*size = roundup(*size, 64 * nvbo->tile_mode);

			} else if (dev_priv->chipset >= 0x30) {
				*align = 32768;
				*size = roundup(*size, 64 * nvbo->tile_mode);

			} else if (dev_priv->chipset >= 0x20) {
				*align = 16384;
				*size = roundup(*size, 64 * nvbo->tile_mode);

			} else if (dev_priv->chipset >= 0x10) {
				*align = 16384;
				*size = roundup(*size, 32 * nvbo->tile_mode);
			}
		}
	} else {
		*size = roundup(*size, (1 << nvbo->page_shift));
		*align = max((1 <<  nvbo->page_shift), *align);
	}

	*size = roundup(*size, PAGE_SIZE);
}

int
nouveau_bo_new(struct drm_device *dev, int size, int align,
	       uint32_t flags, uint32_t tile_mode, uint32_t tile_flags,
	       struct nouveau_bo **pnvbo)
{
	struct drm_nouveau_private *dev_priv = dev->dev_private;
	struct nouveau_bo *nvbo;
	size_t acc_size;
	int ret;

	nvbo = kzalloc(sizeof(struct nouveau_bo), GFP_KERNEL);
	if (!nvbo)
		return -ENOMEM;
	INIT_LIST_HEAD(&nvbo->head);
	INIT_LIST_HEAD(&nvbo->entry);
	INIT_LIST_HEAD(&nvbo->vma_list);
	nvbo->tile_mode = tile_mode;
	nvbo->tile_flags = tile_flags;
	nvbo->bo.bdev = &dev_priv->ttm.bdev;

	nvbo->page_shift = 12;
	if (dev_priv->bar1_vm) {
		if (!(flags & TTM_PL_FLAG_TT) && size > 256 * 1024)
			nvbo->page_shift = dev_priv->bar1_vm->lpg_shift;
	}

	nouveau_bo_fixup_align(nvbo, flags, &align, &size);
	nvbo->bo.mem.num_pages = size >> PAGE_SHIFT;
	nouveau_bo_placement_set(nvbo, flags, 0);

	acc_size = ttm_bo_dma_acc_size(&dev_priv->ttm.bdev, size,
				       sizeof(struct nouveau_bo));

	ret = ttm_bo_init(&dev_priv->ttm.bdev, &nvbo->bo, size,
			  ttm_bo_type_device, &nvbo->placement,
			  align >> PAGE_SHIFT, 0, false, NULL, acc_size,
			  nouveau_bo_del_ttm);
	if (ret) {
		/* ttm will call nouveau_bo_del_ttm if it fails.. */
		return ret;
	}

	*pnvbo = nvbo;
	return 0;
}

static void
set_placement_list(uint32_t *pl, unsigned *n, uint32_t type, uint32_t flags)
{
	*n = 0;

	if (type & TTM_PL_FLAG_VRAM)
		pl[(*n)++] = TTM_PL_FLAG_VRAM | flags;
	if (type & TTM_PL_FLAG_TT)
		pl[(*n)++] = TTM_PL_FLAG_TT | flags;
	if (type & TTM_PL_FLAG_SYSTEM)
		pl[(*n)++] = TTM_PL_FLAG_SYSTEM | flags;
}

static void
set_placement_range(struct nouveau_bo *nvbo, uint32_t type)
{
	struct drm_nouveau_private *dev_priv = nouveau_bdev(nvbo->bo.bdev);
	int vram_pages = dev_priv->vram_size >> PAGE_SHIFT;

	if (dev_priv->card_type == NV_10 &&
	    nvbo->tile_mode && (type & TTM_PL_FLAG_VRAM) &&
	    nvbo->bo.mem.num_pages < vram_pages / 4) {
		/*
		 * Make sure that the color and depth buffers are handled
		 * by independent memory controller units. Up to a 9x
		 * speed up when alpha-blending and depth-test are enabled
		 * at the same time.
		 */
		if (nvbo->tile_flags & NOUVEAU_GEM_TILE_ZETA) {
			nvbo->placement.fpfn = vram_pages / 2;
			nvbo->placement.lpfn = ~0;
		} else {
			nvbo->placement.fpfn = 0;
			nvbo->placement.lpfn = vram_pages / 2;
		}
	}
}

void
nouveau_bo_placement_set(struct nouveau_bo *nvbo, uint32_t type, uint32_t busy)
{
	struct ttm_placement *pl = &nvbo->placement;
	uint32_t flags = TTM_PL_MASK_CACHING |
		(nvbo->pin_refcnt ? TTM_PL_FLAG_NO_EVICT : 0);

	pl->placement = nvbo->placements;
	set_placement_list(nvbo->placements, &pl->num_placement,
			   type, flags);

	pl->busy_placement = nvbo->busy_placements;
	set_placement_list(nvbo->busy_placements, &pl->num_busy_placement,
			   type | busy, flags);

	set_placement_range(nvbo, type);
}

int
nouveau_bo_pin(struct nouveau_bo *nvbo, uint32_t memtype)
{
	struct drm_nouveau_private *dev_priv = nouveau_bdev(nvbo->bo.bdev);
	struct ttm_buffer_object *bo = &nvbo->bo;
	int ret;

	if (nvbo->pin_refcnt && !(memtype & (1 << bo->mem.mem_type))) {
		NV_ERROR(nouveau_bdev(bo->bdev)->dev,
			 "bo %p pinned elsewhere: 0x%08x vs 0x%08x\n", bo,
			 1 << bo->mem.mem_type, memtype);
		return -EINVAL;
	}

	if (nvbo->pin_refcnt++)
		return 0;

	ret = ttm_bo_reserve(bo, false, false, false, 0);
	if (ret)
		goto out;

	nouveau_bo_placement_set(nvbo, memtype, 0);

	ret = nouveau_bo_validate(nvbo, false, false, false);
	if (ret == 0) {
		switch (bo->mem.mem_type) {
		case TTM_PL_VRAM:
			dev_priv->fb_aper_free -= bo->mem.size;
			break;
		case TTM_PL_TT:
			dev_priv->gart_info.aper_free -= bo->mem.size;
			break;
		default:
			break;
		}
	}
	ttm_bo_unreserve(bo);
out:
	if (unlikely(ret))
		nvbo->pin_refcnt--;
	return ret;
}

int
nouveau_bo_unpin(struct nouveau_bo *nvbo)
{
	struct drm_nouveau_private *dev_priv = nouveau_bdev(nvbo->bo.bdev);
	struct ttm_buffer_object *bo = &nvbo->bo;
	int ret;

	if (--nvbo->pin_refcnt)
		return 0;

	ret = ttm_bo_reserve(bo, false, false, false, 0);
	if (ret)
		return ret;

	nouveau_bo_placement_set(nvbo, bo->mem.placement, 0);

	ret = nouveau_bo_validate(nvbo, false, false, false);
	if (ret == 0) {
		switch (bo->mem.mem_type) {
		case TTM_PL_VRAM:
			dev_priv->fb_aper_free += bo->mem.size;
			break;
		case TTM_PL_TT:
			dev_priv->gart_info.aper_free += bo->mem.size;
			break;
		default:
			break;
		}
	}

	ttm_bo_unreserve(bo);
	return ret;
}

int
nouveau_bo_map(struct nouveau_bo *nvbo)
{
	int ret;

	ret = ttm_bo_reserve(&nvbo->bo, false, false, false, 0);
	if (ret)
		return ret;

	ret = ttm_bo_kmap(&nvbo->bo, 0, nvbo->bo.mem.num_pages, &nvbo->kmap);
	ttm_bo_unreserve(&nvbo->bo);
	return ret;
}

void
nouveau_bo_unmap(struct nouveau_bo *nvbo)
{
	if (nvbo)
		ttm_bo_kunmap(&nvbo->kmap);
}

int
nouveau_bo_validate(struct nouveau_bo *nvbo, bool interruptible,
		    bool no_wait_reserve, bool no_wait_gpu)
{
	int ret;

	ret = ttm_bo_validate(&nvbo->bo, &nvbo->placement, interruptible,
			      no_wait_reserve, no_wait_gpu);
	if (ret)
		return ret;

	return 0;
}

u16
nouveau_bo_rd16(struct nouveau_bo *nvbo, unsigned index)
{
	bool is_iomem;
	u16 *mem = ttm_kmap_obj_virtual(&nvbo->kmap, &is_iomem);
	mem = &mem[index];
	if (is_iomem)
		return ioread16_native((void __force __iomem *)mem);
	else
		return *mem;
}

void
nouveau_bo_wr16(struct nouveau_bo *nvbo, unsigned index, u16 val)
{
	bool is_iomem;
	u16 *mem = ttm_kmap_obj_virtual(&nvbo->kmap, &is_iomem);
	mem = &mem[index];
	if (is_iomem)
		iowrite16_native(val, (void __force __iomem *)mem);
	else
		*mem = val;
}

u32
nouveau_bo_rd32(struct nouveau_bo *nvbo, unsigned index)
{
	bool is_iomem;
	u32 *mem = ttm_kmap_obj_virtual(&nvbo->kmap, &is_iomem);
	mem = &mem[index];
	if (is_iomem)
		return ioread32_native((void __force __iomem *)mem);
	else
		return *mem;
}

void
nouveau_bo_wr32(struct nouveau_bo *nvbo, unsigned index, u32 val)
{
	bool is_iomem;
	u32 *mem = ttm_kmap_obj_virtual(&nvbo->kmap, &is_iomem);
	mem = &mem[index];
	if (is_iomem)
		iowrite32_native(val, (void __force __iomem *)mem);
	else
		*mem = val;
}

static struct ttm_tt *
nouveau_ttm_tt_create(struct ttm_bo_device *bdev,
		      unsigned long size, uint32_t page_flags,
		      struct page *dummy_read_page)
{
	struct drm_nouveau_private *dev_priv = nouveau_bdev(bdev);
	struct drm_device *dev = dev_priv->dev;

	switch (dev_priv->gart_info.type) {
#if __OS_HAS_AGP
	case NOUVEAU_GART_AGP:
		return ttm_agp_tt_create(bdev, dev->agp->bridge,
					 size, page_flags, dummy_read_page);
#endif
	case NOUVEAU_GART_PDMA:
	case NOUVEAU_GART_HW:
		return nouveau_sgdma_create_ttm(bdev, size, page_flags,
						dummy_read_page);
	default:
		NV_ERROR(dev, "Unknown GART type %d\n",
			 dev_priv->gart_info.type);
		break;
	}

	return NULL;
}

static int
nouveau_bo_invalidate_caches(struct ttm_bo_device *bdev, uint32_t flags)
{
	/* We'll do this from user space. */
	return 0;
}

static int
nouveau_bo_init_mem_type(struct ttm_bo_device *bdev, uint32_t type,
			 struct ttm_mem_type_manager *man)
{
	struct drm_nouveau_private *dev_priv = nouveau_bdev(bdev);
	struct drm_device *dev = dev_priv->dev;

	switch (type) {
	case TTM_PL_SYSTEM:
		man->flags = TTM_MEMTYPE_FLAG_MAPPABLE;
		man->available_caching = TTM_PL_MASK_CACHING;
		man->default_caching = TTM_PL_FLAG_CACHED;
		break;
	case TTM_PL_VRAM:
		if (dev_priv->card_type >= NV_50) {
			man->func = &nouveau_vram_manager;
			man->io_reserve_fastpath = false;
			man->use_io_reserve_lru = true;
		} else {
			man->func = &ttm_bo_manager_func;
		}
		man->flags = TTM_MEMTYPE_FLAG_FIXED |
			     TTM_MEMTYPE_FLAG_MAPPABLE;
		man->available_caching = TTM_PL_FLAG_UNCACHED |
					 TTM_PL_FLAG_WC;
		man->default_caching = TTM_PL_FLAG_WC;
		break;
	case TTM_PL_TT:
		if (dev_priv->card_type >= NV_50)
			man->func = &nouveau_gart_manager;
		else
			man->func = &ttm_bo_manager_func;
		switch (dev_priv->gart_info.type) {
		case NOUVEAU_GART_AGP:
			man->flags = TTM_MEMTYPE_FLAG_MAPPABLE;
			man->available_caching = TTM_PL_FLAG_UNCACHED |
				TTM_PL_FLAG_WC;
			man->default_caching = TTM_PL_FLAG_WC;
			break;
		case NOUVEAU_GART_PDMA:
		case NOUVEAU_GART_HW:
			man->flags = TTM_MEMTYPE_FLAG_MAPPABLE |
				     TTM_MEMTYPE_FLAG_CMA;
			man->available_caching = TTM_PL_MASK_CACHING;
			man->default_caching = TTM_PL_FLAG_CACHED;
			break;
		default:
			NV_ERROR(dev, "Unknown GART type: %d\n",
				 dev_priv->gart_info.type);
			return -EINVAL;
		}
		break;
	default:
		NV_ERROR(dev, "Unsupported memory type %u\n", (unsigned)type);
		return -EINVAL;
	}
	return 0;
}

static void
nouveau_bo_evict_flags(struct ttm_buffer_object *bo, struct ttm_placement *pl)
{
	struct nouveau_bo *nvbo = nouveau_bo(bo);

	switch (bo->mem.mem_type) {
	case TTM_PL_VRAM:
		nouveau_bo_placement_set(nvbo, TTM_PL_FLAG_TT,
					 TTM_PL_FLAG_SYSTEM);
		break;
	default:
		nouveau_bo_placement_set(nvbo, TTM_PL_FLAG_SYSTEM, 0);
		break;
	}

	*pl = nvbo->placement;
}


/* GPU-assisted copy using NV_MEMORY_TO_MEMORY_FORMAT, can access
 * TTM_PL_{VRAM,TT} directly.
 */

static int
nouveau_bo_move_accel_cleanup(struct nouveau_channel *chan,
			      struct nouveau_bo *nvbo, bool evict,
			      bool no_wait_reserve, bool no_wait_gpu,
			      struct ttm_mem_reg *new_mem)
{
	struct nouveau_fence *fence = NULL;
	int ret;

	ret = nouveau_fence_new(chan, &fence, true);
	if (ret)
		return ret;

	ret = ttm_bo_move_accel_cleanup(&nvbo->bo, fence, NULL, evict,
					no_wait_reserve, no_wait_gpu, new_mem);
	nouveau_fence_unref(&fence);
	return ret;
}

static int
nvc0_bo_move_m2mf(struct nouveau_channel *chan, struct ttm_buffer_object *bo,
		  struct ttm_mem_reg *old_mem, struct ttm_mem_reg *new_mem)
{
	struct nouveau_mem *node = old_mem->mm_node;
	u64 src_offset = node->vma[0].offset;
	u64 dst_offset = node->vma[1].offset;
	u32 page_count = new_mem->num_pages;
	int ret;

	page_count = new_mem->num_pages;
	while (page_count) {
		int line_count = (page_count > 2047) ? 2047 : page_count;

		ret = RING_SPACE(chan, 12);
		if (ret)
			return ret;

		BEGIN_NVC0(chan, 2, NvSubM2MF, 0x0238, 2);
		OUT_RING  (chan, upper_32_bits(dst_offset));
		OUT_RING  (chan, lower_32_bits(dst_offset));
		BEGIN_NVC0(chan, 2, NvSubM2MF, 0x030c, 6);
		OUT_RING  (chan, upper_32_bits(src_offset));
		OUT_RING  (chan, lower_32_bits(src_offset));
		OUT_RING  (chan, PAGE_SIZE); /* src_pitch */
		OUT_RING  (chan, PAGE_SIZE); /* dst_pitch */
		OUT_RING  (chan, PAGE_SIZE); /* line_length */
		OUT_RING  (chan, line_count);
		BEGIN_NVC0(chan, 2, NvSubM2MF, 0x0300, 1);
		OUT_RING  (chan, 0x00100110);

		page_count -= line_count;
		src_offset += (PAGE_SIZE * line_count);
		dst_offset += (PAGE_SIZE * line_count);
	}

	return 0;
}

static int
nv50_bo_move_m2mf(struct nouveau_channel *chan, struct ttm_buffer_object *bo,
		  struct ttm_mem_reg *old_mem, struct ttm_mem_reg *new_mem)
{
	struct nouveau_mem *node = old_mem->mm_node;
	struct nouveau_bo *nvbo = nouveau_bo(bo);
	u64 length = (new_mem->num_pages << PAGE_SHIFT);
	u64 src_offset = node->vma[0].offset;
	u64 dst_offset = node->vma[1].offset;
	int ret;

	while (length) {
		u32 amount, stride, height;

		amount  = min(length, (u64)(4 * 1024 * 1024));
		stride  = 16 * 4;
		height  = amount / stride;

		if (new_mem->mem_type == TTM_PL_VRAM &&
		    nouveau_bo_tile_layout(nvbo)) {
			ret = RING_SPACE(chan, 8);
			if (ret)
				return ret;

			BEGIN_RING(chan, NvSubM2MF, 0x0200, 7);
			OUT_RING  (chan, 0);
			OUT_RING  (chan, 0);
			OUT_RING  (chan, stride);
			OUT_RING  (chan, height);
			OUT_RING  (chan, 1);
			OUT_RING  (chan, 0);
			OUT_RING  (chan, 0);
		} else {
			ret = RING_SPACE(chan, 2);
			if (ret)
				return ret;

			BEGIN_RING(chan, NvSubM2MF, 0x0200, 1);
			OUT_RING  (chan, 1);
		}
		if (old_mem->mem_type == TTM_PL_VRAM &&
		    nouveau_bo_tile_layout(nvbo)) {
			ret = RING_SPACE(chan, 8);
			if (ret)
				return ret;

			BEGIN_RING(chan, NvSubM2MF, 0x021c, 7);
			OUT_RING  (chan, 0);
			OUT_RING  (chan, 0);
			OUT_RING  (chan, stride);
			OUT_RING  (chan, height);
			OUT_RING  (chan, 1);
			OUT_RING  (chan, 0);
			OUT_RING  (chan, 0);
		} else {
			ret = RING_SPACE(chan, 2);
			if (ret)
				return ret;

			BEGIN_RING(chan, NvSubM2MF, 0x021c, 1);
			OUT_RING  (chan, 1);
		}

		ret = RING_SPACE(chan, 14);
		if (ret)
			return ret;

		BEGIN_RING(chan, NvSubM2MF, 0x0238, 2);
		OUT_RING  (chan, upper_32_bits(src_offset));
		OUT_RING  (chan, upper_32_bits(dst_offset));
		BEGIN_RING(chan, NvSubM2MF, 0x030c, 8);
		OUT_RING  (chan, lower_32_bits(src_offset));
		OUT_RING  (chan, lower_32_bits(dst_offset));
		OUT_RING  (chan, stride);
		OUT_RING  (chan, stride);
		OUT_RING  (chan, stride);
		OUT_RING  (chan, height);
		OUT_RING  (chan, 0x00000101);
		OUT_RING  (chan, 0x00000000);
		BEGIN_RING(chan, NvSubM2MF, NV_MEMORY_TO_MEMORY_FORMAT_NOP, 1);
		OUT_RING  (chan, 0);

		length -= amount;
		src_offset += amount;
		dst_offset += amount;
	}

	return 0;
}

static inline uint32_t
nouveau_bo_mem_ctxdma(struct ttm_buffer_object *bo,
		      struct nouveau_channel *chan, struct ttm_mem_reg *mem)
{
	if (mem->mem_type == TTM_PL_TT)
		return chan->gart_handle;
	return chan->vram_handle;
}

static int
nv04_bo_move_m2mf(struct nouveau_channel *chan, struct ttm_buffer_object *bo,
		  struct ttm_mem_reg *old_mem, struct ttm_mem_reg *new_mem)
{
	u32 src_offset = old_mem->start << PAGE_SHIFT;
	u32 dst_offset = new_mem->start << PAGE_SHIFT;
	u32 page_count = new_mem->num_pages;
	int ret;

	ret = RING_SPACE(chan, 3);
	if (ret)
		return ret;

	BEGIN_RING(chan, NvSubM2MF, NV_MEMORY_TO_MEMORY_FORMAT_DMA_SOURCE, 2);
	OUT_RING  (chan, nouveau_bo_mem_ctxdma(bo, chan, old_mem));
	OUT_RING  (chan, nouveau_bo_mem_ctxdma(bo, chan, new_mem));

	page_count = new_mem->num_pages;
	while (page_count) {
		int line_count = (page_count > 2047) ? 2047 : page_count;

		ret = RING_SPACE(chan, 11);
		if (ret)
			return ret;

		BEGIN_RING(chan, NvSubM2MF,
				 NV_MEMORY_TO_MEMORY_FORMAT_OFFSET_IN, 8);
		OUT_RING  (chan, src_offset);
		OUT_RING  (chan, dst_offset);
		OUT_RING  (chan, PAGE_SIZE); /* src_pitch */
		OUT_RING  (chan, PAGE_SIZE); /* dst_pitch */
		OUT_RING  (chan, PAGE_SIZE); /* line_length */
		OUT_RING  (chan, line_count);
		OUT_RING  (chan, 0x00000101);
		OUT_RING  (chan, 0x00000000);
		BEGIN_RING(chan, NvSubM2MF, NV_MEMORY_TO_MEMORY_FORMAT_NOP, 1);
		OUT_RING  (chan, 0);

		page_count -= line_count;
		src_offset += (PAGE_SIZE * line_count);
		dst_offset += (PAGE_SIZE * line_count);
	}

	return 0;
}

static int
nouveau_vma_getmap(struct nouveau_channel *chan, struct nouveau_bo *nvbo,
		   struct ttm_mem_reg *mem, struct nouveau_vma *vma)
{
	struct nouveau_mem *node = mem->mm_node;
	int ret;

	ret = nouveau_vm_get(chan->vm, mem->num_pages << PAGE_SHIFT,
			     node->page_shift, NV_MEM_ACCESS_RO, vma);
	if (ret)
		return ret;

	if (mem->mem_type == TTM_PL_VRAM)
		nouveau_vm_map(vma, node);
	else
		nouveau_vm_map_sg(vma, 0, mem->num_pages << PAGE_SHIFT, node);

	return 0;
}

static int
nouveau_bo_move_m2mf(struct ttm_buffer_object *bo, int evict, bool intr,
		     bool no_wait_reserve, bool no_wait_gpu,
		     struct ttm_mem_reg *new_mem)
{
	struct drm_nouveau_private *dev_priv = nouveau_bdev(bo->bdev);
	struct nouveau_channel *chan = chan = dev_priv->channel;
	struct nouveau_bo *nvbo = nouveau_bo(bo);
	struct ttm_mem_reg *old_mem = &bo->mem;
	int ret;

	mutex_lock_nested(&chan->mutex, NOUVEAU_KCHANNEL_MUTEX);

	/* create temporary vmas for the transfer and attach them to the
	 * old nouveau_mem node, these will get cleaned up after ttm has
	 * destroyed the ttm_mem_reg
	 */
	if (dev_priv->card_type >= NV_50) {
		struct nouveau_mem *node = old_mem->mm_node;

		ret = nouveau_vma_getmap(chan, nvbo, old_mem, &node->vma[0]);
		if (ret)
			goto out;

		ret = nouveau_vma_getmap(chan, nvbo, new_mem, &node->vma[1]);
		if (ret)
			goto out;
	}

	if (dev_priv->card_type < NV_50)
		ret = nv04_bo_move_m2mf(chan, bo, &bo->mem, new_mem);
	else
	if (dev_priv->card_type < NV_C0)
		ret = nv50_bo_move_m2mf(chan, bo, &bo->mem, new_mem);
	else
		ret = nvc0_bo_move_m2mf(chan, bo, &bo->mem, new_mem);
	if (ret == 0) {
		ret = nouveau_bo_move_accel_cleanup(chan, nvbo, evict,
						    no_wait_reserve,
						    no_wait_gpu, new_mem);
	}

out:
	mutex_unlock(&chan->mutex);
	return ret;
}

static int
nouveau_bo_move_flipd(struct ttm_buffer_object *bo, bool evict, bool intr,
		      bool no_wait_reserve, bool no_wait_gpu,
		      struct ttm_mem_reg *new_mem)
{
	u32 placement_memtype = TTM_PL_FLAG_TT | TTM_PL_MASK_CACHING;
	struct ttm_placement placement;
	struct ttm_mem_reg tmp_mem;
	int ret;

	placement.fpfn = placement.lpfn = 0;
	placement.num_placement = placement.num_busy_placement = 1;
	placement.placement = placement.busy_placement = &placement_memtype;

	tmp_mem = *new_mem;
	tmp_mem.mm_node = NULL;
	ret = ttm_bo_mem_space(bo, &placement, &tmp_mem, intr, no_wait_reserve, no_wait_gpu);
	if (ret)
		return ret;

	ret = ttm_tt_bind(bo->ttm, &tmp_mem);
	if (ret)
		goto out;

	ret = nouveau_bo_move_m2mf(bo, true, intr, no_wait_reserve, no_wait_gpu, &tmp_mem);
	if (ret)
		goto out;

	ret = ttm_bo_move_ttm(bo, true, no_wait_reserve, no_wait_gpu, new_mem);
out:
	ttm_bo_mem_put(bo, &tmp_mem);
	return ret;
}

static int
nouveau_bo_move_flips(struct ttm_buffer_object *bo, bool evict, bool intr,
		      bool no_wait_reserve, bool no_wait_gpu,
		      struct ttm_mem_reg *new_mem)
{
	u32 placement_memtype = TTM_PL_FLAG_TT | TTM_PL_MASK_CACHING;
	struct ttm_placement placement;
	struct ttm_mem_reg tmp_mem;
	int ret;

	placement.fpfn = placement.lpfn = 0;
	placement.num_placement = placement.num_busy_placement = 1;
	placement.placement = placement.busy_placement = &placement_memtype;

	tmp_mem = *new_mem;
	tmp_mem.mm_node = NULL;
	ret = ttm_bo_mem_space(bo, &placement, &tmp_mem, intr, no_wait_reserve, no_wait_gpu);
	if (ret)
		return ret;

	ret = ttm_bo_move_ttm(bo, true, no_wait_reserve, no_wait_gpu, &tmp_mem);
	if (ret)
		goto out;

	ret = nouveau_bo_move_m2mf(bo, true, intr, no_wait_reserve, no_wait_gpu, new_mem);
	if (ret)
		goto out;

out:
	ttm_bo_mem_put(bo, &tmp_mem);
	return ret;
}

static void
nouveau_bo_move_ntfy(struct ttm_buffer_object *bo, struct ttm_mem_reg *new_mem)
{
	struct nouveau_bo *nvbo = nouveau_bo(bo);
	struct nouveau_vma *vma;

	/* ttm can now (stupidly) pass the driver bos it didn't create... */
	if (bo->destroy != nouveau_bo_del_ttm)
		return;

	list_for_each_entry(vma, &nvbo->vma_list, head) {
		if (new_mem && new_mem->mem_type == TTM_PL_VRAM) {
			nouveau_vm_map(vma, new_mem->mm_node);
		} else
		if (new_mem && new_mem->mem_type == TTM_PL_TT &&
		    nvbo->page_shift == vma->vm->spg_shift) {
			nouveau_vm_map_sg(vma, 0, new_mem->
					  num_pages << PAGE_SHIFT,
					  new_mem->mm_node);
		} else {
			nouveau_vm_unmap(vma);
		}
	}
}

static int
nouveau_bo_vm_bind(struct ttm_buffer_object *bo, struct ttm_mem_reg *new_mem,
		   struct nouveau_tile_reg **new_tile)
{
	struct drm_nouveau_private *dev_priv = nouveau_bdev(bo->bdev);
	struct drm_device *dev = dev_priv->dev;
	struct nouveau_bo *nvbo = nouveau_bo(bo);
	u64 offset = new_mem->start << PAGE_SHIFT;

	*new_tile = NULL;
	if (new_mem->mem_type != TTM_PL_VRAM)
		return 0;

	if (dev_priv->card_type >= NV_10) {
		*new_tile = nv10_mem_set_tiling(dev, offset, new_mem->size,
						nvbo->tile_mode,
						nvbo->tile_flags);
	}

	return 0;
}

static void
nouveau_bo_vm_cleanup(struct ttm_buffer_object *bo,
		      struct nouveau_tile_reg *new_tile,
		      struct nouveau_tile_reg **old_tile)
{
	struct drm_nouveau_private *dev_priv = nouveau_bdev(bo->bdev);
	struct drm_device *dev = dev_priv->dev;

	nv10_mem_put_tile_region(dev, *old_tile, bo->sync_obj);
	*old_tile = new_tile;
}

static int
nouveau_bo_move(struct ttm_buffer_object *bo, bool evict, bool intr,
		bool no_wait_reserve, bool no_wait_gpu,
		struct ttm_mem_reg *new_mem)
{
	struct drm_nouveau_private *dev_priv = nouveau_bdev(bo->bdev);
	struct nouveau_bo *nvbo = nouveau_bo(bo);
	struct ttm_mem_reg *old_mem = &bo->mem;
	struct nouveau_tile_reg *new_tile = NULL;
	int ret = 0;

	if (dev_priv->card_type < NV_50) {
		ret = nouveau_bo_vm_bind(bo, new_mem, &new_tile);
		if (ret)
			return ret;
	}

	/* Fake bo copy. */
	if (old_mem->mem_type == TTM_PL_SYSTEM && !bo->ttm) {
		BUG_ON(bo->mem.mm_node != NULL);
		bo->mem = *new_mem;
		new_mem->mm_node = NULL;
		goto out;
	}

	/* Software copy if the card isn't up and running yet. */
	if (!dev_priv->channel) {
		ret = ttm_bo_move_memcpy(bo, evict, no_wait_reserve, no_wait_gpu, new_mem);
		goto out;
	}

	/* Hardware assisted copy. */
	if (new_mem->mem_type == TTM_PL_SYSTEM)
		ret = nouveau_bo_move_flipd(bo, evict, intr, no_wait_reserve, no_wait_gpu, new_mem);
	else if (old_mem->mem_type == TTM_PL_SYSTEM)
		ret = nouveau_bo_move_flips(bo, evict, intr, no_wait_reserve, no_wait_gpu, new_mem);
	else
		ret = nouveau_bo_move_m2mf(bo, evict, intr, no_wait_reserve, no_wait_gpu, new_mem);

	if (!ret)
		goto out;

	/* Fallback to software copy. */
	ret = ttm_bo_move_memcpy(bo, evict, no_wait_reserve, no_wait_gpu, new_mem);

out:
	if (dev_priv->card_type < NV_50) {
		if (ret)
			nouveau_bo_vm_cleanup(bo, NULL, &new_tile);
		else
			nouveau_bo_vm_cleanup(bo, new_tile, &nvbo->tile);
	}

	return ret;
}

static int
nouveau_bo_verify_access(struct ttm_buffer_object *bo, struct file *filp)
{
	return 0;
}

static int
nouveau_ttm_io_mem_reserve(struct ttm_bo_device *bdev, struct ttm_mem_reg *mem)
{
	struct ttm_mem_type_manager *man = &bdev->man[mem->mem_type];
	struct drm_nouveau_private *dev_priv = nouveau_bdev(bdev);
	struct drm_device *dev = dev_priv->dev;
	int ret;

	mem->bus.addr = NULL;
	mem->bus.offset = 0;
	mem->bus.size = mem->num_pages << PAGE_SHIFT;
	mem->bus.base = 0;
	mem->bus.is_iomem = false;
	if (!(man->flags & TTM_MEMTYPE_FLAG_MAPPABLE))
		return -EINVAL;
	switch (mem->mem_type) {
	case TTM_PL_SYSTEM:
		/* System memory */
		return 0;
	case TTM_PL_TT:
#if __OS_HAS_AGP
		if (dev_priv->gart_info.type == NOUVEAU_GART_AGP) {
			mem->bus.offset = mem->start << PAGE_SHIFT;
			mem->bus.base = dev_priv->gart_info.aper_base;
			mem->bus.is_iomem = true;
		}
#endif
		break;
	case TTM_PL_VRAM:
	{
		struct nouveau_mem *node = mem->mm_node;
		u8 page_shift;

		if (!dev_priv->bar1_vm) {
			mem->bus.offset = mem->start << PAGE_SHIFT;
			mem->bus.base = pci_resource_start(dev->pdev, 1);
			mem->bus.is_iomem = true;
			break;
		}

		if (dev_priv->card_type >= NV_C0)
			page_shift = node->page_shift;
		else
			page_shift = 12;

		ret = nouveau_vm_get(dev_priv->bar1_vm, mem->bus.size,
				     page_shift, NV_MEM_ACCESS_RW,
				     &node->bar_vma);
		if (ret)
			return ret;

		nouveau_vm_map(&node->bar_vma, node);
		if (ret) {
			nouveau_vm_put(&node->bar_vma);
			return ret;
		}

		mem->bus.offset = node->bar_vma.offset;
		if (dev_priv->card_type == NV_50) /*XXX*/
			mem->bus.offset -= 0x0020000000ULL;
		mem->bus.base = pci_resource_start(dev->pdev, 1);
		mem->bus.is_iomem = true;
	}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void
nouveau_ttm_io_mem_free(struct ttm_bo_device *bdev, struct ttm_mem_reg *mem)
{
	struct drm_nouveau_private *dev_priv = nouveau_bdev(bdev);
	struct nouveau_mem *node = mem->mm_node;

	if (!dev_priv->bar1_vm || mem->mem_type != TTM_PL_VRAM)
		return;

	if (!node->bar_vma.node)
		return;

	nouveau_vm_unmap(&node->bar_vma);
	nouveau_vm_put(&node->bar_vma);
}

static int
nouveau_ttm_fault_reserve_notify(struct ttm_buffer_object *bo)
{
	struct drm_nouveau_private *dev_priv = nouveau_bdev(bo->bdev);
	struct nouveau_bo *nvbo = nouveau_bo(bo);

	/* as long as the bo isn't in vram, and isn't tiled, we've got
	 * nothing to do here.
	 */
	if (bo->mem.mem_type != TTM_PL_VRAM) {
		if (dev_priv->card_type < NV_50 ||
		    !nouveau_bo_tile_layout(nvbo))
			return 0;
	}

	/* make sure bo is in mappable vram */
	if (bo->mem.start + bo->mem.num_pages < dev_priv->fb_mappable_pages)
		return 0;


	nvbo->placement.fpfn = 0;
	nvbo->placement.lpfn = dev_priv->fb_mappable_pages;
	nouveau_bo_placement_set(nvbo, TTM_PL_VRAM, 0);
	return nouveau_bo_validate(nvbo, false, true, false);
}

void
nouveau_bo_fence(struct nouveau_bo *nvbo, struct nouveau_fence *fence)
{
	struct nouveau_fence *old_fence;

	if (likely(fence))
		nouveau_fence_ref(fence);

	spin_lock(&nvbo->bo.bdev->fence_lock);
	old_fence = nvbo->bo.sync_obj;
	nvbo->bo.sync_obj = fence;
	spin_unlock(&nvbo->bo.bdev->fence_lock);

	nouveau_fence_unref(&old_fence);
}

static int
nouveau_ttm_tt_populate(struct ttm_tt *ttm)
{
	struct ttm_dma_tt *ttm_dma = (void *)ttm;
	struct drm_nouveau_private *dev_priv;
	struct drm_device *dev;
	unsigned i;
	int r;

	if (ttm->state != tt_unpopulated)
		return 0;

	dev_priv = nouveau_bdev(ttm->bdev);
	dev = dev_priv->dev;

#if __OS_HAS_AGP
	if (dev_priv->gart_info.type == NOUVEAU_GART_AGP) {
		return ttm_agp_tt_populate(ttm);
	}
#endif

#ifdef CONFIG_SWIOTLB
	if (swiotlb_nr_tbl()) {
		return ttm_dma_populate((void *)ttm, dev->dev);
	}
#endif

	r = ttm_pool_populate(ttm);
	if (r) {
		return r;
	}

	for (i = 0; i < ttm->num_pages; i++) {
		ttm_dma->dma_address[i] = pci_map_page(dev->pdev, ttm->pages[i],
						   0, PAGE_SIZE,
						   PCI_DMA_BIDIRECTIONAL);
		if (pci_dma_mapping_error(dev->pdev, ttm_dma->dma_address[i])) {
			while (--i) {
				pci_unmap_page(dev->pdev, ttm_dma->dma_address[i],
					       PAGE_SIZE, PCI_DMA_BIDIRECTIONAL);
				ttm_dma->dma_address[i] = 0;
			}
			ttm_pool_unpopulate(ttm);
			return -EFAULT;
		}
	}
	return 0;
}

static void
nouveau_ttm_tt_unpopulate(struct ttm_tt *ttm)
{
	struct ttm_dma_tt *ttm_dma = (void *)ttm;
	struct drm_nouveau_private *dev_priv;
	struct drm_device *dev;
	unsigned i;

	dev_priv = nouveau_bdev(ttm->bdev);
	dev = dev_priv->dev;

#if __OS_HAS_AGP
	if (dev_priv->gart_info.type == NOUVEAU_GART_AGP) {
		ttm_agp_tt_unpopulate(ttm);
		return;
	}
#endif

#ifdef CONFIG_SWIOTLB
	if (swiotlb_nr_tbl()) {
		ttm_dma_unpopulate((void *)ttm, dev->dev);
		return;
	}
#endif

	for (i = 0; i < ttm->num_pages; i++) {
		if (ttm_dma->dma_address[i]) {
			pci_unmap_page(dev->pdev, ttm_dma->dma_address[i],
				       PAGE_SIZE, PCI_DMA_BIDIRECTIONAL);
		}
	}

	ttm_pool_unpopulate(ttm);
}

struct ttm_bo_driver nouveau_bo_driver = {
	.ttm_tt_create = &nouveau_ttm_tt_create,
	.ttm_tt_populate = &nouveau_ttm_tt_populate,
	.ttm_tt_unpopulate = &nouveau_ttm_tt_unpopulate,
	.invalidate_caches = nouveau_bo_invalidate_caches,
	.init_mem_type = nouveau_bo_init_mem_type,
	.evict_flags = nouveau_bo_evict_flags,
	.move_notify = nouveau_bo_move_ntfy,
	.move = nouveau_bo_move,
	.verify_access = nouveau_bo_verify_access,
	.sync_obj_signaled = __nouveau_fence_signalled,
	.sync_obj_wait = __nouveau_fence_wait,
	.sync_obj_flush = __nouveau_fence_flush,
	.sync_obj_unref = __nouveau_fence_unref,
	.sync_obj_ref = __nouveau_fence_ref,
	.fault_reserve_notify = &nouveau_ttm_fault_reserve_notify,
	.io_mem_reserve = &nouveau_ttm_io_mem_reserve,
	.io_mem_free = &nouveau_ttm_io_mem_free,
};

struct nouveau_vma *
nouveau_bo_vma_find(struct nouveau_bo *nvbo, struct nouveau_vm *vm)
{
	struct nouveau_vma *vma;
	list_for_each_entry(vma, &nvbo->vma_list, head) {
		if (vma->vm == vm)
			return vma;
	}

	return NULL;
}

int
nouveau_bo_vma_add(struct nouveau_bo *nvbo, struct nouveau_vm *vm,
		   struct nouveau_vma *vma)
{
	const u32 size = nvbo->bo.mem.num_pages << PAGE_SHIFT;
	struct nouveau_mem *node = nvbo->bo.mem.mm_node;
	int ret;

	ret = nouveau_vm_get(vm, size, nvbo->page_shift,
			     NV_MEM_ACCESS_RW, vma);
	if (ret)
		return ret;

	if (nvbo->bo.mem.mem_type == TTM_PL_VRAM)
		nouveau_vm_map(vma, nvbo->bo.mem.mm_node);
	else
	if (nvbo->bo.mem.mem_type == TTM_PL_TT)
		nouveau_vm_map_sg(vma, 0, size, node);

	list_add_tail(&vma->head, &nvbo->vma_list);
	vma->refcount = 1;
	return 0;
}

void
nouveau_bo_vma_del(struct nouveau_bo *nvbo, struct nouveau_vma *vma)
{
	if (vma->node) {
		if (nvbo->bo.mem.mem_type != TTM_PL_SYSTEM) {
			spin_lock(&nvbo->bo.bdev->fence_lock);
			ttm_bo_wait(&nvbo->bo, false, false, false);
			spin_unlock(&nvbo->bo.bdev->fence_lock);
			nouveau_vm_unmap(vma);
		}

		nouveau_vm_put(vma);
		list_del(&vma->head);
	}
}
