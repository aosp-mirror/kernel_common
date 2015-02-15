/* Copyright (c) 2002,2007-2014, The Linux Foundation. All rights reserved.
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
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/ioctl.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/of_coresight.h>
#include <linux/input.h>

#include <mach/socinfo.h>
#include <mach/msm_bus_board.h>
#include <mach/msm_bus.h>

#include "kgsl.h"
#include "kgsl_pwrscale.h"
#include "kgsl_cffdump.h"
#include "kgsl_sharedmem.h"
#include "kgsl_iommu.h"

#include "adreno.h"
#include "adreno_pm4types.h"
#include "adreno_trace.h"

#include "a2xx_reg.h"
#include "a3xx_reg.h"

#define DRIVER_VERSION_MAJOR   3
#define DRIVER_VERSION_MINOR   1

#define NUM_TIMES_RESET_RETRY 5

#define ADRENO_CFG_MHARB \
	(0x10 \
		| (0 << MH_ARBITER_CONFIG__SAME_PAGE_GRANULARITY__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__L1_ARB_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__L1_ARB_HOLD_ENABLE__SHIFT) \
		| (0 << MH_ARBITER_CONFIG__L2_ARB_CONTROL__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__PAGE_SIZE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__TC_REORDER_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__TC_ARB_HOLD_ENABLE__SHIFT) \
		| (0 << MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT_ENABLE__SHIFT) \
		| (0x8 << MH_ARBITER_CONFIG__IN_FLIGHT_LIMIT__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__CP_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__VGT_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__TC_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__RB_CLNT_ENABLE__SHIFT) \
		| (1 << MH_ARBITER_CONFIG__PA_CLNT_ENABLE__SHIFT))

#define ADRENO_MMU_CONFIG						\
	(0x01								\
	 | (MMU_CONFIG << MH_MMU_CONFIG__RB_W_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__CP_W_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__CP_R0_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__CP_R1_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__CP_R2_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__CP_R3_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__CP_R4_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__VGT_R0_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__VGT_R1_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__TC_R_CLNT_BEHAVIOR__SHIFT)	\
	 | (MMU_CONFIG << MH_MMU_CONFIG__PA_W_CLNT_BEHAVIOR__SHIFT))

#define KGSL_LOG_LEVEL_DEFAULT 3

static void adreno_start_work(struct work_struct *work);
static void adreno_input_work(struct work_struct *work);

static struct devfreq_simple_ondemand_data adreno_ondemand_data = {
	.upthreshold = 80,
	.downdifferential = 20,
};

static struct devfreq_msm_adreno_tz_data adreno_tz_data = {
	.bus = {
		.max = 450,
	},
	.device_id = KGSL_DEVICE_3D0,
};

static const struct devfreq_governor_data adreno_governors[] = {
	{ .name = "simple_ondemand", .data = &adreno_ondemand_data },
	{ .name = "msm-adreno-tz", .data = &adreno_tz_data },
};

static const struct kgsl_functable adreno_functable;

static struct adreno_device device_3d0 = {
	.dev = {
		KGSL_DEVICE_COMMON_INIT(device_3d0.dev),
		.pwrscale = KGSL_PWRSCALE_INIT(adreno_governors,
					ARRAY_SIZE(adreno_governors)),
		.name = DEVICE_3D0_NAME,
		.id = KGSL_DEVICE_3D0,
		.mh = {
			.mharb  = ADRENO_CFG_MHARB,
			.mh_intf_cfg1 = 0x00032f07,
			.mpu_base = 0x00000000,
			.mpu_range =  0xFFFFF000,
		},
		.mmu = {
			.config = ADRENO_MMU_CONFIG,
		},
		.pwrctrl = {
			.irq_name = KGSL_3D0_IRQ,
		},
		.iomemname = KGSL_3D0_REG_MEMORY,
		.shadermemname = KGSL_3D0_SHADER_MEMORY,
		.ftbl = &adreno_functable,
		.cmd_log = KGSL_LOG_LEVEL_DEFAULT,
		.ctxt_log = KGSL_LOG_LEVEL_DEFAULT,
		.drv_log = KGSL_LOG_LEVEL_DEFAULT,
		.mem_log = KGSL_LOG_LEVEL_DEFAULT,
		.pwr_log = KGSL_LOG_LEVEL_DEFAULT,
		.pm_dump_enable = 0,
	},
	.gmem_base = 0,
	.gmem_size = SZ_256K,
	.pfp_fw = NULL,
	.pm4_fw = NULL,
	.wait_timeout = 0, 
	.ib_check_level = 0,
	.ft_policy = KGSL_FT_DEFAULT_POLICY,
	.ft_pf_policy = KGSL_FT_PAGEFAULT_DEFAULT_POLICY,
	.fast_hang_detect = 1,
	.long_ib_detect = 1,
	.start_work = __WORK_INITIALIZER(device_3d0.start_work,
		adreno_start_work),
	.input_work = __WORK_INITIALIZER(device_3d0.input_work,
		adreno_input_work),
};

unsigned int ft_detect_regs[FT_DETECT_REGS_COUNT];

static struct workqueue_struct *adreno_wq;


#define ANY_ID (~0)
#define NO_VER (~0)

static const struct {
	enum adreno_gpurev gpurev;
	unsigned int core, major, minor, patchid;
	const char *pm4fw;
	const char *pfpfw;
	struct adreno_gpudev *gpudev;
	unsigned int istore_size;
	unsigned int pix_shader_start;
	
	unsigned int instruction_size;
	
	unsigned int gmem_size;
	unsigned int sync_lock_pm4_ver;
	unsigned int sync_lock_pfp_ver;
	
	unsigned int pm4_jt_idx;
	
	unsigned int pm4_jt_addr;
	
	unsigned int pfp_jt_idx;
	
	unsigned int pfp_jt_addr;
	
	unsigned int pm4_bstrp_size;
	
	unsigned int pfp_bstrp_size;
	
	unsigned int pfp_bstrp_ver;

} adreno_gpulist[] = {
	{ ADRENO_REV_A200, 0, 2, ANY_ID, ANY_ID,
		"yamato_pm4.fw", "yamato_pfp.fw", &adreno_a2xx_gpudev,
		512, 384, 3, SZ_256K, NO_VER, NO_VER },
	{ ADRENO_REV_A203, 0, 1, 1, ANY_ID,
		"yamato_pm4.fw", "yamato_pfp.fw", &adreno_a2xx_gpudev,
		512, 384, 3, SZ_256K, NO_VER, NO_VER },
	{ ADRENO_REV_A205, 0, 1, 0, ANY_ID,
		"yamato_pm4.fw", "yamato_pfp.fw", &adreno_a2xx_gpudev,
		512, 384, 3, SZ_256K, NO_VER, NO_VER },
	{ ADRENO_REV_A220, 2, 1, ANY_ID, ANY_ID,
		"leia_pm4_470.fw", "leia_pfp_470.fw", &adreno_a2xx_gpudev,
		512, 384, 3, SZ_512K, NO_VER, NO_VER },
	{ ADRENO_REV_A225, 2, 2, 0, 5,
		"a225p5_pm4.fw", "a225_pfp.fw", &adreno_a2xx_gpudev,
		1536, 768, 3, SZ_512K, NO_VER, NO_VER },
	{ ADRENO_REV_A225, 2, 2, 0, 6,
		"a225_pm4.fw", "a225_pfp.fw", &adreno_a2xx_gpudev,
		1536, 768, 3, SZ_512K, 0x225011, 0x225002 },
	{ ADRENO_REV_A225, 2, 2, ANY_ID, ANY_ID,
		"a225_pm4.fw", "a225_pfp.fw", &adreno_a2xx_gpudev,
		1536, 768, 3, SZ_512K, 0x225011, 0x225002 },
	
	{ ADRENO_REV_A305, 3, 0, 5, 0,
		"a300_pm4.fw", "a300_pfp.fw", &adreno_a3xx_gpudev,
		512, 0, 2, SZ_256K, 0x3FF037, 0x3FF016 },
	
	{ ADRENO_REV_A320, 3, 2, ANY_ID, ANY_ID,
		"a300_pm4.fw", "a300_pfp.fw", &adreno_a3xx_gpudev,
		512, 0, 2, SZ_512K, 0x3FF037, 0x3FF016 },
	{ ADRENO_REV_A330, 3, 3, 0, ANY_ID,
		"a330_pm4.fw", "a330_pfp.fw", &adreno_a3xx_gpudev,
		512, 0, 2, SZ_1M, NO_VER, NO_VER, 0x8AD, 0x2E4, 0x201, 0x200,
		0x6, 0x20, 0x330020 },
	{ ADRENO_REV_A305B, 3, 0, 5, 0x10,
		"a330_pm4.fw", "a330_pfp.fw", &adreno_a3xx_gpudev,
		512, 0, 2, SZ_128K, NO_VER, NO_VER, 0x8AD, 0x2E4,
		0x201, 0x200 },
	
	{ ADRENO_REV_A305B, 3, 0, 5, 0x12,
		"a330_pm4.fw", "a330_pfp.fw", &adreno_a3xx_gpudev,
		512, 0, 2, SZ_128K, NO_VER, NO_VER, 0x8AD, 0x2E4,
		0x201, 0x200 },
	{ ADRENO_REV_A305C, 3, 0, 5, 0x20,
		"a300_pm4.fw", "a300_pfp.fw", &adreno_a3xx_gpudev,
		512, 0, 2, SZ_128K, 0x3FF037, 0x3FF016 },
};

static unsigned int _wake_nice = -7;

static unsigned int _wake_timeout = 100;

static void adreno_input_work(struct work_struct *work)
{
	struct adreno_device *adreno_dev = container_of(work,
			struct adreno_device, input_work);
	struct kgsl_device *device = &adreno_dev->dev;

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);

	device->flags |= KGSL_FLAG_WAKE_ON_TOUCH;

	kgsl_pwrctrl_wake(device, 0);

	mod_timer(&device->idle_timer,
		jiffies + msecs_to_jiffies(_wake_timeout));
	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
}

static void adreno_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	struct kgsl_device *device = handle->handler->private;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	if ((type == EV_ABS) &&
		!(device->flags & KGSL_FLAG_WAKE_ON_TOUCH) &&
		(device->state == KGSL_STATE_SLUMBER))
		schedule_work(&adreno_dev->input_work);
}

static int adreno_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (handle == NULL)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = handler->name;

	ret = input_register_handle(handle);
	if (ret) {
		kfree(handle);
		return ret;
	}

	ret = input_open_device(handle);
	if (ret) {
		input_unregister_handle(handle);
		kfree(handle);
	}

	return ret;
}

static void adreno_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id adreno_input_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
				BIT_MASK(ABS_MT_POSITION_X) |
				BIT_MASK(ABS_MT_POSITION_Y) },
	},
	{ },
};

static struct input_handler adreno_input_handler = {
	.event = adreno_input_event,
	.connect = adreno_input_connect,
	.disconnect = adreno_input_disconnect,
	.name = "kgsl",
	.id_table = adreno_input_ids,
};


static int adreno_perfcounter_init(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	if (adreno_dev->gpudev->perfcounter_init)
		return adreno_dev->gpudev->perfcounter_init(adreno_dev);
	return 0;
};

static void adreno_perfcounter_close(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	if (adreno_dev->gpudev->perfcounter_close)
		return adreno_dev->gpudev->perfcounter_close(adreno_dev);
}


static int adreno_perfcounter_start(struct adreno_device *adreno_dev)
{
	struct adreno_perfcounters *counters = adreno_dev->gpudev->perfcounters;
	struct adreno_perfcount_group *group;
	unsigned int i, j;
	int ret = 0;

	if (NULL == counters)
		return 0;

	
	for (i = 0; i < counters->group_count; i++) {
		group = &(counters->groups[i]);

		
		for (j = 0; j < group->reg_count; j++) {
			if (group->regs[j].countable ==
					KGSL_PERFCOUNTER_NOT_USED ||
					group->regs[j].countable ==
					KGSL_PERFCOUNTER_BROKEN)
				continue;

			if (adreno_dev->gpudev->perfcounter_enable)
				ret = adreno_dev->gpudev->perfcounter_enable(
					adreno_dev, i, j,
					group->regs[j].countable);
				if (ret)
					goto done;
		}
	}
done:
	return ret;
}


int adreno_perfcounter_read_group(struct adreno_device *adreno_dev,
	struct kgsl_perfcounter_read_group __user *reads, unsigned int count)
{
	struct adreno_perfcounters *counters = adreno_dev->gpudev->perfcounters;
	struct kgsl_device *device = &adreno_dev->dev;
	struct adreno_perfcount_group *group;
	struct kgsl_perfcounter_read_group *list = NULL;
	unsigned int i, j;
	int ret = 0;

	if (NULL == counters)
		return -EINVAL;

	
	if (!adreno_dev->gpudev->perfcounter_read)
		return -EINVAL;

	
	if (reads == NULL || count == 0 || count > 100)
		return -EINVAL;

	list = kmalloc(sizeof(struct kgsl_perfcounter_read_group) * count,
			GFP_KERNEL);
	if (!list)
		return -ENOMEM;

	if (copy_from_user(list, reads,
			sizeof(struct kgsl_perfcounter_read_group) * count)) {
		ret = -EFAULT;
		goto done;
	}

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
	ret = kgsl_active_count_get(device);
	if (ret) {
		kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
		goto done;
	}

	
	for (j = 0; j < count; j++) {

		list[j].value = 0;

		
		if (list[j].groupid >= counters->group_count) {
			ret = -EINVAL;
			break;
		}

		group = &(counters->groups[list[j].groupid]);

		
		for (i = 0; i < group->reg_count; i++) {
			if (group->regs[i].countable == list[j].countable) {
				list[j].value =
					adreno_dev->gpudev->perfcounter_read(
					adreno_dev, list[j].groupid, i);
				break;
			}
		}
	}

	kgsl_active_count_put(device);
	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);

	
	if (ret == 0)
		ret = copy_to_user(reads, list,
			sizeof(struct kgsl_perfcounter_read_group) * count);

done:
	kfree(list);
	return ret;
}


int adreno_perfcounter_get_groupid(struct adreno_device *adreno_dev,
					const char *name)
{

	struct adreno_perfcounters *counters = adreno_dev->gpudev->perfcounters;
	struct adreno_perfcount_group *group;
	int i;

	if (name == NULL)
		return -EINVAL;

	if (NULL == counters)
		return -EINVAL;

	for (i = 0; i < counters->group_count; ++i) {
		group = &(counters->groups[i]);
		if (!strcmp(group->name, name))
			return i;
	}

	return -EINVAL;
}


const char *adreno_perfcounter_get_name(struct adreno_device *adreno_dev,
		unsigned int groupid)
{
	struct adreno_perfcounters *counters = adreno_dev->gpudev->perfcounters;

	if (NULL == counters)
		return NULL;

	if (groupid >= counters->group_count)
		return NULL;

	return counters->groups[groupid].name;
}


int adreno_perfcounter_query_group(struct adreno_device *adreno_dev,
	unsigned int groupid, unsigned int *countables, unsigned int count,
	unsigned int *max_counters)
{
	struct adreno_perfcounters *counters = adreno_dev->gpudev->perfcounters;
	struct kgsl_device *device = &adreno_dev->dev;
	struct adreno_perfcount_group *group;
	unsigned int i, t;
	int ret;
	unsigned int *buf;

	*max_counters = 0;

	if (NULL == counters)
		return -EINVAL;

	if (groupid >= counters->group_count)
		return -EINVAL;

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);

	group = &(counters->groups[groupid]);
	*max_counters = group->reg_count;

	if (countables == NULL || count == 0) {
		kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
		return 0;
	}

	t = min_t(int, group->reg_count, count);

	buf = kmalloc(t * sizeof(unsigned int), GFP_KERNEL);
	if (buf == NULL) {
		kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
		return -ENOMEM;
	}

	for (i = 0; i < t; i++)
		buf[i] = group->regs[i].countable;

	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);

	ret = copy_to_user(countables, buf, sizeof(unsigned int) * t);
	kfree(buf);

	return ret;
}

static inline void refcount_group(struct adreno_perfcount_group *group,
	unsigned int reg, unsigned int flags,
	unsigned int *lo, unsigned int *hi)
{
	if (flags & PERFCOUNTER_FLAG_KERNEL)
		group->regs[reg].kernelcount++;
	else
		group->regs[reg].usercount++;

	if (lo)
		*lo = group->regs[reg].offset;

	if (hi)
		*hi = group->regs[reg].offset_hi;
}


int adreno_perfcounter_get(struct adreno_device *adreno_dev,
	unsigned int groupid, unsigned int countable, unsigned int *offset,
	unsigned int *offset_hi, unsigned int flags)
{
	struct adreno_perfcounters *counters = adreno_dev->gpudev->perfcounters;
	struct adreno_perfcount_group *group;
	unsigned int empty = -1;
	int ret = 0;

	
	if (offset)
		*offset = 0;
	if (offset_hi)
		*offset_hi = 0;

	if (NULL == counters)
		return -EINVAL;

	if (groupid >= counters->group_count)
		return -EINVAL;

	group = &(counters->groups[groupid]);

	if (group->flags & ADRENO_PERFCOUNTER_GROUP_FIXED) {

		if (countable >= group->reg_count)
			return -EINVAL;

		
		if ((group->regs[countable].kernelcount != 0) ||
			(group->regs[countable].usercount != 0)) {
				refcount_group(group, countable, flags,
					offset, offset_hi);
				return 0;
		}

		empty = countable;
	} else {
		unsigned int i;


		for (i = 0; i < group->reg_count; i++) {
			if (group->regs[i].countable == countable) {
				refcount_group(group, i, flags,
					offset, offset_hi);
				return 0;
			} else if (group->regs[i].countable ==
			KGSL_PERFCOUNTER_NOT_USED) {
				
				empty = i;
			}
		}
	}

	
	if (empty == -1)
		return -EBUSY;

	
	ret = adreno_dev->gpudev->perfcounter_enable(adreno_dev, groupid, empty,
		countable);
	if (ret)
		return ret;
	
	group->regs[empty].countable = countable;

	
	if (flags & PERFCOUNTER_FLAG_KERNEL) {
		group->regs[empty].kernelcount = 1;
		group->regs[empty].usercount = 0;
	} else {
		group->regs[empty].kernelcount = 0;
		group->regs[empty].usercount = 1;
	}

	if (offset)
		*offset = group->regs[empty].offset;
	if (offset_hi)
		*offset_hi = group->regs[empty].offset_hi;

	return ret;
}


int adreno_perfcounter_put(struct adreno_device *adreno_dev,
	unsigned int groupid, unsigned int countable, unsigned int flags)
{
	struct adreno_perfcounters *counters = adreno_dev->gpudev->perfcounters;
	struct adreno_perfcount_group *group;

	unsigned int i;

	if (NULL == counters)
		return -EINVAL;

	if (groupid >= counters->group_count)
		return -EINVAL;

	group = &(counters->groups[groupid]);

	for (i = 0; i < group->reg_count; i++) {
		
		if (group->regs[i].countable == countable) {
			
			if (flags & PERFCOUNTER_FLAG_KERNEL &&
					group->regs[i].kernelcount > 0)
				group->regs[i].kernelcount--;
			else if (group->regs[i].usercount > 0)
				group->regs[i].usercount--;
			else
				break;

			
			if (group->regs[i].kernelcount == 0 &&
					group->regs[i].usercount == 0)
				group->regs[i].countable =
					KGSL_PERFCOUNTER_NOT_USED;

			return 0;
		}
	}

	return -EINVAL;
}

static inline void adreno_perfcounter_restore(struct adreno_device *adreno_dev)
{
	if (adreno_dev->gpudev->perfcounter_restore)
		adreno_dev->gpudev->perfcounter_restore(adreno_dev);
}

static inline void adreno_perfcounter_save(struct adreno_device *adreno_dev)
{
	if (adreno_dev->gpudev->perfcounter_save)
		adreno_dev->gpudev->perfcounter_save(adreno_dev);
}

static irqreturn_t adreno_irq_handler(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	return adreno_dev->gpudev->irq_handler(adreno_dev);
}

static void adreno_cleanup_pt(struct kgsl_device *device,
			struct kgsl_pagetable *pagetable)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_ringbuffer *rb = &adreno_dev->ringbuffer;

	kgsl_mmu_unmap(pagetable, &rb->buffer_desc);

	kgsl_mmu_unmap(pagetable, &device->memstore);

	kgsl_mmu_unmap(pagetable, &adreno_dev->pwron_fixup);

	kgsl_mmu_unmap(pagetable, &device->mmu.setstate_memory);

	kgsl_mmu_unmap(pagetable, &adreno_dev->profile.shared_buffer);
}

static int adreno_setup_pt(struct kgsl_device *device,
			struct kgsl_pagetable *pagetable)
{
	int result;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_ringbuffer *rb = &adreno_dev->ringbuffer;

	result = kgsl_mmu_map_global(pagetable, &rb->buffer_desc);


	if (!result)
		result = kgsl_mmu_map_global(pagetable, &device->memstore);

	if (!result)
		result = kgsl_mmu_map_global(pagetable,
			&adreno_dev->pwron_fixup);

	if (!result)
		result = kgsl_mmu_map_global(pagetable,
			&device->mmu.setstate_memory);

	if (!result)
		result = kgsl_mmu_map_global(pagetable,
			&adreno_dev->profile.shared_buffer);

	if (result) {
		
		adreno_cleanup_pt(device, pagetable);
		return result;
	}

	device->mh.mpu_range = adreno_dev->profile.shared_buffer.gpuaddr +
				adreno_dev->profile.shared_buffer.size;

	return 0;
}

static unsigned int _adreno_iommu_setstate_v0(struct kgsl_device *device,
					unsigned int *cmds_orig,
					phys_addr_t pt_val,
					int num_iommu_units, uint32_t flags)
{
	phys_addr_t reg_pt_val;
	unsigned int *cmds = cmds_orig;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int i;

	if (cpu_is_msm8960())
		cmds += adreno_add_change_mh_phys_limit_cmds(cmds, 0xFFFFF000,
					device->mmu.setstate_memory.gpuaddr +
					KGSL_IOMMU_SETSTATE_NOP_OFFSET);
	else
		cmds += adreno_add_bank_change_cmds(cmds,
					KGSL_IOMMU_CONTEXT_USER,
					device->mmu.setstate_memory.gpuaddr +
					KGSL_IOMMU_SETSTATE_NOP_OFFSET);

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	
	cmds += kgsl_mmu_sync_lock(&device->mmu, cmds);

	if (flags & KGSL_MMUFLAGS_PTUPDATE) {
		for (i = 0; i < num_iommu_units; i++) {
			reg_pt_val = kgsl_mmu_get_default_ttbr0(&device->mmu,
						i, KGSL_IOMMU_CONTEXT_USER);
			reg_pt_val &= ~KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
			reg_pt_val |= (pt_val & KGSL_IOMMU_CTX_TTBR0_ADDR_MASK);
			*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
			*cmds++ = kgsl_mmu_get_reg_gpuaddr(&device->mmu, i,
				KGSL_IOMMU_CONTEXT_USER, KGSL_IOMMU_CTX_TTBR0);
			*cmds++ = reg_pt_val;
			*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
			*cmds++ = 0x00000000;

			cmds += adreno_add_read_cmds(device, cmds,
				kgsl_mmu_get_reg_gpuaddr(&device->mmu, i,
				KGSL_IOMMU_CONTEXT_USER, KGSL_IOMMU_CTX_TTBR0),
				reg_pt_val,
				device->mmu.setstate_memory.gpuaddr +
				KGSL_IOMMU_SETSTATE_NOP_OFFSET);
		}
	}
	if (flags & KGSL_MMUFLAGS_TLBFLUSH) {
		for (i = 0; i < num_iommu_units; i++) {
			reg_pt_val = (pt_val + kgsl_mmu_get_default_ttbr0(
						&device->mmu,
						i, KGSL_IOMMU_CONTEXT_USER));
			reg_pt_val &= ~KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
			reg_pt_val |= (pt_val & KGSL_IOMMU_CTX_TTBR0_ADDR_MASK);

			*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
			*cmds++ = kgsl_mmu_get_reg_gpuaddr(&device->mmu, i,
				KGSL_IOMMU_CONTEXT_USER,
				KGSL_IOMMU_CTX_TLBIALL);
			*cmds++ = 1;

			cmds += __adreno_add_idle_indirect_cmds(cmds,
			device->mmu.setstate_memory.gpuaddr +
			KGSL_IOMMU_SETSTATE_NOP_OFFSET);

			cmds += adreno_add_read_cmds(device, cmds,
				kgsl_mmu_get_reg_gpuaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TTBR0),
				reg_pt_val,
				device->mmu.setstate_memory.gpuaddr +
				KGSL_IOMMU_SETSTATE_NOP_OFFSET);
		}
	}

	
	cmds += kgsl_mmu_sync_unlock(&device->mmu, cmds);

	if (cpu_is_msm8960())
		cmds += adreno_add_change_mh_phys_limit_cmds(cmds,
			kgsl_mmu_get_reg_gpuaddr(&device->mmu, 0,
						0, KGSL_IOMMU_GLOBAL_BASE),
			device->mmu.setstate_memory.gpuaddr +
			KGSL_IOMMU_SETSTATE_NOP_OFFSET);
	else
		cmds += adreno_add_bank_change_cmds(cmds,
			KGSL_IOMMU_CONTEXT_PRIV,
			device->mmu.setstate_memory.gpuaddr +
			KGSL_IOMMU_SETSTATE_NOP_OFFSET);

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	return cmds - cmds_orig;
}

static unsigned int _adreno_iommu_setstate_v1(struct kgsl_device *device,
					unsigned int *cmds_orig,
					phys_addr_t pt_val,
					int num_iommu_units, uint32_t flags)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	phys_addr_t ttbr0_val;
	unsigned int reg_pt_val;
	unsigned int *cmds = cmds_orig;
	int i;
	unsigned int ttbr0, tlbiall, tlbstatus, tlbsync, mmu_ctrl;

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	for (i = 0; i < num_iommu_units; i++) {
		ttbr0_val = kgsl_mmu_get_default_ttbr0(&device->mmu,
				i, KGSL_IOMMU_CONTEXT_USER);
		ttbr0_val &= ~KGSL_IOMMU_CTX_TTBR0_ADDR_MASK;
		ttbr0_val |= (pt_val & KGSL_IOMMU_CTX_TTBR0_ADDR_MASK);
		if (flags & KGSL_MMUFLAGS_PTUPDATE) {
			mmu_ctrl = kgsl_mmu_get_reg_ahbaddr(
				&device->mmu, i,
				KGSL_IOMMU_CONTEXT_USER,
				KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL) >> 2;

			ttbr0 = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
						KGSL_IOMMU_CONTEXT_USER,
						KGSL_IOMMU_CTX_TTBR0) >> 2;

			if (kgsl_mmu_hw_halt_supported(&device->mmu, i)) {
				*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
				*cmds++ = 0;
				cmds += adreno_wait_reg_eq(cmds,
					adreno_getreg(adreno_dev,
						ADRENO_REG_CP_WFI_PEND_CTR),
					1, 0xFFFFFFFF, 0xF);

				
				*cmds++ = cp_type3_packet(CP_REG_RMW, 3);
				*cmds++ = mmu_ctrl;
				
				*cmds++ =
				 ~(KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_HALT);
				
				*cmds++ =
				   KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_HALT;
				
				cmds += adreno_wait_reg_eq(cmds, mmu_ctrl,
				KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_IDLE,
				KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_IDLE, 0xF);
			}
			
			if (sizeof(phys_addr_t) > sizeof(unsigned long)) {
				reg_pt_val = ttbr0_val & 0xFFFFFFFF;
				*cmds++ = cp_type0_packet(ttbr0, 1);
				*cmds++ = reg_pt_val;
				reg_pt_val = (unsigned int)
				((ttbr0_val & 0xFFFFFFFF00000000ULL) >> 32);
				*cmds++ = cp_type0_packet(ttbr0 + 1, 1);
				*cmds++ = reg_pt_val;
			} else {
				reg_pt_val = ttbr0_val;
				*cmds++ = cp_type0_packet(ttbr0, 1);
				*cmds++ = reg_pt_val;
			}
			if (kgsl_mmu_hw_halt_supported(&device->mmu, i)) {
				
				*cmds++ = cp_type3_packet(CP_REG_RMW, 3);
				*cmds++ = mmu_ctrl;
				
				*cmds++ =
				   ~(KGSL_IOMMU_IMPLDEF_MICRO_MMU_CTRL_HALT);
				
				*cmds++ = 0;
				
				*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
				*cmds++ = 0;
			}
		}
		if (flags & KGSL_MMUFLAGS_TLBFLUSH) {
			tlbiall = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
						KGSL_IOMMU_CONTEXT_USER,
						KGSL_IOMMU_CTX_TLBIALL) >> 2;
			*cmds++ = cp_type0_packet(tlbiall, 1);
			*cmds++ = 1;

			tlbsync = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
						KGSL_IOMMU_CONTEXT_USER,
						KGSL_IOMMU_CTX_TLBSYNC) >> 2;
			*cmds++ = cp_type0_packet(tlbsync, 1);
			*cmds++ = 0;

			tlbstatus = kgsl_mmu_get_reg_ahbaddr(&device->mmu, i,
					KGSL_IOMMU_CONTEXT_USER,
					KGSL_IOMMU_CTX_TLBSTATUS) >> 2;
			cmds += adreno_wait_reg_eq(cmds, tlbstatus, 0,
					KGSL_IOMMU_CTX_TLBSTATUS_SACTIVE, 0xF);
			
			*cmds++ = cp_type3_packet(CP_WAIT_FOR_ME, 1);
			*cmds++ = 0;
		}
	}

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	return cmds - cmds_orig;
}

static bool adreno_use_default_setstate(struct adreno_device *adreno_dev)
{
	return (adreno_isidle(&adreno_dev->dev) ||
		KGSL_STATE_ACTIVE != adreno_dev->dev.state ||
		atomic_read(&adreno_dev->dev.active_cnt) == 0 ||
		adreno_dev->dev.cff_dump_enable);
}

static int adreno_iommu_setstate(struct kgsl_device *device,
					unsigned int context_id,
					uint32_t flags)
{
	phys_addr_t pt_val;
	unsigned int *link = NULL, *cmds;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int num_iommu_units;
	struct kgsl_context *context;
	struct adreno_context *adreno_ctx = NULL;
	struct adreno_ringbuffer *rb = &adreno_dev->ringbuffer;
	unsigned int result;

	if (adreno_use_default_setstate(adreno_dev)) {
		kgsl_mmu_device_setstate(&device->mmu, flags);
		return 0;
	}
	num_iommu_units = kgsl_mmu_get_num_iommu_units(&device->mmu);

	context = kgsl_context_get(device, context_id);
	if (!context) {
		kgsl_mmu_device_setstate(&device->mmu, flags);
		return 0;
	}
	adreno_ctx = ADRENO_CONTEXT(context);

	link = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (link == NULL) {
		result = -ENOMEM;
		goto done;
	}

	cmds = link;

	result = kgsl_mmu_enable_clk(&device->mmu, KGSL_IOMMU_CONTEXT_USER);

	if (result)
		goto done;

	pt_val = kgsl_mmu_get_pt_base_addr(&device->mmu,
				device->mmu.hwpagetable);

	cmds += __adreno_add_idle_indirect_cmds(cmds,
		device->mmu.setstate_memory.gpuaddr +
		KGSL_IOMMU_SETSTATE_NOP_OFFSET);

	if (msm_soc_version_supports_iommu_v0())
		cmds += _adreno_iommu_setstate_v0(device, cmds, pt_val,
						num_iommu_units, flags);
	else
		cmds += _adreno_iommu_setstate_v1(device, cmds, pt_val,
						num_iommu_units, flags);

	
	*cmds++ = cp_type3_packet(CP_INVALIDATE_STATE, 1);
	*cmds++ = 0x7fff;

	if ((unsigned int) (cmds - link) > (PAGE_SIZE / sizeof(unsigned int))) {
		KGSL_DRV_ERR(device, "Temp command buffer overflow\n");
		BUG();
	}
	result = adreno_ringbuffer_issuecmds(device, adreno_ctx,
			KGSL_CMD_FLAGS_PMODE, link,
			(unsigned int)(cmds - link));

	if (result)
		kgsl_mmu_disable_clk(&device->mmu,
						KGSL_IOMMU_CONTEXT_USER);
	else
		kgsl_mmu_disable_clk_on_ts(&device->mmu, rb->global_ts,
						KGSL_IOMMU_CONTEXT_USER);

done:
	kfree(link);
	kgsl_context_put(context);
	return result;
}

static int adreno_gpummu_setstate(struct kgsl_device *device,
					unsigned int context_id,
					uint32_t flags)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned int link[32];
	unsigned int *cmds = &link[0];
	int sizedwords = 0;
	unsigned int mh_mmu_invalidate = 0x00000003; 
	struct kgsl_context *context;
	struct adreno_context *adreno_ctx = NULL;
	int ret = 0;

	if (adreno_is_a20x(adreno_dev))
		flags |= KGSL_MMUFLAGS_TLBFLUSH;
	if (!adreno_use_default_setstate(adreno_dev)) {
		context = kgsl_context_get(device, context_id);
		if (context == NULL)
			return -EINVAL;

		adreno_ctx = ADRENO_CONTEXT(context);

		if (flags & KGSL_MMUFLAGS_PTUPDATE) {
			
			*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
			*cmds++ = 0x00000000;

			
			*cmds++ = cp_type0_packet(MH_MMU_PT_BASE, 1);
			*cmds++ = kgsl_mmu_get_pt_base_addr(&device->mmu,
					device->mmu.hwpagetable);
			sizedwords += 4;
		}

		if (flags & KGSL_MMUFLAGS_TLBFLUSH) {
			if (!(flags & KGSL_MMUFLAGS_PTUPDATE)) {
				*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE,
								1);
				*cmds++ = 0x00000000;
				sizedwords += 2;
			}
			*cmds++ = cp_type0_packet(MH_MMU_INVALIDATE, 1);
			*cmds++ = mh_mmu_invalidate;
			sizedwords += 2;
		}

		if (flags & KGSL_MMUFLAGS_PTUPDATE &&
			adreno_is_a20x(adreno_dev)) {
			*cmds++ = cp_type3_packet(CP_SET_CONSTANT, 2);
			*cmds++ = (0x4 << 16) |
				(REG_PA_SU_SC_MODE_CNTL - 0x2000);
			*cmds++ = 0;	  
			*cmds++ = cp_type3_packet(CP_SET_BIN_BASE_OFFSET, 1);
			*cmds++ = device->mmu.setstate_memory.gpuaddr;
			*cmds++ = cp_type3_packet(CP_DRAW_INDX_BIN, 6);
			*cmds++ = 0;	  
			*cmds++ = 0x0003C004; 
			*cmds++ = 0;	  
			*cmds++ = 3;	  
			*cmds++ =
			device->mmu.setstate_memory.gpuaddr; 
			*cmds++ = 6;	  
			*cmds++ = cp_type3_packet(CP_DRAW_INDX_BIN, 6);
			*cmds++ = 0;	  
			*cmds++ = 0x0003C004; 
			*cmds++ = 0;	  
			*cmds++ = 3;	  
			
			*cmds++ = device->mmu.setstate_memory.gpuaddr;
			*cmds++ = 6;	  
			*cmds++ = cp_type3_packet(CP_WAIT_FOR_IDLE, 1);
			*cmds++ = 0x00000000;
			sizedwords += 21;
		}


		if (flags & (KGSL_MMUFLAGS_PTUPDATE | KGSL_MMUFLAGS_TLBFLUSH)) {
			*cmds++ = cp_type3_packet(CP_INVALIDATE_STATE, 1);
			*cmds++ = 0x7fff; 
			sizedwords += 2;
		}

		ret = adreno_ringbuffer_issuecmds(device, adreno_ctx,
					KGSL_CMD_FLAGS_PMODE,
					&link[0], sizedwords);

		kgsl_context_put(context);
	} else {
		kgsl_mmu_device_setstate(&device->mmu, flags);
	}

	return ret;
}

static int adreno_setstate(struct kgsl_device *device,
			unsigned int context_id,
			uint32_t flags)
{
	
	if (KGSL_MMU_TYPE_GPU == kgsl_mmu_get_mmutype())
		return adreno_gpummu_setstate(device, context_id, flags);
	else if (KGSL_MMU_TYPE_IOMMU == kgsl_mmu_get_mmutype())
		return adreno_iommu_setstate(device, context_id, flags);

	return 0;
}

static unsigned int
a3xx_getchipid(struct kgsl_device *device)
{
	struct kgsl_device_platform_data *pdata =
		kgsl_device_get_drvdata(device);


	return pdata->chipid;
}

static unsigned int
a2xx_getchipid(struct kgsl_device *device)
{
	unsigned int chipid = 0;
	unsigned int coreid, majorid, minorid, patchid, revid;
	struct kgsl_device_platform_data *pdata =
		kgsl_device_get_drvdata(device);

	

	if (pdata->chipid != 0)
		return pdata->chipid;

	kgsl_regread(device, REG_RBBM_PERIPHID1, &coreid);
	kgsl_regread(device, REG_RBBM_PERIPHID2, &majorid);
	kgsl_regread(device, REG_RBBM_PATCH_RELEASE, &revid);

	if (cpu_is_msm8x60())
		chipid = 2 << 24;
	else
		chipid = (coreid & 0xF) << 24;

	chipid |= ((majorid >> 4) & 0xF) << 16;

	minorid = ((revid >> 0)  & 0xFF);

	patchid = ((revid >> 16) & 0xFF);

	
	
	if (cpu_is_qsd8x50())
		patchid = 1;
	else if ((cpu_is_msm8625() || cpu_is_msm8625q()) && minorid == 0)
		minorid = 1;

	chipid |= (minorid << 8) | patchid;

	return chipid;
}

static unsigned int
adreno_getchipid(struct kgsl_device *device)
{
	struct kgsl_device_platform_data *pdata =
		kgsl_device_get_drvdata(device);


	if (pdata->chipid == 0 || ADRENO_CHIPID_MAJOR(pdata->chipid) == 2)
		return a2xx_getchipid(device);
	else
		return a3xx_getchipid(device);
}

static inline bool _rev_match(unsigned int id, unsigned int entry)
{
	return (entry == ANY_ID || entry == id);
}

static void
adreno_identify_gpu(struct adreno_device *adreno_dev)
{
	unsigned int i, core, major, minor, patchid;

	adreno_dev->chip_id = adreno_getchipid(&adreno_dev->dev);

	core = ADRENO_CHIPID_CORE(adreno_dev->chip_id);
	major = ADRENO_CHIPID_MAJOR(adreno_dev->chip_id);
	minor = ADRENO_CHIPID_MINOR(adreno_dev->chip_id);
	patchid = ADRENO_CHIPID_PATCH(adreno_dev->chip_id);

	for (i = 0; i < ARRAY_SIZE(adreno_gpulist); i++) {
		if (core == adreno_gpulist[i].core &&
		    _rev_match(major, adreno_gpulist[i].major) &&
		    _rev_match(minor, adreno_gpulist[i].minor) &&
		    _rev_match(patchid, adreno_gpulist[i].patchid))
			break;
	}

	if (i == ARRAY_SIZE(adreno_gpulist)) {
		adreno_dev->gpurev = ADRENO_REV_UNKNOWN;
		return;
	}

	adreno_dev->gpurev = adreno_gpulist[i].gpurev;
	adreno_dev->gpudev = adreno_gpulist[i].gpudev;
	adreno_dev->pfp_fwfile = adreno_gpulist[i].pfpfw;
	adreno_dev->pm4_fwfile = adreno_gpulist[i].pm4fw;
	adreno_dev->istore_size = adreno_gpulist[i].istore_size;
	adreno_dev->pix_shader_start = adreno_gpulist[i].pix_shader_start;
	adreno_dev->instruction_size = adreno_gpulist[i].instruction_size;
	adreno_dev->gmem_size = adreno_gpulist[i].gmem_size;
	adreno_dev->pm4_jt_idx = adreno_gpulist[i].pm4_jt_idx;
	adreno_dev->pm4_jt_addr = adreno_gpulist[i].pm4_jt_addr;
	adreno_dev->pm4_bstrp_size = adreno_gpulist[i].pm4_bstrp_size;
	adreno_dev->pfp_jt_idx = adreno_gpulist[i].pfp_jt_idx;
	adreno_dev->pfp_jt_addr = adreno_gpulist[i].pfp_jt_addr;
	adreno_dev->pfp_bstrp_size = adreno_gpulist[i].pfp_bstrp_size;
	adreno_dev->pfp_bstrp_ver = adreno_gpulist[i].pfp_bstrp_ver;
	adreno_dev->gpulist_index = i;
	for (i = 0; i < ADRENO_REG_REGISTER_MAX; i++) {
		if (adreno_dev->gpudev->reg_offsets->offset_0 != i &&
			!adreno_dev->gpudev->reg_offsets->offsets[i]) {
			adreno_dev->gpudev->reg_offsets->offsets[i] =
						ADRENO_REG_UNUSED;
		}
	}
}

static struct platform_device_id adreno_id_table[] = {
	{ DEVICE_3D0_NAME, (kernel_ulong_t)&device_3d0.dev, },
	{},
};

MODULE_DEVICE_TABLE(platform, adreno_id_table);

static struct of_device_id adreno_match_table[] = {
	{ .compatible = "qcom,kgsl-3d0", },
	{}
};

static inline int adreno_of_read_property(struct device_node *node,
	const char *prop, unsigned int *ptr)
{
	int ret = of_property_read_u32(node, prop, ptr);
	if (ret)
		KGSL_CORE_ERR("Unable to read '%s'\n", prop);
	return ret;
}

static struct device_node *adreno_of_find_subnode(struct device_node *parent,
	const char *name)
{
	struct device_node *child;

	for_each_child_of_node(parent, child) {
		if (of_device_is_compatible(child, name))
			return child;
	}

	return NULL;
}

static int adreno_of_get_pwrlevels(struct device_node *parent,
	struct kgsl_device_platform_data *pdata)
{
	struct device_node *node, *child;
	int ret = -EINVAL;

	node = adreno_of_find_subnode(parent, "qcom,gpu-pwrlevels");

	if (node == NULL) {
		KGSL_CORE_ERR("Unable to find 'qcom,gpu-pwrlevels'\n");
		return -EINVAL;
	}

	pdata->num_levels = 0;

	for_each_child_of_node(node, child) {
		unsigned int index;
		struct kgsl_pwrlevel *level;

		if (adreno_of_read_property(child, "reg", &index))
			goto done;

		if (index >= KGSL_MAX_PWRLEVELS) {
			KGSL_CORE_ERR("Pwrlevel index %d is out of range\n",
				index);
			continue;
		}

		if (index >= pdata->num_levels)
			pdata->num_levels = index + 1;

		level = &pdata->pwrlevel[index];

		if (adreno_of_read_property(child, "qcom,gpu-freq",
			&level->gpu_freq))
			goto done;

		if (adreno_of_read_property(child, "qcom,bus-freq",
			&level->bus_freq))
			goto done;

		if (adreno_of_read_property(child, "qcom,io-fraction",
			&level->io_fraction))
			level->io_fraction = 0;
	}

	if (adreno_of_read_property(parent, "qcom,initial-pwrlevel",
		&pdata->init_level))
		pdata->init_level = 1;

	if (pdata->init_level < 0 || pdata->init_level > pdata->num_levels) {
		KGSL_CORE_ERR("Initial power level out of range\n");
		pdata->init_level = 1;
	}

	ret = 0;
done:
	return ret;

}

static int adreno_of_get_iommu(struct device_node *parent,
	struct kgsl_device_platform_data *pdata)
{
	struct device_node *node, *child;
	struct kgsl_device_iommu_data *data = NULL;
	struct kgsl_iommu_ctx *ctxs = NULL;
	u32 reg_val[2];
	int ctx_index = 0;

	node = of_parse_phandle(parent, "iommu", 0);
	if (node == NULL)
		return -EINVAL;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		KGSL_CORE_ERR("kzalloc(%d) failed\n", sizeof(*data));
		goto err;
	}

	if (of_property_read_u32_array(node, "reg", reg_val, 2))
		goto err;

	data->physstart = reg_val[0];
	data->physend = data->physstart + reg_val[1] - 1;
	data->iommu_halt_enable = of_property_read_bool(node,
					"qcom,iommu-enable-halt");

	data->iommu_ctx_count = 0;

	for_each_child_of_node(node, child)
		data->iommu_ctx_count++;

	ctxs = kzalloc(data->iommu_ctx_count * sizeof(struct kgsl_iommu_ctx),
		GFP_KERNEL);

	if (ctxs == NULL) {
		KGSL_CORE_ERR("kzalloc(%d) failed\n",
			data->iommu_ctx_count * sizeof(struct kgsl_iommu_ctx));
		goto err;
	}

	for_each_child_of_node(node, child) {
		int ret = of_property_read_string(child, "label",
				&ctxs[ctx_index].iommu_ctx_name);

		if (ret) {
			KGSL_CORE_ERR("Unable to read KGSL IOMMU 'label'\n");
			goto err;
		}

		if (!strcmp("gfx3d_user", ctxs[ctx_index].iommu_ctx_name)) {
			ctxs[ctx_index].ctx_id = 0;
		} else if (!strcmp("gfx3d_priv",
					ctxs[ctx_index].iommu_ctx_name)) {
			ctxs[ctx_index].ctx_id = 1;
		} else if (!strcmp("gfx3d_spare",
					ctxs[ctx_index].iommu_ctx_name)) {
			ctxs[ctx_index].ctx_id = 2;
		} else {
			KGSL_CORE_ERR("dt: IOMMU context %s is invalid\n",
				ctxs[ctx_index].iommu_ctx_name);
			goto err;
		}

		ctx_index++;
	}

	data->iommu_ctxs = ctxs;

	pdata->iommu_data = data;
	pdata->iommu_count = 1;

	return 0;

err:
	kfree(ctxs);
	kfree(data);

	return -EINVAL;
}

static int adreno_of_get_pdata(struct platform_device *pdev)
{
	struct kgsl_device_platform_data *pdata = NULL;
	struct kgsl_device *device;
	int ret = -EINVAL;

	pdev->id_entry = adreno_id_table;

	pdata = pdev->dev.platform_data;
	if (pdata)
		return 0;

	if (of_property_read_string(pdev->dev.of_node, "label", &pdev->name)) {
		KGSL_CORE_ERR("Unable to read 'label'\n");
		goto err;
	}

	if (adreno_of_read_property(pdev->dev.of_node, "qcom,id", &pdev->id))
		goto err;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (pdata == NULL) {
		KGSL_CORE_ERR("kzalloc(%d) failed\n", sizeof(*pdata));
		ret = -ENOMEM;
		goto err;
	}

	if (adreno_of_read_property(pdev->dev.of_node, "qcom,chipid",
		&pdata->chipid))
		goto err;

	
	ret = adreno_of_get_pwrlevels(pdev->dev.of_node, pdata);
	if (ret)
		goto err;

	
	if (adreno_of_read_property(pdev->dev.of_node, "qcom,pm-qos-latency",
		&pdata->pm_qos_latency))
		pdata->pm_qos_latency = 501;


	if (adreno_of_read_property(pdev->dev.of_node, "qcom,idle-timeout",
		&pdata->idle_timeout))
		pdata->idle_timeout = HZ/12;

	pdata->strtstp_sleepwake = of_property_read_bool(pdev->dev.of_node,
						"qcom,strtstp-sleepwake");

	pdata->bus_control = of_property_read_bool(pdev->dev.of_node,
					"qcom,bus-control");

	if (adreno_of_read_property(pdev->dev.of_node, "qcom,clk-map",
		&pdata->clk_map))
		goto err;

	device = (struct kgsl_device *)pdev->id_entry->driver_data;

	if (device->id != KGSL_DEVICE_3D0)
		goto err;

	

	pdata->bus_scale_table = msm_bus_cl_get_pdata(pdev);
	if (IS_ERR_OR_NULL(pdata->bus_scale_table)) {
		ret = PTR_ERR(pdata->bus_scale_table);
		if (!ret)
			ret = -EINVAL;
		goto err;
	}

	ret = adreno_of_get_iommu(pdev->dev.of_node, pdata);
	if (ret)
		goto err;

	pdata->coresight_pdata = of_get_coresight_platform_data(&pdev->dev,
			pdev->dev.of_node);

	pdev->dev.platform_data = pdata;
	return 0;

err:
	if (pdata) {
		if (pdata->iommu_data)
			kfree(pdata->iommu_data->iommu_ctxs);

		kfree(pdata->iommu_data);
	}

	kfree(pdata);

	return ret;
}

#ifdef CONFIG_MSM_OCMEM
static int
adreno_ocmem_gmem_malloc(struct adreno_device *adreno_dev)
{
	if (!(adreno_is_a330(adreno_dev) ||
		adreno_is_a305b(adreno_dev)))
		return 0;

	
	if (adreno_dev->ocmem_hdl != NULL)
		return 0;

	adreno_dev->ocmem_hdl =
		ocmem_allocate(OCMEM_GRAPHICS, adreno_dev->gmem_size);
	if (adreno_dev->ocmem_hdl == NULL)
		return -ENOMEM;

	adreno_dev->gmem_size = adreno_dev->ocmem_hdl->len;
	adreno_dev->ocmem_base = adreno_dev->ocmem_hdl->addr;

	return 0;
}

static void
adreno_ocmem_gmem_free(struct adreno_device *adreno_dev)
{
	if (!(adreno_is_a330(adreno_dev) ||
		adreno_is_a305b(adreno_dev)))
		return;

	if (adreno_dev->ocmem_hdl == NULL)
		return;

	ocmem_free(OCMEM_GRAPHICS, adreno_dev->ocmem_hdl);
	adreno_dev->ocmem_hdl = NULL;
}
#else
static int
adreno_ocmem_gmem_malloc(struct adreno_device *adreno_dev)
{
	return 0;
}

static void
adreno_ocmem_gmem_free(struct adreno_device *adreno_dev)
{
}
#endif

static int __devinit
adreno_probe(struct platform_device *pdev)
{
	struct kgsl_device *device;
	struct kgsl_device_platform_data *pdata = NULL;
	struct adreno_device *adreno_dev;
	int status = -EINVAL;
	bool is_dt;

	is_dt = of_match_device(adreno_match_table, &pdev->dev);

	if (is_dt && pdev->dev.of_node) {
		status = adreno_of_get_pdata(pdev);
		if (status)
			goto error_return;
	}

	device = (struct kgsl_device *)pdev->id_entry->driver_data;
	adreno_dev = ADRENO_DEVICE(device);
	device->parentdev = &pdev->dev;

	status = adreno_ringbuffer_init(device);
	if (status != 0)
		goto error;

	status = kgsl_device_platform_probe(device);
	if (status)
		goto error_close_rb;

	status = adreno_dispatcher_init(adreno_dev);
	if (status)
		goto error_close_device;

	adreno_debugfs_init(device);
	adreno_profile_init(device);

	adreno_ft_init_sysfs(device);

	kgsl_pwrscale_init(&pdev->dev, CONFIG_MSM_ADRENO_DEFAULT_GOVERNOR);


	device->flags &= ~KGSL_FLAGS_SOFT_RESET;
	pdata = kgsl_device_get_drvdata(device);

	adreno_coresight_init(pdev);

	adreno_input_handler.private = device;

	if (input_register_handler(&adreno_input_handler))
		KGSL_DRV_ERR(device, "Unable to register the input handler\n");

	return 0;

error_close_device:
	kgsl_device_platform_remove(device);
error_close_rb:
	adreno_ringbuffer_close(&adreno_dev->ringbuffer);
error:
	device->parentdev = NULL;
error_return:
	return status;
}

static int __devexit adreno_remove(struct platform_device *pdev)
{
	struct kgsl_device *device;
	struct adreno_device *adreno_dev;

	device = (struct kgsl_device *)pdev->id_entry->driver_data;
	adreno_dev = ADRENO_DEVICE(device);

	input_unregister_handler(&adreno_input_handler);

	adreno_coresight_remove(pdev);
	adreno_profile_close(device);

	kgsl_pwrscale_close(device);

	adreno_dispatcher_close(adreno_dev);
	adreno_ringbuffer_close(&adreno_dev->ringbuffer);
	adreno_perfcounter_close(device);
	kgsl_device_platform_remove(device);

	clear_bit(ADRENO_DEVICE_INITIALIZED, &adreno_dev->priv);

	return 0;
}

static int adreno_init(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int i;
	int ret;

	
	adreno_wq = alloc_workqueue("adreno", WQ_HIGHPRI | WQ_UNBOUND, 1);

	kgsl_pwrctrl_set_state(device, KGSL_STATE_INIT);
	if (test_bit(ADRENO_DEVICE_INITIALIZED, &adreno_dev->priv))
		return 0;

	
	kgsl_pwrctrl_enable(device);

	
	adreno_identify_gpu(adreno_dev);

	if (adreno_ringbuffer_read_pm4_ucode(device)) {
		KGSL_DRV_ERR(device, "Reading pm4 microcode failed %s\n",
			adreno_dev->pm4_fwfile);
		BUG_ON(1);
	}

	if (adreno_ringbuffer_read_pfp_ucode(device)) {
		KGSL_DRV_ERR(device, "Reading pfp microcode failed %s\n",
			adreno_dev->pfp_fwfile);
		BUG_ON(1);
	}

	if (adreno_dev->gpurev == ADRENO_REV_UNKNOWN) {
		KGSL_DRV_ERR(device, "Unknown chip ID %x\n",
			adreno_dev->chip_id);
		BUG_ON(1);
	}

	kgsl_pwrctrl_set_state(device, KGSL_STATE_INIT);

	if ((adreno_dev->pm4_fw_version >=
		adreno_gpulist[adreno_dev->gpulist_index].sync_lock_pm4_ver) &&
		(adreno_dev->pfp_fw_version >=
		adreno_gpulist[adreno_dev->gpulist_index].sync_lock_pfp_ver))
		device->mmu.flags |= KGSL_MMU_FLAGS_IOMMU_SYNC;

	
	ft_detect_regs[0] = adreno_getreg(adreno_dev,
						ADRENO_REG_RBBM_STATUS);
	ft_detect_regs[1] = adreno_getreg(adreno_dev,
						ADRENO_REG_CP_RB_RPTR);
	ft_detect_regs[2] = adreno_getreg(adreno_dev,
						ADRENO_REG_CP_IB1_BASE);
	ft_detect_regs[3] = adreno_getreg(adreno_dev,
						ADRENO_REG_CP_IB1_BUFSZ);
	ft_detect_regs[4] = adreno_getreg(adreno_dev,
						ADRENO_REG_CP_IB2_BASE);
	ft_detect_regs[5] = adreno_getreg(adreno_dev,
						ADRENO_REG_CP_IB2_BUFSZ);
	for (i = 6; i < FT_DETECT_REGS_COUNT; i++)
		ft_detect_regs[i] = 0;

	
	if (adreno_is_a330v2(adreno_dev))
		set_bit(ADRENO_DEVICE_HANG_INTR, &adreno_dev->priv);

	ret = adreno_perfcounter_init(device);
	if (ret)
		goto done;

	
	kgsl_pwrctrl_disable(device);

	
	if (adreno_is_a3xx(adreno_dev))
		adreno_a3xx_pwron_fixup_init(adreno_dev);

	set_bit(ADRENO_DEVICE_INITIALIZED, &adreno_dev->priv);
done:
	return ret;
}

static int _adreno_start(struct adreno_device *adreno_dev)
{
	struct kgsl_device *device = &adreno_dev->dev;
	int status = -EINVAL;
	unsigned int state = device->state;
	unsigned int regulator_left_on = 0;

	kgsl_cffdump_open(device);

	kgsl_pwrctrl_set_state(device, KGSL_STATE_INIT);

	regulator_left_on = (regulator_is_enabled(device->pwrctrl.gpu_reg) ||
				(device->pwrctrl.gpu_cx &&
				regulator_is_enabled(device->pwrctrl.gpu_cx)));

	
	adreno_clear_gpu_fault(adreno_dev);

	
	kgsl_pwrctrl_enable(device);

	
	set_bit(ADRENO_DEVICE_PWRON, &adreno_dev->priv);

	
	if (adreno_is_a2xx(adreno_dev)) {
		if (adreno_is_a20x(adreno_dev)) {
			device->mh.mh_intf_cfg1 = 0;
			device->mh.mh_intf_cfg2 = 0;
		}

		kgsl_mh_start(device);
	}

	status = kgsl_mmu_start(device);
	if (status)
		goto error_clk_off;

	status = adreno_ocmem_gmem_malloc(adreno_dev);
	if (status) {
		KGSL_DRV_ERR(device, "OCMEM malloc failed\n");
		goto error_mmu_off;
	}

	if (regulator_left_on && adreno_dev->gpudev->soft_reset) {
		adreno_dev->gpudev->soft_reset(adreno_dev);
	}

	
	adreno_perfcounter_restore(adreno_dev);

	
	adreno_dev->gpudev->start(adreno_dev);

	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_ON);
	device->ftbl->irqctrl(device, 1);

	status = adreno_ringbuffer_cold_start(&adreno_dev->ringbuffer);
	if (status)
		goto error_irq_off;

	status = adreno_perfcounter_start(adreno_dev);
	if (status)
		goto error_rb_stop;

	
	adreno_dispatcher_start(device);

	device->reset_counter++;

	set_bit(ADRENO_DEVICE_STARTED, &adreno_dev->priv);

	return 0;

error_rb_stop:
	adreno_ringbuffer_stop(&adreno_dev->ringbuffer);
error_irq_off:
	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_OFF);

error_mmu_off:
	kgsl_mmu_stop(&device->mmu);

error_clk_off:
	kgsl_pwrctrl_disable(device);
	
	kgsl_pwrctrl_set_state(device, state);

	return status;
}

static int _status;

static void adreno_start_work(struct work_struct *work)
{
	struct adreno_device *adreno_dev = container_of(work,
		struct adreno_device, start_work);
	struct kgsl_device *device = &adreno_dev->dev;

	
	set_user_nice(current, _wake_nice);

	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
	if (!test_bit(ADRENO_DEVICE_STARTED, &adreno_dev->priv))
		_status = _adreno_start(adreno_dev);
	else
		_status = 0;
	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
}

static int adreno_start(struct kgsl_device *device, int priority)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	
	if (!priority)
		return _adreno_start(adreno_dev);

	queue_work(adreno_wq, &adreno_dev->start_work);
	kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
	flush_work(&adreno_dev->start_work);
	kgsl_mutex_lock(&device->mutex, &device->mutex_owner);

	return _status;
}

static int adreno_stop(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	if (adreno_dev->drawctxt_active)
		kgsl_context_put(&adreno_dev->drawctxt_active->base);

	adreno_dev->drawctxt_active = NULL;

	adreno_dispatcher_stop(adreno_dev);
	adreno_ringbuffer_stop(&adreno_dev->ringbuffer);

	kgsl_mmu_stop(&device->mmu);

	device->ftbl->irqctrl(device, 0);
	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_OFF);
	del_timer_sync(&device->idle_timer);

	adreno_ocmem_gmem_free(adreno_dev);

	
	adreno_perfcounter_save(adreno_dev);

	
	kgsl_pwrctrl_disable(device);

	kgsl_cffdump_close(device);

	clear_bit(ADRENO_DEVICE_STARTED, &adreno_dev->priv);

	return 0;
}

int adreno_reset(struct kgsl_device *device)
{
	int ret = -EINVAL;
	struct kgsl_mmu *mmu = &device->mmu;
	int i = 0;

	
	if (!atomic_read(&mmu->fault)) {
		ret = adreno_soft_reset(device);
		if (ret)
			KGSL_DEV_ERR_ONCE(device, "Device soft reset failed\n");
	}
	if (ret) {
		
		adreno_stop(device);

		
		for (i = 0; i < NUM_TIMES_RESET_RETRY; i++) {
			ret = adreno_start(device, 0);
			if (!ret)
				break;

			msleep(20);
		}
	}
	if (ret)
		return ret;

	if (0 != i)
		KGSL_DRV_WARN(device, "Device hard reset tried %d tries\n", i);


	if (atomic_read(&device->active_cnt))
		kgsl_pwrctrl_set_state(device, KGSL_STATE_ACTIVE);

	
	kgsl_mmu_setstate(&device->mmu, device->mmu.defaultpagetable,
			KGSL_MEMSTORE_GLOBAL);

	return ret;
}

static int _ft_sysfs_store(const char *buf, size_t count, unsigned int *ptr)
{
	char temp[20];
	unsigned long val;
	int rc;

	snprintf(temp, sizeof(temp), "%.*s",
			 (int)min(count, sizeof(temp) - 1), buf);
	rc = kstrtoul(temp, 0, &val);
	if (rc)
		return rc;

	*ptr = val;

	return count;
}

struct adreno_device *_get_adreno_dev(struct device *dev)
{
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	return device ? ADRENO_DEVICE(device) : NULL;
}

static int _ft_policy_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct adreno_device *adreno_dev = _get_adreno_dev(dev);
	int ret;
	if (adreno_dev == NULL)
		return 0;

	mutex_lock(&adreno_dev->dev.mutex);
	ret = _ft_sysfs_store(buf, count, &adreno_dev->ft_policy);
	mutex_unlock(&adreno_dev->dev.mutex);

	return ret;
}

static int _ft_policy_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct adreno_device *adreno_dev = _get_adreno_dev(dev);
	if (adreno_dev == NULL)
		return 0;
	return snprintf(buf, PAGE_SIZE, "0x%X\n", adreno_dev->ft_policy);
}

static int _ft_pagefault_policy_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct adreno_device *adreno_dev = _get_adreno_dev(dev);
	int ret = 0;
	unsigned int policy = 0;
	if (adreno_dev == NULL)
		return 0;

	mutex_lock(&adreno_dev->dev.mutex);

	
	if (count != _ft_sysfs_store(buf, count, &policy))
		ret = -EINVAL;

	if (!ret) {
		policy &= (KGSL_FT_PAGEFAULT_INT_ENABLE |
				KGSL_FT_PAGEFAULT_GPUHALT_ENABLE |
				KGSL_FT_PAGEFAULT_LOG_ONE_PER_PAGE |
				KGSL_FT_PAGEFAULT_LOG_ONE_PER_INT);
		ret = kgsl_mmu_set_pagefault_policy(&(adreno_dev->dev.mmu),
				adreno_dev->ft_pf_policy);
		if (!ret)
			adreno_dev->ft_pf_policy = policy;
	}
	mutex_unlock(&adreno_dev->dev.mutex);

	if (!ret)
		return count;
	else
		return 0;
}

static int _ft_pagefault_policy_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct adreno_device *adreno_dev = _get_adreno_dev(dev);
	if (adreno_dev == NULL)
		return 0;
	return snprintf(buf, PAGE_SIZE, "0x%X\n", adreno_dev->ft_pf_policy);
}

static int _ft_fast_hang_detect_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct adreno_device *adreno_dev = _get_adreno_dev(dev);
	int ret, tmp;

	if (adreno_dev == NULL)
		return 0;

	mutex_lock(&adreno_dev->dev.mutex);

	tmp = adreno_dev->fast_hang_detect;

	ret = _ft_sysfs_store(buf, count, &adreno_dev->fast_hang_detect);

	if (tmp != adreno_dev->fast_hang_detect) {
		if (adreno_dev->fast_hang_detect) {
			if (adreno_dev->gpudev->fault_detect_start &&
				!kgsl_active_count_get(&adreno_dev->dev)) {
				adreno_dev->gpudev->fault_detect_start(
					adreno_dev);
				kgsl_active_count_put(&adreno_dev->dev);
			}
		} else {
			if (adreno_dev->gpudev->fault_detect_stop)
				adreno_dev->gpudev->fault_detect_stop(
					adreno_dev);
		}
	}

	mutex_unlock(&adreno_dev->dev.mutex);

	return ret;

}

static int _ft_fast_hang_detect_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct adreno_device *adreno_dev = _get_adreno_dev(dev);
	if (adreno_dev == NULL)
		return 0;
	return snprintf(buf, PAGE_SIZE, "%d\n",
				(adreno_dev->fast_hang_detect ? 1 : 0));
}

static int _ft_long_ib_detect_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct adreno_device *adreno_dev = _get_adreno_dev(dev);
	int ret;
	if (adreno_dev == NULL)
		return 0;

	mutex_lock(&adreno_dev->dev.mutex);
	ret = _ft_sysfs_store(buf, count, &adreno_dev->long_ib_detect);
	mutex_unlock(&adreno_dev->dev.mutex);

	return ret;

}

static int _ft_long_ib_detect_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct adreno_device *adreno_dev = _get_adreno_dev(dev);
	if (adreno_dev == NULL)
		return 0;
	return snprintf(buf, PAGE_SIZE, "%d\n",
				(adreno_dev->long_ib_detect ? 1 : 0));
}

static ssize_t _wake_timeout_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	return _ft_sysfs_store(buf, count, &_wake_timeout);
}

static ssize_t _wake_timeout_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", _wake_timeout);
}

static ssize_t _ft_hang_intr_status_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned int new_setting, old_setting;
	struct kgsl_device *device = kgsl_device_from_dev(dev);
	struct adreno_device *adreno_dev;
	int ret;
	if (device == NULL)
		return 0;
	adreno_dev = ADRENO_DEVICE(device);

	mutex_lock(&device->mutex);
	ret = _ft_sysfs_store(buf, count, &new_setting);
	if (ret != count)
		goto done;
	if (new_setting)
		new_setting = 1;
	old_setting =
		(test_bit(ADRENO_DEVICE_HANG_INTR, &adreno_dev->priv) ? 1 : 0);
	if (new_setting != old_setting) {
		if (new_setting)
			set_bit(ADRENO_DEVICE_HANG_INTR, &adreno_dev->priv);
		else
			clear_bit(ADRENO_DEVICE_HANG_INTR, &adreno_dev->priv);
		
		switch (device->state) {
		case KGSL_STATE_NAP:
		case KGSL_STATE_SLEEP:
			kgsl_pwrctrl_wake(device, 0);
		case KGSL_STATE_ACTIVE:
			adreno_dev->gpudev->irq_control(adreno_dev, 1);
		case KGSL_STATE_SLUMBER:
		case KGSL_STATE_SUSPEND:
			break;
		default:
			ret = -EACCES;
			
			if (new_setting)
				clear_bit(ADRENO_DEVICE_HANG_INTR,
					&adreno_dev->priv);
			else
				set_bit(ADRENO_DEVICE_HANG_INTR,
					&adreno_dev->priv);
			goto done;
		}
	}
done:
	mutex_unlock(&device->mutex);
	return ret;
}

static ssize_t _ft_hang_intr_status_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct adreno_device *adreno_dev = _get_adreno_dev(dev);
	if (adreno_dev == NULL)
		return 0;
	return snprintf(buf, PAGE_SIZE, "%d\n",
		test_bit(ADRENO_DEVICE_HANG_INTR, &adreno_dev->priv) ? 1 : 0);
}

#define FT_DEVICE_ATTR(name) \
	DEVICE_ATTR(name, 0644,	_ ## name ## _show, _ ## name ## _store);

FT_DEVICE_ATTR(ft_policy);
FT_DEVICE_ATTR(ft_pagefault_policy);
FT_DEVICE_ATTR(ft_fast_hang_detect);
FT_DEVICE_ATTR(ft_long_ib_detect);
FT_DEVICE_ATTR(ft_hang_intr_status);

static DEVICE_INT_ATTR(wake_nice, 0644, _wake_nice);
static FT_DEVICE_ATTR(wake_timeout);

const struct device_attribute *ft_attr_list[] = {
	&dev_attr_ft_policy,
	&dev_attr_ft_pagefault_policy,
	&dev_attr_ft_fast_hang_detect,
	&dev_attr_ft_long_ib_detect,
	&dev_attr_wake_nice.attr,
	&dev_attr_wake_timeout,
	&dev_attr_ft_hang_intr_status,
	NULL,
};

int adreno_ft_init_sysfs(struct kgsl_device *device)
{
	return kgsl_create_device_sysfs_files(device->dev, ft_attr_list);
}

void adreno_ft_uninit_sysfs(struct kgsl_device *device)
{
	kgsl_remove_device_sysfs_files(device->dev, ft_attr_list);
}

static int adreno_getproperty(struct kgsl_device *device,
				enum kgsl_property_type type,
				void *value,
				unsigned int sizebytes)
{
	int status = -EINVAL;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	switch (type) {
	case KGSL_PROP_DEVICE_INFO:
		{
			struct kgsl_devinfo devinfo;

			if (sizebytes != sizeof(devinfo)) {
				status = -EINVAL;
				break;
			}

			memset(&devinfo, 0, sizeof(devinfo));
			devinfo.device_id = device->id+1;
			devinfo.chip_id = adreno_dev->chip_id;
			devinfo.mmu_enabled = kgsl_mmu_enabled();
			devinfo.gpu_id = adreno_dev->gpurev;
			devinfo.gmem_gpubaseaddr = adreno_dev->gmem_base;
			devinfo.gmem_sizebytes = adreno_dev->gmem_size;

			if (copy_to_user(value, &devinfo, sizeof(devinfo)) !=
					0) {
				status = -EFAULT;
				break;
			}
			status = 0;
		}
		break;
	case KGSL_PROP_DEVICE_SHADOW:
		{
			struct kgsl_shadowprop shadowprop;

			if (sizebytes != sizeof(shadowprop)) {
				status = -EINVAL;
				break;
			}
			memset(&shadowprop, 0, sizeof(shadowprop));
			if (device->memstore.hostptr) {
				shadowprop.gpuaddr = device->memstore.gpuaddr;
				shadowprop.size = device->memstore.size;
				shadowprop.flags = KGSL_FLAGS_INITIALIZED |
					KGSL_FLAGS_PER_CONTEXT_TIMESTAMPS;
			}
			if (copy_to_user(value, &shadowprop,
				sizeof(shadowprop))) {
				status = -EFAULT;
				break;
			}
			status = 0;
		}
		break;
	case KGSL_PROP_MMU_ENABLE:
		{
			int mmu_prop = kgsl_mmu_enabled();

			if (sizebytes != sizeof(int)) {
				status = -EINVAL;
				break;
			}
			if (copy_to_user(value, &mmu_prop, sizeof(mmu_prop))) {
				status = -EFAULT;
				break;
			}
			status = 0;
		}
		break;
	case KGSL_PROP_INTERRUPT_WAITS:
		{
			int int_waits = 1;
			if (sizebytes != sizeof(int)) {
				status = -EINVAL;
				break;
			}
			if (copy_to_user(value, &int_waits, sizeof(int))) {
				status = -EFAULT;
				break;
			}
			status = 0;
		}
		break;
	default:
		status = -EINVAL;
	}

	return status;
}

static int adreno_set_constraint(struct kgsl_device *device,
				struct kgsl_context *context,
				struct kgsl_device_constraint *constraint)
{
	int status = 0;

	switch (constraint->type) {
	case KGSL_CONSTRAINT_PWRLEVEL: {
		struct kgsl_device_constraint_pwrlevel pwr;

		if (constraint->size != sizeof(pwr)) {
			status = -EINVAL;
			break;
		}

		if (copy_from_user(&pwr,
				(void __user *)constraint->data,
				sizeof(pwr))) {
			status = -EFAULT;
			break;
		}
		if (pwr.level >= KGSL_CONSTRAINT_PWR_MAXLEVELS) {
			status = -EINVAL;
			break;
		}

		context->pwr_constraint.type =
				KGSL_CONSTRAINT_PWRLEVEL;
		context->pwr_constraint.sub_type = pwr.level;
		}
		break;
	case KGSL_CONSTRAINT_NONE:
		context->pwr_constraint.type = KGSL_CONSTRAINT_NONE;
		break;

	default:
		status = -EINVAL;
		break;
	}

	return status;
}

static int adreno_setproperty(struct kgsl_device_private *dev_priv,
				enum kgsl_property_type type,
				void *value,
				unsigned int sizebytes)
{
	int status = -EINVAL;
	struct kgsl_device *device = dev_priv->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	switch (type) {
	case KGSL_PROP_PWRCTRL: {
			unsigned int enable;

			if (sizebytes != sizeof(enable))
				break;

			if (copy_from_user(&enable, (void __user *) value,
				sizeof(enable))) {
				status = -EFAULT;
				break;
			}

			kgsl_mutex_lock(&device->mutex, &device->mutex_owner);

			if (enable) {
				device->pwrctrl.ctrl_flags = 0;
				adreno_dev->fast_hang_detect = 1;

				if (adreno_dev->gpudev->fault_detect_start)
					adreno_dev->gpudev->fault_detect_start(
						adreno_dev);

				kgsl_pwrscale_enable(device);
			} else {
				kgsl_pwrctrl_wake(device, 0);
				device->pwrctrl.ctrl_flags = KGSL_PWR_ON;
				adreno_dev->fast_hang_detect = 0;
				if (adreno_dev->gpudev->fault_detect_stop)
					adreno_dev->gpudev->fault_detect_stop(
						adreno_dev);
				kgsl_pwrscale_disable(device);
			}

			kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
			status = 0;
		}
		break;
	case KGSL_PROP_PWR_CONSTRAINT: {
			struct kgsl_device_constraint constraint;
			struct kgsl_context *context;

			if (sizebytes != sizeof(constraint))
				break;

			if (copy_from_user(&constraint, value,
				sizeof(constraint))) {
				status = -EFAULT;
				break;
			}

			context = kgsl_context_get_owner(dev_priv,
							constraint.context_id);

			if (context == NULL)
				break;

			status = adreno_set_constraint(device, context,
								&constraint);

			kgsl_context_put(context);
		}
		break;
	default:
		break;
	}

	return status;
}

bool adreno_hw_isidle(struct kgsl_device *device)
{
	unsigned int reg_rbbm_status;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	
	if (adreno_dev->gpudev->irq_pending(adreno_dev))
		return false;

	adreno_readreg(adreno_dev, ADRENO_REG_RBBM_STATUS,
		&reg_rbbm_status);

	if (adreno_is_a2xx(adreno_dev)) {
		if (reg_rbbm_status == 0x110)
			return true;
	} else if (adreno_is_a3xx(adreno_dev)) {
		if (!(reg_rbbm_status & 0x80000000))
			return true;
	}

	return false;
}

int adreno_soft_reset(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int ret;

	if (!adreno_dev->gpudev->soft_reset) {
		dev_WARN_ONCE(device->dev, 1, "Soft reset not supported");
		return -EINVAL;
	}

	if (adreno_dev->drawctxt_active)
		kgsl_context_put(&adreno_dev->drawctxt_active->base);

	adreno_dev->drawctxt_active = NULL;

	
	adreno_ringbuffer_stop(&adreno_dev->ringbuffer);

	if (kgsl_pwrctrl_isenabled(device))
		device->ftbl->irqctrl(device, 0);

	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_OFF);

	adreno_clear_gpu_fault(adreno_dev);

	
	del_timer_sync(&device->idle_timer);

	
	kgsl_pwrctrl_enable(device);

	
	adreno_perfcounter_save(adreno_dev);

	
	adreno_dev->gpudev->soft_reset(adreno_dev);

	
	adreno_perfcounter_restore(adreno_dev);

	
	adreno_dev->gpudev->start(adreno_dev);

	
	kgsl_pwrctrl_irq(device, KGSL_PWRFLAGS_ON);
	device->ftbl->irqctrl(device, 1);


	if (adreno_dev->pm4_jt_idx)
		ret = adreno_ringbuffer_warm_start(&adreno_dev->ringbuffer);
	else
		ret = adreno_ringbuffer_cold_start(&adreno_dev->ringbuffer);

	if (ret)
		return ret;

	device->reset_counter++;

	return 0;
}

bool adreno_isidle(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned int rptr;

	if (!kgsl_pwrctrl_isenabled(device))
		return true;

	rptr = adreno_get_rptr(&adreno_dev->ringbuffer);

	smp_mb();

	if (rptr == adreno_dev->ringbuffer.wptr)
		return adreno_hw_isidle(device);

	return false;
}


int adreno_idle(struct kgsl_device *device)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	unsigned long wait = jiffies + msecs_to_jiffies(ADRENO_IDLE_TIMEOUT);


	BUG_ON(!mutex_is_locked(&device->mutex));

	if (adreno_is_a3xx(adreno_dev))
		kgsl_cffdump_regpoll(device,
			adreno_getreg(adreno_dev, ADRENO_REG_RBBM_STATUS) << 2,
			0x00000000, 0x80000000);
	else
		kgsl_cffdump_regpoll(device,
			adreno_getreg(adreno_dev, ADRENO_REG_RBBM_STATUS) << 2,
			0x110, 0x110);

	while (time_before(jiffies, wait)) {

		if (adreno_gpu_fault(adreno_dev) != 0)
			return -EDEADLK;

		if (adreno_isidle(device))
			return 0;
	}

	return -ETIMEDOUT;
}

static int adreno_drain(struct kgsl_device *device)
{
	INIT_COMPLETION(device->cmdbatch_gate);

	return 0;
}

static int adreno_suspend_context(struct kgsl_device *device)
{
	int status = 0;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

	
	adreno_profile_process_results(device);

	
	if (adreno_dev->drawctxt_active != NULL) {
		adreno_drawctxt_switch(adreno_dev, NULL, 0);
		status = adreno_idle(device);
	}

	return status;
}


struct kgsl_memdesc *adreno_find_ctxtmem(struct kgsl_device *device,
	phys_addr_t pt_base, unsigned int gpuaddr, unsigned int size)
{
	struct kgsl_context *context;
	int next = 0;
	struct kgsl_memdesc *desc = NULL;

	read_lock(&device->context_lock);
	while (1) {
		context = idr_get_next(&device->context_idr, &next);
		if (context == NULL)
			break;

		if (kgsl_mmu_pt_equal(&device->mmu,
					context->proc_priv->pagetable,
					pt_base)) {
			struct adreno_context *adreno_context;

			adreno_context = ADRENO_CONTEXT(context);
			desc = &adreno_context->gpustate;
			if (kgsl_gpuaddr_in_memdesc(desc, gpuaddr, size))
				break;

			desc = &adreno_context->context_gmem_shadow.gmemshadow;
			if (kgsl_gpuaddr_in_memdesc(desc, gpuaddr, size))
				break;
		}
		next = next + 1;
		desc = NULL;
	}
	read_unlock(&device->context_lock);
	return desc;
}

struct kgsl_memdesc *adreno_find_region(struct kgsl_device *device,
						phys_addr_t pt_base,
						unsigned int gpuaddr,
						unsigned int size,
						struct kgsl_mem_entry **entry)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct adreno_ringbuffer *ringbuffer = &adreno_dev->ringbuffer;

	*entry = NULL;
	if (kgsl_gpuaddr_in_memdesc(&ringbuffer->buffer_desc, gpuaddr, size))
		return &ringbuffer->buffer_desc;

	if (kgsl_gpuaddr_in_memdesc(&device->memstore, gpuaddr, size))
		return &device->memstore;

	if (kgsl_gpuaddr_in_memdesc(&adreno_dev->pwron_fixup, gpuaddr, size))
		return &adreno_dev->pwron_fixup;

	if (kgsl_gpuaddr_in_memdesc(&device->mmu.setstate_memory, gpuaddr,
					size))
		return &device->mmu.setstate_memory;

	*entry = kgsl_get_mem_entry(device, pt_base, gpuaddr, size);

	if (*entry)
		return &((*entry)->memdesc);

	return adreno_find_ctxtmem(device, pt_base, gpuaddr, size);
}

uint8_t *adreno_convertaddr(struct kgsl_device *device, phys_addr_t pt_base,
			    unsigned int gpuaddr, unsigned int size,
				struct kgsl_mem_entry **entry)
{
	struct kgsl_memdesc *memdesc;

	memdesc = adreno_find_region(device, pt_base, gpuaddr, size, entry);

	return memdesc ? kgsl_gpuaddr_to_vaddr(memdesc, gpuaddr) : NULL;
}


static void adreno_read(struct kgsl_device *device, void *base,
		unsigned int offsetwords, unsigned int *value,
		unsigned int mem_len)
{

	unsigned int *reg;
	BUG_ON(offsetwords*sizeof(uint32_t) >= mem_len);
	reg = (unsigned int *)(base + (offsetwords << 2));

	if (!in_interrupt())
		kgsl_pre_hwaccess(device);

	*value = __raw_readl(reg);
	rmb();
}

static void adreno_regread(struct kgsl_device *device, unsigned int offsetwords,
	unsigned int *value)
{
	adreno_read(device, device->reg_virt, offsetwords, value,
						device->reg_len);
}

void adreno_shadermem_regread(struct kgsl_device *device,
	unsigned int offsetwords, unsigned int *value)
{
	adreno_read(device, device->shader_mem_virt, offsetwords, value,
					device->shader_mem_len);
}

static void adreno_regwrite(struct kgsl_device *device,
				unsigned int offsetwords,
				unsigned int value)
{
	unsigned int *reg;

	BUG_ON(offsetwords*sizeof(uint32_t) >= device->reg_len);

	if (!in_interrupt())
		kgsl_pre_hwaccess(device);

	kgsl_trace_regwrite(device, offsetwords, value);

	kgsl_cffdump_regwrite(device, offsetwords << 2, value);
	reg = (unsigned int *)(device->reg_virt + (offsetwords << 2));

	wmb();
	__raw_writel(value, reg);
}

static int adreno_waittimestamp(struct kgsl_device *device,
		struct kgsl_context *context,
		unsigned int timestamp,
		unsigned int msecs)
{
	int ret;
	struct adreno_context *drawctxt;

	if (context == NULL) {
		
		dev_WARN_ONCE(device->dev, 1,
			"IOCTL_KGSL_DEVICE_WAITTIMESTAMP is deprecated\n");
		return -ENOTTY;
	}

	
	if (kgsl_context_detached(context))
		return -EINVAL;

	ret = adreno_drawctxt_wait(ADRENO_DEVICE(device), context,
		timestamp, msecs);

	
	drawctxt = ADRENO_CONTEXT(context);

	if (drawctxt->state == ADRENO_CONTEXT_STATE_INVALID)
		ret = -EDEADLK;


	if (!ret && test_and_clear_bit(ADRENO_CONTEXT_FAULT, &drawctxt->priv))
		ret = -EPROTO;

	return ret;
}

static unsigned int adreno_readtimestamp(struct kgsl_device *device,
		struct kgsl_context *context, enum kgsl_timestamp_type type)
{
	unsigned int timestamp = 0;
	unsigned int id = context ? context->id : KGSL_MEMSTORE_GLOBAL;

	switch (type) {
	case KGSL_TIMESTAMP_QUEUED: {
		struct adreno_device *adreno_dev = ADRENO_DEVICE(device);

		timestamp = adreno_context_timestamp(context,
				&adreno_dev->ringbuffer);
		break;
	}
	case KGSL_TIMESTAMP_CONSUMED:
		kgsl_sharedmem_readl(&device->memstore, &timestamp,
			KGSL_MEMSTORE_OFFSET(id, soptimestamp));
		break;
	case KGSL_TIMESTAMP_RETIRED:
		kgsl_sharedmem_readl(&device->memstore, &timestamp,
			KGSL_MEMSTORE_OFFSET(id, eoptimestamp));
		break;
	}

	rmb();

	return timestamp;
}

static long adreno_ioctl(struct kgsl_device_private *dev_priv,
			      unsigned int cmd, void *data)
{
	struct kgsl_device *device = dev_priv->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	int result = 0;

	switch (cmd) {
	case IOCTL_KGSL_DRAWCTXT_SET_BIN_BASE_OFFSET: {
		struct kgsl_drawctxt_set_bin_base_offset *binbase = data;
		struct kgsl_context *context;

		binbase = data;

		context = kgsl_context_get_owner(dev_priv,
			binbase->drawctxt_id);
		if (context) {
			adreno_drawctxt_set_bin_base_offset(
				device, context, binbase->offset);
		} else {
			result = -EINVAL;
			KGSL_DRV_ERR(device,
				"invalid drawctxt drawctxt_id %d "
				"device_id=%d\n",
				binbase->drawctxt_id, device->id);
		}

		kgsl_context_put(context);
		break;
	}
	case IOCTL_KGSL_PERFCOUNTER_GET: {
		struct kgsl_perfcounter_get *get = data;
		kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
		result = kgsl_active_count_get(device);
		if (result == 0) {
			result = adreno_perfcounter_get(adreno_dev,
				get->groupid, get->countable, &get->offset,
				&get->offset_hi, PERFCOUNTER_FLAG_NONE);
			kgsl_active_count_put(device);
		}
		kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
		break;
	}
	case IOCTL_KGSL_PERFCOUNTER_PUT: {
		struct kgsl_perfcounter_put *put = data;
		kgsl_mutex_lock(&device->mutex, &device->mutex_owner);
		result = adreno_perfcounter_put(adreno_dev, put->groupid,
			put->countable, PERFCOUNTER_FLAG_NONE);
		kgsl_mutex_unlock(&device->mutex, &device->mutex_owner);
		break;
	}
	case IOCTL_KGSL_PERFCOUNTER_QUERY: {
		struct kgsl_perfcounter_query *query = data;
		result = adreno_perfcounter_query_group(adreno_dev,
			query->groupid, query->countables,
			query->count, &query->max_counters);
		break;
	}
	case IOCTL_KGSL_PERFCOUNTER_READ: {
		struct kgsl_perfcounter_read *read = data;
		result = adreno_perfcounter_read_group(adreno_dev,
			read->reads, read->count);
		break;
	}
	default:
		KGSL_DRV_INFO(dev_priv->device,
			"invalid ioctl code %08x\n", cmd);
		result = -ENOIOCTLCMD;
		break;
	}
	return result;

}

static inline s64 adreno_ticks_to_us(u32 ticks, u32 freq)
{
	freq /= 1000000;
	return ticks / freq;
}

static void adreno_power_stats(struct kgsl_device *device,
				struct kgsl_power_stats *stats)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_pwrctrl *pwr = &device->pwrctrl;
	struct adreno_busy_data busy_data;

	memset(stats, 0, sizeof(*stats));
	memset(&busy_data, 0, sizeof(busy_data));


	if (device->state != KGSL_STATE_ACTIVE)
		return;

	
	adreno_dev->gpudev->busy_cycles(adreno_dev, &busy_data);

	stats->busy_time = adreno_ticks_to_us(busy_data.gpu_busy,
					      kgsl_pwrctrl_active_freq(pwr));
	stats->ram_time = busy_data.vbif_ram_cycles;
	stats->ram_wait = busy_data.vbif_starved_ram;
}

void adreno_irqctrl(struct kgsl_device *device, int state)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	adreno_dev->gpudev->irq_control(adreno_dev, state);
}

static unsigned int adreno_gpuid(struct kgsl_device *device,
	unsigned int *chipid)
{
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);


	if (chipid != NULL)
		*chipid = adreno_dev->chip_id;


	return (0x0003 << 16) | ((int) adreno_dev->gpurev);
}

static const struct kgsl_functable adreno_functable = {
	
	.regread = adreno_regread,
	.regwrite = adreno_regwrite,
	.idle = adreno_idle,
	.isidle = adreno_isidle,
	.suspend_context = adreno_suspend_context,
	.init = adreno_init,
	.start = adreno_start,
	.stop = adreno_stop,
	.getproperty = adreno_getproperty,
	.waittimestamp = adreno_waittimestamp,
	.readtimestamp = adreno_readtimestamp,
	.issueibcmds = adreno_ringbuffer_issueibcmds,
	.ioctl = adreno_ioctl,
	.setup_pt = adreno_setup_pt,
	.cleanup_pt = adreno_cleanup_pt,
	.power_stats = adreno_power_stats,
	.irqctrl = adreno_irqctrl,
	.gpuid = adreno_gpuid,
	.snapshot = adreno_snapshot,
	.irq_handler = adreno_irq_handler,
	.drain = adreno_drain,
	
	.setstate = adreno_setstate,
	.drawctxt_create = adreno_drawctxt_create,
	.drawctxt_detach = adreno_drawctxt_detach,
	.drawctxt_destroy = adreno_drawctxt_destroy,
	.drawctxt_dump = adreno_drawctxt_dump,
	.setproperty = adreno_setproperty,
	.postmortem_dump = adreno_dump,
	.drawctxt_sched = adreno_drawctxt_sched,
	.resume = adreno_dispatcher_start,
};

static struct platform_driver adreno_platform_driver = {
	.probe = adreno_probe,
	.remove = __devexit_p(adreno_remove),
	.suspend = kgsl_suspend_driver,
	.resume = kgsl_resume_driver,
	.id_table = adreno_id_table,
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_3D_NAME,
		.pm = &kgsl_pm_ops,
		.of_match_table = adreno_match_table,
	}
};

static int __init kgsl_3d_init(void)
{
	return platform_driver_register(&adreno_platform_driver);
}

static void __exit kgsl_3d_exit(void)
{
	platform_driver_unregister(&adreno_platform_driver);
}

module_init(kgsl_3d_init);
module_exit(kgsl_3d_exit);

MODULE_DESCRIPTION("3D Graphics driver");
MODULE_VERSION("1.2");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:kgsl_3d");
