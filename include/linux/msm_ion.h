#ifndef _LINUX_MSM_ION_H
#define _LINUX_MSM_ION_H

#include <linux/ion.h>

enum msm_ion_heap_types {
	ION_HEAP_TYPE_MSM_START = ION_HEAP_TYPE_CUSTOM + 1,
	ION_HEAP_TYPE_DMA = ION_HEAP_TYPE_MSM_START,
	ION_HEAP_TYPE_CP,
	ION_HEAP_TYPE_SECURE_DMA,
	ION_HEAP_TYPE_REMOVED,
};


enum ion_heap_ids {
	INVALID_HEAP_ID = -1,
	ION_CP_MM_HEAP_ID = 8,
	ION_CP_MFC_HEAP_ID = 12,
	ION_FBMEM_HEAP_ID = 13, 
	ION_CP_WB_HEAP_ID = 16, 
	ION_CAMERA_HEAP_ID = 20, 
	ION_SYSTEM_CONTIG_HEAP_ID = 21,
	ION_ADSP_HEAP_ID = 22,
	ION_PIL1_HEAP_ID = 23, 
	ION_SF_HEAP_ID = 24,
	ION_SYSTEM_HEAP_ID = 25,
	ION_PIL2_HEAP_ID = 26, 
	ION_QSECOM_HEAP_ID = 27,
	ION_AUDIO_HEAP_ID = 28,

	ION_MM_FIRMWARE_HEAP_ID = 29,

	ION_HEAP_ID_RESERVED = 31 
};

#define ION_IOMMU_HEAP_ID ION_SYSTEM_HEAP_ID
#define ION_HEAP_TYPE_IOMMU ION_HEAP_TYPE_SYSTEM

enum ion_fixed_position {
	NOT_FIXED,
	FIXED_LOW,
	FIXED_MIDDLE,
	FIXED_HIGH,
};

enum cp_mem_usage {
	VIDEO_BITSTREAM = 0x1,
	VIDEO_PIXEL = 0x2,
	VIDEO_NONPIXEL = 0x3,
	MAX_USAGE = 0x4,
	UNKNOWN = 0x7FFFFFFF,
};

#define ION_HEAP_CP_MASK		(1 << ION_HEAP_TYPE_CP)
#define ION_HEAP_TYPE_DMA_MASK         (1 << ION_HEAP_TYPE_DMA)

#define ION_FLAG_SECURE (1 << ION_HEAP_ID_RESERVED)

#define ION_FLAG_FORCE_CONTIGUOUS (1 << 30)

#define ION_FLAG_POOL_FORCE_ALLOC (1 << 16)

#define ION_SECURE ION_FLAG_SECURE
#define ION_FORCE_CONTIGUOUS ION_FLAG_FORCE_CONTIGUOUS

#define ION_HEAP(bit) (1 << (bit))

#define ION_FBMEM_HEAP_NAME	"fbmem"
#define ION_ADSP_HEAP_NAME	"adsp"
#define ION_SYSTEM_HEAP_NAME	"system"
#define ION_VMALLOC_HEAP_NAME	ION_SYSTEM_HEAP_NAME
#define ION_KMALLOC_HEAP_NAME	"kmalloc"
#define ION_AUDIO_HEAP_NAME	"audio"
#define ION_SF_HEAP_NAME	"sf"
#define ION_MM_HEAP_NAME	"mm"
#define ION_CAMERA_HEAP_NAME	"camera_preview"
#define ION_IOMMU_HEAP_NAME	"iommu"
#define ION_MFC_HEAP_NAME	"mfc"
#define ION_WB_HEAP_NAME	"wb"
#define ION_MM_FIRMWARE_HEAP_NAME	"mm_fw"
#define ION_PIL1_HEAP_NAME  "pil_1"
#define ION_PIL2_HEAP_NAME  "pil_2"
#define ION_QSECOM_HEAP_NAME	"qsecom"

#define ION_SET_CACHED(__cache)		(__cache | ION_FLAG_CACHED)
#define ION_SET_UNCACHED(__cache)	(__cache & ~ION_FLAG_CACHED)

#define ION_IS_CACHED(__flags)	((__flags) & ION_FLAG_CACHED)

#ifdef __KERNEL__

#define ION_IOMMU_UNMAP_DELAYED 1

#define ION_UNSECURE_DELAYED	1

struct ion_cp_heap_pdata {
	enum ion_permission_type permission_type;
	unsigned int align;
	ion_phys_addr_t secure_base; 
	size_t secure_size; 
	int is_cma;
	enum ion_fixed_position fixed_position;
	int iommu_map_all;
	int iommu_2x_map_domain;
	int (*request_region)(void *);
	int (*release_region)(void *);
	void *(*setup_region)(void);
	enum ion_memory_types memory_type;
	int allow_nonsecure_alloc;
};

struct ion_co_heap_pdata {
	int adjacent_mem_id;
	unsigned int align;
	enum ion_fixed_position fixed_position;
	int (*request_region)(void *);
	int (*release_region)(void *);
	void *(*setup_region)(void);
	enum ion_memory_types memory_type;
};

struct ion_cma_pdata {
	unsigned long default_prefetch_size;
};

#ifdef CONFIG_ION

struct ion_client *msm_ion_client_create(unsigned int heap_mask,
					const char *name);

int ion_handle_get_flags(struct ion_client *client, struct ion_handle *handle,
				unsigned long *flags);


int ion_map_iommu(struct ion_client *client, struct ion_handle *handle,
			int domain_num, int partition_num, unsigned long align,
			unsigned long iova_length, unsigned long *iova,
			unsigned long *buffer_size,
			unsigned long flags, unsigned long iommu_flags);



int ion_handle_get_size(struct ion_client *client, struct ion_handle *handle,
			unsigned long *size);

void ion_unmap_iommu(struct ion_client *client, struct ion_handle *handle,
			int domain_num, int partition_num);


int ion_secure_heap(struct ion_device *dev, int heap_id, int version,
			void *data);

int ion_unsecure_heap(struct ion_device *dev, int heap_id, int version,
			void *data);

int msm_ion_do_cache_op(struct ion_client *client, struct ion_handle *handle,
			void *vaddr, unsigned long len, unsigned int cmd);

int msm_ion_secure_heap(int heap_id);

int msm_ion_unsecure_heap(int heap_id);

int msm_ion_secure_heap_2_0(int heap_id, enum cp_mem_usage usage);

int msm_ion_unsecure_heap_2_0(int heap_id, enum cp_mem_usage usage);

int msm_ion_secure_buffer(struct ion_client *client, struct ion_handle *handle,
				enum cp_mem_usage usage, int flags);

int msm_ion_unsecure_buffer(struct ion_client *client,
				struct ion_handle *handle);

uintptr_t msm_ion_heap_meminfo(const bool is_total);
#else
static inline struct ion_client *msm_ion_client_create(unsigned int heap_mask,
					const char *name)
{
	return ERR_PTR(-ENODEV);
}

static inline int ion_map_iommu(struct ion_client *client,
			struct ion_handle *handle, int domain_num,
			int partition_num, unsigned long align,
			unsigned long iova_length, unsigned long *iova,
			unsigned long *buffer_size,
			unsigned long flags,
			unsigned long iommu_flags)
{
	return -ENODEV;
}

static inline int ion_handle_get_size(struct ion_client *client,
				struct ion_handle *handle, unsigned long *size)
{
	return -ENODEV;
}

static inline void ion_unmap_iommu(struct ion_client *client,
			struct ion_handle *handle, int domain_num,
			int partition_num)
{
	return;
}

static inline int ion_secure_heap(struct ion_device *dev, int heap_id,
					int version, void *data)
{
	return -ENODEV;

}

static inline int ion_unsecure_heap(struct ion_device *dev, int heap_id,
					int version, void *data)
{
	return -ENODEV;
}

static inline void ion_mark_dangling_buffers_locked(struct ion_device *dev)
{
}

static inline int msm_ion_do_cache_op(struct ion_client *client,
			struct ion_handle *handle, void *vaddr,
			unsigned long len, unsigned int cmd)
{
	return -ENODEV;
}

static inline int msm_ion_secure_heap(int heap_id)
{
	return -ENODEV;

}

static inline int msm_ion_unsecure_heap(int heap_id)
{
	return -ENODEV;
}

static inline int msm_ion_secure_heap_2_0(int heap_id, enum cp_mem_usage usage)
{
	return -ENODEV;
}

static inline int msm_ion_unsecure_heap_2_0(int heap_id,
					enum cp_mem_usage usage)
{
	return -ENODEV;
}

static inline int msm_ion_secure_buffer(struct ion_client *client,
					struct ion_handle *handle,
					enum cp_mem_usage usage,
					int flags)
{
	return -ENODEV;
}

static inline int msm_ion_unsecure_buffer(struct ion_client *client,
					struct ion_handle *handle)
{
	return -ENODEV;
}

static inline uintptr_t msm_ion_heap_meminfo(const bool is_total)
{
	return 0;
}
#endif 

#endif 

struct ion_flush_data {
	struct ion_handle *handle;
	int fd;
	void *vaddr;
	unsigned int offset;
	unsigned int length;
};

struct ion_prefetch_data {
       int heap_id;
       unsigned long len;
};

struct ion_client_name_data {
	size_t len;
	const char *name;
};

#define ION_IOC_MSM_MAGIC 'M'

#define ION_IOC_CLEAN_CACHES	_IOWR(ION_IOC_MSM_MAGIC, 0, \
						struct ion_flush_data)
#define ION_IOC_INV_CACHES	_IOWR(ION_IOC_MSM_MAGIC, 1, \
						struct ion_flush_data)
#define ION_IOC_CLEAN_INV_CACHES	_IOWR(ION_IOC_MSM_MAGIC, 2, \
						struct ion_flush_data)

#define ION_IOC_PREFETCH               _IOWR(ION_IOC_MSM_MAGIC, 3, \
                                               struct ion_prefetch_data)

#define ION_IOC_DRAIN                  _IOWR(ION_IOC_MSM_MAGIC, 4, \
                                               struct ion_prefetch_data)

#define ION_IOC_CLIENT_DEBUG_NAME	_IOWR(ION_IOC_MSM_MAGIC, 5, \
						struct ion_client_name_data)
#endif
