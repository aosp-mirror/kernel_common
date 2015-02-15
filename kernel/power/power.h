#include <linux/suspend.h>
#include <linux/suspend_ioctls.h>
#include <linux/utsname.h>
#include <linux/freezer.h>

struct swsusp_info {
	struct new_utsname	uts;
	u32			version_code;
	unsigned long		num_physpages;
	int			cpus;
	unsigned long		image_pages;
	unsigned long		pages;
	unsigned long		size;
} __attribute__((aligned(PAGE_SIZE)));

#ifdef CONFIG_HIBERNATION
extern void __init hibernate_reserved_size_init(void);
extern void __init hibernate_image_size_init(void);

#ifdef CONFIG_ARCH_HIBERNATION_HEADER
#define MAX_ARCH_HEADER_SIZE	(sizeof(struct new_utsname) + 4)

extern int arch_hibernation_header_save(void *addr, unsigned int max_size);
extern int arch_hibernation_header_restore(void *addr);

static inline int init_header_complete(struct swsusp_info *info)
{
	return arch_hibernation_header_save(info, MAX_ARCH_HEADER_SIZE);
}

static inline char *check_image_kernel(struct swsusp_info *info)
{
	return arch_hibernation_header_restore(info) ?
			"architecture specific data" : NULL;
}
#endif 

#define PAGES_FOR_IO	((4096 * 1024) >> PAGE_SHIFT)

#define SPARE_PAGES	((1024 * 1024) >> PAGE_SHIFT)

extern bool freezer_test_done;

extern int hibernation_snapshot(int platform_mode);
extern int hibernation_restore(int platform_mode);
extern int hibernation_platform_enter(void);

#else 

static inline void hibernate_reserved_size_init(void) {}
static inline void hibernate_image_size_init(void) {}
#endif 

extern int pfn_is_nosave(unsigned long);

#define power_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

#define power_ro_attr(_name)			\
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0444,			\
	},					\
	.show	= _name##_show,			\
	.store	= NULL,				\
}

extern unsigned long image_size;
extern unsigned long reserved_size;
extern int in_suspend;
extern dev_t swsusp_resume_device;
extern sector_t swsusp_resume_block;

extern asmlinkage int swsusp_arch_suspend(void);
extern asmlinkage int swsusp_arch_resume(void);

extern int create_basic_memory_bitmaps(void);
extern void free_basic_memory_bitmaps(void);
extern int hibernate_preallocate_memory(void);


struct snapshot_handle {
	unsigned int	cur;	
	void		*buffer;	
	int		sync_read;	
};

#define data_of(handle)	((handle).buffer)

extern unsigned int snapshot_additional_pages(struct zone *zone);
extern unsigned long snapshot_get_image_size(void);
extern int snapshot_read_next(struct snapshot_handle *handle);
extern int snapshot_write_next(struct snapshot_handle *handle);
extern void snapshot_write_finalize(struct snapshot_handle *handle);
extern int snapshot_image_loaded(struct snapshot_handle *handle);

extern atomic_t snapshot_device_available;

extern sector_t alloc_swapdev_block(int swap);
extern void free_all_swap_pages(int swap);
extern int swsusp_swap_in_use(void);

#define SF_PLATFORM_MODE	1
#define SF_NOCOMPRESS_MODE	2
#define SF_CRC32_MODE	        4

extern int swsusp_check(void);
extern void swsusp_free(void);
extern int swsusp_read(unsigned int *flags_p);
extern int swsusp_write(unsigned int flags);
extern void swsusp_close(fmode_t);

extern struct block_device *hib_resume_bdev;

extern int hib_bio_read_page(pgoff_t page_off, void *addr,
		struct bio **bio_chain);
extern int hib_bio_write_page(pgoff_t page_off, void *addr,
		struct bio **bio_chain);
extern int hib_wait_on_bio_chain(struct bio **bio_chain);

struct timeval;
extern void swsusp_show_speed(struct timeval *, struct timeval *,
				unsigned int, char *);

#ifdef CONFIG_SUSPEND
extern const char *const pm_states[];

extern bool valid_state(suspend_state_t state);
extern int suspend_devices_and_enter(suspend_state_t state);
#else 
static inline int suspend_devices_and_enter(suspend_state_t state)
{
	return -ENOSYS;
}
static inline bool valid_state(suspend_state_t state) { return false; }
#endif 

#ifdef CONFIG_PM_TEST_SUSPEND
extern void suspend_test_start(void);
extern void suspend_test_finish(const char *label);
#else 
static inline void suspend_test_start(void) {}
static inline void suspend_test_finish(const char *label) {}
#endif 

#ifdef CONFIG_PM_SLEEP
extern int pm_notifier_call_chain(unsigned long val);
#endif

#ifdef CONFIG_HIGHMEM
int restore_highmem(void);
#else
static inline unsigned int count_highmem_pages(void) { return 0; }
static inline int restore_highmem(void) { return 0; }
#endif

enum {
	
	TEST_NONE,
	TEST_CORE,
	TEST_CPUS,
	TEST_PLATFORM,
	TEST_DEVICES,
	TEST_FREEZER,
	
	__TEST_AFTER_LAST
};

#define TEST_FIRST	TEST_NONE
#define TEST_MAX	(__TEST_AFTER_LAST - 1)

extern int pm_test_level;

extern void suspend_sys_sync_queue(void);
extern int suspend_sys_sync_wait(void);

#ifdef CONFIG_SUSPEND_FREEZER
static inline int suspend_freeze_processes(void)
{
	int error;

	error = suspend_sys_sync_wait();
	if (error)
		return error;

	error = freeze_processes();
	if (error)
		return error;

	error = freeze_kernel_threads();
	if (error)
		thaw_processes();

	return error;
}

static inline void suspend_thaw_processes(void)
{
	thaw_processes();
}
#else
static inline int suspend_freeze_processes(void)
{
	return 0;
}

static inline void suspend_thaw_processes(void)
{
}
#endif

#ifdef CONFIG_PM_AUTOSLEEP

extern int pm_autosleep_init(void);
extern int pm_autosleep_lock(void);
extern void pm_autosleep_unlock(void);
extern suspend_state_t pm_autosleep_state(void);
extern int pm_autosleep_set_state(suspend_state_t state);

#else 

static inline int pm_autosleep_init(void) { return 0; }
static inline int pm_autosleep_lock(void) { return 0; }
static inline void pm_autosleep_unlock(void) {}
static inline suspend_state_t pm_autosleep_state(void) { return PM_SUSPEND_ON; }

#endif 

#ifdef CONFIG_PM_WAKELOCKS

extern ssize_t pm_show_wakelocks(char *buf, bool show_active);
extern int pm_wake_lock(const char *buf);
extern int pm_wake_unlock(const char *buf);

#endif 

