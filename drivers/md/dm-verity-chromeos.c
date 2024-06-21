/*
 * Copyright (C) 2010 The Chromium OS Authors <chromium-os-dev@chromium.org>
 *                    All Rights Reserved.
 *
 * This file is released under the GPL.
 */
/*
 * Implements a Chrome OS platform specific error handler.
 * When verity detects an invalid block, this error handling will
 * attempt to corrupt the kernel boot image. On reboot, the bios will
 * detect the kernel corruption and switch to the alternate kernel
 * and root file system partitions.
 *
 * Assumptions:
 * 1. Partitions are specified on the command line using uuid.
 * 2. The kernel partition is the partition number is one less
 *    than the root partition number.
 */
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/device.h>
#include <linux/device-mapper.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mount.h>
#include <linux/notifier.h>
#include <linux/string.h>
#include <asm/page.h>

#include "dm-verity.h"

#define DM_MSG_PREFIX "verity-chromeos"
#define DMVERROR "DMVERROR"

static void chromeos_invalidate_kernel_endio(struct bio *bio)
{
	if (bio->bi_status) {
		DMERR("%s: bio operation failed (status=0x%x)", __func__,
		      bio->bi_status);
	}
	complete(bio->bi_private);
}

static int chromeos_invalidate_kernel_submit(struct bio *bio,
					     struct block_device *bdev,
					     unsigned int op,
					     unsigned int op_flags,
					     struct page *page)
{
	DECLARE_COMPLETION_ONSTACK(wait);
	unsigned int block_size = bdev_logical_block_size(bdev);

	if (block_size > page_size(page))
		panic("dm-verity failed to override signature");

	bio->bi_private = &wait;
	bio->bi_end_io = chromeos_invalidate_kernel_endio;
	bio_set_dev(bio, bdev);

	bio->bi_iter.bi_sector = 0;
	bio->bi_vcnt = 1;
	bio->bi_iter.bi_idx = 0;
	bio->bi_iter.bi_size = block_size;
	bio->bi_iter.bi_bvec_done = 0;
	bio->bi_opf =  op | op_flags;
	bio->bi_io_vec[0].bv_page = page;
	bio->bi_io_vec[0].bv_len = block_size;
	bio->bi_io_vec[0].bv_offset = 0;

	submit_bio(bio);
	/* Wait up to 2 seconds for completion or fail. */
	if (!wait_for_completion_timeout(&wait, msecs_to_jiffies(2000)))
		return -1;
	return 0;
}

static dev_t get_boot_dev_from_root_dev(struct block_device *root_bdev)
{
	/* Very basic sanity checking. This should be better. */
	if (!root_bdev || MAJOR(root_bdev->bd_dev) == 254 ||
	    root_bdev->bd_partno <= 1) {
		return 0;
	}
	return MKDEV(MAJOR(root_bdev->bd_dev), MINOR(root_bdev->bd_dev) - 1);
}

static char kern_guid[48];

/*
 * get_boot_dev is based on dm_get_device_by_uuid in dm_bootcache.
 *
 * This function is marked __ref because it calls the __init marked
 * early_lookup_bdev when called from the early boot code.
 */
static dev_t __ref get_boot_dev(void)
{
	const char partuuid[] = "PARTUUID=";
	char uuid[sizeof(partuuid) + 36];
	char *uuid_str;
	dev_t devt;

	if (!strlen(kern_guid)) {
		DMERR("Couldn't get uuid, try root dev");
		return 0;
	}

	if (strncmp(kern_guid, partuuid, strlen(partuuid))) {
		/* Not prefixed with "PARTUUID=", so add it */
		strcpy(uuid, partuuid);
		strlcat(uuid, kern_guid, sizeof(uuid));
		uuid_str = uuid;
	} else {
		uuid_str = kern_guid;
	}
        if (early_lookup_bdev(uuid_str, &devt)) {
                DMDEBUG("No matching partition for GUID: %s", uuid_str);
                return 0;
        }
	return devt;
}

/*
 * Invalidate the kernel which corresponds to the root block device.
 *
 * This function stamps DMVERROR on the beginning of the kernel partition.
 *
 * The kernel_guid commandline parameter is used to find the kernel partition
 *  number.
 * If that fails, the kernel partition is found by subtracting 1 from
 *  the root partition.
 * The DMVERROR string is stamped over only the CHROMEOS string at the
 *  beginning of the kernel blob, leaving the rest of it intact.
 */
static int chromeos_invalidate_kernel_bio(struct block_device *root_bdev)
{
	int ret = 0;
	struct block_device *bdev;
	struct bio *bio;
	struct page *page;
	dev_t devt;
	blk_mode_t dev_mode;

	devt = get_boot_dev();
	if (!devt) {
		devt = get_boot_dev_from_root_dev(root_bdev);
		if (!devt)
			return -EINVAL;
	}

	/* First we open the device for reading. */
	dev_mode = BLK_OPEN_READ | BLK_OPEN_EXCL;
	bdev = blkdev_get_by_dev(devt, dev_mode,
				 chromeos_invalidate_kernel_bio, NULL);
	if (IS_ERR(bdev)) {
		DMERR("invalidate_kernel: could not open device for reading");
		dev_mode = 0;
		ret = -1;
		goto failed_to_read;
	}

	bio = bio_alloc(NULL, 1, 0, GFP_NOIO);
	if (!bio) {
		ret = -1;
		goto failed_bio_alloc;
	}

	page = alloc_page(GFP_NOIO);
	if (!page) {
		ret = -ENOMEM;
		goto failed_to_alloc_page;
	}

	if (chromeos_invalidate_kernel_submit(bio, bdev,
					      REQ_OP_READ,
					      REQ_SYNC,
					      page)) {
		ret = -1;
		goto failed_to_submit_read;
	}

	/* We have a page. Let's make sure it looks right. */
	if (memcmp("CHROMEOS", page_address(page), 8)) {
		DMERR("invalidate_kernel called on non-kernel partition");
		ret = -EINVAL;
		goto invalid_header;
	} else {
		DMERR("invalidate_kernel: found CHROMEOS kernel partition");
	}

	/* Stamp it and rewrite */
	memcpy(page_address(page), DMVERROR, strlen(DMVERROR));

	/* The block dev was being changed on read. Let's reopen here. */
	blkdev_put(bdev, chromeos_invalidate_kernel_bio);
	dev_mode = BLK_OPEN_WRITE | BLK_OPEN_EXCL;
	bdev = blkdev_get_by_dev(devt, dev_mode,
				 chromeos_invalidate_kernel_bio, NULL);
	if (IS_ERR(bdev)) {
		DMERR("invalidate_kernel: could not open device for writing");
		dev_mode = 0;
		ret = -1;
		goto failed_to_write;
	}

	/* We re-use the same bio to do the write after the read. Need to reset
	 * it to initialize bio->bi_remaining.
	 */
	bio_reset(bio, NULL, 0);

	/*
	 * Request write operation with REQ_FUA flag to ensure that I/O
	 * completion for the write is signaled only after the data has been
	 * committed to non-volatile storage.
	 */
	if (chromeos_invalidate_kernel_submit(bio, bdev, REQ_OP_WRITE,
					      REQ_SYNC | REQ_FUA, page)) {
		ret = -1;
		goto failed_to_submit_write;
	}

	DMERR("invalidate_kernel: completed.");
	ret = 0;
failed_to_submit_write:
failed_to_write:
invalid_header:
	__free_page(page);
failed_to_submit_read:
	/* Technically, we'll leak a page with the pending bio, but
	 *  we're about to panic so it's safer to do the panic() we expect.
	 */
failed_to_alloc_page:
	bio_put(bio);
failed_bio_alloc:
	if (dev_mode)
		blkdev_put(bdev, chromeos_invalidate_kernel_bio);
failed_to_read:
	return ret;
}

static int error_handler(struct notifier_block *nb, unsigned long transient,
			 void *opaque_err)
{
	struct dm_verity_error_state *err =
		(struct dm_verity_error_state *) opaque_err;
	err->behavior = DM_VERITY_ERROR_BEHAVIOR_PANIC;
	if (transient)
		return 0;

	/* Mark the kernel partition as invalid. */
	chromeos_invalidate_kernel_bio(err->dev);
	return 0;
}

static struct notifier_block chromeos_nb = {
	.notifier_call = &error_handler,
	.next = NULL,
	.priority = 1,
};

static int __init dm_verity_chromeos_init(void)
{
	int r;

	r = dm_verity_register_error_notifier(&chromeos_nb);
	if (r < 0)
		DMERR("failed to register handler: %d", r);
	else
		DMINFO("dm-verity-chromeos registered");
	return r;
}

static void __exit dm_verity_chromeos_exit(void)
{
	dm_verity_unregister_error_notifier(&chromeos_nb);
}

module_init(dm_verity_chromeos_init);
module_exit(dm_verity_chromeos_exit);

MODULE_AUTHOR("Will Drewry <wad@chromium.org>");
MODULE_DESCRIPTION("chromeos-specific error handler for dm-verity");
MODULE_LICENSE("GPL");

/* Declare parameter with no module prefix */
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX	""
module_param_string(kern_guid, kern_guid, sizeof(kern_guid), 0);
