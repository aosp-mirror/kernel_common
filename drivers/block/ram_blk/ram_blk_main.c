// SPDX-License-Identifier: GPL-2.0
/*
 * Ram Block Driver
 *
 * This block device driver stores the data in memory. It performs the
 * following tasks:
 *
 *  - Register a block I/O device.
 *  - Create and initialize a gendisk.
 *  - Process struct request_queue.
 *  - Allocates memory to store the data using vmalloc.
 */
#define pr_fmt(fmt)  "%s: %s: " fmt, KBUILD_MODNAME, __func__

#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/numa.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/vmalloc.h>

#define NR_SECTORS			131072
#define RAMDISK_SECTOR_SIZE		512

#define RAM_BLKDEV_MAJOR		240
#define RAM_BLKDEV_MINOR		1
#define RAM_BLKDEV_NAME		"ramdiskblockdev"
#define GENDISK_NAME			"genramdisk"

static struct ram_block_dev {
	struct gendisk *gd;
	struct blk_mq_tag_set tag_set;
	struct request_queue *queue;

	// Fields that are part of the ramdisk.
	uint8_t *ramdisk;
	size_t size;
	spinlock_t lock;

	// Fields to protect access to ramdisk
	struct kref refcount;
} ram_blk_dev;

inline void pr_request(struct request *rq)
{
	switch (rq_data_dir(rq)) {
	case READ:
		pr_info("READ direction request");
		break;
	case WRITE:
		pr_info("WRITE direction request");
		break;
	};

	pr_info("\tblk_rq_pos = __sector =  %llu\n", blk_rq_pos(rq));
	pr_info("\tblk_rq_bytes = __data_len = %d\n", blk_rq_bytes(rq));
	pr_info("\tblk_rq_cur_bytes = %d\n", blk_rq_cur_bytes(rq));

	switch (rq->state) {
	case MQ_RQ_IDLE:
		pr_info("\tmq_rq_state = MQ_RQ_IDLE");
		break;
	case MQ_RQ_IN_FLIGHT:
		pr_info("\tmq_rq_state = MQ_RQ_IN_FLIGHT");
		break;
	case MQ_RQ_COMPLETE:
		pr_info("\tmq_rq_state = MQ_RQ_COMPLETE");
		break;
	};
}

static int exec_request(struct request *rq)
{
	struct bio_vec bvec;
	struct req_iterator iter;
	int i = 0;

	pr_info("Processing segments in bio");

	rq_for_each_segment(bvec, rq, iter) {
		sector_t sector = iter.iter.bi_sector;
		unsigned long bv_offset = bvec.bv_offset;
		unsigned long offset = (sector * RAMDISK_SECTOR_SIZE);
		size_t len = bvec.bv_len;
		char *buffer = kmap_local_page(bvec.bv_page);
		int dir = bio_data_dir(iter.bio);
		unsigned long flags;

		pr_info("%s Segment %d", (dir == READ ? "READ" : "WRITE"), i++);
		pr_info("   sector:    %lld", sector);
		pr_info("   bv_offset: %ld", bv_offset);
		pr_info("   offset:    %ld", offset);
		pr_info("   len:       %ld", len);

		if ((offset + len) > ram_blk_dev.size) {
			pr_err("Ramdisk = %ld, offset = %ld, len = %ld",
				ram_blk_dev.size, offset, len);
			return BLK_STS_IOERR;
		}

		spin_lock_irqsave(&ram_blk_dev.lock, flags);

		switch (dir) {
		case READ:
			memcpy(buffer + bv_offset, ram_blk_dev.ramdisk + offset, len);
			break;
		case WRITE:
			memcpy(ram_blk_dev.ramdisk + offset, buffer + bv_offset, len);
			break;
		};

		spin_unlock_irqrestore(&ram_blk_dev.lock, flags);

		kunmap_local(buffer);
	}

	return BLK_STS_OK;
}

static void release_block_device(struct kref *ref)
{
	// Note: There could be race condition when the block device is released.
	struct ram_block_dev *dev = container_of(ref, struct ram_block_dev, refcount);

	pr_info("Unregistering block I/O device");
	unregister_blkdev(RAM_BLKDEV_MAJOR, RAM_BLKDEV_NAME);

	pr_info("Deleting gendisk");
	del_gendisk(dev->gd);

	pr_info("Destroying queue");

	blk_cleanup_queue(dev->queue);

	pr_info("Freeing tag_set");
	blk_mq_free_tag_set(&dev->tag_set);

	pr_info("Freeing ramdisk");
	vfree(dev->ramdisk);
}

/**
 * Queue a new request from block IO.
 * To process the request, use these methods:
 *
 * - blk_mq_start_request(): Before starting the request, call this function.
 * - blk_mq_requeue_request(): To send again the request to the queue.
 * - blk_mq_end_request(): When the request is completed, call this function
 *   to notify the upper layers.
 */
static blk_status_t blk_mq_ops_ram_queue_rq(struct blk_mq_hw_ctx *hctx,
									const struct blk_mq_queue_data *bd)
{
	struct request *rq = bd->rq;
	int status;

	pr_info("Queuing request\n");

	// Incrementing the kref counter.
	kref_get(&ram_blk_dev.refcount);

	if (bd->last)
		pr_info("This is the 'last' request in the queue");

	pr_request(rq);

	// Start the request.
	blk_mq_start_request(rq);

	if (blk_rq_is_passthrough(rq)) {
		pr_info("Passthrough request not supported\n");
		blk_mq_end_request(rq, BLK_STS_IOERR);
		goto out;
	}

	status = exec_request(rq);
	blk_mq_end_request(rq, status);

out:
	// Decrementing the kref counter.
	kref_put(&ram_blk_dev.refcount, release_block_device);
	return BLK_STS_OK;
}

/*
 * Multi-queue operations.
 */
static const struct blk_mq_ops ram_mq_queue_ops = {
	.queue_rq = blk_mq_ops_ram_queue_rq,
};

/**
 * When the file system is mounted, this method is called.
 */
static int ram_block_open(struct block_device *blk, fmode_t mode)
{
	pr_info("Open block device\n");

	return 0;
}

/**
 * When the file system is unmounted, this method is called.
 */
static void ram_block_release(struct gendisk *disk, fmode_t mode)
{
	pr_info("Release disk %s\n", disk->disk_name);
}

static const struct block_device_operations ram_block_ops = {
	.owner = THIS_MODULE,
	.open = ram_block_open,
	.release = ram_block_release
};

inline int init_blk_mq_tag_set(struct blk_mq_tag_set *tag_set)
{
	tag_set->ops = &ram_mq_queue_ops;
	tag_set->nr_hw_queues = 1;
	tag_set->queue_depth = 16;
	tag_set->numa_node = NUMA_NO_NODE;
	tag_set->cmd_size = 0;
	tag_set->flags = BLK_MQ_F_SHOULD_MERGE;

	pr_info("Before blk_mq_alloc_tag_set()");
	return blk_mq_alloc_tag_set(tag_set);
}

inline int init_request_queue(struct ram_block_dev *dev)
{
	pr_info("Before blk_mq_init_queue()");
	dev->queue = blk_mq_init_queue(&dev->tag_set);
	if (IS_ERR(dev->queue))
		return PTR_ERR(dev->queue);

	pr_info("Before blk_queue_physical_block_size()");
	blk_queue_physical_block_size(dev->queue, RAMDISK_SECTOR_SIZE);
	blk_queue_logical_block_size(dev->queue, RAMDISK_SECTOR_SIZE);

	// https://elixir.bootlin.com/linux/v6.6.2/source/block/blk-settings.c#L284
	blk_queue_max_segment_size(dev->queue, 512);

	return 0;
}

inline int init_gendisk(struct ram_block_dev *dev)
{
	pr_info("Before blk_mq_alloc_disk()");
	dev->gd = blk_mq_alloc_disk(&dev->tag_set, NULL);
	if (IS_ERR(dev->gd))
		return PTR_ERR(dev->gd);

	dev->queue->disk = dev->gd;
	snprintf(dev->gd->disk_name, DISK_NAME_LEN, GENDISK_NAME);
	dev->gd->major = RAM_BLKDEV_MAJOR;
	dev->gd->first_minor = 0;
	dev->gd->minors = 1;
	dev->gd->flags |= GENHD_FL_NO_PART;
	dev->gd->fops = &ram_block_ops;
	dev->gd->queue = dev->queue;
	dev->gd->private_data = dev;

	// The capacity of the block device is NR_SECTORS * RAMDISK_SECTOR_SIZE
	pr_info("Before set_capacity()");
	set_capacity(dev->gd, NR_SECTORS);

	// This block device is not rotations and it is synchronous.
	blk_queue_flag_set(QUEUE_FLAG_NONROT, dev->gd->queue);
	//blk_queue_flag_set(QUEUE_FLAG_SYNCHRONOUS, dev->gd->queue);

	return 0;
}

static int init_ramdisk(struct ram_block_dev *dev)
{
	dev->size = NR_SECTORS * RAMDISK_SECTOR_SIZE;
	dev->ramdisk = vmalloc(dev->size);
	if (dev->ramdisk == NULL)
		return -ENOMEM;

	return 0;
}

static int setup_block_dev(struct ram_block_dev *dev)
{
	int status;

	spin_lock_init(&dev->lock);
	// Initializes the kref and set the counter to 1.
	kref_init(&dev->refcount);

	status = init_ramdisk(dev);
	if (status < 0)
		return status;

	status = init_blk_mq_tag_set(&dev->tag_set);
	if (status < 0)
		goto err_blk_mq_tag;

	status = init_request_queue(dev);
	if (status < 0)
		goto err_blk_init;

	status = init_gendisk(dev);
	if (status < 0)
		goto err_blk_init;

	pr_info("Before add_disk()");
	status = add_disk(dev->gd);
	if (status < 0)
		goto err_blk_init;

	return 0;

err_blk_init:
	blk_mq_free_tag_set(&dev->tag_set);
err_blk_mq_tag:
	vfree(dev->ramdisk);

	return status;
}

static int __init ram_block_init(void)
{
	int status;

	// Register the block device.
	pr_info("Before register_blkdev()");
	status = register_blkdev(RAM_BLKDEV_MAJOR, RAM_BLKDEV_NAME);
	if (status < 0) {
		pr_err("Unable to register '%s'", RAM_BLKDEV_NAME);
		return status;
	}

	status = setup_block_dev(&ram_blk_dev);
	if (status < 0) {
		unregister_blkdev(RAM_BLKDEV_MAJOR, RAM_BLKDEV_NAME);
		return status;
	}

	pr_info("Make sure to create the node:");
	pr_info("    mknod /dev/ramdiskblockdev b 240 0");

	return 0;
}

static void __exit ram_block_exit(void)
{
	unsigned int countrefs;

	pr_info("Exiting ram block driver");

	countrefs = kref_read(&ram_blk_dev.refcount);
		pr_info("ram_blk_dev has [%d] references(s)!", countrefs);

	kref_put(&ram_blk_dev.refcount, release_block_device);
}

module_init(ram_block_init);
module_exit(ram_block_exit);

MODULE_AUTHOR("Juan Yescas");
MODULE_DESCRIPTION("Ram Block Driver");
MODULE_LICENSE("GPL v2");

