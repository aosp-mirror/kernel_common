/*
 * Functions related to generic helpers functions
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/scatterlist.h>

#include "blk.h"

struct bio_batch {
	atomic_t		done;
	unsigned long		flags;
	struct completion	*wait;
};

static void bio_batch_end_io(struct bio *bio, int err)
{
	struct bio_batch *bb = bio->bi_private;

	if (err && (err != -EOPNOTSUPP))
		clear_bit(BIO_UPTODATE, &bb->flags);
	if (atomic_dec_and_test(&bb->done))
		complete(bb->wait);
	bio_put(bio);
}

/**
 * blkdev_issue_discard - queue a discard
 * @bdev:	blockdev to issue discard for
 * @sector:	start sector
 * @nr_sects:	number of sectors to discard
 * @gfp_mask:	memory allocation flags (for bio_alloc)
 * @flags:	BLKDEV_IFL_* flags to control behaviour
 *
 * Description:
 *    Issue a discard request for the sectors in question.
 */
int blkdev_issue_discard(struct block_device *bdev, sector_t sector,
		sector_t nr_sects, gfp_t gfp_mask, unsigned long flags)
{
	DECLARE_COMPLETION_ONSTACK(wait);
	struct request_queue *q = bdev_get_queue(bdev);
	int type = REQ_WRITE | REQ_DISCARD;
	unsigned int max_discard_sectors;
	struct bio_batch bb;
	struct bio *bio;
	int ret = 0;

	if (!q)
		return -ENXIO;

	if (!blk_queue_discard(q))
		return -EOPNOTSUPP;

	/*
	 * Ensure that max_discard_sectors is of the proper
	 * granularity
	 */
	max_discard_sectors = min(q->limits.max_discard_sectors, UINT_MAX >> 9);
	if (unlikely(!max_discard_sectors)) {
		/* Avoid infinite loop below. Being cautious never hurts. */
		return -EOPNOTSUPP;
	} else if (q->limits.discard_granularity) {
		unsigned int disc_sects = q->limits.discard_granularity >> 9;

		max_discard_sectors &= ~(disc_sects - 1);
	}

	if (flags & BLKDEV_DISCARD_SECURE) {
		if (!blk_queue_secdiscard(q))
			return -EOPNOTSUPP;
		type |= REQ_SECURE;
	}

	atomic_set(&bb.done, 1);
	bb.flags = 1 << BIO_UPTODATE;
	bb.wait = &wait;

	while (nr_sects) {
		bio = bio_alloc(gfp_mask, 1);
		if (!bio) {
			ret = -ENOMEM;
			break;
		}

		bio->bi_sector = sector;
		bio->bi_end_io = bio_batch_end_io;
		bio->bi_bdev = bdev;
		bio->bi_private = &bb;

		if (nr_sects > max_discard_sectors) {
			bio->bi_size = max_discard_sectors << 9;
			nr_sects -= max_discard_sectors;
			sector += max_discard_sectors;
		} else {
			bio->bi_size = nr_sects << 9;
			nr_sects = 0;
		}

		atomic_inc(&bb.done);
		submit_bio(type, bio);
	}

	/* Wait for bios in-flight */
	if (!atomic_dec_and_test(&bb.done))
		wait_for_completion(&wait);

	if (!test_bit(BIO_UPTODATE, &bb.flags))
		ret = -EIO;

	return ret;
}
EXPORT_SYMBOL(blkdev_issue_discard);

/**
 * blkdev_issue_sanitize - queue a sanitize request
 * @bdev:	blockdev to issue sanitize for
 * @gfp_mask:	memory allocation flags (for bio_alloc)
 *
 * Description:
 *    Issue a sanitize request for the specified block device
 */
int blkdev_issue_sanitize(struct block_device *bdev, gfp_t gfp_mask)
{
	DECLARE_COMPLETION_ONSTACK(wait);
	struct request_queue *q = bdev_get_queue(bdev);
	int type = REQ_WRITE | REQ_SANITIZE;
	struct bio_batch bb;
	struct bio *bio;
	int ret = 0;

	if (!q)
		return -ENXIO;

	if (!blk_queue_sanitize(q)) {
		pr_err("%s - card doesn't support sanitize", __func__);
		return -EOPNOTSUPP;
	}

	bio = bio_alloc(gfp_mask, 1);
	if (!bio)
		return -ENOMEM;

	atomic_set(&bb.done, 1);
	bb.flags = 1 << BIO_UPTODATE;
	bb.wait = &wait;

	bio->bi_end_io = bio_batch_end_io;
	bio->bi_bdev = bdev;
	bio->bi_private = &bb;

	atomic_inc(&bb.done);
	submit_bio(type, bio);

	/* Wait for bios in-flight */
	if (!atomic_dec_and_test(&bb.done))
		wait_for_completion(&wait);

	if (!test_bit(BIO_UPTODATE, &bb.flags))
		ret = -EIO;

	return ret;
}
EXPORT_SYMBOL(blkdev_issue_sanitize);

/**
 * blkdev_issue_zeroout - generate number of zero filed write bios
 * @bdev:	blockdev to issue
 * @sector:	start sector
 * @nr_sects:	number of sectors to write
 * @gfp_mask:	memory allocation flags (for bio_alloc)
 *
 * Description:
 *  Generate and issue number of bios with zerofiled pages.
 */

int blkdev_issue_zeroout(struct block_device *bdev, sector_t sector,
			sector_t nr_sects, gfp_t gfp_mask)
{
	int ret;
	struct bio *bio;
	struct bio_batch bb;
	unsigned int sz;
	DECLARE_COMPLETION_ONSTACK(wait);

	atomic_set(&bb.done, 1);
	bb.flags = 1 << BIO_UPTODATE;
	bb.wait = &wait;

	ret = 0;
	while (nr_sects != 0) {
		bio = bio_alloc(gfp_mask,
				min(nr_sects, (sector_t)BIO_MAX_PAGES));
		if (!bio) {
			ret = -ENOMEM;
			break;
		}

		bio->bi_sector = sector;
		bio->bi_bdev   = bdev;
		bio->bi_end_io = bio_batch_end_io;
		bio->bi_private = &bb;

		while (nr_sects != 0) {
			sz = min((sector_t) PAGE_SIZE >> 9 , nr_sects);
			ret = bio_add_page(bio, ZERO_PAGE(0), sz << 9, 0);
			nr_sects -= ret >> 9;
			sector += ret >> 9;
			if (ret < (sz << 9))
				break;
		}
		ret = 0;
		atomic_inc(&bb.done);
		submit_bio(WRITE, bio);
	}

	/* Wait for bios in-flight */
	if (!atomic_dec_and_test(&bb.done))
		wait_for_completion(&wait);

	if (!test_bit(BIO_UPTODATE, &bb.flags))
		/* One of bios in the batch was completed with error.*/
		ret = -EIO;

	return ret;
}
EXPORT_SYMBOL(blkdev_issue_zeroout);
