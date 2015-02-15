/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
#include <linux/blkdev.h>
#include <linux/debugfs.h>
#include <linux/test-iosched.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_cmnd.h>
#include <../sd.h>
#include <linux/delay.h>

#define MODULE_NAME "ufs_test"

#define TEST_MAX_BIOS_PER_REQ		16
#define LARGE_PRIME_1	1103515367
#define LARGE_PRIME_2	35757
#define DEFAULT_NUM_OF_BIOS		2

/* the amount of requests that will be inserted */
#define LONG_SEQ_TEST_NUM_REQS  256
/* request queue limitation is 128 requests, and we leave 10 spare requests */
#define QUEUE_MAX_REQUESTS 118
#define MB_MSEC_RATIO_APPROXIMATION ((1024 * 1024) / 1000)
/* actual number of MiB in test multiplied by 10, for single digit precision*/
#define BYTE_TO_MB_x_10(x) ((x * 10) / (1024 * 1024))
/* extract integer value */
#define LONG_TEST_SIZE_INTEGER(x) (BYTE_TO_MB_x_10(x) / 10)
/* and calculate the MiB value fraction */
#define LONG_TEST_SIZE_FRACTION(x) (BYTE_TO_MB_x_10(x) - \
		(LONG_TEST_SIZE_INTEGER(x) * 10))

#define test_pr_debug(fmt, args...) pr_debug("%s: "fmt"\n", MODULE_NAME, args)
#define test_pr_info(fmt, args...) pr_info("%s: "fmt"\n", MODULE_NAME, args)
#define test_pr_err(fmt, args...) pr_err("%s: "fmt"\n", MODULE_NAME, args)

enum ufs_test_testcases {
	UFS_TEST_WRITE_READ_TEST,

	TEST_LONG_SEQUENTIAL_READ,
	TEST_LONG_SEQUENTIAL_WRITE,
};

struct ufs_test_debug {
	struct dentry *write_read_test; /* basic test */
	struct dentry *random_test_seed; /* parameters in utils */
	struct dentry *long_sequential_read_test;
	struct dentry *long_sequential_write_test;
};

struct ufs_test_data {
	/* Data structure for debugfs dentrys */
	struct ufs_test_debug debug;
	/*
	 * Data structure containing individual test information, including
	 * self-defined specific data
	 */
	struct test_info test_info;
	/* device test */
	struct blk_dev_test_type bdt;
	/* A wait queue for OPs to complete */
	wait_queue_head_t wait_q;
	/* a flag for read compleation */
	bool read_completed;
	/* a flag for write compleation */
	bool write_completed;
	/*
	 * To determine the number of r/w bios. When seed = 0, random is
	 * disabled and 2 BIOs are written.
	 */
	unsigned int random_test_seed;
	/* A counter for the number of test requests completed */
	unsigned int completed_req_count;
};

static struct ufs_test_data *utd;

static bool message_repeat;

static char *ufs_test_get_test_case_str(struct test_data *td)
{
	if (!td) {
		test_pr_err("%s: NULL td", __func__);
		return NULL;
	}

	switch (td->test_info.testcase) {
	case UFS_TEST_WRITE_READ_TEST:
		return "UFS write read test";
		break;
	case TEST_LONG_SEQUENTIAL_READ:
		return "UFS long sequential read test";
		break;
	case TEST_LONG_SEQUENTIAL_WRITE:
		return "UFS long sequential write test";
		break;
	default:
		return "Unknown test";
	}
}

static unsigned int ufs_test_pseudo_random_seed(unsigned int *seed_number,
		unsigned int min_val, unsigned int max_val)
{
	int ret = 0;

	if (!seed_number)
		return 0;

	*seed_number = ((unsigned int) (((unsigned long) *seed_number
			* (unsigned long) LARGE_PRIME_1) + LARGE_PRIME_2));
	ret = (unsigned int) ((*seed_number) % max_val);

	return (ret > min_val ? ret : min_val);
}

static void ufs_test_pseudo_rnd_size(unsigned int *seed,
				unsigned int *num_of_bios)
{
	*num_of_bios = ufs_test_pseudo_random_seed(seed, 1,
						TEST_MAX_BIOS_PER_REQ);
	if (!(*num_of_bios))
		*num_of_bios = DEFAULT_NUM_OF_BIOS;
}

static void ufs_test_write_read_test_end_io_fn(struct request *rq, int err)
{
	struct test_request *test_rq = (struct test_request *)rq->elv.priv[0];
	BUG_ON(!test_rq);

	test_rq->req_completed = 1;
	test_rq->req_result = err;

	test_pr_info("%s: request %d completed, err=%d",
			__func__, test_rq->req_id, err);

	utd->write_completed = true;
	wake_up(&utd->wait_q);
}

static struct gendisk *ufs_test_get_rq_disk(void)
{
	struct request_queue *req_q = test_iosched_get_req_queue();
	struct scsi_device *sd;
	struct device *dev;
	struct scsi_disk *sdkp;
	struct gendisk *gd;

	if (!req_q) {
		test_pr_info("%s: Could not fetch request_queue", __func__);
		gd = NULL;
		goto exit;
	}

	sd = (struct scsi_device *)req_q->queuedata;

	dev = &sd->sdev_gendev;
	sdkp = scsi_disk_get_from_dev(dev);
	if (!sdkp) {
		test_pr_info("%s: Could not fatch scsi disk", __func__);
		gd = NULL;
		goto exit;
	}

	gd = sdkp->disk;
exit:
	return gd;
}

static int ufs_test_run_write_read_test(struct test_data *td)
{
	int ret = 0;
	unsigned int start_sec;
	unsigned int num_bios;
	struct request_queue *q = td->req_q;


	start_sec = td->start_sector + sizeof(int) * BIO_U32_SIZE
			* td->num_of_write_bios;
	if (utd->random_test_seed != 0)
		ufs_test_pseudo_rnd_size(&utd->random_test_seed, &num_bios);
	else
		num_bios = DEFAULT_NUM_OF_BIOS;

	/* Adding a write request */
	test_pr_info(
		"%s: Adding a write request with %d bios to Q, req_id=%d"
			, __func__, num_bios, td->wr_rd_next_req_id);

	utd->write_completed = false;
	ret = test_iosched_add_wr_rd_test_req(0, WRITE, start_sec,
					num_bios, TEST_PATTERN_5A,
					ufs_test_write_read_test_end_io_fn);

	if (ret) {
		test_pr_err("%s: failed to add a write request", __func__);
		return ret;
	}

	/* waiting for the write request to finish */
	blk_run_queue(q);
	wait_event(utd->wait_q, utd->write_completed);

	/* Adding a read request*/
	test_pr_info("%s: Adding a read request to Q", __func__);

	ret = test_iosched_add_wr_rd_test_req(0, READ, start_sec,
			num_bios, TEST_PATTERN_5A, NULL);

	if (ret) {
		test_pr_err("%s: failed to add a read request", __func__);
		return ret;
	}

	blk_run_queue(q);
	return ret;
}

static
int ufs_test_write_read_test_open_cb(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	message_repeat = 1;
	test_pr_info("%s:UFS test initialized", __func__);
	return 0;
}

static ssize_t ufs_test_write_read_test_write_cb(struct file *file,
					const char __user *buf,
					size_t count, loff_t *ppos)
{
	int ret = 0;
	int i;
	int number;

	sscanf(buf, "%d", &number);

	if (number <= 0)
		number = 1;

	test_pr_info("%s:the test will run for %d iterations.",
			__func__, number);
	memset(&utd->test_info, 0, sizeof(struct test_info));

	/* Initializing test */
	utd->test_info.data = utd;
	utd->test_info.get_test_case_str_fn = ufs_test_get_test_case_str;
	utd->test_info.testcase = UFS_TEST_WRITE_READ_TEST;
	utd->test_info.get_rq_disk_fn = ufs_test_get_rq_disk;
	utd->test_info.run_test_fn = ufs_test_run_write_read_test;

	/* Running the test multiple times */
	for (i = 0; i < number; ++i) {
		ret = test_iosched_start_test(&utd->test_info);
		if (ret) {
			test_pr_err("%s: Test failed.", __func__);
			return ret;
		}
	}

	test_pr_info("%s: Completed all the ufs test iterations.", __func__);

	return count;
}

static ssize_t ufs_test_write_read_test_read_cb(struct file *file,
		char __user *buffer, size_t count, loff_t *offset)
{
	memset((void *) buffer, 0, count);

	snprintf(buffer, count, "\nThis is a UFS write-read test for debug.\n");

	if (message_repeat == 1) {
		message_repeat = 0;
		return strnlen(buffer, count);
	} else
		return 0;
}

const struct file_operations write_read_test_ops = {
		.open = ufs_test_write_read_test_open_cb,
		.write = ufs_test_write_read_test_write_cb,
		.read = ufs_test_write_read_test_read_cb,
};

static void long_seq_test_free_end_io_fn(struct request *rq, int err)
{
	struct test_request *test_rq;
	struct test_data *ptd = test_get_test_data();

	if (rq)
		test_rq = (struct test_request *)rq->elv.priv[0];
	else {
		test_pr_err("%s: error: NULL request", __func__);
		return;
	}

	BUG_ON(!test_rq);

	spin_lock_irq(&ptd->lock);
	ptd->dispatched_count--;
	list_del_init(&test_rq->queuelist);
	__blk_put_request(ptd->req_q, test_rq->rq);
	spin_unlock_irq(&ptd->lock);

	kfree(test_rq->bios_buffer);
	kfree(test_rq);
	utd->completed_req_count++;

	test_pr_err("%s: request %d completed, err=%d",
	       __func__, test_rq->req_id, err);

	check_test_completion();

}

static int run_long_seq_test(struct test_data *td)
{
	int ret = 0;
	int direction;
	static unsigned int inserted_requests;

	BUG_ON(!td);
	td->test_count = 0;
	utd->completed_req_count = 0;
	inserted_requests = 0;

	if (td->test_info.testcase == TEST_LONG_SEQUENTIAL_READ)
		direction = READ;
	else
		direction = WRITE;

	test_pr_info("%s: Adding %d requests, first req_id=%d",
		     __func__, LONG_SEQ_TEST_NUM_REQS,
		     td->wr_rd_next_req_id);

	do {
		/*
		* since our requests come from a pool containing 128
		* requests, we don't want to exhaust this quantity,
		* therefore we add up to QUEUE_MAX_REQUESTS (which
		* includes a safety margin) and then call the mmc layer
		* to fetch them
		*/
		if (td->test_count >= QUEUE_MAX_REQUESTS) {
			blk_run_queue(td->req_q);
			continue;
		}

		ret = test_iosched_add_wr_rd_test_req(0, direction,
			td->start_sector, TEST_MAX_BIOS_PER_REQ,
			TEST_PATTERN_5A,
			long_seq_test_free_end_io_fn);
		if (ret) {
			test_pr_err("%s: failed to create request" , __func__);
			break;
		}
		inserted_requests++;
		td->test_info.test_byte_count +=
			(TEST_MAX_BIOS_PER_REQ * sizeof(unsigned int) *
			BIO_U32_SIZE);

	} while (inserted_requests < LONG_SEQ_TEST_NUM_REQS);

	/* in this case the queue will not run in the above loop */
	if (LONG_SEQ_TEST_NUM_REQS < QUEUE_MAX_REQUESTS)
		blk_run_queue(td->req_q);

	return ret;
}


void long_seq_test_calc_throughput(unsigned long mtime,
				   unsigned long byte_count)
{
	unsigned long fraction, integer;

	test_pr_info("%s: time is %lu msec, size is %lu.%lu MiB",
			__func__, mtime, LONG_TEST_SIZE_INTEGER(byte_count),
				LONG_TEST_SIZE_FRACTION(byte_count));

	/* we first multiply in order not to lose precision */
	mtime *= MB_MSEC_RATIO_APPROXIMATION;
	/* divide values to get a MiB/sec integer value with one
	   digit of precision
	   */
	fraction = integer = (byte_count * 10) / mtime;
	integer /= 10;
	/* and calculate the MiB value fraction */
	fraction -= integer * 10;

	test_pr_info("%s: Throughput: %lu.%lu MiB/sec\n",
		__func__, integer, fraction);
}

static ssize_t long_sequential_read_test_write(struct file *file,
				const char __user *buf,
				size_t count,
				loff_t *ppos)
{
	int ret = 0;
	int i = 0;
	int number = -1;
	unsigned long mtime, byte_count;

	test_pr_info("%s: -- UFS Long Sequential Read TEST --", __func__);

	sscanf(buf, "%d", &number);

	if (number <= 0)
		number = 1;

	memset(&utd->test_info, 0, sizeof(struct test_info));

	utd->test_info.data = utd;
	utd->test_info.get_rq_disk_fn = ufs_test_get_rq_disk;
	utd->test_info.run_test_fn = run_long_seq_test;
	utd->test_info.get_test_case_str_fn = ufs_test_get_test_case_str;
	utd->test_info.testcase = TEST_LONG_SEQUENTIAL_READ;

	for (i = 0 ; i < number ; ++i) {
		test_pr_info("%s: Cycle # %d / %d", __func__, i+1, number);
		test_pr_info("%s: ====================", __func__);

		ret = test_iosched_start_test(&utd->test_info);
		if (ret)
			break;

		mtime = ktime_to_ms(utd->test_info.test_duration);
		byte_count = utd->test_info.test_byte_count;

		long_seq_test_calc_throughput(mtime, byte_count);

		/* Allow FS requests to be dispatched */
		msleep(1000);
	}

	return count;
}

static ssize_t long_sequential_read_test_read(struct file *file,
			       char __user *buffer,
			       size_t count,
			       loff_t *offset)
{
	memset((void *)buffer, 0, count);

	snprintf(buffer, count,
		 "\nufs_long_sequential_read_test\n"
		 "=========\n"
		 "Description:\n"
		 "This test runs the following scenarios\n"
		 "- Long Sequential Read Test: this test measures read "
		 "throughput at the driver level by sequentially reading many "
		 "large requests.\n");

	if (message_repeat == 1) {
		message_repeat = 0;
		return strnlen(buffer, count);
	} else
		return 0;
}

static bool message_repeat;
static int test_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	message_repeat = 1;
	return 0;
}

const struct file_operations long_sequential_read_test_ops = {
	.open = test_open,
	.write = long_sequential_read_test_write,
	.read = long_sequential_read_test_read,
};

static ssize_t long_sequential_write_test_write(struct file *file,
				const char __user *buf,
				size_t count,
				loff_t *ppos)
{
	int ret = 0;
	int i = 0;
	int number = -1;
	unsigned long mtime, byte_count;

	test_pr_info("%s: -- UFS Long Sequential Write TEST --", __func__);

	sscanf(buf, "%d", &number);

	if (number <= 0)
		number = 1;

	memset(&utd->test_info, 0, sizeof(struct test_info));

	utd->test_info.data = utd;
	utd->test_info.get_rq_disk_fn = ufs_test_get_rq_disk;
	utd->test_info.get_test_case_str_fn = ufs_test_get_test_case_str;
	utd->test_info.run_test_fn = run_long_seq_test;
	utd->test_info.testcase = TEST_LONG_SEQUENTIAL_WRITE;

	for (i = 0 ; i < number ; ++i) {
		test_pr_info("%s: Cycle # %d / %d", __func__, i+1, number);
		test_pr_info("%s: ====================", __func__);

		utd->test_info.test_byte_count = 0;
		ret = test_iosched_start_test(&utd->test_info);
		if (ret)
			break;

		mtime = ktime_to_ms(utd->test_info.test_duration);
		byte_count = utd->test_info.test_byte_count;

		long_seq_test_calc_throughput(mtime, byte_count);

		/* Allow FS requests to be dispatched */
		msleep(1000);
	}

	return count;
}

static ssize_t long_sequential_write_test_read(struct file *file,
			       char __user *buffer,
			       size_t count,
			       loff_t *offset)
{
	memset((void *)buffer, 0, count);

	snprintf(buffer, count,
		 "\nufs_long_sequential_write_test\n"
		 "=========\n"
		 "Description:\n"
		 "This test runs the following scenarios\n"
		 "- Long Sequential Write Test: this test measures write "
		 "throughput at the driver level by sequentially writing many "
		 "large requests\n");

	if (message_repeat == 1) {
		message_repeat = 0;
		return strnlen(buffer, count);
	} else
		return 0;
}

const struct file_operations long_sequential_write_test_ops = {
	.open = test_open,
	.write = long_sequential_write_test_write,
	.read = long_sequential_write_test_read,
};

static void ufs_test_debugfs_cleanup(void)
{
	debugfs_remove_recursive(test_iosched_get_debugfs_tests_root());
}

static int ufs_test_debugfs_init(void)
{
	struct dentry *utils_root, *tests_root;
	int ret = 0;

	utils_root = test_iosched_get_debugfs_utils_root();
	tests_root = test_iosched_get_debugfs_tests_root();

	if (!utils_root || !tests_root) {
		test_pr_err("%s: Failed to create debugfs root.", __func__);
		ret = -EINVAL;
		goto exit;
	}

	utd->debug.random_test_seed = debugfs_create_u32("random_test_seed",
			S_IRUGO | S_IWUGO, utils_root, &utd->random_test_seed);

	if (!utd->debug.random_test_seed) {
		test_pr_err("%s: Could not create debugfs random_test_seed.",
				__func__);
		ret = -ENOMEM;
		goto exit;
	}

	utd->debug.write_read_test = debugfs_create_file("ufs_write_read_test",
					S_IRUGO | S_IWUGO, tests_root,
					NULL, &write_read_test_ops);

	if (!utd->debug.write_read_test) {
		ret = -ENOMEM;
		goto exit_err;
	}

	utd->debug.long_sequential_read_test = debugfs_create_file(
					"ufs_long_sequential_read_test",
					S_IRUGO | S_IWUGO,
					tests_root,
					NULL,
					&long_sequential_read_test_ops);

	if (!utd->debug.long_sequential_read_test) {
		ret = -ENOMEM;
		goto exit_err;
	}

	utd->debug.long_sequential_write_test = debugfs_create_file(
					"ufs_long_sequential_write_test",
					S_IRUGO | S_IWUGO,
					tests_root,
					NULL,
					&long_sequential_write_test_ops);

	if (!utd->debug.long_sequential_write_test) {
		ret = -ENOMEM;
		goto exit_err;
	}

	goto exit;

exit_err:
	debugfs_remove_recursive(tests_root);
exit:
	return ret;
}

static void ufs_test_probe(void)
{
	ufs_test_debugfs_init();
}

static void ufs_test_remove(void)
{
	ufs_test_debugfs_cleanup();
}

int __init ufs_test_init(void)
{
	utd = kzalloc(sizeof(struct ufs_test_data), GFP_KERNEL);
	if (!utd) {
		test_pr_err("%s: failed to allocate ufs_test_data", __func__);
		return -ENODEV;
	}

	init_waitqueue_head(&utd->wait_q);
	utd->bdt.init_fn = ufs_test_probe;
	utd->bdt.exit_fn = ufs_test_remove;
	INIT_LIST_HEAD(&utd->bdt.list);
	test_iosched_register(&utd->bdt);

	return 0;
}
EXPORT_SYMBOL_GPL(ufs_test_init);

static void __exit ufs_test_exit(void)
{
	test_iosched_unregister(&utd->bdt);
	kfree(utd);
}
module_init(ufs_test_init)
;
module_exit(ufs_test_exit)
;

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("UFC test");

