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
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/bitrev.h>
#include <linux/crc-ccitt.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <linux/bif/consumer.h>
#include <linux/bif/driver.h>

/**
 * struct bif_ctrl_dev - holds controller device specific information
 * @list:			Doubly-linked list parameter linking to other
 *				BIF controllers registered in the system
 * @desc:			Description structure for this BIF controller
 * @mutex:			Mutex lock that is used to ensure mutual
 *				exclusion between transactions performed on the
 *				BIF bus for this controller
 * @ctrl_dev:			Device pointer to the BIF controller device
 * @driver_data:		Private data used by the BIF controller
 * @selected_sdev:		Slave device that is currently selected on
 *				the BIF bus of this controller
 * @bus_change_notifier:	Head of a notifier list containing notifier
 *				blocks that are notified when the battery
 *				presence changes
 * @enter_irq_mode_work:	Work task that is scheduled after a transaction
 *				completes when there are consumers that are
 *				actively monitoring BIF slave interrupts
 * @irq_count:			This is a count of the total number of BIF slave
 *				interrupts that are currently being monitored
 *				for the BIF slaves connected to this BIF
 *				controller
 * @irq_mode_delay_jiffies:	Number of jiffies to wait before scheduling the
 *				enter IRQ mode task.  Using a larger value
 *				helps to improve the performance of BIF
 *				consumers that perform many BIF transactions.
 *				Using a smaller value reduces the latency of
 *				BIF slave interrupts.
 * @battery_present:		Cached value of the battery presence.  This is
 *				used to filter out spurious presence update
 *				calls when the battery presence state has not
 *				changed.
 */
struct bif_ctrl_dev {
	struct list_head		list;
	struct bif_ctrl_desc		*desc;
	struct mutex			mutex;
	struct device			*ctrl_dev;
	void				*driver_data;
	struct bif_slave_dev		*selected_sdev;
	struct blocking_notifier_head	bus_change_notifier;
	struct delayed_work		enter_irq_mode_work;
	int				irq_count;
	int				irq_mode_delay_jiffies;
	bool				battery_present;
};

/**
 * struct bif_ctrl - handle used by BIF consumers for bus oriented BIF
 *			operations
 * @bdev:		Pointer to BIF controller device
 * @exclusive_lock:	Flag which indicates that the BIF consumer responsible
 *			for this handle has locked the BIF bus of this
 *			controller.  BIF transactions from other consumers are
 *			blocked until the bus is unlocked.
 */
struct bif_ctrl {
	struct bif_ctrl_dev	*bdev;
	bool			exclusive_lock;
};

/**
 * struct bif_slave_dev - holds BIF slave device information
 * @list:			Doubly-linked list parameter linking to other
 *				BIF slaves that have been enumerated
 * @bdev:			Pointer to the BIF controller device that this
 *				slave is physically connected to
 * @slave_addr:			8-bit BIF DEV_ADR assigned to this slave
 * @unique_id:			80-bit BIF unique ID of the slave
 * @unique_id_bits_known:	Number of bits of the UID that are currently
 *				known.  This number starts is incremented during
 *				a UID search and must end at 80 if the slave
 *				responds to the search properly.
 * @present:			Boolean value showing if this slave is
*				physically present in the system at a given
*				point in time.  The value is set to false if the
*				battery pack containing the slave is
*				disconnected.
 * @l1_data:			BIF DDB L1 data of the slave as read from the
 *				slave's memory
 * @function_directory:		Pointer to the BIF DDB L2 function directory
 *				list as read from the slave's memory
 * @protocol_function:		Pointer to constant protocol function data as
 *				well as software state information if the slave
 *				has a protocol function
 * @slave_ctrl_function:	Pointer to constant slave control function data
 *				as well as software state information if the
 *				slave has a slave control function
 * @nvm_function:		Pointer to constant non-volatile memory function
 *				data as well as software state information if
 *				the slave has a non-volatile memory function
 *
 * bif_slave_dev objects are stored indefinitely after enumeration in order to
 * speed up battery reinsertion.  Only a UID check is needed after inserting a
 * battery assuming it has been enumerated before.
 *
 * unique_id bytes are stored such that unique_id[0] = MSB and
 * unique_id[BIF_UNIQUE_ID_BYTE_LENGTH - 1] = LSB
 */
struct bif_slave_dev {
	struct list_head			list;
	struct bif_ctrl_dev			*bdev;
	u8					slave_addr;
	u8				unique_id[BIF_UNIQUE_ID_BYTE_LENGTH];
	int					unique_id_bits_known;
	bool					present;
	struct bif_ddb_l1_data			l1_data;
	struct bif_ddb_l2_data			*function_directory;
	struct bif_protocol_function		*protocol_function;
	struct bif_slave_control_function	*slave_ctrl_function;
	struct bif_nvm_function			*nvm_function;
};

/**
 * struct bif_slave - handle used by BIF consumers for slave oriented BIF
 *			operations
 * @ctrl:		Consumer BIF controller handle data
 * @sdev:		Pointer to BIF slave device
 */
struct bif_slave {
	struct bif_ctrl				ctrl;
	struct bif_slave_dev			*sdev;
};

/* Number of times to retry a full BIF transaction before returning an error. */
#define BIF_TRANSACTION_RETRY_COUNT	5

static DEFINE_MUTEX(bif_ctrl_list_mutex);
static LIST_HEAD(bif_ctrl_list);
static DEFINE_MUTEX(bif_sdev_list_mutex);
static LIST_HEAD(bif_sdev_list);

static u8 next_dev_addr = 0x02;

#define DEBUG_PRINT_BUFFER_SIZE 256
static void fill_string(char *str, size_t str_len, u8 *buf, int buf_len)
{
	int pos = 0;
	int i;

	for (i = 0; i < buf_len; i++) {
		pos += scnprintf(str + pos, str_len - pos, "0x%02X", buf[i]);
		if (i < buf_len - 1)
			pos += scnprintf(str + pos, str_len - pos, ", ");
	}
}

static void bif_print_slave_data(struct bif_slave_dev *sdev)
{
	char str[DEBUG_PRINT_BUFFER_SIZE];
	u8 *uid;
	int i, j;
	struct bif_object *object;

	if (sdev->unique_id_bits_known != BIF_UNIQUE_ID_BIT_LENGTH)
		return;

	uid = sdev->unique_id;
	pr_debug("BIF slave: 0x%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n",
		uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6],
		uid[7], uid[8], uid[9]);
	pr_debug("  present=%d, dev_adr=0x%02X\n", sdev->present,
		sdev->slave_addr);
	pr_debug("  revision=0x%02X, level=0x%02X, device class=0x%04X\n",
		sdev->l1_data.revision, sdev->l1_data.level,
		sdev->l1_data.device_class);
	pr_debug("  manufacturer ID=0x%04X, product ID=0x%04X\n",
		sdev->l1_data.manufacturer_id, sdev->l1_data.product_id);
	pr_debug("  function directory length=%d\n", sdev->l1_data.length);

	for (i = 0; i < sdev->l1_data.length / 4; i++) {
		pr_debug("  Function %d: type=0x%02X, version=0x%02X, pointer=0x%04X\n",
			i, sdev->function_directory[i].function_type,
			sdev->function_directory[i].function_version,
			sdev->function_directory[i].function_pointer);
	}

	if (sdev->nvm_function) {
		pr_debug("  NVM function: pointer=0x%04X, task=%d, wr_buf_size=%d, nvm_base=0x%04X, nvm_size=%d, nvm_lock_offset=%d\n",
			sdev->nvm_function->nvm_pointer,
			sdev->nvm_function->slave_control_channel,
			(sdev->nvm_function->write_buffer_size
				? sdev->nvm_function->write_buffer_size : 0),
			sdev->nvm_function->nvm_base_address,
			sdev->nvm_function->nvm_size,
			sdev->nvm_function->nvm_lock_offset);
		if (sdev->nvm_function->object_count)
			pr_debug("  NVM objects:\n");
		i = 0;
		list_for_each_entry(object, &sdev->nvm_function->object_list,
					list) {
			pr_debug("    Object %d - addr=0x%04X, data len=%d, type=0x%02X, version=0x%02X, manufacturer ID=0x%04X, crc=0x%04X\n",
				i, object->addr, object->length - 8,
				object->type, object->version,
				object->manufacturer_id, object->crc);
			for (j = 0; j < DIV_ROUND_UP(object->length - 8, 16);
					j++) {
				fill_string(str, DEBUG_PRINT_BUFFER_SIZE,
					object->data + j * 16,
					min(16, object->length - 8 - (j * 16)));
				pr_debug("      data(0x%04X): %s\n", j * 16,
					str);
			}
			i++;
		}
	}
}

static void bif_print_slaves(void)
{
	struct bif_slave_dev *sdev;

	mutex_lock(&bif_sdev_list_mutex);

	list_for_each_entry(sdev, &bif_sdev_list, list) {
		/* Skip slaves without fully known UIDs. */
		if (sdev->unique_id_bits_known != BIF_UNIQUE_ID_BIT_LENGTH)
			continue;
		bif_print_slave_data(sdev);
	}

	mutex_unlock(&bif_sdev_list_mutex);
}

static struct bif_slave_dev *bif_add_slave(struct bif_ctrl_dev *bdev)
{
	struct bif_slave_dev *sdev;

	sdev = kzalloc(sizeof(struct bif_slave_dev), GFP_KERNEL);
	if (sdev == NULL) {
		pr_err("Memory allocation failed for bif_slave_dev\n");
		return ERR_PTR(-ENOMEM);
	}

	sdev->bdev = bdev;
	INIT_LIST_HEAD(&sdev->list);
	list_add_tail(&sdev->list, &bif_sdev_list);

	return sdev;
}

static void bif_remove_slave(struct bif_slave_dev *sdev)
{
	list_del(&sdev->list);
	if (sdev->bdev->selected_sdev == sdev)
		sdev->bdev->selected_sdev = NULL;

	if (sdev->slave_ctrl_function)
		kfree(sdev->slave_ctrl_function->irq_notifier_list);
	kfree(sdev->slave_ctrl_function);
	kfree(sdev->protocol_function);
	kfree(sdev->function_directory);

	kfree(sdev);
}

/* This function assumes that the uid array is all 0 to start with. */
static void set_uid_bit(u8 uid[BIF_UNIQUE_ID_BYTE_LENGTH], unsigned int bit,
			unsigned int value)
{
	u8 mask;

	if (bit >= BIF_UNIQUE_ID_BIT_LENGTH)
		return;

	mask = 1 << (7 - (bit % 8));

	uid[bit / 8] &= ~mask;
	uid[bit / 8] |= value << (7 - (bit % 8));
}

static unsigned int get_uid_bit(u8 uid[BIF_UNIQUE_ID_BYTE_LENGTH],
			unsigned int bit)
{
	if (bit >= BIF_UNIQUE_ID_BIT_LENGTH)
		return 0;

	return (uid[bit / 8] & (1 << (7 - (bit % 8)))) ? 1 : 0;
}

static void bif_enter_irq_mode_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bif_ctrl_dev *bdev
		= container_of(dwork, struct bif_ctrl_dev, enter_irq_mode_work);
	int rc, i;

	mutex_lock(&bdev->mutex);
	for (i = 0; i < BIF_TRANSACTION_RETRY_COUNT; i++) {
		rc = bdev->desc->ops->set_bus_state(bdev,
					BIF_BUS_STATE_INTERRUPT);
		if (rc == 0)
			break;
	}
	mutex_unlock(&bdev->mutex);

	/* Reschedule the task if the transaction failed. */
	if (rc) {
		pr_err("Could not set BIF bus to interrupt mode, rc=%d\n", rc);
		schedule_delayed_work(&bdev->enter_irq_mode_work,
					bdev->irq_mode_delay_jiffies);
	}
}

static void bif_cancel_irq_mode_work(struct bif_ctrl_dev *bdev)
{
	cancel_delayed_work(&bdev->enter_irq_mode_work);
}

static void bif_schedule_irq_mode_work(struct bif_ctrl_dev *bdev)
{
	if (bdev->irq_count > 0 &&
	    bdev->desc->ops->get_bus_state(bdev) != BIF_BUS_STATE_INTERRUPT)
		schedule_delayed_work(&bdev->enter_irq_mode_work,
					bdev->irq_mode_delay_jiffies);
}

static int _bif_select_slave_no_retry(struct bif_slave_dev *sdev)
{
	struct bif_ctrl_dev *bdev = sdev->bdev;
	int rc = 0;
	int i;

	/* Check if the slave is already selected. */
	if (sdev->bdev->selected_sdev == sdev)
		return 0;

	if (sdev->slave_addr) {
		/* Select using DEV_ADR. */
		rc = bdev->desc->ops->bus_transaction(bdev, BIF_TRANS_SDA,
							sdev->slave_addr);
		if (!rc)
			sdev->bdev->selected_sdev = sdev;
	} else if (sdev->unique_id_bits_known == BIF_UNIQUE_ID_BIT_LENGTH) {
		/* Select using full UID. */
		for (i = 0; i < BIF_UNIQUE_ID_BYTE_LENGTH - 1; i++) {
			rc = bdev->desc->ops->bus_transaction(bdev,
				BIF_TRANS_EDA, sdev->unique_id[i]);
			if (rc)
				goto out;
		}

		rc = bdev->desc->ops->bus_transaction(bdev, BIF_TRANS_SDA,
			sdev->unique_id[BIF_UNIQUE_ID_BYTE_LENGTH - 1]);
		if (rc)
			goto out;
	} else {
		pr_err("Cannot select slave because it has neither UID nor DEV_ADR.\n");
		return -EINVAL;
	}

	sdev->bdev->selected_sdev = sdev;

	return 0;
out:
	pr_err("bus_transaction failed, rc=%d\n", rc);
	return rc;
}

static int bif_select_slave(struct bif_slave_dev *sdev)
{
	int rc = -EPERM;
	int i;

	for (i = 0; i < BIF_TRANSACTION_RETRY_COUNT; i++) {
		rc = _bif_select_slave_no_retry(sdev);
		if (rc == 0)
			break;
		/* Force slave reselection. */
		sdev->bdev->selected_sdev = NULL;
	}

	return rc;
}

/*
 * Returns 1 if slave is selected, 0 if slave is not selected, or errno if
 * error.
 */
static int bif_is_slave_selected(struct bif_ctrl_dev *bdev)
{
	int rc = -EPERM;
	int tack, i;

	for (i = 0; i < BIF_TRANSACTION_RETRY_COUNT; i++) {
		/* Attempt a transaction query. */
		rc = bdev->desc->ops->bus_transaction_read(bdev, BIF_TRANS_BC,
						BIF_CMD_TQ, &tack);
		if (rc == 0 || rc == -ETIMEDOUT)
			break;
	}

	if (rc == 0)
		rc = 1;
	else if (rc == -ETIMEDOUT)
		rc = 0;
	else
		pr_err("BIF bus_transaction_read failed, rc=%d\n", rc);

	return rc;
}

/* Read from a specified number of consecutive registers. */
static int _bif_slave_read_no_retry(struct bif_slave_dev *sdev, u16 addr,
			u8 *buf, int len)
{
	struct bif_ctrl_dev *bdev = sdev->bdev;
	int rc = 0;
	int i, response;

	rc = bif_select_slave(sdev);
	if (rc)
		return rc;

	if (bdev->desc->ops->read_slave_registers) {
		/*
		 * Use low level slave register read implementation in order to
		 * receive the benefits of BIF burst reads.
		 */
		rc = bdev->desc->ops->read_slave_registers(bdev, addr, buf,
							   len);
		if (rc)
			pr_debug("read_slave_registers failed, rc=%d\n", rc);
		else
			return rc;
		/*
		 * Fall back on individual transactions if high level register
		 * read failed.
		 */
	}

	for (i = 0; i < len; i++) {
		rc = bdev->desc->ops->bus_transaction(bdev, BIF_TRANS_ERA,
							addr >> 8);
		if (rc) {
			pr_err("bus_transaction failed, rc=%d\n", rc);
			return rc;
		}

		rc = bdev->desc->ops->bus_transaction_read(bdev, BIF_TRANS_RRA,
							addr & 0xFF, &response);
		if (rc) {
			pr_err("bus_transaction_read failed, rc=%d\n", rc);
			return rc;
		}

		if (!(response & BIF_SLAVE_RD_ACK)) {
			pr_err("BIF register read error=0x%02X\n",
				response & BIF_SLAVE_RD_ERR);
			return -EIO;
		}

		buf[i] = response & BIF_SLAVE_RD_DATA;
		addr++;
	}

	return rc;
}

/*
 * Read from a specified number of consecutive registers.  Retry the transaction
 * several times in case of communcation failures.
 */
static int _bif_slave_read(struct bif_slave_dev *sdev, u16 addr, u8 *buf,
			int len)
{
	int rc = -EPERM;
	int i;

	for (i = 0; i < BIF_TRANSACTION_RETRY_COUNT; i++) {
		rc = _bif_slave_read_no_retry(sdev, addr, buf, len);
		if (rc == 0)
			break;
		/* Force slave reselection. */
		sdev->bdev->selected_sdev = NULL;
	}

	return rc;
}

/* Write to a specified number of consecutive registers. */
static int _bif_slave_write_no_retry(struct bif_slave_dev *sdev, u16 addr,
			u8 *buf, int len)
{
	struct bif_ctrl_dev *bdev = sdev->bdev;
	int rc = 0;
	int i;

	rc = bif_select_slave(sdev);
	if (rc)
		return rc;

	if (bdev->desc->ops->write_slave_registers) {
		/*
		 * Use low level slave register write implementation in order to
		 * receive the benefits of BIF burst writes.
		 */
		rc = bdev->desc->ops->write_slave_registers(bdev, addr, buf,
							    len);
		if (rc)
			pr_debug("write_slave_registers failed, rc=%d\n", rc);
		else
			return rc;
		/*
		 * Fall back on individual transactions if high level register
		 * write failed.
		 */
	}

	rc = bdev->desc->ops->bus_transaction(bdev, BIF_TRANS_ERA, addr >> 8);
	if (rc)
		goto out;

	rc = bdev->desc->ops->bus_transaction(bdev, BIF_TRANS_WRA, addr & 0xFF);
	if (rc)
		goto out;

	for (i = 0; i < len; i++) {
		rc = bdev->desc->ops->bus_transaction(bdev, BIF_TRANS_WD,
							buf[i]);
		if (rc)
			goto out;
	}

	return 0;
out:
	pr_err("bus_transaction failed, rc=%d\n", rc);
	return rc;
}

/*
 * Write to a specified number of consecutive registers.  Retry the transaction
 * several times in case of communcation failures.
 */
static int _bif_slave_write(struct bif_slave_dev *sdev, u16 addr, u8 *buf,
			int len)
{
	int rc = -EPERM;
	int i;

	for (i = 0; i < BIF_TRANSACTION_RETRY_COUNT; i++) {
		rc = _bif_slave_write_no_retry(sdev, addr, buf, len);
		if (rc == 0)
			break;
		/* Force slave reselection. */
		sdev->bdev->selected_sdev = NULL;
	}

	return rc;
}

/* Perform a read-modify-write sequence on a single BIF slave register. */
static int _bif_slave_masked_write(struct bif_slave_dev *sdev, u16 addr, u8 val,
			u8 mask)
{
	int rc;
	u8 reg;

	rc = _bif_slave_read(sdev, addr, &reg, 1);
	if (rc)
		return rc;

	reg = (reg & ~mask) | (val & mask);

	return _bif_slave_write(sdev, addr, &reg, 1);
}

static int _bif_check_task(struct bif_slave_dev *sdev, unsigned int task)
{
	if (IS_ERR_OR_NULL(sdev)) {
		pr_err("Invalid slave device handle=%ld\n", PTR_ERR(sdev));
		return -EINVAL;
	} else if (!sdev->bdev) {
		pr_err("BIF controller has been removed\n");
		return -ENXIO;
	} else if (!sdev->slave_ctrl_function
			|| sdev->slave_ctrl_function->task_count == 0) {
		pr_err("BIF slave does not support slave control\n");
		return -ENODEV;
	} else if (task >= sdev->slave_ctrl_function->task_count) {
		pr_err("Requested task: %u greater than max: %u for this slave\n",
			task, sdev->slave_ctrl_function->task_count);
		return -EINVAL;
	}

	return 0;
}

static int _bif_task_is_busy(struct bif_slave_dev *sdev, unsigned int task)
{
	int rc;
	u16 addr;
	u8 reg = 0;

	rc = _bif_check_task(sdev, task);
	if (rc) {
		pr_err("Invalid slave device or task, rc=%d\n", rc);
		return rc;
	}

	/* Check the task busy state. */
	addr = SLAVE_CTRL_FUNC_TASK_BUSY_ADDR(
		sdev->slave_ctrl_function->slave_ctrl_pointer, task);
	rc = _bif_slave_read(sdev, addr, &reg, 1);
	if (rc) {
		pr_err("BIF slave register read failed, rc=%d\n", rc);
		return rc;
	}

	return (reg & BIT(task % SLAVE_CTRL_TASKS_PER_SET)) ? 1 : 0;
}

static int _bif_enable_auto_task(struct bif_slave_dev *sdev, unsigned int task)
{
	int rc;
	u16 addr;
	u8 mask;

	rc = _bif_check_task(sdev, task);
	if (rc) {
		pr_err("Invalid slave device or task, rc=%d\n", rc);
		return rc;
	}

	/* Enable the auto task within the slave */
	mask = BIT(task % SLAVE_CTRL_TASKS_PER_SET);
	addr = SLAVE_CTRL_FUNC_TASK_AUTO_TRIGGER_ADDR(
		sdev->slave_ctrl_function->slave_ctrl_pointer, task);
	if (task / SLAVE_CTRL_TASKS_PER_SET == 0) {
		/* Set global auto task enable. */
		mask |= BIT(0);
	}
	rc = _bif_slave_masked_write(sdev, addr, 0xFF, mask);
	if (rc) {
		pr_err("BIF slave register masked write failed, rc=%d\n", rc);
		return rc;
	}

	/* Set global auto task enable if task not in set 0. */
	if (task / SLAVE_CTRL_TASKS_PER_SET != 0) {
		addr = SLAVE_CTRL_FUNC_TASK_AUTO_TRIGGER_ADDR(
		       sdev->slave_ctrl_function->slave_ctrl_pointer, 0);
		rc = _bif_slave_masked_write(sdev, addr, 0xFF, BIT(0));
		if (rc) {
			pr_err("BIF slave register masked write failed, rc=%d\n",
				rc);
			return rc;
		}
	}

	return rc;
}

static int _bif_disable_auto_task(struct bif_slave_dev *sdev, unsigned int task)
{
	int rc;
	u16 addr;
	u8 mask;

	rc = _bif_check_task(sdev, task);
	if (rc) {
		pr_err("Invalid slave or task, rc=%d\n", rc);
		return rc;
	}

	/* Disable the auto task within the slave */
	mask = BIT(task % SLAVE_CTRL_TASKS_PER_SET);
	addr = SLAVE_CTRL_FUNC_TASK_AUTO_TRIGGER_ADDR(
		sdev->slave_ctrl_function->slave_ctrl_pointer, task);
	rc = _bif_slave_masked_write(sdev, addr, 0x00, mask);
	if (rc) {
		pr_err("BIF slave register masked write failed, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

/*
 * The MIPI-BIF spec does not define a maximum time in which an NVM write must
 * complete.  The following delay and recheck count therefore represent
 * arbitrary but reasonable values.
 */
#define NVM_WRITE_POLL_DELAY_MS		20
#define NVM_WRITE_MAX_POLL_COUNT	50

static int _bif_slave_nvm_raw_write(struct bif_slave_dev *sdev, u16 offset,
				u8 *buf, int len)
{
	int rc = 0;
	int write_len, poll_count, rc2;
	u8 write_buf[3];

	if (!sdev->nvm_function) {
		pr_err("BIF slave has no NVM function\n");
		return -ENODEV;
	} else if (offset + len > sdev->nvm_function->nvm_size) {
		pr_err("write offset + len = %d > NVM size = %d\n",
			offset + len, sdev->nvm_function->nvm_size);
		return -EINVAL;
	} else if (offset < sdev->nvm_function->nvm_lock_offset) {
		pr_err("write offset = %d < first writable offset = %d\n",
			offset, sdev->nvm_function->nvm_lock_offset);
		return -EINVAL;
	}

	rc = _bif_enable_auto_task(sdev,
			sdev->nvm_function->slave_control_channel);
	if (rc) {
		pr_err("Failed to enable NVM auto task, rc=%d\n", rc);
		return rc;
	}

	while (len > 0) {
		write_len = sdev->nvm_function->write_buffer_size;
		if (write_len == 0)
			write_len = 256;
		write_len = min(write_len, len);

		write_buf[0] = offset >> 8;
		write_buf[1] = offset;
		write_buf[2] = (write_len == 256) ? 0 : write_len;

		/* Write offset and size registers. */
		rc = _bif_slave_write(sdev, sdev->nvm_function->nvm_pointer + 6,
					write_buf, 3);
		if (rc) {
			pr_err("BIF slave write failed, rc=%d\n", rc);
			goto done;
		}

		/* Write to NVM write buffer registers. */
		rc = _bif_slave_write(sdev, sdev->nvm_function->nvm_pointer + 9,
					buf, write_len);
		if (rc) {
			pr_err("BIF slave write failed, rc=%d\n", rc);
			goto done;
		}

		/*
		 * Wait for completion of the NVM write which was auto-triggered
		 * by the register write of the last byte in the NVM write
		 * buffer.
		 */
		poll_count = NVM_WRITE_MAX_POLL_COUNT;
		do {
			msleep(NVM_WRITE_POLL_DELAY_MS);
			rc = _bif_task_is_busy(sdev,
				sdev->nvm_function->slave_control_channel);
			poll_count--;
		} while (rc > 0 && poll_count > 0);

		if (rc < 0) {
			pr_err("Failed to check task state, rc=%d", rc);
			goto done;
		} else if (rc > 0) {
			pr_err("BIF slave NVM write not completed after %d ms\n",
			    NVM_WRITE_POLL_DELAY_MS * NVM_WRITE_MAX_POLL_COUNT);
			rc = -ETIMEDOUT;
			goto done;
		}

		len -= write_len;
		offset += write_len;
		buf += write_len;
	}

done:
	rc2 = _bif_disable_auto_task(sdev,
			sdev->nvm_function->slave_control_channel);
	if (rc2) {
		pr_err("Failed to disable NVM auto task, rc=%d\n", rc2);
		return rc2;
	}

	return rc;
}

/* Takes a mutex if this consumer is not an exclusive bus user. */
static void bif_ctrl_lock(struct bif_ctrl *ctrl)
{
	if (!ctrl->exclusive_lock) {
		mutex_lock(&ctrl->bdev->mutex);
		bif_cancel_irq_mode_work(ctrl->bdev);
	}
}

/* Releases a mutex if this consumer is not an exclusive bus user. */
static void bif_ctrl_unlock(struct bif_ctrl *ctrl)
{
	if (!ctrl->exclusive_lock) {
		bif_schedule_irq_mode_work(ctrl->bdev);
		mutex_unlock(&ctrl->bdev->mutex);
	}
}

static void bif_slave_ctrl_lock(struct bif_slave *slave)
{
	bif_ctrl_lock(&slave->ctrl);
}

static void bif_slave_ctrl_unlock(struct bif_slave *slave)
{
	bif_ctrl_unlock(&slave->ctrl);
}

/**
 * bif_crc_ccitt() - calculate the CRC-CCITT CRC value of the data specified
 * @buffer:	Data to calculate the CRC of
 * @len:	Length of the data buffer in bytes
 *
 * MIPI-BIF specifies the usage of CRC-CCITT for BIF data objects.  This
 * function performs the CRC calculation while taking into account the bit
 * ordering used by BIF.
 */
u16 bif_crc_ccitt(const u8 *buffer, unsigned int len)
{
	u16 crc = 0xFFFF;

	while (len--) {
		crc = crc_ccitt_byte(crc, bitrev8(*buffer));
		buffer++;
	}
	return bitrev16(crc);
}
EXPORT_SYMBOL(bif_crc_ccitt);

static u16 bif_object_crc_ccitt(const struct bif_object *object)
{
	u16 crc = 0xFFFF;
	int i;

	crc = crc_ccitt_byte(crc, bitrev8(object->type));
	crc = crc_ccitt_byte(crc, bitrev8(object->version));
	crc = crc_ccitt_byte(crc, bitrev8(object->manufacturer_id >> 8));
	crc = crc_ccitt_byte(crc, bitrev8(object->manufacturer_id));
	crc = crc_ccitt_byte(crc, bitrev8(object->length >> 8));
	crc = crc_ccitt_byte(crc, bitrev8(object->length));

	for (i = 0; i < object->length - 8; i++)
		crc = crc_ccitt_byte(crc, bitrev8(object->data[i]));

	return bitrev16(crc);
}

static int bif_check_task(struct bif_slave *slave, unsigned int task)
{
	if (IS_ERR_OR_NULL(slave)) {
		pr_err("Invalid slave pointer=%ld\n", PTR_ERR(slave));
		return -EINVAL;
	}

	return _bif_check_task(slave->sdev, task);
}

/**
 * bif_request_irq() - request a BIF slave IRQ by slave task number
 * @slave:	BIF slave handle
 * @task:	BIF task number of the IRQ inside of the slave.  This
 *		corresponds to the slave control channel specified for a given
 *		BIF function inside of the slave.
 * @nb:		Notifier block to call when the IRQ fires
 *
 * This function registers a notifier block to call when the BIF slave interrupt
 * is triggered and also enables the interrupt.  The interrupt is enabled inside
 * of the BIF slave's slave control function and also the BIF bus is put into
 * interrupt mode.
 *
 * Returns 0 for success or errno if an error occurred.
 */
int bif_request_irq(struct bif_slave *slave, unsigned int task,
			struct notifier_block *nb)
{
	int rc;
	u16 addr;
	u8 reg, mask;

	rc = bif_check_task(slave, task);
	if (rc) {
		pr_err("Invalid slave or task, rc=%d\n", rc);
		return rc;
	}

	bif_slave_ctrl_lock(slave);

	rc = blocking_notifier_chain_register(
		&slave->sdev->slave_ctrl_function->irq_notifier_list[task], nb);
	if (rc) {
		pr_err("Notifier registration failed, rc=%d\n", rc);
		goto done;
	}

	/* Enable the interrupt within the slave */
	mask = BIT(task % SLAVE_CTRL_TASKS_PER_SET);
	addr = SLAVE_CTRL_FUNC_IRQ_EN_ADDR(
		slave->sdev->slave_ctrl_function->slave_ctrl_pointer, task);
	if (task / SLAVE_CTRL_TASKS_PER_SET == 0) {
		/* Set global interrupt enable. */
		mask |= BIT(0);
	}
	rc = _bif_slave_read(slave->sdev, addr, &reg, 1);
	if (rc) {
		pr_err("BIF slave register read failed, rc=%d\n", rc);
		goto notifier_unregister;
	}
	reg |= mask;
	rc = _bif_slave_write(slave->sdev, addr, &reg, 1);
	if (rc) {
		pr_err("BIF slave register write failed, rc=%d\n", rc);
		goto notifier_unregister;
	}

	/* Set global interrupt enable if task not in set 0. */
	if (task / SLAVE_CTRL_TASKS_PER_SET != 0) {
		mask = BIT(0);
		addr = SLAVE_CTRL_FUNC_IRQ_EN_ADDR(
		       slave->sdev->slave_ctrl_function->slave_ctrl_pointer, 0);
		rc = _bif_slave_read(slave->sdev, addr, &reg, 1);
		if (rc) {
			pr_err("BIF slave register read failed, rc=%d\n", rc);
			goto notifier_unregister;
		}
		reg |= mask;
		rc = _bif_slave_write(slave->sdev, addr, &reg, 1);
		if (rc) {
			pr_err("BIF slave register write failed, rc=%d\n", rc);
			goto notifier_unregister;
		}
	}

	rc = slave->sdev->bdev->desc->ops->set_bus_state(slave->sdev->bdev,
		BIF_BUS_STATE_INTERRUPT);
	if (rc) {
		pr_err("Could not set BIF bus to interrupt mode, rc=%d\n", rc);
		goto notifier_unregister;
	}

	slave->sdev->bdev->irq_count++;
done:
	bif_slave_ctrl_unlock(slave);

	return rc;

notifier_unregister:
	blocking_notifier_chain_unregister(
		&slave->sdev->slave_ctrl_function->irq_notifier_list[task],
		nb);
	bif_slave_ctrl_unlock(slave);

	return rc;

}
EXPORT_SYMBOL(bif_request_irq);

/**
 * bif_free_irq() - free a BIF slave IRQ by slave task number
 * @slave:	BIF slave handle
 * @task:	BIF task number of the IRQ inside of the slave.  This
 *		corresponds to the slave control channel specified for a given
 *		BIF function inside of the slave.
 * @nb:		Notifier block previously registered with this interrupt
 *
 * This function unregisters a notifier block that was previously registered
 * with bif_request_irq().
 *
 * Returns 0 for success or errno if an error occurred.
 */
int bif_free_irq(struct bif_slave *slave, unsigned int task,
			struct notifier_block *nb)
{
	int rc;
	u16 addr;
	u8 reg;

	rc = bif_check_task(slave, task);
	if (rc) {
		pr_err("Invalid slave or task, rc=%d\n", rc);
		return rc;
	}

	bif_slave_ctrl_lock(slave);

	/* Disable the interrupt within the slave */
	reg = BIT(task % SLAVE_CTRL_TASKS_PER_SET);
	addr = SLAVE_CTRL_FUNC_IRQ_CLEAR_ADDR(
		slave->sdev->slave_ctrl_function->slave_ctrl_pointer, task);
	rc = _bif_slave_write(slave->sdev, addr, &reg, 1);
	if (rc) {
		pr_err("BIF slave register write failed, rc=%d\n", rc);
		goto done;
	}

	rc = blocking_notifier_chain_unregister(
		&slave->sdev->slave_ctrl_function->irq_notifier_list[task], nb);
	if (rc) {
		pr_err("Notifier unregistration failed, rc=%d\n", rc);
		goto done;
	}

	slave->sdev->bdev->irq_count--;

	if (slave->sdev->bdev->irq_count == 0) {
		bif_cancel_irq_mode_work(slave->sdev->bdev);
	} else if (slave->sdev->bdev->irq_count < 0) {
		pr_err("Unbalanced IRQ free.\n");
		rc = -EINVAL;
		slave->sdev->bdev->irq_count = 0;
	}
done:
	bif_slave_ctrl_unlock(slave);

	return rc;
}
EXPORT_SYMBOL(bif_free_irq);

/**
 * bif_trigger_task() - trigger a task within a BIF slave
 * @slave:	BIF slave handle
 * @task:	BIF task inside of the slave to trigger.  This corresponds to
 *		the slave control channel specified for a given BIF function
 *		inside of the slave.
 *
 * Returns 0 for success or errno if an error occurred.
 */
int bif_trigger_task(struct bif_slave *slave, unsigned int task)
{
	int rc;
	u16 addr;
	u8 reg;

	rc = bif_check_task(slave, task);
	if (rc) {
		pr_err("Invalid slave or task, rc=%d\n", rc);
		return rc;
	}

	bif_slave_ctrl_lock(slave);

	/* Trigger the task within the slave. */
	reg = BIT(task % SLAVE_CTRL_TASKS_PER_SET);
	addr = SLAVE_CTRL_FUNC_TASK_TRIGGER_ADDR(
		slave->sdev->slave_ctrl_function->slave_ctrl_pointer, task);
	rc = _bif_slave_write(slave->sdev, addr, &reg, 1);
	if (rc) {
		pr_err("BIF slave register write failed, rc=%d\n", rc);
		goto done;
	}

done:
	bif_slave_ctrl_unlock(slave);

	return rc;
}
EXPORT_SYMBOL(bif_trigger_task);

/**
 * bif_enable_auto_task() - enable task auto triggering for the specified task
 * @slave:	BIF slave handle
 * @task:	BIF task inside of the slave to configure for automatic
 *		triggering.  This corresponds to the slave control channel
 *		specified for a given BIF function inside of the slave.
 *
 * Returns 0 for success or errno if an error occurred.
 */
int bif_enable_auto_task(struct bif_slave *slave, unsigned int task)
{
	int rc;

	if (IS_ERR_OR_NULL(slave)) {
		pr_err("Invalid slave pointer=%ld\n", PTR_ERR(slave));
		return -EINVAL;
	}

	bif_slave_ctrl_lock(slave);
	rc = _bif_enable_auto_task(slave->sdev, task);
	bif_slave_ctrl_unlock(slave);

	return rc;
}
EXPORT_SYMBOL(bif_enable_auto_task);

/**
 * bif_disable_auto_task() - disable task auto triggering for the specified task
 * @slave:	BIF slave handle
 * @task:	BIF task inside of the slave to stop automatic triggering on.
 *		This corresponds to the slave control channel specified for a
 *		given BIF function inside of the slave.
 *
 * This function should be called after bif_enable_auto_task() in a paired
 * fashion.
 *
 * Returns 0 for success or errno if an error occurred.
 */
int bif_disable_auto_task(struct bif_slave *slave, unsigned int task)
{
	int rc;

	if (IS_ERR_OR_NULL(slave)) {
		pr_err("Invalid slave pointer=%ld\n", PTR_ERR(slave));
		return -EINVAL;
	}

	bif_slave_ctrl_lock(slave);
	rc = _bif_disable_auto_task(slave->sdev, task);
	bif_slave_ctrl_unlock(slave);

	return rc;
}
EXPORT_SYMBOL(bif_disable_auto_task);

/**
 * bif_task_is_busy() - checks the state of a BIF slave task
 * @slave:	BIF slave handle
 * @task:	BIF task inside of the slave to trigger.  This corresponds to
 *		the slave control channel specified for a given	BIF function
 *		inside of the slave.
 *
 * Returns 1 if the task is busy, 0 if it is not busy, and errno on error.
 */
int bif_task_is_busy(struct bif_slave *slave, unsigned int task)
{
	int rc;

	if (IS_ERR_OR_NULL(slave)) {
		pr_err("Invalid slave pointer=%ld\n", PTR_ERR(slave));
		return -EINVAL;
	}

	bif_slave_ctrl_lock(slave);
	rc = _bif_task_is_busy(slave->sdev, task);
	bif_slave_ctrl_unlock(slave);

	return rc;
}
EXPORT_SYMBOL(bif_task_is_busy);

static int bif_slave_notify_irqs(struct bif_slave_dev *sdev, int set, u8 val)
{
	int rc = 0;
	int i, task;

	for (i = 0; i < SLAVE_CTRL_TASKS_PER_SET; i++) {
		if (val & (1 << i)) {
			task = set * SLAVE_CTRL_TASKS_PER_SET + i;

			rc = blocking_notifier_call_chain(
			    &sdev->slave_ctrl_function->irq_notifier_list[task],
			    task, sdev->bdev);
			rc = notifier_to_errno(rc);
			if (rc)
				pr_err("Notification failed for task %d\n",
					task);
		}
	}

	return rc;
}

static int bif_slave_handle_irq(struct bif_slave_dev *sdev)
{
	struct bif_ctrl_dev *bdev = sdev->bdev;
	bool resp = false;
	int rc = 0;
	int i;
	u16 addr;
	u8 reg;

	mutex_lock(&sdev->bdev->mutex);
	bif_cancel_irq_mode_work(sdev->bdev);

	rc = bif_select_slave(sdev);
	if (rc) {
		pr_err("Could not select slave, rc=%d\n", rc);
		goto done;
	}

	/* Check overall slave interrupt status. */
	rc = bdev->desc->ops->bus_transaction_query(bdev, BIF_TRANS_BC,
						    BIF_CMD_ISTS, &resp);
	if (rc) {
		pr_err("Could not query slave interrupt status, rc=%d\n", rc);
		goto done;
	}

	if (resp) {
		for (i = 0; i < sdev->slave_ctrl_function->task_count
					/ SLAVE_CTRL_TASKS_PER_SET; i++) {
			addr = sdev->slave_ctrl_function->slave_ctrl_pointer
				+ 4 * i + 1;
			rc = _bif_slave_read(sdev, addr, &reg, 1);
			if (rc) {
				pr_err("BIF slave register read failed, rc=%d\n",
					rc);
				goto done;
			}

			/* Ensure that interrupts are pending in the set. */
			if (reg != 0x00) {
				/*
				 * Release mutex before notifying consumers so
				 * that they can use the bus.
				 */
				mutex_unlock(&sdev->bdev->mutex);
				rc = bif_slave_notify_irqs(sdev, i, reg);
				if (rc) {
					pr_err("BIF slave irq notification failed, rc=%d\n",
						rc);
					goto notification_failed;
				}
				mutex_lock(&sdev->bdev->mutex);

				rc = bif_select_slave(sdev);
				if (rc) {
					pr_err("Could not select slave, rc=%d\n",
						rc);
					goto done;
				}

				/* Clear all interrupts in this set. */
				rc = _bif_slave_write(sdev, addr, &reg, 1);
				if (rc) {
					pr_err("BIF slave register write failed, rc=%d\n",
						rc);
					goto done;
				}
			}
		}
	}

done:
	bif_schedule_irq_mode_work(sdev->bdev);
	mutex_unlock(&sdev->bdev->mutex);
notification_failed:
	if (rc == 0)
		rc = resp;
	return rc;
}

/**
 * bif_ctrl_notify_slave_irq() - notify the BIF framework that a slave interrupt
 *				was received by a BIF controller
 * @bdev:	BIF controller device pointer
 *
 * This function should only be called from a BIF controller driver.
 *
 * Returns 0 for success or errno if an error occurred.
 */
int bif_ctrl_notify_slave_irq(struct bif_ctrl_dev *bdev)
{
	struct bif_slave_dev *sdev;
	int rc = 0, handled = 0;

	if (IS_ERR_OR_NULL(bdev))
		return -EINVAL;

	mutex_lock(&bif_sdev_list_mutex);

	list_for_each_entry(sdev, &bif_sdev_list, list) {
		if (sdev->bdev == bdev && sdev->present) {
			rc = bif_slave_handle_irq(sdev);
			if (rc < 0) {
				pr_err("Could not handle BIF slave irq, rc=%d\n",
					rc);
				break;
			}
			handled += rc;
		}
	}

	mutex_unlock(&bif_sdev_list_mutex);

	if (handled == 0)
		pr_info("Spurious BIF slave interrupt detected.\n");

	if (rc > 0)
		rc = 0;

	return rc;
}
EXPORT_SYMBOL(bif_ctrl_notify_slave_irq);

/**
 * bif_ctrl_notify_battery_changed() - notify the BIF framework that a battery
 *				pack has been inserted or removed
 * @bdev:	BIF controller device pointer
 *
 * This function should only be called from a BIF controller driver.
 *
 * Returns 0 for success or errno if an error occurred.
 */
int bif_ctrl_notify_battery_changed(struct bif_ctrl_dev *bdev)
{
	int rc = 0;
	int present;

	if (IS_ERR_OR_NULL(bdev))
		return -EINVAL;

	if (bdev->desc->ops->get_battery_presence) {
		present = bdev->desc->ops->get_battery_presence(bdev);
		if (present < 0) {
			pr_err("Could not determine battery presence, rc=%d\n",
				rc);
			return rc;
		}

		if (bdev->battery_present == !!present)
			return 0;

		bdev->battery_present = present;

		rc = blocking_notifier_call_chain(&bdev->bus_change_notifier,
			present ? BIF_BUS_EVENT_BATTERY_INSERTED
				: BIF_BUS_EVENT_BATTERY_REMOVED, bdev);
		if (rc)
			pr_err("Call chain noification failed, rc=%d\n", rc);
	}

	return rc;
}
EXPORT_SYMBOL(bif_ctrl_notify_battery_changed);

/**
 * bif_ctrl_signal_battery_changed() - notify the BIF framework that a battery
 *				pack has been inserted or removed
 * @ctrl:	BIF controller consumer handle
 *
 * This function should only be called by a BIF consumer driver on systems where
 * the BIF controller driver is unable to determine when a battery is inserted
 * or removed.
 *
 * Returns 0 for success or errno if an error occurred.
 */
int bif_ctrl_signal_battery_changed(struct bif_ctrl *ctrl)
{
	if (IS_ERR_OR_NULL(ctrl))
		return -EINVAL;

	return bif_ctrl_notify_battery_changed(ctrl->bdev);
}
EXPORT_SYMBOL(bif_ctrl_signal_battery_changed);

/**
 * bif_ctrl_notifier_register() - register a notifier block to be called when
 *				a battery pack is inserted or removed
 * @ctrl:	BIF controller consumer handle
 *
 * The value passed into the notifier when it is called is one of
 * enum bif_bus_event.
 *
 * Returns 0 for success or errno if an error occurred.
 */
int bif_ctrl_notifier_register(struct bif_ctrl *ctrl, struct notifier_block *nb)
{
	int rc;

	if (IS_ERR_OR_NULL(ctrl))
		return -EINVAL;

	rc = blocking_notifier_chain_register(&ctrl->bdev->bus_change_notifier,
					      nb);
	if (rc)
		pr_err("Notifier registration failed, rc=%d\n", rc);

	return rc;
}
EXPORT_SYMBOL(bif_ctrl_notifier_register);

/**
 * bif_ctrl_notifier_unregister() - unregister a battery status change notifier
 *				block that was previously registered
 * @ctrl:	BIF controller consumer handle
 *
 * Returns 0 for success or errno if an error occurred.
 */
int bif_ctrl_notifier_unregister(struct bif_ctrl *ctrl,
				 struct notifier_block *nb)
{
	int rc;

	if (IS_ERR_OR_NULL(ctrl))
		return -EINVAL;

	rc =
	    blocking_notifier_chain_unregister(&ctrl->bdev->bus_change_notifier,
						nb);
	if (rc)
		pr_err("Notifier unregistration failed, rc=%d\n", rc);

	return rc;
}
EXPORT_SYMBOL(bif_ctrl_notifier_unregister);

/**
 * bif_get_bus_handle() - returns the BIF controller consumer handle associated
 *			with a BIF slave handle
 * @slave:	BIF slave handle
 *
 * Note, bif_ctrl_put() should never be called for the pointer output by
 * bif_get_bus_handle().
 */
struct bif_ctrl *bif_get_bus_handle(struct bif_slave *slave)
{
	if (IS_ERR_OR_NULL(slave))
		return ERR_PTR(-EINVAL);

	return &slave->ctrl;
}
EXPORT_SYMBOL(bif_get_bus_handle);

/**
 * bif_ctrl_count() - returns the number of registered BIF controllers
 */
int bif_ctrl_count(void)
{
	struct bif_ctrl_dev *bdev;
	int count = 0;

	mutex_lock(&bif_ctrl_list_mutex);

	list_for_each_entry(bdev, &bif_ctrl_list, list) {
		count++;
	}
	mutex_unlock(&bif_ctrl_list_mutex);

	return count;
}
EXPORT_SYMBOL(bif_ctrl_count);

/**
 * bif_ctrl_get_by_id() - get a handle for the id'th BIF controller registered
 *			in the system
 * @id:	Arbitrary number associated with the BIF bus in the system
 *
 * id must be in the range [0, bif_ctrl_count() - 1].  This function should only
 * need to be called by a BIF consumer that is unable to link to a given BIF
 * controller via a device tree binding.
 *
 * Returns a BIF controller consumer handle if successful or an ERR_PTR if not.
 */
struct bif_ctrl *bif_ctrl_get_by_id(unsigned int id)
{
	struct bif_ctrl_dev *bdev;
	struct bif_ctrl_dev *bdev_found = NULL;
	struct bif_ctrl *ctrl = ERR_PTR(-ENODEV);

	mutex_lock(&bif_ctrl_list_mutex);

	list_for_each_entry(bdev, &bif_ctrl_list, list) {
		if (id == 0) {
			bdev_found = bdev;
			break;
		}
		id--;
	}
	mutex_unlock(&bif_ctrl_list_mutex);

	if (bdev_found) {
		ctrl = kzalloc(sizeof(*ctrl), GFP_KERNEL);
		if (!ctrl) {
			pr_err("Bus handle allocation failed\n");
			ctrl = ERR_PTR(-ENOMEM);
		} else {
			ctrl->bdev = bdev_found;
		}
	}

	return ctrl;
}
EXPORT_SYMBOL(bif_ctrl_get_by_id);

/**
 * bif_ctrl_get() - get a handle for the BIF controller that is linked to the
 *			consumer device in the device tree
 * @consumer_dev:	Pointer to the consumer's device
 *
 * In order to use this function, the BIF consumer's device must specify the
 * "qcom,bif-ctrl" property in its device tree node which points to a BIF
 * controller device node.
 *
 * Returns a BIF controller consumer handle if successful or an ERR_PTR if not.
 * If the BIF controller linked to the consumer device has not yet probed, then
 * ERR_PTR(-EPROBE_DEFER) is returned.
 */
struct bif_ctrl *bif_ctrl_get(struct device *consumer_dev)
{
	struct device_node *ctrl_node = NULL;
	struct bif_ctrl_dev *bdev_found = NULL;
	struct bif_ctrl *ctrl = ERR_PTR(-EPROBE_DEFER);
	struct bif_ctrl_dev *bdev = NULL;

	if (!consumer_dev || !consumer_dev->of_node) {
		pr_err("Invalid device node\n");
		return ERR_PTR(-EINVAL);
	}

	ctrl_node = of_parse_phandle(consumer_dev->of_node, "qcom,bif-ctrl", 0);
	if (!ctrl_node) {
		pr_err("Could not find qcom,bif-ctrl property in %s\n",
			consumer_dev->of_node->full_name);
		return ERR_PTR(-ENXIO);
	}

	mutex_lock(&bif_ctrl_list_mutex);
	list_for_each_entry(bdev, &bif_ctrl_list, list) {
		if (bdev->ctrl_dev && bdev->ctrl_dev->of_node == ctrl_node) {
			bdev_found = bdev;
			break;
		}
	}
	mutex_unlock(&bif_ctrl_list_mutex);

	if (bdev_found) {
		ctrl = kzalloc(sizeof(*ctrl), GFP_KERNEL);
		if (!ctrl) {
			pr_err("Bus handle allocation failed\n");
			ctrl = ERR_PTR(-ENOMEM);
		} else {
			ctrl->bdev = bdev_found;
		}
	}

	return ctrl;
}
EXPORT_SYMBOL(bif_ctrl_get);

/**
 * bif_ctrl_put() - frees a BIF controller handle
 * @ctrl:	BIF controller consumer handle
 */
void bif_ctrl_put(struct bif_ctrl *ctrl)
{
	if (!IS_ERR_OR_NULL(ctrl) && ctrl->exclusive_lock)
		mutex_unlock(&ctrl->bdev->mutex);
	kfree(ctrl);
}
EXPORT_SYMBOL(bif_ctrl_put);

static bool bif_slave_object_match(const struct bif_object *object,
		const struct bif_match_criteria *criteria)
{
	return (object->type == criteria->obj_type)
		&& (object->version == criteria->obj_version
			|| !(criteria->match_mask & BIF_MATCH_OBJ_VERSION))
		&& (object->manufacturer_id == criteria->obj_manufacturer_id
		    || !(criteria->match_mask & BIF_MATCH_OBJ_MANUFACTURER_ID));
}

/*
 * Returns true if all parameters are matched, otherwise false.
 * function_type and function_version mean that their exists some function in
 * the slave which has the specified type and subtype.  ctrl == NULL is treated
 * as a wildcard.
 */
static bool bif_slave_match(struct bif_ctrl *ctrl,
	struct bif_slave_dev *sdev, const struct bif_match_criteria *criteria)
{
	int i, type, version;
	struct bif_object *object;
	bool function_found = false;
	bool object_found = false;

    if (!ctrl)
        return false;

	if (ctrl->bdev != sdev->bdev)
		return false;

	if (!sdev->present
	    && (!(criteria->match_mask & BIF_MATCH_IGNORE_PRESENCE)
		|| ((criteria->match_mask & BIF_MATCH_IGNORE_PRESENCE)
		    && !criteria->ignore_presence)))
		return false;

	if ((criteria->match_mask & BIF_MATCH_MANUFACTURER_ID)
	    && sdev->l1_data.manufacturer_id != criteria->manufacturer_id)
		return false;

	if ((criteria->match_mask & BIF_MATCH_PRODUCT_ID)
	    && sdev->l1_data.product_id != criteria->product_id)
		return false;

	if (criteria->match_mask & BIF_MATCH_FUNCTION_TYPE) {
		if (!sdev->function_directory)
			return false;
		for (i = 0; i < sdev->l1_data.length / 4; i++) {
			type = sdev->function_directory[i].function_type;
			version = sdev->function_directory[i].function_version;
			if (type == criteria->function_type &&
				(version == criteria->function_version
					|| !(criteria->match_mask
					       & BIF_MATCH_FUNCTION_VERSION))) {
				function_found = true;
				break;
			}
		}
		if (!function_found)
			return false;
	}

	if (criteria->match_mask & BIF_MATCH_OBJ_TYPE) {
		if (!sdev->nvm_function)
			return false;
		bif_ctrl_lock(ctrl);
		list_for_each_entry(object, &sdev->nvm_function->object_list,
					list) {
			if (bif_slave_object_match(object, criteria)) {
				object_found = true;
				break;
			}
		}
		bif_ctrl_unlock(ctrl);
		if (!object_found)
			return false;
	}

	return true;
}

/**
 * bif_slave_match_count() - returns the number of slaves associated with the
 *			specified BIF controller which fit the matching
 *			criteria
 * @ctrl:		BIF controller consumer handle
 * @match_criteria:	Matching criteria used to filter slaves
 */
int bif_slave_match_count(struct bif_ctrl *ctrl,
			const struct bif_match_criteria *match_criteria)
{
	struct bif_slave_dev *sdev;
	int count = 0;

	mutex_lock(&bif_sdev_list_mutex);

	list_for_each_entry(sdev, &bif_sdev_list, list) {
		if (bif_slave_match(ctrl, sdev, match_criteria))
			count++;
	}

	mutex_unlock(&bif_sdev_list_mutex);

	return count;
}
EXPORT_SYMBOL(bif_slave_match_count);

/**
 * bif_slave_match_get() - get a slave handle for the id'th slave associated
 *			with the specified BIF controller which fits the
 *			matching criteria
 * @ctrl:		BIF controller consumer handle
 * @id:			Index into the set of matching slaves
 * @match_criteria:	Matching criteria used to filter slaves
 *
 * id must be in the range [0, bif_slave_match_count(ctrl, match_criteria) - 1].
 *
 * Returns a BIF slave handle if successful or an ERR_PTR if not.
 */
struct bif_slave *bif_slave_match_get(struct bif_ctrl *ctrl,
	unsigned int id, const struct bif_match_criteria *match_criteria)
{
	struct bif_slave_dev *sdev;
	struct bif_slave *slave = ERR_PTR(-ENODEV);
	struct bif_slave_dev *sdev_found = NULL;
	int count = 0;

	mutex_lock(&bif_sdev_list_mutex);

	list_for_each_entry(sdev, &bif_sdev_list, list) {
		if (bif_slave_match(ctrl, sdev, match_criteria))
			count++;
		if (count == id + 1) {
			sdev_found = sdev;
			break;
		}
	}

	mutex_unlock(&bif_sdev_list_mutex);

	if (sdev_found) {
		slave = kzalloc(sizeof(*slave), GFP_KERNEL);
		if (!slave) {
			pr_err("Slave allocation failed\n");
			slave = ERR_PTR(-ENOMEM);
		} else {
			slave->sdev = sdev_found;
			slave->ctrl.bdev = sdev_found->bdev;
		}
	}

	return slave;
}
EXPORT_SYMBOL(bif_slave_match_get);

/**
 * bif_slave_put() - frees a BIF slave handle
 * @slave:	BIF slave handle
 */
void bif_slave_put(struct bif_slave *slave)
{
	if (!IS_ERR_OR_NULL(slave) && slave->ctrl.exclusive_lock)
		mutex_unlock(&slave->sdev->bdev->mutex);
	kfree(slave);
}
EXPORT_SYMBOL(bif_slave_put);

/**
 * bif_slave_find_function() - get the function pointer and version of a
 *			BIF function if it is present on the specified slave
 * @slave:		BIF slave handle
 * @function:		BIF function to search for inside of the slave
 * @version:		If the function is found, then 'version' is set to the
 *			version value of the function
 * @function_pointer:	If the function is found, then 'function_pointer' is set
 *			to the BIF slave address of the function
 *
 * Returns 0 for success or errno if an error occurred.  If the function is not
 * found in the slave, then -ENODEV is returned.
 */
int bif_slave_find_function(struct bif_slave *slave, u8 function, u8 *version,
				u16 *function_pointer)
{
	int rc = -ENODEV;
	struct bif_ddb_l2_data *func;
	int i;

	if (IS_ERR_OR_NULL(slave) || IS_ERR_OR_NULL(version)
	    || IS_ERR_OR_NULL(function_pointer)) {
		pr_err("Invalid pointer input.\n");
		return -EINVAL;
	}

	func = slave->sdev->function_directory;

	for (i = 0; i < slave->sdev->l1_data.length / 4; i++) {
		if (function == func[i].function_type) {
			*version = func[i].function_version;
			*function_pointer = func[i].function_pointer;
			rc = 0;
			break;
		}
	}

	return rc;
}
EXPORT_SYMBOL(bif_slave_find_function);

static bool bif_object_match(const struct bif_object *object,
		const struct bif_obj_match_criteria *criteria)
{
	return (object->type == criteria->type
			|| !(criteria->match_mask & BIF_OBJ_MATCH_TYPE))
		&& (object->version == criteria->version
			|| !(criteria->match_mask & BIF_OBJ_MATCH_VERSION))
		&& (object->manufacturer_id == criteria->manufacturer_id
		    || !(criteria->match_mask & BIF_OBJ_MATCH_MANUFACTURER_ID));
}

/**
 * bif_object_match_count() - returns the number of objects associated with the
 *			specified BIF slave which fit the matching criteria
 * @slave:		BIF slave handle
 * @match_criteria:	Matching criteria used to filter objects
 */
int bif_object_match_count(struct bif_slave *slave,
			const struct bif_obj_match_criteria *match_criteria)
{
	struct bif_object *object;
	int count = 0;

	if (IS_ERR_OR_NULL(slave) || IS_ERR_OR_NULL(match_criteria)) {
		pr_err("Invalid pointer input.\n");
		return -EINVAL;
	}

	if (!slave->sdev->nvm_function)
		return 0;

	bif_slave_ctrl_lock(slave);
	list_for_each_entry(object, &slave->sdev->nvm_function->object_list,
			    list) {
		if (bif_object_match(object, match_criteria))
			count++;
	}
	bif_slave_ctrl_unlock(slave);

	return count;
}
EXPORT_SYMBOL(bif_object_match_count);

/**
 * bif_object_match_get() - get a BIF object handle for the id'th object found
 *			in the non-volatile memory of the specified BIF slave
 *			which fits the matching criteria
 * @slave:		BIF slave handle
 * @id:			Index into the set of matching objects
 * @match_criteria:	Matching criteria used to filter objects
 *
 * id must be in range [0, bif_object_match_count(slave, match_criteria) - 1].
 *
 * Returns a BIF object handle if successful or an ERR_PTR if not.  This handle
 * must be freed using bif_object_put() when it is no longer needed.
 */
struct bif_object *bif_object_match_get(struct bif_slave *slave,
	unsigned int id, const struct bif_obj_match_criteria *match_criteria)
{
	struct bif_object *object;
	struct bif_object *object_found = NULL;
	struct bif_object *object_consumer = ERR_PTR(-ENODEV);
	int count = 0;

	if (IS_ERR_OR_NULL(slave) || IS_ERR_OR_NULL(match_criteria)) {
		pr_err("Invalid pointer input.\n");
		return ERR_PTR(-EINVAL);
	}

	if (!slave->sdev->nvm_function)
		return object_consumer;

	bif_slave_ctrl_lock(slave);
	list_for_each_entry(object, &slave->sdev->nvm_function->object_list,
			    list) {
		if (bif_object_match(object, match_criteria))
			count++;
		if (count == id + 1) {
			object_found = object;
			break;
		}
	}

	if (object_found) {
		object_consumer = kmemdup(object_found,
					sizeof(*object_consumer), GFP_KERNEL);
		if (!object_consumer) {
			pr_err("out of memory\n");
			object_consumer = ERR_PTR(-ENOMEM);
			goto done;
		}

		object_consumer->data = kmemdup(object_found->data,
					  object_found->length - 8, GFP_KERNEL);
		if (!object_consumer->data) {
			pr_err("out of memory\n");
			kfree(object_consumer);
			object_consumer = ERR_PTR(-ENOMEM);
			goto done;
		}

		/*
		 * Use prev pointer in consumer struct to point to original
		 * struct in the internal linked list.
		 */
		object_consumer->list.prev = &object_found->list;
	}

done:
	bif_slave_ctrl_unlock(slave);

	return object_consumer;

}
EXPORT_SYMBOL(bif_object_match_get);

/**
 * bif_object_put() - frees the memory allocated for a BIF object pointer
 *			returned by bif_object_match_get()
 * @object:		BIF object to free
 */
void bif_object_put(struct bif_object *object)
{
	if (object)
		kfree(object->data);
	kfree(object);
}
EXPORT_SYMBOL(bif_object_put);

/* Copies the contents of object into buf following MIPI-BIF formatting. */
static void bif_object_flatten(u8 *buf, const struct bif_object *object)
{
	buf[0]			= object->type;
	buf[1]			= object->version;
	buf[2]			= object->manufacturer_id >> 8;
	buf[3]			= object->manufacturer_id;
	buf[4]			= object->length >> 8;
	buf[5]			= object->length;
	memcpy(&buf[6], object->data, object->length - 8);
	buf[object->length - 2]	= object->crc >> 8;
	buf[object->length - 1]	= object->crc;
}

/**
 * bif_object_write() - writes a new BIF object at the end of the object list in
 *			the non-volatile memory of a slave
 * @slave:		BIF slave handle
 * @type:		Type of the object
 * @version:		Version of the object
 * @manufacturer_id:	Manufacturer ID number allocated by MIPI
 * @data:		Data contained in the object
 * @data_len:		Length of the data
 *
 * Returns 0 on success or errno on failure.  This function will fail if the NVM
 * lock points to an offset after the BIF object list terminator (0x00).
 */
int bif_object_write(struct bif_slave *slave, u8 type, u8 version,
			u16 manufacturer_id, const u8 *data, int data_len)
{
	struct bif_object *object;
	struct bif_object *tail_object;
	struct bif_nvm_function	*nvm;
	int rc;
	int add_null = 0;
	u16 offset = 0;
	u8 *buf;

	if (IS_ERR_OR_NULL(slave) || IS_ERR_OR_NULL(data)) {
		pr_err("Invalid input pointer\n");
		return -EINVAL;
	}

	nvm = slave->sdev->nvm_function;
	if (!nvm) {
		pr_err("BIF slave has no NVM function\n");
		return -ENODEV;
	}

	bif_slave_ctrl_lock(slave);
	if (nvm->object_count > 0) {
		tail_object = list_entry(nvm->object_list.prev,
					struct bif_object, list);
		offset = tail_object->addr - nvm->nvm_base_address
				+ tail_object->length;
	}

	if (offset < nvm->nvm_lock_offset) {
		pr_err("Cannot write BIF object to NVM because the end of the object list is locked (end=%d < lock=%d)\n",
			offset, nvm->nvm_lock_offset);
		rc = -EPERM;
		goto error_unlock;
	} else if (offset + data_len + 8 > nvm->nvm_size) {
		pr_err("Cannot write BIF object to NVM because there is not enough remaining space (size=%d > remaining=%d)\n",
			data_len + 8, nvm->nvm_size - offset);
		rc = -EINVAL;
		goto error_unlock;
	}

	if (offset + data_len + 8 < nvm->nvm_size)
		add_null = 1;
	object = kzalloc(sizeof(*object), GFP_KERNEL);
	if (!object) {
		pr_err("kzalloc failed\n");
		rc = -ENOMEM;
		goto error_unlock;
	}

	object->data = kzalloc(data_len, GFP_KERNEL);
	if (!object->data) {
		pr_err("kzalloc failed\n");
		rc = -ENOMEM;
		goto free_object;
	}

	buf = kzalloc(data_len + 8 + add_null, GFP_KERNEL);
	if (!buf) {
		pr_err("kzalloc failed\n");
		rc = -ENOMEM;
		goto free_data;
	}

	object->type		= type;
	object->version		= version;
	object->manufacturer_id	= manufacturer_id;
	object->length		= data_len + 8;
	memcpy(object->data, data, data_len);
	object->crc		= bif_object_crc_ccitt(object);
	object->addr		= offset + nvm->nvm_base_address;

	bif_object_flatten(buf, object);
	if (add_null)
		buf[object->length] = BIF_OBJ_END_OF_LIST;

	rc = _bif_slave_nvm_raw_write(slave->sdev, offset, buf,
					object->length + add_null);
	if (rc < 0) {
		pr_err("NVM write failed, rc=%d\n", rc);
		kfree(buf);
		goto free_data;
	}
	kfree(buf);

	list_add_tail(&object->list, &nvm->object_list);
	nvm->object_count++;

	bif_slave_ctrl_unlock(slave);

	return rc;

free_data:
	kfree(object->data);
free_object:
	kfree(object);
error_unlock:
	bif_slave_ctrl_unlock(slave);

	return rc;

}
EXPORT_SYMBOL(bif_object_write);

/*
 * Returns a pointer to the internal object referenced by a consumer object
 * if it exists.  Returns NULL if the internal object cannot be found.
 */
static struct bif_object *bif_object_consumer_search(
	struct bif_nvm_function *nvm, const struct bif_object *consumer_object)
{
	struct bif_object *object = NULL;
	struct bif_object *search_object;

	/*
	 * Internal struct in object linked list is pointed to by consumer
	 * object list.prev.
	 */
	search_object = list_entry(consumer_object->list.prev,
					struct bif_object, list);

	list_for_each_entry(object, &nvm->object_list, list) {
		if (object == search_object)
			break;
	}

	if (object != search_object)
		return NULL;

	return object;
}

/**
 * bif_object_overwrite() - overwrites an existing BIF object found in the
 *			non-volatile memory of a slave
 * @slave:		BIF slave handle
 * @object:		Existing object in the slave to overwrite
 * @type:		Type of the object
 * @version:		Version of the object
 * @manufacturer_id:	Manufacturer ID number allocated by MIPI
 * @data:		Data contained in the object
 * @data_len:		Length of the data
 *
 * Returns 0 on success or errno on failure.  The data stored within 'object'
 * is updated to the new values upon success.  The new data written to the
 * object must have exactly the same length as the old data (i.e.
 * data_len == object->length - 8).
 *
 * This function will fail if the NVM lock points to an offset after the
 * beginning of the existing BIF object.
 */
int bif_object_overwrite(struct bif_slave *slave,
	struct bif_object *object, u8 type, u8 version,
	u16 manufacturer_id, const u8 *data, int data_len)
{
	struct bif_object *edit_object = NULL;
	struct bif_nvm_function *nvm;
	int rc;
	u16 crc;
	u8 *buf;

	if (IS_ERR_OR_NULL(slave) || IS_ERR_OR_NULL(object)
	    || IS_ERR_OR_NULL(data)) {
		pr_err("Invalid input pointer\n");
		return -EINVAL;
	}

	nvm = slave->sdev->nvm_function;
	if (!nvm) {
		pr_err("BIF slave has no NVM function\n");
		return -ENODEV;
	}

	if (data_len + 8 != object->length) {
		pr_err("New data length=%d is different from existing length=%d\n",
			data_len, object->length - 8);
		return -EINVAL;
	}

	bif_slave_ctrl_lock(slave);

	edit_object = bif_object_consumer_search(nvm, object);
	if (!edit_object) {
		pr_err("BIF object not found within slave\n");
		rc = -EINVAL;
		goto error_unlock;
	}

	if (edit_object->addr - nvm->nvm_base_address < nvm->nvm_lock_offset) {
		pr_err("Cannot overwrite BIF object in NVM because some portion of it is locked\n");
		rc = -EPERM;
		goto error_unlock;
	}

	buf = kzalloc(data_len + 8, GFP_KERNEL);
	if (!buf) {
		pr_err("kzalloc failed\n");
		rc = -ENOMEM;
		goto error_unlock;
	}

	buf[0]			= type;
	buf[1]			= version;
	buf[2]			= manufacturer_id >> 8;
	buf[3]			= manufacturer_id;
	buf[4]			= (data_len + 8) >> 8;
	buf[5]			= data_len + 8;
	memcpy(&buf[6], data, data_len);
	crc			= bif_crc_ccitt(buf, data_len + 6);
	buf[data_len + 6]	= crc >> 8;
	buf[data_len + 7]	= crc;

	rc = _bif_slave_nvm_raw_write(slave->sdev,
		object->addr - nvm->nvm_base_address, buf, data_len + 8);
	if (rc < 0) {
		pr_err("NVM write failed, rc=%d\n", rc);
		kfree(buf);
		goto error_unlock;
	}
	kfree(buf);

	/* Update internal object struct. */
	edit_object->type		= type;
	edit_object->version		= version;
	edit_object->manufacturer_id	= manufacturer_id;
	edit_object->length		= data_len + 8;
	memcpy(edit_object->data, data, data_len);
	edit_object->crc		= crc;

	/* Update consumer object struct. */
	object->type			= type;
	object->version			= version;
	object->manufacturer_id		= manufacturer_id;
	object->length			= data_len + 8;
	memcpy(object->data, data, data_len);
	object->crc			= crc;

error_unlock:
	bif_slave_ctrl_unlock(slave);

	return rc;
}
EXPORT_SYMBOL(bif_object_overwrite);

/**
 * bif_object_delete() - deletes an existing BIF object found in the
 *			non-volatile memory of a slave.  Objects found in the
 *			object list in the NVM of the slave are shifted forward
 *			in order to fill the hole left by the deleted object
 * @slave:		BIF slave handle
 * @object:		Existing object in the slave to delete
 *
 * Returns 0 on success or errno on failure.  bif_object_put() must still be
 * called after this function in order to free the memory in the consumer
 * 'object' struct pointer.
 *
 * This function will fail if the NVM lock points to an offset after the
 * beginning of the existing BIF object.
 */
int bif_object_delete(struct bif_slave *slave, const struct bif_object *object)
{
	struct bif_object *del_object = NULL;
	struct bif_object *tail_object;
	struct bif_nvm_function *nvm;
	bool found = false;
	int pos = 0;
	int rc;
	u8 *buf;

	if (IS_ERR_OR_NULL(slave) || IS_ERR_OR_NULL(object)) {
		pr_err("Invalid input pointer\n");
		return -EINVAL;
	}

	nvm = slave->sdev->nvm_function;
	if (!nvm) {
		pr_err("BIF slave has no NVM function\n");
		return -ENODEV;
	}

	bif_slave_ctrl_lock(slave);

	del_object = bif_object_consumer_search(nvm, object);
	if (!del_object) {
		pr_err("BIF object not found within slave\n");
		rc = -EINVAL;
		goto error_unlock;
	}

	if (del_object->addr - nvm->nvm_base_address < nvm->nvm_lock_offset) {
		pr_err("Cannot delete BIF object in NVM because some portion of it is locked\n");
		rc = -EPERM;
		goto error_unlock;
	}

	buf = kmalloc(nvm->nvm_size, GFP_KERNEL);
	if (!buf) {
		pr_err("kzalloc failed\n");
		rc = -ENOMEM;
		goto error_unlock;
	}

	/*
	 * Copy the contents of objects after the one to be deleted into a flat
	 * array.
	 */
	list_for_each_entry(tail_object, &nvm->object_list, list) {
		if (found) {
			bif_object_flatten(&buf[pos], tail_object);
			pos += tail_object->length;
		} else if (tail_object == del_object) {
			found = true;
		}
	}

	/* Add the list terminator. */
	buf[pos++] = BIF_OBJ_END_OF_LIST;

	rc = _bif_slave_nvm_raw_write(slave->sdev,
		del_object->addr - nvm->nvm_base_address, buf, pos);
	if (rc < 0) {
		pr_err("NVM write failed, rc=%d\n", rc);
		kfree(buf);
		goto error_unlock;
	}
	kfree(buf);

	/* Update the addresses of the objects after the one to be deleted. */
	found = false;
	list_for_each_entry(tail_object, &nvm->object_list, list) {
		if (found)
			tail_object->addr -= del_object->length;
		else if (tail_object == del_object)
			found = true;
	}

	list_del(&del_object->list);
	kfree(del_object->data);
	kfree(del_object);
	nvm->object_count--;

error_unlock:
	bif_slave_ctrl_unlock(slave);

	return rc;
}
EXPORT_SYMBOL(bif_object_delete);

/**
 * bif_slave_read() - read contiguous memory values from a BIF slave
 * @slave:	BIF slave handle
 * @addr:	BIF slave address to begin reading at
 * @buf:	Buffer to fill with memory values
 * @len:	Number of byte to read
 *
 * Returns 0 for success or errno if an error occurred.
 */
int bif_slave_read(struct bif_slave *slave, u16 addr, u8 *buf, int len)
{
	int rc;

	if (IS_ERR_OR_NULL(slave) || IS_ERR_OR_NULL(buf)) {
		pr_err("Invalid pointer input.\n");
		return -EINVAL;
	}

	bif_slave_ctrl_lock(slave);

	rc = _bif_slave_read(slave->sdev, addr, buf, len);
	if (rc)
		pr_err("BIF slave read failed, rc=%d\n", rc);

	bif_slave_ctrl_unlock(slave);

	return rc;
}
EXPORT_SYMBOL(bif_slave_read);

/**
 * bif_slave_write() - write contiguous memory values to a BIF slave
 * @slave:	BIF slave handle
 * @addr:	BIF slave address to begin writing at
 * @buf:	Buffer containing values to write
 * @len:	Number of byte to write
 *
 * Returns 0 for success or errno if an error occurred.
 */
int bif_slave_write(struct bif_slave *slave, u16 addr, u8 *buf, int len)
{
	int rc;

	if (IS_ERR_OR_NULL(slave) || IS_ERR_OR_NULL(buf)) {
		pr_err("Invalid pointer input.\n");
		return -EINVAL;
	}

	bif_slave_ctrl_lock(slave);

	rc = _bif_slave_write(slave->sdev, addr, buf, len);
	if (rc)
		pr_err("BIF slave write failed, rc=%d\n", rc);

	bif_slave_ctrl_unlock(slave);

	return rc;
}
EXPORT_SYMBOL(bif_slave_write);

/**
 * bif_slave_nvm_raw_read() - read contiguous memory values from a BIF slave's
 *		non-volatile memory (NVM)
 * @slave:	BIF slave handle
 * @offset:	Offset from the beginning of BIF slave NVM to begin reading at
 * @buf:	Buffer to fill with memory values
 * @len:	Number of byte to read
 *
 * Returns 0 for success or errno if an error occurred.
 */
int bif_slave_nvm_raw_read(struct bif_slave *slave, u16 offset, u8 *buf,
				int len)
{
	if (IS_ERR_OR_NULL(slave)) {
		pr_err("Invalid slave pointer=%ld\n", PTR_ERR(slave));
		return -EINVAL;
	} else if (IS_ERR_OR_NULL(buf)) {
		pr_err("Invalid buffer pointer=%ld\n", PTR_ERR(buf));
		return -EINVAL;
	} else if (!slave->sdev->nvm_function) {
		pr_err("BIF slave has no NVM function\n");
		return -ENODEV;
	} else if (offset + len > slave->sdev->nvm_function->nvm_size) {
		pr_err("read offset + len = %d > NVM size = %d\n",
			offset + len, slave->sdev->nvm_function->nvm_size);
		return -EINVAL;
	}

	return bif_slave_read(slave,
		slave->sdev->nvm_function->nvm_base_address + offset, buf, len);
}
EXPORT_SYMBOL(bif_slave_nvm_raw_read);

/**
 * bif_slave_nvm_raw_write() - write contiguous memory values to a BIF slave's
 *		non-volatile memory (NVM)
 * @slave:	BIF slave handle
 * @offset:	Offset from the beginning of BIF slave NVM to begin writing at
 * @buf:	Buffer containing values to write
 * @len:	Number of byte to write
 *
 * Note that this function does *not* respect the MIPI-BIF object data
 * formatting specification.  It can cause corruption of the object data list
 * stored in NVM if used improperly.
 *
 * Returns 0 for success or errno if an error occurred.
 */
int bif_slave_nvm_raw_write(struct bif_slave *slave, u16 offset, u8 *buf,
				int len)
{
	int rc;

	if (IS_ERR_OR_NULL(slave)) {
		pr_err("Invalid slave pointer=%ld\n", PTR_ERR(slave));
		return -EINVAL;
	} else if (IS_ERR_OR_NULL(buf)) {
		pr_err("Invalid buffer pointer=%ld\n", PTR_ERR(buf));
		return -EINVAL;
	}

	bif_slave_ctrl_lock(slave);
	rc = _bif_slave_nvm_raw_write(slave->sdev, offset, buf, len);
	bif_slave_ctrl_unlock(slave);

	return rc;
}
EXPORT_SYMBOL(bif_slave_nvm_raw_write);

/**
 * bif_slave_is_present() - check if a slave is currently physically present
 *		in the system
 * @slave:	BIF slave handle
 *
 * Returns 1 if the slave is present, 0 if the slave is not present, or errno
 * if an error occurred.
 *
 * This function can be used by BIF consumer drivers to check if their slave
 * handles are still meaningful after battery reinsertion.
 */
int bif_slave_is_present(struct bif_slave *slave)
{
	if (IS_ERR_OR_NULL(slave)) {
		pr_err("Invalid pointer input.\n");
		return -EINVAL;
	}

	return slave->sdev->present;
}
EXPORT_SYMBOL(bif_slave_is_present);

/**
 * bif_slave_is_selected() - check if a slave is currently selected on the BIF
 *		bus
 * @slave:	BIF slave handle
 *
 * Returns 1 if the slave is selected, 0 if the slave is not selected, or errno
 * if an error occurred.
 *
 * This function should not be required under normal circumstances since the
 * bif-core framework ensures that slaves are always selected when needed.
 * It would be most useful when used as a helper in conjunction with
 * bif_ctrl_bus_lock() and the raw transaction functions.
 */
int bif_slave_is_selected(struct bif_slave *slave)
{
	int rc;

	if (IS_ERR_OR_NULL(slave)) {
		pr_err("Invalid pointer input.\n");
		return -EINVAL;
	}

	if (slave->sdev->bdev->selected_sdev != slave->sdev)
		return false;

	bif_slave_ctrl_lock(slave);
	rc = bif_is_slave_selected(slave->sdev->bdev);
	bif_slave_ctrl_unlock(slave);

	return rc;
}
EXPORT_SYMBOL(bif_slave_is_selected);

/**
 * bif_slave_select() - select a slave on the BIF bus
 * @slave:	BIF slave handle
 *
 * Returns 0 on success or errno if an error occurred.
 *
 * This function should not be required under normal circumstances since the
 * bif-core framework ensures that slaves are always selected when needed.
 * It would be most useful when used as a helper in conjunction with
 * bif_ctrl_bus_lock() and the raw transaction functions.
 */
int bif_slave_select(struct bif_slave *slave)
{
	int rc;

	if (IS_ERR_OR_NULL(slave)) {
		pr_err("Invalid pointer input.\n");
		return -EINVAL;
	}

	bif_slave_ctrl_lock(slave);
	slave->sdev->bdev->selected_sdev = NULL;
	rc = bif_select_slave(slave->sdev);
	bif_slave_ctrl_unlock(slave);

	return rc;
}
EXPORT_SYMBOL(bif_slave_select);

/**
 * bif_ctrl_raw_transaction() - perform a raw BIF transaction on the bus which
 *			expects no slave response
 * @ctrl:		BIF controller consumer handle
 * @transaction:	BIF transaction to carry out.  This should be one of the
 *			values in enum bif_transaction.
 * @data:		8-bit data to use in the transaction.  The meaning of
 *			this data depends upon the transaction that is to be
 *			performed.
 *
 * When performing a bus command (BC) transaction, values in enum
 * bif_bus_command may be used for the data parameter.  Additional manufacturer
 * specific values may also be used in a BC transaction.
 *
 * Returns 0 on success or errno if an error occurred.
 *
 * This function should only need to be used when BIF transactions are required
 * that are not handled by the bif-core directly.
 */
int bif_ctrl_raw_transaction(struct bif_ctrl *ctrl, int transaction, u8 data)
{
	int rc;

	if (IS_ERR_OR_NULL(ctrl)) {
		pr_err("Invalid pointer input.\n");
		return -EINVAL;
	}

	bif_ctrl_lock(ctrl);

	rc = ctrl->bdev->desc->ops->bus_transaction(ctrl->bdev, transaction,
							data);
	if (rc)
		pr_err("BIF bus transaction failed, rc=%d\n", rc);

	bif_ctrl_unlock(ctrl);

	return rc;
}
EXPORT_SYMBOL(bif_ctrl_raw_transaction);

/**
 * bif_ctrl_raw_transaction_read() - perform a raw BIF transaction on the bus
 *			which expects an RD or TACK slave response word
 * @ctrl:		BIF controller consumer handle
 * @transaction:	BIF transaction to carry out.  This should be one of the
 *			values in enum bif_transaction.
 * @data:		8-bit data to use in the transaction.  The meaning of
 *			this data depends upon the transaction that is to be
 *			performed.
 * @response:		Pointer to an integer which is filled with the 11-bit
 *			slave response word upon success.  The 11-bit format is
 *			(MSB to LSB) BCF, ACK, EOT, D7-D0.
 *
 * When performing a bus command (BC) transaction, values in enum
 * bif_bus_command may be used for the data parameter.  Additional manufacturer
 * specific values may also be used in a BC transaction.
 *
 * Returns 0 on success or errno if an error occurred.
 *
 * This function should only need to be used when BIF transactions are required
 * that are not handled by the bif-core directly.
 */
int bif_ctrl_raw_transaction_read(struct bif_ctrl *ctrl, int transaction,
					u8 data, int *response)
{
	int rc;

	if (IS_ERR_OR_NULL(ctrl) || IS_ERR_OR_NULL(response)) {
		pr_err("Invalid pointer input.\n");
		return -EINVAL;
	}

	bif_ctrl_lock(ctrl);

	rc = ctrl->bdev->desc->ops->bus_transaction_read(ctrl->bdev,
					transaction, data, response);
	if (rc)
		pr_err("BIF bus transaction failed, rc=%d\n", rc);

	bif_ctrl_unlock(ctrl);

	return rc;
}
EXPORT_SYMBOL(bif_ctrl_raw_transaction_read);

/**
 * bif_ctrl_raw_transaction_query() - perform a raw BIF transaction on the bus
 *			which expects a BQ slave response
 * @ctrl:		BIF controller consumer handle
 * @transaction:	BIF transaction to carry out.  This should be one of the
 *			values in enum bif_transaction.
 * @data:		8-bit data to use in the transaction.  The meaning of
 *			this data depends upon the transaction that is to be
 *			performed.
 * @query_response:	Pointer to boolean which is set to true if a BQ pulse
 *			is receieved, or false if no BQ pulse is received before
 *			timing out.
 *
 * When performing a bus command (BC) transaction, values in enum
 * bif_bus_command may be used for the data parameter.  Additional manufacturer
 * specific values may also be used in a BC transaction.
 *
 * Returns 0 on success or errno if an error occurred.
 *
 * This function should only need to be used when BIF transactions are required
 * that are not handled by the bif-core directly.
 */
int bif_ctrl_raw_transaction_query(struct bif_ctrl *ctrl, int transaction,
		u8 data, bool *query_response)
{
	int rc;

	if (IS_ERR_OR_NULL(ctrl) || IS_ERR_OR_NULL(query_response)) {
		pr_err("Invalid pointer input.\n");
		return -EINVAL;
	}

	bif_ctrl_lock(ctrl);

	rc = ctrl->bdev->desc->ops->bus_transaction_query(ctrl->bdev,
					transaction, data, query_response);
	if (rc)
		pr_err("BIF bus transaction failed, rc=%d\n", rc);

	bif_ctrl_unlock(ctrl);

	return rc;
}
EXPORT_SYMBOL(bif_ctrl_raw_transaction_query);

/**
 * bif_ctrl_bus_lock() - lock the BIF bus of a controller for exclusive access
 * @ctrl:	BIF controller consumer handle
 *
 * This function should only need to be called in circumstances where a BIF
 * consumer is issuing special BIF bus commands that have strict ordering
 * requirements.
 */
void bif_ctrl_bus_lock(struct bif_ctrl *ctrl)
{
	if (IS_ERR_OR_NULL(ctrl)) {
		pr_err("Invalid controller handle.\n");
		return;
	}

	if (ctrl->exclusive_lock) {
		pr_err("BIF bus exclusive lock already held\n");
		return;
	}

	mutex_lock(&ctrl->bdev->mutex);
	ctrl->exclusive_lock = true;
	bif_cancel_irq_mode_work(ctrl->bdev);
}
EXPORT_SYMBOL(bif_ctrl_bus_lock);

/**
 * bif_ctrl_bus_unlock() - lock the BIF bus of a controller that was previously
 *		locked for exclusive access
 * @ctrl:	BIF controller consumer handle
 *
 * This function must only be called after first calling bif_ctrl_bus_lock().
 */
void bif_ctrl_bus_unlock(struct bif_ctrl *ctrl)
{
	if (IS_ERR_OR_NULL(ctrl)) {
		pr_err("Invalid controller handle.\n");
		return;
	}

	if (!ctrl->exclusive_lock) {
		pr_err("BIF bus exclusive lock not already held\n");
		return;
	}

	ctrl->exclusive_lock = false;
	bif_schedule_irq_mode_work(ctrl->bdev);
	mutex_unlock(&ctrl->bdev->mutex);
}
EXPORT_SYMBOL(bif_ctrl_bus_unlock);

/**
 * bif_ctrl_measure_rid() - measure the battery pack Rid pull-down resistance
 *		in ohms
 * @ctrl:	BIF controller consumer handle
 *
 * Returns the resistance of the Rid resistor in ohms if successful or errno
 * if an error occurred.
 */
int bif_ctrl_measure_rid(struct bif_ctrl *ctrl)
{
	int rc;

	if (IS_ERR_OR_NULL(ctrl)) {
		pr_err("Invalid controller handle.\n");
		return -ENODEV;
	}

	if (!ctrl->bdev->desc->ops->get_battery_rid) {
		pr_err("Cannot measure Rid.\n");
		return -ENXIO;
	}

	bif_ctrl_lock(ctrl);

	rc = ctrl->bdev->desc->ops->get_battery_rid(ctrl->bdev);
	if (rc < 0)
		pr_err("Error during Rid measurement, rc=%d\n", rc);

	bif_ctrl_unlock(ctrl);

	return rc;
}
EXPORT_SYMBOL(bif_ctrl_measure_rid);

/**
 * bif_ctrl_get_bus_period() - get the BIF bus period (tau_bif) in nanoseconds
 * @ctrl:	BIF controller consumer handle
 *
 * Returns the currently configured bus period in nanoseconds if successful or
 * errno if an error occurred.
 */
int bif_ctrl_get_bus_period(struct bif_ctrl *ctrl)
{
	int rc;

	if (IS_ERR_OR_NULL(ctrl)) {
		pr_err("Invalid controller handle.\n");
		return -ENODEV;
	}

	if (!ctrl->bdev->desc->ops->get_bus_period) {
		pr_err("Cannot get the BIF bus period.\n");
		return -ENXIO;
	}

	rc = ctrl->bdev->desc->ops->get_bus_period(ctrl->bdev);
	if (rc < 0)
		pr_err("Error during bus period retrieval, rc=%d\n", rc);

	return rc;
}
EXPORT_SYMBOL(bif_ctrl_get_bus_period);

/**
 * bif_ctrl_set_bus_period() - set the BIF bus period (tau_bif) in nanoseconds
 * @ctrl:	BIF controller consumer handle
 * @period_ns:	BIF bus period in nanoseconds to use
 *
 * If the exact period is not supported by the BIF controller hardware, then the
 * next larger supported period will be used.
 *
 * Returns 0 on success or errno if an error occurred.
 */
int bif_ctrl_set_bus_period(struct bif_ctrl *ctrl, int period_ns)
{
	int rc;

	if (IS_ERR_OR_NULL(ctrl)) {
		pr_err("Invalid controller handle.\n");
		return -ENODEV;
	}

	if (!ctrl->bdev->desc->ops->set_bus_period) {
		pr_err("Cannot set the BIF bus period.\n");
		return -ENXIO;
	}

	bif_ctrl_lock(ctrl);
	rc = ctrl->bdev->desc->ops->set_bus_period(ctrl->bdev, period_ns);
	if (rc)
		pr_err("Error during bus period configuration, rc=%d\n", rc);
	bif_ctrl_unlock(ctrl);

	return rc;
}
EXPORT_SYMBOL(bif_ctrl_set_bus_period);

/**
 * bif_ctrl_get_bus_state() - get the current state of the BIF bus
 * @ctrl:	BIF controller consumer handle
 *
 * Returns a bus state from enum bif_bus_state if successful or errno if an
 * error occurred.
 */
int bif_ctrl_get_bus_state(struct bif_ctrl *ctrl)
{
	int rc;

	if (IS_ERR_OR_NULL(ctrl)) {
		pr_err("Invalid controller handle.\n");
		return -ENODEV;
	}

	rc = ctrl->bdev->desc->ops->get_bus_state(ctrl->bdev);
	if (rc < 0)
		pr_err("Error during bus state retrieval, rc=%d\n", rc);

	return rc;
}
EXPORT_SYMBOL(bif_ctrl_get_bus_state);

/**
 * bif_ctrl_set_bus_state() - set the state of the BIF bus
 * @ctrl:	BIF controller consumer handle
 * @state:	State for the BIF bus to enter
 *
 * Returns 0 on success or errno if an error occurred.
 */
int bif_ctrl_set_bus_state(struct bif_ctrl *ctrl, enum bif_bus_state state)
{
	int rc;

	if (IS_ERR_OR_NULL(ctrl)) {
		pr_err("Invalid controller handle.\n");
		return -ENODEV;
	}

	bif_ctrl_lock(ctrl);

	rc = ctrl->bdev->desc->ops->set_bus_state(ctrl->bdev, state);
	if (rc < 0)
		pr_err("Error during bus state configuration, rc=%d\n", rc);

	/*
	 * Uncache the selected slave if the new bus state results in the slave
	 * becoming unselected.
	 */
	if (state == BIF_BUS_STATE_MASTER_DISABLED
	    || state == BIF_BUS_STATE_POWER_DOWN
	    || state == BIF_BUS_STATE_STANDBY)
		ctrl->bdev->selected_sdev = NULL;

	bif_ctrl_unlock(ctrl);

	return rc;
}
EXPORT_SYMBOL(bif_ctrl_set_bus_state);

/*
 * Check if the specified function is a protocol function and if it is, then
 * instantiate protocol function data for the slave.
 */
static int bif_initialize_protocol_function(struct bif_slave_dev *sdev,
		struct bif_ddb_l2_data *func)
{
	int rc = 0;
	u8 buf[4];

	/* Ensure that this is a protocol function. */
	if (func->function_type != BIF_FUNC_PROTOCOL)
		return 0;

	if (sdev->protocol_function) {
		pr_err("Duplicate protocol function found for BIF slave; DEV_ADR=0x%02X\n",
			sdev->slave_addr);
		return -EPERM;
	}

	sdev->protocol_function = kzalloc(sizeof(struct bif_protocol_function),
						GFP_KERNEL);
	if (!sdev->protocol_function) {
		pr_err("out of memory\n");
		return -ENOMEM;
	}

	rc = _bif_slave_read(sdev, func->function_pointer, buf, 4);
	if (rc) {
		pr_err("Protocol function data read failed, rc=%d\n", rc);
		return rc;
	}

	sdev->protocol_function->protocol_pointer  = buf[0] << 8 | buf[1];
	sdev->protocol_function->device_id_pointer = buf[2] << 8 | buf[3];
	sdev->protocol_function->l2_entry = func;

	rc = _bif_slave_read(sdev, sdev->protocol_function->device_id_pointer,
		sdev->protocol_function->device_id, BIF_DEVICE_ID_BYTE_LENGTH);
	if (rc) {
		pr_err("Device ID read failed, rc=%d\n", rc);
		return rc;
	}

	/* Check if this slave does not have a UID value stored. */
	if (sdev->unique_id_bits_known == 0) {
		sdev->unique_id_bits_known = BIF_UNIQUE_ID_BIT_LENGTH;
		/* Fill in UID using manufacturer ID and device ID. */
		sdev->unique_id[0] = sdev->l1_data.manufacturer_id >> 8;
		sdev->unique_id[1] = sdev->l1_data.manufacturer_id;
		memcpy(&sdev->unique_id[2],
			sdev->protocol_function->device_id,
			BIF_DEVICE_ID_BYTE_LENGTH);
	}

	return rc;
}

/*
 * Check if the specified function is a slave control function and if it is,
 * then instantiate slave control function data for the slave.
 */
static int bif_initialize_slave_control_function(struct bif_slave_dev *sdev,
		struct bif_ddb_l2_data *func)
{
	int rc = 0;
	int i;
	u8 buf[3];

	/* Ensure that this is a slave control function. */
	if (func->function_type != BIF_FUNC_SLAVE_CONTROL)
		return 0;

	if (sdev->slave_ctrl_function) {
		pr_err("Duplicate slave control function found for BIF slave; DEV_ADR=0x%02X\n",
			sdev->slave_addr);
		return -EPERM;
	}

	sdev->slave_ctrl_function
		= kzalloc(sizeof(struct bif_protocol_function), GFP_KERNEL);
	if (!sdev->slave_ctrl_function) {
		pr_err("out of memory\n");
		return -ENOMEM;
	}

	rc = _bif_slave_read(sdev, func->function_pointer, buf, 3);
	if (rc) {
		pr_err("Slave control function data read failed, rc=%d\n", rc);
		return rc;
	}

	sdev->slave_ctrl_function->slave_ctrl_pointer = buf[0] << 8 | buf[1];
	sdev->slave_ctrl_function->task_count
		= buf[2] * SLAVE_CTRL_TASKS_PER_SET;
	sdev->slave_ctrl_function->l2_entry = func;

	if (sdev->slave_ctrl_function->task_count > 0) {
		sdev->slave_ctrl_function->irq_notifier_list =
			kzalloc(sizeof(struct blocking_notifier_head)
			    * sdev->slave_ctrl_function->task_count,
			    GFP_KERNEL);
		if (!sdev->slave_ctrl_function->irq_notifier_list) {
			pr_err("out of memory\n");
			kfree(sdev->slave_ctrl_function);
			return -ENOMEM;
		}

		for (i = 0; i < sdev->slave_ctrl_function->task_count; i++) {
			BLOCKING_INIT_NOTIFIER_HEAD(
			    &sdev->slave_ctrl_function->irq_notifier_list[i]);
		}
	}

	return rc;
}

/*
 * Check if the specified function is an NVM function and if it is, then
 * instantiate NVM function data for the slave and read all objects.
 */
static int bif_initialize_nvm_function(struct bif_slave_dev *sdev,
		struct bif_ddb_l2_data *func)
{
	int rc = 0;
	int data_len, read_size;
	u8 buf[8], object_type;
	struct bif_object *object;
	struct bif_object *temp;
	u16 addr;
	u16 crc;

	/* Ensure that this is an NVM function. */
	if (func->function_type != BIF_FUNC_NVM)
		return 0;

	if (sdev->nvm_function) {
		pr_err("Duplicate NVM function found for BIF slave; DEV_ADR=0x%02X\n",
			sdev->slave_addr);
		return -EPERM;
	}

	sdev->nvm_function
		= kzalloc(sizeof(*sdev->nvm_function), GFP_KERNEL);
	if (!sdev->nvm_function) {
		pr_err("out of memory\n");
		return -ENOMEM;
	}

	rc = _bif_slave_read(sdev, func->function_pointer, buf, 8);
	if (rc) {
		pr_err("NVM function data read failed, rc=%d\n", rc);
		return rc;
	}

	sdev->nvm_function->nvm_pointer			= buf[0] << 8 | buf[1];
	sdev->nvm_function->slave_control_channel	= buf[2];
	sdev->nvm_function->write_buffer_size		= buf[3];
	sdev->nvm_function->nvm_base_address		= buf[4] << 8 | buf[5];
	sdev->nvm_function->nvm_size			= buf[6] << 8 | buf[7];

	/* Read NVM lock offset */
	rc = _bif_slave_read(sdev, sdev->nvm_function->nvm_pointer, buf, 2);
	if (rc) {
		pr_err("Slave memory read failed, rc=%d\n", rc);
		return rc;
	}

	sdev->nvm_function->nvm_lock_offset		= buf[0] << 8 | buf[1];

	INIT_LIST_HEAD(&sdev->nvm_function->object_list);

	/* Read object list */
	addr = sdev->nvm_function->nvm_base_address;
	rc = _bif_slave_read(sdev, addr, &object_type, 1);
	if (rc) {
		pr_err("Slave memory read failed, rc=%d\n", rc);
		return rc;
	}

	while (object_type != BIF_OBJ_END_OF_LIST) {
		object = kzalloc(sizeof(*object), GFP_KERNEL);
		if (!object) {
			pr_err("out of memory\n");
			rc = -ENOMEM;
			goto free_data;
		}
		list_add_tail(&object->list, &sdev->nvm_function->object_list);

		rc = _bif_slave_read(sdev, addr + 1, buf + 1, 5);
		if (rc) {
			pr_err("Slave memory read of object header failed; addr=0x%04X, len=%d, rc=%d\n",
				addr + 1, 5, rc);
			goto free_data;
		}

		object->addr		= addr;
		object->type		= object_type;
		object->version		= buf[1];
		object->manufacturer_id	= buf[2] << 8 | buf[3];
		object->length		= buf[4] << 8 | buf[5];

		if ((object->addr + object->length)
		    > (sdev->nvm_function->nvm_base_address
				+ sdev->nvm_function->nvm_size)) {
			pr_warn("warning: BIF slave object is not formatted correctly; NVM base=0x%04X, NVM len=%d, object addr=0x%04X, object len=%d\n",
				sdev->nvm_function->nvm_base_address,
				sdev->nvm_function->nvm_size,
				object->addr,
				object->length);
			/* Limit object size to remaining NVM size. */
			object->length = sdev->nvm_function->nvm_size
				+ sdev->nvm_function->nvm_base_address
				- object->addr;
		}

		/* Object header + CRC takes up 8 bytes. */
		data_len = object->length - 8;
		object->data = kmalloc(data_len, GFP_KERNEL);
		if (!object->data) {
			pr_err("out of memory\n");
			rc = -ENOMEM;
			goto free_data;
		}

		rc = _bif_slave_read(sdev, addr + 6, object->data, data_len);
		if (rc) {
			pr_err("Slave memory read of object data failed; addr=0x%04X, len=%d, rc=%d\n",
				addr + 6, data_len, rc);
			goto free_data;
		}

		if ((object->length + addr) >= (sdev->nvm_function->nvm_size
				+ sdev->nvm_function->nvm_base_address))
			read_size = 2;
		else
			read_size = 3;
		rc = _bif_slave_read(sdev, addr + 6 + data_len, buf, read_size);
		if (rc) {
			pr_err("Slave memory read of object CRC failed; addr=0x%04X, len=%d, rc=%d\n",
				addr + 6 + data_len, read_size, rc);
			goto free_data;
		}

		object->crc = buf[0] << 8 | buf[1];
		object_type = (read_size == 3) ? buf[2] : BIF_OBJ_END_OF_LIST;
		sdev->nvm_function->object_count++;

		crc = bif_object_crc_ccitt(object);
		if (crc != object->crc)
			pr_info("BIF object at addr=0x%04X has invalid CRC; crc calc=0x%04X, crc exp=0x%04X\n",
				object->addr, crc, object->crc);

		addr += object->length;
	}

	return rc;

free_data:
	list_for_each_entry_safe(object, temp,
				&sdev->nvm_function->object_list, list) {
		list_del(&object->list);
		kfree(object->data);
		kfree(object);
	}
	kfree(sdev->nvm_function);
	sdev->nvm_function = NULL;
	return rc;
}

static int bif_parse_slave_data(struct bif_slave_dev *sdev)
{
	int rc = 0;
	u8 buf[10];
	u8 *func_buf;
	struct bif_ddb_l2_data *func;
	int function_count, i;

	rc = _bif_slave_read(sdev, BIF_DDB_L1_BASE_ADDR, buf, 10);
	if (rc) {
		pr_err("DDB L1 data read failed, rc=%d\n", rc);
		return rc;
	}

	sdev->l1_data.revision		= buf[0];
	sdev->l1_data.level		= buf[1];
	sdev->l1_data.device_class	= buf[2] << 8 | buf[3];
	sdev->l1_data.manufacturer_id	= buf[4] << 8 | buf[5];
	sdev->l1_data.product_id	= buf[6] << 8 | buf[7];
	sdev->l1_data.length		= buf[8] << 8 | buf[9];

	function_count = sdev->l1_data.length / 4;
	if (sdev->l1_data.length % 4) {
		pr_err("Function directory length=%d is invalid\n",
				sdev->l1_data.length);
		return -EPROTO;
	}

	/* No DDB L2 function directory */
	if (function_count == 0)
		return 0;

	func_buf = kmalloc(sdev->l1_data.length, GFP_KERNEL);
	if (!func_buf) {
		pr_err("out of memory\n");
		return -ENOMEM;
	}

	sdev->function_directory = kzalloc(
		function_count * sizeof(struct bif_ddb_l2_data), GFP_KERNEL);
	if (!sdev->function_directory) {
		pr_err("out of memory\n");
		return -ENOMEM;
	}

	rc = _bif_slave_read(sdev, BIF_DDB_L2_BASE_ADDR, func_buf,
				sdev->l1_data.length);
	if (rc) {
		pr_err("DDB L2 data read failed, rc=%d\n", rc);
		return rc;
	}

	for (i = 0; i < function_count; i++) {
		func = &sdev->function_directory[i];
		func->function_type	= func_buf[i * 4];
		func->function_version	= func_buf[i * 4 + 1];
		func->function_pointer	= func_buf[i * 4 + 2] << 8
					  | func_buf[i * 4 + 3];
		rc = bif_initialize_protocol_function(sdev, func);
		if (rc)
			goto done;
		rc = bif_initialize_slave_control_function(sdev, func);
		if (rc)
			goto done;
		rc = bif_initialize_nvm_function(sdev, func);
		if (rc)
			goto done;
	}
done:
	kfree(func_buf);
	return rc;
}

static int bif_add_secondary_slaves(struct bif_slave_dev *primary_slave)
{
	int rc = 0;
	int data_len, i;
	u16 crc;
	struct bif_slave_dev *sdev;
	struct bif_object *object;

	list_for_each_entry(object, &primary_slave->nvm_function->object_list,
				list) {
		if (object->type != BIF_OBJ_SEC_SLAVE)
			continue;

		data_len = object->length - 8;
		if (data_len % BIF_UNIQUE_ID_BYTE_LENGTH) {
			pr_info("Invalid secondary slave object found, addr=0x%04X, data len=%d\n",
				object->addr, data_len);
			continue;
		}

		crc = bif_object_crc_ccitt(object);
		if (crc != object->crc) {
			pr_info("BIF object at addr=0x%04X has invalid CRC; crc calc=0x%04X, crc exp=0x%04X\n",
				object->addr, crc, object->crc);
			continue;
		}

		for (i = 0; i < data_len / BIF_UNIQUE_ID_BYTE_LENGTH; i++) {
			sdev = bif_add_slave(primary_slave->bdev);
			if (IS_ERR(sdev)) {
				rc = PTR_ERR(sdev);
				pr_err("bif_add_slave failed, rc=%d\n", rc);
				return rc;
			}
			memcpy(sdev->unique_id,
				&object->data[i * BIF_UNIQUE_ID_BYTE_LENGTH],
				BIF_UNIQUE_ID_BYTE_LENGTH);
			sdev->unique_id_bits_known = BIF_UNIQUE_ID_BIT_LENGTH;

			rc = bif_select_slave(sdev);
			if (rc) {
				pr_err("Could not select slave, rc=%d\n", rc);
				goto free_slave;
			}

			rc = bif_is_slave_selected(sdev->bdev);
			if (rc < 0) {
				pr_err("Transaction failed, rc=%d\n", rc);
				goto free_slave;
			} else if (rc == 1) {
				sdev->present = true;
				sdev->bdev->selected_sdev = sdev;
				rc = bif_parse_slave_data(sdev);
				if (rc) {
					pr_err("Failed to parse secondary slave data, rc=%d\n",
						rc);
					goto free_slave;
				}
			} else {
				sdev->present = false;
				sdev->bdev->selected_sdev = NULL;
			}
		}
	}

	return rc;

free_slave:
	bif_remove_slave(sdev);
	return rc;
}

/*
 * Performs UID search to identify all slaves attached to the bus. Assumes that
 * all necessary locks are held.
 */
static int bif_perform_uid_search(struct bif_ctrl_dev *bdev)
{
	struct bif_slave_dev *sdev;
	struct bif_slave_dev *new_slave;
	bool resp[2], resp_dilc;
	int i;
	int rc = 0;
	u8 cmd_probe[2] = {BIF_CMD_DIP0, BIF_CMD_DIP1};
	u8 cmd_enter[2] = {BIF_CMD_DIE0, BIF_CMD_DIE1};

	/*
	 * Iterate over all partially known UIDs adding new ones as they are
	 * found.
	 */
	list_for_each_entry(sdev, &bif_sdev_list, list) {
		/* Skip slaves with fully known UIDs. */
		if (sdev->unique_id_bits_known == BIF_UNIQUE_ID_BIT_LENGTH
		    || sdev->bdev != bdev)
			continue;

		/* Begin a new UID search. */
		rc = bdev->desc->ops->bus_transaction(bdev, BIF_TRANS_BC,
							BIF_CMD_DISS);
		if (rc) {
			pr_err("bus_transaction failed, rc=%d\n", rc);
			return rc;
		}

		/* Step through all known UID bits (MSB to LSB). */
		for (i = 0; i < sdev->unique_id_bits_known; i++) {
			rc = bdev->desc->ops->bus_transaction(bdev,
				BIF_TRANS_BC,
				cmd_enter[get_uid_bit(sdev->unique_id, i)]);
			if (rc) {
				pr_err("bus_transaction failed, rc=%d\n", rc);
				return rc;
			}
		}

		/* Step through unknown UID bits. */
		for (i = sdev->unique_id_bits_known;
				i < BIF_UNIQUE_ID_BIT_LENGTH; i++) {
			rc = bdev->desc->ops->bus_transaction_query(bdev,
				BIF_TRANS_BC, cmd_probe[0], &resp[0]);
			if (rc) {
				pr_err("bus_transaction failed, rc=%d\n", rc);
				return rc;
			}

			rc = bdev->desc->ops->bus_transaction_query(bdev,
				BIF_TRANS_BC, cmd_probe[1], &resp[1]);
			if (rc) {
				pr_err("bus_transaction failed, rc=%d\n", rc);
				return rc;
			}

			if (resp[0] && resp[1]) {
				/* Create an entry for the new UID branch. */
				new_slave = bif_add_slave(bdev);
				if (IS_ERR(new_slave)) {
					rc = PTR_ERR(sdev);
					pr_err("bif_add_slave failed, rc=%d\n",
						rc);
					return rc;
				}
				memcpy(new_slave->unique_id, sdev->unique_id,
					BIF_UNIQUE_ID_BYTE_LENGTH);
				new_slave->bdev = sdev->bdev;

				set_uid_bit(sdev->unique_id, i, 0);
				sdev->unique_id_bits_known = i + 1;

				set_uid_bit(new_slave->unique_id, i, 1);
				new_slave->unique_id_bits_known = i + 1;
			} else if (resp[0]) {
				set_uid_bit(sdev->unique_id, i, 0);
				sdev->unique_id_bits_known = i + 1;
			} else if (resp[1]) {
				set_uid_bit(sdev->unique_id, i, 1);
				sdev->unique_id_bits_known = i + 1;
			} else {
				pr_debug("no bus query response received\n");
				rc = -ENXIO;
				return rc;
			}

			rc = bdev->desc->ops->bus_transaction(bdev,
				BIF_TRANS_BC, cmd_enter[resp[0] ? 0 : 1]);
			if (rc) {
				pr_err("bus_transaction failed, rc=%d\n", rc);
				return rc;
			}
		}

		rc = bdev->desc->ops->bus_transaction_query(bdev,
			BIF_TRANS_BC, BIF_CMD_DILC, &resp_dilc);
		if (rc) {
			pr_err("bus_transaction failed, rc=%d\n", rc);
			return rc;
		}

		if (resp_dilc) {
			sdev->present = true;
			sdev->bdev->selected_sdev = sdev;
			rc = bif_parse_slave_data(sdev);
			if (rc) {
				pr_err("Failed to parse secondary slave data, rc=%d\n",
					rc);
				return rc;
			}
		} else {
			pr_err("Slave failed to respond to DILC bus command; its UID is thus unverified.\n");
			sdev->unique_id_bits_known = 0;
			rc = -ENXIO;
			return rc;
		}
	}

	return rc;
}

/*
 * Removes slaves from the bif_sdev_list which have the same UID as previous
 * slaves in the list.
 */
static int bif_remove_duplicate_slaves(struct bif_ctrl_dev *bdev)
{
	struct bif_slave_dev *sdev;
	struct bif_slave_dev *last_slave;
	struct bif_slave_dev *temp;

	list_for_each_entry_safe(last_slave, temp, &bif_sdev_list, list) {
		list_for_each_entry(sdev, &bif_sdev_list, list) {
			if (last_slave == sdev) {
				break;
			} else if (memcmp(last_slave->unique_id,
					sdev->unique_id,
					BIF_UNIQUE_ID_BYTE_LENGTH) == 0) {
				bif_remove_slave(last_slave);
				break;
			}
		}
	}

	return 0;
}

static int bif_add_all_slaves(struct bif_ctrl_dev *bdev)
{
	struct bif_slave_dev *sdev;
	int rc = 0;
	int i;
	bool has_slave = false, is_primary_slave = false;

	mutex_lock(&bif_sdev_list_mutex);
	mutex_lock(&bdev->mutex);

	list_for_each_entry(sdev, &bif_sdev_list, list) {
		if (sdev->bdev == bdev) {
			has_slave = true;
			break;
		}
	}

	if (!has_slave) {
		/* Create a single empty slave to start the search algorithm. */
		sdev = bif_add_slave(bdev);
		if (IS_ERR(sdev)) {
			rc = PTR_ERR(sdev);
			pr_err("bif_add_slave failed, rc=%d\n", rc);
			goto out;
		}

		for (i = 0; i < BIF_TRANSACTION_RETRY_COUNT; i++) {
			/* Attempt to select primary slave in battery pack. */
			rc = bdev->desc->ops->bus_transaction(bdev,
				BIF_TRANS_SDA, BIF_PRIMARY_SLAVE_DEV_ADR);
			if (rc == 0)
				break;
		}
		if (rc) {
			pr_err("BIF bus_transaction failed, rc=%d\n", rc);
			goto out;
		}

		/* Check if a slave is selected. */
		rc = bif_is_slave_selected(bdev);
		if (rc < 0) {
			pr_err("BIF bus_transaction failed, rc=%d\n", rc);
			goto out;
		} else {
			is_primary_slave = rc;
		}
	}

	if (is_primary_slave) {
		pr_debug("Using primary slave at DEV_ADR==0x%02X\n",
			BIF_PRIMARY_SLAVE_DEV_ADR);
		sdev->bdev->selected_sdev = sdev;
		sdev->present = true;
		sdev->slave_addr = BIF_PRIMARY_SLAVE_DEV_ADR;
		rc = bif_parse_slave_data(sdev);
		if (rc) {
			pr_err("Failed to parse primary slave data, rc=%d\n",
				rc);
			goto out;
		}
		rc = bif_add_secondary_slaves(sdev);
		if (rc) {
			pr_err("Failed to add secondary slaves, rc=%d\n", rc);
			goto out;
		}
	} else {
		pr_debug("Falling back on full UID search.\n");
		for (i = 0; i < BIF_TRANSACTION_RETRY_COUNT; i++) {
			rc = bif_perform_uid_search(bdev);
			if (rc == 0)
				break;
		}
		if (rc) {
			pr_debug("BIF UID search failed, rc=%d\n", rc);
			goto out;
		}
	}

	bif_remove_duplicate_slaves(bdev);

	mutex_unlock(&bdev->mutex);
	mutex_unlock(&bif_sdev_list_mutex);

	return rc;

out:
	mutex_unlock(&bdev->mutex);
	mutex_unlock(&bif_sdev_list_mutex);
	pr_debug("BIF slave search failed, rc=%d\n", rc);
	return rc;
}

static int bif_add_known_slave(struct bif_ctrl_dev *bdev, u8 slave_addr)
{
	struct bif_slave_dev *sdev;
	int rc = 0;
	int i;

	for (i = 0; i < BIF_TRANSACTION_RETRY_COUNT; i++) {
		/* Attempt to select the slave. */
		rc = bdev->desc->ops->bus_transaction(bdev, BIF_TRANS_SDA,
							slave_addr);
		if (rc == 0)
			break;
	}
	if (rc) {
		pr_err("BIF bus_transaction failed, rc=%d\n", rc);
		return rc;
	}

	/* Check if a slave is selected. */
	rc = bif_is_slave_selected(bdev);
	if (rc < 0) {
		pr_err("BIF bus_transaction failed, rc=%d\n", rc);
		return rc;
	}

	sdev = bif_add_slave(bdev);
	if (IS_ERR(sdev)) {
		rc = PTR_ERR(sdev);
		pr_err("bif_add_slave failed, rc=%d\n", rc);
		return rc;
	}

	sdev->bdev->selected_sdev = sdev;
	sdev->present = true;
	sdev->slave_addr = slave_addr;
	rc = bif_parse_slave_data(sdev);
	if (rc) {
		pr_err("Failed to parse slave data, addr=0x%02X, rc=%d\n",
			slave_addr, rc);
		return rc;
	}

	return rc;
}

static int bif_add_known_slaves_from_dt(struct bif_ctrl_dev *bdev,
					struct device_node *of_node)
{
	int len = 0;
	int rc, i;
	u32 addr;
	const __be32 *val;

	mutex_lock(&bif_sdev_list_mutex);
	mutex_lock(&bdev->mutex);

	val = of_get_property(of_node, "qcom,known-device-addresses", &len);
	len /= sizeof(u32);
	if (val && len == 0) {
		pr_err("qcom,known-device-addresses property is invalid\n");
		rc = -EINVAL;
		goto out;
	}

	for (i = 0; i < len; i++) {
		addr = be32_to_cpup(val++);
		if (addr == 0x00 || addr > 0xFF) {
			rc = -EINVAL;
			pr_err("qcom,known-device-addresses property contains invalid address=0x%X\n",
				addr);
			goto out;
		}
		rc = bif_add_known_slave(bdev, addr);
		if (rc) {
			pr_err("bif_add_known_slave() failed, rc=%d\n", rc);
			goto out;
		}
	}

out:
	if (len > 0)
		bif_remove_duplicate_slaves(bdev);

	mutex_unlock(&bdev->mutex);
	mutex_unlock(&bif_sdev_list_mutex);

	return rc;
}

/*
 * Programs a device address for the specified slave in order to simplify
 * slave selection in the future.
 */
static int bif_assign_slave_dev_addr(struct bif_slave_dev *sdev, u8 dev_addr)
{
	int rc;
	u16 addr;

	if (!sdev->protocol_function) {
		pr_err("Protocol function not present; cannot set device address.\n");
		return -ENODEV;
	}

	addr = PROTOCOL_FUNC_DEV_ADR_ADDR(
			sdev->protocol_function->protocol_pointer);

	rc = _bif_slave_write(sdev, addr, &dev_addr, 1);
	if (rc)
		pr_err("Failed to set slave device address.\n");
	else
		sdev->slave_addr = dev_addr;

	return rc;
}

/* Assigns a unique device address to all slaves which do not have one. */
static int bif_assign_all_slaves_dev_addr(struct bif_ctrl_dev *bdev)
{
	struct bif_slave_dev *sdev;
	struct bif_slave_dev *sibling;
	bool duplicate;
	int rc = 0;
	u8 dev_addr, first_dev_addr;

	mutex_lock(&bif_sdev_list_mutex);
	mutex_lock(&bdev->mutex);

	first_dev_addr = next_dev_addr;
	/*
	 * Iterate over all partially known UIDs adding new ones as they are
	 * found.
	 */
	list_for_each_entry(sdev, &bif_sdev_list, list) {
		/*
		 * Skip slaves without known UIDs, which already have a device
		 * address or which aren't present.
		 */
		if (sdev->unique_id_bits_known != BIF_UNIQUE_ID_BIT_LENGTH
		    || sdev->slave_addr != 0x00 || !sdev->present)
			continue;

		do {
			dev_addr = next_dev_addr;
			duplicate = false;
			list_for_each_entry(sibling, &bif_sdev_list, list) {
				if (sibling->slave_addr == dev_addr) {
					duplicate = true;
					break;
				}
			}

			next_dev_addr = dev_addr + 1;
		} while (duplicate && (next_dev_addr != first_dev_addr));

		if (next_dev_addr == first_dev_addr) {
			pr_err("No more BIF slave device addresses available.\n");
			rc = -ENODEV;
			goto out;
		}

		rc =  bif_assign_slave_dev_addr(sdev, dev_addr);
		if (rc) {
			pr_err("Failed to set slave address.\n");
			goto out;
		}
	}

	mutex_unlock(&bdev->mutex);
	mutex_unlock(&bif_sdev_list_mutex);

	return rc;

out:
	mutex_unlock(&bdev->mutex);
	mutex_unlock(&bif_sdev_list_mutex);
	pr_err("BIF slave device address setting failed, rc=%d\n", rc);
	return rc;
}

/**
 * bdev_get_drvdata() - get the private BIF controller driver data
 * @bdev:	BIF controller device pointer
 */
void *bdev_get_drvdata(struct bif_ctrl_dev *bdev)
{
	return bdev->driver_data;
}
EXPORT_SYMBOL(bdev_get_drvdata);

static const char * const battery_label[] = {
	"unknown",
	"none",
	"special 1",
	"special 2",
	"special 3",
	"low cost",
	"smart",
};

static const char *bif_get_battery_pack_type(int rid_ohm)
{
	const char *label = battery_label[0];

	if (rid_ohm > BIF_BATT_RID_SMART_MAX)
		label = battery_label[1];
	else if (rid_ohm >= BIF_BATT_RID_SMART_MIN)
		label = battery_label[6];
	else if (rid_ohm >= BIF_BATT_RID_LOW_COST_MIN
			&& rid_ohm <= BIF_BATT_RID_LOW_COST_MAX)
		label = battery_label[5];
	else if (rid_ohm >= BIF_BATT_RID_SPECIAL3_MIN
			&& rid_ohm <= BIF_BATT_RID_SPECIAL3_MAX)
		label = battery_label[4];
	else if (rid_ohm >= BIF_BATT_RID_SPECIAL2_MIN
			&& rid_ohm <= BIF_BATT_RID_SPECIAL2_MAX)
		label = battery_label[3];
	else if (rid_ohm >= BIF_BATT_RID_SPECIAL1_MIN
			&& rid_ohm <= BIF_BATT_RID_SPECIAL1_MAX)
		label = battery_label[2];

	return label;
}

/**
 * bif_ctrl_register() - register a BIF controller with the BIF framework
 * @bif_desc:		Pointer to BIF controller descriptor
 * @dev:		Device pointer of the BIF controller
 * @driver_data:	Private driver data to associate with the BIF controller
 * @of_node		Pointer to the device tree node of the BIF controller
 *
 * Returns a BIF controller device pointer for the controller if registration
 * is successful or an ERR_PTR if an error occurred.
 */
struct bif_ctrl_dev *bif_ctrl_register(struct bif_ctrl_desc *bif_desc,
	struct device *dev, void *driver_data, struct device_node *of_node)
{
	struct bif_ctrl_dev *bdev = ERR_PTR(-EINVAL);
	struct bif_slave_dev *sdev;
	bool battery_present = false;
	bool slaves_present = false;
	int rc, rid_ohm;

	if (!bif_desc) {
		pr_err("Invalid bif_desc specified\n");
		return bdev;
	} else if (!bif_desc->name) {
		pr_err("BIF name missing\n");
		return bdev;
	} else if (!bif_desc->ops) {
		pr_err("BIF operations missing\n");
		return bdev;
	} else if (!bif_desc->ops->bus_transaction
			|| !bif_desc->ops->bus_transaction_query
			|| !bif_desc->ops->bus_transaction_read
			|| !bif_desc->ops->get_bus_state
			|| !bif_desc->ops->set_bus_state) {
		pr_err("BIF operation callback function(s) missing\n");
		return bdev;
	}

	bdev = kzalloc(sizeof(struct bif_ctrl_dev), GFP_KERNEL);
	if (bdev == NULL) {
		pr_err("Memory allocation failed for bif_ctrl_dev\n");
		return ERR_PTR(-ENOMEM);
	}

	mutex_init(&bdev->mutex);
	INIT_LIST_HEAD(&bdev->list);
	INIT_DELAYED_WORK(&bdev->enter_irq_mode_work, bif_enter_irq_mode_work);
	bdev->desc			= bif_desc;
	bdev->ctrl_dev			= dev;
	bdev->driver_data		= driver_data;
	bdev->irq_mode_delay_jiffies	= 2;

	mutex_lock(&bif_ctrl_list_mutex);
	list_add_tail(&bdev->list, &bif_ctrl_list);
	mutex_unlock(&bif_ctrl_list_mutex);

	rc = bif_add_all_slaves(bdev);
	if (rc)
		pr_debug("Search for all slaves failed, rc=%d\n", rc);
	rc = bif_add_known_slaves_from_dt(bdev, of_node);
	if (rc)
		pr_err("Adding slaves based on device tree addressed failed, rc=%d.\n",
			rc);
	rc = bif_assign_all_slaves_dev_addr(bdev);
	if (rc)
		pr_err("Failed to set slave device address, rc=%d\n", rc);

	bif_print_slaves();

	if (bdev->desc->ops->get_battery_presence) {
		rc = bdev->desc->ops->get_battery_presence(bdev);
		if (rc < 0) {
			pr_err("Could not determine battery presence, rc=%d\n",
				rc);
		} else {
			battery_present = rc;
			pr_info("Battery pack present = %c\n", rc ? 'Y' : 'N');
		}
	}

	if (bdev->desc->ops->get_battery_rid) {
		rid_ohm = bdev->desc->ops->get_battery_rid(bdev);
		if (rid_ohm >= 0)
			pr_info("Battery pack type = %s (Rid=%d ohm)\n",
				bif_get_battery_pack_type(rid_ohm), rid_ohm);
		else
			pr_err("Could not read Rid, rc=%d\n", rid_ohm);
	}

	list_for_each_entry(sdev, &bif_sdev_list, list) {
		if (sdev->present) {
			battery_present = true;
			slaves_present = true;
			break;
		}
	}

	BLOCKING_INIT_NOTIFIER_HEAD(&bdev->bus_change_notifier);

	/* Disable the BIF bus master if no slaves are found. */
	if (!slaves_present) {
		rc = bdev->desc->ops->set_bus_state(bdev,
			BIF_BUS_STATE_MASTER_DISABLED);
		if (rc < 0)
			pr_err("Could not disble BIF master, rc=%d\n", rc);
	}

	if (battery_present) {
		bdev->battery_present = true;
		rc = blocking_notifier_call_chain(&bdev->bus_change_notifier,
			BIF_BUS_EVENT_BATTERY_INSERTED, bdev);
		if (rc)
			pr_err("Call chain noification failed, rc=%d\n", rc);
	}

	return bdev;
}
EXPORT_SYMBOL(bif_ctrl_register);

/**
 * bif_ctrl_unregister() - unregisters a BIF controller
 * @bdev:	BIF controller device pointer
 */
void bif_ctrl_unregister(struct bif_ctrl_dev *bdev)
{
	if (bdev) {
		mutex_lock(&bif_ctrl_list_mutex);
		list_del(&bdev->list);
		mutex_unlock(&bif_ctrl_list_mutex);
	}
}
EXPORT_SYMBOL(bif_ctrl_unregister);
