/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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


#define pr_fmt(fmt) "%s:%d: " fmt, __func__, __LINE__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/debugfs.h>
#include <linux/spmi.h>
#include <linux/ctype.h>
#include "spmi-dbgfs.h"
#ifdef CONFIG_HTC_POWER_DEBUG
#include <linux/of_device.h>
#include <linux/debugfs.h>
#endif
#include <mach/devices_cmdline.h>


#define ADDR_LEN	 6	
#define CHARS_PER_ITEM   3	
#define ITEMS_PER_LINE	16	
#define MAX_LINE_LENGTH  (ADDR_LEN + (ITEMS_PER_LINE * CHARS_PER_ITEM) + 1)
#define MAX_REG_PER_TRANSACTION	(8)

static const char *DFS_ROOT_NAME	= "spmi";
static const mode_t DFS_MODE = S_IRUSR | S_IWUSR;

struct spmi_log_buffer {
	size_t rpos;	
	size_t wpos;	
	size_t len;	
	char data[0];	
};

struct spmi_ctrl_data {
	u32 cnt;
	u32 addr;
	struct dentry *dir;
	struct list_head node;
	struct spmi_controller *ctrl;
};

struct spmi_trans {
	u32 cnt;	
	u32 addr;	
	u32 offset;	
	bool raw_data;	
	struct spmi_controller *ctrl;
	struct spmi_log_buffer *log; 
};

struct spmi_dbgfs {
	struct dentry *root;
	struct mutex  lock;
	struct list_head ctrl; 
	struct debugfs_blob_wrapper help_msg;
};

static struct spmi_dbgfs dbgfs_data = {
	.lock = __MUTEX_INITIALIZER(dbgfs_data.lock),
	.ctrl = LIST_HEAD_INIT(dbgfs_data.ctrl),
	.help_msg = {
	.data =
"SPMI Debug-FS support\n"
"\n"
"Hierarchy schema:\n"
"/sys/kernel/debug/spmi\n"
"       /help            -- Static help text\n"
"       /spmi-0          -- Directory for SPMI bus 0\n"
"       /spmi-0/address  -- Starting register address for reads or writes\n"
"       /spmi-0/count    -- Number of registers to read (only used for reads)\n"
"       /spmi-0/data     -- Initiates the SPMI read (formatted output)\n"
"       /spmi-0/data_raw -- Initiates the SPMI raw read or write\n"
"       /spmi-n          -- Directory for SPMI bus n\n"
"\n"
"To perform SPMI read or write transactions, you need to first write the\n"
"address of the slave device register to the 'address' file.  For read\n"
"transactions, the number of bytes to be read needs to be written to the\n"
"'count' file.\n"
"\n"
"The 'address' file specifies the 20-bit address of a slave device register.\n"
"The upper 4 bits 'address[19..16]' specify the slave identifier (SID) for\n"
"the slave device.  The lower 16 bits specify the slave register address.\n"
"\n"
"Reading from the 'data' file will initiate a SPMI read transaction starting\n"
"from slave register 'address' for 'count' number of bytes.\n"
"\n"
"Writing to the 'data' file will initiate a SPMI write transaction starting\n"
"from slave register 'address'.  The number of registers written to will\n"
"match the number of bytes written to the 'data' file.\n"
"\n"
"Example: Read 4 bytes starting at register address 0x1234 for SID 2\n"
"\n"
"echo 0x21234 > address\n"
"echo 4 > count\n"
"cat data\n"
"\n"
"Example: Write 3 bytes starting at register address 0x1008 for SID 1\n"
"\n"
"echo 0x11008 > address\n"
"echo 0x01 0x02 0x03 > data\n"
"\n"
"Note that the count file is not used for writes.  Since 3 bytes are\n"
"written to the 'data' file, then 3 bytes will be written across the\n"
"SPMI bus.\n\n",
	},
};

static int spmi_dfs_open(struct spmi_ctrl_data *ctrl_data, struct file *file)
{
	struct spmi_log_buffer *log;
	struct spmi_trans *trans;

	size_t logbufsize = SZ_4K;

	if (!ctrl_data) {
		pr_err("No SPMI controller data\n");
		return -EINVAL;
	}

	
	trans = kzalloc(sizeof(*trans), GFP_KERNEL);

	if (!trans) {
		pr_err("Unable to allocate memory for transaction data\n");
		return -ENOMEM;
	}

	
	log = kzalloc(logbufsize, GFP_KERNEL);

	if (!log) {
		kfree(trans);
		pr_err("Unable to allocate memory for log buffer\n");
		return -ENOMEM;
	}

	log->rpos = 0;
	log->wpos = 0;
	log->len = logbufsize - sizeof(*log);

	trans->log = log;
	trans->cnt = ctrl_data->cnt;
	trans->addr = ctrl_data->addr;
	trans->ctrl = ctrl_data->ctrl;
	trans->offset = trans->addr;

	file->private_data = trans;
	return 0;
}

static int spmi_dfs_data_open(struct inode *inode, struct file *file)
{
	struct spmi_ctrl_data *ctrl_data = inode->i_private;
	return spmi_dfs_open(ctrl_data, file);
}

static int spmi_dfs_raw_data_open(struct inode *inode, struct file *file)
{
	int rc;
	struct spmi_trans *trans;
	struct spmi_ctrl_data *ctrl_data = inode->i_private;

	rc = spmi_dfs_open(ctrl_data, file);
	trans = file->private_data;
	trans->raw_data = true;
	return rc;
}

static int spmi_dfs_close(struct inode *inode, struct file *file)
{
	struct spmi_trans *trans = file->private_data;

	if (trans && trans->log) {
		file->private_data = NULL;
		kfree(trans->log);
		kfree(trans);
	}

	return 0;
}

static int
spmi_read_data(struct spmi_controller *ctrl, uint8_t *buf, int offset, int cnt)
{
	int ret = 0;
	int len;
	uint8_t sid;
	uint16_t addr;

	while (cnt > 0) {
		sid = (offset >> 16) & 0xF;
		addr = offset & 0xFFFF;
		len = min(cnt, MAX_REG_PER_TRANSACTION);

		ret = spmi_ext_register_readl(ctrl, sid, addr, buf, len);
		if (ret < 0) {
			pr_err("SPMI read failed, err = %d\n", ret);
			goto done;
		}

		cnt -= len;
		buf += len;
		offset += len;
	}

done:
	return ret;
}

/**
 * spmi_write_data: writes data across the SPMI bus
 * @ctrl: The SPMI controller
 * @buf: data to be written.
 * @offset: SPMI address offset to start writing to.
 * @cnt: The number of bytes to write.
 *
 * Returns 0 on success, otherwise returns error code from SPMI driver.
 */
static int
spmi_write_data(struct spmi_controller *ctrl, uint8_t *buf, int offset, int cnt)
{
	int ret = 0;
	int len;
	uint8_t sid;
	uint16_t addr;

	while (cnt > 0) {
		sid = (offset >> 16) & 0xF;
		addr = offset & 0xFFFF;
		len = min(cnt, MAX_REG_PER_TRANSACTION);

		ret = spmi_ext_register_writel(ctrl, sid, addr, buf, len);
		if (ret < 0) {
			pr_err("SPMI write failed, err = %d\n", ret);
			goto done;
		}

		cnt -= len;
		buf += len;
		offset += len;
	}

done:
	return ret;
}

/**
 * print_to_log: format a string and place into the log buffer
 * @log: The log buffer to place the result into.
 * @fmt: The format string to use.
 * @...: The arguments for the format string.
 *
 * The return value is the number of characters written to @log buffer
 * not including the trailing '\0'.
 */
static int print_to_log(struct spmi_log_buffer *log, const char *fmt, ...)
{
	va_list args;
	int cnt;
	char *buf = &log->data[log->wpos];
	size_t size = log->len - log->wpos;

	va_start(args, fmt);
	cnt = vscnprintf(buf, size, fmt, args);
	va_end(args);

	log->wpos += cnt;
	return cnt;
}

static int
write_next_line_to_log(struct spmi_trans *trans, int offset, size_t *pcnt)
{
	int i, j;
	u8  data[ITEMS_PER_LINE];
	struct spmi_log_buffer *log = trans->log;

	int cnt = 0;
	int padding = offset % ITEMS_PER_LINE;
	int items_to_read = min(ARRAY_SIZE(data) - padding, *pcnt);
	int items_to_log = min(ITEMS_PER_LINE, padding + items_to_read);

	
	if ((log->len - log->wpos) < MAX_LINE_LENGTH)
		goto done;

	
	if (spmi_read_data(trans->ctrl, data, offset, items_to_read))
		goto done;

	*pcnt -= items_to_read;

	
	cnt = print_to_log(log, "%5.5X ", offset & 0xffff0);
	if (cnt == 0)
		goto done;

	
	for (i = 0; i < padding; ++i) {
		cnt = print_to_log(log, "-- ");
		if (cnt == 0)
			goto done;
	}

	
	for (j = 0; i < items_to_log; ++i, ++j) {
		cnt = print_to_log(log, "%2.2X ", data[j]);
		if (cnt == 0)
			goto done;
	}

	
	if (log->wpos > 0 && log->data[log->wpos - 1] == ' ')
		log->data[log->wpos - 1] = '\n';

done:
	return cnt;
}

static int
write_raw_data_to_log(struct spmi_trans *trans, int offset, size_t *pcnt)
{
	u8  data[16];
	struct spmi_log_buffer *log = trans->log;

	int i;
	int cnt = 0;
	int items_to_read = min(ARRAY_SIZE(data), *pcnt);

	
	if ((log->len - log->wpos) < 80)
		goto done;

	
	if (spmi_read_data(trans->ctrl, data, offset, items_to_read))
		goto done;

	*pcnt -= items_to_read;

	
	for (i = 0; i < items_to_read; ++i) {
		cnt = print_to_log(log, "0x%2.2X ", data[i]);
		if (cnt == 0)
			goto done;
	}

	
	if (log->wpos > 0 && log->data[log->wpos - 1] == ' ')
		log->data[log->wpos - 1] = '\n';

done:
	return cnt;
}

static int get_log_data(struct spmi_trans *trans)
{
	int cnt;
	int last_cnt;
	int items_read;
	int total_items_read = 0;
	u32 offset = trans->offset;
	size_t item_cnt = trans->cnt;
	struct spmi_log_buffer *log = trans->log;
	int (*write_to_log)(struct spmi_trans *, int, size_t *);

	if (item_cnt == 0)
		return 0;

	if (trans->raw_data)
		write_to_log = write_raw_data_to_log;
	else
		write_to_log = write_next_line_to_log;

	
	log->wpos = log->rpos = 0;

	
	do {
		last_cnt = item_cnt;
		cnt = write_to_log(trans, offset, &item_cnt);
		items_read = last_cnt - item_cnt;
		offset += items_read;
		total_items_read += items_read;
	} while (cnt && item_cnt > 0);

	
	trans->cnt = item_cnt;
	trans->offset += total_items_read;

	return total_items_read;
}

/**
 * spmi_dfs_reg_write: write user's byte array (coded as string) over SPMI.
 * @file: file pointer
 * @buf: user data to be written.
 * @count: maximum space available in @buf
 * @ppos: starting position
 * @return number of user byte written, or negative error value
 */
static ssize_t spmi_dfs_reg_write(struct file *file, const char __user *buf,
			size_t count, loff_t *ppos)
{
	int bytes_read;
	int data;
	int pos = 0;
	int cnt = 0;
	u8  *values;
	size_t ret = 0;

	struct spmi_trans *trans = file->private_data;
	u32 offset = trans->offset;

	
	char *kbuf = kmalloc(count + 1, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	ret = copy_from_user(kbuf, buf, count);
	if (ret == count) {
		pr_err("failed to copy data from user\n");
		ret = -EFAULT;
		goto free_buf;
	}

	count -= ret;
	*ppos += count;
	kbuf[count] = '\0';

	
	values = kbuf;

	
	while (sscanf(kbuf + pos, "%i%n", &data, &bytes_read) == 1) {
		pos += bytes_read;
		values[cnt++] = data & 0xff;
	}

	if (!cnt)
		goto free_buf;

	
	ret = spmi_write_data(trans->ctrl, values, offset, cnt);

	if (ret) {
		pr_err("SPMI write failed, err = %zu\n", ret);
	} else {
		ret = count;
		trans->offset += cnt;
	}

free_buf:
	kfree(kbuf);
	return ret;
}

static ssize_t spmi_dfs_reg_read(struct file *file, char __user *buf,
	size_t count, loff_t *ppos)
{
	struct spmi_trans *trans = file->private_data;
	struct spmi_log_buffer *log = trans->log;
	size_t ret;
	size_t len;

	
	if (log->rpos >= log->wpos) {
		if (get_log_data(trans) <= 0)
			return 0;
	}

	len = min(count, log->wpos - log->rpos);

	ret = copy_to_user(buf, &log->data[log->rpos], len);
	if (ret == len) {
		pr_err("error copy SPMI register values to user\n");
		return -EFAULT;
	}

	
	len -= ret;

	*ppos += len;
	log->rpos += len;
	return len;
}

static const struct file_operations spmi_dfs_reg_fops = {
	.open		= spmi_dfs_data_open,
	.release	= spmi_dfs_close,
	.read		= spmi_dfs_reg_read,
	.write		= spmi_dfs_reg_write,
};

static const struct file_operations spmi_dfs_raw_data_fops = {
	.open		= spmi_dfs_raw_data_open,
	.release	= spmi_dfs_close,
	.read		= spmi_dfs_reg_read,
	.write		= spmi_dfs_reg_write,
};

static struct dentry *spmi_dfs_create_fs(void)
{
	struct dentry *root, *file;

	pr_debug("Creating SPMI debugfs file-system\n");
	root = debugfs_create_dir(DFS_ROOT_NAME, NULL);
	if (IS_ERR_OR_NULL(root)) {
		pr_err("Error creating top level directory err:%ld",
			(long)root);
		if (PTR_ERR(root) == -ENODEV)
			pr_err("debugfs is not enabled in the kernel");
		return NULL;
	}

	dbgfs_data.help_msg.size = strlen(dbgfs_data.help_msg.data);

	file = debugfs_create_blob("help", S_IRUGO, root, &dbgfs_data.help_msg);
	if (!file) {
		pr_err("error creating help entry\n");
		goto err_remove_fs;
	}
	return root;

err_remove_fs:
	debugfs_remove_recursive(root);
	return NULL;
}

struct dentry *spmi_dfs_get_root(void)
{
	if (dbgfs_data.root)
		return dbgfs_data.root;

	if (mutex_lock_interruptible(&dbgfs_data.lock) < 0)
		return NULL;
	
	if (!dbgfs_data.root) { 
		dbgfs_data.root = spmi_dfs_create_fs();
	}
	mutex_unlock(&dbgfs_data.lock);
	return dbgfs_data.root;
}

#ifdef CONFIG_HTC_POWER_DEBUG
#define VREG_DUMP_DRIVER_NAME	"htc,vreg-dump"
#define VREG_NAME_VOL_LEN       32
#define VREG_EN_PD_MODE_LEN     8
#define VREG_DUMP_LEN           128

struct _vreg {
	int id;
	int type;
	const char *name;
	u32 base_addr;
};

struct _qpnp_vregs {
	struct _vreg *vregs;
	struct spmi_controller *ctrl;
	u32 en_ctl_offset;
	u32 pd_ctl_offset;
	u32 mode_ctl_offset;
	u32 range_ctl_offset;
	u32 step_ctl_offset;
	u32 en_bit;
	u32 pd_bit;
	int total_vregs;
};

enum {
        VREG_TYPE_NLDO,
        VREG_TYPE_PLDO,
        VREG_TYPE_HF_SMPS,
        VREG_TYPE_FT_SMPS,
        VREG_TYPE_BOOST_SMPS,
        VREG_TYPE_LVS,
};

struct qpnp_voltage_range {
	int             min_uV;
	int             max_uV;
	int             step_uV;
	int             set_point_min_uV;
	unsigned        n_voltages;
	u8              range_sel;
};

#define VREG_IS_LDO(type)       (type == VREG_TYPE_NLDO || type == VREG_TYPE_PLDO)
#define VREG_IS_SMPS(type)      (type == VREG_TYPE_HF_SMPS || type == VREG_TYPE_FT_SMPS || type == VREG_TYPE_BOOST_SMPS)

#define VOLTAGE_RANGE(_range_sel, _min_uV, _set_point_min_uV, _max_uV, \
			_step_uV) \
	{ \
		.min_uV			= _min_uV, \
		.set_point_min_uV	= _set_point_min_uV, \
		.max_uV			= _max_uV, \
		.step_uV		= _step_uV, \
		.range_sel		= _range_sel, \
	}

#define SET_POINTS(_ranges) \
{ \
	.range	= _ranges, \
	.count	= ARRAY_SIZE(_ranges), \
};

static struct qpnp_voltage_range pldo_ranges[] = {
	VOLTAGE_RANGE(2,  750000,  750000, 1537500, 12500),
	VOLTAGE_RANGE(3, 1500000, 1550000, 3075000, 25000),
	VOLTAGE_RANGE(4, 1750000, 3100000, 4900000, 50000),
};

static struct qpnp_voltage_range nldo1_ranges[] = {
	VOLTAGE_RANGE(2,  750000,  750000, 1537500, 12500),
};


static struct qpnp_voltage_range nldo3_ranges[] = {
	VOLTAGE_RANGE(0,  375000,  375000, 1537500, 12500),
};

static struct qpnp_voltage_range smps_ranges[] = {
	VOLTAGE_RANGE(0,  375000,  375000, 1562500, 12500),
	VOLTAGE_RANGE(1, 1550000, 1575000, 3125000, 25000),
};

static struct qpnp_voltage_range ftsmps_ranges[] = {
	VOLTAGE_RANGE(0,       0,  350000, 1275000,  5000),
	VOLTAGE_RANGE(1,       0, 1280000, 2040000, 10000),
};

static struct qpnp_voltage_range boost_ranges[] = {
	VOLTAGE_RANGE(0, 4000000, 4000000, 5550000, 50000),
};

struct _qpnp_vregs qpnp_vregs;
#endif

int spmi_dfs_add_controller(struct spmi_controller *ctrl)
{
	struct dentry *dir;
	struct dentry *root;
	struct dentry *file;
	struct spmi_ctrl_data *ctrl_data;

	pr_debug("Adding controller %s\n", ctrl->dev.kobj.name);
	root = spmi_dfs_get_root();
	if (!root)
		return -ENOENT;

	
	ctrl_data = kzalloc(sizeof(*ctrl_data), GFP_KERNEL);
	if (!ctrl_data)
		return -ENOMEM;

	dir = debugfs_create_dir(ctrl->dev.kobj.name, root);
	if (!dir) {
		pr_err("Error creating entry for spmi controller %s\n",
						ctrl->dev.kobj.name);
		goto err_create_dir_failed;
	}

	ctrl_data->cnt  = 1;
	ctrl_data->dir  = dir;
	ctrl_data->ctrl = ctrl;

	file = debugfs_create_u32("count", DFS_MODE, dir, &ctrl_data->cnt);
	if (!file) {
		pr_err("error creating 'count' entry\n");
		goto err_remove_fs;
	}

	file = debugfs_create_x32("address", DFS_MODE, dir, &ctrl_data->addr);
	if (!file) {
		pr_err("error creating 'address' entry\n");
		goto err_remove_fs;
	}

	file = debugfs_create_file("data", DFS_MODE, dir, ctrl_data,
							&spmi_dfs_reg_fops);
	if (!file) {
		pr_err("error creating 'data' entry\n");
		goto err_remove_fs;
	}

	file = debugfs_create_file("data_raw", DFS_MODE, dir, ctrl_data,
						&spmi_dfs_raw_data_fops);
	if (!file) {
		pr_err("error creating 'data' entry\n");
		goto err_remove_fs;
	}

	list_add(&ctrl_data->node, &dbgfs_data.ctrl);

#ifdef CONFIG_HTC_POWER_DEBUG
	qpnp_vregs.ctrl = ctrl;
#endif
	return 0;

err_remove_fs:
	debugfs_remove_recursive(dir);
err_create_dir_failed:
	kfree(ctrl_data);
	return -ENOMEM;
}

int spmi_dfs_del_controller(struct spmi_controller *ctrl)
{
	int rc;
	struct list_head *pos, *tmp;
	struct spmi_ctrl_data *ctrl_data;

	pr_debug("Deleting controller %s\n", ctrl->dev.kobj.name);

	rc = mutex_lock_interruptible(&dbgfs_data.lock);
	if (rc)
		return rc;

	list_for_each_safe(pos, tmp, &dbgfs_data.ctrl) {
		ctrl_data = list_entry(pos, struct spmi_ctrl_data, node);

		if (ctrl_data->ctrl == ctrl) {
			debugfs_remove_recursive(ctrl_data->dir);
			list_del(pos);
			kfree(ctrl_data);
			rc = 0;
			goto done;
		}
	}
	rc = -EINVAL;
	pr_debug("Unknown controller %s\n", ctrl->dev.kobj.name);

done:
	mutex_unlock(&dbgfs_data.lock);
	return rc;
}

struct dentry *spmi_dfs_create_file(struct spmi_controller *ctrl,
					const char *name, void *data,
					const struct file_operations *fops)
{
	struct spmi_ctrl_data *ctrl_data;

	list_for_each_entry(ctrl_data, &dbgfs_data.ctrl, node) {
		if (ctrl_data->ctrl == ctrl)
			return debugfs_create_file(name,
					DFS_MODE, ctrl_data->dir, data, fops);
	}

	return NULL;
}

static void __exit spmi_dfs_delete_all_ctrl(struct list_head *head)
{
	struct list_head *pos, *tmp;

	list_for_each_safe(pos, tmp, head) {
		struct spmi_ctrl_data *ctrl_data;

		ctrl_data = list_entry(pos, struct spmi_ctrl_data, node);
		list_del(pos);
		kfree(ctrl_data);
	}
}

static void __exit spmi_dfs_destroy(void)
{
	pr_debug("de-initializing spmi debugfs ...");
	if (mutex_lock_interruptible(&dbgfs_data.lock) < 0)
		return;
	if (dbgfs_data.root) {
		debugfs_remove_recursive(dbgfs_data.root);
		dbgfs_data.root = NULL;
		spmi_dfs_delete_all_ctrl(&dbgfs_data.ctrl);
	}
	mutex_unlock(&dbgfs_data.lock);
}

#ifdef CONFIG_HTC_POWER_DEBUG
static int htc_vreg_is_enabled(struct _qpnp_vregs *qpnp_vregs, struct _vreg *vreg)
{
        u8 val;
        int ret = 0, rc = 0;

        ret = spmi_read_data(qpnp_vregs->ctrl, &val, vreg->base_addr + qpnp_vregs->en_ctl_offset, 1);
        if (ret < 0) {
                pr_err("%s: SPMI read failed, err = %d\n", __func__, ret);
                return ret;
        }

        if (val & (1 << qpnp_vregs->en_bit))
                rc = 1; 
        else
                rc = 0; 

        return rc;
}

static int htc_vreg_is_pulldown(struct _qpnp_vregs *qpnp_vregs, struct _vreg *vreg)
{
        u8 val;
        int ret = 0, rc = 0;

        ret = spmi_read_data(qpnp_vregs->ctrl, &val, vreg->base_addr + qpnp_vregs->pd_ctl_offset, 1);
        if (ret < 0) {
                pr_err("%s: SPMI read failed, err = %d\n", __func__, ret);
                return ret;
        }

        if (val & (1 << qpnp_vregs->pd_bit))
                rc = 1; 
        else
                rc = 0; 

        return rc;
}

static int htc_vreg_get_mode(struct _qpnp_vregs *qpnp_vregs, struct _vreg *vreg)
{
        u8 val;
        int ret = 0;

        ret = spmi_read_data(qpnp_vregs->ctrl, &val, vreg->base_addr + qpnp_vregs->mode_ctl_offset, 1);
        if (ret < 0) {
                pr_err("%s: SPMI read failed, err = %d\n", __func__, ret);
                return ret;
        }
        return val;
}

static int htc_vreg_ldo_get_voltage(struct _qpnp_vregs *qpnp_vregs, struct _vreg *vreg)
{
        int i = 0;
        u8 range = 0;
        u8 voltage = 0;
        u32 step = 0;
        u32 vmin = 0;
        int ret = 0;

        ret= spmi_read_data(qpnp_vregs->ctrl, &range, vreg->base_addr + qpnp_vregs->range_ctl_offset, 1);
        if (ret < 0) {
                pr_err("SPMI read failed, err = %d\n", ret);
                return -1;
        }

        ret= spmi_read_data(qpnp_vregs->ctrl, &voltage, vreg->base_addr + qpnp_vregs->step_ctl_offset, 1);
        if (ret < 0) {
                pr_err("SPMI read failed, err = %d\n", ret);
                return -1;
        }

        if (vreg->type == VREG_TYPE_PLDO) {
		for (i = 0; i < ARRAY_SIZE(pldo_ranges); i++) {
			if (pldo_ranges[i].range_sel == range) {
				step = pldo_ranges[i].step_uV;
				vmin = pldo_ranges[i].min_uV;
				break;
			}
		}
        } else if (vreg->type == VREG_TYPE_NLDO) {
                if (range == 2) {
                        step = nldo1_ranges[0].step_uV;
                        vmin = nldo1_ranges[0].min_uV;
                } else {
                        step = nldo3_ranges[0].step_uV;
                        vmin = nldo3_ranges[0].min_uV;
                }
        } else {
                pr_err("%s: vreg type = %d, range = %d, not support\n", __func__, range, vreg->type);
                
                if (range == 0) {
                        step = 120000;
                        vmin = 1380000;
                } else if (range == 1) {
                        step = 60000;
                        vmin = 690000;
                } else {
                        return 0;
                }
        }

        return (step * voltage + vmin);
}

static int htc_vreg_smps_get_voltage(struct _qpnp_vregs *qpnp_vregs, struct _vreg *vreg)
{
        int i = 0;
        u8 range = 0;
        u8 voltage = 0;
        u32 step = 0;
        u32 vmin = 0;
        int ret = 0;

        ret = spmi_read_data(qpnp_vregs->ctrl, &range, vreg->base_addr + qpnp_vregs->range_ctl_offset, 1);
        if (ret < 0) {
                pr_err("SPMI read failed, err = %d\n", ret);
                return -1;
        }

        ret = spmi_read_data(qpnp_vregs->ctrl, &voltage, vreg->base_addr + qpnp_vregs->step_ctl_offset, 1);
        if (ret < 0) {
                pr_err("SPMI read failed, err = %d\n", ret);
                return -1;
        }

        if (vreg->type == VREG_TYPE_HF_SMPS) {
		for (i = 0; i < ARRAY_SIZE(smps_ranges); i++) {
			if (smps_ranges[i].range_sel == range) {
                                step = smps_ranges[i].step_uV;
                                vmin = smps_ranges[i].min_uV;
			        break;
		        }
		}
        } else if (vreg->type == VREG_TYPE_FT_SMPS) {
		for (i = 0; i < ARRAY_SIZE(ftsmps_ranges); i++) {
			if (ftsmps_ranges[i].range_sel == range) {
                                step = ftsmps_ranges[i].step_uV;
                                vmin = ftsmps_ranges[i].min_uV;
			        break;
		        }
		}
        } else {
                step = boost_ranges[0].step_uV;
                vmin = boost_ranges[0].min_uV;
        }

        return (step * voltage + vmin);
}

static int htc_vreg_get_voltage(struct _qpnp_vregs *qpnp_vregs, struct _vreg *vreg)
{
        int ret = 0;

        if (VREG_IS_LDO(vreg->type))
                ret = htc_vreg_ldo_get_voltage(qpnp_vregs, vreg);
        else if (VREG_IS_SMPS(vreg->type))
                ret = htc_vreg_smps_get_voltage(qpnp_vregs, vreg);

        return ret;
}

static int htc_vreg_ldo_set_voltage(struct _qpnp_vregs *qpnp_vregs, struct _vreg *vreg, int val)
{
	int ret, i;
	uint8_t range_sel, voltage_sel;
	int range_sel_flag, range_id = 0;

	
	range_sel_flag = -1;

	
	if (vreg->type == VREG_TYPE_PLDO) {
		for (i = 0; i < ARRAY_SIZE(pldo_ranges); i++) {
			if ((pldo_ranges[i].min_uV < val)
				&& ( val < pldo_ranges[i].max_uV)) {
				range_sel = pldo_ranges[i].range_sel;
				range_id = i;
				range_sel_flag = 1;
				break;
			}
		}
	} else if (vreg->type == VREG_TYPE_NLDO) {
		for (i = 0; i < ARRAY_SIZE(nldo1_ranges); i++) {
			if ((nldo1_ranges[i].min_uV < val)
				&& ( val < nldo1_ranges[i].max_uV)) {
				range_sel = nldo1_ranges[i].range_sel;
				range_id = i;
				range_sel_flag = 1;
				break;
			}
		}
	} else {
		
	}

	
	if (range_sel_flag<0) {
		pr_err("Can't set voltage due to not range can be seleted\n");
		return -1;
	}

	
	if (vreg->type == VREG_TYPE_PLDO) {
		voltage_sel = (val - pldo_ranges[range_id].min_uV)
			/ pldo_ranges[range_id].step_uV;
	} else if (vreg->type == VREG_TYPE_NLDO) {
		voltage_sel = (val - nldo1_ranges[range_id].min_uV)
			/ nldo1_ranges[range_id].step_uV;
	} else {
		
	}

	
	ret = spmi_write_data(qpnp_vregs->ctrl, &range_sel,
		vreg->base_addr + qpnp_vregs->range_ctl_offset, 1);

	if (ret) {
		pr_err("SPMI write failed, err = %zu\n", ret);
		return ret;
	}

	
	ret = spmi_write_data(qpnp_vregs->ctrl, &voltage_sel,
		vreg->base_addr + qpnp_vregs->step_ctl_offset, 1);

	if (ret) {
		pr_err("SPMI write failed, err = %zu\n", ret);
		return ret;
	}

	return 0;
}

static int htc_vreg_smps_set_voltage(struct _qpnp_vregs *qpnp_vregs, struct _vreg *vreg, int val)
{
	int ret, i;
	uint8_t range_sel, voltage_sel;
	int range_sel_flag, range_id = 0;

	
	range_sel_flag = -1;

	if (vreg->type == VREG_TYPE_HF_SMPS) {
		for (i = 0; i < ARRAY_SIZE(smps_ranges); i++) {
			if ((smps_ranges[i].min_uV < val)
				&& ( val < smps_ranges[i].max_uV)) {
				range_sel = smps_ranges[i].range_sel;
				range_id = i;
				range_sel_flag = 1;
				break;
			}
		}
	} else if (vreg->type == VREG_TYPE_FT_SMPS) {
#if 0
		for (i = 0; i < ARRAY_SIZE(ftsmps_ranges); i++) {
			if ((ftsmps_ranges[i].min_uV < val)
				&& ( val < ftsmps_ranges[i].max_uV)) {
				range_sel = ftsmps_ranges[i].range_sel;
				range_id = i;
				break;
			}
		}
#else
		
		ret = spmi_read_data(qpnp_vregs->ctrl, &range_sel, vreg->base_addr + qpnp_vregs->range_ctl_offset, 1);
		if (ret) {
			pr_err("SPMI read failed, err = %zu\n", ret);
			return ret;
		}
		
		for (i = 0; i < ARRAY_SIZE(ftsmps_ranges); i++) {
			if (ftsmps_ranges[i].range_sel == range_sel) {
				range_id = i;
				range_sel_flag = 1;
				break;
			}
		}
#endif
	} else {
		
	}

	
	if (range_sel_flag<0) {
		pr_err("Can't set voltage due to not range can be seleted\n");
		return -1;
	}

	
	if (vreg->type == VREG_TYPE_HF_SMPS) {
		voltage_sel = (val - smps_ranges[range_id].min_uV)
			/ smps_ranges[range_id].step_uV;
	} else if (vreg->type == VREG_TYPE_FT_SMPS) {
		voltage_sel = (val - ftsmps_ranges[range_id].min_uV)
			/ ftsmps_ranges[range_id].step_uV;
	} else {
		
	}

	
	ret = spmi_write_data(qpnp_vregs->ctrl, &range_sel,
		vreg->base_addr + qpnp_vregs->range_ctl_offset, 1);

	if (ret) {
		pr_err("SPMI write failed, err = %zu\n", ret);
		return ret;
	}

	
	ret = spmi_write_data(qpnp_vregs->ctrl, &voltage_sel,
		vreg->base_addr + qpnp_vregs->step_ctl_offset, 1);

	if (ret) {
		pr_err("SPMI write failed, err = %zu\n", ret);
		return ret;
	}

	return 0;
}

static int htc_vreg_set_voltage(struct _qpnp_vregs *qpnp_vregs, struct _vreg *vreg, u64 val)
{
	int ret = 0;

	if (VREG_IS_LDO(vreg->type))
		ret = htc_vreg_ldo_set_voltage(qpnp_vregs, vreg, val);
	else if (VREG_IS_SMPS(vreg->type))
		ret = htc_vreg_smps_set_voltage(qpnp_vregs, vreg, val);

	return ret;
}

static int htc_vreg_is_valid(int vreg_id)
{
        int id_check = 0, dump_check = 1, i = 0;

        for (i = 0; i < qpnp_vregs.total_vregs; i++) {
                if (vreg_id == qpnp_vregs.vregs[i].id) {
                        id_check = 1;
                        break;
                }
        }

        return (id_check && dump_check);
}

int htc_vreg_dump(int vreg_id, struct seq_file *m, char *vreg_buffer, int curr_len)
{
        struct _vreg *vreg = NULL;
        int len = 0;
        int enable = 0;
        int mode = 0;
        int pd = 0;
        int vol = 0;
        char name_buf[VREG_NAME_VOL_LEN];
        char en_buf[VREG_EN_PD_MODE_LEN];
        char pd_buf[VREG_EN_PD_MODE_LEN];
        char mode_buf[VREG_EN_PD_MODE_LEN];
        char vol_buf[VREG_NAME_VOL_LEN];
        char vreg_buf[VREG_DUMP_LEN];

        if (!htc_vreg_is_valid(vreg_id))
                return curr_len;

	vreg = &qpnp_vregs.vregs[vreg_id];

        
        memset(name_buf, 0, VREG_NAME_VOL_LEN);
        len = strlen(vreg->name);
        if (len >= VREG_NAME_VOL_LEN)
                len = VREG_NAME_VOL_LEN - 1;
        memcpy(name_buf, vreg->name, len);
        name_buf[len] = '\0';

        
        memset(en_buf, 0, VREG_EN_PD_MODE_LEN);
        enable = htc_vreg_is_enabled(&qpnp_vregs, vreg);
        if (enable < 0)
                sprintf(en_buf, "NULL");
        else if (enable)
                sprintf(en_buf, "YES ");
        else
                sprintf(en_buf, "NO  ");

        
        memset(mode_buf, 0, VREG_EN_PD_MODE_LEN);
        mode = htc_vreg_get_mode(&qpnp_vregs, vreg);
        if (vreg->type < 2) {
                mode_buf[0] = mode & 0x80 ? 'H' : '_';
                mode_buf[1] = mode & 0x40 ? 'A' : '_';
                mode_buf[2] = mode & 0x20 ? 'B' : '_';
                mode_buf[3] = mode & 0x10 ? 'W' : '_';
        } else if (vreg->type == VREG_TYPE_HF_SMPS) {
                mode_buf[0] = mode & 0x80 ? 'H' : '_';
                mode_buf[1] = mode & 0x40 ? 'A' : '_';
                mode_buf[2] = 'U';
                mode_buf[3] = mode & 0x10 ? 'W' : '_';
        } else if (vreg->type == VREG_TYPE_FT_SMPS) {
                mode_buf[0] = mode & 0x80 ? 'H' : '_';
                mode_buf[1] = mode & 0x40 ? 'A' : '_';
                mode_buf[2] = 'U';
                mode_buf[3] = 'U';
        } else if (vreg->type == VREG_TYPE_LVS) {
                mode_buf[0] = mode & 0x80 ? 'H' : '_';
                mode_buf[1] = mode & 0x40 ? 'A' : '_';
                mode_buf[2] = 'U';
                mode_buf[3] = mode & 0x10 ? 'W' : '_';
        } else {
                mode_buf[0] = 'U';
                mode_buf[1] = 'U';
                mode_buf[2] = 'U';
                mode_buf[3] = 'U';
        }

        
        memset(pd_buf, 0, VREG_EN_PD_MODE_LEN);
        pd = htc_vreg_is_pulldown(&qpnp_vregs, vreg);
        if (pd < 0)
                sprintf(pd_buf, "NULL");
        else if (pd)
                sprintf(pd_buf, "YES ");
        else
                sprintf(pd_buf, "NO  ");

        
        memset(vol_buf, 0, VREG_NAME_VOL_LEN);
        vol = htc_vreg_get_voltage(&qpnp_vregs, vreg);
        if (vol < 0)
                sprintf(vol_buf, "NULL");
        else
                sprintf(vol_buf, "%d uV", vol);

        if (m)
                seq_printf(m, "VREG %s: [Enable]%s, [Mode]%s, [PD]%s, [Vol]%s\n", name_buf, en_buf, mode_buf, pd_buf, vol_buf);
        else
                pr_info("VREG %s: [Enable]%s, [Mode]%s, [PD]%s, [Vol]%s\n", name_buf, en_buf, mode_buf, pd_buf, vol_buf);

        if (vreg_buffer) {
                sprintf(vreg_buf, "VREG %s: [Enable]%s, [Mode]%s, [PD]%s, [Vol]%s\n", name_buf, en_buf, mode_buf, pd_buf, vol_buf);
                vreg_buf[VREG_DUMP_LEN - 1] = '\0';
                curr_len += sprintf(vreg_buffer + curr_len, vreg_buf);
        }

        return curr_len;
}
#if defined(CONFIG_MACH_EYE_UL)
#define PN547_I2C_POWEROFF_SEQUENCE_FOR_EYE
#elif defined(CONFIG_MACH_EYE_WL)
#define PN547_I2C_POWEROFF_SEQUENCE_FOR_EYE
#elif defined(CONFIG_MACH_EYE_WHL)
#define PN547_I2C_POWEROFF_SEQUENCE_FOR_EYE
#elif defined(CONFIG_MACH_MEC_TL)
#define PN547_I2C_POWEROFF_SEQUENCE_FOR_MEC
#elif defined(CONFIG_MACH_MEC_WHL)
#define PN547_I2C_POWEROFF_SEQUENCE_FOR_MEC
#elif defined(CONFIG_MACH_MEC_UL)
#define PN547_I2C_POWEROFF_SEQUENCE_FOR_MEC
#elif defined(CONFIG_MACH_MEC_DUG)
#define PN547_I2C_POWEROFF_SEQUENCE_FOR_MEC
#elif defined(CONFIG_MACH_MEC_DWG)
#define PN547_I2C_POWEROFF_SEQUENCE_FOR_MEC
#elif defined(CONFIG_MACH_DUMMY)
#define PN547_I2C_POWEROFF_SEQUENCE_FOR_B2
#else
#endif
#if defined(PN547_I2C_POWEROFF_SEQUENCE_FOR_EYE)
void force_disable_PM8941_VREG_ID_L22(void)
{
	int ret;
	uint8_t voltage_sel = 0x00;
	
	
	ret = spmi_write_data(qpnp_vregs.ctrl, &voltage_sel, 0x15546, 1);
	if (ret) {
                pr_err("force_disable_PM8941_VREG_ID_L22, SPMI write failed, err = %zu\n", ret);
        }
}
#elif defined(PN547_I2C_POWEROFF_SEQUENCE_FOR_MEC)
void force_disable_PMICGPIO34(void)
{
	int ret;
	uint8_t voltage_sel = 0x10;

	ret = spmi_write_data(qpnp_vregs.ctrl, &voltage_sel, 0xE140, 1);

}
#elif defined(PN547_I2C_POWEROFF_SEQUENCE_FOR_B2)
void force_disable_PMICGPIO34(void)
{
	int ret;
	uint8_t voltage_sel = 0x10;

	ret = spmi_write_data(qpnp_vregs.ctrl, &voltage_sel, 0xE140, 1);

}
void force_disable_PMICLVS1(void)
{
	int ret;
	uint8_t voltage_sel = 0x0;

	ret = spmi_write_data(qpnp_vregs.ctrl, &voltage_sel, 0x18046, 1);

}
#else
#endif

int htc_vregs_dump(char *vreg_buffer, int curr_len)
{
        int i;
        char *title_msg = "------------ PMIC VREG -------------\n";

        if (vreg_buffer)
                curr_len += sprintf(vreg_buffer + curr_len, "%s\n", title_msg);

        pr_info("%s", title_msg);
        for (i = 0; i < qpnp_vregs.total_vregs; i++)
                curr_len = htc_vreg_dump(i, NULL, vreg_buffer, curr_len);

        return curr_len;
}

static int list_vregs_show(struct seq_file *m, void *unused)
{
        int i;
        char *title_msg = "------------ PMIC VREG -------------\n";

        if (m)
                seq_printf(m, title_msg);

        for (i = 0; i < qpnp_vregs.total_vregs; i++)
                htc_vreg_dump(i, m, NULL, 0);

        return 0;
}

static int list_vregs_open(struct inode *inode, struct file *file)
{
        return single_open(file, list_vregs_show, inode->i_private);
}

static const struct file_operations list_vregs_fops = {
        .open = list_vregs_open,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = seq_release,
};

static int voltage_debug_set(void *data, u64 val)
{
	struct _vreg *vreg = data;
	return htc_vreg_set_voltage(&qpnp_vregs, vreg, val);
}

static int voltage_debug_get(void *data, u64 *val)
{
	struct _vreg *vreg = data;
	*val = htc_vreg_get_voltage(&qpnp_vregs, vreg);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(voltage_fops, voltage_debug_get,
			voltage_debug_set, "%lld\n");


static int htc_voltage_debugfs_init(void)
{
		static struct dentry *debugfs_voltages_base;
		struct _vreg *vreg = NULL;
		int i;

		debugfs_voltages_base = debugfs_create_dir("htc_voltage", NULL);

		for (i = 0; i < qpnp_vregs.total_vregs; i++) {
			vreg = &qpnp_vregs.vregs[i];
			if(!debugfs_create_file(vreg->name, S_IRUGO, debugfs_voltages_base,
				vreg, &voltage_fops))
				return -ENOMEM;
		}

        return 0;
}

static int htc_vreg_dump_debugfs_init(void)
{
        static struct dentry *debugfs_vregs_base;

        debugfs_vregs_base = debugfs_create_dir("htc_vreg", NULL);

        if (!debugfs_vregs_base)
                return -ENOMEM;

        if (!debugfs_create_file("list_vregs", S_IRUGO, debugfs_vregs_base,
                                NULL, &list_vregs_fops))
                return -ENOMEM;

        return 0;
}

static int htc_regulator_enable(struct _qpnp_vregs *qpnp_vregs, struct _vreg *vreg)
{
	int ret =0;
	u8 enabled = 0x80;

	ret = spmi_write_data(qpnp_vregs->ctrl, &enabled, vreg->base_addr + qpnp_vregs->en_ctl_offset, 1);
	if (ret < 0) {
		pr_err("%s: SPMI read failed, err = %d\n", __func__, ret);
		return ret;
	}

	return ret;
}

static int htc_regulator_disable(struct _qpnp_vregs *qpnp_vregs, struct _vreg *vreg)
{
	int ret =0;
	u8 disabled = 0x00;

	ret = spmi_write_data(qpnp_vregs->ctrl, &disabled, vreg->base_addr + qpnp_vregs->en_ctl_offset, 1);
	if (ret < 0) {
		pr_err("%s: SPMI read failed, err = %d\n", __func__, ret);
		return ret;
	}

	return ret;
}

static int htc_reg_debug_enable_set(void *data, u64 val)
{
	struct _vreg *vreg = data;
	int ret;

	if (val)
		ret = htc_regulator_enable(&qpnp_vregs, vreg);
	else
		ret = htc_regulator_disable(&qpnp_vregs, vreg);

	return ret;
}

static int htc_reg_debug_enable_get(void *data, u64 *val)
{
	struct _vreg *vreg = data;
	*val = htc_vreg_is_enabled(&qpnp_vregs, vreg);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(switch_fops, htc_reg_debug_enable_get,
			htc_reg_debug_enable_set, "%llu\n");


static int htc_vreg_switch_debugfs_init(void)
{
	static struct dentry *debugfs_switch_base;
	struct _vreg *vreg = NULL;
	int i;

	debugfs_switch_base = debugfs_create_dir("htc_vreg_switchs", NULL);

	for (i = 0; i < qpnp_vregs.total_vregs; i++) {
		vreg = &qpnp_vregs.vregs[i];
		if(!debugfs_create_file(vreg->name, S_IRUGO, debugfs_switch_base,
				vreg, &switch_fops))
		return -ENOMEM;
	}

	return 0;
}

struct vreg_type_lookup_table {
	uint32_t type;
	const char *type_name;
};

static int htc_vreg_get_type(struct device_node *node,
			char *key, uint32_t *val)
{
	int i;
	static struct vreg_type_lookup_table vreg_type_lookup[] = {
		{VREG_TYPE_NLDO, "nldo"},
		{VREG_TYPE_PLDO, "pldo"},
		{VREG_TYPE_HF_SMPS, "hf-smps"},
              {VREG_TYPE_FT_SMPS, "ft-smps"},
              {VREG_TYPE_BOOST_SMPS, "boost-smps"},
              {VREG_TYPE_LVS, "lvs"}
	};
	const char *type_str;
	int ret;

	ret = of_property_read_string(node, key, &type_str);
	if (!ret) {
		ret = -EINVAL;
		for (i = 0; i < ARRAY_SIZE(vreg_type_lookup); i++) {
			if (!strcmp(type_str, vreg_type_lookup[i].type_name)) {
				*val = vreg_type_lookup[i].type;
				ret = 0;
				break;
			}
		}
	}
	return ret;
}

static int __devinit htc_vreg_dump_probe(struct platform_device *pdev)
{
	struct device_node *node = NULL;
        struct _vreg *vreg = NULL;
	char *key = NULL;
        int num_vreg = 0;
	int idx = 0;
        int ret;

	for_each_child_of_node(pdev->dev.of_node, node)
		num_vreg++;
        qpnp_vregs.total_vregs = num_vreg;

	qpnp_vregs.vregs = kzalloc(num_vreg * sizeof(struct _qpnp_vregs), GFP_KERNEL);
	if (!qpnp_vregs.vregs)
		return -ENOMEM;

	for_each_child_of_node(pdev->dev.of_node, node) {
		vreg = &qpnp_vregs.vregs[idx];
		vreg->id = idx++;

		key = "vreg_name";
		ret = of_property_read_string(node, key, &vreg->name);
		if (ret)
			pr_err("%s: Fail to get vreg name\n", __func__);

		key = "base_addr";
		ret = of_property_read_u32(node, key, &vreg->base_addr);
		if (ret)
			pr_err("%s: Fail to get vreg base address\n", __func__);

		key = "type";
		ret = htc_vreg_get_type(node, key, &vreg->type);
		if (ret)
			pr_err("%s: Fail to get vreg type\n", __func__);
	}
	node = pdev->dev.of_node;
        key = "en_ctl_offset";
        ret = of_property_read_u32(node, key, &qpnp_vregs.en_ctl_offset);
        if (ret)
		pr_err("%s: Fail to get enable control offset\n", __func__);

        key = "pd_ctl_offset";
        ret = of_property_read_u32(node, key, &qpnp_vregs.pd_ctl_offset);
        if (ret)
		pr_err("%s: Fail to get pull down control offset\n", __func__);

        key = "mode_ctl_offset";
        ret = of_property_read_u32(node, key, &qpnp_vregs.mode_ctl_offset);
        if (ret)
		pr_err("%s: Fail to get mode control offset\n", __func__);

	key = "range_ctl_offset";
        ret = of_property_read_u32(node, key, &qpnp_vregs.range_ctl_offset);
        if (ret)
		pr_err("%s: Fail to get range control offset\n", __func__);

	key = "step_ctl_offset";
        ret = of_property_read_u32(node, key, &qpnp_vregs.step_ctl_offset);
        if (ret)
		pr_err("%s: Fail to get step control offset\n", __func__);

        key = "en_bit";
        ret = of_property_read_u32(node, key, &qpnp_vregs.en_bit);
        if (ret)
		pr_err("%s: Fail to get enable bit offset\n", __func__);

        key = "pd_bit";
        ret = of_property_read_u32(node, key, &qpnp_vregs.pd_bit);
        if (ret)
		pr_err("%s: Fail to get pull down bit offset\n", __func__);

	htc_vreg_dump_debugfs_init();
	
	if (get_tamper_sf() == 0 && board_is_super_cid())
	{
		htc_voltage_debugfs_init();
		htc_vreg_switch_debugfs_init();
	}
	return 0;
}

static int __devexit htc_vreg_dump_remove(struct spmi_device *spmi)
{
	return 0;
}

static struct of_device_id vreg_match_table[] = {
	{ .compatible = VREG_DUMP_DRIVER_NAME, },
	{}
};


static struct platform_driver htc_vreg_dump_driver = {
	.probe		= htc_vreg_dump_probe,
	.driver		= {
		.name	= VREG_DUMP_DRIVER_NAME,
		.of_match_table = vreg_match_table,
		.owner = THIS_MODULE,
	},
};

int __init htc_vreg_dump_init(void)
{
	return platform_driver_register(&htc_vreg_dump_driver);
}

late_initcall(htc_vreg_dump_init);
#endif

module_exit(spmi_dfs_destroy);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:spmi_debug_fs");
