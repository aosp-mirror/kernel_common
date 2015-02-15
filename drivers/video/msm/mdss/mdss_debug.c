/* Copyright (c) 2009-2013, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/debugfs.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "mdss.h"
#include "mdss_mdp.h"
#include "mdss_mdp_hwio.h"
#include "mdss_debug.h"

#define DEFAULT_BASE_REG_CNT 0x100
#define GROUP_BYTES 4
#define ROW_BYTES 16
#define MAX_VSYNC_COUNT 0xFFFFFFF
struct mdss_debug_data {
	struct dentry *root;
	struct list_head base_list;
};

struct mdss_debug_base {
	struct mdss_debug_data *mdd;
	void __iomem *base;
	size_t off;
	size_t cnt;
	size_t max_offset;
	char *buf;
	size_t buf_len;
	struct list_head head;
};

static int mdss_debug_base_open(struct inode *inode, struct file *file)
{
	
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	file->private_data = inode->i_private;
	return 0;
}

static int mdss_debug_base_release(struct inode *inode, struct file *file)
{
	struct mdss_debug_base *dbg = file->private_data;
	if (dbg && dbg->buf) {
		kfree(dbg->buf);
		dbg->buf_len = 0;
		dbg->buf = NULL;
	}
	return 0;
}

static ssize_t mdss_debug_base_offset_write(struct file *file,
		    const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct mdss_debug_base *dbg = file->private_data;
	u32 off = 0;
	u32 cnt = DEFAULT_BASE_REG_CNT;
	char buf[24];

	if (!dbg)
		return -ENODEV;

	if (count >= sizeof(buf))
		return -EFAULT;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = 0;	

	sscanf(buf, "%5x %x", &off, &cnt);

	if (off > dbg->max_offset)
		return -EINVAL;

	if (cnt > (dbg->max_offset - off))
		cnt = dbg->max_offset - off;

	dbg->off = off;
	dbg->cnt = cnt;

	pr_debug("offset=%x cnt=%x\n", off, cnt);

	return count;
}

static ssize_t mdss_debug_base_offset_read(struct file *file,
			char __user *buff, size_t count, loff_t *ppos)
{
	struct mdss_debug_base *dbg = file->private_data;
	int len = 0;
	char buf[24];

	if (!dbg)
		return -ENODEV;

	if (*ppos)
		return 0;	

	len = snprintf(buf, sizeof(buf), "0x%08x %x\n", dbg->off, dbg->cnt);
	if (len < 0)
		return 0;

	if (copy_to_user(buff, buf, len))
		return -EFAULT;

	*ppos += len;	

	return len;
}

static ssize_t mdss_debug_base_reg_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct mdss_debug_base *dbg = file->private_data;
	struct mdss_data_type *mdata = mdss_res;
	size_t off;
	u32 data, cnt;
	char buf[24];

	if (!dbg || !mdata)
		return -ENODEV;

	if (count >= sizeof(buf))
		return -EFAULT;

	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;

	buf[count] = 0;	

	cnt = sscanf(buf, "%x %x", &off, &data);

	if (cnt < 2)
		return -EFAULT;

	if (off >= dbg->max_offset)
		return -EFAULT;

	if (mdata->debug_inf.debug_enable_clock)
		mdata->debug_inf.debug_enable_clock(1);

	writel_relaxed(data, dbg->base + off);

	if (mdata->debug_inf.debug_enable_clock)
		mdata->debug_inf.debug_enable_clock(0);

	pr_debug("addr=%x data=%x\n", off, data);

	return count;
}

static ssize_t mdss_debug_base_reg_read(struct file *file,
			char __user *user_buf, size_t count, loff_t *ppos)
{
	struct mdss_debug_base *dbg = file->private_data;
	struct mdss_data_type *mdata = mdss_res;
	size_t len;

	if (!dbg || !mdata) {
		pr_err("invalid handle\n");
		return -ENODEV;
	}

	if (!dbg->buf) {
		char dump_buf[64];
		char *ptr;
		int cnt, tot;

		dbg->buf_len = sizeof(dump_buf) *
			DIV_ROUND_UP(dbg->cnt, ROW_BYTES);
		dbg->buf = kzalloc(dbg->buf_len, GFP_KERNEL);

		if (!dbg->buf) {
			pr_err("not enough memory to hold reg dump\n");
			return -ENOMEM;
		}

		ptr = dbg->base + dbg->off;
		tot = 0;

		if (mdata->debug_inf.debug_enable_clock)
			mdata->debug_inf.debug_enable_clock(1);

		for (cnt = dbg->cnt; cnt > 0; cnt -= ROW_BYTES) {
			hex_dump_to_buffer(ptr, min(cnt, ROW_BYTES),
					   ROW_BYTES, GROUP_BYTES, dump_buf,
					   sizeof(dump_buf), false);
			len = scnprintf(dbg->buf + tot, dbg->buf_len - tot,
					"0x%08x: %s\n",
					((int)ptr) - ((int)dbg->base),
					dump_buf);

			ptr += ROW_BYTES;
			tot += len;
			if (tot >= dbg->buf_len)
				break;
		}
		if (mdata->debug_inf.debug_enable_clock)
			mdata->debug_inf.debug_enable_clock(0);

		dbg->buf_len = tot;
	}

	if (*ppos >= dbg->buf_len)
		return 0; 

	len = min(count, dbg->buf_len - (size_t) *ppos);
	if (copy_to_user(user_buf, dbg->buf + *ppos, len)) {
		pr_err("failed to copy to user\n");
		return -EFAULT;
	}

	*ppos += len; 

	return len;
}

static const struct file_operations mdss_off_fops = {
	.open = mdss_debug_base_open,
	.release = mdss_debug_base_release,
	.read = mdss_debug_base_offset_read,
	.write = mdss_debug_base_offset_write,
};

static const struct file_operations mdss_reg_fops = {
	.open = mdss_debug_base_open,
	.release = mdss_debug_base_release,
	.read = mdss_debug_base_reg_read,
	.write = mdss_debug_base_reg_write,
};

int mdss_debug_register_base(const char *name, void __iomem *base,
			     size_t max_offset)
{
	struct mdss_data_type *mdata = mdss_res;
	struct mdss_debug_data *mdd;
	struct mdss_debug_base *dbg;
	struct dentry *ent_off, *ent_reg;
	char dn[80] = "";
	int prefix_len = 0;

	if (!mdata || !mdata->debug_inf.debug_data)
		return -ENODEV;

	mdd = mdata->debug_inf.debug_data;

	dbg = kzalloc(sizeof(*dbg), GFP_KERNEL);
	if (!dbg)
		return -ENOMEM;

	dbg->base = base;
	dbg->max_offset = max_offset;
	dbg->off = 0;
	dbg->cnt = DEFAULT_BASE_REG_CNT;

	if (name)
		prefix_len = snprintf(dn, sizeof(dn), "%s_", name);

	strlcpy(dn + prefix_len, "off", sizeof(dn) - prefix_len);
	ent_off = debugfs_create_file(dn, 0644, mdd->root, dbg, &mdss_off_fops);
	if (IS_ERR_OR_NULL(ent_off)) {
		pr_err("debugfs_create_file: offset fail\n");
		goto off_fail;
	}

	strlcpy(dn + prefix_len, "reg", sizeof(dn) - prefix_len);
	ent_reg = debugfs_create_file(dn, 0644, mdd->root, dbg, &mdss_reg_fops);
	if (IS_ERR_OR_NULL(ent_reg)) {
		pr_err("debugfs_create_file: reg fail\n");
		goto reg_fail;
	}

	list_add(&dbg->head, &mdd->base_list);

	return 0;
reg_fail:
	debugfs_remove(ent_off);
off_fail:
	kfree(dbg);
	return -ENODEV;
}


static int mdss_debug_stat_open(struct inode *inode, struct file *file)
{
	
	file->f_mode &= ~(FMODE_LSEEK | FMODE_PREAD | FMODE_PWRITE);
	file->private_data = inode->i_private;
	return 0;
}

static int mdss_debug_stat_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t mdss_debug_stat_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	struct mdss_data_type *mdata = file->private_data;
	int len, tot;
	char bp[512];

	if (*ppos)
		return 0;	

	len = sizeof(bp);

	tot = scnprintf(bp, len, "\nmdp:\n");

	if (mdata->debug_inf.debug_dump_stats)
		tot += mdata->debug_inf.debug_dump_stats(mdata,
						bp + tot, len - tot);


	tot += scnprintf(bp + tot, len - tot, "\n");

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;	

	return tot;
}

static const struct file_operations mdss_stat_fops = {
	.open = mdss_debug_stat_open,
	.release = mdss_debug_stat_release,
	.read = mdss_debug_stat_read,
};

static int mdss_debugfs_cleanup(struct mdss_debug_data *mdd)
{
	struct mdss_debug_base *base, *tmp;

	if (!mdd)
		return 0;

	list_for_each_entry_safe(base, tmp, &mdd->base_list, head) {
		list_del(&base->head);
		kfree(base);
	}

	if (mdd->root)
		debugfs_remove_recursive(mdd->root);

	kfree(mdd);

	return 0;
}

int mdss_debugfs_init(struct mdss_data_type *mdata)
{
	struct mdss_debug_data *mdd;

	if (mdata->debug_inf.debug_data) {
		pr_warn("mdss debugfs already initialized\n");
		return -EBUSY;
	}

	mdd = kzalloc(sizeof(*mdd), GFP_KERNEL);
	if (!mdd) {
		pr_err("no memory to create mdss debug data\n");
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&mdd->base_list);

	mdd->root = debugfs_create_dir("mdp", NULL);
	if (IS_ERR_OR_NULL(mdd->root)) {
		pr_err("debugfs_create_dir fail, error %ld\n",
		       PTR_ERR(mdd->root));
		mdd->root = NULL;
		mdss_debugfs_cleanup(mdd);
		return -ENODEV;
	}
	debugfs_create_file("stat", 0644, mdd->root, mdata, &mdss_stat_fops);

	debugfs_create_u32("min_mdp_clk", 0644, mdd->root,
			(u32 *)&mdata->min_mdp_clk);

	mdata->debug_inf.debug_data = mdd;

	return 0;
}

int mdss_debugfs_remove(struct mdss_data_type *mdata)
{
	struct mdss_debug_data *mdd = mdata->debug_inf.debug_data;

	mdss_debugfs_cleanup(mdd);
	mdata->debug_inf.debug_data = NULL;

	return 0;
}

int vsync_count;
static struct mdss_mdp_misr_map {
	u32 ctrl_reg;
	u32 value_reg;
	u32 crc_op_mode;
	u32 crc_index;
	bool use_ping;
	bool is_ping_full;
	bool is_pong_full;
	struct mutex crc_lock;
	u32 crc_ping[MISR_CRC_BATCH_SIZE];
	u32 crc_pong[MISR_CRC_BATCH_SIZE];
} mdss_mdp_misr_table[DISPLAY_MISR_MAX] = {
	[DISPLAY_MISR_DSI0] = {
		.ctrl_reg = MDSS_MDP_LP_MISR_CTRL_DSI0,
		.value_reg = MDSS_MDP_LP_MISR_SIGN_DSI0,
		.crc_op_mode = 0,
		.crc_index = 0,
		.use_ping = true,
		.is_ping_full = false,
		.is_pong_full = false,
	},
	[DISPLAY_MISR_DSI1] = {
		.ctrl_reg = MDSS_MDP_LP_MISR_CTRL_DSI1,
		.value_reg = MDSS_MDP_LP_MISR_SIGN_DSI1,
		.crc_op_mode = 0,
		.crc_index = 0,
		.use_ping = true,
		.is_ping_full = false,
		.is_pong_full = false,
	},
	[DISPLAY_MISR_EDP] = {
		.ctrl_reg = MDSS_MDP_LP_MISR_CTRL_EDP,
		.value_reg = MDSS_MDP_LP_MISR_SIGN_EDP,
		.crc_op_mode = 0,
		.crc_index = 0,
		.use_ping = true,
		.is_ping_full = false,
		.is_pong_full = false,
	},
	[DISPLAY_MISR_HDMI] = {
		.ctrl_reg = MDSS_MDP_LP_MISR_CTRL_HDMI,
		.value_reg = MDSS_MDP_LP_MISR_SIGN_HDMI,
		.crc_op_mode = 0,
		.crc_index = 0,
		.use_ping = true,
		.is_ping_full = false,
		.is_pong_full = false,
	},
	[DISPLAY_MISR_MDP] = {
		.ctrl_reg = MDSS_MDP_LP_MISR_CTRL_MDP,
		.value_reg = MDSS_MDP_LP_MISR_SIGN_MDP,
		.crc_op_mode = 0,
		.crc_index = 0,
		.use_ping = true,
		.is_ping_full = false,
		.is_pong_full = false,
	},
};

static inline struct mdss_mdp_misr_map *mdss_misr_get_map(u32 block_id)
{
	struct mdss_mdp_misr_map *map;

	if (block_id > DISPLAY_MISR_MDP) {
		pr_err("MISR Block id (%d) out of range\n", block_id);
		return NULL;
	}

	map = mdss_mdp_misr_table + block_id;
	if ((map->ctrl_reg == 0) || (map->value_reg == 0)) {
		pr_err("MISR Block id (%d) config not found\n", block_id);
		return NULL;
	}

	return map;
}

int mdss_misr_set(struct mdss_data_type *mdata,
			struct mdp_misr *req,
			struct mdss_mdp_ctl *ctl)
{
	struct mdss_mdp_misr_map *map;
	struct mdss_mdp_mixer *mixer;
	u32 config = 0, val = 0;
	u32 mixer_num = 0;
	bool is_valid_wb_mixer = true;
	map = mdss_misr_get_map(req->block_id);
	if (!map) {
		pr_err("Invalid MISR Block=%d\n", req->block_id);
		return -EINVAL;
	}
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON, false);
	if (req->block_id == DISPLAY_MISR_MDP) {
		mixer = mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_DEFAULT);
		if (!mixer) {
			pr_err("mdss_mdp_mixer_get get NULL mixer\n");
			return -EINVAL;
		}
		mixer_num = mixer->num;
		pr_debug("SET MDP MISR BLK to MDSS_MDP_LP_MISR_SEL_LMIX%d_GC\n",
			req->block_id);
		switch (mixer_num) {
		case MDSS_MDP_INTF_LAYERMIXER0:
			pr_debug("Use Layer Mixer 0 for WB CRC\n");
			val = MDSS_MDP_LP_MISR_SEL_LMIX0_GC;
			break;
		case MDSS_MDP_INTF_LAYERMIXER1:
			pr_debug("Use Layer Mixer 1 for WB CRC\n");
			val = MDSS_MDP_LP_MISR_SEL_LMIX1_GC;
			break;
		case MDSS_MDP_INTF_LAYERMIXER2:
			pr_debug("Use Layer Mixer 2 for WB CRC\n");
			val = MDSS_MDP_LP_MISR_SEL_LMIX2_GC;
			break;
		default:
			pr_err("Invalid Layer Mixer %d selected for WB CRC\n",
				mixer_num);
			is_valid_wb_mixer = false;
			break;
		}
		if (is_valid_wb_mixer)
			writel_relaxed(val,
				mdata->mdp_base + MDSS_MDP_LP_MISR_SEL);
	}
	vsync_count = 0;
	map->crc_op_mode = req->crc_op_mode;
	config = (MDSS_MDP_LP_MISR_CTRL_FRAME_COUNT_MASK & req->frame_count) |
			(MDSS_MDP_LP_MISR_CTRL_ENABLE);

	writel_relaxed(MDSS_MDP_LP_MISR_CTRL_STATUS_CLEAR,
			mdata->mdp_base + map->ctrl_reg);
	
	wmb();

	memset(map->crc_ping, 0, sizeof(map->crc_ping));
	memset(map->crc_pong, 0, sizeof(map->crc_pong));
	map->crc_index = 0;
	map->use_ping = true;
	map->is_ping_full = false;
	map->is_pong_full = false;

	if (MISR_OP_BM != map->crc_op_mode) {

		writel_relaxed(config,
				mdata->mdp_base + map->ctrl_reg);
		pr_debug("MISR_CTRL = 0x%x",
				readl_relaxed(mdata->mdp_base + map->ctrl_reg));
	}
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF, false);
	return 0;
}

int mdss_misr_get(struct mdss_data_type *mdata,
			struct mdp_misr *resp,
			struct mdss_mdp_ctl *ctl)
{
	struct mdss_mdp_misr_map *map;
	u32 status;
	int ret = -1;
	int i;

	map = mdss_misr_get_map(resp->block_id);
	if (!map) {
		pr_err("Invalid MISR Block=%d\n", resp->block_id);
		return -EINVAL;
	}
	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON, false);
	switch (map->crc_op_mode) {
	case MISR_OP_SFM:
	case MISR_OP_MFM:
		ret = readl_poll_timeout(mdata->mdp_base + map->ctrl_reg,
				status, status & MDSS_MDP_LP_MISR_CTRL_STATUS,
				MISR_POLL_SLEEP, MISR_POLL_TIMEOUT);
		if (ret == 0) {
			resp->crc_value[0] = readl_relaxed(mdata->mdp_base +
				map->value_reg);
			pr_debug("CRC %d=0x%x\n", resp->block_id,
				resp->crc_value[0]);
			writel_relaxed(0, mdata->mdp_base + map->ctrl_reg);
		} else {
			mdss_mdp_ctl_write(ctl, MDSS_MDP_REG_CTL_START, 1);
			ret = readl_poll_timeout(mdata->mdp_base +
					map->ctrl_reg, status,
					status & MDSS_MDP_LP_MISR_CTRL_STATUS,
					MISR_POLL_SLEEP, MISR_POLL_TIMEOUT);
			if (ret == 0) {
				resp->crc_value[0] =
					readl_relaxed(mdata->mdp_base +
					map->value_reg);
			}
			writel_relaxed(0, mdata->mdp_base + map->ctrl_reg);
		}
		break;
	case MISR_OP_BM:
		if (map->is_ping_full) {
			for (i = 0; i < MISR_CRC_BATCH_SIZE; i++)
				resp->crc_value[i] = map->crc_ping[i];
			memset(map->crc_ping, 0, sizeof(map->crc_ping));
			map->is_ping_full = false;
			ret = 0;
		} else if (map->is_pong_full) {
			for (i = 0; i < MISR_CRC_BATCH_SIZE; i++)
				resp->crc_value[i] = map->crc_pong[i];
			memset(map->crc_pong, 0, sizeof(map->crc_pong));
			map->is_pong_full = false;
			ret = 0;
		} else {
			pr_debug("mdss_mdp_misr_crc_get PING BUF %s\n",
				map->is_ping_full ? "FULL" : "EMPTRY");
			pr_debug("mdss_mdp_misr_crc_get PONG BUF %s\n",
				map->is_pong_full ? "FULL" : "EMPTRY");
		}
		resp->crc_op_mode = map->crc_op_mode;
		break;
	default:
		ret = -ENOSYS;
		break;
	}

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF, false);
	return ret;
}

void mdss_misr_crc_collect(struct mdss_data_type *mdata, int block_id)
{
	struct mdss_mdp_misr_map *map;
	u32 status = 0;
	u32 crc = 0x0BAD0BAD;
	bool crc_stored = false;

	map = mdss_misr_get_map(block_id);
	if (!map || (map->crc_op_mode != MISR_OP_BM))
		return;

	status = readl_relaxed(mdata->mdp_base + map->ctrl_reg);
	if (MDSS_MDP_LP_MISR_CTRL_STATUS & status) {
		crc = readl_relaxed(mdata->mdp_base + map->value_reg);
		if (map->use_ping) {
			if (map->is_ping_full) {
				pr_err("PING Buffer FULL\n");
			} else {
				map->crc_ping[map->crc_index] = crc;
				crc_stored = true;
			}
		} else {
			if (map->is_pong_full) {
				pr_err("PONG Buffer FULL\n");
			} else {
				map->crc_pong[map->crc_index] = crc;
				crc_stored = true;
			}
		}

		if (crc_stored) {
			map->crc_index = (map->crc_index + 1);
			if (map->crc_index == MISR_CRC_BATCH_SIZE) {
				map->crc_index = 0;
				if (true == map->use_ping) {
					map->is_ping_full = true;
					map->use_ping = false;
				} else {
					map->is_pong_full = true;
					map->use_ping = true;
				}
				pr_debug("USE BUFF %s\n", map->use_ping ?
					"PING" : "PONG");
				pr_debug("mdss_misr_crc_collect PING BUF %s\n",
					map->is_ping_full ? "FULL" : "EMPTRY");
				pr_debug("mdss_misr_crc_collect PONG BUF %s\n",
					map->is_pong_full ? "FULL" : "EMPTRY");
			}
		} else {
			pr_err("CRC(%d) Not saved\n", crc);
		}

		writel_relaxed(MDSS_MDP_LP_MISR_CTRL_STATUS_CLEAR,
				mdata->mdp_base + map->ctrl_reg);
		writel_relaxed(MISR_CRC_BATCH_CFG,
				mdata->mdp_base + map->ctrl_reg);
	} else if (0 == status) {
		writel_relaxed(MISR_CRC_BATCH_CFG,
				mdata->mdp_base + map->ctrl_reg);
		pr_debug("$$ Batch CRC Start $$\n");
	}
	pr_debug("$$ Vsync Count = %d, CRC=0x%x Indx = %d$$\n",
		vsync_count, crc, map->crc_index);

	if (MAX_VSYNC_COUNT == vsync_count) {
		pr_err("RESET vsync_count(%d)\n", vsync_count);
		vsync_count = 0;
	} else {
		vsync_count += 1;
	}
}
