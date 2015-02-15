/* arch/arm/mach-msm/mnemosyne.c
 *
 * Copyright (C) 2013 HTC Corporation.
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
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/of.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <mach/msm_iomap.h>
#include <mach/htc_mnemosyne.h>

#define MNEMOSYNE_MODULE_NAME	"mnemosyne"

struct mnemosyne_data *mnemosyne_phys;
EXPORT_SYMBOL(mnemosyne_phys);
struct mnemosyne_data *mnemosyne_base;
EXPORT_SYMBOL(mnemosyne_base);

static atomic_t mnemosync_is_init = ATOMIC_INIT(0);

struct mnemosyne_meta {
	char *name;
	int num;
	int index;	/* start index in raw data */
};

#undef DECLARE_MNEMOSYNE_START
#undef DECLARE_MNEMOSYNE_END
#undef DECLARE_MNEMOSYNE
#undef DECLARE_MNEMOSYNE_ARRAY

#define DECLARE_MNEMOSYNE_START()	struct mnemosyne_meta mnemosyne_meta_data[] = {

#define DECLARE_MNEMOSYNE_END()	};

#define DECLARE_MNEMOSYNE_ARRAY(meta_name, meta_num)	{ \
		.name = #meta_name,			\
		.num = meta_num,			\
		.index = -1,				\
	},

#define DECLARE_MNEMOSYNE(meta_name)	DECLARE_MNEMOSYNE_ARRAY(meta_name, 1)

#include <mach/htc_mnemosyne_footprint.inc>

struct mnemosyne_data *mnemosyne_get_base(void)
{
	return mnemosyne_base;
}
EXPORT_SYMBOL(mnemosyne_get_base);

static int mnemosyne_setup(unsigned int phys, unsigned base)
{
	int i, index;

	if (atomic_read(&mnemosync_is_init)) {
		WARN(1, "%s: init again with phys=0x%08x, bas=0x%08x!\n", MNEMOSYNE_MODULE_NAME, phys, base);
		return -EIO;
	}

	mnemosyne_phys = (struct mnemosyne_data *) phys;
	mnemosyne_base = (struct mnemosyne_data *) base;

	pr_info("%s: phys: 0x%p\n", MNEMOSYNE_MODULE_NAME, mnemosyne_phys);
	pr_info("%s: base: 0x%p\n", MNEMOSYNE_MODULE_NAME, mnemosyne_base);
	pr_info("%s: init success.\n", MNEMOSYNE_MODULE_NAME);

	/* fill start index of each footprint to save query time */
	for (i=0, index=0; i<sizeof(mnemosyne_meta_data)/sizeof(struct mnemosyne_meta); i++) {
		mnemosyne_meta_data[i].index = index;
		index += mnemosyne_meta_data[i].num;
	}

	atomic_inc(&mnemosync_is_init);

	return 0;
}

#define OF_PROP_PHYS_NAME	"htc,phys"
#define OF_PROP_BASE_NAME	"htc,base"
static int mnemosyne_parse_dt(struct device* dev)
{
	struct device_node *node= NULL;
	u32 phys, base;
	int ret;

	pr_info("%s: init from device tree.\n", MNEMOSYNE_MODULE_NAME);

	node = dev->of_node;
	if (node == NULL) {
		pr_err("%s, Can't find device_node", MNEMOSYNE_MODULE_NAME);
		ret = -ENODEV;
		goto PARSE_DT_ERR_OUT;
	}

	ret = of_property_read_u32(node, OF_PROP_PHYS_NAME, &phys);
	if (ret) {
		pr_err("%s: could not get %s for %s\n", MNEMOSYNE_MODULE_NAME, OF_PROP_PHYS_NAME, node->full_name);
		goto PARSE_DT_ERR_OUT;
	}

	ret = of_property_read_u32(node, OF_PROP_BASE_NAME, &base);
	if (ret) {
		pr_err("%s: could not get %s for %s\n", MNEMOSYNE_MODULE_NAME, OF_PROP_BASE_NAME, node->full_name);
		goto PARSE_DT_ERR_OUT;
	}

	mnemosyne_setup(phys, base);

PARSE_DT_ERR_OUT:
	return ret;
}

static int mnemosyne_parse_pdata(struct device* dev)
{
	int ret = 0;
	struct mnemosyne_platform_data *pdata = (struct mnemosyne_platform_data *)dev->platform_data;

	pr_info("%s: init from platform data.\n", MNEMOSYNE_MODULE_NAME);

	if (pdata == NULL) {
		pr_err("%s: No pdata\n", MNEMOSYNE_MODULE_NAME);
		ret = -ENODEV;
		goto PARSE_PDATA_ERR_OUT;
	}

	mnemosyne_setup(pdata->phys, pdata->base);

PARSE_PDATA_ERR_OUT:
	return ret;
}

#ifdef CONFIG_DEBUG_FS
static ssize_t is_init_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char temp_buf[32];
	int size = 0;
	size = sprintf(temp_buf, "%d\n", atomic_read(&mnemosync_is_init));
	return simple_read_from_buffer(buf, count, ppos, temp_buf, size);
}

static const struct file_operations is_init_fops = {
	.read = is_init_read,
};

static void* rawdata_seq_start(struct seq_file *sfile, loff_t *pos)
{
	/* early return for non-init driver. */
	if (atomic_read(&mnemosync_is_init) == 0) {
		pr_warn("%s: not init!\n", MNEMOSYNE_MODULE_NAME);
		return NULL;
	}

	if (*pos < 0)
		*pos = 0;
	else if (*pos >= sizeof(mnemosyne_meta_data)/sizeof(struct mnemosyne_meta))
		return NULL;

	return pos;
}

static void* rawdata_seq_next(struct seq_file *sfile, void *v, loff_t *pos)
{
	++*pos;

	if (*pos >= sizeof(mnemosyne_meta_data)/sizeof(struct mnemosyne_meta))
		return NULL;

	return pos;
}

static void rawdata_seq_stop(struct seq_file *sfile, void *v)
{
	return;
}

static int rawdata_seq_show(struct seq_file *sfile, void *v)
{
	uint32_t pos = *(uint32_t *)v;
	struct mnemosyne_meta* meta = (struct mnemosyne_meta*)mnemosyne_meta_data;
	struct mnemosyne_data *data = mnemosyne_base;
	MNEMOSYNE_ELEMENT_TYPE *rawdata = (MNEMOSYNE_ELEMENT_TYPE *)data;

	int i;

	for (i=0, rawdata+=meta[pos].index; i<meta[pos].num; i++) {
		seq_printf(sfile, "%s", meta[pos].name);
		if (meta[pos].num > 1)
			seq_printf(sfile, "[%d]", i);
		seq_printf(sfile, ": 0x%08x = %u\n", rawdata[i], rawdata[i]);
	}
	return 0;
}

static struct seq_operations rawdata_seq_ops = {
	.start = rawdata_seq_start,
	.next = rawdata_seq_next,
	.stop = rawdata_seq_stop,
	.show = rawdata_seq_show
};

static int rawdata_seq_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &rawdata_seq_ops);
}

static const struct file_operations rawdata_fops = {
	.open		= rawdata_seq_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
	.owner		= THIS_MODULE,
};

static struct dentry *base_dir;

static int __devinit mnemosyne_debugfs_setup(void)
{
	base_dir = debugfs_create_dir(MNEMOSYNE_MODULE_NAME, NULL);

	if (!base_dir) {
		pr_err("%s: create debugfs dir failed\n", MNEMOSYNE_MODULE_NAME);
		return -EIO;
	}

	debugfs_create_file("is_init", S_IRUGO, base_dir, NULL, &is_init_fops);
	debugfs_create_file("rawdata", S_IRUGO, base_dir, NULL, &rawdata_fops);

	return 0;
}
#endif

static int __devinit mnemosyne_probe(struct platform_device *pdev)
{
	if (pdev->dev.of_node) {
		if (mnemosyne_parse_dt(&pdev->dev)) {
			pr_err("%s: parse device tree fail.\n", MNEMOSYNE_MODULE_NAME);
			return -ENODEV;
		}
	} else {
		if (mnemosyne_parse_pdata(&pdev->dev)) {
			pr_err("%s: parse pdata fail.\n", MNEMOSYNE_MODULE_NAME);
			return -ENODEV;
		}
	}

	return 0;
}

static int __devexit mnemosyne_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id mnemosyne_match[] = {
	{       .compatible = "htc,mnemosyne",},
	{},
};

static struct platform_driver mnemosyne_driver = {
	.probe          = mnemosyne_probe,
	.remove         = __devexit_p(mnemosyne_remove),
	.driver         = {
		.name = MNEMOSYNE_MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mnemosyne_match,
	},
};

static int __init mnemosyne_init(void)
{
#ifdef CONFIG_DEBUG_FS
	mnemosyne_debugfs_setup();
#endif

	return platform_driver_register(&mnemosyne_driver);
}

/* For board_early_init use, if called, no need to do device register. */
int mnemosyne_early_init(unsigned int phys, unsigned base)
{
	pr_info("%s: init from early init.\n", MNEMOSYNE_MODULE_NAME);
	return mnemosyne_setup(phys, base);
}

/* do not move it before or equal to core_init, because we need /sys/kernel for SYSFS node. */
arch_initcall(mnemosyne_init);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Jimmy Chen <jimmy.cm_chen@htc.com>");
MODULE_DESCRIPTION("HTC Footprint driver");
MODULE_VERSION("1.1");
MODULE_ALIAS("platform:mnemosyne");

