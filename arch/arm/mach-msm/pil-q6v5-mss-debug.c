#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/spmi.h>
#include <linux/of.h>
#include <mach/subsystem_restart.h>
#include "pil-q6v5-mss-debug.h"

struct pil_mss_debug{
	struct spmi_controller *ctrl;
	int num_adds;
	int *sid;
	int *ad;
};

#define MODEM_DEBUG_PMIC_DEV_NAME "htc,modem-debug-dump-pmic"
static struct pil_mss_debug mss_debug;

static struct of_device_id modem_debug_pmic_match_table[] = {
	{ .compatible = MODEM_DEBUG_PMIC_DEV_NAME },
	{}
};


static u8 debug_dump_pmic_register(u8 sid, u16 ad)
{
	u8 val = 0x00;
	if(NULL == mss_debug.ctrl)
		return val;
	spmi_ext_register_readl(mss_debug.ctrl, sid, ad, &val, 1);
	return val;
}

void modem_read_spmi_setting(int ramdump_enable)
{
	int i, offset;
	u8 val;
	char log_banner[0x50];
	char log_buffer[0x50];
	if(!ramdump_enable)
		return;

	pr_info("%s++\n", __func__);
	offset = 0;
	memset(log_banner, 0, sizeof(log_banner));
	memset(log_buffer, 0, sizeof(log_buffer));
	for(i = 0;i < mss_debug.num_adds;i++)
	{
		val = debug_dump_pmic_register(*(mss_debug.sid + i), *(mss_debug.ad + i));
		sprintf(log_banner + offset, "[%d]%04X ", *(mss_debug.sid + i), *(mss_debug.ad + i));
		offset += sprintf(log_buffer + offset, "   0x%02X ", val);
		if(i%8 == 7)
		{
			pr_info("%s\n", log_banner);
			pr_info("%s\n", log_buffer);
			offset = 0;
			memset(log_banner, 0, sizeof(log_banner));
			memset(log_buffer, 0, sizeof(log_buffer));
		}
	}
	if(i%8 != 7)
	{
		pr_info("%s\n", log_banner);
		pr_info("%s\n", log_buffer);
	}
	pr_info("%s--\n", __func__);
}

EXPORT_SYMBOL(modem_read_spmi_setting);

#define NUM_COL 2

static int parse_tbl(struct device *dev, char *prop, int num_cols,
		u32 **col1, u32 **col2)
{
	int ret, prop_len, num_rows, i, j, k;
	u32 *prop_data;
	u32 *col[num_cols];

	if (!of_find_property(dev->of_node, prop, &prop_len))
		return -EINVAL;

	prop_len /= sizeof(*prop_data);

	if (prop_len % num_cols || prop_len == 0)
		return -EINVAL;

	num_rows = prop_len / num_cols;

	prop_data = devm_kzalloc(dev, prop_len * sizeof(*prop_data),
				 GFP_KERNEL);
	if (!prop_data)
		return -ENOMEM;

	for (i = 0; i < num_cols; i++) {
		col[i] = devm_kzalloc(dev, num_rows * sizeof(u32), GFP_KERNEL);
		if (!col[i])
			return -ENOMEM;
	}

	ret = of_property_read_u32_array(dev->of_node, prop, prop_data,
					 prop_len);
	if (ret)
		return ret;

	k = 0;
	for (i = 0; i < num_rows; i++) {
		for (j = 0; j < num_cols; j++)
			col[j][i] = prop_data[k++];
	}
	if (col1)
		*col1 = col[0];
	if (col2)
		*col2 = col[1];

	devm_kfree(dev, prop_data);

	return num_rows;
}

static int __devinit modem_debg_pmic_probe(struct spmi_device *spmi)
{
	struct device *dev = &spmi->dev;

	pr_info("%s\n", __func__);

	mss_debug.num_adds = parse_tbl(dev, "htc,dump-register", NUM_COL, (u32 **)&mss_debug.sid, (u32 **)&mss_debug.ad);

	if(mss_debug.num_adds < 0) {
		dev_err(dev, "Unable to load register table\n");
		return mss_debug.num_adds;
	}
	mss_debug.ctrl = spmi->ctrl;
	return 0;
}

static struct spmi_driver qpnp_modem_debug_pmic_driver = {
	.probe	= modem_debg_pmic_probe,
	.driver	= {
		.name		= MODEM_DEBUG_PMIC_DEV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= modem_debug_pmic_match_table,
	},
};

static int __init pil_mss_debug_init(void)
{
	return spmi_driver_register(&qpnp_modem_debug_pmic_driver);
}
module_init(pil_mss_debug_init);

MODULE_DESCRIPTION("Support for modem debugging");
MODULE_LICENSE("GPL v2");
