#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <asm/io.h>

#include "htc_radio_smem.h"

static void htc_radio_smem_write(struct htc_smem_type *smem)
{
	
	strncpy(smem->RCMS_Name, RCMS_NAME, sizeof(smem->RCMS_Name));

	pr_info("[smem]%s: RCMS_NAME=%s, version=0x%x, size=%d, "
			"htc_smem_app_run_mode=0x%x .\n",
			__func__, smem->RCMS_Name, smem->version,
			smem->struct_size,
			smem->htc_smem_app_run_mode);
}

static int htc_radio_smem_probe(struct platform_device *pdev)
{
	int ret = -1;
	char *key;
	struct resource *res;
	phys_addr_t smem_start_addr;
	struct htc_smem_type *htc_radio_smem;

	pr_info("[smem]%s: start.\n", __func__);

	
	key = "smem-start-addr";
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, key);
	if(!res){
		ret = -ENODEV;
		goto missing_key;
	}

	smem_start_addr = res->start;

	htc_radio_smem = ioremap(smem_start_addr, sizeof(struct htc_smem_type));

	if(htc_radio_smem) {
		pr_info("[smem]%s: htc_radio_smem=0x%p.\n", __func__, htc_radio_smem);
	}else{
		ret = -ENOMEM;
		goto ioremap_fail;
	}

	
	htc_radio_smem_write(htc_radio_smem);

	iounmap(htc_radio_smem);

	pr_info("[smem]%s: end.\n", __func__);

	return 0;

missing_key:
	pr_err("[smem]%s: missing key: %s", __func__, key);
	return ret;
ioremap_fail:
	pr_err("[smem]%s: ioremap fail, htc_radio_smem:%p.\n", __func__, htc_radio_smem);
	return ret;
}

static struct of_device_id htc_radio_smem_of_match[] = {
	{.compatible = "htc,htc_radio_smem",},
	{},
};
MODULE_DEVICE_TABLE(of, htc_radio_smem_of_match);

static struct platform_driver htc_radio_smem_driver = {
	.probe = htc_radio_smem_probe,
	.driver = {
		.name = "htc_radio_smem",
		.owner = THIS_MODULE,
		.of_match_table = htc_radio_smem_of_match,
	},
};

static int __init htc_radio_smem_init(void)
{
	int ret = -1;
	pr_info("[smem]%s.\n", __func__);

	ret = platform_driver_register(&htc_radio_smem_driver);
	if(ret < 0 ) {
		pr_err("[smem]%s platform_driver register fail. ret:%d\n", __func__, ret);
		goto register_fail;
	}

register_fail:
	return ret;
}

static void __exit htc_radio_smem_exit(void)
{
	platform_driver_unregister(&htc_radio_smem_driver);
}

module_init(htc_radio_smem_init);
module_exit(htc_radio_smem_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("htc radio smem driver");
