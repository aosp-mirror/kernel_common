// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtual Power Management Controller Driver
 *
 * Author: Grzegorz Jaszczyk <jaz@semihalf.com>
 */

#include <linux/acpi.h>
#include <linux/platform_device.h>

#define ACPI_VIRT_PMC_DSM_UUID	"9ea49ba3-434a-49a6-be30-37cc55c4d397"
#define ACPI_VIRT_PMC_NOTIFY 1

static acpi_handle virt_pmc_handle;

static void virt_pmc_s2idle_notify(void)
{
	union acpi_object *out_obj;
	guid_t dsm_guid;

	guid_parse(ACPI_VIRT_PMC_DSM_UUID, &dsm_guid);

	out_obj = acpi_evaluate_dsm(virt_pmc_handle, &dsm_guid,
					0, ACPI_VIRT_PMC_NOTIFY, NULL);

	acpi_handle_debug(virt_pmc_handle, "_DSM function %u evaluation %s\n",
			  ACPI_VIRT_PMC_NOTIFY, out_obj ? "successful" : "failed");

	ACPI_FREE(out_obj);
}

static struct acpi_s2idle_dev_ops pmc_s2idle_dev_ops = {
	.check = virt_pmc_s2idle_notify,
};

static int virt_pmc_probe(struct platform_device *pdev)
{
	int err = 0;
	guid_t dsm_guid;

	virt_pmc_handle = ACPI_HANDLE(&pdev->dev);

	guid_parse(ACPI_VIRT_PMC_DSM_UUID, &dsm_guid);

	if (!acpi_check_dsm(virt_pmc_handle, &dsm_guid, 0,
			    1 << ACPI_VIRT_PMC_NOTIFY)) {
		dev_err(&pdev->dev, "DSM method doesn't support ACPI_VIRT_PMC_NOTIFY\n");
		return -ENODEV;
	}

	err = acpi_register_lps0_dev(&pmc_s2idle_dev_ops);
	if (err)
		dev_err(&pdev->dev, "failed to register LPS0 sleep handler\n");

	return err;
}

static int virt_pmc_remove(struct platform_device *pdev)
{
	acpi_unregister_lps0_dev(&pmc_s2idle_dev_ops);

	return 0;
}

static const struct acpi_device_id virt_pmc_acpi_ids[] = {
	{"HYPE0001", 0}, /* _HID for XXX Power Engine, _CID PNP0D80*/
	{ }
};
MODULE_DEVICE_TABLE(acpi, virt_pmc_acpi_ids);

static struct platform_driver virt_pmc_driver = {
	.driver = {
		.name = "virtual_pmc",
		.acpi_match_table = ACPI_PTR(virt_pmc_acpi_ids),
	},
	.probe = virt_pmc_probe,
	.remove = virt_pmc_remove,
};

module_platform_driver(virt_pmc_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Virtual PMC Driver");
