/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/htc_flashlight.h>
#include "msm_led_flash.h"
#include <mach/devices_cmdline.h>

#define FLASH_NAME "camera-led-lm3643"

#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

extern int32_t msm_led_torch_create_classdev(
				struct platform_device *pdev, void *data);

static enum flash_type flashtype;
static struct msm_led_flash_ctrl_t fctrl;

static int32_t msm_led_trigger_get_subdev_id(struct msm_led_flash_ctrl_t *fctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	if (!subdev_id) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return -EINVAL;
	}
	*subdev_id = fctrl->pdev->id;
	CDBG("%s:%d subdev_id %d\n", __func__, __LINE__, *subdev_id);
	return 0;
}

static int32_t msm_led_trigger_config(struct msm_led_flash_ctrl_t *fctrl,
	void *data)
{
	int rc = 0;
	struct msm_camera_led_cfg_t *cfg = (struct msm_camera_led_cfg_t *)data;
	int led1 = 0 ,led2 = 0;
	CDBG("called led_state %d\n", cfg->cfgtype);

	if (!fctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}

	switch (cfg->cfgtype) {
	case MSM_CAMERA_LED_OFF:
#ifdef CONFIG_FRONT_FLASHLIGHT_COMMON
        htc_flashlight_flash(0,0);
#endif
		break;

	case MSM_CAMERA_LED_LOW:
#ifdef CONFIG_FRONT_FLASHLIGHT_COMMON
        led1 = (int)cfg->ma_value & 0xFFFF;
        led2 = (cfg->ma_value & 0xFFFF0000)>>16;
        pr_info("[CAM][FL] torch led1[%d]led2[%d]", led1, led2);
        htc_flashlight_torch(led1, led2);
#endif
		break;

	case MSM_CAMERA_LED_HIGH:
        pr_info("[CAM][FL] called linear flashlight current value %d", (int)cfg->ma_value);
        led1 = (int)cfg->ma_value & 0xFFFF;
        led2 = (cfg->ma_value & 0xFFFF0000)>>16;
        pr_info("[CAM][FL] led1[%d]led2[%d]", led1, led2);
#ifdef CONFIG_FRONT_FLASHLIGHT_COMMON
        htc_flashlight_flash(led1, led2);
#endif
		break;

	case MSM_CAMERA_LED_INIT:
	case MSM_CAMERA_LED_RELEASE:
#ifdef CONFIG_FRONT_FLASHLIGHT_COMMON
        htc_flashlight_flash(0,0);
#endif
		break;

	default:
		rc = -EFAULT;
		break;
	}
	CDBG("flash_set_led_state: return %d\n", rc);
	return rc;
}

static const struct of_device_id msm_led_trigger_dt_match[] = {
	{.compatible = "htc,camera-led-lm3643"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_led_trigger_dt_match);

static struct platform_driver msm_led_trigger_driver = {
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = msm_led_trigger_dt_match,
	},
};

static int32_t msm_led_trigger_probe(struct platform_device *pdev)
{
	int32_t rc = 0, rc_1 = 0, i = 0;
	struct device_node *of_node = pdev->dev.of_node;
	struct device_node *flash_src_node = NULL;
	uint32_t count = 0;
	struct led_trigger *temp = NULL;

	CDBG("called\n");

	if (!of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	fctrl.pdev = pdev;

	rc = of_property_read_u32(of_node, "cell-index", &pdev->id);
	if (rc < 0) {
		pr_err("failed\n");
		return -EINVAL;
	}
	CDBG("pdev id %d\n", pdev->id);

	rc = of_property_read_u32(of_node,
			"qcom,flash-type", &flashtype);
	if (rc < 0) {
		pr_err("flash-type: read failed\n");
		return -EINVAL;
	}

	if (of_get_property(of_node, "qcom,flash-source", &count)) {
		count /= sizeof(uint32_t);
		CDBG("count %d\n", count);
		if (count > MAX_LED_TRIGGERS) {
			pr_err("invalid count\n");
			return -EINVAL;
		}
		fctrl.num_sources = count;
		for (i = 0; i < count; i++) {
			flash_src_node = of_parse_phandle(of_node,
				"qcom,flash-source", i);
			if (!flash_src_node) {
				pr_err("flash_src_node NULL\n");
				continue;
			}

			rc = of_property_read_string(flash_src_node,
				"linux,default-trigger",
				&fctrl.led_trigger_name[i]);
			if (rc < 0) {
				pr_err("default-trigger: read failed\n");
				of_node_put(flash_src_node);
				continue;
			}

			pr_info("default trigger %s\n", fctrl.led_trigger_name[i]);

			if (flashtype == GPIO_FLASH) {
				
				fctrl.op_current[i] = LED_FULL;
			} else {
				rc = of_property_read_u32(flash_src_node,
					"qcom,current",
					&fctrl.op_current[i]);
				rc_1 = of_property_read_u32(flash_src_node,
					"qcom,max-current",
					&fctrl.max_current[i]);
				if ((rc < 0) || (rc_1 < 0)) {
					pr_err("current: read failed\n");
					of_node_put(flash_src_node);
					continue;
				}
			}

			of_node_put(flash_src_node);

			pr_info("[CAM][FL] max_current[%d] %d\n", i, fctrl.op_current[i]);

			led_trigger_register_simple(fctrl.led_trigger_name[i],
				&fctrl.led_trigger[i]);

			if (flashtype == GPIO_FLASH)
				if (fctrl.led_trigger[i])
					temp = fctrl.led_trigger[i];
		}

		
		flash_src_node = of_parse_phandle(of_node, "qcom,torch-source",
			0);
		if (flash_src_node) {
			rc = of_property_read_string(flash_src_node,
				"linux,default-trigger",
				&fctrl.torch_trigger_name);
			if (rc < 0) {
				pr_err("default-trigger: read failed\n");
				goto torch_failed;
			}

			CDBG("default trigger %s\n",
				fctrl.torch_trigger_name);

			if (flashtype == GPIO_FLASH) {
				
				fctrl.torch_op_current = LED_FULL;
				if (temp)
					fctrl.torch_trigger = temp;
				else
					led_trigger_register_simple(
						fctrl.torch_trigger_name,
						&fctrl.torch_trigger);
			} else {
				rc = of_property_read_u32(flash_src_node,
					"qcom,current",
					&fctrl.torch_op_current);
				rc_1 = of_property_read_u32(flash_src_node,
					"qcom,max-current",
					&fctrl.torch_max_current);

				if ((rc < 0) || (rc_1 < 0)) {
					pr_err("current: read failed\n");
					goto torch_failed;
				}

				CDBG("torch max_current %d\n",
					fctrl.torch_op_current);

				led_trigger_register_simple(
					fctrl.torch_trigger_name,
					&fctrl.torch_trigger);
			}
torch_failed:
			of_node_put(flash_src_node);
		}
	}

	rc = msm_led_flash_create_v4lsubdev(pdev, &fctrl);
	if (!rc)
		msm_led_torch_create_classdev(pdev, &fctrl);

	return rc;
}

static int __init lm3643_led_trigger_add_driver(void)
{
	int32_t rc = 0;
	if (board_mfg_mode() == MFG_MODE_OFFMODE_CHARGING) {
		pr_err("offmode_charging, skip probe\n");
		return 0;
	}
	rc = platform_driver_probe(&msm_led_trigger_driver,
		msm_led_trigger_probe);
	pr_info("called rc =%d\n", rc);
	return rc;
}

static struct msm_flash_fn_t msm_led_trigger_func_tbl = {
	.flash_get_subdev_id = msm_led_trigger_get_subdev_id,
	.flash_led_config = msm_led_trigger_config,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.func_tbl = &msm_led_trigger_func_tbl,
};

module_init(lm3643_led_trigger_add_driver);
MODULE_DESCRIPTION("camera-led-lm3643");
MODULE_LICENSE("GPL v2");
