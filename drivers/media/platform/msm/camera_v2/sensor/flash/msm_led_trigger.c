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

#define FLASH_NAME "camera-led-flash"

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

#if 1 
static struct kobject *led_status_obj; 
#endif 

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
	pr_info("[CAM][FL] called led_state %d\n", cfg->cfgtype);

	if (!fctrl->led_trigger[0]) {
		pr_err("failed\n");
		return -EINVAL;
	}
	switch (cfg->cfgtype) {
	case MSM_CAMERA_LED_OFF:
        #ifndef CONFIG_FLASHLIGHT_TPS61310 
        led_trigger_event(fctrl->led_trigger[0], 0);
        #else
        tps61310_flashlight_control(FL_MODE_OFF);
        #endif 
        break;

	case MSM_CAMERA_LED_LOW:
        #ifndef CONFIG_FLASHLIGHT_TPS61310 
        led_trigger_event(fctrl->led_trigger[0], fctrl->op_current[0] / 2);
        #else
        tps61310_flashlight_control(FL_MODE_PRE_FLASH);
        #endif 
        break;

	case MSM_CAMERA_LED_HIGH:
        #ifndef CONFIG_FLASHLIGHT_TPS61310 
        led_trigger_event(fctrl->led_trigger[0], fctrl->op_current[0]);
        #else
        pr_info("[CAM][FL] called linear flashlight current value %d", (int)cfg->ma_value);
        if (cfg->ma_value == 0)
          tps61310_flashlight_control(FL_MODE_FLASH_LEVEL7);
        else{
          int led1 = (int)cfg->ma_value & 0xFFFF;
          int led2 = (cfg->ma_value & 0xFFFF0000)>>16;
          pr_info("[CAM][FL] led1[%d]led2[%d]", led1, led2);
          if(led1 == 1500 && led2 == 0){
              tps61310_flashlight_mode(led1);
          }else{
              tps61310_flashlight_mode2(led1, led2);
          }
        }
        #endif 
        break;

	case MSM_CAMERA_LED_INIT:
	case MSM_CAMERA_LED_RELEASE:
        #ifndef CONFIG_FLASHLIGHT_TPS61310 
        led_trigger_event(fctrl->led_trigger[0], 0);
        #else
        tps61310_flashlight_control(FL_MODE_OFF);
        #endif 
        break;

	default:
		rc = -EFAULT;
		break;
	}
	pr_info("[CAM][FL] flash_set_led_state: return %d\n", rc);
	return rc;
}

static const struct of_device_id msm_led_trigger_dt_match[] = {
	{.compatible = "qcom,camera-led-flash"},
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

	pr_info("[CAM][FL] called\n");

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
	pr_info("[CAM][FL] pdev id %d\n", pdev->id);

	rc = of_property_read_u32(of_node,
			"qcom,flash-type", &flashtype);
	if (rc < 0) {
		pr_err("flash-type: read failed\n");
		return -EINVAL;
	}

	if (of_get_property(of_node, "qcom,flash-source", &count)) {
		count /= sizeof(uint32_t);
		pr_info("[CAM][FL] count %d\n", count);
		if (count > MAX_LED_TRIGGERS) {
			pr_err("invalid count\n");
			return -EINVAL;
		}
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

#if 1 

static uint32_t led_ril_status_value;
static uint32_t led_wimax_status_value;
static uint32_t led_hotspot_status_value;
static uint16_t led_low_temp_limit = 5;
static uint16_t led_low_cap_limit = 14;
static uint16_t led_low_cap_limit_dual = 14;

static ssize_t led_ril_status_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_ril_status_value);
	return length;
}

static ssize_t led_ril_status_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	if (buf[1] == '\n')
		tmp = buf[0] - 0x30;

	led_ril_status_value = tmp;
	pr_info("[CAM][FL] led_ril_status_value = %d\n", led_ril_status_value);
	return count;
}

static ssize_t led_wimax_status_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_wimax_status_value);
	return length;
}

static ssize_t led_wimax_status_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	if (buf[1] == '\n')
		tmp = buf[0] - 0x30;

	led_wimax_status_value = tmp;
	pr_info("[CAM][FL] led_wimax_status_value = %d\n", led_wimax_status_value);
	return count;
}

static ssize_t led_hotspot_status_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_hotspot_status_value);
	return length;
}

static ssize_t led_hotspot_status_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	tmp = buf[0] - 0x30; 

	led_hotspot_status_value = tmp;
	pr_info("[CAM][FL] led_hotspot_status_value = %d\n", led_hotspot_status_value);
	return count;
}

static ssize_t low_temp_limit_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_low_temp_limit);
	return length;
}

static ssize_t low_cap_limit_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_low_cap_limit);
	return length;
}

static ssize_t low_cap_limit_dual_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_low_cap_limit_dual);
	return length;
}

static DEVICE_ATTR(led_ril_status, 0644,
	led_ril_status_get,
	led_ril_status_set);

static DEVICE_ATTR(led_wimax_status, 0644,
	led_wimax_status_get,
	led_wimax_status_set);

static DEVICE_ATTR(led_hotspot_status, 0644,
	led_hotspot_status_get,
	led_hotspot_status_set);

static DEVICE_ATTR(low_temp_limit, 0444,
	low_temp_limit_get,
	NULL);

static DEVICE_ATTR(low_cap_limit, 0444,
	low_cap_limit_get,
	NULL);

static DEVICE_ATTR(low_cap_limit_dual, 0444,
	low_cap_limit_dual_get,
	NULL);

static int __init msm_led_trigger_sysfs_init(void)
{
	int ret = 0;

	pr_info("[CAM][FL] %s:%d\n", __func__, __LINE__);

	led_status_obj = kobject_create_and_add("camera_led_status", NULL);
	if (led_status_obj == NULL) {
		pr_info("[CAM][FL] msm_camera_led: subsystem_register failed\n");
		ret = -ENOMEM;
		goto error;
	}

	ret = sysfs_create_file(led_status_obj,
		&dev_attr_led_ril_status.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_led_ril_status failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_led_wimax_status.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_led_wimax_status failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_led_hotspot_status.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_led_hotspot_status failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_low_temp_limit.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_low_temp_limit failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_low_cap_limit.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_low_cap_limit failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_low_cap_limit_dual.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_low_cap_limit_dual failed\n");
		ret = -EFAULT;
		goto error;
	}

	pr_info("[CAM][FL] %s:%d ret %d\n", __func__, __LINE__, ret);
	return ret;

error:
	kobject_del(led_status_obj);
	return ret;

}
#endif 

static int __init msm_led_trigger_add_driver(void)
{
    #if 0 

	CDBG("called\n");
	return platform_driver_probe(&msm_led_trigger_driver,
		msm_led_trigger_probe);

    #else

	int32_t rc = 0;
	if (board_mfg_mode() == MFG_MODE_OFFMODE_CHARGING) {
		pr_err("offmode_charging, skip probe\n");
		return 0;
	}
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&msm_led_trigger_driver,
		msm_led_trigger_probe);
	if (!rc) {
		rc = msm_led_trigger_sysfs_init();
	}
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return rc;

    #endif 
}

static struct msm_flash_fn_t msm_led_trigger_func_tbl = {
	.flash_get_subdev_id = msm_led_trigger_get_subdev_id,
	.flash_led_config = msm_led_trigger_config,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.func_tbl = &msm_led_trigger_func_tbl,
};

module_init(msm_led_trigger_add_driver);
MODULE_DESCRIPTION("LED TRIGGER FLASH");
MODULE_LICENSE("GPL v2");
