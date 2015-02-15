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

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/qpnp/pwm.h>
#include <linux/err.h>

#include "mdss_dsi.h"
#include <mach/debug_display.h>
#include <linux/msm_mdp.h>

#define DT_CMD_HDR 6
#define WLED_MAX_LEVEL	4095

DEFINE_LED_TRIGGER(bl_led_trigger);
DEFINE_LED_TRIGGER(bl_led_i2c_trigger);

void mdss_dsi_panel_pwm_cfg(struct mdss_dsi_ctrl_pdata *ctrl)
{
	ctrl->pwm_bl = pwm_request(ctrl->pwm_lpg_chan, "lcd-bklt");
	if (ctrl->pwm_bl == NULL || IS_ERR(ctrl->pwm_bl)) {
		pr_err("%s: Error: lpg_chan=%d pwm request failed",
				__func__, ctrl->pwm_lpg_chan);
	}
}

static void mdss_dsi_panel_bklt_pwm(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	int ret;
	u32 duty;

	if (ctrl->pwm_bl == NULL) {
		pr_err("%s: no PWM\n", __func__);
		return;
	}

	if (level == 0) {
		if (ctrl->pwm_enabled)
			pwm_disable(ctrl->pwm_bl);
		ctrl->pwm_enabled = 0;
		return;
	}

	duty = level * ctrl->pwm_period;
	duty /= ctrl->bklt_max;

	pr_debug("%s: bklt_ctrl=%d pwm_period=%d pwm_gpio=%d pwm_lpg_chan=%d\n",
			__func__, ctrl->bklt_ctrl, ctrl->pwm_period,
				ctrl->pwm_pmic_gpio, ctrl->pwm_lpg_chan);

	pr_debug("%s: ndx=%d level=%d duty=%d\n", __func__,
					ctrl->ndx, level, duty);

	if (ctrl->pwm_enabled) {
		pwm_disable(ctrl->pwm_bl);
		ctrl->pwm_enabled = 0;
	}

	ret = pwm_config_us(ctrl->pwm_bl, duty, ctrl->pwm_period);
	if (ret) {
		pr_err("%s: pwm_config_us() failed err=%d.\n", __func__, ret);
		return;
	}

	ret = pwm_enable(ctrl->pwm_bl);
	if (ret)
		pr_err("%s: pwm_enable() failed err=%d\n", __func__, ret);
	ctrl->pwm_enabled = 1;
}

static char dcs_cmd[2] = {0x54, 0x00}; 
static struct dsi_cmd_desc dcs_read_cmd = {
	{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(dcs_cmd)},
	dcs_cmd
};

u32 mdss_dsi_panel_cmd_read(struct mdss_dsi_ctrl_pdata *ctrl, char cmd0,
		char cmd1, void (*fxn)(int), char *rbuf, int len)
{
	struct dcs_cmd_req cmdreq;

	dcs_cmd[0] = cmd0;
	dcs_cmd[1] = cmd1;
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dcs_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = len;
	cmdreq.rbuf = rbuf;
	cmdreq.cb = fxn; 
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	return 0;
}

static void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds)
{
	struct dcs_cmd_req cmdreq;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT;

	
	if (pcmds->link_state == DSI_LP_MODE)
		cmdreq.flags  |= CMD_REQ_LP_MODE;

	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

#define BRI_SETTING_MIN                 30
#define BRI_SETTING_DEF                 142
#define BRI_SETTING_HIGH		200
#define BRI_SETTING_EXTRA		230
#define BRI_SETTING_MAX                 255
static unsigned char shrink_pwm(int val, int pwm_min, int pwm_default, int pwm_max)
{
        unsigned char shrink_br = BRI_SETTING_MAX;

        if (val <= 0) {
                shrink_br = 0;
        } else if (val > 0 && (val < BRI_SETTING_MIN)) {
                shrink_br = pwm_min;
        } else if ((val >= BRI_SETTING_MIN) && (val <= BRI_SETTING_DEF)) {
                shrink_br = (val - BRI_SETTING_MIN) * (pwm_default - pwm_min) /
                (BRI_SETTING_DEF - BRI_SETTING_MIN) + pwm_min;
        } else if (val > BRI_SETTING_DEF && val <= BRI_SETTING_MAX) {
                shrink_br = (val - BRI_SETTING_DEF) * (pwm_max - pwm_default) /
                (BRI_SETTING_MAX - BRI_SETTING_DEF) + pwm_default;
        } else if (val > BRI_SETTING_MAX)
                shrink_br = pwm_max;

        PR_DISP_INFO("brightness orig=%d, transformed=%d\n", val, shrink_br);

        return shrink_br;
}
static unsigned char linear_pwm(int val, int max_brt, int bl_max)
{
	unsigned char bl_pwm  = BRI_SETTING_MAX;

	bl_pwm = val * bl_max / max_brt;

	PR_DISP_INFO("%s:brightness=%d, bl_pwm=%d\n", __func__,val, bl_pwm);
	return bl_pwm;
}

static unsigned int bl_to_brightness(int val, int brt_dim, int brt_min, int brt_def, int brt_high, int brt_extra, int brt_max)
{
	unsigned int  brt_val;

	if (val <= 0) {
		brt_val = 0;
	} else if (val > 0 && (val < BRI_SETTING_MIN)) {
		brt_val = brt_dim;
	} else if ((val >= BRI_SETTING_MIN) && (val <= BRI_SETTING_DEF)) {
		brt_val = (val - BRI_SETTING_MIN) * (brt_def - brt_min) /
		(BRI_SETTING_DEF - BRI_SETTING_MIN) + brt_min;
	} else if (val > BRI_SETTING_DEF && val <= BRI_SETTING_HIGH) {
		brt_val = (val - BRI_SETTING_DEF) * (brt_high - brt_def) /
		(BRI_SETTING_HIGH - BRI_SETTING_DEF) + brt_def;
	} else if (val > BRI_SETTING_HIGH && val <= BRI_SETTING_EXTRA) {
		brt_val = (val - BRI_SETTING_HIGH) * (brt_extra - brt_high) /
		(BRI_SETTING_EXTRA - BRI_SETTING_HIGH) + brt_high;
	} else if (val > BRI_SETTING_EXTRA && val <= BRI_SETTING_MAX) {
		brt_val = (val - BRI_SETTING_EXTRA) * (brt_max - brt_extra) /
		(BRI_SETTING_MAX - BRI_SETTING_EXTRA) + brt_extra;
	} else if (val > BRI_SETTING_MAX)
		brt_val = brt_max;

	PR_DISP_INFO("%s:level=%d, brightness=%d", __func__, val, brt_val);
	return brt_val;
}

static char led_pwm1[3] = {0x51, 0x0, 0x0};	
static struct dsi_cmd_desc backlight_cmd = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(led_pwm1)},
	led_pwm1
};

static void mdss_dsi_panel_bklt_dcs(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo = &(ctrl->panel_data.panel_info);

	pr_debug("%s: level=%d\n", __func__, level);

	if (!pinfo->act_brt)
		led_pwm1[1] = (unsigned char)shrink_pwm(level, ctrl->pwm_min, ctrl->pwm_default, ctrl->pwm_max);
	else
		led_pwm1[1] = (unsigned char)linear_pwm(level, pinfo->max_brt, pinfo->bl_max);

	led_pwm1[2] = led_pwm1[1];
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &backlight_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static int mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;

	if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		rc = gpio_request(ctrl_pdata->disp_en_gpio,
						"disp_enable");
		if (rc) {
			pr_err("request disp_en gpio failed, rc=%d\n",
				       rc);
			goto disp_en_gpio_err;
		}
	}
	rc = gpio_request(ctrl_pdata->rst_gpio, "disp_rst_n");
	if (rc) {
		pr_err("request reset gpio failed, rc=%d\n",
			rc);
		goto rst_gpio_err;
	}
	if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
		rc = gpio_request(ctrl_pdata->mode_gpio, "panel_mode");
		if (rc) {
			pr_err("request panel mode gpio failed,rc=%d\n",
								rc);
			goto mode_gpio_err;
		}
	}
	return rc;

mode_gpio_err:
	gpio_free(ctrl_pdata->rst_gpio);
rst_gpio_err:
	if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
		gpio_free(ctrl_pdata->disp_en_gpio);
disp_en_gpio_err:
	return rc;
}

int mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int i, rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}

	pr_debug("%s: enable = %d\n", __func__, enable);
	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (enable) {
		rc = mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}
		if (!pinfo->panel_power_on) {
			if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
				gpio_set_value((ctrl_pdata->disp_en_gpio), 1);

			for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
				gpio_set_value((ctrl_pdata->rst_gpio),
					pdata->panel_info.rst_seq[i]);
				if (pdata->panel_info.rst_seq[++i])
					usleep(pinfo->rst_seq[i] * 1000);
			}

		}

		if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
			if (pinfo->mode_gpio_state == MODE_GPIO_HIGH)
				gpio_set_value((ctrl_pdata->mode_gpio), 1);
			else if (pinfo->mode_gpio_state == MODE_GPIO_LOW)
				gpio_set_value((ctrl_pdata->mode_gpio), 0);
		}
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
			gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
			gpio_free(ctrl_pdata->disp_en_gpio);
		}
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
		gpio_free(ctrl_pdata->rst_gpio);
		if (gpio_is_valid(ctrl_pdata->mode_gpio))
			gpio_free(ctrl_pdata->mode_gpio);
	}
	return rc;
}

static char caset[] = {0x2a, 0x00, 0x00, 0x03, 0x00};	
static char paset[] = {0x2b, 0x00, 0x00, 0x05, 0x00};	

static struct dsi_cmd_desc partial_update_enable_cmd[] = {
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(caset)}, caset},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(paset)}, paset},
};

static int mdss_dsi_panel_partial_update(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct dcs_cmd_req cmdreq;
	int rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	mipi  = &pdata->panel_info.mipi;

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	caset[1] = (((pdata->panel_info.roi_x) & 0xFF00) >> 8);
	caset[2] = (((pdata->panel_info.roi_x) & 0xFF));
	caset[3] = (((pdata->panel_info.roi_x - 1 + pdata->panel_info.roi_w)
								& 0xFF00) >> 8);
	caset[4] = (((pdata->panel_info.roi_x - 1 + pdata->panel_info.roi_w)
								& 0xFF));
	partial_update_enable_cmd[0].payload = caset;

	paset[1] = (((pdata->panel_info.roi_y) & 0xFF00) >> 8);
	paset[2] = (((pdata->panel_info.roi_y) & 0xFF));
	paset[3] = (((pdata->panel_info.roi_y - 1 + pdata->panel_info.roi_h)
								& 0xFF00) >> 8);
	paset[4] = (((pdata->panel_info.roi_y - 1 + pdata->panel_info.roi_h)
								& 0xFF));
	partial_update_enable_cmd[1].payload = paset;

	pr_debug("%s: enabling partial update\n", __func__);
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = partial_update_enable_cmd;
	cmdreq.cmds_cnt = 2;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	return rc;
}

extern struct mdss_dsi_pwrctrl pwrctrl_pdata;
static struct mdss_dsi_ctrl_pdata *get_rctrl_data(struct mdss_panel_data *pdata)
{
	if (!pdata || !pdata->next) {
		pr_err("%s: Invalid panel data\n", __func__);
		return NULL;
	}

	return container_of(pdata->next, struct mdss_dsi_ctrl_pdata,
			panel_data);
}

static void mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata,
							u32 bl_level)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

#ifdef CONFIG_HTC_POWER_HACK
	
	if (bl_level == 0) {
		if (ctrl_pdata->cabc_ui_cmds.cmds) {
			struct dcs_cmd_req cmdreq;
			cmdreq.cmds = ctrl_pdata->cabc_ui_cmds.cmds;
			cmdreq.cmds_cnt = ctrl_pdata->cabc_ui_cmds.cmd_cnt;
			cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
			cmdreq.rlen = 0;
			cmdreq.cb = NULL;

			mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);
		}
	}
#endif
	if(pwrctrl_pdata.bkl_config)
		pwrctrl_pdata.bkl_config(&ctrl_pdata->panel_data, ((bl_level == 0) ? 0 : 1));

	switch (ctrl_pdata->bklt_ctrl) {
	case BL_WLED:
		led_trigger_event(bl_led_trigger, bl_level);
		break;
	case BL_PWM:
		mdss_dsi_panel_bklt_pwm(ctrl_pdata, bl_level);
		break;
	case BL_DCS_CMD:
		mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);
		if (ctrl_pdata->shared_pdata.broadcast_enable &&
				ctrl_pdata->ndx == DSI_CTRL_0) {
			struct mdss_dsi_ctrl_pdata *rctrl_pdata = NULL;
			rctrl_pdata = get_rctrl_data(pdata);
			if (!rctrl_pdata) {
				pr_err("%s: Right ctrl data NULL\n", __func__);
				return;
			}
			mdss_dsi_panel_bklt_dcs(rctrl_pdata, bl_level);
		}
		break;
	case BL_I2C:
		if (!ctrl_pdata->panel_data.panel_info.act_brt)
			bl_level = bl_to_brightness(bl_level, ctrl_pdata->brt_dim,
								ctrl_pdata->brt_min,
								ctrl_pdata->brt_def,
								ctrl_pdata->brt_high,
								ctrl_pdata->brt_extra,
								ctrl_pdata->brt_max);
		led_trigger_event(bl_led_i2c_trigger, bl_level);
		break;

	default:
		pr_err("%s: Unknown bl_ctrl configuration\n",
			__func__);
		break;
	}
}

static void mdss_dsi_display_on(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);
	if (ctrl->display_on_wait)
		msleep(ctrl->display_on_wait);

	PR_DISP_INFO("%s: pid=%d\n", __func__, current->pid);
	mdss_dsi_panel_cmds_send(ctrl, &ctrl->display_on_cmds);
}

static int mdss_dsi_panel_on(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);
	mipi  = &pdata->panel_info.mipi;

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	if (pdata->panel_info.first_power_on == 1) {
		pdata->panel_info.first_power_on = 0;
		pr_info("panel on already\n");
	} else if (ctrl->on_cmds.cmd_cnt) {
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->on_cmds);
	}

	if (ctrl->pwm_ctl_type == PWM_PMIC)
		led_trigger_event(bl_led_trigger, WLED_MAX_LEVEL);
	else if (ctrl->pwm_ctl_type == PWM_EXT)
		mdss_dsi_panel_bklt_dcs(ctrl, pdata->panel_info.max_brt);

	PR_DISP_INFO("%s:-\n", __func__);
	return 0;
}

static int mdss_dsi_panel_off(struct mdss_panel_data *pdata)
{
	struct mipi_panel_info *mipi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	mipi  = &pdata->panel_info.mipi;

	if(pwrctrl_pdata.bkl_config)
		pwrctrl_pdata.bkl_config(&ctrl->panel_data, 0);

	if (ctrl->off_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->off_cmds);

	if (ctrl->pwm_ctl_type == PWM_PMIC)
		led_trigger_event(bl_led_trigger, 0);
	else if (ctrl->pwm_ctl_type == PWM_EXT)
		led_trigger_event(bl_led_i2c_trigger, 0); 

	PR_DISP_INFO("%s:-\n", __func__);
	return 0;
}

static void mdss_dsi_parse_lane_swap(struct device_node *np, char *dlane_swap)
{
	const char *data;

	*dlane_swap = DSI_LANE_MAP_0123;
	data = of_get_property(np, "qcom,mdss-dsi-lane-map", NULL);
	if (data) {
		if (!strcmp(data, "lane_map_3012"))
			*dlane_swap = DSI_LANE_MAP_3012;
		else if (!strcmp(data, "lane_map_2301"))
			*dlane_swap = DSI_LANE_MAP_2301;
		else if (!strcmp(data, "lane_map_1230"))
			*dlane_swap = DSI_LANE_MAP_1230;
		else if (!strcmp(data, "lane_map_0321"))
			*dlane_swap = DSI_LANE_MAP_0321;
		else if (!strcmp(data, "lane_map_1032"))
			*dlane_swap = DSI_LANE_MAP_1032;
		else if (!strcmp(data, "lane_map_2103"))
			*dlane_swap = DSI_LANE_MAP_2103;
		else if (!strcmp(data, "lane_map_3210"))
			*dlane_swap = DSI_LANE_MAP_3210;
	}
}

static void mdss_dsi_parse_trigger(struct device_node *np, char *trigger,
		char *trigger_key)
{
	const char *data;

	*trigger = DSI_CMD_TRIGGER_SW;
	data = of_get_property(np, trigger_key, NULL);
	if (data) {
		if (!strcmp(data, "none"))
			*trigger = DSI_CMD_TRIGGER_NONE;
		else if (!strcmp(data, "trigger_te"))
			*trigger = DSI_CMD_TRIGGER_TE;
		else if (!strcmp(data, "trigger_sw_seof"))
			*trigger = DSI_CMD_TRIGGER_SW_SEOF;
		else if (!strcmp(data, "trigger_sw_te"))
			*trigger = DSI_CMD_TRIGGER_SW_TE;
	}
}


static int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key)
{
	const char *data;
	int blen = 0, len;
	char *buf, *bp;
	struct dsi_ctrl_hdr *dchdr;
	int i, cnt;

	data = of_get_property(np, cmd_key, &blen);
	if (!data) {
		pr_err("%s: failed, key=%s\n", __func__, cmd_key);
		return -ENOMEM;
	}

	buf = kzalloc(sizeof(char) * blen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, data, blen);

	
	bp = buf;
	len = blen;
	cnt = 0;
	while (len > sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen > len) {
			pr_err("%s: dtsi cmd=%x error, len=%d",
				__func__, dchdr->dtype, dchdr->dlen);
			goto exit_free;
		}
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		cnt++;
	}

	if (len != 0) {
		pr_err("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		goto exit_free;
	}

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc),
						GFP_KERNEL);
	if (!pcmds->cmds)
		goto exit_free;

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < cnt; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		pcmds->cmds[i].dchdr = *dchdr;
		pcmds->cmds[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
	}

	data = of_get_property(np, link_key, NULL);
	if (data && !strcmp(data, "dsi_hs_mode"))
		pcmds->link_state = DSI_HS_MODE;
	else
		pcmds->link_state = DSI_LP_MODE;

	pr_debug("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt, pcmds->link_state);

	return 0;

exit_free:
	kfree(buf);
	return -ENOMEM;
}


static int mdss_panel_dt_get_dst_fmt(u32 bpp, char mipi_mode, u32 pixel_packing,
				char *dst_format)
{
	int rc = 0;
	switch (bpp) {
	case 3:
		*dst_format = DSI_CMD_DST_FORMAT_RGB111;
		break;
	case 8:
		*dst_format = DSI_CMD_DST_FORMAT_RGB332;
		break;
	case 12:
		*dst_format = DSI_CMD_DST_FORMAT_RGB444;
		break;
	case 16:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB565;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		}
		break;
	case 18:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB666;
			break;
		default:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		}
		break;
	case 24:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB888;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}


static int mdss_dsi_parse_fbc_params(struct device_node *np,
				struct mdss_panel_info *panel_info)
{
	int rc, fbc_enabled = 0;
	u32 tmp;

	fbc_enabled = of_property_read_bool(np,	"qcom,mdss-dsi-fbc-enable");
	if (fbc_enabled) {
		pr_debug("%s:%d FBC panel enabled.\n", __func__, __LINE__);
		panel_info->fbc.enabled = 1;
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bpp", &tmp);
		panel_info->fbc.target_bpp =	(!rc ? tmp : panel_info->bpp);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-packing",
				&tmp);
		panel_info->fbc.comp_mode = (!rc ? tmp : 0);
		panel_info->fbc.qerr_enable = of_property_read_bool(np,
			"qcom,mdss-dsi-fbc-quant-error");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bias", &tmp);
		panel_info->fbc.cd_bias = (!rc ? tmp : 0);
		panel_info->fbc.pat_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-pat-mode");
		panel_info->fbc.vlc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-vlc-mode");
		panel_info->fbc.bflc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-bflc-mode");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-h-line-budget",
				&tmp);
		panel_info->fbc.line_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-budget-ctrl",
				&tmp);
		panel_info->fbc.block_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-block-budget",
				&tmp);
		panel_info->fbc.block_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossless-threshold", &tmp);
		panel_info->fbc.lossless_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-threshold", &tmp);
		panel_info->fbc.lossy_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-rgb-threshold",
				&tmp);
		panel_info->fbc.lossy_rgb_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-mode-idx", &tmp);
		panel_info->fbc.lossy_mode_idx = (!rc ? tmp : 0);
	} else {
		pr_debug("%s:%d Panel does not support FBC.\n",
				__func__, __LINE__);
		panel_info->fbc.enabled = 0;
		panel_info->fbc.target_bpp =
			panel_info->bpp;
	}
	return 0;
}


static int mdss_dsi_parse_reset_seq(struct device_node *np,
		u32 rst_seq[MDSS_DSI_RST_SEQ_LEN], u32 *rst_len,
		const char *name)
{
	int num = 0, i;
	int rc;
	struct property *data;
	u32 tmp[MDSS_DSI_RST_SEQ_LEN];
	*rst_len = 0;
	data = of_find_property(np, name, &num);
	num /= sizeof(u32);
	if (!data || !num || num > MDSS_DSI_RST_SEQ_LEN || num % 2) {
		pr_debug("%s:%d, error reading %s, length found = %d\n",
			__func__, __LINE__, name, num);
	} else {
		rc = of_property_read_u32_array(np, name, tmp, num);
		if (rc)
			pr_debug("%s:%d, error reading %s, rc = %d\n",
				__func__, __LINE__, name, rc);
		else {
			for (i = 0; i < num; ++i)
				rst_seq[i] = tmp[i];
			*rst_len = num;
		}
	}
	return 0;
}

extern int htc_mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);

static int mdss_panel_parse_dt(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	u32 tmp, res[6];
	int rc, i, len;
	const char *data;
	static const char *pdest;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-width", &tmp);
	if (rc) {
		pr_err("%s:%d, panel width not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	pinfo->xres = (!rc ? tmp : 640);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-height", &tmp);
	if (rc) {
		pr_err("%s:%d, panel height not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	pinfo->yres = (!rc ? tmp : 480);

	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-width-dimension", &tmp);
	pinfo->physical_width = (!rc ? tmp : 0);
	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-height-dimension", &tmp);
	pinfo->physical_height = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-left-border", &tmp);
	pinfo->lcdc.xres_pad = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-right-border", &tmp);
	if (!rc)
		pinfo->lcdc.xres_pad += tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-top-border", &tmp);
	pinfo->lcdc.yres_pad = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-bottom-border", &tmp);
	if (!rc)
		pinfo->lcdc.yres_pad += tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bpp", &tmp);
	if (rc) {
		pr_err("%s:%d, bpp not specified\n", __func__, __LINE__);
		return -EINVAL;
	}
	pinfo->bpp = (!rc ? tmp : 24);
	pinfo->mipi.mode = DSI_VIDEO_MODE;
	data = of_get_property(np, "qcom,mdss-dsi-panel-type", NULL);
	if (data && !strncmp(data, "dsi_cmd_mode", 12))
		pinfo->mipi.mode = DSI_CMD_MODE;
	tmp = 0;
	data = of_get_property(np, "qcom,mdss-dsi-pixel-packing", NULL);
	if (data && !strcmp(data, "loose"))
		tmp = 1;
	rc = mdss_panel_dt_get_dst_fmt(pinfo->bpp,
		pinfo->mipi.mode, tmp,
		&(pinfo->mipi.dst_format));
	if (rc) {
		pr_debug("%s: problem determining dst format. Set Default\n",
			__func__);
		pinfo->mipi.dst_format =
			DSI_VIDEO_DST_FORMAT_RGB888;
	}
	pdest = of_get_property(np,
		"qcom,mdss-dsi-panel-destination", NULL);

	if (pdest) {
		if (strlen(pdest) != 9) {
			pr_err("%s: Unknown pdest specified\n", __func__);
			return -EINVAL;
		}
		if (!strcmp(pdest, "display_1"))
			pinfo->pdest = DISPLAY_1;
		else if (!strcmp(pdest, "display_2"))
			pinfo->pdest = DISPLAY_2;
		else {
			pr_debug("%s: incorrect pdest. Set Default\n",
				__func__);
			pinfo->pdest = DISPLAY_1;
		}
	} else {
		pr_debug("%s: pdest not specified. Set Default\n",
				__func__);
		pinfo->pdest = DISPLAY_1;
	}
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-front-porch", &tmp);
	pinfo->lcdc.h_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-back-porch", &tmp);
	pinfo->lcdc.h_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-pulse-width", &tmp);
	pinfo->lcdc.h_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-sync-skew", &tmp);
	pinfo->lcdc.hsync_skew = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-back-porch", &tmp);
	pinfo->lcdc.v_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-front-porch", &tmp);
	pinfo->lcdc.v_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-pulse-width", &tmp);
	pinfo->lcdc.v_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-underflow-color", &tmp);
	pinfo->lcdc.underflow_clr = (!rc ? tmp : 0xff);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-border-color", &tmp);
	pinfo->lcdc.border_clr = (!rc ? tmp : 0);
	pinfo->bklt_ctrl = UNKNOWN_CTRL;
	data = of_get_property(np, "qcom,mdss-dsi-bl-pmic-control-type", NULL);
	if (data) {
		if (!strncmp(data, "bl_ctrl_wled", 12)) {
			led_trigger_register_simple("bkl-trigger",
				&bl_led_trigger);
			pr_debug("%s: SUCCESS-> WLED TRIGGER register\n",
				__func__);
			ctrl_pdata->bklt_ctrl = BL_WLED;
		} else if (!strncmp(data, "bl_ctrl_pwm", 11)) {
			ctrl_pdata->bklt_ctrl = BL_PWM;
			rc = of_property_read_u32(np,
				"qcom,mdss-dsi-bl-pmic-pwm-frequency", &tmp);
			if (rc) {
				pr_err("%s:%d, Error, panel pwm_period\n",
						__func__, __LINE__);
				return -EINVAL;
			}
			ctrl_pdata->pwm_period = tmp;
			rc = of_property_read_u32(np,
				"qcom,mdss-dsi-bl-pmic-bank-select", &tmp);
			if (rc) {
				pr_err("%s:%d, Error, dsi lpg channel\n",
						__func__, __LINE__);
				return -EINVAL;
			}
			ctrl_pdata->pwm_lpg_chan = tmp;
			tmp = of_get_named_gpio(np,
				"qcom,mdss-dsi-pwm-gpio", 0);
			ctrl_pdata->pwm_pmic_gpio = tmp;
		} else if (!strncmp(data, "bl_ctrl_dcs", 11)) {
			ctrl_pdata->bklt_ctrl = BL_DCS_CMD;
		} else if (!strncmp(data, "bl_ctrl_i2c", 11)) {
			led_trigger_register_simple("bl-led-i2c-trigger", &bl_led_i2c_trigger);
			ctrl_pdata->bklt_ctrl = BL_I2C;
		}
	}
	rc = of_property_read_u32(np, "qcom,mdss-brightness-max-level", &tmp);
	pinfo->brightness_max = (!rc ? tmp : MDSS_MAX_BL_BRIGHTNESS);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-min-level", &tmp);
	pinfo->bl_min = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-max-level", &tmp);
	pinfo->bl_max = (!rc ? tmp : 255);
	ctrl_pdata->bklt_max = pinfo->bl_max;

	rc = of_property_read_u32(np, "qcom,mdss-dsi-interleave-mode", &tmp);
	pinfo->mipi.interleave_mode = (!rc ? tmp : 0);

	pinfo->mipi.vsync_enable = of_property_read_bool(np,
		"qcom,mdss-dsi-te-check-enable");
	pinfo->mipi.hw_vsync_mode = of_property_read_bool(np,
		"qcom,mdss-dsi-te-using-te-pin");

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-h-sync-pulse", &tmp);
	pinfo->mipi.pulse_mode_hsa_he = (!rc ? tmp : false);

	pinfo->mipi.hfp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hfp-power-mode");
	pinfo->mipi.hsa_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hsa-power-mode");
	pinfo->mipi.hbp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hbp-power-mode");
	pinfo->mipi.bllp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-bllp-power-mode");
	pinfo->mipi.eof_bllp_power_stop = of_property_read_bool(
		np, "qcom,mdss-dsi-bllp-eof-power-mode");
	pinfo->mipi.traffic_mode = DSI_NON_BURST_SYNCH_PULSE;
	data = of_get_property(np, "qcom,mdss-dsi-traffic-mode", NULL);
	if (data) {
		if (!strcmp(data, "non_burst_sync_event"))
			pinfo->mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;
		else if (!strcmp(data, "burst_mode"))
			pinfo->mipi.traffic_mode = DSI_BURST_MODE;
	}
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-dcs-command", &tmp);
	pinfo->mipi.insert_dcs_cmd =
			(!rc ? tmp : 1);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-wr-mem-continue", &tmp);
	pinfo->mipi.wr_mem_continue =
			(!rc ? tmp : 0x3c);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-wr-mem-start", &tmp);
	pinfo->mipi.wr_mem_start =
			(!rc ? tmp : 0x2c);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-pin-select", &tmp);
	pinfo->mipi.te_sel =
			(!rc ? tmp : 1);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-virtual-channel-id", &tmp);
	pinfo->mipi.vc = (!rc ? tmp : 0);
	pinfo->mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	data = of_get_property(np, "qcom,mdss-dsi-color-order", NULL);
	if (data) {
		if (!strcmp(data, "rgb_swap_rbg"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_RBG;
		else if (!strcmp(data, "rgb_swap_bgr"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_BGR;
		else if (!strcmp(data, "rgb_swap_brg"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_BRG;
		else if (!strcmp(data, "rgb_swap_grb"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_GRB;
		else if (!strcmp(data, "rgb_swap_gbr"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_GBR;
	}
	pinfo->mipi.data_lane0 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-0-state");
	pinfo->mipi.data_lane1 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-1-state");
	pinfo->mipi.data_lane2 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-2-state");
	pinfo->mipi.data_lane3 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-3-state");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-pre", &tmp);
	pinfo->mipi.t_clk_pre = (!rc ? tmp : 0x24);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-post", &tmp);
	pinfo->mipi.t_clk_post = (!rc ? tmp : 0x03);

	pinfo->mipi.rx_eot_ignore = of_property_read_bool(np,
		"qcom,mdss-dsi-rx-eot-ignore");
	pinfo->mipi.tx_eot_append = of_property_read_bool(np,
		"qcom,mdss-dsi-tx-eot-append");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-stream", &tmp);
	pinfo->mipi.stream = (!rc ? tmp : 0);

	data = of_get_property(np, "qcom,mdss-dsi-panel-mode-gpio-state", NULL);
	if (data) {
		if (!strcmp(data, "high"))
			pinfo->mode_gpio_state = MODE_GPIO_HIGH;
		else if (!strcmp(data, "low"))
			pinfo->mode_gpio_state = MODE_GPIO_LOW;
	} else {
		pinfo->mode_gpio_state = MODE_GPIO_NOT_VALID;
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-framerate", &tmp);
	pinfo->mipi.frame_rate = (!rc ? tmp : 60);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-clockrate", &tmp);
	pinfo->clk_rate = (!rc ? tmp : 0);
	data = of_get_property(np, "qcom,mdss-dsi-panel-timings", &len);
	if ((!data) || (len != 12)) {
		pr_err("%s:%d, Unable to read Phy timing settings",
		       __func__, __LINE__);
		goto error;
	}
	for (i = 0; i < len; i++)
		pinfo->mipi.dsi_phy_db.timing[i] = data[i];

	pinfo->mipi.lp11_init = of_property_read_bool(np,
		"qcom,mdss-dsi-lp11-init");
	rc = of_property_read_u32(np, "qcom,mdss-dsi-init-delay-us", &tmp);
	pinfo->mipi.init_delay = (!rc ? tmp : 0);

	mdss_dsi_parse_fbc_params(np, pinfo);

	mdss_dsi_parse_trigger(np, &(pinfo->mipi.mdp_trigger),
		"qcom,mdss-dsi-mdp-trigger");

	mdss_dsi_parse_trigger(np, &(pinfo->mipi.dma_trigger),
		"qcom,mdss-dsi-dma-trigger");

	mdss_dsi_parse_lane_swap(np, &(pinfo->mipi.dlane_swap));

	mdss_dsi_parse_reset_seq(np, pinfo->rst_seq, &(pinfo->rst_seq_len),
		"qcom,mdss-dsi-reset-sequence");

	htc_mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->on_cmds,
		"htc-fmt,mdss-dsi-on-command", "qcom,mdss-dsi-on-command-state");

	htc_mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->off_cmds,
		"htc-fmt,mdss-dsi-off-command", "qcom,mdss-dsi-off-command-state");

	htc_mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->display_on_cmds,
		"htc-fmt,display-on-cmds", "qcom,mdss-dsi-default-command-state");

	if(!ctrl_pdata->on_cmds.cmd_cnt || !ctrl_pdata->on_cmds.blen)
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->on_cmds,
			"qcom,mdss-dsi-on-command", "qcom,mdss-dsi-on-command-state");

	if(!ctrl_pdata->off_cmds.cmd_cnt || !ctrl_pdata->off_cmds.blen)
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->off_cmds,
			"qcom,mdss-dsi-off-command", "qcom,mdss-dsi-off-command-state");

	if(!ctrl_pdata->display_on_cmds.cmd_cnt || !ctrl_pdata->display_on_cmds.blen)
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->display_on_cmds,
			"qcom,display-on-cmds", "qcom,mdss-dsi-default-command-state"); 

	rc = of_property_read_u32(np, "qcom,display-on-wait", &tmp);
	ctrl_pdata->display_on_wait = (!rc ? tmp : 0);

	rc = of_property_read_u32_array(np, "qcom,mdss-shrink-pwm", res, 3);
	if (rc) {
		pr_err("%s:%d, panel dimension not specified\n",
						 __func__, __LINE__);
	}
	ctrl_pdata->pwm_min  = (!rc ? res[0] : BRI_SETTING_MIN);
	ctrl_pdata->pwm_default = (!rc ? res[1] : BRI_SETTING_DEF);
	ctrl_pdata->pwm_max = (!rc ? res[2] : BRI_SETTING_MAX);

	rc = of_property_read_u32(np, "qcom,mdss-pwm-ctl-type", &tmp);
	ctrl_pdata->pwm_ctl_type = (!rc ? tmp : 0);
	if(ctrl_pdata->pwm_ctl_type == PWM_PMIC) {
		led_trigger_register_simple("bkl-trigger", &bl_led_trigger);
		PR_DISP_INFO("%s: PWM type is PMIC\n", __func__);
	}

	rc = of_property_read_u32(np, "htc,mdss-camera-blk", &tmp);
	pinfo->camera_blk = (!rc ? tmp : BRI_SETTING_DEF);

	rc = of_property_read_u32(np, "htc,mdss-camera-dua-blk", &tmp);
	pinfo->camera_dua_blk = (!rc ? tmp : pinfo->camera_blk);

	htc_mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->cabc_off_cmds,
		"htc-fmt,cabc-off-cmds", "qcom,mdss-dsi-default-command-state");

	htc_mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->cabc_ui_cmds,
		"htc-fmt,cabc-ui-cmds", "qcom,mdss-dsi-default-command-state");

	htc_mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->cabc_video_cmds,
		"htc-fmt,cabc-video-cmds", "qcom,mdss-dsi-default-command-state");

	htc_mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->dimming_on_cmds,
		"htc-fmt,dimming-on-cmds", "qcom,mdss-dsi-default-command-state");

	if(!ctrl_pdata->cabc_off_cmds.cmd_cnt || !ctrl_pdata->cabc_off_cmds.blen)
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->cabc_off_cmds,
			"htc,cabc-off-cmds", "qcom,mdss-dsi-default-command-state");

	if(!ctrl_pdata->cabc_ui_cmds.cmd_cnt || !ctrl_pdata->cabc_ui_cmds.blen)
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->cabc_ui_cmds,
			"htc,cabc-ui-cmds", "qcom,mdss-dsi-default-command-state");

	if(!ctrl_pdata->cabc_video_cmds.cmd_cnt || !ctrl_pdata->cabc_video_cmds.blen)
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->cabc_video_cmds,
			"htc,cabc-video-cmds", "qcom,mdss-dsi-default-command-state");

	if(!ctrl_pdata->dimming_on_cmds.cmd_cnt || !ctrl_pdata->dimming_on_cmds.blen)
		mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->dimming_on_cmds,
			"htc,dimming-on-cmds", "qcom,mdss-dsi-default-command-state");

	rc = of_property_read_u32(np, "htc,panel-id", &tmp);
	pinfo->panel_id = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "htc,mdss-pp-hue", &tmp);
	pinfo->mdss_pp_hue = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "htc,mdp-pcc-r", &tmp);
	pinfo->pcc_r = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "htc,mdp-pcc-g", &tmp);
	pinfo->pcc_g = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "htc,mdp-pcc-b", &tmp);
	pinfo->pcc_b = (!rc ? tmp : 0);

	pinfo->skip_frame = of_property_read_bool(np, "htc,skip-frame");

	rc = of_property_read_u32(np, "htc,mdss-max-brt-level", &tmp);
	pinfo->act_max_brt = (!rc ? tmp : MDSS_MAX_BL_BRIGHTNESS);
	pinfo->max_brt = MDSS_MAX_BL_BRIGHTNESS;

	rc = of_property_read_u32_array(np, "htc,mdss-bl-brt", res, 6);
	if (rc) {
		pr_err("%s:%d, mdss-bl-brt not specified\n",
						 __func__, __LINE__);
	}
	ctrl_pdata->brt_dim  = (!rc ? res[0] : BRI_SETTING_MIN);
	ctrl_pdata->brt_min  = (!rc ? res[1] : BRI_SETTING_MIN);
	ctrl_pdata->brt_def = (!rc ? res[2] : BRI_SETTING_DEF);
	ctrl_pdata->brt_high = (!rc ? res[3] : BRI_SETTING_HIGH);
	ctrl_pdata->brt_extra = (!rc ? res[4] : BRI_SETTING_EXTRA);
	ctrl_pdata->brt_max = (!rc ? res[5] : BRI_SETTING_MAX);


#ifdef CONFIG_HTC_POWER_HACK
	rc = of_property_read_u32_array(np, "qcom,mdss-shrink-pwm-power-hack", res, 3);
	if (rc) {
		pr_err("%s:%d, qcom,mdss-shrink-pwm-cmcc not specified\n",
						 __func__, __LINE__);
	} else {
		ctrl_pdata->pwm_min  = res[0];
		ctrl_pdata->pwm_default = res[1];
		ctrl_pdata->pwm_max = res[2];
	}

	rc = of_property_read_u32(np, "htc,mdss-camera-blk-power-hack", &tmp);
	if (rc) {
		pr_err("%s:%d, htc,mdss-camera-blk-cmcc not specified\n",
						__func__, __LINE__);
	} else {
		pinfo->camera_blk = tmp;
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-framerate-power-hack", &tmp);
	if (rc) {
		pr_err("%s:%d, qcom,mdss-dsi-panel-framerate-power-hack not specified\n",
						__func__, __LINE__);
	} else {
		if (tmp)
			pinfo->mipi.frame_rate = tmp;
	}

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->cabc_ui_cmds,
		"htc,cabc-ui-cmds-power-hack", "qcom,mdss-dsi-default-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->cabc_video_cmds,
		"htc,cabc-video-cmds-power-hack", "qcom,mdss-dsi-default-command-state");
#endif

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->frame_suffix_cmds,
		"htc,frame-suffix-cmds", "qcom,mdss-dsi-default-command-state");

	return 0;

error:
	return -EINVAL;
}

int mdss_dsi_panel_init(struct device_node *node,
	struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	bool cmd_cfg_cont_splash)
{
	int rc = 0;
	static const char *panel_name;
	bool cont_splash_enabled;
	bool partial_update_enabled;
	bool even_roi = 0;
#ifdef CONFIG_HTC_POWER_HACK
	u32 tmp;
#endif

	if (!node) {
		pr_err("%s: no panel node\n", __func__);
		return -ENODEV;
	}

	pr_debug("%s:%d\n", __func__, __LINE__);
	panel_name = of_get_property(node, "qcom,mdss-dsi-panel-name", NULL);
	if (!panel_name)
		pr_info("%s:%d, Panel name not specified\n",
						__func__, __LINE__);
	else
		pr_info("%s: Panel Name = %s\n", __func__, panel_name);

	rc = mdss_panel_parse_dt(node, ctrl_pdata);
	if (rc) {
		pr_err("%s:%d panel dt parse failed\n", __func__, __LINE__);
		return rc;
	}

	if (cmd_cfg_cont_splash) {
		cont_splash_enabled = of_property_read_bool(node,
				"qcom,cont-splash-enabled");
		ctrl_pdata->panel_data.panel_info.first_power_on = !cont_splash_enabled;
	} else
		cont_splash_enabled = false;

#ifdef CONFIG_HTC_POWER_HACK
	
	rc = of_property_read_u32(node, "qcom,cont-splash-power-hack", &tmp);
	if (rc) {
		pr_err("%s:%d, qcom,cont-splash-power-hack not specified\n",
						__func__, __LINE__);
	} else {
		cont_splash_enabled = tmp ? true : false;
		ctrl_pdata->panel_data.panel_info.first_power_on = !cont_splash_enabled;
	}
#endif

	if (!cont_splash_enabled) {
		pr_info("%s:%d Continuous splash flag not found.\n",
				__func__, __LINE__);
		ctrl_pdata->panel_data.panel_info.cont_splash_enabled = 0;
	} else {
		pr_info("%s:%d Continuous splash flag enabled.\n",
				__func__, __LINE__);

		ctrl_pdata->panel_data.panel_info.cont_splash_enabled = 1;
	}

	partial_update_enabled = of_property_read_bool(node,
						"qcom,partial-update-enabled");
	if (partial_update_enabled) {
		pr_info("%s:%d Partial update enabled.\n", __func__, __LINE__);
		ctrl_pdata->panel_data.panel_info.partial_update_enabled = 1;
		ctrl_pdata->partial_update_fnc = mdss_dsi_panel_partial_update;
	} else {
		pr_info("%s:%d Partial update disabled.\n", __func__, __LINE__);
		ctrl_pdata->panel_data.panel_info.partial_update_enabled = 0;
		ctrl_pdata->partial_update_fnc = NULL;
	}

	even_roi = of_property_read_bool(node,
						"qcom,mdss-even-coordinate-update");
	if (even_roi) {
		pr_info("%s:%d Even Coordinate Partial update enabled.\n", __func__, __LINE__);
		ctrl_pdata->panel_data.panel_info.even_roi = 1;
	} else {
		ctrl_pdata->panel_data.panel_info.even_roi = 0;
	}

	ctrl_pdata->on = mdss_dsi_panel_on;
	ctrl_pdata->off = mdss_dsi_panel_off;
	ctrl_pdata->panel_data.set_backlight = mdss_dsi_panel_bl_ctrl;
	ctrl_pdata->panel_data.display_on = mdss_dsi_display_on;

	return 0;
}
