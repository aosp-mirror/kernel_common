/*
 * Linux cfg80211 driver - Dongle Host Driver (DHD) related
 *
 * Copyright (C) 1999-2014, Broadcom Corporation
 * 
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 * 
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 * 
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 * $Id: wl_cfg_btcoex.c 467328 2014-04-03 01:23:40Z $
 */

#include <net/rtnetlink.h>

#include <bcmutils.h>
#include <wldev_common.h>
#include <wl_cfg80211.h>
#include <dhd_cfg80211.h>
#include <dngl_stats.h>
#include <dhd.h>
#include <dhdioctl.h>
#include <wlioctl.h>

/* TODO: clean up the BT-Coex code, it still have some legacy ioctl/iovar functions */

/* use New SCO/eSCO smart YG suppression */
#define BT_DHCP_eSCO_FIX
/* this flag boost wifi pkt priority to max, caution: -not fair to sco */
#define BT_DHCP_USE_FLAGS
/* T1 start SCO/ESCo priority suppression */
#define BT_DHCP_OPPR_WIN_TIME		2500
/* T2 turn off SCO/SCO supperesion is (timeout) */
#define BT_DHCP_FLAG_FORCE_TIME 5500

enum wl_cfg80211_btcoex_status {
	BT_DHCP_IDLE,
	BT_DHCP_START,
	BT_DHCP_OPPR_WIN,
	BT_DHCP_FLAG_FORCE_TIMEOUT
};

struct btcoex_info {
	enum wl_cfg80211_btcoex_status bt_state;
	struct delayed_work work;
	struct net_device *dev;

	u32 saved_reg66;
	u32 saved_reg41;
	u32 saved_reg68;
	bool dhcp_saved_status;

#ifdef BT_DHCP_eSCO_FIX
	u32 saved_reg50;
	u32 saved_reg51;
	u32 saved_reg64;
	u32 saved_reg65;
	u32 saved_reg71;
	bool esco_saved_status;
#endif
};

/*
 * get named driver variable to uint register value and return error indication
 * calling example: dev_wlc_intvar_get_reg(dev, "btc_params",66, &reg_value)
 */
static int
dev_wlc_intvar_get_reg(struct net_device *dev, char *name,
	uint reg, int *retval)
{
	union {
		char buf[WLC_IOCTL_SMLEN];
		int val;
	} var;
	int error;

	bcm_mkiovar(name, (char *)(&reg), sizeof(reg),
		(char *)(&var), sizeof(var.buf));
	error = wldev_ioctl(dev, WLC_GET_VAR, (char *)(&var), sizeof(var.buf), false);

	*retval = dtoh32(var.val);
	return (error);
}

static int
dev_wlc_bufvar_set(struct net_device *dev, char *name, char *buf, int len)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
	char ioctlbuf_local[1024];
#else
	static char ioctlbuf_local[1024];
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31) */

	bcm_mkiovar(name, buf, len, ioctlbuf_local, sizeof(ioctlbuf_local));

	return (wldev_ioctl(dev, WLC_SET_VAR, ioctlbuf_local, sizeof(ioctlbuf_local), true));
}
/*
get named driver variable to uint register value and return error indication
calling example: dev_wlc_intvar_set_reg(dev, "btc_params",66, value)
*/
static int
dev_wlc_intvar_set_reg(struct net_device *dev, char *name, char *addr, char * val)
{
	char reg_addr[8];

	memset(reg_addr, 0, sizeof(reg_addr));
	memcpy((char *)&reg_addr[0], (char *)addr, 4);
	memcpy((char *)&reg_addr[4], (char *)val, 4);

	return (dev_wlc_bufvar_set(dev, name, (char *)&reg_addr[0], sizeof(reg_addr)));
}

static bool btcoex_is_sco_active(struct net_device *dev)
{
	int ioc_res = 0;
	bool res = FALSE;
	int sco_id_cnt = 0;
	int param27;
	int i;

	for (i = 0; i < 12; i++) {

		ioc_res = dev_wlc_intvar_get_reg(dev, "btc_params", 27, &param27);

		WL_TRACE(("sample[%d], btc params: 27:%x\n", i, param27));

		if (ioc_res < 0) {
			WL_ERR(("ioc read btc params error\n"));
			break;
		}

		if ((param27 & 0x6) == 2) { /* count both sco & esco  */
			sco_id_cnt++;
		}

		if (sco_id_cnt > 2) {
			WL_TRACE(("sco/esco detected, pkt id_cnt:%d  samples:%d\n",
				sco_id_cnt, i));
			res = TRUE;
			break;
		}

		OSL_SLEEP(5);
	}

	return res;
}

#if defined(BT_DHCP_eSCO_FIX)
/* Enhanced BT COEX settings for eSCO compatibility during DHCP window */
static int set_btc_esco_params(struct btcoex_info *btco_inf, bool trump_sco)
{
	struct net_device *dev = btco_inf->dev;

	char buf_reg50va_dhcp_on[8] =
		{ 50, 00, 00, 00, 0x22, 0x80, 0x00, 0x00 };
	char buf_reg51va_dhcp_on[8] =
		{ 51, 00, 00, 00, 0x00, 0x00, 0x00, 0x00 };
	char buf_reg64va_dhcp_on[8] =
		{ 64, 00, 00, 00, 0x00, 0x00, 0x00, 0x00 };
	char buf_reg65va_dhcp_on[8] =
		{ 65, 00, 00, 00, 0x00, 0x00, 0x00, 0x00 };
	char buf_reg71va_dhcp_on[8] =
		{ 71, 00, 00, 00, 0x00, 0x00, 0x00, 0x00 };
	uint32 regaddr;

	if (trump_sco) {
		/* this should reduce eSCO aggressive retransmit
		 * w/o breaking it
		 */

		/* 1st save current */
		WL_TRACE(("Do new SCO/eSCO coex algo {save & override}\n"));
		if (!dev_wlc_intvar_get_reg(dev, "btc_params", 50, &btco_inf->saved_reg50) &&
		    !dev_wlc_intvar_get_reg(dev, "btc_params", 51, &btco_inf->saved_reg51) &&
		    !dev_wlc_intvar_get_reg(dev, "btc_params", 64, &btco_inf->saved_reg64) &&
		    !dev_wlc_intvar_get_reg(dev, "btc_params", 65, &btco_inf->saved_reg65) &&
		    !dev_wlc_intvar_get_reg(dev, "btc_params", 71, &btco_inf->saved_reg71)) {
			btco_inf->esco_saved_status = TRUE;
			WL_TRACE(("saved bt_params[50,51,64,65,71]:"
				  "0x%x 0x%x 0x%x 0x%x 0x%x\n",
				  btco_inf->saved_reg50, btco_inf->saved_reg51,
				  btco_inf->saved_reg64, btco_inf->saved_reg65,
				  btco_inf->saved_reg71));
		} else {
			WL_ERR((":%s: save btc_params failed\n",
				__FUNCTION__));
			btco_inf->esco_saved_status = FALSE;
			return -1;
		}

		WL_TRACE(("override with [50,51,64,65,71]:"
			  "0x%x 0x%x 0x%x 0x%x 0x%x\n",
			  *(u32 *)(buf_reg50va_dhcp_on+4),
			  *(u32 *)(buf_reg51va_dhcp_on+4),
			  *(u32 *)(buf_reg64va_dhcp_on+4),
			  *(u32 *)(buf_reg65va_dhcp_on+4),
			  *(u32 *)(buf_reg71va_dhcp_on+4)));

		dev_wlc_bufvar_set(dev, "btc_params",
				   buf_reg50va_dhcp_on,
				   sizeof(buf_reg50va_dhcp_on));
		dev_wlc_bufvar_set(dev, "btc_params",
				   buf_reg51va_dhcp_on,
				   sizeof(buf_reg51va_dhcp_on));
		dev_wlc_bufvar_set(dev, "btc_params",
				   buf_reg64va_dhcp_on,
				   sizeof(buf_reg64va_dhcp_on));
		dev_wlc_bufvar_set(dev, "btc_params",
				   buf_reg65va_dhcp_on,
				   sizeof(buf_reg65va_dhcp_on));
		dev_wlc_bufvar_set(dev, "btc_params",
				   buf_reg71va_dhcp_on,
				   sizeof(buf_reg71va_dhcp_on));
	} else if (btco_inf->esco_saved_status) {
		/* restore previously saved bt params */
		WL_TRACE(("Do new SCO/eSCO coex algo {save & override}\n"));

		regaddr = 50;
		dev_wlc_intvar_set_reg(dev, "btc_params",
			(char *)&regaddr, (char *)&btco_inf->saved_reg50);
		regaddr = 51;
		dev_wlc_intvar_set_reg(dev, "btc_params",
			(char *)&regaddr, (char *)&btco_inf->saved_reg51);
		regaddr = 64;
		dev_wlc_intvar_set_reg(dev, "btc_params",
			(char *)&regaddr, (char *)&btco_inf->saved_reg64);
		regaddr = 65;
		dev_wlc_intvar_set_reg(dev, "btc_params",
			(char *)&regaddr, (char *)&btco_inf->saved_reg65);
		regaddr = 71;
		dev_wlc_intvar_set_reg(dev, "btc_params",
			(char *)&regaddr, (char *)&btco_inf->saved_reg71);

		WL_TRACE(("restore bt_params[50,51,64,65,71]:"
			"0x%x 0x%x 0x%x 0x%x 0x%x\n",
			btco_inf->saved_reg50, btco_inf->saved_reg51,
			btco_inf->saved_reg64, btco_inf->saved_reg65,
			btco_inf->saved_reg71));

		btco_inf->esco_saved_status = FALSE;
	} else {
		WL_ERR((":%s att to restore not saved BTCOEX params\n",
			__FUNCTION__));
		return -1;
	}
	return 0;
}
#endif /* BT_DHCP_eSCO_FIX */

static void
wl_cfg80211_bt_setflag(struct btcoex_info *btco_inf, bool set)
{
	struct net_device *dev = btco_inf->dev;
#if defined(BT_DHCP_USE_FLAGS)
	char buf_flag7_dhcp_on[8] = { 7, 00, 00, 00, 0x1, 0x0, 0x00, 0x00 };
	char buf_flag7_default[8]   = { 7, 00, 00, 00, 0x0, 0x00, 0x00, 0x00};
#endif

#if defined(BT_DHCP_eSCO_FIX)
	/* set = 1, save & turn on  0 - off & restore prev settings */
	set_btc_esco_params(btco_inf, set);
#endif

#if defined(BT_DHCP_USE_FLAGS)
	WL_TRACE(("WI-FI priority boost via bt flags, set:%d\n", set));
	if (set == TRUE)
		/* Forcing bt_flag7  */
		dev_wlc_bufvar_set(dev, "btc_flags",
			(char *)&buf_flag7_dhcp_on[0],
			sizeof(buf_flag7_dhcp_on));
	else
		/* Restoring default bt flag7 */
		dev_wlc_bufvar_set(dev, "btc_flags",
			(char *)&buf_flag7_default[0],
			sizeof(buf_flag7_default));
#endif
}

static void wl_cfg80211_bt_handler(struct work_struct *work)
{
	struct btcoex_info *btcx_inf = container_of(work, struct btcoex_info,
						    work.work);

	switch (btcx_inf->bt_state) {
		case BT_DHCP_START:
			/* DHCP started
			 * provide OPPORTUNITY window to get DHCP address
			 */
			WL_TRACE(("bt_dhcp stm: started \n"));

			btcx_inf->bt_state = BT_DHCP_OPPR_WIN;
			schedule_delayed_work(&btcx_inf->work,
				msecs_to_jiffies(BT_DHCP_OPPR_WIN_TIME));
			break;

		case BT_DHCP_OPPR_WIN:
			/* DHCP is not over yet, start lowering BT priority
			 * enforce btc_params + flags if necessary
			 */
			WL_TRACE(("DHCP T1:%d expired\n", BT_DHCP_OPPR_WIN_TIME));
			wl_cfg80211_bt_setflag(btcx_inf, TRUE);
			btcx_inf->bt_state = BT_DHCP_FLAG_FORCE_TIMEOUT;
			schedule_delayed_work(&btcx_inf->work,
				msecs_to_jiffies(BT_DHCP_FLAG_FORCE_TIME));
			break;

		case BT_DHCP_FLAG_FORCE_TIMEOUT:
			/* No dhcp during T1+T2, restore BT priority */
			WL_TRACE(("DHCP wait interval T2:%d msec expired\n",
				BT_DHCP_FLAG_FORCE_TIME));

			/* Restoring default bt priority */
			wl_cfg80211_bt_setflag(btcx_inf, FALSE);
			btcx_inf->bt_state = BT_DHCP_IDLE;
			break;

		default:
			WL_ERR(("error g_status=%d !!!\n",	btcx_inf->bt_state));
			wl_cfg80211_bt_setflag(btcx_inf, FALSE);
			btcx_inf->bt_state = BT_DHCP_IDLE;
			break;
	}

	net_os_wake_unlock(btcx_inf->dev);
}

void *wl_cfg80211_btcoex_init(struct net_device *ndev)
{
	struct btcoex_info *btco_inf;

	btco_inf = kzalloc(sizeof(struct btcoex_info), GFP_KERNEL);
	if (!btco_inf)
		return NULL;

	btco_inf->bt_state = BT_DHCP_IDLE;
	btco_inf->dev = ndev;

	INIT_DELAYED_WORK(&btco_inf->work, wl_cfg80211_bt_handler);

	return btco_inf;
}

void wl_cfg80211_btcoex_deinit(void *inf)
{
	struct btcoex_info *btco_inf = inf;

	if (btco_inf) {
		cancel_delayed_work_sync(&btco_inf->work);
		kfree(btco_inf);
	}
}

int wl_cfg80211_set_btcoex_dhcp(struct net_device *dev, int mode)
{
	dhd_pub_t *dhd = wl_cfg80211_get_dhdp(dev);
	struct bcm_cfg80211 *cfg = wl_get_cfg(dev);
	struct btcoex_info *btco_inf = cfg->btcoex_info;
	char buf_reg66va_dhcp_on[8] = { 66, 00, 00, 00, 0x10, 0x27, 0x00, 0x00 };
	char buf_reg41va_dhcp_on[8] = { 41, 00, 00, 00, 0x33, 0x00, 0x00, 0x00 };
	char buf_reg68va_dhcp_on[8] = { 68, 00, 00, 00, 0x90, 0x01, 0x00, 0x00 };

	uint32 regaddr;
	char buf_flag7_default[8] =   { 7, 00, 00, 00, 0x0, 0x00, 0x00, 0x00};

	switch (mode) {
	case 1:
		WL_TRACE_HW4(("DHCP session starts\n"));

#ifdef PKT_FILTER_SUPPORT
		dhd->dhcp_in_progress = 1;

		if (dhd->early_suspended) {
			WL_TRACE_HW4(("DHCP in progressing , disable packet filter!!!\n"));
			dhd_enable_packet_filter(0, dhd);
		}
#endif

		if (!btco_inf->dhcp_saved_status) {
			/* Retrieve and saved orig regs value */
			if (!dev_wlc_intvar_get_reg(dev, "btc_params", 66,
						    &btco_inf->saved_reg66) &&
			    !dev_wlc_intvar_get_reg(dev, "btc_params", 41,
						    &btco_inf->saved_reg41) &&
			    !dev_wlc_intvar_get_reg(dev, "btc_params", 68,
						    &btco_inf->saved_reg68)) {
				btco_inf->dhcp_saved_status = TRUE;
				WL_TRACE(("Saved 0x%x 0x%x 0x%x\n",
					btco_inf->saved_reg66, btco_inf->saved_reg41,
					btco_inf->saved_reg68));

				/* Disable PM mode during dhpc session */
				/* Start BT timer only for SCO connection */
				if (btcoex_is_sco_active(dev)) {
					/* btc_params 66 */
					dev_wlc_bufvar_set(dev, "btc_params",
						buf_reg66va_dhcp_on,
						sizeof(buf_reg66va_dhcp_on));
					/* btc_params 41 0x33 */
					dev_wlc_bufvar_set(dev, "btc_params",
						buf_reg41va_dhcp_on,
						sizeof(buf_reg41va_dhcp_on));
					/* btc_params 68 0x190 */
					dev_wlc_bufvar_set(dev, "btc_params",
						buf_reg68va_dhcp_on,
						sizeof(buf_reg68va_dhcp_on));

					WL_TRACE(("enable BT DHCP Work\n"));
					btco_inf->bt_state = BT_DHCP_START;
					mod_delayed_work(system_wq, &btco_inf->work, 0);
				}
			}
		} else {
			WL_ERR(("was called w/o DHCP OFF. Continue\n"));
		}
		break;

	case 2:
#ifdef PKT_FILTER_SUPPORT
		dhd->dhcp_in_progress = 0;
		WL_TRACE_HW4(("DHCP is complete \n"));

		/* Enable packet filtering */
		if (dhd->early_suspended) {
			WL_TRACE_HW4(("DHCP is complete , enable packet filter!!!\n"));
			dhd_enable_packet_filter(1, dhd);
		}
#endif /* PKT_FILTER_SUPPORT */

		/* Restoring PM mode */
		cancel_delayed_work_sync(&btco_inf->work);
		WL_TRACE(("bt->bt_state:%d\n", btco_inf->bt_state));
		if (btco_inf->bt_state == BT_DHCP_FLAG_FORCE_TIMEOUT) {
			/* need to restore original btc flags & extra btc params */
			wl_cfg80211_bt_setflag(btco_inf, FALSE);
		}
		btco_inf->bt_state = BT_DHCP_IDLE;

		if (btco_inf->dhcp_saved_status) {
			/* Restoring default btc_flag */
			dev_wlc_bufvar_set(dev, "btc_flags",
					   buf_flag7_default,
					   sizeof(buf_flag7_default));

			/* Restore original values */
			regaddr = 66;
			dev_wlc_intvar_set_reg(dev, "btc_params",
				(char *)&regaddr,
				(char *)&btco_inf->saved_reg66);
			regaddr = 41;
			dev_wlc_intvar_set_reg(dev, "btc_params",
				(char *)&regaddr,
				(char *)&btco_inf->saved_reg41);
			regaddr = 68;
			dev_wlc_intvar_set_reg(dev, "btc_params",
				(char *)&regaddr,
				(char *)&btco_inf->saved_reg68);

			WL_TRACE(("restore regs {66,41,68} <- 0x%x 0x%x 0x%x\n",
				btco_inf->saved_reg66, btco_inf->saved_reg41,
				btco_inf->saved_reg68));

			btco_inf->dhcp_saved_status = FALSE;
		}
		break;

	default:
		WL_ERR(("Unkwown yet power setting, ignored\n"));
		return -EINVAL;
	}

	return 0;
}
