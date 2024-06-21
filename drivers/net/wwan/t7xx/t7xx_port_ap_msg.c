// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2023, Intel Corporation.
 */

#include "t7xx_port.h"
#include "t7xx_port_ap_msg.h"
#include "t7xx_port_devlink.h"
#include "t7xx_port_proxy.h"
#include "t7xx_state_monitor.h"

int t7xx_port_ap_msg_tx(struct t7xx_port *port, char *buff, size_t len)
{
	const struct t7xx_port_conf *port_conf;
	size_t offset, chunk_len = 0, txq_mtu;
	struct t7xx_fsm_ctl *ctl;
	struct sk_buff *skb_ccci;
	enum md_state md_state;
	int ret;

	if (!len || !port->chan_enable)
		return -EINVAL;

	port_conf = port->port_conf;
	ctl = port->t7xx_dev->md->fsm_ctl;
	md_state = t7xx_fsm_get_md_state(ctl);
	if (md_state == MD_STATE_WAITING_FOR_HS1 || md_state == MD_STATE_WAITING_FOR_HS2) {
		dev_warn(port->dev, "Cannot write to %s port when md_state=%d\n",
			 port_conf->name, md_state);
		return -ENODEV;
	}

	txq_mtu = t7xx_get_port_mtu(port);
	for (offset = 0; offset < len; offset += chunk_len) {
		chunk_len = min(len - offset, txq_mtu - sizeof(struct ccci_header));
		skb_ccci = t7xx_port_alloc_skb(chunk_len);
		if (!skb_ccci)
			return -ENOMEM;

		skb_put_data(skb_ccci, buff + offset, chunk_len);
		ret = t7xx_port_send_skb(port, skb_ccci, 0, 0);
		if (ret) {
			dev_kfree_skb_any(skb_ccci);
			dev_err(port->dev, "Write error on %s port, %d\n",
				port_conf->name, ret);
			return ret;
		}
	}

	return len;
}

static int t7xx_port_ap_msg_init(struct t7xx_port *port)
{
	struct t7xx_devlink *dl = port->t7xx_dev->dl;

	port->rx_length_th = T7XX_MAX_QUEUE_LENGTH;
	dl->status = T7XX_DEVLINK_IDLE;
	dl->port = port;

	return 0;
}

static void t7xx_port_ap_msg_uninit(struct t7xx_port *port)
{
	struct t7xx_devlink *dl = port->t7xx_dev->dl;

	dl->mode = T7XX_NORMAL_MODE;
	skb_queue_purge(&port->rx_skb_list);
}

struct port_ops ap_msg_port_ops = {
	.init = &t7xx_port_ap_msg_init,
	.recv_skb = &t7xx_port_enqueue_skb,
	.uninit = &t7xx_port_ap_msg_uninit,
	.enable_chl = &t7xx_port_enable_chl,
	.disable_chl = &t7xx_port_disable_chl,
};
