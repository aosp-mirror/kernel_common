/*
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
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
 * RMNET Data ingress/egress handler
 *
 */

#ifndef _RMNET_DATA_HANDLERS_H_
#define _RMNET_DATA_HANDLERS_H_

void rmnet_egress_handler(struct sk_buff *skb,
			  struct rmnet_logical_ep_conf_s *ep);

rx_handler_result_t rmnet_rx_handler(struct sk_buff **pskb);

#endif /* _RMNET_DATA_HANDLERS_H_ */
