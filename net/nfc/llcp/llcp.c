/*
 * Copyright (C) 2011  Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#define pr_fmt(fmt) "llcp: %s: " fmt, __func__

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/nfc.h>

#include "../nfc.h"
#include "llcp.h"

static u8 llcp_magic[3] = {0x46, 0x66, 0x6d};

static struct list_head llcp_devices;

static void nfc_llcp_socket_release(struct nfc_llcp_local *local)
{
	struct nfc_llcp_sock *parent, *s, *n;
	struct sock *sk, *parent_sk;
	int i;

	mutex_lock(&local->socket_lock);

	for (i = 0; i < LLCP_MAX_SAP; i++) {
		parent = local->sockets[i];
		if (parent == NULL)
			continue;

		/* Release all child sockets */
		list_for_each_entry_safe(s, n, &parent->list, list) {
			list_del_init(&s->list);
			sk = &s->sk;

			lock_sock(sk);

			if (sk->sk_state == LLCP_CONNECTED)
				nfc_put_device(s->dev);

			sk->sk_state = LLCP_CLOSED;

			release_sock(sk);

			sock_orphan(sk);

			s->local = NULL;
		}

		parent_sk = &parent->sk;

		lock_sock(parent_sk);

		if (parent_sk->sk_state == LLCP_LISTEN) {
			struct nfc_llcp_sock *lsk, *n;
			struct sock *accept_sk;

			list_for_each_entry_safe(lsk, n, &parent->accept_queue,
						 accept_queue) {
				accept_sk = &lsk->sk;
				lock_sock(accept_sk);

				nfc_llcp_accept_unlink(accept_sk);

				accept_sk->sk_state = LLCP_CLOSED;

				release_sock(accept_sk);

				sock_orphan(accept_sk);

				lsk->local = NULL;
			}
		}

		if (parent_sk->sk_state == LLCP_CONNECTED)
			nfc_put_device(parent->dev);

		parent_sk->sk_state = LLCP_CLOSED;

		release_sock(parent_sk);

		sock_orphan(parent_sk);

		parent->local = NULL;
	}

	mutex_unlock(&local->socket_lock);
}

static void nfc_llcp_clear_sdp(struct nfc_llcp_local *local)
{
	mutex_lock(&local->sdp_lock);

	local->local_wks = 0;
	local->local_sdp = 0;
	local->local_sap = 0;

	mutex_unlock(&local->sdp_lock);
}

static void nfc_llcp_timeout_work(struct work_struct *work)
{
	struct nfc_llcp_local *local = container_of(work, struct nfc_llcp_local,
						    timeout_work);

	nfc_dep_link_down(local->dev);
}

static void nfc_llcp_symm_timer(unsigned long data)
{
	struct nfc_llcp_local *local = (struct nfc_llcp_local *) data;

	pr_err("SYMM timeout\n");

	queue_work(local->timeout_wq, &local->timeout_work);
}

struct nfc_llcp_local *nfc_llcp_find_local(struct nfc_dev *dev)
{
	struct nfc_llcp_local *local, *n;

	list_for_each_entry_safe(local, n, &llcp_devices, list)
		if (local->dev == dev)
			return local;

	pr_debug("No device found\n");

	return NULL;
}

static char *wks[] = {
	NULL,
	NULL, /* SDP */
	"urn:nfc:sn:ip",
	"urn:nfc:sn:obex",
	"urn:nfc:sn:snep",
};

static int nfc_llcp_wks_sap(char *service_name, size_t service_name_len)
{
	int sap, num_wks;

	pr_debug("%s\n", service_name);

	if (service_name == NULL)
		return -EINVAL;

	num_wks = ARRAY_SIZE(wks);

	for (sap = 0; sap < num_wks; sap++) {
		if (wks[sap] == NULL)
			continue;

		if (strncmp(wks[sap], service_name, service_name_len) == 0)
			return sap;
	}

	return -EINVAL;
}

u8 nfc_llcp_get_sdp_ssap(struct nfc_llcp_local *local,
			 struct nfc_llcp_sock *sock)
{
	mutex_lock(&local->sdp_lock);

	if (sock->service_name != NULL && sock->service_name_len > 0) {
		int ssap = nfc_llcp_wks_sap(sock->service_name,
					    sock->service_name_len);

		if (ssap > 0) {
			pr_debug("WKS %d\n", ssap);

			/* This is a WKS, let's check if it's free */
			if (local->local_wks & BIT(ssap)) {
				mutex_unlock(&local->sdp_lock);

				return LLCP_SAP_MAX;
			}

			set_bit(ssap, &local->local_wks);
			mutex_unlock(&local->sdp_lock);

			return ssap;
		}

		/*
		 * This is not a well known service,
		 * we should try to find a local SDP free spot
		 */
		ssap = find_first_zero_bit(&local->local_sdp, LLCP_SDP_NUM_SAP);
		if (ssap == LLCP_SDP_NUM_SAP) {
			mutex_unlock(&local->sdp_lock);

			return LLCP_SAP_MAX;
		}

		pr_debug("SDP ssap %d\n", LLCP_WKS_NUM_SAP + ssap);

		set_bit(ssap, &local->local_sdp);
		mutex_unlock(&local->sdp_lock);

		return LLCP_WKS_NUM_SAP + ssap;

	} else if (sock->ssap != 0) {
		if (sock->ssap < LLCP_WKS_NUM_SAP) {
			if (!test_bit(sock->ssap, &local->local_wks)) {
				set_bit(sock->ssap, &local->local_wks);
				mutex_unlock(&local->sdp_lock);

				return sock->ssap;
			}

		} else if (sock->ssap < LLCP_SDP_NUM_SAP) {
			if (!test_bit(sock->ssap - LLCP_WKS_NUM_SAP,
				      &local->local_sdp)) {
				set_bit(sock->ssap - LLCP_WKS_NUM_SAP,
					&local->local_sdp);
				mutex_unlock(&local->sdp_lock);

				return sock->ssap;
			}
		}
	}

	mutex_unlock(&local->sdp_lock);

	return LLCP_SAP_MAX;
}

u8 nfc_llcp_get_local_ssap(struct nfc_llcp_local *local)
{
	u8 local_ssap;

	mutex_lock(&local->sdp_lock);

	local_ssap = find_first_zero_bit(&local->local_sap, LLCP_LOCAL_NUM_SAP);
	if (local_ssap == LLCP_LOCAL_NUM_SAP) {
		mutex_unlock(&local->sdp_lock);
		return LLCP_SAP_MAX;
	}

	set_bit(local_ssap, &local->local_sap);

	mutex_unlock(&local->sdp_lock);

	return local_ssap + LLCP_LOCAL_SAP_OFFSET;
}

void nfc_llcp_put_ssap(struct nfc_llcp_local *local, u8 ssap)
{
	u8 local_ssap;
	unsigned long *sdp;

	if (ssap < LLCP_WKS_NUM_SAP) {
		local_ssap = ssap;
		sdp = &local->local_wks;
	} else if (ssap < LLCP_LOCAL_NUM_SAP) {
		local_ssap = ssap - LLCP_WKS_NUM_SAP;
		sdp = &local->local_sdp;
	} else if (ssap < LLCP_MAX_SAP) {
		local_ssap = ssap - LLCP_LOCAL_NUM_SAP;
		sdp = &local->local_sap;
	} else {
		return;
	}

	mutex_lock(&local->sdp_lock);

	clear_bit(local_ssap, sdp);

	mutex_unlock(&local->sdp_lock);
}

u8 *nfc_llcp_general_bytes(struct nfc_dev *dev, size_t *general_bytes_len)
{
	struct nfc_llcp_local *local;

	local = nfc_llcp_find_local(dev);
	if (local == NULL) {
		*general_bytes_len = 0;
		return NULL;
	}

	*general_bytes_len = local->gb_len;

	return local->gb;
}

static int nfc_llcp_build_gb(struct nfc_llcp_local *local)
{
	u8 *gb_cur, *version_tlv, version, version_length;
	u8 *lto_tlv, lto, lto_length;
	u8 *wks_tlv, wks_length;
	u8 gb_len = 0;

	version = LLCP_VERSION_11;
	version_tlv = nfc_llcp_build_tlv(LLCP_TLV_VERSION, &version,
					 1, &version_length);
	gb_len += version_length;

	/* 1500 ms */
	lto = 150;
	lto_tlv = nfc_llcp_build_tlv(LLCP_TLV_VERSION, &lto, 1, &lto_length);
	gb_len += lto_length;

	pr_debug("Local wks 0x%lx\n", local->local_wks);
	wks_tlv = nfc_llcp_build_tlv(LLCP_TLV_WKS, (u8 *)&local->local_wks, 2,
				     &wks_length);
	gb_len += wks_length;

	gb_len += ARRAY_SIZE(llcp_magic);

	if (gb_len > NFC_MAX_GT_LEN) {
		kfree(version_tlv);
		return -EINVAL;
	}

	gb_cur = local->gb;

	memcpy(gb_cur, llcp_magic, ARRAY_SIZE(llcp_magic));
	gb_cur += ARRAY_SIZE(llcp_magic);

	memcpy(gb_cur, version_tlv, version_length);
	gb_cur += version_length;

	memcpy(gb_cur, lto_tlv, lto_length);
	gb_cur += lto_length;

	memcpy(gb_cur, wks_tlv, wks_length);
	gb_cur += wks_length;

	kfree(version_tlv);
	kfree(lto_tlv);

	local->gb_len = gb_len;

	return 0;
}

int nfc_llcp_set_remote_gb(struct nfc_dev *dev, u8 *gb, u8 gb_len)
{
	struct nfc_llcp_local *local = nfc_llcp_find_local(dev);

	if (local == NULL) {
		pr_err("No LLCP device\n");
		return -ENODEV;
	}

	memset(local->remote_gb, 0, NFC_MAX_GT_LEN);
	memcpy(local->remote_gb, gb, gb_len);
	local->remote_gb_len = gb_len;

	if (local->remote_gb == NULL || local->remote_gb_len == 0)
		return -ENODEV;

	if (memcmp(local->remote_gb, llcp_magic, 3)) {
		pr_err("MAC does not support LLCP\n");
		return -EINVAL;
	}

	return nfc_llcp_parse_tlv(local,
				  &local->remote_gb[3],
				  local->remote_gb_len - 3);
}

static void nfc_llcp_tx_work(struct work_struct *work)
{
	struct nfc_llcp_local *local = container_of(work, struct nfc_llcp_local,
						    tx_work);
	struct sk_buff *skb;

	skb = skb_dequeue(&local->tx_queue);
	if (skb != NULL) {
		pr_debug("Sending pending skb\n");
		nfc_data_exchange(local->dev, local->target_idx,
				  skb, nfc_llcp_recv, local);
	} else {
		nfc_llcp_send_symm(local->dev);
	}

	mod_timer(&local->link_timer,
		  jiffies + msecs_to_jiffies(local->remote_lto));
}

static u8 nfc_llcp_dsap(struct sk_buff *pdu)
{
	return (pdu->data[0] & 0xfc) >> 2;
}

static u8 nfc_llcp_ptype(struct sk_buff *pdu)
{
	return ((pdu->data[0] & 0x03) << 2) | ((pdu->data[1] & 0xc0) >> 6);
}

static u8 nfc_llcp_ssap(struct sk_buff *pdu)
{
	return pdu->data[1] & 0x3f;
}

static u8 nfc_llcp_ns(struct sk_buff *pdu)
{
	return pdu->data[2] >> 4;
}

static u8 nfc_llcp_nr(struct sk_buff *pdu)
{
	return pdu->data[2] & 0xf;
}

static void nfc_llcp_set_nrns(struct nfc_llcp_sock *sock, struct sk_buff *pdu)
{
	pdu->data[2] = (sock->send_n << 4) | (sock->recv_n % 16);
	sock->send_n = (sock->send_n + 1) % 16;
	sock->recv_ack_n = (sock->recv_n - 1) % 16;
}

static struct nfc_llcp_sock *nfc_llcp_sock_get(struct nfc_llcp_local *local,
					       u8 ssap, u8 dsap)
{
	struct nfc_llcp_sock *sock, *llcp_sock, *n;

	if (ssap == 0 && dsap == 0)
		return NULL;

	mutex_lock(&local->socket_lock);
	sock = local->sockets[ssap];
	if (sock == NULL) {
		mutex_unlock(&local->socket_lock);
		return NULL;
	}

	pr_debug("root dsap %d (%d)\n", sock->dsap, dsap);

	if (sock->dsap == dsap) {
		sock_hold(&sock->sk);
		mutex_unlock(&local->socket_lock);
		return sock;
	}

	list_for_each_entry_safe(llcp_sock, n, &sock->list, list) {
		pr_debug("llcp_sock %p sk %p dsap %d\n", llcp_sock,
			 &llcp_sock->sk, llcp_sock->dsap);
		if (llcp_sock->dsap == dsap) {
			sock_hold(&llcp_sock->sk);
			mutex_unlock(&local->socket_lock);
			return llcp_sock;
		}
	}

	pr_err("Could not find socket for %d %d\n", ssap, dsap);

	mutex_unlock(&local->socket_lock);

	return NULL;
}

static void nfc_llcp_sock_put(struct nfc_llcp_sock *sock)
{
	sock_put(&sock->sk);
}

static u8 *nfc_llcp_connect_sn(struct sk_buff *skb, size_t *sn_len)
{
	u8 *tlv = &skb->data[2], type, length;
	size_t tlv_array_len = skb->len - LLCP_HEADER_SIZE, offset = 0;

	while (offset < tlv_array_len) {
		type = tlv[0];
		length = tlv[1];

		pr_debug("type 0x%x length %d\n", type, length);

		if (type == LLCP_TLV_SN) {
			*sn_len = length;
			return &tlv[2];
		}

		offset += length + 2;
		tlv += length + 2;
	}

	return NULL;
}

static void nfc_llcp_recv_connect(struct nfc_llcp_local *local,
				  struct sk_buff *skb)
{
	struct sock *new_sk, *parent;
	struct nfc_llcp_sock *sock, *new_sock;
	u8 dsap, ssap, bound_sap, reason;

	dsap = nfc_llcp_dsap(skb);
	ssap = nfc_llcp_ssap(skb);

	pr_debug("%d %d\n", dsap, ssap);

	nfc_llcp_parse_tlv(local, &skb->data[LLCP_HEADER_SIZE],
			   skb->len - LLCP_HEADER_SIZE);

	if (dsap != LLCP_SAP_SDP) {
		bound_sap = dsap;

		mutex_lock(&local->socket_lock);
		sock = local->sockets[dsap];
		if (sock == NULL) {
			mutex_unlock(&local->socket_lock);
			reason = LLCP_DM_NOBOUND;
			goto fail;
		}

		sock_hold(&sock->sk);
		mutex_unlock(&local->socket_lock);

		lock_sock(&sock->sk);

		if (sock->dsap == LLCP_SAP_SDP &&
		    sock->sk.sk_state == LLCP_LISTEN)
			goto enqueue;
	} else {
		u8 *sn;
		size_t sn_len;

		sn = nfc_llcp_connect_sn(skb, &sn_len);
		if (sn == NULL) {
			reason = LLCP_DM_NOBOUND;
			goto fail;
		}

		pr_debug("Service name length %zu\n", sn_len);

		mutex_lock(&local->socket_lock);
		for (bound_sap = 0; bound_sap < LLCP_LOCAL_SAP_OFFSET;
		     bound_sap++) {
			sock = local->sockets[bound_sap];
			if (sock == NULL)
				continue;

			if (sock->service_name == NULL ||
			    sock->service_name_len == 0)
					continue;

			if (sock->service_name_len != sn_len)
				continue;

			if (sock->dsap == LLCP_SAP_SDP &&
			    sock->sk.sk_state == LLCP_LISTEN &&
			    !memcmp(sn, sock->service_name, sn_len)) {
				pr_debug("Found service name at SAP %d\n",
					 bound_sap);
				sock_hold(&sock->sk);
				mutex_unlock(&local->socket_lock);

				lock_sock(&sock->sk);

				goto enqueue;
			}
		}
		mutex_unlock(&local->socket_lock);
	}

	reason = LLCP_DM_NOBOUND;
	goto fail;

enqueue:
	parent = &sock->sk;

	if (sk_acceptq_is_full(parent)) {
		reason = LLCP_DM_REJ;
		release_sock(&sock->sk);
		sock_put(&sock->sk);
		goto fail;
	}

	new_sk = nfc_llcp_sock_alloc(NULL, parent->sk_type, GFP_ATOMIC);
	if (new_sk == NULL) {
		reason = LLCP_DM_REJ;
		release_sock(&sock->sk);
		sock_put(&sock->sk);
		goto fail;
	}

	new_sock = nfc_llcp_sock(new_sk);
	new_sock->dev = local->dev;
	new_sock->local = local;
	new_sock->nfc_protocol = sock->nfc_protocol;
	new_sock->ssap = bound_sap;
	new_sock->dsap = ssap;
	new_sock->parent = parent;

	pr_debug("new sock %p sk %p\n", new_sock, &new_sock->sk);

	list_add_tail(&new_sock->list, &sock->list);

	nfc_llcp_accept_enqueue(&sock->sk, new_sk);

	nfc_get_device(local->dev->idx);

	new_sk->sk_state = LLCP_CONNECTED;

	/* Wake the listening processes */
	parent->sk_data_ready(parent, 0);

	/* Send CC */
	nfc_llcp_send_cc(new_sock);

	release_sock(&sock->sk);
	sock_put(&sock->sk);

	return;

fail:
	/* Send DM */
	nfc_llcp_send_dm(local, dsap, ssap, reason);

	return;

}

int nfc_llcp_queue_i_frames(struct nfc_llcp_sock *sock)
{
	int nr_frames = 0;
	struct nfc_llcp_local *local = sock->local;

	pr_debug("Remote ready %d tx queue len %d remote rw %d",
		 sock->remote_ready, skb_queue_len(&sock->tx_pending_queue),
		 local->remote_rw);

	/* Try to queue some I frames for transmission */
	while (sock->remote_ready &&
	       skb_queue_len(&sock->tx_pending_queue) < local->remote_rw) {
		struct sk_buff *pdu, *pending_pdu;

		pdu = skb_dequeue(&sock->tx_queue);
		if (pdu == NULL)
			break;

		/* Update N(S)/N(R) */
		nfc_llcp_set_nrns(sock, pdu);

		pending_pdu = skb_clone(pdu, GFP_KERNEL);

		skb_queue_tail(&local->tx_queue, pdu);
		skb_queue_tail(&sock->tx_pending_queue, pending_pdu);
		nr_frames++;
	}

	return nr_frames;
}

static void nfc_llcp_recv_hdlc(struct nfc_llcp_local *local,
			       struct sk_buff *skb)
{
	struct nfc_llcp_sock *llcp_sock;
	struct sock *sk;
	u8 dsap, ssap, ptype, ns, nr;

	ptype = nfc_llcp_ptype(skb);
	dsap = nfc_llcp_dsap(skb);
	ssap = nfc_llcp_ssap(skb);
	ns = nfc_llcp_ns(skb);
	nr = nfc_llcp_nr(skb);

	pr_debug("%d %d R %d S %d\n", dsap, ssap, nr, ns);

	llcp_sock = nfc_llcp_sock_get(local, dsap, ssap);
	if (llcp_sock == NULL) {
		nfc_llcp_send_dm(local, dsap, ssap, LLCP_DM_NOCONN);
		return;
	}

	sk = &llcp_sock->sk;
	lock_sock(sk);
	if (sk->sk_state == LLCP_CLOSED) {
		release_sock(sk);
		nfc_llcp_sock_put(llcp_sock);
	}

	/* Pass the payload upstream */
	if (ptype == LLCP_PDU_I) {
		pr_debug("I frame, queueing on %p\n", &llcp_sock->sk);

		if (ns == llcp_sock->recv_n)
			llcp_sock->recv_n = (llcp_sock->recv_n + 1) % 16;
		else
			pr_err("Received out of sequence I PDU\n");

		skb_pull(skb, LLCP_HEADER_SIZE + LLCP_SEQUENCE_SIZE);
		if (sock_queue_rcv_skb(&llcp_sock->sk, skb)) {
			pr_err("receive queue is full\n");
			skb_queue_head(&llcp_sock->tx_backlog_queue, skb);
		}
	}

	/* Remove skbs from the pending queue */
	if (llcp_sock->send_ack_n != nr) {
		struct sk_buff *s, *tmp;

		llcp_sock->send_ack_n = nr;

		skb_queue_walk_safe(&llcp_sock->tx_pending_queue, s, tmp)
			if (nfc_llcp_ns(s) <= nr) {
				skb_unlink(s, &llcp_sock->tx_pending_queue);
				kfree_skb(s);
			}
	}

	if (ptype == LLCP_PDU_RR)
		llcp_sock->remote_ready = true;
	else if (ptype == LLCP_PDU_RNR)
		llcp_sock->remote_ready = false;

	if (nfc_llcp_queue_i_frames(llcp_sock) == 0)
		nfc_llcp_send_rr(llcp_sock);

	release_sock(sk);
	nfc_llcp_sock_put(llcp_sock);
}

static void nfc_llcp_recv_disc(struct nfc_llcp_local *local,
			       struct sk_buff *skb)
{
	struct nfc_llcp_sock *llcp_sock;
	struct sock *sk;
	u8 dsap, ssap;

	dsap = nfc_llcp_dsap(skb);
	ssap = nfc_llcp_ssap(skb);

	llcp_sock = nfc_llcp_sock_get(local, dsap, ssap);
	if (llcp_sock == NULL) {
		nfc_llcp_send_dm(local, dsap, ssap, LLCP_DM_NOCONN);
		return;
	}

	sk = &llcp_sock->sk;
	lock_sock(sk);
	if (sk->sk_state == LLCP_CLOSED) {
		release_sock(sk);
		nfc_llcp_sock_put(llcp_sock);
	}

	if (sk->sk_state == LLCP_CONNECTED) {
		nfc_put_device(local->dev);
		sk->sk_state = LLCP_CLOSED;
		sk->sk_state_change(sk);
	}

	nfc_llcp_send_dm(local, dsap, ssap, LLCP_DM_DISC);

	release_sock(sk);
	nfc_llcp_sock_put(llcp_sock);
}

static void nfc_llcp_recv_cc(struct nfc_llcp_local *local, struct sk_buff *skb)
{
	struct nfc_llcp_sock *llcp_sock;
	u8 dsap, ssap;

	dsap = nfc_llcp_dsap(skb);
	ssap = nfc_llcp_ssap(skb);

	llcp_sock = nfc_llcp_sock_get(local, dsap, ssap);

	if (llcp_sock == NULL)
		llcp_sock = nfc_llcp_sock_get(local, dsap, LLCP_SAP_SDP);

	if (llcp_sock == NULL) {
		pr_err("Invalid CC\n");
		nfc_llcp_send_dm(local, dsap, ssap, LLCP_DM_NOCONN);

		return;
	}

	llcp_sock->dsap = ssap;

	nfc_llcp_parse_tlv(local, &skb->data[LLCP_HEADER_SIZE],
			   skb->len - LLCP_HEADER_SIZE);

	nfc_llcp_sock_put(llcp_sock);
}

static void nfc_llcp_rx_work(struct work_struct *work)
{
	struct nfc_llcp_local *local = container_of(work, struct nfc_llcp_local,
						    rx_work);
	u8 dsap, ssap, ptype;
	struct sk_buff *skb;

	skb = local->rx_pending;
	if (skb == NULL) {
		pr_debug("No pending SKB\n");
		return;
	}

	ptype = nfc_llcp_ptype(skb);
	dsap = nfc_llcp_dsap(skb);
	ssap = nfc_llcp_ssap(skb);

	pr_debug("ptype 0x%x dsap 0x%x ssap 0x%x\n", ptype, dsap, ssap);

	switch (ptype) {
	case LLCP_PDU_SYMM:
		pr_debug("SYMM\n");
		break;

	case LLCP_PDU_CONNECT:
		pr_debug("CONNECT\n");
		nfc_llcp_recv_connect(local, skb);
		break;

	case LLCP_PDU_DISC:
		pr_debug("DISC\n");
		nfc_llcp_recv_disc(local, skb);
		break;

	case LLCP_PDU_CC:
		pr_debug("CC\n");
		nfc_llcp_recv_cc(local, skb);
		break;

	case LLCP_PDU_I:
	case LLCP_PDU_RR:
	case LLCP_PDU_RNR:
		pr_debug("I frame\n");
		nfc_llcp_recv_hdlc(local, skb);
		break;

	}

	queue_work(local->tx_wq, &local->tx_work);
	kfree_skb(local->rx_pending);
	local->rx_pending = NULL;

	return;
}

void nfc_llcp_recv(void *data, struct sk_buff *skb, int err)
{
	struct nfc_llcp_local *local = (struct nfc_llcp_local *) data;

	pr_debug("Received an LLCP PDU\n");
	if (err < 0) {
		pr_err("err %d\n", err);
		return;
	}

	local->rx_pending = skb_get(skb);
	del_timer(&local->link_timer);
	queue_work(local->rx_wq, &local->rx_work);

	return;
}

void nfc_llcp_mac_is_down(struct nfc_dev *dev)
{
	struct nfc_llcp_local *local;

	local = nfc_llcp_find_local(dev);
	if (local == NULL)
		return;

	nfc_llcp_clear_sdp(local);

	/* Close and purge all existing sockets */
	nfc_llcp_socket_release(local);
}

void nfc_llcp_mac_is_up(struct nfc_dev *dev, u32 target_idx,
			u8 comm_mode, u8 rf_mode)
{
	struct nfc_llcp_local *local;

	pr_debug("rf mode %d\n", rf_mode);

	local = nfc_llcp_find_local(dev);
	if (local == NULL)
		return;

	local->target_idx = target_idx;
	local->comm_mode = comm_mode;
	local->rf_mode = rf_mode;

	if (rf_mode == NFC_RF_INITIATOR) {
		pr_debug("Queueing Tx work\n");

		queue_work(local->tx_wq, &local->tx_work);
	} else {
		mod_timer(&local->link_timer,
			  jiffies + msecs_to_jiffies(local->remote_lto));
	}
}

int nfc_llcp_register_device(struct nfc_dev *ndev)
{
	struct device *dev = &ndev->dev;
	struct nfc_llcp_local *local;
	char name[32];
	int err;

	local = kzalloc(sizeof(struct nfc_llcp_local), GFP_KERNEL);
	if (local == NULL)
		return -ENOMEM;

	local->dev = ndev;
	INIT_LIST_HEAD(&local->list);
	mutex_init(&local->sdp_lock);
	mutex_init(&local->socket_lock);
	init_timer(&local->link_timer);
	local->link_timer.data = (unsigned long) local;
	local->link_timer.function = nfc_llcp_symm_timer;

	skb_queue_head_init(&local->tx_queue);
	INIT_WORK(&local->tx_work, nfc_llcp_tx_work);
	snprintf(name, sizeof(name), "%s_llcp_tx_wq", dev_name(dev));
	local->tx_wq =
		alloc_workqueue(name,
				WQ_NON_REENTRANT | WQ_UNBOUND | WQ_MEM_RECLAIM,
				1);
	if (local->tx_wq == NULL) {
		err = -ENOMEM;
		goto err_local;
	}

	local->rx_pending = NULL;
	INIT_WORK(&local->rx_work, nfc_llcp_rx_work);
	snprintf(name, sizeof(name), "%s_llcp_rx_wq", dev_name(dev));
	local->rx_wq =
		alloc_workqueue(name,
				WQ_NON_REENTRANT | WQ_UNBOUND | WQ_MEM_RECLAIM,
				1);
	if (local->rx_wq == NULL) {
		err = -ENOMEM;
		goto err_tx_wq;
	}

	INIT_WORK(&local->timeout_work, nfc_llcp_timeout_work);
	snprintf(name, sizeof(name), "%s_llcp_timeout_wq", dev_name(dev));
	local->timeout_wq =
		alloc_workqueue(name,
				WQ_NON_REENTRANT | WQ_UNBOUND | WQ_MEM_RECLAIM,
				1);
	if (local->timeout_wq == NULL) {
		err = -ENOMEM;
		goto err_rx_wq;
	}

	nfc_llcp_build_gb(local);

	local->remote_miu = LLCP_DEFAULT_MIU;
	local->remote_lto = LLCP_DEFAULT_LTO;
	local->remote_rw = LLCP_DEFAULT_RW;

	list_add(&llcp_devices, &local->list);

	return 0;

err_rx_wq:
	destroy_workqueue(local->rx_wq);

err_tx_wq:
	destroy_workqueue(local->tx_wq);

err_local:
	kfree(local);

	return 0;
}

void nfc_llcp_unregister_device(struct nfc_dev *dev)
{
	struct nfc_llcp_local *local = nfc_llcp_find_local(dev);

	if (local == NULL) {
		pr_debug("No such device\n");
		return;
	}

	list_del(&local->list);
	nfc_llcp_socket_release(local);
	del_timer_sync(&local->link_timer);
	skb_queue_purge(&local->tx_queue);
	destroy_workqueue(local->tx_wq);
	destroy_workqueue(local->rx_wq);
	kfree_skb(local->rx_pending);
	kfree(local);
}

int __init nfc_llcp_init(void)
{
	INIT_LIST_HEAD(&llcp_devices);

	return nfc_llcp_sock_init();
}

void nfc_llcp_exit(void)
{
	nfc_llcp_sock_exit();
}
