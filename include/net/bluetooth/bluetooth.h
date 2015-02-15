/*
   BlueZ - Bluetooth protocol stack for Linux
   Copyright (c) 2000-2001, 2010-2013 The Linux Foundation. All rights reserved.

   Written 2000,2001 by Maxim Krasnyansky <maxk@qualcomm.com>

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 2 as
   published by the Free Software Foundation;

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
   OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT OF THIRD PARTY RIGHTS.
   IN NO EVENT SHALL THE COPYRIGHT HOLDER(S) AND AUTHOR(S) BE LIABLE FOR ANY
   CLAIM, OR ANY SPECIAL INDIRECT OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES
   WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
   ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
   OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

   ALL LIABILITY, INCLUDING LIABILITY FOR INFRINGEMENT OF ANY PATENTS,
   COPYRIGHTS, TRADEMARKS OR OTHER RIGHTS, RELATING TO USE OF THIS
   SOFTWARE IS DISCLAIMED.
*/

#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include <linux/types.h>
#include <asm/byteorder.h>
#include <linux/list.h>
#include <linux/poll.h>
#include <net/sock.h>

#ifndef AF_BLUETOOTH
#define AF_BLUETOOTH	31
#define PF_BLUETOOTH	AF_BLUETOOTH
#endif

/* Reserv for core and drivers use */
#define BT_SKB_RESERVE	8
#define BT_SKB_RESERVE_80211	32

#define BTPROTO_L2CAP	0
#define BTPROTO_HCI	1
#define BTPROTO_SCO	2
#define BTPROTO_RFCOMM	3
#define BTPROTO_BNEP	4
#define BTPROTO_CMTP	5
#define BTPROTO_HIDP	6
#define BTPROTO_AVDTP	7

#define SOL_HCI		0
#define SOL_L2CAP	6
#define SOL_SCO		17
#define SOL_RFCOMM	18

#define BT_SECURITY	4
struct bt_security {
	__u8 level;
	__u8 key_size;
};
#define BT_SECURITY_SDP		0
#define BT_SECURITY_LOW		1
#define BT_SECURITY_MEDIUM	2
#define BT_SECURITY_HIGH	3
#define BT_SECURITY_VERY_HIGH	4

#define BT_DEFER_SETUP	7
#define BT_FLUSHABLE	8

#define BT_POWER	9
struct bt_power {
	__u8 force_active;
};

#define BT_AMP_POLICY          10

/* Require BR/EDR (default policy)
 *   AMP controllers cannot be used
 *   Channel move requests from the remote device are denied
 *   If the L2CAP channel is currently using AMP, move the channel to BR/EDR
 */
#define BT_AMP_POLICY_REQUIRE_BR_EDR   0

/* Prefer BR/EDR
 *   Allow use of AMP controllers
 *   If the L2CAP channel is currently on AMP, move it to BR/EDR
 *   Channel move requests from the remote device are allowed
 */
#define BT_AMP_POLICY_PREFER_BR_EDR    1

/* Prefer AMP
 *   Allow use of AMP controllers
 *   If the L2CAP channel is currently on BR/EDR and AMP controller
 *     resources are available, initiate a channel move to AMP
 *   Channel move requests from the remote device are allowed
 *   If the L2CAP socket has not been connected yet, try to create
 *     and configure the channel directly on an AMP controller rather
 *     than BR/EDR
 */
#define BT_AMP_POLICY_PREFER_AMP       2

#define BT_LE_PARAMS	100

#define BT_LE_SCAN_WINDOW_MIN		0x0004
#define BT_LE_SCAN_WINDOW_MAX		0x4000
#define BT_LE_SCAN_WINDOW_DEF		0x0004

#define BT_LE_SCAN_INTERVAL_MIN		0x0004
#define BT_LE_SCAN_INTERVAL_MAX		0x4000
#define BT_LE_SCAN_INTERVAL_DEF		0x0008

#define BT_LE_CONN_INTERVAL_MIN		0x0006
#define BT_LE_CONN_INTERVAL_MAX		0x0C80
#define BT_LE_CONN_INTERVAL_MIN_DEF	0x0008
#define BT_LE_CONN_INTERVAL_MAX_DEF	0x0100

#define BT_LE_LATENCY_MAX		0x01F4
#define BT_LE_LATENCY_DEF		0x0000

#define BT_LE_SUP_TO_MIN		0x000A
#define BT_LE_SUP_TO_MAX		0x0C80
#define BT_LE_SUP_TO_DEFAULT		0X03E8

#define BT_INFO(fmt, arg...) printk(KERN_INFO "Bluetooth: " fmt "\n" , ## arg)
#define BT_ERR(fmt, arg...)  printk(KERN_ERR "%s: " fmt "\n" , __func__ , ## arg)
#define BT_DBG(fmt, arg...)  pr_debug("%s: " fmt "\n" , __func__ , ## arg)

/* Connection and socket states */
enum {
	BT_CONNECTED = 1, /* Equal to TCP_ESTABLISHED to make net code happy */
	BT_OPEN,
	BT_BOUND,
	BT_LISTEN,
	BT_CONNECT,
	BT_CONNECT2,
	BT_CONFIG,
	BT_DISCONN,
	BT_CLOSED
};

/* BD Address */
typedef struct {
	__u8 b[6];
} __packed bdaddr_t;

#define BDADDR_ANY   (&(bdaddr_t) {{0, 0, 0, 0, 0, 0} })
#define BDADDR_LOCAL (&(bdaddr_t) {{0, 0, 0, 0xff, 0xff, 0xff} })

/* Copy, swap, convert BD Address */
static inline int bacmp(bdaddr_t *ba1, bdaddr_t *ba2)
{
	return memcmp(ba1, ba2, sizeof(bdaddr_t));
}
static inline void bacpy(bdaddr_t *dst, bdaddr_t *src)
{
	memcpy(dst, src, sizeof(bdaddr_t));
}

void baswap(bdaddr_t *dst, bdaddr_t *src);
char *batostr(bdaddr_t *ba);
bdaddr_t *strtoba(char *str);

/* Common socket structures and functions */

#define bt_sk(__sk) ((struct bt_sock *) __sk)

struct bt_le_params {
	__u8  prohibit_remote_chg;
	__u8  filter_policy;
	__u16 scan_interval;
	__u16 scan_window;
	__u16 interval_min;
	__u16 interval_max;
	__u16 latency;
	__u16 supervision_timeout;
	__u16 min_ce_len;
	__u16 max_ce_len;
	__u16 conn_timeout;
};

struct bt_sock {
	struct sock sk;
	bdaddr_t    src;
	bdaddr_t    dst;
	struct list_head accept_q;
	struct sock *parent;
	u32 defer_setup;
	struct bt_le_params le_params;
};

struct bt_sock_list {
	struct hlist_head head;
	rwlock_t          lock;
};

int  bt_sock_register(int proto, const struct net_proto_family *ops);
int  bt_sock_unregister(int proto);
void bt_sock_link(struct bt_sock_list *l, struct sock *s);
void bt_sock_unlink(struct bt_sock_list *l, struct sock *s);
int  bt_sock_recvmsg(struct kiocb *iocb, struct socket *sock,
				struct msghdr *msg, size_t len, int flags);
int  bt_sock_stream_recvmsg(struct kiocb *iocb, struct socket *sock,
			struct msghdr *msg, size_t len, int flags);
uint bt_sock_poll(struct file *file, struct socket *sock, poll_table *wait);
int  bt_sock_ioctl(struct socket *sock, unsigned int cmd, unsigned long arg);
int  bt_sock_wait_state(struct sock *sk, int state, unsigned long timeo);

void bt_accept_enqueue(struct sock *parent, struct sock *sk);
void bt_accept_unlink(struct sock *sk);
struct sock *bt_accept_dequeue(struct sock *parent, struct socket *newsock);

/* Skb helpers */
struct bt_l2cap_control {
	__u8  frame_type;
	__u8  final;
	__u8  sar;
	__u8  super;
	__u16 reqseq;
	__u16 txseq;
	__u8  poll;
	__u8  fcs;
};

struct bt_skb_cb {
	__u8 pkt_type;
	__u8 incoming;
	__u16 expect;
	__u8 retries;
	__u8 force_active;
	unsigned short channel;
	struct bt_l2cap_control control;
};
#define bt_cb(skb) ((struct bt_skb_cb *)((skb)->cb))

static inline struct sk_buff *bt_skb_alloc(unsigned int len, gfp_t how)
{
	struct sk_buff *skb;

	skb = alloc_skb(len + BT_SKB_RESERVE, how);
	if (skb) {
		skb_reserve(skb, BT_SKB_RESERVE);
		bt_cb(skb)->incoming  = 0;
	}
	return skb;
}

static inline struct sk_buff *bt_skb_send_alloc(struct sock *sk,
					unsigned long len, int nb, int *err)
{
	struct sk_buff *skb;

	release_sock(sk);
	skb = sock_alloc_send_skb(sk, len + BT_SKB_RESERVE, nb, err);
	if (skb) {
		skb_reserve(skb, BT_SKB_RESERVE);
		bt_cb(skb)->incoming  = 0;
	}
	lock_sock(sk);

	if (!skb && *err)
		return NULL;

	*err = sock_error(sk);
	if (*err)
		goto out;

	if (sk->sk_shutdown) {
		*err = -ECONNRESET;
		goto out;
	}

	return skb;

out:
	kfree_skb(skb);
	return NULL;
}

int bt_err(__u16 code);

extern int hci_sock_init(void);
extern void hci_sock_cleanup(void);

extern int bt_sysfs_init(void);
extern void bt_sysfs_cleanup(void);

extern struct dentry *bt_debugfs;

int l2cap_init(void);
void l2cap_exit(void);

int sco_init(void);
void sco_exit(void);

#endif /* __BLUETOOTH_H */
