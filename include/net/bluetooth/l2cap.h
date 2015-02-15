/*
   BlueZ - Bluetooth protocol stack for Linux
   Copyright (c) 2000-2001, 2010-2012 The Linux Foundation.  All rights reserved.
   Copyright (C) 2009-2010 Gustavo F. Padovan <gustavo@padovan.org>
   Copyright (C) 2010 Google Inc.

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

#ifndef __L2CAP_H
#define __L2CAP_H

/* L2CAP defaults */
#define L2CAP_DEFAULT_MTU		672
#define L2CAP_DEFAULT_MIN_MTU		48
#define L2CAP_DEFAULT_MAX_SDU_SIZE	0xffff
#define L2CAP_DEFAULT_FLUSH_TO		0xffff
#define L2CAP_MAX_FLUSH_TO		0x7ff
#define L2CAP_DEFAULT_TX_WINDOW		63
#define L2CAP_DEFAULT_MAX_TX		3
#define L2CAP_DEFAULT_RETRANS_TO	2000    /* 2 seconds */
#define L2CAP_DEFAULT_MONITOR_TO	12000   /* 12 seconds */
#define L2CAP_DEFAULT_MAX_PDU_SIZE	1482    /* Sized for AMP or BR/EDR */
#define L2CAP_DEFAULT_ACK_TO		200
#define L2CAP_BREDR_MAX_PAYLOAD		1019    /* 3-DH5 packet */
#define L2CAP_MAX_ERTM_QUEUED		5
#define L2CAP_MIN_ERTM_QUEUED		2

#define L2CAP_A2MP_DEFAULT_MTU		670

#define L2CAP_TX_WIN_MAX_ENHANCED	0x3f
#define L2CAP_TX_WIN_MAX_EXTENDED	0x3fff
#define L2CAP_LE_DEFAULT_MTU		23

#define L2CAP_CONN_TIMEOUT	(40000) /* 40 seconds */
#define L2CAP_INFO_TIMEOUT	(4000)  /*  4 seconds */
#define L2CAP_MOVE_TIMEOUT		(4*HZ)  /*  4 seconds */
#define L2CAP_MOVE_ERTX_TIMEOUT		(60*HZ) /* 60 seconds */

/* L2CAP socket address */
struct sockaddr_l2 {
	sa_family_t	l2_family;
	__le16		l2_psm;
	bdaddr_t	l2_bdaddr;
	__le16		l2_cid;
};

/* L2CAP socket options */
#define L2CAP_OPTIONS	0x01
struct l2cap_options {
	__u16 omtu;
	__u16 imtu;
	__u16 flush_to;
	__u8  mode;
	__u8  fcs;
	__u8  max_tx;
	__u16 txwin_size;
};

#define L2CAP_CONNINFO	0x02
struct l2cap_conninfo {
	__u16 hci_handle;
	__u8  dev_class[3];
};

#define L2CAP_LM	0x03
#define L2CAP_LM_MASTER		0x0001
#define L2CAP_LM_AUTH		0x0002
#define L2CAP_LM_ENCRYPT	0x0004
#define L2CAP_LM_TRUSTED	0x0008
#define L2CAP_LM_RELIABLE	0x0010
#define L2CAP_LM_SECURE		0x0020
#define L2CAP_LM_FLUSHABLE	0x0040

/* L2CAP command codes */
#define L2CAP_COMMAND_REJ		0x01
#define L2CAP_CONN_REQ			0x02
#define L2CAP_CONN_RSP			0x03
#define L2CAP_CONF_REQ			0x04
#define L2CAP_CONF_RSP			0x05
#define L2CAP_DISCONN_REQ		0x06
#define L2CAP_DISCONN_RSP		0x07
#define L2CAP_ECHO_REQ			0x08
#define L2CAP_ECHO_RSP			0x09
#define L2CAP_INFO_REQ			0x0a
#define L2CAP_INFO_RSP			0x0b
#define L2CAP_CREATE_CHAN_REQ	0x0c
#define L2CAP_CREATE_CHAN_RSP	0x0d
#define L2CAP_MOVE_CHAN_REQ		0x0e
#define L2CAP_MOVE_CHAN_RSP		0x0f
#define L2CAP_MOVE_CHAN_CFM		0x10
#define L2CAP_MOVE_CHAN_CFM_RSP	0x11
#define L2CAP_CONN_PARAM_UPDATE_REQ	0x12
#define L2CAP_CONN_PARAM_UPDATE_RSP	0x13

/* L2CAP feature mask */
#define L2CAP_FEAT_FLOWCTL	0x00000001
#define L2CAP_FEAT_RETRANS	0x00000002
#define L2CAP_FEAT_ERTM		0x00000008
#define L2CAP_FEAT_STREAMING	0x00000010
#define L2CAP_FEAT_FCS		0x00000020
#define L2CAP_FEAT_FIXED_CHAN	0x00000080
#define L2CAP_FEAT_EXT_WINDOW	0x00000100
#define L2CAP_FEAT_UCD		0x00000200

/* L2CAP checksum option */
#define L2CAP_FCS_NONE		0x00
#define L2CAP_FCS_CRC16		0x01

/* L2CAP fixed channels */
#define L2CAP_FC_L2CAP		0x02
#define L2CAP_FC_A2MP		0x08

/* L2CAP Control Field */
#define L2CAP_CTRL_SAR               0xC000
#define L2CAP_CTRL_REQSEQ            0x3F00
#define L2CAP_CTRL_TXSEQ             0x007E
#define L2CAP_CTRL_FINAL             0x0080
#define L2CAP_CTRL_POLL              0x0010
#define L2CAP_CTRL_SUPERVISE         0x000C
#define L2CAP_CTRL_FRAME_TYPE        0x0001 /* I- or S-Frame */

#define L2CAP_CTRL_TXSEQ_SHIFT      1
#define L2CAP_CTRL_SUPERVISE_SHIFT  2
#define L2CAP_CTRL_POLL_SHIFT       4
#define L2CAP_CTRL_FINAL_SHIFT      7
#define L2CAP_CTRL_REQSEQ_SHIFT     8
#define L2CAP_CTRL_SAR_SHIFT       14

#define L2CAP_EXT_CTRL_SAR           0x00030000
#define L2CAP_EXT_CTRL_REQSEQ        0x0000FFFC
#define L2CAP_EXT_CTRL_TXSEQ         0xFFFC0000
#define L2CAP_EXT_CTRL_FINAL         0x00000002
#define L2CAP_EXT_CTRL_POLL          0x00040000
#define L2CAP_EXT_CTRL_SUPERVISE     0x00030000
#define L2CAP_EXT_CTRL_FRAME_TYPE    0x00000001 /* I- or S-Frame */

#define L2CAP_EXT_CTRL_FINAL_SHIFT      1
#define L2CAP_EXT_CTRL_REQSEQ_SHIFT     2
#define L2CAP_EXT_CTRL_SAR_SHIFT       16
#define L2CAP_EXT_CTRL_SUPERVISE_SHIFT 16
#define L2CAP_EXT_CTRL_POLL_SHIFT      18
#define L2CAP_EXT_CTRL_TXSEQ_SHIFT     18

/* L2CAP Supervisory Frame Types */
#define L2CAP_SFRAME_RR            0x00
#define L2CAP_SFRAME_REJ           0x01
#define L2CAP_SFRAME_RNR           0x02
#define L2CAP_SFRAME_SREJ          0x03

/* L2CAP Segmentation and Reassembly */
#define L2CAP_SAR_UNSEGMENTED      0x00
#define L2CAP_SAR_START            0x01
#define L2CAP_SAR_END              0x02
#define L2CAP_SAR_CONTINUE         0x03

/* L2CAP ERTM / Streaming extra field lengths */
#define L2CAP_SDULEN_SIZE       2
#define L2CAP_FCS_SIZE          2

/* L2CAP structures */
struct l2cap_hdr {
	__le16     len;
	__le16     cid;
} __packed;
#define L2CAP_HDR_SIZE		4
#define L2CAP_ENHANCED_HDR_SIZE	6
#define L2CAP_EXTENDED_HDR_SIZE	8

struct l2cap_cmd_hdr {
	__u8       code;
	__u8       ident;
	__le16     len;
} __packed;
#define L2CAP_CMD_HDR_SIZE	4

struct l2cap_cmd_rej {
	__le16     reason;
} __packed;

struct l2cap_conn_req {
	__le16     psm;
	__le16     scid;
} __packed;

struct l2cap_conn_rsp {
	__le16     dcid;
	__le16     scid;
	__le16     result;
	__le16     status;
} __packed;

/* channel indentifier */
#define L2CAP_CID_SIGNALING	0x0001
#define L2CAP_CID_CONN_LESS	0x0002
#define L2CAP_CID_A2MP		0x0003
#define L2CAP_CID_LE_DATA	0x0004
#define L2CAP_CID_LE_SIGNALING	0x0005
#define L2CAP_CID_SMP		0x0006
#define L2CAP_CID_DYN_START	0x0040
#define L2CAP_CID_DYN_END	0xffff

/* connect result */
#define L2CAP_CR_SUCCESS	0x0000
#define L2CAP_CR_PEND		0x0001
#define L2CAP_CR_BAD_PSM	0x0002
#define L2CAP_CR_SEC_BLOCK	0x0003
#define L2CAP_CR_NO_MEM		0x0004

/* connect status */
#define L2CAP_CS_NO_INFO	0x0000
#define L2CAP_CS_AUTHEN_PEND	0x0001
#define L2CAP_CS_AUTHOR_PEND	0x0002

struct l2cap_conf_req {
	__le16     dcid;
	__le16     flags;
	__u8       data[0];
} __packed;

struct l2cap_conf_rsp {
	__le16     scid;
	__le16     flags;
	__le16     result;
	__u8       data[0];
} __packed;

#define L2CAP_CONF_SUCCESS	0x0000
#define L2CAP_CONF_UNACCEPT	0x0001
#define L2CAP_CONF_REJECT	0x0002
#define L2CAP_CONF_UNKNOWN	0x0003
#define L2CAP_CONF_PENDING	0x0004
#define L2CAP_CONF_FLOW_SPEC_REJECT	0x0005

struct l2cap_conf_opt {
	__u8       type;
	__u8       len;
	__u8       val[0];
} __packed;
#define L2CAP_CONF_OPT_SIZE	2

#define L2CAP_CONF_HINT		0x80
#define L2CAP_CONF_MASK		0x7f

#define L2CAP_CONF_MTU		0x01
#define L2CAP_CONF_FLUSH_TO	0x02
#define L2CAP_CONF_QOS		0x03
#define L2CAP_CONF_RFC		0x04
#define L2CAP_CONF_FCS		0x05
#define L2CAP_CONF_EXT_FS	0x06
#define L2CAP_CONF_EXT_WINDOW	0x07

/* QOS Service type */
#define L2CAP_SERVICE_NO_TRAFFIC		0x00
#define L2CAP_SERVICE_BEST_EFFORT		0x01
#define L2CAP_SERVICE_GUARANTEED		0x02

#define L2CAP_CONF_MAX_SIZE	22

struct l2cap_conf_rfc {
	__u8       mode;
	__u8       txwin_size;
	__u8       max_transmit;
	__le16     retrans_timeout;
	__le16     monitor_timeout;
	__le16     max_pdu_size;
} __packed;

struct l2cap_conf_ext_fs {
	__u8       id;
	__u8       type;
	__le16     max_sdu;
	__le32     sdu_arr_time;
	__le32     acc_latency;
	__le32     flush_to;
} __packed;

struct l2cap_conf_prm {
	__u8       fcs;
	__le32     flush_to;
};

#define L2CAP_MODE_BASIC	0x00
#define L2CAP_MODE_RETRANS	0x01
#define L2CAP_MODE_FLOWCTL	0x02
#define L2CAP_MODE_ERTM		0x03
#define L2CAP_MODE_STREAMING	0x04

struct l2cap_disconn_req {
	__le16     dcid;
	__le16     scid;
} __packed;

struct l2cap_disconn_rsp {
	__le16     dcid;
	__le16     scid;
} __packed;

struct l2cap_info_req {
	__le16      type;
} __packed;

struct l2cap_info_rsp {
	__le16      type;
	__le16      result;
	__u8        data[0];
} __packed;

struct l2cap_create_chan_req {
	__le16      psm;
	__le16      scid;
	__u8        amp_id;
} __attribute__ ((packed));

struct l2cap_create_chan_rsp {
	__le16      dcid;
	__le16      scid;
	__le16      result;
	__le16      status;
} __attribute__ ((packed));

#define L2CAP_CREATE_CHAN_SUCCESS				(0x0000)
#define L2CAP_CREATE_CHAN_PENDING				(0x0001)
#define L2CAP_CREATE_CHAN_REFUSED_PSM			(0x0002)
#define L2CAP_CREATE_CHAN_REFUSED_SECURITY		(0x0003)
#define L2CAP_CREATE_CHAN_REFUSED_RESOURCES		(0x0004)
#define L2CAP_CREATE_CHAN_REFUSED_CONTROLLER	(0x0005)

#define L2CAP_CREATE_CHAN_STATUS_NONE			(0x0000)
#define L2CAP_CREATE_CHAN_STATUS_AUTHENTICATION	(0x0001)
#define L2CAP_CREATE_CHAN_STATUS_AUTHORIZATION	(0x0002)

struct l2cap_move_chan_req {
	__le16      icid;
	__u8        dest_amp_id;
} __attribute__ ((packed));

struct l2cap_move_chan_rsp {
	__le16      icid;
	__le16      result;
} __attribute__ ((packed));

#define L2CAP_MOVE_CHAN_SUCCESS				(0x0000)
#define L2CAP_MOVE_CHAN_PENDING				(0x0001)
#define L2CAP_MOVE_CHAN_REFUSED_CONTROLLER	(0x0002)
#define L2CAP_MOVE_CHAN_REFUSED_SAME_ID		(0x0003)
#define L2CAP_MOVE_CHAN_REFUSED_CONFIG		(0x0004)
#define L2CAP_MOVE_CHAN_REFUSED_COLLISION	(0x0005)
#define L2CAP_MOVE_CHAN_REFUSED_NOT_ALLOWED	(0x0006)

struct l2cap_move_chan_cfm {
	__le16      icid;
	__le16      result;
} __attribute__ ((packed));

#define L2CAP_MOVE_CHAN_CONFIRMED	(0x0000)
#define L2CAP_MOVE_CHAN_UNCONFIRMED	(0x0001)

struct l2cap_move_chan_cfm_rsp {
	__le16      icid;
} __attribute__ ((packed));

struct l2cap_amp_signal_work {
	struct work_struct work;
	struct l2cap_cmd_hdr cmd;
	struct l2cap_conn *conn;
	struct sk_buff *skb;
	u8 *data;
};

struct l2cap_resegment_work {
	struct work_struct work;
	struct sock *sk;
};

struct l2cap_logical_link_work {
	struct work_struct work;
	struct hci_chan *chan;
	u8 status;
};

/* info type */
#define L2CAP_IT_CL_MTU     0x0001
#define L2CAP_IT_FEAT_MASK  0x0002
#define L2CAP_IT_FIXED_CHAN 0x0003

/* info result */
#define L2CAP_IR_SUCCESS    0x0000
#define L2CAP_IR_NOTSUPP    0x0001

struct l2cap_conn_param_update_req {
	__le16      min;
	__le16      max;
	__le16      latency;
	__le16      to_multiplier;
} __packed;

struct l2cap_conn_param_update_rsp {
	__le16      result;
} __packed;

/* Connection Parameters result */
#define L2CAP_CONN_PARAM_ACCEPTED	0x0000
#define L2CAP_CONN_PARAM_REJECTED	0x0001

/* ----- L2CAP connections ----- */
struct l2cap_chan_list {
	struct sock	*head;
	rwlock_t	lock;
};

struct l2cap_conn {
	struct hci_conn	*hcon;

	bdaddr_t	*dst;
	bdaddr_t	*src;

	unsigned int	mtu;

	__u32		feat_mask;
	__u8		fc_mask;
	struct amp_mgr *mgr;

	__u8		info_state;
	__u8		info_ident;

	struct timer_list info_timer;

	spinlock_t	lock;

	struct sk_buff *rx_skb;
	__u32		rx_len;
	__u8		tx_ident;

	__u8		disc_reason;

	struct l2cap_chan_list chan_list;
};

struct sock_del_list {
	struct sock *sk;
	struct list_head list;
};

#define L2CAP_INFO_CL_MTU_REQ_SENT	0x01
#define L2CAP_INFO_FEAT_MASK_REQ_SENT	0x04
#define L2CAP_INFO_FEAT_MASK_REQ_DONE	0x08

/* ----- L2CAP channel and socket info ----- */
#define l2cap_pi(sk) ((struct l2cap_pinfo *) sk)
#define TX_QUEUE(sk) (&l2cap_pi(sk)->tx_queue)
#define SREJ_QUEUE(sk) (&l2cap_pi(sk)->srej_queue)

struct l2cap_seq_list {
	__u16 head;
	__u16 tail;
	__u16 size;
	__u16 mask;
	__u16 *list;
};

struct l2cap_pinfo {
	struct bt_sock	bt;
	__le16		psm;
	__u16		dcid;
	__u16		scid;

	__u16		imtu;
	__u16		omtu;
	__u16		flush_to;
	__u8		mode;
	__u8		fixed_channel;
	__u8		num_conf_req;
	__u8		num_conf_rsp;
	__u8		incoming;

	__u8		fcs;
	__u8		sec_level;
	__u8		role_switch;
	__u8		force_reliable;
	__u8		flushable;
	__u8		force_active;

	__u8		conf_req[64];
	__u8		conf_len;
	__u8		conf_ident;
	__u16		conf_state;
	__u8		conn_state;
	__u8		tx_state;
	__u8		rx_state;
	__u8		reconf_state;

	__u8		amp_id;
	__u8		amp_move_id;
	__u8		amp_move_state;
	__u8		amp_move_role;
	__u8		amp_move_cmd_ident;
	__u16		amp_move_reqseq;
	__u16		amp_move_event;

	__u16		next_tx_seq;
	__u16		expected_ack_seq;
	__u16		expected_tx_seq;
	__u16		buffer_seq;
	__u16		srej_save_reqseq;
	__u16		last_acked_seq;
	__u32		frames_sent;
	__u16		unacked_frames;
	__u8		retry_count;
	__u16		srej_queue_next;
	__u16		sdu_len;
	struct sk_buff	*sdu;
	struct sk_buff	*sdu_last_frag;
	atomic_t	ertm_queued;

	__u8		ident;

	__u16		tx_win;
	__u16		tx_win_max;
	__u16		ack_win;
	__u8		max_tx;
	__u8		amp_pref;
	__u16		remote_tx_win;
	__u8		remote_max_tx;
	__u8		extended_control;
	__u16		retrans_timeout;
	__u16		monitor_timeout;
	__u16		remote_mps;
	__u16		mps;

	__le16		sport;

	struct delayed_work	retrans_work;
	struct delayed_work	monitor_work;
	struct delayed_work	ack_work;
	struct work_struct	tx_work;
	struct sk_buff_head	tx_queue;
	struct sk_buff_head	srej_queue;
	struct l2cap_seq_list srej_list;
	struct l2cap_seq_list retrans_list;
	struct hci_conn	*ampcon;
	struct hci_chan	*ampchan;
	struct l2cap_conn	*conn;
	struct l2cap_conf_prm local_conf;
	struct l2cap_conf_prm remote_conf;
	struct l2cap_conf_ext_fs local_fs;
	struct l2cap_conf_ext_fs remote_fs;
	struct sock		*next_c;
	struct sock		*prev_c;
};

#define L2CAP_CONF_REQ_SENT       0x0001
#define L2CAP_CONF_INPUT_DONE     0x0002
#define L2CAP_CONF_OUTPUT_DONE    0x0004
#define L2CAP_CONF_MTU_DONE       0x0008
#define L2CAP_CONF_MODE_DONE      0x0010
#define L2CAP_CONF_CONNECT_PEND   0x0020
#define L2CAP_CONF_NO_FCS_RECV    0x0040
#define L2CAP_CONF_STATE2_DEVICE  0x0080
#define L2CAP_CONF_EXT_WIN_RECV   0x0100
#define L2CAP_CONF_LOCKSTEP       0x0200
#define L2CAP_CONF_LOCKSTEP_PEND  0x0400
#define L2CAP_CONF_PEND_SENT      0x0800
#define L2CAP_CONF_EFS_RECV       0x1000

#define L2CAP_CONF_MAX_CONF_REQ 2
#define L2CAP_CONF_MAX_CONF_RSP 2

#define L2CAP_RECONF_NONE          0x00
#define L2CAP_RECONF_INT           0x01
#define L2CAP_RECONF_ACC           0x02

#define L2CAP_CONN_SREJ_ACT        0x01
#define L2CAP_CONN_REJ_ACT         0x02
#define L2CAP_CONN_REMOTE_BUSY     0x04
#define L2CAP_CONN_LOCAL_BUSY      0x08
#define L2CAP_CONN_SEND_FBIT       0x10
#define L2CAP_CONN_SENT_RNR        0x20

#define L2CAP_SEQ_LIST_CLEAR       0xFFFF
#define L2CAP_SEQ_LIST_TAIL        0x8000

#define L2CAP_ERTM_TX_STATE_XMIT          0x01
#define L2CAP_ERTM_TX_STATE_WAIT_F        0x02

#define L2CAP_ERTM_RX_STATE_RECV                    0x01
#define L2CAP_ERTM_RX_STATE_SREJ_SENT               0x02
#define L2CAP_ERTM_RX_STATE_AMP_MOVE                0x03
#define L2CAP_ERTM_RX_STATE_WAIT_P_FLAG             0x04
#define L2CAP_ERTM_RX_STATE_WAIT_P_FLAG_RECONFIGURE 0x05
#define L2CAP_ERTM_RX_STATE_WAIT_F_FLAG             0x06

#define L2CAP_ERTM_TXSEQ_EXPECTED        0x00
#define L2CAP_ERTM_TXSEQ_EXPECTED_SREJ   0x01
#define L2CAP_ERTM_TXSEQ_UNEXPECTED      0x02
#define L2CAP_ERTM_TXSEQ_UNEXPECTED_SREJ 0x03
#define L2CAP_ERTM_TXSEQ_DUPLICATE       0x04
#define L2CAP_ERTM_TXSEQ_DUPLICATE_SREJ  0x05
#define L2CAP_ERTM_TXSEQ_INVALID         0x06
#define L2CAP_ERTM_TXSEQ_INVALID_IGNORE  0x07

#define L2CAP_ERTM_EVENT_DATA_REQUEST          0x01
#define L2CAP_ERTM_EVENT_LOCAL_BUSY_DETECTED   0x02
#define L2CAP_ERTM_EVENT_LOCAL_BUSY_CLEAR      0x03
#define L2CAP_ERTM_EVENT_RECV_REQSEQ_AND_FBIT  0x04
#define L2CAP_ERTM_EVENT_RECV_FBIT             0x05
#define L2CAP_ERTM_EVENT_RETRANS_TIMER_EXPIRES 0x06
#define L2CAP_ERTM_EVENT_MONITOR_TIMER_EXPIRES 0x07
#define L2CAP_ERTM_EVENT_EXPLICIT_POLL         0x08
#define L2CAP_ERTM_EVENT_RECV_IFRAME           0x09
#define L2CAP_ERTM_EVENT_RECV_RR               0x0a
#define L2CAP_ERTM_EVENT_RECV_REJ              0x0b
#define L2CAP_ERTM_EVENT_RECV_RNR              0x0c
#define L2CAP_ERTM_EVENT_RECV_SREJ             0x0d
#define L2CAP_ERTM_EVENT_RECV_FRAME            0x0e

#define L2CAP_AMP_MOVE_NONE      0
#define L2CAP_AMP_MOVE_INITIATOR 1
#define L2CAP_AMP_MOVE_RESPONDER 2

#define L2CAP_AMP_STATE_STABLE			0
#define L2CAP_AMP_STATE_WAIT_CREATE		1
#define L2CAP_AMP_STATE_WAIT_CREATE_RSP		2
#define L2CAP_AMP_STATE_WAIT_MOVE		3
#define L2CAP_AMP_STATE_WAIT_MOVE_RSP		4
#define L2CAP_AMP_STATE_WAIT_MOVE_RSP_SUCCESS	5
#define L2CAP_AMP_STATE_WAIT_MOVE_CONFIRM	6
#define L2CAP_AMP_STATE_WAIT_MOVE_CONFIRM_RSP	7
#define L2CAP_AMP_STATE_WAIT_LOGICAL_COMPLETE	8
#define L2CAP_AMP_STATE_WAIT_LOGICAL_CONFIRM	9
#define L2CAP_AMP_STATE_WAIT_LOCAL_BUSY		10
#define L2CAP_AMP_STATE_WAIT_PREPARE		11
#define L2CAP_AMP_STATE_RESEGMENT		12

#define L2CAP_ATT_ERROR				0x01
#define L2CAP_ATT_MTU_REQ			0x02
#define L2CAP_ATT_MTU_RSP			0x03
#define L2CAP_ATT_RESPONSE_BIT			0x01
#define L2CAP_ATT_INDICATE			0x1D
#define L2CAP_ATT_CONFIRM			0x1E
#define L2CAP_ATT_NOT_SUPPORTED			0x06

#define __delta_seq(x, y, pi) ((x) >= (y) ? (x) - (y) : \
				(pi)->tx_win_max + 1 - (y) + (x))
#define __next_seq(x, pi) ((x + 1) & ((pi)->tx_win_max))

extern bool disable_ertm;
extern const struct proto_ops l2cap_sock_ops;
extern struct bt_sock_list l2cap_sk_list;

int l2cap_init_sockets(void);
void l2cap_cleanup_sockets(void);

u8 l2cap_get_ident(struct l2cap_conn *conn);
void l2cap_send_cmd(struct l2cap_conn *conn, u8 ident, u8 code, u16 len, void *data);
int l2cap_build_conf_req(struct sock *sk, void *data);
int __l2cap_wait_ack(struct sock *sk);

struct sk_buff *l2cap_create_connless_pdu(struct sock *sk, struct msghdr *msg, size_t len);
struct sk_buff *l2cap_create_basic_pdu(struct sock *sk, struct msghdr *msg, size_t len);
struct sk_buff *l2cap_create_iframe_pdu(struct sock *sk, struct msghdr *msg,
				size_t len, u16 sdulen, int reseg);
int l2cap_segment_sdu(struct sock *sk, struct sk_buff_head* seg_queue,
			struct msghdr *msg, size_t len, int reseg);
int l2cap_resegment_queue(struct sock *sk, struct sk_buff_head *queue);
void l2cap_do_send(struct sock *sk, struct sk_buff *skb);
void l2cap_streaming_send(struct sock *sk);
int l2cap_ertm_send(struct sock *sk);
int l2cap_strm_tx(struct sock *sk, struct sk_buff_head *skbs);
int l2cap_ertm_tx(struct sock *sk, struct bt_l2cap_control *control,
			struct sk_buff_head *skbs, u8 event);

int l2cap_sock_le_params_valid(struct bt_le_params *le_params);
void l2cap_sock_set_timer(struct sock *sk, long timeout);
void l2cap_sock_clear_timer(struct sock *sk);
void __l2cap_sock_close(struct sock *sk, int reason);
void l2cap_sock_kill(struct sock *sk);
void l2cap_sock_init(struct sock *sk, struct sock *parent);
struct sock *l2cap_sock_alloc(struct net *net, struct socket *sock,
							int proto, gfp_t prio);
struct sock *l2cap_find_sock_by_fixed_cid_and_dir(__le16 cid, bdaddr_t *src,
						bdaddr_t *dst, int server);
void l2cap_send_disconn_req(struct l2cap_conn *conn, struct sock *sk, int err);
void l2cap_chan_del(struct sock *sk, int err);
int l2cap_do_connect(struct sock *sk);
int l2cap_data_channel(struct sock *sk, struct sk_buff *skb);
void l2cap_amp_move_init(struct sock *sk);
void l2cap_ertm_destruct(struct sock *sk);
void l2cap_ertm_shutdown(struct sock *sk);
void l2cap_ertm_recv_done(struct sock *sk);

void l2cap_fixed_channel_config(struct sock *sk, struct l2cap_options *opt);

void l2cap_recv_deferred_frame(struct sock *sk, struct sk_buff *skb);

void l2cap_amp_physical_complete(int result, u8 remote_id, u8 local_id,
				struct sock *sk);

void l2cap_amp_logical_complete(int result, struct hci_conn *ampcon,
				struct hci_chan *ampchan, struct sock *sk);

void l2cap_amp_logical_destroyed(struct hci_conn *ampcon);

#endif /* __L2CAP_H */
