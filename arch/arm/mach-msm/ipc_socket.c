/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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
#include <linux/types.h>
#include <linux/net.h>
#include <linux/socket.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/poll.h>
#include <linux/fcntl.h>
#include <linux/gfp.h>
#include <linux/msm_ipc.h>
#include <linux/sched.h>
#include <linux/thread_info.h>
#include <linux/qmi_encdec.h>

#include <asm/string.h>
#include <asm/atomic.h>

#include <net/sock.h>

#include <mach/msm_ipc_router.h>
#include <mach/msm_ipc_logging.h>

#include "ipc_router.h"
#include "msm_ipc_router_security.h"

#define msm_ipc_sk(sk) ((struct msm_ipc_sock *)(sk))
#define msm_ipc_sk_port(sk) ((struct msm_ipc_port *)(msm_ipc_sk(sk)->port))
#define REQ_RESP_IPC_LOG_PAGES 5
#define IND_IPC_LOG_PAGES 5
#define IPC_SEND 1
#define IPC_RECV 2
#define IPC_REQ_RESP_LOG(level, buf...) \
do { \
	if (ipc_req_resp_log_txt) { \
		ipc_log_string(ipc_req_resp_log_txt, buf); \
	} \
} while (0) \

#define IPC_IND_LOG(level, buf...) \
do { \
	if (ipc_ind_log_txt) { \
		ipc_log_string(ipc_ind_log_txt, buf); \
	} \
} while (0) \

#ifndef SIZE_MAX
#define SIZE_MAX ((size_t)-1)
#endif

static int sockets_enabled;
static struct proto msm_ipc_proto;
static const struct proto_ops msm_ipc_proto_ops;
static void *ipc_req_resp_log_txt;
static void *ipc_ind_log_txt;

/**
 * msm_ipc_router_ipc_log() - Pass log data to IPC logging framework
 * @tran:	Identifies the data to be a receive or send.
 * @ipc_buf:	Buffer to extract the log data.
 * @port_ptr:	IPC Router port corresponding to the current log data.
 *
 * This function builds the data the would be passed on to the IPC logging
 * framework. The data that would be passed corresponds to the information
 * that is exchanged between the IPC Router and user space modules during
 * request/response/indication transactions.
 */

static void msm_ipc_router_ipc_log(uint8_t tran,
			struct sk_buff *ipc_buf, struct msm_ipc_port *port_ptr)
{
	struct qmi_header *hdr = (struct qmi_header *)ipc_buf->data;

	/*
	 * IPC Logging format is as below:-
	 * <Name>(Name of the User Space Process):
	 * <PID> (PID of the user space process) :
	 * <TID> (TID of the user space thread)  :
	 * <User Space Module>(CLNT or  SERV)    :
	 * <Opertaion Type> (Transmit)		 :
	 * <Control Flag> (Req/Resp/Ind)	 :
	 * <Transaction ID>			 :
	 * <Message ID>				 :
	 * <Message Length>			 :
	 */
	if (ipc_req_resp_log_txt &&
		(((uint8_t) hdr->cntl_flag == QMI_REQUEST_CONTROL_FLAG) ||
		((uint8_t) hdr->cntl_flag == QMI_RESPONSE_CONTROL_FLAG)) &&
		(port_ptr->type == CLIENT_PORT ||
					port_ptr->type == SERVER_PORT)) {
		IPC_REQ_RESP_LOG(KERN_DEBUG,
			"%s %d %d %s %s CF:%x TI:%x MI:%x ML:%x",
			current->comm, current->tgid, current->pid,
			(port_ptr->type == CLIENT_PORT ? "QCCI" : "QCSI"),
			(tran == IPC_RECV ? "RX" :
			(tran == IPC_SEND ? "TX" : "ERR")),
			(uint8_t)hdr->cntl_flag, hdr->txn_id, hdr->msg_id,
			hdr->msg_len);
	} else if (ipc_ind_log_txt &&
		((uint8_t)hdr->cntl_flag == QMI_INDICATION_CONTROL_FLAG) &&
		(port_ptr->type == CLIENT_PORT ||
					port_ptr->type == SERVER_PORT)) {
		IPC_IND_LOG(KERN_DEBUG,
			"%s %d %d %s %s CF:%x TI:%x MI:%x ML:%x",
			current->comm, current->tgid, current->pid,
			(port_ptr->type == CLIENT_PORT ? "QCCI" : "QCSI"),
			(tran == IPC_RECV ? "RX" :
			(tran == IPC_SEND ? "TX" : "ERR")),
			(uint8_t)hdr->cntl_flag, hdr->txn_id, hdr->msg_id,
			hdr->msg_len);
	}
}

static struct sk_buff_head *msm_ipc_router_build_msg(unsigned int num_sect,
					  struct iovec const *msg_sect,
					  size_t total_len)
{
	struct sk_buff_head *msg_head;
	struct sk_buff *msg;
	int i, copied, first = 1;
	int data_size = 0, request_size, offset;
	void *data;
	int last = 0;
	int align_size;

	for (i = 0; i < num_sect; i++)
		data_size += msg_sect[i].iov_len;

	if (!data_size)
		return NULL;
	align_size = ALIGN_SIZE(data_size);

	msg_head = kmalloc(sizeof(struct sk_buff_head), GFP_KERNEL);
	if (!msg_head) {
		pr_err("%s: cannot allocate skb_head\n", __func__);
		return NULL;
	}
	skb_queue_head_init(msg_head);

	for (copied = 1, i = 0; copied && (i < num_sect); i++) {
		data_size = msg_sect[i].iov_len;
		offset = 0;
		if (i == (num_sect - 1))
			last = 1;
		while (offset != msg_sect[i].iov_len) {
			request_size = data_size;
			if (first)
				request_size += IPC_ROUTER_HDR_SIZE;
			if (last)
				request_size += align_size;

			msg = alloc_skb(request_size, GFP_KERNEL);
			if (!msg) {
				if (request_size <= (PAGE_SIZE/2)) {
					pr_err("%s: cannot allocated skb\n",
						__func__);
					goto msg_build_failure;
				}
				data_size = data_size / 2;
				last = 0;
				continue;
			}

			if (first) {
				skb_reserve(msg, IPC_ROUTER_HDR_SIZE);
				first = 0;
			}

			data = skb_put(msg, data_size);
			copied = !copy_from_user(msg->data,
					msg_sect[i].iov_base + offset,
					data_size);
			if (!copied) {
				pr_err("%s: copy_from_user failed\n",
					__func__);
				kfree_skb(msg);
				goto msg_build_failure;
			}
			skb_queue_tail(msg_head, msg);
			offset += data_size;
			data_size = msg_sect[i].iov_len - offset;
			if (i == (num_sect - 1))
				last = 1;
		}
	}
	return msg_head;

msg_build_failure:
	while (!skb_queue_empty(msg_head)) {
		msg = skb_dequeue(msg_head);
		kfree_skb(msg);
	}
	kfree(msg_head);
	return NULL;
}

static int msm_ipc_router_extract_msg(struct msghdr *m,
				      struct rr_packet *pkt)
{
	struct sockaddr_msm_ipc *addr;
	struct rr_header_v1 *hdr;
	struct sk_buff *temp;
	union rr_control_msg *ctl_msg;
	int offset = 0, data_len = 0, copy_len;

	if (!m || !pkt) {
		pr_err("%s: Invalid pointers passed\n", __func__);
		return -EINVAL;
	}
	addr = (struct sockaddr_msm_ipc *)m->msg_name;

	hdr = &(pkt->hdr);
	if (addr && (hdr->type == IPC_ROUTER_CTRL_CMD_RESUME_TX)) {
		temp = skb_peek(pkt->pkt_fragment_q);
		ctl_msg = (union rr_control_msg *)(temp->data);
		addr->family = AF_MSM_IPC;
		addr->address.addrtype = MSM_IPC_ADDR_ID;
		addr->address.addr.port_addr.node_id = ctl_msg->cli.node_id;
		addr->address.addr.port_addr.port_id = ctl_msg->cli.port_id;
		m->msg_namelen = sizeof(struct sockaddr_msm_ipc);
		return offset;
	}
	if (addr && (hdr->type == IPC_ROUTER_CTRL_CMD_DATA)) {
		addr->family = AF_MSM_IPC;
		addr->address.addrtype = MSM_IPC_ADDR_ID;
		addr->address.addr.port_addr.node_id = hdr->src_node_id;
		addr->address.addr.port_addr.port_id = hdr->src_port_id;
		m->msg_namelen = sizeof(struct sockaddr_msm_ipc);
	}

	data_len = hdr->size;
	skb_queue_walk(pkt->pkt_fragment_q, temp) {
		copy_len = data_len < temp->len ? data_len : temp->len;
		if (copy_to_user(m->msg_iov->iov_base + offset, temp->data,
				 copy_len)) {
			pr_err("%s: Copy to user failed\n", __func__);
			return -EFAULT;
		}
		offset += copy_len;
		data_len -= copy_len;
	}
	return offset;
}

static int msm_ipc_router_create(struct net *net,
				 struct socket *sock,
				 int protocol,
				 int kern)
{
	struct sock *sk;
	struct msm_ipc_port *port_ptr;

	if (unlikely(protocol != 0)) {
		pr_err("%s: Protocol not supported\n", __func__);
		return -EPROTONOSUPPORT;
	}

	switch (sock->type) {
	case SOCK_DGRAM:
		break;
	default:
		pr_err("%s: Protocol type not supported\n", __func__);
		return -EPROTOTYPE;
	}

	sk = sk_alloc(net, AF_MSM_IPC, GFP_KERNEL, &msm_ipc_proto);
	if (!sk) {
		pr_err("%s: sk_alloc failed\n", __func__);
		return -ENOMEM;
	}

	port_ptr = msm_ipc_router_create_raw_port(sk, NULL, NULL);
	if (!port_ptr) {
		pr_err("%s: port_ptr alloc failed\n", __func__);
		sk_free(sk);
		return -ENOMEM;
	}

	port_ptr->check_send_permissions = msm_ipc_check_send_permissions;
	sock->ops = &msm_ipc_proto_ops;
	sock_init_data(sock, sk);
	sk->sk_rcvtimeo = DEFAULT_RCV_TIMEO;

	msm_ipc_sk(sk)->port = port_ptr;

	return 0;
}

int msm_ipc_router_bind(struct socket *sock, struct sockaddr *uaddr,
			       int uaddr_len)
{
	struct sockaddr_msm_ipc *addr = (struct sockaddr_msm_ipc *)uaddr;
	struct sock *sk = sock->sk;
	struct msm_ipc_port *port_ptr;
	int ret;
	void *pil;

	if (!sk)
		return -EINVAL;

	if (!check_permissions()) {
		pr_err("%s: %s Do not have permissions\n",
			__func__, current->comm);
		return -EPERM;
	}

	if (!uaddr_len) {
		pr_err("%s: Invalid address length\n", __func__);
		return -EINVAL;
	}

	if (addr->family != AF_MSM_IPC) {
		pr_err("%s: Address family is incorrect\n", __func__);
		return -EAFNOSUPPORT;
	}

	if (addr->address.addrtype != MSM_IPC_ADDR_NAME) {
		pr_err("%s: Address type is incorrect\n", __func__);
		return -EINVAL;
	}

	port_ptr = msm_ipc_sk_port(sk);
	if (!port_ptr)
		return -ENODEV;

	pil = msm_ipc_load_default_node();
	msm_ipc_sk(sk)->default_pil = pil;
	lock_sock(sk);

	ret = msm_ipc_router_register_server(port_ptr, &addr->address);

	release_sock(sk);
	return ret;
}

static int msm_ipc_router_sendmsg(struct kiocb *iocb, struct socket *sock,
				  struct msghdr *m, size_t total_len)
{
	struct sock *sk = sock->sk;
	struct msm_ipc_port *port_ptr = msm_ipc_sk_port(sk);
	struct sockaddr_msm_ipc *dest = (struct sockaddr_msm_ipc *)m->msg_name;
	struct sk_buff_head *msg;
	struct sk_buff *ipc_buf;
	int ret;

	if (!dest)
		return -EDESTADDRREQ;

	if (m->msg_namelen < sizeof(*dest) || dest->family != AF_MSM_IPC)
		return -EINVAL;

	if (total_len > MAX_IPC_PKT_SIZE)
		return -EINVAL;

	lock_sock(sk);
	msg = msm_ipc_router_build_msg(m->msg_iovlen, m->msg_iov, total_len);
	if (!msg) {
		pr_err("%s: Msg build failure\n", __func__);
		ret = -ENOMEM;
		goto out_sendmsg;
	}

	if (port_ptr->type == CLIENT_PORT)
		wait_for_irsc_completion();
	ipc_buf = skb_peek(msg);
	if (ipc_buf)
		msm_ipc_router_ipc_log(IPC_SEND, ipc_buf, port_ptr);
	ret = msm_ipc_router_send_to(port_ptr, msg, &dest->address);
	if (ret != total_len) {
		if (ret < 0) {
			if (ret != -EAGAIN)
				pr_err("%s: Send_to failure %d\n",
							__func__, ret);
			msm_ipc_router_free_skb(msg);
		} else if (ret >= 0) {
			ret = -EFAULT;
		}
	}

out_sendmsg:
	release_sock(sk);
	return ret;
}

static int msm_ipc_router_recvmsg(struct kiocb *iocb, struct socket *sock,
				  struct msghdr *m, size_t buf_len, int flags)
{
	struct sock *sk = sock->sk;
	struct msm_ipc_port *port_ptr = msm_ipc_sk_port(sk);
	struct rr_packet *pkt;
	struct sk_buff *ipc_buf;
	long timeout;
	int ret;

	if (m->msg_iovlen != 1)
		return -EOPNOTSUPP;

	if (!buf_len)
		return -EINVAL;

	lock_sock(sk);
	timeout = sk->sk_rcvtimeo;

	ret = msm_ipc_router_rx_data_wait(port_ptr, timeout);
	if (ret) {
		release_sock(sk);
		if (ret == -ENOMSG)
			m->msg_namelen = 0;
		return ret;
	}

	ret = msm_ipc_router_read(port_ptr, &pkt, buf_len);
	if (ret <= 0 || !pkt) {
		release_sock(sk);
		return ret;
	}

	ret = msm_ipc_router_extract_msg(m, pkt);
	ipc_buf = skb_peek(pkt->pkt_fragment_q);
	if (ipc_buf)
		msm_ipc_router_ipc_log(IPC_RECV, ipc_buf, port_ptr);
	release_pkt(pkt);
	release_sock(sk);
	return ret;
}

static int msm_ipc_router_ioctl(struct socket *sock,
				unsigned int cmd, unsigned long arg)
{
	struct sock *sk = sock->sk;
	struct msm_ipc_port *port_ptr;
	struct server_lookup_args server_arg;
	struct msm_ipc_server_info *srv_info = NULL;
	unsigned int n;
	size_t srv_info_sz = 0;
	int ret;
	void *pil;

	if (!sk)
		return -EINVAL;

	lock_sock(sk);
	port_ptr = msm_ipc_sk_port(sock->sk);
	if (!port_ptr) {
		release_sock(sk);
		return -EINVAL;
	}

	switch (cmd) {
	case IPC_ROUTER_IOCTL_GET_VERSION:
		n = IPC_ROUTER_V1;
		ret = put_user(n, (unsigned int *)arg);
		break;

	case IPC_ROUTER_IOCTL_GET_MTU:
		n = (MAX_IPC_PKT_SIZE - IPC_ROUTER_HDR_SIZE);
		ret = put_user(n, (unsigned int *)arg);
		break;

	case IPC_ROUTER_IOCTL_GET_CURR_PKT_SIZE:
		ret = msm_ipc_router_get_curr_pkt_size(port_ptr);
		break;

	case IPC_ROUTER_IOCTL_LOOKUP_SERVER:
		pil = msm_ipc_load_default_node();
		msm_ipc_sk(sk)->default_pil = pil;
		ret = copy_from_user(&server_arg, (void *)arg,
				     sizeof(server_arg));
		if (ret) {
			ret = -EFAULT;
			break;
		}

		if (server_arg.num_entries_in_array < 0) {
			ret = -EINVAL;
			break;
		}
		if (server_arg.num_entries_in_array) {
			if (server_arg.num_entries_in_array >
				(SIZE_MAX / sizeof(*srv_info))) {
				pr_err("%s: Integer Overflow %d * %d\n",
					__func__, sizeof(*srv_info),
					server_arg.num_entries_in_array);
				ret = -EINVAL;
				break;
			}
			srv_info_sz = server_arg.num_entries_in_array *
					sizeof(*srv_info);
			srv_info = kmalloc(srv_info_sz, GFP_KERNEL);
			if (!srv_info) {
				ret = -ENOMEM;
				break;
			}
		}
		ret = msm_ipc_router_lookup_server_name(&server_arg.port_name,
				srv_info, server_arg.num_entries_in_array,
				server_arg.lookup_mask);
		if (ret < 0) {
			pr_err("%s: Server not found\n", __func__);
			ret = -ENODEV;
			kfree(srv_info);
			break;
		}
		server_arg.num_entries_found = ret;

		ret = copy_to_user((void *)arg, &server_arg,
				   sizeof(server_arg));
		if (srv_info_sz) {
			ret = copy_to_user((void *)(arg + sizeof(server_arg)),
					   srv_info, srv_info_sz);
			if (ret)
				ret = -EFAULT;
			kfree(srv_info);
		}
		break;

	case IPC_ROUTER_IOCTL_BIND_CONTROL_PORT:
		ret = msm_ipc_router_bind_control_port(port_ptr);
		break;

	case IPC_ROUTER_IOCTL_CONFIG_SEC_RULES:
		ret = msm_ipc_config_sec_rules((void *)arg);
		if (ret != -EPERM)
			port_ptr->type = IRSC_PORT;
		break;

	default:
		ret = -EINVAL;
	}
	release_sock(sk);
	return ret;
}

static unsigned int msm_ipc_router_poll(struct file *file,
			struct socket *sock, poll_table *wait)
{
	struct sock *sk = sock->sk;
	struct msm_ipc_port *port_ptr;
	uint32_t mask = 0;

	if (!sk)
		return -EINVAL;

	port_ptr = msm_ipc_sk_port(sk);
	if (!port_ptr)
		return -EINVAL;

	poll_wait(file, &port_ptr->port_rx_wait_q, wait);

	if (!list_empty(&port_ptr->port_rx_q))
		mask |= (POLLRDNORM | POLLIN);

	return mask;
}

static int msm_ipc_router_close(struct socket *sock)
{
	struct sock *sk = sock->sk;
	struct msm_ipc_port *port_ptr = msm_ipc_sk_port(sk);
	void *pil = msm_ipc_sk(sk)->default_pil;
	int ret;

	lock_sock(sk);
	ret = msm_ipc_router_close_port(port_ptr);
	if (pil)
		msm_ipc_unload_default_node(pil);
	release_sock(sk);
	sock_put(sk);
	sock->sk = NULL;

	return ret;
}

static const struct net_proto_family msm_ipc_family_ops = {
	.owner		= THIS_MODULE,
	.family		= AF_MSM_IPC,
	.create		= msm_ipc_router_create
};

static const struct proto_ops msm_ipc_proto_ops = {
	.family         = AF_MSM_IPC,
	.owner		= THIS_MODULE,
	.release        = msm_ipc_router_close,
	.bind		= msm_ipc_router_bind,
	.connect	= sock_no_connect,
	.socketpair	= sock_no_socketpair,
	.accept		= sock_no_accept,
	.getname	= sock_no_getname,
	.poll           = msm_ipc_router_poll,
	.ioctl          = msm_ipc_router_ioctl,
	.listen		= sock_no_listen,
	.shutdown	= sock_no_shutdown,
	.setsockopt     = sock_no_setsockopt,
	.getsockopt     = sock_no_getsockopt,
	.sendmsg	= msm_ipc_router_sendmsg,
	.recvmsg	= msm_ipc_router_recvmsg,
	.mmap		= sock_no_mmap,
	.sendpage	= sock_no_sendpage,
};

static struct proto msm_ipc_proto = {
	.name           = "MSM_IPC",
	.owner          = THIS_MODULE,
	.obj_size       = sizeof(struct msm_ipc_sock),
};

/**
 * msm_ipc_router_ipc_log_init() - Init function for IPC Logging
 *
 * Initialize the buffers to be used to provide the log information
 * pertaining to the request, response and indication data flow that
 * happens between user and kernel spaces.
 */
void msm_ipc_router_ipc_log_init(void)
{
	ipc_req_resp_log_txt =
		ipc_log_context_create(REQ_RESP_IPC_LOG_PAGES,
			"ipc_rtr_req_resp", 0);
	if (!ipc_req_resp_log_txt) {
		pr_err("%s: Unable to create IPC logging for Req/Resp",
			__func__);
	}
	ipc_ind_log_txt =
		ipc_log_context_create(IND_IPC_LOG_PAGES, "ipc_rtr_ind", 0);
	if (!ipc_ind_log_txt) {
		pr_err("%s: Unable to create IPC logging for Indications",
			__func__);
	}
}

int msm_ipc_router_init_sockets(void)
{
	int ret;

	ret = proto_register(&msm_ipc_proto, 1);
	if (ret) {
		pr_err("Failed to register MSM_IPC protocol type\n");
		goto out_init_sockets;
	}

	ret = sock_register(&msm_ipc_family_ops);
	if (ret) {
		pr_err("Failed to register MSM_IPC socket type\n");
		proto_unregister(&msm_ipc_proto);
		goto out_init_sockets;
	}

	sockets_enabled = 1;
	msm_ipc_router_ipc_log_init();
out_init_sockets:
	return ret;
}

void msm_ipc_router_exit_sockets(void)
{
	if (!sockets_enabled)
		return;

	sockets_enabled = 0;
	sock_unregister(msm_ipc_family_ops.family);
	proto_unregister(&msm_ipc_proto);
}
