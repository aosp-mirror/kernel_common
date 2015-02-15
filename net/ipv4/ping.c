/*
 * INET		An implementation of the TCP/IP protocol suite for the LINUX
 *		operating system.  INET is implemented using the  BSD Socket
 *		interface as the means of communication with the user level.
 *
 *		"Ping" sockets
 *
 *		This program is free software; you can redistribute it and/or
 *		modify it under the terms of the GNU General Public License
 *		as published by the Free Software Foundation; either version
 *		2 of the License, or (at your option) any later version.
 *
 * Based on ipv4/udp.c code.
 *
 * Authors:	Vasiliy Kulikov / Openwall (for Linux 2.6),
 *		Pavel Kankovsky (for Linux 2.4.32)
 *
 * Pavel gave all rights to bugs to Vasiliy,
 * none of the bugs are Pavel's now.
 *
 */

#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/socket.h>
#include <linux/sockios.h>
#include <linux/in.h>
#include <linux/errno.h>
#include <linux/timer.h>
#include <linux/mm.h>
#include <linux/inet.h>
#include <linux/netdevice.h>
#include <net/snmp.h>
#include <net/ip.h>
#include <net/icmp.h>
#include <net/protocol.h>
#include <linux/skbuff.h>
#include <linux/proc_fs.h>
#include <linux/export.h>
#include <net/sock.h>
#include <net/ping.h>
#include <net/udp.h>
#include <net/route.h>
#include <net/inet_common.h>
#include <net/checksum.h>

#if IS_ENABLED(CONFIG_IPV6)
#include <linux/in6.h>
#include <linux/icmpv6.h>
#include <net/addrconf.h>
#include <net/ipv6.h>
#include <net/transp_v6.h>
#endif


struct ping_table ping_table;
struct pingv6_ops pingv6_ops;
EXPORT_SYMBOL_GPL(pingv6_ops);

static u16 ping_port_rover;

static inline int ping_hashfn(struct net *net, unsigned num, unsigned mask)
{
	int res = (num + net_hash_mix(net)) & mask;
	pr_debug("hash(%d) = %d\n", num, res);
	return res;
}
EXPORT_SYMBOL_GPL(ping_hash);

static inline struct hlist_nulls_head *ping_hashslot(struct ping_table *table,
					     struct net *net, unsigned num)
{
	return &table->hash[ping_hashfn(net, num, PING_HTABLE_MASK)];
}

int ping_get_port(struct sock *sk, unsigned short ident)
{
	struct hlist_nulls_node *node;
	struct hlist_nulls_head *hlist;
	struct inet_sock *isk, *isk2;
	struct sock *sk2 = NULL;

	isk = inet_sk(sk);
	write_lock_bh(&ping_table.lock);
	if (ident == 0) {
		u32 i;
		u16 result = ping_port_rover + 1;

		for (i = 0; i < (1L << 16); i++, result++) {
			if (!result)
				result++; 
			hlist = ping_hashslot(&ping_table, sock_net(sk),
					    result);
			ping_portaddr_for_each_entry(sk2, node, hlist) {
				isk2 = inet_sk(sk2);

				if (isk2->inet_num == result)
					goto next_port;
			}

			
			ping_port_rover = ident = result;
			break;
next_port:
			;
		}
		if (i >= (1L << 16))
			goto fail;
	} else {
		hlist = ping_hashslot(&ping_table, sock_net(sk), ident);
		ping_portaddr_for_each_entry(sk2, node, hlist) {
			isk2 = inet_sk(sk2);

			if ((isk2->inet_num == ident) &&
			    (sk2 != sk) &&
			    (!sk2->sk_reuse || !sk->sk_reuse))
				goto fail;
		}
	}

	pr_debug("found port/ident = %d\n", ident);
	isk->inet_num = ident;
	if (sk_unhashed(sk)) {
		pr_debug("was not hashed\n");
		sock_hold(sk);
		hlist_nulls_add_head(&sk->sk_nulls_node, hlist);
		sock_prot_inuse_add(sock_net(sk), sk->sk_prot, 1);
	}
	write_unlock_bh(&ping_table.lock);
	return 0;

fail:
	write_unlock_bh(&ping_table.lock);
	return 1;
}
EXPORT_SYMBOL_GPL(ping_get_port);

void ping_hash(struct sock *sk)
{
	pr_debug("ping_hash(sk->port=%u)\n", inet_sk(sk)->inet_num);
	BUG(); 
}

void ping_unhash(struct sock *sk)
{
	struct inet_sock *isk = inet_sk(sk);
	pr_debug("ping_unhash(isk=%p,isk->num=%u)\n", isk, isk->inet_num);
	if (sk_hashed(sk)) {
		write_lock_bh(&ping_table.lock);
		hlist_nulls_del(&sk->sk_nulls_node);
		sock_put(sk);
		isk->inet_num = 0;
		isk->inet_sport = 0;
		sock_prot_inuse_add(sock_net(sk), sk->sk_prot, -1);
		write_unlock_bh(&ping_table.lock);
	}
}
EXPORT_SYMBOL_GPL(ping_unhash);

static struct sock *ping_lookup(struct net *net, struct sk_buff *skb, u16 ident)
{
	struct hlist_nulls_head *hslot = ping_hashslot(&ping_table, net, ident);
	struct sock *sk = NULL;
	struct inet_sock *isk;
	struct hlist_nulls_node *hnode;
	int dif = skb->dev->ifindex;

	if (skb->protocol == htons(ETH_P_IP)) {
		pr_debug("try to find: num = %d, daddr = %pI4, dif = %d\n",
			 (int)ident, &ip_hdr(skb)->daddr, dif);
#if IS_ENABLED(CONFIG_IPV6)
	} else if (skb->protocol == htons(ETH_P_IPV6)) {
		pr_debug("try to find: num = %d, daddr = %pI6c, dif = %d\n",
			 (int)ident, &ipv6_hdr(skb)->daddr, dif);
#endif
	}

	read_lock_bh(&ping_table.lock);

	ping_portaddr_for_each_entry(sk, hnode, hslot) {
		isk = inet_sk(sk);

		pr_debug("iterate\n");
		if (isk->inet_num != ident)
			continue;

		if (skb->protocol == htons(ETH_P_IP) &&
		    sk->sk_family == AF_INET) {
			pr_debug("found: %p: num=%d, daddr=%pI4, dif=%d\n", sk,
				 (int) isk->inet_num, &isk->inet_rcv_saddr,
				 sk->sk_bound_dev_if);

			if (isk->inet_rcv_saddr &&
			    isk->inet_rcv_saddr != ip_hdr(skb)->daddr)
				continue;
#if IS_ENABLED(CONFIG_IPV6)
		} else if (skb->protocol == htons(ETH_P_IPV6) &&
			   sk->sk_family == AF_INET6) {
			struct ipv6_pinfo *np = inet6_sk(sk);

			pr_debug("found: %p: num=%d, daddr=%pI6c, dif=%d\n", sk,
				 (int) isk->inet_num,
				 &inet6_sk(sk)->rcv_saddr,
				 sk->sk_bound_dev_if);

			if (!ipv6_addr_any(&np->rcv_saddr) &&
			    !ipv6_addr_equal(&np->rcv_saddr,
					     &ipv6_hdr(skb)->daddr))
				continue;
#endif
		}

		if (sk->sk_bound_dev_if && sk->sk_bound_dev_if != dif)
			continue;

		sock_hold(sk);
		goto exit;
	}

	sk = NULL;
exit:
	read_unlock_bh(&ping_table.lock);

	return sk;
}

static void inet_get_ping_group_range_net(struct net *net, gid_t *low,
					  gid_t *high)
{
	gid_t *data = net->ipv4.sysctl_ping_group_range;
	unsigned seq;
	do {
		seq = read_seqbegin(&sysctl_local_ports.lock);

		*low = data[0];
		*high = data[1];
	} while (read_seqretry(&sysctl_local_ports.lock, seq));
}


int ping_init_sock(struct sock *sk)
{
	struct net *net = sock_net(sk);
	gid_t group = current_egid();
	gid_t range[2];
	struct group_info *group_info;
	int i, j, count;
	int ret = 0;

	inet_get_ping_group_range_net(net, range, range+1);
	if (range[0] <= group && group <= range[1])
		return 0;

	group_info = get_current_groups();
	count = group_info->ngroups;
	for (i = 0; i < group_info->nblocks; i++) {
		int cp_count = min_t(int, NGROUPS_PER_BLOCK, count);

		for (j = 0; j < cp_count; j++) {
			group = group_info->blocks[i][j];
			if (range[0] <= group && group <= range[1])
				goto out_release_group;
		}

		count -= cp_count;
	}

	ret = -EACCES;

out_release_group:
	put_group_info(group_info);
	return ret;
}
EXPORT_SYMBOL_GPL(ping_init_sock);

void ping_close(struct sock *sk, long timeout)
{
	pr_debug("ping_close(sk=%p,sk->num=%u)\n",
		 inet_sk(sk), inet_sk(sk)->inet_num);
	pr_debug("isk->refcnt = %d\n", sk->sk_refcnt.counter);

	sk_common_release(sk);
}
EXPORT_SYMBOL_GPL(ping_close);

int ping_check_bind_addr(struct sock *sk, struct inet_sock *isk,
			 struct sockaddr *uaddr, int addr_len) {
	struct net *net = sock_net(sk);
	if (sk->sk_family == AF_INET) {
		struct sockaddr_in *addr = (struct sockaddr_in *) uaddr;
		int chk_addr_ret;

		if (addr_len < sizeof(*addr))
			return -EINVAL;

		pr_debug("ping_check_bind_addr(sk=%p,addr=%pI4,port=%d)\n",
			 sk, &addr->sin_addr.s_addr, ntohs(addr->sin_port));

		chk_addr_ret = inet_addr_type(net, addr->sin_addr.s_addr);

		if (addr->sin_addr.s_addr == htonl(INADDR_ANY))
			chk_addr_ret = RTN_LOCAL;

		if ((sysctl_ip_nonlocal_bind == 0 &&
		    isk->freebind == 0 && isk->transparent == 0 &&
		     chk_addr_ret != RTN_LOCAL) ||
		    chk_addr_ret == RTN_MULTICAST ||
		    chk_addr_ret == RTN_BROADCAST)
			return -EADDRNOTAVAIL;

#if IS_ENABLED(CONFIG_IPV6)
	} else if (sk->sk_family == AF_INET6) {
		struct sockaddr_in6 *addr = (struct sockaddr_in6 *) uaddr;
		int addr_type, scoped, has_addr;
		struct net_device *dev = NULL;

		if (addr_len < sizeof(*addr))
			return -EINVAL;

		pr_debug("ping_check_bind_addr(sk=%p,addr=%pI6c,port=%d)\n",
			 sk, addr->sin6_addr.s6_addr, ntohs(addr->sin6_port));

		addr_type = ipv6_addr_type(&addr->sin6_addr);
		scoped = __ipv6_addr_needs_scope_id(addr_type);
		if ((addr_type != IPV6_ADDR_ANY &&
		     !(addr_type & IPV6_ADDR_UNICAST)) ||
		    (scoped && !addr->sin6_scope_id))
			return -EINVAL;

		rcu_read_lock();
		if (addr->sin6_scope_id) {
			dev = dev_get_by_index_rcu(net, addr->sin6_scope_id);
			if (!dev) {
				rcu_read_unlock();
				return -ENODEV;
			}
		}
		has_addr = pingv6_ops.ipv6_chk_addr(net, &addr->sin6_addr, dev,
						    scoped);
		rcu_read_unlock();

		if (!(isk->freebind || isk->transparent || has_addr ||
		      addr_type == IPV6_ADDR_ANY))
			return -EADDRNOTAVAIL;

		if (scoped)
			sk->sk_bound_dev_if = addr->sin6_scope_id;
#endif
	} else {
		return -EAFNOSUPPORT;
	}
	return 0;
}

void ping_set_saddr(struct sock *sk, struct sockaddr *saddr)
{
	if (saddr->sa_family == AF_INET) {
		struct inet_sock *isk = inet_sk(sk);
		struct sockaddr_in *addr = (struct sockaddr_in *) saddr;
		isk->inet_rcv_saddr = isk->inet_saddr = addr->sin_addr.s_addr;
#if IS_ENABLED(CONFIG_IPV6)
	} else if (saddr->sa_family == AF_INET6) {
		struct sockaddr_in6 *addr = (struct sockaddr_in6 *) saddr;
		struct ipv6_pinfo *np = inet6_sk(sk);
		np->rcv_saddr = np->saddr = addr->sin6_addr;
#endif
	}
}

void ping_clear_saddr(struct sock *sk, int dif)
{
	sk->sk_bound_dev_if = dif;
	if (sk->sk_family == AF_INET) {
		struct inet_sock *isk = inet_sk(sk);
		isk->inet_rcv_saddr = isk->inet_saddr = 0;
#if IS_ENABLED(CONFIG_IPV6)
	} else if (sk->sk_family == AF_INET6) {
		struct ipv6_pinfo *np = inet6_sk(sk);
		memset(&np->rcv_saddr, 0, sizeof(np->rcv_saddr));
		memset(&np->saddr, 0, sizeof(np->saddr));
#endif
	}
}

int ping_bind(struct sock *sk, struct sockaddr *uaddr, int addr_len)
{
	struct inet_sock *isk = inet_sk(sk);
	unsigned short snum;
	int err;
	int dif = sk->sk_bound_dev_if;

	err = ping_check_bind_addr(sk, isk, uaddr, addr_len);
	if (err)
		return err;

	lock_sock(sk);

	err = -EINVAL;
	if (isk->inet_num != 0)
		goto out;

	err = -EADDRINUSE;
	ping_set_saddr(sk, uaddr);
	snum = ntohs(((struct sockaddr_in *)uaddr)->sin_port);
	if (ping_get_port(sk, snum) != 0) {
		ping_clear_saddr(sk, dif);
		goto out;
	}

	pr_debug("after bind(): num = %d, dif = %d\n",
		 (int)isk->inet_num,
		 (int)sk->sk_bound_dev_if);

	err = 0;
	if ((sk->sk_family == AF_INET && isk->inet_rcv_saddr) ||
	    (sk->sk_family == AF_INET6 &&
	     !ipv6_addr_any(&inet6_sk(sk)->rcv_saddr)))
		sk->sk_userlocks |= SOCK_BINDADDR_LOCK;

	if (snum)
		sk->sk_userlocks |= SOCK_BINDPORT_LOCK;
	isk->inet_sport = htons(isk->inet_num);
	isk->inet_daddr = 0;
	isk->inet_dport = 0;

#if IS_ENABLED(CONFIG_IPV6)
	if (sk->sk_family == AF_INET6)
		memset(&inet6_sk(sk)->daddr, 0, sizeof(inet6_sk(sk)->daddr));
#endif

	sk_dst_reset(sk);
out:
	release_sock(sk);
	pr_debug("ping_v4_bind -> %d\n", err);
	return err;
}
EXPORT_SYMBOL_GPL(ping_bind);


static inline int ping_supported(int family, int type, int code)
{
	return (family == AF_INET && type == ICMP_ECHO && code == 0) ||
	       (family == AF_INET6 && type == ICMPV6_ECHO_REQUEST && code == 0);
}


void ping_err(struct sk_buff *skb, int offset, u32 info)
{
	int family;
	struct icmphdr *icmph;
	struct inet_sock *inet_sock;
	int type;
	int code;
	struct net *net = dev_net(skb->dev);
	struct sock *sk;
	int harderr;
	int err;

	if (skb->protocol == htons(ETH_P_IP)) {
		struct iphdr *iph = (struct iphdr *)skb->data;
		offset = iph->ihl << 2;
		family = AF_INET;
		type = icmp_hdr(skb)->type;
		code = icmp_hdr(skb)->code;
		icmph = (struct icmphdr *)(skb->data + offset);
	} else if (skb->protocol == htons(ETH_P_IPV6)) {
		family = AF_INET6;
		type = icmp6_hdr(skb)->icmp6_type;
		code = icmp6_hdr(skb)->icmp6_code;
		icmph = (struct icmphdr *) (skb->data + offset);
	} else {
		BUG();
	}

	

	if (!ping_supported(family, icmph->type, icmph->code))
		return;

	pr_debug("ping_err(proto=0x%x,type=%d,code=%d,id=%04x,seq=%04x)\n",
		 skb->protocol, type, code, ntohs(icmph->un.echo.id),
		 ntohs(icmph->un.echo.sequence));

	sk = ping_lookup(net, skb, ntohs(icmph->un.echo.id));
	if (sk == NULL) {
		pr_debug("no socket, dropping\n");
		return;	
	}
	pr_debug("err on socket %p\n", sk);

	err = 0;
	harderr = 0;
	inet_sock = inet_sk(sk);

	if (skb->protocol == htons(ETH_P_IP)) {
		switch (type) {
		default:
		case ICMP_TIME_EXCEEDED:
			err = EHOSTUNREACH;
			break;
		case ICMP_SOURCE_QUENCH:
			err = EREMOTEIO;
			break;
		case ICMP_PARAMETERPROB:
			err = EPROTO;
			harderr = 1;
			break;
		case ICMP_DEST_UNREACH:
			if (code == ICMP_FRAG_NEEDED) { 
				if (inet_sock->pmtudisc != IP_PMTUDISC_DONT) {
					err = EMSGSIZE;
					harderr = 1;
					break;
				}
				goto out;
			}
			err = EHOSTUNREACH;
			if (code <= NR_ICMP_UNREACH) {
				harderr = icmp_err_convert[code].fatal;
				err = icmp_err_convert[code].errno;
			}
			break;
		case ICMP_REDIRECT:
			
			err = EREMOTEIO;
			break;
		}
#if IS_ENABLED(CONFIG_IPV6)
	} else if (skb->protocol == htons(ETH_P_IPV6)) {
		harderr = pingv6_ops.icmpv6_err_convert(type, code, &err);
#endif
	}

	if ((family == AF_INET && !inet_sock->recverr) ||
	    (family == AF_INET6 && !inet6_sk(sk)->recverr)) {
		if (!harderr || sk->sk_state != TCP_ESTABLISHED)
			goto out;
	} else {
		if (family == AF_INET) {
			ip_icmp_error(sk, skb, err, 0 ,
				      info, (u8 *)icmph);
#if IS_ENABLED(CONFIG_IPV6)
		} else if (family == AF_INET6) {
			pingv6_ops.ipv6_icmp_error(sk, skb, err, 0,
						   info, (u8 *)icmph);
#endif
		}
	}
	sk->sk_err = err;
	sk->sk_error_report(sk);
out:
	sock_put(sk);
}
EXPORT_SYMBOL_GPL(ping_err);

void ping_v4_err(struct sk_buff *skb, u32 info)
{
	ping_err(skb, 0, info);
}


int ping_getfrag(void *from, char *to,
		 int offset, int fraglen, int odd, struct sk_buff *skb)
{
	struct pingfakehdr *pfh = (struct pingfakehdr *)from;

	if (offset == 0) {
		if (fraglen < sizeof(struct icmphdr))
			BUG();
		if (csum_partial_copy_fromiovecend(to + sizeof(struct icmphdr),
			    pfh->iov, 0, fraglen - sizeof(struct icmphdr),
			    &pfh->wcheck))
			return -EFAULT;
	} else if (offset < sizeof(struct icmphdr)) {
			BUG();
	} else {
		if (csum_partial_copy_fromiovecend
				(to, pfh->iov, offset - sizeof(struct icmphdr),
				 fraglen, &pfh->wcheck))
			return -EFAULT;
	}

#if IS_ENABLED(CONFIG_IPV6)
	if (pfh->family == AF_INET6) {
		skb->csum = pfh->wcheck;
		skb->ip_summed = CHECKSUM_NONE;
		pfh->wcheck = 0;
	}
#endif

	return 0;
}
EXPORT_SYMBOL_GPL(ping_getfrag);

static int ping_v4_push_pending_frames(struct sock *sk, struct pingfakehdr *pfh,
				       struct flowi4 *fl4)
{
	struct sk_buff *skb = skb_peek(&sk->sk_write_queue);

	pfh->wcheck = csum_partial((char *)&pfh->icmph,
		sizeof(struct icmphdr), pfh->wcheck);
	pfh->icmph.checksum = csum_fold(pfh->wcheck);
	memcpy(icmp_hdr(skb), &pfh->icmph, sizeof(struct icmphdr));
	skb->ip_summed = CHECKSUM_NONE;
	return ip_push_pending_frames(sk, fl4);
}

int ping_common_sendmsg(int family, struct msghdr *msg, size_t len,
			void *user_icmph, size_t icmph_len) {
	u8 type, code;

	if (len > 0xFFFF)
		return -EMSGSIZE;


	
	if (msg->msg_flags & MSG_OOB)
		return -EOPNOTSUPP;

	if (memcpy_fromiovec(user_icmph, msg->msg_iov, icmph_len))
		return -EFAULT;

	if (family == AF_INET) {
		type = ((struct icmphdr *) user_icmph)->type;
		code = ((struct icmphdr *) user_icmph)->code;
#if IS_ENABLED(CONFIG_IPV6)
	} else if (family == AF_INET6) {
		type = ((struct icmp6hdr *) user_icmph)->icmp6_type;
		code = ((struct icmp6hdr *) user_icmph)->icmp6_code;
#endif
	} else {
		BUG();
	}

	if (!ping_supported(family, type, code))
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL_GPL(ping_common_sendmsg);

int ping_v4_sendmsg(struct kiocb *iocb, struct sock *sk, struct msghdr *msg,
		    size_t len)
{
	struct net *net = sock_net(sk);
	struct flowi4 fl4;
	struct inet_sock *inet = inet_sk(sk);
	struct ipcm_cookie ipc;
	struct icmphdr user_icmph;
	struct pingfakehdr pfh;
	struct rtable *rt = NULL;
	struct ip_options_data opt_copy;
	int free = 0;
	__be32 saddr, daddr, faddr;
	u8  tos;
	int err;

	pr_debug("ping_v4_sendmsg(sk=%p,sk->num=%u)\n", inet, inet->inet_num);

	err = ping_common_sendmsg(AF_INET, msg, len, &user_icmph,
				  sizeof(user_icmph));
	if (err)
		return err;


	if (msg->msg_name) {
		struct sockaddr_in *usin = (struct sockaddr_in *)msg->msg_name;
		if (msg->msg_namelen < sizeof(*usin))
			return -EINVAL;
		if (usin->sin_family != AF_INET)
			return -EINVAL;
		daddr = usin->sin_addr.s_addr;
		
	} else {
		if (sk->sk_state != TCP_ESTABLISHED)
			return -EDESTADDRREQ;
		daddr = inet->inet_daddr;
		
	}

	ipc.addr = inet->inet_saddr;
	ipc.opt = NULL;
	ipc.oif = sk->sk_bound_dev_if;
	ipc.tx_flags = 0;
	err = sock_tx_timestamp(sk, &ipc.tx_flags);
	if (err)
		return err;

	if (msg->msg_controllen) {
		err = ip_cmsg_send(sock_net(sk), msg, &ipc);
		if (err)
			return err;
		if (ipc.opt)
			free = 1;
	}
	if (!ipc.opt) {
		struct ip_options_rcu *inet_opt;

		rcu_read_lock();
		inet_opt = rcu_dereference(inet->inet_opt);
		if (inet_opt) {
			memcpy(&opt_copy, inet_opt,
			       sizeof(*inet_opt) + inet_opt->opt.optlen);
			ipc.opt = &opt_copy.opt;
		}
		rcu_read_unlock();
	}

	saddr = ipc.addr;
	ipc.addr = faddr = daddr;

	if (ipc.opt && ipc.opt->opt.srr) {
		if (!daddr)
			return -EINVAL;
		faddr = ipc.opt->opt.faddr;
	}
	tos = RT_TOS(inet->tos);
	if (sock_flag(sk, SOCK_LOCALROUTE) ||
	    (msg->msg_flags & MSG_DONTROUTE) ||
	    (ipc.opt && ipc.opt->opt.is_strictroute)) {
		tos |= RTO_ONLINK;
	}

	if (ipv4_is_multicast(daddr)) {
		if (!ipc.oif)
			ipc.oif = inet->mc_index;
		if (!saddr)
			saddr = inet->mc_addr;
	} else if (!ipc.oif)
		ipc.oif = inet->uc_index;

	flowi4_init_output(&fl4, ipc.oif, sk->sk_mark, tos,
			   RT_SCOPE_UNIVERSE, sk->sk_protocol,
			   inet_sk_flowi_flags(sk), faddr, saddr, 0, 0,
			   sock_i_uid(sk));

	security_sk_classify_flow(sk, flowi4_to_flowi(&fl4));
	rt = ip_route_output_flow(net, &fl4, sk);
	if (IS_ERR(rt)) {
		err = PTR_ERR(rt);
		rt = NULL;
		if (err == -ENETUNREACH)
			IP_INC_STATS_BH(net, IPSTATS_MIB_OUTNOROUTES);
		goto out;
	}

	err = -EACCES;
	if ((rt->rt_flags & RTCF_BROADCAST) &&
	    !sock_flag(sk, SOCK_BROADCAST))
		goto out;

	if (msg->msg_flags & MSG_CONFIRM)
		goto do_confirm;
back_from_confirm:

	if (!ipc.addr)
		ipc.addr = fl4.daddr;

	lock_sock(sk);

	pfh.icmph.type = user_icmph.type; 
	pfh.icmph.code = user_icmph.code; 
	pfh.icmph.checksum = 0;
	pfh.icmph.un.echo.id = inet->inet_sport;
	pfh.icmph.un.echo.sequence = user_icmph.un.echo.sequence;
	pfh.iov = msg->msg_iov;
	pfh.wcheck = 0;
	pfh.family = AF_INET;

	err = ip_append_data(sk, &fl4, ping_getfrag, &pfh, len,
			0, &ipc, &rt, msg->msg_flags);
	if (err)
		ip_flush_pending_frames(sk);
	else
		err = ping_v4_push_pending_frames(sk, &pfh, &fl4);
	release_sock(sk);

out:
	ip_rt_put(rt);
	if (free)
		kfree(ipc.opt);
	if (!err) {
		icmp_out_count(sock_net(sk), user_icmph.type);
		return len;
	}
	return err;

do_confirm:
	dst_confirm(&rt->dst);
	if (!(msg->msg_flags & MSG_PROBE) || len)
		goto back_from_confirm;
	err = 0;
	goto out;
}

int ping_recvmsg(struct kiocb *iocb, struct sock *sk, struct msghdr *msg,
		 size_t len, int noblock, int flags, int *addr_len)
{
	struct inet_sock *isk = inet_sk(sk);
	int family = sk->sk_family;
	struct sockaddr_in *sin;
	struct sockaddr_in6 *sin6;
	struct sk_buff *skb;
	int copied, err;

#ifdef CONFIG_HTC_NETWORK_MODIFY
	err = 0 ;
#endif

	pr_debug("ping_recvmsg(sk=%p,sk->num=%u)\n", isk, isk->inet_num);

	err = -EOPNOTSUPP;
	if (flags & MSG_OOB)
		goto out;

	if (addr_len) {
		if (family == AF_INET)
			*addr_len = sizeof(*sin);
		else if (family == AF_INET6 && addr_len)
			*addr_len = sizeof(*sin6);
	}

	if (flags & MSG_ERRQUEUE) {
		if (family == AF_INET) {
			return ip_recv_error(sk, msg, len);
#if IS_ENABLED(CONFIG_IPV6)
		} else if (family == AF_INET6) {
			return pingv6_ops.ipv6_recv_error(sk, msg, len);
#endif
		}
	}

	skb = skb_recv_datagram(sk, flags, noblock, &err);
	if (!skb)
		goto out;

	copied = skb->len;
	if (copied > len) {
		msg->msg_flags |= MSG_TRUNC;
		copied = len;
	}

	
	err = skb_copy_datagram_iovec(skb, 0, msg->msg_iov, copied);
	if (err)
		goto done;

	sock_recv_timestamp(msg, sk, skb);

	
	if (family == AF_INET) {
		sin = (struct sockaddr_in *) msg->msg_name;
		if (sin) {
			sin->sin_family = AF_INET;
			sin->sin_port = 0 ;
			sin->sin_addr.s_addr = ip_hdr(skb)->saddr;
			memset(sin->sin_zero, 0, sizeof(sin->sin_zero));
		}

		if (isk->cmsg_flags)
			ip_cmsg_recv(msg, skb);

#if IS_ENABLED(CONFIG_IPV6)
	} else if (family == AF_INET6) {
		struct ipv6_pinfo *np = inet6_sk(sk);
		struct ipv6hdr *ip6 = ipv6_hdr(skb);
		sin6 = (struct sockaddr_in6 *) msg->msg_name;
		if (sin6) {
			sin6->sin6_family = AF_INET6;
			sin6->sin6_port = 0;
			sin6->sin6_addr = ip6->saddr;

			sin6->sin6_flowinfo = 0;
			if (np->sndflow)
				sin6->sin6_flowinfo =
					*(__be32 *)ip6 & IPV6_FLOWINFO_MASK;

			sin6->sin6_scope_id = ipv6_iface_scope_id(&sin6->sin6_addr,
								  IP6CB(skb)->iif);
		}

		if (inet6_sk(sk)->rxopt.all)
			pingv6_ops.datagram_recv_ctl(sk, msg, skb);
#endif
	} else {
		BUG();
	}

	err = copied;

done:
	skb_free_datagram(sk, skb);
out:
	pr_debug("ping_recvmsg -> %d\n", err);
	return err;
}
EXPORT_SYMBOL_GPL(ping_recvmsg);

int ping_queue_rcv_skb(struct sock *sk, struct sk_buff *skb)
{
	pr_debug("ping_queue_rcv_skb(sk=%p,sk->num=%d,skb=%p)\n",
		 inet_sk(sk), inet_sk(sk)->inet_num, skb);
	if (sock_queue_rcv_skb(sk, skb) < 0) {
		kfree_skb(skb);
		pr_debug("ping_queue_rcv_skb -> failed\n");
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(ping_queue_rcv_skb);



void ping_rcv(struct sk_buff *skb)
{
	struct sock *sk;
	struct net *net = dev_net(skb->dev);
	struct icmphdr *icmph = icmp_hdr(skb);

	

	pr_debug("ping_rcv(skb=%p,id=%04x,seq=%04x)\n",
		 skb, ntohs(icmph->un.echo.id), ntohs(icmph->un.echo.sequence));

	
	skb_push(skb, skb->data - (u8 *)icmph);

	sk = ping_lookup(net, skb, ntohs(icmph->un.echo.id));
	if (sk != NULL) {
		pr_debug("rcv on socket %p\n", sk);
		ping_queue_rcv_skb(sk, skb_get(skb));
		sock_put(sk);
		return;
	}
	pr_debug("no socket, dropping\n");

	
}
EXPORT_SYMBOL_GPL(ping_rcv);

struct proto ping_prot = {
	.name =		"PING",
	.owner =	THIS_MODULE,
	.init =		ping_init_sock,
	.close =	ping_close,
	.connect =	ip4_datagram_connect,
	.disconnect =	udp_disconnect,
	.setsockopt =	ip_setsockopt,
	.getsockopt =	ip_getsockopt,
	.sendmsg =	ping_v4_sendmsg,
	.recvmsg =	ping_recvmsg,
	.bind =		ping_bind,
	.backlog_rcv =	ping_queue_rcv_skb,
	.hash =		ping_hash,
	.unhash =	ping_unhash,
	.get_port =	ping_get_port,
	.obj_size =	sizeof(struct inet_sock),
};
EXPORT_SYMBOL(ping_prot);

#ifdef CONFIG_PROC_FS

static struct sock *ping_get_first(struct seq_file *seq, int start)
{
	struct sock *sk;
	struct ping_iter_state *state = seq->private;
	struct net *net = seq_file_net(seq);

	for (state->bucket = start; state->bucket < PING_HTABLE_SIZE;
	     ++state->bucket) {
		struct hlist_nulls_node *node;
		struct hlist_nulls_head *hslot;

		hslot = &ping_table.hash[state->bucket];

		if (hlist_nulls_empty(hslot))
			continue;

		sk_nulls_for_each(sk, node, hslot) {
			if (net_eq(sock_net(sk), net))
				goto found;
		}
	}
	sk = NULL;
found:
	return sk;
}

static struct sock *ping_get_next(struct seq_file *seq, struct sock *sk)
{
	struct ping_iter_state *state = seq->private;
	struct net *net = seq_file_net(seq);

	do {
		sk = sk_nulls_next(sk);
	} while (sk && (!net_eq(sock_net(sk), net)));

	if (!sk)
		return ping_get_first(seq, state->bucket + 1);
	return sk;
}

static struct sock *ping_get_idx(struct seq_file *seq, loff_t pos)
{
	struct sock *sk = ping_get_first(seq, 0);

	if (sk)
		while (pos && (sk = ping_get_next(seq, sk)) != NULL)
			--pos;
	return pos ? NULL : sk;
}

static void *ping_seq_start(struct seq_file *seq, loff_t *pos)
{
	struct ping_iter_state *state = seq->private;
	state->bucket = 0;

	read_lock_bh(&ping_table.lock);

	return *pos ? ping_get_idx(seq, *pos-1) : SEQ_START_TOKEN;
}

static void *ping_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct sock *sk;

	if (v == SEQ_START_TOKEN)
		sk = ping_get_idx(seq, 0);
	else
		sk = ping_get_next(seq, v);

	++*pos;
	return sk;
}

static void ping_seq_stop(struct seq_file *seq, void *v)
{
	read_unlock_bh(&ping_table.lock);
}

static void ping_format_sock(struct sock *sp, struct seq_file *f,
		int bucket)
{
	struct inet_sock *inet = inet_sk(sp);
	__be32 dest = inet->inet_daddr;
	__be32 src = inet->inet_rcv_saddr;
	__u16 destp = ntohs(inet->inet_dport);
	__u16 srcp = ntohs(inet->inet_sport);

	seq_printf(f, "%5d: %08X:%04X %08X:%04X"
		" %02X %08X:%08X %02X:%08lX %08X %5d %8d %lu %d %pK %d",
		bucket, src, srcp, dest, destp, sp->sk_state,
		sk_wmem_alloc_get(sp),
		sk_rmem_alloc_get(sp),
		0, 0L, 0, sock_i_uid(sp), 0, sock_i_ino(sp),
		atomic_read(&sp->sk_refcnt), sp,
		atomic_read(&sp->sk_drops));
}

static int ping_seq_show(struct seq_file *seq, void *v)
{
	seq_setwidth(seq, 127);
	if (v == SEQ_START_TOKEN)
		seq_puts(seq, "  sl  local_address rem_address   st tx_queue "
			   "rx_queue tr tm->when retrnsmt   uid  timeout "
			   "inode ref pointer drops");
	else {
		struct ping_iter_state *state = seq->private;

		ping_format_sock(v, seq, state->bucket);
	}
	seq_pad(seq, '\n');
	return 0;
}

static const struct seq_operations ping_seq_ops = {
	.show		= ping_seq_show,
	.start		= ping_seq_start,
	.next		= ping_seq_next,
	.stop		= ping_seq_stop,
};

static int ping_seq_open(struct inode *inode, struct file *file)
{
	return seq_open_net(inode, file, &ping_seq_ops,
			   sizeof(struct ping_iter_state));
}

static const struct file_operations ping_seq_fops = {
	.open		= ping_seq_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release_net,
};

static int ping_proc_register(struct net *net)
{
	struct proc_dir_entry *p;
	int rc = 0;

	p = proc_net_fops_create(net, "icmp", S_IRUGO, &ping_seq_fops);
	if (!p)
		rc = -ENOMEM;
	return rc;
}

static void ping_proc_unregister(struct net *net)
{
	proc_net_remove(net, "icmp");
}


static int __net_init ping_proc_init_net(struct net *net)
{
	return ping_proc_register(net);
}

static void __net_exit ping_proc_exit_net(struct net *net)
{
	ping_proc_unregister(net);
}

static struct pernet_operations ping_net_ops = {
	.init = ping_proc_init_net,
	.exit = ping_proc_exit_net,
};

int __init ping_proc_init(void)
{
	return register_pernet_subsys(&ping_net_ops);
}

void ping_proc_exit(void)
{
	unregister_pernet_subsys(&ping_net_ops);
}

#endif

void __init ping_init(void)
{
	int i;

	for (i = 0; i < PING_HTABLE_SIZE; i++)
		INIT_HLIST_NULLS_HEAD(&ping_table.hash[i], i);
	rwlock_init(&ping_table.lock);
}
