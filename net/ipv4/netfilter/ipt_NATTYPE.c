/*
 * net/ipv4/netfilter/ipt_NATTYPE.c
 * Endpoint Independent, Address Restricted and Port-Address Restricted
 * NAT types' kernel side implementation.
 *
 * (C) Copyright 2011, Ubicom, Inc.
 *
 * This file is part of the Ubicom32 Linux Kernel Port.
 *
 * The Ubicom32 Linux Kernel Port is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 2 of the
 * License, or (at your option) any later version.
 *
 * The Ubicom32 Linux Kernel Port is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Ubicom32 Linux Kernel Port.  If not,
 * see <http://www.gnu.org/licenses/>.
 *
 * Ubicom32 implementation derived from
 * Cameo's implementation(with many thanks):
 */
#include <linux/types.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#include <linux/module.h>
#include <net/protocol.h>
#include <net/checksum.h>
#include <net/ip.h>
#include <linux/tcp.h>
#include <net/netfilter/nf_conntrack.h>
#include <net/netfilter/nf_conntrack_core.h>
#include <net/netfilter/nf_nat_rule.h>
#include <linux/netfilter/x_tables.h>
#include <linux/netfilter_ipv4/ipt_NATTYPE.h>
#include <linux/atomic.h>

#if !defined(NATTYPE_DEBUG)
#define DEBUGP(type, args...)
#else
static const char * const types[] = {"TYPE_PORT_ADDRESS_RESTRICTED",
			"TYPE_ENDPOINT_INDEPENDENT",
			"TYPE_ADDRESS_RESTRICTED"};
static const char * const modes[] = {"MODE_DNAT", "MODE_FORWARD_IN",
			"MODE_FORWARD_OUT"};
#define DEBUGP(args...) printk(KERN_DEBUG args);
#endif

/*
 * TODO: Add magic value checks to data structure.
 */
struct ipt_nattype {
	struct list_head list;
	struct timer_list timeout;
	unsigned long timeout_value;
	unsigned int nattype_cookie;
	unsigned short proto;		/* Protocol: TCP or UDP */
	struct nf_nat_ipv4_range range;	/* LAN side source information */
	unsigned short nat_port;	/* Routed NAT port */
	unsigned int dest_addr;	/* Original egress packets destination addr */
	unsigned short dest_port;/* Original egress packets destination port */
};

#define NATTYPE_COOKIE 0x11abcdef

/*
 * TODO: It might be better to use a hash table for performance in
 * heavy traffic.
 */
static LIST_HEAD(nattype_list);
static DEFINE_SPINLOCK(nattype_lock);

/*
 * nattype_nte_debug_print()
 */
static void nattype_nte_debug_print(const struct ipt_nattype *nte,
				const char *s)
{
#if defined(NATTYPE_DEBUG)
	DEBUGP("%p: %s - proto[%d], src[%pI4:%d], nat[<x>:%d], dest[%pI4:%d]\n",
		nte, s, nte->proto,
		&nte->range.min_ip, ntohs(nte->range.min.all),
		ntohs(nte->nat_port),
		&nte->dest_addr, ntohs(nte->dest_port));
#endif
}

/*
 * nattype_free()
 *	Free the object.
 */
static void nattype_free(struct ipt_nattype *nte)
{
	nattype_nte_debug_print(nte, "free");
	kfree(nte);
}

/*
 * nattype_refresh_timer()
 *	Refresh the timer for this object.
 */
bool nattype_refresh_timer(unsigned long nat_type, unsigned long timeout_value)
{
	struct ipt_nattype *nte = (struct ipt_nattype *)nat_type;
	if (!nte)
		return false;
	spin_lock_bh(&nattype_lock);
	if (nte->nattype_cookie != NATTYPE_COOKIE) {
		spin_unlock_bh(&nattype_lock);
		return false;
	}
	if (del_timer(&nte->timeout)) {
		nte->timeout_value = timeout_value - jiffies;
		nte->timeout.expires = timeout_value;
		add_timer(&nte->timeout);
		spin_unlock_bh(&nattype_lock);
		return true;
	}
	spin_unlock_bh(&nattype_lock);
	return false;
}

/*
 * nattype_timer_timeout()
 *	The timer has gone off, self-destruct
 */
static void nattype_timer_timeout(unsigned long in_nattype)
{
	struct ipt_nattype *nte = (void *) in_nattype;

	/*
	 * The race with list deletion is solved by ensuring
	 * that either this code or the list deletion code
	 * but not both will remove the oject.
	 */
	nattype_nte_debug_print(nte, "timeout");
	spin_lock_bh(&nattype_lock);
	list_del(&nte->list);
	memset(nte, 0, sizeof(struct ipt_nattype));
	spin_unlock_bh(&nattype_lock);
	nattype_free(nte);
}

/*
 * nattype_packet_in_match()
 *	Ingress packet, try to match with this nattype entry.
 */
static bool nattype_packet_in_match(const struct ipt_nattype *nte,
				struct sk_buff *skb,
				const struct ipt_nattype_info *info)
{
	const struct iphdr *iph = ip_hdr(skb);
	uint16_t dst_port = 0;

	/*
	 * If the protocols are not the same, no sense in looking
	 * further.
	 */
	if (nte->proto != iph->protocol) {
		DEBUGP("nattype_packet_in_match: protocol failed: nte proto:" \
			" %d, packet proto: %d\n",
			nte->proto, iph->protocol);
		return false;
	}

	/*
	 * In ADDRESS_RESTRICT, the egress destination must match the source
	 * of this ingress packet.
	 */
	if (info->type == TYPE_ADDRESS_RESTRICTED) {
		if (nte->dest_addr != iph->saddr) {
			DEBUGP("nattype_packet_in_match: dest/src check" \
				" failed: dest_addr: %pI4, src dest: %pI4\n",
				&nte->dest_addr, &iph->saddr);
			return false;
		}
	}

	/*
	 * Obtain the destination port value for TCP or UDP.  The nattype
	 * entries are stored in native (not host).
	 */
	if (iph->protocol == IPPROTO_TCP) {
		struct tcphdr _tcph;
		struct tcphdr *tcph;
		tcph = skb_header_pointer(skb, ip_hdrlen(skb),
			sizeof(_tcph), &_tcph);
		if (!tcph)
			return false;
		dst_port = tcph->dest;
	} else if (iph->protocol == IPPROTO_UDP) {
		struct udphdr _udph;
		struct udphdr *udph;
		udph = skb_header_pointer(skb, ip_hdrlen(skb),
			sizeof(_udph), &_udph);
		if (!udph)
			return false;
		dst_port = udph->dest;
	}

	/*
	 * Our NAT port must match the ingress pacekt's destination packet.
	 */
	if (nte->nat_port != dst_port) {
		DEBUGP("nattype_packet_in_match fail: nat port: %d," \
			" dest_port: %d\n",
			ntohs(nte->nat_port), ntohs(dst_port));
		return false;
	}

	/*
	 * In either EI or AR mode, the ingress packet's src port
	 * can be anything.
	 */
	nattype_nte_debug_print(nte, "INGRESS MATCH");
	return true;
}

/*
 * nattype_compare
 *	Compare two entries, return true if relevant fields are the same.
 */
static bool nattype_compare(struct ipt_nattype *n1, struct ipt_nattype *n2,
		const struct ipt_nattype_info *info)
{
	/*
	 * Protocol compare.
	 */
	if (n1->proto != n2->proto) {
		DEBUGP("nattype_compare: protocol mismatch: %d:%d\n",
				n1->proto, n2->proto);
		return false;
	}

	/*
	 * LAN Source compare.
	 * Since we always keep min/max values the same,
	 * just compare the min values.
	 */
	if (n1->range.min_ip != n2->range.min_ip) {
		DEBUGP("nattype_compare: r.min_ip mismatch: %pI4:%pI4\n",
				&n1->range.min_ip, &n2->range.min_ip);
		return false;
	}

	if (n1->range.min.all != n2->range.min.all) {
		DEBUGP("nattype_compare: r.min mismatch: %d:%d\n",
				ntohs(n1->range.min.all),
				ntohs(n2->range.min.all));
		return false;
	}

	/*
	 * NAT port
	 */
	if (n1->nat_port != n2->nat_port) {
		DEBUGP("nattype_compare: nat_port mistmatch: %d:%d\n",
				ntohs(n1->nat_port), ntohs(n2->nat_port));
		return false;
	}

	/*
	 * Destination Comapre for Address Restricted Cone NAT.
	 */
	if ((info->type == TYPE_ADDRESS_RESTRICTED) &&
		(n1->dest_addr != n2->dest_addr)) {
		DEBUGP("nattype_compare: dest_addr mismatch: %pI4:%pI4\n",
			&n1->dest_addr, &n2->dest_addr);
		return false;
	}

	return true;
}

/*
 * nattype_nat()
 *	Ingress packet on PRE_ROUTING hook, find match, update conntrack
 *      to allow
 */
static unsigned int nattype_nat(struct sk_buff *skb,
				const struct xt_action_param *par)
{
	struct ipt_nattype *nte;

	if (par->hooknum != NF_INET_PRE_ROUTING)
		return XT_CONTINUE;
	spin_lock_bh(&nattype_lock);
	list_for_each_entry(nte, &nattype_list, list) {
		struct nf_conn *ct;
		enum ip_conntrack_info ctinfo;
		struct nf_nat_ipv4_range newrange;
		unsigned int ret;

		if (!nattype_packet_in_match(nte, skb, par->targinfo))
			continue;

		/*
		 * Copy the LAN source data into the ingress' pacekts
		 * conntrack in the reply direction.
		 */
		newrange = nte->range;
		spin_unlock_bh(&nattype_lock);

		/*
		 * Find the ingress packet's conntrack.
		 */
		ct = nf_ct_get(skb, &ctinfo);
		if (!ct) {
			DEBUGP("ingress packet conntrack not found\n");
			return XT_CONTINUE;
		}

		/*
		 * Refresh the timer, if we fail, break
		 * out and forward fail as though we never
		 * found the entry.
		 */
		if (!nattype_refresh_timer((unsigned long)nte,
				jiffies + nte->timeout_value))
			break;

		/*
		 * Expand the ingress conntrack to include the reply as source
		 */
		DEBUGP("Expand ingress conntrack=%p, type=%d, src[%pI4:%d]\n",
			ct, ctinfo, &newrange.min_ip, ntohs(newrange.min.all));
		ct->nattype_entry = (unsigned long)nte;
		ret = nf_nat_setup_info(ct, &newrange, NF_NAT_MANIP_DST);
		DEBUGP("Expand returned: %d\n", ret);
		return ret;
	}
	spin_unlock_bh(&nattype_lock);
	return XT_CONTINUE;
}

/*
 * nattype_forward()
 *	Ingress and Egress packet forwarding hook
 */
static unsigned int nattype_forward(struct sk_buff *skb,
				const struct xt_action_param *par)
{
	const struct iphdr *iph = ip_hdr(skb);
	void *protoh = (void *)iph + iph->ihl * 4;
	struct ipt_nattype *nte;
	struct ipt_nattype *nte2;
	struct nf_conn *ct;
	enum ip_conntrack_info ctinfo;
	const struct ipt_nattype_info *info = par->targinfo;
	uint16_t nat_port;
	enum ip_conntrack_dir dir;


	if (par->hooknum != NF_INET_FORWARD)
		return XT_CONTINUE;

	/*
	 * Egress packet, create a new rule in our list.  If conntrack does
	 * not have an entry, skip this packet.
	 */
	ct = nf_ct_get(skb, &ctinfo);
	if (!ct)
		return XT_CONTINUE;

	/*
	 * Ingress packet, refresh the timer if we find an entry.
	 */
	if (info->mode == MODE_FORWARD_IN) {
		spin_lock_bh(&nattype_lock);
		list_for_each_entry(nte, &nattype_list, list) {
			/*
			 * Compare the ingress packet with the existing
			 * entries looking for a match.
			 */
			if (!nattype_packet_in_match(nte, skb, info))
				continue;
			spin_unlock_bh(&nattype_lock);
			/*
			 * Refresh the timer, if we fail, break
			 * out and forward fail as though we never
			 * found the entry.
			 */
			if (!nattype_refresh_timer((unsigned long)nte,
					ct->timeout.expires))
				break;
			/*
			 * The entry is found and refreshed, the
			 * entry values should not change so print
			 * them outside the lock.
			 */
			nattype_nte_debug_print(nte, "refresh");
			DEBUGP("FORWARD_IN_ACCEPT\n");
			return NF_ACCEPT;
		}
		spin_unlock_bh(&nattype_lock);
		DEBUGP("FORWARD_IN_FAIL\n");
		return XT_CONTINUE;
	}

	dir = CTINFO2DIR(ctinfo);

	nat_port = ct->tuplehash[!dir].tuple.dst.u.all;

	/*
	 * Allocate a new entry
	 */
	nte = kzalloc(sizeof(struct ipt_nattype), GFP_ATOMIC | __GFP_NOWARN);
	if (!nte) {
		DEBUGP("kernel malloc fail\n");
		return XT_CONTINUE;
	}

	INIT_LIST_HEAD(&nte->list);

	nte->proto = iph->protocol;
	nte->nat_port = nat_port;
	nte->dest_addr = iph->daddr;
	nte->range.max_ip = nte->range.min_ip = iph->saddr;

	/*
	 * TOOD: Would it be better to get this information from the
	 * conntrack instead of the headers.
	 */
	if (iph->protocol == IPPROTO_TCP) {
		nte->range.max.tcp.port = nte->range.min.tcp.port =
					((struct tcphdr *)protoh)->source;
		nte->dest_port = ((struct tcphdr *)protoh)->dest;
	} else if (iph->protocol == IPPROTO_UDP) {
		nte->range.max.udp.port = nte->range.min.udp.port =
					((struct udphdr *)protoh)->source;
		nte->dest_port = ((struct udphdr *)protoh)->dest;
	}
	nte->range.flags = (NF_NAT_RANGE_MAP_IPS |
			NF_NAT_RANGE_PROTO_SPECIFIED);

	/*
	 * Initilize the self-destruct timer.
	 */
	init_timer(&nte->timeout);
	nte->timeout.data = (unsigned long)nte;
	nte->timeout.function = nattype_timer_timeout;

	/*
	 * We have created the new nte; however, it might not be unique.
	 * Search the list for a matching entry.  If found, throw away
	 * the new entry and refresh the old.  If not found, atomically
	 * insert the new entry on the list.
	 */
	spin_lock_bh(&nattype_lock);
	list_for_each_entry(nte2, &nattype_list, list) {
		if (!nattype_compare(nte, nte2, info))
			continue;
		spin_unlock_bh(&nattype_lock);
		/*
		 * If we can not refresh this entry, insert our new
		 * entry as this one is timed out and will be removed
		 * from the list shortly.
		 */
		nte2->timeout_value = ct->timeout.expires - jiffies;
		if (!nattype_refresh_timer((unsigned long)nte2,
				ct->timeout.expires))
			break;
		/*
		 * Found and refreshed an existing entry.  Its values
		 * do not change so print the values outside of the lock.
		 *
		 * Free up the new entry.
		 */
		nattype_nte_debug_print(nte2, "refresh");
		nattype_free(nte);
		return XT_CONTINUE;
	}

	/*
	 * Add the new entry to the list.
	 */
	nte->timeout_value = ct->timeout.expires - jiffies;
	nte->timeout.expires = ct->timeout.expires;
	add_timer(&nte->timeout);
	list_add(&nte->list, &nattype_list);
	ct->nattype_entry = (unsigned long)nte;
	nte->nattype_cookie = NATTYPE_COOKIE;
	spin_unlock_bh(&nattype_lock);
	nattype_nte_debug_print(nte, "ADD");
	return XT_CONTINUE;
}

/*
 * nattype_target()
 *	One of the iptables hooks has a packet for us to analyze, do so.
 */
static unsigned int nattype_target(struct sk_buff *skb,
				const struct xt_action_param *par)
{
	const struct ipt_nattype_info *info = par->targinfo;
	const struct iphdr *iph = ip_hdr(skb);

	/*
	 * The default behavior for Linux is PORT and ADDRESS restricted. So
	 * we do not need to create rules/entries if we are in that mode.
	 */
	if (info->type == TYPE_PORT_ADDRESS_RESTRICTED)
		return XT_CONTINUE;

	/*
	 * Check if we have enough data in the skb.
	 */
	if (skb->len < ip_hdrlen(skb))
		return XT_CONTINUE;

	/*
	 * We can not perform endpoint filtering on anything but UDP and TCP.
	 */
	if ((iph->protocol != IPPROTO_TCP) && (iph->protocol != IPPROTO_UDP))
		return XT_CONTINUE;

	/*
	 * Check for LAND attack and ignore.
	 */
	if (iph->daddr == iph->saddr)
		return XT_CONTINUE;

	/*
	 * Check that we have valid source and destination addresses.
	 */
	if ((iph->daddr == (__be32)0) || (iph->saddr == (__be32)0))
		return XT_CONTINUE;

	DEBUGP("nattype_target: type = %s, mode = %s\n",
		types[info->type], modes[info->mode]);

	/*
	 * TODO: why have mode at all since par->hooknum provides
	 * this information?
	 */
	switch (info->mode) {
	case MODE_DNAT:
		return nattype_nat(skb, par);
	case MODE_FORWARD_OUT:
	case MODE_FORWARD_IN:
		return nattype_forward(skb, par);
	}
	return XT_CONTINUE;
}

/*
 * nattype_check()
 *	check info (mode/type) set by iptables.
 */
static int nattype_check(const struct xt_tgchk_param *par)
{
	const struct ipt_nattype_info *info = par->targinfo;
	struct list_head *cur, *tmp;

	if ((info->type != TYPE_PORT_ADDRESS_RESTRICTED) &&
		(info->type != TYPE_ENDPOINT_INDEPENDENT) &&
		(info->type != TYPE_ADDRESS_RESTRICTED)) {
		DEBUGP("nattype_check: unknown type: %d\n", info->type);
		return -EINVAL;
	}

	if (info->mode != MODE_DNAT && info->mode != MODE_FORWARD_IN &&
		info->mode != MODE_FORWARD_OUT) {
		DEBUGP("nattype_check: unknown mode - %d.\n", info->mode);
		return -EINVAL;
	}

	DEBUGP("nattype_check: type = %s, mode = %s\n",
		types[info->type], modes[info->mode]);

	if (par->hook_mask & ~((1 << NF_INET_PRE_ROUTING) |
		(1 << NF_INET_FORWARD))) {
		DEBUGP("nattype_check: bad hooks %x.\n", par->hook_mask);
		return -EINVAL;
	}

	/*
	 * Remove all entries from the nattype list.
	 */
drain:
	spin_lock_bh(&nattype_lock);
	list_for_each_safe(cur, tmp, &nattype_list) {
		struct ipt_nattype *nte = (void *)cur;

		/*
		 * If the timeout is in process, it will tear
		 * us down.  Since it is waiting on the spinlock
		 * we have to give up the spinlock to give the
		 * timeout on another CPU a chance to run.
		 */
		if (!del_timer(&nte->timeout)) {
			spin_unlock_bh(&nattype_lock);
			goto drain;
		}

		DEBUGP("%p: removing from list\n", nte);
		list_del(&nte->list);
		spin_unlock_bh(&nattype_lock);
		nattype_free(nte);
		goto drain;
	}
	spin_unlock_bh(&nattype_lock);
	return 0;
}

static struct xt_target nattype = {
	.name		= "NATTYPE",
	.family		= NFPROTO_IPV4,
	.target		= nattype_target,
	.checkentry	= nattype_check,
	.targetsize	= sizeof(struct ipt_nattype_info),
	.hooks		= ((1 << NF_INET_PRE_ROUTING) |
				(1 << NF_INET_FORWARD)),
	.me		= THIS_MODULE,
};

static int __init init(void)
{
	return xt_register_target(&nattype);
}

static void __exit fini(void)
{
	xt_unregister_target(&nattype);
}

module_init(init);
module_exit(fini);

MODULE_LICENSE("GPL");
