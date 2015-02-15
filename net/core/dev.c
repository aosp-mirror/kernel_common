/*
 * 	NET3	Protocol independent device support routines.
 *
 *		This program is free software; you can redistribute it and/or
 *		modify it under the terms of the GNU General Public License
 *		as published by the Free Software Foundation; either version
 *		2 of the License, or (at your option) any later version.
 *
 *	Derived from the non IP parts of dev.c 1.0.19
 * 		Authors:	Ross Biro
 *				Fred N. van Kempen, <waltje@uWalt.NL.Mugnet.ORG>
 *				Mark Evans, <evansmp@uhura.aston.ac.uk>
 *
 *	Additional Authors:
 *		Florian la Roche <rzsfl@rz.uni-sb.de>
 *		Alan Cox <gw4pts@gw4pts.ampr.org>
 *		David Hinds <dahinds@users.sourceforge.net>
 *		Alexey Kuznetsov <kuznet@ms2.inr.ac.ru>
 *		Adam Sulmicki <adam@cfar.umd.edu>
 *              Pekka Riikonen <priikone@poesidon.pspt.fi>
 *
 *	Changes:
 *              D.J. Barrow     :       Fixed bug where dev->refcnt gets set
 *              			to 2 if register_netdev gets called
 *              			before net_dev_init & also removed a
 *              			few lines of code in the process.
 *		Alan Cox	:	device private ioctl copies fields back.
 *		Alan Cox	:	Transmit queue code does relevant
 *					stunts to keep the queue safe.
 *		Alan Cox	:	Fixed double lock.
 *		Alan Cox	:	Fixed promisc NULL pointer trap
 *		????????	:	Support the full private ioctl range
 *		Alan Cox	:	Moved ioctl permission check into
 *					drivers
 *		Tim Kordas	:	SIOCADDMULTI/SIOCDELMULTI
 *		Alan Cox	:	100 backlog just doesn't cut it when
 *					you start doing multicast video 8)
 *		Alan Cox	:	Rewrote net_bh and list manager.
 *		Alan Cox	: 	Fix ETH_P_ALL echoback lengths.
 *		Alan Cox	:	Took out transmit every packet pass
 *					Saved a few bytes in the ioctl handler
 *		Alan Cox	:	Network driver sets packet type before
 *					calling netif_rx. Saves a function
 *					call a packet.
 *		Alan Cox	:	Hashed net_bh()
 *		Richard Kooijman:	Timestamp fixes.
 *		Alan Cox	:	Wrong field in SIOCGIFDSTADDR
 *		Alan Cox	:	Device lock protection.
 *		Alan Cox	: 	Fixed nasty side effect of device close
 *					changes.
 *		Rudi Cilibrasi	:	Pass the right thing to
 *					set_mac_address()
 *		Dave Miller	:	32bit quantity for the device lock to
 *					make it work out on a Sparc.
 *		Bjorn Ekwall	:	Added KERNELD hack.
 *		Alan Cox	:	Cleaned up the backlog initialise.
 *		Craig Metz	:	SIOCGIFCONF fix if space for under
 *					1 device.
 *	    Thomas Bogendoerfer :	Return ENODEV for dev_open, if there
 *					is no device open function.
 *		Andi Kleen	:	Fix error reporting for SIOCGIFCONF
 *	    Michael Chastain	:	Fix signed/unsigned for SIOCGIFCONF
 *		Cyrus Durgin	:	Cleaned for KMOD
 *		Adam Sulmicki   :	Bug Fix : Network Device Unload
 *					A network device unload needs to purge
 *					the backlog queue.
 *	Paul Rusty Russell	:	SIOCSIFNAME
 *              Pekka Riikonen  :	Netdev boot-time settings code
 *              Andrew Morton   :       Make unregister_netdevice wait
 *              			indefinitely on dev->refcnt
 * 		J Hadi Salim	:	- Backlog queue sampling
 *				        - netif_rx() feedback
 */

#include <asm/uaccess.h>
#include <linux/bitops.h>
#include <linux/capability.h>
#include <linux/cpu.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/hash.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/socket.h>
#include <linux/sockios.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/if_ether.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/notifier.h>
#include <linux/skbuff.h>
#include <net/net_namespace.h>
#include <net/sock.h>
#include <linux/rtnetlink.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/stat.h>
#include <net/dst.h>
#include <net/pkt_sched.h>
#include <net/checksum.h>
#include <net/xfrm.h>
#include <linux/highmem.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/module.h>
#include <linux/netpoll.h>
#include <linux/rcupdate.h>
#include <linux/delay.h>
#include <net/wext.h>
#include <net/iw_handler.h>
#include <asm/current.h>
#include <linux/audit.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/if_arp.h>
#include <linux/if_vlan.h>
#include <linux/ip.h>
#include <net/ip.h>
#include <linux/ipv6.h>
#include <linux/in.h>
#include <linux/jhash.h>
#include <linux/random.h>
#include <trace/events/napi.h>
#include <trace/events/net.h>
#include <trace/events/skb.h>
#include <linux/pci.h>
#include <linux/inetdevice.h>
#include <linux/cpu_rmap.h>
#include <linux/net_tstamp.h>
#include <linux/static_key.h>
#include <net/flow_keys.h>

#include "net-sysfs.h"

#define MAX_GRO_SKBS 8

#define GRO_MAX_HEAD (MAX_HEADER + 128)


#define PTYPE_HASH_SIZE	(16)
#define PTYPE_HASH_MASK	(PTYPE_HASH_SIZE - 1)

static DEFINE_SPINLOCK(ptype_lock);
static struct list_head ptype_base[PTYPE_HASH_SIZE] __read_mostly;
static struct list_head ptype_all __read_mostly;	

DEFINE_RWLOCK(dev_base_lock);
EXPORT_SYMBOL(dev_base_lock);

static inline void dev_base_seq_inc(struct net *net)
{
	while (++net->dev_base_seq == 0);
}

static inline struct hlist_head *dev_name_hash(struct net *net, const char *name)
{
	unsigned hash = full_name_hash(name, strnlen(name, IFNAMSIZ));
	return &net->dev_name_head[hash_32(hash, NETDEV_HASHBITS)];
}

static inline struct hlist_head *dev_index_hash(struct net *net, int ifindex)
{
	return &net->dev_index_head[ifindex & (NETDEV_HASHENTRIES - 1)];
}

static inline void rps_lock(struct softnet_data *sd)
{
#ifdef CONFIG_RPS
	spin_lock(&sd->input_pkt_queue.lock);
#endif
}

static inline void rps_unlock(struct softnet_data *sd)
{
#ifdef CONFIG_RPS
	spin_unlock(&sd->input_pkt_queue.lock);
#endif
}

static int list_netdevice(struct net_device *dev)
{
	struct net *net = dev_net(dev);

	ASSERT_RTNL();

	write_lock_bh(&dev_base_lock);
	list_add_tail_rcu(&dev->dev_list, &net->dev_base_head);
	hlist_add_head_rcu(&dev->name_hlist, dev_name_hash(net, dev->name));
	hlist_add_head_rcu(&dev->index_hlist,
			   dev_index_hash(net, dev->ifindex));
	write_unlock_bh(&dev_base_lock);

	dev_base_seq_inc(net);

	return 0;
}

static void unlist_netdevice(struct net_device *dev)
{
	ASSERT_RTNL();

	
	write_lock_bh(&dev_base_lock);
	list_del_rcu(&dev->dev_list);
	hlist_del_rcu(&dev->name_hlist);
	hlist_del_rcu(&dev->index_hlist);
	write_unlock_bh(&dev_base_lock);

	dev_base_seq_inc(dev_net(dev));
}


static RAW_NOTIFIER_HEAD(netdev_chain);


DEFINE_PER_CPU_ALIGNED(struct softnet_data, softnet_data);
EXPORT_PER_CPU_SYMBOL(softnet_data);

#ifdef CONFIG_LOCKDEP
static const unsigned short netdev_lock_type[] =
	{ARPHRD_NETROM, ARPHRD_ETHER, ARPHRD_EETHER, ARPHRD_AX25,
	 ARPHRD_PRONET, ARPHRD_CHAOS, ARPHRD_IEEE802, ARPHRD_ARCNET,
	 ARPHRD_APPLETLK, ARPHRD_DLCI, ARPHRD_ATM, ARPHRD_METRICOM,
	 ARPHRD_IEEE1394, ARPHRD_EUI64, ARPHRD_INFINIBAND, ARPHRD_SLIP,
	 ARPHRD_CSLIP, ARPHRD_SLIP6, ARPHRD_CSLIP6, ARPHRD_RSRVD,
	 ARPHRD_ADAPT, ARPHRD_ROSE, ARPHRD_X25, ARPHRD_HWX25,
	 ARPHRD_PPP, ARPHRD_CISCO, ARPHRD_LAPB, ARPHRD_DDCMP,
	 ARPHRD_RAWHDLC, ARPHRD_TUNNEL, ARPHRD_TUNNEL6, ARPHRD_FRAD,
	 ARPHRD_SKIP, ARPHRD_LOOPBACK, ARPHRD_LOCALTLK, ARPHRD_FDDI,
	 ARPHRD_BIF, ARPHRD_SIT, ARPHRD_IPDDP, ARPHRD_IPGRE,
	 ARPHRD_PIMREG, ARPHRD_HIPPI, ARPHRD_ASH, ARPHRD_ECONET,
	 ARPHRD_IRDA, ARPHRD_FCPP, ARPHRD_FCAL, ARPHRD_FCPL,
	 ARPHRD_FCFABRIC, ARPHRD_IEEE802_TR, ARPHRD_IEEE80211,
	 ARPHRD_IEEE80211_PRISM, ARPHRD_IEEE80211_RADIOTAP, ARPHRD_PHONET,
	 ARPHRD_PHONET_PIPE, ARPHRD_IEEE802154,
	 ARPHRD_VOID, ARPHRD_NONE};

static const char *const netdev_lock_name[] =
	{"_xmit_NETROM", "_xmit_ETHER", "_xmit_EETHER", "_xmit_AX25",
	 "_xmit_PRONET", "_xmit_CHAOS", "_xmit_IEEE802", "_xmit_ARCNET",
	 "_xmit_APPLETLK", "_xmit_DLCI", "_xmit_ATM", "_xmit_METRICOM",
	 "_xmit_IEEE1394", "_xmit_EUI64", "_xmit_INFINIBAND", "_xmit_SLIP",
	 "_xmit_CSLIP", "_xmit_SLIP6", "_xmit_CSLIP6", "_xmit_RSRVD",
	 "_xmit_ADAPT", "_xmit_ROSE", "_xmit_X25", "_xmit_HWX25",
	 "_xmit_PPP", "_xmit_CISCO", "_xmit_LAPB", "_xmit_DDCMP",
	 "_xmit_RAWHDLC", "_xmit_TUNNEL", "_xmit_TUNNEL6", "_xmit_FRAD",
	 "_xmit_SKIP", "_xmit_LOOPBACK", "_xmit_LOCALTLK", "_xmit_FDDI",
	 "_xmit_BIF", "_xmit_SIT", "_xmit_IPDDP", "_xmit_IPGRE",
	 "_xmit_PIMREG", "_xmit_HIPPI", "_xmit_ASH", "_xmit_ECONET",
	 "_xmit_IRDA", "_xmit_FCPP", "_xmit_FCAL", "_xmit_FCPL",
	 "_xmit_FCFABRIC", "_xmit_IEEE802_TR", "_xmit_IEEE80211",
	 "_xmit_IEEE80211_PRISM", "_xmit_IEEE80211_RADIOTAP", "_xmit_PHONET",
	 "_xmit_PHONET_PIPE", "_xmit_IEEE802154",
	 "_xmit_VOID", "_xmit_NONE"};

static struct lock_class_key netdev_xmit_lock_key[ARRAY_SIZE(netdev_lock_type)];
static struct lock_class_key netdev_addr_lock_key[ARRAY_SIZE(netdev_lock_type)];

static inline unsigned short netdev_lock_pos(unsigned short dev_type)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(netdev_lock_type); i++)
		if (netdev_lock_type[i] == dev_type)
			return i;
	
	return ARRAY_SIZE(netdev_lock_type) - 1;
}

static inline void netdev_set_xmit_lockdep_class(spinlock_t *lock,
						 unsigned short dev_type)
{
	int i;

	i = netdev_lock_pos(dev_type);
	lockdep_set_class_and_name(lock, &netdev_xmit_lock_key[i],
				   netdev_lock_name[i]);
}

static inline void netdev_set_addr_lockdep_class(struct net_device *dev)
{
	int i;

	i = netdev_lock_pos(dev->type);
	lockdep_set_class_and_name(&dev->addr_list_lock,
				   &netdev_addr_lock_key[i],
				   netdev_lock_name[i]);
}
#else
static inline void netdev_set_xmit_lockdep_class(spinlock_t *lock,
						 unsigned short dev_type)
{
}
static inline void netdev_set_addr_lockdep_class(struct net_device *dev)
{
}
#endif



static inline struct list_head *ptype_head(const struct packet_type *pt)
{
	if (pt->type == htons(ETH_P_ALL))
		return &ptype_all;
	else
		return &ptype_base[ntohs(pt->type) & PTYPE_HASH_MASK];
}


void dev_add_pack(struct packet_type *pt)
{
	struct list_head *head = ptype_head(pt);

	spin_lock(&ptype_lock);
	list_add_rcu(&pt->list, head);
	spin_unlock(&ptype_lock);
}
EXPORT_SYMBOL(dev_add_pack);

void __dev_remove_pack(struct packet_type *pt)
{
	struct list_head *head = ptype_head(pt);
	struct packet_type *pt1;

	spin_lock(&ptype_lock);

	list_for_each_entry(pt1, head, list) {
		if (pt == pt1) {
			list_del_rcu(&pt->list);
			goto out;
		}
	}

	pr_warn("dev_remove_pack: %p not found\n", pt);
out:
	spin_unlock(&ptype_lock);
}
EXPORT_SYMBOL(__dev_remove_pack);

void dev_remove_pack(struct packet_type *pt)
{
	__dev_remove_pack(pt);

	synchronize_net();
}
EXPORT_SYMBOL(dev_remove_pack);


static struct netdev_boot_setup dev_boot_setup[NETDEV_BOOT_SETUP_MAX];

static int netdev_boot_setup_add(char *name, struct ifmap *map)
{
	struct netdev_boot_setup *s;
	int i;

	s = dev_boot_setup;
	for (i = 0; i < NETDEV_BOOT_SETUP_MAX; i++) {
		if (s[i].name[0] == '\0' || s[i].name[0] == ' ') {
			memset(s[i].name, 0, sizeof(s[i].name));
			strlcpy(s[i].name, name, IFNAMSIZ);
			memcpy(&s[i].map, map, sizeof(s[i].map));
			break;
		}
	}

	return i >= NETDEV_BOOT_SETUP_MAX ? 0 : 1;
}

int netdev_boot_setup_check(struct net_device *dev)
{
	struct netdev_boot_setup *s = dev_boot_setup;
	int i;

	for (i = 0; i < NETDEV_BOOT_SETUP_MAX; i++) {
		if (s[i].name[0] != '\0' && s[i].name[0] != ' ' &&
		    !strcmp(dev->name, s[i].name)) {
			dev->irq 	= s[i].map.irq;
			dev->base_addr 	= s[i].map.base_addr;
			dev->mem_start 	= s[i].map.mem_start;
			dev->mem_end 	= s[i].map.mem_end;
			return 1;
		}
	}
	return 0;
}
EXPORT_SYMBOL(netdev_boot_setup_check);


unsigned long netdev_boot_base(const char *prefix, int unit)
{
	const struct netdev_boot_setup *s = dev_boot_setup;
	char name[IFNAMSIZ];
	int i;

	sprintf(name, "%s%d", prefix, unit);

	if (__dev_get_by_name(&init_net, name))
		return 1;

	for (i = 0; i < NETDEV_BOOT_SETUP_MAX; i++)
		if (!strcmp(name, s[i].name))
			return s[i].map.base_addr;
	return 0;
}

int __init netdev_boot_setup(char *str)
{
	int ints[5];
	struct ifmap map;

	str = get_options(str, ARRAY_SIZE(ints), ints);
	if (!str || !*str)
		return 0;

	
	memset(&map, 0, sizeof(map));
	if (ints[0] > 0)
		map.irq = ints[1];
	if (ints[0] > 1)
		map.base_addr = ints[2];
	if (ints[0] > 2)
		map.mem_start = ints[3];
	if (ints[0] > 3)
		map.mem_end = ints[4];

	
	return netdev_boot_setup_add(str, &map);
}

__setup("netdev=", netdev_boot_setup);



struct net_device *__dev_get_by_name(struct net *net, const char *name)
{
	struct hlist_node *p;
	struct net_device *dev;
	struct hlist_head *head = dev_name_hash(net, name);

	hlist_for_each_entry(dev, p, head, name_hlist)
		if (!strncmp(dev->name, name, IFNAMSIZ))
			return dev;

	return NULL;
}
EXPORT_SYMBOL(__dev_get_by_name);


struct net_device *dev_get_by_name_rcu(struct net *net, const char *name)
{
	struct hlist_node *p;
	struct net_device *dev;
	struct hlist_head *head = dev_name_hash(net, name);

	hlist_for_each_entry_rcu(dev, p, head, name_hlist)
		if (!strncmp(dev->name, name, IFNAMSIZ))
			return dev;

	return NULL;
}
EXPORT_SYMBOL(dev_get_by_name_rcu);


struct net_device *dev_get_by_name(struct net *net, const char *name)
{
	struct net_device *dev;

	rcu_read_lock();
	dev = dev_get_by_name_rcu(net, name);
	if (dev)
		dev_hold(dev);
	rcu_read_unlock();
	return dev;
}
EXPORT_SYMBOL(dev_get_by_name);


struct net_device *__dev_get_by_index(struct net *net, int ifindex)
{
	struct hlist_node *p;
	struct net_device *dev;
	struct hlist_head *head = dev_index_hash(net, ifindex);

	hlist_for_each_entry(dev, p, head, index_hlist)
		if (dev->ifindex == ifindex)
			return dev;

	return NULL;
}
EXPORT_SYMBOL(__dev_get_by_index);


struct net_device *dev_get_by_index_rcu(struct net *net, int ifindex)
{
	struct hlist_node *p;
	struct net_device *dev;
	struct hlist_head *head = dev_index_hash(net, ifindex);

	hlist_for_each_entry_rcu(dev, p, head, index_hlist)
		if (dev->ifindex == ifindex)
			return dev;

	return NULL;
}
EXPORT_SYMBOL(dev_get_by_index_rcu);



struct net_device *dev_get_by_index(struct net *net, int ifindex)
{
	struct net_device *dev;

	rcu_read_lock();
	dev = dev_get_by_index_rcu(net, ifindex);
	if (dev)
		dev_hold(dev);
	rcu_read_unlock();
	return dev;
}
EXPORT_SYMBOL(dev_get_by_index);


struct net_device *dev_getbyhwaddr_rcu(struct net *net, unsigned short type,
				       const char *ha)
{
	struct net_device *dev;

	for_each_netdev_rcu(net, dev)
		if (dev->type == type &&
		    !memcmp(dev->dev_addr, ha, dev->addr_len))
			return dev;

	return NULL;
}
EXPORT_SYMBOL(dev_getbyhwaddr_rcu);

struct net_device *__dev_getfirstbyhwtype(struct net *net, unsigned short type)
{
	struct net_device *dev;

	ASSERT_RTNL();
	for_each_netdev(net, dev)
		if (dev->type == type)
			return dev;

	return NULL;
}
EXPORT_SYMBOL(__dev_getfirstbyhwtype);

struct net_device *dev_getfirstbyhwtype(struct net *net, unsigned short type)
{
	struct net_device *dev, *ret = NULL;

	rcu_read_lock();
	for_each_netdev_rcu(net, dev)
		if (dev->type == type) {
			dev_hold(dev);
			ret = dev;
			break;
		}
	rcu_read_unlock();
	return ret;
}
EXPORT_SYMBOL(dev_getfirstbyhwtype);


struct net_device *dev_get_by_flags_rcu(struct net *net, unsigned short if_flags,
				    unsigned short mask)
{
	struct net_device *dev, *ret;

	ret = NULL;
	for_each_netdev_rcu(net, dev) {
		if (((dev->flags ^ if_flags) & mask) == 0) {
			ret = dev;
			break;
		}
	}
	return ret;
}
EXPORT_SYMBOL(dev_get_by_flags_rcu);

bool dev_valid_name(const char *name)
{
	if (*name == '\0')
		return false;
	if (strlen(name) >= IFNAMSIZ)
		return false;
	if (!strcmp(name, ".") || !strcmp(name, ".."))
		return false;

	while (*name) {
		if (*name == '/' || isspace(*name))
			return false;
		name++;
	}
	return true;
}
EXPORT_SYMBOL(dev_valid_name);


static int __dev_alloc_name(struct net *net, const char *name, char *buf)
{
	int i = 0;
	const char *p;
	const int max_netdevices = 8*PAGE_SIZE;
	unsigned long *inuse;
	struct net_device *d;

	p = strnchr(name, IFNAMSIZ-1, '%');
	if (p) {
		if (p[1] != 'd' || strchr(p + 2, '%'))
			return -EINVAL;

		
		inuse = (unsigned long *) get_zeroed_page(GFP_ATOMIC);
		if (!inuse)
			return -ENOMEM;

		for_each_netdev(net, d) {
			if (!sscanf(d->name, name, &i))
				continue;
			if (i < 0 || i >= max_netdevices)
				continue;

			
			snprintf(buf, IFNAMSIZ, name, i);
			if (!strncmp(buf, d->name, IFNAMSIZ))
				set_bit(i, inuse);
		}

		i = find_first_zero_bit(inuse, max_netdevices);
		free_page((unsigned long) inuse);
	}

	if (buf != name)
		snprintf(buf, IFNAMSIZ, name, i);
	if (!__dev_get_by_name(net, buf))
		return i;

	return -ENFILE;
}


int dev_alloc_name(struct net_device *dev, const char *name)
{
	char buf[IFNAMSIZ];
	struct net *net;
	int ret;

	BUG_ON(!dev_net(dev));
	net = dev_net(dev);
	ret = __dev_alloc_name(net, name, buf);
	if (ret >= 0)
		strlcpy(dev->name, buf, IFNAMSIZ);
	return ret;
}
EXPORT_SYMBOL(dev_alloc_name);

static int dev_get_valid_name(struct net_device *dev, const char *name)
{
	struct net *net;

	BUG_ON(!dev_net(dev));
	net = dev_net(dev);

	if (!dev_valid_name(name))
		return -EINVAL;

	if (strchr(name, '%'))
		return dev_alloc_name(dev, name);
	else if (__dev_get_by_name(net, name))
		return -EEXIST;
	else if (dev->name != name)
		strlcpy(dev->name, name, IFNAMSIZ);

	return 0;
}

int dev_change_name(struct net_device *dev, const char *newname)
{
	char oldname[IFNAMSIZ];
	int err = 0;
	int ret;
	struct net *net;

	ASSERT_RTNL();
	BUG_ON(!dev_net(dev));

	net = dev_net(dev);
	if (dev->flags & IFF_UP)
		return -EBUSY;

	if (strncmp(newname, dev->name, IFNAMSIZ) == 0)
		return 0;

	memcpy(oldname, dev->name, IFNAMSIZ);

	err = dev_get_valid_name(dev, newname);
	if (err < 0)
		return err;

rollback:
	ret = device_rename(&dev->dev, dev->name);
	if (ret) {
		memcpy(dev->name, oldname, IFNAMSIZ);
		return ret;
	}

	write_lock_bh(&dev_base_lock);
	hlist_del_rcu(&dev->name_hlist);
	write_unlock_bh(&dev_base_lock);

	synchronize_rcu();

	write_lock_bh(&dev_base_lock);
	hlist_add_head_rcu(&dev->name_hlist, dev_name_hash(net, dev->name));
	write_unlock_bh(&dev_base_lock);

	ret = call_netdevice_notifiers(NETDEV_CHANGENAME, dev);
	ret = notifier_to_errno(ret);

	if (ret) {
		
		if (err >= 0) {
			err = ret;
			memcpy(dev->name, oldname, IFNAMSIZ);
			goto rollback;
		} else {
			pr_err("%s: name change rollback failed: %d\n",
			       dev->name, ret);
		}
	}

	return err;
}

int dev_set_alias(struct net_device *dev, const char *alias, size_t len)
{
	ASSERT_RTNL();

	if (len >= IFALIASZ)
		return -EINVAL;

	if (!len) {
		if (dev->ifalias) {
			kfree(dev->ifalias);
			dev->ifalias = NULL;
		}
		return 0;
	}

	dev->ifalias = krealloc(dev->ifalias, len + 1, GFP_KERNEL);
	if (!dev->ifalias)
		return -ENOMEM;

	strlcpy(dev->ifalias, alias, len+1);
	return len;
}


void netdev_features_change(struct net_device *dev)
{
	call_netdevice_notifiers(NETDEV_FEAT_CHANGE, dev);
}
EXPORT_SYMBOL(netdev_features_change);

void netdev_state_change(struct net_device *dev)
{
	if (dev->flags & IFF_UP) {
		call_netdevice_notifiers(NETDEV_CHANGE, dev);
		rtmsg_ifinfo(RTM_NEWLINK, dev, 0);
	}
}
EXPORT_SYMBOL(netdev_state_change);

int netdev_bonding_change(struct net_device *dev, unsigned long event)
{
	return call_netdevice_notifiers(event, dev);
}
EXPORT_SYMBOL(netdev_bonding_change);


void dev_load(struct net *net, const char *name)
{
	struct net_device *dev;
	int no_module;

	rcu_read_lock();
	dev = dev_get_by_name_rcu(net, name);
	rcu_read_unlock();

	no_module = !dev;
	if (no_module && capable(CAP_NET_ADMIN))
		no_module = request_module("netdev-%s", name);
	if (no_module && capable(CAP_SYS_MODULE)) {
		if (!request_module("%s", name))
			pr_err("Loading kernel module for a network device with CAP_SYS_MODULE (deprecated).  Use CAP_NET_ADMIN and alias netdev-%s instead.\n",
			       name);
	}
}
EXPORT_SYMBOL(dev_load);

static int __dev_open(struct net_device *dev)
{
	const struct net_device_ops *ops = dev->netdev_ops;
	int ret;

	ASSERT_RTNL();

	if (!netif_device_present(dev))
		return -ENODEV;

	ret = call_netdevice_notifiers(NETDEV_PRE_UP, dev);
	ret = notifier_to_errno(ret);
	if (ret)
		return ret;

	set_bit(__LINK_STATE_START, &dev->state);

	if (ops->ndo_validate_addr)
		ret = ops->ndo_validate_addr(dev);

	if (!ret && ops->ndo_open)
		ret = ops->ndo_open(dev);

	if (ret)
		clear_bit(__LINK_STATE_START, &dev->state);
	else {
		dev->flags |= IFF_UP;
		net_dmaengine_get();
		dev_set_rx_mode(dev);
		dev_activate(dev);
	}

	return ret;
}

int dev_open(struct net_device *dev)
{
	int ret;

	if (dev->flags & IFF_UP)
		return 0;

	ret = __dev_open(dev);
	if (ret < 0)
		return ret;

	rtmsg_ifinfo(RTM_NEWLINK, dev, IFF_UP|IFF_RUNNING);
	call_netdevice_notifiers(NETDEV_UP, dev);

	return ret;
}
EXPORT_SYMBOL(dev_open);

static int __dev_close_many(struct list_head *head)
{
	struct net_device *dev;

	ASSERT_RTNL();
	might_sleep();

	list_for_each_entry(dev, head, unreg_list) {
		call_netdevice_notifiers(NETDEV_GOING_DOWN, dev);

		clear_bit(__LINK_STATE_START, &dev->state);

		smp_mb__after_clear_bit(); 
	}

	dev_deactivate_many(head);

	list_for_each_entry(dev, head, unreg_list) {
		const struct net_device_ops *ops = dev->netdev_ops;

		if (ops->ndo_stop)
			ops->ndo_stop(dev);

		dev->flags &= ~IFF_UP;
		net_dmaengine_put();
	}

	return 0;
}

static int __dev_close(struct net_device *dev)
{
	int retval;
	LIST_HEAD(single);

	list_add(&dev->unreg_list, &single);
	retval = __dev_close_many(&single);
	list_del(&single);
	return retval;
}

static int dev_close_many(struct list_head *head)
{
	struct net_device *dev, *tmp;
	LIST_HEAD(tmp_list);

	list_for_each_entry_safe(dev, tmp, head, unreg_list)
		if (!(dev->flags & IFF_UP))
			list_move(&dev->unreg_list, &tmp_list);

	__dev_close_many(head);

	list_for_each_entry(dev, head, unreg_list) {
		rtmsg_ifinfo(RTM_NEWLINK, dev, IFF_UP|IFF_RUNNING);
		call_netdevice_notifiers(NETDEV_DOWN, dev);
	}

	
	list_splice(&tmp_list, head);
	return 0;
}

int dev_close(struct net_device *dev)
{
	if (dev->flags & IFF_UP) {
		LIST_HEAD(single);

		list_add(&dev->unreg_list, &single);
		dev_close_many(&single);
		list_del(&single);
	}
	return 0;
}
EXPORT_SYMBOL(dev_close);


void dev_disable_lro(struct net_device *dev)
{
	if (is_vlan_dev(dev))
		dev = vlan_dev_real_dev(dev);

	dev->wanted_features &= ~NETIF_F_LRO;
	netdev_update_features(dev);

	if (unlikely(dev->features & NETIF_F_LRO))
		netdev_WARN(dev, "failed to disable LRO!\n");
}
EXPORT_SYMBOL(dev_disable_lro);


static int dev_boot_phase = 1;


int register_netdevice_notifier(struct notifier_block *nb)
{
	struct net_device *dev;
	struct net_device *last;
	struct net *net;
	int err;

	rtnl_lock();
	err = raw_notifier_chain_register(&netdev_chain, nb);
	if (err)
		goto unlock;
	if (dev_boot_phase)
		goto unlock;
	for_each_net(net) {
		for_each_netdev(net, dev) {
			err = nb->notifier_call(nb, NETDEV_REGISTER, dev);
			err = notifier_to_errno(err);
			if (err)
				goto rollback;

			if (!(dev->flags & IFF_UP))
				continue;

			nb->notifier_call(nb, NETDEV_UP, dev);
		}
	}

unlock:
	rtnl_unlock();
	return err;

rollback:
	last = dev;
	for_each_net(net) {
		for_each_netdev(net, dev) {
			if (dev == last)
				goto outroll;

			if (dev->flags & IFF_UP) {
				nb->notifier_call(nb, NETDEV_GOING_DOWN, dev);
				nb->notifier_call(nb, NETDEV_DOWN, dev);
			}
			nb->notifier_call(nb, NETDEV_UNREGISTER, dev);
			nb->notifier_call(nb, NETDEV_UNREGISTER_BATCH, dev);
		}
	}

outroll:
	raw_notifier_chain_unregister(&netdev_chain, nb);
	goto unlock;
}
EXPORT_SYMBOL(register_netdevice_notifier);


int unregister_netdevice_notifier(struct notifier_block *nb)
{
	struct net_device *dev;
	struct net *net;
	int err;

	rtnl_lock();
	err = raw_notifier_chain_unregister(&netdev_chain, nb);
	if (err)
		goto unlock;

	for_each_net(net) {
		for_each_netdev(net, dev) {
			if (dev->flags & IFF_UP) {
				nb->notifier_call(nb, NETDEV_GOING_DOWN, dev);
				nb->notifier_call(nb, NETDEV_DOWN, dev);
			}
			nb->notifier_call(nb, NETDEV_UNREGISTER, dev);
			nb->notifier_call(nb, NETDEV_UNREGISTER_BATCH, dev);
		}
	}
unlock:
	rtnl_unlock();
	return err;
}
EXPORT_SYMBOL(unregister_netdevice_notifier);


int call_netdevice_notifiers(unsigned long val, struct net_device *dev)
{
	ASSERT_RTNL();
	return raw_notifier_call_chain(&netdev_chain, val, dev);
}
EXPORT_SYMBOL(call_netdevice_notifiers);

static struct static_key netstamp_needed __read_mostly;
#ifdef HAVE_JUMP_LABEL
static atomic_t netstamp_needed_deferred;
#endif

void net_enable_timestamp(void)
{
#ifdef HAVE_JUMP_LABEL
	int deferred = atomic_xchg(&netstamp_needed_deferred, 0);

	if (deferred) {
		while (--deferred)
			static_key_slow_dec(&netstamp_needed);
		return;
	}
#endif
	WARN_ON(in_interrupt());
	static_key_slow_inc(&netstamp_needed);
}
EXPORT_SYMBOL(net_enable_timestamp);

void net_disable_timestamp(void)
{
#ifdef HAVE_JUMP_LABEL
	if (in_interrupt()) {
		atomic_inc(&netstamp_needed_deferred);
		return;
	}
#endif
	static_key_slow_dec(&netstamp_needed);
}
EXPORT_SYMBOL(net_disable_timestamp);

static inline void net_timestamp_set(struct sk_buff *skb)
{
	skb->tstamp.tv64 = 0;
	if (static_key_false(&netstamp_needed))
		__net_timestamp(skb);
}

#define net_timestamp_check(COND, SKB)			\
	if (static_key_false(&netstamp_needed)) {		\
		if ((COND) && !(SKB)->tstamp.tv64)	\
			__net_timestamp(SKB);		\
	}						\

static int net_hwtstamp_validate(struct ifreq *ifr)
{
	struct hwtstamp_config cfg;
	enum hwtstamp_tx_types tx_type;
	enum hwtstamp_rx_filters rx_filter;
	int tx_type_valid = 0;
	int rx_filter_valid = 0;

	if (copy_from_user(&cfg, ifr->ifr_data, sizeof(cfg)))
		return -EFAULT;

	if (cfg.flags) 
		return -EINVAL;

	tx_type = cfg.tx_type;
	rx_filter = cfg.rx_filter;

	switch (tx_type) {
	case HWTSTAMP_TX_OFF:
	case HWTSTAMP_TX_ON:
	case HWTSTAMP_TX_ONESTEP_SYNC:
		tx_type_valid = 1;
		break;
	}

	switch (rx_filter) {
	case HWTSTAMP_FILTER_NONE:
	case HWTSTAMP_FILTER_ALL:
	case HWTSTAMP_FILTER_SOME:
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		rx_filter_valid = 1;
		break;
	}

	if (!tx_type_valid || !rx_filter_valid)
		return -ERANGE;

	return 0;
}

static inline bool is_skb_forwardable(struct net_device *dev,
				      struct sk_buff *skb)
{
	unsigned int len;

	if (!(dev->flags & IFF_UP))
		return false;

	len = dev->mtu + dev->hard_header_len + VLAN_HLEN;
	if (skb->len <= len)
		return true;

	if (skb_is_gso(skb))
		return true;

	return false;
}

int dev_forward_skb(struct net_device *dev, struct sk_buff *skb)
{
	if (skb_shinfo(skb)->tx_flags & SKBTX_DEV_ZEROCOPY) {
		if (skb_copy_ubufs(skb, GFP_ATOMIC)) {
			atomic_long_inc(&dev->rx_dropped);
			kfree_skb(skb);
			return NET_RX_DROP;
		}
	}

	skb_orphan(skb);
	nf_reset(skb);

	if (unlikely(!is_skb_forwardable(dev, skb))) {
		atomic_long_inc(&dev->rx_dropped);
		kfree_skb(skb);
		return NET_RX_DROP;
	}
	skb->skb_iif = 0;
	skb->dev = dev;
	skb_dst_drop(skb);
	skb->tstamp.tv64 = 0;
	skb->pkt_type = PACKET_HOST;
	skb->protocol = eth_type_trans(skb, dev);
	skb->mark = 0;
	secpath_reset(skb);
	nf_reset(skb);
	return netif_rx(skb);
}
EXPORT_SYMBOL_GPL(dev_forward_skb);

static inline int deliver_skb(struct sk_buff *skb,
			      struct packet_type *pt_prev,
			      struct net_device *orig_dev)
{
	atomic_inc(&skb->users);
	return pt_prev->func(skb, skb->dev, pt_prev, orig_dev);
}


static void dev_queue_xmit_nit(struct sk_buff *skb, struct net_device *dev)
{
	struct packet_type *ptype;
	struct sk_buff *skb2 = NULL;
	struct packet_type *pt_prev = NULL;

	rcu_read_lock();
	list_for_each_entry_rcu(ptype, &ptype_all, list) {
		if ((ptype->dev == dev || !ptype->dev) &&
		    (ptype->af_packet_priv == NULL ||
		     (struct sock *)ptype->af_packet_priv != skb->sk)) {
			if (pt_prev) {
				deliver_skb(skb2, pt_prev, skb->dev);
				pt_prev = ptype;
				continue;
			}

			skb2 = skb_clone(skb, GFP_ATOMIC);
			if (!skb2)
				break;

			net_timestamp_set(skb2);

			skb_reset_mac_header(skb2);

			if (skb_network_header(skb2) < skb2->data ||
			    skb2->network_header > skb2->tail) {
				if (net_ratelimit())
					pr_crit("protocol %04x is buggy, dev %s\n",
						ntohs(skb2->protocol),
						dev->name);
				skb_reset_network_header(skb2);
			}

			skb2->transport_header = skb2->network_header;
			skb2->pkt_type = PACKET_OUTGOING;
			pt_prev = ptype;
		}
	}
	if (pt_prev)
		pt_prev->func(skb2, skb->dev, pt_prev, skb->dev);
	rcu_read_unlock();
}

static void netif_setup_tc(struct net_device *dev, unsigned int txq)
{
	int i;
	struct netdev_tc_txq *tc = &dev->tc_to_txq[0];

	
	if (tc->offset + tc->count > txq) {
		pr_warn("Number of in use tx queues changed invalidating tc mappings. Priority traffic classification disabled!\n");
		dev->num_tc = 0;
		return;
	}

	
	for (i = 1; i < TC_BITMASK + 1; i++) {
		int q = netdev_get_prio_tc_map(dev, i);

		tc = &dev->tc_to_txq[q];
		if (tc->offset + tc->count > txq) {
			pr_warn("Number of in use tx queues changed. Priority %i to tc mapping %i is no longer valid. Setting map to 0\n",
				i, q);
			netdev_set_prio_tc_map(dev, i, 0);
		}
	}
}

int netif_set_real_num_tx_queues(struct net_device *dev, unsigned int txq)
{
	int rc;

	if (txq < 1 || txq > dev->num_tx_queues)
		return -EINVAL;

	if (dev->reg_state == NETREG_REGISTERED ||
	    dev->reg_state == NETREG_UNREGISTERING) {
		ASSERT_RTNL();

		rc = netdev_queue_update_kobjects(dev, dev->real_num_tx_queues,
						  txq);
		if (rc)
			return rc;

		if (dev->num_tc)
			netif_setup_tc(dev, txq);

		if (txq < dev->real_num_tx_queues)
			qdisc_reset_all_tx_gt(dev, txq);
	}

	dev->real_num_tx_queues = txq;
	return 0;
}
EXPORT_SYMBOL(netif_set_real_num_tx_queues);

#ifdef CONFIG_RPS
int netif_set_real_num_rx_queues(struct net_device *dev, unsigned int rxq)
{
	int rc;

	if (rxq < 1 || rxq > dev->num_rx_queues)
		return -EINVAL;

	if (dev->reg_state == NETREG_REGISTERED) {
		ASSERT_RTNL();

		rc = net_rx_queue_update_kobjects(dev, dev->real_num_rx_queues,
						  rxq);
		if (rc)
			return rc;
	}

	dev->real_num_rx_queues = rxq;
	return 0;
}
EXPORT_SYMBOL(netif_set_real_num_rx_queues);
#endif

static inline void __netif_reschedule(struct Qdisc *q)
{
	struct softnet_data *sd;
	unsigned long flags;

	local_irq_save(flags);
	sd = &__get_cpu_var(softnet_data);
	q->next_sched = NULL;
	*sd->output_queue_tailp = q;
	sd->output_queue_tailp = &q->next_sched;
	raise_softirq_irqoff(NET_TX_SOFTIRQ);
	local_irq_restore(flags);
}

void __netif_schedule(struct Qdisc *q)
{
	if (!test_and_set_bit(__QDISC_STATE_SCHED, &q->state))
		__netif_reschedule(q);
}
EXPORT_SYMBOL(__netif_schedule);

void dev_kfree_skb_irq(struct sk_buff *skb)
{
	if (atomic_dec_and_test(&skb->users)) {
		struct softnet_data *sd;
		unsigned long flags;

		local_irq_save(flags);
		sd = &__get_cpu_var(softnet_data);
		skb->next = sd->completion_queue;
		sd->completion_queue = skb;
		raise_softirq_irqoff(NET_TX_SOFTIRQ);
		local_irq_restore(flags);
	}
}
EXPORT_SYMBOL(dev_kfree_skb_irq);

void dev_kfree_skb_any(struct sk_buff *skb)
{
	if (in_irq() || irqs_disabled())
		dev_kfree_skb_irq(skb);
	else
		dev_kfree_skb(skb);
}
EXPORT_SYMBOL(dev_kfree_skb_any);


void netif_device_detach(struct net_device *dev)
{
	if (test_and_clear_bit(__LINK_STATE_PRESENT, &dev->state) &&
	    netif_running(dev)) {
		netif_tx_stop_all_queues(dev);
	}
}
EXPORT_SYMBOL(netif_device_detach);

void netif_device_attach(struct net_device *dev)
{
	if (!test_and_set_bit(__LINK_STATE_PRESENT, &dev->state) &&
	    netif_running(dev)) {
		netif_tx_wake_all_queues(dev);
		__netdev_watchdog_up(dev);
	}
}
EXPORT_SYMBOL(netif_device_attach);

static void skb_warn_bad_offload(const struct sk_buff *skb)
{
	static const netdev_features_t null_features = 0;
	struct net_device *dev = skb->dev;
	const char *driver = "";

	if (dev && dev->dev.parent)
		driver = dev_driver_string(dev->dev.parent);

	WARN(1, "%s: caps=(%pNF, %pNF) len=%d data_len=%d gso_size=%d "
	     "gso_type=%d ip_summed=%d\n",
	     driver, dev ? &dev->features : &null_features,
	     skb->sk ? &skb->sk->sk_route_caps : &null_features,
	     skb->len, skb->data_len, skb_shinfo(skb)->gso_size,
	     skb_shinfo(skb)->gso_type, skb->ip_summed);
}

int skb_checksum_help(struct sk_buff *skb)
{
	__wsum csum;
	int ret = 0, offset;

	if (skb->ip_summed == CHECKSUM_COMPLETE)
		goto out_set_summed;

	if (unlikely(skb_shinfo(skb)->gso_size)) {
		skb_warn_bad_offload(skb);
		return -EINVAL;
	}

	offset = skb_checksum_start_offset(skb);
	BUG_ON(offset >= skb_headlen(skb));
	csum = skb_checksum(skb, offset, skb->len - offset, 0);

	offset += skb->csum_offset;
	BUG_ON(offset + sizeof(__sum16) > skb_headlen(skb));

	if (skb_cloned(skb) &&
	    !skb_clone_writable(skb, offset + sizeof(__sum16))) {
		ret = pskb_expand_head(skb, 0, 0, GFP_ATOMIC);
		if (ret)
			goto out;
	}

	*(__sum16 *)(skb->data + offset) = csum_fold(csum);
out_set_summed:
	skb->ip_summed = CHECKSUM_NONE;
out:
	return ret;
}
EXPORT_SYMBOL(skb_checksum_help);

struct sk_buff *skb_gso_segment(struct sk_buff *skb,
	netdev_features_t features)
{
	struct sk_buff *segs = ERR_PTR(-EPROTONOSUPPORT);
	struct packet_type *ptype;
	__be16 type = skb->protocol;
	int vlan_depth = ETH_HLEN;
	int err;

	while (type == htons(ETH_P_8021Q)) {
		struct vlan_hdr *vh;

		if (unlikely(!pskb_may_pull(skb, vlan_depth + VLAN_HLEN)))
			return ERR_PTR(-EINVAL);

		vh = (struct vlan_hdr *)(skb->data + vlan_depth);
		type = vh->h_vlan_encapsulated_proto;
		vlan_depth += VLAN_HLEN;
	}

	skb_reset_mac_header(skb);
	skb->mac_len = skb->network_header - skb->mac_header;
	__skb_pull(skb, skb->mac_len);

	if (unlikely(skb->ip_summed != CHECKSUM_PARTIAL)) {
		skb_warn_bad_offload(skb);

		if (skb_header_cloned(skb) &&
		    (err = pskb_expand_head(skb, 0, 0, GFP_ATOMIC)))
			return ERR_PTR(err);
	}

	rcu_read_lock();
	list_for_each_entry_rcu(ptype,
			&ptype_base[ntohs(type) & PTYPE_HASH_MASK], list) {
		if (ptype->type == type && !ptype->dev && ptype->gso_segment) {
			if (unlikely(skb->ip_summed != CHECKSUM_PARTIAL)) {
				err = ptype->gso_send_check(skb);
				segs = ERR_PTR(err);
				if (err || skb_gso_ok(skb, features))
					break;
				__skb_push(skb, (skb->data -
						 skb_network_header(skb)));
			}
			segs = ptype->gso_segment(skb, features);
			break;
		}
	}
	rcu_read_unlock();

	__skb_push(skb, skb->data - skb_mac_header(skb));

	return segs;
}
EXPORT_SYMBOL(skb_gso_segment);

#ifdef CONFIG_BUG
void netdev_rx_csum_fault(struct net_device *dev)
{
	if (net_ratelimit()) {
		pr_err("%s: hw csum failure\n", dev ? dev->name : "<unknown>");
		dump_stack();
	}
}
EXPORT_SYMBOL(netdev_rx_csum_fault);
#endif


static int illegal_highdma(struct net_device *dev, struct sk_buff *skb)
{
#ifdef CONFIG_HIGHMEM
	int i;
	if (!(dev->features & NETIF_F_HIGHDMA)) {
		for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
			skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
			if (PageHighMem(skb_frag_page(frag)))
				return 1;
		}
	}

	if (PCI_DMA_BUS_IS_PHYS) {
		struct device *pdev = dev->dev.parent;

		if (!pdev)
			return 0;
		for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
			skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
			dma_addr_t addr = page_to_phys(skb_frag_page(frag));
			if (!pdev->dma_mask || addr + PAGE_SIZE - 1 > *pdev->dma_mask)
				return 1;
		}
	}
#endif
	return 0;
}

struct dev_gso_cb {
	void (*destructor)(struct sk_buff *skb);
};

#define DEV_GSO_CB(skb) ((struct dev_gso_cb *)(skb)->cb)

static void dev_gso_skb_destructor(struct sk_buff *skb)
{
	struct dev_gso_cb *cb;

	do {
		struct sk_buff *nskb = skb->next;

		skb->next = nskb->next;
		nskb->next = NULL;
		kfree_skb(nskb);
	} while (skb->next);

	cb = DEV_GSO_CB(skb);
	if (cb->destructor)
		cb->destructor(skb);
}

static int dev_gso_segment(struct sk_buff *skb, netdev_features_t features)
{
	struct sk_buff *segs;

	segs = skb_gso_segment(skb, features);

	
	if (!segs)
		return 0;

	if (IS_ERR(segs))
		return PTR_ERR(segs);

	skb->next = segs;
	DEV_GSO_CB(skb)->destructor = skb->destructor;
	skb->destructor = dev_gso_skb_destructor;

	return 0;
}

static inline void skb_orphan_try(struct sk_buff *skb)
{
	struct sock *sk = skb->sk;

	if (sk && !skb_shinfo(skb)->tx_flags) {
		if (!skb->rxhash)
			skb->rxhash = sk->sk_hash;
		skb_orphan(skb);
	}
}

static bool can_checksum_protocol(netdev_features_t features, __be16 protocol)
{
	return ((features & NETIF_F_GEN_CSUM) ||
		((features & NETIF_F_V4_CSUM) &&
		 protocol == htons(ETH_P_IP)) ||
		((features & NETIF_F_V6_CSUM) &&
		 protocol == htons(ETH_P_IPV6)) ||
		((features & NETIF_F_FCOE_CRC) &&
		 protocol == htons(ETH_P_FCOE)));
}

static netdev_features_t harmonize_features(struct sk_buff *skb,
	__be16 protocol, netdev_features_t features)
{
	if (!can_checksum_protocol(features, protocol)) {
		features &= ~NETIF_F_ALL_CSUM;
		features &= ~NETIF_F_SG;
	} else if (illegal_highdma(skb->dev, skb)) {
		features &= ~NETIF_F_SG;
	}

	return features;
}

netdev_features_t netif_skb_features(struct sk_buff *skb)
{
#ifdef CONFIG_HTC_NETWORK_MODIFY
	__be16 protocol;
	netdev_features_t features;

	if (IS_ERR(skb) || (!skb)) {
		printk(KERN_ERR "[CORE] skb is NULL in %s!\n", __func__);
		return 0;
	}

    protocol = skb->protocol;
    features  = skb->dev->features;
#else
	__be16 protocol = skb->protocol;
	netdev_features_t features = skb->dev->features;
#endif

	if (protocol == htons(ETH_P_8021Q)) {
		struct vlan_ethhdr *veh = (struct vlan_ethhdr *)skb->data;
		protocol = veh->h_vlan_encapsulated_proto;
	} else if (!vlan_tx_tag_present(skb)) {
		return harmonize_features(skb, protocol, features);
	}

	features &= (skb->dev->vlan_features | NETIF_F_HW_VLAN_TX);

	if (protocol != htons(ETH_P_8021Q)) {
		return harmonize_features(skb, protocol, features);
	} else {
		features &= NETIF_F_SG | NETIF_F_HIGHDMA | NETIF_F_FRAGLIST |
				NETIF_F_GEN_CSUM | NETIF_F_HW_VLAN_TX;
		return harmonize_features(skb, protocol, features);
	}
}
EXPORT_SYMBOL(netif_skb_features);

static inline int skb_needs_linearize(struct sk_buff *skb,
				      int features)
{
	return skb_is_nonlinear(skb) &&
			((skb_has_frag_list(skb) &&
				!(features & NETIF_F_FRAGLIST)) ||
			(skb_shinfo(skb)->nr_frags &&
				!(features & NETIF_F_SG)));
}

int dev_hard_start_xmit(struct sk_buff *skb, struct net_device *dev,
			struct netdev_queue *txq)
{
	const struct net_device_ops *ops = dev->netdev_ops;
	int rc = NETDEV_TX_OK;
	unsigned int skb_len;

	if (likely(!skb->next)) {
		netdev_features_t features;

		if (dev->priv_flags & IFF_XMIT_DST_RELEASE)
			skb_dst_drop(skb);

		if (!list_empty(&ptype_all))
			dev_queue_xmit_nit(skb, dev);

		skb_orphan_try(skb);

		features = netif_skb_features(skb);

		if (vlan_tx_tag_present(skb) &&
		    !(features & NETIF_F_HW_VLAN_TX)) {
			skb = __vlan_put_tag(skb, vlan_tx_tag_get(skb));
			if (unlikely(!skb))
				goto out;

#ifdef CONFIG_HTC_NETWORK_MODIFY
	if (IS_ERR(skb) || (!skb)) {
		printk(KERN_ERR "[CORE] skb is NULL in %s!\n", __func__);
		goto out;
	}
#endif

			skb->vlan_tci = 0;
		}

		if (netif_needs_gso(skb, features)) {
			if (unlikely(dev_gso_segment(skb, features)))
				goto out_kfree_skb;
			if (skb->next)
				goto gso;
		} else {
			if (skb_needs_linearize(skb, features) &&
			    __skb_linearize(skb))
				goto out_kfree_skb;

			if (skb->ip_summed == CHECKSUM_PARTIAL) {
				skb_set_transport_header(skb,
					skb_checksum_start_offset(skb));
				if (!(features & NETIF_F_ALL_CSUM) &&
				     skb_checksum_help(skb))
					goto out_kfree_skb;
			}
		}

		skb_len = skb->len;
		rc = ops->ndo_start_xmit(skb, dev);
		trace_net_dev_xmit(skb, rc, dev, skb_len);
		if (rc == NETDEV_TX_OK)
			txq_trans_update(txq);
		return rc;
	}

gso:
	do {
		struct sk_buff *nskb = skb->next;

		skb->next = nskb->next;
		nskb->next = NULL;

		if (dev->priv_flags & IFF_XMIT_DST_RELEASE)
			skb_dst_drop(nskb);

		skb_len = nskb->len;
		rc = ops->ndo_start_xmit(nskb, dev);
		trace_net_dev_xmit(nskb, rc, dev, skb_len);
		if (unlikely(rc != NETDEV_TX_OK)) {
			if (rc & ~NETDEV_TX_MASK)
				goto out_kfree_gso_skb;
			nskb->next = skb->next;
			skb->next = nskb;
			return rc;
		}
		txq_trans_update(txq);
		if (unlikely(netif_xmit_stopped(txq) && skb->next))
			return NETDEV_TX_BUSY;
	} while (skb->next);

out_kfree_gso_skb:
	if (likely(skb->next == NULL))
		skb->destructor = DEV_GSO_CB(skb)->destructor;
out_kfree_skb:
	kfree_skb(skb);
out:
	return rc;
}

static u32 hashrnd __read_mostly;

u16 __skb_tx_hash(const struct net_device *dev, const struct sk_buff *skb,
		  unsigned int num_tx_queues)
{
	u32 hash;
	u16 qoffset = 0;
	u16 qcount = num_tx_queues;

	if (skb_rx_queue_recorded(skb)) {
		hash = skb_get_rx_queue(skb);
		while (unlikely(hash >= num_tx_queues))
			hash -= num_tx_queues;
		return hash;
	}

	if (dev->num_tc) {
		u8 tc = netdev_get_prio_tc_map(dev, skb->priority);
		qoffset = dev->tc_to_txq[tc].offset;
		qcount = dev->tc_to_txq[tc].count;
	}

	if (skb->sk && skb->sk->sk_hash)
		hash = skb->sk->sk_hash;
	else
		hash = (__force u16) skb->protocol ^ skb->rxhash;
	hash = jhash_1word(hash, hashrnd);

	return (u16) (((u64) hash * qcount) >> 32) + qoffset;
}
EXPORT_SYMBOL(__skb_tx_hash);

static inline u16 dev_cap_txqueue(struct net_device *dev, u16 queue_index)
{
	if (unlikely(queue_index >= dev->real_num_tx_queues)) {
		if (net_ratelimit()) {
			pr_warn("%s selects TX queue %d, but real number of TX queues is %d\n",
				dev->name, queue_index,
				dev->real_num_tx_queues);
		}
		return 0;
	}
	return queue_index;
}

static inline int get_xps_queue(struct net_device *dev, struct sk_buff *skb)
{
#ifdef CONFIG_XPS
	struct xps_dev_maps *dev_maps;
	struct xps_map *map;
	int queue_index = -1;

	rcu_read_lock();
	dev_maps = rcu_dereference(dev->xps_maps);
	if (dev_maps) {
		map = rcu_dereference(
		    dev_maps->cpu_map[raw_smp_processor_id()]);
		if (map) {
			if (map->len == 1)
				queue_index = map->queues[0];
			else {
				u32 hash;
				if (skb->sk && skb->sk->sk_hash)
					hash = skb->sk->sk_hash;
				else
					hash = (__force u16) skb->protocol ^
					    skb->rxhash;
				hash = jhash_1word(hash, hashrnd);
				queue_index = map->queues[
				    ((u64)hash * map->len) >> 32];
			}
			if (unlikely(queue_index >= dev->real_num_tx_queues))
				queue_index = -1;
		}
	}
	rcu_read_unlock();

	return queue_index;
#else
	return -1;
#endif
}

static struct netdev_queue *dev_pick_tx(struct net_device *dev,
					struct sk_buff *skb)
{
	int queue_index;
	const struct net_device_ops *ops = dev->netdev_ops;

	if (dev->real_num_tx_queues == 1)
		queue_index = 0;
	else if (ops->ndo_select_queue) {
		queue_index = ops->ndo_select_queue(dev, skb);
		queue_index = dev_cap_txqueue(dev, queue_index);
	} else {
		struct sock *sk = skb->sk;
		queue_index = sk_tx_queue_get(sk);

		if (queue_index < 0 || skb->ooo_okay ||
		    queue_index >= dev->real_num_tx_queues) {
			int old_index = queue_index;

			queue_index = get_xps_queue(dev, skb);
			if (queue_index < 0)
				queue_index = skb_tx_hash(dev, skb);

			if (queue_index != old_index && sk) {
				struct dst_entry *dst =
				    rcu_dereference_check(sk->sk_dst_cache, 1);

				if (dst && skb_dst(skb) == dst)
					sk_tx_queue_set(sk, queue_index);
			}
		}
	}

	skb_set_queue_mapping(skb, queue_index);
	return netdev_get_tx_queue(dev, queue_index);
}

static inline int __dev_xmit_skb(struct sk_buff *skb, struct Qdisc *q,
				 struct net_device *dev,
				 struct netdev_queue *txq)
{
	spinlock_t *root_lock = qdisc_lock(q);
	bool contended;
	int rc;

	qdisc_skb_cb(skb)->pkt_len = skb->len;
	qdisc_calculate_pkt_len(skb, q);
	contended = qdisc_is_running(q);
	if (unlikely(contended))
		spin_lock(&q->busylock);

	spin_lock(root_lock);
	if (unlikely(test_bit(__QDISC_STATE_DEACTIVATED, &q->state))) {
		kfree_skb(skb);
		rc = NET_XMIT_DROP;
	} else if ((q->flags & TCQ_F_CAN_BYPASS) && !qdisc_qlen(q) &&
		   qdisc_run_begin(q)) {
		if (!(dev->priv_flags & IFF_XMIT_DST_RELEASE))
			skb_dst_force(skb);

		qdisc_bstats_update(q, skb);

		if (sch_direct_xmit(skb, q, dev, txq, root_lock)) {
			if (unlikely(contended)) {
				spin_unlock(&q->busylock);
				contended = false;
			}
			__qdisc_run(q);
		} else
			qdisc_run_end(q);

		rc = NET_XMIT_SUCCESS;
	} else {
		skb_dst_force(skb);
		rc = q->enqueue(skb, q) & NET_XMIT_MASK;
		if (qdisc_run_begin(q)) {
			if (unlikely(contended)) {
				spin_unlock(&q->busylock);
				contended = false;
			}
			__qdisc_run(q);
		}
	}
	spin_unlock(root_lock);
	if (unlikely(contended))
		spin_unlock(&q->busylock);
	return rc;
}

#if IS_ENABLED(CONFIG_NETPRIO_CGROUP)
static void skb_update_prio(struct sk_buff *skb)
{
	struct netprio_map *map = rcu_dereference_bh(skb->dev->priomap);

	if ((!skb->priority) && (skb->sk) && map)
		skb->priority = map->priomap[skb->sk->sk_cgrp_prioidx];
}
#else
#define skb_update_prio(skb)
#endif

static DEFINE_PER_CPU(int, xmit_recursion);
#define RECURSION_LIMIT 10

int dev_queue_xmit(struct sk_buff *skb)
{
	struct net_device *dev = skb->dev;
	struct netdev_queue *txq;
	struct Qdisc *q;
	int rc = -ENOMEM;

	rcu_read_lock_bh();

	skb_update_prio(skb);

	txq = dev_pick_tx(dev, skb);
	q = rcu_dereference_bh(txq->qdisc);

#ifdef CONFIG_NET_CLS_ACT
	skb->tc_verd = SET_TC_AT(skb->tc_verd, AT_EGRESS);
#endif
	trace_net_dev_queue(skb);
	if (q->enqueue) {
		rc = __dev_xmit_skb(skb, q, dev, txq);
		goto out;
	}

	if (dev->flags & IFF_UP) {
		int cpu = smp_processor_id(); 

		if (txq->xmit_lock_owner != cpu) {

			if (__this_cpu_read(xmit_recursion) > RECURSION_LIMIT)
				goto recursion_alert;

			HARD_TX_LOCK(dev, txq, cpu);

			if (!netif_xmit_stopped(txq)) {
				__this_cpu_inc(xmit_recursion);
				rc = dev_hard_start_xmit(skb, dev, txq);
				__this_cpu_dec(xmit_recursion);
				if (dev_xmit_complete(rc)) {
					HARD_TX_UNLOCK(dev, txq);
					goto out;
				}
			}
			HARD_TX_UNLOCK(dev, txq);
			if (net_ratelimit())
				pr_crit("Virtual device %s asks to queue packet!\n",
					dev->name);
		} else {
recursion_alert:
			if (net_ratelimit())
				pr_crit("Dead loop on virtual device %s, fix it urgently!\n",
					dev->name);
		}
	}

	rc = -ENETDOWN;
	rcu_read_unlock_bh();

#ifdef CONFIG_HTC_NETWORK_MODIFY
	if (!IS_ERR(skb) && (skb))
		kfree_skb(skb);
#else
	kfree_skb(skb);
#endif
	return rc;
out:
	rcu_read_unlock_bh();
	return rc;
}
EXPORT_SYMBOL(dev_queue_xmit);



int netdev_max_backlog __read_mostly = 1000;
int netdev_tstamp_prequeue __read_mostly = 1;
int netdev_budget __read_mostly = 300;
int weight_p __read_mostly = 64;            

static inline void ____napi_schedule(struct softnet_data *sd,
				     struct napi_struct *napi)
{
	list_add_tail(&napi->poll_list, &sd->poll_list);
	__raise_softirq_irqoff(NET_RX_SOFTIRQ);
}

void __skb_get_rxhash(struct sk_buff *skb)
{
	struct flow_keys keys;
	u32 hash;

	if (!skb_flow_dissect(skb, &keys))
		return;

	if (keys.ports) {
		if ((__force u16)keys.port16[1] < (__force u16)keys.port16[0])
			swap(keys.port16[0], keys.port16[1]);
		skb->l4_rxhash = 1;
	}

	
	if ((__force u32)keys.dst < (__force u32)keys.src)
		swap(keys.dst, keys.src);

	hash = jhash_3words((__force u32)keys.dst,
			    (__force u32)keys.src,
			    (__force u32)keys.ports, hashrnd);
	if (!hash)
		hash = 1;

	skb->rxhash = hash;
}
EXPORT_SYMBOL(__skb_get_rxhash);

#ifdef CONFIG_RPS

struct rps_sock_flow_table __rcu *rps_sock_flow_table __read_mostly;
EXPORT_SYMBOL(rps_sock_flow_table);

struct static_key rps_needed __read_mostly;

static struct rps_dev_flow *
set_rps_cpu(struct net_device *dev, struct sk_buff *skb,
	    struct rps_dev_flow *rflow, u16 next_cpu)
{
	if (next_cpu != RPS_NO_CPU) {
#ifdef CONFIG_RFS_ACCEL
		struct netdev_rx_queue *rxqueue;
		struct rps_dev_flow_table *flow_table;
		struct rps_dev_flow *old_rflow;
		u32 flow_id;
		u16 rxq_index;
		int rc;

		
		if (!skb_rx_queue_recorded(skb) || !dev->rx_cpu_rmap ||
		    !(dev->features & NETIF_F_NTUPLE))
			goto out;
		rxq_index = cpu_rmap_lookup_index(dev->rx_cpu_rmap, next_cpu);
		if (rxq_index == skb_get_rx_queue(skb))
			goto out;

		rxqueue = dev->_rx + rxq_index;
		flow_table = rcu_dereference(rxqueue->rps_flow_table);
		if (!flow_table)
			goto out;
		flow_id = skb->rxhash & flow_table->mask;
		rc = dev->netdev_ops->ndo_rx_flow_steer(dev, skb,
							rxq_index, flow_id);
		if (rc < 0)
			goto out;
		old_rflow = rflow;
		rflow = &flow_table->flows[flow_id];
		rflow->filter = rc;
		if (old_rflow->filter == rflow->filter)
			old_rflow->filter = RPS_NO_FILTER;
	out:
#endif
		rflow->last_qtail =
			per_cpu(softnet_data, next_cpu).input_queue_head;
	}

	rflow->cpu = next_cpu;
	return rflow;
}

static int get_rps_cpu(struct net_device *dev, struct sk_buff *skb,
		       struct rps_dev_flow **rflowp)
{
	struct netdev_rx_queue *rxqueue;
	struct rps_map *map;
	struct rps_dev_flow_table *flow_table;
	struct rps_sock_flow_table *sock_flow_table;
	int cpu = -1;
	u16 tcpu;

	if (skb_rx_queue_recorded(skb)) {
		u16 index = skb_get_rx_queue(skb);
		if (unlikely(index >= dev->real_num_rx_queues)) {
			WARN_ONCE(dev->real_num_rx_queues > 1,
				  "%s received packet on queue %u, but number "
				  "of RX queues is %u\n",
				  dev->name, index, dev->real_num_rx_queues);
			goto done;
		}
		rxqueue = dev->_rx + index;
	} else
		rxqueue = dev->_rx;

	map = rcu_dereference(rxqueue->rps_map);
	if (map) {
		if (map->len == 1 &&
		    !rcu_access_pointer(rxqueue->rps_flow_table)) {
			tcpu = map->cpus[0];
			if (cpu_online(tcpu))
				cpu = tcpu;
			goto done;
		}
	} else if (!rcu_access_pointer(rxqueue->rps_flow_table)) {
		goto done;
	}

	skb_reset_network_header(skb);
	if (!skb_get_rxhash(skb))
		goto done;

	flow_table = rcu_dereference(rxqueue->rps_flow_table);
	sock_flow_table = rcu_dereference(rps_sock_flow_table);
	if (flow_table && sock_flow_table) {
		u16 next_cpu;
		struct rps_dev_flow *rflow;

		rflow = &flow_table->flows[skb->rxhash & flow_table->mask];
		tcpu = rflow->cpu;

		next_cpu = sock_flow_table->ents[skb->rxhash &
		    sock_flow_table->mask];

		if (unlikely(tcpu != next_cpu) &&
		    (tcpu == RPS_NO_CPU || !cpu_online(tcpu) ||
		     ((int)(per_cpu(softnet_data, tcpu).input_queue_head -
		      rflow->last_qtail)) >= 0))
			rflow = set_rps_cpu(dev, skb, rflow, next_cpu);

		if (tcpu != RPS_NO_CPU && cpu_online(tcpu)) {
			*rflowp = rflow;
			cpu = tcpu;
			goto done;
		}
	}

	if (map) {
		tcpu = map->cpus[((u64) skb->rxhash * map->len) >> 32];

		if (cpu_online(tcpu)) {
			cpu = tcpu;
			goto done;
		}
	}

done:
	return cpu;
}

#ifdef CONFIG_RFS_ACCEL

bool rps_may_expire_flow(struct net_device *dev, u16 rxq_index,
			 u32 flow_id, u16 filter_id)
{
	struct netdev_rx_queue *rxqueue = dev->_rx + rxq_index;
	struct rps_dev_flow_table *flow_table;
	struct rps_dev_flow *rflow;
	bool expire = true;
	int cpu;

	rcu_read_lock();
	flow_table = rcu_dereference(rxqueue->rps_flow_table);
	if (flow_table && flow_id <= flow_table->mask) {
		rflow = &flow_table->flows[flow_id];
		cpu = ACCESS_ONCE(rflow->cpu);
		if (rflow->filter == filter_id && cpu != RPS_NO_CPU &&
		    ((int)(per_cpu(softnet_data, cpu).input_queue_head -
			   rflow->last_qtail) <
		     (int)(10 * flow_table->mask)))
			expire = false;
	}
	rcu_read_unlock();
	return expire;
}
EXPORT_SYMBOL(rps_may_expire_flow);

#endif 

static void rps_trigger_softirq(void *data)
{
	struct softnet_data *sd = data;

	____napi_schedule(sd, &sd->backlog);
	sd->received_rps++;
}

#endif 

static int rps_ipi_queued(struct softnet_data *sd)
{
#ifdef CONFIG_RPS
	struct softnet_data *mysd = &__get_cpu_var(softnet_data);

	if (sd != mysd) {
		sd->rps_ipi_next = mysd->rps_ipi_list;
		mysd->rps_ipi_list = sd;

		__raise_softirq_irqoff(NET_RX_SOFTIRQ);
		return 1;
	}
#endif 
	return 0;
}

static int enqueue_to_backlog(struct sk_buff *skb, int cpu,
			      unsigned int *qtail)
{
	struct softnet_data *sd;
	unsigned long flags;

	sd = &per_cpu(softnet_data, cpu);

	local_irq_save(flags);

	rps_lock(sd);
	if (skb_queue_len(&sd->input_pkt_queue) <= netdev_max_backlog) {
		if (skb_queue_len(&sd->input_pkt_queue)) {
enqueue:
			__skb_queue_tail(&sd->input_pkt_queue, skb);
			input_queue_tail_incr_save(sd, qtail);
			rps_unlock(sd);
			local_irq_restore(flags);
			return NET_RX_SUCCESS;
		}

		if (!__test_and_set_bit(NAPI_STATE_SCHED, &sd->backlog.state)) {
			if (!rps_ipi_queued(sd))
				____napi_schedule(sd, &sd->backlog);
		}
		goto enqueue;
	}

	sd->dropped++;
	rps_unlock(sd);

	local_irq_restore(flags);

	atomic_long_inc(&skb->dev->rx_dropped);
	kfree_skb(skb);
	return NET_RX_DROP;
}


int netif_rx(struct sk_buff *skb)
{
	int ret;

	
	if (netpoll_rx(skb))
		return NET_RX_DROP;

	net_timestamp_check(netdev_tstamp_prequeue, skb);

	trace_netif_rx(skb);
#ifdef CONFIG_RPS
	if (static_key_false(&rps_needed)) {
		struct rps_dev_flow voidflow, *rflow = &voidflow;
		int cpu;

		preempt_disable();
		rcu_read_lock();

		cpu = get_rps_cpu(skb->dev, skb, &rflow);
		if (cpu < 0)
			cpu = smp_processor_id();

		ret = enqueue_to_backlog(skb, cpu, &rflow->last_qtail);

		rcu_read_unlock();
		preempt_enable();
	} else
#endif
	{
		unsigned int qtail;
		ret = enqueue_to_backlog(skb, get_cpu(), &qtail);
		put_cpu();
	}
	return ret;
}
EXPORT_SYMBOL(netif_rx);

int netif_rx_ni(struct sk_buff *skb)
{
	int err;

	preempt_disable();
	err = netif_rx(skb);
	if (local_softirq_pending())
		do_softirq();
	preempt_enable();

	return err;
}
EXPORT_SYMBOL(netif_rx_ni);

static void net_tx_action(struct softirq_action *h)
{
	struct softnet_data *sd = &__get_cpu_var(softnet_data);

	if (sd->completion_queue) {
		struct sk_buff *clist;

		local_irq_disable();
		clist = sd->completion_queue;
		sd->completion_queue = NULL;
		local_irq_enable();

		while (clist) {
			struct sk_buff *skb = clist;
			clist = clist->next;

			WARN_ON(atomic_read(&skb->users));
			trace_kfree_skb(skb, net_tx_action);
			__kfree_skb(skb);
		}
	}

	if (sd->output_queue) {
		struct Qdisc *head;

		local_irq_disable();
		head = sd->output_queue;
		sd->output_queue = NULL;
		sd->output_queue_tailp = &sd->output_queue;
		local_irq_enable();

		while (head) {
			struct Qdisc *q = head;
			spinlock_t *root_lock;

			head = head->next_sched;

			root_lock = qdisc_lock(q);
			if (spin_trylock(root_lock)) {
				smp_mb__before_clear_bit();
				clear_bit(__QDISC_STATE_SCHED,
					  &q->state);
				qdisc_run(q);
				spin_unlock(root_lock);
			} else {
				if (!test_bit(__QDISC_STATE_DEACTIVATED,
					      &q->state)) {
					__netif_reschedule(q);
				} else {
					smp_mb__before_clear_bit();
					clear_bit(__QDISC_STATE_SCHED,
						  &q->state);
				}
			}
		}
	}
}

#if (defined(CONFIG_BRIDGE) || defined(CONFIG_BRIDGE_MODULE)) && \
    (defined(CONFIG_ATM_LANE) || defined(CONFIG_ATM_LANE_MODULE))
int (*br_fdb_test_addr_hook)(struct net_device *dev,
			     unsigned char *addr) __read_mostly;
EXPORT_SYMBOL_GPL(br_fdb_test_addr_hook);
#endif

#ifdef CONFIG_NET_CLS_ACT
static int ing_filter(struct sk_buff *skb, struct netdev_queue *rxq)
{
	struct net_device *dev = skb->dev;
	u32 ttl = G_TC_RTTL(skb->tc_verd);
	int result = TC_ACT_OK;
	struct Qdisc *q;

	if (unlikely(MAX_RED_LOOP < ttl++)) {
		if (net_ratelimit())
			pr_warn("Redir loop detected Dropping packet (%d->%d)\n",
				skb->skb_iif, dev->ifindex);
		return TC_ACT_SHOT;
	}

	skb->tc_verd = SET_TC_RTTL(skb->tc_verd, ttl);
	skb->tc_verd = SET_TC_AT(skb->tc_verd, AT_INGRESS);

	q = rxq->qdisc;
	if (q != &noop_qdisc) {
		spin_lock(qdisc_lock(q));
		if (likely(!test_bit(__QDISC_STATE_DEACTIVATED, &q->state)))
			result = qdisc_enqueue_root(skb, q);
		spin_unlock(qdisc_lock(q));
	}

	return result;
}

static inline struct sk_buff *handle_ing(struct sk_buff *skb,
					 struct packet_type **pt_prev,
					 int *ret, struct net_device *orig_dev)
{
	struct netdev_queue *rxq = rcu_dereference(skb->dev->ingress_queue);

	if (!rxq || rxq->qdisc == &noop_qdisc)
		goto out;

	if (*pt_prev) {
		*ret = deliver_skb(skb, *pt_prev, orig_dev);
		*pt_prev = NULL;
	}

	switch (ing_filter(skb, rxq)) {
	case TC_ACT_SHOT:
	case TC_ACT_STOLEN:
		kfree_skb(skb);
		return NULL;
	}

out:
	skb->tc_verd = 0;
	return skb;
}
#endif

int netdev_rx_handler_register(struct net_device *dev,
			       rx_handler_func_t *rx_handler,
			       void *rx_handler_data)
{
	ASSERT_RTNL();

	if (dev->rx_handler)
		return -EBUSY;

	rcu_assign_pointer(dev->rx_handler_data, rx_handler_data);
	rcu_assign_pointer(dev->rx_handler, rx_handler);

	return 0;
}
EXPORT_SYMBOL_GPL(netdev_rx_handler_register);

void netdev_rx_handler_unregister(struct net_device *dev)
{

	ASSERT_RTNL();
	RCU_INIT_POINTER(dev->rx_handler, NULL);
	RCU_INIT_POINTER(dev->rx_handler_data, NULL);
}
EXPORT_SYMBOL_GPL(netdev_rx_handler_unregister);

static int __netif_receive_skb(struct sk_buff *skb)
{
	struct packet_type *ptype, *pt_prev;
	rx_handler_func_t *rx_handler;
	struct net_device *orig_dev;
	struct net_device *null_or_dev;
	bool deliver_exact = false;
	int ret = NET_RX_DROP;
	__be16 type;

	net_timestamp_check(!netdev_tstamp_prequeue, skb);

	trace_netif_receive_skb(skb);

	
	if (netpoll_receive_skb(skb))
		return NET_RX_DROP;

	if (!skb->skb_iif)
		skb->skb_iif = skb->dev->ifindex;
	orig_dev = skb->dev;

	skb_reset_network_header(skb);
	skb_reset_transport_header(skb);
	skb_reset_mac_len(skb);

	pt_prev = NULL;

	rcu_read_lock();

another_round:

	__this_cpu_inc(softnet_data.processed);

	if (skb->protocol == cpu_to_be16(ETH_P_8021Q)) {
		skb = vlan_untag(skb);
		if (unlikely(!skb))
			goto out;
	}

#ifdef CONFIG_NET_CLS_ACT
	if (skb->tc_verd & TC_NCLS) {
		skb->tc_verd = CLR_TC_NCLS(skb->tc_verd);
		goto ncls;
	}
#endif

	list_for_each_entry_rcu(ptype, &ptype_all, list) {
		if (!ptype->dev || ptype->dev == skb->dev) {
			if (pt_prev)
				ret = deliver_skb(skb, pt_prev, orig_dev);
			pt_prev = ptype;
		}
	}

#ifdef CONFIG_NET_CLS_ACT
	skb = handle_ing(skb, &pt_prev, &ret, orig_dev);
	if (!skb)
		goto out;
ncls:
#endif

	rx_handler = rcu_dereference(skb->dev->rx_handler);
	if (vlan_tx_tag_present(skb)) {
		if (pt_prev) {
			ret = deliver_skb(skb, pt_prev, orig_dev);
			pt_prev = NULL;
		}
		if (vlan_do_receive(&skb, !rx_handler))
			goto another_round;
		else if (unlikely(!skb))
			goto out;
	}

	if (rx_handler) {
		if (pt_prev) {
			ret = deliver_skb(skb, pt_prev, orig_dev);
			pt_prev = NULL;
		}
		switch (rx_handler(&skb)) {
		case RX_HANDLER_CONSUMED:
			goto out;
		case RX_HANDLER_ANOTHER:
			goto another_round;
		case RX_HANDLER_EXACT:
			deliver_exact = true;
		case RX_HANDLER_PASS:
			break;
		default:
			BUG();
		}
	}

	
	null_or_dev = deliver_exact ? skb->dev : NULL;

	type = skb->protocol;
	list_for_each_entry_rcu(ptype,
			&ptype_base[ntohs(type) & PTYPE_HASH_MASK], list) {
		if (ptype->type == type &&
		    (ptype->dev == null_or_dev || ptype->dev == skb->dev ||
		     ptype->dev == orig_dev)) {
			if (pt_prev)
				ret = deliver_skb(skb, pt_prev, orig_dev);
			pt_prev = ptype;
		}
	}

	if (pt_prev) {
		ret = pt_prev->func(skb, skb->dev, pt_prev, orig_dev);
	} else {
		atomic_long_inc(&skb->dev->rx_dropped);
		kfree_skb(skb);
		ret = NET_RX_DROP;
	}

out:
	rcu_read_unlock();
	return ret;
}

int netif_receive_skb(struct sk_buff *skb)
{
	net_timestamp_check(netdev_tstamp_prequeue, skb);

	if (skb_defer_rx_timestamp(skb))
		return NET_RX_SUCCESS;

#ifdef CONFIG_RPS
	if (static_key_false(&rps_needed)) {
		struct rps_dev_flow voidflow, *rflow = &voidflow;
		int cpu, ret;

		rcu_read_lock();

		cpu = get_rps_cpu(skb->dev, skb, &rflow);

		if (cpu >= 0) {
			ret = enqueue_to_backlog(skb, cpu, &rflow->last_qtail);
			rcu_read_unlock();
			return ret;
		}
		rcu_read_unlock();
	}
#endif
	return __netif_receive_skb(skb);
}
EXPORT_SYMBOL(netif_receive_skb);

static void flush_backlog(void *arg)
{
	struct net_device *dev = arg;
	struct softnet_data *sd = &__get_cpu_var(softnet_data);
	struct sk_buff *skb, *tmp;

	rps_lock(sd);
	skb_queue_walk_safe(&sd->input_pkt_queue, skb, tmp) {
		if (skb->dev == dev) {
			__skb_unlink(skb, &sd->input_pkt_queue);
			kfree_skb(skb);
			input_queue_head_incr(sd);
		}
	}
	rps_unlock(sd);

	skb_queue_walk_safe(&sd->process_queue, skb, tmp) {
		if (skb->dev == dev) {
			__skb_unlink(skb, &sd->process_queue);
			kfree_skb(skb);
			input_queue_head_incr(sd);
		}
	}
}

static int napi_gro_complete(struct sk_buff *skb)
{
	struct packet_type *ptype;
	__be16 type = skb->protocol;
	struct list_head *head = &ptype_base[ntohs(type) & PTYPE_HASH_MASK];
	int err = -ENOENT;

	if (NAPI_GRO_CB(skb)->count == 1) {
		skb_shinfo(skb)->gso_size = 0;
		goto out;
	}

	rcu_read_lock();
	list_for_each_entry_rcu(ptype, head, list) {
		if (ptype->type != type || ptype->dev || !ptype->gro_complete)
			continue;

		err = ptype->gro_complete(skb);
		break;
	}
	rcu_read_unlock();

	if (err) {
		WARN_ON(&ptype->list == head);
		kfree_skb(skb);
		return NET_RX_SUCCESS;
	}

out:
	return netif_receive_skb(skb);
}

inline void napi_gro_flush(struct napi_struct *napi)
{
	struct sk_buff *skb, *next;

	for (skb = napi->gro_list; skb; skb = next) {
		next = skb->next;
		skb->next = NULL;
		napi_gro_complete(skb);
	}

	napi->gro_count = 0;
	napi->gro_list = NULL;
}
EXPORT_SYMBOL(napi_gro_flush);

enum gro_result dev_gro_receive(struct napi_struct *napi, struct sk_buff *skb)
{
	struct sk_buff **pp = NULL;
	struct packet_type *ptype;
	__be16 type = skb->protocol;
	struct list_head *head = &ptype_base[ntohs(type) & PTYPE_HASH_MASK];
	int same_flow;
	int mac_len;
	enum gro_result ret;

	if (!(skb->dev->features & NETIF_F_GRO) || netpoll_rx_on(skb))
		goto normal;

	if (skb_is_gso(skb) || skb_has_frag_list(skb))
		goto normal;

	rcu_read_lock();
	list_for_each_entry_rcu(ptype, head, list) {
		if (ptype->type != type || ptype->dev || !ptype->gro_receive)
			continue;

		skb_set_network_header(skb, skb_gro_offset(skb));
		mac_len = skb->network_header - skb->mac_header;
		skb->mac_len = mac_len;
		NAPI_GRO_CB(skb)->same_flow = 0;
		NAPI_GRO_CB(skb)->flush = 0;
		NAPI_GRO_CB(skb)->free = 0;

		pp = ptype->gro_receive(&napi->gro_list, skb);
		break;
	}
	rcu_read_unlock();

	if (&ptype->list == head)
		goto normal;

	same_flow = NAPI_GRO_CB(skb)->same_flow;
	ret = NAPI_GRO_CB(skb)->free ? GRO_MERGED_FREE : GRO_MERGED;

	if (pp) {
		struct sk_buff *nskb = *pp;

		*pp = nskb->next;
		nskb->next = NULL;
		napi_gro_complete(nskb);
		napi->gro_count--;
	}

	if (same_flow)
		goto ok;

	if (NAPI_GRO_CB(skb)->flush || napi->gro_count >= MAX_GRO_SKBS)
		goto normal;

	napi->gro_count++;
	NAPI_GRO_CB(skb)->count = 1;
	skb_shinfo(skb)->gso_size = skb_gro_len(skb);
	skb->next = napi->gro_list;
	napi->gro_list = skb;
	ret = GRO_HELD;

pull:
	if (skb_headlen(skb) < skb_gro_offset(skb)) {
		int grow = skb_gro_offset(skb) - skb_headlen(skb);

		BUG_ON(skb->end - skb->tail < grow);

		memcpy(skb_tail_pointer(skb), NAPI_GRO_CB(skb)->frag0, grow);

		skb->tail += grow;
		skb->data_len -= grow;

		skb_shinfo(skb)->frags[0].page_offset += grow;
		skb_frag_size_sub(&skb_shinfo(skb)->frags[0], grow);

		if (unlikely(!skb_frag_size(&skb_shinfo(skb)->frags[0]))) {
			skb_frag_unref(skb, 0);
			memmove(skb_shinfo(skb)->frags,
				skb_shinfo(skb)->frags + 1,
				--skb_shinfo(skb)->nr_frags * sizeof(skb_frag_t));
		}
	}

ok:
	return ret;

normal:
	ret = GRO_NORMAL;
	goto pull;
}
EXPORT_SYMBOL(dev_gro_receive);

static inline gro_result_t
__napi_gro_receive(struct napi_struct *napi, struct sk_buff *skb)
{
	struct sk_buff *p;
	unsigned int maclen = skb->dev->hard_header_len;

	for (p = napi->gro_list; p; p = p->next) {
		unsigned long diffs;

		diffs = (unsigned long)p->dev ^ (unsigned long)skb->dev;
		diffs |= p->vlan_tci ^ skb->vlan_tci;
		if (maclen == ETH_HLEN)
			diffs |= compare_ether_header(skb_mac_header(p),
						      skb_gro_mac_header(skb));
		else if (!diffs)
			diffs = memcmp(skb_mac_header(p),
				       skb_gro_mac_header(skb),
				       maclen);
		NAPI_GRO_CB(p)->same_flow = !diffs;
		NAPI_GRO_CB(p)->flush = 0;
	}

	return dev_gro_receive(napi, skb);
}

gro_result_t napi_skb_finish(gro_result_t ret, struct sk_buff *skb)
{
	switch (ret) {
	case GRO_NORMAL:
		if (netif_receive_skb(skb))
			ret = GRO_DROP;
		break;

	case GRO_DROP:
	case GRO_MERGED_FREE:
		kfree_skb(skb);
		break;

	case GRO_HELD:
	case GRO_MERGED:
		break;
	}

	return ret;
}
EXPORT_SYMBOL(napi_skb_finish);

void skb_gro_reset_offset(struct sk_buff *skb)
{
	NAPI_GRO_CB(skb)->data_offset = 0;
	NAPI_GRO_CB(skb)->frag0 = NULL;
	NAPI_GRO_CB(skb)->frag0_len = 0;

	if (skb->mac_header == skb->tail &&
	    !PageHighMem(skb_frag_page(&skb_shinfo(skb)->frags[0]))) {
		NAPI_GRO_CB(skb)->frag0 =
			skb_frag_address(&skb_shinfo(skb)->frags[0]);
		NAPI_GRO_CB(skb)->frag0_len = skb_frag_size(&skb_shinfo(skb)->frags[0]);
	}
}
EXPORT_SYMBOL(skb_gro_reset_offset);

gro_result_t napi_gro_receive(struct napi_struct *napi, struct sk_buff *skb)
{
	skb_gro_reset_offset(skb);

	return napi_skb_finish(__napi_gro_receive(napi, skb), skb);
}
EXPORT_SYMBOL(napi_gro_receive);

static void napi_reuse_skb(struct napi_struct *napi, struct sk_buff *skb)
{
	__skb_pull(skb, skb_headlen(skb));
	
	skb_reserve(skb, NET_SKB_PAD + NET_IP_ALIGN - skb_headroom(skb));
	skb->vlan_tci = 0;
	skb->dev = napi->dev;
	skb->skb_iif = 0;

	napi->skb = skb;
}

struct sk_buff *napi_get_frags(struct napi_struct *napi)
{
	struct sk_buff *skb = napi->skb;

	if (!skb) {
		skb = netdev_alloc_skb_ip_align(napi->dev, GRO_MAX_HEAD);
		if (skb)
			napi->skb = skb;
	}
	return skb;
}
EXPORT_SYMBOL(napi_get_frags);

gro_result_t napi_frags_finish(struct napi_struct *napi, struct sk_buff *skb,
			       gro_result_t ret)
{
	switch (ret) {
	case GRO_NORMAL:
	case GRO_HELD:
		skb->protocol = eth_type_trans(skb, skb->dev);

		if (ret == GRO_HELD)
			skb_gro_pull(skb, -ETH_HLEN);
		else if (netif_receive_skb(skb))
			ret = GRO_DROP;
		break;

	case GRO_DROP:
	case GRO_MERGED_FREE:
		napi_reuse_skb(napi, skb);
		break;

	case GRO_MERGED:
		break;
	}

	return ret;
}
EXPORT_SYMBOL(napi_frags_finish);

struct sk_buff *napi_frags_skb(struct napi_struct *napi)
{
	struct sk_buff *skb = napi->skb;
	struct ethhdr *eth;
	unsigned int hlen;
	unsigned int off;

	napi->skb = NULL;

	skb_reset_mac_header(skb);
	skb_gro_reset_offset(skb);

	off = skb_gro_offset(skb);
	hlen = off + sizeof(*eth);
	eth = skb_gro_header_fast(skb, off);
	if (skb_gro_header_hard(skb, hlen)) {
		eth = skb_gro_header_slow(skb, hlen, off);
		if (unlikely(!eth)) {
			napi_reuse_skb(napi, skb);
			skb = NULL;
			goto out;
		}
	}

	skb_gro_pull(skb, sizeof(*eth));


#ifdef CONFIG_HTC_NETWORK_MODIFY
	if (IS_ERR(eth) || (!eth))
		printk(KERN_ERR "[CORE] eth is NULL in %s!\n", __func__);
#endif

	skb->protocol = eth->h_proto;

out:
	return skb;
}
EXPORT_SYMBOL(napi_frags_skb);

gro_result_t napi_gro_frags(struct napi_struct *napi)
{
	struct sk_buff *skb = napi_frags_skb(napi);

	if (!skb)
		return GRO_DROP;

	return napi_frags_finish(napi, skb, __napi_gro_receive(napi, skb));
}
EXPORT_SYMBOL(napi_gro_frags);

static void net_rps_action_and_irq_enable(struct softnet_data *sd)
{
#ifdef CONFIG_RPS
	struct softnet_data *remsd = sd->rps_ipi_list;

	if (remsd) {
		sd->rps_ipi_list = NULL;

		local_irq_enable();

		
		while (remsd) {
			struct softnet_data *next = remsd->rps_ipi_next;

			if (cpu_online(remsd->cpu))
				__smp_call_function_single(remsd->cpu,
							   &remsd->csd, 0);
			remsd = next;
		}
	} else
#endif
		local_irq_enable();
}

static int process_backlog(struct napi_struct *napi, int quota)
{
	int work = 0;
	struct softnet_data *sd = container_of(napi, struct softnet_data, backlog);

#ifdef CONFIG_RPS
	if (sd->rps_ipi_list) {
		local_irq_disable();
		net_rps_action_and_irq_enable(sd);
	}
#endif
	napi->weight = weight_p;
	local_irq_disable();
	while (work < quota) {
		struct sk_buff *skb;
		unsigned int qlen;

		while ((skb = __skb_dequeue(&sd->process_queue))) {
			local_irq_enable();
			__netif_receive_skb(skb);
			local_irq_disable();
			input_queue_head_incr(sd);
			if (++work >= quota) {
				local_irq_enable();
				return work;
			}
		}

		rps_lock(sd);
		qlen = skb_queue_len(&sd->input_pkt_queue);
		if (qlen)
			skb_queue_splice_tail_init(&sd->input_pkt_queue,
						   &sd->process_queue);

		if (qlen < quota - work) {
			list_del(&napi->poll_list);
			napi->state = 0;

			quota = work + qlen;
		}
		rps_unlock(sd);
	}
	local_irq_enable();

	return work;
}

void __napi_schedule(struct napi_struct *n)
{
	unsigned long flags;

	local_irq_save(flags);
	____napi_schedule(&__get_cpu_var(softnet_data), n);
	local_irq_restore(flags);
}
EXPORT_SYMBOL(__napi_schedule);

void __napi_complete(struct napi_struct *n)
{
	BUG_ON(!test_bit(NAPI_STATE_SCHED, &n->state));
	BUG_ON(n->gro_list);

	list_del(&n->poll_list);
	smp_mb__before_clear_bit();
	clear_bit(NAPI_STATE_SCHED, &n->state);
}
EXPORT_SYMBOL(__napi_complete);

void napi_complete(struct napi_struct *n)
{
	unsigned long flags;

	if (unlikely(test_bit(NAPI_STATE_NPSVC, &n->state)))
		return;

	napi_gro_flush(n);
	local_irq_save(flags);
	__napi_complete(n);
	local_irq_restore(flags);
}
EXPORT_SYMBOL(napi_complete);

void netif_napi_add(struct net_device *dev, struct napi_struct *napi,
		    int (*poll)(struct napi_struct *, int), int weight)
{
	INIT_LIST_HEAD(&napi->poll_list);
	napi->gro_count = 0;
	napi->gro_list = NULL;
	napi->skb = NULL;
	napi->poll = poll;
	napi->weight = weight;
	list_add(&napi->dev_list, &dev->napi_list);
	napi->dev = dev;
#ifdef CONFIG_NETPOLL
	spin_lock_init(&napi->poll_lock);
	napi->poll_owner = -1;
#endif
	set_bit(NAPI_STATE_SCHED, &napi->state);
}
EXPORT_SYMBOL(netif_napi_add);

void netif_napi_del(struct napi_struct *napi)
{
	struct sk_buff *skb, *next;

	list_del_init(&napi->dev_list);
	napi_free_frags(napi);

	for (skb = napi->gro_list; skb; skb = next) {
		next = skb->next;
		skb->next = NULL;
		kfree_skb(skb);
	}

	napi->gro_list = NULL;
	napi->gro_count = 0;
}
EXPORT_SYMBOL(netif_napi_del);

static void net_rx_action(struct softirq_action *h)
{
	struct softnet_data *sd = &__get_cpu_var(softnet_data);
	unsigned long time_limit = jiffies + 2;
	int budget = netdev_budget;
	void *have;

	local_irq_disable();

	while (!list_empty(&sd->poll_list)) {
		struct napi_struct *n;
		int work, weight;

		if (unlikely(budget <= 0 || time_after(jiffies, time_limit)))
			goto softnet_break;

		local_irq_enable();

		n = list_first_entry(&sd->poll_list, struct napi_struct, poll_list);

		have = netpoll_poll_lock(n);

		weight = n->weight;

		work = 0;
		if (test_bit(NAPI_STATE_SCHED, &n->state)) {
			work = n->poll(n, weight);
			trace_napi_poll(n);
		}

		WARN_ON_ONCE(work > weight);

		budget -= work;

		local_irq_disable();

		if (unlikely(work == weight)) {
			if (unlikely(napi_disable_pending(n))) {
				local_irq_enable();
				napi_complete(n);
				local_irq_disable();
			} else
				list_move_tail(&n->poll_list, &sd->poll_list);
		}

		netpoll_poll_unlock(have);
	}
out:
	net_rps_action_and_irq_enable(sd);

#ifdef CONFIG_NET_DMA
	dma_issue_pending_all();
#endif

	return;

softnet_break:
	sd->time_squeeze++;
	__raise_softirq_irqoff(NET_RX_SOFTIRQ);
	goto out;
}

static gifconf_func_t *gifconf_list[NPROTO];

int register_gifconf(unsigned int family, gifconf_func_t *gifconf)
{
	if (family >= NPROTO)
		return -EINVAL;
	gifconf_list[family] = gifconf;
	return 0;
}
EXPORT_SYMBOL(register_gifconf);




static int dev_ifname(struct net *net, struct ifreq __user *arg)
{
	struct net_device *dev;
	struct ifreq ifr;


	if (copy_from_user(&ifr, arg, sizeof(struct ifreq)))
		return -EFAULT;

	rcu_read_lock();
	dev = dev_get_by_index_rcu(net, ifr.ifr_ifindex);
	if (!dev) {
		rcu_read_unlock();
		return -ENODEV;
	}

	strcpy(ifr.ifr_name, dev->name);
	rcu_read_unlock();

	if (copy_to_user(arg, &ifr, sizeof(struct ifreq)))
		return -EFAULT;
	return 0;
}


static int dev_ifconf(struct net *net, char __user *arg)
{
	struct ifconf ifc;
	struct net_device *dev;
	char __user *pos;
	int len;
	int total;
	int i;


	if (copy_from_user(&ifc, arg, sizeof(struct ifconf)))
		return -EFAULT;

	pos = ifc.ifc_buf;
	len = ifc.ifc_len;


	total = 0;
	for_each_netdev(net, dev) {
		for (i = 0; i < NPROTO; i++) {
			if (gifconf_list[i]) {
				int done;
				if (!pos)
					done = gifconf_list[i](dev, NULL, 0);
				else
					done = gifconf_list[i](dev, pos + total,
							       len - total);
				if (done < 0)
					return -EFAULT;
				total += done;
			}
		}
	}

	ifc.ifc_len = total;

	return copy_to_user(arg, &ifc, sizeof(struct ifconf)) ? -EFAULT : 0;
}

#ifdef CONFIG_PROC_FS

#define BUCKET_SPACE (32 - NETDEV_HASHBITS - 1)

#define get_bucket(x) ((x) >> BUCKET_SPACE)
#define get_offset(x) ((x) & ((1 << BUCKET_SPACE) - 1))
#define set_bucket_offset(b, o) ((b) << BUCKET_SPACE | (o))

static inline struct net_device *dev_from_same_bucket(struct seq_file *seq, loff_t *pos)
{
	struct net *net = seq_file_net(seq);
	struct net_device *dev;
	struct hlist_node *p;
	struct hlist_head *h;
	unsigned int count = 0, offset = get_offset(*pos);

	h = &net->dev_name_head[get_bucket(*pos)];
	hlist_for_each_entry_rcu(dev, p, h, name_hlist) {
		if (++count == offset)
			return dev;
	}

	return NULL;
}

static inline struct net_device *dev_from_bucket(struct seq_file *seq, loff_t *pos)
{
	struct net_device *dev;
	unsigned int bucket;

	do {
		dev = dev_from_same_bucket(seq, pos);
		if (dev)
			return dev;

		bucket = get_bucket(*pos) + 1;
		*pos = set_bucket_offset(bucket, 1);
	} while (bucket < NETDEV_HASHENTRIES);

	return NULL;
}

void *dev_seq_start(struct seq_file *seq, loff_t *pos)
	__acquires(RCU)
{
	rcu_read_lock();
	if (!*pos)
		return SEQ_START_TOKEN;

	if (get_bucket(*pos) >= NETDEV_HASHENTRIES)
		return NULL;

	return dev_from_bucket(seq, pos);
}

void *dev_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	++*pos;
	return dev_from_bucket(seq, pos);
}

void dev_seq_stop(struct seq_file *seq, void *v)
	__releases(RCU)
{
	rcu_read_unlock();
}

static void dev_seq_printf_stats(struct seq_file *seq, struct net_device *dev)
{
	struct rtnl_link_stats64 temp;
	const struct rtnl_link_stats64 *stats = dev_get_stats(dev, &temp);

	seq_printf(seq, "%6s: %7llu %7llu %4llu %4llu %4llu %5llu %10llu %9llu "
		   "%8llu %7llu %4llu %4llu %4llu %5llu %7llu %10llu\n",
		   dev->name, stats->rx_bytes, stats->rx_packets,
		   stats->rx_errors,
		   stats->rx_dropped + stats->rx_missed_errors,
		   stats->rx_fifo_errors,
		   stats->rx_length_errors + stats->rx_over_errors +
		    stats->rx_crc_errors + stats->rx_frame_errors,
		   stats->rx_compressed, stats->multicast,
		   stats->tx_bytes, stats->tx_packets,
		   stats->tx_errors, stats->tx_dropped,
		   stats->tx_fifo_errors, stats->collisions,
		   stats->tx_carrier_errors +
		    stats->tx_aborted_errors +
		    stats->tx_window_errors +
		    stats->tx_heartbeat_errors,
		   stats->tx_compressed);
}

static int dev_seq_show(struct seq_file *seq, void *v)
{
	if (v == SEQ_START_TOKEN)
		seq_puts(seq, "Inter-|   Receive                            "
			      "                    |  Transmit\n"
			      " face |bytes    packets errs drop fifo frame "
			      "compressed multicast|bytes    packets errs "
			      "drop fifo colls carrier compressed\n");
	else
		dev_seq_printf_stats(seq, v);
	return 0;
}

static struct softnet_data *softnet_get_online(loff_t *pos)
{
	struct softnet_data *sd = NULL;

	while (*pos < nr_cpu_ids)
		if (cpu_online(*pos)) {
			sd = &per_cpu(softnet_data, *pos);
			break;
		} else
			++*pos;
	return sd;
}

static void *softnet_seq_start(struct seq_file *seq, loff_t *pos)
{
	return softnet_get_online(pos);
}

static void *softnet_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	++*pos;
	return softnet_get_online(pos);
}

static void softnet_seq_stop(struct seq_file *seq, void *v)
{
}

static int softnet_seq_show(struct seq_file *seq, void *v)
{
	struct softnet_data *sd = v;

	seq_printf(seq, "%08x %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",
		   sd->processed, sd->dropped, sd->time_squeeze, 0,
		   0, 0, 0, 0, 
		   sd->cpu_collision, sd->received_rps);
	return 0;
}

static const struct seq_operations dev_seq_ops = {
	.start = dev_seq_start,
	.next  = dev_seq_next,
	.stop  = dev_seq_stop,
	.show  = dev_seq_show,
};

static int dev_seq_open(struct inode *inode, struct file *file)
{
	return seq_open_net(inode, file, &dev_seq_ops,
			    sizeof(struct seq_net_private));
}

static const struct file_operations dev_seq_fops = {
	.owner	 = THIS_MODULE,
	.open    = dev_seq_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release_net,
};

static const struct seq_operations softnet_seq_ops = {
	.start = softnet_seq_start,
	.next  = softnet_seq_next,
	.stop  = softnet_seq_stop,
	.show  = softnet_seq_show,
};

static int softnet_seq_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &softnet_seq_ops);
}

static const struct file_operations softnet_seq_fops = {
	.owner	 = THIS_MODULE,
	.open    = softnet_seq_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release,
};

static void *ptype_get_idx(loff_t pos)
{
	struct packet_type *pt = NULL;
	loff_t i = 0;
	int t;

	list_for_each_entry_rcu(pt, &ptype_all, list) {
		if (i == pos)
			return pt;
		++i;
	}

	for (t = 0; t < PTYPE_HASH_SIZE; t++) {
		list_for_each_entry_rcu(pt, &ptype_base[t], list) {
			if (i == pos)
				return pt;
			++i;
		}
	}
	return NULL;
}

static void *ptype_seq_start(struct seq_file *seq, loff_t *pos)
	__acquires(RCU)
{
	rcu_read_lock();
	return *pos ? ptype_get_idx(*pos - 1) : SEQ_START_TOKEN;
}

static void *ptype_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct packet_type *pt;
	struct list_head *nxt;
	int hash;

	++*pos;
	if (v == SEQ_START_TOKEN)
		return ptype_get_idx(0);

	pt = v;
	nxt = pt->list.next;
	if (pt->type == htons(ETH_P_ALL)) {
		if (nxt != &ptype_all)
			goto found;
		hash = 0;
		nxt = ptype_base[0].next;
	} else
		hash = ntohs(pt->type) & PTYPE_HASH_MASK;

	while (nxt == &ptype_base[hash]) {
		if (++hash >= PTYPE_HASH_SIZE)
			return NULL;
		nxt = ptype_base[hash].next;
	}
found:
	return list_entry(nxt, struct packet_type, list);
}

static void ptype_seq_stop(struct seq_file *seq, void *v)
	__releases(RCU)
{
	rcu_read_unlock();
}

static int ptype_seq_show(struct seq_file *seq, void *v)
{
	struct packet_type *pt = v;

	if (v == SEQ_START_TOKEN)
		seq_puts(seq, "Type Device      Function\n");
	else if (pt->dev == NULL || dev_net(pt->dev) == seq_file_net(seq)) {
		if (pt->type == htons(ETH_P_ALL))
			seq_puts(seq, "ALL ");
		else
			seq_printf(seq, "%04x", ntohs(pt->type));

		seq_printf(seq, " %-8s %pF\n",
			   pt->dev ? pt->dev->name : "", pt->func);
	}

	return 0;
}

static const struct seq_operations ptype_seq_ops = {
	.start = ptype_seq_start,
	.next  = ptype_seq_next,
	.stop  = ptype_seq_stop,
	.show  = ptype_seq_show,
};

static int ptype_seq_open(struct inode *inode, struct file *file)
{
	return seq_open_net(inode, file, &ptype_seq_ops,
			sizeof(struct seq_net_private));
}

static const struct file_operations ptype_seq_fops = {
	.owner	 = THIS_MODULE,
	.open    = ptype_seq_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release_net,
};


static int __net_init dev_proc_net_init(struct net *net)
{
	int rc = -ENOMEM;

	if (!proc_net_fops_create(net, "dev", S_IRUGO, &dev_seq_fops))
		goto out;
	if (!proc_net_fops_create(net, "softnet_stat", S_IRUGO, &softnet_seq_fops))
		goto out_dev;
	if (!proc_net_fops_create(net, "ptype", S_IRUGO, &ptype_seq_fops))
		goto out_softnet;

	if (wext_proc_init(net))
		goto out_ptype;
	rc = 0;
out:
	return rc;
out_ptype:
	proc_net_remove(net, "ptype");
out_softnet:
	proc_net_remove(net, "softnet_stat");
out_dev:
	proc_net_remove(net, "dev");
	goto out;
}

static void __net_exit dev_proc_net_exit(struct net *net)
{
	wext_proc_exit(net);

	proc_net_remove(net, "ptype");
	proc_net_remove(net, "softnet_stat");
	proc_net_remove(net, "dev");
}

static struct pernet_operations __net_initdata dev_proc_ops = {
	.init = dev_proc_net_init,
	.exit = dev_proc_net_exit,
};

static int __init dev_proc_init(void)
{
	return register_pernet_subsys(&dev_proc_ops);
}
#else
#define dev_proc_init() 0
#endif	


int netdev_set_master(struct net_device *slave, struct net_device *master)
{
	struct net_device *old = slave->master;

	ASSERT_RTNL();

	if (master) {
		if (old)
			return -EBUSY;
		dev_hold(master);
	}

	slave->master = master;

	if (old)
		dev_put(old);
	return 0;
}
EXPORT_SYMBOL(netdev_set_master);

int netdev_set_bond_master(struct net_device *slave, struct net_device *master)
{
	int err;

	ASSERT_RTNL();

	err = netdev_set_master(slave, master);
	if (err)
		return err;
	if (master)
		slave->flags |= IFF_SLAVE;
	else
		slave->flags &= ~IFF_SLAVE;

	rtmsg_ifinfo(RTM_NEWLINK, slave, IFF_SLAVE);
	return 0;
}
EXPORT_SYMBOL(netdev_set_bond_master);

static void dev_change_rx_flags(struct net_device *dev, int flags)
{
	const struct net_device_ops *ops = dev->netdev_ops;

	if ((dev->flags & IFF_UP) && ops->ndo_change_rx_flags)
		ops->ndo_change_rx_flags(dev, flags);
}

static int __dev_set_promiscuity(struct net_device *dev, int inc)
{
	unsigned int old_flags = dev->flags;
	uid_t uid;
	gid_t gid;

	ASSERT_RTNL();

	dev->flags |= IFF_PROMISC;
	dev->promiscuity += inc;
	if (dev->promiscuity == 0) {
		if (inc < 0)
			dev->flags &= ~IFF_PROMISC;
		else {
			dev->promiscuity -= inc;
			pr_warn("%s: promiscuity touches roof, set promiscuity failed. promiscuity feature of device might be broken.\n",
				dev->name);
			return -EOVERFLOW;
		}
	}
	if (dev->flags != old_flags) {
		pr_info("device %s %s promiscuous mode\n",
			dev->name,
			dev->flags & IFF_PROMISC ? "entered" : "left");
		if (audit_enabled) {
			current_uid_gid(&uid, &gid);
			audit_log(current->audit_context, GFP_ATOMIC,
				AUDIT_ANOM_PROMISCUOUS,
				"dev=%s prom=%d old_prom=%d auid=%u uid=%u gid=%u ses=%u",
				dev->name, (dev->flags & IFF_PROMISC),
				(old_flags & IFF_PROMISC),
				audit_get_loginuid(current),
				uid, gid,
				audit_get_sessionid(current));
		}

		dev_change_rx_flags(dev, IFF_PROMISC);
	}
	return 0;
}

int dev_set_promiscuity(struct net_device *dev, int inc)
{
	unsigned int old_flags = dev->flags;
	int err;

	err = __dev_set_promiscuity(dev, inc);
	if (err < 0)
		return err;
	if (dev->flags != old_flags)
		dev_set_rx_mode(dev);
	return err;
}
EXPORT_SYMBOL(dev_set_promiscuity);


int dev_set_allmulti(struct net_device *dev, int inc)
{
	unsigned int old_flags = dev->flags;

	ASSERT_RTNL();

	dev->flags |= IFF_ALLMULTI;
	dev->allmulti += inc;
	if (dev->allmulti == 0) {
		if (inc < 0)
			dev->flags &= ~IFF_ALLMULTI;
		else {
			dev->allmulti -= inc;
			pr_warn("%s: allmulti touches roof, set allmulti failed. allmulti feature of device might be broken.\n",
				dev->name);
			return -EOVERFLOW;
		}
	}
	if (dev->flags ^ old_flags) {
		dev_change_rx_flags(dev, IFF_ALLMULTI);
		dev_set_rx_mode(dev);
	}
	return 0;
}
EXPORT_SYMBOL(dev_set_allmulti);

void __dev_set_rx_mode(struct net_device *dev)
{
	const struct net_device_ops *ops = dev->netdev_ops;

	
	if (!(dev->flags&IFF_UP))
		return;

	if (!netif_device_present(dev))
		return;

	if (!(dev->priv_flags & IFF_UNICAST_FLT)) {
		if (!netdev_uc_empty(dev) && !dev->uc_promisc) {
			__dev_set_promiscuity(dev, 1);
			dev->uc_promisc = true;
		} else if (netdev_uc_empty(dev) && dev->uc_promisc) {
			__dev_set_promiscuity(dev, -1);
			dev->uc_promisc = false;
		}
	}

	if (ops->ndo_set_rx_mode)
		ops->ndo_set_rx_mode(dev);
}

void dev_set_rx_mode(struct net_device *dev)
{
	netif_addr_lock_bh(dev);
	__dev_set_rx_mode(dev);
	netif_addr_unlock_bh(dev);
}

unsigned dev_get_flags(const struct net_device *dev)
{
	unsigned flags;

	flags = (dev->flags & ~(IFF_PROMISC |
				IFF_ALLMULTI |
				IFF_RUNNING |
				IFF_LOWER_UP |
				IFF_DORMANT)) |
		(dev->gflags & (IFF_PROMISC |
				IFF_ALLMULTI));

	if (netif_running(dev)) {
		if (netif_oper_up(dev))
			flags |= IFF_RUNNING;
		if (netif_carrier_ok(dev))
			flags |= IFF_LOWER_UP;
		if (netif_dormant(dev))
			flags |= IFF_DORMANT;
	}

	return flags;
}
EXPORT_SYMBOL(dev_get_flags);

int __dev_change_flags(struct net_device *dev, unsigned int flags)
{
	unsigned int old_flags = dev->flags;
	int ret;

	ASSERT_RTNL();


	dev->flags = (flags & (IFF_DEBUG | IFF_NOTRAILERS | IFF_NOARP |
			       IFF_DYNAMIC | IFF_MULTICAST | IFF_PORTSEL |
			       IFF_AUTOMEDIA)) |
		     (dev->flags & (IFF_UP | IFF_VOLATILE | IFF_PROMISC |
				    IFF_ALLMULTI));


	if ((old_flags ^ flags) & IFF_MULTICAST)
		dev_change_rx_flags(dev, IFF_MULTICAST);

	dev_set_rx_mode(dev);


	ret = 0;
	if ((old_flags ^ flags) & IFF_UP) {	
		ret = ((old_flags & IFF_UP) ? __dev_close : __dev_open)(dev);

		if (!ret)
			dev_set_rx_mode(dev);
	}

	if ((flags ^ dev->gflags) & IFF_PROMISC) {
		int inc = (flags & IFF_PROMISC) ? 1 : -1;

		dev->gflags ^= IFF_PROMISC;
		dev_set_promiscuity(dev, inc);
	}

	if ((flags ^ dev->gflags) & IFF_ALLMULTI) {
		int inc = (flags & IFF_ALLMULTI) ? 1 : -1;

		dev->gflags ^= IFF_ALLMULTI;
		dev_set_allmulti(dev, inc);
	}

	return ret;
}

void __dev_notify_flags(struct net_device *dev, unsigned int old_flags)
{
	unsigned int changes = dev->flags ^ old_flags;

	if (changes & IFF_UP) {
		if (dev->flags & IFF_UP)
			call_netdevice_notifiers(NETDEV_UP, dev);
		else
			call_netdevice_notifiers(NETDEV_DOWN, dev);
	}

	if (dev->flags & IFF_UP &&
	    (changes & ~(IFF_UP | IFF_PROMISC | IFF_ALLMULTI | IFF_VOLATILE)))
		call_netdevice_notifiers(NETDEV_CHANGE, dev);
}

int dev_change_flags(struct net_device *dev, unsigned int flags)
{
	int ret;
	unsigned int changes, old_flags = dev->flags;

	ret = __dev_change_flags(dev, flags);
	if (ret < 0)
		return ret;

	changes = old_flags ^ dev->flags;
	if (changes)
		rtmsg_ifinfo(RTM_NEWLINK, dev, changes);

	__dev_notify_flags(dev, old_flags);
	return ret;
}
EXPORT_SYMBOL(dev_change_flags);

int dev_set_mtu(struct net_device *dev, int new_mtu)
{
	const struct net_device_ops *ops = dev->netdev_ops;
	int err;

	if (new_mtu == dev->mtu)
		return 0;

	
	if (new_mtu < 0)
		return -EINVAL;

	if (!netif_device_present(dev))
		return -ENODEV;

	err = 0;
	if (ops->ndo_change_mtu)
		err = ops->ndo_change_mtu(dev, new_mtu);
	else
		dev->mtu = new_mtu;

	if (!err && dev->flags & IFF_UP)
		call_netdevice_notifiers(NETDEV_CHANGEMTU, dev);
	return err;
}
EXPORT_SYMBOL(dev_set_mtu);

void dev_set_group(struct net_device *dev, int new_group)
{
	dev->group = new_group;
}
EXPORT_SYMBOL(dev_set_group);

int dev_set_mac_address(struct net_device *dev, struct sockaddr *sa)
{
	const struct net_device_ops *ops = dev->netdev_ops;
	int err;

	if (!ops->ndo_set_mac_address)
		return -EOPNOTSUPP;
	if (sa->sa_family != dev->type)
		return -EINVAL;
	if (!netif_device_present(dev))
		return -ENODEV;
	err = ops->ndo_set_mac_address(dev, sa);
	if (!err)
		call_netdevice_notifiers(NETDEV_CHANGEADDR, dev);
	return err;
}
EXPORT_SYMBOL(dev_set_mac_address);

static int dev_ifsioc_locked(struct net *net, struct ifreq *ifr, unsigned int cmd)
{
	int err;
	struct net_device *dev = dev_get_by_name_rcu(net, ifr->ifr_name);

	if (!dev)
		return -ENODEV;

	switch (cmd) {
	case SIOCGIFFLAGS:	
		ifr->ifr_flags = (short) dev_get_flags(dev);
		return 0;

	case SIOCGIFMETRIC:	
		ifr->ifr_metric = 0;
		return 0;

	case SIOCGIFMTU:	
		ifr->ifr_mtu = dev->mtu;
		return 0;

	case SIOCGIFHWADDR:
		if (!dev->addr_len)
			memset(ifr->ifr_hwaddr.sa_data, 0, sizeof ifr->ifr_hwaddr.sa_data);
		else
			memcpy(ifr->ifr_hwaddr.sa_data, dev->dev_addr,
			       min(sizeof ifr->ifr_hwaddr.sa_data, (size_t) dev->addr_len));
		ifr->ifr_hwaddr.sa_family = dev->type;
		return 0;

	case SIOCGIFSLAVE:
		err = -EINVAL;
		break;

	case SIOCGIFMAP:
		ifr->ifr_map.mem_start = dev->mem_start;
		ifr->ifr_map.mem_end   = dev->mem_end;
		ifr->ifr_map.base_addr = dev->base_addr;
		ifr->ifr_map.irq       = dev->irq;
		ifr->ifr_map.dma       = dev->dma;
		ifr->ifr_map.port      = dev->if_port;
		return 0;

	case SIOCGIFINDEX:
		ifr->ifr_ifindex = dev->ifindex;
		return 0;

	case SIOCGIFTXQLEN:
		ifr->ifr_qlen = dev->tx_queue_len;
		return 0;

	default:
		WARN_ON(1);
		err = -ENOTTY;
		break;

	}
	return err;
}

static int dev_ifsioc(struct net *net, struct ifreq *ifr, unsigned int cmd)
{
	int err;
	struct net_device *dev = __dev_get_by_name(net, ifr->ifr_name);
	const struct net_device_ops *ops;

	if (!dev)
		return -ENODEV;

	ops = dev->netdev_ops;

	switch (cmd) {
	case SIOCSIFFLAGS:	
		return dev_change_flags(dev, ifr->ifr_flags);

	case SIOCSIFMETRIC:	
		return -EOPNOTSUPP;

	case SIOCSIFMTU:	
		return dev_set_mtu(dev, ifr->ifr_mtu);

	case SIOCSIFHWADDR:
		return dev_set_mac_address(dev, &ifr->ifr_hwaddr);

	case SIOCSIFHWBROADCAST:
		if (ifr->ifr_hwaddr.sa_family != dev->type)
			return -EINVAL;
		memcpy(dev->broadcast, ifr->ifr_hwaddr.sa_data,
		       min(sizeof ifr->ifr_hwaddr.sa_data, (size_t) dev->addr_len));
		call_netdevice_notifiers(NETDEV_CHANGEADDR, dev);
		return 0;

	case SIOCSIFMAP:
		if (ops->ndo_set_config) {
			if (!netif_device_present(dev))
				return -ENODEV;
			return ops->ndo_set_config(dev, &ifr->ifr_map);
		}
		return -EOPNOTSUPP;

	case SIOCADDMULTI:
		if (!ops->ndo_set_rx_mode ||
		    ifr->ifr_hwaddr.sa_family != AF_UNSPEC)
			return -EINVAL;
		if (!netif_device_present(dev))
			return -ENODEV;
		return dev_mc_add_global(dev, ifr->ifr_hwaddr.sa_data);

	case SIOCDELMULTI:
		if (!ops->ndo_set_rx_mode ||
		    ifr->ifr_hwaddr.sa_family != AF_UNSPEC)
			return -EINVAL;
		if (!netif_device_present(dev))
			return -ENODEV;
		return dev_mc_del_global(dev, ifr->ifr_hwaddr.sa_data);

	case SIOCSIFTXQLEN:
		if (ifr->ifr_qlen < 0)
			return -EINVAL;
		dev->tx_queue_len = ifr->ifr_qlen;
		return 0;

	case SIOCSIFNAME:
		ifr->ifr_newname[IFNAMSIZ-1] = '\0';
		return dev_change_name(dev, ifr->ifr_newname);

	case SIOCSHWTSTAMP:
		err = net_hwtstamp_validate(ifr);
		if (err)
			return err;
		

	default:
		if ((cmd >= SIOCDEVPRIVATE &&
		    cmd <= SIOCDEVPRIVATE + 15) ||
		    cmd == SIOCBONDENSLAVE ||
		    cmd == SIOCBONDRELEASE ||
		    cmd == SIOCBONDSETHWADDR ||
		    cmd == SIOCBONDSLAVEINFOQUERY ||
		    cmd == SIOCBONDINFOQUERY ||
		    cmd == SIOCBONDCHANGEACTIVE ||
		    cmd == SIOCGMIIPHY ||
		    cmd == SIOCGMIIREG ||
		    cmd == SIOCSMIIREG ||
		    cmd == SIOCBRADDIF ||
		    cmd == SIOCBRDELIF ||
		    cmd == SIOCSHWTSTAMP ||
		    cmd == SIOCWANDEV) {
			err = -EOPNOTSUPP;
			if (ops->ndo_do_ioctl) {
				if (netif_device_present(dev))
					err = ops->ndo_do_ioctl(dev, ifr, cmd);
				else
					err = -ENODEV;
			}
		} else
			err = -EINVAL;

	}
	return err;
}



int dev_ioctl(struct net *net, unsigned int cmd, void __user *arg)
{
	struct ifreq ifr;
	int ret;
	char *colon;


	if (cmd == SIOCGIFCONF) {
		rtnl_lock();
		ret = dev_ifconf(net, (char __user *) arg);
		rtnl_unlock();
		return ret;
	}
	if (cmd == SIOCGIFNAME)
		return dev_ifname(net, (struct ifreq __user *)arg);

	if (copy_from_user(&ifr, arg, sizeof(struct ifreq)))
		return -EFAULT;

	ifr.ifr_name[IFNAMSIZ-1] = 0;

	colon = strchr(ifr.ifr_name, ':');
	if (colon)
		*colon = 0;


	switch (cmd) {
	case SIOCGIFFLAGS:
	case SIOCGIFMETRIC:
	case SIOCGIFMTU:
	case SIOCGIFHWADDR:
	case SIOCGIFSLAVE:
	case SIOCGIFMAP:
	case SIOCGIFINDEX:
	case SIOCGIFTXQLEN:
		dev_load(net, ifr.ifr_name);
		rcu_read_lock();
		ret = dev_ifsioc_locked(net, &ifr, cmd);
		rcu_read_unlock();
		if (!ret) {
			if (colon)
				*colon = ':';
			if (copy_to_user(arg, &ifr,
					 sizeof(struct ifreq)))
				ret = -EFAULT;
		}
		return ret;

	case SIOCETHTOOL:
		dev_load(net, ifr.ifr_name);
		rtnl_lock();
		ret = dev_ethtool(net, &ifr);
		rtnl_unlock();
		if (!ret) {
			if (colon)
				*colon = ':';
			if (copy_to_user(arg, &ifr,
					 sizeof(struct ifreq)))
				ret = -EFAULT;
		}
		return ret;

	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSIFNAME:
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		dev_load(net, ifr.ifr_name);
		rtnl_lock();
		ret = dev_ifsioc(net, &ifr, cmd);
		rtnl_unlock();
		if (!ret) {
			if (colon)
				*colon = ':';
			if (copy_to_user(arg, &ifr,
					 sizeof(struct ifreq)))
				ret = -EFAULT;
		}
		return ret;

	case SIOCSIFFLAGS:
	case SIOCSIFMETRIC:
	case SIOCSIFMTU:
	case SIOCSIFMAP:
	case SIOCSIFHWADDR:
	case SIOCSIFSLAVE:
	case SIOCADDMULTI:
	case SIOCDELMULTI:
	case SIOCSIFHWBROADCAST:
	case SIOCSIFTXQLEN:
	case SIOCSMIIREG:
	case SIOCBONDENSLAVE:
	case SIOCBONDRELEASE:
	case SIOCBONDSETHWADDR:
	case SIOCBONDCHANGEACTIVE:
	case SIOCBRADDIF:
	case SIOCBRDELIF:
	case SIOCSHWTSTAMP:
		if (!capable(CAP_NET_ADMIN))
			return -EPERM;
		
	case SIOCBONDSLAVEINFOQUERY:
	case SIOCBONDINFOQUERY:
		dev_load(net, ifr.ifr_name);
		rtnl_lock();
		ret = dev_ifsioc(net, &ifr, cmd);
		rtnl_unlock();
		return ret;

	case SIOCGIFMEM:
	case SIOCSIFMEM:
	case SIOCSIFLINK:
		return -ENOTTY;

	default:
		if (cmd == SIOCWANDEV ||
		    (cmd >= SIOCDEVPRIVATE &&
		     cmd <= SIOCDEVPRIVATE + 15)) {
			dev_load(net, ifr.ifr_name);
			rtnl_lock();
			ret = dev_ifsioc(net, &ifr, cmd);
			rtnl_unlock();
			if (!ret && copy_to_user(arg, &ifr,
						 sizeof(struct ifreq)))
				ret = -EFAULT;
			return ret;
		}
		
		if (cmd >= SIOCIWFIRST && cmd <= SIOCIWLAST)
			return wext_handle_ioctl(net, &ifr, cmd, arg);
		return -ENOTTY;
	}
}


static int dev_new_index(struct net *net)
{
	static int ifindex;
	for (;;) {
		if (++ifindex <= 0)
			ifindex = 1;
		if (!__dev_get_by_index(net, ifindex))
			return ifindex;
	}
}

static LIST_HEAD(net_todo_list);

static void net_set_todo(struct net_device *dev)
{
	list_add_tail(&dev->todo_list, &net_todo_list);
}

static void rollback_registered_many(struct list_head *head)
{
	struct net_device *dev, *tmp;

	BUG_ON(dev_boot_phase);
	ASSERT_RTNL();

	list_for_each_entry_safe(dev, tmp, head, unreg_list) {
		if (dev->reg_state == NETREG_UNINITIALIZED) {
			pr_debug("unregister_netdevice: device %s/%p never was registered\n",
				 dev->name, dev);

			WARN_ON(1);
			list_del(&dev->unreg_list);
			continue;
		}
		dev->dismantle = true;
		BUG_ON(dev->reg_state != NETREG_REGISTERED);
	}

	
	dev_close_many(head);

	list_for_each_entry(dev, head, unreg_list) {
		
		unlist_netdevice(dev);

		dev->reg_state = NETREG_UNREGISTERING;
	}

	synchronize_net();

	list_for_each_entry(dev, head, unreg_list) {
		
		dev_shutdown(dev);


		call_netdevice_notifiers(NETDEV_UNREGISTER, dev);

		if (!dev->rtnl_link_ops ||
		    dev->rtnl_link_state == RTNL_LINK_INITIALIZED)
			rtmsg_ifinfo(RTM_DELLINK, dev, ~0U);

		dev_uc_flush(dev);
		dev_mc_flush(dev);

		if (dev->netdev_ops->ndo_uninit)
			dev->netdev_ops->ndo_uninit(dev);

		
		WARN_ON(dev->master);

		
		netdev_unregister_kobject(dev);
	}

	
	dev = list_first_entry(head, struct net_device, unreg_list);
	call_netdevice_notifiers(NETDEV_UNREGISTER_BATCH, dev);

	synchronize_net();

	list_for_each_entry(dev, head, unreg_list)
		dev_put(dev);
}

static void rollback_registered(struct net_device *dev)
{
	LIST_HEAD(single);

	list_add(&dev->unreg_list, &single);
	rollback_registered_many(&single);
	list_del(&single);
}

static netdev_features_t netdev_fix_features(struct net_device *dev,
	netdev_features_t features)
{
	
	if ((features & NETIF_F_HW_CSUM) &&
	    (features & (NETIF_F_IP_CSUM|NETIF_F_IPV6_CSUM))) {
		netdev_warn(dev, "mixed HW and IP checksum settings.\n");
		features &= ~(NETIF_F_IP_CSUM|NETIF_F_IPV6_CSUM);
	}

	
	if ((features & NETIF_F_SG) &&
	    !(features & NETIF_F_ALL_CSUM)) {
		netdev_dbg(dev,
			"Dropping NETIF_F_SG since no checksum feature.\n");
		features &= ~NETIF_F_SG;
	}

	
	if ((features & NETIF_F_ALL_TSO) && !(features & NETIF_F_SG)) {
		netdev_dbg(dev, "Dropping TSO features since no SG feature.\n");
		features &= ~NETIF_F_ALL_TSO;
	}

	
	if ((features & NETIF_F_ALL_TSO) == NETIF_F_TSO_ECN)
		features &= ~NETIF_F_TSO_ECN;

	
	if ((features & NETIF_F_GSO) && !(features & NETIF_F_SG)) {
		netdev_dbg(dev, "Dropping NETIF_F_GSO since no SG feature.\n");
		features &= ~NETIF_F_GSO;
	}

	
	if (features & NETIF_F_UFO) {
		
		if (!((features & NETIF_F_GEN_CSUM) ||
		    (features & (NETIF_F_IP_CSUM|NETIF_F_IPV6_CSUM))
			    == (NETIF_F_IP_CSUM|NETIF_F_IPV6_CSUM))) {
			netdev_dbg(dev,
				"Dropping NETIF_F_UFO since no checksum offload features.\n");
			features &= ~NETIF_F_UFO;
		}

		if (!(features & NETIF_F_SG)) {
			netdev_dbg(dev,
				"Dropping NETIF_F_UFO since no NETIF_F_SG feature.\n");
			features &= ~NETIF_F_UFO;
		}
	}

	return features;
}

int __netdev_update_features(struct net_device *dev)
{
	netdev_features_t features;
	int err = 0;

	ASSERT_RTNL();

	features = netdev_get_wanted_features(dev);

	if (dev->netdev_ops->ndo_fix_features)
		features = dev->netdev_ops->ndo_fix_features(dev, features);

	
	features = netdev_fix_features(dev, features);

	if (dev->features == features)
		return 0;

	netdev_dbg(dev, "Features changed: %pNF -> %pNF\n",
		&dev->features, &features);

	if (dev->netdev_ops->ndo_set_features)
		err = dev->netdev_ops->ndo_set_features(dev, features);

	if (unlikely(err < 0)) {
		netdev_err(dev,
			"set_features() failed (%d); wanted %pNF, left %pNF\n",
			err, &features, &dev->features);
		return -1;
	}

	if (!err)
		dev->features = features;

	return 1;
}

void netdev_update_features(struct net_device *dev)
{
	if (__netdev_update_features(dev))
		netdev_features_change(dev);
}
EXPORT_SYMBOL(netdev_update_features);

void netdev_change_features(struct net_device *dev)
{
	__netdev_update_features(dev);
	netdev_features_change(dev);
}
EXPORT_SYMBOL(netdev_change_features);

void netif_stacked_transfer_operstate(const struct net_device *rootdev,
					struct net_device *dev)
{
	if (rootdev->operstate == IF_OPER_DORMANT)
		netif_dormant_on(dev);
	else
		netif_dormant_off(dev);

	if (netif_carrier_ok(rootdev)) {
		if (!netif_carrier_ok(dev))
			netif_carrier_on(dev);
	} else {
		if (netif_carrier_ok(dev))
			netif_carrier_off(dev);
	}
}
EXPORT_SYMBOL(netif_stacked_transfer_operstate);

#ifdef CONFIG_RPS
static int netif_alloc_rx_queues(struct net_device *dev)
{
	unsigned int i, count = dev->num_rx_queues;
	struct netdev_rx_queue *rx;

	BUG_ON(count < 1);

	rx = kcalloc(count, sizeof(struct netdev_rx_queue), GFP_KERNEL);
	if (!rx) {
		pr_err("netdev: Unable to allocate %u rx queues\n", count);
		return -ENOMEM;
	}
	dev->_rx = rx;

	for (i = 0; i < count; i++)
		rx[i].dev = dev;
	return 0;
}
#endif

static void netdev_init_one_queue(struct net_device *dev,
				  struct netdev_queue *queue, void *_unused)
{
	
	spin_lock_init(&queue->_xmit_lock);
	netdev_set_xmit_lockdep_class(&queue->_xmit_lock, dev->type);
	queue->xmit_lock_owner = -1;
	netdev_queue_numa_node_write(queue, NUMA_NO_NODE);
	queue->dev = dev;
#ifdef CONFIG_BQL
	dql_init(&queue->dql, HZ);
#endif
}

static int netif_alloc_netdev_queues(struct net_device *dev)
{
	unsigned int count = dev->num_tx_queues;
	struct netdev_queue *tx;

	BUG_ON(count < 1);

	tx = kcalloc(count, sizeof(struct netdev_queue), GFP_KERNEL);
	if (!tx) {
		pr_err("netdev: Unable to allocate %u tx queues\n", count);
		return -ENOMEM;
	}
	dev->_tx = tx;

	netdev_for_each_tx_queue(dev, netdev_init_one_queue, NULL);
	spin_lock_init(&dev->tx_global_lock);

	return 0;
}


int register_netdevice(struct net_device *dev)
{
	int ret;
	struct net *net = dev_net(dev);

	BUG_ON(dev_boot_phase);
	ASSERT_RTNL();

	might_sleep();

	
	BUG_ON(dev->reg_state != NETREG_UNINITIALIZED);
	BUG_ON(!net);

	spin_lock_init(&dev->addr_list_lock);
	netdev_set_addr_lockdep_class(dev);

	dev->iflink = -1;

	ret = dev_get_valid_name(dev, dev->name);
	if (ret < 0)
		goto out;

	
	if (dev->netdev_ops->ndo_init) {
		ret = dev->netdev_ops->ndo_init(dev);
		if (ret) {
			if (ret > 0)
				ret = -EIO;
			goto out;
		}
	}

	dev->ifindex = dev_new_index(net);
	if (dev->iflink == -1)
		dev->iflink = dev->ifindex;

	dev->hw_features |= NETIF_F_SOFT_FEATURES;
	dev->features |= NETIF_F_SOFT_FEATURES;
	dev->wanted_features = dev->features & dev->hw_features;

	
	if (!(dev->flags & IFF_LOOPBACK)) {
		dev->hw_features |= NETIF_F_NOCACHE_COPY;
		if (dev->features & NETIF_F_ALL_CSUM) {
			dev->wanted_features |= NETIF_F_NOCACHE_COPY;
			dev->features |= NETIF_F_NOCACHE_COPY;
		}
	}

	dev->vlan_features |= NETIF_F_HIGHDMA;

	ret = call_netdevice_notifiers(NETDEV_POST_INIT, dev);
	ret = notifier_to_errno(ret);
	if (ret)
		goto err_uninit;

	ret = netdev_register_kobject(dev);
	if (ret)
		goto err_uninit;
	dev->reg_state = NETREG_REGISTERED;

	__netdev_update_features(dev);


	set_bit(__LINK_STATE_PRESENT, &dev->state);

	dev_init_scheduler(dev);
	dev_hold(dev);
	list_netdevice(dev);

	
	ret = call_netdevice_notifiers(NETDEV_REGISTER, dev);
	ret = notifier_to_errno(ret);
	if (ret) {
		rollback_registered(dev);
		dev->reg_state = NETREG_UNREGISTERED;
	}
	if (!dev->rtnl_link_ops ||
	    dev->rtnl_link_state == RTNL_LINK_INITIALIZED)
		rtmsg_ifinfo(RTM_NEWLINK, dev, ~0U);

out:
	return ret;

err_uninit:
	if (dev->netdev_ops->ndo_uninit)
		dev->netdev_ops->ndo_uninit(dev);
	goto out;
}
EXPORT_SYMBOL(register_netdevice);

int init_dummy_netdev(struct net_device *dev)
{
	memset(dev, 0, sizeof(struct net_device));

	dev->reg_state = NETREG_DUMMY;

	
	INIT_LIST_HEAD(&dev->napi_list);

	
	set_bit(__LINK_STATE_PRESENT, &dev->state);
	set_bit(__LINK_STATE_START, &dev->state);


	return 0;
}
EXPORT_SYMBOL_GPL(init_dummy_netdev);


int register_netdev(struct net_device *dev)
{
	int err;

	rtnl_lock();
	err = register_netdevice(dev);
	rtnl_unlock();
	return err;
}
EXPORT_SYMBOL(register_netdev);

int netdev_refcnt_read(const struct net_device *dev)
{
	int i, refcnt = 0;

	for_each_possible_cpu(i)
		refcnt += *per_cpu_ptr(dev->pcpu_refcnt, i);
	return refcnt;
}
EXPORT_SYMBOL(netdev_refcnt_read);

static void netdev_wait_allrefs(struct net_device *dev)
{
	unsigned long rebroadcast_time, warning_time;
	int refcnt;

	linkwatch_forget_dev(dev);

	rebroadcast_time = warning_time = jiffies;
	refcnt = netdev_refcnt_read(dev);

	while (refcnt != 0) {
		if (time_after(jiffies, rebroadcast_time + 1 * HZ)) {
			rtnl_lock();

			
			call_netdevice_notifiers(NETDEV_UNREGISTER, dev);

			if (test_bit(__LINK_STATE_LINKWATCH_PENDING,
				     &dev->state)) {
				linkwatch_run_queue();
			}

			__rtnl_unlock();

			rebroadcast_time = jiffies;
		}

		msleep(250);

		refcnt = netdev_refcnt_read(dev);

		if (time_after(jiffies, warning_time + 10 * HZ)) {
			pr_emerg("unregister_netdevice: waiting for %s to become free. Usage count = %d\n",
				 dev->name, refcnt);
			warning_time = jiffies;
		}
	}
}

void netdev_run_todo(void)
{
	struct list_head list;

	
	list_replace_init(&net_todo_list, &list);

	__rtnl_unlock();

	if (!list_empty(&list))
		rcu_barrier();

	while (!list_empty(&list)) {
		struct net_device *dev
			= list_first_entry(&list, struct net_device, todo_list);
		list_del(&dev->todo_list);

		if (unlikely(dev->reg_state != NETREG_UNREGISTERING)) {
			pr_err("network todo '%s' but state %d\n",
			       dev->name, dev->reg_state);
			dump_stack();
			continue;
		}

		dev->reg_state = NETREG_UNREGISTERED;

		on_each_cpu(flush_backlog, dev, 1);

		netdev_wait_allrefs(dev);

		
		BUG_ON(netdev_refcnt_read(dev));
		WARN_ON(rcu_access_pointer(dev->ip_ptr));
		WARN_ON(rcu_access_pointer(dev->ip6_ptr));
		WARN_ON(dev->dn_ptr);

		if (dev->destructor)
			dev->destructor(dev);

		
		kobject_put(&dev->dev.kobj);
	}
}

void netdev_stats_to_stats64(struct rtnl_link_stats64 *stats64,
			     const struct net_device_stats *netdev_stats)
{
#if BITS_PER_LONG == 64
	BUILD_BUG_ON(sizeof(*stats64) != sizeof(*netdev_stats));
	memcpy(stats64, netdev_stats, sizeof(*stats64));
#else
	size_t i, n = sizeof(*stats64) / sizeof(u64);
	const unsigned long *src = (const unsigned long *)netdev_stats;
	u64 *dst = (u64 *)stats64;

	BUILD_BUG_ON(sizeof(*netdev_stats) / sizeof(unsigned long) !=
		     sizeof(*stats64) / sizeof(u64));
	for (i = 0; i < n; i++)
		dst[i] = src[i];
#endif
}
EXPORT_SYMBOL(netdev_stats_to_stats64);

struct rtnl_link_stats64 *dev_get_stats(struct net_device *dev,
					struct rtnl_link_stats64 *storage)
{
	const struct net_device_ops *ops = dev->netdev_ops;

	if (ops->ndo_get_stats64) {
		memset(storage, 0, sizeof(*storage));
		ops->ndo_get_stats64(dev, storage);
	} else if (ops->ndo_get_stats) {
		netdev_stats_to_stats64(storage, ops->ndo_get_stats(dev));
	} else {
		netdev_stats_to_stats64(storage, &dev->stats);
	}
	storage->rx_dropped += atomic_long_read(&dev->rx_dropped);
	return storage;
}
EXPORT_SYMBOL(dev_get_stats);

struct netdev_queue *dev_ingress_queue_create(struct net_device *dev)
{
	struct netdev_queue *queue = dev_ingress_queue(dev);

#ifdef CONFIG_NET_CLS_ACT
	if (queue)
		return queue;
	queue = kzalloc(sizeof(*queue), GFP_KERNEL);
	if (!queue)
		return NULL;
	netdev_init_one_queue(dev, queue, NULL);
	queue->qdisc = &noop_qdisc;
	queue->qdisc_sleeping = &noop_qdisc;
	rcu_assign_pointer(dev->ingress_queue, queue);
#endif
	return queue;
}

struct net_device *alloc_netdev_mqs(int sizeof_priv, const char *name,
		void (*setup)(struct net_device *),
		unsigned int txqs, unsigned int rxqs)
{
	struct net_device *dev;
	size_t alloc_size;
	struct net_device *p;

	BUG_ON(strlen(name) >= sizeof(dev->name));

	if (txqs < 1) {
		pr_err("alloc_netdev: Unable to allocate device with zero queues\n");
		return NULL;
	}

#ifdef CONFIG_RPS
	if (rxqs < 1) {
		pr_err("alloc_netdev: Unable to allocate device with zero RX queues\n");
		return NULL;
	}
#endif

	alloc_size = sizeof(struct net_device);
	if (sizeof_priv) {
		
		alloc_size = ALIGN(alloc_size, NETDEV_ALIGN);
		alloc_size += sizeof_priv;
	}
	
	alloc_size += NETDEV_ALIGN - 1;

	p = kzalloc(alloc_size, GFP_KERNEL);
	if (!p) {
		pr_err("alloc_netdev: Unable to allocate device\n");
		return NULL;
	}

	dev = PTR_ALIGN(p, NETDEV_ALIGN);
	dev->padded = (char *)dev - (char *)p;

	dev->pcpu_refcnt = alloc_percpu(int);
	if (!dev->pcpu_refcnt)
		goto free_p;

	if (dev_addr_init(dev))
		goto free_pcpu;

	dev_mc_init(dev);
	dev_uc_init(dev);

	dev_net_set(dev, &init_net);

	dev->gso_max_size = GSO_MAX_SIZE;

	INIT_LIST_HEAD(&dev->napi_list);
	INIT_LIST_HEAD(&dev->unreg_list);
	INIT_LIST_HEAD(&dev->link_watch_list);
	dev->priv_flags = IFF_XMIT_DST_RELEASE;
	setup(dev);

	dev->num_tx_queues = txqs;
	dev->real_num_tx_queues = txqs;
	if (netif_alloc_netdev_queues(dev))
		goto free_all;

#ifdef CONFIG_RPS
	dev->num_rx_queues = rxqs;
	dev->real_num_rx_queues = rxqs;
	if (netif_alloc_rx_queues(dev))
		goto free_all;
#endif

	strcpy(dev->name, name);
	dev->group = INIT_NETDEV_GROUP;
	return dev;

free_all:
	free_netdev(dev);
	return NULL;

free_pcpu:
	free_percpu(dev->pcpu_refcnt);
	kfree(dev->_tx);
#ifdef CONFIG_RPS
	kfree(dev->_rx);
#endif

free_p:
	kfree(p);
	return NULL;
}
EXPORT_SYMBOL(alloc_netdev_mqs);

void free_netdev(struct net_device *dev)
{
	struct napi_struct *p, *n;

	release_net(dev_net(dev));

	kfree(dev->_tx);
#ifdef CONFIG_RPS
	kfree(dev->_rx);
#endif

	kfree(rcu_dereference_protected(dev->ingress_queue, 1));

	
	dev_addr_flush(dev);

	list_for_each_entry_safe(p, n, &dev->napi_list, dev_list)
		netif_napi_del(p);

	free_percpu(dev->pcpu_refcnt);
	dev->pcpu_refcnt = NULL;

	
	if (dev->reg_state == NETREG_UNINITIALIZED) {
		kfree((char *)dev - dev->padded);
		return;
	}

	BUG_ON(dev->reg_state != NETREG_UNREGISTERED);
	dev->reg_state = NETREG_RELEASED;

	
	put_device(&dev->dev);
}
EXPORT_SYMBOL(free_netdev);

void synchronize_net(void)
{
	might_sleep();
	if (rtnl_is_locked())
		synchronize_rcu_expedited();
	else
		synchronize_rcu();
}
EXPORT_SYMBOL(synchronize_net);


void unregister_netdevice_queue(struct net_device *dev, struct list_head *head)
{
	ASSERT_RTNL();

	if (head) {
		list_move_tail(&dev->unreg_list, head);
	} else {
		rollback_registered(dev);
		
		net_set_todo(dev);
	}
}
EXPORT_SYMBOL(unregister_netdevice_queue);

void unregister_netdevice_many(struct list_head *head)
{
	struct net_device *dev;

	if (!list_empty(head)) {
		rollback_registered_many(head);
		list_for_each_entry(dev, head, unreg_list)
			net_set_todo(dev);
	}
}
EXPORT_SYMBOL(unregister_netdevice_many);

void unregister_netdev(struct net_device *dev)
{
	rtnl_lock();
	unregister_netdevice(dev);
	rtnl_unlock();
}
EXPORT_SYMBOL(unregister_netdev);


int dev_change_net_namespace(struct net_device *dev, struct net *net, const char *pat)
{
	int err;

	ASSERT_RTNL();

	
	err = -EINVAL;
	if (dev->features & NETIF_F_NETNS_LOCAL)
		goto out;

	
	err = -EINVAL;
	if (dev->reg_state != NETREG_REGISTERED)
		goto out;

	
	err = 0;
	if (net_eq(dev_net(dev), net))
		goto out;

	err = -EEXIST;
	if (__dev_get_by_name(net, dev->name)) {
		
		if (!pat)
			goto out;
		if (dev_get_valid_name(dev, pat) < 0)
			goto out;
	}


	
	dev_close(dev);

	
	err = -ENODEV;
	unlist_netdevice(dev);

	synchronize_net();

	
	dev_shutdown(dev);

	call_netdevice_notifiers(NETDEV_UNREGISTER, dev);
	call_netdevice_notifiers(NETDEV_UNREGISTER_BATCH, dev);
	rtmsg_ifinfo(RTM_DELLINK, dev, ~0U);

	dev_uc_flush(dev);
	dev_mc_flush(dev);

	
	dev_net_set(dev, net);

	
	if (__dev_get_by_index(net, dev->ifindex)) {
		int iflink = (dev->iflink == dev->ifindex);
		dev->ifindex = dev_new_index(net);
		if (iflink)
			dev->iflink = dev->ifindex;
	}

	
	err = device_rename(&dev->dev, dev->name);
	WARN_ON(err);

	
	list_netdevice(dev);

	
	call_netdevice_notifiers(NETDEV_REGISTER, dev);

	rtmsg_ifinfo(RTM_NEWLINK, dev, ~0U);

	synchronize_net();
	err = 0;
out:
	return err;
}
EXPORT_SYMBOL_GPL(dev_change_net_namespace);

static int dev_cpu_callback(struct notifier_block *nfb,
			    unsigned long action,
			    void *ocpu)
{
	struct sk_buff **list_skb;
	struct sk_buff *skb;
	unsigned int cpu, oldcpu = (unsigned long)ocpu;
	struct softnet_data *sd, *oldsd;

	if (action != CPU_DEAD && action != CPU_DEAD_FROZEN)
		return NOTIFY_OK;

	local_irq_disable();
	cpu = smp_processor_id();
	sd = &per_cpu(softnet_data, cpu);
	oldsd = &per_cpu(softnet_data, oldcpu);

	
	list_skb = &sd->completion_queue;
	while (*list_skb)
		list_skb = &(*list_skb)->next;
	
	*list_skb = oldsd->completion_queue;
	oldsd->completion_queue = NULL;

	
	if (oldsd->output_queue) {
		*sd->output_queue_tailp = oldsd->output_queue;
		sd->output_queue_tailp = oldsd->output_queue_tailp;
		oldsd->output_queue = NULL;
		oldsd->output_queue_tailp = &oldsd->output_queue;
	}
	
	if (!list_empty(&oldsd->poll_list)) {
		list_splice_init(&oldsd->poll_list, &sd->poll_list);
		raise_softirq_irqoff(NET_RX_SOFTIRQ);
	}

	raise_softirq_irqoff(NET_TX_SOFTIRQ);
	local_irq_enable();

	
	while ((skb = __skb_dequeue(&oldsd->process_queue))) {
		netif_rx(skb);
		input_queue_head_incr(oldsd);
	}
	while ((skb = __skb_dequeue(&oldsd->input_pkt_queue))) {
		netif_rx(skb);
		input_queue_head_incr(oldsd);
	}

	return NOTIFY_OK;
}


netdev_features_t netdev_increment_features(netdev_features_t all,
	netdev_features_t one, netdev_features_t mask)
{
	if (mask & NETIF_F_GEN_CSUM)
		mask |= NETIF_F_ALL_CSUM;
	mask |= NETIF_F_VLAN_CHALLENGED;

	all |= one & (NETIF_F_ONE_FOR_ALL|NETIF_F_ALL_CSUM) & mask;
	all &= one | ~NETIF_F_ALL_FOR_ALL;

	
	if (all & NETIF_F_GEN_CSUM)
		all &= ~(NETIF_F_ALL_CSUM & ~NETIF_F_GEN_CSUM);

	return all;
}
EXPORT_SYMBOL(netdev_increment_features);

static struct hlist_head *netdev_create_hash(void)
{
	int i;
	struct hlist_head *hash;

	hash = kmalloc(sizeof(*hash) * NETDEV_HASHENTRIES, GFP_KERNEL);
	if (hash != NULL)
		for (i = 0; i < NETDEV_HASHENTRIES; i++)
			INIT_HLIST_HEAD(&hash[i]);

	return hash;
}

static int __net_init netdev_init(struct net *net)
{
	INIT_LIST_HEAD(&net->dev_base_head);

	net->dev_name_head = netdev_create_hash();
	if (net->dev_name_head == NULL)
		goto err_name;

	net->dev_index_head = netdev_create_hash();
	if (net->dev_index_head == NULL)
		goto err_idx;

	return 0;

err_idx:
	kfree(net->dev_name_head);
err_name:
	return -ENOMEM;
}

const char *netdev_drivername(const struct net_device *dev)
{
	const struct device_driver *driver;
	const struct device *parent;
	const char *empty = "";

	parent = dev->dev.parent;
	if (!parent)
		return empty;

	driver = parent->driver;
	if (driver && driver->name)
		return driver->name;
	return empty;
}

int __netdev_printk(const char *level, const struct net_device *dev,
			   struct va_format *vaf)
{
	int r;

	if (dev && dev->dev.parent)
		r = dev_printk(level, dev->dev.parent, "%s: %pV",
			       netdev_name(dev), vaf);
	else if (dev)
		r = printk("%s%s: %pV", level, netdev_name(dev), vaf);
	else
		r = printk("%s(NULL net_device): %pV", level, vaf);

	return r;
}
EXPORT_SYMBOL(__netdev_printk);

int netdev_printk(const char *level, const struct net_device *dev,
		  const char *format, ...)
{
	struct va_format vaf;
	va_list args;
	int r;

	va_start(args, format);

	vaf.fmt = format;
	vaf.va = &args;

	r = __netdev_printk(level, dev, &vaf);
	va_end(args);

	return r;
}
EXPORT_SYMBOL(netdev_printk);

#define define_netdev_printk_level(func, level)			\
int func(const struct net_device *dev, const char *fmt, ...)	\
{								\
	int r;							\
	struct va_format vaf;					\
	va_list args;						\
								\
	va_start(args, fmt);					\
								\
	vaf.fmt = fmt;						\
	vaf.va = &args;						\
								\
	r = __netdev_printk(level, dev, &vaf);			\
	va_end(args);						\
								\
	return r;						\
}								\
EXPORT_SYMBOL(func);

define_netdev_printk_level(netdev_emerg, KERN_EMERG);
define_netdev_printk_level(netdev_alert, KERN_ALERT);
define_netdev_printk_level(netdev_crit, KERN_CRIT);
define_netdev_printk_level(netdev_err, KERN_ERR);
define_netdev_printk_level(netdev_warn, KERN_WARNING);
define_netdev_printk_level(netdev_notice, KERN_NOTICE);
define_netdev_printk_level(netdev_info, KERN_INFO);

static void __net_exit netdev_exit(struct net *net)
{
	kfree(net->dev_name_head);
	kfree(net->dev_index_head);
}

static struct pernet_operations __net_initdata netdev_net_ops = {
	.init = netdev_init,
	.exit = netdev_exit,
};

static void __net_exit default_device_exit(struct net *net)
{
	struct net_device *dev, *aux;
	rtnl_lock();
	for_each_netdev_safe(net, dev, aux) {
		int err;
		char fb_name[IFNAMSIZ];

		
		if (dev->features & NETIF_F_NETNS_LOCAL)
			continue;

		
		if (dev->rtnl_link_ops)
			continue;

		
		snprintf(fb_name, IFNAMSIZ, "dev%d", dev->ifindex);
		err = dev_change_net_namespace(dev, &init_net, fb_name);
		if (err) {
			pr_emerg("%s: failed to move %s to init_net: %d\n",
				 __func__, dev->name, err);
			BUG();
		}
	}
	rtnl_unlock();
}

static void __net_exit default_device_exit_batch(struct list_head *net_list)
{
	struct net_device *dev;
	struct net *net;
	LIST_HEAD(dev_kill_list);

	rtnl_lock();
	list_for_each_entry(net, net_list, exit_list) {
		for_each_netdev_reverse(net, dev) {
			if (dev->rtnl_link_ops)
				dev->rtnl_link_ops->dellink(dev, &dev_kill_list);
			else
				unregister_netdevice_queue(dev, &dev_kill_list);
		}
	}
	unregister_netdevice_many(&dev_kill_list);
	list_del(&dev_kill_list);
	rtnl_unlock();
}

static struct pernet_operations __net_initdata default_device_ops = {
	.exit = default_device_exit,
	.exit_batch = default_device_exit_batch,
};


static int __init net_dev_init(void)
{
	int i, rc = -ENOMEM;

	BUG_ON(!dev_boot_phase);

	if (dev_proc_init())
		goto out;

	if (netdev_kobject_init())
		goto out;

	INIT_LIST_HEAD(&ptype_all);
	for (i = 0; i < PTYPE_HASH_SIZE; i++)
		INIT_LIST_HEAD(&ptype_base[i]);

	if (register_pernet_subsys(&netdev_net_ops))
		goto out;


	for_each_possible_cpu(i) {
		struct softnet_data *sd = &per_cpu(softnet_data, i);

		memset(sd, 0, sizeof(*sd));
		skb_queue_head_init(&sd->input_pkt_queue);
		skb_queue_head_init(&sd->process_queue);
		sd->completion_queue = NULL;
		INIT_LIST_HEAD(&sd->poll_list);
		sd->output_queue = NULL;
		sd->output_queue_tailp = &sd->output_queue;
#ifdef CONFIG_RPS
		sd->csd.func = rps_trigger_softirq;
		sd->csd.info = sd;
		sd->csd.flags = 0;
		sd->cpu = i;
#endif

		sd->backlog.poll = process_backlog;
		sd->backlog.weight = weight_p;
		sd->backlog.gro_list = NULL;
		sd->backlog.gro_count = 0;
	}

	dev_boot_phase = 0;

	if (register_pernet_device(&loopback_net_ops))
		goto out;

	if (register_pernet_device(&default_device_ops))
		goto out;

	open_softirq(NET_TX_SOFTIRQ, net_tx_action);
	open_softirq(NET_RX_SOFTIRQ, net_rx_action);

	hotcpu_notifier(dev_cpu_callback, 0);
	dst_init();
	dev_mcast_init();
	rc = 0;
out:
	return rc;
}

subsys_initcall(net_dev_init);

static int __init initialize_hashrnd(void)
{
	get_random_bytes(&hashrnd, sizeof(hashrnd));
	return 0;
}

late_initcall_sync(initialize_hashrnd);

