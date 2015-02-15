/*
 * INET		An implementation of the TCP/IP protocol suite for the LINUX
 *		operating system.  INET is implemented using the  BSD Socket
 *		interface as the means of communication with the user level.
 *
 *		Definitions for the Interfaces handler.
 *
 * Version:	@(#)dev.h	1.0.10	08/12/93
 *
 * Authors:	Ross Biro
 *		Fred N. van Kempen, <waltje@uWalt.NL.Mugnet.ORG>
 *		Corey Minyard <wf-rch!minyard@relay.EU.net>
 *		Donald J. Becker, <becker@cesdis.gsfc.nasa.gov>
 *		Alan Cox, <alan@lxorguk.ukuu.org.uk>
 *		Bjorn Ekwall. <bj0rn@blox.se>
 *              Pekka Riikonen <priikone@poseidon.pspt.fi>
 *
 *		This program is free software; you can redistribute it and/or
 *		modify it under the terms of the GNU General Public License
 *		as published by the Free Software Foundation; either version
 *		2 of the License, or (at your option) any later version.
 *
 *		Moved to /usr/include/linux for NET3
 */
#ifndef _LINUX_NETDEVICE_H
#define _LINUX_NETDEVICE_H

#include <linux/if.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <linux/if_link.h>

#ifdef __KERNEL__
#include <linux/pm_qos.h>
#include <linux/timer.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <asm/cache.h>
#include <asm/byteorder.h>

#include <linux/percpu.h>
#include <linux/rculist.h>
#include <linux/dmaengine.h>
#include <linux/workqueue.h>
#include <linux/dynamic_queue_limits.h>

#include <linux/ethtool.h>
#include <net/net_namespace.h>
#include <net/dsa.h>
#ifdef CONFIG_DCB
#include <net/dcbnl.h>
#endif
#include <net/netprio_cgroup.h>

#include <linux/netdev_features.h>

struct netpoll_info;
struct device;
struct phy_device;
struct wireless_dev;
					
#define SET_ETHTOOL_OPS(netdev,ops) \
	( (netdev)->ethtool_ops = (ops) )

#define NET_ADDR_PERM		0	
#define NET_ADDR_RANDOM		1	
#define NET_ADDR_STOLEN		2	

#define NET_RX_SUCCESS		0	
#define NET_RX_DROP		1	


#define NET_XMIT_SUCCESS	0x00
#define NET_XMIT_DROP		0x01	
#define NET_XMIT_CN		0x02	
#define NET_XMIT_POLICED	0x03	
#define NET_XMIT_MASK		0x0f	

#define net_xmit_eval(e)	((e) == NET_XMIT_CN ? 0 : (e))
#define net_xmit_errno(e)	((e) != NET_XMIT_CN ? -ENOBUFS : 0)

#define NETDEV_TX_MASK		0xf0

enum netdev_tx {
	__NETDEV_TX_MIN	 = INT_MIN,	
	NETDEV_TX_OK	 = 0x00,	
	NETDEV_TX_BUSY	 = 0x10,	
	NETDEV_TX_LOCKED = 0x20,	
};
typedef enum netdev_tx netdev_tx_t;

static inline bool dev_xmit_complete(int rc)
{
	if (likely(rc < NET_XMIT_MASK))
		return true;

	return false;
}

#endif

#define MAX_ADDR_LEN	32		

#define INIT_NETDEV_GROUP	0

#ifdef  __KERNEL__

#if defined(CONFIG_WLAN) || IS_ENABLED(CONFIG_AX25)
# if defined(CONFIG_MAC80211_MESH)
#  define LL_MAX_HEADER 128
# else
#  define LL_MAX_HEADER 96
# endif
#elif IS_ENABLED(CONFIG_TR)
# define LL_MAX_HEADER 48
#else
# define LL_MAX_HEADER 32
#endif

#if !IS_ENABLED(CONFIG_NET_IPIP) && !IS_ENABLED(CONFIG_NET_IPGRE) && \
    !IS_ENABLED(CONFIG_IPV6_SIT) && !IS_ENABLED(CONFIG_IPV6_TUNNEL)
#define MAX_HEADER LL_MAX_HEADER
#else
#define MAX_HEADER (LL_MAX_HEADER + 48)
#endif

/*
 *	Old network device statistics. Fields are native words
 *	(unsigned long) so they can be read and written atomically.
 */

struct net_device_stats {
	unsigned long	rx_packets;
	unsigned long	tx_packets;
	unsigned long	rx_bytes;
	unsigned long	tx_bytes;
	unsigned long	rx_errors;
	unsigned long	tx_errors;
	unsigned long	rx_dropped;
	unsigned long	tx_dropped;
	unsigned long	multicast;
	unsigned long	collisions;
	unsigned long	rx_length_errors;
	unsigned long	rx_over_errors;
	unsigned long	rx_crc_errors;
	unsigned long	rx_frame_errors;
	unsigned long	rx_fifo_errors;
	unsigned long	rx_missed_errors;
	unsigned long	tx_aborted_errors;
	unsigned long	tx_carrier_errors;
	unsigned long	tx_fifo_errors;
	unsigned long	tx_heartbeat_errors;
	unsigned long	tx_window_errors;
	unsigned long	rx_compressed;
	unsigned long	tx_compressed;
};

#endif  


enum {
        IF_PORT_UNKNOWN = 0,
        IF_PORT_10BASE2,
        IF_PORT_10BASET,
        IF_PORT_AUI,
        IF_PORT_100BASET,
        IF_PORT_100BASETX,
        IF_PORT_100BASEFX
};

#ifdef __KERNEL__

#include <linux/cache.h>
#include <linux/skbuff.h>

#ifdef CONFIG_RPS
#include <linux/static_key.h>
extern struct static_key rps_needed;
#endif

struct neighbour;
struct neigh_parms;
struct sk_buff;

struct netdev_hw_addr {
	struct list_head	list;
	unsigned char		addr[MAX_ADDR_LEN];
	unsigned char		type;
#define NETDEV_HW_ADDR_T_LAN		1
#define NETDEV_HW_ADDR_T_SAN		2
#define NETDEV_HW_ADDR_T_SLAVE		3
#define NETDEV_HW_ADDR_T_UNICAST	4
#define NETDEV_HW_ADDR_T_MULTICAST	5
	bool			synced;
	bool			global_use;
	int			refcount;
	struct rcu_head		rcu_head;
};

struct netdev_hw_addr_list {
	struct list_head	list;
	int			count;
};

#define netdev_hw_addr_list_count(l) ((l)->count)
#define netdev_hw_addr_list_empty(l) (netdev_hw_addr_list_count(l) == 0)
#define netdev_hw_addr_list_for_each(ha, l) \
	list_for_each_entry(ha, &(l)->list, list)

#define netdev_uc_count(dev) netdev_hw_addr_list_count(&(dev)->uc)
#define netdev_uc_empty(dev) netdev_hw_addr_list_empty(&(dev)->uc)
#define netdev_for_each_uc_addr(ha, dev) \
	netdev_hw_addr_list_for_each(ha, &(dev)->uc)

#define netdev_mc_count(dev) netdev_hw_addr_list_count(&(dev)->mc)
#define netdev_mc_empty(dev) netdev_hw_addr_list_empty(&(dev)->mc)
#define netdev_for_each_mc_addr(ha, dev) \
	netdev_hw_addr_list_for_each(ha, &(dev)->mc)

struct hh_cache {
	u16		hh_len;
	u16		__pad;
	seqlock_t	hh_lock;

	
#define HH_DATA_MOD	16
#define HH_DATA_OFF(__len) \
	(HH_DATA_MOD - (((__len - 1) & (HH_DATA_MOD - 1)) + 1))
#define HH_DATA_ALIGN(__len) \
	(((__len)+(HH_DATA_MOD-1))&~(HH_DATA_MOD - 1))
	unsigned long	hh_data[HH_DATA_ALIGN(LL_MAX_HEADER) / sizeof(long)];
};

#define LL_RESERVED_SPACE(dev) \
	((((dev)->hard_header_len+(dev)->needed_headroom)&~(HH_DATA_MOD - 1)) + HH_DATA_MOD)
#define LL_RESERVED_SPACE_EXTRA(dev,extra) \
	((((dev)->hard_header_len+(dev)->needed_headroom+(extra))&~(HH_DATA_MOD - 1)) + HH_DATA_MOD)

struct header_ops {
	int	(*create) (struct sk_buff *skb, struct net_device *dev,
			   unsigned short type, const void *daddr,
			   const void *saddr, unsigned len);
	int	(*parse)(const struct sk_buff *skb, unsigned char *haddr);
	int	(*rebuild)(struct sk_buff *skb);
	int	(*cache)(const struct neighbour *neigh, struct hh_cache *hh, __be16 type);
	void	(*cache_update)(struct hh_cache *hh,
				const struct net_device *dev,
				const unsigned char *haddr);
};


enum netdev_state_t {
	__LINK_STATE_START,
	__LINK_STATE_PRESENT,
	__LINK_STATE_NOCARRIER,
	__LINK_STATE_LINKWATCH_PENDING,
	__LINK_STATE_DORMANT,
};


struct netdev_boot_setup {
	char name[IFNAMSIZ];
	struct ifmap map;
};
#define NETDEV_BOOT_SETUP_MAX 8

extern int __init netdev_boot_setup(char *str);

struct napi_struct {
	struct list_head	poll_list;

	unsigned long		state;
	int			weight;
	int			(*poll)(struct napi_struct *, int);
#ifdef CONFIG_NETPOLL
	spinlock_t		poll_lock;
	int			poll_owner;
#endif

	unsigned int		gro_count;

	struct net_device	*dev;
	struct list_head	dev_list;
	struct sk_buff		*gro_list;
	struct sk_buff		*skb;
};

enum {
	NAPI_STATE_SCHED,	
	NAPI_STATE_DISABLE,	
	NAPI_STATE_NPSVC,	
};

enum gro_result {
	GRO_MERGED,
	GRO_MERGED_FREE,
	GRO_HELD,
	GRO_NORMAL,
	GRO_DROP,
};
typedef enum gro_result gro_result_t;


enum rx_handler_result {
	RX_HANDLER_CONSUMED,
	RX_HANDLER_ANOTHER,
	RX_HANDLER_EXACT,
	RX_HANDLER_PASS,
};
typedef enum rx_handler_result rx_handler_result_t;
typedef rx_handler_result_t rx_handler_func_t(struct sk_buff **pskb);

extern void __napi_schedule(struct napi_struct *n);

static inline bool napi_disable_pending(struct napi_struct *n)
{
	return test_bit(NAPI_STATE_DISABLE, &n->state);
}

static inline bool napi_schedule_prep(struct napi_struct *n)
{
	return !napi_disable_pending(n) &&
		!test_and_set_bit(NAPI_STATE_SCHED, &n->state);
}

static inline void napi_schedule(struct napi_struct *n)
{
	if (napi_schedule_prep(n))
		__napi_schedule(n);
}

static inline bool napi_reschedule(struct napi_struct *napi)
{
	if (napi_schedule_prep(napi)) {
		__napi_schedule(napi);
		return true;
	}
	return false;
}

extern void __napi_complete(struct napi_struct *n);
extern void napi_complete(struct napi_struct *n);

static inline void napi_disable(struct napi_struct *n)
{
	set_bit(NAPI_STATE_DISABLE, &n->state);
	while (test_and_set_bit(NAPI_STATE_SCHED, &n->state))
		msleep(1);
	clear_bit(NAPI_STATE_DISABLE, &n->state);
}

static inline void napi_enable(struct napi_struct *n)
{
	BUG_ON(!test_bit(NAPI_STATE_SCHED, &n->state));
	smp_mb__before_clear_bit();
	clear_bit(NAPI_STATE_SCHED, &n->state);
}

#ifdef CONFIG_SMP
static inline void napi_synchronize(const struct napi_struct *n)
{
	while (test_bit(NAPI_STATE_SCHED, &n->state))
		msleep(1);
}
#else
# define napi_synchronize(n)	barrier()
#endif

enum netdev_queue_state_t {
	__QUEUE_STATE_DRV_XOFF,
	__QUEUE_STATE_STACK_XOFF,
	__QUEUE_STATE_FROZEN,
#define QUEUE_STATE_ANY_XOFF ((1 << __QUEUE_STATE_DRV_XOFF)		| \
			      (1 << __QUEUE_STATE_STACK_XOFF))
#define QUEUE_STATE_ANY_XOFF_OR_FROZEN (QUEUE_STATE_ANY_XOFF		| \
					(1 << __QUEUE_STATE_FROZEN))
};

struct netdev_queue {
	struct net_device	*dev;
	struct Qdisc		*qdisc;
	struct Qdisc		*qdisc_sleeping;
#ifdef CONFIG_SYSFS
	struct kobject		kobj;
#endif
#if defined(CONFIG_XPS) && defined(CONFIG_NUMA)
	int			numa_node;
#endif
	spinlock_t		_xmit_lock ____cacheline_aligned_in_smp;
	int			xmit_lock_owner;
	unsigned long		trans_start;

	unsigned long		trans_timeout;

	unsigned long		state;

#ifdef CONFIG_BQL
	struct dql		dql;
#endif
} ____cacheline_aligned_in_smp;

static inline int netdev_queue_numa_node_read(const struct netdev_queue *q)
{
#if defined(CONFIG_XPS) && defined(CONFIG_NUMA)
	return q->numa_node;
#else
	return NUMA_NO_NODE;
#endif
}

static inline void netdev_queue_numa_node_write(struct netdev_queue *q, int node)
{
#if defined(CONFIG_XPS) && defined(CONFIG_NUMA)
	q->numa_node = node;
#endif
}

#ifdef CONFIG_RPS
struct rps_map {
	unsigned int len;
	struct rcu_head rcu;
	u16 cpus[0];
};
#define RPS_MAP_SIZE(_num) (sizeof(struct rps_map) + ((_num) * sizeof(u16)))

struct rps_dev_flow {
	u16 cpu;
	u16 filter;
	unsigned int last_qtail;
};
#define RPS_NO_FILTER 0xffff

struct rps_dev_flow_table {
	unsigned int mask;
	struct rcu_head rcu;
	struct work_struct free_work;
	struct rps_dev_flow flows[0];
};
#define RPS_DEV_FLOW_TABLE_SIZE(_num) (sizeof(struct rps_dev_flow_table) + \
    ((_num) * sizeof(struct rps_dev_flow)))

struct rps_sock_flow_table {
	unsigned int mask;
	u16 ents[0];
};
#define	RPS_SOCK_FLOW_TABLE_SIZE(_num) (sizeof(struct rps_sock_flow_table) + \
    ((_num) * sizeof(u16)))

#define RPS_NO_CPU 0xffff

static inline void rps_record_sock_flow(struct rps_sock_flow_table *table,
					u32 hash)
{
	if (table && hash) {
		unsigned int cpu, index = hash & table->mask;

		
		cpu = raw_smp_processor_id();

		if (table->ents[index] != cpu)
			table->ents[index] = cpu;
	}
}

static inline void rps_reset_sock_flow(struct rps_sock_flow_table *table,
				       u32 hash)
{
	if (table && hash)
		table->ents[hash & table->mask] = RPS_NO_CPU;
}

extern struct rps_sock_flow_table __rcu *rps_sock_flow_table;

#ifdef CONFIG_RFS_ACCEL
extern bool rps_may_expire_flow(struct net_device *dev, u16 rxq_index,
				u32 flow_id, u16 filter_id);
#endif

struct netdev_rx_queue {
	struct rps_map __rcu		*rps_map;
	struct rps_dev_flow_table __rcu	*rps_flow_table;
	struct kobject			kobj;
	struct net_device		*dev;
} ____cacheline_aligned_in_smp;
#endif 

#ifdef CONFIG_XPS
struct xps_map {
	unsigned int len;
	unsigned int alloc_len;
	struct rcu_head rcu;
	u16 queues[0];
};
#define XPS_MAP_SIZE(_num) (sizeof(struct xps_map) + ((_num) * sizeof(u16)))
#define XPS_MIN_MAP_ALLOC ((L1_CACHE_BYTES - sizeof(struct xps_map))	\
    / sizeof(u16))

struct xps_dev_maps {
	struct rcu_head rcu;
	struct xps_map __rcu *cpu_map[0];
};
#define XPS_DEV_MAPS_SIZE (sizeof(struct xps_dev_maps) +		\
    (nr_cpu_ids * sizeof(struct xps_map *)))
#endif 

#define TC_MAX_QUEUE	16
#define TC_BITMASK	15
struct netdev_tc_txq {
	u16 count;
	u16 offset;
};

#if defined(CONFIG_FCOE) || defined(CONFIG_FCOE_MODULE)
struct netdev_fcoe_hbainfo {
	char	manufacturer[64];
	char	serial_number[64];
	char	hardware_version[64];
	char	driver_version[64];
	char	optionrom_version[64];
	char	firmware_version[64];
	char	model[256];
	char	model_description[256];
};
#endif

/*
 * This structure defines the management hooks for network devices.
 * The following hooks can be defined; unless noted otherwise, they are
 * optional and can be filled with a null pointer.
 *
 * int (*ndo_init)(struct net_device *dev);
 *     This function is called once when network device is registered.
 *     The network device can use this to any late stage initializaton
 *     or semantic validattion. It can fail with an error code which will
 *     be propogated back to register_netdev
 *
 * void (*ndo_uninit)(struct net_device *dev);
 *     This function is called when device is unregistered or when registration
 *     fails. It is not called if init fails.
 *
 * int (*ndo_open)(struct net_device *dev);
 *     This function is called when network device transistions to the up
 *     state.
 *
 * int (*ndo_stop)(struct net_device *dev);
 *     This function is called when network device transistions to the down
 *     state.
 *
 * netdev_tx_t (*ndo_start_xmit)(struct sk_buff *skb,
 *                               struct net_device *dev);
 *	Called when a packet needs to be transmitted.
 *	Must return NETDEV_TX_OK , NETDEV_TX_BUSY.
 *        (can also return NETDEV_TX_LOCKED iff NETIF_F_LLTX)
 *	Required can not be NULL.
 *
 * u16 (*ndo_select_queue)(struct net_device *dev, struct sk_buff *skb);
 *	Called to decide which queue to when device supports multiple
 *	transmit queues.
 *
 * void (*ndo_change_rx_flags)(struct net_device *dev, int flags);
 *	This function is called to allow device receiver to make
 *	changes to configuration when multicast or promiscious is enabled.
 *
 * void (*ndo_set_rx_mode)(struct net_device *dev);
 *	This function is called device changes address list filtering.
 *	If driver handles unicast address filtering, it should set
 *	IFF_UNICAST_FLT to its priv_flags.
 *
 * int (*ndo_set_mac_address)(struct net_device *dev, void *addr);
 *	This function  is called when the Media Access Control address
 *	needs to be changed. If this interface is not defined, the
 *	mac address can not be changed.
 *
 * int (*ndo_validate_addr)(struct net_device *dev);
 *	Test if Media Access Control address is valid for the device.
 *
 * int (*ndo_do_ioctl)(struct net_device *dev, struct ifreq *ifr, int cmd);
 *	Called when a user request an ioctl which can't be handled by
 *	the generic interface code. If not defined ioctl's return
 *	not supported error code.
 *
 * int (*ndo_set_config)(struct net_device *dev, struct ifmap *map);
 *	Used to set network devices bus interface parameters. This interface
 *	is retained for legacy reason, new devices should use the bus
 *	interface (PCI) for low level management.
 *
 * int (*ndo_change_mtu)(struct net_device *dev, int new_mtu);
 *	Called when a user wants to change the Maximum Transfer Unit
 *	of a device. If not defined, any request to change MTU will
 *	will return an error.
 *
 * void (*ndo_tx_timeout)(struct net_device *dev);
 *	Callback uses when the transmitter has not made any progress
 *	for dev->watchdog ticks.
 *
 * struct rtnl_link_stats64* (*ndo_get_stats64)(struct net_device *dev,
 *                      struct rtnl_link_stats64 *storage);
 * struct net_device_stats* (*ndo_get_stats)(struct net_device *dev);
 *	Called when a user wants to get the network device usage
 *	statistics. Drivers must do one of the following:
 *	1. Define @ndo_get_stats64 to fill in a zero-initialised
 *	   rtnl_link_stats64 structure passed by the caller.
 *	2. Define @ndo_get_stats to update a net_device_stats structure
 *	   (which should normally be dev->stats) and return a pointer to
 *	   it. The structure may be changed asynchronously only if each
 *	   field is written atomically.
 *	3. Update dev->stats asynchronously and atomically, and define
 *	   neither operation.
 *
 * int (*ndo_vlan_rx_add_vid)(struct net_device *dev, unsigned short vid);
 *	If device support VLAN filtering (dev->features & NETIF_F_HW_VLAN_FILTER)
 *	this function is called when a VLAN id is registered.
 *
 * int (*ndo_vlan_rx_kill_vid)(struct net_device *dev, unsigned short vid);
 *	If device support VLAN filtering (dev->features & NETIF_F_HW_VLAN_FILTER)
 *	this function is called when a VLAN id is unregistered.
 *
 * void (*ndo_poll_controller)(struct net_device *dev);
 *
 *	SR-IOV management functions.
 * int (*ndo_set_vf_mac)(struct net_device *dev, int vf, u8* mac);
 * int (*ndo_set_vf_vlan)(struct net_device *dev, int vf, u16 vlan, u8 qos);
 * int (*ndo_set_vf_tx_rate)(struct net_device *dev, int vf, int rate);
 * int (*ndo_set_vf_spoofchk)(struct net_device *dev, int vf, bool setting);
 * int (*ndo_get_vf_config)(struct net_device *dev,
 *			    int vf, struct ifla_vf_info *ivf);
 * int (*ndo_set_vf_port)(struct net_device *dev, int vf,
 *			  struct nlattr *port[]);
 * int (*ndo_get_vf_port)(struct net_device *dev, int vf, struct sk_buff *skb);
 * int (*ndo_setup_tc)(struct net_device *dev, u8 tc)
 * 	Called to setup 'tc' number of traffic classes in the net device. This
 * 	is always called from the stack with the rtnl lock held and netif tx
 * 	queues stopped. This allows the netdevice to perform queue management
 * 	safely.
 *
 *	Fiber Channel over Ethernet (FCoE) offload functions.
 * int (*ndo_fcoe_enable)(struct net_device *dev);
 *	Called when the FCoE protocol stack wants to start using LLD for FCoE
 *	so the underlying device can perform whatever needed configuration or
 *	initialization to support acceleration of FCoE traffic.
 *
 * int (*ndo_fcoe_disable)(struct net_device *dev);
 *	Called when the FCoE protocol stack wants to stop using LLD for FCoE
 *	so the underlying device can perform whatever needed clean-ups to
 *	stop supporting acceleration of FCoE traffic.
 *
 * int (*ndo_fcoe_ddp_setup)(struct net_device *dev, u16 xid,
 *			     struct scatterlist *sgl, unsigned int sgc);
 *	Called when the FCoE Initiator wants to initialize an I/O that
 *	is a possible candidate for Direct Data Placement (DDP). The LLD can
 *	perform necessary setup and returns 1 to indicate the device is set up
 *	successfully to perform DDP on this I/O, otherwise this returns 0.
 *
 * int (*ndo_fcoe_ddp_done)(struct net_device *dev,  u16 xid);
 *	Called when the FCoE Initiator/Target is done with the DDPed I/O as
 *	indicated by the FC exchange id 'xid', so the underlying device can
 *	clean up and reuse resources for later DDP requests.
 *
 * int (*ndo_fcoe_ddp_target)(struct net_device *dev, u16 xid,
 *			      struct scatterlist *sgl, unsigned int sgc);
 *	Called when the FCoE Target wants to initialize an I/O that
 *	is a possible candidate for Direct Data Placement (DDP). The LLD can
 *	perform necessary setup and returns 1 to indicate the device is set up
 *	successfully to perform DDP on this I/O, otherwise this returns 0.
 *
 * int (*ndo_fcoe_get_hbainfo)(struct net_device *dev,
 *			       struct netdev_fcoe_hbainfo *hbainfo);
 *	Called when the FCoE Protocol stack wants information on the underlying
 *	device. This information is utilized by the FCoE protocol stack to
 *	register attributes with Fiber Channel management service as per the
 *	FC-GS Fabric Device Management Information(FDMI) specification.
 *
 * int (*ndo_fcoe_get_wwn)(struct net_device *dev, u64 *wwn, int type);
 *	Called when the underlying device wants to override default World Wide
 *	Name (WWN) generation mechanism in FCoE protocol stack to pass its own
 *	World Wide Port Name (WWPN) or World Wide Node Name (WWNN) to the FCoE
 *	protocol stack to use.
 *
 *	RFS acceleration.
 * int (*ndo_rx_flow_steer)(struct net_device *dev, const struct sk_buff *skb,
 *			    u16 rxq_index, u32 flow_id);
 *	Set hardware filter for RFS.  rxq_index is the target queue index;
 *	flow_id is a flow ID to be passed to rps_may_expire_flow() later.
 *	Return the filter ID on success, or a negative error code.
 *
 *	Slave management functions (for bridge, bonding, etc). User should
 *	call netdev_set_master() to set dev->master properly.
 * int (*ndo_add_slave)(struct net_device *dev, struct net_device *slave_dev);
 *	Called to make another netdev an underling.
 *
 * int (*ndo_del_slave)(struct net_device *dev, struct net_device *slave_dev);
 *	Called to release previously enslaved netdev.
 *
 *      Feature/offload setting functions.
 * netdev_features_t (*ndo_fix_features)(struct net_device *dev,
 *		netdev_features_t features);
 *	Adjusts the requested feature flags according to device-specific
 *	constraints, and returns the resulting flags. Must not modify
 *	the device state.
 *
 * int (*ndo_set_features)(struct net_device *dev, netdev_features_t features);
 *	Called to update device configuration to new features. Passed
 *	feature set might be less than what was returned by ndo_fix_features()).
 *	Must return >0 or -errno if it changed dev->features itself.
 *
 */
struct net_device_ops {
	int			(*ndo_init)(struct net_device *dev);
	void			(*ndo_uninit)(struct net_device *dev);
	int			(*ndo_open)(struct net_device *dev);
	int			(*ndo_stop)(struct net_device *dev);
	netdev_tx_t		(*ndo_start_xmit) (struct sk_buff *skb,
						   struct net_device *dev);
	u16			(*ndo_select_queue)(struct net_device *dev,
						    struct sk_buff *skb);
	void			(*ndo_change_rx_flags)(struct net_device *dev,
						       int flags);
	void			(*ndo_set_rx_mode)(struct net_device *dev);
	int			(*ndo_set_mac_address)(struct net_device *dev,
						       void *addr);
	int			(*ndo_validate_addr)(struct net_device *dev);
	int			(*ndo_do_ioctl)(struct net_device *dev,
					        struct ifreq *ifr, int cmd);
	int			(*ndo_set_config)(struct net_device *dev,
					          struct ifmap *map);
	int			(*ndo_change_mtu)(struct net_device *dev,
						  int new_mtu);
	int			(*ndo_neigh_setup)(struct net_device *dev,
						   struct neigh_parms *);
	void			(*ndo_tx_timeout) (struct net_device *dev);

	struct rtnl_link_stats64* (*ndo_get_stats64)(struct net_device *dev,
						     struct rtnl_link_stats64 *storage);
	struct net_device_stats* (*ndo_get_stats)(struct net_device *dev);

	int			(*ndo_vlan_rx_add_vid)(struct net_device *dev,
						       unsigned short vid);
	int			(*ndo_vlan_rx_kill_vid)(struct net_device *dev,
						        unsigned short vid);
#ifdef CONFIG_NET_POLL_CONTROLLER
	void                    (*ndo_poll_controller)(struct net_device *dev);
	int			(*ndo_netpoll_setup)(struct net_device *dev,
						     struct netpoll_info *info);
	void			(*ndo_netpoll_cleanup)(struct net_device *dev);
#endif
	int			(*ndo_set_vf_mac)(struct net_device *dev,
						  int queue, u8 *mac);
	int			(*ndo_set_vf_vlan)(struct net_device *dev,
						   int queue, u16 vlan, u8 qos);
	int			(*ndo_set_vf_tx_rate)(struct net_device *dev,
						      int vf, int rate);
	int			(*ndo_set_vf_spoofchk)(struct net_device *dev,
						       int vf, bool setting);
	int			(*ndo_get_vf_config)(struct net_device *dev,
						     int vf,
						     struct ifla_vf_info *ivf);
	int			(*ndo_set_vf_port)(struct net_device *dev,
						   int vf,
						   struct nlattr *port[]);
	int			(*ndo_get_vf_port)(struct net_device *dev,
						   int vf, struct sk_buff *skb);
	int			(*ndo_setup_tc)(struct net_device *dev, u8 tc);
#if IS_ENABLED(CONFIG_FCOE)
	int			(*ndo_fcoe_enable)(struct net_device *dev);
	int			(*ndo_fcoe_disable)(struct net_device *dev);
	int			(*ndo_fcoe_ddp_setup)(struct net_device *dev,
						      u16 xid,
						      struct scatterlist *sgl,
						      unsigned int sgc);
	int			(*ndo_fcoe_ddp_done)(struct net_device *dev,
						     u16 xid);
	int			(*ndo_fcoe_ddp_target)(struct net_device *dev,
						       u16 xid,
						       struct scatterlist *sgl,
						       unsigned int sgc);
	int			(*ndo_fcoe_get_hbainfo)(struct net_device *dev,
							struct netdev_fcoe_hbainfo *hbainfo);
#endif

#if IS_ENABLED(CONFIG_LIBFCOE)
#define NETDEV_FCOE_WWNN 0
#define NETDEV_FCOE_WWPN 1
	int			(*ndo_fcoe_get_wwn)(struct net_device *dev,
						    u64 *wwn, int type);
#endif

#ifdef CONFIG_RFS_ACCEL
	int			(*ndo_rx_flow_steer)(struct net_device *dev,
						     const struct sk_buff *skb,
						     u16 rxq_index,
						     u32 flow_id);
#endif
	int			(*ndo_add_slave)(struct net_device *dev,
						 struct net_device *slave_dev);
	int			(*ndo_del_slave)(struct net_device *dev,
						 struct net_device *slave_dev);
	netdev_features_t	(*ndo_fix_features)(struct net_device *dev,
						    netdev_features_t features);
	int			(*ndo_set_features)(struct net_device *dev,
						    netdev_features_t features);
	int			(*ndo_neigh_construct)(struct neighbour *n);
	void			(*ndo_neigh_destroy)(struct neighbour *n);
};


struct net_device {

	char			name[IFNAMSIZ];

	struct pm_qos_request	pm_qos_req;

	
	struct hlist_node	name_hlist;
	
	char 			*ifalias;

	unsigned long		mem_end;	
	unsigned long		mem_start;	
	unsigned long		base_addr;	
	unsigned int		irq;		


	unsigned long		state;

	struct list_head	dev_list;
	struct list_head	napi_list;
	struct list_head	unreg_list;

	
	netdev_features_t	features;
	
	netdev_features_t	hw_features;
	
	netdev_features_t	wanted_features;
	
	netdev_features_t	vlan_features;

	
	int			ifindex;
	int			iflink;

	struct net_device_stats	stats;
	atomic_long_t		rx_dropped; 

#ifdef CONFIG_WIRELESS_EXT
	const struct iw_handler_def *	wireless_handlers;
	
	struct iw_public_data *	wireless_data;
#endif
	
	const struct net_device_ops *netdev_ops;
	const struct ethtool_ops *ethtool_ops;

	
	const struct header_ops *header_ops;

	unsigned int		flags;	
	unsigned int		priv_flags; 
	unsigned short		gflags;
	unsigned short		padded;	

	unsigned char		operstate; 
	unsigned char		link_mode; 

	unsigned char		if_port;	
	unsigned char		dma;		

	unsigned int		mtu;	
	unsigned short		type;	
	unsigned short		hard_header_len;	

	unsigned short		needed_headroom;
	unsigned short		needed_tailroom;

	
	unsigned char		perm_addr[MAX_ADDR_LEN]; 
	unsigned char		addr_assign_type; 
	unsigned char		addr_len;	
	unsigned char		neigh_priv_len;
	unsigned short          dev_id;		

	spinlock_t		addr_list_lock;
	struct netdev_hw_addr_list	uc;	
	struct netdev_hw_addr_list	mc;	
	bool			uc_promisc;
	unsigned int		promiscuity;
	unsigned int		allmulti;


	

#if IS_ENABLED(CONFIG_VLAN_8021Q)
	struct vlan_info __rcu	*vlan_info;	
#endif
#if IS_ENABLED(CONFIG_NET_DSA)
	struct dsa_switch_tree	*dsa_ptr;	
#endif
	void 			*atalk_ptr;	
	struct in_device __rcu	*ip_ptr;	
	struct dn_dev __rcu     *dn_ptr;        
	struct inet6_dev __rcu	*ip6_ptr;       
	void			*ec_ptr;	
	void			*ax25_ptr;	
	struct wireless_dev	*ieee80211_ptr;	

	unsigned long		last_rx;	

	struct net_device	*master; 

	
	unsigned char		*dev_addr;	

	struct netdev_hw_addr_list	dev_addrs; 

	unsigned char		broadcast[MAX_ADDR_LEN];	

#ifdef CONFIG_SYSFS
	struct kset		*queues_kset;
#endif

#ifdef CONFIG_RPS
	struct netdev_rx_queue	*_rx;

	
	unsigned int		num_rx_queues;

	
	unsigned int		real_num_rx_queues;

#ifdef CONFIG_RFS_ACCEL
	struct cpu_rmap		*rx_cpu_rmap;
#endif
#endif

	rx_handler_func_t __rcu	*rx_handler;
	void __rcu		*rx_handler_data;

	struct netdev_queue __rcu *ingress_queue;

	struct netdev_queue	*_tx ____cacheline_aligned_in_smp;

	
	unsigned int		num_tx_queues;

	
	unsigned int		real_num_tx_queues;

	
	struct Qdisc		*qdisc;

	unsigned long		tx_queue_len;	
	spinlock_t		tx_global_lock;

#ifdef CONFIG_XPS
	struct xps_dev_maps __rcu *xps_maps;
#endif

	

	unsigned long		trans_start;	

	int			watchdog_timeo; 
	struct timer_list	watchdog_timer;

	
	int __percpu		*pcpu_refcnt;

	
	struct list_head	todo_list;
	
	struct hlist_node	index_hlist;

	struct list_head	link_watch_list;

	
	enum { NETREG_UNINITIALIZED=0,
	       NETREG_REGISTERED,	
	       NETREG_UNREGISTERING,	
	       NETREG_UNREGISTERED,	
	       NETREG_RELEASED,		
	       NETREG_DUMMY,		
	} reg_state:8;

	bool dismantle; 

	enum {
		RTNL_LINK_INITIALIZED,
		RTNL_LINK_INITIALIZING,
	} rtnl_link_state:16;

	
	void (*destructor)(struct net_device *dev);

#ifdef CONFIG_NETPOLL
	struct netpoll_info	*npinfo;
#endif

#ifdef CONFIG_NET_NS
	
	struct net		*nd_net;
#endif

	
	union {
		void				*ml_priv;
		struct pcpu_lstats __percpu	*lstats; 
		struct pcpu_tstats __percpu	*tstats; 
		struct pcpu_dstats __percpu	*dstats; 
	};
	
	struct garp_port __rcu	*garp_port;

	
	struct device		dev;
	
	const struct attribute_group *sysfs_groups[4];

	
	const struct rtnl_link_ops *rtnl_link_ops;

	
#define GSO_MAX_SIZE		65536
	unsigned int		gso_max_size;

#ifdef CONFIG_DCB
	
	const struct dcbnl_rtnl_ops *dcbnl_ops;
#endif
	u8 num_tc;
	struct netdev_tc_txq tc_to_txq[TC_MAX_QUEUE];
	u8 prio_tc_map[TC_BITMASK + 1];

#if IS_ENABLED(CONFIG_FCOE)
	
	unsigned int		fcoe_ddp_xid;
#endif
#if IS_ENABLED(CONFIG_NETPRIO_CGROUP)
	struct netprio_map __rcu *priomap;
#endif
	
	struct phy_device *phydev;

	
	int group;
};
#define to_net_dev(d) container_of(d, struct net_device, dev)

#define	NETDEV_ALIGN		32

static inline
int netdev_get_prio_tc_map(const struct net_device *dev, u32 prio)
{
	return dev->prio_tc_map[prio & TC_BITMASK];
}

static inline
int netdev_set_prio_tc_map(struct net_device *dev, u8 prio, u8 tc)
{
	if (tc >= dev->num_tc)
		return -EINVAL;

	dev->prio_tc_map[prio & TC_BITMASK] = tc & TC_BITMASK;
	return 0;
}

static inline
void netdev_reset_tc(struct net_device *dev)
{
	dev->num_tc = 0;
	memset(dev->tc_to_txq, 0, sizeof(dev->tc_to_txq));
	memset(dev->prio_tc_map, 0, sizeof(dev->prio_tc_map));
}

static inline
int netdev_set_tc_queue(struct net_device *dev, u8 tc, u16 count, u16 offset)
{
	if (tc >= dev->num_tc)
		return -EINVAL;

	dev->tc_to_txq[tc].count = count;
	dev->tc_to_txq[tc].offset = offset;
	return 0;
}

static inline
int netdev_set_num_tc(struct net_device *dev, u8 num_tc)
{
	if (num_tc > TC_MAX_QUEUE)
		return -EINVAL;

	dev->num_tc = num_tc;
	return 0;
}

static inline
int netdev_get_num_tc(struct net_device *dev)
{
	return dev->num_tc;
}

static inline
struct netdev_queue *netdev_get_tx_queue(const struct net_device *dev,
					 unsigned int index)
{
	return &dev->_tx[index];
}

static inline void netdev_for_each_tx_queue(struct net_device *dev,
					    void (*f)(struct net_device *,
						      struct netdev_queue *,
						      void *),
					    void *arg)
{
	unsigned int i;

	for (i = 0; i < dev->num_tx_queues; i++)
		f(dev, &dev->_tx[i], arg);
}

static inline
struct net *dev_net(const struct net_device *dev)
{
	return read_pnet(&dev->nd_net);
}

static inline
void dev_net_set(struct net_device *dev, struct net *net)
{
#ifdef CONFIG_NET_NS
	release_net(dev->nd_net);
	dev->nd_net = hold_net(net);
#endif
}

static inline bool netdev_uses_dsa_tags(struct net_device *dev)
{
#ifdef CONFIG_NET_DSA_TAG_DSA
	if (dev->dsa_ptr != NULL)
		return dsa_uses_dsa_tags(dev->dsa_ptr);
#endif

	return 0;
}

static inline bool netdev_uses_trailer_tags(struct net_device *dev)
{
#ifdef CONFIG_NET_DSA_TAG_TRAILER
	if (dev->dsa_ptr != NULL)
		return dsa_uses_trailer_tags(dev->dsa_ptr);
#endif

	return 0;
}

static inline void *netdev_priv(const struct net_device *dev)
{
	return (char *)dev + ALIGN(sizeof(struct net_device), NETDEV_ALIGN);
}

#define SET_NETDEV_DEV(net, pdev)	((net)->dev.parent = (pdev))

#define SET_NETDEV_DEVTYPE(net, devtype)	((net)->dev.type = (devtype))

void netif_napi_add(struct net_device *dev, struct napi_struct *napi,
		    int (*poll)(struct napi_struct *, int), int weight);

void netif_napi_del(struct napi_struct *napi);

struct napi_gro_cb {
	
	void *frag0;

	
	unsigned int frag0_len;

	
	int data_offset;

	
	int same_flow;

	
	int flush;

	
	int count;

	
	int free;
};

#define NAPI_GRO_CB(skb) ((struct napi_gro_cb *)(skb)->cb)

struct packet_type {
	__be16			type;	
	struct net_device	*dev;	
	int			(*func) (struct sk_buff *,
					 struct net_device *,
					 struct packet_type *,
					 struct net_device *);
	struct sk_buff		*(*gso_segment)(struct sk_buff *skb,
						netdev_features_t features);
	int			(*gso_send_check)(struct sk_buff *skb);
	struct sk_buff		**(*gro_receive)(struct sk_buff **head,
					       struct sk_buff *skb);
	int			(*gro_complete)(struct sk_buff *skb);
	void			*af_packet_priv;
	struct list_head	list;
};

#include <linux/notifier.h>

#define NETDEV_UP	0x0001	
#define NETDEV_DOWN	0x0002
#define NETDEV_REBOOT	0x0003	
#define NETDEV_CHANGE	0x0004	
#define NETDEV_REGISTER 0x0005
#define NETDEV_UNREGISTER	0x0006
#define NETDEV_CHANGEMTU	0x0007
#define NETDEV_CHANGEADDR	0x0008
#define NETDEV_GOING_DOWN	0x0009
#define NETDEV_CHANGENAME	0x000A
#define NETDEV_FEAT_CHANGE	0x000B
#define NETDEV_BONDING_FAILOVER 0x000C
#define NETDEV_PRE_UP		0x000D
#define NETDEV_PRE_TYPE_CHANGE	0x000E
#define NETDEV_POST_TYPE_CHANGE	0x000F
#define NETDEV_POST_INIT	0x0010
#define NETDEV_UNREGISTER_BATCH 0x0011
#define NETDEV_RELEASE		0x0012
#define NETDEV_NOTIFY_PEERS	0x0013
#define NETDEV_JOIN		0x0014

extern int register_netdevice_notifier(struct notifier_block *nb);
extern int unregister_netdevice_notifier(struct notifier_block *nb);
extern int call_netdevice_notifiers(unsigned long val, struct net_device *dev);


extern rwlock_t				dev_base_lock;		


#define for_each_netdev(net, d)		\
		list_for_each_entry(d, &(net)->dev_base_head, dev_list)
#define for_each_netdev_reverse(net, d)	\
		list_for_each_entry_reverse(d, &(net)->dev_base_head, dev_list)
#define for_each_netdev_rcu(net, d)		\
		list_for_each_entry_rcu(d, &(net)->dev_base_head, dev_list)
#define for_each_netdev_safe(net, d, n)	\
		list_for_each_entry_safe(d, n, &(net)->dev_base_head, dev_list)
#define for_each_netdev_continue(net, d)		\
		list_for_each_entry_continue(d, &(net)->dev_base_head, dev_list)
#define for_each_netdev_continue_rcu(net, d)		\
	list_for_each_entry_continue_rcu(d, &(net)->dev_base_head, dev_list)
#define net_device_entry(lh)	list_entry(lh, struct net_device, dev_list)

static inline struct net_device *next_net_device(struct net_device *dev)
{
	struct list_head *lh;
	struct net *net;

	net = dev_net(dev);
	lh = dev->dev_list.next;
	return lh == &net->dev_base_head ? NULL : net_device_entry(lh);
}

static inline struct net_device *next_net_device_rcu(struct net_device *dev)
{
	struct list_head *lh;
	struct net *net;

	net = dev_net(dev);
	lh = rcu_dereference(list_next_rcu(&dev->dev_list));
	return lh == &net->dev_base_head ? NULL : net_device_entry(lh);
}

static inline struct net_device *first_net_device(struct net *net)
{
	return list_empty(&net->dev_base_head) ? NULL :
		net_device_entry(net->dev_base_head.next);
}

static inline struct net_device *first_net_device_rcu(struct net *net)
{
	struct list_head *lh = rcu_dereference(list_next_rcu(&net->dev_base_head));

	return lh == &net->dev_base_head ? NULL : net_device_entry(lh);
}

extern int 			netdev_boot_setup_check(struct net_device *dev);
extern unsigned long		netdev_boot_base(const char *prefix, int unit);
extern struct net_device *dev_getbyhwaddr_rcu(struct net *net, unsigned short type,
					      const char *hwaddr);
extern struct net_device *dev_getfirstbyhwtype(struct net *net, unsigned short type);
extern struct net_device *__dev_getfirstbyhwtype(struct net *net, unsigned short type);
extern void		dev_add_pack(struct packet_type *pt);
extern void		dev_remove_pack(struct packet_type *pt);
extern void		__dev_remove_pack(struct packet_type *pt);

extern struct net_device	*dev_get_by_flags_rcu(struct net *net, unsigned short flags,
						      unsigned short mask);
extern struct net_device	*dev_get_by_name(struct net *net, const char *name);
extern struct net_device	*dev_get_by_name_rcu(struct net *net, const char *name);
extern struct net_device	*__dev_get_by_name(struct net *net, const char *name);
extern int		dev_alloc_name(struct net_device *dev, const char *name);
extern int		dev_open(struct net_device *dev);
extern int		dev_close(struct net_device *dev);
extern void		dev_disable_lro(struct net_device *dev);
extern int		dev_queue_xmit(struct sk_buff *skb);
extern int		register_netdevice(struct net_device *dev);
extern void		unregister_netdevice_queue(struct net_device *dev,
						   struct list_head *head);
extern void		unregister_netdevice_many(struct list_head *head);
static inline void unregister_netdevice(struct net_device *dev)
{
	unregister_netdevice_queue(dev, NULL);
}

extern int 		netdev_refcnt_read(const struct net_device *dev);
extern void		free_netdev(struct net_device *dev);
extern void		synchronize_net(void);
extern int		init_dummy_netdev(struct net_device *dev);
extern void		netdev_resync_ops(struct net_device *dev);

extern struct net_device	*dev_get_by_index(struct net *net, int ifindex);
extern struct net_device	*__dev_get_by_index(struct net *net, int ifindex);
extern struct net_device	*dev_get_by_index_rcu(struct net *net, int ifindex);
extern int		dev_restart(struct net_device *dev);
#ifdef CONFIG_NETPOLL_TRAP
extern int		netpoll_trap(void);
#endif
extern int	       skb_gro_receive(struct sk_buff **head,
				       struct sk_buff *skb);
extern void	       skb_gro_reset_offset(struct sk_buff *skb);

static inline unsigned int skb_gro_offset(const struct sk_buff *skb)
{
	return NAPI_GRO_CB(skb)->data_offset;
}

static inline unsigned int skb_gro_len(const struct sk_buff *skb)
{
	return skb->len - NAPI_GRO_CB(skb)->data_offset;
}

static inline void skb_gro_pull(struct sk_buff *skb, unsigned int len)
{
	NAPI_GRO_CB(skb)->data_offset += len;
}

static inline void *skb_gro_header_fast(struct sk_buff *skb,
					unsigned int offset)
{
	return NAPI_GRO_CB(skb)->frag0 + offset;
}

static inline int skb_gro_header_hard(struct sk_buff *skb, unsigned int hlen)
{
	return NAPI_GRO_CB(skb)->frag0_len < hlen;
}

static inline void *skb_gro_header_slow(struct sk_buff *skb, unsigned int hlen,
					unsigned int offset)
{
	if (!pskb_may_pull(skb, hlen))
		return NULL;

	NAPI_GRO_CB(skb)->frag0 = NULL;
	NAPI_GRO_CB(skb)->frag0_len = 0;
	return skb->data + offset;
}

static inline void *skb_gro_mac_header(struct sk_buff *skb)
{
	return NAPI_GRO_CB(skb)->frag0 ?: skb_mac_header(skb);
}

static inline void *skb_gro_network_header(struct sk_buff *skb)
{
	return (NAPI_GRO_CB(skb)->frag0 ?: skb->data) +
	       skb_network_offset(skb);
}

static inline int dev_hard_header(struct sk_buff *skb, struct net_device *dev,
				  unsigned short type,
				  const void *daddr, const void *saddr,
				  unsigned len)
{
	if (!dev->header_ops || !dev->header_ops->create)
		return 0;

	return dev->header_ops->create(skb, dev, type, daddr, saddr, len);
}

static inline int dev_parse_header(const struct sk_buff *skb,
				   unsigned char *haddr)
{
	const struct net_device *dev = skb->dev;

	if (!dev->header_ops || !dev->header_ops->parse)
		return 0;
	return dev->header_ops->parse(skb, haddr);
}

typedef int gifconf_func_t(struct net_device * dev, char __user * bufptr, int len);
extern int		register_gifconf(unsigned int family, gifconf_func_t * gifconf);
static inline int unregister_gifconf(unsigned int family)
{
	return register_gifconf(family, NULL);
}

struct softnet_data {
	struct Qdisc		*output_queue;
	struct Qdisc		**output_queue_tailp;
	struct list_head	poll_list;
	struct sk_buff		*completion_queue;
	struct sk_buff_head	process_queue;

	
	unsigned int		processed;
	unsigned int		time_squeeze;
	unsigned int		cpu_collision;
	unsigned int		received_rps;

#ifdef CONFIG_RPS
	struct softnet_data	*rps_ipi_list;

	
	struct call_single_data	csd ____cacheline_aligned_in_smp;
	struct softnet_data	*rps_ipi_next;
	unsigned int		cpu;
	unsigned int		input_queue_head;
	unsigned int		input_queue_tail;
#endif
	unsigned		dropped;
	struct sk_buff_head	input_pkt_queue;
	struct napi_struct	backlog;
};

static inline void input_queue_head_incr(struct softnet_data *sd)
{
#ifdef CONFIG_RPS
	sd->input_queue_head++;
#endif
}

static inline void input_queue_tail_incr_save(struct softnet_data *sd,
					      unsigned int *qtail)
{
#ifdef CONFIG_RPS
	*qtail = ++sd->input_queue_tail;
#endif
}

DECLARE_PER_CPU_ALIGNED(struct softnet_data, softnet_data);

extern void __netif_schedule(struct Qdisc *q);

static inline void netif_schedule_queue(struct netdev_queue *txq)
{
	if (!(txq->state & QUEUE_STATE_ANY_XOFF))
		__netif_schedule(txq->qdisc);
}

static inline void netif_tx_schedule_all(struct net_device *dev)
{
	unsigned int i;

	for (i = 0; i < dev->num_tx_queues; i++)
		netif_schedule_queue(netdev_get_tx_queue(dev, i));
}

static inline void netif_tx_start_queue(struct netdev_queue *dev_queue)
{
	clear_bit(__QUEUE_STATE_DRV_XOFF, &dev_queue->state);
}

static inline void netif_start_queue(struct net_device *dev)
{
	netif_tx_start_queue(netdev_get_tx_queue(dev, 0));
}

static inline void netif_tx_start_all_queues(struct net_device *dev)
{
	unsigned int i;

	for (i = 0; i < dev->num_tx_queues; i++) {
		struct netdev_queue *txq = netdev_get_tx_queue(dev, i);
		netif_tx_start_queue(txq);
	}
}

static inline void netif_tx_wake_queue(struct netdev_queue *dev_queue)
{
#ifdef CONFIG_NETPOLL_TRAP
	if (netpoll_trap()) {
		netif_tx_start_queue(dev_queue);
		return;
	}
#endif
	if (test_and_clear_bit(__QUEUE_STATE_DRV_XOFF, &dev_queue->state))
		__netif_schedule(dev_queue->qdisc);
}

static inline void netif_wake_queue(struct net_device *dev)
{
	netif_tx_wake_queue(netdev_get_tx_queue(dev, 0));
}

static inline void netif_tx_wake_all_queues(struct net_device *dev)
{
	unsigned int i;

	for (i = 0; i < dev->num_tx_queues; i++) {
		struct netdev_queue *txq = netdev_get_tx_queue(dev, i);
		netif_tx_wake_queue(txq);
	}
}

static inline void netif_tx_stop_queue(struct netdev_queue *dev_queue)
{
	if (WARN_ON(!dev_queue)) {
		pr_info("netif_stop_queue() cannot be called before register_netdev()\n");
		return;
	}
	set_bit(__QUEUE_STATE_DRV_XOFF, &dev_queue->state);
}

static inline void netif_stop_queue(struct net_device *dev)
{
	netif_tx_stop_queue(netdev_get_tx_queue(dev, 0));
}

static inline void netif_tx_stop_all_queues(struct net_device *dev)
{
	unsigned int i;

	for (i = 0; i < dev->num_tx_queues; i++) {
		struct netdev_queue *txq = netdev_get_tx_queue(dev, i);
		netif_tx_stop_queue(txq);
	}
}

static inline bool netif_tx_queue_stopped(const struct netdev_queue *dev_queue)
{
	return test_bit(__QUEUE_STATE_DRV_XOFF, &dev_queue->state);
}

static inline bool netif_queue_stopped(const struct net_device *dev)
{
	return netif_tx_queue_stopped(netdev_get_tx_queue(dev, 0));
}

static inline bool netif_xmit_stopped(const struct netdev_queue *dev_queue)
{
	return dev_queue->state & QUEUE_STATE_ANY_XOFF;
}

static inline bool netif_xmit_frozen_or_stopped(const struct netdev_queue *dev_queue)
{
	return dev_queue->state & QUEUE_STATE_ANY_XOFF_OR_FROZEN;
}

static inline void netdev_tx_sent_queue(struct netdev_queue *dev_queue,
					unsigned int bytes)
{
#ifdef CONFIG_BQL
	dql_queued(&dev_queue->dql, bytes);

	if (likely(dql_avail(&dev_queue->dql) >= 0))
		return;

	set_bit(__QUEUE_STATE_STACK_XOFF, &dev_queue->state);

	smp_mb();

	
	if (unlikely(dql_avail(&dev_queue->dql) >= 0))
		clear_bit(__QUEUE_STATE_STACK_XOFF, &dev_queue->state);
#endif
}

static inline void netdev_sent_queue(struct net_device *dev, unsigned int bytes)
{
	netdev_tx_sent_queue(netdev_get_tx_queue(dev, 0), bytes);
}

static inline void netdev_tx_completed_queue(struct netdev_queue *dev_queue,
					     unsigned pkts, unsigned bytes)
{
#ifdef CONFIG_BQL
	if (unlikely(!bytes))
		return;

	dql_completed(&dev_queue->dql, bytes);

	smp_mb();

	if (dql_avail(&dev_queue->dql) < 0)
		return;

	if (test_and_clear_bit(__QUEUE_STATE_STACK_XOFF, &dev_queue->state))
		netif_schedule_queue(dev_queue);
#endif
}

static inline void netdev_completed_queue(struct net_device *dev,
					  unsigned pkts, unsigned bytes)
{
	netdev_tx_completed_queue(netdev_get_tx_queue(dev, 0), pkts, bytes);
}

static inline void netdev_tx_reset_queue(struct netdev_queue *q)
{
#ifdef CONFIG_BQL
	clear_bit(__QUEUE_STATE_STACK_XOFF, &q->state);
	dql_reset(&q->dql);
#endif
}

static inline void netdev_reset_queue(struct net_device *dev_queue)
{
	netdev_tx_reset_queue(netdev_get_tx_queue(dev_queue, 0));
}

static inline bool netif_running(const struct net_device *dev)
{
	return test_bit(__LINK_STATE_START, &dev->state);
}


static inline void netif_start_subqueue(struct net_device *dev, u16 queue_index)
{
	struct netdev_queue *txq = netdev_get_tx_queue(dev, queue_index);

	netif_tx_start_queue(txq);
}

static inline void netif_stop_subqueue(struct net_device *dev, u16 queue_index)
{
	struct netdev_queue *txq = netdev_get_tx_queue(dev, queue_index);
#ifdef CONFIG_NETPOLL_TRAP
	if (netpoll_trap())
		return;
#endif
	netif_tx_stop_queue(txq);
}

static inline bool __netif_subqueue_stopped(const struct net_device *dev,
					    u16 queue_index)
{
	struct netdev_queue *txq = netdev_get_tx_queue(dev, queue_index);

	return netif_tx_queue_stopped(txq);
}

static inline bool netif_subqueue_stopped(const struct net_device *dev,
					  struct sk_buff *skb)
{
	return __netif_subqueue_stopped(dev, skb_get_queue_mapping(skb));
}

static inline void netif_wake_subqueue(struct net_device *dev, u16 queue_index)
{
	struct netdev_queue *txq = netdev_get_tx_queue(dev, queue_index);
#ifdef CONFIG_NETPOLL_TRAP
	if (netpoll_trap())
		return;
#endif
	if (test_and_clear_bit(__QUEUE_STATE_DRV_XOFF, &txq->state))
		__netif_schedule(txq->qdisc);
}

static inline u16 skb_tx_hash(const struct net_device *dev,
			      const struct sk_buff *skb)
{
	return __skb_tx_hash(dev, skb, dev->real_num_tx_queues);
}

static inline bool netif_is_multiqueue(const struct net_device *dev)
{
	return dev->num_tx_queues > 1;
}

extern int netif_set_real_num_tx_queues(struct net_device *dev,
					unsigned int txq);

#ifdef CONFIG_RPS
extern int netif_set_real_num_rx_queues(struct net_device *dev,
					unsigned int rxq);
#else
static inline int netif_set_real_num_rx_queues(struct net_device *dev,
						unsigned int rxq)
{
	return 0;
}
#endif

static inline int netif_copy_real_num_queues(struct net_device *to_dev,
					     const struct net_device *from_dev)
{
	netif_set_real_num_tx_queues(to_dev, from_dev->real_num_tx_queues);
#ifdef CONFIG_RPS
	return netif_set_real_num_rx_queues(to_dev,
					    from_dev->real_num_rx_queues);
#else
	return 0;
#endif
}

extern void dev_kfree_skb_irq(struct sk_buff *skb);

extern void dev_kfree_skb_any(struct sk_buff *skb);

extern int		netif_rx(struct sk_buff *skb);
extern int		netif_rx_ni(struct sk_buff *skb);
extern int		netif_receive_skb(struct sk_buff *skb);
extern gro_result_t	dev_gro_receive(struct napi_struct *napi,
					struct sk_buff *skb);
extern gro_result_t	napi_skb_finish(gro_result_t ret, struct sk_buff *skb);
extern gro_result_t	napi_gro_receive(struct napi_struct *napi,
					 struct sk_buff *skb);
extern void		napi_gro_flush(struct napi_struct *napi);
extern struct sk_buff *	napi_get_frags(struct napi_struct *napi);
extern gro_result_t	napi_frags_finish(struct napi_struct *napi,
					  struct sk_buff *skb,
					  gro_result_t ret);
extern struct sk_buff *	napi_frags_skb(struct napi_struct *napi);
extern gro_result_t	napi_gro_frags(struct napi_struct *napi);

static inline void napi_free_frags(struct napi_struct *napi)
{
	kfree_skb(napi->skb);
	napi->skb = NULL;
}

extern int netdev_rx_handler_register(struct net_device *dev,
				      rx_handler_func_t *rx_handler,
				      void *rx_handler_data);
extern void netdev_rx_handler_unregister(struct net_device *dev);

extern bool		dev_valid_name(const char *name);
extern int		dev_ioctl(struct net *net, unsigned int cmd, void __user *);
extern int		dev_ethtool(struct net *net, struct ifreq *);
extern unsigned		dev_get_flags(const struct net_device *);
extern int		__dev_change_flags(struct net_device *, unsigned int flags);
extern int		dev_change_flags(struct net_device *, unsigned);
extern void		__dev_notify_flags(struct net_device *, unsigned int old_flags);
extern int		dev_change_name(struct net_device *, const char *);
extern int		dev_set_alias(struct net_device *, const char *, size_t);
extern int		dev_change_net_namespace(struct net_device *,
						 struct net *, const char *);
extern int		dev_set_mtu(struct net_device *, int);
extern void		dev_set_group(struct net_device *, int);
extern int		dev_set_mac_address(struct net_device *,
					    struct sockaddr *);
extern int		dev_hard_start_xmit(struct sk_buff *skb,
					    struct net_device *dev,
					    struct netdev_queue *txq);
extern int		dev_forward_skb(struct net_device *dev,
					struct sk_buff *skb);

extern int		netdev_budget;

extern void netdev_run_todo(void);

static inline void dev_put(struct net_device *dev)
{
	this_cpu_dec(*dev->pcpu_refcnt);
}

static inline void dev_hold(struct net_device *dev)
{
	this_cpu_inc(*dev->pcpu_refcnt);
}


extern void linkwatch_fire_event(struct net_device *dev);
extern void linkwatch_forget_dev(struct net_device *dev);

static inline bool netif_carrier_ok(const struct net_device *dev)
{
	return !test_bit(__LINK_STATE_NOCARRIER, &dev->state);
}

extern unsigned long dev_trans_start(struct net_device *dev);

extern void __netdev_watchdog_up(struct net_device *dev);

extern void netif_carrier_on(struct net_device *dev);

extern void netif_carrier_off(struct net_device *dev);

extern void netif_notify_peers(struct net_device *dev);

static inline void netif_dormant_on(struct net_device *dev)
{
	if (!test_and_set_bit(__LINK_STATE_DORMANT, &dev->state))
		linkwatch_fire_event(dev);
}

static inline void netif_dormant_off(struct net_device *dev)
{
	if (test_and_clear_bit(__LINK_STATE_DORMANT, &dev->state))
		linkwatch_fire_event(dev);
}

static inline bool netif_dormant(const struct net_device *dev)
{
	return test_bit(__LINK_STATE_DORMANT, &dev->state);
}


static inline bool netif_oper_up(const struct net_device *dev)
{
	return (dev->operstate == IF_OPER_UP ||
		dev->operstate == IF_OPER_UNKNOWN );
}

static inline bool netif_device_present(struct net_device *dev)
{
	return test_bit(__LINK_STATE_PRESENT, &dev->state);
}

extern void netif_device_detach(struct net_device *dev);

extern void netif_device_attach(struct net_device *dev);


enum {
	NETIF_MSG_DRV		= 0x0001,
	NETIF_MSG_PROBE		= 0x0002,
	NETIF_MSG_LINK		= 0x0004,
	NETIF_MSG_TIMER		= 0x0008,
	NETIF_MSG_IFDOWN	= 0x0010,
	NETIF_MSG_IFUP		= 0x0020,
	NETIF_MSG_RX_ERR	= 0x0040,
	NETIF_MSG_TX_ERR	= 0x0080,
	NETIF_MSG_TX_QUEUED	= 0x0100,
	NETIF_MSG_INTR		= 0x0200,
	NETIF_MSG_TX_DONE	= 0x0400,
	NETIF_MSG_RX_STATUS	= 0x0800,
	NETIF_MSG_PKTDATA	= 0x1000,
	NETIF_MSG_HW		= 0x2000,
	NETIF_MSG_WOL		= 0x4000,
};

#define netif_msg_drv(p)	((p)->msg_enable & NETIF_MSG_DRV)
#define netif_msg_probe(p)	((p)->msg_enable & NETIF_MSG_PROBE)
#define netif_msg_link(p)	((p)->msg_enable & NETIF_MSG_LINK)
#define netif_msg_timer(p)	((p)->msg_enable & NETIF_MSG_TIMER)
#define netif_msg_ifdown(p)	((p)->msg_enable & NETIF_MSG_IFDOWN)
#define netif_msg_ifup(p)	((p)->msg_enable & NETIF_MSG_IFUP)
#define netif_msg_rx_err(p)	((p)->msg_enable & NETIF_MSG_RX_ERR)
#define netif_msg_tx_err(p)	((p)->msg_enable & NETIF_MSG_TX_ERR)
#define netif_msg_tx_queued(p)	((p)->msg_enable & NETIF_MSG_TX_QUEUED)
#define netif_msg_intr(p)	((p)->msg_enable & NETIF_MSG_INTR)
#define netif_msg_tx_done(p)	((p)->msg_enable & NETIF_MSG_TX_DONE)
#define netif_msg_rx_status(p)	((p)->msg_enable & NETIF_MSG_RX_STATUS)
#define netif_msg_pktdata(p)	((p)->msg_enable & NETIF_MSG_PKTDATA)
#define netif_msg_hw(p)		((p)->msg_enable & NETIF_MSG_HW)
#define netif_msg_wol(p)	((p)->msg_enable & NETIF_MSG_WOL)

static inline u32 netif_msg_init(int debug_value, int default_msg_enable_bits)
{
	
	if (debug_value < 0 || debug_value >= (sizeof(u32) * 8))
		return default_msg_enable_bits;
	if (debug_value == 0)	
		return 0;
	
	return (1 << debug_value) - 1;
}

static inline void __netif_tx_lock(struct netdev_queue *txq, int cpu)
{
	spin_lock(&txq->_xmit_lock);
	txq->xmit_lock_owner = cpu;
}

static inline void __netif_tx_lock_bh(struct netdev_queue *txq)
{
	spin_lock_bh(&txq->_xmit_lock);
	txq->xmit_lock_owner = smp_processor_id();
}

static inline bool __netif_tx_trylock(struct netdev_queue *txq)
{
	bool ok = spin_trylock(&txq->_xmit_lock);
	if (likely(ok))
		txq->xmit_lock_owner = smp_processor_id();
	return ok;
}

static inline void __netif_tx_unlock(struct netdev_queue *txq)
{
	txq->xmit_lock_owner = -1;
	spin_unlock(&txq->_xmit_lock);
}

static inline void __netif_tx_unlock_bh(struct netdev_queue *txq)
{
	txq->xmit_lock_owner = -1;
	spin_unlock_bh(&txq->_xmit_lock);
}

static inline void txq_trans_update(struct netdev_queue *txq)
{
	if (txq->xmit_lock_owner != -1)
		txq->trans_start = jiffies;
}

static inline void netif_tx_lock(struct net_device *dev)
{
	unsigned int i;
	int cpu;

	spin_lock(&dev->tx_global_lock);
	cpu = smp_processor_id();
	for (i = 0; i < dev->num_tx_queues; i++) {
		struct netdev_queue *txq = netdev_get_tx_queue(dev, i);

		__netif_tx_lock(txq, cpu);
		set_bit(__QUEUE_STATE_FROZEN, &txq->state);
		__netif_tx_unlock(txq);
	}
}

static inline void netif_tx_lock_bh(struct net_device *dev)
{
	local_bh_disable();
	netif_tx_lock(dev);
}

static inline void netif_tx_unlock(struct net_device *dev)
{
	unsigned int i;

	for (i = 0; i < dev->num_tx_queues; i++) {
		struct netdev_queue *txq = netdev_get_tx_queue(dev, i);

		clear_bit(__QUEUE_STATE_FROZEN, &txq->state);
		netif_schedule_queue(txq);
	}
	spin_unlock(&dev->tx_global_lock);
}

static inline void netif_tx_unlock_bh(struct net_device *dev)
{
	netif_tx_unlock(dev);
	local_bh_enable();
}

#define HARD_TX_LOCK(dev, txq, cpu) {			\
	if ((dev->features & NETIF_F_LLTX) == 0) {	\
		__netif_tx_lock(txq, cpu);		\
	}						\
}

#define HARD_TX_UNLOCK(dev, txq) {			\
	if ((dev->features & NETIF_F_LLTX) == 0) {	\
		__netif_tx_unlock(txq);			\
	}						\
}

static inline void netif_tx_disable(struct net_device *dev)
{
	unsigned int i;
	int cpu;

	local_bh_disable();
	cpu = smp_processor_id();
	for (i = 0; i < dev->num_tx_queues; i++) {
		struct netdev_queue *txq = netdev_get_tx_queue(dev, i);

		__netif_tx_lock(txq, cpu);
		netif_tx_stop_queue(txq);
		__netif_tx_unlock(txq);
	}
	local_bh_enable();
}

static inline void netif_addr_lock(struct net_device *dev)
{
	spin_lock(&dev->addr_list_lock);
}

static inline void netif_addr_lock_nested(struct net_device *dev)
{
	spin_lock_nested(&dev->addr_list_lock, SINGLE_DEPTH_NESTING);
}

static inline void netif_addr_lock_bh(struct net_device *dev)
{
	spin_lock_bh(&dev->addr_list_lock);
}

static inline void netif_addr_unlock(struct net_device *dev)
{
	spin_unlock(&dev->addr_list_lock);
}

static inline void netif_addr_unlock_bh(struct net_device *dev)
{
	spin_unlock_bh(&dev->addr_list_lock);
}

#define for_each_dev_addr(dev, ha) \
		list_for_each_entry_rcu(ha, &dev->dev_addrs.list, list)


extern void		ether_setup(struct net_device *dev);

extern struct net_device *alloc_netdev_mqs(int sizeof_priv, const char *name,
				       void (*setup)(struct net_device *),
				       unsigned int txqs, unsigned int rxqs);
#define alloc_netdev(sizeof_priv, name, setup) \
	alloc_netdev_mqs(sizeof_priv, name, setup, 1, 1)

#define alloc_netdev_mq(sizeof_priv, name, setup, count) \
	alloc_netdev_mqs(sizeof_priv, name, setup, count, count)

extern int		register_netdev(struct net_device *dev);
extern void		unregister_netdev(struct net_device *dev);

extern int __hw_addr_add_multiple(struct netdev_hw_addr_list *to_list,
				  struct netdev_hw_addr_list *from_list,
				  int addr_len, unsigned char addr_type);
extern void __hw_addr_del_multiple(struct netdev_hw_addr_list *to_list,
				   struct netdev_hw_addr_list *from_list,
				   int addr_len, unsigned char addr_type);
extern int __hw_addr_sync(struct netdev_hw_addr_list *to_list,
			  struct netdev_hw_addr_list *from_list,
			  int addr_len);
extern void __hw_addr_unsync(struct netdev_hw_addr_list *to_list,
			     struct netdev_hw_addr_list *from_list,
			     int addr_len);
extern void __hw_addr_flush(struct netdev_hw_addr_list *list);
extern void __hw_addr_init(struct netdev_hw_addr_list *list);

extern int dev_addr_add(struct net_device *dev, unsigned char *addr,
			unsigned char addr_type);
extern int dev_addr_del(struct net_device *dev, unsigned char *addr,
			unsigned char addr_type);
extern int dev_addr_add_multiple(struct net_device *to_dev,
				 struct net_device *from_dev,
				 unsigned char addr_type);
extern int dev_addr_del_multiple(struct net_device *to_dev,
				 struct net_device *from_dev,
				 unsigned char addr_type);
extern void dev_addr_flush(struct net_device *dev);
extern int dev_addr_init(struct net_device *dev);

extern int dev_uc_add(struct net_device *dev, unsigned char *addr);
extern int dev_uc_del(struct net_device *dev, unsigned char *addr);
extern int dev_uc_sync(struct net_device *to, struct net_device *from);
extern void dev_uc_unsync(struct net_device *to, struct net_device *from);
extern void dev_uc_flush(struct net_device *dev);
extern void dev_uc_init(struct net_device *dev);

extern int dev_mc_add(struct net_device *dev, unsigned char *addr);
extern int dev_mc_add_global(struct net_device *dev, unsigned char *addr);
extern int dev_mc_del(struct net_device *dev, unsigned char *addr);
extern int dev_mc_del_global(struct net_device *dev, unsigned char *addr);
extern int dev_mc_sync(struct net_device *to, struct net_device *from);
extern void dev_mc_unsync(struct net_device *to, struct net_device *from);
extern void dev_mc_flush(struct net_device *dev);
extern void dev_mc_init(struct net_device *dev);

extern void		dev_set_rx_mode(struct net_device *dev);
extern void		__dev_set_rx_mode(struct net_device *dev);
extern int		dev_set_promiscuity(struct net_device *dev, int inc);
extern int		dev_set_allmulti(struct net_device *dev, int inc);
extern void		netdev_state_change(struct net_device *dev);
extern int		netdev_bonding_change(struct net_device *dev,
					      unsigned long event);
extern void		netdev_features_change(struct net_device *dev);
extern void		dev_load(struct net *net, const char *name);
extern void		dev_mcast_init(void);
extern struct rtnl_link_stats64 *dev_get_stats(struct net_device *dev,
					       struct rtnl_link_stats64 *storage);
extern void netdev_stats_to_stats64(struct rtnl_link_stats64 *stats64,
				    const struct net_device_stats *netdev_stats);

extern int		netdev_max_backlog;
extern int		netdev_tstamp_prequeue;
extern int		weight_p;
extern int		bpf_jit_enable;
extern int		netdev_set_master(struct net_device *dev, struct net_device *master);
extern int netdev_set_bond_master(struct net_device *dev,
				  struct net_device *master);
extern int skb_checksum_help(struct sk_buff *skb);
extern struct sk_buff *skb_gso_segment(struct sk_buff *skb,
	netdev_features_t features);
#ifdef CONFIG_BUG
extern void netdev_rx_csum_fault(struct net_device *dev);
#else
static inline void netdev_rx_csum_fault(struct net_device *dev)
{
}
#endif
extern void		net_enable_timestamp(void);
extern void		net_disable_timestamp(void);

#ifdef CONFIG_PROC_FS
extern void *dev_seq_start(struct seq_file *seq, loff_t *pos);
extern void *dev_seq_next(struct seq_file *seq, void *v, loff_t *pos);
extern void dev_seq_stop(struct seq_file *seq, void *v);
#endif

extern int netdev_class_create_file(struct class_attribute *class_attr);
extern void netdev_class_remove_file(struct class_attribute *class_attr);

extern struct kobj_ns_type_operations net_ns_type_operations;

extern const char *netdev_drivername(const struct net_device *dev);

extern void linkwatch_run_queue(void);

static inline netdev_features_t netdev_get_wanted_features(
	struct net_device *dev)
{
	return (dev->features & ~dev->hw_features) | dev->wanted_features;
}
netdev_features_t netdev_increment_features(netdev_features_t all,
	netdev_features_t one, netdev_features_t mask);
int __netdev_update_features(struct net_device *dev);
void netdev_update_features(struct net_device *dev);
void netdev_change_features(struct net_device *dev);

void netif_stacked_transfer_operstate(const struct net_device *rootdev,
					struct net_device *dev);

netdev_features_t netif_skb_features(struct sk_buff *skb);

static inline bool net_gso_ok(netdev_features_t features, int gso_type)
{
	netdev_features_t feature = gso_type << NETIF_F_GSO_SHIFT;

	
	BUILD_BUG_ON(SKB_GSO_TCPV4   != (NETIF_F_TSO >> NETIF_F_GSO_SHIFT));
	BUILD_BUG_ON(SKB_GSO_UDP     != (NETIF_F_UFO >> NETIF_F_GSO_SHIFT));
	BUILD_BUG_ON(SKB_GSO_DODGY   != (NETIF_F_GSO_ROBUST >> NETIF_F_GSO_SHIFT));
	BUILD_BUG_ON(SKB_GSO_TCP_ECN != (NETIF_F_TSO_ECN >> NETIF_F_GSO_SHIFT));
	BUILD_BUG_ON(SKB_GSO_TCPV6   != (NETIF_F_TSO6 >> NETIF_F_GSO_SHIFT));
	BUILD_BUG_ON(SKB_GSO_FCOE    != (NETIF_F_FSO >> NETIF_F_GSO_SHIFT));

	return (features & feature) == feature;
}

static inline bool skb_gso_ok(struct sk_buff *skb, netdev_features_t features)
{
	return net_gso_ok(features, skb_shinfo(skb)->gso_type) &&
	       (!skb_has_frag_list(skb) || (features & NETIF_F_FRAGLIST));
}

static inline bool netif_needs_gso(struct sk_buff *skb,
				   netdev_features_t features)
{
	return skb_is_gso(skb) && (!skb_gso_ok(skb, features) ||
		unlikely((skb->ip_summed != CHECKSUM_PARTIAL) &&
			 (skb->ip_summed != CHECKSUM_UNNECESSARY)));
}

static inline void netif_set_gso_max_size(struct net_device *dev,
					  unsigned int size)
{
	dev->gso_max_size = size;
}

static inline bool netif_is_bond_slave(struct net_device *dev)
{
	return dev->flags & IFF_SLAVE && dev->priv_flags & IFF_BONDING;
}

static inline bool netif_supports_nofcs(struct net_device *dev)
{
	return dev->priv_flags & IFF_SUPP_NOFCS;
}

extern struct pernet_operations __net_initdata loopback_net_ops;



static inline const char *netdev_name(const struct net_device *dev)
{
	if (dev->reg_state != NETREG_REGISTERED)
		return "(unregistered net_device)";
	return dev->name;
}

extern int __netdev_printk(const char *level, const struct net_device *dev,
			struct va_format *vaf);

extern __printf(3, 4)
int netdev_printk(const char *level, const struct net_device *dev,
		  const char *format, ...);
extern __printf(2, 3)
int netdev_emerg(const struct net_device *dev, const char *format, ...);
extern __printf(2, 3)
int netdev_alert(const struct net_device *dev, const char *format, ...);
extern __printf(2, 3)
int netdev_crit(const struct net_device *dev, const char *format, ...);
extern __printf(2, 3)
int netdev_err(const struct net_device *dev, const char *format, ...);
extern __printf(2, 3)
int netdev_warn(const struct net_device *dev, const char *format, ...);
extern __printf(2, 3)
int netdev_notice(const struct net_device *dev, const char *format, ...);
extern __printf(2, 3)
int netdev_info(const struct net_device *dev, const char *format, ...);

#define MODULE_ALIAS_NETDEV(device) \
	MODULE_ALIAS("netdev-" device)

#if defined(CONFIG_DYNAMIC_DEBUG)
#define netdev_dbg(__dev, format, args...)			\
do {								\
	dynamic_netdev_dbg(__dev, format, ##args);		\
} while (0)
#elif defined(DEBUG)
#define netdev_dbg(__dev, format, args...)			\
	netdev_printk(KERN_DEBUG, __dev, format, ##args)
#else
#define netdev_dbg(__dev, format, args...)			\
({								\
	if (0)							\
		netdev_printk(KERN_DEBUG, __dev, format, ##args); \
	0;							\
})
#endif

#if defined(VERBOSE_DEBUG)
#define netdev_vdbg	netdev_dbg
#else

#define netdev_vdbg(dev, format, args...)			\
({								\
	if (0)							\
		netdev_printk(KERN_DEBUG, dev, format, ##args);	\
	0;							\
})
#endif

#define netdev_WARN(dev, format, args...)			\
	WARN(1, "netdevice: %s\n" format, netdev_name(dev), ##args);


#define netif_printk(priv, type, level, dev, fmt, args...)	\
do {					  			\
	if (netif_msg_##type(priv))				\
		netdev_printk(level, (dev), fmt, ##args);	\
} while (0)

#define netif_level(level, priv, type, dev, fmt, args...)	\
do {								\
	if (netif_msg_##type(priv))				\
		netdev_##level(dev, fmt, ##args);		\
} while (0)

#define netif_emerg(priv, type, dev, fmt, args...)		\
	netif_level(emerg, priv, type, dev, fmt, ##args)
#define netif_alert(priv, type, dev, fmt, args...)		\
	netif_level(alert, priv, type, dev, fmt, ##args)
#define netif_crit(priv, type, dev, fmt, args...)		\
	netif_level(crit, priv, type, dev, fmt, ##args)
#define netif_err(priv, type, dev, fmt, args...)		\
	netif_level(err, priv, type, dev, fmt, ##args)
#define netif_warn(priv, type, dev, fmt, args...)		\
	netif_level(warn, priv, type, dev, fmt, ##args)
#define netif_notice(priv, type, dev, fmt, args...)		\
	netif_level(notice, priv, type, dev, fmt, ##args)
#define netif_info(priv, type, dev, fmt, args...)		\
	netif_level(info, priv, type, dev, fmt, ##args)

#if defined(DEBUG)
#define netif_dbg(priv, type, dev, format, args...)		\
	netif_printk(priv, type, KERN_DEBUG, dev, format, ##args)
#elif defined(CONFIG_DYNAMIC_DEBUG)
#define netif_dbg(priv, type, netdev, format, args...)		\
do {								\
	if (netif_msg_##type(priv))				\
		dynamic_netdev_dbg(netdev, format, ##args);	\
} while (0)
#else
#define netif_dbg(priv, type, dev, format, args...)			\
({									\
	if (0)								\
		netif_printk(priv, type, KERN_DEBUG, dev, format, ##args); \
	0;								\
})
#endif

#if defined(VERBOSE_DEBUG)
#define netif_vdbg	netif_dbg
#else
#define netif_vdbg(priv, type, dev, format, args...)		\
({								\
	if (0)							\
		netif_printk(priv, type, KERN_DEBUG, dev, format, ##args); \
	0;							\
})
#endif

#endif 

#endif	
