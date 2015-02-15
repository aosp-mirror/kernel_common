/* GAN Virtual Ethernet Device
 *
 * Copyright © 2010, Kineto Wireless, Inc. sourcecode@kineto.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ip.h>		/* struct iphdr */
#include <linux/tcp.h>		/* struct tcphdr */

#include <linux/skbuff.h>
#include <linux/wakelock.h>
#include <linux/net.h>
#include <linux/socket.h>


#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
/* For socket etc */
#include <linux/net.h>
#include <net/sock.h>
#include <linux/tcp.h>
#include <linux/in.h>
#include <linux/uaccess.h>
#include <linux/file.h>
#include <linux/socket.h>
/*#include <linux/smp_lock.h> ==> Commenting this since in Kernal 3.0 this feature has been removed.*/
#include <linux/slab.h>
#include <linux/kthread.h>

#include <linux/if_arp.h>
#include <linux/workqueue.h>

#define MODULE_NAME "gannet"

#define GAN_VIF_LINK_PORT	13010
#define GAN_PS_PORT_2		13001

struct gannet_private {
	struct net_device_stats stats;
	struct sockaddr_in tx_addr;
	struct sockaddr_in rx_addr;
	struct socket *tx_sock;
	struct socket *rx_sock;
};

struct gannet_thread {
	struct task_struct *thread;
	struct net_device *dev;
	struct gannet_private *priv;
};

static struct gannet_thread *gthread;
static struct net_device *gdev;
static int gthreadquit;
static int firstsetup;

/* Work queue modifications */
static struct workqueue_struct *gannet_wq;

struct gannet_work_struct {
	struct work_struct gannet_work;
	struct sk_buff *skb;
	struct gannet_private *p;
};


static int count_this_packet(void *_hdr, int len)
{
	struct ethhdr *hdr = _hdr;

	if (len >= ETH_HLEN && hdr->h_proto == htons(ETH_P_ARP))
		return 0;

	return 1;
}


static void rx(unsigned char *buf, int len)
{
	struct sk_buff *skb;
	void *ptr = 0;
	int sz;
	int r;

	sz = len;

	if (sz > 1514) {
		printk(KERN_ERR MODULE_NAME "gannet discarding %d len\n", sz);
		ptr = 0;
	} else {
		skb = dev_alloc_skb(sz + 14 + NET_IP_ALIGN);
		if (skb == NULL) {
			printk(KERN_ERR MODULE_NAME
				   "gannet cannot allocate skb\n");
		} else {
			skb_reserve(skb, NET_IP_ALIGN);

			ptr = skb_put(skb, 14); /* ethernet hdr */
			memcpy(&((unsigned char *) ptr)[6],
				   gthread->dev->dev_addr, 6);

			ptr = skb_put(skb, sz);
			memcpy(ptr, buf, sz);

			skb->dev = gthread->dev;
			skb->protocol = eth_type_trans(skb, gthread->dev);
			skb->protocol = htons(ETH_P_IP);
			skb->ip_summed = CHECKSUM_NONE; /* check it */

			skb->pkt_type = PACKET_HOST;

			if (count_this_packet(ptr, skb->len)) {
				gthread->priv->stats.rx_packets++;
				gthread->priv->stats.rx_bytes += skb->len;
			}
			r = netif_rx(skb);
		}
	}
}


static int ksocket_receive(struct socket *sock,
			   struct sockaddr_in *addr,
			   unsigned char *buf, int len)
{
	struct msghdr msg;
	struct iovec iov;
	mm_segment_t oldfs;
	int size = 0;

	if (sock->sk == NULL)
		return 0;

	iov.iov_base = buf;
	iov.iov_len = len;

	msg.msg_flags = 0;
	msg.msg_name = addr;
	msg.msg_namelen = sizeof(struct sockaddr_in);
	msg.msg_control = NULL;
	msg.msg_controllen = 0;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = NULL;

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	size = sock_recvmsg(sock, &msg, len, msg.msg_flags);
	set_fs(oldfs);

	return size;
}


static int ksocket_sendto(struct socket *sock,
			  struct sockaddr_in *addr,
			  unsigned char *buf, int len)
{
	struct msghdr msg;
	struct iovec iov;
	mm_segment_t oldfs;
	int size = 0;

	if (sock->sk == NULL)
		return 0;

	iov.iov_base = buf;
	iov.iov_len = len;

	msg.msg_flags = 0;
	msg.msg_name = addr;
	msg.msg_namelen = sizeof(struct sockaddr_in);
	msg.msg_control = NULL;
	msg.msg_controllen = 0;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = NULL;

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	size = sock_sendmsg(sock, &msg, len);
	set_fs(oldfs);

	return size;
}


static void gannet_wq_func(struct work_struct *work)
{
	struct gannet_work_struct *gannet_work =
					(struct gannet_work_struct *)work;

	/* write skb->data of skb->len to udp socket */
	struct gannet_private *p = gannet_work->p;
	struct sk_buff *skb = gannet_work->skb;
	char *data;
	int len;

	data = skb->data;
	len = skb->len;

	/* Remove ethernet header */
	data += 14;
	len -= 14;

	if (len != ksocket_sendto(p->tx_sock, &p->tx_addr, data, len)) {
		printk(KERN_ERR
			   "gannet sendto() failed, dropping packet\n");
	} else {
		if (count_this_packet(data, len)) {
			p->stats.tx_packets++;
			p->stats.tx_bytes += len;
		}
	}

	dev_kfree_skb(skb);

	kfree((void *)work);
}


static void gannet_recvloop(void)
{
	int size;
	int bufsize = 1600;
	unsigned char buf[bufsize + 1];
	/* Kernal 3.0 Changes Start */
	static DEFINE_MUTEX(ker_lock_mutex);

	/* kernel thread initialization */
	/* lock_kernel(); ==> Commenting this line since in Kernal 3.0 this feature has been removed.*/
	   mutex_lock(&ker_lock_mutex);
	/* Kernal 3.0 Changes End */


	current->flags |= PF_NOFREEZE;

	/* daemonize (take care with signals,
	   after daemonize they are disabled) */
	daemonize(MODULE_NAME);
	allow_signal(SIGKILL);
	/* Kernal 3.0 Changes Start */
	/* unlock_kernel(); ==> Commenting this line since in Kernal 3.0 this feature has been removed*/
	  mutex_unlock(&ker_lock_mutex);
    /* Kernal 3.0 Changes End */

	/* main loop */
	while (!gthreadquit) {
		memset(&buf, 0, bufsize + 1);
		size = ksocket_receive(gthread->priv->rx_sock,
					   &gthread->priv->rx_addr,
					   buf, bufsize);

		if (signal_pending(current))
			break;

		if (size < 0) {
			printk(KERN_ERR MODULE_NAME
				   "gannet: error getting datagram, "
				   "sock_recvmsg error = %d\n", size);
		} else {
			/* send to kernel */
			rx(buf, size);
		}
	}

	printk(KERN_INFO "gannet thread exit\n");
}


static int gannet_open(struct net_device *dev)
{
	int ret;
	struct gannet_private *p;

	printk(KERN_DEBUG "gannet_open()\n");
	netif_start_queue(dev);

	if (firstsetup == 1) {
		printk(KERN_DEBUG "gannet firstsetup executed\n");
		firstsetup = 0;

		gthreadquit = 0;
		p = netdev_priv(dev);

		/* Create tx socket */
		ret = sock_create(PF_INET, SOCK_DGRAM,
				IPPROTO_UDP, &p->tx_sock);
		if (ret < 0) {
			printk(KERN_ERR
			   "gannet tx socket() failed, failing init.\n");
			unregister_netdev(dev);
			free_netdev(dev);
			return -EIO;	/* I/O error */
		}
		memset(&p->tx_addr, 0, sizeof(p->tx_addr));
		p->tx_addr.sin_family = AF_INET;
		p->tx_addr.sin_port = htons(GAN_PS_PORT_2);
		p->tx_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

		/* Create rx socket */
		ret = sock_create(PF_INET, SOCK_DGRAM,
				IPPROTO_UDP, &p->rx_sock);
		if (ret < 0) {
			printk(KERN_ERR
			   "gannet rx socket() failed, failing init.\n");
			sock_release(p->tx_sock);
			unregister_netdev(dev);
			free_netdev(dev);
			return -EIO;	/* I/O error? */
		}
		memset(&p->rx_addr, 0, sizeof(p->rx_addr));
		p->rx_addr.sin_family = AF_INET;
		p->rx_addr.sin_port = htons(GAN_VIF_LINK_PORT);
		p->rx_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

		/* Bind rx socket */
		ret = p->rx_sock->ops->bind(p->rx_sock,
					(struct sockaddr *) &p->rx_addr,
					sizeof(struct sockaddr));
		if (ret < 0) {
			printk(KERN_ERR "gannet rx socket() bind failed.\n");
			sock_release(p->tx_sock);
			sock_release(p->rx_sock);
			unregister_netdev(dev);
			free_netdev(dev);
			return -EIO;	/* I/O error */
		} else {
			printk(KERN_ERR "gannet rx socket() bind success.\n");
		}

		/* Create kernel thread for rx loop */
		gthread = kzalloc(sizeof(struct gannet_thread), GFP_KERNEL);
		if (gthread == NULL) {
			printk(KERN_ERR MODULE_NAME
			 "gannet: unable to allocate mem for kernel thread\n");
			sock_release(p->tx_sock);
			sock_release(p->rx_sock);
			unregister_netdev(dev);
			free_netdev(dev);
			return -ENOMEM;
		}

		/* Create work queue */
		gannet_wq = create_workqueue("gannet_queue");
		if (gannet_wq == NULL) {
			printk(KERN_ERR MODULE_NAME
			 "gannet: unable to initialize work queue\n");
			kfree(gthread);
			sock_release(p->tx_sock);
			sock_release(p->rx_sock);
			unregister_netdev(dev);
			free_netdev(dev);
			return -ENOMEM;
		}

		/* Store ref to private info */
		gthread->dev = dev;
		gthread->priv = p;

		printk(KERN_INFO "gannet starting kernel thread\n");
		gthread->thread = kthread_run((void *) gannet_recvloop,
					  NULL, MODULE_NAME);
		if (IS_ERR(gthread->thread)) {
				printk(KERN_ERR MODULE_NAME
				"gannet: unable to start kernel thread\n");
			sock_release(p->tx_sock);
			sock_release(p->rx_sock);
			unregister_netdev(dev);
			free_netdev(dev);
			kfree(gthread);
			destroy_workqueue(gannet_wq);
			gthread = NULL;
			return -ENOMEM;
		}
	}

	return 0;
}

static int gannet_stop(struct net_device *dev)
{
	printk(KERN_DEBUG "gannet_stop()\n");
	netif_stop_queue(dev);
	return 0;
}



static int gannet_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct gannet_private *p = netdev_priv(dev);
	int ret;
	struct gannet_work_struct *work;

	if (NULL == ipip_hdr(skb)) {
		printk(KERN_WARNING "gannet dropping packet, no IP header\n");
		dev_kfree_skb(skb);
		return NET_XMIT_DROP;
	}

	if (skb->len < (sizeof(struct ethhdr) + sizeof(struct iphdr))) {
		printk(KERN_WARNING
			   "gannet: Packet too short (%i bytes)\n", skb->len);
		return 0;
	}

	if (gannet_wq) {
		work = kmalloc(sizeof(struct gannet_work_struct), GFP_ATOMIC);
		if (work) {
			INIT_WORK((struct work_struct *)work, gannet_wq_func);
			work->skb = skb;
			work->p = p;

			ret = queue_work(gannet_wq, (struct work_struct *)work);

		} else {
			printk(KERN_WARNING "gannet dropping packet, \
					cannot allocate work queue item\n");
		}
	}

	return 0;
}

static struct net_device_stats *gannet_get_stats(struct net_device *dev)
{
	struct gannet_private *p = netdev_priv(dev);
	return &p->stats;
}

/*static void gannet_set_multicast_list(struct net_device *dev)
{
}*/

static void gannet_tx_timeout(struct net_device *dev)
{
	printk(KERN_DEBUG "gannet_tx_timeout()\n");
}


static const struct net_device_ops gannet_netdev_ops = {
	.ndo_open = gannet_open,
	.ndo_stop = gannet_stop,
	.ndo_start_xmit = gannet_xmit,
	.ndo_get_stats = gannet_get_stats,
	//.ndo_set_multicast_list = gannet_set_multicast_list,
	.ndo_tx_timeout = gannet_tx_timeout,
	.ndo_change_mtu = NULL,
};

static void __init gannet_setup(struct net_device *dev)
{
	printk(KERN_INFO "gannet_setup\n");

	ether_setup(dev);

	dev->mtu = 1000;
	dev->netdev_ops = &gannet_netdev_ops;
	/* The minimum time (in jiffies) that should pass before the networking
	 * layer decides that a transmission timeout has occurred and calls the
	 * driver's tx_time-out function.
	 */
	dev->watchdog_timeo = 200;

	/* keep the default flags, just add NOARP */
	dev->flags |= IFF_NOARP;

	random_ether_addr(dev->dev_addr);
  /* Commenting netif_start_queue to avoid Kernal Panic in 2.3.38 Kernal
	netif_start_queue(dev);*/
}

static int __init gannet_init(void)
{
	int ret;
	struct net_device *dev;

	dev = alloc_netdev_mq(sizeof(struct gannet_private),
			   "gannet%d", gannet_setup, 1);
	if (NULL == dev)
		return -ENOMEM;

	ret = register_netdev(dev);
	if (ret) {
		printk(KERN_ERR "gannet failed to register netdev\n");
		free_netdev(dev);
		return ret;
	}

	gdev = dev;

	firstsetup = 1;

	printk(KERN_INFO "gannet initialized OK\n");

	return 0;
}

static void __exit gannet_exit(void)
{
	gthreadquit = 1;
	if (NULL != gdev) {
		struct gannet_private *p = netdev_priv(gdev);

		flush_scheduled_work();
		destroy_workqueue(gannet_wq);
		sock_release(p->rx_sock);
		sock_release(p->tx_sock);
		unregister_netdev(gdev);
		free_netdev(gdev);
		kfree(gthread);
		gthread = NULL;
		gdev = NULL;
	}
}

module_init(gannet_init);
module_exit(gannet_exit);

MODULE_DESCRIPTION("Kineto GAN Virtual Ethernet Device");
MODULE_ALIAS("gannet");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
