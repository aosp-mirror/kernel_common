/*
 * Linux 2.6.32 and later Kernel module for VMware MVP Hypervisor Support
 *
 * Copyright (C) 2010-2013 VMware, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; see the file COPYING.  If not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#line 5

/**
 * @file
 * @brief Check for required kernel configuration
 *
 * Check to make sure that the kernel options that the MVP hypervisor requires
 * have been enabled in the kernel that this kernel module is being built
 * against.
 */
#include <linux/version.h>

/*
 * Minimum kernel version
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 0)
#error "MVP requires a host kernel newer than 3.0.0"
#endif

/* module loading ability */
#ifndef CONFIG_MODULES
#error "MVP requires kernel loadable module support be enabled " \
	"(CONFIG_MODULES)"
#endif
#ifndef CONFIG_MODULE_UNLOAD
#error "MVP requires kernel module unload support be enabled " \
	"(CONFIG_MODULE_UNLOAD)"
#endif

/* sysfs */
#ifndef CONFIG_SYSFS
#error "MVP requires sysfs support (CONFIG_SYSFS)"
#endif

/* network traffic isolation */
#ifndef CONFIG_NAMESPACES
#error "MVP requires namespace support (CONFIG_NAMESPACES)"
#endif
#ifndef CONFIG_NET_NS
#error "MVP requires network namespace support (CONFIG_NET_NS)"
#endif

/* TCP/IP networking */
#ifndef CONFIG_INET
#error "MVP requires IPv4 support (CONFIG_INET)"
#endif
#ifndef CONFIG_IPV6
#error "MVP requires IPv6 support (CONFIG_IPV6)"
#endif

/* VPN support */
#if !defined(CONFIG_TUN) && !defined(CONFIG_TUN_MODULE)
#error "MVP VPN support requires TUN device support (CONFIG_TUN)"
#endif

#if !defined(CONFIG_NETFILTER) && !defined(PVTCP_DISABLE_NETFILTER)
#error "MVP requires netfilter support (CONFIG_NETFILTER)"
#endif

/* Force /proc/config.gz support for eng/userdebug builds */
#ifdef MVP_DEBUG
#if !defined(CONFIG_IKCONFIG) || !defined(CONFIG_IKCONFIG_PROC)
#error "MVP_DEBUG requires /proc/config.gz support (CONFIG_IKCONFIG_PROC)"
#endif
#endif

/* Sanity check we're only dealing with the memory hotplug + migrate and/or
 * compaction combo */
#ifdef CONFIG_MIGRATION
#if defined(CONFIG_NUMA) || defined(CONFIG_CPUSETS) || \
	defined(CONFIG_MEMORY_FAILURE)
#error "MVP not tested with migration features other than " \
	"CONFIG_MEMORY_HOTPLUG and CONFIG_COMPACTION"
#endif
#endif
