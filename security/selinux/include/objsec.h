/*
 *  NSA Security-Enhanced Linux (SELinux) security module
 *
 *  This file contains the SELinux security data structures for kernel objects.
 *
 *  Author(s):  Stephen Smalley, <sds@epoch.ncsc.mil>
 *		Chris Vance, <cvance@nai.com>
 *		Wayne Salamon, <wsalamon@nai.com>
 *		James Morris <jmorris@redhat.com>
 *
 *  Copyright (C) 2001,2002 Networks Associates Technology, Inc.
 *  Copyright (C) 2003 Red Hat, Inc., James Morris <jmorris@redhat.com>
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License version 2,
 *	as published by the Free Software Foundation.
 */
#ifndef _SELINUX_OBJSEC_H_
#define _SELINUX_OBJSEC_H_

#include <linux/list.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/binfmts.h>
#include <linux/in.h>
#include <linux/spinlock.h>
#include "flask.h"
#include "avc.h"

struct task_security_struct {
	u32 osid;		
	u32 sid;		
	u32 exec_sid;		
	u32 create_sid;		
	u32 keycreate_sid;	
	u32 sockcreate_sid;	
};

struct inode_security_struct {
	struct inode *inode;	
	struct list_head list;	
	struct rcu_head rcu;	
	u32 task_sid;		
	u32 sid;		
	u16 sclass;		
	unsigned char initialized;	
	u32 tag;		
	struct mutex lock;
};

struct file_security_struct {
	u32 sid;		
	u32 fown_sid;		
	u32 isid;		
	u32 pseqno;		
};

struct superblock_security_struct {
	struct super_block *sb;		
	u32 sid;			
	u32 def_sid;			
	u32 mntpoint_sid;		
	unsigned int behavior;		
	unsigned char flags;		
	struct mutex lock;
	struct list_head isec_head;
	spinlock_t isec_lock;
};

struct msg_security_struct {
	u32 sid;	
};

struct ipc_security_struct {
	u16 sclass;	
	u32 sid;	
};

struct netif_security_struct {
	int ifindex;			
	u32 sid;			
};

struct netnode_security_struct {
	union {
		__be32 ipv4;		
		struct in6_addr ipv6;	
	} addr;
	u32 sid;			
	u16 family;			
};

struct netport_security_struct {
	u32 sid;			
	u16 port;			
	u8 protocol;			
};

struct sk_security_struct {
#ifdef CONFIG_NETLABEL
	enum {				
		NLBL_UNSET = 0,
		NLBL_REQUIRE,
		NLBL_LABELED,
		NLBL_REQSKB,
		NLBL_CONNLABELED,
	} nlbl_state;
	struct netlbl_lsm_secattr *nlbl_secattr; 
#endif
	u32 sid;			
	u32 peer_sid;			
	u16 sclass;			
};

struct key_security_struct {
	u32 sid;	
};

extern unsigned int selinux_checkreqprot;

#endif 
