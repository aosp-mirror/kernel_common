/*
 *  HTC - ROOT File Control
 *
 *  This file contains the rootfctrl hook function implementations.
 *
 *  Authors:  Sean Lin, Mike Wu
 *
 *  Copyright (C) 2011 HTC Corp, Sean Lin, Mike Wu
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2,
 *  as published by the Free Software Foundation.
 */


#include <linux/security.h>
#include <linux/namei.h>
#include <linux/limits.h>
#include <linux/cred.h>
#include <linux/sched.h>
#include <linux/kdev_t.h>
#include <linux/stat.h>
#include <linux/fs.h>

#define RTFCTL_NORMAL_MODE		1
#define RTFCTL_TRACKING_MODE		2
#define RTFCTL_TEST_MODE			3
#define RTFCTL_RUN_MODE			RTFCTL_NORMAL_MODE

#define ROOTFCTRL_DEBUG         0

#if ROOTFCTRL_DEBUG
#define RTFCTL_MSG(s...) printk("[RTFCTL]" s)
#else
#define RTFCTL_MSG(s...) do {}while(0)
#endif

#define MAX_NAME_BUF_LEN	160
#define MAX_D_NAME_LEN		28
#define MAX_P_NAME_LEN		(MAX_NAME_BUF_LEN - MAX_D_NAME_LEN - 4) 

pid_t flc_daemon_pid = -1, flc_agent_pid = -1;
pid_t zygote_pid = -1, installd_pid = -1, adbd_pid = -1;

#define UID_FELICA_RW_CTRL		9986
#define UID_FELICA_LOCK_APP		9990
#define UID_MOBILE_FELICA_CLIENT	9989

#define RTFCTL_FELICA_UID_NUM		2
uid_t felica_uid[16] = {UID_FELICA_LOCK_APP, UID_MOBILE_FELICA_CLIENT};

#define RTFCTL_FELICA_FILE_NUM		0
char *felica_file[16] = {"/data/data/jp.co.fsi.felicalock", "/data/data/com.felicanetworks.mfc",
    "/data/dalvik-cache/system@app@KDDI_Felicalock.apk@classes.dex",
    "/data/dalvik-cache/system@app@MobileFeliCaClient.apk@classes.dex"};

#define RTFCTL_FELICA_CFG_NUM		4
char *felica_cfg[16] = {"/system/etc/felica/bookmark.cfg", "/system/etc/felica/common.cfg",
    "/system/etc/felica/mfm.cfg", "/system/etc/felica/mfs.cfg"};

#define RTFCTL_FELICA_APK_NUM		8
char *felica_apk[16] = {"/system/app/MobileFeliCaClient.apk", "/system/app/MobileFeliCaSettingApp.apk",
    "/system/app/MobileFeliCaMenuApp.apk", "/system/app/MobileFeliCaWebPluginBoot.apk",
    "/system/app/MobileFeliCaClient.odex", "/system/app/MobileFeliCaSettingApp.odex",
    "/system/app/MobileFeliCaMenuApp.odex", "/system/app/MobileFeliCaWebPluginBoot.odex"};

#define RTFCTL_FELICA_DEV_NUM		8
char *felica_dev[16] = {"/dev/felica", "/dev/felica_pon", "/dev/felica_cen", "/dev/felica_rfs",
    "/dev/felica_rws", "/dev/felica_int", "/dev/felica_int_poll", "/dev/felica_uid"};
unsigned int felica_dev_t[16][2] = {{91, 0}, {92, 0}, {93, 0}, {94, 0}, {95, 0}, {96, 0}, {97, 0}, {98, 0}};
unsigned int felica_dev_mode[16] = {0666, 0666, 0666, 0444, 0666, 0444, 0400, 0222};
uid_t felica_dev_uid[16] = {UID_MOBILE_FELICA_CLIENT, UID_MOBILE_FELICA_CLIENT, UID_FELICA_LOCK_APP,
		UID_MOBILE_FELICA_CLIENT, UID_MOBILE_FELICA_CLIENT, UID_MOBILE_FELICA_CLIENT,
		UID_MOBILE_FELICA_CLIENT, UID_MOBILE_FELICA_CLIENT};

#define UID_INSTALLD_SERVICE   1012

#define UID_NFC_SERVICE        1027

#ifdef CONFIG_APQ8064_ONLY
#define RTFCTL_NFC_DEV_NUM     8
#else
#define RTFCTL_NFC_DEV_NUM     0   
#endif
char *nfc_dev[16] = {"/dev/ttyHS1", "/dev/snfc_pon", "/dev/snfc_cen", "/dev/snfc_rfs",
    "/dev/snfc_intu", "/dev/snfc_intu_polling", "/dev/snfc_auto_polling", "/dev/snfc_hsel"};
unsigned int nfc_dev_t[16][2] = {{242, 1}, {101, 0}, {102, 0}, {103, 0}, {104, 0}, {105, 0}, {106, 0}, {107, 0}};
unsigned int nfc_dev_mode[16] = {0660, 0660, 0660, 0660, 0660, 0660, 0660, 0660};

static char *get_full_path(struct path *path, struct dentry *dentry, char *buf)
{
	char *full_path = 0;

	
	memset(buf, 0, MAX_NAME_BUF_LEN);
	full_path = d_path(path, buf, MAX_P_NAME_LEN);
	if (!full_path  || IS_ERR(full_path)) {
		RTFCTL_MSG("[ERR] %s: d_path fail...\n", __FUNCTION__);
		return 0;
	}

	if (dentry) {
		if (strlen(dentry->d_name.name) < MAX_D_NAME_LEN) {
			strcat(full_path, "/");
			strcat(full_path, dentry->d_name.name);
		}else
			return 0;
	}

	return full_path;
}

#define RTFCTL_FILE_TYPE_NONE		0
#define RTFCTL_FILE_TYPE_NORMAL	1
#define RTFCTL_FILE_TYPE_DEVICE	2
#define RTFCTL_FILE_TYPE_APK		3
#define RTFCTL_FILE_TYPE_CFG		4
static int is_felica_file(const char *full_path)
{
	int i;

	if (full_path == NULL)
		return RTFCTL_FILE_TYPE_NONE;

	for (i = 0; i < RTFCTL_FELICA_FILE_NUM; i++) {
		if (!strcmp(full_path, felica_file[i])) {
			RTFCTL_MSG("file: %s\n", full_path);
			return RTFCTL_FILE_TYPE_NORMAL;
		}
	}

	for (i = 0; i < RTFCTL_FELICA_CFG_NUM; i++) {
		if (!strcmp(full_path, felica_cfg[i])) {
			RTFCTL_MSG("file: %s\n", full_path);
			return RTFCTL_FILE_TYPE_CFG;
		}
	}

	for (i = 0; i < RTFCTL_FELICA_APK_NUM; i++) {
		if (!strcmp(full_path, felica_apk[i])) {
			RTFCTL_MSG("file: %s\n", full_path);
			return RTFCTL_FILE_TYPE_APK;
		}
	}

	for (i = 0; i < RTFCTL_FELICA_DEV_NUM; i++) {
		if (!strcmp(full_path, felica_dev[i])) {
			RTFCTL_MSG("file: %s\n", full_path);
			return RTFCTL_FILE_TYPE_DEVICE;
		}
	}

	return RTFCTL_FILE_TYPE_NONE;
}

static int is_felica_RWP_file(const char *full_path)
{
	int i;

	if (full_path == NULL)
		return RTFCTL_FILE_TYPE_NONE;

	for (i = 0; i < RTFCTL_FELICA_DEV_NUM; i++) {
		if (!strcmp(full_path, felica_dev[i])) {
			RTFCTL_MSG("file: %s\n", full_path);
			return RTFCTL_FILE_TYPE_DEVICE;
		}
	}

	return RTFCTL_FILE_TYPE_NONE;
}

static int is_felica_WP_file(const char *full_path)
{
	int i;

	if (full_path == NULL)
		return RTFCTL_FILE_TYPE_NONE;

	for (i = 0; i < RTFCTL_FELICA_FILE_NUM; i++) {
		if (!strcmp(full_path, felica_file[i])) {
			RTFCTL_MSG("file: %s\n", full_path);
			return RTFCTL_FILE_TYPE_NORMAL;
		}
	}

	for (i = 0; i < RTFCTL_FELICA_CFG_NUM; i++) {
		if (!strcmp(full_path, felica_cfg[i])) {
			RTFCTL_MSG("file: %s\n", full_path);
			return RTFCTL_FILE_TYPE_CFG;
		}
	}

	for (i = 0; i < RTFCTL_FELICA_APK_NUM; i++) {
		if (!strcmp(full_path, felica_apk[i])) {
			RTFCTL_MSG("file: %s\n", full_path);
			return RTFCTL_FILE_TYPE_APK;
		}
	}

	return RTFCTL_FILE_TYPE_NONE;
}

static int is_felica_uid(uid_t uid)
{
	int i;

	for (i = 0; i < RTFCTL_FELICA_UID_NUM; i++) {
		if (felica_uid[i] == uid)
			return true;
	}
	return false;
}

#define RTFCTL_FELICA_DEV_NONE	0
#define RTFCTL_FELICA_DEV_VALID	1
#define RTFCTL_FELICA_DEV_INVALID	-1
#define RTFCTL_FELICA_DEV_FAKE		-2
static int is_felica_dev(unsigned int major,unsigned int minor, const char *full_path)
{
	int i;

	if (full_path == NULL)
		return RTFCTL_FELICA_DEV_NONE;

	for (i = 0; i < RTFCTL_FELICA_DEV_NUM; i++) {
		if (!strcmp(full_path, felica_dev[i])) {
			RTFCTL_MSG("file: %s (%u, %u)\n", full_path, major, minor);
			if (felica_dev_t[i][0] == major && felica_dev_t[i][1] == minor)
				return RTFCTL_FELICA_DEV_VALID;
			else
				return RTFCTL_FELICA_DEV_INVALID;
		}
	}

	
	for (i = 0; i < RTFCTL_FELICA_DEV_NUM; i++) {
		
		if(felica_dev_t[i][0] == major) {
			RTFCTL_MSG("fake file: %s (%u, %u)\n", full_path, major, minor);
			return RTFCTL_FELICA_DEV_FAKE;
		}
	}

	return RTFCTL_FELICA_DEV_NONE;
}

static int is_felica_mode_valid(umode_t mode, const char *full_path)
{
	int i;

	if (full_path == NULL)
		return true;

	for (i = 0; i < RTFCTL_FELICA_FILE_NUM; i++) {
		if (!strcmp(full_path, felica_file[i])) {
			RTFCTL_MSG("file: %s\n", full_path);
			if (!(mode & 002) && current_uid() == 0)
				return true;
			else {
				RTFCTL_MSG("uid: %d\n", current_uid());
				RTFCTL_MSG("mode: %o\n", mode);
				return false;
			}
		}
	}

	for (i = 0; i < RTFCTL_FELICA_CFG_NUM; i++) {
		if (!strcmp(full_path, felica_cfg[i])) {
			RTFCTL_MSG("file: %s\n", full_path);
			if (mode == 0444 && current_uid() == 0)
				return true;
			else {
				RTFCTL_MSG("uid: %d\n", current_uid());
				RTFCTL_MSG("mode: %o\n", mode);
				return false;
			}
		}
	}

	for (i = 0; i < RTFCTL_FELICA_APK_NUM; i++) {
		if (!strcmp(full_path, felica_apk[i])) {
			RTFCTL_MSG("file: %s\n", full_path);
			if (!(mode & 003) && current_uid() == 0)
				return true;
			else {
				RTFCTL_MSG("uid: %d\n", current_uid());
				RTFCTL_MSG("mode: %o\n", mode);
				return false;
			}
		}
	}

	for (i = 0; i < RTFCTL_FELICA_DEV_NUM; i++) {
		if (!strcmp(full_path, felica_dev[i])) {
			RTFCTL_MSG("file: %s\n", full_path);
			if ((felica_dev_mode[i] == -1 || mode == felica_dev_mode[i]) && current_uid() == 0)
				return true;
			else {
				RTFCTL_MSG("uid: %d\n", current_uid());
				RTFCTL_MSG("mode: %o\n", mode);
				return false;
			}
		}
	}

	return true;
}

static int is_felica_owner_valid(uid_t uid, gid_t gid, const char *full_path)
{
	int i;

	if (full_path == NULL)
		return 1;

	for (i = 0; i < RTFCTL_FELICA_FILE_NUM; i++) {
		if (!strcmp(full_path, felica_file[i])) {
			RTFCTL_MSG("file: %s\n", full_path);
			if ((uid == -1 || is_felica_uid(uid)) && (gid == -1 || is_felica_uid(gid)))
				return 1;
			else {
				RTFCTL_MSG("uid: %d\n", uid);
				RTFCTL_MSG("gid: %d\n", gid);
				return -1;
			}
		}
	}

	for (i = 0; i < RTFCTL_FELICA_CFG_NUM; i++) {
		if (!strcmp(full_path, felica_cfg[i])) {
			RTFCTL_MSG("file: %s\n", full_path);
			if ((uid == -1 || uid == 0) && (gid == -1 || gid == 0))
				return 1;
			else {
				RTFCTL_MSG("uid: %d\n", uid);
				RTFCTL_MSG("gid: %d\n", gid);
				return -1;
			}
		}
	}

	for (i = 0; i < RTFCTL_FELICA_APK_NUM; i++) {
		if (!strcmp(full_path, felica_apk[i])) {
			RTFCTL_MSG("file: %s\n", full_path);
			if ((uid == -1 || uid == 0) && (gid == -1 || gid == 0))
				return 1;
			else {
				RTFCTL_MSG("uid: %d\n", uid);
				RTFCTL_MSG("gid: %d\n", gid);
				return -1;
			}
		}
	}

	for (i = 0; i < RTFCTL_FELICA_DEV_NUM; i++) {
		if (!strcmp(full_path, felica_dev[i])) {
			RTFCTL_MSG("file: %s\n", full_path);
			if ((uid == -1 || felica_dev_uid[i] == -1 || uid == felica_dev_uid[i]) &&
				(gid == -1 || felica_dev_uid[i] == -1 || gid == felica_dev_uid[i]))
				return 1;
			else {
				RTFCTL_MSG("uid: %d\n", uid);
				RTFCTL_MSG("gid: %d\n", gid);
				return -1;
			}
		}
	}

	return 0;
}

#define RTFCTL_NFC_TYPE_NONE     0
#define RTFCTL_NFC_TYPE_DEVICE   1
static int is_nfc_file(const char *full_path)
{
	int i;

	if (full_path == NULL)
		return RTFCTL_NFC_TYPE_NONE;

	for (i = 0; i < RTFCTL_NFC_DEV_NUM; i++) {
		if (!strcmp(full_path, nfc_dev[i])) {
			RTFCTL_MSG("file: %s\n", full_path);
			return RTFCTL_NFC_TYPE_DEVICE;
		}
	}

	return RTFCTL_NFC_TYPE_NONE;
}

static int is_nfc_uid(uid_t uid)
{
#ifdef CONFIG_APQ8064_ONLY
	if (UID_NFC_SERVICE == uid)
		return true;
#endif

	return false;
}

#define RTFCTL_NFC_DEV_NONE      0
#define RTFCTL_NFC_DEV_VALID     1
#define RTFCTL_NFC_DEV_INVALID  -1
#define RTFCTL_NFC_DEV_FAKE     -2
static int is_nfc_dev(unsigned int major,unsigned int minor, const char *full_path)
{
	int i;

	if (full_path == NULL)
		return RTFCTL_NFC_DEV_NONE;

	for (i = 1; i < RTFCTL_NFC_DEV_NUM; i++) {
		if (!strcmp(full_path, nfc_dev[i])) {
			RTFCTL_MSG("file: %s (%u, %u)\n", full_path, major, minor);
			if (nfc_dev_t[i][0] == major && nfc_dev_t[i][1] == minor)
				return RTFCTL_NFC_DEV_VALID;
			else
				return RTFCTL_NFC_DEV_INVALID;
		}
	}

	
	for (i = 1; i < RTFCTL_NFC_DEV_NUM; i++) {
		
		if(nfc_dev_t[i][0] == major) {
			RTFCTL_MSG("fake file: %s (%u, %u)\n", full_path, major, minor);
			return RTFCTL_NFC_DEV_FAKE;
		}
	}

	return RTFCTL_NFC_DEV_NONE;
}
static int is_nfc_mode_valid(umode_t mode, const char *full_path)
{
	int i;

	if (full_path == NULL)
		return true;

	for (i = 0; i < RTFCTL_NFC_DEV_NUM; i++) {
		if (!strcmp(full_path, nfc_dev[i])) {
			RTFCTL_MSG("file: %s\n", full_path);
			if ((nfc_dev_mode[i] == -1 || mode == nfc_dev_mode[i]) && current_uid() == 0)
				return true;
			else {
				RTFCTL_MSG("uid: %d\n", current_uid());
				RTFCTL_MSG("mode: %o\n", mode);
				return false;
			}
		}
	}

	return true;
}

static int is_nfc_owner_valid(uid_t uid, gid_t gid, const char *full_path)
{
	int i;

	if (full_path == NULL)
		return 1;

	for (i = 0; i < RTFCTL_NFC_DEV_NUM; i++) {
		if (!strcmp(full_path, nfc_dev[i])) {
			RTFCTL_MSG("file: %s\n", full_path);
			if ((uid == -1 || is_nfc_uid(uid)) && (gid == -1 || is_nfc_uid(gid)))
				return 1;
			else {
				RTFCTL_MSG("uid: %d\n", uid);
				RTFCTL_MSG("gid: %d\n", gid);
				return -1;
			}
		}
	}

	return 0;
}

#if (RTFCTL_RUN_MODE != RTFCTL_TRACKING_MODE)
static int is_non_felica_root(uid_t uid, pid_t pid)
{
	if (uid == 0) {
		if(pid != flc_daemon_pid && pid != flc_agent_pid)
			return true;
		else {
			RTFCTL_MSG("Felica Root: %s\n", (pid == flc_daemon_pid)? "felica_daemon": "felica_agent");
			return false;
		}
	}

	return false;
}

#define  parent_uid()   (current->real_parent->cred->uid)
static int is_installd(pid_t pid, const char *name, uid_t uid)
{
	if (pid == installd_pid)
		return 1;
	if (name && !strcmp("installd", name) && (uid == 0))
		return 1;
	
	if (name && !strcmp("installd", name) && (uid == UID_INSTALLD_SERVICE))
		return 1;
	return 0;
}

static int is_zygote(struct task_struct *chk_task)
{
	pid_t pid = task_tgid_vnr(chk_task), ppid;
	char tcomm[sizeof(chk_task->comm)];

	while ((pid != 1) && (pid != 2) && (pid != 0) && (chk_task->real_parent)){

		if (pid == zygote_pid)
			return 1;

		ppid = task_tgid_vnr(chk_task->real_parent);
		get_task_comm(tcomm, chk_task);
		if (!strcmp("zygote", tcomm) && (chk_task->cred->uid == 0) && (ppid == 1))
			return 1;

		chk_task = chk_task->real_parent;
		pid = task_tgid_vnr(chk_task);
#if 0
		get_task_comm(tcomm, chk_task);
		printk("[RTFCTL] %s [pid(%d), ppid(%d), uid(%d)]\n", tcomm, pid, task_tgid_vnr(chk_task->real_parent), chk_task->cred->uid);
#endif
	};

	return 0;
}
#endif

static int rootfctrl_dentry_open(struct file *file, const struct cred *cred)
{
	pid_t pid;
	char tcomm[sizeof(current->comm)], name_buf[MAX_NAME_BUF_LEN];
	char *full_path = get_full_path(&file->f_path, NULL, name_buf);

	if (is_felica_RWP_file(full_path) || is_nfc_file(full_path)) {
		pid = task_tgid_vnr(current);
		get_task_comm(tcomm, current);

		RTFCTL_MSG("########## %s (felica/nfc) ##########\n", __FUNCTION__);
		RTFCTL_MSG("pid: %d (%s)\n", pid, tcomm);
		RTFCTL_MSG("uid: %d\n", current_uid());
		RTFCTL_MSG("euid: %d, suid: %d\n", current_euid(), current_suid());

#if (RTFCTL_RUN_MODE != RTFCTL_TRACKING_MODE)
		
		if (is_non_felica_root(current_uid(), pid) || (current_uid() != current_euid())) {
			printk("[RTFCTL] RType-1-1 <%s-%s (%d, %d, %d, %d)>\n", full_path, tcomm, pid,
				current_uid(), current_euid(), current_suid());
			return -EACCES;
		}
#endif
	} else if (is_felica_WP_file(full_path)) {
		pid = task_tgid_vnr(current);
		get_task_comm(tcomm, current);

		RTFCTL_MSG("########## %s (WP) ##########\n", __FUNCTION__);
		RTFCTL_MSG("pid: %d (%s)\n", pid, tcomm);

#if (RTFCTL_RUN_MODE != RTFCTL_TRACKING_MODE)
		
		if (pid == adbd_pid) {
			printk("[RTFCTL] RType-1-2 <%s-%s (%d)>\n", full_path, tcomm, pid);
			return -EACCES;
		}
#endif
	}
	return 0;
}

#if (RTFCTL_RUN_MODE == RTFCTL_TRACKING_MODE)
static int rootfctrl_file_permission(struct file *file, int mask)
{
	char name_buf[MAX_NAME_BUF_LEN];
	if ((mask & 2) && is_felica_file(get_full_path(&file->f_path, NULL, name_buf))) {
		RTFCTL_MSG("*****>>>>> %s <<<<<*****\n", __FUNCTION__);
		RTFCTL_MSG("mask: %d\n", mask);

		if (current_uid() == 0) {
			RTFCTL_MSG("Want to Rejected...\n");
			
		}
	}
	return 0;
}

static int rootfctrl_file_ioctrl(struct file *file, unsigned int cmd, unsigned long arg)
{
	char name_buf[MAX_NAME_BUF_LEN];
	if (is_felica_file(get_full_path(&file->f_path, NULL, name_buf))) {
		RTFCTL_MSG("*****>>>>> %s <<<<<*****\n", __FUNCTION__);
		RTFCTL_MSG("cmd: %x, arg: %lx\n", cmd, arg);

		if (current_uid() == 0) {
			RTFCTL_MSG("Want to Rejected...\n");
			
		}
	}
	return 0;
}

static int rootfctrl_file_fcntl(struct file *file, unsigned int cmd, unsigned long arg)
{
	char name_buf[MAX_NAME_BUF_LEN];
	if (is_felica_file(get_full_path(&file->f_path, NULL, name_buf))) {
		RTFCTL_MSG("*****>>>>> %s <<<<<*****\n", __FUNCTION__);
		RTFCTL_MSG("cmd: %x, arg: %lx\n", cmd, arg);

		if (current_uid() == 0) {
			RTFCTL_MSG("Want to Rejected...\n");
			
		}
	}
	return 0;
}

static int rootfctrl_file_set_fowner(struct file *file)
{
	char name_buf[MAX_NAME_BUF_LEN];
	if (is_felica_file(get_full_path(&file->f_path, NULL, name_buf))) {
		RTFCTL_MSG("*****>>>>> %s <<<<<*****\n", __FUNCTION__);
		if (current_uid() == 0) {
			RTFCTL_MSG("Want to Rejected...\n");
			
		}
	}
	return 0;
}
#endif

static int rootfctrl_path_unlink(struct path *dir, struct dentry *dentry)
{
	pid_t pid;
	char tcomm[sizeof(current->comm)], name_buf[MAX_NAME_BUF_LEN];
	char *full_path = get_full_path(dir, dentry, name_buf);

	if (is_felica_file(full_path) || is_nfc_file(full_path)) {
		pid = task_tgid_vnr(current);
		get_task_comm(tcomm, current);

		RTFCTL_MSG("########## %s ##########\n", __FUNCTION__);
		RTFCTL_MSG("pid: %d (%s)\n", pid, tcomm);

#if (RTFCTL_RUN_MODE != RTFCTL_TRACKING_MODE)
		
		printk("[RTFCTL] RType-2 <%s-%s (%d)>\n", full_path, tcomm, pid);
		return -EACCES;
#endif
	}
	return 0;
}

static int rootfctrl_path_mknod(struct path *dir, struct dentry *dentry, umode_t mode, unsigned int dev)
{
	pid_t pid;
	char tcomm[sizeof(current->comm)], name_buf[MAX_NAME_BUF_LEN];
	char *full_path = get_full_path(dir, dentry, name_buf);
	int ret, format = mode & S_IFMT;

	if (format == S_IFCHR || format == S_IFBLK) {
		dev_t dev_num;

		dev_num = new_decode_dev(dev);

		
		ret = is_felica_dev(MAJOR(dev_num), MINOR(dev_num), full_path);
		if (ret < 0) {
			pid = task_tgid_vnr(current);
			get_task_comm(tcomm, current);

			RTFCTL_MSG("########## %s(felica dev node) ##########\n", __FUNCTION__);
			RTFCTL_MSG("pid: %d (%s)\n", task_tgid_vnr(current), tcomm);
			RTFCTL_MSG("mode: %o\n", mode);
			RTFCTL_MSG("Felica Node major/minor wrong or Fake felica node\n");

#if (RTFCTL_RUN_MODE != RTFCTL_TRACKING_MODE)
			printk("[RTFCTL] RType-3-1-%d <%s (%o, %u, %u)-%s (%d)>\n", ret, full_path,
				mode, MAJOR(dev_num), MINOR(dev_num), tcomm, pid);
			return -EACCES;
#endif
		}

		
		ret = is_nfc_dev(MAJOR(dev_num), MINOR(dev_num), full_path);
		if (ret < 0) {
			pid = task_tgid_vnr(current);
			get_task_comm(tcomm, current);

			RTFCTL_MSG("########## %s(nfc dev node) ##########\n", __FUNCTION__);
			RTFCTL_MSG("pid: %d (%s)\n", task_tgid_vnr(current), tcomm);
			RTFCTL_MSG("mode: %o\n", mode);
			RTFCTL_MSG("Nfc Node major/minor wrong or Fake nfc node\n");

#if (RTFCTL_RUN_MODE != RTFCTL_TRACKING_MODE)
			printk("[RTFCTL] RType-3-2-%d <%s (%o, %u, %u)-%s (%d)>\n", ret, full_path,
				mode, MAJOR(dev_num), MINOR(dev_num), tcomm, pid);
			return -EACCES;
#endif
		}
	}
	else if (is_felica_file(full_path)) {
		pid = task_tgid_vnr(current);
		get_task_comm(tcomm, current);

		RTFCTL_MSG("########## %s(regular file) ##########\n", __FUNCTION__);
		RTFCTL_MSG("pid: %d (%s)\n", task_tgid_vnr(current), tcomm);
		RTFCTL_MSG("mode: %o\n", mode);
		RTFCTL_MSG("Can't create Felica file dynamically\n");

#if (RTFCTL_RUN_MODE != RTFCTL_TRACKING_MODE)
		
		printk("[RTFCTL] RType-3-3 <%s (%o)-%s (%d)>\n", full_path, mode, tcomm, pid);
		return -EACCES;
#endif
	}

	return 0;
}

static int rootfctrl_path_link(struct dentry *old_dentry, struct path *new_dir, struct dentry *new_dentry)
{
	pid_t pid;
	char tcomm[sizeof(current->comm)], name_buf[MAX_NAME_BUF_LEN];
	char *full_path;
	struct path path;

	path.mnt = new_dir->mnt;
	path.dentry = old_dentry;
	full_path = get_full_path(&path, NULL, name_buf);

	if (is_felica_file(full_path) || is_nfc_file(full_path)) {
		pid = task_tgid_vnr(current);
		get_task_comm(tcomm, current);

		RTFCTL_MSG("########## %s ##########\n", __FUNCTION__);
		RTFCTL_MSG("pid: %d (%s)\n", pid, tcomm);
		
		RTFCTL_MSG("new_dentry: %s -> old_dentry: %s\n", new_dentry->d_name.name, old_dentry->d_name.name);

#if (RTFCTL_RUN_MODE != RTFCTL_TRACKING_MODE)
		
		printk("[RTFCTL] RType-4 <%s-%s (%d)>\n", full_path, tcomm, pid);
		return -EACCES;
#endif
	}
	return 0;
}

static int rootfctrl_path_rename(struct path *old_dir, struct dentry *old_dentry,
                    struct path *new_dir, struct dentry *new_dentry)
{
	pid_t pid;
	char tcomm[sizeof(current->comm)];
	char old_name_buf[MAX_NAME_BUF_LEN], new_name_buf[MAX_NAME_BUF_LEN];
	char *old_full_path = get_full_path(old_dir, old_dentry, old_name_buf);
	char *new_full_path = get_full_path(new_dir, new_dentry, new_name_buf);

#if (RTFCTL_RUN_MODE == RTFCTL_TEST_MODE)
	if (is_felica_file(old_full_path) || is_nfc_file(old_full_path))
#else
	if (is_felica_file(old_full_path) || is_felica_file(new_full_path) ||
		is_nfc_file(old_full_path) || is_nfc_file(new_full_path))
#endif
	{
		pid = task_tgid_vnr(current);
		get_task_comm(tcomm, current);

		RTFCTL_MSG("########## %s ##########\n", __FUNCTION__);
		RTFCTL_MSG("pid: %d (%s)\n", pid, tcomm);

#if (RTFCTL_RUN_MODE != RTFCTL_TRACKING_MODE)
		
		printk("[RTFCTL] RType-5 <%s (%d)>\n", tcomm, pid);
		if (old_full_path)
			printk("[RTFCTL] RType-5 <old %s>\n", old_full_path);
		if (new_full_path)
			printk("[RTFCTL] RType-5 <new %s>\n", new_full_path);
		return -EACCES;
#endif
	}
	return 0;
}

static int rootfctrl_path_chmod(struct path *path, umode_t mode)
{
	pid_t pid;
	char tcomm[sizeof(current->comm)], name_buf[MAX_NAME_BUF_LEN];
	char *full_path = get_full_path(path, NULL, name_buf);

	if (!is_felica_mode_valid(mode, full_path) || !is_nfc_mode_valid(mode, full_path)) {
		pid = task_tgid_vnr(current);
		get_task_comm(tcomm, current);

		RTFCTL_MSG("########## %s ##########\n", __FUNCTION__);
		RTFCTL_MSG("pid: %d (%s)\n", pid, tcomm);

#if (RTFCTL_RUN_MODE != RTFCTL_TRACKING_MODE)
		printk("[RTFCTL] RType-6 <%s (%o)-%s (%d, %d)>\n", full_path, mode, tcomm, pid, current_uid());
		return -EACCES;
#endif
	}

	return 0;
}

static int rootfctrl_path_chown(struct path *path, uid_t uid, gid_t gid)
{
	pid_t pid;
	char tcomm[sizeof(current->comm)], name_buf[MAX_NAME_BUF_LEN];
	char *full_path = get_full_path(path, NULL, name_buf);
	int ret, ret2;

	ret = is_felica_owner_valid(uid, gid, full_path);
	ret2 = is_nfc_owner_valid(uid, gid, full_path);
	if ((ret < 0) || (ret2 < 0)) {
		pid = task_tgid_vnr(current);
		get_task_comm(tcomm, current);

		RTFCTL_MSG("########## %s ##########\n", __FUNCTION__);
		RTFCTL_MSG("pid: %d (%s)\n", pid, tcomm);
		RTFCTL_MSG("Change felica/nfc file UID/GID\n");

#if (RTFCTL_RUN_MODE != RTFCTL_TRACKING_MODE)
		printk("[RTFCTL] RType-7-1 <%s (%d, %d)-%s (%d, %d)>\n", full_path, uid, gid,
			tcomm, pid, current_uid());
		return -EACCES;
#endif
	} else if (ret == 0) {
		if ((uid != -1 && is_felica_uid(uid)) || (gid != -1 && is_felica_uid(gid))) {
			pid = task_tgid_vnr(current);
			get_task_comm(tcomm, current);

			RTFCTL_MSG("########## %s ##########\n", __FUNCTION__);
			RTFCTL_MSG("pid: %d (%s)\n", pid, tcomm);
			RTFCTL_MSG("file: %s\n", full_path);
			RTFCTL_MSG("Change other file to Felica UID/GID\n");

#if (RTFCTL_RUN_MODE != RTFCTL_TRACKING_MODE)
			if(!is_installd(pid, tcomm, current_uid())) {
				printk("[RTFCTL] RType-7-2 <%s (%d, %d)-%s (%d, %d) [%d]>\n", full_path, uid, gid,
					tcomm, pid, current_uid(), installd_pid);
				return -EACCES;
			}
#endif
		}
	}
	return 0;
}

#if (RTFCTL_RUN_MODE == RTFCTL_TRACKING_MODE)
static int rootfctrl_path_mkdir(struct path *dir, struct dentry *dentry, umode_t mode)
{
	char name_buf[MAX_NAME_BUF_LEN];
	if (is_felica_file(get_full_path(dir, dentry, name_buf))) {
		RTFCTL_MSG("*****>>>>> %s <<<<<*****\n", __FUNCTION__);
		RTFCTL_MSG("parent: %s\n", get_full_path(dir, NULL, name_buf));
		RTFCTL_MSG("dentry: %s\n", dentry->d_name.name);
		RTFCTL_MSG("mode: %o\n", mode);
		if (current_uid() == 0) {
			RTFCTL_MSG("Rejected...\n");
			
		}
	}
	return 0;
}
static int rootfctrl_path_rmdir(struct path *dir, struct dentry *dentry)
{
	char name_buf[MAX_NAME_BUF_LEN];
	if (is_felica_file(get_full_path(dir, dentry, name_buf))) {
		RTFCTL_MSG("*****>>>>> %s <<<<<*****\n", __FUNCTION__);
		RTFCTL_MSG("parent: %s\n", get_full_path(dir, NULL, name_buf));
		RTFCTL_MSG("dentry: %s\n", dentry->d_name.name);
		if (current_uid() == 0) {
			RTFCTL_MSG("Rejected...\n");
			
		}
	}
	return 0;
}

static int rootfctrl_path_truncate(struct path *path)
{
	char name_buf[MAX_NAME_BUF_LEN];
	if (is_felica_file(get_full_path(path, NULL, name_buf))) {
		RTFCTL_MSG("*****>>>>> %s <<<<<*****\n", __FUNCTION__);
		RTFCTL_MSG("path: %s\n", get_full_path(path, NULL, name_buf));
	}
	return 0;
}

static int rootfctrl_path_symlink(struct path *dir, struct dentry *dentry, const char *old_name)
{
	struct path path;

	path.mnt = dir->mnt;
	path.dentry = dentry;

	if (!strcmp(old_name, "../felica")) {
	
		RTFCTL_MSG("*****>>>>> %s <<<<<*****\n", __FUNCTION__);
		
		
		RTFCTL_MSG("dentry: %s\n", dentry->d_name.name);
		RTFCTL_MSG("old_name: %s\n", old_name);

		
		RTFCTL_MSG(KERN_INFO "Rejected...\n");
		
	}
	return 0;
}
#endif


static int rootfctrl_task_create(unsigned long clone_flags)
{
	pid_t ppid = task_tgid_vnr(current->real_parent);
	char tcomm[sizeof(current->comm)];

	get_task_comm(tcomm, current);

	if (zygote_pid == -1) {
		if (!strcmp("zygote", tcomm) && current_uid() == 0 && ppid == 1) {
			RTFCTL_MSG("########## %s ##########\n", __FUNCTION__);
			zygote_pid = task_tgid_vnr(current);
			printk("[RTFCTL] Current zg: %d\n", zygote_pid);
		}
	}

	if (flc_daemon_pid == -1) {
		if (!strcmp("felica_daemon", tcomm) && current_uid() == 0 && ppid == 1) {
			RTFCTL_MSG("########## %s ##########\n", __FUNCTION__);
			flc_daemon_pid = task_tgid_vnr(current);
			printk("[RTFCTL] Current fdaemon: %d\n", flc_daemon_pid);
		}
	}

	if (flc_agent_pid == -1) {
		if (!strcmp("felica_agent", tcomm) && current_uid() == 0 && ppid == 1) {
			RTFCTL_MSG("########## %s ##########\n", __FUNCTION__);
			flc_agent_pid = task_tgid_vnr(current);
			printk("[RTFCTL] Current fagent: %d\n", flc_agent_pid);
		}
	}

	if (installd_pid == -1) {
		if (!strcmp("installd", tcomm) && current_uid() == 0 && ppid == 1) {
			RTFCTL_MSG("########## %s ##########\n", __FUNCTION__);
			installd_pid = task_tgid_vnr(current);
			printk("[RTFCTL] Current isd: %d\n", installd_pid);
		}
	}

	if (!strcmp("adbd", tcomm) && ppid == 1) {
		RTFCTL_MSG("########## %s ##########\n", __FUNCTION__);
		adbd_pid = task_tgid_vnr(current);
		
	}

	return 0;
}

int rootfctrl_task_kill(struct task_struct *p, struct siginfo *info,
			int sig, u32 secid)
{
	char tcomm[sizeof(p->comm)];
	char tcomm2[sizeof(current->comm)];

	get_task_comm(tcomm2, current);
	get_task_comm(tcomm, p);

	
	if (!strcmp("zygote", tcomm) && (task_tgid_vnr(p) == zygote_pid)) {
		RTFCTL_MSG("########## %s ##########\n", __FUNCTION__);
		RTFCTL_MSG("pid: %d (%s) kill pid: %d (%s)\n", task_tgid_vnr(current), tcomm2, task_tgid_vnr(p), tcomm);
		RTFCTL_MSG("info: %x, isFromKernel: %d\n", (unsigned int)info, SI_FROMKERNEL(info));
		RTFCTL_MSG("sig: %d, secid: %d\n", sig, secid);
		printk("[RTFCTL] zg killed\n");
		zygote_pid = -1;
	}

	
	if (!strcmp("felica_daemon", tcomm) && (task_tgid_vnr(p) == flc_daemon_pid)) {
		RTFCTL_MSG("########## %s ##########\n", __FUNCTION__);
		RTFCTL_MSG("pid: %d (%s) kill pid: %d (%s)\n", task_tgid_vnr(current), tcomm2, task_tgid_vnr(p), tcomm);
		RTFCTL_MSG("info: %x, isFromKernel: %d\n", (unsigned int)info, SI_FROMKERNEL(info));
		RTFCTL_MSG("sig: %d, secid: %d\n", sig, secid);
		printk("[RTFCTL] fdaemon killed\n");
		flc_daemon_pid = -1;
	}

	
	if (!strcmp("felica_agent", tcomm) && (task_tgid_vnr(p) == flc_agent_pid)) {
		RTFCTL_MSG("########## %s ##########\n", __FUNCTION__);
		RTFCTL_MSG("pid: %d (%s) kill pid: %d (%s)\n", task_tgid_vnr(current), tcomm2, task_tgid_vnr(p), tcomm);
		RTFCTL_MSG("info: %x, isFromKernel: %d\n", (unsigned int)info, SI_FROMKERNEL(info));
		RTFCTL_MSG("sig: %d, secid: %d\n", sig, secid);
		printk("[RTFCTL] fagent killed\n");
		flc_agent_pid = -1;
	}

	
	if (!strcmp("installd", tcomm) && (task_tgid_vnr(p) == installd_pid)) {
		RTFCTL_MSG("########## %s ##########\n", __FUNCTION__);
		RTFCTL_MSG("pid: %d (%s) kill pid: %d (%s)\n", task_tgid_vnr(current), tcomm2, task_tgid_vnr(p), tcomm);
		RTFCTL_MSG("info: %x, isFromKernel: %d\n", (unsigned int)info, SI_FROMKERNEL(info));
		RTFCTL_MSG("sig: %d, secid: %d\n", sig, secid);
		printk("[RTFCTL] isd killed\n");
		installd_pid = -1;
	}

	return 0;
}

static int rootfctrl_task_fix_setuid (struct cred *new, const struct cred *old, int flags)
{
	pid_t pid, ppid;
	char tcomm[sizeof(current->comm)];
	char ptcomm[sizeof(current->real_parent->comm)];

	pid = task_tgid_vnr(current);
	ppid = task_tgid_vnr(current->real_parent);
	get_task_comm(tcomm, current);
	get_task_comm(ptcomm, current->real_parent);

#if (RTFCTL_RUN_MODE == RTFCTL_TRACKING_MODE)
	if (is_felica_uid(new->uid) || is_nfc_uid(new->uid)) {
		RTFCTL_MSG("########## %s ##########\n", __FUNCTION__);
		RTFCTL_MSG("old->uid = %d, old->gid = %d\n", old->uid, old->gid);
		RTFCTL_MSG("old->suid = %d, old->sgid = %d\n", old->suid, old->sgid);
		RTFCTL_MSG("old->euid = %d, old->egid = %d\n", old->euid, old->egid);

		RTFCTL_MSG("new->uid = %d, new->gid = %d\n", new->uid, new->gid);
		RTFCTL_MSG("new->suid = %d, new->sgid = %d\n", new->suid, new->sgid);
		RTFCTL_MSG("new->euid = %d, new->egid = %d\n", new->euid, new->egid);
		RTFCTL_MSG("Zygote pid: %d\n", zygote_pid);
		RTFCTL_MSG("Installd pid: %d\n", installd_pid);
	}
#endif

	if ((!is_felica_uid(old->uid) && is_felica_uid(new->uid)) ||
		(!is_nfc_uid(old->uid) && is_nfc_uid(new->uid)) ||
		((old->uid != UID_FELICA_RW_CTRL) && (new->uid == UID_FELICA_RW_CTRL))) {
		RTFCTL_MSG("########## %s ##########\n", __FUNCTION__);
		RTFCTL_MSG("pid: %d (%s) ppid: %d (%s)\n", pid, tcomm, ppid, ptcomm);
		RTFCTL_MSG("Old UID: %d -> New UID: %d\n", old->uid, new->uid);
		RTFCTL_MSG("current zygote_pid: %d, installd_pid: %d\n", zygote_pid, installd_pid);
#if (RTFCTL_RUN_MODE != RTFCTL_TRACKING_MODE)
		if(!is_zygote(current->real_parent) &&
			!is_installd(pid, tcomm, current_uid()) && !is_installd(ppid, ptcomm, parent_uid())) {
			printk("[RTFCTL] RType-8-1 <%d-> %d-%s (%d)-%s (%d) [%d, %d]>\n", old->uid, new->uid,
					tcomm, pid, ptcomm, ppid, zygote_pid, installd_pid);
			return -EACCES;
		}
#endif
	}
	if ((!is_felica_uid(old->gid) && is_felica_uid(new->gid)) ||
		(!is_nfc_uid(old->gid) && is_nfc_uid(new->gid)) ||
		((old->gid != UID_FELICA_RW_CTRL) && (new->gid == UID_FELICA_RW_CTRL))) {
		RTFCTL_MSG("########## %s ##########\n", __FUNCTION__);
		RTFCTL_MSG("pid: %d (%s) ppid: %d (%s)\n", pid, tcomm, ppid, ptcomm);
		RTFCTL_MSG("Old GID: %d -> New GID: %d\n", old->gid, new->gid);
		RTFCTL_MSG("current zygote_pid: %d, installd_pid: %d\n", zygote_pid, installd_pid);
#if (RTFCTL_RUN_MODE != RTFCTL_TRACKING_MODE)
		if(!is_zygote(current->real_parent) &&
			!is_installd(pid, tcomm, current_uid()) && !is_installd(ppid, ptcomm, parent_uid())) {
			printk("[RTFCTL] RType-8-2 <%d-> %d-%s (%d)-%s (%d) [%d, %d]>\n", old->gid, new->gid,
					tcomm, pid, ptcomm, ppid, zygote_pid, installd_pid);
			return -EACCES;
		}
#endif
	}

	return 0;
}

static struct security_operations rootfctrl_ops = {
	.name   =   "rootfctrl",
	.dentry_open        =       rootfctrl_dentry_open,
#if (RTFCTL_RUN_MODE == RTFCTL_TRACKING_MODE)
	.file_set_fowner    =       rootfctrl_file_set_fowner,
	.file_fcntl         =       rootfctrl_file_fcntl,
	.file_ioctl         =       rootfctrl_file_ioctrl,
	.file_permission    =       rootfctrl_file_permission,

	.path_mkdir         =       rootfctrl_path_mkdir,
	.path_rmdir         =       rootfctrl_path_rmdir,
	.path_truncate      =       rootfctrl_path_truncate,
	.path_symlink       =       rootfctrl_path_symlink,
#endif
	.path_unlink        =       rootfctrl_path_unlink,
	.path_mknod         =       rootfctrl_path_mknod,
	.path_link          =       rootfctrl_path_link,
	.path_rename        =       rootfctrl_path_rename,
	.path_chmod         =       rootfctrl_path_chmod,
	.path_chown         =       rootfctrl_path_chown,

	.task_create        =       rootfctrl_task_create,
	.task_kill          =       rootfctrl_task_kill,
	.task_fix_setuid    =       rootfctrl_task_fix_setuid,
};

static struct security_operations rootfctrl_recvy_ops = {
	.name	= "rootfctrl",
};

int board_mfg_mode(void);

static __init int rootfctrl_init(void)
{
	if (!security_module_enable(&rootfctrl_ops)) {
		printk("[RTFCTL][ERR] rootfctrl is not enabled!");
		return 0;
	}

	RTFCTL_MSG("Initializing.....\n");

	if (board_mfg_mode() == 2) {
		if (register_security(&rootfctrl_recvy_ops))
			panic("rootfctrl: Unable to register with kernel .\n");
	}
	else {
		if (register_security(&rootfctrl_ops))
			panic("rootfctrl: Unable to register with kernel.\n");
	}

	return 0;
}

security_initcall(rootfctrl_init);
