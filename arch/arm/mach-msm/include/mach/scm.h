/* Copyright (c) 2010-2013, The Linux Foundation. All rights reserved.
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
#ifndef __MACH_SCM_H
#define __MACH_SCM_H

#define SCM_SVC_BOOT			0x1
#define SCM_SVC_PIL			0x2
#define SCM_SVC_UTIL			0x3
#define SCM_SVC_TZ			0x4
#define SCM_SVC_IO			0x5
#define SCM_SVC_INFO			0x6
#define SCM_SVC_SSD			0x7
#define SCM_SVC_FUSE			0x8
#define SCM_SVC_PWR			0x9
#define SCM_SVC_MP			0xC
#define SCM_SVC_DCVS			0xD
#define SCM_SVC_ES			0x10
#define SCM_SVC_TZSCHEDULER		0xFC
#define SCM_SVC_OEM				0xFE

#define TZ_HTC_SVC_READ_SIMLOCK_MASK	0x0D
#define TZ_HTC_SVC_SIMLOCK_UNLOCK		0x0E
#define TZ_HTC_SVC_GET_SECURITY_LEVEL	0x10
#define TZ_HTC_SVC_MEMPROT				0x15
#define TZ_HTC_SVC_LOG_OPERATOR			0x16
#define TZ_HTC_SVC_ACCESS_ITEM			0x1A
#define TZ_HTC_SVC_3RD_PARTY			0x1B

#define ITEM_MDM9K_SERIAL		0
#define ITEM_CRYPTO_RAMDUMP		1
#define ITEM_ENCRYPT_RAMDUMP	3

#define ITEM_KEYBOX_PROVISION	0x11
#define ITEM_KEYBOX_DATA		0x21
#define ITEM_DEVICE_ID			0x22
#define ITEM_RAND_DATA			0x23
#define ITEM_VALIDATE_KEYBOX	0x26
#define ITEM_READ_MEM			0x28
#define ITEM_CPRMKEY_ADDR		0x31
#define ITEM_CPRMKEY_DATA		0x32
#define ITEM_SD_KEY_ENCRYPT		0x33
#define ITEM_SD_KEY_DECRYPT		0x34
#define ITEM_SEC_ATS			0x39
#define ITEM_REMOTE_MSG         0x3A
#define ITEM_GDRIVE_DATA        0x3C
#define ITEM_VOUCHER_SIG_DATA   0x3E

typedef struct {
	u8 enable;
	u8 chk_cln;
	u32 addr;
	u32 len;
} mem_chk_t;
#define SCM_FUSE_READ			0x7

#define DEFINE_SCM_BUFFER(__n) \
static char __n[PAGE_SIZE] __aligned(PAGE_SIZE);

#define SCM_BUFFER_SIZE(__buf)	sizeof(__buf)

#define SCM_BUFFER_PHYS(__buf)	virt_to_phys(__buf)

#ifdef CONFIG_MSM_SCM
extern int scm_call(u32 svc_id, u32 cmd_id, const void *cmd_buf, size_t cmd_len,
		void *resp_buf, size_t resp_len);

extern int scm_call_noalloc(u32 svc_id, u32 cmd_id, const void *cmd_buf,
		size_t cmd_len, void *resp_buf, size_t resp_len,
		void *scm_buf, size_t scm_buf_size);


extern s32 scm_call_atomic1(u32 svc, u32 cmd, u32 arg1);
extern s32 scm_call_atomic2(u32 svc, u32 cmd, u32 arg1, u32 arg2);
extern s32 scm_call_atomic3(u32 svc, u32 cmd, u32 arg1, u32 arg2, u32 arg3);
extern s32 scm_call_atomic4_3(u32 svc, u32 cmd, u32 arg1, u32 arg2, u32 arg3,
		u32 arg4, u32 *ret1, u32 *ret2);

#define SCM_VERSION(major, minor) (((major) << 16) | ((minor) & 0xFF))

extern u32 scm_get_version(void);
extern int scm_is_call_available(u32 svc_id, u32 cmd_id);
extern int scm_get_feat_version(u32 feat);
extern int secure_read_simlock_mask(void);
extern int secure_simlock_unlock(unsigned int unlock, unsigned char *code);
extern int secure_get_security_level(void);
extern int secure_memprot(void);
extern int secure_log_operation(unsigned int address, unsigned int size,
		unsigned int buf_addr, unsigned int buf_len, int revert);
extern int secure_access_item(unsigned int is_write, unsigned int id, unsigned int buf_len, unsigned char *buf);
extern void scm_flush_range(unsigned long start, unsigned long end);
extern void scm_inv_range(unsigned long start, unsigned long end);

#else

static inline int scm_call(u32 svc_id, u32 cmd_id, const void *cmd_buf,
		size_t cmd_len, void *resp_buf, size_t resp_len)
{
	return 0;
}

static inline int scm_call_noalloc(u32 svc_id, u32 cmd_id,
		const void *cmd_buf, size_t cmd_len, void *resp_buf,
		size_t resp_len, void *scm_buf, size_t scm_buf_size)
{
	return 0;
}

static inline s32 scm_call_atomic1(u32 svc, u32 cmd, u32 arg1)
{
	return 0;
}

static inline s32 scm_call_atomic2(u32 svc, u32 cmd, u32 arg1, u32 arg2)
{
	return 0;
}

static inline s32 scm_call_atomic3(u32 svc, u32 cmd, u32 arg1, u32 arg2,
		u32 arg3)
{
	return 0;
}

static inline s32 scm_call_atomic4_3(u32 svc, u32 cmd, u32 arg1, u32 arg2,
		u32 arg3, u32 arg4, u32 *ret1, u32 *ret2)
{
	return 0;
}

static inline u32 scm_get_version(void)
{
	return 0;
}

static inline int scm_is_call_available(u32 svc_id, u32 cmd_id)
{
	return 0;
}

static inline int scm_get_feat_version(u32 feat)
{
	return 0;
}

#endif
#endif
