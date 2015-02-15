/* include/asm/mach-msm/htc_acoustic_alsa.h
 *
 * Copyright (C) 2012 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/wakelock.h>
#include <linux/sched.h>

#ifndef _ARCH_ARM_MACH_MSM_HTC_ACOUSTIC_QCT_ALSA_H_
#define _ARCH_ARM_MACH_MSM_HTC_ACOUSTIC_QCT_ALSA_H_

#define AUD_HW_NUM			8
#define HTC_AUDIO_TPA2051	0x01
#define HTC_AUDIO_TPA2026	0x02
#define HTC_AUDIO_TPA2028	0x04
#define HTC_AUDIO_A1028		0x08
#define HTC_AUDIO_TPA6185 	0x10
#define HTC_AUDIO_RT5501 	0x20
#define HTC_AUDIO_TFA9887 	0x40
#define HTC_AUDIO_TFA9887L 	0x80

#define ACOUSTIC_IOCTL_MAGIC 'p'
#define ACOUSTIC_SET_Q6_EFFECT		_IOW(ACOUSTIC_IOCTL_MAGIC, 43, unsigned)
#define ACOUSTIC_GET_HTC_REVISION	_IOW(ACOUSTIC_IOCTL_MAGIC, 44, unsigned)
#define ACOUSTIC_GET_HW_COMPONENT	_IOW(ACOUSTIC_IOCTL_MAGIC, 45, unsigned)
#define ACOUSTIC_GET_DMIC_INFO   	_IOW(ACOUSTIC_IOCTL_MAGIC, 46, unsigned)
#define ACOUSTIC_UPDATE_BEATS_STATUS	_IOW(ACOUSTIC_IOCTL_MAGIC, 47, unsigned)
#define ACOUSTIC_UPDATE_LISTEN_NOTIFICATION	_IOW(ACOUSTIC_IOCTL_MAGIC, 48, unsigned)
#define ACOUSTIC_GET_HW_REVISION	_IOR(ACOUSTIC_IOCTL_MAGIC, 49, struct device_info)
#define ACOUSTIC_CONTROL_WAKELOCK	_IOW(ACOUSTIC_IOCTL_MAGIC, 77, unsigned)
#define ACOUSTIC_DUMMY_WAKELOCK 	_IOW(ACOUSTIC_IOCTL_MAGIC, 78, unsigned)
#define ACOUSTIC_AMP_CTRL		_IOR(ACOUSTIC_IOCTL_MAGIC, 50, unsigned)
#define ACOUSTIC_GET_MID		_IOW(ACOUSTIC_IOCTL_MAGIC, 51, unsigned)
#define ACOUSTIC_RAMDUMP		_IOW(ACOUSTIC_IOCTL_MAGIC, 99, unsigned)
#define ACOUSTIC_KILL_PID		_IOW(ACOUSTIC_IOCTL_MAGIC, 88, unsigned)
#define ACOUSTIC_UPDATE_DQ_STATUS	_IOW(ACOUSTIC_IOCTL_MAGIC, 52, unsigned)

#define AUD_AMP_SLAVE_ALL	0xffff

enum HTC_FEATURE {
	HTC_Q6_EFFECT = 0,
	HTC_AUD_24BIT,
};

enum SPK_AMP_TYPE {
	SPK_AMP_RIGHT = 0,
	SPK_AMP_LEFT,
	SPK_AMP_MAX,
};

enum ACOU_AMP_CTRL {
	AMP_READ = 0,
	AMP_WRITE,
};

enum AMP_TYPE {
	AMP_HEADPONE = 0,
	AMP_SPEAKER,
	AMP_RECEIVER,
};

enum HS_NOTIFY_TYPE {
	HS_AMP_N = 0,
	HS_CODEC_N,
	HS_N_MAX,
};

struct hw_component {
	const char *name;
	const unsigned int id;
};

struct amp_register_s {
	int (*aud_amp_f)(int, int);
	struct file_operations* fops;
};

struct amp_ctrl {
	enum ACOU_AMP_CTRL ctrl;
	enum AMP_TYPE type;
	unsigned short slave;
	unsigned int reg;
	unsigned int val;
};

struct acoustic_ops {
	void (*set_q6_effect)(int mode);
	int (*get_htc_revision)(void);
	int (*get_hw_component)(void);
	char* (*get_mid)(void);
	int (*enable_digital_mic)(void);
	int (*enable_24b_audio)(void);
	int (*get_q6_effect) (void);
};

struct hs_notify_t {
	int used;
	void *private_data;
	int (*callback_f)(void*,int);
};

void htc_acoustic_register_ops(struct acoustic_ops *ops);
void htc_acoustic_register_hs_amp(int (*aud_hs_amp_f)(int, int), struct file_operations* ops);
int htc_acoustic_hs_amp_ctrl(int on, int dsp);
void htc_acoustic_register_spk_amp(enum SPK_AMP_TYPE type,int (*aud_spk_amp_f)(int, int), struct file_operations* ops);
int htc_acoustic_spk_amp_ctrl(enum SPK_AMP_TYPE type,int on, int dsp);
int htc_acoustic_query_feature(enum HTC_FEATURE feature);
void htc_acoustic_register_hs_notify(enum HS_NOTIFY_TYPE type, struct hs_notify_t *notify);

struct amp_power_ops {
	void (*set_amp_power_enable)(bool enable);
};

void htc_amp_power_register_ops(struct amp_power_ops *ops);
void htc_amp_power_enable(bool enable);
#endif

