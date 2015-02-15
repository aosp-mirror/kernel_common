/* arch/arm/mach-msm/htc_acoustic_alsa.c
 *
 * Copyright (C) 2012 HTC Corporation
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

#include <linux/device.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/mm.h>
#include <linux/gfp.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <mach/htc_acoustic_alsa.h>
#include <mach/htc_headset_mgr.h>

struct device_info {
	unsigned pcb_id;
	unsigned sku_id;
};

#define D(fmt, args...) printk(KERN_INFO "[AUD] htc-acoustic: "fmt, ##args)
#define E(fmt, args...) printk(KERN_ERR "[AUD] htc-acoustic: "fmt, ##args)

static struct mutex api_lock;
static struct acoustic_ops default_acoustic_ops;
static struct acoustic_ops *the_ops = &default_acoustic_ops;
static struct switch_dev sdev_beats;
static struct switch_dev sdev_dq;
static struct switch_dev sdev_listen_notification;

static struct mutex hs_amp_lock;
static struct amp_register_s hs_amp = {NULL, NULL};

static struct mutex spk_amp_lock;
static struct amp_register_s spk_amp[SPK_AMP_MAX] = {{NULL}};

static struct amp_power_ops default_amp_power_ops = {NULL};
static struct amp_power_ops *the_amp_power_ops = &default_amp_power_ops;

static struct wake_lock htc_acoustic_wakelock;
static struct wake_lock htc_acoustic_wakelock_timeout;
static struct wake_lock htc_acoustic_dummy_wakelock;
static struct hs_notify_t hs_plug_nt[HS_N_MAX] = {{0,NULL,NULL}};
static struct mutex hs_nt_lock;

static int hs_amp_open(struct inode *inode, struct file *file);
static int hs_amp_release(struct inode *inode, struct file *file);
static long hs_amp_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static struct file_operations hs_def_fops = {
	.owner = THIS_MODULE,
	.open = hs_amp_open,
	.release = hs_amp_release,
	.unlocked_ioctl = hs_amp_ioctl,
};

static struct miscdevice hs_amp_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aud_hs_amp",
	.fops = &hs_def_fops,
};

struct hw_component HTC_AUD_HW_LIST[AUD_HW_NUM] = {
	{
		.name = "TPA2051",
		.id = HTC_AUDIO_TPA2051,
	},
	{
		.name = "TPA2026",
		.id = HTC_AUDIO_TPA2026,
	},
	{
		.name = "TPA2028",
		.id = HTC_AUDIO_TPA2028,
	},
	{
		.name = "A1028",
		.id = HTC_AUDIO_A1028,
	},
	{
		.name = "TPA6185",
		.id = HTC_AUDIO_TPA6185,
	},
	{
		.name = "RT5501",
		.id = HTC_AUDIO_RT5501,
	},
	{
		.name = "TFA9887",
		.id = HTC_AUDIO_TFA9887,
	},
	{
		.name = "TFA9887L",
		.id = HTC_AUDIO_TFA9887L,
	},
};
EXPORT_SYMBOL(HTC_AUD_HW_LIST);


#include <linux/atomic.h>
#include <linux/wait.h>
#define HTC_NSD_IOCTL_MAGIC 'n'
#define HTC_NSD_READ_ADSP       _IOW(HTC_NSD_IOCTL_MAGIC, 40, unsigned)
#define HTC_NSD_RELEASE_READ    _IOW(HTC_NSD_IOCTL_MAGIC, 41, unsigned)

enum nsd_state_type {
	AUDIO_NSD_CLOSED,
	AUDIO_NSD_OPENED,
	AUDIO_NSD_FAIL,
};

#define DETECT_RESULT_PAYLOAD_SIZE	8
typedef struct {
	uint32_t flag;
	uint32_t master_vol;
	uint32_t left_vol;
	uint32_t right_vol;
	uint32_t in_event_seq;
	uint32_t out_event_seq;
	uint32_t obj_id;
	uint32_t topology_id;
} detect_result_payload;

typedef struct {
	struct list_head list;
	detect_result_payload payload;
} nsd_event_node;

#define NSD_MAX_Q_LEN   10
typedef struct {
	enum nsd_state_type state;
	struct list_head    event_queue;
	struct list_head    free_event_queue;
	wait_queue_head_t   cmd_wait;
	spinlock_t          lock;
	struct wake_lock    wakelock;
	void*               memory_chunk;
} audio_nsd_info_def;
audio_nsd_info_def audio_nsd_info;

void htc_nsd_update(void* payload) {
	unsigned long flags;
	nsd_event_node *event_node = NULL;
	D("htc_nsd_update, flag 0x%x, (%d, %d, %d), (%d, %d), %x, %x\n",
			*(uint32_t*)(payload),*(uint32_t*)((uint32_t*)payload + 1),
			*(uint32_t*)((uint32_t*)payload + 2),*(uint32_t*)((uint32_t*)payload + 3),
			*(uint32_t*)((uint32_t*)payload + 4),*(uint32_t*)((uint32_t*)payload + 5),
			*(uint32_t*)((uint32_t*)payload + 6),*(uint32_t*)((uint32_t*)payload + 7));

	if (audio_nsd_info.state == AUDIO_NSD_OPENED) {
		wake_lock_timeout(&audio_nsd_info.wakelock, 1*HZ);
		spin_lock_irqsave(&audio_nsd_info.lock, flags);
		
		if (list_empty(&audio_nsd_info.free_event_queue)) {
			spin_unlock_irqrestore(&audio_nsd_info.lock, flags);
			E("htc_nsd_update, free queue is empty");
			return;
		}
		event_node = list_first_entry(&audio_nsd_info.free_event_queue,
				nsd_event_node,
				list);
		list_del(&event_node->list);

		memcpy((void*)&event_node->payload, payload, sizeof(detect_result_payload));

		list_add_tail(&event_node->list,
				&audio_nsd_info.event_queue);

		spin_unlock_irqrestore(&audio_nsd_info.lock, flags);
		wake_up(&audio_nsd_info.cmd_wait);
	} else {
		E("htc_nsd_update Fail, status is %d\n", audio_nsd_info.state);
	}
}
EXPORT_SYMBOL(htc_nsd_update);

static int htc_nsd_open(struct inode *inode, struct file *file)
{
	int i = 0;
	int offset = 0;
	int ret = 0 ;
	unsigned long flags;
	nsd_event_node* event_node = NULL;

	pr_info("%s\n", __func__);

	
	spin_lock_irqsave(&audio_nsd_info.lock, flags);
	INIT_LIST_HEAD(&audio_nsd_info.event_queue);
	INIT_LIST_HEAD(&audio_nsd_info.free_event_queue);

	
	if (audio_nsd_info.state == AUDIO_NSD_CLOSED) {
		audio_nsd_info.memory_chunk = kmalloc(NSD_MAX_Q_LEN * sizeof(nsd_event_node), GFP_KERNEL);

		if (audio_nsd_info.memory_chunk != NULL) {
			for (i = 0; i < NSD_MAX_Q_LEN; i++) {
				event_node = audio_nsd_info.memory_chunk + offset;
				list_add_tail(&event_node->list, &audio_nsd_info.free_event_queue);
				offset = offset + sizeof(nsd_event_node);
			}
			file->private_data = &audio_nsd_info;
			audio_nsd_info.state = AUDIO_NSD_OPENED;
		} else {
			pr_err("%s: No memory for IO buffers\n", __func__);
			ret = -ENOMEM;
		}
	}
	spin_unlock_irqrestore(&audio_nsd_info.lock, flags);
	return ret;
}

static int htc_nsd_release(struct inode *inode, struct file *file)
{
	struct list_head *ptr = NULL;
	struct list_head *next = NULL;
	nsd_event_node *event_node = NULL;
	unsigned long flags;
	audio_nsd_info_def *audio = file->private_data;

	pr_info("%s\n", __func__);
	spin_lock_irqsave(&audio_nsd_info.lock, flags);
	if (audio->state == AUDIO_NSD_OPENED) {
		
		list_for_each_safe(ptr, next, &audio->event_queue) {
			event_node = list_entry(ptr, nsd_event_node, list);
			list_del(&event_node->list);
		}
		list_for_each_safe(ptr, next, &audio->free_event_queue) {
			event_node = list_entry(ptr, nsd_event_node, list);
			list_del(&event_node->list);
		}
	}
	audio->state = AUDIO_NSD_CLOSED;
	spin_unlock_irqrestore(&audio_nsd_info.lock, flags);
	wake_up(&audio_nsd_info.cmd_wait);
	kfree(audio->memory_chunk);
	audio->memory_chunk = NULL;
	return 0;
}

static long
htc_nsd_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	unsigned long flags;
	switch (cmd) {
	case HTC_NSD_READ_ADSP: {
		wait_queue_t wait;
		nsd_event_node *event_node = NULL;
		detect_result_payload tmp_payload = {0};
		init_waitqueue_entry(&wait, current);
		add_wait_queue(&audio_nsd_info.cmd_wait, &wait);
		set_current_state(TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&audio_nsd_info.lock, flags);
		if (list_empty(&audio_nsd_info.event_queue)) {
			spin_unlock_irqrestore(&audio_nsd_info.lock, flags);
			schedule();
			spin_lock_irqsave(&audio_nsd_info.lock, flags);
		}
		set_current_state(TASK_RUNNING);

		if (list_empty(&audio_nsd_info.event_queue)) {
			tmp_payload.flag = 0;
		} else {
			event_node = list_first_entry(&audio_nsd_info.event_queue,
					nsd_event_node, list);
			memcpy(&tmp_payload, (void*)&event_node->payload, sizeof(detect_result_payload));
			list_del(&event_node->list);
			list_add_tail(&event_node->list, &audio_nsd_info.free_event_queue);
		}
		spin_unlock_irqrestore(&audio_nsd_info.lock, flags);
		D("htc_nsd_ioctl, receive %x\n", tmp_payload.flag);
		if (copy_to_user((void *) arg, (void*)&tmp_payload, sizeof(detect_result_payload))) {
			E("htc_nsd_ioctl - HTC_NSD_READ_ADSP: copy to user failed\n");
			rc = -EINVAL;
		}
		remove_wait_queue(&audio_nsd_info.cmd_wait, &wait);
		break;
	}
	case HTC_NSD_RELEASE_READ: {
		uint32_t payload[DETECT_RESULT_PAYLOAD_SIZE] = {0};
		payload[0] = 0xFFFF0000;
		htc_nsd_update(&payload);
		break;
	}
	default: {
		E("htc_nsd_ioctl - Non Support ioctl cmd\n");
		break;
	}

	}
	return rc;
}

static struct file_operations htc_nsd_def_fops = {
	.owner = THIS_MODULE,
	.open = htc_nsd_open,
	.release = htc_nsd_release,
	.unlocked_ioctl = htc_nsd_ioctl,
};

static struct miscdevice htc_nsd_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htc_nsd_misc",
	.fops = &htc_nsd_def_fops,
};

static int nsd_init(void) {
	int ret = 0;
	spin_lock_init(&audio_nsd_info.lock);
	init_waitqueue_head(&audio_nsd_info.cmd_wait);
	wake_lock_init(&audio_nsd_info.wakelock, WAKE_LOCK_SUSPEND, "wakelock");

	ret = misc_register(&htc_nsd_misc);
	if (ret < 0) {
		pr_err("failed to register htc_nsd_misc misc device!\n");
		audio_nsd_info.state = AUDIO_NSD_FAIL;
		return ret;
	}
	audio_nsd_info.state = AUDIO_NSD_CLOSED;
	return ret;
}

extern unsigned int system_rev;
void htc_acoustic_register_spk_amp(enum SPK_AMP_TYPE type,int (*aud_spk_amp_f)(int, int), struct file_operations* ops)
{
	mutex_lock(&spk_amp_lock);

	if(type < SPK_AMP_MAX) {
		spk_amp[type].aud_amp_f = aud_spk_amp_f;
		spk_amp[type].fops = ops;
	}

	mutex_unlock(&spk_amp_lock);
}

int htc_acoustic_spk_amp_ctrl(enum SPK_AMP_TYPE type,int on, int dsp)
{
	int ret = 0;
	mutex_lock(&spk_amp_lock);

	if(type < SPK_AMP_MAX) {
		if(spk_amp[type].aud_amp_f)
			ret = spk_amp[type].aud_amp_f(on,dsp);
	}

	mutex_unlock(&spk_amp_lock);

	return ret;
}

int htc_acoustic_hs_amp_ctrl(int on, int dsp)
{
	int ret = 0;

	mutex_lock(&hs_amp_lock);
	if(hs_amp.aud_amp_f)
		ret = hs_amp.aud_amp_f(on,dsp);
	mutex_unlock(&hs_amp_lock);
	return ret;
}

void htc_acoustic_register_hs_amp(int (*aud_hs_amp_f)(int, int), struct file_operations* ops)
{
	mutex_lock(&hs_amp_lock);
	hs_amp.aud_amp_f = aud_hs_amp_f;
	hs_amp.fops = ops;
	mutex_unlock(&hs_amp_lock);
}

void htc_acoustic_register_hs_notify(enum HS_NOTIFY_TYPE type, struct hs_notify_t *notify)
{
	if(notify == NULL)
		return;

	mutex_lock(&hs_nt_lock);
	if(hs_plug_nt[type].used) {
		pr_err("%s: hs notification %d is reigstered\n",__func__,(int)type);
	} else {
		hs_plug_nt[type].private_data = notify->private_data;
		hs_plug_nt[type].callback_f = notify->callback_f;
		hs_plug_nt[type].used = 1;
	}
	mutex_unlock(&hs_nt_lock);
}

void htc_acoustic_register_ops(struct acoustic_ops *ops)
{
        D("acoustic_register_ops \n");
	mutex_lock(&api_lock);
	the_ops = ops;
	mutex_unlock(&api_lock);
}

int htc_acoustic_query_feature(enum HTC_FEATURE feature)
{
	int ret = -1;
	mutex_lock(&api_lock);
	switch(feature) {
		case HTC_Q6_EFFECT:
			if(the_ops && the_ops->get_q6_effect)
				ret = the_ops->get_q6_effect();
			break;
		case HTC_AUD_24BIT:
			if(the_ops && the_ops->enable_24b_audio)
				ret = the_ops->enable_24b_audio();
			break;
		default:
			break;

	};
	mutex_unlock(&api_lock);
	return ret;
}

static int acoustic_open(struct inode *inode, struct file *file)
{
	D("open\n");
	return 0;
}

static int acoustic_release(struct inode *inode, struct file *file)
{
	D("release\n");
	return 0;
}

static long
acoustic_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0;
	int hw_rev = 0;
	int mode = -1;
	char *mid;
	mutex_lock(&api_lock);
	switch (cmd) {
	case ACOUSTIC_SET_Q6_EFFECT: {
		if (copy_from_user(&mode, (void *)arg, sizeof(mode))) {
			rc = -EFAULT;
			break;
		}
		D("Set Q6 Effect : %d\n", mode);
		if (mode < -1 || mode > 1) {
			E("unsupported Q6 mode: %d\n", mode);
			rc = -EINVAL;
			break;
		}
		if (the_ops->set_q6_effect)
			the_ops->set_q6_effect(mode);
		break;
	}
	case ACOUSTIC_GET_HTC_REVISION:
		if (the_ops->get_htc_revision)
			hw_rev = the_ops->get_htc_revision();
		else
			hw_rev = 1;

		D("Audio HW revision:  %u\n", hw_rev);
		if(copy_to_user((void *)arg, &hw_rev, sizeof(hw_rev))) {
			E("acoustic_ioctl: copy to user failed\n");
			rc = -EINVAL;
		}
		break;
	case ACOUSTIC_GET_HW_COMPONENT:
		if (the_ops->get_hw_component)
			rc = the_ops->get_hw_component();

		D("support components: 0x%x\n", rc);
		if(copy_to_user((void *)arg, &rc, sizeof(rc))) {
			E("acoustic_ioctl: copy to user failed\n");
			rc = -EINVAL;
		}
		break;
	case ACOUSTIC_GET_DMIC_INFO:
		if (the_ops->enable_digital_mic)
			rc = the_ops->enable_digital_mic();

		D("support components: 0x%x\n", rc);
		if(copy_to_user((void *)arg, &rc, sizeof(rc))) {
			E("acoustic_ioctl: copy to user failed\n");
			rc = -EINVAL;
		}
		break;
	case ACOUSTIC_GET_MID:
		if (the_ops->get_mid)
			mid = the_ops->get_mid();

		D("get mid: %s\n", mid);
		if(copy_to_user((void *)arg, mid, strlen(mid))) {
			E("acoustic_ioctl: copy to user failed\n");
			rc = -EINVAL;
		}
		break;
	case ACOUSTIC_UPDATE_BEATS_STATUS: {
		int new_state = -1;

		if (copy_from_user(&new_state, (void *)arg, sizeof(new_state))) {
			rc = -EFAULT;
			break;
		}
		D("Update Beats Status : %d\n", new_state);
		if (new_state < -1 || new_state > 1) {
			E("Invalid Beats status update");
			rc = -EINVAL;
			break;
		}

		sdev_beats.state = -1;
		switch_set_state(&sdev_beats, new_state);
		break;
	}
	case ACOUSTIC_UPDATE_DQ_STATUS: {
		int new_state = -1;

		if (copy_from_user(&new_state, (void *)arg, sizeof(new_state))) {
			rc = -EFAULT;
			break;
		}
		D("Update DQ Status : %d\n", new_state);
		if (new_state < -1 || new_state > 1) {
			E("Invalid Beats status update");
			rc = -EINVAL;
			break;
		}

		sdev_dq.state = -1;
		switch_set_state(&sdev_dq, new_state);
		break;
	}
	case ACOUSTIC_CONTROL_WAKELOCK: {
		int new_state = -1;

		if (copy_from_user(&new_state, (void *)arg, sizeof(new_state))) {
			rc = -EFAULT;
			break;
		}
		D("control wakelock : %d\n", new_state);
		if (new_state == 1) {
			wake_lock_timeout(&htc_acoustic_wakelock, 60*HZ);
		} else {
			wake_lock_timeout(&htc_acoustic_wakelock_timeout, 1*HZ);
			wake_unlock(&htc_acoustic_wakelock);
		}
		break;
	}
	case ACOUSTIC_DUMMY_WAKELOCK: {
		wake_lock_timeout(&htc_acoustic_dummy_wakelock, 5*HZ);
		break;
	}
	case ACOUSTIC_UPDATE_LISTEN_NOTIFICATION: {
		int new_state = -1;

		if (copy_from_user(&new_state, (void *)arg, sizeof(new_state))) {
			rc = -EFAULT;
			break;
		}
		D("Update listen notification : %d\n", new_state);
		if (new_state < -1 || new_state > 1) {
			E("Invalid listen notification state");
			rc = -EINVAL;
			break;
		}

		sdev_listen_notification.state = -1;
		switch_set_state(&sdev_listen_notification, new_state);
		break;
	}
       case ACOUSTIC_RAMDUMP:
#if 0
		pr_err("trigger ramdump by user space\n");
		if (copy_from_user(&mode, (void *)arg, sizeof(mode))) {
			rc = -EFAULT;
			break;
		}

		if (mode >= 4100 && mode <= 4800) {
			dump_stack();
			pr_err("msgid = %d\n", mode);
			BUG();
		}
#endif
                break;
       case ACOUSTIC_GET_HW_REVISION:
		{
			struct device_info info;
			info.pcb_id = system_rev;
			info.sku_id = 0;
			D("acoustic pcb_id: 0x%x, sku_id: 0x%x\n", info.pcb_id, info.sku_id);
			if(copy_to_user((void *)arg, &info, sizeof(info))) {
				E("acoustic_ioctl: copy to user failed\n");
				rc = -EINVAL;
			}
		}
	        break;
	case ACOUSTIC_AMP_CTRL:
		{
			struct amp_ctrl ampctrl;
			int i;

			if (copy_from_user(&ampctrl, (void __user *)arg, sizeof(ampctrl)))
				return -EFAULT;

			if(ampctrl.type == AMP_HEADPONE) {
				mutex_lock(&hs_amp_lock);
				if(hs_amp.fops && hs_amp.fops->unlocked_ioctl) {
					hs_amp.fops->unlocked_ioctl(file, cmd, arg);
				}
				mutex_unlock(&hs_amp_lock);
			} else if (ampctrl.type == AMP_SPEAKER) {
				mutex_lock(&spk_amp_lock);
				for(i=0; i<SPK_AMP_MAX; i++) {
					if(spk_amp[i].fops && spk_amp[i].fops->unlocked_ioctl) {
						spk_amp[i].fops->unlocked_ioctl(file, cmd, arg);
					}
				}
				mutex_unlock(&spk_amp_lock);
			}
		}
		break;
	case ACOUSTIC_KILL_PID:
		{
			int pid = 0;
			struct pid *pid_struct = NULL;

			if (copy_from_user(&pid, (void *)arg, sizeof(pid))) {
				rc = -EFAULT;
				break;
			}

			D("ACOUSTIC_KILL_PID: %d\n", pid);

			if (pid <= 0)
				break;

			pid_struct = find_get_pid(pid);
			if (pid_struct) {
				kill_pid(pid_struct, SIGKILL, 1);
				D("kill pid: %d", pid);
			}
		}
		break;
	default:
		rc = -EINVAL;
	}
	mutex_unlock(&api_lock);
	return rc;
}

static ssize_t beats_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "Beats\n");
}

static ssize_t dq_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "DQ\n");
}

static ssize_t listen_notification_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "Listen_notification\n");
}

static int hs_amp_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	const struct file_operations *old_fops = NULL;

	mutex_lock(&hs_amp_lock);

	if(hs_amp.fops) {
		old_fops = file->f_op;

		file->f_op = fops_get(hs_amp.fops);

		if (file->f_op == NULL) {
			file->f_op = old_fops;
			ret = -ENODEV;
		}
	}
	mutex_unlock(&hs_amp_lock);

	if(ret >= 0) {

		if (file->f_op->open) {
			ret = file->f_op->open(inode, file);
			if (ret) {
				fops_put(file->f_op);
				if(old_fops)
					file->f_op = fops_get(old_fops);
				return ret;
			}
		}

		if(old_fops)
			fops_put(old_fops);
	}

	return ret;
}

static int hs_amp_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long hs_amp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

void htc_amp_power_register_ops(struct amp_power_ops *ops)
{
	D("%s", __func__);
	the_amp_power_ops = ops;
}

void htc_amp_power_enable(bool enable)
{
	D("%s", __func__);
	if(the_amp_power_ops && the_amp_power_ops->set_amp_power_enable)
		the_amp_power_ops->set_amp_power_enable(enable);
}

static int htc_acoustic_hsnotify(int on)
{
	int i = 0;
	mutex_lock(&hs_nt_lock);
	for(i=0; i<HS_N_MAX; i++) {
		if(hs_plug_nt[i].used && hs_plug_nt[i].callback_f)
			hs_plug_nt[i].callback_f(hs_plug_nt[i].private_data, on);
	}
	mutex_unlock(&hs_nt_lock);

	return 0;
}

static struct file_operations acoustic_fops = {
	.owner = THIS_MODULE,
	.open = acoustic_open,
	.release = acoustic_release,
	.unlocked_ioctl = acoustic_ioctl,
};

static struct miscdevice acoustic_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htc-acoustic",
	.fops = &acoustic_fops,
};

static int __init acoustic_init(void)
{
	int ret = 0;

	struct headset_notifier notifier;

	mutex_init(&api_lock);
	mutex_init(&hs_amp_lock);
	mutex_init(&spk_amp_lock);
	mutex_init(&hs_nt_lock);
	ret = misc_register(&acoustic_misc);
	wake_lock_init(&htc_acoustic_wakelock, WAKE_LOCK_SUSPEND, "htc_acoustic");
	wake_lock_init(&htc_acoustic_wakelock_timeout, WAKE_LOCK_SUSPEND, "htc_acoustic_timeout");
	wake_lock_init(&htc_acoustic_dummy_wakelock, WAKE_LOCK_SUSPEND, "htc_acoustic_dummy");

	if (ret < 0) {
		pr_err("failed to register misc device!\n");
		return ret;
	}

	ret = misc_register(&hs_amp_misc);
	if (ret < 0) {
		pr_err("failed to register aud hs amp device!\n");
		return ret;
	}

	sdev_beats.name = "Beats";
	sdev_beats.print_name = beats_print_name;

	ret = switch_dev_register(&sdev_beats);
	if (ret < 0) {
		pr_err("failed to register beats switch device!\n");
		return ret;
	}

	sdev_dq.name = "DQ";
	sdev_dq.print_name = dq_print_name;

	ret = switch_dev_register(&sdev_dq);
	if (ret < 0) {
		pr_err("failed to register DQ switch device!\n");
		return ret;
	}

	sdev_listen_notification.name = "Listen_notification";
	sdev_listen_notification.print_name = listen_notification_print_name;

	ret = switch_dev_register(&sdev_listen_notification);
	if (ret < 0) {
		pr_err("failed to register listen_notification switch device!\n");
		return ret;
	}

	notifier.id = HEADSET_REG_HS_INSERT;
	notifier.func = htc_acoustic_hsnotify;
	headset_notifier_register(&notifier);

	
	nsd_init();
	
	return 0;
}

static void __exit acoustic_exit(void)
{
	misc_deregister(&acoustic_misc);
}

module_init(acoustic_init);
module_exit(acoustic_exit);
