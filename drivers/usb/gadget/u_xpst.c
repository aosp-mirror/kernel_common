/*
 * Copyright (C) 2011 HTC, Inc.
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

#if defined(CONFIG_DIAGFWD_BRIDGE_CODE)
#include <mach/diag_bridge.h>
int diagfwd_connect_bridge(int process_cable);
int diagfwd_write_complete_hsic(struct diag_request *diag_write_ptr);
#endif

struct diag_context _context;
static struct usb_diag_ch *legacych;
static struct diag_context *legacyctxt;

struct diag_context _mdm_context;
static struct usb_diag_ch *mdmch;
static  struct diag_context *mdmctxt;

struct diag_context _qsc_context;
static struct usb_diag_ch *qscch;
static  struct diag_context *qscctxt;

int htc_usb_enable_function(char *name, int ebl);
static struct switch_dev sw_htc_usb_diag;

#if DIAG_XPST
struct device diag_device;
static char *htc_write_buf_copy;
static struct diag_request htc_w_diag_req;
static struct diag_request *htc_write_diag_req;
#define TRX_REQ_BUF_SZ 8192
#define DEBUG_DMBYTES_RECV 3

#if defined(CONFIG_MACH_MECHA) || defined(CONFIG_MACH_VIGOR) || defined(CONFIG_ARCH_MSM8960) || defined(CONFIG_ARCH_MSM8974) || defined(CONFIG_ARCH_DUMMY)
static	uint16_t nv7K9K_table[NV_TABLE_SZ] = {82,
0, 4, 5, 20, 21, 37, 258, 318, 460, 461,
462, 463, 464, 465, 466, 546, 707, 714, 854, 1943,
2825, 2953, 30000, 30001, 30002, 30003, 30004, 30005, 30006, 30007,
30008, 30010, 30013, 30014, 30015, 30018, 30019, 30021, 30031, 30032,
30033, 6, 8, 53, 54, 161, 162, 40, 42, 43,
57, 71, 82, 85, 168, 169, 170, 191, 192, 197,
219, 231, 240, 241, 256, 297, 298, 300, 423, 424,
429, 442, 450, 459, 495, 835, 855, 910, 933, 941,
945, 3634};

static	uint16_t nv7Konly_table[NV_TABLE_SZ] = {43,
25, 27, 29, 32, 33, 34, 35, 36, 176, 177,
259, 260, 261, 262, 263, 264, 265, 296, 319, 374,
375, 67, 69, 70, 74, 178, 215, 179, 255, 285,
401, 403, 405, 409, 426, 452, 1018, 4102, 906, 30026,
30027, 30028, 30029};

static	uint16_t nv7K9Kdiff_table[NV_TABLE_SZ] = {14,
11, 10, 441, 946, 30016, 30030, 562, 32768+11, 32768+10, 32768+441,
32768+946, 32768+30016, 32768+30030, 32768+562};

static	uint16_t nv9Konly_table[NV_TABLE_SZ] = {7, 579, 580, 1192, 1194, 4204, 4964, 818};

static	uint16_t M297K9K_table[M29_TABLE_SZ] = {4, 1, 2, 4, 5};
static	uint16_t M297Konly_table[M29_TABLE_SZ];
static	uint16_t M297K9Kdiff_table[M29_TABLE_SZ];
static	uint16_t M299Konly_table[M29_TABLE_SZ];

static	uint16_t PRL7K9K_table[PRL_TABLE_SZ] = {1, 0};
static	uint16_t PRL7Konly_table[PRL_TABLE_SZ];
static	uint16_t PRL7K9Kdiff_table[PRL_TABLE_SZ];
static	uint16_t PRL9Konly_table[PRL_TABLE_SZ];

static int radio_initialized; 

static int diag2arm9query;
#define XPST_SMD	0
#define XPST_SDIO	1
#define XPST_HSIC	2

#endif

static struct diag_context *get_modem_ctxt(void)
{
#if defined(CONFIG_ARCH_APQ8064)
	return &_mdm_context;
#else
	return &_context;
#endif
}

static struct usb_request *diag_req_new(unsigned len)
{
	struct usb_request *req;
	if (len > SMD_MAX)
		return NULL;
	req = kmalloc(sizeof(struct usb_request), GFP_KERNEL);
	if (!req)
		return NULL;
	req->buf = kmalloc(len, GFP_KERNEL);
	if (!req->buf) {
		kfree(req);
		return NULL;
	}
	return req;
}

static void diag_req_free(struct usb_request *req)
{
	if (!req)
		return;

	if (req->buf) {
		kfree(req->buf);
		req->buf = 0;
	}
	kfree(req);
	req = 0;
}





static void xpst_req_put(struct diag_context *ctxt, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&ctxt->req_lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&ctxt->req_lock, flags);
}

static struct usb_request *xpst_req_get(struct diag_context *ctxt,
		struct list_head *head)
{
	struct usb_request *req = 0;
	unsigned long flags;

	spin_lock_irqsave(&ctxt->req_lock, flags);
	if (!list_empty(head)) {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&ctxt->req_lock, flags);

	return req;
}

#if defined(CONFIG_MACH_MECHA) || defined(CONFIG_MACH_VIGOR) || defined(CONFIG_ARCH_MSM8960) || defined(CONFIG_ARCH_MSM8974) || defined(CONFIG_ARCH_DUMMY)
int decode_encode_hdlc(void*data, int *len, unsigned char *buf_hdlc, int remove, int pos)
{
	struct diag_send_desc_type send = { NULL, NULL, DIAG_STATE_START, 0 };
	struct diag_hdlc_dest_type enc = { NULL, NULL, 0 };
	struct diag_hdlc_decode_type hdlc;
	unsigned char *buf_9k = NULL;
	int ret;


	buf_9k = kzalloc(USB_MAX_OUT_BUF, GFP_KERNEL);
	if (!buf_9k) {
		DIAG_INFO("%s:out of memory\n", __func__);
		return -ENOMEM;
	}

	hdlc.dest_ptr = buf_9k;
	hdlc.dest_size = USB_MAX_OUT_BUF;
	hdlc.src_ptr = data;
	hdlc.src_size = *len;
	hdlc.src_idx = 0;
	hdlc.dest_idx = 0;
	hdlc.escaping = 0;

	ret = diag_hdlc_decode(&hdlc);
	if (!ret) {
		DIAG_INFO("Packet dropped due to bad HDLC coding/CRC\n");
		kfree(buf_9k);
		return -EINVAL;
	}
	if (remove)
		*((char *)buf_9k+pos) = (*((char *)buf_9k+pos) ^ 0x80);
	else
		*((char *)buf_9k+pos) = (*((char *)buf_9k+pos) | 0x80);


	send.state = DIAG_STATE_START;
	send.pkt = hdlc.dest_ptr;
	send.last = (void *)(hdlc.dest_ptr + hdlc.dest_idx - 4);
	send.terminate = 1;
	enc.dest = buf_hdlc;
	enc.dest_last = (void *)(buf_hdlc + 2*hdlc.dest_idx  - 3);
	diag_hdlc_encode(&send, &enc);

	print_hex_dump(KERN_DEBUG, "encode Data"
			, DUMP_PREFIX_ADDRESS, 16, 1, buf_hdlc, hdlc.dest_idx, 1);

	*len = hdlc.dest_idx;

	kfree(buf_9k);
	return 0;


}
#endif
int checkcmd_modem_epst(unsigned char *buf)
{
#if defined(CONFIG_MACH_MECHA) || defined(CONFIG_MACH_VIGOR)
	int j;
	uint16_t nv_num;
	uint16_t max_item;

	if (*buf == EPST_PREFIX) {
		if (*(buf+1) == 0x26 || *(buf+1) == 0x27) {
			max_item = MAX(MAX(nv7K9K_table[0], nv7Konly_table[0]),
					MAX(nv9Konly_table[0], nv7K9Kdiff_table[0]));
			nv_num = *((uint16_t *)(buf+2));
			DIAG_INFO("%s: id = 0x%x nv_num = %d \n", __func__, *(buf+1), nv_num);
			for (j = 1; j < NV_TABLE_SZ; j++) {
				if (j <= nv7K9K_table[0] && nv7K9K_table[j] == nv_num)
					return  DM7K9K;
				if (j <= nv7Konly_table[0] && nv7Konly_table[j] == nv_num)
					return  DM7KONLY;
				if (j <= nv9Konly_table[0]  && nv9Konly_table[j] == nv_num)
					return  DM9KONLY;
				if (j <= nv7K9Kdiff_table[0]  && nv7K9Kdiff_table[j] == nv_num)
					return  DM7K9KDIFF;
				if (j > max_item)
					break;
			}
			return  NO_DEF_ITEM;
		} else if (*(buf+1) == 0x48 || *(buf+1) == 0x49) {
			max_item = MAX(MAX(PRL7K9K_table[0], PRL7Konly_table[0]),
					MAX(PRL9Konly_table[0], PRL7K9Kdiff_table[0]));
			nv_num = *((uint16_t *)(buf+2));
			DIAG_INFO("%s: id = 0x%x nv_num = %d \n", __func__, *(buf+1), nv_num);
			for (j = 1; j < PRL_TABLE_SZ; j++) {
				if (j <= PRL7K9K_table[0] && PRL7K9K_table[j] == nv_num)
					return  DM7K9K;
				if (j <= PRL7Konly_table[0] && PRL7Konly_table[j] == nv_num)
					return  DM7KONLY;
				if (j <= PRL9Konly_table[0]  && PRL9Konly_table[j] == nv_num)
					return  DM9KONLY;
				if (j <= PRL7K9Kdiff_table[0]  && PRL7K9Kdiff_table[j] == nv_num)
					return  DM7K9KDIFF;
				if (j > max_item)
					break;
			}
			return  NO_DEF_ITEM;
		} else if (*(buf+1) == 0xC9) {
			nv_num = *(buf+2);
			DIAG_INFO("%s: id = 0x%x nv_num = %d \n", __func__, *(buf+1), nv_num);
			if (*(buf+2) == 0x01 || *(buf+2) == 0x11)
				return  DM7K9K;
			else
				return  NO_DEF_ITEM;

		} else if (*(buf+1) == 0x29) {
			max_item = MAX(MAX(M297K9K_table[0], M297Konly_table[0]),
					MAX(M299Konly_table[0], M297K9Kdiff_table[0]));
			nv_num = *((uint16_t *)(buf+2));
			DIAG_INFO("%s: id = 0x%x nv_num = %d \n", __func__, *(buf+1), nv_num);
			for (j = 1; j < M29_TABLE_SZ; j++) {
				if (j <= M297K9K_table[0] && M297K9K_table[j] == nv_num)
					return  DM7K9K;
				if (j <= M297Konly_table[0] && M297Konly_table[j] == nv_num)
					return  DM7KONLY;
				if (j <= M299Konly_table[0]  && M299Konly_table[j] == nv_num)
					return  DM9KONLY;
				if (j <= M297K9Kdiff_table[0]  && M297K9Kdiff_table[j] == nv_num)
					return  DM7K9KDIFF;
				if (j > max_item)
					break;
			}
			return  NO_DEF_ITEM;
		} else if (*(buf+1) == 0x41 || *(buf+1) == 0x0C || *(buf+1) == 0x40) {
			return  DM7K9K;
		} else if (*(buf+1) == 0x00 || *(buf+1) == 0xCD || *(buf+1) == 0xD8
				|| *(buf+1) == 0x35 || *(buf+1) == 0x36 || *(buf+1) == 0x59) {
			return  DM7KONLY;
		} else if (*(buf+1) == 0xDF || *(buf+1) == 0xEC) {
			return  DM9KONLY;
		} else if (*(buf+1) == 0x4B && *(buf+2) == 0x0D) {
			return  DM7KONLY;
		} else
			DIAG_INFO("%s:id = 0x%x no default routing path\n", __func__, *(buf+1));
		return NO_DEF_ID;
	} else {
		
		return NO_PST;
	}

#elif defined(CONFIG_ARCH_MSM8960) || defined(CONFIG_ARCH_MSM8974) || defined(CONFIG_ARCH_DUMMY)
	if (*buf == 0xc && radio_initialized == 0 && diag2arm9query) {
		DIAG_INFO("%s: modem is ready\n", __func__);
		radio_initialized = 1;
		wake_up_interruptible(&driver->wait_q);
		return CHECK_MODEM_ALIVE;
	}
	if (*buf == EPST_PREFIX)
#if defined(CONFIG_DIAGFWD_BRIDGE_CODE)
		return DM9KONLY;
#else
		return DM7KONLY;
#endif
	else
		return NO_PST;
#else
	if (_context.diag2arm9_opened)
		return DM7KONLY;
	else
		return NO_PST;
#endif

}
int modem_to_userspace(void *buf, int r, int type, int is9k)
{

	struct diag_context *ctxt = get_modem_ctxt();
	struct usb_request *req;
#if defined(CONFIG_MACH_MECHA) || defined(CONFIG_MACH_VIGOR) || defined(CONFIG_ARCH_MSM8960) || defined(CONFIG_ARCH_MSM8974) || defined(CONFIG_ARCH_DUMMY)
	unsigned char value;
#endif

	if (!ctxt->diag2arm9_opened)
		return 0;
	if (type == CHECK_MODEM_ALIVE) {
		DIAG_INFO("%s: CHECK_MODEM_ALIVE. not route to userspace\n", __func__);
		return 0;
	}
	req = xpst_req_get(ctxt, &ctxt->rx_arm9_idle);
	if (!req) {
		DIAG_INFO("There is no enough request to ARM11!!\n");
		return 0;
	}
	memcpy(req->buf, buf, r);

#if defined(CONFIG_MACH_MECHA) || defined(CONFIG_MACH_VIGOR) || defined(CONFIG_ARCH_MSM8960) || defined(CONFIG_ARCH_MSM8974) || defined(CONFIG_ARCH_DUMMY)
	if (type == DM7K9KDIFF) {
		value = *((uint8_t *)req->buf+1);
		if ((value == 0x27) || (value == 0x26)) {
			if (is9k == 1)
				decode_encode_hdlc(buf, &r, req->buf, 0, 3);
		}
	} else if (type == NO_DEF_ID) {
		
		value = *((uint8_t *)req->buf+2);
		DIAG_INFO("%s:check error cmd=0x%x message=ox%x\n", __func__
				, value, *((uint8_t *)req->buf+1));
		if ((value == 0x27) || (value == 0x26)) {
			if (is9k == 1) {
				decode_encode_hdlc(buf, &r, req->buf, 0, 4);
			}
		}
	}
#endif

	if (is9k == 1)
		print_hex_dump(KERN_DEBUG, "DM Read Packet Data"
				" from 9k radio (first 16 Bytes): ", DUMP_PREFIX_ADDRESS, 16, 1, req->buf, 16, 1);
	else
		print_hex_dump(KERN_DEBUG, "DM Read Packet Data"
				" from 7k radio (first 16 Bytes): ", DUMP_PREFIX_ADDRESS, 16, 1, req->buf, 16, 1);
#if defined(CONFIG_DIAGFWD_BRIDGE_CODE)
	diagfwd_write_complete_hsic(NULL);
	if (driver->hsic_ch)
		queue_work(diag_bridge[HSIC].wq, &driver->diag_read_hsic_work);
#endif
	
	req->actual = r;
	xpst_req_put(ctxt, &ctxt->rx_arm9_done, req);
	wake_up(&ctxt->read_arm9_wq);
	return 1;
}



static long htc_diag_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct diag_context *ctxt = get_modem_ctxt();
	void __user *argp = (void __user *)arg;
	int tmp_value;
	unsigned long flags;
	unsigned char temp_id_table[ID_TABLE_SZ];

	DIAG_INFO("%s:%s(parent:%s): tgid=%d\n", __func__,
			current->comm, current->parent->comm, current->tgid);


	if (_IOC_TYPE(cmd) != USB_DIAG_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case USB_DIAG_FUNC_IOC_ENABLE_SET:
		if (copy_from_user(&tmp_value, argp, sizeof(int)))
			return -EFAULT;
		DIAG_INFO("diag: enable %d\n", tmp_value);
		switch_set_state(&sw_htc_usb_diag, !!tmp_value);
#if defined(CONFIG_MACH_VIGOR) || defined(CONFIG_ARCH_APQ8064)
		htc_usb_enable_function(DIAG_MDM, tmp_value);
#endif
		
		htc_usb_enable_function("acm", 0);
		htc_usb_enable_function(DIAG_LEGACY, tmp_value);

		if (tmp_value) {
			
			diag_smd_enable(driver->smd_data[MODEM_DATA].ch, "diag_ioctl", tmp_value);
		}
#if defined(CONFIG_MACH_MECHA)
		
		smsc251x_mdm_port_sw(tmp_value);
#endif
		
		if (tmp_value == 0)
			ctxt->error = 1;
		wake_up(&ctxt->read_wq);
		break;
	case USB_DIAG_FUNC_IOC_ENABLE_GET:
		tmp_value = ctxt->online;

		if (copy_to_user(argp, &tmp_value, sizeof(tmp_value)))
			return -EFAULT;
		break;

	case USB_DIAG_FUNC_IOC_REGISTER_SET:
		if (copy_from_user(temp_id_table, (unsigned char *)argp, ID_TABLE_SZ))
			return -EFAULT;
		spin_lock_irqsave(&ctxt->req_lock, flags);
		memcpy(ctxt->id_table, temp_id_table, ID_TABLE_SZ);
		print_hex_dump(KERN_DEBUG, "ID_TABLE_SZ Data: ", DUMP_PREFIX_ADDRESS, 16, 1,
				temp_id_table, ID_TABLE_SZ, 1);
		spin_unlock_irqrestore(&ctxt->req_lock, flags);
		break;

	case USB_DIAG_FUNC_IOC_AMR_SET:
		DIAG_INFO("diag: fix me USB_DIAG_FUNC_IOC_AMR_SET\n");
		break;
	case USB_DIAG_FUNC_IOC_LOGTYPE_GET:
		
		tmp_value = (driver->logging_mode == 1)?1:0;
		if (copy_to_user(argp, &tmp_value, sizeof(tmp_value)))
			return -EFAULT;
		break;
	default:
		return -ENOTTY;
	}


	return 0;
}

static ssize_t htc_diag_read(struct file *fp, char __user *buf,
		size_t count, loff_t *pos)
{
	struct diag_context *ctxt = get_modem_ctxt();
	struct usb_request *req = 0;
	int ret = 0;


	DIAG_INFO("%s:%s(parent:%s): tgid=%d\n", __func__,
			current->comm, current->parent->comm, current->tgid);

	
	if (!ctxt->online) {
		ret = wait_event_interruptible(ctxt->read_wq, (ctxt->online || ctxt->error));
		if (ret < 0 || ctxt->error)
			return -EFAULT;
	}

	mutex_lock(&ctxt->user_lock);

	if (ctxt->user_read_len && ctxt->user_readp) {
		if (count > ctxt->user_read_len)
			count = ctxt->user_read_len;
		if (copy_to_user(buf, ctxt->user_readp, count))
			ret = -EFAULT;
		else {
			ctxt->user_readp += count;
			ctxt->user_read_len -= count;
			ret = count;
		}
		goto end;
	}

	mutex_unlock(&ctxt->user_lock);
	ret = wait_event_interruptible(ctxt->read_wq,
			(req = xpst_req_get(ctxt, &ctxt->rx_req_user)) || !ctxt->online);
	mutex_lock(&ctxt->user_lock);

	if (ret < 0) {
		DIAG_INFO("%s: wait_event_interruptible error %d\n",
				__func__, ret);
		goto end;
	}

	if (!ctxt->online) {
		DIAG_INFO("%s: offline\n", __func__);
		ret = -EIO;
		goto end;
	}

	if (req) {
		if (req->actual == 0) {
			DIAG_INFO("%s: no data\n", __func__);
			goto end;
		}
		if (count > req->actual)
			count = req->actual;
		if (copy_to_user(buf, req->buf, count)) {
			ret = -EFAULT;
			goto end;
		}
		req->actual -= count;
		if (req->actual) {
			memcpy(ctxt->user_read_buf, req->buf + count, req->actual);
			ctxt->user_read_len = req->actual;
			ctxt->user_readp = ctxt->user_read_buf;
		}
		ret = count;
	}
end:
	if (req)
		xpst_req_put(ctxt, &ctxt->rx_req_idle, req);

	mutex_unlock(&ctxt->user_lock);
	return ret;
}

static ssize_t htc_diag_write(struct file *fp, const char __user *buf,
		size_t count, loff_t *pos)
{
	struct diag_context *ctxt = get_modem_ctxt();
	int ret = 0;


	DIAG_INFO("%s:%s(parent:%s): tgid=%d\n", __func__,
			current->comm, current->parent->comm, current->tgid);

	mutex_lock(&ctxt->user_lock);

	if (ret < 0) {
		DIAG_INFO("%s: wait_event_interruptible error %d\n",
				__func__, ret);
		goto end;
	}

	if (!ctxt->online) {
		DIAG_INFO("%s: offline\n", __func__);
		ret = -EIO;
		goto end;
	}

	if (count > TRX_REQ_BUF_SZ)
		count = TRX_REQ_BUF_SZ;

	if (!htc_write_buf_copy || !htc_write_diag_req) {
		ret = -EIO;
		goto end;
	}

	if (copy_from_user(htc_write_buf_copy, buf, count)) {
		ret = -EFAULT;
		DIAG_INFO("%s:EFAULT\n", __func__);
		goto end;
	}

	htc_write_diag_req->buf = htc_write_buf_copy;
	htc_write_diag_req->length = count;

	driver->in_busy_dmrounter = 1;

	ret = usb_diag_write(ctxt->ch, htc_write_diag_req);

	if (ret < 0) {
		DIAG_INFO("%s: usb_diag_write error %d\n", __func__, ret);
		goto end;
	}
	ret = count;

end:

	mutex_unlock(&ctxt->user_lock);
	return ret;
}

static int htc_diag_open(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = get_modem_ctxt();
	int rc = 0;
	int n;
	struct usb_request *req;


	DIAG_INFO("%s:%s(parent:%s): tgid=%d\n", __func__,
			current->comm, current->parent->comm, current->tgid);
	if (!ctxt->ready) {
		DIAG_INFO("%s: USB driver do not load\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&ctxt->user_lock);

	if (ctxt->opened) {
		pr_err("%s: already opened\n", __func__);
		rc = -EBUSY;
		goto done;
	}


	ctxt->user_read_len = 0;
	ctxt->user_readp = 0;
	if (!ctxt->user_read_buf) {
		ctxt->user_read_buf = kmalloc(TRX_REQ_BUF_SZ, GFP_KERNEL);
		if (!ctxt->user_read_buf) {
			rc = -ENOMEM;
			goto done;
		}
	}

	if (!htc_write_buf_copy) {
		htc_write_buf_copy = kmalloc(TRX_REQ_BUF_SZ, GFP_KERNEL);
		if (!htc_write_buf_copy) {
			rc = -ENOMEM;
			kfree(ctxt->user_read_buf);
			goto done;
		}
	}

	if (!htc_write_diag_req) {
		htc_write_diag_req = &htc_w_diag_req;
		if (!htc_write_diag_req) {
			kfree(ctxt->user_read_buf);
			kfree(htc_write_buf_copy);
			rc = -ENOMEM;
			goto done;
		}
	}

	
	while ((req = xpst_req_get(ctxt, &ctxt->rx_req_idle)))
		diag_req_free(req);

	for (n = 0; n < 10; n++) {
		req = diag_req_new(SMD_MAX);
		if (!req) {
			while ((req = xpst_req_get(ctxt, &ctxt->rx_req_idle)))
				diag_req_free(req);
			rc = -EFAULT;
			goto done;
		}
		xpst_req_put(ctxt, &ctxt->rx_req_idle, req);
	}

	ctxt->opened = true;
	
	ctxt->error = 0;

done:
	mutex_unlock(&ctxt->user_lock);

	return rc;
}

static int htc_diag_release(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = get_modem_ctxt();
	struct usb_request *req;


	DIAG_INFO("%s: \n", __func__);


	mutex_lock(&ctxt->user_lock);
	ctxt->opened = false;
	ctxt->user_read_len = 0;
	ctxt->user_readp = 0;
	if (ctxt->user_read_buf) {
		kfree(ctxt->user_read_buf);
		ctxt->user_read_buf = 0;
	}
	if (htc_write_buf_copy) {
		kfree(htc_write_buf_copy);
		htc_write_buf_copy = 0;
	}
	if (htc_write_diag_req)
		htc_write_diag_req = 0;
	while ((req = xpst_req_get(ctxt, &ctxt->rx_req_idle)))
		diag_req_free(req);
	while ((req = xpst_req_get(ctxt, &ctxt->rx_req_user)))
		diag_req_free(req);
	mutex_unlock(&ctxt->user_lock);

	return 0;
}

static struct file_operations htc_diag_fops = {
	.owner =   THIS_MODULE,
	.read =    htc_diag_read,
	.write =   htc_diag_write,
	.open =    htc_diag_open,
	.release = htc_diag_release,
	.unlocked_ioctl = htc_diag_ioctl,
};

static struct miscdevice htc_diag_device_fops = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htcdiag",
	.fops = &htc_diag_fops,
};


static int if_route_to_userspace(struct diag_context *ctxt, unsigned int cmd)
{
	unsigned long flags;
	int i;
	unsigned short tmp;
	unsigned char cmd_id, cmd_num;

	tmp = (unsigned short)cmd;

	cmd_num = (unsigned char)(tmp >> 8);
	cmd_id = (unsigned char)(tmp & 0x00ff);

	if (!ctxt->opened || cmd_id == 0)
		return 0;
	
	if (cmd_id >= 0xfb && cmd_id <= 0xff)
		return 1;


	spin_lock_irqsave(&ctxt->req_lock, flags);
	for (i = 0; i < ARRAY_SIZE(ctxt->id_table); i = i+2)
		if (ctxt->id_table[i] == cmd_id) {
			if (ctxt->id_table[i+1] == cmd_num || ctxt->id_table[i+1] == 0xff) {
				spin_unlock_irqrestore(&ctxt->req_lock, flags);
				return 1;
			}
		}
	spin_unlock_irqrestore(&ctxt->req_lock, flags);

	return 0;
}

#if defined(CONFIG_MACH_MECHA) || defined(CONFIG_MACH_VIGOR) || defined(CONFIG_ARCH_MSM8960) || defined(CONFIG_ARCH_MSM8974) || defined(CONFIG_ARCH_DUMMY)
static int check_modem_type(void)
{
#if defined(CONFIG_DIAG_SDIO_PIPE)
	return XPST_SDIO;
#elif defined(CONFIG_DIAGFWD_BRIDGE_CODE)
	return XPST_HSIC;
#else
	return XPST_SMD;
#endif
}

static int check_modem_task_ready(int channel)
{
	static unsigned char phone_status[] = {0xc, 0x14, 0x3a, 0x7e};
	int ret;

	if (radio_initialized) {
		DIAG_INFO("%s:modem status=ready\n", __func__);
		return radio_initialized;
	}

	diag2arm9query = 1;

	switch (channel) {
	case XPST_SMD:
		if (!smd_diag_initialized) {
			DIAG_INFO("%s:modem status=smd not ready\n", __func__);
			return -EAGAIN;
		}
		smd_write(driver->smd_cmd[MODEM_DATA].ch, phone_status, 4);
		break;
#if defined(CONFIG_DIAGFWD_BRIDGE_CODE)
	case XPST_HSIC:
		if (!driver->hsic_device_enabled) {
			DIAG_INFO("%s:modem status=hsic not ready\n", __func__);
			return -EAGAIN;
		}
		driver->in_busy_hsic_write = 1;
		driver->in_busy_hsic_read_on_device = 0;
		diag_bridge_write(phone_status, 4);
		break;
#endif
#if defined(CONFIG_DIAG_SDIO_PIPE)
	case XPST_SDIO:
		DIAG_INFO("%s:modem status=%d\n", __func__, sdio_diag_initialized);
		return sdio_diag_initialized;
#endif
	default:
		DIAG_WARNING("%s: channel type(%d) not support\n", __func__, channel);
		return -EINVAL;
	}

	driver->debug_dmbytes_recv = DEBUG_DMBYTES_RECV;
	ret = wait_event_interruptible_timeout(driver->wait_q, radio_initialized != 0, 4 * HZ);
	DIAG_INFO("%s:modem status=%d %s\n", __func__, radio_initialized, (ret == 0)?"(timeout)":"");

#if defined(CONFIG_DIAGFWD_BRIDGE_CODE)
	diagfwd_write_complete_hsic(NULL);
	if (driver->hsic_ch)
		queue_work(diag_bridge[HSIC].wq, &driver->diag_read_hsic_work);
#endif
	return radio_initialized;
}

static long diag2arm9_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct diag_context *ctxt = get_modem_ctxt();
	void __user *argp = (void __user *)arg;
	unsigned long flags;
	uint16_t temp_nv_table[NV_TABLE_SZ];
	int table_size, ret;
	uint16_t *table_ptr;

	if (_IOC_TYPE(cmd) != USB_DIAG_IOC_MAGIC)
		return -ENOTTY;

	DIAG_INFO("%s:%s(parent:%s): tgid=%d\n", __func__,
			current->comm, current->parent->comm, current->tgid);

	switch (cmd) {

	case USB_DIAG_NV_7K9K_SET:
		DIAG_INFO("USB_DIAG_NV_7K9K_SET\n");
		table_size = NV_TABLE_SZ;
		table_ptr = nv7K9K_table;
		break;
	case USB_DIAG_NV_7KONLY_SET:
		DIAG_INFO("USB_DIAG_NV_7KONLY_SET\n");
		table_size = NV_TABLE_SZ;
		table_ptr = nv7Konly_table;
		break;
	case USB_DIAG_NV_9KONLY_SET:
		DIAG_INFO("USB_DIAG_NV_9KONLY_SET\n");
		table_size = NV_TABLE_SZ;
		table_ptr = nv9Konly_table;
		break;
	case USB_DIAG_NV_7K9KDIFF_SET:
		DIAG_INFO("USB_DIAG_NV_7K9KDIFF_SET\n");
		table_size = NV_TABLE_SZ;
		table_ptr = nv7K9Kdiff_table;
		break;

	case USB_DIAG_PRL_7K9K_SET:
		DIAG_INFO("USB_DIAG_PRL_7K9K_SET\n");
		table_size = PRL_TABLE_SZ;
		table_ptr = PRL7K9K_table;
		break;
	case USB_DIAG_PRL_7KONLY_SET:
		DIAG_INFO("USB_DIAG_PRL_7KONLY_SET\n");
		table_size = PRL_TABLE_SZ;
		table_ptr = PRL7Konly_table;
		break;
	case USB_DIAG_PRL_9KONLY_SET:
		DIAG_INFO("USB_DIAG_PRL_9KONLY_SET\n");
		table_size = PRL_TABLE_SZ;
		table_ptr = PRL9Konly_table;
		break;
	case USB_DIAG_PRL_7K9KDIFF_SET:
		DIAG_INFO("USB_DIAG_PRL_7K9KDIFF_SET\n");
		table_size = PRL_TABLE_SZ;
		table_ptr = PRL7K9Kdiff_table;
		break;

	case USB_DIAG_M29_7K9K_SET:
		DIAG_INFO("USB_DIAG_M29_7K9K_SET\n");
		table_size = M29_TABLE_SZ;
		table_ptr = M297K9K_table;
		break;

	case USB_DIAG_M29_7KONLY_SET:
		DIAG_INFO("USB_DIAG_M29_7KONLY_SET\n");
		table_size = M29_TABLE_SZ;
		table_ptr = M297Konly_table;
		break;
	case USB_DIAG_M29_9KONLY_SET:
		DIAG_INFO("USB_DIAG_M29_9KONLY_SET\n");
		table_size = M29_TABLE_SZ;
		table_ptr = M299Konly_table;
		break;
	case USB_DIAG_M29_7K9KDIFF_SET:
		DIAG_INFO("USB_DIAG_M29_7K9KDIFF_SET\n");
		table_size = M29_TABLE_SZ;
		table_ptr = M297K9Kdiff_table;
		break;
	case USB_DIAG_FUNC_IOC_MODEM_GET:
		ret = check_modem_task_ready(check_modem_type());
		if (ret < 0)
			return ret;
		if (copy_to_user(argp, &ret, sizeof(ret)))
			return -EFAULT;
		else
			return 0;
		break;
	default:
		return -ENOTTY;
	}

	if (copy_from_user(temp_nv_table, (uint8_t *)argp, (table_size*2)))
		return -EFAULT;
	DIAG_INFO("%s:input %d item\n", __func__, temp_nv_table[0]);
	if (temp_nv_table[0] > table_size)
		return -EFAULT;

	spin_lock_irqsave(&ctxt->req_lock, flags);
	memcpy((uint8_t *)table_ptr, (uint8_t *)&temp_nv_table[0], (temp_nv_table[0]+1)*2);
	print_hex_dump(KERN_DEBUG, "TABLE Data: ", DUMP_PREFIX_ADDRESS, 16, 1,
			table_ptr, (*table_ptr+1)*2, 1);
	spin_unlock_irqrestore(&ctxt->req_lock, flags);

	return 0;
}
#endif
static int diag2arm9_open(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = get_modem_ctxt();
	struct usb_request *req;
	int rc = 0;
	int n;
	DIAG_INFO("%s\n", __func__);
	if (!ctxt->ready) {
		DIAG_INFO("%s: USB driver do not load\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&ctxt->diag2arm9_lock);
	if (ctxt->diag2arm9_opened) {
		pr_err("%s: already opened\n", __func__);
		rc = -EBUSY;
		goto done;
	}
	
	while ((req = xpst_req_get(ctxt, &ctxt->rx_arm9_done)))
		diag_req_free(req);

	for (n = 0; n < 8; n++) {
		req = diag_req_new(SMD_MAX);
		if (!req) {
			while ((req = xpst_req_get(ctxt, &ctxt->rx_arm9_idle)))
				diag_req_free(req);
			rc = -EFAULT;
			goto done;
		}
		xpst_req_put(ctxt, &ctxt->rx_arm9_idle, req);
	}
	ctxt->read_arm9_count = 0;
	ctxt->read_arm9_buf = 0;
	ctxt->read_arm9_req = 0;
	ctxt->diag2arm9_opened = true;

#if defined(CONFIG_DIAGFWD_BRIDGE_CODE)
	diagfwd_connect_bridge(0);
#else
	diag_smd_enable(driver->smd_data[MODEM_DATA].ch, "diag2arm9_open", SMD_FUNC_OPEN_DIAG);
#endif

	
	driver->smd_data[MODEM_DATA].in_busy_1 = 0;
	driver->smd_data[MODEM_DATA].in_busy_2 = 0;
#if defined(CONFIG_MACH_VIGOR)
	diag2arm9_buf_9k = kzalloc(USB_MAX_OUT_BUF, GFP_KERNEL);
#endif
done:
	mutex_unlock(&ctxt->diag2arm9_lock);
	return rc;
}

static int diag2arm9_release(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = get_modem_ctxt();
	struct usb_request *req;

	DIAG_INFO("%s\n", __func__);
	mutex_lock(&ctxt->diag2arm9_lock);
	ctxt->diag2arm9_opened = false;
	wake_up(&ctxt->read_arm9_wq);
	mutex_lock(&ctxt->diag2arm9_read_lock);
	while ((req = xpst_req_get(ctxt, &ctxt->rx_arm9_idle)))
		diag_req_free(req);
	while ((req = xpst_req_get(ctxt, &ctxt->rx_arm9_done)))
		diag_req_free(req);
	if (ctxt->read_arm9_req)
		diag_req_free(ctxt->read_arm9_req);
	mutex_unlock(&ctxt->diag2arm9_read_lock);

	
	mutex_unlock(&ctxt->diag2arm9_lock);
#if defined(CONFIG_MACH_VIGOR)
	kfree(diag2arm9_buf_9k);
#endif
	return 0;
}

static ssize_t diag2arm9_write(struct file *fp, const char __user *buf,
		size_t count, loff_t *pos)
{
	struct diag_context *ctxt = get_modem_ctxt();
	int r = count;
	int writed = 0;
#if defined(CONFIG_MACH_MECHA) || defined(CONFIG_MACH_VIGOR) || defined(CONFIG_ARCH_MSM8960) || defined(CONFIG_ARCH_MSM8974) || defined(CONFIG_ARCH_DUMMY)
	int path;
	struct diag_hdlc_decode_type hdlc;
	int ret;
#if defined(CONFIG_MACH_MECHA)
	unsigned char *buf_9k = NULL;
#endif
#endif

	mutex_lock(&ctxt->diag2arm9_write_lock);
	DIAG_INFO("%s : count = %d\n", __func__, count);
	while (count > 0) {
		writed = count > USB_MAX_OUT_BUF ? USB_MAX_OUT_BUF : count;
		if (copy_from_user(ctxt->DM_buf, buf, writed)) {
			r = -EFAULT;
			break;
		}
#if defined(CONFIG_DIAGFWD_BRIDGE_CODE)
		if (driver->hsic_ch == 0) {
			DIAG_INFO("%s: driver->hsic_ch == NULL", __func__);
			r = -EFAULT;
			break;
		}
#else
		if (driver->smd_data[MODEM_DATA].ch == NULL) {
			DIAG_INFO("%s: driver->smd_data[MODEM_DATA].ch == NULL", __func__);
			r = -EFAULT;
			break;
		} else if (ctxt->toARM9_buf == NULL) {
			DIAG_INFO("%s: ctxt->toARM9_buf == NULL", __func__);
			r = -EFAULT;
			break;
		}
#endif

#if defined(CONFIG_MACH_MECHA) || defined(CONFIG_MACH_VIGOR) || defined(CONFIG_ARCH_MSM8960) || defined(CONFIG_ARCH_MSM8974) || defined(CONFIG_ARCH_DUMMY)
		path = checkcmd_modem_epst(ctxt->DM_buf);

		print_hex_dump(KERN_DEBUG, "DM Packet Data"
				" write to radio ", DUMP_PREFIX_ADDRESS, 16, 1, ctxt->DM_buf, writed, 1);

		switch (path) {
		case DM7K9K:
			DIAG_INFO("%s:above date to DM7K9K\n", __func__);
			
#if defined(CONFIG_MACH_MECHA)
			if (sdio_diag_initialized) {
				buf_9k = kzalloc(writed, GFP_KERNEL);
				if (!buf_9k) {
					DIAG_INFO("%s:out of memory\n", __func__);
					mutex_unlock(&ctxt->diag2arm9_write_lock);
					return -ENOMEM;
				}
				memcpy(buf_9k, ctxt->DM_buf, writed);
				msm_sdio_diag_write((void *)buf_9k, writed);
				buf_9k = NULL;
			}
#endif
#if defined(CONFIG_MACH_VIGOR)
			if (driver->sdio_ch) {
				memcpy(diag2arm9_buf_9k, ctxt->DM_buf, writed);
				sdio_write(driver->sdio_ch, diag2arm9_buf_9k, writed);
			} else {
				DIAG_INFO("%s: sdio ch fails\n", __func__);
			}
#endif
			
			hdlc.dest_ptr = ctxt->toARM9_buf;
			hdlc.dest_size = SMD_MAX;
			hdlc.src_ptr = ctxt->DM_buf;
			hdlc.src_size = writed;
			hdlc.src_idx = 0;
			hdlc.dest_idx = 0;
			hdlc.escaping = 0;

			ret = diag_hdlc_decode(&hdlc);
			if (!ret) {
				DIAG_INFO("Packet dropped due to bad HDLC coding/CRC\n");
				r = -EFAULT;
				break;
			}
			smd_write(driver->smd_data[MODEM_DATA].ch, ctxt->toARM9_buf, hdlc.dest_idx-3);
			break;
		case DM9KONLY:
			DIAG_INFO("%s:above date to DM9KONLY\n", __func__);

#if defined(CONFIG_MACH_MECHA)
			if (sdio_diag_initialized) {
				buf_9k = kzalloc(writed, GFP_KERNEL);
				if (!buf_9k) {
					DIAG_INFO("%s:out of memory\n", __func__);
					mutex_unlock(&ctxt->diag2arm9_write_lock);
					return -ENOMEM;
				}
				memcpy(buf_9k, ctxt->DM_buf, writed);
				msm_sdio_diag_write((void *)buf_9k, writed);
				buf_9k = NULL;
			}
#endif
#if defined(CONFIG_MACH_VIGOR)
			if (driver->sdio_ch) {
				memcpy(diag2arm9_buf_9k, ctxt->DM_buf, writed);
				sdio_write(driver->sdio_ch, diag2arm9_buf_9k, writed);
			} else {
				DIAG_INFO("%s: sdio ch fails\n", __func__);
			}

#endif
#if defined(CONFIG_DIAGFWD_BRIDGE_CODE)
			driver->in_busy_hsic_write = 1;
			driver->in_busy_hsic_read_on_device = 0;
			ret = diag_bridge_write(ctxt->DM_buf, writed);
			if (ret) {
				DIAG_INFO(": diag_bridge write failed %d\n", ret);
				if ((-ESHUTDOWN) != ret)
					driver->in_busy_hsic_write = 0;
			}
			queue_work(diag_bridge[HSIC].wq, &driver->diag_read_hsic_work);
#endif
			break;
		case DM7K9KDIFF:
			DIAG_INFO("%s:above data to DM7K9KDIFF\n", __func__);
			
			if ((ctxt->DM_buf[3] & 0x80) == 0x80) {
				DIAG_INFO("%s:DM7K9KDIFF to 9K\n", __func__);
#if defined(CONFIG_MACH_MECHA)
				if (sdio_diag_initialized) {
					buf_9k = kzalloc(USB_MAX_OUT_BUF, GFP_KERNEL);
					if (!buf_9k) {
						DIAG_INFO("%s:out of memory\n", __func__);
						mutex_unlock(&ctxt->diag2arm9_write_lock);
						return -ENOMEM;
					}
					if (decode_encode_hdlc(ctxt->DM_buf, &writed, buf_9k, 1, 3)) {
						kfree(buf_9k);
						mutex_unlock(&ctxt->diag2arm9_write_lock);
						return -EINVAL;
					}
					msm_sdio_diag_write((void *)buf_9k, writed);
					buf_9k = NULL;
				}
#endif
#if defined(CONFIG_MACH_VIGOR)
				if (driver->sdio_ch) {
					if (decode_encode_hdlc(ctxt->DM_buf, &writed, diag2arm9_buf_9k, 1, 3)) {
						mutex_unlock(&ctxt->diag2arm9_write_lock);
						return -EINVAL;
					}
					sdio_write(driver->sdio_ch, diag2arm9_buf_9k, writed);
				} else {
					DIAG_INFO("%s: sdio ch fails\n", __func__);
				}

#endif
			} else {
				DIAG_INFO("%s:DM7K9KDIFF to 7K\n", __func__);
#if defined(CONFIG_MACH_MECHA)
				smd_write(driver->smd_data[MODEM_DATA].ch, ctxt->DM_buf, writed);
#endif
#if defined(CONFIG_MACH_VIGOR)
				
				hdlc.dest_ptr = ctxt->toARM9_buf;
				hdlc.dest_size = SMD_MAX;
				hdlc.src_ptr = ctxt->DM_buf;
				hdlc.src_size = writed;
				hdlc.src_idx = 0;
				hdlc.dest_idx = 0;
				hdlc.escaping = 0;

				ret = diag_hdlc_decode(&hdlc);
				if (!ret) {
					DIAG_INFO("Packet dropped due to bad HDLC coding/CRC\n");
					r = -EFAULT;
					break;
				}
				smd_write(driver->smd_data[MODEM_DATA].ch, ctxt->toARM9_buf, hdlc.dest_idx-3);
#endif
			}
			break;

		case DM7KONLY:
			DIAG_INFO("%s:above data to DM7KONLY\n", __func__);
#if !defined(CONFIG_MACH_VIGOR)
			smd_write(driver->smd_cmd[MODEM_DATA].ch, ctxt->DM_buf, writed);
#else
			
			hdlc.dest_ptr = ctxt->toARM9_buf;
			hdlc.dest_size = SMD_MAX;
			hdlc.src_ptr = ctxt->DM_buf;
			hdlc.src_size = writed;
			hdlc.src_idx = 0;
			hdlc.dest_idx = 0;
			hdlc.escaping = 0;

			ret = diag_hdlc_decode(&hdlc);
			if (!ret) {
				DIAG_INFO("Packet dropped due to bad HDLC coding/CRC\n");
				r = -EFAULT;
				break;
			}
			smd_write(driver->smd_data[MODEM_DATA].ch, ctxt->toARM9_buf, hdlc.dest_idx-3);
#endif
			break;
		case NO_DEF_ID:
		case NO_DEF_ITEM:
		default:
			DIAG_INFO("%s:no default routing path\n", __func__);
			print_hex_dump(KERN_DEBUG, "DM Packet Data"
					" write to radio ", DUMP_PREFIX_ADDRESS, 16, 1, ctxt->DM_buf, writed, 1);
		}
#endif
		buf += writed;
		count -= writed;
		if (count)
			DIAG_INFO("%s :[WARN] count = %d\n", __func__, count);

	}
	driver->debug_dmbytes_recv = DEBUG_DMBYTES_RECV;
	mutex_unlock(&ctxt->diag2arm9_write_lock);

	return r;

}

static ssize_t diag2arm9_read(struct file *fp, char __user *buf,
		size_t count, loff_t *pos)
{
	struct diag_context *ctxt = get_modem_ctxt();
	struct usb_request *req;
	int r = 0, xfer;
	int ret;
	DIAG_INFO("%s\n", __func__);
	mutex_lock(&ctxt->diag2arm9_read_lock);

	
	if (ctxt->read_arm9_count > 0)
		req = ctxt->read_arm9_req;
	else {
retry:
		
		req = 0;
		ret = wait_event_interruptible_timeout(ctxt->read_arm9_wq,
				((req = xpst_req_get(ctxt, &ctxt->rx_arm9_done)) ||
				 !ctxt->diag2arm9_opened), 2 * HZ);
		if (!ctxt->diag2arm9_opened) {
			if (req)
				xpst_req_put(ctxt, &ctxt->rx_arm9_idle, req);
			goto done;
		}
		if (ret < 0 || req == 0)
			goto done;

		if (req->actual == 0) {
			xpst_req_put(ctxt, &ctxt->rx_arm9_idle, req);
			goto retry;
		}
		ctxt->read_arm9_req = req;
		ctxt->read_arm9_count = req->actual;
		ctxt->read_arm9_buf = req->buf;
	}
	xfer = (ctxt->read_arm9_count < count) ? ctxt->read_arm9_count : count;
	if (copy_to_user(buf, ctxt->read_arm9_buf, xfer)) {
		DIAG_INFO("diag: copy_to_user fail\n");
		xpst_req_put(ctxt, &ctxt->rx_arm9_idle, ctxt->read_arm9_req);
		r = -EFAULT;
		goto done;
	}
	ctxt->read_arm9_buf += xfer;
	ctxt->read_arm9_count -= xfer;
	r += xfer;
	
	if (ctxt->read_arm9_count == 0) {
		print_hex_dump(KERN_DEBUG, "DM Packet Data"
				" read from radio ", DUMP_PREFIX_ADDRESS, 16, 1, req->buf, req->actual, 1);
		xpst_req_put(ctxt, &ctxt->rx_arm9_idle, ctxt->read_arm9_req);
		ctxt->read_arm9_req = 0;
	}
done:
	mutex_unlock(&ctxt->diag2arm9_read_lock);
	if (ret < 0)
		pr_info("wait_event_interruptible breaked ret=%d\n", ret);
	return r;
}
static struct file_operations diag2arm9_fops = {
	.owner =   THIS_MODULE,
	.open =    diag2arm9_open,
	.release = diag2arm9_release,
	.write = diag2arm9_write,
	.read = diag2arm9_read,
#if defined(CONFIG_MACH_MECHA) || defined(CONFIG_MACH_VIGOR) || defined(CONFIG_ARCH_MSM8960) || defined(CONFIG_ARCH_MSM8974) || defined(CONFIG_ARCH_DUMMY)
	.unlocked_ioctl = diag2arm9_ioctl,
#endif
};

static struct miscdevice diag2arm9_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "diag_arm9",
	.fops = &diag2arm9_fops,
};

#endif
