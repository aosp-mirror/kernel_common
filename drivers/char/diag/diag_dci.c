/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/diagchar.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/spinlock.h>
#include <linux/ratelimit.h>
#include <linux/reboot.h>
#include <asm/current.h>
#include <mach/restart.h>
#ifdef CONFIG_DIAG_OVER_USB
#include <mach/usbdiag.h>
#endif
#include "diagchar_hdlc.h"
#include "diagmem.h"
#include "diagchar.h"
#include "diagfwd.h"
#include "diagfwd_cntl.h"
#include "diag_dci.h"

static struct timer_list dci_drain_timer;
static int dci_timer_in_progress;
static struct work_struct dci_data_drain_work;

unsigned int dci_max_reg = 100;
unsigned int dci_max_clients = 10;
unsigned char dci_cumulative_log_mask[DCI_LOG_MASK_SIZE];
unsigned char dci_cumulative_event_mask[DCI_EVENT_MASK_SIZE];
struct mutex dci_log_mask_mutex;
struct mutex dci_event_mask_mutex;
struct mutex dci_health_mutex;

spinlock_t ws_lock;
unsigned long ws_lock_flags;

#define DCI_WAKEUP_TIMEOUT 1

#define DCI_CAN_ADD_BUF_TO_LIST(buf)					\
	(buf && buf->data && !buf->in_busy && buf->data_len > 0)	\

#ifdef CONFIG_DEBUG_FS
struct diag_dci_data_info *dci_data_smd;
struct mutex dci_stat_mutex;

void diag_dci_smd_record_info(int read_bytes, uint8_t ch_type,
			      uint8_t peripheral)
{
	static int curr_dci_data_smd;
	static unsigned long iteration;
	struct diag_dci_data_info *temp_data = dci_data_smd;
	if (!temp_data)
		return;
	mutex_lock(&dci_stat_mutex);
	if (curr_dci_data_smd == DIAG_DCI_DEBUG_CNT)
		curr_dci_data_smd = 0;
	temp_data += curr_dci_data_smd;
	temp_data->iteration = iteration + 1;
	temp_data->data_size = read_bytes;
	temp_data->peripheral = peripheral;
	temp_data->ch_type = ch_type;
	diag_get_timestamp(temp_data->time_stamp);
	curr_dci_data_smd++;
	iteration++;
	mutex_unlock(&dci_stat_mutex);
}
#else
void diag_dci_smd_record_info(int read_bytes, uint8_t ch_type,
			      uint8_t peripheral) { }
#endif

static void dci_drain_data(unsigned long data)
{
	queue_work(driver->diag_dci_wq, &dci_data_drain_work);
}

static void dci_check_drain_timer(void)
{
	if (!dci_timer_in_progress) {
		dci_timer_in_progress = 1;
		 mod_timer(&dci_drain_timer, jiffies + msecs_to_jiffies(500));
	}
}

static int diag_dci_init_buffer(struct diag_dci_buffer_t *buffer, int type)
{
	if (!buffer || buffer->data)
		return -EINVAL;

	switch (type) {
	case DCI_BUF_PRIMARY:
		buffer->data = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
		if (!buffer->data)
			return -ENOMEM;
		buffer->capacity = IN_BUF_SIZE;
		break;
	case DCI_BUF_SECONDARY:
		buffer->data = NULL;
		buffer->capacity = IN_BUF_SIZE;
		break;
	case DCI_BUF_CMD:
		buffer->data = kzalloc(PKT_SIZE, GFP_KERNEL);
		if (!buffer->data)
			return -ENOMEM;
		buffer->capacity = PKT_SIZE;
		break;
	default:
		pr_err("diag: In %s, unknown type %d", __func__, type);
		return -EINVAL;
	}

	buffer->data_len = 0;
	buffer->in_busy = 0;
	buffer->buf_type = type;
	mutex_init(&buffer->data_mutex);

	return 0;
}

static inline int diag_dci_check_buffer(struct diag_dci_buffer_t *buf, int len)
{
	if (!buf)
		return -EINVAL;

	
	if ((buf->data_len + len < buf->capacity) && !buf->in_busy)
		return 1;

	return 0;
}

static void dci_add_buffer_to_list(struct diag_dci_client_tbl *client,
				   struct diag_dci_buffer_t *buf)
{
	if (!buf || !client || !buf->data)
		return;

	if (buf->in_list || buf->data_len == 0)
		return;

	mutex_lock(&client->write_buf_mutex);
	list_add_tail(&buf->buf_track, &client->list_write_buf);
	mutex_lock(&buf->data_mutex);
	buf->in_busy = 1;
	buf->in_list = 1;
	mutex_unlock(&buf->data_mutex);
	mutex_unlock(&client->write_buf_mutex);
}

static int diag_dci_get_buffer(struct diag_dci_client_tbl *client,
			       int data_source, int len)
{
	struct diag_dci_buffer_t *buf_primary = NULL;
	struct diag_dci_buffer_t *buf_temp = NULL;
	struct diag_dci_buffer_t *curr = NULL;

	if (!client)
		return -EINVAL;
	if (len < 0 || len > IN_BUF_SIZE)
		return -EINVAL;

	curr = client->buffers[data_source].buf_curr;
	buf_primary = client->buffers[data_source].buf_primary;

	if (curr && diag_dci_check_buffer(curr, len) == 1)
		return 0;

	dci_add_buffer_to_list(client, curr);
	client->buffers[data_source].buf_curr = NULL;

	if (diag_dci_check_buffer(buf_primary, len) == 1) {
		client->buffers[data_source].buf_curr = buf_primary;
		return 0;
	}

	buf_temp = kzalloc(sizeof(struct diag_dci_buffer_t), GFP_KERNEL);
	if (!buf_temp)
		return -EIO;

	if (!diag_dci_init_buffer(buf_temp, DCI_BUF_SECONDARY)) {
		buf_temp->data = diagmem_alloc(driver, driver->itemsize_dci,
					       POOL_TYPE_DCI);
		if (!buf_temp->data) {
			kfree(buf_temp);
			buf_temp = NULL;
			return -ENOMEM;
		}
		client->buffers[data_source].buf_curr = buf_temp;
		return 0;
	}

	kfree(buf_temp);
	buf_temp = NULL;
	return -EIO;
}

void diag_dci_wakeup_clients()
{
	struct list_head *start, *temp;
	struct diag_dci_client_tbl *entry = NULL;

	list_for_each_safe(start, temp, &driver->dci_client_list) {
		entry = list_entry(start, struct diag_dci_client_tbl, track);

		if (!list_empty(&entry->list_write_buf) && !entry->in_service) {
			mutex_lock(&entry->write_buf_mutex);
			entry->in_service = 1;
			mutex_unlock(&entry->write_buf_mutex);
			diag_update_sleeping_process(entry->client->tgid,
						     DCI_DATA_TYPE);
		}
	}
}

void dci_data_drain_work_fn(struct work_struct *work)
{
	int i;
	struct list_head *start, *temp;
	struct diag_dci_client_tbl *entry = NULL;
	struct diag_dci_buf_peripheral_t *proc_buf = NULL;
	struct diag_dci_buffer_t *buf_temp = NULL;

	list_for_each_safe(start, temp, &driver->dci_client_list) {
		entry = list_entry(start, struct diag_dci_client_tbl, track);
		for (i = 0; i < NUM_DCI_PROC; i++) {
			proc_buf = &entry->buffers[i];

			buf_temp = proc_buf->buf_primary;
			if (DCI_CAN_ADD_BUF_TO_LIST(buf_temp))
				dci_add_buffer_to_list(entry, buf_temp);

			buf_temp = proc_buf->buf_cmd;
			if (DCI_CAN_ADD_BUF_TO_LIST(buf_temp))
				dci_add_buffer_to_list(entry, buf_temp);

			buf_temp = proc_buf->buf_curr;
			if (DCI_CAN_ADD_BUF_TO_LIST(buf_temp)) {
				dci_add_buffer_to_list(entry, buf_temp);
				mutex_lock(&proc_buf->buf_mutex);
				proc_buf->buf_curr = NULL;
				mutex_unlock(&proc_buf->buf_mutex);
			}
		}
		if (!list_empty(&entry->list_write_buf) && !entry->in_service) {
			mutex_lock(&entry->write_buf_mutex);
			entry->in_service = 1;
			mutex_unlock(&entry->write_buf_mutex);
			diag_update_sleeping_process(entry->client->tgid,
						     DCI_DATA_TYPE);
		}
	}
	dci_timer_in_progress = 0;
}

void diag_process_apps_dci_read_data(int data_type, void *buf, int recd_bytes)
{
	uint8_t cmd_code;

	if (!buf) {
		pr_err_ratelimited("diag: In %s, Null buf pointer\n", __func__);
		return;
	}

	if (data_type != DATA_TYPE_DCI_LOG && data_type != DATA_TYPE_DCI_EVENT
						&& data_type != DCI_PKT_TYPE) {
		pr_err("diag: In %s, unsupported data_type: 0x%x\n",
				__func__, (unsigned int)data_type);
		return;
	}

	cmd_code = *(uint8_t *)buf;

	switch (cmd_code) {
	case LOG_CMD_CODE:
		extract_dci_log(buf, recd_bytes, APPS_DATA);
		break;
	case EVENT_CMD_CODE:
		extract_dci_events(buf, recd_bytes, APPS_DATA);
		break;
	case DCI_PKT_RSP_CODE:
	case DCI_DELAYED_RSP_CODE:
		extract_dci_pkt_rsp(buf, recd_bytes, APPS_DATA, NULL);
		break;
	default:
		pr_err("diag: In %s, unsupported command code: 0x%x, not log or event\n",
		       __func__, cmd_code);
		return;

	}

	
	diag_dci_wakeup_clients();
	dci_check_drain_timer();
}

int diag_process_smd_dci_read_data(struct diag_smd_info *smd_info, void *buf,
								int recd_bytes)
{
	int read_bytes, dci_pkt_len;
	uint8_t recv_pkt_cmd_code;

	if (driver->num_dci_client == 0) {
		diag_dci_try_deactivate_wakeup_source();
		return 0;
	}

	diag_dci_smd_record_info(recd_bytes, (uint8_t)smd_info->type,
				 (uint8_t)smd_info->peripheral);
	
	read_bytes = 0;
	while (read_bytes < recd_bytes) {
		
		dci_pkt_len = *(uint16_t *)(buf+2);

		if ((dci_pkt_len+5) > (recd_bytes-read_bytes)) {
			pr_err("diag: Invalid length in %s, len: %d, dci_pkt_len: %d",
					__func__, recd_bytes, dci_pkt_len);
			diag_dci_try_deactivate_wakeup_source();
			return 0;
		}
		
		pr_debug("diag: dci: peripheral = %d bytes read = %d, single dci pkt len = %d\n",
			 smd_info->peripheral, read_bytes, dci_pkt_len);
		recv_pkt_cmd_code = *(uint8_t *)(buf+4);
		if (recv_pkt_cmd_code == LOG_CMD_CODE) {
			
			extract_dci_log(buf + 4, recd_bytes - 4,
					smd_info->peripheral);
		} else if (recv_pkt_cmd_code == EVENT_CMD_CODE) {
			
			extract_dci_events(buf + 4, recd_bytes - 4,
					   smd_info->peripheral);
		} else
			extract_dci_pkt_rsp(buf + 4, dci_pkt_len,
					    smd_info->peripheral, smd_info);
		read_bytes += 5 + dci_pkt_len;
		buf += 5 + dci_pkt_len; 
	}

	
	diag_dci_wakeup_clients();
	dci_check_drain_timer();
	diag_dci_try_deactivate_wakeup_source();
	return 0;
}

static inline struct diag_dci_client_tbl *__diag_dci_get_client_entry(
								int client_id)
{
	struct list_head *start, *temp;
	struct diag_dci_client_tbl *entry = NULL;
	list_for_each_safe(start, temp, &driver->dci_client_list) {
		entry = list_entry(start, struct diag_dci_client_tbl, track);
		if (entry->client->tgid == client_id)
			return entry;
	}
	return NULL;
}

static inline int __diag_dci_query_log_mask(struct diag_dci_client_tbl *entry,
							uint16_t log_code)
{
	uint16_t item_num;
	uint8_t equip_id, *log_mask_ptr, byte_mask;
	int byte_index, offset;

	if (!entry) {
		pr_err("diag: In %s, invalid client entry\n", __func__);
		return 0;
	}

	equip_id = LOG_GET_EQUIP_ID(log_code);
	item_num = LOG_GET_ITEM_NUM(log_code);
	byte_index = item_num/8 + 2;
	byte_mask = 0x01 << (item_num % 8);
	offset = equip_id * 514;

	if (offset + byte_index > DCI_LOG_MASK_SIZE) {
		pr_err("diag: In %s, invalid offset: %d, log_code: %d, byte_index: %d\n",
				__func__, offset, log_code, byte_index);
		return 0;
	}

	log_mask_ptr = entry->dci_log_mask;
	log_mask_ptr = log_mask_ptr + offset + byte_index;
	return ((*log_mask_ptr & byte_mask) == byte_mask) ? 1 : 0;

}

static inline int __diag_dci_query_event_mask(struct diag_dci_client_tbl *entry,
							uint16_t event_id)
{
	uint8_t *event_mask_ptr, byte_mask;
	int byte_index, bit_index;

	if (!entry) {
		pr_err("diag: In %s, invalid client entry\n", __func__);
		return 0;
	}

	byte_index = event_id/8;
	bit_index = event_id % 8;
	byte_mask = 0x1 << bit_index;

	if (byte_index > DCI_EVENT_MASK_SIZE) {
		pr_err("diag: In %s, invalid, event_id: %d, byte_index: %d\n",
				__func__, event_id, byte_index);
		return 0;
	}

	event_mask_ptr = entry->dci_event_mask;
	event_mask_ptr = event_mask_ptr + byte_index;
	return ((*event_mask_ptr & byte_mask) == byte_mask) ? 1 : 0;
}

static int diag_dci_filter_commands(struct diag_pkt_header_t *header)
{
	if (!header)
		return -ENOMEM;

	switch (header->cmd_code) {
	case 0x7d: 
	case 0x73: 
	case 0x81: 
	case 0x82: 
	case 0x60: 
		return 1;
	}

	if (header->cmd_code == 0x4b && header->subsys_id == 0x12) {
		switch (header->subsys_cmd_code) {
		case 0x60: 
		case 0x61: 
		case 0x62: 
		case 0x20C: 
		case 0x20D: 
			return 1;
		}
	}

	return 0;
}

static struct dci_pkt_req_entry_t *diag_register_dci_transaction(int uid)
{
	struct dci_pkt_req_entry_t *entry = NULL;
	entry = kzalloc(sizeof(struct dci_pkt_req_entry_t), GFP_KERNEL);
	if (!entry)
		return NULL;

	mutex_lock(&driver->dci_mutex);
	driver->dci_tag++;
	entry->pid = current->tgid;
	entry->uid = uid;
	entry->tag = driver->dci_tag;
	list_add_tail(&entry->track, &driver->dci_req_list);
	mutex_unlock(&driver->dci_mutex);

	return entry;
}

static struct dci_pkt_req_entry_t *diag_dci_get_request_entry(int tag)
{
	struct list_head *start, *temp;
	struct dci_pkt_req_entry_t *entry = NULL;
	list_for_each_safe(start, temp, &driver->dci_req_list) {
		entry = list_entry(start, struct dci_pkt_req_entry_t, track);
		if (entry->tag == tag)
			return entry;
	}
	return NULL;
}

static int diag_dci_remove_req_entry(unsigned char *buf, int len,
				     struct dci_pkt_req_entry_t *entry)
{
	uint16_t rsp_count = 0, delayed_rsp_id = 0;
	if (!buf || len <= 0 || !entry) {
		pr_err("diag: In %s, invalid input buf: %p, len: %d, entry: %p\n",
			__func__, buf, len, entry);
		return -EIO;
	}

	
	if (*buf != 0x80) {
		list_del(&entry->track);
		kfree(entry);
		return 1;
	}

	
	if (len < MIN_DELAYED_RSP_LEN) {
		pr_err("diag: Invalid delayed rsp packet length %d\n", len);
		return -EINVAL;
	}

	delayed_rsp_id = *(uint16_t *)(buf + 8);
	if (delayed_rsp_id == 0) {
		list_del(&entry->track);
		kfree(entry);
		return 1;
	}

	rsp_count = *(uint16_t *)(buf + 10);
	if (rsp_count > 0 && rsp_count < 0x1000) {
		list_del(&entry->track);
		kfree(entry);
		return 1;
	}

	return 0;
}

void extract_dci_pkt_rsp(unsigned char *buf, int len, int data_source,
			 struct diag_smd_info *smd_info)
{
	int tag, curr_client_pid = 0;
	struct diag_dci_client_tbl *entry = NULL;
	void *temp_buf = NULL;
	uint8_t dci_cmd_code, cmd_code_len, delete_flag = 0;
	uint32_t rsp_len = 0;
	struct diag_dci_buffer_t *rsp_buf = NULL;
	struct dci_pkt_req_entry_t *req_entry = NULL;
	unsigned char *temp = buf;

	if (!buf) {
		pr_err("diag: Invalid pointer in %s\n", __func__);
		return;
	}
	dci_cmd_code = *(uint8_t *)(temp);
	if (dci_cmd_code == DCI_PKT_RSP_CODE) {
		cmd_code_len = sizeof(uint8_t);
	} else if (dci_cmd_code == DCI_DELAYED_RSP_CODE) {
		cmd_code_len = sizeof(uint32_t);
	} else {
		pr_err("diag: In %s, invalid command code %d\n", __func__,
								dci_cmd_code);
		return;
	}
	temp += cmd_code_len;
	tag = *(int *)temp;
	temp += sizeof(int);

	rsp_len = len - (cmd_code_len + sizeof(int));
	if ((rsp_len == 0) || (rsp_len > (len - 5))) {
		pr_err("diag: Invalid length in %s, len: %d, rsp_len: %d",
						__func__, len, rsp_len);
		return;
	}

	req_entry = diag_dci_get_request_entry(tag);
	if (!req_entry) {
		pr_err("diag: No matching PID for DCI data\n");
		return;
	}
	curr_client_pid = req_entry->pid;

	
	mutex_lock(&driver->dci_mutex);
	delete_flag = diag_dci_remove_req_entry(temp, rsp_len, req_entry);
	if ((delete_flag != 0) && (delete_flag != 1)) {
		mutex_unlock(&driver->dci_mutex);
		return;
	}
	mutex_unlock(&driver->dci_mutex);

	entry = __diag_dci_get_client_entry(curr_client_pid);
	if (!entry) {
		pr_err("diag: In %s, couldn't find entry\n", __func__);
		return;
	}
	rsp_buf = entry->buffers[data_source].buf_cmd;

	mutex_lock(&rsp_buf->data_mutex);
	if ((rsp_buf->data_len + 9 + rsp_len) > rsp_buf->capacity) {
		pr_alert("diag: create capacity for pkt rsp\n");
		rsp_buf->capacity += 9 + rsp_len;
		temp_buf = krealloc(rsp_buf->data, rsp_buf->capacity,
				    GFP_KERNEL);
		if (!temp_buf) {
			pr_err("diag: DCI realloc failed\n");
			mutex_unlock(&rsp_buf->data_mutex);
			return;
		} else {
			rsp_buf->data = temp_buf;
		}
	}

	
	*(int *)(rsp_buf->data + rsp_buf->data_len) = DCI_PKT_RSP_TYPE;
	rsp_buf->data_len += sizeof(int);
	
	*(int *)(rsp_buf->data + rsp_buf->data_len) = rsp_len + sizeof(int);
	rsp_buf->data_len += sizeof(int);
	*(uint8_t *)(rsp_buf->data + rsp_buf->data_len) = delete_flag;
	rsp_buf->data_len += sizeof(uint8_t);
	*(int *)(rsp_buf->data + rsp_buf->data_len) = req_entry->uid;
	rsp_buf->data_len += sizeof(int);
	memcpy(rsp_buf->data + rsp_buf->data_len, temp, rsp_len);
	rsp_buf->data_len += rsp_len;
	rsp_buf->data_source = data_source;
	if (smd_info)
		smd_info->in_busy_1 = 1;
	mutex_unlock(&rsp_buf->data_mutex);


	dci_add_buffer_to_list(entry, rsp_buf);
}

static void copy_dci_event(unsigned char *buf, int len,
			   struct diag_dci_client_tbl *client, int data_source)
{
	struct diag_dci_buffer_t *data_buffer = NULL;
	struct diag_dci_buf_peripheral_t *proc_buf = NULL;
	int err = 0, total_len = 0;

	if (!buf || !client) {
		pr_err("diag: Invalid pointers in %s", __func__);
		return;
	}

	total_len = sizeof(int) + len;

	proc_buf = &client->buffers[data_source];
	mutex_lock(&proc_buf->buf_mutex);
	mutex_lock(&proc_buf->health_mutex);
	err = diag_dci_get_buffer(client, data_source, total_len);
	if (err) {
		if (err == -ENOMEM)
			proc_buf->health.dropped_events++;
		else
			pr_err("diag: In %s, invalid packet\n", __func__);
		mutex_unlock(&proc_buf->health_mutex);
		mutex_unlock(&proc_buf->buf_mutex);
		return;
	}

	data_buffer = proc_buf->buf_curr;

	proc_buf->health.received_events++;
	mutex_unlock(&proc_buf->health_mutex);
	mutex_unlock(&proc_buf->buf_mutex);

	mutex_lock(&data_buffer->data_mutex);
	*(int *)(data_buffer->data + data_buffer->data_len) = DCI_EVENT_TYPE;
	data_buffer->data_len += sizeof(int);
	memcpy(data_buffer->data + data_buffer->data_len, buf, len);
	data_buffer->data_len += len;
	data_buffer->data_source = data_source;
	mutex_unlock(&data_buffer->data_mutex);

}

void extract_dci_events(unsigned char *buf, int len, int data_source)
{
	uint16_t event_id, event_id_packet, length, temp_len;
	uint8_t payload_len, payload_len_field;
	uint8_t timestamp[8], timestamp_len;
	unsigned char event_data[MAX_EVENT_SIZE];
	unsigned int total_event_len;
	struct list_head *start, *temp;
	struct diag_dci_client_tbl *entry = NULL;

	length =  *(uint16_t *)(buf + 1); 
	if (length == 0) {
		pr_err("diag: Incoming dci event length is invalid\n");
		return;
	}
	temp_len = 3;
	while (temp_len < (length - 1)) {
		event_id_packet = *(uint16_t *)(buf + temp_len);
		event_id = event_id_packet & 0x0FFF; 
		if (event_id_packet & 0x8000) {
			timestamp_len = 2;
		} else {
			timestamp_len = 8;
			memcpy(timestamp, buf + temp_len + 2, timestamp_len);
		}
		
		if (((event_id_packet & 0x6000) >> 13) == 3) {
			payload_len_field = 1;
			payload_len = *(uint8_t *)
					(buf + temp_len + 2 + timestamp_len);
			if (payload_len < (MAX_EVENT_SIZE - 13)) {
				
				memcpy(event_data + 12, buf + temp_len + 2 +
							timestamp_len, 1);
				memcpy(event_data + 13, buf + temp_len + 2 +
					timestamp_len + 1, payload_len);
			} else {
				pr_err("diag: event > %d, payload_len = %d\n",
					(MAX_EVENT_SIZE - 13), payload_len);
				return;
			}
		} else {
			payload_len_field = 0;
			payload_len = (event_id_packet & 0x6000) >> 13;
			
			memcpy(event_data + 12, buf + temp_len + 2 +
						timestamp_len, payload_len);
		}

		temp_len += sizeof(uint16_t) + timestamp_len +
						payload_len_field + payload_len;
		if (temp_len > len) {
			pr_err("diag: Invalid length in %s, len: %d, read: %d",
						__func__, len, temp_len);
			return;
		}

		*(uint16_t *)(event_data) = 10 +
					payload_len_field + payload_len;
		*(uint16_t *)(event_data + 2) = event_id_packet & 0x7FFF;
		memcpy(event_data + 4, timestamp, 8);
		total_event_len = 2 + 10 + payload_len_field + payload_len;
		
		list_for_each_safe(start, temp, &driver->dci_client_list) {
			entry = list_entry(start, struct diag_dci_client_tbl,
									track);
			if (__diag_dci_query_event_mask(entry, event_id)) {
				
				copy_dci_event(event_data, total_event_len,
					       entry, data_source);
			}
		}
	}
}

static void copy_dci_log(unsigned char *buf, int len,
			 struct diag_dci_client_tbl *client, int data_source)
{
	uint16_t log_length = 0;
	struct diag_dci_buffer_t *data_buffer = NULL;
	struct diag_dci_buf_peripheral_t *proc_buf = NULL;
	int err = 0, total_len = 0;

	if (!buf || !client) {
		pr_err("diag: Invalid pointers in %s", __func__);
		return;
	}

	log_length = *(uint16_t *)(buf + 2);
	if (log_length > USHRT_MAX - 4) {
		pr_err("diag: Integer overflow in %s, log_len: %d",
				__func__, log_length);
		return;
	}
	total_len = sizeof(int) + log_length;

	if ((log_length + sizeof(uint16_t) + 2) > len) {
		pr_err("diag: Invalid length in %s, log_len: %d, len: %d",
						__func__, log_length, len);
		return;
	}

	proc_buf = &client->buffers[data_source];
	mutex_lock(&proc_buf->buf_mutex);
	mutex_lock(&proc_buf->health_mutex);
	err = diag_dci_get_buffer(client, data_source, total_len);
	if (err) {
		if (err == -ENOMEM)
			proc_buf->health.dropped_logs++;
		else
			pr_err("diag: In %s, invalid packet\n", __func__);
		mutex_unlock(&proc_buf->health_mutex);
		mutex_unlock(&proc_buf->buf_mutex);
		return;
	}

	data_buffer = proc_buf->buf_curr;
	proc_buf->health.received_logs++;
	mutex_unlock(&proc_buf->health_mutex);
	mutex_unlock(&proc_buf->buf_mutex);

	mutex_lock(&data_buffer->data_mutex);
	if (!data_buffer->data) {
		mutex_unlock(&data_buffer->data_mutex);
		return;
	}

	*(int *)(data_buffer->data + data_buffer->data_len) = DCI_LOG_TYPE;
	data_buffer->data_len += sizeof(int);
	memcpy(data_buffer->data + data_buffer->data_len, buf + sizeof(int),
	       log_length);
	data_buffer->data_len += log_length;
	data_buffer->data_source = data_source;
	mutex_unlock(&data_buffer->data_mutex);
}

void extract_dci_log(unsigned char *buf, int len, int data_source)
{
	uint16_t log_code, read_bytes = 0;
	struct list_head *start, *temp;
	struct diag_dci_client_tbl *entry = NULL;

	if (!buf) {
		pr_err("diag: In %s buffer is NULL\n", __func__);
		return;
	}

	log_code = *(uint16_t *)(buf + 6);
	read_bytes += sizeof(uint16_t) + 6;
	if (read_bytes > len) {
		pr_err("diag: Invalid length in %s, len: %d, read: %d",
						__func__, len, read_bytes);
		return;
	}

	
	list_for_each_safe(start, temp, &driver->dci_client_list) {
		entry = list_entry(start, struct diag_dci_client_tbl, track);
		if (__diag_dci_query_log_mask(entry, log_code)) {
			pr_debug("\t log code %x needed by client %d",
				 log_code, entry->client->tgid);
			
			copy_dci_log(buf, len, entry, data_source);
		}
	}
}

void diag_update_smd_dci_work_fn(struct work_struct *work)
{
	struct diag_smd_info *smd_info = container_of(work,
						struct diag_smd_info,
						diag_notify_update_smd_work);
	int i, j;
	char dirty_bits[16];
	uint8_t *client_log_mask_ptr;
	uint8_t *log_mask_ptr;
	int ret;
	struct list_head *start, *temp;
	struct diag_dci_client_tbl *entry = NULL;

	
	memset(dirty_bits, 0, 16 * sizeof(uint8_t));

	list_for_each_safe(start, temp, &driver->dci_client_list) {
		entry = list_entry(start, struct diag_dci_client_tbl, track);
		client_log_mask_ptr = entry->dci_log_mask;
		for (j = 0; j < 16; j++) {
			if (*(client_log_mask_ptr+1))
				dirty_bits[j] = 1;
			client_log_mask_ptr += 514;
		}
	}

	mutex_lock(&dci_log_mask_mutex);
	
	log_mask_ptr = dci_cumulative_log_mask;
	for (i = 0; i < 16; i++) {
		if (dirty_bits[i])
			*(log_mask_ptr+1) = dirty_bits[i];

		log_mask_ptr += 514;
	}
	mutex_unlock(&dci_log_mask_mutex);

	
	diag_update_userspace_clients(DCI_LOG_MASKS_TYPE);
	
	ret = diag_send_dci_log_mask();

	
	diag_update_userspace_clients(DCI_EVENT_MASKS_TYPE);
	
	ret = diag_send_dci_event_mask();

	smd_info->notify_context = 0;
}

void diag_dci_notify_client(int peripheral_mask, int data)
{
	int stat;
	struct siginfo info;
	struct list_head *start, *temp;
	struct diag_dci_client_tbl *entry = NULL;

	memset(&info, 0, sizeof(struct siginfo));
	info.si_code = SI_QUEUE;
	info.si_int = (peripheral_mask | data);

	
	list_for_each_safe(start, temp, &driver->dci_client_list) {
		entry = list_entry(start, struct diag_dci_client_tbl, track);
		if (entry->client_info.notification_list & peripheral_mask) {
			info.si_signo = entry->client_info.signal_type;
			stat = send_sig_info(entry->client_info.signal_type,
					     &info, entry->client);
			if (stat)
				pr_err("diag: Err sending dci signal to client, signal data: 0x%x, stat: %d\n",
							info.si_int, stat);
		}
	}
}

static int diag_send_dci_pkt(struct diag_master_table entry,
			     unsigned char *buf, int len, int tag)
{
	int i, status = DIAG_DCI_NO_ERROR;
	unsigned int read_len = 0;

	if (len < DCI_PKT_REQ_MIN_LEN) {
		pr_err("diag: dci: Invalid pkt len %d in %s\n", len, __func__);
		return -EIO;
	}
	if (len > APPS_BUF_SIZE - 10) {
		pr_err("diag: dci: Invalid payload length in %s\n", __func__);
		return -EIO;
	}
	
	buf = buf + sizeof(int);
	read_len += sizeof(int);
	len = len - sizeof(int);
	mutex_lock(&driver->dci_mutex);
	
	driver->apps_dci_buf[0] = CONTROL_CHAR; 
	driver->apps_dci_buf[1] = 1; 
	*(uint16_t *)(driver->apps_dci_buf + 2) = len + 4 + 1; 
	driver->apps_dci_buf[4] = DCI_PKT_RSP_CODE;
	*(int *)(driver->apps_dci_buf + 5) = tag;
	for (i = 0; i < len; i++)
		driver->apps_dci_buf[i+9] = *(buf+i);
	read_len += len;
	driver->apps_dci_buf[9+len] = CONTROL_CHAR; 
	if ((read_len + 9) >= USER_SPACE_DATA) {
		pr_err("diag: dci: Invalid length while forming dci pkt in %s",
								__func__);
		mutex_unlock(&driver->dci_mutex);
		return -EIO;
	}
	
	if (entry.client_id == APPS_DATA) {
		driver->dci_pkt_length = len + 10;
		diag_update_pkt_buffer(driver->apps_dci_buf, DCI_PKT_TYPE);
		diag_update_sleeping_process(entry.process_id, DCI_PKT_TYPE);
		mutex_unlock(&driver->dci_mutex);
		return DIAG_DCI_NO_ERROR;
	}

	for (i = 0; i < NUM_SMD_DCI_CHANNELS; i++)
		if (entry.client_id == i) {
			status = 1;
			break;
		}

	if (status) {
		status = diag_dci_write_proc(entry.client_id,
					     DIAG_DATA_TYPE,
					     driver->apps_dci_buf,
					     len + 10);
	} else {
		pr_err("diag: Cannot send packet to peripheral %d",
		       entry.client_id);
		status = DIAG_DCI_SEND_DATA_FAIL;
	}
	mutex_unlock(&driver->dci_mutex);
	return status;
}

static int diag_dci_process_apps_pkt(struct diag_pkt_header_t *pkt_header,
				     unsigned char *req_buf, int tag)
{
	uint8_t cmd_code, subsys_id, i, goto_download = 0;
	uint8_t header_len = sizeof(struct diag_dci_pkt_header_t);
	uint16_t ss_cmd_code;
	uint32_t write_len = 0;
	unsigned char *dest_buf = driver->apps_dci_buf;
	unsigned char *payload_ptr = driver->apps_dci_buf + header_len;
	struct diag_dci_pkt_header_t dci_header;

	if (!pkt_header || !req_buf || tag < 0)
		return -EIO;

	cmd_code = pkt_header->cmd_code;
	subsys_id = pkt_header->subsys_id;
	ss_cmd_code = pkt_header->subsys_cmd_code;

	if (cmd_code == DIAG_CMD_DOWNLOAD) {
		*payload_ptr = DIAG_CMD_DOWNLOAD;
		write_len = sizeof(uint8_t);
		goto_download = 1;
		goto fill_buffer;
	} else if (cmd_code == DIAG_CMD_VERSION) {
		if (chk_polling_response()) {
			for (i = 0; i < 55; i++, write_len++, payload_ptr++)
				*(payload_ptr) = 0;
			goto fill_buffer;
		}
	} else if (cmd_code == DIAG_CMD_EXT_BUILD) {
		if (chk_polling_response()) {
			*payload_ptr = DIAG_CMD_EXT_BUILD;
			write_len = sizeof(uint8_t);
			payload_ptr += sizeof(uint8_t);
			for (i = 0; i < 8; i++, write_len++, payload_ptr++)
				*(payload_ptr) = 0;
			*(int *)(payload_ptr) = chk_config_get_id();
			write_len += sizeof(int);
			goto fill_buffer;
		}
	} else if (cmd_code == DIAG_CMD_LOG_ON_DMND) {
		if (driver->log_on_demand_support) {
			*payload_ptr = DIAG_CMD_LOG_ON_DMND;
			write_len = sizeof(uint8_t);
			payload_ptr += sizeof(uint8_t);
			*(uint16_t *)(payload_ptr) = *(uint16_t *)(req_buf + 1);
			write_len += sizeof(uint16_t);
			payload_ptr += sizeof(uint16_t);
			*payload_ptr = 0x1; 
			write_len += sizeof(uint8_t);
			goto fill_buffer;
		}
	} else if (cmd_code != DIAG_CMD_DIAG_SUBSYS) {
		return DIAG_DCI_TABLE_ERR;
	}

	if (subsys_id == DIAG_SS_DIAG) {
		if (ss_cmd_code == DIAG_DIAG_MAX_PKT_SZ) {
			memcpy(payload_ptr, pkt_header,
					sizeof(struct diag_pkt_header_t));
			write_len = sizeof(struct diag_pkt_header_t);
			*(uint32_t *)(payload_ptr + write_len) = PKT_SIZE;
			write_len += sizeof(uint32_t);
		} else if (ss_cmd_code == DIAG_DIAG_STM) {
			write_len = diag_process_stm_cmd(req_buf, payload_ptr);
		}
	} else if (subsys_id == DIAG_SS_PARAMS) {
		if (ss_cmd_code == DIAG_DIAG_POLL) {
			if (chk_polling_response()) {
				memcpy(payload_ptr, pkt_header,
					sizeof(struct diag_pkt_header_t));
				write_len = sizeof(struct diag_pkt_header_t);
				payload_ptr += write_len;
				for (i = 0; i < 12; i++, write_len++) {
					*(payload_ptr) = 0;
					payload_ptr++;
				}
			}
		} else if (ss_cmd_code == DIAG_DEL_RSP_WRAP) {
			memcpy(payload_ptr, pkt_header,
					sizeof(struct diag_pkt_header_t));
			write_len = sizeof(struct diag_pkt_header_t);
			*(int *)(payload_ptr + write_len) = wrap_enabled;
			write_len += sizeof(int);
		} else if (ss_cmd_code == DIAG_DEL_RSP_WRAP_CNT) {
			wrap_enabled = true;
			memcpy(payload_ptr, pkt_header,
					sizeof(struct diag_pkt_header_t));
			write_len = sizeof(struct diag_pkt_header_t);
			*(uint16_t *)(payload_ptr + write_len) = wrap_count;
			write_len += sizeof(uint16_t);
		}
	}

fill_buffer:
	if (write_len > 0) {
		
		if (write_len + header_len > PKT_SIZE) {
			pr_err("diag: In %s, invalid length %d\n", __func__,
						write_len + header_len);
			return -ENOMEM;
		}
		dci_header.start = CONTROL_CHAR;
		dci_header.version = 1;
		dci_header.len = write_len + sizeof(uint8_t) + sizeof(int);
		dci_header.pkt_code = DCI_PKT_RSP_CODE;
		dci_header.tag = tag;
		driver->in_busy_dcipktdata = 1;
		memcpy(dest_buf, &dci_header, header_len);
		diag_process_apps_dci_read_data(DCI_PKT_TYPE, dest_buf + 4,
						dci_header.len);
		driver->in_busy_dcipktdata = 0;

		if (goto_download) {
			usleep_range(5000, 5100);
			
			msm_set_restart_mode(RESTART_DLOAD);
			pr_alert("diag: download mode set, Rebooting SoC..\n");
			kernel_restart(NULL);
		}
		return DIAG_DCI_NO_ERROR;
	}

	return DIAG_DCI_TABLE_ERR;
}

static int diag_process_dci_pkt_rsp(unsigned char *buf, int len)
{
	int req_uid, ret = DIAG_DCI_TABLE_ERR, i;
	struct diag_pkt_header_t *header = NULL;
	unsigned char *temp = buf;
	unsigned char *req_buf = NULL;
	uint8_t retry_count = 0, max_retries = 3, found = 0;
	uint32_t read_len = 0;
	struct diag_master_table entry;
	struct dci_pkt_req_entry_t *req_entry = NULL;

	if (!buf)
		return -EIO;

	if (len < DCI_PKT_REQ_MIN_LEN || len > USER_SPACE_DATA) {
		pr_err("diag: dci: Invalid length %d len in %s", len, __func__);
		return -EIO;
	}

	req_uid = *(int *)temp; 
	temp += sizeof(int);
	req_buf = temp; 
	header = (struct diag_pkt_header_t *)temp;
	temp += sizeof(struct diag_pkt_header_t);
	read_len = sizeof(int) + sizeof(struct diag_pkt_header_t);
	if (read_len >= USER_SPACE_DATA) {
		pr_err("diag: dci: Invalid length in %s\n", __func__);
		return -EIO;
	}

	
	if (diag_dci_filter_commands(header)) {
		pr_debug("diag: command not supported %d %d %d",
			 header->cmd_code, header->subsys_id,
			 header->subsys_cmd_code);
		return DIAG_DCI_SEND_DATA_FAIL;
	}

	while (retry_count < max_retries) {
		retry_count++;
		if (driver->in_busy_dcipktdata)
			usleep_range(10000, 10100);
		else
			break;
	}
	
	if (driver->in_busy_dcipktdata) {
		pr_err("diag: In %s, apps dci buffer is still busy. Dropping packet\n",
								__func__);
		return -EAGAIN;
	}

	
	req_entry = diag_register_dci_transaction(req_uid);
	if (!req_entry) {
		pr_alert("diag: registering new DCI transaction failed\n");
		return DIAG_DCI_NO_REG;
	}

	
	ret = diag_dci_process_apps_pkt(header, req_buf, req_entry->tag);
	if (ret == DIAG_DCI_NO_ERROR || ret < 0)
		return ret;

	
	for (i = 0; i < diag_max_reg && !found; i++) {
		entry = driver->table[i];
		if (entry.process_id == NO_PROCESS)
			continue;
		if (entry.cmd_code == header->cmd_code &&
			    entry.subsys_id == header->subsys_id &&
			    entry.cmd_code_lo <= header->subsys_cmd_code &&
			    entry.cmd_code_hi >= header->subsys_cmd_code) {
			ret = diag_send_dci_pkt(entry, buf, len,
						req_entry->tag);
			found = 1;
		} else if (entry.cmd_code == 255 && header->cmd_code == 75) {
			if (entry.subsys_id == header->subsys_id &&
			    entry.cmd_code_lo <= header->subsys_cmd_code &&
			    entry.cmd_code_hi >= header->subsys_cmd_code) {
				ret = diag_send_dci_pkt(entry, buf, len,
							req_entry->tag);
				found = 1;
			}
		} else if (entry.cmd_code == 255 && entry.subsys_id == 255) {
			if (entry.cmd_code_lo <= header->cmd_code &&
			    entry.cmd_code_hi >= header->cmd_code) {
				if (entry.cmd_code_lo == MODE_CMD &&
				    entry.cmd_code_hi == MODE_CMD)
					if (entry.client_id != APPS_DATA)
						continue;
					ret = diag_send_dci_pkt(entry, buf, len,
								req_entry->tag);
					found = 1;
			}
		}
	}

	return ret;
}

int diag_process_dci_transaction(unsigned char *buf, int len)
{
	unsigned char *temp = buf;
	uint16_t log_code, item_num;
	int ret = -1, found = 0;
	int count, set_mask, num_codes, bit_index, event_id, offset = 0;
	unsigned int byte_index, read_len = 0;
	uint8_t equip_id, *log_mask_ptr, *head_log_mask_ptr, byte_mask;
	uint8_t *event_mask_ptr;
	struct diag_dci_client_tbl *dci_entry = NULL;

	if (!temp) {
		pr_err("diag: Invalid buffer in %s\n", __func__);
		return -ENOMEM;
	}

	
	if (*(int *)temp > 0) {
		return diag_process_dci_pkt_rsp(buf, len);
	} else if (*(int *)temp == DCI_LOG_TYPE) {
		if (len < DCI_LOG_CON_MIN_LEN || len > USER_SPACE_DATA) {
			pr_err("diag: dci: Invalid length in %s\n", __func__);
			return -EIO;
		}
		
		dci_entry = diag_dci_get_client_entry();
		if (!dci_entry) {
			pr_err("diag: In %s, invalid client\n", __func__);
			return ret;
		}

		
		temp += sizeof(int);
		read_len += sizeof(int);
		set_mask = *(int *)temp;
		temp += sizeof(int);
		read_len += sizeof(int);
		num_codes = *(int *)temp;
		temp += sizeof(int);
		read_len += sizeof(int);

		if (num_codes == 0 || (num_codes >= (USER_SPACE_DATA - 8)/2)) {
			pr_err("diag: dci: Invalid number of log codes %d\n",
								num_codes);
			return -EIO;
		}

		head_log_mask_ptr = dci_entry->dci_log_mask;
		if (!head_log_mask_ptr) {
			pr_err("diag: dci: Invalid Log mask pointer in %s\n",
								__func__);
			return -ENOMEM;
		}
		pr_debug("diag: head of dci log mask %p\n", head_log_mask_ptr);
		count = 0; 
		while (count < num_codes) {
			if (read_len >= USER_SPACE_DATA) {
				pr_err("diag: dci: Invalid length for log type in %s",
								__func__);
				return -EIO;
			}
			log_code = *(uint16_t *)temp;
			equip_id = LOG_GET_EQUIP_ID(log_code);
			item_num = LOG_GET_ITEM_NUM(log_code);
			byte_index = item_num/8 + 2;
			if (byte_index >= (DCI_MAX_ITEMS_PER_LOG_CODE+2)) {
				pr_err("diag: dci: Log type, invalid byte index\n");
				return ret;
			}
			byte_mask = 0x01 << (item_num % 8);
			log_mask_ptr = head_log_mask_ptr;
			found = 0;
			offset = 0;
			while (log_mask_ptr && (offset < DCI_LOG_MASK_SIZE)) {
				if (*log_mask_ptr == equip_id) {
					found = 1;
					pr_debug("diag: find equip id = %x at %p\n",
						 equip_id, log_mask_ptr);
					break;
				} else {
					pr_debug("diag: did not find equip id = %x at %p\n",
						 equip_id, log_mask_ptr);
					log_mask_ptr += 514;
					offset += 514;
				}
			}
			if (!found) {
				pr_err("diag: dci equip id not found\n");
				return ret;
			}
			*(log_mask_ptr+1) = 1; 
			log_mask_ptr = log_mask_ptr + byte_index;
			if (set_mask)
				*log_mask_ptr |= byte_mask;
			else
				*log_mask_ptr &= ~byte_mask;
			
			update_dci_cumulative_log_mask(
				offset, byte_index,
				byte_mask);
			temp += 2;
			read_len += 2;
			count++;
			ret = DIAG_DCI_NO_ERROR;
		}
		
		diag_update_userspace_clients(DCI_LOG_MASKS_TYPE);
		
		ret = diag_send_dci_log_mask();
	} else if (*(int *)temp == DCI_EVENT_TYPE) {
		if (len < DCI_EVENT_CON_MIN_LEN || len > USER_SPACE_DATA) {
			pr_err("diag: dci: Invalid length in %s\n", __func__);
			return -EIO;
		}
		
		dci_entry = diag_dci_get_client_entry();
		if (!dci_entry) {
			pr_err("diag: In %s, invalid client\n", __func__);
			return ret;
		}
		
		temp += sizeof(int);
		read_len += sizeof(int);
		set_mask = *(int *)temp;
		temp += sizeof(int);
		read_len += sizeof(int);
		num_codes = *(int *)temp;
		temp += sizeof(int);
		read_len += sizeof(int);

		if (num_codes == 0 || (num_codes >= (USER_SPACE_DATA - 8)/2)) {
			pr_err("diag: dci: Invalid number of event ids %d\n",
								num_codes);
			return -EIO;
		}

		event_mask_ptr = dci_entry->dci_event_mask;
		if (!event_mask_ptr) {
			pr_err("diag: dci: Invalid event mask pointer in %s\n",
								__func__);
			return -ENOMEM;
		}
		pr_debug("diag: head of dci event mask %p\n", event_mask_ptr);
		count = 0; 
		while (count < num_codes) {
			if (read_len >= USER_SPACE_DATA) {
				pr_err("diag: dci: Invalid length for event type in %s",
								__func__);
				return -EIO;
			}
			event_id = *(int *)temp;
			byte_index = event_id/8;
			if (byte_index >= DCI_EVENT_MASK_SIZE) {
				pr_err("diag: dci: Event type, invalid byte index\n");
				return ret;
			}
			bit_index = event_id % 8;
			byte_mask = 0x1 << bit_index;
			if (set_mask)
				*(event_mask_ptr + byte_index) |= byte_mask;
			else
				*(event_mask_ptr + byte_index) &= ~byte_mask;
			
			update_dci_cumulative_event_mask(byte_index, byte_mask);
			temp += sizeof(int);
			read_len += sizeof(int);
			count++;
			ret = DIAG_DCI_NO_ERROR;
		}
		
		diag_update_userspace_clients(DCI_EVENT_MASKS_TYPE);
		
		ret = diag_send_dci_event_mask();
	} else {
		pr_alert("diag: Incorrect DCI transaction\n");
	}
	return ret;
}


struct diag_dci_client_tbl *diag_dci_get_client_entry()
{
	return __diag_dci_get_client_entry(current->tgid);
}

void update_dci_cumulative_event_mask(int offset, uint8_t byte_mask)
{
	uint8_t *event_mask_ptr;
	uint8_t *update_ptr = dci_cumulative_event_mask;
	struct list_head *start, *temp;
	struct diag_dci_client_tbl *entry = NULL;
	bool is_set = false;

	mutex_lock(&dci_event_mask_mutex);
	update_ptr += offset;
	list_for_each_safe(start, temp, &driver->dci_client_list) {
		entry = list_entry(start, struct diag_dci_client_tbl, track);
		event_mask_ptr = entry->dci_event_mask;
		event_mask_ptr += offset;
		if ((*event_mask_ptr & byte_mask) == byte_mask) {
			is_set = true;
			
			break;
		}
	}
	if (is_set == false)
		*update_ptr &= ~byte_mask;
	else
		*update_ptr |= byte_mask;
	mutex_unlock(&dci_event_mask_mutex);
}

void diag_dci_invalidate_cumulative_event_mask()
{
	int i = 0;
	struct list_head *start, *temp;
	struct diag_dci_client_tbl *entry = NULL;
	uint8_t *update_ptr, *event_mask_ptr;
	update_ptr = dci_cumulative_event_mask;

	mutex_lock(&dci_event_mask_mutex);
	list_for_each_safe(start, temp, &driver->dci_client_list) {
		entry = list_entry(start, struct diag_dci_client_tbl, track);
		event_mask_ptr = entry->dci_event_mask;
		for (i = 0; i < DCI_EVENT_MASK_SIZE; i++)
			*(update_ptr+i) |= *(event_mask_ptr+i);
	}
	mutex_unlock(&dci_event_mask_mutex);
}

int diag_send_dci_event_mask()
{
	void *buf = driver->buf_event_mask_update;
	int header_size = sizeof(struct diag_ctrl_event_mask);
	int ret = DIAG_DCI_NO_ERROR, err = DIAG_DCI_NO_ERROR, i;

	mutex_lock(&driver->diag_cntl_mutex);
	
	driver->event_mask->cmd_type = DIAG_CTRL_MSG_EVENT_MASK;
	driver->event_mask->data_len = 7 + DCI_EVENT_MASK_SIZE;
	driver->event_mask->stream_id = DCI_MASK_STREAM;
	driver->event_mask->status = 3; 
	driver->event_mask->event_config = 0; 
	driver->event_mask->event_mask_size = DCI_EVENT_MASK_SIZE;
	for (i = 0; i < DCI_EVENT_MASK_SIZE; i++) {
		if (dci_cumulative_event_mask[i] != 0) {
			driver->event_mask->event_config = 1;
			break;
		}
	}
	memcpy(buf, driver->event_mask, header_size);
	memcpy(buf+header_size, dci_cumulative_event_mask, DCI_EVENT_MASK_SIZE);
	for (i = 0; i < NUM_SMD_DCI_CHANNELS; i++) {
		if (!driver->smd_dci[i].ch)
			continue;
		err = diag_dci_write_proc(i, DIAG_CNTL_TYPE, buf,
					  header_size + DCI_EVENT_MASK_SIZE);
		if (err != DIAG_DCI_NO_ERROR)
			ret = DIAG_DCI_SEND_DATA_FAIL;
	}
	mutex_unlock(&driver->diag_cntl_mutex);

	return ret;
}

void update_dci_cumulative_log_mask(int offset, unsigned int byte_index,
						uint8_t byte_mask)
{
	int i;
	uint8_t *update_ptr = dci_cumulative_log_mask;
	uint8_t *log_mask_ptr;
	bool is_set = false;
	struct list_head *start, *temp;
	struct diag_dci_client_tbl *entry = NULL;

	mutex_lock(&dci_log_mask_mutex);
	*update_ptr = 0;
	
	for (i = 0; i < 16; i++)
		*(update_ptr + (i*514)) = i;

	update_ptr += offset;
	
	*(update_ptr+1) = 1;
	update_ptr = update_ptr + byte_index;
	list_for_each_safe(start, temp, &driver->dci_client_list) {
		entry = list_entry(start, struct diag_dci_client_tbl, track);
		log_mask_ptr = entry->dci_log_mask;
		log_mask_ptr = log_mask_ptr + offset + byte_index;
		if ((*log_mask_ptr & byte_mask) == byte_mask) {
			is_set = true;
			
			break;
		}
	}

	if (is_set == false)
		*update_ptr &= ~byte_mask;
	else
		*update_ptr |= byte_mask;
	mutex_unlock(&dci_log_mask_mutex);
}

void diag_dci_invalidate_cumulative_log_mask()
{
	int i = 0;
	struct list_head *start, *temp;
	struct diag_dci_client_tbl *entry = NULL;
	uint8_t *update_ptr, *log_mask_ptr;
	update_ptr = dci_cumulative_log_mask;

	mutex_lock(&dci_log_mask_mutex);
	list_for_each_safe(start, temp, &driver->dci_client_list) {
		entry = list_entry(start, struct diag_dci_client_tbl, track);
		log_mask_ptr = entry->dci_log_mask;
		for (i = 0; i < DCI_LOG_MASK_SIZE; i++)
			*(update_ptr+i) |= *(log_mask_ptr+i);
	}
	mutex_unlock(&dci_log_mask_mutex);
}

int diag_send_dci_log_mask()
{
	void *buf = driver->buf_log_mask_update;
	int header_size = sizeof(struct diag_ctrl_log_mask);
	uint8_t *log_mask_ptr = dci_cumulative_log_mask;
	int i, j, ret = DIAG_DCI_NO_ERROR, err = DIAG_DCI_NO_ERROR;
	int updated;

	mutex_lock(&driver->diag_cntl_mutex);
	for (i = 0; i < 16; i++) {
		updated = 1;
		driver->log_mask->cmd_type = DIAG_CTRL_MSG_LOG_MASK;
		driver->log_mask->num_items = 512;
		driver->log_mask->data_len  = 11 + 512;
		driver->log_mask->stream_id = DCI_MASK_STREAM;
		driver->log_mask->status = 3; 
		driver->log_mask->equip_id = *log_mask_ptr;
		driver->log_mask->log_mask_size = 512;
		memcpy(buf, driver->log_mask, header_size);
		memcpy(buf+header_size, log_mask_ptr+2, 512);
		
		for (j = 0; j < NUM_SMD_DCI_CHANNELS; j++) {
			if (!driver->smd_dci[j].ch)
				continue;

			if (!(*(log_mask_ptr+1)))
				continue;
			err = diag_dci_write_proc(j, DIAG_CNTL_TYPE, buf,
				header_size + DCI_MAX_ITEMS_PER_LOG_CODE);
			if (err != DIAG_DCI_NO_ERROR) {
				updated = 0;
				ret = DIAG_DCI_SEND_DATA_FAIL;
			}
		}
		if (updated)
			*(log_mask_ptr+1) = 0; 
		log_mask_ptr += 514;
	}
	mutex_unlock(&driver->diag_cntl_mutex);

	return ret;
}

void create_dci_log_mask_tbl(unsigned char *tbl_buf)
{
	uint8_t i; int count = 0;

	if (!tbl_buf)
		return;

	
	for (i = 0; i < 16; i++) {
		*(uint8_t *)tbl_buf = i;
		pr_debug("diag: put value %x at %p\n", i, tbl_buf);
		memset(tbl_buf+1, 0, 513); 
		tbl_buf += 514;
		count += 514;
	}
}

void create_dci_event_mask_tbl(unsigned char *tbl_buf)
{
	memset(tbl_buf, 0, 512);
}

static int diag_dci_probe(struct platform_device *pdev)
{
	int err = 0;
	int index;

	if (pdev->id == SMD_APPS_MODEM) {
		index = MODEM_DATA;
		err = smd_named_open_on_edge("DIAG_2",
			SMD_APPS_MODEM,
			&driver->smd_dci[index].ch,
			&driver->smd_dci[index],
			diag_smd_notify);
		driver->smd_dci[index].ch_save =
			driver->smd_dci[index].ch;
		if (err)
			pr_err("diag: In %s, cannot open DCI Modem port, Id = %d, err: %d\n",
				__func__, pdev->id, err);
	}

	return err;
}

static int diag_dci_cmd_probe(struct platform_device *pdev)
{
	int err = 0;
	int index;

	if (pdev->id == SMD_APPS_MODEM) {
		index = MODEM_DATA;
		err = smd_named_open_on_edge("DIAG_2_CMD",
			pdev->id,
			&driver->smd_dci_cmd[index].ch,
			&driver->smd_dci_cmd[index],
			diag_smd_notify);
		driver->smd_dci_cmd[index].ch_save =
			driver->smd_dci_cmd[index].ch;
		if (err)
			pr_err("diag: In %s, cannot open DCI Modem CMD port, Id = %d, err: %d\n",
				__func__, pdev->id, err);
	}

	return err;
}

static int diag_dci_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int diag_dci_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static const struct dev_pm_ops diag_dci_dev_pm_ops = {
	.runtime_suspend = diag_dci_runtime_suspend,
	.runtime_resume = diag_dci_runtime_resume,
};

struct platform_driver msm_diag_dci_driver = {
	.probe = diag_dci_probe,
	.driver = {
		.name = "DIAG_2",
		.owner = THIS_MODULE,
		.pm   = &diag_dci_dev_pm_ops,
	},
};

struct platform_driver msm_diag_dci_cmd_driver = {
	.probe = diag_dci_cmd_probe,
	.driver = {
		.name = "DIAG_2_CMD",
		.owner = THIS_MODULE,
		.pm   = &diag_dci_dev_pm_ops,
	},
};

int diag_dci_init(void)
{
	int success = 0;
	int i;

	driver->dci_tag = 0;
	driver->dci_client_id = 0;
	driver->num_dci_client = 0;
	mutex_init(&driver->dci_mutex);
	mutex_init(&dci_log_mask_mutex);
	mutex_init(&dci_event_mask_mutex);
	mutex_init(&dci_health_mutex);
	spin_lock_init(&ws_lock);

	for (i = 0; i < NUM_SMD_DCI_CHANNELS; i++) {
		success = diag_smd_constructor(&driver->smd_dci[i], i,
							SMD_DCI_TYPE);
		if (!success)
			goto err;
	}

	if (driver->supports_separate_cmdrsp) {
		for (i = 0; i < NUM_SMD_DCI_CMD_CHANNELS; i++) {
			success = diag_smd_constructor(&driver->smd_dci_cmd[i],
							i, SMD_DCI_CMD_TYPE);
			if (!success)
				goto err;
		}
	}

	if (driver->apps_dci_buf == NULL) {
		driver->apps_dci_buf = kzalloc(APPS_BUF_SIZE, GFP_KERNEL);
		if (driver->apps_dci_buf == NULL)
			goto err;
	}
	INIT_LIST_HEAD(&driver->dci_client_list);
	INIT_LIST_HEAD(&driver->dci_req_list);

	driver->diag_dci_wq = create_singlethread_workqueue("diag_dci_wq");
	INIT_WORK(&dci_data_drain_work, dci_data_drain_work_fn);
	success = platform_driver_register(&msm_diag_dci_driver);
	if (success) {
		pr_err("diag: Could not register DCI driver\n");
		goto err;
	}
	if (driver->supports_separate_cmdrsp) {
		success = platform_driver_register(&msm_diag_dci_cmd_driver);
		if (success) {
			pr_err("diag: Could not register DCI cmd driver\n");
			goto err;
		}
	}
	setup_timer(&dci_drain_timer, dci_drain_data, 0);
	return DIAG_DCI_NO_ERROR;
err:
	pr_err("diag: Could not initialize diag DCI buffers");
	kfree(driver->apps_dci_buf);
	for (i = 0; i < NUM_SMD_DCI_CHANNELS; i++)
		diag_smd_destructor(&driver->smd_dci[i]);

	if (driver->supports_separate_cmdrsp)
		for (i = 0; i < NUM_SMD_DCI_CMD_CHANNELS; i++)
			diag_smd_destructor(&driver->smd_dci_cmd[i]);

	if (driver->diag_dci_wq)
		destroy_workqueue(driver->diag_dci_wq);
	mutex_destroy(&driver->dci_mutex);
	mutex_destroy(&dci_log_mask_mutex);
	mutex_destroy(&dci_event_mask_mutex);
	mutex_destroy(&dci_health_mutex);
	return DIAG_DCI_NO_REG;
}

void diag_dci_exit(void)
{
	int i;

	for (i = 0; i < NUM_SMD_DCI_CHANNELS; i++)
		diag_smd_destructor(&driver->smd_dci[i]);

	platform_driver_unregister(&msm_diag_dci_driver);

	if (driver->supports_separate_cmdrsp) {
		for (i = 0; i < NUM_SMD_DCI_CMD_CHANNELS; i++)
			diag_smd_destructor(&driver->smd_dci_cmd[i]);

		platform_driver_unregister(&msm_diag_dci_cmd_driver);
	}
	kfree(driver->apps_dci_buf);
	mutex_destroy(&driver->dci_mutex);
	mutex_destroy(&dci_log_mask_mutex);
	mutex_destroy(&dci_event_mask_mutex);
	mutex_destroy(&dci_health_mutex);
	destroy_workqueue(driver->diag_dci_wq);
}

int diag_dci_clear_log_mask()
{
	int j, k, err = DIAG_DCI_NO_ERROR;
	uint8_t *log_mask_ptr, *update_ptr;
	struct list_head *start, *temp;
	struct diag_dci_client_tbl *entry = NULL;

	entry = diag_dci_get_client_entry();
	if (!entry) {
		pr_err("diag: In %s, invalid client entry\n", __func__);
		return DIAG_DCI_TABLE_ERR;
	}

	mutex_lock(&dci_log_mask_mutex);
	create_dci_log_mask_tbl(entry->dci_log_mask);
	memset(dci_cumulative_log_mask, 0x0, DCI_LOG_MASK_SIZE);
	list_for_each_safe(start, temp, &driver->dci_client_list) {
		entry = list_entry(start, struct diag_dci_client_tbl, track);
		update_ptr = dci_cumulative_log_mask;
		log_mask_ptr = entry->dci_log_mask;
		for (j = 0; j < 16; j++) {
			*update_ptr = j;
			*(update_ptr + 1) = 1;
			update_ptr += 2;
			log_mask_ptr += 2;
			for (k = 0; k < 513; k++) {
				*update_ptr |= *log_mask_ptr;
				update_ptr++;
				log_mask_ptr++;
			}
		}
	}
	mutex_unlock(&dci_log_mask_mutex);
	
	diag_update_userspace_clients(DCI_LOG_MASKS_TYPE);
	
	err = diag_send_dci_log_mask();
	return err;
}

int diag_dci_clear_event_mask()
{
	int j, err = DIAG_DCI_NO_ERROR;
	uint8_t *event_mask_ptr, *update_ptr;
	struct list_head *start, *temp;
	struct diag_dci_client_tbl *entry = NULL;

	entry = diag_dci_get_client_entry();
	if (!entry) {
		pr_err("diag: In %s, invalid client entry\n", __func__);
		return DIAG_DCI_TABLE_ERR;
	}

	mutex_lock(&dci_event_mask_mutex);
	memset(entry->dci_event_mask, 0x0, DCI_EVENT_MASK_SIZE);
	memset(dci_cumulative_event_mask, 0x0, DCI_EVENT_MASK_SIZE);
	update_ptr = dci_cumulative_event_mask;

	list_for_each_safe(start, temp, &driver->dci_client_list) {
		entry = list_entry(start, struct diag_dci_client_tbl, track);
		event_mask_ptr = entry->dci_event_mask;
		for (j = 0; j < DCI_EVENT_MASK_SIZE; j++)
			*(update_ptr + j) |= *(event_mask_ptr + j);
	}
	mutex_unlock(&dci_event_mask_mutex);
	
	diag_update_userspace_clients(DCI_EVENT_MASKS_TYPE);
	
	err = diag_send_dci_event_mask();
	return err;
}

int diag_dci_query_log_mask(uint16_t log_code)
{
	return __diag_dci_query_log_mask(diag_dci_get_client_entry(),
					 log_code);
}

int diag_dci_query_event_mask(uint16_t event_id)
{
	return __diag_dci_query_event_mask(diag_dci_get_client_entry(),
					   event_id);
}

uint8_t diag_dci_get_cumulative_real_time()
{
	uint8_t real_time = MODE_NONREALTIME;
	struct list_head *start, *temp;
	struct diag_dci_client_tbl *entry = NULL;

	list_for_each_safe(start, temp, &driver->dci_client_list) {
		entry = list_entry(start, struct diag_dci_client_tbl, track);
		if (entry->real_time == MODE_REALTIME) {
			real_time = 1;
			break;
		}
	}
	return real_time;
}

int diag_dci_set_real_time(uint8_t real_time)
{
	struct diag_dci_client_tbl *entry = NULL;
	entry = diag_dci_get_client_entry();
	if (!entry) {
		pr_err("diag: In %s, invalid client entry\n", __func__);
		return 0;
	}
	entry->real_time = real_time;
	return 1;
}

void diag_dci_try_activate_wakeup_source()
{
	spin_lock_irqsave(&ws_lock, ws_lock_flags);
	pm_wakeup_event(driver->diag_dev, DCI_WAKEUP_TIMEOUT);
	pm_stay_awake(driver->diag_dev);
	spin_unlock_irqrestore(&ws_lock, ws_lock_flags);
}

void diag_dci_try_deactivate_wakeup_source()
{
	spin_lock_irqsave(&ws_lock, ws_lock_flags);
	pm_relax(driver->diag_dev);
	spin_unlock_irqrestore(&ws_lock, ws_lock_flags);
}

int diag_dci_register_client(struct diag_dci_reg_tbl_t *reg_entry)
{
	int i, err = 0;
	struct diag_dci_client_tbl *new_entry = NULL;
	struct diag_dci_buf_peripheral_t *proc_buf = NULL;

	if (!reg_entry)
		return DIAG_DCI_NO_REG;

	if (driver->dci_state == DIAG_DCI_NO_REG)
		return DIAG_DCI_NO_REG;

	if (driver->num_dci_client >= MAX_DCI_CLIENTS)
		return DIAG_DCI_NO_REG;

	new_entry = kzalloc(sizeof(struct diag_dci_client_tbl), GFP_KERNEL);
	if (new_entry == NULL) {
		pr_err("diag: unable to alloc memory\n");
		return DIAG_DCI_NO_REG;
	}

	mutex_lock(&driver->dci_mutex);
	if (!(driver->num_dci_client)) {
		for (i = 0; i < NUM_SMD_DCI_CHANNELS; i++)
			driver->smd_dci[i].in_busy_1 = 0;
		if (driver->supports_separate_cmdrsp)
			for (i = 0; i < NUM_SMD_DCI_CMD_CHANNELS; i++)
				driver->smd_dci_cmd[i].in_busy_1 = 0;
	}

	new_entry->client = current;
	new_entry->client_info.notification_list =
				reg_entry->notification_list;
	new_entry->client_info.signal_type =
				reg_entry->signal_type;
	new_entry->real_time = MODE_REALTIME;
	new_entry->in_service = 0;
	INIT_LIST_HEAD(&new_entry->list_write_buf);
	mutex_init(&new_entry->write_buf_mutex);
	new_entry->dci_log_mask =  kzalloc(DCI_LOG_MASK_SIZE, GFP_KERNEL);
	if (!new_entry->dci_log_mask) {
		pr_err("diag: Unable to create log mask for client, %d",
							driver->dci_client_id);
		goto fail_alloc;
	}
	create_dci_log_mask_tbl(new_entry->dci_log_mask);

	new_entry->dci_event_mask =  kzalloc(DCI_EVENT_MASK_SIZE, GFP_KERNEL);
	if (!new_entry->dci_event_mask) {
		pr_err("diag: Unable to create event mask for client, %d",
							driver->dci_client_id);
		goto fail_alloc;
	}
	create_dci_event_mask_tbl(new_entry->dci_event_mask);

	for (i = 0; i < NUM_DCI_PROC; i++) {
		proc_buf = &new_entry->buffers[i];
		if (!proc_buf)
			goto fail_alloc;

		mutex_init(&proc_buf->health_mutex);
		mutex_init(&proc_buf->buf_mutex);
		proc_buf->health.dropped_events = 0;
		proc_buf->health.dropped_logs = 0;
		proc_buf->health.received_events = 0;
		proc_buf->health.received_logs = 0;
		proc_buf->buf_primary = kzalloc(
					sizeof(struct diag_dci_buffer_t),
					GFP_KERNEL);
		if (!proc_buf->buf_primary)
			goto fail_alloc;
		proc_buf->buf_cmd = kzalloc(sizeof(struct diag_dci_buffer_t),
					    GFP_KERNEL);
		if (!proc_buf->buf_cmd)
			goto fail_alloc;
		err = diag_dci_init_buffer(proc_buf->buf_primary,
					   DCI_BUF_PRIMARY);
		if (err)
			goto fail_alloc;
		err = diag_dci_init_buffer(proc_buf->buf_cmd, DCI_BUF_CMD);
		if (err)
			goto fail_alloc;
		proc_buf->buf_curr = proc_buf->buf_primary;
	}

	list_add_tail(&new_entry->track, &driver->dci_client_list);
	driver->dci_client_id++;
	new_entry->client_info.client_id = driver->dci_client_id;
	reg_entry->client_id = driver->dci_client_id;
	driver->num_dci_client++;
	if (driver->num_dci_client == 1)
		diag_update_proc_vote(DIAG_PROC_DCI, VOTE_UP);
	queue_work(driver->diag_real_time_wq, &driver->diag_real_time_work);
	mutex_unlock(&driver->dci_mutex);

	return driver->dci_client_id;

fail_alloc:
	if (new_entry) {
		for (i = 0; i < NUM_DCI_PROC; i++) {
			proc_buf = &new_entry->buffers[i];
			mutex_destroy(&proc_buf->health_mutex);
			mutex_destroy(&proc_buf->buf_primary->data_mutex);
			mutex_destroy(&proc_buf->buf_cmd->data_mutex);
			if (proc_buf->buf_primary)
				kfree(proc_buf->buf_primary->data);
			kfree(proc_buf->buf_primary);
			if (proc_buf->buf_cmd)
				kfree(proc_buf->buf_cmd->data);
			kfree(proc_buf->buf_cmd);
		}
		kfree(new_entry->dci_event_mask);
		kfree(new_entry->dci_log_mask);
	}
	kfree(new_entry);
	mutex_unlock(&driver->dci_mutex);
	return DIAG_DCI_NO_REG;
}

int diag_dci_deinit_client()
{
	int ret = DIAG_DCI_NO_ERROR, real_time = MODE_REALTIME, i, peripheral;
	struct diag_dci_buf_peripheral_t *proc_buf = NULL;
	struct diag_dci_client_tbl *entry = diag_dci_get_client_entry();
	struct diag_dci_buffer_t *buf_entry, *temp;
	struct list_head *start, *req_temp;
	struct dci_pkt_req_entry_t *req_entry = NULL;
	struct diag_smd_info *smd_info = NULL;

	if (!entry)
		return DIAG_DCI_NOT_SUPPORTED;

	mutex_lock(&driver->dci_mutex);
	list_del(&entry->track);
	driver->num_dci_client--;
	kfree(entry->dci_log_mask);
	diag_update_userspace_clients(DCI_LOG_MASKS_TYPE);
	diag_dci_invalidate_cumulative_log_mask();
	ret = diag_send_dci_event_mask();
	if (ret != DIAG_DCI_NO_ERROR) {
		mutex_unlock(&driver->dci_mutex);
		return ret;
	}
	kfree(entry->dci_event_mask);
	diag_update_userspace_clients(DCI_EVENT_MASKS_TYPE);
	diag_dci_invalidate_cumulative_event_mask();
	ret = diag_send_dci_log_mask();
	if (ret != DIAG_DCI_NO_ERROR) {
		mutex_unlock(&driver->dci_mutex);
		return ret;
	}

	list_for_each_safe(start, req_temp, &driver->dci_req_list) {
		req_entry = list_entry(start, struct dci_pkt_req_entry_t,
				       track);
		if (req_entry->pid == current->tgid) {
			list_del(&req_entry->track);
			kfree(req_entry);
		}
	}

	
	mutex_lock(&entry->write_buf_mutex);
	list_for_each_entry_safe(buf_entry, temp, &entry->list_write_buf,
							buf_track) {
		list_del(&buf_entry->buf_track);
		if (buf_entry->buf_type == DCI_BUF_SECONDARY) {
			mutex_lock(&buf_entry->data_mutex);
			diagmem_free(driver, buf_entry->data, POOL_TYPE_DCI);
			buf_entry->data = NULL;
			mutex_unlock(&buf_entry->data_mutex);
			kfree(buf_entry);
		} else if (buf_entry->buf_type == DCI_BUF_CMD) {
			peripheral = buf_entry->data_source;
			if (peripheral == APPS_DATA)
				continue;
			mutex_lock(&buf_entry->data_mutex);
			smd_info = driver->separate_cmdrsp[peripheral] ?
					&driver->smd_dci_cmd[peripheral] :
					&driver->smd_dci[peripheral];
			smd_info->in_busy_1 = 0;
			mutex_unlock(&buf_entry->data_mutex);
		}
		diag_dci_try_deactivate_wakeup_source();
	}
	mutex_unlock(&entry->write_buf_mutex);

	for (i = 0; i < NUM_DCI_PROC; i++) {
		proc_buf = &entry->buffers[i];
		buf_entry = proc_buf->buf_curr;
		mutex_lock(&proc_buf->buf_mutex);
		
		if (buf_entry && buf_entry->buf_type == DCI_BUF_SECONDARY) {
			mutex_lock(&buf_entry->data_mutex);
			diagmem_free(driver, buf_entry->data, POOL_TYPE_DCI);
			buf_entry->data = NULL;
			mutex_unlock(&buf_entry->data_mutex);
			mutex_destroy(&buf_entry->data_mutex);
			kfree(buf_entry);
		}

		mutex_lock(&proc_buf->buf_primary->data_mutex);
		kfree(proc_buf->buf_primary->data);
		mutex_unlock(&proc_buf->buf_primary->data_mutex);

		mutex_lock(&proc_buf->buf_cmd->data_mutex);
		kfree(proc_buf->buf_cmd->data);
		mutex_unlock(&proc_buf->buf_cmd->data_mutex);

		mutex_destroy(&proc_buf->health_mutex);
		mutex_destroy(&proc_buf->buf_primary->data_mutex);
		mutex_destroy(&proc_buf->buf_cmd->data_mutex);

		kfree(proc_buf->buf_primary);
		kfree(proc_buf->buf_cmd);
		mutex_unlock(&proc_buf->buf_mutex);
	}
	mutex_destroy(&entry->write_buf_mutex);

	kfree(entry);

	if (driver->num_dci_client == 0) {
		diag_update_proc_vote(DIAG_PROC_DCI, VOTE_DOWN);
	} else {
		real_time = diag_dci_get_cumulative_real_time();
		diag_update_real_time_vote(DIAG_PROC_DCI, real_time);
	}
	queue_work(driver->diag_real_time_wq, &driver->diag_real_time_work);

	mutex_unlock(&driver->dci_mutex);

	return DIAG_DCI_NO_ERROR;
}

int diag_dci_write_proc(int peripheral, int pkt_type, char *buf, int len)
{
	struct diag_smd_info *smd_info = NULL;
	int wr_size = 0, retry = 0, err = -EAGAIN, timer = 0, i;

	if (!buf || (peripheral < 0 || peripheral > NUM_SMD_DCI_CHANNELS)
								|| len < 0) {
		pr_err("diag: In %s, invalid data 0x%p, peripheral: %d, len: %d\n",
				__func__, buf, peripheral, len);
		return -EINVAL;
	}

	if (pkt_type == DIAG_DATA_TYPE) {
		for (i = 0; i < NUM_SMD_DCI_CMD_CHANNELS; i++)
			if (peripheral == i)
				smd_info = &driver->smd_dci_cmd[peripheral];
		if (!smd_info)
			smd_info = &driver->smd_dci[peripheral];
	} else if (pkt_type == DIAG_CNTL_TYPE) {
		smd_info = &driver->smd_cntl[peripheral];
	} else {
		pr_err("diag: Invalid DCI pkt type in %s", __func__);
		return -EINVAL;
	}

	if (!smd_info || !smd_info->ch)
		return -EINVAL;

	while (retry < 3) {
		mutex_lock(&smd_info->smd_ch_mutex);
		wr_size = smd_write(smd_info->ch, buf, len);
		if (wr_size == len) {
			pr_debug("diag: successfully wrote pkt_type %d of len %d to %d in trial %d",
					pkt_type, len, peripheral, (retry+1));
			err = DIAG_DCI_NO_ERROR;
			mutex_unlock(&smd_info->smd_ch_mutex);
			break;
		}
		pr_debug("diag: cannot write pkt_type %d of len %d to %d in trial %d",
					pkt_type, len, peripheral, (retry+1));
		retry++;
		mutex_unlock(&smd_info->smd_ch_mutex);

		for (timer = 0; timer < 5; timer++)
			usleep(2000);
	}
	return err;
}

int diag_dci_copy_health_stats(struct diag_dci_health_stats *stats, int proc)
{
	struct diag_dci_client_tbl *entry = NULL;
	struct diag_dci_health_t *health = NULL;
	int i;

	if (!stats)
		return -EINVAL;

	if (proc < ALL_PROC || proc > APPS_DATA)
		return -EINVAL;

	entry = diag_dci_get_client_entry();
	if (!entry)
		return DIAG_DCI_NOT_SUPPORTED;

	stats->stats.dropped_logs = 0;
	stats->stats.dropped_events = 0;
	stats->stats.received_logs = 0;
	stats->stats.received_events = 0;

	if (proc != ALL_PROC) {
		health = &entry->buffers[proc].health;
		stats->stats.dropped_logs = health->dropped_logs;
		stats->stats.dropped_events = health->dropped_events;
		stats->stats.received_logs = health->received_logs;
		stats->stats.received_events = health->received_events;
		if (stats->reset_status) {
			mutex_lock(&entry->buffers[proc].health_mutex);
			health->dropped_logs = 0;
			health->dropped_events = 0;
			health->received_logs = 0;
			health->received_events = 0;
			mutex_unlock(&entry->buffers[proc].health_mutex);
		}
		return DIAG_DCI_NO_ERROR;
	}


	for (i = 0; i < NUM_DCI_PROC; i++) {
		health = &entry->buffers[i].health;
		stats->stats.dropped_logs += health->dropped_logs;
		stats->stats.dropped_events += health->dropped_events;
		stats->stats.received_logs += health->received_logs;
		stats->stats.received_events += health->received_events;
		if (stats->reset_status) {
			mutex_lock(&entry->buffers[i].health_mutex);
			health->dropped_logs = 0;
			health->dropped_events = 0;
			health->received_logs = 0;
			health->received_events = 0;
			mutex_unlock(&entry->buffers[i].health_mutex);
		}
	}
	return DIAG_DCI_NO_ERROR;
}
