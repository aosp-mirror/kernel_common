/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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

#ifndef _MSM_QMI_INTERFACE_H_
#define _MSM_QMI_INTERFACE_H_

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/socket.h>
#include <linux/gfp.h>
#include <linux/qmi_encdec.h>
#include <linux/workqueue.h>

#define QMI_COMMON_TLV_TYPE 0

enum qmi_event_type {
	QMI_RECV_MSG = 1,
	QMI_SERVER_ARRIVE,
	QMI_SERVER_EXIT,
};

struct qmi_handle {
	void *src_port;
	void *dest_info;
	uint16_t next_txn_id;
	struct list_head txn_list;
	struct mutex handle_lock;
	spinlock_t notify_lock;
	void (*notify)(struct qmi_handle *handle, enum qmi_event_type event,
			void *notify_priv);
	void *notify_priv;
	void (*ind_cb)(struct qmi_handle *handle,
			unsigned int msg_id, void *msg,
			unsigned int msg_len, void *ind_cb_priv);
	void *ind_cb_priv;
	int handle_reset;
	wait_queue_head_t reset_waitq;
	struct list_head pending_txn_list;
	struct delayed_work resume_tx_work;
};

enum qmi_result_type_v01 {
	
	QMI_RESULT_TYPE_MIN_ENUM_VAL_V01 = INT_MIN,
	QMI_RESULT_SUCCESS_V01 = 0,
	QMI_RESULT_FAILURE_V01 = 1,
	QMI_RESULT_TYPE_MAX_ENUM_VAL_V01 = INT_MAX,
};

enum qmi_error_type_v01 {
	
	QMI_ERROR_TYPE_MIN_ENUM_VAL_V01 = INT_MIN,
	QMI_ERR_NONE_V01 = 0x0000,
	QMI_ERROR_MALFORMED_MSG_V01 = 0x0001,
	QMI_ERR_NO_MEMORY_V01 = 0x0002,
	QMI_ERR_INTERNAL_V01 = 0x0003,
	QMI_ERR_INVALID_ID_V01 = 0x0029,
	QMI_ERR_INCOMPATIBLE_STATE_V01 = 0x005A,
	QMI_ERROR_TYPE_MAX_ENUM_VAL_V01 = INT_MAX,
};

struct qmi_response_type_v01 {
	enum qmi_result_type_v01 result;
	enum qmi_error_type_v01 error;
};

#ifdef CONFIG_MSM_QMI_INTERFACE

extern struct elem_info qmi_response_type_v01_ei[];
#define get_qmi_response_type_v01_ei() qmi_response_type_v01_ei

struct qmi_handle *qmi_handle_create(
	void (*notify)(struct qmi_handle *handle,
		       enum qmi_event_type event, void *notify_priv),
	void *notify_priv);

int qmi_handle_destroy(struct qmi_handle *handle);

int qmi_register_ind_cb(struct qmi_handle *handle,
	void (*ind_cb)(struct qmi_handle *handle,
		       unsigned int msg_id, void *msg,
		       unsigned int msg_len, void *ind_cb_priv),
	void *ind_cb_priv);

int qmi_send_req_wait(struct qmi_handle *handle,
		      struct msg_desc *req_desc,
		      void *req, unsigned int req_len,
		      struct msg_desc *resp_desc,
		      void *resp, unsigned int resp_len,
		      unsigned long timeout_ms);

int qmi_send_req_nowait(struct qmi_handle *handle,
			struct msg_desc *req_desc,
			void *req, unsigned int req_len,
			struct msg_desc *resp_desc,
			void *resp, unsigned int resp_len,
			void (*resp_cb)(struct qmi_handle *handle,
					unsigned int msg_id, void *msg,
					void *resp_cb_data,
					int stat),
			void *resp_cb_data);

int qmi_recv_msg(struct qmi_handle *handle);

int qmi_connect_to_service(struct qmi_handle *handle,
			   uint32_t service_id,
			   uint32_t service_vers,
			   uint32_t service_ins);

int qmi_svc_event_notifier_register(uint32_t service_id,
				    uint32_t service_vers,
				    uint32_t service_ins,
				    struct notifier_block *nb);

int qmi_svc_event_notifier_unregister(uint32_t service_id,
				      uint32_t service_vers,
				      uint32_t service_ins,
				      struct notifier_block *nb);
#else

#define get_qmi_response_type_v01_ei() NULL

static inline struct qmi_handle *qmi_handle_create(
	void (*notify)(struct qmi_handle *handle,
		       enum qmi_event_type event, void *notify_priv),
	void *notify_priv)
{
	return NULL;
}

static inline int qmi_handle_destroy(struct qmi_handle *handle)
{
	return -ENODEV;
}

static inline int qmi_register_ind_cb(struct qmi_handle *handle,
	void (*ind_cb)(struct qmi_handle *handle,
		       unsigned int msg_id, void *msg,
		       unsigned int msg_len, void *ind_cb_priv),
	void *ind_cb_priv)
{
	return -ENODEV;
}

static inline int qmi_send_req_wait(struct qmi_handle *handle,
				    struct msg_desc *req_desc,
				    void *req, unsigned int req_len,
				    struct msg_desc *resp_desc,
				    void *resp, unsigned int resp_len,
				    unsigned long timeout_ms)
{
	return -ENODEV;
}

static inline int qmi_send_req_nowait(struct qmi_handle *handle,
				struct msg_desc *req_desc,
				void *req, unsigned int req_len,
				struct msg_desc *resp_desc,
				void *resp, unsigned int resp_len,
				void (*resp_cb)(struct qmi_handle *handle,
						unsigned int msg_id, void *msg,
						void *resp_cb_data),
				void *resp_cb_data)
{
	return -ENODEV;
}

static inline int qmi_recv_msg(struct qmi_handle *handle)
{
	return -ENODEV;
}

static inline int qmi_connect_to_service(struct qmi_handle *handle,
					 uint32_t service_id,
					 uint32_t service_vers,
					 uint32_t service_ins)
{
	return -ENODEV;
}

static inline int qmi_svc_event_notifier_register(uint32_t service_id,
						  uint32_t service_vers,
						  uint32_t service_ins,
						  struct notifier_block *nb)
{
	return -ENODEV;
}

static inline int qmi_svc_event_notifier_unregister(uint32_t service_id,
						    uint32_t service_vers,
						    uint32_t service_ins,
						    struct notifier_block *nb)
{
	return -ENODEV;
}

#endif

#endif
