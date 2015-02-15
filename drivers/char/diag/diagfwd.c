/* Copyright (c) 2008-2014, The Linux Foundation. All rights reserved.
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
#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/ratelimit.h>
#include <linux/workqueue.h>
#include <linux/pm_runtime.h>
#include <linux/diagchar.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/of.h>
#include <linux/kmemleak.h>
#ifdef CONFIG_DIAG_OVER_USB
#include <mach/usbdiag.h>
#endif
#include <mach/msm_smd.h>
#include <mach/socinfo.h>
#include <mach/restart.h>
#include "diagmem.h"
#include "diagchar.h"
#include "diagfwd.h"
#include "diagfwd_cntl.h"
#include "diagfwd_hsic.h"
#include "diagchar_hdlc.h"
#ifdef CONFIG_DIAG_SDIO_PIPE
#include "diagfwd_sdio.h"
#endif
#include "diag_dci.h"
#include "diag_masks.h"
#include "diagfwd_bridge.h"

#define STM_CMD_VERSION_OFFSET	4
#define STM_CMD_MASK_OFFSET	5
#define STM_CMD_DATA_OFFSET	6
#define STM_CMD_NUM_BYTES	7

#define STM_RSP_VALID_INDEX		7
#define STM_RSP_SUPPORTED_INDEX		8
#define STM_RSP_SMD_COMPLY_INDEX	9
#define STM_RSP_NUM_BYTES		10

#define STM_COMMAND_VALID 1

#define SMD_DRAIN_BUF_SIZE 4096

int diag_debug_buf_idx;
unsigned char diag_debug_buf[1024];
static unsigned int buf_tbl_size = 10;
int sdio_diag_initialized;
int smd_diag_initialized;
#if DIAG_XPST
static int diag_smd_function_mode;
#endif
struct diag_master_table entry;
int wrap_enabled;
uint16_t wrap_count;

void encode_rsp_and_send(int buf_length)
{
	struct diag_send_desc_type send = { NULL, NULL, DIAG_STATE_START, 0 };
	struct diag_hdlc_dest_type enc = { NULL, NULL, 0 };
	struct diag_smd_info *data = &(driver->smd_data[MODEM_DATA]);
	int err;
	unsigned long flags;

	if (buf_length > APPS_BUF_SIZE) {
		pr_err("diag: In %s, invalid len %d, permissible len %d\n",
					__func__, buf_length, APPS_BUF_SIZE);
		return;
	}

	send.state = DIAG_STATE_START;
	send.pkt = driver->apps_rsp_buf;
	send.last = (void *)(driver->apps_rsp_buf + buf_length);
	send.terminate = 1;
	if (!data->in_busy_1) {
		spin_lock_irqsave(&data->in_busy_lock, flags);
		enc.dest = data->buf_in_1;
		enc.dest_last = (void *)(data->buf_in_1 + APPS_BUF_SIZE - 1);
		diag_hdlc_encode(&send, &enc);
		data->write_ptr_1->buf = data->buf_in_1;
		data->write_ptr_1->length = (int)(enc.dest -
						(void *)(data->buf_in_1));
		data->in_busy_1 = 1;
		err = diag_device_write(data->buf_in_1, data->peripheral,
					data->write_ptr_1);
		if (err) {
			pr_err("diag: In %s, Unable to write to device, err: %d\n",
			       __func__, err);
			data->in_busy_1 = 0;
		}
		memset(driver->apps_rsp_buf, '\0', APPS_BUF_SIZE);
		spin_unlock_irqrestore(&data->in_busy_lock, flags);
	}
}

#ifdef CONFIG_OF
static int has_device_tree(void)
{
	struct device_node *node;

	node = of_find_node_by_path("/");
	if (node) {
		of_node_put(node);
		return 1;
	}
	return 0;
}
#else
static int has_device_tree(void)
{
	return 0;
}
#endif

int chk_config_get_id(void)
{
	
	if (machine_is_msm8x60_fusion() || machine_is_msm8x60_fusn_ffa())
		return 0;

	if (driver->use_device_tree) {
		if (machine_is_msm8974())
			return MSM8974_TOOLS_ID;
		else
			return 0;
	} else {
		switch (socinfo_get_msm_cpu()) {
		case MSM_CPU_8X60:
			return APQ8060_TOOLS_ID;
		case MSM_CPU_8960:
		case MSM_CPU_8960AB:
			return AO8960_TOOLS_ID;
		case MSM_CPU_8064:
		case MSM_CPU_8064AB:
		case MSM_CPU_8064AA:
			return APQ8064_TOOLS_ID;
		case MSM_CPU_8930:
		case MSM_CPU_8930AA:
		case MSM_CPU_8930AB:
			return MSM8930_TOOLS_ID;
		case MSM_CPU_8974:
			return MSM8974_TOOLS_ID;
		case MSM_CPU_8625:
			return MSM8625_TOOLS_ID;
		default:
			return 0;
		}
	}
}

int chk_apps_only(void)
{
	if (driver->use_device_tree)
		return 1;

	switch (socinfo_get_msm_cpu()) {
	case MSM_CPU_8960:
	case MSM_CPU_8960AB:
	case MSM_CPU_8064:
	case MSM_CPU_8064AB:
	case MSM_CPU_8064AA:
	case MSM_CPU_8930:
	case MSM_CPU_8930AA:
	case MSM_CPU_8930AB:
	case MSM_CPU_8627:
	case MSM_CPU_9615:
	case MSM_CPU_8974:
		return 1;
	default:
		return 0;
	}
}

int chk_apps_master(void)
{
	if (driver->use_device_tree)
		return 1;
	else if (soc_class_is_msm8960() || soc_class_is_msm8930() ||
		 soc_class_is_apq8064() || cpu_is_msm9615())
		return 1;
	else
		return 0;
}

int chk_polling_response(void)
{
	if (!(driver->polling_reg_flag) && chk_apps_master())
		return 1;
	else if (!((driver->smd_data[MODEM_DATA].ch) &&
		 (driver->rcvd_feature_mask[MODEM_DATA])) &&
		 (chk_apps_master()))
		return 1;
	else
		return 0;
}

void chk_logging_wakeup(void)
{
	int i;

	
	for (i = 0; i < driver->num_clients; i++)
		if (driver->client_map[i].pid ==
			driver->logging_process_id)
			break;

	if (i < driver->num_clients) {
		if ((driver->data_ready[i] & USERMODE_DIAGFWD) == 0) {
			driver->data_ready[i] |= USERMODE_DIAGFWD;
			pr_info("diag: Force wakeup of logging process\n");
			wake_up_interruptible(&driver->wait_q);
		}
	}
}
int diag_add_hdlc_encoding(struct diag_smd_info *smd_info, void *buf,
			   int total_recd, uint8_t *encode_buf,
			   int *encoded_length)
{
	struct diag_send_desc_type send = { NULL, NULL, DIAG_STATE_START, 0 };
	struct diag_hdlc_dest_type enc = { NULL, NULL, 0 };
	struct data_header {
		uint8_t control_char;
		uint8_t version;
		uint16_t length;
	};
	struct data_header *header;
	int header_size = sizeof(struct data_header);
	uint8_t *end_control_char;
	uint8_t *payload;
	uint8_t *temp_buf;
	uint8_t *temp_encode_buf;
	int src_pkt_len;
	int encoded_pkt_length;
	int max_size;
	int total_processed = 0;
	int bytes_remaining;
	int success = 1;

	temp_buf = buf;
	temp_encode_buf = encode_buf;
	bytes_remaining = *encoded_length;
	while (total_processed < total_recd) {
		header = (struct data_header *)temp_buf;
		
		if (header->control_char != CONTROL_CHAR ||
			header->version != 1) {
			success = 0;
			break;
		}
		payload = temp_buf + header_size;
		end_control_char = payload + header->length;
		if (*end_control_char != CONTROL_CHAR) {
			success = 0;
			break;
		}

		max_size = 2 * header->length + 3;
		if (bytes_remaining < max_size) {
			pr_err("diag: In %s, Not enough room to encode remaining data for peripheral: %d, bytes available: %d, max_size: %d\n",
				__func__, smd_info->peripheral,
				bytes_remaining, max_size);
			success = 0;
			break;
		}

		
		send.state = DIAG_STATE_START;
		send.pkt = payload;
		send.last = (void *)(payload + header->length - 1);
		send.terminate = 1;

		enc.dest = temp_encode_buf;
		enc.dest_last = (void *)(temp_encode_buf + max_size);
		enc.crc = 0;
		diag_hdlc_encode(&send, &enc);

		
		src_pkt_len = (header_size + header->length + 1);
		total_processed += src_pkt_len;
		temp_buf += src_pkt_len;

		encoded_pkt_length = (uint8_t *)enc.dest - temp_encode_buf;
		bytes_remaining -= encoded_pkt_length;
		temp_encode_buf = enc.dest;
	}

	*encoded_length = (int)(temp_encode_buf - encode_buf);

	return success;
}

static int check_bufsize_for_encoding(struct diag_smd_info *smd_info, void *buf,
					int total_recd)
{
	int buf_size = IN_BUF_SIZE;
	int max_size = 2 * total_recd + 3;
	unsigned char *temp_buf;

	if (max_size > IN_BUF_SIZE) {
		if (max_size > MAX_IN_BUF_SIZE) {
			pr_err_ratelimited("diag: In %s, SMD sending packet of %d bytes that may expand to %d bytes, peripheral: %d\n",
				__func__, total_recd, max_size,
				smd_info->peripheral);
			max_size = MAX_IN_BUF_SIZE;
		}
		if (buf == smd_info->buf_in_1_raw) {
			
			if (smd_info->buf_in_1_size < max_size) {
				temp_buf = krealloc(smd_info->buf_in_1,
					max_size, GFP_KERNEL);
				if (temp_buf) {
					smd_info->buf_in_1 = temp_buf;
					smd_info->buf_in_1_size = max_size;
				}
			}
			buf_size = smd_info->buf_in_1_size;
		} else {
			
			if (smd_info->buf_in_2_size < max_size) {
				temp_buf = krealloc(smd_info->buf_in_2,
					max_size, GFP_KERNEL);
				if (temp_buf) {
					smd_info->buf_in_2 = temp_buf;
					smd_info->buf_in_2_size = max_size;
				}
			}
			buf_size = smd_info->buf_in_2_size;
		}
	}

	return buf_size;
}

void diag_ws_on_notify()
{
	pm_stay_awake(driver->diag_dev);
}

void diag_ws_on_read(int pkt_len)
{
	unsigned long flags;

	spin_lock_irqsave(&driver->ws_lock, flags);
	if (pkt_len > 0) {
		driver->ws_ref_count++;
	} else {
		if (driver->ws_ref_count < 1) {
			pm_relax(driver->diag_dev);
			driver->ws_ref_count = 0;
			driver->copy_count = 0;
		}
	}
	spin_unlock_irqrestore(&driver->ws_lock, flags);
}

void diag_ws_on_copy()
{
	unsigned long flags;
	spin_lock_irqsave(&driver->ws_lock, flags);
	driver->copy_count++;
	spin_unlock_irqrestore(&driver->ws_lock, flags);
}

void diag_ws_on_copy_complete()
{
	unsigned long flags;
	spin_lock_irqsave(&driver->ws_lock, flags);
	driver->ws_ref_count -= driver->copy_count;
	if (driver->ws_ref_count < 1) {
		pm_relax(driver->diag_dev);
		driver->ws_ref_count = 0;
	}
	driver->copy_count = 0;
	spin_unlock_irqrestore(&driver->ws_lock, flags);
}

void diag_ws_reset()
{
	unsigned long flags;

	spin_lock_irqsave(&driver->ws_lock, flags);
	pm_relax(driver->diag_dev);
	driver->ws_ref_count = 0;
	driver->copy_count = 0;
	spin_unlock_irqrestore(&driver->ws_lock, flags);
}

int diag_process_smd_read_data(struct diag_smd_info *smd_info, void *buf,
			       int total_recd)
{
	struct diag_request *write_ptr_modem = NULL;
	int *in_busy_ptr = 0;
	int err = 0;
#if DIAG_XPST
	int type;
#endif
	unsigned long flags;

	if ((smd_info->type == SMD_CMD_TYPE) &&
		!driver->separate_cmdrsp[smd_info->peripheral]) {
		
		pr_err("diag, In %s, received data on non-designated command channel: %d\n",
			__func__, smd_info->peripheral);
		goto err;
	}

	
	if (!smd_info->encode_hdlc) {
		if (smd_info->buf_in_1 == buf) {
			write_ptr_modem = smd_info->write_ptr_1;
			in_busy_ptr = &smd_info->in_busy_1;
		} else if (smd_info->buf_in_2 == buf) {
			write_ptr_modem = smd_info->write_ptr_2;
			in_busy_ptr = &smd_info->in_busy_2;
		} else {
			pr_err("diag: In %s, no match for in_busy_1, peripheral: %d\n",
				__func__, smd_info->peripheral);
			goto err;
		}
		if (smd_info->peripheral == MODEM_DATA) {
			DIAGFWD_7K_RAWDATA(buf, "modem", DIAG_DBG_READ);
#if DIAG_XPST && !defined(CONFIG_DIAGFWD_BRIDGE_CODE)
			type = checkcmd_modem_epst(buf);
			if (type) {
				modem_to_userspace(buf, total_recd, type, 0);
				return 0;
			}
			if (driver->qxdmusb_drop && driver->logging_mode == USB_MODE)
				return 0;
#endif
		}

		if (write_ptr_modem) {
			spin_lock_irqsave(&smd_info->in_busy_lock, flags);
			write_ptr_modem->length = total_recd;
			*in_busy_ptr = 1;
			err = diag_device_write(buf, smd_info->peripheral,
						write_ptr_modem);
			spin_unlock_irqrestore(&smd_info->in_busy_lock, flags);
			if (err) {
				pr_err_ratelimited("diag: In %s, diag_device_write error: %d\n",
					__func__, err);
				goto err;
			}
		}
	} else {
		
		if (smd_info->buf_in_1_raw == buf) {
			write_ptr_modem = smd_info->write_ptr_1;
			in_busy_ptr = &smd_info->in_busy_1;
		} else if (smd_info->buf_in_2_raw == buf) {
			write_ptr_modem = smd_info->write_ptr_2;
			in_busy_ptr = &smd_info->in_busy_2;
		} else {
			pr_err("diag: In %s, no match for in_busy_1, peripheral: %d\n",
				__func__, smd_info->peripheral);
			goto err;
		}

		if (write_ptr_modem) {
			int success = 0;
			int write_length = 0;
			unsigned char *write_buf = NULL;

			write_length = check_bufsize_for_encoding(smd_info, buf,
								total_recd);
			if (write_length) {
				write_buf = (buf == smd_info->buf_in_1_raw) ?
					smd_info->buf_in_1 : smd_info->buf_in_2;
				success = diag_add_hdlc_encoding(smd_info, buf,
							total_recd, write_buf,
							&write_length);
				if (success) {
					spin_lock_irqsave(
						&smd_info->in_busy_lock, flags);
					write_ptr_modem->length = write_length;
					*in_busy_ptr = 1;
					err = diag_device_write(write_buf,
							smd_info->peripheral,
							write_ptr_modem);
					spin_unlock_irqrestore(
						&smd_info->in_busy_lock, flags);
					if (err) {
						pr_err_ratelimited("diag: In %s, diag_device_write error: %d\n",
								__func__, err);
						goto err;
					}
				}
			} else {
				goto err;
			}
		}
	}

	return 0;
err:
	if (driver->logging_mode == MEMORY_DEVICE_MODE)
		diag_ws_on_read(0);
	return 0;
}

void diag_smd_queue_read(struct diag_smd_info *smd_info)
{
	if (!smd_info || !smd_info->ch)
		return;

	switch (smd_info->type) {
	case SMD_DCI_TYPE:
	case SMD_DCI_CMD_TYPE:
		queue_work(driver->diag_dci_wq,
			   &(smd_info->diag_read_smd_work));
		break;
	case SMD_DATA_TYPE:
		queue_work(smd_info->wq,
			   &(smd_info->diag_read_smd_work));
		break;
	case SMD_CNTL_TYPE:
	case SMD_CMD_TYPE:
		queue_work(driver->diag_wq,
			   &(smd_info->diag_read_smd_work));
		break;
	default:
		pr_err("diag: In %s, invalid type: %d\n", __func__,
			smd_info->type);
		return;
	}

	if (driver->logging_mode == MEMORY_DEVICE_MODE &&
	    smd_info->type == SMD_DATA_TYPE)
		diag_ws_on_notify();
}

static int diag_smd_resize_buf(struct diag_smd_info *smd_info, void **buf,
			       unsigned int *buf_size,
			       unsigned int requested_size)
{
	int success = 0;
	void *temp_buf = NULL;
	unsigned int new_buf_size = requested_size;

	if (!smd_info)
		return success;

	if (requested_size <= MAX_IN_BUF_SIZE) {
		pr_debug("diag: In %s, SMD peripheral: %d sending in packets up to %d bytes\n",
			__func__, smd_info->peripheral, requested_size);
	} else {
		pr_err_ratelimited("diag: In %s, SMD peripheral: %d, Packet size sent: %d, Max size supported (%d) exceeded. Data beyond max size will be lost\n",
			__func__, smd_info->peripheral, requested_size,
			MAX_IN_BUF_SIZE);
		new_buf_size = MAX_IN_BUF_SIZE;
	}

	
	if (new_buf_size <= *buf_size) {
		success = 1;
		return success;
	}

	temp_buf = krealloc(*buf, new_buf_size, GFP_KERNEL);

	if (temp_buf) {
		
		if (smd_info->encode_hdlc) {
			void *temp_hdlc = NULL;
			if (*buf == smd_info->buf_in_1_raw) {
				smd_info->buf_in_1_raw = temp_buf;
				smd_info->buf_in_1_raw_size = new_buf_size;
				temp_hdlc = krealloc(smd_info->buf_in_1,
							MAX_IN_BUF_SIZE,
							GFP_KERNEL);
				if (temp_hdlc) {
					smd_info->buf_in_1 = temp_hdlc;
					smd_info->buf_in_1_size =
							MAX_IN_BUF_SIZE;
				}
			} else if (*buf == smd_info->buf_in_2_raw) {
				smd_info->buf_in_2_raw = temp_buf;
				smd_info->buf_in_2_raw_size = new_buf_size;
				temp_hdlc = krealloc(smd_info->buf_in_2,
							MAX_IN_BUF_SIZE,
							GFP_KERNEL);
				if (temp_hdlc) {
					smd_info->buf_in_2 = temp_hdlc;
					smd_info->buf_in_2_size =
							MAX_IN_BUF_SIZE;
				}
			}
		} else {
			if (*buf == smd_info->buf_in_1) {
				smd_info->buf_in_1 = temp_buf;
				smd_info->buf_in_1_size = new_buf_size;
			} else if (*buf == smd_info->buf_in_2) {
				smd_info->buf_in_2 = temp_buf;
				smd_info->buf_in_2_size = new_buf_size;
			}
		}
		*buf = temp_buf;
		*buf_size = new_buf_size;
		success = 1;
	} else {
		pr_err_ratelimited("diag: In %s, SMD peripheral: %d. packet size sent: %d, resize to support failed. Data beyond %d will be lost\n",
			__func__, smd_info->peripheral, requested_size,
			*buf_size);
	}

	return success;
}

void diag_smd_send_req(struct diag_smd_info *smd_info)
{
	void *buf = NULL, *temp_buf = NULL;
	int total_recd = 0, r = 0, pkt_len;
	int loop_count = 0;
	int notify = 0;
	int retry = 0;
	int buf_size = 0;
	int resize_success = 0;
	int buf_full = 0;
	static unsigned long buf_status_time;

	if (!smd_info) {
		pr_err("diag: In %s, no smd info. Not able to read.\n",
			__func__);
		return;
	}
	
	if (smd_info->type == SMD_DATA_TYPE) {
		if (smd_info->in_busy_1 && smd_info->in_busy_2) {
			smd_info->buf_full++;
			if (printk_timed_ratelimit(&buf_status_time, 5000)) {
				printk("%s: buffer status(data_type:full:release)= %d:%d:%d,\n",
						__func__, smd_info->type, smd_info->buf_full, smd_info->buf_release);
			}
		}
		else if (!smd_info->in_busy_1 || !smd_info->in_busy_2)
			smd_info->buf_release++;
	}

	
	if (smd_info->type == SMD_DATA_TYPE) {
		
		if (smd_info->encode_hdlc) {
			if (!smd_info->in_busy_1) {
				buf = smd_info->buf_in_1_raw;
				buf_size = smd_info->buf_in_1_raw_size;
			} else if (!smd_info->in_busy_2) {
				buf = smd_info->buf_in_2_raw;
				buf_size = smd_info->buf_in_2_raw_size;
			}
		} else {
			if (!smd_info->in_busy_1) {
				buf = smd_info->buf_in_1;
				buf_size = smd_info->buf_in_1_size;
			} else if (!smd_info->in_busy_2) {
				buf = smd_info->buf_in_2;
				buf_size = smd_info->buf_in_2_size;
			}
		}
	} else if (smd_info->type == SMD_CMD_TYPE) {
		
		if (smd_info->encode_hdlc) {
			if (!smd_info->in_busy_1) {
				buf = smd_info->buf_in_1_raw;
				buf_size = smd_info->buf_in_1_raw_size;
			}
		} else {
			if (!smd_info->in_busy_1) {
				buf = smd_info->buf_in_1;
				buf_size = smd_info->buf_in_1_size;
			}
		}
	} else if (!smd_info->in_busy_1) {
		buf = smd_info->buf_in_1;
		buf_size = smd_info->buf_in_1_size;
	}

	if (!buf && (smd_info->type == SMD_DCI_TYPE ||
					smd_info->type == SMD_DCI_CMD_TYPE))
		diag_dci_try_deactivate_wakeup_source();

drop:
	if (smd_info->ch && buf) {

		pkt_len = smd_cur_packet_size(smd_info->ch);
		if (pkt_len == 0 && (smd_info->type == SMD_DCI_TYPE ||
					smd_info->type == SMD_DCI_CMD_TYPE))
			diag_dci_try_deactivate_wakeup_source();
		if (pkt_len > buf_size)
			resize_success = diag_smd_resize_buf(smd_info, &buf,
							&buf_size, pkt_len);
		temp_buf = buf;
		while (pkt_len && (pkt_len != total_recd)) {
			loop_count++;
			r = smd_read_avail(smd_info->ch);
			pr_debug("diag: In %s, SMD peripheral: %d, received pkt %d %d\n",
				__func__, smd_info->peripheral, r, total_recd);
			if (!r) {
				
				wait_event(driver->smd_wait_q,
					((smd_info->ch == 0) ||
					smd_read_avail(smd_info->ch)));
				
				if (smd_info->ch) {
					pr_debug("diag: In %s, SMD peripheral: %d, return from wait_event\n",
						__func__, smd_info->peripheral);
					continue;
				} else {
					pr_debug("diag: In %s, SMD peripheral: %d, return from wait_event ch closed\n",
						__func__, smd_info->peripheral);
					goto fail_return;
				}
			}

			if (pkt_len < r) {
				pr_err("diag: In %s, SMD peripheral: %d, sending incorrect pkt\n",
					__func__, smd_info->peripheral);
				goto fail_return;
			}
			if (pkt_len > r) {
				pr_debug("diag: In %s, SMD sending partial pkt %d %d %d %d %d %d\n",
				__func__, pkt_len, r, total_recd, loop_count,
				smd_info->peripheral, smd_info->type);
			}

			
			if (total_recd < buf_size) {
				if (total_recd + r > buf_size) {
					r = buf_size - total_recd;
					buf_full = 1;
				}

				total_recd += r;

				
				smd_read(smd_info->ch, temp_buf, r);
				temp_buf += r;
			} else {
				int drain_bytes = (r > SMD_DRAIN_BUF_SIZE) ?
							SMD_DRAIN_BUF_SIZE : r;
				unsigned char *drain_buf = kzalloc(drain_bytes,
								GFP_KERNEL);
				if (drain_buf) {
					total_recd += drain_bytes;
					smd_read(smd_info->ch, drain_buf,
							drain_bytes);
					kfree(drain_buf);
				} else {
					pr_err("diag: In %s, SMD peripheral: %d, unable to allocate drain buffer\n",
						__func__, smd_info->peripheral);
					break;
				}
			}
		}
		if (smd_info->type == SMD_DATA_TYPE &&
		    driver->logging_mode == MEMORY_DEVICE_MODE)
			diag_ws_on_read(pkt_len);

		
		if (smd_info->peripheral == MODEM_DATA &&
			(buf && driver->qxdm2sd_drop && (driver->logging_mode == USB_MODE)
				&& *((unsigned char *)buf) != 0xc8)) {
			DIAG_DBUG("%s:Drop the diag payload :%d\n", __func__, retry);
			DIAGFWD_7K_RAWDATA(buf, "modem", DIAG_DBG_DROP);
			total_recd = 0;
			
			msleep(10);
			r = smd_read_avail(smd_info->ch);
			if (++retry > 20) {
				driver->qxdm2sd_drop = 0;
				return;
			}
			if (r)
				goto drop;
			else {
				DIAG_INFO("Nothing pending in SMD buffer\n");
				driver->qxdm2sd_drop = 0;
				return;
			}
		}
		if (total_recd > 0) {
			if (!buf) {
				pr_err("diag: In %s, SMD peripheral: %d, Out of diagmem for Modem\n",
					__func__, smd_info->peripheral);
			} else if (smd_info->process_smd_read_data) {
				DIAGFWD_7K_RAWDATA(buf, "modem", DIAG_DBG_READ);
				if (buf_full)
					total_recd = buf_size;

				notify = smd_info->process_smd_read_data(
						smd_info, buf, total_recd);
				
				if (notify)
					diag_smd_notify(smd_info,
							SMD_EVENT_DATA);
			}
		}
	} else if (smd_info->ch && !buf &&
		(driver->logging_mode == MEMORY_DEVICE_MODE)) {
			chk_logging_wakeup();
	}
	return;

fail_return:
	if (smd_info->type == SMD_DATA_TYPE &&
	    driver->logging_mode == MEMORY_DEVICE_MODE)
		diag_ws_on_read(0);

	if (smd_info->type == SMD_DCI_TYPE ||
					smd_info->type == SMD_DCI_CMD_TYPE)
		diag_dci_try_deactivate_wakeup_source();
	return;
}

void diag_read_smd_work_fn(struct work_struct *work)
{
	struct diag_smd_info *smd_info = container_of(work,
							struct diag_smd_info,
							diag_read_smd_work);
	diag_smd_send_req(smd_info);
}

#ifdef CONFIG_DIAG_OVER_USB
static int diag_write_to_usb(struct usb_diag_ch *ch,
			     struct diag_request *write_ptr)
{
	int err = 0;
	uint8_t retry_count, max_retries;

	if (!ch || !write_ptr)
		return -EIO;

	retry_count = 0;
	max_retries = 3;

	while (retry_count < max_retries) {
		retry_count++;
		
		if (!driver->usb_connected) {
			err = -ENODEV;
			break;
		}
		err = usb_diag_write(ch, write_ptr);
		if (err == -EAGAIN) {
			usleep_range(10000, 10100);
			continue;
		} else {
			break;
		}
	}
	return err;
}
#endif

int diag_device_write(void *buf, int data_type, struct diag_request *write_ptr)
{
	int i, err = 0, index;
	index = 0;

	if (driver->logging_mode == MEMORY_DEVICE_MODE) {
		if (data_type == APPS_DATA) {
			for (i = 0; i < driver->buf_tbl_size; i++)
				if (driver->buf_tbl[i].length == 0) {
					driver->buf_tbl[i].buf = buf;
					driver->buf_tbl[i].length =
								 driver->used;
#ifdef DIAG_DEBUG
					pr_debug("diag: ENQUEUE buf ptr and length is %p , %d\n",
						 driver->buf_tbl[i].buf,
						 driver->buf_tbl[i].length);
#endif
					break;
				}
		}

#ifdef CONFIG_DIAGFWD_BRIDGE_CODE
		else if (data_type == HSIC_DATA || data_type == HSIC_2_DATA) {
			unsigned long flags;
			int foundIndex = -1;
			index = data_type - HSIC_DATA;
			spin_lock_irqsave(&diag_hsic[index].hsic_spinlock,
									flags);
			for (i = 0; i < diag_hsic[index].poolsize_hsic_write;
									i++) {
				if (diag_hsic[index].hsic_buf_tbl[i].length
									== 0) {
					diag_hsic[index].hsic_buf_tbl[i].buf
									= buf;
					diag_hsic[index].hsic_buf_tbl[i].length
						= diag_bridge[index].write_len;
					diag_hsic[index].
						num_hsic_buf_tbl_entries++;
					foundIndex = i;
					break;
				}
			}
			spin_unlock_irqrestore(&diag_hsic[index].hsic_spinlock,
									flags);
			if (foundIndex == -1)
				err = -1;
			else
				pr_debug("diag: ENQUEUE HSIC buf ptr and length is %x , %d, ch %d\n",
					(unsigned int)buf,
					 diag_bridge[index].write_len, index);

			for (i = 0; i < driver->num_mdmclients; i++)
				if (driver->mdmclient_map[i].pid ==
						driver->mdm_logging_process_id)
					break;

			if (i < driver->num_mdmclients) {
				wake_lock_timeout(&driver->wake_lock, HZ / 2);
				driver->mdmdata_ready[i] |= USERMODE_DIAGFWD;
				pr_debug("diag_mdm: wake up logging process\n");
				wake_up_interruptible(&driver->mdmwait_q);

				return err;
			} else
				return -EINVAL;
		} else if (proc_num == SMUX_DATA) {
			for (i = 0; i < driver->num_qscclients; i++)
				if (driver->qscclient_map[i].pid ==
						driver->qsc_logging_process_id)
					break;

			if (i < driver->num_qscclients) {
				wake_lock_timeout(&driver->wake_lock, HZ / 2);
				driver->qscdata_ready[i] |= USERMODE_DIAGFWD;
				pr_debug("diag_qsc: wake up logging process\n");
				wake_up_interruptible(&driver->qscwait_q);

				return err;
			} else
				return -EINVAL;
		}
#endif
		for (i = 0; i < driver->num_clients; i++)
			if (driver->client_map[i].pid ==
						 driver->logging_process_id)
				break;
		if (i < driver->num_clients) {
			pr_debug("diag: wake up logging process\n");
			driver->data_ready[i] |= USERMODE_DIAGFWD;
			wake_up_interruptible(&driver->wait_q);
		} else
			return -EINVAL;
	} else if (driver->logging_mode == NO_LOGGING_MODE) {
		if ((data_type >= MODEM_DATA) && (data_type <= WCNSS_DATA)) {
			driver->smd_data[data_type].in_busy_1 = 0;
			driver->smd_data[data_type].in_busy_2 = 0;
			queue_work(driver->smd_data[data_type].wq,
				&(driver->smd_data[data_type].
							diag_read_smd_work));
			if (data_type == MODEM_DATA &&
				driver->separate_cmdrsp[data_type]) {
				driver->smd_cmd[data_type].in_busy_1 = 0;
				driver->smd_cmd[data_type].in_busy_2 = 0;
				queue_work(driver->diag_wq,
					&(driver->smd_cmd[data_type].
							diag_read_smd_work));
			}
		}
#ifdef CONFIG_DIAG_SDIO_PIPE
		else if (data_type == SDIO_DATA) {
			driver->in_busy_sdio = 0;
			queue_work(driver->diag_sdio_wq,
				&(driver->diag_read_sdio_work));
		}
#endif
#ifdef CONFIG_DIAGFWD_BRIDGE_CODE
		else if (data_type == HSIC_DATA || data_type == HSIC_2_DATA) {
			index = data_type - HSIC_DATA;
			if (diag_hsic[index].hsic_ch)
				queue_work(diag_bridge[index].wq,
					   &(diag_hsic[index].
					     diag_read_hsic_work));
		}
#endif
		err = -1;
	}
#ifdef CONFIG_DIAG_OVER_USB
	else if (driver->logging_mode == USB_MODE) {
		if (data_type == APPS_DATA) {
			driver->write_ptr_svc = (struct diag_request *)
			(diagmem_alloc(driver, sizeof(struct diag_request),
				 POOL_TYPE_WRITE_STRUCT));
			if (driver->write_ptr_svc) {
				driver->write_ptr_svc->length = driver->used;
				driver->write_ptr_svc->buf = buf;
				err = diag_write_to_usb(driver->legacy_ch,
						driver->write_ptr_svc);
				
				if (err) {
					diagmem_free(driver,
						     (unsigned char *)driver->
						     write_ptr_svc,
						     POOL_TYPE_WRITE_STRUCT);
				}
			} else {
				err = -ENOMEM;
			}
		} else if ((data_type >= MODEM_DATA) &&
				(data_type <= WCNSS_DATA)) {
			write_ptr->buf = buf;
#ifdef DIAG_DEBUG
			printk(KERN_INFO "writing data to USB,"
				"pkt length %d\n", write_ptr->length);
			print_hex_dump(KERN_DEBUG, "Written Packet Data to"
					   " USB: ", 16, 1, DUMP_PREFIX_ADDRESS,
					    buf, write_ptr->length, 1);
#endif 
			err = diag_write_to_usb(driver->legacy_ch, write_ptr);
		}
#ifdef CONFIG_DIAG_SDIO_PIPE
		else if (data_type == SDIO_DATA) {
			if (machine_is_msm8x60_fusion() ||
					 machine_is_msm8x60_fusn_ffa()) {
				write_ptr->buf = buf;
				err = usb_diag_write(driver->mdm_ch, write_ptr);
			} else
				pr_err("diag: Incorrect sdio data "
						"while USB write\n");
		}
#endif
#ifdef CONFIG_DIAGFWD_BRIDGE_CODE
		else if (data_type == HSIC_DATA || data_type == HSIC_2_DATA) {
			index = data_type - HSIC_DATA;
			if (diag_hsic[index].hsic_device_enabled) {
				struct diag_request *write_ptr_mdm;
				write_ptr_mdm = (struct diag_request *)
						diagmem_alloc(driver,
						sizeof(struct diag_request),
							index +
							POOL_TYPE_HSIC_WRITE);
				if (write_ptr_mdm) {
					write_ptr_mdm->buf = buf;
					write_ptr_mdm->length =
					   diag_bridge[index].write_len;
					write_ptr_mdm->context = (void *)index;
					err = usb_diag_write(
					diag_bridge[index].ch, write_ptr_mdm);
					
					if (err) {
						diagmem_free(driver,
							write_ptr_mdm,
							index +
							POOL_TYPE_HSIC_WRITE);
						pr_err_ratelimited("diag: HSIC write failure, err: %d, ch %d\n",
							err, index);
					}
				} else {
					pr_err("diag: allocate write fail\n");
					err = -1;
				}
			} else {
				pr_err("diag: Incorrect HSIC data "
						"while USB write\n");
				err = -1;
			}
		} else if (data_type == SMUX_DATA) {
				write_ptr->buf = buf;
				write_ptr->context = (void *)SMUX;
				pr_debug("diag: writing SMUX data\n");
				err = usb_diag_write(diag_bridge[SMUX].ch,
								 write_ptr);
		}
#endif
		APPEND_DEBUG('d');
	}
#endif 
    return err;
}

void diag_update_pkt_buffer(unsigned char *buf, int type)
{
	unsigned char *ptr = NULL;
	unsigned char *temp = buf;
	unsigned int length;
	int *in_busy = NULL;

	if (!buf) {
		pr_err("diag: Invalid buffer in %s\n", __func__);
		return;
	}

	switch (type) {
	case PKT_TYPE:
		ptr = driver->pkt_buf;
		length = driver->pkt_length;
		in_busy = &driver->in_busy_pktdata;
		break;
	case DCI_PKT_TYPE:
		ptr = driver->dci_pkt_buf;
		length = driver->dci_pkt_length;
		in_busy = &driver->in_busy_dcipktdata;
		break;
	default:
		pr_err("diag: Invalid type %d in %s\n", type, __func__);
		return;
	}

	if (!ptr || length == 0) {
		pr_err("diag: Invalid ptr %p and length %d in %s",
						ptr, length, __func__);
		return;
	}
	mutex_lock(&driver->diagchar_mutex);
	if (CHK_OVERFLOW(ptr, ptr, ptr + PKT_SIZE, length)) {
		memcpy(ptr, temp , length);
		*in_busy = 1;
	} else {
		printk(KERN_CRIT " Not enough buffer space for PKT_RESP\n");
	}
	mutex_unlock(&driver->diagchar_mutex);
}

void diag_update_userspace_clients(unsigned int type)
{
	int i;

	mutex_lock(&driver->diagchar_mutex);
	for (i = 0; i < driver->num_clients; i++)
		if (driver->client_map[i].pid != 0)
			driver->data_ready[i] |= type;
	wake_up_interruptible(&driver->wait_q);
	mutex_unlock(&driver->diagchar_mutex);
}

void diag_update_sleeping_process(int process_id, int data_type)
{
	int i;

	mutex_lock(&driver->diagchar_mutex);
	for (i = 0; i < driver->num_clients; i++)
		if (driver->client_map[i].pid == process_id) {
			driver->data_ready[i] |= data_type;
			break;
		}
	wake_up_interruptible(&driver->wait_q);
	mutex_unlock(&driver->diagchar_mutex);
}

int diag_send_data(struct diag_master_table entry, unsigned char *buf,
					 int len, int type)
{
	int success = 1;
	driver->pkt_length = len;

	
	if (entry.process_id != NON_APPS_PROC) {
		
		if (type != MODEM_DATA) {
			diag_update_pkt_buffer(buf, PKT_TYPE);
			diag_update_sleeping_process(entry.process_id,
							PKT_TYPE);
		}
	} else {
		if (len > 0) {
			if (entry.client_id < NUM_SMD_DATA_CHANNELS) {
				struct diag_smd_info *smd_info;
				int index = entry.client_id;
				if (!driver->rcvd_feature_mask[
					entry.client_id]) {
					pr_debug("diag: In %s, feature mask for peripheral: %d not received yet\n",
						__func__, entry.client_id);
					return 0;
				}
				smd_info = (driver->separate_cmdrsp[index] &&
						index < NUM_SMD_CMD_CHANNELS) ?
						&driver->smd_cmd[index] :
						&driver->smd_data[index];

				if (smd_info->ch) {
					mutex_lock(&smd_info->smd_ch_mutex);
					smd_write(smd_info->ch, buf, len);
					mutex_unlock(&smd_info->smd_ch_mutex);
				} else {
					pr_err("diag: In %s, smd channel %d not open, peripheral: %d, type: %d\n",
						__func__, index,
						smd_info->peripheral,
						smd_info->type);
				}
			} else {
				pr_alert("diag: In %s, incorrect channel: %d",
					__func__, entry.client_id);
				success = 0;
			}
		}
	}

	return success;
}

void diag_process_stm_mask(uint8_t cmd, uint8_t data_mask, int data_type,
			  uint8_t *rsp_supported, uint8_t *rsp_smd_comply)
{
	int status = 0;
	if (data_type >= MODEM_DATA && data_type <= WCNSS_DATA) {
		if (driver->peripheral_supports_stm[data_type]) {
			status = diag_send_stm_state(
				&driver->smd_cntl[data_type], cmd);
			if (status == 1)
				*rsp_smd_comply |= data_mask;
			*rsp_supported |= data_mask;
		} else if (driver->smd_cntl[data_type].ch) {
			*rsp_smd_comply |= data_mask;
		}
		if ((*rsp_smd_comply & data_mask) &&
			(*rsp_supported & data_mask))
			driver->stm_state[data_type] = cmd;

		driver->stm_state_requested[data_type] = cmd;
	} else if (data_type == APPS_DATA) {
		*rsp_supported |= data_mask;
		*rsp_smd_comply |= data_mask;
		driver->stm_state[data_type] = cmd;
		driver->stm_state_requested[data_type] = cmd;
	}
}

int diag_process_stm_cmd(unsigned char *buf, unsigned char *dest_buf)
{
	uint8_t version, mask, cmd;
	uint8_t rsp_supported = 0;
	uint8_t rsp_smd_comply = 0;
	int i;

	if (!buf || !dest_buf) {
		pr_err("diag: Invalid pointers buf: %p, dest_buf %p in %s\n",
		       buf, dest_buf, __func__);
		return -EIO;
	}

	version = *(buf + STM_CMD_VERSION_OFFSET);
	mask = *(buf + STM_CMD_MASK_OFFSET);
	cmd = *(buf + STM_CMD_DATA_OFFSET);

	if ((version != 1) || (cmd > STATUS_STM) ||
		((cmd != STATUS_STM) && ((mask == 0) || (0 != (mask >> 4))))) {
		
		dest_buf[0] = BAD_PARAM_RESPONSE_MESSAGE;
		for (i = 0; i < STM_CMD_NUM_BYTES; i++)
			dest_buf[i+1] = *(buf + i);
		return STM_CMD_NUM_BYTES+1;
	} else if (cmd == STATUS_STM) {
		for (i = 0; i < NUM_SMD_CONTROL_CHANNELS; i++)
			if (driver->peripheral_supports_stm[i])
				rsp_supported |= 1 << i;

		rsp_supported |= DIAG_STM_APPS;
	} else {
		if (mask & DIAG_STM_MODEM)
			diag_process_stm_mask(cmd, DIAG_STM_MODEM, MODEM_DATA,
					&rsp_supported, &rsp_smd_comply);

		if (mask & DIAG_STM_LPASS)
			diag_process_stm_mask(cmd, DIAG_STM_LPASS, LPASS_DATA,
					&rsp_supported, &rsp_smd_comply);

		if (mask & DIAG_STM_WCNSS)
			diag_process_stm_mask(cmd, DIAG_STM_WCNSS, WCNSS_DATA,
					&rsp_supported, &rsp_smd_comply);

		if (mask & DIAG_STM_APPS)
			diag_process_stm_mask(cmd, DIAG_STM_APPS, APPS_DATA,
					&rsp_supported, &rsp_smd_comply);
	}

	for (i = 0; i < STM_CMD_NUM_BYTES; i++)
		dest_buf[i] = *(buf + i);

	dest_buf[STM_RSP_VALID_INDEX] = STM_COMMAND_VALID;
	dest_buf[STM_RSP_SUPPORTED_INDEX] = rsp_supported;
	dest_buf[STM_RSP_SMD_COMPLY_INDEX] = rsp_smd_comply;

	return STM_RSP_NUM_BYTES;
}

int diag_apps_responds()
{
	if (chk_apps_only()) {
		if (driver->smd_data[MODEM_DATA].ch &&
				driver->rcvd_feature_mask[MODEM_DATA]) {
			return 0;
		}
		return 1;
	}
	return 0;
}

int diag_process_apps_pkt(unsigned char *buf, int len)
{
	uint16_t subsys_cmd_code;
	int subsys_id, ssid_first, ssid_last, ssid_range;
	int packet_type = 1, i, cmd_code;
	unsigned char *temp = buf;
	int data_type;
	int mask_ret;
	int status = 0;
#if defined(CONFIG_DIAG_OVER_USB)
	unsigned char *ptr;
#endif

	
	mask_ret = diag_process_apps_masks(buf, len);
	if (mask_ret <= 0)
		return mask_ret;

	
	cmd_code = (int)(*(char *)buf);
	temp++;
	subsys_id = (int)(*(char *)temp);
	temp++;
	subsys_cmd_code = *(uint16_t *)temp;
	temp += 2;
	data_type = APPS_DATA;
	
	if (chk_apps_master() && cmd_code == MODE_CMD) {
		if (subsys_id != RESET_ID)
			data_type = MODEM_DATA;
	}

	pr_debug("diag: %d %d %d", cmd_code, subsys_id, subsys_cmd_code);
	for (i = 0; i < diag_max_reg; i++) {
		entry = driver->table[i];
		if (entry.process_id != NO_PROCESS) {
			if (entry.cmd_code == cmd_code && entry.subsys_id ==
				 subsys_id && entry.cmd_code_lo <=
							 subsys_cmd_code &&
				  entry.cmd_code_hi >= subsys_cmd_code) {
				status = diag_send_data(entry, buf, len,
								data_type);
				if (status)
					packet_type = 0;
			} else if (entry.cmd_code == 255
				  && cmd_code == 75) {
				if (entry.subsys_id ==
					subsys_id &&
				   entry.cmd_code_lo <=
					subsys_cmd_code &&
					 entry.cmd_code_hi >=
					subsys_cmd_code) {
					status = diag_send_data(entry, buf,
								len, data_type);
					if (status)
						packet_type = 0;
				}
			} else if (entry.cmd_code == 255 &&
				  entry.subsys_id == 255) {
				if (entry.cmd_code_lo <=
						 cmd_code &&
						 entry.
						cmd_code_hi >= cmd_code) {
					if (cmd_code == 238)
					{
						int cmdID = (int)(*(char*)(buf+2));
						if(entry.process_id!=NON_APPS_PROC)
						{
							diag_send_data(entry, buf, len, data_type);
							printk(KERN_INFO "%s .. CCI FTM cmdID[0x%2X] for APP handling\n", __func__, cmdID);
							packet_type = 0;
							return 0;
						}
					}
					else
					{
						status = diag_send_data(entry, buf, len, data_type);
						if (status)
							packet_type = 0;
					}
				}
			}
		}
	}
#if defined(CONFIG_DIAG_OVER_USB)
	
	if ((*buf == 0x4b) && (*(buf+1) == 0x12) &&
		(*(uint16_t *)(buf+2) == 0x0055)) {
		for (i = 0; i < 4; i++)
			*(driver->apps_rsp_buf+i) = *(buf+i);
		*(uint32_t *)(driver->apps_rsp_buf+4) = PKT_SIZE;
		encode_rsp_and_send(7);
		return 0;
	} else if ((*buf == 0x4b) && (*(buf+1) == 0x12) &&
		(*(uint16_t *)(buf+2) == DIAG_DIAG_STM)) {
		len = diag_process_stm_cmd(buf, driver->apps_rsp_buf);
		if (len > 0) {
			encode_rsp_and_send(len - 1);
			return 0;
		}
		return len;
	}
	
	else if (diag_apps_responds() && *buf == 0x81) {
		driver->apps_rsp_buf[0] = 0x81;
		driver->apps_rsp_buf[1] = 0x0;
		*(uint16_t *)(driver->apps_rsp_buf + 2) = 0x0;
		*(uint16_t *)(driver->apps_rsp_buf + 4) = EVENT_LAST_ID + 1;
		for (i = 0; i < EVENT_LAST_ID/8 + 1; i++)
			*(unsigned char *)(driver->apps_rsp_buf + 6 + i) = 0x0;
		encode_rsp_and_send(6 + EVENT_LAST_ID/8);
		return 0;
	}
	
	else if (diag_apps_responds() && (*buf == 0x73) &&
							*(int *)(buf+4) == 1) {
		driver->apps_rsp_buf[0] = 0x73;
		*(int *)(driver->apps_rsp_buf + 4) = 0x1; 
		*(int *)(driver->apps_rsp_buf + 8) = 0x0; 
		*(int *)(driver->apps_rsp_buf + 12) =
				LOG_GET_ITEM_NUM(log_code_last_tbl[0]);
		*(int *)(driver->apps_rsp_buf + 16) =
				LOG_GET_ITEM_NUM(log_code_last_tbl[1]);
		*(int *)(driver->apps_rsp_buf + 20) =
				LOG_GET_ITEM_NUM(log_code_last_tbl[2]);
		*(int *)(driver->apps_rsp_buf + 24) =
				LOG_GET_ITEM_NUM(log_code_last_tbl[3]);
		*(int *)(driver->apps_rsp_buf + 28) =
				LOG_GET_ITEM_NUM(log_code_last_tbl[4]);
		*(int *)(driver->apps_rsp_buf + 32) =
				LOG_GET_ITEM_NUM(log_code_last_tbl[5]);
		*(int *)(driver->apps_rsp_buf + 36) =
				LOG_GET_ITEM_NUM(log_code_last_tbl[6]);
		*(int *)(driver->apps_rsp_buf + 40) =
				LOG_GET_ITEM_NUM(log_code_last_tbl[7]);
		*(int *)(driver->apps_rsp_buf + 44) =
				LOG_GET_ITEM_NUM(log_code_last_tbl[8]);
		*(int *)(driver->apps_rsp_buf + 48) =
				LOG_GET_ITEM_NUM(log_code_last_tbl[9]);
		*(int *)(driver->apps_rsp_buf + 52) =
				LOG_GET_ITEM_NUM(log_code_last_tbl[10]);
		*(int *)(driver->apps_rsp_buf + 56) =
				LOG_GET_ITEM_NUM(log_code_last_tbl[11]);
		*(int *)(driver->apps_rsp_buf + 60) =
				LOG_GET_ITEM_NUM(log_code_last_tbl[12]);
		*(int *)(driver->apps_rsp_buf + 64) =
				LOG_GET_ITEM_NUM(log_code_last_tbl[13]);
		*(int *)(driver->apps_rsp_buf + 68) =
				LOG_GET_ITEM_NUM(log_code_last_tbl[14]);
		*(int *)(driver->apps_rsp_buf + 72) =
				LOG_GET_ITEM_NUM(log_code_last_tbl[15]);
		encode_rsp_and_send(75);
		return 0;
	}
	
	else if (diag_apps_responds() && (*buf == 0x7d) &&
							(*(buf+1) == 0x1)) {
		driver->apps_rsp_buf[0] = 0x7d;
		driver->apps_rsp_buf[1] = 0x1;
		driver->apps_rsp_buf[2] = 0x1;
		driver->apps_rsp_buf[3] = 0x0;
		
		*(int *)(driver->apps_rsp_buf + 4) = MSG_MASK_TBL_CNT - 1;
		*(uint16_t *)(driver->apps_rsp_buf + 8) = MSG_SSID_0;
		*(uint16_t *)(driver->apps_rsp_buf + 10) = MSG_SSID_0_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 12) = MSG_SSID_1;
		*(uint16_t *)(driver->apps_rsp_buf + 14) = MSG_SSID_1_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 16) = MSG_SSID_2;
		*(uint16_t *)(driver->apps_rsp_buf + 18) = MSG_SSID_2_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 20) = MSG_SSID_3;
		*(uint16_t *)(driver->apps_rsp_buf + 22) = MSG_SSID_3_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 24) = MSG_SSID_4;
		*(uint16_t *)(driver->apps_rsp_buf + 26) = MSG_SSID_4_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 28) = MSG_SSID_5;
		*(uint16_t *)(driver->apps_rsp_buf + 30) = MSG_SSID_5_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 32) = MSG_SSID_6;
		*(uint16_t *)(driver->apps_rsp_buf + 34) = MSG_SSID_6_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 36) = MSG_SSID_7;
		*(uint16_t *)(driver->apps_rsp_buf + 38) = MSG_SSID_7_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 40) = MSG_SSID_8;
		*(uint16_t *)(driver->apps_rsp_buf + 42) = MSG_SSID_8_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 44) = MSG_SSID_9;
		*(uint16_t *)(driver->apps_rsp_buf + 46) = MSG_SSID_9_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 48) = MSG_SSID_10;
		*(uint16_t *)(driver->apps_rsp_buf + 50) = MSG_SSID_10_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 52) = MSG_SSID_11;
		*(uint16_t *)(driver->apps_rsp_buf + 54) = MSG_SSID_11_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 56) = MSG_SSID_12;
		*(uint16_t *)(driver->apps_rsp_buf + 58) = MSG_SSID_12_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 60) = MSG_SSID_13;
		*(uint16_t *)(driver->apps_rsp_buf + 62) = MSG_SSID_13_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 64) = MSG_SSID_14;
		*(uint16_t *)(driver->apps_rsp_buf + 66) = MSG_SSID_14_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 68) = MSG_SSID_15;
		*(uint16_t *)(driver->apps_rsp_buf + 70) = MSG_SSID_15_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 72) = MSG_SSID_16;
		*(uint16_t *)(driver->apps_rsp_buf + 74) = MSG_SSID_16_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 76) = MSG_SSID_17;
		*(uint16_t *)(driver->apps_rsp_buf + 78) = MSG_SSID_17_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 80) = MSG_SSID_18;
		*(uint16_t *)(driver->apps_rsp_buf + 82) = MSG_SSID_18_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 84) = MSG_SSID_19;
		*(uint16_t *)(driver->apps_rsp_buf + 86) = MSG_SSID_19_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 88) = MSG_SSID_20;
		*(uint16_t *)(driver->apps_rsp_buf + 90) = MSG_SSID_20_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 92) = MSG_SSID_21;
		*(uint16_t *)(driver->apps_rsp_buf + 94) = MSG_SSID_21_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 96) = MSG_SSID_22;
		*(uint16_t *)(driver->apps_rsp_buf + 98) = MSG_SSID_22_LAST;
		*(uint16_t *)(driver->apps_rsp_buf + 100) = MSG_SSID_23;
		*(uint16_t *)(driver->apps_rsp_buf + 102) = MSG_SSID_23_LAST;
		encode_rsp_and_send(103);
		return 0;
	}
	
	else if (diag_apps_responds() && (*buf == 0x7d) &&
							(*(buf+1) == 0x2)) {
		ssid_first = *(uint16_t *)(buf + 2);
		ssid_last = *(uint16_t *)(buf + 4);
		ssid_range = 4 * (ssid_last - ssid_first + 1);
		
		driver->apps_rsp_buf[0] = 0x7d;
		driver->apps_rsp_buf[1] = 0x2;
		*(uint16_t *)(driver->apps_rsp_buf + 2) = ssid_first;
		*(uint16_t *)(driver->apps_rsp_buf + 4) = ssid_last;
		driver->apps_rsp_buf[6] = 0x1;
		driver->apps_rsp_buf[7] = 0x0;
		ptr = driver->apps_rsp_buf + 8;
		
		switch (ssid_first) {
		case MSG_SSID_0:
			if (ssid_range > sizeof(msg_bld_masks_0)) {
				pr_warning("diag: truncating ssid range for ssid 0");
				ssid_range = sizeof(msg_bld_masks_0);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_0[i/4];
			break;
		case MSG_SSID_1:
			if (ssid_range > sizeof(msg_bld_masks_1)) {
				pr_warning("diag: truncating ssid range for ssid 1");
				ssid_range = sizeof(msg_bld_masks_1);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_1[i/4];
			break;
		case MSG_SSID_2:
			if (ssid_range > sizeof(msg_bld_masks_2)) {
				pr_warning("diag: truncating ssid range for ssid 2");
				ssid_range = sizeof(msg_bld_masks_2);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_2[i/4];
			break;
		case MSG_SSID_3:
			if (ssid_range > sizeof(msg_bld_masks_3)) {
				pr_warning("diag: truncating ssid range for ssid 3");
				ssid_range = sizeof(msg_bld_masks_3);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_3[i/4];
			break;
		case MSG_SSID_4:
			if (ssid_range > sizeof(msg_bld_masks_4)) {
				pr_warning("diag: truncating ssid range for ssid 4");
				ssid_range = sizeof(msg_bld_masks_4);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_4[i/4];
			break;
		case MSG_SSID_5:
			if (ssid_range > sizeof(msg_bld_masks_5)) {
				pr_warning("diag: truncating ssid range for ssid 5");
				ssid_range = sizeof(msg_bld_masks_5);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_5[i/4];
			break;
		case MSG_SSID_6:
			if (ssid_range > sizeof(msg_bld_masks_6)) {
				pr_warning("diag: truncating ssid range for ssid 6");
				ssid_range = sizeof(msg_bld_masks_6);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_6[i/4];
			break;
		case MSG_SSID_7:
			if (ssid_range > sizeof(msg_bld_masks_7)) {
				pr_warning("diag: truncating ssid range for ssid 7");
				ssid_range = sizeof(msg_bld_masks_7);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_7[i/4];
			break;
		case MSG_SSID_8:
			if (ssid_range > sizeof(msg_bld_masks_8)) {
				pr_warning("diag: truncating ssid range for ssid 8");
				ssid_range = sizeof(msg_bld_masks_8);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_8[i/4];
			break;
		case MSG_SSID_9:
			if (ssid_range > sizeof(msg_bld_masks_9)) {
				pr_warning("diag: truncating ssid range for ssid 9");
				ssid_range = sizeof(msg_bld_masks_9);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_9[i/4];
			break;
		case MSG_SSID_10:
			if (ssid_range > sizeof(msg_bld_masks_10)) {
				pr_warning("diag: truncating ssid range for ssid 10");
				ssid_range = sizeof(msg_bld_masks_10);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_10[i/4];
			break;
		case MSG_SSID_11:
			if (ssid_range > sizeof(msg_bld_masks_11)) {
				pr_warning("diag: truncating ssid range for ssid 11");
				ssid_range = sizeof(msg_bld_masks_11);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_11[i/4];
			break;
		case MSG_SSID_12:
			if (ssid_range > sizeof(msg_bld_masks_12)) {
				pr_warning("diag: truncating ssid range for ssid 12");
				ssid_range = sizeof(msg_bld_masks_12);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_12[i/4];
			break;
		case MSG_SSID_13:
			if (ssid_range > sizeof(msg_bld_masks_13)) {
				pr_warning("diag: truncating ssid range for ssid 13");
				ssid_range = sizeof(msg_bld_masks_13);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_13[i/4];
			break;
		case MSG_SSID_14:
			if (ssid_range > sizeof(msg_bld_masks_14)) {
				pr_warning("diag: truncating ssid range for ssid 14");
				ssid_range = sizeof(msg_bld_masks_14);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_14[i/4];
			break;
		case MSG_SSID_15:
			if (ssid_range > sizeof(msg_bld_masks_15)) {
				pr_warning("diag: truncating ssid range for ssid 15");
				ssid_range = sizeof(msg_bld_masks_15);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_15[i/4];
			break;
		case MSG_SSID_16:
			if (ssid_range > sizeof(msg_bld_masks_16)) {
				pr_warning("diag: truncating ssid range for ssid 16");
				ssid_range = sizeof(msg_bld_masks_16);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_16[i/4];
			break;
		case MSG_SSID_17:
			if (ssid_range > sizeof(msg_bld_masks_17)) {
				pr_warning("diag: truncating ssid range for ssid 17");
				ssid_range = sizeof(msg_bld_masks_17);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_17[i/4];
			break;
		case MSG_SSID_18:
			if (ssid_range > sizeof(msg_bld_masks_18)) {
				pr_warning("diag: truncating ssid range for ssid 18");
				ssid_range = sizeof(msg_bld_masks_18);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_18[i/4];
			break;
		case MSG_SSID_19:
			if (ssid_range > sizeof(msg_bld_masks_19)) {
				pr_warning("diag: truncating ssid range for ssid 19");
				ssid_range = sizeof(msg_bld_masks_19);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_19[i/4];
			break;
		case MSG_SSID_20:
			if (ssid_range > sizeof(msg_bld_masks_20)) {
				pr_warning("diag: truncating ssid range for ssid 20");
				ssid_range = sizeof(msg_bld_masks_20);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_20[i/4];
			break;
		case MSG_SSID_21:
			if (ssid_range > sizeof(msg_bld_masks_21)) {
				pr_warning("diag: truncating ssid range for ssid 21");
				ssid_range = sizeof(msg_bld_masks_21);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_21[i/4];
			break;
		case MSG_SSID_22:
			if (ssid_range > sizeof(msg_bld_masks_22)) {
				pr_warning("diag: truncating ssid range for ssid 22");
				ssid_range = sizeof(msg_bld_masks_22);
			}
			for (i = 0; i < ssid_range; i += 4)
				*(int *)(ptr + i) = msg_bld_masks_22[i/4];
			break;
		}
		encode_rsp_and_send(8 + ssid_range - 1);
		return 0;
	}
	
	else if ((cpu_is_msm8x60() || chk_apps_master()) && (*buf == 0x3A)) {
		
		driver->apps_rsp_buf[0] = *buf;
		encode_rsp_and_send(0);
		msleep(5000);
		
		msm_set_restart_mode(RESTART_DLOAD);
		printk(KERN_CRIT "diag: download mode set, Rebooting SoC..\n");
		kernel_restart(NULL);
		
		return 0;
	}
	
	else if ((*buf == 0x4b) && (*(buf+1) == 0x32) &&
		(*(buf+2) == 0x03)) {
		
		if (chk_polling_response()) {
			
			for (i = 0; i < 3; i++)
				driver->apps_rsp_buf[i] = *(buf+i);
			for (i = 0; i < 13; i++)
				driver->apps_rsp_buf[i+3] = 0;

			encode_rsp_and_send(15);
			return 0;
		}
	}
	
	else if ((*buf == 0x4b) && (*(buf+1) == 0x32) &&
		(*(buf+2) == 0x04) && (*(buf+3) == 0x0)) {
		memcpy(driver->apps_rsp_buf, buf, 4);
		driver->apps_rsp_buf[4] = wrap_enabled;
		encode_rsp_and_send(4);
		return 0;
	}
	
	else if ((*buf == 0x4b) && (*(buf+1) == 0x32) &&
		(*(buf+2) == 0x05) && (*(buf+3) == 0x0)) {
		wrap_enabled = true;
		memcpy(driver->apps_rsp_buf, buf, 4);
		driver->apps_rsp_buf[4] = wrap_count;
		encode_rsp_and_send(5);
		return 0;
	}
	 
	else if (chk_polling_response()) {
		
		if (*buf == 0x00) {
			for (i = 0; i < 55; i++)
				driver->apps_rsp_buf[i] = 0;

			encode_rsp_and_send(54);
			return 0;
		}
		
		else if (*buf == 0x7c) {
			driver->apps_rsp_buf[0] = 0x7c;
			for (i = 1; i < 8; i++)
				driver->apps_rsp_buf[i] = 0;
			
			*(int *)(driver->apps_rsp_buf + 8) =
							 chk_config_get_id();
			*(unsigned char *)(driver->apps_rsp_buf + 12) = '\0';
			*(unsigned char *)(driver->apps_rsp_buf + 13) = '\0';
			encode_rsp_and_send(13);
			return 0;
		}
	}
#endif
	return packet_type;
}

#ifdef CONFIG_DIAG_OVER_USB
void diag_send_error_rsp(int index)
{
	int i;

	
	if (index > APPS_BUF_SIZE-1) {
		pr_err("diag: cannot send err rsp, huge length: %d\n", index);
		return;
	}

	driver->apps_rsp_buf[0] = 0x13; 
	for (i = 0; i < index; i++)
		driver->apps_rsp_buf[i+1] = *(driver->hdlc_buf+i);
	encode_rsp_and_send(index - 3);
}
#else
static inline void diag_send_error_rsp(int index) {}
#endif

void diag_process_hdlc(void *data, unsigned len)
{
	struct diag_hdlc_decode_type hdlc;
	int ret, type = 0, crc_chk = 0;

	mutex_lock(&driver->diag_hdlc_mutex);

	pr_debug("diag: HDLC decode fn, len of data  %d\n", len);
	hdlc.dest_ptr = driver->hdlc_buf;
	hdlc.dest_size = USB_MAX_OUT_BUF;
	hdlc.src_ptr = data;
	hdlc.src_size = len;
	hdlc.src_idx = 0;
	hdlc.dest_idx = 0;
	hdlc.escaping = 0;

	ret = diag_hdlc_decode(&hdlc);
	if (ret) {
		crc_chk = crc_check(hdlc.dest_ptr, hdlc.dest_idx);
		if (crc_chk) {
			
			pr_err_ratelimited("diag: In %s, bad CRC. Dropping packet\n",
				__func__);
			mutex_unlock(&driver->diag_hdlc_mutex);
			return;
		}
	}

	if (hdlc.dest_idx < 4) {
		pr_err_ratelimited("diag: In %s, message is too short, len: %d, dest len: %d\n",
			__func__, len, hdlc.dest_idx);
		mutex_unlock(&driver->diag_hdlc_mutex);
		return;
	}

	if (ret) {
		type = diag_process_apps_pkt(driver->hdlc_buf,
							  hdlc.dest_idx - 3);
		if (type < 0) {
			mutex_unlock(&driver->diag_hdlc_mutex);
			return;
		}
	} else if (driver->debug_flag) {
		pr_err("diag: In %s, partial packet received, dropping packet, len: %d\n",
			__func__, len);

		print_hex_dump(KERN_DEBUG, "Dropped Packet Data: ", 16, 1,
					   DUMP_PREFIX_ADDRESS, data, len, 1);
		driver->debug_flag = 0;
	}
	
	if (type == 1 && chk_apps_only()) {
		diag_send_error_rsp(hdlc.dest_idx);
		type = 0;
	}
	
	if (!(driver->smd_data[MODEM_DATA].ch) && type == 1) {
		if (chk_apps_only()) {
			diag_send_error_rsp(hdlc.dest_idx);
		} else { 
			if (driver->smd_data[LPASS_DATA].ch) {
				mutex_lock(&driver->smd_data[LPASS_DATA].
								smd_ch_mutex);
				smd_write(driver->smd_data[LPASS_DATA].ch,
						driver->hdlc_buf,
						hdlc.dest_idx - 3);
				mutex_unlock(&driver->smd_data[LPASS_DATA].
								smd_ch_mutex);
			}
		}
		type = 0;
	}

#ifdef DIAG_DEBUG
	pr_debug("diag: hdlc.dest_idx = %d", hdlc.dest_idx);
	for (i = 0; i < hdlc.dest_idx; i++)
		printk(KERN_DEBUG "\t%x", *(((unsigned char *)
							driver->hdlc_buf)+i));
#endif 
	
	if ((driver->smd_data[MODEM_DATA].ch) && (ret) && (type) &&
						(hdlc.dest_idx > 3)) {
		APPEND_DEBUG('g');
		mutex_lock(&driver->smd_data[MODEM_DATA].smd_ch_mutex);
		smd_write(driver->smd_data[MODEM_DATA].ch,
					driver->hdlc_buf, hdlc.dest_idx - 3);
		mutex_unlock(&driver->smd_data[MODEM_DATA].smd_ch_mutex);
		APPEND_DEBUG('h');
#ifdef DIAG_DEBUG
		printk(KERN_INFO "writing data to SMD, pkt length %d\n", len);
		print_hex_dump(KERN_DEBUG, "Written Packet Data to SMD: ", 16,
			       1, DUMP_PREFIX_ADDRESS, data, len, 1);
#endif 
	}
	mutex_unlock(&driver->diag_hdlc_mutex);
}

void diag_reset_smd_data(int queue)
{
	int i;
	unsigned long flags;

	for (i = 0; i < NUM_SMD_DATA_CHANNELS; i++) {
		spin_lock_irqsave(&driver->smd_data[i].in_busy_lock, flags);
		driver->smd_data[i].in_busy_1 = 0;
		driver->smd_data[i].in_busy_2 = 0;
		spin_unlock_irqrestore(&driver->smd_data[i].in_busy_lock,
				       flags);
		if (queue)
			
			queue_work(driver->smd_data[i].wq,
				&(driver->smd_data[i].diag_read_smd_work));
	}

	if (driver->supports_separate_cmdrsp) {
		for (i = 0; i < NUM_SMD_CMD_CHANNELS; i++) {
			spin_lock_irqsave(&driver->smd_cmd[i].in_busy_lock,
					  flags);
			driver->smd_cmd[i].in_busy_1 = 0;
			driver->smd_cmd[i].in_busy_2 = 0;
			spin_unlock_irqrestore(&driver->smd_cmd[i].in_busy_lock,
					       flags);
			if (queue)
				
				queue_work(driver->diag_wq,
					&(driver->smd_cmd[i].
						diag_read_smd_work));
		}
	}
}

#ifdef CONFIG_DIAG_OVER_USB
#define N_LEGACY_WRITE	(driver->poolsize + 6)
#define N_LEGACY_WRITE_CMD ((N_LEGACY_WRITE) + 4)
#define N_LEGACY_READ	1

static void diag_usb_connect_work_fn(struct work_struct *w)
{
	diagfwd_connect();
}

static void diag_usb_disconnect_work_fn(struct work_struct *w)
{
	diagfwd_disconnect();
}

int diagfwd_connect(void)
{
	int err;
	int i;

	printk(KERN_INFO "diag: USB connected\n");
	err = usb_diag_alloc_req(driver->legacy_ch,
			(driver->supports_separate_cmdrsp ?
			N_LEGACY_WRITE_CMD : N_LEGACY_WRITE),
			N_LEGACY_READ);
	if (err) {
		printk(KERN_ERR "diag: unable to alloc USB req on legacy ch");
		goto exit;
	}
	driver->usb_connected = 1;
	driver->qxdmusb_drop = 0;
	diag_reset_smd_data(RESET_AND_QUEUE);
	for (i = 0; i < NUM_SMD_CONTROL_CHANNELS; i++) {
		
		diag_smd_notify(&(driver->smd_cntl[i]), SMD_EVENT_DATA);
	}
	queue_work(driver->diag_real_time_wq,
				 &driver->diag_real_time_work);

	
	queue_work(driver->diag_wq, &(driver->diag_read_work));
#ifdef CONFIG_DIAG_SDIO_PIPE
	if (machine_is_msm8x60_fusion() || machine_is_msm8x60_fusn_ffa()) {
		if (driver->mdm_ch && !IS_ERR(driver->mdm_ch))
			diagfwd_connect_sdio();
		else
			printk(KERN_INFO "diag: No USB MDM ch");
	}
#endif

	return 0;
exit:
	pr_err("diag: unable to alloc USB req on legacy ch, err: %d", err);
	return err;
}

int diagfwd_disconnect(void)
{
	int i;
	unsigned long flags;
	struct diag_smd_info *smd_info = NULL;

	printk(KERN_INFO "diag: USB disconnected\n");
	driver->usb_connected = 0;
	driver->debug_flag = 1;
	if (driver->logging_mode == USB_MODE) {
		driver->qxdmusb_drop = 1;
		for (i = 1; i < NUM_SMD_DATA_CHANNELS; i++) {
			smd_info = &driver->smd_data[i];
			spin_lock_irqsave(&smd_info->in_busy_lock, flags);
			smd_info->in_busy_1 = 1;
			smd_info->in_busy_2 = 1;
			spin_unlock_irqrestore(&smd_info->in_busy_lock, flags);
		}

		if (driver->supports_separate_cmdrsp) {
			for (i = 1; i < NUM_SMD_CMD_CHANNELS; i++) {
				smd_info = &driver->smd_cmd[i];
				spin_lock_irqsave(&smd_info->in_busy_lock,
						  flags);
				smd_info->in_busy_1 = 1;
				smd_info->in_busy_2 = 1;
				spin_unlock_irqrestore(&smd_info->in_busy_lock,
						       flags);
			}
		}
	}
	queue_work(driver->diag_real_time_wq,
				 &driver->diag_real_time_work);
#ifdef CONFIG_DIAG_SDIO_PIPE
	if (machine_is_msm8x60_fusion() || machine_is_msm8x60_fusn_ffa())
		if (driver->mdm_ch && !IS_ERR(driver->mdm_ch))
			diagfwd_disconnect_sdio();
#endif
	
	return 0;
}

static int diagfwd_check_buf_match(int num_channels,
			struct diag_smd_info *data, unsigned char *buf)
{
	int i;
	int found_it = 0;
	unsigned long flags;

	spin_lock_irqsave(&data->in_busy_lock, flags);
	for (i = 0; i < num_channels; i++) {
		if (buf == (void *)data[i].buf_in_1) {
			data[i].in_busy_1 = 0;
			found_it = 1;
			break;
		} else if (buf == (void *)data[i].buf_in_2) {
			data[i].in_busy_2 = 0;
			found_it = 1;
			break;
		}
	}
	spin_unlock_irqrestore(&data->in_busy_lock, flags);

	if (found_it) {
		if (data[i].type == SMD_DATA_TYPE)
			queue_work(data[i].wq,
					&(data[i].diag_read_smd_work));
		else
			queue_work(driver->diag_wq,
					&(data[i].diag_read_smd_work));
	}

	return found_it;
}

int diagfwd_write_complete(struct diag_request *diag_write_ptr)
{
	unsigned char *buf = diag_write_ptr->buf;
	int found_it = 0;

	
	found_it = diagfwd_check_buf_match(NUM_SMD_DATA_CHANNELS,
						driver->smd_data, buf);

	if (!found_it && driver->supports_separate_cmdrsp)
		found_it = diagfwd_check_buf_match(NUM_SMD_CMD_CHANNELS,
						driver->smd_cmd, buf);

#ifdef CONFIG_DIAG_SDIO_PIPE
	if (!found_it) {
		if (buf == (void *)driver->buf_in_sdio) {
			if (machine_is_msm8x60_fusion() ||
				 machine_is_msm8x60_fusn_ffa())
				diagfwd_write_complete_sdio();
			else
				pr_err("diag: Incorrect buffer pointer while WRITE");
			found_it = 1;
		}
	}
#endif
	if (!found_it) {
		if (driver->logging_mode != USB_MODE)
			pr_debug("diag: freeing buffer when not in usb mode\n");

		diagmem_free(driver, (unsigned char *)buf,
						POOL_TYPE_HDLC);
		diagmem_free(driver, (unsigned char *)diag_write_ptr,
						POOL_TYPE_WRITE_STRUCT);
	}
	return 0;
}

int diagfwd_read_complete(struct diag_request *diag_read_ptr)
{
	int status = diag_read_ptr->status;
	unsigned char *buf = diag_read_ptr->buf;

	
	if (buf == (void *)driver->usb_buf_out) {
		driver->read_len_legacy = diag_read_ptr->actual;
		APPEND_DEBUG('s');
#ifdef DIAG_DEBUG
		printk(KERN_INFO "read data from USB, pkt length %d",
		    diag_read_ptr->actual);
		print_hex_dump(KERN_DEBUG, "Read Packet Data from USB: ", 16, 1,
		       DUMP_PREFIX_ADDRESS, diag_read_ptr->buf,
		       diag_read_ptr->actual, 1);
#endif 
#if DIAG_XPST && !defined(CONFIG_DIAGFWD_BRIDGE_CODE)
		if (driver->nohdlc) {
			driver->usb_read_ptr->buf = driver->usb_buf_out;
			driver->usb_read_ptr->length = USB_MAX_OUT_BUF;
			usb_diag_read(driver->legacy_ch, driver->usb_read_ptr);
			return 0;
		}
#endif
		if (driver->logging_mode == USB_MODE) {
			if (status != -ECONNRESET && status != -ESHUTDOWN)
				queue_work(driver->diag_wq,
					&(driver->diag_proc_hdlc_work));
			else
				queue_work(driver->diag_wq,
						 &(driver->diag_read_work));
		}
	}
#ifdef CONFIG_DIAG_SDIO_PIPE
	else if (buf == (void *)driver->usb_buf_mdm_out) {
		if (machine_is_msm8x60_fusion() ||
				 machine_is_msm8x60_fusn_ffa()) {
			driver->read_len_mdm = diag_read_ptr->actual;
			diagfwd_read_complete_sdio();
		} else
			pr_err("diag: Incorrect buffer pointer while READ");
	}
#endif
	else
		printk(KERN_ERR "diag: Unknown buffer ptr from USB");

	return 0;
}

void diag_read_work_fn(struct work_struct *work)
{
	APPEND_DEBUG('d');
	driver->usb_read_ptr->buf = driver->usb_buf_out;
	driver->usb_read_ptr->length = USB_MAX_OUT_BUF;
	usb_diag_read(driver->legacy_ch, driver->usb_read_ptr);
	APPEND_DEBUG('e');
}

void diag_process_hdlc_fn(struct work_struct *work)
{
	APPEND_DEBUG('D');
	diag_process_hdlc(driver->usb_buf_out, driver->read_len_legacy);
	diag_read_work_fn(work);
	APPEND_DEBUG('E');
}

void diag_usb_legacy_notifier(void *priv, unsigned event,
			struct diag_request *d_req)
{
	switch (event) {
	case USB_DIAG_CONNECT:
		queue_work(driver->diag_usb_wq,
			 &driver->diag_usb_connect_work);
		break;
	case USB_DIAG_DISCONNECT:
		queue_work(driver->diag_usb_wq,
			 &driver->diag_usb_disconnect_work);
		break;
	case USB_DIAG_READ_DONE:
		diagfwd_read_complete(d_req);
		break;
	case USB_DIAG_WRITE_DONE:
		diagfwd_write_complete(d_req);
		break;
	default:
		printk(KERN_ERR "Unknown event from USB diag\n");
		break;
	}
}
#endif 

void diag_smd_notify(void *ctxt, unsigned event)
{
	struct diag_smd_info *smd_info = (struct diag_smd_info *)ctxt;
	if (!smd_info)
		return;

	if (event == SMD_EVENT_CLOSE) {
		smd_info->ch = 0;
		if (smd_info->peripheral == MODEM_DATA) {
			smd_diag_initialized = 0;
		}
		wake_up(&driver->smd_wait_q);
		if (smd_info->type == SMD_DATA_TYPE) {
			smd_info->notify_context = event;
			queue_work(driver->diag_cntl_wq,
				 &(smd_info->diag_notify_update_smd_work));
		} else if (smd_info->type == SMD_DCI_TYPE) {
			
			diag_dci_notify_client(smd_info->peripheral_mask,
							DIAG_STATUS_CLOSED);
		} else if (smd_info->type == SMD_CNTL_TYPE) {
			diag_cntl_stm_notify(smd_info,
						CLEAR_PERIPHERAL_STM_STATE);
		}
		return;
	} else if (event == SMD_EVENT_OPEN) {
		if (smd_info->ch_save)
			smd_info->ch = smd_info->ch_save;
		if (smd_info->peripheral == MODEM_DATA) {
			smd_diag_initialized = 1;
		}

		if (smd_info->type == SMD_CNTL_TYPE) {
			smd_info->notify_context = event;
			queue_work(driver->diag_cntl_wq,
				&(smd_info->diag_notify_update_smd_work));
		} else if (smd_info->type == SMD_DCI_TYPE) {
			smd_info->notify_context = event;
			queue_work(driver->diag_dci_wq,
				&(smd_info->diag_notify_update_smd_work));
			
			diag_dci_notify_client(smd_info->peripheral_mask,
							DIAG_STATUS_OPEN);
		}
		wake_up(&driver->smd_wait_q);
		diag_smd_queue_read(smd_info);
	} else if (event == SMD_EVENT_DATA) {
		wake_up(&driver->smd_wait_q);
		diag_smd_queue_read(smd_info);
		if (smd_info->type == SMD_DCI_TYPE ||
		    smd_info->type == SMD_DCI_CMD_TYPE) {
			diag_dci_try_activate_wakeup_source();
		}
	}
}

#if DIAG_XPST
void diag_smd_enable(smd_channel_t *ch, char *src, int mode)
{
	int r = 0, index = MODEM_DATA;
	static smd_channel_t *_ch;
	DIAGFWD_INFO("smd_try_open(%s): mode=%d\n", src, mode);

	mutex_lock(&driver->smd_lock);
	diag_smd_function_mode = mode;
	if (mode) {
		if (!driver->smd_data[index].ch) {
			r = smd_open("DIAG", &driver->smd_data[index].ch, driver, diag_smd_notify);
		if (!r)
			_ch = driver->smd_data[index].ch;
		} else
			_ch = driver->smd_data[index].ch;
	} else {
		if (driver->smd_data[index].ch) {
			r = smd_close(driver->smd_data[index].ch);
			driver->smd_data[index].ch = NULL;
			if (!r)
				_ch = driver->smd_data[index].ch;
		}
	}
	ch = _ch;
	mutex_unlock(&driver->smd_lock);
	DIAGFWD_INFO("smd_try_open(%s): r=%d _ch=%x\n", src, r, (unsigned int)ch);
}
#endif



static int diag_smd_probe(struct platform_device *pdev)
{
	int r = 0;
	int index = -1;
	const char *channel_name = NULL;

	if (pdev->id == SMD_APPS_MODEM) {
		index = MODEM_DATA;
		channel_name = "DIAG";
	}
#if defined(CONFIG_MSM_N_WAY_SMD)
	else if (pdev->id == SMD_APPS_QDSP) {
		index = LPASS_DATA;
		channel_name = "DIAG";
	}
#endif
	else if (pdev->id == SMD_APPS_WCNSS) {
		index = WCNSS_DATA;
		channel_name = "APPS_RIVA_DATA";
	}

	if (index != -1) {
		r = smd_named_open_on_edge(channel_name,
					pdev->id,
					&driver->smd_data[index].ch,
					&driver->smd_data[index],
					diag_smd_notify);
		driver->smd_data[index].ch_save = driver->smd_data[index].ch;
	}

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pr_debug("diag: In %s, open SMD port, Id = %d, r = %d\n",
		__func__, pdev->id, r);

	return 0;
}

static int diag_smd_cmd_probe(struct platform_device *pdev)
{
	int r = 0;
	int index = -1;
	const char *channel_name = NULL;

	if (!driver->supports_separate_cmdrsp)
		return 0;

	if (pdev->id == SMD_APPS_MODEM) {
		index = MODEM_DATA;
		channel_name = "DIAG_CMD";
	}

	if (index != -1) {
		r = smd_named_open_on_edge(channel_name,
			pdev->id,
			&driver->smd_cmd[index].ch,
			&driver->smd_cmd[index],
			diag_smd_notify);
		driver->smd_cmd[index].ch_save =
			driver->smd_cmd[index].ch;
	}

	pr_debug("diag: In %s, open SMD CMD port, Id = %d, r = %d\n",
		__func__, pdev->id, r);

	return 0;
}

static int diag_smd_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int diag_smd_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static const struct dev_pm_ops diag_smd_dev_pm_ops = {
	.runtime_suspend = diag_smd_runtime_suspend,
	.runtime_resume = diag_smd_runtime_resume,
};

static struct platform_driver msm_smd_ch1_driver = {

	.probe = diag_smd_probe,
	.driver = {
		.name = "DIAG",
		.owner = THIS_MODULE,
		.pm   = &diag_smd_dev_pm_ops,
	},
};

static struct platform_driver diag_smd_lite_driver = {

	.probe = diag_smd_probe,
	.driver = {
		.name = "APPS_RIVA_DATA",
		.owner = THIS_MODULE,
		.pm   = &diag_smd_dev_pm_ops,
	},
};

static struct platform_driver
		smd_lite_data_cmd_drivers[NUM_SMD_CMD_CHANNELS] = {
	{
		
		.probe = diag_smd_cmd_probe,
		.driver = {
			.name = "DIAG_CMD",
			.owner = THIS_MODULE,
			.pm   = &diag_smd_dev_pm_ops,
		},
	}
};

int device_supports_separate_cmdrsp(void)
{
	return driver->use_device_tree;
}

void diag_smd_destructor(struct diag_smd_info *smd_info)
{
	if (smd_info->type == SMD_DATA_TYPE)
		destroy_workqueue(smd_info->wq);

	if (smd_info->ch)
		smd_close(smd_info->ch);

	smd_info->ch = 0;
	smd_info->ch_save = 0;
	kfree(smd_info->buf_in_1);
	kfree(smd_info->buf_in_2);
	kfree(smd_info->write_ptr_1);
	kfree(smd_info->write_ptr_2);
	kfree(smd_info->buf_in_1_raw);
	kfree(smd_info->buf_in_2_raw);
}

int diag_smd_constructor(struct diag_smd_info *smd_info, int peripheral,
			  int type)
{
	smd_info->peripheral = peripheral;
	smd_info->type = type;
	smd_info->encode_hdlc = 0;
	mutex_init(&smd_info->smd_ch_mutex);
	spin_lock_init(&smd_info->in_busy_lock);

	switch (peripheral) {
	case MODEM_DATA:
		smd_info->peripheral_mask = DIAG_CON_MPSS;
		break;
	case LPASS_DATA:
		smd_info->peripheral_mask = DIAG_CON_LPASS;
		break;
	case WCNSS_DATA:
		smd_info->peripheral_mask = DIAG_CON_WCNSS;
		break;
	default:
		pr_err("diag: In %s, unknown peripheral, peripheral: %d\n",
			__func__, peripheral);
		goto err;
	}

	smd_info->ch = 0;
	smd_info->ch_save = 0;

	if (smd_info->buf_in_1 == NULL) {
		smd_info->buf_in_1 = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
		if (smd_info->buf_in_1 == NULL)
			goto err;
		smd_info->buf_in_1_size = IN_BUF_SIZE;
		kmemleak_not_leak(smd_info->buf_in_1);
	}

	if (smd_info->write_ptr_1 == NULL) {
		smd_info->write_ptr_1 = kzalloc(sizeof(struct diag_request),
								GFP_KERNEL);
		if (smd_info->write_ptr_1 == NULL)
			goto err;
		kmemleak_not_leak(smd_info->write_ptr_1);
	}

	
	if (smd_info->type == SMD_DATA_TYPE) {
		if (smd_info->buf_in_2 == NULL) {
			smd_info->buf_in_2 = kzalloc(IN_BUF_SIZE, GFP_KERNEL);
			if (smd_info->buf_in_2 == NULL)
				goto err;
			smd_info->buf_in_2_size = IN_BUF_SIZE;
			kmemleak_not_leak(smd_info->buf_in_2);
		}
		if (smd_info->write_ptr_2 == NULL) {
			smd_info->write_ptr_2 =
				kzalloc(sizeof(struct diag_request),
				GFP_KERNEL);
			if (smd_info->write_ptr_2 == NULL)
				goto err;
			kmemleak_not_leak(smd_info->write_ptr_2);
		}
		if (driver->supports_apps_hdlc_encoding) {
			
			if (smd_info->buf_in_1_raw == NULL) {
				smd_info->buf_in_1_raw = kzalloc(IN_BUF_SIZE,
								GFP_KERNEL);
				if (smd_info->buf_in_1_raw == NULL)
					goto err;
				smd_info->buf_in_1_raw_size = IN_BUF_SIZE;
				kmemleak_not_leak(smd_info->buf_in_1_raw);
			}
			if (smd_info->buf_in_2_raw == NULL) {
				smd_info->buf_in_2_raw = kzalloc(IN_BUF_SIZE,
								GFP_KERNEL);
				if (smd_info->buf_in_2_raw == NULL)
					goto err;
				smd_info->buf_in_2_raw_size = IN_BUF_SIZE;
				kmemleak_not_leak(smd_info->buf_in_2_raw);
			}
		}
	}

	if (smd_info->type == SMD_CMD_TYPE &&
		driver->supports_apps_hdlc_encoding) {
		
		if (smd_info->buf_in_1_raw == NULL) {
			smd_info->buf_in_1_raw = kzalloc(IN_BUF_SIZE,
								GFP_KERNEL);
			if (smd_info->buf_in_1_raw == NULL)
				goto err;
			smd_info->buf_in_1_raw_size = IN_BUF_SIZE;
			kmemleak_not_leak(smd_info->buf_in_1_raw);
		}
	}

	
	if (type == SMD_DATA_TYPE) {
		switch (peripheral) {
		case MODEM_DATA:
			smd_info->wq = create_singlethread_workqueue(
						"diag_modem_data_read_wq");
			break;
		case LPASS_DATA:
			smd_info->wq = create_singlethread_workqueue(
						"diag_lpass_data_read_wq");
			break;
		case WCNSS_DATA:
			smd_info->wq = create_singlethread_workqueue(
						"diag_wcnss_data_read_wq");
			break;
		default:
			smd_info->wq = NULL;
			break;
		}
	} else {
		smd_info->wq = NULL;
	}

	INIT_WORK(&(smd_info->diag_read_smd_work), diag_read_smd_work_fn);

	smd_info->notify_context = 0;
	smd_info->general_context = 0;
	switch (type) {
	case SMD_DATA_TYPE:
	case SMD_CMD_TYPE:
		INIT_WORK(&(smd_info->diag_notify_update_smd_work),
						diag_clean_reg_fn);
		INIT_WORK(&(smd_info->diag_general_smd_work),
						diag_cntl_smd_work_fn);
		break;
	case SMD_CNTL_TYPE:
		INIT_WORK(&(smd_info->diag_notify_update_smd_work),
						diag_mask_update_fn);
		INIT_WORK(&(smd_info->diag_general_smd_work),
						diag_cntl_smd_work_fn);
		break;
	case SMD_DCI_TYPE:
	case SMD_DCI_CMD_TYPE:
		INIT_WORK(&(smd_info->diag_notify_update_smd_work),
					diag_update_smd_dci_work_fn);
		INIT_WORK(&(smd_info->diag_general_smd_work),
					diag_cntl_smd_work_fn);
		break;
	default:
		pr_err("diag: In %s, unknown type, type: %d\n", __func__, type);
		goto err;
	}

	switch (type) {
	case SMD_DATA_TYPE:
	case SMD_CMD_TYPE:
		smd_info->process_smd_read_data = diag_process_smd_read_data;
		break;
	case SMD_CNTL_TYPE:
		smd_info->process_smd_read_data =
						diag_process_smd_cntl_read_data;
		break;
	case SMD_DCI_TYPE:
	case SMD_DCI_CMD_TYPE:
		smd_info->process_smd_read_data =
						diag_process_smd_dci_read_data;
		break;
	default:
		pr_err("diag: In %s, unknown type, type: %d\n", __func__, type);
		goto err;
	}

	return 1;
err:
	kfree(smd_info->buf_in_1);
	kfree(smd_info->buf_in_2);
	kfree(smd_info->write_ptr_1);
	kfree(smd_info->write_ptr_2);
	kfree(smd_info->buf_in_1_raw);
	kfree(smd_info->buf_in_2_raw);

	return 0;
}

void diagfwd_init(void)
{
	int success;
	int i;

	wrap_enabled = 0;
	wrap_count = 0;
	diag_debug_buf_idx = 0;
	driver->read_len_legacy = 0;
	driver->use_device_tree = has_device_tree();
	driver->real_time_mode = 1;
	driver->buf_tbl_size = (buf_tbl_size < driver->poolsize_hdlc) ?
				driver->poolsize_hdlc : buf_tbl_size;
	driver->supports_separate_cmdrsp = device_supports_separate_cmdrsp();
	driver->supports_apps_hdlc_encoding = 0;
	mutex_init(&driver->diag_hdlc_mutex);
	mutex_init(&driver->diag_cntl_mutex);
	spin_lock_init(&driver->ws_lock);
	driver->ws_ref_count = 0;
	driver->copy_count = 0;

	for (i = 0; i < NUM_SMD_CONTROL_CHANNELS; i++) {
		driver->separate_cmdrsp[i] = 0;
		driver->peripheral_supports_stm[i] = DISABLE_STM;
		driver->rcvd_feature_mask[i] = 0;
	}

	for (i = 0; i < NUM_STM_PROCESSORS; i++) {
		driver->stm_state_requested[i] = DISABLE_STM;
		driver->stm_state[i] = DISABLE_STM;
	}

	for (i = 0; i < NUM_SMD_DATA_CHANNELS; i++) {
		success = diag_smd_constructor(&driver->smd_data[i], i,
							SMD_DATA_TYPE);
		if (!success)
			goto err;
	}

	if (driver->supports_separate_cmdrsp) {
		for (i = 0; i < NUM_SMD_CMD_CHANNELS; i++) {
			success = diag_smd_constructor(&driver->smd_cmd[i], i,
								SMD_CMD_TYPE);
			if (!success)
				goto err;
		}
	}

	if (driver->usb_buf_out  == NULL &&
	     (driver->usb_buf_out = kzalloc(USB_MAX_OUT_BUF,
					 GFP_KERNEL)) == NULL)
		goto err;
	kmemleak_not_leak(driver->usb_buf_out);
	if (driver->hdlc_buf == NULL
	    && (driver->hdlc_buf = kzalloc(HDLC_MAX, GFP_KERNEL)) == NULL)
		goto err;
	kmemleak_not_leak(driver->hdlc_buf);
	if (driver->user_space_data_buf == NULL)
		driver->user_space_data_buf = kzalloc(USER_SPACE_DATA,
							GFP_KERNEL);
	if (driver->user_space_data_buf == NULL)
		goto err;
	kmemleak_not_leak(driver->user_space_data_buf);
	if (driver->client_map == NULL &&
	    (driver->client_map = kzalloc
	     ((driver->num_clients) * sizeof(struct diag_client_map),
		   GFP_KERNEL)) == NULL)
		goto err;
	kmemleak_not_leak(driver->client_map);
	if (driver->buf_tbl == NULL)
			driver->buf_tbl = kzalloc(driver->buf_tbl_size *
			  sizeof(struct diag_write_device), GFP_KERNEL);
	if (driver->buf_tbl == NULL)
		goto err;
	kmemleak_not_leak(driver->buf_tbl);
	if (driver->data_ready == NULL &&
	     (driver->data_ready = kzalloc(driver->num_clients * sizeof(int)
							, GFP_KERNEL)) == NULL)
		goto err;
	kmemleak_not_leak(driver->data_ready);
	if (driver->table == NULL &&
	     (driver->table = kzalloc(diag_max_reg*
		      sizeof(struct diag_master_table),
		       GFP_KERNEL)) == NULL)
		goto err;
	kmemleak_not_leak(driver->table);

	if (driver->usb_read_ptr == NULL) {
		driver->usb_read_ptr = kzalloc(
			sizeof(struct diag_request), GFP_KERNEL);
		if (driver->usb_read_ptr == NULL)
			goto err;
		kmemleak_not_leak(driver->usb_read_ptr);
	}
	if (driver->pkt_buf == NULL &&
	     (driver->pkt_buf = kzalloc(PKT_SIZE,
			 GFP_KERNEL)) == NULL)
		goto err;
	kmemleak_not_leak(driver->pkt_buf);
	if (driver->dci_pkt_buf == NULL) {
		driver->dci_pkt_buf = kzalloc(PKT_SIZE, GFP_KERNEL);
		if (!driver->dci_pkt_buf)
			goto err;
	}
	kmemleak_not_leak(driver->dci_pkt_buf);
	if (driver->apps_rsp_buf == NULL) {
		driver->apps_rsp_buf = kzalloc(APPS_BUF_SIZE, GFP_KERNEL);
		if (driver->apps_rsp_buf == NULL)
			goto err;
		kmemleak_not_leak(driver->apps_rsp_buf);
	}
	driver->diag_wq = create_singlethread_workqueue("diag_wq");
	driver->diag_usb_wq = create_singlethread_workqueue("diag_usb_wq");
#ifdef CONFIG_DIAG_OVER_USB
	INIT_WORK(&(driver->diag_usb_connect_work),
						 diag_usb_connect_work_fn);
	INIT_WORK(&(driver->diag_usb_disconnect_work),
						 diag_usb_disconnect_work_fn);
	INIT_WORK(&(driver->diag_proc_hdlc_work), diag_process_hdlc_fn);
	INIT_WORK(&(driver->diag_read_work), diag_read_work_fn);
	driver->legacy_ch = usb_diag_open(DIAG_LEGACY, driver,
			diag_usb_legacy_notifier);
	if (IS_ERR(driver->legacy_ch)) {
		printk(KERN_ERR "Unable to open USB diag legacy channel\n");
		goto err;
	}
#endif
#if DIAG_XPST
	mutex_init(&driver->smd_lock);
#endif
	platform_driver_register(&msm_smd_ch1_driver);
	platform_driver_register(&diag_smd_lite_driver);

	if (driver->supports_separate_cmdrsp) {
		for (i = 0; i < NUM_SMD_CMD_CHANNELS; i++)
			platform_driver_register(&smd_lite_data_cmd_drivers[i]);
	}

	return;
err:
	pr_err("diag: Could not initialize diag buffers");

	for (i = 0; i < NUM_SMD_DATA_CHANNELS; i++)
		diag_smd_destructor(&driver->smd_data[i]);

	for (i = 0; i < NUM_SMD_CMD_CHANNELS; i++)
		diag_smd_destructor(&driver->smd_cmd[i]);

	kfree(driver->buf_msg_mask_update);
	kfree(driver->buf_log_mask_update);
	kfree(driver->buf_event_mask_update);
	kfree(driver->usb_buf_out);
	kfree(driver->hdlc_buf);
	kfree(driver->client_map);
	kfree(driver->buf_tbl);
	kfree(driver->data_ready);
	kfree(driver->table);
	kfree(driver->pkt_buf);
	kfree(driver->dci_pkt_buf);
	kfree(driver->usb_read_ptr);
	kfree(driver->apps_rsp_buf);
	kfree(driver->user_space_data_buf);
	if (driver->diag_wq)
		destroy_workqueue(driver->diag_wq);
	if (driver->diag_usb_wq)
		destroy_workqueue(driver->diag_usb_wq);
}

void diagfwd_exit(void)
{
	int i;

	for (i = 0; i < NUM_SMD_DATA_CHANNELS; i++)
		diag_smd_destructor(&driver->smd_data[i]);

#ifdef CONFIG_DIAG_OVER_USB
	usb_diag_close(driver->legacy_ch);
#endif
	platform_driver_unregister(&msm_smd_ch1_driver);
	platform_driver_unregister(&diag_smd_lite_driver);

	if (driver->supports_separate_cmdrsp) {
		for (i = 0; i < NUM_SMD_CMD_CHANNELS; i++) {
			diag_smd_destructor(&driver->smd_cmd[i]);
			platform_driver_unregister(
				&smd_lite_data_cmd_drivers[i]);
		}
	}

	kfree(driver->buf_msg_mask_update);
	kfree(driver->buf_log_mask_update);
	kfree(driver->buf_event_mask_update);
	kfree(driver->usb_buf_out);
	kfree(driver->hdlc_buf);
	kfree(driver->client_map);
	kfree(driver->buf_tbl);
	kfree(driver->data_ready);
	kfree(driver->table);
	kfree(driver->pkt_buf);
	kfree(driver->dci_pkt_buf);
	kfree(driver->usb_read_ptr);
	kfree(driver->apps_rsp_buf);
	kfree(driver->user_space_data_buf);
	destroy_workqueue(driver->diag_wq);
	destroy_workqueue(driver->diag_usb_wq);
}
