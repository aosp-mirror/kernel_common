/* arch/arm/mach-msm/htc_drm-8x60.c
 *
 * Copyright (C) 2011 HTC Corporation.
 * Author: Eddic Hsien <eddic_hsien@htc.com>
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
#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>

#if !defined(CONFIG_ARCH_MSM7X30) && !defined(CONFIG_ARCH_MSM7X27A)
#include <mach/scm.h>
#else	
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <asm/system.h>
#include <linux/semaphore.h>
#include <mach/msm_rpcrouter.h>
#include <mach/oem_rapi_client.h>

#define OEM_RAPI_PROG  0x3000006B
#define OEM_RAPI_VERS  0x00010001

#define OEM_RAPI_NULL_PROC                        0
#define OEM_RAPI_RPC_GLUE_CODE_INFO_REMOTE_PROC   1
#define OEM_RAPI_STREAMING_FUNCTION_PROC          2

#define OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE 128
#endif	

#define DEVICE_NAME "htcdrm"

#define HTCDRM_IOCTL_WIDEVINE	0x2563
#define HTCDRM_IOCTL_DISCRETIX	0x2596
#define HTCDRM_IOCTL_CPRM   	0x2564
#define HTCDRM_IOCTL_GDRIVE	0x2568

#define DEVICE_ID_LEN			32
#define WIDEVINE_KEYBOX_LEN		128
#define CPRM_KEY_LEN		    188

#define HTC_DRM_DEBUG	0
#define TAG "[HTCDRM] "
#undef PDEBUG
#if HTC_DRM_DEBUG
#define PDEBUG(fmt, args...) printk(KERN_DEBUG TAG "[D] %s(%i, %s): " fmt "\n", \
		__func__, current->pid, current->comm, ## args)
#else
#define PDEBUG(fmt, args...) do {} while (0)
#endif

#undef PERR
#define PERR(fmt, args...) printk(KERN_ERR TAG "[K] %s(%i, %s): " fmt "\n", \
		__func__, current->pid, current->comm, ## args)

#undef PINFO
#define PINFO(fmt, args...) printk(KERN_INFO TAG "[I] %s(%i, %s): " fmt "\n", \
		__func__, current->pid, current->comm, ## args)

#if !defined(CONFIG_ARCH_MSM7X30) && !defined(CONFIG_ARCH_MSM7X27A)
#define UP(S)
#else
#define UP(S) up(S)
#endif
static int htcdrm_major;
static struct class *htcdrm_class;
static const struct file_operations htcdrm_fops;

typedef struct _htc_drm_msg_s {
	int func;
	int offset;
	unsigned char *req_buf;
	int req_len;
	unsigned char *resp_buf;
	int resp_len;
} htc_drm_msg_s;

enum {
		HTC_OEMCRYPTO_STORE_KEYBOX = 1,
		HTC_OEMCRYPTO_GET_KEYBOX,
		HTC_OEMCRYPTO_IDENTIFY_DEVICE,
		HTC_OEMCRYPTO_GET_RANDOM,
		HTC_OEMCRYPTO_IS_KEYBOX_VALID,
};

enum {
	HTC_GDRIVE_GET_VOUCHER = 1,
	HTC_GDRIVE_CREATE_VOUCHER_SIGNATURE,
	HTC_GDRIVE_REDEEM_UPDATE,
};

#if !defined(CONFIG_ARCH_MSM7X30) && !defined(CONFIG_ARCH_MSM7X27A)
static unsigned char *htc_device_id;
static unsigned char *htc_keybox;

typedef struct _htc_drm_dix_msg_s {
	int func;
	int ret;
	unsigned char *buf;
	int buf_len;
	int flags;
} htc_drm_dix_msg_s;

typedef union {
	struct {
		void *buffer;
		int size;
	} memref;
	struct {
		unsigned int a;
		unsigned int b;
	} value;
} TEE_Param;

#define	TEE_PARAM_TYPE_NONE 			0
#define	TEE_PARAM_TYPE_VALUE_INPUT		1
#define	TEE_PARAM_TYPE_VALUE_OUTPUT		2
#define	TEE_PARAM_TYPE_VALUE_INOUT		3
#define	TEE_PARAM_TYPE_MEMREF_INPUT		5
#define	TEE_PARAM_TYPE_MEMREF_OUTPUT	6
#define	TEE_PARAM_TYPE_MEMREF_INOUT		7
#define TEE_PARAM_TYPE_GET(t, i) (((t) >> (i*4)) & 0xF)

enum {
    TEE_FUNC_TA_OpenSession = 1,
    TEE_FUNC_TA_CloseSession,
    TEE_FUNC_TA_InvokeCommand,
	TEE_FUNC_TA_CreateEntryPoint,
	TEE_FUNC_TA_DestroyEntryPoint,
	TEE_FUNC_SECURE_STORAGE_INIT,
	TEE_FUNC_SECURE_STORAGE_SYNC,
	TEE_FUNC_HTC_HEAP_INIT,
	TEE_FUNC_HTC_SMEM_INIT,
};

struct CMD_TA_OpenSession {
	unsigned int paramTypes;
	TEE_Param params[4];
	void **sessionContext;
};

struct CMD_TA_CloseSession {
    void *sessionContext;
};

struct CMD_TA_InvokeCommand {
	void *sessionContext;
	unsigned int commandID;
	unsigned int paramTypes;
	TEE_Param params[4];
};

struct CMD_SECURE_STORAGE {
	unsigned char *image_base;
	unsigned int image_size;
};

#define DX_ALLOC_TZ_HEAP 0

#if DX_ALLOC_TZ_HEAP
#define DISCRETIX_HEAP_SIZE	(1024 * 1024)
static int secure_storage_init;
static unsigned char *discretix_tz_heap;
#else
#define DISCRETIX_HEAP_SIZE	0
#endif

#define DX_PRE_ALLOC_BUFFER 0

#endif

#if defined(CONFIG_ARCH_MSM7X30) || defined(CONFIG_ARCH_MSM7X27A)
struct htc_keybox_dev {
	struct platform_device *pdev;
	struct cdev cdev;
	struct device *device;
	struct class *class;
	dev_t dev_num;
	unsigned char keybox_buf[OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE];
	struct semaphore sem;
} htc_keybox_dev;

static struct msm_rpc_client *rpc_client;
static uint32_t open_count;
static DEFINE_MUTEX(oem_rapi_client_lock);
static int oem_rapi_client_cb(struct msm_rpc_client *client,
			      struct rpc_request_hdr *req,
			      struct msm_rpc_xdr *xdr)
{
	uint32_t cb_id, accept_status;
	int rc;
	void *cb_func;
	uint32_t temp;

	struct oem_rapi_client_streaming_func_cb_arg arg;
	struct oem_rapi_client_streaming_func_cb_ret ret;

	arg.input = NULL;
	ret.out_len = NULL;
	ret.output = NULL;

	xdr_recv_uint32(xdr, &cb_id);                    
	xdr_recv_uint32(xdr, &arg.event);                
	xdr_recv_uint32(xdr, (uint32_t *)(&arg.handle)); 
	xdr_recv_uint32(xdr, &arg.in_len);               
	xdr_recv_bytes(xdr, (void **)&arg.input, &temp); 
	xdr_recv_uint32(xdr, &arg.out_len_valid);        
	if (arg.out_len_valid) {
		ret.out_len = kmalloc(sizeof(*ret.out_len), GFP_KERNEL);
		if (!ret.out_len) {
			accept_status = RPC_ACCEPTSTAT_SYSTEM_ERR;
			goto oem_rapi_send_ack;
		}
	}

	xdr_recv_uint32(xdr, &arg.output_valid);         
	if (arg.output_valid) {
		xdr_recv_uint32(xdr, &arg.output_size);  

		ret.output = kmalloc(arg.output_size, GFP_KERNEL);
		if (!ret.output) {
			accept_status = RPC_ACCEPTSTAT_SYSTEM_ERR;
			goto oem_rapi_send_ack;
		}
	}

	cb_func = msm_rpc_get_cb_func(client, cb_id);
	if (cb_func) {
		rc = ((int (*)(struct oem_rapi_client_streaming_func_cb_arg *,
			       struct oem_rapi_client_streaming_func_cb_ret *))
		      cb_func)(&arg, &ret);
		if (rc)
			accept_status = RPC_ACCEPTSTAT_SYSTEM_ERR;
		else
			accept_status = RPC_ACCEPTSTAT_SUCCESS;
	} else
		accept_status = RPC_ACCEPTSTAT_SYSTEM_ERR;

 oem_rapi_send_ack:
	xdr_start_accepted_reply(xdr, accept_status);

	if (accept_status == RPC_ACCEPTSTAT_SUCCESS) {
		uint32_t temp = sizeof(uint32_t);
		xdr_send_pointer(xdr, (void **)&(ret.out_len), temp,
				 xdr_send_uint32);

		
		if (ret.output && ret.out_len)
			xdr_send_bytes(xdr, (const void **)&ret.output,
					     ret.out_len);
		else {
			temp = 0;
			xdr_send_uint32(xdr, &temp);
		}
	}
	rc = xdr_send_msg(xdr);
	if (rc)
		PERR("sending reply failed: %d", rc);

	kfree(arg.input);
	kfree(ret.out_len);
	kfree(ret.output);

	return 0;
}

static int oem_rapi_client_streaming_function_arg(struct msm_rpc_client *client,
						  struct msm_rpc_xdr *xdr,
						  void *data)
{
	int cb_id;
	struct oem_rapi_client_streaming_func_arg *arg = data;

	cb_id = msm_rpc_add_cb_func(client, (void *)arg->cb_func);
	if ((cb_id < 0) && (cb_id != MSM_RPC_CLIENT_NULL_CB_ID))
		return cb_id;

	xdr_send_uint32(xdr, &arg->event);                
	xdr_send_uint32(xdr, &cb_id);                     
	xdr_send_uint32(xdr, (uint32_t *)(&arg->handle)); 
	xdr_send_uint32(xdr, &arg->in_len);               
	xdr_send_bytes(xdr, (const void **)&arg->input,
			     &arg->in_len);                     
	xdr_send_uint32(xdr, &arg->out_len_valid);        
	xdr_send_uint32(xdr, &arg->output_valid);         

	
	if (arg->output_valid)
		xdr_send_uint32(xdr, &arg->output_size);

	return 0;
}

static int oem_rapi_client_streaming_function_ret(struct msm_rpc_client *client,
						  struct msm_rpc_xdr *xdr,
						  void *data)
{
	struct oem_rapi_client_streaming_func_ret *ret = data;
	uint32_t temp;

	
	xdr_recv_pointer(xdr, (void **)&(ret->out_len), sizeof(uint32_t),
			 xdr_recv_uint32);

	
	if (ret->out_len && *ret->out_len)
		xdr_recv_bytes(xdr, (void **)&ret->output, &temp);

	return 0;
}

int oem_rapi_client_streaming_function(
	struct msm_rpc_client *client,
	struct oem_rapi_client_streaming_func_arg *arg,
	struct oem_rapi_client_streaming_func_ret *ret)
{
	return msm_rpc_client_req2(client,
				   OEM_RAPI_STREAMING_FUNCTION_PROC,
				   oem_rapi_client_streaming_function_arg, arg,
				   oem_rapi_client_streaming_function_ret,
				   ret, -1);
}
EXPORT_SYMBOL(oem_rapi_client_streaming_function);

int oem_rapi_client_close(void)
{
	mutex_lock(&oem_rapi_client_lock);
	if (open_count > 0) {
		if (--open_count == 0) {
			msm_rpc_unregister_client(rpc_client);
			PINFO("Disconnected from remote oem rapi server");
		}
	}
	mutex_unlock(&oem_rapi_client_lock);
	return 0;
}
EXPORT_SYMBOL(oem_rapi_client_close);

struct msm_rpc_client *oem_rapi_client_init(void)
{
	mutex_lock(&oem_rapi_client_lock);
	if (open_count == 0) {
		rpc_client = msm_rpc_register_client2("oemrapiclient",
						      OEM_RAPI_PROG,
						      OEM_RAPI_VERS, 0,
						      oem_rapi_client_cb);
		if (!IS_ERR(rpc_client))
			open_count++;
	} else {
		
		open_count++;
	}
	mutex_unlock(&oem_rapi_client_lock);
	return rpc_client;
}
EXPORT_SYMBOL(oem_rapi_client_init);

static ssize_t htc_keybox_read(struct htc_keybox_dev *dev, char *buf, size_t size, loff_t p)
{
	unsigned int count = size;
	int ret_rpc = 0;
	struct oem_rapi_client_streaming_func_arg arg;
	struct oem_rapi_client_streaming_func_ret ret;
	unsigned char nullbuf[OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE];

	memset(dev->keybox_buf, 56, OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE);
	memset(nullbuf, 0, OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE);

	PDEBUG("start:");
	if (p >= OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE)
		return count ? -ENXIO : 0;

	PDEBUG("oem_rapi_client_streaming_function start:");
	if (count == 0xFF) {
		arg.event = OEM_RAPI_CLIENT_EVENT_WIDEVINE_READ_DEVICE_ID;
		memset(dev->keybox_buf, 57, OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE);
		PINFO("Get device id via oem_rapi");
	} else if (count > OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE - p) {
		count = OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE - p;
		arg.event = OEM_RAPI_CLIENT_EVENT_WIDEVINE_READ_KEYBOX;
		PINFO("Get keybox via oem_rapi");
	} else {
		arg.event = OEM_RAPI_CLIENT_EVENT_WIDEVINE_READ_KEYBOX;
		PINFO("Get keybox via oem_rapi");
	}
	arg.cb_func = NULL;
	arg.handle = (void *)0;
	arg.in_len = 0;
	arg.input = (char *)nullbuf;
	arg.out_len_valid = 1;
	arg.output_valid = 1;
	arg.output_size = OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE;
	ret.out_len = NULL;
	ret.output = NULL;

	ret_rpc = oem_rapi_client_streaming_function(rpc_client, &arg, &ret);
	if (ret_rpc) {
		PERR("Get data from modem failed: %d", ret_rpc);
		return -EFAULT;
	}
	PDEBUG("Data obtained from modem %d, ", *(ret.out_len));
	memcpy(dev->keybox_buf, ret.output, *(ret.out_len));
	kfree(ret.out_len);
	kfree(ret.output);

	return 0;
}

static ssize_t htc_keybox_write(struct htc_keybox_dev *dev, const char *buf, size_t size, loff_t p)
{
	unsigned int count = size;
	int ret_rpc = 0;
	struct oem_rapi_client_streaming_func_arg arg;
	struct oem_rapi_client_streaming_func_ret ret;

	PDEBUG("start:");
	if (p >= OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE)
		return count ? -ENXIO : 0;
	if (count > OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE - p)
		count = OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE - p;
	PDEBUG("oem_rapi_client_streaming_function start:");

	arg.event = OEM_RAPI_CLIENT_EVENT_WIDEVINE_WRITE_KEYBOX;
	arg.cb_func = NULL;
	arg.handle = (void *)0;
	arg.in_len = OEM_RAPI_CLIENT_MAX_OUT_BUFF_SIZE;
	arg.input = (char *)dev->keybox_buf;
	arg.out_len_valid = 0;
	arg.output_valid = 0;
	arg.output_size = 0;
	ret.out_len = NULL;
	ret.output = NULL;

	ret_rpc = oem_rapi_client_streaming_function(rpc_client, &arg, &ret);
	if (ret_rpc) {
		PERR("Send data from modem failed: %d", ret_rpc);
		return -EFAULT;
	}
	PDEBUG("Data sent to modem %s", dev->keybox_buf);

	return 0;
}
static struct htc_keybox_dev *keybox_dev;
#endif 

static unsigned char *htc_device_id;
static unsigned char *htc_keybox;

#if !defined(CONFIG_ARCH_MSM7X30) && !defined(CONFIG_ARCH_MSM7X27A)

#define DX_SMEM_SIZE	PAGE_SIZE
static int discretix_smem_size = DX_SMEM_SIZE;
static unsigned char *discretix_smem_ptr;
static unsigned char *discretix_smem_phy;
static unsigned char *discretix_smem_area;

static DEFINE_MUTEX(dx_lock);

#if (DX_PRE_ALLOC_BUFFER)

#define DX_PRE_ALLOC_BUFFER_SIZE ((512 + 64) * 1024)

static unsigned char *dx_memory_pool_ptr;
static unsigned char *dx_memory_pool_phy;
static unsigned int dx_mem_offset;
static unsigned int max_ofs;

static void reset_dx_memory_pool(void)
{
	if (max_ofs < dx_mem_offset)
		max_ofs = dx_mem_offset;
	dx_mem_offset = 0;
}

static unsigned char *dx_kzalloc(int size)
{
	unsigned char *m = NULL;
	unsigned int mask;

	if (dx_memory_pool_ptr == NULL)
		return NULL;

	if (dx_mem_offset + size <= DX_PRE_ALLOC_BUFFER_SIZE) {
		m = dx_memory_pool_ptr + dx_mem_offset;
		memset(m, 0, size);
		dx_mem_offset += size;
		mask = 4096 - 1;
		dx_mem_offset = (dx_mem_offset + mask) & ~mask;
	}
	return m;
}

static unsigned char *dx_virt_to_phys(unsigned char *virt)
{
	return (dx_memory_pool_phy + (virt - dx_memory_pool_ptr));
}
#endif


#if 0
static long htcdrm_discretix_cmd(unsigned int command, unsigned long arg)
{
	htc_drm_dix_msg_s hdix;
	int ret = 0, i;
	unsigned char *ptr, *data, *image, *image_u;
	unsigned char *kbuf[4], *ubuf[4];
	#if defined(DX_PRE_ALLOC_BUFFER)
	unsigned int *sessionContext;
	#else
	unsigned int sessionContext;
	#endif
	unsigned long start, end;

	image_u = NULL;
	image = NULL;

	#if defined(DX_PRE_ALLOC_BUFFER)
	reset_dx_memory_pool();
	sessionContext = (unsigned int *)dx_kzalloc(sizeof(int));
	if (!sessionContext)
		return -EFAULT;
	#endif

	if (copy_from_user(&hdix, (void __user *)arg, sizeof(hdix))) {
		PERR("copy_from_user error (msg)");
		return -EFAULT;
	}

	PDEBUG("htcdrm_discretix_ioctl func: %x", hdix.func);

	
	switch (hdix.func) {
	case TEE_FUNC_TA_OpenSession:
		if (hdix.buf_len != sizeof(struct CMD_TA_OpenSession)) {
			PERR("OpenSession size error");
			return -EFAULT;
		}
		break;
	case TEE_FUNC_TA_InvokeCommand:
		if (hdix.buf_len != sizeof(struct CMD_TA_InvokeCommand)) {
			PERR("InvokeCommand size error");
			return -EFAULT;
		}
		break;
	case TEE_FUNC_TA_CloseSession:
		if (hdix.buf_len != sizeof(struct CMD_TA_CloseSession)) {
			PERR("CloseSession size error");
			return -EFAULT;
		}
		break;
	case TEE_FUNC_TA_CreateEntryPoint:
		if (hdix.buf_len != 0) {
			PERR("CreateEntryPoint size error");
			return -EFAULT;
		}
		break;
	case TEE_FUNC_TA_DestroyEntryPoint:
		if (hdix.buf_len != 0) {
			PERR("DestroyEntryPoint size error");
			return -EFAULT;
		}
		break;
	case TEE_FUNC_SECURE_STORAGE_INIT:
		if (secure_storage_init)
			return 0;

	case TEE_FUNC_HTC_HEAP_INIT:
	case TEE_FUNC_HTC_SMEM_INIT:
	case TEE_FUNC_SECURE_STORAGE_SYNC:
		if (hdix.buf_len != sizeof(struct CMD_SECURE_STORAGE)) {
			PERR("SECURE_STORAGE size error");
			return -EFAULT;
		}
		break;
	default:
		PERR("func: %d error", hdix.func);
		return -EFAULT;
	}

	if (hdix.buf_len != 0) {
		#if defined(DX_PRE_ALLOC_BUFFER)
		ptr = dx_kzalloc(hdix.buf_len);
		#else
		ptr = kzalloc(hdix.buf_len, GFP_KERNEL);
		#endif
		if (ptr == NULL) {
			PERR("allocate the space for data failed (%d)", hdix.buf_len);
			return -EFAULT;
		}
		if (copy_from_user(ptr, (void __user *)hdix.buf, hdix.buf_len)) {
			PERR("copy_from_user error (data)");
			#if !defined(DX_PRE_ALLOC_BUFFER)
			kfree(ptr);
			#endif
			return -EFAULT;
		}

		data = hdix.buf;
		#if defined(DX_PRE_ALLOC_BUFFER)
		hdix.buf = (unsigned char *)dx_virt_to_phys(ptr);
		#else
		hdix.buf = (unsigned char *)virt_to_phys(ptr);
		#endif
	} else {
		data = NULL;
		ptr = NULL;
	}
	for (i = 0; i < 4; i++)
		kbuf[i] = NULL;

	PDEBUG("func = %x", hdix.func);
	switch (hdix.func) {
	case TEE_FUNC_HTC_HEAP_INIT:
		{
			struct CMD_SECURE_STORAGE *s;

			s = (struct CMD_SECURE_STORAGE *)ptr;
			s->image_base = (unsigned char *)virt_to_phys(discretix_tz_heap);
			s->image_size = DISCRETIX_HEAP_SIZE;
		}
		break;
	case TEE_FUNC_HTC_SMEM_INIT:
		{
			struct CMD_SECURE_STORAGE *s;

			s = (struct CMD_SECURE_STORAGE *)ptr;
			s->image_base = (unsigned char *)virt_to_phys(discretix_smem_area);
			s->image_size = discretix_smem_size;
		}
		break;
	case TEE_FUNC_SECURE_STORAGE_INIT:
		{
			struct CMD_SECURE_STORAGE *s;

			s = (struct CMD_SECURE_STORAGE *)ptr;
			#if defined(DX_PRE_ALLOC_BUFFER)
			image = dx_kzalloc(s->image_size);
			#else
			image = kzalloc(s->image_size, GFP_KERNEL);
			#endif
			if (image == NULL) {
				PERR("allocate the space for fat8 image failed");
				return -1;
			}
			if (copy_from_user(image, (void __user *)s->image_base, s->image_size)) {
				PERR("copy_from_user error (image)");
				#if !defined(DX_PRE_ALLOC_BUFFER)
				kfree(image);
				#endif
				return -EFAULT;
			}
			#if defined(DX_PRE_ALLOC_BUFFER)
			s->image_base = (unsigned char *)dx_virt_to_phys(image);
			#else
			s->image_base = (unsigned char *)virt_to_phys(image);
			#endif
		}
		break;
	case TEE_FUNC_SECURE_STORAGE_SYNC:
		{
			struct CMD_SECURE_STORAGE *s;

			s = (struct CMD_SECURE_STORAGE *)ptr;
			#if defined(DX_PRE_ALLOC_BUFFER)
			image = dx_kzalloc(s->image_size);
			#else
			image = kzalloc(s->image_size, GFP_KERNEL);
			#endif
			if (image == NULL) {
				PERR("allocate the space for fat8 image failed");
				return -1;
			}
			image_u = s->image_base;
			#if defined(DX_PRE_ALLOC_BUFFER)
			s->image_base = (unsigned char *)dx_virt_to_phys(image);
			#else
			s->image_base = (unsigned char *)virt_to_phys(image);
			#endif
		}
		break;
	case TEE_FUNC_TA_OpenSession:
		{
			struct CMD_TA_OpenSession *s;

			s = (struct CMD_TA_OpenSession *)ptr;
			#if defined(DX_PRE_ALLOC_BUFFER)
			*sessionContext = 0;
			s->sessionContext = (void *)dx_virt_to_phys((char *)sessionContext);
			#else
			sessionContext = 0;
			s->sessionContext = (void *)virt_to_phys(&sessionContext);
			#endif
			for (i = 0; i < 4; i++) {
				int ptype;

				ubuf[i] = s->params[i].memref.buffer;
				ptype = TEE_PARAM_TYPE_GET(s->paramTypes, i);
				switch (ptype) {
				case TEE_PARAM_TYPE_MEMREF_INPUT:
				case TEE_PARAM_TYPE_MEMREF_OUTPUT:
				case TEE_PARAM_TYPE_MEMREF_INOUT:
					if (s->params[i].memref.size != 0) {
						#if defined(DX_PRE_ALLOC_BUFFER)
						kbuf[i] = dx_kzalloc(s->params[i].memref.size);
						#else
						kbuf[i] = kzalloc(s->params[i].memref.size, GFP_KERNEL);
						#endif
						if (kbuf[i] == NULL) {
							PERR("allocate the space for buffer failed (%d)", s->params[i].memref.size);
							ret = -EFAULT;
							goto discretix_error_exit;
						}
						#if defined(DX_PRE_ALLOC_BUFFER)
						s->params[i].memref.buffer = (unsigned char *)dx_virt_to_phys(kbuf[i]);
						#else
						s->params[i].memref.buffer = (unsigned char *)virt_to_phys(kbuf[i]);
						#endif
					} else
						kbuf[i] = NULL;
					if ((ptype == TEE_PARAM_TYPE_MEMREF_INPUT) ||
						(ptype == TEE_PARAM_TYPE_MEMREF_INOUT)) {
						if (copy_from_user(kbuf[i], (void __user *)ubuf[i], s->params[i].memref.size)) {
							PERR("copy_from_user error (buffer)");
							ret = -EFAULT;
							goto discretix_error_exit;
						}
					}
					break;
				default:
					break;
				}
			}
		}
		break;
	case TEE_FUNC_TA_InvokeCommand:
		{
			struct CMD_TA_InvokeCommand *s;

			s = (struct CMD_TA_InvokeCommand *)ptr;
			
			if (!(s->sessionContext)) {
				PERR("session context is null");
				ret = -EFAULT;
				goto discretix_error_exit;
			}

			for (i = 0; i < 4; i++) {
				int ptype;

				ubuf[i] = s->params[i].memref.buffer;
				ptype = TEE_PARAM_TYPE_GET(s->paramTypes, i);
				switch (ptype) {
				case TEE_PARAM_TYPE_MEMREF_INPUT:
				case TEE_PARAM_TYPE_MEMREF_OUTPUT:
				case TEE_PARAM_TYPE_MEMREF_INOUT:
					if (s->params[i].memref.size != 0) {
						#if defined(DX_PRE_ALLOC_BUFFER)
						kbuf[i] = dx_kzalloc(s->params[i].memref.size);
						#else
						kbuf[i] = kzalloc(s->params[i].memref.size, GFP_KERNEL);
						#endif
						if (kbuf[i] == NULL) {
							PERR("allocate the space for buffer failed (%d)", s->params[i].memref.size);
							ret = -EFAULT;
							goto discretix_error_exit;
						}
						#if defined(DX_PRE_ALLOC_BUFFER)
						s->params[i].memref.buffer = (unsigned char *)dx_virt_to_phys(kbuf[i]);
						#else
						s->params[i].memref.buffer = (unsigned char *)virt_to_phys(kbuf[i]);
						#endif
					} else
						kbuf[i] = NULL;
					if ((ptype == TEE_PARAM_TYPE_MEMREF_INPUT) ||
						(ptype == TEE_PARAM_TYPE_MEMREF_INOUT)) {
						if (copy_from_user(kbuf[i], (void __user *)ubuf[i], s->params[i].memref.size)) {
							PERR("copy_from_user error (buffer)");
							ret = -EFAULT;
							goto discretix_error_exit;
						}
					}
					break;
				default:
					break;
				}
			}
		}
		break;
	default:
		break;
	}
    PDEBUG("##### TZ DX");
	ret = secure_3rd_party_syscall(0, (unsigned char *)&hdix, sizeof(hdix));

	
	if (ptr) {
		start = (unsigned long)ptr;
		end = start + hdix.buf_len;
		scm_flush_range(start, end);
	}

	PDEBUG("##### TZ DX ret=%d(%x)", ret, ret);
	if (ret) {
		PERR("3rd party syscall failed (%d)", ret);
		goto discretix_error_exit;
	}

	if (hdix.ret) {
		PERR("[DX] func:%x ret:%x", hdix.func, hdix.ret);
	}

	switch (hdix.func) {
	case TEE_FUNC_TA_OpenSession:
		{
			struct CMD_TA_OpenSession *s;

			s = (struct CMD_TA_OpenSession *)ptr;
			
			#if defined(DX_PRE_ALLOC_BUFFER)
			start = (unsigned long)sessionContext;
			#else
			start = (unsigned long)&sessionContext;
			#endif
			end = start + 4;
			scm_flush_range(start, end);
			#if defined(DX_PRE_ALLOC_BUFFER)
			s->sessionContext = (void *)(*sessionContext);
			#else
			s->sessionContext = (void *)sessionContext;
			#endif
			for (i = 0; i < 4; i++) {
				int ptype;

				ptype = TEE_PARAM_TYPE_GET(s->paramTypes, i);
				switch (ptype) {
				case TEE_PARAM_TYPE_MEMREF_OUTPUT:
				case TEE_PARAM_TYPE_MEMREF_INOUT:
					if (s->params[i].memref.size != 0) {
						
						start = (unsigned long)kbuf[i];
						end = start + s->params[i].memref.size;
						scm_flush_range(start, end);
						if (copy_to_user((void __user *)ubuf[i], kbuf[i], s->params[i].memref.size)) {
							PERR("copy_to_user error (buffer)");
							ret = -EFAULT;
							goto discretix_error_exit;
						}
					}
					break;
				default:
					break;
				}
			}
		}
		break;
	case TEE_FUNC_TA_InvokeCommand:
		{
			struct CMD_TA_InvokeCommand *s;

			s = (struct CMD_TA_InvokeCommand *)ptr;
			for (i = 0; i < 4; i++) {
				int ptype;

				ptype = TEE_PARAM_TYPE_GET(s->paramTypes, i);
				switch (ptype) {
				case TEE_PARAM_TYPE_MEMREF_OUTPUT:
				case TEE_PARAM_TYPE_MEMREF_INOUT:
					if (s->params[i].memref.size != 0) {
						
						start = (unsigned long)kbuf[i];
						end = start + s->params[i].memref.size;
						scm_flush_range(start, end);
						if (copy_to_user((void __user *)ubuf[i], kbuf[i], s->params[i].memref.size)) {
							PERR("copy_to_user error (buffer)");
							ret = -EFAULT;
							goto discretix_error_exit;
						}
					}
					break;
				default:
					break;
				}
			}
		}
		break;
	case TEE_FUNC_SECURE_STORAGE_INIT:
		{
			struct CMD_SECURE_STORAGE *s;

			s = (struct CMD_SECURE_STORAGE *)ptr;
			s->image_base = image_u;
			#if !defined(DX_PRE_ALLOC_BUFFER)
			kfree(image);
			#endif
			secure_storage_init = 1;
		}
		break;
	case TEE_FUNC_SECURE_STORAGE_SYNC:
		{
			struct CMD_SECURE_STORAGE *s;

			s = (struct CMD_SECURE_STORAGE *)ptr;
			s->image_base = image_u;
			
			start = (unsigned long)image;
			end = start + s->image_size;
			scm_flush_range(start, end);
			if (copy_to_user((void __user *)image_u, image, s->image_size))
				 PERR("copy_to_user error (image)");
			#if !defined(DX_PRE_ALLOC_BUFFER)
			kfree(image);
			#endif
			PDEBUG("sync htc ssd");
		}
		break;
	default:
		break;
	}

	if (hdix.buf_len != 0) {
		if (copy_to_user((void __user *)data, ptr, hdix.buf_len)) {
			PERR("copy_to_user error (data)");
			ret = -EFAULT;
			goto discretix_error_exit;
		}
	}
	if (copy_to_user((void __user *)arg, &hdix, sizeof(hdix))) {
		PERR("copy_to_user error (msg)");
		ret = -EFAULT;
		goto discretix_error_exit;
	}

discretix_error_exit:
	#if !defined(DX_PRE_ALLOC_BUFFER)
	for (i = 0; i < 4; i++)
		if (kbuf[i] != NULL)
			kfree(kbuf[i]);
	kfree(ptr);
	#endif

	return ret;
}
#endif

#if DX_ALLOC_TZ_HEAP
static int htcdrm_discretix_init_heap(void)
{
	htc_drm_dix_msg_s hdix;
	struct CMD_SECURE_STORAGE *s;
	int ret;
	static int init_flag;

	if (init_flag)
		return 0;
	if (discretix_tz_heap == NULL) {
		PERR("discretix_tz_heap null");
		return 0;
	}

	s = (struct CMD_SECURE_STORAGE *)kzalloc(sizeof(struct CMD_SECURE_STORAGE), GFP_KERNEL);
	s->image_base = (unsigned char *)virt_to_phys(discretix_tz_heap);
	s->image_size = DISCRETIX_HEAP_SIZE;

	memset((char *)&hdix, 0, sizeof(hdix));
	hdix.func = TEE_FUNC_HTC_HEAP_INIT;
	hdix.buf = (unsigned char *)virt_to_phys(s);
	hdix.buf_len = sizeof(struct CMD_SECURE_STORAGE);
	ret = secure_3rd_party_syscall(0, (unsigned char *)&hdix, sizeof(hdix));
	if (ret)
		PERR("3rd party syscall failed (%d)", ret);
	kfree(s);
	init_flag = 1;
	return ret;
}
#endif

static long htcdrm_discretix_ioctl(struct file *file, unsigned int command, unsigned long arg)
{
#if DX_ALLOC_TZ_HEAP
	htc_drm_dix_msg_s hdix;

	if (copy_from_user(&hdix, (void __user *)arg, sizeof(hdix))) {
		PERR("copy_from_user error (msg)");
		return -EFAULT;
	}

	if (hdix.func == TEE_FUNC_SECURE_STORAGE_INIT) {
		htcdrm_discretix_init_heap();
	}
	return htcdrm_discretix_cmd(command, arg);
#endif
	return 0;
}
#endif

static long htcdrm_gdrive_ioctl(struct file *file, unsigned int command, unsigned long arg) {

	htc_drm_msg_s hmsg;
	unsigned char *in_buf = NULL, *out_buf = NULL;
	int ret = 0;
	int ii;
	unsigned long start, end;

	PDEBUG("%s entry(%d)", __func__, __LINE__);
	if (copy_from_user(&hmsg, (void __user *)arg, sizeof(hmsg))) {
		PERR("copy_from_user error (msg)");
		return -EFAULT;
	}
	switch(hmsg.func) {
		case HTC_GDRIVE_GET_VOUCHER:
			if ((hmsg.resp_buf == NULL) || !hmsg.resp_len) {
				PERR("invalid arguments");
				return -EFAULT;
			}
			out_buf = kmalloc(hmsg.resp_len, GFP_KERNEL);
			if (!out_buf) {
				PERR("Alloc memory fail");
				return -ENOMEM;
			}
			ret = secure_access_item(0, ITEM_GDRIVE_DATA, hmsg.resp_len, out_buf);
			if (ret)
				PERR("get GDrive voucher failed (%d)", ret);
			else {
				start = (unsigned long)out_buf;
				end = start + hmsg.resp_len;
				scm_inv_range(start, end);
				
				if (copy_to_user((void __user *)hmsg.resp_buf, out_buf, *(unsigned int *)(out_buf) + 4)) {
					PERR("copy_to_user error (gdrive voucher)");
					ret = -EFAULT;
				}
			}
			kfree(out_buf);
			break;
		case HTC_GDRIVE_CREATE_VOUCHER_SIGNATURE:
			PDEBUG("%s entry(%d)", __func__, __LINE__);
			if ((hmsg.resp_buf == NULL) || !hmsg.resp_len) {
				PERR("invalid arguments");
				return -EFAULT;
			}
			if ((in_buf = kmalloc(hmsg.req_len, GFP_KERNEL)) == NULL) {
				PERR("Alloc memory fail");
				return -ENOMEM;
			}
			if ((out_buf = kmalloc(hmsg.resp_len, GFP_KERNEL)) == NULL) {
				PERR("Alloc memory fail");
				kfree(in_buf);
				return -ENOMEM;
			}
			if (copy_from_user(in_buf, (void __user *)hmsg.req_buf, hmsg.req_len)) {
				PERR("copy_from_user error (msg)");
				kfree(in_buf);
				kfree(out_buf);
				return -EFAULT;
			}
			for (ii = 0; ii < hmsg.req_len; ii++) {
				PDEBUG("%02x", in_buf[ii]);
			}
			ret = secure_access_item(1, ITEM_VOUCHER_SIG_DATA, hmsg.req_len, in_buf);
			if (ret)
				PERR("put GDrive data fail (%d)", ret);
			else {
				PDEBUG("Read data from TZ start\n");
				ret = secure_access_item(0, ITEM_VOUCHER_SIG_DATA, hmsg.resp_len, out_buf);
				if (ret)
					PERR("get voucher signature fail (%d)", ret);
				else {
					start = (unsigned long)out_buf;
					end = start + hmsg.resp_len;
					scm_inv_range(start, end);
					if (copy_to_user((void __user *)hmsg.resp_buf, out_buf, hmsg.resp_len)) {
						PERR("copy_to_user error (gdrive voucher)");
						ret = -EFAULT;
					}
				}
			}
			kfree(in_buf);
			kfree(out_buf);
			break;
		case HTC_GDRIVE_REDEEM_UPDATE:
			break;
		default:
			PERR("command error");
			return -EFAULT;
	}
	return ret;
}

static long htcdrm_ioctl(struct file *file, unsigned int command, unsigned long arg)
{
	htc_drm_msg_s hmsg;
	int ret = 0;
	unsigned char *ptr;
	static unsigned char htc_cprmkey[CPRM_KEY_LEN]={0};

	PDEBUG("command = %x", command);
	switch (command) {
	case HTCDRM_IOCTL_WIDEVINE:
		if (copy_from_user(&hmsg, (void __user *)arg, sizeof(hmsg))) {
			PERR("copy_from_user error (msg)");
			return -EFAULT;
		}
#if defined(CONFIG_ARCH_MSM7X30) || defined(CONFIG_ARCH_MSM7X27A)
		oem_rapi_client_init();
		if (down_interruptible(&keybox_dev->sem)) {
			PERR("interrupt error");
			return -EFAULT;
		}
#endif 
		PDEBUG("func = %x", hmsg.func);
		switch (hmsg.func) {
		case HTC_OEMCRYPTO_STORE_KEYBOX:
			if ((hmsg.req_buf == NULL) || (hmsg.req_len != WIDEVINE_KEYBOX_LEN)) {
				PERR("invalid arguments");
				UP(&keybox_dev->sem);
				return -EFAULT;
			}
			if (copy_from_user(htc_keybox, (void __user *)hmsg.req_buf, hmsg.req_len)) {
				PERR("copy_from_user error (keybox)");
				UP(&keybox_dev->sem);
				return -EFAULT;
			}
#if defined(CONFIG_ARCH_MSM7X30) || defined(CONFIG_ARCH_MSM7X27A)
			memcpy(keybox_dev->keybox_buf, htc_keybox, WIDEVINE_KEYBOX_LEN);
			ret = htc_keybox_write(keybox_dev, htc_keybox, hmsg.req_len, hmsg.offset);
			oem_rapi_client_close();
#else
			ret = secure_access_item(1, ITEM_KEYBOX_PROVISION, hmsg.req_len,
					htc_keybox);
#endif	
			if (ret)
				PERR("provision keybox failed (%d)", ret);
			UP(&keybox_dev->sem);
			break;
		case HTC_OEMCRYPTO_GET_KEYBOX:
			if ((hmsg.resp_buf == NULL) || !hmsg.resp_len ||
					((hmsg.offset + hmsg.resp_len) > WIDEVINE_KEYBOX_LEN)) {
				PERR("invalid arguments");
				UP(&keybox_dev->sem);
				return -EFAULT;
			}
#if defined(CONFIG_ARCH_MSM7X30) || defined(CONFIG_ARCH_MSM7X27A)
			ret = htc_keybox_read(keybox_dev, htc_keybox, hmsg.resp_len, hmsg.offset);
			oem_rapi_client_close();
			htc_keybox = keybox_dev->keybox_buf;
#else
			ret = secure_access_item(0, ITEM_KEYBOX_DATA, WIDEVINE_KEYBOX_LEN,
					htc_keybox);
#endif	
			if (ret)
				PERR("get keybox failed (%d)", ret);
			else {
				if (copy_to_user((void __user *)hmsg.resp_buf, htc_keybox + hmsg.offset, hmsg.resp_len)) {
					PERR("copy_to_user error (keybox)");
					UP(&keybox_dev->sem);
					return -EFAULT;
				}
			}
			UP(&keybox_dev->sem);
			break;
		case HTC_OEMCRYPTO_IDENTIFY_DEVICE:
			if ((hmsg.resp_buf == NULL) || ((hmsg.resp_len != DEVICE_ID_LEN) && (hmsg.resp_len != 0xFF))) {
				PERR("invalid arguments");
				UP(&keybox_dev->sem);
				return -EFAULT;
			}

#if defined(CONFIG_ARCH_MSM7X30) || defined(CONFIG_ARCH_MSM7X27A)
			ret = htc_keybox_read(keybox_dev, htc_device_id, hmsg.resp_len, hmsg.offset);
			oem_rapi_client_close();
			htc_device_id = keybox_dev->keybox_buf;
#else
			ret = secure_access_item(0, ITEM_DEVICE_ID, DEVICE_ID_LEN,
					htc_device_id);
#endif	
			if (ret)
				PERR("get device ID failed (%d)", ret);
			else {
				if (copy_to_user((void __user *)hmsg.resp_buf, htc_device_id, DEVICE_ID_LEN)) {
					PERR("copy_to_user error (device ID)");
					UP(&keybox_dev->sem);
					return -EFAULT;
				}
			}
			UP(&keybox_dev->sem);
			break;
		case HTC_OEMCRYPTO_GET_RANDOM:
			if ((hmsg.resp_buf == NULL) || !hmsg.resp_len) {
				PERR("invalid arguments");
				UP(&keybox_dev->sem);
				return -EFAULT;
			}
			ptr = kzalloc(hmsg.resp_len, GFP_KERNEL);
			if (ptr == NULL) {
				PERR("allocate the space for random data failed");
				UP(&keybox_dev->sem);
				return -1;
			}
#if defined(CONFIG_ARCH_MSM7X30) || defined(CONFIG_ARCH_MSM7X27A)
			get_random_bytes(ptr, hmsg.resp_len);
			PINFO("Data get from random entropy");
#else
			get_random_bytes(ptr, hmsg.resp_len);
#endif	
			if (ret)
				PERR("get random data failed (%d)", ret);
			else {
				if (copy_to_user((void __user *)hmsg.resp_buf, ptr, hmsg.resp_len)) {
					PERR("copy_to_user error (random data)");
					kfree(ptr);
					UP(&keybox_dev->sem);
					return -EFAULT;
				}
			}
			UP(&keybox_dev->sem);
			kfree(ptr);
			break;
		case HTC_OEMCRYPTO_IS_KEYBOX_VALID:
#if defined(CONFIG_ARCH_MSM7X30) || defined(CONFIG_ARCH_MSM7X27A)
#else
			ret = secure_access_item(0, ITEM_VALIDATE_KEYBOX, WIDEVINE_KEYBOX_LEN, NULL);
#endif
			UP(&keybox_dev->sem);
			return ret;
		default:
			UP(&keybox_dev->sem);
			PERR("func error");
			return -EFAULT;
		}
		break;
#if !defined(CONFIG_ARCH_MSM7X30) && !defined(CONFIG_ARCH_MSM7X27A)
	case HTCDRM_IOCTL_DISCRETIX:
		mutex_lock(&dx_lock);
		ret = htcdrm_discretix_ioctl(file, command, arg);
		mutex_unlock(&dx_lock);
		break;
#endif
	case HTCDRM_IOCTL_CPRM:
        if (copy_from_user(&hmsg, (void __user *)arg, sizeof(hmsg))) {
            PERR("copy_from_user error (msg)");
            return -EFAULT;
        }

        if ((hmsg.resp_buf == NULL) || !hmsg.resp_len ) {
            PERR("invalid arguments");
            return -EFAULT;
        }

        ret = secure_access_item(0, ITEM_CPRMKEY_DATA, CPRM_KEY_LEN, htc_cprmkey);

        if (ret)
            PERR("get cprmkey failed (%d)", ret);
        else {
            if (copy_to_user( (void __user *)hmsg.resp_buf , htc_cprmkey , hmsg.resp_len)) {
                PERR("copy_to_user error (cprmkey)");
                return -EFAULT;
            }
        }
        break;
	case HTCDRM_IOCTL_GDRIVE:
		ret = htcdrm_gdrive_ioctl(file, command, arg);
		break;

	default:
		PERR("command error");
		return -EFAULT;
	}
	return ret;
}

#if !defined(CONFIG_ARCH_MSM7X30) && !defined(CONFIG_ARCH_MSM7X27A)
static int htcdrm_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int htcdrm_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int htcdrm_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;
	long length = vma->vm_end - vma->vm_start;

	if (length > discretix_smem_size)
		return -EIO;

	PDEBUG("htcdrm_mmap %lx", length);

	
	ret = remap_pfn_range(vma,
			vma->vm_start,
			virt_to_phys((void *)discretix_smem_area) >> PAGE_SHIFT,
			length,
			vma->vm_page_prot);

	return ret;
}


static const struct file_operations htcdrm_fops = {
	.unlocked_ioctl = htcdrm_ioctl,
	.open = htcdrm_open,
	.release = htcdrm_release,
	.mmap = htcdrm_mmap,
	.owner = THIS_MODULE,
};

#else
static const struct file_operations htcdrm_fops = {
	.unlocked_ioctl = htcdrm_ioctl,
	.owner = THIS_MODULE,
};
#endif

static int __init htcdrm_init(void)
{
	int ret;

	htc_device_id = kzalloc(DEVICE_ID_LEN, GFP_KERNEL);
	if (htc_device_id == NULL) {
		PERR("allocate the space for device ID failed");
		return -1;
	}
	htc_keybox = kzalloc(WIDEVINE_KEYBOX_LEN, GFP_KERNEL);
	if (htc_keybox == NULL) {
		PERR("allocate the space for keybox failed");
		kfree(htc_device_id);
		return -1;
	}

#if !defined(CONFIG_ARCH_MSM7X30) && !defined(CONFIG_ARCH_MSM7X27A)
	discretix_smem_ptr = kzalloc(discretix_smem_size + (2 * PAGE_SIZE), GFP_KERNEL);
	if (discretix_smem_ptr == NULL) {
		PERR("allocate the space for DX smem failed");
		kfree(htc_device_id);
		return -1;
	}

	
	discretix_smem_area = (unsigned char *)((((unsigned long)discretix_smem_ptr) + PAGE_SIZE - 1) & PAGE_MASK);
	discretix_smem_phy = (unsigned char *)virt_to_phys(discretix_smem_area);

#if DX_ALLOC_TZ_HEAP
	discretix_tz_heap = kzalloc(DISCRETIX_HEAP_SIZE, GFP_KERNEL);
	if (discretix_tz_heap == NULL) {
		PERR("allocate the space for discretix heap failed");
		kfree(htc_device_id);
		return -1;
	}
#endif

#if (DX_PRE_ALLOC_BUFFER)
	dx_memory_pool_ptr = (unsigned char *)kzalloc(DX_PRE_ALLOC_BUFFER_SIZE, GFP_KERNEL);
	if (dx_memory_pool_ptr == NULL) {
		PERR("allocate dx_memory_pool failed");
		kfree(htc_device_id);
		return -1;
	}
	dx_memory_pool_phy = (unsigned char *)virt_to_phys(dx_memory_pool_ptr);
#endif

#endif
	ret = register_chrdev(0, DEVICE_NAME, &htcdrm_fops);
	if (ret < 0) {
		PERR("register module fail");
		kfree(htc_device_id);
		kfree(htc_keybox);
		return ret;
	}
	htcdrm_major = ret;

	htcdrm_class = class_create(THIS_MODULE, "htcdrm");
	device_create(htcdrm_class, NULL, MKDEV(htcdrm_major, 0), NULL, DEVICE_NAME);

#if defined(CONFIG_ARCH_MSM7X30) || defined(CONFIG_ARCH_MSM7X27A)
	keybox_dev = kzalloc(sizeof(htc_keybox_dev), GFP_KERNEL);
	if (keybox_dev == NULL) {
		PERR("allocate space for keybox_dev failed");
		kfree(keybox_dev);
		return -1;
	}
	sema_init(&keybox_dev->sem, 1);
#endif
	PDEBUG("register module ok");
	return 0;
}


static void  __exit htcdrm_exit(void)
{
	device_destroy(htcdrm_class, MKDEV(htcdrm_major, 0));
	class_unregister(htcdrm_class);
	class_destroy(htcdrm_class);
	unregister_chrdev(htcdrm_major, DEVICE_NAME);
	kfree(htc_device_id);
	kfree(htc_keybox);
#if defined(CONFIG_ARCH_MSM7X30) || defined(CONFIG_ARCH_MSM7X27A)
	kfree(keybox_dev);
#endif

#if !defined(CONFIG_ARCH_MSM7X30) && !defined(CONFIG_ARCH_MSM7X27A)
#if DX_ALLOC_TZ_HEAP
	kfree(discretix_tz_heap);
#endif
	kfree(discretix_smem_ptr);
#endif

	PDEBUG("un-registered module ok");
}

#if DX_ALLOC_TZ_HEAP
module_param(max_ofs, int, S_IRUGO);
MODULE_PARM_DESC(max_ofs, "htcdrm dx max_ofs");
#endif

module_init(htcdrm_init);
module_exit(htcdrm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eddic Hsien");
