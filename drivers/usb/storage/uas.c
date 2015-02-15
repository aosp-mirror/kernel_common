/*
 * USB Attached SCSI
 * Note that this is not the same as the USB Mass Storage driver
 *
 * Copyright Matthew Wilcox for Intel Corp, 2010
 * Copyright Sarah Sharp for Intel Corp, 2010
 *
 * Distributed under the terms of the GNU GPL, version two.
 */

#include <linux/blkdev.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/storage.h>
#include <linux/usb/uas.h>

#include <scsi/scsi.h>
#include <scsi/scsi_dbg.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi_tcq.h>

/*
 * The r00-r01c specs define this version of the SENSE IU data structure.
 * It's still in use by several different firmware releases.
 */
struct sense_iu_old {
	__u8 iu_id;
	__u8 rsvd1;
	__be16 tag;
	__be16 len;
	__u8 status;
	__u8 service_response;
	__u8 sense[SCSI_SENSE_BUFFERSIZE];
};

struct uas_dev_info {
	struct usb_interface *intf;
	struct usb_device *udev;
	int qdepth;
	unsigned cmd_pipe, status_pipe, data_in_pipe, data_out_pipe;
	unsigned use_streams:1;
	unsigned uas_sense_old:1;
	struct scsi_cmnd *cmnd;
	struct urb *status_urb; /* used only if stream support is available */
};

enum {
	ALLOC_STATUS_URB	= (1 << 0),
	SUBMIT_STATUS_URB	= (1 << 1),
	ALLOC_DATA_IN_URB	= (1 << 2),
	SUBMIT_DATA_IN_URB	= (1 << 3),
	ALLOC_DATA_OUT_URB	= (1 << 4),
	SUBMIT_DATA_OUT_URB	= (1 << 5),
	ALLOC_CMD_URB		= (1 << 6),
	SUBMIT_CMD_URB		= (1 << 7),
	COMPLETED_DATA_IN	= (1 << 8),
	COMPLETED_DATA_OUT	= (1 << 9),
	DATA_COMPLETES_CMD	= (1 << 10),
};

/* Overrides scsi_pointer */
struct uas_cmd_info {
	unsigned int state;
	unsigned int stream;
	struct urb *cmd_urb;
	/* status_urb is used only if stream support isn't available */
	struct urb *status_urb;
	struct urb *data_in_urb;
	struct urb *data_out_urb;
	struct list_head list;
};

/* I hate forward declarations, but I actually have a loop */
static int uas_submit_urbs(struct scsi_cmnd *cmnd,
				struct uas_dev_info *devinfo, gfp_t gfp);
static void uas_do_work(struct work_struct *work);

static DECLARE_WORK(uas_work, uas_do_work);
static DEFINE_SPINLOCK(uas_work_lock);
static LIST_HEAD(uas_work_list);

static void uas_do_work(struct work_struct *work)
{
	struct uas_cmd_info *cmdinfo;
	struct uas_cmd_info *temp;
	struct list_head list;
	int err;

	spin_lock_irq(&uas_work_lock);
	list_replace_init(&uas_work_list, &list);
	spin_unlock_irq(&uas_work_lock);

	list_for_each_entry_safe(cmdinfo, temp, &list, list) {
		struct scsi_pointer *scp = (void *)cmdinfo;
		struct scsi_cmnd *cmnd = container_of(scp,
							struct scsi_cmnd, SCp);
		err = uas_submit_urbs(cmnd, cmnd->device->hostdata, GFP_NOIO);
		if (err) {
			list_del(&cmdinfo->list);
			spin_lock_irq(&uas_work_lock);
			list_add_tail(&cmdinfo->list, &uas_work_list);
			spin_unlock_irq(&uas_work_lock);
			schedule_work(&uas_work);
		}
	}
}

static void uas_sense(struct urb *urb, struct scsi_cmnd *cmnd)
{
	struct sense_iu *sense_iu = urb->transfer_buffer;
	struct scsi_device *sdev = cmnd->device;
	struct uas_cmd_info *cmdinfo = (void *)&cmnd->SCp;

	if (urb->actual_length > 16) {
		unsigned len = be16_to_cpup(&sense_iu->len);
		if (len + 16 != urb->actual_length) {
			int newlen = min(len + 16, urb->actual_length) - 16;
			if (newlen < 0)
				newlen = 0;
			sdev_printk(KERN_INFO, sdev, "%s: urb length %d "
				"disagrees with IU sense data length %d, "
				"using %d bytes of sense data\n", __func__,
					urb->actual_length, len, newlen);
			len = newlen;
		}
		memcpy(cmnd->sense_buffer, sense_iu->sense, len);
	}

	cmnd->result = sense_iu->status;
	if (!(cmdinfo->state & DATA_COMPLETES_CMD))
		cmnd->scsi_done(cmnd);
}

static void uas_sense_old(struct urb *urb, struct scsi_cmnd *cmnd)
{
	struct sense_iu_old *sense_iu = urb->transfer_buffer;
	struct scsi_device *sdev = cmnd->device;
	struct uas_cmd_info *cmdinfo = (void *)&cmnd->SCp;

	if (urb->actual_length > 8) {
		unsigned len = be16_to_cpup(&sense_iu->len) - 2;
		if (len + 8 != urb->actual_length) {
			int newlen = min(len + 8, urb->actual_length) - 8;
			if (newlen < 0)
				newlen = 0;
			sdev_printk(KERN_INFO, sdev, "%s: urb length %d "
				"disagrees with IU sense data length %d, "
				"using %d bytes of sense data\n", __func__,
					urb->actual_length, len, newlen);
			len = newlen;
		}
		memcpy(cmnd->sense_buffer, sense_iu->sense, len);
	}

	cmnd->result = sense_iu->status;
	if (!(cmdinfo->state & DATA_COMPLETES_CMD))
		cmnd->scsi_done(cmnd);
}

static void uas_xfer_data(struct urb *urb, struct scsi_cmnd *cmnd,
							unsigned direction)
{
	struct uas_cmd_info *cmdinfo = (void *)&cmnd->SCp;
	int err;

	cmdinfo->state = direction;
	err = uas_submit_urbs(cmnd, cmnd->device->hostdata, GFP_ATOMIC);
	if (err) {
		spin_lock(&uas_work_lock);
		list_add_tail(&cmdinfo->list, &uas_work_list);
		spin_unlock(&uas_work_lock);
		schedule_work(&uas_work);
	}
}

static void uas_stat_cmplt(struct urb *urb)
{
	struct iu *iu = urb->transfer_buffer;
	struct Scsi_Host *shost = urb->context;
	struct uas_dev_info *devinfo = (void *)shost->hostdata[0];
	struct scsi_cmnd *cmnd;
	struct uas_cmd_info *cmdinfo;
	u16 tag;
	int ret;

	if (urb->status) {
		dev_err(&urb->dev->dev, "URB BAD STATUS %d\n", urb->status);
		if (devinfo->use_streams)
			usb_free_urb(urb);
		return;
	}

	tag = be16_to_cpup(&iu->tag) - 1;
	if (tag == 0)
		cmnd = devinfo->cmnd;
	else
		cmnd = scsi_host_find_tag(shost, tag - 1);
	if (!cmnd) {
		if (devinfo->use_streams) {
			usb_free_urb(urb);
			return;
		}
		ret = usb_submit_urb(urb, GFP_ATOMIC);
		if (ret)
			dev_err(&urb->dev->dev, "failed submit status urb\n");
		return;
	}
	cmdinfo = (void *)&cmnd->SCp;

	switch (iu->iu_id) {
	case IU_ID_STATUS:
		if (devinfo->cmnd == cmnd)
			devinfo->cmnd = NULL;

		if (!(cmdinfo->state & COMPLETED_DATA_IN) &&
				cmdinfo->data_in_urb) {
		       if (devinfo->use_streams) {
			       cmdinfo->state |= DATA_COMPLETES_CMD;
			       usb_unlink_urb(cmdinfo->data_in_urb);
		       } else {
			       usb_free_urb(cmdinfo->data_in_urb);
		       }
		}
		if (!(cmdinfo->state & COMPLETED_DATA_OUT) &&
				cmdinfo->data_out_urb) {
			if (devinfo->use_streams) {
				cmdinfo->state |= DATA_COMPLETES_CMD;
				usb_unlink_urb(cmdinfo->data_in_urb);
			} else {
				usb_free_urb(cmdinfo->data_out_urb);
			}
		}

		if (urb->actual_length < 16)
			devinfo->uas_sense_old = 1;
		if (devinfo->uas_sense_old)
			uas_sense_old(urb, cmnd);
		else
			uas_sense(urb, cmnd);
		break;
	case IU_ID_READ_READY:
		uas_xfer_data(urb, cmnd, SUBMIT_DATA_IN_URB);
		break;
	case IU_ID_WRITE_READY:
		uas_xfer_data(urb, cmnd, SUBMIT_DATA_OUT_URB);
		break;
	default:
		scmd_printk(KERN_ERR, cmnd,
			"Bogus IU (%d) received on status pipe\n", iu->iu_id);
	}

	if (devinfo->use_streams) {
		usb_free_urb(urb);
		return;
	}

	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret)
		dev_err(&urb->dev->dev, "failed submit status urb\n");
}

static void uas_data_out_cmplt(struct urb *urb)
{
	struct scsi_cmnd *cmnd = urb->context;
	struct scsi_data_buffer *sdb = scsi_out(cmnd);
	struct uas_cmd_info *cmdinfo = (void *)&cmnd->SCp;

	cmdinfo->state |= COMPLETED_DATA_OUT;

	sdb->resid = sdb->length - urb->actual_length;
	usb_free_urb(urb);

	if (cmdinfo->state & DATA_COMPLETES_CMD)
		cmnd->scsi_done(cmnd);
}

static void uas_data_in_cmplt(struct urb *urb)
{
	struct scsi_cmnd *cmnd = urb->context;
	struct scsi_data_buffer *sdb = scsi_in(cmnd);
	struct uas_cmd_info *cmdinfo = (void *)&cmnd->SCp;

	cmdinfo->state |= COMPLETED_DATA_IN;

	sdb->resid = sdb->length - urb->actual_length;
	usb_free_urb(urb);

	if (cmdinfo->state & DATA_COMPLETES_CMD)
		cmnd->scsi_done(cmnd);
}

static struct urb *uas_alloc_data_urb(struct uas_dev_info *devinfo, gfp_t gfp,
		unsigned int pipe, struct scsi_cmnd *cmnd,
		enum dma_data_direction dir)
{
	struct uas_cmd_info *cmdinfo = (void *)&cmnd->SCp;
	struct usb_device *udev = devinfo->udev;
	struct urb *urb = usb_alloc_urb(0, gfp);
	struct scsi_data_buffer *sdb;
	usb_complete_t complete_fn;
	u16 stream_id = cmdinfo->stream;

	if (!urb)
		goto out;
	if (dir == DMA_FROM_DEVICE) {
		sdb = scsi_in(cmnd);
		complete_fn = uas_data_in_cmplt;
	} else {
		sdb = scsi_out(cmnd);
		complete_fn = uas_data_out_cmplt;
	}
	usb_fill_bulk_urb(urb, udev, pipe, NULL, sdb->length,
			complete_fn, cmnd);
	urb->stream_id = stream_id;
	urb->num_sgs = udev->bus->sg_tablesize ? sdb->table.nents : 0;
	urb->sg = sdb->table.sgl;
 out:
	return urb;
}

static struct urb *uas_alloc_sense_urb(struct uas_dev_info *devinfo, gfp_t gfp,
		struct Scsi_Host *shost, u16 stream_id)
{
	struct usb_device *udev = devinfo->udev;
	struct urb *urb = usb_alloc_urb(0, gfp);
	struct sense_iu *iu;

	if (!urb)
		goto out;

	iu = kzalloc(sizeof(*iu), gfp);
	if (!iu)
		goto free;

	usb_fill_bulk_urb(urb, udev, devinfo->status_pipe, iu, sizeof(*iu),
						uas_stat_cmplt, shost);
	urb->stream_id = stream_id;
	urb->transfer_flags |= URB_FREE_BUFFER;
 out:
	return urb;
 free:
	usb_free_urb(urb);
	return NULL;
}

static struct urb *uas_alloc_cmd_urb(struct uas_dev_info *devinfo, gfp_t gfp,
					struct scsi_cmnd *cmnd, u16 stream_id)
{
	struct usb_device *udev = devinfo->udev;
	struct scsi_device *sdev = cmnd->device;
	struct urb *urb = usb_alloc_urb(0, gfp);
	struct command_iu *iu;
	int len;

	if (!urb)
		goto out;

	len = cmnd->cmd_len - 16;
	if (len < 0)
		len = 0;
	len = ALIGN(len, 4);
	iu = kzalloc(sizeof(*iu) + len, gfp);
	if (!iu)
		goto free;

	iu->iu_id = IU_ID_COMMAND;
	if (blk_rq_tagged(cmnd->request))
		iu->tag = cpu_to_be16(cmnd->request->tag + 2);
	else
		iu->tag = cpu_to_be16(1);
	iu->prio_attr = UAS_SIMPLE_TAG;
	iu->len = len;
	int_to_scsilun(sdev->lun, &iu->lun);
	memcpy(iu->cdb, cmnd->cmnd, cmnd->cmd_len);

	usb_fill_bulk_urb(urb, udev, devinfo->cmd_pipe, iu, sizeof(*iu) + len,
							usb_free_urb, NULL);
	urb->transfer_flags |= URB_FREE_BUFFER;
 out:
	return urb;
 free:
	usb_free_urb(urb);
	return NULL;
}

/*
 * Why should I request the Status IU before sending the Command IU?  Spec
 * says to, but also says the device may receive them in any order.  Seems
 * daft to me.
 */

static int uas_submit_urbs(struct scsi_cmnd *cmnd,
					struct uas_dev_info *devinfo, gfp_t gfp)
{
	struct uas_cmd_info *cmdinfo = (void *)&cmnd->SCp;

	if (cmdinfo->state & ALLOC_STATUS_URB) {
		cmdinfo->status_urb = uas_alloc_sense_urb(devinfo, gfp,
				cmnd->device->host, cmdinfo->stream);
		if (!cmdinfo->status_urb)
			return SCSI_MLQUEUE_DEVICE_BUSY;
		cmdinfo->state &= ~ALLOC_STATUS_URB;
	}

	if (cmdinfo->state & SUBMIT_STATUS_URB) {
		if (usb_submit_urb(cmdinfo->status_urb, gfp)) {
			scmd_printk(KERN_INFO, cmnd,
					"sense urb submission failure\n");
			return SCSI_MLQUEUE_DEVICE_BUSY;
		}
		cmdinfo->state &= ~SUBMIT_STATUS_URB;
	}

	if (cmdinfo->state & ALLOC_DATA_IN_URB) {
		cmdinfo->data_in_urb = uas_alloc_data_urb(devinfo, gfp,
					devinfo->data_in_pipe, cmnd,
					DMA_FROM_DEVICE);
		if (!cmdinfo->data_in_urb)
			return SCSI_MLQUEUE_DEVICE_BUSY;
		cmdinfo->state &= ~ALLOC_DATA_IN_URB;
	}

	if (cmdinfo->state & SUBMIT_DATA_IN_URB) {
		if (usb_submit_urb(cmdinfo->data_in_urb, gfp)) {
			scmd_printk(KERN_INFO, cmnd,
					"data in urb submission failure\n");
			return SCSI_MLQUEUE_DEVICE_BUSY;
		}
		cmdinfo->state &= ~SUBMIT_DATA_IN_URB;
	}

	if (cmdinfo->state & ALLOC_DATA_OUT_URB) {
		cmdinfo->data_out_urb = uas_alloc_data_urb(devinfo, gfp,
					devinfo->data_out_pipe, cmnd,
					DMA_TO_DEVICE);
		if (!cmdinfo->data_out_urb)
			return SCSI_MLQUEUE_DEVICE_BUSY;
		cmdinfo->state &= ~ALLOC_DATA_OUT_URB;
	}

	if (cmdinfo->state & SUBMIT_DATA_OUT_URB) {
		if (usb_submit_urb(cmdinfo->data_out_urb, gfp)) {
			scmd_printk(KERN_INFO, cmnd,
					"data out urb submission failure\n");
			return SCSI_MLQUEUE_DEVICE_BUSY;
		}
		cmdinfo->state &= ~SUBMIT_DATA_OUT_URB;
	}

	if (cmdinfo->state & ALLOC_CMD_URB) {
		cmdinfo->cmd_urb = uas_alloc_cmd_urb(devinfo, gfp, cmnd,
							cmdinfo->stream);
		if (!cmdinfo->cmd_urb)
			return SCSI_MLQUEUE_DEVICE_BUSY;
		cmdinfo->state &= ~ALLOC_CMD_URB;
	}

	if (cmdinfo->state & SUBMIT_CMD_URB) {
		if (usb_submit_urb(cmdinfo->cmd_urb, gfp)) {
			scmd_printk(KERN_INFO, cmnd,
					"cmd urb submission failure\n");
			return SCSI_MLQUEUE_DEVICE_BUSY;
		}
		cmdinfo->state &= ~SUBMIT_CMD_URB;
	}

	return 0;
}

static int uas_queuecommand_lck(struct scsi_cmnd *cmnd,
					void (*done)(struct scsi_cmnd *))
{
	struct scsi_device *sdev = cmnd->device;
	struct uas_dev_info *devinfo = sdev->hostdata;
	struct uas_cmd_info *cmdinfo = (void *)&cmnd->SCp;
	int err;

	BUILD_BUG_ON(sizeof(struct uas_cmd_info) > sizeof(struct scsi_pointer));

	if (devinfo->cmnd)
		return SCSI_MLQUEUE_DEVICE_BUSY;

	if (blk_rq_tagged(cmnd->request)) {
		cmdinfo->stream = cmnd->request->tag + 2;
	} else {
		devinfo->cmnd = cmnd;
		cmdinfo->stream = 1;
	}

	cmnd->scsi_done = done;

	cmdinfo->state = ALLOC_STATUS_URB | SUBMIT_STATUS_URB |
			ALLOC_CMD_URB | SUBMIT_CMD_URB;

	switch (cmnd->sc_data_direction) {
	case DMA_FROM_DEVICE:
		cmdinfo->state |= ALLOC_DATA_IN_URB | SUBMIT_DATA_IN_URB;
		break;
	case DMA_BIDIRECTIONAL:
		cmdinfo->state |= ALLOC_DATA_IN_URB | SUBMIT_DATA_IN_URB;
	case DMA_TO_DEVICE:
		cmdinfo->state |= ALLOC_DATA_OUT_URB | SUBMIT_DATA_OUT_URB;
	case DMA_NONE:
		break;
	}

	if (!devinfo->use_streams) {
		cmdinfo->state &= ~(SUBMIT_DATA_IN_URB | SUBMIT_DATA_OUT_URB |
				ALLOC_STATUS_URB | SUBMIT_STATUS_URB);
		cmdinfo->stream = 0;
	}

	err = uas_submit_urbs(cmnd, devinfo, GFP_ATOMIC);
	if (err) {
		/* If we did nothing, give up now */
		if (cmdinfo->state & SUBMIT_STATUS_URB) {
			usb_free_urb(cmdinfo->status_urb);
			return SCSI_MLQUEUE_DEVICE_BUSY;
		}
		spin_lock(&uas_work_lock);
		list_add_tail(&cmdinfo->list, &uas_work_list);
		spin_unlock(&uas_work_lock);
		schedule_work(&uas_work);
	}

	return 0;
}

static DEF_SCSI_QCMD(uas_queuecommand)

static int uas_eh_abort_handler(struct scsi_cmnd *cmnd)
{
	struct scsi_device *sdev = cmnd->device;
	sdev_printk(KERN_INFO, sdev, "%s tag %d\n", __func__,
							cmnd->request->tag);

/* XXX: Send ABORT TASK Task Management command */
	return FAILED;
}

static int uas_eh_device_reset_handler(struct scsi_cmnd *cmnd)
{
	struct scsi_device *sdev = cmnd->device;
	sdev_printk(KERN_INFO, sdev, "%s tag %d\n", __func__,
							cmnd->request->tag);

/* XXX: Send LOGICAL UNIT RESET Task Management command */
	return FAILED;
}

static int uas_eh_target_reset_handler(struct scsi_cmnd *cmnd)
{
	struct scsi_device *sdev = cmnd->device;
	sdev_printk(KERN_INFO, sdev, "%s tag %d\n", __func__,
							cmnd->request->tag);

/* XXX: Can we reset just the one USB interface?
 * Would calling usb_set_interface() have the right effect?
 */
	return FAILED;
}

static int uas_eh_bus_reset_handler(struct scsi_cmnd *cmnd)
{
	struct scsi_device *sdev = cmnd->device;
	struct uas_dev_info *devinfo = sdev->hostdata;
	struct usb_device *udev = devinfo->udev;

	sdev_printk(KERN_INFO, sdev, "%s tag %d\n", __func__,
							cmnd->request->tag);

	if (usb_reset_device(udev))
		return SUCCESS;

	return FAILED;
}

static int uas_slave_alloc(struct scsi_device *sdev)
{
	sdev->hostdata = (void *)sdev->host->hostdata[0];
	return 0;
}

static int uas_slave_configure(struct scsi_device *sdev)
{
	struct uas_dev_info *devinfo = sdev->hostdata;
	scsi_set_tag_type(sdev, MSG_ORDERED_TAG);
	scsi_activate_tcq(sdev, devinfo->qdepth - 2);
	return 0;
}

static struct scsi_host_template uas_host_template = {
	.module = THIS_MODULE,
	.name = "uas",
	.queuecommand = uas_queuecommand,
	.slave_alloc = uas_slave_alloc,
	.slave_configure = uas_slave_configure,
	.eh_abort_handler = uas_eh_abort_handler,
	.eh_device_reset_handler = uas_eh_device_reset_handler,
	.eh_target_reset_handler = uas_eh_target_reset_handler,
	.eh_bus_reset_handler = uas_eh_bus_reset_handler,
	.can_queue = 65536,	/* Is there a limit on the _host_ ? */
	.this_id = -1,
	.sg_tablesize = SG_NONE,
	.cmd_per_lun = 1,	/* until we override it */
	.skip_settle_delay = 1,
	.ordered_tag = 1,
};

static struct usb_device_id uas_usb_ids[] = {
	{ USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, USB_SC_SCSI, USB_PR_BULK) },
	{ USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, USB_SC_SCSI, USB_PR_UAS) },
	/* 0xaa is a prototype device I happen to have access to */
	{ USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, USB_SC_SCSI, 0xaa) },
	{ }
};
MODULE_DEVICE_TABLE(usb, uas_usb_ids);

static int uas_is_interface(struct usb_host_interface *intf)
{
	return (intf->desc.bInterfaceClass == USB_CLASS_MASS_STORAGE &&
		intf->desc.bInterfaceSubClass == USB_SC_SCSI &&
		intf->desc.bInterfaceProtocol == USB_PR_UAS);
}

static int uas_isnt_supported(struct usb_device *udev)
{
	struct usb_hcd *hcd = bus_to_hcd(udev->bus);

	dev_warn(&udev->dev, "The driver for the USB controller %s does not "
			"support scatter-gather which is\n",
			hcd->driver->description);
	dev_warn(&udev->dev, "required by the UAS driver. Please try an"
			"alternative USB controller if you wish to use UAS.\n");
	return -ENODEV;
}

static int uas_switch_interface(struct usb_device *udev,
						struct usb_interface *intf)
{
	int i;
	int sg_supported = udev->bus->sg_tablesize != 0;

	for (i = 0; i < intf->num_altsetting; i++) {
		struct usb_host_interface *alt = &intf->altsetting[i];

		if (uas_is_interface(alt)) {
			if (!sg_supported)
				return uas_isnt_supported(udev);
			return usb_set_interface(udev,
						alt->desc.bInterfaceNumber,
						alt->desc.bAlternateSetting);
		}
	}

	return -ENODEV;
}

static void uas_configure_endpoints(struct uas_dev_info *devinfo)
{
	struct usb_host_endpoint *eps[4] = { };
	struct usb_interface *intf = devinfo->intf;
	struct usb_device *udev = devinfo->udev;
	struct usb_host_endpoint *endpoint = intf->cur_altsetting->endpoint;
	unsigned i, n_endpoints = intf->cur_altsetting->desc.bNumEndpoints;

	devinfo->uas_sense_old = 0;
	devinfo->cmnd = NULL;

	for (i = 0; i < n_endpoints; i++) {
		unsigned char *extra = endpoint[i].extra;
		int len = endpoint[i].extralen;
		while (len > 1) {
			if (extra[1] == USB_DT_PIPE_USAGE) {
				unsigned pipe_id = extra[2];
				if (pipe_id > 0 && pipe_id < 5)
					eps[pipe_id - 1] = &endpoint[i];
				break;
			}
			len -= extra[0];
			extra += extra[0];
		}
	}

	/*
	 * Assume that if we didn't find a control pipe descriptor, we're
	 * using a device with old firmware that happens to be set up like
	 * this.
	 */
	if (!eps[0]) {
		devinfo->cmd_pipe = usb_sndbulkpipe(udev, 1);
		devinfo->status_pipe = usb_rcvbulkpipe(udev, 1);
		devinfo->data_in_pipe = usb_rcvbulkpipe(udev, 2);
		devinfo->data_out_pipe = usb_sndbulkpipe(udev, 2);

		eps[1] = usb_pipe_endpoint(udev, devinfo->status_pipe);
		eps[2] = usb_pipe_endpoint(udev, devinfo->data_in_pipe);
		eps[3] = usb_pipe_endpoint(udev, devinfo->data_out_pipe);
	} else {
		devinfo->cmd_pipe = usb_sndbulkpipe(udev,
						eps[0]->desc.bEndpointAddress);
		devinfo->status_pipe = usb_rcvbulkpipe(udev,
						eps[1]->desc.bEndpointAddress);
		devinfo->data_in_pipe = usb_rcvbulkpipe(udev,
						eps[2]->desc.bEndpointAddress);
		devinfo->data_out_pipe = usb_sndbulkpipe(udev,
						eps[3]->desc.bEndpointAddress);
	}

	devinfo->qdepth = usb_alloc_streams(devinfo->intf, eps + 1, 3, 256,
								GFP_KERNEL);
	if (devinfo->qdepth < 0) {
		devinfo->qdepth = 256;
		devinfo->use_streams = 0;
	} else {
		devinfo->use_streams = 1;
	}
}

static int uas_alloc_status_urb(struct uas_dev_info *devinfo,
		struct Scsi_Host *shost)
{
	if (devinfo->use_streams) {
		devinfo->status_urb = NULL;
		return 0;
	}

	devinfo->status_urb = uas_alloc_sense_urb(devinfo, GFP_KERNEL,
			shost, 0);
	if (!devinfo->status_urb)
		goto err_s_urb;

	if (usb_submit_urb(devinfo->status_urb, GFP_KERNEL))
		goto err_submit_urb;

	return 0;
err_submit_urb:
	usb_free_urb(devinfo->status_urb);
err_s_urb:
	return -ENOMEM;
}

static void uas_free_streams(struct uas_dev_info *devinfo)
{
	struct usb_device *udev = devinfo->udev;
	struct usb_host_endpoint *eps[3];

	eps[0] = usb_pipe_endpoint(udev, devinfo->status_pipe);
	eps[1] = usb_pipe_endpoint(udev, devinfo->data_in_pipe);
	eps[2] = usb_pipe_endpoint(udev, devinfo->data_out_pipe);
	usb_free_streams(devinfo->intf, eps, 3, GFP_KERNEL);
}

/*
 * XXX: What I'd like to do here is register a SCSI host for each USB host in
 * the system.  Follow usb-storage's design of registering a SCSI host for
 * each USB device for the moment.  Can implement this by walking up the
 * USB hierarchy until we find a USB host.
 */
static int uas_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	int result;
	struct Scsi_Host *shost;
	struct uas_dev_info *devinfo;
	struct usb_device *udev = interface_to_usbdev(intf);

	if (uas_switch_interface(udev, intf))
		return -ENODEV;

	devinfo = kmalloc(sizeof(struct uas_dev_info), GFP_KERNEL);
	if (!devinfo)
		return -ENOMEM;

	result = -ENOMEM;
	shost = scsi_host_alloc(&uas_host_template, sizeof(void *));
	if (!shost)
		goto free;

	shost->max_cmd_len = 16 + 252;
	shost->max_id = 1;
	shost->sg_tablesize = udev->bus->sg_tablesize;

	devinfo->intf = intf;
	devinfo->udev = udev;
	uas_configure_endpoints(devinfo);

	result = scsi_init_shared_tag_map(shost, devinfo->qdepth - 2);
	if (result)
		goto free;

	result = scsi_add_host(shost, &intf->dev);
	if (result)
		goto deconfig_eps;

	shost->hostdata[0] = (unsigned long)devinfo;

	result = uas_alloc_status_urb(devinfo, shost);
	if (result)
		goto err_alloc_status;

	scsi_scan_host(shost);
	usb_set_intfdata(intf, shost);
	return result;

err_alloc_status:
	scsi_remove_host(shost);
	shost = NULL;
deconfig_eps:
	uas_free_streams(devinfo);
 free:
	kfree(devinfo);
	if (shost)
		scsi_host_put(shost);
	return result;
}

static int uas_pre_reset(struct usb_interface *intf)
{
/* XXX: Need to return 1 if it's not our device in error handling */
	return 0;
}

static int uas_post_reset(struct usb_interface *intf)
{
/* XXX: Need to return 1 if it's not our device in error handling */
	return 0;
}

static void uas_disconnect(struct usb_interface *intf)
{
	struct Scsi_Host *shost = usb_get_intfdata(intf);
	struct uas_dev_info *devinfo = (void *)shost->hostdata[0];

	scsi_remove_host(shost);
	usb_kill_urb(devinfo->status_urb);
	usb_free_urb(devinfo->status_urb);
	uas_free_streams(devinfo);
	kfree(devinfo);
}

/*
 * XXX: Should this plug into libusual so we can auto-upgrade devices from
 * Bulk-Only to UAS?
 */
static struct usb_driver uas_driver = {
	.name = "uas",
	.probe = uas_probe,
	.disconnect = uas_disconnect,
	.pre_reset = uas_pre_reset,
	.post_reset = uas_post_reset,
	.id_table = uas_usb_ids,
};

module_usb_driver(uas_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Matthew Wilcox and Sarah Sharp");
