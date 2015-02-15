
static int diagcharqsc_open(struct inode *inode, struct file *file)
{
	int i = 0;

	DIAG_INFO("%s:%s(parent:%s): tgid=%d\n", __func__,
			current->comm, current->parent->comm, current->tgid);

	if (driver) {
		mutex_lock(&driver->diagcharqsc_mutex);

		for (i = 0; i < driver->num_qscclients; i++)
			if (driver->qscclient_map[i].pid == 0)
				break;

		if (i < driver->num_qscclients) {
			driver->qscclient_map[i].pid = current->tgid;
			strncpy(driver->qscclient_map[i].name, current->comm, 20);
			driver->qscclient_map[i].name[19] = '\0';
		} else {
			mutex_unlock(&driver->diagcharqsc_mutex);
			DIAG_INFO("%s:reach max client count\n", __func__);
			for (i = 0; i < driver->num_clients; i++)
				DIAG_WARNING("%d) %s PID=%d", i, driver->
					qscclient_map[i].name,
					driver->qscclient_map[i].pid);
			return -ENOMEM;
		}

		driver->qscdata_ready[i] |= MSG_MASKS_TYPE;
		driver->qscdata_ready[i] |= EVENT_MASKS_TYPE;
		driver->qscdata_ready[i] |= LOG_MASKS_TYPE;

		if (driver->ref_count == 0)
			diagmem_init(driver);
		driver->ref_count++;

		mutex_unlock(&driver->diagcharqsc_mutex);
		return 0;
	}

	return -ENOMEM;

}

static int diagcharqsc_close(struct inode *inode, struct file *file)
{

	int i = 0;

	DIAG_INFO("%s:%s(parent:%s): tgid=%d\n", __func__,
			current->comm, current->parent->comm, current->tgid);

	if (driver) {
		mutex_lock(&driver->diagcharqsc_mutex);

		driver->ref_count--;
		/* On Client exit, try to destroy all 3 pools */
		diagmem_exit(driver, POOL_TYPE_COPY);
		diagmem_exit(driver, POOL_TYPE_HDLC);
		diagmem_exit(driver, POOL_TYPE_WRITE_STRUCT);

		for (i = 0; i < driver->num_qscclients; i++)
			if (driver->qscclient_map[i].pid == current->tgid) {
				driver->qscclient_map[i].pid = 0;
				break;
			}

		if (i < driver->num_qscclients)
			DIAG_INFO("%s:#%d(%d) %s close\n", __func__,
				i, current->tgid, current->comm);
		else
			DIAG_WARNING("%s: nothing close\n", __func__);
		mutex_unlock(&driver->diagcharqsc_mutex);
		return 0;
	}

	return -ENOMEM;
}

static long diagcharqsc_ioctl(struct file *filp,
		unsigned int iocmd, unsigned long ioarg)
{
	int success = -1, i;

	if (iocmd == DIAG_IOCTL_SWITCH_LOGGING) {
		mutex_lock(&driver->diagcharqsc_mutex);
		driver->logging_mode = (int)ioarg;
		driver->qsc_logging_process_id = current->tgid;
		mutex_unlock(&driver->diagcharqsc_mutex);
		if (driver->logging_mode == MEMORY_DEVICE_MODE) {
			DIAG_INFO("diagcharqsc_ioctl enable\n");
			/* diagfwd_disconnect(); */
#ifdef CONFIG_DIAGFWD_BRIDGE_CODE
			/* FIXME: Dyson porting */
			#if 0
			diagfwd_cancel_hsic();
			#endif
			diagfwd_connect_bridge(0);
#endif
			driver->qxdm2sd_drop = 0;
		} else if (driver->logging_mode == USB_MODE) {
			DIAG_INFO("diagcharqsc_ioctl disable\n");
			/* diagfwd_connect(); */
#ifdef CONFIG_DIAGFWD_BRIDGE_CODE
			/* FIXME: Dyson porting */
			#if 0
			diag_clear_hsic_tbl();
			diagfwd_cancel_hsic();
			#endif
			diagfwd_connect_bridge(0);
#endif
			driver->qxdm2sd_drop = 1;
		}
		success = 1;
	} else if (iocmd == DIAG_IOCTL_NONBLOCKING_TIMEOUT) {
		for (i = 0; i < driver->num_qscclients; i++)
			if (driver->qscclient_map[i].pid == current->tgid)
				break;
		if (i == -1)
			return -EINVAL;
		mutex_lock(&driver->diagcharqsc_mutex);
		driver->qscclient_map[i].timeout = (int)ioarg;
		mutex_unlock(&driver->diagcharqsc_mutex);

		success = 1;
	}
	return success;
}

static int diagcharqsc_read(struct file *file, char __user *buf, size_t count,
		loff_t *ppos)
{

	int index = -1, i = 0, ret = 0, timeout = 0;
	int num_data = 0, data_type;
	int exit_stat;

	if (diag9k_debug_mask)
		DIAG_INFO("%s:%s(parent:%s): tgid=%d\n", __func__,
			current->comm, current->parent->comm, current->tgid);

	for (i = 0; i < driver->num_qscclients; i++)
		if (driver->qscclient_map[i].pid == current->tgid) {
			index = i;
			timeout = driver->qscclient_map[i].timeout;
		}

	if (index == -1) {
		DIAG_ERR("%s:%s(parent:%s): tgid=%d "
				"Client PID not found in table\n", __func__,
				current->comm, current->parent->comm, current->tgid);
		for (i = 0; i < driver->num_qscclients; i++)
			DIAG_ERR("\t#%d: %d\n", i, driver->qscclient_map[i].pid);
		return -EINVAL;
	}

	if (timeout)
		wait_event_interruptible_timeout(driver->qscwait_q,
				driver->qscdata_ready[index], timeout * HZ);
	else
		wait_event_interruptible(driver->qscwait_q,
				driver->qscdata_ready[index]);

	mutex_lock(&driver->diagcharqsc_mutex);

	if ((driver->qscdata_ready[index] & USER_SPACE_DATA_TYPE) && (driver->
				logging_mode == MEMORY_DEVICE_MODE)) {
		/*Copy the type of data being passed*/
		data_type = driver->data_ready[index] & USER_SPACE_DATA_TYPE;
		COPY_USER_SPACE_OR_EXIT(buf, data_type, 4);
		/* place holder for number of data field */
		ret += 4;

		/* Copy date from remote processors */
		exit_stat = diag_copy_remote(buf, count, &ret, &num_data);
		if (exit_stat == 1)
			goto exit;

		/* copy number of data fields */
		COPY_USER_SPACE_OR_EXIT(buf+4, num_data, 4);
		ret -= 4;

		driver->qscdata_ready[index] ^= USER_SPACE_DATA_TYPE;

		goto exit;
	} else if (driver->qscdata_ready[index] & USER_SPACE_DATA_TYPE) {
		/* In case, the thread wakes up and the logging mode is
		   not memory device any more, the condition needs to be cleared */
		driver->qscdata_ready[index] ^= USER_SPACE_DATA_TYPE;
	} else if (driver->qscdata_ready[index] & USERMODE_DIAGFWD) {
		data_type = USERMODE_DIAGFWD_LEGACY;
		driver->qscdata_ready[index] ^= USERMODE_DIAGFWD;
		COPY_USER_SPACE_OR_EXIT(buf, data_type, 4);

		/* WARN: we did not care the num_data so assign it to negative value */
		num_data = -QSC;
		/* Copy date from remote processors */
		exit_stat = diag_copy_remote(buf, count, &ret, &num_data);
		if (exit_stat == 1)
			goto exit;

		if (diag9k_debug_mask)
			pr_info("%s() return %d byte\n", __func__, ret);
		goto exit;
	}

	if (driver->qscdata_ready[index] & DEINIT_TYPE) {

		driver->qscdata_ready[index] ^= DEINIT_TYPE;
		goto exit;
	}

	if (driver->qscdata_ready[index] & MSG_MASKS_TYPE) {

		driver->qscdata_ready[index] ^= MSG_MASKS_TYPE;
		goto exit;
	}

	if (driver->qscdata_ready[index] & EVENT_MASKS_TYPE) {

		driver->qscdata_ready[index] ^= EVENT_MASKS_TYPE;
		goto exit;
	}

	if (driver->qscdata_ready[index] & LOG_MASKS_TYPE) {

		driver->qscdata_ready[index] ^= LOG_MASKS_TYPE;
		goto exit;
	}

	if (driver->qscdata_ready[index] & PKT_TYPE) {

		driver->qscdata_ready[index] ^= PKT_TYPE;
		goto exit;
	}
exit:
	if (ret)
		wake_lock_timeout(&driver->wake_lock, HZ / 2);

	mutex_unlock(&driver->diagcharqsc_mutex);

	return ret;
}

static int diagcharqsc_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{

	int err, pkt_type;
	int payload_size;

	if (diag9k_debug_mask)
		DIAG_INFO("%s:%s(parent:%s): tgid=%d\n", __func__,
				current->comm, current->parent->comm, current->tgid);

#ifdef CONFIG_DIAG_OVER_USB
	if (((driver->logging_mode == USB_MODE) && (!driver->usb_connected)) ||
			(driver->logging_mode == NO_LOGGING_MODE)) {
		/*Drop the diag payload */
		return -EIO;
	}
#endif /* DIAG over USB */

	/* Get the packet type F3/log/event/Pkt response */
	err = copy_from_user((&pkt_type), buf, 4);
	/*First 4 bytes indicate the type of payload - ignore these */
	payload_size = count - 4;
	if (pkt_type == USER_SPACE_DATA_TYPE) {
		err = copy_from_user(driver->user_space_qsc_data, buf + 4,
							 payload_size);
		/* Check masks for On-Device logging */
		if (driver->mask_check) {
			if (!mask_request_validate(driver->user_space_qsc_data)) {
				DIAG_ERR("mask request Invalid ..cannot send to modem \n");
				return -EFAULT;
			}
		}

		pr_debug("diag: user space data %d\n", payload_size);
		DIAGFWD_9K_RAWDATA(driver->user_space_qsc_data, "SMUX", DIAG_DBG_WRITE);

#ifdef CONFIG_DIAGFWD_BRIDGE_CODE
		if (driver->diag_smux_enabled && driver->lcid) {
			if (payload_size > 0) {
				err = msm_smux_write(driver->lcid, NULL,
						driver->user_space_qsc_data, payload_size);
				if (err) {
					pr_err("diag:send mask to MDM err %d",
							err);
					return err;
				}
			}
		}
#endif
		if (diag9k_debug_mask)
			pr_info("%s() %d byte\n", __func__, payload_size);
		return count;
	} else if (driver->qsc_logging_process_id == current->tgid) {
		err = copy_from_user(driver->user_space_qsc_data, buf + 4, payload_size);

		pr_debug("diag: user space data %d\n", payload_size);
		DIAGFWD_9K_RAWDATA(driver->user_space_qsc_data, "SMUX", DIAG_DBG_WRITE);

		if (driver->diag_smux_enabled && driver->lcid) {
			if (payload_size > 0) {
				err = msm_smux_write(driver->lcid, NULL,
						driver->user_space_qsc_data, payload_size);
				if (err) {
					pr_err("diag:send mask to MDM err %d",
							err);
					return err;
				}
			}
		}

		if (diag9k_debug_mask)
			pr_info("%s() %d byte\n", __func__, payload_size);
		return count;
	}
	return 0;
}

static const struct file_operations diagcharqscfops = {
	.owner = THIS_MODULE,
	.read = diagcharqsc_read,
	.write = diagcharqsc_write,
	.unlocked_ioctl = diagcharqsc_ioctl,
	.open = diagcharqsc_open,
	.release = diagcharqsc_close
};

static __maybe_unused void diagcharqsc_setup_cdev(dev_t devno)
{
	int err;

	driver->cdev_qsc = cdev_alloc();
	cdev_init(driver->cdev_qsc, &diagcharqscfops);

	driver->cdev_qsc->owner = THIS_MODULE;
	driver->cdev_qsc->ops = &diagcharqscfops;

	err = cdev_add(driver->cdev_qsc, devno, 1);

	if (err) {
		DIAG_ERR("diagchar cdev qsc registration failed !\n\n");
		return;
	}

	device_create(driver->diagchar_class, NULL, devno, (void *)driver, "diag_qsc");
}

static __maybe_unused void diagcharqsc_init(void)
{
	driver->num_qscclients = 1;
	init_waitqueue_head(&driver->qscwait_q);
	mutex_init(&driver->diagcharqsc_mutex);

	if (driver->user_space_qsc_data == NULL)
		driver->user_space_qsc_data = kzalloc(USER_SPACE_DATA, GFP_KERNEL);
		if (driver->user_space_qsc_data == NULL)
			goto err;
	kmemleak_not_leak(driver->user_space_qsc_data);
	if (driver->qscclient_map == NULL &&
		(driver->qscclient_map = kzalloc
			((driver->num_qscclients) * sizeof(struct diag_client_map),
			GFP_KERNEL)) == NULL)
		goto err;
	kmemleak_not_leak(driver->qscclient_map);
	if (driver->qscdata_ready == NULL &&
		(driver->qscdata_ready = kzalloc(driver->num_qscclients * sizeof(struct
			diag_client_map), GFP_KERNEL)) == NULL)
		goto err;
	kmemleak_not_leak(driver->qscdata_ready);
err:
	kfree(driver->qscdata_ready);
	kfree(driver->qscclient_map);
	kfree(driver->user_space_qsc_data);
}
