/*
 * SPI init/core code
 *
 * Copyright (C) 2005 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/cache.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/mod_devicetable.h>
#include <linux/spi/spi.h>
#include <linux/of_spi.h>
#include <linux/pm_runtime.h>
#include <linux/export.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/kthread.h>

static void spidev_release(struct device *dev)
{
	struct spi_device	*spi = to_spi_device(dev);

	
	if (spi->master->cleanup)
		spi->master->cleanup(spi);

	spi_master_put(spi->master);
	kfree(spi);
}

static ssize_t
modalias_show(struct device *dev, struct device_attribute *a, char *buf)
{
	const struct spi_device	*spi = to_spi_device(dev);

	return sprintf(buf, "%s\n", spi->modalias);
}

static struct device_attribute spi_dev_attrs[] = {
	__ATTR_RO(modalias),
	__ATTR_NULL,
};


static const struct spi_device_id *spi_match_id(const struct spi_device_id *id,
						const struct spi_device *sdev)
{
	while (id->name[0]) {
		if (!strcmp(sdev->modalias, id->name))
			return id;
		id++;
	}
	return NULL;
}

const struct spi_device_id *spi_get_device_id(const struct spi_device *sdev)
{
	const struct spi_driver *sdrv = to_spi_driver(sdev->dev.driver);

	
	if (sdrv == NULL) {
		pr_err("%s: sdrv is NULL\n", __func__);
		return NULL;
	}
	

	return spi_match_id(sdrv->id_table, sdev);
}
EXPORT_SYMBOL_GPL(spi_get_device_id);

static int spi_match_device(struct device *dev, struct device_driver *drv)
{
	const struct spi_device	*spi = to_spi_device(dev);
	const struct spi_driver	*sdrv = to_spi_driver(drv);

	
	if (of_driver_match_device(dev, drv))
		return 1;

	
	if (sdrv == NULL) {
		pr_err("%s: sdrv is NULL\n", __func__);
		return 0;
	}
	

	if (sdrv->id_table)
		return !!spi_match_id(sdrv->id_table, spi);

	return strcmp(spi->modalias, drv->name) == 0;
}

static int spi_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	const struct spi_device		*spi = to_spi_device(dev);

	add_uevent_var(env, "MODALIAS=%s%s", SPI_MODULE_PREFIX, spi->modalias);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int spi_legacy_suspend(struct device *dev, pm_message_t message)
{
	int			value = 0;
	struct spi_driver	*drv = to_spi_driver(dev->driver);

	
	if (drv) {
		if (drv->suspend)
			value = drv->suspend(to_spi_device(dev), message);
		else
			dev_dbg(dev, "... can't suspend\n");
	}
	return value;
}

static int spi_legacy_resume(struct device *dev)
{
	int			value = 0;
	struct spi_driver	*drv = to_spi_driver(dev->driver);

	
	if (drv) {
		if (drv->resume)
			value = drv->resume(to_spi_device(dev));
		else
			dev_dbg(dev, "... can't resume\n");
	}
	return value;
}

static int spi_pm_suspend(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_suspend(dev);
	else
		return spi_legacy_suspend(dev, PMSG_SUSPEND);
}

static int spi_pm_resume(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_resume(dev);
	else
		return spi_legacy_resume(dev);
}

static int spi_pm_freeze(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_freeze(dev);
	else
		return spi_legacy_suspend(dev, PMSG_FREEZE);
}

static int spi_pm_thaw(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_thaw(dev);
	else
		return spi_legacy_resume(dev);
}

static int spi_pm_poweroff(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_poweroff(dev);
	else
		return spi_legacy_suspend(dev, PMSG_HIBERNATE);
}

static int spi_pm_restore(struct device *dev)
{
	const struct dev_pm_ops *pm = dev->driver ? dev->driver->pm : NULL;

	if (pm)
		return pm_generic_restore(dev);
	else
		return spi_legacy_resume(dev);
}
#else
#define spi_pm_suspend	NULL
#define spi_pm_resume	NULL
#define spi_pm_freeze	NULL
#define spi_pm_thaw	NULL
#define spi_pm_poweroff	NULL
#define spi_pm_restore	NULL
#endif

static const struct dev_pm_ops spi_pm = {
	.suspend = spi_pm_suspend,
	.resume = spi_pm_resume,
	.freeze = spi_pm_freeze,
	.thaw = spi_pm_thaw,
	.poweroff = spi_pm_poweroff,
	.restore = spi_pm_restore,
	SET_RUNTIME_PM_OPS(
		pm_generic_runtime_suspend,
		pm_generic_runtime_resume,
		pm_generic_runtime_idle
	)
};

struct bus_type spi_bus_type = {
	.name		= "spi",
	.dev_attrs	= spi_dev_attrs,
	.match		= spi_match_device,
	.uevent		= spi_uevent,
	.pm		= &spi_pm,
};
EXPORT_SYMBOL_GPL(spi_bus_type);


static int spi_drv_probe(struct device *dev)
{
	const struct spi_driver		*sdrv = to_spi_driver(dev->driver);

	
	if (sdrv == NULL) {
		pr_err("%s: sdrv is NULL\n", __func__);
		return -ENODEV;
	}
	

	return sdrv->probe(to_spi_device(dev));
}

static int spi_drv_remove(struct device *dev)
{
	const struct spi_driver		*sdrv = to_spi_driver(dev->driver);

	
	if (sdrv == NULL) {
		pr_err("%s: sdrv is NULL\n", __func__);
		return -ENODEV;
	}
	

	return sdrv->remove(to_spi_device(dev));
}

static void spi_drv_shutdown(struct device *dev)
{
	const struct spi_driver		*sdrv = to_spi_driver(dev->driver);

	
	if (sdrv == NULL) {
		pr_err("%s: sdrv is NULL\n", __func__);
		return;
	}
	

	sdrv->shutdown(to_spi_device(dev));
}

int spi_register_driver(struct spi_driver *sdrv)
{
	sdrv->driver.bus = &spi_bus_type;
	if (sdrv->probe)
		sdrv->driver.probe = spi_drv_probe;
	if (sdrv->remove)
		sdrv->driver.remove = spi_drv_remove;
	if (sdrv->shutdown)
		sdrv->driver.shutdown = spi_drv_shutdown;
	return driver_register(&sdrv->driver);
}
EXPORT_SYMBOL_GPL(spi_register_driver);



struct boardinfo {
	struct list_head	list;
	struct spi_board_info	board_info;
};

static LIST_HEAD(board_list);
static LIST_HEAD(spi_master_list);

static DEFINE_MUTEX(board_lock);

struct spi_device *spi_alloc_device(struct spi_master *master)
{
	struct spi_device	*spi;
	struct device		*dev = master->dev.parent;

	if (!spi_master_get(master))
		return NULL;

	spi = kzalloc(sizeof *spi, GFP_KERNEL);
	if (!spi) {
		dev_err(dev, "cannot alloc spi_device\n");
		spi_master_put(master);
		return NULL;
	}

	spi->master = master;
	spi->dev.parent = &master->dev;
	spi->dev.bus = &spi_bus_type;
	spi->dev.release = spidev_release;
	device_initialize(&spi->dev);
	return spi;
}
EXPORT_SYMBOL_GPL(spi_alloc_device);

int spi_add_device(struct spi_device *spi)
{
	static DEFINE_MUTEX(spi_add_lock);
	struct device *dev = spi->master->dev.parent;
	struct device *d;
	int status;

	
	if (spi->chip_select >= spi->master->num_chipselect) {
		dev_err(dev, "cs%d >= max %d\n",
			spi->chip_select,
			spi->master->num_chipselect);
		return -EINVAL;
	}

	
	dev_set_name(&spi->dev, "%s.%u", dev_name(&spi->master->dev),
			spi->chip_select);


	mutex_lock(&spi_add_lock);

	d = bus_find_device_by_name(&spi_bus_type, NULL, dev_name(&spi->dev));
	if (d != NULL) {
		dev_err(dev, "chipselect %d already in use\n",
				spi->chip_select);
		put_device(d);
		status = -EBUSY;
		goto done;
	}

	status = spi_setup(spi);
	if (status < 0) {
		dev_err(dev, "can't setup %s, status %d\n",
				dev_name(&spi->dev), status);
		goto done;
	}

	
	status = device_add(&spi->dev);
	if (status < 0)
		dev_err(dev, "can't add %s, status %d\n",
				dev_name(&spi->dev), status);
	else
		dev_dbg(dev, "registered child %s\n", dev_name(&spi->dev));

done:
	mutex_unlock(&spi_add_lock);
	return status;
}
EXPORT_SYMBOL_GPL(spi_add_device);

struct spi_device *spi_new_device(struct spi_master *master,
				  struct spi_board_info *chip)
{
	struct spi_device	*proxy;
	int			status;


	proxy = spi_alloc_device(master);
	if (!proxy)
		return NULL;

	WARN_ON(strlen(chip->modalias) >= sizeof(proxy->modalias));

	proxy->chip_select = chip->chip_select;
	proxy->max_speed_hz = chip->max_speed_hz;
	proxy->mode = chip->mode;
	proxy->irq = chip->irq;
	strlcpy(proxy->modalias, chip->modalias, sizeof(proxy->modalias));
	proxy->dev.platform_data = (void *) chip->platform_data;
	proxy->controller_data = chip->controller_data;
	proxy->controller_state = NULL;

	status = spi_add_device(proxy);
	if (status < 0) {
		spi_dev_put(proxy);
		return NULL;
	}

	return proxy;
}
EXPORT_SYMBOL_GPL(spi_new_device);

static void spi_match_master_to_boardinfo(struct spi_master *master,
				struct spi_board_info *bi)
{
	struct spi_device *dev;

	if (master->bus_num != bi->bus_num)
		return;

	dev = spi_new_device(master, bi);
	if (!dev)
		dev_err(master->dev.parent, "can't create new device for %s\n",
			bi->modalias);
}

int __devinit
spi_register_board_info(struct spi_board_info const *info, unsigned n)
{
	struct boardinfo *bi;
	int i;

	bi = kzalloc(n * sizeof(*bi), GFP_KERNEL);
	if (!bi)
		return -ENOMEM;

	for (i = 0; i < n; i++, bi++, info++) {
		struct spi_master *master;

		memcpy(&bi->board_info, info, sizeof(*info));
		mutex_lock(&board_lock);
		list_add_tail(&bi->list, &board_list);
		list_for_each_entry(master, &spi_master_list, list)
			spi_match_master_to_boardinfo(master, &bi->board_info);
		mutex_unlock(&board_lock);
	}

	return 0;
}


static void spi_pump_messages(struct kthread_work *work)
{
	struct spi_master *master =
		container_of(work, struct spi_master, pump_messages);
	unsigned long flags;
	bool was_busy = false;
	int ret;

	
	spin_lock_irqsave(&master->queue_lock, flags);
	if (list_empty(&master->queue) || !master->running) {
		if (master->busy) {
			ret = master->unprepare_transfer_hardware(master);
			if (ret) {
				spin_unlock_irqrestore(&master->queue_lock, flags);
				dev_err(&master->dev,
					"failed to unprepare transfer hardware\n");
				return;
			}
		}
		master->busy = false;
		spin_unlock_irqrestore(&master->queue_lock, flags);
		return;
	}

	
	if (master->cur_msg) {
		spin_unlock_irqrestore(&master->queue_lock, flags);
		return;
	}
	
	master->cur_msg =
	    list_entry(master->queue.next, struct spi_message, queue);

	list_del_init(&master->cur_msg->queue);
	if (master->busy)
		was_busy = true;
	else
		master->busy = true;
	spin_unlock_irqrestore(&master->queue_lock, flags);

	if (!was_busy) {
		ret = master->prepare_transfer_hardware(master);
		if (ret) {
			dev_err(&master->dev,
				"failed to prepare transfer hardware\n");
			return;
		}
	}

	ret = master->transfer_one_message(master, master->cur_msg);
	if (ret) {
		dev_err(&master->dev,
			"failed to transfer one message from queue\n");
		return;
	}
}

static int spi_init_queue(struct spi_master *master)
{
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };

	INIT_LIST_HEAD(&master->queue);
	spin_lock_init(&master->queue_lock);

	master->running = false;
	master->busy = false;

	init_kthread_worker(&master->kworker);
	master->kworker_task = kthread_run(kthread_worker_fn,
					   &master->kworker,
					   dev_name(&master->dev));
	if (IS_ERR(master->kworker_task)) {
		dev_err(&master->dev, "failed to create message pump task\n");
		return -ENOMEM;
	}
	init_kthread_work(&master->pump_messages, spi_pump_messages);

	if (master->rt) {
		dev_info(&master->dev,
			"will run message pump with realtime priority\n");
		sched_setscheduler(master->kworker_task, SCHED_FIFO, &param);
	}

	return 0;
}

struct spi_message *spi_get_next_queued_message(struct spi_master *master)
{
	struct spi_message *next;
	unsigned long flags;

	
	spin_lock_irqsave(&master->queue_lock, flags);
	if (list_empty(&master->queue))
		next = NULL;
	else
		next = list_entry(master->queue.next,
				  struct spi_message, queue);
	spin_unlock_irqrestore(&master->queue_lock, flags);

	return next;
}
EXPORT_SYMBOL_GPL(spi_get_next_queued_message);

void spi_finalize_current_message(struct spi_master *master)
{
	struct spi_message *mesg;
	unsigned long flags;

	spin_lock_irqsave(&master->queue_lock, flags);
	mesg = master->cur_msg;
	master->cur_msg = NULL;

	queue_kthread_work(&master->kworker, &master->pump_messages);
	spin_unlock_irqrestore(&master->queue_lock, flags);

	mesg->state = NULL;
	if (mesg->complete)
		mesg->complete(mesg->context);
}
EXPORT_SYMBOL_GPL(spi_finalize_current_message);

static int spi_start_queue(struct spi_master *master)
{
	unsigned long flags;

	spin_lock_irqsave(&master->queue_lock, flags);

	if (master->running || master->busy) {
		spin_unlock_irqrestore(&master->queue_lock, flags);
		return -EBUSY;
	}

	master->running = true;
	master->cur_msg = NULL;
	spin_unlock_irqrestore(&master->queue_lock, flags);

	queue_kthread_work(&master->kworker, &master->pump_messages);

	return 0;
}

static int spi_stop_queue(struct spi_master *master)
{
	unsigned long flags;
	unsigned limit = 500;
	int ret = 0;

	spin_lock_irqsave(&master->queue_lock, flags);

	while ((!list_empty(&master->queue) || master->busy) && limit--) {
		spin_unlock_irqrestore(&master->queue_lock, flags);
		msleep(10);
		spin_lock_irqsave(&master->queue_lock, flags);
	}

	if (!list_empty(&master->queue) || master->busy)
		ret = -EBUSY;
	else
		master->running = false;

	spin_unlock_irqrestore(&master->queue_lock, flags);

	if (ret) {
		dev_warn(&master->dev,
			 "could not stop message queue\n");
		return ret;
	}
	return ret;
}

static int spi_destroy_queue(struct spi_master *master)
{
	int ret;

	ret = spi_stop_queue(master);

	if (ret) {
		dev_err(&master->dev, "problem destroying queue\n");
		return ret;
	}

	flush_kthread_worker(&master->kworker);
	kthread_stop(master->kworker_task);

	return 0;
}

static int spi_queued_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct spi_master *master = spi->master;
	unsigned long flags;

	spin_lock_irqsave(&master->queue_lock, flags);

	if (!master->running) {
		spin_unlock_irqrestore(&master->queue_lock, flags);
		return -ESHUTDOWN;
	}
	msg->actual_length = 0;
	msg->status = -EINPROGRESS;

	list_add_tail(&msg->queue, &master->queue);
	if (master->running && !master->busy)
		queue_kthread_work(&master->kworker, &master->pump_messages);

	spin_unlock_irqrestore(&master->queue_lock, flags);
	return 0;
}

static int spi_master_initialize_queue(struct spi_master *master)
{
	int ret;

	master->queued = true;
	master->transfer = spi_queued_transfer;

	
	ret = spi_init_queue(master);
	if (ret) {
		dev_err(&master->dev, "problem initializing queue\n");
		goto err_init_queue;
	}
	ret = spi_start_queue(master);
	if (ret) {
		dev_err(&master->dev, "problem starting queue\n");
		goto err_start_queue;
	}

	return 0;

err_start_queue:
err_init_queue:
	spi_destroy_queue(master);
	return ret;
}


static void spi_master_release(struct device *dev)
{
	struct spi_master *master;

	master = container_of(dev, struct spi_master, dev);
	kfree(master);
}

static struct class spi_master_class = {
	.name		= "spi_master",
	.owner		= THIS_MODULE,
	.dev_release	= spi_master_release,
};



struct spi_master *spi_alloc_master(struct device *dev, unsigned size)
{
	struct spi_master	*master;

	if (!dev)
		return NULL;

	master = kzalloc(size + sizeof *master, GFP_KERNEL);
	if (!master)
		return NULL;

	device_initialize(&master->dev);
	master->dev.class = &spi_master_class;
	master->dev.parent = get_device(dev);
	spi_master_set_devdata(master, &master[1]);

	return master;
}
EXPORT_SYMBOL_GPL(spi_alloc_master);

int spi_register_master(struct spi_master *master)
{
	static atomic_t		dyn_bus_id = ATOMIC_INIT((1<<15) - 1);
	struct device		*dev = master->dev.parent;
	struct boardinfo	*bi;
	int			status = -ENODEV;
	int			dynamic = 0;

	if (!dev)
		return -ENODEV;

	if (master->num_chipselect == 0)
		return -EINVAL;

	
	if (master->bus_num < 0) {
		master->bus_num = atomic_dec_return(&dyn_bus_id);
		dynamic = 1;
	}

	spin_lock_init(&master->bus_lock_spinlock);
	mutex_init(&master->bus_lock_mutex);
	master->bus_lock_flag = 0;

	dev_set_name(&master->dev, "spi%u", master->bus_num);
	status = device_add(&master->dev);
	if (status < 0)
		goto done;
	dev_dbg(dev, "registered master %s%s\n", dev_name(&master->dev),
			dynamic ? " (dynamic)" : "");

	
	if (master->transfer)
		dev_info(dev, "master is unqueued, this is deprecated\n");
	else {
		status = spi_master_initialize_queue(master);
		if (status) {
			device_unregister(&master->dev);
			goto done;
		}
	}

	mutex_lock(&board_lock);
	list_add_tail(&master->list, &spi_master_list);
	list_for_each_entry(bi, &board_list, list)
		spi_match_master_to_boardinfo(master, &bi->board_info);
	mutex_unlock(&board_lock);

	
	of_register_spi_devices(master);
done:
	return status;
}
EXPORT_SYMBOL_GPL(spi_register_master);

static int __unregister(struct device *dev, void *null)
{
	spi_unregister_device(to_spi_device(dev));
	return 0;
}

void spi_unregister_master(struct spi_master *master)
{
	int dummy;

	if (master->queued) {
		if (spi_destroy_queue(master))
			dev_err(&master->dev, "queue remove failed\n");
	}

	mutex_lock(&board_lock);
	list_del(&master->list);
	mutex_unlock(&board_lock);

	dummy = device_for_each_child(&master->dev, NULL, __unregister);
	device_unregister(&master->dev);
}
EXPORT_SYMBOL_GPL(spi_unregister_master);

int spi_master_suspend(struct spi_master *master)
{
	int ret;

	
	if (!master->queued)
		return 0;

	ret = spi_stop_queue(master);
	if (ret)
		dev_err(&master->dev, "queue stop failed\n");

	return ret;
}
EXPORT_SYMBOL_GPL(spi_master_suspend);

int spi_master_resume(struct spi_master *master)
{
	int ret;

	if (!master->queued)
		return 0;

	ret = spi_start_queue(master);
	if (ret)
		dev_err(&master->dev, "queue restart failed\n");

	return ret;
}
EXPORT_SYMBOL_GPL(spi_master_resume);

static int __spi_master_match(struct device *dev, void *data)
{
	struct spi_master *m;
	u16 *bus_num = data;

	m = container_of(dev, struct spi_master, dev);
	return m->bus_num == *bus_num;
}

struct spi_master *spi_busnum_to_master(u16 bus_num)
{
	struct device		*dev;
	struct spi_master	*master = NULL;

	dev = class_find_device(&spi_master_class, NULL, &bus_num,
				__spi_master_match);
	if (dev)
		master = container_of(dev, struct spi_master, dev);
	
	return master;
}
EXPORT_SYMBOL_GPL(spi_busnum_to_master);




int spi_setup(struct spi_device *spi)
{
	unsigned	bad_bits;
	int		status;

	bad_bits = spi->mode & ~spi->master->mode_bits;
	if (bad_bits) {
		dev_err(&spi->dev, "setup: unsupported mode bits %x\n",
			bad_bits);
		return -EINVAL;
	}

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	status = spi->master->setup(spi);

	dev_dbg(&spi->dev, "setup mode %d, %s%s%s%s"
				"%u bits/w, %u Hz max --> %d\n",
			(int) (spi->mode & (SPI_CPOL | SPI_CPHA)),
			(spi->mode & SPI_CS_HIGH) ? "cs_high, " : "",
			(spi->mode & SPI_LSB_FIRST) ? "lsb, " : "",
			(spi->mode & SPI_3WIRE) ? "3wire, " : "",
			(spi->mode & SPI_LOOP) ? "loopback, " : "",
			spi->bits_per_word, spi->max_speed_hz,
			status);

	return status;
}
EXPORT_SYMBOL_GPL(spi_setup);

static int __spi_async(struct spi_device *spi, struct spi_message *message)
{
	struct spi_master *master = spi->master;

	if ((master->flags & SPI_MASTER_HALF_DUPLEX)
			|| (spi->mode & SPI_3WIRE)) {
		struct spi_transfer *xfer;
		unsigned flags = master->flags;

		list_for_each_entry(xfer, &message->transfers, transfer_list) {
			if (xfer->rx_buf && xfer->tx_buf)
				return -EINVAL;
			if ((flags & SPI_MASTER_NO_TX) && xfer->tx_buf)
				return -EINVAL;
			if ((flags & SPI_MASTER_NO_RX) && xfer->rx_buf)
				return -EINVAL;
		}
	}

	message->spi = spi;
	message->status = -EINPROGRESS;
	return master->transfer(spi, message);
}

int spi_async(struct spi_device *spi, struct spi_message *message)
{
	struct spi_master *master = spi->master;
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&master->bus_lock_spinlock, flags);

	if (master->bus_lock_flag)
		ret = -EBUSY;
	else
		ret = __spi_async(spi, message);

	spin_unlock_irqrestore(&master->bus_lock_spinlock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(spi_async);

int spi_async_locked(struct spi_device *spi, struct spi_message *message)
{
	struct spi_master *master = spi->master;
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&master->bus_lock_spinlock, flags);

	ret = __spi_async(spi, message);

	spin_unlock_irqrestore(&master->bus_lock_spinlock, flags);

	return ret;

}
EXPORT_SYMBOL_GPL(spi_async_locked);




static void spi_complete(void *arg)
{
	complete(arg);
}

static int __spi_sync(struct spi_device *spi, struct spi_message *message,
		      int bus_locked)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;
	struct spi_master *master = spi->master;

	message->complete = spi_complete;
	message->context = &done;

	if (!bus_locked)
		mutex_lock(&master->bus_lock_mutex);

	status = spi_async_locked(spi, message);

	if (!bus_locked)
		mutex_unlock(&master->bus_lock_mutex);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
	}
	message->context = NULL;
	return status;
}

int spi_sync(struct spi_device *spi, struct spi_message *message)
{
	return __spi_sync(spi, message, 0);
}
EXPORT_SYMBOL_GPL(spi_sync);

int spi_sync_locked(struct spi_device *spi, struct spi_message *message)
{
	return __spi_sync(spi, message, 1);
}
EXPORT_SYMBOL_GPL(spi_sync_locked);

int spi_bus_lock(struct spi_master *master)
{
	unsigned long flags;

	mutex_lock(&master->bus_lock_mutex);

	spin_lock_irqsave(&master->bus_lock_spinlock, flags);
	master->bus_lock_flag = 1;
	spin_unlock_irqrestore(&master->bus_lock_spinlock, flags);

	

	return 0;
}
EXPORT_SYMBOL_GPL(spi_bus_lock);

int spi_bus_unlock(struct spi_master *master)
{
	master->bus_lock_flag = 0;

	mutex_unlock(&master->bus_lock_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(spi_bus_unlock);

#define	SPI_BUFSIZ	max(32,SMP_CACHE_BYTES)

static u8	*buf;

/**
 * spi_write_then_read - SPI synchronous write followed by read
 * @spi: device with which data will be exchanged
 * @txbuf: data to be written (need not be dma-safe)
 * @n_tx: size of txbuf, in bytes
 * @rxbuf: buffer into which data will be read (need not be dma-safe)
 * @n_rx: size of rxbuf, in bytes
 * Context: can sleep
 *
 * This performs a half duplex MicroWire style transaction with the
 * device, sending txbuf and then reading rxbuf.  The return value
 * is zero for success, else a negative errno status code.
 * This call may only be used from a context that may sleep.
 *
 * Parameters to this routine are always copied using a small buffer;
 * portable code should never use this for more than 32 bytes.
 * Performance-sensitive or bulk transfer code should instead use
 * spi_{async,sync}() calls with dma-safe buffers.
 */
int spi_write_then_read(struct spi_device *spi,
		const void *txbuf, unsigned n_tx,
		void *rxbuf, unsigned n_rx)
{
	static DEFINE_MUTEX(lock);

	int			status;
	struct spi_message	message;
	struct spi_transfer	x[2];
	u8			*local_buf;

	if ((n_tx + n_rx) > SPI_BUFSIZ)
		return -EINVAL;

	spi_message_init(&message);
	memset(x, 0, sizeof x);
	if (n_tx) {
		x[0].len = n_tx;
		spi_message_add_tail(&x[0], &message);
	}
	if (n_rx) {
		x[1].len = n_rx;
		spi_message_add_tail(&x[1], &message);
	}

	
	if (!mutex_trylock(&lock)) {
		local_buf = kmalloc(SPI_BUFSIZ, GFP_KERNEL);
		if (!local_buf)
			return -ENOMEM;
	} else
		local_buf = buf;

	memcpy(local_buf, txbuf, n_tx);
	x[0].tx_buf = local_buf;
	x[1].rx_buf = local_buf + n_tx;

	
	status = spi_sync(spi, &message);
	if (status == 0)
		memcpy(rxbuf, x[1].rx_buf, n_rx);

	if (x[0].tx_buf == buf)
		mutex_unlock(&lock);
	else
		kfree(local_buf);

	return status;
}
EXPORT_SYMBOL_GPL(spi_write_then_read);


static int __init spi_init(void)
{
	int	status;

	buf = kmalloc(SPI_BUFSIZ, GFP_KERNEL);
	if (!buf) {
		status = -ENOMEM;
		goto err0;
	}

	status = bus_register(&spi_bus_type);
	if (status < 0)
		goto err1;

	status = class_register(&spi_master_class);
	if (status < 0)
		goto err2;
	return 0;

err2:
	bus_unregister(&spi_bus_type);
err1:
	kfree(buf);
	buf = NULL;
err0:
	return status;
}

postcore_initcall(spi_init);

