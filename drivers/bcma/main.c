/*
 * Broadcom specific AMBA
 * Bus subsystem
 *
 * Licensed under the GNU/GPL. See COPYING for details.
 */

#include "bcma_private.h"
#include <linux/module.h>
#include <linux/bcma/bcma.h>
#include <linux/slab.h>

MODULE_DESCRIPTION("Broadcom's specific AMBA driver");
MODULE_LICENSE("GPL");

/* contains the number the next bus should get. */
static unsigned int bcma_bus_next_num = 0;

/* bcma_buses_mutex locks the bcma_bus_next_num */
static DEFINE_MUTEX(bcma_buses_mutex);

static int bcma_bus_match(struct device *dev, struct device_driver *drv);
static int bcma_device_probe(struct device *dev);
static int bcma_device_remove(struct device *dev);
static int bcma_device_uevent(struct device *dev, struct kobj_uevent_env *env);

static ssize_t manuf_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bcma_device *core = container_of(dev, struct bcma_device, dev);
	return sprintf(buf, "0x%03X\n", core->id.manuf);
}
static ssize_t id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bcma_device *core = container_of(dev, struct bcma_device, dev);
	return sprintf(buf, "0x%03X\n", core->id.id);
}
static ssize_t rev_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bcma_device *core = container_of(dev, struct bcma_device, dev);
	return sprintf(buf, "0x%02X\n", core->id.rev);
}
static ssize_t class_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct bcma_device *core = container_of(dev, struct bcma_device, dev);
	return sprintf(buf, "0x%X\n", core->id.class);
}
static struct device_attribute bcma_device_attrs[] = {
	__ATTR_RO(manuf),
	__ATTR_RO(id),
	__ATTR_RO(rev),
	__ATTR_RO(class),
	__ATTR_NULL,
};

static struct bus_type bcma_bus_type = {
	.name		= "bcma",
	.match		= bcma_bus_match,
	.probe		= bcma_device_probe,
	.remove		= bcma_device_remove,
	.uevent		= bcma_device_uevent,
	.dev_attrs	= bcma_device_attrs,
};

struct bcma_device *bcma_find_core(struct bcma_bus *bus, u16 coreid)
{
	struct bcma_device *core;

	list_for_each_entry(core, &bus->cores, list) {
		if (core->id.id == coreid)
			return core;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(bcma_find_core);

static void bcma_release_core_dev(struct device *dev)
{
	struct bcma_device *core = container_of(dev, struct bcma_device, dev);
	if (core->io_addr)
		iounmap(core->io_addr);
	if (core->io_wrap)
		iounmap(core->io_wrap);
	kfree(core);
}

static int bcma_register_cores(struct bcma_bus *bus)
{
	struct bcma_device *core;
	int err, dev_id = 0;

	list_for_each_entry(core, &bus->cores, list) {
		/* We support that cores ourself */
		switch (core->id.id) {
		case BCMA_CORE_CHIPCOMMON:
		case BCMA_CORE_PCI:
		case BCMA_CORE_PCIE:
		case BCMA_CORE_MIPS_74K:
			continue;
		}

		core->dev.release = bcma_release_core_dev;
		core->dev.bus = &bcma_bus_type;
		dev_set_name(&core->dev, "bcma%d:%d", bus->num, dev_id);

		switch (bus->hosttype) {
		case BCMA_HOSTTYPE_PCI:
			core->dev.parent = &bus->host_pci->dev;
			core->dma_dev = &bus->host_pci->dev;
			core->irq = bus->host_pci->irq;
			break;
		case BCMA_HOSTTYPE_SOC:
			core->dev.dma_mask = &core->dev.coherent_dma_mask;
			core->dma_dev = &core->dev;
			break;
		case BCMA_HOSTTYPE_SDIO:
			break;
		}

		err = device_register(&core->dev);
		if (err) {
			pr_err("Could not register dev for core 0x%03X\n",
			       core->id.id);
			continue;
		}
		core->dev_registered = true;
		dev_id++;
	}

	return 0;
}

static void bcma_unregister_cores(struct bcma_bus *bus)
{
	struct bcma_device *core;

	list_for_each_entry(core, &bus->cores, list) {
		if (core->dev_registered)
			device_unregister(&core->dev);
	}
}

int __devinit bcma_bus_register(struct bcma_bus *bus)
{
	int err;
	struct bcma_device *core;

	mutex_lock(&bcma_buses_mutex);
	bus->num = bcma_bus_next_num++;
	mutex_unlock(&bcma_buses_mutex);

	/* Scan for devices (cores) */
	err = bcma_bus_scan(bus);
	if (err) {
		pr_err("Failed to scan: %d\n", err);
		return -1;
	}

	/* Init CC core */
	core = bcma_find_core(bus, BCMA_CORE_CHIPCOMMON);
	if (core) {
		bus->drv_cc.core = core;
		bcma_core_chipcommon_init(&bus->drv_cc);
	}

	/* Init MIPS core */
	core = bcma_find_core(bus, BCMA_CORE_MIPS_74K);
	if (core) {
		bus->drv_mips.core = core;
		bcma_core_mips_init(&bus->drv_mips);
	}

	/* Init PCIE core */
	core = bcma_find_core(bus, BCMA_CORE_PCIE);
	if (core) {
		bus->drv_pci.core = core;
		bcma_core_pci_init(&bus->drv_pci);
	}

	/* Try to get SPROM */
	err = bcma_sprom_get(bus);
	if (err == -ENOENT) {
		pr_err("No SPROM available\n");
	} else if (err)
		pr_err("Failed to get SPROM: %d\n", err);

	/* Register found cores */
	bcma_register_cores(bus);

	pr_info("Bus registered\n");

	return 0;
}

void bcma_bus_unregister(struct bcma_bus *bus)
{
	bcma_unregister_cores(bus);
}

int __init bcma_bus_early_register(struct bcma_bus *bus,
				   struct bcma_device *core_cc,
				   struct bcma_device *core_mips)
{
	int err;
	struct bcma_device *core;
	struct bcma_device_id match;

	bcma_init_bus(bus);

	match.manuf = BCMA_MANUF_BCM;
	match.id = BCMA_CORE_CHIPCOMMON;
	match.class = BCMA_CL_SIM;
	match.rev = BCMA_ANY_REV;

	/* Scan for chip common core */
	err = bcma_bus_scan_early(bus, &match, core_cc);
	if (err) {
		pr_err("Failed to scan for common core: %d\n", err);
		return -1;
	}

	match.manuf = BCMA_MANUF_MIPS;
	match.id = BCMA_CORE_MIPS_74K;
	match.class = BCMA_CL_SIM;
	match.rev = BCMA_ANY_REV;

	/* Scan for mips core */
	err = bcma_bus_scan_early(bus, &match, core_mips);
	if (err) {
		pr_err("Failed to scan for mips core: %d\n", err);
		return -1;
	}

	/* Init CC core */
	core = bcma_find_core(bus, BCMA_CORE_CHIPCOMMON);
	if (core) {
		bus->drv_cc.core = core;
		bcma_core_chipcommon_init(&bus->drv_cc);
	}

	/* Init MIPS core */
	core = bcma_find_core(bus, BCMA_CORE_MIPS_74K);
	if (core) {
		bus->drv_mips.core = core;
		bcma_core_mips_init(&bus->drv_mips);
	}

	pr_info("Early bus registered\n");

	return 0;
}

#ifdef CONFIG_PM
int bcma_bus_suspend(struct bcma_bus *bus)
{
	struct bcma_device *core;

	list_for_each_entry(core, &bus->cores, list) {
		struct device_driver *drv = core->dev.driver;
		if (drv) {
			struct bcma_driver *adrv = container_of(drv, struct bcma_driver, drv);
			if (adrv->suspend)
				adrv->suspend(core);
		}
	}
	return 0;
}

int bcma_bus_resume(struct bcma_bus *bus)
{
	struct bcma_device *core;

	/* Init CC core */
	core = bcma_find_core(bus, BCMA_CORE_CHIPCOMMON);
	if (core) {
		bus->drv_cc.setup_done = false;
		bcma_core_chipcommon_init(&bus->drv_cc);
	}

	list_for_each_entry(core, &bus->cores, list) {
		struct device_driver *drv = core->dev.driver;
		if (drv) {
			struct bcma_driver *adrv = container_of(drv, struct bcma_driver, drv);
			if (adrv->resume)
				adrv->resume(core);
		}
	}

	return 0;
}
#endif

int __bcma_driver_register(struct bcma_driver *drv, struct module *owner)
{
	drv->drv.name = drv->name;
	drv->drv.bus = &bcma_bus_type;
	drv->drv.owner = owner;

	return driver_register(&drv->drv);
}
EXPORT_SYMBOL_GPL(__bcma_driver_register);

void bcma_driver_unregister(struct bcma_driver *drv)
{
	driver_unregister(&drv->drv);
}
EXPORT_SYMBOL_GPL(bcma_driver_unregister);

static int bcma_bus_match(struct device *dev, struct device_driver *drv)
{
	struct bcma_device *core = container_of(dev, struct bcma_device, dev);
	struct bcma_driver *adrv = container_of(drv, struct bcma_driver, drv);
	const struct bcma_device_id *cid = &core->id;
	const struct bcma_device_id *did;

	for (did = adrv->id_table; did->manuf || did->id || did->rev; did++) {
	    if ((did->manuf == cid->manuf || did->manuf == BCMA_ANY_MANUF) &&
		(did->id == cid->id || did->id == BCMA_ANY_ID) &&
		(did->rev == cid->rev || did->rev == BCMA_ANY_REV) &&
		(did->class == cid->class || did->class == BCMA_ANY_CLASS))
			return 1;
	}
	return 0;
}

static int bcma_device_probe(struct device *dev)
{
	struct bcma_device *core = container_of(dev, struct bcma_device, dev);
	struct bcma_driver *adrv = container_of(dev->driver, struct bcma_driver,
					       drv);
	int err = 0;

	if (adrv->probe)
		err = adrv->probe(core);

	return err;
}

static int bcma_device_remove(struct device *dev)
{
	struct bcma_device *core = container_of(dev, struct bcma_device, dev);
	struct bcma_driver *adrv = container_of(dev->driver, struct bcma_driver,
					       drv);

	if (adrv->remove)
		adrv->remove(core);

	return 0;
}

static int bcma_device_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct bcma_device *core = container_of(dev, struct bcma_device, dev);

	return add_uevent_var(env,
			      "MODALIAS=bcma:m%04Xid%04Xrev%02Xcl%02X",
			      core->id.manuf, core->id.id,
			      core->id.rev, core->id.class);
}

static int __init bcma_modinit(void)
{
	int err;

	err = bus_register(&bcma_bus_type);
	if (err)
		return err;

#ifdef CONFIG_BCMA_HOST_PCI
	err = bcma_host_pci_init();
	if (err) {
		pr_err("PCI host initialization failed\n");
		err = 0;
	}
#endif

	return err;
}
fs_initcall(bcma_modinit);

static void __exit bcma_modexit(void)
{
#ifdef CONFIG_BCMA_HOST_PCI
	bcma_host_pci_exit();
#endif
	bus_unregister(&bcma_bus_type);
}
module_exit(bcma_modexit)
