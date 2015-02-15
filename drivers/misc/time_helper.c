#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/time_helper.h>

MODULE_LICENSE("Dual BSD/GPL");

struct time_helper {
    struct cdev dev;
};

static int time_helper_major = 0;
static int time_helper_minor = 0;
static int time_helper_nr_devs = 1;

static struct class* time_helper_class = NULL;
static struct time_helper *time_helper_dev=NULL;

static long time_helper_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int time_helper_open(struct inode* inode, struct file* filp);
static int time_helper_release(struct inode* inode, struct file* filp);

static struct file_operations time_helper_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = time_helper_ioctl,
    .open = time_helper_open,
    .release = time_helper_release,
};

static int time_helper_open(struct inode* inode, struct file* filp)
{
    struct time_helper *dev;

    dev = container_of(inode->i_cdev, struct time_helper, dev);
    filp->private_data = dev;

    return 0;
}

static int time_helper_release(struct inode* inode, struct file* filp)
{
    return 0;
}

static long time_helper_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int retval = 0;

    if (_IOC_TYPE(cmd) != TIME_HELPER_IOC_MAGIC) return -ENOTTY;
    if (_IOC_NR(cmd) > TIME_HELPER_IOC_MAXNR) return -ENOTTY;

    switch (cmd) {
    case TIME_HELPER_IOCRESET:
        break;
    case TIME_HELPER_IOCXMONOTONIC2REALTIME:
        {
            struct timespec mono_ts;
            struct timespec boot_time;

            if (copy_from_user(&mono_ts, (struct timespec __user *)arg, sizeof(struct timespec))) {
                printk(KERN_ALERT "copy_from_user fail\n");
                return -EFAULT;
            }
            getboottime(&boot_time);
            monotonic_to_bootbased(&mono_ts);
            mono_ts = timespec_add(mono_ts, boot_time);
            if (copy_to_user((struct timespec __user *)arg, &mono_ts, sizeof(struct timespec))) {
                printk(KERN_ALERT "copy_to_user fail\n");
                return -EFAULT;
            }
        }
        break;
    default:
        return -ENOTTY;
    }

    return retval;
}

static int  __time_helper_setup_dev(struct time_helper* dev)
{
    int err;
    dev_t devno = MKDEV(time_helper_major, time_helper_minor);

    memset(dev, 0, sizeof(struct time_helper));

    cdev_init(&(dev->dev), &time_helper_fops);
    dev->dev.owner = THIS_MODULE;
    dev->dev.ops = &time_helper_fops;

    err = cdev_add(&(dev->dev), devno, 1);
    if(err) {
        return err;
    }
    return 0;
}

static int time_helper_init(void)
{
    int err=-1;
    dev_t dev=0;

    if (time_helper_major) {
        dev = MKDEV(time_helper_major, time_helper_minor);
        err = register_chrdev_region(dev, time_helper_nr_devs, "time_helper");
    } else {
        err = alloc_chrdev_region(&dev, time_helper_minor, time_helper_nr_devs, "time_helper");
        time_helper_major = MAJOR(dev);
    }
    if (err < 0) {
        printk(KERN_WARNING "time_helper: can't get major %d\n", time_helper_major);
        goto fail;
    }

    time_helper_dev = (struct time_helper *)kmalloc(sizeof(struct time_helper), GFP_KERNEL);
    if(!time_helper_dev) {
        err = -ENOMEM;
        printk(KERN_ALERT"Failed to alloc hello_dev.\n");
        goto unregister;
    }

    err = __time_helper_setup_dev(time_helper_dev);
    if(err) {
        printk(KERN_ALERT"Failed to setup dev: %d.\n", err);
        goto cleanup;
    }

    time_helper_class = class_create(THIS_MODULE, TIME_HELPER_DEVICE_CLASS_NAME);
    if(IS_ERR(time_helper_class)) {
        err = PTR_ERR(time_helper_class);
        printk(KERN_ALERT"Failed to create time_helper class.\n");
        goto destroy_cdev;
    }

    {
        struct device* temp = NULL;

        temp = device_create(time_helper_class, NULL, dev, "%s", TIME_HELPER_DEVICE_FILE_NAME);
        if(IS_ERR(temp)) {
            err = PTR_ERR(temp);
            printk(KERN_ALERT"Failed to create time_helper device.");
            goto destroy_class;
        }
    }

    return 0;

destroy_class:
    class_destroy(time_helper_class);
destroy_cdev:
    cdev_del(&(time_helper_dev->dev));
cleanup:
    kfree(time_helper_dev);
unregister:
    unregister_chrdev_region(MKDEV(time_helper_major, time_helper_minor), time_helper_nr_devs);
fail:
    return err;
}

static void time_helper_exit(void)
{
    dev_t devno = MKDEV(time_helper_major, time_helper_minor);

    if(time_helper_class) {
        device_destroy(time_helper_class, MKDEV(time_helper_major, time_helper_minor));
        class_destroy(time_helper_class);
    }

    if(time_helper_dev) {
        cdev_del(&(time_helper_dev->dev));
        kfree(time_helper_dev);
    }

    unregister_chrdev_region(devno, time_helper_nr_devs);
}

module_init(time_helper_init);
module_exit(time_helper_exit);
