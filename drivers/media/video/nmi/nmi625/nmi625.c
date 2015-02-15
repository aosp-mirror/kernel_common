/*****************************************************************************
 Copyright(c) 2010 NMI Inc. All Rights Reserved
 
 File name : nmi_hw.c
 
 Description : NM625 host interface
 
 History : 
 ----------------------------------------------------------------------
 2010/05/17 	ssw		initial
*******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <mach/gpio.h>

#include "nmi625.h"
#include "nmi625-i2c.h"

//#define GPIO_ISDBT_RSTn 85
#define NMI_NAME		"dmb"
#define NMI625_HW_CHIP_ID_CHECK

int i2c_init(void);

int dmb_open (struct inode *inode, struct file *filp);
long dmb_ioctl (struct file *filp, unsigned int cmd, unsigned long arg);
int dmb_release (struct inode *inode, struct file *filp);

/* GPIO Setting */
void dmb_hw_setting(void);
void dmb_hw_init(void);
void dmb_hw_deinit(void);
void dmb_hw_rst(unsigned long, unsigned long);

/* GPIO Setting For TV */
//unsigned int gpio_isdbt_pwr_en;
//unsigned int gpio_isdbt_rstn;	
//unsigned int gpio_atv_rstn;

extern unsigned int HWREV;

//extern int oneseg_select_antenna(unsigned char data);

static struct file_operations dmb_fops = 
{
	.owner		= THIS_MODULE,
	.unlocked_ioctl		= dmb_ioctl,
	.open		= dmb_open,
	.release	= dmb_release,
};

static struct miscdevice nmi_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = NMI_NAME,
    .fops = &dmb_fops,
};

int dmb_open (struct inode *inode, struct file *filp)
{
	DMB_OPEN_INFO_T *hOpen = NULL;

	printk("dmb open\n");

	hOpen = (DMB_OPEN_INFO_T *)kmalloc(sizeof(DMB_OPEN_INFO_T), GFP_KERNEL);
	filp->private_data = hOpen;

	oneseg_power(1);
	nmi625_i2c_init();

	return 0;
}

int dmb_release (struct inode *inode, struct file *filp)
{
//	DMB_OPEN_INFO_T *hOpen;

	printk("dmb release \n");

    	nmi625_i2c_deinit();
    	oneseg_power(0);
	return 0;
}

long dmb_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
	long res = 1;

	ioctl_info info;

	int	err = 0, size;

	if(_IOC_TYPE(cmd) != IOCTL_MAGIC) return -EINVAL;
	if(_IOC_NR(cmd) >= IOCTL_MAXNR) return -EINVAL;

	size = _IOC_SIZE(cmd);

	if(size) {
		if(_IOC_DIR(cmd) & _IOC_READ)
			err = access_ok(VERIFY_WRITE, (void *) arg, size);
		else if(_IOC_DIR(cmd) & _IOC_WRITE)
			err = access_ok(VERIFY_READ, (void *) arg, size);
		if(!err) {
			printk("%s : Wrong argument! cmd(0x%08x) _IOC_NR(%d) _IOC_TYPE(0x%x) _IOC_SIZE(%d) _IOC_DIR(0x%x)\n", 
			__FUNCTION__, cmd, _IOC_NR(cmd), _IOC_TYPE(cmd), _IOC_SIZE(cmd), _IOC_DIR(cmd));
			return err;
		}
	}

	switch(cmd) 
	{
		case IOCTL_DMB_RESET:
			if (copy_from_user(&info, (void *)arg, size))
				return -EFAULT;
			// reset the chip with the info[0], if info[0] is zero then isdbt reset, 
			// if info[0] is one then atv reset.
			printk("DMB_RESET enter.., info.buf[0] %d, info.buf[1] %d\n", info.buf[0], info.buf[1]);
			dmb_hw_rst(info.buf[0], info.buf[1]);
			break;
		case IOCTL_DMB_INIT:
			nmi625_i2c_init();
			break;
		case IOCTL_DMB_BYTE_READ:
		
			break;
		case IOCTL_DMB_WORD_READ:
			
			break;
		case IOCTL_DMB_LONG_READ:
			
			break;
		case IOCTL_DMB_BULK_READ:
			if (copy_from_user(&info, (void *)arg, size))
				return -EFAULT;
			res = nmi625_i2c_read(0, (u16)info.dev_addr, (u8 *)(info.buf), info.size);
			if (copy_to_user((void *)arg, (void *)&info, size))
				return -EFAULT;
			break;
		case IOCTL_DMB_BYTE_WRITE:
			//copy_from_user((void *)&info, (void *)arg, size);
			//res = BBM_BYTE_WRITE(0, (u16)info.buff[0], (u8)info.buff[1]);
			break;
		case IOCTL_DMB_WORD_WRITE:
			
			break;
		case IOCTL_DMB_LONG_WRITE:
			
			break;
		case IOCTL_DMB_BULK_WRITE:
			if (copy_from_user(&info, (void *)arg, size))
				return -EFAULT;
			nmi625_i2c_write(0, (u16)info.dev_addr, (u8 *)(info.buf), info.size);
			break;
		case IOCTL_DMB_TUNER_SELECT:
			
			break;
		case IOCTL_DMB_TUNER_READ:
			
			break;
		case IOCTL_DMB_TUNER_WRITE:
			
			break;
		case IOCTL_DMB_TUNER_SET_FREQ:
			
			break;
		case IOCTL_DMB_POWER_ON:
			printk("DMB_POWER_ON enter..\n");
			dmb_hw_init();
			break;
		case IOCTL_DMB_POWER_OFF:
			printk("DMB_POWER_OFF enter..\n");
			//nmi_i2c_deinit(NULL);
			dmb_hw_deinit();
			break;
///+HTC ADD for antenna control 
/*
		case IOCTL_DMB_ANTENNA_SELECT:
			printk("DMB_ANTENNA_SELECT enter..\n");
			if (copy_from_user(&info, (void *)arg, size))
				return -EFAULT;
			if (info.buf[0] > 0X0F)
			{
				printk("DMB_ANTENNA_SELECT: select antenna %d > 0X0F !!!\n", info.buf[0]);
				return -EFAULT;
			}
		    oneseg_select_antenna(info.buf[0]);
			break;
*/
///-
		default:
			printk("dmb_ioctl : Undefined ioctl command\n");
			res = 1;
			break;
	}
	return res;
}


void dmb_hw_setting(void)
{
//	gpio_isdbt_pwr_en = GPIO_ISDBT_PWR_EN;
//		gpio_isdbt_rstn = GPIO_ISDBT_RSTn;

//	gpio_request(gpio_isdbt_pwr_en,"ISDBT_EN");
//	udelay(50);
//	gpio_direction_output(gpio_isdbt_pwr_en,1);

//	gpio_request(gpio_isdbt_rstn,"ISDBT_RSTn");
//	    udelay(50);
//	gpio_direction_output(gpio_isdbt_rstn,1);

}


void dmb_hw_setting_gpio_free(void)
{
    /* GPIO free*/
//	gpio_free(gpio_atv_rstn);
//	gpio_free(gpio_isdbt_rstn);
//	gpio_free(gpio_isdbt_pwr_en);	
}
	
void dmb_hw_init(void)
{
//	gpio_set_value(gpio_isdbt_pwr_en, 1);  // ISDBT EN Enable

//	if (HWREV == 16)          // Derek: above rev1.0
//    	{
//		gpio_set_value(GPIO_TV_CLK_EN, 1); //Derek.ji 2010.017.27 NMI 26M clk Ensable			
//    	}
//	mdelay(1);
}

void dmb_hw_rst(unsigned long _arg1, unsigned long _arg2)
{
	// _arg1 is the chip selection. if 0, then isdbt. if 1, then atv.
	// _arg2 is the reset polarity. if 0, then low. if 1, then high.
//	if(0 == _arg1){
//		gpio_set_value(gpio_isdbt_rstn, _arg2);  
//	}else {
//		gpio_set_value(gpio_atv_rstn, _arg2); 
//	}
}

void dmb_hw_deinit(void)
{
//	gpio_set_value(gpio_isdbt_pwr_en, 0);  // ISDBT EN Disable

//    if (HWREV == 16)          // Derek: above rev1.0
//    	{
//			gpio_set_value(GPIO_TV_CLK_EN, 0);	//Derek.ji 2010.017.27 NMI 26M clk Disable					
//	}
}

int dmb_init(void)
{
	int result;

	printk("dmb dmb_init \n");

	/*misc device registration*/
	result = misc_register(&nmi_misc_device);
        if (result) {
                printk(KERN_ERR "[ISDB-T]: can't regist nmi device\n");
                return result;
        }

	/* nmi i2c init	*/
	//nmi625_i2c_init();
	//nmi625_i2c_read_chip_id();

	return 0;
}

void dmb_exit(void)
{
	printk("dmb dmb_exit \n");

//	nmi625_i2c_deinit();

//	dmb_hw_deinit();

	/*misc device deregistration*/
	misc_deregister(&nmi_misc_device);
}

module_init(dmb_init);
module_exit(dmb_exit);

MODULE_LICENSE("Dual BSD/GPL");

