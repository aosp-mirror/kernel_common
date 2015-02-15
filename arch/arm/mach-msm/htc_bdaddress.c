/* arch/arm/mach-msm/htc_bluetooth.c
 *
 * Code to extract Bluetooth bd_address information
 * from ATAG set up by the bootloader.
 *
 * Copyright (C) 2010 HTC Corporation
 * Author:Yomin Lin <yomin_lin@htc.com>
 * Author:Allen Ou <allen_ou@htc.com>
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <asm/setup.h>
#include <mach/htc_bdaddress.h>
#include <linux/of.h>

#define ATAG_BT_DEBUG

#define MAX_BT_SIZE 0x8U

#define CALIBRATION_DATA_PATH "/calibration_data"
#define BT_FLASH_DATA "bt_flash"

static unsigned char bt_bd_ram[MAX_BT_SIZE];
static char bdaddress[20];

static unsigned char *get_bt_bd_ram(void)
{
     struct device_node *offset = of_find_node_by_path(CALIBRATION_DATA_PATH);
     int p_size;
     unsigned char *p_data;
#ifdef ATAG_BT_DEBUG
     unsigned int i;
#endif

     p_size = 0;
     p_data = NULL;
     if (offset) {
          
          p_data = (unsigned char*) of_get_property(offset, BT_FLASH_DATA, &p_size);
#ifdef ATAG_BT_DEBUG
          if (p_data) {
          	pr_debug(KERN_INFO "BT addr:");
               for (i = 0; i < p_size; ++i)
                   pr_debug("%02x ", p_data[i]);
          }
#endif
     }
        if (p_data != NULL)
            memcpy(bt_bd_ram, p_data, p_size);

	return (bt_bd_ram);
}

#if 0 
static int __init parse_tag_bt(const struct tag *tag)
{
	unsigned char *dptr = (unsigned char *)(&tag->u);
	unsigned size;
	#ifdef ATAG_BT_DEBUG
    unsigned i;
	#endif

	size = min((tag->hdr.size-2)*sizeof(__u32), MAX_BT_SIZE);
	memcpy((void *)bt_bd_ram, (void *)dptr, size);

	#ifdef ATAG_BT_DEBUG
	printk(KERN_INFO "BT Data size= %d, 0x%x,",
			tag->hdr.size, tag->hdr.tag);

	for (i = 0; i < size; i++)
		printk(KERN_INFO "%02x,", bt_bd_ram[i]);
	#endif

	return 0;
}
__tagtable(ATAG_BLUETOOTH, parse_tag_bt);
#endif

void bt_export_bd_address(void)
{
	unsigned char cTemp[6];

	memcpy(cTemp, get_bt_bd_ram(), 6);
        
	sprintf(bdaddress, "%02x:%02x:%02x:%02x:%02x:%02x",
			cTemp[0], cTemp[1], cTemp[2],
			cTemp[3], cTemp[4], cTemp[5]);

        printk(KERN_INFO "fd=%02x, apply=%02x\n", cTemp[2]+1, cTemp[5]+2);
        printk(KERN_INFO "fd=%02x, state=%02x\n", cTemp[4]+2, cTemp[1]+1);
        printk(KERN_INFO "fd=%02x, status=%02x\n", cTemp[0]+1, cTemp[3]+2);

}
module_param_string(bdaddress, bdaddress, sizeof(bdaddress), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bdaddress, "BT MAC ADDRESS");

