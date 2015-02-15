/* arch/arm/mach-msm/htc_wifi_nvs.c
 *
 * Code to extract WiFi calibration information from ATAG set up 
 * by the bootloader.
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Dmitry Shmidt <dimitrysh@google.com>
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/export.h>
#include <linux/proc_fs.h>

#include <asm/setup.h>

#include <linux/of.h>

/* configuration tags specific to msm */
//#define ATAG_MSM_WIFI	0x57494649 /* MSM WiFi */

#define NVS_MAX_SIZE	0x800U
#define NVS_LEN_OFFSET	0x0C
#define NVS_DATA_OFFSET	0x40


#define CALIBRATION_DATA_PATH "/calibration_data"
#define WIFI_FLASH_DATA "wifi_eeprom"

static unsigned char wifi_nvs_ram[NVS_MAX_SIZE];
static struct proc_dir_entry *wifi_calibration;
static struct proc_dir_entry *wifi_data;

unsigned char *get_wifi_nvs_ram( void )
{
     struct device_node *offset = of_find_node_by_path(CALIBRATION_DATA_PATH);
     int p_size;
     unsigned char *p_data;
#ifdef MSM_WIFI_DEBUG
     unsigned int i;
#endif

     p_size = 0;
     p_data = NULL;
     if (offset) {
          /* of_get_property 會回傳property的address，並把長度填入*p_size */
          p_data = (unsigned char*) of_get_property(offset, WIFI_FLASH_DATA, &p_size);
#ifdef MSM_WIFI_DEBUG
          if (p_data) {
               for (i = 0; i < p_size; ++i)
                   pr_debug("%02x ", p_data[i]);
          }
#endif
     }
        if (p_data != NULL)
            memcpy(wifi_nvs_ram, p_data, p_size);

	return( wifi_nvs_ram );
}
EXPORT_SYMBOL(get_wifi_nvs_ram);

unsigned char* wlan_random_mac(unsigned char *set_mac_addr)
{
    static unsigned char mac_addr[6]={0,0,0,0,0,0};
    if(set_mac_addr != NULL){
        mac_addr[0]=set_mac_addr[0];
        mac_addr[1]=set_mac_addr[1];
        mac_addr[2]=set_mac_addr[2];
        mac_addr[3]=set_mac_addr[3];
        mac_addr[4]=set_mac_addr[4];
        mac_addr[5]=set_mac_addr[5];
    }
    return mac_addr;
}
EXPORT_SYMBOL(wlan_random_mac);

#if 0
static int __init parse_tag_msm_wifi(const struct tag *tag)
{
	unsigned char *dptr = (unsigned char *)(&tag->u);
	unsigned size;
#ifdef ATAG_MSM_WIFI_DEBUG
	unsigned i;
#endif

	size = min((tag->hdr.size - 2) * sizeof(__u32), NVS_MAX_SIZE);
#ifdef ATAG_MSM_WIFI_DEBUG
	printk("WiFi Data size = %d , 0x%x\n", tag->hdr.size, tag->hdr.tag);
	for(i=0;( i < size );i++) {
		printk("%02x ", *dptr++);
	}
#endif	
	memcpy(wifi_nvs_ram, dptr, size);
	return 0;
}

__tagtable(ATAG_MSM_WIFI, parse_tag_msm_wifi);
#endif

static unsigned wifi_get_nvs_size( void )
{
	unsigned char *ptr;
	unsigned len;

	ptr = get_wifi_nvs_ram();
	/* Size in format LE assumed */
	memcpy(&len, ptr + NVS_LEN_OFFSET, sizeof(len));
	len = min(len, (NVS_MAX_SIZE - NVS_DATA_OFFSET));
	return len;
}

int wifi_calibration_size_set(void)
{
	if (wifi_calibration != NULL)
		wifi_calibration->size = wifi_get_nvs_size();
	return 0;
}

static int wifi_calibration_read_proc(char *page, char **start, off_t off,
					int count, int *eof, void *data)
{
	unsigned char *ptr;
	unsigned len;

	ptr = get_wifi_nvs_ram();
	len = min(wifi_get_nvs_size(), (unsigned)count);
	memcpy(page, ptr + NVS_DATA_OFFSET, len);
	return len;
}

static int wifi_data_read_proc(char *page, char **start, off_t off,
					int count, int *eof, void *data)
{
	unsigned char *ptr;

	ptr = get_wifi_nvs_ram();
	memcpy(page, ptr, NVS_DATA_OFFSET);
	return NVS_DATA_OFFSET;
}

static int __init wifi_nvs_init(void)
{
	wifi_calibration = create_proc_entry("calibration", 0444, NULL);
	if (wifi_calibration != NULL) {
		wifi_calibration->size = wifi_get_nvs_size();
		wifi_calibration->read_proc = wifi_calibration_read_proc;
		wifi_calibration->write_proc = NULL;
	}

	wifi_data = create_proc_entry("wifi_data", 0444, NULL);
	if (wifi_data != NULL) {
		wifi_data->size = NVS_DATA_OFFSET;
		wifi_data->read_proc = wifi_data_read_proc;
		wifi_data->write_proc = NULL;
	}
	return 0;
}

late_initcall(wifi_nvs_init);
