/*
 *  linux/include/linux/mmc/sdhci.h - Secure Digital Host Controller Interface
 *
 *  Copyright (C) 2005-2008 Pierre Ossman, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */
#ifndef LINUX_MMC_SDHCI_H
#define LINUX_MMC_SDHCI_H

#include <linux/scatterlist.h>
#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/pm_qos.h>

struct sdhci_next {
	unsigned int sg_count;
	s32 cookie;
};

enum sdhci_power_policy {
	SDHCI_PERFORMANCE_MODE,
	SDHCI_POWER_SAVE_MODE,
};

struct sdhci_host {
	
	const char *hw_name;	

	unsigned int quirks;	

#define SDHCI_QUIRK_CLOCK_BEFORE_RESET			(1<<0)
#define SDHCI_QUIRK_FORCE_DMA				(1<<1)
#define SDHCI_QUIRK_NO_CARD_NO_RESET			(1<<2)
#define SDHCI_QUIRK_SINGLE_POWER_WRITE			(1<<3)
#define SDHCI_QUIRK_RESET_CMD_DATA_ON_IOS		(1<<4)
#define SDHCI_QUIRK_BROKEN_DMA				(1<<5)
#define SDHCI_QUIRK_BROKEN_ADMA				(1<<6)
#define SDHCI_QUIRK_32BIT_DMA_ADDR			(1<<7)
#define SDHCI_QUIRK_32BIT_DMA_SIZE			(1<<8)
#define SDHCI_QUIRK_32BIT_ADMA_SIZE			(1<<9)
#define SDHCI_QUIRK_RESET_AFTER_REQUEST			(1<<10)
#define SDHCI_QUIRK_NO_SIMULT_VDD_AND_POWER		(1<<11)
#define SDHCI_QUIRK_BROKEN_TIMEOUT_VAL			(1<<12)
#define SDHCI_QUIRK_BROKEN_SMALL_PIO			(1<<13)
#define SDHCI_QUIRK_NO_BUSY_IRQ				(1<<14)
#define SDHCI_QUIRK_BROKEN_CARD_DETECTION		(1<<15)
#define SDHCI_QUIRK_INVERTED_WRITE_PROTECT		(1<<16)
#define SDHCI_QUIRK_NONSTANDARD_CLOCK			(1<<17)
#define SDHCI_QUIRK_PIO_NEEDS_DELAY			(1<<18)
#define SDHCI_QUIRK_RESTORE_IRQS_AFTER_RESET		(1<<19)
#define SDHCI_QUIRK_FORCE_BLK_SZ_2048			(1<<20)
#define SDHCI_QUIRK_NO_MULTIBLOCK			(1<<21)
#define SDHCI_QUIRK_FORCE_1_BIT_DATA			(1<<22)
#define SDHCI_QUIRK_DELAY_AFTER_POWER			(1<<23)
#define SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK		(1<<24)
#define SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN		(1<<25)
#define SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC		(1<<26)
#define SDHCI_QUIRK_MISSING_CAPS			(1<<27)
#define SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12		(1<<28)
#define SDHCI_QUIRK_NO_HISPD_BIT			(1<<29)
#define SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC		(1<<30)
#define SDHCI_QUIRK_UNSTABLE_RO_DETECT			(1<<31)

	unsigned int quirks2;	

#define SDHCI_QUIRK2_HOST_OFF_CARD_ON			(1<<0)
#define SDHCI_QUIRK2_RDWR_TX_ACTIVE_EOT			(1<<1)
#define SDHCI_QUIRK2_SLOW_INT_CLR			(1<<2)
#define SDHCI_QUIRK2_IGNORE_CMDCRC_FOR_TUNING		(1<<3)
#define SDHCI_QUIRK2_ALWAYS_USE_BASE_CLOCK		(1<<4)
#define SDHCI_QUIRK2_USE_MAX_DISCARD_SIZE		(1<<5)
#define SDHCI_QUIRK2_IGNORE_DATATOUT_FOR_R1BCMD		(1<<6)
#define SDHCI_QUIRK2_BROKEN_PRESET_VALUE		(1<<7)
#define SDHCI_QUIRK2_USE_RESERVED_MAX_TIMEOUT		(1<<8)
#define SDHCI_QUIRK2_DIVIDE_TOUT_BY_4 (1 << 9)

#define SDHCI_QUIRK2_IGN_DATA_END_BIT_ERROR             (1<<9)
	int irq;		
	void __iomem *ioaddr;	

	const struct sdhci_ops *ops;	

	struct regulator *vmmc;	

	
	struct mmc_host *mmc;	
	u64 dma_mask;		

#if defined(CONFIG_LEDS_CLASS) || defined(CONFIG_LEDS_CLASS_MODULE)
	struct led_classdev led;	
	char led_name[32];
#endif

	spinlock_t lock;	

	int flags;		
#define SDHCI_USE_SDMA		(1<<0)	
#define SDHCI_USE_ADMA		(1<<1)	
#define SDHCI_REQ_USE_DMA	(1<<2)	
#define SDHCI_DEVICE_DEAD	(1<<3)	
#define SDHCI_SDR50_NEEDS_TUNING (1<<4)	
#define SDHCI_NEEDS_RETUNING	(1<<5)	
#define SDHCI_AUTO_CMD12	(1<<6)	
#define SDHCI_AUTO_CMD23	(1<<7)	
#define SDHCI_PV_ENABLED	(1<<8)	
#define SDHCI_SDIO_IRQ_ENABLED	(1<<9)	
#define SDHCI_HS200_NEEDS_TUNING (1<<10)	
#define SDHCI_HS400_NEEDS_TUNING (1<<11)	

	unsigned int version;	

	unsigned int max_clk;	
	unsigned int timeout_clk;	
	unsigned int clk_mul;	

	unsigned int clock;	
	u8 pwr;			

	bool runtime_suspended;	

	struct mmc_request *mrq;	
	struct mmc_command *cmd;	
	struct mmc_data *data;	
	unsigned int data_early:1;	

	struct sg_mapping_iter sg_miter;	
	unsigned int blocks;	

	int sg_count;		

	u8 *adma_desc;		
	u8 *align_buffer;	

	unsigned int adma_desc_sz; 
	unsigned int align_buf_sz; 
	unsigned int adma_max_desc; 

	dma_addr_t adma_addr;	
	dma_addr_t align_addr;	

	struct tasklet_struct card_tasklet;	
	struct tasklet_struct finish_tasklet;

	struct timer_list timer;	

	unsigned int caps;	

	unsigned int            ocr_avail_sdio;	
	unsigned int            ocr_avail_sd;
	unsigned int            ocr_avail_mmc;

	wait_queue_head_t	buf_ready_int;	
	unsigned int		tuning_done;	

	unsigned int		tuning_count;	
	unsigned int		tuning_mode;	
#define SDHCI_TUNING_MODE_1	0
	struct timer_list	tuning_timer;	

	unsigned int cpu_dma_latency_us;
	struct pm_qos_request pm_qos_req_dma;
	unsigned int pm_qos_timeout_us;         
	struct device_attribute pm_qos_tout;

	struct sdhci_next next_data;
	ktime_t data_start_time;
	struct mutex ios_mutex;
	enum sdhci_power_policy power_policy;

	bool irq_enabled; 
	bool async_int_supp;  
	bool disable_sdio_irq_deferred; 
	u32 auto_cmd_err_sts;
	unsigned long private[0] ____cacheline_aligned;
	int disable_sdcard_uhs;
};
#endif 
