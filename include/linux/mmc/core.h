/*
 *  linux/include/linux/mmc/core.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef LINUX_MMC_CORE_H
#define LINUX_MMC_CORE_H

#ifdef __KERNEL__
#include <linux/interrupt.h>
#include <linux/completion.h>

struct request;
struct mmc_data;
struct mmc_request;

struct mmc_command {
	u32			opcode;
	u32			arg;
#define MMC_CMD23_ARG_REL_WR	(1 << 31)
#define MMC_CMD23_ARG_PACKED	((0 << 31) | (1 << 30))
#define MMC_CMD23_ARG_TAG_REQ	(1 << 29)
	u32			resp[4];
	unsigned int		flags;		
#endif 
#define MMC_RSP_PRESENT	(1 << 0)
#define MMC_RSP_136	(1 << 1)		
#define MMC_RSP_CRC	(1 << 2)		
#define MMC_RSP_BUSY	(1 << 3)		
#define MMC_RSP_OPCODE	(1 << 4)		

#define MMC_CMD_MASK	(3 << 5)		
#define MMC_CMD_AC	(0 << 5)
#define MMC_CMD_ADTC	(1 << 5)
#define MMC_CMD_BC	(2 << 5)
#define MMC_CMD_BCR	(3 << 5)

#define MMC_RSP_SPI_S1	(1 << 7)		
#define MMC_RSP_SPI_S2	(1 << 8)		
#define MMC_RSP_SPI_B4	(1 << 9)		
#define MMC_RSP_SPI_BUSY (1 << 10)		

#define MMC_RSP_NONE	(0)
#define MMC_RSP_R1	(MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R1B	(MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE|MMC_RSP_BUSY)
#define MMC_RSP_R2	(MMC_RSP_PRESENT|MMC_RSP_136|MMC_RSP_CRC)
#define MMC_RSP_R3	(MMC_RSP_PRESENT)
#define MMC_RSP_R4	(MMC_RSP_PRESENT)
#define MMC_RSP_R5	(MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R6	(MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)
#define MMC_RSP_R7	(MMC_RSP_PRESENT|MMC_RSP_CRC|MMC_RSP_OPCODE)

#ifdef __KERNEL__
#define mmc_resp_type(cmd)	((cmd)->flags & (MMC_RSP_PRESENT|MMC_RSP_136|MMC_RSP_CRC|MMC_RSP_BUSY|MMC_RSP_OPCODE))

#define MMC_RSP_SPI_R1	(MMC_RSP_SPI_S1)
#define MMC_RSP_SPI_R1B	(MMC_RSP_SPI_S1|MMC_RSP_SPI_BUSY)
#define MMC_RSP_SPI_R2	(MMC_RSP_SPI_S1|MMC_RSP_SPI_S2)
#define MMC_RSP_SPI_R3	(MMC_RSP_SPI_S1|MMC_RSP_SPI_B4)
#define MMC_RSP_SPI_R4	(MMC_RSP_SPI_S1|MMC_RSP_SPI_B4)
#define MMC_RSP_SPI_R5	(MMC_RSP_SPI_S1|MMC_RSP_SPI_S2)
#define MMC_RSP_SPI_R7	(MMC_RSP_SPI_S1|MMC_RSP_SPI_B4)

#define mmc_spi_resp_type(cmd)	((cmd)->flags & \
		(MMC_RSP_SPI_S1|MMC_RSP_SPI_BUSY|MMC_RSP_SPI_S2|MMC_RSP_SPI_B4))

#define mmc_cmd_type(cmd)	((cmd)->flags & MMC_CMD_MASK)

	unsigned int		retries;	
	unsigned int		error;		


	unsigned int		cmd_timeout_ms;	
	
	bool			ignore_timeout;

	struct mmc_data		*data;		
	struct mmc_request	*mrq;		
};

struct mmc_data {
	unsigned int		timeout_ns;	
	unsigned int		timeout_clks;	
	unsigned int		blksz;		
	unsigned int		blocks;		
	unsigned int		error;		
	unsigned int		flags;

#define MMC_DATA_WRITE	(1 << 8)
#define MMC_DATA_READ	(1 << 9)
#define MMC_DATA_STREAM	(1 << 10)

	unsigned int		bytes_xfered;

	struct mmc_command	*stop;		
	struct mmc_request	*mrq;		

	unsigned int		sg_len;		
	struct scatterlist	*sg;		
	s32			host_cookie;	
	bool			fault_injected; 
};

struct mmc_host;
struct mmc_request {
	struct mmc_command	*sbc;		
	struct mmc_command	*cmd;
	struct mmc_data		*data;
	struct mmc_command	*stop;

	struct completion	completion;
	void			(*done)(struct mmc_request *);
	struct mmc_host		*host;
};

struct mmc_card;
struct mmc_async_req;

extern int mmc_stop_bkops(struct mmc_card *);
extern int mmc_read_bkops_status(struct mmc_card *);
extern bool mmc_card_is_prog_state(struct mmc_card *);
extern struct mmc_async_req *mmc_start_req(struct mmc_host *,
					   struct mmc_async_req *, int *);
extern int mmc_interrupt_hpi(struct mmc_card *);
extern void mmc_wait_for_req(struct mmc_host *, struct mmc_request *);
extern int mmc_wait_for_cmd(struct mmc_host *, struct mmc_command *, int);
extern int mmc_app_cmd(struct mmc_host *, struct mmc_card *);
extern int mmc_wait_for_app_cmd(struct mmc_host *, struct mmc_card *,
	struct mmc_command *, int);
extern int mmc_card_start_bkops(struct mmc_card *host);
extern int mmc_card_stop_bkops(struct mmc_card *card);
extern void mmc_start_bkops(struct mmc_card *card, bool from_exception);
extern void mmc_start_delayed_bkops(struct mmc_card *card);
extern void mmc_start_idle_time_bkops(struct work_struct *work);
extern void mmc_bkops_completion_polling(struct work_struct *work);
extern int __mmc_switch(struct mmc_card *, u8, u8, u8, unsigned int, bool,
			bool);
extern int mmc_switch(struct mmc_card *, u8, u8, u8, unsigned int);
extern int mmc_switch_ignore_timeout(struct mmc_card *, u8, u8, u8,
				     unsigned int);
extern int mmc_send_ext_csd(struct mmc_card *card, u8 *ext_csd);

#define MMC_ERASE_ARG		0x00000000
#define MMC_TRIM_ARG		0x00000001
#define MMC_DISCARD_ARG		0x00000003

#define MMC_SECURE_ARGS		0x80000000
#define MMC_TRIM_ARGS		0x00008001

extern int mmc_erase(struct mmc_card *card, unsigned int from, unsigned int nr,
		     unsigned int arg);
extern int mmc_can_erase(struct mmc_card *card);
extern int mmc_can_trim(struct mmc_card *card);
extern int mmc_can_discard(struct mmc_card *card);
extern int mmc_can_sanitize(struct mmc_card *card);
extern int mmc_can_secure_erase_trim(struct mmc_card *card);
extern int mmc_erase_group_aligned(struct mmc_card *card, unsigned int from,
				   unsigned int nr);
extern unsigned int mmc_calc_max_discard(struct mmc_card *card);

extern int mmc_set_blocklen(struct mmc_card *card, unsigned int blocklen);
extern int mmc_set_blockcount(struct mmc_card *card, unsigned int blockcount,
			      bool is_rel_write);
extern int mmc_hw_reset(struct mmc_host *host);
extern int mmc_hw_reset_check(struct mmc_host *host);
extern int mmc_can_reset(struct mmc_card *card);

extern void mmc_set_data_timeout(struct mmc_data *, const struct mmc_card *);
extern unsigned int mmc_align_data_size(struct mmc_card *, unsigned int);

extern int __mmc_claim_host(struct mmc_host *host, atomic_t *abort);
extern void mmc_release_host(struct mmc_host *host);
extern int mmc_try_claim_host(struct mmc_host *host);
extern void mmc_set_ios(struct mmc_host *host);
extern int mmc_flush_cache(struct mmc_card *);

extern int mmc_detect_card_removed(struct mmc_host *host);

extern void mmc_blk_init_bkops_statistics(struct mmc_card *card);
extern void mmc_rpm_hold(struct mmc_host *host, struct device *dev);
extern void mmc_rpm_release(struct mmc_host *host, struct device *dev);

static inline void mmc_claim_host(struct mmc_host *host)
{
	__mmc_claim_host(host, NULL);
}

extern u32 mmc_vddrange_to_ocrmask(int vdd_min, int vdd_max);

#endif 
#endif 
