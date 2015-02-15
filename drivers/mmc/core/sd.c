/*
 *  linux/drivers/mmc/core/sd.c
 *
 *  Copyright (C) 2003-2004 Russell King, All Rights Reserved.
 *  SD support Copyright (C) 2004 Ian Molton, All Rights Reserved.
 *  Copyright (C) 2005-2007 Pierre Ossman, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/stat.h>

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/pm_runtime.h>

#include "core.h"
#include "bus.h"
#include "mmc_ops.h"
#include "sd.h"
#include "sd_ops.h"

#define UHS_SDR104_MIN_DTR	(100 * 1000 * 1000)
#define UHS_DDR50_MIN_DTR	(50 * 1000 * 1000)
#define UHS_SDR50_MIN_DTR	(50 * 1000 * 1000)
#define UHS_SDR25_MIN_DTR	(25 * 1000 * 1000)
#define UHS_SDR12_MIN_DTR	(12.5 * 1000 * 1000)

static const unsigned int tran_exp[] = {
	10000,		100000,		1000000,	10000000,
	0,		0,		0,		0
};

static const unsigned char tran_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

static const unsigned int tacc_exp[] = {
	1,	10,	100,	1000,	10000,	100000,	1000000, 10000000,
};

static const unsigned int tacc_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

#define UNSTUFF_BITS(resp,start,size)					\
	({								\
		const int __size = size;				\
		const u32 __mask = (__size < 32 ? 1 << __size : 0) - 1;	\
		const int __off = 3 - ((start) / 32);			\
		const int __shft = (start) & 31;			\
		u32 __res;						\
									\
		__res = resp[__off] >> __shft;				\
		if (__size + __shft > 32)				\
			__res |= resp[__off-1] << ((32 - __shft) % 32);	\
		__res & __mask;						\
	})

void mmc_decode_cid(struct mmc_card *card)
{
	u32 *resp = card->raw_cid;

	memset(&card->cid, 0, sizeof(struct mmc_cid));

	card->cid.manfid		= UNSTUFF_BITS(resp, 120, 8);
	card->cid.oemid			= UNSTUFF_BITS(resp, 104, 16);
	card->cid.prod_name[0]		= UNSTUFF_BITS(resp, 96, 8);
	card->cid.prod_name[1]		= UNSTUFF_BITS(resp, 88, 8);
	card->cid.prod_name[2]		= UNSTUFF_BITS(resp, 80, 8);
	card->cid.prod_name[3]		= UNSTUFF_BITS(resp, 72, 8);
	card->cid.prod_name[4]		= UNSTUFF_BITS(resp, 64, 8);
	card->cid.hwrev			= UNSTUFF_BITS(resp, 60, 4);
	card->cid.fwrev			= UNSTUFF_BITS(resp, 56, 4);
	card->cid.serial		= UNSTUFF_BITS(resp, 24, 32);
	card->cid.year			= UNSTUFF_BITS(resp, 12, 8);
	card->cid.month			= UNSTUFF_BITS(resp, 8, 4);

	card->cid.year += 2000; 
}

static int mmc_decode_csd(struct mmc_card *card)
{
	struct mmc_csd *csd = &card->csd;
	unsigned int e, m, csd_struct;
	u32 *resp = card->raw_csd;

	csd_struct = UNSTUFF_BITS(resp, 126, 2);

	switch (csd_struct) {
	case 0:
		m = UNSTUFF_BITS(resp, 115, 4);
		e = UNSTUFF_BITS(resp, 112, 3);
		csd->tacc_ns	 = (tacc_exp[e] * tacc_mant[m] + 9) / 10;
		csd->tacc_clks	 = UNSTUFF_BITS(resp, 104, 8) * 100;

		m = UNSTUFF_BITS(resp, 99, 4);
		e = UNSTUFF_BITS(resp, 96, 3);
		csd->max_dtr	  = tran_exp[e] * tran_mant[m];
		csd->cmdclass	  = UNSTUFF_BITS(resp, 84, 12);

		e = UNSTUFF_BITS(resp, 47, 3);
		m = UNSTUFF_BITS(resp, 62, 12);
		csd->capacity	  = (1 + m) << (e + 2);

		csd->read_blkbits = UNSTUFF_BITS(resp, 80, 4);
		csd->read_partial = UNSTUFF_BITS(resp, 79, 1);
		csd->write_misalign = UNSTUFF_BITS(resp, 78, 1);
		csd->read_misalign = UNSTUFF_BITS(resp, 77, 1);
		csd->r2w_factor = UNSTUFF_BITS(resp, 26, 3);
		csd->write_blkbits = UNSTUFF_BITS(resp, 22, 4);
		csd->write_partial = UNSTUFF_BITS(resp, 21, 1);

		if (UNSTUFF_BITS(resp, 46, 1)) {
			csd->erase_size = 1;
		} else if (csd->write_blkbits >= 9) {
			csd->erase_size = UNSTUFF_BITS(resp, 39, 7) + 1;
			csd->erase_size <<= csd->write_blkbits - 9;
		}
		break;
	case 1:
		mmc_card_set_blockaddr(card);

		csd->tacc_ns	 = 0; 
		csd->tacc_clks	 = 0; 

		m = UNSTUFF_BITS(resp, 99, 4);
		e = UNSTUFF_BITS(resp, 96, 3);
		csd->max_dtr	  = tran_exp[e] * tran_mant[m];
		csd->cmdclass	  = UNSTUFF_BITS(resp, 84, 12);
		csd->c_size	  = UNSTUFF_BITS(resp, 48, 22);

		
		if (csd->c_size >= 0xFFFF)
			mmc_card_set_ext_capacity(card);

		m = UNSTUFF_BITS(resp, 48, 22);
		csd->capacity     = (1 + m) << 10;

		csd->read_blkbits = 9;
		csd->read_partial = 0;
		csd->write_misalign = 0;
		csd->read_misalign = 0;
		csd->r2w_factor = 4; 
		csd->write_blkbits = 9;
		csd->write_partial = 0;
		csd->erase_size = 1;
		break;
	default:
		pr_err("%s: unrecognised CSD structure version %d\n",
			mmc_hostname(card->host), csd_struct);
		return -EINVAL;
	}

	card->erase_size = csd->erase_size;

	return 0;
}

static int mmc_decode_scr(struct mmc_card *card)
{
	struct sd_scr *scr = &card->scr;
	unsigned int scr_struct;
	u32 resp[4];

	resp[3] = card->raw_scr[1];
	resp[2] = card->raw_scr[0];

	scr_struct = UNSTUFF_BITS(resp, 60, 4);
	if (scr_struct != 0) {
		pr_err("%s: unrecognised SCR structure version %d\n",
			mmc_hostname(card->host), scr_struct);
		return -EINVAL;
	}

	scr->sda_vsn = UNSTUFF_BITS(resp, 56, 4);
	scr->bus_widths = UNSTUFF_BITS(resp, 48, 4);
	if (scr->sda_vsn == SCR_SPEC_VER_2)
		
		scr->sda_spec3 = UNSTUFF_BITS(resp, 47, 1);

	if (UNSTUFF_BITS(resp, 55, 1))
		card->erased_byte = 0xFF;
	else
		card->erased_byte = 0x0;

	if (scr->sda_spec3)
		scr->cmds = UNSTUFF_BITS(resp, 32, 2);
	return 0;
}

static int mmc_read_ssr(struct mmc_card *card)
{
	unsigned int au, es, et, eo, spd_cls;
	unsigned char card_spd_val[5] = {0, 2, 4, 6, 10};
	int err, i;
	u32 *ssr;

	if (!(card->csd.cmdclass & CCC_APP_SPEC)) {
		pr_warning("%s: card lacks mandatory SD Status "
			"function.\n", mmc_hostname(card->host));
		return 0;
	}

	ssr = kmalloc(64, GFP_KERNEL);
	if (!ssr)
		return -ENOMEM;

	err = mmc_app_sd_status(card, ssr);
	if (err) {
		pr_warning("%s: problem reading SD Status "
			"register.\n", mmc_hostname(card->host));
		err = 0;
		goto out;
	}

	for (i = 0; i < 16; i++)
		ssr[i] = be32_to_cpu(ssr[i]);

	au = UNSTUFF_BITS(ssr, 428 - 384, 4);
	if (au > 0 && au <= 9) {
		card->ssr.au = 1 << (au + 3);
		es = UNSTUFF_BITS(ssr, 408 - 384, 16);
		et = UNSTUFF_BITS(ssr, 402 - 384, 6);
		eo = UNSTUFF_BITS(ssr, 400 - 384, 2);
		if (es && et) {
			card->ssr.erase_timeout = (et * 1000) / es;
			card->ssr.erase_offset = eo * 1000;
		}
		printk(KERN_INFO "%s: au : %d KB\n", mmc_hostname(card->host), card->ssr.au);
	} else {
		pr_warning("%s: SD Status: Invalid Allocation Unit "
			"size.\n", mmc_hostname(card->host));
	}
	spd_cls = UNSTUFF_BITS(ssr, 440 - 384, 8);
	if (spd_cls < 5 && spd_cls > 0) {
		printk(KERN_INFO "%s: speed class type is CLASS %d\n", mmc_hostname(card->host), card_spd_val[spd_cls]);
		card->speed_class = card_spd_val[spd_cls];
	}
	else {
		printk(KERN_INFO "%s: Unknown speed class type\n", mmc_hostname(card->host));
		card->speed_class = -1;
	}
out:
	kfree(ssr);
	return err;
}

static int mmc_read_switch(struct mmc_card *card)
{
	int err;
	u8 *status;

	if (card->scr.sda_vsn < SCR_SPEC_VER_1)
		return 0;

	if (!(card->csd.cmdclass & CCC_SWITCH)) {
		pr_warning("%s: card lacks mandatory switch "
			"function, performance might suffer.\n",
			mmc_hostname(card->host));
		return 0;
	}

	err = -EIO;

	status = kmalloc(64, GFP_KERNEL);
	if (!status) {
		pr_err("%s: could not allocate a buffer for "
			"switch capabilities.\n",
			mmc_hostname(card->host));
		return -ENOMEM;
	}

	
	err = mmc_sd_switch(card, 0, 0, 1, status);
	if (err) {
		if (err != -EINVAL && err != -ENOSYS && err != -EFAULT)
			goto out;

		pr_warning("%s: problem reading Bus Speed modes.\n",
			mmc_hostname(card->host));
		err = 0;

		goto out;
	}

	if (status[13] & SD_MODE_HIGH_SPEED)
		card->sw_caps.hs_max_dtr = HIGH_SPEED_MAX_DTR;

	if (card->scr.sda_spec3) {
		card->sw_caps.sd3_bus_mode = status[13];

		
		err = mmc_sd_switch(card, 0, 2, 1, status);
		if (err) {
			if (err != -EINVAL && err != -ENOSYS && err != -EFAULT)
				goto out;

			pr_warning("%s: problem reading "
				"Driver Strength.\n",
				mmc_hostname(card->host));
			err = 0;

			goto out;
		}

		card->sw_caps.sd3_drv_type = status[9];

		
		err = mmc_sd_switch(card, 0, 3, 1, status);
		if (err) {
			if (err != -EINVAL && err != -ENOSYS && err != -EFAULT)
				goto out;

			pr_warning("%s: problem reading "
				"Current Limit.\n",
				mmc_hostname(card->host));
			err = 0;

			goto out;
		}

		card->sw_caps.sd3_curr_limit = status[7];
	}

out:
	kfree(status);

	return err;
}

int mmc_sd_switch_hs(struct mmc_card *card)
{
	int err;
	u8 *status;

	card->sw_caps.uhs_max_dtr = 0;

	if (card->scr.sda_vsn < SCR_SPEC_VER_1)
		return 0;

	if (!(card->csd.cmdclass & CCC_SWITCH))
		return 0;

	if (!(card->host->caps & MMC_CAP_SD_HIGHSPEED))
		return 0;

	if (card->sw_caps.hs_max_dtr == 0)
		return 0;

	err = -EIO;

	status = kmalloc(64, GFP_KERNEL);
	if (!status) {
		pr_err("%s: could not allocate a buffer for "
			"switch capabilities.\n", mmc_hostname(card->host));
		return -ENOMEM;
	}

	err = mmc_sd_switch(card, 1, 0, 1, status);
	if (err)
		goto out;

	if ((status[16] & 0xF) != 1) {
		pr_warning("%s: Problem switching card "
			"into high-speed mode!\n",
			mmc_hostname(card->host));
		err = 0;
	} else {
		err = 1;
	}

out:
	kfree(status);

	return err;
}

static int sd_select_driver_type(struct mmc_card *card, u8 *status)
{
	int host_drv_type = SD_DRIVER_TYPE_B;
	int card_drv_type = SD_DRIVER_TYPE_B;
	int drive_strength;
	int err;

	if (!(card->host->caps & (MMC_CAP_DRIVER_TYPE_A | MMC_CAP_DRIVER_TYPE_C
	    | MMC_CAP_DRIVER_TYPE_D)))
		return 0;

	if (!card->host->ops->select_drive_strength)
		return 0;

	if (card->host->caps & MMC_CAP_DRIVER_TYPE_A)
		host_drv_type |= SD_DRIVER_TYPE_A;

	if (card->host->caps & MMC_CAP_DRIVER_TYPE_C)
		host_drv_type |= SD_DRIVER_TYPE_C;

	if (card->host->caps & MMC_CAP_DRIVER_TYPE_D)
		host_drv_type |= SD_DRIVER_TYPE_D;

	if (card->sw_caps.sd3_drv_type & SD_DRIVER_TYPE_A)
		card_drv_type |= SD_DRIVER_TYPE_A;

	if (card->sw_caps.sd3_drv_type & SD_DRIVER_TYPE_C)
		card_drv_type |= SD_DRIVER_TYPE_C;

	if (card->sw_caps.sd3_drv_type & SD_DRIVER_TYPE_D)
		card_drv_type |= SD_DRIVER_TYPE_D;

	mmc_host_clk_hold(card->host);
	drive_strength = card->host->ops->select_drive_strength(
		card->sw_caps.uhs_max_dtr,
		host_drv_type, card_drv_type);
	mmc_host_clk_release(card->host);

	err = mmc_sd_switch(card, 1, 2, drive_strength, status);
	if (err)
		return err;

	if ((status[15] & 0xF) != drive_strength) {
		pr_warning("%s: Problem setting drive strength!\n",
			mmc_hostname(card->host));
		return 0;
	}

	mmc_set_driver_type(card->host, drive_strength);

	return 0;
}

static void sd_update_bus_speed_mode(struct mmc_card *card)
{
	if (!(card->host->caps & (MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 |
	    MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_SDR104 | MMC_CAP_UHS_DDR50))) {
		card->sd_bus_speed = 0;
		return;
	}

	if ((card->host->caps & MMC_CAP_UHS_SDR104) &&
	    (card->sw_caps.sd3_bus_mode & SD_MODE_UHS_SDR104) &&
	    (card->host->f_max > UHS_SDR104_MIN_DTR)) {
			card->sd_bus_speed = UHS_SDR104_BUS_SPEED;
	} else if ((card->host->caps & MMC_CAP_UHS_DDR50) &&
		   (card->sw_caps.sd3_bus_mode & SD_MODE_UHS_DDR50) &&
		    (card->host->f_max > UHS_DDR50_MIN_DTR)) {
			card->sd_bus_speed = UHS_DDR50_BUS_SPEED;
	} else if ((card->host->caps & (MMC_CAP_UHS_SDR104 |
		    MMC_CAP_UHS_SDR50)) && (card->sw_caps.sd3_bus_mode &
		    SD_MODE_UHS_SDR50) &&
		    (card->host->f_max > UHS_SDR50_MIN_DTR)) {
			card->sd_bus_speed = UHS_SDR50_BUS_SPEED;
	} else if ((card->host->caps & (MMC_CAP_UHS_SDR104 |
		    MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_SDR25)) &&
		   (card->sw_caps.sd3_bus_mode & SD_MODE_UHS_SDR25) &&
		 (card->host->f_max > UHS_SDR25_MIN_DTR)) {
			card->sd_bus_speed = UHS_SDR25_BUS_SPEED;
	} else if ((card->host->caps & (MMC_CAP_UHS_SDR104 |
		    MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_SDR25 |
		    MMC_CAP_UHS_SDR12)) && (card->sw_caps.sd3_bus_mode &
		    SD_MODE_UHS_SDR12)) {
			card->sd_bus_speed = UHS_SDR12_BUS_SPEED;
	}
}

static int sd_set_bus_speed_mode(struct mmc_card *card, u8 *status)
{
	int err;
	unsigned int timing = 0;

	switch (card->sd_bus_speed) {
	case UHS_SDR104_BUS_SPEED:
		timing = MMC_TIMING_UHS_SDR104;
		card->sw_caps.uhs_max_dtr = UHS_SDR104_MAX_DTR;
		break;
	case UHS_DDR50_BUS_SPEED:
		timing = MMC_TIMING_UHS_DDR50;
		card->sw_caps.uhs_max_dtr = UHS_DDR50_MAX_DTR;
		break;
	case UHS_SDR50_BUS_SPEED:
		timing = MMC_TIMING_UHS_SDR50;
		card->sw_caps.uhs_max_dtr = UHS_SDR50_MAX_DTR;
		break;
	case UHS_SDR25_BUS_SPEED:
		timing = MMC_TIMING_UHS_SDR25;
		card->sw_caps.uhs_max_dtr = UHS_SDR25_MAX_DTR;
		break;
	case UHS_SDR12_BUS_SPEED:
		timing = MMC_TIMING_UHS_SDR12;
		card->sw_caps.uhs_max_dtr = UHS_SDR12_MAX_DTR;
		break;
	default:
		return 0;
	}

	err = mmc_sd_switch(card, 1, 0, card->sd_bus_speed, status);
	if (err)
		return err;

	if ((status[16] & 0xF) != card->sd_bus_speed)
		pr_warning("%s: Problem setting bus speed mode!\n",
			mmc_hostname(card->host));
	else {
		mmc_set_timing(card->host, timing);
		mmc_set_clock(card->host, card->sw_caps.uhs_max_dtr);
	}

	return 0;
}

static int sd_set_current_limit(struct mmc_card *card, u8 *status)
{
	int current_limit = 0;
	int err;

	if ((card->sd_bus_speed == UHS_SDR50_BUS_SPEED) ||
	    (card->sd_bus_speed == UHS_SDR104_BUS_SPEED) ||
	    (card->sd_bus_speed == UHS_DDR50_BUS_SPEED)) {
		if (card->host->caps & MMC_CAP_MAX_CURRENT_800) {
			if (card->sw_caps.sd3_curr_limit & SD_MAX_CURRENT_800)
				current_limit = SD_SET_CURRENT_LIMIT_800;
			else if (card->sw_caps.sd3_curr_limit &
					SD_MAX_CURRENT_600)
				current_limit = SD_SET_CURRENT_LIMIT_600;
			else if (card->sw_caps.sd3_curr_limit &
					SD_MAX_CURRENT_400)
				current_limit = SD_SET_CURRENT_LIMIT_400;
			else if (card->sw_caps.sd3_curr_limit &
					SD_MAX_CURRENT_200)
				current_limit = SD_SET_CURRENT_LIMIT_200;
		} else if (card->host->caps & MMC_CAP_MAX_CURRENT_600) {
			if (card->sw_caps.sd3_curr_limit & SD_MAX_CURRENT_600)
				current_limit = SD_SET_CURRENT_LIMIT_600;
			else if (card->sw_caps.sd3_curr_limit &
					SD_MAX_CURRENT_400)
				current_limit = SD_SET_CURRENT_LIMIT_400;
			else if (card->sw_caps.sd3_curr_limit &
					SD_MAX_CURRENT_200)
				current_limit = SD_SET_CURRENT_LIMIT_200;
		} else if (card->host->caps & MMC_CAP_MAX_CURRENT_400) {
			if (card->sw_caps.sd3_curr_limit & SD_MAX_CURRENT_400)
				current_limit = SD_SET_CURRENT_LIMIT_400;
			else if (card->sw_caps.sd3_curr_limit &
					SD_MAX_CURRENT_200)
				current_limit = SD_SET_CURRENT_LIMIT_200;
		} else if (card->host->caps & MMC_CAP_MAX_CURRENT_200) {
			if (card->sw_caps.sd3_curr_limit & SD_MAX_CURRENT_200)
				current_limit = SD_SET_CURRENT_LIMIT_200;
		}
	} else
		current_limit = SD_SET_CURRENT_LIMIT_200;

	err = mmc_sd_switch(card, 1, 3, current_limit, status);
	if (err)
		return err;

	if (((status[15] >> 4) & 0x0F) != current_limit)
		pr_warning("%s: Problem setting current limit!\n",
			mmc_hostname(card->host));

	return 0;
}

static int mmc_sd_change_bus_speed(struct mmc_host *host, unsigned long *freq)
{
	int err = 0;
	struct mmc_card *card;

	mmc_claim_host(host);
	card = host->card;

	
	if (!card || !freq) {
		err = -EINVAL;
		goto out;
	}

	if (mmc_card_uhs(card)) {
		if (*freq > card->sw_caps.uhs_max_dtr)
			*freq = card->sw_caps.uhs_max_dtr;
	} else {
		if (*freq > mmc_sd_get_max_clock(card))
			*freq = mmc_sd_get_max_clock(card);
	}

	if (*freq < host->f_min)
		*freq = host->f_min;

	mmc_set_clock(host, (unsigned int) (*freq));

	if (!mmc_host_is_spi(card->host) && mmc_sd_card_uhs(card)
			&& card->host->ops->execute_tuning) {
		mmc_host_clk_hold(card->host);
		err = card->host->ops->execute_tuning(card->host,
				MMC_SEND_TUNING_BLOCK);
		mmc_host_clk_release(card->host);

		if (err) {
			pr_warn("%s: %s: tuning execution failed %d. Restoring to previous clock %lu\n",
				   mmc_hostname(card->host), __func__, err,
				   host->clk_scaling.curr_freq);
			mmc_set_clock(host, host->clk_scaling.curr_freq);
		}
	}

out:
	mmc_release_host(host);
	return err;
}

static int mmc_sd_init_uhs_card(struct mmc_card *card)
{
	int err;
	u8 *status;

	if (!card->scr.sda_spec3)
		return 0;

	if (!(card->csd.cmdclass & CCC_SWITCH))
		return 0;

	status = kmalloc(64, GFP_KERNEL);
	if (!status) {
		pr_err("%s: could not allocate a buffer for "
			"switch capabilities.\n", mmc_hostname(card->host));
		return -ENOMEM;
	}

	
	if ((card->host->caps & MMC_CAP_4_BIT_DATA) &&
	    (card->scr.bus_widths & SD_SCR_BUS_WIDTH_4)) {
		err = mmc_app_set_bus_width(card, MMC_BUS_WIDTH_4);
		if (err)
			goto out;

		mmc_set_bus_width(card->host, MMC_BUS_WIDTH_4);
	}

	sd_update_bus_speed_mode(card);

	
	err = sd_select_driver_type(card, status);
	if (err)
		goto out;

	
	err = sd_set_current_limit(card, status);
	if (err)
		goto out;

	
	err = sd_set_bus_speed_mode(card, status);
	if (err)
		goto out;

	
	if (!mmc_host_is_spi(card->host) && card->host->ops->execute_tuning) {
		mmc_host_clk_hold(card->host);
		err = card->host->ops->execute_tuning(card->host,
						      MMC_SEND_TUNING_BLOCK);
		mmc_host_clk_release(card->host);
	}

out:
	kfree(status);

	return err;
}

MMC_DEV_ATTR(cid, "%08x%08x%08x%08x\n", card->raw_cid[0], card->raw_cid[1],
	card->raw_cid[2], card->raw_cid[3]);
MMC_DEV_ATTR(csd, "%08x%08x%08x%08x\n", card->raw_csd[0], card->raw_csd[1],
	card->raw_csd[2], card->raw_csd[3]);
MMC_DEV_ATTR(scr, "%08x%08x\n", card->raw_scr[0], card->raw_scr[1]);
MMC_DEV_ATTR(date, "%02d/%04d\n", card->cid.month, card->cid.year);
MMC_DEV_ATTR(erase_size, "%u\n", card->erase_size << 9);
MMC_DEV_ATTR(preferred_erase_size, "%u\n", card->pref_erase << 9);
MMC_DEV_ATTR(fwrev, "0x%x\n", card->cid.fwrev);
MMC_DEV_ATTR(hwrev, "0x%x\n", card->cid.hwrev);
MMC_DEV_ATTR(manfid, "0x%06x\n", card->cid.manfid);
MMC_DEV_ATTR(name, "%s\n", card->cid.prod_name);
MMC_DEV_ATTR(oemid, "0x%04x\n", card->cid.oemid);
MMC_DEV_ATTR(serial, "0x%08x\n", card->cid.serial);


static struct attribute *sd_std_attrs[] = {
	&dev_attr_cid.attr,
	&dev_attr_csd.attr,
	&dev_attr_scr.attr,
	&dev_attr_date.attr,
	&dev_attr_erase_size.attr,
	&dev_attr_preferred_erase_size.attr,
	&dev_attr_fwrev.attr,
	&dev_attr_hwrev.attr,
	&dev_attr_manfid.attr,
	&dev_attr_name.attr,
	&dev_attr_oemid.attr,
	&dev_attr_serial.attr,
	NULL,
};

static struct attribute_group sd_std_attr_group = {
	.attrs = sd_std_attrs,
};

static const struct attribute_group *sd_attr_groups[] = {
	&sd_std_attr_group,
	NULL,
};

struct device_type sd_type = {
	.groups = sd_attr_groups,
};

int mmc_sd_get_cid(struct mmc_host *host, u32 ocr, u32 *cid, u32 *rocr)
{
	int err;

	mmc_go_idle(host);

	err = mmc_send_if_cond(host, ocr);
	if (!err)
		ocr |= SD_OCR_CCS;

	if (host->caps & (MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 |
	    MMC_CAP_UHS_SDR50 | MMC_CAP_UHS_SDR104 | MMC_CAP_UHS_DDR50))
		ocr |= SD_OCR_S18R;

	
	if (host->caps & (MMC_CAP_SET_XPC_330 | MMC_CAP_SET_XPC_300 |
	    MMC_CAP_SET_XPC_180))
		ocr |= SD_OCR_XPC;

try_again:
	err = mmc_send_app_op_cond(host, ocr, rocr);
	if (err)
		return err;

	if (!mmc_host_is_spi(host) && rocr &&
	   ((*rocr & 0x41000000) == 0x41000000)) {
		err = mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_180, true);
		if (err) {
			mmc_power_cycle(host);
			ocr &= ~SD_OCR_S18R;
			goto try_again;
		}
	}

	if (mmc_host_is_spi(host))
		err = mmc_send_cid(host, cid);
	else
		err = mmc_all_send_cid(host, cid);

	return err;
}

int mmc_sd_get_csd(struct mmc_host *host, struct mmc_card *card)
{
	int err;

	err = mmc_send_csd(card, card->raw_csd);
	if (err)
		return err;

	err = mmc_decode_csd(card);
	if (err)
		return err;

	return 0;
}

int mmc_sd_setup_card(struct mmc_host *host, struct mmc_card *card,
	bool reinit)
{
	int err;
#ifdef CONFIG_MMC_PARANOID_SD_INIT
	int retries;
#endif

	if (!reinit) {
		err = mmc_app_send_scr(card, card->raw_scr);
		if (err)
			return err;

		err = mmc_decode_scr(card);
		if (err)
			return err;

		err = mmc_read_ssr(card);
		if (err)
			return err;

		
		mmc_init_erase(card);

#ifdef CONFIG_MMC_PARANOID_SD_INIT
		for (retries = 1; retries <= 3; retries++) {
			err = mmc_read_switch(card);
			if (!err) {
				if (retries > 1) {
					printk(KERN_WARNING
					       "%s: recovered\n", 
					       mmc_hostname(host));
				}
				break;
			} else {
				printk(KERN_WARNING
				       "%s: read switch failed (attempt %d)\n",
				       mmc_hostname(host), retries);
			}
		}
#else
		err = mmc_read_switch(card);
#endif

		if (err)
			return err;
	}

	if (mmc_host_is_spi(host)) {
		err = mmc_spi_set_crc(host, use_spi_crc);
		if (err)
			return err;
	}

	if (!reinit) {
		int ro = -1;

		if (host->ops->get_ro) {
			mmc_host_clk_hold(card->host);
			ro = host->ops->get_ro(host);
			mmc_host_clk_release(card->host);
		}

		if (ro < 0) {
			pr_warning("%s: host does not "
				"support reading read-only "
				"switch. assuming write-enable.\n",
				mmc_hostname(host));
		} else if (ro > 0) {
			mmc_card_set_readonly(card);
		}
	}

	return 0;
}

unsigned mmc_sd_get_max_clock(struct mmc_card *card)
{
	unsigned max_dtr = (unsigned int)-1;

	if (mmc_card_highspeed(card)) {
		if (max_dtr > card->sw_caps.hs_max_dtr)
			max_dtr = card->sw_caps.hs_max_dtr;
	} else if (max_dtr > card->csd.max_dtr) {
		max_dtr = card->csd.max_dtr;
	}

	return max_dtr;
}

void mmc_sd_go_highspeed(struct mmc_card *card)
{
	mmc_card_set_highspeed(card);
	mmc_set_timing(card->host, MMC_TIMING_SD_HS);
}

static int mmc_sd_init_card(struct mmc_host *host, u32 ocr,
	struct mmc_card *oldcard)
{
	struct mmc_card *card;
	int err = 0, status = 0;
	u32 cid[4];
	u32 rocr = 0;

	BUG_ON(!host);
	WARN_ON(!host->claimed);

	
	mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_330, 0);

	err = mmc_sd_get_cid(host, ocr, cid, &rocr);
	if (err)
		return err;

	if (oldcard) {
		if (memcmp(cid, oldcard->raw_cid, sizeof(cid)) != 0)
			return -ENOENT;

		card = oldcard;
	} else {
		card = mmc_alloc_card(host, &sd_type);
		if (IS_ERR(card))
			return PTR_ERR(card);

		card->type = MMC_TYPE_SD;
		memcpy(card->raw_cid, cid, sizeof(card->raw_cid));
	}

	if (!mmc_host_is_spi(host)) {
		err = mmc_send_relative_addr(host, &card->rca);
		if (err)
			return err;
	}

	if (!oldcard) {
		err = mmc_sd_get_csd(host, card);
		if (err)
			return err;

		mmc_decode_cid(card);
	}

	if (!mmc_host_is_spi(host)) {
		err = mmc_select_card(card);
		if (err)
			return err;
	}

	err = mmc_sd_setup_card(host, card, oldcard != NULL);
	if (err)
		goto free_card;

	
	if (rocr & SD_ROCR_S18A) {
		err = mmc_sd_init_uhs_card(card);
		if (err)
			goto free_card;

		
		mmc_card_set_uhs(card);

		if (host->ops->enable_preset_value) {
			mmc_host_clk_hold(card->host);
			host->ops->enable_preset_value(host, true);
			mmc_host_clk_release(card->host);
		}
	} else {
		err = mmc_sd_switch_hs(card);
		if (err > 0)
			mmc_sd_go_highspeed(card);
		else if (err)
			goto free_card;

		mmc_set_clock(host, mmc_sd_get_max_clock(card));

		if ((host->caps & MMC_CAP_4_BIT_DATA) &&
			(card->scr.bus_widths & SD_SCR_BUS_WIDTH_4)) {
			err = mmc_app_set_bus_width(card, MMC_BUS_WIDTH_4);
			if (err)
				goto free_card;

			mmc_set_bus_width(host, MMC_BUS_WIDTH_4);
		}
	}

	err = mmc_send_status(card, &status);
	printk(KERN_INFO "%s: %s status : %#x, err = %#x, speed : %d\n",
		mmc_hostname(host), __func__, status, err,
		card->sw_caps.uhs_max_dtr ?
			card->sw_caps.uhs_max_dtr : card->sw_caps.hs_max_dtr);

	host->card = card;
	return 0;

free_card:
	if (!oldcard)
		mmc_remove_card(card);

	return err;
}

static void mmc_sd_remove(struct mmc_host *host)
{
	BUG_ON(!host);
	BUG_ON(!host->card);
	printk(KERN_INFO "%s: %s, claimed : %d, claim_cnt : %d\n", mmc_hostname(host),
		__func__, host->claimed, host->claim_cnt);

	mmc_exit_clk_scaling(host);
	mmc_remove_card(host->card);

	mmc_claim_host(host);
	host->card = NULL;
	mmc_release_host(host);
	printk(KERN_INFO "%s: %s, claimed : %d, claim_cnt : %d\n", mmc_hostname(host),
		__func__, host->claimed, host->claim_cnt);
}

static int mmc_sd_alive(struct mmc_host *host)
{
	return !host->ops->get_cd(host);
}

static void mmc_sd_detect(struct mmc_host *host)
{
	int ret = 0;
	const unsigned char ejected = 1;
#if 0
        int retries = 5;
#endif

	BUG_ON(!host);
	BUG_ON(!host->card);
	printk(KERN_INFO "%s: %s, claimed : %d, claim_cnt : %d\n", mmc_hostname(host),
	       __func__, host->claimed, host->claim_cnt);
	mmc_claim_host(host);

#if 0
	while(retries) {
		err = mmc_send_status(host->card, NULL);
		if (err) {
			retries--;
			udelay(5);
			continue;
		}
		break;
	}
	if (!retries) {
		printk(KERN_ERR "%s(%s): Unable to re-detect card (%d)\n",
		       __func__, mmc_hostname(host), err);
		err = _mmc_detect_card_removed(host);
	}
#endif
	ret = _mmc_detect_card_removed(host);

	mmc_release_host(host);

	if (ret == ejected) {
		remove_card(host);
#if 0
		mmc_sd_remove(host);

		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_power_off(host);
		mmc_release_host(host);
#endif
	}
}

static int mmc_sd_suspend(struct mmc_host *host)
{
	BUG_ON(!host);
	BUG_ON(!host->card);


	mmc_disable_clk_scaling(host);
	mmc_rpm_hold(host, &host->class_dev);
	mmc_claim_host(host);
	if (!mmc_host_is_spi(host))
		mmc_deselect_cards(host);
	host->card->state &= ~MMC_STATE_HIGHSPEED;
	mmc_release_host(host);
	mmc_rpm_release(host, &host->class_dev);

	return 0;
}

static int mmc_sd_resume(struct mmc_host *host)
{
	int err;
#ifdef CONFIG_MMC_PARANOID_SD_INIT
	int retries;
	int delayTime;
#endif

	BUG_ON(!host);
	BUG_ON(!host->card);

	host->crc_count = 0;
	
	host->caps |= host->caps_uhs;
	mmc_claim_host(host);
#ifdef CONFIG_MMC_PARANOID_SD_INIT
	retries = 5;
	delayTime = 5;
	while (retries) {
		if (host->ops->get_cd && host->ops->get_cd(host) == 0) {
			printk(KERN_ERR "%s(%s): find no card. Stop trying\n",
				__func__, mmc_hostname(host));
			break;
		}
		err = mmc_sd_init_card(host, host->ocr, host->card);

		if (err) {
			printk(KERN_ERR "%s: Re-init card rc = %d (retries = %d)\n",
			       mmc_hostname(host), err, retries);
			mmc_power_off(host);
			usleep_range(delayTime*1000, delayTime*1000+500);
			mmc_power_up(host);
			mmc_select_voltage(host, host->ocr);
			retries--;
			delayTime *= 2;
			continue;
		}
		break;
	}
#else
	err = mmc_sd_init_card(host, host->ocr, host->card);
#endif
	mmc_release_host(host);

	if (mmc_can_scale_clk(host))
		mmc_init_clk_scaling(host);

	return err;
}

static int mmc_sd_power_restore(struct mmc_host *host)
{
	int ret;

	
	mmc_disable_clk_scaling(host);

	host->card->state &= ~MMC_STATE_HIGHSPEED;
	mmc_claim_host(host);
	ret = mmc_sd_init_card(host, host->ocr, host->card);
	mmc_release_host(host);

	if (mmc_can_scale_clk(host))
		mmc_init_clk_scaling(host);

	return ret;
}

static const struct mmc_bus_ops mmc_sd_ops = {
	.remove = mmc_sd_remove,
	.detect = mmc_sd_detect,
	.suspend = NULL,
	.resume = NULL,
	.power_restore = mmc_sd_power_restore,
	.alive = mmc_sd_alive,
	.change_bus_speed = mmc_sd_change_bus_speed,
};

static const struct mmc_bus_ops mmc_sd_ops_unsafe = {
	.remove = mmc_sd_remove,
	.detect = mmc_sd_detect,
	.suspend = mmc_sd_suspend,
	.resume = mmc_sd_resume,
	.power_restore = mmc_sd_power_restore,
	.alive = mmc_sd_alive,
	.change_bus_speed = mmc_sd_change_bus_speed,
};

static void mmc_sd_attach_bus_ops(struct mmc_host *host)
{
	const struct mmc_bus_ops *bus_ops;

	if (!mmc_card_is_removable(host))
		bus_ops = &mmc_sd_ops_unsafe;
	else
		bus_ops = &mmc_sd_ops;
	mmc_attach_bus(host, bus_ops);
}

int mmc_attach_sd(struct mmc_host *host)
{
	int err;
	u32 ocr;
#ifdef CONFIG_MMC_PARANOID_SD_INIT
	int retries;
	int delayTime;
#endif

	BUG_ON(!host);
	WARN_ON(!host->claimed);

	
	if (host->ops->enable_preset_value) {
		mmc_host_clk_hold(host);
		host->ops->enable_preset_value(host, false);
		mmc_host_clk_release(host);
	}

	err = mmc_send_app_op_cond(host, 0, &ocr);
	if (err)
		return err;

	mmc_sd_attach_bus_ops(host);
	if (host->ocr_avail_sd)
		host->ocr_avail = host->ocr_avail_sd;

	if (mmc_host_is_spi(host)) {
		mmc_go_idle(host);

		err = mmc_spi_read_ocr(host, 0, &ocr);
		if (err)
			goto err;
	}

	if (ocr & 0x7F) {
		pr_warning("%s: card claims to support voltages "
		       "below the defined range. These will be ignored.\n",
		       mmc_hostname(host));
		ocr &= ~0x7F;
	}

	if ((ocr & MMC_VDD_165_195) &&
	    !(host->ocr_avail_sd & MMC_VDD_165_195)) {
		pr_warning("%s: SD card claims to support the "
		       "incompletely defined 'low voltage range'. This "
		       "will be ignored.\n", mmc_hostname(host));
		ocr &= ~MMC_VDD_165_195;
	}

	host->ocr = mmc_select_voltage(host, ocr);

	if (!host->ocr) {
		err = -EINVAL;
		goto err;
	}

#ifdef CONFIG_MMC_PARANOID_SD_INIT
	retries = 5;
	delayTime = 5;
	while (retries) {
		err = mmc_sd_init_card(host, host->ocr, NULL);
		if (err) {
			if (host->ops->get_cd && host->ops->get_cd(host) == 0) {
				printk(KERN_ERR "%s(%s): find no card (%d). Stop trying\n",
					__func__, mmc_hostname(host), err);
				goto err;
			}
			retries--;
			mmc_power_off(host);
			usleep_range(delayTime*1000, delayTime*1000+500);
			mmc_power_up(host);
			mmc_select_voltage(host, host->ocr);
			delayTime *= 2;
			continue;
		}
		break;
	}

	if (!retries) {
		printk(KERN_ERR "%s: mmc_sd_init_card() failure (err = %d)\n",
		       mmc_hostname(host), err);
		goto err;
	}
#else
	err = mmc_sd_init_card(host, host->ocr, NULL);
	if (err)
		goto err;
#endif

	mmc_release_host(host);
	err = mmc_add_card(host->card);
	mmc_claim_host(host);
	if (err)
		goto remove_card;

	mmc_init_clk_scaling(host);

	return 0;

remove_card:
	mmc_release_host(host);
	mmc_remove_card(host->card);
	host->card = NULL;
	mmc_claim_host(host);
err:
	mmc_detach_bus(host);

	pr_err("%s: error %d whilst initialising SD card\n",
		mmc_hostname(host), err);

	return err;
}

