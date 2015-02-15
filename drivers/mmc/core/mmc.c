/*
 *  linux/drivers/mmc/core/mmc.c
 *
 *  Copyright (C) 2003-2004 Russell King, All Rights Reserved.
 *  Copyright (C) 2005-2007 Pierre Ossman, All Rights Reserved.
 *  MMCv4 support Copyright (C) 2006 Philip Langdale, All Rights Reserved.
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
#include <linux/pm_runtime.h>
#include <linux/reboot.h>

#include "core.h"
#include "bus.h"
#include "mmc_ops.h"
#include "sd_ops.h"

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

static const struct mmc_fixup mmc_fixups[] = {
	MMC_FIXUP_EXT_CSD_REV(CID_NAME_ANY, CID_MANFID_HYNIX,
			      0x014a, add_quirk, MMC_QUIRK_BROKEN_HPI, 5),

	
	MMC_FIXUP_EXT_CSD_REV("MMC16G", CID_MANFID_KINGSTON, CID_OEMID_ANY,
			add_quirk, MMC_QUIRK_BROKEN_HPI, 5),

#if 0
	MMC_FIXUP("H8G2d", CID_MANFID_HYNIX, CID_OEMID_ANY, add_quirk_mmc,
		  MMC_QUIRK_CACHE_DISABLE),

	MMC_FIXUP("MMC16G", CID_MANFID_KINGSTON, CID_OEMID_ANY, add_quirk_mmc,
		  MMC_QUIRK_CACHE_DISABLE),
#endif
	MMC_FIXUP("MAG2GC", CID_MANFID_SAMSUNG, CID_OEMID_ANY, remove_quirk_mmc,
		  MMC_QUIRK_CACHE_DISABLE),
	MMC_FIXUP("AWPD3R", CID_MANFID_SAMSUNG, CID_OEMID_ANY, remove_quirk_mmc,
		  MMC_QUIRK_CACHE_DISABLE),
	MMC_FIXUP("BWBC3R", CID_MANFID_SAMSUNG, CID_OEMID_ANY, remove_quirk_mmc,
		  MMC_QUIRK_CACHE_DISABLE),
	MMC_FIXUP("SEM16G", CID_MANFID_SANDISK, CID_OEMID_ANY, remove_quirk_mmc,
		  MMC_QUIRK_CACHE_DISABLE),
	MMC_FIXUP("SEM32G", CID_MANFID_SANDISK, CID_OEMID_ANY, remove_quirk_mmc,
		  MMC_QUIRK_CACHE_DISABLE),
	MMC_FIXUP("HAG4d", CID_MANFID_HYNIX, CID_OEMID_ANY, remove_quirk_mmc,
		  MMC_QUIRK_CACHE_DISABLE),
	MMC_FIXUP("HBG4e", CID_MANFID_HYNIX, CID_OEMID_ANY, remove_quirk_mmc,
		  MMC_QUIRK_CACHE_DISABLE),
	MMC_FIXUP("HAG2e", CID_MANFID_HYNIX, CID_OEMID_ANY, remove_quirk_mmc,
		  MMC_QUIRK_CACHE_DISABLE),

	END_FIXUP
};

static int mmc_decode_cid(struct mmc_card *card)
{
	u32 *resp = card->raw_cid;

	switch (card->csd.mmca_vsn) {
	case 0: 
	case 1: 
		card->cid.manfid	= UNSTUFF_BITS(resp, 104, 24);
		card->cid.prod_name[0]	= UNSTUFF_BITS(resp, 96, 8);
		card->cid.prod_name[1]	= UNSTUFF_BITS(resp, 88, 8);
		card->cid.prod_name[2]	= UNSTUFF_BITS(resp, 80, 8);
		card->cid.prod_name[3]	= UNSTUFF_BITS(resp, 72, 8);
		card->cid.prod_name[4]	= UNSTUFF_BITS(resp, 64, 8);
		card->cid.prod_name[5]	= UNSTUFF_BITS(resp, 56, 8);
		card->cid.prod_name[6]	= UNSTUFF_BITS(resp, 48, 8);
		card->cid.hwrev		= UNSTUFF_BITS(resp, 44, 4);
		card->cid.fwrev		= UNSTUFF_BITS(resp, 40, 4);
		card->cid.serial	= UNSTUFF_BITS(resp, 16, 24);
		card->cid.month		= UNSTUFF_BITS(resp, 12, 4);
		card->cid.year		= UNSTUFF_BITS(resp, 8, 4) + 1997;
		break;

	case 2: 
	case 3: 
	case 4: 
		card->cid.manfid	= UNSTUFF_BITS(resp, 120, 8);
		card->cid.oemid		= UNSTUFF_BITS(resp, 104, 16);
		card->cid.prod_name[0]	= UNSTUFF_BITS(resp, 96, 8);
		card->cid.prod_name[1]	= UNSTUFF_BITS(resp, 88, 8);
		card->cid.prod_name[2]	= UNSTUFF_BITS(resp, 80, 8);
		card->cid.prod_name[3]	= UNSTUFF_BITS(resp, 72, 8);
		card->cid.prod_name[4]	= UNSTUFF_BITS(resp, 64, 8);
		card->cid.prod_name[5]	= UNSTUFF_BITS(resp, 56, 8);
		card->cid.serial	= UNSTUFF_BITS(resp, 16, 32);
		card->cid.month		= UNSTUFF_BITS(resp, 12, 4);
		card->cid.year		= UNSTUFF_BITS(resp, 8, 4) + 1997;

		
		if (card->cid.manfid == CID_MANFID_TOSHIBA ||
		    card->cid.manfid ==  CID_MANFID_MICRON ||
		    card->cid.manfid == CID_MANFID_SAMSUNG ||
		    card->cid.manfid == CID_MANFID_HYNIX)
			card->cid.fwrev = UNSTUFF_BITS(resp, 48, 8);

		break;

	default:
		pr_err("%s: card has unknown MMCA version %d\n",
			mmc_hostname(card->host), card->csd.mmca_vsn);
		return -EINVAL;
	}

	return 0;
}

static void mmc_set_erase_size(struct mmc_card *card)
{
	if (card->ext_csd.erase_group_def & 1)
		card->erase_size = card->ext_csd.hc_erase_size;
	else
		card->erase_size = card->csd.erase_size;

	mmc_init_erase(card);
}

static int mmc_decode_csd(struct mmc_card *card)
{
	struct mmc_csd *csd = &card->csd;
	unsigned int e, m, a, b;
	u32 *resp = card->raw_csd;

	csd->structure = UNSTUFF_BITS(resp, 126, 2);
	if (csd->structure == 0) {
		pr_err("%s: unrecognised CSD structure version %d\n",
			mmc_hostname(card->host), csd->structure);
		return -EINVAL;
	}

	csd->mmca_vsn	 = UNSTUFF_BITS(resp, 122, 4);
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

	if (csd->write_blkbits >= 9) {
		a = UNSTUFF_BITS(resp, 42, 5);
		b = UNSTUFF_BITS(resp, 37, 5);
		csd->erase_size = (a + 1) * (b + 1);
		csd->erase_size <<= csd->write_blkbits - 9;
	}

	return 0;
}

static int mmc_get_ext_csd(struct mmc_card *card, u8 **new_ext_csd)
{
	int err;
	u8 *ext_csd;

	BUG_ON(!card);
	BUG_ON(!new_ext_csd);

	*new_ext_csd = NULL;

	if (card->csd.mmca_vsn < CSD_SPEC_VER_4)
		return 0;

	ext_csd = kmalloc(512, GFP_KERNEL);
	if (!ext_csd) {
		pr_err("%s: could not allocate a buffer to "
			"receive the ext_csd.\n", mmc_hostname(card->host));
		return -ENOMEM;
	}

	err = mmc_send_ext_csd(card, ext_csd);
	if (err) {
		kfree(ext_csd);
		*new_ext_csd = NULL;

		if ((err != -EINVAL)
		 && (err != -ENOSYS)
		 && (err != -EFAULT))
			return err;

		if (card->csd.capacity == (4096 * 512)) {
			pr_err("%s: unable to read EXT_CSD "
				"on a possible high capacity card. "
				"Card will be ignored.\n",
				mmc_hostname(card->host));
		} else {
			pr_warning("%s: unable to read "
				"EXT_CSD, performance might "
				"suffer.\n",
				mmc_hostname(card->host));
			err = 0;
		}
	} else
		*new_ext_csd = ext_csd;

	return err;
}

static void mmc_select_card_type(struct mmc_card *card)
{
	struct mmc_host *host = card->host;
	u8 card_type = card->ext_csd.raw_card_type & EXT_CSD_CARD_TYPE_MASK;
	unsigned int caps = host->caps, caps2 = host->caps2;
	unsigned int hs_max_dtr = 0;

	if (card_type & EXT_CSD_CARD_TYPE_26)
		hs_max_dtr = MMC_HIGH_26_MAX_DTR;

	if (caps & MMC_CAP_MMC_HIGHSPEED &&
			card_type & EXT_CSD_CARD_TYPE_52)
		hs_max_dtr = MMC_HIGH_52_MAX_DTR;

	if ((caps & MMC_CAP_1_8V_DDR &&
			card_type & EXT_CSD_CARD_TYPE_DDR_1_8V) ||
	    (caps & MMC_CAP_1_2V_DDR &&
			card_type & EXT_CSD_CARD_TYPE_DDR_1_2V))
		hs_max_dtr = MMC_HIGH_DDR_MAX_DTR;

	if ((caps2 & MMC_CAP2_HS200_1_8V_SDR &&
			card_type & EXT_CSD_CARD_TYPE_SDR_1_8V) ||
	    (caps2 & MMC_CAP2_HS200_1_2V_SDR &&
			card_type & EXT_CSD_CARD_TYPE_SDR_1_2V))
		hs_max_dtr = MMC_HS200_MAX_DTR;

	if ((caps2 & MMC_CAP2_HS400_1_8V &&
			card_type & EXT_CSD_CARD_TYPE_HS400_1_8V) ||
	    (caps2 & MMC_CAP2_HS400_1_2V &&
			card_type & EXT_CSD_CARD_TYPE_HS400_1_2V))
		hs_max_dtr = MMC_HS400_MAX_DTR;

	card->ext_csd.hs_max_dtr = hs_max_dtr;
	card->ext_csd.card_type = card_type;
}

static int mmc_read_ext_csd(struct mmc_card *card, u8 *ext_csd)
{
	int err = 0, idx;
	unsigned int part_size;
	u8 hc_erase_grp_sz = 0, hc_wp_grp_sz = 0;

	BUG_ON(!card);

	if (!ext_csd)
		return 0;

	
	card->ext_csd.raw_ext_csd_structure = ext_csd[EXT_CSD_STRUCTURE];
	if (card->csd.structure == 3) {
		if (card->ext_csd.raw_ext_csd_structure > 2) {
			pr_err("%s: unrecognised EXT_CSD structure "
				"version %d\n", mmc_hostname(card->host),
					card->ext_csd.raw_ext_csd_structure);
			err = -EINVAL;
			goto out;
		}
	}

	card->ext_csd.rev = ext_csd[EXT_CSD_REV];
	if (card->ext_csd.rev > 7) {
		pr_err("%s: unrecognised EXT_CSD revision %d\n",
			mmc_hostname(card->host), card->ext_csd.rev);
		err = -EINVAL;
		goto out;
	}

	if (mmc_card_mmc(card))
		card->quirks |= MMC_QUIRK_CACHE_DISABLE;

	
	mmc_fixup_device(card, mmc_fixups);

	card->ext_csd.raw_sectors[0] = ext_csd[EXT_CSD_SEC_CNT + 0];
	card->ext_csd.raw_sectors[1] = ext_csd[EXT_CSD_SEC_CNT + 1];
	card->ext_csd.raw_sectors[2] = ext_csd[EXT_CSD_SEC_CNT + 2];
	card->ext_csd.raw_sectors[3] = ext_csd[EXT_CSD_SEC_CNT + 3];
	if (card->ext_csd.rev >= 2) {
		card->ext_csd.sectors =
			ext_csd[EXT_CSD_SEC_CNT + 0] << 0 |
			ext_csd[EXT_CSD_SEC_CNT + 1] << 8 |
			ext_csd[EXT_CSD_SEC_CNT + 2] << 16 |
			ext_csd[EXT_CSD_SEC_CNT + 3] << 24;

		
		if (card->ext_csd.sectors > (2u * 1024 * 1024 * 1024) / 512)
			mmc_card_set_blockaddr(card);
	}

	card->ext_csd.raw_card_type = ext_csd[EXT_CSD_CARD_TYPE];
	mmc_select_card_type(card);

	card->ext_csd.raw_drive_strength = ext_csd[EXT_CSD_DRIVE_STRENGTH];

	card->ext_csd.raw_s_a_timeout = ext_csd[EXT_CSD_S_A_TIMEOUT];
	card->ext_csd.raw_erase_timeout_mult =
		ext_csd[EXT_CSD_ERASE_TIMEOUT_MULT];
	card->ext_csd.raw_hc_erase_grp_size =
		ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE];
	if (card->ext_csd.rev >= 3) {
		u8 sa_shift = ext_csd[EXT_CSD_S_A_TIMEOUT];
		card->ext_csd.part_config = ext_csd[EXT_CSD_PART_CONFIG];

		
		card->ext_csd.part_time = 10 * ext_csd[EXT_CSD_PART_SWITCH_TIME];

		
		if (sa_shift > 0 && sa_shift <= 0x17)
			card->ext_csd.sa_timeout =
					1 << ext_csd[EXT_CSD_S_A_TIMEOUT];
		card->ext_csd.erase_group_def =
			ext_csd[EXT_CSD_ERASE_GROUP_DEF];
		card->ext_csd.hc_erase_timeout = 300 *
			ext_csd[EXT_CSD_ERASE_TIMEOUT_MULT];
		card->ext_csd.hc_erase_size =
			ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE] << 10;

		card->ext_csd.rel_sectors = ext_csd[EXT_CSD_REL_WR_SEC_C];

		if (ext_csd[EXT_CSD_BOOT_MULT] && mmc_boot_partition_access(card->host)) {
			for (idx = 0; idx < MMC_NUM_BOOT_PARTITION; idx++) {
				part_size = ext_csd[EXT_CSD_BOOT_MULT] << 17;
				mmc_part_add(card, part_size,
					EXT_CSD_PART_CONFIG_ACC_BOOT0 + idx,
					"boot%d", idx, true,
					MMC_BLK_DATA_AREA_BOOT);
			}
		}
	}

	card->ext_csd.raw_hc_erase_gap_size =
		ext_csd[EXT_CSD_HC_WP_GRP_SIZE];
	card->ext_csd.raw_sec_trim_mult =
		ext_csd[EXT_CSD_SEC_TRIM_MULT];
	card->ext_csd.raw_sec_erase_mult =
		ext_csd[EXT_CSD_SEC_ERASE_MULT];
	card->ext_csd.raw_sec_feature_support =
		ext_csd[EXT_CSD_SEC_FEATURE_SUPPORT];
	card->ext_csd.raw_trim_mult =
		ext_csd[EXT_CSD_TRIM_MULT];
	if (card->ext_csd.rev >= 4) {
		card->ext_csd.raw_partition_support = ext_csd[EXT_CSD_PARTITION_SUPPORT];
		if ((ext_csd[EXT_CSD_PARTITION_SUPPORT] & 0x2) &&
		    (ext_csd[EXT_CSD_PARTITION_ATTRIBUTE] & 0x1)) {
			hc_erase_grp_sz =
				ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE];
			hc_wp_grp_sz =
				ext_csd[EXT_CSD_HC_WP_GRP_SIZE];

			card->ext_csd.enhanced_area_en = 1;
			card->ext_csd.enhanced_area_offset =
				(ext_csd[139] << 24) + (ext_csd[138] << 16) +
				(ext_csd[137] << 8) + ext_csd[136];
			if (mmc_card_blockaddr(card))
				card->ext_csd.enhanced_area_offset <<= 9;
			card->ext_csd.enhanced_area_size =
				(ext_csd[142] << 16) + (ext_csd[141] << 8) +
				ext_csd[140];
			card->ext_csd.enhanced_area_size *=
				(size_t)(hc_erase_grp_sz * hc_wp_grp_sz);
			card->ext_csd.enhanced_area_size <<= 9;
		} else {
			card->ext_csd.enhanced_area_offset = -EINVAL;
			card->ext_csd.enhanced_area_size = -EINVAL;
		}

		if (ext_csd[EXT_CSD_PARTITION_SUPPORT] &
			EXT_CSD_PART_SUPPORT_PART_EN) {
			if (card->ext_csd.enhanced_area_en != 1) {
				hc_erase_grp_sz =
					ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE];
				hc_wp_grp_sz =
					ext_csd[EXT_CSD_HC_WP_GRP_SIZE];

				card->ext_csd.enhanced_area_en = 1;
			}

			for (idx = 0; idx < MMC_NUM_GP_PARTITION; idx++) {
				if (!ext_csd[EXT_CSD_GP_SIZE_MULT + idx * 3] &&
				!ext_csd[EXT_CSD_GP_SIZE_MULT + idx * 3 + 1] &&
				!ext_csd[EXT_CSD_GP_SIZE_MULT + idx * 3 + 2])
					continue;
				part_size =
				(ext_csd[EXT_CSD_GP_SIZE_MULT + idx * 3 + 2]
					<< 16) +
				(ext_csd[EXT_CSD_GP_SIZE_MULT + idx * 3 + 1]
					<< 8) +
				ext_csd[EXT_CSD_GP_SIZE_MULT + idx * 3];
				part_size *= (size_t)(hc_erase_grp_sz *
					hc_wp_grp_sz);
				mmc_part_add(card, part_size << 19,
					EXT_CSD_PART_CONFIG_ACC_GP0 + idx,
					"gp%d", idx, false,
					MMC_BLK_DATA_AREA_GP);
			}
		}
		card->ext_csd.sec_trim_mult =
			ext_csd[EXT_CSD_SEC_TRIM_MULT];
		card->ext_csd.sec_erase_mult =
			ext_csd[EXT_CSD_SEC_ERASE_MULT];
		card->ext_csd.sec_feature_support =
			ext_csd[EXT_CSD_SEC_FEATURE_SUPPORT];
		card->ext_csd.trim_timeout = 300 *
			ext_csd[EXT_CSD_TRIM_MULT];

		card->ext_csd.boot_ro_lock = ext_csd[EXT_CSD_BOOT_WP];
		card->ext_csd.boot_ro_lockable = true;
	}

	if (card->ext_csd.rev >= 5) {
		
		if ((ext_csd[EXT_CSD_HPI_FEATURES] & 0x1) &&
				!(card->quirks & MMC_QUIRK_BROKEN_HPI)) {
			card->ext_csd.hpi = 1;
			if (ext_csd[EXT_CSD_HPI_FEATURES] & 0x2)
				card->ext_csd.hpi_cmd = MMC_STOP_TRANSMISSION;
			else
				card->ext_csd.hpi_cmd = MMC_SEND_STATUS;
			card->ext_csd.out_of_int_time =
				ext_csd[EXT_CSD_OUT_OF_INTERRUPT_TIME] * 10;
		}

		if ((ext_csd[EXT_CSD_BKOPS_SUPPORT] & 0x1) &&
		    card->ext_csd.hpi) {
			card->ext_csd.bkops = 1;
			card->ext_csd.bkops_en = ext_csd[EXT_CSD_BKOPS_EN];
			card->ext_csd.raw_bkops_status =
				ext_csd[EXT_CSD_BKOPS_STATUS];
			if (!card->ext_csd.bkops_en &&
				card->host->caps2 & MMC_CAP2_INIT_BKOPS) {
				err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
					EXT_CSD_BKOPS_EN, 1, 0);
				if (err)
					pr_warn("%s: Enabling BKOPS failed\n",
						mmc_hostname(card->host));
				else
					card->ext_csd.bkops_en = 1;
			}
		}

		pr_info("%s: BKOPS_EN bit = %d\n",
			mmc_hostname(card->host), card->ext_csd.bkops_en);

		card->ext_csd.rel_param = ext_csd[EXT_CSD_WR_REL_PARAM];
		card->ext_csd.rst_n_function = ext_csd[EXT_CSD_RST_N_FUNCTION];

		card->ext_csd.raw_rpmb_size_mult = ext_csd[EXT_CSD_RPMB_MULT];
		if (ext_csd[EXT_CSD_RPMB_MULT]) {
			mmc_part_add(card, ext_csd[EXT_CSD_RPMB_MULT] << 17,
				EXT_CSD_PART_CONFIG_ACC_RPMB,
				"rpmb", 0, false,
				MMC_BLK_DATA_AREA_RPMB);
		}
	}

	card->ext_csd.raw_erased_mem_count = ext_csd[EXT_CSD_ERASED_MEM_CONT];
	if (ext_csd[EXT_CSD_ERASED_MEM_CONT])
		card->erased_byte = 0xFF;
	else
		card->erased_byte = 0x0;

	
	if (card->ext_csd.rev >= 6) {
		card->ext_csd.feature_support |= MMC_DISCARD_FEATURE;

		card->ext_csd.generic_cmd6_time = 10 *
			ext_csd[EXT_CSD_GENERIC_CMD6_TIME];
		card->ext_csd.power_off_longtime = 10 *
			ext_csd[EXT_CSD_POWER_OFF_LONG_TIME];

		card->ext_csd.cache_size =
			ext_csd[EXT_CSD_CACHE_SIZE + 0] << 0 |
			ext_csd[EXT_CSD_CACHE_SIZE + 1] << 8 |
			ext_csd[EXT_CSD_CACHE_SIZE + 2] << 16 |
			ext_csd[EXT_CSD_CACHE_SIZE + 3] << 24;

		if (ext_csd[EXT_CSD_DATA_SECTOR_SIZE] == 1)
			card->ext_csd.data_sector_size = 4096;
		else
			card->ext_csd.data_sector_size = 512;

		if ((ext_csd[EXT_CSD_DATA_TAG_SUPPORT] & 1) &&
		    (ext_csd[EXT_CSD_TAG_UNIT_SIZE] <= 8)) {
			card->ext_csd.data_tag_unit_size =
			((unsigned int) 1 << ext_csd[EXT_CSD_TAG_UNIT_SIZE]) *
			(card->ext_csd.data_sector_size);
		} else {
			card->ext_csd.data_tag_unit_size = 0;
		}

		card->ext_csd.max_packed_writes =
			ext_csd[EXT_CSD_MAX_PACKED_WRITES];
		card->ext_csd.max_packed_reads =
			ext_csd[EXT_CSD_MAX_PACKED_READS];
	}

	if (mmc_card_mmc(card)) {
		char *buf;
		int i, j;
		ssize_t n = 0;
		pr_info("%s: cid %08x%08x%08x%08x\n",
				mmc_hostname(card->host),
				card->raw_cid[0], card->raw_cid[1],
				card->raw_cid[2], card->raw_cid[3]);
		pr_info("%s: csd %08x%08x%08x%08x\n",
				mmc_hostname(card->host),
				card->raw_csd[0], card->raw_csd[1],
				card->raw_csd[2], card->raw_csd[3]);

		buf = kmalloc(512, GFP_KERNEL);
		if (buf) {
			for (i = 0; i < 32; i++) {
				for (j = 511 - (16 * i); j >= 496 - (16 * i); j--)
					n += sprintf(buf + n, "%02x", ext_csd[j]);
				n += sprintf(buf + n, "\n");
				pr_info("%s: ext_csd %s", mmc_hostname(card->host), buf);
				n = 0;
			}
		}
		if (buf)
			kfree(buf);

		
		if (card->cid.manfid == CID_MANFID_SANDISK ||
		    card->cid.manfid == CID_MANFID_SANDISK_2) {
			if (card->ext_csd.rev == 6)
				card->cid.fwrev =
				ext_csd[EXT_CSD_VENDOR_SPECIFIC_FIELDS_73] & 0x3F;
			
		}
		if ((card->cid.manfid == CID_MANFID_HYNIX) && !strncmp(card->cid.prod_name, "HAG2e", 5)
			&& (card->cid.fwrev < 6)) {
			pr_info("%s: disable urgent request for Hynix eMMC(fwrev %d)\n",
				mmc_hostname(card->host), card->cid.fwrev);
			card->quirks |= MMC_QUIRK_URGENT_REQUEST_DISABLE;
		}
	}

out:
	return err;
}

static inline void mmc_free_ext_csd(u8 *ext_csd)
{
	kfree(ext_csd);
}


static int mmc_compare_ext_csds(struct mmc_card *card, unsigned bus_width)
{
	u8 *bw_ext_csd;
	int err;

	if (bus_width == MMC_BUS_WIDTH_1)
		return 0;

	err = mmc_get_ext_csd(card, &bw_ext_csd);

	if (err || bw_ext_csd == NULL) {
		if (bus_width != MMC_BUS_WIDTH_1)
			err = -EINVAL;
		goto out;
	}

	if (bus_width == MMC_BUS_WIDTH_1)
		goto out;

	
	err = !((card->ext_csd.raw_partition_support ==
			bw_ext_csd[EXT_CSD_PARTITION_SUPPORT]) &&
		(card->ext_csd.raw_erased_mem_count ==
			bw_ext_csd[EXT_CSD_ERASED_MEM_CONT]) &&
		(card->ext_csd.rev ==
			bw_ext_csd[EXT_CSD_REV]) &&
		(card->ext_csd.raw_ext_csd_structure ==
			bw_ext_csd[EXT_CSD_STRUCTURE]) &&
		(card->ext_csd.raw_card_type ==
			bw_ext_csd[EXT_CSD_CARD_TYPE]) &&
		(card->ext_csd.raw_s_a_timeout ==
			bw_ext_csd[EXT_CSD_S_A_TIMEOUT]) &&
		(card->ext_csd.raw_hc_erase_gap_size ==
			bw_ext_csd[EXT_CSD_HC_WP_GRP_SIZE]) &&
		(card->ext_csd.raw_erase_timeout_mult ==
			bw_ext_csd[EXT_CSD_ERASE_TIMEOUT_MULT]) &&
		(card->ext_csd.raw_hc_erase_grp_size ==
			bw_ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE]) &&
		(card->ext_csd.raw_sec_trim_mult ==
			bw_ext_csd[EXT_CSD_SEC_TRIM_MULT]) &&
		(card->ext_csd.raw_sec_erase_mult ==
			bw_ext_csd[EXT_CSD_SEC_ERASE_MULT]) &&
		(card->ext_csd.raw_sec_feature_support ==
			bw_ext_csd[EXT_CSD_SEC_FEATURE_SUPPORT]) &&
		(card->ext_csd.raw_trim_mult ==
			bw_ext_csd[EXT_CSD_TRIM_MULT]) &&
		(card->ext_csd.raw_sectors[0] ==
			bw_ext_csd[EXT_CSD_SEC_CNT + 0]) &&
		(card->ext_csd.raw_sectors[1] ==
			bw_ext_csd[EXT_CSD_SEC_CNT + 1]) &&
		(card->ext_csd.raw_sectors[2] ==
			bw_ext_csd[EXT_CSD_SEC_CNT + 2]) &&
		(card->ext_csd.raw_sectors[3] ==
			bw_ext_csd[EXT_CSD_SEC_CNT + 3]));
	if (err)
		err = -EINVAL;

out:
	mmc_free_ext_csd(bw_ext_csd);
	return err;
}

MMC_DEV_ATTR(cid, "%08x%08x%08x%08x\n", card->raw_cid[0], card->raw_cid[1],
	card->raw_cid[2], card->raw_cid[3]);
MMC_DEV_ATTR(csd, "%08x%08x%08x%08x\n", card->raw_csd[0], card->raw_csd[1],
	card->raw_csd[2], card->raw_csd[3]);
MMC_DEV_ATTR(date, "%02d/%04d\n", card->cid.month, card->cid.year);
MMC_DEV_ATTR(erase_size, "%u\n", card->erase_size << 9);
MMC_DEV_ATTR(preferred_erase_size, "%u\n", card->pref_erase << 9);
MMC_DEV_ATTR(fwrev, "0x%x\n", card->cid.fwrev);
MMC_DEV_ATTR(hwrev, "0x%x\n", card->cid.hwrev);
MMC_DEV_ATTR(manfid, "0x%06x\n", card->cid.manfid);
MMC_DEV_ATTR(name, "%s\n", card->cid.prod_name);
MMC_DEV_ATTR(oemid, "0x%04x\n", card->cid.oemid);
MMC_DEV_ATTR(serial, "0x%08x\n", card->cid.serial);
MMC_DEV_ATTR(enhanced_area_offset, "%llu\n",
		card->ext_csd.enhanced_area_offset);
MMC_DEV_ATTR(enhanced_area_size, "%u\n", card->ext_csd.enhanced_area_size);
MMC_DEV_ATTR(raw_rpmb_size_mult, "%#x\n", card->ext_csd.raw_rpmb_size_mult);
MMC_DEV_ATTR(rel_sectors, "%#x\n", card->ext_csd.rel_sectors);

static ssize_t mmc_manf_name_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mmc_card *card = mmc_dev_to_card(dev);
	int count = 0;

	switch (card->cid.manfid) {
	case CID_MANFID_SANDISK:
	case CID_MANFID_SANDISK_2:
		count = sprintf(buf, "Sandisk\n");
		break;
	case CID_MANFID_TOSHIBA:
		count = sprintf(buf, "Toshiba\n");
		break;
	case CID_MANFID_MICRON:
		count = sprintf(buf, "Micron\n");
		break;
	case CID_MANFID_SAMSUNG:
		count = sprintf(buf, "Samsung\n");
		break;
	case CID_MANFID_HYNIX:
		count = sprintf(buf, "Hynix\n");
		break;
	default:
		count = sprintf(buf, "Unknown\n");
	}

	return count;
}
DEVICE_ATTR(manf_name, S_IRUGO, mmc_manf_name_show, NULL);


static struct attribute *mmc_std_attrs[] = {
	&dev_attr_cid.attr,
	&dev_attr_csd.attr,
	&dev_attr_date.attr,
	&dev_attr_erase_size.attr,
	&dev_attr_preferred_erase_size.attr,
	&dev_attr_fwrev.attr,
	&dev_attr_hwrev.attr,
	&dev_attr_manfid.attr,
	&dev_attr_name.attr,
	&dev_attr_oemid.attr,
	&dev_attr_serial.attr,
	&dev_attr_enhanced_area_offset.attr,
	&dev_attr_enhanced_area_size.attr,
	&dev_attr_raw_rpmb_size_mult.attr,
	&dev_attr_rel_sectors.attr,
	&dev_attr_manf_name.attr,
	NULL,
};

static struct attribute_group mmc_std_attr_group = {
	.attrs = mmc_std_attrs,
};

static const struct attribute_group *mmc_attr_groups[] = {
	&mmc_std_attr_group,
	NULL,
};

static struct device_type mmc_type = {
	.groups = mmc_attr_groups,
};

static int mmc_select_powerclass(struct mmc_card *card,
		unsigned int bus_width, u8 *ext_csd)
{
	int err = 0;
	unsigned int pwrclass_val;
	unsigned int index = 0;
	struct mmc_host *host;

	BUG_ON(!card);

	host = card->host;
	BUG_ON(!host);

	if (ext_csd == NULL)
		return 0;

	
	if (card->csd.mmca_vsn < CSD_SPEC_VER_4)
		return 0;

	
	if (bus_width == EXT_CSD_BUS_WIDTH_1)
		return 0;

	switch (1 << host->ios.vdd) {
	case MMC_VDD_165_195:
		if (host->ios.clock <= 26000000)
			index = EXT_CSD_PWR_CL_26_195;
		else if	(host->ios.clock <= 52000000)
			index = (bus_width <= EXT_CSD_BUS_WIDTH_8) ?
				EXT_CSD_PWR_CL_52_195 :
				EXT_CSD_PWR_CL_DDR_52_195;
		else if (host->ios.clock <= 200000000)
			index = (bus_width <= EXT_CSD_BUS_WIDTH_8) ?
				EXT_CSD_PWR_CL_200_195 :
				EXT_CSD_PWR_CL_DDR_200_195;
		break;
	case MMC_VDD_27_28:
	case MMC_VDD_28_29:
	case MMC_VDD_29_30:
	case MMC_VDD_30_31:
	case MMC_VDD_31_32:
	case MMC_VDD_32_33:
	case MMC_VDD_33_34:
	case MMC_VDD_34_35:
	case MMC_VDD_35_36:
		if (host->ios.clock <= 26000000)
			index = EXT_CSD_PWR_CL_26_360;
		else if	(host->ios.clock <= 52000000)
			index = (bus_width <= EXT_CSD_BUS_WIDTH_8) ?
				EXT_CSD_PWR_CL_52_360 :
				EXT_CSD_PWR_CL_DDR_52_360;
		else if (host->ios.clock <= 200000000)
			index = (bus_width <= EXT_CSD_BUS_WIDTH_8) ?
				EXT_CSD_PWR_CL_200_360 :
				EXT_CSD_PWR_CL_DDR_200_360;
		break;
	default:
		pr_warning("%s: Voltage range not supported "
			   "for power class.\n", mmc_hostname(host));
		return -EINVAL;
	}

	pwrclass_val = ext_csd[index];

	if (bus_width & (EXT_CSD_BUS_WIDTH_8 | EXT_CSD_DDR_BUS_WIDTH_8))
		pwrclass_val = (pwrclass_val & EXT_CSD_PWR_CL_8BIT_MASK) >>
				EXT_CSD_PWR_CL_8BIT_SHIFT;
	else
		pwrclass_val = (pwrclass_val & EXT_CSD_PWR_CL_4BIT_MASK) >>
				EXT_CSD_PWR_CL_4BIT_SHIFT;

	
	if (pwrclass_val > 0) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_POWER_CLASS,
				 pwrclass_val,
				 card->ext_csd.generic_cmd6_time);
	}

	return err;
}

static int mmc_select_bus_width(struct mmc_card *card, int ddr, u8 *ext_csd)
{
	struct mmc_host *host;
	int retry = 3;
	static unsigned ext_csd_bits[][2] = {
		{ EXT_CSD_BUS_WIDTH_8, EXT_CSD_DDR_BUS_WIDTH_8 },
		{ EXT_CSD_BUS_WIDTH_4, EXT_CSD_DDR_BUS_WIDTH_4 },
		{ EXT_CSD_BUS_WIDTH_1, EXT_CSD_BUS_WIDTH_1 },
	};
	static unsigned bus_widths[] = {
		MMC_BUS_WIDTH_8,
		MMC_BUS_WIDTH_4,
		MMC_BUS_WIDTH_1
	};
	unsigned idx, bus_width = 0;
	int err = 0;

	host = card->host;

	if ((card->csd.mmca_vsn < CSD_SPEC_VER_4) ||
	    !(host->caps & (MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA)))
		goto out;

	if (host->caps & MMC_CAP_8_BIT_DATA)
		idx = 0;
	else
		idx = 1;

	for (; idx < ARRAY_SIZE(bus_widths); idx++) {
		bus_width = bus_widths[idx];
		if (bus_width == MMC_BUS_WIDTH_1)
			ddr = 0; 
		err = mmc_select_powerclass(card, ext_csd_bits[idx][0],
					    ext_csd);
		if (err)
			pr_warning("%s: power class selection to " \
				   "bus width %d failed\n",
				   mmc_hostname(host),
				   1 << bus_width);

do_retry:
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_BUS_WIDTH,
				 ext_csd_bits[idx][0],
				 card->ext_csd.generic_cmd6_time);
		if (!err) {
			mmc_set_bus_width(host, bus_width);

			if (!(host->caps & MMC_CAP_BUS_WIDTH_TEST)) {
				err = mmc_compare_ext_csds(card, bus_width);
				if (err) {
					pr_err("%s: compare_ext_csd return %d, bus width %d, retry %d\n",
						mmc_hostname(card->host), err, 1 << bus_width, retry);
					if (retry-- > 0)
						goto do_retry;
				}
			} else
				err = mmc_bus_test(card, bus_width);
			if (!err)
				break;
		}
	}

	if (!err && ddr) {
		err = mmc_select_powerclass(card, ext_csd_bits[idx][1],
					    ext_csd);
		if (err)
			pr_warning("%s: power class selection to " \
				   "bus width %d ddr %d failed\n",
				   mmc_hostname(host),
				   1 << bus_width, ddr);
			err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
					 EXT_CSD_BUS_WIDTH,
					 ext_csd_bits[idx][1],
					 card->ext_csd.generic_cmd6_time);
	}

out:
	return err;
}

static int mmc_select_hs(struct mmc_card *card, u8 *ext_csd)
{
	int err = 0;
	struct mmc_host *host;

	host = card->host;

	if (!(host->caps & MMC_CAP_MMC_HIGHSPEED) ||
		!(card->ext_csd.card_type & EXT_CSD_CARD_TYPE_52)) {
		err = -EOPNOTSUPP;
		goto out;
	}

	err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				EXT_CSD_HS_TIMING, 1,
				card->ext_csd.generic_cmd6_time);

	if (err && err != -EBADMSG)
		goto out;

	mmc_card_set_highspeed(card);
	mmc_set_timing(host, MMC_TIMING_MMC_HS);
	mmc_set_clock(host, MMC_HIGH_52_MAX_DTR);

	err = mmc_select_bus_width(card, 0, ext_csd);

out:
	if (err && err != -EOPNOTSUPP)
		pr_warning("%s: Switch to HighSpeed mode failed (err:%d)\n",
				mmc_hostname(host), err);
	return err;
}

static int mmc_select_hsddr(struct mmc_card *card, u8 *ext_csd)
{
	int ddr = 0, err = 0;
	struct mmc_host *host;

	host = card->host;

	if (!(host->caps & MMC_CAP_HSDDR) ||
		!(card->ext_csd.card_type & EXT_CSD_CARD_TYPE_DDR_52)) {
		err = -EOPNOTSUPP;
		goto out;
	}

	err = mmc_select_hs(card, ext_csd);
	if (err)
		goto out;
	mmc_card_clr_highspeed(card);

	if ((card->ext_csd.card_type & EXT_CSD_CARD_TYPE_DDR_1_8V)
		&& ((host->caps & (MMC_CAP_1_8V_DDR |
		     MMC_CAP_UHS_DDR50))
			== (MMC_CAP_1_8V_DDR | MMC_CAP_UHS_DDR50)))
			ddr = MMC_1_8V_DDR_MODE;
	else if ((card->ext_csd.card_type & EXT_CSD_CARD_TYPE_DDR_1_2V)
		&& ((host->caps & (MMC_CAP_1_2V_DDR |
		     MMC_CAP_UHS_DDR50))
			== (MMC_CAP_1_2V_DDR | MMC_CAP_UHS_DDR50)))
			ddr = MMC_1_2V_DDR_MODE;

	err = mmc_select_bus_width(card, ddr, ext_csd);
	if (err)
		goto out;

	if (host->ios.bus_width == MMC_BUS_WIDTH_1) {
		pr_err("%s: failed to switch to wide bus\n",
			mmc_hostname(host));
		goto out;
	}

	if (ddr == MMC_1_2V_DDR_MODE) {
		err = mmc_set_signal_voltage(host,
			MMC_SIGNAL_VOLTAGE_120, 0);
		if (err)
			goto out;
	}
	mmc_card_set_ddr_mode(card);
	mmc_set_timing(host, MMC_TIMING_UHS_DDR50);
	mmc_set_bus_width(host, host->ios.bus_width);

out:
	if (err && err != -EOPNOTSUPP)
		pr_warning("%s: Switch to HighSpeed DDR mode failed (err:%d)\n",
				mmc_hostname(host), err);
	return err;
}

static int mmc_select_hs200(struct mmc_card *card, u8 *ext_csd)
{
	int err = 0;
	struct mmc_host *host;

	host = card->host;

	if (!(host->caps2 & MMC_CAP2_HS200) ||
		!(card->ext_csd.card_type & EXT_CSD_CARD_TYPE_HS200)) {
		err = -EOPNOTSUPP;
		goto out;
	}

	if (card->ext_csd.card_type & EXT_CSD_CARD_TYPE_SDR_1_2V &&
	    host->caps2 & MMC_CAP2_HS200_1_2V_SDR)
		if (mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_120, 0))
			err = mmc_set_signal_voltage(host,
						     MMC_SIGNAL_VOLTAGE_180, 0);
	
	if (err)
		goto out;

	err = mmc_select_bus_width(card, 0, ext_csd);
	if (err) {
		pr_err("%s: select bus width failed\n",
			mmc_hostname(host));
		goto out;
	}

	if (host->ios.bus_width == MMC_BUS_WIDTH_1) {
		pr_err("%s: failed to switch to wide bus\n",
			mmc_hostname(host));
		goto out;
	}

	
	err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				EXT_CSD_HS_TIMING, 2, 0);

	if (err && err != -EBADMSG) {
		pr_err("%s: HS200 switch failed\n",
			mmc_hostname(host));
		goto out;
	}

	if (mmc_card_hs400(card))
		mmc_set_timing(host, MMC_TIMING_MMC_HS400);
	else
		mmc_set_timing(host, MMC_TIMING_MMC_HS200);

	mmc_set_clock(host, MMC_HS200_MAX_DTR);

	if (host->ops->execute_tuning) {
		mmc_host_clk_hold(host);
		err = host->ops->execute_tuning(host,
				MMC_SEND_TUNING_BLOCK_HS200);
		mmc_host_clk_release(host);
	}
	if (err) {
		pr_warning("%s: tuning execution failed\n",
			   mmc_hostname(host));
		goto out;
	}
	mmc_card_set_hs200(card);

	if (card->cid.manfid == SAMSUNG_MMC)
		mmc_send_single_read(card, host, 0);

out:
	if (err && err != -EOPNOTSUPP)
		pr_warning("%s: Switch to HS200 mode failed (err:%d)\n",
				mmc_hostname(host), err);
	return err;
}

static int mmc_select_hs400(struct mmc_card *card, u8 *ext_csd)
{
	int err = 0;
	struct mmc_host *host;

	host = card->host;

	if (!(host->caps2 & MMC_CAP2_HS400) ||
		!(card->ext_csd.card_type & EXT_CSD_CARD_TYPE_HS400)) {
		err = -EOPNOTSUPP;
		goto out;
	}

	mmc_card_set_hs400(card);
	err = mmc_select_hs200(card, ext_csd);
	if (err)
		goto out;
	mmc_card_clr_hs200(card);

	if ((card->ext_csd.card_type & EXT_CSD_CARD_TYPE_HS400_1_2V)
	    && (host->caps2 & MMC_CAP2_HS400_1_2V))
		if (mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_120, 0))
				err = mmc_set_signal_voltage(host,
						MMC_SIGNAL_VOLTAGE_180, 0);
	
	if (err)
		goto out;

	mmc_set_timing(host, MMC_TIMING_LEGACY);
	mmc_set_clock(host, MMC_HIGH_26_MAX_DTR);

	err = mmc_select_hs(card, ext_csd);
	if (err)
		goto out;
	mmc_card_clr_highspeed(card);

	
	err = mmc_select_hsddr(card, ext_csd);
	if (err)
		goto out;
	mmc_card_clr_ddr_mode(card);

	if (host->ios.bus_width != MMC_BUS_WIDTH_8) {
		pr_err("%s: failed to switch to 8-bit bus width\n",
			mmc_hostname(host));
		goto out;
	}

	
	err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_HS_TIMING, 3, 0);
	if (err && err != -EBADMSG) {
		pr_err("%s: Setting HS_TIMING to HS400 failed (err:%d)\n",
			mmc_hostname(host), err);
		goto out;
	}

	mmc_set_timing(host, MMC_TIMING_MMC_HS400);
	mmc_set_clock(host, MMC_HS400_MAX_DTR);

	if (host->ops->execute_tuning) {
		mmc_host_clk_hold(host);
		err = host->ops->execute_tuning(host,
				MMC_SEND_TUNING_BLOCK_HS400);
		mmc_host_clk_release(host);
	}
	if (err) {
		pr_err("%s: tuning execution failed (err:%d)\n",
			   mmc_hostname(host), err);
		goto out;
	}
	mmc_card_set_hs400(card);

out:
	if (err && err != -EOPNOTSUPP) {
		pr_warning("%s: Switch to HS400 mode failed (err:%d)\n",
				mmc_hostname(host), err);
		mmc_card_clr_hs400(card);
	}
	return err;
}

int mmc_set_clock_bus_speed(struct mmc_card *card, unsigned long freq)
{
	int err;

	if (freq < MMC_HS400_MAX_DTR) {
		mmc_set_timing(card->host, MMC_TIMING_LEGACY);
		mmc_set_clock(card->host, MMC_HIGH_26_MAX_DTR);

		err = mmc_select_hs(card, card->cached_ext_csd);
	} else {
		err = mmc_select_hs400(card, card->cached_ext_csd);
	}

	return err;
}

static int mmc_change_bus_speed(struct mmc_host *host, unsigned long *freq)
{
	int err = 0;
	struct mmc_card *card;

	mmc_claim_host(host);
	card = host->card;

	if (!card || !freq) {
		err = -EINVAL;
		goto out;
	}

	if (mmc_card_highspeed(card) || mmc_card_hs200(card)
			|| mmc_card_ddr_mode(card)
			|| mmc_card_hs400(card)) {
		if (*freq > card->ext_csd.hs_max_dtr)
			*freq = card->ext_csd.hs_max_dtr;
	} else if (*freq > card->csd.max_dtr) {
		*freq = card->csd.max_dtr;
	}

	if (*freq < host->f_min)
		*freq = host->f_min;

	if (mmc_card_hs400(card)) {
		err = mmc_set_clock_bus_speed(card, *freq);
		if (err)
			goto out;
	} else {
		mmc_set_clock(host, (unsigned int) (*freq));
	}

	if (mmc_card_hs200(card) && card->host->ops->execute_tuning) {
		mmc_host_clk_hold(card->host);
		err = card->host->ops->execute_tuning(card->host,
				MMC_SEND_TUNING_BLOCK_HS200);
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

static int mmc_reboot_notify(struct notifier_block *notify_block,
		unsigned long event, void *unused)
{
	struct mmc_card *card = container_of(
			notify_block, struct mmc_card, reboot_notify);

	if (event != SYS_RESTART)
		card->issue_long_pon = true;
	else
		card->issue_long_pon = false;

	return NOTIFY_OK;
}

static int mmc_select_bus_speed(struct mmc_card *card, u8 *ext_csd)
{
	int err = 0;

	BUG_ON(!card);

	if (!mmc_select_hs400(card, ext_csd))
		goto out;
	if (!mmc_select_hs200(card, ext_csd))
		goto out;
	if (!mmc_select_hsddr(card, ext_csd))
		goto out;
	if (!mmc_select_hs(card, ext_csd))
		goto out;

	mmc_set_clock(card->host, card->csd.max_dtr);
	err = mmc_select_bus_width(card, 0, ext_csd);

out:
	return err;
}

static int mmc_init_card(struct mmc_host *host, u32 ocr,
	struct mmc_card *oldcard)
{
	struct mmc_card *card;
	int err = 0;
	u32 cid[4];
	u32 rocr;
	u8 *ext_csd = NULL;

	BUG_ON(!host);
	WARN_ON(!host->claimed);

	
	if (!mmc_host_is_spi(host))
		mmc_set_bus_mode(host, MMC_BUSMODE_OPENDRAIN);

	
	mmc_set_signal_voltage(host, MMC_SIGNAL_VOLTAGE_330, 0);

	mmc_go_idle(host);

	
	err = mmc_send_op_cond(host, ocr | (1 << 30), &rocr);
	if (err)
		goto err;

	if (mmc_host_is_spi(host)) {
		err = mmc_spi_set_crc(host, use_spi_crc);
		if (err)
			goto err;
	}

	if (mmc_host_is_spi(host))
		err = mmc_send_cid(host, cid);
	else
		err = mmc_all_send_cid(host, cid);
	if (err)
		goto err;

	if (oldcard) {
		if (memcmp(cid, oldcard->raw_cid, sizeof(cid)) != 0) {
			err = -ENOENT;
			goto err;
		}

		card = oldcard;
	} else {
		card = mmc_alloc_card(host, &mmc_type);
		if (IS_ERR(card)) {
			err = PTR_ERR(card);
			goto err;
		}

		card->type = MMC_TYPE_MMC;
		card->rca = 1;
		memcpy(card->raw_cid, cid, sizeof(card->raw_cid));
		card->reboot_notify.notifier_call = mmc_reboot_notify;
		host->card = card;
	}

	if (!mmc_host_is_spi(host)) {
		err = mmc_set_relative_addr(card);
		if (err)
			goto free_card;

		mmc_set_bus_mode(host, MMC_BUSMODE_PUSHPULL);
	}

	if (!oldcard) {
		err = mmc_send_csd(card, card->raw_csd);
		if (err)
			goto free_card;

		err = mmc_decode_csd(card);
		if (err)
			goto free_card;
		err = mmc_decode_cid(card);
		if (err)
			goto free_card;
	}

	if (!mmc_host_is_spi(host)) {
		err = mmc_select_card(card);
		if (err)
			goto free_card;
	}

	if (!oldcard) {

		err = mmc_get_ext_csd(card, &ext_csd);
		if (err)
			goto free_card;
		card->cached_ext_csd = ext_csd;
		err = mmc_read_ext_csd(card, ext_csd);
		if (err)
			goto free_card;

		if (!(mmc_card_blockaddr(card)) && (rocr & (1<<30)))
			mmc_card_set_blockaddr(card);

		
		mmc_set_erase_size(card);

		if (card->ext_csd.sectors && (rocr & MMC_CARD_SECTOR_ADDR))
			mmc_card_set_blockaddr(card);
	}

	if (card->ext_csd.enhanced_area_en ||
	    (card->ext_csd.rev >= 3 && (host->caps2 & MMC_CAP2_HC_ERASE_SZ))) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_ERASE_GROUP_DEF, 1,
				 card->ext_csd.generic_cmd6_time);

		if (err && err != -EBADMSG)
			goto free_card;

		if (err) {
			err = 0;
			card->ext_csd.enhanced_area_offset = -EINVAL;
			card->ext_csd.enhanced_area_size = -EINVAL;
		} else {
			card->ext_csd.erase_group_def = 1;
			mmc_set_erase_size(card);
		}
	}
#if 0
	
	if (card->ext_csd.rev >= 6) {
		
		if (card->cid.manfid == SAMSUNG_MMC) {
			if (card->ext_csd.sec_feature_support & EXT_CSD_SEC_SANITIZE)
				card->need_sanitize = 1;
			pr_info("%s: set need_sanitize\n", mmc_hostname(card->host));
		}
	}
#endif
	if (card->ext_csd.part_config & EXT_CSD_PART_CONFIG_ACC_MASK) {
		card->ext_csd.part_config &= ~EXT_CSD_PART_CONFIG_ACC_MASK;
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_PART_CONFIG,
				 card->ext_csd.part_config,
				 card->ext_csd.part_time);
		if (err && err != -EBADMSG)
			goto free_card;
		card->part_curr = card->ext_csd.part_config &
				  EXT_CSD_PART_CONFIG_ACC_MASK;
	}

	if ((host->caps2 & MMC_CAP2_POWEROFF_NOTIFY) &&
	    (card->ext_csd.rev >= 6)) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				 EXT_CSD_POWER_OFF_NOTIFICATION,
				 EXT_CSD_POWER_ON,
				 card->ext_csd.generic_cmd6_time);
		if (err && err != -EBADMSG)
			goto free_card;

		if (!err)
			card->ext_csd.power_off_notification = EXT_CSD_POWER_ON;
	}

	err = mmc_select_bus_speed(card, ext_csd);
	if (err)
		goto free_card;

	if (card->ext_csd.hpi) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				EXT_CSD_HPI_MGMT, 1,
				card->ext_csd.generic_cmd6_time);
		if (err && err != -EBADMSG)
			goto free_card;
		if (err) {
			pr_warning("%s: Enabling HPI failed\n",
				   mmc_hostname(card->host));
			err = 0;
		} else
			card->ext_csd.hpi_en = 1;
	}

	if ((host->caps2 & MMC_CAP2_CACHE_CTRL) &&
	    (card->ext_csd.cache_size > 0) && card->ext_csd.hpi_en &&
	    ((card->quirks & MMC_QUIRK_CACHE_DISABLE) == 0)) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				EXT_CSD_CACHE_CTRL, 1,
				card->ext_csd.generic_cmd6_time);
		if (err && err != -EBADMSG)
			goto free_card;

		if (err) {
			pr_warning("%s: Cache is supported, "
					"but failed to turn on (%d)\n",
					mmc_hostname(card->host), err);
			card->ext_csd.cache_ctrl = 0;
			err = 0;
		} else {
			card->ext_csd.cache_ctrl = 1;
		}
	}

	if (card->quirks & MMC_QUIRK_CACHE_DISABLE)
		card->ext_csd.cache_ctrl = 0;

	if ((host->caps2 & MMC_CAP2_PACKED_WR &&
			card->ext_csd.max_packed_writes > 0) ||
	    (host->caps2 & MMC_CAP2_PACKED_RD &&
			card->ext_csd.max_packed_reads > 0)) {
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				EXT_CSD_EXP_EVENTS_CTRL,
				EXT_CSD_PACKED_EVENT_EN,
				card->ext_csd.generic_cmd6_time);
		if (err && err != -EBADMSG)
			goto free_card;
		if (err) {
			pr_warning("%s: Enabling packed event failed\n",
					mmc_hostname(card->host));
			card->ext_csd.packed_event_en = 0;
			err = 0;
		} else {
			card->ext_csd.packed_event_en = 1;
		}

	}

	if (!oldcard) {
		if ((host->caps2 & MMC_CAP2_PACKED_CMD) &&
		    (card->ext_csd.max_packed_writes > 0)) {
			card->wr_pack_stats.packing_events = kzalloc(
				(card->ext_csd.max_packed_writes + 1) *
				sizeof(*card->wr_pack_stats.packing_events),
				GFP_KERNEL);
			if (!card->wr_pack_stats.packing_events)
				goto free_card;
		}

		if (card->ext_csd.bkops_en) {
			INIT_DELAYED_WORK(&card->bkops_info.dw,
					  mmc_start_idle_time_bkops);

			card->bkops_info.delay_ms = MMC_IDLE_BKOPS_TIME_MS;
			if (card->bkops_info.host_delay_ms)
				card->bkops_info.delay_ms =
					card->bkops_info.host_delay_ms;
		}
	}


	return 0;

free_card:
	if (!oldcard) {
		host->card = NULL;
		mmc_remove_card(card);
	}
err:
	return err;
}

static int mmc_can_poweroff_notify(const struct mmc_card *card)
{
	return card &&
		mmc_card_mmc(card) &&
		(card->ext_csd.power_off_notification == EXT_CSD_POWER_ON);
}

static int mmc_poweroff_notify(struct mmc_card *card, unsigned int notify_type)
{
	unsigned int timeout = card->ext_csd.generic_cmd6_time;
	int err;

	
	if (notify_type == EXT_CSD_POWER_OFF_LONG)
		timeout = card->ext_csd.power_off_longtime;

	err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			 EXT_CSD_POWER_OFF_NOTIFICATION,
			 notify_type, timeout);
	if (err)
		pr_err("%s: Power Off Notification timed out, %u\n",
		       mmc_hostname(card->host), timeout);

	
	card->ext_csd.power_off_notification = EXT_CSD_NO_POWER_NOTIFICATION;

	return err;
}

int mmc_send_long_pon(struct mmc_card *card)
{
	int err = 0;
	struct mmc_host *host = card->host;

	mmc_claim_host(host);
	if (card->issue_long_pon && mmc_can_poweroff_notify(card)) {
		err = mmc_poweroff_notify(host->card, EXT_CSD_POWER_OFF_LONG);
		if (err)
			pr_warning("%s: error %d sending Long PON",
					mmc_hostname(host), err);
	}
	mmc_release_host(host);
	return err;
}

static void mmc_remove(struct mmc_host *host)
{
	BUG_ON(!host);
	BUG_ON(!host->card);

	unregister_reboot_notifier(&host->card->reboot_notify);

	mmc_exit_clk_scaling(host);
	mmc_remove_card(host->card);

	mmc_claim_host(host);
	host->card = NULL;
	mmc_release_host(host);
}

static int mmc_alive(struct mmc_host *host)
{
	return mmc_send_status(host->card, NULL);
}

static void mmc_detect(struct mmc_host *host)
{
	int err;

	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_rpm_hold(host, &host->card->dev);
	mmc_claim_host(host);

	err = _mmc_detect_card_removed(host);

	mmc_release_host(host);

	if (!err)
		mmc_rpm_release(host, &host->card->dev);

	if (err) {
		mmc_remove(host);

		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_power_off(host);
		mmc_release_host(host);
	}
}

static int mmc_suspend(struct mmc_host *host)
{
	int err = 0;

	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_disable_clk_scaling(host);

	mmc_claim_host(host);

	if (host->card && mmc_card_need_bkops_in_suspend(host->card)) {
		pr_info("%s: Force bkops and let card not sleep\n",
				mmc_hostname(host));
		goto out;
	}

	err = mmc_cache_ctrl(host, 0);
	if (err)
		goto out;

	if (mmc_card_can_sleep(host))
		err = mmc_card_sleep(host);
	else if (!mmc_host_is_spi(host))
		mmc_deselect_cards(host);
	host->card->state &= ~(MMC_STATE_HIGHSPEED | MMC_STATE_HIGHSPEED_200);

out:
	mmc_release_host(host);
	return err;
}

static int mmc_reinit(struct mmc_host *host)
{
	int err;

	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_claim_host(host);
	err = mmc_init_card(host, host->ocr, host->card);
	mmc_release_host(host);

	return err;
}

static int mmc_resume(struct mmc_host *host)
{
	int err;

	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_claim_host(host);
	err = mmc_init_card(host, host->ocr, host->card);
	mmc_release_host(host);

	if (mmc_can_scale_clk(host))
		mmc_init_clk_scaling(host);

	return err;
}

static int mmc_power_restore(struct mmc_host *host)
{
	int ret;

	
	mmc_disable_clk_scaling(host);

	host->card->state &= ~(MMC_STATE_HIGHSPEED | MMC_STATE_HIGHSPEED_200);
	mmc_claim_host(host);
	ret = mmc_init_card(host, host->ocr, host->card);
	mmc_release_host(host);

	if (mmc_can_scale_clk(host))
		mmc_init_clk_scaling(host);

	return ret;
}

static int mmc_sleep(struct mmc_host *host)
{
	struct mmc_card *card = host->card;
	int err = -ENOSYS;

	if (card && card->ext_csd.rev >= 3) {
		err = mmc_card_sleepawake(host, 1);
		if (err < 0)
			pr_warn("%s: Error %d while putting card into sleep",
				 mmc_hostname(host), err);
	}

	return err;
}

static int mmc_awake(struct mmc_host *host)
{
	struct mmc_card *card = host->card;
	int err = -ENOSYS;

	if (card && card->ext_csd.rev >= 3) {
		err = mmc_card_sleepawake(host, 0);
		if (err < 0)
			pr_debug("%s: Error %d while awaking sleeping card",
				 mmc_hostname(host), err);
	}

	return err;
}

static const struct mmc_bus_ops mmc_ops = {
	.awake = mmc_awake,
	.sleep = mmc_sleep,
	.remove = mmc_remove,
	.detect = mmc_detect,
	.suspend = NULL,
	.resume = NULL,
	.power_restore = mmc_power_restore,
	.alive = mmc_alive,
	.change_bus_speed = mmc_change_bus_speed,
};

static const struct mmc_bus_ops mmc_ops_unsafe = {
	.awake = mmc_awake,
	.sleep = mmc_sleep,
	.remove = mmc_remove,
	.detect = mmc_detect,
	.suspend = mmc_suspend,
	.resume = mmc_resume,
	.reinit = mmc_reinit,
	.power_restore = mmc_power_restore,
	.alive = mmc_alive,
	.change_bus_speed = mmc_change_bus_speed,
};

static void mmc_attach_bus_ops(struct mmc_host *host)
{
	const struct mmc_bus_ops *bus_ops;

	if (!mmc_card_is_removable(host))
		bus_ops = &mmc_ops_unsafe;
	else
		bus_ops = &mmc_ops;
	mmc_attach_bus(host, bus_ops);
}

int mmc_attach_mmc(struct mmc_host *host)
{
	int err;
	u32 ocr;

	BUG_ON(!host);
	WARN_ON(!host->claimed);

	
	if (!mmc_host_is_spi(host))
		mmc_set_bus_mode(host, MMC_BUSMODE_OPENDRAIN);

	err = mmc_send_op_cond(host, 0, &ocr);
	if (err)
		return err;

	mmc_attach_bus_ops(host);
	if (host->ocr_avail_mmc)
		host->ocr_avail = host->ocr_avail_mmc;

	if (mmc_host_is_spi(host)) {
		err = mmc_spi_read_ocr(host, 1, &ocr);
		if (err)
			goto err;
	}

	if (ocr & 0x7F) {
		pr_warning("%s: card claims to support voltages "
		       "below the defined range. These will be ignored.\n",
		       mmc_hostname(host));
		ocr &= ~0x7F;
	}

	host->ocr = mmc_select_voltage(host, ocr);

	if (!host->ocr) {
		err = -EINVAL;
		goto err;
	}

	err = mmc_init_card(host, host->ocr, NULL);
	if (err)
		goto err;

	mmc_release_host(host);
	err = mmc_add_card(host->card);
	mmc_claim_host(host);
	if (err)
		goto remove_card;

	mmc_init_clk_scaling(host);

	register_reboot_notifier(&host->card->reboot_notify);

	return 0;

remove_card:
	mmc_release_host(host);
	mmc_remove_card(host->card);
	mmc_claim_host(host);
	host->card = NULL;
err:
	mmc_detach_bus(host);

	pr_err("%s: error %d whilst initialising MMC card\n",
		mmc_hostname(host), err);

	return err;
}
