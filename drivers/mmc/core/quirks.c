/*
 *  This file contains work-arounds for many known SD/MMC
 *  and SDIO hardware bugs.
 *
 *  Copyright (c) 2011 Andrei Warkentin <andreiw@motorola.com>
 *  Copyright (c) 2011 Pierre Tardy <tardyp@gmail.com>
 *  Inspired from pci fixup code:
 *  Copyright (c) 1999 Martin Mares <mj@ucw.cz>
 *
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/mmc/card.h>

#ifndef SDIO_VENDOR_ID_TI
#define SDIO_VENDOR_ID_TI		0x0097
#endif

#ifndef SDIO_DEVICE_ID_TI_WL1271
#define SDIO_DEVICE_ID_TI_WL1271	0x4076
#endif

#ifndef SDIO_VENDOR_ID_STE
#define SDIO_VENDOR_ID_STE		0x0020
#endif

#ifndef SDIO_DEVICE_ID_STE_CW1200
#define SDIO_DEVICE_ID_STE_CW1200	0x2280
#endif

#ifndef SDIO_VENDOR_ID_MSM
#define SDIO_VENDOR_ID_MSM		0x0070
#endif

#ifndef SDIO_DEVICE_ID_MSM_WCN1314
#define SDIO_DEVICE_ID_MSM_WCN1314	0x2881
#endif

#ifndef SDIO_VENDOR_ID_MSM_QCA
#define SDIO_VENDOR_ID_MSM_QCA		0x271
#endif

#ifndef SDIO_DEVICE_ID_MSM_QCA_AR6003_1
#define SDIO_DEVICE_ID_MSM_QCA_AR6003_1	0x300
#endif

#ifndef SDIO_DEVICE_ID_MSM_QCA_AR6003_2
#define SDIO_DEVICE_ID_MSM_QCA_AR6003_2	0x301
#endif

#ifndef SDIO_DEVICE_ID_MSM_QCA_AR6004_1
#define SDIO_DEVICE_ID_MSM_QCA_AR6004_1	0x400
#endif

#ifndef SDIO_DEVICE_ID_MSM_QCA_AR6004_2
#define SDIO_DEVICE_ID_MSM_QCA_AR6004_2	0x401
#endif

/*
 * This hook just adds a quirk for all sdio devices
 */
static void add_quirk_for_sdio_devices(struct mmc_card *card, int data)
{
	if (mmc_card_sdio(card))
		card->quirks |= data;
}

static const struct mmc_fixup mmc_fixup_methods[] = {
	/* by default sdio devices are considered CLK_GATING broken */
	/* good cards will be whitelisted as they are tested */
	SDIO_FIXUP(SDIO_ANY_ID, SDIO_ANY_ID,
		   add_quirk_for_sdio_devices,
		   MMC_QUIRK_BROKEN_CLK_GATING),

	SDIO_FIXUP(SDIO_VENDOR_ID_TI, SDIO_DEVICE_ID_TI_WL1271,
		   remove_quirk, MMC_QUIRK_BROKEN_CLK_GATING),

	SDIO_FIXUP(SDIO_VENDOR_ID_MSM, SDIO_DEVICE_ID_MSM_WCN1314,
		   remove_quirk, MMC_QUIRK_BROKEN_CLK_GATING),

	SDIO_FIXUP(SDIO_VENDOR_ID_MSM_QCA, SDIO_DEVICE_ID_MSM_QCA_AR6003_1,
		   remove_quirk, MMC_QUIRK_BROKEN_CLK_GATING),

	SDIO_FIXUP(SDIO_VENDOR_ID_MSM_QCA, SDIO_DEVICE_ID_MSM_QCA_AR6003_2,
		   remove_quirk, MMC_QUIRK_BROKEN_CLK_GATING),

	SDIO_FIXUP(SDIO_VENDOR_ID_MSM_QCA, SDIO_DEVICE_ID_MSM_QCA_AR6004_1,
		   remove_quirk, MMC_QUIRK_BROKEN_CLK_GATING),

	SDIO_FIXUP(SDIO_VENDOR_ID_MSM_QCA, SDIO_DEVICE_ID_MSM_QCA_AR6004_2,
		   remove_quirk, MMC_QUIRK_BROKEN_CLK_GATING),

	SDIO_FIXUP(SDIO_VENDOR_ID_TI, SDIO_DEVICE_ID_TI_WL1271,
		   add_quirk, MMC_QUIRK_NONSTD_FUNC_IF),

	SDIO_FIXUP(SDIO_VENDOR_ID_TI, SDIO_DEVICE_ID_TI_WL1271,
		   add_quirk, MMC_QUIRK_DISABLE_CD),

	SDIO_FIXUP(SDIO_VENDOR_ID_STE, SDIO_DEVICE_ID_STE_CW1200,
		   add_quirk, MMC_QUIRK_BROKEN_BYTE_MODE_512),

	END_FIXUP
};

void mmc_fixup_device(struct mmc_card *card, const struct mmc_fixup *table)
{
	const struct mmc_fixup *f;
	u64 rev = cid_rev_card(card);

	/* Non-core specific workarounds. */
	if (!table)
		table = mmc_fixup_methods;

	for (f = table; f->vendor_fixup; f++) {
		if ((f->manfid == CID_MANFID_ANY ||
		     f->manfid == card->cid.manfid) &&
		    (f->oemid == CID_OEMID_ANY ||
		     f->oemid == card->cid.oemid) &&
		    (f->name == CID_NAME_ANY ||
		     !strncmp(f->name, card->cid.prod_name,
			      sizeof(card->cid.prod_name))) &&
		    (f->ext_csd_rev == EXT_CSD_REV_ANY ||
		     f->ext_csd_rev == card->ext_csd.rev) &&
		    (f->cis_vendor == card->cis.vendor ||
		     f->cis_vendor == (u16) SDIO_ANY_ID) &&
		    (f->cis_device == card->cis.device ||
		     f->cis_device == (u16) SDIO_ANY_ID) &&
		    rev >= f->rev_start && rev <= f->rev_end) {
			dev_dbg(&card->dev, "calling %pF\n", f->vendor_fixup);
			f->vendor_fixup(card, f->data);
		}
	}
}
EXPORT_SYMBOL(mmc_fixup_device);
