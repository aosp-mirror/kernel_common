#ifndef __ASM_ARCH_MSM_DEVICES_CMDLINE_H
#define __ASM_ARCH_MSM_DEVICES_CMDLINE_H



enum {
	MFG_MODE_NORMAL,
	MFG_MODE_FACTORY2,
	MFG_MODE_RECOVERY,
	MFG_MODE_CHARGE,
	MFG_MODE_POWER_TEST,
	MFG_MODE_OFFMODE_CHARGING,
	MFG_MODE_MFGKERNEL_DIAG58,
	MFG_MODE_GIFT_MODE,
	MFG_MODE_MFGKERNEL,
	MFG_MODE_MINI,
};

enum {
	BUILD_MODE_SHIP,
	BUILD_MODE_MFG,
	BUILD_MODE_ENG,
};

int board_mfg_mode(void);
int board_build_flag(void);
char *board_serialno(void);
char *board_mid(void);
int board_is_super_cid(void);
unsigned int get_tamper_sf(void);
unsigned char *board_get_google_boot_reason(void);
unsigned int get_atsdebug(void);
#endif
