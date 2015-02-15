/*
 * Definitions for rt5506 Headphone amp chip.
 */
#ifndef RT5506_H
#define RT5506_H

#include <linux/ioctl.h>
#include <linux/wakelock.h>
#include <mach/rpm-regulator.h>
#include <mach/rpm-regulator-smd.h>
#include <linux/regulator/consumer.h>

#define RT5506_I2C_NAME "rt5506"
#define MAX_REG_DATA 15

struct rt5506_platform_data {
	uint32_t gpio_rt5506_enable;
	const char *power_supply;
	struct rpm_regulator *power_reg;
};

struct rt5506_reg_data {
	unsigned char addr;
	unsigned char val;
};

struct rt5506_config {
	unsigned int reg_len;
        struct rt5506_reg_data reg[MAX_REG_DATA];
};

struct rt5506_comm_data {
	unsigned int out_mode;
        struct rt5506_config config;
};

struct rt5506_config_data {
	unsigned int mode_num;
	struct rt5506_comm_data *cmd_data;  /* [mode][mode_kind][reserve][cmds..] */
};

enum {
        AMP_INIT = 0,
        AMP_MUTE,
        AMP_MAX_FUNC
};

enum PLAYBACK_MODE {
	PLAYBACK_MODE_OFF = AMP_MAX_FUNC,
	PLAYBACK_MODE_PLAYBACK,
	PLAYBACK_MODE_PLAYBACK8OH,
	PLAYBACK_MODE_PLAYBACK16OH,
	PLAYBACK_MODE_PLAYBACK32OH,
	PLAYBACK_MODE_PLAYBACK64OH,
	PLAYBACK_MODE_PLAYBACK128OH,
	PLAYBACK_MODE_PLAYBACK256OH,
	PLAYBACK_MODE_PLAYBACK500OH,
	PLAYBACK_MODE_PLAYBACK1KOH,
	PLAYBACK_MODE_VOICE,
	PLAYBACK_MODE_TTY,
	PLAYBACK_MODE_FM,
	PLAYBACK_MODE_RING,
	PLAYBACK_MODE_MFG,
	PLAYBACK_MODE_BEATS_8_64,
	PLAYBACK_MODE_BEATS_128_500,
	PLAYBACK_MODE_MONO,
	PLAYBACK_MODE_MONO_BEATS,
	PLAYBACK_MAX_MODE
};

enum HEADSET_QUERY_STATUS {
    QUERY_OFF = 0,
    QUERY_HEADSET,
    QUERY_FINISH,
};


enum AMP_STATUS {
    STATUS_OFF = 0,
    STATUS_PLAYBACK,
    STATUS_SUSPEND,

};

enum HEADSET_OM {
    HEADSET_8OM = 0,
    HEADSET_16OM,
    HEADSET_32OM,
    HEADSET_64OM,
    HEADSET_128OM,
    HEADSET_256OM,
    HEADSET_500OM,
    HEADSET_1KOM,
    HEADSET_MONO,
    HEADSET_OM_UNDER_DETECT,
};

enum AMP_GPIO_STATUS {
     AMP_GPIO_OFF = 0,
     AMP_GPIO_ON,
     AMP_GPIO_QUERRTY_ON,
};

enum AMP_S4_STATUS {
     AMP_S4_AUTO = 0,
     AMP_S4_PWM,
};

#define QUERY_IMMED           msecs_to_jiffies(0)
#define QUERY_LATTER          msecs_to_jiffies(200)
#define AMP_SENSE_READY    0x80

#define AMP_IOCTL_MAGIC 'g'
#define AMP_SET_CONFIG	_IOW(AMP_IOCTL_MAGIC, 0x01,	unsigned)
#define AMP_READ_CONFIG	_IOW(AMP_IOCTL_MAGIC, 0x02, unsigned)
#define AMP_SET_MODE        _IOW(AMP_IOCTL_MAGIC, 0x03, unsigned)
#define AMP_SET_PARAM       _IOW(AMP_IOCTL_MAGIC, 0x04,  unsigned)
#define AMP_WRITE_REG       _IOW(AMP_IOCTL_MAGIC, 0x07,  unsigned)
#define AMP_QUERY_OM       _IOW(AMP_IOCTL_MAGIC, 0x08,  unsigned)

#endif

