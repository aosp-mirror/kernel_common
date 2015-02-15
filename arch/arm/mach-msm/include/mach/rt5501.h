/*
 * Definitions for rt5501 Headphone amp chip.
 */
#ifndef RT5501_H
#define RT5501_H

#include <linux/ioctl.h>
#include <linux/wakelock.h>
#include <mach/rpm-regulator.h>
#include <mach/rpm-regulator-smd.h>
#include <linux/regulator/consumer.h>

#define RT5501_I2C_NAME "rt5501"
#define SPKR_OUTPUT 0
#define HEADSET_OUTPUT 1
#define DUAL_OUTPUT 2
#define HANDSET_OUTPUT 3
#define LINEOUT_OUTPUT 4
#define NO_OUTPUT 5
#define MODE_CMD_LEM 9
#define MAX_REG_DATA 15

struct rt5501_platform_data {
	uint32_t gpio_rt5501_enable;
	const char *power_supply;
	struct rpm_regulator *power_reg;
};

struct rt5501_reg_data {
	unsigned char addr;
	unsigned char val;
};

struct rt5501_config {
	unsigned int reg_len;
        struct rt5501_reg_data reg[MAX_REG_DATA];
};

struct rt5501_comm_data {
	unsigned int out_mode;
        struct rt5501_config config;
};

struct rt5501_config_data {
	unsigned int mode_num;
	struct rt5501_comm_data *cmd_data;  /* [mode][mode_kind][reserve][cmds..] */
};

enum {
        RT5501_INIT = 0,
        RT5501_MUTE,
        RT5501_MAX_FUNC
};

enum RT5501_Mode {
	RT5501_MODE_OFF = RT5501_MAX_FUNC,
	RT5501_MODE_PLAYBACK,
	RT5501_MODE_PLAYBACK8OH,
	RT5501_MODE_PLAYBACK16OH,
	RT5501_MODE_PLAYBACK32OH,
	RT5501_MODE_PLAYBACK64OH,
	RT5501_MODE_PLAYBACK128OH,
	RT5501_MODE_PLAYBACK256OH,
	RT5501_MODE_PLAYBACK500OH,
	RT5501_MODE_PLAYBACK1KOH,
	RT5501_MODE_VOICE,
	RT5501_MODE_TTY,
	RT5501_MODE_FM,
	RT5501_MODE_RING,
	RT5501_MODE_MFG,
	RT5501_MODE_BEATS_8_64,
	RT5501_MODE_BEATS_128_500,
	RT5501_MODE_MONO,
	RT5501_MODE_MONO_BEATS,
	RT5501_MAX_MODE
};

enum HEADSET_QUERY_STATUS{
    RT5501_QUERY_OFF = 0,
    RT5501_QUERY_HEADSET,
    RT5501_QUERY_FINISH,
};


enum RT5501_STATUS{
    RT5501_OFF = 0,
    RT5501_PLAYBACK,
    RT5501_SUSPEND,

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
#define RT5501_SENSE_READY    0x80

#define RT5501_IOCTL_MAGIC 'g'
#define RT5501_SET_CONFIG	_IOW(RT5501_IOCTL_MAGIC, 0x01,	unsigned)
#define RT5501_READ_CONFIG	_IOW(RT5501_IOCTL_MAGIC, 0x02, unsigned)
#define RT5501_SET_MODE        _IOW(RT5501_IOCTL_MAGIC, 0x03, unsigned)
#define RT5501_SET_PARAM       _IOW(RT5501_IOCTL_MAGIC, 0x04,  unsigned)
#define RT5501_WRITE_REG       _IOW(RT5501_IOCTL_MAGIC, 0x07,  unsigned)
#define RT5501_QUERY_OM       _IOW(RT5501_IOCTL_MAGIC, 0x08,  unsigned)

int query_rt5501(void);
void set_rt5501_amp(int on);
#endif

