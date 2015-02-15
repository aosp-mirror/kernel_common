/* arch/arm/mach-msm/include/mach/board.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2013, The Linux Foundation. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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

#ifndef __ASM_ARCH_MSM_BOARD_H
#define __ASM_ARCH_MSM_BOARD_H

#include <linux/types.h>
#include <linux/input.h>
#include <linux/usb.h>
#include <linux/leds-pmic8058.h>
#include <linux/clkdev.h>
#include <linux/of_platform.h>
#include <linux/msm_ssbi.h>
#include <mach/msm_bus.h>

#define WLAN_RF_REG_ADDR_START_OFFSET   0x3
#define WLAN_RF_REG_DATA_START_OFFSET   0xf
#define WLAN_RF_READ_REG_CMD            0x3
#define WLAN_RF_WRITE_REG_CMD           0x2
#define WLAN_RF_READ_CMD_MASK           0x3fff

struct msm_camera_io_ext {
	uint32_t mdcphy;
	uint32_t mdcsz;
	uint32_t appphy;
	uint32_t appsz;
	uint32_t camifpadphy;
	uint32_t camifpadsz;
	uint32_t csiphy;
	uint32_t csisz;
	uint32_t csiirq;
	uint32_t csiphyphy;
	uint32_t csiphysz;
	uint32_t csiphyirq;
	uint32_t ispifphy;
	uint32_t ispifsz;
	uint32_t ispifirq;
};

struct msm_camera_io_clk {
	uint32_t mclk_clk_rate;
	uint32_t vfe_clk_rate;
};

struct msm_cam_expander_info {
	struct i2c_board_info const *board_info;
	int bus_id;
};

struct msm_camera_device_platform_data {
	int (*camera_gpio_on) (void);
	void (*camera_gpio_off)(void);
	struct msm_camera_io_ext ioext;
	struct msm_camera_io_clk ioclk;
	uint8_t csid_core;
	uint8_t is_vpe;
	struct msm_bus_scale_pdata *cam_bus_scale_table;
	uint8_t csiphy_core;
};

#ifdef CONFIG_SENSORS_MT9T013
struct msm_camera_legacy_device_platform_data {
	int sensor_reset;
	int sensor_pwd;
	int vcm_pwd;
	void (*config_gpio_on) (void);
	void (*config_gpio_off)(void);
};
#endif

#define MSM_CAMERA_FLASH_NONE 0
#define MSM_CAMERA_FLASH_LED  1

#define MSM_CAMERA_FLASH_SRC_PMIC (0x00000001<<0)
#define MSM_CAMERA_FLASH_SRC_PWM  (0x00000001<<1)
#define MSM_CAMERA_FLASH_SRC_CURRENT_DRIVER	(0x00000001<<2)
#define MSM_CAMERA_FLASH_SRC_EXT     (0x00000001<<3)
#define MSM_CAMERA_FLASH_SRC_LED (0x00000001<<3)
#define MSM_CAMERA_FLASH_SRC_LED1 (0x00000001<<4)

struct msm_camera_sensor_flash_pmic {
	uint8_t num_of_src;
	uint32_t low_current;
	uint32_t high_current;
	enum pmic8058_leds led_src_1;
	enum pmic8058_leds led_src_2;
	int (*pmic_set_current)(enum pmic8058_leds id, unsigned mA);
};

struct msm_camera_sensor_flash_pwm {
	uint32_t freq;
	uint32_t max_load;
	uint32_t low_load;
	uint32_t high_load;
	uint32_t channel;
};

struct pmic8058_leds_platform_data;
struct msm_camera_sensor_flash_current_driver {
	uint32_t low_current;
	uint32_t high_current;
	const struct pmic8058_leds_platform_data *driver_channel;
};

enum msm_camera_ext_led_flash_id {
	MAM_CAMERA_EXT_LED_FLASH_SC628A,
	MAM_CAMERA_EXT_LED_FLASH_TPS61310,
};

struct msm_camera_sensor_flash_external {
	uint32_t led_en;
	uint32_t led_flash_en;
	enum msm_camera_ext_led_flash_id flash_id;
	struct msm_cam_expander_info *expander_info;
};

struct msm_camera_sensor_flash_led {
	const char *led_name;
	const int led_name_len;
};

struct msm_camera_sensor_flash_src {
	int flash_sr_type;
	struct gpio *init_gpio_tbl;
	uint8_t init_gpio_tbl_size;
	struct msm_gpio_set_tbl *set_gpio_tbl;
	uint8_t set_gpio_tbl_size;

	union {
		struct msm_camera_sensor_flash_pmic pmic_src;
		struct msm_camera_sensor_flash_pwm pwm_src;
		struct msm_camera_sensor_flash_current_driver
			current_driver_src;
		struct msm_camera_sensor_flash_external
			ext_driver_src;
		struct msm_camera_sensor_flash_led led_src;
	} _fsrc;
};

struct msm_camera_sensor_flash_data {
	int flash_type;
	struct msm_camera_sensor_flash_src *flash_src;
	struct i2c_board_info const *board_info;
	int bus_id;
	uint8_t flash_src_index;
};

struct msm_camera_sensor_strobe_flash_data {
	uint8_t flash_trigger;
	uint8_t flash_charge; 
	uint8_t flash_charge_done;
	uint32_t flash_recharge_duration;
	uint32_t irq;
	spinlock_t spin_lock;
	spinlock_t timer_lock;
	int state;
};

enum msm_camera_type {
	BACK_CAMERA_2D,
	FRONT_CAMERA_2D,
	BACK_CAMERA_3D,
	BACK_CAMERA_INT_3D,
};

enum msm_sensor_type {
	BAYER_SENSOR,
	YUV_SENSOR,
};

struct msm_gpio_set_tbl {
	unsigned gpio;
	unsigned long flags;
	uint32_t delay;
};

struct msm_camera_gpio_num_info {
	uint16_t gpio_num[20]; 
	uint8_t valid[20];
};

struct msm_camera_gpio_conf {
	void *cam_gpiomux_conf_tbl;
	uint8_t cam_gpiomux_conf_tbl_size;
	struct gpio *cam_gpio_common_tbl;
	uint8_t cam_gpio_common_tbl_size;
	struct gpio *cam_gpio_req_tbl;
	uint8_t cam_gpio_req_tbl_size;
	struct msm_gpio_set_tbl *cam_gpio_set_tbl;
	uint8_t cam_gpio_set_tbl_size;
	uint32_t gpio_no_mux;
	uint32_t *camera_off_table;
	uint8_t camera_off_table_size;
	uint32_t *camera_on_table;
	uint8_t camera_on_table_size;
	struct msm_camera_gpio_num_info *gpio_num_info;
};

enum msm_camera_i2c_mux_mode {
	MODE_R,
	MODE_L,
	MODE_DUAL
};

struct msm_camera_i2c_conf {
	uint8_t use_i2c_mux;
	struct platform_device *mux_dev;
	enum msm_camera_i2c_mux_mode i2c_mux_mode;
};

struct msm_camera_sensor_platform_info {
	int mount_angle;
	int sensor_reset;
	struct camera_vreg_t *cam_vreg;
	int num_vreg;
	int32_t (*ext_power_ctrl) (int enable);
	struct msm_camera_gpio_conf *gpio_conf;
	struct msm_camera_i2c_conf *i2c_conf;
	struct msm_camera_csi_lane_params *csi_lane_params;
};

enum msm_camera_actuator_name {
	MSM_ACTUATOR_MAIN_CAM_0,
	MSM_ACTUATOR_MAIN_CAM_1,
	MSM_ACTUATOR_MAIN_CAM_2,
	MSM_ACTUATOR_MAIN_CAM_3,
	MSM_ACTUATOR_MAIN_CAM_4,
	MSM_ACTUATOR_MAIN_CAM_5,
	MSM_ACTUATOR_WEB_CAM_0,
	MSM_ACTUATOR_WEB_CAM_1,
	MSM_ACTUATOR_WEB_CAM_2,
};

struct msm_actuator_info {
	struct i2c_board_info const *board_info;
	enum msm_camera_actuator_name cam_name;
	int bus_id;
	int vcm_pwd;
	int vcm_enable;
};

struct msm_eeprom_info {
	struct i2c_board_info const *board_info;
	int bus_id;
	int eeprom_reg_addr;
	int eeprom_read_length;
	int eeprom_i2c_slave_addr;
};

struct msm_camera_sensor_info {
	const char *sensor_name;
	int sensor_reset_enable;
	int sensor_reset;
	int sensor_pwd;
	int vcm_pwd;
	int vcm_enable;
	int mclk;
	int flash_type;
	struct msm_camera_sensor_platform_info *sensor_platform_info;
	struct msm_camera_device_platform_data *pdata;
	struct resource *resource;
	uint8_t num_resources;
	struct msm_camera_sensor_flash_data *flash_data;
	int csi_if;
	struct msm_camera_sensor_strobe_flash_data *strobe_flash_data;
	char *eeprom_data;
	enum msm_camera_type camera_type;
	enum msm_sensor_type sensor_type;
	struct msm_actuator_info *actuator_info;
	int pmic_gpio_enable;
	struct msm_eeprom_info *eeprom_info;
};

struct msm_camera_board_info {
	struct i2c_board_info *board_info;
	uint8_t num_i2c_board_info;
};

int msm_get_cam_resources(struct msm_camera_sensor_info *);

struct clk_lookup;

struct snd_endpoint {
	int id;
	const char *name;
};

struct msm_snd_endpoints {
	struct snd_endpoint *endpoints;
	unsigned num;
};

struct cad_endpoint {
	int id;
	const char *name;
	uint32_t capability;
};

struct msm_cad_endpoints {
	struct cad_endpoint *endpoints;
	unsigned num;
};

#define MSM_MAX_DEC_CNT 14
enum msm_adspdec_concurrency {
	MSM_ADSP_CODEC_WAV = 0,
	MSM_ADSP_CODEC_ADPCM = 1,
	MSM_ADSP_CODEC_MP3 = 2,
	MSM_ADSP_CODEC_REALAUDIO = 3,
	MSM_ADSP_CODEC_WMA = 4,
	MSM_ADSP_CODEC_AAC = 5,
	MSM_ADSP_CODEC_RESERVED = 6,
	MSM_ADSP_CODEC_MIDI = 7,
	MSM_ADSP_CODEC_YADPCM = 8,
	MSM_ADSP_CODEC_QCELP = 9,
	MSM_ADSP_CODEC_AMRNB = 10,
	MSM_ADSP_CODEC_AMRWB = 11,
	MSM_ADSP_CODEC_EVRC = 12,
	MSM_ADSP_CODEC_WMAPRO = 13,
	MSM_ADSP_CODEC_AC3 = 23,
	MSM_ADSP_MODE_TUNNEL = 24,
	MSM_ADSP_MODE_NONTUNNEL = 25,
	MSM_ADSP_MODE_LP = 26,
	MSM_ADSP_OP_DMA = 28,
	MSM_ADSP_OP_DM = 29,
};

struct msm_adspdec_info {
	const char *module_name;
	unsigned module_queueid;
	int module_decid; 
	unsigned nr_codec_support;
};

struct dec_instance_table {
	uint8_t max_instances_same_dec;
	uint8_t max_instances_diff_dec;
};

struct msm_adspdec_database {
	unsigned num_dec;
	unsigned num_concurrency_support;
	unsigned int *dec_concurrency_table; 
	struct msm_adspdec_info  *dec_info_list;
	struct dec_instance_table *dec_instance_list;
};

enum msm_mdp_hw_revision {
	MDP_REV_20 = 1,
	MDP_REV_22,
	MDP_REV_30,
	MDP_REV_303,
	MDP_REV_31,
	MDP_REV_40,
	MDP_REV_41,
	MDP_REV_42,
	MDP_REV_43,
	MDP_REV_44,
};

struct msm_panel_common_pdata {
	uintptr_t hw_revision_addr;
	int gpio;
	bool bl_lock;
	spinlock_t bl_spinlock;
	int (*backlight_level)(int level, int max, int min);
	int (*pmic_backlight)(int level);
	int (*rotate_panel)(void);
	int (*backlight) (int level, int mode);
	int (*panel_num)(void);
	void (*panel_config_gpio)(int);
	int (*vga_switch)(int select_vga);
	int *gpio_num;
	u32 mdp_max_clk;
#ifdef CONFIG_MSM_BUS_SCALING
	struct msm_bus_scale_pdata *mdp_bus_scale_table;
#endif
	int mdp_rev;
	u32 ov0_wb_size;  
	u32 ov1_wb_size;  
	u32 mem_hid;
	char cont_splash_enabled;
	u32 splash_screen_addr;
	u32 splash_screen_size;
	char mdp_iommu_split_domain;
	u32 avtimer_phy;
};



struct lcdc_platform_data {
	int (*lcdc_gpio_config)(int on);
	int (*lcdc_power_save)(int);
	unsigned int (*lcdc_get_clk)(void);
#ifdef CONFIG_MSM_BUS_SCALING
	struct msm_bus_scale_pdata *bus_scale_table;
#endif
	int (*lvds_pixel_remap)(void);
};

struct tvenc_platform_data {
	int poll;
	int (*pm_vid_en)(int on);
#ifdef CONFIG_MSM_BUS_SCALING
	struct msm_bus_scale_pdata *bus_scale_table;
#endif
};

struct mddi_platform_data {
	int (*mddi_power_save)(int on);
	int (*mddi_sel_clk)(u32 *clk_rate);
	int (*mddi_client_power)(u32 client_id);
};

struct mipi_dsi_platform_data {
	int vsync_gpio;
	int (*dsi_power_save)(int on);
	int (*dsi_client_reset)(void);
	int (*get_lane_config)(void);
	char (*splash_is_enabled)(void);
	int target_type;
};

enum mipi_dsi_3d_ctrl {
	FPGA_EBI2_INTF,
	FPGA_SPI_INTF,
};

struct mipi_dsi_phy_ctrl {
	uint32_t regulator[5];
	uint32_t timing[12];
	uint32_t ctrl[4];
	uint32_t strength[4];
	uint32_t pll[21];
};

struct mipi_dsi_panel_platform_data {
	int fpga_ctrl_mode;
	int fpga_3d_config_addr;
	int *gpio;
	struct mipi_dsi_phy_ctrl *phy_ctrl_settings;
	char dlane_swap;
	void (*dsi_pwm_cfg)(void);
	char enable_wled_bl_ctrl;
	void (*gpio_set_backlight)(int bl_level);
};

struct lvds_panel_platform_data {
	int *gpio;
};

struct msm_wfd_platform_data {
	char (*wfd_check_mdp_iommu_split)(void);
};

#define PANEL_NAME_MAX_LEN 50
struct msm_fb_platform_data {
	int (*detect_client)(const char *name);
	int mddi_prescan;
	unsigned char ext_resolution;
	int (*allow_set_offset)(void);
	char prim_panel_name[PANEL_NAME_MAX_LEN];
	char ext_panel_name[PANEL_NAME_MAX_LEN];
};

struct msm_hdmi_platform_data {
	int irq;
	int (*cable_detect)(int insert);
	int (*comm_power)(int on, int show);
	int (*enable_5v)(int on);
	int (*core_power)(int on, int show);
	int (*cec_power)(int on);
	int (*panel_power)(int on);
	int (*gpio_config)(int on);
	int (*init_irq)(void);
	bool (*check_hdcp_hw_support)(void);
	bool is_mhl_enabled;
};

struct msm_mhl_platform_data {
	int irq;
	
	uint32_t gpio_mhl_int;
	
	uint32_t gpio_mhl_reset;
	
	uint32_t gpio_mhl_power;
	
	uint32_t gpio_hdmi_mhl_mux;
	bool mhl_enabled;
};

struct msm_i2c_platform_data {
	int clk_freq;
	bool clk_ctl_xfer;
	uint32_t rmutex;
	const char *rsl_id;
	uint32_t pm_lat;
	int pri_clk;
	int pri_dat;
	int aux_clk;
	int aux_dat;
	int src_clk_rate;
	int use_gsbi_shared_mode;
	int keep_ahb_clk_on;
	void (*msm_i2c_config_gpio)(int iface, int config_type);
	bool active_only;
	uint32_t master_id;
};

struct msm_i2c_ssbi_platform_data {
	const char *rsl_id;
	enum msm_ssbi_controller_type controller_type;
};

struct msm_vidc_platform_data {
	int memtype;
	u32 enable_ion;
	int disable_dmx;
	int disable_fullhd;
	u32 cp_enabled;
	u32 secure_wb_heap;
#ifdef CONFIG_MSM_BUS_SCALING
	struct msm_bus_scale_pdata *vidc_bus_client_pdata;
#endif
	int cont_mode_dpb_count;
	int disable_turbo;
	unsigned long fw_addr;
};

enum msm_vidc_v4l2_iommu_map {
	MSM_VIDC_V4L2_IOMMU_MAP_NS = 0,
	MSM_VIDC_V4L2_IOMMU_MAP_CP,
	MSM_VIDC_V4L2_IOMMU_MAP_MAX,
};

struct msm_vidc_v4l2_platform_data {
	int64_t **iommu_table;
	int num_iommu_table;

	int64_t **load_table;
	int num_load_table;

	uint32_t max_load;
};

struct vcap_platform_data {
	unsigned *gpios;
	int num_gpios;
	struct msm_bus_scale_pdata *bus_client_pdata;
};

#if defined(CONFIG_USB_PEHCI_HCD) || defined(CONFIG_USB_PEHCI_HCD_MODULE)
struct isp1763_platform_data {
	unsigned reset_gpio;
	int (*setup_gpio)(int enable);
};
#endif

#ifdef CONFIG_OF_DEVICE
void msm_8974_init(struct of_dev_auxdata **);
#endif
void msm_add_devices(void);
void msm_8974_add_devices(void);
void msm_8974_add_drivers(void);
void msm_map_common_io(void);
void msm_map_qsd8x50_io(void);
void msm_map_msm8x60_io(void);
void msm_map_msm8960_io(void);
void msm_map_msm8930_io(void);
void msm_map_apq8064_io(void);
void msm_map_msm7x30_io(void);
void msm_map_fsm9xxx_io(void);
void msm_map_fsm9900_io(void);
void fsm9900_init_gpiomux(void);
void msm_map_8974_io(void);
void msm_map_8084_io(void);
void msm_map_msmkrypton_io(void);
void msm_map_msmsamarium_io(void);
void msm_map_msm8625_io(void);
void msm_map_msm9625_io(void);
void msm_init_irq(void);
void msm_8974_init_irq(void);
void vic_handle_irq(struct pt_regs *regs);
void msm_8974_reserve(void);
void msm_8974_very_early(void);
void msm_8974_init_gpiomux(void);
void msm_htc_8974_init_gpiomux(void);
void apq8084_init_gpiomux(void);
void msm9625_init_gpiomux(void);
void msmkrypton_init_gpiomux(void);
void msmsamarium_init_gpiomux(void);
void msm_map_mpq8092_io(void);
void mpq8092_init_gpiomux(void);
void msm_map_msm8226_io(void);
void msm8226_init_irq(void);
void msm8226_init_gpiomux(void);
void msm8226_htc_init_gpiomux(void);
void msm8610_init_gpiomux(void);
void msm_map_msm8610_io(void);
void msm8610_init_irq(void);

#if defined(CONFIG_ARCH_MSM7X27)
void msm_clk_dump_debug_info(void);
#else
static inline void msm_clk_dump_debug_info(void) {}
#endif

struct mmc_platform_data;
int msm_add_sdcc(unsigned int controller,
		struct mmc_platform_data *plat);

void msm_pm_register_irqs(void);
struct msm_usb_host_platform_data;
int msm_add_host(unsigned int host,
		struct msm_usb_host_platform_data *plat);

void msm_hsusb_set_vbus_state(int online);
void msm_otg_set_vbus_state(int online);
void htc_dwc3_msm_otg_set_vbus_state(int online);
enum usb_connect_type {
	CONNECT_TYPE_NOTIFY = -3,
	CONNECT_TYPE_CLEAR = -2,
	CONNECT_TYPE_UNKNOWN = -1,
	CONNECT_TYPE_NONE = 0,
	CONNECT_TYPE_USB,
	CONNECT_TYPE_AC,
	CONNECT_TYPE_9V_AC,
	CONNECT_TYPE_WIRELESS,
	CONNECT_TYPE_INTERNAL,
	CONNECT_TYPE_UNSUPPORTED,
#ifdef CONFIG_MACH_VERDI_LTE
	
	CONNECT_TYPE_USB_9V_AC,
#endif
	CONNECT_TYPE_MHL_AC,
};
struct t_usb_status_notifier{
	struct list_head notifier_link;
	const char *name;
	void (*func)(int cable_type);
};
int htc_usb_register_notifier(struct t_usb_status_notifier *notifer);
int64_t htc_qpnp_adc_get_usbid_adc(void);
int usb_get_connect_type(void);
static LIST_HEAD(g_lh_usb_notifier_list);

struct t_cable_status_notifier{
	struct list_head cable_notifier_link;
	const char *name;
	void (*func)(int cable_type);
};
int cable_detect_register_notifier(struct t_cable_status_notifier *);
static LIST_HEAD(g_lh_calbe_detect_notifier_list);

#ifdef CONFIG_HTC_MHL_DETECTION
struct t_mhl_status_notifier{
        struct list_head mhl_notifier_link;
        const char *name;
        void (*func)(bool isMHL, int charging_type);
};
int mhl_detect_register_notifier(struct t_mhl_status_notifier *);
static LIST_HEAD(g_lh_mhl_detect_notifier_list);
#endif

struct t_usb_host_status_notifier{
	struct list_head usb_host_notifier_link;
	const char *name;
	void (*func)(bool cable_in);
};
int usb_host_detect_register_notifier(struct t_usb_host_status_notifier *);
static LIST_HEAD(g_lh_usb_host_detect_notifier_list);

void msm_snddev_init(void);
void msm_snddev_init_timpani(void);
void msm_snddev_poweramp_on(void);
void msm_snddev_poweramp_off(void);
void msm_snddev_hsed_voltage_on(void);
void msm_snddev_hsed_voltage_off(void);
void msm_snddev_tx_route_config(void);
void msm_snddev_tx_route_deconfig(void);

extern phys_addr_t msm_shared_ram_phys; 

extern int get_partition_num_by_name(char *name);

u32 wcnss_rf_read_reg(u32 rf_reg_addr);
#endif
