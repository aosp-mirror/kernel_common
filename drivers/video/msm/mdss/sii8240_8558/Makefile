#
# Makefile for the Silicon Image 8240/8558 MHL TX device driver
#

ccflags-y += -DBUILD_NUM_STRING=\"$(BUILD_NUM)\"

ccflags-y += -DMHL_PRODUCT_NUM=8240_8558
ccflags-y += -DMHL_DRIVER_NAME=\"sii8240_sii8558drv\"
ccflags-y += -DMHL_DEVICE_NAME=\"sii-8240-8558\"

# RCP_INPUTDEV_SUPPORT supports android key mapping for incoming RCP keys.
# Disable this support if key mapping is not required instead rcp related sysfs is exposed.
# Enable this support if key mapping is required and turn off rcp related sysfs.
ccflags-y += -DRCP_INPUTDEV_SUPPORT
#
# ENABLE_GEN2 enables hardware driven WRITE_BURST handling.
ccflags-y += -DENABLE_GEN2

# DDC_BYPASS_API
#ccflags-y += -DDDC_BYPASS_API

# PUSH_PULL_8240 enables PUSH/PULL mode for 8240 starter kit with NPN/PNP level shifter re-work.
ccflags-y += -DPUSH_PULL_8240

# DRIVER_SPEC_SYSFS enables support for the new sysfs interfaces
ccflags-y += -DDRIVER_SPEC_SYSFS

# MEDIA_DATA_TUNNEL_SUPPORT is for MDT support in chip independent code
ccflags-y += -DMEDIA_DATA_TUNNEL_SUPPORT
# Print propter log by attribute file to change level
ccflags-y += -DDEBUG
#
# the next three lines are optional - they enable greater verbosity in debug output
ccflags-y  += -DENABLE_EDID_INFO_PRINT
#ccflags-$(CONFIG_DEBUG_DRIVER) += -DENABLE_EDID_DEBUG_PRINT
#ccflags-$(CONFIG_DEBUG_DRIVER) += -DENABLE_DUMP_INFOFRAME
#
# uncomment next line to prevent EDID parsing. Useful for debugging.
#ccflags-$(CONFIG_DEBUG_DRIVER) += -DEDID_PASSTHROUGH

obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8240_SII8558) += platform.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8240_SII8558) += mhl_linux_tx.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8240_SII8558) += mhl_rcp_inputdev.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8240_SII8558) += mhl_supp.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8240_SII8558) += si_8240_8558_drv.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8240_SII8558) += si_mhl2_edid_3d.o
obj-$(CONFIG_FB_MSM_MDSS_HDMI_MHL_SII8240_SII8558) += si_mdt_inputdev.o

