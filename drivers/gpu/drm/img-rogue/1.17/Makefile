img_basedir := $(srctree)/drivers/gpu/drm/img-rogue/1.17
include $(img_basedir)/config_kernel.mk

obj-$(CONFIG_DRM_POWERVR_ROGUE_1_17) += pvrsrvkm_1_17.o

ccflags-y += \
 -include config_kernel.h \
 -I$(img_basedir) \
 -I$(img_basedir)/km \
 -D__linux__

include $(img_basedir)/pvrsrvkm.mk
include $(img_basedir)/mt8173/Makefile
