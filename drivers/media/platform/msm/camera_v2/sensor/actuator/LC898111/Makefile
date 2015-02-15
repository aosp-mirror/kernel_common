GCC_VERSION      := $(shell $(CONFIG_SHELL) $(PWD)/scripts/gcc-version.sh $(CROSS_COMPILE)gcc)
EXTRA_CFLAGS += -Idrivers/media/platform/msm/camera_v2
EXTRA_CFLAGS += -Idrivers/media/platform/msm/camera_v2/sensor/io
EXTRA_CFLAGS += -Idrivers/media/platform/msm/camera_v2/sensor/cci
obj-$(CONFIG_TI201_ACT) += HtcActOisBinder.o OisIni.o OisCmd.o
