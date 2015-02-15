ifeq ("$(origin PRIVATE_RCMS_NAME)", "command line")
KBUILD_RCMS_NAME := $(PRIVATE_RCMS_NAME)
endif

ccflags-$(CONFIG_HTC_RADIO_SMEM) += -DRCMS_NAME="\"$(KBUILD_RCMS_NAME)\""
obj-$(CONFIG_HTC_RADIO_SMEM)    += htc_radio_smem.o
