#Android makefile to build kernel as a part of Android Build
PERL		= perl

KERNEL_TARGET := $(strip $(INSTALLED_KERNEL_TARGET))
ifeq ($(KERNEL_TARGET),)
INSTALLED_KERNEL_TARGET := $(PRODUCT_OUT)/kernel
endif

ifeq ($(TARGET_PREBUILT_KERNEL),)

KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
KERNEL_CONFIG := $(KERNEL_OUT)/.config
ifeq ($(TARGET_KERNEL_APPEND_DTB), true)
TARGET_PREBUILT_INT_KERNEL := $(KERNEL_OUT)/arch/arm/boot/zImage-dtb
else
TARGET_PREBUILT_INT_KERNEL := $(KERNEL_OUT)/arch/arm/boot/zImage
endif
KERNEL_HEADERS_INSTALL := $(KERNEL_OUT)/usr
KERNEL_MODULES_INSTALL := system
KERNEL_MODULES_OUT := $(TARGET_OUT)/lib/modules
KERNEL_IMG=$(KERNEL_OUT)/arch/arm/boot/Image

DTS_NAMES ?= $(shell $(PERL) -e 'while (<>) {$$a = $$1 if /CONFIG_ARCH_((?:MSM|QSD|MPQ)[a-zA-Z0-9]+)=y/; $$r = $$1 if /CONFIG_MSM_SOC_REV_(?!NONE)(\w+)=y/; $$arch = $$arch.lc("$$a$$r ") if /CONFIG_ARCH_((?:MSM|QSD|MPQ)[a-zA-Z0-9]+)=y/} print $$arch;' $(KERNEL_CONFIG))
KERNEL_USE_OF ?= $(shell $(PERL) -e '$$of = "n"; while (<>) { if (/CONFIG_USE_OF=y/) { $$of = "y"; break; } } print $$of;' kernel/arch/arm/configs/$(KERNEL_DEFCONFIG))
KERNEL_ENABLE_EXFAT ?= $(shell cat kernel/arch/arm/configs/$(KERNEL_DEFCONFIG) | egrep -v "^\s*\#" | egrep "CONFIG_EXFAT_FS" | sed 's/^\s*CONFIG_EXFAT_FS\s*=\s*//' )
KERNEL_EXFAT_PATH ?= $(shell cat kernel/arch/arm/configs/$(KERNEL_DEFCONFIG) | egrep -v "^\s*\#" | egrep "CONFIG_EXFAT_PATH" | sed 's/^\s*CONFIG_EXFAT_PATH\s*=\s*\"//' | sed 's/\".*//' )
KERNEL_EXFAT_VERSION ?= $(shell cat kernel/arch/arm/configs/$(KERNEL_DEFCONFIG) | egrep -v "^\s*\#" | egrep "CONFIG_EXFAT_VERSION" | sed 's/^\s*CONFIG_EXFAT_VERSION\s*=\s*\"//' | sed 's/\".*//' )
BUILD_PATH ?= $(shell pwd)

ifeq "$(KERNEL_USE_OF)" "y"
DTS_FILES = $(wildcard $(TOP)/kernel/arch/arm/boot/dts/$(DTS_NAME)*.dts)
DTS_FILE = $(lastword $(subst /, ,$(1)))
DTB_FILE = $(addprefix $(KERNEL_OUT)/arch/arm/boot/,$(patsubst %.dts,%.dtb,$(call DTS_FILE,$(1))))
ZIMG_FILE = $(addprefix $(KERNEL_OUT)/arch/arm/boot/,$(patsubst %.dts,%-zImage,$(call DTS_FILE,$(1))))
KERNEL_ZIMG = $(KERNEL_OUT)/arch/arm/boot/zImage
DTC = $(KERNEL_OUT)/scripts/dtc/dtc

define append-dtb
#mkdir -p $(KERNEL_OUT)/arch/arm/boot;\
$(foreach DTS_NAME, $(DTS_NAMES), \
   $(foreach d, $(DTS_FILES), \
      $(DTC) -p 1024 -O dtb -o $(call DTB_FILE,$(d)) $(d); \
      cat $(KERNEL_ZIMG) $(call DTB_FILE,$(d)) > $(call ZIMG_FILE,$(d));))
endef
else

define append-dtb
endef
endif

ifeq ($(TARGET_USES_UNCOMPRESSED_KERNEL),true)
$(info Using uncompressed kernel)
TARGET_PREBUILT_KERNEL := $(KERNEL_OUT)/piggy
else
TARGET_PREBUILT_KERNEL := $(TARGET_PREBUILT_INT_KERNEL)
endif

define mv-modules
mdpath=`find $(KERNEL_MODULES_OUT) -type f -name modules.dep`;\
if [ "$$mdpath" != "" ];then\
mpath=`dirname $$mdpath`;\
ko=`find $$mpath/kernel -type f -name *.ko`;\
for i in $$ko; do mv $$i $(KERNEL_MODULES_OUT)/; done;\
fi
endef

define clean-module-folder
mdpath=`find $(KERNEL_MODULES_OUT) -type f -name modules.dep`;\
if [ "$$mdpath" != "" ];then\
mpath=`dirname $$mdpath`; rm -rf $$mpath;\
fi
endef

$(KERNEL_OUT):
	mkdir -p $(KERNEL_OUT)

$(KERNEL_CONFIG): $(KERNEL_OUT)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) $(KERNEL_DEFCONFIG)

$(KERNEL_OUT)/piggy : $(TARGET_PREBUILT_INT_KERNEL)
	$(hide) gunzip -c $(KERNEL_OUT)/arch/arm/boot/compressed/piggy.gzip > $(KERNEL_OUT)/piggy

$(TARGET_PREBUILT_INT_KERNEL): $(KERNEL_OUT) $(KERNEL_CONFIG) $(KERNEL_HEADERS_INSTALL)
ifeq ($(KERNEL_ENABLE_EXFAT), m)
	cp vendor/tuxera/exfat/tuxera_update_htc.sh kernel/
	cp vendor/tuxera/exfat/update_tuxera.sh kernel/
	cp vendor/tuxera/exfat/build_exfat.sh kernel/
	cp -rf vendor/tuxera/exfat/texfat kernel/fs/
	cp -rf vendor/tuxera/exfat/$(KERNEL_EXFAT_PATH) kernel/fs/
	mkdir -p $(KERNEL_OUT)/fs/$(KERNEL_EXFAT_PATH)
	# Update exFAT module after vmlinux but before modules
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) vmlinux
ifeq ($(HTC_DEBUG_FLAG), DEBUG)
ifeq ($(strip $(KERNEL_EXFAT_VERSION)),)
	./kernel/update_tuxera.sh -p $(KERNEL_EXFAT_PATH) -t target/htc.d/htc -o $(KERNEL_OUT)
else
	./kernel/update_tuxera.sh -p $(KERNEL_EXFAT_PATH) -t $(KERNEL_EXFAT_VERSION) -o $(KERNEL_OUT)
endif
else
ifeq ($(TARGET_BUILD_VARIANT), user)
	$(warning "User-Release")
ifeq ($(strip $(KERNEL_EXFAT_VERSION)),)
	./kernel/update_tuxera.sh -p $(KERNEL_EXFAT_PATH) -t target/htc.d/htc -o $(KERNEL_OUT) -r
else
	./kernel/update_tuxera.sh -p $(KERNEL_EXFAT_PATH) -t $(KERNEL_EXFAT_VERSION) -o $(KERNEL_OUT) -r
endif
else
	$(warning "NonUser-Release")
ifeq ($(strip $(KERNEL_EXFAT_VERSION)),)
	./kernel/update_tuxera.sh -p $(KERNEL_EXFAT_PATH) -t target/htc.d/htc -o $(KERNEL_OUT) -u
else
	./kernel/update_tuxera.sh -p $(KERNEL_EXFAT_PATH) -t $(KERNEL_EXFAT_VERSION) -o $(KERNEL_OUT) -u
endif
endif

endif
endif
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) modules
	$(MAKE) -C kernel O=../$(KERNEL_OUT) INSTALL_MOD_PATH=../../$(KERNEL_MODULES_INSTALL) INSTALL_MOD_STRIP=1 ARCH=arm CROSS_COMPILE=arm-eabi- PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) modules_install
ifeq ($(KERNEL_ENABLE_EXFAT), m)
ifeq ($(HTC_DEBUG_FLAG), DEBUG)
	# Build exfat modules for DEBUG
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) SUBDIRS=$(BUILD_PATH)/kernel/fs/$(KERNEL_EXFAT_PATH)/objects modules
	$(MAKE) -C kernel O=../$(KERNEL_OUT) SUBDIRS=fs/$(KERNEL_EXFAT_PATH)/objects INSTALL_MOD_PATH=../../$(KERNEL_MODULES_INSTALL) ARCH=arm CROSS_COMPILE=arm-eabi- PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) modules_install
	cp kernel/fs/$(KERNEL_EXFAT_PATH)/objects/texfat.ko $(KERNEL_MODULES_OUT)/
	mkdir -p $(TARGET_OUT)/bin/
	cp -rf kernel/fs/$(KERNEL_EXFAT_PATH)/bin/* $(TARGET_OUT)/bin/
else
ifeq ($(TARGET_BUILD_VARIANT), user)
	# Build exfat modules for NonDebug-USER
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) SUBDIRS=$(BUILD_PATH)/kernel/fs/$(KERNEL_EXFAT_PATH)/objects-user modules
	$(MAKE) -C kernel O=../$(KERNEL_OUT) SUBDIRS=fs/$(KERNEL_EXFAT_PATH)/objects-user INSTALL_MOD_PATH=../../$(KERNEL_MODULES_INSTALL) ARCH=arm CROSS_COMPILE=arm-eabi- PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) modules_install
	cp kernel/fs/$(KERNEL_EXFAT_PATH)/objects-user/texfat.ko $(KERNEL_MODULES_OUT)/
	mkdir -p $(TARGET_OUT)/bin/
	cp -rf kernel/fs/$(KERNEL_EXFAT_PATH)/bin/* $(TARGET_OUT)/bin/
else
	# Build exfat modules for NonDebug-USERDEBUG
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) SUBDIRS=$(BUILD_PATH)/kernel/fs/$(KERNEL_EXFAT_PATH)/objects-userdebug modules
	$(MAKE) -C kernel O=../$(KERNEL_OUT) SUBDIRS=fs/$(KERNEL_EXFAT_PATH)/objects-userdebug INSTALL_MOD_PATH=../../$(KERNEL_MODULES_INSTALL) ARCH=arm CROSS_COMPILE=arm-eabi- PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) modules_install
	cp kernel/fs/$(KERNEL_EXFAT_PATH)/objects-userdebug/texfat.ko $(KERNEL_MODULES_OUT)/
	mkdir -p $(TARGET_OUT)/bin/
	cp -rf kernel/fs/$(KERNEL_EXFAT_PATH)/bin/* $(TARGET_OUT)/bin/
endif
endif
endif
	$(mv-modules)
	$(clean-module-folder)
	$(append-dtb)
ifeq ($(MOCANA_FIPS_MODULE), true)
	vendor/mocana/scripts/build_dar.sh -s `pwd`/vendor/mocana -k 3.4.10 -K `pwd`/kernel -t `pwd`/$(KERNEL_MODULES_OUT) -z `pwd`/$(KERNEL_OUT)
	$(MAKE) -C kernel ARCH=arm CROSS_COMPILE=arm-eabi- PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) mrproper
endif
ifeq ($(KERNEL_ENABLE_EXFAT), m)
	rm kernel/tuxera_update_htc.sh
	rm kernel/update_tuxera.sh
	rm kernel/build_exfat.sh
	rm -rf kernel/fs/texfat*
endif
$(KERNEL_HEADERS_INSTALL): $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) headers_install

kerneltags: $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) tags

kernelconfig: $(KERNEL_OUT) $(KERNEL_CONFIG)
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) menuconfig
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- PRIVATE_RCMS_NAME=$(PRIVATE_RCMS_NAME) savedefconfig
	cp $(KERNEL_OUT)/defconfig kernel/arch/arm/configs/$(KERNEL_DEFCONFIG)

endif
