#
# Makefile for miscellaneous I2C chip drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#
#proximity-objs := cm3629.o
obj-$(CONFIG_SENSORS_NFC_PN544) += pn544.o
obj-$(CONFIG_AMP_TFA9887)       += tfa9887.o
obj-$(CONFIG_AMP_TFA9887L)      += tfa9887l.o
obj-$(CONFIG_AMP_RT5501)	+= rt5501.o
obj-$(CONFIG_AMP_RT5501_EVM)	+= rt5501_evm.o
obj-$(CONFIG_AMP_RT5506)	+= rt5506.o

ifeq ($(CONFIG_INPUT_CAPELLA_CM3629),y)
proximity-objs := cm3629.o
obj-$(CONFIG_INPUT_CAPELLA_CM3629)      += proximity.o
endif

ifeq ($(CONFIG_INPUT_CAPELLA_CM36686),y)
proximity-objs := cm36686.o
obj-$(CONFIG_INPUT_CAPELLA_CM36686)      += proximity.o
endif

obj-$(CONFIG_INPUT_CAPELLA_CM32181)     += cm32181.o
obj-$(CONFIG_SENSORS_AKM8975)   += akm8975.o
obj-$(CONFIG_SENSORS_AKM8975_PANA_GYRO)   += akm8975_pana_gyro.o
obj-$(CONFIG_SENSORS_PANASONIC_GYRO)	+= ewtzmu2.o
obj-$(CONFIG_SENSORS_AK8963)    += akm8963.o
obj-$(CONFIG_SENSORS_AK8963_DOE_PLUS)    += akm8963_doe_plus.o
obj-$(CONFIG_SENSORS_R3GD20)    += r3gd20.o
obj-$(CONFIG_SENSORS_LSM330_ACC)	+= lsm330_acc.o
obj-$(CONFIG_SENSORS_LSM330_GYRO)	+= lsm330_gyr.o
obj-$(CONFIG_SENSORS_BMA250)	+= bma250.o
obj-$(CONFIG_BOSCH_BMA250)      += bma250_bosch.o
obj-$(CONFIG_INPUT_YAS_MAGNETOMETER) += yas53x_kernel.o
obj-$(CONFIG_VP_A1028)          += a1028.o
obj-$(CONFIG_INPUT_CWSTM32)    += CwMcuSensor.o
obj-$(CONFIG_LIGHTSENSOR_EPL88051) += epl88051.o
ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif

