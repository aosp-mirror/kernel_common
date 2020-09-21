# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for trusty components
#

obj-$(CONFIG_TRUSTY)		+= trusty-core.o
trusty-core-objs		+= trusty.o trusty-mem.o
trusty-core-$(CONFIG_ARM)	+= trusty-smc-arm.o
trusty-core-$(CONFIG_ARM64)	+= trusty-smc-arm64.o
obj-$(CONFIG_TRUSTY_IRQ)	+= trusty-irq.o
obj-$(CONFIG_TRUSTY_LOG)	+= trusty-log.o
obj-$(CONFIG_TRUSTY_TEST)	+= trusty-test.o
obj-$(CONFIG_TRUSTY_VIRTIO)	+= trusty-virtio.o
obj-$(CONFIG_TRUSTY_VIRTIO_IPC)	+= trusty-ipc.o
