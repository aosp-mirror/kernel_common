ccflags-y += -I$(srctree)/drivers/media/platform/camx/cam_utils
ccflags-y += -I$(srctree)/drivers/media/platform/camx/cam_req_mgr
ccflags-y += -I$(srctree)/drivers/media/platform/camx/cam_core
ccflags-y += -I$(srctree)/drivers/media/platform/camx/cam_cpas/include
ccflags-y += -I$(srctree)/drivers/media/platform/camx/cam_jpeg
ccflags-y += -I$(srctree)/drivers/media/platform/camx/cam_jpeg/jpeg_hw/include
ccflags-y += -I$(srctree)/drivers/media/platform/camx/cam_jpeg/jpeg_hw/
ccflags-y += -I$(srctree)/drivers/media/platform/camx/cam_jpeg/jpeg_hw/jpeg_enc_hw
ccflags-y += -I$(srctree)/drivers/media/platform/camx/cam_smmu

obj-$(CONFIG_SPECTRA_CAMERA) += jpeg_enc_dev.o jpeg_enc_core.o jpeg_enc_soc.o \
	cam_jenc_v4l2.o \
	cam_jenc_v4l2_common.o \
	cam_jenc_v4l2_hal.o
