#ifndef __SHUB_CTRL_H__
#define __SHUB_CTRL_H__

#define SHUB_FIRMWARE_UPDATE_SUPPORT 1



#ifdef SHUB_FIRMWARE_UPDATE_SUPPORT
typedef struct {
    uint8_t arch;
    uint8_t sense;
    uint8_t cw_lib;
    uint8_t water;
    uint8_t active_engine;
    uint8_t project_mapping;
} mcu_fw_version_t;

#define SHUB_FIRMWARE_UPDATE_DEVICE_NAME        "shub_fw_fla"
#define SHUB_FW_IOCTL_CODE                      (0xCC)
#define SHUB_FW_IOCTL_PRE_FLASH                 _IO(SHUB_FW_IOCTL_CODE, 1)
#define SHUB_FW_IOCTL_POST_FLASH                _IO(SHUB_FW_IOCTL_CODE, 2)
#define SHUB_FW_IOCTL_GET_FW_VERSION            _IOR(SHUB_FW_IOCTL_CODE, 3, mcu_fw_version_t)
#endif



#endif 

