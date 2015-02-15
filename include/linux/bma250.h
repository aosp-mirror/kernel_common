#ifndef BMA250_H
#define BMA250_H

#include <linux/ioctl.h>

#define SENSOR_NAME 			"bma250"
#define ABSMIN				-512
#define ABSMAX				512
#define SLOPE_THRESHOLD_VALUE 		32
#define SLOPE_DURATION_VALUE 		1
#define INTERRUPT_LATCH_MODE 		13
#define INTERRUPT_ENABLE 		1
#define INTERRUPT_DISABLE 		0
#define MAP_SLOPE_INTERRUPT 		2
#define SLOPE_X_INDEX 			5
#define SLOPE_Y_INDEX 			6
#define SLOPE_Z_INDEX 			7
#define BMA250_MAX_DELAY		200
#define BMA250_CHIP_ID			3
#define BMA250E_CHIP_ID			0xF9
#define BMA250_RANGE_SET		0
#define BMA250_BW_SET			2

#define LOW_G_INTERRUPT				REL_Z
#define HIGH_G_INTERRUPT 			REL_HWHEEL
#define SLOP_INTERRUPT 				REL_DIAL
#define DOUBLE_TAP_INTERRUPT 			REL_WHEEL
#define SINGLE_TAP_INTERRUPT 			REL_MISC
#define ORIENT_INTERRUPT 			ABS_PRESSURE
#define FLAT_INTERRUPT 				ABS_DISTANCE


#define HIGH_G_INTERRUPT_X_HAPPENED			1
#define HIGH_G_INTERRUPT_Y_HAPPENED 			2
#define HIGH_G_INTERRUPT_Z_HAPPENED 			3
#define HIGH_G_INTERRUPT_X_NEGATIVE_HAPPENED 		4
#define HIGH_G_INTERRUPT_Y_NEGATIVE_HAPPENED		5
#define HIGH_G_INTERRUPT_Z_NEGATIVE_HAPPENED 		6
#define SLOPE_INTERRUPT_X_HAPPENED 			7
#define SLOPE_INTERRUPT_Y_HAPPENED 			8
#define SLOPE_INTERRUPT_Z_HAPPENED 			9
#define SLOPE_INTERRUPT_X_NEGATIVE_HAPPENED 		10
#define SLOPE_INTERRUPT_Y_NEGATIVE_HAPPENED 		11
#define SLOPE_INTERRUPT_Z_NEGATIVE_HAPPENED 		12
#define DOUBLE_TAP_INTERRUPT_HAPPENED 			13
#define SINGLE_TAP_INTERRUPT_HAPPENED 			14
#define UPWARD_PORTRAIT_UP_INTERRUPT_HAPPENED 		15
#define UPWARD_PORTRAIT_DOWN_INTERRUPT_HAPPENED	 	16
#define UPWARD_LANDSCAPE_LEFT_INTERRUPT_HAPPENED 	17
#define UPWARD_LANDSCAPE_RIGHT_INTERRUPT_HAPPENED	18
#define DOWNWARD_PORTRAIT_UP_INTERRUPT_HAPPENED 	19
#define DOWNWARD_PORTRAIT_DOWN_INTERRUPT_HAPPENED 	20
#define DOWNWARD_LANDSCAPE_LEFT_INTERRUPT_HAPPENED 	21
#define DOWNWARD_LANDSCAPE_RIGHT_INTERRUPT_HAPPENED 	22
#define FLAT_INTERRUPT_TURE_HAPPENED			23
#define FLAT_INTERRUPT_FALSE_HAPPENED			24
#define LOW_G_INTERRUPT_HAPPENED			25

#define PAD_LOWG					0
#define PAD_HIGHG					1
#define PAD_SLOP					2
#define PAD_DOUBLE_TAP					3
#define PAD_SINGLE_TAP					4
#define PAD_ORIENT					5
#define PAD_FLAT					6


#define BMA250_CHIP_ID_REG                      0x00
#define BMA250_VERSION_REG                      0x01
#define BMA250_X_AXIS_LSB_REG                   0x02
#define BMA250_X_AXIS_MSB_REG                   0x03
#define BMA250_Y_AXIS_LSB_REG                   0x04
#define BMA250_Y_AXIS_MSB_REG                   0x05
#define BMA250_Z_AXIS_LSB_REG                   0x06
#define BMA250_Z_AXIS_MSB_REG                   0x07
#define BMA250_TEMP_RD_REG                      0x08
#define BMA250_STATUS1_REG                      0x09
#define BMA250_STATUS2_REG                      0x0A
#define BMA250_STATUS_TAP_SLOPE_REG             0x0B
#define BMA250_STATUS_ORIENT_HIGH_REG           0x0C
#define BMA250_RANGE_SEL_REG                    0x0F
#define BMA250_BW_SEL_REG                       0x10
#define BMA250_MODE_CTRL_REG                    0x11
#define BMA250_LOW_NOISE_CTRL_REG               0x12
#define BMA250_DATA_CTRL_REG                    0x13
#define BMA250_RESET_REG                        0x14
#define BMA250_INT_ENABLE1_REG                  0x16
#define BMA250_INT_ENABLE2_REG                  0x17
#define BMA250_INT1_PAD_SEL_REG                 0x19
#define BMA250_INT_DATA_SEL_REG                 0x1A
#define BMA250_INT2_PAD_SEL_REG                 0x1B
#define BMA250_INT_SRC_REG                      0x1E
#define BMA250_INT_SET_REG                      0x20
#define BMA250_INT_CTRL_REG                     0x21
#define BMA250_LOW_DURN_REG                     0x22
#define BMA250_LOW_THRES_REG                    0x23
#define BMA250_LOW_HIGH_HYST_REG                0x24
#define BMA250_HIGH_DURN_REG                    0x25
#define BMA250_HIGH_THRES_REG                   0x26
#define BMA250_SLOPE_DURN_REG                   0x27
#define BMA250_SLOPE_THRES_REG                  0x28
#define BMA250_TAP_PARAM_REG                    0x2A
#define BMA250_TAP_THRES_REG                    0x2B
#define BMA250_ORIENT_PARAM_REG                 0x2C
#define BMA250_THETA_BLOCK_REG                  0x2D
#define BMA250_THETA_FLAT_REG                   0x2E
#define BMA250_FLAT_HOLD_TIME_REG               0x2F
#define BMA250_STATUS_LOW_POWER_REG             0x31
#define BMA250_SELF_TEST_REG                    0x32
#define BMA250_EEPROM_CTRL_REG                  0x33
#define BMA250_SERIAL_CTRL_REG                  0x34
#define BMA250_CTRL_UNLOCK_REG                  0x35
#define BMA250_OFFSET_CTRL_REG                  0x36
#define BMA250_OFFSET_PARAMS_REG                0x37
#define BMA250_OFFSET_FILT_X_REG                0x38
#define BMA250_OFFSET_FILT_Y_REG                0x39
#define BMA250_OFFSET_FILT_Z_REG                0x3A
#define BMA250_OFFSET_UNFILT_X_REG              0x3B
#define BMA250_OFFSET_UNFILT_Y_REG              0x3C
#define BMA250_OFFSET_UNFILT_Z_REG              0x3D
#define BMA250_SPARE_0_REG                      0x3E
#define BMA250_SPARE_1_REG                      0x3F

#define BMA250_ACC_X_LSB__POS           6
#define BMA250_ACC_X_LSB__LEN           2
#define BMA250_ACC_X_LSB__MSK           0xC0
#define BMA250_ACC_X_LSB__REG           BMA250_X_AXIS_LSB_REG

#define BMA250_ACC_X_MSB__POS           0
#define BMA250_ACC_X_MSB__LEN           8
#define BMA250_ACC_X_MSB__MSK           0xFF
#define BMA250_ACC_X_MSB__REG           BMA250_X_AXIS_MSB_REG

#define BMA250_ACC_Y_LSB__POS           6
#define BMA250_ACC_Y_LSB__LEN           2
#define BMA250_ACC_Y_LSB__MSK           0xC0
#define BMA250_ACC_Y_LSB__REG           BMA250_Y_AXIS_LSB_REG

#define BMA250_ACC_Y_MSB__POS           0
#define BMA250_ACC_Y_MSB__LEN           8
#define BMA250_ACC_Y_MSB__MSK           0xFF
#define BMA250_ACC_Y_MSB__REG           BMA250_Y_AXIS_MSB_REG

#define BMA250_ACC_Z_LSB__POS           6
#define BMA250_ACC_Z_LSB__LEN           2
#define BMA250_ACC_Z_LSB__MSK           0xC0
#define BMA250_ACC_Z_LSB__REG           BMA250_Z_AXIS_LSB_REG

#define BMA250_ACC_Z_MSB__POS           0
#define BMA250_ACC_Z_MSB__LEN           8
#define BMA250_ACC_Z_MSB__MSK           0xFF
#define BMA250_ACC_Z_MSB__REG           BMA250_Z_AXIS_MSB_REG

#define BMA250_RANGE_SEL__POS             0
#define BMA250_RANGE_SEL__LEN             4
#define BMA250_RANGE_SEL__MSK             0x0F
#define BMA250_RANGE_SEL__REG             BMA250_RANGE_SEL_REG

#define BMA250_BANDWIDTH__POS             0
#define BMA250_BANDWIDTH__LEN             5
#define BMA250_BANDWIDTH__MSK             0x1F
#define BMA250_BANDWIDTH__REG             BMA250_BW_SEL_REG

#define BMA250_EN_LOW_POWER__POS          6
#define BMA250_EN_LOW_POWER__LEN          1
#define BMA250_EN_LOW_POWER__MSK          0x40
#define BMA250_EN_LOW_POWER__REG          BMA250_MODE_CTRL_REG

#define BMA250_EN_SUSPEND__POS            7
#define BMA250_EN_SUSPEND__LEN            1
#define BMA250_EN_SUSPEND__MSK            0x80
#define BMA250_EN_SUSPEND__REG            BMA250_MODE_CTRL_REG

#define BMA250_INT_MODE_SEL__POS                0
#define BMA250_INT_MODE_SEL__LEN                4
#define BMA250_INT_MODE_SEL__MSK                0x0F
#define BMA250_INT_MODE_SEL__REG                BMA250_INT_CTRL_REG

#define BMA250_LOWG_INT_S__POS          0
#define BMA250_LOWG_INT_S__LEN          1
#define BMA250_LOWG_INT_S__MSK          0x01
#define BMA250_LOWG_INT_S__REG          BMA250_STATUS1_REG

#define BMA250_HIGHG_INT_S__POS          1
#define BMA250_HIGHG_INT_S__LEN          1
#define BMA250_HIGHG_INT_S__MSK          0x02
#define BMA250_HIGHG_INT_S__REG          BMA250_STATUS1_REG

#define BMA250_SLOPE_INT_S__POS          2
#define BMA250_SLOPE_INT_S__LEN          1
#define BMA250_SLOPE_INT_S__MSK          0x04
#define BMA250_SLOPE_INT_S__REG          BMA250_STATUS1_REG

#define BMA250_DOUBLE_TAP_INT_S__POS     4
#define BMA250_DOUBLE_TAP_INT_S__LEN     1
#define BMA250_DOUBLE_TAP_INT_S__MSK     0x10
#define BMA250_DOUBLE_TAP_INT_S__REG     BMA250_STATUS1_REG

#define BMA250_SINGLE_TAP_INT_S__POS     5
#define BMA250_SINGLE_TAP_INT_S__LEN     1
#define BMA250_SINGLE_TAP_INT_S__MSK     0x20
#define BMA250_SINGLE_TAP_INT_S__REG     BMA250_STATUS1_REG

#define BMA250_ORIENT_INT_S__POS         6
#define BMA250_ORIENT_INT_S__LEN         1
#define BMA250_ORIENT_INT_S__MSK         0x40
#define BMA250_ORIENT_INT_S__REG         BMA250_STATUS1_REG

#define BMA250_FLAT_INT_S__POS           7
#define BMA250_FLAT_INT_S__LEN           1
#define BMA250_FLAT_INT_S__MSK           0x80
#define BMA250_FLAT_INT_S__REG           BMA250_STATUS1_REG

#define BMA250_DATA_INT_S__POS           7
#define BMA250_DATA_INT_S__LEN           1
#define BMA250_DATA_INT_S__MSK           0x80
#define BMA250_DATA_INT_S__REG           BMA250_STATUS2_REG

#define BMA250_SLOPE_FIRST_X__POS        0
#define BMA250_SLOPE_FIRST_X__LEN        1
#define BMA250_SLOPE_FIRST_X__MSK        0x01
#define BMA250_SLOPE_FIRST_X__REG        BMA250_STATUS_TAP_SLOPE_REG

#define BMA250_SLOPE_FIRST_Y__POS        1
#define BMA250_SLOPE_FIRST_Y__LEN        1
#define BMA250_SLOPE_FIRST_Y__MSK        0x02
#define BMA250_SLOPE_FIRST_Y__REG        BMA250_STATUS_TAP_SLOPE_REG

#define BMA250_SLOPE_FIRST_Z__POS        2
#define BMA250_SLOPE_FIRST_Z__LEN        1
#define BMA250_SLOPE_FIRST_Z__MSK        0x04
#define BMA250_SLOPE_FIRST_Z__REG        BMA250_STATUS_TAP_SLOPE_REG

#define BMA250_SLOPE_SIGN_S__POS         3
#define BMA250_SLOPE_SIGN_S__LEN         1
#define BMA250_SLOPE_SIGN_S__MSK         0x08
#define BMA250_SLOPE_SIGN_S__REG         BMA250_STATUS_TAP_SLOPE_REG

#define BMA250_TAP_FIRST_X__POS        4
#define BMA250_TAP_FIRST_X__LEN        1
#define BMA250_TAP_FIRST_X__MSK        0x10
#define BMA250_TAP_FIRST_X__REG        BMA250_STATUS_TAP_SLOPE_REG

#define BMA250_TAP_FIRST_Y__POS        5
#define BMA250_TAP_FIRST_Y__LEN        1
#define BMA250_TAP_FIRST_Y__MSK        0x20
#define BMA250_TAP_FIRST_Y__REG        BMA250_STATUS_TAP_SLOPE_REG

#define BMA250_TAP_FIRST_Z__POS        6
#define BMA250_TAP_FIRST_Z__LEN        1
#define BMA250_TAP_FIRST_Z__MSK        0x40
#define BMA250_TAP_FIRST_Z__REG        BMA250_STATUS_TAP_SLOPE_REG

#define BMA250_TAP_FIRST_XYZ__POS        4
#define BMA250_TAP_FIRST_XYZ__LEN        3
#define BMA250_TAP_FIRST_XYZ__MSK        0x70
#define BMA250_TAP_FIRST_XYZ__REG        BMA250_STATUS_TAP_SLOPE_REG

#define BMA250_TAP_SIGN_S__POS         7
#define BMA250_TAP_SIGN_S__LEN         1
#define BMA250_TAP_SIGN_S__MSK         0x80
#define BMA250_TAP_SIGN_S__REG         BMA250_STATUS_TAP_SLOPE_REG

#define BMA250_HIGHG_FIRST_X__POS        0
#define BMA250_HIGHG_FIRST_X__LEN        1
#define BMA250_HIGHG_FIRST_X__MSK        0x01
#define BMA250_HIGHG_FIRST_X__REG        BMA250_STATUS_ORIENT_HIGH_REG

#define BMA250_HIGHG_FIRST_Y__POS        1
#define BMA250_HIGHG_FIRST_Y__LEN        1
#define BMA250_HIGHG_FIRST_Y__MSK        0x02
#define BMA250_HIGHG_FIRST_Y__REG        BMA250_STATUS_ORIENT_HIGH_REG

#define BMA250_HIGHG_FIRST_Z__POS        2
#define BMA250_HIGHG_FIRST_Z__LEN        1
#define BMA250_HIGHG_FIRST_Z__MSK        0x04
#define BMA250_HIGHG_FIRST_Z__REG        BMA250_STATUS_ORIENT_HIGH_REG

#define BMA250_HIGHG_SIGN_S__POS         3
#define BMA250_HIGHG_SIGN_S__LEN         1
#define BMA250_HIGHG_SIGN_S__MSK         0x08
#define BMA250_HIGHG_SIGN_S__REG         BMA250_STATUS_ORIENT_HIGH_REG

#define BMA250_ORIENT_S__POS             4
#define BMA250_ORIENT_S__LEN             3
#define BMA250_ORIENT_S__MSK             0x70
#define BMA250_ORIENT_S__REG             BMA250_STATUS_ORIENT_HIGH_REG

#define BMA250_FLAT_S__POS               7
#define BMA250_FLAT_S__LEN               1
#define BMA250_FLAT_S__MSK               0x80
#define BMA250_FLAT_S__REG               BMA250_STATUS_ORIENT_HIGH_REG

#define BMA250_EN_SLOPE_X_INT__POS         0
#define BMA250_EN_SLOPE_X_INT__LEN         1
#define BMA250_EN_SLOPE_X_INT__MSK         0x01
#define BMA250_EN_SLOPE_X_INT__REG         BMA250_INT_ENABLE1_REG

#define BMA250_EN_SLOPE_Y_INT__POS         1
#define BMA250_EN_SLOPE_Y_INT__LEN         1
#define BMA250_EN_SLOPE_Y_INT__MSK         0x02
#define BMA250_EN_SLOPE_Y_INT__REG         BMA250_INT_ENABLE1_REG

#define BMA250_EN_SLOPE_Z_INT__POS         2
#define BMA250_EN_SLOPE_Z_INT__LEN         1
#define BMA250_EN_SLOPE_Z_INT__MSK         0x04
#define BMA250_EN_SLOPE_Z_INT__REG         BMA250_INT_ENABLE1_REG

#define BMA250_EN_SLOPE_XYZ_INT__POS         0
#define BMA250_EN_SLOPE_XYZ_INT__LEN         3
#define BMA250_EN_SLOPE_XYZ_INT__MSK         0x07
#define BMA250_EN_SLOPE_XYZ_INT__REG         BMA250_INT_ENABLE1_REG

#define BMA250_EN_DOUBLE_TAP_INT__POS      4
#define BMA250_EN_DOUBLE_TAP_INT__LEN      1
#define BMA250_EN_DOUBLE_TAP_INT__MSK      0x10
#define BMA250_EN_DOUBLE_TAP_INT__REG      BMA250_INT_ENABLE1_REG

#define BMA250_EN_SINGLE_TAP_INT__POS      5
#define BMA250_EN_SINGLE_TAP_INT__LEN      1
#define BMA250_EN_SINGLE_TAP_INT__MSK      0x20
#define BMA250_EN_SINGLE_TAP_INT__REG      BMA250_INT_ENABLE1_REG

#define BMA250_EN_ORIENT_INT__POS          6
#define BMA250_EN_ORIENT_INT__LEN          1
#define BMA250_EN_ORIENT_INT__MSK          0x40
#define BMA250_EN_ORIENT_INT__REG          BMA250_INT_ENABLE1_REG

#define BMA250_EN_FLAT_INT__POS            7
#define BMA250_EN_FLAT_INT__LEN            1
#define BMA250_EN_FLAT_INT__MSK            0x80
#define BMA250_EN_FLAT_INT__REG            BMA250_INT_ENABLE1_REG

#define BMA250_EN_HIGHG_X_INT__POS         0
#define BMA250_EN_HIGHG_X_INT__LEN         1
#define BMA250_EN_HIGHG_X_INT__MSK         0x01
#define BMA250_EN_HIGHG_X_INT__REG         BMA250_INT_ENABLE2_REG

#define BMA250_EN_HIGHG_Y_INT__POS         1
#define BMA250_EN_HIGHG_Y_INT__LEN         1
#define BMA250_EN_HIGHG_Y_INT__MSK         0x02
#define BMA250_EN_HIGHG_Y_INT__REG         BMA250_INT_ENABLE2_REG

#define BMA250_EN_HIGHG_Z_INT__POS         2
#define BMA250_EN_HIGHG_Z_INT__LEN         1
#define BMA250_EN_HIGHG_Z_INT__MSK         0x04
#define BMA250_EN_HIGHG_Z_INT__REG         BMA250_INT_ENABLE2_REG

#define BMA250_EN_HIGHG_XYZ_INT__POS         2
#define BMA250_EN_HIGHG_XYZ_INT__LEN         1
#define BMA250_EN_HIGHG_XYZ_INT__MSK         0x04
#define BMA250_EN_HIGHG_XYZ_INT__REG         BMA250_INT_ENABLE2_REG

#define BMA250_EN_LOWG_INT__POS            3
#define BMA250_EN_LOWG_INT__LEN            1
#define BMA250_EN_LOWG_INT__MSK            0x08
#define BMA250_EN_LOWG_INT__REG            BMA250_INT_ENABLE2_REG

#define BMA250_EN_NEW_DATA_INT__POS        4
#define BMA250_EN_NEW_DATA_INT__LEN        1
#define BMA250_EN_NEW_DATA_INT__MSK        0x10
#define BMA250_EN_NEW_DATA_INT__REG        BMA250_INT_ENABLE2_REG

#define BMA250_EN_INT1_PAD_LOWG__POS        0
#define BMA250_EN_INT1_PAD_LOWG__LEN        1
#define BMA250_EN_INT1_PAD_LOWG__MSK        0x01
#define BMA250_EN_INT1_PAD_LOWG__REG        BMA250_INT1_PAD_SEL_REG

#define BMA250_EN_INT1_PAD_HIGHG__POS       1
#define BMA250_EN_INT1_PAD_HIGHG__LEN       1
#define BMA250_EN_INT1_PAD_HIGHG__MSK       0x02
#define BMA250_EN_INT1_PAD_HIGHG__REG       BMA250_INT1_PAD_SEL_REG

#define BMA250_EN_INT1_PAD_SLOPE__POS       2
#define BMA250_EN_INT1_PAD_SLOPE__LEN       1
#define BMA250_EN_INT1_PAD_SLOPE__MSK       0x04
#define BMA250_EN_INT1_PAD_SLOPE__REG       BMA250_INT1_PAD_SEL_REG

#define BMA250_EN_INT1_PAD_DB_TAP__POS      4
#define BMA250_EN_INT1_PAD_DB_TAP__LEN      1
#define BMA250_EN_INT1_PAD_DB_TAP__MSK      0x10
#define BMA250_EN_INT1_PAD_DB_TAP__REG      BMA250_INT1_PAD_SEL_REG

#define BMA250_EN_INT1_PAD_SNG_TAP__POS     5
#define BMA250_EN_INT1_PAD_SNG_TAP__LEN     1
#define BMA250_EN_INT1_PAD_SNG_TAP__MSK     0x20
#define BMA250_EN_INT1_PAD_SNG_TAP__REG     BMA250_INT1_PAD_SEL_REG

#define BMA250_EN_INT1_PAD_ORIENT__POS      6
#define BMA250_EN_INT1_PAD_ORIENT__LEN      1
#define BMA250_EN_INT1_PAD_ORIENT__MSK      0x40
#define BMA250_EN_INT1_PAD_ORIENT__REG      BMA250_INT1_PAD_SEL_REG

#define BMA250_EN_INT1_PAD_FLAT__POS        7
#define BMA250_EN_INT1_PAD_FLAT__LEN        1
#define BMA250_EN_INT1_PAD_FLAT__MSK        0x80
#define BMA250_EN_INT1_PAD_FLAT__REG        BMA250_INT1_PAD_SEL_REG

#define BMA250_EN_INT2_PAD_LOWG__POS        0
#define BMA250_EN_INT2_PAD_LOWG__LEN        1
#define BMA250_EN_INT2_PAD_LOWG__MSK        0x01
#define BMA250_EN_INT2_PAD_LOWG__REG        BMA250_INT2_PAD_SEL_REG

#define BMA250_EN_INT2_PAD_HIGHG__POS       1
#define BMA250_EN_INT2_PAD_HIGHG__LEN       1
#define BMA250_EN_INT2_PAD_HIGHG__MSK       0x02
#define BMA250_EN_INT2_PAD_HIGHG__REG       BMA250_INT2_PAD_SEL_REG

#define BMA250_EN_INT2_PAD_SLOPE__POS       2
#define BMA250_EN_INT2_PAD_SLOPE__LEN       1
#define BMA250_EN_INT2_PAD_SLOPE__MSK       0x04
#define BMA250_EN_INT2_PAD_SLOPE__REG       BMA250_INT2_PAD_SEL_REG

#define BMA250_EN_INT2_PAD_DB_TAP__POS      4
#define BMA250_EN_INT2_PAD_DB_TAP__LEN      1
#define BMA250_EN_INT2_PAD_DB_TAP__MSK      0x10
#define BMA250_EN_INT2_PAD_DB_TAP__REG      BMA250_INT2_PAD_SEL_REG

#define BMA250_EN_INT2_PAD_SNG_TAP__POS     5
#define BMA250_EN_INT2_PAD_SNG_TAP__LEN     1
#define BMA250_EN_INT2_PAD_SNG_TAP__MSK     0x20
#define BMA250_EN_INT2_PAD_SNG_TAP__REG     BMA250_INT2_PAD_SEL_REG

#define BMA250_EN_INT2_PAD_ORIENT__POS      6
#define BMA250_EN_INT2_PAD_ORIENT__LEN      1
#define BMA250_EN_INT2_PAD_ORIENT__MSK      0x40
#define BMA250_EN_INT2_PAD_ORIENT__REG      BMA250_INT2_PAD_SEL_REG

#define BMA250_EN_INT2_PAD_FLAT__POS        7
#define BMA250_EN_INT2_PAD_FLAT__LEN        1
#define BMA250_EN_INT2_PAD_FLAT__MSK        0x80
#define BMA250_EN_INT2_PAD_FLAT__REG        BMA250_INT2_PAD_SEL_REG

#define BMA250_EN_INT1_PAD_NEWDATA__POS     0
#define BMA250_EN_INT1_PAD_NEWDATA__LEN     1
#define BMA250_EN_INT1_PAD_NEWDATA__MSK     0x01
#define BMA250_EN_INT1_PAD_NEWDATA__REG     BMA250_INT_DATA_SEL_REG

#define BMA250_EN_INT2_PAD_NEWDATA__POS     7
#define BMA250_EN_INT2_PAD_NEWDATA__LEN     1
#define BMA250_EN_INT2_PAD_NEWDATA__MSK     0x80
#define BMA250_EN_INT2_PAD_NEWDATA__REG     BMA250_INT_DATA_SEL_REG


#define BMA250_UNFILT_INT_SRC_LOWG__POS        0
#define BMA250_UNFILT_INT_SRC_LOWG__LEN        1
#define BMA250_UNFILT_INT_SRC_LOWG__MSK        0x01
#define BMA250_UNFILT_INT_SRC_LOWG__REG        BMA250_INT_SRC_REG

#define BMA250_UNFILT_INT_SRC_HIGHG__POS       1
#define BMA250_UNFILT_INT_SRC_HIGHG__LEN       1
#define BMA250_UNFILT_INT_SRC_HIGHG__MSK       0x02
#define BMA250_UNFILT_INT_SRC_HIGHG__REG       BMA250_INT_SRC_REG

#define BMA250_UNFILT_INT_SRC_SLOPE__POS       2
#define BMA250_UNFILT_INT_SRC_SLOPE__LEN       1
#define BMA250_UNFILT_INT_SRC_SLOPE__MSK       0x04
#define BMA250_UNFILT_INT_SRC_SLOPE__REG       BMA250_INT_SRC_REG

#define BMA250_UNFILT_INT_SRC_TAP__POS         4
#define BMA250_UNFILT_INT_SRC_TAP__LEN         1
#define BMA250_UNFILT_INT_SRC_TAP__MSK         0x10
#define BMA250_UNFILT_INT_SRC_TAP__REG         BMA250_INT_SRC_REG

#define BMA250_UNFILT_INT_SRC_DATA__POS        5
#define BMA250_UNFILT_INT_SRC_DATA__LEN        1
#define BMA250_UNFILT_INT_SRC_DATA__MSK        0x20
#define BMA250_UNFILT_INT_SRC_DATA__REG        BMA250_INT_SRC_REG

#define BMA250_INT1_PAD_ACTIVE_LEVEL__POS       0
#define BMA250_INT1_PAD_ACTIVE_LEVEL__LEN       1
#define BMA250_INT1_PAD_ACTIVE_LEVEL__MSK       0x01
#define BMA250_INT1_PAD_ACTIVE_LEVEL__REG       BMA250_INT_SET_REG

#define BMA250_INT2_PAD_ACTIVE_LEVEL__POS       2
#define BMA250_INT2_PAD_ACTIVE_LEVEL__LEN       1
#define BMA250_INT2_PAD_ACTIVE_LEVEL__MSK       0x04
#define BMA250_INT2_PAD_ACTIVE_LEVEL__REG       BMA250_INT_SET_REG

#define BMA250_INT1_PAD_OUTPUT_TYPE__POS        1
#define BMA250_INT1_PAD_OUTPUT_TYPE__LEN        1
#define BMA250_INT1_PAD_OUTPUT_TYPE__MSK        0x02
#define BMA250_INT1_PAD_OUTPUT_TYPE__REG        BMA250_INT_SET_REG

#define BMA250_INT2_PAD_OUTPUT_TYPE__POS        3
#define BMA250_INT2_PAD_OUTPUT_TYPE__LEN        1
#define BMA250_INT2_PAD_OUTPUT_TYPE__MSK        0x08
#define BMA250_INT2_PAD_OUTPUT_TYPE__REG        BMA250_INT_SET_REG


#define BMA250_INT_MODE_SEL__POS                0
#define BMA250_INT_MODE_SEL__LEN                4
#define BMA250_INT_MODE_SEL__MSK                0x0F
#define BMA250_INT_MODE_SEL__REG                BMA250_INT_CTRL_REG


#define BMA250_INT_RESET_LATCHED__POS           7
#define BMA250_INT_RESET_LATCHED__LEN           1
#define BMA250_INT_RESET_LATCHED__MSK           0x80
#define BMA250_INT_RESET_LATCHED__REG           BMA250_INT_CTRL_REG

#define BMA250_LOWG_DUR__POS                    0
#define BMA250_LOWG_DUR__LEN                    8
#define BMA250_LOWG_DUR__MSK                    0xFF
#define BMA250_LOWG_DUR__REG                    BMA250_LOW_DURN_REG

#define BMA250_LOWG_THRES__POS                  0
#define BMA250_LOWG_THRES__LEN                  8
#define BMA250_LOWG_THRES__MSK                  0xFF
#define BMA250_LOWG_THRES__REG                  BMA250_LOW_THRES_REG

#define BMA250_LOWG_HYST__POS                   0
#define BMA250_LOWG_HYST__LEN                   2
#define BMA250_LOWG_HYST__MSK                   0x03
#define BMA250_LOWG_HYST__REG                   BMA250_LOW_HIGH_HYST_REG

#define BMA250_LOWG_INT_MODE__POS               2
#define BMA250_LOWG_INT_MODE__LEN               1
#define BMA250_LOWG_INT_MODE__MSK               0x04
#define BMA250_LOWG_INT_MODE__REG               BMA250_LOW_HIGH_HYST_REG

#define BMA250_HIGHG_DUR__POS                    0
#define BMA250_HIGHG_DUR__LEN                    8
#define BMA250_HIGHG_DUR__MSK                    0xFF
#define BMA250_HIGHG_DUR__REG                    BMA250_HIGH_DURN_REG

#define BMA250_HIGHG_THRES__POS                  0
#define BMA250_HIGHG_THRES__LEN                  8
#define BMA250_HIGHG_THRES__MSK                  0xFF
#define BMA250_HIGHG_THRES__REG                  BMA250_HIGH_THRES_REG

#define BMA250_HIGHG_HYST__POS                  6
#define BMA250_HIGHG_HYST__LEN                  2
#define BMA250_HIGHG_HYST__MSK                  0xC0
#define BMA250_HIGHG_HYST__REG                  BMA250_LOW_HIGH_HYST_REG

#define BMA250_SLOPE_DUR__POS                    0
#define BMA250_SLOPE_DUR__LEN                    2
#define BMA250_SLOPE_DUR__MSK                    0x03
#define BMA250_SLOPE_DUR__REG                    BMA250_SLOPE_DURN_REG

#define BMA250_SLOPE_THRES__POS                  0
#define BMA250_SLOPE_THRES__LEN                  8
#define BMA250_SLOPE_THRES__MSK                  0xFF
#define BMA250_SLOPE_THRES__REG                  BMA250_SLOPE_THRES_REG

#define BMA250_TAP_DUR__POS                    0
#define BMA250_TAP_DUR__LEN                    3
#define BMA250_TAP_DUR__MSK                    0x07
#define BMA250_TAP_DUR__REG                    BMA250_TAP_PARAM_REG

#define BMA250_TAP_SHOCK_DURN__POS             6
#define BMA250_TAP_SHOCK_DURN__LEN             1
#define BMA250_TAP_SHOCK_DURN__MSK             0x40
#define BMA250_TAP_SHOCK_DURN__REG             BMA250_TAP_PARAM_REG

#define BMA250_TAP_QUIET_DURN__POS             7
#define BMA250_TAP_QUIET_DURN__LEN             1
#define BMA250_TAP_QUIET_DURN__MSK             0x80
#define BMA250_TAP_QUIET_DURN__REG             BMA250_TAP_PARAM_REG

#define BMA250_TAP_THRES__POS                  0
#define BMA250_TAP_THRES__LEN                  5
#define BMA250_TAP_THRES__MSK                  0x1F
#define BMA250_TAP_THRES__REG                  BMA250_TAP_THRES_REG

#define BMA250_TAP_SAMPLES__POS                6
#define BMA250_TAP_SAMPLES__LEN                2
#define BMA250_TAP_SAMPLES__MSK                0xC0
#define BMA250_TAP_SAMPLES__REG                BMA250_TAP_THRES_REG

#define BMA250_ORIENT_MODE__POS                  0
#define BMA250_ORIENT_MODE__LEN                  2
#define BMA250_ORIENT_MODE__MSK                  0x03
#define BMA250_ORIENT_MODE__REG                  BMA250_ORIENT_PARAM_REG

#define BMA250_ORIENT_BLOCK__POS                 2
#define BMA250_ORIENT_BLOCK__LEN                 2
#define BMA250_ORIENT_BLOCK__MSK                 0x0C
#define BMA250_ORIENT_BLOCK__REG                 BMA250_ORIENT_PARAM_REG

#define BMA250_ORIENT_HYST__POS                  4
#define BMA250_ORIENT_HYST__LEN                  3
#define BMA250_ORIENT_HYST__MSK                  0x70
#define BMA250_ORIENT_HYST__REG                  BMA250_ORIENT_PARAM_REG

#define BMA250_ORIENT_AXIS__POS                  7
#define BMA250_ORIENT_AXIS__LEN                  1
#define BMA250_ORIENT_AXIS__MSK                  0x80
#define BMA250_ORIENT_AXIS__REG                  BMA250_THETA_BLOCK_REG

#define BMA250_THETA_BLOCK__POS                  0
#define BMA250_THETA_BLOCK__LEN                  6
#define BMA250_THETA_BLOCK__MSK                  0x3F
#define BMA250_THETA_BLOCK__REG                  BMA250_THETA_BLOCK_REG

#define BMA250_THETA_FLAT__POS                  0
#define BMA250_THETA_FLAT__LEN                  6
#define BMA250_THETA_FLAT__MSK                  0x3F
#define BMA250_THETA_FLAT__REG                  BMA250_THETA_FLAT_REG

#define BMA250_FLAT_HOLD_TIME__POS              4
#define BMA250_FLAT_HOLD_TIME__LEN              2
#define BMA250_FLAT_HOLD_TIME__MSK              0x30
#define BMA250_FLAT_HOLD_TIME__REG              BMA250_FLAT_HOLD_TIME_REG

#define BMA250_EN_SELF_TEST__POS                0
#define BMA250_EN_SELF_TEST__LEN                2
#define BMA250_EN_SELF_TEST__MSK                0x03
#define BMA250_EN_SELF_TEST__REG                BMA250_SELF_TEST_REG



#define BMA250_NEG_SELF_TEST__POS               2
#define BMA250_NEG_SELF_TEST__LEN               1
#define BMA250_NEG_SELF_TEST__MSK               0x04
#define BMA250_NEG_SELF_TEST__REG               BMA250_SELF_TEST_REG


#define BMA250_LOW_POWER_MODE_S__POS            0
#define BMA250_LOW_POWER_MODE_S__LEN            1
#define BMA250_LOW_POWER_MODE_S__MSK            0x01
#define BMA250_LOW_POWER_MODE_S__REG            BMA250_STATUS_LOW_POWER_REG

#define BMA250_EN_FAST_COMP__POS                5
#define BMA250_EN_FAST_COMP__LEN                2
#define BMA250_EN_FAST_COMP__MSK                0x60
#define BMA250_EN_FAST_COMP__REG                BMA250_OFFSET_CTRL_REG

#define BMA250_FAST_COMP_RDY_S__POS             4
#define BMA250_FAST_COMP_RDY_S__LEN             1
#define BMA250_FAST_COMP_RDY_S__MSK             0x10
#define BMA250_FAST_COMP_RDY_S__REG             BMA250_OFFSET_CTRL_REG

#define BMA250_COMP_TARGET_OFFSET_X__POS        1
#define BMA250_COMP_TARGET_OFFSET_X__LEN        2
#define BMA250_COMP_TARGET_OFFSET_X__MSK        0x06
#define BMA250_COMP_TARGET_OFFSET_X__REG        BMA250_OFFSET_PARAMS_REG

#define BMA250_COMP_TARGET_OFFSET_Y__POS        3
#define BMA250_COMP_TARGET_OFFSET_Y__LEN        2
#define BMA250_COMP_TARGET_OFFSET_Y__MSK        0x18
#define BMA250_COMP_TARGET_OFFSET_Y__REG        BMA250_OFFSET_PARAMS_REG

#define BMA250_COMP_TARGET_OFFSET_Z__POS        5
#define BMA250_COMP_TARGET_OFFSET_Z__LEN        2
#define BMA250_COMP_TARGET_OFFSET_Z__MSK        0x60
#define BMA250_COMP_TARGET_OFFSET_Z__REG        BMA250_OFFSET_PARAMS_REG

#define BMA250_UNLOCK_EE_WRITE_SETTING__POS     0
#define BMA250_UNLOCK_EE_WRITE_SETTING__LEN     1
#define BMA250_UNLOCK_EE_WRITE_SETTING__MSK     0x01
#define BMA250_UNLOCK_EE_WRITE_SETTING__REG     BMA250_EEPROM_CTRL_REG

#define BMA250_START_EE_WRITE_SETTING__POS      1
#define BMA250_START_EE_WRITE_SETTING__LEN      1
#define BMA250_START_EE_WRITE_SETTING__MSK      0x02
#define BMA250_START_EE_WRITE_SETTING__REG      BMA250_EEPROM_CTRL_REG

#define BMA250_EE_WRITE_SETTING_S__POS          2
#define BMA250_EE_WRITE_SETTING_S__LEN          1
#define BMA250_EE_WRITE_SETTING_S__MSK          0x04
#define BMA250_EE_WRITE_SETTING_S__REG          BMA250_EEPROM_CTRL_REG

#define BMA250_RANGE_2G                 0
#define BMA250_RANGE_4G                 1
#define BMA250_RANGE_8G                 2
#define BMA250_RANGE_16G                3

#define BMA250_BW_7_81HZ        0x08
#define BMA250_BW_15_63HZ       0x09
#define BMA250_BW_31_25HZ       0x0A
#define BMA250_BW_62_50HZ       0x0B
#define BMA250_BW_125HZ         0x0C
#define BMA250_BW_250HZ         0x0D
#define BMA250_BW_500HZ         0x0E
#define BMA250_BW_1000HZ        0x0F

#define BMA250_MODE_NORMAL      0
#define BMA250_MODE_LOWPOWER    1
#define BMA250_MODE_SUSPEND     2

#define BMA250_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define BMA250_SET_BITSLICE(regvar, bitname, val)\
		((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

#define BMA250_I2C_NAME "bma250"

#define BMAIO			0xA1

#define bma250_CHIP_ID_REG	0x00
#define bma250_X_AXIS_LSB_REG	0x2
#define bma250_RANGE_SEL_REG	0x0F
#define bma250_BW_SEL_REG	0x10
#define bma250_MODE_CTRL_REG	0x11

#define BMA_IOCTL_INIT			_IO(BMAIO, 0x31)
#define BMA_IOCTL_WRITE			_IOW(BMAIO, 0x32, char[5])
#define BMA_IOCTL_READ			_IOWR(BMAIO, 0x33, char[5])
#define BMA_IOCTL_READ_ACCELERATION	_IOWR(BMAIO, 0x34, short[7])
#define BMA_IOCTL_SET_MODE		_IOW(BMAIO, 0x35, short)
#define BMA_IOCTL_GET_INT		_IOR(BMAIO, 0x36, short)
#define BMA_IOCTL_GET_CHIP_LAYOUT	_IOR(BMAIO, 0x37, short)
#define BMA_IOCTL_GET_CALI_MODE		_IOR(BMAIO, 0x38, short)
#define BMA_IOCTL_SET_CALI_MODE		_IOW(BMAIO, 0x39, short)
#define BMA_IOCTL_READ_CALI_VALUE       _IOR(BMAIO, 0x3a, char[3])
#define BMA_IOCTL_WRITE_CALI_VALUE      _IOW(BMAIO, 0x3b, int)
#define BMA_IOCTL_GET_UPDATE_USER_CALI_DATA    _IOR(BMAIO, 0x3c, short)
#define BMA_IOCTL_SET_UPDATE_USER_CALI_DATA    _IOW(BMAIO, 0x3d, short)

#define BMA_RANGE_2G		0x3
#define BMA_RANGE_4G		0x5
#define BMA_RANGE_8G		0x8
#define BMA_RANGE_16G		0xC

#define BMA_BW_7_81HZ		0x8
#define BMA_BW_15_63HZ		0x9
#define BMA_BW_31_25HZ		0xA
#define BMA_BW_62_5HZ		0xB
#define BMA_BW_125HZ		0xC
#define BMA_BW_250HZ		0xD
#define BMA_BW_500HZ		0xE
#define BMA_BW_1000HZ		0xF

#define E_OUT_OF_RANGE          (char)(-2)

#define bma250_MODE_NORMAL      0
#define bma250_MODE_SUSPEND     1

extern unsigned int gs_kvalue;

struct bma250_platform_data {
	int intr;
	int chip_layout;
	int calibration_mode;
	int gs_kvalue;
	int SR_3v_used;
	unsigned int (*G_Sensor_Compass_POR)(void);

        
        u8 axis_map_x;
        u8 axis_map_y;
        u8 axis_map_z;

        u8 negate_x;
        u8 negate_y;
        u8 negate_z;

	int (*power_LPM)(int on);
};

#endif
