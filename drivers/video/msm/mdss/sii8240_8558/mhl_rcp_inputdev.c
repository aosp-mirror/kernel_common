/*

SiI8558 / SiI8240 Linux Driver

Copyright (C) 2013 Silicon Image, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation version 2.
This program is distributed AS-IS WITHOUT ANY WARRANTY of any
kind, whether express or implied; INCLUDING without the implied warranty
of MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE or NON-INFRINGEMENT.  See
the GNU General Public License for more details at http://www.gnu.org/licenses/gpl-2.0.html.

*/

/**
   @file mhl_rcp_inputdev.c
*/

#include <linux/input.h>
#include <linux/cdev.h>
#include <linux/hrtimer.h>
#include "si_fw_macros.h"
#include "si_mhl_defs.h"
#include "si_infoframe.h"
#include "si_edid.h"
#include "si_mhl2_edid_3d_api.h"
#include "si_mhl_tx_hw_drv_api.h"
#ifdef MEDIA_DATA_TUNNEL_SUPPORT
#include "si_mdt_inputdev.h"
#endif
#include "mhl_linux_tx.h"
#include "platform.h"
#include "mhl_rcp_inputdev.h"

typedef enum{
	ph0_idle
	,ph3_press_and_hold_key
	,ph8_hold_mode

	,num_rcp_states
}rcp_state_e;

typedef enum{
	 rcp_received_and_acked_normal_key_press
	,rcp_received_and_acked_normal_key_press_same
	,rcp_received_and_acked_normal_key_release
	,rcp_received_and_acked_normal_key_release_same
	,rcp_received_and_acked_press_and_hold_key_press
	,rcp_received_and_acked_press_and_hold_key_press_same
	,rcp_received_and_acked_press_and_hold_key_release
	,rcp_received_and_acked_press_and_hold_key_release_same
	,rcp_T_hold_maintain_expired
	,rcp_T_press_mode_expired

	,num_rcp_events
}rcp_event_e;

#if defined(DEBUG)
static char *state_strings[num_rcp_states]=
{
	"idle"
	,"press_and_hold_key"
	,"hold_mode"
};

static char *event_strings[num_rcp_events]=
{
	 "normal_key_press"
	,"normal_key_press_same"
	,"normal_key_release"
	,"normal_key_release_same"
	,"press_and_hold_key_press"
	,"press_and_hold_key_press_same"
	,"press_and_hold_key_release"
	,"press_and_hold_key_release_same"
	,"rcp_T_hold_maintain_expired"
	,"rcp_T_press_mode_expired"
};
#endif

rcp_state_e current_rcp_state=ph0_idle;
uint8_t rcp_previous_key=0,rcp_current_key=0;

rcp_keymap_t	rcpSupportTable [MHL_NUM_RCP_KEY_CODES] = {
/* 0x00=Select		*/	 {0,0,0,{KEY_ENTER		,0},(MHL_DEV_LD_GUI)	}
/* 0x00=Up		*/	,{0,1,0,{KEY_UP			,0},(MHL_DEV_LD_GUI)	}
/* 0x02=Down		*/	,{0,1,0,{KEY_DOWN		,0},(MHL_DEV_LD_GUI)	}
/* 0x03=Left		*/	,{0,1,0,{KEY_LEFT		,0},(MHL_DEV_LD_GUI)	}
/* 0x04=Right		*/	,{0,1,0,{KEY_RIGHT		,0},(MHL_DEV_LD_GUI)	}
/* 0x05 = right-up	*/	,{1,1,0,{KEY_RIGHT 	,KEY_UP	  },(MHL_DEV_LD_GUI)	}
/* 0x06 = right-down	*/	,{1,1,0,{KEY_LEFT	,KEY_DOWN },(MHL_DEV_LD_GUI)	}
/* 0x07 = left-up	*/	,{1,1,0,{KEY_LEFT	,KEY_UP   },(MHL_DEV_LD_GUI)	}
/* 0x08 = left-down	*/	,{1,1,0,{KEY_LEFT	,KEY_DOWN },(MHL_DEV_LD_GUI)	}
/* 0x09=RootMenu	*/	,{0,0,0,{KEY_HOMEPAGE		,0},(MHL_DEV_LD_GUI)	}
/* 0x0AReserved		*/	,{0,0,0,{KEY_UNKNOWN		,0},0			}
/* 0x0BReserved		*/	,{0,0,0,{KEY_UNKNOWN		,0},0			}
/* 0x0CReserved		*/	,{0,0,0,{KEY_BOOKMARKS		,0},0			}
/* 0x0D=Exit		*/	,{0,0,0,{KEY_BACK		,0},(MHL_DEV_LD_GUI)	}
/* 0x0EReserved		*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x0FReserved		*/	,{0,0,0,{KEY_RESERVED		,0},0			}

/* 0x10 Reserved	*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x10 Reserved	*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x12 Reserved	*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x13 Reserved	*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x14 Reserved	*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x15 Reserved	*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x16 Reserved	*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x17 Reserved	*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x18 Reserved	*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x19 Reserved	*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x1A Reserved	*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x1B Reserved	*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x1C Reserved	*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x1D Reserved	*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x1E Reserved	*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x1F Reserved	*/	,{0,0,0,{KEY_RESERVED		,0},0			}

/* Numeric keys 0x20	*/	,{0,0,0,{KEY_NUMERIC_0		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)	}
/* Numeric keys 0x21	*/	,{0,0,0,{KEY_NUMERIC_1		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)	}
/* Numeric keys 0x22	*/	,{0,0,0,{KEY_NUMERIC_2		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)	}
/* Numeric keys 0x23	*/	,{0,0,0,{KEY_NUMERIC_3		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)	}
/* Numeric keys 0x24	*/	,{0,0,0,{KEY_NUMERIC_4		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)	}
/* Numeric keys 0x25	*/	,{0,0,0,{KEY_NUMERIC_5		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)	}
/* Numeric keys 0x26	*/	,{0,0,0,{KEY_NUMERIC_6		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)	}
/* Numeric keys 0x27	*/	,{0,0,0,{KEY_NUMERIC_7		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)	}
/* Numeric keys 0x28	*/	,{0,0,0,{KEY_NUMERIC_8		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)	}
/* Numeric keys 0x29	*/	,{0,0,0,{KEY_NUMERIC_9		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)	}
/* 0x2A = Dot		*/	,{0,0,0,{KEY_DOT		,0},0			}
/* Enter key = 0x2B	*/	,{0,0,0,{KEY_ENTER		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)	}
/* Clear key = 0x2C	*/	,{0,0,0,{KEY_CLEAR		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER)	}
/* 2D Reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 2E Reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 2F Reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0			}

/* 0x30 = Channel Up	*/	,{0,1,0,{KEY_CHANNELUP		,0},(MHL_DEV_LD_TUNER)		}
/* 0x30 = Channel Dn	*/	,{0,1,0,{KEY_CHANNELDOWN	,0},(MHL_DEV_LD_TUNER)		}
/* 0x32 = Previous Channel	*/	,{0,0,0,{KEY_UNKNOWN		,0},(MHL_DEV_LD_TUNER)	}
/* 0x33 = Sound Select		*/	,{0,0,0,{KEY_SOUND		,0},(MHL_DEV_LD_AUDIO)	}
/* 0x34 = Input Select		*/	,{0,0,0,{KEY_UNKNOWN		,0},0			}
/* 0x35 = Show Information	*/	,{0,0,0,{KEY_PROGRAM		,0},0			}
/* 0x36 = Help      		*/	,{0,0,0,{KEY_UNKNOWN		,0},0			}
/* 0x37 = Page Up   		*/	,{0,1,0,{KEY_PAGEUP		,0},0			}
/* 0x38 = Page Down 		*/	,{0,1,0,{KEY_PAGEDOWN		,0},0			}
/* 0x39-0x3F Reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x39-0x3F Reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x39-0x3F Reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x39-0x3F Reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x39-0x3F Reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x39-0x3F Reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x39-0x3F Reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0			}

/* 0x40 = Undefined		*/ 	,{0,0,0,{KEY_RESERVED		,0},0			}
/* 0x40 = Volume Up		*/ 	,{0,1,0,{KEY_VOLUMEUP		,0},(MHL_DEV_LD_SPEAKER)	}
/* 0x42 = Volume Down		*/ 	,{0,1,0,{KEY_VOLUMEDOWN		,0},(MHL_DEV_LD_SPEAKER)	}
/* 0x43 = Mute 			*/ 	,{0,0,0,{KEY_MUTE		,0},(MHL_DEV_LD_SPEAKER)	}
/* 0x44 = Play			*/ 	,{0,0,0,{KEY_PLAY		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO)	}
/* 0x45 = Stop 			*/	,{0,0,0,{KEY_STOP		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_RECORD)	}
/* 0x46 = Pause			*/	,{0,0,0,{KEY_PLAYPAUSE		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_RECORD)	}
/* 0x47 = Record		*/ 	,{0,0,0,{KEY_RECORD		,0},(MHL_DEV_LD_RECORD)	}
/* 0x48 = Rewind		*/	,{0,1,0,{KEY_REWIND		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO)	}
/* 0x49 = Fast Forward		*/	,{0,1,0,{KEY_FASTFORWARD	,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO)	}
/* 0x4A = Eject			*/	,{0,0,0,{KEY_EJECTCD		,0},(MHL_DEV_LD_MEDIA)	}
/* 0x4B = Forward		*/ 	,{0,1,0,{KEY_NEXTSONG		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA)	}
/* 0x4C = Backward		*/	,{0,1,0,{KEY_PREVIOUSSONG	,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA)	}
/* 4D Reserved			*/ 	,{0,0,0,{KEY_RESERVED		,0},0}
/* 4E Reserved			*/ 	,{0,0,0,{KEY_RESERVED		,0},0}
/* 4F Reserved			*/ 	,{0,0,0,{KEY_RESERVED		,0},0}

/* 0x50 = Angle			*/	,{0,0,0,{KEY_UNKNOWN		,0},0}
/* 0x50 = Subpicture		*/	,{0,0,0,{KEY_UNKNOWN		,0},0}
/* 52 Reserved			*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 53 Reserved			*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 54 Reserved 			*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 55 Reserved 			*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 56 Reserved			*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 57 Reserved			*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 58 Reserved 			*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 59 Reserved 			*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 5A Reserved 			*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 5B Reserved 			*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 5C Reserved  		*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 5D Reserved  		*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 5E Reserved  		*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 5F Reserved  		*/	,{0,0,0,{KEY_RESERVED		,0},0}

/* 0x60 = Play 			*/	,{0,0,0,{KEY_PLAYPAUSE 		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO)	}
/* 0x60 = Pause the Play	*/	,{0,0,0,{KEY_PLAYPAUSE		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO)	}
/* 0x62 = Record 		*/	,{0,0,0,{KEY_RECORD		,0},(MHL_DEV_LD_RECORD)	}
/* 0x63 = Pause the Record	*/	,{0,0,0,{KEY_PAUSE 		,0},(MHL_DEV_LD_RECORD)	}
/* 0x64 = Stop			*/	,{0,0,0,{KEY_STOP		,0},(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_RECORD)	}
/* 0x65 = Mute			*/	,{0,0,0,{KEY_MUTE		,0},(MHL_DEV_LD_SPEAKER)	}
/* 0x66 = Restore Mute 		*/	,{0,0,0,{KEY_MUTE 		,0},(MHL_DEV_LD_SPEAKER)	}
/* 0x67 Undefined		*/	,{0,0,0,{KEY_UNKNOWN		,0},0}
/* 0x68 Undefined		*/	,{0,0,0,{KEY_UNKNOWN		,0},0}
/* 0x69 reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 0x6A reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 0x6B reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 0x6C reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 0x6D reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 0x6E reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 0x6F reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0}

/* 0x70 reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 0x70 reserved		*/	,{0,0,0,{KEY_F1 		,0},0}
/* 0x72 reserved		*/	,{0,0,0,{KEY_F2 		,0},0}
/* 0x73 reserved		*/	,{0,0,0,{KEY_F3 		,0},0}
/* 0x74 reserved		*/	,{0,0,0,{KEY_F4 		,0},0}
/* 0x75 reserved		*/	,{0,0,0,{KEY_F5 		,0},0}
/* 0x76 reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 0x77 reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 0x78 reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 0x79 reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 0x7A reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 0x7B reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 0x7C reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 0x7D reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0}
/* 0x7E Undefined		*/	,{0,0,0,{KEY_VENDOR		,0},0}
/* 0x7F reserved		*/	,{0,0,0,{KEY_RESERVED		,0},0}
};

static u16 rcp_def_keymap[MHL_NUM_RCP_KEY_CODES]
#ifdef OLD_KEYMAP_TABLE //(
= {
	KEY_SELECT,
	KEY_UP,
	KEY_DOWN,
	KEY_LEFT,
	KEY_RIGHT,
	KEY_UNKNOWN,	/* right-up */
	KEY_UNKNOWN,	/* right-down */
	KEY_UNKNOWN,	/* left-up */
	KEY_UNKNOWN,	/* left-down */
	KEY_MENU,
	KEY_UNKNOWN,	/* setup */
	KEY_UNKNOWN,	/* contents */
	KEY_UNKNOWN,	/* favorite */
	KEY_EXIT,
	KEY_RESERVED,	/* 0x0e */
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x1F */
	KEY_NUMERIC_0,
	KEY_NUMERIC_1,
	KEY_NUMERIC_2,
	KEY_NUMERIC_3,
	KEY_NUMERIC_4,
	KEY_NUMERIC_5,
	KEY_NUMERIC_6,
	KEY_NUMERIC_7,
	KEY_NUMERIC_8,
	KEY_NUMERIC_9,
	KEY_DOT,
	KEY_ENTER,
	KEY_CLEAR,
	KEY_RESERVED,	/* 0x2D */
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x2F */
	KEY_UNKNOWN,	/* channel up */
	KEY_UNKNOWN,	/* channel down */
	KEY_UNKNOWN,	/* previous channel */
	KEY_UNKNOWN,	/* sound select */
	KEY_UNKNOWN,	/* input select */
	KEY_UNKNOWN,	/* show information */
	KEY_UNKNOWN,	/* help */
	KEY_UNKNOWN,	/* page up */
	KEY_UNKNOWN,	/* page down */
	KEY_RESERVED,	/* 0x39 */
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x3F */
	KEY_RESERVED,	/* 0x40 */
	KEY_UNKNOWN,	/* volume up */
	KEY_UNKNOWN,	/* volume down */
	KEY_UNKNOWN,	/* mute */
	KEY_PLAY,
	KEY_STOP,
	KEY_PLAYPAUSE,
	KEY_UNKNOWN,	/* record */
	KEY_REWIND,
	KEY_FASTFORWARD,
	KEY_UNKNOWN,	/* eject */
	KEY_NEXTSONG,
	KEY_PREVIOUSSONG,
	KEY_RESERVED,	/* 0x4D */
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x4F */
	KEY_UNKNOWN,	/* angle */
	KEY_UNKNOWN,	/* subtitle */
	KEY_RESERVED,	/* 0x52 */
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x5F */
	KEY_PLAY,
	KEY_PAUSE,
	KEY_UNKNOWN,	/* record_function */
	KEY_UNKNOWN,	/* pause_record_function */
	KEY_STOP,
	KEY_UNKNOWN,	/* mute_function */
	KEY_UNKNOWN,	/* restore_volume_function */
	KEY_UNKNOWN,	/* tune_function */
	KEY_UNKNOWN,	/* select_media_function */
	KEY_RESERVED,	/* 0x69 */
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x70 */
	KEY_UNKNOWN,	/* F1 */
	KEY_UNKNOWN,	/* F2 */
	KEY_UNKNOWN,	/* F3 */
	KEY_UNKNOWN,	/* F4 */
	KEY_UNKNOWN,	/* F5 */
	KEY_RESERVED,	/* 0x76 */
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,	/* 0x7D */
	KEY_VENDOR,
	KEY_RESERVED,	/* 0x7F */
}
#endif //)
;

#ifdef OLD_KEYMAP_TABLE //(
int generate_rcp_input_event(struct mhl_dev_context *dev_context, uint8_t rcp_keycode)
{
	int	status = -EINVAL;

	if (dev_context->rcp_input_dev){
		if (rcp_keycode < ARRAY_SIZE(rcp_def_keymap) &&
				rcp_def_keymap[rcp_keycode] != KEY_UNKNOWN &&
				rcp_def_keymap[rcp_keycode] != KEY_RESERVED) {

			input_report_key(dev_context->rcp_input_dev, rcp_keycode, 1);
			input_report_key(dev_context->rcp_input_dev, rcp_keycode, 0);
			input_sync(dev_context->rcp_input_dev);

			status = 0;
		}
	}
	return status;
}
#else //)(

static int rcp_trigger_key_action(struct mhl_dev_context *dev_context,uint8_t index,bool press_release)
{
	int status = -EINVAL;

	if (press_release == 0)
		return 0;

	if (dev_context->rcp_input_dev) {
		input_report_key(dev_context->rcp_input_dev, rcp_def_keymap[index], press_release);
		input_sync(dev_context->rcp_input_dev);
		input_report_key(dev_context->rcp_input_dev, rcp_def_keymap[index], !press_release);
		input_sync(dev_context->rcp_input_dev);
		status = 0;
	}
	return status;
}
static int handle_rcp_event(struct mhl_dev_context *dev_context, uint8_t current_key,
				uint8_t prev_key,rcp_event_e event)
{
	int status = 0;
	uint8_t current_index = current_key & MHL_RCP_KEY_ID_MASK;
        /*uint8_t prev_index = prev_key & MHL_RCP_KEY_ID_MASK;*/

	MHL_TX_DBG_ERR(,"received 0x%02x: %s(%d) in state: %s(%d)\n"
					,current_key
					,event_strings[event],event
					,state_strings[current_rcp_state],current_rcp_state
					);
	/* now process the event according to the current state */
	switch (event) {
	case rcp_received_and_acked_normal_key_press:
	case rcp_received_and_acked_normal_key_press_same:
		status = rcp_trigger_key_action(dev_context,current_index,1);
		/* no update for current_rcp_state */
		break;
	case rcp_received_and_acked_normal_key_release:
	case rcp_received_and_acked_normal_key_release_same:
		status = rcp_trigger_key_action(dev_context,current_index,0);
		/* no update for current_rcp_state */
	default:
		MHL_TX_DBG_ERR(,"[MHL] case error");
		break;
	}
#ifdef MHL_RCP_PRESS_AND_HOLD  	/*Due to only support simple press/release only mark out ""homad and press" case.*/
	switch(current_rcp_state) {
	case ph0_idle:
		switch(event){
		case rcp_received_and_acked_normal_key_press:
		case rcp_received_and_acked_normal_key_press_same:
			status = rcp_trigger_key_action(dev_context,current_index,1);
			/* no update for current_rcp_state */
			break;
		case rcp_received_and_acked_normal_key_release:
		case rcp_received_and_acked_normal_key_release_same:
			status = rcp_trigger_key_action(dev_context,current_index,0);
			/* no update for current_rcp_state */
			break;
		case rcp_received_and_acked_press_and_hold_key_press:
		case rcp_received_and_acked_press_and_hold_key_press_same:
			mhl_tx_start_timer(dev_context
						, dev_context->timer_T_press_mode
						, T_PRESS_MODE);
			current_rcp_state = ph3_press_and_hold_key;
			break;

		case rcp_received_and_acked_press_and_hold_key_release:
		case rcp_received_and_acked_press_and_hold_key_release_same:
			MHL_TX_DBG_ERR(,"unexpected %s(%d) in state: %s(%d)\n"
					,event_strings[event],event
					,state_strings[current_rcp_state],current_rcp_state
					);
			break;
		default:
			MHL_TX_DBG_ERR(,"unexpected event: %d in state: %d\n"
							,event,current_rcp_state);
			/* no update for current_rcp_state */
			status = -EINVAL;
		}
		break;
	case ph3_press_and_hold_key:
		switch(event){
		case rcp_received_and_acked_normal_key_press:
		case rcp_received_and_acked_normal_key_press_same:
			mhl_tx_stop_timer(dev_context
						, dev_context->timer_T_press_mode);
			rcp_trigger_key_action(dev_context,prev_index,0);
			/* OK to overwrite status */
			status = rcp_trigger_key_action(dev_context,current_index,1);
			current_rcp_state = ph0_idle;
			break;
		case rcp_received_and_acked_normal_key_release:
		case rcp_received_and_acked_normal_key_release_same:
			mhl_tx_stop_timer(dev_context
						, dev_context->timer_T_press_mode);
			rcp_trigger_key_action(dev_context,prev_index,0);
			rcp_trigger_key_action(dev_context,current_index,1);
			status = rcp_trigger_key_action(dev_context,current_index,0);
			current_rcp_state = ph0_idle;
			break;
		case rcp_received_and_acked_press_and_hold_key_press:
			mhl_tx_start_timer(dev_context
						, dev_context->timer_T_press_mode
						, T_PRESS_MODE);
			status = rcp_trigger_key_action(dev_context,prev_index,1);
			/* no update for current_rcp_state */
			break;
		case rcp_received_and_acked_press_and_hold_key_press_same:
			mhl_tx_stop_timer(dev_context
						, dev_context->timer_T_press_mode);
			mhl_tx_start_timer(dev_context
						, dev_context->timer_T_hold_maintain
						, T_HOLD_MAINTAIN);
			status = rcp_trigger_key_action(dev_context,prev_index,1);
			current_rcp_state = ph8_hold_mode;
			break;
		case rcp_received_and_acked_press_and_hold_key_release:
		case rcp_received_and_acked_press_and_hold_key_release_same:
			mhl_tx_stop_timer(dev_context
						, dev_context->timer_T_press_mode);
			status = rcp_trigger_key_action(dev_context,prev_index,0);
			current_rcp_state = ph0_idle;
			break;
		case rcp_T_press_mode_expired:
			mhl_tx_start_timer(dev_context
						, dev_context->timer_T_hold_maintain
						, T_HOLD_MAINTAIN);
			status = rcp_trigger_key_action(dev_context,prev_index,0);
			current_rcp_state = ph8_hold_mode;
			break;
		default:
			MHL_TX_DBG_ERR(,"unexpected event: %d in state: %d\n"
							,event,current_rcp_state);
			/* no update for current_rcp_state */
			status = -EINVAL;
		}
		break;
	case ph8_hold_mode:
		switch(event){
		case rcp_received_and_acked_normal_key_press:
		case rcp_received_and_acked_normal_key_press_same:
			mhl_tx_stop_timer(dev_context
						, dev_context->timer_T_hold_maintain);
			rcp_trigger_key_action(dev_context,prev_index,0);
			status = rcp_trigger_key_action(dev_context,current_index,1);
			current_rcp_state = ph0_idle;
			break;
		case rcp_received_and_acked_normal_key_release:
		case rcp_received_and_acked_normal_key_release_same:
			mhl_tx_stop_timer(dev_context
						, dev_context->timer_T_hold_maintain);
			rcp_trigger_key_action(dev_context,prev_index,0);
			rcp_trigger_key_action(dev_context,current_index,1);
			status = rcp_trigger_key_action(dev_context,current_index,0);
			current_rcp_state = ph0_idle;
			break;
		case rcp_received_and_acked_press_and_hold_key_press:
			mhl_tx_stop_timer(dev_context
						, dev_context->timer_T_hold_maintain);
			mhl_tx_start_timer(dev_context
						, dev_context->timer_T_press_mode
						, T_PRESS_MODE);
			status = rcp_trigger_key_action(dev_context,prev_index,1);
			current_rcp_state = ph3_press_and_hold_key;
			break;
		case rcp_received_and_acked_press_and_hold_key_press_same:
			mhl_tx_start_timer(dev_context
						, dev_context->timer_T_hold_maintain
						, T_HOLD_MAINTAIN);
			status = rcp_trigger_key_action(dev_context,prev_index,1);
			/* no update for current_rcp_state */
			break;
		case rcp_received_and_acked_press_and_hold_key_release:
			mhl_tx_stop_timer(dev_context
						, dev_context->timer_T_hold_maintain);
			rcp_trigger_key_action(dev_context,prev_index,0);
			rcp_trigger_key_action(dev_context,current_index,1);
			status = rcp_trigger_key_action(dev_context,current_index,0);
			current_rcp_state = ph0_idle;
			break;
		case rcp_received_and_acked_press_and_hold_key_release_same:
			mhl_tx_stop_timer(dev_context
						, dev_context->timer_T_hold_maintain);
			status = rcp_trigger_key_action(dev_context,prev_index,0);
			current_rcp_state = ph0_idle;
			break;
		case rcp_T_hold_maintain_expired:
			status = rcp_trigger_key_action(dev_context,prev_index,0);
			current_rcp_state = ph0_idle;
			break;
		default:
			MHL_TX_DBG_ERR(,"unexpected event: %d in state: %d\n"
							,event,current_rcp_state);
			/* no update for current_rcp_state */
			status = -EINVAL;
		}
		break;
	default:
		MHL_TX_DBG_ERR(,"irrational state value:%d\n",current_rcp_state);
	}
#endif
	return status;
}

static void timer_callback_T_hold_maintain_handler(void *param)
{
	struct mhl_dev_context *dev_context = (struct mhl_dev_context *)param;
	handle_rcp_event(dev_context,rcp_current_key,rcp_previous_key
					,rcp_T_hold_maintain_expired);
}

static void timer_callback_T_press_mode_handler(void *param)
{
	struct mhl_dev_context *dev_context = (struct mhl_dev_context *)param;
	handle_rcp_event(dev_context,rcp_current_key,rcp_previous_key
					,rcp_T_press_mode_expired);
}

int generate_rcp_input_event(struct mhl_dev_context *dev_context,
							 uint8_t rcp_keycode)
{
	/*
		Since, in MHL, bit 7 == 1 indicates key release,
			and, in Linux, zero means key release,
			we use XOR (^) to invert the sense.
	*/
	int status = -EINVAL;
	int index = rcp_keycode & MHL_RCP_KEY_ID_MASK;

	if (rcp_def_keymap[index] != KEY_UNKNOWN &&
		rcp_def_keymap[index] != KEY_RESERVED){

		rcp_event_e event;
		int mhl_key_press = (rcp_keycode & MHL_RCP_KEY_RELEASED_MASK)? 0 : 1;
		if (mhl_key_press) {
#ifdef MHL_RCP_PRESS_AND_HOLD /*Due to only support simple press/release only mark out ""homad and press" case.*/
			if (rcpSupportTable[index].press_and_hold_key){
				if (index == rcp_previous_key) {
					event = rcp_received_and_acked_press_and_hold_key_press_same;
				} else {
					event = rcp_received_and_acked_press_and_hold_key_press;
				}
			}else{
				if (index == rcp_previous_key) {
					event = rcp_received_and_acked_normal_key_press_same;
				} else {
					event = rcp_received_and_acked_normal_key_press;
				}
			}
#endif
			event = rcp_received_and_acked_normal_key_press;
		} else {
#ifdef MHL_RCP_PRESS_AND_HOLD
			if (rcpSupportTable[index].press_and_hold_key){
				if (index == rcp_previous_key){
					event = rcp_received_and_acked_press_and_hold_key_release_same;
				}else{
					event = rcp_received_and_acked_press_and_hold_key_release;
				}
			}else{
				if (index == rcp_previous_key){
					event = rcp_received_and_acked_normal_key_release_same;
				}else{
					event = rcp_received_and_acked_normal_key_release;
				}
			}
#endif
			event = rcp_received_and_acked_normal_key_release;
		}
		status = handle_rcp_event(dev_context, rcp_keycode,rcp_current_key,event);
	}

	rcp_previous_key = rcp_current_key;
	rcp_current_key  = rcp_keycode;

	return status;
}

#endif //)
uint8_t init_rcp_input_dev(struct mhl_dev_context *dev_context)
{
	int i;
	uint8_t error;
	struct input_dev	*rcp_input_dev;
	int ret;

	if (dev_context->rcp_input_dev != NULL) {
		MHL_TX_DBG_INFO(dev_context, "RCP input device already exists!\n");
		return 0;
	}

	rcp_input_dev = input_allocate_device();
	if (!rcp_input_dev) {
		MHL_TX_DBG_ERR(dev_context, "Failed to allocate RCP input device\n");
		return -ENOMEM;
	}

	set_bit(EV_KEY, rcp_input_dev->evbit);

//	rcp_input_dev->phys        = "mdt_kbd/input0";
	rcp_input_dev->name        = "MHL Remote Control";
	rcp_input_dev->keycode     = rcp_def_keymap;
	rcp_input_dev->keycodesize = sizeof(u16);
	rcp_input_dev->keycodemax  = ARRAY_SIZE(rcp_def_keymap);

	for (i = 1; i < ARRAY_SIZE(rcp_def_keymap); i++) {
		u16	keycode = rcp_def_keymap[i];
		if (keycode != KEY_UNKNOWN && keycode != KEY_RESERVED)
			set_bit(keycode, rcp_input_dev->keybit);
	}

	rcp_input_dev->id.bustype = BUS_VIRTUAL;
//	rcp_input_dev->id.vendor  = 0x1095;
//	rcp_input_dev->id.product = 0x8240;

	error = input_register_device(rcp_input_dev);
	if (error) {
		MHL_TX_DBG_ERR(dev_context, "Failed to register device\n");
		input_free_device(rcp_input_dev);
		return error;
	}
	ret = mhl_tx_create_timer(dev_context
				,timer_callback_T_press_mode_handler
				,dev_context
				,&dev_context->timer_T_press_mode
				);
	if (ret != 0) {
		MHL_TX_DBG_ERR(dev_context, "failed in created timer_T_press_mode!\n");
	} else {
		ret = mhl_tx_create_timer(dev_context
					,timer_callback_T_hold_maintain_handler
					,dev_context
					,&dev_context->timer_T_hold_maintain
					);
		if (ret != 0) {
			MHL_TX_DBG_ERR(dev_context, "failed to create timer_T_hold_maintain!\n");
		} else {
			MHL_TX_DBG_INFO(dev_context, "device created\n");
			dev_context->rcp_input_dev = rcp_input_dev;

			return 0;
		}
		mhl_tx_delete_timer(dev_context,&dev_context->timer_T_press_mode);
	}
	return -1;
}

void destroy_rcp_input_dev(struct mhl_dev_context *dev_context)
{
	if (dev_context->timer_T_press_mode) {
		mhl_tx_delete_timer(dev_context,&dev_context->timer_T_press_mode);
	}
	if (dev_context->timer_T_hold_maintain) {
		mhl_tx_delete_timer(dev_context,&dev_context->timer_T_hold_maintain);
	}
	if (dev_context->rcp_input_dev) {
		input_unregister_device(dev_context->rcp_input_dev);
		dev_context->rcp_input_dev = NULL;
	}
}

void rcp_input_dev_one_time_init(struct mhl_dev_context *dev_context)
{
	int i;
	for (i = 0; i < MHL_NUM_RCP_KEY_CODES; ++i) {
		rcp_def_keymap[i] = rcpSupportTable[i].map[0];
	}
}
