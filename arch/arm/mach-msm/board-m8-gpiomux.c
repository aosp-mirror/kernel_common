/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/ioport.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <mach/socinfo.h>

#define m8_UHL_PID 305

static struct gpiomux_setting ap2mdm_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting mdm2ap_status_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};
#if 0
static struct gpiomux_setting mdm2ap_errfatal_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};
#endif
static struct gpiomux_setting mdm2ap_pblrdy = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};


static struct gpiomux_setting ap2mdm_soft_reset_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting ap2mdm_wakeup = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct msm_gpiomux_config mdm_configs[] __initdata = {
	
	{
		.gpio = 105,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_cfg,
		}
	},
	
	{
		.gpio = 46,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mdm2ap_status_cfg,
		}
	},
	
#if 0
	{
		.gpio = 82,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mdm2ap_errfatal_cfg,
		}
	},
#endif
	
	{
		.gpio = 106,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_cfg,
		}
	},
	
	{
		.gpio = 24,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_soft_reset_cfg,
		}
	},
	
	{
		.gpio = 104,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ap2mdm_wakeup,
		}
	},
	
	{
		.gpio = 80,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mdm2ap_pblrdy,
		}
	},
};
#if 0
static struct gpiomux_setting gpio_uart_config = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};
#endif
static struct gpiomux_setting slimbus = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_KEEPER,
};

#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
static struct gpiomux_setting gpio_spi_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_12MA,
	.pull = GPIOMUX_PULL_NONE,
};
#endif

static struct gpiomux_setting wcnss_5wire_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting wcnss_5wire_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv  = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting gpio_i2c_config = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting lcd_te_act_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting lcd_te_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting lcd_id_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting lcd_id_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting atmel_resout_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting atmel_resout_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting atmel_int_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting atmel_int_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting taiko_reset = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting taiko_int = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct msm_gpiomux_config msm_touch_configs[] __initdata = {
	{
		.gpio      = 60,		
		.settings = {
			[GPIOMUX_ACTIVE] = &atmel_resout_act_cfg,
			[GPIOMUX_SUSPENDED] = &atmel_resout_sus_cfg,
		},
	},
	{
		.gpio      = 61,		
		.settings = {
			[GPIOMUX_ACTIVE] = &atmel_int_act_cfg,
			[GPIOMUX_SUSPENDED] = &atmel_int_sus_cfg,
		},
	},

};

#if 0	
static struct gpiomux_setting hsic_act_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_12MA,
	.pull = GPIOMUX_PULL_NONE,
};
#endif

static struct gpiomux_setting hsic_resume_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting hsic_resume_susp_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting cir_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct msm_gpiomux_config msm_cir_configs[] = {
	{
		.gpio = 0,               
		.settings = {
			[GPIOMUX_ACTIVE] = &cir_cfg,
		},
	},
	{
		.gpio = 1,               
		.settings = {
			[GPIOMUX_ACTIVE] = &cir_cfg,
		},
	},
};

static struct gpiomux_setting felica_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct msm_gpiomux_config msm_felica_configs[] = {
	{
		.gpio = 85,               
		.settings = {
			[GPIOMUX_ACTIVE] = &felica_cfg,
		},
	},
	{
		.gpio = 86,               
		.settings = {
			[GPIOMUX_ACTIVE] = &felica_cfg,
		},
	},
};

static struct msm_gpiomux_config msm_hsic_configs[] = {
#if 0	
	{
		.gpio = 144,               
		.settings = {
			[GPIOMUX_ACTIVE] = &hsic_act_cfg,
			[GPIOMUX_SUSPENDED] = &hsic_sus_cfg,
		},
	},
#endif
#if 0
	{
		.gpio = 145,               
		.settings = {
			[GPIOMUX_ACTIVE] = &hsic_act_cfg,
			[GPIOMUX_SUSPENDED] = &hsic_sus_cfg,
		},
	},
#endif
	{
		.gpio = 80,
		.settings = {
			[GPIOMUX_ACTIVE] = &hsic_resume_act_cfg,
			[GPIOMUX_SUSPENDED] = &hsic_resume_susp_cfg,
		},
	},
};

static struct gpiomux_setting aud_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting aud_active_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting  qua_mi2s_act_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting  com_aud_fun1_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting  com_aud_gpio_lo_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct msm_gpiomux_config msm_aud_configs[] __initdata = {
	{
		.gpio = 62,    
		.settings = {
			[GPIOMUX_SUSPENDED] = &aud_suspend_cfg,
			[GPIOMUX_ACTIVE]    = &aud_active_cfg,
		},
	},
	{
		.gpio = 57,    
		.settings = {
			[GPIOMUX_SUSPENDED] = &aud_suspend_cfg,
			[GPIOMUX_ACTIVE]    = &aud_active_cfg,
		},
	},
	{
		.gpio = 103,  
		.settings = {
			[GPIOMUX_SUSPENDED] = &aud_suspend_cfg,
			[GPIOMUX_ACTIVE]    = &aud_active_cfg,
		},
	},
	{
		.gpio = 69, 
		.settings = {
			[GPIOMUX_SUSPENDED] = &com_aud_gpio_lo_cfg,
			[GPIOMUX_ACTIVE] = &com_aud_fun1_cfg,
		},
	},
	{
		.gpio = 54, 
		.settings = {
			[GPIOMUX_SUSPENDED] = &com_aud_gpio_lo_cfg,
			[GPIOMUX_ACTIVE] = &com_aud_gpio_lo_cfg,
		},
	},
};

static struct msm_gpiomux_config msm_aud_spk_china_configs[] __initdata = {
	{
		.gpio	= 58, 
		.settings = {
			[GPIOMUX_SUSPENDED] = &com_aud_gpio_lo_cfg,
			[GPIOMUX_ACTIVE] = &qua_mi2s_act_cfg,
		},
	},
	{
		.gpio	= 59, 
			.settings = {
			[GPIOMUX_SUSPENDED] = &com_aud_gpio_lo_cfg,
			[GPIOMUX_ACTIVE] = &qua_mi2s_act_cfg,
		},
	},
	{
		.gpio = 60, 
		.settings = {
			[GPIOMUX_SUSPENDED] = &com_aud_gpio_lo_cfg,
			[GPIOMUX_ACTIVE] = &qua_mi2s_act_cfg,
		},
	},
	{
		.gpio = 61, 
		.settings = {
			[GPIOMUX_SUSPENDED] = &com_aud_gpio_lo_cfg,
			[GPIOMUX_ACTIVE] = &qua_mi2s_act_cfg,
		},
	},
};

static struct gpiomux_setting mhl_intr_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting hdmi_suspend_1_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting hdmi_suspend_2_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting hdmi_active_1_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting hdmi_active_2_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct msm_gpiomux_config msm_mhl_configs[] __initdata = {
	{
		
		.gpio = 74,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mhl_intr_cfg,
			[GPIOMUX_ACTIVE]    = &mhl_intr_cfg,
		},
	},
};


static struct msm_gpiomux_config msm_hdmi_configs[] __initdata = {
	{
		.gpio = 32,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_1_cfg,
		},
	},
	{
		.gpio = 33,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_1_cfg,
		},
	},
	{
		.gpio = 34,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_2_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_2_cfg,
		},
	},
};

static struct gpiomux_setting gpio_uart7_active_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gpio_uart7_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct msm_gpiomux_config msm_blsp2_uart7_configs[] __initdata = {
	{
		.gpio	= 41,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_uart7_active_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_uart7_suspend_cfg,
		},
	},
	{
		.gpio	= 42,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_uart7_active_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_uart7_suspend_cfg,
		},
	},
	{
		.gpio	= 43,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_uart7_active_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_uart7_suspend_cfg,
		},
	},
	{
		.gpio	= 44,	
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_uart7_active_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_uart7_suspend_cfg,
		},
	},
};

static struct msm_gpiomux_config msm_lcd_configs[] __initdata = {
	{
		.gpio = 12,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_te_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_te_sus_cfg,
		},
	},
	{
		.gpio = 55,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_id_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_id_sus_cfg,
		},
	},
	{
		.gpio = 56,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_id_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_id_sus_cfg,
		},
	},
};

static struct msm_gpiomux_config msm_blsp_configs[] __initdata = {
#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
	{
		.gpio      = 0,		
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_spi_config,
		},
	},
	{
		.gpio      = 1,		
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_spi_config,
		},
	},
#if 0	
	{
		.gpio      = 3,		
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_spi_config,
		},
	},
#endif
#if 0	
	{
		.gpio      = 9,		
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_spi_cs2_config,
		},
	},
	{
		.gpio      = 8,		
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_spi_cs1_config,
		},
	},
#endif
#endif
	{
		.gpio      = 6,		
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio      = 7,		
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},

        {
                .gpio      = 10,        
                .settings = {
                        [GPIOMUX_ACTIVE] = &gpio_i2c_config,
                        [GPIOMUX_SUSPENDED] = &gpio_i2c_config,
                },
        },
        {
                .gpio      = 11,        
                .settings = {
                        [GPIOMUX_ACTIVE] = &gpio_i2c_config,
                        [GPIOMUX_SUSPENDED] = &gpio_i2c_config,
                },
        },
#if 0
        {
                .gpio      = 21,        
                .settings = {
                        [GPIOMUX_ACTIVE] = &gpio_i2c_config,
                        [GPIOMUX_SUSPENDED] = &gpio_i2c_config,
                },
        },
        {
                .gpio      = 22,        
                .settings = {
                        [GPIOMUX_ACTIVE] = &gpio_i2c_config,
                        [GPIOMUX_SUSPENDED] = &gpio_i2c_config,
                },
        },
#endif
        {
                .gpio      = 29,        
                .settings = {
                        [GPIOMUX_ACTIVE] = &gpio_i2c_config,
                        [GPIOMUX_SUSPENDED] = &gpio_i2c_config,
                },
        },
        {
                .gpio      = 30,        
                .settings = {
                        [GPIOMUX_ACTIVE] = &gpio_i2c_config,
                        [GPIOMUX_SUSPENDED] = &gpio_i2c_config,
                },
        },

#if 0
	{
		.gpio      = 83,		
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio      = 84,		
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio      = 4,			
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_uart_config,
		},
	},
	{
		.gpio      = 5,			
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_uart_config,
		},
	},
	
	{
		.gpio      = 53,		
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_spi_config,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
	
	{
		.gpio      = 54,		
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_spi_config,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},

	{
		.gpio      = 81,		
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_epm_config,
		},
	},
#endif
};

static struct msm_gpiomux_config msm8974_slimbus_config[] __initdata = {
	{
		.gpio	= 70,		
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},
	{
		.gpio	= 71,		
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},
};

static struct gpiomux_setting cam_settings[] = {
	{
		.func = GPIOMUX_FUNC_GPIO, 
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_DOWN,
		.dir = GPIOMUX_IN,
	},

	{
		.func = GPIOMUX_FUNC_1, 
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, 
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_LOW,
	},

	{
		.func = GPIOMUX_FUNC_1, 
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_2, 
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, 
		.drv = GPIOMUX_DRV_4MA,
		.pull = GPIOMUX_PULL_DOWN,
		.dir = GPIOMUX_IN,
	},

	{
		.func = GPIOMUX_FUNC_2, 
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, 
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_IN,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, 
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
		.dir = GPIOMUX_IN,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, 
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_HIGH,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, 
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_LOW,
	},

	{
		.func = GPIOMUX_FUNC_1, 
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_DOWN,
		.dir = GPIOMUX_IN,
	},

	{
		.func = GPIOMUX_FUNC_1, 
		.drv = GPIOMUX_DRV_6MA,
		.pull = GPIOMUX_PULL_DOWN,
		.dir = GPIOMUX_IN,
	},

	{
		.func = GPIOMUX_FUNC_1, 
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_1,  
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_KEEPER,
	},

	{
		.func = GPIOMUX_FUNC_1, 
		.drv = GPIOMUX_DRV_16MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, 
		.drv = GPIOMUX_DRV_16MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_LOW,
	},

	{
		.func = GPIOMUX_FUNC_1, 
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, 
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
		.dir = GPIOMUX_IN,
	},

	{
		.func = GPIOMUX_FUNC_1, 
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_LOW,
	},

	{
		.func = GPIOMUX_FUNC_1, 
		.drv = GPIOMUX_DRV_4MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, 
		.drv = GPIOMUX_DRV_4MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_LOW,
	},
	{
		.func = GPIOMUX_FUNC_2, 
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},
	{
		.func = GPIOMUX_FUNC_GPIO, 
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_UP,
		.dir = GPIOMUX_IN,
	},
};

static struct gpiomux_setting nfc_irq_active_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting nfc_irq_sleep_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config nfc_irq_config __initdata = {
		.gpio = 144,
		.settings = {
		[GPIOMUX_ACTIVE]    = &nfc_irq_active_config,
		[GPIOMUX_SUSPENDED] = &nfc_irq_sleep_config,
	},
};

static struct gpiomux_setting sd_card_det_active_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting sd_card_det_sleep_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config sd_card_det __initdata = {
	.gpio = 62,
	.settings = {
		[GPIOMUX_ACTIVE]    = &sd_card_det_active_config,
		[GPIOMUX_SUSPENDED] = &sd_card_det_sleep_config,
	},
};

static struct msm_gpiomux_config msm_sensor_configs[] __initdata = {
	{
		.gpio = 15, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[20], 
			[GPIOMUX_SUSPENDED] = &cam_settings[21], 
		},
	},
	{
		.gpio = 16, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[20], 
			[GPIOMUX_SUSPENDED] = &cam_settings[21], 
		},
	},
	{
		.gpio = 17, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[20], 
			[GPIOMUX_SUSPENDED] = &cam_settings[21], 
		},
	},
	{
		.gpio = 19, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[17], 
			[GPIOMUX_SUSPENDED] = &cam_settings[18], 
		},
	},
	{
		.gpio = 20, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[17], 
			[GPIOMUX_SUSPENDED] = &cam_settings[18], 
		},
	},
	{
		.gpio = 21, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[17], 
			[GPIOMUX_SUSPENDED] = &cam_settings[18], 
		},
	},
	{
		.gpio = 22, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[17], 
			[GPIOMUX_SUSPENDED] = &cam_settings[18], 
		},
	},
	{
		.gpio = 145, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[10], 
			[GPIOMUX_SUSPENDED] = &cam_settings[10], 
		},
	},
	{
		.gpio = 31, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[10], 
			[GPIOMUX_SUSPENDED] = &cam_settings[10], 
		},
	},
};

static struct msm_gpiomux_config msm_sensor_configs_non_common[] __initdata = {
	{
		.gpio = 85, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[7], 
			[GPIOMUX_SUSPENDED] = &cam_settings[8], 
		},
	},
};

static struct msm_gpiomux_config msm_sensor_configs_china_sku[] __initdata = {
	{
		.gpio = 132, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[23], 
			[GPIOMUX_SUSPENDED] = &cam_settings[10], 
		},
	},
};

static struct msm_gpiomux_config msm_sensor_configs_non_china_sku[] __initdata = {
	{
		.gpio = 117, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[23], 
			[GPIOMUX_SUSPENDED] = &cam_settings[10], 
		},
	},
};

static struct msm_gpiomux_config msm_blsp_rawchip_spi_configs[] __initdata = {
	
	{
		.gpio	   =  81, 
		.settings = {
			[GPIOMUX_ACTIVE] = &cam_settings[22], 
			[GPIOMUX_SUSPENDED] = &cam_settings[10], 
		},
	},
	{
		.gpio	   = 82, 
		.settings = {
			[GPIOMUX_ACTIVE] = &cam_settings[22], 
			[GPIOMUX_SUSPENDED] = &cam_settings[10], 
		},
	},
	{
		.gpio	   = 83, 
		.settings = {
			[GPIOMUX_ACTIVE] = &cam_settings[17], 
			[GPIOMUX_SUSPENDED] = &cam_settings[10], 
		},
	},
	{
		.gpio	   = 84, 
		.settings = {
			[GPIOMUX_ACTIVE] = &cam_settings[17], 
			[GPIOMUX_SUSPENDED] = &cam_settings[10], 
		},
	},
	{
		.gpio = 79, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[7], 
			[GPIOMUX_SUSPENDED] = &cam_settings[8], 
		},
	},
	{
		.gpio = 80, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[7], 
			[GPIOMUX_SUSPENDED] = &cam_settings[8], 
		},
	},
};

static struct msm_gpiomux_config wcnss_5wire_interface[] = {
	{
		.gpio = 36,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 37,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 38,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 39,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 40,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
};

static struct msm_gpiomux_config msm_taiko_config[] __initdata = {
	{
		.gpio	= 63,		
		.settings = {
			[GPIOMUX_SUSPENDED] = &taiko_reset,
		},
	},
	{
		.gpio	= 72,		
		.settings = {
			[GPIOMUX_SUSPENDED] = &taiko_int,
		},
	},
};

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct gpiomux_setting sdc4_clk_actv_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting sdc4_cmd_data_0_3_actv_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdc4_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting sdc4_data_1_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct msm_gpiomux_config msm8974_sdc4_configs[] __initdata = {
	{
		
		.gpio      = 92,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdc4_cmd_data_0_3_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdc4_suspend_cfg,
		},
	},
	{
		
		.gpio      = 94,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdc4_cmd_data_0_3_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdc4_suspend_cfg,
		},
	},
#if 0
	
	{
		
		.gpio      = 95,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdc4_cmd_data_0_3_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdc4_data_1_suspend_cfg,
		},
	},
	{
		
		.gpio      = 96,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdc4_cmd_data_0_3_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdc4_suspend_cfg,
		},
	},
	{	
		
		.gpio      = 91,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdc4_cmd_data_0_3_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdc4_suspend_cfg,
		},
	},

	{
		
		.gpio      = 93,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdc4_clk_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdc4_suspend_cfg,
		},
	},
#endif
};

static void msm_gpiomux_sdc4_install(void)
{
	msm_gpiomux_install(msm8974_sdc4_configs,
			    ARRAY_SIZE(msm8974_sdc4_configs));
}
#else
static void msm_gpiomux_sdc4_install(void) {}
#endif 

void __init msm_htc_8974_init_gpiomux(void)
{
	int rc;

	rc = msm_gpiomux_init_dt();
	if (rc) {
		pr_err("%s failed %d\n", __func__, rc);
		return;
	}

	msm_gpiomux_install(msm_blsp_configs, ARRAY_SIZE(msm_blsp_configs));
	msm_gpiomux_install(msm_blsp2_uart7_configs,
			 ARRAY_SIZE(msm_blsp2_uart7_configs));
	msm_gpiomux_install(wcnss_5wire_interface,
				ARRAY_SIZE(wcnss_5wire_interface));

	msm_gpiomux_install(msm8974_slimbus_config,
			ARRAY_SIZE(msm8974_slimbus_config));

	msm_gpiomux_install(msm_touch_configs, ARRAY_SIZE(msm_touch_configs));

	msm_gpiomux_install(msm_sensor_configs, ARRAY_SIZE(msm_sensor_configs));
	msm_gpiomux_install(msm_sensor_configs_non_common, ARRAY_SIZE(msm_sensor_configs_non_common));

    
    if (of_machine_pid() == m8_UHL_PID)
        msm_gpiomux_install(msm_sensor_configs_china_sku, ARRAY_SIZE(msm_sensor_configs_china_sku));
    else
        msm_gpiomux_install(msm_sensor_configs_non_china_sku, ARRAY_SIZE(msm_sensor_configs_non_china_sku));

	msm_gpiomux_install(&nfc_irq_config, 1);

	msm_gpiomux_install(&sd_card_det, 1);

	msm_gpiomux_sdc4_install();

	msm_gpiomux_install(msm_taiko_config, ARRAY_SIZE(msm_taiko_config));

	msm_gpiomux_install(msm_hsic_configs, ARRAY_SIZE(msm_hsic_configs));

	msm_gpiomux_install(msm_hdmi_configs, ARRAY_SIZE(msm_hdmi_configs));
	msm_gpiomux_install(msm_mhl_configs, ARRAY_SIZE(msm_mhl_configs));

	msm_gpiomux_install_nowrite(msm_lcd_configs,
			ARRAY_SIZE(msm_lcd_configs));

	msm_gpiomux_install(msm_aud_configs,
				 ARRAY_SIZE(msm_aud_configs));

	msm_gpiomux_install(msm_aud_spk_china_configs, ARRAY_SIZE(msm_aud_spk_china_configs));

	msm_gpiomux_install(msm_cir_configs, ARRAY_SIZE(msm_cir_configs));

	if (of_machine_pid() == 268 || of_board_is_m8wlj())
		msm_gpiomux_install(msm_felica_configs, ARRAY_SIZE(msm_felica_configs));

	if (socinfo_get_platform_subtype() == PLATFORM_SUBTYPE_MDM)
		msm_gpiomux_install(mdm_configs,
			ARRAY_SIZE(mdm_configs));

	msm_gpiomux_install(msm_blsp_rawchip_spi_configs, ARRAY_SIZE(msm_blsp_rawchip_spi_configs));

}
