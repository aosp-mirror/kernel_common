#ifndef _LINUX_LP5521_HTC_H
#define _LINUX_LP5521_HTC_H

#define LED_I2C_NAME "LP5521-LED"

#define ENABLE_REGISTER 	0x00
#define OPRATION_REGISTER	0x01
#define R_PWM_CONTROL	 	0x02
#define G_PWM_CONTROL 		0x03
#define B_PWM_CONTROL 		0x04



#define I2C_WRITE_RETRY_TIMES		2
#define LED_I2C_WRITE_BLOCK_SIZE	80

struct led_i2c_config {
	const char *name;
};

struct led_i2c_platform_data {
	struct led_i2c_config *led_config;
	int num_leds;
	int ena_gpio;
	int ena_gpio_io_ext;
	int tri_gpio;
	int button_lux;
};



void led_behavior(struct i2c_client *client, int val);
void lp5521_led_current_set_for_key(int brightness_key);

#endif 

