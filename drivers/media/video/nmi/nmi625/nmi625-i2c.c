/*****************************************************************************
 Copyright(c) 2010 NMI Inc. All Rights Reserved
 
 File name : nmi625-i2c.c
 
 Description :  Generic I2C driver for NM625
 
 History : 
 ----------------------------------------------------------------------
 2010/05/17 	ssw		initial
*******************************************************************************/
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include "nmi625-i2c.h"

#define HTC_ADD_FOR_LNA_CONTROL 0
#define HTC_ADD_PREVENT_REINIT_I2C 1

static struct i2c_driver nmi625_i2c_driver;
static struct i2c_client *nmi625_i2c_client = NULL;

int nmi625_init = 0;

#if HTC_ADD_PREVENT_REINIT_I2C
int nmi625_i2c_has_init = 0;
#endif 

struct nmi625_state{
	struct i2c_client	*client;	
};
struct nmi625_state *nmi625_state;

struct nmi625_platform_data {
        int _1v2_en;
        int rst;
        int en;
	int lna_2v85;
        int lna_en;
};
struct nmi625_platform_data *pdata;

int fm_ant_power(int on)
{
    int ret =0;

    if (on)
    {
        printk(KERN_INFO "[FM] %s: on \n", __func__);

        //turn on ONESEG_LNA_2V85
        ret = gpio_request(pdata->lna_2v85, "oneseg_lna_2v85");
        if (ret < 0) {
                pr_err("[FM] %s: gpio_request failed %d\n", __func__, ret);
                return ret;
        }
        ret = gpio_direction_output(pdata->lna_2v85, 1);
        if (ret < 0) {
                pr_err("[FM] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(pdata->lna_2v85);
                return ret;
        }

        //set ONESEG_LNA_EN enabled
        ret = gpio_request(pdata->lna_en, "oneseg_lna_en");
        if (ret < 0) {
                pr_err("[FM] %s: gpio_request failed %d\n", __func__, ret);
                return ret;
        }
        ret = gpio_direction_output(pdata->lna_en, 0);
        if (ret < 0) {
                pr_err("[FM] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(pdata->lna_en);
                return ret;
        }
    }else
    {
        /*Power OFF sequence*/
        printk(KERN_INFO "[FM] %s: off \n", __func__);

        //unset ONESEG_LNA_EN
        ret = gpio_request(pdata->lna_en, "oneseg_lna_en");
        if (ret < 0) {
                pr_err("[FM] %s: gpio_request failed %d\n", __func__, ret);
                return ret;
        }
        ret = gpio_direction_output(pdata->lna_en, 0);
        if (ret < 0) {
                pr_err("[FM] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(pdata->lna_en);
                return ret;
        }

        //turn off ONESEG_LNA_2V85
        ret = gpio_request(pdata->lna_2v85, "oneseg_lna_2v85");
        if (ret < 0) {
                pr_err("[FM] %s: gpio_request failed %d\n", __func__, ret);
                return ret;
        }
        ret = gpio_direction_output(pdata->lna_2v85, 0);
        if (ret < 0) {
                pr_err("[FM] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(pdata->lna_2v85);
                return ret;
        }
    }

    return ret;
}

int oneseg_power(int on)
{
    int ret =0;

    if (on)
    {
        printk(KERN_INFO "[1SEG] %s: on \n", __func__);

        //set 1seg tuner power (1.2v)
        ret = gpio_request(pdata->_1v2_en, "oneseg_1v2_en");
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_request failed %d\n", __func__, ret);
                return ret;
        }
        ret = gpio_direction_output(pdata->_1v2_en, 1);
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(pdata->_1v2_en);
                return ret;
        }

       //set ONESEG_EN to HIGH
	ret = gpio_request(pdata->en, "oneseg_en");
        if (ret < 0) {
		pr_err("[1SEG] %s: gpio_request failed %d\n", __func__, ret);
                return ret;
	}
        ret = gpio_direction_output(pdata->en, 1);
        if (ret < 0) {
        	pr_err("[1SEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(pdata->en);
                return ret;
	}

        //set ONESEG_RST to HIGH
        ret = gpio_request(pdata->rst, "oneseg_rst");
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_request failed %d\n", __func__, ret);
                return ret;
        }
        ret = gpio_direction_output(pdata->rst, 1);
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(pdata->rst);
                return ret;
        }

	msleep(10);

        //set ONESEG_RST to low
        ret = gpio_direction_output(pdata->rst, 0);
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(pdata->rst);
                return ret;
        }

        msleep(1);

        //set ONESEG_RST to HIGH
        ret = gpio_direction_output(pdata->rst, 1);
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(pdata->rst);
                return ret;
        }

	msleep(10);

        //turn on ONESEG_LNA_2V85
        ret = gpio_request(pdata->lna_2v85, "oneseg_lna_2v85");
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_request failed %d\n", __func__, ret);
                return ret;
        }
        ret = gpio_direction_output(pdata->lna_2v85, 1);
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(pdata->lna_2v85);
                return ret;
        }

        //set ONESEG_LNA_EN enabled
        ret = gpio_request(pdata->lna_en, "oneseg_lna_en");
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_request failed %d\n", __func__, ret);
                return ret;
        }
        ret = gpio_direction_output(pdata->lna_en, 1);
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(pdata->lna_en);
                return ret;
        }
    }else
    {
        /*Power OFF sequence*/
        printk(KERN_INFO "[1SEG] %s: off \n", __func__);

        //unset ONESEG_LNA_EN
        ret = gpio_request(pdata->lna_en, "oneseg_lna_en");
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_request failed %d\n", __func__, ret);
                return ret;
        }
        ret = gpio_direction_output(pdata->lna_en, 0);
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(pdata->lna_en);
                return ret;
        }

        //turn off ONESEG_LNA_2V85
        ret = gpio_request(pdata->lna_2v85, "oneseg_lna_2v85");
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_request failed %d\n", __func__, ret);
                return ret;
        }
        ret = gpio_direction_output(pdata->lna_2v85, 0);
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(pdata->lna_2v85);
                return ret;
        }

        //unset ONESEG_EN
        ret = gpio_request(pdata->en, "oneseg_en");
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_request failed %d\n", __func__, ret);
                return ret;
        }
        ret = gpio_direction_output(pdata->en, 0);
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(pdata->en);
                return ret;
        }

        msleep(10);

        //unset tuner power (1.2v)
        ret = gpio_request(pdata->_1v2_en, "oneseg_1v2_en");
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_request failed %d\n", __func__, ret);
                return ret;
        }
        ret = gpio_direction_output(pdata->_1v2_en, 0);
        if (ret < 0) {
                pr_err("[1SEG] %s: gpio_direction_output failed %d\n", __func__, ret);
                gpio_free(pdata->_1v2_en);
                return ret;
        }
    }

    return ret;
}

int nmi625_i2c_init(void)
{
	int res = 0;

	printk("nmi625_i2c_init ENTER...\n");
	
#if HTC_ADD_PREVENT_REINIT_I2C	
	if (nmi625_i2c_has_init){
		printk("nmi625_i2c already inited, return...\n");
		return res;
	}
#endif
/*	
	nmi625_i2c_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	
	if(nmi625_i2c_client== NULL) {
		printk("nmi625_i2c_client NULL...\n");
		return -ENOMEM;
	}

	res=i2c_add_driver(&nmi625_i2c_driver);
*/  
#if HTC_ADD_PREVENT_REINIT_I2C	
	if (NULL == nmi625_i2c_client->adapter) {
		printk("nmi625_i2c_client->adapter == NULL !!\n");
	}
	else {
		printk("nmi625_i2c_client->adapter (%08lu)\n", (unsigned long)nmi625_i2c_client->adapter);
	}
#endif
	
	if (res)
		pr_err("%s: Can't add nmi625 i2c drv, res=%d\n", __func__, res);
	else {
		
#if HTC_ADD_PREVENT_REINIT_I2C			
		nmi625_i2c_has_init = 1;
#endif

		pr_info("%s: Added nmi625 i2c drv\n", __func__);
	}
	
	return res;
}


int nmi625_i2c_deinit(void)
{
	printk("nmi625_i2c_deinit ENTER...\n");

//	i2c_del_driver(&nmi625_i2c_driver);

#if HTC_ADD_PREVENT_REINIT_I2C
    nmi625_i2c_has_init = 0;
#endif

	return 0;
}


int nmi625_i2c_read(void *hDevice, unsigned short addr, unsigned char *data, unsigned short length) 
{
	int res;
	struct i2c_msg rmsg;

	rmsg.addr = addr;
	rmsg.flags = I2C_M_RD;
	rmsg.len = length;
	rmsg.buf = data;
		
	res = i2c_transfer(nmi625_i2c_client->adapter, &rmsg, 1);

//	printk("nmi625_i2c_read nmi625_i2c_client->addr (%08lu)\n", (unsigned long)rmsg.addr);
//	printk("nmi625_i2c_client->adapter (%08lu)\n", (unsigned long)nmi625_i2c_client->adapter);
//	printk("nmi625_i2c_read (%02x)(%02x)(%02x)(%02x)\n", data[0],data[1],data[2],data[3]);

	return 0;
}

int nmi625_i2c_write(void *hDevice, unsigned short addr, unsigned char *data, unsigned short length)
{
	int res;
	struct i2c_msg wmsg;

	if(length+1>I2C_MAX_SEND_LENGTH)
	{
		printk(".......error %s", __FUNCTION__);
		return -ENODEV;
	}
	wmsg.addr = addr;
	wmsg.flags = I2C_M_WR;
	wmsg.len = length;
	wmsg.buf = data;
	
	res = i2c_transfer(nmi625_i2c_client->adapter, &wmsg, 1);

//	printk("nmi625_i2c_write nmi625_i2c_client->addr (%08lu)\n", (unsigned long)wmsg.addr);
//	printk("nmi625_i2c_client->adapter (%08lu)\n", (unsigned long)nmi625_i2c_client->adapter);
//	printk("nmi625_i2c_write (%02x)(%02x)(%02x)(%02x)(%02x)(%02x)\n", data[0],data[1],data[2],data[3],data[4],data[5]);

	return 0;
}

void nmi625_i2c_read_chip_id(void)
{
	u8 cmd[16] = {0,};
	u8 cmd1[16] = {0,};
	struct i2c_msg rmsg;
	struct i2c_msg wmsg;

	cmd[0] = 0x80;
	cmd[1] = 0x00;
	cmd[2] = 0x64;
	cmd[3] = 0x00;
	cmd[4] = 0x00;
	cmd[5] = 0x04;

	wmsg.addr = 0x61;
	wmsg.flags = I2C_M_WR;
	wmsg.len = 6;
	wmsg.buf = cmd;

	printk("nmi625_i2c_read_chip_id()\n");
	i2c_transfer(nmi625_i2c_client->adapter, &wmsg, 1);

	printk("nmi625_i2c_client->addr (%08lu)\n", (unsigned long)nmi625_i2c_client->addr);

	rmsg.addr = 0x61;
	rmsg.flags = I2C_M_RD;
	rmsg.len = 4;
	rmsg.buf = cmd1;
	printk("nmi625_i2c_client->adapter (%08lu)\n", (unsigned long)nmi625_i2c_client->adapter);
	i2c_transfer(nmi625_i2c_client->adapter, &rmsg, 1);
	printk("Nmi 325 Chip Id (%02x)(%02x)(%02x)(%02x)\n", cmd1[0],cmd1[1],cmd1[2],cmd1[3]);
}

static int nmi625_i2c_remove(struct i2c_client *client)
{
	struct nmi625_state *nmi625 = i2c_get_clientdata(client);

	kfree(pdata);
	kfree(nmi625);
	return 0;
}

static int nmi625_parse_dt(struct device *dev, struct nmi625_platform_data *pdata)
{
        struct property *prop;
        struct device_node *dt = dev->of_node;

        prop = of_find_property(dt, "nmi625,nmi625_1v2_en", NULL);
        if (prop) {
                pdata->_1v2_en = of_get_named_gpio(dt, "nmi625,nmi625_1v2_en", 0);
        }

        prop = of_find_property(dt, "nmi625,nmi625_en", NULL);
        if (prop) {
                pdata->en = of_get_named_gpio(dt, "nmi625,nmi625_en", 0);
        }

        prop = of_find_property(dt, "nmi625,nmi625_rst", NULL);
        if (prop) {
                pdata->rst = of_get_named_gpio(dt, "nmi625,nmi625_rst", 0);
        }

        prop = of_find_property(dt, "nmi625,nmi625_lna_2v85", NULL);
        if (prop) {
                pdata->lna_2v85 = of_get_named_gpio(dt, "nmi625,nmi625_lna_2v85", 0);
        }

        prop = of_find_property(dt, "nmi625,nmi625_lna_en", NULL);
        if (prop) {
                pdata->lna_en = of_get_named_gpio(dt, "nmi625,nmi625_lna_en", 0);
        }

        return 0;
}

int read_gpio_from_dt(struct device *dev)
{
        int ret = 0;

        printk("%s+++\n", __func__);
        pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
        if (pdata == NULL)
                ret = -ENOMEM;
        ret = nmi625_parse_dt(dev, pdata);
	return ret;
}


static int nmi625_i2c_probe(struct i2c_client *client,  const struct i2c_device_id *id)
{
	struct nmi625_state *nmi625;
	struct device *dev = &client->dev;

	nmi625 = kzalloc(sizeof(struct nmi625_state), GFP_KERNEL);
	if (nmi625 == NULL) {		
		printk("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	nmi625->client = client;
	i2c_set_clientdata(client, nmi625);
	
	/* rest of the initialisation goes here.*/
	
	printk("nmi625 attach success!!!\n");

	nmi625_i2c_client = client;

	nmi625_init = 1;

        read_gpio_from_dt(dev);
	
	return 0;
}


static const struct i2c_device_id nmi625_device_id[] = {
	{"nmi625", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, nmi625_device_id);

#if HTC_ADD_FOR_LNA_CONTROL
extern int oneseg_set_lna(int enable);

static int nmi625_i2c_suspend(struct device *dev)
{
	printk("nmi625_i2c_suspend!!!\n");
	oneseg_set_lna(0);
	return 0;
}

static int nmi625_i2c_resume(struct device *dev)
{
    printk("nmi625_i2c_resume!!!\n");
    oneseg_set_lna(1);
    return 0;
}

static struct dev_pm_ops nmi625_pm_ops = 
{   .suspend = nmi625_i2c_suspend,
    .resume  = nmi625_i2c_resume,
};
#endif //HTC_ADD_FOR_LNA_CONTROL

static const struct of_device_id nmi625_mttable[] = {
        { .compatible = "nmi625"},
        { },
};

static struct i2c_driver nmi625_i2c_driver = {
	.driver = {
		.name = "nmi625",
		.owner = THIS_MODULE,
		.of_match_table = nmi625_mttable,
#if HTC_ADD_FOR_LNA_CONTROL		
		.pm = &nmi625_pm_ops,
#endif		
	},
	.probe	= nmi625_i2c_probe,
	.remove	= __devexit_p(nmi625_i2c_remove),
	.id_table	= nmi625_device_id,
};

module_i2c_driver(nmi625_i2c_driver);
