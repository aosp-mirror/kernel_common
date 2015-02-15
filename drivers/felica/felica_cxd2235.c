#ifdef CONFIG_FELICA_CXD2235_DD

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/termios.h>
#include <linux/serial_core.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <mach/board.h>
#include <linux/felica_cxd2235.h>
#include <mach/devices_cmdline.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

/******************************************************************************
 * global variable
 ******************************************************************************/

static struct class *felica_class;
static struct class *snfc_class;

static int felica_int_pin;
static int felica_int_irq;

static int snfc_intu_pin;
static int snfc_intu_irq;

/* storages for communicate to netlink */
static int gfa_open_cnt;
static int gfa_pid;
static int gfa_connect_flag;
static struct sock *gfanl_sk = NULL;
static char gfa_send_str[FELICA_NL_MSG_SIZE];
static char gfa_rcv_str[FELICA_NL_MSG_SIZE];
static int gfa_wait_flag;

/* I2C power control information storage for CEN terminal */
static unsigned int gfelica_cen_opencnt;

/* FeliCaLock status storage */
static char gfelica_cen_status;

/* R/W functions availability information storage */
static char gfelica_rw_status;

/* IRQ data storage for INT terminal monitoring */
struct felica_int_irqdata
{
	struct delayed_work	work;
	wait_queue_head_t	read_wait;
	int					irq_done;
	int					open_flag;
};
static struct felica_int_irqdata	gint_irq;
static struct felica_int_irqdata	*pgint_irq = &gint_irq;

/* IRQ data storage for INTU terminal monitoring */
struct snfc_intu_irqdata
{
	struct delayed_work	work;
	wait_queue_head_t	read_wait;
	int					irq_done;
	int					open_flag;
};
static struct snfc_intu_irqdata	gintu_irq;
static struct snfc_intu_irqdata	*pgintu_irq = &gintu_irq;

struct snfc_auto_polling
{
	int			auto_polling_done;
	wait_queue_head_t	auto_polling_wait;
	struct delayed_work 	snfc_auto_polling_work;
};

/* storages for access restriction */
static uid_t gmfc_uid  = -1;
static uid_t gmfl_uid  = -1;
static uid_t grwm_uid  = -1;
static uid_t gdiag_uid = -1;
static uid_t gdtl_uid  = -1;

/* package name's storage for access restriction */
static char gdiag_name[DIAG_NAME_MAXSIZE+1];

static struct felica_platform_data *felica_pdata;

static char gfelica_uart_status;
static char gsnfc_uart_status;

static struct mutex uart_mutex;

static DECLARE_WAIT_QUEUE_HEAD(wait_snfc_uart_release);

void set_felica_uart_status(int status) {
	gfelica_uart_status = status;
}

int get_felica_uart_status(void) {
	return gfelica_uart_status;
}

void set_snfc_uart_status(int status) {
	gsnfc_uart_status = status;
	wake_up_interruptible(&wait_snfc_uart_release);
}

int get_snfc_uart_status(void) {
	return gsnfc_uart_status;
}

#ifdef CONFIG_OF
static int fel_hsel_gpio;
static int fel_pon_gpio;
static int fel_cenz_gpio;
static int felica_cen_gpio;
static int felica_lock_gpio;
static int fel_hw_rst_gpio;
static int fel_int_gpio;
static int fel_intu_gpio;
static int fel_rfs_gpio;

static void felica_pon_gpio_func(int rwtype, int wvalue, int *rvalue)
{
	if (rwtype == GPIOWRITE) {
		/*printk(KERN_DEBUG "[FELICA_DD] %s set pon[%x]\n", __func__, wvalue);*/
		gpio_set_value(fel_pon_gpio, wvalue);
	}
	else if (rwtype == GPIOREAD) {
		*rvalue = gpio_get_value(fel_pon_gpio);
		/*printk(KERN_DEBUG "[FELICA_DD] %s pon[%x]\n", __func__, *rvalue);*/
	}

	return;
}

static void felica_cen_dtyp_d_func(int rwtype, int wvalue, int *rvalue)
{
	if (rwtype == GPIOWRITE) {
		gpio_set_value(felica_cen_gpio, wvalue);
	}
	else if (rwtype == GPIOREAD) {
		*rvalue = gpio_get_value(felica_cen_gpio);
	}

	return;
}

static void felica_cen_dtyp_cp_func(int rwtype, int wvalue, int *rvalue)
{
	if (rwtype == GPIOWRITE) {
		gpio_set_value(felica_lock_gpio, wvalue);
	}
	else if (rwtype == GPIOREAD) {
		*rvalue = gpio_get_value(felica_lock_gpio);
	}

	return;
}

static void felica_cen_gpio_func(int rwtype, int wvalue, int *rvalue)
{
	if (rwtype == GPIOWRITE) {
		printk(KERN_INFO "[FELICA_DD] %s set cen[%x]\n", __func__, wvalue);
		gpio_set_value_cansleep(felica_lock_gpio, GPIO_VALUE_LOW);
		gpio_set_value_cansleep(felica_cen_gpio, wvalue);
		msleep(1);
		gpio_set_value_cansleep(felica_lock_gpio, GPIO_VALUE_HIGH);
		msleep(1);
		gpio_set_value_cansleep(felica_lock_gpio, GPIO_VALUE_LOW);
		msleep(1);
		gpio_set_value_cansleep(felica_cen_gpio, GPIO_VALUE_LOW);
	}
	else if (rwtype == GPIOREAD) {
		*rvalue = gpio_get_value(fel_cenz_gpio)?  FELICA_CEN_LOCK : FELICA_CEN_UNLOCK;
		/*printk(KERN_DEBUG "[FELICA_DD] %s cen[%x]\n", __func__, *rvalue);*/
	}

	return;
}

static void felica_rfs_gpio_func(int rwtype, int wvalue, int *rvalue)
{
	if (rwtype == GPIOREAD) {
		*rvalue = gpio_get_value(fel_rfs_gpio);
		/*printk(KERN_DEBUG "[FELICA_DD] %s rfs[%x]\n", __func__, *rvalue);*/
	}

	return;
}

static void felica_int_gpio_func(int rwtype, int wvalue, int *rvalue)
{
	if (rwtype == GPIOREAD) {
		*rvalue = gpio_get_value(fel_int_gpio);
		 printk(KERN_INFO "[FELICA_DD] %s int[%x]\n", __func__, *rvalue);
	}

	return;
}

static void felica_con_gpio_func(int rwtype, int wvalue, int *rvalue)
{
	return;
}

static void felica_hsel_gpio_func(int rwtype, int wvalue, int *rvalue)
{
#if 0
	unsigned ret;

	struct pm8xxx_mpp_config_data hsel_mpp = {
		.type	= PM8XXX_MPP_TYPE_D_OUTPUT,
		.level	= PM8921_MPP_DIG_LEVEL_S4,
	};
#endif
	if (rwtype == GPIOWRITE) {
		gpio_set_value(fel_hsel_gpio, wvalue);
#if 0
		if (wvalue) {
			/*printk(KERN_DEBUG "[FELICA_DD] %s set hsel HIGH\n", __func__);*/
			hsel_mpp.control = PM8XXX_MPP_DOUT_CTRL_HIGH;
			ret = pm8xxx_mpp_config(PM8921_MPP_PM_TO_SYS(10),
								&hsel_mpp);
			if (ret < 0)
				pr_err("%s:MPP8 configuration failed\n", __func__);
		} else {
			/*printk(KERN_DEBUG "[FELICA_DD] %s set hsel LOW\n", __func__);*/
			hsel_mpp.control = PM8XXX_MPP_DOUT_CTRL_LOW;
			ret = pm8xxx_mpp_config(PM8921_MPP_PM_TO_SYS(10),
								&hsel_mpp);
			if (ret < 0)
				pr_err("%s:MPP10 config failed\n", __func__);
		}
#endif
	}
}

static void felica_suspend(void)
{

	return;
}

static void felica_resume(void)
{

	return;
}


static void felica_setup_gpio(void)
{
	return;
}

#if 0
static struct felica_platform_data felica_data = {
	.int_irq = 0,
	.int_gpio = 0,
	.intu_irq = 0,
	.intu_gpio = 0,
	.setup_gpio = felica_setup_gpio,
	.sleep_gpio = felica_suspend,
	.wakeup_gpio = felica_resume,
	.pon_gpio_func = felica_pon_gpio_func,
	.cen_dtyp_d_func = felica_cen_dtyp_d_func,
	.cen_dtyp_cp_func = felica_cen_dtyp_cp_func,
	.cen_gpio_func = felica_cen_gpio_func,
	.rfs_gpio_func = felica_rfs_gpio_func,
	.int_gpio_func = felica_int_gpio_func,
	.con_gpio_func = felica_con_gpio_func,
	.hsel_gpio_func = felica_hsel_gpio_func,
};

static struct platform_device felica_device = {
	.name = "felica",
	.id = 0,
	.dev		= {
		.platform_data	= &felica_data,
	},
};

int __init m7wl_j_init_felica(void)
{
	printk(KERN_INFO "[FELICA_DD] %s\n", __func__);
	return platform_device_register(&m7wl_j_felica_device);
}
#endif
static int felica_parse_dt(struct device *dev, struct felica_platform_data *pdata)
{
	uint8_t i;
	int ret = 0;
	const char *parser_gpio[] = {"felica,fel_hsel_gpio", "felica,fel_pon_gpio", "felica,fel_cenz_gpio", "felica,felica_cen_gpio", "felica,felica_lock_gpio",
				"felica,fel_hw_rst_gpio", "felica,fel_int_gpio", "felica,fel_intu_gpio", "felica,fel_rfs_gpio"};
	int gpio[ARRAY_SIZE(parser_gpio)] = {0};
	struct device_node *dt = dev->of_node;

	FELICA_LOG_INFO("[FELICA_DD] %s()+\n", __func__);

	for (i = 0; i < ARRAY_SIZE(parser_gpio); i++) {
		gpio[i] = of_get_named_gpio(dt, parser_gpio[i], 0);
		FELICA_LOG_DEBUG("[FELICA_DD] %s  = %d",parser_gpio[i], gpio[i]);
		if (!gpio_is_valid(gpio[i])) {
			FELICA_LOG_ERR("[FELICA_DD] DT %s parser fail, ret = %d",parser_gpio[i], gpio[i]);
			goto parser_fail;
		}
	}

	fel_hsel_gpio  = gpio[0];
	fel_pon_gpio  = gpio[1];
	fel_cenz_gpio  = gpio[2];
	felica_cen_gpio  = gpio[3];
	felica_lock_gpio  = gpio[4];
	fel_hw_rst_gpio  = gpio[5];
	fel_int_gpio  = gpio[6];
	fel_intu_gpio  = gpio[7];
	fel_rfs_gpio  = gpio[8];

	FELICA_LOG_INFO("[FELICA_DD] DT:hsel[%d],pon[%d],cenz[%d],cen[%d],lock[%d],hwrst[%d],int[%d],intu[%d],rfs[%d]",
	fel_hsel_gpio, fel_pon_gpio, fel_cenz_gpio, felica_cen_gpio, felica_lock_gpio, fel_hw_rst_gpio, fel_int_gpio, fel_intu_gpio, fel_rfs_gpio);
/*
	felica_data.int_irq = gpio_to_irq(fel_int_gpio);
	felica_data.int_gpio = fel_int_gpio;
	felica_data.intu_irq = gpio_to_irq(fel_intu_gpio);
	felica_data.intu_gpio = fel_intu_gpio;
	pdata = &felica_data;
*/
	pdata -> int_irq = gpio_to_irq(fel_int_gpio);
	pdata -> int_gpio = fel_int_gpio;
	pdata -> intu_irq = gpio_to_irq(fel_intu_gpio);
	pdata -> intu_gpio = fel_intu_gpio;
	pdata -> setup_gpio = felica_setup_gpio;
	pdata -> sleep_gpio = felica_suspend;
	pdata -> wakeup_gpio = felica_resume;
	pdata -> pon_gpio_func = felica_pon_gpio_func;
	pdata -> cen_dtyp_d_func = felica_cen_dtyp_d_func;
	pdata -> cen_dtyp_cp_func = felica_cen_dtyp_cp_func;
	pdata -> cen_gpio_func = felica_cen_gpio_func;
	pdata -> rfs_gpio_func = felica_rfs_gpio_func;
	pdata -> int_gpio_func = felica_int_gpio_func;
	pdata -> con_gpio_func = felica_con_gpio_func;
	pdata -> hsel_gpio_func = felica_hsel_gpio_func;

	FELICA_LOG_INFO("[FELICA_DD] %s()-\n", __func__);
	return 0;

parser_fail:
	return ret;
}

#endif

/******************************************************************************
 * /dev/felica
 ******************************************************************************/
/* character device definition */
static dev_t devid_felica_uart;
static struct cdev cdev_felica_uart;
static const struct file_operations fops_felica_uart = {
	.owner				= THIS_MODULE,
	.open				= felica_uart_open,
	.release			= felica_uart_close,
	.read				= felica_uart_read,
	.write				= felica_uart_write,
	.fsync				= felica_uart_sync,
	.unlocked_ioctl			= felica_uart_ioctl,
};

/* add mechanism to fix race condition issue */
struct felica_sem_data {
	struct semaphore felica_sem;
};
static struct felica_sem_data *dev_sem;

/*
 * initialize device
 */
void felica_uart_init(void)
{
	int ret;
	struct device *device_felica_uart;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
#if 1
	#define FELICA_MAJOR_UART					91
	devid_felica_uart = MKDEV(FELICA_MAJOR_UART, FELICA_MINOR);
	ret = register_chrdev_region(devid_felica_uart, FELICA_MINOR_COUNT, FELICA_UART_NAME);
#else
	devid_felica_uart = MKDEV(FELICA_MAJOR, FELICA_MINOR);
	ret = alloc_chrdev_region(&devid_felica_uart, FELICA_BASEMINOR, FELICA_MINOR_COUNT, FELICA_UART_NAME);
	ret = register_chrdev_region(devid_felica_uart, FELICA_MINOR_COUNT, FELICA_UART_NAME);
#endif

	if( ret < 0 )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(alloc_chrdev_region), ret=[%d]", __func__, ret);
		return;
	}

	cdev_init(&cdev_felica_uart, &fops_felica_uart);
	ret = cdev_add(&cdev_felica_uart, devid_felica_uart, FELICA_MINOR_COUNT);
	if( ret < 0 )
	{
		unregister_chrdev_region(devid_felica_uart, FELICA_MINOR_COUNT);
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(cdev_add), ret=[%d]", __func__, ret);
		return;
	}

	device_felica_uart = device_create(felica_class, NULL, devid_felica_uart, NULL, FELICA_UART_NAME);
	if( IS_ERR(device_felica_uart) )
	{
		cdev_del(&cdev_felica_uart);
		unregister_chrdev_region(devid_felica_uart, FELICA_MINOR_COUNT);
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(device_create)", __func__);
		return;
	}
	gfa_open_cnt=0;
	/* fix race condition issue */
	dev_sem = kmalloc(sizeof(struct felica_sem_data), GFP_KERNEL);
	if (dev_sem == NULL) {
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(dev_sem malloc)", __func__);
		cdev_del(&cdev_felica_uart);
		unregister_chrdev_region(devid_felica_uart, FELICA_MINOR_COUNT);
		return;
	}
	sema_init(&dev_sem->felica_sem,1);

	FELICA_LOG_DEBUG("[FELICA_DD] %s END, major=[%d], minor=[%d]", __func__, MAJOR(devid_felica_uart), MINOR(devid_felica_uart));
}

/*
 * finalize device
 */
void felica_uart_exit(void)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
	/* fix race condition issue */
	kfree(dev_sem);

	device_destroy(felica_class, devid_felica_uart);
	cdev_del(&cdev_felica_uart);
	unregister_chrdev_region(devid_felica_uart, FELICA_MINOR_COUNT);

	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
}

/*
 * open device
 */
int felica_uart_open(struct inode *inode, struct file *file)
{
#ifdef FELICA_CONFIG_ACCESS_RESTRICTION
	uid_t uid;
#endif
	int ret;
	FELICA_LOG_INFO("[FELICA_DD] %s START", __func__);

#ifdef FELICA_CONFIG_ACCESS_RESTRICTION
	uid = __task_cred(current)->uid;
	if( (uid != gmfc_uid) && (uid != gdtl_uid) &&
        (uid != gdiag_uid)  )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s END, uid=[%d], gmfc_uid=[%d], gdiag_uid=[%d], gdtl_uid=[%d]", __func__, uid, gmfc_uid, gdiag_uid, gdtl_uid);
		if (board_mfg_mode() != 1)
			return -EACCES;
	}

#endif
	/* fix race condition issue */
	if (down_interruptible(&dev_sem->felica_sem)) {
		FELICA_LOG_ERR("[FELICA_DD] %s race condition", __func__);
		return -ERESTARTSYS;
	}
	FELICA_LOG_DEBUG("[FELICA_DD] %s DOWN SEM", __func__);

	mutex_lock(&uart_mutex);
	while (get_snfc_uart_status() == 1) {
		FELICA_LOG_INFO("[FELICA_DD] %s wait snfc_uart_status[%d]", __func__, get_snfc_uart_status());
		mutex_unlock(&uart_mutex);
		ret = wait_event_interruptible(wait_snfc_uart_release, get_snfc_uart_status() == 0);
		mutex_lock(&uart_mutex);
		FELICA_LOG_INFO("[FELICA_DD] %s Done(wait_event_interruptible), ret=[%d], snfc_uart_status[%d]", __func__, ret, get_snfc_uart_status());
	}

	FELICA_LOG_INFO("[FELICA_DD] %s get snfc_uart_status[%d]", __func__, get_snfc_uart_status());

	if( gfa_open_cnt == 0 )
	{
		memset(gfa_send_str, 0, FELICA_NL_MSG_SIZE);
		memset(gfa_rcv_str, 0, FELICA_NL_MSG_SIZE);
		gfa_send_str[0] = FELICA_NL_REQ_OPEN;
		felica_nl_send_msg(1);
		felica_nl_wait_ret_msg();
		if( gfa_rcv_str[1] == FELICA_NL_EFAILED )
		{
			FELICA_LOG_ERR("[FELICA_DD] %s Open Fail", __func__);
			/* fix race condition issue */
			up(&dev_sem->felica_sem);
			goto felica_uart_open_error;
		}
	}
	gfa_open_cnt++;
	/* fix race condition issue */
	up(&dev_sem->felica_sem);
	FELICA_LOG_DEBUG("[FELICA_DD] %s UP SEM", __func__);

	set_felica_uart_status(1);
	mutex_unlock(&uart_mutex);
	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return 0;

felica_uart_open_error:
	mutex_unlock(&uart_mutex);
	return -EFAULT;

}

/*
 * close device
 */
int felica_uart_close(struct inode *inode, struct file *file)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
	/* fix race condition issue */
	if (down_interruptible(&dev_sem->felica_sem)) {
		FELICA_LOG_ERR("[FELICA_DD] %s race condition", __func__);
		return -ERESTARTSYS;
	}
	FELICA_LOG_INFO("[FELICA_DD] %s START", __func__);

	mutex_lock(&uart_mutex);
	gfa_open_cnt--;
	if( gfa_open_cnt == 0 )
	{
		memset(gfa_send_str, 0, FELICA_NL_MSG_SIZE);
		memset(gfa_rcv_str, 0, FELICA_NL_MSG_SIZE);
		gfa_send_str[0] = FELICA_NL_REQ_CLOSE;
		felica_nl_send_msg(1);
		felica_nl_wait_ret_msg();
		if( gfa_rcv_str[1] == FELICA_NL_EFAILED )
		{
			FELICA_LOG_ERR("[FELICA_DD] %s Close Fail", __func__);
			gfa_open_cnt++;
			up(&dev_sem->felica_sem);
			goto felica_uart_close_error;
		}
	}
	/* fix race condition issue */
	up(&dev_sem->felica_sem);
	FELICA_LOG_DEBUG("[FELICA_DD] %s UP SEM", __func__);

	set_felica_uart_status(0);
	mutex_unlock(&uart_mutex);
	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return 0;

felica_uart_close_error:
	mutex_unlock(&uart_mutex);
	return -EFAULT;
}

/*
 * read operation
 */
ssize_t felica_uart_read(struct file *file, char __user *buf, size_t len, loff_t *ppos)
{

	int ret=0;
	size_t wk_len = 0;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START, len:%d", __func__, len);

	if (len <=0 ) {
		FELICA_LOG_ERR("[FELICA_DD] %s START, (len <=0 ), return", __func__);
		return 0;
	}

	/* fix race condition issue */
	if (down_interruptible(&dev_sem->felica_sem)) {
		FELICA_LOG_ERR("[FELICA_DD] %s race condition", __func__);
		return -ERESTARTSYS;
	}
	FELICA_LOG_DEBUG("[FELICA_DD] %s DOWN SEM", __func__);

	memset(gfa_send_str, 0, FELICA_NL_MSG_SIZE);
	memset(gfa_rcv_str, 0, FELICA_NL_MSG_SIZE);
	wk_len = len;
	if( FELICA_NL_MSG_DATA_SIZE < wk_len )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s  read max size over [%d]", __func__, wk_len);
		wk_len = FELICA_NL_MSG_DATA_SIZE;
	}
	gfa_send_str[0] = FELICA_NL_REQ_READ;
	gfa_send_str[1] = (char)(wk_len >> 8);
	gfa_send_str[2] = (char)wk_len;
	felica_nl_send_msg(3);
	wk_len = 0;
	felica_nl_wait_ret_msg();
	if( gfa_rcv_str[1] == FELICA_NL_SUCCESS )
	{
		wk_len = ( ( (int)gfa_rcv_str[2] << 8 ) & 0xFF00 ) | (int)gfa_rcv_str[3];
		ret = copy_to_user(buf, &gfa_rcv_str[4], wk_len);
		FELICA_LOG_DEBUG("[FELICA_DD] %s, len:%d, buf: %x %x %x %x", __func__, wk_len, buf[0], buf[1], buf[2], buf[3]);
		if( ret != 0 )
		{
			FELICA_LOG_ERR("[FELICA_DD] %s ERROR(copy_from_user), ret=[%d]", __func__, ret);
			/* fix race condition issue */
			up(&dev_sem->felica_sem);
			return -EFAULT;
		}
		*ppos = *ppos + wk_len;
	}
	else
	{
		FELICA_LOG_ERR(" %s FAIL", __func__);
		/* fix race condition issue */
		up(&dev_sem->felica_sem);
		return -EFAULT;
	}

	FELICA_LOG_DEBUG("[FELICA_DD] %s Success = %d, [0x%x, 0x%x]", __func__, wk_len, buf[1], buf[3]);
	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	/* fix race condition issue */
	up(&dev_sem->felica_sem);
	FELICA_LOG_DEBUG("[FELICA_DD] %s UP SEM", __func__);

	return (ssize_t)wk_len;
}

/*
 * write operation
 */
ssize_t felica_uart_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	int ret=0;
	size_t wk_len = 0;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
	if (len <=0 ) {
		FELICA_LOG_ERR("[FELICA_DD] %s START, (len <=0 ), return", __func__);
		return 0;
	}
	/* fix race condition issue */
	if (down_interruptible(&dev_sem->felica_sem)) {
		FELICA_LOG_ERR("[FELICA_DD] %s race condition", __func__);
		return -ERESTARTSYS;
	}
	FELICA_LOG_DEBUG("[FELICA_DD] %s DOWN SEM", __func__);
	memset(gfa_send_str, 0, FELICA_NL_MSG_SIZE);
	memset(gfa_rcv_str, 0, FELICA_NL_MSG_SIZE);
	wk_len = len;
	if( FELICA_NL_MSG_DATA_SIZE < wk_len )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s  read max size over [%d]", __func__, wk_len);
		wk_len = FELICA_NL_MSG_DATA_SIZE;
	}
	gfa_send_str[0] = FELICA_NL_REQ_WRITE;
	gfa_send_str[1] = (char)(wk_len >> 8 );
	gfa_send_str[2] = (char)wk_len;
	ret = copy_from_user(&gfa_send_str[3], data, wk_len);
	if( ret != 0 )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(copy_from_user), ret=[%d]", __func__, ret);
		/* fix race condition issue */
		up(&dev_sem->felica_sem);
		return -EFAULT;
	}
	felica_nl_send_msg(3+len);
	felica_nl_wait_ret_msg();
	wk_len = ( ( (int)gfa_rcv_str[2] << 8 ) & 0xFF00 ) | (int)gfa_rcv_str[3]; /* add for AndroidOS update. */
	if( gfa_rcv_str[1] == FELICA_NL_EFAILED )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s Write Fail", __func__);
		/* fix race condition issue */
		up(&dev_sem->felica_sem);
		return -EINVAL;
	}
	FELICA_LOG_DEBUG("[FELICA_DD] %s request = %d, Success = %d", __func__, len, wk_len);

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);

	/* fix race condition issue */
	up(&dev_sem->felica_sem);
	FELICA_LOG_DEBUG("[FELICA_DD] %s UP SEM", __func__);
	return (ssize_t)wk_len; /* add for AndroidOS update. */
}

/*
 * sync operation
 */
int felica_uart_sync(struct file *file, loff_t start, loff_t end, int datasync)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return 0;
}

/*
 * available operation
 */
long felica_uart_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned int ret_str = 0;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	switch (cmd) {
	case FIONREAD:
		/* fix race condition issue */
		if (down_interruptible(&dev_sem->felica_sem)) {
			FELICA_LOG_ERR("[FELICA_DD] %s race condition", __func__);
			return -ERESTARTSYS;
		}
		FELICA_LOG_DEBUG("[FELICA_DD] %s DOWN SEM", __func__);
		memset(gfa_send_str, 0, FELICA_NL_MSG_SIZE);
		memset(gfa_rcv_str, 0, FELICA_NL_MSG_SIZE);
		gfa_send_str[0] = FELICA_NL_REQ_AVAIABLE;
		felica_nl_send_msg(1);
		felica_nl_wait_ret_msg();
		if( gfa_rcv_str[1] == FELICA_NL_SUCCESS )
		{
			/* create response data */
			ret_str = ( ( (unsigned int)gfa_rcv_str[2] << 8 ) & 0xFF00 ) | (unsigned int) gfa_rcv_str[3];
			FELICA_LOG_DEBUG("Available Success data size [%d]", ret_str);
		}
		else
		{
			FELICA_LOG_ERR("[FELICA_DD] %s Available Fail", __func__);
			/* fix race condition issue */
			up(&dev_sem->felica_sem);
			return -EINVAL;
		}
		FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
		/* fix race condition issue */
		up(&dev_sem->felica_sem);

		FELICA_LOG_DEBUG("[FELICA_DD] %s UP SEM", __func__);
		return put_user(ret_str, (unsigned int __user *) arg);

	default:
		FELICA_LOG_INFO("[FELICA_DD] %s  default cmd=0x%x arg=0x%lx", __func__, cmd, arg);
		return -ENOIOCTLCMD;
	}

}

/*
 * create netlink socket
 */
void felica_nl_init(void)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
	gfa_connect_flag = 0;
	gfa_pid = 0;
	gfa_wait_flag = 0;
	memset(gfa_send_str, 0, FELICA_NL_MSG_SIZE);
	memset(gfa_rcv_str, 0, FELICA_NL_MSG_SIZE);
	gfanl_sk = netlink_kernel_create(&init_net, FELICA_NL_NETLINK_USER, 0, felica_nl_recv_msg, NULL, THIS_MODULE);
	if( !gfanl_sk )
	{
		FELICA_LOG_ERR("Error creating socket. %s\n",__func__);
	}
	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
}

/*
 * release netlink socket
 */
void felica_nl_exit(void)
{

	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
	netlink_kernel_release(gfanl_sk);
	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
}

/*
 * send message to FeliCa-Agent
 */
void felica_nl_send_msg(int len)
{

	struct nlmsghdr *nlh;
	struct sk_buff *skb_out;
	int msg_size = 0;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	if( !gfanl_sk )
	{
		FELICA_LOG_ERR("[FELICA_DD]Error Not creating socket. %s\n",__func__);
		return;
	}
	if( gfa_pid == 0 )
	{
		FELICA_LOG_ERR("[FELICA_DD]Error Not Rcv Connect Msg %s\n",__func__);
		return;
	}

	msg_size = len;
	skb_out = nlmsg_new(msg_size, 0);

	if( !skb_out )
	{
		FELICA_LOG_ERR("Failed to allocate new skb_out %s\n",__func__);
		return;
	}
	nlh = nlmsg_put(skb_out, 0, 0, NLMSG_DONE, msg_size, 0);
	NETLINK_CB(skb_out).dst_group = 0;
	memcpy(NLMSG_DATA(nlh), gfa_send_str, msg_size);

	// "skb_out" will release by netlink.
	nlmsg_unicast(gfanl_sk, skb_out, gfa_pid);
	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
}

/*
 * receive message from FeliCa-Agent
 */
void felica_nl_recv_msg(struct sk_buff *skb)
{

	struct nlmsghdr *nlh;
	struct sk_buff *wskb;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	if(!skb)
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(skb NULL)", __func__);
		return;
	}

	wskb = skb_get(skb);
	if( wskb && ( wskb->len > NLMSG_SPACE(0) ) )
	{
		nlh = nlmsg_hdr(wskb);
		memcpy(gfa_rcv_str, NLMSG_DATA(nlh), sizeof(gfa_rcv_str));
		if( ( gfa_rcv_str[0] == FELICA_NL_CONNECT_MSG ) && ( gfa_connect_flag == 0 ) )
		{
			/* pid of sending process */
			gfa_pid = nlh->nlmsg_pid;
			gfa_connect_flag = 1;
		}
		else if( ( gfa_rcv_str[0] == FELICA_NL_RESPONCE ) && ( gfa_pid == nlh->nlmsg_pid ) )
		{
			/* wake up */
			gfa_wait_flag = 1;
		}
		else
		{
			FELICA_LOG_ERR("[FELICA_DD] %s ERROR(RCV Undefine MSG)", __func__);
			FELICA_LOG_ERR("RCV MSG [%d]", gfa_rcv_str[0]);
			FELICA_LOG_ERR("rcv pid [%d]", nlh->nlmsg_pid);
			FELICA_LOG_ERR("gfa_pid [%d]", gfa_pid);
		}
	}
	kfree_skb(skb);

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
}

/*
 * waiting to receive messages from FeliCa-Agent
 */
void felica_nl_wait_ret_msg(void)
{
	unsigned int cnt=0;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START ", __func__);
	while(1)
	{
		if( gfa_wait_flag == 1 )
		{
			FELICA_LOG_DEBUG("[FELICA_DD] %s sleep cnt [%d]", __func__, cnt);
			break;
		}
		mdelay(1);
		cnt++;
	}
	gfa_wait_flag = 0;
	FELICA_LOG_DEBUG("[FELICA_DD] %s END ", __func__);
}



/******************************************************************************
 * /dev/felica_pon
 ******************************************************************************/

/* character device definition */
static dev_t devid_felica_pon;
static struct cdev cdev_felica_pon;
static struct file_operations fops_felica_pon = {
	.owner		= THIS_MODULE,
	.open		= felica_pon_open,
	.release	= felica_pon_close,
	.read		= felica_pon_read,
	.write		= felica_pon_write,
};

/*
 * initialize device
 */
void felica_pon_init(void)
{
	int ret;
	struct device *device_felica_pon;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

#if 1
	#define FELICA_MAJOR_PON					92
	devid_felica_pon = MKDEV(FELICA_MAJOR_PON, FELICA_MINOR);
	ret = register_chrdev_region(devid_felica_pon, FELICA_MINOR_COUNT, FELICA_PON_NAME);
#else
	devid_felica_pon = MKDEV(FELICA_MAJOR, FELICA_MINOR);
	ret = alloc_chrdev_region(&devid_felica_pon, FELICA_BASEMINOR, FELICA_MINOR_COUNT, FELICA_PON_NAME);
#endif

	if( ret < 0 )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(alloc_chrdev_region), ret=[%d]", __func__, ret);
		return;
	}

	cdev_init(&cdev_felica_pon, &fops_felica_pon);
	ret = cdev_add(&cdev_felica_pon, devid_felica_pon, FELICA_MINOR_COUNT);
	if( ret < 0 )
	{
		unregister_chrdev_region(devid_felica_pon, FELICA_MINOR_COUNT);
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(cdev_add), ret=[%d]", __func__, ret);
		return;
	}

	device_felica_pon = device_create(felica_class, NULL, devid_felica_pon, NULL, FELICA_PON_NAME);
	if( IS_ERR(device_felica_pon) )
	{
		cdev_del(&cdev_felica_pon);
		unregister_chrdev_region(devid_felica_pon, FELICA_MINOR_COUNT);
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(device_create)", __func__);
		return;
	}

	FELICA_LOG_DEBUG("[FELICA_DD] %s END, major=[%d], minor=[%d]", __func__, MAJOR(devid_felica_pon), MINOR(devid_felica_pon));
}

/*
 * finalize device
 */
void felica_pon_exit(void)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	device_destroy(felica_class, devid_felica_pon);
	cdev_del(&cdev_felica_pon);
	unregister_chrdev_region(devid_felica_pon, FELICA_MINOR_COUNT);

	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
}

/*
 * open device
 */
int felica_pon_open(struct inode *inode, struct file *file)
{
#ifdef FELICA_CONFIG_ACCESS_RESTRICTION
	uid_t uid;
#endif
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

#ifdef FELICA_CONFIG_ACCESS_RESTRICTION
	uid = __task_cred(current)->uid;
	if( (uid != gmfc_uid) && (uid != gdtl_uid) &&
        (uid != gdiag_uid)  )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s END, uid=[%d], gmfc_uid=[%d], gdiag_uid=[%d], gdtl_uid=[%d]", __func__, uid, gmfc_uid, gdiag_uid, gdtl_uid);
		if (board_mfg_mode() != 1)
			return -EACCES;
	}
#endif
	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return 0;
}

/*
 * close device
 */
int felica_pon_close(struct inode *inode, struct file *file)
{
	uid_t uid;

	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	uid = __task_cred(current)->uid;
	if( uid == gdtl_uid  )
	{
		if( felica_pdata->pon_gpio_func )
			felica_pdata->pon_gpio_func(GPIOWRITE, GPIO_VALUE_LOW, NULL);
		else
			FELICA_LOG_ERR("[FELICA_DD] %s felica_pdata->pon_gpio_func is NULL", __func__);
	}

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return 0;
}

/*
 * read operation
 */
ssize_t felica_pon_read(struct file *file, char __user *buf, size_t len, loff_t *ppos)
{
	int ret;
	char retparam;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	if( felica_pdata->pon_gpio_func )
		felica_pdata->pon_gpio_func(GPIOREAD, 0, &ret);
	else {
		FELICA_LOG_ERR("[FELICA_DD] %s felica_pdata->pon_gpio_func is NULL", __func__);
		return -EFAULT;
	}

	if( ret == GPIO_VALUE_HIGH )
	{
		retparam = FELICA_PON_WIRED;
		FELICA_LOG_INFO("[FELICA_DD] Wired interface Status is [%d][start]", retparam);
	}
	else if( ret == GPIO_VALUE_LOW )
	{
		retparam = FELICA_PON_WIRELESS;
		FELICA_LOG_INFO("[FELICA_DD] Wired interface Status is [%d][standby]", retparam);
	}
	else
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(gpio_get_value), ret=[%d]", __func__, ret);
		return -EIO;
	}

	ret = copy_to_user(buf, &retparam, FELICA_PON_DATA_LEN);
	if( ret != 0 )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(copy_to_user), ret=[%d]", __func__, ret);
		return -EFAULT;
	}
	*ppos += 1;

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return FELICA_PON_DATA_LEN;
}

/*
 * write operation
 */
ssize_t felica_pon_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	char pon;
	int ret;
	int setparam;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	ret = copy_from_user(&pon, data, FELICA_PON_DATA_LEN);
	if( ret != 0 )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(copy_from_user), ret=[%d]", __func__, ret);
		return -EFAULT;
	}

	if( pon == FELICA_PON_WIRED )
	{
		setparam = GPIO_VALUE_HIGH;
		FELICA_LOG_INFO("[FELICA_DD] Set wired interface to [%d][start]", setparam);
	}
	else if( pon == FELICA_PON_WIRELESS )
	{
		setparam = GPIO_VALUE_LOW;
		FELICA_LOG_INFO("[FELICA_DD] Set wired interface to [%d][standby]", setparam);
	}
	else
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(copy_from_user), pon=[%d]", __func__, pon);
		return -EINVAL;
	}

	if( felica_pdata->pon_gpio_func )
		felica_pdata->pon_gpio_func(GPIOWRITE, setparam, NULL);
	else {
		FELICA_LOG_ERR("[FELICA_DD] %s felica_pdata->pon_gpio_func is NULL", __func__);
		return -EFAULT;
	}

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return FELICA_PON_DATA_LEN;
}





/******************************************************************************
 * /dev/felica_cen
 ******************************************************************************/

/* character device definition */
static dev_t devid_felica_cen;
static struct cdev cdev_felica_cen;
static struct file_operations fops_felica_cen = {
	.owner		= THIS_MODULE,
	.open		= felica_cen_open,
	.release	= felica_cen_close,
	.read		= felica_cen_read,
	.write		= felica_cen_write,
};

/*
 * initialize device
 */
void felica_cen_init(void)
{
	int ret;
	struct device *device_felica_cen;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	//FELICA_LOG_DEBUG("[FELICA_DD] %s DTYPE_D:%d, DTYPE_CP:%d", __func__, GPIO_OUT_FELICA_DTYPE_D, GPIO_OUT_FELICA_DTYPE_CP);

#if 1
	#define FELICA_MAJOR_CEN					93
	devid_felica_cen = MKDEV(FELICA_MAJOR_CEN, FELICA_MINOR);
	ret = register_chrdev_region(devid_felica_cen, FELICA_MINOR_COUNT, FELICA_CEN_NAME);
#else
	devid_felica_cen = MKDEV(FELICA_MAJOR, FELICA_MINOR);
	ret = alloc_chrdev_region(&devid_felica_cen, FELICA_BASEMINOR, FELICA_MINOR_COUNT, FELICA_CEN_NAME);
#endif
	if( ret < 0 )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(alloc_chrdev_region), ret=[%d]", __func__, ret);
		return;
	}

	cdev_init(&cdev_felica_cen, &fops_felica_cen);
	ret = cdev_add(&cdev_felica_cen, devid_felica_cen, FELICA_MINOR_COUNT);
	if( ret < 0 )
	{
		unregister_chrdev_region(devid_felica_cen, FELICA_MINOR_COUNT);
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(cdev_add), ret=[%d]", __func__, ret);
		return;
	}

	device_felica_cen = device_create(felica_class, NULL, devid_felica_cen, NULL, FELICA_CEN_NAME);
	if( IS_ERR(device_felica_cen) )
	{
		cdev_del(&cdev_felica_cen);
		unregister_chrdev_region(devid_felica_cen, FELICA_MINOR_COUNT);
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(device_create)", __func__);
		return;
	}

	gfelica_cen_status = FELICA_CEN_STATUS_INIT;
	gfelica_cen_opencnt = 0;
	FELICA_LOG_DEBUG("[FELICA_DD] %s END, major=[%d], minor=[%d]", __func__, MAJOR(devid_felica_cen), MINOR(devid_felica_cen));
}

/*
 * finalize device
 */
void felica_cen_exit(void)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	device_destroy(felica_class, devid_felica_cen);
	cdev_del(&cdev_felica_cen);
	unregister_chrdev_region(devid_felica_cen, FELICA_MINOR_COUNT);

	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
}

/*
 * open device
 */
int felica_cen_open(struct inode *inode, struct file *file)
{
#ifdef FELICA_CONFIG_ACCESS_RESTRICTION
	uid_t uid;
#endif
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

#ifdef FELICA_CONFIG_ACCESS_RESTRICTION
	uid = __task_cred(current)->uid;
	FELICA_LOG_INFO("[FELICA_DD] %s uid:%d, gmfl_uid:%d, gdiag_uid:%d, gmfc_uid:%d, gdtl_uid:%d, mfg_mode:%d\n",
		__func__, uid, gmfl_uid, gdiag_uid, gmfc_uid, gdtl_uid, board_mfg_mode());
	if( file->f_mode & FMODE_WRITE )
	{
		if( (uid != gdiag_uid) &&
	        (uid != gmfl_uid)  )
		{
			FELICA_LOG_ERR("[FELICA_DD] %s END_F1, uid=[%d], gmfc_uid=[%d], gdiag_uid=[%d], gmfl_uid=[%d], gdtl_uid=[%d]",
					__func__, uid, gmfc_uid, gdiag_uid, gmfl_uid, gdtl_uid);
			if (board_mfg_mode() != 1)
				return -EACCES;
		}
	}else{
		if( (uid != gmfc_uid) && (uid != gdtl_uid) &&
	        (uid != gdiag_uid) &&
	        (uid != gmfl_uid)  )
		{
			FELICA_LOG_ERR("[FELICA_DD] %s END_F2, uid=[%d], gmfc_uid=[%d], gdiag_uid=[%d], gmfl_uid=[%d], gdtl_uid=[%d]",
					__func__, uid, gmfc_uid, gdiag_uid, gmfl_uid, gdtl_uid);
			if (board_mfg_mode() != 1)
				return -EACCES;
		}
	}

#endif
	gfelica_cen_opencnt++;
	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return 0;
}

/*
 * close device
 */
int felica_cen_close(struct inode *inode, struct file *file)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
	gfelica_cen_opencnt--;
	if ( gfelica_cen_opencnt == 0 )
	{

	}
	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return 0;
}

/*
 * read operation
 */
ssize_t felica_cen_read(struct file *file, char __user *buf, size_t len, loff_t *ppos)
{
	int ret;
	char retparam;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	if( felica_pdata->cen_gpio_func )
		felica_pdata->cen_gpio_func(GPIOREAD, 0, &ret);
	else
		FELICA_LOG_ERR("[FELICA_DD] %s felica_pdata->cen_gpio_func is NULL", __func__);


	if( gfelica_cen_status == GPIO_VALUE_HIGH )
	{
		retparam = FELICA_CEN_UNLOCK;
		FELICA_LOG_INFO("[FELICA_DD] FeliCa-Lock status is [%d][UnLock]", retparam);
	}
	else if( gfelica_cen_status == GPIO_VALUE_LOW )
	{
		retparam = FELICA_CEN_LOCK;
		FELICA_LOG_INFO("[FELICA_DD] FeliCa-Lock status is [%d][Lock]", retparam);
	}
	else
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(gpio_get_value), ret=[%d]", __func__, gfelica_cen_status);
		return -EIO;
	}

	ret = copy_to_user(buf, &retparam, FELICA_CEN_DATA_LEN);
	if( ret != 0 )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(copy_to_user), ret=[%d]", __func__, ret);
		return -EFAULT;
	}
	*ppos += 1;

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return FELICA_CEN_DATA_LEN;
}

/*
 * write operation
 */
ssize_t felica_cen_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	char cen;
	int ret;
	int setparam;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	ret = copy_from_user(&cen, data, FELICA_CEN_DATA_LEN);
	if( ret != 0 )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(copy_from_user), ret=[%d]", __func__, ret);
		return -EFAULT;
	}

	if( cen == FELICA_CEN_UNLOCK )
	{
		setparam = GPIO_VALUE_HIGH;
		FELICA_LOG_INFO("[FELICA_DD] Set FeliCa-Lock status to [%d][UnLock]", setparam);
	}
	else if( cen == FELICA_CEN_LOCK )
	{
		setparam = GPIO_VALUE_LOW;
		FELICA_LOG_INFO("[FELICA_DD] Set FeliCa-Lock status to [%d][Lock]", setparam);
	}
	else
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(copy_from_user), cen=[%d]", __func__, cen);
		return -EINVAL;
	}

	if( felica_pdata->cen_gpio_func )
		felica_pdata->cen_gpio_func(GPIOWRITE, setparam, NULL);
	else {
		FELICA_LOG_ERR("[FELICA_DD] %s felica_pdata->cen_gpio_func is NULL", __func__);
		return -EFAULT;
	}
	gfelica_cen_status = setparam;

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return FELICA_CEN_DATA_LEN;	/* add for AndroidOS update. */
}

/******************************************************************************
 * /dev/felica_rfs
 ******************************************************************************/

/* character device definition */
static dev_t devid_felica_rfs;
static struct cdev cdev_felica_rfs;
static struct file_operations fops_felica_rfs = {
	.owner		= THIS_MODULE,
	.open		= felica_rfs_open,
	.release	= felica_rfs_close,
	.read		= felica_rfs_read,
};

/*
 * initialize device
 */
void felica_rfs_init(void)
{
	int ret;
	struct device *device_felica_rfs;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

#if 1
	#define FELICA_MAJOR_RFS					94
	devid_felica_rfs = MKDEV(FELICA_MAJOR_RFS, FELICA_MINOR);
	ret = register_chrdev_region(devid_felica_rfs, FELICA_MINOR_COUNT, FELICA_RFS_NAME);
#else
	devid_felica_rfs = MKDEV(FELICA_MAJOR, FELICA_MINOR);
	ret = alloc_chrdev_region(&devid_felica_rfs, FELICA_BASEMINOR, FELICA_MINOR_COUNT, FELICA_RFS_NAME);
#endif

	if( ret < 0 )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(alloc_chrdev_region), ret=[%d]", __func__, ret);
		return;
	}

	cdev_init(&cdev_felica_rfs, &fops_felica_rfs);
	ret = cdev_add(&cdev_felica_rfs, devid_felica_rfs, FELICA_MINOR_COUNT);
	if( ret < 0 )
	{
		unregister_chrdev_region(devid_felica_rfs, FELICA_MINOR_COUNT);
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(cdev_add), ret=[%d]", __func__, ret);
		return;
	}

	device_felica_rfs = device_create(felica_class, NULL, devid_felica_rfs, NULL, FELICA_RFS_NAME);
	if( IS_ERR(device_felica_rfs) )
	{
		cdev_del(&cdev_felica_rfs);
		unregister_chrdev_region(devid_felica_rfs, FELICA_MINOR_COUNT);
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(device_create)", __func__);
		return;
	}

	FELICA_LOG_DEBUG("[FELICA_DD] %s END, major=[%d], minor=[%d]", __func__, MAJOR(devid_felica_rfs), MINOR(devid_felica_rfs));
}

/*
 * finalize device
 */
void felica_rfs_exit(void)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	device_destroy(felica_class, devid_felica_rfs);
	cdev_del(&cdev_felica_rfs);
	unregister_chrdev_region(devid_felica_rfs, FELICA_MINOR_COUNT);

	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
}

/*
 * open device
 */
int felica_rfs_open(struct inode *inode, struct file *file)
{
#ifdef FELICA_CONFIG_ACCESS_RESTRICTION
	uid_t uid;
#endif
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

#ifdef FELICA_CONFIG_ACCESS_RESTRICTION
	uid = __task_cred(current)->uid;

	if( (uid != gmfc_uid) && (uid != gdtl_uid) &&
		(uid != gdiag_uid)  )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s END, uid=[%d], gmfc_uid=[%d], gdiag_uid=[%d], gdtl_uid=[%d]", __func__, uid, gmfc_uid, gdiag_uid, gdtl_uid);
		if (board_mfg_mode() != 1)
			return -EACCES;
	}

#endif
	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return 0;
}

/*
 * close device
 */
int felica_rfs_close(struct inode *inode, struct file *file)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
	/* no operation */
	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return 0;
}

/*
 * read operation
 */
ssize_t felica_rfs_read(struct file *file, char __user *buf, size_t len, loff_t *ppos)
{
	int ret;
	char retparam;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	if( felica_pdata->rfs_gpio_func )
		felica_pdata->rfs_gpio_func(GPIOREAD, 0, &ret);
	else {
		FELICA_LOG_ERR("[FELICA_DD] %s felica_pdata->rfs_gpio_func is NULL", __func__);
		return -EFAULT;
	}

	if( ret == GPIO_VALUE_HIGH )
	{
		retparam = FELICA_RFS_STANDBY;
		FELICA_LOG_DEBUG("Carrier-Wave Status is [%d][standby]", retparam);
	}
	else if( ret == GPIO_VALUE_LOW )
	{
		retparam = FELICA_RFS_DETECTED;
		FELICA_LOG_DEBUG("Carrier-Wave Status is [%d][detected]", retparam);
	}
	else
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(gpio_get_value), ret=[%d]", __func__, ret);
		return -EIO;
	}

	ret = copy_to_user(buf, &retparam, FELICA_RFS_DATA_LEN);
	if( ret != 0 )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(copy_to_user), ret=[%d]", __func__, ret);
		return -EFAULT;
	}
	*ppos += 1;

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return FELICA_RFS_DATA_LEN;
}



/******************************************************************************
 * /dev/felica_rws
 ******************************************************************************/

/* character device definition */
static dev_t devid_felica_rws;
static struct cdev cdev_felica_rws;
static struct file_operations fops_felica_rws = {
	.owner		= THIS_MODULE,
	.open		= felica_rws_open,
	.release	= felica_rws_close,
	.read		= felica_rws_read,
	.write		= felica_rws_write,
};

/*
 * initialize device
 */
void felica_rws_init(void)
{
	int ret;
	struct device *device_felica_rws;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

#if 1
	#define FELICA_MAJOR_RWS					95
	devid_felica_rws = MKDEV(FELICA_MAJOR_RWS, FELICA_MINOR);
	ret = register_chrdev_region(devid_felica_rws, FELICA_MINOR_COUNT, FELICA_RWS_NAME);
#else
	devid_felica_rws = MKDEV(FELICA_MAJOR, FELICA_MINOR);
	ret = alloc_chrdev_region(&devid_felica_rws, FELICA_BASEMINOR, FELICA_MINOR_COUNT, FELICA_RWS_NAME);
#endif

	if( ret < 0 )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(alloc_chrdev_region), ret=[%d]", __func__, ret);
		return;
	}

	cdev_init(&cdev_felica_rws, &fops_felica_rws);
	ret = cdev_add(&cdev_felica_rws, devid_felica_rws, FELICA_MINOR_COUNT);
	if( ret < 0 )
	{
		unregister_chrdev_region(devid_felica_rws, FELICA_MINOR_COUNT);
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(cdev_add), ret=[%d]", __func__, ret);
		return;
	}

	device_felica_rws = device_create(felica_class, NULL, devid_felica_rws, NULL, FELICA_RWS_NAME);
	if( IS_ERR(device_felica_rws) )
	{
		cdev_del(&cdev_felica_rws);
		unregister_chrdev_region(devid_felica_rws, FELICA_MINOR_COUNT);
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(device_create)", __func__);
		return;
	}

	gfelica_rw_status = FELICA_RW_STATUS_INIT;

	FELICA_LOG_DEBUG("[FELICA_DD] %s END, major=[%d], minor=[%d]", __func__, MAJOR(devid_felica_rws), MINOR(devid_felica_rws));
}

/*
 * finalize device
 */
void felica_rws_exit(void)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	device_destroy(felica_class, devid_felica_rws);
	cdev_del(&cdev_felica_rws);
	unregister_chrdev_region(devid_felica_rws, FELICA_MINOR_COUNT);

	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
}

/*
 * open device
 */
int felica_rws_open(struct inode *inode, struct file *file)
{
#ifdef FELICA_CONFIG_ACCESS_RESTRICTION
	uid_t uid;
#endif
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

#ifdef FELICA_CONFIG_ACCESS_RESTRICTION
	uid = __task_cred(current)->uid;
	if( file->f_mode & FMODE_WRITE )
	{
		if( uid != grwm_uid )
		{
			FELICA_LOG_ERR("[FELICA_DD] %s END, uid=[%d], gmfc_uid=[%d], gdiag_uid=[%d], gdtl_uid=[%d]", __func__, uid, gmfc_uid, gdiag_uid, gdtl_uid);
			if (board_mfg_mode() != 1)
				return -EACCES;
		}
	}else{
		if( (uid != gmfc_uid) && (uid != gdtl_uid) &&
			(uid != grwm_uid)  )
		{
			FELICA_LOG_ERR("[FELICA_DD] %s END, uid=[%d], gmfc_uid=[%d], gdiag_uid=[%d], gdtl_uid=[%d]", __func__, uid, gmfc_uid, gdiag_uid, gdtl_uid);
			if (board_mfg_mode() != 1)
				return -EACCES;
		}
	}

#endif

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return 0;
}

/*
 * close device
 */
int felica_rws_close(struct inode *inode, struct file *file)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
	/* no operation */
	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return 0;
}

/*
 * read operation
 */
ssize_t felica_rws_read(struct file *file, char __user *buf, size_t len, loff_t *ppos)
{
	int ret;
	char retparam;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	if( gfelica_rw_status == FELICA_RW_STATUS_ENABLE )
	{
		retparam = FELICA_RW_STATUS_ENABLE;
		FELICA_LOG_DEBUG("ReaderWriterFunction status is [%d][enabled]", retparam);
	}
	else if( gfelica_rw_status == FELICA_RW_STATUS_DISABLE )
	{
		retparam = FELICA_RW_STATUS_DISABLE;
		FELICA_LOG_DEBUG("ReaderWriterFunction status is [%d][disabled]", retparam);
	}
	else
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(gfelica_rw_status), RWM=[%d]", __func__, gfelica_rw_status);
		return -EIO;
	}

	ret = copy_to_user(buf, &retparam, FELICA_RWS_DATA_LEN);
	if( ret != 0 )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(copy_to_user), ret=[%d]", __func__, ret);
		return -EFAULT;
	}
	*ppos += 1;

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return FELICA_RWS_DATA_LEN;
}

/*
 * write operation
 */
ssize_t felica_rws_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	char work;
	int ret;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	ret = copy_from_user(&work, data, FELICA_RWS_DATA_LEN);
	if( ret != 0 )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(copy_from_user), ret=[%d]", __func__, ret);
		return -EFAULT;
	}

	if( work == FELICA_RW_STATUS_ENABLE )
	{
		FELICA_LOG_DEBUG("Set ReaderWriterFunction status to [%d][enable]", work);
	}
	else if( work == FELICA_RW_STATUS_DISABLE )
	{
		FELICA_LOG_DEBUG("Set ReaderWriterFunction status to s[%d][disable]", work);
	}
	else
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(copy_from_user), work=[%d]", __func__, work);
		return -EINVAL;
	}

	gfelica_rw_status = work;

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return (ssize_t)FELICA_RWS_DATA_LEN; /* add for AndroidOS update. */
}





/******************************************************************************
 * /dev/felica_int
 ******************************************************************************/

/* character device definition */
static dev_t devid_felica_int;
static struct cdev cdev_felica_int;
static struct file_operations fops_felica_int = {
	.owner		= THIS_MODULE,
	.open		= felica_int_open,
	.release	= felica_int_close,
	.read		= felica_int_read,
};

/*
 * initialize device
 */
void felica_int_init(void)
{
	int ret;

	struct device *device_felica_int;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

#if 1
	#define FELICA_MAJOR_INT					96
	devid_felica_int = MKDEV(FELICA_MAJOR_INT, FELICA_MINOR);
	ret = register_chrdev_region(devid_felica_int, FELICA_MINOR_COUNT, FELICA_INT_NAME);
#else
	devid_felica_int = MKDEV(FELICA_MAJOR, FELICA_MINOR);
	ret = alloc_chrdev_region(&devid_felica_int, FELICA_BASEMINOR, FELICA_MINOR_COUNT, FELICA_INT_NAME);
#endif
	if( ret < 0 )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(alloc_chrdev_region), ret=[%d]", __func__, ret);
		return;
	}

	cdev_init(&cdev_felica_int, &fops_felica_int);
	ret = cdev_add(&cdev_felica_int, devid_felica_int, FELICA_MINOR_COUNT);
	if( ret < 0 )
	{
		unregister_chrdev_region(devid_felica_int, FELICA_MINOR_COUNT);
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(cdev_add), ret=[%d]", __func__, ret);
		return;
	}

	device_felica_int = device_create(felica_class, NULL, devid_felica_int, NULL, FELICA_INT_NAME);
	if( IS_ERR(device_felica_int) )
	{
		cdev_del(&cdev_felica_int);
		unregister_chrdev_region(devid_felica_int, FELICA_MINOR_COUNT);
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(device_create)", __func__);
		return;
	}

	FELICA_LOG_DEBUG("[FELICA_DD] %s END, major=[%d], minor=[%d]", __func__, MAJOR(devid_felica_int), MINOR(devid_felica_int));
}

/*
 * finalize device
 */
void felica_int_exit(void)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	device_destroy(felica_class, devid_felica_int);
	cdev_del(&cdev_felica_int);
	unregister_chrdev_region(devid_felica_int, FELICA_MINOR_COUNT);

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
}

/*
 * open device
 */
int felica_int_open(struct inode *inode, struct file *file)
{
#ifdef FELICA_CONFIG_ACCESS_RESTRICTION
	uid_t uid;
#endif
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

#ifdef FELICA_CONFIG_ACCESS_RESTRICTION
	uid = __task_cred(current)->uid;
	if( uid != gdiag_uid)
	{
		FELICA_LOG_ERR("[FELICA_DD] %s END, uid=[%d], gdiag_uid=[%d], ", __func__, uid, gdiag_uid);
		if (board_mfg_mode() != 1)
			return -EACCES;
	}

#endif
	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return 0;
}

/*
 * close device
 */
int felica_int_close(struct inode *inode, struct file *file)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
	/* no operation */
	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return 0;
}

/*
 * read operation
 */
ssize_t felica_int_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	int ret;
	char retparam;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	ret = gpio_get_value(felica_int_pin);

	if( ret == GPIO_VALUE_HIGH )
	{
		retparam = FELICA_INT_HIGH;
		FELICA_LOG_DEBUG("INT-PIN value is [%d][HIGH]", retparam);
	}
	else if( ret == GPIO_VALUE_LOW )
	{
		retparam = FELICA_INT_LOW;
		FELICA_LOG_DEBUG("INT-PIN value is [%d][LOW]", retparam);
	}
	else
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(gpio_get_value), ret=[%d]", __func__, ret);
		return -EIO;
	}

	ret = copy_to_user(buf, &retparam, FELICA_INT_DATA_LEN);
	if( ret != 0 )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(copy_to_user), ret=[%d]", __func__, ret);
		return -EFAULT;
	}
	*ppos += 1;

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return FELICA_INT_DATA_LEN;
}


/******************************************************************************
 * /dev/felica_int_poll
 ******************************************************************************/

/* character device definition */
static dev_t devid_felica_int_poll;
static struct cdev cdev_felica_int_poll;
static struct file_operations fops_felica_int_poll = {
	.owner		= THIS_MODULE,
	.open		= felica_int_poll_open,
	.release	= felica_int_poll_close,
	.read		= felica_int_poll_read,
	.poll		= felica_int_poll_poll,
};

/*
 * top half of irq_handler
 */
irqreturn_t felica_int_irq_handler(int irq, void *dev_id)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	disable_irq_nosync(felica_int_irq);

	schedule_delayed_work(&pgint_irq->work, msecs_to_jiffies(FELICA_INT_DELAY_TIME));

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return IRQ_HANDLED;
}

/*
 * bottom half of irq_handler
 */
void felica_int_irq_work(struct work_struct *work)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	enable_irq(felica_int_irq);

	pgint_irq->irq_done = 1;
	wake_up_interruptible(&pgint_irq->read_wait);

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
}

/*
 * initialize device
 */
void felica_int_poll_init(void)
{
	int ret;

	struct device *device_felica_int_poll;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

#if 1
	#define FELICA_MAJOR_INT_POLL				97
	devid_felica_int_poll = MKDEV(FELICA_MAJOR_INT_POLL, FELICA_MINOR);
	ret = register_chrdev_region(devid_felica_int_poll, FELICA_MINOR_COUNT, FELICA_INT_POLL_NAME);
#else
	devid_felica_int_poll = MKDEV(FELICA_MAJOR, FELICA_MINOR);
	ret = alloc_chrdev_region(&devid_felica_int_poll, FELICA_BASEMINOR, FELICA_MINOR_COUNT, FELICA_INT_POLL_NAME);
#endif
	if( ret < 0 )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(alloc_chrdev_region), ret=[%d]", __func__, ret);
		return;
	}

	cdev_init(&cdev_felica_int_poll, &fops_felica_int_poll);
	ret = cdev_add(&cdev_felica_int_poll, devid_felica_int_poll, FELICA_MINOR_COUNT);
	if( ret < 0 )
	{
		unregister_chrdev_region(devid_felica_int_poll, FELICA_MINOR_COUNT);
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(cdev_add), ret=[%d]", __func__, ret);
		return;
	}

	device_felica_int_poll = device_create(felica_class, NULL, devid_felica_int_poll, NULL, FELICA_INT_POLL_NAME);
	if( IS_ERR(device_felica_int_poll) )
	{
		cdev_del(&cdev_felica_int_poll);
		unregister_chrdev_region(devid_felica_int_poll, FELICA_MINOR_COUNT);
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(device_create)", __func__);
		return;
	}

	memset(pgint_irq, 0x00, sizeof(struct felica_int_irqdata));
	INIT_DELAYED_WORK(&pgint_irq->work, felica_int_irq_work);
	init_waitqueue_head(&pgint_irq->read_wait);
	ret = request_irq(felica_int_irq,
				felica_int_irq_handler,
				IRQF_TRIGGER_FALLING,
				FELICA_INT_POLL_NAME,
				(void*)pgint_irq);
	if( ret != 0 )
	{
		device_destroy(felica_class, devid_felica_int_poll);
		cdev_del(&cdev_felica_int_poll);
		unregister_chrdev_region(devid_felica_int_poll, FELICA_MINOR_COUNT);
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(request_irq), ret=[%d]", __func__, ret);
		return;
	}

	ret = enable_irq_wake(felica_int_irq);

	if( ret < 0 )
	{
		free_irq(felica_int_irq, (void*)pgint_irq);
		device_destroy(felica_class, devid_felica_int_poll);
		cdev_del(&cdev_felica_int_poll);
		unregister_chrdev_region(devid_felica_int_poll, FELICA_MINOR_COUNT);
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(enable_irq_wake), ret=[%d]", __func__, ret);
		return;
	}

	pgint_irq->irq_done = 0;

	FELICA_LOG_DEBUG("[FELICA_DD] %s END, major=[%d], minor=[%d]", __func__, MAJOR(devid_felica_int_poll), MINOR(devid_felica_int_poll));
}

/*
 * finalize device
 */
void felica_int_poll_exit(void)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	disable_irq(felica_int_irq);

	free_irq(felica_int_irq, (void*)pgint_irq);

	device_destroy(felica_class, devid_felica_int_poll);
	cdev_del(&cdev_felica_int_poll);
	unregister_chrdev_region(devid_felica_int_poll, FELICA_MINOR_COUNT);

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
}

/*
 * open device
 */
int felica_int_poll_open(struct inode *inode, struct file *file)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
	/* no operation */
	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return 0;
}

/*
 * close device
 */
int felica_int_poll_close(struct inode *inode, struct file *file)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
	/* no operation */
	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return 0;
}

/*
 * read operation
 */
ssize_t felica_int_poll_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	int ret;
	char retparam;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	if( !pgint_irq->irq_done )
	{
		ret = wait_event_interruptible(pgint_irq->read_wait, pgint_irq->irq_done == 1);
		if( ret < 0 )
		{
			FELICA_LOG_WARN("[FELICA_DD] %s warn(wait_event_interruptible), ret=[%d]", __func__, ret);
			return -EINTR;
		}
	}

	ret = gpio_get_value(felica_int_pin);

	if( ret == GPIO_VALUE_HIGH )
	{
		retparam = FELICA_INT_HIGH;
		FELICA_LOG_DEBUG("INT-PIN value is [%d][HIGH]", retparam);
	}
	else if( ret == GPIO_VALUE_LOW )
	{
		retparam = FELICA_INT_LOW;
		FELICA_LOG_DEBUG("INT-PIN value is [%d][LOW]", retparam);
	}
	else
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(gpio_get_value), ret=[%d]", __func__, ret);
		return -EIO;
	}

	ret = copy_to_user(buf, &retparam, FELICA_INT_DATA_LEN);
	if( ret != 0 )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(copy_to_user), ret=[%d]", __func__, ret);
		return -EFAULT;
	}
	*ppos += 1;

	pgint_irq->irq_done = 0;

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return FELICA_INT_DATA_LEN;
}

/*
 * poll operation
 */
unsigned int felica_int_poll_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;

	FELICA_LOG_DEBUG("%s START", __func__);

	poll_wait(file, &pgint_irq->read_wait, wait);
	if( pgint_irq->irq_done )
	{
		mask = POLLIN | POLLRDNORM;
	}
	FELICA_LOG_DEBUG("%s END", __func__);

	return (mask);
}


/******************************************************************************
 * /dev/felica_uid
 ******************************************************************************/

/* character device definition */
static dev_t devid_felica_uid;
static struct cdev cdev_felica_uid;
static struct file_operations fops_felica_uid = {
	.owner				= THIS_MODULE,
	.open				= felica_uid_open,
	.release			= felica_uid_close,
	.unlocked_ioctl		= felica_uid_ioctl,
};

/*
 * initialize device
 */
void felica_uid_init(void)
{
	int ret;
	struct device *device_felica_uid;
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

#if 1
	#define FELICA_MAJOR_UID				98
	devid_felica_uid = MKDEV(FELICA_MAJOR_UID, FELICA_MINOR);
	ret = register_chrdev_region(devid_felica_uid, FELICA_MINOR_COUNT, FELICA_UID_NAME);
#else
	devid_felica_uid = MKDEV(FELICA_MAJOR, FELICA_MINOR);
	ret = alloc_chrdev_region(&devid_felica_uid, FELICA_BASEMINOR, FELICA_MINOR_COUNT, FELICA_UID_NAME);
#endif

	if( ret < 0 )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(alloc_chrdev_region), ret=[%d]", __func__, ret);
		return;
	}

	cdev_init(&cdev_felica_uid, &fops_felica_uid);
	ret = cdev_add(&cdev_felica_uid, devid_felica_uid, FELICA_MINOR_COUNT);
	if( ret < 0 )
	{
		unregister_chrdev_region(devid_felica_uid, FELICA_MINOR_COUNT);
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(cdev_add), ret=[%d]", __func__, ret);
		return;
	}

	device_felica_uid = device_create(felica_class, NULL, devid_felica_uid, NULL, FELICA_UID_NAME);
	if( IS_ERR(device_felica_uid) )
	{
		cdev_del(&cdev_felica_uid);
		unregister_chrdev_region(devid_felica_uid, FELICA_MINOR_COUNT);
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(device_create)", __func__);
		return;
	}

	memset( gdiag_name, 0x00, DIAG_NAME_MAXSIZE+1 );

	FELICA_LOG_DEBUG("[FELICA_DD] %s END, major=[%d], minor=[%d]", __func__, MAJOR(devid_felica_uid), MINOR(devid_felica_uid));
}

/*
 * finalize device
 */
void felica_uid_exit(void)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	device_destroy(felica_class, devid_felica_uid);
	cdev_del(&cdev_felica_uid);
	unregister_chrdev_region(devid_felica_uid, FELICA_MINOR_COUNT);

	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
}

/*
 * open device
 */
int felica_uid_open(struct inode *inode, struct file *file)
{
	char* cmdpos;
	static char cmdline[1025];
	static unsigned long start_adr,end_adr,leng ;

	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	start_adr = current->mm->arg_start;
	end_adr   = current->mm->arg_end;
	leng      = end_adr - start_adr;

	if( 1024 < leng)
	{
		leng = 1024;
	}

	cmdpos = (char*)(current->mm->arg_start);
	memcpy( cmdline,cmdpos ,leng );
	cmdline[leng] = '\0';

	if( ( strncmp(cmdline,gdiag_name,leng) != 0 ) &&
		( strncmp(cmdline,PROCESS_NAME_FELICA_DAEMON,leng) != 0 ) )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR, %s", __func__, cmdline);
		return -EACCES;
	}
	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return 0;
}

/*
 * close device
 */
int felica_uid_close(struct inode *inode, struct file *file)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);
	/* no operation */
	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return 0;
}

/*
 * uid registration
 */
long felica_uid_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START, cmd=[%d]", __func__, cmd);

	switch(cmd)
	{
		case SET_FELICA_UID_MFC:
			gmfc_uid = *((int*)arg);
			FELICA_LOG_INFO("[FELICA_DD] %s cmd:0x%x, set gmfc_uid:%d\n", __func__, cmd, gmfc_uid);
			break;
		case SET_FELICA_UID_RWM:
			grwm_uid = *((int*)arg);
			FELICA_LOG_INFO("[FELICA_DD] %s cmd:0x%x, set grwm_uid:%d\n", __func__, cmd, grwm_uid);
			break;
		case SET_FELICA_UID_MFL:
			gmfl_uid = *((int*)arg);
			FELICA_LOG_INFO("[FELICA_DD] %s cmd:0x%x, set gmfl_uid:%d\n", __func__, cmd, gmfl_uid);
			break;
		case SET_FELICA_UID_DTL:
			gdtl_uid = *((int*)arg);
			FELICA_LOG_INFO("[FELICA_DD] %s cmd:0x%x, set gdtl_uid:%d\n", __func__, cmd, gdtl_uid);
			break;
		case SET_FELICA_UID_DIAG:
			gdiag_uid = *((int*)arg);
			FELICA_LOG_INFO("[FELICA_DD] %s cmd:0x%x, set gdiag_uid:%d\n", __func__, cmd, gdiag_uid);
			break;
		case SET_FELICA_NAME_DIAG:
			if( !copy_from_user( gdiag_name, (char*)arg, DIAG_NAME_MAXSIZE ) )
			{
				gdiag_name[DIAG_NAME_MAXSIZE]='\0';
				FELICA_LOG_INFO("[FELICA_DD] %s cmd:0x%x, gdiag_name to [%s]\n", __func__, cmd, gdiag_name);
			}
			break;
		default:
			FELICA_LOG_ERR("[FELICA_DD] %s ERROR(unknown command)", __func__);
			break;
	}

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
	return 0;
}



/******************************************************************************
 * /dev/snfc_pon
 ******************************************************************************/

/* character device definition */
static dev_t devid_snfc_pon;
static struct cdev cdev_snfc_pon;
static struct file_operations fops_snfc_pon = {
	.owner		= THIS_MODULE,
	.open		= snfc_pon_open,
	.release	= snfc_pon_close,
	.read		= snfc_pon_read,
	.write		= snfc_pon_write,
};

/*
 * initialize device
 */
void snfc_pon_init(void)
{
	int ret;
	struct device *device_snfc_pon;
	SNFC_LOG_INFO("[SNFC_DD] %s START", __func__);

#if 1
	#define SNFC_MAJOR_PON					101
	devid_snfc_pon = MKDEV(SNFC_MAJOR_PON, SNFC_MINOR);
	ret = register_chrdev_region(devid_snfc_pon, SNFC_MINOR_COUNT, SNFC_PON_NAME);
#else
	devid_snfc_pon = MKDEV(SNFC_MAJOR, SNFC_MINOR);
	ret = alloc_chrdev_region(&devid_snfc_pon, SNFC_BASEMINOR, SNFC_MINOR_COUNT, SNFC_PON_NAME);
#endif

	if( ret < 0 )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(alloc_chrdev_region), ret=[%d]", __func__, ret);
		return;
	}

	cdev_init(&cdev_snfc_pon, &fops_snfc_pon);
	ret = cdev_add(&cdev_snfc_pon, devid_snfc_pon, SNFC_MINOR_COUNT);
	if( ret < 0 )
	{
		unregister_chrdev_region(devid_snfc_pon, SNFC_MINOR_COUNT);
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(cdev_add), ret=[%d]", __func__, ret);
		return;
	}

	device_snfc_pon = device_create(snfc_class, NULL, devid_snfc_pon, NULL, SNFC_PON_NAME);
	if( IS_ERR(device_snfc_pon) )
	{
		cdev_del(&cdev_snfc_pon);
		unregister_chrdev_region(devid_snfc_pon, SNFC_MINOR_COUNT);
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(device_create)", __func__);
		return;
	}

	SNFC_LOG_DEBUG("[SNFC_DD] %s END, major=[%d], minor=[%d]", __func__, MAJOR(devid_snfc_pon), MINOR(devid_snfc_pon));
}

/*
 * finalize device
 */
void snfc_pon_exit(void)
{
	SNFC_LOG_INFO("[SNFC_DD] %s START", __func__);

	device_destroy(snfc_class, devid_snfc_pon);
	cdev_del(&cdev_snfc_pon);
	unregister_chrdev_region(devid_snfc_pon, SNFC_MINOR_COUNT);

	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);
}

/*
 * open device
 */
int snfc_pon_open(struct inode *inode, struct file *file)
{
#ifdef SNFC_CONFIG_ACCESS_RESTRICTION
	uid_t uid;
#endif
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);

#ifdef SNFC_CONFIG_ACCESS_RESTRICTION
	uid = __task_cred(current)->uid;
	if( (uid != gmfc_uid) && (uid != gdtl_uid) &&
        (uid != gdiag_uid)  )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s END, uid=[%d], gmfc_uid=[%d], gdiag_uid=[%d], gdtl_uid=[%d]", __func__, uid, gmfc_uid, gdiag_uid, gdtl_uid);
		return -EACCES;
	}

#endif
	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return 0;
}

/*
 * close device
 */
int snfc_pon_close(struct inode *inode, struct file *file)
{
	uid_t uid;

	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);

	uid = __task_cred(current)->uid;
	if( uid == gdtl_uid  )
	{
		//if( felica_pdata->pon_gpio_func )
		//	felica_pdata->pon_gpio_func(GPIOWRITE, GPIO_VALUE_LOW, NULL);
		//else {
		//	SNFC_LOG_ERR("[SNFC_DD] %s felica_pdata->pon_gpio_func is NULL", __func__);
		//	return -EFAULT;
		//}
	}

	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return 0;
}

/*
 * read operation
 */
ssize_t snfc_pon_read(struct file *file, char __user *buf, size_t len, loff_t *ppos)
{
	int ret;
	char retparam;
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);

	if( felica_pdata->pon_gpio_func )
		felica_pdata->pon_gpio_func(GPIOREAD, 0, &ret);
	else {
		SNFC_LOG_ERR("[SNFC_DD] %s felica_pdata->pon_gpio_func is NULL", __func__);
		return -EFAULT;
	}

	if( ret == GPIO_VALUE_HIGH )
	{
		retparam = SNFC_PON_WIRED;
		SNFC_LOG_INFO("[SNFC_DD] %s pon is HIGH [start]", __func__);
	}
	else if( ret == GPIO_VALUE_LOW )
	{
		retparam = SNFC_PON_WIRELESS;
		SNFC_LOG_INFO("[SNFC_DD] %s pon is LOW [standby]", __func__);
	}
	else
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(gpio_get_value), ret=[%d]", __func__, ret);
		return -EIO;
	}

	ret = copy_to_user(buf, &retparam, SNFC_PON_DATA_LEN);
	if( ret != 0 )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(copy_to_user), ret=[%d]", __func__, ret);
		return -EFAULT;
	}
	*ppos += 1;

	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return SNFC_PON_DATA_LEN;
}

/*
 * write operation
 */
ssize_t snfc_pon_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	char pon;
	int ret;
	int setparam;
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);

	ret = copy_from_user(&pon, data, SNFC_PON_DATA_LEN);
	if( ret != 0 )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(copy_from_user), ret=[%d]", __func__, ret);
		return -EFAULT;
	}

	if( pon == SNFC_PON_WIRED )
	{
		setparam = GPIO_VALUE_HIGH;
		SNFC_LOG_INFO("[SNFC_DD] %s set pon to HIGH [start]", __func__);
	}
	else if( pon == SNFC_PON_WIRELESS )
	{
		setparam = GPIO_VALUE_LOW;
		SNFC_LOG_INFO("[SNFC_DD] %s set pon to LOW [standby]", __func__);
	}
	else
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(copy_from_user), pon=[%d]", __func__, pon);
		return -EINVAL;
	}

	if( felica_pdata->pon_gpio_func )
		felica_pdata->pon_gpio_func(GPIOWRITE, setparam, NULL);
	else {
		SNFC_LOG_ERR("[SNFC_DD] %s felica_pdata->pon_gpio_func is NULL", __func__);
		return -EFAULT;
	}

	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return SNFC_PON_DATA_LEN;
}

/******************************************************************************
 * /dev/snfc_cen
 ******************************************************************************/

/* character device definition */
static dev_t devid_snfc_cen;
static struct cdev cdev_snfc_cen;
static struct file_operations fops_snfc_cen = {
	.owner		= THIS_MODULE,
	.open		= snfc_cen_open,
	.release	= snfc_cen_close,
	.read		= snfc_cen_read,
};

/*
 * initialize device
 */
void snfc_cen_init(void)
{
	int ret;
	struct device *device_snfc_cen;
	SNFC_LOG_INFO("[SNFC_DD] %s START", __func__);

#if 1
	#define SNFC_MAJOR_CEN					102
	devid_snfc_cen = MKDEV(SNFC_MAJOR_CEN, SNFC_MINOR);
	ret = register_chrdev_region(devid_snfc_cen, SNFC_MINOR_COUNT, SNFC_CEN_NAME);
#else
	devid_snfc_cen = MKDEV(SNFC_MAJOR, SNFC_MINOR);
	ret = alloc_chrdev_region(&devid_snfc_cen, SNFC_BASEMINOR, SNFC_MINOR_COUNT, SNFC_CEN_NAME);
#endif

	if( ret < 0 )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(alloc_chrdev_region), ret=[%d]", __func__, ret);
		return;
	}

	cdev_init(&cdev_snfc_cen, &fops_snfc_cen);
	ret = cdev_add(&cdev_snfc_cen, devid_snfc_cen, SNFC_MINOR_COUNT);
	if( ret < 0 )
	{
		unregister_chrdev_region(devid_snfc_cen, SNFC_MINOR_COUNT);
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(cdev_add), ret=[%d]", __func__, ret);
		return;
	}

	device_snfc_cen = device_create(snfc_class, NULL, devid_snfc_cen, NULL, SNFC_CEN_NAME);
	if( IS_ERR(device_snfc_cen) )
	{
		cdev_del(&cdev_snfc_cen);
		unregister_chrdev_region(devid_snfc_cen, SNFC_MINOR_COUNT);
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(device_create)", __func__);
		return;
	}

	SNFC_LOG_DEBUG("[SNFC_DD] %s END, major=[%d], minor=[%d]", __func__, MAJOR(devid_snfc_cen), MINOR(devid_snfc_cen));
}

/*
 * finalize device
 */
void snfc_cen_exit(void)
{
	SNFC_LOG_INFO("[SNFC_DD] %s START", __func__);

	device_destroy(snfc_class, devid_snfc_cen);
	cdev_del(&cdev_snfc_cen);
	unregister_chrdev_region(devid_snfc_cen, SNFC_MINOR_COUNT);

	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);
}

/*
 * open device
 */
int snfc_cen_open(struct inode *inode, struct file *file)
{
#ifdef SNFC_CONFIG_ACCESS_RESTRICTION
	uid_t uid;
#endif
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);

#ifdef SNFC_CONFIG_ACCESS_RESTRICTION
	uid = __task_cred(current)->uid;

	if( (uid != gmfc_uid) && (uid != gdtl_uid) &&
		(uid != gdiag_uid)  )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s END, uid=[%d], gmfc_uid=[%d], gdiag_uid=[%d], gdtl_uid=[%d]", __func__, uid, gmfc_uid, gdiag_uid, gdtl_uid);
		return -EACCES;
	}

#endif
	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return 0;
}

/*
 * close device
 */
int snfc_cen_close(struct inode *inode, struct file *file)
{
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);
	/* no operation */
	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return 0;
}

/*
 * read operation
 */
ssize_t snfc_cen_read(struct file *file, char __user *buf, size_t len, loff_t *ppos)
{
	int ret;
	char retparam;
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);

	if( felica_pdata->cen_gpio_func )
		felica_pdata->cen_gpio_func(GPIOREAD, 0, &ret);
	else
		SNFC_LOG_ERR("[SNFC_DD] %s felica_pdata->cen_gpio_func is NULL", __func__);

	if( ret == SNFC_CEN_UNLOCK )
	{
		retparam = FELICA_CEN_UNLOCK;
		SNFC_LOG_INFO("[SNFC_DD] %s CEN [UnLock]", __func__);
	}
	else if( ret == SNFC_CEN_LOCK )
	{
		retparam = FELICA_CEN_LOCK;
		SNFC_LOG_INFO("[SNFC_DD] %s CEN [Lock]", __func__);
	}
	else
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(gpio_get_value), ret=[%d]", __func__, ret);
		return -EIO;
	}

	ret = copy_to_user(buf, &retparam, SNFC_CEN_DATA_LEN);
	if( ret != 0 )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(copy_to_user), ret=[%d]", __func__, ret);
		return -EFAULT;
	}
	*ppos += 1;

	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return SNFC_CEN_DATA_LEN;
}

/******************************************************************************
 * /dev/snfc_rfs
 ******************************************************************************/

/* character device definition */
static dev_t devid_snfc_rfs;
static struct cdev cdev_snfc_rfs;
static struct file_operations fops_snfc_rfs = {
	.owner		= THIS_MODULE,
	.open		= snfc_rfs_open,
	.release	= snfc_rfs_close,
	.read		= snfc_rfs_read,
};

/*
 * initialize device
 */
void snfc_rfs_init(void)
{
	int ret;
	struct device *device_snfc_rfs;
	SNFC_LOG_INFO("[SNFC_DD] %s START", __func__);

#if 1
	#define SNFC_MAJOR_RFS					103
	devid_snfc_rfs = MKDEV(SNFC_MAJOR_RFS, SNFC_MINOR);
	ret = register_chrdev_region(devid_snfc_rfs, SNFC_MINOR_COUNT, SNFC_RFS_NAME);
#else
	devid_snfc_rfs = MKDEV(SNFC_MAJOR, SNFC_MINOR);
	ret = alloc_chrdev_region(&devid_snfc_rfs, SNFC_BASEMINOR, SNFC_MINOR_COUNT, SNFC_RFS_NAME);
#endif

	if( ret < 0 )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(alloc_chrdev_region), ret=[%d]", __func__, ret);
		return;
	}

	cdev_init(&cdev_snfc_rfs, &fops_snfc_rfs);
	ret = cdev_add(&cdev_snfc_rfs, devid_snfc_rfs, SNFC_MINOR_COUNT);
	if( ret < 0 )
	{
		unregister_chrdev_region(devid_snfc_rfs, SNFC_MINOR_COUNT);
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(cdev_add), ret=[%d]", __func__, ret);
		return;
	}

	device_snfc_rfs = device_create(snfc_class, NULL, devid_snfc_rfs, NULL, SNFC_RFS_NAME);
	if( IS_ERR(device_snfc_rfs) )
	{
		cdev_del(&cdev_snfc_rfs);
		unregister_chrdev_region(devid_snfc_rfs, SNFC_MINOR_COUNT);
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(device_create)", __func__);
		return;
	}

	SNFC_LOG_DEBUG("[SNFC_DD] %s END, major=[%d], minor=[%d]", __func__, MAJOR(devid_snfc_rfs), MINOR(devid_snfc_rfs));
}

/*
 * finalize device
 */
void snfc_rfs_exit(void)
{
	SNFC_LOG_INFO("[SNFC_DD] %s START", __func__);

	device_destroy(snfc_class, devid_snfc_rfs);
	cdev_del(&cdev_snfc_rfs);
	unregister_chrdev_region(devid_snfc_rfs, SNFC_MINOR_COUNT);

	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);
}

/*
 * open device
 */
int snfc_rfs_open(struct inode *inode, struct file *file)
{
#ifdef SNFC_CONFIG_ACCESS_RESTRICTION
	uid_t uid;
#endif
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);

#ifdef SNFC_CONFIG_ACCESS_RESTRICTION
	uid = __task_cred(current)->uid;

	if( (uid != gmfc_uid) && (uid != gdtl_uid) &&
		(uid != gdiag_uid)  )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s END, uid=[%d], gmfc_uid=[%d], gdiag_uid=[%d], gdtl_uid=[%d]", __func__, uid, gmfc_uid, gdiag_uid, gdtl_uid);
		return -EACCES;
	}

#endif
	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return 0;
}

/*
 * close device
 */
int snfc_rfs_close(struct inode *inode, struct file *file)
{
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);
	/* no operation */
	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return 0;
}

/*
 * read operation
 */
ssize_t snfc_rfs_read(struct file *file, char __user *buf, size_t len, loff_t *ppos)
{
	int ret;
	char retparam;
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);

	if( felica_pdata->rfs_gpio_func )
		felica_pdata->rfs_gpio_func(GPIOREAD, 0, &ret);
	else {
		FELICA_LOG_ERR("[SNFC_DD] %s felica_pdata->rfs_gpio_func is NULL", __func__);
		return -EFAULT;
	}

	if( ret == GPIO_VALUE_HIGH )
	{
		retparam = SNFC_RFS_STANDBY;
		SNFC_LOG_DEBUG("[SNFC_DD] %s RFS is HIGH [standby]", __func__);
	}
	else if( ret == GPIO_VALUE_LOW )
	{
		retparam = SNFC_RFS_DETECTED;
		SNFC_LOG_DEBUG("[SNFC_DD] %s RFS is LOW [detected]", __func__);
	}
	else
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(gpio_get_value), ret=[%d]", __func__, ret);
		return -EIO;
	}

	ret = copy_to_user(buf, &retparam, SNFC_RFS_DATA_LEN);
	if( ret != 0 )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(copy_to_user), ret=[%d]", __func__, ret);
		return -EFAULT;
	}
	*ppos += 1;

	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return SNFC_RFS_DATA_LEN;
}

/******************************************************************************
 * /dev/snfc_intu
 ******************************************************************************/

/* character device definition */
static dev_t devid_snfc_intu;
static struct cdev cdev_snfc_intu;
static struct file_operations fops_snfc_intu = {
	.owner		= THIS_MODULE,
	.open		= snfc_intu_open,
	.release	= snfc_intu_close,
	.read		= snfc_intu_read,
};

/*
 * initialize device
 */
void snfc_intu_init(void)
{
	int ret;

	struct device *device_snfc_intu;
	SNFC_LOG_INFO("[SNFC_DD] %s START", __func__);

#if 1
	#define SNFC_MAJOR_INTU					104
	devid_snfc_intu = MKDEV(SNFC_MAJOR_INTU, SNFC_MINOR);
	ret = register_chrdev_region(devid_snfc_intu, SNFC_MINOR_COUNT, SNFC_INTU_NAME);
#else
	devid_snfc_intu = MKDEV(SNFC_MAJOR, SNFC_MINOR);
	ret = alloc_chrdev_region(&devid_snfc_intu, SNFC_BASEMINOR, SNFC_MINOR_COUNT, SNFC_INTU_NAME);
#endif
	if( ret < 0 )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(alloc_chrdev_region), ret=[%d]", __func__, ret);
		return;
	}

	cdev_init(&cdev_snfc_intu, &fops_snfc_intu);
	ret = cdev_add(&cdev_snfc_intu, devid_snfc_intu, SNFC_MINOR_COUNT);
	if( ret < 0 )
	{
		unregister_chrdev_region(devid_snfc_intu, SNFC_MINOR_COUNT);
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(cdev_add), ret=[%d]", __func__, ret);
		return;
	}

	device_snfc_intu = device_create(snfc_class, NULL, devid_snfc_intu, NULL, SNFC_INTU_NAME);
	if( IS_ERR(device_snfc_intu) )
	{
		cdev_del(&cdev_snfc_intu);
		unregister_chrdev_region(devid_snfc_intu, SNFC_MINOR_COUNT);
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(device_create)", __func__);
		return;
	}

	SNFC_LOG_DEBUG("[SNFC_DD] %s END, major=[%d], minor=[%d]", __func__, MAJOR(devid_snfc_intu), MINOR(devid_snfc_intu));
}

/*
 * finalize device
 */
void snfc_intu_exit(void)
{
	SNFC_LOG_INFO("[SNFC_DD] %s START", __func__);

	device_destroy(snfc_class, devid_snfc_intu);
	cdev_del(&cdev_snfc_intu);
	unregister_chrdev_region(devid_snfc_intu, SNFC_MINOR_COUNT);

	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
}

/*
 * open device
 */
int snfc_intu_open(struct inode *inode, struct file *file)
{
#ifdef SNFC_CONFIG_ACCESS_RESTRICTION
	uid_t uid;
#endif
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);

#ifdef SNFC_CONFIG_ACCESS_RESTRICTION
	uid = __task_cred(current)->uid;
	if( uid != gdiag_uid)
	{
		SNFC_LOG_ERR("[SNFC_DD] %s END, uid=[%d], gdiag_uid=[%d], ", __func__, uid, gdiag_uid);
		return -EACCES;
	}

#endif
	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return 0;
}

/*
 * close device
 */
int snfc_intu_close(struct inode *inode, struct file *file)
{
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);
	/* no operation */
	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return 0;
}

/*
 * read operation
 */
ssize_t snfc_intu_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	int ret;
	char retparam;
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);

	ret = gpio_get_value(snfc_intu_pin);

	if( ret == GPIO_VALUE_HIGH )
	{
		retparam = SNFC_INTU_HIGH;
		SNFC_LOG_INFO("[SNFC_DD] %s INTU-PIN is HIGH", __func__);
	}
	else if( ret == GPIO_VALUE_LOW )
	{
		retparam = SNFC_INTU_LOW;
		SNFC_LOG_INFO("[SNFC_DD] %s INTU-PIN is LOW", __func__);
	}
	else
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(gpio_get_value), ret=[%d]", __func__, ret);
		return -EIO;
	}

	ret = copy_to_user(buf, &retparam, SNFC_INTU_DATA_LEN);
	if( ret != 0 )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(copy_to_user), ret=[%d]", __func__, ret);
		return -EFAULT;
	}
	*ppos += 1;

	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return SNFC_INTU_DATA_LEN;
}

/******************************************************************************
 * /dev/snfc_intu_poll
 ******************************************************************************/

/* character device definition */
static dev_t devid_snfc_intu_poll;
static struct cdev cdev_snfc_intu_poll;
static struct file_operations fops_snfc_intu_poll = {
	.owner		= THIS_MODULE,
	.open		= snfc_intu_poll_open,
	.release	= snfc_intu_poll_close,
	.read		= snfc_intu_poll_read,
	.poll		= snfc_intu_poll_poll,
};

/*
 * top half of irq_handler
 */
irqreturn_t snfc_intu_irq_handler(int irq, void *dev_id)
{
	SNFC_LOG_INFO("[SNFC_DD] %s START", __func__);

	disable_irq_nosync(snfc_intu_irq);

	schedule_delayed_work(&pgintu_irq->work, msecs_to_jiffies(SNFC_INTU_DELAY_TIME));

	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return IRQ_HANDLED;
}

/*
 * bottom half of irq_handler
 */
void snfc_intu_irq_work(struct work_struct *work)
{
	SNFC_LOG_INFO("[SNFC_DD] %s START", __func__);

	enable_irq(snfc_intu_irq);

	pgintu_irq->irq_done = 1;
	wake_up_interruptible(&pgintu_irq->read_wait);

	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
}

/*
 * initialize device
 */
void snfc_intu_poll_init(void)
{
	int ret;

	struct device *device_snfc_intu_poll;
	SNFC_LOG_INFO("[SNFC_DD] %s START", __func__);

#if 1
	#define SNFC_MAJOR_INTU_POLL				105
	devid_snfc_intu_poll = MKDEV(SNFC_MAJOR_INTU_POLL, SNFC_MINOR);
	ret = register_chrdev_region(devid_snfc_intu_poll, SNFC_MINOR_COUNT, SNFC_INTU_POLL_NAME);
#else
	devid_snfc_intu_poll = MKDEV(SNFC_MAJOR, SNFC_MINOR);
	ret = alloc_chrdev_region(&devid_snfc_intu_poll, SNFC_BASEMINOR, SNFC_MINOR_COUNT, SNFC_INTU_POLL_NAME);
#endif
	if( ret < 0 )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(alloc_chrdev_region), ret=[%d]", __func__, ret);
		return;
	}

	cdev_init(&cdev_snfc_intu_poll, &fops_snfc_intu_poll);
	ret = cdev_add(&cdev_snfc_intu_poll, devid_snfc_intu_poll, SNFC_MINOR_COUNT);
	if( ret < 0 )
	{
		unregister_chrdev_region(devid_snfc_intu_poll, SNFC_MINOR_COUNT);
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(cdev_add), ret=[%d]", __func__, ret);
		return;
	}

	device_snfc_intu_poll = device_create(snfc_class, NULL, devid_snfc_intu_poll, NULL, SNFC_INTU_POLL_NAME);
	if( IS_ERR(device_snfc_intu_poll) )
	{
		cdev_del(&cdev_snfc_intu_poll);
		unregister_chrdev_region(devid_snfc_intu_poll, SNFC_MINOR_COUNT);
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(device_create)", __func__);
		return;
	}

	memset(pgintu_irq, 0x00, sizeof(struct snfc_intu_irqdata));
	INIT_DELAYED_WORK(&pgintu_irq->work, snfc_intu_irq_work);
	init_waitqueue_head(&pgintu_irq->read_wait);
	ret = request_irq(	snfc_intu_irq,
				snfc_intu_irq_handler,
				IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
				SNFC_INTU_POLL_NAME,
				(void*)pgintu_irq);
	if( ret != 0 )
	{
		device_destroy(snfc_class, devid_snfc_intu_poll);
		cdev_del(&cdev_snfc_intu_poll);
		unregister_chrdev_region(devid_snfc_intu_poll, SNFC_MINOR_COUNT);
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(request_irq), ret=[%d]", __func__, ret);
		return;
	}

	ret = enable_irq_wake(snfc_intu_irq);

	if( ret < 0 )
	{
		free_irq(snfc_intu_irq, (void*)pgintu_irq);
		device_destroy(snfc_class, devid_snfc_intu_poll);
		cdev_del(&cdev_snfc_intu_poll);
		unregister_chrdev_region(devid_snfc_intu_poll, SNFC_MINOR_COUNT);
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(enable_irq_wake), ret=[%d]", __func__, ret);
		return;
	}

	pgintu_irq->irq_done = 0;

	SNFC_LOG_DEBUG("[SNFC_DD] %s END, major=[%d], minor=[%d]", __func__, MAJOR(devid_snfc_intu_poll), MINOR(devid_snfc_intu_poll));
}

/*
 * finalize device
 */
void snfc_intu_poll_exit(void)
{
	SNFC_LOG_INFO("[SNFC_DD] %s START", __func__);

	disable_irq(snfc_intu_irq);

	free_irq(snfc_intu_irq, (void*)pgintu_irq);

	device_destroy(snfc_class, devid_snfc_intu_poll);
	cdev_del(&cdev_snfc_intu_poll);
	unregister_chrdev_region(devid_snfc_intu_poll, SNFC_MINOR_COUNT);

	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
}

/*
 * open device
 */
int snfc_intu_poll_open(struct inode *inode, struct file *file)
{
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);
	/* no operation */
	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return 0;
}

/*
 * close device
 */
int snfc_intu_poll_close(struct inode *inode, struct file *file)
{
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);
	/* no operation */
	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return 0;
}

/*
 * read operation
 */
ssize_t snfc_intu_poll_read(struct file *file, char __user * buf, size_t len, loff_t * ppos)
{
	int ret;
	char retparam;
	SNFC_LOG_INFO("[SNFC_DD] %s START", __func__);

	if( !pgintu_irq->irq_done )
	{
		ret = wait_event_interruptible(pgintu_irq->read_wait, pgintu_irq->irq_done == 1);
		if( ret < 0 )
		{
			SNFC_LOG_WARN("[SNFC_DD] %s warn(wait_event_interruptible), ret=[%d], pgintu_irq->irq_done=[%x]", __func__, ret, pgintu_irq->irq_done);
			return -EINTR;
		}
	}

	ret = gpio_get_value(snfc_intu_pin);

	if( ret == GPIO_VALUE_HIGH )
	{
		retparam = SNFC_INTU_HIGH;
		SNFC_LOG_INFO("[SNFC_DD] %s INTU-PIN is HIGH", __func__);
	}
	else if( ret == GPIO_VALUE_LOW )
	{
		retparam = SNFC_INTU_LOW;
		SNFC_LOG_INFO("[SNFC_DD] %s INTU-PIN is LOW", __func__);
	}
	else
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(gpio_get_value), ret=[%d]", __func__, ret);
		return -EIO;
	}

	ret = copy_to_user(buf, &retparam, SNFC_INTU_DATA_LEN);
	if( ret != 0 )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(copy_to_user), ret=[%d]", __func__, ret);
		return -EFAULT;
	}
	*ppos += 1;

	pgintu_irq->irq_done = 0;

	SNFC_LOG_INFO("[SNFC_DD] %s END", __func__);
	return SNFC_INTU_DATA_LEN;
}

/*
 * poll operation
 */
unsigned int snfc_intu_poll_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;

	SNFC_LOG_DEBUG("%s START", __func__);

	poll_wait(file, &pgintu_irq->read_wait, wait);
	if( pgintu_irq->irq_done )
	{
		mask = POLLIN | POLLRDNORM;
	}
	SNFC_LOG_DEBUG("%s END", __func__);

	return (mask);
}

/******************************************************************************
 * /dev/snfc_auto_polling
 ******************************************************************************/

/* character device definition */
static dev_t devid_snfc_auto_polling;
static struct cdev cdev_snfc_auto_polling;
static struct file_operations fops_snfc_auto_polling = {
	.owner		= THIS_MODULE,
	.open		= snfc_auto_polling_open,
	.release	= snfc_auto_polling_close,
	.read		= snfc_auto_polling_read,
};

#define AUTO_POLLING_CHECK_DELAY 	10
static struct snfc_auto_polling *auto_polling;

int snfc_auto_polling_check_func(void)
{
	int cen_status = 0;
	int rfs_status = 0;
	if( felica_pdata->cen_gpio_func )
		felica_pdata->cen_gpio_func(GPIOREAD, 0, &cen_status);
	else
		SNFC_LOG_ERR("[SNFC_DD] %s felica_pdata->cen_gpio_func is NULL", __func__);

	if( felica_pdata->rfs_gpio_func )
			felica_pdata->rfs_gpio_func(GPIOREAD, 0, &rfs_status);
	else
		SNFC_LOG_ERR("[SNFC_DD] %s felica_pdata->rfs_gpio_func is NULL", __func__);

	if( GPIO_VALUE_HIGH == rfs_status && FELICA_CEN_UNLOCK == cen_status && get_felica_uart_status() == 0)
	{
		SNFC_LOG_DEBUG("[SNFC_DD] %s Done", __func__);
		return 1;
	}

	return 0;
}

static void snfc_auto_polling_work_func(struct work_struct *dummy)
{

	if( snfc_auto_polling_check_func() == 1)
	{
		auto_polling->auto_polling_done = 1;
		SNFC_LOG_DEBUG("[SNFC_DD] %s Done[%d] ", __func__, auto_polling->auto_polling_done );
		wake_up_interruptible(&auto_polling->auto_polling_wait);
	}
	else {
		auto_polling->auto_polling_done = 0;
		schedule_delayed_work(&auto_polling->snfc_auto_polling_work, msecs_to_jiffies(AUTO_POLLING_CHECK_DELAY));
	}
}

/*
 * initialize device
 */
void snfc_auto_polling_init(void)
{
	int ret;
	struct device *device_snfc_auto_polling;
	SNFC_LOG_INFO("[SNFC_DD] %s START", __func__);

#if 1
	#define SNFC_MAJOR_AUTO_POLLING					106
	devid_snfc_auto_polling = MKDEV(SNFC_MAJOR_AUTO_POLLING, SNFC_MINOR);
	ret = register_chrdev_region(devid_snfc_auto_polling, SNFC_MINOR_COUNT, SNFC_AUTO_POLLING_NAME);
#else
	devid_snfc_auto_polling = MKDEV(SNFC_MAJOR, SNFC_MINOR);
	ret = alloc_chrdev_region(&devid_snfc_auto_polling, SNFC_BASEMINOR, SNFC_MINOR_COUNT, SNFC_AUTO_POLLING_NAME);
#endif

	if( ret < 0 )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(alloc_chrdev_region), ret=[%d]", __func__, ret);
		return;
	}

	auto_polling = kzalloc(sizeof(*auto_polling), GFP_KERNEL);
	if (!auto_polling) {
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(Cannot allocate device data)", __func__);
		return;
	}
	init_waitqueue_head(&auto_polling->auto_polling_wait);
	INIT_DELAYED_WORK(&auto_polling->snfc_auto_polling_work, snfc_auto_polling_work_func);

	cdev_init(&cdev_snfc_auto_polling, &fops_snfc_auto_polling);
	ret = cdev_add(&cdev_snfc_auto_polling, devid_snfc_auto_polling, SNFC_MINOR_COUNT);
	if( ret < 0 )
	{
		unregister_chrdev_region(devid_snfc_auto_polling, SNFC_MINOR_COUNT);
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(cdev_add), ret=[%d]", __func__, ret);
		goto snfc_auto_polling_free;
	}

	device_snfc_auto_polling = device_create(snfc_class, NULL, devid_snfc_auto_polling, NULL, SNFC_AUTO_POLLING_NAME);
	if( IS_ERR(device_snfc_auto_polling) )
	{
		cdev_del(&cdev_snfc_auto_polling);
		unregister_chrdev_region(devid_snfc_auto_polling, SNFC_MINOR_COUNT);
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(device_create)", __func__);
		goto snfc_auto_polling_free;
	}

	SNFC_LOG_DEBUG("[SNFC_DD] %s END, major=[%d], minor=[%d]", __func__, MAJOR(devid_snfc_auto_polling), MINOR(devid_snfc_auto_polling));
	return;

snfc_auto_polling_free:
	kfree(auto_polling);
}

/*
 * finalize device
 */
void snfc_auto_polling_exit(void)
{
	SNFC_LOG_INFO("[SNFC_DD] %s START", __func__);

	device_destroy(snfc_class, devid_snfc_auto_polling);
	cdev_del(&cdev_snfc_auto_polling);
	unregister_chrdev_region(devid_snfc_auto_polling, SNFC_MINOR_COUNT);

	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);
}

/*
 * open device
 */
int snfc_auto_polling_open(struct inode *inode, struct file *file)
{
#ifdef SNFC_CONFIG_ACCESS_RESTRICTION
	uid_t uid;
#endif
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);

#ifdef SNFC_CONFIG_ACCESS_RESTRICTION
	uid = __task_cred(current)->uid;

	if( (uid != gmfc_uid) && (uid != gdtl_uid) &&
		(uid != gdiag_uid)  )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s END, uid=[%d], gmfc_uid=[%d], gdiag_uid=[%d], gdtl_uid=[%d]", __func__, uid, gmfc_uid, gdiag_uid, gdtl_uid);
		return -EACCES;
	}

#endif
	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return 0;
}

/*
 * close device
 */
int snfc_auto_polling_close(struct inode *inode, struct file *file)
{
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);
	/* no operation */
	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return 0;
}

/*
 * read operation
 */
ssize_t snfc_auto_polling_read(struct file *file, char __user *buf, size_t len, loff_t *ppos)
{
	int ret;
	char retparam;
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);

	if( snfc_auto_polling_check_func() == 1)
		retparam = 1;
	else {
		auto_polling->auto_polling_done = 0;
		schedule_delayed_work(&auto_polling->snfc_auto_polling_work, msecs_to_jiffies(AUTO_POLLING_CHECK_DELAY));
		ret = wait_event_interruptible(auto_polling->auto_polling_wait, auto_polling->auto_polling_done == 1);
		if( ret < 0 )
		{
			retparam = 0;
			cancel_delayed_work_sync(&auto_polling->snfc_auto_polling_work);
			SNFC_LOG_WARN("[SNFC_DD] %s warn(wait_event_interruptible), ret=[%d], auto_polling->auto_polling_done=[%x]", __func__, ret, auto_polling->auto_polling_done);
			return -EINTR;
		}
		else
			retparam = 1;
	}

	ret = copy_to_user(buf, &retparam, SNFC_AUTO_POLLING_DATA_LEN);
	if( ret != 0 )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(copy_to_user), ret=[%d]", __func__, ret);
		return -EFAULT;
	}
	*ppos += 1;

	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return SNFC_AUTO_POLLING_DATA_LEN;
}

/******************************************************************************
 * /dev/snfc_hsel
 ******************************************************************************/

/* character device definition */
static dev_t devid_snfc_hsel;
static struct cdev cdev_snfc_hsel;
static struct file_operations fops_snfc_hsel = {
	.owner		= THIS_MODULE,
	.open		= snfc_hsel_open,
	.release	= snfc_hsel_close,
	.read		= snfc_hsel_read,
	.write		= snfc_hsel_write,
};

/*
 * initialize device
 */
void snfc_hsel_init(void)
{
	int ret;
	struct device *device_snfc_hsel;
	SNFC_LOG_INFO("[SNFC_DD] %s START", __func__);

#if 1
	#define SNFC_MAJOR_HSEL					107
	devid_snfc_hsel = MKDEV(SNFC_MAJOR_HSEL, SNFC_MINOR);
	ret = register_chrdev_region(devid_snfc_hsel, SNFC_MINOR_COUNT, SNFC_HSEL_NAME);
#else
	devid_snfc_hsel = MKDEV(SNFC_MAJOR, SNFC_MINOR);
	ret = alloc_chrdev_region(&devid_snfc_hsel, SNFC_BASEMINOR, SNFC_MINOR_COUNT, SNFC_HSEL_NAME);
#endif

	if( ret < 0 )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(alloc_chrdev_region), ret=[%d]", __func__, ret);
		return;
	}

	cdev_init(&cdev_snfc_hsel, &fops_snfc_hsel);
	ret = cdev_add(&cdev_snfc_hsel, devid_snfc_hsel, SNFC_MINOR_COUNT);
	if( ret < 0 )
	{
		unregister_chrdev_region(devid_snfc_hsel, SNFC_MINOR_COUNT);
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(cdev_add), ret=[%d]", __func__, ret);
		return;
	}

	device_snfc_hsel = device_create(snfc_class, NULL, devid_snfc_hsel, NULL, SNFC_HSEL_NAME);
	if( IS_ERR(device_snfc_hsel) )
	{
		cdev_del(&cdev_snfc_hsel);
		unregister_chrdev_region(devid_snfc_hsel, SNFC_MINOR_COUNT);
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(device_create)", __func__);
		return;
	}

	SNFC_LOG_DEBUG("[SNFC_DD] %s END, major=[%d], minor=[%d]", __func__, MAJOR(devid_snfc_hsel), MINOR(devid_snfc_hsel));
}

/*
 * finalize device
 */
void snfc_hsel_exit(void)
{
	SNFC_LOG_INFO("[SNFC_DD] %s START", __func__);

	device_destroy(snfc_class, devid_snfc_hsel);
	cdev_del(&cdev_snfc_hsel);
	unregister_chrdev_region(devid_snfc_hsel, SNFC_MINOR_COUNT);

	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);
}

/*
 * open device
 */
int snfc_hsel_open(struct inode *inode, struct file *file)
{
#ifdef SNFC_CONFIG_ACCESS_RESTRICTION
	uid_t uid;
#endif
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);

#ifdef SNFC_CONFIG_ACCESS_RESTRICTION
	uid = __task_cred(current)->uid;
	if( (uid != gmfc_uid) && (uid != gdtl_uid) &&
        (uid != gdiag_uid)  )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s END, uid=[%d], gmfc_uid=[%d], gdiag_uid=[%d], gdtl_uid=[%d]", __func__, uid, gmfc_uid, gdiag_uid, gdtl_uid);
		return -EACCES;
	}

#endif
	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return 0;
}

/*
 * close device
 */
int snfc_hsel_close(struct inode *inode, struct file *file)
{
	uid_t uid;

	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);

	uid = __task_cred(current)->uid;
	if( uid == gdtl_uid  )
	{
		if( felica_pdata->hsel_gpio_func )
			felica_pdata->hsel_gpio_func(GPIOWRITE, GPIO_VALUE_LOW, NULL);
		else {
			FELICA_LOG_ERR("[SNFC_DD] %s felica_pdata->hsel_gpio_func is NULL", __func__);
			return -EFAULT;
		}
		SNFC_LOG_DEBUG("[SNFC_DD] %s set hsel to LOW [standby]", __func__);
	}

	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return 0;
}

/*
 * read operation
 */
ssize_t snfc_hsel_read(struct file *file, char __user *buf, size_t len, loff_t *ppos)
{
	int ret;
	char retparam;
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);

	if( felica_pdata->hsel_gpio_func )
		felica_pdata->hsel_gpio_func(GPIOREAD, 0, &ret);
	else
		FELICA_LOG_ERR("[FELICA_DD] %s felica_pdata->hsel_gpio_func is NULL", __func__);

	if( ret == GPIO_VALUE_HIGH )
	{
		retparam = SNFC_HSEL_WIRED;
		SNFC_LOG_INFO("[SNFC_DD] %s HSEL is HIGH [start]", __func__);
	}
	else if( ret == GPIO_VALUE_LOW )
	{
		retparam = SNFC_HSEL_WIRELESS;
		SNFC_LOG_INFO("[SNFC_DD] %s HSEL is LOW [standby]", __func__);
	}
	else
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(gpio_get_value), ret=[%d]", __func__, ret);
		return -EIO;
	}

	ret = copy_to_user(buf, &retparam, SNFC_HSEL_DATA_LEN);
	if( ret != 0 )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(copy_to_user), ret=[%d]", __func__, ret);
		return -EFAULT;
	}
	*ppos += 1;

	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return SNFC_HSEL_DATA_LEN;
}

/*
 * write operation
 */
ssize_t snfc_hsel_write(struct file *file, const char __user *data, size_t len, loff_t *ppos)
{
	char hsel;
	int ret;
	int setparam, rfs_status, need_delay_time;
	SNFC_LOG_DEBUG("[SNFC_DD] %s START", __func__);

	ret = copy_from_user(&hsel, data, SNFC_HSEL_DATA_LEN);
	if( ret != 0 )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(copy_from_user), ret=[%d]", __func__, ret);
		return -EFAULT;
	}

	mutex_lock(&uart_mutex);
	if( (hsel == SNFC_HSEL_WIRED) || (hsel == SNFC_HSEL_FOR_TARGET) || (hsel == SNFC_HSEL_FOR_INTU) )
	{
		need_delay_time = 1;
		if(hsel == SNFC_HSEL_FOR_INTU)
			need_delay_time = 0;
		else if(hsel == SNFC_HSEL_FOR_TARGET) {
			if( felica_pdata->rfs_gpio_func ) {
				felica_pdata->rfs_gpio_func(GPIOREAD, 0, &rfs_status);
				if( GPIO_VALUE_LOW == rfs_status )
					need_delay_time = 0;
			}
			else
				SNFC_LOG_ERR("[SNFC_DD] %s felica_pdata->rfs_gpio_func is NULL", __func__);
		}

		if( get_felica_uart_status() == 1 ) {
			SNFC_LOG_ERR("[SNFC_DD] %s felica is useing uart", __func__);
			goto snfc_hsel_write_error;
		}

		setparam = GPIO_VALUE_HIGH;
		if( felica_pdata->pon_gpio_func )
			felica_pdata->pon_gpio_func(GPIOWRITE, setparam, NULL);
		else {
			SNFC_LOG_ERR("[SNFC_DD] %s felica_pdata->pon_gpio_func is NULL", __func__);
			goto snfc_hsel_write_error;
		}

		if( felica_pdata->hsel_gpio_func )
			felica_pdata->hsel_gpio_func(GPIOWRITE, setparam, NULL);
		else {
			SNFC_LOG_ERR("[SNFC_DD] %s felica_pdata->hsel_gpio_func is NULL", __func__);
			goto snfc_hsel_write_error;
		}

		set_snfc_uart_status(1);

		if( need_delay_time == 1 )
			msleep(10);

		SNFC_LOG_DEBUG("[SNFC_DD] %s Set HSEL to HIGH [start], case:[%d], need_delay_time:[%d]", __func__, hsel, need_delay_time);
	}
	else if( hsel == SNFC_HSEL_WIRELESS )
	{
		setparam = GPIO_VALUE_LOW;
		if( felica_pdata->pon_gpio_func )
			felica_pdata->pon_gpio_func(GPIOWRITE, setparam, NULL);
		else {
			SNFC_LOG_ERR("[SNFC_DD] %s felica_pdata->pon_gpio_func is NULL", __func__);
			goto snfc_hsel_write_error;
		}

		if( felica_pdata->hsel_gpio_func )
			felica_pdata->hsel_gpio_func(GPIOWRITE, setparam, NULL);
		else {
			SNFC_LOG_ERR("[SNFC_DD] %s felica_pdata->hsel_gpio_func is NULL", __func__);
			goto snfc_hsel_write_error;
		}

		set_snfc_uart_status(0);

		SNFC_LOG_DEBUG("[SNFC_DD] %s Set HSEL to LOW [standby]", __func__);
	}
	else
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(copy_from_user), hsel=[%d]", __func__, hsel);
		goto snfc_hsel_write_error;
	}

	mutex_unlock(&uart_mutex);
	SNFC_LOG_DEBUG("[SNFC_DD] %s END", __func__);
	return SNFC_HSEL_DATA_LEN;

snfc_hsel_write_error:
	mutex_unlock(&uart_mutex);
	return -EINVAL;

}

/******************************************************************************
 * Mobile FeliCa device driver initialization / termination function
 ******************************************************************************/

/*
 * to set initial value to each terminal
 */
void felica_initialize_pin(void)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	if( felica_pdata->pon_gpio_func )
		felica_pdata->pon_gpio_func(GPIOWRITE, GPIO_VALUE_LOW, NULL);
	else
		FELICA_LOG_ERR("[FELICA_DD] %s felica_pdata->pon_gpio_func is NULL", __func__);

	if( felica_pdata->hsel_gpio_func )
		felica_pdata->hsel_gpio_func(GPIOWRITE, GPIO_VALUE_LOW, NULL);
	else
		FELICA_LOG_ERR("[FELICA_DD] %s felica_pdata->hsel_gpio_func is NULL", __func__);

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
}

/*
 * to set final value to each terminal
 */
void felica_finalize_pin(void)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	if( felica_pdata->pon_gpio_func )
		felica_pdata->pon_gpio_func(GPIOWRITE, GPIO_VALUE_LOW, NULL);
	else
		FELICA_LOG_ERR("[FELICA_DD] %s felica_pdata->pon_gpio_func is NULL", __func__);

	if( felica_pdata->hsel_gpio_func )
		felica_pdata->hsel_gpio_func(GPIOWRITE, GPIO_VALUE_LOW, NULL);
	else
		FELICA_LOG_ERR("[FELICA_DD] %s felica_pdata->hsel_gpio_func is NULL", __func__);

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
}

/*
 * device driver registration
 */
void felica_register_device(void)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	felica_uart_init();
	felica_pon_init();
	felica_cen_init();
	felica_rfs_init();
	felica_rws_init();
	felica_int_init();
	felica_int_poll_init();
	felica_uid_init();

	snfc_pon_init();
	snfc_cen_init();
	snfc_rfs_init();
	snfc_intu_init();
	snfc_intu_poll_init();
	snfc_auto_polling_init();
	snfc_hsel_init();

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
}

/*
 * device driver deregistration
 */
void felica_deregister_device(void)
{
	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	snfc_hsel_exit();
	snfc_auto_polling_exit();
	snfc_intu_poll_exit();
	snfc_intu_exit();
	snfc_rfs_exit();
	snfc_cen_exit();
	snfc_pon_exit();

	felica_uid_exit();
	felica_int_poll_exit();
	felica_int_exit();
	felica_rws_exit();
	felica_rfs_exit();
	felica_cen_exit();
	felica_pon_exit();
	felica_uart_exit();

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);
}

#ifdef CONFIG_PM
static int felica_sleep(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int felica_wakeup(struct platform_device *pdev)
{
	return 0;
}
#endif

static int felica_probe(struct platform_device *pdev)
{
	int ret;
	struct felica_platform_data *pdata;

	FELICA_LOG_INFO("[FELICA_DD] %s()+\n", __func__);

	if (pdev->dev.of_node) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL) {
			FELICA_LOG_ERR("[FELICA_DD] %s platform_data alloc memry fail", __func__);
			return -ENOMEM;
		}
		ret = felica_parse_dt(&pdev->dev, pdata);
	} else {
		pdata = pdev->dev.platform_data;
		if (pdata == NULL) {
			FELICA_LOG_ERR("[FELICA_DD] %s Fail, platform_data is NULL", __func__);
			return	-ENODEV;
		}
	}

	felica_pdata = pdata;

	felica_int_pin = pdata->int_gpio;
	felica_int_irq = pdata->int_irq;

	snfc_intu_pin = pdata->intu_gpio;
	snfc_intu_irq = pdata->intu_irq;

	if (pdata->setup_gpio != NULL) {
		FELICA_LOG_DEBUG("[FELICA_DD] %s, pdata->setup_gpio", __func__);
		pdata->setup_gpio();
	}

	felica_class = class_create(THIS_MODULE, "felica");
	if( IS_ERR(felica_class) )
	{
		FELICA_LOG_ERR("[FELICA_DD] %s ERROR(class_create)", __func__);
		return PTR_ERR(felica_class);
	}

	snfc_class = class_create(THIS_MODULE, "snfc");
	if( IS_ERR(snfc_class) )
	{
		SNFC_LOG_ERR("[SNFC_DD] %s ERROR(class_create)", __func__);
		return PTR_ERR(snfc_class);
	}
	mutex_init(&uart_mutex);
	gfelica_uart_status = 0;
	gsnfc_uart_status = 0;
	felica_initialize_pin();
	felica_register_device();
	felica_nl_init();
	// MFC UID registration
	schedule_delayed_work(&pgint_irq->work, msecs_to_jiffies(10));

	FELICA_LOG_INFO("[FELICA_DD] %s()-\n", __func__);

	return 0;
}

static int felica_remove(struct platform_device *pdev)
{
	struct felica_platform_data *pdata;

	pdata = pdev->dev.platform_data;

	FELICA_LOG_DEBUG("[FELICA_DD] %s START", __func__);

	felica_nl_exit();
	felica_deregister_device();
	felica_finalize_pin();
	class_destroy(felica_class);

	FELICA_LOG_DEBUG("[FELICA_DD] %s END", __func__);

	return 0;
}

static const struct of_device_id felica_match_table[] = {
	{ .compatible = "sony,cxd2235-felica"},
	{ },
};
static struct platform_driver felica_driver = {
	.probe		= felica_probe,
	.remove		= felica_remove,
	.driver		= {
		.name	= "felica",
		.of_match_table = felica_match_table,
	},
#if CONFIG_PM
	.suspend	= felica_sleep,
	.resume 	= felica_wakeup,
#endif
};

/*
 * The entry point for initialization module
 */
int __init felica_init(void)
{
	return platform_driver_register(&felica_driver);
}

/*
 * The entry point for the termination module
 */
void __exit felica_exit(void)
{
	platform_driver_unregister(&felica_driver);
}

module_init(felica_init);
module_exit(felica_exit);

MODULE_DESCRIPTION("felica_dd");
MODULE_LICENSE("GPL v2");

#endif/* CONFIG_FELICA_CXD2235_DD */
