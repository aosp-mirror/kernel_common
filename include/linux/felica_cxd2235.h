#ifndef _FELICA_CXD2235_H
#define _FELICA_CXD2235_H

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/mfd/pm8xxx/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/ioctl.h>
#include <linux/module.h>
#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>

#define  GPIOWRITE	0
#define  GPIOREAD	1
/******************************************************************************
 * log
 ******************************************************************************/

#define FELICA_DEBUG
#define SNFC_DEBUG


/******************************************************************************
 * log
 ******************************************************************************/

#ifdef FELICA_DEBUG
#define FELICA_LOG_DEBUG(fmt, args...) printk(KERN_INFO fmt,## args)
#else
#define FELICA_LOG_DEBUG(fmt, args...)
#endif
#define FELICA_LOG_ERR(fmt, args...) printk(KERN_ERR fmt,## args)

#define FELICA_LOG_INFO(fmt, args...) printk(KERN_INFO fmt,## args)

#define FELICA_LOG_WARN(fmt, args...) printk(KERN_WARNING fmt,## args)

#ifdef SNFC_DEBUG
#define SNFC_LOG_DEBUG(fmt, args...) printk(KERN_INFO fmt,## args)
#else
#define SNFC_LOG_DEBUG(fmt, args...)
#endif
#define SNFC_LOG_ERR(fmt, args...) printk(KERN_ERR fmt,## args)

#define SNFC_LOG_INFO(fmt, args...) printk(KERN_INFO fmt,## args)

#define SNFC_LOG_WARN(fmt, args...) printk(KERN_WARNING fmt,## args)

/******************************************************************************
 * config option
 ******************************************************************************/

//#define FELICA_CONFIG_ACCESS_RESTRICTION

struct felica_platform_data {
	unsigned int int_irq;
	unsigned int int_gpio;
	unsigned int intu_irq;
	unsigned int intu_gpio;
	void (*setup_gpio)(void);
	void (*sleep_gpio)(void);
	void (*wakeup_gpio)(void);
	void (*pon_gpio_func)(int rwtype, int wvalue, int *rvalue);
	void (*cen_dtyp_d_func)(int rwtype, int wvalue, int *rvalue);
	void (*cen_dtyp_cp_func)(int rwtype, int wvalue, int *rvalue);
	void (*cen_gpio_func)(int rwtype, int wvalue, int *rvalue);
	void (*rfs_gpio_func)(int rwtype, int wvalue, int *rvalue);
	void (*int_gpio_func)(int rwtype, int wvalue, int *rvalue);
	void (*con_gpio_func)(int rwtype, int wvalue, int *rvalue);
	void (*hsel_gpio_func)(int rwtype, int wvalue, int *rvalue);
};

/******************************************************************************
 * common
 ******************************************************************************/

/* constant definition */
#define FELICA_MAJOR					10 /* same value as MISC_MAJOR */
#define FELICA_MINOR					0
#define FELICA_BASEMINOR				0 /* first of the requested range of minor numbers */
#define FELICA_MINOR_COUNT				1 /* the number of minor numbers required */

#define SNFC_MAJOR					10 /* same value as MISC_MAJOR */
#define SNFC_MINOR					0
#define SNFC_BASEMINOR					0 /* first of the requested range of minor numbers */
#define SNFC_MINOR_COUNT					1 /* the number of minor numbers required */

#define GPIO_VALUE_HIGH					1
#define GPIO_VALUE_LOW					0

/* function prototype */
void felica_initialize_pin(void);
void felica_finalize_pin(void);
void felica_register_device(void);
void felica_deregister_device(void);
int __init felica_init(void);
void __exit felica_exit(void);

/******************************************************************************
 * /dev/felica
 ******************************************************************************/

/* constant definition */
#define FELICA_UART_NAME				"felica"
#define FELICA_NL_NETLINK_USER			31
#define FELICA_NL_SUCCESS				1
#define FELICA_NL_EFAILED				2
#define FELICA_NL_REQ_OPEN				0x01
#define FELICA_NL_REQ_CLOSE				0x02
#define FELICA_NL_REQ_READ				0x03
#define FELICA_NL_REQ_WRITE				0x04
#define FELICA_NL_REQ_AVAIABLE			0x05
#define FELICA_NL_REQ_SYNC				0x06
#define FELICA_NL_RESPONCE				0xFE
#define FELICA_NL_CONNECT_MSG			0xFF
#define FELICA_NL_MSG_DATA_SIZE			4096
#define FELICA_NL_MSG_SIZE				FELICA_NL_MSG_DATA_SIZE+4

/* function prototype */
void felica_uart_init(void);
void felica_uart_exit(void);
int felica_uart_open(struct inode *inode, struct file *file);
int felica_uart_close(struct inode *inode, struct file *file);
ssize_t felica_uart_read(struct file *file, char __user *buf, size_t len, loff_t *ppos);
ssize_t felica_uart_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);
int felica_uart_sync(struct file *file, loff_t start, loff_t end, int datasync);
long felica_uart_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
void felica_nl_init(void);
void felica_nl_exit(void);
void felica_nl_send_msg(int len);
void felica_nl_recv_msg(struct sk_buff *skb);
void felica_nl_wait_ret_msg(void);





/******************************************************************************
 * /dev/felica_pon
 ******************************************************************************/

/* constant definition */
#define FELICA_PON_NAME					"felica_pon"
#define GPIO_PINID_FELICA_PON			25	/* 54 */
#define FELICA_PON_DATA_LEN				1
#define FELICA_PON_WIRELESS				0
#define FELICA_PON_WIRED				1

/* function prototype */
void felica_pon_init(void);
void felica_pon_exit(void);
int felica_pon_open(struct inode *inode, struct file *file);
int felica_pon_close(struct inode *inode, struct file *file);
ssize_t felica_pon_read(struct file *file, char __user *buf, size_t len, loff_t *ppos);
ssize_t felica_pon_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);





/******************************************************************************
 * /dev/felica_cen
 ******************************************************************************/

/* constant definition */
#define FELICA_CEN_NAME					"felica_cen"
#define FELICA_CEN_STATUS_INIT			0
#define PM8921_GPIO_BASE				NR_GPIO_IRQS
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)	(pm_gpio - 1 + PM8921_GPIO_BASE)
#define GPIO_OUT_FELICA_DTYPE_D			PM8921_GPIO_PM_TO_SYS(9)
#define GPIO_OUT_FELICA_DTYPE_CP		PM8921_GPIO_PM_TO_SYS(10)
#define FELICA_CEN_DATA_LEN				1
#define FELICA_CEN_LOCK					0
#define FELICA_CEN_UNLOCK				1

/* function prototype */
void felica_cen_init(void);
void felica_cen_exit(void);
int felica_cen_open(struct inode *inode, struct file *file);
int felica_cen_close(struct inode *inode, struct file *file);
ssize_t felica_cen_read(struct file *file, char __user *buf, size_t len, loff_t *ppos);
ssize_t felica_cen_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);





/******************************************************************************
 * /dev/felica_rfs
 ******************************************************************************/

/* constant definition */
#define FELICA_RFS_NAME					"felica_rfs"
#define GPIO_PINID_FELICA_RFS			10	/* 55 */
#define FELICA_RFS_DATA_LEN				1
#define FELICA_RFS_STANDBY				0
#define FELICA_RFS_DETECTED				1

/* function prototype */
void felica_rfs_init(void);
void felica_rfs_exit(void);
int felica_rfs_open(struct inode *inode, struct file *file);
int felica_rfs_close(struct inode *inode, struct file *file);
ssize_t felica_rfs_read(struct file *file, char __user *buf, size_t len, loff_t *ppos);





/******************************************************************************
 * /dev/felica_rws
 ******************************************************************************/

/* constant definition */
#define FELICA_RWS_NAME					"felica_rws"
#define GPIO_PINID_FELICA_CON			13	/* 71 */
#define FELICA_RWS_DATA_LEN				1
#define FELICA_RW_STATUS_INIT			0
#define FELICA_RW_STATUS_ENABLE			0
#define FELICA_RW_STATUS_DISABLE		1

/* function prototype */
void felica_rws_init(void);
void felica_rws_exit(void);
int felica_rws_open(struct inode *inode, struct file *file);
int felica_rws_close(struct inode *inode, struct file *file);
ssize_t felica_rws_read(struct file *file, char __user *buf, size_t len, loff_t *ppos);
ssize_t felica_rws_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);





/******************************************************************************
 * /dev/felica_int
 ******************************************************************************/

/* constant definition */
#define FELICA_INT_NAME					"felica_int"
#define GPIO_PINID_FELICA_INT			24	/* 58 */
#define FELICA_INT_DATA_LEN				1
#define FELICA_INT_DELAY_TIME			3
#define FELICA_INT_LOW					0
#define FELICA_INT_HIGH					1

/* function prototype */
void felica_int_init(void);
void felica_int_exit(void);
int felica_int_open(struct inode *inode, struct file *file);
int felica_int_close(struct inode *inode, struct file *file);
ssize_t felica_int_read(struct file *file, char __user *buf, size_t len, loff_t *ppos);





/******************************************************************************
 * /dev/felica_int_poll
 ******************************************************************************/

/* constant definition */
#define FELICA_INT_POLL_NAME			"felica_int_poll"

/* function prototype */
irqreturn_t felica_int_irq_handler(int irq, void *dev_id);
void felica_int_irq_work(struct work_struct *work);
void felica_int_poll_init(void);
void felica_int_poll_exit(void);
int felica_int_poll_open(struct inode *inode, struct file *file);
int felica_int_poll_close(struct inode *inode, struct file *file);
ssize_t felica_int_poll_read(struct file *file, char __user *buf, size_t len, loff_t *ppos);
unsigned int felica_int_poll_poll(struct file *file, poll_table *wait);





/******************************************************************************
 * /dev/felica_uid
 ******************************************************************************/

/* constant definition */
#define FELICA_UID_NAME					"felica_uid"
#define DIAG_NAME_MAXSIZE				15
#define PROCESS_NAME_FELICA_DAEMON		"/system/bin/felica_daemon"
#define FELICA_MAGIC					0xF8
#define SET_FELICA_UID_MFC				_IOW(FELICA_MAGIC, 1,void *)
#define SET_FELICA_UID_RWM				_IOW(FELICA_MAGIC, 2,void *)
#define SET_FELICA_UID_MFL				_IOW(FELICA_MAGIC, 3,void *)
#define SET_FELICA_UID_DIAG				_IOW(FELICA_MAGIC, 4,void *)
#define SET_FELICA_NAME_DIAG			_IOW(FELICA_MAGIC, 5,void *)
#define SET_FELICA_UID_DTL				_IOW(FELICA_MAGIC, 6,void *)

/* function prototype */
void felica_uid_init(void);
void felica_uid_exit(void);
int felica_uid_open(struct inode *inode, struct file *file);
int felica_uid_close(struct inode *inode, struct file *file);
long felica_uid_ioctl(struct file *file, unsigned int cmd, unsigned long arg);





/******************************************************************************
 * /dev/snfc_pon
 ******************************************************************************/

/* constant definition */
#define SNFC_PON_NAME					"snfc_pon"
#define GPIO_PINID_SNFC_PON				25	/* 54 */
#define SNFC_PON_DATA_LEN				1
#define SNFC_PON_WIRELESS				0
#define SNFC_PON_WIRED					1

/* function prototype */
void snfc_pon_init(void);
void snfc_pon_exit(void);
int snfc_pon_open(struct inode *inode, struct file *file);
int snfc_pon_close(struct inode *inode, struct file *file);
ssize_t snfc_pon_read(struct file *file, char __user *buf, size_t len, loff_t *ppos);
ssize_t snfc_pon_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);





/******************************************************************************
 * /dev/snfc_cen
 ******************************************************************************/

/* constant definition */
#define SNFC_CEN_NAME					"snfc_cen"
#define GPIO_PINID_SNFC_CEN				10	/* 55 */
#define SNFC_CEN_DATA_LEN				1
#define SNFC_CEN_STANDBY				0
#define SNFC_CEN_DETECTED				1
#define SNFC_CEN_DATA_LEN				1
#define SNFC_CEN_LOCK					0
#define SNFC_CEN_UNLOCK					1

/* function prototype */
void snfc_cen_init(void);
void snfc_cen_exit(void);
int snfc_cen_open(struct inode *inode, struct file *file);
int snfc_cen_close(struct inode *inode, struct file *file);
ssize_t snfc_cen_read(struct file *file, char __user *buf, size_t len, loff_t *ppos);





/******************************************************************************
 * /dev/snfc_rfs
 ******************************************************************************/

/* constant definition */
#define SNFC_RFS_NAME					"snfc_rfs"
#define GPIO_PINID_SNFC_RFS				10	/* 55 */
#define SNFC_RFS_DATA_LEN				1
#define SNFC_RFS_STANDBY				0
#define SNFC_RFS_DETECTED				1
#define SNFC_RFS_DATA_LEN				1
#define SNFC_RFS_STANDBY				0
#define SNFC_RFS_DETECTED				1

/* function prototype */
void snfc_rfs_init(void);
void snfc_rfs_exit(void);
int snfc_rfs_open(struct inode *inode, struct file *file);
int snfc_rfs_close(struct inode *inode, struct file *file);
ssize_t snfc_rfs_read(struct file *file, char __user *buf, size_t len, loff_t *ppos);





/******************************************************************************
 * /dev/snfc_intu
 ******************************************************************************/

/* constant definition */
#define SNFC_INTU_NAME					"snfc_intu"
#define GPIO_PINID_SNFC_INTU			24	/* 58 */
#define SNFC_INTU_DATA_LEN				1
#define SNFC_INTU_DELAY_TIME			3
#define SNFC_INTU_LOW					0
#define SNFC_INTU_HIGH					1

/* function prototype */
void snfc_intu_init(void);
void snfc_intu_exit(void);
int snfc_intu_open(struct inode *inode, struct file *file);
int snfc_intu_close(struct inode *inode, struct file *file);
ssize_t snfc_intu_read(struct file *file, char __user *buf, size_t len, loff_t *ppos);





/******************************************************************************
 * /dev/snfc_intu_poll
 ******************************************************************************/

/* constant definition */
#define SNFC_INTU_POLL_NAME			"snfc_intu_polling"

/* function prototype */
irqreturn_t snfc_intu_irq_handler(int irq, void *dev_id);
void snfc_intu_irq_work(struct work_struct *work);
void snfc_intu_poll_init(void);
void snfc_intu_poll_exit(void);
int snfc_intu_poll_open(struct inode *inode, struct file *file);
int snfc_intu_poll_close(struct inode *inode, struct file *file);
ssize_t snfc_intu_poll_read(struct file *file, char __user *buf, size_t len, loff_t *ppos);
unsigned int snfc_intu_poll_poll(struct file *file, poll_table *wait);




/******************************************************************************
 * /dev/snfc_auto_polling
 ******************************************************************************/

/* constant definition */
#define SNFC_AUTO_POLLING_NAME					"snfc_auto_polling"
#define GPIO_PINID_SNFC_AUTO_POLLING				10	/* 55 */
#define SNFC_AUTO_POLLING_DATA_LEN				1
#define SNFC_AUTO_POLLING_STANDBY				0
#define SNFC_AUTO_POLLING_DETECTED				1

/* function prototype */
void snfc_auto_polling_init(void);
void snfc_auto_polling_exit(void);
int snfc_auto_polling_open(struct inode *inode, struct file *file);
int snfc_auto_polling_close(struct inode *inode, struct file *file);
ssize_t snfc_auto_polling_read(struct file *file, char __user *buf, size_t len, loff_t *ppos);





/******************************************************************************
 * /dev/snfc_hsel
 ******************************************************************************/

/* constant definition */
#define SNFC_HSEL_NAME					"snfc_hsel"
#define GPIO_PINID_SNFC_HSEL				25	/* 54 */
#define SNFC_HSEL_DATA_LEN				1
#define SNFC_HSEL_WIRELESS				0
#define SNFC_HSEL_WIRED					1
#define SNFC_HSEL_FOR_TARGET				3
#define SNFC_HSEL_FOR_INTU				4

/* function prototype */
void snfc_hsel_init(void);
void snfc_hsel_exit(void);
int snfc_hsel_open(struct inode *inode, struct file *file);
int snfc_hsel_close(struct inode *inode, struct file *file);
ssize_t snfc_hsel_read(struct file *file, char __user *buf, size_t len, loff_t *ppos);
ssize_t snfc_hsel_write(struct file *file, const char __user *data, size_t len, loff_t *ppos);





#endif /* _FELICA_CXD2235_H */
