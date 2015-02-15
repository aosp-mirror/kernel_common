#ifndef _FELICA_H
#define _FELICA_H

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


/******************************************************************************
 * log
 ******************************************************************************/

//#define FELICA_DEBUG


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

/******************************************************************************
 * config option
 ******************************************************************************/

#define FELICA_CONFIG_ACCESS_RESTRICTION

struct felica_platform_data {
	unsigned int pon_gpio;
	unsigned int cen_dtyp_d;
	unsigned int cen_dtyp_cp;
	unsigned int cen_gpio;
	unsigned int rfs_gpio;
	unsigned int int_gpio;
	unsigned int con_gpio;
	void (*setup_gpio)(void);
	void (*sleep_gpio)(void);
	void (*wakeup_gpio)(void);
};

/******************************************************************************
 * common
 ******************************************************************************/

/* constant definition */
#define FELICA_MAJOR					10 /* same value as MISC_MAJOR */
#define FELICA_MINOR					0
#define FELICA_BASEMINOR				0 /* first of the requested range of minor numbers */
#define FELICA_MINOR_COUNT				1 /* the number of minor numbers required */
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
int felica_cen_dtype_gpio_config(void);





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


#endif /* _FELICA_H */
