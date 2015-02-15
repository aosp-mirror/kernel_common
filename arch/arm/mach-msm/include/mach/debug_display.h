/*
 *
 */
#ifndef __LOG_MESSAGE_H__
#define __LOG_MESSAGE_H__


#define PR_DISP_EMERG(fmt, args...) 		printk(KERN_EMERG "[DISP] "fmt, ##args);
#define PR_DISP_ALERT(fmt, args...) 		printk(KERN_ALERT "[DISP] "fmt, ##args);
#define PR_DISP_CRIT(fmt, args...) 		printk(KERN_CRIT "[DISP] "fmt, ##args);
#define PR_DISP_ERR(fmt, args...) 		printk(KERN_ERR "[DISP] "fmt, ##args);
#define PR_DISP_WARN(fmt, args...) 		printk(KERN_WARNING "[DISP] "fmt, ##args);
#define PR_DISP_NOTICE(fmt, args...) 	printk(KERN_NOTICE "[DISP] "fmt, ##args);
#define PR_DISP_INFO(fmt, args...) 		printk(KERN_INFO "[DISP] "fmt, ##args)
#define PR_DISP_CONT(fmt, args...) 		printk(KERN_CONT "[DISP] "fmt, ##args);
#define PR_DISP_DEBUG(fmt, args...) 		printk(KERN_DEBUG "[DISP] "fmt, ##args);

#endif /* __HDMI_COMMON_H__ */


