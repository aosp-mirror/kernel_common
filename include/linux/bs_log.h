#ifndef __BS_LOG_H
#define __BS_LOG_H

#include <linux/kernel.h>

#define LOG_LEVEL_E 3
#define LOG_LEVEL_N 5
#define LOG_LEVEL_I 6
#define LOG_LEVEL_D 7

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_I
#endif

#ifndef MODULE_TAG
#define MODULE_TAG "<>"
#endif

#if (LOG_LEVEL >= LOG_LEVEL_E)
#define PERR(fmt, args...) printk(KERN_ERR "\n" "[E]" \
				KERN_ERR MODULE_TAG "<%s><%d>" fmt "\n",\
				__FUNCTION__, __LINE__, ##args)
#else
#define PERR(fmt, args...)
#endif 

#if (LOG_LEVEL >= LOG_LEVEL_N)
#define PNOTICE(fmt, args...) printk(KERN_NOTICE "\n" "[N]" \
				KERN_NOTICE MODULE_TAG "<%s><%d>" fmt "\n",\
				__FUNCTION__, __LINE__, ##args)
#else
#define PNOTICE(fmt, args...)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_I)
#define PINFO(fmt, args...) printk(KERN_INFO "\n" "[I]" \
				KERN_INFO MODULE_TAG "<%s><%d>" fmt "\n",\
				__FUNCTION__, __LINE__, ##args)
#else
#define PINFO(fmt, args...)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_D)
#define PDEBUG(fmt, args...) printk(KERN_DEBUG "\n" "[D]" \
				KERN_DEBUG MODULE_TAG "<%s><%d>" fmt "\n",\
				__FUNCTION__, __LINE__, ##args)
#else
#define PDEBUG(fmt, args...)
#endif

#endif
