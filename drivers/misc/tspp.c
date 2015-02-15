/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>        
#include <linux/kernel.h>        
#include <linux/err.h>           
#include <linux/list.h>          
#include <linux/cdev.h>
#include <linux/init.h>          
#include <linux/io.h>            
#include <linux/device.h>        
#include <linux/sched.h>         
#include <linux/pm_runtime.h>    
#include <linux/fs.h>
#include <linux/uaccess.h>       
#include <linux/slab.h>          
#include <linux/ioport.h>        
#include <linux/dma-mapping.h>   
#include <linux/dmapool.h>       
#include <linux/delay.h>         
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/poll.h>          
#include <linux/wait.h>          
#include <linux/tspp.h>          
#include <linux/bitops.h>        
#include <linux/regulator/consumer.h>
#include <mach/sps.h>            
#include <mach/gpio.h>
#include <linux/wakelock.h>      
#include <linux/timer.h>         
#include <linux/jiffies.h>       
#include <mach/dma.h>
#include <mach/msm_tspp.h>
#include <mach/rpm-regulator-smd.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/string.h>
#include <mach/msm_bus.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#define TSPP_TSIF_INSTANCES            2
#define TSPP_GPIOS_PER_TSIF            4
#define TSPP_FILTER_TABLES             3
#define TSPP_MAX_DEVICES               1
#define TSPP_NUM_CHANNELS              16
#define TSPP_NUM_PRIORITIES            16
#define TSPP_NUM_KEYS                  8
#define INVALID_CHANNEL                0xFFFFFFFF

#define TSPP_SPS_DESCRIPTOR_COUNT      (8 * 1024 - 1)
#define TSPP_PACKET_LENGTH             188
#define TSPP_MIN_BUFFER_SIZE           (TSPP_PACKET_LENGTH)

#define TSPP_MAX_BUFFER_SIZE           (32 * 1024 - 1)

#define TSPP_USE_DMA_POOL(buff_size)   ((buff_size) < PAGE_SIZE)

#define TSPP_NUM_BUFFERS               (TSPP_SPS_DESCRIPTOR_COUNT - 1)
#define TSPP_TSIF_DEFAULT_TIME_LIMIT   60
#define SPS_DESCRIPTOR_SIZE            8
#define MIN_ACCEPTABLE_BUFFER_COUNT    2
#define TSPP_DEBUG(msg...)

#define TSIF_STS_CTL_OFF               (0x0)
#define TSIF_TIME_LIMIT_OFF            (0x4)
#define TSIF_CLK_REF_OFF               (0x8)
#define TSIF_LPBK_FLAGS_OFF            (0xc)
#define TSIF_LPBK_DATA_OFF            (0x10)
#define TSIF_TEST_CTL_OFF             (0x14)
#define TSIF_TEST_MODE_OFF            (0x18)
#define TSIF_TEST_RESET_OFF           (0x1c)
#define TSIF_TEST_EXPORT_OFF          (0x20)
#define TSIF_TEST_CURRENT_OFF         (0x24)

#define TSIF_DATA_PORT_OFF            (0x100)

#define TSIF_STS_CTL_EN_IRQ       BIT(28)
#define TSIF_STS_CTL_PACK_AVAIL   BIT(27)
#define TSIF_STS_CTL_1ST_PACKET   BIT(26)
#define TSIF_STS_CTL_OVERFLOW     BIT(25)
#define TSIF_STS_CTL_LOST_SYNC    BIT(24)
#define TSIF_STS_CTL_TIMEOUT      BIT(23)
#define TSIF_STS_CTL_INV_SYNC     BIT(21)
#define TSIF_STS_CTL_INV_NULL     BIT(20)
#define TSIF_STS_CTL_INV_ERROR    BIT(19)
#define TSIF_STS_CTL_INV_ENABLE   BIT(18)
#define TSIF_STS_CTL_INV_DATA     BIT(17)
#define TSIF_STS_CTL_INV_CLOCK    BIT(16)
#define TSIF_STS_CTL_SPARE        BIT(15)
#define TSIF_STS_CTL_EN_NULL      BIT(11)
#define TSIF_STS_CTL_EN_ERROR     BIT(10)
#define TSIF_STS_CTL_LAST_BIT     BIT(9)
#define TSIF_STS_CTL_EN_TIME_LIM  BIT(8)
#define TSIF_STS_CTL_EN_TCR       BIT(7)
#define TSIF_STS_CTL_TEST_MODE    BIT(6)
#define TSIF_STS_CTL_MODE_2       BIT(5)
#define TSIF_STS_CTL_EN_DM        BIT(4)
#define TSIF_STS_CTL_STOP         BIT(3)
#define TSIF_STS_CTL_START        BIT(0)

#define TSPP_RST			0x00
#define TSPP_CLK_CONTROL		0x04
#define TSPP_CONFIG			0x08
#define TSPP_CONTROL			0x0C
#define TSPP_PS_DISABLE			0x10
#define TSPP_MSG_IRQ_STATUS		0x14
#define TSPP_MSG_IRQ_MASK		0x18
#define TSPP_IRQ_STATUS			0x1C
#define TSPP_IRQ_MASK			0x20
#define TSPP_IRQ_CLEAR			0x24
#define TSPP_PIPE_ERROR_STATUS(_n)	(0x28 + (_n << 2))
#define TSPP_STATUS			0x68
#define TSPP_CURR_TSP_HEADER		0x6C
#define TSPP_CURR_PID_FILTER		0x70
#define TSPP_SYSTEM_KEY(_n)		(0x74 + (_n << 2))
#define TSPP_CBC_INIT_VAL(_n)		(0x94 + (_n << 2))
#define TSPP_DATA_KEY_RESET		0x9C
#define TSPP_KEY_VALID			0xA0
#define TSPP_KEY_ERROR			0xA4
#define TSPP_TEST_CTRL			0xA8
#define TSPP_VERSION			0xAC
#define TSPP_GENERICS			0xB0
#define TSPP_NOP			0xB4

#define TSPP_RST_RESET                    BIT(0)

#define TSPP_CLK_CONTROL_FORCE_CRYPTO     BIT(9)
#define TSPP_CLK_CONTROL_FORCE_PES_PL     BIT(8)
#define TSPP_CLK_CONTROL_FORCE_PES_AF     BIT(7)
#define TSPP_CLK_CONTROL_FORCE_RAW_CTRL   BIT(6)
#define TSPP_CLK_CONTROL_FORCE_PERF_CNT   BIT(5)
#define TSPP_CLK_CONTROL_FORCE_CTX_SEARCH BIT(4)
#define TSPP_CLK_CONTROL_FORCE_TSP_PROC   BIT(3)
#define TSPP_CLK_CONTROL_FORCE_CONS_AHB2MEM BIT(2)
#define TSPP_CLK_CONTROL_FORCE_TS_AHB2MEM BIT(1)
#define TSPP_CLK_CONTROL_SET_CLKON        BIT(0)

#define TSPP_CONFIG_SET_PACKET_LENGTH(_a, _b) (_a = (_a & 0xF0) | \
((_b & 0xF) << 8))
#define TSPP_CONFIG_GET_PACKET_LENGTH(_a) ((_a >> 8) & 0xF)
#define TSPP_CONFIG_DUP_WITH_DISC_EN		BIT(7)
#define TSPP_CONFIG_PES_SYNC_ERROR_MASK   BIT(6)
#define TSPP_CONFIG_PS_LEN_ERR_MASK       BIT(5)
#define TSPP_CONFIG_PS_CONT_ERR_UNSP_MASK BIT(4)
#define TSPP_CONFIG_PS_CONT_ERR_MASK      BIT(3)
#define TSPP_CONFIG_PS_DUP_TSP_MASK       BIT(2)
#define TSPP_CONFIG_TSP_ERR_IND_MASK      BIT(1)
#define TSPP_CONFIG_TSP_SYNC_ERR_MASK     BIT(0)

#define TSPP_CONTROL_PID_FILTER_LOCK      BIT(5)
#define TSPP_CONTROL_FORCE_KEY_CALC       BIT(4)
#define TSPP_CONTROL_TSP_CONS_SRC_DIS     BIT(3)
#define TSPP_CONTROL_TSP_TSIF1_SRC_DIS    BIT(2)
#define TSPP_CONTROL_TSP_TSIF0_SRC_DIS    BIT(1)
#define TSPP_CONTROL_PERF_COUNT_INIT      BIT(0)

#define TSPP_MSG_TSPP_IRQ                 BIT(2)
#define TSPP_MSG_TSIF_1_IRQ               BIT(1)
#define TSPP_MSG_TSIF_0_IRQ               BIT(0)

#define TSPP_IRQ_STATUS_TSP_RD_CMPL		BIT(19)
#define TSPP_IRQ_STATUS_KEY_ERROR		BIT(18)
#define TSPP_IRQ_STATUS_KEY_SWITCHED_BAD	BIT(17)
#define TSPP_IRQ_STATUS_KEY_SWITCHED		BIT(16)
#define TSPP_IRQ_STATUS_PS_BROKEN(_n)		BIT((_n))

#define TSPP_PIPE_PES_SYNC_ERROR		BIT(3)
#define TSPP_PIPE_PS_LENGTH_ERROR		BIT(2)
#define TSPP_PIPE_PS_CONTINUITY_ERROR		BIT(1)
#define TSPP_PIP_PS_LOST_START			BIT(0)

#define TSPP_STATUS_TSP_PKT_AVAIL		BIT(10)
#define TSPP_STATUS_TSIF1_DM_REQ		BIT(6)
#define TSPP_STATUS_TSIF0_DM_REQ		BIT(2)
#define TSPP_CURR_FILTER_TABLE			BIT(0)

#define TSPP_GENERICS_CRYPTO_GEN		BIT(12)
#define TSPP_GENERICS_MAX_CONS_PIPES		BIT(7)
#define TSPP_GENERICS_MAX_PIPES			BIT(2)
#define TSPP_GENERICS_TSIF_1_GEN		BIT(1)
#define TSPP_GENERICS_TSIF_0_GEN		BIT(0)

#define TSPP_PID_FILTER_TABLE0      0x800
#define TSPP_PID_FILTER_TABLE1      0x880
#define TSPP_PID_FILTER_TABLE2      0x900
#define TSPP_GLOBAL_PERFORMANCE     0x980 
#define TSPP_PIPE_CONTEXT           0x990 
#define TSPP_PIPE_PERFORMANCE       0x998 
#define TSPP_TSP_BUFF_WORD(_n)      (0xC10 + (_n << 2))
#define TSPP_DATA_KEY               0xCD0

struct debugfs_entry {
	const char *name;
	mode_t mode;
	int offset;
};

static const struct debugfs_entry debugfs_tsif_regs[] = {
	{"sts_ctl",             S_IRUGO | S_IWUSR, TSIF_STS_CTL_OFF},
	{"time_limit",          S_IRUGO | S_IWUSR, TSIF_TIME_LIMIT_OFF},
	{"clk_ref",             S_IRUGO | S_IWUSR, TSIF_CLK_REF_OFF},
	{"lpbk_flags",          S_IRUGO | S_IWUSR, TSIF_LPBK_FLAGS_OFF},
	{"lpbk_data",           S_IRUGO | S_IWUSR, TSIF_LPBK_DATA_OFF},
	{"test_ctl",            S_IRUGO | S_IWUSR, TSIF_TEST_CTL_OFF},
	{"test_mode",           S_IRUGO | S_IWUSR, TSIF_TEST_MODE_OFF},
	{"test_reset",                    S_IWUSR, TSIF_TEST_RESET_OFF},
	{"test_export",         S_IRUGO | S_IWUSR, TSIF_TEST_EXPORT_OFF},
	{"test_current",        S_IRUGO,           TSIF_TEST_CURRENT_OFF},
	{"data_port",           S_IRUSR,           TSIF_DATA_PORT_OFF},
};

static const struct debugfs_entry debugfs_tspp_regs[] = {
	{"rst",                 S_IRUGO | S_IWUSR, TSPP_RST},
	{"clk_control",         S_IRUGO | S_IWUSR, TSPP_CLK_CONTROL},
	{"config",              S_IRUGO | S_IWUSR, TSPP_CONFIG},
	{"control",             S_IRUGO | S_IWUSR, TSPP_CONTROL},
	{"ps_disable",          S_IRUGO | S_IWUSR, TSPP_PS_DISABLE},
	{"msg_irq_status",      S_IRUGO | S_IWUSR, TSPP_MSG_IRQ_STATUS},
	{"msg_irq_mask",        S_IRUGO | S_IWUSR, TSPP_MSG_IRQ_MASK},
	{"irq_status",          S_IRUGO | S_IWUSR, TSPP_IRQ_STATUS},
	{"irq_mask",            S_IRUGO | S_IWUSR, TSPP_IRQ_MASK},
	{"irq_clear",           S_IRUGO | S_IWUSR, TSPP_IRQ_CLEAR},
	
	{"status",              S_IRUGO | S_IWUSR, TSPP_STATUS},
	{"curr_tsp_header",     S_IRUGO | S_IWUSR, TSPP_CURR_TSP_HEADER},
	{"curr_pid_filter",     S_IRUGO | S_IWUSR, TSPP_CURR_PID_FILTER},
	
	
	{"data_key_reset",      S_IRUGO | S_IWUSR, TSPP_DATA_KEY_RESET},
	{"key_valid",           S_IRUGO | S_IWUSR, TSPP_KEY_VALID},
	{"key_error",           S_IRUGO | S_IWUSR, TSPP_KEY_ERROR},
	{"test_ctrl",           S_IRUGO | S_IWUSR, TSPP_TEST_CTRL},
	{"version",             S_IRUGO | S_IWUSR, TSPP_VERSION},
	{"generics",            S_IRUGO | S_IWUSR, TSPP_GENERICS},
	{"pid_filter_table0",   S_IRUGO | S_IWUSR, TSPP_PID_FILTER_TABLE0},
	{"pid_filter_table1",   S_IRUGO | S_IWUSR, TSPP_PID_FILTER_TABLE1},
	{"pid_filter_table2",   S_IRUGO | S_IWUSR, TSPP_PID_FILTER_TABLE2},
	{"global_performance",  S_IRUGO | S_IWUSR, TSPP_GLOBAL_PERFORMANCE},
	{"pipe_context",        S_IRUGO | S_IWUSR, TSPP_PIPE_CONTEXT},
	{"pipe_performance",    S_IRUGO | S_IWUSR, TSPP_PIPE_PERFORMANCE},
	{"data_key",            S_IRUGO | S_IWUSR, TSPP_DATA_KEY}
};

struct tspp_pid_filter {
	u32 filter;			
	u32 config;			
};

#define FILTER_HEADER_ERROR_MASK          BIT(7)
#define FILTER_TRANS_END_DISABLE          BIT(6)
#define FILTER_DEC_ON_ERROR_EN            BIT(5)
#define FILTER_DECRYPT                    BIT(4)
#define FILTER_HAS_ENCRYPTION(_p)         (_p->config & FILTER_DECRYPT)
#define FILTER_GET_PIPE_NUMBER0(_p)       (_p->config & 0xF)
#define FILTER_SET_PIPE_NUMBER0(_p, _b)   (_p->config = \
			(_p->config & ~0xF) | (_b & 0xF))
#define FILTER_GET_PIPE_PROCESS0(_p)      ((_p->filter >> 30) & 0x3)
#define FILTER_SET_PIPE_PROCESS0(_p, _b)  (_p->filter = \
			(_p->filter & ~(0x3<<30)) | ((_b & 0x3) << 30))
#define FILTER_GET_PIPE_PID(_p)           ((_p->filter >> 13) & 0x1FFF)
#define FILTER_SET_PIPE_PID(_p, _b)       (_p->filter = \
			(_p->filter & ~(0x1FFF<<13)) | ((_b & 0x1FFF) << 13))
#define FILTER_GET_PID_MASK(_p)           (_p->filter & 0x1FFF)
#define FILTER_SET_PID_MASK(_p, _b)       (_p->filter = \
			(_p->filter & ~0x1FFF) | (_b & 0x1FFF))
#define FILTER_GET_PIPE_PROCESS1(_p)      ((_p->config >> 30) & 0x3)
#define FILTER_SET_PIPE_PROCESS1(_p, _b)  (_p->config = \
			(_p->config & ~(0x3<<30)) | ((_b & 0x3) << 30))
#define FILTER_GET_KEY_NUMBER(_p)         ((_p->config >> 8) & 0x7)
#define FILTER_SET_KEY_NUMBER(_p, _b)     (_p->config = \
			(_p->config & ~(0x7<<8)) | ((_b & 0x7) << 8))

struct tspp_global_performance_regs {
	u32 tsp_total;
	u32 tsp_ignored;
	u32 tsp_error;
	u32 tsp_sync;
};

struct tspp_pipe_context_regs {
	u16 pes_bytes_left;
	u16 count;
	u32 tsif_suffix;
} __packed;
#define CONTEXT_GET_STATE(_a)					(_a & 0x3)
#define CONTEXT_UNSPEC_LENGTH					BIT(11)
#define CONTEXT_GET_CONT_COUNT(_a)			((_a >> 12) & 0xF)

#define MSEC_TO_JIFFIES(msec)			((msec) * HZ / 1000)

struct tspp_pipe_performance_regs {
	u32 tsp_total;
	u32 ps_duplicate_tsp;
	u32 tsp_no_payload;
	u32 tsp_broken_ps;
	u32 ps_total_num;
	u32 ps_continuity_error;
	u32 ps_length_error;
	u32 pes_sync_error;
};

struct tspp_tsif_device {
	void __iomem *base;
	u32 time_limit;
	u32 ref_count;
	enum tspp_tsif_mode mode;
	int clock_inverse;
	int data_inverse;
	int sync_inverse;
	int enable_inverse;
	u32 tsif_irq;

	
	struct dentry *dent_tsif;
	struct dentry *debugfs_tsif_regs[ARRAY_SIZE(debugfs_tsif_regs)];
	u32 stat_rx;
	u32 stat_overflow;
	u32 stat_lost_sync;
	u32 stat_timeout;
};

enum tspp_buf_state {
	TSPP_BUF_STATE_EMPTY,	
	TSPP_BUF_STATE_WAITING, 
	TSPP_BUF_STATE_DATA,    
	TSPP_BUF_STATE_LOCKED   
};

struct tspp_mem_buffer {
	struct tspp_mem_buffer *next;
	struct sps_mem_buffer sps;
	struct tspp_data_descriptor desc; 
	enum tspp_buf_state state;
	size_t filled;          
	int read_index;         
};

struct tspp_channel {
	struct cdev cdev;
	struct device *dd;
	struct tspp_device *pdev; 
	struct sps_pipe *pipe;
	struct sps_connect config;
	struct sps_register_event event;
	struct tspp_mem_buffer *data;    
	struct tspp_mem_buffer *read;    
	struct tspp_mem_buffer *waiting; 
	struct tspp_mem_buffer *locked;  
	wait_queue_head_t in_queue; 
	u32 id;           
	int used;         
	int key;          
	u32 buffer_size;  
	u32 max_buffers;  
	u32 buffer_count; 
	u32 filter_count; 
	u32 int_freq;     
	enum tspp_source src;
	enum tspp_mode mode;
	tspp_notifier *notifier; 
	void *notify_data;       
	u32 expiration_period_ms; 
	struct timer_list expiration_timer;
	struct dma_pool *dma_pool;
	tspp_memfree *memfree;   
	void *user_info; 
};

struct tspp_pid_filter_table {
	struct tspp_pid_filter filter[TSPP_NUM_PRIORITIES];
};

struct tspp_key_entry {
	u32 even_lsb;
	u32 even_msb;
	u32 odd_lsb;
	u32 odd_msb;
};

struct tspp_key_table {
	struct tspp_key_entry entry[TSPP_NUM_KEYS];
};

struct tspp_device {
	struct list_head devlist; 
	struct platform_device *pdev;
	void __iomem *base;
	uint32_t tsif_bus_client;
	unsigned int tspp_irq;
	unsigned int bam_irq;
	u32 bam_handle;
	struct sps_bam_props bam_props;
	struct wake_lock wake_lock;
	spinlock_t spinlock;
	struct tasklet_struct tlet;
	struct tspp_tsif_device tsif[TSPP_TSIF_INSTANCES];
	
	struct clk *tsif_pclk;
	struct clk *tsif_ref_clk;
	
	struct regulator *tsif_vreg;
	
	struct tspp_pid_filter_table *filters[TSPP_FILTER_TABLES];
	struct tspp_channel channels[TSPP_NUM_CHANNELS];
	struct tspp_key_table *tspp_key_table;
	struct tspp_global_performance_regs *tspp_global_performance;
	struct tspp_pipe_context_regs *tspp_pipe_context;
	struct tspp_pipe_performance_regs *tspp_pipe_performance;
	bool req_irqs;

	struct dentry *dent;
	struct dentry *debugfs_regs[ARRAY_SIZE(debugfs_tspp_regs)];
#if defined(CONFIG_FB)
        struct notifier_block fb_notif;
#endif
};


static struct class *tspp_class;
static int tspp_key_entry;
static dev_t tspp_minor;  

static LIST_HEAD(tspp_devices);

static ssize_t tspp_read(struct file *, char __user *, size_t, loff_t *);
static ssize_t tspp_open(struct inode *inode, struct file *filp);
static unsigned int tspp_poll(struct file *filp, struct poll_table_struct *p);
static ssize_t tspp_release(struct inode *inode, struct file *filp);
static long tspp_ioctl(struct file *, unsigned int, unsigned long);

static const struct file_operations tspp_fops = {
	.owner   = THIS_MODULE,
	.read    = tspp_read,
	.open    = tspp_open,
	.poll    = tspp_poll,
	.release = tspp_release,
	.unlocked_ioctl   = tspp_ioctl,
};

static void enable_tspp_interrupt(struct tspp_device *tsppdev, int enable)
{
	if (enable)
	{
		
		enable_irq(tsppdev->tspp_irq);
		enable_irq(tsppdev->tsif[0].tsif_irq);
		enable_irq(tsppdev->tsif[1].tsif_irq);
		enable_irq(tsppdev->bam_irq);
		printk("%s, enable TSPP INTERRUPT\n", __func__);
	}else
	{
	        
		disable_irq(tsppdev->tspp_irq);
		disable_irq(tsppdev->tsif[0].tsif_irq);
		disable_irq(tsppdev->tsif[1].tsif_irq);
		disable_irq(tsppdev->bam_irq);
		printk("%s, disable TSPP INTERRUPT\n", __func__);
	}
}

static irqreturn_t tspp_isr(int irq, void *dev)
{
	struct tspp_device *device = dev;
	u32 status, mask;
	u32 data;

	status = readl_relaxed(device->base + TSPP_IRQ_STATUS);
	mask = readl_relaxed(device->base + TSPP_IRQ_MASK);
	status &= mask;

	if (!status) {
		dev_warn(&device->pdev->dev, "Spurious interrupt");
		return IRQ_NONE;
	}

	

	if (status & TSPP_IRQ_STATUS_KEY_ERROR) {
		
		data = readl_relaxed(device->base + TSPP_KEY_ERROR);
		dev_info(&device->pdev->dev, "key error 0x%x", data);
	}
	if (status & TSPP_IRQ_STATUS_KEY_SWITCHED_BAD) {
		data = readl_relaxed(device->base + TSPP_KEY_VALID);
		dev_info(&device->pdev->dev, "key invalidated: 0x%x", data);
	}
	if (status & TSPP_IRQ_STATUS_KEY_SWITCHED)
		dev_info(&device->pdev->dev, "key switched");

	if (status & 0xffff)
		dev_info(&device->pdev->dev, "broken pipe %i", status & 0xffff);

	writel_relaxed(status, device->base + TSPP_IRQ_CLEAR);

	wmb();
	return IRQ_HANDLED;
}

static irqreturn_t tsif_isr(int irq, void *dev)
{
	struct tspp_tsif_device *tsif_device = dev;
	u32 sts_ctl = ioread32(tsif_device->base + TSIF_STS_CTL_OFF);

	if (!(sts_ctl & (TSIF_STS_CTL_PACK_AVAIL |
			 TSIF_STS_CTL_OVERFLOW |
			 TSIF_STS_CTL_LOST_SYNC |
			 TSIF_STS_CTL_TIMEOUT)))
		return IRQ_NONE;

	if (sts_ctl & TSIF_STS_CTL_OVERFLOW)
		tsif_device->stat_overflow++;

	if (sts_ctl & TSIF_STS_CTL_LOST_SYNC)
		tsif_device->stat_lost_sync++;

	if (sts_ctl & TSIF_STS_CTL_TIMEOUT)
		tsif_device->stat_timeout++;

	iowrite32(sts_ctl, tsif_device->base + TSIF_STS_CTL_OFF);

	wmb();
	return IRQ_HANDLED;
}

static void tspp_sps_complete_cb(struct sps_event_notify *notify)
{
	struct tspp_device *pdev = notify->user;
	tasklet_schedule(&pdev->tlet);
}

static void tspp_expiration_timer(unsigned long data)
{
	struct tspp_device *pdev = (struct tspp_device *)data;

	if (pdev)
		tasklet_schedule(&pdev->tlet);
}

static void tspp_sps_complete_tlet(unsigned long data)
{
	int i;
	int complete;
	unsigned long flags;
	struct sps_iovec iovec;
	struct tspp_channel *channel;
	struct tspp_device *device = (struct tspp_device *)data;
	spin_lock_irqsave(&device->spinlock, flags);

	for (i = 0; i < TSPP_NUM_CHANNELS; i++) {
		complete = 0;
		channel = &device->channels[i];

		if (!channel->used || !channel->waiting)
			continue;

		
		if (channel->expiration_period_ms)
			del_timer(&channel->expiration_timer);

		
		while (channel->waiting->state == TSPP_BUF_STATE_WAITING) {
			if (sps_get_iovec(channel->pipe, &iovec) != 0) {
				pr_err("tspp: Error in iovec on channel %i",
					channel->id);
				break;
			}
			if (iovec.size == 0)
				break;

			if (iovec.addr != channel->waiting->sps.phys_base)
				pr_err("tspp: buffer mismatch %pa",
					&channel->waiting->sps.phys_base);

			complete = 1;
			channel->waiting->state = TSPP_BUF_STATE_DATA;
			channel->waiting->filled = iovec.size;
			channel->waiting->read_index = 0;

			if (channel->src == TSPP_SOURCE_TSIF0)
				device->tsif[0].stat_rx++;
			else if (channel->src == TSPP_SOURCE_TSIF1)
				device->tsif[1].stat_rx++;

			
			channel->waiting = channel->waiting->next;
		}

		
		if (complete) {
			wake_up_interruptible(&channel->in_queue);

			
			if (channel->notifier)
				channel->notifier(channel->id,
					channel->notify_data);
		}

		
		if (channel->expiration_period_ms)
			mod_timer(&channel->expiration_timer,
				jiffies +
				MSEC_TO_JIFFIES(
					channel->expiration_period_ms));
	}

	spin_unlock_irqrestore(&device->spinlock, flags);
}

static int tspp_gpios_disable(const struct tspp_tsif_device *tsif_device,
				const struct msm_gpio *table,
				int size)
{
	int rc = 0;
	int i;
	const struct msm_gpio *g;

	for (i = size-1; i >= 0; i--) {
		int tmp;
		g = table + i;

		
		if ((tsif_device->mode != TSPP_TSIF_MODE_2) &&
			(strnstr(g->label, "sync", strlen(g->label)) != NULL))
			continue;

		tmp = gpio_tlmm_config(GPIO_CFG(GPIO_PIN(g->gpio_cfg),
			0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			GPIO_CFG_DISABLE);
		if (tmp) {
			pr_err("tspp_gpios_disable(0x%08x, GPIO_CFG_DISABLE) <%s> failed: %d\n",
			       g->gpio_cfg, g->label ?: "?", rc);
			pr_err("tspp: pin %d func %d dir %d pull %d drvstr %d\n",
			       GPIO_PIN(g->gpio_cfg), GPIO_FUNC(g->gpio_cfg),
			       GPIO_DIR(g->gpio_cfg), GPIO_PULL(g->gpio_cfg),
			       GPIO_DRVSTR(g->gpio_cfg));
			if (!rc)
				rc = tmp;
		}
	}

	return rc;
}

static int tspp_gpios_enable(const struct tspp_tsif_device *tsif_device,
				const struct msm_gpio *table,
				int size)
{
	int rc;
	int i;
	const struct msm_gpio *g;

	for (i = 0; i < size; i++) {
		g = table + i;

		
		if ((tsif_device->mode != TSPP_TSIF_MODE_2) &&
			(strnstr(g->label, "sync", strlen(g->label)) != NULL))
			continue;

		rc = gpio_tlmm_config(g->gpio_cfg, GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("tspp: gpio_tlmm_config(0x%08x, GPIO_CFG_ENABLE) <%s> failed: %d\n",
			       g->gpio_cfg, g->label ?: "?", rc);
			pr_err("tspp: pin %d func %d dir %d pull %d drvstr %d\n",
			       GPIO_PIN(g->gpio_cfg), GPIO_FUNC(g->gpio_cfg),
			       GPIO_DIR(g->gpio_cfg), GPIO_PULL(g->gpio_cfg),
			       GPIO_DRVSTR(g->gpio_cfg));
			goto err;
		}
	}
	return 0;
err:
	tspp_gpios_disable(tsif_device, table, i);

	return rc;
}


static int tspp_config_gpios(struct tspp_device *device,
				enum tspp_source source,
				int enable)
{
	const struct msm_gpio *table;
	struct msm_tspp_platform_data *pdata = device->pdev->dev.platform_data;
	int num_gpios = (pdata->num_gpios / TSPP_TSIF_INSTANCES);
	int i = 0;

	if (num_gpios != TSPP_GPIOS_PER_TSIF) {
		pr_err("tspp %s: unexpected number of GPIOs %d, expected %d\n",
			__func__, num_gpios, TSPP_GPIOS_PER_TSIF);
		return -EINVAL;
	}

	switch (source) {
	case TSPP_SOURCE_TSIF0:
		i = 0;
		break;
	case TSPP_SOURCE_TSIF1:
		i = 1;
		break;
	default:
		pr_err("tspp %s: invalid source\n", __func__);
		return -EINVAL;
	}

	table = pdata->gpios + (i * num_gpios);
	if (enable)
		return tspp_gpios_enable(&device->tsif[i], table, num_gpios);
	else
		return tspp_gpios_disable(&device->tsif[i], table, num_gpios);
}

static int tspp_clock_start(struct tspp_device *device)
{
	int rc;

	if (device == NULL) {
		pr_err("tspp: Can't start clocks, invalid device\n");
		return -EINVAL;
	}

	if (device->tsif_bus_client) {
		rc = msm_bus_scale_client_update_request(
					device->tsif_bus_client, 1);
		if (rc) {
			pr_err("tspp: Can't enable bus\n");
			return -EBUSY;
		}
	}

	if (device->tsif_vreg) {
		rc = regulator_set_voltage(device->tsif_vreg,
					RPM_REGULATOR_CORNER_SUPER_TURBO,
					RPM_REGULATOR_CORNER_SUPER_TURBO);
		if (rc) {
			pr_err("Unable to set CX voltage.\n");
			if (device->tsif_bus_client)
				msm_bus_scale_client_update_request(
					device->tsif_bus_client, 0);
			return rc;
		}
	}

	if (device->tsif_pclk && clk_prepare_enable(device->tsif_pclk) != 0) {
		pr_err("tspp: Can't start pclk");

		if (device->tsif_vreg) {
			regulator_set_voltage(device->tsif_vreg,
					RPM_REGULATOR_CORNER_NONE,
					RPM_REGULATOR_CORNER_SUPER_TURBO);
		}

		if (device->tsif_bus_client)
			msm_bus_scale_client_update_request(
				device->tsif_bus_client, 0);
		return -EBUSY;
	}

	if (device->tsif_ref_clk &&
		clk_prepare_enable(device->tsif_ref_clk) != 0) {
		pr_err("tspp: Can't start ref clk");
		clk_disable_unprepare(device->tsif_pclk);
		if (device->tsif_vreg) {
			regulator_set_voltage(device->tsif_vreg,
					RPM_REGULATOR_CORNER_NONE,
					RPM_REGULATOR_CORNER_SUPER_TURBO);
		}

		if (device->tsif_bus_client)
			msm_bus_scale_client_update_request(
				device->tsif_bus_client, 0);
		return -EBUSY;
	}

	return 0;
}

static void tspp_clock_stop(struct tspp_device *device)
{
	int rc;

	if (device == NULL) {
		pr_err("tspp: Can't stop clocks, invalid device\n");
		return;
	}

	if (device->tsif_pclk)
		clk_disable_unprepare(device->tsif_pclk);

	if (device->tsif_ref_clk)
		clk_disable_unprepare(device->tsif_ref_clk);

	if (device->tsif_vreg) {
		rc = regulator_set_voltage(device->tsif_vreg,
					RPM_REGULATOR_CORNER_NONE,
					RPM_REGULATOR_CORNER_SUPER_TURBO);
		if (rc)
			pr_err("Unable to set CX voltage.\n");
	}

	if (device->tsif_bus_client) {
		rc = msm_bus_scale_client_update_request(
					device->tsif_bus_client, 0);
		if (rc)
			pr_err("tspp: Can't disable bus\n");
	}
}

static int tspp_start_tsif(struct tspp_tsif_device *tsif_device)
{
	int start_hardware = 0;
	u32 ctl;

	if (tsif_device->ref_count == 0) {
		start_hardware = 1;
	} else if (tsif_device->ref_count > 0) {
		ctl = readl_relaxed(tsif_device->base + TSIF_STS_CTL_OFF);
		if ((ctl & TSIF_STS_CTL_START) != 1) {
			
			pr_warn("tspp: tsif hw not started but ref count > 0");
			start_hardware = 1;
		}
	}

	if (start_hardware) {
		ctl = TSIF_STS_CTL_EN_IRQ |
				TSIF_STS_CTL_EN_DM;

		if (tsif_device->clock_inverse)
			ctl |= TSIF_STS_CTL_INV_CLOCK;

		if (tsif_device->data_inverse)
			ctl |= TSIF_STS_CTL_INV_DATA;

		if (tsif_device->sync_inverse)
			ctl |= TSIF_STS_CTL_INV_SYNC;

		if (tsif_device->enable_inverse)
			ctl |= TSIF_STS_CTL_INV_ENABLE;

		switch (tsif_device->mode) {
		case TSPP_TSIF_MODE_LOOPBACK:
			ctl |= TSIF_STS_CTL_EN_NULL |
					TSIF_STS_CTL_EN_ERROR |
					TSIF_STS_CTL_TEST_MODE;
			break;
		case TSPP_TSIF_MODE_1:
			ctl |= TSIF_STS_CTL_EN_TIME_LIM |
					TSIF_STS_CTL_EN_TCR;
			break;
		case TSPP_TSIF_MODE_2:
			ctl |= TSIF_STS_CTL_EN_TIME_LIM |
					TSIF_STS_CTL_EN_TCR |
					TSIF_STS_CTL_MODE_2;
			break;
		default:
			pr_warn("tspp: unknown tsif mode 0x%x",
				tsif_device->mode);
		}
		writel_relaxed(ctl, tsif_device->base + TSIF_STS_CTL_OFF);
		writel_relaxed(tsif_device->time_limit,
			  tsif_device->base + TSIF_TIME_LIMIT_OFF);
		wmb();
		writel_relaxed(ctl | TSIF_STS_CTL_START,
			  tsif_device->base + TSIF_STS_CTL_OFF);
		wmb();
	}

	ctl = readl_relaxed(tsif_device->base + TSIF_STS_CTL_OFF);
	if (!(ctl & TSIF_STS_CTL_START))
		return -EBUSY;

	tsif_device->ref_count++;
	return 0;
}

static void tspp_stop_tsif(struct tspp_tsif_device *tsif_device)
{
	if (tsif_device->ref_count == 0)
		return;

	tsif_device->ref_count--;

	if (tsif_device->ref_count == 0) {
		writel_relaxed(TSIF_STS_CTL_STOP,
			tsif_device->base + TSIF_STS_CTL_OFF);
		wmb();
	}
}

static int tspp_channels_in_use(struct tspp_device *pdev)
{
	int i;
	int count = 0;
	for (i = 0; i < TSPP_NUM_CHANNELS; i++)
		count += (pdev->channels[i].used ? 1 : 0);

	return count;
}

static struct tspp_device *tspp_find_by_id(int id)
{
	struct tspp_device *dev;
	list_for_each_entry(dev, &tspp_devices, devlist) {
		if (dev->pdev->id == id)
			return dev;
	}
	return NULL;
}

static int tspp_get_key_entry(void)
{
	int i;
	for (i = 0; i < TSPP_NUM_KEYS; i++) {
		if (!(tspp_key_entry & (1 << i))) {
			tspp_key_entry |= (1 << i);
			return i;
		}
	}
	return 1 < TSPP_NUM_KEYS;
}

static void tspp_free_key_entry(int entry)
{
	if (entry > TSPP_NUM_KEYS) {
		pr_err("tspp_free_key_entry: index out of bounds");
		return;
	}

	tspp_key_entry &= ~(1 << entry);
}

static int tspp_alloc_buffer(u32 channel_id, struct tspp_data_descriptor *desc,
	u32 size, struct dma_pool *dma_pool, tspp_allocator *alloc, void *user)
{
	if (size < TSPP_MIN_BUFFER_SIZE ||
		size > TSPP_MAX_BUFFER_SIZE) {
		pr_err("tspp: bad buffer size %i", size);
		return -ENOMEM;
	}

	if (alloc) {
		TSPP_DEBUG("tspp using alloc function");
		desc->virt_base = alloc(channel_id, size,
			&desc->phys_base, user);
	} else {
		if (!dma_pool)
			desc->virt_base = dma_alloc_coherent(NULL, size,
				&desc->phys_base, GFP_KERNEL);
		else
			desc->virt_base = dma_pool_alloc(dma_pool, GFP_KERNEL,
				&desc->phys_base);

		if (desc->virt_base == 0) {
			pr_err("tspp: dma buffer allocation failed %i\n", size);
			return -ENOMEM;
		}
	}

	desc->size = size;
	return 0;
}

static int tspp_queue_buffer(struct tspp_channel *channel,
	struct tspp_mem_buffer *buffer)
{
	int rc;
	u32 flags = 0;

	
	if (channel->int_freq < 1)
		channel->int_freq = 1;

	
	if (buffer->desc.id % channel->int_freq == channel->int_freq-1)
		flags = SPS_IOVEC_FLAG_INT;

	
	rc = sps_transfer_one(channel->pipe,
		buffer->sps.phys_base,
		buffer->sps.size,
		channel->pdev,
		flags);
	if (rc < 0)
		return rc;

	buffer->state = TSPP_BUF_STATE_WAITING;

	return 0;
}

static int tspp_global_reset(struct tspp_device *pdev)
{
	u32 i, val;

	
	for (i = 0; i < TSPP_TSIF_INSTANCES; i++) {
		pdev->tsif[i].ref_count = 1; 
		tspp_stop_tsif(&pdev->tsif[i]); 
		pdev->tsif[i].time_limit = TSPP_TSIF_DEFAULT_TIME_LIMIT;
		pdev->tsif[i].clock_inverse = 0;
		pdev->tsif[i].data_inverse = 0;
		pdev->tsif[i].sync_inverse = 0;
		pdev->tsif[i].enable_inverse = 0;
	}
	writel_relaxed(TSPP_RST_RESET, pdev->base + TSPP_RST);
	wmb();

	
	for (i = 0; i < TSPP_FILTER_TABLES; i++)
		memset(pdev->filters[i],
			0, sizeof(struct tspp_pid_filter_table));

	
	val = (2 << TSPP_NUM_CHANNELS) - 1;
	writel_relaxed(val, pdev->base + TSPP_PS_DISABLE);

	
	val = readl_relaxed(pdev->base + TSPP_CONTROL);
	writel_relaxed(val | TSPP_CLK_CONTROL_FORCE_PERF_CNT,
		pdev->base + TSPP_CONTROL);
	wmb();
	memset(pdev->tspp_global_performance, 0,
		sizeof(struct tspp_global_performance_regs));
	memset(pdev->tspp_pipe_context, 0,
		sizeof(struct tspp_pipe_context_regs));
	memset(pdev->tspp_pipe_performance, 0,
		sizeof(struct tspp_pipe_performance_regs));
	wmb();
	writel_relaxed(val & ~TSPP_CLK_CONTROL_FORCE_PERF_CNT,
		pdev->base + TSPP_CONTROL);
	wmb();

	val = readl_relaxed(pdev->base + TSPP_CONFIG);
	val &= ~(TSPP_CONFIG_PS_LEN_ERR_MASK |
			TSPP_CONFIG_PS_CONT_ERR_UNSP_MASK |
			TSPP_CONFIG_PS_CONT_ERR_MASK);
	TSPP_CONFIG_SET_PACKET_LENGTH(val, TSPP_PACKET_LENGTH);
	writel_relaxed(val, pdev->base + TSPP_CONFIG);
	writel_relaxed(0x0007ffff, pdev->base + TSPP_IRQ_MASK);
	writel_relaxed(0x000fffff, pdev->base + TSPP_IRQ_CLEAR);
	writel_relaxed(0, pdev->base + TSPP_RST);
	wmb();

	tspp_key_entry = 0;

	return 0;
}

static int tspp_select_source(u32 dev, u32 channel_id,
	struct tspp_select_source *src)
{
	
	if (src->source > TSPP_SOURCE_MEM) {
		pr_err("tspp source out of bounds");
		return -EINVAL;
	}

	
	tspp_open_stream(dev, channel_id, src);

	return 0;
}

static int tspp_set_iv(struct tspp_channel *channel, struct tspp_iv *iv)
{
	struct tspp_device *pdev = channel->pdev;

	writel_relaxed(iv->data[0], pdev->base + TSPP_CBC_INIT_VAL(0));
	writel_relaxed(iv->data[1], pdev->base + TSPP_CBC_INIT_VAL(1));
	return 0;
}

static int tspp_set_system_keys(struct tspp_channel *channel,
	struct tspp_system_keys *keys)
{
	int i;
	struct tspp_device *pdev = channel->pdev;

	for (i = 0; i < TSPP_NUM_SYSTEM_KEYS; i++)
		writel_relaxed(keys->data[i], pdev->base + TSPP_SYSTEM_KEY(i));

	return 0;
}

static int tspp_channel_init(struct tspp_channel *channel,
	struct tspp_device *pdev)
{
	channel->cdev.owner = THIS_MODULE;
	cdev_init(&channel->cdev, &tspp_fops);
	channel->pdev = pdev;
	channel->data = NULL;
	channel->read = NULL;
	channel->waiting = NULL;
	channel->locked = NULL;
	channel->id = MINOR(tspp_minor);
	channel->used = 0;
	channel->buffer_size = TSPP_MIN_BUFFER_SIZE;
	channel->max_buffers = TSPP_NUM_BUFFERS;
	channel->buffer_count = 0;
	channel->filter_count = 0;
	channel->int_freq = 1;
	channel->src = TSPP_SOURCE_NONE;
	channel->mode = TSPP_MODE_DISABLED;
	channel->notifier = NULL;
	channel->notify_data = NULL;
	channel->expiration_period_ms = 0;
	channel->memfree = NULL;
	channel->user_info = NULL;
	init_waitqueue_head(&channel->in_queue);

	if (cdev_add(&channel->cdev, tspp_minor++, 1) != 0) {
		pr_err("tspp: cdev_add failed");
		return -EBUSY;
	}

	channel->dd = device_create(tspp_class, NULL, channel->cdev.dev,
		channel, "tspp%02d", channel->id);
	if (IS_ERR(channel->dd)) {
		pr_err("tspp: device_create failed: %i",
			(int)PTR_ERR(channel->dd));
		cdev_del(&channel->cdev);
		return -EBUSY;
	}

	return 0;
}

static int tspp_set_buffer_size(struct tspp_channel *channel,
	struct tspp_buffer *buf)
{
	if (channel->buffer_count > 0) {
		pr_err("tspp: cannot set buffer size - buffers already allocated\n");
		return -EPERM;
	}

	if (buf->size < TSPP_MIN_BUFFER_SIZE)
		channel->buffer_size = TSPP_MIN_BUFFER_SIZE;
	else if (buf->size > TSPP_MAX_BUFFER_SIZE)
		channel->buffer_size = TSPP_MAX_BUFFER_SIZE;
	else
		channel->buffer_size = buf->size;

	return 0;
}

static void tspp_set_tsif_mode(struct tspp_channel *channel,
	enum tspp_tsif_mode mode)
{
	int index;

	switch (channel->src) {
	case TSPP_SOURCE_TSIF0:
		index = 0;
		break;
	case TSPP_SOURCE_TSIF1:
		index = 1;
		break;
	default:
		pr_warn("tspp: can't set mode for non-tsif source %d",
			channel->src);
		return;
	}
	channel->pdev->tsif[index].mode = mode;
}

static void tspp_set_signal_inversion(struct tspp_channel *channel,
					int clock_inverse, int data_inverse,
					int sync_inverse, int enable_inverse)
{
	int index;

	switch (channel->src) {
	case TSPP_SOURCE_TSIF0:
		index = 0;
		break;
	case TSPP_SOURCE_TSIF1:
		index = 1;
		break;
	default:
		return;
	}
	channel->pdev->tsif[index].clock_inverse = clock_inverse;
	channel->pdev->tsif[index].data_inverse = data_inverse;
	channel->pdev->tsif[index].sync_inverse = sync_inverse;
	channel->pdev->tsif[index].enable_inverse = enable_inverse;
}

static int tspp_is_buffer_size_aligned(u32 size, enum tspp_mode mode)
{
	u32 alignment;

	switch (mode) {
	case TSPP_MODE_RAW:
		
		alignment = (TSPP_PACKET_LENGTH + 4);
		if (size % alignment)
			return 0;
		return 1;

	case TSPP_MODE_RAW_NO_SUFFIX:
		
		alignment = TSPP_PACKET_LENGTH;
		if (size % alignment)
			return 0;
		return 1;

	case TSPP_MODE_DISABLED:
	case TSPP_MODE_PES:
	default:
		
		return 1;
	}

}

static u32 tspp_align_buffer_size_by_mode(u32 size, enum tspp_mode mode)
{
	u32 new_size;
	u32 alignment;

	switch (mode) {
	case TSPP_MODE_RAW:
		
		alignment = (TSPP_PACKET_LENGTH + 4);
		break;

	case TSPP_MODE_RAW_NO_SUFFIX:
		
		alignment = TSPP_PACKET_LENGTH;
		break;

	case TSPP_MODE_DISABLED:
	case TSPP_MODE_PES:
	default:
		
		alignment = 1;
		break;
	}
	
	new_size = (((size + alignment - 1) / alignment) * alignment);
	return new_size;
}

static void tspp_destroy_buffers(u32 channel_id, struct tspp_channel *channel)
{
	int i;
	struct tspp_mem_buffer *pbuf, *temp;

	pbuf = channel->data;
	for (i = 0; i < channel->buffer_count; i++) {
		if (pbuf->desc.phys_base) {
			if (channel->memfree) {
				channel->memfree(channel_id,
					pbuf->desc.size,
					pbuf->desc.virt_base,
					pbuf->desc.phys_base,
					channel->user_info);
			} else {
				if (!channel->dma_pool)
					dma_free_coherent(NULL,
						pbuf->desc.size,
						pbuf->desc.virt_base,
						pbuf->desc.phys_base);
				else
					dma_pool_free(channel->dma_pool,
						pbuf->desc.virt_base,
						pbuf->desc.phys_base);
			}
			pbuf->desc.phys_base = 0;
		}
		pbuf->desc.virt_base = 0;
		pbuf->state = TSPP_BUF_STATE_EMPTY;
		temp = pbuf;
		pbuf = pbuf->next;
		kfree(temp);
	}
}

static int msm_tspp_req_irqs(struct tspp_device *device)
{
	int rc;
	int i;
	int j;

	rc = request_irq(device->tspp_irq, tspp_isr, IRQF_SHARED,
		dev_name(&device->pdev->dev), device);
	if (rc) {
		dev_err(&device->pdev->dev,
			"failed to request TSPP IRQ %d : %d",
			device->tspp_irq, rc);
		return rc;
	}

	for (i = 0; i < TSPP_TSIF_INSTANCES; i++) {
		rc = request_irq(device->tsif[i].tsif_irq,
			tsif_isr, IRQF_SHARED, dev_name(&device->pdev->dev),
			&device->tsif[i]);
		if (rc) {
			dev_err(&device->pdev->dev,
				"failed to request TSIF%d IRQ: %d",
				i, rc);
			goto failed;
		}
	}
	device->req_irqs = true;
	return 0;

failed:
	free_irq(device->tspp_irq, device);
	for (j = 0; j < i; j++)
		free_irq(device->tsif[j].tsif_irq, device);

	return rc;
}

static inline void msm_tspp_free_irqs(struct tspp_device *device)
{
	int i;

	for (i = 0; i < TSPP_TSIF_INSTANCES; i++) {
		if (device->tsif[i].tsif_irq)
			free_irq(device->tsif[i].tsif_irq,  &device->tsif[i]);
	}

	if (device->tspp_irq)
		free_irq(device->tspp_irq, device);
	device->req_irqs = false;
}


int tspp_open_stream(u32 dev, u32 channel_id,
			struct tspp_select_source *source)
{
	u32 val;
	int rc;
	struct tspp_device *pdev;
	struct tspp_channel *channel;
	bool req_irqs = false;

	TSPP_DEBUG("tspp_open_stream %i %i %i %i",
		dev, channel_id, source->source, source->mode);

	if (dev >= TSPP_MAX_DEVICES) {
		pr_err("tspp: device id out of range");
		return -ENODEV;
	}

	if (channel_id >= TSPP_NUM_CHANNELS) {
		pr_err("tspp: channel id out of range");
		return -ECHRNG;
	}

	pdev = tspp_find_by_id(dev);
	if (!pdev) {
		pr_err("tspp_str: can't find device %i", dev);
		return -ENODEV;
	}
	channel = &pdev->channels[channel_id];
	channel->src = source->source;
	tspp_set_tsif_mode(channel, source->mode);
	tspp_set_signal_inversion(channel, source->clk_inverse,
			source->data_inverse, source->sync_inverse,
			source->enable_inverse);

	
	if (!pdev->req_irqs && (source->source == TSPP_SOURCE_TSIF0 ||
		source->source == TSPP_SOURCE_TSIF1)) {
		rc = msm_tspp_req_irqs(pdev);
		if (rc) {
			pr_err("tspp: error requesting irqs\n");
			return rc;
		}
		req_irqs = true;
	}

	switch (source->source) {
	case TSPP_SOURCE_TSIF0:
		if (tspp_config_gpios(pdev, channel->src, 1) != 0) {
			rc = -EBUSY;
			pr_err("tspp: error enabling tsif0 GPIOs\n");
			goto free_irq;
		}
		
		if (tspp_start_tsif(&pdev->tsif[0]) != 0) {
			rc = -EBUSY;
			pr_err("tspp: error starting tsif0");
			goto free_irq;
		}
		if (pdev->tsif[0].ref_count == 1) {
			val = readl_relaxed(pdev->base + TSPP_CONTROL);
			writel_relaxed(val & ~TSPP_CONTROL_TSP_TSIF0_SRC_DIS,
				pdev->base + TSPP_CONTROL);
			wmb();
		}
		break;
	case TSPP_SOURCE_TSIF1:
		if (tspp_config_gpios(pdev, channel->src, 1) != 0) {
			rc = -EBUSY;
			pr_err("tspp: error enabling tsif1 GPIOs\n");
			goto free_irq;
		}
		
		if (tspp_start_tsif(&pdev->tsif[1]) != 0) {
			rc = -EBUSY;
			pr_err("tspp: error starting tsif1");
			goto free_irq;
		}
		if (pdev->tsif[1].ref_count == 1) {
			val = readl_relaxed(pdev->base + TSPP_CONTROL);
			writel_relaxed(val & ~TSPP_CONTROL_TSP_TSIF1_SRC_DIS,
				pdev->base + TSPP_CONTROL);
			wmb();
		}
		break;
	case TSPP_SOURCE_MEM:
		break;
	default:
		pr_err("tspp: channel %i invalid source %i",
			channel->id, source->source);
		return -EBUSY;
	}

	return 0;

free_irq:
	
	if (req_irqs)
		msm_tspp_free_irqs(pdev);
	return rc;
}
EXPORT_SYMBOL(tspp_open_stream);

int tspp_close_stream(u32 dev, u32 channel_id)
{
	u32 val;
	u32 prev_ref_count = 0;
	struct tspp_device *pdev;
	struct tspp_channel *channel;

	if (channel_id >= TSPP_NUM_CHANNELS) {
		pr_err("tspp: channel id out of range");
		return -ECHRNG;
	}
	pdev = tspp_find_by_id(dev);
	if (!pdev) {
		pr_err("tspp_cs: can't find device %i", dev);
		return -EBUSY;
	}
	channel = &pdev->channels[channel_id];

	switch (channel->src) {
	case TSPP_SOURCE_TSIF0:
		prev_ref_count = pdev->tsif[0].ref_count;
		tspp_stop_tsif(&pdev->tsif[0]);
		if (tspp_config_gpios(pdev, channel->src, 0) != 0)
			pr_err("tspp: error disabling tsif0 GPIOs\n");

		if (prev_ref_count == 1) {
			val = readl_relaxed(pdev->base + TSPP_CONTROL);
			writel_relaxed(val | TSPP_CONTROL_TSP_TSIF0_SRC_DIS,
				pdev->base + TSPP_CONTROL);
			wmb();
		}
		break;
	case TSPP_SOURCE_TSIF1:
		prev_ref_count = pdev->tsif[1].ref_count;
		tspp_stop_tsif(&pdev->tsif[1]);
		if (tspp_config_gpios(pdev, channel->src, 0) != 0)
			pr_err("tspp: error disabling tsif0 GPIOs\n");

		if (prev_ref_count == 1) {
			val = readl_relaxed(pdev->base + TSPP_CONTROL);
			writel_relaxed(val | TSPP_CONTROL_TSP_TSIF1_SRC_DIS,
				pdev->base + TSPP_CONTROL);
			wmb();
		}
		break;
	case TSPP_SOURCE_MEM:
		break;
	case TSPP_SOURCE_NONE:
		break;
	}

	channel->src = TSPP_SOURCE_NONE;

	
	if ((pdev->tsif[0].ref_count + pdev->tsif[1].ref_count) == 0 &&
		prev_ref_count)
		msm_tspp_free_irqs(pdev);

	return 0;
}
EXPORT_SYMBOL(tspp_close_stream);

static int tspp_init_sps_device(struct tspp_device *dev)
{
	int ret;

	ret = sps_register_bam_device(&dev->bam_props, &dev->bam_handle);
	if (ret) {
		pr_err("tspp: failed to register bam device, err-%d\n", ret);
		return ret;
	}

	ret = sps_device_reset(dev->bam_handle);
	if (ret) {
		sps_deregister_bam_device(dev->bam_handle);
		pr_err("tspp: error resetting bam device, err=%d\n", ret);
		return ret;
	}

	return 0;
}

int tspp_open_channel(u32 dev, u32 channel_id)
{
	int rc = 0;
	struct sps_connect *config;
	struct sps_register_event *event;
	struct tspp_channel *channel;
	struct tspp_device *pdev;

	if (channel_id >= TSPP_NUM_CHANNELS) {
		pr_err("tspp: channel id out of range");
		return -ECHRNG;
	}
	pdev = tspp_find_by_id(dev);
	if (!pdev) {
		pr_err("tspp_oc: can't find device %i", dev);
		return -ENODEV;
	}
	channel = &pdev->channels[channel_id];

	if (channel->used) {
		pr_err("tspp channel already in use");
		return -EBUSY;
	}

	config = &channel->config;
	event = &channel->event;

	
	if (tspp_channels_in_use(pdev) == 0) {
		rc = tspp_clock_start(pdev);
		if (rc)
			return rc;

		if (pdev->bam_handle == SPS_DEV_HANDLE_INVALID) {
			rc = tspp_init_sps_device(pdev);
			if (rc) {
				pr_err("tspp: failed to init sps device, err=%d\n",
					rc);
				tspp_clock_stop(pdev);
				return rc;
			}
		}

		wake_lock(&pdev->wake_lock);
	}

	
	channel->used = 1;

	
	channel->pipe = sps_alloc_endpoint();
	if (channel->pipe == 0) {
		pr_err("tspp: error allocating endpoint");
		rc = -ENOMEM;
		goto err_sps_alloc;
	}

	
	sps_get_config(channel->pipe, config);

	config->source = pdev->bam_handle;
	config->destination = SPS_DEV_HANDLE_MEM;
	config->mode = SPS_MODE_SRC;
	config->options =
		SPS_O_AUTO_ENABLE | 
		SPS_O_STREAMING | 
		SPS_O_DESC_DONE | 
		SPS_O_ACK_TRANSFERS | 
		SPS_O_HYBRID; 
	config->src_pipe_index = channel->id;
	config->desc.size =
		TSPP_SPS_DESCRIPTOR_COUNT * SPS_DESCRIPTOR_SIZE;
	config->desc.base = dma_alloc_coherent(NULL,
						config->desc.size,
						&config->desc.phys_base,
						GFP_KERNEL);
	if (config->desc.base == 0) {
		pr_err("tspp: error allocating sps descriptors");
		rc = -ENOMEM;
		goto err_desc_alloc;
	}

	memset(config->desc.base, 0, config->desc.size);

	rc = sps_connect(channel->pipe, config);
	if (rc) {
		pr_err("tspp: error connecting bam");
		goto err_connect;
	}

	event->mode = SPS_TRIGGER_CALLBACK;
	event->options = SPS_O_DESC_DONE;
	event->callback = tspp_sps_complete_cb;
	event->xfer_done = NULL;
	event->user = pdev;

	rc = sps_register_event(channel->pipe, event);
	if (rc) {
		pr_err("tspp: error registering event");
		goto err_event;
	}

	init_timer(&channel->expiration_timer);
	channel->expiration_timer.function = tspp_expiration_timer;
	channel->expiration_timer.data = (unsigned long)pdev;
	channel->expiration_timer.expires = 0xffffffffL;

	rc = pm_runtime_get(&pdev->pdev->dev);
	if (rc < 0) {
		dev_err(&pdev->pdev->dev,
			"Runtime PM: Unable to wake up tspp device, rc = %d",
			rc);
	}
	return 0;

err_event:
	sps_disconnect(channel->pipe);
err_connect:
	dma_free_coherent(NULL, config->desc.size, config->desc.base,
		config->desc.phys_base);
err_desc_alloc:
	sps_free_endpoint(channel->pipe);
err_sps_alloc:
	channel->used = 0;
	return rc;
}
EXPORT_SYMBOL(tspp_open_channel);

int tspp_close_channel(u32 dev, u32 channel_id)
{
	int i;
	int id;
	int table_idx;
	u32 val;
	unsigned long flags;

	struct sps_connect *config;
	struct tspp_device *pdev;
	struct tspp_channel *channel;

	if (channel_id >= TSPP_NUM_CHANNELS) {
		pr_err("tspp: channel id out of range");
		return -ECHRNG;
	}
	pdev = tspp_find_by_id(dev);
	if (!pdev) {
		pr_err("tspp_close: can't find device %i", dev);
		return -ENODEV;
	}
	channel = &pdev->channels[channel_id];

	
	if (!channel->used)
		return 0;

	spin_lock_irqsave(&pdev->spinlock, flags);
	channel->used = 0;
	channel->waiting = NULL;
	spin_unlock_irqrestore(&pdev->spinlock, flags);

	if (channel->expiration_period_ms)
		del_timer(&channel->expiration_timer);

	channel->notifier = NULL;
	channel->notify_data = NULL;
	channel->expiration_period_ms = 0;

	config = &channel->config;
	pdev = channel->pdev;

	
	val = readl_relaxed(pdev->base + TSPP_PS_DISABLE);
	writel_relaxed(val | channel->id, pdev->base + TSPP_PS_DISABLE);
	wmb();

	
	for (table_idx = 0; table_idx < TSPP_FILTER_TABLES; table_idx++) {
		for (i = 0; i < TSPP_NUM_PRIORITIES; i++) {
			struct tspp_pid_filter *filter =
				&pdev->filters[table_idx]->filter[i];
			id = FILTER_GET_PIPE_NUMBER0(filter);
			if (id == channel->id) {
				if (FILTER_HAS_ENCRYPTION(filter))
					tspp_free_key_entry(
						FILTER_GET_KEY_NUMBER(filter));
				filter->config = 0;
				filter->filter = 0;
			}
		}
	}
	channel->filter_count = 0;

	
	if (sps_disconnect(channel->pipe) != 0)
		pr_warn("tspp: Error freeing sps endpoint (%i)", channel->id);

	
	dma_free_coherent(NULL, config->desc.size, config->desc.base,
		config->desc.phys_base);

	sps_free_endpoint(channel->pipe);

	tspp_destroy_buffers(channel_id, channel);
	if (channel->dma_pool) {
		dma_pool_destroy(channel->dma_pool);
		channel->dma_pool = NULL;
	}

	channel->src = TSPP_SOURCE_NONE;
	channel->mode = TSPP_MODE_DISABLED;
	channel->memfree = NULL;
	channel->user_info = NULL;
	channel->buffer_count = 0;
	channel->data = NULL;
	channel->read = NULL;
	channel->locked = NULL;

	if (tspp_channels_in_use(pdev) == 0) {
		sps_deregister_bam_device(pdev->bam_handle);
		pdev->bam_handle = SPS_DEV_HANDLE_INVALID;

		wake_unlock(&pdev->wake_lock);
		tspp_clock_stop(pdev);
	}

	pm_runtime_put(&pdev->pdev->dev);

	return 0;
}
EXPORT_SYMBOL(tspp_close_channel);

int tspp_get_ref_clk_counter(u32 dev, enum tspp_source source, u32 *tcr_counter)
{
	struct tspp_device *pdev;
	struct tspp_tsif_device *tsif_device;

	if (!tcr_counter)
		return -EINVAL;

	pdev = tspp_find_by_id(dev);
	if (!pdev) {
		pr_err("tspp_get_ref_clk_counter: can't find device %i\n", dev);
		return -ENODEV;
	}

	switch (source) {
	case TSPP_SOURCE_TSIF0:
		tsif_device = &pdev->tsif[0];
		break;

	case TSPP_SOURCE_TSIF1:
		tsif_device = &pdev->tsif[1];
		break;

	default:
		tsif_device = NULL;
		break;
	}

	if (tsif_device && tsif_device->ref_count)
		*tcr_counter = ioread32(tsif_device->base + TSIF_CLK_REF_OFF);
	else
		*tcr_counter = 0;

	return 0;
}
EXPORT_SYMBOL(tspp_get_ref_clk_counter);

int tspp_add_filter(u32 dev, u32 channel_id,
	struct tspp_filter *filter)
{
	int i, rc;
	int other_channel;
	int entry;
	u32 val, pid, enabled;
	struct tspp_device *pdev;
	struct tspp_pid_filter p;
	struct tspp_channel *channel;

	TSPP_DEBUG("tspp: add filter");
	if (channel_id >= TSPP_NUM_CHANNELS) {
		pr_err("tspp: channel id out of range");
		return -ECHRNG;
	}
	pdev = tspp_find_by_id(dev);
	if (!pdev) {
		pr_err("tspp_add: can't find device %i", dev);
		return -ENODEV;
	}

	channel = &pdev->channels[channel_id];

	if (filter->source > TSPP_SOURCE_MEM) {
		pr_err("tspp invalid source");
		return -ENOSR;
	}

	if (filter->priority >= TSPP_NUM_PRIORITIES) {
		pr_err("tspp invalid filter priority");
		return -ENOSR;
	}

	channel->mode = filter->mode;
	if ((channel->buffer_count > 0) &&
	   (!tspp_is_buffer_size_aligned(channel->buffer_size, channel->mode)))
		pr_warn("tspp: buffers allocated with incorrect alignment\n");

	if (filter->mode == TSPP_MODE_PES) {
		for (i = 0; i < TSPP_NUM_PRIORITIES; i++) {
			struct tspp_pid_filter *tspp_filter =
				&pdev->filters[channel->src]->filter[i];
			pid = FILTER_GET_PIPE_PID((tspp_filter));
			enabled = FILTER_GET_PIPE_PROCESS0(tspp_filter);
			if (enabled && (pid == filter->pid)) {
				other_channel =
					FILTER_GET_PIPE_NUMBER0(tspp_filter);
				pr_err("tspp: pid 0x%x already in use by channel %i",
					filter->pid, other_channel);
				return -EBADSLT;
			}
		}
	}

	
	enabled = FILTER_GET_PIPE_PROCESS0(
		(&(pdev->filters[channel->src]->filter[filter->priority])));
	if (enabled) {
		pr_err("tspp: filter priority %i source %i is already enabled\n",
			filter->priority, channel->src);
		return -ENOSR;
	}

	if (channel->mode == TSPP_MODE_PES) {
		val = readl_relaxed(pdev->base + TSPP_PS_DISABLE);
		writel_relaxed(val | (1 << channel->id),
			pdev->base + TSPP_PS_DISABLE);
		wmb();
	}

	
	p.filter = 0;
	p.config = FILTER_TRANS_END_DISABLE;
	FILTER_SET_PIPE_PROCESS0((&p), filter->mode);
	FILTER_SET_PIPE_PID((&p), filter->pid);
	FILTER_SET_PID_MASK((&p), filter->mask);
	FILTER_SET_PIPE_NUMBER0((&p), channel->id);
	FILTER_SET_PIPE_PROCESS1((&p), TSPP_MODE_DISABLED);
	if (filter->decrypt) {
		entry = tspp_get_key_entry();
		if (entry == -1) {
			pr_err("tspp: no more keys available!");
		} else {
			p.config |= FILTER_DECRYPT;
			FILTER_SET_KEY_NUMBER((&p), entry);
		}
	}

	pdev->filters[channel->src]->
		filter[filter->priority].config = p.config;
	pdev->filters[channel->src]->
		filter[filter->priority].filter = p.filter;

	if (channel->buffer_count == 0) {
		channel->buffer_size =
		tspp_align_buffer_size_by_mode(channel->buffer_size,
							channel->mode);
		rc = tspp_allocate_buffers(dev, channel->id,
					channel->max_buffers,
					channel->buffer_size,
					channel->int_freq, NULL, NULL, NULL);
		if (rc != 0) {
			pr_err("tspp: tspp_allocate_buffers failed\n");
			return rc;
		}
	}

	
	val = readl_relaxed(pdev->base + TSPP_PS_DISABLE);
	writel_relaxed(val & ~(1 << channel->id), pdev->base + TSPP_PS_DISABLE);
	wmb();
	val = readl_relaxed(pdev->base + TSPP_PS_DISABLE);

	channel->filter_count++;

	return 0;
}
EXPORT_SYMBOL(tspp_add_filter);

int tspp_remove_filter(u32 dev, u32 channel_id,
	struct tspp_filter *filter)
{
	int entry;
	u32 val;
	struct tspp_device *pdev;
	int src;
	struct tspp_pid_filter *tspp_filter;
	struct tspp_channel *channel;

	if (channel_id >= TSPP_NUM_CHANNELS) {
		pr_err("tspp: channel id out of range");
		return -ECHRNG;
	}
	pdev = tspp_find_by_id(dev);
	if (!pdev) {
		pr_err("tspp_remove: can't find device %i", dev);
		return -ENODEV;
	}
	if (filter->priority >= TSPP_NUM_PRIORITIES) {
		pr_err("tspp invalid filter priority");
		return -ENOSR;
	}
	channel = &pdev->channels[channel_id];

	src = channel->src;
	tspp_filter = &(pdev->filters[src]->filter[filter->priority]);

	
	val = readl_relaxed(pdev->base + TSPP_PS_DISABLE);
	writel_relaxed(val | channel->id, pdev->base + TSPP_PS_DISABLE);
	wmb();

	
	if (tspp_filter->config & FILTER_DECRYPT) {
		entry = FILTER_GET_KEY_NUMBER(tspp_filter);
		tspp_free_key_entry(entry);
	}

	
	tspp_filter->config = 0;
	tspp_filter->filter = 0;

	channel->filter_count--;

	
	val = readl_relaxed(pdev->base + TSPP_PS_DISABLE);
	writel_relaxed(val & ~(1 << channel->id),
		pdev->base + TSPP_PS_DISABLE);
	wmb();
	val = readl_relaxed(pdev->base + TSPP_PS_DISABLE);

	return 0;
}
EXPORT_SYMBOL(tspp_remove_filter);

int tspp_set_key(u32 dev, u32 channel_id, struct tspp_key *key)
{
	int i;
	int id;
	int key_index;
	int data;
	struct tspp_channel *channel;
	struct tspp_device *pdev;

	if (channel_id >= TSPP_NUM_CHANNELS) {
		pr_err("tspp: channel id out of range");
		return -ECHRNG;
	}
	pdev = tspp_find_by_id(dev);
	if (!pdev) {
		pr_err("tspp_set: can't find device %i", dev);
		return -ENODEV;
	}
	channel = &pdev->channels[channel_id];

	
	for (i = 0; i < TSPP_NUM_PRIORITIES; i++) {
		struct tspp_pid_filter *tspp_filter =
			&(pdev->filters[channel->src]->filter[i]);
		id = FILTER_GET_PIPE_NUMBER0(tspp_filter);
		if (id == channel->id) {
			if (FILTER_HAS_ENCRYPTION(tspp_filter)) {
				key_index = FILTER_GET_KEY_NUMBER(tspp_filter);
				break;
			}
		}
	}
	if (i == TSPP_NUM_PRIORITIES) {
		pr_err("tspp: no encryption on this channel");
		return -ENOKEY;
	}

	if (key->parity == TSPP_KEY_PARITY_EVEN) {
		pdev->tspp_key_table->entry[key_index].even_lsb = key->lsb;
		pdev->tspp_key_table->entry[key_index].even_msb = key->msb;
	} else {
		pdev->tspp_key_table->entry[key_index].odd_lsb = key->lsb;
		pdev->tspp_key_table->entry[key_index].odd_msb = key->msb;
	}
	data = readl_relaxed(channel->pdev->base + TSPP_KEY_VALID);

	return 0;
}
EXPORT_SYMBOL(tspp_set_key);

int tspp_register_notification(u32 dev, u32 channel_id,
	tspp_notifier *pNotify, void *userdata, u32 timer_ms)
{
	struct tspp_channel *channel;
	struct tspp_device *pdev;

	if (channel_id >= TSPP_NUM_CHANNELS) {
		pr_err("tspp: channel id out of range");
		return -ECHRNG;
	}
	pdev = tspp_find_by_id(dev);
	if (!pdev) {
		pr_err("tspp_reg: can't find device %i", dev);
		return -ENODEV;
	}
	channel = &pdev->channels[channel_id];
	channel->notifier = pNotify;
	channel->notify_data = userdata;
	channel->expiration_period_ms = timer_ms;

	return 0;
}
EXPORT_SYMBOL(tspp_register_notification);

int tspp_unregister_notification(u32 dev, u32 channel_id)
{
	struct tspp_channel *channel;
	struct tspp_device *pdev;

	if (channel_id >= TSPP_NUM_CHANNELS) {
		pr_err("tspp: channel id out of range");
		return -ECHRNG;
	}
	pdev = tspp_find_by_id(dev);
	if (!pdev) {
		pr_err("tspp_unreg: can't find device %i", dev);
		return -ENODEV;
	}
	channel = &pdev->channels[channel_id];
	channel->notifier = NULL;
	channel->notify_data = 0;
	return 0;
}
EXPORT_SYMBOL(tspp_unregister_notification);

const struct tspp_data_descriptor *tspp_get_buffer(u32 dev, u32 channel_id)
{
	struct tspp_mem_buffer *buffer;
	struct tspp_channel *channel;
	struct tspp_device *pdev;
	unsigned long flags;

	if (channel_id >= TSPP_NUM_CHANNELS) {
		pr_err("tspp: channel id out of range");
		return NULL;
	}
	pdev = tspp_find_by_id(dev);
	if (!pdev) {
		pr_err("tspp_get: can't find device %i", dev);
		return NULL;
	}

	spin_lock_irqsave(&pdev->spinlock, flags);

	channel = &pdev->channels[channel_id];

	if (!channel->read) {
		spin_unlock_irqrestore(&pdev->spinlock, flags);
		pr_warn("tspp: no buffer to get on channel %i!",
			channel->id);
		return NULL;
	}

	buffer = channel->read;
	
	if (buffer->state != TSPP_BUF_STATE_DATA) {
		spin_unlock_irqrestore(&pdev->spinlock, flags);
		return NULL;
	}

	if (buffer->state == TSPP_BUF_STATE_DATA) {
		
		buffer->state = TSPP_BUF_STATE_LOCKED;

		
		channel->read = channel->read->next;
	}

	spin_unlock_irqrestore(&pdev->spinlock, flags);

	return &buffer->desc;
}
EXPORT_SYMBOL(tspp_get_buffer);

int tspp_release_buffer(u32 dev, u32 channel_id, u32 descriptor_id)
{
	int i, found = 0;
	struct tspp_mem_buffer *buffer;
	struct tspp_channel *channel;
	struct tspp_device *pdev;
	unsigned long flags;

	if (channel_id >= TSPP_NUM_CHANNELS) {
		pr_err("tspp: channel id out of range");
		return -ECHRNG;
	}
	pdev = tspp_find_by_id(dev);
	if (!pdev) {
		pr_err("tspp: can't find device %i", dev);
		return -ENODEV;
	}

	spin_lock_irqsave(&pdev->spinlock, flags);

	channel = &pdev->channels[channel_id];

	if (descriptor_id > channel->buffer_count)
		pr_warn("tspp: desc id looks weird 0x%08x", descriptor_id);

	
	buffer = channel->locked;
	for (i = 0; i < channel->buffer_count; i++) {
		if (buffer->desc.id == descriptor_id) {
			found = 1;
			break;
		}
		buffer = buffer->next;
	}
	channel->locked = channel->locked->next;

	if (!found) {
		spin_unlock_irqrestore(&pdev->spinlock, flags);
		pr_err("tspp: cant find desc %i", descriptor_id);
		return -EINVAL;
	}

	
	if (buffer->state != TSPP_BUF_STATE_LOCKED) {
		spin_unlock_irqrestore(&pdev->spinlock, flags);
		pr_err("tspp: buffer %i not locked", descriptor_id);
		return -EINVAL;
	}
	
	buffer->state = TSPP_BUF_STATE_WAITING;

	if (tspp_queue_buffer(channel, buffer))
		pr_warn("tspp: can't requeue buffer");

	spin_unlock_irqrestore(&pdev->spinlock, flags);

	return 0;
}
EXPORT_SYMBOL(tspp_release_buffer);

int tspp_allocate_buffers(u32 dev, u32 channel_id, u32 count, u32 size,
			u32 int_freq, tspp_allocator *alloc,
			tspp_memfree *memfree, void *user)
{
	struct tspp_channel *channel;
	struct tspp_device *pdev;
	struct tspp_mem_buffer *last = NULL;

	TSPP_DEBUG("tspp_allocate_buffers");

	if (channel_id >= TSPP_NUM_CHANNELS) {
		pr_err("%s: channel id out of range", __func__);
		return -ECHRNG;
	}

	pdev = tspp_find_by_id(dev);
	if (!pdev) {
		pr_err("%s: can't find device %i", __func__, dev);
		return -ENODEV;
	}

	if (count < MIN_ACCEPTABLE_BUFFER_COUNT) {
		pr_err("%s: tspp requires a minimum of %i buffers\n",
			__func__, MIN_ACCEPTABLE_BUFFER_COUNT);
		return -EINVAL;
	}

	if (count > TSPP_NUM_BUFFERS) {
		pr_err("%s: tspp requires a maximum of %i buffers\n",
			__func__, TSPP_NUM_BUFFERS);
		return -EINVAL;
	}

	channel = &pdev->channels[channel_id];

	if (channel->buffer_count > 0) {
		pr_err("%s: buffers already allocated for channel %u",
			__func__, channel_id);
		return -EINVAL;
	}

	channel->max_buffers = count;

	
	if (int_freq > channel->max_buffers) {
		int_freq = channel->max_buffers;
		pr_warn("%s: setting interrupt frequency to %u\n",
			__func__, int_freq);
	}
	channel->int_freq = int_freq;
	channel->buffer_size = size;

	
	channel->memfree = memfree;
	channel->user_info = user;

	if (TSPP_USE_DMA_POOL(channel->buffer_size)) {
		channel->dma_pool = dma_pool_create("tspp",
			NULL, channel->buffer_size, 0, 0);
		if (!channel->dma_pool) {
			pr_err("%s: Can't allocate memory pool\n", __func__);
			return -ENOMEM;
		}
	} else {
		channel->dma_pool = NULL;
	}


	for (channel->buffer_count = 0;
		channel->buffer_count < channel->max_buffers;
		channel->buffer_count++) {

		
		struct tspp_mem_buffer *desc = (struct tspp_mem_buffer *)
			kmalloc(sizeof(struct tspp_mem_buffer), GFP_KERNEL);
		if (!desc) {
			pr_warn("%s: Can't allocate desc %i",
				__func__, channel->buffer_count);
			break;
		}

		desc->desc.id = channel->buffer_count;
		
		if (tspp_alloc_buffer(channel_id, &desc->desc,
			channel->buffer_size, channel->dma_pool,
			alloc, user) != 0) {
			kfree(desc);
			pr_warn("%s: Can't allocate buffer %i",
				__func__, channel->buffer_count);
			break;
		}

		
		desc->filled = 0;
		desc->read_index = 0;
		if (!channel->data) {
			channel->data = desc;
			desc->next = channel->data;
		} else {
			last->next = desc;
		}
		last = desc;
		desc->next = channel->data;

		
		desc->sps.phys_base = desc->desc.phys_base;
		desc->sps.base = desc->desc.virt_base;
		desc->sps.size = desc->desc.size;

		
		if (tspp_queue_buffer(channel, desc))
			pr_err("%s: can't queue buffer %i",
				__func__, desc->desc.id);
	}

	if (channel->buffer_count < channel->max_buffers) {
		tspp_destroy_buffers(channel_id, channel);
		channel->buffer_count = 0;

		if (channel->dma_pool) {
			dma_pool_destroy(channel->dma_pool);
			channel->dma_pool = NULL;
		}
		return -ENOMEM;
	}

	channel->waiting = channel->data;
	channel->read = channel->data;
	channel->locked = channel->data;

	
	if (channel->expiration_period_ms)
		mod_timer(&channel->expiration_timer,
			jiffies +
			MSEC_TO_JIFFIES(
				channel->expiration_period_ms));

	return 0;
}
EXPORT_SYMBOL(tspp_allocate_buffers);

static ssize_t tspp_open(struct inode *inode, struct file *filp)
{
	u32 dev;
	struct tspp_channel *channel;

	TSPP_DEBUG("tspp_open");
	channel = container_of(inode->i_cdev, struct tspp_channel, cdev);
	filp->private_data = channel;
	dev = channel->pdev->pdev->id;

	
	
	
	printk("%s, enabe TSPP INTERRUPT\n", __func__);
	enable_tspp_interrupt(channel->pdev, 1);
	

	
	if (channel->used) {
		pr_err("tspp channel %i already in use",
			MINOR(channel->cdev.dev));
		return -EACCES;
	}

	if (tspp_open_channel(dev, channel->id) != 0) {
		pr_err("tspp: error opening channel");
		return -EACCES;
	}

	return 0;
}

static unsigned int tspp_poll(struct file *filp, struct poll_table_struct *p)
{
	unsigned long flags;
	unsigned int mask = 0;
	struct tspp_channel *channel;
	channel = filp->private_data;

	
	poll_wait(filp, &channel->in_queue, p);

	spin_lock_irqsave(&channel->pdev->spinlock, flags);
	if (channel->read &&
		channel->read->state == TSPP_BUF_STATE_DATA)
		mask = POLLIN | POLLRDNORM;

	spin_unlock_irqrestore(&channel->pdev->spinlock, flags);

	return mask;
}

static ssize_t tspp_release(struct inode *inode, struct file *filp)
{
	struct tspp_channel *channel = filp->private_data;
	u32 dev = channel->pdev->pdev->id;
	TSPP_DEBUG("tspp_release");

	
	
	
	printk("%s, disable TSPP INTERRUPT\n", __func__);
	enable_tspp_interrupt(channel->pdev, 0);
	

	tspp_close_channel(dev, channel->id);

	return 0;
}

static ssize_t tspp_read(struct file *filp, char __user *buf, size_t count,
			 loff_t *f_pos)
{
	size_t size = 0;
	size_t transferred = 0;
	struct tspp_channel *channel;
	struct tspp_mem_buffer *buffer;
	channel = filp->private_data;

	TSPP_DEBUG("tspp_read");

	while (!channel->read) {
		if (filp->f_flags & O_NONBLOCK) {
			pr_warn("tspp: no buffer on channel %i!",
				channel->id);
			return -EAGAIN;
		}
		
		if (wait_event_interruptible(channel->in_queue,
			(channel->read != NULL))) {
			pr_err("tspp: rude awakening\n");
			return -ERESTARTSYS;
		}
	}

	buffer = channel->read;

	
	while (buffer->state != TSPP_BUF_STATE_DATA) {
		if (filp->f_flags & O_NONBLOCK) {
			pr_warn("tspp: nothing to read on channel %i!",
				channel->id);
			return -EAGAIN;
		}
		
		if (wait_event_interruptible(channel->in_queue,
			(buffer->state == TSPP_BUF_STATE_DATA))) {
			pr_err("tspp: rude awakening\n");
			return -ERESTARTSYS;
		}
	}

	while (buffer->state == TSPP_BUF_STATE_DATA) {
		size = min(count, buffer->filled);
		if (size == 0)
			break;

		if (copy_to_user(buf, buffer->desc.virt_base +
			buffer->read_index, size)) {
			pr_err("tspp: error copying to user buffer");
			return -EBUSY;
		}
		buf += size;
		count -= size;
		transferred += size;
		buffer->read_index += size;

		if (buffer->read_index == buffer->filled) {
			buffer->state = TSPP_BUF_STATE_WAITING;

			if (tspp_queue_buffer(channel, buffer))
				pr_err("tspp: can't submit transfer");

			channel->locked = channel->read;
			channel->read = channel->read->next;
		}
	}

	return transferred;
}

static long tspp_ioctl(struct file *filp,
			unsigned int param0, unsigned long param1)
{
	u32 dev;
	int rc = -1;
	struct tspp_channel *channel;
	struct tspp_select_source ss;
	struct tspp_filter f;
	struct tspp_key k;
	struct tspp_iv iv;
	struct tspp_system_keys sk;
	struct tspp_buffer b;
	channel = filp->private_data;
	dev = channel->pdev->pdev->id;

	if ((param0 != TSPP_IOCTL_CLOSE_STREAM) && !param1)
		return -EINVAL;

	switch (param0) {
	case TSPP_IOCTL_SELECT_SOURCE:
		if (!access_ok(VERIFY_READ, param1,
			sizeof(struct tspp_select_source))) {
			return -EBUSY;
		}
		if (__copy_from_user(&ss, (void *)param1,
			sizeof(struct tspp_select_source)) == 0)
			rc = tspp_select_source(dev, channel->id, &ss);
		break;
	case TSPP_IOCTL_ADD_FILTER:
		if (!access_ok(VERIFY_READ, param1,
			sizeof(struct tspp_filter))) {
			return -ENOSR;
		}
		if (__copy_from_user(&f, (void *)param1,
			sizeof(struct tspp_filter)) == 0)
			rc = tspp_add_filter(dev, channel->id, &f);
		break;
	case TSPP_IOCTL_REMOVE_FILTER:
		if (!access_ok(VERIFY_READ, param1,
			sizeof(struct tspp_filter))) {
			return -EBUSY;
		}
		if (__copy_from_user(&f, (void *)param1,
			sizeof(struct tspp_filter)) == 0)
			rc = tspp_remove_filter(dev, channel->id, &f);
		break;
	case TSPP_IOCTL_SET_KEY:
		if (!access_ok(VERIFY_READ, param1,
			sizeof(struct tspp_key))) {
			return -EBUSY;
		}
		if (__copy_from_user(&k, (void *)param1,
			sizeof(struct tspp_key)) == 0)
			rc = tspp_set_key(dev, channel->id, &k);
		break;
	case TSPP_IOCTL_SET_IV:
		if (!access_ok(VERIFY_READ, param1,
			sizeof(struct tspp_iv))) {
			return -EBUSY;
		}
		if (__copy_from_user(&iv, (void *)param1,
			sizeof(struct tspp_iv)) == 0)
			rc = tspp_set_iv(channel, &iv);
		break;
	case TSPP_IOCTL_SET_SYSTEM_KEYS:
		if (!access_ok(VERIFY_READ, param1,
			sizeof(struct tspp_system_keys))) {
			return -EINVAL;
		}
		if (__copy_from_user(&sk, (void *)param1,
			sizeof(struct tspp_system_keys)) == 0)
			rc = tspp_set_system_keys(channel, &sk);
		break;
	case TSPP_IOCTL_BUFFER_SIZE:
		if (!access_ok(VERIFY_READ, param1,
			sizeof(struct tspp_buffer))) {
			rc = -EINVAL;
		}
		if (__copy_from_user(&b, (void *)param1,
			sizeof(struct tspp_buffer)) == 0)
			rc = tspp_set_buffer_size(channel, &b);
		break;
	case TSPP_IOCTL_CLOSE_STREAM:
		rc = tspp_close_stream(dev, channel->id);
		break;
	default:
		pr_err("tspp: Unknown ioctl %i", param0);
	}

	if (rc != 0)
		rc = -ENOIOCTLCMD;

	return rc;
}

static int debugfs_iomem_x32_set(void *data, u64 val)
{
	writel_relaxed(val, data);
	wmb();
	return 0;
}

static int debugfs_iomem_x32_get(void *data, u64 *val)
{
	*val = readl_relaxed(data);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_iomem_x32, debugfs_iomem_x32_get,
			debugfs_iomem_x32_set, "0x%08llx");

static void tsif_debugfs_init(struct tspp_tsif_device *tsif_device,
	int instance)
{
	char name[10];
	snprintf(name, 10, "tsif%i", instance);
	tsif_device->dent_tsif = debugfs_create_dir(
	      name, NULL);
	if (tsif_device->dent_tsif) {
		int i;
		void __iomem *base = tsif_device->base;
		for (i = 0; i < ARRAY_SIZE(debugfs_tsif_regs); i++) {
			tsif_device->debugfs_tsif_regs[i] =
			   debugfs_create_file(
				debugfs_tsif_regs[i].name,
				debugfs_tsif_regs[i].mode,
				tsif_device->dent_tsif,
				base + debugfs_tsif_regs[i].offset,
				&fops_iomem_x32);
		}

		debugfs_create_u32(
			"stat_rx_chunks",
			S_IRUGO | S_IWUSR | S_IWGRP,
			tsif_device->dent_tsif,
			&tsif_device->stat_rx);

		debugfs_create_u32(
			"stat_overflow",
			S_IRUGO | S_IWUSR | S_IWGRP,
			tsif_device->dent_tsif,
			&tsif_device->stat_overflow);

		debugfs_create_u32(
			"stat_lost_sync",
			S_IRUGO | S_IWUSR | S_IWGRP,
			tsif_device->dent_tsif,
			&tsif_device->stat_lost_sync);

		debugfs_create_u32(
			"stat_timeout",
			S_IRUGO | S_IWUSR | S_IWGRP,
			tsif_device->dent_tsif,
			&tsif_device->stat_timeout);
	}
}

static void tsif_debugfs_exit(struct tspp_tsif_device *tsif_device)
{
	if (tsif_device->dent_tsif) {
		int i;
		debugfs_remove_recursive(tsif_device->dent_tsif);
		tsif_device->dent_tsif = NULL;
		for (i = 0; i < ARRAY_SIZE(debugfs_tsif_regs); i++)
			tsif_device->debugfs_tsif_regs[i] = NULL;
	}
}

static void tspp_debugfs_init(struct tspp_device *device, int instance)
{
	char name[10];
	snprintf(name, 10, "tspp%i", instance);
	device->dent = debugfs_create_dir(
	      name, NULL);
	if (device->dent) {
		int i;
		void __iomem *base = device->base;
		for (i = 0; i < ARRAY_SIZE(debugfs_tspp_regs); i++) {
			device->debugfs_regs[i] =
			   debugfs_create_file(
				debugfs_tspp_regs[i].name,
				debugfs_tspp_regs[i].mode,
				device->dent,
				base + debugfs_tspp_regs[i].offset,
				&fops_iomem_x32);
		}
	}
}

static void tspp_debugfs_exit(struct tspp_device *device)
{
	if (device->dent) {
		int i;
		debugfs_remove_recursive(device->dent);
		device->dent = NULL;
		for (i = 0; i < ARRAY_SIZE(debugfs_tspp_regs); i++)
			device->debugfs_regs[i] = NULL;
	}
}

static __devinit struct msm_tspp_platform_data *
msm_tspp_dt_to_pdata(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct msm_tspp_platform_data *data;
	struct msm_gpio *gpios;
	struct property *prop;
	int i, rc;
	int gpio;
	u32 gpio_func;

	
	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		pr_err("tspp: Unable to allocate platform data\n");
		return NULL;
	}
	rc = of_property_read_string(node, "qcom,tsif-pclk", &data->tsif_pclk);
	if (rc) {
		pr_err("tspp: Could not find tsif-pclk property, err = %d\n",
			rc);
		return NULL;
	}
	rc = of_property_read_string(node, "qcom,tsif-ref-clk",
			&data->tsif_ref_clk);
	if (rc) {
		pr_err("tspp: Could not find tsif-ref-clk property, err = %d\n",
			rc);
		return NULL;
	}

	data->tsif_vreg_present = 0;
	prop = of_find_property(node, "vdd_cx-supply", NULL);
	if (prop)
		data->tsif_vreg_present = 1;

	data->num_gpios = of_gpio_count(node);
	if (data->num_gpios == 0) {
		pr_err("tspp: Could not find GPIO definitions\n");
		return NULL;
	}
	gpios = devm_kzalloc(&pdev->dev,
			(data->num_gpios * sizeof(struct msm_gpio)),
			GFP_KERNEL);
	if (!gpios) {
		pr_err("tspp: Unable to allocate memory for GPIOs table\n");
		return NULL;
	}
	
	if (of_property_read_u32(node, "qcom,gpios-func", &gpio_func)) {
		pr_err("tspp: Could not find gpios-func property\n");
		return NULL;
	}
	for (i = 0; i < data->num_gpios; i++) {
		gpio = of_get_gpio(node, i);
		gpios[i].gpio_cfg = GPIO_CFG(gpio, gpio_func,
						GPIO_CFG_INPUT,
						GPIO_CFG_PULL_DOWN,
						GPIO_CFG_2MA);
		rc = of_property_read_string_index(node, "qcom,gpio-names",
						i, &gpios[i].label);
		if (rc)
			pr_warn("tspp: Could not find gpio-names property\n");
	}

	data->gpios = gpios;

	return data;
}

static int msm_tspp_map_irqs(struct platform_device *pdev,
				struct tspp_device *device)
{
	int rc;

	

	
	rc = platform_get_irq_byname(pdev, "TSIF_TSPP_IRQ");
	if (rc > 0) {
		device->tspp_irq = rc;
	} else {
		dev_err(&pdev->dev, "failed to get TSPP IRQ");
		return -EINVAL;
	}

	
	rc = platform_get_irq_byname(pdev, "TSIF0_IRQ");
	if (rc > 0) {
		device->tsif[0].tsif_irq = rc;
	} else {
		dev_err(&pdev->dev, "failed to get TSIF0 IRQ");
		return -EINVAL;
	}

	rc = platform_get_irq_byname(pdev, "TSIF1_IRQ");
	if (rc > 0) {
		device->tsif[1].tsif_irq = rc;
	} else {
		dev_err(&pdev->dev, "failed to get TSIF1 IRQ");
		return -EINVAL;
	}

	
	rc = platform_get_irq_byname(pdev, "TSIF_BAM_IRQ");
	if (rc > 0) {
		device->bam_irq = rc;
	} else {
		dev_err(&pdev->dev, "failed to get TSPP BAM IRQ");
		return -EINVAL;
	}

	return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
        struct fb_event *evdata = data;
        int *blank;

        struct tspp_device *tspp_device_data = container_of(self, struct tspp_device, fb_notif);

        if (evdata && evdata->data && tspp_device_data && tspp_device_data->tsif_bus_client) {
                if (event == FB_EVENT_BLANK) {
                        blank = evdata->data;
                        if (*blank == FB_BLANK_UNBLANK)
			{
				printk("%s, enable_tspp_interrupt\n", __func__);
                                enable_tspp_interrupt(tspp_device_data, 1);
			}
                        else if (*blank == FB_BLANK_POWERDOWN)
			{
				printk("%s, disable_tspp_interrupt\n", __func__);
                                enable_tspp_interrupt(tspp_device_data, 0);
			}
                }
        }

        return 0;
}
#endif

static int __devinit msm_tspp_probe(struct platform_device *pdev)
{
	int rc = -ENODEV;
	u32 version;
	u32 i, j;
	struct msm_tspp_platform_data *data;
	struct tspp_device *device;
	struct resource *mem_tsif0;
	struct resource *mem_tsif1;
	struct resource *mem_tspp;
	struct resource *mem_bam;
	struct tspp_channel *channel;
	struct msm_bus_scale_pdata *tspp_bus_pdata = NULL;

	if (pdev->dev.of_node) {
		
		data = msm_tspp_dt_to_pdata(pdev);
		
		rc = of_property_read_u32(pdev->dev.of_node,
					"cell-index", &pdev->id);
		if (rc)
			pdev->id = -1;

		pdev->dev.platform_data = data;

		tspp_bus_pdata = msm_bus_cl_get_pdata(pdev);
	} else {
		
		data = pdev->dev.platform_data;
		tspp_bus_pdata = NULL;
	}
	if (!data) {
		pr_err("tspp: Platform data not available");
		rc = -EINVAL;
		goto out;
	}

	
	if ((pdev->id < 0) || (pdev->id >= TSPP_MAX_DEVICES)) {
		pr_err("tspp: Invalid device ID %d", pdev->id);
		rc = -EINVAL;
		goto out;
	}

	
	device = kzalloc(sizeof(struct tspp_device), GFP_KERNEL);
	if (!device) {
		pr_err("tspp: Failed to allocate memory for device");
		rc = -ENOMEM;
		goto out;
	}

	
	device->pdev = pdev;
	platform_set_drvdata(pdev, device);

	
	if (tspp_bus_pdata) {
		device->tsif_bus_client =
			msm_bus_scale_register_client(tspp_bus_pdata);
		if (!device->tsif_bus_client)
			pr_err("tspp: Unable to register bus client\n");
	} else {
		device->tsif_bus_client = 0;
	}

	
	if (data->tsif_vreg_present) {
		device->tsif_vreg = devm_regulator_get(&pdev->dev, "vdd_cx");
		if (IS_ERR(device->tsif_vreg)) {
			rc = PTR_ERR(device->tsif_vreg);
			device->tsif_vreg = NULL;
			goto err_regultaor;
		}

		
		rc = regulator_set_voltage(device->tsif_vreg,
					RPM_REGULATOR_CORNER_NONE,
					RPM_REGULATOR_CORNER_SUPER_TURBO);
		if (rc) {
			dev_err(&pdev->dev, "Unable to set CX voltage.\n");
			goto err_regultaor;
		}

		rc = regulator_enable(device->tsif_vreg);
		if (rc) {
			dev_err(&pdev->dev, "Unable to enable CX regulator.\n");
			goto err_regultaor;
		}
	}

	
	if (data->tsif_pclk) {
		device->tsif_pclk = clk_get(&pdev->dev, data->tsif_pclk);
		if (IS_ERR(device->tsif_pclk)) {
			rc = PTR_ERR(device->tsif_pclk);
			device->tsif_pclk = NULL;
			goto err_pclock;
		}
	}
	if (data->tsif_ref_clk) {
		device->tsif_ref_clk = clk_get(&pdev->dev, data->tsif_ref_clk);
		if (IS_ERR(device->tsif_ref_clk)) {
			rc = PTR_ERR(device->tsif_ref_clk);
			device->tsif_ref_clk = NULL;
			goto err_refclock;
		}
	}

	
	mem_tsif0 = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, "MSM_TSIF0_PHYS");
	if (!mem_tsif0) {
		pr_err("tspp: Missing tsif0 MEM resource");
		rc = -ENXIO;
		goto err_res_tsif0;
	}
	device->tsif[0].base = ioremap(mem_tsif0->start,
		resource_size(mem_tsif0));
	if (!device->tsif[0].base) {
		pr_err("tspp: ioremap failed");
		goto err_map_tsif0;
	}

	mem_tsif1 = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, "MSM_TSIF1_PHYS");
	if (!mem_tsif1) {
		dev_err(&pdev->dev, "Missing tsif1 MEM resource");
		rc = -ENXIO;
		goto err_res_tsif1;
	}
	device->tsif[1].base = ioremap(mem_tsif1->start,
		resource_size(mem_tsif1));
	if (!device->tsif[1].base) {
		dev_err(&pdev->dev, "ioremap failed");
		goto err_map_tsif1;
	}

	mem_tspp = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, "MSM_TSPP_PHYS");
	if (!mem_tspp) {
		dev_err(&pdev->dev, "Missing MEM resource");
		rc = -ENXIO;
		goto err_res_dev;
	}
	device->base = ioremap(mem_tspp->start, resource_size(mem_tspp));
	if (!device->base) {
		dev_err(&pdev->dev, "ioremap failed");
		goto err_map_dev;
	}

	mem_bam = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, "MSM_TSPP_BAM_PHYS");
	if (!mem_bam) {
		pr_err("tspp: Missing bam MEM resource");
		rc = -ENXIO;
		goto err_res_bam;
	}
	memset(&device->bam_props, 0, sizeof(device->bam_props));
	device->bam_props.phys_addr = mem_bam->start;
	device->bam_props.virt_addr = ioremap(mem_bam->start,
		resource_size(mem_bam));
	if (!device->bam_props.virt_addr) {
		dev_err(&pdev->dev, "ioremap failed");
		goto err_map_bam;
	}

	if (msm_tspp_map_irqs(pdev, device))
		goto err_irq;
	device->req_irqs = false;

	
	
	
	printk("%s, disable TSPP INTERRUPT\n", __func__);
	enable_tspp_interrupt(device, 0);
	

	
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	tspp_debugfs_init(device, 0);

	for (i = 0; i < TSPP_TSIF_INSTANCES; i++)
		tsif_debugfs_init(&device->tsif[i], i);

	wake_lock_init(&device->wake_lock, WAKE_LOCK_SUSPEND,
		dev_name(&pdev->dev));

	
	device->filters[0] = device->base + TSPP_PID_FILTER_TABLE0;
	device->filters[1] = device->base + TSPP_PID_FILTER_TABLE1;
	device->filters[2] = device->base + TSPP_PID_FILTER_TABLE2;
	device->tspp_key_table = device->base + TSPP_DATA_KEY;
	device->tspp_global_performance =
		device->base + TSPP_GLOBAL_PERFORMANCE;
	device->tspp_pipe_context =
		device->base + TSPP_PIPE_CONTEXT;
	device->tspp_pipe_performance =
		device->base + TSPP_PIPE_PERFORMANCE;

	device->bam_props.summing_threshold = 0x10;
	device->bam_props.irq = device->bam_irq;
	device->bam_props.manage = SPS_BAM_MGR_LOCAL;

	if (tspp_clock_start(device) != 0) {
		dev_err(&pdev->dev, "Can't start clocks");
		goto err_clock;
	}

	device->bam_handle = SPS_DEV_HANDLE_INVALID;

	spin_lock_init(&device->spinlock);
	tasklet_init(&device->tlet, tspp_sps_complete_tlet,
			(unsigned long)device);

	
	tspp_global_reset(device);

	version = readl_relaxed(device->base + TSPP_VERSION);
	if ((version != 0x1) && (((version >> 28) & 0xF) != 0x1))
		pr_warn("tspp: unrecognized hw version=%i", version);

	
	for (i = 0; i < TSPP_NUM_CHANNELS; i++) {
		if (tspp_channel_init(&(device->channels[i]), device) != 0) {
			pr_err("tspp_channel_init failed");
			goto err_channel;
		}
	}

	
	tspp_clock_stop(device);

	
	list_add_tail(&(device->devlist), &tspp_devices);

#if defined(CONFIG_FB)
	{
        int error;

        device->fb_notif.notifier_call = fb_notifier_callback;
        error = fb_register_client(&device->fb_notif);

        if (error)
                pr_err("Unable to register fb_notifier: %d\n", error);
	}
#endif

	return 0;

err_channel:
	
	for (j = 0; j < i; j++) {
		channel = &(device->channels[i]);
		device_destroy(tspp_class, channel->cdev.dev);
		cdev_del(&channel->cdev);
	}

	sps_deregister_bam_device(device->bam_handle);
err_clock:
	tspp_debugfs_exit(device);
	for (i = 0; i < TSPP_TSIF_INSTANCES; i++)
		tsif_debugfs_exit(&device->tsif[i]);
err_irq:
	iounmap(device->bam_props.virt_addr);
err_map_bam:
err_res_bam:
	iounmap(device->base);
err_map_dev:
err_res_dev:
	iounmap(device->tsif[1].base);
err_map_tsif1:
err_res_tsif1:
	iounmap(device->tsif[0].base);
err_map_tsif0:
err_res_tsif0:
	if (device->tsif_ref_clk)
		clk_put(device->tsif_ref_clk);
err_refclock:
	if (device->tsif_pclk)
		clk_put(device->tsif_pclk);
err_pclock:
	if (device->tsif_vreg)
		regulator_disable(device->tsif_vreg);
err_regultaor:
	if (device->tsif_bus_client)
		msm_bus_scale_unregister_client(device->tsif_bus_client);
	kfree(device);

out:
	return rc;
}

static int __devexit msm_tspp_remove(struct platform_device *pdev)
{
	struct tspp_channel *channel;
	u32 i;

	struct tspp_device *device = platform_get_drvdata(pdev);

	
	for (i = 0; i < TSPP_NUM_CHANNELS; i++) {
		channel = &device->channels[i];
		tspp_close_channel(device->pdev->id, i);
		device_destroy(tspp_class, channel->cdev.dev);
		cdev_del(&channel->cdev);
	}

	for (i = 0; i < TSPP_TSIF_INSTANCES; i++)
		tsif_debugfs_exit(&device->tsif[i]);

	if (device->tsif_bus_client)
		msm_bus_scale_unregister_client(device->tsif_bus_client);

	wake_lock_destroy(&device->wake_lock);
	if (device->req_irqs)
		msm_tspp_free_irqs(device);

	iounmap(device->bam_props.virt_addr);
	iounmap(device->base);
	for (i = 0; i < TSPP_TSIF_INSTANCES; i++)
		iounmap(device->tsif[i].base);

	if (device->tsif_ref_clk)
		clk_put(device->tsif_ref_clk);

	if (device->tsif_pclk)
		clk_put(device->tsif_pclk);

	if (device->tsif_vreg)
		regulator_disable(device->tsif_vreg);

	pm_runtime_disable(&pdev->dev);

	kfree(device);

	return 0;
}


static int tspp_runtime_suspend(struct device *dev_)
{
	
	
	struct platform_device *pdev;
	struct tspp_device *tsppdev;
	

	dev_dbg(dev_, "pm_runtime: suspending...");

	
	
	pdev = container_of(dev_, struct platform_device, dev);
	tsppdev = platform_get_drvdata(pdev);

	
	printk("%s, disable TSPP INTERRUPT\n", __func__);
	enable_tspp_interrupt(tsppdev, 0);
	

	return 0;
}

static int tspp_runtime_resume(struct device *dev_)
{
	
	
	struct platform_device *pdev;
	struct tspp_device *tsppdev;
	

	dev_dbg(dev_, "pm_runtime: resuming...");

	
	
	pdev = container_of(dev_, struct platform_device, dev);
	tsppdev = platform_get_drvdata(pdev);

	
	printk("%s, enable TSPP INTERRUPT\n", __func__);
	enable_tspp_interrupt(tsppdev, 1);
	

	return 0;
}

static const struct dev_pm_ops tspp_dev_pm_ops = {
	.runtime_suspend = tspp_runtime_suspend,
	.runtime_resume = tspp_runtime_resume,
};

static struct of_device_id msm_match_table[] = {
	{.compatible = "qcom,msm_tspp"},
	{}
};

static struct platform_driver msm_tspp_driver = {
	.probe          = msm_tspp_probe,
	.remove         = __exit_p(msm_tspp_remove),
	.driver         = {
		.name   = "msm_tspp",
		.pm     = &tspp_dev_pm_ops,
		.of_match_table = msm_match_table,
	},
};

static int __init mod_init(void)
{
	int rc;

	
	rc = alloc_chrdev_region(&tspp_minor, 0, TSPP_NUM_CHANNELS, "tspp");
	if (rc) {
		pr_err("tspp: alloc_chrdev_region failed: %d", rc);
		goto err_devrgn;
	}

	tspp_class = class_create(THIS_MODULE, "tspp");
	if (IS_ERR(tspp_class)) {
		rc = PTR_ERR(tspp_class);
		pr_err("tspp: Error creating class: %d", rc);
		goto err_class;
	}

	
	rc = platform_driver_register(&msm_tspp_driver);
	if (rc) {
		pr_err("tspp: platform_driver_register failed: %d", rc);
		goto err_register;
	}

	return 0;

err_register:
	class_destroy(tspp_class);
err_class:
	unregister_chrdev_region(0, TSPP_NUM_CHANNELS);
err_devrgn:
	return rc;
}

static void __exit mod_exit(void)
{
	
	platform_driver_unregister(&msm_tspp_driver);

	
	class_destroy(tspp_class);
	unregister_chrdev_region(0, TSPP_NUM_CHANNELS);
}

module_init(mod_init);
module_exit(mod_exit);

MODULE_DESCRIPTION("TSPP platform device and char dev");
MODULE_LICENSE("GPL v2");
