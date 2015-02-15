#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <net/tcp.h>

#define MAXDATASIZE	5000000
#define MAXTMPSIZE	150

#define NIPQUAD(addr) \
    ((unsigned char *)&addr)[0], \
    ((unsigned char *)&addr)[1], \
    ((unsigned char *)&addr)[2], \
    ((unsigned char *)&addr)[3]

unsigned int probe_seq_tx;

/* Using Procfs to record log data */
static struct proc_dir_entry *proc_mtd;
static char *ProcBuffer;
static int Ring=0, WritingLength;
#if 0//Remove network packet info
/*++SSD_RIL@20120110: print network packet info to kernrl log after HSIC resume*/
static int enable_log = 0;
/*--SSD_RIL*/
#endif
static DEFINE_MUTEX(probe_data_mutexlock);

extern void (*record_probe_data_fp)(struct sock *sk, int type, size_t size, unsigned long long t_pre);
#if 0//Remove network packet info
/*++SSD_RIL@20120110: print network packet info to kernrl log after HSIC resume*/
#if defined(CONFIG_USB_EHCI_HCD) && defined(CONFIG_USB_EHCI_MSM_HSIC)
extern void (*set_htc_monitor_resume_state_fp)(void);
#endif
#endif
/*--SSD_RIL*/
void record_probe_data(struct sock *sk, int type, size_t size, unsigned long long t_pre);
#if 0//Remove network packet info
/*++SSD_RIL@20120110: print network packet info to kernrl log after HSIC resume*/
void set_htc_monitor_resume_state(void);
/*--SSD_RIL*/
#endif

static void* 	log_seq_start(struct seq_file *sfile, loff_t *pos);
static void* 	log_seq_next(struct seq_file *sfile, void *v, loff_t *pos);
static void 	log_seq_stop(struct seq_file *sfile, void *v);
static int 	log_seq_show(struct seq_file *sfile, void *v);

static struct seq_operations log_seq_ops = {
	.start = log_seq_start,
	.next = log_seq_next,
	.stop = log_seq_stop,
	.show = log_seq_show
};

static int log_proc_open(struct inode *inode, struct file *file);

static struct file_operations log_proc_ops = {
	.owner = THIS_MODULE,
	.open = log_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release
};

static struct kobject *htc_monitor_status_obj;
static uint32_t htc_monitor_param = 0; /* Default: disable */

static ssize_t htc_monitor_param_get(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
        ssize_t length;
        length = sprintf(buf, "%d\n", htc_monitor_param);
        return length;
}

static ssize_t htc_monitor_param_set(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        unsigned long result;
	ssize_t ret = -EINVAL;

	if(htc_monitor_param == 1) {
		pr_info(" htc_monitor_param already enabled, can not be disabled\n");
		return -EINVAL;
	}

	ret = strict_strtoul(buf, 10, &result);
	if (!ret) {
		if( result != 1) /* Only allow enable */
			return -EINVAL;
		else {
			htc_monitor_param = 1;
			pr_info(" htc_monitor_param: %d\n",  htc_monitor_param);
			record_probe_data_fp = record_probe_data;
#if 0//Remove network packet info
			/*++SSD_RIL@20120110: print network packet info to kernrl log after HSIC resume*/
#if defined(CONFIG_USB_EHCI_HCD) && defined(CONFIG_USB_EHCI_MSM_HSIC)
			set_htc_monitor_resume_state_fp = set_htc_monitor_resume_state;
#endif
			/*--SSD_RIL*/
#endif
		}
	}

        return ret;
}

static DEVICE_ATTR(htc_monitor_param, 0644,
        htc_monitor_param_get,
        htc_monitor_param_set);

int init_module(void)
{
	int ret;

	ProcBuffer = vmalloc(sizeof(char)*MAXDATASIZE);
	if(ProcBuffer == NULL) {
		return -ENOMEM;
	}
	memset(ProcBuffer,0,sizeof(char)*MAXDATASIZE);
	WritingLength = 0;

	/* Procfs setting */
	if( (proc_mtd = create_proc_entry("htc_monitor", 0444, NULL)) ) {
		proc_mtd->proc_fops = &log_proc_ops;
	}

	/* Attribute file setting */
	htc_monitor_status_obj = kobject_create_and_add("htc_monitor_status", NULL);
	if (htc_monitor_status_obj == NULL) {
		pr_info("kobject_create_and_add: htc_monitor_status failed\n");
                return -EFAULT;
	}

	ret = sysfs_create_file(htc_monitor_status_obj,
             &dev_attr_htc_monitor_param.attr);
        if (ret) {
                pr_info("sysfs_create_file: dev_attr_htc_monitor_param failed\n");
                return -EFAULT;
        }

	return 0;
}

void cleanup_module(void)
{
	vfree(ProcBuffer);

	/* Procfs setting */
	remove_proc_entry("htc_monitor", NULL);

	/* Attribute file setting */
	if(htc_monitor_status_obj != NULL) {
		sysfs_remove_file(htc_monitor_status_obj,&dev_attr_htc_monitor_param.attr);
		kobject_put(htc_monitor_status_obj);
	}
}

static void* log_seq_start(struct seq_file *sfile, loff_t *pos)
{
	if(*pos >= MAXDATASIZE) {
		return NULL;
	} else {
		if (*pos >= WritingLength && 0==Ring)
			return NULL;
	}
	return &ProcBuffer[*pos];
}

static void* log_seq_next(struct seq_file *sfile, void *v, loff_t *pos)
{
	(*pos)++;
	if(*pos >= MAXDATASIZE) {
		return NULL;
	} else {
		if (*pos >= WritingLength && 0==Ring)
			return NULL;
	}
	return &ProcBuffer[*pos];
}

static void log_seq_stop(struct seq_file *sfile, void *v)
{
	return ;
}

static int log_seq_show(struct seq_file *sfile, void *v)
{
	char c = *((char *)v);
	seq_putc(sfile, c);

	return 0;
}

static int log_proc_open(struct inode *inode, struct file *file)
{
	if( htc_monitor_param == 0 )
		return -EPERM;
	return seq_open(file, &log_seq_ops);
}

#if 0//Remove network packet info
/*++SSD_RIL@20120110: print network packet info to kernrl log after HSIC resume*/
void set_htc_monitor_resume_state(void)
{
	enable_log = 1;
}
/*--SSD_RIL*/
#endif

void record_probe_data(struct sock *sk, int type, size_t size, unsigned long long t_pre)
{
	char Tmp1[MAXTMPSIZE];
	int Tmp1_len;
	struct inet_sock *inet = inet_sk(sk);
	__be16 sport, dport;
	__be32 daddr, saddr;
	unsigned long long t_now;
	unsigned long nanosec_rem;
	unsigned long nanosec_rem_pre;
	t_now = sched_clock();
	nanosec_rem=do_div(t_now, 1000000000U);
	nanosec_rem_pre=do_div(t_pre, 1000000000U);

	if (!inet)
		return;

	saddr=inet->inet_rcv_saddr;
	sport=inet->inet_num;
	daddr=inet->inet_daddr;
	dport=inet->inet_dport;

	//filter
	if (0x00000000==saddr || 0x0100007f==saddr)
		return;

	memset(Tmp1,0,sizeof(char)*MAXTMPSIZE);

	switch (type)
	{
		case 1: //send
		{
			unsigned long long t_diff=t_now-t_pre;
			unsigned long nanosec_rem_diff;

			if (nanosec_rem>=nanosec_rem_pre)
				nanosec_rem_diff=nanosec_rem-nanosec_rem_pre;
			else {
				if (t_diff>0) {
					t_diff=t_diff-1;
					nanosec_rem_diff=1000000000+nanosec_rem-nanosec_rem_pre;
				} else {
					t_diff=t_pre;
					nanosec_rem_diff=nanosec_rem_pre;
				}
			}
			snprintf(Tmp1,MAXTMPSIZE,"[%05u.%09lu] UID%05d PID%05d        SEND S.IP:%03d.%03d.%03d.%03d/%05d, D.IP:%03d.%03d.%03d.%03d/%05d,%08d Bytes,D.T[%01u.%09lu]\n",
					(unsigned)t_now,nanosec_rem,
					current->cred->uid, current->pid,
					NIPQUAD(saddr),sport,
					NIPQUAD(daddr),dport,
					size,(unsigned)t_diff,nanosec_rem_diff);
			break;
		}
		case 2: //recv
		{
			unsigned long long t_diff=t_now-t_pre;
			unsigned long nanosec_rem_diff;

			if (nanosec_rem>=nanosec_rem_pre)
				nanosec_rem_diff=nanosec_rem-nanosec_rem_pre;
			else {
				if (t_diff>0) {
					t_diff=t_diff-1;
					nanosec_rem_diff=1000000000+nanosec_rem-nanosec_rem_pre;
				} else {
					t_diff=t_pre;
					nanosec_rem_diff=nanosec_rem_pre;
				}
			}
			snprintf(Tmp1,MAXTMPSIZE,"[%05u.%09lu] UID%05d PID%05d        RECV S.IP:%03d.%03d.%03d.%03d/%05d, D.IP:%03d.%03d.%03d.%03d/%05d,%08d Bytes,D.T[%01u.%09lu]\n",
					(unsigned)t_now,nanosec_rem,
					current->cred->uid,current->pid,
					NIPQUAD(saddr),sport,
					NIPQUAD(daddr),dport,
					size,(unsigned)t_diff,nanosec_rem_diff);
			break;
		}
		case 3: //accept
			snprintf(Tmp1,MAXTMPSIZE,"[%05u.%09lu] UID%05d PID%05d      ACCEPT S.IP:%03d.%03d.%03d.%03d/%05d, D.IP:%03d.%03d.%03d.%03d/%05d,              ,                \n",(unsigned)t_now,nanosec_rem,current->cred->uid,current->pid,NIPQUAD(saddr),sport,NIPQUAD(daddr),dport);
			break;
		case 4: //tcp connect
			snprintf(Tmp1,MAXTMPSIZE,"[%05u.%09lu] UID%05d PID%05d TCP CONNECT S.IP:%03d.%03d.%03d.%03d/%05d, D.IP:%03d.%03d.%03d.%03d/%05d,              ,                \n",(unsigned)t_now,nanosec_rem,current->cred->uid,current->pid,NIPQUAD(saddr),sport,NIPQUAD(daddr),dport);
			break;
		case 5: //udp connect
			snprintf(Tmp1,MAXTMPSIZE,"[%05u.%09lu] UID%05d PID%05d UDP CONNECT S.IP:%03d.%03d.%03d.%03d/%05d, D.IP:%03d.%03d.%03d.%03d/%05d,              ,                \n",(unsigned)t_now,nanosec_rem,current->cred->uid,current->pid,NIPQUAD(saddr),sport,NIPQUAD(daddr),dport);
			break;
		case 6: //close
			snprintf(Tmp1,MAXTMPSIZE,"[%05u.%09lu] UID%05d PID%05d       CLOSE S.IP:%03d.%03d.%03d.%03d/%05d, D.IP:%03d.%03d.%03d.%03d/%05d,              ,                \n",(unsigned)t_now,nanosec_rem,current->cred->uid,current->pid,NIPQUAD(saddr),sport,NIPQUAD(daddr),dport);
			break;
		default:
			break;
	}

	Tmp1_len = strlen(Tmp1);

	mutex_lock(&probe_data_mutexlock);
	if(WritingLength + Tmp1_len < MAXDATASIZE) {
		memcpy(&ProcBuffer[WritingLength], Tmp1, Tmp1_len);
		WritingLength += Tmp1_len;
	} else {
		WritingLength=0;
		Ring=1;
		memcpy(&ProcBuffer[WritingLength], Tmp1, Tmp1_len);
		WritingLength += Tmp1_len;
	}
#if 0
	/*++SSD_RIL@20120110: print network packet info to kernrl log after HSIC resume*/
	if ( enable_log ) {
		enable_log = 0;
		pr_info("[htc_monitor]%s", Tmp1);
	}
	/*--SSD_RIL*/
#endif
	mutex_unlock(&probe_data_mutexlock);
	return;
}

static int __init monitor_init(void)
{
	return init_module();
}
static void __exit monitor_exit(void)
{
	cleanup_module();
}

module_init(monitor_init);
module_exit(monitor_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("htc monitor driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:htc_monitor");
