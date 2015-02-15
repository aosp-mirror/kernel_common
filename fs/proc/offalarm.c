#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <htc_debug/stability/dirty_file_detector.h>
#include <htc_offalarm.h>

static int offalarm_proc_show(struct seq_file *m, void *v)
{
	if(rtc_alarm_trigger)
		seq_printf(m, "androidboot.bootreason=rtc_alarm androidboot.alarmid=%d androidboot.alarmtime=%d\n",htc_offalarm.alarm_id, htc_offalarm.alarm_time);
	else
		seq_printf(m, "Normal.\n");
	return 0;
}

static int offalarm_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, offalarm_proc_show, NULL);
}

static const struct file_operations offalarm_proc_fops = {
	.open		= offalarm_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_offalarm_init(void)
{
	proc_create("offalarm", 0, NULL, &offalarm_proc_fops);
	return 0;
}
module_init(proc_offalarm_init);
