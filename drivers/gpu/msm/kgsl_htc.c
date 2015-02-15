#include <mach/board.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <mach/devices_cmdline.h>

#include "kgsl_htc.h"
#include "adreno.h"

static int gpu_fault_no_panic_set(void *data, u64 val)
{
	struct kgsl_device *device = data;
	device->gpu_fault_no_panic = val;
	printk("kgsl: %s: gpu_fault_no_panic = %d\n",
		__FUNCTION__, device->gpu_fault_no_panic);
	return 0;
}

static int gpu_fault_no_panic_get(void *data, u64 *val)
{
	struct kgsl_device *device = data;
	*val = device->gpu_fault_no_panic;
	printk("kgsl: %s: gpu_fault_no_panic = %d\n",
		__FUNCTION__, device->gpu_fault_no_panic);
	return 0;
}

static int ctx_dump_set(void* data, u64 val)
{
	struct kgsl_device *device = data;

	read_lock(&device->context_lock);
	kgsl_dump_contextpid_locked(&device->context_idr);
	read_unlock(&device->context_lock);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(ctx_dump_fops,
				NULL,
				ctx_dump_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(gpu_fault_no_panic_fops,
				gpu_fault_no_panic_get,
				gpu_fault_no_panic_set, "%llu\n");

unsigned int kgsl_get_alloc_size(int detailed)
{
	struct kgsl_driver_htc_priv *priv = &kgsl_driver.priv;

	if (detailed && time_after(jiffies, priv->next_jiffies)) {
		priv->next_jiffies = jiffies + 20*HZ;
		schedule_work(&kgsl_driver.priv.work);
	}

	return kgsl_driver.stats.page_alloc;
}

static void do_print_mem_detail(struct work_struct *work)
{
	struct kgsl_driver_htc_priv *priv = container_of(work,
			struct kgsl_driver_htc_priv, work);
	struct kgsl_driver *driver = container_of(priv,
			struct kgsl_driver, priv);
	struct kgsl_process_private *private;
	int i;

	printk("kgsl: kgsl_driver.stats.page_alloc = %u\n", driver->stats.page_alloc);
	printk("kgsl: kgsl_driver.stats.page_alloc_kernel = %u\n", driver->stats.vmalloc);

	mutex_lock(&driver->process_mutex);
	list_for_each_entry(private, &driver->process_list, list) {
		if (!private)
			continue;
		for (i = 0; i < KGSL_MEM_ENTRY_MAX; i++) {
			switch (i) {
			case KGSL_MEM_ENTRY_PAGE_ALLOC:
				if (private->stats[KGSL_MEM_ENTRY_PAGE_ALLOC].cur != 0)
					printk("kgsl: proc %5d alloc page %8d bytes\n", private->pid, private->stats[KGSL_MEM_ENTRY_PAGE_ALLOC].cur);
				break;
			}
		}
	}
	mutex_unlock(&driver->process_mutex);
}

int kgsl_driver_htc_init(struct kgsl_driver_htc_priv *priv)
{
	priv->next_jiffies = jiffies;
	INIT_WORK(&priv->work, do_print_mem_detail);
	return 0;
}

int kgsl_device_htc_init(struct kgsl_device *device)
{
	device->gpu_fault_no_panic = CONFIG_MSM_KGSL_DEFAULT_GPU_HUNG_NO_PANIC;

	if (!device->d_debugfs || IS_ERR(device->d_debugfs))
		return 1;
	debugfs_create_file("contexpid_dump",  0644, device->d_debugfs, device,
					&ctx_dump_fops);
	debugfs_create_file("gpu_fault_no_panic",  0644, device->d_debugfs, device,
                    &gpu_fault_no_panic_fops);
	return 0;
}

void kgsl_dump_contextpid_locked(struct idr *context_idr)
{
	int i = 0;
	struct kgsl_context *context;
	struct task_struct *task;
	struct task_struct *parent_task;
	char task_name[TASK_COMM_LEN+1];
	char task_parent_name[TASK_COMM_LEN+1];
	pid_t ppid;

	printk(" == [KGSL] context maximal count is %d, dump context id, pid, name, group leader name ==\n",KGSL_MEMSTORE_MAX);
	for (i = 0; i <KGSL_MEMSTORE_MAX; i++) {

		context = idr_find(context_idr, i);

		if (context  && context->dev_priv &&  context->dev_priv->process_priv) {
			task = find_task_by_pid_ns(context->dev_priv->process_priv->pid, &init_pid_ns);
			if (!task) {
				printk("can't find context's task: context id %d\n", context->id);
				continue;
			}
			parent_task = task->group_leader;
			get_task_comm(task_name, task);

			if (parent_task) {
				get_task_comm(task_parent_name, parent_task);
				ppid = context->dev_priv->process_priv->pid;
			} else {
				task_parent_name[0] = '\0';
				ppid = 0;
			}

			printk("context id=%d\t\t pid=%d\t\t %s\t\t %s\n", context->id, ppid, task_name, task_parent_name);
		}
	}
}

#ifndef CONFIG_MSM_KGSL_KILL_HANG_PROCESS
static int adreno_kill_suspect(struct kgsl_device *device, int pid)
{
	return 1;
}
#else

struct kgsl_process_name {
	char name[TASK_COMM_LEN+1];
};

static const struct kgsl_process_name kgsl_blocking_process_tbl[] = {
	{"SurfaceFlinger"},
	{"surfaceflinger"},
	{"ndroid.systemui"},
	{"droid.htcdialer"},
	{"m.android.phone"},
	{"mediaserver"},
};

static int adreno_kill_suspect(struct kgsl_device *device, int pid)
{
	int ret = 1;
	int cankill = 1;
	char suspect_task_comm[TASK_COMM_LEN+1];
	char suspect_task_parent_comm[TASK_COMM_LEN+1];
	int suspect_tgid;
	struct task_struct *suspect_task = find_task_by_pid_ns(pid, &init_pid_ns);
	struct task_struct *suspect_parent_task = suspect_task->group_leader;
	int i = 0;

	suspect_tgid = task_tgid_nr(suspect_task);
	get_task_comm(suspect_task_comm, suspect_task);

	if (suspect_parent_task)
		get_task_comm(suspect_task_parent_comm, suspect_parent_task);
	else
		suspect_task_parent_comm[0] = '\0';

	

	for (i = 0; i < ARRAY_SIZE(kgsl_blocking_process_tbl); i++) {
		if (!((strncmp(suspect_task_comm,
				kgsl_blocking_process_tbl[i].name, TASK_COMM_LEN)) &&
				(strncmp(suspect_task_parent_comm,
				kgsl_blocking_process_tbl[i].name, TASK_COMM_LEN)))) {
			cankill=0;
			break;
		}
	}

	if (cankill) {
		KGSL_DRV_ERR(device, "We need to kill suspect process "
				"causing gpu hung, tgid=%d, name=%s, pname=%s\n",
				suspect_tgid, suspect_task_comm, suspect_task_parent_comm);

		do_send_sig_info(SIGKILL,
				SEND_SIG_FORCED, suspect_task, true);
		ret = 0;
	} else {
		KGSL_DRV_ERR(device, "We can't kill suspect process "
				"causing gpu hung due to stability, tgid=%d, name=%s, pname=%s\n",
				suspect_tgid, suspect_task_comm, suspect_task_parent_comm);
	}

	return ret;
}
#endif

void adreno_fault_panic(struct kgsl_device *device, unsigned int pid, int fault) {

	char fault_string[512];

	if (device->gpu_fault_no_panic > 0)
		device->gpu_fault_no_panic--;

	if(device->gpu_fault_no_panic)
		return;

	if (board_mfg_mode() || adreno_kill_suspect(device, pid)) {
		snprintf(fault_string, sizeof(fault_string), "Recoverable GPU Hang (0x%x)", fault);
		msleep(10000);
		panic(fault_string);
	}
}
