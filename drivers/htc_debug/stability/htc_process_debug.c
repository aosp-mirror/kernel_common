#include <linux/init.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/slab.h>

#define process_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

struct kobject *process_kobj;
void show_block_state_work(struct work_struct *work);


static DEFINE_RWLOCK(task_comm_lock);
static LIST_HEAD(task_comm_list);
static DECLARE_DELAYED_WORK(show_block_state_struct, show_block_state_work);

struct task_comm {
	struct list_head list;
	char comm[TASK_COMM_LEN];
};

void show_block_state_work(struct work_struct *work){
	printk(KERN_WARNING "### Show All Blocked State ###\n");
	show_state_filter(TASK_UNINTERRUPTIBLE);
}

void send_signal_debug_dump(int sig, struct task_struct *t)
{

	struct task_comm *tc;
#if 0 
	
	if(sig == SIGKILL){
		if((t->state & TASK_UNINTERRUPTIBLE) || (t->exit_state & EXIT_ZOMBIE)){
			printk(KERN_WARNING "%s: %s(%d) send SIGKILL to %s(%d), but %s might not dead right now due to its %s state.\n",
				__func__,
				current->comm, current->pid,
				t->comm, t->pid, t->comm,
				(t->exit_state & EXIT_ZOMBIE) ? "Z" : "D");
			if(t->exit_state & EXIT_ZOMBIE)
				printk(KERN_WARNING "Please check its parent:%s(%d) or thread group.\n",
					t->real_parent->comm, t->real_parent->pid);
			
			schedule_delayed_work(&show_block_state_struct, 1 * HZ);
		}
	}
#endif
	if (t->comm) {
		read_lock(&task_comm_lock);
		list_for_each_entry(tc, &task_comm_list, list) {
			if (sig != SIGCHLD && tc->comm && (!strncmp(t->comm, tc->comm, TASK_COMM_LEN)) ) {
				
				if ((!strncmp(t->comm, "main", 4)) && t->parent->pid != 1 ) {
					break;
				}
				printk("%s: %s(%d)[group %s(%d), parent %s(%d)] send signal %d to %s(%d)\n", __func__,
					current->comm, current->pid,
					(current->group_leader) ? current->group_leader->comm : "Unknown",
					(current->group_leader) ? current->group_leader->pid : -1,
					(current->parent) ? current->parent->comm : "Unknown",
					(current->parent) ? current->parent->pid : -1,
					sig,
					t->comm, t->pid);
				dump_stack();
				break;
			}
		}
		read_unlock(&task_comm_lock);
	}
}

void do_group_exit_debug_dump(int exit_code)
{
	struct task_struct *t = (current->group_leader) ? current->group_leader : current;
	struct task_comm *tc;

	if (t->comm) {
		read_lock(&task_comm_lock);
		list_for_each_entry(tc, &task_comm_list, list) {
			if (tc->comm && (!strncmp(t->comm, tc->comm, TASK_COMM_LEN)) ) {
				
				if ((!strncmp(t->comm, "main", 4)) && t->parent->pid != 1 ) {
					break;
				}
				printk(KERN_INFO "%s: %s(%d)[group %s(%d) parent %s(%d)] call exit with code %d\n", __func__,
				current->comm, current->pid,
				(current->group_leader) ? current->group_leader->comm : "Unknown",
				(current->group_leader) ? current->group_leader->pid : -1,
				(current->parent) ? current->parent->comm : "Unknown",
				(current->parent) ? current->parent->pid : -1,
				exit_code);
				break;
			}
		}
		read_unlock(&task_comm_lock);
	}
}

static ssize_t task_comm_list_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	struct task_comm *tc, *tc1;

	if(n > 0 && n < TASK_COMM_LEN) {
		tc1 = kmalloc(sizeof(struct task_comm), GFP_KERNEL);
		memcpy(tc1->comm, buf, n);
		tc1->comm[n-1] = '\0';

		write_lock(&task_comm_lock);
		list_for_each_entry(tc, &task_comm_list, list) {
			if (!strcmp(tc->comm, tc1->comm)) {
				write_unlock(&task_comm_lock);
				kfree(tc1);
				printk("%s: %s is existed, so ignore it.\n", __func__, tc->comm);
				return n;
			}
		}

		list_add_tail(&tc1->list, &task_comm_list);
		write_unlock(&task_comm_lock);
		printk("%s: Add %s to monitor list tail successfully\n", __func__, tc1->comm);
	}

	return n;
}

static ssize_t task_comm_list_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	struct task_comm *tc;
	char *s = buf;

	read_lock(&task_comm_lock);
	list_for_each_entry(tc, &task_comm_list, list) {
		s += sprintf(s, "%s\n", tc->comm);
	}
	read_unlock(&task_comm_lock);

	return (s - buf);
}

process_attr(task_comm_list);

static struct attribute *g[] = {
	&task_comm_list_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

static int __init track_process_init(void)
{
	process_kobj = kobject_create_and_add("process", NULL);
	if (!process_kobj)
		return -ENOMEM;
	else
		return sysfs_create_group(process_kobj, &attr_group);
}

core_initcall(track_process_init);
