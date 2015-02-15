#include <linux/ctype.h>
#include <linux/module.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/utsname.h>
#include <linux/fdtable.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <mach/board_htc.h>

enum {
	FD_M_DEBUG_FAILURE    = BIT(0),
	FD_M_DEBUG_ERROR  = BIT(1),
	FD_M_DEBUG_NEW    = BIT(2),
	FD_M_DEBUG_ACCESS = BIT(3),
	FD_M_DEBUG_LOOKUP = BIT(4),
	FD_M_DEBUG_LOCK = BIT(5),
	FD_M_DEBUG_STATUS = BIT(6),
};
static int fd_m_debug_mask = FD_M_DEBUG_FAILURE;
module_param(fd_m_debug_mask, int, 0);

#define fd_m_attr(_name) \
	static struct kobj_attribute _name##_attr = {	\
		.attr	= {				\
			.name = __stringify(_name),	\
			.mode = 0644,			\
		},					\
		.show	= _name##_show,			\
		.store	= _name##_store,		\
	}
#define FD_ACTION_ADD	1
#define FD_ACTION_QUERY	0
#define FD_ACTION_DEL	-1

static spinlock_t fd_list_lock;

ssize_t fd_list_lock_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf);
ssize_t fd_list_lock_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n);
ssize_t fd_list_unlock_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf);
ssize_t fd_list_unlock_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n);

ssize_t fd_list_forced_unlock_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf);
ssize_t fd_list_forced_unlock_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n);

fd_m_attr(fd_list_lock);
fd_m_attr(fd_list_unlock);
fd_m_attr(fd_list_forced_unlock);

extern unsigned int get_tamper_sf(void);

static struct attribute *fd_attributes[] = {
	&fd_list_lock_attr.attr,
	&fd_list_unlock_attr.attr,
	&fd_list_forced_unlock_attr.attr,
	NULL,
};
static struct attribute_group fd_attr_group = {
	.attrs = fd_attributes,
};
struct fd_list_node {
	struct rb_node    node;
	int   pid;
	char  fd_mid[0];
};

struct rb_root fd_list_root;

void create_fd_list_entry(struct kobject *kobj)
{
	if (!kobj)
		return;
	if ((!get_tamper_sf())) {
		fd_list_lock_attr.attr.mode = 0666;
		fd_list_unlock_attr.attr.mode = 0666;
		fd_list_forced_unlock_attr.attr.mode = 0666;
	}
	if (sysfs_create_group(kobj, &fd_attr_group)) {
		printk(KERN_ERR
				"Unable to create fd_list attributes\n");
	} else {
		spin_lock_init(&fd_list_lock);
	}
}


int fd_check(unsigned int fd)
{
	struct file * filp;
	struct files_struct *files = current->files;
	struct fdtable *fdt;
    if (fd_m_debug_mask & FD_M_DEBUG_LOCK)
		printk("%s:file_spin_lock+\n", __func__);
	spin_lock(&files->file_lock);
	fdt = files_fdtable(files);

	if (fd >= fdt->max_fds || fd < 0)
		goto out_unlock;

	filp = fdt->fd[fd];
	if (!filp)
		goto out_unlock;

	if (fd_m_debug_mask & FD_M_DEBUG_LOCK)
		printk("%s:file_spin_lock-\n", __func__);
	spin_unlock(&files->file_lock);
	if (fd_m_debug_mask & FD_M_DEBUG_ERROR)
		printk("fd_check: fd=%d\n",fd);
	return 0;

out_unlock:
	if (fd_m_debug_mask & FD_M_DEBUG_LOCK)
		printk("%s:file_spin_lock-\n", __func__);
	spin_unlock(&files->file_lock);
	printk("fd error: invalid fd=%d\n",fd);
	return -1;
}

static int fd_monitor_start = 0;
static struct fd_list_node *lookup_fd_list(
		const char *buf, int action, int ignore_mid, int fdcheck)
{
	struct rb_node **p = &fd_list_root.rb_node;
	struct rb_node *parent = NULL;
	struct fd_list_node *l;
	int cur_pid = current->tgid;
	int diff = 0, fd_len = 0;
	int data_len = 0;
	const char *arg;
	char fd[10] = {0};
	unsigned int  fd_num = 0;
	int lock = 0;
	/* Find length of fd_mid */
	arg = buf;
	while (*arg && !isspace(*arg))
	{
		if(*arg == '_') {
			fd_len = arg - buf;
		}
		arg++;
	}

	if(!fd_len || buf[0] == '-') {
		printk("fd error : illegal fd %s\n", buf);
		goto bad_arg;
	} else if (fdcheck) {
		snprintf(fd, fd_len+1, buf);
		fd_num = simple_strtol(fd, (char **)&fd, 0);
		if(fd_check(fd_num))
			goto bad_arg;
	}
	data_len = arg - buf;

	if (data_len <= 0)
		goto bad_arg;

	if (fd_m_debug_mask & FD_M_DEBUG_LOCK)
		printk("%s:lock+\n", __func__);
	spin_lock_bh(&fd_list_lock);
	lock = 1;
	/* Lookup fd in rbtree */
	while (*p) {
		parent = *p;
		l = rb_entry(parent, struct fd_list_node, node);
		if (l->pid < cur_pid) {
			p = &(*p)->rb_right;
			continue;
		} else if (l->pid == cur_pid) {
			diff = strncmp(buf, l->fd_mid, ignore_mid?(fd_len + 1):data_len);
			/* avoid string partial match case, ex. buf = 123_45, l->fd_mid = 123_456 */
			if (!diff && l->fd_mid[data_len] != '\0'&& !ignore_mid)
				diff = -1;
		} else if (l->pid > cur_pid) {
			p = &(*p)->rb_left;
			continue;
		}

		if (fd_m_debug_mask & FD_M_DEBUG_ERROR)
			pr_info("lookup_fd_list: compare %d-%.*s %d-%s %d \n",
					cur_pid, data_len, buf, l->pid, l->fd_mid, diff);

		if (diff < 0) {
			p = &(*p)->rb_left;
			/* For specific unregister condition : fd matches but mid */
			if (action == FD_ACTION_DEL && !ignore_mid && !strncmp(buf, l->fd_mid, (fd_len + 1)))
				goto return_l;
		} else if (diff > 0) {
			p = &(*p)->rb_right;
			/* For specific unregister condition : fd matches but mid */
			if (action == FD_ACTION_DEL && !ignore_mid && !strncmp(buf, l->fd_mid, (fd_len + 1)))
				goto return_l;
		} else {
			if (action == FD_ACTION_DEL) {
				rb_erase(&l->node, &fd_list_root);
				kfree(l);
				l = NULL;
			}
			if(action == FD_ACTION_ADD) {
				printk("fd error: %s(%d) try to register an exist fd=%s\n",current->comm, current->tgid, buf);
				goto bad_arg;
			}
			goto return_l;
		}
	}

	/* Allocate and add new fd to rbtree */
	if (action != FD_ACTION_ADD) {
		if (fd_m_debug_mask & FD_M_DEBUG_ERROR)
			pr_info("lookup_fd_list: %.*s not found\n",
					data_len, buf);
		goto bad_arg;
	}
	l = kzalloc(sizeof(*l) + data_len + 1, GFP_ATOMIC);
	if (l == NULL) {
		if (fd_m_debug_mask & FD_M_DEBUG_FAILURE)
			pr_err("lookup_fd_list: failed to action "
					"memory for %.*s\n", data_len, buf);
		goto bad_arg;
	}
	memcpy(l->fd_mid, buf, data_len);
	l->pid = cur_pid;
	if (fd_m_debug_mask & FD_M_DEBUG_NEW)
		pr_info("lookup_fd_list: new fd %s\n", l->fd_mid);
	rb_link_node(&l->node, parent, p);
	rb_insert_color(&l->node, &fd_list_root);

return_l:
	if (fd_m_debug_mask & FD_M_DEBUG_LOCK)
		printk("%s:lock-\n", __func__);
	if(lock)
		spin_unlock_bh(&fd_list_lock);
	return l;

bad_arg:
	if (fd_m_debug_mask & FD_M_DEBUG_LOCK)
		printk("%s:lock-\n", __func__);
	if(lock)
		spin_unlock_bh(&fd_list_lock);
	if (fd_m_debug_mask & FD_M_DEBUG_ERROR)
		pr_info("lookup_fd_list: fd, %.*s, bad arg, %s\n",
				data_len, buf, arg);
	return ERR_PTR(-EINVAL);
}

ssize_t fd_list_lock_show(
		struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	struct rb_node *n;
	struct fd_list_node *l;
    if (fd_m_debug_mask & FD_M_DEBUG_LOCK)
		printk("%s:lock+\n", __func__);
	spin_lock_bh(&fd_list_lock);
	printk("fd_list:<pid>_<fd>_<mid>\n");
	for (n = rb_first(&fd_list_root); n != NULL; n = rb_next(n)) {
		l = rb_entry(n, struct fd_list_node, node);
		printk("fd_list:%d_%s\n", l->pid, l->fd_mid);
	}
	if (fd_m_debug_mask & FD_M_DEBUG_LOCK)
		printk("%s:lock-\n", __func__);
	spin_unlock_bh(&fd_list_lock);
	return (s - buf);
}

ssize_t fd_list_lock_store(
		struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	struct fd_list_node *l;
    if (fd_m_debug_mask & FD_M_DEBUG_STATUS)
		printk("fd_list register: pid=%d, fd_mid=%s\n", current->tgid, buf);
	fd_monitor_start = 1;
	l = lookup_fd_list(buf, FD_ACTION_ADD, 1, 1);
	if (IS_ERR(l)) {
		n = PTR_ERR(l);
		goto bad_fd;
	}

	if (fd_m_debug_mask & FD_M_DEBUG_ACCESS)
		pr_info("fd_list_lock_store: %s\n", l->fd_mid);

bad_fd:
	return n;
}

ssize_t fd_list_unlock_show(
		struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return 0;
}

ssize_t fd_list_forced_unlock_show(
		struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return 0;
}

int in_fd_list(const int fd,const int mid)
{
	struct fd_list_node *l;
	char str[11] = {0};
	int ret = 0;
	if (fd > 1024 || !fd_monitor_start)
		return 0;
	snprintf(str , 10, "%d_%d", fd, mid);
	if (fd_m_debug_mask & FD_M_DEBUG_ERROR)
		printk("%s: string = %s\n", __func__, str);
	l = lookup_fd_list(str, FD_ACTION_QUERY, 1, 0);
	if (IS_ERR(l)) {
		if (fd_m_debug_mask & FD_M_DEBUG_ERROR)
			printk("%s: not found pid=%d mid=%d fd=%d\n", __func__, current->tgid, mid, fd);
		ret = 0;
	} else {
		printk("%s: found pid=%d fd=%d\n", __func__, l->pid, fd);
		ret = 1;
	}
	return ret;
}
EXPORT_SYMBOL(in_fd_list);

ssize_t fd_list_unlock_store(
		struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	struct fd_list_node *l;
	int resend_abrtsig = 0;
	if (fd_m_debug_mask & FD_M_DEBUG_STATUS)
		printk("fd_list unregister: pid=%d, fd_mid=%s\n", current->tgid, buf);
	l = lookup_fd_list(buf, FD_ACTION_DEL, 0, 1);

	if (IS_ERR(l)) {
		n = PTR_ERR(l);
		printk("fd error: %s(%d) tries to unregister an invalid fd= %s\n", current->comm, current->tgid, buf);
		goto bad_fd;
	} else if (l != NULL) {
		printk("fd error: %s(%d) improper module tries to unregister %s\n", current->comm, current->tgid, buf);
		force_sig(SIGABRT, current);
		resend_abrtsig = 1;
		n = 0;
		goto bad_fd;
	}

bad_fd:
	if (resend_abrtsig)
		force_sig(SIGABRT, current);
	return n;
}

ssize_t fd_list_forced_unlock_store(
		struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	struct fd_list_node *l;

	printk("fd_list froced unregister: pid=%d, fd_mid=%s\n", current->tgid, buf);
	l = lookup_fd_list(buf, FD_ACTION_DEL, 1, 0);
	return n;
}

int clean_fd_list(const int cur_pid, const int callfrom)
{
	struct rb_node **p = &fd_list_root.rb_node;
	struct rb_node *parent = NULL;
	struct fd_list_node *l;
	int    cleaned = 0;
	if(!fd_monitor_start)
		return 0;
    if (fd_m_debug_mask & FD_M_DEBUG_LOCK)
		printk("%s:lock+\n", __func__);
	spin_lock_bh(&fd_list_lock);
	/* Lookup fd in rbtree */
	while (*p) {
		parent = *p;
		l = rb_entry(parent, struct fd_list_node, node);
		if (l->pid < cur_pid) {
			p = &(*p)->rb_right;
			continue;
		} else if (l->pid == cur_pid) {
			rb_erase(&l->node, &fd_list_root);
			kfree(l);
			l = NULL;
			cleaned = 1;
		} else if (l->pid > cur_pid) {
			p = &(*p)->rb_left;
			continue;
		}
	}
	if (fd_m_debug_mask & FD_M_DEBUG_LOCK)
		printk("%s:lock-\n", __func__);
	spin_unlock_bh(&fd_list_lock);
	if ((fd_m_debug_mask & FD_M_DEBUG_STATUS) && cleaned != 0)
		printk("clean_fd_list: pid=%d, from=%d\n", cur_pid, callfrom);
	return 0;
}
EXPORT_SYMBOL(clean_fd_list);
