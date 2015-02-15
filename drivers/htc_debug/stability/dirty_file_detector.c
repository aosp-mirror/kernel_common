#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>

#define DIRTY_FILE_RECORD "/system/dirty_file_record"
static int dirty_file_detected = 0;
static mm_segment_t oldfs;

static void InitKernelEnv(void) {
	oldfs = get_fs();
	set_fs(KERNEL_DS);
}

static void DinitKernelEnv(void) {
	set_fs(oldfs);
}

static struct file *OpenFile(char *path, int flag, int mode){
	struct file *fp = 0;

	fp = filp_open(path, flag, mode);
	if (IS_ERR(fp)) {
		return NULL;
	} else
		return fp;
}

static int WriteFile(struct file *fp, const char *buf, int readlen) {
	if (fp->f_op && fp->f_op->read)
		return fp->f_op->write(fp, buf, readlen, &fp->f_pos);
	else
		return -1;
}

#if 0
/* nobody use this function yet, comment out */
static int ReadFile(struct file *fp, char *buf, int readlen)
{
	if (fp->f_op && fp->f_op->read)
		return fp->f_op->read(fp, buf, readlen, &fp->f_pos);
	else
		return -1;
}
#endif

static int CloseFile(struct file *fp) {
	filp_close(fp, NULL);
	return 0;
}

int mark_system_dirty(const char *file_name)
{
	struct file *fp;
	int ret = 0;
	static char tmp_str[8] = {0};

	dirty_file_detected = 1;

	/* if the previous file name is the same as this one, return directly */
	if (likely(!strncmp(tmp_str, file_name, sizeof(tmp_str) - 1))) {
		printk("%s: duplicated file %s, skip it\n", __func__, file_name);
		return 0;
	}

	/* copy previous string to prevent redundant recording */
	snprintf(tmp_str, sizeof(tmp_str), "%s", file_name);

	/* write file_name into /system/dirty_file */
	InitKernelEnv();
	fp = OpenFile(DIRTY_FILE_RECORD, O_RDWR | O_APPEND | O_CREAT, S_IRUSR | S_IRGRP | S_IROTH);
	if (fp != NULL) {
		WriteFile(fp, file_name, strlen(file_name));
		WriteFile(fp, "\n", 1);
		CloseFile(fp);
		printk("%s: succeeded to add %s\n", __func__, file_name);
	} else {
		printk("%s: failed to add %s\n", __func__, file_name);
		ret = -1;
	}
	DinitKernelEnv();
	return ret;
}

static int probe_system_dirty(void)
{
	struct file *fp;
	InitKernelEnv();
	fp = OpenFile(DIRTY_FILE_RECORD, O_RDONLY, 0);
	if (fp != NULL) {
		CloseFile(fp);
		dirty_file_detected = 1;
	}
	DinitKernelEnv();
	return dirty_file_detected;
}

int is_system_dirty(void)
{
	if (dirty_file_detected)
		return 1;
	else
		return probe_system_dirty();
}

#if 0
/* nobody use this function yet, comment out */
/* dump DIRTY_FILE_RECORD in kernel space */
static void dump_system_dirty_record(void)
{
	struct file *fp;
	char tmp_str[32] = {0};
	int ret = 0;
	if (!dirty_file_detected) {
		printk("%s: partition clean, skip it\n", __func__);
		return;
	}
	InitKernelEnv();
	fp = OpenFile(DIRTY_FILE_RECORD, O_RDONLY, 0);
	if (fp != NULL) {
		printk("%s +\n", __func__);
		while((ret = ReadFile(fp, tmp_str, sizeof(tmp_str))) > 0) {
			printk("%.*s", ret, tmp_str);
		}
		CloseFile(fp);
		printk("%s -\n", __func__);
	}
	DinitKernelEnv();
}
#endif
