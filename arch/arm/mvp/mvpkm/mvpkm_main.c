/*
 * Linux 2.6.32 and later Kernel module for VMware MVP Hypervisor Support
 *
 * Copyright (C) 2010-2013 VMware, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; see the file COPYING.  If not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#line 5


#define __KERNEL_SYSCALLS__
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/syscalls.h>
#include <linux/kmod.h>
#include <linux/socket.h>
#include <linux/net.h>
#include <linux/skbuff.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/smp.h>
#include <linux/capability.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/sysfs.h>
#include <linux/debugfs.h>
#include <linux/pid.h>
#include <linux/highmem.h>
#include <linux/syscalls.h>
#include <linux/swap.h>

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif

#include <net/sock.h>

#include <asm/cacheflush.h>
#include <asm/memory.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <linux/uaccess.h>

#include "mvp.h"
#include "mvp_version.h"
#include "mvpkm_types.h"
#include "mvpkm_private.h"
#include "mvpkm_kernel.h"
#include "actions.h"
#include "wscalls.h"
#include "arm_inline.h"
#include "tsc.h"
#include "mksck_kernel.h"
#include "mmu_types.h"
#include "mvp_timer.h"
#include "qp.h"
#include "qp_host_kernel.h"
#include "cpufreq_kernel.h"
#include "mvpkm_comm_ev.h"
#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER
#include "mvp_balloon.h"
#endif



static _Bool
LockedListAdd(struct MvpkmVM *vm,
	      __u32 mpn,
	      __u32 order,
	      PhysMem_RegionType forRegion);
static _Bool LockedListDel(struct MvpkmVM *vm, __u32 mpn);
static void  LockedListUnlockAll(struct MvpkmVM *vm);
static _Bool LockedListLookup(struct MvpkmVM *vm, __u32 mpn);
static int   SetupMonitor(struct MvpkmVM *vm);
static int   RunMonitor(struct MvpkmVM *vm);
static MPN
AllocZeroedFreePages(struct MvpkmVM *vm,
		     uint32 order,
		     _Bool highmem,
		     PhysMem_RegionType forRegion,
		     HKVA *hkvaRet);
static HKVA  MapWSPHKVA(struct MvpkmVM *vm, HkvaMapInfo *mapInfo);
static void  UnmapWSPHKVA(struct MvpkmVM *vm);
static int   MvpkmWaitForInt(struct MvpkmVM *vm, _Bool suspend);
static void  ReleaseVM(struct MvpkmVM *vm);

uid_t Mvpkm_vmwareUid;
EXPORT_SYMBOL(Mvpkm_vmwareUid);
gid_t Mvpkm_vmwareGid;
EXPORT_SYMBOL(Mvpkm_vmwareGid);

static int lowmemAdjSize;
static int lowmemAdj[6];

static DECLARE_BITMAP(vcpuAffinity, NR_CPUS);

struct cpumask inMonitor;

static struct kobject *mvpkmKObj;

static struct kobject *balloonKObj;

static ssize_t
version_show(struct kobject *kobj,
	     struct kobj_attribute *attr,
	     char *buf)
{
	return snprintf(buf, PAGE_SIZE,
			MVP_VERSION_FORMATSTR "\n", MVP_VERSION_FORMATARGS);
}

static struct kobj_attribute versionAttr = __ATTR_RO(version);

static ssize_t
background_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
#ifndef CONFIG_ANDROID_LOW_MEMORY_KILLER
	return snprintf(buf, PAGE_SIZE, "0\n");
#else
	
	FATAL_IF(lowmemAdjSize != 6);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			Balloon_AndroidBackgroundPages(lowmemAdj[4]));
#endif
}

static struct kobj_attribute backgroundAttr = __ATTR_RO(background);

static ssize_t
other_file_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	int32 other_file = 0;

#ifndef LOWMEMKILLER_VARIANT
#define LOWMEMKILLER_VARIANT 0
#endif

#ifndef LOWMEMKILLER_MD5
#define LOWMEMKILLER_MD5 0
#endif

#ifndef LOWMEMKILLER_SHRINK_MD5
#define LOWMEMKILLER_SHRINK_MD5 0
#endif

#if LOWMEMKILLER_VARIANT == 1
	other_file = global_page_state(NR_ACTIVE_FILE) +
		     global_page_state(NR_INACTIVE_FILE);
#elif LOWMEMKILLER_VARIANT == 2
	other_file = global_page_state(NR_FILE_PAGES);
#elif LOWMEMKILLER_VARIANT == 3
	other_file = global_page_state(NR_FILE_PAGES) -
		     global_page_state(NR_SHMEM);
#elif LOWMEMKILLER_VARIANT == 4
	other_file = global_page_state(NR_FREE_PAGES) +
		     global_page_state(NR_FILE_PAGES);
#elif LOWMEMKILLER_VARIANT == 5
	other_file = global_page_state(NR_FILE_PAGES) -
		     global_page_state(NR_SHMEM);
#elif LOWMEMKILLER_VARIANT == 6
	other_file = global_page_state(NR_FILE_PAGES) -
		     global_page_state(NR_SHMEM) - global_page_state(NR_MLOCK);
#elif LOWMEMKILLER_VARIANT == 8
     if (global_page_state(NR_SHMEM) + global_page_state(NR_MLOCK) + total_swapcache_pages <
              global_page_state(NR_FILE_PAGES))
              other_file = global_page_state(NR_FILE_PAGES) -
                                             global_page_state(NR_SHMEM) -
                                             global_page_state(NR_MLOCK) -
                                             total_swapcache_pages;
     else
              other_file = 0;
#elif defined(NONANDROID)
#else
#warning "Unknown lowmemorykiller variant in hosted/module/mvpkm_main.c, " \
	 "falling back on default (see other_file_show for the remedy)"
	 other_file = global_page_state(NR_FILE_PAGES);
#endif

#define _STRINGIFY(x) (#x)
#define STRINGIFY(x) _STRINGIFY(x)
	return snprintf(buf, PAGE_SIZE, "%d %d %s %s\n", other_file,
			LOWMEMKILLER_VARIANT, STRINGIFY(LOWMEMKILLER_MD5),
			STRINGIFY(LOWMEMKILLER_SHRINK_MD5));
#undef _STRINGIFY
#undef STRINGIFY
}

static struct kobj_attribute otherFileAttr = __ATTR_RO(other_file);


static struct dentry *mvpDebugDentry;

static int
InMonitorShow(struct seq_file *m,
	      void *private)
{
	seq_bitmap_list(m, cpumask_bits(&inMonitor), nr_cpumask_bits);
	seq_puts(m, "\n");
	return 0;
}

static int
InMonitorOpen(struct inode *inode,
	      struct file *file)
{
	return single_open(file, InMonitorShow, NULL);
}

static const struct file_operations inMonitorFops = {
	.open = InMonitorOpen,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct kset *mvpkmKSet;

static ssize_t
MvpkmAttrShow(struct kobject *kobj,
	      struct attribute *attr,
	      char *buf);
static ssize_t
MvpkmAttrStore(struct kobject *kobj,
	       struct attribute *attr,
	       const char *buf,
	       size_t count);

static void MvpkmKObjRelease(struct kobject *kobj)
	__attribute__((optimize("-fomit-frame-pointer")));



static void
MvpkmKObjRelease(struct kobject *kobj)
{
	struct MvpkmVM *vm = container_of(kobj, struct MvpkmVM, kobj);

	ReleaseVM(vm);

	module_put(THIS_MODULE);
}


static const struct sysfs_ops mvpkmSysfsOps = {
	.show = MvpkmAttrShow,
	.store = MvpkmAttrStore
};

static struct attribute mvpkmLockedPagesAttr = {
	.name = "locked_pages",
	.mode = 0444,
};

static struct attribute mvpkmBalloonWatchdogAttr = {
	.name = "balloon_watchdog",
	.mode = 0444
};

static struct attribute mvpkmMonitorAttr = {
	.name = "monitor",
	.mode = 0400,
};

static struct attribute *mvpkmDefaultAttrs[] = {
	&mvpkmLockedPagesAttr,
	&mvpkmBalloonWatchdogAttr,
	&mvpkmMonitorAttr,
	NULL,
};

static struct kobj_type mvpkmKType = {
	.sysfs_ops = &mvpkmSysfsOps,
	.release = MvpkmKObjRelease,
	.default_attrs = mvpkmDefaultAttrs,
};

#ifndef CONFIG_SYS_HYPERVISOR
struct kobject *hypervisor_kobj;
EXPORT_SYMBOL_GPL(hypervisor_kobj);
#endif



extern struct kobject *kset_find_obj(struct kset *, const char *)
	__attribute__((weak));



struct kobject *
kset_find_obj(struct kset *kset,
	      const char *name)
{
	struct kobject *k;
	struct kobject *ret = NULL;

	spin_lock(&kset->list_lock);
	list_for_each_entry(k, &kset->list, entry) {
		if (kobject_name(k) && !strcmp(kobject_name(k), name)) {
			ret = kobject_get(k);
			break;
		}
	}
	spin_unlock(&kset->list_lock);
	return ret;
}



struct kset *
Mvpkm_FindVMNamedKSet(int vmID,
		      const char *name)
{
	struct MvpkmVM *vm;
	struct kobject *kobj;
	char vmName[32] = {}; 
	struct kset *res = NULL;

	if (!mvpkmKSet)
		return NULL;

	snprintf(vmName, sizeof(vmName), "%d", vmID);
	
	vmName[sizeof(vmName) - 1] = '\0';

	kobj = kset_find_obj(mvpkmKSet, vmName);
	if (!kobj)
		return NULL;

	vm = container_of(kobj, struct MvpkmVM, kobj);

	if (!strcmp(name, "devices"))
		res = kset_get(vm->devicesKSet);
	else if (!strcmp(name, "misc"))
		res = kset_get(vm->miscKSet);

	kobject_put(kobj);
	return res;
}
EXPORT_SYMBOL(Mvpkm_FindVMNamedKSet);




MODULE_LICENSE("GPL"); 

static int MvpkmFault(struct vm_area_struct *vma, struct vm_fault *vmf);


static struct vm_operations_struct mvpkmVMOps = {
	.fault = MvpkmFault
};

static long
MvpkmUnlockedIoctl(struct file *filep,
		   unsigned int cmd,
		   unsigned long arg);
static int MvpkmOpen(struct inode *inode, struct file *filp);
static int MvpkmRelease(struct inode *inode, struct file *filp);
static int MvpkmMMap(struct file *filp, struct vm_area_struct *vma);

static const struct file_operations mvpkmFileOps = {
	.owner            = THIS_MODULE,
	.unlocked_ioctl   = MvpkmUnlockedIoctl,
	.open             = MvpkmOpen,
	.release          = MvpkmRelease,
	.mmap             = MvpkmMMap
};

static struct miscdevice mvpkmDev = {
	.minor  = 165,
	.name   = "mvpkm",
	.fops   = &mvpkmFileOps
};

static struct pid *initTgid;


#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER
static int MvpkmShrink(struct shrinker *this, struct shrink_control *sc);

static struct shrinker mvpkmShrinker = {
	.shrink = MvpkmShrink,
	.seeks = DEFAULT_SEEKS
};
#endif

module_param_array(vcpuAffinity, ulong, NULL, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(vcpuAffinity, "vCPU affinity");



static int __init
MvpkmInit(void)
{
	int err = 0;
	_Bool mksckInited = false;
	_Bool cpuFreqInited = false;

	pr_info("Mvpkm: " MVP_VERSION_FORMATSTR "\n", MVP_VERSION_FORMATARGS);
	pr_info("Mvpkm: started from process %s tgid=%d, pid=%d\n",
		current->comm, task_tgid_vnr(current), task_pid_vnr(current));

	if (bitmap_empty(vcpuAffinity, nr_cpumask_bits))
		bitmap_copy(vcpuAffinity, cpumask_bits(cpu_possible_mask),
			    nr_cpumask_bits);

	err = misc_register(&mvpkmDev);
	if (err)
		return -ENOENT;

	err = Mksck_Init();
	if (err)
		goto error;
	else
		mksckInited = true;

	mksckInited = true;
	QP_HostInit();

	CpuFreq_Init();
	cpuFreqInited = true;

	initTgid = get_pid(task_tgid(current));

#ifndef CONFIG_SYS_HYPERVISOR
	hypervisor_kobj = kobject_create_and_add("hypervisor", NULL);
	if (!hypervisor_kobj) {
		err = -ENOMEM;
		goto error;
	}
#endif

	mvpkmKObj = kobject_create_and_add("mvp", hypervisor_kobj);
	if (!mvpkmKObj) {
		err = -ENOMEM;
		goto error;
	}
	balloonKObj = kobject_create_and_add("lowmem", mvpkmKObj);
	if (!balloonKObj) {
		err = -ENOMEM;
		goto error;
	}
	mvpkmKSet = kset_create_and_add("vm", NULL, mvpkmKObj);
	if (!mvpkmKSet) {
		err = -ENOMEM;
		goto error;
	}

	err = sysfs_create_file(mvpkmKObj, &versionAttr.attr);
	if (err)
		goto error;

	err = sysfs_create_file(balloonKObj, &backgroundAttr.attr);
	if (err)
		goto error;

	err = sysfs_create_file(balloonKObj, &otherFileAttr.attr);
	if (err)
		goto error;

#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER
	register_shrinker(&mvpkmShrinker);
#endif

	
	mvpDebugDentry = debugfs_create_dir("mvp", NULL);
	if (mvpDebugDentry) {
		debugfs_create_file("inMonitor", S_IRUGO,
				    mvpDebugDentry, NULL, &inMonitorFops);
		MksckPageInfo_Init(mvpDebugDentry);
	}

	return 0;

error:
	if (mvpkmKSet)
		kset_unregister(mvpkmKSet);

	if (balloonKObj) {
		kobject_del(balloonKObj);
		kobject_put(balloonKObj);
	}

	if (mvpkmKObj) {
		kobject_del(mvpkmKObj);
		kobject_put(mvpkmKObj);
	}

#ifndef CONFIG_SYS_HYPERVISOR
	if (hypervisor_kobj) {
		kobject_del(hypervisor_kobj);
		kobject_put(hypervisor_kobj);
	}
#endif

	if (cpuFreqInited)
		CpuFreq_Exit();

	if (mksckInited)
		Mksck_Exit();

	if (initTgid)
		put_pid(initTgid);

	misc_deregister(&mvpkmDev);
	return err;
}

void
MvpkmExit(void)
{
	PRINTK("MvpkmExit called !\n");

	if (mvpDebugDentry)
		debugfs_remove_recursive(mvpDebugDentry);

#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER
	unregister_shrinker(&mvpkmShrinker);
#endif

	kset_unregister(mvpkmKSet);
	kobject_del(balloonKObj);
	kobject_put(balloonKObj);
	kobject_del(mvpkmKObj);
	kobject_put(mvpkmKObj);
#ifndef CONFIG_SYS_HYPERVISOR
	kobject_del(hypervisor_kobj);
	kobject_put(hypervisor_kobj);
#endif

	CpuFreq_Exit();

	Mksck_Exit();

	put_pid(initTgid);

	misc_deregister(&mvpkmDev);
}

module_init(MvpkmInit);
module_exit(MvpkmExit);

module_param_array_named(lowmemAdj, lowmemAdj, int, &lowmemAdjSize,
			 S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(lowmemAdj,
		 "copy of /sys/module/lowmemorykiller/parameters/adj");

#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER
static void
WatchdogCB(unsigned long data)
{
	struct MvpkmVM *vm = (struct MvpkmVM *)data;

	pr_err("Balloon watchdog expired (%d s)!\n",
	       BALLOON_WATCHDOG_TIMEOUT_SECS);
	vm->watchdogTriggered = true;

	Mvpkm_WakeGuest(vm, ACTION_ABORT);
}

static int
MvpkmShrink(struct shrinker *this,
	    struct shrink_control *sc)
{
	uint32 locked = 0;
	struct kobject *k;
	int nrToScan = sc->nr_to_scan;

	spin_lock(&mvpkmKSet->list_lock);

	list_for_each_entry(k, &mvpkmKSet->list, entry) {
		struct MvpkmVM *vm = container_of(k, struct MvpkmVM, kobj);

		locked += ATOMIC_GETO(vm->usedPages);

		if (nrToScan > 0 &&
		    down_read_trylock(&vm->wspSem)) {
			if (vm->wsp) {
				if (vm->balloonWDEnabled) {
					struct timer_list *t =
						&vm->balloonWDTimer;

					if (!timer_pending(t)) {
						t->data = (unsigned long)vm;
						t->function = WatchdogCB;
						t->expires = jiffies +
					    BALLOON_WATCHDOG_TIMEOUT_SECS * HZ;
						add_timer(t);
					}
				}

				Mvpkm_WakeGuest(vm, ACTION_BALLOON);
			}

			up_read(&vm->wspSem);
		}
	}

	spin_unlock(&mvpkmKSet->list_lock);

	return locked;
}
#endif

int
MvpkmOpen(struct inode *inode,
	  struct file *filp)
{
	struct MvpkmVM *vm;

	if (initTgid != task_tgid(current)) {
		pr_err("%s: MVPKM can be opened only from MVPD (process %d).\n",
			__func__, pid_vnr(initTgid));
		return -EPERM;
	}
	pr_debug("%s: Allocating an MvpkmVM structure from process %s tgid=%d, pid=%d\n",
		 __func__, current->comm, task_tgid_vnr(current),
		 task_pid_vnr(current));

	vm = kmalloc(sizeof(struct MvpkmVM), GFP_KERNEL);
	if (!vm)
		return -ENOMEM;

	memset(vm, 0, sizeof(*vm));

	init_timer(&vm->balloonWDTimer);
	init_rwsem(&vm->lockedSem);
	init_rwsem(&vm->wspSem);
	init_rwsem(&vm->monThreadTaskSem);
	vm->monThreadTask = NULL;
	vm->isMonitorInited = false;

	filp->private_data = vm;

	if (!Mvpkm_vmwareUid)
		current_uid_gid(&Mvpkm_vmwareUid, &Mvpkm_vmwareGid);

	return 0;
}

static void
ReleaseVM(struct MvpkmVM *vm)
{
	del_timer_sync(&vm->balloonWDTimer);

	down_write(&vm->wspSem);

	if (vm->isMonitorInited) {
		MonitorTimer_Request(&vm->monTimer, 0);
		Mksck_WspRelease(vm->wsp);
		vm->wsp = NULL;
#ifdef CONFIG_HAS_WAKELOCK
		wake_lock_destroy(&vm->wakeLock);
#endif
	}

	up_write(&vm->wspSem);

	LockedListUnlockAll(vm);

	UnmapWSPHKVA(vm);


	kfree(vm);
}

int
MvpkmRelease(struct inode *inode,
	     struct file *filp)
{
	struct MvpkmVM *vm = filp->private_data;

	if (vm->isMonitorInited) {
		ASSERT(vm->wsp);
		QP_DetachAll(vm->wsp->guestId);
	}


	kset_unregister(vm->miscKSet);
	kset_unregister(vm->devicesKSet);

	if (vm->haveKObj) {

		kobject_del(&vm->kobj);
		kobject_put(&vm->kobj);
	} else {
		ReleaseVM(vm);
	}

	filp->private_data = NULL;

	pr_info("%s: Released MvpkmVM structure from process %s tgid=%d, pid=%d\n",
		__func__, current->comm, task_tgid_vnr(current),
		task_pid_vnr(current));

	return 0;
}

static int
MvpkmFault(struct vm_area_struct *vma,
	   struct vm_fault *vmf)
{
	unsigned long address = (unsigned long)vmf->virtual_address;
	MPN mpn = vmf->pgoff;
	struct MvpkmVM *vm = vma->vm_file->private_data;


	if (!pfn_valid(mpn)) {
		pr_err("MvpkmMMap: Failed to insert %x @ %lx, mpn invalid\n",
		       mpn, address);
	} else if (LockedListLookup(vm, mpn)) {
		if (vm_insert_page(vma, address, pfn_to_page(mpn)) == 0)
			return VM_FAULT_NOPAGE;

		pr_err("MvpkmMMap: Failed to insert %x @ %lx\n",
		       mpn, address);
	} else if (MksckPage_LookupAndInsertPage(vma, address, mpn) == 0) {
		return VM_FAULT_NOPAGE;
	}

	if (vm->stubPageMPN) {
		if (vm_insert_page(vma, address,
				   pfn_to_page(vm->stubPageMPN)) == 0) {
			pr_info("MvpkmMMap: mapped the stub page at %x @ %lx\n",
				mpn, address);
			return VM_FAULT_NOPAGE;
		}

		pr_err("MvpkmMMap: Could not insert stub page %x @ %lx\n",
		       mpn, address);
	}

	return VM_FAULT_SIGBUS;
}

static ssize_t
MvpkmAttrShow(struct kobject *kobj,
	      struct attribute *attr,
	      char *buf)
{
	if (attr == &mvpkmLockedPagesAttr) {
		struct MvpkmVM *vm = container_of(kobj, struct MvpkmVM, kobj);

		return snprintf(buf, PAGE_SIZE, "%d\n",
				ATOMIC_GETO(vm->usedPages));
	} else if (attr == &mvpkmMonitorAttr) {
		struct MvpkmVM *vm = container_of(kobj, struct MvpkmVM, kobj);

		return snprintf(buf, PAGE_SIZE, "hostActions %x callno %d\n",
				ATOMIC_GETO(vm->wsp->hostActions),
				WSP_Params(vm->wsp)->callno);
	} else if (attr == &mvpkmBalloonWatchdogAttr) {
		struct MvpkmVM *vm = container_of(kobj, struct MvpkmVM, kobj);

		vm->balloonWDEnabled = true;
		del_timer_sync(&vm->balloonWDTimer);

		buf[0] = 1;
		return 1;
	} else {
		return -EPERM;
	}
}

static ssize_t
MvpkmAttrStore(struct kobject *kobj,
	       struct attribute *attr,
	       const char *buf,
	       size_t count)
{
	return -EPERM;
}

static int
MvpkmMMap(struct file *filp,
	  struct vm_area_struct *vma)
{
	vma->vm_ops = &mvpkmVMOps;

	return 0;
}

#ifdef CONFIG_ARM_LPAE
static void
DetermineMemAttrLPAE(ARM_MemAttrNormal *attribMAN)
{
	HKVA hkva = __get_free_pages(GFP_KERNEL, 0);

	ARM_LPAE_L3D *pt = (ARM_LPAE_L3D *)hkva;
	ARM_LPAE_L3D *kernL3D = &pt[0], *userL3D = &pt[1];
	uint32 attr, mair0, mair1;

	set_pte_ext((pte_t *)kernL3D, pfn_pte(0, PAGE_KERNEL), 0);
	set_pte_ext((pte_t *)userL3D, pfn_pte(0, PAGE_NONE), 0);

	pr_info("DetermineMemAttr: Kernel L3D AttrIndx=%x SH=%x\n",
		kernL3D->blockS1.attrIndx, kernL3D->blockS1.sh);

	pr_info("DetermineMemAttr: User   L3D AttrIndx=%x SH=%x\n",
		userL3D->blockS1.attrIndx, userL3D->blockS1.sh);

	ASSERT(kernL3D->blockS1.attrIndx == userL3D->blockS1.attrIndx);
	ASSERT(kernL3D->blockS1.sh == userL3D->blockS1.sh);

	switch (kernL3D->blockS1.sh) {
	case 0:
		attribMAN->share = ARM_SHARE_ATTR_NONE;
		break;
	case 2:
		attribMAN->share = ARM_SHARE_ATTR_OUTER;
		break;
	case 3:
		attribMAN->share = ARM_SHARE_ATTR_INNER;
		break;
	default:
		FATAL();
	}

	ARM_MRC_CP15(MAIR0, mair0);
	ARM_MRC_CP15(MAIR1, mair1);

	attr = MVP_EXTRACT_FIELD(kernL3D->blockS1.attrIndx >= 4 ? mair1 : mair0,
				 8 * (kernL3D->blockS1.attrIndx % 4),
				 8);

#define MAIR_ATTR_2_CACHE_ATTR(x, y) \
	do { \
		switch (x) { \
		case 2: \
			(y) = ARM_CACHE_ATTR_NORMAL_WT; \
			break; \
		case 3: \
			(y) = ARM_CACHE_ATTR_NORMAL_WB; \
			break; \
		default: \
			FATAL(); \
		} \
	} while (0)

	MAIR_ATTR_2_CACHE_ATTR(MVP_EXTRACT_FIELD(attr, 2, 2),
			       attribMAN->innerCache);
	MAIR_ATTR_2_CACHE_ATTR(MVP_EXTRACT_FIELD(attr, 6, 2),
			       attribMAN->outerCache);

#undef MAIR_ATTR_2_CACHE_ATTR

	pr_info("DetermineMemAttr: innerCache %x outerCache %x share %x\n",
		attribMAN->innerCache,
		attribMAN->outerCache,
		attribMAN->share);

	free_pages(hkva, 0);
}

#else

static void
DetermineMemAttrNonLPAE(ARM_L2D *attribL2D,
			ARM_MemAttrNormal *attribMAN)
{
	HKVA hkva = __get_free_pages(GFP_KERNEL, 0);
	uint32 sctlr;
	ARM_L2D *pt = (ARM_L2D *)hkva;
	ARM_L2D *kernL2D = &pt[0], *userL2D = &pt[1];

	const uint32 set_pte_ext_offset = 0;

	set_pte_ext((pte_t *)(kernL2D + set_pte_ext_offset/sizeof(ARM_L2D)),
		    pfn_pte(0, PAGE_KERNEL),
		    0);
	set_pte_ext((pte_t *)(userL2D + set_pte_ext_offset/sizeof(ARM_L2D)),
		    pfn_pte(0, PAGE_NONE),
		    0);

	kernL2D += 2048/sizeof(ARM_L2D);
	userL2D += 2048/sizeof(ARM_L2D);

	pr_info("DetermineMemAttr: Kernel L2D TEX=%x CB=%x S=%x\n",
		kernL2D->small.tex,
		kernL2D->small.cb,
		kernL2D->small.s);

	pr_info("DetermineMemAttr: User   L2D TEX=%x CB=%x S=%x\n",
		userL2D->small.tex,
		userL2D->small.cb,
		userL2D->small.s);

	ASSERT((kernL2D->small.tex & 1) == (userL2D->small.tex & 1));
	ASSERT(kernL2D->small.cb == userL2D->small.cb);
	ASSERT(kernL2D->small.s == userL2D->small.s);

	*attribL2D = *kernL2D;


	ARM_MRC_CP15(CONTROL_REGISTER, sctlr);

	if (sctlr & ARM_CP15_CNTL_TRE) {
		uint32 prrr, nmrr, indx, type;
		uint32 innerCache, outerCache, outerShare, share;

		pr_info("DetermineMemAttr: TEX remapping enabled\n");

		ARM_MRC_CP15(PRIMARY_REGION_REMAP, prrr);
		ARM_MRC_CP15(NORMAL_MEMORY_REMAP, nmrr);

		pr_info("DetermineMemAttr: PRRR=%x NMRR=%x\n",
			prrr, nmrr);


		indx = (MVP_BIT(kernL2D->small.tex, 0) << 2) |
		       kernL2D->small.cb;

		type = MVP_EXTRACT_FIELD(prrr, 2 * indx, 2);
		ASSERT(type == 2);

		innerCache = MVP_EXTRACT_FIELD(nmrr, 2 * indx, 2);
		outerCache = MVP_EXTRACT_FIELD(nmrr, 16 + 2 * indx, 2);
		outerShare = !MVP_BIT(prrr, 24 + indx);
		share = MVP_BIT(prrr, 18 + kernL2D->small.s);

		pr_info("DetermineMemAttr: type %x innerCache %x outerCache %x"\
			" share %x outerShare %x\n",
			type, innerCache, outerCache, share, outerShare);

		if (share) {
			if (outerShare)
				attribMAN->share = ARM_SHARE_ATTR_OUTER;
			else
				attribMAN->share = ARM_SHARE_ATTR_INNER;
		} else {
			attribMAN->share = ARM_SHARE_ATTR_NONE;
		}

		attribMAN->innerCache = innerCache;
		attribMAN->outerCache = outerCache;
	} else {
		NOT_IMPLEMENTED_JIRA(1849);
	}

	free_pages(hkva, 0);
}
#endif

long
MvpkmUnlockedIoctl(struct file  *filp,
		   unsigned int  cmd,
		   unsigned long arg)
{
	struct MvpkmVM *vm = filp->private_data;
	int retval = 0;

	switch (cmd) {
	case MVPKM_DISABLE_FAULT:
		if (!vm->stubPageMPN) {
			uint32 *ptr;

			vm->stubPageMPN = AllocZeroedFreePages(vm, 0, false,
					       MEMREGION_MAINMEM, (HKVA *)&ptr);
			if (!vm->stubPageMPN)
				break;

			ptr[0] = MVPKM_STUBPAGE_BEG;
			ptr[PAGE_SIZE/sizeof(uint32) - 1] = MVPKM_STUBPAGE_END;
		}
		break;
	case MVPKM_LOCK_MPN: {
		struct MvpkmLockMPN buf;

		if (copy_from_user(&buf, (void *)arg, sizeof(buf)))
			return -EFAULT;

		buf.mpn = AllocZeroedFreePages(vm, buf.order, false,
					       buf.forRegion, NULL);
		if (buf.mpn == 0)
			return -ENOMEM;

		if (copy_to_user((void *)arg, &buf, sizeof(buf)))
			return -EFAULT;
		break;
		}
	case MVPKM_UNLOCK_MPN: {
		struct MvpkmLockMPN buf;

		if (copy_from_user(&buf, (void *)arg, sizeof(buf)))
			return -EFAULT;

		if (!LockedListDel(vm, buf.mpn))
			return -EINVAL;
		break;
		}
	case MVPKM_MAP_WSPHKVA: {
		MvpkmMapHKVA mvpkmMapInfo;
		HkvaMapInfo mapInfo[WSP_PAGE_COUNT];

		if (copy_from_user(&mvpkmMapInfo, (void *)arg,
				   sizeof(mvpkmMapInfo)))
			return -EFAULT;

		if (copy_from_user(mapInfo, (void *)mvpkmMapInfo.mapInfo,
				   sizeof(mapInfo)))
			return -EFAULT;

		mvpkmMapInfo.hkva = MapWSPHKVA(vm, mapInfo);
		BUG_ON(mvpkmMapInfo.hkva == 0);

		if (mvpkmMapInfo.forRegion == MEMREGION_WSP)
			vm->wsp = (WorldSwitchPage *) mvpkmMapInfo.hkva;

		if (copy_to_user((void *)arg, &mvpkmMapInfo,
				 sizeof(mvpkmMapInfo)))
			return -EFAULT;
		break;
		}
	case MVPKM_RUN_MONITOR:
		if (!vm->isMonitorInited)
			vm->isMonitorInited =
				((retval = SetupMonitor(vm)) == 0);

		if (vm->isMonitorInited)
			retval = RunMonitor(vm);

		break;
	case MVPKM_ABORT_MONITOR:
		if (!vm->isMonitorInited)
			return -EINVAL;

		ASSERT(vm->wsp != NULL);

		pr_err("MvpkmIoctl: Aborting monitor.\n");
		Mvpkm_WakeGuest(vm, ACTION_ABORT);
		break;
	case MVPKM_CPU_INFO: {
		struct MvpkmCpuInfo buf;
		uint32 mpidr;

#ifdef CONFIG_ARM_LPAE
		DetermineMemAttrLPAE(&buf.attribMAN);
		buf.attribL2D.u = 0;
#else
		DetermineMemAttrNonLPAE(&buf.attribL2D, &buf.attribMAN);
#endif
		ARM_MRC_CP15(MPIDR, mpidr);

		buf.mpExt = mpidr & ARM_CP15_MPIDR_MP;

		if (copy_to_user((int *)arg, &buf,
				 sizeof(struct MvpkmCpuInfo)))
			retval = -EFAULT;
		break; }
	default:
		retval = -EINVAL;
		break;
	}

	PRINTK("Returning from IOCTL(%d) retval = %d %s\n",
	       cmd, retval, signal_pending(current) ? "(pending signal)" : "");

	return retval;
}





struct LockedPage {
	struct {
		__u32 mpn:20;       
		__u32 order:6;      
		__u32 forRegion:6;  
	} page;
	struct rb_node rb;
};

static void FreeLockedPages(struct LockedPage *lp);

static struct LockedPage *
LockedListSearch(struct rb_root *root,
		 __u32 mpn)
{
	struct rb_node *n = root->rb_node;

	while (n) {
		struct LockedPage *lp = rb_entry(n, struct LockedPage, rb);

		if (lp->page.mpn == (mpn & (~0UL << lp->page.order)))
			return lp;

		if (mpn < lp->page.mpn)
			n = n->rb_left;
		else
			n = n->rb_right;
	}

	return NULL;
}


static _Bool
LockedListDel(struct MvpkmVM *vm,
	      __u32 mpn)
{
	struct LockedPage *lp;

	down_write(&vm->lockedSem);

	lp = LockedListSearch(&vm->lockedRoot, mpn);

	if (lp == NULL || lp->page.mpn != mpn) {
		up_write(&vm->lockedSem);
		return false;
	}

	FreeLockedPages(lp);

	if (lp->page.forRegion == MEMREGION_MAINMEM)
		ATOMIC_SUBV(vm->usedPages, 1U << lp->page.order);

	rb_erase(&lp->rb, &vm->lockedRoot);
	kfree(lp);

	up_write(&vm->lockedSem);

	return true;
}

static _Bool
LockedListLookup(struct MvpkmVM *vm,
		 __u32 mpn)
{
	struct LockedPage *lp;

	down_read(&vm->lockedSem);

	lp = LockedListSearch(&vm->lockedRoot, mpn);

	up_read(&vm->lockedSem);

	return lp != NULL;
}

static _Bool
LockedListAdd(struct MvpkmVM *vm,
	      __u32 mpn,
	      __u32 order,
	      PhysMem_RegionType forRegion)
{
	struct rb_node *parent, **p;
	struct LockedPage *tp, *lp = kmalloc(sizeof(*lp), GFP_KERNEL);

	if (!lp)
		return false;

	lp->page.mpn       = mpn;
	lp->page.order     = order;
	lp->page.forRegion = forRegion;

	down_write(&vm->lockedSem);

	if (forRegion == MEMREGION_MAINMEM)
		ATOMIC_ADDV(vm->usedPages, 1U << order);

	p = &vm->lockedRoot.rb_node;
	parent = NULL;

	while (*p) {
		parent = *p;
		tp = rb_entry(parent, struct LockedPage, rb);

		ASSERT(tp->page.mpn != (mpn & (~0UL << tp->page.order)));

		if (mpn < tp->page.mpn)
			p = &(*p)->rb_left;
		else
			p = &(*p)->rb_right;
	}

	rb_link_node(&lp->rb, parent, p);

	rb_insert_color(&lp->rb, &vm->lockedRoot);

	up_write(&vm->lockedSem);

	return true;
}

static void
LockedListNuke(struct rb_node *node)
{
	while (node) {
		if (node->rb_left) {
			node = node->rb_left;
		} else if (node->rb_right) {
			node = node->rb_right;
		} else {
			struct LockedPage *lp =
				rb_entry(node, struct LockedPage, rb);

			node = rb_parent(node);
			if (node) {
				if (node->rb_left)
					node->rb_left = NULL;
				else
					node->rb_right = NULL;
			}

			FreeLockedPages(lp);
			kfree(lp);
		}
	}
}

static void
LockedListUnlockAll(struct MvpkmVM *vm)
{

	down_write(&vm->lockedSem);

	LockedListNuke(vm->lockedRoot.rb_node);

	ATOMIC_SETV(vm->usedPages, 0);

	up_write(&vm->lockedSem);
}


static MPN
AllocZeroedFreePages(struct MvpkmVM *vm,
		     uint32 order,
		     _Bool highmem,
		     PhysMem_RegionType forRegion,
		     HKVA *hkvaRet)
{
	MPN mpn;
	struct page *page;

	if (order > PAGE_ALLOC_COSTLY_ORDER)
		pr_warn("Order %d allocation for region %d exceeds the safe " \
			"maximum order %d\n",
			order,
			forRegion,
			PAGE_ALLOC_COSTLY_ORDER);

	do {
		gfp_t gfp = GFP_USER | __GFP_COMP | __GFP_ZERO;
		if (highmem) {
			gfp |= __GFP_HIGHMEM;
#ifdef CONFIG_MEMORY_HOTPLUG
			gfp |= __GFP_MOVABLE;
#endif
		}
		page = alloc_pages(gfp, order);

		if (page == NULL)
			return 0;

		mpn = page_to_pfn(page);
	} while (mpn == 0 || mpn == INVALID_MPN);

	if (!LockedListAdd(vm, mpn, order, forRegion)) {
		__free_pages(page, order);
		return 0;
	}

	if (hkvaRet)
		*hkvaRet = highmem ? 0 : __phys_to_virt(page_to_phys(page));

	return mpn;
}

static HKVA
MapWSPHKVA(struct MvpkmVM *vm,
	   HkvaMapInfo *mapInfo)
{
	unsigned int i;
	struct page **pages = NULL;
	struct page **pagesPtr;
	pgprot_t prot;
	int retval;
	int allocateCount = WSP_PAGE_COUNT + 1; 
	int pageIndex = 0;
	HKVA dummyPage = (HKVA)NULL;
	HKVA start;
	HKVA startSegment;
	HKVA endSegment;

	ASSERT(allocateCount == 3);
	ASSERT_ON_COMPILE(WSP_PAGE_COUNT == 2);

	BUG_ON(vm->wspHkvaArea);

	vm->wspHkvaArea = __get_vm_area((allocateCount * PAGE_SIZE),
					VM_ALLOC, MODULES_VADDR, MODULES_END);
	if (!vm->wspHkvaArea)
		return 0;

	pages = kmalloc(allocateCount * sizeof(struct page *), GFP_TEMPORARY);
	if (!pages)
		goto err;

	pagesPtr = pages;

	dummyPage = __get_free_pages(GFP_KERNEL, 0);
	if (!dummyPage)
		goto err;

	vm->wspHKVADummyPage = dummyPage;

	for (i = 0; i < allocateCount; i++)
		pages[i] = virt_to_page(dummyPage);

	start = (HKVA)vm->wspHkvaArea->addr;
	startSegment = start & ~(ARM_L1D_SECTION_SIZE - 1);
	endSegment   = (start + PAGE_SIZE) & ~(ARM_L1D_SECTION_SIZE - 1);

	pageIndex = (startSegment != endSegment);

	for (i = pageIndex; i < pageIndex + WSP_PAGE_COUNT; i++)
		pages[i] = pfn_to_page(mapInfo[i - pageIndex].mpn);

	prot = PAGE_KERNEL_EXEC;

	retval = map_vm_area(vm->wspHkvaArea, prot, &pagesPtr);
	if (retval < 0)
		goto err;

	kfree(pages);

	return (HKVA)(vm->wspHkvaArea->addr) + pageIndex * PAGE_SIZE;

err:
	if (dummyPage) {
		free_pages(dummyPage, 0);
		vm->wspHKVADummyPage = (HKVA)NULL;
	}

	kfree(pages);

	free_vm_area(vm->wspHkvaArea);
	vm->wspHkvaArea = (struct vm_struct *)NULL;

	return 0;
}

static void
UnmapWSPHKVA(struct MvpkmVM *vm)
{
	if (vm->wspHkvaArea)
		free_vm_area(vm->wspHkvaArea);

	if (vm->wspHKVADummyPage) {
		free_pages(vm->wspHKVADummyPage, 0);
		vm->wspHKVADummyPage = (HKVA)NULL;
	}
}

static void
FreeLockedPages(struct LockedPage *lp)
{
	struct page *page;
	int count;

	page = pfn_to_page(lp->page.mpn);
	count = page_count(page);

	if (count == 0) {
		pr_err("%s: found locked page with 0 reference (mpn %05x)\n",
		       __func__, lp->page.mpn);
		return;
	}

	if (count == 1) {
		int i;

		for (i = 0; i < (1 << lp->page.order); i++)
			clear_highpage(page + i);
	} else if (lp->page.forRegion != MEMREGION_MAINMEM) {
		pr_warn("%s: mpn 0x%05x for region %d is still in use\n",
			__func__, lp->page.mpn, lp->page.forRegion);
	}

	__free_pages(page, lp->page.order);
}


static int
SetupMonitor(struct MvpkmVM *vm)
{
	int retval;
	WorldSwitchPage *wsp = vm->wsp;

#if (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__) > 40501
#define USE_ARCH_EXTENSION_SEC 1
#else
#define USE_ARCH_EXTENSION_SEC 0
#endif

	if (!wsp || wsp->wspHKVA != (HKVA)wsp)
		return -EINVAL;

	retval = Mksck_WspInitialize(vm);
	if (retval)
		return retval;

	vm->kobj.kset = mvpkmKSet;
	retval = kobject_init_and_add(&vm->kobj, &mvpkmKType,
				      NULL, "%d", wsp->guestId);
	if (retval)
		goto error;


	__module_get(THIS_MODULE);
	vm->haveKObj = true;


	vm->devicesKSet = kset_create_and_add("devices", NULL, &vm->kobj);
	if (!vm->devicesKSet) {
		retval = -ENOMEM;
		goto error;
	}

	vm->miscKSet = kset_create_and_add("misc", NULL, &vm->kobj);
	if (!vm->miscKSet) {
		kset_unregister(vm->devicesKSet);
		vm->devicesKSet = NULL;
		retval = -ENOMEM;
		goto error;
	}

	down_write(&vm->wspSem);

	if (wsp->monType == MONITOR_TYPE_VE) {
		static const uint32 normalCacheAttr2MAIR[4] = {
						0x4, 0xf, 0xa, 0xe };

		uint32 hmair0 =
			((normalCacheAttr2MAIR[wsp->memAttr.innerCache] |
			(normalCacheAttr2MAIR[wsp->memAttr.outerCache] << 4))
			<< 8 * MVA_MEMORY) |
			(0x4 << 8 * MVA_DEVICE);

		uint32 htcr =
			0x80000000 |
			(wsp->memAttr.innerCache << 8) |
			(wsp->memAttr.outerCache << 10) |
			(wsp->memAttr.share << 12);

		static const uint32 hsctlr = 0x30c5187d;

		register uint32 r0 asm("r0") = wsp->monVA.excVec;
		register uint32 r1 asm("r1") = wsp->regSave.ve.mHTTBR;
		register uint32 r2 asm("r2") = htcr;
		register uint32 r3 asm("r3") = hmair0;
		register uint32 r4 asm("r4") = hsctlr;

		asm volatile (
#if USE_ARCH_EXTENSION_SEC
			".arch_extension sec\n\t"
#endif
			"smc 0"
			:
			: "r" (r0), "r" (r1), "r" (r2), "r" (r3), "r" (r4)
			: "memory"
		);
	}

	init_waitqueue_head(&vm->wfiWaitQ);

	MonitorTimer_Setup(vm);

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&vm->wakeLock, WAKE_LOCK_SUSPEND, "mvpkm");
#endif

	wsp->mvpkmVersion = MVP_VERSION_CODE;
	up_write(&vm->wspSem);
	flush_cache_all();
	return 0;

error:
	Mksck_WspRelease(wsp);
	vm->wsp = NULL;
	return retval;
}

static
void FlushAllCpuCaches(void *info)
{
	flush_cache_all();
}

static int
RunMonitor(struct MvpkmVM *vm)
{
	int ii;
	unsigned long flags;
	WorldSwitchPage *wsp = vm->wsp;
	int retval = 0;
	unsigned int freq = -1;

	ASSERT(wsp);

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock(&vm->wakeLock);
#endif

	if (cpumask_intersects(to_cpumask(vcpuAffinity), cpu_active_mask))
		set_cpus_allowed_ptr(current, to_cpumask(vcpuAffinity));

	down_write(&vm->monThreadTaskSem);
	vm->monThreadTask = get_current();
	up_write(&vm->monThreadTaskSem);

	local_irq_save(flags);
	while (wsp->critSecCount > 0 ||
	       (!signal_pending(current) &&
		!(ATOMIC_GETO(wsp->hostActions) & ACTION_ABORT))) {

		cpumask_set_cpu(smp_processor_id(), &inMonitor);

		{
			uint32 pmnc;
			uint32 pmcnt;

			
			ARM_MRC_CP15(PERF_MON_CONTROL_REGISTER, pmnc);
			if ((pmnc & (ARM_PMNC_E | ARM_PMNC_D)) !=
			    (ARM_PMNC_E)) {
				pmnc |=  ARM_PMNC_E;  

				
				pmnc &= ~ARM_PMNC_D;
				ARM_MCR_CP15(PERF_MON_CONTROL_REGISTER, pmnc);
			}

			
			ARM_MRC_CP15(PERF_MON_COUNT_SET, pmcnt);
			if ((pmcnt & ARM_PMCNT_C) != ARM_PMCNT_C) {
				pmcnt |= ARM_PMCNT_C;
				ARM_MCR_CP15(PERF_MON_COUNT_SET, pmcnt);
			}
		}

		{
			struct TscToRate64Cb ttr;

			if (CpuFreqUpdate(&freq, &ttr)) {
				wsp->tscToRate64Mult = ttr.mult;
				wsp->tscToRate64Shift = ttr.shift;
			}
		}

		ASSERT_ON_COMPILE(MVP_TIMER_RATE64 == NSEC_PER_SEC);
		{
			ktime_t now = ktime_get();

			TSC_READ(wsp->switchedAtTSC);
			wsp->switchedAt64 = ktime_to_ns(now);
		}

		SWITCH_VFP_TO_MONITOR;

		switch (wsp->monType) {
		case MONITOR_TYPE_LPV: {
			uint32 hostVBAR;

			ARM_MRC_CP15(VECTOR_BASE, hostVBAR);
			(*wsp->switchToMonitor)(&wsp->regSave);
			ARM_MCR_CP15(VECTOR_BASE, hostVBAR);
			break;
		}
		case MONITOR_TYPE_VE: {
			register uint32 r1 asm("r1") = wsp->regSave.ve.mHTTBR;

			asm volatile (
				".word " MVP_STRINGIFY(ARM_INSTR_HVC_A1_ENC(0))
				: "=r" (r1) : "r" (r1) : "r0", "r2", "memory"
			);
			break;
		}
		default:
			FATAL();
		}

		SWITCH_VFP_TO_HOST;

		cpumask_clear_cpu(smp_processor_id(), &inMonitor);

		local_irq_restore(flags);

		for (ii = 0; ii < MKSCK_MAX_SHARES; ii++) {
			if (wsp->isPageMapped[ii])
				Mksck_WakeBlockedSockets(
					MksckPage_GetFromIdx(ii));
		}

		switch (WSP_Params(wsp)->callno) {
		case WSCALL_ACQUIRE_PAGE: {
			uint32 i;

			for (i = 0; i < WSP_Params(wsp)->pages.pages; ++i) {
				MPN mpn = AllocZeroedFreePages(vm,
					      WSP_Params(wsp)->pages.order,
					      true,
					      WSP_Params(wsp)->pages.forRegion,
					      NULL);

				if (mpn == 0) {
					pr_err("WSCALL_ACQUIRE_PAGE: no order "\
					       "%u pages available\n",
					       WSP_Params(wsp)->pages.order);
					WSP_Params(wsp)->pages.pages = i;
					break;
				}

				WSP_Params(wsp)->pages.mpns[i] = mpn;
			}
			break;
			}

		case WSCALL_RELEASE_PAGE: {
			uint32 i;

			for (i = 0; i < WSP_Params(wsp)->pages.pages; ++i) {
				if (!LockedListDel(vm,
					    WSP_Params(wsp)->pages.mpns[i])) {
					WSP_Params(wsp)->pages.pages = i;
					break;
				}
			}

			break;
			}

		case WSCALL_MUTEXLOCK:
			retval =
			    Mutex_Lock((void *)WSP_Params(wsp)->mutex.mtxHKVA,
				       WSP_Params(wsp)->mutex.mode);

			if (retval < 0) {
				WSP_Params(wsp)->mutex.ok = false;
				goto monitorExit;
			}

			wsp->critSecCount++;
			WSP_Params(wsp)->mutex.ok = true;
			break;

		case WSCALL_MUTEXUNLOCK:
			Mutex_Unlock((void *)WSP_Params(wsp)->mutex.mtxHKVA,
				     WSP_Params(wsp)->mutex.mode);
			break;

		case WSCALL_MUTEXUNLSLEEP:
			retval = Mutex_UnlSleepTest(
				    (void *)WSP_Params(wsp)->mutex.mtxHKVA,
				    WSP_Params(wsp)->mutex.mode,
				    WSP_Params(wsp)->mutex.cvi,
				    &wsp->hostActions,
				    ACTION_ABORT);
			if (retval < 0)
				goto monitorExit;
			break;
		case WSCALL_MUTEXUNLWAKE:
			Mutex_UnlWake((void *)WSP_Params(wsp)->mutex.mtxHKVA,
				      WSP_Params(wsp)->mutex.mode,
				      WSP_Params(wsp)->mutex.cvi,
				      WSP_Params(wsp)->mutex.all);
			break;

		case WSCALL_WAIT:
#ifdef CONFIG_HAS_WAKELOCK
			if (WSP_Params(wsp)->wait.suspendMode) {
				wake_unlock(&vm->wakeLock);
				retval = MvpkmWaitForInt(vm, true);
				wake_lock(&vm->wakeLock);
				WSP_Params(wsp)->wait.suspendMode = 0;
			} else {
				retval = MvpkmWaitForInt(vm, false);
			}
#else
			retval =
			    MvpkmWaitForInt(vm,
					    WSP_Params(wsp)->wait.suspendMode);
#endif
			if (retval < 0)
				goto monitorExit;

			break;

		case WSCALL_IRQ:
			break;

		case WSCALL_GET_PAGE_FROM_VMID: {
			MksckPage *mksckPage;

			mksckPage = MksckPage_GetFromVmIdIncRefc(
					WSP_Params(wsp)->pageMgmnt.vmId);
			if (mksckPage) {
				int ii;
				int pageIndex;

				WSP_Params(wsp)->pageMgmnt.found = true;
				for (ii = 0; ii < MKSCKPAGE_TOTAL; ii++) {
					WSP_Params(wsp)->pageMgmnt.mpn[ii] =
				    vmalloc_to_pfn((void *)(((HKVA)mksckPage) +
						   ii * PAGE_SIZE));
				}

				pageIndex = MKSCK_VMID2IDX(mksckPage->vmId);
				ASSERT(!wsp->isPageMapped[pageIndex]);
				wsp->isPageMapped[pageIndex] = true;
			} else {
				WSP_Params(wsp)->pageMgmnt.found = false;
			}
			break;
			}

		case WSCALL_REMOVE_PAGE_FROM_VMID: {
			MksckPage *mksckPage;
			int pageIndex;

			mksckPage =
			 MksckPage_GetFromVmId(WSP_Params(wsp)->pageMgmnt.vmId);
			pageIndex = MKSCK_VMID2IDX(mksckPage->vmId);

			ASSERT(wsp->isPageMapped[pageIndex]);
			wsp->isPageMapped[pageIndex] = false;
			MksckPage_DecRefc(mksckPage);
			break;
			}

		case WSCALL_READTOD: {
			struct timeval nowTV;

			do_gettimeofday(&nowTV);
			WSP_Params(wsp)->tod.now = nowTV.tv_sec;
			WSP_Params(wsp)->tod.nowusec = nowTV.tv_usec;
			break;
			}

		case WSCALL_LOG: {
			int len = strlen(WSP_Params(wsp)->log.messg);

			pr_info("VMM: %s%s",
				WSP_Params(wsp)->log.messg,
				(WSP_Params(wsp)->log.messg[len-1] == '\n') ?
					"" : "\n");
			break;
			}

		case WSCALL_ABORT:
			retval = WSP_Params(wsp)->abort.status;
			goto monitorExit;

		case WSCALL_QP_GUEST_ATTACH: {
			int32 rc;
			QPInitArgs args;
			uint32 base;
			uint32 nrPages;

			args.id       = WSP_Params(wsp)->qp.id;
			args.capacity = WSP_Params(wsp)->qp.capacity;
			args.type     = WSP_Params(wsp)->qp.type;
			base          = WSP_Params(wsp)->qp.base;
			nrPages       = WSP_Params(wsp)->qp.nrPages;

			rc = QP_GuestAttachRequest(vm, &args, base, nrPages);

			WSP_Params(wsp)->qp.rc           = rc;
			WSP_Params(wsp)->qp.id           = args.id;
			break;
			}

		case WSCALL_QP_NOTIFY: {
			QPInitArgs args;

			args.id       = WSP_Params(wsp)->qp.id;
			args.capacity = WSP_Params(wsp)->qp.capacity;
			args.type     = WSP_Params(wsp)->qp.type;

			WSP_Params(wsp)->qp.rc = QP_NotifyListener(&args);
			break;
			}

		case WSCALL_MONITOR_TIMER:
			MonitorTimer_Request(&vm->monTimer,
					     WSP_Params(wsp)->timer.when64);
			break;

		case WSCALL_COMM_SIGNAL:
			Mvpkm_CommEvSignal(&WSP_Params(wsp)->commEvent.transpID,
					   WSP_Params(wsp)->commEvent.event);
			break;

		case WSCALL_FLUSH_ALL_DCACHES:
			on_each_cpu(FlushAllCpuCaches, NULL, 1);
			break;

		default:
			retval = -EPIPE;
			goto monitorExit;
		}

		if (need_resched())
			schedule();

		if (cpumask_intersects(to_cpumask(vcpuAffinity),
				       cpu_active_mask) &&
		    !cpumask_equal(to_cpumask(vcpuAffinity),
				   &current->cpus_allowed))
			set_cpus_allowed_ptr(current, to_cpumask(vcpuAffinity));

		local_irq_save(flags);
	}

	local_irq_restore(flags);

monitorExit:
	ASSERT(wsp->critSecCount == 0);

	if (ATOMIC_GETO(wsp->hostActions) & ACTION_ABORT) {
		PRINTK("Monitor has ABORT flag set.\n");
		retval = ExitStatusHostRequest;
	}

	if (retval == ExitStatusHostRequest && vm->watchdogTriggered)
		retval = ExitStatusVMMFatalKnown;

#ifdef CONFIG_HAS_WAKELOCK
	wake_unlock(&vm->wakeLock);
#endif

	down_write(&vm->monThreadTaskSem);
	vm->monThreadTask = NULL;
	up_write(&vm->monThreadTaskSem);

	return retval;
}

int
MvpkmWaitForInt(struct MvpkmVM *vm,
		_Bool suspend)
{
	WorldSwitchPage *wsp = vm->wsp;
	wait_queue_head_t *q = &vm->wfiWaitQ;

	if (suspend) {
		return wait_event_interruptible(*q,
				ATOMIC_GETO(wsp->hostActions) != 0);
	} else {
		int ret;

		ret = wait_event_interruptible_timeout(*q,
				ATOMIC_GETO(wsp->hostActions) != 0, 10*HZ);
		if (ret == 0)
			pr_warn("MvpkmWaitForInt: guest stuck for 10s in " \
				"WFI! (hostActions %08x)\n",
				ATOMIC_GETO(wsp->hostActions));

		return ret > 0 ? 0 : ret;
	}
}


void
Mvpkm_WakeGuest(struct MvpkmVM *vm,
		int why)
{
	ASSERT(why != 0);

	
	if (ATOMIC_ORO(vm->wsp->hostActions, why) & why)
		
		return;

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock(&vm->wakeLock);
#endif

	if (down_read_trylock(&vm->monThreadTaskSem)) {
		if (vm->monThreadTask) {
			wake_up_process(vm->monThreadTask);
			kick_process(vm->monThreadTask);
		}
		up_read(&vm->monThreadTaskSem);
	} else {
		pr_warn("Unexpected failure to acquire monThreadTaskSem!\n");
	}
}
